#include "sim/parts/ds18b20.hpp"
#include "sim/parts/i2c_slave.hpp"
#include "sim/parts/lua_part.hpp"
#include "sim/part.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>

namespace sim {

namespace {

// NTC thermistor in a divider with a fixed resistor. By default the fixed
// resistor goes to the supply and the NTC to ground, so the voltage rises
// with temperature. The output is an analog source on the net.
class Ntc final : public Part {
public:
    Ntc(PartContext &ctx)
        : Part(ctx.name()), scheduler_(ctx.scheduler()), pin_(ctx.pin("out")) {
        ParamReader &p = ctx.params();
        r25_ = p.resistance("r25", 10000.0);
        beta_ = p.number("beta", 3950.0);
        r_series_ = p.resistance("r_series", 10000.0);
        temp_c_ = p.number("temp_c", 25.0);
        supply_mv_ = p.number("supply_mv", 3300.0);
        const std::string topology = p.string("series").value_or("pullup");
        if (topology != "pullup" && topology != "pulldown")
            throw std::invalid_argument("param 'series' must be pullup or pulldown");
        ntc_to_ground_ = topology == "pullup";
        p.finish();
        if (!(beta_ > 0) || !(supply_mv_ > 0))
            throw std::invalid_argument("beta and supply_mv must be positive");
        update();
    }

    std::string type() const override { return "ntc"; }

    void register_endpoints(EndpointRegistry &registry,
                            const std::function<std::uint64_t()> &now) override {
        registry.add(name() + ".temp_c",
                     std::make_unique<ParamEndpoint>(
                         name() + ".temp_c", [this] { return Value{temp_c_}; },
                         [this](const Value &v) {
                             temp_c_ = value_as_number(v);
                             update();
                         },
                         now));
        registry.add(name() + ".mv",
                     std::make_unique<ParamEndpoint>(
                         name() + ".mv", [this] { return Value{millivolts()}; },
                         [](const Value &) {
                             throw std::logic_error("mv is derived from temp_c");
                         },
                         now));
    }

    double resistance_ohm() const {
        const double kelvin = temp_c_ + 273.15;
        return r25_ * std::exp(beta_ * (1.0 / kelvin - 1.0 / 298.15));
    }
    double millivolts() const {
        const double r_ntc = resistance_ohm();
        const double ratio = ntc_to_ground_ ? r_ntc / (r_ntc + r_series_)
                                            : r_series_ / (r_ntc + r_series_);
        return supply_mv_ * ratio;
    }

private:
    void update() {
        if (temp_c_ <= -273.0)
            throw std::invalid_argument("temp_c is below absolute zero");
        pin_.set_analog_mv(static_cast<std::uint32_t>(std::lround(millivolts())),
                           scheduler_.now());
    }

    Scheduler &scheduler_;
    Pin &pin_;
    double r25_{}, beta_{}, r_series_{}, temp_c_{}, supply_mv_{};
    bool ntc_to_ground_{true};
};

// Generic register-file I2C device: the first written byte selects a register,
// later writes store into it and reads return it; the pointer auto-increments.
class I2cRegs final : public Part {
public:
    I2cRegs(PartContext &ctx)
        : Part(ctx.name()), sda_(ctx.pin("sda")), scl_(ctx.pin("scl")) {
        ParamReader &p = ctx.params();
        const std::int64_t address = p.integer("address", -1);
        if (address < 0 || address > 0x7f)
            throw std::invalid_argument("param 'address' (7-bit, e.g. 0x44) is required");
        address_ = static_cast<std::uint8_t>(address);
        const std::int64_t size = p.integer("size", 256);
        if (size < 1 || size > 256)
            throw std::invalid_argument("param 'size' must be 1..256");
        regs_.assign(static_cast<std::size_t>(size), 0);
        // init: "00=1a,01=ff" (hex register=value pairs)
        if (const auto init = p.string("init")) {
            std::size_t pos = 0;
            while (pos < init->size()) {
                const std::size_t comma = init->find(',', pos);
                const std::string item =
                    init->substr(pos, comma == std::string::npos ? comma : comma - pos);
                const std::size_t eq = item.find('=');
                std::size_t used_reg = 0, used_val = 0;
                unsigned long reg = 0, val = 0;
                try {
                    if (eq == std::string::npos)
                        throw std::invalid_argument("");
                    reg = std::stoul(item.substr(0, eq), &used_reg, 16);
                    val = std::stoul(item.substr(eq + 1), &used_val, 16);
                } catch (const std::exception &) {
                    throw std::invalid_argument("param 'init' must be hex pairs like 00=1a,01=ff");
                }
                if (used_reg != eq || used_val != item.size() - eq - 1 ||
                    reg >= regs_.size() || val > 0xff)
                    throw std::invalid_argument("param 'init' has an invalid entry '" + item + "'");
                regs_[reg] = static_cast<std::uint8_t>(val);
                if (comma == std::string::npos)
                    break;
                pos = comma + 1;
            }
        }
        p.finish();
        I2cSlave::Handlers handlers;
        handlers.on_address = [this](std::uint8_t a, bool) { return a == address_; };
        handlers.on_start = [this] { pointer_set_ = false; };
        handlers.on_write = [this](std::uint8_t byte) {
            if (!pointer_set_) {
                pointer_ = byte;
                pointer_set_ = true;
                return pointer_ < regs_.size();
            }
            if (pointer_ >= regs_.size())
                return false;
            regs_[pointer_++] = byte;
            return true;
        };
        handlers.on_read = [this] {
            return pointer_ < regs_.size() ? regs_[pointer_++] : std::uint8_t{0xff};
        };
        slave_ = std::make_unique<I2cSlave>(sda_, scl_, std::move(handlers));
    }

    std::string type() const override { return "i2c_regs"; }
    const std::vector<std::uint8_t> &registers() const { return regs_; }

    void register_endpoints(EndpointRegistry &registry,
                            const std::function<std::uint64_t()> &now) override {
        // Registers are readable/writable from outside as "<name>.reg<hex>".
        for (std::size_t i = 0; i < regs_.size(); ++i) {
            char suffix[8];
            std::snprintf(suffix, sizeof suffix, ".reg%02zx", i);
            registry.add(name() + suffix,
                         std::make_unique<ParamEndpoint>(
                             name() + suffix,
                             [this, i] { return Value{static_cast<std::int64_t>(regs_[i])}; },
                             [this, i](const Value &v) {
                                 const double n = value_as_number(v);
                                 if (n < 0 || n > 255 || n != std::floor(n))
                                     throw std::invalid_argument("register value must be 0..255");
                                 regs_[i] = static_cast<std::uint8_t>(n);
                             },
                             now));
        }
    }

private:
    Pin &sda_;
    Pin &scl_;
    std::uint8_t address_{};
    std::vector<std::uint8_t> regs_;
    std::uint8_t pointer_{0};
    bool pointer_set_{false};
    std::unique_ptr<I2cSlave> slave_;
};

} // namespace

const PartRegistry &PartRegistry::builtin() {
    static const PartRegistry registry = [] {
        PartRegistry r;
        r.add("ds18b20", [](PartContext &ctx) -> std::unique_ptr<Part> {
            ParamReader &p = ctx.params();
            Pin &dq = ctx.pin("dq");
            const auto rom_text = p.string("rom");
            const auto rom = rom_text ? Ds18b20::parse_rom(*rom_text)
                                      : Ds18b20::default_rom(ctx.name());
            const double temperature = p.number("temperature_c", 25.0);
            const std::int64_t resolution = p.integer("resolution", 12);
            p.finish();
            if (resolution < 9 || resolution > 12)
                throw std::invalid_argument("param 'resolution' must be 9..12");
            return std::make_unique<Ds18b20>(ctx.name(), ctx.scheduler(), dq, rom,
                                             temperature,
                                             static_cast<unsigned>(resolution));
        });
        r.add("ntc", [](PartContext &ctx) -> std::unique_ptr<Part> {
            return std::make_unique<Ntc>(ctx);
        });
        r.add("script", [](PartContext &ctx) -> std::unique_ptr<Part> {
            return make_lua_part(ctx);
        });
        r.add("i2c_regs", [](PartContext &ctx) -> std::unique_ptr<Part> {
            return std::make_unique<I2cRegs>(ctx);
        });
        return r;
    }();
    return registry;
}

} // namespace sim

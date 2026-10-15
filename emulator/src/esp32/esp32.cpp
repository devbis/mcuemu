#include "esp32emu/esp32.hpp"

#include <unicorn/unicorn.h>

#include "aes.hpp"
#include "blocks.hpp"
#include "gpio.hpp"
#include "intc.hpp"
#include "ieee802154.hpp"
#include "rom.hpp"
#include "spi_mem.hpp"
#include "timers.hpp"
#include "timg.hpp"
#include "esp32emu/esp32c6_regs.hpp"
#include "esp32emu/esp32c6_rom.hpp"
#include "esp32emu/esp32h2_regs.hpp"
#include "esp32emu/esp32h2_rom.hpp"

#include <algorithm>
#include <deque>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace esp32emu {

namespace {


constexpr std::uint32_t Ebreak = 0x00100073;

} // namespace

// UART0: the transmit FIFO (bytes go out through Esp32::uart_tx, instantly) and a receive FIFO fed by
// receive(). Interrupts (one line, `source`): RXFIFO_FULL while the FIFO holds at least
// CONF1.RXFIFO_FULL_THRHD bytes, RXFIFO_TOUT while it holds any (the model has no idle-timeout timing), and the
// transmit ones (TXFIFO_EMPTY, TX_DONE) after each byte written; the others never fire. INT_ST is
// INT_RAW & INT_ENA; INT_CLR drops the latched bits and the receive conditions return while they hold.
// The register layout is identical on the C6 and the H2.
class Uart0 final : public Block {
public:
    static constexpr std::size_t RxFifoSize = 128;
    Uart0(Esp32 &chip, unsigned source, IrqSink irq)
        : Block(chip.layout().uart0, 0x1000), chip_(chip), source_(source), irq_(std::move(irq)) {}

    bool can_receive() const { return rx_.size() < RxFifoSize; }
    bool receive(std::uint8_t byte) {
        if (!can_receive())
            return false;
        rx_.push_back(byte);
        update_irq();
        return true;
    }
    void reset() override {
        scratch_.clear();
        rx_.clear();
        latched_ = 0;
        update_irq();
    }

    std::uint32_t read(std::uint32_t offset, unsigned) override {
        namespace u = esp32c6_regs::uart0;
        switch (offset & ~3u) {
        case u::reg::FIFO: {
            if (rx_.empty())
                return 0;
            const std::uint8_t byte = rx_.front();
            rx_.pop_front();
            update_irq();
            return byte;
        }
        case u::reg::STATUS: return static_cast<std::uint32_t>(rx_.size()) << u::field::STATUS_RXFIFO_CNT_shift;
        case u::reg::INT_RAW: return raw();
        case u::reg::INT_ST: return raw() & stored(u::reg::INT_ENA);
        // REG_UPDATE.UPDATE self-clears once the new configuration has been
        // synchronised to the UART core clock domain (the driver polls for it).
        case u::reg::REG_UPDATE: return stored(u::reg::REG_UPDATE) & ~(1u << u::field::REG_UPDATE_REG_UPDATE_shift);
        default: return stored(offset & ~3u);
        }
    }
    void write(std::uint32_t offset, std::uint32_t value, unsigned) override {
        namespace u = esp32c6_regs::uart0;
        switch (offset & ~3u) {
        case u::reg::FIFO:
            chip_.uart_tx(static_cast<std::uint8_t>(value));
            latched_ |= TxBits;
            update_irq();
            return;
        case u::reg::INT_CLR:
            latched_ &= ~value;
            update_irq();
            return;
        case u::reg::INT_RAW: // write one to set (software trigger)
            latched_ |= value;
            update_irq();
            return;
        default: scratch_[offset & ~3u] = value;
        }
        if ((offset & ~3u) == u::reg::INT_ENA || (offset & ~3u) == u::reg::CONF1)
            update_irq();
    }

private:
    static constexpr std::uint32_t TxBits = 1u << 1 | 1u << 14; // TXFIFO_EMPTY, TX_DONE
    std::uint32_t stored(std::uint32_t offset) const {
        auto it = scratch_.find(offset);
        return it == scratch_.end() ? 0 : it->second;
    }
    std::uint32_t raw() const {
        namespace u = esp32c6_regs::uart0;
        std::uint32_t bits = latched_;
        const std::uint32_t threshold = stored(u::reg::CONF1) >> u::field::CONF1_RXFIFO_FULL_THRHD_shift & 0xff;
        if (!rx_.empty()) {
            bits |= 1u << 8; // RXFIFO_TOUT
            if (rx_.size() >= std::max<std::uint32_t>(threshold, 1))
                bits |= 1u << 0; // RXFIFO_FULL
        }
        return bits;
    }
    void update_irq() { irq_(source_, (raw() & stored(esp32c6_regs::uart0::reg::INT_ENA)) != 0); }

    Esp32 &chip_;
    unsigned source_;
    IrqSink irq_;
    std::deque<std::uint8_t> rx_;
    std::uint32_t latched_{0};
    std::map<std::uint32_t, std::uint32_t> scratch_;
};

namespace {
template <typename Symbols> std::vector<RomSymbol> symbols_of(const Symbols &table) {
    std::vector<RomSymbol> out;
    for (const auto &symbol : table)
        out.push_back({symbol.address, symbol.name});
    return out;
}
} // namespace

const Layout &layout_of(Variant variant) {
    static const Layout c6 = {"ESP32-C6", 13, 0x50000, 0x80000, 0x4000, 160000000, 40000000,
                              esp32c6_regs::uart0::base, esp32c6_regs::systimer::base, esp32c6_regs::timg0::base,
                              esp32c6_regs::timg1::base, esp32c6_regs::spi0::base, esp32c6_regs::spi1::base,
                              esp32c6_regs::ieee802154::base, esp32c6_regs::lp_timer::base,
                              esp32c6_regs::interrupt_core0::base,
                              12, esp32c6_regs::systimer::irq::SYSTIMER_TARGET0, esp32c6_regs::gpio::base, esp32c6_regs::io_mux::base, 31,
                              esp32c6_regs::gpio::irq::GPIO, esp32c6_regs::uart0::irq::UART0, 0x134, 0x600af800, true,
                              symbols_of(esp32c6_rom::symbols)};
    static const Layout h2 = {"ESP32-H2", 16, 0x20000, 0x50000, 0x1000, 96000000, 32000000,
                              esp32h2_regs::uart0::base, esp32h2_regs::systimer::base, esp32h2_regs::timg0::base,
                              esp32h2_regs::timg1::base, esp32h2_regs::spi0::base, esp32h2_regs::spi1::base,
                              0x600A3000 /* IEEE802154_REG_BASE in ESP-IDF's soc/esp32h2/register/soc/reg_base.h; the esp-pacs SVD says 0x60047000 */,
                              esp32h2_regs::lp_timer::base,
                              esp32h2_regs::interrupt_core0::base,
                              21, esp32h2_regs::systimer::irq::SYSTIMER_TARGET0, esp32h2_regs::gpio::base, esp32h2_regs::io_mux::base, 28,
                              esp32h2_regs::gpio::irq::GPIO, esp32h2_regs::uart0::irq::UART0, 0x104, 0x600ad800, false,
                              symbols_of(esp32h2_rom::symbols)};
    return variant == Variant::C6 ? c6 : h2;
}

// PCR (clock and reset control): registers read back what was written, and the H2's
// BUS_CLK_UPDATE.BUS_CLOCK_UPDATE bit is the hardware's "apply the clock
// configuration" strobe: the bootloader writes it and polls until it clears.
class Pcr final : public StatusBlock {
public:
    explicit Pcr(std::uint32_t base) : StatusBlock(base, 0x1000) {}
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override {
        if ((offset & ~3u) == esp32h2_regs::pcr::reg::BUS_CLK_UPDATE)
            value &= ~1u;
        StatusBlock::write(offset, value, size);
    }
};

// The modem block's analog I2C master (regi2c). A transaction is one write of
// a control word at +0x0 or +0x4 (one per host): bits 7:0 block, 15:8 register, 23:16 data, bit 24 = write
// (ESP-IDF regi2c_ctrl_ll.h). A read leaves the register's value in bits 23:16 of
// the same word, which the driver polls (bit 25 = busy, never set: transactions are
// instant). The registers are plain storage. ANA_CONF0 (+0x18) bit 24 is the
// "calibration done" flag the bootloader waits for after configuring the BBPLL.
class AnalogMaster final : public StatusBlock {
public:
    explicit AnalogMaster(std::uint32_t base)
        : StatusBlock(base, 0x400, std::vector<StatusBlock::Forced>{{0x18, 1u << 24, 0}}) {
    }
    void reset() override {
        StatusBlock::reset(); // the analog registers keep their values across a CPU reset
    }
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override {
        if (((offset & ~3u) == 0x0 || (offset & ~3u) == 0x4) && size == 4) { // the control words of the two hosts
            const std::uint32_t key = value & 0xffff;
            if (value & (1u << 24)) {
                cells_[key] = static_cast<std::uint8_t>(value >> 16);
            } else {
                auto it = cells_.find(key);
                value = (value & ~0x00ff0000u) | (static_cast<std::uint32_t>(it == cells_.end() ? 0 : it->second) << 16);
            }
        }
        StatusBlock::write(offset, value, size);
    }

private:
    std::map<std::uint32_t, std::uint8_t> cells_;
};

Esp32::Esp32(const Config &config)
    : config_(config), layout_(layout_of(config.variant)), core_(riscv::Core::Config{layout_.clock_hz}),
      flash_(config.flash_size, 0xff) {
    const std::uint32_t rom_size = layout_.rom_size;
    std::uint8_t *rom = core_.map_memory(RomBase, rom_size, false);
    for (std::uint32_t offset = 0; offset < rom_size; offset += 4)
        std::memcpy(rom + offset, &Ebreak, 4);
    auto word = [&](std::uint32_t address, std::uint32_t value) {
        std::memcpy(rom + (address - RomBase), &value, 4);
    };
    word(0x40000010, layout_.chip_id); // _rom_chip_id
    word(0x40000014, 0);  // _rom_eco_version
    core_.map_memory(SramBase, layout_.sram_size, true);
    core_.map_memory(LpSramBase, layout_.lp_sram_size, true);
    cache_ = core_.map_memory(CacheBase, CacheSize, false);
    // Espressif's performance counter CSRs: MPCER (event select), MPCMR (mode,
    // bit 0 enables counting) and MPCCR, the cycle counter that esp_cpu_get_cycle_count()
    // and the clock configuration code read. The counter counts CPU cycles while
    // enabled; writing MPCCR sets it.
    // The range also covers the debug trigger CSRs (tselect, tdata1.., tcontrol:
    // 0x7a0..0x7a8; esp-hal arms a trigger as a stack guard) and the physical memory
    // attribute CSRs PMACFG/PMAADDR (0xbc0..0xbdf, which the H2's bootloader writes
    // to mark memory regions cacheable). They are plain storage here: no trigger
    // ever fires. Anything else in the range is left to the core.
    core_.set_csr_handler(0x7a0, 0xbdf, [this](unsigned csr, bool write, std::uint32_t &value) {
        if (csr >= 0xbc0 || (csr >= 0x7a0 && csr <= 0x7a8)) {
            std::uint32_t &reg = pma_[csr];
            if (write)
                reg = value;
            else
                value = reg;
            return true;
        }
        if (csr < 0x7e0 || csr > 0x7e2)
            return false;
        std::uint32_t &mode = pc_regs_[1];
        const std::uint64_t now = core_.cycles();
        if (csr == 0x7e2) {
            if (write)
                pc_count_ = value - (mode & 1u ? now : 0);
            else
                value = static_cast<std::uint32_t>(pc_count_ + (mode & 1u ? now : 0));
            return true;
        }
        std::uint32_t &reg = pc_regs_[csr - 0x7e0];
        if (write) {
            if (csr == 0x7e1) { // freeze or resume without losing the count
                const std::uint32_t before = static_cast<std::uint32_t>(pc_count_ + (mode & 1u ? now : 0));
                reg = value;
                pc_count_ = before - (mode & 1u ? now : 0);
            } else {
                reg = value;
            }
        } else {
            value = reg;
        }
        return true;
    });
    // Time-driven peripherals: one pass over the blocks, after every register
    // write and when the hart is due; it returns the next cycle anything happens.
    core_.set_service_handler([this](std::uint64_t now) {
        std::uint64_t next = NoEvent;
        for (auto &block : blocks_)
            next = std::min(next, block->service(now));
        return next;
    });
    core_.map_mmio(PeriphBase, PeriphSize, *this);
    core_.map_mmio(IntcBase, IntcSize, *this);

    auto add = [&](std::unique_ptr<Block> block) {
        by_base_[block->base()] = block.get();
        blocks_.push_back(std::move(block));
    };
    rom_ = std::make_unique<Rom>(*this);
    {
        auto uart = std::make_unique<Uart0>(*this, layout_.uart0_source, [this](unsigned source, bool level) {
            if (intc_)
                intc_->set_source(source, level);
        });
        uart_ = uart.get();
        add(std::move(uart));
    }
    intc_ = std::make_unique<Intc>([this](int cause) {
        if (trace_irq) {
            char buffer[80];
            std::snprintf(buffer, sizeof buffer, "[irq] line %d at %llu ns (pc 0x%08x)", cause,
                          static_cast<unsigned long long>(time_ns()), core_.pc());
            log(buffer);
        }
        core_.set_interrupt(cause);
    });
    intc_->set_status_offset(layout_.int_status);
    add(std::make_unique<IntMatrixBlock>(layout_.int_matrix, *intc_));
    add(std::make_unique<PlicBlock>(0x20001000, *intc_));
    add(std::make_unique<Systimer>(layout_.systimer, layout_.clock_hz, layout_.systimer_source,
                                   [this] { return cycles(); },
                                   [this](unsigned source, bool level) { intc_->set_source(source, level); }));
    add(std::make_unique<LpTimer>(layout_.lp_timer, layout_.clock_hz, [this] { return cycles(); }));
    if (layout_.phy_rom) {
        // A free-running microsecond counter the PHY blob times its polling loops with
        // (wait_i2c_sdm_stable reads 0x600ad000 and gives up 10000 counts later);
        // identity and base inferred from that code, not documented.
        add(std::make_unique<MicrosecondCounter>(0x600ad000, layout_.clock_hz, [this] { return cycles(); }));
    }
    // PHY digital registers (baseband/frontend, 0x600a0000..): the blob starts a
    // calibration by writing a start bit and polls a done bit. Each forced bit is
    // one such poll seen in libphy: on the C6 txdc_cal_new waits for bit 22 of
    // +0x418 and ram_iq_est_enable for bit 16 of +0x4a0; on the H2 the init loop
    // waits for the three-bit state in bits 16:14 of +0x814 to reach 7. There is no RF
    // model behind them.
    std::vector<StatusBlock::Forced> phy_forced;
    if (layout_.phy_rom)
        phy_forced = {{0x418, 1u << 22, 0}, {0x4a0, 1u << 16, 0}};
    else
        phy_forced = {{0x814, 7u << 14, 0}, {0x4a0, 1u << 16, 0}};
    add(std::make_unique<StatusBlock>(0x600a0000, 0x8000, phy_forced));
    {
        auto mac = std::make_unique<Ieee802154>(
            layout_.ieee802154, layout_.clock_hz, layout_.zb_mac_source, [this] { return cycles(); },
            [this](unsigned source, bool level) { intc_->set_source(source, level); },
            [this](std::uint32_t a, void *out, std::size_t n) { return core_.read_memory(a, out, n); },
            [this](std::uint32_t a, const void *data, std::size_t n) { return core_.write_memory(a, data, n); },
            [this](std::uint64_t cycle) { core_.request_service(cycle); });
        mac_ = mac.get();
        add(std::move(mac));
    }
    add(std::make_unique<Timg>(layout_.timg0, layout_.xtal_hz));
    add(std::make_unique<Timg>(layout_.timg1, layout_.xtal_hz));
    auto spi0 = std::make_unique<SpiMem0>(layout_.spi0, flash_, cache_, [this](unsigned page) {
        invalidate_code();
        if (config_.stub_phy)
            stub_phy_in(CacheBase + page * SpiMem0::PageSize, cache_ + page * SpiMem0::PageSize,
                        SpiMem0::PageSize);
    });
    spi0_ = spi0.get();
    add(std::move(spi0));
    add(std::make_unique<SpiMem1>(layout_.spi1, config.flash_size));
    {
        auto mux = std::make_unique<IoMux>(layout_.io_mux, layout_.gpio_pins);
        auto gpio = std::make_unique<Gpio>(layout_.gpio, layout_.gpio_pins, layout_.gpio_source, *mux,
                                           [this](unsigned source, bool level) { intc_->set_source(source, level); });
        gpio_ = gpio.get();
        add(std::move(mux));
        add(std::move(gpio));
    }
    add(std::make_unique<Aes>(0x60088000)); // the same base on the C6 and the H2
    if (config.variant == Variant::H2) {
        add(std::make_unique<Pcr>(esp32h2_regs::pcr::base));
        // eFuse: the factory MAC (RD_MAC_SYS_0/1, little-endian: byte 0 first). Everything else reads as
        // blank (0). The address is made up (an Espressif OUI), the same (H2 only for now; the C6's register names differ).
        auto efuse = std::make_unique<StatusBlock>(esp32h2_regs::efuse::base, 0x200);
        efuse->write(esp32h2_regs::efuse::reg::RD_MAC_SYS_0, 0xf911'2233u, 4);
        efuse->write(esp32h2_regs::efuse::reg::RD_MAC_SYS_1, 0x0000'6055u, 4);
        efuse->keep_as_power_on_state();
        add(std::move(efuse));
    } else {
        // C6 eFuse. Field positions are from ESP-IDF's esp_efuse_table.csv: BLOCK1 holds the MAC (words 0/1),
        // the wafer revision and BLK_VERSION (word 3), BLOCK2 the ADC calibration (RD_SYS_PART1_DATA4..7:
        // TEMP_CALIB bits 8:0, OCODE bits 16:9). The calibration words are read from a real ESP32-C6FH4
        // (rev v0.2); the MAC is made up and the unique ID (BLOCK2 words 0..3) stays blank.
        // BLK_VERSION >= 0.1 makes ocode_init() take OCODE from here instead of waiting for an analog
        // calibration that never completes.
        namespace ef = esp32c6_regs::efuse::reg;
        auto efuse = std::make_unique<StatusBlock>(esp32c6_regs::efuse::base, 0x200);
        efuse->write(ef::RD_MAC_SPI_SYS_0, 0xf912'2233u, 4);
        efuse->write(ef::RD_MAC_SPI_SYS_1, 0x0000'6055u, 4);
        efuse->write(ef::RD_MAC_SPI_SYS_3, 0x1908'0000u, 4);
        efuse->write(ef::RD_MAC_SPI_SYS_4, 0x0000'0029u, 4);
        efuse->write(ef::RD_SYS_PART1_DATA4, 0x8c56'8d9au, 4);
        efuse->write(ef::RD_SYS_PART1_DATA5, 0x271c'4651u, 4);
        efuse->write(ef::RD_SYS_PART1_DATA6, 0x0980'800cu, 4);
        efuse->write(ef::RD_SYS_PART1_DATA7, 0x1111'1111u, 4);
        efuse->keep_as_power_on_state();
        add(std::move(efuse));
    }

    // Analog I2C master of the modem block: bootloader_clock_configure() waits
    // for ANA_CONF0 bit 24 (BBPLL calibration done) after configuring the PLL
    // through regi2c; calibration is instant here.
    add(std::make_unique<AnalogMaster>(layout_.modem_analog));
}

Esp32::~Esp32() = default;

bool Esp32::uart_rx(std::uint8_t byte) { return uart_->receive(byte); }
bool Esp32::uart_can_receive() const { return uart_->can_receive(); }
unsigned Esp32::gpio_pins() const { return layout_.gpio_pins; }
void Esp32::set_pad_input(unsigned pad, bool high) { gpio_->set_input(pad, high); }
PadDrive Esp32::pad_drive(unsigned pad) const { return gpio_->drive(pad); }
void Esp32::set_pad_observer(std::function<void(unsigned pad)> observer) { gpio_->on_drive_changed = std::move(observer); }
void Esp32::raise_source(unsigned source, bool level) { intc_->set_source(source, level); }

void Esp32::flash_modified() { spi0_->refresh(); }

namespace {

// A function of the closed PHY blob (libphy), found by its first instructions.
// `mask` bytes of 0 are wildcards: addresses that differ between builds.
struct Signature {
    const char *name;
    std::vector<std::uint8_t> bytes;
    std::vector<std::uint8_t> mask;
    std::uint32_t result; // value the stub returns
};

const std::vector<Signature> &phy_signatures() {
    static const std::vector<Signature> table = {
        // register_chipv7_phy: sp -= 0x110, then saves and a memset of the 0x80-byte
        // local. Calls the calibration that needs the real RF front end.
        {"register_chipv7_phy",
         {0x6d, 0x71, 0x23, 0x20, 0x21, 0x11, 0xd2, 0xdd, 0xd6, 0xdb, 0x2a, 0x8a, 0x2e, 0x89, 0xb2, 0x8a,
          0x81, 0x45, 0x13, 0x06, 0x00, 0x08, 0x88, 0x18, 0x23, 0x26, 0x11, 0x10, 0x23, 0x24, 0x81, 0x10},
         std::vector<std::uint8_t>(32, 0xff),
         0},
        // phy_track_pll_internal: lui a5,<hi>; lhu a5,<lo>(a5); andi a5,a5,6; beqz; then the
        // frame setup. Reads a RAM flag and the timer; its callee overruns the stack.
        {"phy_track_pll_internal",
         {0xb7, 0x97, 0, 0, 0x83, 0xd7, 0, 0, 0x99, 0x8b, 0x8d, 0xc7, 0x41, 0x11, 0x06, 0xc6, 0x22, 0xc4, 0x26,
          0xc2},
         {0xff, 0xff, 0, 0, 0xff, 0xff, 0, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
          0xff},
         0},
    };
    return table;
}

} // namespace

void Esp32::stub_phy_in(std::uint32_t virtual_base, const std::uint8_t *data, std::size_t length) {
    for (const Signature &sig : phy_signatures()) {
        if (length < sig.bytes.size())
            continue;
        for (std::size_t at = 0; at + sig.bytes.size() <= length; at += 2) {
            bool match = true;
            for (std::size_t k = 0; match && k < sig.bytes.size(); ++k)
                match = (data[at + k] & sig.mask[k]) == (sig.bytes[k] & sig.mask[k]);
            if (!match)
                continue;
            const std::uint32_t address = virtual_base + static_cast<std::uint32_t>(at);
            stub_function(address, sig.result);
            if (trace_rom)
                log(std::string("[phy] stubbed ") + sig.name + " at 0x" + [&] {
                    char hex[12];
                    std::snprintf(hex, sizeof hex, "%08x", address);
                    return std::string(hex);
                }());
            break;
        }
    }
}

void Esp32::connect_radio(mcuemu::VirtualAir &air, const std::string &node) { mac_->connect(air, node); }

void Esp32::load_flash(const std::vector<std::uint8_t> &data, std::uint32_t offset) {
    if (static_cast<std::uint64_t>(offset) + data.size() > flash_.size())
        throw std::invalid_argument("image does not fit the flash");
    std::memcpy(flash_.data() + offset, data.data(), data.size());
}

void Esp32::log(const std::string &line) const { std::cerr << line << '\n'; }

void Esp32::reset() {
    for (auto &block : blocks_)
        block->reset();
    scratch_.clear(); // registers no block models go back to zero too
    // The ROM loader: header (magic 0xE9, segment count, entry), then segments
    // of (load address, size, data). Only segments that land in SRAM are copied
    // here; IROM/DROM segments stay in the flash for the cache MMU.
    if (flash_.size() < 24 || flash_[0] != 0xe9)
        throw std::runtime_error("no ESP image header (0xE9) at flash offset 0");
    std::uint8_t *sram = core_.host_pointer(SramBase, layout_.sram_size);
    std::memset(sram, 0, layout_.sram_size);
    const unsigned segments = flash_[1];
    std::uint32_t entry = 0;
    std::memcpy(&entry, &flash_[4], 4);
    std::uint32_t offset = 24;
    for (unsigned i = 0; i < segments; ++i) {
        std::uint32_t address = 0, length = 0;
        if (offset + 8 > flash_.size())
            throw std::runtime_error("truncated image header");
        std::memcpy(&address, &flash_[offset], 4);
        std::memcpy(&length, &flash_[offset + 4], 4);
        offset += 8;
        if (offset + length > flash_.size())
            throw std::runtime_error("truncated image segment");
        if (address >= SramBase && address + length <= SramBase + layout_.sram_size)
            std::memcpy(sram + (address - SramBase), &flash_[offset], length);
        else if (address >= LpSramBase && address + length <= LpSramBase + layout_.lp_sram_size)
            core_.write_memory(address, &flash_[offset], length);
        else
            log("image segment at 0x" + std::to_string(address) + " is not RAM, left in flash");
        offset += length;
    }
    // Data the ROM leaves behind in the top of the SRAM and that boot code reads:
    // rom_spiflash_legacy_data points to a esp_rom_spiflash_legacy_data_t (the
    // chip description, then dummy_len_plus[3] and sig_matrix).
    const std::uint32_t LegacyData = SramBase + layout_.sram_size - 0x100;
    const std::uint32_t LegacyDataPointer = SramBase + layout_.sram_size - 0x14;
    const std::uint32_t chip[8] = {0, config_.flash_size, 0x10000, 0x1000, 0x100, 0xffff, 0, 0};
    core_.write_memory(LegacyData, chip, sizeof chip);
    core_.write_memory(LegacyDataPointer, &LegacyData, 4);
    if (layout_.phy_rom) {
        std::uint32_t romfuncs[64];
        for (unsigned k = 0; k < 64; ++k)
            romfuncs[k] = phy_rom_slots() + 4 * k;
        core_.write_memory(phy_rom_funcs(), romfuncs, sizeof romfuncs);
    }
    core_.reset(entry);
    // The ROM loader calls the image's entry on its own stack, in the top of the SRAM below
    // the data it keeps there (a C entry point such as the IDF bootloader's relies on it).
    core_.set_reg(2, SramBase + layout_.sram_size - 0x800);
    // Interrupts are globally enabled when the image starts (mstatus.MIE): esp-hal never sets the bit
    // itself, it only saves and restores it around critical sections and WFI, so
    // something before it (the ROM or the bootloader) must have. Inferred from the H2 app, which
    // otherwise never takes an interrupt.
    std::uint32_t mstatus = 0x1808; // MPP = machine, MIE
    uc_reg_write(core_.engine(), UC_RISCV_REG_MSTATUS, &mstatus);
}

void Esp32::stub_function(std::uint32_t address, std::uint32_t value) {
    stubs_[address] = value;
    core_.add_breakpoint(address);
}

riscv::RunResult Esp32::run_until(std::uint64_t target_ns) {
    const std::uint64_t target = core_.ns_to_cycles(target_ns);
    for (;;) {
        riscv::RunResult result = core_.run(target);
        if (result.reason == riscv::StopReason::Halted) {
            const std::uint32_t pc = core_.pc();
            auto stub = stubs_.find(pc);
            if (stub != stubs_.end()) {
                core_.set_reg(10, stub->second);
                core_.set_reg(11, 0);
                core_.set_reg(32, core_.reg(1)); // return to ra
                continue;
            }
            if (pc >= RomBase && pc < RomBase + layout_.rom_size && rom_->call(pc))
                continue;
        }
        return result;
    }
}

void Esp32::uart_tx(std::uint8_t byte) {
    if (on_uart_tx)
        on_uart_tx(byte);
}

// ---- peripheral window ----------------------------------------------------------

namespace {
Block *find_block(const std::map<std::uint32_t, Block *> &blocks, std::uint32_t address) {
    auto it = blocks.upper_bound(address);
    if (it == blocks.begin())
        return nullptr;
    --it;
    Block *block = it->second;
    return address < block->base() + block->size() ? block : nullptr;
}
} // namespace

std::uint32_t Esp32::read(std::uint32_t address, unsigned size) {
    if (Block *block = find_block(by_base_, address)) {
        const std::uint32_t value = block->read(address - block->base(), size);
        trace_block(false, address, value, size);
        return value;
    }
    auto &words = scratch_;
    auto it = words.find(address & ~3u);
    const std::uint32_t value = it == words.end() ? 0 : it->second;
    if (trace_mmio && trace_limit != 0 && trace_limit-- != 0) {
        char buffer[96];
        std::snprintf(buffer, sizeof buffer, "[mmio] R%u 0x%08x -> 0x%08x  pc=0x%08x", size, address,
                      value, core_.pc());
        log(buffer);
    }
    return value;
}

void Esp32::trace_block(bool write, std::uint32_t address, std::uint32_t value, unsigned size) {
    for (const auto &range : trace_blocks) {
        if (address < range.first || address >= range.second || trace_limit == 0)
            continue;
        --trace_limit;
        char buffer[112];
        std::snprintf(buffer, sizeof buffer, "[blk] %s%u 0x%08x %s 0x%08x  pc=0x%08x t=%llu ns", write ? "W" : "R",
                      size, address, write ? "<-" : "->", value, core_.pc(),
                      static_cast<unsigned long long>(time_ns()));
        log(buffer);
        return;
    }
}

void Esp32::write(std::uint32_t address, std::uint32_t value, unsigned size) {
    if (Block *block = find_block(by_base_, address)) {
        trace_block(true, address, value, size);
        block->write(address - block->base(), value, size);
        core_.request_service(core_.cycles()); // a write can arm or fire a timer
        return;
    }
    auto &word = scratch_[address & ~3u];
    if (size == 4) {
        word = value;
    } else {
        const unsigned shift = (address & 3u) * 8;
        const std::uint32_t mask = (size == 1 ? 0xffu : 0xffffu) << shift;
        word = (word & ~mask) | ((value << shift) & mask);
    }
    if (trace_mmio && trace_limit != 0 && trace_limit-- != 0) {
        char buffer[96];
        std::snprintf(buffer, sizeof buffer, "[mmio] W%u 0x%08x <- 0x%08x  pc=0x%08x", size, address,
                      value, core_.pc());
        log(buffer);
    }
}

} // namespace esp32emu

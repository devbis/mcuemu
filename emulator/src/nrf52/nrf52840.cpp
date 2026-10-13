#include "nrf52emu/nrf52840.hpp"

#include "nrf52/gpio.hpp"
#include "nrf52/peripherals.hpp"

#include <algorithm>
#include <cstring>
#include <ostream>
#include <stdexcept>

namespace nrf52emu {

// ---- Peripheral helpers ------------------------------------------------------

std::uint64_t Peripheral::now() const { return chip_.core().cycles(); }
std::uint64_t Peripheral::clock_hz() const { return chip_.core().clock_hz(); }
void Peripheral::set_irq_level(bool level) {
    if (irq_ >= 0)
        chip_.core().set_irq_line(static_cast<unsigned>(irq_), level);
}
void Peripheral::event_raised(std::uint32_t event_offset) { chip_.route_event(base_ + event_offset); }
void Peripheral::request_service(std::uint64_t at_cycle) { chip_.request_service(at_cycle); }

// ---- construction ---------------------------------------------------------------

namespace {

void put32(std::uint8_t *base, std::uint32_t offset, std::uint32_t value) {
    std::memcpy(base + offset, &value, 4);
}

std::uint64_t splitmix(std::uint64_t &state) {
    std::uint64_t z = (state += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

} // namespace

Nrf52840::Nrf52840() {
    cortexm::Core::Config config;
    config.model = cortexm::Model::M4;
    config.clock_hz = ClockHz;
    config.irq_count = 48;
    config.priority_bits = 3;
    core_ = std::make_unique<cortexm::Core>(config);
    flash_ = core_->map_memory(0, FlashSize, false);
    ram_ = core_->map_memory(RamBase, RamSize, true);
    ficr_ = core_->map_memory(FicrBase, 0x1000, false);
    uicr_ = core_->map_memory(UicrBase, 0x1000, false);
    std::memset(ficr_, 0xff, 0x1000);
    std::memset(ram_, 0, RamSize);
    // FICR (nRF52840 Product Specification, "FICR").
    put32(ficr_, 0x010, 4096);     // CODEPAGESIZE
    put32(ficr_, 0x014, 256);      // CODESIZE
    put32(ficr_, 0x100, 0x52840);  // INFO.PART
    put32(ficr_, 0x104, 0x41414430); // INFO.VARIANT "AAD0"
    put32(ficr_, 0x108, 0x2004);   // INFO.PACKAGE QI
    put32(ficr_, 0x10c, 256);      // INFO.RAM (KiB)
    put32(ficr_, 0x110, 1024);     // INFO.FLASH (KiB)
    set_device_id(0x0123456789abcdefull);

    build_peripherals();
    core_->map_mmio(PeripheralBase, 0x80000, *this); // ids 0..127
    core_->map_mmio(GpioBase, 0x1000, *this);
    core_->set_service_handler([this](std::uint64_t now) { return service(now); });
    core_->set_write_protect_handler([this](std::uint32_t address, unsigned size, std::uint32_t value) {
        return static_cast<Nvmc *>(peripherals_[Nvmc::Id].get())->on_store(address, size, value);
    });
}

Nrf52840::~Nrf52840() = default;

void Nrf52840::set_device_id(std::uint64_t id) {
    std::uint64_t state = id;
    put32(ficr_, 0x060, static_cast<std::uint32_t>(id));
    put32(ficr_, 0x064, static_cast<std::uint32_t>(id >> 32));
    for (std::uint32_t i = 0; i != 4; ++i) {
        put32(ficr_, 0x080 + 4 * i, static_cast<std::uint32_t>(splitmix(state))); // ER
        put32(ficr_, 0x090 + 4 * i, static_cast<std::uint32_t>(splitmix(state))); // IR
    }
    put32(ficr_, 0x0a0, 1); // DEVICEADDRTYPE: random static
    put32(ficr_, 0x0a4, static_cast<std::uint32_t>(splitmix(state)));
    put32(ficr_, 0x0a8, static_cast<std::uint32_t>(splitmix(state) & 0xffffu) | 0xc000u);
    if (auto *rng = dynamic_cast<Rng *>(peripherals_[Rng::Id].get()))
        rng->seed(id ^ 0x5deece66dull);
}

void Nrf52840::build_peripherals() {
    gpio_ = std::make_unique<Gpio>(*this);
    peripherals_[PowerClock::Id] = std::make_unique<PowerClock>(*this);
    peripherals_[2] = std::make_unique<Uarte>(*this, 2, 0);
    peripherals_[40] = std::make_unique<Uarte>(*this, 40, 1);
    peripherals_[3] = std::make_unique<Twim>(*this, 3);
    peripherals_[4] = std::make_unique<Twim>(*this, 4);
    peripherals_[Gpiote::Id] = std::make_unique<Gpiote>(*this, *gpio_);
    for (unsigned id : {8u, 9u, 10u})
        peripherals_[id] = std::make_unique<Timer>(*this, id, 4);
    for (unsigned id : {26u, 27u})
        peripherals_[id] = std::make_unique<Timer>(*this, id, 6);
    peripherals_[11] = std::make_unique<Rtc>(*this, 11, 3);
    peripherals_[17] = std::make_unique<Rtc>(*this, 17, 4);
    peripherals_[36] = std::make_unique<Rtc>(*this, 36, 4);
    peripherals_[Saadc::Id] = std::make_unique<Saadc>(*this);
    peripherals_[Ecb::Id] = std::make_unique<Ecb>(*this);
    peripherals_[Temp::Id] = std::make_unique<Temp>(*this);
    peripherals_[Rng::Id] = std::make_unique<Rng>(*this);
    peripherals_[Wdt::Id] = std::make_unique<Wdt>(*this);
    for (unsigned id = 20; id <= 25; ++id)
        peripherals_[id] = std::make_unique<Egu>(*this, id);
    peripherals_[Nvmc::Id] = std::make_unique<Nvmc>(*this);
    peripherals_[Ppi::Id] = std::make_unique<Ppi>(*this);
    for (unsigned id = 0; id != peripherals_.size(); ++id)
        if (!peripherals_[id])
            peripherals_[id] = std::make_unique<StubPeripheral>(
                *this, PeripheralBase + id * 0x1000u, id < 48 ? static_cast<int>(id) : -1);
}

void Nrf52840::load_firmware(const std::vector<std::uint8_t> &image, std::uint32_t offset) {
    if (offset > FlashSize || image.size() > FlashSize - offset)
        throw std::runtime_error("firmware does not fit in the 1 MiB flash");
    std::copy(image.begin(), image.end(), flash_ + offset);
    code_end_ = std::max<std::uint32_t>(code_end_, offset + static_cast<std::uint32_t>(image.size()));
    core_->invalidate_code();
}

void Nrf52840::reset() {
    system_off_ = false;
    gpio_->reset();
    for (auto &peripheral : peripherals_)
        peripheral->reset();
    core_->reset();
    serviced_.clear();
    for (unsigned id = 0; id != peripherals_.size(); ++id)
        if (dynamic_cast<StubPeripheral *>(peripherals_[id].get()) == nullptr &&
            dynamic_cast<Ppi *>(peripherals_[id].get()) == nullptr)
            serviced_.push_back(id);
}

void Nrf52840::request_reset(std::uint32_t reason_bits) {
    static_cast<PowerClock *>(peripherals_[PowerClock::Id].get())->add_reset_reason(reason_bits);
    core_->request_reset();
}

void Nrf52840::request_system_off() {
    system_off_ = true;
    core_->request_reset(); // leaves the run loop; run() then idles
}

void Nrf52840::wake_from_system_off() {
    if (!system_off_)
        return;
    system_off_ = false;
    static_cast<PowerClock *>(peripherals_[PowerClock::Id].get())->add_reset_reason(1u << 16);
}

cortexm::RunResult Nrf52840::run(std::uint64_t until_cycle) {
    for (;;) {
        if (system_off_) {
            core_->advance_to(until_cycle);
            cortexm::RunResult result;
            result.reason = cortexm::StopReason::Deadline;
            return result;
        }
        cortexm::RunResult result = core_->run(until_cycle);
        if (result.reason != cortexm::StopReason::Reset)
            return result;
        if (system_off_)
            continue;
        reset();
    }
}

// ---- service, PPI -----------------------------------------------------------------

std::uint64_t Nrf52840::service(std::uint64_t now) {
    std::uint64_t next = Never;
    for (unsigned id : serviced_)
        next = std::min(next, peripherals_[id]->service(now));
    return next;
}

void Nrf52840::route_event(std::uint32_t address) {
    static_cast<Ppi *>(peripherals_[Ppi::Id].get())->route(address);
}

void Nrf52840::trigger_task(std::uint32_t address) {
    if (address < PeripheralBase || address >= PeripheralBase + 0x80000)
        return;
    peripherals_[(address >> 12) & 0x7f]->task(address & 0xfff);
}

// ---- pads --------------------------------------------------------------------------

PadDrive Nrf52840::pad_drive(unsigned pad) const { return pad < PadCount ? gpio_->drive(pad) : PadDrive::HiZ; }

void Nrf52840::set_pad_input(unsigned pad, bool high) {
    if (pad < PadCount)
        gpio_->set_external(pad, high);
}

void Nrf52840::notify_pad(unsigned pad) {
    if (pad_observer_)
        pad_observer_(pad);
}

namespace {
Uarte *uart_of(std::array<std::unique_ptr<Peripheral>, 128> &slots, unsigned index) {
    return index < Nrf52840::UartCount ? static_cast<Uarte *>(slots[index == 0 ? 2 : 40].get()) : nullptr;
}
} // namespace

bool Nrf52840::uart_can_receive(unsigned index) {
    Uarte *uart = uart_of(peripherals_, index);
    return uart && uart->can_receive();
}

bool Nrf52840::uart_receive(unsigned index, std::uint8_t byte) {
    Uarte *uart = uart_of(peripherals_, index);
    return uart && uart->receive(byte);
}

std::vector<std::uint8_t> Nrf52840::uart_take_tx(unsigned index) {
    Uarte *uart = uart_of(peripherals_, index);
    return uart ? uart->take_tx() : std::vector<std::uint8_t>{};
}

std::uint32_t Nrf52840::analog_input_mv(unsigned pad) const {
    if (analog_provider_)
        return analog_provider_(pad);
    return pad < PadCount && gpio_->level(pad) ? supply_mv_ : 0;
}

bool Nrf52840::parse_pad(const std::string &name, unsigned &pad) {
    // "P0.00" .. "P0.31", "P1.00" .. "P1.15"
    if (name.size() < 4 || name[0] != 'P' || name[2] != '.')
        return false;
    if (name[1] != '0' && name[1] != '1')
        return false;
    unsigned pin = 0;
    for (std::size_t i = 3; i != name.size(); ++i) {
        if (name[i] < '0' || name[i] > '9')
            return false;
        pin = pin * 10 + static_cast<unsigned>(name[i] - '0');
    }
    if (name.size() > 5 || pin >= (name[1] == '0' ? 32u : 16u))
        return false;
    pad = (name[1] - '0') * 32 + pin;
    return true;
}

// ---- MMIO dispatch ----------------------------------------------------------------

std::uint32_t Nrf52840::read(std::uint32_t address, unsigned size) {
    std::uint32_t value = 0;
    if (address >= GpioBase) {
        value = gpio_->read(address - GpioBase);
    } else {
        Peripheral &p = *peripherals_[(address >> 12) & 0x7f];
        value = p.read(address & 0xffc);
    }
    if (trace_)
        *trace_ << "R 0x" << std::hex << address << " -> 0x" << value << " pc=0x" << core_->pc()
                << std::dec << '\n';
    if (size < 4)
        value = (value >> ((address & 3u) * 8)) & (size == 1 ? 0xffu : 0xffffu);
    return value;
}

void Nrf52840::write(std::uint32_t address, std::uint32_t value, unsigned size) {
    if (trace_)
        *trace_ << "W 0x" << std::hex << address << " <- 0x" << value << " pc=0x" << core_->pc()
                << std::dec << '\n';
    if (size < 4)
        value <<= (address & 3u) * 8;
    if (address >= GpioBase)
        gpio_->write(address - GpioBase, value);
    else
        peripherals_[(address >> 12) & 0x7f]->write(address & 0xffc, value);
}

} // namespace nrf52emu

#include "mcs51emu/ca51f253l3.hpp"

#include "unicorn/mcs51.h"
#include "unicorn/unicorn.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace mcs51emu {
namespace {

constexpr int SfrStackPointer = 0x81;
constexpr int SfrPort0 = 0x80;
constexpr int SfrPort1 = 0x90;
constexpr int SfrPort2 = 0xa0;
constexpr int SfrPort3 = 0xb0;
constexpr std::uint8_t SfrUart0Control = 0x98;
constexpr std::uint8_t SfrUart0Buffer = 0x99;
constexpr std::uint8_t UartTransmitInterruptFlag = 0x02;
constexpr std::uint8_t SfrUart1Control = 0x9a;
constexpr std::uint8_t SfrUart1Buffer = 0x9b;
constexpr std::uint8_t SfrUart1ReloadLow = 0x9c;
constexpr std::uint8_t SfrUart1ReloadHigh = 0x9d;
constexpr std::uint8_t Uart1InterruptFlags = 0x03;
constexpr std::uint64_t Uart1BitClockDivider = 32;
constexpr std::uint64_t Uart1FrameBits = 10;
constexpr std::uint8_t SfrInterruptEnable = 0xa8;
constexpr std::uint8_t InterruptEnableGlobal = 0x80;
constexpr std::uint8_t InterruptEnableUart1 = 0x40;
constexpr std::uint16_t Uart1InterruptVector = 0x0033;
constexpr std::uint8_t SfrPwmUpdate = 0xdb;
constexpr std::uint8_t SfrTimer0Control = 0x88;
constexpr std::uint8_t SfrTimerMode = 0x89;
constexpr std::uint8_t SfrTimer0Low = 0x8a;
constexpr std::uint8_t SfrTimer0High = 0x8c;
constexpr std::uint8_t Timer0Run = 0x10;
constexpr std::uint8_t Timer0Overflow = 0x20;
constexpr std::uint16_t XsfrFscmd = 0xc01;
constexpr std::uint16_t XsfrFsdat = 0xc02;
constexpr std::uint16_t XsfrLock = 0xc03;
constexpr std::uint16_t XsfrPtsl = 0xc05;
constexpr std::uint16_t XsfrPtsh = 0xc06;
constexpr std::uint8_t FlashCmdDataRead = 1;
constexpr std::uint8_t FlashCmdDataWrite = 2;
constexpr std::uint8_t FlashCmdDataEraseSector = 3;
constexpr std::uint8_t FlashUnlockData = 0x2a;
constexpr std::uint8_t FlashLock = 0xaa;
constexpr std::size_t FlashSectorSize = 128;

std::uint8_t hex_byte(std::string_view text, std::size_t offset,
                      const std::string &path) {
    if (offset + 2 > text.size())
        throw std::runtime_error(path + ": truncated Intel HEX record");
    unsigned value = 0;
    for (std::size_t i = offset; i < offset + 2; ++i) {
        const unsigned char ch = static_cast<unsigned char>(text[i]);
        const int digit = std::isdigit(ch) ? ch - '0'
                          : std::toupper(ch) >= 'A' && std::toupper(ch) <= 'F'
                                ? std::toupper(ch) - 'A' + 10
                                : -1;
        if (digit < 0)
            throw std::runtime_error(path + ": invalid Intel HEX digit");
        value = (value << 4) | static_cast<unsigned>(digit);
    }
    return static_cast<std::uint8_t>(value);
}

std::vector<std::uint8_t> load_intel_hex(const std::string &path) {
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("cannot open firmware image: " + path);

    std::vector<std::uint8_t> flash(Ca51F253L3::FlashSize, 0xff);
    std::string line;
    std::uint32_t upper_address = 0;
    bool eof_seen = false;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty())
            continue;
        if (eof_seen)
            throw std::runtime_error(path + ": data follows Intel HEX EOF record");
        if (line[0] != ':' || line.size() < 11 || (line.size() & 1u) == 0)
            throw std::runtime_error(path + ": malformed Intel HEX record at line " +
                                     std::to_string(line_number));

        const std::string_view record(line.data() + 1, line.size() - 1);
        const std::uint8_t count = hex_byte(record, 0, path);
        const std::uint8_t address_hi = hex_byte(record, 2, path);
        const std::uint8_t address_lo = hex_byte(record, 4, path);
        const std::uint8_t type = hex_byte(record, 6, path);
        if (record.size() != 10 + static_cast<std::size_t>(count) * 2)
            throw std::runtime_error(path + ": Intel HEX byte count mismatch at line " +
                                     std::to_string(line_number));

        unsigned sum = count + address_hi + address_lo + type;
        std::vector<std::uint8_t> data;
        data.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            data.push_back(hex_byte(record, 8 + i * 2, path));
            sum += data.back();
        }
        const std::uint8_t checksum = hex_byte(record, 8 + count * 2, path);
        sum += checksum;
        if ((sum & 0xffu) != 0)
            throw std::runtime_error(path + ": Intel HEX checksum mismatch at line " +
                                     std::to_string(line_number));

        const std::uint16_t offset = static_cast<std::uint16_t>(
            (static_cast<unsigned>(address_hi) << 8) | address_lo);
        switch (type) {
        case 0x00: {
            const std::uint64_t absolute = upper_address + offset;
            if (absolute + count > flash.size())
                throw std::runtime_error(path +
                                         ": firmware exceeds CA51F253L3 32 KiB flash");
            std::copy(data.begin(), data.end(), flash.begin() + absolute);
            break;
        }
        case 0x01:
            if (count != 0 || offset != 0)
                throw std::runtime_error(path + ": malformed Intel HEX EOF record");
            eof_seen = true;
            break;
        case 0x02:
            if (count != 2 || offset != 0)
                throw std::runtime_error(path + ": malformed segment address record");
            upper_address = static_cast<std::uint32_t>(
                ((static_cast<unsigned>(data[0]) << 8) | data[1]) << 4);
            break;
        case 0x04:
            if (count != 2 || offset != 0)
                throw std::runtime_error(path + ": malformed linear address record");
            upper_address = static_cast<std::uint32_t>(
                ((static_cast<unsigned>(data[0]) << 8) | data[1]) << 16);
            break;
        case 0x03: case 0x05:
            // Start-address records are metadata; reset entry is address zero
            // for the initial CA51F253L3 model.
            break;
        default:
            throw std::runtime_error(path + ": unsupported Intel HEX record type " +
                                     std::to_string(type));
        }
    }
    if (!eof_seen)
        throw std::runtime_error(path + ": missing Intel HEX EOF record");
    return flash;
}

std::vector<std::uint8_t> load_firmware_file(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot open firmware image: " + path);
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)),
                                    std::istreambuf_iterator<char>());
    if (bytes.empty())
        throw std::runtime_error("firmware image is empty: " + path);

    const auto first = std::find_if_not(bytes.begin(), bytes.end(), [](std::uint8_t ch) {
        return std::isspace(static_cast<unsigned char>(ch));
    });
    if (first != bytes.end() && *first == ':')
        return load_intel_hex(path);
    if (bytes.size() > Ca51F253L3::FlashSize)
        throw std::runtime_error("firmware exceeds CA51F253L3 32 KiB flash");
    bytes.resize(Ca51F253L3::FlashSize, 0xff);
    return bytes;
}

} // namespace

Ca51F253L3::Ca51F253L3() {
    uc_engine *engine = nullptr;
    const uc_err open_error = uc_open(UC_ARCH_MCS51, UC_MODE_LITTLE_ENDIAN, &engine);
    if (open_error != UC_ERR_OK)
        throw std::runtime_error(std::string("uc_open(MCS-51): ") +
                                 uc_strerror(open_error));
    uc_ = reinterpret_cast<uc_struct *>(engine);
    try {
        check_error(uc_mem_map(engine, 0, CodeSize,
                               UC_PROT_READ | UC_PROT_EXEC),
                    "map code space");
        std::vector<std::uint8_t> erased_code(CodeSize, 0xff);
        check_error(uc_mem_write(engine, 0, erased_code.data(), erased_code.size()),
                    "initialize erased flash");
        check_error(uc_mem_map(engine, XdataBase, XdataFlashPageOffset,
                               UC_PROT_ALL),
                    "map XDATA");
        check_error(uc_mmio_map(engine, XdataBase + XdataFlashPageOffset,
                                XdataFlashPageSize, xdata_flash_read, this,
                                xdata_flash_write, this),
                    "map data flash registers");
        check_error(uc_mem_map(engine, IdataBase, MappedInternalSize, UC_PROT_ALL),
                    "map IDATA");
        check_error(uc_mmio_map(engine, SfrBase, MappedInternalSize,
                                sfr_mmio_read, this, sfr_mmio_write, this),
                    "map SFR and UART");
        data_flash_.fill(0xff);
        reset();
    } catch (...) {
        uc_close(engine);
        uc_ = nullptr;
        throw;
    }
}

Ca51F253L3::~Ca51F253L3() {
    if (uc_)
        uc_close(reinterpret_cast<uc_engine *>(uc_));
}

void Ca51F253L3::check_error(int error, const char *operation) const {
    const auto result = static_cast<uc_err>(error);
    if (result != UC_ERR_OK)
        throw std::runtime_error(std::string("Unicorn ") + operation + ": " +
                                 uc_strerror(result));
}

void Ca51F253L3::load_firmware(const std::string &path) {
    const std::vector<std::uint8_t> image = load_firmware_file(path);
    check_error(uc_mem_write(reinterpret_cast<uc_engine *>(uc_), 0,
                             image.data(), image.size()),
                "load firmware");
    reset();
}

void Ca51F253L3::reset() {
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    std::array<std::uint8_t, MappedInternalSize> zeros{};
    check_error(uc_mem_write(engine, IdataBase, zeros.data(), zeros.size()),
                "reset IDATA");
    sfr_.fill(0);
    xdata_flash_page_.fill(0);
    uart0_output_.clear();
    uart1_output_.clear();
    data_flash_command_ = 0;
    data_flash_unlocked_ = false;
    data_flash_address_ = 0;
    timer0_last_cycles_ = 0;
    uart1_last_cycles_ = 0;
    uart1_tx_cycles_remaining_ = 0;
    uart1_tx_byte_ = 0;
    uart1_tx_active_ = false;
    uart1_tx_waiting_.clear();
    uart1_irq_requested_ = false;

    // 8051-compatible reset values. CA51-specific SFR reset behavior and
    // peripherals are added as their register documentation is modeled.
    write_sfr(SfrStackPointer, 0x07);
    write_sfr(SfrPort0, 0xff);
    write_sfr(SfrPort1, 0xff);
    write_sfr(SfrPort2, 0xff);
    write_sfr(SfrPort3, 0xff);
    std::uint16_t pc = 0;
    std::uint16_t interrupt_vector = 0;
    std::uint64_t cycles = 0;
    check_error(uc_reg_write(engine, UC_MCS51_REG_PC, &pc), "reset PC");
    check_error(uc_reg_write(engine, UC_MCS51_REG_IRQ_VECTOR,
                             &interrupt_vector),
                "reset pending interrupt");
    check_error(uc_reg_write(engine, UC_MCS51_REG_CYCLES, &cycles),
                "reset cycle counter");
    std::uint64_t instructions = 0;
    check_error(uc_reg_write(engine, UC_MCS51_REG_INSTRUCTIONS, &instructions),
                "reset instruction counter");
}

std::uint64_t Ca51F253L3::xdata_flash_read(uc_struct *opaque_uc,
                                           std::uint64_t offset, unsigned size,
                                           void *user_data) {
    (void)opaque_uc;
    auto *self = static_cast<Ca51F253L3 *>(user_data);
    std::uint64_t value = 0;
    for (unsigned i = 0; i < size; ++i) {
        const std::uint64_t address = offset + i;
        std::uint8_t byte = address < self->xdata_flash_page_.size()
                                ? self->xdata_flash_page_[address]
                                : 0xff;
        if (address == XsfrFsdat &&
            self->data_flash_command_ == FlashCmdDataRead) {
            byte = self->data_flash_address_ < self->data_flash_.size()
                       ? self->data_flash_[self->data_flash_address_]
                       : 0xff;
            ++self->data_flash_address_;
        }
        value |= static_cast<std::uint64_t>(byte) << (i * 8u);
    }
    return value;
}

void Ca51F253L3::xdata_flash_write(uc_struct *opaque_uc,
                                   std::uint64_t offset, unsigned size,
                                   std::uint64_t value, void *user_data) {
    (void)opaque_uc;
    auto *self = static_cast<Ca51F253L3 *>(user_data);
    for (unsigned i = 0; i < size; ++i) {
        const std::uint64_t address = offset + i;
        if (address >= self->xdata_flash_page_.size())
            continue;
        const std::uint8_t byte = static_cast<std::uint8_t>(value >> (i * 8u));
        self->xdata_flash_page_[address] = byte;
        switch (address) {
        case XsfrLock:
            self->data_flash_unlocked_ = byte == FlashUnlockData;
            if (byte == FlashLock)
                self->data_flash_unlocked_ = false;
            break;
        case XsfrPtsl:
        case XsfrPtsh:
            self->data_flash_address_ = static_cast<std::uint16_t>(
                self->xdata_flash_page_[XsfrPtsl] |
                (static_cast<std::uint16_t>(self->xdata_flash_page_[XsfrPtsh])
                 << 8));
            break;
        case XsfrFscmd:
            self->data_flash_command_ = byte;
            if (byte == 0) {
                break;
            }
            self->data_flash_address_ = static_cast<std::uint16_t>(
                self->xdata_flash_page_[XsfrPtsl] |
                (static_cast<std::uint16_t>(self->xdata_flash_page_[XsfrPtsh])
                 << 8));
            if (byte == FlashCmdDataEraseSector && self->data_flash_unlocked_) {
                const std::size_t sector_start =
                    (self->data_flash_address_ / FlashSectorSize) * FlashSectorSize;
                if (sector_start < self->data_flash_.size()) {
                    const std::size_t erase_size = std::min(
                        FlashSectorSize, self->data_flash_.size() - sector_start);
                    std::fill_n(self->data_flash_.begin() + sector_start,
                                erase_size, 0xff);
                }
                self->data_flash_command_ = 0;
                self->xdata_flash_page_[XsfrFscmd] = 0;
            }
            break;
        case XsfrFsdat:
            if (self->data_flash_command_ == FlashCmdDataWrite &&
                self->data_flash_unlocked_ &&
                self->data_flash_address_ < self->data_flash_.size()) {
                self->data_flash_[self->data_flash_address_] &= byte;
                ++self->data_flash_address_;
            }
            break;
        default:
            break;
        }
    }
}

std::uint64_t Ca51F253L3::sfr_mmio_read(uc_struct *opaque_uc,
                                        std::uint64_t offset, unsigned size,
                                        void *user_data) {
    (void)opaque_uc;
    auto *self = static_cast<Ca51F253L3 *>(user_data);
    std::uint64_t value = 0;
    for (unsigned i = 0; i < size; ++i) {
        const std::uint64_t address = offset + i;
        const std::uint8_t byte = address < self->sfr_.size()
                                      ? self->sfr_[address]
                                      : 0xff;
        value |= static_cast<std::uint64_t>(byte) << (i * 8u);
    }
    return value;
}

void Ca51F253L3::sfr_mmio_write(uc_struct *opaque_uc, std::uint64_t offset,
                                unsigned size, std::uint64_t value,
                                void *user_data) {
    (void)opaque_uc;
    auto *self = static_cast<Ca51F253L3 *>(user_data);
    for (unsigned i = 0; i < size; ++i) {
        const std::uint64_t address = offset + i;
        if (address >= self->sfr_.size())
            continue;
        const std::uint8_t byte = static_cast<std::uint8_t>(value >> (i * 8u));
        if (address == SfrUart1Control) {
            // CA51F253L3 UART1 uses write-one-to-clear RI1/TI1 status bits.
            const std::uint8_t pending = self->sfr_[address] & Uart1InterruptFlags;
            self->sfr_[address] = static_cast<std::uint8_t>(
                (byte & ~Uart1InterruptFlags) |
                (pending & ~(byte & Uart1InterruptFlags)));
        } else {
            self->sfr_[address] = byte;
        }
        if (address == SfrUart0Buffer) {
            self->uart0_output_.push_back(static_cast<char>(byte));
            // Complete transmit immediately; baud-rate timing is not modeled.
            self->sfr_[SfrUart0Control] |= UartTransmitInterruptFlag;
        } else if (address == SfrUart1Buffer) {
            self->start_uart1_transmit(byte, opaque_uc);
        } else if (address == SfrPwmUpdate) {
            // PWMUPD is a write-one update request. The peripheral clears it
            // after applying the shadow values; completion is immediate here.
            self->sfr_[SfrPwmUpdate] = 0;
        } else if (address == SfrTimer0Control || address == SfrTimerMode) {
            self->rebase_timer0();
        }
    }
    self->update_uart1_interrupt(opaque_uc);
    const bool timer0_active =
        (self->sfr_[SfrTimer0Control] & Timer0Run) != 0 &&
        (self->sfr_[SfrTimerMode] & 0x0f) == 0x01;
    if (self->fast_mode_ &&
        (timer0_active || self->uart1_irq_requested_ ||
         self->uart1_tx_active_))
        (void)uc_emu_stop(reinterpret_cast<uc_engine *>(opaque_uc));
}

std::uint64_t Ca51F253L3::uart1_frame_cycles() const noexcept {
    const std::uint16_t reload = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(sfr_[SfrUart1ReloadHigh]) << 8) |
        sfr_[SfrUart1ReloadLow]);
    const std::uint64_t reload_count = reload & 0x03ffu;
    const std::uint64_t timer_period = 0x400u - reload_count;
    // UART1's baud counter advances at FOSC/32. Mode 1 sends 8 data bits,
    // a start bit, and a stop bit; TI1 is raised when the stop bit completes.
    return timer_period * Uart1BitClockDivider * Uart1FrameBits;
}

void Ca51F253L3::start_uart1_transmit(std::uint8_t byte,
                                      uc_struct *opaque_uc) noexcept {
    if (uart1_tx_active_) {
        uart1_tx_waiting_.push_back(byte);
        return;
    }

    uart1_tx_byte_ = byte;
    uart1_tx_cycles_remaining_ = uart1_frame_cycles();
    uart1_tx_active_ = true;
    std::uint64_t cycles = 0;
    if (uc_reg_read(reinterpret_cast<uc_engine *>(opaque_uc),
                    UC_MCS51_REG_CYCLES, &cycles) == UC_ERR_OK)
        uart1_last_cycles_ = cycles;
}

void Ca51F253L3::advance_uart1(uc_struct *opaque_uc) noexcept {
    if (!uart1_tx_active_)
        return;

    auto *engine = reinterpret_cast<uc_engine *>(opaque_uc);
    std::uint64_t cycles = 0;
    if (uc_reg_read(engine, UC_MCS51_REG_CYCLES, &cycles) != UC_ERR_OK)
        return;
    if (cycles < uart1_last_cycles_) {
        uart1_last_cycles_ = cycles;
        return;
    }

    std::uint64_t elapsed = cycles - uart1_last_cycles_;
    uart1_last_cycles_ = cycles;
    bool completed = false;
    while (uart1_tx_active_ && elapsed >= uart1_tx_cycles_remaining_) {
        elapsed -= uart1_tx_cycles_remaining_;
        uart1_output_.push_back(static_cast<char>(uart1_tx_byte_));
        sfr_[SfrUart1Control] |= UartTransmitInterruptFlag;
        completed = true;

        if (uart1_tx_waiting_.empty()) {
            uart1_tx_active_ = false;
            uart1_tx_cycles_remaining_ = 0;
        } else {
            uart1_tx_byte_ = uart1_tx_waiting_.front();
            uart1_tx_waiting_.pop_front();
            uart1_tx_cycles_remaining_ = uart1_frame_cycles();
        }
    }
    if (uart1_tx_active_)
        uart1_tx_cycles_remaining_ -= elapsed;
    if (completed)
        update_uart1_interrupt(opaque_uc);
}

void Ca51F253L3::update_uart1_interrupt(uc_struct *opaque_uc) noexcept {
    const bool pending =
        (sfr_[SfrInterruptEnable] &
         (InterruptEnableGlobal | InterruptEnableUart1)) ==
            (InterruptEnableGlobal | InterruptEnableUart1) &&
        (sfr_[SfrUart1Control] & Uart1InterruptFlags) != 0;
    if (pending == uart1_irq_requested_)
        return;

    const std::uint16_t vector = pending ? Uart1InterruptVector : 0;
    auto *engine = reinterpret_cast<uc_engine *>(opaque_uc);
    if (uc_reg_write(engine, UC_MCS51_REG_IRQ_VECTOR, &vector) == UC_ERR_OK)
        uart1_irq_requested_ = pending;
}

void Ca51F253L3::rebase_timer0() noexcept {
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    std::uint64_t cycles = 0;
    if (uc_reg_read(engine, UC_MCS51_REG_CYCLES, &cycles) == UC_ERR_OK)
        timer0_last_cycles_ = cycles;
}

void Ca51F253L3::advance_timer0(std::uint64_t cycle_delta) noexcept {
    if (cycle_delta == 0 ||
        (sfr_[SfrTimer0Control] & Timer0Run) == 0 ||
        (sfr_[SfrTimerMode] & 0x0f) != 0x01)
        return;

    const std::uint32_t timer =
        (static_cast<std::uint32_t>(sfr_[SfrTimer0High]) << 8) |
        sfr_[SfrTimer0Low];
    const std::uint64_t advanced = timer + cycle_delta;
    if (advanced > 0xffffu)
        sfr_[SfrTimer0Control] |= Timer0Overflow;
    const std::uint16_t value = static_cast<std::uint16_t>(advanced);
    sfr_[SfrTimer0Low] = static_cast<std::uint8_t>(value);
    sfr_[SfrTimer0High] = static_cast<std::uint8_t>(value >> 8);
}

void Ca51F253L3::code_hook(uc_struct *opaque_uc, std::uint64_t address,
                           std::uint32_t size, void *user_data) {
    auto *self = static_cast<Ca51F253L3 *>(user_data);
    auto *engine = reinterpret_cast<uc_engine *>(opaque_uc);
    self->advance_uart1(opaque_uc);
    const bool timer0_active = (self->sfr_[SfrTimer0Control] & Timer0Run) != 0 &&
                               (self->sfr_[SfrTimerMode] & 0x0f) == 0x01;
    if (timer0_active) {
        std::uint64_t cycles = 0;
        if (uc_reg_read(engine, UC_MCS51_REG_CYCLES, &cycles) == UC_ERR_OK) {
            if (cycles >= self->timer0_last_cycles_)
                self->advance_timer0(cycles - self->timer0_last_cycles_);
            self->timer0_last_cycles_ = cycles;
        }
    }
    if (address == Uart1InterruptVector) {
        // The CPU consumed the pending vector. Re-arm it if the device flag
        // remains set, such as when an ISR has not acknowledged the source.
        self->uart1_irq_requested_ = false;
        self->update_uart1_interrupt(opaque_uc);
    }
    if (!self->trace_output_ && !timer0_active &&
        !self->uart1_irq_requested_ && !self->uart1_tx_active_) {
        (void)uc_emu_stop(engine);
        return;
    }
    if (!self->trace_output_)
        return;
    auto &out = *self->trace_output_;

    std::array<std::uint8_t, 3> bytes{};
    const std::size_t read_size = std::min<std::size_t>(size, bytes.size());
    if (uc_mem_read(engine, address, bytes.data(), read_size) != UC_ERR_OK)
        return;
    out << "pc=0x" << std::hex << std::setw(4) << std::setfill('0') << address
        << "  ";
    for (std::size_t i = 0; i < read_size; ++i)
        out << std::setw(2) << static_cast<unsigned>(bytes[i]) << ' ';
    out << std::dec << std::setfill(' ') << '\n';
}

void Ca51F253L3::set_code_hook_enabled(bool enabled) {
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    if (enabled == code_hook_installed_)
        return;
    if (enabled) {
        check_error(uc_hook_add(engine, reinterpret_cast<uc_hook *>(&code_hook_),
                                UC_HOOK_CODE,
                                reinterpret_cast<void *>(
                                    static_cast<uc_cb_hookcode_t>(Ca51F253L3::code_hook)),
                                this, 1, 0),
                    "install code hook");
        code_hook_installed_ = true;
    } else {
        check_error(uc_hook_del(engine, static_cast<uc_hook>(code_hook_)),
                    "remove code hook");
        code_hook_ = 0;
        code_hook_installed_ = false;
    }
}

RunResult Ca51F253L3::run(std::size_t instruction_limit) {
    RunResult result;
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    std::uint64_t before = 0;
    check_error(uc_reg_read(engine, UC_MCS51_REG_INSTRUCTIONS, &before),
                "read instruction counter");
    uc_err error = UC_ERR_OK;
    while (result.instructions < instruction_limit) {
        const bool timer0_active =
            (sfr_[SfrTimer0Control] & Timer0Run) != 0 &&
            (sfr_[SfrTimerMode] & 0x0f) == 0x01;
        const bool hook_needed = trace_output_ || timer0_active ||
                                 uart1_irq_requested_ || uart1_tx_active_;
        set_code_hook_enabled(hook_needed);
        fast_mode_ = !hook_needed;

        std::uint16_t pc = 0;
        check_error(uc_reg_read(engine, UC_MCS51_REG_PC, &pc), "read PC");
        error = uc_emu_start(engine, pc, CodeSize, 0,
                             instruction_limit - result.instructions);
        fast_mode_ = false;
        if (error != UC_ERR_OK)
            break;

        std::uint64_t current = 0;
        check_error(uc_reg_read(engine, UC_MCS51_REG_INSTRUCTIONS, &current),
                    "read instruction counter");
        result.instructions = static_cast<std::size_t>(current - before);
        if (result.instructions >= instruction_limit)
            break;

        if (result.instructions == 0) {
            error = UC_ERR_EXCEPTION;
            break;
        }
    }
    std::uint64_t after = 0;
    check_error(uc_reg_read(engine, UC_MCS51_REG_INSTRUCTIONS, &after),
                "read instruction counter");
    result.instructions = static_cast<std::size_t>(after - before);
    check_error(uc_reg_read(engine, UC_MCS51_REG_PC, &result.pc), "read PC");
    check_error(uc_reg_read(engine, UC_MCS51_REG_CYCLES, &result.cycles),
                "read cycle counter");
    result.step_limit_reached = error == UC_ERR_OK &&
                                result.instructions >= instruction_limit;
    if (error != UC_ERR_OK)
        result.error = uc_strerror(error);
    return result;
}

std::uint8_t Ca51F253L3::read_sfr(std::uint8_t address) const {
    return sfr_[address];
}

std::uint8_t Ca51F253L3::read_idata(std::uint8_t address) const {
    std::uint8_t value = 0;
    const uc_err error = uc_mem_read(reinterpret_cast<uc_engine *>(uc_),
                                     IdataBase + address, &value, sizeof(value));
    if (error != UC_ERR_OK)
        throw std::runtime_error(std::string("Unicorn read IDATA: ") +
                                 uc_strerror(error));
    return value;
}

std::uint8_t Ca51F253L3::read_xdata(std::uint16_t address) const {
    std::uint8_t value = 0;
    const uc_err error = uc_mem_read(reinterpret_cast<uc_engine *>(uc_),
                                     XdataBase + address, &value, sizeof(value));
    if (error != UC_ERR_OK)
        throw std::runtime_error(std::string("Unicorn read XDATA: ") +
                                 uc_strerror(error));
    return value;
}

void Ca51F253L3::write_sfr(std::uint8_t address, std::uint8_t value) {
    sfr_[address] = value;
    if (address == SfrTimer0Control || address == SfrTimerMode)
        rebase_timer0();
}

} // namespace mcs51emu

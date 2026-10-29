#include "mcs51emu/cc2530_debug_programmer.hpp"

#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace mcs51emu {

namespace {

// Table 3-1 encodings (the three don't-care bits are 0).
constexpr std::uint8_t ChipErase = 0x10;
constexpr std::uint8_t WrConfig = 0x18;
constexpr std::uint8_t ReadStatus = 0x30;
constexpr std::uint8_t DebugInstr = 0x50;
constexpr std::uint8_t GetChipId = 0x68;
constexpr std::uint8_t BurstWrite = 0x80;
// Table 3-3 / Table 3-2.
constexpr std::uint8_t StatusChipEraseBusy = 0x80;
constexpr std::uint8_t StatusDebugLocked = 0x04;
// SOFT_POWER_MODE | TIMER_SUSPEND, DMA_PAUSE clear: the DMA must run.
constexpr std::uint8_t Config = 0x22;

// SFRs and XREGs (SWRU191F 2.2.3, 4.4.4, 6.5, 8.8).
constexpr std::uint8_t SfrDmairq = 0xd1;
constexpr std::uint8_t SfrDma1cfgl = 0xd2;
constexpr std::uint8_t SfrDma1cfgh = 0xd3;
constexpr std::uint8_t SfrDma0cfgl = 0xd4;
constexpr std::uint8_t SfrDma0cfgh = 0xd5;
constexpr std::uint8_t SfrDmaarm = 0xd6;
constexpr std::uint8_t SfrMemctr = 0xc7;
constexpr std::uint8_t SfrClkconcmd = 0xc6;
constexpr std::uint8_t SfrClkconsta = 0x9e;
constexpr std::uint16_t Dbgdata = 0x6260;
constexpr std::uint16_t Fctl = 0x6270;
constexpr std::uint16_t Faddrl = 0x6271;
constexpr std::uint16_t Fwdata = 0x6273;
constexpr std::uint8_t FctlBusy = 0x80;
constexpr std::uint8_t FctlAbort = 0x20;
constexpr std::uint8_t FctlWriteCm01 = 0x06; // CM = 01 (reset value), WRITE

// Programmer memory plan (SRAM, below the DATA alias at 0x1f00): one page
// buffer and the two DMA descriptors.
constexpr std::uint16_t Buffer = 0x0000;
constexpr std::uint16_t Descriptor0 = 0x0800;
constexpr std::uint16_t Descriptor1 = 0x0808;

constexpr std::size_t PageSize = 2048; // 6.1
constexpr std::size_t PagesPerBank = 16;
constexpr std::uint64_t ChipEraseCycles = 640000;              // 20 ms (6, 2.2.3)
constexpr std::uint64_t PageWriteCycles = PageSize / 4 * 640;  // 512 words x 20 us
constexpr unsigned MaxPolls = 1000;

std::string hex(unsigned value, int width) {
    std::ostringstream text;
    text << "0x" << std::hex << std::setw(width) << std::setfill('0') << value;
    return text.str();
}

class Programmer {
public:
    Programmer(const Cc2530DebugIfLink &link, Cc2530DebugProgramResult &result)
        : link_(link), result_(result) {}

    Cc2530DebugIfResponse send(const std::vector<std::uint8_t> &bytes, const char *what) {
        ++result_.commands;
        Cc2530DebugIfResponse response = link_.command(bytes);
        if (result_.commands == 1)
            result_.start_cycle = response.cycle;
        result_.end_cycle = response.cycle;
        if (!response.accepted)
            throw std::runtime_error(std::string("CC2530 debug programming: ") + what +
                                     " refused (status " +
                                     hex(response.bytes.empty() ? 0 : response.bytes[0], 2) +
                                     ")");
        return response;
    }

    std::uint8_t status() { return send({ReadStatus}, "READ_STATUS").bytes.at(0); }

    std::uint8_t instr(std::initializer_list<std::uint8_t> code) {
        std::vector<std::uint8_t> bytes{static_cast<std::uint8_t>(DebugInstr | code.size())};
        bytes.insert(bytes.end(), code.begin(), code.end());
        return send(bytes, "DEBUG_INSTR").bytes.at(0);
    }

    void write_sfr(std::uint8_t sfr, std::uint8_t value) { instr({0x75, sfr, value}); } // MOV dir,#
    std::uint8_t read_sfr(std::uint8_t sfr) { return instr({0xe5, sfr}); }             // MOV A,dir
    void set_dptr(std::uint16_t address) {                                              // MOV DPTR,#
        instr({0x90, static_cast<std::uint8_t>(address >> 8), static_cast<std::uint8_t>(address)});
    }
    std::uint8_t movx_read() { return instr({0xe0}); } // MOVX A,@DPTR
    void movx_write(std::uint8_t value) {
        instr({0x74, value}); // MOV A,#
        instr({0xf0});        // MOVX @DPTR,A
    }
    void inc_dptr() { instr({0xa3}); }

    void write_xdata(std::uint16_t address, const std::vector<std::uint8_t> &bytes) {
        set_dptr(address);
        for (std::size_t i = 0; i < bytes.size(); ++i) {
            movx_write(bytes[i]);
            if (i + 1 != bytes.size())
                inc_dptr();
        }
    }

    void wait(std::uint64_t cycles) {
        if (link_.wait)
            link_.wait(cycles);
    }

private:
    const Cc2530DebugIfLink &link_;
    Cc2530DebugProgramResult &result_;
};

std::vector<std::uint8_t> descriptor(std::uint16_t source, std::uint16_t destination,
                                     std::uint8_t trigger, std::uint8_t flags) {
    // SWRU191F Table 8-2: SRC, DEST, VLEN/LEN = 2048 (VLEN 000: LEN
    // transfers), WORDSIZE 0 / TMODE 00 (single) / TRIG, then
    // SRCINC/DESTINC/IRQMASK/M8/PRIORITY.
    return {static_cast<std::uint8_t>(source >> 8), static_cast<std::uint8_t>(source),
            static_cast<std::uint8_t>(destination >> 8), static_cast<std::uint8_t>(destination),
            static_cast<std::uint8_t>(PageSize >> 8), static_cast<std::uint8_t>(PageSize & 0xff),
            trigger, flags};
}

} // namespace

Cc2530DebugIfLink cc2530_debug_if_link(Cc2530F256 &chip) {
    Cc2530DebugIfLink link;
    link.enter = [&chip] { chip.debug_if_enter(); };
    link.command = [&chip](const std::vector<std::uint8_t> &bytes) {
        return chip.debug_if_command(bytes);
    };
    link.wait = [&chip](std::uint64_t cycles) {
        const Cc2530RunResult result = chip.run(SIZE_MAX, chip.registers().cycles + cycles);
        if (!result.error.empty())
            throw std::runtime_error("CC2530 debug programming: " + result.error);
    };
    link.reset = [&chip] {
        chip.set_reset_pin(false);
        chip.set_reset_pin(true);
    };
    return link;
}

Cc2530DebugProgramResult cc2530_debug_program(const Cc2530DebugIfLink &link,
                                              const Cc2530FlashImage &image,
                                              const Cc2530DebugProgramOptions &options) {
    if (!link.enter || !link.command || (options.run && !link.reset))
        throw std::invalid_argument("CC2530 debug programming: incomplete debug link");
    if (image.bytes.size() != Cc2530F256::FlashSize)
        throw std::invalid_argument("CC2530 debug programming: the image must cover 256 KiB");
    std::vector<std::uint8_t> flash = image.bytes;
    if (options.debug_lock)
        flash.back() &= 0x7f; // 3.4.1 / Table 3-5: DBGLOCK = 0 locks

    Cc2530DebugProgramResult result;
    Programmer chip(link, result);

    // 1. CHIP_ERASE must be the first command after entering debug mode
    // (Table 3-1); it is accepted on a locked chip (3.4.1).
    link.enter();
    result.was_locked = (chip.status() & StatusDebugLocked) != 0;
    chip.send({ChipErase}, "CHIP_ERASE");
    chip.wait(ChipEraseCycles);
    unsigned polls = 0;
    while (chip.status() & StatusChipEraseBusy) {
        if (++polls == MaxPolls)
            throw std::runtime_error("CC2530 debug programming: CHIP_ERASE does not finish");
        chip.wait(ChipEraseCycles / 20);
    }

    // 2. "After the debug-lock bit has changed ... the device must be reset
    // to lock or unlock the debug interface" (3.4.1).
    link.enter();
    if (chip.status() & StatusDebugLocked)
        throw std::runtime_error("CC2530 debug programming: still locked after CHIP_ERASE");
    const Cc2530DebugIfResponse id = chip.send({GetChipId}, "GET_CHIP_ID");
    result.chip_id = id.bytes.at(0);
    result.chip_version = id.bytes.at(1);
    if (result.chip_id != 0xa5)
        throw std::runtime_error("CC2530 debug programming: chip ID " + hex(result.chip_id, 2) +
                                 " is not a CC2530 (0xa5)");
    chip.send({WrConfig, Config}, "WR_CONFIG");

    // 3. 32 MHz crystal oscillator as system clock (the debugger cannot be
    // used with a divided clock, 3.1 note), as TI's programmers do.
    chip.write_sfr(SfrClkconcmd, 0x80);
    for (polls = 0; chip.read_sfr(SfrClkconsta) != 0x80;)
        if (++polls == MaxPolls)
            throw std::runtime_error("CC2530 debug programming: the clock does not switch");

    // 4. DMA channel 0: DBGDATA -> buffer on DBG_BW (trigger 31, Table
    // 8-1), destination incremented; channel 1: buffer -> FWDATA on FLASH
    // (trigger 18) as in Figure 6-1 (single mode, byte size, SRCINC 1,
    // IRQMASK, high priority).
    std::vector<std::uint8_t> descriptors = descriptor(Dbgdata, Buffer, 31, 0x12);
    const std::vector<std::uint8_t> flash_descriptor = descriptor(Buffer, Fwdata, 18, 0x4a);
    descriptors.insert(descriptors.end(), flash_descriptor.begin(), flash_descriptor.end());
    chip.write_xdata(Descriptor0, descriptors);
    chip.write_sfr(SfrDma0cfgh, Descriptor0 >> 8);
    chip.write_sfr(SfrDma0cfgl, Descriptor0 & 0xff);
    chip.write_sfr(SfrDma1cfgh, Descriptor1 >> 8); // channel 1 = first DMA1CFG descriptor
    chip.write_sfr(SfrDma1cfgl, Descriptor1 & 0xff);
    chip.write_sfr(SfrDmairq, 0x00);

    std::vector<std::size_t> pages;
    for (std::size_t page = 0; page < Cc2530F256::FlashSize / PageSize; ++page) {
        const auto begin = flash.begin() + static_cast<std::ptrdiff_t>(page * PageSize);
        if (std::find_if(begin, begin + PageSize, [](std::uint8_t b) { return b != 0xff; }) !=
            begin + PageSize)
            pages.push_back(page);
    }

    // 5. One page at a time, ascending, so the lock-bit page comes last.
    for (const std::size_t page : pages) {
        const std::size_t address = page * PageSize;
        chip.write_sfr(SfrDmaarm, 0x01);
        std::vector<std::uint8_t> burst{BurstWrite, 0x00}; // length 0 = 2048 (Figure 3-5)
        burst.insert(burst.end(), flash.begin() + static_cast<std::ptrdiff_t>(address),
                     flash.begin() + static_cast<std::ptrdiff_t>(address + PageSize));
        chip.send(burst, "BURST_WRITE");
        for (polls = 0; !(chip.read_sfr(SfrDmairq) & 0x01u);)
            if (++polls == MaxPolls)
                throw std::runtime_error("CC2530 debug programming: DBG_BW DMA does not finish");
        chip.write_sfr(SfrDmairq, 0xfe);

        const std::size_t word = address / 4; // FADDRH:FADDRL is a word address (6.5)
        chip.write_xdata(Faddrl, {static_cast<std::uint8_t>(word & 0xff),
                                  static_cast<std::uint8_t>(word >> 8)});
        chip.write_sfr(SfrDmaarm, 0x02);
        chip.set_dptr(Fctl);
        chip.movx_write(FctlWriteCm01);
        chip.wait(PageWriteCycles);
        std::uint8_t fctl = 0;
        for (polls = 0; (fctl = chip.movx_read()) & FctlBusy;) {
            if (++polls == MaxPolls)
                throw std::runtime_error("CC2530 debug programming: flash write at " +
                                         hex(static_cast<unsigned>(address), 5) +
                                         " does not finish");
            chip.wait(PageWriteCycles / 64);
        }
        if (fctl & FctlAbort)
            throw std::runtime_error("CC2530 debug programming: flash write at " +
                                     hex(static_cast<unsigned>(address), 5) +
                                     " aborted (FCTL " + hex(fctl, 2) + ")");
        if (!(chip.read_sfr(SfrDmairq) & 0x02u))
            throw std::runtime_error("CC2530 debug programming: FLASH DMA at " +
                                     hex(static_cast<unsigned>(address), 5) +
                                     " did not complete");
        chip.write_sfr(SfrDmairq, 0xfd);
        ++result.pages;
    }

    // 6. Read-back through the XBANK window at XDATA 0x8000 (2.2.2).
    if (options.verify) {
        for (const std::size_t page : pages) {
            chip.write_sfr(SfrMemctr, static_cast<std::uint8_t>(page / PagesPerBank));
            chip.set_dptr(static_cast<std::uint16_t>(0x8000 + (page % PagesPerBank) * PageSize));
            for (std::size_t i = 0; i < PageSize; ++i) {
                const std::size_t address = page * PageSize + i;
                const std::uint8_t value = chip.movx_read();
                if (value != flash[address])
                    throw std::runtime_error("CC2530 debug programming: verify failed at " +
                                             hex(static_cast<unsigned>(address), 5) + ": read " +
                                             hex(value, 2) + ", expected " +
                                             hex(flash[address], 2));
                chip.inc_dptr();
                ++result.bytes_verified;
            }
        }
        chip.write_sfr(SfrMemctr, 0x00);
    }

    if (options.run)
        link.reset();
    return result;
}

} // namespace mcs51emu

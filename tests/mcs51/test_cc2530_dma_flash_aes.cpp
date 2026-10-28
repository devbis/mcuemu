// CC2530F256 DMA controller (SWRU191F Chapter 8), flash controller
// (Chapter 6, 2.2.3, 3.4.1) and AES coprocessor (Chapter 15) tests.
//
// Every scenario runs hand-encoded 8051 machine code: programs build DMA
// descriptors in XDATA, arm channels, write FCTL/FWDATA from SRAM code
// (MEMCTR.XMAP) and drive ENCCS/ENCDI/ENCDO. Expected values come from the
// documents: data moved per descriptor field (Table 8-2), the arming time of
// nine system clocks per channel (8.1), the flash timing of SWRU191F 2.2.3
// (page erase 20 ms, 4-byte write 20 us), FIPS-197 Appendix C.1 and NIST
// SP 800-38A F.2-F.5 vectors. Where the documents give no number, the test
// states the emulator's model-defined rule it relies on (see the model
// sources): one DMA byte transfer takes 2 clocks and a word transfer 4,
// AES processing ends 40 clocks after the last input byte. CPU instruction
// times are SWRU191F Table 2-3 (cycles_of); a CPU write takes effect at the
// start clock of the writing instruction, and programs pad with 1-clock
// NOPs so that their writes land at the offsets the expectations use.

#include "cc2530_dma.hpp"
#include "cc2530_periph_support.hpp"

#include <cstdint>
#include <iomanip>
#include <optional>
#include <source_location>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cc2530test;

namespace {

// ---- registers ----
constexpr std::uint8_t SP = 0x81;
constexpr std::uint8_t PCON = 0x87;
constexpr std::uint8_t S0CON = 0x98;
constexpr std::uint8_t FMAP = 0x9f;
constexpr std::uint8_t IEN0 = 0xa8;
constexpr std::uint8_t ENCDI = 0xb1;
constexpr std::uint8_t ENCDO = 0xb2;
constexpr std::uint8_t ENCCS = 0xb3;
constexpr std::uint8_t IEN1 = 0xb8;
constexpr std::uint8_t SLEEPCMD = 0xbe;
constexpr std::uint8_t IRCON = 0xc0;
constexpr std::uint8_t MEMCTR = 0xc7;
constexpr std::uint8_t DMAIRQ = 0xd1;
constexpr std::uint8_t DMA1CFGL = 0xd2;
constexpr std::uint8_t DMA1CFGH = 0xd3;
constexpr std::uint8_t DMA0CFGL = 0xd4;
constexpr std::uint8_t DMA0CFGH = 0xd5;
constexpr std::uint8_t DMAARM = 0xd6;
constexpr std::uint8_t DMAREQ = 0xd7;
constexpr std::uint8_t SfrB = 0xf0;

constexpr std::uint16_t FCTL = 0x6270;
constexpr std::uint16_t FADDRL = 0x6271;
constexpr std::uint16_t FADDRH = 0x6272;
constexpr std::uint16_t FWDATA = 0x6273;

constexpr std::uint16_t IdataInXdata = 0x1f00; // SWRU191F 2.2.1 / Figure 2-1
constexpr std::uint16_t Desc0 = 0x0f00;        // channel 0 descriptor
constexpr std::uint16_t Desc1 = 0x0f10;        // channels 1-4 descriptors
constexpr std::uint16_t SramCode = 0x9000;     // CODE address of SRAM 0x1000 (XMAP)

// IDATA used by the interrupt service routines.
constexpr std::uint8_t DmaIsrCount = 0x70;
constexpr std::uint8_t DmaIsrFlags = 0x71;
constexpr std::uint8_t EncIsrCount = 0x72;
constexpr std::uint8_t EncBlocksLeft = 0x73;
constexpr std::uint8_t EncNextCommand = 0x74;

// AES subroutines (fixed addresses in flash, see base_program()).
constexpr std::uint16_t AesIn = 0x6000;
constexpr std::uint16_t AesOut = 0x6020;
constexpr std::uint16_t AesBlock = 0x6040;

// ENCCS commands (SWRU191F 15.10): MODE[6:4], CMD[2:1], ST[0].
constexpr std::uint8_t ModeCbc = 0, ModeCfb = 1, ModeOfb = 2, ModeCtr = 3, ModeEcb = 4,
                       ModeCbcMac = 5;
constexpr std::uint8_t enc_encrypt(std::uint8_t mode) { return static_cast<std::uint8_t>(mode << 4 | 0x01); }
constexpr std::uint8_t enc_decrypt(std::uint8_t mode) { return static_cast<std::uint8_t>(mode << 4 | 0x03); }
constexpr std::uint8_t EncLoadKey = 0x05;
constexpr std::uint8_t enc_load_iv(std::uint8_t mode) { return static_cast<std::uint8_t>(mode << 4 | 0x07); }

// DMA descriptor fields (Table 8-2).
constexpr std::uint8_t Single = 0, Block = 1, RepeatedSingle = 2, RepeatedBlock = 3;
constexpr std::uint8_t Inc0 = 0, Inc1 = 1, Inc2 = 2, IncMinus1 = 3;
constexpr std::uint8_t PrioLow = 0, PrioAssured = 1, PrioHigh = 2;
constexpr std::uint8_t TrigNone = 0, TrigPrev = 1, TrigFlash = 18, TrigEncDw = 29,
                       TrigEncUp = 30;

struct Desc {
    std::uint16_t src = 0;
    std::uint16_t dst = 0;
    std::uint8_t vlen = 0;
    std::uint16_t len = 1;
    bool word = false;
    std::uint8_t tmode = Block;
    std::uint8_t trig = TrigNone;
    std::uint8_t srcinc = Inc1;
    std::uint8_t destinc = Inc1;
    bool irqmask = false;
    bool m8 = false;
    std::uint8_t priority = PrioLow;

    std::vector<std::uint8_t> bytes() const {
        return {hi(src),
                lo(src),
                hi(dst),
                lo(dst),
                static_cast<std::uint8_t>(vlen << 5 | (len >> 8 & 0x1f)),
                lo(len),
                static_cast<std::uint8_t>((word ? 0x80 : 0) | tmode << 5 | trig),
                static_cast<std::uint8_t>(srcinc << 6 | destinc << 4 | (irqmask ? 0x08 : 0) |
                                          (m8 ? 0x04 : 0) | priority)};
    }
};

Desc desc(std::uint16_t src, std::uint16_t dst, std::uint16_t len) {
    Desc d;
    d.src = src;
    d.dst = dst;
    d.len = len;
    return d;
}

std::uint16_t desc_address(unsigned channel) {
    return channel == 0 ? Desc0 : static_cast<std::uint16_t>(Desc1 + 8 * (channel - 1));
}

// ---- program builder ----
struct Asm {
    std::uint16_t origin;
    std::vector<std::uint8_t> code;

    explicit Asm(std::uint16_t at) : origin(at) {}
    std::uint16_t here() const { return static_cast<std::uint16_t>(origin + code.size()); }
    Asm &op(std::initializer_list<std::uint8_t> bytes) {
        code.insert(code.end(), bytes);
        return *this;
    }
    Asm &op(const std::vector<std::uint8_t> &bytes) {
        code.insert(code.end(), bytes.begin(), bytes.end());
        return *this;
    }
    // MOV direct,#imm (Table 2-3: 3 clocks)
    Asm &mov(std::uint8_t dir, std::uint8_t value) { return op({0x75, dir, value}); }
    // MOV direct2,direct1 (Table 2-3: 4 clocks)
    Asm &copy(std::uint8_t to, std::uint8_t from) { return op({0x85, from, to}); }
    // MOV Rn,#imm
    Asm &movr(unsigned r, std::uint8_t value) {
        return op({static_cast<std::uint8_t>(0x78 + r), value});
    }
    Asm &dptr(std::uint16_t address) { return op(mov_dptr(address)); }
    // MOV DPTR,#address; MOV A,#value; MOVX @DPTR,A
    Asm &xwrite(std::uint16_t address, std::uint8_t value) { return op(movx_write(address, value)); }
    Asm &xread_to(std::uint16_t address, std::uint8_t dir) { return op(movx_read_to(address, dir)); }
    // Stores `bytes` at XDATA `address` with MOVX @DPTR,A / INC DPTR.
    Asm &xstore(std::uint16_t address, const std::vector<std::uint8_t> &bytes) {
        dptr(address);
        for (const std::uint8_t byte : bytes)
            op({0x74, byte, 0xf0, 0xa3});
        return *this;
    }
    Asm &descriptor(unsigned channel, const Desc &d) { return xstore(desc_address(channel), d.bytes()); }
    // DMA0CFG -> Desc0, DMA1CFG -> Desc1.
    Asm &point_descriptors() {
        return mov(DMA0CFGH, hi(Desc0)).mov(DMA0CFGL, lo(Desc0)).mov(DMA1CFGH, hi(Desc1)).mov(DMA1CFGL, lo(Desc1));
    }
    Asm &nops(unsigned count) {
        code.insert(code.end(), count, 0x00);
        return *this;
    }
    Asm &halt() { return op({0x80, 0xfe}); } // SJMP $
    Asm &lcall(std::uint16_t address) { return op({0x12, hi(address), lo(address)}); }
    Asm &ljmp(std::uint16_t address) { return op({0x02, hi(address), lo(address)}); }
    std::uint8_t rel(std::uint16_t target, unsigned length) const {
        const int offset = static_cast<int>(target) - static_cast<int>(here() + length);
        if (offset < -128 || offset > 127)
            throw std::logic_error("branch out of range");
        return static_cast<std::uint8_t>(offset);
    }
    Asm &djnz(unsigned r, std::uint16_t target) {
        const std::uint8_t offset = rel(target, 2);
        return op({static_cast<std::uint8_t>(0xd8 + r), offset});
    }
    Asm &sjmp(std::uint16_t target) {
        const std::uint8_t offset = rel(target, 2);
        return op({0x80, offset});
    }
    // MOV R5,#outer; L1: MOV R6,#inner; L2: MOV R7,#0; DJNZ R7,$;
    // DJNZ R6,L2; DJNZ R5,L1: delay_cycles(outer, inner) clocks.
    Asm &delay(std::uint8_t outer, std::uint8_t inner) {
        movr(5, outer);
        const std::uint16_t l1 = here();
        movr(6, inner);
        const std::uint16_t l2 = here();
        movr(7, 0);
        djnz(7, here());
        djnz(6, l2);
        return djnz(5, l1);
    }
    // MOV DPTR,#FWDATA-style byte stream: MOV A,#b; MOVX @DPTR,A per byte.
    Asm &xbytes(const std::vector<std::uint8_t> &bytes) {
        for (const std::uint8_t byte : bytes)
            op({0x74, byte, 0xf0});
        return *this;
    }
};

// SWRU191F Table 2-3 instruction times the expectations are built from.
constexpr unsigned MovDirImm = cycles_of({0x75}); // MOV direct,#data: 3
constexpr unsigned MovxWrite = cycles_of({0xf0}); // MOVX @DPTR,A: 4

// Asm::delay(): MOV Rn,#data 2 and DJNZ Rn,rel 3 clocks (Table 2-3), so
// 2 + outer * (5 + 773 * inner) clocks.
constexpr std::uint64_t delay_cycles(unsigned outer, unsigned inner) {
    const std::uint64_t mov = cycles_of({0x7d}), djnz = cycles_of({0xdf});
    return mov + outer * (mov + inner * (mov + 256 * djnz + djnz) + djnz);
}

void place(Flash &flash, const Asm &code) { flash.at(code.origin).op(code.code); }

// Copies SRAM code assembled for CODE 0x8000+ (MEMCTR.XMAP) into SRAM.
void place_sram(Cc2530F256 &device, const Asm &code) {
    for (std::size_t i = 0; i < code.code.size(); ++i)
        device.write_sram(static_cast<std::uint16_t>(code.origin - 0x8000 + i), code.code[i]);
}

// Interrupt vectors and service routines shared by all programs, the AES
// subroutines and `main` (origin 0x0100).
Flash base_program(const Asm &main) {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00}); // LJMP 0x0100
    flash.at(0x0023).op({0x02, 0x00, 0x60}); // ENC  -> 0x0060
    flash.at(0x002b).op({0xc2, 0xc7, 0x32}); // ST: CLR IRCON.STIF; RETI
    flash.at(0x0043).op({0x02, 0x00, 0x80}); // DMA  -> 0x0080
    // ENC: count, clear S0CON.ENCIF_1/ENCIF_0 (SWRU191F 15.8: not cleared by
    // hardware), and start the next block with ENCCS = [EncNextCommand]
    // while [EncBlocksLeft] != 0.
    flash.at(0x0060).op({0xc0, 0xe0,                   // PUSH ACC
                         0x05, EncIsrCount,            // INC count
                         0xc2, 0x98, 0xc2, 0x99,       // CLR ENCIF_0, CLR ENCIF_1
                         0xe5, EncBlocksLeft,          // MOV A,left
                         0x60, 0x05,                   // JZ +5
                         0x15, EncBlocksLeft,          // DEC left
                         0x85, EncNextCommand, ENCCS,  // MOV ENCCS,next
                         0xd0, 0xe0,                   // POP ACC
                         0x32});                       // RETI
    // DMA: clear IRCON.DMAIF (not cleared by hardware), count, OR the
    // DMAIRQ flags seen into DmaIsrFlags and clear exactly those (R/W0).
    flash.at(0x0080).op({0xc0, 0xe0,         // PUSH ACC
                         0xc2, 0xc0,         // CLR DMAIF
                         0x05, DmaIsrCount,  // INC count
                         0xe5, DMAIRQ,       // MOV A,DMAIRQ
                         0x42, DmaIsrFlags,  // ORL flags,A
                         0xf4,               // CPL A
                         0x52, DMAIRQ,       // ANL DMAIRQ,A
                         0xd0, 0xe0,         // POP ACC
                         0x32});             // RETI

    // aes_in: R2 bytes from @R0 to ENCDI, then wait 92 clocks (MOV Rn,#data
    // 2 + 30 x DJNZ 3, Table 2-3; > the model-defined AES latency of 40).
    Asm in(AesIn);
    in.op({0xea, 0xfc}); // MOV A,R2; MOV R4,A
    const std::uint16_t in_loop = in.here();
    in.op({0x86, ENCDI, 0x08}).djnz(4, in_loop); // MOV ENCDI,@R0; INC R0
    in.op({0x7f, 30, 0xdf, 0xfe, 0x22});         // MOV R7,#30; DJNZ R7,$; RET
    place(flash, in);
    // aes_out: R2 bytes from ENCDO to @R1.
    Asm out(AesOut);
    out.op({0xea, 0xfc});
    const std::uint16_t out_loop = out.here();
    out.op({0xa7, ENCDO, 0x09}).djnz(4, out_loop).op({0x22}); // MOV @R1,ENCDO; INC R1
    place(flash, out);
    // aes_block: R3 chunks of R2 bytes, each written, processed and read.
    Asm block(AesBlock);
    const std::uint16_t block_loop = block.here();
    block.lcall(AesIn).lcall(AesOut).djnz(3, block_loop).op({0x22});
    place(flash, block);

    place(flash, main);
    return flash;
}

// ---- running and checking ----
void fail(const std::source_location &loc, const std::string &message) {
    std::cerr << loc.file_name() << ':' << loc.line() << ": " << message << '\n';
    ++failures;
}

std::string hex(const std::vector<std::uint8_t> &bytes) {
    std::ostringstream out;
    for (const std::uint8_t byte : bytes)
        out << std::hex << std::setw(2) << std::setfill('0') << +byte;
    return out.str();
}

std::vector<std::uint8_t> bytes_of(const std::string &text) {
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i + 1 < text.size(); i += 2)
        bytes.push_back(static_cast<std::uint8_t>(std::stoul(text.substr(i, 2), nullptr, 16)));
    return bytes;
}

void expect_bytes(const std::vector<std::uint8_t> &actual, const std::vector<std::uint8_t> &expected,
                  const std::source_location loc = std::source_location::current()) {
    if (actual != expected)
        fail(loc, "got " + hex(actual) + ", want " + hex(expected));
}

void expect(std::uint64_t actual, std::uint64_t expected, const char *what,
            const std::source_location loc = std::source_location::current()) {
    if (actual != expected) {
        std::ostringstream out;
        out << what << ": got 0x" << std::hex << actual << ", want 0x" << expected;
        fail(loc, out.str());
    }
}

std::vector<std::uint8_t> xdata(const Cc2530F256 &device, std::uint16_t address, std::size_t count) {
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i < count; ++i)
        bytes.push_back(device.peek_xdata(static_cast<std::uint16_t>(address + i)));
    return bytes;
}

std::vector<std::uint8_t> idata(const Cc2530F256 &device, std::uint8_t address, std::size_t count) {
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i < count; ++i)
        bytes.push_back(device.read_idata(static_cast<std::uint8_t>(address + i)));
    return bytes;
}

std::vector<std::uint8_t> flash_bytes(const Cc2530F256 &device, std::uint32_t address, std::size_t count) {
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i < count; ++i)
        bytes.push_back(device.read_flash(static_cast<std::uint32_t>(address + i)));
    return bytes;
}

void write_xdata(Cc2530F256 &device, std::uint16_t address, const std::vector<std::uint8_t> &bytes) {
    for (std::size_t i = 0; i < bytes.size(); ++i)
        device.write_sram(static_cast<std::uint16_t>(address + i), bytes[i]);
}

void write_idata(Cc2530F256 &device, std::uint8_t address, const std::vector<std::uint8_t> &bytes) {
    write_xdata(device, static_cast<std::uint16_t>(IdataInXdata + address), bytes);
}

// Runs until the CPU is about to execute the instruction at `pc`; returns
// the cycle counter there (the start time of that instruction).
std::uint64_t run_to_pc(Cc2530F256 &device, std::uint16_t pc,
                        const std::source_location loc = std::source_location::current()) {
    device.set_breakpoint(pc);
    const Cc2530RunResult result = device.run(20'000'000);
    device.set_breakpoint(std::nullopt);
    if (!result.error.empty())
        fail(loc, "run error: " + result.error);
    if (!result.break_hit || result.pc != pc)
        fail(loc, "breakpoint not reached");
    return device.registers().cycles;
}

// Runs to exactly `cycle` (the program must be at a NOP sled there).
void run_to(Cc2530F256 &device, std::uint64_t cycle,
            const std::source_location loc = std::source_location::current()) {
    const std::uint64_t now = device.registers().cycles;
    if (cycle <= now) {
        fail(loc, "run_to: cycle " + std::to_string(cycle) + " already reached");
        return;
    }
    const Cc2530RunResult result = device.run(20'000'000, cycle);
    if (!result.error.empty())
        fail(loc, "run error: " + result.error);
    if (device.registers().cycles != cycle)
        fail(loc, "stopped at cycle " + std::to_string(device.registers().cycles) + " instead of " +
                      std::to_string(cycle));
}

// Runs for about `cycles` more clocks (no exactness required).
void settle(Cc2530F256 &device, std::uint64_t cycles,
            const std::source_location loc = std::source_location::current()) {
    const Cc2530RunResult result = device.run(20'000'000, device.registers().cycles + cycles);
    if (!result.error.empty())
        fail(loc, "run error: " + result.error);
}

mcs51emu::cc2530::Dma &dma(Cc2530F256 &device) { return *device.peripherals().dma; }

// ======================================================================
// DMA (SWRU191F Chapter 8)
// ======================================================================

// 8.1: arming one channel takes nine system clocks; a DMAREQ written during
// that time is kept (it is a register bit, cleared only when the transfer
// starts) and served when the channel is ready. Block transfer, LEN = 4,
// one byte per 2 clocks (model-defined). Non-repetitive: DMAARM clears and
// DMAIRQ.DMAIF0 is set on completion (8.8); IRQMASK = 0 keeps IRCON.DMAIF 0.
void test_dma_manual_block_timing() {
    Asm main(0x0100);
    main.mov(SP, 0x30).xstore(0x0200, {0x11, 0x22, 0x33, 0x44, 0x55});
    main.descriptor(0, desc(0x0200, 0x0300, 4)).point_descriptors();
    const std::uint16_t arm = main.here();
    main.mov(DMAARM, 0x01).mov(DMAREQ, 0x01).nops(64).halt();

    Cc2530F256 device;
    load(device, base_program(main));
    const std::uint64_t t = run_to_pc(device, arm);
    run_to(device, t + MovDirImm); // DMAREQ written at +3
    expect(device.read_sfr(DMAARM), 0x01, "DMAARM while configuring");
    run_to(device, t + 8);
    expect(device.read_sfr(DMAREQ), 0x01, "DMAREQ kept during the 9-clock fetch");
    run_to(device, t + 9);
    expect(device.read_sfr(DMAREQ), 0x00, "DMAREQ cleared when the transfer starts");
    run_to(device, t + 10);
    expect(device.peek_xdata(0x0300), 0x00, "no byte before the first slot ends");
    run_to(device, t + 11);
    expect_bytes(xdata(device, 0x0300, 2), {0x11, 0x00});
    run_to(device, t + 13);
    expect_bytes(xdata(device, 0x0300, 3), {0x11, 0x22, 0x00});
    run_to(device, t + 15);
    expect_bytes(xdata(device, 0x0300, 4), {0x11, 0x22, 0x33, 0x00});
    run_to(device, t + 16);
    expect(device.read_sfr(DMAIRQ), 0x00, "DMAIRQ before completion");
    expect(device.read_sfr(DMAARM), 0x01, "DMAARM before completion");
    run_to(device, t + 17);
    expect_bytes(xdata(device, 0x0300, 5), {0x11, 0x22, 0x33, 0x44, 0x00});
    expect(device.read_sfr(DMAIRQ), 0x01, "DMAIRQ.DMAIF0");
    expect(device.read_sfr(DMAARM), 0x00, "DMAARM cleared after a block transfer");
    expect(device.read_sfr(IRCON) & 0x01, 0, "IRCON.DMAIF with IRQMASK = 0");
}

// 8.1 trigger rules with an external trigger source (white-box trigger()):
// - a trigger within the 9-clock arming time of a fresh channel is lost;
// - single mode moves one byte per trigger, triggers arriving while the
//   channel is busy are counted ("missed events") and served in turn;
// - after reconfiguration, a trigger of the previously configured source
//   during the fetch is a missed event served when the channel is ready
//   "even though the new trigger source is not the same", while a trigger
//   of the new source within the fetch time is lost;
// - writing DMAREQ clears missed-trigger counting.
void test_dma_trigger_rules() {
    Asm main(0x0100);
    main.mov(SP, 0x30);
    Desc first = desc(0x0200, 0x0300, 3);
    first.tmode = Single;
    first.trig = 2;
    main.descriptor(0, first).point_descriptors();
    const std::uint16_t arm1 = main.here();
    main.mov(DMAARM, 0x01).nops(60);
    Desc second = desc(0x0210, 0x0310, 2);
    second.tmode = Single;
    second.trig = 3;
    main.descriptor(0, second);
    const std::uint16_t arm2 = main.here();
    main.mov(DMAARM, 0x01).nops(60);
    Desc third = desc(0x0220, 0x0320, 4);
    third.tmode = Single;
    third.trig = 3;
    main.descriptor(0, third);
    const std::uint16_t arm3 = main.here();
    main.mov(DMAARM, 0x01).nops(60).halt();

    Cc2530F256 device;
    load(device, base_program(main));
    write_xdata(device, 0x0200, {0xa0, 0xa1, 0xa2});
    write_xdata(device, 0x0210, {0xb0, 0xb1});
    write_xdata(device, 0x0220, {0xc0, 0xc1, 0xc2, 0xc3});

    const std::uint64_t t = run_to_pc(device, arm1);
    run_to(device, t + 4);
    dma(device).trigger(2); // during the arming time: lost
    run_to(device, t + 20);
    expect(device.peek_xdata(0x0300), 0x00, "trigger during arming is lost");
    dma(device).trigger(2);
    run_to(device, t + 21);
    expect(device.peek_xdata(0x0300), 0x00, "slot in progress");
    run_to(device, t + 22);
    expect(device.peek_xdata(0x0300), 0xa0, "single mode: one byte per trigger");
    expect(device.peek_xdata(0x0301), 0x00, "single mode: only one byte");
    dma(device).trigger(2);
    dma(device).trigger(2);
    dma(device).trigger(0); // trigger 0 is never an external event
    run_to(device, t + 24);
    expect_bytes(xdata(device, 0x0300, 3), {0xa0, 0xa1, 0x00});
    run_to(device, t + 26);
    expect_bytes(xdata(device, 0x0300, 4), {0xa0, 0xa1, 0xa2, 0x00});
    expect(device.read_sfr(DMAARM), 0x00, "LEN reached: disarmed");
    expect(device.read_sfr(DMAIRQ), 0x01, "DMAIF0");

    const std::uint64_t tb = run_to_pc(device, arm2);
    run_to(device, tb + 3);
    dma(device).trigger(2); // previous source during the fetch: missed event
    run_to(device, tb + 5);
    dma(device).trigger(3); // new source during the fetch: lost
    run_to(device, tb + 10);
    expect(device.peek_xdata(0x0310), 0x00, "fetch ends at +9, slot ends at +11");
    run_to(device, tb + 11);
    expect(device.peek_xdata(0x0310), 0xb0, "missed old-source trigger served at ready");
    run_to(device, tb + 20);
    expect(device.peek_xdata(0x0311), 0x00, "new-source trigger during the fetch was lost");
    dma(device).trigger(2); // no longer the configured source
    run_to(device, tb + 24);
    expect(device.peek_xdata(0x0311), 0x00, "old source ignored after the fetch");
    dma(device).trigger(3);
    run_to(device, tb + 25);
    expect(device.peek_xdata(0x0311), 0x00, "slot in progress");
    run_to(device, tb + 26);
    expect(device.peek_xdata(0x0311), 0xb1, "new source after the fetch");
    expect(device.read_sfr(DMAARM), 0x00, "second configuration complete");

    const std::uint64_t tc = run_to_pc(device, arm3);
    run_to(device, tc + 20);
    dma(device).trigger(3);
    dma(device).trigger(3);
    dma(device).trigger(3); // one transfer starts, two are counted
    run_to(device, tc + 21);
    device.write_sfr(DMAREQ, 0x01); // clears the count, requests one transfer
    run_to(device, tc + 40);
    expect_bytes(xdata(device, 0x0320, 4), {0xc0, 0xc1, 0x00, 0x00});
    expect(device.read_sfr(DMAARM), 0x01, "two of four transfers done");
    expect(device.read_sfr(DMAREQ), 0x00, "DMAREQ served");
}

// Table 8-2 VLEN with byte transfers: the first byte is the length n;
// 001 -> n + 1, 010 -> n, 011 -> n + 2, 100 -> n + 3 transfers, 111 -> LEN.
void test_dma_vlen_modes() {
    {
        Asm main(0x0100);
        main.mov(SP, 0x30);
        const std::uint8_t vlen[5] = {1, 2, 3, 4, 7};
        for (unsigned c = 0; c < 5; ++c) {
            Desc d = desc(static_cast<std::uint16_t>(0x0200 + 0x20 * c),
                          static_cast<std::uint16_t>(0x0300 + 0x20 * c), c == 4 ? 4 : 20);
            d.vlen = vlen[c];
            main.descriptor(c, d);
        }
        main.point_descriptors().mov(DMAARM, 0x1f).mov(DMAREQ, 0x1f).nops(200);
        const std::uint16_t end = main.here();
        main.halt();

        Cc2530F256 device;
        load(device, base_program(main));
        for (unsigned c = 0; c < 5; ++c) {
            std::vector<std::uint8_t> data{5};
            for (unsigned i = 1; i < 20; ++i)
                data.push_back(static_cast<std::uint8_t>(0x40 * (c + 1) + i));
            write_xdata(device, static_cast<std::uint16_t>(0x0200 + 0x20 * c), data);
        }
        run_to_pc(device, end);
        const unsigned count[5] = {6, 5, 7, 8, 4};
        for (unsigned c = 0; c < 5; ++c) {
            const auto src = static_cast<std::uint16_t>(0x0200 + 0x20 * c);
            const auto dst = static_cast<std::uint16_t>(0x0300 + 0x20 * c);
            std::vector<std::uint8_t> want = xdata(device, src, count[c]);
            want.push_back(0);
            expect_bytes(xdata(device, dst, count[c] + 1), want);
        }
        expect(device.read_sfr(DMAIRQ), 0x1f, "all five channels complete");
        expect(device.read_sfr(DMAARM), 0x00, "all disarmed");
    }
    {
        // M8 (8.2.11: 7 bits of the length byte), the LEN maximum, word
        // mode (first word bits 12:0 = n; model-defined: the low byte is at
        // the lower address), VLEN 010 with n = 0 (model-defined: the
        // length byte itself is still transferred), VLEN 100 with M8.
        Asm main(0x0100);
        main.mov(SP, 0x30);
        Desc d0 = desc(0x0200, 0x0300, 20);
        d0.vlen = 1;
        d0.m8 = true;
        Desc d1 = desc(0x0220, 0x0320, 10);
        d1.vlen = 1;
        Desc d2 = desc(0x0240, 0x0340, 20);
        d2.vlen = 1;
        d2.word = true;
        Desc d3 = desc(0x0260, 0x0360, 20);
        d3.vlen = 2;
        Desc d4 = desc(0x0280, 0x0380, 6);
        d4.vlen = 4;
        d4.m8 = true;
        main.descriptor(0, d0).descriptor(1, d1).descriptor(2, d2).descriptor(3, d3).descriptor(4, d4);
        main.point_descriptors().mov(DMAARM, 0x1f).mov(DMAREQ, 0x1f).nops(200);
        const std::uint16_t end = main.here();
        main.halt();

        Cc2530F256 device;
        load(device, base_program(main));
        const std::uint8_t first[5] = {0x85, 0x20, 0x03, 0x00, 0xff};
        for (unsigned c = 0; c < 5; ++c) {
            std::vector<std::uint8_t> data{first[c]};
            for (unsigned i = 1; i < 24; ++i)
                data.push_back(static_cast<std::uint8_t>(0x40 * (c + 1) + i));
            if (c == 2)
                data[1] = 0xe0; // bits 15:13 of the length word are ignored
            write_xdata(device, static_cast<std::uint16_t>(0x0200 + 0x20 * c), data);
        }
        run_to_pc(device, end);
        const unsigned count[5] = {6, 10, 8, 1, 6};
        for (unsigned c = 0; c < 5; ++c) {
            const auto src = static_cast<std::uint16_t>(0x0200 + 0x20 * c);
            const auto dst = static_cast<std::uint16_t>(0x0300 + 0x20 * c);
            std::vector<std::uint8_t> want = xdata(device, src, count[c]);
            want.push_back(0);
            expect_bytes(xdata(device, dst, count[c] + 1), want);
        }
        expect(device.read_sfr(DMAIRQ), 0x1f, "all five channels complete");
    }
}

// Table 8-2 WORDSIZE/SRCINC/DESTINC: increments of 0, 1, 2 or -1 transfer
// units (a word is two bytes); 16-bit addresses wrap (model-defined at
// 0xFFFF -> 0x0000). The source at 0xFFFF reads the XBANK flash window.
void test_dma_word_size_and_increments() {
    Asm main(0x0100);
    main.mov(SP, 0x30);
    Desc d0 = desc(0x0200, 0x0300, 3); // word, +1 word source, +2 words destination
    d0.word = true;
    d0.destinc = Inc2;
    Desc d1 = desc(0x0224, 0x0320, 3); // word, -1 word source
    d1.word = true;
    d1.srcinc = IncMinus1;
    Desc d2 = desc(0x0240, 0x0342, 3); // byte, fixed source, -1 destination
    d2.srcinc = Inc0;
    d2.destinc = IncMinus1;
    Desc d3 = desc(0x0260, 0x0360, 3); // byte, +2 source
    d3.srcinc = Inc2;
    Desc d4 = desc(0xffff, 0x0380, 3); // source wraps to 0x0000
    main.descriptor(0, d0).descriptor(1, d1).descriptor(2, d2).descriptor(3, d3).descriptor(4, d4);
    main.point_descriptors();
    const std::uint16_t arm = main.here();
    main.mov(DMAARM, 0x1f).mov(DMAREQ, 0x01).nops(100).mov(DMAREQ, 0x1e).nops(100);
    const std::uint16_t end = main.here();
    main.halt();

    Flash flash = base_program(main);
    flash.at(0x7fff).op({0xa5}); // XDATA 0xFFFF with MEMCTR.XBANK = 0
    Cc2530F256 device;
    load(device, flash);
    write_xdata(device, 0x0000, {0x5a, 0x5b, 0x5c});
    write_xdata(device, 0x0200, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06});
    write_xdata(device, 0x0220, {0x10, 0x11, 0x12, 0x13, 0x14, 0x15});
    write_xdata(device, 0x0240, {0x77, 0x78});
    write_xdata(device, 0x0260, {0x20, 0x21, 0x22, 0x23, 0x24});

    // Word slots: 4 clocks each (model-defined). Five channels armed: channel
    // 0 is ready last, at +45; its words land at +49, +53, +57.
    const std::uint64_t t = run_to_pc(device, arm);
    run_to(device, t + 48);
    expect(device.peek_xdata(0x0300), 0x00, "first word not yet written");
    run_to(device, t + 49);
    expect_bytes(xdata(device, 0x0300, 2), {0x01, 0x02});
    run_to(device, t + 57);
    expect(device.read_sfr(DMAIRQ), 0x01, "channel 0 complete after three words");
    run_to_pc(device, end);
    expect_bytes(xdata(device, 0x0300, 11),
                 {0x01, 0x02, 0x00, 0x00, 0x03, 0x04, 0x00, 0x00, 0x05, 0x06, 0x00});
    expect_bytes(xdata(device, 0x0320, 7), {0x14, 0x15, 0x12, 0x13, 0x10, 0x11, 0x00});
    expect_bytes(xdata(device, 0x033f, 5), {0x00, 0x77, 0x77, 0x77, 0x00});
    expect_bytes(xdata(device, 0x0360, 4), {0x20, 0x22, 0x24, 0x00});
    expect_bytes(xdata(device, 0x0380, 4), {0xa5, 0x5a, 0x5b, 0x00});
    expect(device.read_sfr(DMAIRQ), 0x1f, "all complete");
}

// 8.2.7: repeated modes rearm after completion - the configuration is
// fetched again (Figure 8-1; nine clocks), so pointers restart at the
// descriptor addresses and descriptor changes take effect. DMAARM stays 1;
// a DMAREQ written during the re-fetch is served when the channel is ready;
// a trigger of the (unchanged) source during the re-fetch is a missed event.
void test_dma_repeated_modes() {
    {
        Asm main(0x0100);
        main.mov(SP, 0x30);
        Desc d = desc(0x0200, 0x0300, 4);
        d.tmode = RepeatedBlock;
        main.descriptor(0, d).point_descriptors();
        const std::uint16_t arm = main.here();
        // The second DMAREQ write at +20.
        main.mov(DMAARM, 0x01).mov(DMAREQ, 0x01).nops(20 - 2 * MovDirImm).mov(DMAREQ, 0x01);
        main.nops(80).halt();

        Cc2530F256 device;
        load(device, base_program(main));
        write_xdata(device, 0x0200, {0xa0, 0xa1, 0xa2, 0xa3});
        write_xdata(device, 0x0210, {0xb0, 0xb1, 0xb2, 0xb3});
        const std::uint64_t t = run_to_pc(device, arm);
        run_to(device, t + 12);
        device.write_sram(Desc0 + 1, 0x10); // SRCADDR -> 0x0210 for the next fetch
        run_to(device, t + 16);
        expect(device.peek_xdata(0x0303), 0x00, "fourth byte at +17");
        run_to(device, t + 17);
        expect_bytes(xdata(device, 0x0300, 4), {0xa0, 0xa1, 0xa2, 0xa3});
        expect(device.read_sfr(DMAIRQ), 0x01, "DMAIF0 after the first block");
        expect(device.read_sfr(DMAARM), 0x01, "repeated block stays armed");
        device.write_sfr(DMAIRQ, 0x00);
        run_to(device, t + 20 + MovDirImm);
        expect(device.read_sfr(DMAREQ), 0x01, "DMAREQ written at +20 waits for the re-fetch");
        run_to(device, t + 26);
        expect(device.read_sfr(DMAREQ), 0x00, "re-fetch ends at +26");
        run_to(device, t + 27);
        expect(device.peek_xdata(0x0300), 0xa0, "second block not started");
        run_to(device, t + 28);
        expect(device.peek_xdata(0x0300), 0xb0, "pointers reloaded from the new descriptor");
        run_to(device, t + 34);
        expect_bytes(xdata(device, 0x0300, 4), {0xb0, 0xb1, 0xb2, 0xb3});
        expect(device.read_sfr(DMAIRQ), 0x01, "DMAIF0 after the second block");
        expect(device.read_sfr(DMAARM), 0x01, "still armed");
        device.write_sfr(DMAARM, 0x81); // ABORT
        expect(device.read_sfr(DMAARM), 0x00, "aborted");
    }
    {
        Asm main(0x0100);
        main.mov(SP, 0x30);
        Desc d = desc(0x0220, 0x0320, 2);
        d.tmode = RepeatedSingle;
        d.trig = 2;
        main.descriptor(1, d).point_descriptors();
        const std::uint16_t arm = main.here();
        main.mov(DMAARM, 0x02).nops(80).halt();

        Cc2530F256 device;
        load(device, base_program(main));
        write_xdata(device, 0x0220, {0xc0, 0xc1});
        const std::uint64_t t = run_to_pc(device, arm);
        run_to(device, t + 10);
        dma(device).trigger(2);
        run_to(device, t + 12);
        expect(device.peek_xdata(0x0320), 0xc0, "first single transfer");
        run_to(device, t + 14);
        dma(device).trigger(2);
        run_to(device, t + 16);
        expect(device.peek_xdata(0x0321), 0xc1, "second single transfer");
        expect(device.read_sfr(DMAIRQ), 0x02, "DMAIF1");
        expect(device.read_sfr(DMAARM), 0x02, "repeated single stays armed");
        run_to(device, t + 17);
        device.write_sram(0x0220, 0xd0);
        run_to(device, t + 20);
        dma(device).trigger(2); // during the re-fetch (+16 .. +25)
        run_to(device, t + 26);
        expect(device.peek_xdata(0x0320), 0xc0, "missed trigger waits for the re-fetch");
        run_to(device, t + 27);
        expect(device.peek_xdata(0x0320), 0xd0, "served at ready, pointers reloaded");
        device.write_sfr(DMAARM, 0x82);
        expect(device.read_sfr(DMAARM), 0x00, "aborted");
        dma(device).trigger(2);
        run_to(device, t + 40);
        expect(device.peek_xdata(0x0321), 0xc1, "aborted channel ignores triggers");
    }
}

// Table 8-1 trigger 1 (PREV): completion of channel n triggers channel n+1.
// 8.1 arming order: with several channels armed together channel 1 is ready
// first and channel 0 last; the model spaces them one clock apart at the
// end of the 9k-clock window (model-defined).
void test_dma_prev_chain_and_arming_order() {
    {
        Asm main(0x0100);
        main.mov(SP, 0x30);
        Desc d0 = desc(0x0200, 0x0300, 2);
        Desc d1 = desc(0x0210, 0x0302, 2);
        d1.trig = TrigPrev;
        Desc d2 = desc(0x0220, 0x0304, 1);
        d2.tmode = Single;
        d2.trig = TrigPrev;
        main.descriptor(0, d0).descriptor(1, d1).descriptor(2, d2).point_descriptors();
        const std::uint16_t arm = main.here();
        main.mov(DMAARM, 0x07).mov(DMAREQ, 0x01).nops(60).halt();

        Cc2530F256 device;
        load(device, base_program(main));
        write_xdata(device, 0x0200, {0xe0, 0xe1});
        write_xdata(device, 0x0210, {0xe2, 0xe3});
        write_xdata(device, 0x0220, {0xe4});
        const std::uint64_t t = run_to_pc(device, arm);
        run_to(device, t + 28);
        expect(device.peek_xdata(0x0300), 0x00, "channel 0 ready at +27");
        run_to(device, t + 29);
        expect(device.peek_xdata(0x0300), 0xe0, "channel 0 first byte");
        run_to(device, t + 31);
        expect(device.read_sfr(DMAIRQ), 0x01, "channel 0 complete");
        run_to(device, t + 32);
        expect(device.peek_xdata(0x0302), 0x00, "channel 1 slot in progress");
        run_to(device, t + 33);
        expect(device.peek_xdata(0x0302), 0xe2, "channel 1 triggered by PREV");
        run_to(device, t + 35);
        expect(device.read_sfr(DMAIRQ), 0x03, "channel 1 complete");
        run_to(device, t + 36);
        expect(device.peek_xdata(0x0304), 0x00, "channel 2 slot in progress");
        run_to(device, t + 37);
        expect_bytes(xdata(device, 0x0300, 6), {0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0x00});
        expect(device.read_sfr(DMAIRQ), 0x07, "chain complete");
        expect(device.read_sfr(DMAARM), 0x00, "all disarmed");
    }
    {
        Asm main(0x0100);
        main.mov(SP, 0x30);
        for (unsigned c = 0; c < 5; ++c) {
            Desc d = desc(static_cast<std::uint16_t>(0x0200 + c), 0x0300, 1);
            main.descriptor(c, d);
        }
        main.point_descriptors();
        const std::uint16_t arm = main.here();
        main.mov(DMAARM, 0x1f).mov(DMAREQ, 0x1f).nops(80).halt();

        Cc2530F256 device;
        load(device, base_program(main));
        write_xdata(device, 0x0200, {0xc0, 0xc1, 0xc2, 0xc3, 0xc4});
        const std::uint64_t t = run_to_pc(device, arm);
        // 45 clocks for five channels: ready at +41 (ch1), +42, +43, +44,
        // +45 (ch0); equal priorities in round-robin order.
        run_to(device, t + 42);
        expect(device.peek_xdata(0x0300), 0x00, "nothing before channel 1's slot ends");
        std::vector<std::uint8_t> seen;
        for (unsigned k = 0; k < 5; ++k) {
            run_to(device, t + 43 + 2 * k);
            seen.push_back(device.peek_xdata(0x0300));
        }
        expect_bytes(seen, {0xc1, 0xc2, 0xc3, 0xc4, 0xc0});
    }
}

// 8.2.8: the highest priority wins each transfer; a tie is resolved
// round-robin (model-defined: per transfer, starting after the channel
// granted last). A high-priority request interrupts a running low-priority
// block between two transfers.
void test_dma_priority_and_round_robin() {
    const auto sample = [](Cc2530F256 &device, std::uint64_t first, unsigned count) {
        std::vector<std::uint8_t> seen;
        for (unsigned k = 0; k < count; ++k) {
            run_to(device, first + 2 * k);
            seen.push_back(device.peek_xdata(0x0300));
        }
        return seen;
    };
    {
        Asm main(0x0100);
        main.mov(SP, 0x30);
        Desc low = desc(0x0200, 0x0300, 3);
        low.destinc = Inc0;
        Desc high = desc(0x0210, 0x0300, 3);
        high.destinc = Inc0;
        high.priority = PrioHigh;
        Desc assured = desc(0x0220, 0x0300, 3);
        assured.destinc = Inc0;
        assured.priority = PrioAssured;
        main.descriptor(0, low).descriptor(1, high).descriptor(2, assured).point_descriptors();
        const std::uint16_t arm = main.here();
        main.mov(DMAARM, 0x07).nops(30 - MovDirImm).mov(DMAREQ, 0x07).nops(60).halt(); // DMAREQ at +30

        Cc2530F256 device;
        load(device, base_program(main));
        write_xdata(device, 0x0200, {0xa0, 0xa1, 0xa2});
        write_xdata(device, 0x0210, {0xb0, 0xb1, 0xb2});
        write_xdata(device, 0x0220, {0xc0, 0xc1, 0xc2});
        const std::uint64_t t = run_to_pc(device, arm);
        // Slots end at +32, +34, ...; sampled one clock later, after the
        // 3-clock DMAREQ write.
        expect_bytes(sample(device, t + 33, 9), {0xb0, 0xb1, 0xb2, 0xc0, 0xc1, 0xc2, 0xa0, 0xa1, 0xa2});
    }
    {
        Asm main(0x0100);
        main.mov(SP, 0x30);
        Desc low = desc(0x0200, 0x0300, 4);
        low.destinc = Inc0;
        Desc high = desc(0x0210, 0x0301, 3);
        high.destinc = Inc0;
        high.priority = PrioHigh;
        main.descriptor(0, low).descriptor(1, high).point_descriptors();
        const std::uint16_t arm = main.here();
        // DMAREQ0 at +20, DMAREQ1 at +23.
        main.mov(DMAARM, 0x03).nops(20 - MovDirImm).mov(DMAREQ, 0x01).mov(DMAREQ, 0x02).nops(40).halt();

        Cc2530F256 device;
        load(device, base_program(main));
        write_xdata(device, 0x0200, {0xa0, 0xa1, 0xa2, 0xa3});
        write_xdata(device, 0x0210, {0xb0, 0xb1, 0xb2});
        const std::uint64_t t = run_to_pc(device, arm);
        // DMAREQ1 is written at t+23 during A1's slot (t+22..t+24); the
        // high-priority channel takes over at the next arbitration. The
        // 3-clock DMAREQ1 write (Table 2-3) leaves no instruction boundary
        // while A1's byte is the last one moved, so the channels write
        // separate bytes (low 0x0300, high 0x0301) and each sample reads
        // both: a0, a1, then b0-b2, then a2, a3.
        const auto both = [&device](std::uint64_t at) {
            run_to(device, at);
            return xdata(device, 0x0300, 2);
        };
        expect_bytes(both(t + 23), {0xa0, 0x00});
        expect_bytes(both(t + 26), {0xa1, 0xb0});
        expect_bytes(both(t + 28), {0xa1, 0xb1});
        expect_bytes(both(t + 30), {0xa1, 0xb2});
        expect_bytes(both(t + 32), {0xa2, 0xb2});
        expect_bytes(both(t + 34), {0xa3, 0xb2});
    }
    {
        Asm main(0x0100);
        main.mov(SP, 0x30);
        Desc first = desc(0x0200, 0x0300, 3);
        first.destinc = Inc0;
        Desc second = desc(0x0230, 0x0300, 3);
        second.destinc = Inc0;
        main.descriptor(0, first).descriptor(3, second).point_descriptors();
        const std::uint16_t arm = main.here();
        main.mov(DMAARM, 0x09).nops(20 - MovDirImm).mov(DMAREQ, 0x09).nops(40).halt(); // DMAREQ at +20

        Cc2530F256 device;
        load(device, base_program(main));
        write_xdata(device, 0x0200, {0xd0, 0xd1, 0xd2});
        write_xdata(device, 0x0230, {0xa0, 0xa1, 0xa2});
        const std::uint64_t t = run_to_pc(device, arm);
        // Slots end at +22, +24, ...; sampled one clock later.
        expect_bytes(sample(device, t + 23, 6), {0xd0, 0xa0, 0xd1, 0xa1, 0xd2, 0xa2});
    }
}

// 8.4: DMAARM.ABORT with DMAARMn = 1 disarms the channel; an aborted
// channel sets no DMAIRQ flag and requests no interrupt. 8.1: the DMAREQ
// bit is not cleared by disarming, so rearming serves it.
void test_dma_abort() {
    Asm main(0x0100);
    main.mov(SP, 0x30).mov(IEN1, 0x01).mov(IEN0, 0x80);
    Desc d0 = desc(0x0200, 0x0300, 8);
    d0.irqmask = true;
    Desc d1 = desc(0x0210, 0x0310, 2);
    d1.irqmask = true;
    main.descriptor(0, d0).descriptor(1, d1).point_descriptors();
    const std::uint16_t arm_a = main.here();
    // ABORT at +14.
    main.mov(DMAARM, 0x01).mov(DMAREQ, 0x01).nops(14 - 2 * MovDirImm).mov(DMAARM, 0x81).nops(20);
    main.copy(0x40, DMAARM).copy(0x41, DMAIRQ);
    const std::uint16_t arm_b = main.here();
    // DMAREQ at +3, ABORT at +6 (before ready at +9), rearm at +26.
    main.mov(DMAARM, 0x02).mov(DMAREQ, 0x02).mov(DMAARM, 0x82).nops(26 - 3 * MovDirImm);
    main.mov(DMAARM, 0x02).nops(60);
    const std::uint16_t end = main.here();
    main.halt();

    Cc2530F256 device;
    load(device, base_program(main));
    write_xdata(device, 0x0200, {1, 2, 3, 4, 5, 6, 7, 8});
    write_xdata(device, 0x0210, {9, 10});
    const std::uint64_t t = run_to_pc(device, arm_a);
    run_to(device, t + 13);
    expect_bytes(xdata(device, 0x0300, 3), {1, 2, 0});
    run_to(device, t + 14 + MovDirImm); // ABORT written at +14, the slot ending at +15 is cancelled
    expect(device.read_sfr(DMAARM), 0x00, "aborted channel disarmed");
    run_to(device, t + 30);
    expect_bytes(xdata(device, 0x0300, 3), {1, 2, 0});
    expect(device.read_sfr(DMAIRQ), 0x00, "no DMAIRQ flag after abort");

    const std::uint64_t tb = run_to_pc(device, arm_b);
    run_to(device, tb + 3 * MovDirImm); // the ABORT write (+6) has executed
    expect(device.read_sfr(DMAARM), 0x00, "aborted before it was ready");
    expect(device.read_sfr(DMAREQ), 0x02, "DMAREQ not cleared by disarming");
    run_to(device, tb + 25);
    expect(device.peek_xdata(0x0310), 0x00, "no transfer while disarmed");
    expect(device.read_sfr(DMAREQ), 0x02, "DMAREQ still pending");
    run_to(device, tb + 26 + MovDirImm);
    expect(device.read_sfr(DMAARM), 0x02, "rearmed at +26");
    run_to(device, tb + 35);
    expect(device.read_sfr(DMAREQ), 0x00, "pending DMAREQ served at ready (+35)");
    run_to(device, tb + 39);
    expect_bytes(xdata(device, 0x0310, 2), {9, 10});
    run_to_pc(device, end);
    expect(device.read_idata(0x40), 0x00, "DMAARM.ABORT reads 0");
    expect(device.read_idata(0x41), 0x00, "DMAIRQ after abort");
    expect(device.read_idata(DmaIsrCount), 1, "one DMA interrupt (channel 1)");
    expect(device.read_idata(DmaIsrFlags), 0x02, "DMAIF1 seen by the ISR");
    expect(device.read_sfr(DMAIRQ), 0x00, "cleared by the ISR");
    expect(device.read_sfr(IRCON) & 0x01, 0, "IRCON.DMAIF cleared by the ISR");
}

// 8.5/8.8: DMAIRQ flags are set on completion regardless of IRQMASK and are
// R/W0; IRCON.DMAIF only with IRQMASK = 1. Rearming with IRQMASK changed
// to 1 while the stored flag is set "could generate an interrupt based on
// the stored interrupt flag" (8.5) - the model raises it at the fetch.
void test_dma_irq_flags() {
    Asm main(0x0100);
    main.mov(SP, 0x30).mov(IEN1, 0x01).mov(IEN0, 0x80);
    Desc d = desc(0x0200, 0x0300, 1);
    d.tmode = Single;
    main.descriptor(0, d).point_descriptors();
    main.mov(DMAARM, 0x01).nops(12).mov(DMAREQ, 0x01).nops(10);
    main.copy(0x40, DMAIRQ).copy(0x41, IRCON);
    main.mov(DMAIRQ, 0xff).copy(0x42, DMAIRQ).copy(0x43, DmaIsrCount);
    d.irqmask = true;
    main.xwrite(Desc0 + 7, d.bytes()[7]);
    main.mov(DMAARM, 0x01).nops(20);
    main.copy(0x44, DmaIsrCount).copy(0x45, DMAIRQ);
    main.mov(DMAREQ, 0x01).nops(20);
    main.copy(0x46, DmaIsrCount).copy(0x47, DmaIsrFlags).copy(0x48, DMAARM);
    const std::uint16_t end = main.here();
    main.halt();

    Cc2530F256 device;
    load(device, base_program(main));
    write_xdata(device, 0x0200, {0x3c});
    run_to_pc(device, end);
    expect(device.read_idata(0x40), 0x01, "DMAIF0 set with IRQMASK = 0");
    expect(device.read_idata(0x41) & 0x01, 0, "IRCON.DMAIF not set with IRQMASK = 0");
    expect(device.read_idata(0x42), 0x01, "writing 1 to DMAIRQ has no effect");
    expect(device.read_idata(0x43), 0, "no interrupt with IRQMASK = 0");
    expect(device.read_idata(0x44), 1, "stored flag interrupts on rearm with IRQMASK = 1");
    expect(device.read_idata(0x45), 0x00, "ISR cleared DMAIRQ");
    expect(device.read_idata(0x46), 2, "completion interrupt");
    expect(device.read_idata(0x47), 0x01, "ISR saw DMAIF0");
    expect(device.read_idata(0x48), 0x00, "single mode disarmed after LEN = 1");
    expect(device.peek_xdata(0x0300), 0x3c, "data moved");
}

// The DMA uses the full XDATA map: a write to 0x70C7 is MEMCTR (SFR alias,
// SWRU191F 2.2.1), selecting XBANK 2 for the PREV-chained read at 0x8000;
// CPU-internal SFRs (B) are not reachable through the alias (core map,
// counted as unmapped). External triggers come from other peripherals.
void test_dma_sfr_alias_xbank_and_external_trigger() {
    Asm main(0x0100);
    main.mov(SP, 0x30);
    Desc d0 = desc(0x0200, 0x70c7, 1);
    Desc d1 = desc(0x8000, 0x0300, 2);
    d1.trig = TrigPrev;
    Desc d2 = desc(0x0201, 0x70f0, 1);
    Desc d3 = desc(0x70c7, 0x0310, 1);
    d3.tmode = Single;
    d3.trig = 20;
    main.descriptor(0, d0).descriptor(1, d1).descriptor(2, d2).descriptor(3, d3).point_descriptors();
    main.mov(DMAARM, 0x0f).nops(40).mov(DMAREQ, 0x05).nops(40);
    const std::uint16_t end = main.here();
    main.nops(40).halt();

    Flash flash = base_program(main);
    flash.at(0x10000).op({0x2a, 0x2b}); // bank 2 offset 0
    Cc2530F256 device;
    load(device, flash);
    write_xdata(device, 0x0200, {0x02, 0x99});
    const std::uint64_t unmapped = device.diagnostics().unmapped_xdata_writes;
    run_to_pc(device, end);
    expect(device.memctr(), 0x02, "MEMCTR written through 0x70C7");
    expect_bytes(xdata(device, 0x0300, 3), {0x2a, 0x2b, 0x00});
    expect(device.read_sfr(SfrB), 0x00, "B not writable through XDATA");
    expect(device.diagnostics().unmapped_xdata_writes - unmapped, 1, "one unmapped DMA write");
    expect(device.read_sfr(DMAIRQ), 0x07, "channels 0-2 complete");
    expect(device.peek_xdata(0x0310), 0x00, "channel 3 waits for trigger 20");
    dma(device).trigger(20);
    settle(device, 10);
    expect(device.peek_xdata(0x0310), 0x02, "SFR read through 0x70C7");
    expect(device.read_sfr(DMAIRQ), 0x0f, "channel 3 complete");
}

// The DMA runs on the system clock: it makes no progress in PM1 (SWRU191F
// 4.1: the high-speed oscillators are off) and continues after wake-up.
void test_dma_halts_in_pm1() {
    Asm main(0x0100);
    main.mov(SP, 0x30).mov(IEN0, 0xa0).mov(SLEEPCMD, 0x05);
    main.descriptor(0, desc(0x0200, 0x0300, 4)).point_descriptors();
    // DMAREQ first: it stays set until a transfer starts (8.1, 8.8), and
    // with 3-clock MOVs (Table 2-3) PM1 then begins at the boundary after
    // the PCON write (+9), before the channel armed at +3 is ready (+12).
    main.mov(DMAREQ, 0x01).mov(DMAARM, 0x01).mov(PCON, 0x01).nops(1);
    const std::uint16_t end = main.here();
    main.nops(40).halt();

    Cc2530F256 device;
    load(device, base_program(main));
    write_xdata(device, 0x0200, {0x61, 0x62, 0x63, 0x64});
    // SWRU191F 11.2: EA = STIE = 1 and the default sleep timer compare
    // (0xFFFFFF) would wake PM1 after 2^24 - 1 LF periods, so bound the run.
    Cc2530RunResult result = device.run(1000, 100'000);
    CHECK(result.error.empty());
    CHECK(result.sleeping);
    CHECK(device.power_mode() == Cc2530PowerMode::Pm1);
    settle(device, 5000);
    expect_bytes(xdata(device, 0x0300, 4), {0, 0, 0, 0});
    expect(device.read_sfr(DMAREQ), 0x01, "request pending in PM1");
    expect(device.read_sfr(DMAARM), 0x01, "armed in PM1");
    device.write_sfr(IRCON, 0x80); // sleep timer flag: PM1 wake-up source
    run_to_pc(device, end);
    settle(device, 30);
    CHECK(device.power_mode() == Cc2530PowerMode::Active);
    expect_bytes(xdata(device, 0x0300, 4), {0x61, 0x62, 0x63, 0x64});
    expect(device.read_sfr(DMAIRQ), 0x01, "complete after wake-up");
}

// A PREV chain with two DMA interrupts is identical in fast, step and trace
// runs. The 6-byte block of channel 1 (12 clocks, no interrupt) separates
// the two interrupting completions: with Table 2-3 timing the ISR clears
// IRCON.DMAIF no earlier than 15 clocks after channel 0 completes (entry 7, LJMP 4,
// PUSH 4), so channel 2's completion requests a second interrupt instead of
// merging into the first.
void test_dma_run_mode_equivalence() {
    Asm main(0x0100);
    main.mov(SP, 0x30).mov(IEN1, 0x01).mov(IEN0, 0x80);
    Desc d0 = desc(0x0200, 0x0300, 3);
    d0.irqmask = true;
    Desc d1 = desc(0x0200, 0x0330, 6);
    d1.trig = TrigPrev;
    Desc d2 = desc(0x0200, 0x0310, 3);
    d2.trig = TrigPrev;
    d2.word = true;
    d2.irqmask = true;
    Desc d3 = desc(0x0200, 0x0320, 2);
    d3.tmode = RepeatedSingle;
    d3.trig = TrigPrev;
    main.descriptor(0, d0).descriptor(1, d1).descriptor(2, d2).descriptor(3, d3);
    main.point_descriptors().mov(DMAARM, 0x0f).mov(DMAREQ, 0x01);
    const std::uint16_t loop = main.here();
    main.op({0x05, 0x50}).sjmp(loop); // INC 0x50; SJMP loop
    const Flash flash = base_program(main);
    for_each_mode(
        flash, 400,
        [](Cc2530F256 &device) { write_xdata(device, 0x0200, {1, 2, 3, 4, 5, 6}); },
        [](Cc2530F256 &device, Mode) {
            expect_bytes(xdata(device, 0x0300, 3), {1, 2, 3});
            expect_bytes(xdata(device, 0x0330, 6), {1, 2, 3, 4, 5, 6});
            expect_bytes(xdata(device, 0x0310, 6), {1, 2, 3, 4, 5, 6});
            expect_bytes(xdata(device, 0x0320, 2), {1, 0});
            expect(device.read_idata(DmaIsrCount), 2, "two DMA interrupts");
        },
        "dma prev chain");
}

// ======================================================================
// Flash controller (SWRU191F Chapter 6; timing SWRU191F 2.2.3 and Chapter 6
// introduction: "Flash-page erase timing 20 ms", "Flash-write timing
// (4 bytes) 20 us" = 640000 and 640 clocks of the 32 MHz system clock; the
// data sheet SWRS081B gives no erase/write time, only 20k erase cycles).
// ======================================================================

constexpr std::uint64_t EraseCycles = 640'000;
constexpr std::uint64_t WordCycles = 640;

// Flash `main`: switch MEMCTR.XMAP on and continue in SRAM at CODE 0x9000.
Asm sram_entry() {
    Asm main(0x0100);
    main.mov(SP, 0x30).mov(MEMCTR, 0x08).ljmp(SramCode);
    return main;
}

std::vector<std::uint8_t> filled(std::size_t count, std::uint8_t value) {
    return std::vector<std::uint8_t>(count, value);
}

// 6.3: a page erase from SRAM code sets the page to 0xFF after 20 ms;
// FCTL.BUSY/ERASE are 1 meanwhile and the CPU keeps running. The erased
// page is visible through CODE (FMAP bank) and the XDATA XBANK window.
void test_flash_erase_from_sram() {
    Asm code(SramCode);
    code.mov(FMAP, 2).mov(MEMCTR, 0x0a); // CODE 0xA000+ and XBANK = bank 2
    code.dptr(0xa800).op({0xe4, 0x93, 0xf5, 0x42, 0xe0, 0xf5, 0x43}); // MOVC/MOVX pre-reads
    code.xwrite(FADDRH, 0x4a);                                         // page 37
    code.dptr(FCTL).op({0x74, 0x05});
    const std::uint16_t erase = code.here();
    code.op({0xf0}); // MOVX @DPTR,A: FCTL.ERASE = 1
    code.delay(5, 165).nops(3000);
    code.dptr(0xa800).op({0xe4, 0x93, 0xf5, 0x40, 0xe0, 0xf5, 0x41});
    code.xread_to(FCTL, 0x44);
    const std::uint16_t end = code.here();
    code.halt();

    Flash flash = base_program(sram_entry());
    flash.at(0x127f0).op(filled(0x820, 0x00)); // page 37 and its neighbours' edges
    Cc2530F256 device;
    load(device, flash);
    place_sram(device, code);
    const std::uint64_t t = run_to_pc(device, erase);
    run_to(device, t + MovxWrite);
    expect(device.peek_xdata(FCTL), 0x85, "BUSY | CM=01 | ERASE");
    const std::uint64_t sled = t + MovxWrite + delay_cycles(5, 165);
    CHECK(sled < t + EraseCycles - 1);
    run_to(device, t + EraseCycles - 1);
    expect(device.peek_xdata(FCTL), 0x85, "still erasing at 20 ms - 1 clock");
    expect(device.read_flash(0x12800), 0x00, "model-defined: the erase takes effect at completion");
    run_to(device, t + EraseCycles);
    expect(device.peek_xdata(FCTL), 0x04, "erase complete");
    run_to_pc(device, end);
    expect_bytes(flash_bytes(device, 0x12800, 0x800), filled(0x800, 0xff));
    expect_bytes(flash_bytes(device, 0x127f0, 0x10), filled(0x10, 0x00));
    expect_bytes(flash_bytes(device, 0x13000, 0x10), filled(0x10, 0x00));
    expect_bytes(idata(device, 0x40, 5), {0xff, 0xff, 0x00, 0x00, 0x04});
    expect(device.diagnostics().cpu_stall_cycles, 0, "no stall when executing from SRAM");
}

// 6.3.1: started from flash code, the CPU stalls until the erase is done
// and resumes with the next instruction. A reset abandons the erase
// (model-defined default: the page is left unchanged; the other outcomes and
// power cuts are in test_cc2530_flash_powercut.cpp).
Asm erase_page6_program(std::uint16_t &erase, std::uint16_t &next) {
    Asm main(0x0100);
    main.mov(SP, 0x30).xwrite(FADDRH, 0x0c).dptr(FCTL).op({0x74, 0x05});
    erase = main.here();
    main.op({0xf0});
    next = main.here();
    main.xread_to(FCTL, 0x44).dptr(0x3000).op({0xe4, 0x93, 0xf5, 0x40}).halt();
    return main;
}

Flash erase_page6_flash(const Asm &main) {
    Flash flash = base_program(main);
    flash.at(0x3000).op(filled(0x800, 0x00)); // page 6
    return flash;
}

void test_flash_erase_stalls_cpu_from_flash() {
    std::uint16_t erase = 0, next = 0;
    const Asm main = erase_page6_program(erase, next);
    {
        Cc2530F256 device;
        load(device, erase_page6_flash(main));
        const std::uint64_t t = run_to_pc(device, erase);
        const std::uint64_t stalls = device.diagnostics().cpu_stall_cycles;
        expect(run_to_pc(device, next), t + EraseCycles, "next instruction after 20 ms");
        expect(device.diagnostics().cpu_stall_cycles - stalls, EraseCycles - MovxWrite,
               "stalled clocks");
        settle(device, 20);
        expect(device.read_idata(0x44), 0x04, "FCTL after the stall");
        expect(device.read_idata(0x40), 0xff, "erased byte via MOVC");
        expect_bytes(flash_bytes(device, 0x3000, 0x800), filled(0x800, 0xff));
        expect(device.read_code(0x37ff), 0xff, "CODE view refreshed");
    }
    {
        Cc2530F256 device;
        load(device, erase_page6_flash(main));
        const std::uint64_t t = run_to_pc(device, erase);
        run_to(device, t + 1000);
        expect(device.pc(), next, "stalled before the next instruction");
        expect(device.peek_xdata(FCTL), 0x85, "erasing");
        device.reset();
        expect(device.peek_xdata(FCTL), 0x04, "FCTL reset value");
        expect(device.peek_xdata(FADDRH), 0x00, "FADDRH reset value");
        expect_bytes(flash_bytes(device, 0x3000, 0x10), filled(0x10, 0x00));
        // The stall ended with the reset: the program runs at once.
        const std::uint64_t stalls = device.diagnostics().cpu_stall_cycles;
        const std::uint64_t r = device.registers().cycles;
        CHECK(run_to_pc(device, erase) - r < 40);
        expect(device.diagnostics().cpu_stall_cycles, stalls, "no stall after the reset");
    }
}

// 6.2.1: word writes from SRAM code. Four FWDATA bytes program one 32-bit
// word (bits can only be cleared); FULL is 1 while the next word waits;
// FWDATA is ignored while FULL; FADDR increments per word; without the next
// four bytes within 20 us the write ends (BUSY = WRITE = 0, no ABORT).
// Flash content persists across reset().
void test_flash_word_writes_from_sram() {
    Asm code(SramCode);
    code.xwrite(FADDRH, 0x0c).xwrite(FADDRL, 0x00); // word 0x0C00 = byte 0x3000
    code.dptr(FCTL).op({0x74, 0x06});
    const std::uint16_t write = code.here();
    code.op({0xf0}); // FCTL.WRITE = 1
    code.dptr(FWDATA).xbytes({0x3c, 0xff, 0x0f}).op({0x74, 0xf0});
    const std::uint16_t fourth = code.here();
    code.op({0xf0});
    code.xbytes({0x11, 0x22, 0x33, 0x44, 0x00}); // 9th byte while FULL
    code.nops(1400);
    // Time-out: only three bytes within 20 us.
    code.xwrite(FADDRL, 0x10).dptr(FCTL).op({0x74, 0x06});
    const std::uint16_t timeout = code.here();
    code.op({0xf0});
    code.dptr(FWDATA).xbytes({0x00, 0x00, 0x00}).nops(700);
    code.dptr(FWDATA).xbytes({0x00}); // after the time-out: ignored
    const std::uint16_t end = code.here();
    code.halt();

    Flash flash = base_program(sram_entry());
    flash.at(0x3000).op({0xf0, 0x0f, 0x55, 0xaa});
    Cc2530F256 device;
    load(device, flash);
    place_sram(device, code);
    const std::uint64_t tw = run_to_pc(device, write);
    run_to(device, tw + MovxWrite);
    expect(device.peek_xdata(FCTL), 0x86, "BUSY | CM=01 | WRITE");
    const std::uint64_t t = run_to_pc(device, fourth);
    // After the fourth byte's MOVX and five MOV A,#data; MOVX @DPTR,A pairs.
    run_to(device, t + MovxWrite + 5 * cycles_of({0x74, 0xf0}));
    expect(device.peek_xdata(FCTL), 0xc6, "FULL after the second word");
    run_to(device, t + WordCycles - 1);
    expect(device.peek_xdata(FCTL), 0xc6, "first word in progress");
    expect_bytes(flash_bytes(device, 0x3000, 4), {0xf0, 0x0f, 0x55, 0xaa});
    run_to(device, t + WordCycles);
    expect(device.peek_xdata(FCTL), 0x86, "second word started, FULL = 0");
    expect_bytes(flash_bytes(device, 0x3000, 4), {0x30, 0x0f, 0x05, 0xa0});
    expect(device.peek_xdata(FADDRL), 0x01, "FADDR incremented");
    run_to(device, t + 2 * WordCycles - 1);
    expect(device.peek_xdata(FCTL), 0x86, "second word in progress");
    run_to(device, t + 2 * WordCycles);
    expect(device.peek_xdata(FCTL), 0x04, "write finished");
    expect_bytes(flash_bytes(device, 0x3000, 12),
                 {0x30, 0x0f, 0x05, 0xa0, 0x11, 0x22, 0x33, 0x44, 0xff, 0xff, 0xff, 0xff});
    expect(device.peek_xdata(FADDRL), 0x02, "FADDR after two words");
    expect(device.read_code(0x3004), 0x11, "CODE view refreshed");

    const std::uint64_t tt = run_to_pc(device, timeout);
    run_to(device, tt + WordCycles - 1);
    expect(device.peek_xdata(FCTL), 0x86, "waiting for the fourth byte");
    run_to(device, tt + WordCycles);
    expect(device.peek_xdata(FCTL), 0x04, "timed out without ABORT");
    run_to_pc(device, end);
    expect_bytes(flash_bytes(device, 0x3040, 8), filled(8, 0xff));
    expect(device.peek_xdata(FADDRL), 0x10, "FADDR unchanged by the time-out");
    expect(device.diagnostics().cpu_stall_cycles, 0, "no stall from SRAM");

    device.reset();
    expect_bytes(flash_bytes(device, 0x3000, 8), {0x30, 0x0f, 0x05, 0xa0, 0x11, 0x22, 0x33, 0x44});
    expect(device.read_code(0x3000), 0x30, "CODE view after reset");
    expect(device.peek_xdata(FCTL), 0x04, "FCTL after reset");
    expect(device.peek_xdata(FADDRH), 0x00, "FADDRH after reset");
    expect(device.peek_xdata(FADDRL), 0x00, "FADDRL after reset");
    device.power_on_reset();
    expect(device.read_flash(0x3007), 0x44, "flash kept by a power-on reset");
}

// 6.2.3 / 6.4: DMA trigger FLASH (18) feeds FWDATA; the controller asserts
// it (four pulses) when the write starts and when it can take the next
// word. From flash code the CPU stalls until the whole write is done.
Asm flash_dma_program(std::uint16_t origin, std::uint16_t &write, std::uint16_t &next) {
    Asm code(origin);
    Desc d = desc(0x0200, FWDATA, 8);
    d.tmode = Single;
    d.trig = TrigFlash;
    d.destinc = Inc0;
    d.priority = PrioHigh;
    code.descriptor(0, d).point_descriptors();
    code.xwrite(FADDRH, 0x0e); // page 7
    code.mov(DMAARM, 0x01).nops(12).dptr(FCTL).op({0x74, 0x06});
    write = code.here();
    code.op({0xf0});
    next = code.here();
    code.nops(1300).xread_to(FCTL, 0x44).copy(0x45, DMAIRQ).halt();
    return code;
}

void test_flash_dma_write() {
    const std::vector<std::uint8_t> data{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    {
        std::uint16_t write = 0, next = 0;
        Asm main(0x0100);
        main.mov(SP, 0x30);
        const Asm body = flash_dma_program(main.here(), write, next);
        main.op(body.code);
        Cc2530F256 device;
        load(device, base_program(main));
        write_xdata(device, 0x0200, data);
        const std::uint64_t t = run_to_pc(device, write);
        const std::uint64_t stalls = device.diagnostics().cpu_stall_cycles;
        // Bytes at +2..+8 (word 0 starts at +8), +10..+16 (FULL), word 1
        // from +648 to +1288.
        expect(run_to_pc(device, next), t + 8 + 2 * WordCycles, "stalled for both words");
        expect(device.diagnostics().cpu_stall_cycles - stalls, 8 + 2 * WordCycles - MovxWrite,
               "stall clocks");
        expect_bytes(flash_bytes(device, 0x3800, 12),
                     {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0xff, 0xff, 0xff, 0xff});
        expect(device.read_sfr(DMAIRQ), 0x01, "DMA channel 0 done");
        expect(device.peek_xdata(FCTL), 0x04, "write finished");
        expect(device.peek_xdata(FADDRH) << 8 | device.peek_xdata(FADDRL), 0x0e02, "FADDR");
    }
    {
        std::uint16_t write = 0, next = 0;
        const Asm code = flash_dma_program(SramCode, write, next);
        Cc2530F256 device;
        load(device, base_program(sram_entry()));
        place_sram(device, code);
        write_xdata(device, 0x0200, data);
        const std::uint64_t t = run_to_pc(device, write);
        run_to(device, t + 15);
        expect(device.read_sfr(DMAIRQ), 0x00, "seventh byte moved, eighth in its slot");
        run_to(device, t + 16);
        expect(device.read_sfr(DMAIRQ), 0x01, "all eight bytes moved");
        expect(device.peek_xdata(FCTL), 0xc6, "second word waits: FULL");
        run_to(device, t + 8 + WordCycles - 1);
        expect(device.peek_xdata(FCTL), 0xc6, "first word in progress");
        expect_bytes(flash_bytes(device, 0x3800, 4), {0xff, 0xff, 0xff, 0xff});
        run_to(device, t + 8 + WordCycles);
        expect(device.peek_xdata(FCTL), 0x86, "second word started");
        expect_bytes(flash_bytes(device, 0x3800, 4), {0x01, 0x02, 0x03, 0x04});
        run_to(device, t + 8 + 2 * WordCycles - 1);
        expect(device.peek_xdata(FCTL), 0x86, "second word in progress");
        run_to(device, t + 8 + 2 * WordCycles);
        expect(device.peek_xdata(FCTL), 0x04, "write finished");
        expect_bytes(flash_bytes(device, 0x3800, 8), data);
        expect(device.diagnostics().cpu_stall_cycles, 0, "no stall from SRAM");
    }
}

// 3.4.1 / Table 3-5 / 6.2.1: the lock bits in the last 16 flash bytes (one
// per page, 0 = locked) make erase and write of a locked page fail with
// FCTL.ABORT = 1; the lock-bit page itself is locked outside debug mode.
// ABORT clears when the next operation starts. FCTL.CM is read back.
void test_flash_lock_bits() {
    Asm code(SramCode);
    code.xwrite(FADDRH, 0x14).xwrite(FCTL, 0x05).xread_to(FCTL, 0x40); // erase page 10
    code.xwrite(FADDRL, 0x00).xwrite(FCTL, 0x06).xread_to(FCTL, 0x41); // write page 10
    code.dptr(FWDATA).xbytes({0x00, 0x00, 0x00, 0x00}).xread_to(FCTL, 0x42);
    code.xwrite(FADDRH, 0xfe).xwrite(FCTL, 0x05).xread_to(FCTL, 0x43); // erase page 127
    code.xwrite(FADDRH, 0x16).xwrite(FCTL, 0x06).xread_to(FCTL, 0x44); // write page 11
    code.dptr(FWDATA).xbytes({0x12, 0x34, 0x56, 0x78}).delay(1, 2).xread_to(FCTL, 0x45);
    code.xwrite(FADDRH, 0xfc).xwrite(FADDRL, 0x00).xwrite(FCTL, 0x06); // write page 126
    code.dptr(FWDATA).xbytes({0x9a, 0xbc, 0xde, 0xf0}).delay(1, 2).xread_to(FCTL, 0x46);
    code.xwrite(FCTL, 0x0c).xread_to(FCTL, 0x47).xwrite(FCTL, 0x04).xread_to(FCTL, 0x48);
    const std::uint16_t end = code.here();
    code.halt();

    Flash flash = base_program(sram_entry());
    flash.at(0x5000).op(filled(0x800, 0x5a));       // page 10
    flash.at(0x3f800).op(filled(0x10, 0x00));       // start of page 127
    flash.at(Cc2530F256::LockBitsStart + 1).op({0xfb}); // PAGELOCK[10] = 0
    flash.at(Cc2530F256::LockBitsStart + 15).op({0x7f}); // DBGLOCK = 0 (no effect)
    Cc2530F256 device;
    load(device, flash);
    place_sram(device, code);
    run_to_pc(device, end);
    expect_bytes(idata(device, 0x40, 9), {0x24, 0x86, 0x24, 0x24, 0x86, 0x04, 0x04, 0x0c, 0x04});
    expect_bytes(flash_bytes(device, 0x5000, 0x800), filled(0x800, 0x5a));
    expect_bytes(flash_bytes(device, 0x5800, 8), {0x12, 0x34, 0x56, 0x78, 0xff, 0xff, 0xff, 0xff});
    expect_bytes(flash_bytes(device, 0x3f000, 4), {0x9a, 0xbc, 0xde, 0xf0});
    expect_bytes(flash_bytes(device, 0x3f800, 0x10), filled(0x10, 0x00));
    expect_bytes(flash_bytes(device, Cc2530F256::LockBitsStart, 16),
                 {0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                  0xff, 0x7f});
}

// A flash erase stalling the CPU is identical in fast, step and trace runs.
void test_flash_run_mode_equivalence() {
    std::uint16_t erase = 0, next = 0;
    const Flash flash = erase_page6_flash(erase_page6_program(erase, next));
    for_each_mode(
        flash, EraseCycles + 1000, [](Cc2530F256 &) {},
        [](Cc2530F256 &device, Mode) {
            expect_bytes(flash_bytes(device, 0x3000, 0x800), filled(0x800, 0xff));
            expect(device.read_idata(0x44), 0x04, "FCTL after the erase");
            expect(device.read_idata(0x40), 0xff, "erased byte via MOVC");
            expect(device.diagnostics().cpu_stall_cycles, EraseCycles - MovxWrite, "stall clocks");
        },
        "flash erase");
}

// ======================================================================
// AES coprocessor (SWRU191F Chapter 15). Vectors: FIPS-197 Appendix C.1,
// NIST SP 800-38A F.2.1/F.2.2 (CBC), F.3.13/F.3.14 (CFB128), F.4.1 (OFB),
// F.5.1 (CTR), first two blocks each.
// ======================================================================

const std::vector<std::uint8_t> NistKey = bytes_of("2b7e151628aed2a6abf7158809cf4f3c");
const std::vector<std::uint8_t> NistIv = bytes_of("000102030405060708090a0b0c0d0e0f");
const std::vector<std::uint8_t> NistCounter = bytes_of("f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff");
const std::vector<std::uint8_t> NistPlain =
    bytes_of("6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51");
const std::vector<std::uint8_t> NistCbc =
    bytes_of("7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b2");
const std::vector<std::uint8_t> NistCfb =
    bytes_of("3b3fd92eb72dad20333449f8e83cfb4ac8a64537a0b3a93fcde3cdad9f1ce58b");
const std::vector<std::uint8_t> NistOfb =
    bytes_of("3b3fd92eb72dad20333449f8e83cfb4a7789508d16918f03f53c52dac54ed825");
const std::vector<std::uint8_t> NistCtr =
    bytes_of("874d6191b620e3261bef6864990db6ce9806f66b7970fdff8617187bb9fffdff");
const std::vector<std::uint8_t> FipsKey = bytes_of("000102030405060708090a0b0c0d0e0f");
const std::vector<std::uint8_t> FipsPlain = bytes_of("00112233445566778899aabbccddeeff");
const std::vector<std::uint8_t> FipsCipher = bytes_of("69c4e0d86a7b0430d8cdb78070b4c55a");

// IDATA layout of the AES programs.
constexpr std::uint8_t KeyAt = 0x80, IvAt = 0x90, InAt = 0xa0, OutAt = 0xc0;

// MOV ENCCS,#command; MOV R0,#from; MOV R2,#16; LCALL aes_in
Asm &aes_load(Asm &code, std::uint8_t command, std::uint8_t from) {
    return code.mov(ENCCS, command).movr(0, from).movr(2, 16).lcall(AesIn);
}

// FIPS-197 C.1 with ENCDI written by MOV direct,#imm: key load, ECB encrypt
// (timed), read-out and ECB decrypt of the result.
Asm aes_ecb_program(bool interrupts, std::uint16_t &key_load, std::uint16_t &start, std::uint16_t &end) {
    Asm main(0x0100);
    main.mov(SP, 0x30);
    if (interrupts)
        main.mov(IEN0, 0x90); // EA | ENCIE
    key_load = main.here();
    main.mov(ENCCS, EncLoadKey);
    for (const std::uint8_t byte : FipsKey)
        main.mov(ENCDI, byte);
    main.nops(50).mov(S0CON, 0x00); // acknowledge the key-load ENCIF pair
    start = main.here();
    main.mov(ENCCS, enc_encrypt(ModeEcb));
    for (const std::uint8_t byte : FipsPlain)
        main.mov(ENCDI, byte);
    main.nops(60).movr(1, OutAt).movr(2, 16).lcall(AesOut);
    main.mov(ENCCS, enc_decrypt(ModeEcb)).movr(0, OutAt).movr(2, 16).lcall(AesIn);
    main.movr(1, OutAt + 16).movr(2, 16).lcall(AesOut).copy(0x45, ENCCS);
    end = main.here();
    main.halt();
    return main;
}

// 15.2-15.4, 15.8, 15.10: key load (ENCIF pair and RDY at completion, as
// observed on hardware - see cc2530_aes.cpp), ECB block with RDY low
// while processing, ENCIF_1/ENCIF_0 and RDY when the block is done
// (model-defined: 40 clocks after the last input byte), decryption.
void test_aes_ecb_cpu() {
    std::uint16_t key_load = 0, start = 0, end = 0;
    Cc2530F256 device;
    load(device, base_program(aes_ecb_program(false, key_load, start, end)));
    expect(device.read_sfr(ENCCS), 0x08, "ENCCS reset value (RDY)");
    // MOV ENCCS at tk, 16 x MOV ENCDI from tk+3; the last byte at tk+48.
    constexpr unsigned LastByte = 16 * MovDirImm;
    const std::uint64_t tk = run_to_pc(device, key_load);
    run_to(device, tk + MovDirImm);
    expect(device.read_sfr(ENCCS), 0x04, "load key: CMD = 10, RDY = 0");
    run_to(device, tk + LastByte + 39);
    expect(device.read_sfr(ENCCS), 0x04, "key load in progress");
    expect(device.read_sfr(S0CON) & 0x03, 0, "no key-load interrupt before completion");
    run_to(device, tk + LastByte + 40);
    expect(device.read_sfr(ENCCS), 0x0c, "key loaded");
    expect(device.read_sfr(S0CON) & 0x03, 0x03, "key load: ENCIF_1 | ENCIF_0");
    const std::uint64_t ts = run_to_pc(device, start);
    run_to(device, ts + MovDirImm);
    expect(device.read_sfr(ENCCS), 0x40, "ECB encrypt, RDY = 0");
    run_to(device, ts + LastByte + 39);
    expect(device.read_sfr(ENCCS), 0x40, "processing");
    expect(device.read_sfr(S0CON) & 0x03, 0, "no interrupt yet");
    run_to(device, ts + LastByte + 40);
    expect(device.read_sfr(ENCCS), 0x48, "block done");
    expect(device.read_sfr(S0CON) & 0x03, 0x03, "ENCIF_1 | ENCIF_0");
    expect(device.read_sfr(ENCDO), FipsCipher[0], "first output byte");
    run_to_pc(device, end);
    expect_bytes(idata(device, OutAt, 16), FipsCipher);
    expect_bytes(idata(device, OutAt + 16, 16), FipsPlain);
    expect(device.read_idata(0x45), 0x4a, "ECB decrypt done");
}

// Key and IV loads each set the ENCIF pair at completion (hardware
// observation, see cc2530_aes.cpp); with EA | ENCIE an IV load dispatches
// the ENC vector, whose ISR clears the flags.
void test_aes_load_interrupts() {
    Asm main(0x0100);
    main.mov(SP, 0x30);
    aes_load(main, EncLoadKey, KeyAt);
    main.nops(60).copy(0x40, S0CON).mov(S0CON, 0x00).mov(IEN0, 0x90);
    aes_load(main, enc_load_iv(ModeCbc), IvAt);
    main.nops(60).copy(0x41, ENCCS).copy(0x42, S0CON);
    const std::uint16_t end = main.here();
    main.halt();

    Cc2530F256 device;
    load(device, base_program(main));
    write_idata(device, KeyAt, NistKey);
    write_idata(device, IvAt, NistIv);
    run_to_pc(device, end);
    expect(device.read_idata(0x40), 0x03, "key load: ENCIF_1 | ENCIF_0");
    expect(device.read_idata(EncIsrCount), 1, "IV load: one ENC interrupt");
    expect(device.read_idata(0x41), 0x0e, "IV loaded: CBC, CMD = 11, RDY");
    expect(device.read_idata(0x42) & 0x03, 0x00, "ENC ISR cleared the IV-load pair");
}

// 15.4-15.5: two 128-bit blocks through ENCDI/ENCDO by the CPU. CBC uses
// one start per 16-byte block; CFB, OFB and CTR load and read four 32-bit
// sub-blocks per start. The IV (or CTR counter block) is loaded through
// ENCDI.
void aes_mode_case(std::uint8_t command, const std::vector<std::uint8_t> &iv,
                   const std::vector<std::uint8_t> &input, const std::vector<std::uint8_t> &expected,
                   const std::source_location loc = std::source_location::current()) {
    const std::uint8_t mode = command >> 4 & 0x07;
    const bool sub_blocks = mode == ModeCfb || mode == ModeOfb || mode == ModeCtr;
    Asm main(0x0100);
    main.mov(SP, 0x30);
    aes_load(main, EncLoadKey, KeyAt);
    aes_load(main, enc_load_iv(mode), IvAt);
    main.movr(0, InAt).movr(1, OutAt);
    for (unsigned block = 0; block < 2; ++block)
        main.mov(ENCCS, command)
            .movr(2, sub_blocks ? 4 : 16)
            .movr(3, sub_blocks ? 4 : 1)
            .lcall(AesBlock);
    main.copy(0x40, ENCCS);
    const std::uint16_t end = main.here();
    main.halt();

    Cc2530F256 device;
    load(device, base_program(main));
    write_idata(device, KeyAt, NistKey);
    write_idata(device, IvAt, iv);
    write_idata(device, InAt, input);
    run_to_pc(device, end, loc);
    expect_bytes(idata(device, OutAt, 32), expected, loc);
    expect(device.read_idata(0x40), static_cast<std::uint8_t>((command & 0x76) | 0x08), "ENCCS when done",
           loc);
}

void test_aes_modes_cpu() {
    aes_mode_case(enc_encrypt(ModeCbc), NistIv, NistPlain, NistCbc);
    aes_mode_case(enc_decrypt(ModeCbc), NistIv, NistCbc, NistPlain);
    aes_mode_case(enc_encrypt(ModeCfb), NistIv, NistPlain, NistCfb);
    aes_mode_case(enc_decrypt(ModeCfb), NistIv, NistCfb, NistPlain);
    aes_mode_case(enc_encrypt(ModeOfb), NistIv, NistPlain, NistOfb);
    aes_mode_case(enc_decrypt(ModeOfb), NistIv, NistOfb, NistPlain);
    aes_mode_case(enc_encrypt(ModeCtr), NistCounter, NistPlain, NistCtr);
    aes_mode_case(enc_decrypt(ModeCtr), NistCounter, NistCtr, NistPlain);
    aes_mode_case(enc_encrypt(ModeEcb), NistIv, NistPlain,
                  bytes_of("3ad77bb40d7a3660a89ecaf32466ef97f5d3d58503b9699de785895a96fdbaaf"));
}

// 15.6: CBC-MAC - all blocks but the last in CBC-MAC mode (no output), the
// last in CBC mode; its output is the MAC (= the last CBC ciphertext block
// for the same key/IV).
void test_aes_cbc_mac() {
    Asm main(0x0100);
    main.mov(SP, 0x30);
    aes_load(main, EncLoadKey, KeyAt);
    aes_load(main, enc_load_iv(ModeCbcMac), IvAt);
    main.mov(S0CON, 0x00); // acknowledge the load ENCIF pairs
    aes_load(main, enc_encrypt(ModeCbcMac), InAt); // block 1: R0 continues at InAt + 16
    main.copy(0x40, ENCCS).copy(0x41, S0CON).mov(S0CON, 0x00);
    main.mov(ENCCS, enc_encrypt(ModeCbc)).movr(2, 16).lcall(AesIn);
    main.movr(1, OutAt).movr(2, 16).lcall(AesOut).copy(0x42, ENCCS);
    const std::uint16_t end = main.here();
    main.halt();

    Cc2530F256 device;
    load(device, base_program(main));
    write_idata(device, KeyAt, NistKey);
    write_idata(device, IvAt, NistIv);
    write_idata(device, InAt, NistPlain);
    write_idata(device, OutAt, filled(16, 0xee));
    run_to_pc(device, end);
    expect_bytes(idata(device, 0x40, 3), {0x58, 0x03, 0x08});
    expect_bytes(idata(device, OutAt, 16), std::vector<std::uint8_t>(NistCbc.begin() + 16, NistCbc.end()));
}

// 15.9: data in and out by DMA. Channel 0 moves bytes to ENCDI (SFR alias
// 0x70B1) on ENC_DW (29), channel 1 from ENCDO (0x70B2) on ENC_UP (30);
// the ENC interrupt (15.8) starts the second block.
void aes_dma_case(std::uint8_t command, const std::vector<std::uint8_t> &iv,
                  const std::vector<std::uint8_t> &expected,
                  const std::source_location loc = std::source_location::current()) {
    const std::uint8_t mode = command >> 4 & 0x07;
    Asm main(0x0100);
    main.mov(SP, 0x30);
    aes_load(main, EncLoadKey, KeyAt);
    aes_load(main, enc_load_iv(mode), IvAt);
    Desc down = desc(0x0400, 0x7000 | ENCDI, 32);
    down.tmode = Single;
    down.trig = TrigEncDw;
    down.destinc = Inc0;
    Desc up = desc(0x7000 | ENCDO, 0x0500, 32);
    up.tmode = Single;
    up.trig = TrigEncUp;
    up.srcinc = Inc0;
    main.descriptor(0, down).descriptor(1, up).point_descriptors();
    // Acknowledge the key/IV load ENCIF pairs before enabling ENCIE.
    main.mov(S0CON, 0x00);
    main.mov(EncBlocksLeft, 1).mov(EncNextCommand, command).mov(IEN0, 0x90);
    main.mov(DMAARM, 0x03).nops(20).mov(ENCCS, command);
    const std::uint16_t wait = main.here();
    main.op({0xe5, EncIsrCount, 0xb4, 0x02}); // MOV A,count; CJNE A,#2,wait
    const std::uint8_t back = main.rel(wait, 1);
    main.op({back});
    main.nops(80);
    const std::uint16_t end = main.here();
    main.halt();

    Cc2530F256 device;
    load(device, base_program(main));
    write_idata(device, KeyAt, NistKey);
    write_idata(device, IvAt, iv);
    write_xdata(device, 0x0400, NistPlain);
    run_to_pc(device, end, loc);
    expect_bytes(xdata(device, 0x0500, 32), expected, loc);
    expect(device.read_idata(EncIsrCount), 2, "ENC interrupts", loc);
    expect(device.read_sfr(DMAIRQ), 0x03, "both channels done", loc);
    expect(device.read_sfr(DMAARM), 0x00, "both channels disarmed", loc);
}

void test_aes_dma() {
    aes_dma_case(enc_encrypt(ModeCbc), NistIv, NistCbc);
    aes_dma_case(enc_encrypt(ModeCtr), NistCounter, NistCtr);
}

// 15.2 / 15.10 / 4.6: the AES registers, key and IV are lost in PM2/PM3
// (not retained); PM1 keeps them.
void aes_power_mode_case(bool pm2) {
    Asm main(0x0100);
    main.mov(SP, 0x30).mov(IEN0, 0xa0); // EA | STIE (wake-up source)
    aes_load(main, EncLoadKey, KeyAt);
    main.mov(ENCCS, enc_encrypt(ModeEcb)).movr(0, InAt).movr(1, OutAt).movr(2, 16).movr(3, 1).lcall(AesBlock);
    main.copy(0x40, ENCCS).mov(SLEEPCMD, pm2 ? 0x06 : 0x05).mov(PCON, 0x01).nops(1);
    main.copy(0x41, ENCCS);
    main.mov(ENCCS, enc_encrypt(ModeEcb)).movr(0, InAt).movr(2, 16).movr(3, 1).lcall(AesBlock);
    const std::uint16_t end = main.here();
    main.halt();

    Cc2530F256 device;
    load(device, base_program(main));
    write_idata(device, KeyAt, FipsKey);
    write_idata(device, InAt, filled(16, 0x00));
    // SWRU191F 11.2: EA = STIE = 1 and the default sleep timer compare
    // (0xFFFFFF) would wake PM1/PM2 after 2^24 - 1 LF periods, so bound the run.
    Cc2530RunResult result = device.run(100'000, 10'000'000);
    CHECK(result.error.empty());
    CHECK(result.sleeping);
    CHECK(device.power_mode() == (pm2 ? Cc2530PowerMode::Pm2 : Cc2530PowerMode::Pm1));
    expect(device.read_sfr(ENCCS), pm2 ? 0x08 : 0x48, "ENCCS in the power mode");
    device.write_sfr(IRCON, 0x80); // sleep timer flag wakes PM1/PM2
    run_to_pc(device, end);
    expect_bytes(idata(device, 0x40, 2), {0x48, static_cast<std::uint8_t>(pm2 ? 0x08 : 0x48)});
    // AES-128(key, 0^128): key 000102..0f before sleep; after PM2 the key
    // is the reset value 0^128.
    expect_bytes(idata(device, OutAt, 16), bytes_of("c6a13b37878f5b826f4f8162a1c8d879"));
    expect_bytes(idata(device, OutAt + 16, 16),
                 bytes_of(pm2 ? "66e94bd4ef8a2c3b884cfa59ca342b2e" : "c6a13b37878f5b826f4f8162a1c8d879"));
}

void test_aes_power_modes() {
    aes_power_mode_case(true);
    aes_power_mode_case(false);
}

// An AES key load, ECB block and ENC interrupts (key load, encrypt,
// decrypt) are identical in fast, step and trace runs.
void test_aes_run_mode_equivalence() {
    std::uint16_t key_load = 0, start = 0, end = 0;
    const Flash flash = base_program(aes_ecb_program(true, key_load, start, end));
    for_each_mode(
        flash, 1500, [](Cc2530F256 &) {},
        [end](Cc2530F256 &device, Mode) {
            expect(device.pc(), end, "program finished");
            expect_bytes(idata(device, OutAt, 32),
                         bytes_of("69c4e0d86a7b0430d8cdb78070b4c55a00112233445566778899aabbccddeeff"));
            expect(device.read_idata(EncIsrCount), 3, "three ENC interrupts");
        },
        "aes ecb");
}

} // namespace

int main() {
    test_dma_manual_block_timing();
    test_dma_trigger_rules();
    test_dma_vlen_modes();
    test_dma_word_size_and_increments();
    test_dma_repeated_modes();
    test_dma_prev_chain_and_arming_order();
    test_dma_priority_and_round_robin();
    test_dma_abort();
    test_dma_irq_flags();
    test_dma_sfr_alias_xbank_and_external_trigger();
    test_dma_halts_in_pm1();
    test_dma_run_mode_equivalence();
    test_flash_erase_from_sram();
    test_flash_erase_stalls_cpu_from_flash();
    test_flash_word_writes_from_sram();
    test_flash_dma_write();
    test_flash_lock_bits();
    test_flash_run_mode_equivalence();
    test_aes_ecb_cpu();
    test_aes_load_interrupts();
    test_aes_modes_cpu();
    test_aes_cbc_mac();
    test_aes_dma();
    test_aes_power_modes();
    test_aes_run_mode_equivalence();
    return finish("cc2530 dma/flash/aes");
}

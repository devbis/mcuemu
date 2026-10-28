// CC2530F256 flash controller (SWRU191F Chapter 6, 2.2.3, 3.4.1).
//
// Model summary
// - FCTL/FADDRH/FADDRL/FWDATA at 0x6270-0x6273 (6.5).
// - Page erase (6.3): the page addressed by FADDRH[7:1] becomes 0xFF after
//   the flash-page erase time of 20 ms (6, 2.2.3: "Flash-page erase timing
//   20 ms") = 640000 cycles of the 32 MHz time base.
// - Write sequence (6.2.1): FCTL.WRITE starts the state machine; four FWDATA
//   bytes (LSB first) program one 32-bit word at FADDRH:FADDRL; the word
//   write takes 20 us (6, 2.2.3: "Flash-write timing (4 bytes) 20 us") = 640
//   cycles. The four bytes must arrive within 20 us since WRITE was set or
//   FULL last became 0, else the sequence times out (WRITE = BUSY = 0).
//   Programming only clears bits (6.2: "the 0-bits are programmed to 0 and
//   the 1-bits are ignored").
// - Lock bits (2.2.3, 3.4.1): one PAGELOCK bit per page in the last 16 bytes
//   of the flash, 0 = locked; the lock-bit page itself is implicitly locked
//   when not in debug mode (the debug interface is not modeled, so always).
//   A locked page aborts the operation (FCTL.ABORT = 1).
// - Effects are applied deterministically at the completion of the erase /
//   word write; a reset or power loss before that abandons the operation
//   (a partially erased/written flash after power loss is not modeled).
// - The CPU stalls while executing from flash during an operation (6.3.1).
//
// All internal times are HF-clock times; the events are processed by
// Dma::advance() so the FLASH DMA trigger and DMA writes to FWDATA
// interleave at exact cycles.

#include "cc2530_flash.hpp"

#include "cc2530_dma.hpp"

#include "mcs51emu/cc2530f256.hpp"

#include <algorithm>

namespace mcs51emu::cc2530 {

namespace {

constexpr Reg RegFctl = 0x6270;
constexpr Reg RegFaddrl = 0x6271;
constexpr Reg RegFaddrh = 0x6272;
constexpr Reg RegFwdata = 0x6273;

constexpr std::uint8_t FctlBusy = 0x80;
constexpr std::uint8_t FctlFull = 0x40;
constexpr std::uint8_t FctlAbort = 0x20;
constexpr std::uint8_t FctlCm = 0x0c;
constexpr std::uint8_t FctlWrite = 0x02;
constexpr std::uint8_t FctlErase = 0x01;

// SWRU191F Chapter 6 / 2.2.3 flash timing, in cycles of the 32 MHz base.
constexpr std::uint64_t PageEraseCycles = 20 * CyclesPerSecond / 1000;   // 20 ms
constexpr std::uint64_t WordWriteCycles = 20 * CyclesPerSecond / 1000000; // 20 us

constexpr std::uint32_t PageSize = 2048;                       // 6.1
constexpr unsigned PageCount = Cc2530F256::FlashSize / PageSize; // 128
constexpr unsigned WordsPerPage = PageSize / 4;

} // namespace

FlashController::FlashController(Bus &bus) : Peripheral(bus) {
    for (Reg reg = RegFctl; reg <= RegFwdata; ++reg)
        bus_.claim_xreg(reg, this);
}

void FlashController::reset(ResetCause) {
    // 6.5 reset values: FCTL = 0x04 (CM = 01), FADDRH/L = FWDATA = 0. Flash
    // contents are kept by the core; an operation in progress is abandoned.
    op_ = Op::None;
    write_after_erase_ = false;
    erase_end_ = 0;
    erase_page_ = 0;
    window_end_ = 0;
    programming_ = false;
    program_end_ = 0;
    program_address_ = 0;
    std::fill(std::begin(program_data_), std::end(program_data_), 0);
    std::fill(std::begin(buffer_), std::end(buffer_), 0);
    buffered_ = 0;
    full_ = false;
    abort_ = false;
    cache_mode_ = 1;
    bus_.storage(RegFctl) = peek(RegFctl);
}

std::uint8_t FlashController::read(Reg reg) { return peek(reg); }

std::uint8_t FlashController::peek(Reg reg) const {
    switch (reg) {
    case RegFctl: {
        std::uint8_t value = static_cast<std::uint8_t>(cache_mode_ << 2);
        if (op_ != Op::None)
            value |= FctlBusy;
        if (full_)
            value |= FctlFull;
        if (abort_)
            value |= FctlAbort;
        if (op_ == Op::Write || (op_ == Op::Erase && write_after_erase_))
            value |= FctlWrite;
        if (op_ == Op::Erase)
            value |= FctlErase;
        return value;
    }
    case RegFwdata:
        return 0; // R0/W
    default:
        return bus_.storage(reg);
    }
}

void FlashController::write(Reg reg, std::uint8_t value) {
    const std::uint64_t at = access_time();
    switch (reg) {
    case RegFctl:
        // CM[1:0] (R/W): the cache has no functional effect in the emulator
        // (flash read timing is not modeled); the value is stored and read
        // back. Modes other than the reset mode 01 are counted as unmodeled.
        if (((value & FctlCm) >> 2) != 1)
            bus_.note_unmodeled(reg, true);
        cache_mode_ = static_cast<std::uint8_t>((value & FctlCm) >> 2);
        // WRITE and ERASE are R/W1/H0: writing 0 has no effect. "Setting
        // WRITE to 1 when ERASE is 1 has no effect", "Setting ERASE to 1
        // when WRITE is 1 has no effect" (6.5); a new operation can only
        // start while the controller is idle.
        if (op_ == Op::None) {
            if (value & FctlErase)
                start_erase(at, (value & FctlWrite) != 0);
            else if (value & FctlWrite)
                start_write(at);
        }
        break;
    case RegFwdata:
        // "This register can only be written to when FCTL.WRITE is 1";
        // "writes to FWDATA are ignored when the FULL flag is set" (6.5).
        // Model-defined: during the erase phase of an ERASE+WRITE command
        // FWDATA is not accepted yet.
        if (op_ == Op::Write && !full_) {
            buffer_[buffered_++] = value;
            if (buffered_ == 4) {
                buffered_ = 0;
                if (!programming_)
                    start_word(at, buffer_);
                else
                    full_ = true; // FULL goes high after the last byte
            }
        }
        break;
    default: // FADDRL, FADDRH: R/W
        bus_.storage(reg) = value;
        break;
    }
    bus_.storage(RegFctl) = peek(RegFctl);
}

std::uint64_t FlashController::deadline() const { return bus_.hf_deadline(next_event()); }

void FlashController::service(std::uint64_t) { bus_.soc().dma->advance(bus_.hf_now()); }

void FlashController::power_mode_changed(PowerMode, PowerMode to) {
    // 6.2.1 / 6.3: "Power mode 1, 2, or 3 must not be entered while writing
    // to the flash" / "while erasing a page". The real outcome is undefined;
    // the model freezes the operation with the HF clock (hf_deadline is
    // Never) and counts the violation as unmodeled.
    if (to != PowerMode::Active && to != PowerMode::Idle && op_ != Op::None)
        bus_.note_unmodeled(RegFctl, true);
}

std::uint64_t FlashController::access_time() const { return bus_.soc().dma->access_time(); }

std::uint64_t FlashController::next_event() const {
    switch (op_) {
    case Op::Erase:
        return erase_end_;
    case Op::Write:
        return programming_ ? program_end_ : window_end_;
    default:
        return Never;
    }
}

void FlashController::process_until(std::uint64_t limit) {
    for (std::uint64_t at = next_event(); at <= limit; at = next_event()) {
        if (op_ == Op::Erase)
            finish_erase(at);
        else if (programming_)
            finish_word(at);
        else
            end_write(); // no complete word within 20 us: timed out (6.2.1)
    }
    bus_.storage(RegFctl) = peek(RegFctl);
}

bool FlashController::page_locked(unsigned page) const {
    // SWRU191F 3.4.1 / Table 3-5: PAGELOCK[FLASH_PAGES-2:0] start at the
    // first of the last 16 flash bytes, bit 0 of the first byte = page 0;
    // 0 = locked. The lock-bit page (the last page) has no lock bit and is
    // "implicitly locked when not in debug mode" (2.2.3).
    if (page >= PageCount - 1)
        return true;
    const std::uint8_t bits = bus_.flash()[Cc2530F256::LockBitsStart + page / 8];
    return ((bits >> (page % 8)) & 1u) == 0;
}

std::uint16_t FlashController::faddr() const {
    return static_cast<std::uint16_t>(bus_.storage(RegFaddrh) << 8 | bus_.storage(RegFaddrl));
}

void FlashController::set_faddr(std::uint16_t word_address) {
    bus_.storage(RegFaddrh) = static_cast<std::uint8_t>(word_address >> 8);
    bus_.storage(RegFaddrl) = static_cast<std::uint8_t>(word_address);
}

void FlashController::stall_if_executing_from_flash(std::uint64_t until) {
    // 6.3.1: "while executing program code from within flash memory, when a
    // flash erase or write operation is initiated the CPU stalls, and
    // program execution resumes from the next instruction when the flash
    // controller has completed the operation". Model-defined: checked when
    // an erase, a write sequence or a word write starts; a write sequence
    // stalls the CPU word by word (each word start extends the stall).
    if (bus_.cpu_executing_from_flash())
        bus_.stall_cpu_until(bus_.hf_deadline(until));
}

void FlashController::request_data() {
    // 6.4: "Four trigger pulses are generated" when the controller is ready
    // to accept new FWDATA data.
    for (unsigned i = 0; i < 4; ++i)
        bus_.dma_trigger(trigger::Flash);
}

void FlashController::start_erase(std::uint64_t at, bool then_write) {
    // ABORT "is cleared when a write or page erase is started" (6.5).
    abort_ = false;
    const unsigned page = bus_.storage(RegFaddrh) >> 1; // FADDRH[7:1] (6.1)
    if (page_locked(page)) {
        // Model-defined: an aborted erase (and the write that would follow
        // it) ends at once with ABORT = 1, BUSY = ERASE = WRITE = 0.
        abort_ = true;
        return;
    }
    op_ = Op::Erase;
    erase_page_ = page;
    write_after_erase_ = then_write;
    erase_end_ = at + PageEraseCycles;
    stall_if_executing_from_flash(erase_end_);
}

void FlashController::finish_erase(std::uint64_t at) {
    // Model-defined: the page is latched when the erase starts.
    const std::uint32_t begin = erase_page_ * PageSize;
    std::vector<std::uint8_t> &flash = bus_.flash();
    std::fill(flash.begin() + begin, flash.begin() + begin + PageSize, 0xff);
    bus_.flash_changed(begin, begin + PageSize);
    op_ = Op::None;
    const bool then_write = write_after_erase_;
    write_after_erase_ = false;
    // 6.3: "the page erase is performed before the page-write operation
    // starts".
    if (then_write)
        start_write(at);
}

void FlashController::start_write(std::uint64_t at) {
    abort_ = false;
    op_ = Op::Write;
    buffered_ = 0;
    full_ = false;
    programming_ = false;
    // 6.2.1 step 3: four FWDATA writes within 20 us.
    window_end_ = at + WordWriteCycles;
    stall_if_executing_from_flash(window_end_);
    // 6.2.3: "starting a flash write by setting FCTL.WRITE to 1 triggers the
    // first DMA transfer".
    request_data();
}

void FlashController::start_word(std::uint64_t at, const std::uint8_t *data) {
    const std::uint16_t address = faddr();
    if (page_locked(address / WordsPerPage)) {
        // 6.2.1 step 5: "If the 4 bytes could not be written to the flash due
        // to the page being locked, FCTL.BUSY (and FCTL.WRITE) are 0 and
        // FCTL.ABORT is 1."
        abort_ = true;
        end_write();
        return;
    }
    programming_ = true;
    program_address_ = address;
    std::copy(data, data + 4, program_data_);
    program_end_ = at + WordWriteCycles;
    // FULL is 0 again: the next four bytes are due within 20 us (6.2.1).
    window_end_ = program_end_;
    stall_if_executing_from_flash(program_end_);
    request_data();
}

void FlashController::finish_word(std::uint64_t at) {
    std::vector<std::uint8_t> &flash = bus_.flash();
    const std::uint32_t begin = static_cast<std::uint32_t>(program_address_) * 4;
    for (unsigned i = 0; i < 4; ++i)
        flash[begin + i] = static_cast<std::uint8_t>(flash[begin + i] & program_data_[i]);
    bus_.flash_changed(begin, begin + 4);
    programming_ = false;
    // 6.2: words are programmed serially from the start address.
    // Model-defined: FADDRH:FADDRL increments (16-bit) when a word is done.
    set_faddr(static_cast<std::uint16_t>(program_address_ + 1));
    if (full_) {
        full_ = false;
        start_word(at, buffer_);
    } else {
        end_write(); // the next word did not arrive in time (6.2.1 step 5)
    }
}

void FlashController::end_write() {
    op_ = Op::None;
    write_after_erase_ = false;
    programming_ = false;
    full_ = false;
    buffered_ = 0;
}

} // namespace mcs51emu::cc2530

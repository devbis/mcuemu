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
//   word write. A reset or power cut before that ends the operation with the
//   configured model-defined outcome (SWRU191F does not specify it):
//   abandon (default, no flash change), partial (a seeded subset of the bits
//   the operation changes lands, in proportion to the elapsed fraction of
//   the 20 ms / 20 us operation time) or complete.
// - The CPU stalls while executing from flash during an operation (6.3.1).
// - 6.2.2 multiple-write limits since the last page erase: a 0 written at
//   most twice to a bit, a 32-bit word written at most 8 times, a page at
//   most 1024 times. Violations are counted per word write; the default
//   policy keeps the deterministic AND, the model-defined `nondeterministic`
//   policy gives the violating word a seeded pseudo-random value.
// - Fault injection: power cuts at a virtual time and triggers on the nth
//   erase / word write; per-page erase and write counters and an optional
//   operation log (docs/cc2530.md "Flash fault injection").
//
// All internal times are HF-clock times; the events are processed by
// Dma::advance() so the FLASH DMA trigger and DMA writes to FWDATA
// interleave at exact cycles.

#include "cc2530_flash.hpp"

#include "cc2530_dma.hpp"

#include "mcs51emu/cc2530f256.hpp"

#include <algorithm>
#include <bit>
#include <stdexcept>

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

// SWRU191F 6.2.2 limits since the last page erase.
constexpr unsigned MaxZeroWritesPerBit = 2;
constexpr unsigned MaxWritesPerWord = 8;
constexpr std::uint32_t MaxWritesPerPage = 1024;

// Draw index of the nondeterministic word value; indices 0-31 are the
// per-bit draws of a partial word write with the same operation serial.
constexpr std::uint64_t NondeterministicIndex = 32;

std::uint64_t mix(std::uint64_t x) {
    x += 0x9e3779b97f4a7c15ull;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
    return x ^ (x >> 31);
}

std::uint32_t load_word(const std::vector<std::uint8_t> &flash, std::uint32_t begin) {
    return static_cast<std::uint32_t>(flash[begin]) |
           static_cast<std::uint32_t>(flash[begin + 1]) << 8 |
           static_cast<std::uint32_t>(flash[begin + 2]) << 16 |
           static_cast<std::uint32_t>(flash[begin + 3]) << 24;
}

} // namespace

std::uint64_t fault_draw(std::uint64_t seed, std::uint64_t stream, std::uint64_t index) {
    return mix(mix(seed ^ mix(stream)) + index);
}

FlashController::FlashController(Bus &bus) : Peripheral(bus) {
    for (Reg reg = RegFctl; reg <= RegFwdata; ++reg)
        bus_.claim_xreg(reg, this);
}

void FlashController::reset(ResetCause) {
    // 6.5 reset values: FCTL = 0x04 (CM = 01), FADDRH/L = FWDATA = 0. Flash
    // contents are kept by the core; an operation in progress has already
    // been given its interrupted outcome (interrupt_operation()). The fault
    // model state (configuration, 6.2.2 tracking, scheduled cuts, triggers,
    // log) is kept: it is not chip state.
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

std::uint64_t FlashController::deadline() const {
    // The fault deadlines stay out of next_event(): Dma::advance() only
    // processes the controller's own operation events.
    return std::min(bus_.hf_deadline(next_event()), fault_deadline());
}

void FlashController::service(std::uint64_t now) {
    // Operation events first: an operation ending at the very cycle of a cut
    // completes.
    bus_.soc().dma->advance(bus_.hf_now());
    fire_due_faults(now);
}

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
        ++op_serial_;
        log_op(at, Cc2530FlashOpKind::PageErase, page * PageSize, Cc2530FlashOutcome::Aborted,
               Cc2530FlashCut::None, ResetCause::PowerOn, 0, 0, 0);
        return;
    }
    op_ = Op::Erase;
    erase_page_ = page;
    write_after_erase_ = then_write;
    erase_end_ = at + PageEraseCycles;
    erase_serial_ = ++op_serial_;
    stall_if_executing_from_flash(erase_end_);
    op_started(Cc2530FlashOpKind::PageErase, page, at);
}

void FlashController::finish_erase(std::uint64_t at) {
    // Model-defined: the page is latched when the erase starts.
    const std::uint32_t begin = erase_page_ * PageSize;
    std::vector<std::uint8_t> &flash = bus_.flash();
    std::uint32_t changed = 0;
    for (std::uint32_t i = begin; i < begin + PageSize; ++i)
        changed += static_cast<std::uint32_t>(std::popcount(static_cast<std::uint8_t>(~flash[i])));
    std::fill(flash.begin() + begin, flash.begin() + begin + PageSize, 0xff);
    bus_.flash_changed(begin, begin + PageSize);
    op_ = Op::None;
    erase_applied(erase_page_, 1);
    log_op(at, Cc2530FlashOpKind::PageErase, begin, Cc2530FlashOutcome::Completed,
           Cc2530FlashCut::None, ResetCause::PowerOn, PageEraseCycles, changed, 0);
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
        ++op_serial_;
        log_op(at, Cc2530FlashOpKind::WordWrite, static_cast<std::uint32_t>(address) * 4,
               Cc2530FlashOutcome::Aborted, Cc2530FlashCut::None, ResetCause::PowerOn, 0, 0, 0);
        end_write();
        return;
    }
    programming_ = true;
    program_address_ = address;
    std::copy(data, data + 4, program_data_);
    program_end_ = at + WordWriteCycles;
    program_serial_ = ++op_serial_;
    // FULL is 0 again: the next four bytes are due within 20 us (6.2.1).
    window_end_ = program_end_;
    stall_if_executing_from_flash(program_end_);
    op_started(Cc2530FlashOpKind::WordWrite, address / WordsPerPage, at);
    request_data();
}

void FlashController::finish_word(std::uint64_t at) {
    std::uint32_t changed = 0;
    const std::uint8_t violations =
        program_word(program_address_, program_data_, program_serial_, WordWriteCycles, changed);
    log_op(at, Cc2530FlashOpKind::WordWrite, static_cast<std::uint32_t>(program_address_) * 4,
           Cc2530FlashOutcome::Completed, Cc2530FlashCut::None, ResetCause::PowerOn,
           WordWriteCycles, changed, violations);
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

// ---- fault model ----

void FlashController::bind(Cc2530Diagnostics &diagnostics,
                           std::function<void(Cc2530FaultAction)> request) {
    diagnostics_ = &diagnostics;
    request_fault_ = std::move(request);
}

void FlashController::interrupt_operation(Cc2530FlashCut cut, ResetCause cause) {
    // SWRU191F does not say what a reset or power loss leaves in a page being
    // erased or a word being programmed (6.2.1 / 6.3 only forbid entering
    // PM1-PM3). Model-defined outcomes, see Cc2530FlashInterrupted. A write
    // sequence waiting for FWDATA bytes has no flash operation in flight; a
    // buffered (FULL) word and the write phase of ERASE+WRITE never start.
    if (op_ == Op::None || (op_ == Op::Write && !programming_))
        return;
    const std::uint64_t hf = bus_.hf_now();
    const Cc2530FlashInterrupted mode = config_.interrupted;
    std::vector<std::uint8_t> &flash = bus_.flash();
    std::uint32_t changed = 0;
    if (op_ == Op::Erase) {
        ++diagnostics_->flash_interrupted_erases;
        const std::uint64_t start = erase_end_ - PageEraseCycles;
        const std::uint64_t elapsed = std::min(hf - std::min(hf, start), PageEraseCycles);
        const std::uint32_t begin = erase_page_ * PageSize;
        Cc2530FlashOutcome outcome = Cc2530FlashOutcome::Abandoned;
        if (mode == Cc2530FlashInterrupted::Partial) {
            // Model-defined: every 0 bit of the page (bit index b = 8 * byte
            // offset + bit) is erased to 1 when draw(seed, erase serial, b)
            // mod 20 ms < elapsed, i.e. with probability elapsed / 20 ms.
            for (std::uint32_t i = 0; i < PageSize; ++i) {
                std::uint8_t byte = flash[begin + i];
                for (unsigned bit = 0; bit < 8; ++bit) {
                    if (byte & (1u << bit))
                        continue;
                    if (fault_draw(config_.seed, erase_serial_, i * 8u + bit) % PageEraseCycles <
                        elapsed) {
                        byte = static_cast<std::uint8_t>(byte | (1u << bit));
                        ++changed;
                    }
                }
                flash[begin + i] = byte;
            }
            bus_.flash_changed(begin, begin + PageSize);
            // A partial erase is no erase: the 6.2.2 tracking goes on.
            outcome = Cc2530FlashOutcome::Partial;
        } else if (mode == Cc2530FlashInterrupted::Complete) {
            for (std::uint32_t i = begin; i < begin + PageSize; ++i)
                changed += static_cast<std::uint32_t>(
                    std::popcount(static_cast<std::uint8_t>(~flash[i])));
            std::fill(flash.begin() + begin, flash.begin() + begin + PageSize, 0xff);
            bus_.flash_changed(begin, begin + PageSize);
            erase_applied(erase_page_, 1);
            outcome = Cc2530FlashOutcome::Forced;
        }
        log_op(hf, Cc2530FlashOpKind::PageErase, begin, outcome, cut, cause, elapsed, changed, 0);
    } else {
        ++diagnostics_->flash_interrupted_writes;
        const std::uint64_t start = program_end_ - WordWriteCycles;
        const std::uint64_t elapsed = std::min(hf - std::min(hf, start), WordWriteCycles);
        Cc2530FlashOutcome outcome = Cc2530FlashOutcome::Abandoned;
        std::uint8_t violations = 0;
        if (mode == Cc2530FlashInterrupted::Partial) {
            violations =
                program_word(program_address_, program_data_, program_serial_, elapsed, changed);
            outcome = Cc2530FlashOutcome::Partial;
        } else if (mode == Cc2530FlashInterrupted::Complete) {
            violations = program_word(program_address_, program_data_, program_serial_,
                                      WordWriteCycles, changed);
            outcome = Cc2530FlashOutcome::Forced;
        }
        log_op(hf, Cc2530FlashOpKind::WordWrite, static_cast<std::uint32_t>(program_address_) * 4,
               outcome, cut, cause, elapsed, changed, violations);
    }
    // The operation is over; reset() clears the rest of the controller state.
    op_ = Op::None;
    write_after_erase_ = false;
    programming_ = false;
    full_ = false;
    buffered_ = 0;
}

void FlashController::flash_replaced() {
    op_ = Op::None;
    write_after_erase_ = false;
    programming_ = false;
    full_ = false;
    buffered_ = 0;
    for (auto &wear : wear_)
        wear.reset();
    log_.clear();
    op_serial_ = erase_serial_ = program_serial_ = 0;
}

void FlashController::time_restarted() {
    std::erase_if(triggers_, [](const Trigger &trigger) { return trigger.fire_hf != Never; });
}

void FlashController::erase_applied(unsigned first_page, unsigned count) {
    for (unsigned page = first_page; page < first_page + count && page < PageCount; ++page) {
        ++diagnostics_->flash_page_erases[page];
        wear_[page].reset(); // 6.2.2 limits count "since the last page erase"
    }
}

void FlashController::record_erase(unsigned first_page, unsigned count, Cc2530FlashOpKind op,
                                   Cc2530FlashOutcome outcome, std::uint64_t elapsed_cycles) {
    if (first_page >= PageCount || count == 0 || count > PageCount - first_page)
        throw std::out_of_range("flash erase record outside the 128 pages");
    erase_applied(first_page, count);
    log_op(bus_.hf_now(), op, first_page * PageSize, outcome, Cc2530FlashCut::None,
           ResetCause::PowerOn, elapsed_cycles, 0, 0);
}

void FlashController::schedule_power_cut(std::uint64_t at_cycle) {
    cuts_.insert(std::upper_bound(cuts_.begin(), cuts_.end(), at_cycle), at_cycle);
    bus_.reschedule();
}

void FlashController::add_trigger(const Cc2530FlashTrigger &trigger) {
    if (trigger.nth == 0)
        throw std::invalid_argument("flash trigger: nth counts from 1");
    if (trigger.page && *trigger.page >= PageCount)
        throw std::out_of_range("flash trigger: page outside 0-127");
    triggers_.push_back(Trigger{trigger, 0, Never});
}

std::vector<Cc2530FlashLogEntry> FlashController::take_log() {
    std::vector<Cc2530FlashLogEntry> entries;
    entries.swap(log_);
    return entries;
}

std::uint8_t FlashController::program_word(std::uint16_t word_address, const std::uint8_t *data,
                                           std::uint64_t serial, std::uint64_t elapsed,
                                           std::uint32_t &changed) {
    const std::uint32_t begin = static_cast<std::uint32_t>(word_address) * 4;
    const unsigned page = begin / PageSize;
    const unsigned word = (begin % PageSize) / 4;
    std::unique_ptr<PageWear> &slot = wear_[page];
    if (!slot)
        slot = std::make_unique<PageWear>();
    PageWear &wear = *slot;
    const std::uint32_t value = static_cast<std::uint32_t>(data[0]) |
                                static_cast<std::uint32_t>(data[1]) << 8 |
                                static_cast<std::uint32_t>(data[2]) << 16 |
                                static_cast<std::uint32_t>(data[3]) << 24;
    // 6.2.2: every word write counts, whatever its data; every 0 data bit is
    // a "0 written" to that bit, whether or not it is already 0.
    static_assert(MaxZeroWritesPerBit == 2, "zero_once/zero_twice track up to two 0-writes");
    const std::uint32_t zeros = ~value;
    std::uint8_t violations = 0;
    if (zeros & wear.zero_twice[word])
        violations |= Cc2530FlashBitLimit;
    wear.zero_twice[word] |= zeros & wear.zero_once[word];
    wear.zero_once[word] |= zeros;
    if (wear.word_writes[word] < 0xff)
        ++wear.word_writes[word];
    if (wear.word_writes[word] > MaxWritesPerWord)
        violations |= Cc2530FlashWordLimit;
    if (++wear.page_writes > MaxWritesPerPage)
        violations |= Cc2530FlashPageLimit;
    if (violations & Cc2530FlashBitLimit)
        ++diagnostics_->flash_bit_limit_violations;
    if (violations & Cc2530FlashWordLimit)
        ++diagnostics_->flash_word_limit_violations;
    if (violations & Cc2530FlashPageLimit)
        ++diagnostics_->flash_page_limit_violations;
    ++diagnostics_->flash_page_writes[page];

    std::vector<std::uint8_t> &flash = bus_.flash();
    const std::uint32_t old = load_word(flash, begin);
    // 6.2: "the 0-bits are programmed to 0 and the 1-bits are ignored".
    std::uint32_t target = old & value;
    if (violations && config_.limit_policy == Cc2530FlashLimitPolicy::Nondeterministic) {
        // 6.2.2: "The state of any bit of a 32-bit flash word is
        // nondeterministic if these limitations are violated". Model-defined
        // fault injection: the word takes a seeded pseudo-random value.
        target = static_cast<std::uint32_t>(fault_draw(config_.seed, serial, NondeterministicIndex));
    }
    std::uint32_t result = target;
    if (elapsed < WordWriteCycles) {
        // Model-defined partial word write: each bit that would change lands
        // when draw(seed, write serial, bit) mod 20 us < elapsed.
        result = old;
        for (unsigned bit = 0; bit < 32; ++bit) {
            const std::uint32_t mask = 1u << bit;
            if (((old ^ target) & mask) &&
                fault_draw(config_.seed, serial, bit) % WordWriteCycles < elapsed)
                result = (result & ~mask) | (target & mask);
        }
    }
    changed = static_cast<std::uint32_t>(std::popcount(old ^ result));
    for (unsigned i = 0; i < 4; ++i)
        flash[begin + i] = static_cast<std::uint8_t>(result >> (8 * i));
    bus_.flash_changed(begin, begin + 4);
    return violations;
}

void FlashController::op_started(Cc2530FlashOpKind op, unsigned page, std::uint64_t at) {
    bool armed = false;
    for (Trigger &trigger : triggers_) {
        if (trigger.fire_hf != Never)
            continue;
        const Cc2530FlashOpFilter filter = trigger.spec.op;
        if ((filter == Cc2530FlashOpFilter::Erase && op != Cc2530FlashOpKind::PageErase) ||
            (filter == Cc2530FlashOpFilter::Write && op != Cc2530FlashOpKind::WordWrite))
            continue;
        if (trigger.spec.page && *trigger.spec.page != page)
            continue;
        if (++trigger.matched == trigger.spec.nth) {
            trigger.fire_hf = at + trigger.spec.delay_cycles;
            armed = true;
        }
    }
    if (armed)
        bus_.reschedule();
}

void FlashController::log_op(std::uint64_t at_hf, Cc2530FlashOpKind op, std::uint32_t address,
                             Cc2530FlashOutcome outcome, Cc2530FlashCut cut, ResetCause cause,
                             std::uint64_t elapsed, std::uint32_t changed,
                             std::uint8_t violations) {
    if (!config_.log)
        return;
    // HF times map to virtual time through the current offset between the
    // two domains (an operation never spans PM1-PM3 time).
    const std::uint64_t hf = bus_.hf_now();
    const std::uint64_t now = bus_.now();
    const std::uint64_t back = hf > at_hf ? hf - at_hf : 0;
    Cc2530FlashLogEntry entry;
    entry.cycle = now > back ? now - back : 0;
    entry.op = op;
    entry.address = address;
    entry.page = address / PageSize;
    entry.outcome = outcome;
    entry.cut = cut;
    entry.reset_cause = cause;
    entry.elapsed_cycles = elapsed;
    entry.changed_bits = changed;
    entry.violations = violations;
    log_.push_back(entry);
}

std::uint64_t FlashController::fault_deadline() const {
    std::uint64_t deadline = cuts_.empty() ? Never : cuts_.front();
    for (const Trigger &trigger : triggers_)
        if (trigger.fire_hf != Never)
            deadline = std::min(deadline, bus_.hf_deadline(trigger.fire_hf));
    return deadline;
}

void FlashController::fire_due_faults(std::uint64_t now) {
    bool power_cut = false;
    bool reset = false;
    while (!cuts_.empty() && cuts_.front() <= now) {
        cuts_.erase(cuts_.begin());
        power_cut = true;
    }
    if (!triggers_.empty()) {
        const std::uint64_t hf = bus_.hf_now();
        std::erase_if(triggers_, [&](const Trigger &trigger) {
            if (trigger.fire_hf == Never || trigger.fire_hf > hf)
                return false;
            (trigger.spec.action == Cc2530FaultAction::PowerCut ? power_cut : reset) = true;
            return true;
        });
    }
    // Several due faults at one boundary act once; a power cut includes a
    // reset.
    if (!request_fault_)
        return;
    if (power_cut)
        request_fault_(Cc2530FaultAction::PowerCut);
    else if (reset)
        request_fault_(Cc2530FaultAction::Reset);
}

} // namespace mcs51emu::cc2530

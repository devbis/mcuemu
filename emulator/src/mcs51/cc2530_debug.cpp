#include "cc2530_debug.hpp"

#include "cc2530_dma.hpp"
#include "cc2530_flash.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace mcs51emu::cc2530 {

namespace {

constexpr std::uint16_t Dbgdata = 0x6260; // Table 8-1 DBG_BW, 7.x register list (R)
constexpr std::uint16_t Chver = 0x6249;
constexpr std::uint16_t Chipid = 0x624a;
constexpr std::uint8_t SfrSp = 0x81;
constexpr std::uint8_t SfrFmap = 0x9f;
// DATA 0xff is the top byte of SRAM, XDATA 0x1fff (2.2.2).
constexpr std::uint16_t DataTop = 0x1fff;
// 3.4.1, Table 3-5: DBGLOCK is bit 7 of the last flash byte (bit 127 of the
// lock-bit structure); 0 = locked.
constexpr std::uint32_t DbglockByte = 0x3ffff;

// Table 3-1 command codes. The three LSBs are don't care, except for the yy
// field of DEBUG_INSTR and the kkk field of BURST_WRITE.
enum Command : std::uint8_t {
    ChipErase = 0x10,
    WrConfig = 0x18,
    RdConfig = 0x20,
    GetPc = 0x28,
    ReadStatus = 0x30,
    SetHwBrkpnt = 0x38,
    Halt = 0x40,
    Resume = 0x48,
    DebugInstr = 0x50,
    StepInstr = 0x58,
    GetBm = 0x60,
    GetChipId = 0x68,
    BurstWrite = 0x80,
};

constexpr std::uint8_t code_of(std::uint8_t command) {
    return static_cast<std::uint8_t>(command & 0xf8u);
}

bool defined(std::uint8_t command) {
    switch (code_of(command)) {
    case ChipErase:
    case WrConfig:
    case RdConfig:
    case GetPc:
    case ReadStatus:
    case SetHwBrkpnt:
    case Halt:
    case Resume:
    case StepInstr:
    case GetBm:
    case GetChipId:
    case BurstWrite:
        return true;
    case DebugInstr:
        return (command & 0x03u) != 0; // yy = 1..3 instruction bytes
    default:
        return false;
    }
}

} // namespace

Debug::Debug(Bus &bus) : Peripheral(bus) { bus.claim_xreg(Dbgdata, this); }

unsigned Debug::input_length(std::uint8_t command, std::uint8_t second) {
    switch (code_of(command)) {
    case WrConfig:
        return 2;
    case SetHwBrkpnt:
        return 4;
    case DebugInstr:
        return 1u + (command & 0x03u);
    case BurstWrite: {
        // Figure 3-5: an 11-bit length kkk:b7-b0; 0 means 2048.
        const unsigned length = ((command & 0x07u) << 8) | second;
        return 2u + (length != 0 ? length : 2048u);
    }
    default:
        return 1;
    }
}

unsigned Debug::output_length(std::uint8_t command) {
    const std::uint8_t code = code_of(command);
    return code == GetPc || code == GetChipId ? 2 : 1;
}

// ---- lifecycle ----

void Debug::reset(ResetCause) {
    bus_.storage(Dbgdata) = 0;
    // 3.4.1: "after the debug-lock bit has changed ... the device must be
    // reset to lock or unlock the debug interface", so DBGLOCK is latched
    // here and DEBUG_LOCKED reports the latched value.
    locked_ = (bus_.flash()[DbglockByte] & 0x80u) == 0;
    entry_edges_ = 0;
    last_dc_ = bus_.soc().io->level(2, 2);
    stack_shadow_ = bus_.xdata_peek(DataTop);
    stack_overflow_ = false;
    apply_config();
}

void Debug::before_reset(ResetCause cause) {
    if (!active_)
        return;
    // Model-defined (SWRU191F does not say how debug mode ends): a power-on
    // or RESET_N reset leaves it; a watchdog or clock-loss reset resets the
    // chip under the debugger, which keeps its connection, configuration
    // and breakpoints (a pending STEP_INSTR then steps the first
    // instruction at 0x0000).
    if (cause == ResetCause::PowerOn || cause == ResetCause::External)
        leave(bus_.now(), true);
}

void Debug::enter() {
    active_ = true;
    // 3.1: the CPU is halted when debug mode is entered.
    halted_ = true;
    halt_by_breakpoint_ = false;
    step_pending_ = false;
    skip_breakpoint_once_ = false;
    waiting_for_step_ = false;
    config_ = ConfigReset;
    breakpoints_ = {};
    erase_enabled_ = true;
    inbox_.clear();
    command_.clear();
    received_ = expected_ = 0;
    burst_accepted_ = false;
    host_busy_ = false;
    host_due_ = 0;
    host_port_free_at_ = bus_.now();
    host_response_.reset();
    wire_reset();
    stack_shadow_ = bus_.xdata_peek(DataTop);
    apply_config();
    bus_.soc().io->peripheral_outputs_changed();
    bus_.reschedule();
}

void Debug::leave(std::uint64_t at, bool resetting) {
    active_ = false;
    halted_ = false;
    halt_by_breakpoint_ = false;
    step_pending_ = false;
    skip_breakpoint_once_ = false;
    waiting_for_step_ = false;
    if (host_busy_) {
        host_busy_ = false;
        host_response_ = Response{{}, false, at};
    }
    inbox_.clear();
    command_.clear();
    received_ = expected_ = 0;
    burst_accepted_ = false;
    wire_reset();
    // A reset in progress resets the DMA, the timers and the pads itself.
    if (!resetting) {
        apply_config();
        bus_.soc().io->peripheral_outputs_changed();
    }
    bus_.reschedule();
}

void Debug::power_mode_changed(PowerMode, PowerMode to) {
    // 3.5 / Table 3-2: with SOFT_POWER_MODE cleared the digital regulator is
    // turned off in PM2/PM3, which "disables debug mode".
    if (active_ && !(config_ & ConfigSoftPowerMode) && (to == PowerMode::Pm2 || to == PowerMode::Pm3))
        leave(bus_.now(), false);
}

// ---- registers ----

std::uint8_t Debug::read(Reg reg) { return peek(reg); }

std::uint8_t Debug::peek(Reg reg) const { return bus_.storage(reg); }

void Debug::write(Reg, std::uint8_t) {
    // DBGDATA is read-only (R); only BURST_WRITE loads it.
}

// ---- scheduling ----

std::uint64_t Debug::deadline() const { return inbox_.empty() ? Never : inbox_.front().at; }

void Debug::service(std::uint64_t now) { drain(now); }

void Debug::drain(std::uint64_t limit) {
    while (!inbox_.empty() && inbox_.front().at <= limit) {
        const Byte byte = inbox_.front();
        inbox_.pop_front();
        process_byte(byte);
    }
    bus_.reschedule();
}

// ---- command engine ----

void Debug::process_byte(const Byte &byte) {
    if (received_ == 0) {
        source_ = byte.source;
        command_.clear();
        expected_ = code_of(byte.value) == BurstWrite ? 2 : input_length(byte.value);
    }
    ++received_;
    const std::uint8_t command = received_ == 1 ? byte.value : command_.front();
    if (code_of(command) != BurstWrite) {
        command_.push_back(byte.value);
        if (received_ == expected_)
            execute(byte.at);
        return;
    }
    // BURST_WRITE: acceptance is decided at the command byte, so each data
    // byte reaches DBGDATA (and raises DBG_BW) as it arrives.
    if (received_ == 1) {
        command_.push_back(byte.value);
        burst_accepted_ = accepted(command);
        note_issued(command);
        return;
    }
    if (received_ == 2) {
        expected_ = input_length(command, byte.value);
        return;
    }
    if (burst_accepted_) {
        bus_.storage(Dbgdata) = byte.value;
        bus_.dma_trigger(trigger::DbgBw);
    }
    if (received_ == expected_) {
        received_ = expected_ = 0;
        const bool ok = burst_accepted_;
        burst_accepted_ = false;
        respond({status_byte()}, ok, byte.at);
    }
}

bool Debug::accepted(std::uint8_t command) const {
    if (!defined(command))
        return false;
    const std::uint8_t code = code_of(command);
    // Table 3-3: during a chip erase the status (and the ID) can be read.
    if (bus_.soc().flash->chip_erase_busy() && code != ReadStatus && code != GetChipId)
        return false;
    // 3.4.1: a locked interface accepts only these three.
    if (locked_ && code != ChipErase && code != ReadStatus && code != GetChipId)
        return false;
    if (code == ChipErase && !erase_enabled_)
        return false;
    // 3.5: a reduced command set in power modes; HALT wakes the chip.
    const bool sleeping = bus_.power_mode() != PowerMode::Active || bus_.sleep_requested();
    if (sleeping && code != ReadStatus && code != Halt && code != GetChipId)
        return false;
    // Table 3-1: RESUME, DEBUG_INSTR and STEP_INSTR need a halted CPU.
    if ((code == Resume || code == DebugInstr || code == StepInstr) && !halted_)
        return false;
    return true;
}

void Debug::note_issued(std::uint8_t command) {
    // Table 3-1 CHIP_ERASE: "If any other command except READ_STATUS is
    // issued, then the use of CHIP_ERASE is disabled" (until debug mode is
    // entered again).
    const std::uint8_t code = code_of(command);
    if (code != ReadStatus && code != ChipErase)
        erase_enabled_ = false;
}

void Debug::execute(std::uint64_t at) {
    const std::vector<std::uint8_t> bytes = std::move(command_);
    command_.clear();
    received_ = expected_ = 0;
    const std::uint8_t command = bytes.front();
    const std::uint8_t code = code_of(command);
    const bool ok = accepted(command);
    note_issued(command);
    if (!ok) {
        // Model-defined: a refused or undefined command shifts out the
        // status byte in every output position.
        const std::uint8_t status = status_byte();
        respond(std::vector<std::uint8_t>(output_length(command), status), false, at);
        return;
    }
    Soc &soc = bus_.soc();
    switch (code) {
    case ChipErase:
        soc.flash->start_chip_erase();
        respond({status_byte()}, true, at); // BUSY is already set (Table 3-3)
        return;
    case WrConfig:
        config_ = static_cast<std::uint8_t>(bytes[1] & ConfigMask);
        apply_config();
        respond({status_byte()}, true, at);
        return;
    case RdConfig:
        respond({config_}, true, at);
        return;
    case GetPc: {
        const std::uint16_t pc = bus_.cpu_pc();
        respond({static_cast<std::uint8_t>(pc >> 8), static_cast<std::uint8_t>(pc)}, true, at);
        return;
    }
    case ReadStatus:
        respond({status_byte()}, true, at);
        return;
    case SetHwBrkpnt: {
        // 3.3.3: bits 5:4 number, bit 3 enable, bits 2:0 bank (address
        // bits 18:16), then address bits 15:8 and 7:0.
        Breakpoint &breakpoint = breakpoints_[(bytes[1] >> 4) & 0x03u];
        breakpoint.enabled = (bytes[1] & 0x08u) != 0;
        breakpoint.bank = static_cast<std::uint8_t>(bytes[1] & 0x07u);
        breakpoint.address = static_cast<std::uint16_t>(bytes[2] << 8 | bytes[3]);
        respond({status_byte()}, true, at);
        return;
    }
    case Halt:
        // 3.5: HALT brings the chip up from a power mode in the halted state.
        if (bus_.power_mode() != PowerMode::Active || bus_.sleep_requested())
            bus_.wake_for_debug_halt();
        if (!halted_) {
            halted_ = true;
            halt_by_breakpoint_ = false;
            check_stack();
            apply_config();
        }
        respond({status_byte()}, true, at);
        return;
    case Resume:
        halted_ = false;
        skip_breakpoint_once_ = true; // leave a breakpoint address
        apply_config();
        soc.flash->cpu_resumed();
        respond({status_byte()}, true, at);
        return;
    case DebugInstr: {
        const std::uint8_t a = bus_.debug_execute(bytes.data() + 1, command & 0x03u);
        respond({a}, true, at);
        return;
    }
    case StepInstr:
        // The response (A) is produced by step_completed() once the core
        // has executed the instruction.
        step_pending_ = true;
        waiting_for_step_ = true;
        apply_config();
        soc.flash->cpu_resumed();
        return;
    case GetBm:
        respond({static_cast<std::uint8_t>(bus_.storage(sfr_reg(SfrFmap)) & 0x07u)}, true, at);
        return;
    case GetChipId:
        respond({bus_.storage(Chipid), bus_.storage(Chver)}, true, at);
        return;
    default:
        respond({status_byte()}, false, at);
        return;
    }
}

void Debug::respond(std::vector<std::uint8_t> bytes, bool ok, std::uint64_t at) {
    if (source_ == Source::Host) {
        host_busy_ = false;
        host_port_free_at_ = at + bytes.size() * HostByteCycles;
        host_response_ = Response{std::move(bytes), ok, at};
        return;
    }
    wire_response_ = std::move(bytes);
}

void Debug::step_completed(std::uint8_t accumulator) {
    if (!step_pending_)
        return;
    step_pending_ = false;
    waiting_for_step_ = false;
    check_stack();
    apply_config();
    respond({accumulator}, true, bus_.now());
}

// ---- status and configuration ----

bool Debug::stack_hit() const {
    // Model-defined approximation of "the CPU writes to DATA memory space
    // at address 0xFF": DATA 0xff changed since the last sample, or SP is
    // 0xff (a push reached it).
    return bus_.xdata_peek(DataTop) != stack_shadow_ || bus_.storage(sfr_reg(SfrSp)) == 0xff;
}

void Debug::check_stack() {
    if (stack_hit())
        stack_overflow_ = true;
    stack_shadow_ = bus_.xdata_peek(DataTop);
}

std::uint8_t Debug::status() const {
    std::uint8_t value = 0;
    if (bus_.soc().flash->chip_erase_busy())
        value |= StatusChipEraseBusy;
    // Table 3-4: 10 = entering a power mode (PCON.IDLE set, not settled),
    // 11 = stable in a power mode. The model wakes instantly, so 01
    // (transition up) is never seen.
    const PowerMode mode = bus_.power_mode();
    if (mode != PowerMode::Active)
        value |= StatusPconIdle | StatusPmActive;
    else if (bus_.sleep_requested())
        value |= StatusPconIdle;
    if (active_ && halted_)
        value |= StatusCpuHalted;
    if (halt_by_breakpoint_)
        value |= StatusHaltStatus;
    if (locked_)
        value |= StatusDebugLocked;
    // The system clock oscillators are off in PM1-PM3 (4.1).
    if (mode == PowerMode::Active || mode == PowerMode::Idle)
        value |= StatusOscillatorStable;
    if (stack_overflow_ || stack_hit())
        value |= StatusStackOverflow;
    return value;
}

std::uint8_t Debug::status_byte() {
    check_stack();
    return status();
}

void Debug::apply_config() {
    // Table 3-2. Model-defined scope: DMA_PAUSE stops every DMA transfer
    // while it is set in debug mode (halted or not); TIMERS_OFF stops Timer
    // 1, 3 and 4 in debug mode; TIMER_SUSPEND stops them while the CPU is
    // halted but not while it executes a STEP_INSTR. Debug instructions take
    // no virtual time, so timers are always "suspended during debug
    // instructions". The sleep timer, the watchdog and the MAC timer keep
    // running.
    bus_.soc().dma->set_paused(active_ && (config_ & ConfigDmaPause) != 0);
    const bool suspend = active_ && ((config_ & ConfigTimersOff) != 0 ||
                                     ((config_ & ConfigTimerSuspend) != 0 && halted_ && !step_pending_));
    bus_.set_timers_suspended(suspend);
}

// ---- hardware breakpoints ----

bool Debug::breakpoint_at(std::uint16_t pc, std::uint8_t fmap) const {
    if (!active_ || halted_ || step_pending_ || skip_breakpoint_once_)
        return false;
    for (const Breakpoint &breakpoint : breakpoints_) {
        // Model-defined: the bank bits (address bits 18:16) are compared
        // only in the banked upper half; the lower half is the common area.
        if (breakpoint.enabled && breakpoint.address == pc &&
            (pc < 0x8000 || breakpoint.bank == (fmap & 0x07u)))
            return true;
    }
    return false;
}

bool Debug::breakpoints_armed() const noexcept {
    if (!active_ || halted_ || step_pending_)
        return false;
    return std::any_of(breakpoints_.begin(), breakpoints_.end(),
                       [](const Breakpoint &breakpoint) { return breakpoint.enabled; });
}

void Debug::breakpoint_hit() {
    halted_ = true;
    halt_by_breakpoint_ = true;
    check_stack();
    apply_config();
}

// ---- DC/DD wire (3.2) ----

void Debug::wire_reset() {
    wire_ = WireState::Input;
    bits_ = 0;
    shift_ = 0;
    wire_command_ = 0;
    wire_received_ = wire_expected_ = 0;
    wire_response_.reset();
    output_.clear();
    output_bit_ = 0;
    dummy_bits_ = 0;
    dd_drive_ = PinDrive::HiZ;
}

void Debug::set_dd(PinDrive drive) {
    if (dd_drive_ == drive)
        return;
    dd_drive_ = drive;
    bus_.soc().io->peripheral_outputs_changed();
}

void Debug::latch_ready() {
    // Figures 3-3/3-4: after the last input bit (and after every 8 wait
    // clocks) the chip pulls DD low when the response is ready, high
    // otherwise. Model-defined: the level changes only at DC falling edges,
    // so it is stable whenever the debugger samples it.
    set_dd(wire_response_ ? PinDrive::Low : PinDrive::High);
}

void Debug::drive_output_bit() {
    const std::uint8_t byte = output_[output_bit_ / 8];
    set_dd(((byte >> (7 - output_bit_ % 8)) & 1u) ? PinDrive::High : PinDrive::Low);
}

void Debug::pads_changed(std::uint64_t at) {
    Io &io = *bus_.soc().io;
    const bool dc = io.level(2, 2);
    if (dc == last_dc_)
        return;
    last_dc_ = dc;
    if (bus_.reset_held()) {
        // 3.1: two falling DC edges while RESET_N is low select debug mode.
        if (!dc)
            ++entry_edges_;
        return;
    }
    if (!active_ || host_busy_)
        return;
    if (dc) {
        // Rising edge: the chip presents the next output bit (Figure 3-1:
        // the debugger samples DD while DC is high).
        if (wire_ == WireState::Wait) {
            if (dd_drive_ == PinDrive::Low && wire_response_) {
                output_ = std::move(*wire_response_);
                wire_response_.reset();
                output_bit_ = 0;
                wire_ = WireState::Output;
                drive_output_bit();
            } else {
                wire_ = WireState::Dummy; // Figure 3-4: eight wait clocks
                dummy_bits_ = 0;
            }
        } else if (wire_ == WireState::Output) {
            drive_output_bit();
        }
        return;
    }
    // Falling edge: the chip samples DD, MSB first.
    switch (wire_) {
    case WireState::Input: {
        shift_ = static_cast<std::uint8_t>(shift_ << 1 | (io.level(2, 1) ? 1u : 0u));
        if (++bits_ < 8)
            return;
        const std::uint8_t value = shift_;
        bits_ = 0;
        shift_ = 0;
        ++wire_received_;
        if (wire_received_ == 1) {
            wire_command_ = value;
            wire_expected_ = code_of(value) == BurstWrite ? 2 : input_length(value);
        } else if (wire_received_ == 2 && code_of(wire_command_) == BurstWrite) {
            wire_expected_ = input_length(wire_command_, value);
        }
        inbox_.push_back(Byte{value, at, Source::Wire});
        // Model-defined: a byte is processed at the falling edge that
        // completes it when the CPU is between instructions; inside an
        // instruction it waits for the boundary (service()).
        if (!bus_.engine_running())
            drain(at);
        else
            bus_.reschedule();
        if (wire_received_ == wire_expected_) {
            wire_received_ = wire_expected_ = 0;
            wire_ = WireState::Wait;
            latch_ready();
        }
        return;
    }
    case WireState::Dummy:
        if (++dummy_bits_ == 8) {
            wire_ = WireState::Wait;
            latch_ready();
        }
        return;
    case WireState::Output:
        if (++output_bit_ == output_.size() * 8) {
            output_.clear();
            output_bit_ = 0;
            wire_ = WireState::Input;
            set_dd(PinDrive::HiZ); // DD turns back to an input
        }
        return;
    case WireState::Wait:
        return;
    }
}

// ---- host port ----

void Debug::host_submit(const std::vector<std::uint8_t> &bytes) {
    if (!active_)
        throw std::logic_error("CC2530 debug interface: the chip is not in debug mode");
    if (host_busy_)
        throw std::logic_error("CC2530 debug interface: a command is outstanding");
    if (wire_ != WireState::Input || bits_ != 0 || wire_received_ != 0 || received_ != 0 ||
        !inbox_.empty())
        throw std::logic_error("CC2530 debug interface: the DC/DD wire is in a command");
    if (bytes.empty())
        throw std::invalid_argument("CC2530 debug interface: empty command");
    unsigned length = input_length(bytes[0]);
    if (code_of(bytes[0]) == BurstWrite)
        length = bytes.size() >= 2 ? input_length(bytes[0], bytes[1]) : 2;
    if (bytes.size() != length)
        throw std::invalid_argument("CC2530 debug interface: command 0x" +
                                    std::string(1, "0123456789abcdef"[bytes[0] >> 4]) +
                                    std::string(1, "0123456789abcdef"[bytes[0] & 15]) +
                                    " takes " + std::to_string(length) + " bytes, got " +
                                    std::to_string(bytes.size()));
    host_response_.reset();
    const std::uint64_t start = std::max(bus_.now(), host_port_free_at_);
    for (std::size_t i = 0; i < bytes.size(); ++i)
        inbox_.push_back(Byte{bytes[i], start + (i + 1) * HostByteCycles, Source::Host});
    host_busy_ = true;
    host_due_ = start + bytes.size() * HostByteCycles;
    bus_.reschedule();
}

std::optional<Debug::Response> Debug::host_take() {
    std::optional<Response> response = std::move(host_response_);
    host_response_.reset();
    return response;
}

} // namespace mcs51emu::cc2530

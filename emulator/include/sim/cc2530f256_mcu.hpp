#pragma once

#include "mcs51emu/cc2530_hit_report.hpp"
#include "mcs51emu/cc2530f256.hpp"
#include "sim/mcu.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim {

// CC2530F256 adapter: pads P0.0-P0.7, P1.0-P1.7, P2.0-P2.4 wired to nets,
// USART0/USART1 in UART mode as uart0/uart1, and the IEEE 802.15.4 radio on a
// shared VirtualAir ("zigbee" only). The peripheral models count time in
// 32 MHz cycles, so the radio can only be attached with the default
// clock_hz of 32 MHz.
//
// read_memory() address spaces (the 8051 has several overlapping 16-bit
// spaces, so each gets its own 32-bit window):
//   0x0000_0000-0x0000_ffff  CODE as the CPU sees it now (FMAP/XMAP applied)
//   0x0001_0000-0x0001_ffff  XDATA (side-effect free; XREGs read as latched)
//   0x0002_0000-0x0002_00ff  IDATA (DATA/IDATA, also XDATA 0x1f00-0x1fff)
//   0x0003_0080-0x0003_00ff  SFRs
//   0x0004_0000-0x0004_003f  CPU snapshot (layout: CpuSnapshot* below)
//   0x0100_0000-0x0103_ffff  physical flash
// write_memory() accepts only XDATA SRAM (0x0001_0000-0x0001_1fff) and IDATA.
class Cc2530F256Mcu final : public Mcu {
public:
    static constexpr std::uint32_t CodeWindow = 0x00000000;
    static constexpr std::uint32_t XdataWindow = 0x00010000;
    static constexpr std::uint32_t IdataWindow = 0x00020000;
    static constexpr std::uint32_t SfrWindow = 0x00030000;
    static constexpr std::uint32_t CpuSnapshotWindow = 0x00040000;
    static constexpr std::uint32_t CpuSnapshotSize = 0x40;
    static constexpr std::uint32_t FlashWindow = 0x01000000;

    // CPU snapshot offsets (multi-byte fields little-endian).
    static constexpr unsigned CpuSnapshotPc = 0x00;        // u16 logical PC
    static constexpr unsigned CpuSnapshotFmap = 0x02;
    static constexpr unsigned CpuSnapshotMemctr = 0x03;
    static constexpr unsigned CpuSnapshotA = 0x04;
    static constexpr unsigned CpuSnapshotB = 0x05;
    static constexpr unsigned CpuSnapshotPsw = 0x06;
    static constexpr unsigned CpuSnapshotSp = 0x07;
    static constexpr unsigned CpuSnapshotDpl = 0x08;
    static constexpr unsigned CpuSnapshotDph = 0x09;
    static constexpr unsigned CpuSnapshotMpage = 0x0a;
    // bit 0 sleeping, bit 1 in an ISR, bit 2 idle self-loop
    static constexpr unsigned CpuSnapshotFlags = 0x0b;
    static constexpr unsigned CpuSnapshotR0 = 0x0c;        // R0-R7, bank PSW.RS
    static constexpr unsigned CpuSnapshotPhysicalPc = 0x14; // u32, 0xffffffff in SRAM
    static constexpr unsigned CpuSnapshotCycles = 0x18;    // u64
    static constexpr unsigned CpuSnapshotInstructions = 0x20; // u64
    static constexpr unsigned CpuSnapshotInterrupts = 0x28; // u32 taken
    static constexpr unsigned CpuSnapshotDpl1 = 0x2c;
    static constexpr unsigned CpuSnapshotDph1 = 0x2d;
    static constexpr unsigned CpuSnapshotDps = 0x2e;
    static constexpr unsigned CpuSnapshotIrqLevels = 0x2f;  // bit L: level L in ISR
    static constexpr unsigned CpuSnapshotPostedVector = 0x30; // u16, 0 = none
    static constexpr unsigned CpuSnapshotPostedLevel = 0x32;
    static constexpr unsigned CpuSnapshotIrqInhibit = 0x33;  // 1: after RETI
    // 0x34-0x3f reserved, read 0.

    // The engine counts classic MCS-51 machine cycles (not CC2530 clock
    // cycles); `clock_hz` converts them to world time. The clock is not
    // derived from CLKCONCMD.
    explicit Cc2530F256Mcu(std::string name, std::uint32_t clock_hz = 32000000);

    std::string_view model() const override { return "cc2530f256"; }
    void load_firmware(const std::string &path) override;
    void reset() override;
    void set_start_offset(std::uint64_t offset_ns) override { offset_ns_ = offset_ns; }
    std::uint64_t time_ns() const override;
    McuState run_until(std::uint64_t target_ns) override;
    McuState state() const noexcept override { return state_; }
    const std::string &fault_message() const noexcept override { return fault_; }

    std::vector<std::string> pin_names() const override;
    void connect_pin(std::string_view pad, Pin &pin) override;
    void attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                      const std::vector<std::string> &protocols) override;
    unsigned uart_count() const override { return 2; }
    bool uart_can_receive(unsigned index) const override { return index < 2; }
    // Hands `byte` to the USART when its receiver can take it now (UART
    // mode, receiver enabled, idle); otherwise false and the endpoint keeps
    // the byte pending.
    bool uart_rx(unsigned index, std::uint8_t byte) override;
    std::vector<std::uint8_t> uart_take_tx(unsigned index) override;
    void read_memory(std::uint32_t address, std::uint8_t *out,
                     std::size_t length) override;
    void write_memory(std::uint32_t address, const std::uint8_t *data,
                      std::size_t length) override;

    mcs51emu::Cc2530F256 &device() noexcept { return device_; }

    // Debugger emulation: halt whenever the CPU reaches logical `pc`; on the
    // N-th arrival apply the XDATA SRAM writes listed for hit N (in order)
    // and resume. Arrivals are counted at instruction boundaries, also when
    // a run quantum ends exactly at `pc`. With a report, arrivals 1..hits
    // first append the halted state to `file` in the `mcuemu run
    // --report-hits` format (the file is truncated by set_debugger; a later
    // power-on reset restarts the count and appends). The breakpoint is
    // removed after the last arrival that has writes or is reported. Throws
    // if a write leaves XDATA SRAM, nothing is to be written or reported, or
    // the report file cannot be created.
    struct DebuggerWrite {
        unsigned hit{1};
        std::uint16_t xdata{0};
        std::vector<std::uint8_t> bytes;
    };
    struct DebuggerReport {
        std::string file;
        unsigned hits{1};
        mcs51emu::Cc2530HitDump dump;
    };
    void set_debugger(std::uint16_t pc, std::vector<DebuggerWrite> writes,
                      std::optional<DebuggerReport> report = std::nullopt);
    // Arrivals counted at the debugger breakpoint so far.
    unsigned debugger_arrivals() const noexcept { return debugger_arrivals_; }

private:
    static constexpr unsigned PadCount = 24; // index = port * 8 + pin
    std::uint8_t read_byte(std::uint32_t address) const;
    std::uint64_t cycle_ns(std::uint64_t cycle) const;
    // True when something outside the chip (a net, the air, a UART
    // endpoint) can still change its inputs.
    bool externally_driven() const;

    mcs51emu::Cc2530F256 device_;
    std::uint32_t clock_hz_;
    std::uint64_t offset_ns_{0};
    std::uint64_t cycles_{0};
    McuState state_{McuState::Running};
    std::string fault_;
    std::array<Pin *, PadCount> pins_{};
    mcuemu::VirtualAir *air_{};
    std::string air_node_;
    bool air_attached_{false};
    bool uart_used_{false};
    std::optional<std::uint16_t> debugger_pc_;
    std::vector<DebuggerWrite> debugger_writes_;
    unsigned debugger_last_hit_{0};
    std::unique_ptr<std::ofstream> debugger_report_out_;
    unsigned debugger_report_hits_{0};
    mcs51emu::Cc2530HitDump debugger_report_dump_;
    unsigned debugger_arrivals_{0};
    // The CPU stands at the breakpoint and this arrival is already counted.
    bool debugger_at_break_{false};
    void debugger_arrive();
};

} // namespace sim

#pragma once

#include "bl702emu/bl702.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace bl702emu {

inline constexpr std::uint64_t NoEvent = std::numeric_limits<std::uint64_t>::max();

// A peripheral: a register window the chip routes accesses to. Offsets are relative to `base`.
class Block {
public:
    Block(Bl702 &chip, std::uint32_t base, std::uint32_t size) : chip_(chip), base_(base), size_(size) {}
    virtual ~Block() = default;

    std::uint32_t base() const noexcept { return base_; }
    std::uint32_t size() const noexcept { return size_; }

    // Back to the power-on state (a reset of the chip).
    virtual void reset() {}
    virtual std::uint32_t read(std::uint32_t offset, unsigned size) = 0;
    virtual void write(std::uint32_t offset, std::uint32_t value, unsigned size) = 0;
    // Brings time-driven state up to `now` (core cycles) and returns the cycle of the next event.
    virtual std::uint64_t service(std::uint64_t now) {
        (void)now;
        return NoEvent;
    }

protected:
    Bl702 &chip_;

private:
    std::uint32_t base_;
    std::uint32_t size_;
};

// Word registers. Accesses of 1 or 2 bytes become a read-modify-write of the word, so a block only
// implements read_word and write_word.
class RegisterBlock : public Block {
public:
    using Block::Block;
    void reset() override { words_.clear(); }
    std::uint32_t read(std::uint32_t offset, unsigned size) final;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) final;

protected:
    virtual std::uint32_t read_word(std::uint32_t offset);
    virtual void write_word(std::uint32_t offset, std::uint32_t value);
    // Storage of the registers that have no side effect (what was written reads back).
    std::uint32_t word(std::uint32_t offset) const;
    void set_word(std::uint32_t offset, std::uint32_t value);

private:
    std::map<std::uint32_t, std::uint32_t> words_;
};

// UART0: the transmit FIFO. Bytes leave at once (the model has no baud-rate timing).
class Uart0 final : public RegisterBlock {
public:
    explicit Uart0(Bl702 &chip);

protected:
    std::uint32_t read_word(std::uint32_t offset) override;
    void write_word(std::uint32_t offset, std::uint32_t value) override;
};

// TIMER0: a free-running counter. The SDK sets the divider to 31 on the 32 MHz clock, so it counts
// 1 MHz. The compare registers and interrupts are stored but do nothing: the sensor firmware polls
// the counter.
class Timer0 final : public RegisterBlock {
public:
    explicit Timer0(Bl702 &chip);
    void reset() override;

protected:
    std::uint32_t read_word(std::uint32_t offset) override;
    void write_word(std::uint32_t offset, std::uint32_t value) override;

private:
    std::uint32_t counter() const;
    std::uint64_t base_cycle_{0}; // cycle of the last start or stop of the counter
    std::uint32_t base_count_{0}; // counter value at base_cycle_
    bool running_{false};
};

// L1 cache controller (0x40009000). The cache itself is not modelled: its operations (the flush and
// invalidate routines of the SDK poll bit 3 for completion, as bflb_cache_invalidate does) finish at once,
// so that bit always reads 1.
class L1c final : public RegisterBlock {
public:
    explicit L1c(Bl702 &chip);

protected:
    std::uint32_t read_word(std::uint32_t offset) override;
};

// The flash controller: the XIP offset (SF_CTRL_SF_ID0_OFFSET moves the XIP window over the flash, see
// Bl702::set_xip_offset_register) and the SAHB command interface of bank 1 (the driver writes a command
// into IF_SAHB_0 with its command words and data buffer, sets TRIG, and waits for BUSY to clear). The
// commands run at once on the physical flash: identification, status, write enable, reads, page program,
// and the 4 KiB, 32 KiB, 64 KiB and chip erases. Everything else reads back what was written.
class SfCtrl final : public RegisterBlock {
public:
    explicit SfCtrl(Bl702 &chip);

protected:
    std::uint32_t read_word(std::uint32_t offset) override;
    void write_word(std::uint32_t offset, std::uint32_t value) override;

private:
    void run_command();
    std::uint8_t buffer(std::uint32_t index);
    void set_buffer(std::uint32_t index, std::uint8_t value);
    bool write_enabled_{false};
};

// eFuse controller: the chip ID (the MAC) and the controller's status (IF_CTRL_0 at +0x800, where the SDK
// waits for the auto-load to finish). Read-only, and it keeps its contents across a reset.
class Efuse final : public RegisterBlock {
public:
    Efuse(Bl702 &chip, const std::array<std::uint8_t, 8> &mac);
    void reset() override {}

protected:
    std::uint32_t read_word(std::uint32_t offset) override;
    void write_word(std::uint32_t, std::uint32_t) override {}
};

// SEC_ENG AES-128 engine: ECB, the blocks of one run given by the MSG_LEN field of CTRL. The key is the
// four KEY registers, each read as a little-endian word (the order the zigbee-rs driver writes them). The
// transform is instant; the BUSY bit always reads 0.
class Aes final : public RegisterBlock {
public:
    explicit Aes(Bl702 &chip);

protected:
    std::uint32_t read_word(std::uint32_t offset) override;
    void write_word(std::uint32_t offset, std::uint32_t value) override;

private:
    void run(std::uint32_t ctrl);
};

// True random number generator of the SEC_ENG block (0x40004200). A trigger fills the eight output words
// (the driver reads 32 bytes of entropy this way: sys_rand_get, which the Zigbee stack uses for its EUI-64). The
// output is a fixed-seed xorshift, so runs repeat; an unmapped TRNG would read zeros and give an all-zero EUI.
class Trng final : public RegisterBlock {
public:
    explicit Trng(Bl702 &chip);

protected:
    void write_word(std::uint32_t offset, std::uint32_t value) override;

private:
    std::uint32_t next();
    std::uint32_t state_{0x2545f491};
};

class Ieee802154;

// The CLIC of hart 0 (the SiFive draft as the BFLB SoC lays it out: per interrupt an INTIP, INTIE and
// INTCFG byte, and CFG). Level-sensitive lines from the peripherals are pending while asserted; INTIP
// is the software pending bit, which mnxti clears when it claims an interrupt.
class Clic final : public Block {
public:
    static constexpr unsigned Interrupts = 80;
    static constexpr std::uint32_t IntipOffset = 0x000, IntieOffset = 0x400, IntcfgOffset = 0x800;

    explicit Clic(Bl702 &chip);
    std::uint32_t read(std::uint32_t offset, unsigned size) override;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;

    void set_line(unsigned irq, bool asserted);
    // The highest-level pending, enabled interrupt whose level is above `mil` and `thresh` (its number, and its
    // level in `level`), or -1. With `claim`, its software pending bit is cleared.
    int pick(std::uint32_t mil, std::uint32_t thresh, std::uint32_t &level, bool claim);

private:
    std::uint8_t regs_[0x1000]{};
    bool line_[Interrupts]{};
};

// The machine timer (CLINT): mtime counts the 1 MHz timebase, mtimecmp raises CLIC interrupt 7 when mtime
// reaches it, and msip raises interrupt 3.
class Clint final : public Block {
public:
    static constexpr unsigned TimerIrq = 7, SoftwareIrq = 3;
    static constexpr std::uint64_t TimebaseHz = 1000000;
    Clint(Bl702 &chip, Clic &clic);
    void reset() override;
    std::uint32_t read(std::uint32_t offset, unsigned size) override;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;
    std::uint64_t service(std::uint64_t now) override;

private:
    std::uint64_t mtime(std::uint64_t now) const;
    Clic &clic_;
    std::uint64_t mtimecmp_{~std::uint64_t{0}};
    std::uint64_t mtime_offset_{0};
    bool msip_{false};
};

// RF and PHY registers (0x40001000..). Only what the firmware reads back is modelled: the CCA result
// (worked out by the M154 block on the shared air) and the RSSI of the last frame. Everything else reads
// back what was written. The RF calibration flags (0x4000100c) stay zero, so the driver skips its
// calibration sequences (docs/bl702.md).
class RfPhy final : public RegisterBlock {
public:
    explicit RfPhy(Bl702 &chip);
    void connect(Ieee802154 &mac) { mac_ = &mac; }
    // A chip-side update of a result register (PHY_CCA_CTRL bits, PHY_RSSI).
    void store(std::uint32_t address, std::uint32_t value) { set_word(address - base(), value); }
    std::uint32_t load(std::uint32_t address) const { return word(address - base()); }

protected:
    void write_word(std::uint32_t offset, std::uint32_t value) override;

private:
    Ieee802154 *mac_{nullptr};
};

// The IEEE 802.15.4 MAC (M154, 0x4c000000): the radio the firmware drives by polling. Transmit, receive,
// CCA, energy detect and the interrupt status bits are modelled. Frames cross the shared VirtualAir
// without the FCS, which the hardware appends on transmit and checks on receive.
class Ieee802154 final : public RegisterBlock {
public:
    Ieee802154(Bl702 &chip, RfPhy &phy);
    void connect(mcuemu::VirtualAir &air, const std::string &node);
    void reset() override;
    std::uint64_t service(std::uint64_t now) override;
    // The interrupt line of the MAC: an enabled status bit is set.
    bool interrupt_asserted() const { return (irq_pending_ & word(IrqEnableOffset)) != 0; }

    // Clear channel assessment on the current channel, now. Returns PHY_CCA_CTRL with the result bits set:
    // bit 2 (done), bit 7 (busy) and the raw RSSI in bits 8..14 (dBm = 5 - raw).
    std::uint32_t assess(std::uint32_t cca_control);

protected:
    std::uint32_t read_word(std::uint32_t offset) override;
    void write_word(std::uint32_t offset, std::uint32_t value) override;

private:
    static constexpr std::uint32_t IrqRxDone = 0x0100'0000, IrqRxCrc = 0x0008'0000, IrqRxAck = 0x0040'0000, IrqTxFinished = 0x1000'0000;
    static constexpr std::uint32_t RxControlCommands = (1u << 8) | (1u << 12);
    static constexpr std::uint32_t LongAddrLow = 0xd0, LongAddrHigh = 0xd4;
    static constexpr std::uint32_t FcfAckRequest = 1u << 5, AddressModeExtended = 3;
    static constexpr std::uint64_t AckTurnaroundNs = 192000;
    static constexpr std::uint64_t PollPeriodCycles = 512; // 16 us between two looks at the air
    static constexpr std::uint32_t IrqEnableOffset = 0x020;
    std::uint8_t channel() const;
    std::uint64_t now_cycles() const;
    void start_tx();
    void deliver_rx(std::uint64_t now_ns);
    bool addressed_to_us(const std::vector<std::uint8_t> &frame) const;
    void acknowledge(const mcuemu::RadioFrame &frame);

    RfPhy &phy_;
    mcuemu::VirtualAir *air_{nullptr};
    std::string node_;
    bool tx_busy_{false};
    bool rx_on_{false};
    bool tx_ack_requested_{false};
    std::uint64_t tx_end_cycle_{0};
    std::uint32_t irq_pending_{0};
    std::uint8_t tx_buffer_[128]{};
    std::uint8_t rx_buffer_[128]{};
    std::uint32_t rx_phy_length_{0};
    std::uint64_t time_offset_{0}; // the timer counter (1 MHz): cycles / 32 plus this
    std::uint64_t timers(std::uint64_t now);
    std::uint64_t timer_ticks() const;
};

} // namespace bl702emu

#pragma once

#include "efr32emu/efr32mg21.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace efr32emu {

constexpr std::uint64_t Never = std::numeric_limits<std::uint64_t>::max();

// One peripheral block. `reg` is the offset inside the block (0..0xFFF); the
// Series 2 set/clear/toggle aliases are resolved by the chip before the call.
class Block {
public:
    explicit Block(Efr32mg21 &chip) : chip_(chip) {}
    virtual ~Block() = default;

    virtual std::uint32_t read(std::uint32_t reg) = 0;
    virtual void write(std::uint32_t reg, std::uint32_t value) = 0;
    // The value an alias operation starts from; reading must not have side effects.
    virtual std::uint32_t peek(std::uint32_t reg) { return read(reg); }
    // Advances time-driven state to `now` (core cycles); returns the next cycle
    // at which the block needs attention.
    virtual std::uint64_t service(std::uint64_t now) {
        (void)now;
        return Never;
    }
    virtual void reset() {}

protected:
    std::uint64_t now() const;
    std::uint64_t clock_hz() const;
    void set_irq_level(unsigned irq, bool level);
    Efr32mg21 &chip() noexcept { return chip_; }

    Efr32mg21 &chip_;
};

// Registers that read back what was written, for blocks the model does not
// implement. Firmware that waits for hardware behind such a block stops at the
// wait, which `--trace-mmio` makes visible.
class RegBlock : public Block {
public:
    using Block::Block;
    std::uint32_t read(std::uint32_t reg) override { return regs_[(reg >> 2) & 0x3ff]; }
    void write(std::uint32_t reg, std::uint32_t value) override { regs_[(reg >> 2) & 0x3ff] = value; }
    void reset() override { regs_.fill(0); }

protected:
    std::array<std::uint32_t, 1024> regs_{};
};

// CMU (0x40008000). Only the system clock selection is observable.
class Cmu final : public RegBlock {
public:
    static constexpr std::uint32_t Sysclkctrl = 0x070;
    using RegBlock::RegBlock;
    void reset() override;
    // SYSCLKCTRL.CLKSEL: 1 FSRCO, 2 HFRCODPLL, 3 HFXO, 4 CLKIN0
    std::uint32_t clock_select() const { return regs_[Sysclkctrl >> 2] & 7u; }
};

// HFXO0 (0x4000C000). STATUS follows CTRL and the system clock request.
class Hfxo final : public RegBlock {
public:
    Hfxo(Efr32mg21 &chip, Cmu &cmu) : RegBlock(chip), cmu_(cmu) {}
    std::uint32_t read(std::uint32_t reg) override;

private:
    Cmu &cmu_;
};

// DPLL0 (0x4001C000): lock follows the enable bit at once. No frequency model;
// CMU_DPLLLock only waits for IF.LOCK after EN_SET.
class Dpll final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
};

// RTCC (0x48000000 secure, 0x58000000 non-secure alias). The Zephyr kernel tick
// runs on the sleeptimer, which runs on this counter, so it has to count.
// Counter: 32-bit, clocked at 32768 Hz (inferred: the LFXO/LFRCO clock is not
// modelled) divided by 2^CFG.CNTPRESC, runs while EN and CMD.START are set.
// Compare channel 1 in output-compare mode sets IF.CC1 when the counter reaches
// OCVALUE + 1 (inferred from sl_sleeptimer_hal_rtcc.c, which programs
// OCVALUE = target - 1 and says the match fires one tick later).
class Rtcc final : public RegBlock {
public:
    static constexpr unsigned Irq = 10; // RTCC_IRQn
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    bool counting() const;
    std::uint64_t ticks(std::uint64_t at) const;
    std::uint64_t cycle_for(std::uint64_t tick) const;
    void freeze();
    void arm();
    void update_irq();

    // The counter is ticks_at_ at core cycle cycle_at_ while it runs; armed_tick_
    // is the next compare match (Never when the channel is not armed).
    std::uint64_t ticks_at_{0}, cycle_at_{0}, armed_tick_{Never};
    bool started_{false};
};

// USART0 (0x40058000), the console. Transmission is instantaneous: TXC, TXBL and
// TXIDLE read as set, and each byte written to TXDATA is appended to the chip's
// UART0 output. Everything else reads back what was written.
class Usart final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
};

// SEMAILBOX host (0x4C000000 secure, 0x5C000000 non-secure): the mailbox to the
// Secure Element that se_manager uses for its commands. The SE itself is not
// modelled; the mailbox answers every command with SUCCESS and no output data.
// Inferred from sli_se_manager_mailbox.c: the host writes TX_HEADER (bytes) and
// then the command words to FIFO; the response is ready (RX_STATUS.RXINT) once
// the words the header announces have arrived, and RX_HEADER holds the response.
class SeMailbox final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    void reset() override;

private:
    unsigned words_{0};
};

// Radio blocks RAIL drives directly, outside the zigbee-rs register map: the
// BUFC (0x44004000 secure, 0x54004000 non-secure), the RAC (0xA8020000 / 0xB8020000)
// and the sequencer and FRC RAM (0xA0000000 / 0xB0000000). RAIL polls several
// words here. Each word is plain storage unless a hook below says otherwise.
// Every hook is an inference from the code that polls the word (see
// docs/efr32mg21.md, "RAIL registers"), not a documented behaviour.
class RailRegs final {
public:
    std::uint32_t read(std::uint32_t address);
    void write(std::uint32_t address, std::uint32_t value, unsigned size = 4);
    void reset() { words_.clear(); }
    // Direct access for the protimer model: plain stored value, no aliasing.
    std::uint32_t peek(std::uint32_t address) const;
    void set_bits(std::uint32_t address, std::uint32_t mask);
    void clear_bits(std::uint32_t address, std::uint32_t mask);
    // True once after a write starts a transmit (RAC CMD.TXEN); the chip then
    // schedules the end of the frame.
    bool take_tx_start();
    // RX is requested while RAC RXENSRCEN bit 1 is set (RAIL sets it through the
    // SET alias before it starts a receive, and clears all of bits 0..7 to stop).
    bool rx_enabled() const;
    // RXFRAME (RAC STATUS.STATE 3) from a frame's delivery until the firmware
    // acknowledges it (BUFC IF bit 18 cleared). RAIL's helper reads this state to
    // decide the frame is being received (sets its gate flag B).
    void set_rx_frame(bool on) { rx_frame_ = on; }
    bool rx_frame() const { return rx_frame_; }

private:
    std::unordered_map<std::uint32_t, std::uint32_t> words_;
    bool tx_started_{false};
    bool rx_frame_{false};
};

// MSC (0x40030000): page erase and word programming by the register protocol
// the zigbee-rs flash driver uses.
class Msc final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    void reset() override;

private:
    bool unlocked_{false};
    bool invalid_address_{false};
    std::uint32_t address_{0};
};

// GPIO (0x4003C000): four ports, EXTI channels 0..7 (GPIO_EVEN/GPIO_ODD).
class Gpio final : public Block {
public:
    explicit Gpio(Efr32mg21 &chip) : Block(chip) {}
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    void reset() override;

    PadDrive drive(unsigned pad) const;
    bool level(unsigned pad) const;
    void set_external(unsigned pad, bool high);

private:
    static constexpr unsigned PortCount = 4;
    unsigned mode(unsigned pad) const { return (modes_[pad / 16][(pad % 16) / 8] >> (((pad % 16) % 8) * 4)) & 0xfu; }
    bool dout(unsigned pad) const { return (dout_[pad / 16] >> (pad % 16)) & 1u; }
    void changed(unsigned pad);
    void refresh_exti(unsigned pad, bool before, bool after);
    void update_irq();

    std::array<std::array<std::uint32_t, 2>, PortCount> modes_{};
    std::array<std::uint32_t, PortCount> dout_{};
    std::array<std::uint32_t, PortCount> ctrl_{};
    std::uint32_t extipsel_{0}, extipinsel_{0}, extirise_{0}, extifall_{0}, if_{0}, ien_{0};
    std::array<bool, 64> external_valid_{};
    std::array<bool, 64> external_{};
    std::array<bool, 64> last_{};
};

// The radio blocks (FRC, BUFC, SYNTH, RAC, MODEM, AGC) at 0x40090000. See
// radio.cpp: the register interface is the one zigbee-rs's driver uses, not
// the silicon's.
class Radio final : public Block {
public:
    explicit Radio(Efr32mg21 &chip) : Block(chip) {}
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    std::uint32_t peek(std::uint32_t reg) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;
    void connect(mcuemu::VirtualAir &air, const std::string &node);
    // Sends a TX FIFO image laid out as the RAIL path writes it (PHR, then the
    // PSDU) on `channel`. Returns the frame's time on air in ns.
    std::uint64_t transmit_fifo(const std::vector<std::uint8_t> &fifo, std::uint8_t channel);
    // Frames that ended on `channel` since the previous call, each as the RAIL RX
    // buffer holds it: PHR (payload plus the two FCS bytes), then the payload.
    std::vector<std::vector<std::uint8_t>> receive_fifo(std::uint8_t channel, std::uint64_t now_ns);

private:
    enum class State : std::uint32_t { Off = 0, Idle = 1, Rx = 2, Tx = 3 };

    void command(std::uint32_t value);
    void begin_transmit();
    void poll_air(std::uint64_t ns);
    std::uint8_t channel() const { return static_cast<std::uint8_t>(11 + (std::min<std::uint32_t>(chno_, 15))); }
    std::int8_t energy_dbm(std::uint64_t at_ns) const;
    void update_irq();
    std::uint64_t reschedule() const;
    std::uint64_t to_ns(std::uint64_t cycles) const;

    mcuemu::VirtualAir *air_{nullptr};
    std::string node_;
    std::array<std::uint32_t, 0x6000 / 4> regs_{};
    State state_{State::Idle};
    std::uint32_t rac_if_{0}, rac_ien_{0}, frc_if_{0}, chno_{0};
    std::vector<std::uint8_t> tx_fifo_;
    std::vector<std::uint8_t> rx_fifo_;
    std::size_t rx_read_{0};
    bool rx_pending_{false};
    std::int8_t rx_rssi_{-127};
    std::uint64_t tx_end_ns_{0};
    bool tx_active_{false};
    std::uint64_t rx_start_ns_{0}, last_poll_ns_{0}, rail_poll_ns_{0};
};

} // namespace efr32emu

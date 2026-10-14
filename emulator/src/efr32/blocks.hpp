#pragma once

#include "efr32emu/efr32mg21.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
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
    std::uint64_t rx_start_ns_{0}, last_poll_ns_{0};
};

} // namespace efr32emu

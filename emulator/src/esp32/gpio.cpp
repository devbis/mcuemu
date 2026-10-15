#include "gpio.hpp"

#include "esp32emu/esp32h2_regs.hpp"

namespace esp32emu {

namespace {
// The layouts of the two chips agree on these (checked against both SVDs); the C6 has more pads.
namespace g = esp32h2_regs::gpio::reg;
constexpr std::uint32_t Pin0 = esp32h2_regs::gpio::reg::PIN0, FuncOutSel0 = esp32h2_regs::gpio::reg::FUNC0_OUT_SEL_CFG;
constexpr unsigned IntTypeShift = esp32h2_regs::gpio::field::PIN0_INT_TYPE_shift,
                   IntEnaShift = esp32h2_regs::gpio::field::PIN0_INT_ENA_shift;
constexpr std::uint32_t SimpleGpioOut = 0x80;
constexpr unsigned MuxWpd = esp32h2_regs::io_mux::field::GPIO0_FUN_WPD_shift,
                   MuxWpu = esp32h2_regs::io_mux::field::GPIO0_FUN_WPU_shift;
} // namespace

bool IoMux::pull_up(unsigned pad) const {
    return pad < pins_ && (const_cast<IoMux *>(this)->StatusBlock::read(4 + 4 * pad, 4) >> MuxWpu & 1u);
}
bool IoMux::pull_down(unsigned pad) const {
    return pad < pins_ && (const_cast<IoMux *>(this)->StatusBlock::read(4 + 4 * pad, 4) >> MuxWpd & 1u);
}

void IoMux::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    StatusBlock::write(offset, value, size);
    if (changed)
        changed();
}

Gpio::Gpio(std::uint32_t base, unsigned pins, unsigned source, IoMux &mux, IrqSink irq)
    : StatusBlock(base, 0x700), pins_(pins), source_(source), mux_(mux), irq_(std::move(irq)) {
    mux_.changed = [this] {
        if (on_drive_changed)
            for (unsigned pad = 0; pad < pins_; ++pad)
                on_drive_changed(pad);
    };
}

void Gpio::reset() {
    StatusBlock::reset();
    out_ = enable_ = status_ = 0;
    update_irq();
}

std::uint32_t Gpio::pin_config(unsigned pad) const {
    return const_cast<Gpio *>(this)->StatusBlock::read(Pin0 + 4 * pad, 4);
}

PadDrive Gpio::drive(unsigned pad) const {
    const std::uint32_t sel = const_cast<Gpio *>(this)->StatusBlock::read(FuncOutSel0 + 4 * pad, 4) & 0xff;
    if (sel == SimpleGpioOut && (enable_ >> pad & 1u))
        return (out_ >> pad & 1u) ? PadDrive::High : PadDrive::Low;
    if (mux_.pull_up(pad))
        return PadDrive::PullUp;
    if (mux_.pull_down(pad))
        return PadDrive::PullDown;
    return PadDrive::HiZ;
}

void Gpio::set_input(unsigned pad, bool high) {
    if (pad >= pins_)
        return;
    const bool was = in_ >> pad & 1u;
    in_ = (in_ & ~(1u << pad)) | (std::uint32_t{high} << pad);
    const unsigned type = pin_config(pad) >> IntTypeShift & 7u;
    if ((type == 1 && !was && high) || (type == 2 && was && !high) || (type == 3 && was != high))
        status_ |= 1u << pad;
    evaluate_levels();
    update_irq();
}

// Level interrupt types hold their status while the condition lasts.
void Gpio::evaluate_levels() {
    for (unsigned pad = 0; pad < pins_; ++pad) {
        const unsigned type = pin_config(pad) >> IntTypeShift & 7u;
        const bool level = in_ >> pad & 1u;
        if ((type == 4 && !level) || (type == 5 && level))
            status_ |= 1u << pad;
    }
}

void Gpio::update_irq() {
    std::uint32_t pending = 0;
    for (unsigned pad = 0; pad < pins_; ++pad)
        if ((status_ >> pad & 1u) && (pin_config(pad) >> IntEnaShift & 1u))
            pending |= 1u << pad;
    irq_(source_, pending != 0);
}

std::uint32_t Gpio::read(std::uint32_t offset, unsigned size) {
    switch (offset & ~3u) {
    case g::OUT: return out_;
    case g::ENABLE: return enable_;
    case g::IN: return in_;
    case g::STATUS: return status_;
    case g::PCPU_INT: {
        std::uint32_t pending = 0;
        for (unsigned pad = 0; pad < pins_; ++pad)
            if ((status_ >> pad & 1u) && (pin_config(pad) >> IntEnaShift & 1u))
                pending |= 1u << pad;
        return pending;
    }
    default: return StatusBlock::read(offset, size);
    }
}

void Gpio::write(std::uint32_t offset, std::uint32_t value, unsigned size) {
    const std::uint32_t before_out = out_, before_enable = enable_;
    switch (offset & ~3u) {
    case g::OUT: out_ = value; break;
    case g::OUT_W1TS: out_ |= value; break;
    case g::OUT_W1TC: out_ &= ~value; break;
    case g::ENABLE: enable_ = value; break;
    case g::ENABLE_W1TS: enable_ |= value; break;
    case g::ENABLE_W1TC: enable_ &= ~value; break;
    case g::STATUS: status_ = value; break;
    case g::STATUS_W1TS: status_ |= value; break;
    case g::STATUS_W1TC:
        status_ &= ~value;
        evaluate_levels();
        break;
    default: StatusBlock::write(offset, value, size); break;
    }
    if (offset >= Pin0 && offset < Pin0 + 4 * MaxPins) {
        evaluate_levels();
    }
    update_irq();
    if (on_drive_changed) {
        std::uint32_t changed = (before_out ^ out_) | (before_enable ^ enable_);
        if ((offset & ~3u) >= FuncOutSel0)
            changed = (1u << pins_) - 1;
        for (unsigned pad = 0; pad < pins_; ++pad)
            if (changed >> pad & 1u)
                on_drive_changed(pad);
    }
}

} // namespace esp32emu

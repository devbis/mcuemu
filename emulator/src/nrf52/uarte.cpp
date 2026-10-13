// nRF52840 UARTE0/1.
//
// Evidence: nRF52840 Product Specification, chapter "UARTE". BAUDRATE holds
// baud * 2^32 / 16 MHz with the low 12 bits unused. Inferred, not documented:
// STOPRX raises RXTO and ENDRX immediately; hardware flow control is ignored;
// an overrun (byte with a full four-byte FIFO) raises ERROR and sets ERRORSRC.OVERRUN.

#include "nrf52/gpio.hpp"
#include "nrf52/peripherals.hpp"

#include <cstring>

namespace nrf52emu {

std::uint32_t Uarte::event_register(unsigned bit) {
    switch (bit) {
    case Cts: return 0x100;
    case Ncts: return 0x104;
    case RxDrdy: return 0x108;
    case EndRx: return 0x110;
    case TxDrdy: return 0x120;
    case Error: return 0x124;
    case EndTx: return 0x128;
    case RxTo: return 0x144;
    case RxStarted: return 0x14c;
    case TxStarted: return 0x150;
    case TxStopped: return 0x158;
    default: return 0;
    }
}

void Uarte::reset() {
    events_ = inten_ = shorts_ = enable_ = 0;
    baud_ = 0x04000000;
    for (auto &v : psel_) v = ~0u;
    rx_ptr_ = rx_max_ = rx_amount_ = tx_ptr_ = tx_max_ = tx_amount_ = 0;
    config_ = errorsrc_ = 0;
    rx_active_ = tx_active_ = false;
    tx_next_ = Never;
    rx_fifo_count_ = 0;
    tx_bytes_.clear();
    update_irq();
}

void Uarte::update_irq() { set_irq_level((events_ & inten_) != 0); }

void Uarte::raise(unsigned bit) {
    events_ |= 1u << bit;
    event_raised(event_register(bit));
    update_irq();
}

std::uint64_t Uarte::byte_cycles() const {
    const std::uint64_t baud = (static_cast<std::uint64_t>(baud_) * 16000000ull) >> 32;
    return 10ull * clock_hz() / (baud ? baud : 1);
}

void Uarte::apply_pads() {
    Gpio &gpio = chip_.gpio();
    const std::uint32_t txd = psel_[1];
    if (txd != ~0u && !(txd & 0x80000000u)) {
        const unsigned pad = ((txd >> 5) & 1u) * 32 + (txd & 31u);
        if (pad < Nrf52840::PadCount)
            gpio.set_override(pad, enable_ == 8 ? std::optional<Gpio::Override>(Gpio::Override{true, false})
                                                  : std::nullopt);
    }
}

void Uarte::deliver_rx() {
    while (rx_active_ && rx_fifo_count_ != 0) {
        const std::uint8_t byte = rx_fifo_[0];
        for (unsigned i = 1; i != rx_fifo_count_; ++i)
            rx_fifo_[i - 1] = rx_fifo_[i];
        --rx_fifo_count_;
        const std::uint32_t address = rx_ptr_ + rx_amount_;
        if (address >= Nrf52840::RamBase && address < Nrf52840::RamBase + Nrf52840::RamSize)
            chip_.ram()[address - Nrf52840::RamBase] = byte;
        ++rx_amount_;
        raise(RxDrdy);
        if (rx_amount_ >= rx_max_) {
            rx_active_ = false;
            raise(EndRx);
            if (shorts_ & (1u << 5)) { // ENDRX_STARTRX
                rx_active_ = true;
                rx_amount_ = 0;
                raise(RxStarted);
            }
        }
    }
}

bool Uarte::receive(std::uint8_t byte) {
    if (enable_ != 8)
        return false;
    if (rx_fifo_count_ >= RxFifo) {
        errorsrc_ |= 1u; // OVERRUN
        raise(Error);
        return false;
    }
    rx_fifo_[rx_fifo_count_++] = byte;
    deliver_rx();
    return true;
}

void Uarte::task(std::uint32_t offset) {
    switch (offset) {
    case 0x000: // STARTRX
        if (enable_ != 8)
            break;
        rx_active_ = true;
        rx_amount_ = 0;
        raise(RxStarted);
        deliver_rx();
        break;
    case 0x004: // STOPRX
        if (rx_active_) {
            rx_active_ = false;
            raise(RxTo);
            raise(EndRx);
        }
        break;
    case 0x008: // STARTTX
        if (enable_ != 8)
            break;
        tx_active_ = true;
        tx_amount_ = 0;
        raise(TxStarted);
        tx_next_ = now() + byte_cycles();
        request_service(tx_next_);
        break;
    case 0x00c: // STOPTX
        if (tx_active_) {
            tx_active_ = false;
            tx_next_ = Never;
        }
        raise(TxStopped);
        break;
    case 0x02c: // FLUSHRX
        deliver_rx();
        raise(EndRx);
        break;
    default: break;
    }
}

std::uint64_t Uarte::service(std::uint64_t t) {
    while (tx_active_ && tx_next_ <= t) {
        if (tx_amount_ >= tx_max_) {
            tx_active_ = false;
            tx_next_ = Never;
            raise(EndTx);
            break;
        }
        const std::uint32_t address = tx_ptr_ + tx_amount_;
        std::uint8_t byte = 0;
        if (address >= Nrf52840::RamBase && address < Nrf52840::RamBase + Nrf52840::RamSize)
            byte = chip_.ram()[address - Nrf52840::RamBase];
        else if (address < Nrf52840::FlashSize)
            byte = chip_.flash()[address]; // EasyDMA can read flash too
        tx_bytes_.push_back(byte);
        ++tx_amount_;
        raise(TxDrdy);
        if (tx_amount_ >= tx_max_) {
            tx_active_ = false;
            tx_next_ = Never;
            raise(EndTx);
        } else {
            tx_next_ += byte_cycles();
        }
    }
    return tx_active_ ? tx_next_ : Never;
}

std::uint32_t Uarte::read(std::uint32_t offset) {
    for (unsigned bit : {0u, 1u, 2u, 4u, 7u, 8u, 9u, 17u, 19u, 20u, 22u})
        if (event_register(bit) == offset)
            return (events_ >> bit) & 1u;
    switch (offset) {
    case 0x200: return shorts_;
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x480: return errorsrc_;
    case 0x500: return enable_;
    case 0x508: return psel_[0];
    case 0x50c: return psel_[1];
    case 0x510: return psel_[2];
    case 0x514: return psel_[3];
    case 0x524: return baud_;
    case 0x534: return rx_ptr_;
    case 0x538: return rx_max_;
    case 0x53c: return rx_amount_;
    case 0x544: return tx_ptr_;
    case 0x548: return tx_max_;
    case 0x54c: return tx_amount_;
    case 0x56c: return config_;
    default: return 0;
    }
}

void Uarte::write(std::uint32_t offset, std::uint32_t value) {
    if (offset < 0x100) {
        if (value & 1u)
            task(offset);
        return;
    }
    for (unsigned bit : {0u, 1u, 2u, 4u, 7u, 8u, 9u, 17u, 19u, 20u, 22u}) {
        if (event_register(bit) == offset) {
            if (value & 1u)
                events_ |= 1u << bit;
            else
                events_ &= ~(1u << bit);
            update_irq();
            return;
        }
    }
    switch (offset) {
    case 0x200: shorts_ = value; break;
    case 0x304: inten_ |= value; update_irq(); break;
    case 0x308: inten_ &= ~value; update_irq(); break;
    case 0x480: errorsrc_ &= ~value; break;
    case 0x500:
        enable_ = value & 0xfu;
        apply_pads();
        break;
    case 0x508: psel_[0] = value; break;
    case 0x50c:
        psel_[1] = value;
        apply_pads();
        break;
    case 0x510: psel_[2] = value; break;
    case 0x514: psel_[3] = value; break;
    case 0x524: baud_ = value; break;
    case 0x534: rx_ptr_ = value; break;
    case 0x538: rx_max_ = value & 0xffffu; break;
    case 0x544: tx_ptr_ = value; break;
    case 0x548: tx_max_ = value & 0xffffu; break;
    case 0x56c: config_ = value; break;
    default: break;
    }
}

} // namespace nrf52emu

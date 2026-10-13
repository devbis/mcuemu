// nRF52840 ECB (AES-128 block encryption, EasyDMA).
//
// Evidence: nRF52840 Product Specification, chapter "ECB": ECBDATAPTR points at
// KEY (16 bytes), CLEARTEXT (16) and CIPHERTEXT (16). The byte order is the
// standard one (first byte most significant), matching Zephyr's use of the
// hardware for big-endian data.

#include "crypto/aes128.hpp"
#include "nrf52/peripherals.hpp"

#include <cstring>

namespace nrf52emu {

void Ecb::reset() {
    end_ = error_ = false;
    inten_ = ptr_ = 0;
    done_at_ = Never;
    update_irq();
}

void Ecb::update_irq() { set_irq_level((end_ && (inten_ & 1u)) || (error_ && (inten_ & 2u))); }

void Ecb::task(std::uint32_t offset) {
    if (offset == 0x000) {
        done_at_ = now() + 7u * (clock_hz() / 1000000u);
        request_service(done_at_);
    } else if (offset == 0x004) {
        done_at_ = Never;
        error_ = true;
        event_raised(0x104);
        update_irq();
    }
}

std::uint64_t Ecb::service(std::uint64_t t) {
    if (done_at_ <= t) {
        done_at_ = Never;
        if (ptr_ >= Nrf52840::RamBase && ptr_ + 48 <= Nrf52840::RamBase + Nrf52840::RamSize) {
            std::uint8_t *data = chip_.ram() + (ptr_ - Nrf52840::RamBase);
            crypto::Block key, plain;
            std::memcpy(key.data(), data, 16);
            std::memcpy(plain.data(), data + 16, 16);
            const crypto::Block cipher = crypto::aes128_encrypt(key, plain);
            std::memcpy(data + 32, cipher.data(), 16);
            end_ = true;
            event_raised(0x100);
        } else {
            error_ = true;
            event_raised(0x104);
        }
        update_irq();
    }
    return done_at_;
}

std::uint32_t Ecb::read(std::uint32_t offset) {
    switch (offset) {
    case 0x100: return end_ ? 1 : 0;
    case 0x104: return error_ ? 1 : 0;
    case 0x300:
    case 0x304:
    case 0x308: return inten_;
    case 0x504: return ptr_;
    default: return 0;
    }
}

void Ecb::write(std::uint32_t offset, std::uint32_t value) {
    switch (offset) {
    case 0x000:
    case 0x004:
        if (value & 1u)
            task(offset);
        break;
    case 0x100: end_ = (value & 1u) != 0; update_irq(); break;
    case 0x104: error_ = (value & 1u) != 0; update_irq(); break;
    case 0x304: inten_ |= value & 3u; update_irq(); break;
    case 0x308: inten_ &= ~value; update_irq(); break;
    case 0x504: ptr_ = value; break;
    default: break;
    }
}

} // namespace nrf52emu

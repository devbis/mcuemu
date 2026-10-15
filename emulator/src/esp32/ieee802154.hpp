#pragma once

#include "blocks.hpp"
#include "mcuemu/radio.hpp"
#include "timers.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace esp32emu {

// The IEEE 802.15.4 MAC (base 0x600A3000 on the C6). Written from the open
// drivers (esp-hal's esp-radio ieee802154/{raw,hal,pib}.rs and ESP-IDF's
// esp_ieee802154_dev.c), not from the TRM: what is modelled is exactly what
// they touch, and the closed PHY (libphy) sits below it and is not involved.
//
//  - COMMAND opcodes: TX 0x41, RX 0x42, CCA+TX 0x43, ED 0x44, STOP 0x45,
//    timers 0x4c..0x4f.
//  - Frames move by DMA: TXDMA_ADDR points at {PHR, MAC bytes}, RXDMA_ADDR at the
//    receive buffer. The PHR counts the 2-byte FCS (which the hardware appends and
//    checks, so it is not in the buffer); a received frame is stored as
//    {PHR, MAC bytes, RSSI, LQI} (ESP-IDF reads rssi at [PHR - 1], lqi at [PHR]).
//  - EVENT_STATUS is write-1-to-clear; the interrupt line is EVENT_STATUS &
//    EVENT_EN. Aborts raise TxAbort/RxAbort only for the reasons enabled in the
//    two abort-control registers and leave the reason code in TX_STATUS/RX_STATUS.
//  - Hardware ACKs: a unicast frame with the ack-request bit (frame version 0/1)
//    is answered 192 us after its end when HW_AUTO_ACK_TX_EN is set, with the
//    pending bit from ACK_FRAME_PENDING_EN. A transmitted frame that asks for an
//    ACK waits in RX-ACK for one when HW_AUTO_ACK_RX_EN is set; the driver's timer0
//    (microseconds from the 0x4c command) is what notices a missing ACK.
//  - The address filter follows CTRL_CFG: promiscuous mode takes everything,
//    otherwise the destination PAN/address must match an enabled INFx.
// Not modelled: enhanced ACKs (frame version 2), security, coexistence, ED
// scan results from the air, MAC_INF multi-PAN beyond the filter.
class Ieee802154 final : public Block {
public:
    using MemRead = std::function<bool(std::uint32_t address, void *out, std::size_t length)>;
    using MemWrite = std::function<bool(std::uint32_t address, const void *data, std::size_t length)>;
    using Request = std::function<void(std::uint64_t cycle)>;

    // `source` is the MAC's interrupt matrix source (ETS_ZB_MAC_SOURCE in soc/interrupts.h: 12 on the
    // C6, 21 on the H2).
    Ieee802154(std::uint32_t base, std::uint32_t cpu_hz, unsigned source, CycleSource cycles, IrqSink irq, MemRead read, MemWrite write,
               Request request_service);

    // Joins the shared medium as `node`.
    void connect(mcuemu::VirtualAir &air, const std::string &node);

    std::uint32_t read(std::uint32_t offset, unsigned size) override;
    void write(std::uint32_t offset, std::uint32_t value, unsigned size) override;
    std::uint64_t service(std::uint64_t now) override;

private:
    enum class State { Idle, Rx, Tx, RxAck, TxAck, Ed };

    std::uint32_t reg(std::uint32_t offset) const { return regs_[offset / 4]; }
    std::uint64_t to_ns(std::uint64_t cycles) const;
    std::uint64_t to_cycles(std::uint64_t ns) const;
    std::uint8_t channel() const;
    void command(std::uint8_t opcode);
    void start_tx(bool cca);
    void start_rx();
    void stop();
    void raise(std::uint32_t event);
    void tx_abort(unsigned code);
    void rx_abort(unsigned code);
    void update_irq();
    void poll_air(std::uint64_t ns);
    bool accept(const std::vector<std::uint8_t> &frame, bool &ack_needed) const;
    void store(const mcuemu::RadioFrame &frame);
    void deliver(const mcuemu::RadioFrame &frame, bool ack_wanted);
    void send_ack(std::uint64_t at_ns);
    void timers(std::uint64_t ns);
    std::uint64_t next_event_cycle() const;

    std::uint32_t cpu_hz_;
    unsigned source_;
    CycleSource cycles_;
    IrqSink irq_;
    MemRead mem_read_;
    MemWrite mem_write_;
    Request request_;

    mcuemu::VirtualAir *air_{nullptr};
    std::string node_;
    std::array<std::uint32_t, 0x200 / 4> regs_{};
    std::uint32_t events_{0};
    std::uint32_t tx_abort_code_{0}, rx_abort_code_{0};
    State state_{State::Idle};

    // Transmit / receive-ack
    std::vector<std::uint8_t> tx_frame_; // MAC bytes without FCS
    std::uint64_t tx_start_ns_{0}, tx_end_ns_{0};
    bool tx_sfd_done_{false};
    std::uint64_t ack_wait_since_ns_{0};
    // Receive
    std::uint64_t rx_start_ns_{0}, last_poll_ns_{0};
    // Ack the hardware owes for the frame it just received
    bool ack_due_{false};
    std::uint64_t ack_at_ns_{0};
    std::uint8_t ack_seq_{0};
    std::uint64_t ack_end_ns_{0};
    bool ack_on_air_{false};
    // Energy detection
    std::uint64_t ed_end_ns_{0};
    // Timers: start time in ns, 0 stopped
    bool timer_running_[2]{false, false};
    std::uint64_t timer_start_ns_[2]{0, 0};
};

} // namespace esp32emu

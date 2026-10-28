#pragma once

// CC2530 USART0/USART1 (SWRU191F Chapter 17): UART and SPI master/slave
// modes, baud-rate generator, URXn/UTXn interrupts and DMA triggers.
// UART data is exchanged with the host as bytes or as frames with a 9th
// bit, stop-bit errors or a break (frames are not rendered on pads). SPI
// master and slave shift bits on the pads (SCK edges at exact cycles); a
// master transfer calls the optional host device instead when one is set.

#include "cc2530_soc.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <vector>

namespace mcs51emu::cc2530 {

class Usart final : public Peripheral {
public:
    // `index` 0 or 1.
    Usart(Bus &bus, unsigned index);
    const char *name() const override { return index_ == 0 ? "USART0" : "USART1"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;

    // ---- host ----
    // UART receive path: true while the receiver is enabled (UxCSR.RE) and
    // its host queue can take another byte.
    bool can_receive() const;
    // Queues a byte; it arrives (URXn, UxDBUF) one frame time after the
    // previous one at the configured baud rate.
    void receive(std::uint8_t byte);
    // Queues a frame (9th bit, stop-bit errors, break); a plain frame is a
    // byte.
    void receive(const Cc2530UartFrame &frame);
    std::vector<std::uint8_t> take_transmitted();
    // Records transmitted UART frames with their 9th bit while on.
    void set_frame_capture(bool on);
    std::vector<Cc2530UartFrame> take_transmitted_frames();
    // SPI master: when set, called once per transferred byte with the MOSI
    // byte; returns the MISO byte (byte level, the pads do not toggle).
    // Without a device the transfer runs on the pads (SCK, MOSI, MISO).
    void set_spi_device(std::function<std::uint8_t(std::uint8_t)> device);

private:
    friend class Io;
    friend struct IoAnalogUsartTestAccess; // white-box test access
    friend struct SpiTestAccess;           // white-box test access
    friend struct UartErrorsTestAccess;    // white-box test access

    // Exact bit timing: bit k after the anchor ends at anchor +
    // ceil((frac + k * num) / den) cycles of the HF domain.
    struct BitClock {
        std::uint64_t anchor{}, frac{}, num{1}, den{1};
        std::uint64_t time_after(unsigned bits) const;
        // End of `halves` half bits: anchor + ceil((frac + halves * num / 2) / den).
        std::uint64_t time_after_halves(unsigned halves) const;
        void advance(unsigned bits);
        void restart(std::uint64_t at, std::uint64_t n, std::uint64_t d);
    };

    // Pad use for Io (role 0 RX/MI, 1 TX/MO, 2 CT/SS, 3 RT/C): nullopt when
    // the function does not use the pad, otherwise the output level, or
    // nullopt inside for an input.
    std::optional<std::optional<bool>> pad_use(unsigned role) const;
    // A port 0/1 pad level changed, called by Io: UART CTS (flow control)
    // and SPI slave SCK/SSN edges.
    void pad_levels_changed();

    bool uart() const;
    bool spi_master() const;
    bool spi_slave() const;
    void bit_rate(std::uint64_t &num, std::uint64_t &den) const;
    unsigned frame_bits() const;
    // `chained`: the frame directly follows the previous one (exact timing).
    void try_start_tx(std::uint64_t at, bool chained);
    void tx_complete();
    // SPI master on the pads: HF time of the next SCK edge and its handling.
    std::uint64_t spi_edge_hf() const;
    void spi_master_edge();
    // SPI slave: SCK/SSN pad changes, shift register load.
    void spi_slave_pads();
    void spi_slave_load();
    void spi_slave_reset(bool snapshot);
    // Reports pad changes made at HF time `hf_at` to Io (and the host).
    void publish_pads(std::uint64_t hf_at);
    void try_start_rx(std::uint64_t at);
    void rx_complete();
    void rx_second_stop();
    void deliver(std::uint8_t byte);
    void abort_all(bool drop_queue);
    void set_csr_bits(std::uint8_t mask, bool set);
    void outputs_changed();
    void trigger_dma(unsigned trigger);

    unsigned index_;
    std::function<std::uint8_t(std::uint8_t)> spi_device_;
    std::vector<std::uint8_t> transmitted_;
    bool capture_frames_{};
    std::vector<Cc2530UartFrame> transmitted_frames_;
    // Transmit side: the data buffer (`tx_hold_`, double buffering) and the
    // shift register.
    std::optional<std::uint8_t> tx_hold_;
    bool tx_busy_{};
    std::uint8_t tx_byte_{};
    Cc2530UartFrame::Ninth tx_ninth_{}; // the 9th bit of the UART frame
    unsigned tx_bits_{};
    std::uint64_t tx_end_hf_{Never};
    BitClock tx_clock_;
    // Receive side.
    std::deque<Cc2530UartFrame> rx_queue_; // host frames not yet on the wire
    bool rx_active_{};
    bool rx_discard_{}; // frame aborted by FLUSH, still occupies the wire
    Cc2530UartFrame rx_frame_{};
    std::uint8_t rx_ucr_{}; // UxUCR latched at the start bit
    unsigned rx_bits_{};
    std::uint64_t rx_start_hf_{};
    std::uint64_t rx_done_hf_{Never};
    std::uint64_t rx_idle_hf_{}; // end of the last frame on the RX wire
    // End of a second stop bit at the wrong level: FE is set then (17.1.4).
    std::uint64_t rx_fe2_hf_{Never};
    BitClock rx_clock_;
    std::uint8_t rx_data_{};                // UxDBUF read register
    std::optional<std::uint8_t> rx_pending_; // received while RX_BYTE = 1
    // SPI master on the pads (17.2.1): UxGCR latched at the start of the
    // byte, the next of the 16 SCK edges, the bits sampled from MISO and
    // the SCK/MOSI output levels.
    bool spi_pads_{};
    std::uint8_t spi_gcr_{};
    unsigned spi_edge_{};
    std::uint8_t spi_rx_{};
    bool sck_out_{};
    bool mosi_out_{};
    // SPI slave (17.2.2, 17.3): last SCK/SSN pad levels seen, bits shifted
    // in the current byte, the transmit shift register and the MISO level.
    bool slave_sck_{};
    bool slave_ssn_{true};
    unsigned slave_bits_{};
    std::uint8_t slave_rx_{};
    std::uint8_t slave_tx_{};
    bool slave_loaded_{};
    bool miso_out_{};
    std::array<std::uint64_t, 32> dma_triggers_{};
};

} // namespace mcs51emu::cc2530

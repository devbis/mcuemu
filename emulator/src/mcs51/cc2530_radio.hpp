#pragma once

// CC2530 IEEE 802.15.4 radio (SWRU191F Chapter 23): RF core registers
// (XREG 0x6000-0x61ff, RFD/RFST/RFIRQF0/RFIRQF1/RFERRF SFRs), the command
// strobe processor (CSP), TX/RX FIFOs, frame filtering, source address
// matching, automatic acknowledgment, CCA/RSSI/LQI and CRC, connected to
// the shared mcuemu::VirtualAir (2.4 GHz O-QPSK, 250 kbit/s).

#include "cc2530_soc.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace mcuemu {
class VirtualAir;
}

namespace mcs51emu::cc2530 {

class Radio final : public Peripheral {
public:
    explicit Radio(Bus &bus);
    ~Radio() override;
    const char *name() const override { return "RF"; }
    void reset(ResetCause cause) override;
    std::uint8_t read(Reg reg) override;
    std::uint8_t peek(Reg reg) const override;
    void write(Reg reg, std::uint8_t value) override;
    std::uint64_t deadline() const override;
    void service(std::uint64_t now) override;
    void power_mode_changed(PowerMode from, PowerMode to) override;
    void clock_changed() override;
    bool may_interrupt() const override;

    // Joins `air` as `node`. Air time (ns) = ns_offset + cycles * 1000 / 32.
    // Host wiring: kept across every chip reset.
    void attach_air(mcuemu::VirtualAir &air, std::string node, std::uint64_t ns_offset);
    // Seeds the receiver noise (RFRND, CSP RANDXY). Kept across resets.
    void set_noise_seed(std::uint64_t seed);

    // ---- RF timing domain (shared with MacTimer, see advance_rf_domain) ----
    // HF time of the next internal radio event, or Never.
    std::uint64_t next_event_hf() const;
    // Processes the event returned by next_event_hf().
    void process_next_event();
    // MAC timer overflow (period wrap) at HF time `hf` (CSP WAIT/WAITX/CSPT).
    void mac_timer_overflow(std::uint64_t hf);
    // MAC timer event 1 or 2 pulse at HF time `hf` (CSP WEVENT1/WEVENT2).
    void mac_timer_event(unsigned which, std::uint64_t hf);

    // Test-only observation (the DMA skeleton cannot record triggers):
    // number of DMA trigger 19 (radio) assertions since construction.
    std::uint64_t dma_trigger_count() const;
    // Test-only: current FSM state number (FSMSTAT0[5:0]).
    unsigned fsm_state() const;

private:
    struct State;
    std::unique_ptr<State> state_;
};

} // namespace mcs51emu::cc2530

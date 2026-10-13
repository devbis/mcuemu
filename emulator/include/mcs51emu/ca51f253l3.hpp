#ifndef MCS51EMU_CA51F253L3_HPP
#define MCS51EMU_CA51F253L3_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iosfwd>
#include <string>

struct uc_struct;

namespace mcs51emu {

struct RunResult {
    std::size_t instructions{};
    std::uint16_t pc{};
    std::uint64_t cycles{};
    bool step_limit_reached{};
    std::string error;
};

// Initial CA51F253L3 memory shell. The core uses a 64 KiB 8051 code address
// space, while image loading enforces the device's 32 KiB flash capacity.
class Ca51F253L3 {
public:
    static constexpr std::size_t FlashSize = 32 * 1024;
    static constexpr std::uint64_t CodeSize = 64 * 1024;
    static constexpr std::uint64_t XdataBase = 0x00010000ULL;
    static constexpr std::uint64_t XdataSize = 64 * 1024;
    static constexpr std::uint64_t XdataFlashPageOffset = 0xf000;
    static constexpr std::uint64_t XdataFlashPageSize = 4096;
    static constexpr std::size_t DataFlashSize = 512;
    static constexpr std::uint64_t IdataBase = 0x00020000ULL;
    static constexpr std::uint64_t SfrBase = 0x00030000ULL;
    static constexpr std::uint64_t MappedInternalSize = 4096;

    Ca51F253L3();
    ~Ca51F253L3();
    Ca51F253L3(const Ca51F253L3 &) = delete;
    Ca51F253L3 &operator=(const Ca51F253L3 &) = delete;

    void load_firmware(const std::string &path);
    void reset();
    RunResult run(std::size_t instruction_limit);

    std::uint8_t read_sfr(std::uint8_t address) const;
    std::uint8_t read_idata(std::uint8_t address) const;
    std::uint8_t read_xdata(std::uint16_t address) const;
    void write_sfr(std::uint8_t address, std::uint8_t value);
    const std::string &uart0_output() const noexcept { return uart0_output_; }
    const std::string &uart1_output() const noexcept { return uart1_output_; }
    // UART1 receive: bytes queue up (up to Uart1RxQueueSize) and are presented
    // one at a time through S1BUF with RI1 set, like the single-byte receive
    // buffer of the chip. Returns false when the queue is full.
    static constexpr std::size_t Uart1RxQueueSize = 16;
    bool uart1_receive(std::uint8_t byte);
    bool uart1_can_receive() const noexcept { return uart1_rx_queue_.size() < Uart1RxQueueSize; }
    void set_trace(std::ostream *output) noexcept { trace_output_ = output; }

private:
    static std::uint64_t xdata_flash_read(uc_struct *uc, std::uint64_t offset,
                                          unsigned size, void *user_data);
    static void xdata_flash_write(uc_struct *uc, std::uint64_t offset,
                                  unsigned size, std::uint64_t value,
                                  void *user_data);
    static std::uint64_t sfr_mmio_read(uc_struct *uc, std::uint64_t offset,
                                       unsigned size, void *user_data);
    static void sfr_mmio_write(uc_struct *uc, std::uint64_t offset,
                               unsigned size, std::uint64_t value,
                               void *user_data);
    static void code_hook(uc_struct *uc, std::uint64_t address,
                          std::uint32_t size, void *user_data);
    void set_code_hook_enabled(bool enabled);
    void check_error(int error, const char *operation) const;
    void rebase_timer0() noexcept;
    void advance_timer0(std::uint64_t cycle_delta) noexcept;
    void start_uart1_transmit(std::uint8_t byte, uc_struct *uc) noexcept;
    void advance_uart1(uc_struct *uc) noexcept;
    std::uint64_t uart1_frame_cycles() const noexcept;
    void update_uart1_interrupt(uc_struct *uc) noexcept;
    void deliver_uart1_rx() noexcept;

    uc_struct *uc_{};
    std::size_t code_hook_{};
    bool code_hook_installed_{};
    bool fast_mode_{};
    std::array<std::uint8_t, 256> sfr_{};
    std::array<std::uint8_t, XdataFlashPageSize> xdata_flash_page_{};
    std::array<std::uint8_t, DataFlashSize> data_flash_{};
    std::uint16_t data_flash_address_{};
    std::uint8_t data_flash_command_{};
    bool data_flash_unlocked_{};
    std::uint64_t timer0_last_cycles_{};
    std::uint64_t uart1_last_cycles_{};
    std::uint64_t uart1_tx_cycles_remaining_{};
    std::uint8_t uart1_tx_byte_{};
    bool uart1_tx_active_{};
    std::deque<std::uint8_t> uart1_tx_waiting_;
    bool uart1_irq_requested_{};
    std::deque<std::uint8_t> uart1_rx_queue_;
    std::uint8_t uart1_rx_byte_{};
    std::string uart0_output_;
    std::string uart1_output_;
    std::ostream *trace_output_{};
};

} // namespace mcs51emu

#endif

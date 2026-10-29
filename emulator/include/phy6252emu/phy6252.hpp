#pragma once

#include "cortexm/core.hpp"
#include "phy6252emu/image.hpp"
#include "mcuemu/radio.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace phy6252emu {

enum class GpioDrive { HiZ, Low, High, PullUp, PullDown };

// Initial PB-03F model. Peripheral coverage is limited to observed Zephyr paths.
class Phy6252 final : public cortexm::Bus {
public:
    Phy6252();
    void load_firmware(const std::vector<std::uint8_t> &image);
    // Optional external 96 KiB ROM image, or a 128 KiB dump with an erased tail.
    void load_rom_dump(const std::vector<std::uint8_t> &dump);
    // PHY62xx efuse_read accepts a block number and an eight-byte output buffer.
    // A supplied 32-byte dump contains four blocks in order; no dump means all
    // blocks are unprogrammed because their device-specific contents are unknown.
    void load_efuse_dump(const std::vector<std::uint8_t> &dump);
    void reset();
    cortexm::RunResult run(std::uint64_t until_cycle);
    cortexm::Core &core() noexcept { return core_; }
    const cortexm::Core &core() const noexcept { return core_; }
    std::vector<std::uint8_t> take_uart_tx();
    void uart_receive(std::uint8_t byte);
    void connect_radio(mcuemu::VirtualAir &air, std::string node);
    bool radio_receiving() const noexcept;
    std::uint8_t radio_channel() const noexcept;
    void receive_radio_frame(const mcuemu::RadioFrame &frame);
    void set_gpio_observer(std::function<void(unsigned)> observer);
    void set_gpio_input(unsigned pin, bool high);
    GpioDrive gpio_drive(unsigned pin) const noexcept;
    std::uint32_t read(std::uint32_t address, unsigned size) override;
    void write(std::uint32_t address, std::uint32_t value, unsigned size) override;

private:
    cortexm::Core core_;
    std::uint8_t *flash_, *ram_, *vector_alias_;
    std::uint32_t run_address_{0};
    std::vector<std::uint8_t> boot_ram_;
    std::vector<std::uint8_t> rom_dump_;
    std::array<std::array<std::uint8_t, 8>, 4> efuse_blocks_{};
    std::array<bool, 4> efuse_block_present_{};
    std::array<bool, 4> efuse_missing_warning_logged_{};
    std::uint8_t lcr_{0}, ier_{0}, dll_{0}, dlm_{0}, mcr_{0}, scratch_{0};
    std::deque<std::uint8_t> rx_;
    std::vector<std::uint8_t> tx_;
    std::unordered_map<std::uint32_t, std::uint32_t> control_registers_;
    std::unordered_map<std::uint32_t, std::uint32_t> rf_registers_;
    std::unordered_map<std::uint32_t, std::uint32_t> ll_registers_;
    std::vector<std::uint8_t> tx_fifo_;
    std::array<std::uint8_t, 0x800> rx_fifo_{};
    std::size_t rx_fifo_read_{0};
    mcuemu::VirtualAir *air_{nullptr};
    std::string radio_node_;
    std::uint64_t radio_tx_end_cycle_{0};
    std::uint64_t radio_rx_after_ns_{0};
    std::uint64_t radio_rx_deadline_cycle_{0};
    std::uint32_t radio_rx_timeout_us_{0};
    bool radio_tx_active_{false};
    bool radio_rx_active_{false};
    std::int8_t radio_rx_rssi_{-100};
    std::uint32_t spif_config_{0x80000000u};
    std::uint32_t spif_command_{0};
    std::uint32_t spif_read_data_{0};
    std::uint32_t dma_raw_transfer_status_{0}, dma_channel_enable_{0};
    std::uint64_t watchdog_deadline_ns_{0};
    bool watchdog_interrupt_pending_{false};
    std::uint64_t rtc_epoch_ns_{0};
    std::uint32_t rtc_epoch_count_{0};
    bool sleep_allowed_{false};
    std::uint32_t sleep_mode_{0};
    bool unexported_init_hook_warned_{false};
    bool pendsv_dispatch_warning_logged_{false};
    bool ll_rand_warning_logged_{false};
    bool baseband_irq_dispatch_warning_logged_{false};
    bool gpio_irq_dispatch_warning_logged_{false};
    std::array<bool, 6> timer_irq_dispatch_warning_logged_{};
    bool ll_context_warning_logged_{false};
    bool rf_sample_e8_warning_logged_{false};
    std::uint32_t heap_start_{0}, heap_size_{0};
    std::uint32_t random_state_{0x6d2b79f5};
    std::map<std::uint32_t, std::uint32_t> free_heap_blocks_;
    std::unordered_map<std::uint32_t, std::uint32_t> heap_allocations_;
    std::string pending_fault_;
    struct TimerState {
        std::uint32_t load{0}, current{0}, control{0};
        bool interrupt_pending{false};
        std::uint64_t last_cycle{0}, cycle_remainder{0};
    };
    std::array<TimerState, 6> timers_{};
    std::uint32_t gpio_data_{0}, gpio_direction_{0};
    std::uint32_t gpio_interrupt_enable_{0}, gpio_interrupt_mask_{0x007fffff};
    std::uint32_t gpio_interrupt_type_{0}, gpio_interrupt_polarity_{0};
    std::uint32_t gpio_interrupt_status_{0}, gpio_input_{0};
    std::function<void(unsigned)> gpio_observer_;

    std::uint32_t read_register(std::uint32_t address, unsigned size);
    void write_register(std::uint32_t address, std::uint32_t value, unsigned size);
    std::uint64_t service_peripherals(std::uint64_t now);
    std::uint64_t service_radio(std::uint64_t now);
    void sync_timer(unsigned timer, std::uint64_t now);
    void update_timer_irq(unsigned timer);
    std::uint32_t gpio_input_data() const noexcept;
    void update_gpio_irq();
    void notify_gpio(std::uint32_t changed);
    void start_radio_tx();
    void schedule_radio_rx_timeout();
    void update_radio_irq();
    std::int8_t radio_energy_dbm(std::uint64_t at_ns) const;
    void rom_log_printf();
    std::uint32_t heap_allocate(std::uint32_t size);
    void heap_release(std::uint32_t address);
    void dma_copy_channel3();
    void watchdog_restart();
    std::uint32_t rtc_count(std::uint64_t at_ns) const noexcept;
    void unsupported_register(std::uint32_t address, const char *operation,
                              std::uint32_t value = 0, unsigned size = 0);
    bool rom_flash_write();
    bool rom_flash_erase();
};
} // namespace phy6252emu

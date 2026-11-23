#pragma once

#include "cortexm/core.hpp"
#include "mcuemu/radio.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace stm32wb55emu {

// STM32WB55 CPU1 (Cortex-M4F) and CPU2 (Cortex-M0+) sharing flash and SRAM.
// Without a raw CPU2 image, the built-in Thread RCP model services IPCC on CPU2.
class Stm32wb55 final : public cortexm::Bus {
public:
    static constexpr std::uint32_t FlashBase = 0x08000000;
    static constexpr std::uint32_t FlashSize = 0x100000;
    static constexpr std::uint32_t RamBase = 0x20000000;
    static constexpr std::uint32_t RamSize = 0x40000;
    static constexpr std::uint32_t ClockHz = 64000000;
    static constexpr std::uint32_t Cpu2ClockHz = 32000000;

    Stm32wb55();
    ~Stm32wb55() override;
    Stm32wb55(const Stm32wb55 &) = delete;
    Stm32wb55 &operator=(const Stm32wb55 &) = delete;

    void load_firmware(const std::vector<std::uint8_t> &image, std::uint32_t offset = 0);
    void load_cpu2_firmware(const std::vector<std::uint8_t> &image,
                            std::uint32_t offset = 0);
    bool cpu2_has_vector_table() const noexcept { return cpu2_vector_table_found_; }
    void reset();
    cortexm::Core &core() noexcept { return *core_; }
    std::uint64_t time_ns() const;
    cortexm::RunResult run_cpu2_until(std::uint64_t target_ns);
    std::uint64_t cpu2_time_ns() const;
    void set_device_id(std::uint64_t id);
    void connect_radio(mcuemu::VirtualAir &air, const std::string &node);
    bool uart_rx(std::uint8_t byte);
    std::vector<std::uint8_t> uart_take_tx();

    std::uint32_t read(std::uint32_t address, unsigned size) override;
    void write(std::uint32_t address, std::uint32_t value, unsigned size) override;

private:
    struct RadioTx {
        std::uint64_t frame_id{};
        std::uint64_t finish_ns{};
        std::uint64_t response_ns{};
        std::uint8_t channel{};
        std::uint8_t sequence{};
        bool wants_ack{};
        bool broadcast{};
        std::vector<std::uint8_t> ack;
        std::int8_t ack_rssi{-50};
    };

    class Cpu2Bus;

    std::uint64_t service(std::uint64_t now_cycle);
    std::uint64_t service_cpu2(std::uint64_t now_cycle);
    std::uint64_t core_time_ns(const cortexm::Core &core) const;
    void request_service_at_ns(cortexm::Core &core, std::uint64_t time_ns);
    void rng_advance(std::uint64_t now_ns);
    void rng_update_irq();
    std::uint32_t rng_next_word();
    std::uint32_t lptim_counter(std::uint64_t now_ns) const;
    void lptim_advance(std::uint64_t now_ns);
    std::uint64_t lptim_next_event_ns(std::uint64_t now_ns) const;
    void lptim_update_irq();
    std::uint32_t read_mmio(std::uint32_t address, unsigned size);
    void write_mmio(std::uint32_t address, std::uint32_t value, unsigned size);
    std::uint32_t read_cpu2_mmio(std::uint32_t address, unsigned size);
    void write_cpu2_mmio(std::uint32_t address, std::uint32_t value, unsigned size);
    void execute_cpu2_rf_register_command(std::uint32_t command);
    std::uint16_t cpu2_mac_read_register(unsigned index, std::uint64_t now_ns);
    void cpu2_mac_write_register(unsigned index, std::uint16_t value,
                                 std::uint64_t now_ns);
    void cpu2_mac_update_irq();
    void cpu2_mac_write_command(std::uint16_t command, std::uint64_t now_ns);
    void cpu2_mac_advance(std::uint64_t now_ns);
    void cpu2_mac_poll_radio(std::uint64_t now_ns);
    void cpu2_mac_receive_frame(const mcuemu::RadioFrame &frame,
                                std::uint64_t now_ns);
    std::uint32_t cpu2_mac_counter(std::uint64_t now_ns) const;
    void ipcc_command(unsigned channel);
    void complete_system_command();
    void start_cpu2();
    void initialize_wireless_info();
    bool memory_read(std::uint32_t address, void *out, std::size_t size) const;
    bool memory_write(std::uint32_t address, const void *data, std::size_t size);
    std::uint32_t memory_u32(std::uint32_t address) const;
    void memory_put32(std::uint32_t address, std::uint32_t value);
    void signal_cpu1_rx(unsigned channel);
    void signal_cpu1_tx(unsigned channel);
    void signal_cpu2_rx(unsigned channel);
    void signal_cpu2_tx(unsigned channel);
    void signal_ready_event();
    void signal_spinel(std::vector<std::uint8_t> frame);
    void signal_system_event(std::uint16_t subevent);
    std::uint32_t ref_table_entry(unsigned index) const;
    std::uint32_t thread_table_entry(unsigned index) const;
    void handle_spinel_request(const std::uint8_t *frame, std::size_t size);
    void handle_spinel_tx(const std::uint8_t *value, std::size_t size,
                          std::uint8_t tid);
    std::int8_t radio_rssi_dbm(std::uint8_t channel, std::uint64_t at_ns) const;
    void poll_radio(std::uint64_t now_ns);
    void complete_radio_tx(std::uint64_t now_ns);
    void queue_rx_frame(const mcuemu::RadioFrame &frame);
    void deliver_rx_frame();
    std::vector<std::uint8_t> spinel_value(std::uint32_t property) const;
    static std::uint16_t ieee802154_fcs(const std::vector<std::uint8_t> &bytes);
    static bool hdlc_decode(const std::uint8_t *data, std::size_t size,
                            std::vector<std::uint8_t> &frame);
    static std::vector<std::uint8_t> hdlc_encode(const std::vector<std::uint8_t> &frame);
    static void append_varint(std::vector<std::uint8_t> &out, std::uint32_t value);
    static bool read_varint(const std::uint8_t *data, std::size_t size,
                            std::size_t &used, std::uint32_t &value);

    cortexm::Core::Config config_;
    std::unique_ptr<Cpu2Bus> cpu2_bus_;
    std::unique_ptr<cortexm::Core> core_;
    std::unique_ptr<cortexm::Core> cpu2_core_;
    std::uint8_t *flash_{nullptr};
    std::uint8_t *ram_{nullptr};
    std::uint8_t *system_memory_{nullptr};
    std::uint32_t cpu2_vector_table_address_{0};
    std::uint32_t cpu2_flash_offset_{0};
    std::uint32_t cpu2_reset_vector_offset_{0};
    std::map<std::uint32_t, std::uint32_t> registers_;
    std::array<std::uint8_t, 0x7e> cpu2_rf_indirect_registers_{};
    std::array<std::uint16_t, 0x100> cpu2_mac_registers_{};
    std::deque<std::uint8_t> cpu2_mac_rx_fifo_;
    std::optional<mcuemu::RadioFrame> cpu2_mac_rx_pending_frame_;
    std::vector<std::uint8_t> cpu2_mac_tx_fifo_;
    std::array<bool, 4> cpu2_mac_compare_fired_{};
    std::uint32_t ipcc_c1cr_{0};
    std::uint32_t ipcc_c1mr_{0xffffffffu};
    std::uint32_t ipcc_c2cr_{0};
    std::uint32_t ipcc_c2mr_{0xffffffffu};
    std::uint32_t c1toc2sr_{0};
    std::uint32_t c2toc1sr_{0};
    std::uint32_t lptim_isr_{0};
    std::uint32_t lptim_ier_{0};
    std::uint32_t lptim_cfgr_{0};
    std::uint32_t lptim_cr_{0};
    std::uint32_t lptim_cmp_{0};
    std::uint32_t lptim_arr_{0xffff};
    std::uint64_t lptim_epoch_ns_{0};
    std::uint32_t lptim_epoch_count_{0};
    std::uint32_t rng_cr_{0};
    std::uint32_t rng_state_{0x6d2b79f5u};
    std::uint64_t rng_next_ready_ns_{0};
    bool rng_data_ready_{false};
    std::uint64_t device_id_{0x0123456789abcdefull};
    std::uint16_t short_address_{0xffff};
    std::uint32_t code_end_{0};
    std::uint32_t vector_table_offset_{0};
    bool cpu2_image_loaded_{false};
    bool cpu2_vector_table_found_{false};
    bool cpu2_code_alias_mapped_{false};
    bool cpu2_model_active_{false};
    bool cpu2_model_boot_pending_{false};
    std::uint32_t cpu2_model_pending_channels_{0};
    bool cpu2_started_{false};
    bool cpu2_stopped_{false};
    bool cpu2_mac_tx_has_phr_{false};
    bool cpu2_mac_tx_fifo_busy_{false};
    bool cpu2_mac_tx_pending_{false};
    bool cpu2_mac_tx_active_{false};
    bool cpu2_mac_rssi_valid_{false};
    std::uint8_t cpu2_mac_tx_phr_{};
    std::int8_t cpu2_mac_rssi_dbm_{-100};
    std::uint64_t cpu2_mac_radio_poll_ns_{0};
    std::uint64_t cpu2_mac_rx_next_event_ns_{0};
    std::uint64_t cpu2_mac_tx_start_ns_{0};
    std::uint64_t cpu2_mac_tx_finish_ns_{0};
    cortexm::RunResult cpu2_stop_result_{};
    std::uint64_t cpu2_boot_time_ns_{0};
    std::uint64_t cpu2_boot_cycle_{0};
    cortexm::Core *mmio_core_{nullptr};
    mcuemu::VirtualAir *air_{nullptr};
    std::string radio_node_;
    std::uint64_t radio_poll_ns_{0};
    std::uint8_t radio_channel_{11};
    std::uint8_t radio_tx_power_{8};
    std::int8_t cca_threshold_{-75};
    bool radio_enabled_{false};
    bool radio_raw_stream_{false};
    bool rx_notification_outstanding_{false};
    std::uint32_t spinel_tid_counter_{0};
    std::vector<std::uint8_t> eui64_;
    std::deque<std::uint8_t> uart_rx_;
    std::vector<std::uint8_t> uart_tx_;
    std::deque<std::vector<std::uint8_t>> rx_queue_;
    std::vector<std::uint8_t> pending_spinel_response_;
    std::uint8_t pending_tid_{0};
    std::uint32_t pending_property_{0};
    std::uint64_t pending_response_ns_{0};
    bool spinel_response_pending_{false};
    bool pending_tx_response_{false};
    RadioTx pending_tx_{};
    std::uint64_t last_rx_ns_{0};
};

} // namespace stm32wb55emu

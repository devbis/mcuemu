// The ESP32 IEEE 802.15.4 MAC block on its own: two instances on one medium,
// driven the way esp-hal's driver drives the registers.

#include "../test_support.hpp"
#include "esp32emu/esp32c6_regs.hpp"
#include "esp32emu/esp32h2_regs.hpp"
#include "aes.hpp"
#include "gpio.hpp"
#include "esp32emu/esp32.hpp"
#include "ieee802154.hpp"

#include <cstring>
#include <memory>

namespace mcuemu_tests {

namespace {
namespace r = esp32c6_regs::ieee802154::reg;
constexpr std::uint32_t Hz = 160000000;

struct Node {
    std::vector<std::uint8_t> ram = std::vector<std::uint8_t>(0x1000, 0);
    std::uint64_t cycles{0};
    bool line{false};
    std::unique_ptr<esp32emu::Ieee802154> mac;

    Node(mcuemu::VirtualAir &air, const char *name) {
        mac = std::make_unique<esp32emu::Ieee802154>(
            0x600a3000, Hz, 12, [this] { return cycles; }, [this](unsigned, bool level) { line = level; },
            [this](std::uint32_t a, void *out, std::size_t n) {
                std::memcpy(out, &ram[a], n);
                return true;
            },
            [this](std::uint32_t a, const void *in, std::size_t n) {
                std::memcpy(&ram[a], in, n);
                return true;
            },
            [](std::uint64_t) {});
        mac->connect(air, name);
    }
    void w(std::uint32_t reg, std::uint32_t v) { mac->write(reg, v, 4); }
    std::uint32_t rd(std::uint32_t reg) { return mac->read(reg, 4); }
    void command(std::uint8_t op) { w(r::COMMAND, op); }
};

// Advances both nodes to `us` microseconds in 8 us steps.
void run(Node &a, Node &b, std::uint64_t us) {
    for (std::uint64_t t = 0; t <= us; t += 8) {
        a.cycles = b.cycles = t * (Hz / 1000000);
        a.mac->service(a.cycles);
        b.mac->service(b.cycles);
    }
}

void configure(Node &n, std::uint16_t pan, std::uint16_t addr) {
    n.w(r::EVENT_EN, 0x1fff);
    n.w(r::CHANNEL, 3 + 5 * (15 - 11)); // channel 15
    n.w(r::CTRL_CFG, (1u << 0) | (1u << 3) | (1u << 28)); // auto ACK tx/rx, interface 0
    n.w(r::INF0_PAN_ID, pan);
    n.w(r::INF0_SHORT_ADDR, addr);
}
} // namespace

void test_esp32_mac_delivers_a_unicast_frame_and_gets_the_hardware_ack() {
    mcuemu::VirtualAir air;
    Node a(air, "esp-a"), b(air, "esp-b");
    configure(a, 0x1234, 0x0001);
    configure(b, 0x1234, 0x0002);

    // B listens; its receive buffer is at 0x100.
    b.w(r::RXDMA_ADDR, 0x100);
    b.command(0x42);

    // A sends a data frame to B asking for an ACK: FCF 0x8861 (data, AR, PAN
    // compression, short addresses), seq 7, PAN, dest, src, payload "hi".
    const std::uint8_t frame[] = {0x61, 0x88, 0x07, 0x34, 0x12, 0x02, 0x00, 0x01, 0x00, 'h', 'i'};
    a.ram[0x10] = sizeof frame + 2; // PHR counts the FCS
    std::memcpy(&a.ram[0x11], frame, sizeof frame);
    a.w(r::TXDMA_ADDR, 0x10);
    a.w(r::RXDMA_ADDR, 0x200); // where the ACK lands
    a.command(0x41);

    run(a, b, 2000);
    CHECK(b.rd(r::EVENT_STATUS) & 2u); // RxDone
    CHECK(b.ram[0x100] == sizeof frame + 2);
    CHECK(std::memcmp(&b.ram[0x101], frame, sizeof frame) == 0);
    CHECK(b.rd(r::EVENT_STATUS) & 4u); // AckTxDone after the ACK went out
    CHECK(b.line);
    CHECK(a.rd(r::EVENT_STATUS) & 1u); // TxDone
    CHECK(a.rd(r::EVENT_STATUS) & 8u); // AckRxDone
    CHECK(a.ram[0x200] == 5);          // PHR of the ACK: 3 bytes + FCS
    CHECK(a.ram[0x201] == 0x02 && a.ram[0x203] == 7); // ACK frame type, the frame's sequence number

    // The line follows EVENT_STATUS & EVENT_EN and the status is write-one-to-clear.
    b.w(r::EVENT_STATUS, 0xffff);
    CHECK(!b.line);
    CHECK(b.rd(r::EVENT_STATUS) == 0);
}

void test_esp32_mac_filters_foreign_frames_and_times_the_ack_out() {
    mcuemu::VirtualAir air;
    Node a(air, "esp-a"), b(air, "esp-b");
    configure(a, 0x1234, 0x0001);
    configure(b, 0x1234, 0x0002);
    b.w(r::RXDMA_ADDR, 0x100);
    b.command(0x42);

    // Addressed to 0x0009: B's filter drops it, so A never sees an ACK.
    const std::uint8_t frame[] = {0x61, 0x88, 0x08, 0x34, 0x12, 0x09, 0x00, 0x01, 0x00, 'x'};
    a.ram[0x10] = sizeof frame + 2;
    std::memcpy(&a.ram[0x11], frame, sizeof frame);
    a.w(r::TXDMA_ADDR, 0x10);
    a.w(r::RXDMA_ADDR, 0x200);
    a.command(0x41);
    // The driver's ACK timeout: timer0 counts microseconds from its start command.
    a.w(r::TIME0_THRESHOLD, 1000);
    a.command(0x4c);
    run(a, b, 3000);
    CHECK(!(b.rd(r::EVENT_STATUS) & 2u));
    CHECK(a.rd(r::EVENT_STATUS) & 1u);   // TxDone: the frame went out
    CHECK(!(a.rd(r::EVENT_STATUS) & 8u)); // no ACK
    CHECK(a.rd(r::EVENT_STATUS) & 0x100u); // Timer0Overflow
}

void test_esp32_aes_block_and_register_interface() {
    // FIPS-197 appendix C.1 (AES-128) and C.3 (AES-256).
    const std::uint8_t plain[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    std::uint8_t key[32];
    for (unsigned i = 0; i < 32; ++i)
        key[i] = static_cast<std::uint8_t>(i);
    std::uint8_t out[16], back[16];
    esp32emu::aes_block(key, 16, plain, out, false);
    const std::uint8_t expect128[16] = {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
                                        0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a};
    CHECK(std::memcmp(out, expect128, 16) == 0);
    esp32emu::aes_block(key, 16, out, back, true);
    CHECK(std::memcmp(back, plain, 16) == 0);
    esp32emu::aes_block(key, 32, plain, out, false);
    const std::uint8_t expect256[16] = {0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf,
                                        0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89};
    CHECK(std::memcmp(out, expect256, 16) == 0);

    // Through the registers, the way esp-hal drives them.
    esp32emu::Aes aes(0x60088000);
    for (unsigned i = 0; i < 4; ++i) {
        std::uint32_t word;
        std::memcpy(&word, key + 4 * i, 4);
        aes.write(4 * i, word, 4);
        std::memcpy(&word, plain + 4 * i, 4);
        aes.write(0x20 + 4 * i, word, 4);
    }
    aes.write(0x40, 0, 4);
    aes.write(0x48, 1, 4);
    CHECK(aes.read(0x4c, 4) == 0);
    std::uint8_t result[16];
    for (unsigned i = 0; i < 4; ++i) {
        const std::uint32_t word = aes.read(0x30 + 4 * i, 4);
        std::memcpy(result + 4 * i, &word, 4);
    }
    CHECK(std::memcmp(result, expect128, 16) == 0);
}

void test_esp32_gpio_drive_input_and_interrupts() {
    namespace g = esp32h2_regs::gpio::reg;
    esp32emu::IoMux mux(0x60090000, 28);
    bool line = false;
    esp32emu::Gpio gpio(0x60091000, 28, 22, mux, [&](unsigned, bool level) { line = level; });
    int drive_events = 0;
    gpio.on_drive_changed = [&](unsigned) { ++drive_events; };

    CHECK(gpio.drive(9) == esp32emu::PadDrive::HiZ);
    CHECK((gpio.read(g::IN, 4) >> 9 & 1u) == 1u); // an idle input reads high
    // A pull-up from IO_MUX shows as a pull; GPIO9 output needs the simple-GPIO selection and the enable.
    mux.write(4 + 4 * 9, 1u << esp32h2_regs::io_mux::field::GPIO0_FUN_WPU_shift, 4);
    CHECK(gpio.drive(9) == esp32emu::PadDrive::PullUp);
    gpio.write(g::FUNC0_OUT_SEL_CFG + 4 * 9, 0x80, 4);
    gpio.write(g::ENABLE_W1TS, 1u << 9, 4);
    gpio.write(g::OUT_W1TC, 1u << 9, 4);
    CHECK(gpio.drive(9) == esp32emu::PadDrive::Low);
    gpio.write(g::OUT_W1TS, 1u << 9, 4);
    CHECK(gpio.drive(9) == esp32emu::PadDrive::High);
    CHECK(drive_events > 0);

    // A falling-edge interrupt on GPIO5, enabled for the CPU: status latches until W1TC.
    gpio.write(g::PIN0 + 4 * 5, (2u << esp32h2_regs::gpio::field::PIN0_INT_TYPE_shift) |
                                    (1u << esp32h2_regs::gpio::field::PIN0_INT_ENA_shift), 4);
    gpio.set_input(5, false);
    CHECK(line);
    CHECK((gpio.read(g::STATUS, 4) >> 5 & 1u) == 1u);
    gpio.set_input(5, true);
    CHECK(line); // still latched
    gpio.write(g::STATUS_W1TC, 1u << 5, 4);
    CHECK(!line);
}

void test_esp32_uart_receive_fifo_and_interrupt() {
    namespace u = esp32c6_regs::uart0;
    esp32emu::Esp32 chip;
    CHECK(chip.uart_can_receive());
    chip.write(esp32c6_regs::uart0::base + u::reg::INT_ENA, 1u << 8, 4); // RXFIFO_TOUT
    CHECK(chip.read(esp32c6_regs::uart0::base + u::reg::STATUS, 4) == 0);
    CHECK(chip.uart_rx('a'));
    CHECK(chip.uart_rx('b'));
    CHECK((chip.read(esp32c6_regs::uart0::base + u::reg::STATUS, 4) & 0xff) == 2);
    CHECK((chip.read(esp32c6_regs::uart0::base + u::reg::INT_ST, 4) >> 8 & 1u) == 1u);
    CHECK(chip.read(esp32c6_regs::uart0::base + u::reg::FIFO, 4) == 'a');
    CHECK(chip.read(esp32c6_regs::uart0::base + u::reg::FIFO, 4) == 'b');
    CHECK((chip.read(esp32c6_regs::uart0::base + u::reg::INT_ST, 4) >> 8 & 1u) == 0u); // drained
}

} // namespace mcuemu_tests

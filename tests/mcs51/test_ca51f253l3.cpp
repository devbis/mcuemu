#include "mcs51emu/ca51f253l3.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

unsigned failures = 0;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::cerr << __FILE__ << ':' << __LINE__                           \
                      << ": check failed: " #condition << '\n';                \
            ++failures;                                                       \
        }                                                                      \
    } while (false)

std::filesystem::path temp_file(const char *name) {
    return std::filesystem::temp_directory_path() /
           (std::string("tc32-emu-") + name);
}

void write_binary(const std::filesystem::path &path,
                  const std::uint8_t *bytes, std::size_t size) {
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char *>(bytes),
                 static_cast<std::streamsize>(size));
}

void test_ca51_raw_image_executes_code_and_xdata_access() {
    const auto path = temp_file("mcs51-raw.bin");
    constexpr std::array<std::uint8_t, 14> program{
        0x75, 0x90, 0x55, // MOV P1,#0x55
        0x75, 0x30, 0x6d, // MOV 0x30,#0x6d
        0x90, 0x01, 0x00, // MOV DPTR,#0x0100
        0x74, 0x71,       // MOV A,#0x71
        0xf0,             // MOVX @DPTR,A
        0x80, 0xfe        // SJMP $
    };
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(6);
    CHECK(run.error.empty());
    CHECK(run.instructions == 6);
    CHECK(run.pc == 12);
    CHECK(run.cycles == 11);
    CHECK(device.read_sfr(0x90) == 0x55);
    CHECK(device.read_idata(0x30) == 0x6d);
    CHECK(device.read_xdata(0x0100) == 0x71);
    std::filesystem::remove(path);
}

void test_ca51_intel_hex_image_loads() {
    const auto path = temp_file("mcs51.hex");
    {
        std::ofstream output(path);
        output << ":04000000742A80FEE0\n"
                  ":00000001FF\n";
    }

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(2);
    CHECK(run.error.empty());
    CHECK(run.instructions == 2);
    CHECK(run.pc == 2);
    CHECK(run.cycles == 3);
    std::filesystem::remove(path);
}

void test_ca51_cjne_indirect_uses_immediate_and_relative_bytes() {
    const auto path = temp_file("mcs51-cjne.bin");
    constexpr std::array<std::uint8_t, 13> program{
        0x78, 0x40,       // MOV R0,#0x40
        0x76, 0x12,       // MOV @R0,#0x12
        0xb6, 0x34, 0x02, // CJNE @R0,#0x34,skip
        0x74, 0x00,       // skipped
        0x74, 0x5a,       // MOV A,#0x5a
        0x80, 0xfe        // SJMP $
    };
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(5);
    CHECK(run.error.empty());
    CHECK(run.instructions == 5);
    CHECK(run.pc == 11);
    CHECK(run.cycles == 7);
    CHECK(device.read_idata(0x40) == 0x12);
    CHECK(device.read_sfr(0xe0) == 0x5a);
    CHECK((device.read_sfr(0xd0) & 0x80) != 0);
    std::filesystem::remove(path);
}

void test_ca51_acall_ret_preserve_stack_and_return_pc() {
    const auto path = temp_file("mcs51-call-ret.bin");
    constexpr std::array<std::uint8_t, 11> program{
        0x11, 0x08,       // ACALL 0x0008
        0x80, 0xfe,       // SJMP $
        0x00, 0x00, 0x00, 0x00,
        0x74, 0x5a,       // MOV A,#0x5a
        0x22              // RET
    };
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    device.write_sfr(0x81, 0x7f); // Firmware stack crosses into upper IDATA
    const mcs51emu::RunResult run = device.run(3);
    CHECK(run.error.empty());
    CHECK(run.pc == 0x0002);
    CHECK(device.read_sfr(0xe0) == 0x5a);
    CHECK(device.read_sfr(0x81) == 0x7f);
    std::filesystem::remove(path);
}

void test_ca51_cross_page_nested_call_stack() {
    const auto path = temp_file("mcs51-cross-page-call.bin");
    std::array<std::uint8_t, 0x6904> program{};
    program.fill(0x00);
    program[0] = 0x02;
    program[1] = 0x60;
    program[2] = 0x00; // LJMP 0x6000
    program[0x6000] = 0xd1;
    program[0x6001] = 0xf8; // ACALL 0x66f8
    program[0x6002] = 0x80;
    program[0x6003] = 0xfe; // SJMP $
    program[0x66f8] = 0x12;
    program[0x66f9] = 0x69;
    program[0x66fa] = 0x00; // LCALL 0x6900
    program[0x66fb] = 0x74;
    program[0x66fc] = 0x5a; // MOV A,#0x5a
    program[0x66fd] = 0x22; // RET
    program[0x6900] = 0x75;
    program[0x6901] = 0x40;
    program[0x6902] = 0x77; // MOV 0x40,#0x77
    program[0x6903] = 0x22; // RET
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    device.write_sfr(0x81, 0x7f);
    const mcs51emu::RunResult run = device.run(7);
    CHECK(run.error.empty());
    CHECK(run.pc == 0x6002);
    CHECK(device.read_sfr(0xe0) == 0x5a);
    CHECK(device.read_idata(0x40) == 0x77);
    CHECK(device.read_sfr(0x81) == 0x7f);
    std::filesystem::remove(path);
}

void test_ca51_compiler_save_restore_helper_abi() {
    const auto path = temp_file("mcs51-save-restore-abi.bin");
    std::array<std::uint8_t, 0x6a30> program{};
    program.fill(0x00);
    program[0] = 0x02;
    program[1] = 0x5f;
    program[2] = 0xfc; // LJMP 0x5ffc
    program[0x5ffc] = 0x12;
    program[0x5ffd] = 0x60;
    program[0x5ffe] = 0x00; // LCALL function at 0x6000
    program[0x5fff] = 0x80;
    program[0x6000] = 0x12;
    program[0x6001] = 0x69;
    program[0x6002] = 0xf6; // LCALL __mcs51_save_r3
    program[0x6003] = 0x74;
    program[0x6004] = 0x5a; // function body begins after trampoline
    program[0x6005] = 0x12;
    program[0x6006] = 0x6a;
    program[0x6007] = 0x02; // LCALL __mcs51_restore_r3
    program[0x6008] = 0x22; // RET to outer caller
    const std::array<std::uint8_t, 12> save_r3{
        0xd0, 0x83, 0xd0, 0x82, 0xc0, 0x02,
        0xc0, 0x03, 0xc0, 0x04, 0xe4, 0x73};
    std::copy(save_r3.begin(), save_r3.end(), program.begin() + 0x69f6);
    const std::array<std::uint8_t, 15> restore_r3{
        0xd0, 0x83, 0xd0, 0x82, 0xd0, 0x04, 0xd0, 0x03,
        0xd0, 0x02, 0xc0, 0x82, 0xc0, 0x83, 0x22};
    std::copy(restore_r3.begin(), restore_r3.end(), program.begin() + 0x6a02);
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    device.write_sfr(0x81, 0x7f);
    const mcs51emu::RunResult run = device.run(21);
    CHECK(run.error.empty());
    CHECK(run.pc == 0x5fff);
    CHECK(device.read_sfr(0xe0) == 0x5a);
    CHECK(device.read_sfr(0x81) == 0x7f);
    std::filesystem::remove(path);
}

void test_ca51_sdk_data_flash_read_write_and_erase() {
    const auto path = temp_file("mcs51-data-flash.bin");
    std::array<std::uint8_t, 128> program{};
    std::size_t length = 0;
    const auto emit = [&](std::uint8_t byte) {
        program[length++] = byte;
    };
    const auto mov_dptr = [&](std::uint8_t high, std::uint8_t low) {
        emit(0x90);
        emit(high);
        emit(low);
    };
    const auto mov_a = [&](std::uint8_t value) {
        emit(0x74);
        emit(value);
    };
    const auto write_xdata = [&](std::uint16_t address, std::uint8_t value) {
        mov_dptr(static_cast<std::uint8_t>(address >> 8),
                 static_cast<std::uint8_t>(address));
        mov_a(value);
        emit(0xf0); // MOVX @DPTR,A
    };
    const auto set_data_address_zero = [&]() {
        write_xdata(0xfc05, 0);
        write_xdata(0xfc06, 0);
    };

    write_xdata(0xfc03, 0x2a); // unlock data flash
    set_data_address_zero();
    write_xdata(0xfc01, 2); // byte write command
    write_xdata(0xfc02, 0x5a);
    write_xdata(0xfc01, 0); // finish command

    set_data_address_zero();
    write_xdata(0xfc01, 1); // byte read command
    mov_dptr(0xfc, 0x02);
    emit(0xe0); // MOVX A,@DPTR
    emit(0xf5);
    emit(0x40); // MOV 0x40,A

    write_xdata(0xfc03, 0xaa); // lock, then unlock for erase
    write_xdata(0xfc03, 0x2a);
    set_data_address_zero();
    write_xdata(0xfc01, 3); // erase sector 0
    set_data_address_zero();
    write_xdata(0xfc01, 1);
    mov_dptr(0xfc, 0x02);
    emit(0xe0);
    emit(0xf5);
    emit(0x41); // MOV 0x41,A
    emit(0x80);
    emit(0xfe); // SJMP $
    write_binary(path, program.data(), length);

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(100);
    CHECK(run.error.empty());
    CHECK(device.read_idata(0x40) == 0x5a);
    CHECK(device.read_idata(0x41) == 0xff);
    std::filesystem::remove(path);
}

void test_ca51_uart0_sbuf_emits_bytes_and_sets_ti0() {
    const auto path = temp_file("mcs51-uart0.bin");
    constexpr std::array<std::uint8_t, 14> program{
        0x75, 0x98, 0x50, // MOV S0CON,#mode 1
        0x75, 0x99, 'O',  // MOV S0BUF,#'O'
        0x30, 0x99, 0xfd, // JNB TI0,$
        0xc2, 0x99,       // CLR TI0
        0x75, 0x99, 'K'   // MOV S0BUF,#'K'
    };
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(5);
    CHECK(run.error.empty());
    CHECK(device.uart0_output() == "OK");
    CHECK((device.read_sfr(0x98) & 0x02) != 0);
    std::filesystem::remove(path);
}

void test_ca51_uart1_transmit_interrupt_clears_flag() {
    const auto path = temp_file("mcs51-uart1.bin");
    std::array<std::uint8_t, 0x60> program{};
    program.fill(0x00);
    program[0] = 0x02;
    program[1] = 0x00;
    program[2] = 0x40; // LJMP main at 0x0040
    program[0x33] = 0x02;
    program[0x34] = 0x00;
    program[0x35] = 0x51; // UART1 interrupt vector
    program[0x40] = 0x75;
    program[0x41] = 0xa8;
    program[0x42] = 0xc0; // MOV IE,#(EA|ES1)
    program[0x43] = 0x75;
    program[0x44] = 0x9a;
    program[0x45] = 0xd0; // MOV S1CON,#UART1 mode
    program[0x46] = 0x75;
    program[0x47] = 0x9d;
    program[0x48] = 0x03; // UART1 reload high
    program[0x49] = 0x75;
    program[0x4a] = 0x9c;
    program[0x4b] = 0xff; // reload=0x3ff -> 115200 baud at 3.6864 MHz
    program[0x4c] = 0x75;
    program[0x4d] = 0x9b;
    program[0x4e] = 'A'; // MOV S1BUF,#'A'
    program[0x4f] = 0x80;
    program[0x50] = 0xfe; // SJMP $
    program[0x51] = 0x75;
    program[0x52] = 0x9a;
    program[0x53] = 0xd2; // W1C TI1
    program[0x54] = 0x75;
    program[0x55] = 0x40;
    program[0x56] = 0x77; // MOV 0x40,#0x77
    program[0x57] = 0x32; // RETI
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(1000);
    CHECK(run.error.empty());
    CHECK(device.uart1_output() == "A");
    CHECK((device.read_sfr(0x9a) & 0x02) == 0);
    CHECK(device.read_idata(0x40) == 0x77);
    std::filesystem::remove(path);
}

void test_ca51_uart1_transmits_queued_byte_after_frame_time() {
    const auto path = temp_file("mcs51-uart1-queue.bin");
    std::array<std::uint8_t, 0x90> program{};
    program.fill(0x00);
    program[0] = 0x02;
    program[1] = 0x00;
    program[2] = 0x40; // LJMP main at 0x0040
    program[0x33] = 0x02;
    program[0x34] = 0x00;
    program[0x35] = 0x80; // UART1 interrupt vector
    program[0x40] = 0x75;
    program[0x41] = 0xa8;
    program[0x42] = 0xc0; // MOV IE,#(EA|ES1)
    program[0x43] = 0x75;
    program[0x44] = 0x9a;
    program[0x45] = 0xd0; // MOV S1CON,#UART1 mode
    program[0x46] = 0x75;
    program[0x47] = 0x9d;
    program[0x48] = 0x03;
    program[0x49] = 0x75;
    program[0x4a] = 0x9c;
    program[0x4b] = 0xff;
    program[0x4c] = 0x75;
    program[0x4d] = 0x9b;
    program[0x4e] = 'A'; // Start first byte.
    program[0x4f] = 0x75;
    program[0x50] = 0x40;
    program[0x51] = 0x01; // Firmware marks the queued byte after writing S1BUF.
    program[0x52] = 0x80;
    program[0x53] = 0xfe; // SJMP $
    // ISR: acknowledge TI1, send B if the firmware queue has a byte, RETI.
    program[0x80] = 0x75;
    program[0x81] = 0x9a;
    program[0x82] = 0xd2; // W1C TI1
    program[0x83] = 0xe5;
    program[0x84] = 0x40; // MOV A,0x40
    program[0x85] = 0x60;
    program[0x86] = 0x05; // JZ RETI
    program[0x87] = 0x15;
    program[0x88] = 0x40; // DEC 0x40
    program[0x89] = 0x75;
    program[0x8a] = 0x9b;
    program[0x8b] = 'B'; // MOV S1BUF,#'B'
    program[0x8c] = 0x32; // RETI
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(2000);
    CHECK(run.error.empty());
    CHECK(device.uart1_output() == "AB");
    CHECK((device.read_sfr(0x9a) & 0x02) == 0);
    CHECK(device.read_idata(0x40) == 0);
    std::filesystem::remove(path);
}

void test_ca51_pwm_update_request_self_clears() {
    const auto path = temp_file("mcs51-pwmupd.bin");
    constexpr std::array<std::uint8_t, 3> program{0x75, 0xdb, 0x01};
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(1);
    CHECK(run.error.empty());
    CHECK(device.read_sfr(0xdb) == 0);
    std::filesystem::remove(path);
}

void test_ca51_timer0_overflow_enters_isr_and_reti_returns() {
    const auto path = temp_file("mcs51-timer0.bin");
    std::array<std::uint8_t, 16> program{};
    program.fill(0x00); // NOPs around the timer vector
    program[0x01] = 0x80;
    program[0x02] = 0xfe; // SJMP $
    program[0x0b] = 0x75;
    program[0x0c] = 0x40;
    program[0x0d] = 0x77; // MOV 0x40,#0x77
    program[0x0e] = 0x32; // RETI
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    device.write_sfr(0x89, 0x01); // Timer0 mode 1
    device.write_sfr(0x8c, 0xff);
    device.write_sfr(0x8a, 0xfe);
    device.write_sfr(0x88, 0x10); // Timer0 run
    device.write_sfr(0xa8, 0x82); // EA and ET0

    const mcs51emu::RunResult before_irq = device.run(3);
    CHECK(before_irq.error.empty());
    CHECK(before_irq.pc == 0x000b);
    const mcs51emu::RunResult in_isr = device.run(2);
    CHECK(in_isr.error.empty());
    CHECK(in_isr.pc == 0x0001);
    CHECK(device.read_idata(0x40) == 0x77);
    CHECK(device.read_sfr(0x81) == 0x07);
    std::filesystem::remove(path);
}

void test_ca51_timer0_can_be_enabled_during_fast_run() {
    const auto path = temp_file("mcs51-timer0-fast-to-hook.bin");
    std::array<std::uint8_t, 0x40> program{};
    program.fill(0x00);
    program[0] = 0x02;
    program[1] = 0x00;
    program[2] = 0x20; // LJMP main at 0x0020
    program[0x0b] = 0x75;
    program[0x0c] = 0x40;
    program[0x0d] = 0x77; // Timer0 ISR: MOV 0x40,#0x77
    program[0x0e] = 0x32; // RETI
    program[0x20] = 0x75;
    program[0x21] = 0x89;
    program[0x22] = 0x01; // MOV TMOD,#mode 1
    program[0x23] = 0x75;
    program[0x24] = 0x8c;
    program[0x25] = 0xff; // MOV TH0,#0xff
    program[0x26] = 0x75;
    program[0x27] = 0x8a;
    program[0x28] = 0xfe; // MOV TL0,#0xfe
    program[0x29] = 0x75;
    program[0x2a] = 0xa8;
    program[0x2b] = 0x82; // MOV IE,#(EA|ET0)
    program[0x2c] = 0x75;
    program[0x2d] = 0x88;
    program[0x2e] = 0x10; // Start Timer0 while executing the fast path
    program[0x2f] = 0x80;
    program[0x30] = 0xfe; // SJMP $
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(20);
    CHECK(run.error.empty());
    CHECK(device.read_idata(0x40) == 0x77);
    CHECK((device.read_sfr(0x88) & 0x20) == 0);
    std::filesystem::remove(path);
}

void test_ca51_llvm_delay_loop_finishes() {
    const auto path = temp_file("mcs51-llvm-delay-loop.bin");
    std::array<std::uint8_t, 0x38> program{};
    program[0] = 0x75;
    program[1] = 0x30;
    program[2] = 21; // MOV 0x30,#21
    program[3] = 0x75;
    program[4] = 0x31;
    program[5] = 0; // MOV 0x31,#0
    program[6] = 0x12;
    program[7] = 0;
    program[8] = 0x20; // LCALL the copied Delay_50us body
    program[9] = 0x80;
    program[10] = 0xfe; // SJMP $
    const std::array<std::uint8_t, 24> delay_body{
        0xe5, 0x30, 0x45, 0x31, 0x60, 0x11, 0x78, 0x38,
        0x00, 0x00, 0xd8, 0xfd, 0x15, 0x30, 0xe5, 0x30,
        0xb4, 0xff, 0x02, 0x15, 0x31, 0x80, 0xe9, 0x22};
    std::copy(delay_body.begin(), delay_body.end(), program.begin() + 0x20);
    write_binary(path, program.data(), program.size());

    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run = device.run(5000);
    CHECK(run.error.empty());
    CHECK(run.instructions == 5000);
    CHECK(run.pc == 0x0009);
    CHECK(device.read_idata(0x30) == 0);
    CHECK(device.read_idata(0x31) == 0);
    CHECK(device.read_idata(0x00) == 0);
    std::filesystem::remove(path);
}

void test_ca51_rejects_oversized_raw_image() {
    const auto path = temp_file("mcs51-oversized.bin");
    std::array<std::uint8_t, mcs51emu::Ca51F253L3::FlashSize + 1> image{};
    write_binary(path, image.data(), image.size());

    bool rejected = false;
    try {
        mcs51emu::Ca51F253L3 device;
        device.load_firmware(path.string());
    } catch (const std::exception &) {
        rejected = true;
    }
    CHECK(rejected);
    std::filesystem::remove(path);
}

// Two small programs that exercise the event-driven peripherals. TIMER reloads
// Timer0 in its ISR and echoes a counter on UART0; UART1 sends a counter byte
// over UART1 forever, waiting for TI1 between frames.
std::vector<std::uint8_t> timer_isr_program() {
    std::vector<std::uint8_t> image(0x300, 0xff);
    auto put = [&](std::size_t at, std::initializer_list<std::uint8_t> bytes) {
        std::copy(bytes.begin(), bytes.end(), image.begin() + at);
    };
    put(0x0000, {0x02, 0x01, 0x00});             // LJMP main
    put(0x000b, {0x02, 0x02, 0x00});             // timer0 vector: LJMP isr
    put(0x0100, {0x75, 0x81, 0x30,               // MOV SP,#0x30
                 0x75, 0x89, 0x01,               // MOV TMOD,#1
                 0x75, 0x8c, 0xff,               // MOV TH0,#0xff
                 0x75, 0x8a, 0x00,               // MOV TL0,#0
                 0xd2, 0x8c, 0xd2, 0xa9, 0xd2, 0xaf, // SETB TR0, ET0, EA
                 0x80, 0xfe});                   // SJMP $
    put(0x0200, {0x75, 0x8c, 0xfe, 0x75, 0x8a, 0x10, // reload TH0:TL0
                 0x05, 0x30,                     // INC 0x30
                 0x85, 0x30, 0x99,               // MOV S0BUF,0x30
                 0x32});                         // RETI
    return image;
}

std::vector<std::uint8_t> uart1_stream_program() {
    std::vector<std::uint8_t> image(0x200, 0xff);
    auto put = [&](std::size_t at, std::initializer_list<std::uint8_t> bytes) {
        std::copy(bytes.begin(), bytes.end(), image.begin() + at);
    };
    put(0x0000, {0x02, 0x01, 0x00});
    put(0x0100, {0x75, 0x81, 0x30, 0x75, 0x9c, 0x00, 0x75, 0x9d, 0x03, // SP, reload
                 0x05, 0x30,                     // loop: INC 0x30
                 0x85, 0x30, 0x9b,               // MOV S1BUF,0x30
                 0xe5, 0x9a, 0x54, 0x02, 0x60, 0xfa, // wait: poll TI1
                 0x75, 0x9a, 0x02,               // clear TI1 (write one)
                 0x85, 0x30, 0x99,               // MOV S0BUF,0x30
                 0x80, 0xed});                   // SJMP loop
    return image;
}

struct Snapshot {
    std::uint64_t instructions{};
    std::uint16_t pc{};
    std::uint64_t cycles{};
    std::string uart0, uart1;
    std::vector<std::uint8_t> idata, sfr;
};

Snapshot run_program(const std::vector<std::uint8_t> &image, bool traced,
                     std::uint64_t cycle_limit) {
    const auto path = temp_file("mcs51-equivalence.bin");
    write_binary(path, image.data(), image.size());
    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    std::ostringstream trace; // makes the device use its per-instruction hook
    if (traced)
        device.set_trace(&trace);
    const mcs51emu::RunResult run =
        device.run(std::numeric_limits<std::size_t>::max(), cycle_limit);
    CHECK(run.error.empty());
    Snapshot snapshot{run.instructions, run.pc, run.cycles, device.uart0_output(),
                      device.uart1_output(), {}, {}};
    for (unsigned a = 0; a < 0x100; ++a)
        snapshot.idata.push_back(device.read_idata(static_cast<std::uint8_t>(a)));
    for (unsigned a = 0x80; a < 0x100; ++a)
        snapshot.sfr.push_back(device.read_sfr(static_cast<std::uint8_t>(a)));
    std::filesystem::remove(path);
    return snapshot;
}

void check_fast_path_matches_hook_path(const std::vector<std::uint8_t> &image,
                                       std::uint64_t cycles) {
    const Snapshot fast = run_program(image, false, cycles);
    const Snapshot hooked = run_program(image, true, cycles);
    // The hooked run counts in chunks, so it may stop a few instructions past
    // the limit; compare the state both reach by running the fast one to the
    // same instruction count.
    CHECK(fast.cycles >= cycles);
    CHECK(hooked.cycles >= cycles);
    const auto path = temp_file("mcs51-equivalence2.bin");
    write_binary(path, image.data(), image.size());
    mcs51emu::Ca51F253L3 again;
    again.load_firmware(path.string());
    const mcs51emu::RunResult same_length =
        again.run(static_cast<std::size_t>(hooked.instructions));
    CHECK(same_length.instructions == hooked.instructions);
    CHECK(same_length.cycles == hooked.cycles);
    CHECK(same_length.pc == hooked.pc);
    CHECK(again.uart0_output() == hooked.uart0);
    CHECK(again.uart1_output() == hooked.uart1);
    for (unsigned a = 0; a < 0x100; ++a)
        CHECK(again.read_idata(static_cast<std::uint8_t>(a)) ==
              hooked.idata[a]);
    for (unsigned a = 0x80; a < 0x100; ++a)
        CHECK(again.read_sfr(static_cast<std::uint8_t>(a)) == hooked.sfr[a - 0x80]);
    std::filesystem::remove(path);
}

void test_ca51_fast_run_matches_per_instruction_hook() {
    check_fast_path_matches_hook_path(timer_isr_program(), 20000);
    check_fast_path_matches_hook_path(uart1_stream_program(), 300000);
}

void test_ca51_uart1_stream_sends_a_frame_per_baud_period() {
    // Reload 0x300: a frame is (0x400-0x300) * 32 * 10 = 81920 cycles.
    const Snapshot snapshot = run_program(uart1_stream_program(), false, 81920ull * 10 + 1000);
    CHECK(snapshot.uart1.size() == 10);
    for (std::size_t i = 0; i < snapshot.uart1.size(); ++i)
        CHECK(static_cast<std::uint8_t>(snapshot.uart1[i]) == i + 1);
}

void test_ca51_run_stops_at_the_cycle_limit() {
    const auto path = temp_file("mcs51-cycle-limit.bin");
    const std::array<std::uint8_t, 2> program{0x80, 0xfe}; // SJMP $: 2 cycles
    write_binary(path, program.data(), program.size());
    mcs51emu::Ca51F253L3 device;
    device.load_firmware(path.string());
    const mcs51emu::RunResult run =
        device.run(std::numeric_limits<std::size_t>::max(), 101);
    CHECK(run.error.empty());
    CHECK(run.cycles == 102); // never more than one instruction past the limit
    CHECK(run.instructions == 51);
    const mcs51emu::RunResult limited = device.run(10, 1000000);
    CHECK(limited.instructions == 10);
    CHECK(limited.cycles == 122);
    std::filesystem::remove(path);
}

} // namespace

int main() {
    test_ca51_raw_image_executes_code_and_xdata_access();
    test_ca51_intel_hex_image_loads();
    test_ca51_cjne_indirect_uses_immediate_and_relative_bytes();
    test_ca51_acall_ret_preserve_stack_and_return_pc();
    test_ca51_cross_page_nested_call_stack();
    test_ca51_compiler_save_restore_helper_abi();
    test_ca51_sdk_data_flash_read_write_and_erase();
    test_ca51_uart0_sbuf_emits_bytes_and_sets_ti0();
    test_ca51_uart1_transmit_interrupt_clears_flag();
    test_ca51_uart1_transmits_queued_byte_after_frame_time();
    test_ca51_pwm_update_request_self_clears();
    test_ca51_timer0_overflow_enters_isr_and_reti_returns();
    test_ca51_timer0_can_be_enabled_during_fast_run();
    test_ca51_llvm_delay_loop_finishes();
    test_ca51_rejects_oversized_raw_image();
    test_ca51_fast_run_matches_per_instruction_hook();
    test_ca51_uart1_stream_sends_a_frame_per_baud_period();
    test_ca51_run_stops_at_the_cycle_limit();
    if (failures != 0) {
        std::cerr << failures << " MCS-51 check(s) failed\n";
        return 1;
    }
    std::cout << "all MCS-51 emulator checks passed\n";
    return 0;
}

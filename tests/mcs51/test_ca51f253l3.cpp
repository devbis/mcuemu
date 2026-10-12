#include "mcs51emu/ca51f253l3.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

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
    test_ca51_pwm_update_request_self_clears();
    test_ca51_timer0_overflow_enters_isr_and_reti_returns();
    test_ca51_llvm_delay_loop_finishes();
    test_ca51_rejects_oversized_raw_image();
    if (failures != 0) {
        std::cerr << failures << " MCS-51 check(s) failed\n";
        return 1;
    }
    std::cout << "all MCS-51 emulator checks passed\n";
    return 0;
}

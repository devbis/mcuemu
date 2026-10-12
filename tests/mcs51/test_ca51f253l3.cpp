#include "mcs51emu/ca51f253l3.hpp"

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
    test_ca51_rejects_oversized_raw_image();
    if (failures != 0) {
        std::cerr << failures << " MCS-51 check(s) failed\n";
        return 1;
    }
    std::cout << "all MCS-51 emulator checks passed\n";
    return 0;
}

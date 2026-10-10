#include "../test_support.hpp"
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include "tc32emu/api.h"

namespace tc32emu_tests {

void test_c_api_register_access() {
    tc32emu_engine *engine = tc32emu_open("F1K");
    CHECK(engine != nullptr);
    if (!engine)
        return;
    CHECK(tc32emu_reg_write(engine, 4, 0x12345678u) == 0);
    std::uint32_t value = 0;
    CHECK(tc32emu_reg_read(engine, 4, &value) == 0);
    CHECK(value == 0x12345678u);

    const auto path = std::filesystem::temp_directory_path() /
                      "tc32emu-c-api-test.bin";
    const std::array<std::uint8_t, 4> image{0x05, 0xa0, 0x00, 0xcf};
    {
        std::ofstream output(path, std::ios::binary);
        output.write(reinterpret_cast<const char *>(image.data()), image.size());
    }
    CHECK(tc32emu_load_raw(engine, path.string().c_str(), 0) == 0);
    tc32emu_run_result result{};
    CHECK(tc32emu_run(engine, 8, &result) == 0);
    CHECK(result.reason == TC32EMU_HALTED);
    CHECK(result.instructions == 2);
    CHECK(tc32emu_reg_read(engine, 0, &value) == 0);
    CHECK(value == 5);
    std::filesystem::remove(path);
    tc32emu_close(engine);
}

} // namespace tc32emu_tests

#include "../test_support.hpp"
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include "mcuemu/api.h"

namespace mcuemu_tests {

void test_c_api_register_access() {
    mcuemu_engine *engine = mcuemu_open("F1K");
    CHECK(engine != nullptr);
    if (!engine)
        return;
    CHECK(mcuemu_reg_write(engine, 4, 0x12345678u) == 0);
    std::uint32_t value = 0;
    CHECK(mcuemu_reg_read(engine, 4, &value) == 0);
    CHECK(value == 0x12345678u);

    const auto path = std::filesystem::temp_directory_path() /
                      "mcuemu-c-api-test.bin";
    const std::array<std::uint8_t, 4> image{0x05, 0xa0, 0x00, 0xcf};
    {
        std::ofstream output(path, std::ios::binary);
        output.write(reinterpret_cast<const char *>(image.data()), image.size());
    }
    CHECK(mcuemu_load_raw(engine, path.string().c_str(), 0) == 0);
    mcuemu_run_result result{};
    CHECK(mcuemu_run(engine, 8, &result) == 0);
    CHECK(result.reason == MCUEMU_HALTED);
    CHECK(result.instructions == 2);
    CHECK(mcuemu_reg_read(engine, 0, &value) == 0);
    CHECK(value == 5);
    std::filesystem::remove(path);
    mcuemu_close(engine);
}

} // namespace mcuemu_tests

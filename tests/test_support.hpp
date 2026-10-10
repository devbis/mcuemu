#pragma once

#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <vector>

namespace tc32emu_tests {

inline int failures = 0;

inline std::vector<std::uint8_t> words(
    std::initializer_list<std::uint16_t> values) {
    std::vector<std::uint8_t> bytes;
    for (std::uint16_t value : values) {
        bytes.push_back(static_cast<std::uint8_t>(value));
        bytes.push_back(static_cast<std::uint8_t>(value >> 8));
    }
    return bytes;
}

} // namespace tc32emu_tests

#define CHECK(expr)                                                              \
    do {                                                                         \
        if (!(expr)) {                                                           \
            std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: "      \
                      << #expr << '\n';                                         \
            ++tc32emu_tests::failures;                                           \
        }                                                                        \
    } while (false)

#include "rom.hpp"

#include "esp32emu/esp32.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

namespace esp32emu {

namespace {
constexpr std::uint32_t ResetReasonPowerOn = 1;

// MD5 as the ROM implements it: struct MD5Context { u32 buf[4]; u32 bits[2]; u8 in[64]; }
// (esp_rom_md5.h), the digest of RFC 1321.
struct Md5Context {
    std::uint32_t buf[4];
    std::uint32_t bits[2];
    std::uint8_t in[64];
};

std::uint32_t rotl(std::uint32_t x, unsigned n) { return x << n | x >> (32 - n); }

void md5_block(std::uint32_t state[4], const std::uint8_t block[64]) {
    static const std::uint32_t k[64] = {
        0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
        0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
        0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
        0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
        0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
        0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
        0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
        0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};
    static const unsigned r[64] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                                   5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20,
                                   4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                                   6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
    std::uint32_t m[16];
    std::memcpy(m, block, 64);
    std::uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    for (unsigned i = 0; i < 64; ++i) {
        std::uint32_t f;
        unsigned g;
        if (i < 16) { f = (b & c) | (~b & d); g = i; }
        else if (i < 32) { f = (d & b) | (~d & c); g = (5 * i + 1) % 16; }
        else if (i < 48) { f = b ^ c ^ d; g = (3 * i + 5) % 16; }
        else { f = c ^ (b | ~d); g = (7 * i) % 16; }
        const std::uint32_t next = b + rotl(a + f + k[i] + m[g], r[i]);
        a = d; d = c; c = b; b = next;
    }
    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
}
std::array<std::uint8_t, 32> sha256(const std::vector<std::uint8_t> &message) {
    static const std::uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    std::uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::vector<std::uint8_t> data = message;
    const std::uint64_t bits = static_cast<std::uint64_t>(message.size()) * 8;
    data.push_back(0x80);
    while (data.size() % 64 != 56)
        data.push_back(0);
    for (int i = 7; i >= 0; --i)
        data.push_back(static_cast<std::uint8_t>(bits >> (8 * i)));
    auto ror = [](std::uint32_t x, unsigned n) { return x >> n | x << (32 - n); };
    for (std::size_t offset = 0; offset < data.size(); offset += 64) {
        std::uint32_t w[64];
        for (unsigned i = 0; i < 16; ++i)
            w[i] = data[offset + 4 * i] << 24 | data[offset + 4 * i + 1] << 16 | data[offset + 4 * i + 2] << 8 |
                   data[offset + 4 * i + 3];
        for (unsigned i = 16; i < 64; ++i) {
            const std::uint32_t s0 = ror(w[i - 15], 7) ^ ror(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const std::uint32_t s1 = ror(w[i - 2], 17) ^ ror(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
        for (unsigned i = 0; i < 64; ++i) {
            const std::uint32_t t1 = hh + (ror(e, 6) ^ ror(e, 11) ^ ror(e, 25)) + ((e & f) ^ (~e & g)) + k[i] + w[i];
            const std::uint32_t t2 = (ror(a, 2) ^ ror(a, 13) ^ ror(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
            hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }
    std::array<std::uint8_t, 32> out;
    for (unsigned i = 0; i < 8; ++i)
        for (unsigned j = 0; j < 4; ++j)
            out[4 * i + j] = static_cast<std::uint8_t>(h[i] >> (24 - 8 * j));
    return out;
}
} // namespace

Rom::Rom(Esp32 &chip) : chip_(chip) {
    for (const auto &symbol : chip_.layout().rom_symbols)
        names_.emplace(symbol.address, symbol.name); // first name of an alias group wins

    riscv::Core &core = chip_.core();
    auto reg = [&core](unsigned index) { return core.reg(index); };
    auto arg64 = [reg](unsigned low) {
        return static_cast<std::uint64_t>(reg(10 + low)) | static_cast<std::uint64_t>(reg(11 + low)) << 32;
    };

    // ---- memory and strings ----
    add("memset", [this, &core, reg] {
        const std::uint32_t dst = reg(10), len = reg(12);
        std::vector<std::uint8_t> fill(len, static_cast<std::uint8_t>(reg(11)));
        core.write_memory(dst, fill.data(), len);
        result_ = dst;
    });
    auto copy = [this, &core, reg] {
        const std::uint32_t dst = reg(10), src = reg(11), len = reg(12);
        std::vector<std::uint8_t> buffer(len);
        core.read_memory(src, buffer.data(), len);
        core.write_memory(dst, buffer.data(), len);
        result_ = dst;
    };
    add("memcpy", copy);
    add("memmove", copy);
    add("memcmp", [this, &core, reg] {
        const std::uint32_t a = reg(10), b = reg(11), len = reg(12);
        std::vector<std::uint8_t> x(len), y(len);
        core.read_memory(a, x.data(), len);
        core.read_memory(b, y.data(), len);
        const int order = len ? std::memcmp(x.data(), y.data(), len) : 0;
        result_ = static_cast<std::uint32_t>(order < 0 ? -1 : order > 0 ? 1 : 0);
    });

    // MD5 (the bootloader checks the partition table with it)
    add("MD5Init", [&core, reg] {
        Md5Context ctx{{0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476}, {0, 0}, {}};
        core.write_memory(reg(10), &ctx, sizeof ctx);
    });
    add("MD5Update", [&core, reg] {
        Md5Context ctx;
        core.read_memory(reg(10), &ctx, sizeof ctx);
        std::vector<std::uint8_t> data(reg(12));
        core.read_memory(reg(11), data.data(), data.size());
        for (std::uint8_t byte : data) {
            ctx.in[(ctx.bits[0] >> 3) & 63] = byte;
            ctx.bits[0] += 8;
            if (ctx.bits[0] < 8)
                ++ctx.bits[1];
            if (((ctx.bits[0] >> 3) & 63) == 0)
                md5_block(ctx.buf, ctx.in);
        }
        core.write_memory(reg(10), &ctx, sizeof ctx);
    });
    add("MD5Final", [&core, reg] {
        Md5Context ctx;
        core.read_memory(reg(11), &ctx, sizeof ctx);
        const std::uint64_t bits = static_cast<std::uint64_t>(ctx.bits[1]) << 32 | ctx.bits[0];
        std::uint8_t pad[72] = {0x80};
        const unsigned fill = ((ctx.bits[0] >> 3) & 63) < 56 ? 56 - ((ctx.bits[0] >> 3) & 63)
                                                              : 120 - ((ctx.bits[0] >> 3) & 63);
        for (unsigned i = 0; i < fill; ++i) {
            ctx.in[(ctx.bits[0] >> 3) & 63] = pad[i == 0 ? 0 : 1] ;
            ctx.bits[0] += 8;
            if (((ctx.bits[0] >> 3) & 63) == 0)
                md5_block(ctx.buf, ctx.in);
        }
        std::memcpy(ctx.in + 56, &bits, 8);
        md5_block(ctx.buf, ctx.in);
        core.write_memory(reg(10), ctx.buf, 16);
    });

    // SHA-256 (the bootloader hashes the app image). The firmware's context is opaque
    // to it, so the data is collected by context address and hashed at the end.
    add("ets_sha_enable", [] {});
    add("ets_sha_disable", [] {});
    add("ets_sha_init", [this, reg] { sha_data_[reg(10)].clear(); });
    add("ets_sha_starts", [this, reg] { sha_data_[reg(10)].clear(); });
    add("ets_sha_update", [this, &core, reg] {
        std::vector<std::uint8_t> chunk(reg(12));
        core.read_memory(reg(11), chunk.data(), chunk.size());
        auto &data = sha_data_[reg(10)];
        data.insert(data.end(), chunk.begin(), chunk.end());
    });
    add("ets_sha_finish", [this, &core, reg] {
        const auto digest = sha256(sha_data_[reg(10)]);
        core.write_memory(reg(11), digest.data(), digest.size());
        sha_data_.erase(reg(10));
    });

    // The ROM's flash routines (esp-storage and the OTA code use them): physical flash offsets,
    // NOR semantics (a write can only clear bits, an erase sets a sector or block to 0xff), instant.
    auto flash_read = [this, &core, reg] {
        auto &flash = chip_.flash();
        const std::uint32_t at = reg(10), len = reg(12);
        if (at + std::uint64_t{len} <= flash.size())
            core.write_memory(reg(11), flash.data() + at, len);
        result_ = 0;
    };
    auto flash_write = [this, &core, reg] {
        auto &flash = chip_.flash();
        const std::uint32_t at = reg(10), len = reg(12);
        if (at + std::uint64_t{len} <= flash.size()) {
            std::vector<std::uint8_t> data(len);
            core.read_memory(reg(11), data.data(), len);
            for (std::uint32_t i = 0; i < len; ++i)
                flash[at + i] &= data[i];
            chip_.flash_modified();
        }
        result_ = 0;
    };
    auto erase = [this, reg](std::uint32_t unit) {
        return [this, reg, unit] {
            auto &flash = chip_.flash();
            const std::uint64_t at = std::uint64_t{reg(10)} * unit;
            if (at + unit <= flash.size()) {
                std::memset(flash.data() + at, 0xff, unit);
                chip_.flash_modified();
            }
            result_ = 0;
        };
    };
    for (const char *name : {"esp_rom_spiflash_read", "_esp_rom_spiflash_read"})
        add(name, flash_read);
    for (const char *name : {"esp_rom_spiflash_write", "_esp_rom_spiflash_write"})
        add(name, flash_write);
    for (const char *name : {"esp_rom_spiflash_erase_sector", "_esp_rom_spiflash_erase_sector"})
        add(name, erase(0x1000));
    for (const char *name : {"esp_rom_spiflash_erase_block", "_esp_rom_spiflash_erase_block"})
        add(name, erase(0x10000));
    add("esp_rom_spiflash_unlock", [] {});
    add("ets_get_printf_channel", [] {});

    // CRC32 (reflected, polynomial 0xedb88320) the way the ROM's crc32_le takes it: the running
    // value is passed in and out in its final (inverted) form.
    add("crc32_le", [this, &core, reg] {
        std::vector<std::uint8_t> data(reg(12));
        core.read_memory(reg(11), data.data(), data.size());
        std::uint32_t crc = ~reg(10);
        for (std::uint8_t byte : data) {
            crc ^= byte;
            for (int bit = 0; bit < 8; ++bit)
                crc = crc >> 1 ^ (crc & 1 ? 0xedb88320u : 0);
        }
        result_ = ~crc;
    });

    // C strings: the ROM's newlib subset
    auto cstr = [&core](std::uint32_t address, std::size_t limit = ~std::size_t{0}) {
        std::string text;
        for (char c; text.size() < limit && core.read_memory(address + text.size(), &c, 1) && c != 0;)
            text.push_back(c);
        return text;
    };
    add("strlen", [this, cstr, reg] { result_ = cstr(reg(10)).size(); });
    add("strnlen", [this, cstr, reg] { result_ = cstr(reg(10), reg(11)).size(); });
    add("strcpy", [this, cstr, &core, reg] {
        const std::string text = cstr(reg(11)) + '\0';
        core.write_memory(reg(10), text.data(), text.size());
        result_ = reg(10);
    });
    add("strncpy", [this, cstr, &core, reg] {
        std::string text = cstr(reg(11), reg(12));
        text.resize(reg(12), '\0');
        core.write_memory(reg(10), text.data(), text.size());
        result_ = reg(10);
    });
    add("strcat", [this, cstr, &core, reg] {
        const std::string text = cstr(reg(11)) + '\0';
        core.write_memory(reg(10) + static_cast<std::uint32_t>(cstr(reg(10)).size()), text.data(), text.size());
        result_ = reg(10);
    });
    add("strcmp", [this, cstr, reg] {
        const int order = cstr(reg(10)).compare(cstr(reg(11)));
        result_ = static_cast<std::uint32_t>(order < 0 ? -1 : order > 0 ? 1 : 0);
    });
    add("strncmp", [this, cstr, reg] {
        const int order = cstr(reg(10), reg(12)).compare(cstr(reg(11), reg(12)));
        result_ = static_cast<std::uint32_t>(order < 0 ? -1 : order > 0 ? 1 : 0);
    });
    add("strchr", [this, cstr, reg] {
        const std::string text = cstr(reg(10));
        const char wanted = static_cast<char>(reg(11));
        const auto at = wanted == 0 ? text.size() : text.find(wanted);
        result_ = at == std::string::npos ? 0 : reg(10) + static_cast<std::uint32_t>(at);
    });

    // ---- libgcc 64-bit arithmetic (RV32 has no 64-bit mul/div) ----
    add("__muldi3", [this, arg64] { result_ = arg64(0) * arg64(2); });
    add("__udivdi3", [this, arg64] { result_ = arg64(2) ? arg64(0) / arg64(2) : ~0ull; });
    add("__umoddi3", [this, arg64] { result_ = arg64(2) ? arg64(0) % arg64(2) : arg64(0); });
    add("__divdi3", [this, arg64] {
        const auto a = static_cast<std::int64_t>(arg64(0)), b = static_cast<std::int64_t>(arg64(2));
        result_ = static_cast<std::uint64_t>(b ? a / b : -1);
    });
    add("__moddi3", [this, arg64] {
        const auto a = static_cast<std::int64_t>(arg64(0)), b = static_cast<std::int64_t>(arg64(2));
        result_ = static_cast<std::uint64_t>(b ? a % b : a);
    });

    // 32-bit builtins (clz of 0 is undefined in libgcc; 32/64 like the ROM's table)
    add("__clzsi2", [this, reg] { result_ = reg(10) ? __builtin_clz(reg(10)) : 32; });
    add("__ctzsi2", [this, reg] { result_ = reg(10) ? __builtin_ctz(reg(10)) : 32; });
    add("__ffssi2", [this, reg] { result_ = reg(10) ? __builtin_ctz(reg(10)) + 1 : 0; });
    add("__popcountsi2", [this, reg] { result_ = __builtin_popcount(reg(10)); });
    add("__paritysi2", [this, reg] { result_ = __builtin_parity(reg(10)); });
    add("__bswapsi2", [this, reg] { result_ = __builtin_bswap32(reg(10)); });
    add("__clzdi2", [this, arg64] { result_ = arg64(0) ? __builtin_clzll(arg64(0)) : 64; });
    add("__ctzdi2", [this, arg64] { result_ = arg64(0) ? __builtin_ctzll(arg64(0)) : 64; });
    add("__ffsdi2", [this, arg64] { result_ = arg64(0) ? __builtin_ctzll(arg64(0)) + 1 : 0; });
    add("__popcountdi2", [this, arg64] { result_ = __builtin_popcountll(arg64(0)); });
    add("__bswapdi2", [this, arg64] { result_ = __builtin_bswap64(arg64(0)); });
    add("__ashldi3", [this, arg64, reg] { result_ = arg64(0) << (reg(12) & 63); });
    add("__lshrdi3", [this, arg64, reg] { result_ = arg64(0) >> (reg(12) & 63); });
    add("__ashrdi3", [this, arg64, reg] {
        result_ = static_cast<std::uint64_t>(static_cast<std::int64_t>(arg64(0)) >> (reg(12) & 63));
    });
    add("__negdi2", [this, arg64] { result_ = 0 - arg64(0); });
    add("__mulsi3", [this, reg] { result_ = static_cast<std::uint32_t>(reg(10) * reg(11)); });
    add("__udivsi3", [this, reg] { result_ = reg(11) ? reg(10) / reg(11) : ~0u; });
    add("__umodsi3", [this, reg] { result_ = reg(11) ? reg(10) % reg(11) : reg(10); });
    add("__divsi3", [this, reg] {
        const auto a = static_cast<std::int32_t>(reg(10)), b = static_cast<std::int32_t>(reg(11));
        result_ = static_cast<std::uint32_t>(b ? a / b : -1);
    });
    add("__modsi3", [this, reg] {
        const auto a = static_cast<std::int32_t>(reg(10)), b = static_cast<std::int32_t>(reg(11));
        result_ = static_cast<std::uint32_t>(b ? a % b : a);
    });

    // ---- console ----
    add("ets_printf", [this, reg] {
        for (char c : format_printf(reg(10), 1))
            chip_.uart_tx(static_cast<std::uint8_t>(c));
    });
    auto putc = [this, reg] { chip_.uart_tx(static_cast<std::uint8_t>(reg(10))); };
    add("uart_tx_one_char", putc);
    add("uart_tx_one_char2", putc);
    add("ets_write_char_uart", putc);
    for (const char *quiet : {"uart_tx_flush", "uart_tx_wait_idle", "uart_tx_switch",
                              "ets_install_uart_printf", "ets_install_putc1", "ets_install_putc2",
                              "ets_update_cpu_frequency"})
        add(quiet, [] {});

    // ---- interrupt controller programming, as the ROM does it through the PLIC and
    // the interrupt matrix registers ----
    constexpr std::uint32_t Plic = 0x20001000, Matrix = 0x60010000;
    add("esprv_intc_int_enable", [this, reg] {
        chip_.write(Plic + 0x0, chip_.read(Plic + 0x0, 4) | reg(10), 4);
    });
    add("esprv_intc_int_disable", [this, reg] {
        chip_.write(Plic + 0x0, chip_.read(Plic + 0x0, 4) & ~reg(10), 4);
    });
    add("esprv_intc_int_set_type", [this, reg] {
        const std::uint32_t bit = 1u << reg(10);
        const std::uint32_t type = chip_.read(Plic + 0x4, 4);
        chip_.write(Plic + 0x4, reg(11) ? type | bit : type & ~bit, 4); // 1 = edge
    });
    add("esprv_intc_int_set_priority", [this, reg] { chip_.write(Plic + 0x10 + 4 * reg(10), reg(11), 4); });
    add("esprv_intc_int_set_threshold", [this, reg] { chip_.write(Plic + 0x90, reg(10), 4); });
    add("intr_matrix_set", [this, reg] { chip_.write(Matrix + 4 * reg(11), reg(12), 4); }); // (cpu, source, line)

    // ---- PHY: the RF front end is not modelled. The blob asks the ROM for a table
    // of its ROM routines and for the PHY parameter area; both live in the top of
    // the SRAM (reset() reserves it) and every table entry points at a ROM slot
    // that returns 0.
    add("phy_get_romfuncs", [this] { result_ = chip_.phy_rom_funcs(); });
    // Entry k of that table points at slot k of a scratch range in the ROM, so a
    // call through it is logged as phy_romfunc[k] (and returns 0) instead of
    // all of them looking alike. The table's layout is not public.
    for (unsigned k = 0; k < 64; ++k)
        add_address(chip_.phy_rom_slots() + 4 * k, "phy_romfunc[" + std::to_string(k) + "]", [] {});

    // The analog I2C master the PHY programs its PLL and calibration engines
    // through: (block, host, register) -> 8 bits, modelled as plain storage.
    // Slots 20, 23 and 24 of the table are read, masked read and masked write
    // (from the argument lists in libphy's rfpll_chgp_cal); slot 21 is taken to
    // be the plain write. The ROM entries of the same names share the storage.
    auto analog_read = [this, reg] {
        result_ = analog_[reg(10) << 8 | (reg(12) & 0xff)] | analog_forced(reg(10), reg(12));
    };
    auto analog_read_mask = [this, reg] {
        const unsigned msb = reg(13), lsb = reg(14);
        const std::uint32_t value = analog_[reg(10) << 8 | (reg(12) & 0xff)] | analog_forced(reg(10), reg(12));
        result_ = (value >> lsb) & ((1u << (msb - lsb + 1)) - 1);
    };
    auto analog_write = [this, reg] { analog_[reg(10) << 8 | (reg(12) & 0xff)] = reg(13) & 0xff; };
    auto analog_write_mask = [this, reg] {
        const unsigned msb = reg(13), lsb = reg(14);
        const std::uint32_t mask = ((1u << (msb - lsb + 1)) - 1) << lsb;
        std::uint32_t &value = analog_[reg(10) << 8 | (reg(12) & 0xff)];
        value = (value & ~mask) | ((reg(15) << lsb) & mask);
    };
    add_address(chip_.phy_rom_slots() + 4 * 20, "phy_romfunc[20]=rom_i2c_readReg", analog_read);
    add_address(chip_.phy_rom_slots() + 4 * 21, "phy_romfunc[21]=rom_i2c_writeReg", analog_write);
    add_address(chip_.phy_rom_slots() + 4 * 23, "phy_romfunc[23]=rom_i2c_readReg_Mask", analog_read_mask);
    add_address(chip_.phy_rom_slots() + 4 * 24, "phy_romfunc[24]=rom_i2c_writeReg_Mask", analog_write_mask);
    add("rom_i2c_readReg", analog_read);
    add("rom_i2c_writeReg", analog_write);
    add("rom_i2c_readReg_Mask", analog_read_mask);
    add("rom_i2c_writeReg_Mask", analog_write_mask);
    // The tone detector's SAR reading (txdc_cal_pwdet and friends scan a DC offset until the reading
    // turns upwards). The model has no analog front end, so it returns a rising ramp: the scans see their
    // minimum at once and stop after a few steps instead of running every iteration (50 per phase, with
    // 10 us delays), which keeps the PHY init short. The values mean nothing.
    add("get_tone_sar_dout", [this] { result_ = (sar_ramp_++ * 8u) & 0xffu; });
    add("phy_param_addr", [this] { result_ = chip_.phy_params(); });

    // ---- resets: the run stops with StopReason::Reset and the caller restarts ----
    auto restart = [this] {
        chip_.log("ROM: software reset requested");
        chip_.core().request_reset();
    };
    add("software_reset", restart);
    add("software_reset_cpu", restart);

    // ---- time and chip state ----
    add("ets_delay_us", [this, &core, reg] {
        core.advance_to(core.cycles() +
                        static_cast<std::uint64_t>(reg(10)) * (chip_.clock_hz() / 1000000));
    });
    add("ets_get_cpu_frequency", [this] { result_ = chip_.clock_hz() / 1000000; });
    add("rtc_get_reset_reason", [this] { result_ = ResetReasonPowerOn; });
    add("rtc_get_wakeup_cause", [] {});

    // ---- flash cache and MMU: the cache is not modelled beyond the MMU table ----
    for (const auto &symbol : chip_.layout().rom_symbols)
        if (std::strncmp(symbol.name, "Cache_", 6) == 0 &&
            handlers_.find(symbol.address) == handlers_.end())
            add(symbol.name, [] {});
    for (const char *quiet : {"esp_rom_spiflash_config_clk", "spi_dummy_len_fix",
                              "esp_rom_spiflash_wait_idle", "ets_efuse_cache_encryption_enabled"})
        add(quiet, [] {});
}

void Rom::add_address(std::uint32_t address, std::string name, Handler handler) {
    names_[address] = std::move(name);
    handlers_[address] = std::move(handler);
}

void Rom::add(const char *name, Handler handler) {
    for (const auto &symbol : chip_.layout().rom_symbols)
        if (std::strcmp(symbol.name, name) == 0) {
            handlers_[symbol.address] = std::move(handler);
            return;
        }
    if (chip_.trace_rom)
        chip_.log(std::string("rom: no symbol named ") + name);
}

// Analog registers whose bits the hardware sets by itself, read by the PHY as
// "done": the PLL charge pump calibration (block 0x62, register 0xe, bit 7).
std::uint32_t Rom::analog_forced(std::uint32_t block, std::uint32_t reg) const {
    if (block == 0x62 && (reg & 0xff) == 0xe)
        return 0x80u;
    // wait_i2c_sdm_stable (libphy) reads block 0x63, register 0 until it returns 0x5b, giving up after 10 ms.
    if (block == 0x63 && (reg & 0xff) == 0x0)
        return 0x5bu;
    return 0u;
}

const char *Rom::name_at(std::uint32_t address) const {
    auto it = names_.find(address);
    return it == names_.end() ? nullptr : it->second.c_str();
}

bool Rom::call(std::uint32_t pc) {
    riscv::Core &core = chip_.core();
    result_ = 0;
    auto it = handlers_.find(pc);
    if (chip_.trace_rom && chip_.trace_limit != 0 && chip_.trace_limit-- != 0) {
        char buffer[160];
        const char *name = name_at(pc);
        std::snprintf(buffer, sizeof buffer, "[rom] %s a0=0x%x a1=0x%x a2=0x%x ra=0x%x t=%llu",
                      name ? name : "?", core.reg(10), core.reg(11), core.reg(12), core.reg(1),
                      static_cast<unsigned long long>(chip_.time_ns()));
        chip_.log(buffer);
    }
    if (it != handlers_.end()) {
        it->second();
    } else if (++unhandled_[pc] <= 5) { // a polling loop would flood the log
        char buffer[112];
        const char *name = name_at(pc);
        std::snprintf(buffer, sizeof buffer, "ROM call to %s (0x%08x, ra 0x%08x), returning 0%s",
                      name ? name : "?", pc, core.reg(1), unhandled_[pc] == 5 ? " (not logged again)" : "");
        chip_.log(buffer);
    }
    core.set_reg(10, static_cast<std::uint32_t>(result_));
    core.set_reg(11, static_cast<std::uint32_t>(result_ >> 32));
    core.set_reg(32, core.reg(1)); // return to ra
    return true;
}

std::string Rom::format_printf(std::uint32_t format, unsigned first_arg) {
    riscv::Core &core = chip_.core();
    auto cstring = [&](std::uint32_t address) {
        std::string text;
        for (char c; core.read_memory(address, &c, 1) && c != 0; ++address)
            text.push_back(c);
        return text;
    };
    std::string out;
    const std::string fmt = cstring(format);
    unsigned arg = first_arg; // arguments are in a0..a7, then on the stack from sp
    auto next = [&]() -> std::uint32_t {
        if (arg < 8)
            return core.reg(10 + arg++);
        std::uint32_t value = 0;
        core.read_memory(core.reg(2) + 4 * (arg++ - 8), &value, 4);
        return value;
    };
    for (std::size_t i = 0; i < fmt.size(); ++i) {
        if (fmt[i] != '%') {
            out.push_back(fmt[i]);
            continue;
        }
        std::string spec = "%";
        while (++i < fmt.size() && std::strchr("-+ #0123456789.l", fmt[i]))
            if (fmt[i] != 'l')
                spec.push_back(fmt[i]);
        if (i >= fmt.size())
            break;
        char buffer[64];
        switch (fmt[i]) {
        case 'd': case 'i':
            std::snprintf(buffer, sizeof buffer, (spec + "d").c_str(), static_cast<int>(next()));
            out += buffer;
            break;
        case 'u': case 'x': case 'X': case 'o':
            std::snprintf(buffer, sizeof buffer, (spec + fmt[i]).c_str(), next());
            out += buffer;
            break;
        case 'p':
            std::snprintf(buffer, sizeof buffer, "0x%x", next());
            out += buffer;
            break;
        case 'c':
            out.push_back(static_cast<char>(next()));
            break;
        case 's':
            out += cstring(next());
            break;
        case '%':
            out.push_back('%');
            break;
        default:
            out += spec + fmt[i];
        }
    }
    return out;
}

} // namespace esp32emu

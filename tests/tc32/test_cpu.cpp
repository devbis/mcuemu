#include "../test_support.hpp"
#include <cstdint>
#include <iostream>
#include "tc32emu/cpu.hpp"
#include "tc32emu/tlsr8258.hpp"

namespace tc32emu_tests {

void test_cpu_arithmetic_and_branch() {
    tc32emu::Tlsr8258 device;
    device.load_firmware(words({
        0xa005, // tmov r0, #5
        0xb002, // tadd r0, #2
        0xa807, // tcmp r0, #7
        0xc000, // tj eq, skip the next halfword
        0xa2ee, // tmov r2, #0xee (skipped)
        0xa22a, // tmov r2, #0x2a
        0xaa2a, // tcmp r2, #0x2a
        0xcf01, // tserv #1
    }));
    tc32emu::Cpu cpu(device);
    const auto result = cpu.run(32);
    if (result.reason != tc32emu::StopReason::Halted)
        std::cerr << "arith run: reason=" << static_cast<int>(result.reason)
                  << " pc=" << result.pc << " count=" << result.instructions
                  << " message=" << result.message << '\n';
    CHECK(result.reason == tc32emu::StopReason::Halted);
    CHECK(cpu.reg(0) == 7);
    CHECK(cpu.reg(2) == 0x2a);
    CHECK((cpu.cpsr() & tc32emu::Cpu::FlagZ) != 0);
    CHECK(result.instructions == 7);
}

void test_cpu_load_store() {
    tc32emu::Tlsr8258 device;
    // str r0, [r1, r2]; ldr r3, [r1, r2]; tserv #0
    device.load_firmware(words({0x1000u | (2u << 6) | (1u << 3),
                                0x1800u | (2u << 6) | (1u << 3) | 3u,
                                0xcf00}));
    tc32emu::Cpu cpu(device);
    cpu.set_reg(0, 0x12345678u);
    cpu.set_reg(1, tc32emu::Tlsr8258::RetainedSramBase + 0x100u);
    cpu.set_reg(2, 0);
    const auto result = cpu.run(8);
    if (result.reason != tc32emu::StopReason::Halted)
        std::cerr << "memory run: reason=" << static_cast<int>(result.reason)
                  << " pc=" << result.pc << " count=" << result.instructions
                  << " r3=" << std::hex << cpu.reg(3) << std::dec
                  << " message=" << result.message << '\n';
    CHECK(result.reason == tc32emu::StopReason::Halted);
    CHECK(cpu.reg(3) == 0x12345678u);
    CHECK(device.read32(tc32emu::Tlsr8258::RetainedSramBase + 0x100u) ==
          0x12345678u);
}

void test_cpu_vendor_mmio_alias() {
    tc32emu::Tlsr8258 device;
    device.load_firmware(words({
        0x4008, // strb r0, [r1, #0]
        0xcf00, // tserv #0
    }));
    tc32emu::Cpu cpu(device);
    cpu.set_reg(0, 0x5a);
    cpu.set_reg(1, tc32emu::Tlsr8258::VendorRegisterAliasBase + 0x12d2);
    const auto result = cpu.run(4);
    CHECK(result.reason == tc32emu::StopReason::Halted);
    CHECK(device.read8(tc32emu::Tlsr8258::RegisterBase + 0x12d2) == 0x5a);
    CHECK(device.read8(tc32emu::Tlsr8258::VendorRegisterAliasBase + 0x12d2) ==
          0x5a);
}

void test_cpu_asr_by_32() {
    tc32emu::Tlsr8258 device;
    device.load_firmware(words({0xe001, 0xcf00})); // tasr r1, r0, #32
    tc32emu::Cpu cpu(device);
    cpu.set_reg(0, 0x80000000u);
    const auto result = cpu.run(4);
    CHECK(result.reason == tc32emu::StopReason::Halted);
    CHECK(cpu.reg(1) == 0xffffffffu);
    CHECK((cpu.cpsr() & tc32emu::Cpu::FlagC) != 0);
}

void test_cpu_low_register_move_and_three_operand_immediates() {
    tc32emu::Tlsr8258 device;
    device.load_firmware(words({
        0xec01, // tmov r1, r0 (imm3-zero add alias)
        0xed42, // tadd r2, r0, #5
        0xeec3, // tsub r3, r0, #3
        0xcf00,
    }));
    tc32emu::Cpu cpu(device);
    cpu.set_reg(0, 10);
    cpu.set_cpsr(cpu.cpsr() | tc32emu::Cpu::FlagC | tc32emu::Cpu::FlagV);
    CHECK(cpu.step());
    CHECK((cpu.cpsr() & (tc32emu::Cpu::FlagC | tc32emu::Cpu::FlagV)) == 0);
    const auto result = cpu.run(8);
    CHECK(result.reason == tc32emu::StopReason::Halted);
    CHECK(cpu.reg(1) == 10);
    CHECK(cpu.reg(2) == 15);
    CHECK(cpu.reg(3) == 7);
}

void test_cpu_long_branches_and_call() {
    tc32emu::Tlsr8258 branch_device;
    branch_device.load_firmware(words({
        0xa000, // tmov r0, #0, setting Z
        0x9000, 0x2001, // long tj eq to address 0x08
        0xa101, // skipped
        0xa102, // tmov r1, #2
        0xcf00,
    }));
    tc32emu::Cpu branch_cpu(branch_device);
    const auto branch_result = branch_cpu.run(16);
    CHECK(branch_result.reason == tc32emu::StopReason::Halted);
    CHECK(branch_cpu.reg(1) == 2);
    CHECK(branch_result.instructions == 4);

    tc32emu::Tlsr8258 call_device;
    call_device.load_firmware(words({
        0x9000, 0x9802, // tjl to address 0x08
        0xcf00,         // return address
        0x0000,
        0xa109,         // function: tmov r1, #9
        0x0770,         // tjex lr
    }));
    tc32emu::Cpu call_cpu(call_device);
    const auto call_result = call_cpu.run(16);
    CHECK(call_result.reason == tc32emu::StopReason::Halted);
    CHECK(call_cpu.reg(1) == 9);
    CHECK(call_cpu.reg(14) == 4);
    CHECK(call_result.instructions == 4);
}

void test_cpu_status_modes_and_exception_return() {
    tc32emu::Tlsr8258 device;
    device.load_firmware(words({
        0x6bd0, // tmssr r0
        0x6bc1, // tmcsr r1
        0x6bda, // tmrss r2
        0x6900, // treti
    }));
    tc32emu::Cpu cpu(device);
    cpu.set_reg(13, 0x840100);
    cpu.set_reg(14, 0x1234);
    cpu.set_reg(0, 0x13); // saved status to restore on treti
    cpu.set_reg(1, 0x12); // IRQ mode
    CHECK(cpu.step());
    CHECK(cpu.step());
    CHECK((cpu.cpsr() & 0x1fu) == 0x12u);
    CHECK(cpu.reg(13) == 0);

    cpu.set_reg(13, 0x840200);
    CHECK(cpu.step());
    CHECK(cpu.reg(2) == 0x13);
    cpu.set_spsr(0x13); // exception return restores the IRQ bank's SPSR
    constexpr std::uint32_t irq_return_pc = 0x24;
    constexpr std::uint32_t irq_stack = 0x840200;
    for (unsigned byte = 0; byte != 4; ++byte) {
        device.write8(irq_stack + byte,
                      static_cast<std::uint8_t>(irq_return_pc >> (8u * byte)));
    }
    cpu.set_reg(13, irq_stack); // treti {pc} consumes the saved IRQ return PC
    cpu.set_reg(14, 0xdeadbeef); // IRQ LR has been clobbered by nested calls
    CHECK(cpu.step());
    CHECK((cpu.cpsr() & 0x1fu) == 0x13u);
    CHECK(cpu.reg(13) == 0x840100);
    CHECK(cpu.reg(14) == 0x1234);
    CHECK(cpu.pc() == irq_return_pc);
}

} // namespace tc32emu_tests

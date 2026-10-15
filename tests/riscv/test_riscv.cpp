// The fork's RISC-V backend as the ESP32-C6/H2 need it: synchronous traps go to
// the guest's own handler and a host-side controller raises interrupts with
// uc_riscv_irq(). The programs are in programs/*.s.

#include "../test_support.hpp"
#include "programs.hpp"

#include <unicorn/unicorn.h>

namespace mcuemu_tests {

namespace {

constexpr std::uint64_t Flash = 0x42000000;

// The programs acknowledge an interrupt by storing to 0x60000008, as a
// peripheral clear would; the window drops the level like the controller does.
void ack_write(uc_engine *uc, std::uint64_t offset, unsigned, std::uint64_t, void *) {
    if (offset == 8)
        uc_riscv_irq(uc, -1);
}
std::uint64_t ack_read(uc_engine *, std::uint64_t, unsigned, void *) { return 0; }

struct Hart {
    uc_engine *uc{};
    std::uint32_t pc{static_cast<std::uint32_t>(Flash)};

    explicit Hart(const std::vector<std::uint8_t> &program) {
        CHECK(uc_open(UC_ARCH_RISCV, UC_MODE_RISCV32, &uc) == UC_ERR_OK);
        CHECK(uc_mem_map(uc, Flash, 0x10000, UC_PROT_ALL) == UC_ERR_OK);
        CHECK(uc_mem_write(uc, Flash, program.data(), program.size()) == UC_ERR_OK);
        CHECK(uc_mmio_map(uc, 0x60000000, 0x1000, ack_read, nullptr, ack_write, nullptr) ==
              UC_ERR_OK);
        CHECK(uc_reg_write(uc, UC_RISCV_REG_PC, &pc) == UC_ERR_OK);
    }
    ~Hart() { uc_close(uc); }

    // Runs until the hart sleeps in wfi (UC_ERR_OK, pc after the wfi) or hits
    // ebreak (UC_ERR_INSN_INVALID, pc at the ebreak).
    uc_err run() {
        uc_err err = uc_emu_start(uc, pc, Flash + 0xffff, 0, 100000);
        uc_reg_read(uc, UC_RISCV_REG_PC, &pc);
        return err;
    }
    std::uint32_t reg(int id) {
        std::uint32_t value = 0;
        uc_reg_read(uc, id, &value);
        return value;
    }
};

} // namespace

void test_riscv_ecall_reaches_the_guest_trap_handler() {
    Hart hart(riscv_programs::trap_irq);
    CHECK(hart.run() == UC_ERR_OK); // slept in wfi after the ecall came back
    CHECK(hart.reg(UC_RISCV_REG_S0) == 1);
    CHECK(hart.reg(UC_RISCV_REG_MCAUSE) == 0xb); // ecall from M-mode
    CHECK(hart.reg(UC_RISCV_REG_S1) == 0);
}

void test_riscv_host_interrupt_wakes_wfi_with_its_own_cause() {
    Hart hart(riscv_programs::trap_irq);
    CHECK(hart.run() == UC_ERR_OK);
    std::uint32_t wfi_next = hart.pc;
    // 20 has no bit in mie/mip: only the host-side controller can raise it.
    CHECK(uc_riscv_irq(hart.uc, 20) == UC_ERR_OK);
    CHECK(hart.run() == UC_ERR_INSN_INVALID); // handler returned, program hit ebreak
    CHECK(hart.reg(UC_RISCV_REG_S1) == 0x80000014);
    CHECK(hart.reg(UC_RISCV_REG_MEPC) == wfi_next); // wake continues after the wfi
}

void test_riscv_interrupt_waits_for_mstatus_mie() {
    Hart hart(riscv_programs::trap_irq);
    CHECK(uc_riscv_irq(hart.uc, 7) == UC_ERR_OK); // raised before the program sets MIE
    CHECK(hart.run() == UC_ERR_OK);
    CHECK(hart.reg(UC_RISCV_REG_S1) == 0x80000007); // taken at the first instruction after csrsi
    CHECK(hart.reg(UC_RISCV_REG_S0) == 1);          // after the ecall, not instead of it
}

void test_riscv_vectored_mtvec_enters_at_base_plus_4n() {
    Hart hart(riscv_programs::vectored);
    CHECK(hart.run() == UC_ERR_OK);
    CHECK(uc_riscv_irq(hart.uc, 19) == UC_ERR_OK);
    CHECK(hart.run() == UC_ERR_INSN_INVALID);
    CHECK(hart.reg(UC_RISCV_REG_S2) == 19);
}

} // namespace mcuemu_tests

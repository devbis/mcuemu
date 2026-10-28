// CC2530 Timer 2 (MAC timer) tests. Programs are hand-assembled MCS-51
// machine code; expected values come from SWRU191F Chapter 22.
#include "cc2530_periph_support.hpp"

#include "cc2530_mactimer.hpp"
#include "cc2530_timers.hpp"

#include <optional>

using namespace cc2530test;
using mcs51emu::cc2530::lf_tick_cycle;
using mcs51emu::cc2530::lf_ticks;

namespace {

constexpr std::uint8_t T2CTRL = 0x94, T2EVTCFG = 0x9C, T2IRQF = 0xA1, T2M0 = 0xA2, T2M1 = 0xA3,
                       T2MOVF0 = 0xA4, T2MOVF1 = 0xA5, T2MOVF2 = 0xA6, T2IRQM = 0xA7,
                       T2MSEL = 0xC3, IRCON = 0xC0, IEN0 = 0xA8, SLEEPCMD = 0xBE, PCON = 0x87;
constexpr std::uint8_t BitEa = 0xAF, BitT2ie = 0xBA, BitStie = 0xAD;
constexpr std::uint16_t Main = 0x0100, T2Isr = 0x00A0;

std::vector<std::uint8_t> mov(std::uint8_t dir, std::uint8_t imm) { return mov_dir_imm(dir, imm); }
// MOV dst,src
std::vector<std::uint8_t> mov_dd(std::uint8_t dst, std::uint8_t src) { return {0x85, src, dst}; }
// MOV R7,#n; DJNZ R7,$
std::vector<std::uint8_t> delay(std::uint8_t n) { return {0x7F, n, 0xDF, 0xFE}; }
// Writes a 16-bit internal register selected by T2MSEL.T2MSEL (22.5).
std::vector<std::uint8_t> set16(unsigned sel, std::uint16_t v) {
    std::vector<std::uint8_t> code = mov(T2MSEL, static_cast<std::uint8_t>(sel));
    for (auto b : mov(T2M0, lo(v)))
        code.push_back(b);
    for (auto b : mov(T2M1, hi(v)))
        code.push_back(b);
    return code;
}
// Writes a 24-bit internal register selected by T2MSEL.T2MOVFSEL (22.1.7).
std::vector<std::uint8_t> set24(unsigned sel, std::uint32_t v) {
    std::vector<std::uint8_t> code = mov(T2MSEL, static_cast<std::uint8_t>(sel << 4));
    for (const auto &part : {mov(T2MOVF0, static_cast<std::uint8_t>(v)),
                             mov(T2MOVF1, static_cast<std::uint8_t>(v >> 8)),
                             mov(T2MOVF2, static_cast<std::uint8_t>(v >> 16))})
        for (auto b : part)
            code.push_back(b);
    return code;
}

// Reset vector -> Main; T2 vector (0x53) -> ISR counting entries in 0x30,
// storing T2IRQF in 0x31 and clearing every flag.
Flash program() {
    Flash flash;
    flash.at(0x0000).op({0x02, hi(Main), lo(Main)});
    flash.at(0x0053).op({0x02, hi(T2Isr), lo(T2Isr)});
    flash.at(T2Isr).op({0x05, 0x30}).op(mov_dd(0x31, T2IRQF)).op(mov(T2IRQF, 0x00)).op({0x32});
    flash.at(Main);
    return flash;
}
// One-cycle NOPs up to 0x7FF0, then LJMP back: every cycle is an
// instruction boundary, so events are serviced at their exact time.
void sled(Flash &flash) {
    const std::uint16_t start = flash.here();
    while (flash.here() < 0x7FF0)
        flash.op({0x00});
    flash.op({0x02, hi(start), lo(start)});
}

std::uint64_t run_to_pc(Cc2530F256 &device, std::uint16_t pc) {
    device.set_breakpoint(pc);
    const Cc2530RunResult r = device.run(10'000'000);
    device.set_breakpoint(std::nullopt);
    CHECK(r.error.empty());
    CHECK(r.break_hit);
    return device.registers().cycles;
}
void run_to(Cc2530F256 &device, std::uint64_t cycle) {
    const Cc2530RunResult r = device.run(10'000'000, cycle);
    CHECK(r.error.empty());
    CHECK_EQ(device.registers().cycles, cycle);
}

void test_reset_values_and_masks() {
    Flash flash = program();
    std::uint8_t dst = 0x40;
    for (const std::uint8_t sfr :
         {T2CTRL, T2EVTCFG, T2IRQF, T2M0, T2M1, T2MOVF0, T2MOVF1, T2MOVF2, T2IRQM, T2MSEL})
        flash.op(mov_dd(dst++, sfr));
    // 7:4 R0, STATE read-only, SYNC/LATCH_MODE R/W, RUN = 0 (22.5 T2CTRL).
    flash.op(mov(T2CTRL, 0xFE)).op(mov_dd(0x50, T2CTRL));
    flash.op(mov(T2EVTCFG, 0xFF)).op(mov_dd(0x51, T2EVTCFG)); // 7 and 3 R0
    flash.op(mov(T2IRQM, 0xFF)).op(mov_dd(0x52, T2IRQM));     // 7:6 R0
    flash.op(mov(T2MSEL, 0xFF)).op(mov_dd(0x53, T2MSEL));     // 7 and 3 R0
    flash.op(mov(T2IRQF, 0xFF)).op(mov_dd(0x54, T2IRQF));     // R/W0: 1 sets nothing
    flash.op({0x80, 0xFE});
    Cc2530F256 device;
    load(device, flash);
    CHECK(device.run(200).error.empty());
    // Table 22-1 and 22.5: everything 0 except T2CTRL.SYNC = 1.
    const std::uint8_t expected[] = {0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (unsigned i = 0; i < 10; ++i)
        CHECK_EQ(device.read_idata(static_cast<std::uint8_t>(0x40 + i)), expected[i]);
    CHECK_EQ(device.read_idata(0x50), 0x0A);
    CHECK_EQ(device.read_idata(0x51), 0x77);
    CHECK_EQ(device.read_idata(0x52), 0x3F);
    CHECK_EQ(device.read_idata(0x53), 0x77);
    CHECK_EQ(device.read_idata(0x54), 0x00);
}

constexpr std::uint16_t Period = 1000, Cmp1 = 300, Cmp2 = 700;

// Period 1000, compares 300/700, only the period interrupt enabled, timer
// started with SYNC = 0 at the marked instruction.
Flash period_program(std::uint16_t &start) {
    Flash flash = program();
    flash.op(set16(2, Period)).op(set16(3, Cmp1)).op(set16(4, Cmp2));
    flash.op(mov(T2IRQM, 0x01)).op({0xD2, BitEa, 0xD2, BitT2ie});
    start = flash.here();
    flash.op(mov(T2CTRL, 0x01));
    sled(flash);
    return flash;
}

void test_period_compare_and_interrupt_timing() {
    std::uint16_t start = 0;
    const Flash flash = period_program(start);
    Cc2530F256 device;
    load(device, flash);
    const std::uint64_t t0 = run_to_pc(device, start);
    // 22.1.1/22.1.2: RUN = 1 with SYNC = 0 starts the timer immediately;
    // it increments every 32 MHz cycle. 22.1.5: compare flags are set when
    // the timer counts to t2_cmp1/t2_cmp2, regardless of T2IRQM (22.2).
    run_to(device, t0 + Cmp1 - 1);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x00);
    run_to(device, t0 + Cmp1);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x02);
    CHECK_EQ(device.read_sfr(IRCON) & 0x04, 0); // compare 1 masked: no T2IF
    run_to(device, t0 + Cmp2 - 1);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x02);
    run_to(device, t0 + Cmp2);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x06);
    run_to(device, t0 + Period - 1);
    CHECK(!device.in_interrupt());
    CHECK_EQ(device.read_idata(0x30), 0);
    // 22.1.3: at the period the timer wraps to 0, PERF is set and, with
    // TIMER2_PERM = 1, the T2 interrupt (vector 0x53) is requested. The
    // event is applied at the boundary at t0 + 1000; the entry takes
    // EntryCycles (SWRU191F 2.5.2).
    CHECK_EQ(run_to_pc(device, 0x0053), t0 + Period + EntryCycles);
    // ISR: flags captured (PERF | CMP1F | CMP2F) and cleared.
    run_to(device, t0 + Period + 40);
    CHECK_EQ(device.read_idata(0x30), 1);
    CHECK_EQ(device.read_idata(0x31), 0x07);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x00);
    // Second period: the timer restarted from 0 at t0 + 1000.
    run_to(device, t0 + Period + Cmp1 - 1);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x00);
    run_to(device, t0 + Period + Cmp1);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x02);
    CHECK_EQ(run_to_pc(device, 0x0053), t0 + 2 * Period + EntryCycles);
}

void test_period_interrupt_all_modes() {
    std::uint16_t start = 0;
    const Flash flash = period_program(start);
    // 10 periods after a start near cycle 60: 10 T2 interrupts in every
    // execution mode, identical final state.
    for_each_mode(
        flash, 10'800, [](Cc2530F256 &) {},
        [](Cc2530F256 &device, Mode mode) {
            if (device.read_idata(0x30) != 10) {
                std::cerr << "period interrupts [" << mode_name(mode)
                          << "]: " << +device.read_idata(0x30) << " entries\n";
                ++failures;
            }
            CHECK_EQ(device.read_idata(0x31), 0x07);
        },
        "mactimer period interrupts");
}

void test_latching() {
    Flash flash = program();
    const std::uint16_t start = flash.here();
    flash.op(mov(T2CTRL, 0x01)).op(delay(250)); // period 0: 16-bit roll-over
    const std::uint16_t read_a = flash.here();
    flash.op({0xE5, T2M0}).op({0xF5, 0x40}).op(delay(250)).op(mov_dd(0x41, T2M1));
    // Stop (SYNC = 0: immediate), period 100, t2tim = 0 while idle.
    flash.op(mov(T2CTRL, 0x00)).op(set16(2, 100)).op(set16(0, 0));
    const std::uint16_t start_b = flash.here();
    flash.op(mov(T2CTRL, 0x09)).op(delay(200)); // LATCH_MODE = 1, RUN
    const std::uint16_t read_c = flash.here();
    flash.op({0xE5, T2M0}).op({0xF5, 0x42}).op(delay(100));
    flash.op(mov_dd(0x43, T2MOVF0)).op(mov_dd(0x44, T2MOVF1)).op(mov_dd(0x45, T2MOVF2));
    flash.op(mov(T2CTRL, 0x01)); // LATCH_MODE = 0, still running
    const std::uint16_t read_d = flash.here();
    flash.op({0xE5, T2MOVF0}).op({0xF5, 0x46});
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    const std::uint64_t t0 = run_to_pc(device, start);
    const std::uint64_t ta = run_to_pc(device, read_a);
    const std::uint64_t tb = run_to_pc(device, start_b);
    const std::uint64_t tc = run_to_pc(device, read_c);
    const std::uint64_t td = run_to_pc(device, read_d);
    run_to(device, td + 20);
    // 22.1.2: T2M0 returns t2tim[7:0] and latches t2tim[15:8] for T2M1,
    // which still returns the latched byte after the timer moved on.
    CHECK(ta - t0 > 256);
    CHECK_EQ(device.read_idata(0x40), (ta - t0) & 0xFF);
    CHECK_EQ(device.read_idata(0x41), ((ta - t0) >> 8) & 0xFF);
    // 22.1.6 / T2CTRL.LATCH_MODE = 1: the T2M0 read latches the whole
    // overflow counter; t2ovf counts period events (period 100).
    CHECK_EQ(device.read_idata(0x42), (tc - tb) % 100);
    CHECK_EQ(device.read_idata(0x43), ((tc - tb) / 100) & 0xFF);
    CHECK_EQ(device.read_idata(0x44), 0);
    CHECK_EQ(device.read_idata(0x45), 0);
    // LATCH_MODE = 0: T2MOVF0 returns the current overflow count.
    CHECK_EQ(device.read_idata(0x46), ((td - tb) / 100) & 0xFF);
    CHECK(device.read_idata(0x46) != device.read_idata(0x43));
}

void test_delta_and_overflow_write_order() {
    constexpr std::uint16_t Delta = 0x0123;
    Flash flash = program();
    const std::uint16_t start = flash.here();
    flash.op(mov(T2CTRL, 0x01)).op(delay(100)).op(mov(T2M0, lo(Delta)));
    const std::uint16_t write = flash.here();
    flash.op(mov(T2M1, hi(Delta))).op(delay(200));
    const std::uint16_t read = flash.here();
    flash.op({0xE5, T2M0}).op({0xF5, 0x40}).op(mov_dd(0x41, T2M1));
    // 22.1.7: LSB first, the write takes effect on the MSB write.
    flash.op(mov(T2MOVF0, 0x11)).op(mov(T2MOVF1, 0x22));
    flash.op(mov_dd(0x42, T2MOVF0)).op(mov_dd(0x43, T2MOVF1)).op(mov_dd(0x44, T2MOVF2));
    flash.op(mov(T2MOVF2, 0x33));
    flash.op(mov_dd(0x45, T2MOVF0)).op(mov_dd(0x46, T2MOVF1)).op(mov_dd(0x47, T2MOVF2));
    // Reserved T2MSEL code 101: counted as unmodeled.
    flash.op(mov(T2MSEL, 0x05)).op(mov(T2M0, 0x55)).op(mov(T2M1, 0x66));
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    const std::uint64_t t0 = run_to_pc(device, start);
    const std::uint64_t tw = run_to_pc(device, write);
    const std::uint64_t tr = run_to_pc(device, read);
    run_to(device, tr + 100);
    // 22.1.4: the delta write halts the timer at its value at the write for
    // Delta cycles, then it counts again: value = (tr - t0) - Delta.
    CHECK(tr > tw + Delta);
    const std::uint64_t value = tr - t0 - Delta;
    CHECK_EQ(device.read_idata(0x40), value & 0xFF);
    CHECK_EQ(device.read_idata(0x41), (value >> 8) & 0xFF);
    CHECK_EQ(device.read_idata(0x42), 0);
    CHECK_EQ(device.read_idata(0x43), 0);
    CHECK_EQ(device.read_idata(0x44), 0);
    CHECK_EQ(device.read_idata(0x45), 0x11);
    CHECK_EQ(device.read_idata(0x46), 0x22);
    CHECK_EQ(device.read_idata(0x47), 0x33);
    const auto &writes = device.diagnostics().unmodeled_feature_writes;
    CHECK(writes.count(0x7000 | T2M1) && writes.at(0x7000 | T2M1) == 1);
}

void test_overflow_counter_events_and_outputs() {
    Flash flash = program();
    flash.op(set16(2, 50)).op(set24(2, 4)).op(set24(3, 2)).op(set24(4, 3));
    // EVENT1 = t2ovf_per_event (011), EVENT2 = t2_per_event (000) (22.5).
    flash.op(mov(T2EVTCFG, 0x03)).op(mov(T2IRQM, 0x10)); // OVF_COMPARE1M only
    const std::uint16_t start = flash.here();
    flash.op(mov(T2CTRL, 0x01));
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    auto &timer = *device.peripherals().mac_timer;
    const std::uint64_t t0 = run_to_pc(device, start);
    // 22.1.3/22.1.6: period events every 50 cycles increment t2ovf.
    run_to(device, t0 + 49);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x00);
    run_to(device, t0 + 50);
    // 22.1.5: t2_cmp1/t2_cmp2 keep their reset value 0x0000 (Table 22-2);
    // the timer counts to 0 at the period wrap: both compare flags as well.
    CHECK_EQ(device.read_sfr(T2IRQF), 0x07);
    CHECK_EQ(device.read_sfr(IRCON) & 0x04, 0); // PERF masked
    CHECK_EQ(timer.event_pulses(2), 1u);
    // 22.1.9: t2ovf counts to t2ovf_cmp1 = 2 at the second period event;
    // OVF_COMPARE1M = 1 requests the T2 interrupt (IRCON.T2IF, EA = 0).
    run_to(device, t0 + 99);
    CHECK_EQ(device.read_sfr(IRCON) & 0x04, 0);
    run_to(device, t0 + 100);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x17);
    CHECK_EQ(device.read_sfr(IRCON) & 0x04, 0x04);
    run_to(device, t0 + 150);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x37); // t2ovf_cmp2 = 3
    CHECK_EQ(timer.event_pulses(1), 0u);
    // 22.1.8: t2ovf would count to t2ovf_per = 4: wraps to 0, OVF_PERF.
    run_to(device, t0 + 199);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x37);
    run_to(device, t0 + 200);
    CHECK_EQ(device.read_sfr(T2IRQF), 0x3F);
    // 22.3: T2_EVENT1 pulsed once (overflow period), T2_EVENT2 at each of
    // the four period events.
    CHECK_EQ(timer.event_pulses(1), 1u);
    CHECK_EQ(timer.event_pulses(2), 4u);
    // T2EVTCFG = 111/111: no event.
    device.write_sfr(T2EVTCFG, 0x77);
    run_to(device, t0 + 400);
    CHECK_EQ(timer.event_pulses(1), 1u);
    CHECK_EQ(timer.event_pulses(2), 4u);
}

void test_flag_clear_semantics() {
    Flash flash = program();
    flash.op(set16(2, 100)).op(set16(3, 10)).op(set16(4, 20));
    flash.op(mov(T2CTRL, 0x01)).op(delay(60)); // > 100 cycles: PERF, CMP1F, CMP2F
    flash.op(mov(T2CTRL, 0x00));               // stop: no new flags
    flash.op(mov_dd(0x40, T2IRQF));
    flash.op(mov(T2IRQF, 0xFF)).op(mov_dd(0x41, T2IRQF)); // writing 1: no effect
    flash.op({0x53, T2IRQF, 0xFE}).op(mov_dd(0x42, T2IRQF)); // ANL: clears PERF only
    flash.op(mov(T2IRQF, 0x00)).op(mov_dd(0x43, T2IRQF));
    flash.op({0x80, 0xFE});
    Cc2530F256 device;
    load(device, flash);
    CHECK(device.run(1000).error.empty());
    // 22.5 T2IRQF: R/W0 flags, set by hardware, cleared by writing 0.
    CHECK_EQ(device.read_idata(0x40), 0x07);
    CHECK_EQ(device.read_idata(0x41), 0x07);
    CHECK_EQ(device.read_idata(0x42), 0x06);
    CHECK_EQ(device.read_idata(0x43), 0x00);
    // No interrupt: T2IRQM = 0 (22.2).
    CHECK_EQ(device.read_sfr(IRCON) & 0x04, 0);
}

void test_sfd_capture() {
    Flash flash = program();
    const std::uint16_t start = flash.here();
    flash.op(mov(T2CTRL, 0x01)).op(delay(250)).op(delay(250));
    const std::uint16_t read = flash.here();
    flash.op(mov(T2MSEL, 0x11)); // t2_cap / t2ovf_cap
    flash.op(mov_dd(0x40, T2M0)).op(mov_dd(0x41, T2M1));
    flash.op(mov_dd(0x42, T2MOVF0)).op(mov_dd(0x43, T2MOVF1)).op(mov_dd(0x44, T2MOVF2));
    flash.op(mov(T2M0, 0xAA)).op(mov(T2M1, 0xBB)); // t2_cap is read-only
    flash.op(mov_dd(0x45, T2M0)).op(mov_dd(0x46, T2M1));
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    const std::uint64_t t0 = run_to_pc(device, start);
    // Stop inside the first delay loop at the first DJNZ boundary at or
    // after t0 + 700 (Table 2-3: MOV direct,#data 3, MOV Rn,#data 2,
    // DJNZ Rn 3), after the capture time t0 + 650.
    constexpr unsigned Djnz = cycles_of({0xDF}), Loop = cycles_of({0x75, 0x7F});
    run_to(device, t0 + Loop + Djnz * ((700 - Loop + Djnz - 1) / Djnz));
    // 22.1.10: the radio SFD going high captures t2tim and t2ovf.
    device.peripherals().mac_timer->sfd_capture(t0 + 650);
    run_to_pc(device, read);
    run_to(device, device.registers().cycles + 100);
    CHECK_EQ(device.read_idata(0x40), lo(650));
    CHECK_EQ(device.read_idata(0x41), hi(650));
    CHECK_EQ(device.read_idata(0x42), 0);
    CHECK_EQ(device.read_idata(0x43), 0);
    CHECK_EQ(device.read_idata(0x44), 0);
    CHECK_EQ(device.read_idata(0x45), lo(650));
    CHECK_EQ(device.read_idata(0x46), hi(650));
}

void test_synchronized_stop_and_start() {
    Flash flash = program();
    const std::uint16_t start = flash.here();
    flash.op(mov(T2CTRL, 0x01)); // asynchronous first start
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    auto state = [&device] { return device.read_sfr(T2CTRL) & 0x04; };
    const std::uint64_t t0 = run_to_pc(device, start);
    const std::uint64_t ts = t0 + 200;
    run_to(device, ts);
    CHECK_EQ(state(), 0x04);
    device.write_sfr(T2CTRL, 0x02);
    // 22.4.2: RUN = 0 with SYNC = 1: the timer runs until the 32 kHz rising
    // edge (the next Sleep Timer increment, every 976.5625 cycles), then
    // STATE goes 1 -> 0 and the Sleep Timer value is stored.
    const std::uint64_t e1 = lf_tick_cycle(lf_ticks(ts) + 1);
    run_to(device, e1 - 1);
    CHECK_EQ(state(), 0x04);
    run_to(device, e1);
    CHECK_EQ(state(), 0x00);
    const std::uint64_t tst = e1 - t0; // stored timer value (period 0)
    // The stopped timer holds its value.
    run_to(device, e1 + 500);
    CHECK_EQ(device.read_sfr(T2M0), lo(tst));
    const auto &sleep_timer = *device.peripherals().sleep_timer;
    const std::uint32_t nst = sleep_timer.count(); // same 32 kHz period as e1
    const std::uint64_t tr = e1 + 3000;
    run_to(device, tr);
    device.write_sfr(T2CTRL, 0x03);
    // 22.4.3: RUN = 1 with SYNC = 1 waits for the next edge; RUN mode
    // (STATE = 1) TOH = 86 cycles after it.
    const std::uint64_t e2 = lf_tick_cycle(lf_ticks(tr) + 1);
    run_to(device, e2 + 85);
    CHECK_EQ(state(), 0x00);
    run_to(device, e2 + 86);
    CHECK_EQ(state(), 0x04);
    const std::uint32_t nc = sleep_timer.count(); // same 32 kHz period as e2
    // 22.4.3: C = Nd * 976.5625 + tST + TOH (rounded), loaded value
    // T = C mod t2per (65536 here, period 0); Nd = Sleep Timer ticks
    // between the stop edge and the start edge, here read from the Sleep
    // Timer itself (Nt = Nc - NST > 0).
    const std::uint64_t nd = lf_ticks(e2) - lf_ticks(e1);
    CHECK(nc > nst);
    CHECK_EQ(static_cast<std::uint64_t>(nc - nst), nd);
    const std::uint64_t c = (nd * 15625 + 8) / 16 + tst + 86;
    const std::uint64_t at = e2 + 86 + 1000;
    run_to(device, at);
    const std::uint16_t expected = static_cast<std::uint16_t>((c + 1000) & 0xFFFF);
    CHECK_EQ(device.read_sfr(T2M0), lo(expected));
    // 22.4.1: "it appears that the timer has not been stopped": within the
    // documented +-1 of a timer that kept counting since t0.
    const auto continuous = static_cast<std::uint16_t>((at - t0) & 0xFFFF);
    const int diff = static_cast<int>(expected) - static_cast<int>(continuous);
    CHECK(diff >= -1 && diff <= 1);
}

void test_pm1_freezes_the_timer() {
    Flash flash = program();
    flash.at(0x002B).op({0x32}); // sleep timer vector: RETI
    flash.at(Main);
    flash.op(set16(2, 1000)).op(mov(T2IRQM, 0x01));
    flash.op({0xD2, BitEa, 0xD2, BitStie, 0xD2, BitT2ie});
    flash.op(mov(SLEEPCMD, 0x01)); // PM1
    const std::uint16_t start = flash.here();
    flash.op(mov(T2CTRL, 0x01)).op(delay(100));
    while ((flash.here() & 3u) != 1u) // PCON write, next instruction aligned (4.2)
        flash.op({0x00});
    flash.op(mov(PCON, 0x01)).op({0x00});
    sled(flash);
    Cc2530F256 device;
    load(device, flash);
    const std::uint64_t t0 = run_to_pc(device, start);
    // Wake by the sleep-timer interrupt source 10000 cycles later.
    device.schedule_interrupt_flag_for_test(t0 + 20'000, 5);
    run_to(device, t0 + 30'000);
    const auto &diag = device.diagnostics();
    CHECK_EQ(diag.power_mode_entries_by_mode[2], 1u);
    const std::uint64_t asleep = diag.power_mode_cycles_by_mode[2];
    CHECK(asleep > 10'000);
    // SWRU191F 4.1 / 22: PM1 stops the 32 MHz clock, the timer does not
    // count while asleep: (30000 - asleep) cycles of counting = that many
    // period events at 1000 cycles each.
    const std::uint64_t counted = 30'000 - asleep;
    CHECK_EQ(device.read_idata(0x30), counted / 1000);
}

} // namespace

int main() {
    test_reset_values_and_masks();
    test_period_compare_and_interrupt_timing();
    test_period_interrupt_all_modes();
    test_latching();
    test_delta_and_overflow_write_order();
    test_overflow_counter_events_and_outputs();
    test_flag_clear_semantics();
    test_sfd_capture();
    test_synchronized_stop_and_start();
    test_pm1_freezes_the_timer();
    return finish("cc2530 mactimer");
}

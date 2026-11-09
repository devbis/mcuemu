/* TLSR8269 smoke firmware: built against the Telink SDK startup and driver
 * library (platform/boot/826x, libdrivers_826x.a). It prints over the UART
 * through the SDK's DMA uart_Send(), and counts a hardware timer interrupt.
 * Build: tests/tlsr826x/build.sh */
#include "driver_826x.h"

static unsigned char msg[4 + 16] __attribute__((aligned(4)));
static unsigned char rxbuf[64] __attribute__((aligned(16)));
volatile unsigned int ticks;

void irq_handler(void) {
    if (reg_irq_src & FLD_IRQ_TMR0_EN) {
        reg_tmr_sta = FLD_TMR_STA_TMR0;
        reg_tmr0_tick = 0; /* the timer stops at the capture value: restart it */
        ++ticks;
    }
}

static void say(const char *s) {
    unsigned n = 0;
    while (s[n] && n < 16) { msg[4 + n] = (unsigned char)s[n]; ++n; }
    msg[0] = (unsigned char)n; msg[1] = msg[2] = msg[3] = 0;
    while (!uart_Send(msg)) {}
    while (!(reg_dma_irq_src & FLD_DMA_UART_TX)) {}
    reg_dma_irq_src = FLD_DMA_UART_TX;
}

int main(void) {
    cpu_wakeup_init();
    clock_init(SYS_CLK_32M_RC);
    reg_tmr0_tick = 0;
    reg_tmr0_capt = 32000 * 5; /* 5 ms at 32 MHz */
    reg_tmr_ctrl8 = FLD_TMR0_EN;
    reg_irq_mask |= FLD_IRQ_TMR0_EN;
    irq_enable();
    UART_GPIO_CFG_PB2_PB3();
    uart_RecBuffInit(rxbuf, sizeof(rxbuf));
    uart_Init(115200, 0, 0, NOCONTROL);
    say("tlsr826x up\n");
    while (ticks < 3) {}
    say("ticks ok\n");
    for (;;) {}
}

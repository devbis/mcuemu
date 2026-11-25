/* Self test of the sleeps of the PHY62x2: runs on the board and in the emulator, prints with printk */
#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/retention/retention.h>
#include <zephyr/irq.h>
#include "soc.h"

#define MAGIC 0x5eed0001U
#define RTC_CNT (*(volatile uint32_t *)0x4000f028U)
#define SLEEP_R0 (*(volatile uint32_t *)0x4000f0c0U)
#define ROM_CAUSE (*(volatile uint32_t *)0x1fff0a80U)
#define SUSPEND_MS 500U
#define SUSPENDS 3U

struct state {
	uint32_t magic;
	uint32_t boots;
};

/* The UART loses its divisor and its interrupt enable in the suspend; the console is polled */
static uint32_t suspend_once(uint32_t ms)
{
	volatile uint32_t *const u = (volatile uint32_t *)0x40004000U;
	const uint32_t lcr = u[3];
	uint32_t dll, dlh, rtc0, rtc1;

	u[3] = lcr | 0x80U;
	dll = u[0];
	dlh = u[1];
	u[3] = lcr;
	k_busy_wait(30000);
	rtc0 = RTC_CNT;
	phy62x2_suspend((uint64_t)ms * 32768U / 1000U);
	rtc1 = RTC_CNT;
	u[3] = 0x80U;
	u[0] = dll;
	u[1] = dlh;
	u[3] = lcr;
	u[2] = 7U;
	return rtc1 - rtc0;
}

int main(void)
{
	const struct device *ret = DEVICE_DT_GET(DT_NODELABEL(retention0));
	struct state st = {0};
	uint32_t cause = 0U;

	(void)retention_read(ret, 0, (uint8_t *)&st, sizeof(st));
	if (st.magic != MAGIC) {
		st.magic = MAGIC;
		st.boots = 0U;
	}
	st.boots++;
	(void)retention_write(ret, 0, (uint8_t *)&st, sizeof(st));
	(void)hwinfo_get_reset_cause(&cause);
	printk("SLEEPTEST boot %u hwinfo 0x%x rom cause %u SLEEP_R0 0x%x\n", st.boots, cause,
	       ROM_CAUSE, SLEEP_R0);

	if (st.boots == 1U) {
		bool pass = true;

#ifdef NO_RC
		/* PMCTL0 bit 28: the RC oscillator of the RTC runs in the sleep (SDK hal_rtc_clock_config) */
		*(volatile uint32_t *)0x4000f014U &= ~BIT(28);
#endif

		for (uint32_t i = 0U; i < SUSPENDS; i++) {
			uint32_t ticks;
			unsigned int key = irq_lock();

			ticks = suspend_once(SUSPEND_MS);
			irq_unlock(key);
			printk("SLEEPTEST suspend %u returned after %u ticks\n", i, ticks);
			if (ticks < SUSPEND_MS * 32U || ticks > SUSPEND_MS * 32U + 3000U) {
				pass = false;
			}
		}
		printk("SLEEPTEST SUSPEND %s\n", pass ? "PASS" : "FAIL");
		printk("SLEEPTEST deep sleep %u ms\n", SUSPEND_MS);
		k_busy_wait(30000);
		(void)irq_lock();
		phy62x2_deep_sleep(SUSPEND_MS);
	}

	printk("SLEEPTEST DEEP %s\n", (st.boots == 2U && ROM_CAUSE == 5U) ? "PASS" : "FAIL");
	printk("SLEEPTEST DONE\n");
	k_sleep(K_FOREVER);
	return 0;
}

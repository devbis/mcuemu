/* SPDX-License-Identifier: MIT */
#include "mac.h"
#include "tl.h"

int main(void) {
    mac_init();
    tl_init();
    for (;;) {
        tl_poll();
        mac_poll();
        mac_flush();
        __asm volatile("cpsid i");
        if (!wake && !mac_busy()) __asm volatile("wfi");
        wake = 0;
        __asm volatile("cpsie i");
    }
}

#include <soc/timer_group_reg.h>
#include <hal/wdt_hal.h>

void wdt_off() {
    wdt_hal_context_t wdt0ctx = {.inst = WDT_MWDT0, .mwdt_dev = &TIMERG0};
    wdt_hal_write_protect_disable(&wdt0ctx);
    wdt_hal_disable(&wdt0ctx);
    wdt_hal_write_protect_enable(&wdt0ctx);
    wdt_hal_context_t wdt1ctx = {.inst = WDT_MWDT1, .mwdt_dev = &TIMERG1};
    wdt_hal_write_protect_disable(&wdt1ctx);
    wdt_hal_disable(&wdt1ctx);
    wdt_hal_write_protect_enable(&wdt1ctx);
}
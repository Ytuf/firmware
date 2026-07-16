// Fault catcher + auto-recovery for the FreeWili RP2350 (Cortex-M33) build.
//
// Root cause of the idle lockup (caught 2026-07-03): a raw assert() inside the
// TinyUSB native-USB HOST stack (hcd_rp2040.c:557 `assert(!ep->active)`) fails on
// a transfer resubmit race while servicing the GPS -> newlib abort() -> _exit ->
// BKPT -> HardFault, and because the stock RP2350 build ships NO fault handler
// that becomes a double-fault LOCKUP (PC=0xEFFFFFFE) that kills USB / re-enumerates
// CM0. This file makes the firmware survive both the assert and any real fault:
//
//   1. __assert_func override — a failed assert() anywhere records to g_fw_assert
//      and REBOOTS (clean self-heal) instead of abort->BKPT->lockup. (The known
//      hcd:557/562 asserts are also patched to return gracefully with no reboot;
//      this is the durable backstop for those and any other assert.)
//   2. Real fault handlers (Hard/Mem/Bus/Usage) — capture CFSR/HFSR/stacked-PC to
//      g_fw_fault, then REBOOT.
//   3. Loop guard — g_fw_recover.reboot_guard counts fast consecutive recover-
//      reboots; after a threshold we SPIN instead (so a fault storm can't boot-
//      loop — it stops in a diagnosable state). freewili_fault_tick() clears the
//      guard once the board has been up a while (proving the reboot recovered).
//
// g_fw_fault / g_fw_assert / g_fw_recover live in .uninitialized_data so they
// survive the recover-reboot. Read over SWD: nm the elf, mdw the struct.

#if defined(FREEWILI) && defined(PICO_RP2350)

#include <stdint.h>
#include "hardware/watchdog.h"
#include "pico/time.h"

#define FW_NOINIT __attribute__((section(".uninitialized_data"), used))
#define FW_RECOVER_MAGIC 0x5EC0FFEEu
#define FW_REBOOT_GUARD_MAX 6u      // spin after this many fast reboots
#define FW_GUARD_CLEAR_MS 60000u    // uptime that proves a reboot recovered

typedef struct {
    uint32_t magic; // 0xFA017CAF once a fault is captured
    uint32_t count, src, cfsr, hfsr, mmfar, bfar;
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr, sp, msp, psp;
} FwFaultRec;

typedef struct {
    uint32_t magic; // 0xA55E4707 once an assert is neutralized
    uint32_t count, line;
    const char *file, *func, *expr;
} FwAssertRec;

typedef struct {
    uint32_t magic;        // FW_RECOVER_MAGIC once initialized (else cold boot)
    uint32_t reboot_guard; // consecutive fast recover-reboots
    uint32_t assert_total; // asserts neutralized across resets
    uint32_t fault_total;  // real faults caught across resets
} FwRecover;

volatile FwFaultRec g_fw_fault FW_NOINIT;
volatile FwAssertRec g_fw_assert FW_NOINIT;
volatile FwRecover g_fw_recover FW_NOINIT;

extern "C" {

// USB-host stuck-EP recovery counters (referenced by name from hcd_rp2040.c).
// noinit so they accumulate across recover-reboots. If reclaim_xfer/setup climb,
// the stuck-EP theory is confirmed; last_ep is the ep_addr of the last reclaim.
volatile unsigned int g_fw_hcd_reclaim_xfer FW_NOINIT;
volatile unsigned int g_fw_hcd_reclaim_setup FW_NOINIT;
volatile unsigned int g_fw_hcd_last_ep FW_NOINIT;
volatile unsigned int g_fw_hcd_spurious FW_NOINIT; // ignored spurious/late completions

// After capturing a fault/assert, HOLD (spin) — do NOT reboot. The earlier
// watchdog_reboot recovery wedged the core / boot-looped (2026-07-04). As a pure
// backstop, capture-and-hold is safe + diagnosable; the real fix (USB-host state-
// machine hardening in hcd_rp2040.c) keeps the asserts from firing at all.
static void fw_recover_reboot(void)
{
    g_fw_recover.reboot_guard++;
    __asm volatile("dsb");
    for (;;)
        __asm volatile("wfi");
}

// newlib assert() failure lands here. Default aborts -> BKPT -> lockup; we record
// it and reboot instead. noreturn-compatible (never returns).
void __assert_func(const char *file, int line, const char *func, const char *expr)
{
    g_fw_assert.line = (uint32_t)line;
    g_fw_assert.file = file;
    g_fw_assert.func = func;
    g_fw_assert.expr = expr;
    g_fw_assert.count++;
    g_fw_recover.assert_total++;
    __asm volatile("dsb");
    g_fw_assert.magic = 0xA55E4707u;
    __asm volatile("dsb");
    fw_recover_reboot();
    for (;;) {
    }
}

__attribute__((used)) void fw_fault_capture(uint32_t src, uint32_t *frame)
{
    g_fw_fault.src = src;
    g_fw_fault.cfsr = *(volatile uint32_t *)0xE000ED28u;
    g_fw_fault.hfsr = *(volatile uint32_t *)0xE000ED2Cu;
    g_fw_fault.mmfar = *(volatile uint32_t *)0xE000ED34u;
    g_fw_fault.bfar = *(volatile uint32_t *)0xE000ED38u;
    g_fw_fault.r0 = frame[0];
    g_fw_fault.r1 = frame[1];
    g_fw_fault.r2 = frame[2];
    g_fw_fault.r3 = frame[3];
    g_fw_fault.r12 = frame[4];
    g_fw_fault.lr = frame[5];
    g_fw_fault.pc = frame[6];
    g_fw_fault.xpsr = frame[7];
    g_fw_fault.sp = (uint32_t)frame;
    uint32_t msp, psp;
    __asm volatile("mrs %0, msp" : "=r"(msp));
    __asm volatile("mrs %0, psp" : "=r"(psp));
    g_fw_fault.msp = msp;
    g_fw_fault.psp = psp;
    g_fw_fault.count++;
    g_fw_recover.fault_total++;
    __asm volatile("dsb");
    g_fw_fault.magic = 0xFA017CAFu; // set LAST: presence == complete capture
    __asm volatile("dsb");
    fw_recover_reboot();
    for (;;) {
    }
}

// Naked entry points: tag the source, pick the frame's stack (EXC_RETURN bit 2:
// 0=MSP, 1=PSP) into r1, then jump to the C capturer.
__attribute__((naked)) static void fw_fault_hard(void)
{
    __asm volatile("movs r0,#3\n tst lr,#4\n ite eq\n mrseq r1,msp\n mrsne r1,psp\n b fw_fault_capture\n");
}
__attribute__((naked)) static void fw_fault_mem(void)
{
    __asm volatile("movs r0,#4\n tst lr,#4\n ite eq\n mrseq r1,msp\n mrsne r1,psp\n b fw_fault_capture\n");
}
__attribute__((naked)) static void fw_fault_bus(void)
{
    __asm volatile("movs r0,#5\n tst lr,#4\n ite eq\n mrseq r1,msp\n mrsne r1,psp\n b fw_fault_capture\n");
}
__attribute__((naked)) static void fw_fault_usage(void)
{
    __asm volatile("movs r0,#6\n tst lr,#4\n ite eq\n mrseq r1,msp\n mrsne r1,psp\n b fw_fault_capture\n");
}

// 512-byte-aligned RAM vector table (>= 68 RP2350 vectors, next pow2 of 272B).
static uint32_t s_fw_vtable[128] __attribute__((aligned(512)));

void freewili_fault_init(void)
{
    // Cold boot => .uninitialized_data holds garbage. Initialize the recover state
    // (and clear stale capture magics) only when the magic proves it wasn't a
    // preserved-across-reset image.
    if (g_fw_recover.magic != FW_RECOVER_MAGIC) {
        g_fw_recover.magic = FW_RECOVER_MAGIC;
        g_fw_recover.reboot_guard = 0;
        g_fw_recover.assert_total = 0;
        g_fw_recover.fault_total = 0;
        g_fw_fault.magic = 0;
        g_fw_assert.magic = 0;
        g_fw_hcd_reclaim_xfer = 0;
        g_fw_hcd_reclaim_setup = 0;
        g_fw_hcd_last_ep = 0;
        g_fw_hcd_spurious = 0;
    }

    volatile uint32_t *SCB_VTOR = (volatile uint32_t *)0xE000ED08u;
    volatile uint32_t *SCB_SHCSR = (volatile uint32_t *)0xE000ED24u;

    const uint32_t *cur = (const uint32_t *)(*SCB_VTOR);
    for (int i = 0; i < 128; i++)
        s_fw_vtable[i] = cur[i];
    s_fw_vtable[3] = (uint32_t)&fw_fault_hard;  // HardFault
    s_fw_vtable[4] = (uint32_t)&fw_fault_mem;   // MemManage
    s_fw_vtable[5] = (uint32_t)&fw_fault_bus;   // BusFault
    s_fw_vtable[6] = (uint32_t)&fw_fault_usage; // UsageFault
    __asm volatile("dsb");
    *SCB_VTOR = (uint32_t)s_fw_vtable;
    __asm volatile("dsb\n isb");

    // MEMFAULTENA | BUSFAULTENA | USGFAULTENA — don't let faults escalate blind.
    *SCB_SHCSR |= (1u << 16) | (1u << 17) | (1u << 18);
    __asm volatile("dsb\n isb");
}

// Call from the main loop: once the board has been up long enough to prove the
// last recover-reboot succeeded, clear the loop guard so future rare faults are
// tolerated (each gets its own reboot budget again).
void freewili_fault_tick(void)
{
    if (g_fw_recover.reboot_guard != 0 && to_ms_since_boot(get_absolute_time()) > FW_GUARD_CLEAR_MS)
        g_fw_recover.reboot_guard = 0;
}

} // extern "C"

#endif // FREEWILI && PICO_RP2350

// 240 MHz sys-clock support: QMI flash-timing + core-voltage management.
//
// arduino-pico's main() raises clk_sys via set_sys_clock_khz() (a static
// inline that lowers to an out-of-line set_sys_clock_pll() call) but never
// retunes the flash QMI divider — it only touches m[1]/PSRAM, absent here —
// and its vreg bump is compiled only #if PICO_RP2040. The RP2350 bootrom
// boots this board with m[0].timing = 0x60007203 (CLKDIV=3, RXDELAY=2;
// measured live 2026-07-02): 50 MHz flash SCK at the 150 MHz runtime
// default, i.e. exactly the W25Q128JV's 03h serial-read ceiling, and 40 MHz
// at the shipping 120 MHz baseline. Raising clk_sys to 240 with CLKDIV=3
// would clock the flash at 80 MHz and XIP returns garbage (verified live
// 2026-07-01: random precise bus faults, ASCII-as-pointer loads).
//
// Two wrap layers fix this (flags in variants/rp2350/freewili/platformio.ini):
//
//  1. __wrap_set_sys_clock_pll — before a clock RAISE: bump vreg to 1.15 V
//     (RP2350 default ~1.10 V is only rated to ~150 MHz), then slow the
//     flash divider for the TARGET frequency, then let the PLL switch run.
//     set_sys_clock_pll parks clk_sys on the 48 MHz USB PLL mid-switch, so
//     with the divider pre-applied the flash SCK path is 25 → 8 → 40 MHz,
//     never above spec. On a LOWER the order flips (switch, then retune).
//
//  2. __wrap_flash_range_erase/_program/_do_cmd — pico-sdk restores XIP
//     after EVERY flash op by re-running the bootrom's saved XIP-setup stub
//     from BOOTRAM (flash_enable_xip_via_boot2 with boot2_source=none.S),
//     which rewrites m[0].timing with the boot-time CLKDIV=3 → 80 MHz at
//     240. Re-apply our timing after each op, with the whole
//     {__real_* … re-apply} sequence under PRIMASK: the FreeRTOS flash
//     lockout is BASEPRI=16, which priority-0 IRQs (the PIO-USB packet IRQ)
//     pierce — and that handler touches flash-resident code, which is fatal
//     while XIP is down at any clock and silently corrupting in the
//     reverted-timing tail at 240. PRIMASK holds it off on this core for
//     the op (the device NAKs in hardware; the host retries). The same
//     pierce on the PARKED core is a pre-existing lottery this wrapper
//     cannot reach. __real_* and these wrappers are fully RAM-resident, so
//     the reverted window itself never fetches from flash. All in-tree
//     callers (LittleFS, EEPROM, Updater, FatFS) reach these symbols
//     cross-TU, so the linker wrap intercepts them.
//
// KNOWN ESCAPE: flash_get_unique_id() calls flash_do_cmd() intra-TU
// (unwrappable), and its own body is flash-resident — a call at 240 MHz
// would fetch garbage before any wrapper could retime. It is dead code on
// RP2350 (pico_unique_id uses the OTP ROM call) and must stay that way:
// never add a flash_get_unique_id caller to this build.
//
// Target flash SCK is 40 MHz == the shipping 120 MHz baseline (120/3), not
// the 50 MHz bootrom point, for margin. RXDELAY (units: half a clk_sys
// cycle) is derived from the field-proven baseline point — 2 units at
// 120 MHz = 8.33 ns — as ceil(clk_sys / 60 MHz): 4 @240, 2 @120. That is
// deliberately NOT rescaled from the live register (a live-value ratchet
// would creep to the field cap if the clock ever changed twice per boot).
// If a sweep is needed, build with -DFW_QMI_RXDELAY_OVERRIDE=<3|4|5>.

#if defined(FREEWILI) && defined(PICO_RP2350)

#include <hardware/structs/qmi.h>
#include <hardware/regs/addressmap.h> // XIP_NOCACHE_NOALLOC_BASE
#include <hardware/sync.h>
#include <hardware/clocks.h>
#include <hardware/vreg.h>
#include <pico/platform.h> // __not_in_flash_func, busy_wait_at_least_cycles

// Flash SCK ceiling: the W25Q128JV is on 03h single-lane reads (50 MHz max);
// 40 MHz reproduces the proven 120 MHz-baseline operating point.
#define FW_FLASH_SCK_TARGET_HZ 40000000u
// One RXDELAY unit (half a clk_sys cycle) per 60 MHz of clk_sys == the
// baseline's absolute sample delay (2 units @120 MHz).
#define FW_QMI_RXDELAY_UNIT_HZ 60000000u

// Timing word computed at the clock raise; flash-op wrappers re-apply it.
// 0 until the first set_sys_clock_pll call (nothing to re-apply before then:
// the bootrom timing is correct for the 150 MHz runtime default).
static volatile uint32_t s_fw_qmi_timing;

static void __not_in_flash_func(fw_qmi_write_timing)(uint32_t t)
{
    uint32_t save = save_and_disable_interrupts();
    qmi_hw->m[0].timing = t;
    (void)*(volatile uint32_t *)XIP_NOCACHE_NOALLOC_BASE; // datasheet-mandated resync read
    __dmb();
    restore_interrupts(save);
}

static uint32_t __not_in_flash_func(fw_qmi_compute_timing)(uint32_t target_hz)
{
    uint32_t cur = qmi_hw->m[0].timing;

    uint32_t clkdiv = (target_hz + FW_FLASH_SCK_TARGET_HZ - 1u) / FW_FLASH_SCK_TARGET_HZ;
    if (clkdiv < 1u)
        clkdiv = 1u;
    if (clkdiv > (QMI_M0_TIMING_CLKDIV_BITS >> QMI_M0_TIMING_CLKDIV_LSB))
        clkdiv = QMI_M0_TIMING_CLKDIV_BITS >> QMI_M0_TIMING_CLKDIV_LSB;

#ifdef FW_QMI_RXDELAY_OVERRIDE
    uint32_t rx = FW_QMI_RXDELAY_OVERRIDE;
#else
    uint32_t rx = (target_hz + FW_QMI_RXDELAY_UNIT_HZ - 1u) / FW_QMI_RXDELAY_UNIT_HZ;
#endif
    if (rx > (QMI_M0_TIMING_RXDELAY_BITS >> QMI_M0_TIMING_RXDELAY_LSB))
        rx = QMI_M0_TIMING_RXDELAY_BITS >> QMI_M0_TIMING_RXDELAY_LSB;

    uint32_t t = cur & ~(QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS);
    t |= (clkdiv << QMI_M0_TIMING_CLKDIV_LSB) & QMI_M0_TIMING_CLKDIV_BITS;
    t |= (rx << QMI_M0_TIMING_RXDELAY_LSB) & QMI_M0_TIMING_RXDELAY_BITS;
    return t;
}

extern "C" {

void __real_set_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2);
void __real_flash_range_erase(uint32_t flash_offs, size_t count);
void __real_flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);
void __real_flash_do_cmd(const uint8_t *txbuf, uint8_t *rxbuf, size_t count);

void __not_in_flash_func(__wrap_set_sys_clock_pll)(uint32_t vco_freq, uint post_div1, uint post_div2)
{
    uint32_t target_hz = vco_freq / (post_div1 * post_div2);
    uint32_t cur_hz = clock_get_hz(clk_sys);

    if (target_hz > 150000000u && vreg_get_voltage() < VREG_VOLTAGE_1_15) {
        // vreg_set_voltage blocks until POWMAN finishes the ramp; add ~2 ms
        // of analog settle on top (timer may not be reliable yet — count CPU
        // cycles at the still-current clock).
        vreg_set_voltage(VREG_VOLTAGE_1_15);
        busy_wait_at_least_cycles(cur_hz / 500u);
    }

    uint32_t t = fw_qmi_compute_timing(target_hz);
    if (target_hz >= cur_hz) {
        // Raising: slow the flash clock first so SCK never overshoots.
        s_fw_qmi_timing = t;
        fw_qmi_write_timing(t);
        __real_set_sys_clock_pll(vco_freq, post_div1, post_div2);
    } else {
        // Lowering: the old (larger) divider stays in-spec during the
        // switch; retune afterwards.
        __real_set_sys_clock_pll(vco_freq, post_div1, post_div2);
        s_fw_qmi_timing = t;
        fw_qmi_write_timing(t);
    }
}

void __not_in_flash_func(__wrap_flash_range_erase)(uint32_t flash_offs, size_t count)
{
    uint32_t save = save_and_disable_interrupts();
    __real_flash_range_erase(flash_offs, count);
    uint32_t t = s_fw_qmi_timing;
    if (t)
        fw_qmi_write_timing(t);
    restore_interrupts(save);
}

void __not_in_flash_func(__wrap_flash_range_program)(uint32_t flash_offs, const uint8_t *data, size_t count)
{
    uint32_t save = save_and_disable_interrupts();
    __real_flash_range_program(flash_offs, data, count);
    uint32_t t = s_fw_qmi_timing;
    if (t)
        fw_qmi_write_timing(t);
    restore_interrupts(save);
}

void __not_in_flash_func(__wrap_flash_do_cmd)(const uint8_t *txbuf, uint8_t *rxbuf, size_t count)
{
    uint32_t save = save_and_disable_interrupts();
    __real_flash_do_cmd(txbuf, rxbuf, count);
    uint32_t t = s_fw_qmi_timing;
    if (t)
        fw_qmi_write_timing(t);
    restore_interrupts(save);
}

} // extern "C"

#endif // FREEWILI && PICO_RP2350

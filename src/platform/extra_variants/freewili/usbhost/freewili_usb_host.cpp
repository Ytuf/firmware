// FreeWili 2 DISPLAY RP2350 — native USB host bring-up (Task 3 spike).
// See freewili_usb_host.h for the overview.
//
// Integration route: arduino-pico's own Adafruit_TinyUSB stack (Route 1 of the
// task brief). Defining USE_TINYUSB + USE_TINYUSB_HOST (in platformio.ini)
// flips the native controller (roothub port 0) from CDC *device* to *host*
// (see ports/rp2040/tusb_config_rp2040.h: USE_TINYUSB_HOST -> CFG_TUD_ENABLED
// 0, CFG_TUH_ENABLED 1, CFG_TUH_RPI_PIO_USB 0). Because per-TU include-path
// ordering made the core's device-only tusb_config.h race Adafruit's host-aware
// one, we now force ONE global config via -D CFG_TUSB_CONFIG_FILE (see
// variants/rp2350/freewili/tusb_config_freewili.h) so every TU agrees on
// CFG_TUH_ENABLED + CFG_TUH_HUB (the u-blox sits behind the CH334F hub).

#include "configuration.h" // LOG_INFO / LOG_WARN

#if defined(FREEWILI) && defined(USE_TINYUSB_HOST)

#include "freewili_usb_host.h"

#include "../freewili_gps.h"
#include "Adafruit_TinyUSB.h"
#include <hardware/gpio.h>

extern "C" {
#include "pio_usb.h" // pio_usb_device_task — vendored lib/pico_pio_usb
}
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <pico/time.h>

// Enumeration control stages have host-side deadlines (SET_ADDRESS gives the
// device 50 ms) that Meshtastic's main loop cannot meet — its boot-time gaps
// run 100-300 ms and the host resets us mid-enumeration (observed live:
// Set Address processed, then the host's request to the new address timed
// out and it started over). Pump the device engine from a repeating hardware
// timer instead. TinyUSB_Device_Task is ISR-safe by design (mutex_try_enter
// — the rp2040 port itself invokes it from a user IRQ).
static repeating_timer_t s_usb_device_timer;

// The device engine (pio_usb_device_task + TinyUSB_Device_Task) is non-reentrant
// and is pumped from BOTH this 4 kHz timer ISR and the main loop on core0. With
// no USB device attached, the main loop reaches its pump region at a fixed early
// instant that coincides with the timer ticks; the ISR then re-enters the engine
// mid-pump and the striped __usb_mutex spinlock deadlocks (observed: boot hangs
// in mutex_exit -> spin_lock_unsafe_blocking, LR here). The ISR strictly nests
// inside loop() on core0 (never truly concurrent), so a plain flag is race-free:
// skip the tick whenever the main loop already owns the engine.
static volatile bool s_usb_pump_active = false;

static bool freewili_usb_device_timer_cb(repeating_timer_t *)
{
    if (s_usb_pump_active)
        return true; // main loop is mid-pump — don't re-enter the device engine
    pio_usb_device_task();
    TinyUSB_Device_Task();
    return true; // keep repeating
}

// GDB-observable state.
volatile uint16_t g_freewili_gps_vid = 0;
volatile uint16_t g_freewili_gps_pid = 0;
volatile uint32_t g_freewili_gps_mount_count = 0;
volatile uint32_t g_freewili_gps_nmea_bytes = 0;
volatile uint32_t g_freewili_usbhost_task_count = 0;

// Native USB host on roothub port 0 (no ctor args == native controller).
static Adafruit_USBH_Host USBHost;

// CDC-host endpoint bound to the first mounted CDC interface (the GPS).
static Adafruit_USBH_CDC SerialGps;

// True only between tuh_cdc_mount_cb and tuh_cdc_umount_cb. Touching
// SerialGps while unmounted passes TUSB_INDEX_INVALID into tuh_cdc_*,
// whose TU_ASSERT executes a BKPT whenever a debugger has been attached
// since power-on (DHCSR.C_DEBUGEN latches) — halting core0 on every
// loop() pass. Gate all CDC access on this flag.
static volatile bool s_gps_cdc_mounted = false;

void freewiliUsbHostInit(void)
{
    // Bring up the TinyUSB host stack on the native controller (rhport 0).
    // VBUS on the USB-A ports (HP1/HP2_EN) is already driven high by the IO
    // expander during initIOExpanderPicoSDK(), so devices can enumerate.
    USBHost.begin(0);
    LOG_INFO("FreeWili USB host: native controller up on rhport0\n");

    // Task 5 device side: pio_usb device mode expects an EXTERNAL 1.5k
    // pull-up on D+ (pio_usb_device_init calls gpio_disable_pulls with a
    // "needs external pull-up" comment). On FW2 the pull-up exists but is
    // shared: GPIO42/USB_SEC_P runs through R344 (1.5k) to WIO_BOOT — the
    // Wio-E5's PB13 boot strap (rev 23 sheets 3+7). Attach only works while
    // the WIO drives PB13 HIGH; the current wio-e5-bridge leaves it floating,
    // so the hub sees nothing until the bridge adds that (pending). The
    // internal pull below keeps the strap weakly high in the meantime
    // (~50k — NOT enough for USB attach, but harmless).
    gpio_pull_up(PIO_USB_DP_PIN_DEFAULT);

    // 250 us cadence ≈ 4 kHz: fast enough for control-stage deadlines, a few
    // µs of work per tick when idle.
    add_repeating_timer_us(-250, freewili_usb_device_timer_cb, nullptr, &s_usb_device_timer);

    // The pio_usb packet IRQ must OUTRANK the pump timer (both default to
    // 0x80): pio_usb_device_task() busy-waits through the 10 ms bus-reset
    // SE0 inside the timer callback, and a same-priority packet IRQ can't
    // preempt it — token responses miss the host's turnaround window and
    // enumeration data stages time out (observed live). Priority 0 lets
    // packet handling interrupt the pump.
    irq_set_priority(PIO_IRQ_NUM(pio0, 0), 0);
}

void freewiliUsbHostService(void)
{
    // Proves the host task is actually being polled (GDB-observable). If this
    // stays 0 the loop never reaches here; if it climbs while mount_count stays
    // 0 the stack is polled but nothing enumerates.
    g_freewili_usbhost_task_count++;

    // Drive enumeration / transfers. Cooperative, non-blocking.
    USBHost.task();

    // Task 5: pump the PIO-USB DEVICE side (Meshtastic CDC console). Under
    // FreeRTOS nothing else runs these — the core only calls them from the
    // non-FreeRTOS delay()/yield() paths.
    //
    // pio_usb_device_task() is the pio-usb device engine for this library
    // revision: it converts the IRQ-level rport->ints into TinyUSB dcd
    // events (via pio_usb_device_irq_handler → dcd_pio_usb.c), continues
    // pending EP0 descriptor stages, detects bus reset by polling SE0, and
    // re-enables the edge-detector SM after a reset. Without this pump the
    // device attaches (pull-up) but never answers enumeration.
    s_usb_pump_active = true; // block the timer ISR from re-entering the device engine
    pio_usb_device_task();
    TinyUSB_Device_Task();
    TinyUSB_Device_FlushCDC();
    s_usb_pump_active = false;

    // Drain the GPS CDC stream and log raw NMEA. tuh_cdc_rx_cb queues bytes;
    // we read them here on the main loop so the ISR/callback stays short.
    if (s_gps_cdc_mounted && SerialGps.connected()) {
        uint8_t buf[64];
        int n = SerialGps.read(buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = 0;
            g_freewili_gps_nmea_bytes += (uint32_t)n;
            // Task 2/4: parse + inject into NodeDB (map + position broadcasts).
            freewiliGpsFeed(buf, (size_t)n);
            // Raw NMEA bytes ($GNRMC/$GNGGA/...). Trailing CR/LF included.
            LOG_INFO("FreeWili GPS NMEA: %s", (const char *)buf);
        }
    }
}

//------------- TinyUSB host callbacks -------------//

extern "C" {

// Any device (including hubs) mounts here. Log VID/PID for the spike; the
// u-blox M8 is expected to report VID=1546 PID=01a8.
void tuh_mount_cb(uint8_t dev_addr)
{
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);
    g_freewili_gps_vid = vid;
    g_freewili_gps_pid = pid;
    LOG_INFO("FreeWili USB mount: dev %u VID=%04x PID=%04x\n", dev_addr, vid, pid);
}

void tuh_umount_cb(uint8_t dev_addr)
{
    LOG_INFO("FreeWili USB umount: dev %u\n", dev_addr);
}

// A CDC (ACM) interface enumerated — bind our SerialGps to it.
void tuh_cdc_mount_cb(uint8_t idx)
{
    SerialGps.mount(idx);
    g_freewili_gps_mount_count++;
    s_gps_cdc_mounted = true;
    LOG_INFO("FreeWili USB CDC mounted: idx %u (GPS)\n", idx);
}

void tuh_cdc_umount_cb(uint8_t idx)
{
    s_gps_cdc_mounted = false;
    SerialGps.umount(idx);
    freewiliGpsResetFix(); // Task 6: UI degrades to node-list w/o local fix
    LOG_INFO("FreeWili USB CDC unmounted: idx %u\n", idx);
}

} // extern "C"

#endif // FREEWILI && USE_TINYUSB_HOST

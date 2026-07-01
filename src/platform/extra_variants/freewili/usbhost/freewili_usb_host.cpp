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

#include "Adafruit_TinyUSB.h"

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
}

void freewiliUsbHostService(void)
{
    // Proves the host task is actually being polled (GDB-observable). If this
    // stays 0 the loop never reaches here; if it climbs while mount_count stays
    // 0 the stack is polled but nothing enumerates.
    g_freewili_usbhost_task_count++;

    // Drive enumeration / transfers. Cooperative, non-blocking.
    USBHost.task();

    // Drain the GPS CDC stream and log raw NMEA. tuh_cdc_rx_cb queues bytes;
    // we read them here on the main loop so the ISR/callback stays short.
    if (s_gps_cdc_mounted && SerialGps.connected()) {
        uint8_t buf[64];
        int n = SerialGps.read(buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = 0;
            g_freewili_gps_nmea_bytes += (uint32_t)n;
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
    LOG_INFO("FreeWili USB CDC unmounted: idx %u\n", idx);
}

} // extern "C"

#endif // FREEWILI && USE_TINYUSB_HOST

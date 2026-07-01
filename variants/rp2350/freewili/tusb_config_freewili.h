/*
 * FreeWili 2 DISPLAY (RP2350B) — GLOBAL TinyUSB config for the native-USB HOST build.
 *
 * WHY THIS FILE EXISTS (the include-path landmine):
 *   The arduino-pico core ships TWO candidate `tusb_config.h` files:
 *     - framework-arduinopico/include/tusb_config.h            (DEVICE-only; no CFG_TUH_*)
 *     - Adafruit_TinyUSB_Arduino/src/tusb_config.h -> ports/rp2040/tusb_config_rp2040.h
 *   Which one a given translation unit picks depends on per-TU `-I` ordering, so
 *   different TUs disagreed on CFG_TUD_ENABLED / CFG_TUH_ENABLED (the fragility flagged
 *   in task-3-report.md). If the HOST sources (usbh.c / hub.c / cdc_host.c) happen to
 *   compile against the device-only config, CFG_TUH_ENABLED is undefined -> the entire
 *   host stack compiles to nothing -> USBHost.begin()/task() are no-ops -> nothing
 *   behind the CH334F hub can ever enumerate.
 *
 *   tusb_option.h honours `CFG_TUSB_CONFIG_FILE`: if that macro is defined it includes
 *   the named file INSTEAD of "tusb_config.h", bypassing the include-path race entirely.
 *   platformio.ini defines `-D CFG_TUSB_CONFIG_FILE="tusb_config_freewili.h"` so EVERY
 *   TU that pulls in tusb_option.h uses this one file -> one consistent configuration.
 *
 * Mirrors the proven pico-SDK reference at
 *   freewili-firmware/freewilimain/usb-host/tusb_config.h
 * (CFG_TUH_HUB=1, CFG_TUH_CDC=2, CH34x vendor-CDC on, native host on rhport0).
 *
 * Device stack is fully OFF (native controller is a HOST). CFG_TUD_CDC is kept nonzero
 * so the Adafruit library still emits the `Serial` object + its host-mode no-op methods
 * (Adafruit_USBD_CDC.cpp guards the object on CFG_TUD_CDC, the real I/O on CFG_TUD_ENABLED),
 * which keeps the link consistent without a stubbed Serial.
 */

#ifndef _FREEWILI_TUSB_CONFIG_H_
#define _FREEWILI_TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RP2040 /* RP2350 shares the RP2040 TinyUSB port */
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_PICO
#endif

// Spike diagnostics: TinyUSB logs go to a RAM ring buffer (no console in host
// mode) — see usbhost/freewili_tusb_log.cpp; dump g_freewili_tusb_log over SWD.
// Level 2 traces every enumeration step/transfer result. Set back to 0 when done.
//
// Defining CFG_TUSB_DEBUG_PRINTF makes Adafruit_TinyUSB_API.cpp emit the
// function DEFINITION itself, writing to SERIAL_TUSB_DEBUG (default Serial1 —
// a real UART we must not drive). Point SERIAL_TUSB_DEBUG at a ring-buffer
// object instead of defining the printf ourselves (that would double-define).
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 2
#endif
#define CFG_TUSB_DEBUG_PRINTF freewili_tusb_ram_printf
#ifdef __cplusplus
#include <stddef.h> /* size_t — this header is included before any libc header */
class FreeWiliTusbRingLog
{
  public:
    void begin(unsigned long) {}
    size_t write(const char *s);
};
extern FreeWiliTusbRingLog g_freewiliTusbLog;
#define SERIAL_TUSB_DEBUG g_freewiliTusbLog
#endif

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN TU_ATTR_ALIGNED(4)

//--------------------------------------------------------------------
// ROLE: native controller (roothub port 0) = HOST, device OFF
//--------------------------------------------------------------------

#define CFG_TUD_ENABLED     0 /* device stack off — native root is a host */
#define CFG_TUH_ENABLED     1
#define CFG_TUH_RPI_PIO_USB 0 /* native RP2350 USB controller, NOT Pico-PIO-USB */
#define CFG_TUH_MAX3421     0
#define BOARD_TUH_RHPORT    0
#define CFG_TUH_MAX_SPEED   OPT_MODE_FULL_SPEED

//--------------------------------------------------------------------
// DEVICE — disabled, but keep CFG_TUD_CDC nonzero so the library still
// defines the (inert) `Serial` object and its host-mode stub methods.
//--------------------------------------------------------------------

#define CFG_TUD_ENDPOINT0_SIZE 64
#define CFG_TUD_CDC            2
#define CFG_TUD_MSC            0
#define CFG_TUD_HID            0
#define CFG_TUD_MIDI          0
#define CFG_TUD_VENDOR        0
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256

//--------------------------------------------------------------------
// HOST
//--------------------------------------------------------------------

// Enumeration scratch buffer — 512 (matches the reference) is comfortable for a
// hub + full-speed device config descriptor.
#define CFG_TUH_ENUMERATION_BUFSIZE 512

// The u-blox sits BEHIND two hub tiers — hub support is mandatory and MUST
// be >= 2: tier 1 is the on-board CH334F (VID 1A86 PID 8091; D1/D2 wired to
// the USB-A jacks), tier 2 is a hub INSIDE the Intrepid GPS dongle itself
// (class-09 device on the CH334F port, u-blox CDC behind it). Each hub needs
// its own address slot; with 1, the dongle's hub fails enumeration at
// usbh.c:1577 `TU_ASSERT(new_addr != 0)` (verified via the RAM log 2026-07-01)
// and nothing plugged into USB-A can ever mount. 3 = one slot of headroom.
// Documented in ../freewili-firmware/agents/hardware/usb-topology.md.
#define CFG_TUH_HUB 3

// Devices (excluding hubs). Reference uses 4.
#define CFG_TUH_DEVICE_MAX 4

// CDC host: >=2 interfaces (the GPS is one; headroom for a second CDC).
#define CFG_TUH_CDC 2

// Serial-adapter "vendor" CDC drivers (mirror the reference: only CH34x on; the
// Intrepid u-blox M8 is a genuine CDC-ACM device so these are for completeness).
#define CFG_TUH_CDC_FTDI   0
#define CFG_TUH_CDC_CP210X 0
#define CFG_TUH_CDC_CH34X  1
#define CFG_TUH_CDC_PL2303 0

#define CFG_TUH_CDC_RX_BUFSIZE 128
#define CFG_TUH_CDC_TX_BUFSIZE 128

// Classes not needed for the GPS spike — keep the host surface minimal.
// NB: CFG_TUH_MSC MUST stay nonzero. Adafruit_USBH_CDC.cpp has a guard bug
// (`#if CFG_TUH_ENABLED && CFG_TUH_MSC` — should be CFG_TUH_CDC), so the entire
// Adafruit_USBH_CDC class (ctor/mount/read/connected...) compiles out and the
// link fails with 0 for MSC. Leave it at 1 (matches the proven rp2040 config).
#define CFG_TUH_MSC    1
#define CFG_TUH_HID    0
#define CFG_TUH_VENDOR 0

// Assert DTR|RTS and 115200 8N1 on the CDC device when it mounts (GPS default).
#define CFG_TUH_CDC_LINE_CONTROL_ON_ENUM (CDC_CONTROL_LINE_STATE_DTR | CDC_CONTROL_LINE_STATE_RTS)
#define CFG_TUH_CDC_LINE_CODING_ON_ENUM                                                            \
    {                                                                                              \
        115200, CDC_LINE_CODING_STOP_BITS_1, CDC_LINE_CODING_PARITY_NONE, 8                        \
    }

#ifdef __cplusplus
}
#endif

#endif /* _FREEWILI_TUSB_CONFIG_H_ */

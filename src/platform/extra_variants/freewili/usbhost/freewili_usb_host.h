// FreeWili 2 DISPLAY RP2350 — native USB host bring-up (Task 3 spike)
//
// Runs the RP2350 native USB controller (GPIO66/67, roothub port 0) as a
// TinyUSB HOST via arduino-pico's Adafruit_TinyUSB stack (USE_TINYUSB +
// USE_TINYUSB_HOST). Enumerates the Intrepid u-blox M8 GPS (VID 1546 /
// PID 01A8, native USB-CDC) plugged into a USB-A Host port (behind the CH334F
// hub) and logs its VID/PID and raw NMEA.
//
// This is a transport spike: it proves the host path before the GPS is wired
// into NodeDB/the map. See .superpowers/sdd/task-3-brief.md.

#ifndef FREEWILI_USB_HOST_H_
#define FREEWILI_USB_HOST_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the native USB host stack (tuh_init on roothub port 0).
// Call once, after board/LoRa init (from lateInitVariant()).
void freewiliUsbHostInit(void);

// Poll the host stack + drain any mounted CDC device. Must be called
// cooperatively and frequently from the main loop (freewiliLoopHook()).
void freewiliUsbHostService(void);

// GDB-observable state (the console is a no-op in host mode, so on-hardware
// verification reads these directly).
extern volatile uint16_t g_freewili_gps_vid;
extern volatile uint16_t g_freewili_gps_pid;
extern volatile uint32_t g_freewili_gps_mount_count;
extern volatile uint32_t g_freewili_gps_nmea_bytes;

#ifdef __cplusplus
}
#endif

#endif // FREEWILI_USB_HOST_H_

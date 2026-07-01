// FreeWili 2 — USB *device* no-op stubs for native-host mode (Task 3 spike).
//
// With USE_TINYUSB + USE_TINYUSB_HOST the native controller becomes a HOST.
// The arduino-pico core (cores/rp2040/main.cpp at boot, delay.cpp yield() every
// loop) still calls the TinyUSB device entry points TinyUSB_Device_Init(),
// TinyUSB_Device_Task() and TinyUSB_Device_FlushCDC(). In host mode the
// Adafruit library compiles those out (its Adafruit_TinyUSB_API.cpp sees
// CFG_TUD_ENABLED==0), and the core only *weakly* declares them — so an
// undefined weak reference would resolve to address 0 and hard-fault at boot.
// These no-ops bind those references to real (do-nothing) code.
//
// NOTE: the `Serial` object (Adafruit_USBD_CDC) and all its I/O methods are
// still provided by the library itself (Adafruit_USBD_CDC.cpp is compiled with
// the device stack ON due to arduino-pico's include-path precedence for
// tusb_config.h), so they are intentionally NOT stubbed here. `Serial` is inert
// at runtime because the device stack is never tud_init()'d. Losing the
// USB-serial console is expected for this spike.
//
// This file is compiled ONLY for the FreeWili host build.

#include "configuration.h"

#if defined(FREEWILI) && defined(USE_TINYUSB_HOST)

#include <stdint.h>

extern "C" {
void TinyUSB_Device_Init(uint8_t rhport)
{
    (void)rhport; // native controller runs as host; nothing to do here
}
void TinyUSB_Device_Task(void) {}
void TinyUSB_Device_FlushCDC(void) {}
}

#endif // FREEWILI && USE_TINYUSB_HOST

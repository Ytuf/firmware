#pragma once

// USB-GPS position injector (Task 2 of the USB-GPS plan). Feed raw NMEA bytes
// from any transport (USB-CDC host today, UART tap as fallback); on each fresh
// valid fix the parsed meshtastic_Position is injected straight into NodeDB
// (setLocalPosition + updatePosition RX_SRC_LOCAL), bypassing the SerialUART-
// bound GPS class. The map/broadcast pipeline picks it up from there.

#include <stddef.h>
#include <stdint.h>

/// Feed raw NMEA bytes into the file-static TinyGPSPlus reader. When a fresh
/// valid fix is available (throttled to one injection per 2 s), builds a
/// Position via freewiliGpsParse(), seeds the RTC from the GPS date/time, and
/// injects into NodeDB. Safe to call with any chunking, from the main loop.
void freewiliGpsFeed(const uint8_t *buf, size_t len);

/// True once at least one fix has been injected this boot (cleared on GPS
/// unplug by the caller via freewiliGpsResetFix, so the UI can degrade).
bool freewiliGpsHasFix();

/// Forget the fix (e.g. on CDC unmount) so status reflects reality.
void freewiliGpsResetFix();

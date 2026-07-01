// FreeWili 2 — TinyUSB debug log -> RAM ring buffer (Task 3 spike diagnostics).
//
// The native controller is a HOST, so there is no USB-serial console to print
// to. tusb_config_freewili.h defines CFG_TUSB_DEBUG_PRINTF (whose definition
// lives in Adafruit_TinyUSB_API.cpp) and points its SERIAL_TUSB_DEBUG sink at
// this ring. Dump over SWD: OpenOCD `dump_image` on g_freewili_tusb_log, then
// rotate at g_freewili_tusb_log_head if total > size. Debug plumbing only.

#include "configuration.h"

#if defined(FREEWILI) && defined(USE_TINYUSB_HOST)

#include "tusb_config_freewili.h"
#include <stdint.h>
#include <string.h>

#define FWLOG_SIZE 8192

// GDB/SWD-observable ring buffer.
volatile char g_freewili_tusb_log[FWLOG_SIZE];
volatile uint32_t g_freewili_tusb_log_head = 0;
volatile uint32_t g_freewili_tusb_log_total = 0;

size_t FreeWiliTusbRingLog::write(const char *s)
{
    size_t n = strlen(s);
    uint32_t h = g_freewili_tusb_log_head;
    for (size_t i = 0; i < n; i++)
        g_freewili_tusb_log[(h + i) % FWLOG_SIZE] = s[i];
    g_freewili_tusb_log_head = (uint32_t)((h + n) % FWLOG_SIZE);
    g_freewili_tusb_log_total += (uint32_t)n;
    return n;
}

FreeWiliTusbRingLog g_freewiliTusbLog;

#endif // FREEWILI && USE_TINYUSB_HOST

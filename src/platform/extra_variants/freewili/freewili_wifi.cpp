// FreeWili WiFi survey (wardrive) — Display side. See freewili_wifi.h.
//
// The Display drives the whole survey itself over wilibsp: it periodically
// calls ow_wireless_wifi_on_scan_for_access_points() to kick a scan (the ESP
// scan is one-shot, so this repeats on a timer), then drains the resulting
// events. MAIN reports each discovered WiFi AP to the Display as a OneWili
// text event (fwMenuWifi.cpp addEventData("wifiscan", ...)), framed as:
//   [*wifiscan <ts> <seq> <bssid> <rssi> <channel> <band> <authmode> <ssid> <ok>]
// over the shared UART0 FwGUI link (opcode 0x5D). This replaces the old
// bespoke "0xFA 0xCE" relay, which MAIN no longer sends. wilibsp's
// ow_open_fwgui()/ow_poll_text_line() own the link; this file polls for
// "wifiscan" events, parses each with parseWifiscanEvent (freewili_wifi_parse.*,
// host-testable), and keeps the same AP table the survey screen already reads
// via freewiliWifiGetAps().

#include "configuration.h"
#include "freewili_wifi.h"

#if defined(FREEWILI)

#include "concurrency/OSThread.h"
#include "freewili_sd.h"
#include "freewili_wifi_parse.h"
#include "onewili.h"
#include "onewili_fwgui.h"
#include <Arduino.h>
#include <string.h>

// MAIN talks to the Display over the hardware UART0 at 8 Mbaud (its Intrepid GUI
// protocol). wilibsp's ow_open_fwgui() mirrors the shipping Intrepid Display
// firmware's setup (RP2350 hardware uart0, not a bit-banged PIO UART --
// arduino-pico's SerialPIO cannot sample/emit a clean 8 Mbaud frame), and also
// brings up GPIO2/3 as hardware CTS/RTS (see freewili_wifi.h Step 5 decision).
// Reaching 8 Mbaud on uart0 requires clk_peri >= 128 MHz; freewili_qmi.cpp
// raises clk_peri to sys_clk (240 MHz) for exactly this.

// SWD-observable.
volatile uint32_t g_freewili_wifi_frames = 0;   // valid wifiscan events parsed
volatile uint32_t g_freewili_wifi_ap_count = 0; // live APs in the table
volatile int32_t g_freewili_wifi_last_rssi = 0;
volatile uint32_t g_freewili_wifi_uart_ok = 0;  // 1 = ow_open_fwgui succeeded
volatile uint32_t g_freewili_wifi_rx_drops = 0; // wilibsp stream FIFO drops (ow_fwgui_dropped_frames)
volatile int32_t g_freewili_wifi_scan_status = 0; // ow_status of the last scan-kick call

// The ESP scan is one-shot per command, so re-kick it on a timer to keep the survey filling.
static constexpr uint32_t FW_WIFI_SCAN_INTERVAL_MS = 5000;

static ow_device s_owDev;
static FreewiliWifiAp s_aps[FW_WIFI_MAX_APS];

static bool bssidEq(const uint8_t *a, const uint8_t *b)
{
    for (int i = 0; i < 6; i++)
        if (a[i] != b[i])
            return false;
    return true;
}

static uint32_t liveApCount()
{
    uint32_t n = 0;
    for (int i = 0; i < FW_WIFI_MAX_APS; i++)
        if (s_aps[i].used)
            n++;
    return n;
}

// Insert or refresh an AP keyed by BSSID. When full, evict the oldest-seen.
static void upsertAp(const FreewiliWifiAp &parsed, uint32_t now)
{
    int slot = -1, freeSlot = -1, oldest = -1;
    uint32_t oldestMs = 0xFFFFFFFFu;
    for (int i = 0; i < FW_WIFI_MAX_APS; i++) {
        if (s_aps[i].used) {
            if (bssidEq(s_aps[i].bssid, parsed.bssid)) {
                slot = i;
                break;
            }
            if (s_aps[i].lastSeenMs < oldestMs) {
                oldestMs = s_aps[i].lastSeenMs;
                oldest = i;
            }
        } else if (freeSlot < 0) {
            freeSlot = i;
        }
    }
    // The only point that still knows this BSSID was not already in the table.
    // Hooking the survey log anywhere downstream would re-append every AP on
    // every scan, forever.
    const bool isNew = (slot < 0);
    if (slot < 0)
        slot = (freeSlot >= 0) ? freeSlot : oldest;
    if (slot < 0)
        return;

    FreewiliWifiAp &ap = s_aps[slot];
    memcpy(ap.bssid, parsed.bssid, sizeof(ap.bssid));
    ap.rssi = parsed.rssi;
    ap.channel = parsed.channel;
    ap.band = parsed.band;
    ap.authmode = parsed.authmode;
    memcpy(ap.ssid, parsed.ssid, sizeof(ap.ssid));
    ap.lastSeenMs = now;
    ap.used = true;

    g_freewili_wifi_frames++;
    g_freewili_wifi_last_rssi = ap.rssi;
    g_freewili_wifi_ap_count = liveApCount();

    if (isNew)
        freewili_sd_note_ap(ap);   // staging memcpy only; the flush is in runOnce
}

size_t freewiliWifiGetAps(FreewiliWifiAp *out, size_t max)
{
    size_t n = 0;
    for (int i = 0; i < FW_WIFI_MAX_APS && n < max; i++)
        if (s_aps[i].used)
            out[n++] = s_aps[i];
    return n;
}

#ifdef FW_WIFI_TEST_INJECT
// A few fake APs so the survey screen can be verified without the MAIN link.
static void injectTestAps()
{
    struct {
        const char *ssid;
        int8_t rssi;
        uint8_t ch, band;
    } fake[] = {
        {"DEFCON-Open", -42, 6, 0},   {"linksys", -67, 11, 0},
        {"Pineapple_5G", -55, 36, 1}, {"xfinitywifi", -81, 1, 0},
        {"hunter2", -73, 44, 1},
    };
    uint32_t now = millis();
    for (unsigned k = 0; k < sizeof(fake) / sizeof(fake[0]); k++) {
        FreewiliWifiAp ap = {};
        for (int i = 0; i < 6; i++)
            ap.bssid[i] = (uint8_t)(0x10 * (k + 1) + i);
        ap.rssi = fake[k].rssi;
        ap.channel = fake[k].ch;
        ap.band = fake[k].band;
        ap.authmode = 3;
        strncpy(ap.ssid, fake[k].ssid, FW_WIFI_SSID_LEN - 1);
        upsertAp(ap, now);
    }
}
#endif

namespace
{
class FreewiliWifi : public concurrency::OSThread
{
  public:
    FreewiliWifi() : concurrency::OSThread("FWWifi") {}

    void begin()
    {
        ow_status st = ow_open_fwgui(&s_owDev);
        g_freewili_wifi_uart_ok = (st == OW_OK) ? 1u : 0u;
#ifdef FW_WIFI_TEST_INJECT
        injectTestAps();
#endif
        LOG_INFO("FreeWili WiFi survey: OneWili fwgui link open, status=%d", (int)st);
    }

  protected:
    int32_t runOnce() override
    {
        if (g_freewili_wifi_uart_ok && millis() >= m_nextScanMs) {
            // Fire-and-forget: never block this cooperative thread waiting for MAIN's
            // ack. The scan result arrives as async [*wifiscan] events. "w\\w\\s" is the
            // wireless>wifi>scan menu path (ow_wireless_wifi_on_scan_for_access_points).
            g_freewili_wifi_scan_status = (int32_t)ow_send_cmd_noreply(&s_owDev, "w\\w\\s");
            m_nextScanMs = millis() + FW_WIFI_SCAN_INTERVAL_MS;
        }

        char id[32];
        char args[200];
        // Cap events per tick. MAIN streams the FwGUI link continuously, so an
        // unbounded drain never returns and starves the cooperative scheduler
        // (freezes the whole UI). 32 covers a full scan's APs with margin; the
        // rest is picked up on the next tick.
        int budget = 32;
        while (budget-- > 0 && ow_poll_text_line(&s_owDev, id, sizeof(id), args, sizeof(args)) == 1) {
            if (strcmp(id, "wifiscan") != 0)
                continue; // not our event -- some other menu's event, ignore
            FreewiliWifiAp parsed{};
            if (parseWifiscanEvent(args, parsed))
                upsertAp(parsed, millis());
        }
        g_freewili_wifi_rx_drops = ow_fwgui_dropped_frames();
        // Only place that may block on the SDFS link: the event queue above is
        // drained first, so nothing is half-received while we round-trip.
        freewili_sd_service();
        return 20;
    }

  private:
    uint32_t m_nextScanMs = 0; // fires immediately on the first tick
};

FreewiliWifi *s_wifi = nullptr;
} // namespace

void freewili_register_wifi()
{
    if (s_wifi)
        return;
    s_wifi = new FreewiliWifi();
    s_wifi->begin();
}

#else
void freewili_register_wifi() {}
size_t freewiliWifiGetAps(FreewiliWifiAp *, size_t)
{
    return 0;
}
#endif // FREEWILI

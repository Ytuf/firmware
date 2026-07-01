#include "freewili_gps.h"

#include "configuration.h"

#include "TinyGPS++.h"
#include "freewili_gps_parse.h"
#include "gps/RTC.h"
#include "mesh/NodeDB.h"

// GDB/SWD-observable state (no console in USB-host mode).
volatile uint32_t g_freewili_gps_fix_count = 0;
volatile int32_t g_freewili_gps_last_lat_i = 0;
volatile int32_t g_freewili_gps_last_lon_i = 0;

static TinyGPSPlus s_reader;
static bool s_hasFix = false;
static uint32_t s_lastInjectMs = 0;

void freewiliGpsFeed(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i)
        s_reader.encode((char)buf[i]);

    // Throttle the parse+inject step (not the byte feed) — the u-blox emits
    // 1 Hz bursts and NodeDB doesn't need every one. freewiliGpsParse() only
    // consumes TinyGPSPlus's "updated" flag when it actually runs, so a fix
    // arriving during the holdoff is picked up on the next pass.
    uint32_t now = millis();
    if (s_lastInjectMs != 0 && (now - s_lastInjectMs) < 2000)
        return;

    meshtastic_Position pos = meshtastic_Position_init_default;
    if (!freewiliGpsParse(s_reader, pos))
        return;
    s_lastInjectMs = now;

    // GPS time is authoritative — seed the RTC (also fixes the on-screen
    // clock on units with an unset MCP7940).
    if (s_reader.date.isValid() && s_reader.time.isValid()) {
        struct tm t;
        t.tm_sec = s_reader.time.second();
        t.tm_min = s_reader.time.minute();
        t.tm_hour = s_reader.time.hour();
        t.tm_mday = s_reader.date.day();
        t.tm_mon = s_reader.date.month() - 1;
        t.tm_year = s_reader.date.year() - 1900;
        t.tm_isdst = false;
        perhapsSetRTC(RTCQualityGPS, t);
    }
    pos.time = getValidTime(RTCQualityGPS, false);

    nodeDB->setLocalPosition(pos);
    nodeDB->updatePosition(nodeDB->getNodeNum(), pos, RX_SRC_LOCAL);

    s_hasFix = true;
    g_freewili_gps_fix_count++;
    g_freewili_gps_last_lat_i = pos.latitude_i;
    g_freewili_gps_last_lon_i = pos.longitude_i;
}

bool freewiliGpsHasFix()
{
    return s_hasFix;
}

void freewiliGpsResetFix()
{
    s_hasFix = false;
}

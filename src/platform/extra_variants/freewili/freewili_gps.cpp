#include "freewili_gps.h"

#include "configuration.h"

#include "GPSStatus.h"
#include "TinyGPS++.h"
#include "freewili_gps_parse.h"
#include "gps/RTC.h"
#include "mesh/NodeDB.h"

// GDB/SWD-observable state (no console in USB-host mode).
volatile uint32_t g_freewili_gps_fix_count = 0;
volatile int32_t g_freewili_gps_last_lat_i = 0;
volatile int32_t g_freewili_gps_last_lon_i = 0;

// Raw-NMEA tail: last bytes received from the GPS CDC, as a ring, for SWD
// inspection (dump as ASCII to read the actual $Gx sentences — fix quality,
// sats in view, RMC A/V status — without a console).
volatile uint32_t g_freewili_gps_nmea_head = 0;
char g_freewili_gps_nmea_tail[512];

static TinyGPSPlus s_reader;
static bool s_hasFix = false;
static uint32_t s_lastInjectMs = 0;

void freewiliGpsFeed(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        g_freewili_gps_nmea_tail[g_freewili_gps_nmea_head % sizeof(g_freewili_gps_nmea_tail)] = (char)buf[i];
        g_freewili_gps_nmea_head++;
        s_reader.encode((char)buf[i]);
    }

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

    // Also feed the standard GPS status object. The stock "Position"/compass
    // screen and the header sat-count are all #if HAS_GPS and read gpsStatus
    // (not nodeDB); with no real GPS driver on this board, nothing else ever
    // populates it. hasLock=true + isConnected=true makes updateStatus() stamp
    // lastFixMillis and notify the screen observers.
    meshtastic::GPSStatus status(true, true, false, pos);
    gpsStatus->updateStatus(&status);

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

// Static fallback position so the map/screens work indoors with no satellite
// lock (a bare GNSS often never fixes indoors — including at the DEFCON booth).
// A real fix, if one arrives, overrides this via freewiliGpsFeed(). Enable with
// -DFW_GPS_STATIC_FALLBACK; set the coords to the booth via
// -DFW_GPS_FALLBACK_LAT_I / _LON_I (1e-7 deg). Default = the lab's last fix.
#ifndef FW_GPS_FALLBACK_LAT_I
#define FW_GPS_FALLBACK_LAT_I 427819224 // 42.7819 N
#endif
#ifndef FW_GPS_FALLBACK_LON_I
#define FW_GPS_FALLBACK_LON_I (-830634938) // 83.0635 W
#endif

void freewiliGpsInjectFallback()
{
#ifdef FW_GPS_STATIC_FALLBACK
    if (s_hasFix) // a real fix already landed — don't stomp it
        return;

    meshtastic_Position pos = meshtastic_Position_init_default;
    pos.latitude_i = FW_GPS_FALLBACK_LAT_I;
    pos.has_latitude_i = true;
    pos.longitude_i = FW_GPS_FALLBACK_LON_I;
    pos.has_longitude_i = true;
    pos.location_source = meshtastic_Position_LocSource_LOC_MANUAL;
    pos.time = getValidTime(RTCQualityDevice, false);

    nodeDB->setLocalPosition(pos);
    nodeDB->updatePosition(nodeDB->getNodeNum(), pos, RX_SRC_LOCAL);
    meshtastic::GPSStatus status(true, true, false, pos);
    gpsStatus->updateStatus(&status);

    g_freewili_gps_last_lat_i = pos.latitude_i;
    g_freewili_gps_last_lon_i = pos.longitude_i;
#endif
}

#include "freewili_gps_parse.h"

// gm_mktime() is a portable (no-timezone, no-libc-mktime) UTC struct-tm -> epoch-seconds
// helper already used by src/gps/GPS.cpp for the exact same NMEA date/time -> timestamp
// conversion; reuse it here instead of duplicating the calendar math.
#include "RTC.h"

bool freewiliGpsParse(TinyGPSPlus &reader, meshtastic_Position &out)
{
    if (!(reader.location.isValid() && reader.location.isUpdated()))
        return false;

    out.latitude_i = (int32_t)(reader.location.lat() * 1e7);
    out.has_latitude_i = true;
    out.longitude_i = (int32_t)(reader.location.lng() * 1e7);
    out.has_longitude_i = true;

    if (reader.altitude.isValid()) {
        out.altitude = (int32_t)reader.altitude.meters();
        out.has_altitude = true;
    }

    if (reader.satellites.isValid()) {
        out.sats_in_view = (uint32_t)reader.satellites.value();
    }

    if (reader.date.isValid() && reader.time.isValid()) {
        struct tm t;
        t.tm_sec = reader.time.second();
        t.tm_min = reader.time.minute();
        t.tm_hour = reader.time.hour();
        t.tm_mday = reader.date.day();
        t.tm_mon = reader.date.month() - 1;
        t.tm_year = reader.date.year() - 1900;
        t.tm_isdst = false;
        out.timestamp = (uint32_t)gm_mktime(&t);
    }

    out.location_source = meshtastic_Position_LocSource_LOC_EXTERNAL;

    return true;
}

#pragma once

// Pure, transport-independent NMEA -> meshtastic_Position transform for the FreeWili USB GPS
// feature. No USB I/O, no NodeDB, no hardware access here -- this file only touches a
// TinyGPSPlus reader that the caller has already fed with encode() and a Position struct to
// fill in, so it can be exercised directly by the native/portduino unit tests.

#include "TinyGPS++.h"
#include <meshtastic/mesh.pb.h>

/// Turns the current state of `reader` (after the caller has fed it NMEA bytes via
/// `reader.encode()`) into `out`.
///
/// Returns true only when `reader.location` is both valid and updated, i.e. there is a fresh
/// fix since the reader's location was last consumed. On success, fills:
///   - latitude_i / longitude_i (+ has_latitude_i / has_longitude_i)
///   - altitude (+ has_altitude) when reader.altitude is valid
///   - sats_in_view when reader.satellites is valid
///   - timestamp, derived from reader.date/reader.time when both are valid
///   - location_source = meshtastic_Position_LocSource_LOC_EXTERNAL
///
/// On failure (no fresh valid fix), `out` is left untouched and false is returned.
bool freewiliGpsParse(TinyGPSPlus &reader, meshtastic_Position &out);

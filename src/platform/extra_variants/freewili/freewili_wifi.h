#pragma once

// FreeWili WiFi survey (wardrive) — Display side.
//
// The ESP32-C5 (MAIN-owned) scans WiFi via Intrepid's "bottlenose" firmware;
// MAIN forwards each discovered AP to the Display over UART0 (Display GPIO0/1 =
// Serial1, crossed to MAIN). This module reads that link, keeps a table of
// heard APs, and exposes it to the survey screen. The wire payload is the ESP's
// native bnose_wifi_scan_record passed through unchanged (see freewili_wifi.cpp
// for the framing).

#include <stdint.h>
#include <stddef.h>

#define FW_WIFI_SSID_LEN 33 // matches bnose_wifi_scan_record.ssid (32 + NUL)
#define FW_WIFI_MAX_APS 48

// One access point. Mirrors the ESP bottlenose scan record + a local timestamp.
struct FreewiliWifiAp {
    uint8_t bssid[6];
    int8_t rssi;   // dBm (signed)
    uint8_t channel;
    uint8_t band;  // 0 = 2.4 GHz, 1 = 5 GHz
    uint8_t authmode;
    char ssid[FW_WIFI_SSID_LEN];
    uint32_t lastSeenMs;
    bool used;
};

// Start the UART reader OSThread (Serial1 <- MAIN). Call once at setup.
void freewili_register_wifi();

// Copy up to `max` live APs into `out` (unsorted). Returns the count copied.
// Safe to call from the screen thread.
size_t freewiliWifiGetAps(FreewiliWifiAp *out, size_t max);

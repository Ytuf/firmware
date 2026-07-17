#pragma once

// FreeWili WiFi survey (wardrive) — Display side.
//
// The ESP32-C5 (MAIN-owned) scans WiFi via Intrepid's "bottlenose" firmware;
// MAIN reports each discovered AP to the Display as a OneWili text event,
// "[*wifiscan ...]", over the shared UART0 FwGUI link (see freewili_wifi.cpp
// for the exact framing and how it's consumed via wilibsp).
//
// RTS/CTS decision (plan Phase 1 Step 5): wired ON. This build's UART0 FwGUI
// link now runs with hardware flow control on GPIO2 (CTS) / GPIO3 (RTS), via
// wilibsp's unmodified ow_open_fwgui(). Evidence GPIO2/3 are actually free on
// this variant, despite being labeled LORA_SCK/LORA_MOSI:
//   - variants/rp2350/freewili/variant.h: LORA_SCK/LORA_MOSI/LORA_MISO/LORA_CS
//     are documented as "Dummy ... pins to satisfy Meshtastic's unconditional
//     SPI init block" — this variant's LoRa radio rides UART1 (WIO-E5), not SPI.
//   - src/main.cpp, the RP2040/RP2350 LoRa-SPI bring-up block: the
//     `#if defined(FREEWILI)` arm is empty ("FreeWili uses UART radio — no SPI
//     LoRa. SPI1 is managed by the display driver.") — GPIO2/3 are never
//     pinMode'd/gpio_set_function'd for SPI on this variant. A repo-wide grep
//     for pinMode/gpio_init/gpio_set_function on pins 2/3 under
//     extra_variants/freewili and variants/rp2350/freewili turns up nothing.
//   - MAIN's peer of this exact link is already flow-controlled: FW2Main's
//     obDisplayUART.init() (targets/fw2main/Fw2MainAppSetup.cpp) passes real
//     UART0_CTS/UART0_RTS pins, and rpSerialComm.cpp turns a non-negative
//     iCtsPin/iRtsPin into uart_set_hw_flow(uart0, true, true) — MAIN is
//     already running this link with flow control on, so enabling it here
//     closes the loop rather than introducing new behavior.
// The stale claim in an earlier revision of this file (GPIO2/3 "repurposed as
// the LoRa SPI1 SCK/MOSI" and therefore unavailable) was never checked against
// the runtime init path above; it did not hold for this variant. Phase 4 (SD)
// can rely on RTS/CTS already being live on this link.

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

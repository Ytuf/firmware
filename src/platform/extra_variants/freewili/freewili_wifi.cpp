// FreeWili WiFi survey (wardrive) — Display side. See freewili_wifi.h.
//
// Reads WiFi AP records forwarded by the MAIN CPU over UART0. The UART0 Tx/Rx
// functions are CROSSED between the two CPUs (see agents/hardware/pinouts.md):
// on the DISPLAY, GPIO0 = UART0_Rx (from MAIN) and GPIO1 = UART0_Tx (to MAIN),
// which is the mirror of MAIN's GPIO0=Tx / GPIO1=Rx. Because GPIO0's hardware-
// UART function is TX (not RX), the hardware uart0 can't receive on it, so this
// link uses a bit-banged PIO UART (SerialPIO), which drives RX/TX on any pin.
// MAIN drives the ESP32-C5 scan
// via rpBottleNoseOrca and forwards each bnose_wifi_scan_record unchanged inside
// a small frame:
//
//   0xFA 0xCE | type(1) | len(1) | payload[len] | cksum(1)
//   type 1 = WIFI_AP, payload = the 43-byte ESP record
//   cksum  = (type + len + sum(payload)) & 0xFF
//
// The parser hunts for the 0xFA 0xCE sync, so any unrelated bytes on the shared
// UART (Intrepid heartbeats, noise) are skipped harmlessly.

#include "configuration.h"
#include "freewili_wifi.h"

#if defined(FREEWILI)

#include "concurrency/OSThread.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include <Arduino.h>

// MAIN talks to the Display over the hardware UART0 at 8 Mbaud (its Intrepid GUI
// protocol). That is exactly how the shipping Intrepid Display firmware does this
// link (freewilidisplay: uart_init(uart0, 8'000'000), 8N1), so we mirror it with
// the RP2350 hardware uart0 rather than a bit-banged PIO UART — arduino-pico's
// SerialPIO cannot sample/emit a clean 8 Mbaud frame. Reaching 8 Mbaud on uart0
// requires clk_peri >= 128 MHz; freewili_qmi.cpp raises clk_peri to sys_clk
// (240 MHz) for exactly this. The peripheral owns the pin DIRECTION: setting
// GPIO0 and GPIO1 to GPIO_FUNC_UART makes GPIO0 = uart0 TX and GPIO1 = uart0 RX
// (fixed RP2350 function map), matching MAIN across the board's crossed wiring.
//
// No RTS/CTS: on this Meshtastic build GPIO2/3 (the UART0 CTS/RTS pins) are
// repurposed as the LoRa SPI1 SCK/MOSI (variant.h LORA_SCK=2 / LORA_MOSI=3), so
// hardware flow control is unavailable. The relay traffic is tiny (a 6-byte
// enable out; low-rate AP frames in after MAIN pauses its GUI), so the 32-byte
// FIFO is adequate without it.
#ifndef FW_WIFI_UART_BAUD
#define FW_WIFI_UART_BAUD 8000000
#endif
#define FW_WIFI_UART uart0
#define FW_WIFI_TX_PIN 0 // GPIO0 -> uart0 TX (to MAIN across the crossed link)
#define FW_WIFI_RX_PIN 1 // GPIO1 -> uart0 RX (from MAIN)
#define FW_WIFI_ENABLE_PERIOD_MS 2000

// Wire framing.
static const uint8_t WIFI_SYNC1 = 0xFA;
static const uint8_t WIFI_SYNC2 = 0xCE;
static const uint8_t WIFI_TYPE_AP = 1;     // MAIN -> Display: one AP record
static const uint8_t WIFI_TYPE_ENABLE = 2; // Display -> MAIN: start/refresh the relay
static const int WIFI_REC_LEN = 43;        // sizeof bnose_wifi_scan_record

// Offsets within the 43-byte record.
enum { REC_BSSID = 0, REC_RSSI = 6, REC_CHAN = 7, REC_BAND = 8, REC_AUTH = 9, REC_SSID = 10 };

// SWD-observable.
volatile uint32_t g_freewili_wifi_frames = 0;   // valid AP frames decoded
volatile uint32_t g_freewili_wifi_rx_bytes = 0;  // raw UART bytes seen
volatile uint32_t g_freewili_wifi_ap_count = 0;  // live APs in the table
volatile int32_t g_freewili_wifi_last_rssi = 0;
volatile uint32_t g_freewili_wifi_uart_ok = 0;   // 1 = uart0 reached ~8 Mbaud (clk_peri raised)
volatile uint32_t g_freewili_wifi_tx_sends = 0;  // enable frames transmitted
volatile uint32_t g_freewili_wifi_rx_drops = 0;  // bytes dropped (ring full)

// RX interrupt + ring buffer. This link has no hardware flow control (GPIO2/3
// are the LoRa SPI here), so at 8 Mbaud a burst of forwarded AP frames overruns
// the 32-byte uart0 FIFO between the ~20 ms runOnce polls. The RX ISR keeps the
// FIFO drained into this ring; runOnce decodes from the ring at its own pace.
#define FW_WIFI_RING_SZ 2048u // power of two
static volatile uint8_t s_rxRing[FW_WIFI_RING_SZ];
static volatile uint32_t s_rxHead = 0; // written by ISR
static volatile uint32_t s_rxTail = 0; // read by runOnce

static void __not_in_flash_func(fwWifiRxIsr)()
{
    while (uart_is_readable(FW_WIFI_UART)) {
        uint8_t b = (uint8_t)uart_getc(FW_WIFI_UART);
        uint32_t nh = (s_rxHead + 1u) & (FW_WIFI_RING_SZ - 1u);
        if (nh != s_rxTail) {
            s_rxRing[s_rxHead] = b;
            s_rxHead = nh;
        } else {
            g_freewili_wifi_rx_drops++; // ring full (should not happen at this drain rate)
        }
    }
}

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
static void upsertAp(const uint8_t *rec, uint32_t now)
{
    int slot = -1, freeSlot = -1, oldest = -1;
    uint32_t oldestMs = 0xFFFFFFFFu;
    for (int i = 0; i < FW_WIFI_MAX_APS; i++) {
        if (s_aps[i].used) {
            if (bssidEq(s_aps[i].bssid, rec + REC_BSSID)) {
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
    if (slot < 0)
        slot = (freeSlot >= 0) ? freeSlot : oldest;
    if (slot < 0)
        return;

    FreewiliWifiAp &ap = s_aps[slot];
    for (int i = 0; i < 6; i++)
        ap.bssid[i] = rec[REC_BSSID + i];
    ap.rssi = (int8_t)rec[REC_RSSI];
    ap.channel = rec[REC_CHAN];
    ap.band = rec[REC_BAND];
    ap.authmode = rec[REC_AUTH];
    for (int i = 0; i < FW_WIFI_SSID_LEN - 1; i++)
        ap.ssid[i] = rec[REC_SSID + i];
    ap.ssid[FW_WIFI_SSID_LEN - 1] = 0;
    ap.lastSeenMs = now;
    ap.used = true;

    g_freewili_wifi_frames++;
    g_freewili_wifi_last_rssi = ap.rssi;
    g_freewili_wifi_ap_count = liveApCount();
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
        uint8_t rec[WIFI_REC_LEN] = {0};
        for (int i = 0; i < 6; i++)
            rec[REC_BSSID + i] = (uint8_t)(0x10 * (k + 1) + i);
        rec[REC_RSSI] = (uint8_t)fake[k].rssi;
        rec[REC_CHAN] = fake[k].ch;
        rec[REC_BAND] = fake[k].band;
        rec[REC_AUTH] = 3;
        strncpy((char *)&rec[REC_SSID], fake[k].ssid, FW_WIFI_SSID_LEN - 1);
        upsertAp(rec, now);
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
        // Hardware uart0, mirroring the shipping Intrepid Display firmware. Setting
        // GPIO0/1 to GPIO_FUNC_UART fixes GPIO0 = uart0 TX, GPIO1 = uart0 RX.
        uint32_t actual = uart_init(FW_WIFI_UART, FW_WIFI_UART_BAUD);
        gpio_set_function(FW_WIFI_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(FW_WIFI_RX_PIN, GPIO_FUNC_UART);
        uart_set_hw_flow(FW_WIFI_UART, false, false); // GPIO2/3 are LoRa SPI here
        uart_set_format(FW_WIFI_UART, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(FW_WIFI_UART, true);
        // RX ISR -> ring (see notes at s_rxRing). RX + RX-timeout IRQ, no TX IRQ.
        irq_set_exclusive_handler(UART0_IRQ, fwWifiRxIsr);
        irq_set_enabled(UART0_IRQ, true);
        uart_set_irq_enables(FW_WIFI_UART, true, false);
        // actual baud within a few % of 8 Mbaud confirms clk_peri was raised; if
        // clk_peri were still 48 MHz uart_init would clamp far below 8 Mbaud.
        g_freewili_wifi_uart_ok = (actual >= 7000000u && actual <= 9000000u) ? 1u : 0u;
#ifdef FW_WIFI_TEST_INJECT
        injectTestAps();
#endif
        LOG_INFO("FreeWili WiFi survey: hardware uart0 @ %u (asked %d) on GPIO%d/%d ok=%u", (unsigned)actual,
                 FW_WIFI_UART_BAUD, FW_WIFI_TX_PIN, FW_WIFI_RX_PIN, (unsigned)g_freewili_wifi_uart_ok);
    }

  protected:
    int32_t runOnce() override
    {
        // Re-send the enable request periodically so MAIN starts (or restarts)
        // the relay whenever it comes up. MAIN only acts on this request, so the
        // real Display firmware path is never affected.
        uint32_t now = millis();
        if (now - m_lastEnableMs >= FW_WIFI_ENABLE_PERIOD_MS) {
            m_lastEnableMs = now;
            sendEnable();
        }
        while (s_rxTail != s_rxHead) {
            uint8_t b = s_rxRing[s_rxTail];
            s_rxTail = (s_rxTail + 1u) & (FW_WIFI_RING_SZ - 1u);
            g_freewili_wifi_rx_bytes++;
            feed(b);
        }
        return 20;
    }

  private:
    uint32_t m_lastEnableMs = 0;

    void sendEnable()
    {
        // 0xFA 0xCE | type | len=4 | 4-byte magic | cksum. The magic is high-entropy
        // so MAIN's byte-stream enable detector can't be falsely triggered by
        // ordinary GUI traffic. Must match MAIN (fwWifiRelay.cpp WIFI_ENABLE_MAGIC).
        uint8_t f[9];
        f[0] = WIFI_SYNC1;
        f[1] = WIFI_SYNC2;
        f[2] = WIFI_TYPE_ENABLE;
        f[3] = 4; // len
        f[4] = 0x57;
        f[5] = 0xA7;
        f[6] = 0x2C;
        f[7] = 0x6B;
        f[8] = (uint8_t)(f[2] + f[3] + f[4] + f[5] + f[6] + f[7]);
        uart_write_blocking(FW_WIFI_UART, f, sizeof(f));
        g_freewili_wifi_tx_sends++;
    }

    enum { WAIT_S1, WAIT_S2, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CKSUM } m_st = WAIT_S1;
    uint8_t m_type = 0, m_len = 0, m_sum = 0, m_idx = 0;
    uint8_t m_buf[64];

    void feed(uint8_t b)
    {
        switch (m_st) {
        case WAIT_S1:
            if (b == WIFI_SYNC1)
                m_st = WAIT_S2;
            break;
        case WAIT_S2:
            m_st = (b == WIFI_SYNC2) ? WAIT_TYPE : (b == WIFI_SYNC1 ? WAIT_S2 : WAIT_S1);
            break;
        case WAIT_TYPE:
            m_type = b;
            m_sum = b;
            m_st = WAIT_LEN;
            break;
        case WAIT_LEN:
            m_len = b;
            m_sum += b;
            m_idx = 0;
            if (m_len == 0 || m_len > sizeof(m_buf))
                m_st = WAIT_S1; // implausible
            else
                m_st = WAIT_PAYLOAD;
            break;
        case WAIT_PAYLOAD:
            m_buf[m_idx++] = b;
            m_sum += b;
            if (m_idx >= m_len)
                m_st = WAIT_CKSUM;
            break;
        case WAIT_CKSUM:
            if (b == m_sum && m_type == WIFI_TYPE_AP && m_len == WIFI_REC_LEN)
                upsertAp(m_buf, millis());
            m_st = WAIT_S1;
            break;
        }
    }
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

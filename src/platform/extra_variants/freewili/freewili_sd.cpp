// FreeWili wardrive -> SD card (Display side). See freewili_sd.h.

#include "configuration.h"
#include "freewili_sd.h"

#if defined(FREEWILI)

#include "onewili_fwgui.h"
#include "sdfs_client.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

// Wire tunables come from platformio.ini -D flags, not from sdfs_wire.h's
// defaults (512/255). A mismatch with MAIN is silent on the wire and corrupts
// every transfer, so fail the build instead of discovering it on hardware.
static_assert(SDFS_MAX_PAYLOAD == 96, "SDFS_MAX_PAYLOAD must match MAIN (sdfslib/CMakeLists.txt)");
static_assert(SDFS_MAX_PATH == 128, "SDFS_MAX_PATH must match MAIN (sdfslib/CMakeLists.txt)");
static_assert(SDFS_MAX_FRAME == 235, "SDFS_MAX_FRAME must be 11+128+96 to match MAIN");

#define FWSD_DIR    "/wardrive"
#define FWSD_PATH   "/wardrive/survey.csv"

// Staging buffer for rows not yet on the card. 48 APs * ~72B fits with margin.
#define FWSD_STAGE_MAX 4096
// Flush once this much is queued, or after FWSD_FLUSH_MS with anything queued.
#define FWSD_FLUSH_BYTES 512
#define FWSD_FLUSH_MS    15000
// Bytes pushed per blocking pace round-trip. 4 chunks + the seek = 5 frames,
// inside MAIN's 8-slot inbound ring (serial_comm_main.c SDFS_RX_SLOTS).
#define FWSD_PACE (4u * (uint32_t)(SDFS_MAX_PAYLOAD - 1u))
// sdfs_client's blocking ops are unyielding spins, so this bounds how long a
// lost reply can stall the cooperative scheduler. One poll is ~300ns (a DMA
// transfer_count read plus the RTS guard), and a round trip costs MAIN one or
// two ~62 Hz loop passes (16-32ms), so this must be well above ~110k polls --
// 30000 (~9ms) timed out on every single operation on hardware. ~700k is about
// 200ms: comfortably longer than a healthy round trip, still bounded.
#define FWSD_TIMEOUT_POLLS 700000

// SWD-observable, __attribute__((used)) because this build is -Os.
#define FWSD_DIAG __attribute__((used))
FWSD_DIAG volatile uint32_t g_fwsd_rows_queued = 0;   // rows staged
FWSD_DIAG volatile uint32_t g_fwsd_rows_written = 0;  // rows confirmed on the card
FWSD_DIAG volatile uint32_t g_fwsd_bytes_written = 0; // payload bytes appended
FWSD_DIAG volatile uint32_t g_fwsd_flushes = 0;       // successful flushes
FWSD_DIAG volatile uint32_t g_fwsd_errors = 0;        // failed flushes
FWSD_DIAG volatile int32_t g_fwsd_last_status = -1;   // last sdfs_status_t seen
FWSD_DIAG volatile uint32_t g_fwsd_stage_drops = 0;   // rows dropped, staging full
FWSD_DIAG volatile uint32_t g_fwsd_file_size = 0;     // size reported by stat
FWSD_DIAG volatile int32_t g_fwsd_mkdir_status = -1;  // 0=created, 7=already existed
// Readback proof: after the first successful flush, read survey.csv back THROUGH
// MAIN's f_read (the physical card) into this SWD-observable buffer. Dumping
// g_fwsd_readbuf over SWD shows the actual on-card CSV bytes, defeating any
// "the Display just believes it wrote" doubt.
#define FWSD_READBUF_MAX 512
FWSD_DIAG volatile int32_t  g_fwsd_readback_status = -2; // -2=not tried; SDFS status otherwise
FWSD_DIAG volatile uint32_t g_fwsd_readback_len = 0;
FWSD_DIAG char              g_fwsd_readbuf[FWSD_READBUF_MAX] __attribute__((used));
static bool s_readbackDone = false;

static char s_stage[FWSD_STAGE_MAX];
static uint32_t s_stageLen = 0;
static uint32_t s_stageRows = 0;
static uint32_t s_lastFlushMs = 0;
static sdfs_client_t s_sd;
static bool s_sdReady = false;
static bool s_dirMade = false;

static void ensureClient()
{
    if (s_sdReady)
        return;
    static sdfs_transport_t tp;
    tp = ow_fwgui_sdfs_transport();
    sdfs_client_init(&s_sd, &tp);
    s_sd.timeout_polls = FWSD_TIMEOUT_POLLS;
    s_sdReady = true;
}

// Commas and newlines would break the CSV; SSIDs may legally contain both.
static void sanitize(char *p)
{
    for (; *p; p++)
        if (*p == ',' || *p == '\n' || *p == '\r')
            *p = '_';
}

void freewili_sd_note_ap(const FreewiliWifiAp &ap)
{
    char ssid[FW_WIFI_SSID_LEN];
    memcpy(ssid, ap.ssid, sizeof(ssid));
    ssid[sizeof(ssid) - 1] = 0;
    sanitize(ssid);

    char row[128];
    int n = snprintf(row, sizeof(row), "%02X:%02X:%02X:%02X:%02X:%02X,%d,%u,%u,%u,%lu,%s\n", ap.bssid[0],
                     ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5], (int)ap.rssi,
                     (unsigned)ap.channel, (unsigned)ap.band, (unsigned)ap.authmode,
                     (unsigned long)ap.lastSeenMs, ssid);
    if (n <= 0)
        return;
    if ((uint32_t)n > FWSD_STAGE_MAX - s_stageLen) {
        g_fwsd_stage_drops++;
        return;
    }
    memcpy(s_stage + s_stageLen, row, (size_t)n);
    s_stageLen += (uint32_t)n;
    s_stageRows++;
    g_fwsd_rows_queued = s_stageRows;
}

// One append pass: stat (to learn the append offset) -> open -> paced writes ->
// close with a byte-count check. Returns true if every byte is confirmed on the
// card. Handles are opened and closed inside this call so a failure can never
// strand one out of MAIN's 2-handle pool.
// A reply that arrives after its operation gave up would otherwise be consumed
// by the NEXT operation, which then sees a mismatched req_id, drops it, and
// times out too -- one late frame desyncs the lane permanently. Start every
// flush from a clean queue.
static void drainStaleFrames()
{
    static sdfs_transport_t tp;
    tp = ow_fwgui_sdfs_transport();
    uint8_t junk[SDFS_MAX_FRAME];
    size_t n = 0;
    for (int i = 0; i < 2 * 8; i++) // bounded by the lane's slot count
        if (tp.recv(tp.ctx, junk, sizeof(junk), &n) != 1)
            break;
}

static bool flushStage()
{
    ensureClient();
    drainStaleFrames();

    if (!s_dirMade) {
        // Best-effort: an existing directory comes back as SDFS_ERR_BAD_REQUEST
        // (host maps FatFs FR_EXIST -> BAD_REQUEST, sdfs_storage_fatfs.c map_fr),
        // which is the normal case on every boot after the first. Never fail the
        // flush on mkdir -- the open below is the real test.
        g_fwsd_mkdir_status = (int32_t)sdfs_mkdir(&s_sd, FWSD_DIR);
        s_dirMade = true;
    }

    // Append mode starts at EOF, which only the server knows; ask, so the pace
    // seeks below land on the true write position instead of overwriting.
    int isDir = 0;
    uint32_t size = 0;
    sdfs_status_t st = sdfs_stat(&s_sd, FWSD_PATH, &isDir, &size);
    if (st != SDFS_OK)
        size = 0; // not created yet
    g_fwsd_file_size = size;

    uint8_t h = 0xFF;
    st = sdfs_open(&s_sd, FWSD_PATH, 2 /*append*/, &h);
    g_fwsd_last_status = (int32_t)st;
    if (st != SDFS_OK || h == 0xFF)
        return false;

    uint32_t pos = size;
    uint32_t off = 0;
    bool ok = true;
    while (off < s_stageLen) {
        uint32_t take = s_stageLen - off;
        if (take > FWSD_PACE)
            take = FWSD_PACE;
        st = sdfs_hwrite(&s_sd, h, (const uint8_t *)s_stage + off, take);
        if (st != SDFS_OK) {
            ok = false;
            break;
        }
        off += take;
        pos += take;
        // Blocking pace: MAIN processes this after the chunks above, so its
        // inbound ring is drained before the next burst.
        st = sdfs_seek(&s_sd, h, pos);
        if (st != SDFS_OK) {
            ok = false;
            break;
        }
    }
    g_fwsd_last_status = (int32_t)st;

    sdfs_status_t cl = sdfs_hclose(&s_sd, h, ok ? off : SDFS_HCLOSE_NO_VERIFY);
    if (ok && cl != SDFS_OK) {
        ok = false;
        g_fwsd_last_status = (int32_t)cl;
    }
    if (!ok)
        return false;

    g_fwsd_bytes_written += off;
    g_fwsd_rows_written += s_stageRows;
    g_fwsd_flushes++;
    return true;
}

// Read survey.csv back off the card (through MAIN's SDFS server / f_read) into
// g_fwsd_readbuf, once. Proves the bytes are really on MAIN's filesystem, and
// exercises the SDFS read data plane (open mode 0 + hread).
static void doReadback()
{
    ensureClient();
    uint8_t h = 0xFF;
    sdfs_status_t st = sdfs_open(&s_sd, FWSD_PATH, 0 /*read*/, &h);
    if (st != SDFS_OK || h == 0xFF) {
        g_fwsd_readback_status = (int32_t)st;
        return;
    }
    uint32_t got = 0;
    st = sdfs_hread(&s_sd, h, (uint8_t *)g_fwsd_readbuf, FWSD_READBUF_MAX, &got);
    sdfs_hclose(&s_sd, h, SDFS_HCLOSE_NO_VERIFY);
    g_fwsd_readback_len = got;
    g_fwsd_readback_status = (int32_t)st;
}

void freewili_sd_service()
{
    if (s_stageLen == 0) {
        // Nothing to write; if we have flushed at least once and not yet proven
        // the on-card content, read it back now (idle, no contention).
        if (g_fwsd_flushes > 0 && !s_readbackDone) {
            s_readbackDone = true;
            doReadback();
        }
        return;
    }
    uint32_t now = millis();
    if (s_stageLen < FWSD_FLUSH_BYTES && (now - s_lastFlushMs) < FWSD_FLUSH_MS)
        return;
    s_lastFlushMs = now;

    if (flushStage()) {
        s_stageLen = 0;
        s_stageRows = 0;
        g_fwsd_rows_queued = 0;
    } else {
        g_fwsd_errors++;
        // Keep the rows staged and retry on the next interval. If staging fills
        // meanwhile, note_ap drops and counts rather than overrunning.
    }
}

#else
void freewili_sd_note_ap(const FreewiliWifiAp &) {}
void freewili_sd_service() {}
#endif

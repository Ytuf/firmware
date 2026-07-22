// SD-backed persistence for the FreeWili 2. See freewili_persist.h.

#include "configuration.h"
#include "freewili_persist.h"

#if defined(FREEWILI)

#include "onewili_fwgui.h"
#include "sdfs_client.h"
#include "freewili_sd_read.h"
#include <Arduino.h>
#include <string.h>

static_assert(SDFS_MAX_PAYLOAD == 96, "SDFS_MAX_PAYLOAD must match MAIN");
static_assert(SDFS_MAX_PATH == 128, "SDFS_MAX_PATH must match MAIN");

// Meshtastic's LittleFS paths are like "/prefs/config.proto"; MAIN's SDFS
// storage prepends its own volume ("1:"), so we prefix an app dir and pass the
// LittleFS path through. Result on the card: 1:/meshtastic/prefs/config.proto.
#define FWPERSIST_DIR    "/meshtastic"
#define FWPERSIST_SUBDIR "/meshtastic/prefs"

// Same pacing as the wardrive writer: 4 HWRITE chunks + a blocking seek stays
// inside MAIN's 8-slot inbound ring. Config blobs are < 1 KB so this is a
// handful of round trips.
#define FWP_PACE (4u * (uint32_t)(SDFS_MAX_PAYLOAD - 1u))
// Blocking-op poll bound (see freewili_sd.cpp): ~200ms, well above one MAIN loop.
#define FWP_TIMEOUT_POLLS 700000

extern "C" {
volatile uint32_t g_fwpersist_reads_ok       __attribute__((used)) = 0;
volatile uint32_t g_fwpersist_reads_fail     __attribute__((used)) = 0;
volatile uint32_t g_fwpersist_writes_ok      __attribute__((used)) = 0;
volatile uint32_t g_fwpersist_writes_fail    __attribute__((used)) = 0;
volatile int32_t  g_fwpersist_last_status    __attribute__((used)) = -2;
volatile uint32_t g_fwpersist_last_read_len  __attribute__((used)) = 0;
volatile uint32_t g_fwpersist_last_write_len __attribute__((used)) = 0;
}

static sdfs_client_t s_sd;
static bool s_ready = false;

void freewili_persist_init()
{
    if (s_ready)
        return;
    ow_fwgui_link_ensure();                 // link up before NodeDB loads config
    static sdfs_transport_t tp;
    tp = ow_fwgui_sdfs_transport();
    sdfs_client_init(&s_sd, &tp);
    s_sd.timeout_polls = FWP_TIMEOUT_POLLS;
    s_ready = true;
}

// Map a Meshtastic LittleFS path to the on-card path (volume-relative; MAIN's
// storage adds "1:"). "/prefs/config.proto" -> "/meshtastic/prefs/config.proto".
static void mapPath(const char *littlefs_path, char *out, size_t cap)
{
    snprintf(out, cap, "%s%s", FWPERSIST_DIR, littlefs_path);
}

// A late/dropped reply would be consumed by the next op (mismatched req_id ->
// timeout), desyncing the lane; start each op from a clean queue.
static void drainStale()
{
    static sdfs_transport_t tp;
    tp = ow_fwgui_sdfs_transport();
    uint8_t junk[SDFS_MAX_FRAME];
    size_t n = 0;
    for (int i = 0; i < 2 * 8; i++)
        if (tp.recv(tp.ctx, junk, sizeof(junk), &n) != 1)
            break;
}

size_t freewili_persist_read(const char *littlefs_path, uint8_t *buf, size_t cap)
{
    if (!s_ready)
        freewili_persist_init();

    char path[SDFS_MAX_PATH];
    mapPath(littlefs_path, path, sizeof(path));

    size_t n = freewili_sd_read(path, buf, cap);
    g_fwpersist_last_status = g_fwsdread_last_status;
    if (n == 0 && g_fwsdread_last_status != SDFS_OK) {
        g_fwpersist_reads_fail++;
        return 0;
    }
    g_fwpersist_last_read_len = (uint32_t)n;
    g_fwpersist_reads_ok++;
    return n;
}

bool freewili_persist_write(const char *littlefs_path, const uint8_t *buf, size_t len)
{
    if (!s_ready)
        freewili_persist_init();
    drainStale();

    // Best-effort dir create; already-exists returns BAD_REQUEST (FR_EXIST).
    sdfs_mkdir(&s_sd, FWPERSIST_DIR);
    sdfs_mkdir(&s_sd, FWPERSIST_SUBDIR);

    char path[SDFS_MAX_PATH];
    mapPath(littlefs_path, path, sizeof(path));

    uint8_t h = 0xFF;
    sdfs_status_t st = sdfs_open(&s_sd, path, 1 /*write/truncate*/, &h);
    g_fwpersist_last_status = (int32_t)st;
    if (st != SDFS_OK || h == 0xFF) {
        g_fwpersist_writes_fail++;
        return false;
    }

    uint32_t off = 0;
    bool ok = true;
    while (off < len) {
        uint32_t take = (uint32_t)len - off;
        if (take > FWP_PACE)
            take = FWP_PACE;
        st = sdfs_hwrite(&s_sd, h, buf + off, take);
        if (st != SDFS_OK) {
            ok = false;
            break;
        }
        off += take;
        st = sdfs_seek(&s_sd, h, off);   // blocking pace: drains MAIN's ring
        if (st != SDFS_OK) {
            ok = false;
            break;
        }
    }

    sdfs_status_t cl = sdfs_hclose(&s_sd, h, ok ? off : SDFS_HCLOSE_NO_VERIFY);
    if (ok && cl != SDFS_OK) {
        ok = false;
        st = cl;
    }
    g_fwpersist_last_status = (int32_t)st;
    if (!ok) {
        g_fwpersist_writes_fail++;
        return false;
    }
    g_fwpersist_last_write_len = off;
    g_fwpersist_writes_ok++;
    return true;
}

#else
void freewili_persist_init() {}
size_t freewili_persist_read(const char *, uint8_t *, size_t) { return 0; }
bool freewili_persist_write(const char *, const uint8_t *, size_t) { return false; }
#endif

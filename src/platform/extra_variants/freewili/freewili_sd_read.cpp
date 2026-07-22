// Shared SDFS blob reader, lifted from the proven freewili_persist read path.
// See freewili_sd_read.h.

#include "configuration.h"
#include "freewili_sd_read.h"
#if defined(FREEWILI)
#include "onewili_fwgui.h"
#include "sdfs_client.h"
#include <string.h>

extern "C" {
volatile uint32_t g_fwsdread_ok __attribute__((used)) = 0;
volatile uint32_t g_fwsdread_fail __attribute__((used)) = 0;
volatile uint32_t g_fwsdread_last_len __attribute__((used)) = 0;
volatile int32_t  g_fwsdread_last_status __attribute__((used)) = -2;
}
static sdfs_client_t s_c;
static bool s_ready = false;

void freewili_sd_read_init()
{
    if (s_ready) return;
    ow_fwgui_link_ensure();
    static sdfs_transport_t tp;
    tp = ow_fwgui_sdfs_transport();
    sdfs_client_init(&s_c, &tp);
    s_c.timeout_polls = 700000;
    s_ready = true;
}

static void drainStale()
{
    static sdfs_transport_t tp;
    tp = ow_fwgui_sdfs_transport();
    uint8_t junk[SDFS_MAX_FRAME]; size_t n = 0;
    for (int i = 0; i < 16; i++)
        if (tp.recv(tp.ctx, junk, sizeof(junk), &n) != 1) break;
}

size_t freewili_sd_read(const char *volpath, uint8_t *buf, size_t cap)
{
    if (!s_ready) freewili_sd_read_init();
    drainStale();
    uint8_t h = 0xFF;
    sdfs_status_t st = sdfs_open(&s_c, volpath, 0, &h);
    g_fwsdread_last_status = (int32_t)st;
    if (st != SDFS_OK || h == 0xFF) { g_fwsdread_fail++; return 0; }
    uint32_t got = 0;
    st = sdfs_hread(&s_c, h, buf, (uint32_t)cap, &got);
    sdfs_hclose(&s_c, h, SDFS_HCLOSE_NO_VERIFY);
    g_fwsdread_last_status = (int32_t)st;
    if (st != SDFS_OK) { g_fwsdread_fail++; return 0; }
    g_fwsdread_last_len = got; g_fwsdread_ok++;
    return (size_t)got;
}
#else
void freewili_sd_read_init() {}
size_t freewili_sd_read(const char *, uint8_t *, size_t) { return 0; }
#endif

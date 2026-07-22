// Color OSM map render for the FreeWili ST7789 panel. Tiles are read off the
// microSD (over SDFS via freewili_sd_read), decoded to RGB565 line-by-line, and
// each row is pushed straight to the panel via FreeWiliPanel::pushRect -- this
// mirrors TFTDisplay.cpp's proven mono flush (one pushRect(x,y,w,1,...) per row)
// so it needs no full-tile buffer and cannot overflow SRAM.
//   freewili_map_render/service draw the full-screen covering grid, static
//   center, no PSRAM, one tile per service tick; the carousel frame callback
//   drives render() and main.cpp's loop drives service().
// The decode shares s_png / s_pngbuf / s_line / s_px / s_py and the clipped pngDraw.

#include "configuration.h"
#include "freewili_map.h"
#if defined(FREEWILI)
#include "freewili_panel.h"
#include "freewili_sd_read.h"
#include "freewili_map_math.h"
#include <PNGdec.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Panel wants big-endian RGB565 on the wire (TFTDisplay.cpp's mono flush
// bswap16's its colors before pushRect). One named constant so a color fix
// is a one-line flip if the webcam shows wrong colors.
static const int FWMAP_PNG_ENDIAN = PNG_RGB565_BIG_ENDIAN;

extern "C" {
volatile uint32_t g_fwmap_tiles_loaded __attribute__((used)) = 0;
volatile uint32_t g_fwmap_tiles_failed __attribute__((used)) = 0;

// Task 5 covering-grid render state. Static center = Mountain View, z16.
volatile int32_t  g_fwmap_center_lat_i __attribute__((used)) = 373861000;
volatile int32_t  g_fwmap_center_lon_i __attribute__((used)) = -1220839000;
volatile int32_t  g_fwmap_zoom         __attribute__((used)) = 16;
volatile uint32_t g_fwmap_active       __attribute__((used)) = 0; // 1 while the map owns the panel
volatile uint32_t g_fwmap_grid_pending __attribute__((used)) = 0; // covering tiles left to draw
volatile uint32_t g_fwmap_grid_total   __attribute__((used)) = 0; // covering tiles this grid

// Task 6a overlay state.
volatile uint32_t g_fwmap_overlays_drawn __attribute__((used)) = 0; // overlay repaint counter

// Task 6b: carousel frame index the map was registered at (Screen.cpp sets it at
// frame registration; -1 until then). The frame callback gates its takeover on
// state->currentFrame == this.
volatile int32_t  g_fwmap_frame_index  __attribute__((used)) = -1;

// Fix-loss repaint request: drawFreewiliMap sets this on the active->inactive
// edge (a REAL fix lost while the map frame is current); Screen::runOnce()
// consumes it and does display(true) on the single flush core to erase the
// directly-drawn color band the diff-flush can't. NOT a display() from the
// callback -- that path runs inside the flush and would be reentrant on spi1.
volatile bool     g_fwmap_force_repaint __attribute__((used)) = false;

// Task 6b-rev: region-aware takeover band. The color map owns ONLY the rows
// [g_fwmap_band_top, g_fwmap_band_bottom); the mono top banner stays above and
// the mono page-indicator stays below. Runtime-tunable so the bounds can clear
// the banner/indicator exactly without a rebuild.
volatile int32_t  g_fwmap_band_top     __attribute__((used)) = 24;  // first map row (below the mono banner ~FONT_HEIGHT_SMALL+1)
volatile int32_t  g_fwmap_band_bottom  __attribute__((used)) = 290; // one past last map row (above the page-indicator at y=height-8)
}

static PNG      s_png;
static uint8_t  s_pngbuf[48 * 1024]; // compressed tile input (tiles on card are <=40 KB)
static uint16_t s_line[256];         // one decoded row, RGB565 (512 B)
static int16_t  s_px, s_py;          // where this tile draws on the panel

// Panel is 480 wide x 320 tall. The map render is confined to the takeover band
// [g_fwmap_band_top, g_fwmap_band_bottom); the vertical center is the band center.
static const int32_t  FWMAP_W = 480;
static const uint16_t FWMAP_BG_GRAY = 0x8410; // neutral gray (R~=G~=B, byte-order agnostic)

// Set once the current grid's overlays are painted; reset each render() so a new
// grid (or a zoom re-render) repaints them exactly once, always after the tiles.
static bool s_overlays_done = false;

// One covering-grid tile: card tile (tx,ty), its top-left on the panel (sx,sy),
// and its draw state. Edge tiles overhang the screen; pngDraw clips.
enum { GT_PENDING = 0, GT_DRAWN = 1, GT_FAILED = 2 };
struct GridTile {
    int32_t tx, ty;
    int16_t sx, sy;
    uint8_t state;
};
static GridTile s_grid[9]; // 3x3 covering grid, only visible tiles populated

// PNGdec draw callback: PNG_DRAW_CALLBACK is int(PNGDRAW*) -- returning 0 aborts
// the decode (PNG_QUIT_EARLY), so return 1 to keep going. Clips the row to
// [0,480) in x and to the takeover band [g_fwmap_band_top, g_fwmap_band_bottom)
// in y: covering-grid edge tiles overhang, and a negative x would wrap to a huge
// uint16 in pushRect and corrupt the panel. Band-clip keeps tiles off the mono
// banner/page-indicator rows the mono flush owns.
static int pngDraw(PNGDRAW *d)
{
    int32_t sy = s_py + d->y;
    if (sy < g_fwmap_band_top || sy >= g_fwmap_band_bottom) return 1; // row outside band -> skip, keep decoding
    s_png.getLineAsRGB565(d, s_line, FWMAP_PNG_ENDIAN, 0xffffffff);
    int32_t src = (s_px < 0) ? -s_px : 0;  // left-clip
    int32_t dx  = s_px + src;
    int32_t w   = 256 - src;
    if (dx + w > FWMAP_W) w = FWMAP_W - dx; // right-clip
    if (w > 0) freewiliGetTFT()->pushRect(dx, sy, w, 1, &s_line[src]);
    return 1;
}

// Task 6a: fill a screen rect with a solid RGB565 color, byte-swapped for the
// panel (pushRect + __builtin_bswap16 is the proven-correct path; fillScreen
// mis-renders solid colors). Clipped to [0,FWMAP_W) in x and to the takeover band
// [g_fwmap_band_top, g_fwmap_band_bottom) in y exactly like pngDraw -- a negative
// or overhanging pushRect x would wrap to a huge uint16 and corrupt the panel,
// and the band-clip keeps every fill (background + overlays) off the mono
// banner/indicator rows. The scratch row holds one solid color, so a span wider
// than the 256-entry s_line buffer is pushed in <=256 px chunks -- reuses s_line,
// no new buffer. Single-threaded; drawn after the tiles.
static void fwmapFillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color565)
{
    uint16_t be = __builtin_bswap16(color565);
    for (int32_t i = 0; i < 256; i++) s_line[i] = be;
    int32_t x0 = (x < 0) ? 0 : x;                 // left-clip
    int32_t x1 = x + w;
    if (x1 > FWMAP_W) x1 = FWMAP_W;               // right-clip
    for (int32_t r = 0; r < h; r++) {
        int32_t sy = y + r;
        if (sy < g_fwmap_band_top || sy >= g_fwmap_band_bottom) continue; // clip to band
        for (int32_t cx = x0; cx < x1; cx += 256) {
            int32_t ww = x1 - cx;
            if (ww > 256) ww = 256;
            freewiliGetTFT()->pushRect(cx, sy, ww, 1, s_line);
        }
    }
}

// Task 6a/6b-rev: draw the graphical overlays on top of the completed tile grid.
// Called by service() once the grid finishes (and once per zoom re-render). No
// font: the zoom level is shown as colored blocks, not glyphs. 6b-rev: the mono
// top banner now provides status, so the full-width header bar and the fix dot
// are DROPPED; the compact zoom indicator moves to the top-left of the band and
// the self marker sits at the band center. Everything is band-clipped by
// fwmapFillRect, so no overlay pixel lands on the mono banner/indicator rows.
void freewili_map_draw_overlays()
{
    int32_t bandCenterY = (g_fwmap_band_top + g_fwmap_band_bottom) / 2;

    // Zoom indicator: three 8x8 blocks at the top-left of the band for z14/z16/z18.
    // The current zoom's block is bright white, the other two dim gray. 14->0,16->1,18->2.
    int32_t zi = (g_fwmap_zoom <= 14) ? 0 : (g_fwmap_zoom >= 18 ? 2 : 1);
    for (int32_t b = 0; b < 3; b++) {
        uint16_t c = (b == zi) ? COLOR565(255, 255, 255) : COLOR565(80, 80, 80);
        fwmapFillRect(6 + b * 12, g_fwmap_band_top + 2, 8, 8, c);
    }

    // Self marker at the band center (240, bandCenterY): a white 11x11 square with
    // a red 7x7 inset (2px white border). No heading tick (deferred).
    fwmapFillRect(240 - 5, bandCenterY - 5, 11, 11, COLOR565(255, 255, 255));
    fwmapFillRect(240 - 3, bandCenterY - 3, 7, 7, COLOR565(255, 0, 0));

    g_fwmap_overlays_drawn++;
}

// Task 5: (re)compute the covering grid for the current center/zoom, paint the
// gray background once, mark every visible covering tile pending, and take over
// the panel. One-shot "start/refresh the map" call (the fillScreen blocks ~90 ms;
// acceptable). service() then draws the tiles one per tick.
void freewili_map_render()
{
    FwMapPixel c = fwMapProject(g_fwmap_center_lat_i, g_fwmap_center_lon_i, g_fwmap_zoom);
    int32_t cx = (int32_t)floor(c.tile_xf);
    int32_t cy = (int32_t)floor(c.tile_yf);
    double  ox = (c.tile_xf - cx) * 256.0; // center pixel offset within center tile
    double  oy = (c.tile_yf - cy) * 256.0;

    // 6b-rev: the map is confined to the takeover band; its vertical center is the
    // band center, not the panel center.
    int32_t bandCenterY = (g_fwmap_band_top + g_fwmap_band_bottom) / 2;

    uint32_t total = 0;
    for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
            int32_t sx = (int32_t)lround(240.0 - ox + i * 256.0);
            int32_t sy = (int32_t)lround((double)bandCenterY - oy + j * 256.0);
            // Visible iff the 256x256 extent intersects [0,480) x [band_top,band_bottom).
            if (sx < FWMAP_W && sx + 256 > 0 && sy < g_fwmap_band_bottom && sy + 256 > g_fwmap_band_top) {
                s_grid[total].tx    = cx + i;
                s_grid[total].ty    = cy + j;
                s_grid[total].sx    = (int16_t)sx;
                s_grid[total].sy    = (int16_t)sy;
                s_grid[total].state = GT_PENDING;
                total++;
            }
        }
    }
    g_fwmap_grid_total = total;

    g_freewili_color_takeover = true; // suppress the mono flush so color pixels survive
    // Paint ONLY the band gray (fillScreen would paint over the mono banner/indicator).
    fwmapFillRect(0, g_fwmap_band_top, FWMAP_W, g_fwmap_band_bottom - g_fwmap_band_top, FWMAP_BG_GRAY);

    g_fwmap_grid_pending = total;
    g_fwmap_active       = 1;
    s_overlays_done      = false; // repaint overlays once this new grid completes
}

// Task 5: draw at most ONE pending covering tile per call (read+decode+blit).
// Blocking read is fine -- same as the Task-4 entry; the loop breathes between
// tiles. Call once per loop tick while g_fwmap_active.
void freewili_map_service()
{
    if (!g_fwmap_active) return;

    GridTile *g = nullptr;
    for (uint32_t k = 0; k < g_fwmap_grid_total; k++) {
        if (s_grid[k].state == GT_PENDING) {
            g = &s_grid[k];
            break;
        }
    }
    if (g == nullptr) {
        // Grid complete (every tile drawn or failed -> g_fwmap_grid_pending == 0).
        // Paint the overlays once, AFTER all tiles (they cover the whole screen).
        if (!s_overlays_done) {
            freewili_map_draw_overlays();
            s_overlays_done = true;
        }
        return;
    }

    char path[64];
    snprintf(path, sizeof(path), "/maptiles/%ld/%ld/%ld.png",
             (long)g_fwmap_zoom, (long)g->tx, (long)g->ty);

    size_t n = freewili_sd_read(path, s_pngbuf, sizeof(s_pngbuf));
    if (n == 0) {
        g->state = GT_FAILED;
        g_fwmap_tiles_failed++;
        g_fwmap_grid_pending--;
        return;
    }

    if (s_png.openRAM(s_pngbuf, (int)n, pngDraw) != PNG_SUCCESS) {
        g->state = GT_FAILED;
        g_fwmap_tiles_failed++;
        g_fwmap_grid_pending--;
        return;
    }

    // Reject an oversized tile: pngDraw's getLineAsRGB565 writes getWidth() pixels
    // into the 256-entry s_line, so a tile wider than 256px (e.g. a 512px @2x tile
    // dropped on the card) would overrun s_line into adjacent .bss. Height needs no
    // guard -- pngDraw clips rows to the band.
    if (s_png.getWidth() > 256) {
        s_png.close();
        g->state = GT_FAILED;
        g_fwmap_tiles_failed++;
        g_fwmap_grid_pending--;
        return;
    }

    s_px = g->sx;
    s_py = g->sy;

    int rc = s_png.decode(nullptr, 0);
    s_png.close();
    if (rc != PNG_SUCCESS) {
        g->state = GT_FAILED;
        g_fwmap_tiles_failed++;
    } else {
        g->state = GT_DRAWN;
        g_fwmap_tiles_loaded++;
    }
    g_fwmap_grid_pending--;
}

// Shared "set zoom + re-render" helper. render() recomputes the grid, repaints
// gray, and re-marks pending (resetting s_overlays_done); service() then redraws
// the tiles and, on completion, the overlays. Only re-renders when the level
// actually changed, so a clamped zoom_in/out at the end stop is a no-op.
static void freewili_map_set_zoom(int32_t z)
{
    if (z == g_fwmap_zoom) return;
    g_fwmap_zoom = z;
    freewili_map_render();
}

// Task 6b-rev: UP = zoom in (14 -> 16 -> 18, CLAMP at 18).
void freewili_map_zoom_in()
{
    int32_t z = g_fwmap_zoom;
    if (z < 16)      z = 16;
    else if (z < 18) z = 18; // else already >=18: clamp
    freewili_map_set_zoom(z);
}

// Task 6b-rev: DOWN = zoom out (18 -> 16 -> 14, CLAMP at 14).
void freewili_map_zoom_out()
{
    int32_t z = g_fwmap_zoom;
    if (z > 16)      z = 16;
    else if (z > 14) z = 14; // else already <=14: clamp
    freewili_map_set_zoom(z);
}

#else
void freewili_map_render() {}
void freewili_map_service() {}
void freewili_map_draw_overlays() {}
void freewili_map_zoom_in() {}
void freewili_map_zoom_out() {}
#endif

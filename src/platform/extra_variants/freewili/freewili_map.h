#pragma once
#include <stdint.h>

// Color OSM map render for the FreeWili ST7789 panel: tiles read off MAIN's
// microSD (over SDFS via freewili_sd_read), decoded to RGB565 and pushed to the
// panel via FreeWiliPanel::pushRect. The covering-grid render is below (Task 5).

// Task 5: full-screen covering-grid color map render, static center, no PSRAM.
// render() (re)computes the covering grid for the current center/zoom, paints the
// gray background, marks every covering tile pending, and takes over the panel.
// service() draws at most ONE pending covering tile per call (read+decode+blit);
// call it once per loop tick while g_fwmap_active.
void freewili_map_render();
void freewili_map_service();

// Task 6a: graphical overlays (self marker + zoom indicator) drawn on top of the
// completed tile grid, and zoom control. draw_overlays() paints the zoom blocks
// then the band-center self marker (graphics only -- the panel has no font).
// Task 6b-rev: zoom_in()/zoom_out() step 14<->16<->18 with clamping (UP/DOWN
// buttons on the map frame), re-rendering only when the level actually changes.
void freewili_map_draw_overlays();
void freewili_map_zoom_in();
void freewili_map_zoom_out();

extern "C" {
extern volatile uint32_t g_fwmap_tiles_loaded, g_fwmap_tiles_failed;
// Task 5 covering-grid render state.
extern volatile int32_t  g_fwmap_center_lat_i, g_fwmap_center_lon_i, g_fwmap_zoom;
extern volatile uint32_t g_fwmap_active;       // 1 while the map owns the panel
extern volatile uint32_t g_fwmap_grid_pending; // covering tiles left to draw
extern volatile uint32_t g_fwmap_grid_total;   // covering tiles in the current grid
// Task 6a overlay state.
extern volatile uint32_t g_fwmap_overlays_drawn; // bumped each time overlays repaint
// Task 6b: carousel frame index the map was registered at (set at frame
// registration in Screen.cpp), so the frame callback can gate on being current.
extern volatile int32_t  g_fwmap_frame_index;
// Fix-loss repaint request: drawFreewiliMap sets this on the active->inactive
// edge; Screen::runOnce() consumes it and does display(true) on the single flush
// core (never a reentrant display() from the callback).
extern volatile bool     g_fwmap_force_repaint;
// Task 6b-rev: region-aware takeover band. The color map owns rows
// [g_fwmap_band_top, g_fwmap_band_bottom); the mono banner is above and the mono
// page-indicator below. Runtime-tunable (default top=24, bottom=290).
extern volatile int32_t  g_fwmap_band_top, g_fwmap_band_bottom;
}

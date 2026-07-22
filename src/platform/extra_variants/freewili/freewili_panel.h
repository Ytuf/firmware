#pragma once

// Panel takeover seam.
//
// FreeWili's ST7789 480x480 panel is NOT driven by LovyanGFX -- TFTDisplay.cpp's
// FREEWILI branch uses its own raw pico-sdk SPI driver ("LovyanGFX breaks SPI on
// RP2350"), so there is no lgfx::LGFX_Device to hand out here. FreeWiliPanel is the
// minimal interface that driver actually implements, exposed so other FreeWili
// modules (e.g. a color map screen) can draw straight to the live panel object
// without pulling in TFTDisplay.cpp's hardware/spi.h etc. headers.
//
// g_freewili_color_takeover, when true, tells TFTDisplay::display() to skip the
// mono->color flush so whatever was drawn directly via freewiliGetTFT() survives.
// Clearing it and forcing a fromBlank repaint restores the normal mono UI.

#include <stdint.h>

class FreeWiliPanel {
  public:
    virtual ~FreeWiliPanel() = default;
    virtual void fillScreen(uint16_t color) = 0;
    virtual void pushRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data) = 0;
};

FreeWiliPanel *freewiliGetTFT(); // live panel object; nullptr until display init
extern volatile bool g_freewili_color_takeover;

// Region-aware takeover band (defined in freewili_map.cpp). When
// g_freewili_color_takeover is set, TFTDisplay::display() flushes the mono banner
// rows (y < g_fwmap_band_top) and page-indicator rows (y >= g_fwmap_band_bottom)
// but NEVER the band [g_fwmap_band_top, g_fwmap_band_bottom) -- the color map owns
// those pixels directly. Runtime-tunable (default top=24, bottom=290).
extern "C" {
extern volatile int32_t g_fwmap_band_top;
extern volatile int32_t g_fwmap_band_bottom;
}

#pragma once
#include <stdint.h>

struct FwMapTileCoord {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct FwMapPixel {
    double tile_xf; // fractional tile X (index = floor, pixel = frac*256)
    double tile_yf;
};

// Web-Mercator forward projection. lat_i/lon_i are 1e-7 degrees.
FwMapPixel fwMapProject(int32_t lat_i, int32_t lon_i, int32_t z);

// Inverse of fwMapProject. Fills lat_i/lon_i (1e-7 degrees).
void fwMapUnproject(double tile_xf, double tile_yf, int32_t z, int32_t *lat_i, int32_t *lon_i);

// 2^z, the number of tiles along each axis at zoom z.
int32_t fwMapTilesPerAxis(int32_t z);

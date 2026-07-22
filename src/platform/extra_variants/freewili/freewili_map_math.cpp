#include "freewili_map_math.h"
#include <math.h>

int32_t fwMapTilesPerAxis(int32_t z) { return (int32_t)1 << z; }

FwMapPixel fwMapProject(int32_t lat_i, int32_t lon_i, int32_t z)
{
    const double lat = lat_i * 1e-7;
    const double lon = lon_i * 1e-7;
    const double n = (double)fwMapTilesPerAxis(z);
    const double lat_rad = lat * M_PI / 180.0;
    FwMapPixel p;
    p.tile_xf = n * ((lon + 180.0) / 360.0);
    p.tile_yf = n * (1.0 - (log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI)) / 2.0;
    return p;
}

void fwMapUnproject(double tile_xf, double tile_yf, int32_t z, int32_t *lat_i, int32_t *lon_i)
{
    const double n = (double)fwMapTilesPerAxis(z);
    const double lon = tile_xf / n * 360.0 - 180.0;
    const double lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * tile_yf / n)));
    const double lat = lat_rad * 180.0 / M_PI;
    if (lon_i) *lon_i = (int32_t)lround(lon * 1e7);
    if (lat_i) *lat_i = (int32_t)lround(lat * 1e7);
}

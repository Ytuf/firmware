#include <unity.h>
#include <math.h>
#include "freewili_map_math.h"

void setUp(void) {}
void tearDown(void) {}

// Mountain View 37.3861, -122.0839 at z16. Reference tile (standard slippy formula):
// x=10543, y=25420. (Recomputed independently in Python from the same
// Web-Mercator formula; the brief's stated 10659/25705 did not match.)
void test_project_known_point_z16(void)
{
    FwMapPixel p = fwMapProject(373861000, -1220839000, 16);
    TEST_ASSERT_EQUAL_INT32(10543, (int32_t)floor(p.tile_xf));
    TEST_ASSERT_EQUAL_INT32(25420, (int32_t)floor(p.tile_yf));
    // pixel-in-tile is in [0,256)
    double px = (p.tile_xf - floor(p.tile_xf)) * 256.0;
    double py = (p.tile_yf - floor(p.tile_yf)) * 256.0;
    TEST_ASSERT_TRUE(px >= 0.0 && px < 256.0);
    TEST_ASSERT_TRUE(py >= 0.0 && py < 256.0);
}

// Round-trip: project then unproject returns the original within ~1e-7 deg tolerance.
void test_project_unproject_roundtrip(void)
{
    int32_t lat0 = 373861000, lon0 = -1220839000;
    FwMapPixel p = fwMapProject(lat0, lon0, 16);
    int32_t lat1 = 0, lon1 = 0;
    fwMapUnproject(p.tile_xf, p.tile_yf, 16, &lat1, &lon1);
    TEST_ASSERT_INT32_WITHIN(50, lat0, lat1);   // <~5e-6 deg
    TEST_ASSERT_INT32_WITHIN(50, lon0, lon1);
}

void test_tiles_per_axis(void)
{
    TEST_ASSERT_EQUAL_INT32(16384, fwMapTilesPerAxis(14));
    TEST_ASSERT_EQUAL_INT32(65536, fwMapTilesPerAxis(16));
    TEST_ASSERT_EQUAL_INT32(262144, fwMapTilesPerAxis(18));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_project_known_point_z16);
    RUN_TEST(test_project_unproject_roundtrip);
    RUN_TEST(test_tiles_per_axis);
    return UNITY_END();
}

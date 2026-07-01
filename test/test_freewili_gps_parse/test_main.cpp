#include <TinyGPS++.h>
#include <unity.h>

#include "TestUtil.h"
#include "freewili_gps_parse.h"

// $GPRMC / $GPGGA with a known fix: 37.3861 deg N, -122.0839 deg W (Mountain View).
// Checksums are real (XOR of all chars between '$' and '*') so TinyGPS++ actually accepts
// these sentences instead of silently discarding them for a bad checksum.
static const char *NMEA = "$GPRMC,081836.00,A,3723.1660,N,12205.0340,W,0.0,0.0,010726,,,A*4D\r\n"
                           "$GPGGA,081836.00,3723.1660,N,12205.0340,W,1,08,0.9,30.0,M,,,,*1C\r\n";

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_parses_fix_to_position_ints(void)
{
    TinyGPSPlus r;
    for (const char *p = NMEA; *p; ++p)
        r.encode(*p);

    meshtastic_Position pos = meshtastic_Position_init_default;
    TEST_ASSERT_TRUE(freewiliGpsParse(r, pos));
    TEST_ASSERT_TRUE(pos.has_latitude_i);
    TEST_ASSERT_TRUE(pos.has_longitude_i);
    TEST_ASSERT_INT32_WITHIN(200, 373861000, pos.latitude_i);   // ~37.3861e7
    TEST_ASSERT_INT32_WITHIN(200, -1220839000, pos.longitude_i);
    TEST_ASSERT_EQUAL(meshtastic_Position_LocSource_LOC_EXTERNAL, pos.location_source);
}

void test_no_fix_returns_false(void)
{
    TinyGPSPlus r;
    meshtastic_Position pos = meshtastic_Position_init_default;
    TEST_ASSERT_FALSE(freewiliGpsParse(r, pos));
}

void setup()
{
    initializeTestEnvironment();

    UNITY_BEGIN();
    RUN_TEST(test_parses_fix_to_position_ints);
    RUN_TEST(test_no_fix_returns_false);
    exit(UNITY_END());
}

void loop() {}

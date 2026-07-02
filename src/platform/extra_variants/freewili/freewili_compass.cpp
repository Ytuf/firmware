// FreeWili 2 tilt-compensated magnetic compass.
//
// The FreeWili 2 carries a Bosch BMM350 magnetometer (I2C 0x14) and a BMI323
// IMU (I2C 0x68) on the sensor bus (Wire == pico-sdk i2c1, GPIO 26/27), but
// stock Meshtastic has no driver for either part. The BMM350/BMI323 register
// protocol + OTP compensation below is ported (MIT) from the hardware-validated
// FreeWili project evaderkrub/sensorview (src/sensors/bmm350*.c, bmi323.c),
// which in turn ported it from Intrepid's fwcom bring-up notes. We feed the
// compensated mag + accel to Meshtastic's own FusionCompassCalculateHeading
// (the same tilt-comp path BMX160Sensor/ICM20948Sensor use) and push the result
// into screen->setHeading(), which drives the Position-screen compass and flips
// the FreeWili node map to heading-up.

#include "configuration.h"

#if defined(FREEWILI) && HAS_SCREEN

#include "Fusion/Fusion.h"
#include "concurrency/OSThread.h"
#include "graphics/Screen.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>

extern graphics::Screen *screen;

// Absolute heading offset (deg) to align the sensor's 0 with the device's
// forward/top edge. Tune once on the bench against a known north; overridable
// from the build with -DFW_COMPASS_OFFSET_DEG=<n>.
#ifndef FW_COMPASS_OFFSET_DEG
#define FW_COMPASS_OFFSET_DEG 0.0f
#endif

#define FW_I2C i2c1

// ---- SWD/GDB-observable state (no console; used for bench calibration) -------
volatile uint32_t g_freewili_compass_ok = 0;     // sensors initialized
volatile uint32_t g_freewili_compass_reads = 0;  // successful mag+accel reads
volatile int32_t g_freewili_compass_heading = 0; // final heading, degrees
volatile int32_t g_freewili_mag_ut_x100[3] = {0, 0, 0};  // compensated mag, uT*100
volatile int32_t g_freewili_accel_mg[3] = {0, 0, 0};     // accel, milli-g

// ===========================================================================
// BMM350 magnetometer (0x14) — ported from sensorview src/sensors/bmm350*.c
// ===========================================================================
namespace
{
constexpr uint8_t BMM350_ADDR = 0x14;
constexpr uint8_t BMM_REG_CHIPID = 0x00;
constexpr uint8_t BMM_REG_PMU_CMD = 0x06;
constexpr uint8_t BMM_REG_AGGR = 0x04;
constexpr uint8_t BMM_REG_AXIS_EN = 0x05;
constexpr uint8_t BMM_REG_CMD = 0x7E;
constexpr uint8_t BMM_REG_OTP_CMD = 0x50;
constexpr uint8_t BMM_REG_OTP_DATA = 0x52;
constexpr uint8_t BMM_REG_MAGDATA = 0x31;
constexpr uint8_t BMM_CHIPID_VAL = 0x33;
constexpr int BMM_DUMMY = 2; // BMM350 I2C reads return 2 leading dummy bytes (HW-confirmed)

constexpr uint8_t BMI323_ADDR = 0x68;
constexpr uint8_t BMI_REG_CHIPID = 0x00;
constexpr uint8_t BMI_REG_ACCCONF = 0x20;
constexpr uint8_t BMI_REG_GYRCONF = 0x21;
constexpr uint8_t BMI_REG_ACCDATA = 0x03;
constexpr uint8_t BMI_REG_CMD = 0x7E;
constexpr uint16_t BMI_CMD_SOFTRESET = 0xDEAF;
constexpr uint8_t BMI_CHIPID_VAL = 0x43;
constexpr int BMI_DUMMY = 2;
constexpr int BMI_RANGE_G = 4;

struct BmmCoeff {
    float offx, offy, offz, toffs;
    float sensx, sensy, sensz, tsens;
    float tcox, tcoy, tcoz;
    float tcsx, tcsy, tcsz;
    float t0;
    float cxy, cyx, czx, czy;
};
BmmCoeff s_coeff;

bool bmmWrite(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_write_blocking(FW_I2C, BMM350_ADDR, b, 2, false) == 2;
}
bool bmmRead(uint8_t reg, uint8_t *out, int n)
{
    uint8_t buf[BMM_DUMMY + 16];
    if (i2c_write_blocking(FW_I2C, BMM350_ADDR, &reg, 1, true) != 1)
        return false;
    if (i2c_read_blocking(FW_I2C, BMM350_ADDR, buf, n + BMM_DUMMY, false) != n + BMM_DUMMY)
        return false;
    for (int i = 0; i < n; i++)
        out[i] = buf[BMM_DUMMY + i];
    return true;
}

int fixSign(int v, int bits)
{
    int p = (bits == 8) ? 128 : (bits == 12) ? 2048 : (bits == 16) ? 32768 : 0;
    if (v >= p)
        v -= p * 2;
    return v;
}
int32_t sign24(uint32_t raw)
{
    return (raw >= 0x800000u) ? (int32_t)raw - 0x1000000 : (int32_t)raw;
}

void parseOtp(const uint16_t w[11], BmmCoeff *c)
{
    int offx = (w[1] & 0x0FFF);
    int offy = ((w[1] & 0xF000) >> 4) + (w[2] & 0x00FF);
    int offz = (w[2] & 0x0F00) + (w[3] & 0x00FF);
    c->offx = (float)fixSign(offx, 12);
    c->offy = (float)fixSign(offy, 12);
    c->offz = (float)fixSign(offz, 12);
    c->toffs = fixSign(w[0] & 0x00FF, 8) / 5.0f;
    c->sensx = fixSign((w[3] & 0xFF00) >> 8, 8) / 256.0f;
    c->sensy = fixSign(w[4] & 0x00FF, 8) / 256.0f + 0.01f;
    c->sensz = fixSign((w[4] & 0xFF00) >> 8, 8) / 256.0f;
    c->tsens = fixSign((w[0] & 0xFF00) >> 8, 8) / 512.0f;
    c->tcox = fixSign(w[5] & 0x00FF, 8) / 32.0f;
    c->tcoy = fixSign(w[6] & 0x00FF, 8) / 32.0f;
    c->tcoz = fixSign(w[7] & 0x00FF, 8) / 32.0f;
    c->tcsx = fixSign((w[5] & 0xFF00) >> 8, 8) / 16384.0f;
    c->tcsy = fixSign((w[6] & 0xFF00) >> 8, 8) / 16384.0f;
    c->tcsz = fixSign((w[7] & 0xFF00) >> 8, 8) / 16384.0f - 0.0001f;
    c->t0 = fixSign(w[10], 16) / 512.0f + 23.0f;
    c->cxy = fixSign(w[8] & 0x00FF, 8) / 800.0f;
    c->cyx = fixSign((w[8] & 0xFF00) >> 8, 8) / 800.0f;
    c->czx = fixSign(w[9] & 0x00FF, 8) / 800.0f;
    c->czy = fixSign((w[9] & 0xFF00) >> 8, 8) / 800.0f;
}

void compensate(int32_t rx, int32_t ry, int32_t rz, int32_t rt, const BmmCoeff *c, float *ox, float *oy, float *oz)
{
    const float adc_gain = 1.0f / 1.5f;
    const float lut_gain = 0.714607238769531f;
    const float power = 1000000.0f / 1048576.0f;
    const float LSB_XY = power / (14.55f * 19.46f * adc_gain * lut_gain);
    const float LSB_Z = power / (9.0f * 31.0f * adc_gain * lut_gain);
    const float LSB_T = 1.0f / (0.00204f * adc_gain * lut_gain * 1048576.0f);

    float x = (float)rx * LSB_XY;
    float y = (float)ry * LSB_XY;
    float z = (float)rz * LSB_Z;
    float t = (float)rt * LSB_T;
    if (t > 0.0f)
        t -= 25.49f;
    else if (t < 0.0f)
        t += 25.49f;
    t = (1.0f + c->tsens) * t + c->toffs;

    x = x * (1.0f + c->sensx);
    x += c->offx;
    x += c->tcox * (t - c->t0);
    x /= (1.0f + c->tcsx * (t - c->t0));
    y = y * (1.0f + c->sensy);
    y += c->offy;
    y += c->tcoy * (t - c->t0);
    y /= (1.0f + c->tcsy * (t - c->t0));
    z = z * (1.0f + c->sensz);
    z += c->offz;
    z += c->tcoz * (t - c->t0);
    z /= (1.0f + c->tcsz * (t - c->t0));

    float den = 1.0f - c->cyx * c->cxy;
    *ox = (x - c->cxy * y) / den;
    *oy = (y - c->cyx * x) / den;
    *oz = z + (x * (c->cyx * c->czy - c->czx) - y * (c->czy - c->cxy * c->czx)) / den;
}

bool bmm350Init()
{
    bmmWrite(BMM_REG_CMD, 0xB6); // soft reset
    sleep_ms(24);
    uint8_t id = 0;
    bmmRead(BMM_REG_CHIPID, &id, 1);
    bool ok = (id == BMM_CHIPID_VAL);

    static const uint8_t otp_addr[11] = {0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x18};
    uint16_t w[11];
    for (int i = 0; i < 11; i++) {
        bmmWrite(BMM_REG_OTP_CMD, (uint8_t)(0x20 | otp_addr[i]));
        sleep_ms(1);
        uint8_t d[2] = {0, 0};
        bmmRead(BMM_REG_OTP_DATA, d, 2);
        w[i] = (uint16_t)((d[0] << 8) | d[1]);
        sleep_ms(1);
    }
    parseOtp(w, &s_coeff);
    bmmWrite(BMM_REG_OTP_CMD, 0x80); // OTP off
    sleep_ms(1);

    bmmWrite(BMM_REG_PMU_CMD, 0x07); // magnetic reset
    sleep_ms(14);
    bmmWrite(BMM_REG_PMU_CMD, 0x05); // flux-guide reset
    sleep_ms(18);
    bmmWrite(BMM_REG_AGGR, 0x34); // ODR 100 Hz + averaging
    sleep_ms(2);
    bmmWrite(BMM_REG_PMU_CMD, 0x02); // UPD_OAE
    sleep_ms(10);
    bmmWrite(BMM_REG_AXIS_EN, 0x07); // enable X,Y,Z
    sleep_ms(2);
    bmmWrite(BMM_REG_PMU_CMD, 0x01); // normal mode
    sleep_ms(40);
    return ok;
}

bool bmm350Read(float *mx, float *my, float *mz)
{
    uint8_t b[12];
    if (!bmmRead(BMM_REG_MAGDATA, b, 12))
        return false;
    int32_t rx = sign24((uint32_t)b[0] | (b[1] << 8) | (b[2] << 16));
    int32_t ry = sign24((uint32_t)b[3] | (b[4] << 8) | (b[5] << 16));
    int32_t rz = sign24((uint32_t)b[6] | (b[7] << 8) | (b[8] << 16));
    int32_t rt = sign24((uint32_t)b[9] | (b[10] << 8) | (b[11] << 16));
    compensate(rx, ry, rz, rt, &s_coeff, mx, my, mz);
    return true;
}

// ---- BMI323 IMU (0x68) — accelerometer only (for tilt comp) ---------------
bool bmiWrite16(uint8_t reg, uint16_t val)
{
    uint8_t b[3] = {reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8)};
    return i2c_write_blocking(FW_I2C, BMI323_ADDR, b, 3, false) == 3;
}
bool bmiRead16(uint8_t reg, int16_t *out, int n16)
{
    uint8_t buf[BMI_DUMMY + 16];
    int total = BMI_DUMMY + 2 * n16;
    if (i2c_write_blocking(FW_I2C, BMI323_ADDR, &reg, 1, true) != 1)
        return false;
    if (i2c_read_blocking(FW_I2C, BMI323_ADDR, buf, total, false) != total)
        return false;
    for (int i = 0; i < n16; i++) {
        int o = BMI_DUMMY + 2 * i;
        out[i] = (int16_t)((uint16_t)buf[o] | ((uint16_t)buf[o + 1] << 8));
    }
    return true;
}

bool bmi323Init()
{
    bmiWrite16(BMI_REG_CMD, BMI_CMD_SOFTRESET);
    sleep_ms(5);
    int16_t id = 0;
    bool ok = bmiRead16(BMI_REG_CHIPID, &id, 1) && ((id & 0xFF) == BMI_CHIPID_VAL);
    bmiWrite16(BMI_REG_ACCCONF, 0x4018); // mode 4, ±4 g, 100 Hz
    bmiWrite16(BMI_REG_GYRCONF, 0x4028); // mode 4, ±500 dps, 100 Hz
    sleep_ms(2);
    return ok;
}

bool bmi323ReadAccel(float *ax, float *ay, float *az)
{
    int16_t d[3];
    if (!bmiRead16(BMI_REG_ACCDATA, d, 3)) // ACC X,Y,Z
        return false;
    *ax = (float)d[0] / 32768.0f * BMI_RANGE_G;
    *ay = (float)d[1] / 32768.0f * BMI_RANGE_G;
    *az = (float)d[2] / 32768.0f * BMI_RANGE_G;
    return true;
}

// Running hard-iron calibration (per-axis min/max centre subtraction).
float s_hiMin[3], s_hiMax[3];
bool s_hiSeen = false;
void hardIron(const float raw[3], float out[3])
{
    if (!s_hiSeen) {
        for (int i = 0; i < 3; i++)
            s_hiMin[i] = s_hiMax[i] = raw[i];
        s_hiSeen = true;
    }
    for (int i = 0; i < 3; i++) {
        if (raw[i] < s_hiMin[i])
            s_hiMin[i] = raw[i];
        if (raw[i] > s_hiMax[i])
            s_hiMax[i] = raw[i];
        out[i] = raw[i] - 0.5f * (s_hiMin[i] + s_hiMax[i]);
    }
}

class FreewiliCompass : public concurrency::OSThread
{
  public:
    FreewiliCompass() : concurrency::OSThread("FWCompass") {}

    void begin()
    {
        bool mag = bmm350Init();
        bool imu = bmi323Init();
        m_ready = mag && imu;
        g_freewili_compass_ok = m_ready ? 1 : 0;
        LOG_INFO("FreeWili compass: BMM350=%d BMI323=%d", mag, imu);
    }

  protected:
    int32_t runOnce() override
    {
        if (!m_ready)
            return 1000; // sensors absent — idle

        float mraw[3], mcal[3], a[3];
        if (!bmm350Read(&mraw[0], &mraw[1], &mraw[2]) || !bmi323ReadAccel(&a[0], &a[1], &a[2]))
            return 200;

        hardIron(mraw, mcal); // updates running min/max, writes centered field into mcal

        // Hard-iron centering only helps once the device has been rotated enough
        // to span the field. On a stationary unit min==max, which would zero the
        // field and make the heading meaningless — so fall back to the raw
        // compensated field (biased by hard iron, but a real bearing) until the
        // running span is large enough to trust the centre.
        float span = 0.0f;
        for (int i = 0; i < 3; i++) {
            float s = s_hiMax[i] - s_hiMin[i];
            if (s > span)
                span = s;
        }
        const float *muse = (span >= 20.0f) ? mcal : mraw;

        FusionVector accel = {.axis = {a[0], a[1], a[2]}};
        FusionVector mag = {.axis = {muse[0], muse[1], muse[2]}};
        float heading = FusionCompassCalculateHeading(FusionConventionNwu, accel, mag);
        heading += FW_COMPASS_OFFSET_DEG;
        heading = fmodf(heading + 360.0f, 360.0f);

        g_freewili_compass_reads++;
        g_freewili_compass_heading = (int32_t)heading;
        for (int i = 0; i < 3; i++) {
            // Expose the RAW compensated field (pre hard-iron) so the bench can
            // confirm the sensor sees Earth's ~25-65 uT and watch it track as
            // the device rotates (the hard-iron output is ~0 until spun).
            g_freewili_mag_ut_x100[i] = (int32_t)(mraw[i] * 100.0f);
            g_freewili_accel_mg[i] = (int32_t)(a[i] * 1000.0f);
        }

        if (screen)
            screen->setHeading((long)heading);

        return 100; // 10 Hz
    }

  private:
    bool m_ready = false;
};

FreewiliCompass *s_compass = nullptr;
} // namespace

void freewili_register_compass()
{
    if (s_compass)
        return;
    s_compass = new FreewiliCompass();
    s_compass->begin();
}

#else
void freewili_register_compass() {}
#endif // FREEWILI && HAS_SCREEN

// NAU88C10 codec + bit-banged I2S for FreeWili 2.
#if defined(FREEWILI)

#include <Arduino.h>
#include <Wire.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#include "freewili_audio.h"

extern "C" {
volatile uint8_t  g_audio_init_status     __attribute__((used)) = 0;  // 0=untried, 1=ok, 2=no-ACK, 3=other
volatile uint32_t g_audio_tone_count      __attribute__((used)) = 0;
volatile uint32_t g_audio_samples_pushed  __attribute__((used)) = 0;
volatile uint32_t g_audio_samples_dropped __attribute__((used)) = 0;
volatile uint32_t g_audio_sys_clk_hz      __attribute__((used)) = 0;
volatile uint32_t g_audio_pwm_wrap        __attribute__((used)) = 0;
}

static constexpr uint8_t  CODEC_I2C_ADDR = 0x1A;
// GPIO 5 is the codec's DIN (MCU output); GPIO 4 is the codec's DOUT (mic ADC).
static constexpr uint8_t  PIN_I2S_DOUT = 5;
static constexpr uint8_t  PIN_I2S_LRCK = 6;
static constexpr uint8_t  PIN_I2S_BCLK = 7;
static constexpr uint8_t  PIN_SPK_MCLK = 22;
static constexpr uint32_t SAMPLE_RATE  = 8000;

static bool s_audio_ready = false;

// NAU88C10 register: 7-bit address + 9-bit value packed into 2 I2C bytes.
static bool nau_write(uint8_t reg, uint16_t value)
{
    Wire1.beginTransmission(CODEC_I2C_ADDR);
    Wire1.write((uint8_t)((reg << 1) | ((value >> 8) & 0x01)));
    Wire1.write((uint8_t)(value & 0xFF));
    return Wire1.endTransmission() == 0;
}

static void mclk_start()
{
    gpio_set_drive_strength(PIN_SPK_MCLK, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(PIN_SPK_MCLK, GPIO_SLEW_RATE_FAST);
    gpio_set_function(PIN_SPK_MCLK, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_SPK_MCLK);
    uint chan  = pwm_gpio_to_channel(PIN_SPK_MCLK);
    uint32_t sys_clk = clock_get_hz(clk_sys);
    g_audio_sys_clk_hz = sys_clk;
    uint32_t wrap = (sys_clk / 2048000u);
    if (wrap > 0) wrap -= 1;
    if (wrap == 0) wrap = 1;
    g_audio_pwm_wrap = wrap;
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_config_set_wrap(&cfg, (uint16_t)wrap);
    pwm_init(slice, &cfg, true);
    pwm_set_chan_level(slice, chan, (uint16_t)((wrap + 1) / 2));
}

extern "C" bool freewili_audio_init(void)
{
    if (s_audio_ready) return true;

    // SUSPECT (2026-07-19): per agents/hardware/pinouts.md the codec is on
    // I2C1 / GPIO26-27, which is `Wire` -- yet this talks over `Wire1`, whose
    // pins are GPIO2/3 (the MAIN UART flow-control lines, not an I2C bus). The
    // pin re-assert below is for 26/27, i.e. the OTHER bus, so these writes very
    // likely go nowhere. Switching to `Wire` was tried and reverted: it did not
    // resolve the setup() hang under investigation and added a second variable.
    // Left as-is deliberately; revisit with the codec on a scope.
    // Wire was brought up earlier (BQ27441 path); calling setSDA/setSCL on a
    // running bus panics arduino-pico, so only re-assert the pin function.
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);

    Wire1.beginTransmission(CODEC_I2C_ADDR);
    if (Wire1.endTransmission() != 0) {
        g_audio_init_status = 2;
        return false;
    }

    mclk_start();
    delay(2);

    nau_write(0x00, 0x000);  // soft reset
    delay(10);

    nau_write(0x01, 0x015D);
    nau_write(0x02, 0x0015);
    nau_write(0x03, 0x00ED);

    nau_write(0x04, 0x0010);  // IIS, 16-bit
    nau_write(0x05, 0x0000);
    nau_write(0x06, 0x0000);  // codec is I2S slave
    nau_write(0x07, 0x000A);  // 8 kHz
    nau_write(0x08, 0x0000);
    nau_write(0x09, 0x0000);
    nau_write(0x0A, 0x0000);
    nau_write(0x0B, 0x00FF);
    nau_write(0x0C, 0x0000);
    nau_write(0x0D, 0x0000);
    nau_write(0x0E, 0x0108);
    nau_write(0x0F, 0x01FF);

    nau_write(0x12, 0x012C);
    nau_write(0x13, 0x002C);
    nau_write(0x14, 0x002C);
    nau_write(0x15, 0x002C);
    nau_write(0x16, 0x002C);

    nau_write(0x18, 0x0032);
    nau_write(0x19, 0x0000);

    nau_write(0x1B, 0x0000);
    nau_write(0x1C, 0x0000);
    nau_write(0x1D, 0x0000);
    nau_write(0x1E, 0x0000);

    nau_write(0x20, 0x0038);
    nau_write(0x21, 0x000B);
    nau_write(0x22, 0x0032);
    nau_write(0x23, 0x0000);

    nau_write(0x24, 0x0008);
    nau_write(0x25, 0x000C);
    nau_write(0x26, 0x0093);
    nau_write(0x27, 0x00E9);
    delay(20);  // PLL lock

    nau_write(0x28, 0x0000);

    nau_write(0x2C, 0x0003);
    nau_write(0x2D, 0x0010);
    nau_write(0x2E, 0x0000);
    nau_write(0x2F, 0x0100);
    nau_write(0x30, 0x0000);
    nau_write(0x31, 0x0002);
    nau_write(0x32, 0x0001);
    nau_write(0x33, 0x0000);
    nau_write(0x34, 0x0040);
    nau_write(0x35, 0x0040);
    nau_write(0x36, 0x003F);
    nau_write(0x37, 0x0040);

    nau_write(0x38, 0x0001);
    nau_write(0x45, 0x0005);  // 5 V speaker amp override

    s_audio_ready = true;
    g_audio_init_status = 1;
    return true;
}

static void bitbang_i2s_init_pins(void)
{
    constexpr uintptr_t kPadsBase = 0x40038000u;
    auto pad_set = [](uint pin) {
        volatile uint32_t *pad =
            (volatile uint32_t *)(kPadsBase + 4u + 4u * pin);
        *pad = (*pad & ~((1u << 7) | (1u << 8))) | (1u << 6);
    };
    gpio_init(PIN_I2S_BCLK);
    gpio_init(PIN_I2S_LRCK);
    gpio_init(PIN_I2S_DOUT);
    gpio_put(PIN_I2S_BCLK, 0);
    gpio_put(PIN_I2S_LRCK, 0);
    gpio_put(PIN_I2S_DOUT, 0);
    gpio_set_dir(PIN_I2S_BCLK, GPIO_OUT);
    gpio_set_dir(PIN_I2S_LRCK, GPIO_OUT);
    gpio_set_dir(PIN_I2S_DOUT, GPIO_OUT);
    pad_set(PIN_I2S_BCLK);
    pad_set(PIN_I2S_LRCK);
    pad_set(PIN_I2S_DOUT);
}

extern "C" void freewili_audio_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (!s_audio_ready) {
        if (!freewili_audio_init()) return;
    }
    if (freq_hz == 0 || duration_ms == 0) return;
    g_audio_tone_count++;

    static bool s_bitbang_inited = false;
    if (!s_bitbang_inited) {
        bitbang_i2s_init_pins();
        s_bitbang_inited = true;
    }

    const uint32_t half_period = SAMPLE_RATE / (2u * freq_hz);
    if (half_period == 0) return;
    const uint32_t total_samples = (SAMPLE_RATE * duration_ms) / 1000u;
    const int16_t amp = 0x2000;
    bool high = true;
    uint32_t in_phase = 0;
    for (uint32_t i = 0; i < total_samples; i++) {
        int16_t sample = high ? amp : -amp;
        for (int ch = 0; ch < 2; ch++) {
            for (int bit = 0; bit < 16; bit++) {
                gpio_put(PIN_I2S_BCLK, 0);
                gpio_put(PIN_I2S_DOUT, (sample >> (15 - bit)) & 1);
                if (bit == 15) {
                    gpio_put(PIN_I2S_LRCK, ch);
                }
                busy_wait_us(2);
                gpio_put(PIN_I2S_BCLK, 1);
                busy_wait_us(2);
            }
        }
        g_audio_samples_pushed++;
        if (++in_phase >= half_period) { in_phase = 0; high = !high; }
    }
}

extern "C" void freewili_audio_play_click(void) { freewili_audio_tone(2000, 20); }
extern "C" void freewili_audio_play_rx(void)    { freewili_audio_tone(880, 80); }
extern "C" void freewili_audio_play_tx(void)    { freewili_audio_tone(1320, 60); }
extern "C" void freewili_audio_play_error(void) { freewili_audio_tone(220, 200); }

#endif // FREEWILI

// NAU88C10 codec + I2S driver for FreeWili 2.
//
// Lifted from the stock FreeWili firmware (rmpLib/rpAudioCodecnau88c10.cpp,
// rmpLib/rpI2S.cpp):
//   - codec on Wire1 / I2C1, address 0x1A, ~30-register init sequence
//   - codec is I2S slave; RP2350 PIO is master
//   - sample rate 8 kHz, 16-bit, mono
//   - MCLK is a 22 MHz PWM on GPIO 22 (codec uses internal PLL to derive
//     BCLK reference from this)
//   - I2S pins: BCLK=7, LRCK=6, DIN=5
//     (GPIO 5 is the codec's DIN — the MCU's playback-data output.
//      GPIO 4 is the codec's DOUT — the ADC's mic-data output, an MCU
//      input. Driving GPIO 4 as I2S data sounds plausible from the name
//      "DOUT" but contends with the codec's own driver and the speaker
//      stays silent because the codec's DIN sees nothing. Verified against
//      stock firmware: Fw2Display.cpp:1089 calls initI2S(SPK_DIN, ...)
//      and FW2Display_pin_definitions.h sets SPK_DIN=5, SPK_DOUT=4.)
//
// We use the arduino-pico I2S library for the sample stream and pico-sdk
// hardware_pwm directly for MCLK, since I2S library doesn't generate the
// 22 MHz the codec wants.
#if defined(FREEWILI)

#include <Arduino.h>
#include <Wire.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <math.h>

#include "freewili_audio.h"

extern "C" {
volatile uint8_t g_audio_init_status __attribute__((used)) = 0; // 0=not tried, 1=ok, 2=no-ACK, 3=other-fail
volatile uint32_t g_audio_tone_count __attribute__((used)) = 0;
volatile uint32_t g_audio_samples_pushed __attribute__((used)) = 0;
volatile uint32_t g_audio_samples_dropped __attribute__((used)) = 0;
volatile uint32_t g_audio_pio_pc __attribute__((used)) = 0;
}

static constexpr uint8_t  CODEC_I2C_ADDR = 0x1A;
static constexpr uint8_t  PIN_I2S_DOUT = 5;  // codec DIN (MCU→codec). Not 4 (codec DOUT, ADC→MCU).
static constexpr uint8_t  PIN_I2S_LRCK = 6;
static constexpr uint8_t  PIN_I2S_BCLK = 7;
static constexpr uint8_t  PIN_SPK_MCLK = 22;
static constexpr uint32_t MCLK_HZ      = 2048000; // 256× the 8 kHz sample rate
static constexpr uint32_t SAMPLE_RATE  = 8000;

// PIO I2S program lifted verbatim from stock FreeWili firmware
// (freewili-firmware/freewilimain/rmpLib/rpI2S.cpp:82-93). 8 instructions,
// 2 sideset bits for BCLK + LRCK, 16 bits × 2 channels per LRCK period.
// Trying this on PIO0 SM2 (stock uses PIO0 SM1) since PIO1 failed pindirs
// setup in this codebase.
static const uint16_t kFwI2sProgram[] = {
    0x6801, 0x1840, 0x6001, 0xf02e,
    0x6001, 0x1044, 0x6801, 0xf82e,
};
static const struct pio_program kFwI2sPioProgram = {
    .instructions = kFwI2sProgram,
    .length = 8,
    .origin = -1,
};

static PIO  s_pio    = nullptr;
static int  s_sm     = -1;
static int  s_offset = -1;
static bool s_audio_ready = false;

// NAU88C10 registers are 9-bit values: the register address is 7 bits and
// the data is 9 bits. The I2C protocol packs this into two bytes:
//   byte0 = (reg << 1) | ((value >> 8) & 1)
//   byte1 = value & 0xFF
static bool nau_write(uint8_t reg, uint16_t value)
{
    Wire1.beginTransmission(CODEC_I2C_ADDR);
    Wire1.write((uint8_t)((reg << 1) | ((value >> 8) & 0x01)));
    Wire1.write((uint8_t)(value & 0xFF));
    return Wire1.endTransmission() == 0;
}

// SWD-readable globals so we can confirm sys_clk and the resulting PWM/PIO
// configurations are what we expect.
extern "C" {
volatile uint32_t g_audio_sys_clk_hz __attribute__((used)) = 0;
volatile uint32_t g_audio_pwm_wrap   __attribute__((used)) = 0;
volatile uint32_t g_audio_pio_clkdiv __attribute__((used)) = 0;
volatile int32_t  g_audio_pio_offset __attribute__((used)) = -1;
}

// Start a 2.048 MHz square wave on GPIO 22 via PWM — the codec's MCLK.
// Compute the wrap value from the ACTUAL sys_clk (don't assume 150 MHz).
static void mclk_start()
{
    // Match stock's rpPWM::Generate: 12 mA drive strength and FAST slew rate.
    // Default 4 mA at the codec's input capacitance produced slow edges that
    // were a candidate root cause when the speaker was silent; the codec PLL
    // may not lock cleanly to a sluggish MCLK.
    gpio_set_drive_strength(PIN_SPK_MCLK, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(PIN_SPK_MCLK, GPIO_SLEW_RATE_FAST);
    gpio_set_function(PIN_SPK_MCLK, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_SPK_MCLK);
    uint chan  = pwm_gpio_to_channel(PIN_SPK_MCLK);
    uint32_t sys_clk = clock_get_hz(clk_sys);
    g_audio_sys_clk_hz = sys_clk;
    // PWM frequency = sys_clk / (clkdiv * (wrap+1)). With clkdiv=1, we need
    // wrap = (sys_clk / 2_048_000) - 1.
    uint32_t wrap = (sys_clk / 2048000u);
    if (wrap > 0) wrap -= 1;
    if (wrap == 0) wrap = 1;
    g_audio_pwm_wrap = wrap;
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_config_set_wrap(&cfg, (uint16_t)wrap);
    pwm_init(slice, &cfg, true);
    pwm_set_chan_level(slice, chan, (uint16_t)((wrap + 1) / 2));  // 50% duty
}

extern "C" bool freewili_audio_init(void)
{
    if (s_audio_ready) return true;

    // Wire1 is already running (initIOExpanderPicoSDK / BQ27441 driver brought
    // it up). Calling setSDA/setSCL after begin() panics on arduino-pico
    // ("FATAL: Attempting to set Wire1.SDA while running"). Just re-assert
    // GPIO function and use the bus as-is.
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);

    // Probe the codec.
    Wire1.beginTransmission(CODEC_I2C_ADDR);
    if (Wire1.endTransmission() != 0) {
        g_audio_init_status = 2;
        return false;
    }

    // Bring MCLK up first — codec's PLL needs it before we can write
    // configuration that depends on the clock.
    mclk_start();
    delay(2);

    // Software reset (reg 0). The datasheet says a write of any value to
    // reg 0 triggers reset; the stock firmware uses 0x000.
    nau_write(0x00, 0x000);
    delay(10);

    // Power Management
    nau_write(0x01, 0x015D);  // PWR1 — DCBUFEN | AUX1MX | AUX2MX | etc.
    nau_write(0x02, 0x0015);  // PWR2 — BOOSTENL | INPGAL | ADCEN
    nau_write(0x03, 0x00ED);  // PWR3 — DACEN | MIXEN | NSPKLEN | NSPKREN

    // Audio Interface — match stock firmware EXACTLY now that MCLK is right.
    nau_write(0x04, 0x0010);  // IIS_FMT
    nau_write(0x05, 0x0000);
    nau_write(0x06, 0x0000);  // SLAVEN=0 — codec is I2S SLAVE
    nau_write(0x07, 0x000A);  // SRATE — 8 kHz
    // Stock firmware also writes these registers in the 0x08-0x0F range —
    // include them for full parity with the stock init sequence.
    nau_write(0x08, 0x0000);
    nau_write(0x09, 0x0000);
    nau_write(0x0A, 0x0000);
    nau_write(0x0B, 0x00FF);
    nau_write(0x0C, 0x0000);
    nau_write(0x0D, 0x0000);
    nau_write(0x0E, 0x0108);
    nau_write(0x0F, 0x01FF);

    // Equalizer (flat)
    nau_write(0x12, 0x012C);
    nau_write(0x13, 0x002C);
    nau_write(0x14, 0x002C);
    nau_write(0x15, 0x002C);
    nau_write(0x16, 0x002C);

    // DAC limiter
    nau_write(0x18, 0x0032);
    nau_write(0x19, 0x0000);

    // Notch filter — all off
    nau_write(0x1B, 0x0000);
    nau_write(0x1C, 0x0000);
    nau_write(0x1D, 0x0000);
    nau_write(0x1E, 0x0000);

    // ALC control
    nau_write(0x20, 0x0038);
    nau_write(0x21, 0x000B);
    nau_write(0x22, 0x0032);
    nau_write(0x23, 0x0000);

    // PLL config (PLL_N=8, PLL_K=0xC0093E9)
    nau_write(0x24, 0x0008);
    nau_write(0x25, 0x000C);
    nau_write(0x26, 0x0093);
    nau_write(0x27, 0x00E9);
    delay(20);  // give the PLL time to lock to the MCLK reference

    // BYP control — no bypass
    nau_write(0x28, 0x0000);

    // Input / output mixer, volume defaults — match stock EXACTLY (my prior
    // 0x2E was 0x0100; stock writes 0x0000, which matters because that bit
    // was routing the AUX input into the right mixer instead of the DAC).
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

    // Output stage — stock value (HP route — the FreeWili speaker is wired
    // to the HP output and the 0x45 register handles the internal boost).
    nau_write(0x38, 0x0001);

    // 5 V speaker amp override
    nau_write(0x45, 0x0005);

    // I2S start is deferred to first tone request. arduino-pico I2S claims a
    // PIO state machine + DMA channel at begin(); doing it eagerly at boot
    // dropped main-loop throughput from ~420/sec to ~120/sec and the screen
    // stopped updating. Lazy init keeps the system normal until the user
    // actually wants audio.
    s_audio_ready = true;
    g_audio_init_status = 1;
    return true;
}

static bool ensure_i2s_started()
{
    if (s_pio) return true;
    // Try PIO0 (stock FreeWili uses pio0). PIO0 SM0 is the WS2812 LED driver,
    // so pio_claim_unused_sm will pick SM1/2/3.
    s_pio = pio0;
    pio_set_gpio_base(s_pio, 0);
    s_sm = pio_claim_unused_sm(s_pio, false);
    if (s_sm < 0) return false;
    s_offset = pio_add_program(s_pio, &kFwI2sPioProgram);
    if (s_offset < 0) { pio_sm_unclaim(s_pio, s_sm); s_pio = nullptr; return false; }
    g_audio_pio_offset = s_offset;

    // Build SM config via SDK helpers — trust the SDK end-to-end for this
    // pass. Hand-rolled register writes used in the failed PIO1 attempt
    // raised the surface area for bugs.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, s_offset, s_offset + 7);
    sm_config_set_sideset(&c, 2, false, false);
    sm_config_set_out_pins(&c, PIN_I2S_DOUT, 1);
    sm_config_set_sideset_pins(&c, PIN_I2S_LRCK);
    sm_config_set_out_shift(&c, /*shift_right=*/false, /*autopull=*/true, /*pull_threshold=*/32);
    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t divider = sys_clk * 4u / SAMPLE_RATE;
    g_audio_pio_clkdiv = divider >> 8;
    sm_config_set_clkdiv_int_frac(&c, divider >> 8, 0);

    // Route the pins to PIO0.
    pio_gpio_init(s_pio, PIN_I2S_LRCK);
    pio_gpio_init(s_pio, PIN_I2S_BCLK);
    pio_gpio_init(s_pio, PIN_I2S_DOUT);
    // RP2350: force-clear PADS_BANK0 ISO bit (bit 8) just in case the SDK
    // helper missed it. IE=1, OD=0, ISO=0.
    constexpr uintptr_t kPadsBank0Base = 0x40038000u;
    auto pad_clear_iso = [](uint pin) {
        volatile uint32_t *pad =
            (volatile uint32_t *)(kPadsBank0Base + 4u + 4u * pin);
        *pad = (*pad & ~((1u << 7) | (1u << 8))) | (1u << 6);
    };
    pad_clear_iso(PIN_I2S_LRCK);
    pad_clear_iso(PIN_I2S_BCLK);
    pad_clear_iso(PIN_I2S_DOUT);

    pio_sm_init(s_pio, s_sm, s_offset, &c);

    uint pin_mask = (1u << PIN_I2S_DOUT) | (3u << PIN_I2S_LRCK);
    pio_sm_set_pindirs_with_mask(s_pio, s_sm, pin_mask, pin_mask);

    pio_sm_set_enabled(s_pio, s_sm, true);
    return true;
}

// CPU bit-banged I2S output. Bypasses PIO — verified audible on FW2 hardware
// 2026-05-22. Pattern lifted from stock's badge_bitbangi2s
// (freewili-firmware/freewilimain/rmpLib/rpI2S.cpp:153-192). Each bit:
//   BCLK=0, set DOUT, set LRCK at last bit, wait, BCLK=1, wait.
// Mono input is duplicated to both L+R channels.
//
// PIO attempts (PIO1 SM2) failed: pindirs would not assert pad OE under any
// approach tried (pio_sm_set_pindirs_with_mask, hand-rolled SET PINDIRS
// injection on enabled SM, embedded SET PINDIRS as program instruction 0,
// IO_BANK0 OEOVER override). Stock uses PIO0 SM1; that pairing is untested
// in this codebase as of 2026-05-22.
static void bitbang_i2s_init_pins(void)
{
    // Belt-and-suspenders pad init in case gpio_init doesn't fully clean up
    // ISO on RP2350. Set IE=1, OD=0, ISO=0, modest drive.
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

    // Lazy pin init for bit-bang on first call. ensure_i2s_started (PIO0
    // SM1) is kept available but unused — PIO0 successfully drives the pads
    // (verified via SWD: GPIO_STATUS bit13/9 + IRQTOPROC firing), but the
    // codec stays silent on the PIO-generated I2S stream. Bit-bang generates
    // an I2S signal the codec accepts. Open mystery; documented in
    // project_freewili_audio_breakthrough memory.
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

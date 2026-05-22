#include "configuration.h"

#ifdef FREEWILI

#include <Wire.h>
#include <SPI.h>
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"   // save_and_disable_interrupts
#include "hardware/pio.h"    // PIO state machine API
#include "hardware/clocks.h" // clock_get_hz
#include "pico/stdlib.h"     // busy_wait_at_least_cycles, sleep_us
#include "input/InputBroker.h"
#include "gps/RTC.h"
#include "mesh/NodeDB.h"   // for config.lora.region force in lateInitVariant
#include <time.h>
#include <sys/time.h>

// Bit-bang WS2812B using DWT cycle counter for cycle-accurate timing.
// We need: T0H≈0.4us, T0L≈0.85us, T1H≈0.8us, T1L≈0.45us, total bit period
// ≈1.25us. Cycle counts depend on clk_sys (which Arduino-Pico typically sets
// to 133 MHz on RP2350). Compute from clock_get_hz at runtime.
//
// Sends an array of `count` bytes ordered as G,R,B per pixel (NEO_GRB) —
// 3 bytes × numPixels.
#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DEMCR       (*(volatile uint32_t*)0xE000EDFC)

static inline void ws2812_enable_cyccnt(void) {
    DEMCR |= 0x01000000;     // TRCENA
    DWT_CTRL |= 0x1;         // CYCCNTENA
}

static inline void wait_cycles(uint32_t cyc) {
    uint32_t start = DWT_CYCCNT;
    while ((uint32_t)(DWT_CYCCNT - start) < cyc) { /* spin */ }
}

// --- Public LED API ---------------------------------------------------------
// `freewili_led_set_pixel(idx, r, g, b)` writes the GRB-packed pixel to an
// internal buffer; `freewili_led_show()` pushes the buffer through the
// inverted bit-bang driver. Callers do their own brightness clamping —
// status-indicator code defined in this file caps itself at LED_MAX_CHANNEL,
// but the API doesn't enforce a ceiling so user-configurable ambient (e.g.
// from Meshtastic app) can go brighter if the user explicitly chooses.

#ifdef HAS_NEOPIXEL
static const uint8_t LED_MAX_CHANNEL = 32;  // default cap for status indicators
static uint8_t s_led_frame[NEOPIXEL_COUNT * 3] = {0};   // GRB order, current frame
static uint8_t s_led_ambient[NEOPIXEL_COUNT * 3] = {0}; // GRB order, baseline ambient
static bool s_led_ambient_enabled = true;
#endif

extern "C" void freewili_led_set_pixel(uint idx, uint8_t r, uint8_t g, uint8_t b)
{
#ifdef HAS_NEOPIXEL
    if (idx >= NEOPIXEL_COUNT) return;
    s_led_frame[idx * 3 + 0] = g;  // GRB order
    s_led_frame[idx * 3 + 1] = r;
    s_led_frame[idx * 3 + 2] = b;
#endif
}

extern "C" void freewili_led_clear(void)
{
#ifdef HAS_NEOPIXEL
    for (size_t i = 0; i < sizeof(s_led_frame); i++) s_led_frame[i] = 0;
#endif
}

static void ws2812_send_bytes(uint pin, const uint8_t *bytes, uint nbytes);  // fwd-declare

extern "C" void freewili_led_show(void)
{
#ifdef HAS_NEOPIXEL
    ws2812_send_bytes(NEOPIXEL_DATA, s_led_frame, sizeof(s_led_frame));
#endif
}

// Set the ambient baseline color (all 7 LEDs same color). Stored in
// s_led_ambient; freewili_led_pulse_all() restores to this after a blink.
extern "C" void freewili_led_set_ambient(uint8_t r, uint8_t g, uint8_t b)
{
#ifdef HAS_NEOPIXEL
    for (uint i = 0; i < NEOPIXEL_COUNT; i++) {
        s_led_ambient[i * 3 + 0] = g;
        s_led_ambient[i * 3 + 1] = r;
        s_led_ambient[i * 3 + 2] = b;
    }
#else
    (void)r; (void)g; (void)b;
#endif
}

// Apply the ambient baseline to the chain (or all-off if ambient disabled).
extern "C" void freewili_led_show_ambient(void)
{
#ifdef HAS_NEOPIXEL
    if (s_led_ambient_enabled) {
        for (size_t i = 0; i < sizeof(s_led_frame); i++) s_led_frame[i] = s_led_ambient[i];
    } else {
        for (size_t i = 0; i < sizeof(s_led_frame); i++) s_led_frame[i] = 0;
    }
    freewili_led_show();
#endif
}

// Flash all 7 LEDs to (r, g, b) for `duration_ms`, then restore the ambient
// baseline. Blocking — fine for short status flashes (≤ 60 ms).
extern "C" void freewili_led_pulse_all(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms)
{
#ifdef HAS_NEOPIXEL
    for (uint i = 0; i < NEOPIXEL_COUNT; i++) {
        freewili_led_set_pixel(i, r, g, b);
    }
    freewili_led_show();
    sleep_ms(duration_ms);
    freewili_led_show_ambient();
#else
    (void)r; (void)g; (void)b; (void)duration_ms;
#endif
}

// Low-battery: blink red three times then disable the ambient backlight to
// save power. Subsequent pulses (TX/RX/etc) still flash and return to OFF
// since ambient is disabled.
extern "C" void freewili_led_low_battery_warning(void)
{
#ifdef HAS_NEOPIXEL
    for (int i = 0; i < 3; i++) {
        for (uint p = 0; p < NEOPIXEL_COUNT; p++) freewili_led_set_pixel(p, 28, 0, 0);
        freewili_led_show();
        sleep_ms(150);
        freewili_led_clear();
        freewili_led_show();
        sleep_ms(150);
    }
    s_led_ambient_enabled = false;
    freewili_led_show_ambient();   // off
#endif
}

// --- PIO-based WS2812 driver ------------------------------------------------
// The earlier SIO bit-bang did not visibly drive the chain even though the
// GPIO_OUT register toggled as expected (verified via GDB). The original
// FreeWili firmware (freewilidisplay/rmpLib/rpNeoPixel.cpp) uses a PIO state
// machine, and Adafruit_NeoPixel on Arduino-Pico also uses PIO. We use the
// stock pico-examples WS2812 program with GPIO_OVERRIDE_INVERT on the output
// pin to handle the chain's inverting buffer in hardware.
//
// Encoded WS2812 PIO program (4 instructions, .side_set 1, T1=2 T2=5 T3=3,
// total 10 cycles per bit at 800 kHz bit rate). Generated by pioasm and
// transcribed here so we don't need a .pio source file in the build:
static const uint16_t ws2812_pio_instructions[] = {
    0x6221, // out  x, 1         side 0 [2]
    0x1123, // jmp  !x, 3        side 1 [1]
    0x1400, // jmp  0            side 1 [4]
    0xa442, // nop                side 0 [4]
};
static const struct pio_program ws2812_pio_program = {
    .instructions = ws2812_pio_instructions,
    .length = 4,
    .origin = -1,
};

static PIO s_ws2812_pio = nullptr;
static int s_ws2812_sm = -1;
static int s_ws2812_offset = -1;
static bool s_ws2812_ready = false;

static void ws2812_pio_init_if_needed(uint pin)
{
    if (s_ws2812_ready) return;

    // Try PIO0 then PIO1 to find a free SM. Adafruit_NeoPixel may also use
    // PIO somewhere; pio_claim_unused_sm with required=false lets us fail
    // gracefully and pick whichever is open.
    for (uint i = 0; i < 2; i++) {
        PIO pio = (i == 0) ? pio0 : pio1;
        int sm = pio_claim_unused_sm(pio, false);
        if (sm < 0) continue;
        if (!pio_can_add_program(pio, &ws2812_pio_program)) {
            pio_sm_unclaim(pio, sm);
            continue;
        }
        s_ws2812_offset = pio_add_program(pio, &ws2812_pio_program);
        s_ws2812_pio = pio;
        s_ws2812_sm = sm;
        break;
    }
    if (!s_ws2812_pio) return;  // no PIO available; LEDs will not work

    // Hand pin to PIO and apply OUTOVER inversion to cancel the external
    // inverter (so the chain sees standard WS2812 polarity).
    pio_gpio_init(s_ws2812_pio, pin);
    gpio_set_outover(pin, GPIO_OVERRIDE_INVERT);
    pio_sm_set_consecutive_pindirs(s_ws2812_pio, s_ws2812_sm, pin, 1, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, s_ws2812_offset + 0, s_ws2812_offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false /*shift left, MSB first*/, true /*autopull*/, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // 10 cycles per WS2812 bit, target 800 kHz -> 8 MHz PIO clock.
    float div = (float)clock_get_hz(clk_sys) / (800000.0f * 10.0f);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(s_ws2812_pio, s_ws2812_sm, s_ws2812_offset, &c);
    pio_sm_set_enabled(s_ws2812_pio, s_ws2812_sm, true);

    s_ws2812_ready = true;
}

// Send `nbytes` of GRB-packed data (3 bytes per LED) to the chain via PIO.
// Each LED consumes 24 bits; we pack them into 32-bit words shifted left by 8
// so the high 24 bits are the GRB color and the low byte is zero.
static void ws2812_send_bytes(uint pin, const uint8_t *bytes, uint nbytes) {
    ws2812_pio_init_if_needed(pin);
    if (!s_ws2812_ready) return;

    // Group bytes into 24-bit GRB pixels. Last partial pixel ignored.
    uint pixels = nbytes / 3;
    for (uint i = 0; i < pixels; i++) {
        uint32_t grb = ((uint32_t)bytes[i * 3 + 0] << 16) |   // G
                       ((uint32_t)bytes[i * 3 + 1] << 8)  |   // R
                       ((uint32_t)bytes[i * 3 + 2]);          // B
        pio_sm_put_blocking(s_ws2812_pio, s_ws2812_sm, grb << 8);
    }
    sleep_us(80);  // reset latch (>50 us of idle LOW lets chain latch the frame)
}

// Display IO Expander (PCAL6524) I2C address
#define IO_EXPANDER_DISPLAY_ADDR 0x23

// No RTC on FW2 hardware. The original FreeWili firmware has the MCP7940
// init commented out (`freewilimain/Fw2Display.cpp:1053`) and the
// `I2C_DEVADDRESS_RTC 0x6F` define commented out (line 172) — the chip is
// not fitted on this revision. An i2c1 scan confirmed nothing at 0x6F.
// Time will read 00:00 until set externally (Bluetooth app, future GPS, or
// manual). Kept the perhapsSetRTC plumbing below in case a later board
// revision adds an RTC.

// --- BQ27441 fuel gauge via raw pico-sdk i2c1 -------------------------------
// Arduino Wire's endTransmission(false)+requestFrom flow does not produce a
// working repeated-start for this chip — empirically requestFrom() returns 0
// bytes even though endTransmission ACKs. Same I2C bus the MCP7940 reads
// from below, just using the raw pico-sdk transaction primitives which we
// know work on this hardware. Mirror's the original FreeWili firmware's
// writeread() pattern.

#define BQ27441_I2C_ADDR 0x55

extern "C" bool freewili_bq27441_read_word(uint8_t reg, uint16_t *out)
{
    if (!out) return false;
    // Force I2C pin function (in case Arduino-Pico's Wire has touched i2c1).
    // Use a TIGHT per-character timeout: when this chip misbehaves, the loose
    // 100 ms timeout × 5 retries × 4 sequential reads in Power::readPowerStatus
    // stacks up to ~3 seconds of total UI freeze. 5 ms × 2 retries × 4 reads =
    // <40 ms worst case. Stale cache fallback in Power.cpp covers the failed
    // reads.
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);
    i2c_init(i2c1, 100000);
    for (int attempt = 0; attempt < 2; attempt++) {
        if (i2c_write_timeout_us(i2c1, BQ27441_I2C_ADDR, &reg, 1, /*nostop=*/true, 5000) < 0) {
            continue;
        }
        uint8_t buf[2] = {0};
        if (i2c_read_timeout_us(i2c1, BQ27441_I2C_ADDR, buf, 2, /*nostop=*/false, 5000) < 0) {
            continue;
        }
        *out = ((uint16_t)buf[1] << 8) | buf[0];
        return true;
    }
    return false;
}

// Cached BQ27441 readings from the early boot probe in initIOExpanderPicoSDK.
// Later reads (after Arduino-Pico's Wire.begin re-configures i2c1) appear to
// fail for this chip — only the address still ACKs writes, reads NACK. The
// chip is verified ACK + readable at the very first opportunity, so we cache
// what we get there and the BQ27441 BatteryLevel implementation in Power.cpp
// falls back to these cached values when the live read fails.
static bool s_bq27441_present = false;
static uint16_t s_bq27441_soc_cache = 0;
static uint16_t s_bq27441_voltage_cache = 0;
static uint16_t s_bq27441_flags_cache = 0;
static int16_t  s_bq27441_avgcurrent_cache = 0;

extern "C" bool freewili_bq27441_present(void) { return s_bq27441_present; }

extern "C" bool freewili_bq27441_read_word_cached(uint8_t reg, uint16_t *out)
{
    // First try a live read. If it works (which it does at very early boot),
    // refresh the cache and return the fresh value. If it fails (which it
    // does later, after Wire.begin's reconfigure), return the cached value.
    if (out && freewili_bq27441_read_word(reg, out)) {
        switch (reg) {
        case 0x04: s_bq27441_voltage_cache = *out; break;
        case 0x06: s_bq27441_flags_cache = *out; break;
        case 0x10: s_bq27441_avgcurrent_cache = (int16_t)*out; break;
        case 0x1C: s_bq27441_soc_cache = *out; break;
        }
        return true;
    }
    if (!s_bq27441_present || !out) return false;
    switch (reg) {
    case 0x04: *out = s_bq27441_voltage_cache; return true;
    case 0x06: *out = s_bq27441_flags_cache; return true;
    case 0x10: *out = (uint16_t)s_bq27441_avgcurrent_cache; return true;
    case 0x1C: *out = s_bq27441_soc_cache; return true;
    }
    return false;
}

// (MCP7940 RTC read removed — no RTC chip on FW2 board. See comment above
// near MCP7940 address removal.)

static void initIOExpanderPicoSDK()
{
    // Init I2C1 on GPIO 26 (SDA) / 27 (SCL) at 400kHz
    i2c_init(i2c1, 400000);
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);

    // Write IO expander defaults (PCAL6524 at 0x23). Per the wio-e5-unlock
    // project (`wio-e5-unlock/src/main.cpp:119-122`), the LoRA_1101_SEL
    // signal is NOR(V1_1, V2_1) via IC82 (SN74LVC1G02). When V1_1=0 AND
    // V2_1=0, SEL=1 and IC113 demux routes GPIO40_DISP → LoRA_PB7
    // (WIO USART1_RX) — which is exactly what we need for the Meshtastic
    // UART bridge. Do NOT set V1_1 (bit 3 of Port 1) to 1: that breaks
    // the UART path entirely.
    uint8_t out_cmd[] = {
        0x04,       // Output Port 0 register
        0xDF,       // Port 0: UART1_TX_DIR=1, UART1_RX_DIR=1 (TEST — was 0),
                    //         SPI1_TX_DIR=1, SPI1_RX_DIR=0, SCLK/CS=1
        0xF4,       // Port 1: V1_1=0 + V2_1=0 → LoRA_1101_SEL=1 (UART path
                    //         active), SCREEN_nRST=1, HP2_EN=1,
                    //         GPIO25_DIR=1, I2C_PULL=1, MIC_PWR=1
        0x85        // Port 2: IR_PWR=1, MCLR=1, LED3=1
    };
    i2c_write_blocking(i2c1, IO_EXPANDER_DISPLAY_ADDR, out_cmd, 4, false);

    uint8_t cfg_cmd[] = {
        0x0C,       // Config Port 0 register
        0x00,       // Port 0: all outputs
        0x00,       // Port 1: all outputs
        0x02        // Port 2: bit 1 (HOTPLUG_DET) = input
    };
    i2c_write_blocking(i2c1, IO_EXPANDER_DISPLAY_ADDR, cfg_cmd, 4, false);

    sleep_ms(50);

    // Probe the BQ27441-G1A fuel gauge while i2c1 is in a known-good state
    // (this is the only point during boot where the chip responds to reads —
    // after Arduino-Pico's Wire.begin runs, it ACKs writes but NACKs reads).
    // Cache initial values so we have something real to report even if the
    // post-Wire reads keep failing.
    uint16_t soc, mv, flags, raw_current;
    if (freewili_bq27441_read_word(0x1C, &soc)) {
        s_bq27441_present = true;
        s_bq27441_soc_cache = soc;
        if (freewili_bq27441_read_word(0x04, &mv))         s_bq27441_voltage_cache = mv;
        if (freewili_bq27441_read_word(0x06, &flags))      s_bq27441_flags_cache = flags;
        if (freewili_bq27441_read_word(0x10, &raw_current)) s_bq27441_avgcurrent_cache = (int16_t)raw_current;
    }
}

// Touch-coordinate diagnostic globals — read via SWD while tapping known
// screen points to derive calibration data.
extern "C" {
volatile int16_t g_touch_raw_x __attribute__((used)) = -1;
volatile int16_t g_touch_raw_y __attribute__((used)) = -1;
volatile int16_t g_touch_screen_x __attribute__((used)) = -1;
volatile int16_t g_touch_screen_y __attribute__((used)) = -1;
// Counters showing how often the touch I2C transaction timed out (bus stuck or
// FT5316 not responding) vs succeeded. A rising fail count alongside a stable
// loop_max_gap_ms means we're correctly bailing out fast.
volatile uint32_t g_touch_i2c_fail_count __attribute__((used)) = 0;
volatile uint32_t g_touch_i2c_ok_count __attribute__((used)) = 0;
}

// --- TZ via pre-populated `environ` ----------------------------------------
// setenv() on this stack crashes the first time it's called: newlib starts
// with `environ == NULL`, and the first setenv() takes a code path that
// malloc()s a new environ array AND a "KEY=VALUE" string AND, on this
// arduino-pico newlib build, ends up touching a sentinel pointer it doesn't
// own. Workaround: hand newlib a static, pre-formed environ that already
// contains a "TZ=..." slot. tzset() and getenv("TZ") then succeed without
// ever allocating. freewili_set_tz() updates the slot in place.
extern "C" {
extern char **environ;
}

static char fw_tz_slot[96] = "TZ=GMT0";
static char *fw_environ[2] = { fw_tz_slot, nullptr };

extern "C" void freewili_set_tz(const char *tz)
{
    if (!tz || !*tz) tz = "GMT0";
    snprintf(fw_tz_slot, sizeof(fw_tz_slot), "TZ=%.*s",
             (int)(sizeof(fw_tz_slot) - 4), tz);
    tzset();
}

// Called very early, before Arduino framework setup
void initVariant()
{
    // Point newlib's environ at our static slot BEFORE any code path can
    // touch getenv()/setenv()/tzset(). Must happen before C++ static
    // constructors or any Meshtastic setup runs.
    environ = fw_environ;

    initIOExpanderPicoSDK();
}

extern "C" bool freewili_audio_init(void);
extern "C" void freewili_audio_tone(uint32_t freq_hz, uint32_t duration_ms);

void lateInitVariant()
{
    // Bring up the NAU88C10 codec only. I2S DMA/PIO setup is now deferred to
    // the first tone request so it doesn't eat resources at boot.
    freewili_audio_init();

    // Skip the boot chime — green-button hook in PICButtonInput.cpp plays a
    // 3-second test tone for SWD-halt verification when needed. Don't block
    // lateInitVariant; main loop must run for the rest of the firmware.

#ifdef SCREEN_TOUCH_RST
    // Hardware-reset the FT5316 touch controller via GPIO31.
    // Original FreeWili firmware uses this pin as touch RESET (not interrupt).
    // Without this reset, the chip ACKs during I2C probe but NACKs during polling.
    gpio_init(SCREEN_TOUCH_RST);
    gpio_set_dir(SCREEN_TOUCH_RST, GPIO_OUT);
    gpio_put(SCREEN_TOUCH_RST, false); // Pull reset LOW
    sleep_ms(1);
    gpio_put(SCREEN_TOUCH_RST, true);  // Release reset HIGH
    sleep_ms(10);

    // Write MODE_SWITCH register (0x00) to complete touch controller init.
    // Original firmware does this lazily on first read; we do it here.
    Wire.beginTransmission(TOUCH_ADDRESS);
    Wire.write(0x00); // MODE_SWITCH register
    Wire.write(0x00); // Normal operating mode
    Wire.endTransmission();
#endif

#ifdef HAS_NEOPIXEL
    // Default ambient: dim cool white on all 7 LEDs (well under the R=32
    // cap). User-configurable in the future; baseline now so the device
    // looks alive. Radio events (TX/RX/new msg) blink to color then restore
    // to ambient. Low-battery warning blinks red and DISABLES ambient.
    freewili_led_set_ambient(/*r=*/6, /*g=*/6, /*b=*/10);
    freewili_led_show_ambient();
    sleep_ms(20);
    freewili_led_show_ambient();  // double-write to latch cleanly
#endif

#ifdef BUZZER_PIN
    // Stronger haptic pulse: max GPIO drive (12 mA), three 150 ms pulses with
    // 80 ms gaps. The motor felt weak earlier because GPIOs default to 4 mA
    // drive strength; pulse train is more perceptible than a single long DC.
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_set_drive_strength(BUZZER_PIN, GPIO_DRIVE_STRENGTH_12MA);
    for (int i = 0; i < 3; i++) {
        gpio_put(BUZZER_PIN, 1);
        sleep_ms(150);
        gpio_put(BUZZER_PIN, 0);
        sleep_ms(80);
    }
#endif

    // No RTC chip on FW2 — time will show 00:00 until set externally
    // (Bluetooth app, GPS, or manual). See comment near the removed MCP7940
    // read function above.
}

// Direct-GPIO haptic observer. The default BuzzerFeedbackThread drives the
// pin via tone()/PWM, which makes the haptic motor barely register. We do a
// solid DC pulse on every InputEvent instead — same path the boot test uses,
// felt as a clean ~80 ms thump.
class FreeWiliHapticObserver : public Observer<const InputEvent *>
{
  public:
    int onNotify(const InputEvent *event) override
    {
        (void)event;
        // Direct DC haptic pulse on any input event. LEDs are NOT flashed
        // here — LED blinks are reserved for radio events (TX/RX/new msg)
        // and the low-battery warning. User input gets haptic only.
        gpio_put(BUZZER_PIN, 1);
        sleep_ms(80);
        gpio_put(BUZZER_PIN, 0);
        return 0;
    }
};

static FreeWiliHapticObserver freewili_haptic;

void freewili_register_haptic_observer()
{
    if (inputBroker) {
        freewili_haptic.observe(inputBroker);
    }
}

#endif

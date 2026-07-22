#include "configuration.h"

#ifdef FREEWILI

#include <Wire.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "input/InputBroker.h"
#include <time.h>

#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DEMCR       (*(volatile uint32_t*)0xE000EDFC)

static inline void ws2812_enable_cyccnt(void) {
    DEMCR |= 0x01000000;
    DWT_CTRL |= 0x1;
}

static inline void wait_cycles(uint32_t cyc) {
    uint32_t start = DWT_CYCCNT;
    while ((uint32_t)(DWT_CYCCNT - start) < cyc) { }
}

#ifdef HAS_NEOPIXEL
static const uint8_t LED_MAX_CHANNEL = 32;
static uint8_t s_led_frame[NEOPIXEL_COUNT * 3] = {0};   // GRB
static uint8_t s_led_ambient[NEOPIXEL_COUNT * 3] = {0}; // GRB
static bool s_led_ambient_enabled = true;
#endif

extern "C" void freewili_led_set_pixel(uint idx, uint8_t r, uint8_t g, uint8_t b)
{
#ifdef HAS_NEOPIXEL
    if (idx >= NEOPIXEL_COUNT) return;
    s_led_frame[idx * 3 + 0] = g;
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

static void ws2812_send_bytes(uint pin, const uint8_t *bytes, uint nbytes);

extern "C" void freewili_led_show(void)
{
#ifdef HAS_NEOPIXEL
    ws2812_send_bytes(NEOPIXEL_DATA, s_led_frame, sizeof(s_led_frame));
#endif
}

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
    freewili_led_show_ambient();
#endif
}

// WS2812 PIO program: 4 instructions, .side_set 1, T1=2 T2=5 T3=3 (10 cycles/bit
// at 800 kHz). LED data line goes through an inverting buffer on FW2 — we
// cancel it with GPIO_OVERRIDE_INVERT on the sideset pin.
static const uint16_t ws2812_pio_instructions[] = {
    0x6221, 0x1123, 0x1400, 0xa442,
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
    if (!s_ws2812_pio) return;

    pio_gpio_init(s_ws2812_pio, pin);
    gpio_set_outover(pin, GPIO_OVERRIDE_INVERT);
    pio_sm_set_consecutive_pindirs(s_ws2812_pio, s_ws2812_sm, pin, 1, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, s_ws2812_offset + 0, s_ws2812_offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    float div = (float)clock_get_hz(clk_sys) / (800000.0f * 10.0f);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(s_ws2812_pio, s_ws2812_sm, s_ws2812_offset, &c);
    pio_sm_set_enabled(s_ws2812_pio, s_ws2812_sm, true);

    s_ws2812_ready = true;
}

static void ws2812_send_bytes(uint pin, const uint8_t *bytes, uint nbytes) {
    ws2812_pio_init_if_needed(pin);
    if (!s_ws2812_ready) return;

    uint pixels = nbytes / 3;
    for (uint i = 0; i < pixels; i++) {
        uint32_t grb = ((uint32_t)bytes[i * 3 + 0] << 16) |
                       ((uint32_t)bytes[i * 3 + 1] << 8)  |
                       ((uint32_t)bytes[i * 3 + 2]);
        pio_sm_put_blocking(s_ws2812_pio, s_ws2812_sm, grb << 8);
    }
    sleep_us(80);  // WS2812 reset latch
}

#define IO_EXPANDER_DISPLAY_ADDR 0x23
#define BQ27441_I2C_ADDR 0x55

// Tracks whether i2c1 has been brought up at 400 kHz with the right pinmux.
// initIOExpanderPicoSDK() and the BQ27441 read helper both rely on this; the
// hot-path read defensively re-initializes once if Wire1.begin() in main()
// reset the bus speed/pinmux between boot init and the first runtime read.
static bool s_i2c1_inited_400k = false;

static void freewili_i2c1_ensure_400k()
{
    if (s_i2c1_inited_400k) return;
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);
    i2c_init(i2c1, 400000);
    s_i2c1_inited_400k = true;
}

// Arduino Wire's repeated-start path doesn't talk reliably to BQ27441 on this
// bus, so we use raw pico-sdk i2c1 with tight per-byte timeouts.
extern "C" bool freewili_bq27441_read_word(uint8_t reg, uint16_t *out)
{
    if (!out) return false;
    freewili_i2c1_ensure_400k();
    for (int attempt = 0; attempt < 2; attempt++) {
        if (i2c_write_timeout_us(i2c1, BQ27441_I2C_ADDR, &reg, 1, true, 5000) < 0) {
            continue;
        }
        uint8_t buf[2] = {0};
        if (i2c_read_timeout_us(i2c1, BQ27441_I2C_ADDR, buf, 2, false, 5000) < 0) {
            continue;
        }
        *out = ((uint16_t)buf[1] << 8) | buf[0];
        return true;
    }
    return false;
}

static bool s_bq27441_present = false;
static uint16_t s_bq27441_soc_cache = 0;
static uint16_t s_bq27441_voltage_cache = 0;
static uint16_t s_bq27441_flags_cache = 0;
static int16_t  s_bq27441_avgcurrent_cache = 0;

extern "C" bool freewili_bq27441_present(void) { return s_bq27441_present; }

// BQ27441 only reads cleanly at very early boot; afterwards it ACKs writes but
// NACKs reads. Cache early values and fall back to them when later reads fail.
extern "C" bool freewili_bq27441_read_word_cached(uint8_t reg, uint16_t *out)
{
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

static void initIOExpanderPicoSDK()
{
    // Boot-time i2c1 bring-up. Deliberately does NOT mark s_i2c1_inited_400k:
    // Wire1.begin() runs later in main() and can reset the bus speed, so the
    // BQ27441 hot path needs to re-assert 400 kHz on its first runtime call.
    i2c_init(i2c1, 400000);
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);

    // LoRA antenna mux: V1_1=0, V2_1=0 → LoRA_1101_SEL=1 routes GPIO40 →
    // WIO USART1_RX. Setting V1_1=1 breaks the UART bridge.
    uint8_t out_cmd[] = {
        0x04,  // Output Port 0
        0xDF,  // UART/SPI dir bits
        0xF4,  // V1_1=0, V2_1=0, SCREEN_nRST=1, HP2_EN=1, etc.
        0x85,  // IR_PWR=1, MCLR=1, LED3=1
    };
    i2c_write_blocking(i2c1, IO_EXPANDER_DISPLAY_ADDR, out_cmd, 4, false);

    uint8_t cfg_cmd[] = {
        0x0C,  // Config Port 0
        0x00,  // all outputs
        0x00,
        0x02,  // P2_1 (HOTPLUG_DET) reserved as input; not currently read by firmware.
    };
    i2c_write_blocking(i2c1, IO_EXPANDER_DISPLAY_ADDR, cfg_cmd, 4, false);

    sleep_ms(50);

    uint16_t soc, mv, flags, raw_current;
    if (freewili_bq27441_read_word(0x1C, &soc)) {
        s_bq27441_present = true;
        s_bq27441_soc_cache = soc;
        if (freewili_bq27441_read_word(0x04, &mv))         s_bq27441_voltage_cache = mv;
        if (freewili_bq27441_read_word(0x06, &flags))      s_bq27441_flags_cache = flags;
        if (freewili_bq27441_read_word(0x10, &raw_current)) s_bq27441_avgcurrent_cache = (int16_t)raw_current;
    }

    // The early-boot reads above lock s_i2c1_inited_400k. Clear it so the
    // first runtime read (after Wire1.begin() runs in main()) re-asserts the
    // 400 kHz speed and the I2C pinmux.
    s_i2c1_inited_400k = false;
}

extern "C" {
volatile int16_t  g_touch_raw_x            __attribute__((used)) = -1;
volatile int16_t  g_touch_raw_y            __attribute__((used)) = -1;
volatile int16_t  g_touch_screen_x         __attribute__((used)) = -1;
volatile int16_t  g_touch_screen_y         __attribute__((used)) = -1;
volatile uint32_t g_touch_i2c_fail_count   __attribute__((used)) = 0;
volatile uint32_t g_touch_i2c_ok_count     __attribute__((used)) = 0;
volatile uint32_t g_touch_i2c_recover_count __attribute__((used)) = 0; // bus recoveries fired
}

// Recover a wedged i2c1 controller/bus. A timed-out or NACK'd touch transaction
// can leave the DW_apb_i2c in an abort/restart-pending state (STOP never issued)
// and the FT5316 clock-stretching, after which every touch read fails forever.
// i2c_init() hardware-resets the controller (clears the held state); re-assert the
// pinmux + pull-ups, then pulse SCREEN_TOUCH_RST so the FT5316 releases the bus and
// re-boots. Called from TFTDisplay::getTouch after N consecutive failures so touch
// self-heals instead of staying dead until reboot. Speed stays 400 kHz (in-spec).
extern "C" void freewili_touch_bus_recover(void)
{
    i2c_init(i2c1, 400000);
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);
#ifdef SCREEN_TOUCH_RST
    gpio_set_dir(SCREEN_TOUCH_RST, GPIO_OUT);
    gpio_put(SCREEN_TOUCH_RST, false);
    sleep_ms(1);
    gpio_put(SCREEN_TOUCH_RST, true);
    sleep_ms(10);
    uint8_t mode[2] = {0x00, 0x00}; // FT5316 MODE_SWITCH -> normal operating mode
    i2c_write_timeout_us(i2c1, TOUCH_ADDRESS, mode, 2, false, 3000);
#endif
}

// newlib's first setenv() crashes on this arduino-pico build (touches a
// sentinel pointer it doesn't own). Hand it a pre-formed `environ` with a
// TZ slot so tzset() never allocates.
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

void initVariant()
{
    // Must run before any static ctor / Meshtastic setup that touches env vars.
    environ = fw_environ;

    initIOExpanderPicoSDK();
}

extern "C" bool freewili_audio_init(void);
extern "C" void freewili_audio_tone(uint32_t freq_hz, uint32_t duration_ms);

#ifdef USE_TINYUSB_HOST
extern "C" void freewiliUsbHostInit(void);
#endif

void lateInitVariant()
{
    freewili_audio_init();

#ifdef USE_TINYUSB_HOST
    // Task 3 spike: bring up the native USB controller as a HOST and read the
    // u-blox M8 GPS (USB-CDC) on a USB-A port. See usbhost/freewili_usb_host.*.
    freewiliUsbHostInit();
#endif

#ifdef SCREEN_TOUCH_RST
    // FT5316 needs a hardware reset before it will respond to polling reads.
    gpio_init(SCREEN_TOUCH_RST);
    gpio_set_dir(SCREEN_TOUCH_RST, GPIO_OUT);
    gpio_put(SCREEN_TOUCH_RST, false);
    sleep_ms(1);
    gpio_put(SCREEN_TOUCH_RST, true);
    sleep_ms(10);

    Wire.beginTransmission(TOUCH_ADDRESS);
    Wire.write(0x00);  // MODE_SWITCH
    Wire.write(0x00);  // normal operating mode
    Wire.endTransmission();
#endif

#ifdef HAS_NEOPIXEL
    freewili_led_set_ambient(6, 6, 10);
    freewili_led_show_ambient();
    sleep_ms(20);
    freewili_led_show_ambient();
#endif

#ifdef BUZZER_PIN
    // Boot haptic test: max drive (12 mA), three 150 ms pulses.
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
}

// DC pulse on every input event — the default BuzzerFeedbackThread tone()/PWM
// path makes the motor barely register on this hardware.
class FreeWiliHapticObserver : public Observer<const InputEvent *>
{
  public:
    int onNotify(const InputEvent *event) override
    {
        (void)event;
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

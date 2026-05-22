#pragma once

#ifndef PRIVATE_HW
#define PRIVATE_HW  // Community/DIY hardware designation
#endif

// --- Display (ST7789 TFT 480x320 via SPI0) ---
#define ST7789_CS 9
#define ST7789_RS 8          // DC pin
#define ST7789_SDA 11        // MOSI
#define ST7789_SCK 10
#define ST7789_MISO -1       // Not connected
#define ST7789_RESET -1      // Controlled via IO expander (SCREEN_nRST)
#define ST7789_BL 25         // Backlight enable (PWM)
#define TFT_BACKLIGHT_ON HIGH
#define TFT_WIDTH 480
#define TFT_HEIGHT 320
#define SPI_FREQUENCY 40000000
#define SCREEN_WIDTH TFT_WIDTH
#define SCREEN_HEIGHT TFT_HEIGHT
#define BRIGHTNESS_DEFAULT 200
#define USE_TFTDISPLAY 1

// --- Touch (FT5316/FT6336U via I2C) ---
#define TOUCH_SCREEN
#define HAS_TOUCHSCREEN 1
#define TOUCH_I2C_PORT 0     // Wire (overridden to i2c1 via __WIRE0_DEVICE)
#define TOUCH_ADDRESS 0x38   // FT5316 (FT5x06 family, chip ID 0x11)
#define SCREEN_TOUCH_RST 31  // GPIO31 — touch controller RESET (not interrupt, despite schematic net name)

// On-screen keyboard for freetext compose (CannedMessageModule). Without this,
// the freetext screen has no keyboard AND no exit gesture, trapping the user.
// USE_VIRTUAL_KEYBOARD enables:
//   - Touch-tap input mapped to on-screen key positions (shift, backspace,
//     space, enter, character keys, 123/ABC layout toggle).
//   - Swipe-LEFT to dismiss the freetext screen (handled in
//     CannedMessageModule::handleFreeTextInput).
#define USE_VIRTUAL_KEYBOARD 1

// --- I2C Bus ---
// GPIO 26/27 are i2c1 pins. Wire is overridden to i2c1 via
// -D__WIRE0_DEVICE=i2c1 in platformio.ini build_flags.
#define I2C_SDA 26
#define I2C_SCL 27

// --- Radio (UART to WIO-E5, both on UART1 / Serial2) ---
// GPIO 40 = UART1_TX (standard F2 mux). GPIO 23 = UART1_RX, but only via the
// F11/UART_AUX function (not the default F2). Both pins route through the
// PCAL6524 antenna mux to WIO PB6/PB7. The wio-e5-unlock app proved this
// pin pair works for talking to the bridge. UARTRadioInterface::init does a
// raw gpio_set_function(23, GPIO_FUNC_UART_AUX) after Serial2.begin().
//
// (Previous config used UART_RADIO_TX_PIN=32 + USE_SPLIT_UART_RADIO. GPIO 32
// is actually CC1101 GDO0 — bytes never reached the WIO. Fixed 2026-05-19.)
#define USE_UART_RADIO 1
#define UART_RADIO_TX_PIN 40
#define UART_RADIO_RX_PIN 23
#define UART_RADIO_BAUD 115200

// Disable standard SPI radio defines
#undef USE_SX1262
#undef USE_SX1268
#undef USE_SX1280
#undef RF95_IRQ

// Dummy SPI LoRa pins to satisfy main.cpp SPI init block
// (they won't be used since we use UART radio, but the code compiles unconditionally)
// SPI1 is used ONLY for the display, not LoRa (we use UART radio).
// Do NOT define HW_SPI1_DEVICE — we don't want Meshtastic's SPI init
// to touch SPI1 at all. Our display driver manages SPI1 directly via Pico SDK.
// Dummy LORA pins to satisfy code that references them:
#ifndef LORA_SCK
#define LORA_SCK 2      // SPI0 pin (unused, keeps Meshtastic SPI init on SPI0)
#endif
#ifndef LORA_MOSI
#define LORA_MOSI 3     // SPI0 pin (unused)
#endif
#ifndef LORA_MISO
#define LORA_MISO 4     // SPI0 pin (unused)
#endif
#ifndef LORA_CS
#define LORA_CS 5       // SPI0 pin (unused)
#endif

// --- Buttons (PIC16 UART via SerialPIO) ---
// PIC16 sends button state as 4-byte frames at 62500 baud:
//   0xC0 0xC5 [MSB] [LSB] — 16-bit button bitmap, BIG-ENDIAN.
// (Per fw2_pic16/barebones-firmware.X/main.c:175 "Custom baud: 1mhz/(4(3+1)) = 62500"
//  and freewilimain/rmpLib/rpPICComm.cpp parser order.)
// Pin names are from the PIC's perspective: PIC_UART_TX_PIN=38 is our
// MCU's TX line INTO the PIC (which the PIC RXes on), PIC_UART_RX_PIN=39 is
// our MCU's RX line FROM the PIC.
// Original FreeWili firmware uses HW UART1 with AUX function on these pins,
// but UART1 is taken by the LoRa bridge — so we use SerialPIO.
#define HAS_PIC_BUTTON_INPUT 1
#define PIC_UART_TX_PIN 38
#define PIC_UART_RX_PIN 39
#define PIC_UART_BAUD 62500

// No direct GPIO button
#define BUTTON_PIN -1

// --- Audio (NAU88C10 codec, optional) ---
// I2S_DOUT here means the MCU's data-out / codec's DIN (playback). GPIO 4
// is the codec's DOUT (ADC out, MCU input) and must NOT be used as the
// MCU's data line — driving GPIO 4 contends with the codec's own driver
// and the speaker stays silent. Stock firmware: SPK_DIN=5 / SPK_DOUT=4
// (FW2Display_pin_definitions.h), initI2S(SPK_DIN, ...) (Fw2Display.cpp).
#define I2S_DOUT 5
#define I2S_BCLK 7
#define I2S_LRCK 6
#define I2S_MCLK 22          // SPK_MCLK

// --- Power ---
// Battery monitored via I2C fuel gauge, not ADC
#define BATTERY_PIN -1
#define BATTERY_SENSE_RESOLUTION_BITS 10

// --- Notifications: haptic feedback ---
// GPIO 20 is IR_TX (drives 3 IR emitters) — definitely NOT a buzzer.
// GPIO 46 is HAP_MOTOR — the actual haptic feedback motor on this board.
// Meshtastic's "buzzer" path is just a digital toggle, which is exactly what
// the haptic motor wants. Driving it through HAS_BUZZER plumbing gives us
// notification-on-message for free.
#define EXT_NOTIFY_OUT 46
#define HAS_BUZZER 1
#define BUZZER_PIN EXT_NOTIFY_OUT

// --- LED chain (WS2812-class, 7 LEDs on GPIO 21) ---
// GPIO 21 is LED_SERIAL on this board — a chain of 7 addressable LEDs, not
// a simple single LED. Driving it with digitalWrite(HIGH/LOW) doesn't work.
// Wire it through Meshtastic's NeoPixel/AmbientLighting path instead, and
// undefine LED_PIN so the heartbeat code doesn't fight us for the pin.
#define LED_PIN -1               // disable Meshtastic's simple LED heartbeat
#define HAS_NEOPIXEL 1
// HAS_RGB_LED is auto-defined by configuration.h when HAS_NEOPIXEL is set —
// don't define it here to avoid a redefinition warning.
#define NEOPIXEL_DATA 21
#define NEOPIXEL_COUNT 7
#define NEOPIXEL_TYPE (NEO_GRB + NEO_KHZ800)

// --- Battery fuel gauge (BQ27441 at I2C1 0x55) ---
// Direct-I2C driver lives in Power.cpp under HAS_BQ27441. The mverch67
// BQ27220 driver was a dead end on Arduino-Pico (its Wire.begin(sda, scl)
// signature is ESP32-only and the BQ27220 register map differs from the
// BQ27441-G1 actually fitted to this board).
#define HAS_BQ27441 1
#define BQ27441_ADDR 0x55

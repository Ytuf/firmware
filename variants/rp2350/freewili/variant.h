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

// --- Touch (FT6336U via I2C) ---
#define TOUCH_SCREEN
#define HAS_TOUCHSCREEN 1
#define TOUCH_I2C_PORT 0     // Wire (overridden to i2c1 via __WIRE0_DEVICE)
#define TOUCH_ADDRESS 0x38   // FT6336U / FT5x06 family

// --- I2C Bus ---
// GPIO 26/27 are i2c1 pins. Wire is overridden to i2c1 via
// -D__WIRE0_DEVICE=i2c1 in platformio.ini build_flags.
#define I2C_SDA 26
#define I2C_SCL 27

// --- Radio (UART to WIO-E5 — split across two hardware UARTs) ---
// GPIO 32 = UART0_TX (Serial1), GPIO 23 = UART1_RX (Serial2)
#define USE_UART_RADIO 1
#define UART_RADIO_TX_PIN 32
#define UART_RADIO_RX_PIN 23
#define UART_RADIO_BAUD 115200
#define USE_SPLIT_UART_RADIO 1 // TX via Serial1 (UART0), RX via Serial2 (UART1)

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
// Temporarily disabled to isolate PIO allocation conflict
// #define HAS_PIC_BUTTON_INPUT 1
// #define PIC_UART_TX_PIN 38
// #define PIC_UART_RX_PIN 39
#define PIC_UART_BAUD 9600

// No direct GPIO button
#define BUTTON_PIN -1

// --- Audio (NAU88C10 codec, optional) ---
#define I2S_DOUT 4
#define I2S_BCLK 7
#define I2S_LRCK 6
#define I2S_MCLK 22          // SPK_MCLK

// --- Power ---
// Battery monitored via I2C fuel gauge, not ADC
#define BATTERY_PIN -1
#define BATTERY_SENSE_RESOLUTION_BITS 10

// --- Audio Notifications ---
#define EXT_NOTIFY_OUT 20        // Use a spare GPIO for notification buzzer/LED
#define HAS_BUZZER 1             // Enable buzzer support
#define BUZZER_PIN EXT_NOTIFY_OUT

// --- LED ---
#define LED_PIN 21            // LED_SERIAL on display CPU

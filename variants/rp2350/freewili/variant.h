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
#define TOUCH_I2C_PORT 0
#define TOUCH_ADDRESS 0x38   // FT6336U / FT5x06 family

// --- I2C Bus ---
#define I2C_SDA 26
#define I2C_SCL 27

// --- Radio (UART to WIO-E5, NOT SPI) ---
#define USE_UART_RADIO 1
#define UART_RADIO_TX_PIN 32   // RP2350 TX -> WIO-E5 PB7 (UART1_RX)
#define UART_RADIO_RX_PIN 23   // RP2350 RX <- WIO-E5 PB6 (UART1_TX)
#define UART_RADIO_BAUD 115200

// Disable standard SPI radio defines
#undef USE_SX1262
#undef USE_SX1268
#undef USE_SX1280
#undef RF95_IRQ

// Dummy SPI LoRa pins to satisfy main.cpp SPI init block
// (they won't be used since we use UART radio, but the code compiles unconditionally)
#ifndef LORA_SCK
#define LORA_SCK -1
#endif
#ifndef LORA_MOSI
#define LORA_MOSI -1
#endif
#ifndef LORA_MISO
#define LORA_MISO -1
#endif
#ifndef LORA_CS
#define LORA_CS -1
#endif

// --- Buttons (PIC16 UART) ---
#define HAS_PIC_BUTTON_INPUT 1
#define PIC_UART_RX_PIN 38
#define PIC_UART_TX_PIN 39
#define PIC_UART_BAUD 9600   // PIC16 default baud

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

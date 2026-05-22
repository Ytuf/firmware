#pragma once

#ifndef PRIVATE_HW
#define PRIVATE_HW
#endif

// Display: ST7789 480x320 via SPI1
#define ST7789_CS 9
#define ST7789_RS 8
#define ST7789_SDA 11
#define ST7789_SCK 10
#define ST7789_MISO -1
#define ST7789_RESET -1  // IO-expander controlled
#define ST7789_BL 25
#define TFT_BACKLIGHT_ON HIGH
#define TFT_WIDTH 480
#define TFT_HEIGHT 320
#define SPI_FREQUENCY 40000000
#define SCREEN_WIDTH TFT_WIDTH
#define SCREEN_HEIGHT TFT_HEIGHT
#define BRIGHTNESS_DEFAULT 200
#define USE_TFTDISPLAY 1

// Touch: FT5316 via I2C1
#define TOUCH_SCREEN
#define HAS_TOUCHSCREEN 1
#define TOUCH_I2C_PORT 0
#define TOUCH_ADDRESS 0x38
#define SCREEN_TOUCH_RST 31  // RESET line (not interrupt)

#define USE_VIRTUAL_KEYBOARD 1

// I2C1
#define I2C_SDA 26
#define I2C_SCL 27

// LoRa via WIO-E5 UART bridge on UART1 (Serial2).
// GPIO 40 = UART1_TX (F2). GPIO 23 = UART1_RX via F11/UART_AUX — applied
// post-Serial2.begin() in UARTRadioInterface::init.
#define USE_UART_RADIO 1
#define UART_RADIO_TX_PIN 40
#define UART_RADIO_RX_PIN 23
#define UART_RADIO_BAUD 115200

#undef USE_SX1262
#undef USE_SX1268
#undef USE_SX1280
#undef RF95_IRQ

// Dummy LORA_* on SPI0 pins to satisfy Meshtastic's unconditional SPI init
// block. SPI1 is the display — do NOT define HW_SPI1_DEVICE.
#ifndef LORA_SCK
#define LORA_SCK 2
#endif
#ifndef LORA_MOSI
#define LORA_MOSI 3
#endif
#ifndef LORA_MISO
#define LORA_MISO 4
#endif
#ifndef LORA_CS
#define LORA_CS 5
#endif

// PIC16 button input: 16-bit bitmap big-endian @ 62500 baud on SerialPIO
// (hardware UARTs are taken). PIN names are from the PIC's perspective —
// PIC_UART_TX_PIN (38) is the MCU's TX into the PIC.
#define HAS_PIC_BUTTON_INPUT 1
#define PIC_UART_TX_PIN 38
#define PIC_UART_RX_PIN 39
#define PIC_UART_BAUD 62500

#define BUTTON_PIN -1

// Audio (NAU88C10): I2S_DOUT = codec DIN (MCU output). GPIO 4 is the codec's
// ADC output — must NOT be driven.
#define I2S_DOUT 5
#define I2S_BCLK 7
#define I2S_LRCK 6
#define I2S_MCLK 22

// Battery monitored via I2C fuel gauge.
#define BATTERY_PIN -1
#define BATTERY_SENSE_RESOLUTION_BITS 10

// Haptic motor on HAP_MOTOR (GPIO 46) wired through the BUZZER plumbing.
#define EXT_NOTIFY_OUT 46
#define HAS_BUZZER 1
#define BUZZER_PIN EXT_NOTIFY_OUT

// 7× WS2812 chain on GPIO 21 with an external inverting buffer on the data
// line. LED_PIN is disabled so the heartbeat doesn't fight for the pin.
#define LED_PIN -1
#define HAS_NEOPIXEL 1
#define NEOPIXEL_DATA 21
#define NEOPIXEL_COUNT 7
#define NEOPIXEL_TYPE (NEO_GRB + NEO_KHZ800)

#define HAS_BQ27441 1
#define BQ27441_ADDR 0x55

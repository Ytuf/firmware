#include "configuration.h"

#ifdef FREEWILI

#include <Wire.h>
#include <SPI.h>
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// Display IO Expander (PCAL6524) I2C address
#define IO_EXPANDER_DISPLAY_ADDR 0x23

static void initIOExpanderPicoSDK()
{
    // Init I2C1 on GPIO 26 (SDA) / 27 (SCL) at 400kHz
    i2c_init(i2c1, 400000);
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);

    // Write IO expander defaults (PCAL6524 at 0x23)
    // Sets SPI buffer directions, SCREEN_nRST, backlight GPIO direction, etc.
    uint8_t out_cmd[] = {
        0x04,       // Output Port 0 register
        0xDB,       // Port 0: SPI buffers as output
        0xF4,       // Port 1: SCREEN_nRST=1, GPIO25_DIR=1, V1=0, V2=0
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
}

// Called very early, before Arduino framework setup
void initVariant()
{
    initIOExpanderPicoSDK();
}

void lateInitVariant()
{
    // Nothing needed — display is handled by TFTDisplay::connect()
}

#endif

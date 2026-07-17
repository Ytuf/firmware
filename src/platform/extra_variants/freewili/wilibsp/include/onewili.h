/* OneWili C API - generated from FreeWili firmware sources. Do not edit. */
#ifndef ONEWILI_H
#define ONEWILI_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "onewili_enums.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OW_OK = 0, OW_ERR_ARG, OW_ERR_IO, OW_ERR_TIMEOUT,
    OW_ERR_FAILED, OW_ERR_PROTOCOL, OW_ERR_BUFFER
} ow_status;

/* Callback transport: you supply the serial I/O (see examples/serial_pc.h).
 * write: return >=0 on success, <0 on error.
 * read : return bytes read (>0), 0 on timeout, <0 on error. */
typedef struct ow_transport {
    void* ctx;
    int (*write)(void* ctx, const uint8_t* data, size_t len);
    int (*read)(void* ctx, uint8_t* buf, size_t cap, uint32_t timeout_ms);
} ow_transport;

#ifndef OW_CMD_MAX
#define OW_CMD_MAX 1024
#endif
#ifndef OW_RESP_MAX
#define OW_RESP_MAX 4096
#endif
#ifndef OW_DEFAULT_TIMEOUT_MS
#define OW_DEFAULT_TIMEOUT_MS 5000
#endif

#ifndef OW_TEXT_EVENT_QUEUE
#define OW_TEXT_EVENT_QUEUE 8
#endif

typedef struct ow_device {
    ow_transport t;
    char line[OW_RESP_MAX];
    size_t line_len;
    /* Spontaneous text-event lines "[*<id> ...]" captured during calls or
     * polls; oldest dropped (and counted) when full. */
    char evq[OW_TEXT_EVENT_QUEUE][OW_RESP_MAX];
    uint32_t evq_head, evq_count;
    uint32_t dropped_text_events;
} ow_device;

/* Sends 0x02 (reset nav to root + quiet mode). */
ow_status ow_open(ow_device* dev, const ow_transport* transport);
void ow_close(ow_device* dev);

/* Poll for a spontaneous text event "[*<id> <args>]". Non-blocking (one
 * zero-timeout read pass). id/args receive the split line body.
 * Returns 1 = filled, 0 = none pending, negative = -(ow_status). */
int ow_poll_text_line(ow_device* dev, char* id, size_t id_cap,
                      char* args, size_t args_cap);

/* High. Sets a GPIO high.  Wire: i\g\s */
ow_status ow_io_gpio_set_io_high(ow_device* dev, int32_t pin);

/* Low. Sets a GPIO low.  Wire: i\g\l */
ow_status ow_io_gpio_set_io_low(ow_device* dev, int32_t pin);

/* Toggle. Toggles the specified GPIO.  Wire: i\g\t */
ow_status ow_io_gpio_set_io_toggle(ow_device* dev, int32_t pin);

/* PWM IO. Enables the PWM feature of GPIO.  Wire: i\g\p */
ow_status ow_io_gpio_set_pwm(ow_device* dev, int32_t gpio_number, double freq, double duty);

/* Get All IOs (hex). Reads all the IOs in a bitfield.  Wire: i\g\u */
ow_status ow_io_gpio_read_all(ow_device* dev, uint32_t* gpiostate);

/* Stream IO reads. Sends GPIO values as a specific millisecond rate to host.  Wire: i\g\o */
ow_status ow_io_gpio_stream_io(ow_device* dev, int32_t reportratems);

/* Toggle High-Speed Bidirectional IO. Toggle utilizing GPIO27 to set the direction of GPIO26..  Wire: i\g\e */
ow_status ow_io_gpio_toggle_hsbdio(ow_device* dev, int32_t pin);


/* Events emitted by GPIO Functions: */
/*   gpioReport (binary) - time_stamp_ns=hexU64, gpio_bitfield=hexU32 - Periodic GPIO bitfield report (binary API) */
#define OW_EVENT_IO_GPIO_GPIO_REPORT "gpioReport"
/* SPI1 Rx (12). IO direction for SPI1 Rx pin 12 (out/in).  Wire: i\g\a\a */
ow_status ow_io_gpio_io_direction_settings_s_pi1_rx12(ow_device* dev);

/* GPIO 26 (26). IO direction for GPIO 26 (out/in).  Wire: i\g\a\b */
ow_status ow_io_gpio_io_direction_settings_g_pio2626(ow_device* dev);

/* SPI1 CS (13). IO direction for SPI1 CS pin 13 (out/in).  Wire: i\g\a\c */
ow_status ow_io_gpio_io_direction_settings_s_pi1cs13(ow_device* dev);

/* GPIO (27). IO direction for GPIO 27 (out/in).  Wire: i\g\a\l */
ow_status ow_io_gpio_io_direction_settings_g_pio27(ow_device* dev);

/* UART1 Rx (9). IO direction for UART1 Rx pin 9 (out/in).  Wire: i\g\a\e */
ow_status ow_io_gpio_io_direction_settings_u_art1_rx9(ow_device* dev);

/* UART1 CTS (10). IO direction for UART1 CTS pin 10 (out/in).  Wire: i\g\a\f */
ow_status ow_io_gpio_io_direction_settings_u_art1cts10(ow_device* dev);

/* UART1 Tx (8). IO direction for UART1 Tx pin 8 (out/in).  Wire: i\g\a\g */
ow_status ow_io_gpio_io_direction_settings_u_art1_tx8(ow_device* dev);

/* UART1 RTS (11). IO direction for UART1 RTS pin 11 (out/in).  Wire: i\g\a\m */
ow_status ow_io_gpio_io_direction_settings_u_art1rts11(ow_device* dev);

/* SPI1 Tx (15). IO direction for SPI1 Tx pin 15 (out/in).  Wire: i\g\a\i */
ow_status ow_io_gpio_io_direction_settings_s_pi1_tx15(ow_device* dev);

/* SPI1 SCLK (14). IO direction for SPI1 SCLK pin 14 (out/in).  Wire: i\g\a\j */
ow_status ow_io_gpio_io_direction_settings_s_pi1sclk14(ow_device* dev);

/* GPIO25 (25). IO direction for GPIO 25 (out/in).  Wire: i\g\a\k */
ow_status ow_io_gpio_io_direction_settings_g_pio2525(ow_device* dev);

/* Write. Writes data to a specific I2C Address.  Wire: i\u\w */
ow_status ow_io_uart_u_art_write(ow_device* dev, const uint8_t* data_bytes, size_t data_bytes_len);

/* Enable UART Read Events. Reads the number from the address.  Wire: i\u\r */
ow_status ow_io_uart_toggle_stream(ow_device* dev, uint8_t* data_bytes, size_t data_bytes_cap, size_t* data_bytes_len);

/* Enable UART API mode. Tests all addresses for I2C Response.  Wire: i\u\t */
ow_status ow_io_uart_uart_enable_api_mode(ow_device* dev);


/* Events emitted by UART Functions: */
/*   uart1 (text) - data_bytes=hexbytes - uart receive frame */
#define OW_EVENT_IO_UART_UART1 "uart1"
/* Baud Rate. UART baud rate in bits per second.  Wire: i\u\s\f */
ow_status ow_io_uart_settings_baud_rate(ow_device* dev, int32_t value);

/* RTS Hand Shaking. Enable RTS hardware handshaking.  Wire: i\u\s\r */
ow_status ow_io_uart_settings_r_ts_hand_shaking(ow_device* dev);

/* CTS Hand Shaking. Enable CTS hardware handshaking.  Wire: i\u\s\c */
ow_status ow_io_uart_settings_c_ts_hand_shaking(ow_device* dev);

/* Data Bits. UART data bits.  Wire: i\u\s\b */
ow_status ow_io_uart_settings_data_bits(ow_device* dev, int32_t value);

/* Parity. UART parity mode.  Wire: i\u\s\p */
ow_status ow_io_uart_settings_parity(ow_device* dev, int32_t value);

/* Stop Bits. UART stop bits.  Wire: i\u\s\s */
ow_status ow_io_uart_settings_stop_bits(ow_device* dev, int32_t value);

/* Module. Which UART module handles the port.  Wire: i\u\s\m */
ow_status ow_io_uart_settings_module(ow_device* dev, int32_t value);

/* SFP Poll. Polls for SFP Modules on the I2C bus. If any are found, return the PHY's temperature in Celsius and Signal Quality Indicator (SQI).  Wire: i\m\a */
ow_status ow_io_mdio_mdio_poll_sfp(ow_device* dev);

/* SFP Read. Reads a value from a register on the specified device address.  Wire: i\m\b */
ow_status ow_io_mdio_mdio_read_sfp(ow_device* dev, uint8_t device_address, const uint8_t* register_address, size_t register_address_len, uint32_t* sfp_response);

/* SFP Write. Writes a value to a register on the specified device address.  Wire: i\m\c */
ow_status ow_io_mdio_mdio_write_sfp(ow_device* dev, uint8_t device_address, const uint8_t* register_address, size_t register_address_len, const uint8_t* data_bytes, size_t data_bytes_len);

/* SFP Read-Modify-Write. Read-Modify-Writes a value to a register on the specified device address. '1' bits in the mask indicate an overwrite.  Wire: i\m\e */
ow_status ow_io_mdio_mdiormwsfp(ow_device* dev, uint8_t device_address, const uint8_t* register_address, size_t register_address_len, const uint8_t* mask_bytes, size_t mask_bytes_len, const uint8_t* data_bytes, size_t data_bytes_len);

/* PHY Address Poll. Polls all 32 possible PHY addresses. Test for a response from status register. Returns PHY addresses and clause compatibility.  Wire: i\m\y */
ow_status ow_io_mdio_mdio_poll(ow_device* dev);

/* Clause 22 Read. Reads a value from a register belonging to a Clause-22-Compatible-PHY.  Wire: i\m\g */
ow_status ow_io_mdio_mdio_read22(ow_device* dev, uint8_t phy_address, uint8_t register_address, uint32_t* mdio_response);

/* Clause 22 Write. Writes a value to a register belonging to a Clause-22-Compatible-PHY.  Wire: i\m\i */
ow_status ow_io_mdio_mdio_write22(ow_device* dev, uint8_t phy_address, uint8_t register_address, const uint8_t* data_bytes, size_t data_bytes_len);

/* Clause 22 Read-Modify-Write. Read-Modify-Writes a value to a register belonging to a Clause-45-Compatible-PHY. '1' bits in the mask indicate an overwrite.  Wire: i\m\j */
ow_status ow_io_mdio_mdiormw22(ow_device* dev, uint8_t phy_address, uint8_t register_address, const uint8_t* mask_bytes, size_t mask_bytes_len, const uint8_t* data_bytes, size_t data_bytes_len);

/* Clause 45 Read. Reads a value from a register belonging to a Clause-45-Compatible-PHY.  Wire: i\m\k */
ow_status ow_io_mdio_mdio_read45(ow_device* dev, uint8_t phy_address, uint8_t mmd_address, uint32_t register_address, uint32_t* mdio_response);

/* Clause 45 Write. Writes a value to a register belonging to a Clause-45-Compatible-PHY.  Wire: i\m\l */
ow_status ow_io_mdio_mdio_write45(ow_device* dev, uint8_t phy_address, uint8_t mmd_address, uint32_t register_address, const uint8_t* data_bytes, size_t data_bytes_len);

/* Clause 45 Read-Modify-Write. Read-Modify-Writes a value to a register belonging to a Clause-45-Compatible-PHY. '1' bits in the mask indicate an overwrite.  Wire: i\m\m */
ow_status ow_io_mdio_mdiormw45(ow_device* dev, uint8_t phy_address, uint8_t mmd_address, uint32_t register_address, const uint8_t* mask_bytes, size_t mask_bytes_len, const uint8_t* data_bytes, size_t data_bytes_len);

/* Clause 22 Access to Clause 45 Read. Reads a value from a register belonging to a Clause-45-Emulation-Compatible-PHY.  Wire: i\m\n */
ow_status ow_io_mdio_mdio_read_emu(ow_device* dev, uint8_t phy_address, uint8_t mmd_address, uint32_t register_address, uint32_t* mdio_response);

/* Clause 22 Access to Clause 45 Write. Writes a value to a register belonging to a Clause-45-Emulation-Compatible-PHY.  Wire: i\m\o */
ow_status ow_io_mdio_mdio_write_emu(ow_device* dev, uint8_t phy_address, uint8_t mmd_address, uint32_t register_address, const uint8_t* data_bytes, size_t data_bytes_len);

/* Clause 22 Access to Clause 45 Read-Modify-Write. Read-Modify-Writes a value to a register belonging to a Clause-45-Emulation-Compatible-PHY. '1' bits in the mask indicate an overwrite.  Wire: i\m\p */
ow_status ow_io_mdio_mdiormw_emu(ow_device* dev, uint8_t phy_address, uint8_t mmd_address, uint32_t register_address, const uint8_t* mask_bytes, size_t mask_bytes_len, const uint8_t* data_bytes, size_t data_bytes_len);

/* Stream Accel. Streams accelerometer (and temperature) data to the host at the given rate..  Wire: i\s\o */
ow_status ow_io_sensors_enable_accel_stream(ow_device* dev, int32_t stream_rate_ms);


/* Events emitted by Sensor Functions: */
/*   accel (text) - range=decS32, x=decS32, y=decS32, z=decS32, temp_c=decS32, temp_f=decS32, is_moving=bool - Accelerometer Data */
#define OW_EVENT_IO_SENSORS_ACCEL "accel"
/* Write. Writes data to a specific I2C Address.  Wire: i\i\w */
ow_status ow_io_i2c_i2c_write(ow_device* dev, uint8_t address, uint8_t register_, const uint8_t* data_bytes, size_t data_bytes_len);

/* Read. Reads the number from the address.  Wire: i\i\r */
ow_status ow_io_i2c_i2c_read(ow_device* dev, uint8_t* i2crepsone, size_t i2crepsone_cap, size_t* i2crepsone_len);

/* Poll. Tests all addresses for I2C Response.  Wire: i\i\p */
ow_status ow_io_i2c_i2c_poll(ow_device* dev);

/* Frequency. I2C bus clock frequency in Hz.  Wire: i\i\s\f */
ow_status ow_io_i2c_settings_frequency(ow_device* dev, int32_t value);

/* PullUps. Enable I2C bus pull-up resistors.  Wire: i\i\s\p */
ow_status ow_io_i2c_settings_pull_ups(ow_device* dev, int32_t value);

/* Write and Read. Writes data to SPI and returns response data.  Wire: i\e\w */
ow_status ow_io_spi_s_pi_write(ow_device* dev, const uint8_t* data_bytes, size_t data_bytes_len, uint8_t* spi_response, size_t spi_response_cap, size_t* spi_response_len);

/* Frequency. SPI clock frequency in Hz.  Wire: i\e\s\f */
ow_status ow_io_spi_settings_frequency(ow_device* dev, int32_t value);

/* Chip Select Pin. GPIO pin used as SPI chip select.  Wire: i\e\s\c */
ow_status ow_io_spi_settings_chip_select_pin(ow_device* dev, int32_t value);

/* Data Bits. SPI data bits per transfer.  Wire: i\e\s\b */
ow_status ow_io_spi_settings_data_bits(ow_device* dev, int32_t value);

/* CPOL. SPI clock polarity.  Wire: i\e\s\p */
ow_status ow_io_spi_settings_c_pol(ow_device* dev);

/* CPHA. SPI clock phase.  Wire: i\e\s\a */
ow_status ow_io_spi_settings_c_pha(ow_device* dev);

/* Stream CAN(FD). Streams received CAN frames and errors to the host..  Wire: i\c\o */
ow_status ow_io_canfd_enable_canfd_stream(ow_device* dev, int32_t channel, int32_t enabled);

/* Transmit CAN(FD). Transmits a CAN(FD) frame..  Wire: i\c\w */
ow_status ow_io_canfd_write_canfd(ow_device* dev, int32_t channel, uint32_t arb_id, int32_t can_fd, int32_t xtd_id, const uint8_t* data_in, size_t data_in_len);

/* Transmit CAN(FD) Periodic. Transmits a CAN(FD) frame periodically (period in us; 0 = as fast as possible)..  Wire: i\c\p */
ow_status ow_io_canfd_write_canfd_periodic(ow_device* dev, int32_t index, int32_t enable, int32_t period, int32_t channel, uint32_t arb_id, int32_t can_fd, int32_t xtd_id, const uint8_t* data_in, size_t data_in_len);

/* Setup Filter. Sets up a hardware receive filter (the byte-filter args are optional)..  Wire: i\c\f */
ow_status ow_io_canfd_setup_filter(ow_device* dev, int32_t channel, int32_t index, int32_t enable, int32_t xtd_id, uint32_t mask, uint32_t accept, uint32_t maskb0, uint32_t accept_b0, uint32_t maskb1, uint32_t accept_b1);

/* Read CAN Register(s). Reads 32-bit words from CAN controller SFR registers..  Wire: i\c\r */
ow_status ow_io_canfd_read_can_registers(ow_device* dev, int32_t channel, uint32_t start_address, int32_t word_count, char* registers, size_t registers_cap);

/* Set CAN Register. Sets a CAN controller register..  Wire: i\c\s */
ow_status ow_io_canfd_set_can_register(ow_device* dev, int32_t channel, uint32_t start_address, int32_t byte_count, uint32_t word_to_write);

/* Stream Analog In. Streams analog input values to the host at the given rate..  Wire: i\j\s */
ow_status ow_io_analog_in_enable_analog_in_stream(ow_device* dev, int32_t stream_rate_ms);

/* Set Analog Output. sets the voltage of an analog output 0 or 1. ch 2 and 3 are use for window comparator.  Wire: i\a\s */
ow_status ow_io_analog_out_set_analog_output(ow_device* dev, int32_t channel, double value);

/* Set Trigger Window. Trigger will be 1 when TrigV is between V- and V+..  Wire: i\a\t */
ow_status ow_io_analog_out_set_trigger_window(ow_device* dev, double value_low, double value_high);

/* Enable Trigger. Enables the Trigger Input to CPU.  Wire: i\a\e */
ow_status ow_io_analog_out_set_enable_trigger(ow_device* dev);

/* Set Programmable VOut. Sets the programmable VOut: enable then target voltage..  Wire: i\a\u */
ow_status ow_io_analog_out_set_v_prog_vout(ow_device* dev, int32_t enable, double set_voltage);

/* Glitch Programmable VOut. Briefly glitches the programmable VOut for the given nanoseconds..  Wire: i\a\g */
ow_status ow_io_analog_out_set_glitch(ow_device* dev, int32_t nano_seconds);

/* configure. Configures digital playback.  Wire: i\p\c */
ow_status ow_io_logic_player_setup_player(ow_device* dev, int32_t sample_rate_ns, int32_t sample_count, int32_t pin_start, int32_t pin_stop, int32_t start_mode, int32_t trigger_pin, bool loop);

/* configure analog. Configures DAC playback.  Wire: i\p\a */
ow_status ow_io_logic_player_setup_analog(ow_device* dev, int32_t mask, int32_t analog_rate_ns, int32_t analog_resolution);

/* load. Loads a raw buffer from the filesystem.  Wire: i\p\l */
ow_status ow_io_logic_player_load_file(ow_device* dev, const char* file_path);

/* start. Starts playback.  Wire: i\p\s */
ow_status ow_io_logic_player_start(ow_device* dev);

/* stop. Stops playback.  Wire: i\p\e */
ow_status ow_io_logic_player_stop(ow_device* dev);

/* configure. Configures the logic analyzer capture..  Wire: i\b\c */
ow_status ow_io_logic_analyzer_setup_logic_analyzer(ow_device* dev, int32_t sample_rate_ns, int32_t sample_count, int32_t pin_start, int32_t pin_stop, int32_t trigger_pin, int32_t trigger_type, int32_t rearm);

/* configure analog. Configures the analog capture inputs..  Wire: i\b\a */
ow_status ow_io_logic_analyzer_setup_analog(ow_device* dev, int32_t analog_mask, int32_t analog_rate_ns, int32_t analog_res);

/* start. Starts logic analyzer capture..  Wire: i\b\s */
ow_status ow_io_logic_analyzer_start(ow_device* dev);

/* stop. Stops logic analyzer capture..  Wire: i\b\e */
ow_status ow_io_logic_analyzer_stop(ow_device* dev);

/* trigger. Manually triggers the logic analyzer..  Wire: i\b\t */
ow_status ow_io_logic_analyzer_trigger(ow_device* dev, int32_t trigger_type);

/* Take a Picture. Take a picture from WILEye and save its SD card or FREE-WILi's Files system by file name..  Wire: i\f\t */
ow_status ow_io_wil_eye_take_picture(ow_device* dev, int32_t destination, const char* filename);

/* Start Recording Video. Start recording video from WILEye and save it to SD card by file name.  Wire: i\f\v */
ow_status ow_io_wil_eye_start_recording_video(ow_device* dev, const char* filename);

/* Stop Recording Video. Stop recording video from WILEye.  Wire: i\f\s */
ow_status ow_io_wil_eye_stop_recording_video(ow_device* dev);

/* Stream AI Detection Events. Stream AI Detection Events from WILEye.  Wire: i\f\a */
ow_status ow_io_wil_eye_toggle_ai_detection_stream(ow_device* dev, int32_t ai_stream_mode);

/* Set Zoom. Set the zoom level of WILEye.  Wire: i\f\m */
ow_status ow_io_wil_eye_set_zoom_level(ow_device* dev, int32_t zoom);

/* Set Contrast. Set the contrast level of WILEye.  Wire: i\f\c */
ow_status ow_io_wil_eye_set_contrast(ow_device* dev, int32_t contrast);

/* Set Saturation. Set the saturation level of WILEye.  Wire: i\f\i */
ow_status ow_io_wil_eye_set_saturation(ow_device* dev, int32_t saturation);

/* Set Brightness. Set the brightness level of WILEye.  Wire: i\f\b */
ow_status ow_io_wil_eye_set_brightness(ow_device* dev, int32_t brightness);

/* Set Hue. Set the hue level of WILEye.  Wire: i\f\u */
ow_status ow_io_wil_eye_set_hue(ow_device* dev, int32_t hue);

/* Set Resolution. Set the resolution state of WILEye.  Wire: i\f\y */
ow_status ow_io_wil_eye_set_resolution(ow_device* dev, int32_t resolutionstate);

/* Enable Disable Flash. Set the flash state of WILEye.  Wire: i\f\l */
ow_status ow_io_wil_eye_set_flash_state(ow_device* dev, bool flash);


/* Events emitted by WILEye Functions: */
/*   WILEye (text) - data_bytes=hexbytes - WILEye received AI Detection Data */
#define OW_EVENT_IO_WIL_EYE_WIL_EYE "WILEye"
/* Play Audio File. Plays a .wav file from the sounds directory..  Wire: i\k\f */
ow_status ow_io_audio_play_audio_file(ow_device* dev, const char* file_path);

/* Record Audio. Records audio to a file (blank name = auto-named)..  Wire: i\k\r */
ow_status ow_io_audio_record_audio_file(ow_device* dev, const char* file_name);

/* Play Audio Asset. Plays a built-in audio asset by index or name..  Wire: i\k\a */
ow_status ow_io_audio_play_audio_asset(ow_device* dev, const char* asset_name);

/* Stream Audio. Enables or disables audio streaming to the host..  Wire: i\k\s */
ow_status ow_io_audio_enable_audio_stream(ow_device* dev, int32_t enable);

/* Numbers to Speech. Speaks the given number aloud..  Wire: i\k\n */
ow_status ow_io_audio_numbers_to_speech(ow_device* dev, double number);

/* Play Tone. Plays a tone of the given frequency, duration, and amplitude..  Wire: i\k\t */
ow_status ow_io_audio_tone(ow_device* dev, double frequency, double duration_ms, double amplitude);

/* Text to Speech. Speaks the given text aloud (text to speech)..  Wire: i\k\v */
ow_status ow_io_audio_speak(ow_device* dev, const char* text);

/* Set Board LED. Sets a led to a specific color.  Wire: g\s */
ow_status ow_gui_set_led_color(ow_device* dev, int32_t ledindex, int32_t red, int32_t green, int32_t blue, int32_t duration, ow_ow_led_manager_led_mode mode);

/* Show FWI Image. Shows an freewili image (fwi) file from the file system..  Wire: g\l */
ow_status ow_gui_show_fwi_image(ow_device* dev, const char* filename);

/* Reset Display. Clears any GUI menu actions done to the display such as show image or show text.  Wire: g\t */
ow_status ow_gui_clear_display(ow_device* dev);

/* Show Text Display. Show text on the free wili display.  Wire: g\p */
ow_status ow_gui_show_text(ow_device* dev, const char* texttodisplay);

/* Read All Buttons. Sets the baud rate for I2C in Hz.  Wire: g\u */
ow_status ow_gui_read_all(ow_device* dev);

/* Stream Buttons. Sends GPIO values as a specific rate to host.  Wire: g\o */
ow_status ow_gui_stream_io(ow_device* dev, int32_t pin);

/* Show Asset Image. Reads the number from the address.  Wire: g\a */
ow_status ow_gui_show_image_asset_by_id(ow_device* dev, int32_t image_id);

/* Add Panel. Reinitializes the custom panel for controls..  Wire: g\c\a */
ow_status ow_gui_panels_add_panel(ow_device* dev, bool use_tile, int32_t tile_id, const char* color, bool show_menu);

/* Add Panel Picklist. Shows a panel that allows user to pick from a list..  Wire: g\c\b */
ow_status ow_gui_panels_add_panel_picklist(ow_device* dev, bool use_tile, int32_t tile_id, int32_t icon_id, int32_t log_index, const char* back_color, const char* fore_color, const char* caption);

/* Show Panel.  Wire: g\c\c */
ow_status ow_gui_panels_show_panel(ow_device* dev, int32_t index);

/* Add LED. Add a LED control to the panel..  Wire: g\b\a */
ow_status ow_gui_controls_add_led(ow_device* dev, int32_t index, int32_t x, int32_t y, int32_t color, int32_t size, bool inital_value);

/* Add LogList. Adds a Log control or a list control to the panel..  Wire: g\b\b */
ow_status ow_gui_controls_add_log_list(ow_device* dev, int32_t index, int32_t log, int32_t x, int32_t y, int32_t width, int32_t height, int32_t font_type, int32_t font_size, const char* back_color, const char* fore_color, bool list_mode);

/* Add Plot. Adds a plot to the panel..  Wire: g\b\c */
ow_status ow_gui_controls_add_plot(ow_device* dev, int32_t index, int32_t plot_data_index_bit_field, int32_t x, int32_t y, int32_t width, int32_t height, int32_t min_y, int32_t max_y, const char* back_color);

/* Add Number. add a numeric control to a panel.  Wire: g\b\l */
ow_status ow_gui_controls_add_number(ow_device* dev, int32_t index, int32_t x, int32_t y, int32_t width, int32_t font_type, int32_t font_size, const char* fore_color, const char* back_color, bool is_float, int32_t float_digit_count, bool is_hex_format, bool is_unsigned);

/* Add Text. Add static text to the panel.  Wire: g\b\e */
ow_status ow_gui_controls_add_text(ow_device* dev, int32_t index, int32_t x, int32_t y, int32_t font_type, int32_t font_size, const char* fore_color, const char* back_color, const char* text);

/* Add Bargraph. Add a bar graph to a panel..  Wire: g\b\f */
ow_status ow_gui_controls_add_bargraph(ow_device* dev, int32_t index, int32_t x, int32_t y, int32_t width, int32_t height, int32_t min, int32_t max, const char* bar_color);

/* Add Meter. Add a Meter control to a panel.  Wire: g\b\g */
ow_status ow_gui_controls_add_meter(ow_device* dev, int32_t index, int32_t x, int32_t y, int32_t width, int32_t height, int32_t min, int32_t max, const char* needle_color);

/* Add Button. Add a button control to a panel.  Wire: g\b\i */
ow_status ow_gui_controls_add_button(ow_device* dev, int32_t index, int32_t x, int32_t y, int32_t width, int32_t height, const char* fore_color, const char* back_color, const char* text);

/* Add Picture. Shows a ROM picture on the panel..  Wire: g\b\j */
ow_status ow_gui_controls_add_picture(ow_device* dev, int32_t index, int32_t x, int32_t y, int32_t picture_id);

/* Add Picture From File. Loads a picture from the file system.  Wire: g\b\k */
ow_status ow_gui_controls_add_picture_from_file(ow_device* dev, int32_t index, int32_t x, int32_t y, const char* picture_path);

/* Set Control Value Text. sets the text value of a control.  Wire: g\e\a */
ow_status ow_gui_control_properties_set_control_value_text(ow_device* dev, int32_t index, const char* text);

/* Set Control Value Int. Set the text value of a control.  Wire: g\e\b */
ow_status ow_gui_control_properties_set_control_value_int(ow_device* dev, int32_t index, int32_t value);

/* Set Control Value Float. Set the float value of the control..  Wire: g\e\c */
ow_status ow_gui_control_properties_set_control_value_float(ow_device* dev, int32_t index, double value);

/* Set List Item Text. Sets the text and color of a specific list item.  Wire: g\e\k */
ow_status ow_gui_control_properties_set_list_item_text(ow_device* dev, int32_t log_index, int32_t list_item, int32_t color, const char* text);

/* Set Control Value Min Max Int. Sets whether a min and max is applied to a controls value.  Wire: g\e\e */
ow_status ow_gui_control_properties_set_control_value_min_max_int(ow_device* dev, int32_t index, bool enable, int32_t min, int32_t max);

/* Set Control Value Min Max Float. Sets whether a min and max is applied to a controls value.  Wire: g\e\l */
ow_status ow_gui_control_properties_set_control_value_min_max_float(ow_device* dev, int32_t index, bool enable, double min, double max);

/* Set Plot Data. This adds data to a plot.  Wire: g\e\f */
ow_status ow_gui_control_properties_set_plot_data(ow_device* dev, int32_t plot_data_index, int32_t settings, int32_t value);

/* Set List Item Selected. This sets which item in a list is selected..  Wire: g\e\g */
ow_status ow_gui_control_properties_set_list_item_selected(ow_device* dev, int32_t log_index, int32_t list_index);

/* Set List Item Top Index. This sets the first viewable item in the list. .  Wire: g\e\i */
ow_status ow_gui_control_properties_set_list_item_top_index(ow_device* dev, int32_t log_item, int32_t list_index);

/* Set Control Property. Sets a property based on a property type index.  Wire: g\e\j */
ow_status ow_gui_control_properties_set_control_property(ow_device* dev, int32_t index, int32_t property, int32_t value);

/* Message Box. Shows a message box with optional buttons and auto close timer. .  Wire: g\f\a */
ow_status ow_gui_dialogs_message_box(ow_device* dev, int32_t auto_close_half_sec, bool show_ok, bool show_ok_cancel, bool show_none, int32_t picture_index, const char* message);

/* Set Dialog Description. Sets the description of the dialog..  Wire: g\f\b */
ow_status ow_gui_dialogs_set_dialog_description(ow_device* dev, const char* description);

/* Progress Bar. shows a dialog with a progress bar.  Wire: g\f\c */
ow_status ow_gui_dialogs_progress_bar(ow_device* dev, int32_t picture_index, bool ok_to_close, bool auto_close_at100, int32_t auto_close_half_sec, const char* title);

/* Number Edit. Shows a dialog box to edit numbers.  Wire: g\f\k */
ow_status ow_gui_dialogs_number_edit(ow_device* dev, int32_t min, int32_t max, int32_t initial, bool use_min_max, bool is_unsigned, bool hex_fomat, const char* message);

/* Number Edit Float. Shows a dialog to enter a float number.  Wire: g\f\e */
ow_status ow_gui_dialogs_number_edit_float(ow_device* dev, double min, double max, double initial, bool use_min_max, int32_t digit_count, const char* message);

/* Text Edit. Shows a dialog to edit a text value..  Wire: g\f\f */
ow_status ow_gui_dialogs_text_edit(ow_device* dev, const char* message, const char* inital_value);

/* Pick List. Shows a list of items to pick from. The list of items is loaded into a log..  Wire: g\f\g */
ow_status ow_gui_dialogs_pick_list(ow_device* dev, int32_t log_index, const char* message);

/* Show Text Editor. Shows a full screen text editor..  Wire: g\f\i */
ow_status ow_gui_dialogs_show_text_editor(ow_device* dev, int32_t editor_type, const char* message, const char* inital_value, bool* basic);

/* Set Progess Dialog Value. Sets the value of progress on the dialog.  Wire: g\f\j */
ow_status ow_gui_dialogs_set_progess_dialog_value(ow_device* dev, int32_t value0_to100);

/* Software Reset. Performs a software reset of the device..  Wire: h\s\1 */
ow_status ow_hardware_settings_home_software_reset(ow_device* dev);

/* Reset To Bootloader. Resets the device into the USB bootloader..  Wire: h\s\2 */
ow_status ow_hardware_settings_home_software_reset_to_bootloader(ow_device* dev);

/* All Settings To Defaults. Restores all settings to their default values..  Wire: h\s\3 */
ow_status ow_hardware_settings_home_all_settings_to_defaults(ow_device* dev);

/* Baud Rate. UART baud rate in bits per second.  Wire: h\s\u\f */
ow_status ow_hardware_settings_home_uart_settings_baud_rate(ow_device* dev, int32_t value);

/* RTS Hand Shaking. Enable RTS hardware handshaking.  Wire: h\s\u\r */
ow_status ow_hardware_settings_home_uart_settings_r_ts_hand_shaking(ow_device* dev);

/* CTS Hand Shaking. Enable CTS hardware handshaking.  Wire: h\s\u\c */
ow_status ow_hardware_settings_home_uart_settings_c_ts_hand_shaking(ow_device* dev);

/* Data Bits. UART data bits.  Wire: h\s\u\b */
ow_status ow_hardware_settings_home_uart_settings_data_bits(ow_device* dev, int32_t value);

/* Parity. UART parity mode.  Wire: h\s\u\p */
ow_status ow_hardware_settings_home_uart_settings_parity(ow_device* dev, int32_t value);

/* Stop Bits. UART stop bits.  Wire: h\s\u\s */
ow_status ow_hardware_settings_home_uart_settings_stop_bits(ow_device* dev, int32_t value);

/* Module. Which UART module handles the port.  Wire: h\s\u\m */
ow_status ow_hardware_settings_home_uart_settings_module(ow_device* dev, int32_t value);

/* Frequency. I2C bus clock frequency in Hz.  Wire: h\s\i\f */
ow_status ow_hardware_settings_home_i2c_settings_frequency(ow_device* dev, int32_t value);

/* PullUps. Enable I2C bus pull-up resistors.  Wire: h\s\i\p */
ow_status ow_hardware_settings_home_i2c_settings_pull_ups(ow_device* dev, int32_t value);

/* Frequency. SPI clock frequency in Hz.  Wire: h\s\s\f */
ow_status ow_hardware_settings_home_spi_settings_frequency(ow_device* dev, int32_t value);

/* Chip Select Pin. GPIO pin used as SPI chip select.  Wire: h\s\s\c */
ow_status ow_hardware_settings_home_spi_settings_chip_select_pin(ow_device* dev, int32_t value);

/* Data Bits. SPI data bits per transfer.  Wire: h\s\s\b */
ow_status ow_hardware_settings_home_spi_settings_data_bits(ow_device* dev, int32_t value);

/* CPOL. SPI clock polarity.  Wire: h\s\s\p */
ow_status ow_hardware_settings_home_spi_settings_c_pol(ow_device* dev);

/* CPHA. SPI clock phase.  Wire: h\s\s\a */
ow_status ow_hardware_settings_home_spi_settings_c_pha(ow_device* dev);

/* SPI1 Rx (12). IO direction for SPI1 Rx pin 12 (out/in).  Wire: h\s\o\a */
ow_status ow_hardware_settings_home_io_direction_settings_s_pi1_rx12(ow_device* dev);

/* GPIO 26 (26). IO direction for GPIO 26 (out/in).  Wire: h\s\o\b */
ow_status ow_hardware_settings_home_io_direction_settings_g_pio2626(ow_device* dev);

/* SPI1 CS (13). IO direction for SPI1 CS pin 13 (out/in).  Wire: h\s\o\c */
ow_status ow_hardware_settings_home_io_direction_settings_s_pi1cs13(ow_device* dev);

/* GPIO (27). IO direction for GPIO 27 (out/in).  Wire: h\s\o\l */
ow_status ow_hardware_settings_home_io_direction_settings_g_pio27(ow_device* dev);

/* UART1 Rx (9). IO direction for UART1 Rx pin 9 (out/in).  Wire: h\s\o\e */
ow_status ow_hardware_settings_home_io_direction_settings_u_art1_rx9(ow_device* dev);

/* UART1 CTS (10). IO direction for UART1 CTS pin 10 (out/in).  Wire: h\s\o\f */
ow_status ow_hardware_settings_home_io_direction_settings_u_art1cts10(ow_device* dev);

/* UART1 Tx (8). IO direction for UART1 Tx pin 8 (out/in).  Wire: h\s\o\g */
ow_status ow_hardware_settings_home_io_direction_settings_u_art1_tx8(ow_device* dev);

/* UART1 RTS (11). IO direction for UART1 RTS pin 11 (out/in).  Wire: h\s\o\m */
ow_status ow_hardware_settings_home_io_direction_settings_u_art1rts11(ow_device* dev);

/* SPI1 Tx (15). IO direction for SPI1 Tx pin 15 (out/in).  Wire: h\s\o\i */
ow_status ow_hardware_settings_home_io_direction_settings_s_pi1_tx15(ow_device* dev);

/* SPI1 SCLK (14). IO direction for SPI1 SCLK pin 14 (out/in).  Wire: h\s\o\j */
ow_status ow_hardware_settings_home_io_direction_settings_s_pi1sclk14(ow_device* dev);

/* GPIO25 (25). IO direction for GPIO 25 (out/in).  Wire: h\s\o\k */
ow_status ow_hardware_settings_home_io_direction_settings_g_pio2525(ow_device* dev);

/* Clk Source.  Wire: h\s\f\c */
ow_status ow_hardware_settings_home_fpga_clock_settings_clk_source(ow_device* dev, int32_t value);

/* Clk Divider (int).  Wire: h\s\f\i */
ow_status ow_hardware_settings_home_fpga_clock_settings_clk_divider_int(ow_device* dev, int32_t value);

/* Clk Divider (Frac).  Wire: h\s\f\f */
ow_status ow_hardware_settings_home_fpga_clock_settings_clk_divider_frac(ow_device* dev, int32_t value);

/* Comms Mode.  Wire: h\s\f\m */
ow_status ow_hardware_settings_home_fpga_clock_settings_comms_mode(ow_device* dev, int32_t value);

/* FrequencyMhz. basic frequency calculated automatically (default = 433.92). The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ.  Wire: h\s\r\f */
ow_status ow_hardware_settings_home_radio_settings_frequency_mhz(ow_device* dev, int32_t value);

/* Modulation. set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.  Wire: h\s\r\m */
ow_status ow_hardware_settings_home_radio_settings_modulation(ow_device* dev, int32_t value);

/* Devation. Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz..  Wire: h\s\r\a */
ow_status ow_hardware_settings_home_radio_settings_devation(ow_device* dev, int32_t value);

/* Channel. Channelnumber from 0 to 255. Default is channel 0.  Wire: h\s\r\b */
ow_status ow_hardware_settings_home_radio_settings_channel(ow_device* dev, int32_t value);

/* ChannelSpacing. channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz..  Wire: h\s\r\c */
ow_status ow_hardware_settings_home_radio_settings_channel_spacing(ow_device* dev, int32_t value);

/* RxBandwidth. Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz..  Wire: h\s\r\y */
ow_status ow_hardware_settings_home_radio_settings_rx_bandwidth(ow_device* dev, int32_t value);

/* DataRate. Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud.  Wire: h\s\r\e */
ow_status ow_hardware_settings_home_radio_settings_data_rate(ow_device* dev, int32_t value);

/* PowerAmp. TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max.  Wire: h\s\r\g */
ow_status ow_hardware_settings_home_radio_settings_power_amp(ow_device* dev, int32_t value);

/* SyncMode. Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync- carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold..  Wire: h\s\r\1 */
ow_status ow_hardware_settings_home_radio_settings_sync_mode(ow_device* dev, int32_t value);

/* SyncWord. sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low).  Wire: h\s\r\i */
ow_status ow_hardware_settings_home_radio_settings_sync_word(ow_device* dev, int32_t value);

/* AddrCheck. Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast..  Wire: h\s\r\j */
ow_status ow_hardware_settings_home_radio_settings_addr_check(ow_device* dev, int32_t value);

/* Address. Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF)..  Wire: h\s\r\k */
ow_status ow_hardware_settings_home_radio_settings_address(ow_device* dev, int32_t value);

/* WhiteData. Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on..  Wire: h\s\r\l */
ow_status ow_hardware_settings_home_radio_settings_white_data(ow_device* dev);

/* PacketFormat. Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins..  Wire: h\s\r\n */
ow_status ow_hardware_settings_home_radio_settings_packet_format(ow_device* dev, int32_t value);

/* LengthConfig. 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved.  Wire: h\s\r\o */
ow_status ow_hardware_settings_home_radio_settings_length_config(ow_device* dev, int32_t value);

/* PacketLength. Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed..  Wire: h\s\r\p */
ow_status ow_hardware_settings_home_radio_settings_packet_length(ow_device* dev, int32_t value);

/* CRCEnabled. 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX..  Wire: h\s\r\x */
ow_status ow_hardware_settings_home_radio_settings_c_rc_enabled(ow_device* dev);

/* CRCAutoFlush. Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size..  Wire: h\s\r\0 */
ow_status ow_hardware_settings_home_radio_settings_c_rc_auto_flush(ow_device* dev);

/* DCBlockingFilter. Disable digital DC blocking filter before demodulator. Only for data rates <= 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity)..  Wire: h\s\r\r */
ow_status ow_hardware_settings_home_radio_settings_d_c_blocking_filter(ow_device* dev);

/* Manchester. Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable..  Wire: h\s\r\s */
ow_status ow_hardware_settings_home_radio_settings_manchester(ow_device* dev);

/* ForwordErrorCorrection. Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable..  Wire: h\s\r\t */
ow_status ow_hardware_settings_home_radio_settings_forword_error_correction(ow_device* dev);

/* PreambleBytes. Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24.  Wire: h\s\r\u */
ow_status ow_hardware_settings_home_radio_settings_preamble_bytes(ow_device* dev, int32_t value);

/* PQT. Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4-PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted..  Wire: h\s\r\v */
ow_status ow_hardware_settings_home_radio_settings_p_qt(ow_device* dev, int32_t value);

/* AppendStatus. When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK..  Wire: h\s\r\w */
ow_status ow_hardware_settings_home_radio_settings_append_status(ow_device* dev);

/* FrequencyMhz. basic frequency calculated automatically (default = 433.92). The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ.  Wire: h\s\t\f */
ow_status ow_hardware_settings_home_radio_settings_2_frequency_mhz(ow_device* dev, int32_t value);

/* Modulation. set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.  Wire: h\s\t\m */
ow_status ow_hardware_settings_home_radio_settings_2_modulation(ow_device* dev, int32_t value);

/* Devation. Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz..  Wire: h\s\t\a */
ow_status ow_hardware_settings_home_radio_settings_2_devation(ow_device* dev, int32_t value);

/* Channel. Channelnumber from 0 to 255. Default is channel 0.  Wire: h\s\t\b */
ow_status ow_hardware_settings_home_radio_settings_2_channel(ow_device* dev, int32_t value);

/* ChannelSpacing. channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz..  Wire: h\s\t\c */
ow_status ow_hardware_settings_home_radio_settings_2_channel_spacing(ow_device* dev, int32_t value);

/* RxBandwidth. Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz..  Wire: h\s\t\y */
ow_status ow_hardware_settings_home_radio_settings_2_rx_bandwidth(ow_device* dev, int32_t value);

/* DataRate. Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud.  Wire: h\s\t\e */
ow_status ow_hardware_settings_home_radio_settings_2_data_rate(ow_device* dev, int32_t value);

/* PowerAmp. TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max.  Wire: h\s\t\g */
ow_status ow_hardware_settings_home_radio_settings_2_power_amp(ow_device* dev, int32_t value);

/* SyncMode. Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync- carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold..  Wire: h\s\t\1 */
ow_status ow_hardware_settings_home_radio_settings_2_sync_mode(ow_device* dev, int32_t value);

/* SyncWord. sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low).  Wire: h\s\t\i */
ow_status ow_hardware_settings_home_radio_settings_2_sync_word(ow_device* dev, int32_t value);

/* AddrCheck. Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast..  Wire: h\s\t\j */
ow_status ow_hardware_settings_home_radio_settings_2_addr_check(ow_device* dev, int32_t value);

/* Address. Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF)..  Wire: h\s\t\k */
ow_status ow_hardware_settings_home_radio_settings_2_address(ow_device* dev, int32_t value);

/* WhiteData. Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on..  Wire: h\s\t\l */
ow_status ow_hardware_settings_home_radio_settings_2_white_data(ow_device* dev);

/* PacketFormat. Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins..  Wire: h\s\t\n */
ow_status ow_hardware_settings_home_radio_settings_2_packet_format(ow_device* dev, int32_t value);

/* LengthConfig. 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved.  Wire: h\s\t\o */
ow_status ow_hardware_settings_home_radio_settings_2_length_config(ow_device* dev, int32_t value);

/* PacketLength. Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed..  Wire: h\s\t\p */
ow_status ow_hardware_settings_home_radio_settings_2_packet_length(ow_device* dev, int32_t value);

/* CRCEnabled. 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX..  Wire: h\s\t\x */
ow_status ow_hardware_settings_home_radio_settings_2_c_rc_enabled(ow_device* dev);

/* CRCAutoFlush. Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size..  Wire: h\s\t\0 */
ow_status ow_hardware_settings_home_radio_settings_2_c_rc_auto_flush(ow_device* dev);

/* DCBlockingFilter. Disable digital DC blocking filter before demodulator. Only for data rates <= 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity)..  Wire: h\s\t\r */
ow_status ow_hardware_settings_home_radio_settings_2_d_c_blocking_filter(ow_device* dev);

/* Manchester. Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable..  Wire: h\s\t\s */
ow_status ow_hardware_settings_home_radio_settings_2_manchester(ow_device* dev);

/* ForwordErrorCorrection. Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable..  Wire: h\s\t\t */
ow_status ow_hardware_settings_home_radio_settings_2_forword_error_correction(ow_device* dev);

/* PreambleBytes. Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24.  Wire: h\s\t\u */
ow_status ow_hardware_settings_home_radio_settings_2_preamble_bytes(ow_device* dev, int32_t value);

/* PQT. Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4-PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted..  Wire: h\s\t\v */
ow_status ow_hardware_settings_home_radio_settings_2_p_qt(ow_device* dev, int32_t value);

/* AppendStatus. When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK..  Wire: h\s\t\w */
ow_status ow_hardware_settings_home_radio_settings_2_append_status(ow_device* dev);

/* Default View. Default view for the RF Analyzer.  Wire: h\s\a\a */
ow_status ow_hardware_settings_home_radio_fa_settings_default_view(ow_device* dev, int32_t value);

/* Year.  Wire: h\s\c\y */
ow_status ow_hardware_settings_home_rtc_settings_year(ow_device* dev, int32_t value);

/* Month.  Wire: h\s\c\n */
ow_status ow_hardware_settings_home_rtc_settings_month(ow_device* dev, int32_t value);

/* Day.  Wire: h\s\c\e */
ow_status ow_hardware_settings_home_rtc_settings_day(ow_device* dev, int32_t value);

/* Day Of Week.  Wire: h\s\c\w */
ow_status ow_hardware_settings_home_rtc_settings_day_of_week(ow_device* dev, int32_t value);

/* Hours.  Wire: h\s\c\o */
ow_status ow_hardware_settings_home_rtc_settings_hours(ow_device* dev, int32_t value);

/* Minutes.  Wire: h\s\c\m */
ow_status ow_hardware_settings_home_rtc_settings_minutes(ow_device* dev, int32_t value);

/* Seconds.  Wire: h\s\c\s */
ow_status ow_hardware_settings_home_rtc_settings_seconds(ow_device* dev, int32_t value);

/* Trim. Add or subtract n*2 clock cycles every minute.  Wire: h\s\c\t */
ow_status ow_hardware_settings_home_rtc_settings_trim(ow_device* dev, int32_t value);

/* Channel 1.  Wire: h\s\n\a */
ow_status ow_hardware_settings_home_light_show_settings_channel1(ow_device* dev, int32_t value);

/* Channel 2.  Wire: h\s\n\b */
ow_status ow_hardware_settings_home_light_show_settings_channel2(ow_device* dev, int32_t value);

/* Channel 3.  Wire: h\s\n\c */
ow_status ow_hardware_settings_home_light_show_settings_channel3(ow_device* dev, int32_t value);

/* Channel 4.  Wire: h\s\n\f */
ow_status ow_hardware_settings_home_light_show_settings_channel4(ow_device* dev, int32_t value);

/* Use WiliWasm.  Wire: h\s\n\e */
ow_status ow_hardware_settings_home_light_show_settings_use_wili_wasm(ow_device* dev);

/* Enable Station Mode.  Wire: h\s\w\s */
ow_status ow_hardware_settings_home_wifi_settings_enable_station_mode(ow_device* dev);

/* SSID for Station Mode.  Wire: h\s\w\e */
ow_status ow_hardware_settings_home_wifi_settings_s_sid_for_station_mode(ow_device* dev, const char* value);

/* Password for Station Mode.  Wire: h\s\w\p */
ow_status ow_hardware_settings_home_wifi_settings_password_for_station_mode(ow_device* dev, const char* value);

/* Enable AP Mode.  Wire: h\s\w\a */
ow_status ow_hardware_settings_home_wifi_settings_enable_ap_mode(ow_device* dev);

/* AP Auth.  Wire: h\s\w\u */
ow_status ow_hardware_settings_home_wifi_settings_a_p_auth(ow_device* dev, int32_t value);

/* AP hide SSID.  Wire: h\s\w\i */
ow_status ow_hardware_settings_home_wifi_settings_a_p_hide_ssid(ow_device* dev);

/* SSID for AP.  Wire: h\s\w\g */
ow_status ow_hardware_settings_home_wifi_settings_s_sid_for_ap(ow_device* dev, const char* value);

/* Password for AP.  Wire: h\s\w\x */
ow_status ow_hardware_settings_home_wifi_settings_password_for_ap(ow_device* dev, const char* value);

/* Enable BT.  Wire: h\s\b\s */
ow_status ow_hardware_settings_home_ble_settings_enable_bt(ow_device* dev);

/* BT <-> Terminal.  Wire: h\s\b\t */
ow_status ow_hardware_settings_home_ble_settings_b_t_terminal(ow_device* dev);

/* BT Advert Name.  Wire: h\s\b\a */
ow_status ow_hardware_settings_home_ble_settings_b_t_advert_name(ow_device* dev, const char* value);

/* Orca Com over UART. Set Communication protocol for connected Orca device over UART.  Wire: h\s\g\u */
ow_status ow_hardware_settings_home_orca_settings_orca_com_over_uart(ow_device* dev, int32_t value);

/* Start WS Server.  Wire: h\s\k\r */
ow_status ow_hardware_settings_home_websocket_settings_start_ws_server(ow_device* dev);

/* WS Server Port.  Wire: h\s\k\p */
ow_status ow_hardware_settings_home_websocket_settings_w_s_server_port(ow_device* dev, int32_t value);

/* Auth Mode.  Wire: h\s\k\m */
ow_status ow_hardware_settings_home_websocket_settings_auth_mode(ow_device* dev, int32_t value);

/* Auth Username.  Wire: h\s\k\u */
ow_status ow_hardware_settings_home_websocket_settings_auth_username(ow_device* dev, const char* value);

/* Auth Password.  Wire: h\s\k\e */
ow_status ow_hardware_settings_home_websocket_settings_auth_password(ow_device* dev, const char* value);

/* CAN1 Mode. CAN Type or UART over CAN PHY.  Wire: h\s\p\a */
ow_status ow_hardware_settings_home_neptune_settings_c_an1_mode(ow_device* dev, int32_t value);

/* CAN1 Rate. Baudrate of CAN or UART over CAN PHY.  Wire: h\s\p\b */
ow_status ow_hardware_settings_home_neptune_settings_c_an1_rate(ow_device* dev, int32_t value);

/* CAN1 FD D Rate. Baud Rate for CANFD Data section.  Wire: h\s\p\c */
ow_status ow_hardware_settings_home_neptune_settings_c_an1fdd_rate(ow_device* dev, int32_t value);

/* CAN1 Listen Only. Enables Listen Only mode.  Wire: h\s\p\y */
ow_status ow_hardware_settings_home_neptune_settings_c_an1_listen_only(ow_device* dev);

/* CAN1 Tx Retry. CAN Transmit retry options.  Wire: h\s\p\e */
ow_status ow_hardware_settings_home_neptune_settings_c_an1_tx_retry(ow_device* dev, int32_t value);

/* CAN1 Cust Baud. Hex Value String for Register C1NBTCFG. Blank to disable..  Wire: h\s\p\f */
ow_status ow_hardware_settings_home_neptune_settings_c_an1_cust_baud(ow_device* dev, const char* value);

/* CAN1 Cust Data Baud. Hex Value String for Register C1DBTCFG.  Wire: h\s\p\g */
ow_status ow_hardware_settings_home_neptune_settings_c_an1_cust_data_baud(ow_device* dev, const char* value);

/* CAN1 Termination. Enables termination for network..  Wire: h\s\p\1 */
ow_status ow_hardware_settings_home_neptune_settings_c_an1_termination(ow_device* dev);

/* CAN1 API Enabled. Set Wili API Base ID.  Wire: h\s\p\i */
ow_status ow_hardware_settings_home_neptune_settings_c_an1api_enabled(ow_device* dev);

/* CAN API ID. Enables Terminal over CANFD.  Wire: h\s\p\j */
ow_status ow_hardware_settings_home_neptune_settings_c_anapiid(ow_device* dev, int32_t value);

/* CAN2 Mode. CAN Type or UART over CAN PHY.  Wire: h\s\p\k */
ow_status ow_hardware_settings_home_neptune_settings_c_an2_mode(ow_device* dev, int32_t value);

/* CAN2 Rate. Baudrate of CAN or UART over CAN PHY.  Wire: h\s\p\l */
ow_status ow_hardware_settings_home_neptune_settings_c_an2_rate(ow_device* dev, int32_t value);

/* CAN2 FD D Rate. Baud Rate for CANFD Data section.  Wire: h\s\p\m */
ow_status ow_hardware_settings_home_neptune_settings_c_an2fdd_rate(ow_device* dev, int32_t value);

/* CAN2 Listen Only. Enables Listen Only mode.  Wire: h\s\p\n */
ow_status ow_hardware_settings_home_neptune_settings_c_an2_listen_only(ow_device* dev);

/* CAN2 Tx Retry. CAN Transmit retry options.  Wire: h\s\p\o */
ow_status ow_hardware_settings_home_neptune_settings_c_an2_tx_retry(ow_device* dev, int32_t value);

/* CAN2 Cust Baud. Hex Value String for Register C1NBTCFG. Blank to disable..  Wire: h\s\p\p */
ow_status ow_hardware_settings_home_neptune_settings_c_an2_cust_baud(ow_device* dev, const char* value);

/* CAN2 Cust Data Baud. Hex Value String for Register C1DBTCFG.  Wire: h\s\p\r */
ow_status ow_hardware_settings_home_neptune_settings_c_an2_cust_data_baud(ow_device* dev, const char* value);

/* CAN2 Termination. Enables termination for network..  Wire: h\s\p\s */
ow_status ow_hardware_settings_home_neptune_settings_c_an2_termination(ow_device* dev);

/* CAN2 API Enabled. Enables Wili API over CANFD.  Wire: h\s\p\t */
ow_status ow_hardware_settings_home_neptune_settings_c_an2api_enabled(ow_device* dev);

/* LIN Master En. Enables LIN Master Pull Resistor.  Wire: h\s\p\u */
ow_status ow_hardware_settings_home_neptune_settings_l_in_master_en(ow_device* dev);

/* LIN Baud Rate. Baud Rate for LIN.  Wire: h\s\p\v */
ow_status ow_hardware_settings_home_neptune_settings_l_in_baud_rate(ow_device* dev, int32_t value);

/* Analog In En. Enables analog input measurement.  Wire: h\s\p\x */
ow_status ow_hardware_settings_home_neptune_settings_analog_in_en(ow_device* dev);

/* Startup Wasm Script. Path to wasm or RTHON script..  Wire: h\s\e\a */
ow_status ow_hardware_settings_home_general_settings_startup_wasm_script(ow_device* dev, const char* value);

/* Startup Zoom Script. Path to zoom script..  Wire: h\s\e\b */
ow_status ow_hardware_settings_home_general_settings_startup_zoom_script(ow_device* dev, const char* value);

/* Default FPGA Script. Path to FPGA bit file.  Wire: h\s\e\c */
ow_status ow_hardware_settings_home_general_settings_default_fpga_script(ow_device* dev, const char* value);

/* Wasm debug level. Debug messaging from WiliWasm.  Wire: h\s\e\f */
ow_status ow_hardware_settings_home_general_settings_wasm_debug_level(ow_device* dev, int32_t value);

/* Stream Battery Info. Enables or disables streaming of battery info to the host..  Wire: h\a\o */
ow_status ow_hardware_system_enable_battery_stream(ow_device* dev, int32_t enable);

/* Enable Reader. Enable/disable NFC reader with auto tag streaming.  Wire: w\n\r */
ow_status ow_wireless_nfc_enable_reader(ow_device* dev, int32_t enable);

/* Connect To Bootloader. Instruct the ESP32 to enter into bootloader.  Wire: w\a\b */
ow_status ow_wireless_esp32_flasher_enter_bootloader(ow_device* dev, int32_t upgrade_transmission_rate);

/* Reset. Instruct the ESP32 to enter into application.  Wire: w\a\r */
ow_status ow_wireless_esp32_flasher_enter_application(ow_device* dev);

/* Read Chip ID And Security Info. Toggle ESP32's Enable Pin.  Wire: w\a\i */
ow_status ow_wireless_esp32_flasher_get_i_dand_security(ow_device* dev, int32_t* esp_chip_id, int32_t* version, bool* sb_en, bool* sbar_en, bool* sdm_en, bool* sbrk_1, bool* sbrk_2, bool* sbrk_3, bool* jtag_sw_dis, bool* jtag_hw_dis, bool* flash_enc_en, bool* dcache_dis, bool* icache_dis);

/* Read Flash Size. Toggle ESP32's Enable Pin.  Wire: w\a\k */
ow_status ow_wireless_esp32_flasher_read_flash_size(ow_device* dev, int32_t* flash_size_bytes);

/* Read MAC. Returns MAC of esp32.  Wire: w\a\m */
ow_status ow_wireless_esp32_flasher_read_esp32mac(ow_device* dev, char* esp32_mac, size_t esp32_mac_cap);

/* Erase All Flash. Toggle ESP32's Enable Pin.  Wire: w\a\e */
ow_status ow_wireless_esp32_flasher_erase_all_flash(ow_device* dev);

/* Start Writing Flash Operations. Prepares ESP32 to write flash at offset and expected size. Block size can be up to 128 bytes.  Wire: w\a\f */
ow_status ow_wireless_esp32_flasher_start_flash_operations(ow_device* dev, uint32_t offset, int32_t size, int32_t block_size);

/* Finish Flash Writing Operations. Ends ESP32 Flashing Operations..  Wire: w\a\p */
ow_status ow_wireless_esp32_flasher_stop_flash_operation(ow_device* dev, bool reboot);

/* Write Flash. Writes Binary Blob into flash.  Wire: w\a\o */
ow_status ow_wireless_esp32_flasher_flash_write(ow_device* dev, const uint8_t* flash_data, size_t flash_data_len);

/* Read Flash. Reads binary blob from flash with given address and size..  Wire: w\a\j */
ow_status ow_wireless_esp32_flasher_flash_read(ow_device* dev, uint32_t offset, int32_t size);

/* Start Memory Write Operations. Perpares memeory write operations on the esp32. Max Block Size size is 128.  Wire: w\a\y */
ow_status ow_wireless_esp32_flasher_start_write_memory_operations(ow_device* dev, uint32_t offset, uint32_t memory_block, int32_t block_size);

/* Write Memory. Perpares memeory write operations on the esp32. Max Block Size size is 128.  Wire: w\a\0 */
ow_status ow_wireless_esp32_flasher_memory_write(ow_device* dev, uint32_t offset, uint32_t memory_block, int32_t block_size);

/* Stop Memory Write Operations. Disables memory write operations on esp32 and sets entry point in ram.  Wire: w\a\t */
ow_status ow_wireless_esp32_flasher_stop_memory_operation(ow_device* dev, uint32_t entry_address);

/* Write Register. Writes a 4 byte value onto a register in the esp32.  Wire: w\a\g */
ow_status ow_wireless_esp32_flasher_register_write(ow_device* dev, uint32_t offset, uint32_t value);

/* Read Register. Reads a 4 byte value from a register in the esp32.  Wire: w\a\c */
ow_status ow_wireless_esp32_flasher_register_read(ow_device* dev, uint32_t offset, uint32_t* memory_block);

/* Flash Default App. Flash default application onto ESP32.  Wire: w\a\n */
ow_status ow_wireless_esp32_flasher_flash_default(ow_device* dev);

/* Enable Wifi Events. Toggle Wifi Event Streaming.  Wire: w\w\r */
ow_status ow_wireless_wifi_toggle_events(ow_device* dev);

/* Start Access Point. Starts up Access Point with provided SSID and Password.  Wire: w\w\a */
ow_status ow_wireless_wifi_on_start_access_point(ow_device* dev, const char* ssid, const char* password, int32_t authmode, bool hidessid);

/* Stop Access Point. Turns off Access Point.  Wire: w\w\t */
ow_status ow_wireless_wifi_on_discconect_from_station(ow_device* dev);

/* Get Stations connected to AP. Turns off Access Point.  Wire: w\w\g */
ow_status ow_wireless_wifi_get_connected_devices(ow_device* dev);

/* Connect to a Wifi Access Point. Connect to a WAP with provided SSID and Password.  Wire: w\w\c */
ow_status ow_wireless_wifi_on_connect_to_station(ow_device* dev, const char* ssid, const char* password);

/* Disconnect From Wifi Access Point. Disconnect from Wifi Stations.  Wire: w\w\f */
ow_status ow_wireless_wifi_on_discconect_from_station_2(ow_device* dev);

/* Scan for Access Points. Scans for available WIFI networks.  Wire: w\w\s */
ow_status ow_wireless_wifi_on_scan_for_access_points(ow_device* dev);

/* Print out Wifi Info. Scans for available Wifi networks.  Wire: w\w\p */
ow_status ow_wireless_wifi_on_get_wif_info(ow_device* dev);

/* Enable Station Mode.  Wire: w\w\e\s */
ow_status ow_wireless_wifi_settings_enable_station_mode(ow_device* dev);

/* SSID for Station Mode.  Wire: w\w\e\e */
ow_status ow_wireless_wifi_settings_s_sid_for_station_mode(ow_device* dev, const char* value);

/* Password for Station Mode.  Wire: w\w\e\p */
ow_status ow_wireless_wifi_settings_password_for_station_mode(ow_device* dev, const char* value);

/* Enable AP Mode.  Wire: w\w\e\a */
ow_status ow_wireless_wifi_settings_enable_ap_mode(ow_device* dev);

/* AP Auth.  Wire: w\w\e\u */
ow_status ow_wireless_wifi_settings_a_p_auth(ow_device* dev, int32_t value);

/* AP hide SSID.  Wire: w\w\e\i */
ow_status ow_wireless_wifi_settings_a_p_hide_ssid(ow_device* dev);

/* SSID for AP.  Wire: w\w\e\g */
ow_status ow_wireless_wifi_settings_s_sid_for_ap(ow_device* dev, const char* value);

/* Password for AP.  Wire: w\w\e\x */
ow_status ow_wireless_wifi_settings_password_for_ap(ow_device* dev, const char* value);

/* Start BT Advertising. Sets the Host Name for the Bluetooth LE.  Wire: w\b\a */
ow_status ow_wireless_bluetooth_le_on_start_bt_advertising(ow_device* dev, const char* hostname);

/* Stop BT Advertising. Stops BT Advertising.  Wire: w\b\t */
ow_status ow_wireless_bluetooth_le_on_stop_bt_advertising(ow_device* dev);

/* Scan for BT Devices. Scans for BT devices for a given duration.  Wire: w\b\s */
ow_status ow_wireless_bluetooth_le_on_scan_bt_devices(ow_device* dev, int32_t durationms);

/* Toggle Enable Terminal API Mode. Enables BLE to FreeWili Terminal API Mode.  Wire: w\b\e */
ow_status ow_wireless_bluetooth_le_on_enable_terminal(ow_device* dev);

/* Enable BT.  Wire: w\b\b\s */
ow_status ow_wireless_bluetooth_le_ble_settings_enable_bt(ow_device* dev);

/* BT <-> Terminal.  Wire: w\b\b\t */
ow_status ow_wireless_bluetooth_le_ble_settings_b_t_terminal(ow_device* dev);

/* BT Advert Name.  Wire: w\b\b\a */
ow_status ow_wireless_bluetooth_le_ble_settings_b_t_advert_name(ow_device* dev, const char* value);

/* Stream IR. Enables or disables streaming of received IR codes to the host..  Wire: w\i\o */
ow_status ow_wireless_ir_enable_ir_stream(ow_device* dev, int32_t enable);

/* Send IR. Transmits a 4-byte IR code..  Wire: w\i\a */
ow_status ow_wireless_ir_send_ir_data(ow_device* dev, int32_t ir_code);

/* Launch Script.  Wire: s\a */
ow_status ow_scripting_launch_script(ow_device* dev);

/* Stream ZoomIO Data. Enables or disables streaming of ZoomIO receive data to the host..  Wire: s\b\o */
ow_status ow_scripting_zoom_io_enable_rx_stream(ow_device* dev, int32_t enable);

/* Write to FIFO. Sends a single ZoomIO message after the given delay (us)..  Wire: s\b\w */
ow_status ow_scripting_zoom_io_send_data(ow_device* dev, int32_t delay, const uint8_t* data, size_t data_len);

/* Update Schedule Table. Updates a schedule-table transmit message..  Wire: s\b\u */
ow_status ow_scripting_zoom_io_update_table_data(ow_device* dev, int32_t table_index, int32_t delay, const uint8_t* data, size_t data_len);

/* Setup Schedule Table. Sets up the schedule table size (0 to disable)..  Wire: s\b\p */
ow_status ow_scripting_zoom_io_enable_schedule_table(ow_device* dev, int32_t number_of_entries);

/* Compile test. Compiles built-in ZoomIO milestone program and launches it on core1 as RISC-V.  Wire: s\b\c */
ow_status ow_scripting_zoom_io_compile_test(ow_device* dev);

/* Run ZoomIO. Compile and run a ZoomIO program on the RISC-V core1.  Wire: s\b\r */
ow_status ow_scripting_zoom_io_run_zio(ow_device* dev, const char* path);

/* Stop ZoomIO. Reset core1 to stop the running program.  Wire: s\b\s */
ow_status ow_scripting_zoom_io_stop_zio(ow_device* dev);

/* Launch App.  Wire: a\a */
ow_status ow_apps_launch_app(ow_device* dev);

/* Enable Linux CPU.  Wire: l\a */
ow_status ow_linux_enable_linux_cpu(ow_device* dev);


#ifdef __cplusplus
}
#endif
#endif /* ONEWILI_H */

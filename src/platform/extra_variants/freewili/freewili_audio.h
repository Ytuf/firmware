#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// One-time init for the NAU88C10 codec + I2S master + 22 MHz PWM MCLK.
// Idempotent — safe to call multiple times. Returns false if the codec
// didn't ACK on I2C1 (then audio calls below are no-ops).
bool freewili_audio_init(void);

// Generate a square-wave tone at freq_hz for duration_ms via I2S DMA into
// the codec. Blocking but fast for short durations (~10–200 ms).
void freewili_audio_tone(uint32_t freq_hz, uint32_t duration_ms);

// Predefined "event" sounds wired up to RX, TX, button-press etc.
void freewili_audio_play_rx(void);
void freewili_audio_play_tx(void);
void freewili_audio_play_click(void);
void freewili_audio_play_error(void);

#ifdef __cplusplus
}
#endif

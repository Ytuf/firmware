#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool freewili_audio_init(void);
void freewili_audio_tone(uint32_t freq_hz, uint32_t duration_ms);
void freewili_audio_play_rx(void);
void freewili_audio_play_tx(void);
void freewili_audio_play_click(void);
void freewili_audio_play_error(void);

#ifdef __cplusplus
}
#endif

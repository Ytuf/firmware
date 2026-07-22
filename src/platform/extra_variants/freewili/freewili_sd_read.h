#pragma once
#include <stdint.h>
#include <stddef.h>
void freewili_sd_read_init();
size_t freewili_sd_read(const char *volpath, uint8_t *buf, size_t cap);
extern "C" {
extern volatile uint32_t g_fwsdread_ok, g_fwsdread_fail, g_fwsdread_last_len;
extern volatile int32_t  g_fwsdread_last_status;
}

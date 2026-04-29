#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool amp_init(void);
bool amp_play_audio(const uint8_t *data, size_t len);
bool amp_stream_start(uint32_t sample_rate_hz);
bool amp_stream_write(const uint8_t *data, size_t len, uint32_t timeout_ms);
void amp_stream_stop(void);

#ifdef __cplusplus
}
#endif

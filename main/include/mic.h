#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sample_rate;
    uint8_t channels;
    uint8_t bits_per_sample;
    uint32_t max_record_time_ms;
    uint32_t min_record_time_ms;
    uint32_t vad_no_speech_timeout_ms;
    uint32_t vad_speech_confirm_ms;
    uint32_t vad_silence_end_ms;
    uint32_t vad_min_speech_ms;
    int16_t vad_min_energy;
    int16_t vad_energy_offset;
} mic_config_t;

mic_config_t mic_get_default_config(void);
bool mic_init(void);
bool mic_init_with_config(const mic_config_t *config);
bool mic_read(void *dst, size_t len, size_t *out_read, uint32_t timeout_ms);
bool mic_read_pcm16(int16_t *dst, size_t samples, size_t *out_samples, uint32_t timeout_ms);
bool mic_record_wav(uint8_t **out_wav, size_t *out_len);

#ifdef __cplusplus
}
#endif

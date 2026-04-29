#include "mic.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mem_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if __has_include("driver/i2s_std.h")
#include "driver/i2s_std.h"
#define USE_I2S_STD 1
static i2s_chan_handle_t g_rx_chan = NULL;
#else
#include "driver/i2s.h"
#define USE_I2S_STD 0
#endif

static const char *TAG = "mic";
static const bool I2S_MIC_DATA_ON_LEFT_SLOT = true;
static const int INMP441_DATA_RIGHT_SHIFT = 7;

static mic_config_t g_mic_config = {
    .sample_rate = APP_VOICE_SAMPLE_RATE_HZ,
    .channels = 1,
    .bits_per_sample = 32,
    .max_record_time_ms = 10000,
    .min_record_time_ms = 300,
    .vad_no_speech_timeout_ms = 2800,
    .vad_speech_confirm_ms = 60,
    .vad_silence_end_ms = 700,
    .vad_min_speech_ms = 300,
    .vad_min_energy = 24,
    .vad_energy_offset = 14,
};

static uint32_t get_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static int32_t pcm16_mean_abs(const int16_t *samples, size_t sample_count) {
    if (samples == NULL || sample_count == 0U) {
        return 0;
    }

    int64_t sum = 0;
    for (size_t i = 0U; i < sample_count; ++i) {
        const int32_t v = samples[i];
        sum += (v >= 0) ? v : -v;
    }
    return (int32_t)(sum / (int64_t)sample_count);
}

mic_config_t mic_get_default_config(void) {
    return g_mic_config;
}

static void write_wav_header(uint8_t *out44, uint32_t pcm_bytes) {
    const uint32_t sample_rate = g_mic_config.sample_rate;
    const uint8_t channels = g_mic_config.channels;
    const uint8_t bits_per_sample = 16;
    const uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8U);
    const uint16_t block_align = channels * (bits_per_sample / 8U);
    const uint32_t riff_size = 36U + pcm_bytes;

    memcpy(out44 + 0, "RIFF", 4);
    out44[4] = (uint8_t)(riff_size & 0xFFU);
    out44[5] = (uint8_t)((riff_size >> 8) & 0xFFU);
    out44[6] = (uint8_t)((riff_size >> 16) & 0xFFU);
    out44[7] = (uint8_t)((riff_size >> 24) & 0xFFU);
    memcpy(out44 + 8, "WAVEfmt ", 8);
    out44[16] = 16;
    out44[20] = 1;
    out44[22] = channels;
    out44[24] = (uint8_t)(sample_rate & 0xFFU);
    out44[25] = (uint8_t)((sample_rate >> 8) & 0xFFU);
    out44[26] = (uint8_t)((sample_rate >> 16) & 0xFFU);
    out44[27] = (uint8_t)((sample_rate >> 24) & 0xFFU);
    out44[28] = (uint8_t)(byte_rate & 0xFFU);
    out44[29] = (uint8_t)((byte_rate >> 8) & 0xFFU);
    out44[30] = (uint8_t)((byte_rate >> 16) & 0xFFU);
    out44[31] = (uint8_t)((byte_rate >> 24) & 0xFFU);
    out44[32] = (uint8_t)(block_align & 0xFFU);
    out44[33] = (uint8_t)((block_align >> 8) & 0xFFU);
    out44[34] = bits_per_sample;
    memcpy(out44 + 36, "data", 4);
    out44[40] = (uint8_t)(pcm_bytes & 0xFFU);
    out44[41] = (uint8_t)((pcm_bytes >> 8) & 0xFFU);
    out44[42] = (uint8_t)((pcm_bytes >> 16) & 0xFFU);
    out44[43] = (uint8_t)((pcm_bytes >> 24) & 0xFFU);
}

static size_t convert_32bit_to_16bit(const uint8_t *src, uint8_t *dst, size_t src_bytes) {
    const size_t src_samples = src_bytes / 4U;
    for (size_t i = 0U; i < src_samples; ++i) {
        uint32_t raw = 0U;
        raw |= (uint32_t)src[i * 4U];
        raw |= (uint32_t)src[i * 4U + 1U] << 8;
        raw |= (uint32_t)src[i * 4U + 2U] << 16;
        raw |= (uint32_t)src[i * 4U + 3U] << 24;
        const int32_t sample_24 = (int32_t)raw >> INMP441_DATA_RIGHT_SHIFT;
        const int16_t sample_16 = (int16_t)(sample_24 >> 8);
        dst[i * 2U] = (uint8_t)(sample_16 & 0xFF);
        dst[i * 2U + 1U] = (uint8_t)((sample_16 >> 8) & 0xFF);
    }
    return src_samples * 2U;
}

static bool init_i2s_std(void) {
#if USE_I2S_STD
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 8,
        .dma_frame_num = 128,
        .auto_clear = true,
    };
    if (i2s_new_channel(&chan_cfg, NULL, &g_rx_chan) != ESP_OK) {
        return false;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(g_mic_config.sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = (gpio_num_t)APP_VOICE_I2S_BCLK_PIN,
                .ws = (gpio_num_t)APP_VOICE_I2S_WS_PIN,
                .dout = I2S_GPIO_UNUSED,
                .din = (gpio_num_t)APP_VOICE_I2S_DIN_PIN,
            },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    std_cfg.slot_cfg.slot_mask = I2S_MIC_DATA_ON_LEFT_SLOT ? I2S_STD_SLOT_LEFT : I2S_STD_SLOT_RIGHT;

    if (i2s_channel_init_std_mode(g_rx_chan, &std_cfg) != ESP_OK) {
        return false;
    }
    return i2s_channel_enable(g_rx_chan) == ESP_OK;
#else
    return false;
#endif
}

bool mic_init(void) {
    return mic_init_with_config(&g_mic_config);
}

bool mic_init_with_config(const mic_config_t *config) {
    if (config != NULL) {
        g_mic_config = *config;
    }
    if (!init_i2s_std()) {
        ESP_LOGE(TAG, "mic init failed");
        return false;
    }
    return true;
}

bool mic_read(void *dst, size_t len, size_t *out_read, uint32_t timeout_ms) {
    size_t bytes_read = 0U;
    esp_err_t err = ESP_FAIL;
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
#if USE_I2S_STD
    if (g_rx_chan != NULL) {
        err = i2s_channel_read(g_rx_chan, dst, len, &bytes_read, ticks);
    }
#else
    err = i2s_read(I2S_NUM_0, dst, len, &bytes_read, ticks);
#endif
    if (out_read != NULL) {
        *out_read = bytes_read;
    }
    return err == ESP_OK || bytes_read > 0U;
}

bool mic_read_pcm16(int16_t *dst, size_t samples, size_t *out_samples, uint32_t timeout_ms) {
    if (out_samples != NULL) {
        *out_samples = 0U;
    }
    if (dst == NULL || samples == 0U) {
        return false;
    }

    size_t total_samples = 0U;
    uint8_t temp32[256];
    const uint32_t start_ms = get_time_ms();
    while (total_samples < samples) {
        size_t need_samples = samples - total_samples;
        if (need_samples > 64U) {
            need_samples = 64U;
        }

        const uint32_t elapsed_ms = get_time_ms() - start_ms;
        if (elapsed_ms >= timeout_ms) {
            break;
        }
        size_t bytes_read = 0U;
        if (!mic_read(temp32, need_samples * 4U, &bytes_read, timeout_ms - elapsed_ms)) {
            continue;
        }

        const size_t safe_bytes = (bytes_read / 4U) * 4U;
        if (safe_bytes == 0U) {
            continue;
        }
        const size_t converted = convert_32bit_to_16bit(temp32, (uint8_t *)(dst + total_samples), safe_bytes);
        total_samples += converted / sizeof(int16_t);
    }

    if (out_samples != NULL) {
        *out_samples = total_samples;
    }
    return total_samples > 0U;
}

bool mic_record_wav(uint8_t **out_wav, size_t *out_len) {
    if (out_wav == NULL || out_len == NULL) {
        return false;
    }
    *out_wav = NULL;
    *out_len = 0U;

    const uint32_t sample_rate = g_mic_config.sample_rate;
    const uint32_t channels = g_mic_config.channels;
    const uint32_t max_record_time_ms = g_mic_config.max_record_time_ms;
    const uint32_t min_record_time_ms = g_mic_config.min_record_time_ms;
    const uint32_t vad_no_speech_timeout_ms = g_mic_config.vad_no_speech_timeout_ms;
    const uint32_t vad_speech_confirm_ms = g_mic_config.vad_speech_confirm_ms;
    const uint32_t vad_silence_end_ms = g_mic_config.vad_silence_end_ms;
    const uint32_t vad_min_speech_ms = g_mic_config.vad_min_speech_ms;
    const int32_t vad_min_energy = g_mic_config.vad_min_energy;
    const int32_t vad_energy_offset = g_mic_config.vad_energy_offset;

    const uint32_t max_pcm_bytes = (sample_rate * max_record_time_ms / 1000U) * channels * 2U;
    const size_t max_wav_len = 44U + (size_t)max_pcm_bytes;
    uint8_t *wav = (uint8_t *)app_mem_alloc(max_wav_len, MEM_TYPE_LARGE);
    if (wav == NULL) {
        return false;
    }
    memset(wav, 0, max_wav_len);

    uint8_t *temp32 = (uint8_t *)app_mem_alloc(2048U, MEM_TYPE_SMALL);
    if (temp32 == NULL) {
        app_mem_free(wav);
        return false;
    }

    size_t written_16bit = 0U;
    const uint32_t start_ms = get_time_ms();
    bool speech_started = false;
    uint32_t speech_run_ms = 0U;
    uint32_t silence_run_ms = 0U;
    uint32_t speech_total_ms = 0U;
    int32_t noise_floor = vad_min_energy;
    bool noise_floor_inited = false;

    while (written_16bit < max_pcm_bytes) {
        size_t bytes_read_32 = 0U;
        if (!mic_read(temp32, 512U, &bytes_read_32, 50U)) {
            vTaskDelay(1);
        } else {
            size_t safe_bytes_32 = (bytes_read_32 / 4U) * 4U;
            const size_t remaining_16 = max_pcm_bytes - written_16bit;
            if ((safe_bytes_32 / 2U) > remaining_16) {
                safe_bytes_32 = remaining_16 * 2U;
            }
            const size_t converted = convert_32bit_to_16bit(temp32, wav + 44U + written_16bit, safe_bytes_32);
            if (converted > 0U) {
                int16_t *chunk_pcm = (int16_t *)(wav + 44U + written_16bit);
                const size_t chunk_samples = converted / sizeof(int16_t);
                uint32_t chunk_ms = (uint32_t)((chunk_samples * 1000U + sample_rate - 1U) / sample_rate);
                if (chunk_ms == 0U) {
                    chunk_ms = 1U;
                }
                const int32_t mean_abs = pcm16_mean_abs(chunk_pcm, chunk_samples);
                if (!noise_floor_inited) {
                    noise_floor = mean_abs;
                    noise_floor_inited = true;
                } else if (!speech_started) {
                    noise_floor = (noise_floor * 31 + mean_abs) / 32;
                }

                int32_t threshold = noise_floor + vad_energy_offset;
                if (threshold < vad_min_energy) {
                    threshold = vad_min_energy;
                }

                if (mean_abs >= threshold) {
                    speech_run_ms += chunk_ms;
                    silence_run_ms = 0U;
                    speech_total_ms += chunk_ms;
                } else {
                    silence_run_ms += chunk_ms;
                    speech_run_ms = 0U;
                }

                if (!speech_started && speech_run_ms >= vad_speech_confirm_ms) {
                    speech_started = true;
                }
                written_16bit += converted;
                if (speech_started && silence_run_ms >= vad_silence_end_ms && speech_total_ms >= vad_min_speech_ms) {
                    break;
                }
            }
        }

        const uint32_t now_ms = get_time_ms();
        if (!speech_started && (now_ms - start_ms) >= vad_no_speech_timeout_ms) {
            app_mem_free(temp32);
            app_mem_free(wav);
            return false;
        }
        if ((now_ms - start_ms) >= max_record_time_ms) {
            break;
        }
    }

    app_mem_free(temp32);
    const uint32_t record_time_ms = get_time_ms() - start_ms;
    if (!speech_started || speech_total_ms < vad_min_speech_ms || record_time_ms < min_record_time_ms || written_16bit == 0U) {
        app_mem_free(wav);
        return false;
    }

    write_wav_header(wav, (uint32_t)written_16bit);
    *out_wav = wav;
    *out_len = 44U + written_16bit;
    return true;
}

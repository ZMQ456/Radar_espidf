#include "amp.h"

#include <limits.h>
#include <string.h>

#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if __has_include("driver/i2s_std.h")
#include "driver/i2s_std.h"
#define USE_I2S_STD 1
static i2s_chan_handle_t g_tx_chan = NULL;
#else
#include "driver/i2s.h"
#define USE_I2S_STD 0
#endif

static const char *TAG = "amp";
static const size_t AMP_CHUNK_SIZE = 1024U;
static const float AMP_DEFAULT_GAIN = 1.0f;
static bool g_stream_active = false;

static uint32_t get_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

bool amp_init(void) {
#if USE_I2S_STD
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = 512;
    if (i2s_new_channel(&chan_cfg, &g_tx_chan, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "create tx channel failed");
        return false;
    }

    i2s_std_config_t std_cfg = {};
    std_cfg.clk_cfg.sample_rate_hz = APP_VOICE_SAMPLE_RATE_HZ;
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO;
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    std_cfg.slot_cfg.ws_width = I2S_DATA_BIT_WIDTH_16BIT;
    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.bclk = (gpio_num_t)APP_VOICE_I2S_BCLK_PIN;
    std_cfg.gpio_cfg.ws = (gpio_num_t)APP_VOICE_I2S_WS_PIN;
    std_cfg.gpio_cfg.dout = (gpio_num_t)APP_VOICE_I2S_DOUT_PIN;
    std_cfg.gpio_cfg.din = I2S_GPIO_UNUSED;
    if (i2s_channel_init_std_mode(g_tx_chan, &std_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "init tx std mode failed");
        return false;
    }
    if (i2s_channel_enable(g_tx_chan) != ESP_OK) {
        ESP_LOGE(TAG, "enable tx channel failed");
        return false;
    }
    return true;
#else
    i2s_config_t amp_cfg = {};
    amp_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    amp_cfg.sample_rate = APP_VOICE_SAMPLE_RATE_HZ;
    amp_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    amp_cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    amp_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    amp_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    amp_cfg.dma_buf_count = 8;
    amp_cfg.dma_buf_len = 512;
    amp_cfg.tx_desc_auto_clear = true;
    i2s_pin_config_t pin_cfg = {};
    pin_cfg.bck_io_num = APP_VOICE_I2S_BCLK_PIN;
    pin_cfg.ws_io_num = APP_VOICE_I2S_WS_PIN;
    pin_cfg.data_out_num = APP_VOICE_I2S_DOUT_PIN;
    pin_cfg.data_in_num = I2S_PIN_NO_CHANGE;

    if (i2s_driver_install(I2S_NUM_1, &amp_cfg, 0, NULL) != ESP_OK) {
        return false;
    }
    if (i2s_set_pin(I2S_NUM_1, &pin_cfg) != ESP_OK) {
        i2s_driver_uninstall(I2S_NUM_1);
        return false;
    }
    return true;
#endif
}

bool amp_stream_start(uint32_t sample_rate_hz) {
#if USE_I2S_STD
    (void)sample_rate_hz;
    if (g_tx_chan == NULL) {
        return false;
    }
#else
    i2s_zero_dma_buffer(I2S_NUM_1);
    if (i2s_set_clk(I2S_NUM_1, sample_rate_hz, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_ONLY_LEFT) != ESP_OK) {
        return false;
    }
#endif
    g_stream_active = true;
    return true;
}

bool amp_stream_write(const uint8_t *data, size_t len, uint32_t timeout_ms) {
    if (!g_stream_active || data == NULL || len == 0U) {
        return false;
    }

    const size_t pcm_len = len & ~((size_t)1);
    uint8_t gain_chunk[AMP_CHUNK_SIZE];
    size_t offset = 0U;
    while (offset < pcm_len) {
        size_t chunk_len = pcm_len - offset;
        if (chunk_len > AMP_CHUNK_SIZE) {
            chunk_len = AMP_CHUNK_SIZE;
        }
        chunk_len &= ~((size_t)1);

        for (size_t i = 0U; i < chunk_len; i += 2U) {
            const uint8_t *src = data + offset + i;
            int16_t sample = (int16_t)((src[1] << 8) | src[0]);
            const float amplified = sample * AMP_DEFAULT_GAIN;
            if (amplified > INT16_MAX) {
                sample = INT16_MAX;
            } else if (amplified < INT16_MIN) {
                sample = INT16_MIN;
            } else {
                sample = (int16_t)amplified;
            }
            gain_chunk[i] = (uint8_t)(sample & 0xFF);
            gain_chunk[i + 1U] = (uint8_t)((sample >> 8) & 0xFF);
        }

        size_t sent = 0U;
        const uint32_t start_ms = get_time_ms();
        while (sent < chunk_len) {
            const uint32_t elapsed_ms = get_time_ms() - start_ms;
            const uint32_t remain_ms = (timeout_ms > elapsed_ms) ? (timeout_ms - elapsed_ms) : 0U;
            TickType_t ticks = 0;
            if (remain_ms > 0U) {
                ticks = pdMS_TO_TICKS(remain_ms);
                if (ticks == 0) {
                    ticks = 1;
                }
            }

            size_t written = 0U;
            esp_err_t err = ESP_OK;
#if USE_I2S_STD
            err = i2s_channel_write(g_tx_chan, gain_chunk + sent, chunk_len - sent, &written, ticks);
#else
            err = i2s_write(I2S_NUM_1, gain_chunk + sent, chunk_len - sent, &written, ticks);
#endif
            if (written > 0U) {
                sent += written;
            }
            if (err == ESP_OK || (err == ESP_ERR_TIMEOUT && remain_ms > 0U)) {
                continue;
            }
            ESP_LOGE(TAG, "stream write failed err=%d", (int)err);
            return false;
        }

        offset += chunk_len;
    }
    return true;
}

void amp_stream_stop(void) {
    if (!g_stream_active) {
        return;
    }
#if USE_I2S_STD
    if (g_tx_chan != NULL) {
        uint8_t silence[AMP_CHUNK_SIZE] = {};
        size_t written = 0U;
        i2s_channel_write(g_tx_chan, silence, sizeof(silence), &written, portMAX_DELAY);
    }
#else
    i2s_zero_dma_buffer(I2S_NUM_1);
#endif
    g_stream_active = false;
}

bool amp_play_audio(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0U) {
        return false;
    }

    const uint8_t *pcm = data;
    size_t pcm_len = len;
    if (len >= 44U && data[0] == 'R' && data[1] == 'I' && data[2] == 'F' && data[3] == 'F') {
        pcm = data + 44U;
        pcm_len = len - 44U;
    }

    if (!amp_stream_start(APP_VOICE_SAMPLE_RATE_HZ)) {
        return false;
    }
    const bool ok = amp_stream_write(pcm, pcm_len, 5000U);
    amp_stream_stop();
    return ok;
}

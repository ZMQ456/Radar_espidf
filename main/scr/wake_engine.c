#include "wake_engine.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mic.h"

#if __has_include("esp_afe_sr_iface.h") && (__has_include("esp_afe_sr_models.h") || __has_include("model_path.h"))
#include "esp_afe_sr_iface.h"
#if __has_include("esp_afe_sr_models.h")
#include "esp_afe_sr_models.h"
#else
#include "model_path.h"
#endif
#include "esp_wn_iface.h"
#define WAKE_ENGINE_SR_AVAILABLE 1
#else
#define WAKE_ENGINE_SR_AVAILABLE 0
#endif

static const char *TAG = "wake_engine";

#define WAKE_SEM_WAIT_MS 30
#define WAKE_SHORT_WAIT_MS 10
#define WAKE_RESOURCE_RETRY_MS 500
#define WAKE_DEFAULT_TRIGGER_COOLDOWN_MS 3000
#define WAKE_DEFAULT_MIC_READ_TIMEOUT_MS 60
#define WAKE_DEFAULT_DIAG_LOG_INTERVAL_MS 2000
#define WAKE_DEFAULT_TASK_YIELD_MS 2
#define WAKE_DEFAULT_LOOP_YIELD_EVERY_N 8

static wake_engine_config_t s_cfg = {0};
static bool s_cfg_ready = false;
static uint32_t s_last_trigger_ms = 0;
static uint32_t s_last_diag_log_ms = 0;

#if WAKE_ENGINE_SR_AVAILABLE
static const esp_afe_sr_iface_t *s_afe_iface = NULL;
static esp_afe_sr_data_t *s_afe_data = NULL;
static afe_config_t *s_afe_cfg = NULL;
static srmodel_list_t *s_sr_models = NULL;
static int16_t *s_wake_feed_buf = NULL;
static size_t s_wake_feed_samples = 0U;

static void apply_default_config(wake_engine_config_t *cfg) {
    if (cfg->trigger_cooldown_ms == 0U) {
        cfg->trigger_cooldown_ms = WAKE_DEFAULT_TRIGGER_COOLDOWN_MS;
    }
    if (cfg->mic_read_timeout_ms == 0U) {
        cfg->mic_read_timeout_ms = WAKE_DEFAULT_MIC_READ_TIMEOUT_MS;
    }
    if (cfg->diag_log_interval_ms == 0U) {
        cfg->diag_log_interval_ms = WAKE_DEFAULT_DIAG_LOG_INTERVAL_MS;
    }
    if (cfg->task_yield_ms == 0U) {
        cfg->task_yield_ms = WAKE_DEFAULT_TASK_YIELD_MS;
    }
    if (cfg->loop_yield_every_n == 0U) {
        cfg->loop_yield_every_n = WAKE_DEFAULT_LOOP_YIELD_EVERY_N;
    }
}

static bool init_sr_resources(void) {
    s_sr_models = esp_srmodel_init("model");
    if (s_sr_models == NULL) {
        ESP_LOGW(TAG, "ESP-SR model partition not ready");
        return false;
    }

    s_afe_cfg = afe_config_init("M", s_sr_models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    if (s_afe_cfg == NULL) {
        return false;
    }

    s_afe_iface = esp_afe_handle_from_config(s_afe_cfg);
    if (s_afe_iface == NULL) {
        return false;
    }

    s_afe_data = s_afe_iface->create_from_config(s_afe_cfg);
    if (s_afe_data == NULL) {
        return false;
    }

    const int feed_chunk = s_afe_iface->get_feed_chunksize(s_afe_data);
    const int feed_ch = s_afe_iface->get_feed_channel_num(s_afe_data);
    if (feed_chunk <= 0 || feed_ch <= 0) {
        return false;
    }

    s_wake_feed_samples = (size_t)feed_chunk * (size_t)feed_ch;
    s_wake_feed_buf = (int16_t *)heap_caps_malloc(
        s_wake_feed_samples * sizeof(int16_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return s_wake_feed_buf != NULL;
}

bool wake_engine_init(const wake_engine_config_t *cfg) {
    if (cfg == NULL || cfg->audio_sem == NULL || cfg->trigger_fn == NULL) {
        ESP_LOGE(TAG, "invalid wake config");
        return false;
    }

    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg = *cfg;
    apply_default_config(&s_cfg);
    s_last_trigger_ms = 0U;
    s_last_diag_log_ms = 0U;

    if (!init_sr_resources()) {
        s_cfg_ready = false;
        return false;
    }

    s_cfg_ready = true;
    return true;
}

void wake_engine_task(void *arg) {
    (void)arg;

    const TickType_t wake_yield_ticks = pdMS_TO_TICKS(s_cfg.task_yield_ms) > 0 ? pdMS_TO_TICKS(s_cfg.task_yield_ms) : 1;
    const TickType_t short_wait_ticks = pdMS_TO_TICKS(WAKE_SHORT_WAIT_MS) > 0 ? pdMS_TO_TICKS(WAKE_SHORT_WAIT_MS) : 1;
    const TickType_t sem_wait_ticks = pdMS_TO_TICKS(WAKE_SEM_WAIT_MS) > 0 ? pdMS_TO_TICKS(WAKE_SEM_WAIT_MS) : 1;
    const TickType_t resource_wait_ticks = pdMS_TO_TICKS(WAKE_RESOURCE_RETRY_MS) > 0 ? pdMS_TO_TICKS(WAKE_RESOURCE_RETRY_MS) : 1;
    uint32_t loop_count = 0U;
    bool sem_blocked_prev = false;

    while (true) {
        if (!s_cfg_ready || s_afe_iface == NULL || s_afe_data == NULL || s_wake_feed_buf == NULL || s_wake_feed_samples == 0U) {
            vTaskDelay(resource_wait_ticks);
            continue;
        }

        if (xSemaphoreTake(s_cfg.audio_sem, sem_wait_ticks) != pdTRUE) {
            sem_blocked_prev = true;
            vTaskDelay(short_wait_ticks);
            continue;
        }
        if (sem_blocked_prev && s_afe_iface->reset_buffer != NULL) {
            s_afe_iface->reset_buffer(s_afe_data);
            sem_blocked_prev = false;
        }

        size_t got_samples = 0U;
        const bool got_audio = mic_read_pcm16(s_wake_feed_buf, s_wake_feed_samples, &got_samples, s_cfg.mic_read_timeout_ms);
        xSemaphoreGive(s_cfg.audio_sem);
        if (!got_audio || got_samples == 0U) {
            vTaskDelay(short_wait_ticks);
            continue;
        }

        if (got_samples < s_wake_feed_samples) {
            memset(s_wake_feed_buf + got_samples, 0, (s_wake_feed_samples - got_samples) * sizeof(int16_t));
        }

        s_afe_iface->feed(s_afe_data, s_wake_feed_buf);
        afe_fetch_result_t *result = s_afe_iface->fetch(s_afe_data);
        if (result == NULL) {
            vTaskDelay(short_wait_ticks);
            continue;
        }

        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        if (now_ms - s_last_diag_log_ms >= s_cfg.diag_log_interval_ms) {
            s_last_diag_log_ms = now_ms;
        }

        if (result->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, "wake detected word=%d model=%d", result->wake_word_index, result->wakenet_model_index);
            if (now_ms - s_last_trigger_ms >= s_cfg.trigger_cooldown_ms) {
                s_last_trigger_ms = now_ms;
                (void)s_cfg.trigger_fn(s_cfg.trigger_ctx);
            }
        }

        loop_count++;
        if ((loop_count % s_cfg.loop_yield_every_n) == 0U) {
            vTaskDelay(wake_yield_ticks);
        }
    }
}

#else

bool wake_engine_init(const wake_engine_config_t *cfg) {
    (void)cfg;
    ESP_LOGW(TAG, "ESP-SR headers not found, wake disabled");
    return false;
}

void wake_engine_task(void *arg) {
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif

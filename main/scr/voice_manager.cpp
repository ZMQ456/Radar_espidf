#include "voice_manager.h"

#include <cstring>

#include "amp.h"
#include "app_config.h"
#include "audio_pipeline.h"
#include "device_identity.h"
#include "esp_log.h"
#include "mem_utils.h"
#include "mic.h"
#include "radar_platform.h"
#include "wake_engine.h"
#include "wifi_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "voice_manager";

static constexpr EventBits_t EVT_PIPELINE_BUSY = BIT0;
static constexpr EventBits_t EVT_RECORDING = BIT1;
static constexpr EventBits_t EVT_NET_PROCESSING = BIT2;
static constexpr EventBits_t EVT_STREAM_ACTIVE = BIT3;
static constexpr EventBits_t EVT_STREAM_EOF = BIT4;
static constexpr EventBits_t EVT_STREAM_PLAY_DONE = BIT5;
static constexpr EventBits_t EVT_ABORT_REQUEST = BIT6;

static constexpr uint32_t kCloudRespMaxBytes = 128U * 1024U;
static constexpr uint32_t kCloudTempBytes = 2048U;
static constexpr uint32_t kAudioB64PoolSize = 2U;
static constexpr uint32_t kPcmRingbufBytes = 32U * 1024U;
static constexpr uint32_t kStreamPlayWaitTimeoutMs = 30000U;
static constexpr uint32_t kStreamRingbufRecvTimeoutMs = 20U;
static constexpr uint32_t kStreamUnderrunSilenceMs = 20U;
static constexpr uint32_t kStreamPcmWriteTimeoutMs = 1200U;
static constexpr uint32_t kStreamSilenceWriteTimeoutMs = 400U;
static constexpr uint32_t kWakeUiTailGuardMs = 120U;
static constexpr uint32_t kWakeMicReadTimeoutMs = 60U;
static constexpr uint32_t kWakeDiagLogIntervalMs = 2000U;
static constexpr uint32_t kWakeTaskYieldMs = 2U;
static constexpr uint32_t kWakeLoopYieldEveryN = 8U;
static constexpr uint32_t kWakeTaskPriority = 3U;
static constexpr BaseType_t kWakeTaskCoreId = 1;

typedef enum {
    VOICE_UI_EVT_RECORD = 1,
} voice_ui_evt_id_t;

typedef struct {
    voice_ui_evt_id_t id;
} voice_ui_evt_t;

typedef struct {
    char *audio_b64;
} cloud_audio_req_t;

static QueueHandle_t gRecordTriggerQueue = nullptr;
static QueueHandle_t gCloudReqQueue = nullptr;
static QueueHandle_t gAudioB64FreeQueue = nullptr;
static QueueHandle_t gVoiceUiQueue = nullptr;
static RingbufHandle_t gPcmRingbuf = nullptr;
static SemaphoreHandle_t gAudioSem = nullptr;
static SemaphoreHandle_t gMicSem = nullptr;
static EventGroupHandle_t gStateEvents = nullptr;
static char *gAudioB64Pool[kAudioB64PoolSize] = {};
static size_t gAudioB64BufSize = 0U;
static audio_pipeline_cloud_workbuf_t gCloudWorkbuf = {};
static bool gVoiceInitialized = false;
static bool gWakeReady = false;

static uint32_t stream_underrun_silence_bytes(void) {
    return (APP_VOICE_SAMPLE_RATE_HZ * 2U * kStreamUnderrunSilenceMs) / 1000U;
}

static bool voice_pipeline_busy(void) {
    return gStateEvents != nullptr && (xEventGroupGetBits(gStateEvents) & EVT_PIPELINE_BUSY) != 0U;
}

static bool voice_trigger_from_wake(void *ctx) {
    (void)ctx;
    return voice_manager_trigger_session();
}

static void stream_playback_task(void *arg) {
    (void)arg;
    const uint32_t silence_len = stream_underrun_silence_bytes();
    uint8_t silence_chunk[640] = {};

    while (true) {
        xEventGroupWaitBits(gStateEvents, EVT_STREAM_ACTIVE, pdFALSE, pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(gAudioSem, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGE(TAG, "playback task audio sem timeout");
            xEventGroupSetBits(gStateEvents, EVT_STREAM_PLAY_DONE);
            continue;
        }
        if (!amp_stream_start(APP_VOICE_SAMPLE_RATE_HZ)) {
            ESP_LOGE(TAG, "amp stream start failed");
            xSemaphoreGive(gAudioSem);
            xEventGroupSetBits(gStateEvents, EVT_STREAM_PLAY_DONE);
            continue;
        }

        while (true) {
            size_t item_size = 0U;
            uint8_t *item = (uint8_t *)xRingbufferReceive(gPcmRingbuf, &item_size, pdMS_TO_TICKS(kStreamRingbufRecvTimeoutMs));
            if (item != nullptr) {
                const bool ok = amp_stream_write(item, item_size, kStreamPcmWriteTimeoutMs);
                vRingbufferReturnItem(gPcmRingbuf, item);
                if (!ok) {
                    ESP_LOGE(TAG, "pcm stream write failed");
                    break;
                }
                continue;
            }

            const EventBits_t bits = xEventGroupGetBits(gStateEvents);
            if ((bits & EVT_STREAM_EOF) != 0U || (bits & EVT_ABORT_REQUEST) != 0U) {
                break;
            }
            if (!amp_stream_write(silence_chunk, silence_len < sizeof(silence_chunk) ? silence_len : sizeof(silence_chunk), kStreamSilenceWriteTimeoutMs)) {
                break;
            }
        }

        amp_stream_stop();
        xSemaphoreGive(gAudioSem);
        xEventGroupClearBits(gStateEvents, EVT_STREAM_ACTIVE | EVT_STREAM_EOF | EVT_ABORT_REQUEST);
        xEventGroupSetBits(gStateEvents, EVT_STREAM_PLAY_DONE);
    }
}

static void voice_ui_task(void *arg) {
    (void)arg;
    voice_ui_evt_t evt = {};
    while (true) {
        if (xQueueReceive(gVoiceUiQueue, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (evt.id != VOICE_UI_EVT_RECORD) {
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(kWakeUiTailGuardMs));
        uint8_t trigger = 1U;
        if (xQueueSend(gRecordTriggerQueue, &trigger, pdMS_TO_TICKS(200)) != pdTRUE) {
            ESP_LOGE(TAG, "record trigger enqueue failed");
            xEventGroupClearBits(gStateEvents, EVT_PIPELINE_BUSY);
        }
    }
}

static void record_task(void *arg) {
    (void)arg;
    uint8_t trigger = 0U;
    while (true) {
        if (xQueueReceive(gRecordTriggerQueue, &trigger, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        xEventGroupSetBits(gStateEvents, EVT_PIPELINE_BUSY | EVT_RECORDING);
        if (xSemaphoreTake(gMicSem, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGE(TAG, "mic sem timeout");
            xEventGroupClearBits(gStateEvents, EVT_RECORDING | EVT_PIPELINE_BUSY);
            continue;
        }

        uint8_t *wav = nullptr;
        size_t wav_len = 0U;
        if (!mic_record_wav(&wav, &wav_len)) {
            ESP_LOGW(TAG, "voice record failed or no speech");
            xSemaphoreGive(gMicSem);
            xEventGroupClearBits(gStateEvents, EVT_RECORDING | EVT_PIPELINE_BUSY);
            continue;
        }
        xSemaphoreGive(gMicSem);
        xEventGroupClearBits(gStateEvents, EVT_RECORDING);

        char *audio_b64 = nullptr;
        if (!audio_pipeline_prepare_upload_b64(wav, wav_len, gAudioB64FreeQueue, gAudioB64BufSize, &audio_b64)) {
            ESP_LOGE(TAG, "prepare upload b64 failed");
            xEventGroupClearBits(gStateEvents, EVT_PIPELINE_BUSY);
            continue;
        }

        cloud_audio_req_t req = {.audio_b64 = audio_b64};
        if (xQueueSend(gCloudReqQueue, &req, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGE(TAG, "cloud queue send failed");
            audio_pipeline_recycle_audio_b64_buffer(gAudioB64FreeQueue, audio_b64);
            xEventGroupClearBits(gStateEvents, EVT_PIPELINE_BUSY);
        }
    }
}

static void cloud_task(void *arg) {
    (void)arg;
    audio_pipeline_stream_sync_t stream_sync = {
        .state_events = gStateEvents,
        .stream_active_bit = EVT_STREAM_ACTIVE,
        .stream_eof_bit = EVT_STREAM_EOF,
        .stream_play_done_bit = EVT_STREAM_PLAY_DONE,
        .abort_request_bit = EVT_ABORT_REQUEST,
        .stream_play_done_timeout_ms = kStreamPlayWaitTimeoutMs,
        .pcm_ringbuf = gPcmRingbuf,
    };

    while (true) {
        cloud_audio_req_t req = {};
        if (xQueueReceive(gCloudReqQueue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        while (!wifi_manager_is_connected()) {
            ESP_LOGW(TAG, "voice cloud wait wifi");
            radar_sleep_ms(500);
        }

        const DeviceIdentity identity = device_identity_get();
        audio_pipeline_server_config_t server_cfg = {
            .host = APP_VOICE_SERVER_HOST,
            .port = APP_VOICE_SERVER_PORT,
            .device_id = identity.device_id != 0U ? identity.device_id : APP_VOICE_DEVICE_ID_DEFAULT,
        };

        xEventGroupSetBits(gStateEvents, EVT_NET_PROCESSING);
        const bool ok = audio_pipeline_send_audio_request(
            &server_cfg,
            &stream_sync,
            req.audio_b64,
            nullptr,
            nullptr,
            nullptr,
            &gCloudWorkbuf,
            false);
        audio_pipeline_recycle_audio_b64_buffer(gAudioB64FreeQueue, req.audio_b64);
        xEventGroupClearBits(gStateEvents, EVT_NET_PROCESSING | EVT_PIPELINE_BUSY);
        ESP_LOGI(TAG, "voice cloud request %s", ok ? "ok" : "failed");
    }
}

static void wake_word_task(void *arg) {
    wake_engine_task(arg);
}

bool voice_manager_init(void) {
    if (gVoiceInitialized) {
        return true;
    }
    if (!APP_VOICE_ENABLED) {
        ESP_LOGI(TAG, "voice disabled by config");
        return false;
    }

    gRecordTriggerQueue = xQueueCreate(4, sizeof(uint8_t));
    gCloudReqQueue = xQueueCreate(2, sizeof(cloud_audio_req_t));
    gAudioB64FreeQueue = xQueueCreate(kAudioB64PoolSize, sizeof(char *));
    gVoiceUiQueue = xQueueCreate(4, sizeof(voice_ui_evt_t));
    gPcmRingbuf = xRingbufferCreate(kPcmRingbufBytes, RINGBUF_TYPE_BYTEBUF);
    gAudioSem = xSemaphoreCreateMutex();
    gMicSem = xSemaphoreCreateMutex();
    gStateEvents = xEventGroupCreate();
    if (gRecordTriggerQueue == nullptr || gCloudReqQueue == nullptr || gAudioB64FreeQueue == nullptr ||
        gVoiceUiQueue == nullptr || gPcmRingbuf == nullptr || gAudioSem == nullptr || gMicSem == nullptr ||
        gStateEvents == nullptr) {
        ESP_LOGE(TAG, "voice RTOS object create failed");
        return false;
    }

    if (!audio_pipeline_init_audio_b64_pool(gAudioB64Pool, kAudioB64PoolSize, gAudioB64FreeQueue, &gAudioB64BufSize) ||
        !audio_pipeline_init_cloud_workbuf(&gCloudWorkbuf, kCloudTempBytes, kCloudRespMaxBytes)) {
        ESP_LOGE(TAG, "voice workspace init failed");
        audio_pipeline_deinit_cloud_workbuf(&gCloudWorkbuf);
        audio_pipeline_deinit_audio_b64_pool(gAudioB64Pool, kAudioB64PoolSize, &gAudioB64BufSize);
        return false;
    }

    if (!mic_init()) {
        ESP_LOGE(TAG, "voice mic init failed");
        return false;
    }
    if (!amp_init()) {
        ESP_LOGE(TAG, "voice amp init failed");
        return false;
    }

    wake_engine_config_t wake_cfg = {
        .audio_sem = gMicSem,
        .trigger_fn = voice_trigger_from_wake,
        .trigger_ctx = nullptr,
        .trigger_cooldown_ms = APP_VOICE_WAKE_TRIGGER_COOLDOWN_MS,
        .mic_read_timeout_ms = kWakeMicReadTimeoutMs,
        .diag_log_interval_ms = kWakeDiagLogIntervalMs,
        .task_yield_ms = kWakeTaskYieldMs,
        .loop_yield_every_n = kWakeLoopYieldEveryN,
    };
    gWakeReady = wake_engine_init(&wake_cfg);
    if (!gWakeReady) {
        ESP_LOGW(TAG, "wake word unavailable, use `voice_once` command to trigger manually");
    }

    xTaskCreate(stream_playback_task, "voice_stream_play", 6144, nullptr, 6, nullptr);
    xTaskCreate(voice_ui_task, "voice_ui_task", 4096, nullptr, 5, nullptr);
    xTaskCreate(record_task, "voice_record_task", 8192, nullptr, 5, nullptr);
    xTaskCreate(cloud_task, "voice_cloud_task", 12288, nullptr, 5, nullptr);
    if (gWakeReady) {
        xTaskCreatePinnedToCore(wake_word_task, "voice_wake_task", 8192, nullptr, kWakeTaskPriority, nullptr, kWakeTaskCoreId);
    }

    gVoiceInitialized = true;
    ESP_LOGI(TAG, "voice manager initialized");
    return true;
}

bool voice_manager_is_initialized(void) {
    return gVoiceInitialized;
}

bool voice_manager_trigger_session(void) {
    if (!gVoiceInitialized) {
        return false;
    }
    if (voice_pipeline_busy()) {
        ESP_LOGW(TAG, "voice pipeline busy, trigger ignored");
        return false;
    }

    const voice_ui_evt_t evt = {.id = VOICE_UI_EVT_RECORD};
    if (xQueueSend(gVoiceUiQueue, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "voice ui queue full");
        return false;
    }
    xEventGroupSetBits(gStateEvents, EVT_PIPELINE_BUSY);
    ESP_LOGI(TAG, "voice session triggered");
    return true;
}

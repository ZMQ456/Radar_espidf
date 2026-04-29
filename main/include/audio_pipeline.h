#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char *temp;
    size_t temp_cap;
    char *resp;
    size_t resp_cap;
} audio_pipeline_cloud_workbuf_t;

typedef struct {
    const char *host;
    uint16_t port;
    uint32_t device_id;
} audio_pipeline_server_config_t;

typedef struct {
    EventGroupHandle_t state_events;
    EventBits_t stream_active_bit;
    EventBits_t stream_eof_bit;
    EventBits_t stream_play_done_bit;
    EventBits_t abort_request_bit;
    uint32_t stream_play_done_timeout_ms;
    RingbufHandle_t pcm_ringbuf;
} audio_pipeline_stream_sync_t;

typedef struct {
    bool include;
    bool on;
    uint8_t brightness_percent;
} audio_pipeline_light_state_t;

typedef bool (*audio_pipeline_response_hook_fn)(const char *resp_snapshot, void *ctx);

bool audio_pipeline_init_cloud_workbuf(audio_pipeline_cloud_workbuf_t *wb, size_t temp_bytes, size_t resp_bytes);
void audio_pipeline_deinit_cloud_workbuf(audio_pipeline_cloud_workbuf_t *wb);

bool audio_pipeline_init_audio_b64_pool(char **pool, int pool_size, QueueHandle_t free_queue, size_t *out_buf_size);
void audio_pipeline_deinit_audio_b64_pool(char **pool, int pool_size, size_t *buf_size);
void audio_pipeline_recycle_audio_b64_buffer(QueueHandle_t free_queue, char *buf);

void audio_pipeline_preprocess_wav_for_asr(uint8_t *wav, size_t wav_len);
bool audio_pipeline_prepare_upload_b64(
    uint8_t *wav,
    size_t wav_len,
    QueueHandle_t audio_b64_free_queue,
    size_t audio_b64_buf_size,
    char **out_audio_b64);

bool audio_pipeline_send_audio_request(
    const audio_pipeline_server_config_t *server_cfg,
    const audio_pipeline_stream_sync_t *stream_sync,
    const char *audio_b64,
    const audio_pipeline_light_state_t *light_state,
    audio_pipeline_response_hook_fn response_hook,
    void *response_hook_ctx,
    audio_pipeline_cloud_workbuf_t *wb,
    bool suppress_retry_prompt_audio);

#ifdef __cplusplus
}
#endif

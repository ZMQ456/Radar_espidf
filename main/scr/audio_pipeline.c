#include "audio_pipeline.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "amp.h"
#include "mem_utils.h"
#include "mic.h"
#include "tcp.h"
#include "utils.h"

#define B64_DECODE_BATCH 4096
#define CLOUD_RECV_SLICE_TIMEOUT_MS 300
#define CLOUD_RECV_MAX_IDLE_MS 15000

typedef struct {
    bool wav_checked;
    bool wav_mode;
    uint8_t wav_hdr[44];
    size_t wav_hdr_len;
    bool has_tail;
    uint8_t tail_byte;
} stream_pcm_ctx_t;

static const char *TAG = "audio_pipeline";

static inline bool is_b64_char(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '+' || c == '/' ||
           c == '=';
}

static bool ringbuf_send_pcm(RingbufHandle_t pcm_ringbuf, const uint8_t *data, size_t len) {
    if (data == NULL || len == 0U) {
        return true;
    }
    return pcm_ringbuf != NULL && xRingbufferSend(pcm_ringbuf, data, len, pdMS_TO_TICKS(3000)) == pdTRUE;
}

static bool stream_send_pcm_chunk(
    RingbufHandle_t pcm_ringbuf,
    stream_pcm_ctx_t *ctx,
    const uint8_t *data,
    size_t len,
    bool final_flush) {
    if (ctx == NULL || data == NULL || len == 0U) {
        return true;
    }

    const uint8_t *p = data;
    size_t n = len;
    if (!ctx->wav_checked) {
        const size_t need = 44U - ctx->wav_hdr_len;
        const size_t take = (n < need) ? n : need;
        memcpy(ctx->wav_hdr + ctx->wav_hdr_len, p, take);
        ctx->wav_hdr_len += take;
        p += take;
        n -= take;

        if (ctx->wav_hdr_len == 44U) {
            ctx->wav_checked = true;
            if (ctx->wav_hdr[0] == 'R' && ctx->wav_hdr[1] == 'I' && ctx->wav_hdr[2] == 'F' && ctx->wav_hdr[3] == 'F' &&
                ctx->wav_hdr[8] == 'W' && ctx->wav_hdr[9] == 'A' && ctx->wav_hdr[10] == 'V' && ctx->wav_hdr[11] == 'E') {
                ctx->wav_mode = true;
            } else {
                ctx->wav_mode = false;
                if (!ringbuf_send_pcm(pcm_ringbuf, ctx->wav_hdr, ctx->wav_hdr_len)) {
                    return false;
                }
            }
        } else if (!final_flush) {
            return true;
        }
    }

    if (ctx->has_tail && n > 0U) {
        uint8_t pair[2] = {ctx->tail_byte, p[0]};
        if (!ringbuf_send_pcm(pcm_ringbuf, pair, sizeof(pair))) {
            return false;
        }
        ctx->has_tail = false;
        ++p;
        --n;
    }

    const size_t even_n = n & ~((size_t)1);
    if (even_n > 0U && !ringbuf_send_pcm(pcm_ringbuf, p, even_n)) {
        return false;
    }
    p += even_n;
    n -= even_n;

    if (n == 1U) {
        ctx->has_tail = true;
        ctx->tail_byte = p[0];
    }
    if (final_flush) {
        ctx->has_tail = false;
    }
    return true;
}

static bool decode_and_send_b64_batch(
    RingbufHandle_t pcm_ringbuf,
    char *b64_batch,
    size_t *batch_len,
    bool final_flush,
    stream_pcm_ctx_t *pcm_ctx) {
    if (b64_batch == NULL || batch_len == NULL || pcm_ctx == NULL) {
        return false;
    }

    size_t n = *batch_len;
    if (n == 0U) {
        return true;
    }

    size_t proc = final_flush ? n : ((n / 4U) * 4U);
    if (proc == 0U) {
        return true;
    }
    if (final_flush) {
        while ((proc % 4U) != 0U && proc + 1U < B64_DECODE_BATCH) {
            b64_batch[proc++] = '=';
        }
    }

    uint8_t pcm_out[(B64_DECODE_BATCH * 3U / 4U) + 16U];
    size_t pcm_len = 0U;
    if (!base64_decode((const uint8_t *)b64_batch, proc, pcm_out, sizeof(pcm_out), &pcm_len)) {
        return false;
    }
    if (!stream_send_pcm_chunk(pcm_ringbuf, pcm_ctx, pcm_out, pcm_len, final_flush)) {
        return false;
    }

    const size_t remain = n - proc;
    if (remain > 0U) {
        memmove(b64_batch, b64_batch + proc, remain);
    }
    *batch_len = remain;
    return true;
}

static void ringbuf_drain_all(RingbufHandle_t pcm_ringbuf) {
    if (pcm_ringbuf == NULL) {
        return;
    }
    while (true) {
        size_t item_size = 0U;
        void *item = xRingbufferReceive(pcm_ringbuf, &item_size, 0);
        if (item == NULL) {
            break;
        }
        vRingbufferReturnItem(pcm_ringbuf, item);
    }
}

static bool stream_abort_requested(const audio_pipeline_stream_sync_t *stream_sync) {
    if (stream_sync == NULL || stream_sync->state_events == NULL || stream_sync->abort_request_bit == 0U) {
        return false;
    }
    return (xEventGroupGetBits(stream_sync->state_events) & stream_sync->abort_request_bit) != 0U;
}

static void signal_stream_eof(const audio_pipeline_stream_sync_t *stream_sync) {
    if (stream_sync != NULL && stream_sync->state_events != NULL) {
        xEventGroupSetBits(stream_sync->state_events, stream_sync->stream_eof_bit);
    }
}

bool audio_pipeline_init_cloud_workbuf(audio_pipeline_cloud_workbuf_t *wb, size_t temp_bytes, size_t resp_bytes) {
    if (wb == NULL || temp_bytes == 0U || resp_bytes == 0U) {
        return false;
    }

    wb->temp = (char *)heap_caps_malloc(temp_bytes, MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
    wb->temp_cap = temp_bytes;
    wb->resp = (char *)heap_caps_malloc(resp_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (wb->resp == NULL) {
        wb->resp = (char *)heap_caps_malloc(resp_bytes, MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
    }
    wb->resp_cap = resp_bytes;
    if (wb->temp == NULL || wb->resp == NULL) {
        audio_pipeline_deinit_cloud_workbuf(wb);
        return false;
    }
    wb->resp[0] = '\0';
    return true;
}

void audio_pipeline_deinit_cloud_workbuf(audio_pipeline_cloud_workbuf_t *wb) {
    if (wb == NULL) {
        return;
    }
    if (wb->temp != NULL) {
        heap_caps_free(wb->temp);
        wb->temp = NULL;
    }
    if (wb->resp != NULL) {
        heap_caps_free(wb->resp);
        wb->resp = NULL;
    }
    wb->temp_cap = 0U;
    wb->resp_cap = 0U;
}

bool audio_pipeline_init_audio_b64_pool(char **pool, int pool_size, QueueHandle_t free_queue, size_t *out_buf_size) {
    if (pool == NULL || pool_size <= 0 || free_queue == NULL || out_buf_size == NULL) {
        return false;
    }

    const mic_config_t mic_cfg = mic_get_default_config();
    const uint32_t max_pcm_16bit =
        (uint32_t)(mic_cfg.sample_rate * mic_cfg.max_record_time_ms / 1000U) * mic_cfg.channels * 2U;
    const size_t max_wav_len = 44U + (size_t)max_pcm_16bit;
    *out_buf_size = ((max_wav_len + 2U) / 3U) * 4U + 1U;

    for (int i = 0; i < pool_size; ++i) {
        pool[i] = (char *)app_mem_alloc(*out_buf_size, MEM_TYPE_LARGE);
        if (pool[i] == NULL || xQueueSend(free_queue, &pool[i], 0) != pdTRUE) {
            audio_pipeline_deinit_audio_b64_pool(pool, pool_size, out_buf_size);
            return false;
        }
    }
    return true;
}

void audio_pipeline_deinit_audio_b64_pool(char **pool, int pool_size, size_t *buf_size) {
    if (pool == NULL || pool_size <= 0) {
        return;
    }
    for (int i = 0; i < pool_size; ++i) {
        if (pool[i] != NULL) {
            app_mem_free(pool[i]);
            pool[i] = NULL;
        }
    }
    if (buf_size != NULL) {
        *buf_size = 0U;
    }
}

void audio_pipeline_recycle_audio_b64_buffer(QueueHandle_t free_queue, char *buf) {
    if (free_queue != NULL && buf != NULL) {
        (void)xQueueSend(free_queue, &buf, pdMS_TO_TICKS(1000));
    }
}

void audio_pipeline_preprocess_wav_for_asr(uint8_t *wav, size_t wav_len) {
    if (wav == NULL || wav_len <= 44U) {
        return;
    }

    int16_t *pcm = (int16_t *)(wav + 44U);
    const size_t samples = (wav_len - 44U) / sizeof(int16_t);
    if (samples == 0U) {
        return;
    }

    int64_t sum = 0;
    for (size_t i = 0U; i < samples; ++i) {
        sum += pcm[i];
    }
    const int32_t dc = (int32_t)(sum / (int64_t)samples);

    int32_t peak = 0;
    for (size_t i = 0U; i < samples; ++i) {
        int32_t v = (int32_t)pcm[i] - dc;
        if (v > 32767) {
            v = 32767;
        } else if (v < -32768) {
            v = -32768;
        }
        pcm[i] = (int16_t)v;
        const int32_t abs_v = (v >= 0) ? v : -v;
        if (abs_v > peak) {
            peak = abs_v;
        }
    }

    const int32_t target_peak = 12000;
    if (peak > 0 && peak < target_peak) {
        int32_t gain_q12 = (target_peak << 12) / peak;
        if (gain_q12 > (3 << 12)) {
            gain_q12 = (3 << 12);
        }
        for (size_t i = 0U; i < samples; ++i) {
            int32_t v = ((int32_t)pcm[i] * gain_q12) >> 12;
            if (v > 32767) {
                v = 32767;
            } else if (v < -32768) {
                v = -32768;
            }
            pcm[i] = (int16_t)v;
        }
    }
}

bool audio_pipeline_prepare_upload_b64(
    uint8_t *wav,
    size_t wav_len,
    QueueHandle_t audio_b64_free_queue,
    size_t audio_b64_buf_size,
    char **out_audio_b64) {
    if (out_audio_b64 != NULL) {
        *out_audio_b64 = NULL;
    }
    if (wav == NULL || audio_b64_free_queue == NULL || out_audio_b64 == NULL) {
        if (wav != NULL) {
            app_mem_free(wav);
        }
        return false;
    }

    audio_pipeline_preprocess_wav_for_asr(wav, wav_len);

    char *audio_b64 = NULL;
    if (xQueueReceive(audio_b64_free_queue, &audio_b64, pdMS_TO_TICKS(1000)) != pdTRUE || audio_b64 == NULL) {
        app_mem_free(wav);
        return false;
    }

    const size_t b64_len = ((wav_len + 2U) / 3U) * 4U + 1U;
    if (b64_len > audio_b64_buf_size) {
        app_mem_free(wav);
        audio_pipeline_recycle_audio_b64_buffer(audio_b64_free_queue, audio_b64);
        return false;
    }

    size_t actual_b64_len = 0U;
    const bool enc_ok = base64_encode(wav, wav_len, (uint8_t *)audio_b64, b64_len, &actual_b64_len);
    app_mem_free(wav);
    if (!enc_ok || actual_b64_len == 0U) {
        audio_pipeline_recycle_audio_b64_buffer(audio_b64_free_queue, audio_b64);
        return false;
    }

    audio_b64[actual_b64_len] = '\0';
    *out_audio_b64 = audio_b64;
    return true;
}

bool audio_pipeline_send_audio_request(
    const audio_pipeline_server_config_t *server_cfg,
    const audio_pipeline_stream_sync_t *stream_sync,
    const char *audio_b64,
    const audio_pipeline_light_state_t *light_state,
    audio_pipeline_response_hook_fn response_hook,
    void *response_hook_ctx,
    audio_pipeline_cloud_workbuf_t *wb,
    bool suppress_retry_prompt_audio) {
    (void)light_state;
    (void)suppress_retry_prompt_audio;

    if (server_cfg == NULL || stream_sync == NULL || audio_b64 == NULL || wb == NULL || wb->temp == NULL ||
        wb->temp_cap == 0U || stream_sync->pcm_ringbuf == NULL || stream_sync->state_events == NULL) {
        ESP_LOGE(TAG, "invalid cloud pipeline args");
        return false;
    }
    if (stream_abort_requested(stream_sync)) {
        return false;
    }

    tcp_client_config_t tcp_cfg = {
        .host = server_cfg->host,
        .port = server_cfg->port,
        .timeout_ms = 5000,
        .recv_timeout_ms = 5000,
    };
    tcp_client_handle_t client = tcp_client_create(&tcp_cfg);
    if (client == NULL || !tcp_client_connect(client)) {
        tcp_client_destroy(client);
        return false;
    }

    const size_t b64_len = strlen(audio_b64);
    const size_t msg_len = 160U + b64_len;
    char *msg = (char *)app_mem_alloc(msg_len, msg_len > 4096U ? MEM_TYPE_LARGE : MEM_TYPE_SMALL);
    if (msg == NULL) {
        tcp_client_destroy(client);
        return false;
    }

    const int written = snprintf(
        msg,
        msg_len,
        "{\"type\":\"audio\",\"deviceId\":%lu,\"device_id\":%lu,\"audio\":\"%s\"}\n",
        (unsigned long)server_cfg->device_id,
        (unsigned long)server_cfg->device_id,
        audio_b64);
    if (written <= 0 || (size_t)written >= msg_len) {
        app_mem_free(msg);
        tcp_client_destroy(client);
        return false;
    }

    const int sent = tcp_client_send(client, msg, (size_t)written, 10000);
    app_mem_free(msg);
    if (sent != written) {
        tcp_client_destroy(client);
        return false;
    }

    ringbuf_drain_all(stream_sync->pcm_ringbuf);
    xEventGroupClearBits(stream_sync->state_events, stream_sync->stream_eof_bit | stream_sync->stream_play_done_bit);
    xEventGroupSetBits(stream_sync->state_events, stream_sync->stream_active_bit);

    enum { FIND_KEY = 0, FIND_COLON, FIND_VALUE_QUOTE, CAPTURE_VALUE } stream_state = FIND_KEY;
    const char *audio_key = "\"audio\"";
    size_t key_match = 0U;
    bool extracted_stream = false;
    bool b64_overflow = false;
    uint32_t idle_wait_ms = 0U;
    char b64_batch[B64_DECODE_BATCH] = {};
    size_t b64_batch_len = 0U;
    stream_pcm_ctx_t pcm_ctx = {0};
    size_t total_snapshot = 0U;
    wb->resp[0] = '\0';

    while (true) {
        if (stream_abort_requested(stream_sync)) {
            tcp_client_destroy(client);
            signal_stream_eof(stream_sync);
            return false;
        }

        const int recv_len = tcp_client_recv(client, wb->temp, wb->temp_cap, CLOUD_RECV_SLICE_TIMEOUT_MS);
        if (recv_len < 0) {
            break;
        }
        if (recv_len == 0) {
            idle_wait_ms += CLOUD_RECV_SLICE_TIMEOUT_MS;
            if (idle_wait_ms >= CLOUD_RECV_MAX_IDLE_MS) {
                break;
            }
            continue;
        }
        idle_wait_ms = 0U;

        for (int i = 0; i < recv_len; ++i) {
            const char c = wb->temp[i];
            if (total_snapshot + 1U < wb->resp_cap) {
                wb->resp[total_snapshot++] = c;
                wb->resp[total_snapshot] = '\0';
            }

            if (extracted_stream || b64_overflow) {
                continue;
            }

            switch (stream_state) {
            case FIND_KEY:
                if (c == audio_key[key_match]) {
                    ++key_match;
                    if (audio_key[key_match] == '\0') {
                        key_match = 0U;
                        stream_state = FIND_COLON;
                    }
                } else {
                    key_match = (c == audio_key[0]) ? 1U : 0U;
                }
                break;
            case FIND_COLON:
                if (c == ':') {
                    stream_state = FIND_VALUE_QUOTE;
                }
                break;
            case FIND_VALUE_QUOTE:
                if (c == '"') {
                    if (response_hook != NULL) {
                        (void)response_hook(wb->resp, response_hook_ctx);
                    }
                    stream_state = CAPTURE_VALUE;
                }
                break;
            case CAPTURE_VALUE:
                if (c == '"') {
                    extracted_stream = true;
                } else if (is_b64_char(c)) {
                    if (b64_batch_len + 1U >= sizeof(b64_batch)) {
                        if (!decode_and_send_b64_batch(
                                stream_sync->pcm_ringbuf, b64_batch, &b64_batch_len, false, &pcm_ctx)) {
                            tcp_client_destroy(client);
                            signal_stream_eof(stream_sync);
                            return false;
                        }
                    }
                    if (b64_batch_len + 1U >= sizeof(b64_batch)) {
                        b64_overflow = true;
                    } else {
                        b64_batch[b64_batch_len++] = c;
                    }
                }
                break;
            default:
                break;
            }
        }

        if (extracted_stream || b64_overflow) {
            break;
        }
    }

    tcp_client_destroy(client);

    if (!extracted_stream || b64_overflow) {
        signal_stream_eof(stream_sync);
        return false;
    }
    if (!decode_and_send_b64_batch(stream_sync->pcm_ringbuf, b64_batch, &b64_batch_len, true, &pcm_ctx)) {
        signal_stream_eof(stream_sync);
        return false;
    }

    xEventGroupSetBits(stream_sync->state_events, stream_sync->stream_eof_bit);
    const EventBits_t bits = xEventGroupWaitBits(
        stream_sync->state_events,
        stream_sync->stream_play_done_bit,
        pdTRUE,
        pdTRUE,
        pdMS_TO_TICKS(stream_sync->stream_play_done_timeout_ms));
    return (bits & stream_sync->stream_play_done_bit) != 0U;
}

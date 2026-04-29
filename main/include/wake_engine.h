#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*wake_engine_trigger_fn_t)(void *ctx);

typedef struct {
    SemaphoreHandle_t audio_sem;
    wake_engine_trigger_fn_t trigger_fn;
    void *trigger_ctx;
    uint32_t trigger_cooldown_ms;
    uint32_t mic_read_timeout_ms;
    uint32_t diag_log_interval_ms;
    uint32_t task_yield_ms;
    uint32_t loop_yield_every_n;
} wake_engine_config_t;

bool wake_engine_init(const wake_engine_config_t *cfg);
void wake_engine_task(void *arg);

#ifdef __cplusplus
}
#endif

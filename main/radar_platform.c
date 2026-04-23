#include "radar_platform.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint64_t radar_now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

void radar_sleep_ms(uint32_t delay_ms)
{
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

void radar_restart(void)
{
    esp_restart();
}

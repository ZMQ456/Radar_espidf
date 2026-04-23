#ifndef RADAR_UART_H
#define RADAR_UART_H

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "radar_manager.h"
#include "radar_config.h"

bool radar_uart_init(void);
void radar_uart_start_task(void);
bool radar_uart_is_initialized(void);
void sendRadarCommand(uint8_t ctrl, uint8_t cmd, uint8_t value);
void initR60ABD1(void);
size_t radar_uart_read_available(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms);
bool radar_parse_frame(const uint8_t *frame, size_t frame_len);

#endif

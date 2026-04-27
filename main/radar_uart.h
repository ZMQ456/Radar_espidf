#ifndef RADAR_UART_H
#define RADAR_UART_H

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "radar_manager.h"
#include "radar_config.h"

bool radar_uart_init(void);//初始化雷达UART
void radar_uart_start_task(void);
bool radar_uart_is_initialized(void);//检查雷达UART是否初始化
void sendRadarCommand(uint8_t ctrl, uint8_t cmd, uint8_t value);//发送雷达命令
void initR60ABD1(void);//初始化R60ABD1
size_t radar_uart_read_available(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms);//读取可用数据
bool radar_parse_frame(const uint8_t *frame, size_t frame_len);//解析雷达帧 

#endif

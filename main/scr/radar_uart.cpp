/**
 * @file radar_uart.cpp
 * @brief 毫米波雷达 UART 通信模块
 * 
 * 功能概述：
 * - 管理 ESP32 与 R60ABD1 雷达的 UART 通信
 * - 解析雷达上传的数据帧（存在、运动、心率、呼吸、睡眠等）
 * - 发送配置命令到雷达（查询、设置参数）
 * 
 * 核心功能：
 * 1. UART 初始化：配置波特率、数据位、停止位等
 * 2. 帧解析：从字节流中提取完整数据帧
 * 3. 数据解析：解析不同类型的雷达数据帧
 * 4. 命令发送：向雷达发送配置命令
 * 
 * 数据帧格式：
 * - 帧头：0x53 0x59（2 字节）
 * - 控制字：标识数据类型（0x80=存在/运动，0x81=呼吸，0x85=心率，0x84=睡眠）
 * - 命令字：具体数据标识
 * - 数据长度：2 字节（高字节在前）
 * - 数据区：变长
 * - 校验和：1 字节（累加和）
 * - 帧尾：0x54 0x43（2 字节）
 * 
 * 数据类型：
 * - 存在检测：人体存在、运动状态
 * - 生命体征：心率、呼吸率、波形数据
 * - 睡眠监测：睡眠状态、评分、分期
 * - 位置信息：X/Y/Z 坐标
 * - 异常告警：挣扎、离床、呼吸暂停
 * 
 * 依赖：
 * - ESP-IDF UART 驱动
 * - radar_platform.h（时间函数）
 * - radar_manager.h（全局 sensorData）
 */
// ============================================================================
// 头文件包含和全局变量
// ============================================================================
#include "radar_uart.h"  // UART 接口声明

#include <inttypes.h>    // 格式化宏（PRIu64）
#include <cstring>       // C 字符串操作

#include "driver/uart.h" // ESP32 UART 驱动
#include "esp_log.h"     // ESP32 日志系统
#include "radar_platform.h" // 平台接口（时间函数）

static const char *TAG = "radar_uart";  // 日志标签
static const uart_port_t RADAR_UART_NUM = static_cast<uart_port_t>(RADAR_UART_PORT);  // UART 端口号

// UART 接收任务相关
static TaskHandle_t radarUartTaskHandle = nullptr;  // 任务句柄
static bool radarUartInitialized = false;            // 初始化标志

// 帧缓冲区和状态
static uint8_t frameBuffer[RADAR_FRAME_BUFFER_SIZE];  // 帧数据缓冲区
static size_t frameIndex = 0;                          // 当前写入位置
static bool inFrame = false;                           // 是否正在接收帧
static uint8_t prevByte = 0;                           // 上一个接收的字节（用于检测帧头）

// ============================================================================
// 内部辅助函数
// ============================================================================
/**
 * @brief 解析有符号坐标值
 * 
 * @param raw_value 原始 16 位值（最高位为符号位）
 * @return int16_t 有符号坐标值
 * 
 * 编码格式：
 * - 最高位（bit 15）为符号位：1=负数，0=正数
 * - 低 15 位（bit 0-14）为绝对值
 * 
 * 示例：
 * - 0x0064 (100) → 100
 * - 0x8064 (100|0x8000) → -100
 * 
 * 用途：
 * - 解析人体位置坐标（pos_x, pos_y, pos_z）
 */
static int16_t parse_signed_coordinate(uint16_t raw_value)
{
    return (raw_value & 0x8000U) ? -static_cast<int16_t>(raw_value & 0x7FFFU)
                                 : static_cast<int16_t>(raw_value);
}

// ============================================================================
// 雷达数据帧解析
// ============================================================================
/**
 * @brief 解析雷达数据帧
 * 
 * @param frame 帧数据缓冲区
 * @param frame_len 帧长度
 * @return true 解析成功
 * @return false 解析失败（帧头/帧尾/校验和错误）
 * 
 * 数据帧格式：
 * +--------+--------+--------+--------+--------+--------+
 * | 帧头 1 | 帧头 2 | 控制字 | 命令字 | 长度 H | 长度 L |
 * +--------+--------+--------+--------+--------+--------+
 * |  数据区 (变长) ...                                     |
 * +--------+--------+--------+--------+--------+--------+
 * | 校验和 | 帧尾 1 | 帧尾 2 |
 * +--------+--------+--------+
 * 
 * 控制字分类：
 * - 0x80: 存在/运动/位置
 * - 0x81: 呼吸相关
 * - 0x85: 心率相关
 * - 0x84: 睡眠相关
 * 
 * 处理流程：
 * 1. 验证帧头帧尾（0x53 0x59 ... 0x54 0x43）
 * 2. 计算校验和（累加和）
 * 3. 解析控制字和命令字
 * 4. 根据数据类型更新 sensorData
 * 5. 记录时间戳
 */
bool radar_parse_frame(const uint8_t *frame, size_t frame_len)
{
    // 步骤 1: 参数有效性检查
    if (frame == nullptr || frame_len < 8) {
        return false;
    }

    // 步骤 2: 验证帧头帧尾
    // 帧头：0x53 0x59 ("SY")
    // 帧尾：0x54 0x43 ("TC")
    if (frame[0] != FRAME_HEADER1 || frame[1] != FRAME_HEADER2 ||
        frame[frame_len - 2] != FRAME_TAIL1 || frame[frame_len - 1] != FRAME_TAIL2) {
        return false;
    }

    // 步骤 3: 计算并验证校验和
    // 校验和 = 帧头 + 控制字 + 命令字 + 长度 + 数据区（不包括校验和本身和帧尾）
    uint8_t checksum = 0;
    for (size_t i = 0; i < frame_len - 3; ++i) {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    if (checksum != frame[frame_len - 3]) {
        ESP_LOGW(TAG, "frame checksum mismatch calc=0x%02X recv=0x%02X",
                 checksum, frame[frame_len - 3]);
        return false;
    }

    // 步骤 4: 解析帧头信息
    const uint8_t ctrlByte = frame[2];  // 控制字（数据类型）
    const uint8_t cmdByte = frame[3];   // 命令字（具体标识）
    const uint16_t dataLen = static_cast<uint16_t>((frame[4] << 8) | frame[5]);  // 数据长度

    // 日志 4：解析器入口日志
    ESP_LOGI(TAG, "parse frame ctrl=0x%02X cmd=0x%02X dataLen=%u",
             ctrlByte, cmdByte, (unsigned)dataLen);

    // 步骤 5: 验证数据长度
    // 帧结构：帧头 (2) + 控制字 (1) + 命令字 (1) + 长度 (2) + 数据区 (dataLen) + 校验和 (1) + 帧尾 (2)
    if ((6U + dataLen + 3U) > frame_len) {
        ESP_LOGW(TAG, "frame length mismatch dataLen=%u frameLen=%u",
                 static_cast<unsigned>(dataLen), static_cast<unsigned>(frame_len));
        return false;
    }

    // 步骤 6: 根据控制字和命令字解析数据
    switch (ctrlByte) {
    // ========================================================================
    // 控制字 0x80: 存在检测、运动状态、位置信息
    // ========================================================================
    case 0x80:
        switch (cmdByte) {
        // 命令字 0x01: 人体存在状态
        case 0x01:
            if (dataLen >= 1) {
                sensorData.presence = frame[6];  // 0=无人，1=有人
                sensorData.last_update_ms = radar_now_ms();
            }
            break;
        
        // 命令字 0x02: 运动状态
        case 0x02:
            if (dataLen >= 1) {
                sensorData.motion = frame[6];  // 0=静止，1=运动
                sensorData.last_update_ms = radar_now_ms();
            }
            break;
        
        // 命令字 0x03: 体动幅度
        case 0x03:
            if (dataLen >= 1) {
                sensorData.body_movement = frame[6];  // 0-100 幅度值
                sensorData.last_update_ms = radar_now_ms();
            }
            break;
        
        // 命令字 0x04: 人体距离（毫米）
        case 0x04:
            if (dataLen >= 2) {
                sensorData.distance = static_cast<uint16_t>((frame[6] << 8) | frame[7]);  // 高字节在前
                sensorData.last_update_ms = radar_now_ms();
            }
            break;
        
        // 命令字 0x05: 三维坐标（X/Y/Z）
        case 0x05:
            if (dataLen >= 6) {
                // 每个坐标 2 字节，有符号数（最高位为符号位）
                sensorData.pos_x = parse_signed_coordinate(static_cast<uint16_t>((frame[6] << 8) | frame[7]));
                sensorData.pos_y = parse_signed_coordinate(static_cast<uint16_t>((frame[8] << 8) | frame[9]));
                sensorData.pos_z = parse_signed_coordinate(static_cast<uint16_t>((frame[10] << 8) | frame[11]));
                sensorData.last_update_ms = radar_now_ms();
            }
            break;
        default:
            break;
        }
        break;

    // ========================================================================
    // 控制字 0x81: 呼吸相关数据
    // ========================================================================
    case 0x81:
        switch (cmdByte) {
        // 命令字 0x01: 呼吸状态（未使用）
        case 0x01:
            if (dataLen >= 1) sensorData.breath_status = frame[6];
            break;
        
        // 命令字 0x02: 呼吸率（次/分钟）
        case 0x02:
            if (dataLen >= 1) {
                sensorData.breath_rate = static_cast<float>(frame[6]);  // 整数转浮点
                // 有效性判断：0-35 次/分钟为正常范围
                sensorData.breath_valid = (sensorData.breath_rate >= 0.0f && sensorData.breath_rate <= 35.0f);
                sensorData.last_update_ms = radar_now_ms();
                ESP_LOGI(TAG, "breath updated rr=%.1f valid=%u", sensorData.breath_rate, sensorData.breath_valid);
            }
            break;
        
        // 命令字 0x05: 呼吸波形数据（5 个点）
        case 0x05:
            if (dataLen >= 5) {
                // 波形数据为偏移码（128 为 0 点）
                for (uint16_t i = 0; i < 5U && i < dataLen; ++i) {
                    sensorData.breath_waveform[i] = static_cast<int8_t>(frame[6 + i] - 128);
                }
                sensorData.breathing_waveform = sensorData.breath_waveform[0];  // 取第一个点作为代表
            }
            break;
        default:
            break;
        }
        break;

    // ========================================================================
    // 控制字 0x85: 心率相关数据
    // ========================================================================
    case 0x85:
        switch (cmdByte) {
        // 命令字 0x02: 心率（次/分钟）
        case 0x02:
            if (dataLen >= 1) {
                sensorData.heart_rate = static_cast<float>(frame[6]);  // 整数转浮点
                // 有效性判断：60-120 次/分钟为正常范围
                sensorData.heart_valid = (sensorData.heart_rate >= 60.0f && sensorData.heart_rate <= 120.0f);
                sensorData.last_update_ms = radar_now_ms();
                ESP_LOGI(TAG, "heart updated hr=%.1f valid=%u", sensorData.heart_rate, sensorData.heart_valid);
            }
            break;
        
        // 命令字 0x05: 心率波形数据（5 个点）
        case 0x05:
            if (dataLen >= 5) {
                // 波形数据为偏移码（128 为 0 点）
                for (uint16_t i = 0; i < 5U && i < dataLen; ++i) {
                    sensorData.heart_waveform[i] = static_cast<int8_t>(frame[6 + i] - 128);
                }
                sensorData.heartbeat_waveform = sensorData.heart_waveform[0];  // 取第一个点作为代表
            }
            break;
        default:
            break;
        }
        break;

    // ========================================================================
    // 控制字 0x84: 睡眠相关数据
    // ========================================================================
    case 0x84:
        switch (cmdByte) {
        // 命令字 0x01/0x81: 在床状态
        case 0x01:
        case 0x81:
            if (dataLen >= 1) {
                sensorData.bed_status = frame[6];      // 在床状态
                sensorData.bed_entry = frame[6];       // 入床标志
            }
            break;
        
        // 命令字 0x03/0x83: 清醒时间（分钟）
        case 0x03:
        case 0x83:
            if (dataLen >= 2) {
                sensorData.awake_time = static_cast<uint16_t>((frame[6] << 8) | frame[7]);  // 高字节在前
            }
            break;
        
        // 命令字 0x04/0x84: 浅睡时间（分钟）
        case 0x04:
        case 0x84:
            if (dataLen >= 2) {
                sensorData.light_sleep_time = static_cast<uint16_t>((frame[6] << 8) | frame[7]);
            }
            break;
        
        // 命令字 0x05/0x85: 深睡时间（分钟）
        case 0x05:
        case 0x85:
            if (dataLen >= 2) {
                sensorData.deep_sleep_time = static_cast<uint16_t>((frame[6] << 8) | frame[7]);
            }
            break;
        
        // 命令字 0x06: 睡眠评分（0-100）
        case 0x06:
            if (dataLen >= 1) {
                sensorData.sleep_score = frame[6];
            }
            break;
        
        // 命令字 0x86: 睡眠评分（扩展）
        case 0x86:
            if (dataLen >= 2) {
                sensorData.sleep_score = frame[6];
            }
            break;
        
        // 命令字 0x0C/0x8D: 睡眠汇总（存在、状态、平均心率呼吸）
        case 0x0C:
        case 0x8D:
            if (dataLen >= 8) {
                sensorData.presence = frame[6];              // 在床状态
                sensorData.sleep_state = frame[7];           // 睡眠状态（0=清醒，1=入睡）
                sensorData.avg_breath_rate = frame[8];       // 平均呼吸率
                sensorData.avg_heart_rate = frame[9];        // 平均心率
                sensorData.turnover_count = frame[10];       // 翻身次数
                sensorData.large_move_ratio = frame[11];     // 大幅度活动比例
                sensorData.small_move_ratio = frame[12];     // 小幅度活动比例
                sensorData.apnea_count = frame[13];          // 呼吸暂停次数
                sensorData.last_update_ms = radar_now_ms();
                ESP_LOGI(TAG, "sleep summary presence=%u sleep=%u avg_rr=%u avg_hr=%u",
                         sensorData.presence, sensorData.sleep_state,
                         sensorData.avg_breath_rate, sensorData.avg_heart_rate);
            }
            break;
        
        // 命令字 0x0D/0x8F: 完整睡眠报告（评分、分期、离床时间等）
        case 0x0D:
        case 0x8F:
            if (dataLen >= 12) {
                sensorData.sleep_score = frame[6];                          // 睡眠评分
                sensorData.sleep_total_time = static_cast<uint16_t>((frame[7] << 8) | frame[8]);  // 总睡眠时间（分钟）
                sensorData.sleep_time = sensorData.sleep_total_time;        // 睡眠总时长
                sensorData.awake_ratio = frame[9];                          // 清醒比例（%）
                sensorData.light_sleep_ratio = frame[10];                   // 浅睡比例（%）
                sensorData.deep_sleep_ratio = frame[11];                    // 深睡比例（%）
                sensorData.bed_Out_Time = frame[12];                        // 离床时间（分钟）
                sensorData.turn_count = frame[13];                          // 翻身次数
                sensorData.turnover_count = frame[14];                      // 翻身次数（别名）
                sensorData.avg_breath_rate = frame[15];                     // 平均呼吸率
                sensorData.avg_heart_rate = frame[16];                      // 平均心率
                sensorData.apnea_count = frame[17];                         // 呼吸暂停次数
                sensorData.last_update_ms = radar_now_ms();
            }
            break;
        
        // 命令字 0x0E/0x8E: 异常状态标志
        case 0x0E:
        case 0x8E:
            if (dataLen >= 1) {
                sensorData.abnormal_state = frame[6];  // 各 bit 表示不同异常
            }
            break;
        
        // 命令字 0x10/0x90: 睡眠等级
        case 0x10:
        case 0x90:
            if (dataLen >= 1) {
                sensorData.sleep_grade = frame[6];  // 睡眠质量等级
            }
            break;
        
        // 命令字 0x11/0x91: 挣扎告警
        case 0x11:
        case 0x91:
            if (dataLen >= 1) {
                sensorData.struggle_alert = frame[6];  // 挣扎告警标志
            }
            break;
        
        // 命令字 0x12/0x92: 无人告警
        case 0x12:
        case 0x92:
            if (dataLen >= 1) {
                sensorData.no_one_alert = frame[6];  // 无人告警标志
            }
            break;
        default:
            break;
        }
        break;

    // ========================================================================
    // 默认情况：未知控制字
    // ========================================================================
    default:
        break;
    }

    // 步骤 7: 最终有效性检查（兜底逻辑）
    sensorData.last_update_ms = radar_now_ms();
    sensorData.heart_valid = (sensorData.heart_rate > 0.0f && sensorData.heart_rate < 200.0f);   // 放宽范围到 0-200
    sensorData.breath_valid = (sensorData.breath_rate >= 0.1f && sensorData.breath_rate <= 60.0f);  // 放宽范围到 0.1-60

    // 步骤 8: 打印解析结果日志
    ESP_LOGI(TAG,
             "sensor presence=%u motion=%u dist=%u hr=%.1f rr=%.1f sleep=%u body=%u updated=%" PRIu64,
             sensorData.presence,
             sensorData.motion,
             sensorData.distance,
             sensorData.heart_rate,
             sensorData.breath_rate,
             sensorData.sleep_state,
             sensorData.body_movement,
             sensorData.last_update_ms);
    return true;
}

// ============================================================================
// UART 接收任务
// ============================================================================
/**
 * @brief UART 接收任务（后台运行）
 * 
 * @param parameter 任务参数（未使用）
 * 
 * 任务逻辑：
 * 1. 循环读取 UART 数据（每次最多 128 字节）
 * 2. 打印接收日志（前 32 字节的 hex）
 * 3. 逐字节解析，检测帧头帧尾
 * 4. 组帧完成后调用 radar_parse_frame 解析
 * 5. 处理异常（缓冲区溢出）
 * 
 * 帧同步逻辑：
 * - 检测帧头：0x53 0x59（连续两个字节）
 * - 接收数据：存入 frameBuffer
 * - 检测帧尾：0x54 0x43（连续两个字节）
 * - 组帧完成：调用解析函数
 * 
 * 特点：
 * - 无限循环（任务优先级 5）
 * - 阻塞式读取（超时 100ms）
 * - 自动重连（永不退出）
 */
static void radar_uart_task(void *parameter)
{
    (void)parameter;

    uint8_t rxBuffer[128];  // 每次读取的缓冲区

    while (true) {
        // 步骤 1: 从 UART 读取数据（阻塞 100ms）
        const int len = uart_read_bytes(RADAR_UART_NUM, rxBuffer, sizeof(rxBuffer), pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;  // 无数据，继续等待
        }

        // 日志 1：确认 UART 收到字节
        ESP_LOGI(TAG, "radar rx len=%d", len);

        // 日志 2：打印前 32 字节的 hex（调试用）
        if (len > 0) {
            char hexBuf[256] = {};
            int used = 0;
            for (int i = 0; i < len && i < 32; ++i) {
                used += snprintf(hexBuf + used, sizeof(hexBuf) - used, "%02X ", rxBuffer[i]);
                if (used >= (int)sizeof(hexBuf)) break;
            }
            ESP_LOGI(TAG, "radar rx hex: %s", hexBuf);
        }

        // 步骤 2: 逐字节解析
        for (int i = 0; i < len; ++i) {
            const uint8_t c = rxBuffer[i];

            if (!inFrame) {
                // 步骤 2.1: 检测帧头
                // 如果上一个字节是 0x53，当前是 0x59，则检测到帧头
                if (prevByte == FRAME_HEADER1 && c == FRAME_HEADER2) {
                    inFrame = true;
                    frameIndex = 0;
                    frameBuffer[frameIndex++] = FRAME_HEADER1;  // 写入帧头第 1 字节
                    frameBuffer[frameIndex++] = FRAME_HEADER2;  // 写入帧头第 2 字节
                }
            } else {
                // 步骤 2.2: 接收帧数据
                if (frameIndex < sizeof(frameBuffer)) {
                    frameBuffer[frameIndex++] = c;

                    // 步骤 2.3: 检测帧尾
                    // 检查最后两个字节是否为 0x54 0x43
                    if (frameIndex >= 2 &&
                        frameBuffer[frameIndex - 2] == FRAME_TAIL1 &&
                        frameBuffer[frameIndex - 1] == FRAME_TAIL2) {
                        // 日志 3：组帧完成
                        ESP_LOGI(TAG, "radar frame complete len=%u", (unsigned)frameIndex);
                        
                        // 步骤 2.4: 解析帧数据
                        const bool parsed = radar_parse_frame(frameBuffer, frameIndex);
                        if (!parsed) {
                            ESP_LOGW(TAG, "received invalid radar frame, len=%u", static_cast<unsigned>(frameIndex));
                        }
                        
                        // 步骤 2.5: 重置状态，准备接收下一帧
                        inFrame = false;
                        frameIndex = 0;
                    }
                } else {
                    // 步骤 2.6: 缓冲区溢出处理
                    ESP_LOGW(TAG, "frame buffer overflow, dropping frame");
                    inFrame = false;
                    frameIndex = 0;
                }
            }

            prevByte = c;  // 记录上一个字节，用于检测帧头
        }
    }
}

// ============================================================================
// 初始化和任务管理
// ============================================================================
/**
 * @brief 初始化 UART 接口
 * 
 * @return true 初始化成功
 * @return false 初始化失败
 * 
 * 配置参数：
 * - 波特率：RADAR_BAUD_RATE（由 radar_uart.h 定义）
 * - 数据位：8 位
 * - 停止位：1 位
 * - 校验位：无
 * - 流控制：无
 * 
 * 处理流程：
 * 1. 检查是否已初始化（避免重复）
 * 2. 配置 UART 参数
 * 3. 安装 UART 驱动（只接收，不发送）
 * 4. 应用参数配置
 * 5. 配置 GPIO 引脚（TX/RX）
 * 6. 设置初始化标志
 * 
 * 注意：
 * - 此函数只初始化硬件，不启动接收任务
 * - 需要调用 radar_uart_start_task() 启动后台任务
 */
bool radar_uart_init(void)
{
    if (radarUartInitialized) {
        return true;  // 避免重复初始化
    }

    // 步骤 1: 配置 UART 参数
    uart_config_t uartConfig = {};
    uartConfig.baud_rate = RADAR_BAUD_RATE;
    uartConfig.data_bits = UART_DATA_8_BITS;
    uartConfig.parity = UART_PARITY_DISABLE;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uartConfig.rx_flow_ctrl_thresh = 0;
    uartConfig.source_clk = UART_SCLK_DEFAULT;
    uartConfig.flags.allow_pd = 0;  // 睡眠时不允许掉电保存
    uartConfig.flags.backup_before_sleep = 0;

    // 步骤 2: 安装 UART 驱动
    // 只配置 RX 缓冲区（1024 字节），TX 缓冲区为 0（只接收不发送）
    esp_err_t err = uart_driver_install(RADAR_UART_NUM, RADAR_UART_BUFFER_SIZE, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", static_cast<int>(err));
        return false;
    }

    // 步骤 3: 应用参数配置
    err = uart_param_config(RADAR_UART_NUM, &uartConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", static_cast<int>(err));
        return false;
    }

    // 步骤 4: 配置 GPIO 引脚
    err = uart_set_pin(RADAR_UART_NUM, RADAR_TX_PIN, RADAR_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %d", static_cast<int>(err));
        return false;
    }

    // 步骤 5-6: 设置标志和日志
    radarUartInitialized = true;
    ESP_LOGI(TAG, "uart initialized on port=%d tx=%d rx=%d baud=%d",
             static_cast<int>(RADAR_UART_NUM),
             RADAR_TX_PIN,
             RADAR_RX_PIN,
             RADAR_BAUD_RATE);
    return true;
}

/**
 * @brief 启动 UART 接收任务
 * 
 * 处理流程：
 * 1. 检查是否已初始化，未初始化则先初始化
 * 2. 检查任务是否已创建（避免重复）
 * 3. 创建 FreeRTOS 任务（优先级 5，栈大小 4096 字节）
 * 
 * 任务参数：
 * - 名称："radar_uart_task"
 * - 栈大小：4096 字节
 * - 优先级：5（中等优先级）
 * - 任务句柄：radarUartTaskHandle
 * 
 * 注意：
 * - 此函数在系统启动时由 radar_manager.cpp 调用
 * - 任务会一直运行直到系统重启
 */
void radar_uart_start_task(void)
{
    if (!radarUartInitialized) {
        if (!radar_uart_init()) {
            return;  // 初始化失败，直接返回
        }
    }

    if (radarUartTaskHandle != nullptr) {
        return;  // 避免重复创建任务
    }

    xTaskCreate(radar_uart_task, "radar_uart_task", 4096, nullptr, 5, &radarUartTaskHandle);
}

/**
 * @brief 检查 UART 是否已初始化
 * 
 * @return true 已初始化
 * @return false 未初始化
 */
bool radar_uart_is_initialized(void)
{
    return radarUartInitialized;
}

/**
 * @brief 发送雷达配置命令
 * 
 * @param ctrl 控制字
 * @param cmd 命令字
 * @param value 参数值
 * 
 * 命令帧格式：
 * +--------+--------+--------+--------+--------+--------+
 * | 0x53   | 0x59   | ctrl   | cmd    | 0x00   | 0x01   |
 * +--------+--------+--------+--------+--------+--------+
 * | value  | checksum | 0x00 | 0x54   | 0x43   |
 * +--------+--------+--------+--------+--------+
 * 
 * 处理流程：
 * 1. 检查初始化状态
 * 2. 构建命令帧（10 字节）
 * 3. 计算校验和（前 7 字节累加和）
 * 4. 通过 UART 发送
 * 5. 打印日志
 * 
 * 示例命令：
 * - 查询存在：sendRadarCommand(0x80, 0x81, 0x01)
 * - 查询心率：sendRadarCommand(0x85, 0x80, 0x0F)
 * - 设置模式：sendRadarCommand(0x84, 0x00, 0x01)
 * 
 * 注意：
 * - 此函数在初始化时由 initR60ABD1() 调用
 * - 运行时一般不调用（雷达自动上报数据）
 */
void sendRadarCommand(uint8_t ctrl, uint8_t cmd, uint8_t value)
{
    if (!radarUartInitialized && !radar_uart_init()) {
        return;
    }

    // 构建命令帧
    uint8_t command[10];
    command[0] = FRAME_HEADER1;   // 0x53
    command[1] = FRAME_HEADER2;   // 0x59
    command[2] = ctrl;            // 控制字
    command[3] = cmd;             // 命令字
    command[4] = 0x00;            // 数据长度高字节（0）
    command[5] = 0x01;            // 数据长度低字节（1）
    command[6] = value;           // 参数值
    command[7] = 0x00;            // 校验和（待计算）
    
    // 计算校验和（前 7 字节累加和）
    for (int i = 0; i < 7; ++i) {
        command[7] += command[i];
    }
    
    command[8] = FRAME_TAIL1;     // 0x54
    command[9] = FRAME_TAIL2;     // 0x43

    // 发送命令
    const int written = uart_write_bytes(RADAR_UART_NUM, command, sizeof(command));
    ESP_LOGI(TAG, "sent radar command ctrl=0x%02X cmd=0x%02X value=0x%02X bytes=%d",
             ctrl, cmd, value, written);
}

/**
 * @brief 初始化 R60ABD1 雷达传感器
 * 
 * 处理流程：
 * 1. 检查 UART 是否已初始化
 * 2. 发送查询存在命令（唤醒雷达）
 * 3. 发送 12 个配置命令（每个间隔 50ms）
 * 
 * 启动命令列表（12 个）：
 * 1. {0x80, 0x00, 0x01}: 启用存在检测
 * 2. {0x81, 0x00, 0x01}: 启用呼吸检测
 * 3. {0x85, 0x00, 0x01}: 启用心率检测
 * 4. {0x84, 0x00, 0x01}: 启用睡眠检测
 * 5. {0x81, 0x0C, 0x01}: 查询呼吸状态
 * 6. {0x85, 0x0A, 0x01}: 查询心率状态
 * 7. {0x84, 0x13, 0x01}: 查询睡眠状态
 * 8. {0x84, 0x14, 0x01}: 查询在床状态
 * 9. {0x80, 0x80, 0x0F}: 设置存在检测参数
 * 10. {0x81, 0x80, 0x0F}: 设置呼吸检测参数
 * 11. {0x85, 0x80, 0x0F}: 设置心率检测参数
 * 12. {0x84, 0x80, 0x0F}: 设置睡眠检测参数
 * 
 * 注意：
 * - 此函数在系统启动时由 radar_manager.cpp 调用
 * - 每个命令间隔 50ms，总耗时约 600ms
 * - 雷达会开始自动上报数据
 */
void initR60ABD1(void)
{
    // 启动命令列表（12 个配置命令）
    static const uint8_t startupCommands[][3] = {
        {0x80, 0x00, 0x01},  // 启用存在检测
        {0x81, 0x00, 0x01},  // 启用呼吸检测
        {0x85, 0x00, 0x01},  // 启用心率检测
        {0x84, 0x00, 0x01},  // 启用睡眠检测
        {0x81, 0x0C, 0x01},  // 查询呼吸状态
        {0x85, 0x0A, 0x01},  // 查询心率状态
        {0x84, 0x13, 0x01},  // 查询睡眠状态
        {0x84, 0x14, 0x01},  // 查询在床状态
        {0x80, 0x80, 0x0F},  // 设置存在检测参数
        {0x81, 0x80, 0x0F},  // 设置呼吸检测参数
        {0x85, 0x80, 0x0F},  // 设置心率检测参数
        {0x84, 0x80, 0x0F},  // 设置睡眠检测参数
    };

    // 步骤 1: 确保 UART 已初始化
    if (!radarUartInitialized && !radar_uart_init()) {
        return;
    }

    // 步骤 2: 发送查询存在命令（唤醒雷达）
    const uint8_t queryPresenceCmd[] = {0x53, 0x59, 0x80, 0x81, 0x00, 0x01, 0x00, 0x7D, 0x54, 0x43};
    uart_write_bytes(RADAR_UART_NUM, queryPresenceCmd, sizeof(queryPresenceCmd));
    radar_sleep_ms(50);  // 等待雷达响应

    // 步骤 3: 发送 12 个配置命令
    for (const auto &startupCommand : startupCommands) {
        sendRadarCommand(startupCommand[0], startupCommand[1], startupCommand[2]);
        radar_sleep_ms(50);  // 每个命令间隔 50ms
    }
}

/**
 * @brief 读取 UART 数据（阻塞式）
 * 
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @param timeout_ms 超时时间（毫秒）
 * @return size_t 实际读取的字节数
 * 
 * 处理流程：
 * 1. 检查初始化状态
 * 2. 调用 uart_read_bytes 读取数据
 * 3. 返回实际读取的字节数
 * 
 * 注意：
 * - 此函数一般不使用（数据由后台任务自动解析）
 * - 用于特殊场景下的手动读取
 * - 阻塞时间由 timeout_ms 指定
 * 
 * 用途：
 * - 调试模式下的手动读取
 * - 特殊命令的响应读取
 */
size_t radar_uart_read_available(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms)
{
    if (!radarUartInitialized && !radar_uart_init()) {
        return 0;  // 初始化失败返回 0
    }

    const int len = uart_read_bytes(RADAR_UART_NUM, buffer, buffer_size, pdMS_TO_TICKS(timeout_ms));
    return len > 0 ? static_cast<size_t>(len) : 0;
}

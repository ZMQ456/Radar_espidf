/**
 * @file tasks_manager.cpp
 * @brief 多任务管理器模块
 * 
 * 功能概述：
 * - 管理 ESP32 系统的多个后台任务
 * - 协调 WiFi、BLE、MQTT、雷达、LED 等模块
 * - 提供设备配置、数据上报、用户交互功能
 * 
 * 核心任务（7 个）：
 * 1. radar_command_task: 定时发送雷达查询命令
 * 2. radar_status_task: 监控雷达状态，上报 MQTT/InfluxDB
 * 3. radar_analysis_task: 生理数据分析（情绪、睡眠）
 * 4. device_console_task: 串口命令行接口
 * 5. boot_button_task: 监控启动按钮（长按清除配置）
 * 6. led_control_task: 网络状态指示灯控制
 * 7. wifi_monitor_task: WiFi 连接状态监控
 * 8. ble_config_task: BLE 配置和数据发送
 * 
 * 任务优先级：
 * - boot_button_task: 2（中高优先级）
 * - radar_command_task: 4（高优先级）
 * - radar_analysis_task: 3（中优先级）
 * - radar_status_task: 2（中优先级）
 * - wifi_monitor_task: 2（中优先级）
 * - ble_config_task: 2（中优先级）
 * - device_console_task: 1（低优先级）
 * - led_control_task: 1（低优先级）
 * 
 * 全局状态：
 * - currentNetworkStatus: 网络状态（初始/连接中/已连接/断开）
 * - gContinuousSendEnabled: 连续发送模式开关
 * - gClearConfigRequested: 清除配置标志
 * - gForceLedOff: 强制关闭 LED 标志
 * 
 * 依赖模块：
 * - wifi_manager: WiFi 连接管理
 * - ble_manager: BLE 通信管理
 * - mqtt_manager: MQTT 客户端管理
 * - influx_manager: InfluxDB 数据上报
 * - radar_manager: 雷达数据管理
 * - device_command: 命令行解析
 */
#include "tasks_manager.h"

#include <inttypes.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "app_config.h"
#include "ble_manager.h"
#include "device_identity.h"
#include "device_command.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "influx_manager.h"
#include "emotion_analyzer_simple.h"
#include "radar_manager.h"
#include "sleep_analyzer.h"
#include "radar_platform.h"
#include "radar_uart.h"
#include "system_state.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

static const char *TAG = "tasks_manager";

static TaskHandle_t radarCommandTaskHandle = nullptr;
static TaskHandle_t radarStatusTaskHandle = nullptr;
static TaskHandle_t radarAnalysisTaskHandle = nullptr;
static TaskHandle_t deviceConsoleTaskHandle = nullptr;
static TaskHandle_t bootButtonTaskHandle = nullptr;
static TaskHandle_t ledControlTaskHandle = nullptr;
static TaskHandle_t wifiMonitorTaskHandle = nullptr;
static TaskHandle_t bleConfigTaskHandle = nullptr;
static bool tasksInitialized = false;
TaskNetworkStatus currentNetworkStatus = NET_INITIAL;
static uint64_t gLastBlinkTimeMs = 0U;
static bool gLedState = false;
static uint32_t gBreatheValue = APP_LED_BREATHE_MIN;
static bool gBreatheIncreasing = true;
static bool gClearConfigRequested = false;
static bool gForceLedOff = false;
static bool gContinuousSendEnabled = false;
static bool gHasLastContinuousData = false;
static uint32_t gContinuousSendIntervalMs = 500U;
static uint64_t gLastContinuousCheckMs = 0U;
static SensorData gLastContinuousData = {};
static RadarReportSnapshot gLastInfluxDailySnapshot = {};
static bool gHasLastInfluxDailySnapshot = false;
static uint64_t gLastInfluxDailySendMs = 0U;
static uint64_t gLastInfluxSleepSendMs = 0U;
static uint64_t gLastMqttHeartbeatMs = 0U;
static uint64_t gLastMqttDailySendMs = 0U;
static uint64_t gLastMqttSleepSendMs = 0U;

static constexpr uint32_t kInfluxDailyForceSendIntervalMs = 60000U;
static constexpr uint32_t kInfluxDailyMinSendIntervalMs = 1500U;
static constexpr uint32_t kInfluxSleepSendIntervalMs = 5000U;
static constexpr uint32_t kMqttHeartbeatIntervalMs = 10000U;
static constexpr uint32_t kMqttDailyDataIntervalMs = 5000U;
static constexpr uint32_t kMqttSleepDataIntervalMs = 10000U;

/**
 * @brief 为当前任务注册软件看门狗
 * 
 * @param task_name 任务名称（用于日志输出）
 * 
 * 功能说明：
 * - 将当前任务添加到 ESP32 软件看门狗监控列表
 * - 防止任务阻塞导致系统死机
 * - 看门狗超时后会自动重启系统
 * 
 * 处理流程：
 * 1. 检查看门狗是否已启用
 * 2. 如果已启用（ESP_OK）→ 直接返回
 * 3. 如果是无效状态（ESP_ERR_INVALID_STATE）→ 返回
 * 4. 否则调用 esp_task_wdt_add 添加当前任务
 * 5. 记录失败日志（如果添加失败）
 * 
 * 用途：
 * - 所有长时间运行的任务都应该注册看门狗
 * - 配合 task_watchdog_reset_current 使用
 * 
 * 注意：
 * - 注册后必须定期调用 task_watchdog_reset_current
 * - 否则看门狗超时会导致系统重启
 */
static void task_watchdog_register_current(const char *task_name)
{
    // 步骤 1: 检查看门狗状态
    const esp_err_t status = esp_task_wdt_status(nullptr);
    if (status == ESP_OK) {
        return;  // 看门狗已启用，无需操作
    }
    if (status == ESP_ERR_INVALID_STATE) {
        return;  // 看门狗未初始化或已禁用
    }

    // 步骤 2: 添加当前任务到看门狗监控列表
    const esp_err_t err = esp_task_wdt_add(nullptr);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "watchdog add failed for %s: %s", task_name, esp_err_to_name(err));
    }
}

/**
 * @brief 重置当前任务的看门狗计时器
 * 
 * 功能说明：
 * - 重置软件看门狗的倒计时
 * - 防止看门狗超时重启系统
 * 
 * 处理流程：
 * 1. 检查看门狗是否已启用
 * 2. 如果已启用 → 调用 esp_task_wdt_reset 重置
 * 3. 如果未启用 → 不做任何操作
 * 
 * 用途：
 * - 在任务主循环中定期调用
 * - 向看门狗报告"我还活着"
 * 
 * 注意：
 * - 必须在注册看门狗后使用
 * - 调用间隔必须小于看门狗超时时间
 * - 通常与 esp_task_wdt_add 配合使用
 */
static void task_watchdog_reset_current(void)
{
    // 检查看门狗状态并重置
    if (esp_task_wdt_status(nullptr) == ESP_OK) {
        (void)esp_task_wdt_reset();
    }
}

/**
 * @brief 查找 JSON 值的起始位置
 * 
 * @param json JSON 字符串
 * @param key 要查找的键名
 * @return const char* 值的起始位置，找不到返回 nullptr
 * 
 * 功能说明：
 * - 在 JSON 字符串中查找指定 key 对应的值
 * - 返回值的起始位置（跳过冒号和空白字符）
 * 
 * 处理流程：
 * 1. 构造查找模式："key"
 * 2. 在 JSON 中查找该模式
 * 3. 从 key 后查找冒号
 * 4. 跳过冒号后的空白字符（空格、制表符、换行）
 * 5. 返回值的起始位置
 * 
 * 示例：
 * JSON: {"name":"John","age":30}
 * key: "age"
 * 返回：指向 "30" 的指针
 * 
 * 用途：
 * - JSON 解析的辅助函数
 * - 配合 json_extract_* 系列函数使用
 * 
 * 注意：
 * - 不处理嵌套 JSON
 * - 不验证 JSON 格式
 */
static const char *find_json_value_start(const char *json, const char *key)
{
    // 参数有效性检查
    if (json == nullptr || key == nullptr) {
        return nullptr;
    }

    // 步骤 1: 构造查找模式
    char pattern[64] = {};
    std::snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    
    // 步骤 2: 查找 key 的位置
    const char *keyPos = std::strstr(json, pattern);
    if (keyPos == nullptr) {
        return nullptr;  // 未找到 key
    }

    // 步骤 3: 查找冒号
    const char *colonPos = std::strchr(keyPos + std::strlen(pattern), ':');
    if (colonPos == nullptr) {
        return nullptr;  // 格式错误
    }

    // 步骤 4: 跳过空白字符
    const char *valuePos = colonPos + 1;
    while (*valuePos == ' ' || *valuePos == '\t' || *valuePos == '\r' || *valuePos == '\n') {
        ++valuePos;
    }
    
    // 步骤 5: 返回值的起始位置
    return valuePos;
}

/**
 * @brief 解析十六进制字符
 * 
 * @param c 输入字符（'0'-'9'、'a'-'f'、'A'-'F'）
 * @return int 对应的数值（0-15），无效字符返回 -1
 * 
 * 功能说明：
 * - 将单个十六进制字符转换为对应的数值
 * - 支持大小写字母
 * 
 * 映射关系：
 * - '0'-'9' → 0-9
 * - 'a'-'f' → 10-15
 * - 'A'-'F' → 10-15
 * - 其他字符 → -1
 * 
 * 用途：
 * - JSON 转义序列解析（\uXXXX）
 * - Unicode 码点解析
 * 
 * 示例：
 * json_hex_nibble('0') → 0
 * json_hex_nibble('A') → 10
 * json_hex_nibble('f') → 15
 * json_hex_nibble('G') → -1
 */
static int json_hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;  // 无效字符
}

/**
 * @brief 从 JSON 中提取字符串值
 * 
 * @param json JSON 字符串
 * @param key 要提取的键名
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @return true 提取成功
 * @return false 提取失败（key 不存在、格式错误、缓冲区太小）
 * 
 * 功能说明：
 * - 从 JSON 字符串中提取指定 key 的字符串值
 * - 处理 JSON 转义字符（\" \\ \b \f \n \r \t \uXXXX）
 * - 支持 Unicode 码点转换（\uXXXX → UTF-8）
 * 
 * 处理流程：
 * 1. 参数有效性检查（buffer 非空、buffer_size>0）
 * 2. 查找 key 对应的值起始位置
 * 3. 验证值是否为字符串（以 " 开头）
 * 4. 逐字符解析：
 *    - 普通字符：直接复制到缓冲区
 *    - 转义字符（\）：根据后续字符转换
 *      - \" → "
 *      - \\ → \
 *      - \/ → /
 *      - \b → 退格
 *      - \f → 换页
 *      - \n → 换行
 *      - \r → 回车
 *      - \t → 制表符
 *      - \uXXXX → Unicode 码点 → UTF-8 编码
 *    - 遇到结束引号 → 返回 true
 * 5. 缓冲区满或字符串结束 → 返回结果
 * 
 * Unicode 转换规则：
 * - 0x0000-0x007F: 单字节 UTF-8 (0xxxxxxx)
 * - 0x0080-0x07FF: 双字节 UTF-8 (110xxxxx 10xxxxxx)
 * - 0x0800-0xFFFF: 三字节 UTF-8 (1110xxxx 10xxxxxx 10xxxxxx)
 * 
 * 用途：
 * - 解析 BLE 接收的 JSON 配置命令
 * - 解析设备 ID、序列号、WiFi 凭证等字符串字段
 * 
 * 示例：
 * JSON: {"name":"John\\nDoe","age":"30"}
 * key: "name"
 * buffer: "John\nDoe"（\n 被转换为换行符）
 * 
 * 注意：
 * - 缓冲区会预留 1 字节存放 '\0' 终止符
 * - 如果解析失败，缓冲区内容不确定
 * - Unicode 转换只支持基本多文种平面（BMP）
 */
static bool json_extract_string_value(const char *json,
                                      const char *key,
                                      char *buffer,
                                      size_t buffer_size)
{
    // 步骤 1: 参数有效性检查
    if (buffer == nullptr || buffer_size == 0U) {
        return false;
    }

    // 步骤 2: 查找 key 对应的值起始位置
    const char *valuePos = find_json_value_start(json, key);
    if (valuePos == nullptr || *valuePos != '"') {
        return false;  // key 不存在或值不是字符串
    }

    // 步骤 3: 跳过起始引号
    ++valuePos;
    
    // 步骤 4: 逐字符解析
    size_t outIndex = 0U;
    while (*valuePos != '\0' && outIndex < (buffer_size - 1U)) {
        if (*valuePos == '"') {
            buffer[outIndex] = '\0';  // 遇到结束引号
            return true;
        }

        if (*valuePos != '\\') {
            // 普通字符：直接复制
            buffer[outIndex++] = *valuePos++;
            continue;
        }

        // 转义字符处理
        ++valuePos;
        if (*valuePos == '\0') {
            break;
        }

        switch (*valuePos) {
        case '"':  // 转义引号
        case '\\': // 转义反斜杠
        case '/':  // 转义斜杠
            buffer[outIndex++] = *valuePos++;
            break;
        case 'b':  // 退格
            buffer[outIndex++] = '\b';
            ++valuePos;
            break;
        case 'f':  // 换页
            buffer[outIndex++] = '\f';
            ++valuePos;
            break;
        case 'n':  // 换行
            buffer[outIndex++] = '\n';
            ++valuePos;
            break;
        case 'r':  // 回车
            buffer[outIndex++] = '\r';
            ++valuePos;
            break;
        case 't':  // 制表符
            buffer[outIndex++] = '\t';
            ++valuePos;
            break;
        case 'u': {  // Unicode 码点
            const int h0 = json_hex_nibble(valuePos[1]);
            const int h1 = json_hex_nibble(valuePos[2]);
            const int h2 = json_hex_nibble(valuePos[3]);
            const int h3 = json_hex_nibble(valuePos[4]);
            if (h0 < 0 || h1 < 0 || h2 < 0 || h3 < 0) {
                buffer[outIndex] = '\0';
                return false;  // 无效的十六进制字符
            }
            const unsigned codepoint = (unsigned)((h0 << 12) | (h1 << 8) | (h2 << 4) | h3);
            
            // 根据码点范围转换为 UTF-8 编码
            if (codepoint <= 0x7FU) {
                // 单字节 UTF-8
                buffer[outIndex++] = static_cast<char>(codepoint);
            } else if (codepoint <= 0x7FFU && outIndex + 1U < buffer_size - 1U) {
                // 双字节 UTF-8
                buffer[outIndex++] = static_cast<char>(0xC0U | (codepoint >> 6));
                buffer[outIndex++] = static_cast<char>(0x80U | (codepoint & 0x3FU));
            } else if (outIndex + 2U < buffer_size - 1U) {
                // 三字节 UTF-8
                buffer[outIndex++] = static_cast<char>(0xE0U | (codepoint >> 12));
                buffer[outIndex++] = static_cast<char>(0x80U | ((codepoint >> 6) & 0x3FU));
                buffer[outIndex++] = static_cast<char>(0x80U | (codepoint & 0x3FU));
            }
            valuePos += 5;  // 跳过 \uXXXX
            break;
        }
        default:  // 其他转义字符：原样复制
            buffer[outIndex++] = *valuePos++;
            break;
        }
    }
    
    buffer[outIndex] = '\0';  // 添加终止符
    return false;  // 未找到结束引号
}

/**
 * @brief 从 JSON 中提取 uint16 数值
 * 
 * @param json JSON 字符串
 * @param key 要提取的键名
 * @param value 输出值指针
 * @return true 提取成功
 * @return false 提取失败（key 不存在、格式错误、超出范围）
 * 
 * 功能说明：
 * - 从 JSON 字符串中提取指定 key 的 uint16 数值
 * - 范围检查：0-65535（0-0xFFFF）
 * 
 * 处理流程：
 * 1. 参数有效性检查（value 非空）
 * 2. 查找 key 对应的值起始位置
 * 3. 使用 strtoul 解析数值
 * 4. 范围检查（<= 65535）
 * 5. 转换为 uint16_t 并返回
 * 
 * 用途：
 * - 解析设备 ID、端口号等 16 位整数字段
 * 
 * 示例：
 * JSON: {"deviceId":1234,"port":8080}
 * key: "deviceId"
 * value: 1234
 * 
 * 注意：
 * - 不支持负数（无符号类型）
 * - 超过 65535 会返回 false
 */
static bool json_extract_uint16_value(const char *json, const char *key, uint16_t *value)
{
    // 步骤 1: 参数有效性检查
    if (value == nullptr) {
        return false;
    }

    // 步骤 2: 查找 key 对应的值起始位置
    const char *valuePos = find_json_value_start(json, key);
    if (valuePos == nullptr) {
        return false;  // key 不存在
    }

    // 步骤 3: 解析数值
    char *endPtr = nullptr;
    const unsigned long parsed = std::strtoul(valuePos, &endPtr, 10);
    
    // 步骤 4: 范围检查
    if (endPtr == valuePos || parsed > 65535UL) {
        return false;  // 解析失败或超出范围
    }

    // 步骤 5: 转换并返回
    *value = static_cast<uint16_t>(parsed);
    return true;
}

/**
 * @brief 从 JSON 中提取 uint32 数值
 * 
 * @param json JSON 字符串
 * @param key 要提取的键名
 * @param value 输出值指针
 * @return true 提取成功
 * @return false 提取失败（key 不存在、格式错误、超出范围）
 * 
 * 功能说明：
 * - 从 JSON 字符串中提取指定 key 的 uint32 数值
 * - 范围检查：0-4294967295（0-0xFFFFFFFF）
 * 
 * 处理流程：
 * 1. 参数有效性检查（value 非空）
 * 2. 查找 key 对应的值起始位置
 * 3. 使用 strtoul 解析数值
 * 4. 范围检查（<= 0xFFFFFFFF）
 * 5. 转换为 uint32_t 并返回
 * 
 * 用途：
 * - 解析时间戳、计数器等 32 位整数字段
 * 
 * 示例：
 * JSON: {"timestamp":1699876543,"count":42}
 * key: "timestamp"
 * value: 1699876543
 * 
 * 注意：
 * - 不支持负数（无符号类型）
 * - 超过 0xFFFFFFFF 会返回 false
 */
static bool json_extract_uint32_value(const char *json, const char *key, uint32_t *value)
{
    // 步骤 1: 参数有效性检查
    if (value == nullptr) {
        return false;
    }

    // 步骤 2: 查找 key 对应的值起始位置
    const char *valuePos = find_json_value_start(json, key);
    if (valuePos == nullptr) {
        return false;  // key 不存在
    }

    // 步骤 3: 解析数值
    char *endPtr = nullptr;
    const unsigned long parsed = std::strtoul(valuePos, &endPtr, 10);
    
    // 步骤 4: 范围检查
    if (endPtr == valuePos || parsed > 0xFFFFFFFFUL) {
        return false;  // 解析失败或超出范围
    }

    // 步骤 5: 转换并返回
    *value = static_cast<uint32_t>(parsed);
    return true;
}

/**
 * @brief 从 JSON 中提取 uint64 数值
 * 
 * @param json JSON 字符串
 * @param key 要提取的键名
 * @param value 输出值指针
 * @return true 提取成功
 * @return false 提取失败（key 不存在、格式错误）
 * 
 * 功能说明：
 * - 从 JSON 字符串中提取指定 key 的 uint64 数值
 * - 支持完整 64 位范围（0-0xFFFFFFFFFFFFFFFF）
 * 
 * 处理流程：
 * 1. 参数有效性检查（value 非空）
 * 2. 查找 key 对应的值起始位置
 * 3. 使用 strtoull 解析数值（64 位版本）
 * 4. 检查解析是否成功
 * 5. 转换为 uint64_t 并返回
 * 
 * 用途：
 * - 解析设备序列号、64 位时间戳等字段
 * 
 * 示例：
 * JSON: {"deviceSn":12345678901234567890}
 * key: "deviceSn"
 * value: 12345678901234567890
 * 
 * 注意：
 * - 不支持负数（无符号类型）
 * - 不进行范围检查（完整 64 位）
 * - 使用 strtoull 而不是 strtoul
 */
static bool json_extract_uint64_value(const char *json, const char *key, uint64_t *value)
{
    // 步骤 1: 参数有效性检查
    if (value == nullptr) {
        return false;
    }

    // 步骤 2: 查找 key 对应的值起始位置
    const char *valuePos = find_json_value_start(json, key);
    if (valuePos == nullptr) {
        return false;  // key 不存在
    }

    // 步骤 3: 解析数值（64 位版本）
    char *endPtr = nullptr;
    const unsigned long long parsed = std::strtoull(valuePos, &endPtr, 10);
    
    // 步骤 4: 检查解析是否成功
    if (endPtr == valuePos) {
        return false;  // 解析失败
    }

    // 步骤 5: 转换并返回
    *value = static_cast<uint64_t>(parsed);
    return true;
}

/**
 * @brief JSON 字符串转义
 * 
 * @param input 输入字符串
 * @param output 输出缓冲区（转义后的字符串）
 * @param output_size 输出缓冲区大小
 * 
 * 功能说明：
 * - 将普通字符串转义为 JSON 格式
 * - 处理特殊字符和控制字符
 * 
 * 转义规则：
 * - " → \"
 * - \\ → \\
 * - 退格 → \b
 * - 换页 → \f
 * - 换行 → \n
 * - 回车 → \r
 * - 制表符 → \t
 * - 其他控制字符（< 0x20）→ \uXXXX
 * - 普通字符 → 原样输出
 * 
 * 处理流程：
 * 1. 参数有效性检查
 * 2. 初始化输出缓冲区
 * 3. 逐字符处理：
 *    - 特殊字符：使用转义序列
 *    - 控制字符：使用 \uXXXX 格式
 *    - 普通字符：直接复制
 * 4. 添加终止符
 * 
 * 用途：
 * - 构造 JSON 响应数据
 * - 将用户输入转换为安全的 JSON 字符串
 * 
 * 示例：
 * input: "Hello\nWorld\"Test\""
 * output: "Hello\\nWorld\\\"Test\\\""
 * 
 * 注意：
 * - 输出缓冲区会预留 1 字节存放 '\0'
 * - 控制字符使用 6 字节的 \uXXXX 格式
 * - 不处理 UTF-8 多字节字符（按单字节处理）
 */
static void json_escape_string(const char *input, char *output, size_t output_size)
{
    // 步骤 1: 参数有效性检查
    if (output == nullptr || output_size == 0U) {
        return;
    }

    // 步骤 2: 初始化输出缓冲区
    output[0] = '\0';
    if (input == nullptr) {
        return;
    }

    // 步骤 3: 逐字符处理
    size_t used = 0U;
    for (size_t i = 0U; input[i] != '\0' && used < output_size - 1U; ++i) {
        const unsigned char c = static_cast<unsigned char>(input[i]);
        const char *escaped = nullptr;
        
        // 检查是否需要转义
        switch (c) {
        case '\"':  // 引号
            escaped = "\\\"";
            break;
        case '\\':  // 反斜杠
            escaped = "\\\\";
            break;
        case '\b':  // 退格
            escaped = "\\b";
            break;
        case '\f':  // 换页
            escaped = "\\f";
            break;
        case '\n':  // 换行
            escaped = "\\n";
            break;
        case '\r':  // 回车
            escaped = "\\r";
            break;
        case '\t':  // 制表符
            escaped = "\\t";
            break;
        default:
            break;
        }

        if (escaped != nullptr) {
            // 特殊字符：使用转义序列
            const size_t len = std::strlen(escaped);
            if (used + len >= output_size) {
                break;  // 缓冲区不足
            }
            std::memcpy(output + used, escaped, len);
            used += len;
            continue;
        }

        if (c < 0x20U) {
            // 控制字符：使用 \uXXXX 格式
            if (used + 6U >= output_size) {
                break;  // 缓冲区不足
            }
            std::snprintf(output + used, output_size - used, "\\u%04X", (unsigned)c);
            used += 6U;
            continue;
        }

        // 普通字符：直接复制
        output[used++] = static_cast<char>(c);
        output[used] = '\0';
    }
}

/**
 * @brief 计算 Modbus CRC16 校验码
 * 
 * @param data 输入数据字符串
 * @return uint16_t CRC16 校验码
 * 
 * 功能说明：
 * - 计算 Modbus 协议的 CRC16 校验码
 * - 用于数据完整性验证
 * 
 * 算法参数：
 * - 多项式：0xA001（反向 0x8005）
 * - 初始值：0xFFFF
 * - 输入反转：是
 * - 输出反转：是
 * 
 * 处理流程：
 * 1. 初始化 CRC = 0xFFFF
 * 2. 对每个字节：
 *    - CRC ^= 当前字节
 *    - 处理 8 位：
 *      - 如果最低位为 1：右移 1 位，异或 0xA001
 *      - 如果最低位为 0：右移 1 位
 * 3. 返回最终 CRC 值
 * 
 * 用途：
 * - 雷达数据校验
 * - BLE 数据传输完整性验证
 * 
 * 示例：
 * calculate_modbus_crc16("75.5|18.2|1|1|1|5|3") → 0xA1B2（示例值）
 */
static uint16_t calculate_modbus_crc16(const char *data)
{
    uint16_t crc = 0xFFFFU;  // 初始值
    if (data == nullptr) {
        return crc;
    }

    // 逐字节计算 CRC
    while (*data != '\0') {
        crc ^= static_cast<uint8_t>(*data++);  // 异或当前字节
        for (uint8_t i = 0U; i < 8U; ++i) {
            if ((crc & 0x0001U) != 0U) {
                crc >>= 1U;
                crc ^= 0xA001U;  // 多项式
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/**
 * @brief 检查连续传感器数据是否发生变化
 * 
 * @param currentData 当前传感器数据
 * @return true 数据发生变化
 * @return false 数据未变化
 * 
 * 功能说明：
 * - 比较当前数据与上次发送的数据
 * - 检测关键字段是否变化
 * 
 * 检测字段（7 个）：
 * 1. presence: 人体存在状态
 * 2. motion: 运动能量
 * 3. sleep_state: 睡眠状态
 * 4. heart_rate: 心率
 * 5. breath_rate: 呼吸率
 * 6. heartbeat_waveform: 心跳波形
 * 7. breathing_waveform: 呼吸波形
 * 
 * 处理流程：
 * 1. 检查是否有上次的数据
 *    - 没有 → 返回 true（首次发送）
 * 2. 逐字段比较
 *    - 任一字段变化 → 返回 true
 *    - 所有字段相同 → 返回 false
 * 
 * 用途：
 * - 决定是否发送连续数据包
 * - 避免发送重复数据
 * 
 * 注意：
 * - 只检测核心生命体征数据
 * - 不检测位置、体动等辅助数据
 */
static bool continuous_sensor_data_changed(const SensorData &currentData)
{
    // 首次发送：没有上次的数据，直接返回 true
    if (!gHasLastContinuousData) {
        return true;
    }

    // 逐字段比较（任一变化即返回 true）
    return currentData.presence != gLastContinuousData.presence ||
           currentData.motion != gLastContinuousData.motion ||
           currentData.sleep_state != gLastContinuousData.sleep_state ||
           currentData.heart_rate != gLastContinuousData.heart_rate ||
           currentData.breath_rate != gLastContinuousData.breath_rate ||
           currentData.heartbeat_waveform != gLastContinuousData.heartbeat_waveform ||
           currentData.breathing_waveform != gLastContinuousData.breathing_waveform;
}

/**
 * @brief 检查 InfluxDB 日常快照是否发生变化
 * 
 * @param currentData 当前雷达报告快照
 * @return true 数据发生变化
 * @return false 数据未变化
 * 
 * 功能说明：
 * - 比较当前快照与上次发送的快照
 * - 检测 InfluxDB 上报所需的所有字段
 * 
 * 检测字段（15 个）：
 * 1. presence: 人体存在
 * 2. motion: 运动能量
 * 3. distance: 距离
 * 4. sleep_state: 睡眠状态
 * 5. pos_x/y/z: 人体位置坐标
 * 6. heartbeat_waveform: 心跳波形
 * 7. breathing_waveform: 呼吸波形
 * 8. abnormal_state: 异常状态
 * 9. bed_status: 床状态
 * 10. struggle_alert: 挣扎告警
 * 11. no_one_alert: 无人告警
 * 12. heart_rate: 心率
 * 13. breath_rate: 呼吸率
 * 
 * 处理流程：
 * 1. 检查是否有上次的快照
 *    - 没有 → 返回 true（首次发送）
 * 2. 逐字段比较
 *    - 任一字段变化 → 返回 true
 *    - 所有字段相同 → 返回 false
 * 
 * 用途：
 * - 决定是否上报 InfluxDB 日常数据
 * - 减少重复数据传输
 * 
 * 注意：
 * - 比 continuous_sensor_data_changed 检测更多字段
 * - 包含位置、告警等辅助数据
 */
static bool influx_daily_snapshot_changed(const RadarReportSnapshot &currentData)
{
    // 首次发送：没有上次的快照，直接返回 true
    if (!gHasLastInfluxDailySnapshot) {
        return true;
    }

    // 逐字段比较（15 个字段，任一变化即返回 true）
    return currentData.presence != gLastInfluxDailySnapshot.presence ||
           currentData.motion != gLastInfluxDailySnapshot.motion ||
           currentData.distance != gLastInfluxDailySnapshot.distance ||
           currentData.sleep_state != gLastInfluxDailySnapshot.sleep_state ||
           currentData.pos_x != gLastInfluxDailySnapshot.pos_x ||
           currentData.pos_y != gLastInfluxDailySnapshot.pos_y ||
           currentData.pos_z != gLastInfluxDailySnapshot.pos_z ||
           currentData.heartbeat_waveform != gLastInfluxDailySnapshot.heartbeat_waveform ||
           currentData.breathing_waveform != gLastInfluxDailySnapshot.breathing_waveform ||
           currentData.abnormal_state != gLastInfluxDailySnapshot.abnormal_state ||
           currentData.bed_status != gLastInfluxDailySnapshot.bed_status ||
           currentData.struggle_alert != gLastInfluxDailySnapshot.struggle_alert ||
           currentData.no_one_alert != gLastInfluxDailySnapshot.no_one_alert ||
           currentData.heart_rate != gLastInfluxDailySnapshot.heart_rate ||
           currentData.breath_rate != gLastInfluxDailySnapshot.breath_rate;
}

// 连续雷达字符串分片发送函数（支持小程序的 [i/n] 分片协议）
static bool send_pipe_frame_chunked_to_ble(const char *text)
{
    if (text == nullptr || text[0] == '\0') {
        return false;
    }

    const size_t textLen = std::strlen(text);
    static constexpr size_t kChunkPayloadLen = 15; // 每片正文最多 15 字节

    const size_t totalChunks =
        (textLen + kChunkPayloadLen - 1U) / kChunkPayloadLen;

    // 如果不超过 1 片，直接发送
    if (totalChunks <= 1U) {
        return ble_manager_send_text(text);
    }

    bool allSent = true;
    size_t offset = 0U;

    for (size_t index = 0; index < totalChunks; ++index) {
        const size_t remain = textLen - offset;
        const size_t pieceLen =
            remain < kChunkPayloadLen ? remain : kChunkPayloadLen;

        char chunk[32] = {};
        std::snprintf(chunk,
                      sizeof(chunk),
                      "[%u/%u]%.*s",
                      (unsigned)(index + 1U),
                      (unsigned)totalChunks,
                      (int)pieceLen,
                      text + offset);

        ESP_LOGI(TAG, "BLE chunk send [%u/%u]: %s",
                 (unsigned)(index + 1U), (unsigned)totalChunks, chunk);

        if (!ble_manager_send_text(chunk)) {
            allSent = false;
            break;
        }

        offset += pieceLen;

        // 片间延时 10ms，避免 BLE 拥塞
        if ((index + 1U) < totalChunks) {
            radar_sleep_ms(10);
        }
    }

    return allSent;
}

static bool send_continuous_radar_packet_to_ble(void)
{
    const SensorData currentData = radar_manager_get_sensor_data();
    // 不再因"数据没变化"就拦截，保持稳定发送
    // const bool changed = continuous_sensor_data_changed(currentData);

    char radarDataCore[128] = {};
    if (currentData.presence > 0U) {
        std::snprintf(radarDataCore,
                      sizeof(radarDataCore),
                      "%.1f|%.1f|%d|%d|%u|%u|%u",
                      currentData.heart_rate,
                      currentData.breath_rate,
                      currentData.heartbeat_waveform,
                      currentData.breathing_waveform,
                      (unsigned)currentData.presence,
                      (unsigned)currentData.motion,
                      (unsigned)currentData.sleep_state);
    } else {
        std::snprintf(radarDataCore, sizeof(radarDataCore), "0.0|0.0|0|0|0|0|0");
    }

    char payload[160] = {};
    std::snprintf(payload,
                  sizeof(payload),
                  "%s|%x",
                  radarDataCore,
                  (unsigned)calculate_modbus_crc16(radarDataCore));

    // 使用分片发送函数，适配小程序的 [i/n] 协议
    const bool sent = send_pipe_frame_chunked_to_ble(payload);
    gLastContinuousData = currentData;
    gHasLastContinuousData = true;
    return sent;
}

static bool send_radar_data_snapshot_to_ble(void)
{
    const DeviceIdentity identity = device_identity_get();
    const SensorData currentData = radar_manager_get_sensor_data();
    const uint64_t nowMs = radar_now_ms();

    char payload[1024] = {};
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"type\":\"radarData\",\"success\":true,\"deviceId\":%u,"
                  "\"timestamp\":%" PRIu64 ",\"presence\":%u,\"heartRate\":%.1f,"
                  "\"breathRate\":%.1f,\"motion\":%u,\"heartbeatWaveform\":%d,"
                  "\"breathingWaveform\":%d,\"distance\":%u,\"bodyMovement\":%u,"
                  "\"breathStatus\":%u,\"sleepState\":%u,\"sleepTime\":%u,"
                  "\"sleepScore\":%u,\"avgHeartRate\":%u,\"avgBreathRate\":%u,"
                  "\"turnCount\":%u,\"largeMoveRatio\":%u,\"smallMoveRatio\":%u,"
                  "\"sleepQualityGrade\":%u,\"totalSleepDuration\":%u,"
                  "\"awakeDurationRatio\":%u,\"lightSleepRatio\":%u,\"deepSleepRatio\":%u,"
                  "\"outOfBedDuration\":%u,\"apneaCount\":%u,\"struggleAlert\":%u,"
                  "\"noOneAlert\":%u,\"awakeDuration\":%u,\"lightSleepDuration\":%u,"
                  "\"deepSleepDuration\":%u,\"humanPositionX\":%d,\"humanPositionY\":%d,"
                  "\"humanPositionZ\":%d,\"abnormalState\":%u,\"heartValid\":%u,"
                  "\"breathValid\":%u}",
                  (unsigned)identity.device_id,
                  nowMs,
                  (unsigned)currentData.presence,
                  currentData.heart_rate,
                  currentData.breath_rate,
                  (unsigned)currentData.motion,
                  currentData.heartbeat_waveform,
                  currentData.breathing_waveform,
                  (unsigned)currentData.distance,
                  (unsigned)currentData.body_movement,
                  (unsigned)currentData.breath_status,
                  (unsigned)currentData.sleep_state,
                  (unsigned)currentData.sleep_time,
                  (unsigned)currentData.sleep_score,
                  (unsigned)currentData.avg_heart_rate,
                  (unsigned)currentData.avg_breath_rate,
                  (unsigned)currentData.turn_count,
                  (unsigned)currentData.large_move_ratio,
                  (unsigned)currentData.small_move_ratio,
                  (unsigned)currentData.sleep_grade,
                  (unsigned)currentData.sleep_total_time,
                  (unsigned)currentData.awake_ratio,
                  (unsigned)currentData.light_sleep_ratio,
                  (unsigned)currentData.deep_sleep_ratio,
                  (unsigned)currentData.bed_Out_Time,
                  (unsigned)currentData.apnea_count,
                  (unsigned)currentData.struggle_alert,
                  (unsigned)currentData.no_one_alert,
                  (unsigned)currentData.awake_time,
                  (unsigned)currentData.light_sleep_time,
                  (unsigned)currentData.deep_sleep_time,
                  (int)currentData.pos_x,
                  (int)currentData.pos_y,
                  (int)currentData.pos_z,
                  (unsigned)currentData.abnormal_state,
                  (unsigned)currentData.heart_valid,
                  (unsigned)currentData.breath_valid);
    return ble_manager_send_text(payload);
}

static NetworkStatus to_system_network_status(TaskNetworkStatus status)
{
    switch (status) {
    case NET_CONNECTED:
        return NETWORK_STATUS_CONNECTED;
    case NET_CONNECTING:
        return NETWORK_STATUS_CONNECTING;
    case NET_DISCONNECTED:
        return NETWORK_STATUS_DISCONNECTED;
    case NET_INITIAL:
    default:
        return NETWORK_STATUS_INITIAL;
    }
}

void setNetworkStatus(TaskNetworkStatus status)
{
    currentNetworkStatus = status;
    if (status == NET_CONNECTED) {
        gBreatheValue = APP_LED_BREATHE_MIN;
        gBreatheIncreasing = true;
    }
    system_state_set_network_status(to_system_network_status(status));
}

void WiFiEvent(TaskWiFiEvent event)
{
    switch (event) {
    case TASK_WIFI_EVENT_STA_START:
        setNetworkStatus(NET_INITIAL);
        break;
    case TASK_WIFI_EVENT_STA_CONNECTED:
        setNetworkStatus(NET_CONNECTING);
        break;
    case TASK_WIFI_EVENT_STA_GOT_IP:
        setNetworkStatus(NET_CONNECTED);
        break;
    case TASK_WIFI_EVENT_STA_DISCONNECTED:
    case TASK_WIFI_EVENT_STA_STOP:
        setNetworkStatus(NET_DISCONNECTED);
        break;
    default:
        break;
    }
}

static void configure_boot_button_gpio(void)
{
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << APP_BOOT_BUTTON_PIN);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&config);
}

static void configure_led_gpio(void)
{
    ledc_timer_config_t timerConfig = {};
    timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
    timerConfig.duty_resolution = LEDC_TIMER_8_BIT;
    timerConfig.timer_num = LEDC_TIMER_0;
    timerConfig.freq_hz = 5000;
    timerConfig.clk_cfg = LEDC_AUTO_CLK;
    timerConfig.deconfigure = false;
    ledc_timer_config(&timerConfig);

    ledc_channel_config_t channelConfig = {};
    channelConfig.gpio_num = APP_NETWORK_LED_PIN;
    channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConfig.channel = LEDC_CHANNEL_0;
    channelConfig.intr_type = LEDC_INTR_DISABLE;
    channelConfig.timer_sel = LEDC_TIMER_0;
    channelConfig.duty = 0;
    channelConfig.hpoint = 0;
    ledc_channel_config(&channelConfig);

    gpio_config_t clearLedConfig = {};
    clearLedConfig.pin_bit_mask = (1ULL << APP_CONFIG_CLEAR_PIN);
    clearLedConfig.mode = GPIO_MODE_OUTPUT;
    clearLedConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    clearLedConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    clearLedConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&clearLedConfig);
    gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 0);
}

static void set_network_led_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void clearStoredConfig(void)
{
    ESP_LOGW(TAG, "clearing stored wifi/device configuration");
    (void)wifi_manager_clear_credentials();
    (void)device_identity_reset_device_id_default();
    (void)mqtt_manager_refresh_identity();
    setNetworkStatus(NET_DISCONNECTED);
    sendStatusToBLE();
}

void sendStatusToBLE(void)
{
    const DeviceIdentity identity = device_identity_get();
    const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();
    const BleManagerSnapshot bleSnapshot = ble_manager_get_snapshot();
    char escapedSsid[80] = {};
    char escapedBleName[80] = {};
    json_escape_string(wifiSnapshot.ssid[0] != '\0' ? wifiSnapshot.ssid : "",
                       escapedSsid,
                       sizeof(escapedSsid));
    json_escape_string(bleSnapshot.device_name[0] != '\0' ? bleSnapshot.device_name : "",
                       escapedBleName,
                       sizeof(escapedBleName));

    char payload[512] = {};
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"type\":\"status\",\"deviceId\":%u,\"deviceSn\":%" PRIu64 ","
                  "\"wifiConfigured\":%s,\"wifiConnected\":%s,\"wifiSsid\":\"%s\","
                  "\"wifiIP\":\"%s\",\"ipAddress\":\"%s\","
                  "\"bleReady\":%u,\"bleAdvertising\":%u,\"bleConnected\":%u,"
                  "\"bleNotify\":%u,\"bleName\":\"%s\"}",
                  (unsigned)identity.device_id,
                  identity.device_sn,
                  wifiSnapshot.has_credentials ? "true" : "false",
                  wifiSnapshot.connected ? "true" : "false",
                  escapedSsid,
                  wifiSnapshot.ip,
                  wifiSnapshot.ip,
                  (unsigned)bleSnapshot.initialized,
                  (unsigned)bleSnapshot.advertising,
                  (unsigned)bleSnapshot.connected,
                  (unsigned)bleSnapshot.notify_enabled,
                  escapedBleName);

    const bool sent = ble_manager_send_text(payload);
    ESP_LOGI(TAG, "BLE status payload %s: %s", sent ? "sent" : "prepared", payload);
}

void processBLEConfig(void)
{
    static char receivedText[256] = {};
    if (!ble_manager_consume_received_text(receivedText, sizeof(receivedText))) {
        return;
    }

    ESP_LOGI(TAG, "processing BLE payload: %s", receivedText);

    static char command[32] = {};
    if (!json_extract_string_value(receivedText, "command", command, sizeof(command))) {
        (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"missing command\"}");
        return;
    }

    if (std::strcmp(command, "queryStatus") == 0) {
        const DeviceIdentity identity = device_identity_get();
        const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();
        static char escapedSsid[80] = {};
        json_escape_string(wifiSnapshot.ssid[0] != '\0' ? wifiSnapshot.ssid : "",
                           escapedSsid,
                           sizeof(escapedSsid));
        static char payload[512] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"deviceStatus\",\"success\":true,\"deviceId\":%u,"
                      "\"wifiConfigured\":%s,\"wifiConnected\":%s,\"wifiSsid\":\"%s\","
                      "\"wifiIP\":\"%s\",\"ipAddress\":\"%s\"}",
                      (unsigned)identity.device_id,
                      wifiSnapshot.has_credentials ? "true" : "false",
                      wifiSnapshot.connected ? "true" : "false",
                      escapedSsid,
                      wifiSnapshot.ip,
                      wifiSnapshot.ip);
        (void)ble_manager_send_text(payload);
        return;
    }

    if (std::strcmp(command, "setDeviceId") == 0) {
        uint16_t newDeviceId = 0U;
        if (!json_extract_uint16_value(receivedText, "newDeviceId", &newDeviceId)) {
            (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"missing newDeviceId\"}");
            return;
        }

        if (newDeviceId < 1000U || newDeviceId > 1999U) {
            (void)ble_manager_send_text(
                "{\"type\":\"error\",\"message\":\"deviceId out of range 1000-1999\"}");
            return;
        }

        if (!device_identity_set_device_id(newDeviceId)) {
            (void)ble_manager_send_text(
                "{\"type\":\"setDeviceIdResult\",\"success\":false,\"message\":\"persist failed\"}");
            return;
        }

        static char payload[160] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"setDeviceIdResult\",\"success\":true,"
                      "\"message\":\"\\u8bbe\\u5907ID\\u8bbe\\u7f6e\\u6210\\u529f\","
                      "\"newDeviceId\":%u}",
                      (unsigned)newDeviceId);
        (void)ble_manager_send_text(payload);
        sendStatusToBLE();
        return;
    }

    if (std::strcmp(command, "setDeviceSn") == 0) {
        uint64_t newDeviceSn = 0ULL;
        if (!json_extract_uint64_value(receivedText, "newDeviceSn", &newDeviceSn)) {
            (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"missing newDeviceSn\"}");
            return;
        }

        if (!device_identity_set_device_sn(newDeviceSn)) {
            (void)ble_manager_send_text(
                "{\"type\":\"setDeviceSnResult\",\"success\":false,\"message\":\"persist failed\"}");
            return;
        }

        static char payload[192] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"setDeviceSnResult\",\"success\":true,\"newDeviceSn\":%" PRIu64 "}",
                      newDeviceSn);
        (void)ble_manager_refresh_advertising_data();
        (void)mqtt_manager_refresh_identity();
        (void)ble_manager_send_text(payload);
        sendStatusToBLE();
        return;
    }

    if (std::strcmp(command, "setWiFiConfig") == 0) {
        static char ssid[33] = {};
        static char password[65] = {};

        if (!json_extract_string_value(receivedText, "ssid", ssid, sizeof(ssid))) {
            (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"missing ssid\"}");
            return;
        }

        if (!json_extract_string_value(receivedText, "password", password, sizeof(password))) {
            password[0] = '\0';
        }

        const bool connected = wifi_manager_configure_and_connect(ssid, password);
        const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();
        static char escapedSsid[80] = {};
        static char escapedPassword[140] = {};
        json_escape_string(ssid, escapedSsid, sizeof(escapedSsid));
        json_escape_string(password, escapedPassword, sizeof(escapedPassword));
        static char payload[384] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"wifiConfigResult\",\"success\":%s,\"ssid\":\"%s\","
                      "\"message\":\"%s\",\"ip\":\"%s\",\"rssi\":%d}",
                      connected ? "true" : "false",
                      escapedSsid,
                      connected
                          ? "\\u0057\\u0069\\u0046\\u0069\\u914d\\u7f6e\\u6210\\u529f"
                          : "\\u0057\\u0069\\u0046\\u0069\\u914d\\u7f6e\\u5931\\u8d25\\uff0c\\u8bf7\\u68c0\\u67e5\\u5bc6\\u7801\\u662f\\u5426\\u6b63\\u786e",
                      wifiSnapshot.ip,
                      (int)wifiSnapshot.rssi);
        (void)ble_manager_send_text(payload);
        if (connected) {
            static char connectedPayload[512] = {};
            std::snprintf(connectedPayload,
                          sizeof(connectedPayload),
                          "{\"type\":\"wifiConnected\",\"success\":true,\"ssid\":\"%s\","
                          "\"password\":\"%s\",\"ip\":\"%s\",\"rssi\":%d}",
                          escapedSsid,
                          escapedPassword,
                          wifiSnapshot.ip,
                          (int)wifiSnapshot.rssi);
            (void)ble_manager_send_text(connectedPayload);
        }
        sendStatusToBLE();
        return;
    }

    if (std::strcmp(command, "echo") == 0) {
        static char content[128] = {};
        const bool hasContent =
            json_extract_string_value(receivedText, "content", content, sizeof(content));

        static char payload[384] = {};
        if (hasContent) {
            static char escapedContent[260] = {};
            json_escape_string(content, escapedContent, sizeof(escapedContent));
            std::snprintf(payload,
                          sizeof(payload),
                          "{\"type\":\"echoResponse\",\"originalContent\":\"%s\","
                          "\"receivedSuccessfully\":true}",
                          escapedContent);
        } else {
            std::snprintf(payload,
                          sizeof(payload),
                          "{\"type\":\"echoResponse\",\"receivedSuccessfully\":true,"
                          "\"message\":\"Echo command received\"}");
        }
        (void)ble_manager_send_text(payload);
        return;
    }

    if (std::strcmp(command, "queryRadarData") == 0) {
        (void)send_radar_data_snapshot_to_ble();
        return;
    }

    if (std::strcmp(command, "startContinuousSend") == 0) {
        uint32_t intervalMs = gContinuousSendIntervalMs;
        if (json_extract_uint32_value(receivedText, "interval", &intervalMs)) {
            if (intervalMs < 100U) {
                intervalMs = 100U;
            } else if (intervalMs > 10000U) {
                intervalMs = 10000U;
            }
            gContinuousSendIntervalMs = intervalMs;
        }

        gContinuousSendEnabled = true;
        gHasLastContinuousData = false;
        gLastContinuousCheckMs = 0U;

        static char payload[192] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"startContinuousSendResult\",\"success\":true,"
                      "\"message\":\"\\u5df2\\u542f\\u52a8\\u6301\\u7eed\\u53d1\\u9001\\u6a21\\u5f0f\","
                      "\"interval\":%u}",
                      (unsigned)gContinuousSendIntervalMs);
        (void)ble_manager_send_text(payload);

        // 新增：立即补发一帧实时数据
        gLastContinuousCheckMs = radar_now_ms();
        (void)send_continuous_radar_packet_to_ble();

        return;
    }

    if (std::strcmp(command, "stopContinuousSend") == 0) {
        gContinuousSendEnabled = false;
        static char payload[160] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"stopContinuousSendResult\",\"success\":true,"
                      "\"message\":\"\\u5df2\\u505c\\u6b62\\u6301\\u7eed\\u53d1\\u9001\\u6a21\\u5f0f\"}");
        (void)ble_manager_send_text(payload);
        return;
    }

    if (std::strcmp(command, "scanWiFi") == 0) {
        // 先减少到 8 个，避免栈溢出，稳定后再改回 16
        static WiFiScanRecord scanRecords[8] = {};
        const uint32_t count = wifi_manager_scan_networks(scanRecords, 8U);

        static char payload[3072] = {};
        size_t used = std::snprintf(payload,
                                    sizeof(payload),
                                    "{\"type\":\"scanWiFiResult\",\"success\":true,\"count\":%u,\"networks\":[",
                                    (unsigned)count);

        for (uint32_t i = 0; i < count && used < sizeof(payload); ++i) {
            const bool saved = wifi_manager_has_saved_network(scanRecords[i].ssid);
            static char escapedSsid[80] = {};
            static char escapedEncryption[40] = {};
            json_escape_string(scanRecords[i].ssid, escapedSsid, sizeof(escapedSsid));
            json_escape_string(scanRecords[i].encryption,
                               escapedEncryption,
                               sizeof(escapedEncryption));
            const int written = std::snprintf(payload + used,
                                              sizeof(payload) - used,
                                              "%s{\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%u,"
                                              "\"encryption\":\"%s\",\"saved\":%s}",
                                              i == 0U ? "" : ",",
                                              escapedSsid,
                                              (int)scanRecords[i].rssi,
                                              (unsigned)scanRecords[i].channel,
                                              escapedEncryption,
                                              saved ? "true" : "false");
            if (written <= 0) {
                break;
            }
            used += static_cast<size_t>(written);
        }

        if (used < sizeof(payload)) {
            std::snprintf(payload + used, sizeof(payload) - used, "]}");
        } else {
            payload[sizeof(payload) - 2U] = ']';
            payload[sizeof(payload) - 1U] = '\0';
        }

        (void)ble_manager_send_text(payload);
        return;
    }

    if (std::strcmp(command, "getSavedNetworks") == 0) {
        static WiFiSavedNetwork savedNetworks[10] = {};
        const uint32_t count = wifi_manager_get_saved_networks(savedNetworks, 10U);

        static char payload[1024] = {};
        size_t used = std::snprintf(payload,
                                    sizeof(payload),
                                    "{\"type\":\"savedNetworks\",\"success\":true,\"count\":%u,\"networks\":[",
                                    (unsigned)count);

        for (uint32_t i = 0; i < count && used < sizeof(payload); ++i) {
            static char escapedSsid[80] = {};
            json_escape_string(savedNetworks[i].ssid, escapedSsid, sizeof(escapedSsid));
            const int written = std::snprintf(payload + used,
                                              sizeof(payload) - used,
                                              "%s{\"ssid\":\"%s\"}",
                                              i == 0U ? "" : ",",
                                              escapedSsid);
            if (written <= 0) {
                break;
            }
            used += static_cast<size_t>(written);
        }

        if (used < sizeof(payload)) {
            std::snprintf(payload + used, sizeof(payload) - used, "]}");
        } else {
            payload[sizeof(payload) - 2U] = ']';
            payload[sizeof(payload) - 1U] = '\0';
        }

        (void)ble_manager_send_text(payload);
        return;
    }

    (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"unknown command\"}");
}

void tasks_manager_set_continuous_send(bool enabled, unsigned long interval_ms)
{
    uint32_t clampedInterval = static_cast<uint32_t>(interval_ms);
    if (clampedInterval < 100U) {
        clampedInterval = 100U;
    } else if (clampedInterval > 10000U) {
        clampedInterval = 10000U;
    }

    gContinuousSendEnabled = enabled;
    gContinuousSendIntervalMs = clampedInterval;
    gHasLastContinuousData = false;
    gLastContinuousCheckMs = 0U;
}

bool tasks_manager_get_continuous_send_enabled(void)
{
    return gContinuousSendEnabled;
}

unsigned long tasks_manager_get_continuous_send_interval_ms(void)
{
    return static_cast<unsigned long>(gContinuousSendIntervalMs);
}

// ============================================================================
// 任务函数实现
// ============================================================================
/**
 * @brief 启动按钮监控任务
 * 
 * @param parameter 任务参数（未使用）
 * 
 * 功能概述：
 * - 监控启动按钮的按下状态
 * - 长按 APP_CLEAR_CONFIG_DURATION_MS 毫秒后清除配置并重启
 * - 控制清除配置引脚的电平
 * 
 * 处理流程：
 * 1. 配置 GPIO（按钮输入、清除引脚输出）
 * 2. 循环检测按钮状态
 * 3. 检测到按下：记录时间，设置清除引脚高电平
 * 4. 检测到释放：取消清除标志，设置清除引脚低电平
 * 5. 长按超时：清除配置、关闭 LED、重启系统
 * 
 * 长按逻辑：
 * - 按下时间 >= APP_CLEAR_CONFIG_DURATION_MS：触发清除配置
 * - 按下时间 < APP_CLEAR_CONFIG_DURATION_MS：视为普通按下，不触发
 * 
 * 清除配置操作：
 * 1. 清除 WiFi 凭证
 * 2. 重置设备 ID
 * 3. 刷新 MQTT 身份
 * 4. 设置网络状态为断开
 * 5. 发送 BLE 状态更新
 * 6. 延时 1 秒后重启
 * 
 * 注意：
 * - 此任务优先级为 2（中高优先级）
 * - 使用看门狗防止任务阻塞
 * - 清除配置后无法撤销
 */
void bootButtonMonitorTask(void *parameter)
{
    (void)parameter;

    // 步骤 1: 配置 GPIO
    configure_boot_button_gpio();
    gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 0);

    uint64_t buttonPressStartMs = 0U;  // 按下开始时间
    bool buttonPressed = false;         // 按下标志

    ESP_LOGI(TAG, "boot button monitor task started");
    task_watchdog_register_current("boot_button_task");

    while (true) {
        task_watchdog_reset_current();  // 重置看门狗
        const int buttonLevel = gpio_get_level((gpio_num_t)APP_BOOT_BUTTON_PIN);

        // 步骤 2: 检测按下事件
        if (buttonLevel == 0 && !buttonPressed) {
            buttonPressed = true;
            buttonPressStartMs = radar_now_ms();
            gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 1);  // 设置清除引脚高电平
            ESP_LOGI(TAG, "boot button pressed, hold %u ms to clear config",
                     (unsigned)APP_CLEAR_CONFIG_DURATION_MS);
        } 
        // 步骤 3: 检测释放事件
        else if (buttonLevel != 0 && buttonPressed) {
            buttonPressed = false;
            if (!gClearConfigRequested) {
                gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 0);  // 取消清除
                ESP_LOGI(TAG, "boot button released, clear cancelled");
            }
        }

        // 步骤 4: 检查长按超时
        if (buttonPressed &&
            !gClearConfigRequested &&
            (radar_now_ms() - buttonPressStartMs) >= APP_CLEAR_CONFIG_DURATION_MS) {
            // 步骤 5: 执行清除配置操作
            gClearConfigRequested = true;
            gForceLedOff = true;  // 强制关闭 LED
            clearStoredConfig();  // 清除存储配置
            gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 0);
            set_network_led_duty(0);  // 关闭 LED
            ESP_LOGW(TAG, "configuration cleared, restarting");
            radar_sleep_ms(1000);  // 延时 1 秒
            esp_restart();  // 重启系统
        }

        radar_sleep_ms(50);  // 50ms 检测周期
    }
}

/**
 * @brief LED 指示灯控制任务
 * 
 * @param parameter 任务参数（未使用）
 * 
 * 功能概述：
 * - 根据网络状态控制 LED 指示灯的亮灭模式
 * - 支持三种模式：慢闪（断开）、快闪（连接中）、呼吸（已连接）
 * - 使用 PWM（LEDc）控制 LED 亮度
 * 
 * LED 模式：
 * 1. NET_INITIAL/NET_DISCONNECTED: 慢闪（500ms 间隔）
 *    - 亮 500ms，灭 500ms
 *    - 表示 WiFi 未连接
 * 
 * 2. NET_CONNECTING: 快闪（200ms 间隔）
 *    - 亮 200ms，灭 200ms
 *    - 表示正在连接 WiFi
 * 
 * 3. NET_CONNECTED: 呼吸灯（10ms 间隔）
 *    - 亮度从 0 渐变到 255，再从 255 渐变到 0
 *    - 渐变步长：APP_LED_BREATHE_STEP
 *    - 表示 WiFi 已连接
 * 
 * 处理流程：
 * 1. 配置 LEDc（PWM 定时器、通道）
 * 2. 初始化 LED 状态（关闭）
 * 3. 循环检测网络状态
 * 4. 根据状态选择 LED 模式
 * 5. 更新 LED 占空比
 * 
 * 特殊处理：
 * - gForceLedOff = true: 强制关闭 LED（长按清除配置时）
 * - 呼吸灯：使用 gBreatheValue 记录当前亮度，gBreatheIncreasing 记录方向
 * 
 * 注意：
 * - 此任务优先级为 1（低优先级）
 * - 使用看门狗防止任务阻塞
 * - PWM 频率：5000Hz
 * - 分辨率：8 位（0-255）
 */
void ledControlTask(void *parameter)
{
    (void)parameter;

    // 步骤 1: 配置 LEDc GPIO 和定时器
    configure_led_gpio();
    set_network_led_duty(0);  // 初始关闭 LED
    gLastBlinkTimeMs = radar_now_ms();

    ESP_LOGI(TAG, "network led control task started");
    task_watchdog_register_current("led_control_task");

    while (true) {
        task_watchdog_reset_current();  // 重置看门狗
        const uint64_t nowMs = radar_now_ms();

        // 步骤 2: 强制关闭 LED（长按清除配置时）
        if (gForceLedOff) {
            set_network_led_duty(0);
            radar_sleep_ms(10);
            continue;
        }

        // 步骤 3: 根据网络状态选择 LED 模式
        switch (currentNetworkStatus) {
        // 模式 1: 初始状态或断开 - 慢闪
        case NET_INITIAL:
        case NET_DISCONNECTED:
            if ((nowMs - gLastBlinkTimeMs) >= APP_LED_SLOW_BLINK_INTERVAL_MS) {
                gLedState = !gLedState;  // 切换状态
                set_network_led_duty(gLedState ? 255U : 0U);  // 亮/灭
                gLastBlinkTimeMs = nowMs;
            }
            break;
        
        // 模式 2: 连接中 - 快闪
        case NET_CONNECTING:
            if ((nowMs - gLastBlinkTimeMs) >= APP_LED_FAST_BLINK_INTERVAL_MS) {
                gLedState = !gLedState;  // 切换状态
                set_network_led_duty(gLedState ? 255U : 0U);  // 亮/灭
                gLastBlinkTimeMs = nowMs;
            }
            break;
        
        // 模式 3: 已连接 - 呼吸灯
        case NET_CONNECTED:
            if ((nowMs - gLastBlinkTimeMs) >= APP_LED_BREATHE_INTERVAL_MS) {
                set_network_led_duty(gBreatheValue);  // 设置当前亮度
                
                // 更新呼吸值和方向
                if (gBreatheIncreasing) {
                    gBreatheValue += APP_LED_BREATHE_STEP;
                    if (gBreatheValue >= APP_LED_BREATHE_MAX) {
                        gBreatheValue = APP_LED_BREATHE_MAX;
                        gBreatheIncreasing = false;  // 到达最大值，开始减小
                    }
                } else if (gBreatheValue > APP_LED_BREATHE_STEP) {
                    gBreatheValue -= APP_LED_BREATHE_STEP;  // 减小亮度
                } else {
                    gBreatheValue = APP_LED_BREATHE_MIN;
                    gBreatheIncreasing = true;  // 到达最小值，开始增加
                }
                gLastBlinkTimeMs = nowMs;
            }
            break;
        }

        radar_sleep_ms(10);  // 10ms 控制周期
    }
}

/**
 * @brief WiFi 监控任务
 * 
 * @param parameter 任务参数（未使用）
 * 
 * 功能概述：
 * - 监控 WiFi 连接状态
 * - 根据 WiFi 状态更新系统网络状态
 * - 处理 WiFi 凭证和自动重连
 * 
 * 处理流程：
 * 1. 初始化 WiFi 管理器
 * 2. 检查已保存的凭证：
 *    - 有凭证：等待后台自动重连
 *    - 无凭证但有默认配置：使用默认配置连接
 *    - 无凭证：等待用户配置
 * 3. 循环监控 WiFi 状态
 * 4. 根据状态更新系统网络状态
 * 5. 打印状态日志
 * 
 * WiFi 状态分类：
 * - WIFI_MANAGER_CONNECTED → NET_CONNECTED
 * - WIFI_MANAGER_SCANNING/CONNECTING/CONFIGURING → NET_CONNECTING
 * - WIFI_MANAGER_DISCONNECTED/IDLE/INITIALIZED/ERROR → NET_DISCONNECTED
 * 
 * 凭证处理逻辑：
 * 1. 优先使用已保存的凭证（NVs 存储）
 * 2. 如果没有保存的凭证，使用 app_config.h 中的默认配置
 * 3. 如果都没有，等待用户通过 BLE 配置
 * 
 * 注意：
 * - 此任务优先级为 2（中优先级）
 * - 不再主动调用 wifi_manager_connect()，让后台重连任务负责
 * - 检测周期：30 秒
 */
void wifiMonitorTask(void *parameter)
{
    (void)parameter;

    ESP_LOGI(TAG, "wifi monitor task started");

    // 步骤 1: 初始化 WiFi 管理器
    if (!wifi_manager_init()) {
        ESP_LOGE(TAG, "wifi manager init failed inside wifi monitor task");
        return;
    }

    // 步骤 2: 检查并处理 WiFi 凭证
    const WiFiManagerSnapshot initialSnapshot = wifi_manager_get_snapshot();
    if (initialSnapshot.has_credentials != 0U) {
        // 情况 1: 有已保存的凭证，等待后台自动重连
        ESP_LOGI(TAG, "stored wifi credentials found, waiting for background reconnect");
        // 不再主动调用 wifi_manager_connect()，让后台重连任务负责唯一连接入口
    } else if (APP_WIFI_DEFAULT_SSID[0] != '\0') {
        // 情况 2: 无保存凭证，但有默认配置
        ESP_LOGI(TAG, "using default wifi credentials from app config");
        (void)wifi_manager_configure_and_connect(APP_WIFI_DEFAULT_SSID, APP_WIFI_DEFAULT_PASSWORD);
    } else {
        // 情况 3: 无凭证，等待用户配置
        ESP_LOGI(TAG, "no wifi credentials configured yet");
    }

    // 步骤 3: 循环监控 WiFi 状态
    while (true) {
        const WiFiManagerSnapshot snapshot = wifi_manager_get_snapshot();
        
        // 步骤 4: 根据 WiFi 状态更新系统网络状态
        switch (snapshot.state) {
        case WIFI_MANAGER_CONNECTED:
            setNetworkStatus(NET_CONNECTED);
            break;
        case WIFI_MANAGER_SCANNING:
        case WIFI_MANAGER_CONNECTING:
        case WIFI_MANAGER_CONFIGURING:
            setNetworkStatus(NET_CONNECTING);
            break;
        case WIFI_MANAGER_DISCONNECTED:
        case WIFI_MANAGER_IDLE:
        case WIFI_MANAGER_INITIALIZED:
        case WIFI_MANAGER_ERROR:
            setNetworkStatus(NET_DISCONNECTED);
            break;
        default:
            setNetworkStatus(NET_INITIAL);
            break;
        }

        // 步骤 5: 打印状态日志
        ESP_LOGI(TAG,
                 "wifi monitor state=%d connected=%u creds=%u retry=%u ssid=%s",
                 (int)snapshot.state,
                 (unsigned)snapshot.connected,
                 (unsigned)snapshot.has_credentials,
                 (unsigned)snapshot.reconnect_attempts,
                 snapshot.ssid[0] != '\0' ? snapshot.ssid : "-");

        radar_sleep_ms(30000);  // 30 秒检测周期
    }
}

void bleConfigTask(void *parameter)
{
    (void)parameter;

    ESP_LOGI(TAG, "ble config task started");

    if (!ble_manager_init()) {
        ESP_LOGW(TAG, "ble manager init failed inside ble config task");
        return;
    }

    sendStatusToBLE();
    task_watchdog_register_current("ble_config_task");

    while (true) {
        task_watchdog_reset_current();
        
        ESP_LOGI(TAG, "before BLE process, watermark=%u words",
                 (unsigned)uxTaskGetStackHighWaterMark(nullptr));
        
        processBLEConfig();
        
        ESP_LOGI(TAG, "after BLE process, watermark=%u words",
                 (unsigned)uxTaskGetStackHighWaterMark(nullptr));

        const BleManagerSnapshot bleSnapshot = ble_manager_get_snapshot();
        if (bleSnapshot.initialized == 0U) {
            ESP_LOGW(TAG, "ble not initialized, retrying");
            (void)ble_manager_init();
        }

        static uint8_t lastConnected = 0U;
        static uint8_t lastNotifyEnabled = 0U;
        if (bleSnapshot.connected != lastConnected ||
            bleSnapshot.notify_enabled != lastNotifyEnabled) {
            // 新增：连接恢复后，如果 continuous 模式还开着，自动补发一帧
            if (lastConnected == 0U && bleSnapshot.connected != 0U &&
                gContinuousSendEnabled &&
                !wifi_manager_is_connecting()) {
                ESP_LOGI(TAG, "BLE reconnected, sending initial continuous frame");
                gLastContinuousCheckMs = radar_now_ms();
                (void)send_continuous_radar_packet_to_ble();
            }
            // 去掉断连时自动清零 gContinuousSendEnabled 的逻辑
            // if (lastConnected != 0U && bleSnapshot.connected == 0U) {
            //     gContinuousSendEnabled = false;
            // }
            lastConnected = bleSnapshot.connected;
            lastNotifyEnabled = bleSnapshot.notify_enabled;
            // WiFi 连接中时降频发送 BLE 状态包
            if (!wifi_manager_is_connecting()) {
                sendStatusToBLE();
            }
        }

        if (gContinuousSendEnabled &&
            ble_manager_is_client_connected() &&
            !wifi_manager_is_connecting()) {  // WiFi 连接中时暂停 BLE 连续发送
            const uint64_t nowMs = radar_now_ms();
            if (gLastContinuousCheckMs == 0U ||
                (nowMs - gLastContinuousCheckMs) >= gContinuousSendIntervalMs) {
                gLastContinuousCheckMs = nowMs;
                (void)send_continuous_radar_packet_to_ble();
            }
        }

        radar_sleep_ms(10);
    }
}

/**
 * @brief 雷达命令任务
 * 
 * @param parameter 任务参数（未使用）
 * 
 * 功能概述：
 * - 定时向雷达发送查询命令
 * - 维持雷达数据持续上报
 * - 循环发送 8 个预设命令
 * 
 * 命令列表（8 个）：
 * 1. {0x84, 0x81, 0x0F}: 查询睡眠状态
 * 2. {0x84, 0x8D, 0x0F}: 查询睡眠汇总
 * 3. {0x84, 0x8F, 0x0F}: 查询完整睡眠报告
 * 4. {0x84, 0x8E, 0x0F}: 查询异常状态
 * 5. {0x84, 0x91, 0x0F}: 查询挣扎告警
 * 6. {0x84, 0x83, 0x0F}: 查询清醒时间
 * 7. {0x85, 0x02, 0x0F}: 查询心率
 * 8. {0x81, 0x02, 0x0F}: 查询呼吸率
 * 
 * 处理流程：
 * 1. 注册看门狗
 * 2. 循环发送命令（索引 0-7）
 * 3. 每个命令间隔 2 秒
 * 4. 发送完一轮后重新开始
 * 
 * 注意：
 * - 此任务优先级为 4（高优先级）
 * - 使用看门狗防止任务阻塞
 * - 命令周期：16 秒（8 个命令 × 2 秒）
 * - 命令参数 0x0F 表示查询模式
 */
static void radar_command_task(void *parameter)
{
    (void)parameter;

    // 预设命令列表（8 个查询命令）
    static const uint8_t radarCommands[][3] = {
        {0x84, 0x81, 0x0F},  // 查询睡眠状态
        {0x84, 0x8D, 0x0F},  // 查询睡眠汇总
        {0x84, 0x8F, 0x0F},  // 查询完整睡眠报告
        {0x84, 0x8E, 0x0F},  // 查询异常状态
        {0x84, 0x91, 0x0F},  // 查询挣扎告警
        {0x84, 0x83, 0x0F},  // 查询清醒时间
        {0x85, 0x02, 0x0F},  // 查询心率
        {0x81, 0x02, 0x0F},  // 查询呼吸率
    };

    size_t commandIndex = 0;  // 当前命令索引
    task_watchdog_register_current("radar_command_task");

    while (true) {
        task_watchdog_reset_current();  // 重置看门狗
        const uint8_t *command = radarCommands[commandIndex];
        
        // 发送雷达命令
        sendRadarCommand(command[0], command[1], command[2]);

        // 更新索引（循环）
        commandIndex = (commandIndex + 1U) % (sizeof(radarCommands) / sizeof(radarCommands[0]));
        radar_sleep_ms(2000);  // 2 秒间隔
    }
}

/**
 * @brief 雷达状态任务
 * 
 * @param parameter 任务参数（未使用）
 * 
 * 功能概述：
 * - 监控雷达数据状态
 * - 上报数据到 MQTT 和 InfluxDB
 * - 打印系统综合状态日志
 * 
 * 核心功能：
 * 1. 状态监控：
 *    - 获取雷达快照（传感器数据）
 *    - 获取变化标志
 *    - 获取系统状态、BLE、WiFi、MQTT 快照
 *    - 打印综合状态日志
 * 
 * 2. MQTT 数据上报：
 *    - 数据变化时立即上报
 *    - 无人时发送心跳（10 秒间隔）
 *    - 有人时发送日常数据（5 秒间隔）
 *    - 有人时发送睡眠数据（10 秒间隔）
 * 
 * 3. InfluxDB 数据上报：
 *    - 日常数据：变化时上报，最少 1.5 秒间隔，强制 60 秒间隔
 *    - 睡眠数据：5 秒间隔定时上报
 * 
 * 处理流程：
 * 1. 获取所有状态快照
 * 2. 打印综合状态日志
 * 3. 检查数据变化，上报 MQTT
 * 4. 检查心跳超时，发送心跳包
 * 5. 检查日常数据超时，上报日常数据
 * 6. 检查睡眠数据超时，上报睡眠数据
 * 7. 上报 InfluxDB 日常数据
 * 8. 上报 InfluxDB 睡眠数据
 * 
 * 注意：
 * - 此任务优先级为 2（中优先级）
 * - 栈大小 8192 字节（需要多个 snapshot 和 buffer）
 * - 使用看门狗防止任务阻塞
 * - 执行周期：5 秒
 */
static void radar_status_task(void *parameter)
{
    (void)parameter;
    task_watchdog_register_current("radar_status_task");

    // 使用 static 变量减少栈占用
    static RadarReportSnapshot snapshot = {};
    static RadarChangeFlags changeFlags = {};
    static SystemStateSnapshot systemState = {};
    static BleManagerSnapshot bleState = {};
    static WiFiManagerSnapshot wifiState = {};
    static MqttManagerSnapshot mqttState = {};

    while (true) {
        task_watchdog_reset_current();  // 重置看门狗
        
        // 步骤 1: 获取所有状态快照
        snapshot = radar_manager_get_report_snapshot(10000);  // 10 秒新鲜度
        changeFlags = radar_manager_get_change_flags();
        systemState = system_state_get_snapshot();
        bleState = ble_manager_get_snapshot();
        wifiState = wifi_manager_get_snapshot();
        mqttState = mqtt_manager_get_snapshot();

        // 步骤 2: 打印综合状态日志
        ESP_LOGI(TAG,
                 "system phase=%d net=%d dev=%u sn=%" PRIu64 " radar=%u tasks=%u ble=%u wifi=%u mqtt=%u err=%u uptime=%" PRIu64 "ms | ble init=%u adv=%u conn=%u notify=%u name=%s | wifi state=%d conn=%u creds=%u retry=%u ssid=%s | mqtt state=%d conn=%u cfg=%u retry=%u | radar uart=%s fresh=%s changed=%s person=%s bed=%s vitals=%s sample_age=%" PRIu64 "ms presence=%u motion=%u dist=%u hr=%.1f rr=%.1f sleep=%u body=%u abnormal=%u",
                 (int)systemState.phase,
                 (int)systemState.network_status,
                 (unsigned)systemState.current_device_id,
                 systemState.device_sn,
                 static_cast<unsigned>(systemState.radar_ready),
                 static_cast<unsigned>(systemState.tasks_ready),
                 static_cast<unsigned>(systemState.ble_ready),
                 static_cast<unsigned>(systemState.wifi_ready),
                 static_cast<unsigned>(systemState.mqtt_ready),
                 static_cast<unsigned>(systemState.error),
                 systemState.uptime_ms,
                 static_cast<unsigned>(bleState.initialized),
                 static_cast<unsigned>(bleState.advertising),
                 static_cast<unsigned>(bleState.connected),
                 static_cast<unsigned>(bleState.notify_enabled),
                 bleState.device_name[0] != '\0' ? bleState.device_name : "-",
                 (int)wifiState.state,
                 static_cast<unsigned>(wifiState.connected),
                 static_cast<unsigned>(wifiState.has_credentials),
                 static_cast<unsigned>(wifiState.reconnect_attempts),
                 wifiState.ssid[0] != '\0' ? wifiState.ssid : "-",
                 (int)mqttState.state,
                 static_cast<unsigned>(mqttState.connected),
                 static_cast<unsigned>(mqttState.configured),
                 static_cast<unsigned>(mqttState.reconnect_attempts),
                 radar_manager_is_initialized() ? "ready" : "down",
                 snapshot.has_fresh_sample ? "yes" : "no",
                 changeFlags.any_changed ? "yes" : "no",
                 snapshot.has_person ? "yes" : "no",
                 snapshot.bed_occupied ? "yes" : "no",
                 snapshot.has_valid_vitals ? "yes" : "no",
                 (uint64_t)snapshot.sample_age_ms,
                 static_cast<unsigned>(snapshot.presence),
                 static_cast<unsigned>(snapshot.motion),
                 static_cast<unsigned>(snapshot.distance),
                 snapshot.heart_rate,
                 snapshot.breath_rate,
                 static_cast<unsigned>(snapshot.sleep_state),
                 static_cast<unsigned>(snapshot.body_movement),
                 static_cast<unsigned>(snapshot.abnormal_state));

        // 步骤 3: 数据变化时上报 MQTT
        if (changeFlags.any_changed) {
            ESP_LOGI(TAG, "radar_status_task watermark before publish=%u words",
                     (unsigned)uxTaskGetStackHighWaterMark(nullptr));
            
            ESP_LOGI(TAG,
                     "change flags presence=%u motion=%u sleep=%u body=%u distance=%u hr=%u rr=%u abnormal=%u",
                     changeFlags.presence_changed,
                     changeFlags.motion_changed,
                     changeFlags.sleep_state_changed,
                     changeFlags.body_movement_changed,
                     changeFlags.distance_changed,
                     changeFlags.heart_rate_changed,
                     changeFlags.breath_rate_changed,
                     changeFlags.abnormal_state_changed);

            if (mqtt_manager_is_connected()) {
                const bool published = mqtt_manager_publish_radar_snapshot(&snapshot);
                const bool isNoOne = snapshot.presence == 0U ||
                                     (snapshot.heart_rate == 0.0f && snapshot.breath_rate == 0.0f);
                if (published) {
                    if (isNoOne) {
                        gLastMqttHeartbeatMs = radar_now_ms();  // 更新心跳时间
                    } else {
                        gLastMqttDailySendMs = radar_now_ms();  // 更新日常数据时间
                    }
                }
            }

            radar_manager_mark_snapshot_consumed();  // 标记已消费
        }

        const uint64_t nowMs = radar_now_ms();
        const bool isMqttNoOne = snapshot.presence == 0U ||
                                 (snapshot.heart_rate == 0.0f && snapshot.breath_rate == 0.0f);
        
        // 步骤 4: 无人时发送 MQTT 心跳包
        if (mqtt_manager_is_connected() &&
            isMqttNoOne &&
            (gLastMqttHeartbeatMs == 0U ||
             (nowMs - gLastMqttHeartbeatMs) >= kMqttHeartbeatIntervalMs)) {
            task_watchdog_reset_current();
            if (mqtt_manager_publish_online()) {
                gLastMqttHeartbeatMs = radar_now_ms();
            }
            task_watchdog_reset_current();
        }

        // 步骤 5: 有人时发送 MQTT 日常数据
        if (mqtt_manager_is_connected() &&
            !isMqttNoOne &&
            (gLastMqttDailySendMs == 0U ||
             (nowMs - gLastMqttDailySendMs) >= kMqttDailyDataIntervalMs)) {
            task_watchdog_reset_current();
            if (mqtt_manager_publish_radar_snapshot(&snapshot)) {
                gLastMqttDailySendMs = radar_now_ms();
            }
            task_watchdog_reset_current();
        }

        // 步骤 6: 有人时发送 MQTT 睡眠数据
        if (mqtt_manager_is_connected() &&
            !isMqttNoOne &&
            (gLastMqttSleepSendMs == 0U ||
             (nowMs - gLastMqttSleepSendMs) >= kMqttSleepDataIntervalMs)) {
            task_watchdog_reset_current();
            if (mqtt_manager_publish_sleep_snapshot(&snapshot)) {
                gLastMqttSleepSendMs = radar_now_ms();
            }
            task_watchdog_reset_current();
        }

        // 步骤 7: 上报 InfluxDB 日常数据
        const bool dailyChanged = influx_daily_snapshot_changed(snapshot);
        const bool dailyForceDue = (gLastInfluxDailySendMs == 0U) ||
                                   ((nowMs - gLastInfluxDailySendMs) >= kInfluxDailyForceSendIntervalMs);
        const bool dailyMinElapsed = (gLastInfluxDailySendMs == 0U) ||
                                     ((nowMs - gLastInfluxDailySendMs) >= kInfluxDailyMinSendIntervalMs);

        if ((dailyChanged || dailyForceDue) && dailyMinElapsed) {
            task_watchdog_reset_current();
            ESP_LOGI(TAG, "radar_status_task watermark before influx daily=%u words",
                     (unsigned)uxTaskGetStackHighWaterMark(nullptr));
            if (influx_manager_send_daily_data(&snapshot)) {
                gLastInfluxDailySnapshot = snapshot;
                gHasLastInfluxDailySnapshot = true;
                gLastInfluxDailySendMs = radar_now_ms();
            }
            task_watchdog_reset_current();
        }

        // 步骤 8: 上报 InfluxDB 睡眠数据
        if (gLastInfluxSleepSendMs == 0U ||
            (nowMs - gLastInfluxSleepSendMs) >= kInfluxSleepSendIntervalMs) {
            const SensorData currentData = radar_manager_get_sensor_data();
            task_watchdog_reset_current();
            ESP_LOGI(TAG, "radar_status_task watermark before influx sleep=%u words",
                     (unsigned)uxTaskGetStackHighWaterMark(nullptr));
            (void)influx_manager_send_sleep_data(&currentData);
            gLastInfluxSleepSendMs = radar_now_ms();
            task_watchdog_reset_current();
        }

        radar_sleep_ms(5000);  // 5 秒执行周期
    }
}

/**
 * @brief 雷达数据分析任务
 * 
 * @param parameter 任务参数（未使用）
 * 
 * 功能概述：
 * - 分析心率和呼吸率数据
 * - 计算情绪状态（压力、放松）
 * - 分析睡眠质量
 * - 估算 HRV（心率变异性）
 * 
 * 核心组件：
 * 1. PhysioDataProcessor: 生理数据处理
 *    - 心率平滑和趋势分析
 *    - 呼吸率平滑和趋势分析
 *    - HRV 估算
 * 
 * 2. SimpleEmotionAnalyzer: 情绪分析
 *    - 基于生理数据的情绪状态
 *    - 压力水平计算
 *    - 放松水平计算
 *    - 基线校准
 * 
 * 3. SleepAnalyzer: 睡眠分析
 *    - 睡眠阶段识别
 *    - 睡眠质量评估
 * 
 * 处理流程：
 * 1. 获取雷达传感器数据
 * 2. 检查数据有效性（心率或呼吸率有效）
 * 3. 更新生理数据处理器
 * 4. 获取心率、呼吸率、HRV 数据
 * 5. 构建体动数据
 * 6. 校准情绪分析基线
 * 7. 分析情绪状态
 * 8. 更新睡眠分析
 * 9. 打印分析结果
 * 
 * 注意：
 * - 此任务优先级为 3（中优先级）
 * - 栈大小 8192 字节
 * - 使用看门狗防止任务阻塞
 * - 执行周期：1 秒
 */
static void radar_analysis_task(void *parameter)
{
    (void)parameter;

    // 初始化分析器
    PhysioDataProcessor processor;      // 生理数据处理器
    SimpleEmotionAnalyzer emotionAnalyzer;  // 情绪分析器
    SleepAnalyzer sleepAnalyzer;        // 睡眠分析器
    task_watchdog_register_current("radar_analysis_task");

    while (true) {
        task_watchdog_reset_current();  // 重置看门狗
        const SensorData currentData = radar_manager_get_sensor_data();
        
        // 步骤 1: 检查数据有效性
        const bool hasVitalSample = (currentData.heart_valid != 0U) || (currentData.breath_valid != 0U);
        if (!hasVitalSample) {
            radar_sleep_ms(1000);  // 无有效数据，跳过
            continue;
        }

        const float hr = currentData.heart_valid ? currentData.heart_rate : 0.0f;
        const float rr = currentData.breath_valid ? currentData.breath_rate : 0.0f;
        if (hr <= 0.0f && rr <= 0.0f) {
            radar_sleep_ms(1000);  // 心率和呼吸率都为 0，跳过
            continue;
        }

        // 步骤 2: 更新生理数据处理器
        processor.update(hr,
                         rr,
                         currentData.heart_valid ? 80.0f : 0.0f,  // 心率置信度
                         currentData.breath_valid ? 80.0f : 0.0f); // 呼吸率置信度

        // 步骤 3: 获取处理结果
        const HeartRateData hrData = processor.getHeartRateData();
        const RespirationData rrData = processor.getRespirationData();
        const HRVEstimate hrvData = processor.getHRVEstimate();

        // 步骤 4: 构建体动数据
        BodyMovementData movementData = {};
        movementData.movement = currentData.body_movement;
        movementData.movementSmoothed = static_cast<float>(currentData.body_movement);
        movementData.movementMean = static_cast<float>(currentData.body_movement);
        movementData.movementStd = 0.0f;
        movementData.activityLevel = static_cast<float>(currentData.body_movement) / 100.0f;
        movementData.isValid = true;
        movementData.timestamp = radar_now_ms();

        // 步骤 5: 校准情绪分析基线
        if (hrData.isValid || rrData.isValid) {
            emotionAnalyzer.calibrateBaseline(hrData, rrData, movementData);
        }

        // 步骤 6: 分析情绪状态
        const EmotionResult emotionResult =
            emotionAnalyzer.analyze(hrData, rrData, hrvData, movementData);
        
        // 步骤 7: 更新睡眠分析
        sleepAnalyzer.update(hrData, rrData, hrvData, movementData);

        // 步骤 8: 打印分析结果
        ESP_LOGI(TAG,
                 "analysis emotion=%s sleep=%s hr=%.1f rr=%.1f stress=%.1f relax=%.1f",
                 EmotionOutput::toBrief(emotionResult).c_str(),
                 sleepAnalyzer.formatState().c_str(),
                 hrData.bpmSmoothed,
                 rrData.rateSmoothed,
                 emotionResult.stressLevel,
                 emotionResult.relaxationLevel);

        radar_sleep_ms(1000);  // 1 秒执行周期
    }
}

/**
 * @brief 设备控制台任务
 * 
 * @param parameter 任务参数（未使用）
 * 
 * 功能概述：
 * - 提供串口命令行接口
 * - 解析和执行用户输入的命令
 * - 支持调试和配置功能
 * 
 * 处理流程：
 * 1. 等待用户输入（从 stdin 读取）
 * 2. 读取一行命令
 * 3. 调用 device_command_handle_line 解析
 * 4. 处理完成后继续等待下一行
 * 
 * 支持的命令：
 * - help: 显示帮助信息
 * - show_device: 显示设备信息
 * - show_wifi: 显示 WiFi 状态
 * - show_ble: 显示 BLE 状态
 * - show_mqtt: 显示 MQTT 状态
 * - set_device_id: 设置设备 ID
 * - set_device_sn: 设置设备序列号
 * - clear_wifi: 清除 WiFi 配置
 * - scan_wifi: 扫描 WiFi 网络
 * - reboot: 重启系统
 * 
 * 注意：
 * - 此任务优先级为 1（低优先级）
 * - 使用标准输入（stdin）
 * - 执行周期：200ms（无输入时）
 * - 需要启用 CONFIG_NEWLIB_STDIN 配置
 */
static void device_console_task(void *parameter)
{
    (void)parameter;

    ESP_LOGI(TAG, "device console ready, type 'help' or 'show_device'");

    char line[128];  // 命令行缓冲区
    while (true) {
        // 从标准输入读取一行
        if (std::fgets(line, sizeof(line), stdin) != nullptr) {
            (void)device_command_handle_line(line);  // 解析并执行命令
            continue;
        }

        radar_sleep_ms(200);  // 无输入时延时 200ms
    }
}

bool tasks_manager_init(void)
{
    if (tasksInitialized) {
        return true;
    }

    ESP_LOGI(TAG, "initializing tasks manager");
    setNetworkStatus(NET_INITIAL);

    if (radarCommandTaskHandle == nullptr) {
        xTaskCreate(radar_command_task,
                    "radar_command_task",
                    4096,
                    nullptr,
                    4,
                    &radarCommandTaskHandle);
    }

    if (radarStatusTaskHandle == nullptr) {
        xTaskCreate(radar_status_task,
                    "radar_status_task",
                    8192,  // 从 4096 增加到 8192，因为任务需要多个 snapshot 和 MQTT/Influx buffer
                    nullptr,
                    2,
                    &radarStatusTaskHandle);
    }

    if (radarAnalysisTaskHandle == nullptr) {
        xTaskCreate(radar_analysis_task,
                    "radar_analysis_task",
                    8192,
                    nullptr,
                    3,
                    &radarAnalysisTaskHandle);
    }

    if (deviceConsoleTaskHandle == nullptr) {
        xTaskCreate(device_console_task,
                    "device_console_task",
                    4096,
                    nullptr,
                    1,
                    &deviceConsoleTaskHandle);
    }

    if (bootButtonTaskHandle == nullptr) {
        xTaskCreate(bootButtonMonitorTask,
                    "boot_button_task",
                    4096,
                    nullptr,
                    2,
                    &bootButtonTaskHandle);
    }

    if (ledControlTaskHandle == nullptr) {
        xTaskCreate(ledControlTask,
                    "led_control_task",
                    3072,
                    nullptr,
                    1,
                    &ledControlTaskHandle);
    }

    if (wifiMonitorTaskHandle == nullptr) {
        xTaskCreate(wifiMonitorTask,
                    "wifi_monitor_task",
                    8192,  // 从 4096 增加到 8192，避免栈溢出
                    nullptr,
                    2,
                    &wifiMonitorTaskHandle);
    }

    if (bleConfigTaskHandle == nullptr) {
        xTaskCreate(bleConfigTask,
                    "ble_config_task",
                    8192,  // 从 4096 增加到 8192，避免栈溢出
                    nullptr,
                    1,
                    &bleConfigTaskHandle);
    }

    tasksInitialized = true;
    system_state_set_tasks_ready(1U);
    system_state_set_phase(SYSTEM_PHASE_TASKS_READY);
    ESP_LOGI(TAG, "tasks manager started");
    return true;
}

bool initAllTasks(void)
{
    ESP_LOGI(TAG, "initAllTasks called");
    return tasks_manager_init();
}

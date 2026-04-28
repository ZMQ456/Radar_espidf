/**
 * @file mqtt_manager.cpp
 * @brief MQTT 客户端管理模块（阿里云物联网平台）
 * 
 * 功能概述：
 * - 实现与阿里云物联网平台的 MQTT 连接和通信
 * - 支持设备属性上报（心跳、日常数据、睡眠数据）
 * - 支持云端指令下行处理（设置参数、查询状态）
 * - 自动重连机制和状态管理
 * 
 * 核心特性：
 * 1. 鉴权认证：基于 ProductKey、DeviceName、Password 三元组
 * 2. 自动生成：DeviceName 使用 device_sn，Password 使用 MD5 签名
 * 3. 话题管理：自动构建订阅和发布话题
 * 4. JSON 解析：内置轻量级 JSON 解析器（支持转义、Unicode）
 * 5. 失败处理：独立重连任务，不阻塞主线程
 * 
 * MQTT 连接参数：
 * - ProductKey: dEkr5BkkXTFZFBdR
 * - DeviceModel: radar_1.0
 * - ClientID: {ProductKey}_{DeviceName}_{DeviceModel}
 * - Username: {DeviceName}
 * - Password: MD5({ProductSecret} + {ClientID})
 * 
 * 话题格式：
 * - 订阅：/sys/{ProductKey}/{DeviceName}/c/#
 * - 发布：/sys/{ProductKey}/{DeviceName}/s/event/property/post
 * 
 * 支持的下行方法：
 * - thing.service.property.set: 设置设备参数
 * - thing.service.property.get: 查询设备状态
 */
// ============================================================================
// 头文件包含和全局变量
// ============================================================================
#include "mqtt_manager.h"  // MQTT 管理器接口声明

#include <cstdio>         // C 标准输入输出（snprintf, sprintf）
#include <cstdlib>        // C 标准库（strtoul）
#include <cstring>        // C 字符串操作（strlen, strncpy）

#include "device_identity.h"  // 设备身份（获取 device_sn）
#include "esp_err.h"          // ESP32 错误码定义
#include "esp_event.h"        // ESP32 事件循环
#include "esp_log.h"          // ESP32 日志系统
#include "esp_rom_md5.h"      // MD5 哈希算法（生成密码）
#include "mqtt_client.h"      // ESP MQTT 客户端
#include "radar_platform.h"   // 平台接口（radar_now_ms, radar_sleep_ms）
#include "system_state.h"     // 系统状态管理
#include "tasks_manager.h"    // 任务管理器（连续发送配置）
#include "wifi_manager.h"     // WiFi 管理器（检查连接状态）
#include "freertos/FreeRTOS.h" // FreeRTOS 接口
#include "freertos/task.h"     // FreeRTOS 任务管理

static const char *TAG = "mqtt_manager";  // 日志标签

// 全局 MQTT 状态快照
static MqttManagerSnapshot gMqttState = {};

// MQTT 客户端句柄
static esp_mqtt_client_handle_t gMqttClient = nullptr;

// MQTT 连接任务句柄
static TaskHandle_t gMqttConnectTaskHandle = nullptr;

// MQTT 消息 ID 计数器（用于请求 - 响应匹配）
static uint32_t gMqttMessageId = 1U;

// 鉴权三元组缓冲区
static char gMqttDeviceName[32] = {};           // DeviceName
static char gMqttUsername[32] = {};             // Username
static char gMqttPassword[33] = {};             // Password (32 字符 + '\0')
static char gMqttSubscribeTopic[128] = {};      // 订阅话题
static char gMqttPropertyPostTopic[128] = {};   // 发布话题

// ============================================================================
// 鉴权和身份管理函数
// ============================================================================
/**
 * @brief 生成 payload 中使用的 deviceId（使用 device_sn）
 * 
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * 
 * 用途：
 * - MQTT payload 中的 deviceId 字段
 * - 使用 device_sn（64 位）而不是 device_id（16 位）
 * - 和 Arduino 版本保持一致
 * 
 * 格式：
 * - 直接输出 device_sn 的十进制字符串
 * - 例："1234567890123456789"
 */
static void mqtt_manager_make_payload_device_id(char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return;
    }

    const DeviceIdentity identity = device_identity_get();
    std::snprintf(buffer,
                  buffer_size,
                  "%llu",
                  static_cast<unsigned long long>(identity.device_sn));
}

// MQTT 连接常量
static constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;      // 重连间隔（5 秒）
static constexpr const char *MQTT_PRODUCT_KEY = "dEkr5BkkXTFZFBdR";     // 阿里云 ProductKey
static constexpr const char *MQTT_DEVICE_MODEL = "radar_1.0";           // 设备型号
static constexpr const char *MQTT_PRODUCT_SECRET = "2e7957febfcb48b08a1c69b8deb56738";  // 阿里云 ProductSecret

/**
 * @brief 生成 MQTT DeviceName
 * 
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * 
 * 生成规则：
 * 1. 优先使用 device_sn（如果非 0）
 * 2. 如果 device_sn 为 0，使用 MAC 地址（去除冒号）
 * 3. 如果 MAC 获取失败，使用 "unknown"
 * 
 * 示例：
 * - device_sn=123456 → "123456"
 * - MAC="AA:BB:CC:DD:EE:FF" → "AABBCCDDEEFF"
 */
static void mqtt_manager_make_device_name(char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return;
    }

    const DeviceIdentity identity = device_identity_get();
    
    // 规则 1: 优先使用 device_sn
    if (identity.device_sn != 0ULL) {
        std::snprintf(buffer,
                      buffer_size,
                      "%llu",
                      static_cast<unsigned long long>(identity.device_sn));
        return;
    }

    // 规则 2: 使用 MAC 地址
    char mac[24] = {};
    if (!radar_manager_get_device_mac(mac, sizeof(mac))) {
        // 规则 3: MAC 获取失败，使用 "unknown"
        std::strncpy(buffer, "unknown", buffer_size - 1U);
        buffer[buffer_size - 1U] = '\0';
        return;
    }

    // 去除 MAC 地址中的冒号
    size_t out = 0U;
    for (size_t i = 0U; mac[i] != '\0' && out < buffer_size - 1U; ++i) {
        if (mac[i] != ':') {
            buffer[out++] = mac[i];
        }
    }
    buffer[out] = '\0';
}

/**
 * @brief 生成 MQTT Password（MD5 签名）
 * 
 * @param client_id MQTT ClientID
 * @param buffer 输出缓冲区（至少 33 字节）
 * @param buffer_size 缓冲区大小
 * 
 * 算法：
 * 1. 拼接字符串：raw = ProductSecret + ClientID
 * 2. 计算 MD5：password = MD5(raw)
 * 3. 转换为十六进制字符串（32 字符）
 * 
 * 示例：
 * - ProductSecret: "2e7957febfcb48b08a1c69b8deb56738"
 * - ClientID: "dEkr5BkkXTFZFBdR_123456_radar_1.0"
 * - raw: "2e7957febfcb48b08a1c69b8deb56738dEkr5BkkXTFZFBdR_123456_radar_1.0"
 * - password: "a1b2c3d4e5f6..." (32 字符十六进制)
 * 
 * 注意：
 * - 这是阿里云物联网平台的鉴权方式
 * - ClientID 必须与连接时使用的完全一致
 */
static void mqtt_manager_make_password(const char *client_id, char *buffer, size_t buffer_size)
{
    if (client_id == nullptr || buffer == nullptr || buffer_size < 33U) {
        return;
    }

    // 步骤 1: 拼接字符串
    char raw[160] = {};
    std::snprintf(raw, sizeof(raw), "%s%s", MQTT_PRODUCT_SECRET, client_id);

    // 步骤 2: 计算 MD5
    uint8_t digest[ESP_ROM_MD5_DIGEST_LEN] = {};
    md5_context_t ctx = {};
    esp_rom_md5_init(&ctx);
    esp_rom_md5_update(&ctx, raw, static_cast<uint32_t>(std::strlen(raw)));
    esp_rom_md5_final(digest, &ctx);

    // 步骤 3: 转换为十六进制字符串
    for (size_t i = 0U; i < sizeof(digest); ++i) {
        std::snprintf(buffer + (i * 2U), buffer_size - (i * 2U), "%02x", digest[i]);
    }
    buffer[32] = '\0';  // 确保字符串结束
}

/**
 * @brief 刷新 MQTT 鉴权身份（DeviceName、Username、Password、Topics）
 * 
 * 处理流程：
 * 1. 生成 DeviceName（调用 mqtt_manager_make_device_name）
 * 2. 设置 Username = DeviceName
 * 3. 生成 ClientID = ProductKey_DeviceName_DeviceModel
 * 4. 生成 Password = MD5(ProductSecret + ClientID)
 * 5. 构建订阅话题：/sys/{ProductKey}/{DeviceName}/c/#
 * 6. 构建发布话题：/sys/{ProductKey}/{DeviceName}/s/event/property/post
 * 7. 复制发布话题到 test_topic（用于调试）
 * 
 * 示例输出：
 * - DeviceName: "123456"
 * - Username: "123456"
 * - ClientID: "dEkr5BkkXTFZFBdR_123456_radar_1.0"
 * - Password: "a1b2c3d4e5f6..."
 * - Subscribe: "/sys/dEkr5BkkXTFZFBdR/123456/c/#"
 * - Publish: "/sys/dEkr5BkkXTFZFBdR/123456/s/event/property/post"
 */
static void mqtt_manager_refresh_enjoy_iot_identity(void)
{
    // 步骤 1-2: 生成 DeviceName 和 Username
    mqtt_manager_make_device_name(gMqttDeviceName, sizeof(gMqttDeviceName));
    std::strncpy(gMqttUsername, gMqttDeviceName, sizeof(gMqttUsername) - 1U);
    gMqttUsername[sizeof(gMqttUsername) - 1U] = '\0';

    // 步骤 3: 生成 ClientID
    std::snprintf(gMqttState.client_id,
                  sizeof(gMqttState.client_id),
                  "%s_%s_%s",
                  MQTT_PRODUCT_KEY,
                  gMqttDeviceName,
                  MQTT_DEVICE_MODEL);

    // 步骤 4: 生成 Password
    mqtt_manager_make_password(gMqttState.client_id, gMqttPassword, sizeof(gMqttPassword));

    // 步骤 5-6: 构建话题
    std::snprintf(gMqttSubscribeTopic,
                  sizeof(gMqttSubscribeTopic),
                  "/sys/%s/%s/c/#",
                  MQTT_PRODUCT_KEY,
                  gMqttDeviceName);
    std::snprintf(gMqttPropertyPostTopic,
                  sizeof(gMqttPropertyPostTopic),
                  "/sys/%s/%s/s/event/property/post",
                  MQTT_PRODUCT_KEY,
                  gMqttDeviceName);

    // 步骤 7: 复制到 test_topic
    std::strncpy(gMqttState.test_topic,
                 gMqttPropertyPostTopic,
                 sizeof(gMqttState.test_topic) - 1U);
    gMqttState.test_topic[sizeof(gMqttState.test_topic) - 1U] = '\0';
}

// ============================================================================
// JSON 解析辅助函数
// ============================================================================
/**
 * @brief 查找 JSON 中某个 key 的 value 起始位置
 * 
 * @param json JSON 字符串
 * @param key 要查找的 key
 * @return const char* value 的起始位置，失败返回 nullptr
 * 
 * 查找过程：
 * 1. 查找 "key" 的位置
 * 2. 找到后面的冒号 :
 * 3. 跳过空白字符（空格、制表符、换行）
 * 4. 返回 value 的起始位置
 * 
 * 示例：
 * @code
 * json = {"method":"set","id":"123"}
 * key = "method"
 * 返回指向 "set" 的指针（包括引号）
 * @endcode
 */
static const char *json_find_value_start(const char *json, const char *key)
{
    if (json == nullptr || key == nullptr) {
        return nullptr;
    }

    // 步骤 1: 构造 "key" 模式并查找
    char pattern[48] = {};
    std::snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *keyPos = std::strstr(json, pattern);
    if (keyPos == nullptr) {
        return nullptr;
    }

    // 步骤 2: 找到冒号
    const char *colonPos = std::strchr(keyPos + std::strlen(pattern), ':');
    if (colonPos == nullptr) {
        return nullptr;
    }

    // 步骤 3: 跳过空白字符
    const char *valuePos = colonPos + 1;
    while (*valuePos == ' ' || *valuePos == '\t' || *valuePos == '\r' || *valuePos == '\n') {
        ++valuePos;
    }
    return valuePos;  // 步骤 4: 返回 value 起始位置
}

// 从 payload 中找到 params 对象的起始位置
static const char *json_find_params_object(const char *json)
{
    if (json == nullptr) {
        return nullptr;
    }

    const char *paramsPos = std::strstr(json, "\"params\"");
    if (paramsPos == nullptr) {
        return nullptr;
    }

    const char *colonPos = std::strchr(paramsPos + 8, ':');
    if (colonPos == nullptr) {
        return nullptr;
    }

    const char *bracePos = std::strchr(colonPos + 1, '{');
    if (bracePos == nullptr) {
        return nullptr;
    }

    return bracePos;
}

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
    return -1;
}

static bool json_extract_string(const char *json, const char *key, char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return false;
    }

    const char *valuePos = json_find_value_start(json, key);
    if (valuePos == nullptr || *valuePos != '"') {
        return false;
    }

    ++valuePos;
    size_t out = 0U;
    while (*valuePos != '\0' && out < buffer_size - 1U) {
        if (*valuePos == '"') {
            buffer[out] = '\0';
            return true;
        }

        if (*valuePos != '\\') {
            buffer[out++] = *valuePos++;
            continue;
        }

        ++valuePos;
        if (*valuePos == '\0') {
            break;
        }

        switch (*valuePos) {
        case '"':
        case '\\':
        case '/':
            buffer[out++] = *valuePos++;
            break;
        case 'b':
            buffer[out++] = '\b';
            ++valuePos;
            break;
        case 'f':
            buffer[out++] = '\f';
            ++valuePos;
            break;
        case 'n':
            buffer[out++] = '\n';
            ++valuePos;
            break;
        case 'r':
            buffer[out++] = '\r';
            ++valuePos;
            break;
        case 't':
            buffer[out++] = '\t';
            ++valuePos;
            break;
        case 'u': {
            const int h0 = json_hex_nibble(valuePos[1]);
            const int h1 = json_hex_nibble(valuePos[2]);
            const int h2 = json_hex_nibble(valuePos[3]);
            const int h3 = json_hex_nibble(valuePos[4]);
            if (h0 < 0 || h1 < 0 || h2 < 0 || h3 < 0) {
                buffer[out] = '\0';
                return false;
            }
            const unsigned codepoint = (unsigned)((h0 << 12) | (h1 << 8) | (h2 << 4) | h3);
            if (codepoint <= 0x7FU) {
                buffer[out++] = static_cast<char>(codepoint);
            } else if (codepoint <= 0x7FFU && out + 1U < buffer_size - 1U) {
                buffer[out++] = static_cast<char>(0xC0U | (codepoint >> 6));
                buffer[out++] = static_cast<char>(0x80U | (codepoint & 0x3FU));
            } else if (out + 2U < buffer_size - 1U) {
                buffer[out++] = static_cast<char>(0xE0U | (codepoint >> 12));
                buffer[out++] = static_cast<char>(0x80U | ((codepoint >> 6) & 0x3FU));
                buffer[out++] = static_cast<char>(0x80U | (codepoint & 0x3FU));
            }
            valuePos += 5;
            break;
        }
        default:
            buffer[out++] = *valuePos++;
            break;
        }
    }
    buffer[out] = '\0';
    return false;
}

static bool json_extract_bool(const char *json, const char *key, bool *value)
{
    if (value == nullptr) {
        return false;
    }

    const char *valuePos = json_find_value_start(json, key);
    if (valuePos == nullptr) {
        return false;
    }

    if (std::strncmp(valuePos, "true", 4) == 0) {
        *value = true;
        return true;
    }
    if (std::strncmp(valuePos, "false", 5) == 0) {
        *value = false;
        return true;
    }
    return false;
}

static bool json_extract_ulong(const char *json, const char *key, unsigned long *value)
{
    if (value == nullptr) {
        return false;
    }

    const char *valuePos = json_find_value_start(json, key);
    if (valuePos == nullptr) {
        return false;
    }

    char *endPtr = nullptr;
    const unsigned long parsed = std::strtoul(valuePos, &endPtr, 10);
    if (endPtr == valuePos) {
        return false;
    }
    *value = parsed;
    return true;
}

static void mqtt_manager_build_reply_topic(const char *request_topic,
                                           char *buffer,
                                           size_t buffer_size)
{
    if (request_topic == nullptr || buffer == nullptr || buffer_size == 0U) {
        return;
    }

    std::strncpy(buffer, request_topic, buffer_size - 1U);
    buffer[buffer_size - 1U] = '\0';

    char *commandPos = std::strstr(buffer, "/c/");
    if (commandPos != nullptr) {
        commandPos[1] = 's';
    }

    const size_t used = std::strlen(buffer);
    if (used < buffer_size - 1U) {
        std::snprintf(buffer + used, buffer_size - used, "_reply");
    }
}

static bool mqtt_manager_publish_reply(const char *request_topic,
                                       const char *request_id,
                                       const char *request_method,
                                       int code,
                                       const char *params_json)
{
    char replyTopic[160] = {};
    mqtt_manager_build_reply_topic(request_topic, replyTopic, sizeof(replyTopic));
    if (replyTopic[0] == '\0') {
        return false;
    }

    char payload[512] = {};
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"id\":\"%s\",\"method\":\"%s_reply\",\"code\":%d,\"params\":%s}",
                  request_id != nullptr ? request_id : "",
                  request_method != nullptr ? request_method : "",
                  code,
                  params_json != nullptr ? params_json : "{}");

    return mqtt_manager_publish_text(replyTopic, payload, 1, 0);
}

/**
 * @brief 处理 MQTT 下行消息
 * 
 * @param topic MQTT 主题
 * @param payload MQTT 消息内容
 * 
 * 功能说明：
 * - 解析下行消息的 method 和 id
 * - 根据 method 类型分发处理
 * - 支持平台回执消息（_reply）
 * 
 * 支持的下行方法（5 类）：
 * 
 * 1. thing.service.property.set
 *    - 功能：设置设备参数
 *    - 参数：continuousSendEnabled, continuousSendInterval
 *    - 响应：回复 success:true
 * 
 * 2. thing.service.property.get
 *    - 功能：查询设备状态
 *    - 响应：回复当前传感器数据（心率、呼吸率、存在等）
 * 
 * 3. thing.service.*（其他服务）
 *    - 功能：其他服务调用
 *    - 响应：回复 success:true（通用响应）
 * 
 * 4. *_reply（平台回执）
 *    - 功能：平台对上报数据的确认
 *    - 类型：thing.event.property.post_reply 等
 *    - 处理：仅记录日志，不回复
 *    - 判断：method 包含 "_reply" 字符串
 * 
 * 5. 不支持的方法
 *    - 处理：打印警告日志
 * 
 * 处理流程：
 * 1. 提取 method 和 id 字段
 * 2. 判断 method 类型
 * 3. 执行对应处理逻辑
 * 4. 回复或记录日志
 * 
 * 注意：
 * - reply 消息不需要回复
 * - 仅看 code==0 确认上报成功即可
 * - 避免将 reply 误判为 unsupported
 */
static void mqtt_manager_handle_downlink(const char *topic, const char *payload)
{
    // 步骤 1: 提取 method 和 id 字段
    char method[64] = {};
    char id[32] = {};
    (void)json_extract_string(payload, "method", method, sizeof(method));
    (void)json_extract_string(payload, "id", id, sizeof(id));

    // 类型 4: 平台回执消息（_reply）
    // 处理 thing.event.property.post_reply 等回执消息
    if (std::strstr(method, "_reply") != nullptr) {
        ESP_LOGI(TAG, "mqtt reply received method=%s", method);
        return;  // 回执消息不需要处理，直接返回
    }

    // 类型 1: thing.service.property.set
    if (std::strcmp(method, "thing.service.property.set") == 0) {
        bool enabled = tasks_manager_get_continuous_send_enabled();
        unsigned long interval = tasks_manager_get_continuous_send_interval_ms();
        
        // 从 params 对象中读取字段（和 Arduino 一致）
        const char *paramsStart = json_find_params_object(payload);
        if (paramsStart != nullptr) {
            (void)json_extract_bool(paramsStart, "continuousSendEnabled", &enabled);
            (void)json_extract_ulong(paramsStart, "continuousSendInterval", &interval);
        }
        
        tasks_manager_set_continuous_send(enabled, interval);
        (void)mqtt_manager_publish_reply(topic, id, method, 0, "{\"success\":true}");
        return;
    }

    // 类型 2: thing.service.property.get
    if (std::strcmp(method, "thing.service.property.get") == 0) {
        const SensorData data = radar_manager_get_sensor_data();
        char params[384] = {};
        std::snprintf(params,
                      sizeof(params),
                      "{\"heartRate\":%.1f,\"breathingRate\":%.1f,"
                      "\"personDetected\":%u,\"humanActivity\":%u,\"humanDistance\":%u,"
                      "\"sleepState\":%u,\"continuousSendEnabled\":%s,"
                      "\"continuousSendInterval\":%lu}",
                      data.heart_rate,
                      data.breath_rate,
                      (unsigned)data.presence,
                      (unsigned)data.motion,
                      (unsigned)data.distance,
                      (unsigned)data.sleep_state,
                      tasks_manager_get_continuous_send_enabled() ? "true" : "false",
                      tasks_manager_get_continuous_send_interval_ms());
        (void)mqtt_manager_publish_reply(topic, id, method, 0, params);
        return;
    }

    // 类型 3: thing.service.*（其他服务）
    if (std::strncmp(method, "thing.service.", 14) == 0) {
        (void)mqtt_manager_publish_reply(topic, id, method, 0, "{\"success\":true}");
        return;
    }

    // 类型 5: 不支持的方法
    ESP_LOGW(TAG, "unsupported mqtt method=%s", method);
}

static void mqtt_manager_set_state_internal(MqttManagerState state)
{
    gMqttState.state = state;
    gMqttState.last_state_change_ms = radar_now_ms();
}

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        gMqttState.connected = 1U;
        mqtt_manager_set_state_internal(MQTT_MANAGER_CONNECTED);
        ESP_LOGI(TAG, "mqtt connected");
        if (gMqttSubscribeTopic[0] != '\0') {
            esp_mqtt_client_subscribe(gMqttClient, gMqttSubscribeTopic, 1);
            ESP_LOGI(TAG, "mqtt subscribed topic=%s", gMqttSubscribeTopic);
        }
        (void)mqtt_manager_publish_online();
        break;
    case MQTT_EVENT_DISCONNECTED:
        gMqttState.connected = 0U;
        mqtt_manager_set_state_internal(MQTT_MANAGER_DISCONNECTED);
        ESP_LOGW(TAG, "mqtt disconnected");
        break;
    case MQTT_EVENT_ERROR:
        gMqttState.connected = 0U;
        mqtt_manager_set_state_internal(MQTT_MANAGER_ERROR);
        ESP_LOGE(TAG, "mqtt error");
        break;
    case MQTT_EVENT_DATA: {
        char topic[160] = {};
        char payload[1024] = {};
        const int topicLen = event->topic_len < (int)(sizeof(topic) - 1U)
                                 ? event->topic_len
                                 : (int)(sizeof(topic) - 1U);
        const int payloadLen = event->data_len < (int)(sizeof(payload) - 1U)
                                   ? event->data_len
                                   : (int)(sizeof(payload) - 1U);
        std::memcpy(topic, event->topic, static_cast<size_t>(topicLen));
        topic[topicLen] = '\0';
        std::memcpy(payload, event->data, static_cast<size_t>(payloadLen));
        payload[payloadLen] = '\0';
        ESP_LOGI(TAG, "mqtt downlink topic=%s payload=%s", topic, payload);
        mqtt_manager_handle_downlink(topic, payload);
        break;
    }
    default:
        break;
    }
}

static void mqtt_connect_task(void *parameter)
{
    (void)parameter;

    while (true) {
        // 安全清理：在错误/断开状态下回收 client 实例
        if (gMqttClient != nullptr &&
            (gMqttState.state == MQTT_MANAGER_ERROR ||
             gMqttState.state == MQTT_MANAGER_DISCONNECTED) &&
            gMqttState.connected == 0U) {
            ESP_LOGI(TAG, "mqtt client recycle state=%d", (int)gMqttState.state);
            esp_mqtt_client_destroy(gMqttClient);
            gMqttClient = nullptr;
        }

        const bool shouldConnect =
            gMqttState.initialized != 0U &&
            gMqttState.configured != 0U &&
            gMqttState.connected == 0U &&
            wifi_manager_is_connected() &&
            gMqttState.state != MQTT_MANAGER_CONNECTING;

        if (shouldConnect) {
            if (gMqttClient == nullptr) {
                mqtt_manager_refresh_enjoy_iot_identity();
                esp_mqtt_client_config_t mqttCfg = {};
                mqttCfg.broker.address.uri = gMqttState.uri;
                mqttCfg.credentials.client_id = gMqttState.client_id;
                mqttCfg.credentials.username = gMqttUsername;
                mqttCfg.credentials.authentication.password = gMqttPassword;
                mqttCfg.buffer.size = 1024;
                gMqttClient = esp_mqtt_client_init(&mqttCfg);
                if (gMqttClient == nullptr) {
                    mqtt_manager_set_state_internal(MQTT_MANAGER_ERROR);
                    ESP_LOGE(TAG, "failed to init mqtt client");
                    vTaskDelay(pdMS_TO_TICKS(MQTT_RECONNECT_INTERVAL_MS));
                    continue;
                }
                esp_mqtt_client_register_event(gMqttClient,
                                               MQTT_EVENT_ANY,
                                               mqtt_event_handler,
                                               nullptr);
            }

            gMqttState.reconnect_attempts += 1U;
            mqtt_manager_set_state_internal(MQTT_MANAGER_CONNECTING);
            ESP_LOGI(TAG, "mqtt connect attempt uri=%s", gMqttState.uri);

            esp_err_t err = esp_mqtt_client_start(gMqttClient);
            if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
                mqtt_manager_set_state_internal(MQTT_MANAGER_ERROR);
                ESP_LOGE(TAG, "esp_mqtt_client_start failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MQTT_RECONNECT_INTERVAL_MS));
    }
}

bool mqtt_manager_init(void)
{
    if (gMqttState.initialized) {
        return true;
    }

    gMqttState.initialized = 1U;
    gMqttState.connected = 0U;
    gMqttState.configured = 0U;
    gMqttState.reconnect_attempts = 0U;
    gMqttState.uri[0] = '\0';
    gMqttState.client_id[0] = '\0';
    gMqttState.test_topic[0] = '\0';
    mqtt_manager_set_state_internal(MQTT_MANAGER_INITIALIZED);
    system_state_set_mqtt_ready(1U);

    if (gMqttConnectTaskHandle == nullptr) {
        xTaskCreate(mqtt_connect_task,
                    "mqtt_connect_task",
                    4096,
                    nullptr,
                    2,
                    &gMqttConnectTaskHandle);
    }

    ESP_LOGI(TAG, "minimal mqtt manager initialized");
    return true;
}

bool mqtt_manager_configure(const char *uri, const char *client_id)
{
    if (uri == nullptr || uri[0] == '\0') {
        ESP_LOGW(TAG, "mqtt config rejected: empty uri");
        return false;
    }

    std::strncpy(gMqttState.uri, uri, sizeof(gMqttState.uri) - 1U);
    gMqttState.uri[sizeof(gMqttState.uri) - 1U] = '\0';
    (void)client_id;
    mqtt_manager_refresh_enjoy_iot_identity();
    gMqttState.configured = 1U;

    if (gMqttClient != nullptr) {
        esp_mqtt_client_stop(gMqttClient);
        esp_mqtt_client_destroy(gMqttClient);
        gMqttClient = nullptr;
    }

    mqtt_manager_set_state_internal(MQTT_MANAGER_INITIALIZED);
    ESP_LOGI(TAG,
             "mqtt configured uri=%s client_id=%s username=%s sub=%s post=%s",
             gMqttState.uri,
             gMqttState.client_id,
             gMqttUsername,
             gMqttSubscribeTopic,
             gMqttPropertyPostTopic);
    // 打印完整鉴权三元组，用于和 Arduino 版逐字符对比
    ESP_LOGI(TAG,
             "mqtt auth client_id=%s username=%s password=%s",
             gMqttState.client_id,
             gMqttUsername,
             gMqttPassword);
    return true;
}

bool mqtt_manager_is_initialized(void)
{
    return gMqttState.initialized != 0U;
}

bool mqtt_manager_is_connected(void)
{
    return gMqttState.connected != 0U;
}

MqttManagerSnapshot mqtt_manager_get_snapshot(void)
{
    return gMqttState;
}

bool mqtt_manager_refresh_identity(void)
{
    if (gMqttState.initialized == 0U) {
        return false;
    }

    if (gMqttClient != nullptr) {
        (void)esp_mqtt_client_stop(gMqttClient);
        esp_mqtt_client_destroy(gMqttClient);
        gMqttClient = nullptr;
    }

    gMqttState.connected = 0U;
    mqtt_manager_refresh_enjoy_iot_identity();
    mqtt_manager_set_state_internal(gMqttState.configured != 0U ? MQTT_MANAGER_INITIALIZED
                                                                 : MQTT_MANAGER_DISCONNECTED);
    ESP_LOGI(TAG,
             "mqtt identity refreshed client_id=%s username=%s sub=%s post=%s",
             gMqttState.client_id,
             gMqttUsername,
             gMqttSubscribeTopic,
             gMqttPropertyPostTopic);
    return true;
}

bool mqtt_manager_publish_text(const char *topic, const char *payload, int qos, int retain)
{
    if (gMqttClient == nullptr || !mqtt_manager_is_connected() || topic == nullptr || payload == nullptr) {
        return false;
    }

    const int msgId = esp_mqtt_client_publish(gMqttClient, topic, payload, 0, qos, retain);
    return msgId >= 0;
}

bool mqtt_manager_publish_online(void)
{
    if (gMqttPropertyPostTopic[0] == '\0') {
        return false;
    }

    const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();

    const SensorData data = radar_manager_get_sensor_data();

    // 使用 device_sn 作为 deviceId（和 Arduino 一致）
    static char deviceId[32] = {};
    mqtt_manager_make_payload_device_id(deviceId, sizeof(deviceId));

    static char payload[384];
    payload[0] = '\0';
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"id\":\"%lu\",\"method\":\"thing.event.property.post\","
                  "\"params\":{\"deviceId\":\"%s\",\"reportType\":\"heartbeat\","
                  "\"personDetected\":0,\"heartbeat\":1,\"timestamp\":%llu,"
                  "\"sleepState\":%u,\"noOneAlert\":%u,\"wifiIP\":\"%s\"}}",
                  static_cast<unsigned long>(gMqttMessageId++),
                  deviceId,
                  (unsigned long long)radar_now_ms(),
                  static_cast<unsigned>(data.sleep_state),
                  static_cast<unsigned>(data.no_one_alert),
                  wifiSnapshot.ip);

    const bool ok = mqtt_manager_publish_text(gMqttPropertyPostTopic, payload, 1, 0);
    ESP_LOGI(TAG, "mqtt heartbeat publish topic=%s ok=%s", gMqttPropertyPostTopic, ok ? "yes" : "no");
    return ok;
}

bool mqtt_manager_publish_radar_snapshot(const RadarReportSnapshot *snapshot)
{
    if (snapshot == nullptr) {
        return false;
    }

    if (gMqttPropertyPostTopic[0] == '\0') {
        return false;
    }

    const bool isNoOne = snapshot->presence == 0U ||
                         (snapshot->heart_rate == 0.0f && snapshot->breath_rate == 0.0f);
    const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();

    // 使用 device_sn 作为 deviceId（和 Arduino 一致）
    static char deviceId[32] = {};
    mqtt_manager_make_payload_device_id(deviceId, sizeof(deviceId));

    static char payload[1024];
    payload[0] = '\0';
    if (isNoOne) {
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"id\":\"%lu\",\"method\":\"thing.event.property.post\","
                      "\"params\":{\"deviceId\":\"%s\",\"reportType\":\"heartbeat\","
                      "\"personDetected\":0,\"heartbeat\":1,\"timestamp\":%llu,"
                      "\"sleepState\":%u,\"noOneAlert\":%u,\"wifiIP\":\"%s\"}}",
                      static_cast<unsigned long>(gMqttMessageId++),
                      deviceId,
                      (unsigned long long)radar_now_ms(),
                      static_cast<unsigned>(snapshot->sleep_state),
                      static_cast<unsigned>(snapshot->no_one_alert),
                      wifiSnapshot.ip);
    } else {
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"id\":\"%lu\",\"method\":\"thing.event.property.post\","
                      "\"params\":{\"deviceId\":\"%s\",\"reportType\":\"daily\","
                      "\"heartRate\":%.1f,\"breathingRate\":%.1f,"
                      "\"personDetected\":%u,\"humanActivity\":%u,\"humanDistance\":%u,"
                      "\"sleepState\":%u,\"bodyMovement\":%u,\"abnormalState\":%u,"
                      "\"humanPositionX\":%d,\"humanPositionY\":%d,\"humanPositionZ\":%d,"
                      "\"heartbeatWaveform\":%d,\"breathingWaveform\":%d,"
                      "\"bedStatus\":%u,\"struggleAlert\":%u,\"noOneAlert\":%u,"
                      "\"wifiIP\":\"%s\"}}",
                      static_cast<unsigned long>(gMqttMessageId++),
                      deviceId,
                      snapshot->heart_rate,
                      snapshot->breath_rate,
                      static_cast<unsigned>(snapshot->presence),
                      static_cast<unsigned>(snapshot->motion),
                      static_cast<unsigned>(snapshot->distance),
                      static_cast<unsigned>(snapshot->sleep_state),
                      static_cast<unsigned>(snapshot->body_movement),
                      static_cast<unsigned>(snapshot->abnormal_state),
                      (int)snapshot->pos_x,
                      (int)snapshot->pos_y,
                      (int)snapshot->pos_z,
                      snapshot->heartbeat_waveform,
                      snapshot->breathing_waveform,
                      static_cast<unsigned>(snapshot->bed_status),
                      static_cast<unsigned>(snapshot->struggle_alert),
                      static_cast<unsigned>(snapshot->no_one_alert),
                      wifiSnapshot.ip);
    }

    const bool ok = mqtt_manager_publish_text(gMqttPropertyPostTopic, payload, 1, 0);
    ESP_LOGI(TAG, "mqtt property publish topic=%s ok=%s", gMqttPropertyPostTopic, ok ? "yes" : "no");
    return ok;
}

bool mqtt_manager_publish_sleep_snapshot(const RadarReportSnapshot *snapshot)
{
    if (snapshot == nullptr) {
        return false;
    }

    if (gMqttPropertyPostTopic[0] == '\0') {
        return false;
    }

    if (snapshot->sleep_state != 0U && snapshot->sleep_state != 1U) {
        ESP_LOGD(TAG, "skip mqtt sleep publish, sleep_state=%u", (unsigned)snapshot->sleep_state);
        return false;
    }

    // 使用 device_sn 作为 deviceId（和 Arduino 一致）
    static char deviceId[32] = {};
    mqtt_manager_make_payload_device_id(deviceId, sizeof(deviceId));

    static char sleepPayload[1024];
    sleepPayload[0] = '\0';
    std::snprintf(sleepPayload,
                  sizeof(sleepPayload),
                  "{\"id\":\"%lu\",\"method\":\"thing.event.property.post\","
                  "\"params\":{\"deviceId\":\"%s\",\"reportType\":\"sleep\","
                  "\"sleepQualityScore\":%u,\"sleepQualityGrade\":%u,"
                  "\"totalSleepDuration\":%u,\"awakeDurationRatio\":%u,"
                  "\"lightSleepRatio\":%u,\"deepSleepRatio\":%u,"
                  "\"outOfBedDuration\":%u,\"outOfBedCount\":%u,\"turnCount\":%u,"
                  "\"avgBreathingRate\":%u,\"avgHeartRate\":%u,\"apneaCount\":%u,"
                  "\"abnormalState\":%u,\"bodyMovement\":%u,\"breathStatus\":%u,"
                  "\"sleepState\":%u,\"largeMoveRatio\":%u,\"smallMoveRatio\":%u,"
                  "\"struggleAlert\":%u,\"noOneAlert\":%u,\"awakeDuration\":%u,"
                  "\"lightSleepDuration\":%u,\"deepSleepDuration\":%u}}",
                  static_cast<unsigned long>(gMqttMessageId++),
                  deviceId,
                  static_cast<unsigned>(snapshot->sleep_score),
                  static_cast<unsigned>(snapshot->sleep_grade),
                  static_cast<unsigned>(snapshot->sleep_total_time),
                  static_cast<unsigned>(snapshot->awake_ratio),
                  static_cast<unsigned>(snapshot->light_sleep_ratio),
                  static_cast<unsigned>(snapshot->deep_sleep_ratio),
                  static_cast<unsigned>(snapshot->bed_Out_Time),
                  static_cast<unsigned>(snapshot->turn_count),
                  static_cast<unsigned>(snapshot->turnover_count),
                  static_cast<unsigned>(snapshot->avg_breath_rate),
                  static_cast<unsigned>(snapshot->avg_heart_rate),
                  static_cast<unsigned>(snapshot->apnea_count),
                  static_cast<unsigned>(snapshot->abnormal_state),
                  static_cast<unsigned>(snapshot->body_movement),
                  static_cast<unsigned>(snapshot->breath_status),
                  static_cast<unsigned>(snapshot->sleep_state),
                  static_cast<unsigned>(snapshot->large_move_ratio),
                  static_cast<unsigned>(snapshot->small_move_ratio),
                  static_cast<unsigned>(snapshot->struggle_alert),
                  static_cast<unsigned>(snapshot->no_one_alert),
                  static_cast<unsigned>(snapshot->awake_time),
                  static_cast<unsigned>(snapshot->light_sleep_time),
                  static_cast<unsigned>(snapshot->deep_sleep_time));

    const bool ok = mqtt_manager_publish_text(gMqttPropertyPostTopic, sleepPayload, 1, 0);
    ESP_LOGI(TAG, "mqtt sleep publish topic=%s ok=%s", gMqttPropertyPostTopic, ok ? "yes" : "no");
    return ok;
}

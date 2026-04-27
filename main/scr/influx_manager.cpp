/**
 * @file influx_manager.cpp
 * @brief InfluxDB 数据上报管理模块
 * 
 * 功能概述：
 * - 将雷达监测数据上报到 InfluxDB 时序数据库
 * - 支持日常数据（心率、呼吸率等）和睡眠数据上报
 * - 实现失败冷却机制，避免频繁重试导致系统不稳定
 * 
 * 核心特性：
 * 1. HTTP POST 请求：使用 ESP HTTP 客户端发送 Line Protocol 格式数据
 * 2. 失败冷却：连续失败 3 次后进入 10 秒冷却，不主动重启 WiFi
 * 3. WiFi 检查：发送前检查 WiFi 连接状态和信号强度
 * 4. 数据验证：上报前验证数据有效性，避免无效数据入库
 * 
 * InfluxDB 配置：
 * - Host: APP_INFLUXDB_HOST
 * - Port: APP_INFLUXDB_PORT
 * - Org: APP_INFLUXDB_ORG
 * - Bucket: APP_INFLUXDB_BUCKET
 * - Token: APP_INFLUXDB_TOKEN
 * 
 * 数据格式（Line Protocol）：
 * measurement,tag=field value
 * 例：daily_data,deviceId=1001,dataType=daily heartRate=72.5,breathingRate=16.2,...
 */
// ============================================================================
// 头文件包含和全局常量
// ============================================================================
#include "influx_manager.h"  // InfluxDB 管理器接口声明

#include "app_config.h"       // 应用配置（InfluxDB 连接参数）
#include "device_identity.h"  // 设备身份（获取 deviceId）
#include "esp_err.h"          // ESP32 错误码定义
#include "esp_http_client.h"  // ESP HTTP 客户端
#include "esp_log.h"          // ESP32 日志系统
#include "radar_platform.h"   // 平台接口（radar_now_ms, radar_sleep_ms）
#include "wifi_manager.h"     // WiFi 管理器（检查连接状态）
#include <cstdio>             // C 标准输入输出（snprintf）
#include <cstring>            // C 字符串操作（strlen）

static const char *TAG = "influx_manager";  // 日志标签

// 失败处理常量
static constexpr uint32_t kRefusedCooldownMs = 10000U;      // 失败冷却时间（10 秒）
static constexpr uint8_t kInfluxFailReconnectThreshold = 3U;  // 触发冷却的失败次数阈值

// 全局状态变量
static uint64_t gLastRefusedTimeMs = 0U;  // 上次失败时间戳
static uint8_t gInfluxFailCount = 0U;     // 连续失败计数器

// ============================================================================
// 内部辅助函数
// ============================================================================
/**
 * @brief 检查是否处于失败冷却期
 * 
 * @return true 正在冷却中，应跳过发送
 * @return false 可以正常发送
 * 
 * 逻辑：
 * - 如果 gLastRefusedTimeMs 为 0，表示从未失败，返回 false
 * - 如果当前时间 - 上次失败时间 < 10 秒，返回 true（冷却中）
 * - 否则返回 false（冷却结束）
 */
static bool influx_is_in_refused_cooldown(void)
{
    if (gLastRefusedTimeMs == 0U) {
        return false;
    }
    return (radar_now_ms() - gLastRefusedTimeMs) < kRefusedCooldownMs;
}

/**
 * @brief 检查 WiFi 是否可用
 * 
 * @return true WiFi 可用，可以尝试发送
 * @return false WiFi 不可用，跳过发送
 * 
 * 检查项目：
 * 1. WiFi 是否已连接
 * 2. 信号强度是否足够（RSSI >= -85 dBm）
 * 3. 是否已获取 IP 地址
 * 4. 是否处于失败冷却期
 * 
 * 注意：
 * - 此函数只读检查，不修改任何状态
 * - 失败时会记录详细原因（ESP_LOGD/W）
 */
static bool influx_wifi_ready(void)
{
    // 获取 WiFi 状态快照
    const WiFiManagerSnapshot snapshot = wifi_manager_get_snapshot();
    
    // 检查 1: WiFi 连接状态
    if (!snapshot.connected) {
        ESP_LOGD(TAG, "WiFi not connected, skip InfluxDB send");
        return false;
    }
    
    // 检查 2: 信号强度
    if (snapshot.rssi < -85) {
        ESP_LOGW(TAG, "WiFi signal too weak (%d dBm), skip InfluxDB send", (int)snapshot.rssi);
        return false;
    }
    
    // 检查 3: IP 地址
    if (snapshot.ip[0] == '\0') {
        ESP_LOGW(TAG, "WiFi has no IP, skip InfluxDB send");
        return false;
    }
    
    // 检查 4: 失败冷却
    if (influx_is_in_refused_cooldown()) {
        ESP_LOGW(TAG, "InfluxDB failure cooldown active, skip send");
        return false;
    }
    
    return true;  // 所有检查通过
}

/**
 * @brief 发送 Line Protocol 数据到 InfluxDB
 * 
 * @param line Line Protocol 格式的字符串
 * @param timeout_ms HTTP 请求超时时间（毫秒）
 * @return true 发送成功（HTTP 204）
 * @return false 发送失败
 * 
 * 处理流程：
 * 1. 参数验证（空指针检查）
 * 2. WiFi 状态检查
 * 3. 构建完整 URL（包含 org、bucket 等参数）
 * 4. 配置 HTTP 客户端
 * 5. 设置请求头（Authorization、Content-Type 等）
 * 6. 执行 HTTP POST 请求
 * 7. 检查响应状态码
 * 8. 失败处理（计数、冷却）
 * 
 * URL 格式：
 * http://host:port/api/v2/write?org=xxx&bucket=xxx
 * 
 * 请求头：
 * - Authorization: Token <token>
 * - Content-Type: text/plain; charset=utf-8
 * - Connection: close
 * - Host: <host>
 * 
 * 失败处理策略（模仿 Arduino）：
 * - 失败时增加 gInfluxFailCount
 * - 记录失败时间到 gLastRefusedTimeMs
 * - 达到阈值（3 次）后重置计数器，进入冷却
 * - 冷却期间不主动重启 WiFi
 */
static bool influx_post_line(const char *line, uint32_t timeout_ms)
{
    // 步骤 1: 参数验证
    if (line == nullptr || line[0] == '\0') {
        return false;
    }
    
    // 步骤 2: WiFi 检查
    if (!influx_wifi_ready()) {
        return false;
    }

    // 步骤 3: 构建 URL
    char url[192] = {};
    std::snprintf(url,
                  sizeof(url),
                  "http://%s:%u/api/v2/write?org=%s&bucket=%s",
                  APP_INFLUXDB_HOST,
                  (unsigned)APP_INFLUXDB_PORT,
                  APP_INFLUXDB_ORG,
                  APP_INFLUXDB_BUCKET);

    // 步骤 4: 配置 HTTP 客户端
    esp_http_client_config_t config = {};
    config.url = url;
    config.method = HTTP_METHOD_POST;
    config.timeout_ms = (int)timeout_ms;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        ESP_LOGE(TAG, "http client init failed");
        return false;
    }

    // 步骤 5: 设置请求头
    char auth[192] = {};
    std::snprintf(auth, sizeof(auth), "Token %s", APP_INFLUXDB_TOKEN);
    esp_http_client_set_header(client, "Authorization", auth);
    esp_http_client_set_header(client, "Content-Type", "text/plain; charset=utf-8");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Host", APP_INFLUXDB_HOST);
    esp_http_client_set_post_field(client, line, std::strlen(line));

    // 步骤 6: 执行请求
    const esp_err_t err = esp_http_client_perform(client);
    const int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    // 步骤 7: 检查响应
    if (err == ESP_OK && status == 204) {
        gInfluxFailCount = 0U;  // 成功则重置失败计数
        ESP_LOGI(TAG, "InfluxDB write ok: %s", line);
        return true;
    }

    // 步骤 8: 失败处理
    ESP_LOGW(TAG, "InfluxDB write failed err=%s status=%d", esp_err_to_name(err), status);

    // 模仿 Arduino：失败先进入冷却，不主动动 WiFi
    if (err != ESP_OK || status == 0) {
        if (gInfluxFailCount < 255U) {
            ++gInfluxFailCount;  // 增加失败计数
        }
        gLastRefusedTimeMs = radar_now_ms();  // 记录失败时间

        // 达到阈值，进入冷却
        if (gInfluxFailCount >= kInfluxFailReconnectThreshold) {
            ESP_LOGW(TAG,
                     "InfluxDB failures reached %u, enter cooldown and keep WiFi unchanged",
                     (unsigned)gInfluxFailCount);
            gInfluxFailCount = 0U;  // 重置计数器
        }
    }

    return false;
}

// ============================================================================
// 公共接口函数
// ============================================================================
/**
 * @brief 发送日常监测数据到 InfluxDB
 * 
 * @param snapshot 雷达报告快照（包含心率、呼吸率、位置等）
 * @return true 发送成功
 * @return false 发送失败（数据无效、WiFi 不可用、HTTP 错误等）
 * 
 * 数据验证：
 * 1. 空指针检查
 * 2. 心率/呼吸率 > 0
 * 3. 心率 <= 200 bpm（排除异常值）
 * 4. 呼吸率 <= 50 bpm（排除异常值）
 * 
 * Line Protocol 格式：
 * @code
 * daily_data,deviceId=1001,dataType=daily 
 * heartRate=72.5,breathingRate=16.2,
 * personDetected=1,humanActivity=0,humanDistance=120,
 * sleepState=0,humanPositionX=50,humanPositionY=30,humanPositionZ=10,
 * heartbeatWaveform=65,breathingWaveform=42,abnormalState=0,
 * bedStatus=0,struggleAlert=0,noOneAlert=0
 * @endcode
 * 
 * 字段说明：
 * - measurement: daily_data
 * - tags: deviceId, dataType
 * - fields: 心率、呼吸率、存在检测、活动状态、距离、睡眠状态、位置坐标等
 * 
 * 注意：
 * - 使用静态缓冲区（512 字节）避免栈溢出
 * - 超时时间 15 秒（适合网络较慢的情况）
 */
bool influx_manager_send_daily_data(const RadarReportSnapshot *snapshot)
{
    // 步骤 1: 空指针检查
    if (snapshot == nullptr) {
        return false;
    }
    
    // 步骤 2: 数据有效性检查（> 0）
    if (snapshot->heart_rate <= 0.0f || snapshot->breath_rate <= 0.0f) {
        ESP_LOGD(TAG, "heart/breath empty, skip daily InfluxDB send");
        return false;
    }
    
    // 步骤 3: 数据范围检查（排除异常值）
    if (snapshot->heart_rate > 200.0f || snapshot->breath_rate > 50.0f) {
        ESP_LOGW(TAG, "invalid vital data hr=%.1f rr=%.1f, skip InfluxDB send",
                 snapshot->heart_rate,
                 snapshot->breath_rate);
        return false;
    }

    // 步骤 4: 获取设备身份
    const DeviceIdentity identity = device_identity_get();
    
    // 步骤 5: 构建 Line Protocol 字符串
    static char line[512];
    line[0] = '\0';
    std::snprintf(line,
                  sizeof(line),
                  "daily_data,deviceId=%u,dataType=daily "
                  "heartRate=%.1f,breathingRate=%.1f,"
                  "personDetected=%ui,humanActivity=%ui,humanDistance=%ui,"
                  "sleepState=%ui,humanPositionX=%di,humanPositionY=%di,humanPositionZ=%di,"
                  "heartbeatWaveform=%di,breathingWaveform=%di,abnormalState=%ui,"
                  "bedStatus=%ui,struggleAlert=%ui,noOneAlert=%ui",
                  (unsigned)identity.device_id,
                  snapshot->heart_rate,
                  snapshot->breath_rate,
                  (unsigned)snapshot->presence,
                  (unsigned)snapshot->motion,
                  (unsigned)snapshot->distance,
                  (unsigned)snapshot->sleep_state,
                  (int)snapshot->pos_x,
                  (int)snapshot->pos_y,
                  (int)snapshot->pos_z,
                  snapshot->heartbeat_waveform,
                  snapshot->breathing_waveform,
                  (unsigned)snapshot->abnormal_state,
                  (unsigned)snapshot->bed_status,
                  (unsigned)snapshot->struggle_alert,
                  (unsigned)snapshot->no_one_alert);

    // 步骤 6: 发送数据（超时 15 秒）
    return influx_post_line(line, 15000U);
}

/**
 * @brief 发送睡眠数据到 InfluxDB
 * 
 * @param data 传感器数据结构（包含睡眠评分、时长、各阶段等）
 * @return true 发送成功
 * @return false 发送失败（数据无效、WiFi 不可用、HTTP 错误等）
 * 
 * 数据验证：
 * 1. 空指针检查
 * 2. 睡眠总时长 > 0（排除未入睡情况）
 * 
 * Line Protocol 格式：
 * @code
 * sleep_data,deviceId=1001,dataType=sleep
 * sleepQualityScore=85,sleepQualityGrade=1,totalSleepDuration=420,
 * awakeDurationRatio=5,lightSleepRatio=55,deepSleepRatio=40,
 * outOfBedDuration=0,outOfBedCount=1,turnCount=15,
 * avgBreathingRate=16,avgHeartRate=68,apneaCount=2,
 * abnormalState=0,bodyMovement=3,breathStatus=1,sleepState=1,
 * largeMoveRatio=2,smallMoveRatio=8,struggleAlert=0,noOneAlert=0,
 * awakeDuration=20,lightSleepDuration=230,deepSleepDuration=170
 * @endcode
 * 
 * 字段说明：
 * - measurement: sleep_data
 * - tags: deviceId, dataType
 * - fields: 睡眠质量评分、时长、各阶段比例、离床次数、呼吸/心率、体动等
 * 
 * 重试机制：
 * - 最多重试 3 次
 * - 每次重试间隔 500ms
 * - 任意一次成功即返回 true
 * 
 * 注意：
 * - 使用静态缓冲区（768 字节）避免栈溢出
 * - 超时时间 5 秒（快速失败）
 * - 重试机制提高成功率
 */
bool influx_manager_send_sleep_data(const SensorData *data)
{
    // 步骤 1: 空指针检查
    if (data == nullptr) {
        return false;
    }
    
    // 步骤 2: 睡眠时长检查（未入睡则跳过）
    if (data->sleep_total_time == 0U) {
        return false;
    }

    // 步骤 3: 获取设备身份
    const DeviceIdentity identity = device_identity_get();
    
    // 步骤 4: 构建 Line Protocol 字符串
    static char line[768];
    line[0] = '\0';
    std::snprintf(line,
                  sizeof(line),
                  "sleep_data,deviceId=%u,dataType=sleep "
                  "sleepQualityScore=%ui,sleepQualityGrade=%ui,totalSleepDuration=%ui,"
                  "awakeDurationRatio=%ui,lightSleepRatio=%ui,deepSleepRatio=%ui,"
                  "outOfBedDuration=%ui,outOfBedCount=%ui,turnCount=%ui,"
                  "avgBreathingRate=%ui,avgHeartRate=%ui,apneaCount=%ui,"
                  "abnormalState=%ui,bodyMovement=%ui,breathStatus=%ui,sleepState=%ui,"
                  "largeMoveRatio=%ui,smallMoveRatio=%ui,struggleAlert=%ui,noOneAlert=%ui,"
                  "awakeDuration=%ui,lightSleepDuration=%ui,deepSleepDuration=%ui",
                  (unsigned)identity.device_id,
                  (unsigned)data->sleep_score,
                  (unsigned)data->sleep_grade,
                  (unsigned)data->sleep_total_time,
                  (unsigned)data->awake_ratio,
                  (unsigned)data->light_sleep_ratio,
                  (unsigned)data->deep_sleep_ratio,
                  (unsigned)data->bed_Out_Time,
                  (unsigned)data->turn_count,
                  (unsigned)data->turnover_count,
                  (unsigned)data->avg_breath_rate,
                  (unsigned)data->avg_heart_rate,
                  (unsigned)data->apnea_count,
                  (unsigned)data->abnormal_state,
                  (unsigned)data->body_movement,
                  (unsigned)data->breath_status,
                  (unsigned)data->sleep_state,
                  (unsigned)data->large_move_ratio,
                  (unsigned)data->small_move_ratio,
                  (unsigned)data->struggle_alert,
                  (unsigned)data->no_one_alert,
                  (unsigned)data->awake_time,
                  (unsigned)data->light_sleep_time,
                  (unsigned)data->deep_sleep_time);

    // 步骤 5: 重试机制（最多 3 次）
    for (uint8_t retry = 0U; retry < 3U; ++retry) {
        if (retry > 0U) {
            radar_sleep_ms(500);  // 重试前等待 500ms
        }
        if (influx_post_line(line, 5000U)) {  // 超时 5 秒
            return true;
        }
    }

    return false;  // 3 次重试均失败
}

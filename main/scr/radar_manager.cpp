/**
 * @file radar_manager.cpp
 * @brief 毫米波雷达管理器模块
 * 
 * 功能概述：
 * - 管理 R60ABD1 毫米波雷达传感器
 * - 提供人体存在、心率、呼吸率等生命体征数据
 * - 支持睡眠监测、体动检测、异常状态告警
 * - 提供数据新鲜度、变化检测、快照管理功能
 * 
 * 核心特性：
 * 1. 雷达初始化：UART 通信、传感器配置
 * 2. 数据缓存：全局 sensorData 结构体存储最新数据
 * 3. 快照管理：支持多次消费，避免数据丢失
 * 4. 变化检测：智能判断数据是否有意义变化
 * 5. 设备标识：基于 MAC 地址和 device_sn 生成唯一标识
 * 
 * 数据结构：
 * - SensorData: 完整传感器数据（心率、呼吸、睡眠、体动等）
 * - RadarVitalsSnapshot: 生命体征快照
 * - RadarChangeFlags: 变化标志位
 * - RadarReportSnapshot: 上报用快照（包含有效性标志）
 * 
 * 应用场景：
 * - 睡眠监测：睡眠状态、离床检测、翻身计数
 * - 生命体征：心率、呼吸率实时监测
 * - 异常告警：挣扎告警、无人告警、异常状态
 */
// ============================================================================
// 头文件包含和全局变量
// ============================================================================
#include "radar_manager.h"  // 雷达管理器接口声明

#include <cstdio>          // C 标准输入输出（snprintf）
#include <cstring>         // C 字符串操作（strlen, memcpy）

#include "device_identity.h"  // 设备身份（生成设备标识）
#include "esp_log.h"          // ESP32 日志系统
#include "esp_mac.h"          // ESP32 MAC 地址读取
#include "radar_platform.h"   // 平台接口（radar_now_ms）
#include "radar_uart.h"       // 雷达 UART 通信
#include "system_state.h"     // 系统状态管理

static const char *TAG = "radar_manager";  // 日志标签

// 初始化标志
static bool radarManagerInitialized = false;

// 上次消费的快照（用于变化检测）
static RadarVitalsSnapshot lastConsumedSnapshot = {};
static bool hasConsumedSnapshot = false;

// 全局传感器数据（由 radar_uart.cpp 更新）
SensorData sensorData = {};

// ============================================================================
// 内部辅助函数
// ============================================================================
/**
 * @brief 计算浮点数绝对值
 * 
 * @param value 输入值
 * @return float 绝对值
 */
static float abs_float(float value)
{
    return value >= 0.0f ? value : -value;
}

/**
 * @brief 计算 CRC32 校验和
 * 
 * @param data 数据缓冲区
 * @param length 数据长度
 * @return uint32_t CRC32 值
 * 
 * 算法：
 * - 标准 CRC32（多项式 0xEDB88320）
 * - 初始值：0xFFFFFFFF
 * - 最终取反
 * 
 * 用途：
 * - 生成设备唯一标识（radar_manager_generate_device_hash）
 */
static uint32_t calculate_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ (0xEDB88320U & static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1U))));
        }
    }
    return ~crc;
}

// ============================================================================
// 初始化和状态查询
// ============================================================================
/**
 * @brief 初始化雷达管理器
 * 
 * @return true 初始化成功
 * @return false 初始化失败（UART 初始化失败）
 * 
 * 处理流程：
 * 1. 检查是否已初始化（避免重复）
 * 2. 初始化 UART 通信接口
 * 3. 启动 UART 接收任务（后台解析雷达数据）
 * 4. 配置 R60ABD1 雷达传感器
 * 5. 设置初始化标志
 * 6. 更新系统状态（radar_ready, phase）
 * 
 * 注意：
 * - 此函数在系统启动时由 main.cpp 调用
 * - UART 任务会在后台持续更新 sensorData
 */
bool initRadarManager(void)
{
    // 步骤 1: 避免重复初始化
    if (radarManagerInitialized) {
        return true;
    }

    ESP_LOGI(TAG, "initializing radar manager");

    // 步骤 2: 初始化 UART
    if (!radar_uart_init()) {
        ESP_LOGE(TAG, "radar uart init failed");
        system_state_set_error(1U);
        return false;
    }

    // 步骤 3: 启动 UART 接收任务
    radar_uart_start_task();
    
    // 步骤 4: 配置雷达传感器
    initR60ABD1();

    // 步骤 5-6: 设置标志和系统状态
    radarManagerInitialized = true;
    system_state_set_radar_ready(1U);
    system_state_set_phase(SYSTEM_PHASE_RADAR_READY);
    ESP_LOGI(TAG, "radar manager initialized");
    return true;
}

/**
 * @brief 检查雷达管理器是否已初始化
 * 
 * @return true 已初始化
 * @return false 未初始化
 */
bool radar_manager_is_initialized(void)
{
    return radarManagerInitialized;
}

/**
 * @brief 获取传感器完整数据（按值返回）
 * 
 * @return SensorData 传感器数据结构（按值复制）
 * 
 * 返回内容：
 * - 生命体征：心率、呼吸率
 * - 存在检测：presence、motion
 * - 睡眠监测：sleep_state、sleep_score 等
 * - 体动检测：body_movement、large_move_ratio 等
 * - 位置信息：pos_x、pos_y、pos_z
 * - 波形数据：heartbeat_waveform、breathing_waveform
 * - 告警信息：struggle_alert、no_one_alert
 * 
 * 注意：
 * - 按值返回，线程安全
 * - 数据由 UART 后台任务持续更新
 */
SensorData radar_manager_get_sensor_data(void)
{
    return sensorData;
}

/**
 * @brief 检查是否检测到人体
 * 
 * @return true 检测到人体（presence != 0）
 * @return false 未检测到人体
 */
bool radar_manager_has_person(void)
{
    return sensorData.presence != 0U;
}

/**
 * @brief 检查床铺是否被占用
 * 
 * @return true 床铺被占用（有人或睡眠中）
 * @return false 床铺空闲
 * 
 * 判断条件：
 * - presence != 0（检测到人体）
 * - sleep_state > 0（睡眠状态）
 */
bool radar_manager_is_bed_occupied(void)
{
    return sensorData.presence != 0U || sensorData.sleep_state > 0U;
}

/**
 * @brief 检查是否有有效生命体征数据
 * 
 * @return true 有心率或呼吸率数据（至少一个有效）
 * @return false 数据无效
 */
bool radar_manager_has_valid_vitals(void)
{
    return sensorData.heart_valid != 0U || sensorData.breath_valid != 0U;
}

/**
 * @brief 检查是否有新鲜样本
 * 
 * @param max_age_ms 最大允许年龄（毫秒）
 * @return true 样本年龄 <= max_age_ms
 * @return false 样本太老或无样本
 * 
 * 判断条件：
 * - last_update_ms != 0（有样本）
 * - now - last_update_ms <= max_age_ms（年龄足够小）
 * 
 * 用途：
 * - 确保上报的数据是最新的
 * - 避免上报过期数据
 */
bool radar_manager_has_fresh_sample(uint32_t max_age_ms)
{
    if (sensorData.last_update_ms == 0) {
        return false;
    }

    const uint64_t nowMs = radar_now_ms();
    return nowMs >= sensorData.last_update_ms &&
           (nowMs - sensorData.last_update_ms) <= max_age_ms;
}

RadarVitalsSnapshot radar_manager_get_vitals_snapshot(void)
{
    RadarVitalsSnapshot snapshot = {};
    snapshot.heart_rate = sensorData.heart_rate;
    snapshot.breath_rate = sensorData.breath_rate;
    snapshot.presence = sensorData.presence;
    snapshot.motion = sensorData.motion;
    snapshot.sleep_state = sensorData.sleep_state;
    snapshot.body_movement = sensorData.body_movement;
    snapshot.breath_status = sensorData.breath_status;
    snapshot.distance = sensorData.distance;
    snapshot.abnormal_state = sensorData.abnormal_state;
    snapshot.avg_heart_rate = sensorData.avg_heart_rate;
    snapshot.avg_breath_rate = sensorData.avg_breath_rate;
    snapshot.turn_count = sensorData.turn_count;
    snapshot.large_move_ratio = sensorData.large_move_ratio;
    snapshot.small_move_ratio = sensorData.small_move_ratio;
    snapshot.sleep_total_time = sensorData.sleep_total_time;
    snapshot.sleep_score = sensorData.sleep_score;
    snapshot.sleep_grade = sensorData.sleep_grade;
    snapshot.awake_ratio = sensorData.awake_ratio;
    snapshot.light_sleep_ratio = sensorData.light_sleep_ratio;
    snapshot.deep_sleep_ratio = sensorData.deep_sleep_ratio;
    snapshot.awake_time = sensorData.awake_time;
    snapshot.light_sleep_time = sensorData.light_sleep_time;
    snapshot.deep_sleep_time = sensorData.deep_sleep_time;
    snapshot.turnover_count = sensorData.turnover_count;
    snapshot.struggle_alert = sensorData.struggle_alert;
    snapshot.no_one_alert = sensorData.no_one_alert;
    snapshot.bed_status = sensorData.bed_status;
    snapshot.bed_Out_Time = sensorData.bed_Out_Time;
    snapshot.apnea_count = sensorData.apnea_count;
    snapshot.pos_x = sensorData.pos_x;
    snapshot.pos_y = sensorData.pos_y;
    snapshot.pos_z = sensorData.pos_z;
    snapshot.heartbeat_waveform = sensorData.heartbeat_waveform;
    snapshot.breathing_waveform = sensorData.breathing_waveform;
    snapshot.last_update_ms = sensorData.last_update_ms;
    return snapshot;
}

RadarChangeFlags radar_manager_get_change_flags(void)
{
    RadarChangeFlags flags = {};
    const RadarVitalsSnapshot current = radar_manager_get_vitals_snapshot();

    if (!hasConsumedSnapshot) {
        if (current.last_update_ms != 0U) {
            flags.any_changed = 1U;
        }
        return flags;
    }

    flags.presence_changed = current.presence != lastConsumedSnapshot.presence;
    flags.motion_changed = current.motion != lastConsumedSnapshot.motion;
    flags.sleep_state_changed = current.sleep_state != lastConsumedSnapshot.sleep_state;
    flags.body_movement_changed =
        abs_float((float)current.body_movement - (float)lastConsumedSnapshot.body_movement) >= 5.0f;
    flags.distance_changed =
        current.distance > lastConsumedSnapshot.distance
            ? (current.distance - lastConsumedSnapshot.distance) >= 10U
            : (lastConsumedSnapshot.distance - current.distance) >= 10U;
    flags.heart_rate_changed =
        abs_float(current.heart_rate - lastConsumedSnapshot.heart_rate) >= 3.0f;
    flags.breath_rate_changed =
        abs_float(current.breath_rate - lastConsumedSnapshot.breath_rate) >= 2.0f;
    flags.abnormal_state_changed = current.abnormal_state != lastConsumedSnapshot.abnormal_state;

    flags.any_changed = flags.presence_changed ||
                        flags.motion_changed ||
                        flags.sleep_state_changed ||
                        flags.body_movement_changed ||
                        flags.distance_changed ||
                        flags.heart_rate_changed ||
                        flags.breath_rate_changed ||
                        flags.abnormal_state_changed;
    return flags;
}

bool radar_manager_has_meaningful_change(void)
{
    return radar_manager_get_change_flags().any_changed != 0U;
}

void radar_manager_mark_snapshot_consumed(void)
{
    lastConsumedSnapshot = radar_manager_get_vitals_snapshot();
    hasConsumedSnapshot = true;
}

/**
 * @brief 获取上报用快照
 * 
 * @param max_age_ms 最大允许样本年龄（毫秒）
 * @return RadarReportSnapshot 上报用快照
 * 
 * 返回内容（40+ 字段）：
 * - 完整传感器数据（心率、呼吸、睡眠、体动等）
 * - 有效性标志：has_person、bed_occupied、has_valid_vitals 等
 * - 时间信息：last_update_ms、sample_age_ms
 * 
 * 有效性判断：
 * - has_person: presence != 0
 * - bed_occupied: presence != 0 || sleep_state > 0
 * - has_valid_vitals: heart_valid || breath_valid
 * - has_fresh_sample: 样本年龄 <= max_age_ms
 * - has_meaningful_change: 有变化标志
 * 
 * 用途：
 * - 用于 MQTT/InfluxDB 数据上报
 * - 提供完整的数据和有效性标志
 */
RadarReportSnapshot radar_manager_get_report_snapshot(uint32_t max_age_ms)
{
    RadarReportSnapshot snapshot = {};
    const bool hasFreshSample = radar_manager_has_fresh_sample(max_age_ms);
    const bool hasPerson = radar_manager_has_person();
    const bool bedOccupied = radar_manager_is_bed_occupied();
    const bool hasValidVitals = radar_manager_has_valid_vitals();
    const bool hasMeaningfulChange = radar_manager_has_meaningful_change();

    // 复制传感器数据（所有字段）
    snapshot.heart_rate = sensorData.heart_rate;
    snapshot.breath_rate = sensorData.breath_rate;
    snapshot.distance = sensorData.distance;
    snapshot.presence = sensorData.presence;
    snapshot.motion = sensorData.motion;
    snapshot.sleep_state = sensorData.sleep_state;
    snapshot.body_movement = sensorData.body_movement;
    snapshot.breath_status = sensorData.breath_status;
    snapshot.abnormal_state = sensorData.abnormal_state;
    snapshot.heart_valid = sensorData.heart_valid;
    snapshot.breath_valid = sensorData.breath_valid;
    snapshot.avg_heart_rate = sensorData.avg_heart_rate;
    snapshot.avg_breath_rate = sensorData.avg_breath_rate;
    snapshot.turn_count = sensorData.turn_count;
    snapshot.large_move_ratio = sensorData.large_move_ratio;
    snapshot.small_move_ratio = sensorData.small_move_ratio;
    snapshot.sleep_total_time = sensorData.sleep_total_time;
    snapshot.sleep_score = sensorData.sleep_score;
    snapshot.sleep_grade = sensorData.sleep_grade;
    snapshot.awake_ratio = sensorData.awake_ratio;
    snapshot.light_sleep_ratio = sensorData.light_sleep_ratio;
    snapshot.deep_sleep_ratio = sensorData.deep_sleep_ratio;
    snapshot.awake_time = sensorData.awake_time;
    snapshot.light_sleep_time = sensorData.light_sleep_time;
    snapshot.deep_sleep_time = sensorData.deep_sleep_time;
    snapshot.turnover_count = sensorData.turnover_count;
    snapshot.struggle_alert = sensorData.struggle_alert;
    snapshot.no_one_alert = sensorData.no_one_alert;
    snapshot.bed_status = sensorData.bed_status;
    snapshot.bed_Out_Time = sensorData.bed_Out_Time;
    snapshot.apnea_count = sensorData.apnea_count;
    snapshot.pos_x = sensorData.pos_x;
    snapshot.pos_y = sensorData.pos_y;
    snapshot.pos_z = sensorData.pos_z;
    snapshot.heartbeat_waveform = sensorData.heartbeat_waveform;
    snapshot.breathing_waveform = sensorData.breathing_waveform;
    
    // 设置有效性标志
    snapshot.has_person = hasPerson ? 1U : 0U;
    snapshot.bed_occupied = bedOccupied ? 1U : 0U;
    snapshot.has_valid_vitals = hasValidVitals ? 1U : 0U;
    snapshot.has_fresh_sample = hasFreshSample ? 1U : 0U;
    snapshot.has_meaningful_change = hasMeaningfulChange ? 1U : 0U;
    snapshot.last_update_ms = sensorData.last_update_ms;

    // 计算样本年龄
    if (sensorData.last_update_ms != 0U) {
        const uint64_t nowMs = radar_now_ms();
        if (nowMs >= sensorData.last_update_ms) {
            snapshot.sample_age_ms = (uint32_t)(nowMs - sensorData.last_update_ms);
        }
    }

    return snapshot;
}

/**
 * @brief 获取设备 MAC 地址（字符串格式）
 * 
 * @param buffer 输出缓冲区（至少 18 字节）
 * @param buffer_size 缓冲区大小
 * @return true 获取成功
 * @return false 获取失败（buffer 太小或读取失败）
 * 
 * 输出格式：
 * - "AA:BB:CC:DD:EE:FF"（17 字符 + '\0'）
 * - 大写十六进制，冒号分隔
 * 
 * 用途：
 * - 生成设备唯一标识
 * - MQTT ClientID
 */
bool radar_manager_get_device_mac(char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size < 18U) {
        return false;
    }

    uint8_t mac[6] = {};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        buffer[0] = '\0';
        return false;
    }

    std::snprintf(buffer,
                  buffer_size,
                  "%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0],
                  mac[1],
                  mac[2],
                  mac[3],
                  mac[4],
                  mac[5]);
    return true;
}

/**
 * @brief 生成设备唯一标识（CRC32 哈希）
 * 
 * @return uint32_t 设备哈希值
 * 
 * 生成流程：
 * 1. 获取 MAC 地址字符串（"AA:BB:CC:DD:EE:FF"）
 * 2. 去除冒号（"AABBCCDDEEFF"）
 * 3. 获取 device_sn
 * 4. 拼接：hashInput = "SN{device_sn}|{compactMac}"
 * 5. 计算 CRC32：hash = CRC32(hashInput)
 * 
 * 示例：
 * - MAC: "AA:BB:CC:DD:EE:FF"
 * - device_sn: 1234567890123456789
 * - hashInput: "SN1234567890123456789|AABBCCDDEEFF"
 * - hash: 0x12345678
 * 
 * 用途：
 * - 生成 MQTT ClientID（当 device_sn 为 0 时）
 * - 设备唯一标识
 */
uint32_t radar_manager_generate_device_hash(void)
{
    char macStr[18] = {};
    if (!radar_manager_get_device_mac(macStr, sizeof(macStr))) {
        return 0U;
    }

    // 去除冒号
    char compactMac[13] = {};
    size_t outIndex = 0U;
    for (size_t i = 0; macStr[i] != '\0' && outIndex < (sizeof(compactMac) - 1U); ++i) {
        if (macStr[i] != ':') {
            compactMac[outIndex++] = macStr[i];
        }
    }
    compactMac[outIndex] = '\0';

    // 拼接哈希输入
    const DeviceIdentity identity = device_identity_get();
    char hashInput[96] = {};
    std::snprintf(hashInput,
                  sizeof(hashInput),
                  "SN%llu|%s",
                  static_cast<unsigned long long>(identity.device_sn),
                  compactMac);

    // 计算 CRC32
    const uint32_t hash = calculate_crc32(reinterpret_cast<const uint8_t *>(hashInput),
                                          std::strlen(hashInput));
    ESP_LOGI(TAG,
             "device hash input=%s hash=0x%08X",
             hashInput,
             static_cast<unsigned>(hash));
    return hash;
}

/**
 * @brief 获取设备名称
 * 
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @return true 获取成功
 * @return false 获取失败
 * 
 * 生成规则：
 * 1. 优先使用 device_sn："Radar_{device_sn}"
 * 2. 如果 device_sn 为 0，使用 MAC 地址："Radar_{compactMac}"
 * 
 * 示例：
 * - device_sn=123456 → "Radar_123456"
 * - MAC="AA:BB:CC:DD:EE:FF" → "Radar_AABBCCDDEEFF"
 * 
 * 用途：
 * - 设备显示名称
 * - 日志标识
 */
bool radar_manager_get_device_name(char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return false;
    }

    const DeviceIdentity identity = device_identity_get();
    
    // 规则 1: 优先使用 device_sn
    if (identity.device_sn != 0ULL) {
        std::snprintf(buffer,
                      buffer_size,
                      "Radar_%llu",
                      static_cast<unsigned long long>(identity.device_sn));
        return true;
    }

    // 规则 2: 使用 MAC 地址
    char macStr[18] = {};
    if (!radar_manager_get_device_mac(macStr, sizeof(macStr))) {
        buffer[0] = '\0';
        return false;
    }

    char compactMac[13] = {};
    size_t outIndex = 0U;
    for (size_t i = 0; macStr[i] != '\0' && outIndex < (sizeof(compactMac) - 1U); ++i) {
        if (macStr[i] != ':') {
            compactMac[outIndex++] = macStr[i];
        }
    }
    compactMac[outIndex] = '\0';

    std::snprintf(buffer, buffer_size, "Radar_%s", compactMac);
    return true;
}

/**
 * @brief 获取默认 MQTT ClientID
 * 
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @return true 获取成功
 * @return false 获取失败
 * 
 * 生成规则：
 * 1. 优先使用 device_sn："espidf-radar-{device_sn}"
 * 2. 如果 device_sn 为 0，使用设备哈希："espidf-radar-{hash}"
 * 
 * 示例：
 * - device_sn=123456 → "espidf-radar-123456"
 * - hash=0x12345678 → "espidf-radar-12345678"
 * 
 * 用途：
 * - MQTT 连接时的 ClientID
 * - 阿里云物联网平台设备标识
 */
bool radar_manager_get_default_mqtt_client_id(char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return false;
    }

    const DeviceIdentity identity = device_identity_get();
    
    // 规则 1: 优先使用 device_sn
    if (identity.device_sn != 0ULL) {
        std::snprintf(buffer,
                      buffer_size,
                      "espidf-radar-%llu",
                      static_cast<unsigned long long>(identity.device_sn));
        return true;
    }

    // 规则 2: 使用设备哈希
    const uint32_t hash = radar_manager_generate_device_hash();
    if (hash == 0U) {
        buffer[0] = '\0';
        return false;
    }

    std::snprintf(buffer, buffer_size, "espidf-radar-%08X", static_cast<unsigned>(hash));
    return true;
}

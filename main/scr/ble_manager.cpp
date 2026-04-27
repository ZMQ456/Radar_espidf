/**
 * @file ble_manager.cpp
 * @brief BLE（蓝牙低功耗）管理器实现文件
 * 
 * 功能概述：
 * - 管理 BLE 设备广播和连接
 * - 处理 GATT 服务器事件
 * - 收发 JSON 格式的文本数据
 * - 支持通知（Notify）功能
 * 
 * 主要特性：
 * - 自定义 128-bit 服务 UUID：a8c1e5c0-3d5d-4a9d-8d5e-7c8b6a4e2f1a
 * - 支持读、写、通知操作
 * - 自动管理广播数据和扫描响应
 * - 接收数据缓冲和 JSON 解析
 * - 发送数据分块处理（每块 20 字节）
 * 
 * 使用场景：
 * - 与手机小程序通信
 * - 配置设备参数（WiFi、设备 ID 等）
 * - 上传雷达数据
 */

#include "ble_manager.h"

// 标准库
#include <cstdio>
#include <cstring>

// ESP-IDF 组件
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 项目模块
#include "radar_manager.h"
#include "radar_platform.h"
#include "sdkconfig.h"
#include "system_state.h"

// 蓝牙相关头文件（仅在启用 BT 时包含）
#if CONFIG_BT_ENABLED
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#endif

// ==================== 日志标签 ====================
/// 用于 ESP_LOG* 系列宏，标识日志来源
static const char *TAG = "ble_manager";

// ==================== BLE 状态管理 ====================
/// 全局 BLE 状态快照，包含初始化、广播、连接、通知等状态
static BleManagerSnapshot gBleState = {};

// ==================== 接收数据缓冲 ====================
/// 已解析的完整 JSON 负载缓冲区（等待被消费）
static char gReceivedText[512] = {};
/// 标记是否有待处理的 JSON 数据
static bool gHasPendingText = false;
/// 原始接收缓冲区（存储未解析的数据片段）
static char gReceiveBuffer[512] = {};
/// 接收缓冲区中已使用的长度
static size_t gReceiveLength = 0U;
/// 最后一次接收到数据的时间戳（毫秒）
static uint64_t gLastReceiveTimeMs = 0U;

// ==================== 常量定义 ====================
/// BLE 通知数据包大小（20 字节）
/// 受限于 BLE MTU 和 GATT 属性最大长度
static constexpr size_t BLE_JSON_PACKET_SIZE = 20U;

/// BLE 接收超时时间（5000 毫秒 = 5 秒）
/// 超过此时间未收到完整 JSON，将强制处理缓冲区中的数据
static constexpr uint32_t BLE_RECEIVE_TIMEOUT_MS = 5000U;

/**
 * @brief 尝试从接收缓冲区提取完整的 JSON 负载
 * 
 * 关键规则：
 * - 如果 gHasPendingText 为 true（上一条 JSON 还未被消费），直接返回 false
 * - 只提取一条完整 JSON，即使缓冲区还有更多数据也不继续提取
 * - 提取成功后，将剩余数据前移，等待下次调用
 * 
 * 工作流程：
 * 1. 检查是否有待处理数据（有则直接返回）
 * 2. 检查缓冲区是否为空
 * 3. 查找第一个 '{' 字符
 * 4. 使用深度计数匹配对应的 '}' 字符（处理嵌套 JSON）
 * 5. 找到完整 JSON 后：
 *    - 复制到 gReceivedText 缓冲区
 *    - 标记 gHasPendingText = true
 *    - 清理接收缓冲区，将剩余数据前移
 * 6. 如果没找到完整 JSON，清理前面的无效数据
 * 
 * @return true - 成功提取到完整 JSON 负载
 * @return false - 未找到完整 JSON 或缓冲区为空或有待处理数据
 */
static bool ble_try_extract_complete_payload(void)
{
    // 关键：如果上一条完整 JSON 还没被消费，绝不再提取下一条
    if (gHasPendingText) {
        return false;
    }

    if (gReceiveLength == 0U) {
        return false;
    }

    char *openBrace = static_cast<char *>(std::memchr(gReceiveBuffer, '{', gReceiveLength));
    if (openBrace == nullptr) {
        return false;
    }

    const size_t openIndex = static_cast<size_t>(openBrace - gReceiveBuffer);
    int depth = 0;
    for (size_t i = openIndex; i < gReceiveLength; ++i) {
        if (gReceiveBuffer[i] == '{') {
            ++depth;
        } else if (gReceiveBuffer[i] == '}') {
            --depth;
            if (depth == 0) {
                const size_t payloadLen = i - openIndex + 1U;
                const size_t copyLen = payloadLen < (sizeof(gReceivedText) - 1U)
                                           ? payloadLen
                                           : (sizeof(gReceivedText) - 1U);
                std::memcpy(gReceivedText, gReceiveBuffer + openIndex, copyLen);
                gReceivedText[copyLen] = '\0';
                gHasPendingText = true;

                const size_t remaining = gReceiveLength - (i + 1U);
                if (remaining > 0U) {
                    std::memmove(gReceiveBuffer, gReceiveBuffer + i + 1U, remaining);
                }
                gReceiveLength = remaining;
                gReceiveBuffer[gReceiveLength] = '\0';
                return true;
            }
        }
    }

    if (openIndex > 0U) {
        const size_t remaining = gReceiveLength - openIndex;
        std::memmove(gReceiveBuffer, gReceiveBuffer + openIndex, remaining);
        gReceiveLength = remaining;
        gReceiveBuffer[gReceiveLength] = '\0';
    }

    return false;
}

/**
 * @brief 将接收到的 BLE 数据片段追加到接收缓冲区
 * 
 * 关键规则：
 * - 即使 gHasPendingText 为 true，新分片也可以继续进入 gReceiveBuffer
 * - 但只有在 gHasPendingText 为 false 时，才调用提取函数
 * 
 * 处理流程：
 * 1. 检查输入参数有效性
 * 2. 计算缓冲区剩余空间
 * 3. 复制数据到缓冲区（不超过剩余空间）
 * 4. 更新接收时间戳
 * 5. 如果数据过长，记录警告并丢弃多余数据
 * 6. 如果当前没有待处理命令，尝试提取完整的 JSON 负载
 * 
 * @param data - 接收到的数据指针
 * @param length - 数据长度
 */
static void ble_append_received_fragment(const uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0U) {
        return;
    }

    const size_t freeLen = (sizeof(gReceiveBuffer) - 1U) - gReceiveLength;
    const size_t copyLen = length < freeLen ? length : freeLen;
    if (copyLen > 0U) {
        std::memcpy(gReceiveBuffer + gReceiveLength, data, copyLen);
        gReceiveLength += copyLen;
        gReceiveBuffer[gReceiveLength] = '\0';
        gLastReceiveTimeMs = radar_now_ms();
    }

    if (copyLen < length) {
        ESP_LOGW(TAG, "BLE receive buffer full, dropped %u bytes", (unsigned)(length - copyLen));
    }

    // 关键：只有在没有待处理命令时，才尝试提取新 JSON
    if (!gHasPendingText) {
        (void)ble_try_extract_complete_payload();
    }
}

// ==================== BLE 组件内部全局变量（仅在启用 BT 时） ====================

/// BLE 应用 ID（用于 GATT 注册）
/// 0x55AA 是自定义标识符
static constexpr uint16_t BLE_APP_ID = 0x55AA;

/// CCCD 描述符 UUID（16-bit 标准 UUID）
/// 用于配置客户端特征（启用/禁用通知）
static constexpr uint16_t BLE_DESCRIPTOR_UUID = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

/// 制造商数据长度（9 字节）
/// 格式：[公司 ID(2)] + [设备类型 (1)] + [版本 (1)] + [保留 (1)] + [设备哈希 (4)]
static constexpr size_t BLE_MANUFACTURER_DATA_LEN = 9U;

// ==================== BLE 组件状态标志 ====================
/// 蓝牙控制器是否已就绪（完成初始化和启用）
static bool gBleControllerReady = false;
/// GATT 应用是否已注册
static bool gGattRegistered = false;
/// 主广告包数据是否已配置完成
static bool gAdvDataConfigured = false;
/// 扫描响应数据是否已配置完成
static bool gScanRspDataConfigured = false;
/// BLE 服务是否已创建完成
static bool gServiceCreated = false;
/// 当前 BLE MTU 值（默认 23 字节）
static uint16_t gCurrentMtu = 23;
// ==================== GATT 服务器句柄和连接信息 ====================
/// GATT 接口（由 GATTS 注册回调返回）
static esp_gatt_if_t gGattIf = ESP_GATT_IF_NONE;
/// 当前连接的 ID（由连接事件返回）
static uint16_t gConnectionId = 0U;
/// 服务句柄（服务创建后分配）
static uint16_t gServiceHandle = 0U;
/// 特征值句柄（特征值添加后分配）
static uint16_t gCharacteristicHandle = 0U;
/// 描述符句柄（描述符添加后分配）
static uint16_t gDescriptorHandle = 0U;
/// 当前特征值的长度
static uint16_t gAttributeValueLength = 0U;
// ==================== UUID 和数据缓冲区 ====================
/// 制造商数据缓冲区（当前未使用，已禁用）
static uint8_t gManufacturerData[BLE_MANUFACTURER_DATA_LEN] = {};

/// 服务 UUID（128-bit，小端字节序）
/// UUID: a8c1e5c0-3d5d-4a9d-8d5e-7c8b6a4e2f1a
/// 用于小程序过滤和发现设备
static uint8_t gServiceUuidBytes[ESP_UUID_LEN_128] = {
    0x1A, 0x2F, 0x4E, 0x6A, 0x8B, 0x7C, 0x5E, 0x8D,
    0x9D, 0x4A, 0x5D, 0x3D, 0xC0, 0xE5, 0xC1, 0xA8,
};

/// 特征值 UUID（128-bit，小端字节序）
/// UUID: be5b483e-36e1-4688-b7f5-ea07361b26a8
/// 用于数据传输的主特征值
static uint8_t gCharacteristicUuidBytes[ESP_UUID_LEN_128] = {
    0xA8, 0x26, 0x1B, 0x36, 0x07, 0xEA, 0xF5, 0xB7,
    0x88, 0x46, 0xE1, 0x36, 0x3E, 0x48, 0xB5, 0xBE,
};

/// 特征值属性缓冲区（存储当前特征值数据）
/// 最大 256 字节，用于暂存待发送或已接收的数据
static uint8_t gAttributeValue[256] = {};

/// BLE 广播参数配置
/// 包括广播间隔、类型、地址类型、信道映射、过滤策略等
static esp_ble_adv_params_t gAdvParams = {};

/**
 * @brief 从设备身份管理模块获取设备名并更新 BLE 状态
 * 
 * 工作流程：
 * 1. 从 radar_manager 获取设备名（最多 32 字符）
 * 2. 如果获取失败，使用默认名 "Radar_Unknown"
 * 3. 将设备名复制到 BLE 状态结构体
 * 4. 确保字符串以 null 结尾
 * 
 * @return true - 成功更新设备名
 */
static bool ble_update_device_name_from_identity(void)
{
    char deviceName[32] = {};
    if (!radar_manager_get_device_name(deviceName, sizeof(deviceName))) {
        std::strncpy(deviceName, "Radar_Unknown", sizeof(deviceName) - 1U);
        deviceName[sizeof(deviceName) - 1U] = '\0';
    }

    std::strncpy(gBleState.device_name, deviceName, sizeof(gBleState.device_name) - 1U);
    gBleState.device_name[sizeof(gBleState.device_name) - 1U] = '\0';
    return true;
}

/**
 * @brief 准备广播数据中的制造商数据字段
 * 
 * 数据格式（9 字节）：
 * - [0-1]: 公司标识符 (0xFFFF - 自定义)
 * - [2]: 设备类型标识 ('R' = Radar)
 * - [3]: 协议版本 (0x01)
 * - [4]: 保留位 (0x00)
 * - [5-8]: 设备哈希值（大端序）
 * 
 * 用途：
 * - 用于快速识别设备类型和身份
 * - 当前版本已禁用（不放入广播包）
 */
static void ble_prepare_advertising_payload(void)
{
    const uint32_t deviceHash = radar_manager_generate_device_hash();

    gManufacturerData[0] = 0xFF;
    gManufacturerData[1] = 0xFF;
    gManufacturerData[2] = 'R';
    gManufacturerData[3] = 0x01;
    gManufacturerData[4] = 0x00;
    gManufacturerData[5] = static_cast<uint8_t>((deviceHash >> 24) & 0xFFU);
    gManufacturerData[6] = static_cast<uint8_t>((deviceHash >> 16) & 0xFFU);
    gManufacturerData[7] = static_cast<uint8_t>((deviceHash >> 8) & 0xFFU);
    gManufacturerData[8] = static_cast<uint8_t>(deviceHash & 0xFFU);
}

/**
 * @brief 检查条件满足时启动 BLE 广播
 * 
 * 启动条件：
 * 1. 主广告包数据已配置完成 (gAdvDataConfigured)
 * 2. 扫描响应数据已配置完成 (gScanRspDataConfigured)
 * 3. BLE 服务已创建完成 (gServiceCreated)
 * 4. 当前没有客户端连接 (gBleState.connected == 0)
 * 
 * 如果条件满足，调用 esp_ble_gap_start_advertising 启动广播
 */
static void ble_start_advertising_if_ready(void)
{
    if (!gAdvDataConfigured || !gScanRspDataConfigured || !gServiceCreated || gBleState.connected != 0U) {
        return;
    }

    esp_err_t err = esp_ble_gap_start_advertising(&gAdvParams);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "BLE advertising requested name=%s", gBleState.device_name);
}

/**
 * @brief 配置 BLE 广播数据包和扫描响应数据
 * 
 * 广播策略：
 * - 主广告包：只包含 flags + 128-bit service UUID（用于被小程序过滤发现）
 * - 扫描响应：只包含设备名（节省主广播包空间，避免数据被挤掉）
 * 
 * 这样设计的原因：
 * 1. ESP32 的广播包有长度限制（31 字节）
 * 2. 如果主广告包塞入太多数据（如 manufacturer data），可能导致 service UUID 被截断
 * 3. 小程序使用 service UUID 进行过滤，必须确保 UUID 完整出现在主广告包中
 * 4. 设备名放在扫描响应中，只在主动扫描时返回，不占用主广播包空间
 */
static void ble_configure_advertising_payloads(void)
{
    // 准备广播数据（生成 manufacturer data 等）
    ble_prepare_advertising_payload();

    // ==================== 配置主广告包 ====================
    esp_ble_adv_data_t advData = {};
    advData.set_scan_rsp = false;           // 设置为广播数据（不是扫描响应）
    advData.include_name = false;           // 不在主广告包中包含设备名
    advData.include_txpower = false;        // 不包含发射功率
    advData.min_interval = 0x20;            // 最小广播间隔 (32 * 0.625ms = 20ms)
    advData.max_interval = 0x40;            // 最大广播间隔 (64 * 0.625ms = 40ms)
    advData.appearance = 0x00;              // 外观类型 (0x00 = 未知/通用)

    // 关键：主广告包优先放 128-bit service UUID，不塞 manufacturer data
    // 这是为了确保小程序能通过 service UUID 过滤发现设备
    advData.manufacturer_len = 0;                       // 厂商数据长度设为 0（不发送）
    advData.p_manufacturer_data = nullptr;              // 厂商数据指针设为空
    advData.service_data_len = 0;                       // 服务数据长度设为 0
    advData.p_service_data = nullptr;                   // 服务数据指针设为空
    advData.service_uuid_len = sizeof(gServiceUuidBytes); // 服务 UUID 长度 (16 字节 = 128-bit)
    advData.p_service_uuid = gServiceUuidBytes;         // 指向 128-bit UUID 数组
                                                        // UUID: a8c1e5c0-3d5d-4a9d-8d5e-7c8b6a4e2f1a
    advData.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
                                                        // 广播标志：
                                                        // - 通用可发现模式
                                                        // - 不支持经典蓝牙（仅 BLE）

    // 配置主广告包数据
    esp_err_t err = esp_ble_gap_config_adv_data(&advData);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config adv data failed: %s", esp_err_to_name(err));
    }

    // ==================== 配置扫描响应数据 ====================
    esp_ble_adv_data_t scanRspData = {};
    scanRspData.set_scan_rsp = true;        // 设置为扫描响应数据
    scanRspData.include_name = true;        // 在扫描响应中包含设备名
    scanRspData.include_txpower = false;    // 不包含发射功率
    scanRspData.min_interval = 0x20;        // 最小广播间隔
    scanRspData.max_interval = 0x40;        // 最大广播间隔
    scanRspData.appearance = 0x00;          // 外观类型

    // 扫描响应里只放设备名，不塞 UUID / manufacturer data
    // 原因：
    // 1. 主广告包空间有限，要优先保证 service UUID
    // 2. 设备名较长（最多 32 字节），放在扫描响应中更合适
    // 3. 扫描响应也有 31 字节限制，但只放设备名足够
    scanRspData.manufacturer_len = 0;       // 厂商数据长度设为 0
    scanRspData.p_manufacturer_data = nullptr; // 厂商数据指针设为空
    scanRspData.service_data_len = 0;       // 服务数据长度设为 0
    scanRspData.p_service_data = nullptr;   // 服务数据指针设为空
    scanRspData.service_uuid_len = 0;       // 服务 UUID 长度设为 0（已在主广告包中）
    scanRspData.p_service_uuid = nullptr;   // 服务 UUID 指针设为空
    scanRspData.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
                                            // 广播标志与主广告包一致

    // 配置扫描响应数据
    err = esp_ble_gap_config_adv_data(&scanRspData);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config scan response failed: %s", esp_err_to_name(err));
    }
}

#if CONFIG_BT_ENABLED
static void ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    (void)param;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        gAdvDataConfigured = true;
        ble_start_advertising_if_ready();
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        gScanRspDataConfigured = true;
        ble_start_advertising_if_ready();
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        gBleState.advertising = 1U;
        ESP_LOGI(TAG, "BLE advertising started name=%s", gBleState.device_name);
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        gBleState.advertising = 0U;
        ESP_LOGI(TAG, "BLE advertising stopped");
        break;
    default:
        break;
    }
}

static void ble_gatts_callback(esp_gatts_cb_event_t event,
                               esp_gatt_if_t gatts_if,
                               esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        gGattIf = gatts_if;

        esp_ble_gap_set_device_name(gBleState.device_name);
        ble_configure_advertising_payloads();

        esp_gatt_srvc_id_t serviceId = {};
        serviceId.is_primary = true;
        serviceId.id.inst_id = 0x00;
        serviceId.id.uuid.len = ESP_UUID_LEN_128;
        std::memcpy(serviceId.id.uuid.uuid.uuid128,
                    gServiceUuidBytes,
                    sizeof(serviceId.id.uuid.uuid.uuid128));
        esp_ble_gatts_create_service(gatts_if, &serviceId, 6U);
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        gServiceHandle = param->create.service_handle;
        gServiceCreated = true;
        esp_ble_gatts_start_service(gServiceHandle);

        esp_bt_uuid_t characteristicUuid = {};
        characteristicUuid.len = ESP_UUID_LEN_128;
        std::memcpy(characteristicUuid.uuid.uuid128,
                    gCharacteristicUuidBytes,
                    sizeof(characteristicUuid.uuid.uuid128));
        const esp_gatt_char_prop_t property =
            ESP_GATT_CHAR_PROP_BIT_READ |
            ESP_GATT_CHAR_PROP_BIT_WRITE |
            ESP_GATT_CHAR_PROP_BIT_NOTIFY;

        esp_ble_gatts_add_char(gServiceHandle,
                               &characteristicUuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               property,
                               nullptr,
                               nullptr);
        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        gCharacteristicHandle = param->add_char.attr_handle;

        esp_bt_uuid_t descriptorUuid = {};
        descriptorUuid.len = ESP_UUID_LEN_16;
        descriptorUuid.uuid.uuid16 = BLE_DESCRIPTOR_UUID;
        esp_ble_gatts_add_char_descr(gServiceHandle,
                                     &descriptorUuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     nullptr,
                                     nullptr);

        ble_start_advertising_if_ready();
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gDescriptorHandle = param->add_char_descr.attr_handle;
        break;

    case ESP_GATTS_CONNECT_EVT:
        gBleState.connected = 1U;
        gConnectionId = param->connect.conn_id;
        ESP_LOGI(TAG, "BLE client connected conn_id=%u", gConnectionId);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        gBleState.connected = 0U;
        gBleState.notify_enabled = 0U;
        gBleState.advertising = 0U;
        ESP_LOGI(TAG, "BLE client disconnected");
        ble_start_advertising_if_ready();
        break;

    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep) {
            if (param->write.handle == gDescriptorHandle && param->write.len >= 2U) {
                const uint16_t cccdValue =
                    static_cast<uint16_t>(param->write.value[1] << 8) |
                    static_cast<uint16_t>(param->write.value[0]);
                gBleState.notify_enabled = (cccdValue & 0x0001U) ? 1U : 0U;
                ESP_LOGI(TAG, "BLE notify %s",
                         gBleState.notify_enabled ? "enabled" : "disabled");
            } else if (param->write.handle == gCharacteristicHandle) {
                const size_t copyLen =
                    param->write.len < (sizeof(gAttributeValue) - 1U)
                        ? param->write.len
                        : (sizeof(gAttributeValue) - 1U);
                std::memcpy(gAttributeValue, param->write.value, copyLen);
                gAttributeValue[copyLen] = '\0';
                gAttributeValueLength = static_cast<uint16_t>(copyLen);
                ble_append_received_fragment(param->write.value, param->write.len);
                ESP_LOGI(TAG, "BLE write received: %s", reinterpret_cast<const char *>(gAttributeValue));
            }

            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if,
                                            param->write.conn_id,
                                            param->write.trans_id,
                                            ESP_GATT_OK,
                                            nullptr);
            }
        }
        break;

    case ESP_GATTS_MTU_EVT:
        // 接收 MTU 协商结果
        gCurrentMtu = param->mtu.mtu;
        ESP_LOGI(TAG, "BLE MTU updated: %u", (unsigned)gCurrentMtu);
        break;

    default:
        break;
    }
}
#endif

bool ble_manager_init(void)
{
    if (gBleState.initialized != 0U) {
        return true;
    }

    ble_update_device_name_from_identity();

#if !CONFIG_BT_ENABLED
    gBleState.initialized = 1U;
    gBleState.advertising = 0U;
    gBleState.connected = 0U;
    gBleState.notify_enabled = 0U;
    system_state_set_ble_ready(1U);
    ESP_LOGW(TAG, "BLE is not enabled in sdkconfig, skeleton loaded with name=%s", gBleState.device_name);
    return true;
#else
    if (!gBleControllerReady) {
        esp_bt_controller_config_t btCfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_bt_controller_init(&btCfg);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_bluedroid_init();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_bluedroid_enable();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(err));
            return false;
        }

        std::memset(&gAdvParams, 0, sizeof(gAdvParams));
        gAdvParams.adv_int_min = 0x20;
        gAdvParams.adv_int_max = 0x40;
        gAdvParams.adv_type = ADV_TYPE_IND;
        gAdvParams.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
        gAdvParams.channel_map = ADV_CHNL_ALL;
        gAdvParams.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

        esp_err_t errGap = esp_ble_gap_register_callback(ble_gap_callback);
        if (errGap != ESP_OK) {
            ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %s", esp_err_to_name(errGap));
            return false;
        }

        esp_err_t errGatt = esp_ble_gatts_register_callback(ble_gatts_callback);
        if (errGatt != ESP_OK) {
            ESP_LOGE(TAG, "esp_ble_gatts_register_callback failed: %s", esp_err_to_name(errGatt));
            return false;
        }

        gBleControllerReady = true;
    }

    if (!gGattRegistered) {
        esp_err_t err = esp_ble_gatts_app_register(BLE_APP_ID);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_ble_gatts_app_register failed: %s", esp_err_to_name(err));
            return false;
        }

        // 设置本地 MTU 为 247 字节（最大推荐值）
        esp_ble_gatt_set_local_mtu(247);
        gGattRegistered = true;
    }

    gBleState.initialized = 1U;
    gBleState.advertising = 0U;
    gBleState.connected = 0U;
    gBleState.notify_enabled = 0U;
    system_state_set_ble_ready(1U);
    ESP_LOGI(TAG, "BLE manager initialized name=%s", gBleState.device_name);
    return true;
#endif
}

bool ble_manager_refresh_advertising_data(void)
{
#if !CONFIG_BT_ENABLED
    ble_update_device_name_from_identity();
    return true;
#else
    if (gBleState.initialized == 0U || !gBleControllerReady) {
        ble_update_device_name_from_identity();
        return true;
    }

    ble_update_device_name_from_identity();
    esp_ble_gap_set_device_name(gBleState.device_name);

    if (gBleState.advertising != 0U) {
        const esp_err_t stopErr = esp_ble_gap_stop_advertising();
        if (stopErr != ESP_OK && stopErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "esp_ble_gap_stop_advertising failed before refresh: %s",
                     esp_err_to_name(stopErr));
        }
    }

    gAdvDataConfigured = false;
    gScanRspDataConfigured = false;
    ble_configure_advertising_payloads();
    ESP_LOGI(TAG, "BLE advertising data refreshed name=%s", gBleState.device_name);
    return true;
#endif
}

bool ble_manager_is_initialized(void)
{
    return gBleState.initialized != 0U;
}

bool ble_manager_is_client_connected(void)
{
    return gBleState.connected != 0U;
}

/**
 * @brief 通过 BLE 通知发送文本数据给客户端
 * 
 * 发送流程：
 * 1. 检查输入参数有效性
 * 2. 如果 BT 未启用，记录日志并返回失败
 * 3. 计算最大有效载荷（MTU - 3）
 * 4. 检查发送条件：
 *    - 客户端已连接
 *    - 通知已使能
 *    - GATT 接口有效
 *    - 特征值句柄有效
 * 5. 如果条件不满足，仅保存数据但不发送
 * 6. 如果条件满足：
 *    - 如果数据 <= maxPayload，一次发送
 *    - 如果数据 > maxPayload，分块发送（每块 maxPayload 字节）
 *    - 块间延时 10ms，避免缓冲区溢出
 * 7. 记录发送结果日志
 * 
 * @param payload - 要发送的文本数据（JSON 格式）
 * @return true - 全部数据发送成功
 * @return false - 发送失败或未发送
 */
bool ble_manager_send_text(const char *payload)
{
    if (payload == nullptr || payload[0] == '\0') {
        return false;
    }

#if !CONFIG_BT_ENABLED
    ESP_LOGW(TAG, "BLE send skipped because BT is disabled: %s", payload);
    return false;
#else
    const size_t payloadLen = std::strlen(payload);

    if (!gBleState.connected || !gBleState.notify_enabled ||
        gGattIf == ESP_GATT_IF_NONE || gCharacteristicHandle == 0U) {
        ESP_LOGI(TAG, "BLE payload staged without notify: %s", payload);
        return false;
    }

    // 计算最大有效载荷 = MTU - 3（ATT 协议头开销）
    size_t maxPayload = 20;  // 默认值（安全值）
    if (gCurrentMtu > 3) {
        maxPayload = gCurrentMtu - 3;
    }

    // 保护：避免异常值
    if (maxPayload == 0 || maxPayload > 244) {
        maxPayload = 20;
    }

    // 如果一包能发下，就不要分片
    if (payloadLen <= maxPayload) {
        esp_err_t err = esp_ble_gatts_send_indicate(
            gGattIf,
            gConnectionId,
            gCharacteristicHandle,
            static_cast<uint16_t>(payloadLen),
            reinterpret_cast<uint8_t *>(const_cast<char *>(payload)),
            false);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BLE notify failed: %s", esp_err_to_name(err));
            return false;
        }

        ESP_LOGI(TAG, "BLE notify sent: %s", payload);
        return true;
    }

    // 超过 MTU 再分片
    bool allChunksSent = true;

    for (size_t offset = 0U; offset < payloadLen; offset += maxPayload) {
        const size_t remaining = payloadLen - offset;
        const size_t chunkLen = (remaining < maxPayload) ? remaining : maxPayload;

        esp_err_t err = esp_ble_gatts_send_indicate(
            gGattIf,
            gConnectionId,
            gCharacteristicHandle,
            static_cast<uint16_t>(chunkLen),
            reinterpret_cast<uint8_t *>(const_cast<char *>(payload + offset)),
            false);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BLE chunk notify failed: %s", esp_err_to_name(err));
            allChunksSent = false;
            break;
        }

        // 分片间留一点时间
        if ((offset + chunkLen) < payloadLen) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    ESP_LOGI(TAG, "BLE notify %s: %s", allChunksSent ? "sent" : "partial", payload);
    return allChunksSent;
#endif
}

/**
 * @brief 消费（读取并清除）接收到的 BLE 文本数据
 * 
 * 关键规则：
 * - 只有在没有待处理命令时，才允许超时兜底处理
 * - 消费完一条 JSON 后，立即尝试从剩余缓冲中提取下一条（但只准备一条）
 * 
 * 处理逻辑：
 * 1. 如果没有待处理数据但缓冲区有数据且超时（5 秒）：
 *    - 将缓冲区数据作为待处理数据
 *    - 清空接收缓冲区
 *    - 记录超时日志
 * 2. 如果输出缓冲区无效或没有待处理数据，返回失败
 * 3. 复制待处理数据到输出缓冲区
 * 4. 清空待处理数据
 * 5. 消费完成后，立即尝试从剩余缓冲中提取下一条完整 JSON
 * 
 * 超时机制的作用：
 * - 防止数据碎片长期占用缓冲区
 * - 确保即使没有完整 JSON 也能处理数据
 * 
 * @param buffer - 输出缓冲区指针
 * @param buffer_size - 输出缓冲区大小
 * @return true - 成功复制数据到输出缓冲区
 * @return false - 没有数据可复制或参数无效
 */
bool ble_manager_consume_received_text(char *buffer, uint32_t buffer_size)
{
    // 超时兜底逻辑：只有在没有待处理命令时，才允许把"残余缓冲"按超时处理
    if (!gHasPendingText && gReceiveLength > 0U && gLastReceiveTimeMs != 0U &&
        (radar_now_ms() - gLastReceiveTimeMs) > BLE_RECEIVE_TIMEOUT_MS) {
        const size_t copyLen = gReceiveLength < (sizeof(gReceivedText) - 1U)
                                   ? gReceiveLength
                                   : (sizeof(gReceivedText) - 1U);
        std::memcpy(gReceivedText, gReceiveBuffer, copyLen);
        gReceivedText[copyLen] = '\0';
        gReceiveBuffer[0] = '\0';
        gReceiveLength = 0U;
        gHasPendingText = true;
        ESP_LOGW(TAG, "BLE receive timeout, forwarding partial payload");
    }

    if (buffer == nullptr || buffer_size == 0U || !gHasPendingText) {
        return false;
    }

    const size_t copyLen = std::strlen(gReceivedText) < (buffer_size - 1U)
                               ? std::strlen(gReceivedText)
                               : (buffer_size - 1U);
    std::memcpy(buffer, gReceivedText, copyLen);
    buffer[copyLen] = '\0';
    gReceivedText[0] = '\0';
    gHasPendingText = false;

    // 关键：消费完一条后，立即尝试从剩余缓冲中提取下一条（但只准备一条）
    (void)ble_try_extract_complete_payload();

    return true;
}

/**
 * @brief 获取 BLE 管理器的状态快照
 * 
 * 返回的状态包括：
 * - initialized: 是否已初始化
 * - advertising: 是否正在广播
 * - connected: 是否有客户端连接
 * - notify_enabled: 通知是否已使能
 * - device_name: 设备名
 * 
 * @return BleManagerSnapshot - BLE 状态快照结构体（按值返回）
 */
BleManagerSnapshot ble_manager_get_snapshot(void)
{
    return gBleState;
}

// ==================== 文件结束 ====================
// ble_manager.cpp 完成
// 
// 主要功能总结：
// 1. BLE 设备管理（初始化、广播、连接）
// 2. GATT 服务器事件处理
// 3. 数据收发（JSON 格式，分块处理）
// 4. 状态管理（全局状态快照）
// 
// 关键特性：
// - 自定义 128-bit UUID 用于设备发现
// - 优化的广播包结构（UUID 在主包，设备名在扫描响应）
// - 接收缓冲区管理和 JSON 解析
// - 发送数据分块（20 字节/块）
// - 超时机制处理数据碎片
// ====================

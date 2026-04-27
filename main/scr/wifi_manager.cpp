/**
 * @file wifi_manager.cpp
 * @brief WiFi 管理器模块
 * 
 * 功能概述：
 * - 管理 ESP32 的 WiFi 连接功能
 * - 提供 WiFi 扫描、连接、断开、重连功能
 * - 保存和加载 WiFi 凭证到 NVS 闪存
 * - 支持多个已保存网络的自动切换
 * - 后台自动重连任务
 * 
 * 核心功能：
 * 1. WiFi 连接管理：
 *    - 使用指定 SSID 和密码连接
 *    - 自动重连机制
 *    - 连接超时处理（10 秒）
 *    - 状态机保护（避免并发冲突）
 * 
 * 2. WiFi 网络扫描：
 *    - 主动扫描可用网络
 *    - 获取 SSID、RSSI、加密方式等信息
 *    - 支持隐藏网络
 * 
 * 3. 凭证管理：
 *    - 保存 WiFi 凭证到 NVS（最多 10 个）
 *    - 从 NVS 加载已保存凭证
 *    - 清除所有凭证
 *    - 索引式存储（wifi_0, pass_0 等）
 * 
 * 4. 自动重连：
 *    - 后台重连任务（5 秒间隔）
 *    - 断开后自动尝试重连
 *    - 避免在配置/扫描时重连
 * 
 * WiFi 状态机（7 种状态）：
 * 1. WIFI_MANAGER_IDLE: 空闲状态
 * 2. WIFI_MANAGER_INITIALIZED: 已初始化
 * 3. WIFI_MANAGER_SCANNING: 扫描中
 * 4. WIFI_MANAGER_CONNECTING: 连接中
 * 5. WIFI_MANAGER_CONFIGURING: 配置中
 * 6. WIFI_MANAGER_CONNECTED: 已连接
 * 7. WIFI_MANAGER_DISCONNECTED: 已断开
 * 8. WIFI_MANAGER_ERROR: 错误状态
 * 
 * 网络状态映射：
 * - CONNECTED → NET_CONNECTED
 * - SCANNING/CONNECTING/CONFIGURING → NET_CONNECTING
 * - DISCONNECTED/IDLE/INITIALIZED → NET_DISCONNECTED
 * - ERROR → NET_DISCONNECTED
 * 
 * 关键设计：
 * 1. 互斥锁保护：gWifiOperationMutex 保护 WiFi 操作
 * 2. 状态机保护：避免并发连接冲突
 * 3. ESP_ERR_WIFI_STATE 容忍：处理 WiFi 状态冲突
 * 4. 事件组同步：WIFI_CONNECTED_BIT、WIFI_FAIL_BIT
 * 5. 后台重连：gReconnectTaskHandle 定期尝试重连
 * 
 * 依赖模块：
 * - esp_netif: ESP32 网络接口
 * - esp_wifi: ESP32 WiFi 驱动
 * - nvs_flash: 非易失性存储
 * - system_state: 系统状态管理
 * - tasks_manager: 任务管理器
 */
#include "wifi_manager.h"

#include <cstring>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "radar_platform.h"
#include "system_state.h"
#include "tasks_manager.h"

// ============================================================================
// 全局变量和常量定义
// ============================================================================
static const char *TAG = "wifi_manager";  // 日志标签

// WiFi 状态快照（全局可见）
static WiFiManagerSnapshot gWifiState = {};

// 当前活动的 WiFi 凭证
static char gSsid[33] = {};      // SSID 缓冲区（最多 32 字节 + 1）
static char gPassword[65] = {};  // 密码缓冲区（最多 64 字节 + 1）

// 已保存的 WiFi 网络（最多 10 个）
static constexpr uint32_t WIFI_MANAGER_MAX_SAVED_NETWORKS = 10U;
typedef struct {
    char ssid[33];       // SSID（最多 32 字节）
    char password[65];   // 密码（最多 64 字节）
} SavedWiFiCredential;
static SavedWiFiCredential gSavedNetworks[WIFI_MANAGER_MAX_SAVED_NETWORKS] = {};
static uint32_t gSavedNetworkCount = 0U;  // 已保存网络数量

// ESP32 网络接口
static esp_netif_t *gStaNetif = nullptr;  // STA 模式网络接口

// WiFi 事件组
static EventGroupHandle_t gWifiEventGroup = nullptr;

// 初始化标志
static bool gEspNetifInitialized = false;    // ESP 网络接口已初始化
static bool gWifiDriverInitialized = false;  // WiFi 驱动已初始化
static bool gEventHandlersRegistered = false; // 事件处理器已注册

// 后台重连任务
static TaskHandle_t gReconnectTaskHandle = nullptr;

// 互斥锁（保护 WiFi 操作）
static SemaphoreHandle_t gWifiOperationMutex = nullptr;

// 操作标志
static volatile bool gManualConfigActive = false;  // 手动配置中
static volatile bool gScanInProgress = false;      // 扫描进行中
static volatile bool gWifiConnecting = false;      // WiFi 连接中

// 事件组位定义
static constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;  // 连接成功位
static constexpr EventBits_t WIFI_FAIL_BIT = BIT1;       // 连接失败位

// 时间参数
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;     // 连接超时（10 秒）
static constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 5000;   // 重连间隔（5 秒）

// RSSI 阈值
static constexpr int8_t WIFI_MIN_RSSI_THRESHOLD = -127;  // 最小信号强度

// NVS 存储键名
static const char *WIFI_NVS_NAMESPACE = "wifi_cfg";     // NVS 命名空间
static const char *WIFI_NVS_KEY_SSID = "ssid";          // SSID 键名
static const char *WIFI_NVS_KEY_PASS = "pass";          // 密码键名

static void wifi_manager_set_state_internal(WiFiManagerState state);

// ============================================================================
// 工具函数
// ============================================================================
/**
 * @brief 将 WiFi 认证模式转换为字符串
 * 
 * @param authmode WiFi 认证模式
 * @return const char* 对应的字符串描述
 * 
 * 支持的认证模式：
 * - WIFI_AUTH_OPEN: 开放网络（无加密）
 * - WIFI_AUTH_WEP: WEP 加密（已过时）
 * - WIFI_AUTH_WPA_PSK: WPA-PSK
 * - WIFI_AUTH_WPA2_PSK: WPA2-PSK
 * - WIFI_AUTH_WPA_WPA2_PSK: WPA/WPA2 混合
 * - WIFI_AUTH_WPA2_ENTERPRISE: WPA2 企业版
 * - WIFI_AUTH_WPA3_PSK: WPA3-PSK
 * - WIFI_AUTH_WPA2_WPA3_PSK: WPA2/WPA3 混合
 * 
 * 用途：
 * - WiFi 扫描结果输出
 * - 日志打印
 */
static const char *wifi_manager_authmode_to_string(wifi_auth_mode_t authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        return "open";
    case WIFI_AUTH_WEP:
        return "WEP";
    case WIFI_AUTH_WPA_PSK:
        return "WPA";
    case WIFI_AUTH_WPA2_PSK:
        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:
        return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE:
        return "WPA2-EAP";
    case WIFI_AUTH_WPA3_PSK:
        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:
        return "WPA2/WPA3";
    default:
        return "unknown";
    }
}

/**
 * @brief 在已保存的网络中查找密码
 * 
 * @param ssid 要查找的 SSID
 * @param password 输出密码缓冲区
 * @param password_size 缓冲区大小
 * @return true 找到密码
 * @return false 未找到或参数无效
 * 
 * 功能说明：
 * - 遍历已保存的网络列表
 * - 查找匹配的 SSID
 * - 复制对应的密码到输出缓冲区
 * 
 * 处理流程：
 * 1. 参数有效性检查
 * 2. 遍历 gSavedNetworks 数组
 * 3. 使用 strcmp 比较 SSID
 * 4. 找到匹配：复制密码，返回 true
 * 5. 遍历结束未找到：返回 false
 * 
 * 用途：
 * - 自动连接时获取密码
 * - 验证 SSID 是否已保存
 * 
 * 注意：
 * - 密码复制使用 strncpy（安全复制）
 * - 确保密码以 '\0' 结尾
 */
static bool wifi_manager_find_saved_password(const char *ssid, char *password, size_t password_size)
{
    // 步骤 1: 参数有效性检查
    if (ssid == nullptr || password == nullptr || password_size == 0U) {
        return false;
    }

    // 步骤 2: 遍历已保存的网络
    for (uint32_t i = 0; i < gSavedNetworkCount; ++i) {
        // 步骤 3: 比较 SSID
        if (std::strcmp(gSavedNetworks[i].ssid, ssid) == 0) {
            // 步骤 4: 复制密码
            std::strncpy(password, gSavedNetworks[i].password, password_size - 1U);
            password[password_size - 1U] = '\0';
            return true;
        }
    }

    // 步骤 5: 未找到
    return false;
}

/**
 * @brief 复制活动的 WiFi 凭证到全局缓冲区
 * 
 * @param ssid 输入 SSID
 * @param password 输入密码
 * 
 * 功能说明：
 * - 将 SSID 和密码复制到全局变量
 * - 同时更新 gWifiState.ssid
 * - 确保所有字符串以 '\0' 结尾
 * 
 * 处理流程：
 * 1. 复制 SSID 到 gSsid（最多 32 字节）
 * 2. 复制密码到 gPassword（最多 64 字节）
 * 3. 复制 SSID 到 gWifiState.ssid
 * 4. 确保所有字符串正确终止
 * 
 * 用途：
 * - 保存当前活动的连接凭证
 * - 供其他函数访问当前 SSID
 * 
 * 注意：
 * - 使用 strncpy 防止缓冲区溢出
 * - 手动添加终止符 '\0'
 */
static void wifi_manager_copy_active_credentials(const char *ssid, const char *password)
{
    // 步骤 1: 复制 SSID
    std::strncpy(gSsid, ssid, sizeof(gSsid) - 1U);
    gSsid[sizeof(gSsid) - 1U] = '\0';
    
    // 步骤 2: 复制密码
    std::strncpy(gPassword, password, sizeof(gPassword) - 1U);
    gPassword[sizeof(gPassword) - 1U] = '\0';
    
    // 步骤 3: 更新状态快照
    std::strncpy(gWifiState.ssid, gSsid, sizeof(gWifiState.ssid) - 1U);
    gWifiState.ssid[sizeof(gWifiState.ssid) - 1U] = '\0';
}

/**
 * @brief 生成索引式键名
 * 
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @param prefix 键名前缀
 * @param index 索引编号
 * 
 * 功能说明：
 * - 生成格式化的键名：前缀 + 索引
 * - 用于 NVS 存储的键名生成
 * 
 * 生成格式：
 * - "{prefix}{index}"
 * - 示例："wifi_0", "pass_1", "wifi_2"
 * 
 * 处理流程：
 * 1. 使用 snprintf 格式化字符串
 * 2. 确保字符串正确终止
 * 
 * 用途：
 * - 生成 NVS 存储键名
 * - 索引式访问已保存网络
 * 
 * 示例：
 * prefix = "wifi_", index = 0 → "wifi_0"
 * prefix = "pass_", index = 1 → "pass_1"
 * 
 * 注意：
 * - 缓冲区大小至少为 strlen(prefix) + 数字位数 + 1
 */
static void wifi_manager_make_indexed_key(char *buffer,
                                          size_t buffer_size,
                                          const char *prefix,
                                          uint32_t index)
{
    snprintf(buffer, buffer_size, "%s%u", prefix, (unsigned)index);
}

/**
 * @brief 使用指定的 SSID 和密码连接 WiFi
 * 
 * 关键设计：
 * 1. 状态机保护：避免并发连接冲突
 * 2. ESP_ERR_WIFI_STATE 容忍：处理 WiFi 状态冲突
 * 3. 互斥锁保护：确保 WiFi 操作原子性
 * 4. 连接超时：10 秒超时机制
 * 5. 事件组同步：等待连接结果
 * 
 * 处理流程（12 步）：
 * 1. 参数检查：验证 SSID 有效性
 * 2. 获取互斥锁：保护 WiFi 操作（500ms 超时）
 * 3. 状态检查：避免重复连接
 * 4. 设置状态：WIFI_MANAGER_CONNECTING
 * 5. 设置标志：gWifiConnecting = true（让 BLE 让路）
 * 6. 清空事件组：清除连接/失败标志
 * 7. 复制凭证：保存到全局缓冲区
 * 8. 配置 WiFi：设置 SSID、密码、认证模式
 * 9. 启动 WiFi：set_mode → set_config → start
 * 10. 发起连接：esp_wifi_connect()
 * 11. 释放锁：等待结果时不持锁
 * 12. 等待结果：事件组等待（10 秒超时）
 * 
 * ESP_ERR_WIFI_STATE 容忍：
 * - esp_wifi_set_mode: 容忍此错误（WiFi 已在运行）
 * - esp_wifi_set_config: 容忍此错误（配置中/连接中）
 * - esp_wifi_start: 容忍此错误（已启动）
 * - esp_wifi_connect: 容忍此错误（正连接）
 * 
 * 连接参数：
 * - 认证模式：WIFI_AUTH_OPEN（最低要求）
 * - PMF 配置：capable=true, required=false
 * - 省电模式：WIFI_PS_NONE（关闭省电）
 * 
 * 返回条件：
 * - 成功：收到 WIFI_CONNECTED_BIT 事件
 * - 失败：超时/错误/状态不允许
 * 
 * @param ssid - WiFi SSID（不能为空）
 * @param password - WiFi 密码（可为空表示开放网络）
 * @return true - 连接成功
 * @return false - 连接失败或状态不允许
 * 
 * 注意：
 * - 连接过程中 gWifiConnecting 标志保持为 true
 * - 连接成功/失败后才清除此标志
 * - 互斥锁在等待结果时会释放
 */
static bool wifi_manager_connect_with_credentials(const char *ssid, const char *password)
{
    // 步骤 1: 参数检查 - 验证 SSID 有效性
    if (ssid == nullptr || ssid[0] == '\0') {
        ESP_LOGW(TAG, "wifi connect skipped: empty ssid");
        return false;
    }

    // 步骤 2: 获取互斥锁 - 保护 WiFi 操作（500ms 超时）
    if (gWifiOperationMutex != nullptr) {
        if (xSemaphoreTake(gWifiOperationMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
            ESP_LOGW(TAG, "wifi connect skipped: operation mutex busy");
            return false;
        }
    }

    // 步骤 3: 状态检查 - 避免重复连接
    if (gWifiState.state == WIFI_MANAGER_CONNECTING) {
        ESP_LOGW(TAG, "wifi connect skipped: already connecting");
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return false;
    }

    // 步骤 3: 状态检查 - 避免与扫描冲突
    if (gScanInProgress) {
        ESP_LOGW(TAG, "wifi connect skipped: scan in progress");
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return false;
    }

    // 步骤 4: 设置状态 - CONNECTING
    wifi_manager_set_state_internal(WIFI_MANAGER_CONNECTING);
    
    // 步骤 5: 设置标志 - gWifiConnecting = true（让 BLE 让路）
    gWifiConnecting = true;
    
    // 重置连接状态信息
    gWifiState.connected = 0U;
    gWifiState.ip[0] = '\0';
    gWifiState.rssi = -127;

    // 步骤 7: 复制凭证 - 保存到全局缓冲区
    wifi_manager_copy_active_credentials(ssid, password);

    // 步骤 6: 清空事件组 - 清除连接/失败标志
    if (gWifiEventGroup != nullptr) {
        xEventGroupClearBits(gWifiEventGroup, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    }

    // 步骤 8: 配置 WiFi - 准备连接参数
    wifi_config_t wifiConfig = {};
    std::strncpy(reinterpret_cast<char *>(wifiConfig.sta.ssid),
                 gSsid,
                 sizeof(wifiConfig.sta.ssid) - 1U);
    std::strncpy(reinterpret_cast<char *>(wifiConfig.sta.password),
                 gPassword,
                 sizeof(wifiConfig.sta.password) - 1U);
    wifiConfig.sta.threshold.authmode = WIFI_AUTH_OPEN;  // 最低认证要求
    wifiConfig.sta.pmf_cfg.capable = true;               // 支持 PMF
    wifiConfig.sta.pmf_cfg.required = false;             // 不强制 PMF

    // 步骤 9a: 设置模式 - 容忍 ESP_ERR_WIFI_STATE
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK && err != ESP_ERR_WIFI_STATE) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(err));
        gWifiConnecting = false;
        wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return false;
    }

    // 步骤 9b: 设置配置 - 容忍 ESP_ERR_WIFI_STATE
    err = esp_wifi_set_config(WIFI_IF_STA, &wifiConfig);
    if (err != ESP_OK) {
        if (err == ESP_ERR_WIFI_STATE) {
            ESP_LOGW(TAG, "esp_wifi_set_config skipped: station busy/connecting");
        } else {
            ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        }
        gWifiConnecting = false;
        wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return false;
    }

    // 步骤 9c: 启动 WiFi - 容忍 ESP_ERR_WIFI_STATE
    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_STATE) {
        ESP_LOGE(TAG, "esp_wifi_start failed for ssid=%s: %s", gSsid, esp_err_to_name(err));
        gWifiConnecting = false;
        wifi_manager_set_state_internal(WIFI_MANAGER_ERROR);
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return false;
    }

    // 关闭 WiFi 省电模式，提高连接稳定性
    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "wifi power save disabled");
    }

    // 步骤 10: 发起连接 - 容忍 ESP_ERR_WIFI_STATE
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        if (err == ESP_ERR_WIFI_STATE) {
            ESP_LOGW(TAG, "esp_wifi_connect skipped: station busy/connecting");
        } else {
            ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        }
        gWifiConnecting = false;
        wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return false;
    }

    // 步骤 11: 释放锁 - 等待结果时不持锁
    if (gWifiOperationMutex != nullptr) {
        xSemaphoreGive(gWifiOperationMutex);
    }

    // 检查事件组
    if (gWifiEventGroup == nullptr) {
        ESP_LOGE(TAG, "wifi event group missing");
        wifi_manager_set_state_internal(WIFI_MANAGER_ERROR);
        return false;
    }

    // 步骤 12: 等待结果 - 事件组等待（10 秒超时）
    const EventBits_t bits = xEventGroupWaitBits(gWifiEventGroup,
                                                 WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                 pdTRUE,  // 清除位
                                                 pdFALSE, // 任意位
                                                 pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if ((bits & WIFI_CONNECTED_BIT) != 0U) {
        ESP_LOGI(TAG, "wifi connected to ssid=%s", gSsid);
        return true;  // 连接成功
    }

    // 连接失败/超时
    ESP_LOGW(TAG, "wifi connect timeout/fail ssid=%s", gSsid);
    gWifiConnecting = false;
    wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
    return false;
}

/**
 * @brief 内部状态机设置函数
 * 
 * @param state 新的 WiFi 管理器状态
 * 
 * 功能说明：
 * - 更新全局状态快照
 * - 记录状态变化时间戳
 * - 同步网络状态到系统状态模块
 * 
 * 状态映射关系（8 种状态 → 4 种网络状态）：
 * 1. CONNECTED → NET_CONNECTED（已连接）
 * 2. SCANNING → NET_CONNECTING（连接中）
 * 3. CONNECTING → NET_CONNECTING（连接中）
 * 4. CONFIGURING → NET_CONNECTING（连接中）
 * 5. DISCONNECTED → NET_DISCONNECTED（已断开）
 * 6. IDLE → NET_DISCONNECTED（已断开）
 * 7. INITIALIZED → NET_DISCONNECTED（已断开）
 * 8. ERROR → NET_DISCONNECTED（错误）
 * 
 * 用途：
 * - 统一状态管理
 * - 同步网络状态指示灯
 * - 供其他模块查询 WiFi 状态
 * 
 * 注意：
 * - 此函数被频繁调用
 * - 状态变化会触发系统状态更新
 */
static void wifi_manager_set_state_internal(WiFiManagerState state)
{
    // 更新全局状态快照
    gWifiState.state = state;
    
    // 记录状态变化时间戳（毫秒）
    gWifiState.last_state_change_ms = radar_now_ms();

    // 同步网络状态到系统状态模块
    switch (state) {
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
        setNetworkStatus(NET_DISCONNECTED);
        break;
    case WIFI_MANAGER_ERROR:
        setNetworkStatus(NET_DISCONNECTED);
        break;
    default:
        setNetworkStatus(NET_INITIAL);
        break;
    }
}

/**
 * @brief WiFi 事件处理函数
 * 
 * @param arg 事件参数（未使用）
 * @param event_base 事件基础类型（WIFI_EVENT 或 IP_EVENT）
 * @param event_id 事件 ID
 * @param event_data 事件数据（未使用）
 * 
 * 处理的事件类型（4 种）：
 * 
 * 1. WIFI_EVENT_STA_START（STA 启动）
 *    - 调用 tasks_manager 的 WiFi 事件处理
 *    - 打印日志
 * 
 * 2. WIFI_EVENT_STA_CONNECTED（已连接 AP）
 *    - 调用 tasks_manager 的 WiFi 事件处理
 *    - 打印日志
 *    - 注意：此时还未获取 IP
 * 
 * 3. WIFI_EVENT_STA_DISCONNECTED（断开连接）
 *    - 重置连接标志和状态
 *    - 清除 gWifiConnecting（让 BLE 恢复）
 *    - 设置状态为 DISCONNECTED
 *    - 调用 tasks_manager 的 WiFi 事件处理
 *    - 设置失败事件位（唤醒等待线程）
 *    - 打印日志
 * 
 * 4. IP_EVENT_STA_GOT_IP（获取 IP 地址）
 *    - 设置连接成功标志
 *    - 设置状态为 CONNECTED
 *    - 清除 gWifiConnecting（连接成功）
 *    - 调用 tasks_manager 的 WiFi 事件处理
 *    - 获取 AP 信息（RSSI）
 *    - 获取 IP 地址信息
 *    - 设置成功事件位（唤醒等待线程）
 *    - 打印日志（SSID、IP、RSSI）
 * 
 * 事件处理流程：
 * 1. 判断事件基础类型（WIFI_EVENT 或 IP_EVENT）
 * 2. 根据事件 ID 分发处理
 * 3. 更新全局状态和标志
 * 4. 通知 tasks_manager
 * 5. 设置事件组位（如适用）
 * 6. 打印日志
 * 
 * 注意：
 * - 此函数在中断上下文中执行
 * - 避免在此函数中执行耗时操作
 * - gWifiConnecting 标志在断开/成功时清除
 */
static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;
    (void)event_data;

    // 事件 1: STA 启动
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        WiFiEvent(TASK_WIFI_EVENT_STA_START);
        ESP_LOGI(TAG, "wifi sta started");
        return;
    }

    // 事件 2: 已连接 AP
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        WiFiEvent(TASK_WIFI_EVENT_STA_CONNECTED);
        ESP_LOGI(TAG, "wifi sta connected");
        return;
    }

    // 事件 3: 断开连接
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // 重置连接状态
        gWifiState.connected = 0U;
        gWifiConnecting = false;  // 连接失败/断开，清除标志（让 BLE 恢复）
        gWifiState.rssi = -127;
        gWifiState.ip[0] = '\0';
        
        // 更新状态
        wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
        
        // 通知 tasks_manager
        WiFiEvent(TASK_WIFI_EVENT_STA_DISCONNECTED);
        
        // 设置失败事件位（唤醒等待线程）
        if (gWifiEventGroup != nullptr) {
            xEventGroupSetBits(gWifiEventGroup, WIFI_FAIL_BIT);
        }
        ESP_LOGW(TAG, "wifi disconnected");
        return;
    }

    // 事件 4: 获取 IP 地址
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // 设置连接成功标志
        gWifiState.connected = 1U;
        
        // 更新状态为 CONNECTED
        wifi_manager_set_state_internal(WIFI_MANAGER_CONNECTED);
        
        // 清除连接标志（连接成功）
        gWifiConnecting = false;
        
        // 通知 tasks_manager
        WiFiEvent(TASK_WIFI_EVENT_STA_GOT_IP);

        // 获取 AP 信息（RSSI）
        wifi_ap_record_t apInfo = {};
        if (esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK) {
            gWifiState.rssi = static_cast<int8_t>(apInfo.rssi);
        }

        // 获取 IP 地址信息
        esp_netif_ip_info_t ipInfo = {};
        if (gStaNetif != nullptr && esp_netif_get_ip_info(gStaNetif, &ipInfo) == ESP_OK) {
            snprintf(gWifiState.ip,
                     sizeof(gWifiState.ip),
                     IPSTR,
                     IP2STR(&ipInfo.ip));
        }

        // 设置成功事件位（唤醒等待线程）
        if (gWifiEventGroup != nullptr) {
            xEventGroupSetBits(gWifiEventGroup, WIFI_CONNECTED_BIT);
        }
        
        ESP_LOGI(TAG,
                 "wifi got ip, ssid=%s ip=%s rssi=%d",
                 gSsid,
                 gWifiState.ip,
                 static_cast<int>(gWifiState.rssi));
    }
}

/**
 * @brief 后台 WiFi 重连任务
 * 
 * @param parameter 任务参数（未使用）
 * 
 * 功能说明：
 * - 无限循环，定期检查是否需要重连
 * - 重连间隔：5 秒
 * - 避免在配置/扫描/错误时重连
 * 
 * 重连条件（7 个，必须同时满足）：
 * 1. gWifiState.initialized != 0：已初始化
 * 2. gWifiState.has_credentials != 0：有凭证
 * 3. gWifiState.connected == 0：未连接
 * 4. !gManualConfigActive：非手动配置中
 * 5. !gScanInProgress：非扫描中
 * 6. state != CONNECTING/CONFIGURING/ERROR：状态允许
 * 
 * 任务流程：
 * 1. 无限循环
 * 2. 检查重连条件
 * 3. 如果满足条件：调用 wifi_manager_connect()
 * 4. 延迟 5 秒
 * 5. 重复步骤 2
 * 
 * 设计要点：
 * - 低优先级后台任务
 * - 不频繁尝试重连（避免资源浪费）
 * - 避免与其他操作冲突
 * 
 * 注意：
 * - 此任务永不停止
 * - 使用 vTaskDelay 而非 usleep
 * - 连接函数返回值被忽略（下次循环再试）
 */
static void wifi_reconnect_task(void *parameter)
{
    (void)parameter;

    // 无限循环
    while (true) {
        // 步骤 2: 检查重连条件（7 个）
        const bool shouldReconnect =
            gWifiState.initialized != 0U &&           // 已初始化
            gWifiState.has_credentials != 0U &&       // 有凭证
            gWifiState.connected == 0U &&             // 未连接
            !gManualConfigActive &&                   // 非手动配置中
            !gScanInProgress &&                       // 非扫描中
            gWifiState.state != WIFI_MANAGER_CONNECTING &&  // 非连接中
            gWifiState.state != WIFI_MANAGER_CONFIGURING && // 非配置中
            gWifiState.state != WIFI_MANAGER_ERROR;         // 非错误状态

        if (shouldReconnect) {
            // 步骤 3: 尝试重连
            ESP_LOGI(TAG, "background wifi reconnect attempt");
            (void)wifi_manager_connect();
        }

        // 步骤 4: 延迟 5 秒
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_INTERVAL_MS));
    }
}

static bool wifi_manager_ensure_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool wifi_manager_load_credentials_from_nvs(void)
{
    gSavedNetworkCount = 0U;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs open for read failed: %s", esp_err_to_name(err));
        return false;
    }

    for (uint32_t i = 0; i < WIFI_MANAGER_MAX_SAVED_NETWORKS; ++i) {
        char ssidKey[16] = {};
        char passKey[16] = {};
        wifi_manager_make_indexed_key(ssidKey, sizeof(ssidKey), "wifi_", i);
        wifi_manager_make_indexed_key(passKey, sizeof(passKey), "pass_", i);

        size_t ssidLen = sizeof(gSavedNetworks[i].ssid);
        size_t passLen = sizeof(gSavedNetworks[i].password);
        esp_err_t ssidErr = nvs_get_str(handle, ssidKey, gSavedNetworks[i].ssid, &ssidLen);
        if (ssidErr != ESP_OK) {
            continue;
        }

        esp_err_t passErr = nvs_get_str(handle, passKey, gSavedNetworks[i].password, &passLen);
        if (passErr != ESP_OK) {
            gSavedNetworks[i].ssid[0] = '\0';
            continue;
        }

        ++gSavedNetworkCount;
    }

    if (gSavedNetworkCount == 0U) {
        size_t ssidLen = sizeof(gSsid);
        size_t passLen = sizeof(gPassword);
        err = nvs_get_str(handle, WIFI_NVS_KEY_SSID, gSsid, &ssidLen);
        if (err == ESP_OK) {
            err = nvs_get_str(handle, WIFI_NVS_KEY_PASS, gPassword, &passLen);
        }
        if (err == ESP_OK) {
            wifi_manager_copy_active_credentials(gSsid, gPassword);
            std::strncpy(gSavedNetworks[0].ssid, gSsid, sizeof(gSavedNetworks[0].ssid) - 1U);
            std::strncpy(gSavedNetworks[0].password, gPassword, sizeof(gSavedNetworks[0].password) - 1U);
            gSavedNetworkCount = 1U;
        }
    } else {
        wifi_manager_copy_active_credentials(gSavedNetworks[0].ssid, gSavedNetworks[0].password);
    }

    gWifiState.has_credentials = gSavedNetworkCount > 0U ? 1U : 0U;
    nvs_close(handle);
    if (gSavedNetworkCount > 0U) {
        ESP_LOGI(TAG, "loaded %u wifi credential set(s) from nvs, active ssid=%s",
                 (unsigned)gSavedNetworkCount,
                 gSsid);
        return true;
    }
    return false;
}

static bool wifi_manager_save_credentials_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open for write failed: %s", esp_err_to_name(err));
        return false;
    }

    for (uint32_t i = 0; i < WIFI_MANAGER_MAX_SAVED_NETWORKS && err == ESP_OK; ++i) {
        char ssidKey[16] = {};
        char passKey[16] = {};
        wifi_manager_make_indexed_key(ssidKey, sizeof(ssidKey), "wifi_", i);
        wifi_manager_make_indexed_key(passKey, sizeof(passKey), "pass_", i);

        if (i < gSavedNetworkCount && gSavedNetworks[i].ssid[0] != '\0') {
            err = nvs_set_str(handle, ssidKey, gSavedNetworks[i].ssid);
            if (err == ESP_OK) {
                err = nvs_set_str(handle, passKey, gSavedNetworks[i].password);
            }
        } else {
            esp_err_t ssidErr = nvs_erase_key(handle, ssidKey);
            if (ssidErr != ESP_OK && ssidErr != ESP_ERR_NVS_NOT_FOUND) {
                err = ssidErr;
            }
            if (err == ESP_OK) {
                esp_err_t passErr = nvs_erase_key(handle, passKey);
                if (passErr != ESP_OK && passErr != ESP_ERR_NVS_NOT_FOUND) {
                    err = passErr;
                }
            }
        }
    }
    if (err == ESP_OK) {
        err = nvs_set_str(handle, WIFI_NVS_KEY_SSID, gSsid);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(handle, WIFI_NVS_KEY_PASS, gPassword);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "saving wifi credentials failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "saved %u wifi credential set(s) to nvs, active ssid=%s",
             (unsigned)gSavedNetworkCount,
             gSsid);
    return true;
}

bool wifi_manager_clear_credentials(void)
{
    if (!wifi_manager_ensure_nvs()) {
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open for clear failed: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_erase_key(handle, WIFI_NVS_KEY_SSID);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        esp_err_t passErr = nvs_erase_key(handle, WIFI_NVS_KEY_PASS);
        if (passErr != ESP_OK && passErr != ESP_ERR_NVS_NOT_FOUND) {
            err = passErr;
        }
    }
    for (uint32_t i = 0; i < WIFI_MANAGER_MAX_SAVED_NETWORKS && err == ESP_OK; ++i) {
        char ssidKey[16] = {};
        char passKey[16] = {};
        wifi_manager_make_indexed_key(ssidKey, sizeof(ssidKey), "wifi_", i);
        wifi_manager_make_indexed_key(passKey, sizeof(passKey), "pass_", i);

        esp_err_t ssidErr = nvs_erase_key(handle, ssidKey);
        if (ssidErr != ESP_OK && ssidErr != ESP_ERR_NVS_NOT_FOUND) {
            err = ssidErr;
            break;
        }

        esp_err_t passErr = nvs_erase_key(handle, passKey);
        if (passErr != ESP_OK && passErr != ESP_ERR_NVS_NOT_FOUND) {
            err = passErr;
            break;
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "clearing wifi credentials failed: %s", esp_err_to_name(err));
        return false;
    }

    gSsid[0] = '\0';
    gPassword[0] = '\0';
    gWifiState.ssid[0] = '\0';
    gWifiState.has_credentials = 0U;
    gWifiState.reconnect_attempts = 0U;
    gSavedNetworkCount = 0U;
    std::memset(gSavedNetworks, 0, sizeof(gSavedNetworks));
    wifi_manager_disconnect();
    ESP_LOGI(TAG, "wifi credentials cleared");
    return true;
}

uint32_t wifi_manager_scan_networks(WiFiScanRecord *records, uint32_t max_records)
{
    if (records == nullptr || max_records == 0U || !gWifiState.initialized) {
        return 0U;
    }

    if (gWifiOperationMutex != nullptr &&
        xSemaphoreTake(gWifiOperationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "wifi scan skipped because wifi operation is busy");
        return 0U;
    }

    gScanInProgress = true;
    wifi_manager_set_state_internal(WIFI_MANAGER_SCANNING);

    wifi_scan_config_t scanConfig = {};
    scanConfig.show_hidden = true;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK && err != ESP_ERR_WIFI_MODE) {
        ESP_LOGE(TAG, "esp_wifi_set_mode before scan failed: %s", esp_err_to_name(err));
        gScanInProgress = false;
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return 0U;
    }

    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_STATE) {
        ESP_LOGE(TAG, "esp_wifi_start before scan failed: %s", esp_err_to_name(err));
        gScanInProgress = false;
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return 0U;
    }

    // 关闭 WiFi 省电模式，提高连接稳定性
    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "wifi power save disabled");
    }

    uint16_t apCount = 0U;
    for (uint32_t retry = 0U; retry < 3U; ++retry) {
        err = esp_wifi_scan_start(&scanConfig, true);
        if (err == ESP_OK) {
            err = esp_wifi_scan_get_ap_num(&apCount);
            if (err == ESP_OK && apCount > 0U) {
                break;
            }
        }
        
        // 如果是 STATE 错误，说明正在连接中，直接退出
        if (err == ESP_ERR_WIFI_STATE) {
            ESP_LOGW(TAG, "wifi scan skipped because sta is connecting");
            gScanInProgress = false;
            wifi_manager_set_state_internal(gWifiState.connected ? WIFI_MANAGER_CONNECTED : WIFI_MANAGER_DISCONNECTED);
            if (gWifiOperationMutex != nullptr) {
                xSemaphoreGive(gWifiOperationMutex);
            }
            return 0U;
        }
        
        ESP_LOGW(TAG, "wifi scan attempt %u failed/empty: %s", (unsigned)(retry + 1U), esp_err_to_name(err));
        radar_sleep_ms(500);
    }

    if (err != ESP_OK || apCount == 0U) {
        gScanInProgress = false;
        wifi_manager_set_state_internal(gWifiState.connected ? WIFI_MANAGER_CONNECTED : WIFI_MANAGER_DISCONNECTED);
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return 0U;
    }

    wifi_ap_record_t apRecords[16] = {};
    uint16_t fetchCount = apCount > 16U ? 16U : apCount;
    err = esp_wifi_scan_get_ap_records(&fetchCount, apRecords);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_scan_get_ap_records failed: %s", esp_err_to_name(err));
        gScanInProgress = false;
        wifi_manager_set_state_internal(gWifiState.connected ? WIFI_MANAGER_CONNECTED : WIFI_MANAGER_DISCONNECTED);
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return 0U;
    }

    uint32_t copyCount = 0U;
    for (uint16_t i = 0; i < fetchCount && copyCount < max_records; ++i) {
        if (apRecords[i].rssi < WIFI_MIN_RSSI_THRESHOLD) {
            continue;
        }

        std::strncpy(records[copyCount].ssid,
                     reinterpret_cast<const char *>(apRecords[i].ssid),
                     sizeof(records[copyCount].ssid) - 1U);
        records[copyCount].ssid[sizeof(records[copyCount].ssid) - 1U] = '\0';
        records[copyCount].rssi = static_cast<int8_t>(apRecords[i].rssi);
        records[copyCount].channel = static_cast<uint8_t>(apRecords[i].primary);
        std::strncpy(records[copyCount].encryption,
                     wifi_manager_authmode_to_string(apRecords[i].authmode),
                     sizeof(records[copyCount].encryption) - 1U);
        records[copyCount].encryption[sizeof(records[copyCount].encryption) - 1U] = '\0';
        ++copyCount;
    }

    gScanInProgress = false;
    wifi_manager_set_state_internal(gWifiState.connected ? WIFI_MANAGER_CONNECTED : WIFI_MANAGER_DISCONNECTED);
    if (gWifiOperationMutex != nullptr) {
        xSemaphoreGive(gWifiOperationMutex);
    }
    ESP_LOGI(TAG, "wifi scan finished with %u AP records", (unsigned)copyCount);
    return copyCount;
}

uint32_t wifi_manager_get_saved_networks(WiFiSavedNetwork *networks, uint32_t max_networks)
{
    if (networks == nullptr || max_networks == 0U || gSavedNetworkCount == 0U) {
        return 0U;
    }

    const uint32_t copyCount = gSavedNetworkCount < max_networks ? gSavedNetworkCount : max_networks;
    for (uint32_t i = 0; i < copyCount; ++i) {
        std::strncpy(networks[i].ssid, gSavedNetworks[i].ssid, sizeof(networks[i].ssid) - 1U);
        networks[i].ssid[sizeof(networks[i].ssid) - 1U] = '\0';
    }
    return copyCount;
}

bool wifi_manager_has_saved_network(const char *ssid)
{
    if (ssid == nullptr || ssid[0] == '\0') {
        return false;
    }

    for (uint32_t i = 0; i < gSavedNetworkCount; ++i) {
        if (std::strcmp(gSavedNetworks[i].ssid, ssid) == 0) {
            return true;
        }
    }

    return false;
}

static bool wifi_manager_ensure_stack(void)
{
    if (!wifi_manager_ensure_nvs()) {
        return false;
    }

    if (!gEspNetifInitialized) {
        ESP_ERROR_CHECK(esp_netif_init());
        esp_err_t err = esp_event_loop_create_default();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "event loop init failed: %s", esp_err_to_name(err));
            return false;
        }
        gEspNetifInitialized = true;
    }

    if (gStaNetif == nullptr) {
        gStaNetif = esp_netif_create_default_wifi_sta();
        if (gStaNetif == nullptr) {
            ESP_LOGE(TAG, "failed to create default wifi sta netif");
            return false;
        }
    }

    if (gWifiEventGroup == nullptr) {
        gWifiEventGroup = xEventGroupCreate();
        if (gWifiEventGroup == nullptr) {
            ESP_LOGE(TAG, "failed to create wifi event group");
            return false;
        }
    }

    if (gWifiOperationMutex == nullptr) {
        gWifiOperationMutex = xSemaphoreCreateMutex();
        if (gWifiOperationMutex == nullptr) {
            ESP_LOGE(TAG, "failed to create wifi operation mutex");
            return false;
        }
    }

    if (!gWifiDriverInitialized) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_wifi_init(&cfg);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
            return false;
        }
        gWifiDriverInitialized = true;
    }

    if (!gEventHandlersRegistered) {
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                                   ESP_EVENT_ANY_ID,
                                                   &wifi_event_handler,
                                                   nullptr));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                                   IP_EVENT_STA_GOT_IP,
                                                   &wifi_event_handler,
                                                   nullptr));
        gEventHandlersRegistered = true;
    }

    return true;
}

bool wifi_manager_init(void)
{
    if (gWifiState.initialized) {
        return true;
    }

    if (!wifi_manager_ensure_stack()) {
        wifi_manager_set_state_internal(WIFI_MANAGER_ERROR);
        system_state_set_error(1U);
        return false;
    }

    gWifiState.initialized = 1U;
    gWifiState.connected = 0U;
    gWifiState.has_credentials = 0U;
    gWifiState.rssi = -127;
    gWifiState.reconnect_attempts = 0U;
    gSavedNetworkCount = 0U;
    std::memset(gSavedNetworks, 0, sizeof(gSavedNetworks));
    gWifiState.ssid[0] = '\0';
    gSsid[0] = '\0';
    gPassword[0] = '\0';
    (void)wifi_manager_load_credentials_from_nvs();
    wifi_manager_set_state_internal(WIFI_MANAGER_INITIALIZED);
    system_state_set_wifi_ready(1U);

    if (gReconnectTaskHandle == nullptr) {
        xTaskCreate(wifi_reconnect_task,
                    "wifi_reconnect_task",
                    8192,  // 从 4096 增加到 8192，避免栈溢出
                    nullptr,
                    2,
                    &gReconnectTaskHandle);
    }

    ESP_LOGI(TAG, "minimal wifi manager initialized");
    return true;
}

bool wifi_manager_is_initialized(void)
{
    return gWifiState.initialized != 0U;
}

bool wifi_manager_is_connected(void)
{
    return gWifiState.connected != 0U;
}

// 返回 WiFi 是否正在连接中（scan/auth/connect 阶段）
bool wifi_manager_is_connecting(void)
{
    return gWifiConnecting;
}

WiFiManagerState wifi_manager_get_state(void)
{
    return gWifiState.state;
}

WiFiManagerSnapshot wifi_manager_get_snapshot(void)
{
    return gWifiState;
}

void wifi_manager_set_credentials_present(uint8_t present)
{
    gWifiState.has_credentials = present ? 1U : 0U;
}

bool wifi_manager_set_credentials(const char *ssid, const char *password)
{
    if (ssid == nullptr || password == nullptr || ssid[0] == '\0') {
        ESP_LOGW(TAG, "wifi credentials rejected: empty ssid or null input");
        return false;
    }

    uint32_t targetIndex = gSavedNetworkCount;
    for (uint32_t i = 0; i < gSavedNetworkCount; ++i) {
        if (std::strcmp(gSavedNetworks[i].ssid, ssid) == 0) {
            targetIndex = i;
            break;
        }
    }

    if (targetIndex >= WIFI_MANAGER_MAX_SAVED_NETWORKS) {
        targetIndex = WIFI_MANAGER_MAX_SAVED_NETWORKS - 1U;
    }

    std::strncpy(gSavedNetworks[targetIndex].ssid, ssid, sizeof(gSavedNetworks[targetIndex].ssid) - 1U);
    gSavedNetworks[targetIndex].ssid[sizeof(gSavedNetworks[targetIndex].ssid) - 1U] = '\0';
    std::strncpy(gSavedNetworks[targetIndex].password, password, sizeof(gSavedNetworks[targetIndex].password) - 1U);
    gSavedNetworks[targetIndex].password[sizeof(gSavedNetworks[targetIndex].password) - 1U] = '\0';

    if (targetIndex == gSavedNetworkCount && gSavedNetworkCount < WIFI_MANAGER_MAX_SAVED_NETWORKS) {
        ++gSavedNetworkCount;
    }

    wifi_manager_copy_active_credentials(ssid, password);
    wifi_manager_set_credentials_present(1U);
    wifi_manager_set_state_internal(WIFI_MANAGER_CONFIGURING);

    if (!wifi_manager_save_credentials_to_nvs()) {
        ESP_LOGW(TAG, "wifi credentials stored in memory but not persisted");
    }

    ESP_LOGI(TAG, "wifi credentials stored for ssid=%s", gSsid);
    return true;
}

bool wifi_manager_configure_and_connect(const char *ssid, const char *password)
{
    if (!gWifiState.initialized || ssid == nullptr || ssid[0] == '\0') {
        return false;
    }

    gManualConfigActive = true;
    wifi_manager_set_state_internal(WIFI_MANAGER_CONFIGURING);

    esp_err_t stopScanErr = esp_wifi_scan_stop();
    if (stopScanErr != ESP_OK && stopScanErr != ESP_ERR_WIFI_NOT_STARTED &&
        stopScanErr != ESP_ERR_WIFI_STATE) {
        ESP_LOGW(TAG, "esp_wifi_scan_stop during manual config failed: %s",
                 esp_err_to_name(stopScanErr));
    }
    gScanInProgress = false;
    (void)esp_wifi_disconnect();

    char actualPassword[65] = {};
    if (password != nullptr && password[0] != '\0') {
        std::strncpy(actualPassword, password, sizeof(actualPassword) - 1U);
    } else {
        (void)wifi_manager_find_saved_password(ssid, actualPassword, sizeof(actualPassword));
    }

    WiFiScanRecord scanRecords[16] = {};
    const uint32_t scanCount = wifi_manager_scan_networks(scanRecords, 16U);
    bool networkFound = false;
    bool signalTooWeak = false;
    int8_t foundRssi = -127;

    for (uint32_t i = 0; i < scanCount; ++i) {
        if (std::strcmp(scanRecords[i].ssid, ssid) == 0) {
            foundRssi = scanRecords[i].rssi;
            if (scanRecords[i].rssi >= WIFI_MIN_RSSI_THRESHOLD) {
                networkFound = true;
                break;
            }
            signalTooWeak = true;
        }
    }

    if (!networkFound) {
        ESP_LOGW(TAG,
                 "manual wifi config target not usable ssid=%s found_rssi=%d weak=%s",
                 ssid,
                 (int)foundRssi,
                 signalTooWeak ? "yes" : "no");
        gManualConfigActive = false;
        wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
        return false;
    }

    const bool connected = wifi_manager_connect_with_credentials(ssid, actualPassword);
    if (connected) {
        if (!wifi_manager_set_credentials(ssid, actualPassword)) {
            ESP_LOGW(TAG, "manual wifi connected but credential persistence failed ssid=%s", ssid);
        }
        gManualConfigActive = false;
        wifi_manager_set_state_internal(WIFI_MANAGER_CONNECTED);
        return true;
    }

    gManualConfigActive = false;
    wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
    return false;
}

bool wifi_manager_connect(void)
{
    if (!gWifiState.initialized) {
        ESP_LOGW(TAG, "wifi connect requested before init");
        return false;
    }

    if (!gWifiState.has_credentials) {
        wifi_manager_set_state_internal(WIFI_MANAGER_CONFIGURING);
        ESP_LOGW(TAG, "wifi connect requested without credentials");
        return false;
    }

    if (gWifiState.connected) {
        return true;
    }

    WiFiScanRecord scanRecords[16] = {};
    const uint32_t scanCount = wifi_manager_scan_networks(scanRecords, 16U);

    if (scanCount > 0U && gSavedNetworkCount > 0U) {
        struct CandidateNetwork {
            uint32_t saved_index;
            int8_t rssi;
        };

        CandidateNetwork candidates[WIFI_MANAGER_MAX_SAVED_NETWORKS] = {};
        uint32_t candidateCount = 0U;

        for (uint32_t savedIndex = 0; savedIndex < gSavedNetworkCount; ++savedIndex) {
            for (uint32_t scanIndex = 0; scanIndex < scanCount; ++scanIndex) {
                if (std::strcmp(gSavedNetworks[savedIndex].ssid, scanRecords[scanIndex].ssid) == 0 &&
                    scanRecords[scanIndex].rssi >= WIFI_MIN_RSSI_THRESHOLD) {
                    candidates[candidateCount].saved_index = savedIndex;
                    candidates[candidateCount].rssi = scanRecords[scanIndex].rssi;
                    ++candidateCount;
                    break;
                }
            }
        }

        for (uint32_t i = 0; i + 1U < candidateCount; ++i) {
            for (uint32_t j = 0; j + 1U < candidateCount - i; ++j) {
                if (candidates[j].rssi < candidates[j + 1U].rssi) {
                    const CandidateNetwork temp = candidates[j];
                    candidates[j] = candidates[j + 1U];
                    candidates[j + 1U] = temp;
                }
            }
        }

        for (uint32_t i = 0; i < candidateCount; ++i) {
            const uint32_t savedIndex = candidates[i].saved_index;
            ESP_LOGI(TAG,
                     "trying saved network match ssid=%s rssi=%d",
                     gSavedNetworks[savedIndex].ssid,
                     (int)candidates[i].rssi);
            if (wifi_manager_connect_with_credentials(gSavedNetworks[savedIndex].ssid,
                                                      gSavedNetworks[savedIndex].password)) {
                return true;
            }
        }
    }

    for (uint32_t i = 0; i < gSavedNetworkCount; ++i) {
        if (std::strcmp(gSavedNetworks[i].ssid, gSsid) == 0) {
            return wifi_manager_connect_with_credentials(gSavedNetworks[i].ssid,
                                                         gSavedNetworks[i].password);
        }
    }

    if (gSsid[0] != '\0') {
        return wifi_manager_connect_with_credentials(gSsid, gPassword);
    }

    return false;
}

void wifi_manager_disconnect(void)
{
    if (gWifiDriverInitialized) {
        esp_wifi_disconnect();
    }
    gWifiState.connected = 0U;
    gWifiState.rssi = -127;
    gWifiState.ip[0] = '\0';
    if (gWifiState.has_credentials != 0U) {
        gWifiState.reconnect_attempts = 0U;
    }
    wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
    WiFiEvent(TASK_WIFI_EVENT_STA_STOP);
    ESP_LOGI(TAG, "wifi disconnected");
}

const char *wifi_manager_get_ssid(void)
{
    return gSsid;
}

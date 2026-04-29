#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "mqtt_manager.h"
#include "system_state.h"
#include "voice_manager.h"

#include <inttypes.h>
// ============================================================================
// 头文件包含
// ============================================================================
#include "app_config.h"       // 应用配置（WiFi 密码、MQTT 服务器、InfluxDB 参数等）
#include "device_identity.h"  // 设备身份管理（device_id, device_sn, 设备哈希）
#include "driver/gpio.h"       // GPIO 驱动（用于启动按钮检测）
#include "esp_err.h"           // ESP32 错误码定义
#include "esp_log.h"           // ESP32 日志系统
#include "esp_task_wdt.h"     // 任务看门狗（防止系统死锁）
#include "freertos/FreeRTOS.h" // FreeRTOS 实时操作系统
#include "freertos/task.h"     // FreeRTOS 任务管理
#include "radar_manager.h"     // 雷达传感器管理器（毫米波数据采集）
#include "system_state.h"      // 系统状态管理（错误码、运行阶段）
#include "tasks_manager.h"     // 任务管理器（创建 WiFi/BLE/MQTT/InfluxDB 任务）
#include "mqtt_manager.h"      // MQTT 客户端管理（物联网通信）
#include <inttypes.h>          // 格式化输出（PRIu64 等）
#include <cstdio>              // 标准输入输出

static const char *TAG = "radar_bootstrap";  // 日志标签，用于标识启动阶段日志

// ============================================================================
// 辅助函数：等待启动按钮释放
// ============================================================================
/**
 * @brief 等待用户释放启动按钮
 * 
 * 作用：
 * - 防止系统上电时按钮被误按导致异常启动
 * - 检测 GPIO 引脚电平，如果为低电平（按下），则循环等待直到释放
 * 
 * 硬件连接：
 * - APP_BOOT_BUTTON_PIN: 启动按钮连接的 GPIO 引脚
 * - 低电平有效（按下时为 0，释放时为 1）
 */
static void wait_for_boot_button_release(void)
{
    // 配置 GPIO 为输入模式
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << APP_BOOT_BUTTON_PIN);  // 设置要配置的 GPIO 引脚
    config.mode = GPIO_MODE_INPUT;                         // 输入模式
    config.pull_up_en = GPIO_PULLUP_ENABLE;               // 启用上拉电阻（释放时为高电平）
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;          // 禁用下拉电阻
    config.intr_type = GPIO_INTR_DISABLE;                 // 禁用中断（使用轮询方式）
    gpio_config(&config);

    vTaskDelay(pdMS_TO_TICKS(10));  // 短暂延时，等待 GPIO 配置生效
    
    // 检测按钮是否被按下（低电平）
    if (gpio_get_level((gpio_num_t)APP_BOOT_BUTTON_PIN) == 0) {
        ESP_LOGW(TAG, "boot button held at startup, waiting for release");
        // 循环等待直到按钮释放（电平变高）
        while (gpio_get_level((gpio_num_t)APP_BOOT_BUTTON_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));  // 每 50ms 检测一次
        }
        ESP_LOGI(TAG, "boot button released, continuing startup");
    }
}

// ============================================================================
// 辅助函数：初始化任务看门狗
// ============================================================================
/**
 * @brief 初始化任务看门狗（Task Watchdog Timer）
 * 
 * 作用：
 * - 监控系统任务是否正常运行
 * - 如果某个任务在 30 秒内没有"喂狗"（重置看门狗），系统会自动重启
 * - 防止系统死锁或任务卡死
 * 
 * 配置参数：
 * - timeout_ms: 30 秒超时
 * - idle_core_mask: 0（监控所有 CPU 核心）
 * - trigger_panic: true（超时时触发系统重启）
 */
static void init_task_watchdog(void)
{
    esp_task_wdt_config_t config = {};
    config.timeout_ms = 30000;        // 超时时间：30 秒
    config.idle_core_mask = 0;        // 监控所有 CPU 核心（ESP32 是双核）
    config.trigger_panic = true;      // 超时时触发系统重启（而不是仅打印警告）

    // 初始化看门狗，如果已经初始化过则重新配置
    esp_err_t err = esp_task_wdt_init(&config);
    if (err == ESP_ERR_INVALID_STATE) {
        err = esp_task_wdt_reconfigure(&config);  // 重新配置已存在的看门狗
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "task watchdog init failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "task watchdog configured timeout=%ums", (unsigned)config.timeout_ms);
    }
}

// ============================================================================
// 主函数：app_main
// ============================================================================
/**
 * @brief ESP32 应用程序主入口
 * 
 * 这是 ESP-IDF 框架的标准入口函数，相当于普通程序的 main()
 * 系统启动后会自动调用此函数
 * 
 * 执行流程：
 * 1. 初始化系统状态模块
 * 2. 等待启动按钮释放
 * 3. 初始化任务看门狗
 * 4. 初始化设备身份（获取唯一标识）
 * 5. 初始化雷达管理器（毫米波传感器）
 * 6. 初始化所有后台任务（WiFi/BLE/MQTT/InfluxDB 等）
 * 7. 配置 MQTT 客户端参数
 * 8. 打印设备身份信息
 * 9. 设置系统为运行阶段
 */
extern "C" void app_main(void)
{
    // ------------------------------------------------------------------------
    // 步骤 1: 初始化系统状态
    // ------------------------------------------------------------------------
    system_state_init();  // 初始化系统状态机（错误码、运行阶段等）
    
    // ------------------------------------------------------------------------
    // 步骤 2: 等待启动按钮释放
    // ------------------------------------------------------------------------
    wait_for_boot_button_release();  // 确保用户没有误按启动按钮
    
    // ------------------------------------------------------------------------
    // 步骤 3: 初始化任务看门狗
    // ------------------------------------------------------------------------
    init_task_watchdog();  // 配置 30 秒超时的看门狗，防止系统死锁

    // ------------------------------------------------------------------------
    // 步骤 4: 初始化设备身份
    // ------------------------------------------------------------------------
    /**
     * 设备身份是物联网设备的唯一标识，用于：
     * - MQTT 通信时的 client_id
     * - InfluxDB 数据上传时的 deviceId
     * - 云端识别设备身份
     */
    if (!device_identity_init()) {
        system_state_set_error(1U);  // 设置系统错误码
        ESP_LOGE(TAG, "device identity init failed");
        return;  // 身份初始化失败，无法继续运行
    }

    // 获取系统启动时的状态快照
    const SystemStateSnapshot bootState = system_state_get_snapshot();
    ESP_LOGI(TAG, "starting minimal radar task scaffold");
    ESP_LOGI(TAG,
             "bootstrap identity device_id=%u device_sn=%" PRIu64,
             (unsigned)bootState.current_device_id,  // 设备 ID（32 位）
             bootState.device_sn);                    // 设备序列号（64 位）

    // ------------------------------------------------------------------------
    // 步骤 5: 初始化雷达管理器
    // ------------------------------------------------------------------------
    /**
     * 雷达管理器负责：
     * - 控制毫米波雷达传感器
     * - 采集心率、呼吸率、睡眠状态等数据
     * - 提供原始传感器数据
     */
    if (!initRadarManager()) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "radar manager init failed");
        return;  // 雷达初始化失败，无法继续运行
    }

    // ------------------------------------------------------------------------
    // 步骤 6: 初始化所有后台任务
    // ------------------------------------------------------------------------
    /**
     * 创建并启动多个后台任务（FreeRTOS 任务）：
     * - wifi_manager_task: WiFi 连接管理
     * - ble_config_task: BLE 配置和数据传输
     * - mqtt_connect_task: MQTT 连接和数据上传
     * - influx_post_task: InfluxDB 数据上传
     * - radar_command_task: 雷达命令处理
     * - system_status_task: 系统状态监控
     */
    if (!initAllTasks()) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "task init failed");
        return;  // 任务初始化失败
    }

    // ------------------------------------------------------------------------
    // 步骤 7: 配置并初始化 MQTT 客户端
    // ------------------------------------------------------------------------
    if (!mqtt_manager_init()) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "mqtt manager init failed");
        return;  // MQTT 初始化失败
    }

    // 构建 MQTT 客户端 ID
    char mqttClientId[64] = {};
    if (APP_MQTT_CLIENT_ID[0] != '\0') {
        // 使用配置文件中指定的客户端 ID
        std::snprintf(mqttClientId, sizeof(mqttClientId), "%s", APP_MQTT_CLIENT_ID);
    } else if (!radar_manager_get_default_mqtt_client_id(mqttClientId, sizeof(mqttClientId))) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "default mqtt client id build failed");
        return;
    }

    // 配置 MQTT 服务器地址和客户端 ID
    if (!mqtt_manager_configure(APP_MQTT_BROKER_URI, mqttClientId)) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "mqtt manager configure failed");
        return;
    }

    // ------------------------------------------------------------------------
    // 步骤 8: 打印设备身份信息
    // ------------------------------------------------------------------------
    /**
     * 收集并打印设备的完整身份信息，用于调试和云端注册
     */
    char deviceName[32] = {};      // 设备名称（如 ESP32_XXXX）
    char deviceMac[18] = {};       // MAC 地址（如 AA:BB:CC:DD:EE:FF）
    const bool hasDeviceName = radar_manager_get_device_name(deviceName, sizeof(deviceName));
    const bool hasDeviceMac = radar_manager_get_device_mac(deviceMac, sizeof(deviceMac));
    const uint32_t deviceHash = radar_manager_generate_device_hash();  // 设备哈希值

    ESP_LOGI(TAG,
             "identity name=%s mac=%s hash=0x%08X mqtt_client_id=%s",
             hasDeviceName ? deviceName : "-",  // 如果获取失败显示 "-"
             hasDeviceMac ? deviceMac : "-",
             static_cast<unsigned>(deviceHash),
             mqttClientId);
    ESP_LOGI(TAG, "mqtt configured uri=%s client_id=%s", APP_MQTT_BROKER_URI, mqttClientId);
    if (!voice_manager_init()) {
        ESP_LOGW(TAG, "voice manager init skipped or failed");
    }

    // ------------------------------------------------------------------------
    // 步骤 9: 设置系统为运行阶段
    // ------------------------------------------------------------------------
    system_state_set_phase(SYSTEM_PHASE_RUNNING);  // 系统状态机切换到"运行中"阶段
    ESP_LOGI(TAG, "system bootstrap complete");    // 启动流程完成
    
    // 注意：app_main() 函数在此结束，但系统不会停止
    // 所有后台任务（WiFi/BLE/MQTT 等）会继续独立运行
}

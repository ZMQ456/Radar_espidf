/**
 * @file device_identity.cpp
 * @brief 设备身份管理模块
 * 
 * 功能概述：
 * - 管理设备的唯一身份标识（device_id, device_sn）
 * - 基于 NVS（Non-Volatile Storage）持久化存储
 * - 支持读取、设置、重置设备身份参数
 * 
 * 核心特性：
 * - 掉电保护：设备身份参数存储在 NVS 中，重启不丢失
 * - 默认值：首次启动时自动使用默认值并保存
 * - 线程安全：通过系统状态模块同步设备身份
 * 
 * 数据存储：
 * - Namespace: "device_cfg"
 * - Key: "deviceId" (uint16_t), "deviceSn" (uint64_t)
 * 
 * 使用场景：
 * - 设备出厂时设置唯一序列号
 * - 云端注册时使用 device_id 作为标识
 * - 本地调试时查看/修改设备身份
 */
// ============================================================================
// 头文件包含和全局变量
// ============================================================================
#include "device_identity.h"  // 设备身份接口声明

#include "app_config.h"       // 应用配置（默认设备 ID 和序列号）
#include "esp_err.h"          // ESP32 错误码定义
#include "esp_log.h"          // ESP32 日志系统
#include "nvs.h"              // NVS 底层接口
#include "nvs_flash.h"        // NVS Flash 管理
#include "system_state.h"     // 系统状态管理（同步设备身份）

static const char *TAG = "device_identity";  // 日志标签

// NVS 存储键值
static const char *DEVICE_IDENTITY_NAMESPACE = "device_cfg";     // 命名空间
static const char *DEVICE_IDENTITY_KEY_ID = "deviceId";          // 设备 ID 键名
static const char *DEVICE_IDENTITY_KEY_SN = "deviceSn";          // 序列号键名

// 全局设备身份缓存（避免频繁读取 NVS）
static DeviceIdentity gDeviceIdentity = {
    APP_DEVICE_ID_DEFAULT,     // 默认设备 ID（来自配置文件）
    APP_DEVICE_SN_DEFAULT,     // 默认序列号（来自配置文件）
};
static bool gDeviceIdentityInitialized = false;  // 初始化标志

// ============================================================================
// 内部辅助函数
// ============================================================================
/**
 * @brief 确保 NVS 存储已初始化
 * 
 * @return true NVS 初始化成功
 * @return false NVS 初始化失败
 * 
 * 处理流程：
 * 1. 调用 nvs_flash_init() 初始化 NVS
 * 2. 如果没有可用空间或版本不匹配，擦除并重新初始化
 * 3. 处理错误情况（除 ESP_ERR_INVALID_STATE 外都视为失败）
 * 
 * 错误处理：
 * - ESP_ERR_NVS_NO_FREE_PAGES: NVS 空间不足，擦除重来
 * - ESP_ERR_NVS_NEW_VERSION_FOUND: NVS 版本不匹配，擦除重来
 * - ESP_ERR_INVALID_STATE: 已初始化，可以接受
 * 
 * 注意：此函数只在系统启动时调用一次
 */
static bool device_identity_ensure_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    
    // 处理特殊情况：空间不足或版本不匹配
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());  // 擦除 NVS
        err = nvs_flash_init();              // 重新初始化
    }

    // 检查最终结果（允许已初始化的状态）
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

/**
 * @brief 将当前设备身份持久化到 NVS
 * 
 * @return true 保存成功
 * @return false 保存失败
 * 
 * 处理流程：
 * 1. 打开 NVS 命名空间（读写模式）
 * 2. 写入 device_id（uint16_t）
 * 3. 写入 device_sn（uint64_t）
 * 4. 提交事务（commit）
 * 5. 关闭 NVS 句柄
 * 6. 同步到系统状态模块
 * 
 * 错误处理：
 * - NVS 打开失败：记录错误并返回
 * - 写入失败：记录错误并返回
 * - 提交失败：记录错误并返回
 * 
 * 注意：此函数会被 set_device_id/set_device_sn 调用
 */
static bool device_identity_persist(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_IDENTITY_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open failed: %s", esp_err_to_name(err));
        return false;
    }

    // 写入 device_id
    err = nvs_set_u16(handle, DEVICE_IDENTITY_KEY_ID, gDeviceIdentity.device_id);
    if (err == ESP_OK) {
        // 写入 device_sn
        err = nvs_set_u64(handle, DEVICE_IDENTITY_KEY_SN, gDeviceIdentity.device_sn);
    }
    if (err == ESP_OK) {
        // 提交事务
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs persist failed: %s", esp_err_to_name(err));
        return false;
    }

    // 同步到系统状态模块
    system_state_set_device_id(gDeviceIdentity.device_id);
    system_state_set_device_sn(gDeviceIdentity.device_sn);
    return true;
}

// ============================================================================
// 公共接口函数
// ============================================================================
/**
 * @brief 初始化设备身份模块
 * 
 * @return true 初始化成功
 * @return false 初始化失败（NVS 错误）
 * 
 * 处理流程：
 * 1. 检查是否已初始化（避免重复初始化）
 * 2. 确保 NVS 已初始化
 * 3. 打开 NVS 命名空间
 * 4. 读取存储的 device_id 和 device_sn
 * 5. 如果不存在，使用默认值并保存
 * 6. 提交事务并关闭 NVS
 * 7. 更新全局缓存和系统状态
 * 
 * 首次启动行为：
 * - 从配置文件读取默认值（APP_DEVICE_ID_DEFAULT, APP_DEVICE_SN_DEFAULT）
 * - 写入 NVS 永久保存
 * - 后续启动从 NVS 读取
 * 
 * 错误处理：
 * - NVS 初始化失败：返回 false
 * - NVS 打开失败：返回 false
 * - 读取/写入失败：返回 false
 * 
 * 注意：此函数在系统启动时由 main.cpp 调用
 */
bool device_identity_init(void)
{
    // 避免重复初始化
    if (gDeviceIdentityInitialized) {
        system_state_set_device_id(gDeviceIdentity.device_id);
        system_state_set_device_sn(gDeviceIdentity.device_sn);
        return true;
    }

    // 确保 NVS 可用
    if (!device_identity_ensure_nvs()) {
        return false;
    }

    // 打开 NVS 命名空间
    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_IDENTITY_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open failed: %s", esp_err_to_name(err));
        return false;
    }

    // 初始化默认值
    uint16_t storedDeviceId = APP_DEVICE_ID_DEFAULT;
    uint64_t storedDeviceSn = APP_DEVICE_SN_DEFAULT;

    // 读取 device_id（如果不存在则写入默认值）
    err = nvs_get_u16(handle, DEVICE_IDENTITY_KEY_ID, &storedDeviceId);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = nvs_set_u16(handle, DEVICE_IDENTITY_KEY_ID, storedDeviceId);
    }
    
    // 读取 device_sn（如果不存在则写入默认值）
    if (err == ESP_OK) {
        err = nvs_get_u64(handle, DEVICE_IDENTITY_KEY_SN, &storedDeviceSn);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            storedDeviceSn = APP_DEVICE_SN_DEFAULT;
            err = nvs_set_u64(handle, DEVICE_IDENTITY_KEY_SN, storedDeviceSn);
        }
    }
    
    // 提交事务
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs load failed: %s", esp_err_to_name(err));
        return false;
    }

    // 更新全局缓存
    gDeviceIdentity.device_id = storedDeviceId;
    gDeviceIdentity.device_sn = storedDeviceSn;
    gDeviceIdentityInitialized = true;

    // 同步到系统状态
    system_state_set_device_id(gDeviceIdentity.device_id);
    system_state_set_device_sn(gDeviceIdentity.device_sn);

    ESP_LOGI(TAG,
             "loaded device identity device_id=%u device_sn=%llu",
             (unsigned)gDeviceIdentity.device_id,
             (unsigned long long)gDeviceIdentity.device_sn);
    return true;
}

/**
 * @brief 获取当前设备身份
 * 
 * @return DeviceIdentity 设备身份结构体（按值返回）
 * 
 * 返回内容：
 * - device_id: 16 位设备 ID
 * - device_sn: 64 位设备序列号
 * 
 * 注意：
 * - 返回的是全局缓存的副本，不会读取 NVS
 * - 线程安全（按值返回，无共享状态）
 * 
 * 使用示例：
 * @code
 * DeviceIdentity identity = device_identity_get();
 * ESP_LOGI(TAG, "device_id=%u", identity.device_id);
 * @endcode
 */
DeviceIdentity device_identity_get(void)
{
    return gDeviceIdentity;
}

/**
 * @brief 设置设备 ID 并持久化
 * 
 * @param device_id 新的设备 ID（建议范围 1000-65535）
 * @return true 设置成功
 * @return false 设置失败（NVS 写入错误）
 * 
 * 处理流程：
 * 1. 更新全局缓存
 * 2. 调用 device_identity_persist() 写入 NVS
 * 3. 同步到系统状态模块
 * 
 * 注意：
 * - 此函数会立即写入 NVS，频繁调用可能影响 Flash 寿命
 * - 建议范围 1000-65535（由 device_command.cpp 保证）
 * - 掉电后数据不丢失
 * 
 * 使用示例：
 * @code
 * if (device_identity_set_device_id(1234)) {
 *     ESP_LOGI(TAG, "device id set success");
 * }
 * @endcode
 */
bool device_identity_set_device_id(uint16_t device_id)
{
    gDeviceIdentity.device_id = device_id;
    return device_identity_persist();
}

/**
 * @brief 设置设备序列号并持久化
 * 
 * @param device_sn 新的设备序列号（64 位无符号整数）
 * @return true 设置成功
 * @return false 设置失败（NVS 写入错误）
 * 
 * 处理流程：
 * 1. 更新全局缓存
 * 2. 调用 device_identity_persist() 写入 NVS
 * 3. 同步到系统状态模块
 * 
 * 注意：
 * - 此函数会立即写入 NVS，频繁调用可能影响 Flash 寿命
 * - 序列号应该是全局唯一的
 * - 掉电后数据不丢失
 * 
 * 使用示例：
 * @code
 * if (device_identity_set_device_sn(9876543210ULL)) {
 *     ESP_LOGI(TAG, "device sn set success");
 * }
 * @endcode
 */
bool device_identity_set_device_sn(uint64_t device_sn)
{
    gDeviceIdentity.device_sn = device_sn;
    return device_identity_persist();
}

/**
 * @brief 重置设备身份为默认值
 * 
 * @return true 重置成功
 * @return false 重置失败（NVS 写入错误）
 * 
 * 处理流程：
 * 1. 恢复默认 device_id（APP_DEVICE_ID_DEFAULT）
 * 2. 恢复默认 device_sn（APP_DEVICE_SN_DEFAULT）
 * 3. 调用 device_identity_persist() 写入 NVS
 * 4. 同步到系统状态模块
 * 
 * 使用场景：
 * - 设备恢复出厂设置
 * - 测试环境重置身份
 * 
 * 注意：
 * - 此操作会覆盖 NVS 中存储的身份信息
 * - 重置后需要重新配置设备身份
 * 
 * 使用示例：
 * @code
 * if (device_identity_reset_defaults()) {
 *     ESP_LOGI(TAG, "device identity reset to defaults");
 * }
 * @endcode
 */
bool device_identity_reset_defaults(void)
{
    gDeviceIdentity.device_id = APP_DEVICE_ID_DEFAULT;
    gDeviceIdentity.device_sn = APP_DEVICE_SN_DEFAULT;
    return device_identity_persist();
}

/**
 * @brief 仅重置设备 ID 为默认值
 * 
 * @return true 重置成功
 * @return false 重置失败（NVS 写入错误）
 * 
 * 处理流程：
 * 1. 恢复默认 device_id（APP_DEVICE_ID_DEFAULT）
 * 2. 保持 device_sn 不变
 * 3. 调用 device_identity_persist() 写入 NVS
 * 4. 同步到系统状态模块
 * 
 * 使用场景：
 * - 仅重置设备 ID，保留序列号
 * - 重新分配设备 ID
 * 
 * 注意：
 * - 此操作只影响 device_id，不影响 device_sn
 * - 掉电后数据不丢失
 * 
 * 使用示例：
 * @code
 * if (device_identity_reset_device_id_default()) {
 *     ESP_LOGI(TAG, "device id reset to default");
 * }
 * @endcode
 */
bool device_identity_reset_device_id_default(void)
{
    gDeviceIdentity.device_id = APP_DEVICE_ID_DEFAULT;
    return device_identity_persist();
}

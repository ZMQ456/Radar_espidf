/**
 * @file radar_platform.c
 * @brief ESP32 平台相关工具函数
 * 
 * 功能概述：
 * - 提供统一的时间获取接口
 * - 提供任务延时接口
 * - 提供系统重启接口
 * 
 * 核心函数：
 * 1. radar_now_ms(): 获取系统启动以来的毫秒数
 * 2. radar_sleep_ms(): 任务延时（毫秒级）
 * 3. radar_restart(): 系统重启
 * 
 * 设计目的：
 * - 封装 ESP32 SDK 函数，提供统一的接口
 * - 便于代码移植和维护
 * - 避免在业务代码中直接调用 SDK 函数
 * 
 * 依赖：
 * - esp_timer: 高精度定时器（微秒级）
 * - FreeRTOS: 实时操作系统（任务调度）
 * - esp_system: 系统控制（重启）
 */
// ============================================================================
// 头文件包含
// ============================================================================
#include "radar_platform.h"  // 平台接口声明

#include "esp_system.h"      // ESP32 系统控制（重启）
#include "esp_timer.h"       // ESP32 高精度定时器（微秒级）
#include "freertos/FreeRTOS.h" // FreeRTOS 接口
#include "freertos/task.h"     // FreeRTOS 任务管理（延时）

// ============================================================================
// 平台工具函数
// ============================================================================
/**
 * @brief 获取系统启动以来的时间（毫秒）
 * 
 * @return uint64_t 从系统启动到现在的毫秒数
 * 
 * 实现原理：
 * - 使用 esp_timer_get_time() 获取微秒数
 * - 除以 1000 转换为毫秒
 * 
 * 特点：
 * - 高精度（微秒级定时器）
 * - 单调递增（不会回退）
 * - 系统重启后归零
 * 
 * 用途：
 * - 计算时间间隔
 * - 超时判断
 * - 数据新鲜度检查
 * - 性能分析
 * 
 * 示例：
 * @code
 * uint64_t start = radar_now_ms();
 * // ... 执行某些操作 ...
 * uint64_t elapsed = radar_now_ms() - start;
 * ESP_LOGI(TAG, "耗时：%llu ms", (unsigned long long)elapsed);
 * @endcode
 */
uint64_t radar_now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief 任务延时（毫秒）
 * 
 * @param delay_ms 延时时间（毫秒）
 * 
 * 实现原理：
 * - 使用 FreeRTOS 的 vTaskDelay()
 * - 将毫秒转换为系统 tick（1 tick = 10ms @ CONFIG_FREERTOS_HZ=100）
 * 
 * 特点：
 * - 阻塞当前任务
 * - 让出 CPU 使用权
 * - 其他任务可以继续运行
 * - 精确度取决于系统 tick 频率
 * 
 * 注意：
 * - 在中断服务程序中不能使用
 * - 实际延时可能略长于指定值（向上取整到 tick）
 * 
 * 示例：
 * @code
 * // 延时 1 秒
 * radar_sleep_ms(1000);
 * 
 * // 延时 500 毫秒
 * radar_sleep_ms(500);
 * @endcode
 */
void radar_sleep_ms(uint32_t delay_ms)
{
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

/**
 * @brief 系统重启
 * 
 * 实现原理：
 * - 调用 esp_restart() 重启 ESP32
 * - 相当于硬件复位
 * 
 * 特点：
 * - 立即执行（不会返回）
 * - 所有外设重新初始化
 * - NVS 数据保持不变
 * - RTC 内存数据保持不变
 * 
 * 用途：
 * - 系统故障恢复
 * - 配置更改后重启
 * - 看门狗超时后的软件重启
 * 
 * 注意：
 * - 此函数不会返回
 * - 重启前无法执行清理操作
 * - 建议在重启前记录日志
 * 
 * 示例：
 * @code
 * if (system_critical_error()) {
 *     ESP_LOGE(TAG, "critical error, restarting...");
 *     radar_restart();
 *     // 不会执行到这里
 * }
 * @endcode
 */
void radar_restart(void)
{
    esp_restart();
}

/**
 * @file system_state.cpp
 * @brief 系统状态管理模块
 * 
 * 功能概述：
 * - 提供全局系统状态的集中管理
 * - 跟踪各子系统就绪状态（雷达、任务、BLE、WiFi、MQTT）
 * - 维护系统运行阶段和网络状态
 * - 提供设备标识信息（设备 ID、序列号）
 * - 计算系统运行时间
 * 
 * 系统状态组成：
 * 1. 系统阶段（SystemPhase）：
 *    - SYSTEM_PHASE_BOOT: 启动阶段
 *    - SYSTEM_PHASE_NORMAL: 正常运行
 *    - SYSTEM_PHASE_ERROR: 错误状态
 * 
 * 2. 网络状态（NetworkStatus）：
 *    - NETWORK_STATUS_INITIAL: 初始状态
 *    - NETWORK_STATUS_DISCONNECTED: 断开连接
 *    - NETWORK_STATUS_CONNECTING: 连接中
 *    - NETWORK_STATUS_CONNECTED: 已连接
 * 
 * 3. 子系统就绪标志（5 个）：
 *    - radar_ready: 雷达传感器就绪
 *    - tasks_ready: 多任务管理器就绪
 *    - ble_ready: BLE 模块就绪
 *    - wifi_ready: WiFi 模块就绪
 *    - mqtt_ready: MQTT 客户端就绪
 * 
 * 4. 设备标识：
 *    - current_device_id: 设备 ID（16 位）
 *    - device_sn: 设备序列号（64 位）
 * 
 * 5. 时间信息：
 *    - boot_time_ms: 启动时间戳
 *    - uptime_ms: 运行时间（动态计算）
 * 
 * 6. 错误标志：
 *    - error: 系统错误状态
 * 
 * 设计思想：
 * - 单例模式：全局唯一的系统状态实例
 * - 快照机制：获取状态时返回副本，避免竞争
 * - 实时计算：运行时间在获取时动态计算
 * - 错误优先：设置错误标志时自动切换到错误阶段
 * 
 * 依赖模块：
 * - app_config.h: 设备默认配置
 * - radar_platform.h: 时间函数（radar_now_ms）
 * - system_state.h: 状态数据结构定义
 */
#include "system_state.h"

#include "app_config.h"
#include "radar_platform.h"

// 全局系统状态实例（单例模式）
static SystemStateSnapshot gSystemState = {};

/**
 * @brief 初始化系统状态
 * 
 * 处理流程：
 * 1. 设置系统阶段为启动阶段（SYSTEM_PHASE_BOOT）
 * 2. 设置网络状态为初始状态（NETWORK_STATUS_INITIAL）
 * 3. 清零所有就绪标志（雷达、任务、BLE、WiFi、MQTT）
 * 4. 清零错误标志
 * 5. 设置默认设备 ID 和序列号
 * 6. 记录启动时间戳
 * 7. 初始化运行时间为 0
 * 
 * 调用时机：
 * - 系统启动时调用一次
 * - 在所有模块初始化之前调用
 * 
 * 注意：
 * - 此函数不重置设备 ID 和序列号（保持默认值）
 * - 启动时间戳使用 radar_now_ms() 获取
 */
void system_state_init(void)
{
    // 步骤 1: 设置系统阶段
    gSystemState.phase = SYSTEM_PHASE_BOOT;
    
    // 步骤 2: 设置网络状态
    gSystemState.network_status = NETWORK_STATUS_INITIAL;
    
    // 步骤 3: 清零所有就绪标志
    gSystemState.radar_ready = 0U;
    gSystemState.tasks_ready = 0U;
    gSystemState.ble_ready = 0U;
    gSystemState.wifi_ready = 0U;
    gSystemState.mqtt_ready = 0U;
    
    // 步骤 4: 清零错误标志
    gSystemState.error = 0U;
    
    // 步骤 5: 设置默认设备标识
    gSystemState.current_device_id = APP_DEVICE_ID_DEFAULT;
    gSystemState.device_sn = APP_DEVICE_SN_DEFAULT;
    
    // 步骤 6: 记录启动时间戳
    gSystemState.boot_time_ms = radar_now_ms();
    
    // 步骤 7: 初始化运行时间
    gSystemState.uptime_ms = 0U;
}

/**
 * @brief 设置系统运行阶段
 * 
 * @param phase 新的系统阶段
 * 
 * 系统阶段说明：
 * - SYSTEM_PHASE_BOOT: 启动阶段（初始化中）
 * - SYSTEM_PHASE_NORMAL: 正常运行阶段
 * - SYSTEM_PHASE_ERROR: 错误阶段（发生严重错误）
 * 
 * 典型转换流程：
 * BOOT → NORMAL（初始化完成）
 * NORMAL → ERROR（发生错误）
 * 
 * 注意：
 * - 错误阶段只能通过系统重启退出
 * - 设置错误标志会自动切换到错误阶段
 */
void system_state_set_phase(SystemPhase phase)
{
    gSystemState.phase = phase;
}

/**
 * @brief 设置雷达就绪标志
 * 
 * @param ready 就绪状态（0=未就绪，非 0=就绪）
 * 
 * 功能说明：
 * - 雷达传感器初始化完成后设置为就绪
 * - 雷达故障或断开时设置为未就绪
 * 
 * 用途：
 * - 其他模块检查雷达可用性
 * - 系统状态日志输出
 */
void system_state_set_radar_ready(uint8_t ready)
{
    gSystemState.radar_ready = ready ? 1U : 0U;
}

/**
 * @brief 设置任务管理器就绪标志
 * 
 * @param ready 就绪状态（0=未就绪，非 0=就绪）
 * 
 * 功能说明：
 * - 所有 FreeRTOS 任务创建完成后设置为就绪
 * - 表示多任务调度系统已启动
 * 
 * 用途：
 * - 检查系统任务调度是否正常
 */
void system_state_set_tasks_ready(uint8_t ready)
{
    gSystemState.tasks_ready = ready ? 1U : 0U;
}

/**
 * @brief 设置 BLE 就绪标志
 * 
 * @param ready 就绪状态（0=未就绪，非 0=就绪）
 * 
 * 功能说明：
 * - BLE 管理器初始化完成后设置为就绪
 * - BLE 故障时设置为未就绪
 * 
 * 用途：
 * - 检查 BLE 功能可用性
 * - 决定是否启动 BLE 配置任务
 */
void system_state_set_ble_ready(uint8_t ready)
{
    gSystemState.ble_ready = ready ? 1U : 0U;
}

/**
 * @brief 设置 WiFi 就绪标志
 * 
 * @param ready 就绪状态（0=未就绪，非 0=就绪）
 * 
 * 功能说明：
 * - WiFi 管理器初始化完成后设置为就绪
 * - WiFi 故障时设置为未就绪
 * 
 * 用途：
 * - 检查 WiFi 功能可用性
 * - 决定是否启动 WiFi 监控任务
 */
void system_state_set_wifi_ready(uint8_t ready)
{
    gSystemState.wifi_ready = ready ? 1U : 0U;
}

/**
 * @brief 设置 MQTT 就绪标志
 * 
 * @param ready 就绪状态（0=未就绪，非 0=就绪）
 * 
 * 功能说明：
 * - MQTT 管理器初始化完成后设置为就绪
 * - MQTT 故障时设置为未就绪
 * 
 * 用途：
 * - 检查 MQTT 功能可用性
 * - 决定是否启动数据上报
 */
void system_state_set_mqtt_ready(uint8_t ready)
{
    gSystemState.mqtt_ready = ready ? 1U : 0U;
}

/**
 * @brief 设置系统错误标志
 * 
 * @param error 错误状态（0=无错误，非 0=有错误）
 * 
 * 功能说明：
 * - 设置错误标志时自动切换到错误阶段
 * - 错误阶段表示系统发生严重故障
 * 
 * 副作用：
 * - 如果 error 非 0，自动设置 phase = SYSTEM_PHASE_ERROR
 * - 错误阶段只能通过重启退出
 * 
 * 用途：
 * - 报告系统级错误
 * - 触发错误处理机制
 */
void system_state_set_error(uint8_t error)
{
    gSystemState.error = error ? 1U : 0U;
    if (gSystemState.error) {
        gSystemState.phase = SYSTEM_PHASE_ERROR;  // 自动切换到错误阶段
    }
}

/**
 * @brief 设置网络状态
 * 
 * @param status 新的网络状态
 * 
 * 网络状态说明：
 * - NETWORK_STATUS_INITIAL: 初始状态（未开始连接）
 * - NETWORK_STATUS_DISCONNECTED: 断开连接（连接失败或已断开）
 * - NETWORK_STATUS_CONNECTING: 连接中（正在尝试连接）
 * - NETWORK_STATUS_CONNECTED: 已连接（网络连接成功）
 * 
 * 用途：
 * - 控制 LED 指示灯模式
 * - 决定是否发送网络数据
 * - 系统状态日志输出
 */
void system_state_set_network_status(NetworkStatus status)
{
    gSystemState.network_status = status;
}

/**
 * @brief 设置设备 ID
 * 
 * @param device_id 新的设备 ID（16 位无符号整数）
 * 
 * 功能说明：
 * - 设备 ID 用于在网络中唯一标识设备
 * - 可从 NVS 存储加载或用户配置
 * 
 * 用途：
 * - MQTT 主题标识
 * - 服务器端设备识别
 * - 日志输出
 */
void system_state_set_device_id(uint16_t device_id)
{
    gSystemState.current_device_id = device_id;
}

/**
 * @brief 设置设备序列号
 * 
 * @param device_sn 新的设备序列号（64 位无符号整数）
 * 
 * 功能说明：
 * - 序列号是设备的唯一硬件标识
 * - 通常在生产时烧录
 * 
 * 用途：
 * - 设备追溯
 * - 防伪验证
 * - 高级设备管理
 */
void system_state_set_device_sn(uint64_t device_sn)
{
    gSystemState.device_sn = device_sn;
}

/**
 * @brief 获取系统状态快照
 * 
 * @return SystemStateSnapshot 系统状态快照（副本）
 * 
 * 功能说明：
 * - 返回当前系统状态的完整副本
 * - 自动计算运行时间（uptime_ms）
 * - 线程安全（返回副本，避免竞争）
 * 
 * 处理流程：
 * 1. 复制全局状态结构到局部变量
 * 2. 获取当前时间戳
 * 3. 计算运行时间 = 当前时间 - 启动时间
 * 4. 处理时间回绕（当前时间 < 启动时间时返回 0）
 * 5. 返回快照
 * 
 * 用途：
 * - 系统状态监控
 * - 日志输出
 * - 远程状态报告
 * 
 * 注意：
 * - 返回的是副本，修改快照不影响全局状态
 * - 运行时间是实时计算的，每次调用可能不同
 * - 处理了时间回绕情况（系统运行约 583 天后可能回绕）
 */
SystemStateSnapshot system_state_get_snapshot(void)
{
    // 步骤 1: 复制全局状态
    SystemStateSnapshot snapshot = gSystemState;
    
    // 步骤 2: 获取当前时间戳
    const uint64_t nowMs = radar_now_ms();
    
    // 步骤 3-4: 计算运行时间（处理时间回绕）
    if (nowMs >= snapshot.boot_time_ms) {
        snapshot.uptime_ms = nowMs - snapshot.boot_time_ms;
    } else {
        snapshot.uptime_ms = 0U;  // 时间回绕保护
    }
    
    // 步骤 5: 返回快照
    return snapshot;
}

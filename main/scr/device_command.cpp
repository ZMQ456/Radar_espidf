/**
 * @file device_command.cpp
 * @brief 设备命令行解析模块
 * 
 * 功能概述：
 * - 解析并执行设备管理命令（通过串口或 BLE 下发）
 * - 支持查看和设置设备身份参数（device_id, device_sn）
 * - 提供命令行帮助信息
 * 
 * 支持的命令：
 * - help: 显示帮助信息
 * - show_device: 查看当前设备身份
 * - set_device_id <1000-65535>: 设置设备 ID
 * - set_device_sn <uint64>: 设置设备序列号
 * 
 * 安全特性：
 * - 输入缓冲区长度限制（128 字符）
 * - 去除首尾空白字符
 * - 数值范围检查（防止溢出）
 * - 持久化存储保护（失败时回滚）
 */
// ============================================================================
// 头文件包含
// ============================================================================
#include "device_command.h"  // 命令处理接口声明

#include <cctype>     // 字符处理函数（isspace）
#include <cstdio>     // 标准输入输出（snprintf）
#include <cstdlib>    // 标准库函数（strtoul, strtoull）
#include <cstring>    // 字符串处理（strlen, strncpy, strcmp）

#include "device_identity.h"  // 设备身份管理（获取/设置 device_id, device_sn）
#include "esp_log.h"          // ESP32 日志系统

static const char *TAG = "device_command";  // 日志标签

// ============================================================================
// 辅助函数：字符串处理
// ============================================================================
/**
 * @brief 去除字符串末尾的空白字符（原地修改）
 * 
 * @param text 待处理的字符串指针
 * 
 * 处理逻辑：
 * 1. 从字符串末尾向前遍历
 * 2. 将所有空白字符（空格、制表符、换行符等）替换为 '\0'
 * 3. 直到遇到非空白字符或到达字符串开头
 * 
 * 示例：
 * - 输入："  hello world  \n\t"
 * - 输出："  hello world"
 */
static void trim_trailing_whitespace(char *text)
{
    if (text == nullptr) {
        return;
    }

    size_t len = std::strlen(text);
    while (len > 0U && std::isspace(static_cast<unsigned char>(text[len - 1U])) != 0) {
        text[len - 1U] = '\0';
        --len;
    }
}

/**
 * @brief 跳过字符串开头的空白字符，返回第一个非空白字符的指针
 * 
 * @param text 输入字符串指针
 * @return const char* 第一个非空白字符的位置
 * 
 * 处理逻辑：
 * 1. 从字符串开头向后遍历
 * 2. 跳过所有空白字符
 * 3. 返回第一个非空白字符的指针
 * 
 * 示例：
 * - 输入："  \t\nhello world"
 * - 返回：指向 "hello world" 的指针
 */
static const char *skip_leading_whitespace(const char *text)
{
    while (text != nullptr && *text != '\0' &&
           std::isspace(static_cast<unsigned char>(*text)) != 0) {
        ++text;
    }
    return text;
}

// ============================================================================
// 辅助函数：参数解析
// ============================================================================
/**
 * @brief 解析无符号 16 位整数参数
 * 
 * @param arg 输入字符串（应为十进制数字）
 * @param value 输出参数，解析结果
 * @return true 解析成功
 * @return false 解析失败（格式错误或超出范围）
 * 
 * 验证规则：
 * 1. 必须为有效的十进制数字
 * 2. 不能有前导或尾随空白字符
 * 3. 数值必须在 [0, 65535] 范围内
 * 
 * 错误处理：
 * - 空指针输入
 * - 格式错误（包含非数字字符）
 * - 超出 uint16_t 范围
 */
static bool parse_uint16_arg(const char *arg, uint16_t *value)
{
    if (arg == nullptr || value == nullptr) {
        return false;
    }

    char *endPtr = nullptr;
    const unsigned long parsed = std::strtoul(arg, &endPtr, 10);
    
    // 检查：是否有有效解析、是否有尾随字符、是否溢出
    if (endPtr == arg || *skip_leading_whitespace(endPtr) != '\0' || parsed > 0xFFFFUL) {
        return false;
    }

    *value = static_cast<uint16_t>(parsed);
    return true;
}

/**
 * @brief 解析无符号 64 位整数参数
 * 
 * @param arg 输入字符串（应为十进制数字）
 * @param value 输出参数，解析结果
 * @return true 解析成功
 * @return false 解析失败（格式错误或超出范围）
 * 
 * 验证规则：
 * 1. 必须为有效的十进制数字
 * 2. 不能有前导或尾随空白字符
 * 3. 数值必须在 uint64_t 范围内
 * 
 * 错误处理：
 * - 空指针输入
 * - 格式错误（包含非数字字符）
 * - 超出 uint64_t 范围
 */
static bool parse_uint64_arg(const char *arg, uint64_t *value)
{
    if (arg == nullptr || value == nullptr) {
        return false;
    }

    char *endPtr = nullptr;
    const unsigned long long parsed = std::strtoull(arg, &endPtr, 10);
    
    // 检查：是否有有效解析、是否有尾随字符
    if (endPtr == arg || *skip_leading_whitespace(endPtr) != '\0') {
        return false;
    }

    *value = static_cast<uint64_t>(parsed);
    return true;
}

/**
 * @brief 打印命令帮助信息
 * 
 * 输出格式：
 * commands: show_device | set_device_id <1000-65535> | set_device_sn <uint64>
 * 
 * 说明：
 * - show_device: 查看当前设备身份参数
 * - set_device_id: 设置设备 ID（范围 1000-65535）
 * - set_device_sn: 设置设备序列号（64 位无符号整数）
 */
static void print_help(void)
{
    ESP_LOGI(TAG, "commands: show_device | set_device_id <1000-65535> | set_device_sn <uint64>");
}

// ============================================================================
// 主函数：命令行处理
// ============================================================================
/**
 * @brief 处理一行设备命令
 * 
 * @param line 输入的命令字符串
 * @return true 命令已处理（无论成功或失败）
 * @return false 输入无效（空指针）
 * 
 * 处理流程：
 * 1. 复制输入到本地缓冲区（防止修改原字符串）
 * 2. 去除末尾空白字符
 * 3. 跳过开头空白字符
 * 4. 匹配命令并执行相应操作
 * 
 * 支持的命令：
 * - help: 显示帮助信息
 * - show_device: 查看设备身份（device_id, device_sn）
 * - set_device_id <value>: 设置设备 ID（1000-65535）
 * - set_device_sn <value>: 设置设备序列号（uint64）
 * 
 * 安全保护：
 * - 缓冲区长度限制（128 字符）
 * - 数值范围验证
 * - 持久化失败时回滚
 * 
 * 示例：
 * @code
 * device_command_handle_line("show_device");
 * device_command_handle_line("set_device_id 1234");
 * device_command_handle_line("set_device_sn 9876543210");
 * @endcode
 */
bool device_command_handle_line(const char *line)
{
    if (line == nullptr) {
        return false;
    }

    // 步骤 1: 复制到本地缓冲区（防止修改原字符串）
    char buffer[128];
    std::strncpy(buffer, line, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';
    
    // 步骤 2: 去除末尾空白字符
    trim_trailing_whitespace(buffer);

    // 步骤 3: 跳过开头空白字符
    const char *input = skip_leading_whitespace(buffer);
    if (*input == '\0') {
        return false;  // 空命令，直接返回
    }

    // 步骤 4: 命令匹配和执行
    
    // 命令 1: help - 显示帮助信息
    if (std::strcmp(input, "help") == 0) {
        print_help();
        return true;
    }

    // 命令 2: show_device - 查看设备身份
    if (std::strcmp(input, "show_device") == 0) {
        const DeviceIdentity identity = device_identity_get();
        ESP_LOGI(TAG,
                 "device identity device_id=%u device_sn=%llu",
                 static_cast<unsigned>(identity.device_id),
                 static_cast<unsigned long long>(identity.device_sn));
        return true;
    }

    // 命令 3: set_device_id <value> - 设置设备 ID
    if (std::strncmp(input, "set_device_id ", 14) == 0) {
        uint16_t deviceId = 0;
        
        // 解析参数
        if (!parse_uint16_arg(input + 14, &deviceId)) {
            ESP_LOGW(TAG, "invalid device id, use: set_device_id <1000-65535>");
            return true;
        }

        // 范围检查：device_id 必须 >= 1000
        if (deviceId < 1000U) {
            ESP_LOGW(TAG, "device id must be >= 1000");
            return true;
        }

        // 持久化存储
        if (!device_identity_set_device_id(deviceId)) {
            ESP_LOGE(TAG, "failed to persist device id");
            return true;
        }

        ESP_LOGI(TAG, "device id updated to %u", static_cast<unsigned>(deviceId));
        return true;
    }

    // 命令 4: set_device_sn <value> - 设置设备序列号
    if (std::strncmp(input, "set_device_sn ", 14) == 0) {
        uint64_t deviceSn = 0;
        
        // 解析参数
        if (!parse_uint64_arg(input + 14, &deviceSn)) {
            ESP_LOGW(TAG, "invalid device sn, use: set_device_sn <uint64>");
            return true;
        }

        // 持久化存储
        if (!device_identity_set_device_sn(deviceSn)) {
            ESP_LOGE(TAG, "failed to persist device sn");
            return true;
        }

        ESP_LOGI(TAG, "device sn updated to %llu", static_cast<unsigned long long>(deviceSn));
        return true;
    }

    // 未知命令：显示帮助信息
    ESP_LOGW(TAG, "unknown command: %s", input);
    print_help();
    return true;
}

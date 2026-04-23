#include "device_command.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "device_identity.h"
#include "esp_log.h"

static const char *TAG = "device_command";

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

static const char *skip_leading_whitespace(const char *text)
{
    while (text != nullptr && *text != '\0' &&
           std::isspace(static_cast<unsigned char>(*text)) != 0) {
        ++text;
    }
    return text;
}

static bool parse_uint16_arg(const char *arg, uint16_t *value)
{
    if (arg == nullptr || value == nullptr) {
        return false;
    }

    char *endPtr = nullptr;
    const unsigned long parsed = std::strtoul(arg, &endPtr, 10);
    if (endPtr == arg || *skip_leading_whitespace(endPtr) != '\0' || parsed > 0xFFFFUL) {
        return false;
    }

    *value = static_cast<uint16_t>(parsed);
    return true;
}

static bool parse_uint64_arg(const char *arg, uint64_t *value)
{
    if (arg == nullptr || value == nullptr) {
        return false;
    }

    char *endPtr = nullptr;
    const unsigned long long parsed = std::strtoull(arg, &endPtr, 10);
    if (endPtr == arg || *skip_leading_whitespace(endPtr) != '\0') {
        return false;
    }

    *value = static_cast<uint64_t>(parsed);
    return true;
}

static void print_help(void)
{
    ESP_LOGI(TAG, "commands: show_device | set_device_id <1000-65535> | set_device_sn <uint64>");
}

bool device_command_handle_line(const char *line)
{
    if (line == nullptr) {
        return false;
    }

    char buffer[128];
    std::strncpy(buffer, line, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';
    trim_trailing_whitespace(buffer);

    const char *input = skip_leading_whitespace(buffer);
    if (*input == '\0') {
        return false;
    }

    if (std::strcmp(input, "help") == 0) {
        print_help();
        return true;
    }

    if (std::strcmp(input, "show_device") == 0) {
        const DeviceIdentity identity = device_identity_get();
        ESP_LOGI(TAG,
                 "device identity device_id=%u device_sn=%llu",
                 static_cast<unsigned>(identity.device_id),
                 static_cast<unsigned long long>(identity.device_sn));
        return true;
    }

    if (std::strncmp(input, "set_device_id ", 14) == 0) {
        uint16_t deviceId = 0;
        if (!parse_uint16_arg(input + 14, &deviceId)) {
            ESP_LOGW(TAG, "invalid device id, use: set_device_id <1000-65535>");
            return true;
        }

        if (deviceId < 1000U) {
            ESP_LOGW(TAG, "device id must be >= 1000");
            return true;
        }

        if (!device_identity_set_device_id(deviceId)) {
            ESP_LOGE(TAG, "failed to persist device id");
            return true;
        }

        ESP_LOGI(TAG, "device id updated to %u", static_cast<unsigned>(deviceId));
        return true;
    }

    if (std::strncmp(input, "set_device_sn ", 14) == 0) {
        uint64_t deviceSn = 0;
        if (!parse_uint64_arg(input + 14, &deviceSn)) {
            ESP_LOGW(TAG, "invalid device sn, use: set_device_sn <uint64>");
            return true;
        }

        if (!device_identity_set_device_sn(deviceSn)) {
            ESP_LOGE(TAG, "failed to persist device sn");
            return true;
        }

        ESP_LOGI(TAG, "device sn updated to %llu", static_cast<unsigned long long>(deviceSn));
        return true;
    }

    ESP_LOGW(TAG, "unknown command: %s", input);
    print_help();
    return true;
}

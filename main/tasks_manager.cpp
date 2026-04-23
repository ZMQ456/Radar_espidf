#include "tasks_manager.h"

#include <inttypes.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "app_config.h"
#include "ble_manager.h"
#include "device_identity.h"
#include "device_command.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "influx_manager.h"
#include "emotion_analyzer_simple.h"
#include "radar_manager.h"
#include "sleep_analyzer.h"
#include "radar_platform.h"
#include "radar_uart.h"
#include "system_state.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

static const char *TAG = "tasks_manager";

static TaskHandle_t radarCommandTaskHandle = nullptr;
static TaskHandle_t radarStatusTaskHandle = nullptr;
static TaskHandle_t radarAnalysisTaskHandle = nullptr;
static TaskHandle_t deviceConsoleTaskHandle = nullptr;
static TaskHandle_t bootButtonTaskHandle = nullptr;
static TaskHandle_t ledControlTaskHandle = nullptr;
static TaskHandle_t wifiMonitorTaskHandle = nullptr;
static TaskHandle_t bleConfigTaskHandle = nullptr;
static bool tasksInitialized = false;
TaskNetworkStatus currentNetworkStatus = NET_INITIAL;
static uint64_t gLastBlinkTimeMs = 0U;
static bool gLedState = false;
static uint32_t gBreatheValue = APP_LED_BREATHE_MIN;
static bool gBreatheIncreasing = true;
static bool gClearConfigRequested = false;
static bool gForceLedOff = false;
static bool gContinuousSendEnabled = false;
static bool gHasLastContinuousData = false;
static uint32_t gContinuousSendIntervalMs = 500U;
static uint64_t gLastContinuousCheckMs = 0U;
static SensorData gLastContinuousData = {};
static RadarReportSnapshot gLastInfluxDailySnapshot = {};
static bool gHasLastInfluxDailySnapshot = false;
static uint64_t gLastInfluxDailySendMs = 0U;
static uint64_t gLastInfluxSleepSendMs = 0U;
static uint64_t gLastMqttHeartbeatMs = 0U;
static uint64_t gLastMqttDailySendMs = 0U;
static uint64_t gLastMqttSleepSendMs = 0U;

static constexpr uint32_t kInfluxDailyForceSendIntervalMs = 60000U;
static constexpr uint32_t kInfluxDailyMinSendIntervalMs = 1500U;
static constexpr uint32_t kInfluxSleepSendIntervalMs = 5000U;
static constexpr uint32_t kMqttHeartbeatIntervalMs = 10000U;
static constexpr uint32_t kMqttDailyDataIntervalMs = 5000U;
static constexpr uint32_t kMqttSleepDataIntervalMs = 10000U;

static void task_watchdog_register_current(const char *task_name)
{
    const esp_err_t status = esp_task_wdt_status(nullptr);
    if (status == ESP_OK) {
        return;
    }
    if (status == ESP_ERR_INVALID_STATE) {
        return;
    }

    const esp_err_t err = esp_task_wdt_add(nullptr);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "watchdog add failed for %s: %s", task_name, esp_err_to_name(err));
    }
}

static void task_watchdog_reset_current(void)
{
    if (esp_task_wdt_status(nullptr) == ESP_OK) {
        (void)esp_task_wdt_reset();
    }
}

static const char *find_json_value_start(const char *json, const char *key)
{
    if (json == nullptr || key == nullptr) {
        return nullptr;
    }

    char pattern[64] = {};
    std::snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *keyPos = std::strstr(json, pattern);
    if (keyPos == nullptr) {
        return nullptr;
    }

    const char *colonPos = std::strchr(keyPos + std::strlen(pattern), ':');
    if (colonPos == nullptr) {
        return nullptr;
    }

    const char *valuePos = colonPos + 1;
    while (*valuePos == ' ' || *valuePos == '\t' || *valuePos == '\r' || *valuePos == '\n') {
        ++valuePos;
    }
    return valuePos;
}

static int json_hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static bool json_extract_string_value(const char *json,
                                      const char *key,
                                      char *buffer,
                                      size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return false;
    }

    const char *valuePos = find_json_value_start(json, key);
    if (valuePos == nullptr || *valuePos != '"') {
        return false;
    }

    ++valuePos;
    size_t outIndex = 0U;
    while (*valuePos != '\0' && outIndex < (buffer_size - 1U)) {
        if (*valuePos == '"') {
            buffer[outIndex] = '\0';
            return true;
        }

        if (*valuePos != '\\') {
            buffer[outIndex++] = *valuePos++;
            continue;
        }

        ++valuePos;
        if (*valuePos == '\0') {
            break;
        }

        switch (*valuePos) {
        case '"':
        case '\\':
        case '/':
            buffer[outIndex++] = *valuePos++;
            break;
        case 'b':
            buffer[outIndex++] = '\b';
            ++valuePos;
            break;
        case 'f':
            buffer[outIndex++] = '\f';
            ++valuePos;
            break;
        case 'n':
            buffer[outIndex++] = '\n';
            ++valuePos;
            break;
        case 'r':
            buffer[outIndex++] = '\r';
            ++valuePos;
            break;
        case 't':
            buffer[outIndex++] = '\t';
            ++valuePos;
            break;
        case 'u': {
            const int h0 = json_hex_nibble(valuePos[1]);
            const int h1 = json_hex_nibble(valuePos[2]);
            const int h2 = json_hex_nibble(valuePos[3]);
            const int h3 = json_hex_nibble(valuePos[4]);
            if (h0 < 0 || h1 < 0 || h2 < 0 || h3 < 0) {
                buffer[outIndex] = '\0';
                return false;
            }
            const unsigned codepoint = (unsigned)((h0 << 12) | (h1 << 8) | (h2 << 4) | h3);
            if (codepoint <= 0x7FU) {
                buffer[outIndex++] = static_cast<char>(codepoint);
            } else if (codepoint <= 0x7FFU && outIndex + 1U < buffer_size - 1U) {
                buffer[outIndex++] = static_cast<char>(0xC0U | (codepoint >> 6));
                buffer[outIndex++] = static_cast<char>(0x80U | (codepoint & 0x3FU));
            } else if (outIndex + 2U < buffer_size - 1U) {
                buffer[outIndex++] = static_cast<char>(0xE0U | (codepoint >> 12));
                buffer[outIndex++] = static_cast<char>(0x80U | ((codepoint >> 6) & 0x3FU));
                buffer[outIndex++] = static_cast<char>(0x80U | (codepoint & 0x3FU));
            }
            valuePos += 5;
            break;
        }
        default:
            buffer[outIndex++] = *valuePos++;
            break;
        }
    }
    buffer[outIndex] = '\0';
    return false;
}

static bool json_extract_uint16_value(const char *json, const char *key, uint16_t *value)
{
    if (value == nullptr) {
        return false;
    }

    const char *valuePos = find_json_value_start(json, key);
    if (valuePos == nullptr) {
        return false;
    }

    char *endPtr = nullptr;
    const unsigned long parsed = std::strtoul(valuePos, &endPtr, 10);
    if (endPtr == valuePos || parsed > 65535UL) {
        return false;
    }

    *value = static_cast<uint16_t>(parsed);
    return true;
}

static bool json_extract_uint32_value(const char *json, const char *key, uint32_t *value)
{
    if (value == nullptr) {
        return false;
    }

    const char *valuePos = find_json_value_start(json, key);
    if (valuePos == nullptr) {
        return false;
    }

    char *endPtr = nullptr;
    const unsigned long parsed = std::strtoul(valuePos, &endPtr, 10);
    if (endPtr == valuePos || parsed > 0xFFFFFFFFUL) {
        return false;
    }

    *value = static_cast<uint32_t>(parsed);
    return true;
}

static bool json_extract_uint64_value(const char *json, const char *key, uint64_t *value)
{
    if (value == nullptr) {
        return false;
    }

    const char *valuePos = find_json_value_start(json, key);
    if (valuePos == nullptr) {
        return false;
    }

    char *endPtr = nullptr;
    const unsigned long long parsed = std::strtoull(valuePos, &endPtr, 10);
    if (endPtr == valuePos) {
        return false;
    }

    *value = static_cast<uint64_t>(parsed);
    return true;
}

static void json_escape_string(const char *input, char *output, size_t output_size)
{
    if (output == nullptr || output_size == 0U) {
        return;
    }

    output[0] = '\0';
    if (input == nullptr) {
        return;
    }

    size_t used = 0U;
    for (size_t i = 0U; input[i] != '\0' && used < output_size - 1U; ++i) {
        const unsigned char c = static_cast<unsigned char>(input[i]);
        const char *escaped = nullptr;
        switch (c) {
        case '\"':
            escaped = "\\\"";
            break;
        case '\\':
            escaped = "\\\\";
            break;
        case '\b':
            escaped = "\\b";
            break;
        case '\f':
            escaped = "\\f";
            break;
        case '\n':
            escaped = "\\n";
            break;
        case '\r':
            escaped = "\\r";
            break;
        case '\t':
            escaped = "\\t";
            break;
        default:
            break;
        }

        if (escaped != nullptr) {
            const size_t len = std::strlen(escaped);
            if (used + len >= output_size) {
                break;
            }
            std::memcpy(output + used, escaped, len);
            used += len;
            continue;
        }

        if (c < 0x20U) {
            if (used + 6U >= output_size) {
                break;
            }
            std::snprintf(output + used, output_size - used, "\\u%04X", (unsigned)c);
            used += 6U;
            continue;
        }

        output[used++] = static_cast<char>(c);
        output[used] = '\0';
    }
}

static uint16_t calculate_modbus_crc16(const char *data)
{
    uint16_t crc = 0xFFFFU;
    if (data == nullptr) {
        return crc;
    }

    while (*data != '\0') {
        crc ^= static_cast<uint8_t>(*data++);
        for (uint8_t i = 0U; i < 8U; ++i) {
            if ((crc & 0x0001U) != 0U) {
                crc >>= 1U;
                crc ^= 0xA001U;
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static bool continuous_sensor_data_changed(const SensorData &currentData)
{
    if (!gHasLastContinuousData) {
        return true;
    }

    return currentData.presence != gLastContinuousData.presence ||
           currentData.motion != gLastContinuousData.motion ||
           currentData.sleep_state != gLastContinuousData.sleep_state ||
           currentData.heart_rate != gLastContinuousData.heart_rate ||
           currentData.breath_rate != gLastContinuousData.breath_rate ||
           currentData.heartbeat_waveform != gLastContinuousData.heartbeat_waveform ||
           currentData.breathing_waveform != gLastContinuousData.breathing_waveform;
}

static bool influx_daily_snapshot_changed(const RadarReportSnapshot &currentData)
{
    if (!gHasLastInfluxDailySnapshot) {
        return true;
    }

    return currentData.presence != gLastInfluxDailySnapshot.presence ||
           currentData.motion != gLastInfluxDailySnapshot.motion ||
           currentData.distance != gLastInfluxDailySnapshot.distance ||
           currentData.sleep_state != gLastInfluxDailySnapshot.sleep_state ||
           currentData.pos_x != gLastInfluxDailySnapshot.pos_x ||
           currentData.pos_y != gLastInfluxDailySnapshot.pos_y ||
           currentData.pos_z != gLastInfluxDailySnapshot.pos_z ||
           currentData.heartbeat_waveform != gLastInfluxDailySnapshot.heartbeat_waveform ||
           currentData.breathing_waveform != gLastInfluxDailySnapshot.breathing_waveform ||
           currentData.abnormal_state != gLastInfluxDailySnapshot.abnormal_state ||
           currentData.bed_status != gLastInfluxDailySnapshot.bed_status ||
           currentData.struggle_alert != gLastInfluxDailySnapshot.struggle_alert ||
           currentData.no_one_alert != gLastInfluxDailySnapshot.no_one_alert ||
           currentData.heart_rate != gLastInfluxDailySnapshot.heart_rate ||
           currentData.breath_rate != gLastInfluxDailySnapshot.breath_rate;
}

static bool send_continuous_radar_packet_to_ble(void)
{
    const SensorData currentData = radar_manager_get_sensor_data();
    if (!continuous_sensor_data_changed(currentData)) {
        return false;
    }

    char radarDataCore[128] = {};
    if (currentData.presence > 0U) {
        std::snprintf(radarDataCore,
                      sizeof(radarDataCore),
                      "%.1f|%.1f|%d|%d|%u|%u|%u",
                      currentData.heart_rate,
                      currentData.breath_rate,
                      currentData.heartbeat_waveform,
                      currentData.breathing_waveform,
                      (unsigned)currentData.presence,
                      (unsigned)currentData.motion,
                      (unsigned)currentData.sleep_state);
    } else {
        std::snprintf(radarDataCore, sizeof(radarDataCore), "0.0|0.0|0|0|0|0|0");
    }

    char payload[160] = {};
    std::snprintf(payload,
                  sizeof(payload),
                  "%s|%x",
                  radarDataCore,
                  (unsigned)calculate_modbus_crc16(radarDataCore));

    const bool sent = ble_manager_send_text(payload);
    gLastContinuousData = currentData;
    gHasLastContinuousData = true;
    return sent;
}

static bool send_radar_data_snapshot_to_ble(void)
{
    const DeviceIdentity identity = device_identity_get();
    const SensorData currentData = radar_manager_get_sensor_data();
    const uint64_t nowMs = radar_now_ms();

    char payload[1024] = {};
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"type\":\"radarData\",\"success\":true,\"deviceId\":%u,"
                  "\"timestamp\":%" PRIu64 ",\"presence\":%u,\"heartRate\":%.1f,"
                  "\"breathRate\":%.1f,\"motion\":%u,\"heartbeatWaveform\":%d,"
                  "\"breathingWaveform\":%d,\"distance\":%u,\"bodyMovement\":%u,"
                  "\"breathStatus\":%u,\"sleepState\":%u,\"sleepTime\":%u,"
                  "\"sleepScore\":%u,\"avgHeartRate\":%u,\"avgBreathRate\":%u,"
                  "\"turnCount\":%u,\"largeMoveRatio\":%u,\"smallMoveRatio\":%u,"
                  "\"sleepQualityGrade\":%u,\"totalSleepDuration\":%u,"
                  "\"awakeDurationRatio\":%u,\"lightSleepRatio\":%u,\"deepSleepRatio\":%u,"
                  "\"outOfBedDuration\":%u,\"apneaCount\":%u,\"struggleAlert\":%u,"
                  "\"noOneAlert\":%u,\"awakeDuration\":%u,\"lightSleepDuration\":%u,"
                  "\"deepSleepDuration\":%u,\"humanPositionX\":%d,\"humanPositionY\":%d,"
                  "\"humanPositionZ\":%d,\"abnormalState\":%u,\"heartValid\":%u,"
                  "\"breathValid\":%u}",
                  (unsigned)identity.device_id,
                  nowMs,
                  (unsigned)currentData.presence,
                  currentData.heart_rate,
                  currentData.breath_rate,
                  (unsigned)currentData.motion,
                  currentData.heartbeat_waveform,
                  currentData.breathing_waveform,
                  (unsigned)currentData.distance,
                  (unsigned)currentData.body_movement,
                  (unsigned)currentData.breath_status,
                  (unsigned)currentData.sleep_state,
                  (unsigned)currentData.sleep_time,
                  (unsigned)currentData.sleep_score,
                  (unsigned)currentData.avg_heart_rate,
                  (unsigned)currentData.avg_breath_rate,
                  (unsigned)currentData.turn_count,
                  (unsigned)currentData.large_move_ratio,
                  (unsigned)currentData.small_move_ratio,
                  (unsigned)currentData.sleep_grade,
                  (unsigned)currentData.sleep_total_time,
                  (unsigned)currentData.awake_ratio,
                  (unsigned)currentData.light_sleep_ratio,
                  (unsigned)currentData.deep_sleep_ratio,
                  (unsigned)currentData.bed_Out_Time,
                  (unsigned)currentData.apnea_count,
                  (unsigned)currentData.struggle_alert,
                  (unsigned)currentData.no_one_alert,
                  (unsigned)currentData.awake_time,
                  (unsigned)currentData.light_sleep_time,
                  (unsigned)currentData.deep_sleep_time,
                  (int)currentData.pos_x,
                  (int)currentData.pos_y,
                  (int)currentData.pos_z,
                  (unsigned)currentData.abnormal_state,
                  (unsigned)currentData.heart_valid,
                  (unsigned)currentData.breath_valid);
    return ble_manager_send_text(payload);
}

static NetworkStatus to_system_network_status(TaskNetworkStatus status)
{
    switch (status) {
    case NET_CONNECTED:
        return NETWORK_STATUS_CONNECTED;
    case NET_CONNECTING:
        return NETWORK_STATUS_CONNECTING;
    case NET_DISCONNECTED:
        return NETWORK_STATUS_DISCONNECTED;
    case NET_INITIAL:
    default:
        return NETWORK_STATUS_INITIAL;
    }
}

void setNetworkStatus(TaskNetworkStatus status)
{
    currentNetworkStatus = status;
    if (status == NET_CONNECTED) {
        gBreatheValue = APP_LED_BREATHE_MIN;
        gBreatheIncreasing = true;
    }
    system_state_set_network_status(to_system_network_status(status));
}

void WiFiEvent(TaskWiFiEvent event)
{
    switch (event) {
    case TASK_WIFI_EVENT_STA_START:
        setNetworkStatus(NET_INITIAL);
        break;
    case TASK_WIFI_EVENT_STA_CONNECTED:
        setNetworkStatus(NET_CONNECTING);
        break;
    case TASK_WIFI_EVENT_STA_GOT_IP:
        setNetworkStatus(NET_CONNECTED);
        break;
    case TASK_WIFI_EVENT_STA_DISCONNECTED:
    case TASK_WIFI_EVENT_STA_STOP:
        setNetworkStatus(NET_DISCONNECTED);
        break;
    default:
        break;
    }
}

static void configure_boot_button_gpio(void)
{
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << APP_BOOT_BUTTON_PIN);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&config);
}

static void configure_led_gpio(void)
{
    ledc_timer_config_t timerConfig = {};
    timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
    timerConfig.duty_resolution = LEDC_TIMER_8_BIT;
    timerConfig.timer_num = LEDC_TIMER_0;
    timerConfig.freq_hz = 5000;
    timerConfig.clk_cfg = LEDC_AUTO_CLK;
    timerConfig.deconfigure = false;
    ledc_timer_config(&timerConfig);

    ledc_channel_config_t channelConfig = {};
    channelConfig.gpio_num = APP_NETWORK_LED_PIN;
    channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConfig.channel = LEDC_CHANNEL_0;
    channelConfig.intr_type = LEDC_INTR_DISABLE;
    channelConfig.timer_sel = LEDC_TIMER_0;
    channelConfig.duty = 0;
    channelConfig.hpoint = 0;
    ledc_channel_config(&channelConfig);

    gpio_config_t clearLedConfig = {};
    clearLedConfig.pin_bit_mask = (1ULL << APP_CONFIG_CLEAR_PIN);
    clearLedConfig.mode = GPIO_MODE_OUTPUT;
    clearLedConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    clearLedConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    clearLedConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&clearLedConfig);
    gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 0);
}

static void set_network_led_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void clearStoredConfig(void)
{
    ESP_LOGW(TAG, "clearing stored wifi/device configuration");
    (void)wifi_manager_clear_credentials();
    (void)device_identity_reset_device_id_default();
    (void)mqtt_manager_refresh_identity();
    setNetworkStatus(NET_DISCONNECTED);
    sendStatusToBLE();
}

void sendStatusToBLE(void)
{
    const DeviceIdentity identity = device_identity_get();
    const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();
    const BleManagerSnapshot bleSnapshot = ble_manager_get_snapshot();
    char escapedSsid[80] = {};
    char escapedBleName[80] = {};
    json_escape_string(wifiSnapshot.ssid[0] != '\0' ? wifiSnapshot.ssid : "",
                       escapedSsid,
                       sizeof(escapedSsid));
    json_escape_string(bleSnapshot.device_name[0] != '\0' ? bleSnapshot.device_name : "",
                       escapedBleName,
                       sizeof(escapedBleName));

    char payload[512] = {};
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"type\":\"status\",\"deviceId\":%u,\"deviceSn\":%" PRIu64 ","
                  "\"wifiConfigured\":%s,\"wifiConnected\":%s,\"wifiSsid\":\"%s\","
                  "\"wifiIP\":\"%s\",\"ipAddress\":\"%s\","
                  "\"bleReady\":%u,\"bleAdvertising\":%u,\"bleConnected\":%u,"
                  "\"bleNotify\":%u,\"bleName\":\"%s\"}",
                  (unsigned)identity.device_id,
                  identity.device_sn,
                  wifiSnapshot.has_credentials ? "true" : "false",
                  wifiSnapshot.connected ? "true" : "false",
                  escapedSsid,
                  wifiSnapshot.ip,
                  wifiSnapshot.ip,
                  (unsigned)bleSnapshot.initialized,
                  (unsigned)bleSnapshot.advertising,
                  (unsigned)bleSnapshot.connected,
                  (unsigned)bleSnapshot.notify_enabled,
                  escapedBleName);

    const bool sent = ble_manager_send_text(payload);
    ESP_LOGI(TAG, "BLE status payload %s: %s", sent ? "sent" : "prepared", payload);
}

void processBLEConfig(void)
{
    char receivedText[256] = {};
    if (!ble_manager_consume_received_text(receivedText, sizeof(receivedText))) {
        return;
    }

    ESP_LOGI(TAG, "processing BLE payload: %s", receivedText);

    char command[32] = {};
    if (!json_extract_string_value(receivedText, "command", command, sizeof(command))) {
        (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"missing command\"}");
        return;
    }

    if (std::strcmp(command, "queryStatus") == 0) {
        const DeviceIdentity identity = device_identity_get();
        const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();
        char escapedSsid[80] = {};
        json_escape_string(wifiSnapshot.ssid[0] != '\0' ? wifiSnapshot.ssid : "",
                           escapedSsid,
                           sizeof(escapedSsid));
        char payload[512] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"deviceStatus\",\"success\":true,\"deviceId\":%u,"
                      "\"wifiConfigured\":%s,\"wifiConnected\":%s,\"wifiSsid\":\"%s\","
                      "\"wifiIP\":\"%s\",\"ipAddress\":\"%s\"}",
                      (unsigned)identity.device_id,
                      wifiSnapshot.has_credentials ? "true" : "false",
                      wifiSnapshot.connected ? "true" : "false",
                      escapedSsid,
                      wifiSnapshot.ip,
                      wifiSnapshot.ip);
        (void)ble_manager_send_text(payload);
        return;
    }

    if (std::strcmp(command, "setDeviceId") == 0) {
        uint16_t newDeviceId = 0U;
        if (!json_extract_uint16_value(receivedText, "newDeviceId", &newDeviceId)) {
            (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"missing newDeviceId\"}");
            return;
        }

        if (newDeviceId < 1000U || newDeviceId > 1999U) {
            (void)ble_manager_send_text(
                "{\"type\":\"error\",\"message\":\"deviceId out of range 1000-1999\"}");
            return;
        }

        if (!device_identity_set_device_id(newDeviceId)) {
            (void)ble_manager_send_text(
                "{\"type\":\"setDeviceIdResult\",\"success\":false,\"message\":\"persist failed\"}");
            return;
        }

        char payload[160] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"setDeviceIdResult\",\"success\":true,"
                      "\"message\":\"\\u8bbe\\u5907ID\\u8bbe\\u7f6e\\u6210\\u529f\","
                      "\"newDeviceId\":%u}",
                      (unsigned)newDeviceId);
        (void)ble_manager_send_text(payload);
        sendStatusToBLE();
        return;
    }

    if (std::strcmp(command, "setDeviceSn") == 0) {
        uint64_t newDeviceSn = 0ULL;
        if (!json_extract_uint64_value(receivedText, "newDeviceSn", &newDeviceSn)) {
            (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"missing newDeviceSn\"}");
            return;
        }

        if (!device_identity_set_device_sn(newDeviceSn)) {
            (void)ble_manager_send_text(
                "{\"type\":\"setDeviceSnResult\",\"success\":false,\"message\":\"persist failed\"}");
            return;
        }

        char payload[192] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"setDeviceSnResult\",\"success\":true,\"newDeviceSn\":%" PRIu64 "}",
                      newDeviceSn);
        (void)ble_manager_refresh_advertising_data();
        (void)mqtt_manager_refresh_identity();
        (void)ble_manager_send_text(payload);
        sendStatusToBLE();
        return;
    }

    if (std::strcmp(command, "setWiFiConfig") == 0) {
        char ssid[33] = {};
        char password[65] = {};

        if (!json_extract_string_value(receivedText, "ssid", ssid, sizeof(ssid))) {
            (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"missing ssid\"}");
            return;
        }

        if (!json_extract_string_value(receivedText, "password", password, sizeof(password))) {
            password[0] = '\0';
        }

        const bool connected = wifi_manager_configure_and_connect(ssid, password);
        const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();
        char escapedSsid[80] = {};
        char escapedPassword[140] = {};
        json_escape_string(ssid, escapedSsid, sizeof(escapedSsid));
        json_escape_string(password, escapedPassword, sizeof(escapedPassword));
        char payload[384] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"wifiConfigResult\",\"success\":%s,\"ssid\":\"%s\","
                      "\"message\":\"%s\",\"ip\":\"%s\",\"rssi\":%d}",
                      connected ? "true" : "false",
                      escapedSsid,
                      connected
                          ? "\\u0057\\u0069\\u0046\\u0069\\u914d\\u7f6e\\u6210\\u529f"
                          : "\\u0057\\u0069\\u0046\\u0069\\u914d\\u7f6e\\u5931\\u8d25\\uff0c\\u8bf7\\u68c0\\u67e5\\u5bc6\\u7801\\u662f\\u5426\\u6b63\\u786e",
                      wifiSnapshot.ip,
                      (int)wifiSnapshot.rssi);
        (void)ble_manager_send_text(payload);
        if (connected) {
            char connectedPayload[512] = {};
            std::snprintf(connectedPayload,
                          sizeof(connectedPayload),
                          "{\"type\":\"wifiConnected\",\"success\":true,\"ssid\":\"%s\","
                          "\"password\":\"%s\",\"ip\":\"%s\",\"rssi\":%d}",
                          escapedSsid,
                          escapedPassword,
                          wifiSnapshot.ip,
                          (int)wifiSnapshot.rssi);
            (void)ble_manager_send_text(connectedPayload);
        }
        sendStatusToBLE();
        return;
    }

    if (std::strcmp(command, "echo") == 0) {
        char content[128] = {};
        const bool hasContent =
            json_extract_string_value(receivedText, "content", content, sizeof(content));

        char payload[384] = {};
        if (hasContent) {
            char escapedContent[260] = {};
            json_escape_string(content, escapedContent, sizeof(escapedContent));
            std::snprintf(payload,
                          sizeof(payload),
                          "{\"type\":\"echoResponse\",\"originalContent\":\"%s\","
                          "\"receivedSuccessfully\":true}",
                          escapedContent);
        } else {
            std::snprintf(payload,
                          sizeof(payload),
                          "{\"type\":\"echoResponse\",\"receivedSuccessfully\":true,"
                          "\"message\":\"Echo command received\"}");
        }
        (void)ble_manager_send_text(payload);
        return;
    }

    if (std::strcmp(command, "queryRadarData") == 0) {
        (void)send_radar_data_snapshot_to_ble();
        return;
    }

    if (std::strcmp(command, "startContinuousSend") == 0) {
        uint32_t intervalMs = gContinuousSendIntervalMs;
        if (json_extract_uint32_value(receivedText, "interval", &intervalMs)) {
            if (intervalMs < 100U) {
                intervalMs = 100U;
            } else if (intervalMs > 10000U) {
                intervalMs = 10000U;
            }
            gContinuousSendIntervalMs = intervalMs;
        }

        gContinuousSendEnabled = true;
        gHasLastContinuousData = false;
        gLastContinuousCheckMs = 0U;

        char payload[192] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"startContinuousSendResult\",\"success\":true,"
                      "\"message\":\"\\u5df2\\u542f\\u52a8\\u6301\\u7eed\\u53d1\\u9001\\u6a21\\u5f0f\","
                      "\"interval\":%u}",
                      (unsigned)gContinuousSendIntervalMs);
        (void)ble_manager_send_text(payload);
        return;
    }

    if (std::strcmp(command, "stopContinuousSend") == 0) {
        gContinuousSendEnabled = false;
        char payload[160] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"stopContinuousSendResult\",\"success\":true,"
                      "\"message\":\"\\u5df2\\u505c\\u6b62\\u6301\\u7eed\\u53d1\\u9001\\u6a21\\u5f0f\"}");
        (void)ble_manager_send_text(payload);
        return;
    }

    if (std::strcmp(command, "scanWiFi") == 0) {
        WiFiScanRecord scanRecords[16] = {};
        const uint32_t count = wifi_manager_scan_networks(scanRecords, 16U);

        static char payload[3072] = {};
        size_t used = std::snprintf(payload,
                                    sizeof(payload),
                                    "{\"type\":\"scanWiFiResult\",\"success\":true,\"count\":%u,\"networks\":[",
                                    (unsigned)count);

        for (uint32_t i = 0; i < count && used < sizeof(payload); ++i) {
            const bool saved = wifi_manager_has_saved_network(scanRecords[i].ssid);
            char escapedSsid[80] = {};
            char escapedEncryption[40] = {};
            json_escape_string(scanRecords[i].ssid, escapedSsid, sizeof(escapedSsid));
            json_escape_string(scanRecords[i].encryption,
                               escapedEncryption,
                               sizeof(escapedEncryption));
            const int written = std::snprintf(payload + used,
                                              sizeof(payload) - used,
                                              "%s{\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%u,"
                                              "\"encryption\":\"%s\",\"saved\":%s}",
                                              i == 0U ? "" : ",",
                                              escapedSsid,
                                              (int)scanRecords[i].rssi,
                                              (unsigned)scanRecords[i].channel,
                                              escapedEncryption,
                                              saved ? "true" : "false");
            if (written <= 0) {
                break;
            }
            used += static_cast<size_t>(written);
        }

        if (used < sizeof(payload)) {
            std::snprintf(payload + used, sizeof(payload) - used, "]}");
        } else {
            payload[sizeof(payload) - 2U] = ']';
            payload[sizeof(payload) - 1U] = '\0';
        }

        (void)ble_manager_send_text(payload);
        return;
    }

    if (std::strcmp(command, "getSavedNetworks") == 0) {
        WiFiSavedNetwork savedNetworks[10] = {};
        const uint32_t count = wifi_manager_get_saved_networks(savedNetworks, 10U);

        static char payload[1024] = {};
        size_t used = std::snprintf(payload,
                                    sizeof(payload),
                                    "{\"type\":\"savedNetworks\",\"success\":true,\"count\":%u,\"networks\":[",
                                    (unsigned)count);

        for (uint32_t i = 0; i < count && used < sizeof(payload); ++i) {
            char escapedSsid[80] = {};
            json_escape_string(savedNetworks[i].ssid, escapedSsid, sizeof(escapedSsid));
            const int written = std::snprintf(payload + used,
                                              sizeof(payload) - used,
                                              "%s{\"ssid\":\"%s\"}",
                                              i == 0U ? "" : ",",
                                              escapedSsid);
            if (written <= 0) {
                break;
            }
            used += static_cast<size_t>(written);
        }

        if (used < sizeof(payload)) {
            std::snprintf(payload + used, sizeof(payload) - used, "]}");
        } else {
            payload[sizeof(payload) - 2U] = ']';
            payload[sizeof(payload) - 1U] = '\0';
        }

        (void)ble_manager_send_text(payload);
        return;
    }

    (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"unknown command\"}");
}

void tasks_manager_set_continuous_send(bool enabled, unsigned long interval_ms)
{
    uint32_t clampedInterval = static_cast<uint32_t>(interval_ms);
    if (clampedInterval < 100U) {
        clampedInterval = 100U;
    } else if (clampedInterval > 10000U) {
        clampedInterval = 10000U;
    }

    gContinuousSendEnabled = enabled;
    gContinuousSendIntervalMs = clampedInterval;
    gHasLastContinuousData = false;
    gLastContinuousCheckMs = 0U;
}

bool tasks_manager_get_continuous_send_enabled(void)
{
    return gContinuousSendEnabled;
}

unsigned long tasks_manager_get_continuous_send_interval_ms(void)
{
    return static_cast<unsigned long>(gContinuousSendIntervalMs);
}

void bootButtonMonitorTask(void *parameter)
{
    (void)parameter;

    configure_boot_button_gpio();
    gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 0);

    uint64_t buttonPressStartMs = 0U;
    bool buttonPressed = false;

    ESP_LOGI(TAG, "boot button monitor task started");
    task_watchdog_register_current("boot_button_task");

    while (true) {
        task_watchdog_reset_current();
        const int buttonLevel = gpio_get_level((gpio_num_t)APP_BOOT_BUTTON_PIN);

        if (buttonLevel == 0 && !buttonPressed) {
            buttonPressed = true;
            buttonPressStartMs = radar_now_ms();
            gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 1);
            ESP_LOGI(TAG, "boot button pressed, hold %u ms to clear config",
                     (unsigned)APP_CLEAR_CONFIG_DURATION_MS);
        } else if (buttonLevel != 0 && buttonPressed) {
            buttonPressed = false;
            if (!gClearConfigRequested) {
                gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 0);
                ESP_LOGI(TAG, "boot button released, clear cancelled");
            }
        }

        if (buttonPressed &&
            !gClearConfigRequested &&
            (radar_now_ms() - buttonPressStartMs) >= APP_CLEAR_CONFIG_DURATION_MS) {
            gClearConfigRequested = true;
            gForceLedOff = true;
            clearStoredConfig();
            gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 0);
            set_network_led_duty(0);
            ESP_LOGW(TAG, "configuration cleared, restarting");
            radar_sleep_ms(1000);
            esp_restart();
        }

        radar_sleep_ms(50);
    }
}

void ledControlTask(void *parameter)
{
    (void)parameter;

    configure_led_gpio();
    set_network_led_duty(0);
    gLastBlinkTimeMs = radar_now_ms();

    ESP_LOGI(TAG, "network led control task started");
    task_watchdog_register_current("led_control_task");

    while (true) {
        task_watchdog_reset_current();
        const uint64_t nowMs = radar_now_ms();

        if (gForceLedOff) {
            set_network_led_duty(0);
            radar_sleep_ms(10);
            continue;
        }

        switch (currentNetworkStatus) {
        case NET_INITIAL:
        case NET_DISCONNECTED:
            if ((nowMs - gLastBlinkTimeMs) >= APP_LED_SLOW_BLINK_INTERVAL_MS) {
                gLedState = !gLedState;
                set_network_led_duty(gLedState ? 255U : 0U);
                gLastBlinkTimeMs = nowMs;
            }
            break;
        case NET_CONNECTING:
            if ((nowMs - gLastBlinkTimeMs) >= APP_LED_FAST_BLINK_INTERVAL_MS) {
                gLedState = !gLedState;
                set_network_led_duty(gLedState ? 255U : 0U);
                gLastBlinkTimeMs = nowMs;
            }
            break;
        case NET_CONNECTED:
            if ((nowMs - gLastBlinkTimeMs) >= APP_LED_BREATHE_INTERVAL_MS) {
                set_network_led_duty(gBreatheValue);
                if (gBreatheIncreasing) {
                    gBreatheValue += APP_LED_BREATHE_STEP;
                    if (gBreatheValue >= APP_LED_BREATHE_MAX) {
                        gBreatheValue = APP_LED_BREATHE_MAX;
                        gBreatheIncreasing = false;
                    }
                } else if (gBreatheValue > APP_LED_BREATHE_STEP) {
                    gBreatheValue -= APP_LED_BREATHE_STEP;
                } else {
                    gBreatheValue = APP_LED_BREATHE_MIN;
                    gBreatheIncreasing = true;
                }
                gLastBlinkTimeMs = nowMs;
            }
            break;
        }

        radar_sleep_ms(10);
    }
}

void wifiMonitorTask(void *parameter)
{
    (void)parameter;

    ESP_LOGI(TAG, "wifi monitor task started");

    if (!wifi_manager_init()) {
        ESP_LOGE(TAG, "wifi manager init failed inside wifi monitor task");
        return;
    }

    const WiFiManagerSnapshot initialSnapshot = wifi_manager_get_snapshot();
    if (initialSnapshot.has_credentials != 0U) {
        ESP_LOGI(TAG, "stored wifi credentials found, attempting connect");
        (void)wifi_manager_connect();
    } else if (APP_WIFI_DEFAULT_SSID[0] != '\0') {
        ESP_LOGI(TAG, "using default wifi credentials from app config");
        (void)wifi_manager_configure_and_connect(APP_WIFI_DEFAULT_SSID, APP_WIFI_DEFAULT_PASSWORD);
    } else {
        ESP_LOGI(TAG, "no wifi credentials configured yet");
    }

    while (true) {
        const WiFiManagerSnapshot snapshot = wifi_manager_get_snapshot();
        switch (snapshot.state) {
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
        case WIFI_MANAGER_ERROR:
            setNetworkStatus(NET_DISCONNECTED);
            break;
        default:
            setNetworkStatus(NET_INITIAL);
            break;
        }

        ESP_LOGI(TAG,
                 "wifi monitor state=%d connected=%u creds=%u retry=%u ssid=%s",
                 (int)snapshot.state,
                 (unsigned)snapshot.connected,
                 (unsigned)snapshot.has_credentials,
                 (unsigned)snapshot.reconnect_attempts,
                 snapshot.ssid[0] != '\0' ? snapshot.ssid : "-");

        radar_sleep_ms(30000);
    }
}

void bleConfigTask(void *parameter)
{
    (void)parameter;

    ESP_LOGI(TAG, "ble config task started");

    if (!ble_manager_init()) {
        ESP_LOGW(TAG, "ble manager init failed inside ble config task");
        return;
    }

    sendStatusToBLE();
    task_watchdog_register_current("ble_config_task");

    while (true) {
        task_watchdog_reset_current();
        processBLEConfig();

        const BleManagerSnapshot bleSnapshot = ble_manager_get_snapshot();
        if (bleSnapshot.initialized == 0U) {
            ESP_LOGW(TAG, "ble not initialized, retrying");
            (void)ble_manager_init();
        }

        static uint8_t lastConnected = 0U;
        static uint8_t lastNotifyEnabled = 0U;
        if (bleSnapshot.connected != lastConnected ||
            bleSnapshot.notify_enabled != lastNotifyEnabled) {
            if (lastConnected != 0U && bleSnapshot.connected == 0U) {
                gContinuousSendEnabled = false;
            }
            lastConnected = bleSnapshot.connected;
            lastNotifyEnabled = bleSnapshot.notify_enabled;
            sendStatusToBLE();
        }

        if (gContinuousSendEnabled && ble_manager_is_client_connected()) {
            const uint64_t nowMs = radar_now_ms();
            if (gLastContinuousCheckMs == 0U ||
                (nowMs - gLastContinuousCheckMs) >= gContinuousSendIntervalMs) {
                gLastContinuousCheckMs = nowMs;
                (void)send_continuous_radar_packet_to_ble();
            }
        }

        radar_sleep_ms(10);
    }
}

static void radar_command_task(void *parameter)
{
    (void)parameter;

    static const uint8_t radarCommands[][3] = {
        {0x84, 0x81, 0x0F},
        {0x84, 0x8D, 0x0F},
        {0x84, 0x8F, 0x0F},
        {0x84, 0x8E, 0x0F},
        {0x84, 0x91, 0x0F},
        {0x84, 0x83, 0x0F},
        {0x85, 0x02, 0x0F},
        {0x81, 0x02, 0x0F},
    };

    size_t commandIndex = 0;
    task_watchdog_register_current("radar_command_task");

    while (true) {
        task_watchdog_reset_current();
        const uint8_t *command = radarCommands[commandIndex];
        sendRadarCommand(command[0], command[1], command[2]);

        commandIndex = (commandIndex + 1U) % (sizeof(radarCommands) / sizeof(radarCommands[0]));
        radar_sleep_ms(2000);
    }
}

static void radar_status_task(void *parameter)
{
    (void)parameter;
    task_watchdog_register_current("radar_status_task");

    while (true) {
        task_watchdog_reset_current();
        const RadarReportSnapshot snapshot = radar_manager_get_report_snapshot(10000);
        const RadarChangeFlags changeFlags = radar_manager_get_change_flags();
        const SystemStateSnapshot systemState = system_state_get_snapshot();
        const BleManagerSnapshot bleState = ble_manager_get_snapshot();
        const WiFiManagerSnapshot wifiState = wifi_manager_get_snapshot();
        const MqttManagerSnapshot mqttState = mqtt_manager_get_snapshot();

        ESP_LOGI(TAG,
                 "system phase=%d net=%d dev=%u sn=%" PRIu64 " radar=%u tasks=%u ble=%u wifi=%u mqtt=%u err=%u uptime=%" PRIu64 "ms | ble init=%u adv=%u conn=%u notify=%u name=%s | wifi state=%d conn=%u creds=%u retry=%u ssid=%s | mqtt state=%d conn=%u cfg=%u retry=%u | radar uart=%s fresh=%s changed=%s person=%s bed=%s vitals=%s sample_age=%" PRIu64 "ms presence=%u motion=%u dist=%u hr=%.1f rr=%.1f sleep=%u body=%u abnormal=%u",
                 (int)systemState.phase,
                 (int)systemState.network_status,
                 (unsigned)systemState.current_device_id,
                 systemState.device_sn,
                 static_cast<unsigned>(systemState.radar_ready),
                 static_cast<unsigned>(systemState.tasks_ready),
                 static_cast<unsigned>(systemState.ble_ready),
                 static_cast<unsigned>(systemState.wifi_ready),
                 static_cast<unsigned>(systemState.mqtt_ready),
                 static_cast<unsigned>(systemState.error),
                 systemState.uptime_ms,
                 static_cast<unsigned>(bleState.initialized),
                 static_cast<unsigned>(bleState.advertising),
                 static_cast<unsigned>(bleState.connected),
                 static_cast<unsigned>(bleState.notify_enabled),
                 bleState.device_name[0] != '\0' ? bleState.device_name : "-",
                 (int)wifiState.state,
                 static_cast<unsigned>(wifiState.connected),
                 static_cast<unsigned>(wifiState.has_credentials),
                 static_cast<unsigned>(wifiState.reconnect_attempts),
                 wifiState.ssid[0] != '\0' ? wifiState.ssid : "-",
                 (int)mqttState.state,
                 static_cast<unsigned>(mqttState.connected),
                 static_cast<unsigned>(mqttState.configured),
                 static_cast<unsigned>(mqttState.reconnect_attempts),
                 radar_manager_is_initialized() ? "ready" : "down",
                 snapshot.has_fresh_sample ? "yes" : "no",
                 changeFlags.any_changed ? "yes" : "no",
                 snapshot.has_person ? "yes" : "no",
                 snapshot.bed_occupied ? "yes" : "no",
                 snapshot.has_valid_vitals ? "yes" : "no",
                 (uint64_t)snapshot.sample_age_ms,
                 static_cast<unsigned>(snapshot.presence),
                 static_cast<unsigned>(snapshot.motion),
                 static_cast<unsigned>(snapshot.distance),
                 snapshot.heart_rate,
                 snapshot.breath_rate,
                 static_cast<unsigned>(snapshot.sleep_state),
                 static_cast<unsigned>(snapshot.body_movement),
                 static_cast<unsigned>(snapshot.abnormal_state));

        if (changeFlags.any_changed) {
            ESP_LOGI(TAG,
                     "change flags presence=%u motion=%u sleep=%u body=%u distance=%u hr=%u rr=%u abnormal=%u",
                     changeFlags.presence_changed,
                     changeFlags.motion_changed,
                     changeFlags.sleep_state_changed,
                     changeFlags.body_movement_changed,
                     changeFlags.distance_changed,
                     changeFlags.heart_rate_changed,
                     changeFlags.breath_rate_changed,
                     changeFlags.abnormal_state_changed);

            if (mqtt_manager_is_connected()) {
                const bool published = mqtt_manager_publish_radar_snapshot(&snapshot);
                const bool isNoOne = snapshot.presence == 0U ||
                                     (snapshot.heart_rate == 0.0f && snapshot.breath_rate == 0.0f);
                if (published) {
                    if (isNoOne) {
                        gLastMqttHeartbeatMs = radar_now_ms();
                    } else {
                        gLastMqttDailySendMs = radar_now_ms();
                    }
                }
            }

            radar_manager_mark_snapshot_consumed();
        }

        const uint64_t nowMs = radar_now_ms();
        const bool isMqttNoOne = snapshot.presence == 0U ||
                                 (snapshot.heart_rate == 0.0f && snapshot.breath_rate == 0.0f);
        if (mqtt_manager_is_connected() &&
            isMqttNoOne &&
            (gLastMqttHeartbeatMs == 0U ||
             (nowMs - gLastMqttHeartbeatMs) >= kMqttHeartbeatIntervalMs)) {
            task_watchdog_reset_current();
            if (mqtt_manager_publish_online()) {
                gLastMqttHeartbeatMs = radar_now_ms();
            }
            task_watchdog_reset_current();
        }

        if (mqtt_manager_is_connected() &&
            !isMqttNoOne &&
            (gLastMqttDailySendMs == 0U ||
             (nowMs - gLastMqttDailySendMs) >= kMqttDailyDataIntervalMs)) {
            task_watchdog_reset_current();
            if (mqtt_manager_publish_radar_snapshot(&snapshot)) {
                gLastMqttDailySendMs = radar_now_ms();
            }
            task_watchdog_reset_current();
        }

        if (mqtt_manager_is_connected() &&
            !isMqttNoOne &&
            (gLastMqttSleepSendMs == 0U ||
             (nowMs - gLastMqttSleepSendMs) >= kMqttSleepDataIntervalMs)) {
            task_watchdog_reset_current();
            if (mqtt_manager_publish_sleep_snapshot(&snapshot)) {
                gLastMqttSleepSendMs = radar_now_ms();
            }
            task_watchdog_reset_current();
        }

        const bool dailyChanged = influx_daily_snapshot_changed(snapshot);
        const bool dailyForceDue = (gLastInfluxDailySendMs == 0U) ||
                                   ((nowMs - gLastInfluxDailySendMs) >= kInfluxDailyForceSendIntervalMs);
        const bool dailyMinElapsed = (gLastInfluxDailySendMs == 0U) ||
                                     ((nowMs - gLastInfluxDailySendMs) >= kInfluxDailyMinSendIntervalMs);

        if ((dailyChanged || dailyForceDue) && dailyMinElapsed) {
            task_watchdog_reset_current();
            if (influx_manager_send_daily_data(&snapshot)) {
                gLastInfluxDailySnapshot = snapshot;
                gHasLastInfluxDailySnapshot = true;
                gLastInfluxDailySendMs = radar_now_ms();
            }
            task_watchdog_reset_current();
        }

        if (gLastInfluxSleepSendMs == 0U ||
            (nowMs - gLastInfluxSleepSendMs) >= kInfluxSleepSendIntervalMs) {
            const SensorData currentData = radar_manager_get_sensor_data();
            task_watchdog_reset_current();
            (void)influx_manager_send_sleep_data(&currentData);
            gLastInfluxSleepSendMs = radar_now_ms();
            task_watchdog_reset_current();
        }

        radar_sleep_ms(5000);
    }
}

static void radar_analysis_task(void *parameter)
{
    (void)parameter;

    PhysioDataProcessor processor;
    SimpleEmotionAnalyzer emotionAnalyzer;
    SleepAnalyzer sleepAnalyzer;
    task_watchdog_register_current("radar_analysis_task");

    while (true) {
        task_watchdog_reset_current();
        const SensorData currentData = radar_manager_get_sensor_data();
        const bool hasVitalSample = (currentData.heart_valid != 0U) || (currentData.breath_valid != 0U);
        if (!hasVitalSample) {
            radar_sleep_ms(1000);
            continue;
        }

        const float hr = currentData.heart_valid ? currentData.heart_rate : 0.0f;
        const float rr = currentData.breath_valid ? currentData.breath_rate : 0.0f;
        if (hr <= 0.0f && rr <= 0.0f) {
            radar_sleep_ms(1000);
            continue;
        }

        processor.update(hr,
                         rr,
                         currentData.heart_valid ? 80.0f : 0.0f,
                         currentData.breath_valid ? 80.0f : 0.0f);

        const HeartRateData hrData = processor.getHeartRateData();
        const RespirationData rrData = processor.getRespirationData();
        const HRVEstimate hrvData = processor.getHRVEstimate();

        BodyMovementData movementData = {};
        movementData.movement = currentData.body_movement;
        movementData.movementSmoothed = static_cast<float>(currentData.body_movement);
        movementData.movementMean = static_cast<float>(currentData.body_movement);
        movementData.movementStd = 0.0f;
        movementData.activityLevel = static_cast<float>(currentData.body_movement) / 100.0f;
        movementData.isValid = true;
        movementData.timestamp = radar_now_ms();

        if (hrData.isValid || rrData.isValid) {
            emotionAnalyzer.calibrateBaseline(hrData, rrData, movementData);
        }

        const EmotionResult emotionResult =
            emotionAnalyzer.analyze(hrData, rrData, hrvData, movementData);
        sleepAnalyzer.update(hrData, rrData, hrvData, movementData);

        ESP_LOGI(TAG,
                 "analysis emotion=%s sleep=%s hr=%.1f rr=%.1f stress=%.1f relax=%.1f",
                 EmotionOutput::toBrief(emotionResult).c_str(),
                 sleepAnalyzer.formatState().c_str(),
                 hrData.bpmSmoothed,
                 rrData.rateSmoothed,
                 emotionResult.stressLevel,
                 emotionResult.relaxationLevel);

        radar_sleep_ms(1000);
    }
}

static void device_console_task(void *parameter)
{
    (void)parameter;

    ESP_LOGI(TAG, "device console ready, type 'help' or 'show_device'");

    char line[128];
    while (true) {
        if (std::fgets(line, sizeof(line), stdin) != nullptr) {
            (void)device_command_handle_line(line);
            continue;
        }

        radar_sleep_ms(200);
    }
}

bool tasks_manager_init(void)
{
    if (tasksInitialized) {
        return true;
    }

    ESP_LOGI(TAG, "initializing tasks manager");
    setNetworkStatus(NET_INITIAL);

    if (radarCommandTaskHandle == nullptr) {
        xTaskCreate(radar_command_task,
                    "radar_command_task",
                    4096,
                    nullptr,
                    4,
                    &radarCommandTaskHandle);
    }

    if (radarStatusTaskHandle == nullptr) {
        xTaskCreate(radar_status_task,
                    "radar_status_task",
                    4096,
                    nullptr,
                    2,
                    &radarStatusTaskHandle);
    }

    if (radarAnalysisTaskHandle == nullptr) {
        xTaskCreate(radar_analysis_task,
                    "radar_analysis_task",
                    8192,
                    nullptr,
                    3,
                    &radarAnalysisTaskHandle);
    }

    if (deviceConsoleTaskHandle == nullptr) {
        xTaskCreate(device_console_task,
                    "device_console_task",
                    4096,
                    nullptr,
                    1,
                    &deviceConsoleTaskHandle);
    }

    if (bootButtonTaskHandle == nullptr) {
        xTaskCreate(bootButtonMonitorTask,
                    "boot_button_task",
                    4096,
                    nullptr,
                    2,
                    &bootButtonTaskHandle);
    }

    if (ledControlTaskHandle == nullptr) {
        xTaskCreate(ledControlTask,
                    "led_control_task",
                    3072,
                    nullptr,
                    1,
                    &ledControlTaskHandle);
    }

    if (wifiMonitorTaskHandle == nullptr) {
        xTaskCreate(wifiMonitorTask,
                    "wifi_monitor_task",
                    4096,
                    nullptr,
                    2,
                    &wifiMonitorTaskHandle);
    }

    if (bleConfigTaskHandle == nullptr) {
        xTaskCreate(bleConfigTask,
                    "ble_config_task",
                    4096,
                    nullptr,
                    1,
                    &bleConfigTaskHandle);
    }

    tasksInitialized = true;
    system_state_set_tasks_ready(1U);
    system_state_set_phase(SYSTEM_PHASE_TASKS_READY);
    ESP_LOGI(TAG, "tasks manager started");
    return true;
}

bool initAllTasks(void)
{
    ESP_LOGI(TAG, "initAllTasks called");
    return tasks_manager_init();
}

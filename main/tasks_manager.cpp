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
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
    while (*valuePos != '\0' && *valuePos != '"' && outIndex < (buffer_size - 1U)) {
        buffer[outIndex++] = *valuePos++;
    }
    buffer[outIndex] = '\0';
    return *valuePos == '"';
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
    (void)device_identity_reset_defaults();
    setNetworkStatus(NET_DISCONNECTED);
    sendStatusToBLE();
}

void sendStatusToBLE(void)
{
    const DeviceIdentity identity = device_identity_get();
    const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();
    const BleManagerSnapshot bleSnapshot = ble_manager_get_snapshot();

    char payload[256] = {};
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"type\":\"status\",\"deviceId\":%u,\"deviceSn\":%" PRIu64 ","
                  "\"wifiConfigured\":%s,\"wifiConnected\":%s,\"wifiSsid\":\"%s\","
                  "\"bleReady\":%u,\"bleAdvertising\":%u,\"bleConnected\":%u,"
                  "\"bleNotify\":%u,\"bleName\":\"%s\"}",
                  (unsigned)identity.device_id,
                  identity.device_sn,
                  wifiSnapshot.has_credentials ? "true" : "false",
                  wifiSnapshot.connected ? "true" : "false",
                  wifiSnapshot.ssid[0] != '\0' ? wifiSnapshot.ssid : "",
                  (unsigned)bleSnapshot.initialized,
                  (unsigned)bleSnapshot.advertising,
                  (unsigned)bleSnapshot.connected,
                  (unsigned)bleSnapshot.notify_enabled,
                  bleSnapshot.device_name[0] != '\0' ? bleSnapshot.device_name : "");

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
        char payload[256] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"deviceStatus\",\"success\":true,\"deviceId\":%u,"
                      "\"wifiConfigured\":%s,\"wifiConnected\":%s,\"wifiSsid\":\"%s\"}",
                      (unsigned)identity.device_id,
                      wifiSnapshot.has_credentials ? "true" : "false",
                      wifiSnapshot.connected ? "true" : "false",
                      wifiSnapshot.ssid[0] != '\0' ? wifiSnapshot.ssid : "");
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
                      "{\"type\":\"setDeviceIdResult\",\"success\":true,\"newDeviceId\":%u}",
                      (unsigned)newDeviceId);
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

        const bool configured = wifi_manager_set_credentials(ssid, password);
        if (!configured) {
            (void)ble_manager_send_text(
                "{\"type\":\"wifiConfigResult\",\"success\":false,\"message\":\"store failed\"}");
            return;
        }

        const bool connected = wifi_manager_connect();
        char payload[256] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"wifiConfigResult\",\"success\":%s,\"ssid\":\"%s\","
                      "\"message\":\"%s\"}",
                      connected ? "true" : "false",
                      ssid,
                      connected ? "wifi connected" : "wifi connect failed");
        (void)ble_manager_send_text(payload);
        sendStatusToBLE();
        return;
    }

    if (std::strcmp(command, "echo") == 0) {
        char content[128] = {};
        const bool hasContent =
            json_extract_string_value(receivedText, "content", content, sizeof(content));

        char payload[256] = {};
        if (hasContent) {
            std::snprintf(payload,
                          sizeof(payload),
                          "{\"type\":\"echoResponse\",\"originalContent\":\"%s\","
                          "\"receivedSuccessfully\":true}",
                          content);
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
        const DeviceIdentity identity = device_identity_get();
        const SensorData currentData = radar_manager_get_sensor_data();
        const uint64_t nowMs = radar_now_ms();

        char payload[512] = {};
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"type\":\"radarData\",\"success\":true,\"deviceId\":%u,"
                      "\"timestamp\":%" PRIu64 ",\"presence\":%u,\"heartRate\":%.1f,"
                      "\"breathRate\":%.1f,\"motion\":%u,\"heartbeatWaveform\":%d,"
                      "\"breathingWaveform\":%d,\"distance\":%u,\"bodyMovement\":%u,"
                      "\"breathStatus\":%u,\"sleepState\":%u,\"sleepTime\":%u,"
                      "\"sleepScore\":%u,\"avgHeartRate\":%u,\"avgBreathRate\":%u,"
                      "\"abnormalState\":%u,\"heartValid\":%u,\"breathValid\":%u}",
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
                      (unsigned)currentData.abnormal_state,
                      (unsigned)currentData.heart_valid,
                      (unsigned)currentData.breath_valid);
        (void)ble_manager_send_text(payload);
        return;
    }

    (void)ble_manager_send_text("{\"type\":\"error\",\"message\":\"unknown command\"}");
}

void bootButtonMonitorTask(void *parameter)
{
    (void)parameter;

    configure_boot_button_gpio();
    gpio_set_level((gpio_num_t)APP_CONFIG_CLEAR_PIN, 0);

    uint64_t buttonPressStartMs = 0U;
    bool buttonPressed = false;

    ESP_LOGI(TAG, "boot button monitor task started");

    while (true) {
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

    while (true) {
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

    while (true) {
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
            lastConnected = bleSnapshot.connected;
            lastNotifyEnabled = bleSnapshot.notify_enabled;
            sendStatusToBLE();
        }

        radar_sleep_ms(1000);
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

    while (true) {
        const uint8_t *command = radarCommands[commandIndex];
        sendRadarCommand(command[0], command[1], command[2]);

        commandIndex = (commandIndex + 1U) % (sizeof(radarCommands) / sizeof(radarCommands[0]));
        radar_sleep_ms(2000);
    }
}

static void radar_status_task(void *parameter)
{
    (void)parameter;

    while (true) {
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
                (void)mqtt_manager_publish_radar_snapshot(&snapshot);
            }

            radar_manager_mark_snapshot_consumed();
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

    while (true) {
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

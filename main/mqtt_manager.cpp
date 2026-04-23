#include "mqtt_manager.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "device_identity.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mbedtls/md5.h"
#include "mqtt_client.h"
#include "radar_platform.h"
#include "system_state.h"
#include "tasks_manager.h"
#include "wifi_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mqtt_manager";
static MqttManagerSnapshot gMqttState = {};
static esp_mqtt_client_handle_t gMqttClient = nullptr;
static TaskHandle_t gMqttConnectTaskHandle = nullptr;
static uint32_t gMqttMessageId = 1U;
static char gMqttDeviceName[32] = {};
static char gMqttUsername[32] = {};
static char gMqttPassword[33] = {};
static char gMqttSubscribeTopic[128] = {};
static char gMqttPropertyPostTopic[128] = {};

static constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;
static constexpr const char *MQTT_PRODUCT_KEY = "dEkr5BkkXTFZFBdR";
static constexpr const char *MQTT_DEVICE_MODEL = "radar_1.0";
static constexpr const char *MQTT_PRODUCT_SECRET = "2e7957febfcb48b08a1c69b8deb56738";

static void mqtt_manager_make_device_name(char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return;
    }

    const DeviceIdentity identity = device_identity_get();
    if (identity.device_sn != 0ULL) {
        std::snprintf(buffer,
                      buffer_size,
                      "%llu",
                      static_cast<unsigned long long>(identity.device_sn));
        return;
    }

    char mac[24] = {};
    if (!radar_manager_get_device_mac(mac, sizeof(mac))) {
        std::strncpy(buffer, "unknown", buffer_size - 1U);
        buffer[buffer_size - 1U] = '\0';
        return;
    }

    size_t out = 0U;
    for (size_t i = 0U; mac[i] != '\0' && out < buffer_size - 1U; ++i) {
        if (mac[i] != ':') {
            buffer[out++] = mac[i];
        }
    }
    buffer[out] = '\0';
}

static void mqtt_manager_make_password(const char *client_id, char *buffer, size_t buffer_size)
{
    if (client_id == nullptr || buffer == nullptr || buffer_size < 33U) {
        return;
    }

    char raw[160] = {};
    std::snprintf(raw, sizeof(raw), "%s%s", MQTT_PRODUCT_SECRET, client_id);

    unsigned char digest[16] = {};
    mbedtls_md5_context ctx;
    mbedtls_md5_init(&ctx);
    (void)mbedtls_md5_starts(&ctx);
    (void)mbedtls_md5_update(&ctx,
                             reinterpret_cast<const unsigned char *>(raw),
                             std::strlen(raw));
    (void)mbedtls_md5_finish(&ctx, digest);
    mbedtls_md5_free(&ctx);

    for (size_t i = 0U; i < sizeof(digest); ++i) {
        std::snprintf(buffer + (i * 2U), buffer_size - (i * 2U), "%02x", digest[i]);
    }
    buffer[32] = '\0';
}

static void mqtt_manager_refresh_enjoy_iot_identity(void)
{
    mqtt_manager_make_device_name(gMqttDeviceName, sizeof(gMqttDeviceName));
    std::strncpy(gMqttUsername, gMqttDeviceName, sizeof(gMqttUsername) - 1U);
    gMqttUsername[sizeof(gMqttUsername) - 1U] = '\0';

    std::snprintf(gMqttState.client_id,
                  sizeof(gMqttState.client_id),
                  "%s_%s_%s",
                  MQTT_PRODUCT_KEY,
                  gMqttDeviceName,
                  MQTT_DEVICE_MODEL);

    mqtt_manager_make_password(gMqttState.client_id, gMqttPassword, sizeof(gMqttPassword));

    std::snprintf(gMqttSubscribeTopic,
                  sizeof(gMqttSubscribeTopic),
                  "/sys/%s/%s/c/#",
                  MQTT_PRODUCT_KEY,
                  gMqttDeviceName);
    std::snprintf(gMqttPropertyPostTopic,
                  sizeof(gMqttPropertyPostTopic),
                  "/sys/%s/%s/s/event/property/post",
                  MQTT_PRODUCT_KEY,
                  gMqttDeviceName);

    std::strncpy(gMqttState.test_topic,
                 gMqttPropertyPostTopic,
                 sizeof(gMqttState.test_topic) - 1U);
    gMqttState.test_topic[sizeof(gMqttState.test_topic) - 1U] = '\0';
}

static const char *json_find_value_start(const char *json, const char *key)
{
    if (json == nullptr || key == nullptr) {
        return nullptr;
    }

    char pattern[48] = {};
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

static bool json_extract_string(const char *json, const char *key, char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return false;
    }

    const char *valuePos = json_find_value_start(json, key);
    if (valuePos == nullptr || *valuePos != '"') {
        return false;
    }

    ++valuePos;
    size_t out = 0U;
    while (*valuePos != '\0' && out < buffer_size - 1U) {
        if (*valuePos == '"') {
            buffer[out] = '\0';
            return true;
        }

        if (*valuePos != '\\') {
            buffer[out++] = *valuePos++;
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
            buffer[out++] = *valuePos++;
            break;
        case 'b':
            buffer[out++] = '\b';
            ++valuePos;
            break;
        case 'f':
            buffer[out++] = '\f';
            ++valuePos;
            break;
        case 'n':
            buffer[out++] = '\n';
            ++valuePos;
            break;
        case 'r':
            buffer[out++] = '\r';
            ++valuePos;
            break;
        case 't':
            buffer[out++] = '\t';
            ++valuePos;
            break;
        case 'u': {
            const int h0 = json_hex_nibble(valuePos[1]);
            const int h1 = json_hex_nibble(valuePos[2]);
            const int h2 = json_hex_nibble(valuePos[3]);
            const int h3 = json_hex_nibble(valuePos[4]);
            if (h0 < 0 || h1 < 0 || h2 < 0 || h3 < 0) {
                buffer[out] = '\0';
                return false;
            }
            const unsigned codepoint = (unsigned)((h0 << 12) | (h1 << 8) | (h2 << 4) | h3);
            if (codepoint <= 0x7FU) {
                buffer[out++] = static_cast<char>(codepoint);
            } else if (codepoint <= 0x7FFU && out + 1U < buffer_size - 1U) {
                buffer[out++] = static_cast<char>(0xC0U | (codepoint >> 6));
                buffer[out++] = static_cast<char>(0x80U | (codepoint & 0x3FU));
            } else if (out + 2U < buffer_size - 1U) {
                buffer[out++] = static_cast<char>(0xE0U | (codepoint >> 12));
                buffer[out++] = static_cast<char>(0x80U | ((codepoint >> 6) & 0x3FU));
                buffer[out++] = static_cast<char>(0x80U | (codepoint & 0x3FU));
            }
            valuePos += 5;
            break;
        }
        default:
            buffer[out++] = *valuePos++;
            break;
        }
    }
    buffer[out] = '\0';
    return false;
}

static bool json_extract_bool(const char *json, const char *key, bool *value)
{
    if (value == nullptr) {
        return false;
    }

    const char *valuePos = json_find_value_start(json, key);
    if (valuePos == nullptr) {
        return false;
    }

    if (std::strncmp(valuePos, "true", 4) == 0) {
        *value = true;
        return true;
    }
    if (std::strncmp(valuePos, "false", 5) == 0) {
        *value = false;
        return true;
    }
    return false;
}

static bool json_extract_ulong(const char *json, const char *key, unsigned long *value)
{
    if (value == nullptr) {
        return false;
    }

    const char *valuePos = json_find_value_start(json, key);
    if (valuePos == nullptr) {
        return false;
    }

    char *endPtr = nullptr;
    const unsigned long parsed = std::strtoul(valuePos, &endPtr, 10);
    if (endPtr == valuePos) {
        return false;
    }
    *value = parsed;
    return true;
}

static void mqtt_manager_build_reply_topic(const char *request_topic,
                                           char *buffer,
                                           size_t buffer_size)
{
    if (request_topic == nullptr || buffer == nullptr || buffer_size == 0U) {
        return;
    }

    std::strncpy(buffer, request_topic, buffer_size - 1U);
    buffer[buffer_size - 1U] = '\0';

    char *commandPos = std::strstr(buffer, "/c/");
    if (commandPos != nullptr) {
        commandPos[1] = 's';
    }

    const size_t used = std::strlen(buffer);
    if (used < buffer_size - 1U) {
        std::snprintf(buffer + used, buffer_size - used, "_reply");
    }
}

static bool mqtt_manager_publish_reply(const char *request_topic,
                                       const char *request_id,
                                       const char *request_method,
                                       int code,
                                       const char *params_json)
{
    char replyTopic[160] = {};
    mqtt_manager_build_reply_topic(request_topic, replyTopic, sizeof(replyTopic));
    if (replyTopic[0] == '\0') {
        return false;
    }

    char payload[512] = {};
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"id\":\"%s\",\"method\":\"%s_reply\",\"code\":%d,\"params\":%s}",
                  request_id != nullptr ? request_id : "",
                  request_method != nullptr ? request_method : "",
                  code,
                  params_json != nullptr ? params_json : "{}");

    return mqtt_manager_publish_text(replyTopic, payload, 1, 0);
}

static void mqtt_manager_handle_downlink(const char *topic, const char *payload)
{
    char method[64] = {};
    char id[32] = {};
    (void)json_extract_string(payload, "method", method, sizeof(method));
    (void)json_extract_string(payload, "id", id, sizeof(id));

    if (std::strcmp(method, "thing.service.property.set") == 0) {
        bool enabled = tasks_manager_get_continuous_send_enabled();
        unsigned long interval = tasks_manager_get_continuous_send_interval_ms();
        (void)json_extract_bool(payload, "continuousSendEnabled", &enabled);
        (void)json_extract_ulong(payload, "continuousSendInterval", &interval);
        tasks_manager_set_continuous_send(enabled, interval);
        (void)mqtt_manager_publish_reply(topic, id, method, 0, "{\"success\":true}");
        return;
    }

    if (std::strcmp(method, "thing.service.property.get") == 0) {
        const SensorData data = radar_manager_get_sensor_data();
        char params[384] = {};
        std::snprintf(params,
                      sizeof(params),
                      "{\"heartRate\":%.1f,\"breathingRate\":%.1f,"
                      "\"personDetected\":%u,\"humanActivity\":%u,\"humanDistance\":%u,"
                      "\"sleepState\":%u,\"continuousSendEnabled\":%s,"
                      "\"continuousSendInterval\":%lu}",
                      data.heart_rate,
                      data.breath_rate,
                      (unsigned)data.presence,
                      (unsigned)data.motion,
                      (unsigned)data.distance,
                      (unsigned)data.sleep_state,
                      tasks_manager_get_continuous_send_enabled() ? "true" : "false",
                      tasks_manager_get_continuous_send_interval_ms());
        (void)mqtt_manager_publish_reply(topic, id, method, 0, params);
        return;
    }

    if (std::strncmp(method, "thing.service.", 14) == 0) {
        (void)mqtt_manager_publish_reply(topic, id, method, 0, "{\"success\":true}");
        return;
    }

    ESP_LOGW(TAG, "unsupported mqtt method=%s", method);
}

static void mqtt_manager_set_state_internal(MqttManagerState state)
{
    gMqttState.state = state;
    gMqttState.last_state_change_ms = radar_now_ms();
}

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        gMqttState.connected = 1U;
        mqtt_manager_set_state_internal(MQTT_MANAGER_CONNECTED);
        ESP_LOGI(TAG, "mqtt connected");
        if (gMqttSubscribeTopic[0] != '\0') {
            esp_mqtt_client_subscribe(gMqttClient, gMqttSubscribeTopic, 1);
            ESP_LOGI(TAG, "mqtt subscribed topic=%s", gMqttSubscribeTopic);
        }
        (void)mqtt_manager_publish_online();
        break;
    case MQTT_EVENT_DISCONNECTED:
        gMqttState.connected = 0U;
        mqtt_manager_set_state_internal(MQTT_MANAGER_DISCONNECTED);
        ESP_LOGW(TAG, "mqtt disconnected");
        break;
    case MQTT_EVENT_ERROR:
        gMqttState.connected = 0U;
        mqtt_manager_set_state_internal(MQTT_MANAGER_ERROR);
        ESP_LOGE(TAG, "mqtt error");
        break;
    case MQTT_EVENT_DATA: {
        char topic[160] = {};
        char payload[1024] = {};
        const int topicLen = event->topic_len < (int)(sizeof(topic) - 1U)
                                 ? event->topic_len
                                 : (int)(sizeof(topic) - 1U);
        const int payloadLen = event->data_len < (int)(sizeof(payload) - 1U)
                                   ? event->data_len
                                   : (int)(sizeof(payload) - 1U);
        std::memcpy(topic, event->topic, static_cast<size_t>(topicLen));
        topic[topicLen] = '\0';
        std::memcpy(payload, event->data, static_cast<size_t>(payloadLen));
        payload[payloadLen] = '\0';
        ESP_LOGI(TAG, "mqtt downlink topic=%s payload=%s", topic, payload);
        mqtt_manager_handle_downlink(topic, payload);
        break;
    }
    default:
        break;
    }
}

static void mqtt_connect_task(void *parameter)
{
    (void)parameter;

    while (true) {
        const bool shouldConnect =
            gMqttState.initialized != 0U &&
            gMqttState.configured != 0U &&
            gMqttState.connected == 0U &&
            wifi_manager_is_connected() &&
            gMqttState.state != MQTT_MANAGER_CONNECTING;

        if (shouldConnect) {
            if (gMqttClient == nullptr) {
                mqtt_manager_refresh_enjoy_iot_identity();
                esp_mqtt_client_config_t mqttCfg = {};
                mqttCfg.broker.address.uri = gMqttState.uri;
                mqttCfg.credentials.client_id = gMqttState.client_id;
                mqttCfg.credentials.username = gMqttUsername;
                mqttCfg.credentials.authentication.password = gMqttPassword;
                mqttCfg.buffer.size = 1024;
                gMqttClient = esp_mqtt_client_init(&mqttCfg);
                if (gMqttClient == nullptr) {
                    mqtt_manager_set_state_internal(MQTT_MANAGER_ERROR);
                    ESP_LOGE(TAG, "failed to init mqtt client");
                    vTaskDelay(pdMS_TO_TICKS(MQTT_RECONNECT_INTERVAL_MS));
                    continue;
                }
                esp_mqtt_client_register_event(gMqttClient,
                                               MQTT_EVENT_ANY,
                                               mqtt_event_handler,
                                               nullptr);
            }

            gMqttState.reconnect_attempts += 1U;
            mqtt_manager_set_state_internal(MQTT_MANAGER_CONNECTING);
            ESP_LOGI(TAG, "mqtt connect attempt uri=%s", gMqttState.uri);

            esp_err_t err = esp_mqtt_client_start(gMqttClient);
            if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
                mqtt_manager_set_state_internal(MQTT_MANAGER_ERROR);
                ESP_LOGE(TAG, "esp_mqtt_client_start failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MQTT_RECONNECT_INTERVAL_MS));
    }
}

bool mqtt_manager_init(void)
{
    if (gMqttState.initialized) {
        return true;
    }

    gMqttState.initialized = 1U;
    gMqttState.connected = 0U;
    gMqttState.configured = 0U;
    gMqttState.reconnect_attempts = 0U;
    gMqttState.uri[0] = '\0';
    gMqttState.client_id[0] = '\0';
    gMqttState.test_topic[0] = '\0';
    mqtt_manager_set_state_internal(MQTT_MANAGER_INITIALIZED);
    system_state_set_mqtt_ready(1U);

    if (gMqttConnectTaskHandle == nullptr) {
        xTaskCreate(mqtt_connect_task,
                    "mqtt_connect_task",
                    4096,
                    nullptr,
                    2,
                    &gMqttConnectTaskHandle);
    }

    ESP_LOGI(TAG, "minimal mqtt manager initialized");
    return true;
}

bool mqtt_manager_configure(const char *uri, const char *client_id)
{
    if (uri == nullptr || uri[0] == '\0') {
        ESP_LOGW(TAG, "mqtt config rejected: empty uri");
        return false;
    }

    std::strncpy(gMqttState.uri, uri, sizeof(gMqttState.uri) - 1U);
    gMqttState.uri[sizeof(gMqttState.uri) - 1U] = '\0';
    (void)client_id;
    mqtt_manager_refresh_enjoy_iot_identity();
    gMqttState.configured = 1U;

    if (gMqttClient != nullptr) {
        esp_mqtt_client_stop(gMqttClient);
        esp_mqtt_client_destroy(gMqttClient);
        gMqttClient = nullptr;
    }

    mqtt_manager_set_state_internal(MQTT_MANAGER_INITIALIZED);
    ESP_LOGI(TAG,
             "mqtt configured uri=%s client_id=%s username=%s sub=%s post=%s",
             gMqttState.uri,
             gMqttState.client_id,
             gMqttUsername,
             gMqttSubscribeTopic,
             gMqttPropertyPostTopic);
    return true;
}

bool mqtt_manager_is_initialized(void)
{
    return gMqttState.initialized != 0U;
}

bool mqtt_manager_is_connected(void)
{
    return gMqttState.connected != 0U;
}

MqttManagerSnapshot mqtt_manager_get_snapshot(void)
{
    return gMqttState;
}

bool mqtt_manager_refresh_identity(void)
{
    if (gMqttState.initialized == 0U) {
        return false;
    }

    if (gMqttClient != nullptr) {
        (void)esp_mqtt_client_stop(gMqttClient);
        esp_mqtt_client_destroy(gMqttClient);
        gMqttClient = nullptr;
    }

    gMqttState.connected = 0U;
    mqtt_manager_refresh_enjoy_iot_identity();
    mqtt_manager_set_state_internal(gMqttState.configured != 0U ? MQTT_MANAGER_INITIALIZED
                                                                 : MQTT_MANAGER_DISCONNECTED);
    ESP_LOGI(TAG,
             "mqtt identity refreshed client_id=%s username=%s sub=%s post=%s",
             gMqttState.client_id,
             gMqttUsername,
             gMqttSubscribeTopic,
             gMqttPropertyPostTopic);
    return true;
}

bool mqtt_manager_publish_text(const char *topic, const char *payload, int qos, int retain)
{
    if (gMqttClient == nullptr || !mqtt_manager_is_connected() || topic == nullptr || payload == nullptr) {
        return false;
    }

    const int msgId = esp_mqtt_client_publish(gMqttClient, topic, payload, 0, qos, retain);
    return msgId >= 0;
}

bool mqtt_manager_publish_online(void)
{
    if (gMqttPropertyPostTopic[0] == '\0') {
        return false;
    }

    const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();

    const SensorData data = radar_manager_get_sensor_data();

    char payload[384];
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"id\":\"%lu\",\"method\":\"thing.event.property.post\","
                  "\"params\":{\"deviceId\":\"%s\",\"reportType\":\"heartbeat\","
                  "\"personDetected\":0,\"heartbeat\":1,\"timestamp\":%llu,"
                  "\"sleepState\":%u,\"noOneAlert\":%u,\"wifiIP\":\"%s\"}}",
                  static_cast<unsigned long>(gMqttMessageId++),
                  gMqttDeviceName,
                  (unsigned long long)radar_now_ms(),
                  static_cast<unsigned>(data.sleep_state),
                  static_cast<unsigned>(data.no_one_alert),
                  wifiSnapshot.ip);

    const bool ok = mqtt_manager_publish_text(gMqttPropertyPostTopic, payload, 1, 0);
    ESP_LOGI(TAG, "mqtt heartbeat publish topic=%s ok=%s", gMqttPropertyPostTopic, ok ? "yes" : "no");
    return ok;
}

bool mqtt_manager_publish_radar_snapshot(const RadarReportSnapshot *snapshot)
{
    if (snapshot == nullptr) {
        return false;
    }

    if (gMqttPropertyPostTopic[0] == '\0') {
        return false;
    }

    const bool isNoOne = snapshot->presence == 0U ||
                         (snapshot->heart_rate == 0.0f && snapshot->breath_rate == 0.0f);
    const WiFiManagerSnapshot wifiSnapshot = wifi_manager_get_snapshot();

    char payload[1024] = {};
    if (isNoOne) {
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"id\":\"%lu\",\"method\":\"thing.event.property.post\","
                      "\"params\":{\"deviceId\":\"%s\",\"reportType\":\"heartbeat\","
                      "\"personDetected\":0,\"heartbeat\":1,\"timestamp\":%llu,"
                      "\"sleepState\":%u,\"noOneAlert\":%u,\"wifiIP\":\"%s\"}}",
                      static_cast<unsigned long>(gMqttMessageId++),
                      gMqttDeviceName,
                      (unsigned long long)radar_now_ms(),
                      static_cast<unsigned>(snapshot->sleep_state),
                      static_cast<unsigned>(snapshot->no_one_alert),
                      wifiSnapshot.ip);
    } else {
        std::snprintf(payload,
                      sizeof(payload),
                      "{\"id\":\"%lu\",\"method\":\"thing.event.property.post\","
                      "\"params\":{\"deviceId\":\"%s\",\"reportType\":\"daily\","
                      "\"heartRate\":%.1f,\"breathingRate\":%.1f,"
                      "\"personDetected\":%u,\"humanActivity\":%u,\"humanDistance\":%u,"
                      "\"sleepState\":%u,\"bodyMovement\":%u,\"abnormalState\":%u,"
                      "\"humanPositionX\":%d,\"humanPositionY\":%d,\"humanPositionZ\":%d,"
                      "\"heartbeatWaveform\":%d,\"breathingWaveform\":%d,"
                      "\"bedStatus\":%u,\"struggleAlert\":%u,\"noOneAlert\":%u,"
                      "\"wifiIP\":\"%s\"}}",
                      static_cast<unsigned long>(gMqttMessageId++),
                      gMqttDeviceName,
                      snapshot->heart_rate,
                      snapshot->breath_rate,
                      static_cast<unsigned>(snapshot->presence),
                      static_cast<unsigned>(snapshot->motion),
                      static_cast<unsigned>(snapshot->distance),
                      static_cast<unsigned>(snapshot->sleep_state),
                      static_cast<unsigned>(snapshot->body_movement),
                      static_cast<unsigned>(snapshot->abnormal_state),
                      (int)snapshot->pos_x,
                      (int)snapshot->pos_y,
                      (int)snapshot->pos_z,
                      snapshot->heartbeat_waveform,
                      snapshot->breathing_waveform,
                      static_cast<unsigned>(snapshot->bed_status),
                      static_cast<unsigned>(snapshot->struggle_alert),
                      static_cast<unsigned>(snapshot->no_one_alert),
                      wifiSnapshot.ip);
    }

    const bool ok = mqtt_manager_publish_text(gMqttPropertyPostTopic, payload, 1, 0);
    ESP_LOGI(TAG, "mqtt property publish topic=%s ok=%s", gMqttPropertyPostTopic, ok ? "yes" : "no");
    return ok;
}

bool mqtt_manager_publish_sleep_snapshot(const RadarReportSnapshot *snapshot)
{
    if (snapshot == nullptr) {
        return false;
    }

    if (gMqttPropertyPostTopic[0] == '\0') {
        return false;
    }

    if (snapshot->sleep_state != 0U && snapshot->sleep_state != 1U) {
        ESP_LOGD(TAG, "skip mqtt sleep publish, sleep_state=%u", (unsigned)snapshot->sleep_state);
        return false;
    }

    char sleepPayload[1024] = {};
    std::snprintf(sleepPayload,
                  sizeof(sleepPayload),
                  "{\"id\":\"%lu\",\"method\":\"thing.event.property.post\","
                  "\"params\":{\"deviceId\":\"%s\",\"reportType\":\"sleep\","
                  "\"sleepQualityScore\":%u,\"sleepQualityGrade\":%u,"
                  "\"totalSleepDuration\":%u,\"awakeDurationRatio\":%u,"
                  "\"lightSleepRatio\":%u,\"deepSleepRatio\":%u,"
                  "\"outOfBedDuration\":%u,\"outOfBedCount\":%u,\"turnCount\":%u,"
                  "\"avgBreathingRate\":%u,\"avgHeartRate\":%u,\"apneaCount\":%u,"
                  "\"abnormalState\":%u,\"bodyMovement\":%u,\"breathStatus\":%u,"
                  "\"sleepState\":%u,\"largeMoveRatio\":%u,\"smallMoveRatio\":%u,"
                  "\"struggleAlert\":%u,\"noOneAlert\":%u,\"awakeDuration\":%u,"
                  "\"lightSleepDuration\":%u,\"deepSleepDuration\":%u}}",
                  static_cast<unsigned long>(gMqttMessageId++),
                  gMqttDeviceName,
                  static_cast<unsigned>(snapshot->sleep_score),
                  static_cast<unsigned>(snapshot->sleep_grade),
                  static_cast<unsigned>(snapshot->sleep_total_time),
                  static_cast<unsigned>(snapshot->awake_ratio),
                  static_cast<unsigned>(snapshot->light_sleep_ratio),
                  static_cast<unsigned>(snapshot->deep_sleep_ratio),
                  static_cast<unsigned>(snapshot->bed_Out_Time),
                  static_cast<unsigned>(snapshot->turn_count),
                  static_cast<unsigned>(snapshot->turnover_count),
                  static_cast<unsigned>(snapshot->avg_breath_rate),
                  static_cast<unsigned>(snapshot->avg_heart_rate),
                  static_cast<unsigned>(snapshot->apnea_count),
                  static_cast<unsigned>(snapshot->abnormal_state),
                  static_cast<unsigned>(snapshot->body_movement),
                  static_cast<unsigned>(snapshot->breath_status),
                  static_cast<unsigned>(snapshot->sleep_state),
                  static_cast<unsigned>(snapshot->large_move_ratio),
                  static_cast<unsigned>(snapshot->small_move_ratio),
                  static_cast<unsigned>(snapshot->struggle_alert),
                  static_cast<unsigned>(snapshot->no_one_alert),
                  static_cast<unsigned>(snapshot->awake_time),
                  static_cast<unsigned>(snapshot->light_sleep_time),
                  static_cast<unsigned>(snapshot->deep_sleep_time));

    const bool ok = mqtt_manager_publish_text(gMqttPropertyPostTopic, sleepPayload, 1, 0);
    ESP_LOGI(TAG, "mqtt sleep publish topic=%s ok=%s", gMqttPropertyPostTopic, ok ? "yes" : "no");
    return ok;
}

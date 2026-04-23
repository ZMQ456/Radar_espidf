#include "mqtt_manager.h"

#include <cstdio>
#include <cstring>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "radar_platform.h"
#include "system_state.h"
#include "wifi_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mqtt_manager";
static MqttManagerSnapshot gMqttState = {};
static esp_mqtt_client_handle_t gMqttClient = nullptr;
static TaskHandle_t gMqttConnectTaskHandle = nullptr;

static constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;

static void mqtt_manager_refresh_test_topic(void)
{
    if (gMqttState.client_id[0] == '\0') {
        gMqttState.test_topic[0] = '\0';
        return;
    }
    std::snprintf(gMqttState.test_topic,
                  sizeof(gMqttState.test_topic),
                  "radar/%s/status",
                  gMqttState.client_id);
}

static void mqtt_manager_build_data_topic(char *buffer, size_t buffer_size)
{
    if (gMqttState.client_id[0] == '\0') {
        if (buffer_size > 0U) {
            buffer[0] = '\0';
        }
        return;
    }

    std::snprintf(buffer,
                  buffer_size,
                  "radar/%s/data",
                  gMqttState.client_id);
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
    (void)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        gMqttState.connected = 1U;
        mqtt_manager_set_state_internal(MQTT_MANAGER_CONNECTED);
        ESP_LOGI(TAG, "mqtt connected");
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
                esp_mqtt_client_config_t mqttCfg = {};
                mqttCfg.broker.address.uri = gMqttState.uri;
                mqttCfg.credentials.client_id = gMqttState.client_id;
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
    if (uri == nullptr || client_id == nullptr || uri[0] == '\0' || client_id[0] == '\0') {
        ESP_LOGW(TAG, "mqtt config rejected: empty uri or client_id");
        return false;
    }

    std::strncpy(gMqttState.uri, uri, sizeof(gMqttState.uri) - 1U);
    gMqttState.uri[sizeof(gMqttState.uri) - 1U] = '\0';
    std::strncpy(gMqttState.client_id, client_id, sizeof(gMqttState.client_id) - 1U);
    gMqttState.client_id[sizeof(gMqttState.client_id) - 1U] = '\0';
    mqtt_manager_refresh_test_topic();
    gMqttState.configured = 1U;

    if (gMqttClient != nullptr) {
        esp_mqtt_client_stop(gMqttClient);
        esp_mqtt_client_destroy(gMqttClient);
        gMqttClient = nullptr;
    }

    mqtt_manager_set_state_internal(MQTT_MANAGER_INITIALIZED);
    ESP_LOGI(TAG, "mqtt configured uri=%s client_id=%s", gMqttState.uri, gMqttState.client_id);
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
    if (gMqttState.test_topic[0] == '\0') {
        return false;
    }

    char payload[192];
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"event\":\"online\",\"client_id\":\"%s\",\"uptime_ms\":%llu}",
                  gMqttState.client_id,
                  (unsigned long long)radar_now_ms());

    const bool ok = mqtt_manager_publish_text(gMqttState.test_topic, payload, 1, 0);
    ESP_LOGI(TAG, "mqtt online publish topic=%s ok=%s", gMqttState.test_topic, ok ? "yes" : "no");
    return ok;
}

bool mqtt_manager_publish_radar_snapshot(const RadarReportSnapshot *snapshot)
{
    if (snapshot == nullptr) {
        return false;
    }

    char topic[96];
    mqtt_manager_build_data_topic(topic, sizeof(topic));
    if (topic[0] == '\0') {
        return false;
    }

    char payload[768];
    std::snprintf(payload,
                  sizeof(payload),
                  "{\"event\":\"radar_data\",\"client_id\":\"%s\",\"heart_rate\":%.1f,\"breath_rate\":%.1f,"
                  "\"distance\":%u,\"presence\":%u,\"motion\":%u,\"sleep_state\":%u,\"body_movement\":%u,"
                  "\"abnormal_state\":%u,\"heart_valid\":%u,\"breath_valid\":%u,\"has_person\":%u,"
                  "\"bed_occupied\":%u,\"has_valid_vitals\":%u,\"has_fresh_sample\":%u,"
                  "\"has_meaningful_change\":%u,\"sample_age_ms\":%lu}",
                  gMqttState.client_id,
                  snapshot->heart_rate,
                  snapshot->breath_rate,
                  static_cast<unsigned>(snapshot->distance),
                  static_cast<unsigned>(snapshot->presence),
                  static_cast<unsigned>(snapshot->motion),
                  static_cast<unsigned>(snapshot->sleep_state),
                  static_cast<unsigned>(snapshot->body_movement),
                  static_cast<unsigned>(snapshot->abnormal_state),
                  static_cast<unsigned>(snapshot->heart_valid),
                  static_cast<unsigned>(snapshot->breath_valid),
                  static_cast<unsigned>(snapshot->has_person),
                  static_cast<unsigned>(snapshot->bed_occupied),
                  static_cast<unsigned>(snapshot->has_valid_vitals),
                  static_cast<unsigned>(snapshot->has_fresh_sample),
                  static_cast<unsigned>(snapshot->has_meaningful_change),
                  static_cast<unsigned long>(snapshot->sample_age_ms));

    const bool ok = mqtt_manager_publish_text(topic, payload, 1, 0);
    ESP_LOGI(TAG, "mqtt radar publish topic=%s ok=%s", topic, ok ? "yes" : "no");
    return ok;
}

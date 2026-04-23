#include "app_config.h"
#include "device_identity.h"
#include "esp_log.h"
#include "radar_manager.h"
#include "system_state.h"
#include "tasks_manager.h"
#include "mqtt_manager.h"
#include <inttypes.h>
#include <cstdio>

static const char *TAG = "radar_bootstrap";

extern "C" void app_main(void)
{
    system_state_init();

    if (!device_identity_init()) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "device identity init failed");
        return;
    }

    const SystemStateSnapshot bootState = system_state_get_snapshot();
    ESP_LOGI(TAG, "starting minimal radar task scaffold");
    ESP_LOGI(TAG,
             "bootstrap identity device_id=%u device_sn=%" PRIu64,
             (unsigned)bootState.current_device_id,
             bootState.device_sn);

    if (!initRadarManager()) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "radar manager init failed");
        return;
    }

    if (!initAllTasks()) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "task init failed");
        return;
    }

    if (!mqtt_manager_init()) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "mqtt manager init failed");
        return;
    }

    char mqttClientId[64] = {};
    if (APP_MQTT_CLIENT_ID[0] != '\0') {
        std::snprintf(mqttClientId, sizeof(mqttClientId), "%s", APP_MQTT_CLIENT_ID);
    } else if (!radar_manager_get_default_mqtt_client_id(mqttClientId, sizeof(mqttClientId))) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "default mqtt client id build failed");
        return;
    }

    if (!mqtt_manager_configure(APP_MQTT_BROKER_URI, mqttClientId)) {
        system_state_set_error(1U);
        ESP_LOGE(TAG, "mqtt manager configure failed");
        return;
    }

    char deviceName[32] = {};
    char deviceMac[18] = {};
    const bool hasDeviceName = radar_manager_get_device_name(deviceName, sizeof(deviceName));
    const bool hasDeviceMac = radar_manager_get_device_mac(deviceMac, sizeof(deviceMac));
    const uint32_t deviceHash = radar_manager_generate_device_hash();

    ESP_LOGI(TAG,
             "identity name=%s mac=%s hash=0x%08X mqtt_client_id=%s",
             hasDeviceName ? deviceName : "-",
             hasDeviceMac ? deviceMac : "-",
             static_cast<unsigned>(deviceHash),
             mqttClientId);
    ESP_LOGI(TAG, "mqtt configured uri=%s client_id=%s", APP_MQTT_BROKER_URI, mqttClientId);

    system_state_set_phase(SYSTEM_PHASE_RUNNING);
    ESP_LOGI(TAG, "system bootstrap complete");
}

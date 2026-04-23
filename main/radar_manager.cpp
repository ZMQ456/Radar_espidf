#include "radar_manager.h"

#include <cstdio>
#include <cstring>

#include "device_identity.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "radar_platform.h"
#include "radar_uart.h"
#include "system_state.h"

static const char *TAG = "radar_manager";
static bool radarManagerInitialized = false;
static RadarVitalsSnapshot lastConsumedSnapshot = {};
static bool hasConsumedSnapshot = false;

SensorData sensorData = {};

static float abs_float(float value)
{
    return value >= 0.0f ? value : -value;
}

static uint32_t calculate_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ (0xEDB88320U & static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1U))));
        }
    }
    return ~crc;
}

bool initRadarManager(void)
{
    if (radarManagerInitialized) {
        return true;
    }

    ESP_LOGI(TAG, "initializing radar manager");

    if (!radar_uart_init()) {
        ESP_LOGE(TAG, "radar uart init failed");
        system_state_set_error(1U);
        return false;
    }

    radar_uart_start_task();
    initR60ABD1();

    radarManagerInitialized = true;
    system_state_set_radar_ready(1U);
    system_state_set_phase(SYSTEM_PHASE_RADAR_READY);
    ESP_LOGI(TAG, "radar manager initialized");
    return true;
}

bool radar_manager_is_initialized(void)
{
    return radarManagerInitialized;
}

SensorData radar_manager_get_sensor_data(void)
{
    return sensorData;
}

bool radar_manager_has_person(void)
{
    return sensorData.presence != 0U;
}

bool radar_manager_is_bed_occupied(void)
{
    return sensorData.presence != 0U || sensorData.sleep_state > 0U;
}

bool radar_manager_has_valid_vitals(void)
{
    return sensorData.heart_valid != 0U || sensorData.breath_valid != 0U;
}

bool radar_manager_has_fresh_sample(uint32_t max_age_ms)
{
    if (sensorData.last_update_ms == 0) {
        return false;
    }

    const uint64_t nowMs = radar_now_ms();
    return nowMs >= sensorData.last_update_ms &&
           (nowMs - sensorData.last_update_ms) <= max_age_ms;
}

RadarVitalsSnapshot radar_manager_get_vitals_snapshot(void)
{
    RadarVitalsSnapshot snapshot = {};
    snapshot.heart_rate = sensorData.heart_rate;
    snapshot.breath_rate = sensorData.breath_rate;
    snapshot.presence = sensorData.presence;
    snapshot.motion = sensorData.motion;
    snapshot.sleep_state = sensorData.sleep_state;
    snapshot.body_movement = sensorData.body_movement;
    snapshot.distance = sensorData.distance;
    snapshot.abnormal_state = sensorData.abnormal_state;
    snapshot.last_update_ms = sensorData.last_update_ms;
    return snapshot;
}

RadarChangeFlags radar_manager_get_change_flags(void)
{
    RadarChangeFlags flags = {};
    const RadarVitalsSnapshot current = radar_manager_get_vitals_snapshot();

    if (!hasConsumedSnapshot) {
        if (current.last_update_ms != 0U) {
            flags.any_changed = 1U;
        }
        return flags;
    }

    flags.presence_changed = current.presence != lastConsumedSnapshot.presence;
    flags.motion_changed = current.motion != lastConsumedSnapshot.motion;
    flags.sleep_state_changed = current.sleep_state != lastConsumedSnapshot.sleep_state;
    flags.body_movement_changed =
        abs_float((float)current.body_movement - (float)lastConsumedSnapshot.body_movement) >= 5.0f;
    flags.distance_changed =
        current.distance > lastConsumedSnapshot.distance
            ? (current.distance - lastConsumedSnapshot.distance) >= 10U
            : (lastConsumedSnapshot.distance - current.distance) >= 10U;
    flags.heart_rate_changed =
        abs_float(current.heart_rate - lastConsumedSnapshot.heart_rate) >= 3.0f;
    flags.breath_rate_changed =
        abs_float(current.breath_rate - lastConsumedSnapshot.breath_rate) >= 2.0f;
    flags.abnormal_state_changed = current.abnormal_state != lastConsumedSnapshot.abnormal_state;

    flags.any_changed = flags.presence_changed ||
                        flags.motion_changed ||
                        flags.sleep_state_changed ||
                        flags.body_movement_changed ||
                        flags.distance_changed ||
                        flags.heart_rate_changed ||
                        flags.breath_rate_changed ||
                        flags.abnormal_state_changed;
    return flags;
}

bool radar_manager_has_meaningful_change(void)
{
    return radar_manager_get_change_flags().any_changed != 0U;
}

void radar_manager_mark_snapshot_consumed(void)
{
    lastConsumedSnapshot = radar_manager_get_vitals_snapshot();
    hasConsumedSnapshot = true;
}

RadarReportSnapshot radar_manager_get_report_snapshot(uint32_t max_age_ms)
{
    RadarReportSnapshot snapshot = {};
    const bool hasFreshSample = radar_manager_has_fresh_sample(max_age_ms);
    const bool hasPerson = radar_manager_has_person();
    const bool bedOccupied = radar_manager_is_bed_occupied();
    const bool hasValidVitals = radar_manager_has_valid_vitals();
    const bool hasMeaningfulChange = radar_manager_has_meaningful_change();

    snapshot.heart_rate = sensorData.heart_rate;
    snapshot.breath_rate = sensorData.breath_rate;
    snapshot.distance = sensorData.distance;
    snapshot.presence = sensorData.presence;
    snapshot.motion = sensorData.motion;
    snapshot.sleep_state = sensorData.sleep_state;
    snapshot.body_movement = sensorData.body_movement;
    snapshot.abnormal_state = sensorData.abnormal_state;
    snapshot.heart_valid = sensorData.heart_valid;
    snapshot.breath_valid = sensorData.breath_valid;
    snapshot.has_person = hasPerson ? 1U : 0U;
    snapshot.bed_occupied = bedOccupied ? 1U : 0U;
    snapshot.has_valid_vitals = hasValidVitals ? 1U : 0U;
    snapshot.has_fresh_sample = hasFreshSample ? 1U : 0U;
    snapshot.has_meaningful_change = hasMeaningfulChange ? 1U : 0U;
    snapshot.last_update_ms = sensorData.last_update_ms;

    if (sensorData.last_update_ms != 0U) {
        const uint64_t nowMs = radar_now_ms();
        if (nowMs >= sensorData.last_update_ms) {
            snapshot.sample_age_ms = (uint32_t)(nowMs - sensorData.last_update_ms);
        }
    }

    return snapshot;
}

bool radar_manager_get_device_mac(char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size < 18U) {
        return false;
    }

    uint8_t mac[6] = {};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        buffer[0] = '\0';
        return false;
    }

    std::snprintf(buffer,
                  buffer_size,
                  "%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0],
                  mac[1],
                  mac[2],
                  mac[3],
                  mac[4],
                  mac[5]);
    return true;
}

uint32_t radar_manager_generate_device_hash(void)
{
    char macStr[18] = {};
    if (!radar_manager_get_device_mac(macStr, sizeof(macStr))) {
        return 0U;
    }

    const DeviceIdentity identity = device_identity_get();
    char hashInput[96] = {};
    std::snprintf(hashInput,
                  sizeof(hashInput),
                  "SN%llu|%s",
                  static_cast<unsigned long long>(identity.device_sn),
                  macStr);

    const uint32_t hash = calculate_crc32(reinterpret_cast<const uint8_t *>(hashInput),
                                          std::strlen(hashInput));
    ESP_LOGI(TAG,
             "device hash input=%s hash=0x%08X",
             hashInput,
             static_cast<unsigned>(hash));
    return hash;
}

bool radar_manager_get_device_name(char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return false;
    }

    const DeviceIdentity identity = device_identity_get();
    if (identity.device_sn != 0ULL) {
        std::snprintf(buffer,
                      buffer_size,
                      "Radar_%llu",
                      static_cast<unsigned long long>(identity.device_sn));
        return true;
    }

    char macStr[18] = {};
    if (!radar_manager_get_device_mac(macStr, sizeof(macStr))) {
        buffer[0] = '\0';
        return false;
    }

    char compactMac[13] = {};
    size_t outIndex = 0U;
    for (size_t i = 0; macStr[i] != '\0' && outIndex < (sizeof(compactMac) - 1U); ++i) {
        if (macStr[i] != ':') {
            compactMac[outIndex++] = macStr[i];
        }
    }
    compactMac[outIndex] = '\0';

    std::snprintf(buffer, buffer_size, "Radar_%s", compactMac);
    return true;
}

bool radar_manager_get_default_mqtt_client_id(char *buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U) {
        return false;
    }

    const DeviceIdentity identity = device_identity_get();
    if (identity.device_sn != 0ULL) {
        std::snprintf(buffer,
                      buffer_size,
                      "espidf-radar-%llu",
                      static_cast<unsigned long long>(identity.device_sn));
        return true;
    }

    const uint32_t hash = radar_manager_generate_device_hash();
    if (hash == 0U) {
        buffer[0] = '\0';
        return false;
    }

    std::snprintf(buffer, buffer_size, "espidf-radar-%08X", static_cast<unsigned>(hash));
    return true;
}

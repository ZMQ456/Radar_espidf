#include "influx_manager.h"

#include "app_config.h"
#include "device_identity.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "radar_platform.h"
#include "wifi_manager.h"
#include <cstdio>
#include <cstring>

static const char *TAG = "influx_manager";
static constexpr uint32_t kRefusedCooldownMs = 10000U;
static constexpr uint8_t kInfluxFailReconnectThreshold = 3U;
static uint64_t gLastRefusedTimeMs = 0U;
static uint8_t gInfluxFailCount = 0U;

static bool influx_is_in_refused_cooldown(void)
{
    if (gLastRefusedTimeMs == 0U) {
        return false;
    }
    return (radar_now_ms() - gLastRefusedTimeMs) < kRefusedCooldownMs;
}

static bool influx_wifi_ready(void)
{
    const WiFiManagerSnapshot snapshot = wifi_manager_get_snapshot();
    if (!snapshot.connected) {
        ESP_LOGD(TAG, "WiFi not connected, skip InfluxDB send");
        return false;
    }
    if (snapshot.rssi < -85) {
        ESP_LOGW(TAG, "WiFi signal too weak (%d dBm), skip InfluxDB send", (int)snapshot.rssi);
        return false;
    }
    if (snapshot.ip[0] == '\0') {
        ESP_LOGW(TAG, "WiFi has no IP, skip InfluxDB send");
        return false;
    }
    if (influx_is_in_refused_cooldown()) {
        ESP_LOGW(TAG, "InfluxDB refused cooldown active, skip send");
        return false;
    }
    return true;
}

static bool influx_post_line(const char *line, uint32_t timeout_ms)
{
    if (line == nullptr || line[0] == '\0') {
        return false;
    }
    if (!influx_wifi_ready()) {
        return false;
    }

    char url[192] = {};
    std::snprintf(url,
                  sizeof(url),
                  "http://%s:%u/api/v2/write?org=%s&bucket=%s",
                  APP_INFLUXDB_HOST,
                  (unsigned)APP_INFLUXDB_PORT,
                  APP_INFLUXDB_ORG,
                  APP_INFLUXDB_BUCKET);

    esp_http_client_config_t config = {};
    config.url = url;
    config.method = HTTP_METHOD_POST;
    config.timeout_ms = (int)timeout_ms;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        ESP_LOGE(TAG, "http client init failed");
        return false;
    }

    char auth[192] = {};
    std::snprintf(auth, sizeof(auth), "Token %s", APP_INFLUXDB_TOKEN);
    esp_http_client_set_header(client, "Authorization", auth);
    esp_http_client_set_header(client, "Content-Type", "text/plain; charset=utf-8");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Host", APP_INFLUXDB_HOST);
    esp_http_client_set_post_field(client, line, std::strlen(line));

    const esp_err_t err = esp_http_client_perform(client);
    const int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err == ESP_OK && status == 204) {
        gInfluxFailCount = 0U;
        ESP_LOGI(TAG, "InfluxDB write ok: %s", line);
        return true;
    }

    ESP_LOGW(TAG, "InfluxDB write failed err=%s status=%d", esp_err_to_name(err), status);
    if (err != ESP_OK || status == 0) {
        if (gInfluxFailCount < 255U) {
            ++gInfluxFailCount;
        }
        gLastRefusedTimeMs = radar_now_ms();
        if (gInfluxFailCount >= kInfluxFailReconnectThreshold) {
            ESP_LOGW(TAG, "InfluxDB failures reached %u, reconnect WiFi", (unsigned)gInfluxFailCount);
            gInfluxFailCount = 0U;
            wifi_manager_disconnect();
            (void)wifi_manager_connect();
        }
    }
    return false;
}

bool influx_manager_send_daily_data(const RadarReportSnapshot *snapshot)
{
    if (snapshot == nullptr) {
        return false;
    }
    if (snapshot->heart_rate <= 0.0f || snapshot->breath_rate <= 0.0f) {
        ESP_LOGD(TAG, "heart/breath empty, skip daily InfluxDB send");
        return false;
    }
    if (snapshot->heart_rate > 200.0f || snapshot->breath_rate > 50.0f) {
        ESP_LOGW(TAG, "invalid vital data hr=%.1f rr=%.1f, skip InfluxDB send",
                 snapshot->heart_rate,
                 snapshot->breath_rate);
        return false;
    }

    const DeviceIdentity identity = device_identity_get();
    char line[512] = {};
    std::snprintf(line,
                  sizeof(line),
                  "daily_data,deviceId=%u,dataType=daily "
                  "heartRate=%.1f,breathingRate=%.1f,"
                  "personDetected=%ui,humanActivity=%ui,humanDistance=%ui,"
                  "sleepState=%ui,humanPositionX=%di,humanPositionY=%di,humanPositionZ=%di,"
                  "heartbeatWaveform=%di,breathingWaveform=%di,abnormalState=%ui,"
                  "bedStatus=%ui,struggleAlert=%ui,noOneAlert=%ui",
                  (unsigned)identity.device_id,
                  snapshot->heart_rate,
                  snapshot->breath_rate,
                  (unsigned)snapshot->presence,
                  (unsigned)snapshot->motion,
                  (unsigned)snapshot->distance,
                  (unsigned)snapshot->sleep_state,
                  (int)snapshot->pos_x,
                  (int)snapshot->pos_y,
                  (int)snapshot->pos_z,
                  snapshot->heartbeat_waveform,
                  snapshot->breathing_waveform,
                  (unsigned)snapshot->abnormal_state,
                  (unsigned)snapshot->bed_status,
                  (unsigned)snapshot->struggle_alert,
                  (unsigned)snapshot->no_one_alert);

    return influx_post_line(line, 15000U);
}

bool influx_manager_send_sleep_data(const SensorData *data)
{
    if (data == nullptr) {
        return false;
    }
    if (data->sleep_total_time == 0U) {
        return false;
    }

    const DeviceIdentity identity = device_identity_get();
    char line[768] = {};
    std::snprintf(line,
                  sizeof(line),
                  "sleep_data,deviceId=%u,dataType=sleep "
                  "sleepQualityScore=%ui,sleepQualityGrade=%ui,totalSleepDuration=%ui,"
                  "awakeDurationRatio=%ui,lightSleepRatio=%ui,deepSleepRatio=%ui,"
                  "outOfBedDuration=%ui,outOfBedCount=%ui,turnCount=%ui,"
                  "avgBreathingRate=%ui,avgHeartRate=%ui,apneaCount=%ui,"
                  "abnormalState=%ui,bodyMovement=%ui,breathStatus=%ui,sleepState=%ui,"
                  "largeMoveRatio=%ui,smallMoveRatio=%ui,struggleAlert=%ui,noOneAlert=%ui,"
                  "awakeDuration=%ui,lightSleepDuration=%ui,deepSleepDuration=%ui",
                  (unsigned)identity.device_id,
                  (unsigned)data->sleep_score,
                  (unsigned)data->sleep_grade,
                  (unsigned)data->sleep_total_time,
                  (unsigned)data->awake_ratio,
                  (unsigned)data->light_sleep_ratio,
                  (unsigned)data->deep_sleep_ratio,
                  (unsigned)data->bed_Out_Time,
                  (unsigned)data->turn_count,
                  (unsigned)data->turnover_count,
                  (unsigned)data->avg_breath_rate,
                  (unsigned)data->avg_heart_rate,
                  (unsigned)data->apnea_count,
                  (unsigned)data->abnormal_state,
                  (unsigned)data->body_movement,
                  (unsigned)data->breath_status,
                  (unsigned)data->sleep_state,
                  (unsigned)data->large_move_ratio,
                  (unsigned)data->small_move_ratio,
                  (unsigned)data->struggle_alert,
                  (unsigned)data->no_one_alert,
                  (unsigned)data->awake_time,
                  (unsigned)data->light_sleep_time,
                  (unsigned)data->deep_sleep_time);

    for (uint8_t retry = 0U; retry < 3U; ++retry) {
        if (retry > 0U) {
            radar_sleep_ms(500);
        }
        if (influx_post_line(line, 5000U)) {
            return true;
        }
    }

    return false;
}

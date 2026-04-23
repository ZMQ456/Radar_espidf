#include "wifi_manager.h"

#include <cstring>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "radar_platform.h"
#include "system_state.h"
#include "tasks_manager.h"

static const char *TAG = "wifi_manager";
static WiFiManagerSnapshot gWifiState = {};
static char gSsid[33] = {};
static char gPassword[65] = {};
static constexpr uint32_t WIFI_MANAGER_MAX_SAVED_NETWORKS = 10U;
typedef struct {
    char ssid[33];
    char password[65];
} SavedWiFiCredential;
static SavedWiFiCredential gSavedNetworks[WIFI_MANAGER_MAX_SAVED_NETWORKS] = {};
static uint32_t gSavedNetworkCount = 0U;
static esp_netif_t *gStaNetif = nullptr;
static EventGroupHandle_t gWifiEventGroup = nullptr;
static bool gEspNetifInitialized = false;
static bool gWifiDriverInitialized = false;
static bool gEventHandlersRegistered = false;
static TaskHandle_t gReconnectTaskHandle = nullptr;
static SemaphoreHandle_t gWifiOperationMutex = nullptr;
static volatile bool gManualConfigActive = false;
static volatile bool gScanInProgress = false;

static constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
static constexpr EventBits_t WIFI_FAIL_BIT = BIT1;
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
static constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 5000;
static constexpr int8_t WIFI_MIN_RSSI_THRESHOLD = -127;
static const char *WIFI_NVS_NAMESPACE = "wifi_cfg";
static const char *WIFI_NVS_KEY_SSID = "ssid";
static const char *WIFI_NVS_KEY_PASS = "pass";

static void wifi_manager_set_state_internal(WiFiManagerState state);

static const char *wifi_manager_authmode_to_string(wifi_auth_mode_t authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        return "open";
    case WIFI_AUTH_WEP:
        return "WEP";
    case WIFI_AUTH_WPA_PSK:
        return "WPA";
    case WIFI_AUTH_WPA2_PSK:
        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:
        return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE:
        return "WPA2-EAP";
    case WIFI_AUTH_WPA3_PSK:
        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:
        return "WPA2/WPA3";
    default:
        return "unknown";
    }
}

static bool wifi_manager_find_saved_password(const char *ssid, char *password, size_t password_size)
{
    if (ssid == nullptr || password == nullptr || password_size == 0U) {
        return false;
    }

    for (uint32_t i = 0; i < gSavedNetworkCount; ++i) {
        if (std::strcmp(gSavedNetworks[i].ssid, ssid) == 0) {
            std::strncpy(password, gSavedNetworks[i].password, password_size - 1U);
            password[password_size - 1U] = '\0';
            return true;
        }
    }

    return false;
}

static void wifi_manager_copy_active_credentials(const char *ssid, const char *password)
{
    std::strncpy(gSsid, ssid, sizeof(gSsid) - 1U);
    gSsid[sizeof(gSsid) - 1U] = '\0';
    std::strncpy(gPassword, password, sizeof(gPassword) - 1U);
    gPassword[sizeof(gPassword) - 1U] = '\0';
    std::strncpy(gWifiState.ssid, gSsid, sizeof(gWifiState.ssid) - 1U);
    gWifiState.ssid[sizeof(gWifiState.ssid) - 1U] = '\0';
}

static void wifi_manager_make_indexed_key(char *buffer,
                                          size_t buffer_size,
                                          const char *prefix,
                                          uint32_t index)
{
    snprintf(buffer, buffer_size, "%s%u", prefix, (unsigned)index);
}

static bool wifi_manager_connect_with_credentials(const char *ssid, const char *password)
{
    if (ssid == nullptr || password == nullptr || ssid[0] == '\0') {
        return false;
    }

    wifi_manager_copy_active_credentials(ssid, password);
    wifi_manager_set_state_internal(WIFI_MANAGER_CONNECTING);
    gWifiState.reconnect_attempts += 1U;
    gWifiState.connected = 0U;
    gWifiState.rssi = -127;

    if (gWifiEventGroup != nullptr) {
        xEventGroupClearBits(gWifiEventGroup, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    }

    wifi_config_t wifiConfig = {};
    std::strncpy(reinterpret_cast<char *>(wifiConfig.sta.ssid),
                 gSsid,
                 sizeof(wifiConfig.sta.ssid) - 1U);
    std::strncpy(reinterpret_cast<char *>(wifiConfig.sta.password),
                 gPassword,
                 sizeof(wifiConfig.sta.password) - 1U);
    wifiConfig.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifiConfig.sta.pmf_cfg.capable = true;
    wifiConfig.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));

    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_STATE) {
        ESP_LOGE(TAG, "esp_wifi_start failed for ssid=%s: %s", gSsid, esp_err_to_name(err));
        wifi_manager_set_state_internal(WIFI_MANAGER_ERROR);
        return false;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect failed for ssid=%s: %s", gSsid, esp_err_to_name(err));
        wifi_manager_set_state_internal(WIFI_MANAGER_ERROR);
        return false;
    }

    if (gWifiEventGroup == nullptr) {
        ESP_LOGE(TAG, "wifi event group missing");
        wifi_manager_set_state_internal(WIFI_MANAGER_ERROR);
        return false;
    }

    const EventBits_t bits = xEventGroupWaitBits(gWifiEventGroup,
                                                 WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                 pdTRUE,
                                                 pdFALSE,
                                                 pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if ((bits & WIFI_CONNECTED_BIT) != 0U) {
        ESP_LOGI(TAG, "wifi connected to ssid=%s", gSsid);
        return true;
    }

    ESP_LOGW(TAG, "wifi connect timeout/fail ssid=%s", gSsid);
    wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
    return false;
}

static void wifi_manager_set_state_internal(WiFiManagerState state)
{
    gWifiState.state = state;
    gWifiState.last_state_change_ms = radar_now_ms();

    switch (state) {
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
        setNetworkStatus(NET_DISCONNECTED);
        break;
    case WIFI_MANAGER_ERROR:
        setNetworkStatus(NET_DISCONNECTED);
        break;
    default:
        setNetworkStatus(NET_INITIAL);
        break;
    }
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        WiFiEvent(TASK_WIFI_EVENT_STA_START);
        ESP_LOGI(TAG, "wifi sta started");
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        WiFiEvent(TASK_WIFI_EVENT_STA_CONNECTED);
        ESP_LOGI(TAG, "wifi sta connected");
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        gWifiState.connected = 0U;
        gWifiState.rssi = -127;
        gWifiState.ip[0] = '\0';
        wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
        WiFiEvent(TASK_WIFI_EVENT_STA_DISCONNECTED);
        if (gWifiEventGroup != nullptr) {
            xEventGroupSetBits(gWifiEventGroup, WIFI_FAIL_BIT);
        }
        ESP_LOGW(TAG, "wifi disconnected");
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        gWifiState.connected = 1U;
        wifi_manager_set_state_internal(WIFI_MANAGER_CONNECTED);
        WiFiEvent(TASK_WIFI_EVENT_STA_GOT_IP);

        wifi_ap_record_t apInfo = {};
        if (esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK) {
            gWifiState.rssi = static_cast<int8_t>(apInfo.rssi);
        }

        esp_netif_ip_info_t ipInfo = {};
        if (gStaNetif != nullptr && esp_netif_get_ip_info(gStaNetif, &ipInfo) == ESP_OK) {
            snprintf(gWifiState.ip,
                     sizeof(gWifiState.ip),
                     IPSTR,
                     IP2STR(&ipInfo.ip));
        }

        if (gWifiEventGroup != nullptr) {
            xEventGroupSetBits(gWifiEventGroup, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG,
                 "wifi got ip, ssid=%s ip=%s rssi=%d",
                 gSsid,
                 gWifiState.ip,
                 static_cast<int>(gWifiState.rssi));
    }
}

static void wifi_reconnect_task(void *parameter)
{
    (void)parameter;

    while (true) {
        const bool shouldReconnect =
            gWifiState.initialized != 0U &&
            gWifiState.has_credentials != 0U &&
            gWifiState.connected == 0U &&
            !gManualConfigActive &&
            !gScanInProgress &&
            gWifiState.state != WIFI_MANAGER_CONNECTING &&
            gWifiState.state != WIFI_MANAGER_CONFIGURING &&
            gWifiState.state != WIFI_MANAGER_ERROR;

        if (shouldReconnect) {
            ESP_LOGI(TAG, "background wifi reconnect attempt");
            (void)wifi_manager_connect();
        }

        vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_INTERVAL_MS));
    }
}

static bool wifi_manager_ensure_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool wifi_manager_load_credentials_from_nvs(void)
{
    gSavedNetworkCount = 0U;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs open for read failed: %s", esp_err_to_name(err));
        return false;
    }

    for (uint32_t i = 0; i < WIFI_MANAGER_MAX_SAVED_NETWORKS; ++i) {
        char ssidKey[16] = {};
        char passKey[16] = {};
        wifi_manager_make_indexed_key(ssidKey, sizeof(ssidKey), "wifi_", i);
        wifi_manager_make_indexed_key(passKey, sizeof(passKey), "pass_", i);

        size_t ssidLen = sizeof(gSavedNetworks[i].ssid);
        size_t passLen = sizeof(gSavedNetworks[i].password);
        esp_err_t ssidErr = nvs_get_str(handle, ssidKey, gSavedNetworks[i].ssid, &ssidLen);
        if (ssidErr != ESP_OK) {
            continue;
        }

        esp_err_t passErr = nvs_get_str(handle, passKey, gSavedNetworks[i].password, &passLen);
        if (passErr != ESP_OK) {
            gSavedNetworks[i].ssid[0] = '\0';
            continue;
        }

        ++gSavedNetworkCount;
    }

    if (gSavedNetworkCount == 0U) {
        size_t ssidLen = sizeof(gSsid);
        size_t passLen = sizeof(gPassword);
        err = nvs_get_str(handle, WIFI_NVS_KEY_SSID, gSsid, &ssidLen);
        if (err == ESP_OK) {
            err = nvs_get_str(handle, WIFI_NVS_KEY_PASS, gPassword, &passLen);
        }
        if (err == ESP_OK) {
            wifi_manager_copy_active_credentials(gSsid, gPassword);
            std::strncpy(gSavedNetworks[0].ssid, gSsid, sizeof(gSavedNetworks[0].ssid) - 1U);
            std::strncpy(gSavedNetworks[0].password, gPassword, sizeof(gSavedNetworks[0].password) - 1U);
            gSavedNetworkCount = 1U;
        }
    } else {
        wifi_manager_copy_active_credentials(gSavedNetworks[0].ssid, gSavedNetworks[0].password);
    }

    gWifiState.has_credentials = gSavedNetworkCount > 0U ? 1U : 0U;
    nvs_close(handle);
    if (gSavedNetworkCount > 0U) {
        ESP_LOGI(TAG, "loaded %u wifi credential set(s) from nvs, active ssid=%s",
                 (unsigned)gSavedNetworkCount,
                 gSsid);
        return true;
    }
    return false;
}

static bool wifi_manager_save_credentials_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open for write failed: %s", esp_err_to_name(err));
        return false;
    }

    for (uint32_t i = 0; i < WIFI_MANAGER_MAX_SAVED_NETWORKS && err == ESP_OK; ++i) {
        char ssidKey[16] = {};
        char passKey[16] = {};
        wifi_manager_make_indexed_key(ssidKey, sizeof(ssidKey), "wifi_", i);
        wifi_manager_make_indexed_key(passKey, sizeof(passKey), "pass_", i);

        if (i < gSavedNetworkCount && gSavedNetworks[i].ssid[0] != '\0') {
            err = nvs_set_str(handle, ssidKey, gSavedNetworks[i].ssid);
            if (err == ESP_OK) {
                err = nvs_set_str(handle, passKey, gSavedNetworks[i].password);
            }
        } else {
            esp_err_t ssidErr = nvs_erase_key(handle, ssidKey);
            if (ssidErr != ESP_OK && ssidErr != ESP_ERR_NVS_NOT_FOUND) {
                err = ssidErr;
            }
            if (err == ESP_OK) {
                esp_err_t passErr = nvs_erase_key(handle, passKey);
                if (passErr != ESP_OK && passErr != ESP_ERR_NVS_NOT_FOUND) {
                    err = passErr;
                }
            }
        }
    }
    if (err == ESP_OK) {
        err = nvs_set_str(handle, WIFI_NVS_KEY_SSID, gSsid);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(handle, WIFI_NVS_KEY_PASS, gPassword);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "saving wifi credentials failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "saved %u wifi credential set(s) to nvs, active ssid=%s",
             (unsigned)gSavedNetworkCount,
             gSsid);
    return true;
}

bool wifi_manager_clear_credentials(void)
{
    if (!wifi_manager_ensure_nvs()) {
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open for clear failed: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_erase_key(handle, WIFI_NVS_KEY_SSID);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        esp_err_t passErr = nvs_erase_key(handle, WIFI_NVS_KEY_PASS);
        if (passErr != ESP_OK && passErr != ESP_ERR_NVS_NOT_FOUND) {
            err = passErr;
        }
    }
    for (uint32_t i = 0; i < WIFI_MANAGER_MAX_SAVED_NETWORKS && err == ESP_OK; ++i) {
        char ssidKey[16] = {};
        char passKey[16] = {};
        wifi_manager_make_indexed_key(ssidKey, sizeof(ssidKey), "wifi_", i);
        wifi_manager_make_indexed_key(passKey, sizeof(passKey), "pass_", i);

        esp_err_t ssidErr = nvs_erase_key(handle, ssidKey);
        if (ssidErr != ESP_OK && ssidErr != ESP_ERR_NVS_NOT_FOUND) {
            err = ssidErr;
            break;
        }

        esp_err_t passErr = nvs_erase_key(handle, passKey);
        if (passErr != ESP_OK && passErr != ESP_ERR_NVS_NOT_FOUND) {
            err = passErr;
            break;
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "clearing wifi credentials failed: %s", esp_err_to_name(err));
        return false;
    }

    gSsid[0] = '\0';
    gPassword[0] = '\0';
    gWifiState.ssid[0] = '\0';
    gWifiState.has_credentials = 0U;
    gWifiState.reconnect_attempts = 0U;
    gSavedNetworkCount = 0U;
    std::memset(gSavedNetworks, 0, sizeof(gSavedNetworks));
    wifi_manager_disconnect();
    ESP_LOGI(TAG, "wifi credentials cleared");
    return true;
}

uint32_t wifi_manager_scan_networks(WiFiScanRecord *records, uint32_t max_records)
{
    if (records == nullptr || max_records == 0U || !gWifiState.initialized) {
        return 0U;
    }

    if (gWifiOperationMutex != nullptr &&
        xSemaphoreTake(gWifiOperationMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "wifi scan skipped because wifi operation is busy");
        return 0U;
    }

    gScanInProgress = true;
    wifi_manager_set_state_internal(WIFI_MANAGER_SCANNING);

    wifi_scan_config_t scanConfig = {};
    scanConfig.show_hidden = true;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK && err != ESP_ERR_WIFI_MODE) {
        ESP_LOGE(TAG, "esp_wifi_set_mode before scan failed: %s", esp_err_to_name(err));
        gScanInProgress = false;
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return 0U;
    }

    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_STATE) {
        ESP_LOGE(TAG, "esp_wifi_start before scan failed: %s", esp_err_to_name(err));
        gScanInProgress = false;
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return 0U;
    }

    uint16_t apCount = 0U;
    for (uint32_t retry = 0U; retry < 3U; ++retry) {
        err = esp_wifi_scan_start(&scanConfig, true);
        if (err == ESP_OK) {
            err = esp_wifi_scan_get_ap_num(&apCount);
            if (err == ESP_OK && apCount > 0U) {
                break;
            }
        }
        ESP_LOGW(TAG, "wifi scan attempt %u failed/empty: %s", (unsigned)(retry + 1U), esp_err_to_name(err));
        radar_sleep_ms(500);
    }

    if (err != ESP_OK || apCount == 0U) {
        gScanInProgress = false;
        wifi_manager_set_state_internal(gWifiState.connected ? WIFI_MANAGER_CONNECTED : WIFI_MANAGER_DISCONNECTED);
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return 0U;
    }

    wifi_ap_record_t apRecords[16] = {};
    uint16_t fetchCount = apCount > 16U ? 16U : apCount;
    err = esp_wifi_scan_get_ap_records(&fetchCount, apRecords);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_scan_get_ap_records failed: %s", esp_err_to_name(err));
        gScanInProgress = false;
        wifi_manager_set_state_internal(gWifiState.connected ? WIFI_MANAGER_CONNECTED : WIFI_MANAGER_DISCONNECTED);
        if (gWifiOperationMutex != nullptr) {
            xSemaphoreGive(gWifiOperationMutex);
        }
        return 0U;
    }

    uint32_t copyCount = 0U;
    for (uint16_t i = 0; i < fetchCount && copyCount < max_records; ++i) {
        if (apRecords[i].rssi < WIFI_MIN_RSSI_THRESHOLD) {
            continue;
        }

        std::strncpy(records[copyCount].ssid,
                     reinterpret_cast<const char *>(apRecords[i].ssid),
                     sizeof(records[copyCount].ssid) - 1U);
        records[copyCount].ssid[sizeof(records[copyCount].ssid) - 1U] = '\0';
        records[copyCount].rssi = static_cast<int8_t>(apRecords[i].rssi);
        records[copyCount].channel = static_cast<uint8_t>(apRecords[i].primary);
        std::strncpy(records[copyCount].encryption,
                     wifi_manager_authmode_to_string(apRecords[i].authmode),
                     sizeof(records[copyCount].encryption) - 1U);
        records[copyCount].encryption[sizeof(records[copyCount].encryption) - 1U] = '\0';
        ++copyCount;
    }

    gScanInProgress = false;
    wifi_manager_set_state_internal(gWifiState.connected ? WIFI_MANAGER_CONNECTED : WIFI_MANAGER_DISCONNECTED);
    if (gWifiOperationMutex != nullptr) {
        xSemaphoreGive(gWifiOperationMutex);
    }
    ESP_LOGI(TAG, "wifi scan finished with %u AP records", (unsigned)copyCount);
    return copyCount;
}

uint32_t wifi_manager_get_saved_networks(WiFiSavedNetwork *networks, uint32_t max_networks)
{
    if (networks == nullptr || max_networks == 0U || gSavedNetworkCount == 0U) {
        return 0U;
    }

    const uint32_t copyCount = gSavedNetworkCount < max_networks ? gSavedNetworkCount : max_networks;
    for (uint32_t i = 0; i < copyCount; ++i) {
        std::strncpy(networks[i].ssid, gSavedNetworks[i].ssid, sizeof(networks[i].ssid) - 1U);
        networks[i].ssid[sizeof(networks[i].ssid) - 1U] = '\0';
    }
    return copyCount;
}

bool wifi_manager_has_saved_network(const char *ssid)
{
    if (ssid == nullptr || ssid[0] == '\0') {
        return false;
    }

    for (uint32_t i = 0; i < gSavedNetworkCount; ++i) {
        if (std::strcmp(gSavedNetworks[i].ssid, ssid) == 0) {
            return true;
        }
    }

    return false;
}

static bool wifi_manager_ensure_stack(void)
{
    if (!wifi_manager_ensure_nvs()) {
        return false;
    }

    if (!gEspNetifInitialized) {
        ESP_ERROR_CHECK(esp_netif_init());
        esp_err_t err = esp_event_loop_create_default();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "event loop init failed: %s", esp_err_to_name(err));
            return false;
        }
        gEspNetifInitialized = true;
    }

    if (gStaNetif == nullptr) {
        gStaNetif = esp_netif_create_default_wifi_sta();
        if (gStaNetif == nullptr) {
            ESP_LOGE(TAG, "failed to create default wifi sta netif");
            return false;
        }
    }

    if (gWifiEventGroup == nullptr) {
        gWifiEventGroup = xEventGroupCreate();
        if (gWifiEventGroup == nullptr) {
            ESP_LOGE(TAG, "failed to create wifi event group");
            return false;
        }
    }

    if (gWifiOperationMutex == nullptr) {
        gWifiOperationMutex = xSemaphoreCreateMutex();
        if (gWifiOperationMutex == nullptr) {
            ESP_LOGE(TAG, "failed to create wifi operation mutex");
            return false;
        }
    }

    if (!gWifiDriverInitialized) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_wifi_init(&cfg);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
            return false;
        }
        gWifiDriverInitialized = true;
    }

    if (!gEventHandlersRegistered) {
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                                   ESP_EVENT_ANY_ID,
                                                   &wifi_event_handler,
                                                   nullptr));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                                   IP_EVENT_STA_GOT_IP,
                                                   &wifi_event_handler,
                                                   nullptr));
        gEventHandlersRegistered = true;
    }

    return true;
}

bool wifi_manager_init(void)
{
    if (gWifiState.initialized) {
        return true;
    }

    if (!wifi_manager_ensure_stack()) {
        wifi_manager_set_state_internal(WIFI_MANAGER_ERROR);
        system_state_set_error(1U);
        return false;
    }

    gWifiState.initialized = 1U;
    gWifiState.connected = 0U;
    gWifiState.has_credentials = 0U;
    gWifiState.rssi = -127;
    gWifiState.reconnect_attempts = 0U;
    gSavedNetworkCount = 0U;
    std::memset(gSavedNetworks, 0, sizeof(gSavedNetworks));
    gWifiState.ssid[0] = '\0';
    gSsid[0] = '\0';
    gPassword[0] = '\0';
    (void)wifi_manager_load_credentials_from_nvs();
    wifi_manager_set_state_internal(WIFI_MANAGER_INITIALIZED);
    system_state_set_wifi_ready(1U);

    if (gReconnectTaskHandle == nullptr) {
        xTaskCreate(wifi_reconnect_task,
                    "wifi_reconnect_task",
                    4096,
                    nullptr,
                    2,
                    &gReconnectTaskHandle);
    }

    ESP_LOGI(TAG, "minimal wifi manager initialized");
    return true;
}

bool wifi_manager_is_initialized(void)
{
    return gWifiState.initialized != 0U;
}

bool wifi_manager_is_connected(void)
{
    return gWifiState.connected != 0U;
}

WiFiManagerState wifi_manager_get_state(void)
{
    return gWifiState.state;
}

WiFiManagerSnapshot wifi_manager_get_snapshot(void)
{
    return gWifiState;
}

void wifi_manager_set_credentials_present(uint8_t present)
{
    gWifiState.has_credentials = present ? 1U : 0U;
}

bool wifi_manager_set_credentials(const char *ssid, const char *password)
{
    if (ssid == nullptr || password == nullptr || ssid[0] == '\0') {
        ESP_LOGW(TAG, "wifi credentials rejected: empty ssid or null input");
        return false;
    }

    uint32_t targetIndex = gSavedNetworkCount;
    for (uint32_t i = 0; i < gSavedNetworkCount; ++i) {
        if (std::strcmp(gSavedNetworks[i].ssid, ssid) == 0) {
            targetIndex = i;
            break;
        }
    }

    if (targetIndex >= WIFI_MANAGER_MAX_SAVED_NETWORKS) {
        targetIndex = WIFI_MANAGER_MAX_SAVED_NETWORKS - 1U;
    }

    std::strncpy(gSavedNetworks[targetIndex].ssid, ssid, sizeof(gSavedNetworks[targetIndex].ssid) - 1U);
    gSavedNetworks[targetIndex].ssid[sizeof(gSavedNetworks[targetIndex].ssid) - 1U] = '\0';
    std::strncpy(gSavedNetworks[targetIndex].password, password, sizeof(gSavedNetworks[targetIndex].password) - 1U);
    gSavedNetworks[targetIndex].password[sizeof(gSavedNetworks[targetIndex].password) - 1U] = '\0';

    if (targetIndex == gSavedNetworkCount && gSavedNetworkCount < WIFI_MANAGER_MAX_SAVED_NETWORKS) {
        ++gSavedNetworkCount;
    }

    wifi_manager_copy_active_credentials(ssid, password);
    wifi_manager_set_credentials_present(1U);
    wifi_manager_set_state_internal(WIFI_MANAGER_CONFIGURING);

    if (!wifi_manager_save_credentials_to_nvs()) {
        ESP_LOGW(TAG, "wifi credentials stored in memory but not persisted");
    }

    ESP_LOGI(TAG, "wifi credentials stored for ssid=%s", gSsid);
    return true;
}

bool wifi_manager_configure_and_connect(const char *ssid, const char *password)
{
    if (!gWifiState.initialized || ssid == nullptr || ssid[0] == '\0') {
        return false;
    }

    gManualConfigActive = true;
    wifi_manager_set_state_internal(WIFI_MANAGER_CONFIGURING);

    esp_err_t stopScanErr = esp_wifi_scan_stop();
    if (stopScanErr != ESP_OK && stopScanErr != ESP_ERR_WIFI_NOT_STARTED &&
        stopScanErr != ESP_ERR_WIFI_STATE) {
        ESP_LOGW(TAG, "esp_wifi_scan_stop during manual config failed: %s",
                 esp_err_to_name(stopScanErr));
    }
    gScanInProgress = false;
    (void)esp_wifi_disconnect();

    char actualPassword[65] = {};
    if (password != nullptr && password[0] != '\0') {
        std::strncpy(actualPassword, password, sizeof(actualPassword) - 1U);
    } else {
        (void)wifi_manager_find_saved_password(ssid, actualPassword, sizeof(actualPassword));
    }

    WiFiScanRecord scanRecords[16] = {};
    const uint32_t scanCount = wifi_manager_scan_networks(scanRecords, 16U);
    bool networkFound = false;
    bool signalTooWeak = false;
    int8_t foundRssi = -127;

    for (uint32_t i = 0; i < scanCount; ++i) {
        if (std::strcmp(scanRecords[i].ssid, ssid) == 0) {
            foundRssi = scanRecords[i].rssi;
            if (scanRecords[i].rssi >= WIFI_MIN_RSSI_THRESHOLD) {
                networkFound = true;
                break;
            }
            signalTooWeak = true;
        }
    }

    if (!networkFound) {
        ESP_LOGW(TAG,
                 "manual wifi config target not usable ssid=%s found_rssi=%d weak=%s",
                 ssid,
                 (int)foundRssi,
                 signalTooWeak ? "yes" : "no");
        gManualConfigActive = false;
        wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
        return false;
    }

    const bool connected = wifi_manager_connect_with_credentials(ssid, actualPassword);
    if (connected) {
        if (!wifi_manager_set_credentials(ssid, actualPassword)) {
            ESP_LOGW(TAG, "manual wifi connected but credential persistence failed ssid=%s", ssid);
        }
        gManualConfigActive = false;
        wifi_manager_set_state_internal(WIFI_MANAGER_CONNECTED);
        return true;
    }

    gManualConfigActive = false;
    wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
    return false;
}

bool wifi_manager_connect(void)
{
    if (!gWifiState.initialized) {
        ESP_LOGW(TAG, "wifi connect requested before init");
        return false;
    }

    if (!gWifiState.has_credentials) {
        wifi_manager_set_state_internal(WIFI_MANAGER_CONFIGURING);
        ESP_LOGW(TAG, "wifi connect requested without credentials");
        return false;
    }

    if (gWifiState.connected) {
        return true;
    }

    WiFiScanRecord scanRecords[16] = {};
    const uint32_t scanCount = wifi_manager_scan_networks(scanRecords, 16U);

    if (scanCount > 0U && gSavedNetworkCount > 0U) {
        struct CandidateNetwork {
            uint32_t saved_index;
            int8_t rssi;
        };

        CandidateNetwork candidates[WIFI_MANAGER_MAX_SAVED_NETWORKS] = {};
        uint32_t candidateCount = 0U;

        for (uint32_t savedIndex = 0; savedIndex < gSavedNetworkCount; ++savedIndex) {
            for (uint32_t scanIndex = 0; scanIndex < scanCount; ++scanIndex) {
                if (std::strcmp(gSavedNetworks[savedIndex].ssid, scanRecords[scanIndex].ssid) == 0 &&
                    scanRecords[scanIndex].rssi >= WIFI_MIN_RSSI_THRESHOLD) {
                    candidates[candidateCount].saved_index = savedIndex;
                    candidates[candidateCount].rssi = scanRecords[scanIndex].rssi;
                    ++candidateCount;
                    break;
                }
            }
        }

        for (uint32_t i = 0; i + 1U < candidateCount; ++i) {
            for (uint32_t j = 0; j + 1U < candidateCount - i; ++j) {
                if (candidates[j].rssi < candidates[j + 1U].rssi) {
                    const CandidateNetwork temp = candidates[j];
                    candidates[j] = candidates[j + 1U];
                    candidates[j + 1U] = temp;
                }
            }
        }

        for (uint32_t i = 0; i < candidateCount; ++i) {
            const uint32_t savedIndex = candidates[i].saved_index;
            ESP_LOGI(TAG,
                     "trying saved network match ssid=%s rssi=%d",
                     gSavedNetworks[savedIndex].ssid,
                     (int)candidates[i].rssi);
            if (wifi_manager_connect_with_credentials(gSavedNetworks[savedIndex].ssid,
                                                      gSavedNetworks[savedIndex].password)) {
                return true;
            }
        }
    }

    for (uint32_t i = 0; i < gSavedNetworkCount; ++i) {
        if (std::strcmp(gSavedNetworks[i].ssid, gSsid) == 0) {
            return wifi_manager_connect_with_credentials(gSavedNetworks[i].ssid,
                                                         gSavedNetworks[i].password);
        }
    }

    if (gSsid[0] != '\0') {
        return wifi_manager_connect_with_credentials(gSsid, gPassword);
    }

    return false;
}

void wifi_manager_disconnect(void)
{
    if (gWifiDriverInitialized) {
        esp_wifi_disconnect();
    }
    gWifiState.connected = 0U;
    gWifiState.rssi = -127;
    gWifiState.ip[0] = '\0';
    if (gWifiState.has_credentials != 0U) {
        gWifiState.reconnect_attempts = 0U;
    }
    wifi_manager_set_state_internal(WIFI_MANAGER_DISCONNECTED);
    WiFiEvent(TASK_WIFI_EVENT_STA_STOP);
    ESP_LOGI(TAG, "wifi disconnected");
}

const char *wifi_manager_get_ssid(void)
{
    return gSsid;
}

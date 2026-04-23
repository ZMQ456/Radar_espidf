#include "wifi_manager.h"

#include <cstring>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
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
static esp_netif_t *gStaNetif = nullptr;
static EventGroupHandle_t gWifiEventGroup = nullptr;
static bool gEspNetifInitialized = false;
static bool gWifiDriverInitialized = false;
static bool gEventHandlersRegistered = false;
static TaskHandle_t gReconnectTaskHandle = nullptr;

static constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
static constexpr EventBits_t WIFI_FAIL_BIT = BIT1;
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
static constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 5000;
static const char *WIFI_NVS_NAMESPACE = "wifi_cfg";
static const char *WIFI_NVS_KEY_SSID = "ssid";
static const char *WIFI_NVS_KEY_PASS = "pass";

static void wifi_manager_set_state_internal(WiFiManagerState state)
{
    gWifiState.state = state;
    gWifiState.last_state_change_ms = radar_now_ms();

    switch (state) {
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

        if (gWifiEventGroup != nullptr) {
            xEventGroupSetBits(gWifiEventGroup, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG, "wifi got ip, ssid=%s rssi=%d", gSsid, static_cast<int>(gWifiState.rssi));
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
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs open for read failed: %s", esp_err_to_name(err));
        return false;
    }

    size_t ssidLen = sizeof(gSsid);
    size_t passLen = sizeof(gPassword);
    err = nvs_get_str(handle, WIFI_NVS_KEY_SSID, gSsid, &ssidLen);
    if (err != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    err = nvs_get_str(handle, WIFI_NVS_KEY_PASS, gPassword, &passLen);
    if (err != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    std::strncpy(gWifiState.ssid, gSsid, sizeof(gWifiState.ssid) - 1U);
    gWifiState.ssid[sizeof(gWifiState.ssid) - 1U] = '\0';
    gWifiState.has_credentials = 1U;
    nvs_close(handle);
    ESP_LOGI(TAG, "loaded wifi credentials from nvs for ssid=%s", gSsid);
    return true;
}

static bool wifi_manager_save_credentials_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open for write failed: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_str(handle, WIFI_NVS_KEY_SSID, gSsid);
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

    ESP_LOGI(TAG, "saved wifi credentials to nvs for ssid=%s", gSsid);
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
    wifi_manager_disconnect();
    ESP_LOGI(TAG, "wifi credentials cleared");
    return true;
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

    std::strncpy(gSsid, ssid, sizeof(gSsid) - 1U);
    gSsid[sizeof(gSsid) - 1U] = '\0';
    std::strncpy(gPassword, password, sizeof(gPassword) - 1U);
    gPassword[sizeof(gPassword) - 1U] = '\0';
    std::strncpy(gWifiState.ssid, gSsid, sizeof(gWifiState.ssid) - 1U);
    gWifiState.ssid[sizeof(gWifiState.ssid) - 1U] = '\0';
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
    if (!wifi_manager_set_credentials(ssid, password)) {
        return false;
    }
    return wifi_manager_connect();
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
    wifiConfig.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifiConfig.sta.pmf_cfg.capable = true;
    wifiConfig.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));

    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        wifi_manager_set_state_internal(WIFI_MANAGER_ERROR);
        return false;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
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

void wifi_manager_disconnect(void)
{
    if (gWifiDriverInitialized) {
        esp_wifi_disconnect();
    }
    gWifiState.connected = 0U;
    gWifiState.rssi = -127;
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

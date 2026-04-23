#include "device_identity.h"

#include "app_config.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "system_state.h"

static const char *TAG = "device_identity";
static const char *DEVICE_IDENTITY_NAMESPACE = "device_cfg";
static const char *DEVICE_IDENTITY_KEY_ID = "deviceId";
static const char *DEVICE_IDENTITY_KEY_SN = "deviceSn";

static DeviceIdentity gDeviceIdentity = {
    APP_DEVICE_ID_DEFAULT,
    APP_DEVICE_SN_DEFAULT,
};
static bool gDeviceIdentityInitialized = false;

static bool device_identity_ensure_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

static bool device_identity_persist(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_IDENTITY_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open failed: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_u16(handle, DEVICE_IDENTITY_KEY_ID, gDeviceIdentity.device_id);
    if (err == ESP_OK) {
        err = nvs_set_u64(handle, DEVICE_IDENTITY_KEY_SN, gDeviceIdentity.device_sn);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs persist failed: %s", esp_err_to_name(err));
        return false;
    }

    system_state_set_device_id(gDeviceIdentity.device_id);
    system_state_set_device_sn(gDeviceIdentity.device_sn);
    return true;
}

bool device_identity_init(void)
{
    if (gDeviceIdentityInitialized) {
        system_state_set_device_id(gDeviceIdentity.device_id);
        system_state_set_device_sn(gDeviceIdentity.device_sn);
        return true;
    }

    if (!device_identity_ensure_nvs()) {
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_IDENTITY_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open failed: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t storedDeviceId = APP_DEVICE_ID_DEFAULT;
    uint64_t storedDeviceSn = APP_DEVICE_SN_DEFAULT;

    err = nvs_get_u16(handle, DEVICE_IDENTITY_KEY_ID, &storedDeviceId);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = nvs_set_u16(handle, DEVICE_IDENTITY_KEY_ID, storedDeviceId);
    }
    if (err == ESP_OK) {
        err = nvs_get_u64(handle, DEVICE_IDENTITY_KEY_SN, &storedDeviceSn);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            storedDeviceSn = APP_DEVICE_SN_DEFAULT;
            err = nvs_set_u64(handle, DEVICE_IDENTITY_KEY_SN, storedDeviceSn);
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs load failed: %s", esp_err_to_name(err));
        return false;
    }

    gDeviceIdentity.device_id = storedDeviceId;
    gDeviceIdentity.device_sn = storedDeviceSn;
    gDeviceIdentityInitialized = true;

    system_state_set_device_id(gDeviceIdentity.device_id);
    system_state_set_device_sn(gDeviceIdentity.device_sn);

    ESP_LOGI(TAG,
             "loaded device identity device_id=%u device_sn=%llu",
             (unsigned)gDeviceIdentity.device_id,
             (unsigned long long)gDeviceIdentity.device_sn);
    return true;
}

DeviceIdentity device_identity_get(void)
{
    return gDeviceIdentity;
}

bool device_identity_set_device_id(uint16_t device_id)
{
    gDeviceIdentity.device_id = device_id;
    return device_identity_persist();
}

bool device_identity_set_device_sn(uint64_t device_sn)
{
    gDeviceIdentity.device_sn = device_sn;
    return device_identity_persist();
}

bool device_identity_reset_defaults(void)
{
    gDeviceIdentity.device_id = APP_DEVICE_ID_DEFAULT;
    gDeviceIdentity.device_sn = APP_DEVICE_SN_DEFAULT;
    return device_identity_persist();
}

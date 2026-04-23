#include "ble_manager.h"

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "radar_manager.h"
#include "sdkconfig.h"
#include "system_state.h"

#if CONFIG_BT_ENABLED
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#endif

static const char *TAG = "ble_manager";
static BleManagerSnapshot gBleState = {};
static char gReceivedText[512] = {};
static bool gHasPendingText = false;

#if CONFIG_BT_ENABLED
static constexpr uint16_t BLE_APP_ID = 0x55AA;
static constexpr uint16_t BLE_SERVICE_UUID = 0xA8C1;
static constexpr uint16_t BLE_CHARACTERISTIC_UUID = 0xBEB5;
static constexpr uint16_t BLE_DESCRIPTOR_UUID = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static constexpr size_t BLE_MANUFACTURER_DATA_LEN = 9U;

static bool gBleControllerReady = false;
static bool gGattRegistered = false;
static bool gAdvDataConfigured = false;
static bool gServiceCreated = false;
static esp_gatt_if_t gGattIf = ESP_GATT_IF_NONE;
static uint16_t gConnectionId = 0U;
static uint16_t gServiceHandle = 0U;
static uint16_t gCharacteristicHandle = 0U;
static uint16_t gDescriptorHandle = 0U;
static uint16_t gAttributeValueLength = 0U;
static uint8_t gManufacturerData[BLE_MANUFACTURER_DATA_LEN] = {};
static uint8_t gServiceUuidBytes[2] = {
    static_cast<uint8_t>(BLE_SERVICE_UUID & 0xFFU),
    static_cast<uint8_t>((BLE_SERVICE_UUID >> 8) & 0xFFU),
};
static uint8_t gAttributeValue[256] = {};

static esp_ble_adv_params_t gAdvParams = {};

static void ble_prepare_advertising_payload(void)
{
    const uint32_t deviceHash = radar_manager_generate_device_hash();

    gManufacturerData[0] = 0xFF;
    gManufacturerData[1] = 0xFF;
    gManufacturerData[2] = 'R';
    gManufacturerData[3] = 0x01;
    gManufacturerData[4] = 0x00;
    gManufacturerData[5] = static_cast<uint8_t>((deviceHash >> 24) & 0xFFU);
    gManufacturerData[6] = static_cast<uint8_t>((deviceHash >> 16) & 0xFFU);
    gManufacturerData[7] = static_cast<uint8_t>((deviceHash >> 8) & 0xFFU);
    gManufacturerData[8] = static_cast<uint8_t>(deviceHash & 0xFFU);
}

static void ble_start_advertising_if_ready(void)
{
    if (!gAdvDataConfigured || !gServiceCreated) {
        return;
    }

    esp_err_t err = esp_ble_gap_start_advertising(&gAdvParams);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "BLE advertising requested name=%s", gBleState.device_name);
}

static void ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    (void)param;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        gAdvDataConfigured = true;
        ble_start_advertising_if_ready();
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        gBleState.advertising = 1U;
        ESP_LOGI(TAG, "BLE advertising started name=%s", gBleState.device_name);
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        gBleState.advertising = 0U;
        ESP_LOGI(TAG, "BLE advertising stopped");
        break;
    default:
        break;
    }
}

static void ble_gatts_callback(esp_gatts_cb_event_t event,
                               esp_gatt_if_t gatts_if,
                               esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        gGattIf = gatts_if;

        esp_ble_gap_set_device_name(gBleState.device_name);

        ble_prepare_advertising_payload();

        esp_ble_adv_data_t advData = {};
        advData.set_scan_rsp = false;
        advData.include_name = true;
        advData.include_txpower = false;
        advData.min_interval = 0x20;
        advData.max_interval = 0x40;
        advData.appearance = 0x00;
        advData.manufacturer_len = BLE_MANUFACTURER_DATA_LEN;
        advData.p_manufacturer_data = gManufacturerData;
        advData.service_data_len = 0;
        advData.p_service_data = nullptr;
        advData.service_uuid_len = sizeof(gServiceUuidBytes);
        advData.p_service_uuid = gServiceUuidBytes;
        advData.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
        esp_ble_gap_config_adv_data(&advData);

        esp_gatt_srvc_id_t serviceId = {};
        serviceId.is_primary = true;
        serviceId.id.inst_id = 0x00;
        serviceId.id.uuid.len = ESP_UUID_LEN_16;
        serviceId.id.uuid.uuid.uuid16 = BLE_SERVICE_UUID;
        esp_ble_gatts_create_service(gatts_if, &serviceId, 6U);
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        gServiceHandle = param->create.service_handle;
        gServiceCreated = true;
        esp_ble_gatts_start_service(gServiceHandle);

        esp_bt_uuid_t characteristicUuid = {};
        characteristicUuid.len = ESP_UUID_LEN_16;
        characteristicUuid.uuid.uuid16 = BLE_CHARACTERISTIC_UUID;
        const esp_gatt_char_prop_t property =
            ESP_GATT_CHAR_PROP_BIT_READ |
            ESP_GATT_CHAR_PROP_BIT_WRITE |
            ESP_GATT_CHAR_PROP_BIT_NOTIFY;

        esp_ble_gatts_add_char(gServiceHandle,
                               &characteristicUuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               property,
                               nullptr,
                               nullptr);
        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        gCharacteristicHandle = param->add_char.attr_handle;

        esp_bt_uuid_t descriptorUuid = {};
        descriptorUuid.len = ESP_UUID_LEN_16;
        descriptorUuid.uuid.uuid16 = BLE_DESCRIPTOR_UUID;
        esp_ble_gatts_add_char_descr(gServiceHandle,
                                     &descriptorUuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     nullptr,
                                     nullptr);

        ble_start_advertising_if_ready();
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gDescriptorHandle = param->add_char_descr.attr_handle;
        break;

    case ESP_GATTS_CONNECT_EVT:
        gBleState.connected = 1U;
        gConnectionId = param->connect.conn_id;
        ESP_LOGI(TAG, "BLE client connected conn_id=%u", gConnectionId);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        gBleState.connected = 0U;
        gBleState.notify_enabled = 0U;
        gBleState.advertising = 0U;
        ESP_LOGI(TAG, "BLE client disconnected");
        ble_start_advertising_if_ready();
        break;

    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep) {
            if (param->write.handle == gDescriptorHandle && param->write.len >= 2U) {
                const uint16_t cccdValue =
                    static_cast<uint16_t>(param->write.value[1] << 8) |
                    static_cast<uint16_t>(param->write.value[0]);
                gBleState.notify_enabled = (cccdValue & 0x0001U) ? 1U : 0U;
                ESP_LOGI(TAG, "BLE notify %s",
                         gBleState.notify_enabled ? "enabled" : "disabled");
            } else if (param->write.handle == gCharacteristicHandle) {
                const size_t copyLen =
                    param->write.len < (sizeof(gAttributeValue) - 1U)
                        ? param->write.len
                        : (sizeof(gAttributeValue) - 1U);
                std::memcpy(gAttributeValue, param->write.value, copyLen);
                gAttributeValue[copyLen] = '\0';
                gAttributeValueLength = static_cast<uint16_t>(copyLen);
                std::memcpy(gReceivedText, gAttributeValue, copyLen + 1U);
                gHasPendingText = true;
                ESP_LOGI(TAG, "BLE write received: %s", reinterpret_cast<const char *>(gAttributeValue));
            }
        }
        break;

    default:
        break;
    }
}
#endif

bool ble_manager_init(void)
{
    if (gBleState.initialized != 0U) {
        return true;
    }

    char deviceName[32] = {};
    if (!radar_manager_get_device_name(deviceName, sizeof(deviceName))) {
        std::strncpy(deviceName, "Radar_Unknown", sizeof(deviceName) - 1U);
        deviceName[sizeof(deviceName) - 1U] = '\0';
    }

    std::strncpy(gBleState.device_name, deviceName, sizeof(gBleState.device_name) - 1U);
    gBleState.device_name[sizeof(gBleState.device_name) - 1U] = '\0';

#if !CONFIG_BT_ENABLED
    gBleState.initialized = 1U;
    gBleState.advertising = 0U;
    gBleState.connected = 0U;
    gBleState.notify_enabled = 0U;
    system_state_set_ble_ready(1U);
    ESP_LOGW(TAG, "BLE is not enabled in sdkconfig, skeleton loaded with name=%s", gBleState.device_name);
    return true;
#else
    if (!gBleControllerReady) {
        esp_bt_controller_config_t btCfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_bt_controller_init(&btCfg);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_bluedroid_init();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_bluedroid_enable();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(err));
            return false;
        }

        std::memset(&gAdvParams, 0, sizeof(gAdvParams));
        gAdvParams.adv_int_min = 0x20;
        gAdvParams.adv_int_max = 0x40;
        gAdvParams.adv_type = ADV_TYPE_IND;
        gAdvParams.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
        gAdvParams.channel_map = ADV_CHNL_ALL;
        gAdvParams.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

        esp_err_t errGap = esp_ble_gap_register_callback(ble_gap_callback);
        if (errGap != ESP_OK) {
            ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %s", esp_err_to_name(errGap));
            return false;
        }

        esp_err_t errGatt = esp_ble_gatts_register_callback(ble_gatts_callback);
        if (errGatt != ESP_OK) {
            ESP_LOGE(TAG, "esp_ble_gatts_register_callback failed: %s", esp_err_to_name(errGatt));
            return false;
        }

        gBleControllerReady = true;
    }

    if (!gGattRegistered) {
        esp_err_t err = esp_ble_gatts_app_register(BLE_APP_ID);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_ble_gatts_app_register failed: %s", esp_err_to_name(err));
            return false;
        }

        esp_ble_gatt_set_local_mtu(256);
        gGattRegistered = true;
    }

    gBleState.initialized = 1U;
    gBleState.advertising = 0U;
    gBleState.connected = 0U;
    gBleState.notify_enabled = 0U;
    system_state_set_ble_ready(1U);
    ESP_LOGI(TAG, "BLE manager initialized name=%s", gBleState.device_name);
    return true;
#endif
}

bool ble_manager_is_initialized(void)
{
    return gBleState.initialized != 0U;
}

bool ble_manager_is_client_connected(void)
{
    return gBleState.connected != 0U;
}

bool ble_manager_send_text(const char *payload)
{
    if (payload == nullptr || payload[0] == '\0') {
        return false;
    }

#if !CONFIG_BT_ENABLED
    ESP_LOGW(TAG, "BLE send skipped because BT is disabled: %s", payload);
    return false;
#else
    const size_t payloadLen = std::strlen(payload);
    const size_t copyLen = payloadLen < (sizeof(gAttributeValue) - 1U)
                               ? payloadLen
                               : (sizeof(gAttributeValue) - 1U);
    std::memcpy(gAttributeValue, payload, copyLen);
    gAttributeValue[copyLen] = '\0';
    gAttributeValueLength = static_cast<uint16_t>(copyLen);

    if (gCharacteristicHandle != 0U) {
        esp_ble_gatts_set_attr_value(gCharacteristicHandle,
                                     gAttributeValueLength,
                                     gAttributeValue);
    }

    if (!gBleState.connected || !gBleState.notify_enabled ||
        gGattIf == ESP_GATT_IF_NONE || gCharacteristicHandle == 0U) {
        ESP_LOGI(TAG, "BLE payload staged without notify: %s", payload);
        return false;
    }

    const esp_err_t err = esp_ble_gatts_send_indicate(gGattIf,
                                                      gConnectionId,
                                                      gCharacteristicHandle,
                                                      gAttributeValueLength,
                                                      gAttributeValue,
                                                      false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gatts_send_indicate failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "BLE notify sent: %s", payload);
    return true;
#endif
}

bool ble_manager_consume_received_text(char *buffer, uint32_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0U || !gHasPendingText) {
        return false;
    }

    const size_t copyLen = std::strlen(gReceivedText) < (buffer_size - 1U)
                               ? std::strlen(gReceivedText)
                               : (buffer_size - 1U);
    std::memcpy(buffer, gReceivedText, copyLen);
    buffer[copyLen] = '\0';
    gReceivedText[0] = '\0';
    gHasPendingText = false;
    return true;
}

BleManagerSnapshot ble_manager_get_snapshot(void)
{
    return gBleState;
}

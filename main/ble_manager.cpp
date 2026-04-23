#include "ble_manager.h"

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "radar_manager.h"
#include "radar_platform.h"
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
static char gReceiveBuffer[512] = {};
static size_t gReceiveLength = 0U;
static uint64_t gLastReceiveTimeMs = 0U;

static constexpr size_t BLE_JSON_PACKET_SIZE = 20U;
static constexpr uint32_t BLE_RECEIVE_TIMEOUT_MS = 5000U;

static bool ble_try_extract_complete_payload(void)
{
    if (gReceiveLength == 0U) {
        return false;
    }

    char *openBrace = static_cast<char *>(std::memchr(gReceiveBuffer, '{', gReceiveLength));
    if (openBrace == nullptr) {
        return false;
    }

    const size_t openIndex = static_cast<size_t>(openBrace - gReceiveBuffer);
    int depth = 0;
    for (size_t i = openIndex; i < gReceiveLength; ++i) {
        if (gReceiveBuffer[i] == '{') {
            ++depth;
        } else if (gReceiveBuffer[i] == '}') {
            --depth;
            if (depth == 0) {
                const size_t payloadLen = i - openIndex + 1U;
                const size_t copyLen = payloadLen < (sizeof(gReceivedText) - 1U)
                                           ? payloadLen
                                           : (sizeof(gReceivedText) - 1U);
                std::memcpy(gReceivedText, gReceiveBuffer + openIndex, copyLen);
                gReceivedText[copyLen] = '\0';
                gHasPendingText = true;

                const size_t remaining = gReceiveLength - (i + 1U);
                if (remaining > 0U) {
                    std::memmove(gReceiveBuffer, gReceiveBuffer + i + 1U, remaining);
                }
                gReceiveLength = remaining;
                gReceiveBuffer[gReceiveLength] = '\0';
                return true;
            }
        }
    }

    if (openIndex > 0U) {
        const size_t remaining = gReceiveLength - openIndex;
        std::memmove(gReceiveBuffer, gReceiveBuffer + openIndex, remaining);
        gReceiveLength = remaining;
        gReceiveBuffer[gReceiveLength] = '\0';
    }

    return false;
}

static void ble_append_received_fragment(const uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0U) {
        return;
    }

    const size_t freeLen = (sizeof(gReceiveBuffer) - 1U) - gReceiveLength;
    const size_t copyLen = length < freeLen ? length : freeLen;
    if (copyLen > 0U) {
        std::memcpy(gReceiveBuffer + gReceiveLength, data, copyLen);
        gReceiveLength += copyLen;
        gReceiveBuffer[gReceiveLength] = '\0';
        gLastReceiveTimeMs = radar_now_ms();
    }

    if (copyLen < length) {
        ESP_LOGW(TAG, "BLE receive buffer full, dropped %u bytes", (unsigned)(length - copyLen));
    }

    (void)ble_try_extract_complete_payload();
}

#if CONFIG_BT_ENABLED
static constexpr uint16_t BLE_APP_ID = 0x55AA;
static constexpr uint16_t BLE_DESCRIPTOR_UUID = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static constexpr size_t BLE_MANUFACTURER_DATA_LEN = 9U;

static bool gBleControllerReady = false;
static bool gGattRegistered = false;
static bool gAdvDataConfigured = false;
static bool gScanRspDataConfigured = false;
static bool gServiceCreated = false;
static esp_gatt_if_t gGattIf = ESP_GATT_IF_NONE;
static uint16_t gConnectionId = 0U;
static uint16_t gServiceHandle = 0U;
static uint16_t gCharacteristicHandle = 0U;
static uint16_t gDescriptorHandle = 0U;
static uint16_t gAttributeValueLength = 0U;
static uint8_t gManufacturerData[BLE_MANUFACTURER_DATA_LEN] = {};
static uint8_t gServiceUuidBytes[ESP_UUID_LEN_128] = {
    0x1A, 0x2F, 0x4E, 0x6A, 0x8B, 0x7C, 0x5E, 0x8D,
    0x9D, 0x4A, 0x5D, 0x3D, 0xC0, 0xE5, 0xC1, 0xA8,
};
static uint8_t gCharacteristicUuidBytes[ESP_UUID_LEN_128] = {
    0xA8, 0x26, 0x1B, 0x36, 0x07, 0xEA, 0xF5, 0xB7,
    0x88, 0x46, 0xE1, 0x36, 0x3E, 0x48, 0xB5, 0xBE,
};
static uint8_t gAttributeValue[256] = {};

static esp_ble_adv_params_t gAdvParams = {};

static bool ble_update_device_name_from_identity(void)
{
    char deviceName[32] = {};
    if (!radar_manager_get_device_name(deviceName, sizeof(deviceName))) {
        std::strncpy(deviceName, "Radar_Unknown", sizeof(deviceName) - 1U);
        deviceName[sizeof(deviceName) - 1U] = '\0';
    }

    std::strncpy(gBleState.device_name, deviceName, sizeof(gBleState.device_name) - 1U);
    gBleState.device_name[sizeof(gBleState.device_name) - 1U] = '\0';
    return true;
}

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
    if (!gAdvDataConfigured || !gScanRspDataConfigured || !gServiceCreated || gBleState.connected != 0U) {
        return;
    }

    esp_err_t err = esp_ble_gap_start_advertising(&gAdvParams);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "BLE advertising requested name=%s", gBleState.device_name);
}

static void ble_configure_advertising_payloads(void)
{
    ble_prepare_advertising_payload();

    esp_ble_adv_data_t advData = {};
    advData.set_scan_rsp = false;
    advData.include_name = false;
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

    esp_ble_adv_data_t scanRspData = {};
    scanRspData.set_scan_rsp = true;
    scanRspData.include_name = true;
    scanRspData.include_txpower = false;
    scanRspData.min_interval = 0x20;
    scanRspData.max_interval = 0x40;
    scanRspData.appearance = 0x00;
    scanRspData.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
    esp_ble_gap_config_adv_data(&scanRspData);
}

static void ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    (void)param;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        gAdvDataConfigured = true;
        ble_start_advertising_if_ready();
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        gScanRspDataConfigured = true;
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
        ble_configure_advertising_payloads();

        esp_gatt_srvc_id_t serviceId = {};
        serviceId.is_primary = true;
        serviceId.id.inst_id = 0x00;
        serviceId.id.uuid.len = ESP_UUID_LEN_128;
        std::memcpy(serviceId.id.uuid.uuid.uuid128,
                    gServiceUuidBytes,
                    sizeof(serviceId.id.uuid.uuid.uuid128));
        esp_ble_gatts_create_service(gatts_if, &serviceId, 6U);
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        gServiceHandle = param->create.service_handle;
        gServiceCreated = true;
        esp_ble_gatts_start_service(gServiceHandle);

        esp_bt_uuid_t characteristicUuid = {};
        characteristicUuid.len = ESP_UUID_LEN_128;
        std::memcpy(characteristicUuid.uuid.uuid128,
                    gCharacteristicUuidBytes,
                    sizeof(characteristicUuid.uuid.uuid128));
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
                ble_append_received_fragment(param->write.value, param->write.len);
                ESP_LOGI(TAG, "BLE write received: %s", reinterpret_cast<const char *>(gAttributeValue));
            }

            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if,
                                            param->write.conn_id,
                                            param->write.trans_id,
                                            ESP_GATT_OK,
                                            nullptr);
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

    ble_update_device_name_from_identity();

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

bool ble_manager_refresh_advertising_data(void)
{
#if !CONFIG_BT_ENABLED
    ble_update_device_name_from_identity();
    return true;
#else
    if (gBleState.initialized == 0U || !gBleControllerReady) {
        ble_update_device_name_from_identity();
        return true;
    }

    ble_update_device_name_from_identity();
    esp_ble_gap_set_device_name(gBleState.device_name);

    if (gBleState.advertising != 0U) {
        const esp_err_t stopErr = esp_ble_gap_stop_advertising();
        if (stopErr != ESP_OK && stopErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "esp_ble_gap_stop_advertising failed before refresh: %s",
                     esp_err_to_name(stopErr));
        }
    }

    gAdvDataConfigured = false;
    gScanRspDataConfigured = false;
    ble_configure_advertising_payloads();
    ESP_LOGI(TAG, "BLE advertising data refreshed name=%s", gBleState.device_name);
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
    const size_t copyLen =
        payloadLen < (sizeof(gAttributeValue) - 1U) ? payloadLen : (sizeof(gAttributeValue) - 1U);
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

    bool allChunksSent = true;
    for (size_t offset = 0U; offset < payloadLen; offset += BLE_JSON_PACKET_SIZE) {
        const size_t remaining = payloadLen - offset;
        const size_t chunkLen =
            remaining < BLE_JSON_PACKET_SIZE ? remaining : BLE_JSON_PACKET_SIZE;

        const esp_err_t err =
            esp_ble_gatts_send_indicate(gGattIf,
                                        gConnectionId,
                                        gCharacteristicHandle,
                                        static_cast<uint16_t>(chunkLen),
                                        reinterpret_cast<uint8_t *>(const_cast<char *>(payload + offset)),
                                        false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ble_gatts_send_indicate failed: %s", esp_err_to_name(err));
            allChunksSent = false;
            break;
        }

        if ((offset + chunkLen) < payloadLen) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    ESP_LOGI(TAG, "BLE notify %s: %s", allChunksSent ? "sent" : "partial", payload);
    return allChunksSent;
#endif
}

bool ble_manager_consume_received_text(char *buffer, uint32_t buffer_size)
{
    if (!gHasPendingText && gReceiveLength > 0U && gLastReceiveTimeMs != 0U &&
        (radar_now_ms() - gLastReceiveTimeMs) > BLE_RECEIVE_TIMEOUT_MS) {
        const size_t copyLen = gReceiveLength < (sizeof(gReceivedText) - 1U)
                                   ? gReceiveLength
                                   : (sizeof(gReceivedText) - 1U);
        std::memcpy(gReceivedText, gReceiveBuffer, copyLen);
        gReceivedText[copyLen] = '\0';
        gReceiveBuffer[0] = '\0';
        gReceiveLength = 0U;
        gHasPendingText = true;
        ESP_LOGW(TAG, "BLE receive timeout, forwarding partial payload");
    }

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

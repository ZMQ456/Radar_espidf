#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t initialized;
    uint8_t advertising;
    uint8_t connected;
    uint8_t notify_enabled;
    char device_name[32];
} BleManagerSnapshot;

bool ble_manager_init(void);
bool ble_manager_is_initialized(void);
bool ble_manager_is_client_connected(void);
bool ble_manager_send_text(const char *payload);
bool ble_manager_consume_received_text(char *buffer, uint32_t buffer_size);
BleManagerSnapshot ble_manager_get_snapshot(void);

#ifdef __cplusplus
}
#endif

#endif

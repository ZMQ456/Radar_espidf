#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIFI_MANAGER_IDLE = 0,
    WIFI_MANAGER_INITIALIZED,
    WIFI_MANAGER_CONNECTING,
    WIFI_MANAGER_CONNECTED,
    WIFI_MANAGER_DISCONNECTED,
    WIFI_MANAGER_CONFIGURING,
    WIFI_MANAGER_ERROR
} WiFiManagerState;

typedef struct {
    WiFiManagerState state;
    uint8_t initialized;
    uint8_t connected;
    uint8_t has_credentials;
    int8_t rssi;
    uint32_t reconnect_attempts;
    uint64_t last_state_change_ms;
    char ssid[33];
} WiFiManagerSnapshot;

bool wifi_manager_init(void);
bool wifi_manager_is_initialized(void);
bool wifi_manager_is_connected(void);
WiFiManagerState wifi_manager_get_state(void);
WiFiManagerSnapshot wifi_manager_get_snapshot(void);
void wifi_manager_set_credentials_present(uint8_t present);
bool wifi_manager_set_credentials(const char *ssid, const char *password);
bool wifi_manager_configure_and_connect(const char *ssid, const char *password);
bool wifi_manager_connect(void);
void wifi_manager_disconnect(void);
bool wifi_manager_clear_credentials(void);
const char *wifi_manager_get_ssid(void);

#ifdef __cplusplus
}
#endif

#endif

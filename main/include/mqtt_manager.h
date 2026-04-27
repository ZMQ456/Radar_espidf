#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdint.h>

#include "radar_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MQTT_MANAGER_IDLE = 0,
    MQTT_MANAGER_INITIALIZED,
    MQTT_MANAGER_CONNECTING,
    MQTT_MANAGER_CONNECTED,
    MQTT_MANAGER_DISCONNECTED,
    MQTT_MANAGER_ERROR
} MqttManagerState;

typedef struct {
    MqttManagerState state;
    uint8_t initialized;
    uint8_t connected;
    uint8_t configured;
    uint32_t reconnect_attempts;
    uint64_t last_state_change_ms;
    char uri[128];
    char client_id[64];
    char test_topic[96];
} MqttManagerSnapshot;

bool mqtt_manager_init(void);
bool mqtt_manager_configure(const char *uri, const char *client_id);
bool mqtt_manager_is_initialized(void);
bool mqtt_manager_is_connected(void);
MqttManagerSnapshot mqtt_manager_get_snapshot(void);
bool mqtt_manager_refresh_identity(void);
bool mqtt_manager_publish_text(const char *topic, const char *payload, int qos, int retain);
bool mqtt_manager_publish_online(void);
bool mqtt_manager_publish_radar_snapshot(const RadarReportSnapshot *snapshot);
bool mqtt_manager_publish_sleep_snapshot(const RadarReportSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif

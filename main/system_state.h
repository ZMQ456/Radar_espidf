#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYSTEM_PHASE_BOOT = 0,
    SYSTEM_PHASE_RADAR_READY,
    SYSTEM_PHASE_TASKS_READY,
    SYSTEM_PHASE_RUNNING,
    SYSTEM_PHASE_ERROR
} SystemPhase;

typedef enum {
    NETWORK_STATUS_INITIAL = 0,
    NETWORK_STATUS_DISCONNECTED,
    NETWORK_STATUS_CONNECTING,
    NETWORK_STATUS_CONNECTED
} NetworkStatus;

typedef struct {
    SystemPhase phase;
    NetworkStatus network_status;
    uint8_t radar_ready;
    uint8_t tasks_ready;
    uint8_t ble_ready;
    uint8_t wifi_ready;
    uint8_t mqtt_ready;
    uint8_t error;
    uint16_t current_device_id;
    uint64_t device_sn;
    uint64_t boot_time_ms;
    uint64_t uptime_ms;
} SystemStateSnapshot;

void system_state_init(void);
void system_state_set_phase(SystemPhase phase);
void system_state_set_radar_ready(uint8_t ready);
void system_state_set_tasks_ready(uint8_t ready);
void system_state_set_ble_ready(uint8_t ready);
void system_state_set_wifi_ready(uint8_t ready);
void system_state_set_mqtt_ready(uint8_t ready);
void system_state_set_error(uint8_t error);
void system_state_set_network_status(NetworkStatus status);
void system_state_set_device_id(uint16_t device_id);
void system_state_set_device_sn(uint64_t device_sn);
SystemStateSnapshot system_state_get_snapshot(void);

#ifdef __cplusplus
}
#endif

#endif

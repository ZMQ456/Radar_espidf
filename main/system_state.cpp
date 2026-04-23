#include "system_state.h"

#include "app_config.h"
#include "radar_platform.h"

static SystemStateSnapshot gSystemState = {};

void system_state_init(void)
{
    gSystemState.phase = SYSTEM_PHASE_BOOT;
    gSystemState.network_status = NETWORK_STATUS_INITIAL;
    gSystemState.radar_ready = 0U;
    gSystemState.tasks_ready = 0U;
    gSystemState.ble_ready = 0U;
    gSystemState.wifi_ready = 0U;
    gSystemState.mqtt_ready = 0U;
    gSystemState.error = 0U;
    gSystemState.current_device_id = APP_DEVICE_ID_DEFAULT;
    gSystemState.device_sn = APP_DEVICE_SN_DEFAULT;
    gSystemState.boot_time_ms = radar_now_ms();
    gSystemState.uptime_ms = 0U;
}

void system_state_set_phase(SystemPhase phase)
{
    gSystemState.phase = phase;
}

void system_state_set_radar_ready(uint8_t ready)
{
    gSystemState.radar_ready = ready ? 1U : 0U;
}

void system_state_set_tasks_ready(uint8_t ready)
{
    gSystemState.tasks_ready = ready ? 1U : 0U;
}

void system_state_set_ble_ready(uint8_t ready)
{
    gSystemState.ble_ready = ready ? 1U : 0U;
}

void system_state_set_wifi_ready(uint8_t ready)
{
    gSystemState.wifi_ready = ready ? 1U : 0U;
}

void system_state_set_mqtt_ready(uint8_t ready)
{
    gSystemState.mqtt_ready = ready ? 1U : 0U;
}

void system_state_set_error(uint8_t error)
{
    gSystemState.error = error ? 1U : 0U;
    if (gSystemState.error) {
        gSystemState.phase = SYSTEM_PHASE_ERROR;
    }
}

void system_state_set_network_status(NetworkStatus status)
{
    gSystemState.network_status = status;
}

void system_state_set_device_id(uint16_t device_id)
{
    gSystemState.current_device_id = device_id;
}

void system_state_set_device_sn(uint64_t device_sn)
{
    gSystemState.device_sn = device_sn;
}

SystemStateSnapshot system_state_get_snapshot(void)
{
    SystemStateSnapshot snapshot = gSystemState;
    const uint64_t nowMs = radar_now_ms();
    if (nowMs >= snapshot.boot_time_ms) {
        snapshot.uptime_ms = nowMs - snapshot.boot_time_ms;
    } else {
        snapshot.uptime_ms = 0U;
    }
    return snapshot;
}

#ifndef RADAR_MANAGER_H
#define RADAR_MANAGER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float breath_rate;
    float heart_rate;
    uint8_t breath_valid;
    uint8_t heart_valid;
    uint8_t presence;
    uint8_t motion;
    int heartbeat_waveform;
    int breathing_waveform;
    uint16_t distance;
    uint8_t body_movement;
    uint8_t breath_status;
    uint8_t sleep_state;
    uint32_t sleep_time;
    uint8_t sleep_score;
    uint8_t abnormal_state;
    uint8_t avg_heart_rate;
    uint8_t avg_breath_rate;
    int16_t pos_x;
    int16_t pos_y;
    int16_t pos_z;
    uint64_t last_update_ms;
} SensorData;

typedef struct {
    float heart_rate;
    float breath_rate;
    uint8_t presence;
    uint8_t motion;
    uint8_t sleep_state;
    uint8_t body_movement;
    uint16_t distance;
    uint8_t abnormal_state;
    uint64_t last_update_ms;
} RadarVitalsSnapshot;

typedef struct {
    uint8_t presence_changed;
    uint8_t motion_changed;
    uint8_t sleep_state_changed;
    uint8_t body_movement_changed;
    uint8_t distance_changed;
    uint8_t heart_rate_changed;
    uint8_t breath_rate_changed;
    uint8_t abnormal_state_changed;
    uint8_t any_changed;
} RadarChangeFlags;

typedef struct {
    float heart_rate;
    float breath_rate;
    uint16_t distance;
    uint8_t presence;
    uint8_t motion;
    uint8_t sleep_state;
    uint8_t body_movement;
    uint8_t abnormal_state;
    uint8_t heart_valid;
    uint8_t breath_valid;
    uint8_t has_person;
    uint8_t bed_occupied;
    uint8_t has_valid_vitals;
    uint8_t has_fresh_sample;
    uint8_t has_meaningful_change;
    uint64_t last_update_ms;
    uint32_t sample_age_ms;
} RadarReportSnapshot;

extern SensorData sensorData;

bool initRadarManager(void);
bool radar_manager_is_initialized(void);
SensorData radar_manager_get_sensor_data(void);
bool radar_manager_has_fresh_sample(uint32_t max_age_ms);
bool radar_manager_has_person(void);
bool radar_manager_is_bed_occupied(void);
bool radar_manager_has_valid_vitals(void);
RadarVitalsSnapshot radar_manager_get_vitals_snapshot(void);
RadarChangeFlags radar_manager_get_change_flags(void);
bool radar_manager_has_meaningful_change(void);
void radar_manager_mark_snapshot_consumed(void);
RadarReportSnapshot radar_manager_get_report_snapshot(uint32_t max_age_ms);
uint32_t radar_manager_generate_device_hash(void);
bool radar_manager_get_device_mac(char *buffer, size_t buffer_size);
bool radar_manager_get_device_name(char *buffer, size_t buffer_size);
bool radar_manager_get_default_mqtt_client_id(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif

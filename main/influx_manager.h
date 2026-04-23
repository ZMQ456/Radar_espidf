#ifndef INFLUX_MANAGER_H
#define INFLUX_MANAGER_H

#include "radar_manager.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool influx_manager_send_daily_data(const RadarReportSnapshot *snapshot);
bool influx_manager_send_sleep_data(const SensorData *data);

#ifdef __cplusplus
}
#endif

#endif

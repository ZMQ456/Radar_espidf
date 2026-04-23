#ifndef TASKS_MANAGER_H
#define TASKS_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NET_INITIAL = 0,
    NET_DISCONNECTED,
    NET_CONNECTING,
    NET_CONNECTED
} TaskNetworkStatus;

typedef enum {
    TASK_WIFI_EVENT_STA_START = 0,
    TASK_WIFI_EVENT_STA_CONNECTED,
    TASK_WIFI_EVENT_STA_GOT_IP,
    TASK_WIFI_EVENT_STA_DISCONNECTED,
    TASK_WIFI_EVENT_STA_STOP
} TaskWiFiEvent;

extern TaskNetworkStatus currentNetworkStatus;

void setNetworkStatus(TaskNetworkStatus status);
void WiFiEvent(TaskWiFiEvent event);
bool initRadarManager(void);
bool initAllTasks(void);
bool tasks_manager_init(void);
void bootButtonMonitorTask(void *parameter);
void ledControlTask(void *parameter);
void wifiMonitorTask(void *parameter);
void bleConfigTask(void *parameter);
void sendStatusToBLE(void);
void processBLEConfig(void);
void tasks_manager_set_continuous_send(bool enabled, unsigned long interval_ms);
bool tasks_manager_get_continuous_send_enabled(void);
unsigned long tasks_manager_get_continuous_send_interval_ms(void);

#ifdef __cplusplus
}
#endif

#endif

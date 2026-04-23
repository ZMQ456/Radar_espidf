#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// Optional WiFi defaults. Keep these empty unless you want boot-time auto-connect.
#define APP_WIFI_DEFAULT_SSID     ""  
#define APP_WIFI_DEFAULT_PASSWORD ""

// Minimal MQTT test configuration.
#define APP_MQTT_BROKER_URI "mqtt://192.168.1.100:1883"
#define APP_MQTT_CLIENT_ID  "espidf-radar-demo"

// Temporary bootstrap identity defaults.
#define APP_DEVICE_ID_DEFAULT 1001U
#define APP_DEVICE_SN_DEFAULT 0ULL

// GPIO layout aligned with the Arduino prototype.
#define APP_BOOT_BUTTON_PIN 0
#define APP_NETWORK_LED_PIN 5
#define APP_CONFIG_CLEAR_PIN 4

// Task timing aligned to the original tasks_manager.cpp behavior.
#define APP_CLEAR_CONFIG_DURATION_MS 3000U
#define APP_LED_SLOW_BLINK_INTERVAL_MS 1000U
#define APP_LED_FAST_BLINK_INTERVAL_MS 200U
#define APP_LED_BREATHE_INTERVAL_MS 40U
#define APP_LED_BREATHE_MIN 0U
#define APP_LED_BREATHE_MAX 155U
#define APP_LED_BREATHE_STEP 5U

#endif

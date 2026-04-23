#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define RADAR_UART_PORT  2
#define RADAR_RX_PIN    16
#define RADAR_TX_PIN    17
#define RADAR_BAUD_RATE 115200
#define RADAR_UART_BUFFER_SIZE 4096
#define RADAR_FRAME_BUFFER_SIZE 256

#define FRAME_HEADER1 0x53
#define FRAME_HEADER2 0x59
#define FRAME_TAIL1   0x54
#define FRAME_TAIL2   0x43

#define DEBUG_UART_PORT  0
#define DEBUG_BAUD_RATE 115200

#define SAMPLE_RATE_HZ      50
#define SAMPLE_INTERVAL_MS  20
#define RX_BUFFER_SIZE      512
#define DATA_BUFFER_SIZE    300

#define HR_MIN_NORMAL    40
#define HR_MAX_NORMAL    200
#define HR_RESTING_MIN   50
#define HR_RESTING_MAX   90
#define HR_SUDDEN_CHANGE 30

#define RR_MIN_NORMAL    8
#define RR_MAX_NORMAL    30
#define RR_RESTING_MIN   12
#define RR_RESTING_MAX   20
#define RR_SUDDEN_CHANGE 10

#define EMA_ALPHA_HR 0.15f
#define EMA_ALPHA_RR 0.10f

#define EMOTION_HR_LOW      60
#define EMOTION_HR_NORMAL   80
#define EMOTION_HR_ELEVATED 100
#define EMOTION_HR_HIGH     120

#define EMOTION_RR_SLOW   10
#define EMOTION_RR_NORMAL 16
#define EMOTION_RR_FAST   22

#define EMOTION_HRV_LOW    30
#define EMOTION_HRV_NORMAL 60
#define EMOTION_HRV_HIGH   100

#define constrain_value(x, min_value, max_value) \
    ((x) < (min_value) ? (min_value) : ((x) > (max_value) ? (max_value) : (x)))

typedef enum {
    EMOTION_CALM = 0,
    EMOTION_HAPPY,
    EMOTION_EXCITED,
    EMOTION_ANXIOUS,
    EMOTION_ANGRY,
    EMOTION_SAD,
    EMOTION_STRESSED,
    EMOTION_RELAXED,
    EMOTION_UNKNOWN
} EmotionType;

static const char *const EMOTION_NAMES[] = {
    "calm",
    "happy",
    "excited",
    "anxious",
    "angry",
    "sad",
    "stressed",
    "relaxed",
    "unknown"
};

#ifdef __cplusplus
}
#endif

#endif

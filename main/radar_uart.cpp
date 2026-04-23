#include "radar_uart.h"

#include <inttypes.h>
#include <cstring>

#include "driver/uart.h"
#include "esp_log.h"
#include "radar_platform.h"

static const char *TAG = "radar_uart";
static const uart_port_t RADAR_UART_NUM = static_cast<uart_port_t>(RADAR_UART_PORT);

static TaskHandle_t radarUartTaskHandle = nullptr;
static bool radarUartInitialized = false;
static uint8_t frameBuffer[RADAR_FRAME_BUFFER_SIZE];
static size_t frameIndex = 0;
static bool inFrame = false;
static uint8_t prevByte = 0;

static int16_t parse_signed_coordinate(uint16_t raw_value)
{
    return (raw_value & 0x8000U) ? -static_cast<int16_t>(raw_value & 0x7FFFU)
                                 : static_cast<int16_t>(raw_value);
}

bool radar_parse_frame(const uint8_t *frame, size_t frame_len)
{
    if (frame == nullptr || frame_len < 8) {
        return false;
    }

    if (frame[0] != FRAME_HEADER1 || frame[1] != FRAME_HEADER2 ||
        frame[frame_len - 2] != FRAME_TAIL1 || frame[frame_len - 1] != FRAME_TAIL2) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i < frame_len - 3; ++i) {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    if (checksum != frame[frame_len - 3]) {
        ESP_LOGW(TAG, "frame checksum mismatch calc=0x%02X recv=0x%02X",
                 checksum, frame[frame_len - 3]);
        return false;
    }

    const uint8_t ctrlByte = frame[2];
    const uint8_t cmdByte = frame[3];
    const uint16_t dataLen = static_cast<uint16_t>((frame[4] << 8) | frame[5]);

    if ((6U + dataLen + 3U) > frame_len) {
        ESP_LOGW(TAG, "frame length mismatch dataLen=%u frameLen=%u",
                 static_cast<unsigned>(dataLen), static_cast<unsigned>(frame_len));
        return false;
    }

    switch (ctrlByte) {
    case 0x80:
        switch (cmdByte) {
        case 0x01:
            if (dataLen >= 1) sensorData.presence = frame[6];
            break;
        case 0x02:
            if (dataLen >= 1) sensorData.motion = frame[6];
            break;
        case 0x03:
            if (dataLen >= 1) sensorData.body_movement = frame[6];
            break;
        case 0x04:
            if (dataLen >= 2) {
                sensorData.distance = static_cast<uint16_t>((frame[6] << 8) | frame[7]);
            }
            break;
        case 0x05:
            if (dataLen >= 6) {
                sensorData.pos_x = parse_signed_coordinate(static_cast<uint16_t>((frame[6] << 8) | frame[7]));
                sensorData.pos_y = parse_signed_coordinate(static_cast<uint16_t>((frame[8] << 8) | frame[9]));
                sensorData.pos_z = parse_signed_coordinate(static_cast<uint16_t>((frame[10] << 8) | frame[11]));
            }
            break;
        default:
            break;
        }
        break;

    case 0x81:
        switch (cmdByte) {
        case 0x01:
            if (dataLen >= 1) sensorData.breath_status = frame[6];
            break;
        case 0x02:
            if (dataLen >= 1) {
                sensorData.breath_rate = static_cast<float>(frame[6]);
                sensorData.breath_valid = (sensorData.breath_rate >= 0.0f && sensorData.breath_rate <= 35.0f);
            }
            break;
        case 0x05:
            if (dataLen >= 5) {
                for (uint16_t i = 0; i < 5U && i < dataLen; ++i) {
                    sensorData.breath_waveform[i] = static_cast<int8_t>(frame[6 + i] - 128);
                }
                sensorData.breathing_waveform = sensorData.breath_waveform[0];
            }
            break;
        default:
            break;
        }
        break;

    case 0x85:
        switch (cmdByte) {
        case 0x02:
            if (dataLen >= 1) {
                sensorData.heart_rate = static_cast<float>(frame[6]);
                sensorData.heart_valid = (sensorData.heart_rate >= 60.0f && sensorData.heart_rate <= 120.0f);
            }
            break;
        case 0x05:
            if (dataLen >= 5) {
                for (uint16_t i = 0; i < 5U && i < dataLen; ++i) {
                    sensorData.heart_waveform[i] = static_cast<int8_t>(frame[6 + i] - 128);
                }
                sensorData.heartbeat_waveform = sensorData.heart_waveform[0];
            }
            break;
        default:
            break;
        }
        break;

    case 0x84:
        switch (cmdByte) {
        case 0x01:
        case 0x81:
            if (dataLen >= 1) {
                sensorData.bed_status = frame[6];
                sensorData.bed_entry = frame[6];
            }
            break;
        case 0x03:
        case 0x83:
            if (dataLen >= 2) {
                sensorData.awake_time = static_cast<uint16_t>((frame[6] << 8) | frame[7]);
            }
            break;
        case 0x04:
        case 0x84:
            if (dataLen >= 2) {
                sensorData.light_sleep_time = static_cast<uint16_t>((frame[6] << 8) | frame[7]);
            }
            break;
        case 0x05:
        case 0x85:
            if (dataLen >= 2) {
                sensorData.deep_sleep_time = static_cast<uint16_t>((frame[6] << 8) | frame[7]);
            }
            break;
        case 0x06:
            if (dataLen >= 1) {
                sensorData.sleep_score = frame[6];
            }
            break;
        case 0x86:
            if (dataLen >= 2) {
                sensorData.sleep_score = frame[6];
            }
            break;
        case 0x0C:
        case 0x8D:
            if (dataLen >= 8) {
                sensorData.presence = frame[6];
                sensorData.sleep_state = frame[7];
                sensorData.avg_breath_rate = frame[8];
                sensorData.avg_heart_rate = frame[9];
                sensorData.turnover_count = frame[10];
                sensorData.large_move_ratio = frame[11];
                sensorData.small_move_ratio = frame[12];
                sensorData.apnea_count = frame[13];
            }
            break;
        case 0x0D:
        case 0x8F:
            if (dataLen >= 12) {
                sensorData.sleep_score = frame[6];
                sensorData.sleep_total_time = static_cast<uint16_t>((frame[7] << 8) | frame[8]);
                sensorData.sleep_time = sensorData.sleep_total_time;
                sensorData.awake_ratio = frame[9];
                sensorData.light_sleep_ratio = frame[10];
                sensorData.deep_sleep_ratio = frame[11];
                sensorData.bed_Out_Time = frame[12];
                sensorData.turn_count = frame[13];
                sensorData.turnover_count = frame[14];
                sensorData.avg_breath_rate = frame[15];
                sensorData.avg_heart_rate = frame[16];
                sensorData.apnea_count = frame[17];
            }
            break;
        case 0x0E:
        case 0x8E:
            if (dataLen >= 1) {
                sensorData.abnormal_state = frame[6];
            }
            break;
        case 0x10:
        case 0x90:
            if (dataLen >= 1) {
                sensorData.sleep_grade = frame[6];
            }
            break;
        case 0x11:
        case 0x91:
            if (dataLen >= 1) {
                sensorData.struggle_alert = frame[6];
            }
            break;
        case 0x12:
        case 0x92:
            if (dataLen >= 1) {
                sensorData.no_one_alert = frame[6];
            }
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }

    sensorData.last_update_ms = radar_now_ms();
    sensorData.heart_valid = (sensorData.heart_rate > 0.0f && sensorData.heart_rate < 200.0f);
    sensorData.breath_valid = (sensorData.breath_rate >= 0.1f && sensorData.breath_rate <= 60.0f);

    ESP_LOGI(TAG,
             "sensor presence=%u motion=%u dist=%u hr=%.1f rr=%.1f sleep=%u body=%u updated=%" PRIu64,
             sensorData.presence,
             sensorData.motion,
             sensorData.distance,
             sensorData.heart_rate,
             sensorData.breath_rate,
             sensorData.sleep_state,
             sensorData.body_movement,
             sensorData.last_update_ms);
    return true;
}

static void radar_uart_task(void *parameter)
{
    (void)parameter;

    uint8_t rxBuffer[128];

    while (true) {
        const int len = uart_read_bytes(RADAR_UART_NUM, rxBuffer, sizeof(rxBuffer), pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; ++i) {
            const uint8_t c = rxBuffer[i];

            if (!inFrame) {
                if (prevByte == FRAME_HEADER1 && c == FRAME_HEADER2) {
                    inFrame = true;
                    frameIndex = 0;
                    frameBuffer[frameIndex++] = FRAME_HEADER1;
                    frameBuffer[frameIndex++] = FRAME_HEADER2;
                }
            } else {
                if (frameIndex < sizeof(frameBuffer)) {
                    frameBuffer[frameIndex++] = c;

                    if (frameIndex >= 2 &&
                        frameBuffer[frameIndex - 2] == FRAME_TAIL1 &&
                        frameBuffer[frameIndex - 1] == FRAME_TAIL2) {
                        const bool parsed = radar_parse_frame(frameBuffer, frameIndex);
                        if (!parsed) {
                            ESP_LOGW(TAG, "received invalid radar frame, len=%u", static_cast<unsigned>(frameIndex));
                        }
                        inFrame = false;
                        frameIndex = 0;
                    }
                } else {
                    ESP_LOGW(TAG, "frame buffer overflow, dropping frame");
                    inFrame = false;
                    frameIndex = 0;
                }
            }

            prevByte = c;
        }
    }
}

bool radar_uart_init(void)
{
    if (radarUartInitialized) {
        return true;
    }

    const uart_config_t uartConfig = {
        .baud_rate = RADAR_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .backup_before_sleep = 0,
        },
    };

    esp_err_t err = uart_driver_install(RADAR_UART_NUM, RADAR_UART_BUFFER_SIZE, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", static_cast<int>(err));
        return false;
    }

    err = uart_param_config(RADAR_UART_NUM, &uartConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", static_cast<int>(err));
        return false;
    }

    err = uart_set_pin(RADAR_UART_NUM, RADAR_TX_PIN, RADAR_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %d", static_cast<int>(err));
        return false;
    }

    radarUartInitialized = true;
    ESP_LOGI(TAG, "uart initialized on port=%d tx=%d rx=%d baud=%d",
             static_cast<int>(RADAR_UART_NUM),
             RADAR_TX_PIN,
             RADAR_RX_PIN,
             RADAR_BAUD_RATE);
    return true;
}

void radar_uart_start_task(void)
{
    if (!radarUartInitialized) {
        if (!radar_uart_init()) {
            return;
        }
    }

    if (radarUartTaskHandle != nullptr) {
        return;
    }

    xTaskCreate(radar_uart_task, "radar_uart_task", 4096, nullptr, 5, &radarUartTaskHandle);
}

bool radar_uart_is_initialized(void)
{
    return radarUartInitialized;
}

void sendRadarCommand(uint8_t ctrl, uint8_t cmd, uint8_t value)
{
    if (!radarUartInitialized && !radar_uart_init()) {
        return;
    }

    uint8_t command[10];
    command[0] = FRAME_HEADER1;
    command[1] = FRAME_HEADER2;
    command[2] = ctrl;
    command[3] = cmd;
    command[4] = 0x00;
    command[5] = 0x01;
    command[6] = value;
    command[7] = 0x00;
    for (int i = 0; i < 7; ++i) {
        command[7] += command[i];
    }
    command[8] = FRAME_TAIL1;
    command[9] = FRAME_TAIL2;

    const int written = uart_write_bytes(RADAR_UART_NUM, command, sizeof(command));
    ESP_LOGI(TAG, "sent radar command ctrl=0x%02X cmd=0x%02X value=0x%02X bytes=%d",
             ctrl, cmd, value, written);
}

void initR60ABD1(void)
{
    static const uint8_t startupCommands[][3] = {
        {0x80, 0x00, 0x01},
        {0x81, 0x00, 0x01},
        {0x85, 0x00, 0x01},
        {0x84, 0x00, 0x01},
        {0x81, 0x0C, 0x01},
        {0x85, 0x0A, 0x01},
        {0x84, 0x13, 0x01},
        {0x84, 0x14, 0x01},
        {0x80, 0x80, 0x0F},
        {0x81, 0x80, 0x0F},
        {0x85, 0x80, 0x0F},
        {0x84, 0x80, 0x0F},
    };

    if (!radarUartInitialized && !radar_uart_init()) {
        return;
    }

    const uint8_t queryPresenceCmd[] = {0x53, 0x59, 0x80, 0x81, 0x00, 0x01, 0x00, 0x7D, 0x54, 0x43};
    uart_write_bytes(RADAR_UART_NUM, queryPresenceCmd, sizeof(queryPresenceCmd));
    radar_sleep_ms(50);

    for (const auto &startupCommand : startupCommands) {
        sendRadarCommand(startupCommand[0], startupCommand[1], startupCommand[2]);
        radar_sleep_ms(50);
    }
}

size_t radar_uart_read_available(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms)
{
    if (!radarUartInitialized && !radar_uart_init()) {
        return 0;
    }

    const int len = uart_read_bytes(RADAR_UART_NUM, buffer, buffer_size, pdMS_TO_TICKS(timeout_ms));
    return len > 0 ? static_cast<size_t>(len) : 0;
}

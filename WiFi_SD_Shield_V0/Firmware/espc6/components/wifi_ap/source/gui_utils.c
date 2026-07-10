#include <stdbool.h>
#include "gui_task.h"
#include "common.h"
#include "esp_log.h"
#define GUI_TAG "[gui_utils.c]"

static bool is_start_command(uint8_t command)
{
    switch (command) {
        case START_EEG_STREAMING:
        case START_MIC_STREAMING:
        case START_STREAMING_ALL:
        case START_IMU_STREAMING:
        case START_EEG_MIC_STREAMING:
        case START_EMG_STREAMING:
        case START_DUMMY_STREAMING:
            return true;
        default:
            return false;
    }
}

static bool is_stop_command(uint8_t command)
{
    switch (command) {
        case STOP_EEG_STREAMING:
        case STOP_MIC_STREAMING:
        case STOP_STREAMING_ALL:
        case STOP_IMU_STREAMING:
        case STOP_EEG_MIC_STREAMING:
        case STOP_EMG_STREAMING:
        case STOP_DUMMY_STREAMING:
        case ESP_STOP_COMMAND:
            return true;
        default:
            return false;
    }
}

esp_err_t parse_gui_command(uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        ESP_LOGE(GUI_TAG, "Empty GUI command buffer");
        return ESP_ERR_INVALID_ARG;
    }

    if (len != 1) {
        ESP_LOGW(GUI_TAG, "Ignoring non-command GUI frame, len=%u", (unsigned)len);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t command = buf[0];
    esp_err_t ret = validate_command(command);
    if (ret != ESP_OK) {
        ESP_LOGE(GUI_TAG, "Invalid command received from GUI: %u", (unsigned)command);
        return ret;
    }

    switch (node_state) {
        case STATE_IDLE:
            if (!is_start_command(command)) {
                ESP_LOGW(GUI_TAG, "Unexpected command %u while idle", (unsigned)command);
                return ESP_ERR_INVALID_STATE;
            }
            rx_gui_data_to_fwd[0] = command; // copy the command to the forwarding buffer for the send_to_biogap task
            xEventGroupSetBits(g_evt, B_START_CMD_RCV);

            ESP_LOGI(GUI_TAG, "Received START command: %d", (unsigned)command);
            // if (rx_gui_task_handle != NULL) {
            //     xTaskNotifyGive(rx_gui_task_handle);
            // }
            break;
        case STATE_STREAMING:
            if (!is_stop_command(command)) {
                ESP_LOGW(GUI_TAG, "Unexpected command %u while streaming", (unsigned)command);
                return ESP_ERR_INVALID_STATE;
            }
            xEventGroupSetBits(g_evt, B_STOP_CMD_RPT_PENDING);
            rx_gui_data_to_fwd[0] = command; // copy the command to the forwarding buffer for the send_to_biogap task
            break;

        default:
            ESP_LOGW(GUI_TAG, "Unknown node state %d", (int)node_state);
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

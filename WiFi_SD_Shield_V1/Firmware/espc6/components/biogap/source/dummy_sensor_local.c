#include "dummy_sensor_local.h"
#include "biogap.h"
#include "gui_task.h"
#include "common.h"
#include "esp_timer.h"
#include "esp_log.h"

#define DUMMY_LOCAL_TAG "[dummy_sensor_local.c]"

/* Matches the nRF dummy sensor exactly: 4 samples/packet at 100ms/sample
 * (Firmware/src_NRF/sensors/dummy_sensor/dummy_sensor_appl.h, SENSOR_SAMPLING_PERIOD_MS). */
#define DUMMY_SAMPLES_PER_PACKET   4
#define DUMMY_BYTES_PER_SAMPLE     50
#define DUMMY_PACKET_PERIOD_MS     (1 * DUMMY_SAMPLES_PER_PACKET)

/**
 * @brief Build one 211-byte dummy-sensor packet, byte-identical to
 * build_full_dummy_packet()/fill_dummy_exg_sample() in dummy_sensor_appl.c.
 *
 * Layout: [0] 0x55 header, [1:3) counter (u16 LE), [3:7) timestamp us (u32 LE),
 * [7:207) 4 samples x 50 bytes (sample[i] = (sample_index*50 + i) & 0xFF),
 * [207:210) metadata (board_id, sync_pulse_count, reserved), [210] 0xAA tailer.
 */
static void build_dummy_packet(uint8_t *packet, uint16_t counter)
{
    uint32_t ts_us = (uint32_t)esp_timer_get_time();

    packet[0] = NRF_EXG_HEADER;
    packet[1] = (uint8_t)(counter & 0xFF);
    packet[2] = (uint8_t)((counter >> 8) & 0xFF);
    packet[3] = (uint8_t)(ts_us & 0xFF);
    packet[4] = (uint8_t)((ts_us >> 8) & 0xFF);
    packet[5] = (uint8_t)((ts_us >> 16) & 0xFF);
    packet[6] = (uint8_t)((ts_us >> 24) & 0xFF);

    for (uint32_t s = 0; s < DUMMY_SAMPLES_PER_PACKET; s++) {
        uint32_t base = s * DUMMY_BYTES_PER_SAMPLE;
        for (uint32_t i = 0; i < DUMMY_BYTES_PER_SAMPLE; i++) {
            packet[7 + s * DUMMY_BYTES_PER_SAMPLE + i] = (uint8_t)(base + i);
        }
    }

    packet[NRF_EXG_PACKET_SIZE - 4] = 0x01; /* board_id placeholder */
    packet[NRF_EXG_PACKET_SIZE - 3] = 0x00; /* sync_pulse_count placeholder */
    packet[NRF_EXG_PACKET_SIZE - 2] = 0x00; /* reserved */
    packet[NRF_EXG_PACKET_SIZE - 1] = NRF_EXG_TAILER;
}

void dummy_sensor_local_task(void *pv)
{
    (void)pv;
    uint8_t packet[NRF_EXG_PACKET_SIZE];
    uint16_t counter = 0;

    while (1) {
        xEventGroupWaitBits(g_evt, B_START_CMD_RCV, pdTRUE, pdFALSE, portMAX_DELAY);

        ESP_LOGI(DUMMY_LOCAL_TAG, "Starting local dummy sensor generation");
        node_state = STATE_STREAMING;
        counter = 0;
        xEventGroupClearBits(g_evt, B_STOP_CMD_FWD_TO_BIOGAP);
        xEventGroupSetBits(g_evt, B_START_CMD_FWD_TO_BIOGAP);

        TickType_t last_wake = xTaskGetTickCount();
        while (!(xEventGroupGetBits(g_evt) & (B_STOP_CMD_RCV_GUI | B_STOP_CMD_RCV_FORCED))) {
            build_dummy_packet(packet, counter++);
            if (add_to_ringbuffer(packet, NRF_EXG_PACKET_SIZE) != ESP_OK) {
                ESP_LOGW(DUMMY_LOCAL_TAG, "Failed to add packet to ringbuffer");
            }
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DUMMY_PACKET_PERIOD_MS));
        }

        ESP_LOGI(DUMMY_LOCAL_TAG, "Stop requested, halting local dummy sensor generation");
        node_state = STATE_IDLE;
        xEventGroupClearBits(g_evt, B_START_CMD_RCV | B_START_CMD_FWD_TO_BIOGAP);
        xEventGroupSetBits(g_evt, B_STOP_CMD_FWD_TO_BIOGAP);
    }
}

esp_err_t prepare_for_restart_local_dummy(void)
{
    xEventGroupWaitBits(g_evt, B_STOP_CMD_FWD_TO_BIOGAP, pdFALSE, pdFALSE, portMAX_DELAY);

    esp_err_t ret = rb_soft_flush(biogap_ringbuf);
    if (ret != ESP_OK) {
        ESP_LOGE(DUMMY_LOCAL_TAG, "Failed to soft-flush BIOGAP ringbuffer");
        return ret;
    }

    xEventGroupClearBits(g_evt, B_RINGBUFFER_FULL);
    xEventGroupClearBits(g_evt, B_STOP_CMD_RCV_GUI | B_STOP_CMD_RCV_FORCED | B_STOP_CMD_FWD_TO_BIOGAP | B_STOP_CMD_RPT_PENDING);
    return ESP_OK;
}

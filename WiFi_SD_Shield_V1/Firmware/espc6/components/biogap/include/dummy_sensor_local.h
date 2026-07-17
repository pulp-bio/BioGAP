#ifndef DUMMY_SENSOR_LOCAL_H
#define DUMMY_SENSOR_LOCAL_H

#include "esp_err.h"

/**
 * @brief Task that generates synthetic dummy-sensor packets locally on the ESP32,
 * bypassing SPI/BIOGAP entirely.
 *
 * Waits for B_START_CMD_RCV (already set by parse_gui_command() on receiving
 * START_DUMMY_STREAMING), then periodically builds packets - byte-identical in
 * format to Firmware/src_NRF/sensors/dummy_sensor/dummy_sensor_appl.c on the nRF -
 * and pushes them into biogap_ringbuf via add_to_ringbuffer(), the same helper the
 * real SPI-read path uses, so tx_to_gui() needs no changes to relay them to BioGUI.
 *
 * On B_STOP_CMD_RCV_GUI it stops generating, sets node_state back to STATE_IDLE,
 * and signals B_STOP_CMD_FWD_TO_BIOGAP - the same handshake contract the real
 * send_to_biogap_task_nrf_master_esp_slave uses - so prepare_for_restart_local_dummy()
 * can wait on it exactly like prepare_for_restart() does for the real path.
 *
 * Only meaningful when ESP_LOCAL_DUMMY_SENSOR is set (shield_config.h).
 */
void dummy_sensor_local_task(void *pv);

/**
 * @brief Local-dummy-mode counterpart of prepare_for_restart().
 *
 * Waits for B_STOP_CMD_FWD_TO_BIOGAP (set by dummy_sensor_local_task once it has
 * stopped), soft-flushes biogap_ringbuf, and clears the STOP-related event bits -
 * everything prepare_for_restart() does except the SPI slave / pre-queue
 * re-initialization, which does not apply when there is no real SPI link.
 */
esp_err_t prepare_for_restart_local_dummy(void);

#endif // DUMMY_SENSOR_LOCAL_H

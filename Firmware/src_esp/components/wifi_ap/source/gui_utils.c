/*
 * ----------------------------------------------------------------------
 *
 * File: gui_utils.c
 *
 * Last edited: 17.07.2026
 *
 * Copyright (c) 2026 ETH Zurich and University of Bologna
 *
 * Authors:
 * - Giusy Spacone (gspacone@iis.ee.ethz.ch), ETH Zurich
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdbool.h>
#include <string.h>
#include "gui_task.h"
#include "common.h"
#include "esp_log.h"
#define GUI_TAG "[gui_utils.c]"

/* Bytes accumulated so far for the in-progress config+start sequence (see
 * the STATE_IDLE branch below). Distinct from rx_gui_data_len, which is
 * only set once, to the final total, right when the accumulated sequence
 * is relayed. */
static size_t gui_accum_len = 0;

/** @brief Validate and dispatch a command chunk received from BioGUI. */
esp_err_t parse_gui_command(uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        ESP_LOGE(GUI_TAG, "Empty GUI command buffer");
        return ESP_ERR_INVALID_ARG;
    }

    if (len > RX_FROM_GUI_BUF_SIZE) {
        ESP_LOGW(GUI_TAG, "GUI frame too large (len=%u, max=%u), dropping", (unsigned)len, (unsigned)RX_FROM_GUI_BUF_SIZE);
        return ESP_ERR_INVALID_ARG;
    }

    if (node_state == STATE_STREAMING) {
        /* STOP-quiesce trigger only -- enter_stop_quiesce_state()
         * (biogap_read.c) has its own complete delivery mechanism (stomping
         * the bulk pool's tx buffers) and doesn't use the general relay/
         * accumulation path below. Keeping these mutually exclusive by
         * node_state avoids both ever retrieving completions at the same
         * time on the same SPI host. */
        memcpy(rx_gui_data_to_fwd, buf, len);
        rx_gui_data_len = len;
        xEventGroupSetBits(g_evt, B_STOP_CMD_RPT_PENDING);
        return ESP_OK;
    }

    /* STATE_IDLE: the ESP never interprets what an opcode means -- config
     * commands (ADS settings, WULPUS conf, start, ...) all look the same to
     * it. The GUI's whole config+start sequence can span multiple received
     * chunks, so accumulate raw bytes into rx_gui_data_to_fwd across calls
     * and only relay once GUI_CONFIG_END_MARKER arrives as its own,
     * dedicated one-byte chunk (sent by the GUI after the full sequence).
     * This keeps the whole sequence to exactly one control-frame relay
     * (biogap_send.c's propagate_first_start_command_to_biogap_master())
     * before streaming begins, so biogap_read.c can safely arm the bulk
     * pre-queue pool right after that one relay is confirmed delivered,
     * with nothing else ever competing for the same SPI slave queue slot. */
    if (len == 1 && buf[0] == GUI_CONFIG_END_MARKER) {
        rx_gui_data_len = gui_accum_len;
        gui_accum_len = 0;
        ESP_LOGI(GUI_TAG, "End of config sequence, relaying %u bytes to NRF", (unsigned)rx_gui_data_len);
        xEventGroupSetBits(g_evt, B_START_CMD_RCV);
        return ESP_OK;
    }

    if (gui_accum_len + len > RX_FROM_GUI_BUF_SIZE - 3) {
        ESP_LOGE(GUI_TAG, "Accumulated GUI config would exceed %u bytes, dropping sequence",
                 (unsigned)(RX_FROM_GUI_BUF_SIZE - 3));
        gui_accum_len = 0;
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(&rx_gui_data_to_fwd[gui_accum_len], buf, len);
    gui_accum_len += len;
    return ESP_OK;
}

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
#include "gui_task.h"
#include "common.h"
#include "esp_log.h"
#define GUI_TAG "[gui_utils.c]"


/** @brief Validate and dispatch a single command byte received from BioGUI. */
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

    switch (node_state) {
        case STATE_IDLE:
            rx_gui_data_to_fwd[0] = command; // copy the command to the forwarding buffer for the send_to_biogap task
            xEventGroupSetBits(g_evt, B_START_CMD_RCV);
            ESP_LOGI(GUI_TAG, "Received START command: %d", (unsigned)command);
            break;
        case STATE_STREAMING:
            xEventGroupSetBits(g_evt, B_STOP_CMD_RPT_PENDING);
            rx_gui_data_to_fwd[0] = command; // copy the command to the forwarding buffer for the send_to_biogap task
            break;
        default:
            ESP_LOGW(GUI_TAG, "Unknown node state %d", (int)node_state);
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

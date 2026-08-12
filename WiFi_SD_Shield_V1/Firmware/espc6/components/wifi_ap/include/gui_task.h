/*
 * ----------------------------------------------------------------------
 *
 * File: gui_task.h
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

#ifndef GUI_TASK_H
#define GUI_TASK_H

#include "softap_main.h"
#include "biogap.h"
#include "common.h"

/* Matches SPI_FROM_BIOGAP_MAX_SIZE so a GUI command of any length up to a
 * full SPI transaction can be received and relayed in one shot -- config
 * commands (ADS settings, WULPUS conf) are multi-byte, not just opcodes. */
#define RX_FROM_GUI_BUF_SIZE SPI_FROM_BIOGAP_MAX_SIZE

/* Sent by the GUI as its own dedicated one-byte packet once the whole
 * config+start sequence (ADS settings, WULPUS settings, start opcode, ...)
 * has been sent. Pure wire-level framing, like ESP_SPI_HEADER/TAILER -- the
 * ESP doesn't interpret any of the sequence, it just watches for this
 * marker to know when to relay everything accumulated so far in one control
 * frame. See parse_gui_command() (gui_utils.c). */
#define GUI_CONFIG_END_MARKER 0xEE


/**  @brief  Buffer to store incoming commands from the GUI*/
extern uint8_t rx_data_from_gui[RX_FROM_GUI_BUF_SIZE];
extern uint8_t rx_gui_data_to_fwd[RX_FROM_GUI_BUF_SIZE];
/** @brief Number of valid bytes currently in rx_gui_data_to_fwd. */
extern size_t rx_gui_data_len;

/** @brief Binds to the GUI socket */
esp_err_t bind_to_gui();

/** @brief Task to receive data from the GUI */
void rx_from_gui(void *pv);

/** @brief Handler for rx_from_gui task */
extern TaskHandle_t rx_gui_task_handle;

/** @brief Task to send data to the GUI */
void tx_to_gui(void *pv);

/** @brief Parses commands received from the GUI */
esp_err_t parse_gui_command(uint8_t *buf, size_t len); 

/** @brief Tears down/reinitializes the SPI bus + DMA buffers, guaranteeing an
 *  empty transaction queue. Only safe once the NRF is confirmed (or very
 *  likely) idle -- see its doc comment in gui_task.c for the two call sites
 *  (post-STOP, and before every non-first START). */
esp_err_t reset_spi_bus_for_restart(void);

/** @brief Finishes restart bookkeeping (ringbuffer flush, idle re-arm, event
 *  bits) after a STOP. Sole caller: enter_stop_quiesce_state() (biogap_read.c). */
esp_err_t prepare_for_restart();

/** @brief Drains biogap_ringbuf completely, robust against late in-flight enqueues */
esp_err_t rb_soft_flush(RingbufHandle_t rb);

#endif // GUI_TASK_H
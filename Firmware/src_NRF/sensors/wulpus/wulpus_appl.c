/*
 * ----------------------------------------------------------------------
 *
 * File: wulpus_appl.c
 *
 * Copyright (C) 2026, ETH Zurich
 *
 * Authors:
 * - Sebastian Frey (sefrey@iis.ee.ethz.ch), ETH Zurich
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

/**
 * @file wulpus_appl.c
 * @brief WULPUS ultrasound sensor interface – nRF5340 / Zephyr port
 *
 * This module is a direct port of the nRF52 WULPUS_PROBE_3 firmware to
 * Zephyr on the nRF5340. It preserves the full BLE / SPI packet protocol
 * so the existing WULPUS receiver dongle works without modification.
 *
 * Protocol summary:
 *   - MSP430 signals a new US frame by asserting HOST_DATA_RDY (rising edge).
 *   - nRF5340 performs 4 × 201-byte full-duplex SPI transfers over SPI_B.
 *   - The received frame (4 × 201 bytes) is forwarded via BLE NUS as four
 *     211-byte notifications, each = 7-byte header/counter/timestamp prefix
 *     + one 201-byte SPI chunk + 3 reserved bytes (see WULPUS_META_* and the
 *     BLE forwarding thread).
 *   - Ring buffer holds up to WULPUS_MAX_FRAMES complete US frames.
 *   - The same TX buffer (MSP430 configuration) is used for all 4 SPI TXs.
 *
 * SPI_B pin assignment on BioGAP:
 *   SCK  = P1.07   MOSI = P0.30   MISO = P0.29   CS = P1.08
 *
 * Control GPIOs:
 *   HOST_DATA_RDY = P1.13  input,  rising-edge from MSP430
 *   HOST_LINK_RDY = P0.27  output, driven HIGH after MSP430 config received
 */

#include "sensors/wulpus/wulpus_appl.h"
#include "wifi_sd_shield/wifi_sd_shield_appl.h"
#include "ble/ble_appl.h"
#include "bsp/pwr_bsp.h"

#include <nrfx_spim.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(wulpus, LOG_LEVEL_INF);

/*==============================================================================
 * Protocol constants – must stay in sync with old nRF52 firmware
 *============================================================================*/

/** @brief Bytes per SPI transfer (TX and RX) */
#define WULPUS_BYTES_PER_XFER   201

/** @brief BLE packet headers for the four transfer chunks of one US frame */
#define WULPUS_BLE_HDR_XFER_0   0x10
#define WULPUS_BLE_HDR_XFER_1   0x11
#define WULPUS_BLE_HDR_XFER_2   0x12
#define WULPUS_BLE_HDR_XFER_3   0x13

/** @brief SPI transfers per US frame */
#define WULPUS_NUMBER_OF_XFERS  4

/** @brief Maximum number of buffered US frames */
#define WULPUS_MAX_FRAMES       35

/** @brief Settle time after enabling the WULPUS rails (VA0/VD0/VD2) before
 *  talking to the MSP430: rail ramp + MSP430 boot. */
#define WULPUS_POWERUP_DELAY_MS 300

/*==============================================================================
 * NRFX SPIM1 configuration
 *============================================================================*/

/**
 * @brief NRFX SPIM instance index (SPI_B of the BioGAP mainboard).
 *
 * nRF5340 serial-peripheral (serial-box) sharing:
 *   SERIAL0 (IRQ 8): SPIM0/TWIM0/UARTE0  – TWIM0 used by I2C_A (PMIC/LED/IMU)
 *   SERIAL1 (IRQ 9): SPIM1/TWIM1/UARTE1  – TWIM1 used by I2C_B (PPG shield)
 *   SERIAL2 (IRQ10): SPIM2/TWIM2/UARTE2  – SPIM2 = SPI_B (this driver)
 *   SERIAL3 (IRQ11): SPIM3/TWIM3/UARTE3  – kept free so UART3 remains
 *                                          available for the GAP9 UART
 *   SPIM4   (IRQ12): dedicated high-speed – SPIM4 = SPI_A (ADS1298)
 *
 * Pins must match the spi_b_default pinctrl in the board DTS
 * (nrf5340_senseiv1_cpuapp.dts).
 */
#define WULPUS_SPIM_INST_IDX    2

/** @brief SPI interrupt priority (same as ADS1298 driver) */
#define WULPUS_SPI_INT_PRIO     2

/*
 * Absolute GPIO pin numbers for nrfx:  P0.x = x,  P1.x = 32 + x
 */
#define WULPUS_PIN_SCK   NRF_GPIO_PIN_MAP(1, 7)   /* SPI_B_CLK  = P1.07 */
#define WULPUS_PIN_MOSI  NRF_GPIO_PIN_MAP(0, 30)  /* SPI_B_MOSI = P0.30 */
#define WULPUS_PIN_MISO  NRF_GPIO_PIN_MAP(0, 29)  /* SPI_B_MISO = P0.29 */
#define WULPUS_PIN_CS    NRF_GPIO_PIN_MAP(1, 8)   /* SPI_B_CS   = P1.08 */

/*==============================================================================
 * Device tree node references (GPIOs defined in the project overlay)
 *============================================================================*/

#define WULPUS_DATA_RDY_NODE  DT_NODELABEL(wulpus_host_data_rdy)
#define WULPUS_LINK_RDY_NODE  DT_NODELABEL(wulpus_host_link_rdy)

/*==============================================================================
 * Buffer type (identical layout to old ArrayList_type)
 *============================================================================*/

typedef struct {
    uint8_t buffer[WULPUS_BYTES_PER_XFER];
} wulpus_xfer_buf_t;

/*==============================================================================
 * Ring buffer
 *
 * Laid out as a flat array of (WULPUS_NUMBER_OF_XFERS * WULPUS_MAX_FRAMES)
 * contiguous slots.  Frame N occupies slots [N*4 .. N*4+3].
 *
 * Each slot's 201 SPI bytes are copied into one BLE packet at offset
 * WULPUS_SPI_OFF (byte 7), behind the header/counter/timestamp prefix.
 *============================================================================*/
static wulpus_xfer_buf_t m_rx_buf[WULPUS_NUMBER_OF_XFERS * WULPUS_MAX_FRAMES];

/** @brief Per-frame acquisition timestamp (µs), captured in the SPI thread when
 *  the frame is received, so the ring-buffer queueing delay does not skew it. */
static uint32_t m_frame_ts[WULPUS_MAX_FRAMES];

/** @brief SPI TX buffer – holds MSP430 configuration, repeated for all 4 TXs */
static uint8_t m_tx_buf[WULPUS_BYTES_PER_XFER];

/*==============================================================================
 * Ring buffer indices
 *   buffer_counter  – next write slot  (incremented by SPI thread)
 *   current_buffer  – next read slot   (incremented by BLE thread)
 *   buffer_content  – frames available
 *============================================================================*/
static volatile int buffer_counter = 0;
static volatile int current_buffer = 0;
static volatile int buffer_content = 0;

/*
 * Per-frame BLE packet counter, written into each frame's padding tail
 * (mirrors the EEG/EMG packet counter). Reset in wulpus_set_msp_config()
 * whenever a new config/restart is received, so each streaming session
 * starts from 0 like the ExG counter.
 */
static uint16_t wulpus_frame_counter = 0;

/*==============================================================================
 * State flags
 *============================================================================*/
static volatile bool msp_conf_received = false;
static volatile bool wulpus_active     = false;

/*
 * Write position for BLE-fragmented MSP430 config assembly.
 * The WULPUS PRO dongle sends a 105-byte config split as two BLE NUS
 * notifications (64 + 41 bytes) because its nRF52 NUS MTU is 64 bytes.
 * A new config always starts with byte 0xFA (250) or 0xFB (251).
 * Continuation fragments start at other byte values and are appended.
 */
static uint16_t m_tx_write_pos = 0;

/*==============================================================================
 * NRFX SPIM instance
 *============================================================================*/
static nrfx_spim_t wulpus_spim = NRFX_SPIM_INSTANCE(WULPUS_SPIM_INST_IDX);

/*==============================================================================
 * Synchronization
 *============================================================================*/

/** @brief Given by SPIM event handler when a transfer completes */
K_SEM_DEFINE(wulpus_spim_done, 0, 1);

/** @brief Given by GPIO ISR when HOST_DATA_RDY rises */
K_SEM_DEFINE(wulpus_data_rdy_sem, 0, 1);

/** @brief Given by SPI thread when a frame is ready to forward over BLE */
K_SEM_DEFINE(wulpus_ble_ready_sem, 0, WULPUS_MAX_FRAMES);

/*==============================================================================
 * GPIO
 *============================================================================*/
static const struct gpio_dt_spec host_data_rdy =
    GPIO_DT_SPEC_GET(WULPUS_DATA_RDY_NODE, gpios);

static const struct gpio_dt_spec host_link_rdy =
    GPIO_DT_SPEC_GET(WULPUS_LINK_RDY_NODE, gpios);

static struct gpio_callback data_rdy_cb_data;

/*==============================================================================
 * SPIM event handler
 * Called from interrupt context when a SPI transfer finishes.
 *============================================================================*/
static void spim_event_handler(nrfx_spim_evt_t const *p_event, void *p_context)
{
    if (p_event->type == NRFX_SPIM_EVENT_DONE) {
        k_sem_give(&wulpus_spim_done);
    }
}

/*==============================================================================
 * GPIO callback – HOST_DATA_RDY rising edge
 * Signals the SPI acquisition thread that the MSP430 has a new US frame.
 *============================================================================*/
static void data_rdy_gpio_cb(const struct device *dev,
                             struct gpio_callback *cb,
                             uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    if (wulpus_active) {
        k_sem_give(&wulpus_data_rdy_sem);
    }
}

/*==============================================================================
 * SPI acquisition thread
 *
 * Waits for HOST_DATA_RDY, then performs 4 consecutive 201-byte SPI
 * transfers.  Each transfer fills the next slot of the ring buffer.
 * On completion the BLE forwarding thread is signalled.
 *============================================================================*/
#define WULPUS_SPI_STACK_SIZE  2048
#define WULPUS_SPI_PRIORITY    4

static void wulpus_spi_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    LOG_INF("WULPUS SPI thread started");

    while (1) {
        k_sem_take(&wulpus_data_rdy_sem, K_FOREVER);

        /* Timestamp frame acquisition here (≈ first sample), not at BLE-forward
         * time, so the ring-buffer queueing delay doesn't skew it. Stored for
         * the slot being filled; the BLE thread reads it back per frame. */
        m_frame_ts[buffer_counter] = k_cyc_to_us_floor32(k_cycle_get_32());

        for (int i = 0; i < WULPUS_NUMBER_OF_XFERS; i++) {
            nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(
                m_tx_buf,
                WULPUS_BYTES_PER_XFER,
                m_rx_buf[buffer_counter * WULPUS_NUMBER_OF_XFERS + i].buffer,
                WULPUS_BYTES_PER_XFER
            );

            // to debug
            uint8_t rx_data[WULPUS_BYTES_PER_XFER];
            memcpy(rx_data, m_rx_buf[buffer_counter * WULPUS_NUMBER_OF_XFERS + i].buffer, WULPUS_BYTES_PER_XFER);
            if(i==0){
                uint16_t acq_nrf_msp = rx_data[2] | (rx_data[3] << 8);
                uint8_t tx_id = rx_data[1];
                uint8_t sox_max = rx_data[0];
                //LOG_INF("WULPUS SPI frame %d, xfer %d, sof mask:%02X, tx_id: %d, acq_nrf_msp: %d", 
                //    buffer_counter, i, sox_max, tx_id, acq_nrf_msp);
            }

            nrfx_err_t err = nrfx_spim_xfer(&wulpus_spim, &xfer, 0);
            if (err != NRFX_SUCCESS) {
                LOG_ERR("SPIM xfer %d failed: 0x%x", i, err);
                continue;
            }

            /* Block until the transfer-done callback fires */
            if (k_sem_take(&wulpus_spim_done, K_MSEC(100)) != 0) {
                LOG_ERR("SPIM xfer %d timeout", i);
            }
        }

        /* Frame complete – advance write pointer */
        buffer_content++;
        if (buffer_content > WULPUS_MAX_FRAMES - 1) {
            LOG_WRN("WULPUS ring buffer overflow");
            buffer_content = WULPUS_MAX_FRAMES - 1;
        }

        buffer_counter++;
        if (buffer_counter == WULPUS_MAX_FRAMES) {
            buffer_counter = 0;
        }
        //LOG_INF("WULPUS SPI done, give BLE ready sem");
        k_sem_give(&wulpus_ble_ready_sem);
    }
}

K_THREAD_DEFINE(wulpus_spi_tid, WULPUS_SPI_STACK_SIZE,
                wulpus_spi_thread, NULL, NULL, NULL,
                WULPUS_SPI_PRIORITY, 0, 0);

/*==============================================================================
 * BLE forwarding thread
 *
 * Drains the ring buffer and enqueues 4 BLE NUS notifications per frame.
 * Each BLE packet is framed with a 7-byte header/counter/timestamp prefix
 * (see WULPUS_META_* below), then one 201-byte SPI transfer chunk, then a
 * 3-byte reserved tail:
 *   packet 0 : 0x10 + 6 meta + 201 payload + 3 reserved = 211 bytes total
 *   packet 1 : 0x11 + 6 meta + 201 payload + 3 reserved = 211 bytes total
 *   packet 2 : 0x12 + 6 meta + 201 payload + 3 reserved = 211 bytes total
 *   packet 3 : 0x13 + 6 meta + 201 payload + 3 reserved = 211 bytes total
 *
 * The per-frame counter+timestamp prefix is mirrored into all four chunks so
 * the receiver can read it from any chunk, and matches the EEG/EMG/MIC/PPG
 * packet layout.
 *
 * 211 bytes matches EEG_PCKT_SIZE so the biogui can use a single
 * packetSize=211 for both WULPUS and EEG/EMG on the same stream.
 *============================================================================*/
#define WULPUS_BLE_STACK_SIZE  2048
#define WULPUS_BLE_PRIORITY    5
/** Standardized BLE packet size – must equal EEG_PCKT_SIZE (211). */
#define WULPUS_BLE_PKT_SIZE  (WULPUS_BYTES_PER_XFER + 1U + 9U)

/*
 * Per-frame metadata written at the FRONT of every chunk, mirrored into all
 * four chunks of a frame so the receiver can read it from any chunk. This
 * matches the EEG/EMG/MIC/PPG packet layout (header, counter, timestamp):
 *   [0]       chunk header (0x10..0x13)
 *   [1:3]     frame counter (uint16 LE)
 *   [3:7]     timestamp us  (uint32 LE)
 *   [7:208]   201-byte SPI payload chunk
 *   [208:211] reserved (zero)
 */
#define WULPUS_META_CNT_OFF  1U
#define WULPUS_META_TS_OFF   3U
#define WULPUS_SPI_OFF       7U

bool wulpus_cfg_sent = false;  // Global variable to track if the WULPUS config has been sent
static void wulpus_ble_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    //LOG_INF("WULPUS BLE thread started");

    uint8_t ble_packet[WULPUS_BLE_PKT_SIZE];
    /* Zero once. The header [0], metadata [1..6] and SPI payload [7..207] are
     * rewritten per chunk/frame below; the reserved bytes [208..210] stay zero. */
    memset(ble_packet, 0, WULPUS_BLE_PKT_SIZE);

    while (1) {
        k_sem_take(&wulpus_ble_ready_sem, K_FOREVER);

        //LOG_INF("WULPUS BLE ready, processing frames");
        while (current_buffer != buffer_counter) {
            int base = current_buffer * WULPUS_NUMBER_OF_XFERS;

            /* Per-frame metadata at the front of every chunk (mirrored), matching
             * the ExG/MIC/PPG layout: counter (u16 LE) + microsecond timestamp
             * (u32 LE). These persist across the four sends because memcpy below
             * only rewrites the SPI payload region [7:208] and the header [0].
             * The timestamp was captured at frame acquisition (SPI thread). */
            uint32_t ts_us = m_frame_ts[current_buffer];
            ble_packet[WULPUS_META_CNT_OFF]     = (uint8_t)(wulpus_frame_counter);
            ble_packet[WULPUS_META_CNT_OFF + 1] = (uint8_t)(wulpus_frame_counter >> 8);
            //LOG_INF("WULPUS frame %d, timestamp %u us", wulpus_frame_counter, ts_us);
            ble_packet[WULPUS_META_TS_OFF]      = (uint8_t)(ts_us);
            ble_packet[WULPUS_META_TS_OFF + 1]  = (uint8_t)(ts_us >> 8);
            ble_packet[WULPUS_META_TS_OFF + 2]  = (uint8_t)(ts_us >> 16);
            ble_packet[WULPUS_META_TS_OFF + 3]  = (uint8_t)(ts_us >> 24);
            wulpus_frame_counter++;

            ble_packet[0] = WULPUS_BLE_HDR_XFER_0;
            memcpy(&ble_packet[WULPUS_SPI_OFF], m_rx_buf[base + 0].buffer, WULPUS_BYTES_PER_XFER);

            #if defined(CONFIG_WI_FI)
                if (wulpus_cfg_sent) {
                LOG_INF("Sending WULPUS frame %d to ESP", wulpus_frame_counter - 1);
                add_data_to_esp_send_buffer(ble_packet, WULPUS_BLE_PKT_SIZE);
                }
            #else

                            
                uint8_t tx_id_sent = ble_packet[8];
                uint16_t wulpus_msp_acq_nr =((uint16_t)ble_packet[10] << 8) |(uint16_t)ble_packet[9];

                //LOG_INF("Skip sending first chunk via ble, tx_id: %d, acq_nr: %d", tx_id_sent, wulpus_msp_acq_nr);

                add_data_to_send_buffer(ble_packet, WULPUS_BLE_PKT_SIZE);
            #endif

            

            ble_packet[0] = WULPUS_BLE_HDR_XFER_1;
            memcpy(&ble_packet[WULPUS_SPI_OFF], m_rx_buf[base + 1].buffer, WULPUS_BYTES_PER_XFER);

            #if defined(CONFIG_WI_FI)
                if (wulpus_cfg_sent) {
                    add_data_to_esp_send_buffer(ble_packet, WULPUS_BLE_PKT_SIZE);
                }
            #else
                add_data_to_send_buffer(ble_packet, WULPUS_BLE_PKT_SIZE);
            #endif

            ble_packet[0] = WULPUS_BLE_HDR_XFER_2;
            memcpy(&ble_packet[WULPUS_SPI_OFF], m_rx_buf[base + 2].buffer, WULPUS_BYTES_PER_XFER);

            #if defined(CONFIG_WI_FI)
                if (wulpus_cfg_sent) {
                    add_data_to_esp_send_buffer(ble_packet, WULPUS_BLE_PKT_SIZE);
                }
            #else
                add_data_to_send_buffer(ble_packet, WULPUS_BLE_PKT_SIZE);
            #endif


            ble_packet[0] = WULPUS_BLE_HDR_XFER_3;
            memcpy(&ble_packet[WULPUS_SPI_OFF], m_rx_buf[base + 3].buffer, WULPUS_BYTES_PER_XFER);

            #if defined(CONFIG_WI_FI)
                if (wulpus_cfg_sent) {
                    add_data_to_esp_send_buffer(ble_packet, WULPUS_BLE_PKT_SIZE);
                }
            #else
                add_data_to_send_buffer(ble_packet, WULPUS_BLE_PKT_SIZE);
            #endif


            buffer_content--;
            current_buffer++;
            if (current_buffer == WULPUS_MAX_FRAMES) {
                current_buffer = 0;
            }
        }
    }
}


/*==============================================================================
 * ESP forwarding thread
 *
 * Collects the samples received from the MSP and builds a full US scan.
 * 
 *
 *============================================================================*/

 
/* esp_packet[WULPUS_ESP_PACKET_SIZE] (814 bytes) here plus
 * add_data_to_esp_send_buffer()'s own esp_packet_t local (ESP_PCKT_MAX_SIZE=850,
 * ~852 bytes) add up to ~1666 bytes in just these two frames -- same
 * ESP_PCKT_MAX_SIZE-driven stack pressure fixed for the SPI NRF-ESP
 * sender/receiver threads, missed here. */
#define WULPUS_ESP_STACK_SIZE  4096
#define WULPUS_ESP_PRIORITY    5

/*packet : header + 6 meta + 4*201 payload + 3 reserved (zero) + tailer = 815 bytes total*/
/*
 *   [0]       header (WULPUS_FULL_HEADER)
 *   [1:3]     frame counter (uint16 LE). Take the counter at the first SPI chunk of the frame
 *   [3:7]     timestamp us  (uint32 LE). Take the timestamp at the first SPI chunk of the frame
 *   [7:811]   4*201-byte SPI payload
 *   [811:814] reserved (zero)
 *   [814]     tailer (WULPUS_FULL_TAILER)
 */
#define WULPUS_ESP_PACKET_SIZE  (4 * WULPUS_BYTES_PER_XFER + 1U + 1U + 2U + 4U +3U)
#define WULPUS_FULL_HEADER   0XCC
#define WULPUS_FULL_TAILER   0xDD
static void wulpus_esp_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    LOG_INF("WULPUS ESP thread started");
      
    bool first_chunck = true; 
    uint8_t esp_packet[WULPUS_ESP_PACKET_SIZE];
    /* Zero once. The header [0], metadata [1..6] and SPI payload [7..207] are
     * rewritten per chunk/frame below; the reserved bytes [208..210] stay zero. */
    memset(esp_packet, 0, WULPUS_ESP_PACKET_SIZE);

    while (1) {
        
        // Receive a chunck of US data from the MSP
        k_sem_take(&wulpus_ble_ready_sem, K_FOREVER);
        //LOG_INF("rcv WULPUS BLE ready, processing frames");
        while (current_buffer != buffer_counter) {
            int base = current_buffer * WULPUS_NUMBER_OF_XFERS;

            /* Per-frame metadata at the front of every chunk (mirrored), matching
             * the ExG/MIC/PPG layout: counter (u16 LE) + microsecond timestamp
             * (u32 LE). These persist across the four sends because memcpy below
             * only rewrites the SPI payload region [7:208] and the header [0].
             * The timestamp was captured at frame acquisition (SPI thread). */

            uint16_t off = WULPUS_SPI_OFF;
            esp_packet[0] = WULPUS_FULL_HEADER;
            // take the counter and timestamp at the first SPI chunk of the frame
            uint32_t ts_us = m_frame_ts[current_buffer];
            esp_packet[WULPUS_META_CNT_OFF]     = (uint8_t)(wulpus_frame_counter);
            esp_packet[WULPUS_META_CNT_OFF + 1] = (uint8_t)(wulpus_frame_counter >> 8);
            //LOG_INF("WULPUS frame %d, timestamp %u us", wulpus_frame_counter, ts_us);
            esp_packet[WULPUS_META_TS_OFF]      = (uint8_t)(ts_us);
            esp_packet[WULPUS_META_TS_OFF + 1]  = (uint8_t)(ts_us >> 8);
            esp_packet[WULPUS_META_TS_OFF + 2]  = (uint8_t)(ts_us >> 16);
            esp_packet[WULPUS_META_TS_OFF + 3]  = (uint8_t)(ts_us >> 24);
            wulpus_frame_counter++;
            // Copy the MSP payload 
            memcpy(&esp_packet[WULPUS_SPI_OFF], m_rx_buf[base + 0].buffer, WULPUS_BYTES_PER_XFER);
            first_chunck = false;
            off += WULPUS_BYTES_PER_XFER;

            // read the rest
            for (int i = 1; i < WULPUS_NUMBER_OF_XFERS; i++) {
                memcpy(&esp_packet[off], m_rx_buf[base + i].buffer, WULPUS_BYTES_PER_XFER);
                off += WULPUS_BYTES_PER_XFER;
            }
            esp_packet[WULPUS_ESP_PACKET_SIZE - 1] = WULPUS_FULL_TAILER;


            if (wulpus_cfg_sent) {
                
                uint8_t tx_id_sent = esp_packet[8];
                uint16_t wulpus_msp_acq_nr =
                ((uint16_t)esp_packet[10] << 8) |
                (uint16_t)esp_packet[9];

                LOG_INF("Sending WULPUS frame %d to ESP, header: 0x%02X, tailer: 0x%02X, acq_nr: %d, tx_id: %d",
                wulpus_frame_counter - 1, esp_packet[0], esp_packet[WULPUS_ESP_PACKET_SIZE - 1], wulpus_msp_acq_nr, tx_id_sent);
                // commented here to check if the prolem is somwhere else on the chain
                add_data_to_esp_send_buffer(esp_packet, WULPUS_ESP_PACKET_SIZE);
            }

            buffer_content--;
            current_buffer++;
            if (current_buffer == WULPUS_MAX_FRAMES) {
                current_buffer = 0;
            }
        }
    }
}


/* BLE thread started if BLE is enabled*/
#if defined CONFIG_WI_FI
    /* Send to ESP thread started if WiFi - SD card is enabled */
    K_THREAD_DEFINE(wulpus_esp_tid, WULPUS_ESP_STACK_SIZE,
                    wulpus_esp_thread, NULL, NULL, NULL,
                    WULPUS_ESP_PRIORITY, 0, 0);

#else
    K_THREAD_DEFINE(wulpus_ble_tid, WULPUS_BLE_STACK_SIZE,
                    wulpus_ble_thread, NULL, NULL, NULL,
                    WULPUS_BLE_PRIORITY, 0, 0);

#endif

/*==============================================================================
 * Public API
 *============================================================================*/

void wulpus_init(void)
{
    /* --- HOST_LINK_RDY output (P0.27) – initially LOW --- */
    if (!device_is_ready(host_link_rdy.port)) {
        LOG_ERR("HOST_LINK_RDY GPIO port not ready");
        return;
    }
    if (gpio_pin_configure_dt(&host_link_rdy, GPIO_OUTPUT_INACTIVE) < 0) {
        LOG_ERR("HOST_LINK_RDY configure failed");
        return;
    }

    /* --- HOST_DATA_RDY input + rising-edge interrupt (P1.13) --- */
    if (!device_is_ready(host_data_rdy.port)) {
        LOG_ERR("HOST_DATA_RDY GPIO port not ready");
        return;
    }
    if (gpio_pin_configure_dt(&host_data_rdy, GPIO_INPUT) < 0) {
        LOG_ERR("HOST_DATA_RDY configure failed");
        return;
    }
    if (gpio_pin_interrupt_configure_dt(&host_data_rdy,
                                        GPIO_INT_EDGE_TO_ACTIVE) < 0) {
        LOG_ERR("HOST_DATA_RDY interrupt configure failed");
        return;
    }
    gpio_init_callback(&data_rdy_cb_data, data_rdy_gpio_cb,
                       BIT(host_data_rdy.pin));
    if (gpio_add_callback(host_data_rdy.port, &data_rdy_cb_data) < 0) {
        LOG_ERR("HOST_DATA_RDY callback add failed");
        return;
    }

    /* --- NRFX SPIM3 – same pattern as ADS1298 (SPIM2) --- */
#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(WULPUS_SPIM_INST_IDX)),
                WULPUS_SPI_INT_PRIO,
                NRFX_SPIM_INST_HANDLER_GET(WULPUS_SPIM_INST_IDX), 0, 0);
#endif

    nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(
        WULPUS_PIN_SCK, WULPUS_PIN_MOSI, WULPUS_PIN_MISO, WULPUS_PIN_CS);

    config.frequency    = NRFX_MHZ_TO_HZ(8);
    config.mode         = NRF_SPIM_MODE_1;        /* CPOL=0, CPHA=1 */
    config.bit_order    = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    config.irq_priority = WULPUS_SPI_INT_PRIO;

    nrfx_err_t err = nrfx_spim_init(&wulpus_spim, &config, spim_event_handler, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_spim_init failed: 0x%x", err);
        return;
    }

    LOG_INF("WULPUS initialized – HOST_LINK_RDY LOW, waiting for MSP430 config");

    /* Race-condition guard: if a BLE config packet arrived before wulpus_init()
     * ran (possible when the host connects quickly and sends the combined
     * EEG+WULPUS start sequence before the 5-second boot delay completes),
     * wulpus_set_msp_config() will have set wulpus_active=true and asserted
     * HOST_LINK_RDY HIGH – but gpio_pin_configure_dt() above just reset it LOW.
     * Re-assert here now that SPI and the GPIO ISR are fully set up. */
    if (wulpus_active) {
        gpio_pin_set_dt(&host_link_rdy, 1);
        LOG_INF("WULPUS: pre-init config detected – HOST_LINK_RDY re-asserted");
    }
}


void wulpus_set_msp_config(const uint8_t *config, uint16_t len)
{
    /* Rails are powered on demand at the first WULPUS use, not at boot:
     * the VD0 5 V boost must not run during EEG/EMG-only sessions
     * (switching noise; on battery it can desense the BLE radio). */
    static bool wulpus_rails_on = false;
    if (!wulpus_rails_on) {
        LOG_INF("Powering WULPUS rails (VA0/VD0/VD2)");
        if (wulpus_power_on() != 0) {
            LOG_ERR("WULPUS rail power-on failed - config not sent");
            return;
        }
        wulpus_rails_on = true;
        k_msleep(WULPUS_POWERUP_DELAY_MS);
    }
    LOG_INF("WULPUS powered on");
    if (config != NULL && len > 0) {
        /* 0xFA (250) = new config, 0xFB (251) = restart — start a fresh assembly */
        LOG_INF("WULPUS wulpus_set_msp_config recv %u bytes - first byte is %d, last byte is %d", len, config[0], config[len - 1]);
        if (config[0] == 250 || config[0] == 251) {
            m_tx_write_pos = 0;
            wulpus_frame_counter = 0;   /* new streaming session: reset counter */
        }
        uint16_t copy_len = MIN(len, (uint16_t)(WULPUS_BYTES_PER_XFER - m_tx_write_pos));
        memcpy(m_tx_buf + m_tx_write_pos, config, copy_len);
        m_tx_write_pos += copy_len;
    }

    /* Reset ring buffer */
    buffer_counter = 0;
    current_buffer = 0;
    buffer_content = 0;

    msp_conf_received = true;
    wulpus_active     = true;

    /* Assert HOST_LINK_RDY → MSP430 will start sending US frames */
    gpio_pin_set_dt(&host_link_rdy, 1);

    LOG_INF("WULPUS config fragment (%u bytes, tx_buf filled %u/%u) – HOST_LINK_RDY HIGH",
            (unsigned)len, (unsigned)m_tx_write_pos, (unsigned)WULPUS_BYTES_PER_XFER);
}

void wulpus_stop(void)
{
    wulpus_active     = false;
    msp_conf_received = false;

    gpio_pin_set_dt(&host_link_rdy, 0);

    LOG_INF("WULPUS stopped – HOST_LINK_RDY LOW");
}

bool wulpus_is_streaming(void)
{
    return wulpus_active;
}

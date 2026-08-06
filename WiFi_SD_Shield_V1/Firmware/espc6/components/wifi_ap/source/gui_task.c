/*
 * ----------------------------------------------------------------------
 *
 * File: gui_task.c
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

#include "gui_task.h"
#include "dummy_sensor_local.h"
#include "freertos/ringbuf.h"
#define GUI_TAG "[gui_task.c]"

bool gui_connected = false;
int gui_sock = -1;
uint8_t rx_data_from_gui[RX_FROM_GUI_BUF_SIZE] = {0};
uint8_t rx_gui_data_to_fwd[RX_FROM_GUI_BUF_SIZE] = {0};
size_t rx_gui_data_len = 0;
TaskHandle_t rx_gui_task_handle = NULL;

static bool start_reported = false;
static bool stop_reported = false;

/** @brief Block in accept() for a single TCP connection from BioGUI on PORT_LAPTOP. */
esp_err_t bind_to_gui()
{

    // accept the GUI socket
    int ls = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (ls < 0){
            ESP_LOGE(GUI_TAG, "socket errno=%d", errno); 
        vTaskDelete(NULL); 
        return ESP_FAIL;
    }

    int opt = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in a = {0};
    a.sin_family = AF_INET; 
    a.sin_port = htons(PORT_LAPTOP); 
    a.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(ls, (struct sockaddr*)&a, sizeof(a)) != 0 || listen(ls, 1) != 0) {
        ESP_LOGE(GUI_TAG, "bind/listen errno=%d", errno);
        close(ls); 
        vTaskDelete(NULL); 
        return ESP_FAIL;
    }
    ESP_LOGI(GUI_TAG, "Listening on %d", PORT_LAPTOP);

    struct sockaddr_storage sa;
    socklen_t sl = sizeof(sa);
    int s = accept(ls, (struct sockaddr*)&sa, &sl);
    if (s < 0) {
        ESP_LOGW(GUI_TAG, "accept failed errno=%d", errno);
        close(ls);
        return ESP_FAIL;
    }
    ESP_LOGI(GUI_TAG, "Accepted with fd=%d", s);

    /* Only one GUI connection is ever served at a time (accept() above is
     * blocking, single-shot); the listening socket isn't needed again until
     * the next call to bind_to_gui(), which creates a fresh one. */
    close(ls);

    /* keepalive */
    int ka = 1, idle = 5, intv = 5, cnt = 3;
    setsockopt(s, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
    setsockopt(s, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(s, IPPROTO_TCP, TCP_KEEPINTVL, &intv, sizeof(intv));
    setsockopt(s, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

    /* set timeouts to avoid blocking forever */
    struct timeval rcv_to = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));

    struct timeval snd_to = { .tv_sec = 0, .tv_usec = 200 * 1000 }; /* 200 ms */
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));

    /* expose the accepted socket to the rest of the app */
    gui_sock = s;
    gui_connected = true;

    /* Notify other tasks that the GUI socket is bound and available */
    if (g_evt) {
        xEventGroupSetBits(g_evt, B_GUI_SOCKET_BIND);
    }
    return ESP_OK;
}

/** @brief Drain biogap_ringbuf completely, robust against late in-flight enqueues. */
esp_err_t rb_soft_flush(RingbufHandle_t rb)
{
    if (rb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t item_size = 0;
    void *item = NULL;
    size_t items_flushed = 0;
    int empty_rounds = 0;

    /* Keep draining until the ringbuffer stays empty for a few checks.
     * This makes the flush robust against late in-flight enqueues.
     */
    for (int round = 0; round < 64; round++) {
        bool drained_this_round = false;

        while ((item = xRingbufferReceive(rb, &item_size, 0)) != NULL) {
            vRingbufferReturnItem(rb, item);
            items_flushed++;
            drained_this_round = true;
        }

        UBaseType_t items_waiting = 0;
        vRingbufferGetInfo(rb, NULL, NULL, NULL, NULL, &items_waiting);

        if (!drained_this_round && items_waiting == 0) {
            empty_rounds++;
            if (empty_rounds >= 3) {
                ESP_LOGI(GUI_TAG, "Soft-flushed ringbuffer completely, items=%u", (unsigned)items_flushed);
                return ESP_OK;
            }
        } else {
            empty_rounds = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGW(GUI_TAG, "Ringbuffer flush reached retry limit, items=%u", (unsigned)items_flushed);
    return ESP_OK;
}


/** @brief Send all len bytes of buf to sock, retrying through partial writes.
 *
 * A single send() call is allowed to write fewer bytes than requested when
 * the socket's send buffer is under pressure -- that's normal TCP behavior,
 * not an error. Treating such a partial write as success (as a bare send()
 * call does) puts a truncated packet on the wire and permanently shifts the
 * byte alignment of every packet sent afterward, since the receiver has no
 * way to know part of a packet is missing.
 *
 * Once any bytes have actually been committed to the wire for this packet,
 * giving up is no longer a safe way to fail -- those bytes can't be
 * un-sent, so abandoning the rest guarantees a misaligned stream for every
 * packet sent afterward. Retrying costs only time, not correctness, so the
 * retry budget is far more generous after the first byte goes out than
 * before it (nothing lost yet, safe to give up quickly and drop the whole
 * packet). Both are still bounded so a genuinely dead socket eventually
 * gets reported as failed rather than retrying forever.
 */
static int send_all(int sock, const uint8_t *buf, size_t len)
{
    size_t sent_total = 0;
    int attempts = 0;
    const int max_attempts_before_commit = 5;
    const int max_attempts_after_commit = 25;

    while (sent_total < len) {
        int sent = send(sock, buf + sent_total, len - sent_total, 0);
        if (sent < 0) {
            int limit = (sent_total > 0) ? max_attempts_after_commit : max_attempts_before_commit;
            if (++attempts >= limit) {
                ESP_LOGE(GUI_TAG, "send_all gave up after %d attempts (%u/%u bytes sent)",
                         attempts, (unsigned)sent_total, (unsigned)len);
                return -1;
            }
            continue;
        }
        sent_total += (size_t)sent;
        if (sent_total < len) {
            ESP_LOGW(GUI_TAG, "Partial send: %u/%u bytes sent, retrying...", (unsigned)sent_total, (unsigned)len);
        }
    }
    return 0;
}

/** @brief Task to transmit data to the GUI
 * This task is responsible for draining the content of the ringbuffer and sending it to the GUI via WiFi.
*/
void tx_to_gui(void *pvParameters)
{
    uint16_t counter_rcv_prev = 0;
    while(1){
        if(node_state == STATE_STREAMING){
            size_t item_size = 0;
            uint8_t *item = (uint8_t *)xRingbufferReceive(biogap_ringbuf, &item_size, portMAX_DELAY);

            if(item != NULL){

                if(item[0] == WULPUS_HDR_XFER_0 || item[0] == NRF_EXG_HEADER){
                    uint16_t counter = (item[2] << 8) | item[1];
                    if(counter != (counter_rcv_prev + 1)){
                        ESP_LOGW(GUI_TAG, "Missed packet(s): expected counter %d, got %d", counter_rcv_prev + 1, counter);
                    }
                    counter_rcv_prev = counter;
                }

                // Check if we have a WULPUS packet received via BLE
                if(item[0] == WULPUS_HDR_XFER_0 || item[0] == WULPUS_HDR_XFER_1 || item[0] == WULPUS_HDR_XFER_2 || item[0] == WULPUS_HDR_XFER_3){
                    //ESP_LOGI(GUI_TAG, "Received WULPUS packet with header: 0x%02X", item[0]);
                    // Add bytes for TCP packet check
                    uint8_t pckt_tmp[item_size+2];
                    pckt_tmp[0] = BIOGUI_CHECK_HEADER;
                    memcpy(&pckt_tmp[1], item, item_size);
                    pckt_tmp[item_size+1] = BIOGUI_CHECK_TAILER;
                    int sent = send_all(gui_sock, pckt_tmp, item_size+2);
                    if (sent < 0) {
                        ESP_LOGE(GUI_TAG, "Failed to send tmp WULPUS packet to GUI, dropping packet (errno=%d)", errno);
                    }
                }
                
                else{
                    // EXG packet 
                    // send immediately to the gui
                    int sent = send_all(gui_sock, item, item_size);
                    if (sent < 0) {
                        ESP_LOGE(GUI_TAG, "Failed to send data to GUI, dropping packet (errno=%d)", errno);
                    }
                }
                /* biogap_ringbuf is RINGBUF_TYPE_NOSPLIT, which requires items
                 * to be returned in the exact order they were received --
                 * always return here, even on a failed send. Leaving a failed
                 * item unreturned while later items keep getting returned
                 * violates that FIFO order and corrupts the ringbuffer's
                 * internal bookkeeping (this was the actual cause of the
                 * GUI-side counter getting stuck on a stale/garbage value
                 * after a send failure, not a WiFi/TCP framing issue). */
                vRingbufferReturnItem(biogap_ringbuf, (void *)item);
            }
        }

        else{
            vTaskDelay(pdMS_TO_TICKS(100)); // Sleep for a while when not streaming to avoid busy loop
        }
    }

    vTaskDelete(NULL);
}


/**
 * @brief Tear down and reinitialize the SPI slave bus and DMA buffers,
 * guaranteeing an empty transaction queue.
 */
esp_err_t reset_spi_bus_for_restart(void)
{
    esp_err_t ret = free_prequeue_resources();
    ret = spi_slave_free(SPI_HOST_DEVICE);
    ESP_LOGI(GUI_TAG, "Freed pre-queue resources for restart");
    ret = init_nrf_spi_master_esp_slave_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(GUI_TAG, "Failed to re-initialize NRF SPI bus");
        return ret;
    }
    ESP_LOGI(GUI_TAG, "Re-initialized NRF SPI bus for next acquisition");
    current_spi_mode = SPI_MODE_NRF;
    return ESP_OK;
}

/**
 * @brief Finish restart bookkeeping after a STOP: flush the ring buffer,
 * re-arm the SPI bus with idle-content transactions, and clear the
 * STOP/START event bits.
 *
 * Sole caller: enter_stop_quiesce_state() (biogap_read.c), right after
 * reset_spi_bus_for_restart().
 */
esp_err_t prepare_for_restart(){
    ESP_LOGI(GUI_TAG, "Preparing for restart after STOP command");
    // drain completely the ringbuffer
    esp_err_t ret = rb_soft_flush(biogap_ringbuf);
    if (ret != ESP_OK) {
        ESP_LOGE(GUI_TAG, "Failed to soft-flush BIOGAP ringbuffer");
        vTaskDelete(NULL);
        return ret;
    }
    xEventGroupClearBits(g_evt, B_RINGBUFFER_FULL);
    start_reported = false;
    stop_reported = false;
    xEventGroupClearBits(g_evt, B_STOP_CMD_RCV_GUI | B_STOP_CMD_RCV_FORCED | B_STOP_CMD_FWD_TO_BIOGAP | B_STOP_CMD_RPT_PENDING);
    return ESP_OK;
}


/** @brief Task: receives GUI commands and drives the START/STOP/reconnect state machine. */
void rx_from_gui(void *pvParameters)
{
    rx_gui_task_handle = xTaskGetCurrentTaskHandle();
    uint8_t gui_rx_buf[256];
    esp_err_t ret; 
    while(1){
        // Receive data from the GUI 
        int bytes_received = recv(gui_sock, gui_rx_buf, sizeof(gui_rx_buf), 0);

        if (bytes_received==0){
            // The GUI closed the connection. This is expected as part of a normal
            // stop-then-disconnect cycle (and BioGUI may reconnect for the next
            // session), so don't tear down this task -- go back to waiting for a
            // new connection instead. If this happens while still streaming, force
            // a STOP first so the SPI link gets quiesced and the NRF told to stop,
            // instead of leaving it streaming into a socket nobody is reading.
            ESP_LOGW(GUI_TAG, "GUI closed the connection (node_state=%d)", (int)node_state);

            if (node_state == STATE_STREAMING) {
                rx_gui_data_to_fwd[0] = STOP_DUMMY_STREAMING;
                //xEventGroupSetBits(g_evt, B_STOP_CMD_RCV_FORCED);     --> might be the cause of the race
            #if ESP_LOCAL_DUMMY_SENSOR
                /* Local-dummy mode has no equivalent of enter_stop_quiesce_state()
                 * to trigger this on its own, so it's still called directly here. */
                prepare_for_restart_local_dummy();
            #else
                /* Real path: enter_stop_quiesce_state() (biogap_read.c), running in
                 * the read task, picks up B_STOP_CMD_RCV_FORCED and calls
                 * prepare_for_restart() itself once STOP delivery is confirmed.
                 * Do NOT also call it here -- two tasks racing through
                 * free/reinit/allocate on the same SPI bus corrupts its state. */
        #endif
            }
            start_reported = false;
            stop_reported = false;

            close(gui_sock);
            gui_sock = -1;
            gui_connected = false;
            xEventGroupClearBits(g_evt, B_GUI_SOCKET_BIND);

            ESP_LOGI(GUI_TAG, "Waiting for a new GUI connection...");
            if (bind_to_gui() != ESP_OK) {
                ESP_LOGE(GUI_TAG, "Failed to re-bind to GUI, giving up");
                break;
            }
            continue;
        }
        if (bytes_received<0){
            // Handle Error Code 11 (EAGAIN / EWOULDBLOCK) -> means that no data was available right now
            // ESP_LOGI(GUI_TAG, "No data received from GUI, errno=%d", errno);
            if (errno == EAGAIN || errno == EWOULDBLOCK) {        
                // Add a small delay to avoid busy-waiting
                vTaskDelay(pdMS_TO_TICKS(1000));            // this is not critical, we can wait and let other tasks run
                continue;
            }
            break; 
        }

        else{
            ret = parse_gui_command(&gui_rx_buf, bytes_received);
            if (ret != ESP_OK) {
                ESP_LOGW(GUI_TAG, "Failed to parse GUI command");
                continue; // ignore this command and keep listening for new ones
            }
            if (!start_reported && (xEventGroupGetBits(g_evt) & B_START_CMD_RCV)) {
                xEventGroupClearBits(g_evt, B_START_CMD_RCV);
                start_reported = true;
                stop_reported = false;
            }

            if (xEventGroupGetBits(g_evt) & B_STOP_CMD_RPT_PENDING) {
                xEventGroupSetBits(g_evt, B_STOP_CMD_RCV_GUI);
                xEventGroupClearBits(g_evt, B_STOP_CMD_RPT_PENDING);
                ESP_LOGI(GUI_TAG, "STOP command received from GUI, preparing for restart");
                //node_state = STATE_IDLE;
                stop_reported = true;
                start_reported = false;
#if ESP_LOCAL_DUMMY_SENSOR
                /* Local-dummy mode has no equivalent of enter_stop_quiesce_state()
                 * to trigger this on its own, so it's still called directly here. */
                prepare_for_restart_local_dummy();
#else
                /* Real path: enter_stop_quiesce_state() (biogap_read.c), running in
                 * the read task, picks up B_STOP_CMD_RCV_GUI and calls
                 * prepare_for_restart() itself once STOP delivery is confirmed.
                 * Do NOT also call it here -- two tasks racing through
                 * free/reinit/allocate on the same SPI bus corrupts its state. */
#endif
            }

        }
        
    }

// cleanup:
//     ESP_LOGI(GUI_TAG, "RX task cleaning up...");
//     shutdown(gui_sock, SHUT_RDWR);
//     close(gui_sock);
//     gui_sock = -1;
//     gui_connected = false;
    vTaskDelete(NULL);
}
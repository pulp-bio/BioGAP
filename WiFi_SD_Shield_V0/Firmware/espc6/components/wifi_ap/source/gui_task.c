#include "gui_task.h"
#include "freertos/ringbuf.h"
#define GUI_TAG "[gui_task.c]" 

bool gui_connected = false;
int gui_sock = -1;
uint8_t rx_data_from_gui[RX_FROM_GUI_BUF_SIZE] = {0}; 

static TaskHandle_t rx_gui_task_handle = NULL;
static esp_timer_handle_t start_sim_timer_handle = NULL;
static esp_timer_handle_t stop_sim_timer_handle = NULL;



static void start_sim_timer_callback(void *arg)
{
    rx_data_from_gui[0] = START_DUMMY_STREAMING;
    xEventGroupSetBits(g_evt, B_START_CMD_RCV);
    ESP_LOGI(GUI_TAG, "Simulated receiving START command from GUI after 10 seconds");
    if (rx_gui_task_handle != NULL) {
        xTaskNotifyGive(rx_gui_task_handle);
    }
}

static void stop_sim_timer_callback(void *arg)
{
    rx_data_from_gui[0] = STOP_DUMMY_STREAMING;
    xEventGroupSetBits(g_evt, B_STOP_CMD_RPT_PENDING);
    if (rx_gui_task_handle != NULL) {
        xTaskNotifyGive(rx_gui_task_handle);
    }
    ESP_LOGI(GUI_TAG, "STOP timer");
}

esp_err_t bind_to_gui(void *pv)
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

static esp_err_t rb_soft_flush(RingbufHandle_t rb)
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

// for now, just a tmp implementation
void rx_from_gui(void *pvParameters)
{
    bool acquisition_start_and_stop_simulated = false;
    // Keeps on listening from the GUI
    // If stop command received -> forward stop command to the nodes
    // Receive data
    rx_gui_task_handle = xTaskGetCurrentTaskHandle();
    bool start_reported = false;
    bool stop_reported = false;
    
    if (start_sim_timer_handle == NULL) {
        esp_timer_create_args_t start_timer_args = {
            .callback = start_sim_timer_callback,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "gui_start_sim",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&start_timer_args, &start_sim_timer_handle) != ESP_OK) {
            ESP_LOGE(GUI_TAG, "Failed to create start simulator timer");
            vTaskDelete(NULL);
            return;
        }
    }

    if (stop_sim_timer_handle == NULL) {
        esp_timer_create_args_t stop_timer_args = {
            .callback = stop_sim_timer_callback,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "gui_stop_sim",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&stop_timer_args, &stop_sim_timer_handle) != ESP_OK) {
            ESP_LOGE(GUI_TAG, "Failed to create stop simulator timer");
            vTaskDelete(NULL);
            return;
        }
    }

    if (esp_timer_start_once(start_sim_timer_handle, 5 * 1000000LL) != ESP_OK) {
        ESP_LOGE(GUI_TAG, "Failed to start start simulator timer");
        vTaskDelete(NULL);
        return;
    }

    if (esp_timer_start_once(stop_sim_timer_handle,10  * 1000000LL) != ESP_OK) {
        ESP_LOGE(GUI_TAG, "Failed to start stop simulator timer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(GUI_TAG, "Dummy GUI timers armed: START in 10 s, STOP in 20 s");

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!start_reported && (xEventGroupGetBits(g_evt) & B_START_CMD_RCV)) {
            xEventGroupClearBits(g_evt, B_START_CMD_RCV);
            start_reported = true;
            stop_reported = false;
            ESP_LOGI(GUI_TAG, "Simulated receiving START command from GUI after 10 seconds");
        }

        if (xEventGroupGetBits(g_evt) & B_STOP_CMD_RPT_PENDING) {
            ESP_LOGI(GUI_TAG, "Simulated receiving STOP command");
            rx_data_from_gui[0] = STOP_DUMMY_STREAMING;         // will be replaced by the actual STOP command later
            //sendbuf_persistent[PACKET_SZ - 1] = ESP_EXG_TAILER;
            xEventGroupSetBits(g_evt, B_STOP_CMD_RCV_GUI);
            xEventGroupClearBits(g_evt, B_STOP_CMD_RPT_PENDING);
            stop_reported = true;
            start_reported = false;
            acquisition_start_and_stop_simulated = true;
        }

        // check if we want to do it again
        if(acquisition_start_and_stop_simulated){
            acquisition_start_and_stop_simulated = false;
            // REALLOCATE the resources for the next acquisition since the previous ones were freed during the STOP quiesce
            xEventGroupWaitBits(g_evt, B_STOP_CMD_FWD_TO_BIOGAP, pdFALSE, pdFALSE, portMAX_DELAY);
            esp_err_t ret = free_prequeue_resources();
            // reinitialize completely the SPI slave
            ret = spi_slave_free(SPI_HOST_DEVICE);
            ESP_LOGI(GUI_TAG, "Freed pre-queue resources after STOP command");
            // drain completely the ringbuffer
            ret = rb_soft_flush(biogap_ringbuf);
            if (ret != ESP_OK) {
                ESP_LOGE(GUI_TAG, "Failed to soft-flush BIOGAP ringbuffer");
                vTaskDelete(NULL);
                return;
            }
            xEventGroupClearBits(g_evt, B_RINGBUFFER_FULL);
            ESP_LOGI(GUI_TAG, "BIOGAP Ringbuffer created with size %d bytes", RINGBUFF_SIZE);
            // Set MUX to NRF mode (LOW = 0)

            //gpio_reset_pin(NRF_SPI_MOSI_GPIO);
            //gpio_reset_pin(NRF_SPI_MISO_GPIO);
            //gpio_reset_pin(NRF_SPI_SCLK_GPIO);
            // Small delay to let signal settle
            esp_rom_delay_us(100);
            ret = init_nrf_spi_master_esp_slave_bus();
            if (ret != ESP_OK) {
                ESP_LOGE(GUI_TAG, "Failed to re-initialize NRF SPI bus"); 
                vTaskDelete(NULL);
                return;
            }
            ESP_LOGI(GUI_TAG, "Re-initialized NRF SPI bus for next acquisition");
            current_spi_mode = SPI_MODE_NRF;
            ret = allocate_prequeue_resources();
            ESP_LOGI(GUI_TAG, "Re-allocated pre-queue resources for next acquisition");
            if (ret != ESP_OK) {
                ESP_LOGE(GUI_TAG, "Failed to allocate pre-queue resources for new acquisition");
                vTaskDelete(NULL);
                return;
            }
            start_reported = false;
            stop_reported = false;
            xEventGroupClearBits(g_evt, B_STOP_CMD_RCV_GUI | B_STOP_CMD_FWD_TO_BIOGAP | B_STOP_CMD_RPT_PENDING);
            if (esp_timer_start_once(start_sim_timer_handle, 10 * 1000000LL) != ESP_OK) {
                ESP_LOGE(GUI_TAG, "Failed to restart start simulator timer");
                vTaskDelete(NULL);
                return;
            }

            if (esp_timer_start_once(stop_sim_timer_handle, 15 * 1000000LL) != ESP_OK) {
                ESP_LOGE(GUI_TAG, "Failed to restart stop simulator timer");
                vTaskDelete(NULL);
                return;
            }
            xEventGroupClearBits(g_evt, B_STOP_CMD_RPT_PENDING);
            ESP_LOGI(GUI_TAG, "Dummy GUI timers re-armed: START in 10 s, STOP in 20 s");
        }
    }
}

/** @brief Task to transmit data to the GUI 
 * This task is responsible for draining the content of the ringbuffer and sending it to the GUI via WiFi.
*/
void tx_to_gui(void *pvParameters)
{

    while(1){
        if(node_state == STATE_STREAMING){
            size_t item_size = 0;
            uint8_t *item = (uint8_t *)xRingbufferReceive(biogap_ringbuf, &item_size, portMAX_DELAY);

            if(item !=NULL){
                // send immediately to the gui
                //int sent = send(gui_sock, item, item_size, 0);
                int sent = 1; //hardcoded for now
                if (sent < 0) {
                    // item will stay in the ringbuffer since we havent' return it yet. 
                    ESP_LOGE(GUI_TAG, "Failed to send data to GUI");
                }
                else{
                    // remove the item from the ringbuffer only if it was sent successfully
                    vRingbufferReturnItem(biogap_ringbuf, (void *)item);
                }
            }
        }

        else{
            vTaskDelay(pdMS_TO_TICKS(100)); // Sleep for a while when not streaming to avoid busy loop
        }
    }

    vTaskDelete(NULL);
}
// void rx_from_gui(void *pvParameters)
// {
//     // Keeps on listening from the GUI
//     // If stop command received -> forward stop command to the nodes
//     // Receive data
//     uint8_t gui_rx_buf[256];

//     while(1){
//         // Receive data from the GUI 
//         int n = recv(gui_sock, gui_rx_buf, sizeof(gui_rx_buf), 0);

//         if (n==0){
//             // This mean the GUI socket has been closed
//             ESP_LOGW(GUI_TAG, "RX with GUI closed");
//             break; // go out 
//         }
//         if (n<0){

//             // Handle Error Code 11 (EAGAIN / EWOULDBLOCK) -> means that no data was available right now
//             if (errno == EAGAIN || errno == EWOULDBLOCK) {        
//                 // Add a small delay to avoid busy-waiting
//                 vTaskDelay(pdMS_TO_TICKS(2000));            // this is not critical, we can wait and let other tasks run
//                 continue;
//             }
//             break; 
//         }
//         else{
//             // parse the data received from the GUI and check what command it is
//             for (int shift = 0; shift < n; shift++) {
//             if (memcmp(&gui_rx_buf[shift], STOP_STREAMING_CMD, STOP_CMD_LEN) == 0) {
//                 ESP_LOGI(GUI_TAG, "Received STOP command from GUI");
//                 xEventGroupSetBits(g_evt, B_STOP_STREAM);
//                 goto cleanup;
//                 }
//             }
//         }
        
//     }

// cleanup:
//     ESP_LOGI(GUI_TAG, "RX task cleaning up...");
//     shutdown(gui_sock, SHUT_RDWR);
//     close(gui_sock);
//     gui_sock = -1;
//     gui_connected = false;
//     xEventGroupClearBits(g_evt, B_GUI_SOCKET_BIND);

//     vTaskDelete(NULL);
// }
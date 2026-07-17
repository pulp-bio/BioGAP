#ifndef GUI_TASK_H
#define GUI_TASK_H

#include "softap_main.h"
#include "biogap.h"

#define RX_FROM_GUI_BUF_SIZE 20


/**  @brief  Buffer to store incoming commands from the GUI*/
extern uint8_t rx_data_from_gui[RX_FROM_GUI_BUF_SIZE]; 
extern uint8_t rx_gui_data_to_fwd[RX_FROM_GUI_BUF_SIZE];

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
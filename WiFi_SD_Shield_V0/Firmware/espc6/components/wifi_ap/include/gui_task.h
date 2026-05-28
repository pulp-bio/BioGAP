#ifndef GUI_TASK_H
#define GUI_TASK_H

#include "softap_main.h"

#define RX_FROM_GUI_BUF_SIZE 20

/**  @brief  Buffer to store incoming commands from the GUI*/
extern uint8_t rx_data_from_gui[RX_FROM_GUI_BUF_SIZE]; 

/** @brief Binds to the GUI socket */
esp_err_t bind_to_gui(void *pv);

/** @brief Task to receive data from the GUI */
void rx_from_gui(void *pv);

/** @brief Task to send data to the GUI */
void tx_to_gui(void *pv);

#endif // GUI_TASK_H
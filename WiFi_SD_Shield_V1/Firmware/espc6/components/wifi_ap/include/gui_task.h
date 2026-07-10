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

/** @brief Prepares the system for restart after a stop command */
esp_err_t prepare_for_restart();

#endif // GUI_TASK_H


/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2020 Prevas A/S
 * Copyright (c) 2025 ETH Zurich
 *
 * File: bluestooth.c
 *
 * Last edited: 23.07.2025
 *
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
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/drivers/sensor.h>

#include <bluetooth/services/nsms.h>
#include <bluetooth/services/nus.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>

#include "ble_appl.h"
#include "ads_appl.h"


LOG_MODULE_REGISTER(main_bluetooth, LOG_LEVEL_DBG);

#include <zephyr/settings/settings.h>

#define TEST_DATA_LEN 240
#define PATTERN "Hello BLE NUS Test!\n"
#define PATTERN_LEN (sizeof(PATTERN) - 1)
#define NUM_TRANSMISSIONS 100

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)
#define PRIORITY_BLE_THREAD 7

#define THROUGHPUT_CONFIG_TIMEOUT K_SECONDS(20)



#define INTERVAL_MIN   0x6  // Minimum interval, 7.5 ms 
#define INTERVAL_MAX   0x6  // Maximum interval, 7.5 ms 

static K_SEM_DEFINE(throughput_sem, 0, 1);

static struct k_work advertise_work;


struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[CONFIG_BT_NUS_UART_BUFFER_SIZE];
	uint16_t len;
};

static uint8_t test_data[TEST_DATA_LEN];

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static struct bt_gatt_exchange_params exchange_params;
static struct bt_le_conn_param *conn_param =
	BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);

struct bt_conn_le_phy_param *phy = BT_CONN_LE_PHY_PARAM_2M;
struct bt_conn_le_data_len_param *data_len = BT_LE_DATA_LEN_PARAM_MAX;

static volatile bool data_length_req;





static K_SEM_DEFINE(ble_init_ok, 0, 1);

/*
static const struct device *const bme_dev = DEVICE_DT_GET_ONE(bosch_bme680);
*/
static void update_phy(struct bt_conn *conn);
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params);
static void update_data_length(struct bt_conn *conn);
static void update_mtu(struct bt_conn *conn);
static void le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info);
//static int connection_configuration_set(const struct bt_le_conn_param *conn_param,
//			const struct bt_conn_le_phy_param *phy,
//			const struct bt_conn_le_data_len_param *data_len);
void measure_throughput(void);
void print_ble_conn_info(void);
void init_test_data(void);

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
	memcpy(ble_data_available.data,data,len);
	ble_data_available.available = true;
    ble_data_available.size = len;
	k_sem_give(&ble_data_received);

}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
    .sent = NULL,
    .send_enabled = NULL,
};


#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
			level, err);
	}
}
#endif


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void advertise(struct k_work *work) {
  int err;

  bt_le_adv_stop();

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    settings_load();
  }

  err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
	}

  LOG_INF("Bluetooth initialized");

//k_sem_give(&ble_init_ok);

if (IS_ENABLED(CONFIG_SETTINGS)) {
	settings_load();
}

err = bt_nus_init(&nus_cb);
if (err) {
	LOG_ERR("Failed to initialize UART service (err: %d)", err);
	return 0;
}

err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
				ARRAY_SIZE(sd));
if (err) {
	LOG_ERR("Advertising failed to start (err %d)", err);
	return 0;
}

  LOG_INF("Bluetooth advertising started");
}

static void connected(struct bt_conn *conn, uint8_t err) {
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	struct bt_conn_info info = {0};
	current_conn = bt_conn_ref(conn);

	/* *** Request new connection parameters right away *** */
	int ret = bt_conn_le_param_update(conn, conn_param);
	if (ret) {
	  LOG_ERR("Conn param update failed: %d", ret);
	}


	update_phy(conn);
	update_data_length(conn);
	update_mtu(conn);


	k_sem_give(&ble_init_ok);


}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	//k_sem_take(&ble_init_ok, K_FOREVER);
  	
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
  k_work_submit(&advertise_work);
}


static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}


static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	LOG_INF("Connection parameters update request received.\n");
	LOG_INF("Minimum interval: %d, Maximum interval: %d\n",
	       param->interval_min, param->interval_max);
	LOG_INF("Latency: %d, Timeout: %d\n", param->latency, param->timeout);

	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	LOG_INF("___CB___ Connection parameters updated.\n"
	       " interval: %d, latency: %d, timeout: %d\n",
	       interval, latency, timeout);

	k_sem_give(&throughput_sem);
}

static void le_phy_updated(struct bt_conn *conn,
			   struct bt_conn_le_phy_info *param)
{
	LOG_INF("___CB___ LE PHY updated: TX PHY %s, RX PHY %s\n",
	       phy2str(param->tx_phy), phy2str(param->rx_phy));

	k_sem_give(&throughput_sem);
}


static void le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    uint16_t tx_len     = info->tx_max_len; 
    uint16_t tx_time    = info->tx_max_time;
    uint16_t rx_len     = info->rx_max_len;
    uint16_t rx_time    = info->rx_max_time;
    LOG_INF("___CB___  Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}


static void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params)
{
	LOG_INF("___CB___ MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if (!att_err) {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}

static void update_phy(struct bt_conn *conn)
{
    int err;
    const struct bt_conn_le_phy_param preferred_phy = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
    };
    err = bt_conn_le_phy_update(conn, &preferred_phy);
    if (err) {
        LOG_ERR("bt_conn_le_phy_update() returned %d", err);
    }
}

static void update_data_length(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    err = bt_conn_le_data_len_update(conn, &my_data_len);
    if (err) {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

static void update_mtu(struct bt_conn *conn)
{
    int err;
    exchange_params.func = exchange_func;

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	.le_phy_updated = le_phy_updated,
	.le_data_len_updated = le_data_length_updated,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

static void bt_ready(int err) {
  if (err != 0) {
    LOG_ERR("Bluetooth failed to initialise: %d", err);
  } else {
    k_work_submit(&advertise_work);
  }
}

void start_bluetooth_adverts(void) {
  int rc;

  k_work_init(&advertise_work, advertise);
  rc = bt_enable(bt_ready);

  if (rc != 0) {
    LOG_ERR("Bluetooth enable failed: %d", rc);
  }
}

void update_status(struct sensor_value *temp, struct sensor_value *press, struct sensor_value *humidity,
                   struct sensor_value *gas_res) {
  char temp_str[20];
  char press_str[20];
  char humidity_str[20];
  char gas_res_str[20];

  snprintf(temp_str, sizeof(temp_str), "%d.%06d", temp->val1, temp->val2);
  snprintf(press_str, sizeof(press_str), "%d.%06d", press->val1, press->val2);
  snprintf(humidity_str, sizeof(humidity_str), "%d.%06d", humidity->val1, humidity->val2);
  snprintf(gas_res_str, sizeof(gas_res_str), "%d.%06d", gas_res->val1, gas_res->val2);

}


void ble_write_thread(void)
{
	// Don't go any further until BLE is initialized 
	k_sem_take(&ble_init_ok, K_FOREVER);
	// TODO, add semaphore again
	LOG_INF("Hello from ble_write_thread \n");
	k_sleep(K_MSEC(5000));

	// TODO: Remove again
	Set_ADS_Function(READ);
	
	//LOG_INF("Initiating throughput optimization \n");
	//int err;
	//err = connection_configuration_set(conn_param, phy, data_len);
	//if (err) {
	//	return err;
	//}
	//LOG_INF("Done with throughput optimization \n");
	
	// Make sure that all BLE procedures are finished.
	//k_sleep(K_MSEC(500));

	for (;;) {
		
		//int err;
		//err = bt_nus_send(NULL, test_data, sizeof(test_data));
        //if (err) {
        //    LOG_ERR("Failed to send data over BLE connection (err: %d)", err);
        //}
		//print_ble_conn_info();
		k_sleep(K_MSEC(100));
		//k_yield();
		//measure_throughput();

	}

	for (;;) {
		// Wait indefinitely for data to be sent over bluetooth 
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}
		

		k_free(buf);
	}
}

/*
static int connection_configuration_set(const struct bt_le_conn_param *conn_param,
			const struct bt_conn_le_phy_param *phy,
			const struct bt_conn_le_data_len_param *data_len)
{
	LOG_INF("connection_configuration_set start\n");
	int err;
	struct bt_conn_info info = {0};

	err = bt_conn_get_info(current_conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d\n", err);
		return err;
	}
	LOG_INF("bt_conn_get_info returned\n");

		// Connection role
	LOG_INF("Connection role: %s", info.role == BT_CONN_ROLE_CENTRAL ? "central" : "peripheral");

	// Connection interval
	LOG_INF("Conn. interval is %u units\n", info.le.interval);

	// Latency
	LOG_INF("Conn. latency is %u", info.le.latency);

	// Supervision timeout
	LOG_INF("Supervision timeout is %u units", info.le.timeout);

	// Address info
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(info.le.src, addr, sizeof(addr));
	LOG_INF("Source address: %s", addr);

	bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));
	LOG_INF("Destination address: %s", addr);

	// PHY info (if available)
	#if defined(CONFIG_BT_CONN_INFO_EXT)
	LOG_INF("TX PHY: %s", info.le.tx_phy == BT_GAP_LE_PHY_1M ? "LE 1M" :
	                       info.le.tx_phy == BT_GAP_LE_PHY_2M ? "LE 2M" :
	                       info.le.tx_phy == BT_GAP_LE_PHY_CODED ? "LE Coded" : "Unknown");

	LOG_INF("RX PHY: %s", info.le.rx_phy == BT_GAP_LE_PHY_1M ? "LE 1M" :
	                       info.le.rx_phy == BT_GAP_LE_PHY_2M ? "LE 2M" :
	                       info.le.rx_phy == BT_GAP_LE_PHY_CODED ? "LE Coded" : "Unknown");
	#endif

	// Data length (if available)
	#if defined(CONFIG_BT_DATA_LEN_UPDATE)
	LOG_INF("TX max length: %u bytes", info.le.data_len->tx_max_len);
    LOG_INF("TX max time: %u us", info.le.data_len->tx_max_time);
    LOG_INF("RX max length: %u bytes", info.le.data_len->rx_max_len);
    LOG_INF("RX max time: %u us", info.le.data_len->rx_max_time);
	#endif




	err = bt_conn_le_phy_update(current_conn, phy);
	if (err) {
		LOG_ERR("PHY update failed: %d\n", err);
		return err;
	}
	LOG_INF("bt_conn_le_phy_update returned\n");

	LOG_INF("PHY update pending\n");
	err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
	if (err) {
		LOG_ERR("PHY update timeout\n");
		return err;
	}
	LOG_INF("PHY update pending got the semaphore\n");


	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(current_conn, conn_param);
		if (err) {
			LOG_ERR("Connection parameters update failed: %d\n",
				    err);
			return err;
		}
		

		LOG_INF("Connection parameters update pending\n");
		err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
		if (err) {
			LOG_ERR("Connection parameters update timeout\n");
			return err;
		}
	}

	if (info.le.data_len->tx_max_len != data_len->tx_max_len) {
		data_length_req = true;

		err = bt_conn_le_data_len_update(current_conn, data_len);
		if (err) {
			LOG_ERR("LE data length update failed: %d\n",
				    err);
			return err;
		}
		LOG_INF("LE Data length update pending\n");
		err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
		if (err) {
			LOG_ERR("LE Data Length update timeout\n");
			return err;
		}
	}

	return 0;
}
*/

void print_ble_conn_info(void)
{
	k_sleep(K_MSEC(2000));
	int err;
	struct bt_conn_info info = {0};

	err = bt_conn_get_info(current_conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d\n", err);
		return err;
	}
	//#if defined(CONFIG_BT_DATA_LEN_UPDATE)
	LOG_INF("TX max length: %u bytes", info.le.data_len->tx_max_len);
	LOG_INF("TX max time: %u us", info.le.data_len->tx_max_time);
	LOG_INF("RX max length: %u bytes", info.le.data_len->rx_max_len);
	LOG_INF("RX max time: %u us", info.le.data_len->rx_max_time);
	// Connection interval
	LOG_INF("Conn. interval is %u units\n", info.le.interval);
	// Latency
	LOG_INF("Conn. latency is %u", info.le.latency);
	// Supervision timeout
	LOG_INF("Supervision timeout is %u units", info.le.timeout);

	//#endif
}

void measure_throughput(void)
{
    uint32_t start_time, end_time;
    size_t total_bytes_sent = 0;
    int transmissions = 0;

	init_test_data();

    // Get the starting time in milliseconds
    start_time = k_uptime_get_32();

    for (transmissions = 0; transmissions < NUM_TRANSMISSIONS; ++transmissions) {

        // Send data over Bluetooth
        if (bt_nus_send(NULL, test_data, sizeof(test_data))) {
            LOG_WRN("Failed to send data over BLE connection");
        } else {
            total_bytes_sent += sizeof(test_data); // Accumulate total bytes sent
        }
		//k_sleep(K_MSEC(10)); // Adjust delay as needed

    }

    // Get the end time in milliseconds
    end_time = k_uptime_get_32();

    // Calculate the time taken in seconds and throughput
    uint32_t time_taken_ms = end_time - start_time;
    float time_taken_s = time_taken_ms / 1000.0;
    float throughput = 8 * total_bytes_sent / time_taken_s / 1000000; // Bytes per second

    LOG_INF("Time taken for %d transmissions: %u ms", NUM_TRANSMISSIONS, time_taken_ms);
	LOG_INF("Total bytes sent: %u", total_bytes_sent);
	LOG_INF("Throughput: %.2f Mbit/s", throughput);
}


void init_test_data(void)
{
    size_t offset = 0;

    // Fill test_data with the repeating pattern until TEST_DATA_LEN is reached
    while (offset < TEST_DATA_LEN) {
        size_t copy_len = (offset + PATTERN_LEN <= TEST_DATA_LEN) ? PATTERN_LEN : (TEST_DATA_LEN - offset);
        memcpy(&test_data[offset], PATTERN, copy_len);
        offset += copy_len;
    }
}


void send_data_ble(char *data_array, int16_t length)
{
	// print the data to be sent
	LOG_INF("Sending data over BLE: %s", data_array);
	
    //k_msgq_put(&send_msgq, &data);
    if (bt_nus_send(NULL, data_array, length)) {
		LOG_WRN("Failed to send data over BLE connection");
	} 
}

K_THREAD_DEFINE(ble_write_thread_id, CONFIG_BT_NUS_THREAD_STACK_SIZE, ble_write_thread, NULL, NULL, NULL, PRIORITY_BLE_THREAD, 0, 0);




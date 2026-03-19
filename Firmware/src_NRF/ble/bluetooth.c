/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2020 Prevas A/S
 * Copyright (c) 2025 ETH Zurich
 *
 * File: bluetooth.c
 *
 * Last edited: 05.12.2025
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

/**
 * @file bluetooth.c
 * @brief Bluetooth Low Energy (BLE) Communication Implementation
 *
 * Implements BLE functionality for the SENSEI platform including:
 * - Nordic UART Service (NUS) for data streaming
 * - Connection parameter optimization for high throughput
 * - Packet statistics for debugging transmission issues
 *
 * @see bluetooth.h for public API documentation
 */

#include "ble/bluetooth.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/drivers/sensor.h>

#include <bluetooth/services/nsms.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "afe/ads_appl.h"
#include "ble/ble_appl.h"

LOG_MODULE_REGISTER(main_bluetooth, LOG_LEVEL_DBG);

/*==============================================================================
 * Private Definitions
 *============================================================================*/

/** @brief Test data length for throughput measurement */
#define TEST_DATA_LEN 240

/** @brief Test pattern for throughput measurement */
#define PATTERN "Hello BLE NUS Test!\n"
#define PATTERN_LEN (sizeof(PATTERN) - 1)

/** @brief Number of transmissions for throughput test */
#define NUM_TRANSMISSIONS 100

/** @brief Device name from Kconfig */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/** @brief BLE write thread priority */
#define PRIORITY_BLE_THREAD 7

/** @brief Timeout for throughput configuration */
#define THROUGHPUT_CONFIG_TIMEOUT K_SECONDS(20)

/** @brief Connection interval in 1.25ms units (7.5ms for high throughput) */
#define INTERVAL_MIN 0x6
#define INTERVAL_MAX 0x6

/*==============================================================================
 * Private Variables - Packet Statistics
 *============================================================================*/

/** @brief EEG packets successfully sent (header 0x55) */
static volatile uint32_t ble_eeg_packets_sent = 0;

/** @brief MIC packets successfully sent (header 0xAA) */
static volatile uint32_t ble_mic_packets_sent = 0;

/** @brief IMU packets successfully sent (header 0x56) */
static volatile uint32_t ble_imu_packets_sent = 0;

/** @brief Other/unknown packets successfully sent */
static volatile uint32_t ble_other_packets_sent = 0;

/** @brief Packets that failed to send */
static volatile uint32_t ble_packets_failed = 0;

/*==============================================================================
 * Private Variables - Connection Management
 *============================================================================*/

/** @brief Semaphore for throughput measurement synchronization */
static K_SEM_DEFINE(throughput_sem, 0, 1);

/** @brief Work item for advertising restart */
static struct k_work advertise_work;

/** @brief Test data buffer for throughput measurement */
static uint8_t test_data[TEST_DATA_LEN];

/** @brief Current active BLE connection */
static struct bt_conn *current_conn;

/** @brief Connection used for authentication (if security enabled) */
static struct bt_conn *auth_conn;

/** @brief GATT MTU exchange parameters */
static struct bt_gatt_exchange_params exchange_params;

/** @brief Connection parameters for high throughput (7.5ms interval) */
static struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);

struct bt_conn_le_phy_param *phy = BT_CONN_LE_PHY_PARAM_2M;
struct bt_conn_le_data_len_param *data_len = BT_LE_DATA_LEN_PARAM_MAX;

static volatile bool data_length_req;

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static void update_phy(struct bt_conn *conn);
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params);
static void update_data_length(struct bt_conn *conn);
static void update_mtu(struct bt_conn *conn);
static void le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info);
static void init_test_data(void);

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
  memcpy(ble_data_available.data, data, len);
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
static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (!err) {
    LOG_INF("Security changed: %s level %u", addr, level);
  } else {
    LOG_WRN("Security failed: %s level %u err %d", addr, level, err);
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

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    settings_load();
  }

  err = bt_nus_init(&nus_cb);
  if (err) {
    LOG_ERR("Failed to initialize UART service (err: %d)", err);
    return;
  }

  err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    LOG_ERR("Advertising failed to start (err %d)", err);
    return;
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

static const char *phy2str(uint8_t phy) {
  switch (phy) {
  case 0:
    return "No packets";
  case BT_GAP_LE_PHY_1M:
    return "LE 1M";
  case BT_GAP_LE_PHY_2M:
    return "LE 2M";
  case BT_GAP_LE_PHY_CODED:
    return "LE Coded";
  default:
    return "Unknown";
  }
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param) {
  LOG_INF("Connection parameters update request received.\n");
  LOG_INF("Minimum interval: %d, Maximum interval: %d\n", param->interval_min, param->interval_max);
  LOG_INF("Latency: %d, Timeout: %d\n", param->latency, param->timeout);

  return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout) {
  LOG_INF("___CB___ Connection parameters updated.\n"
          " interval: %d, latency: %d, timeout: %d\n",
          interval, latency, timeout);

  k_sem_give(&throughput_sem);
}

static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param) {
  LOG_INF("___CB___ LE PHY updated: TX PHY %s, RX PHY %s\n", phy2str(param->tx_phy), phy2str(param->rx_phy));

  k_sem_give(&throughput_sem);
}

static void le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info) {
  uint16_t tx_len = info->tx_max_len;
  uint16_t tx_time = info->tx_max_time;
  uint16_t rx_len = info->rx_max_len;
  uint16_t rx_time = info->rx_max_time;
  LOG_INF("___CB___  Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}

static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params) {
  LOG_INF("___CB___ MTU exchange %s", att_err == 0 ? "successful" : "failed");
  if (!att_err) {
    uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3; // 3 bytes used for Attribute headers.
    LOG_INF("New MTU: %d bytes", payload_mtu);
  }
}

static void update_phy(struct bt_conn *conn) {
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

static void update_data_length(struct bt_conn *conn) {
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

static void update_mtu(struct bt_conn *conn) {
  int err;
  exchange_params.func = exchange_func;

  err = bt_gatt_exchange_mtu(conn, &exchange_params);
  if (err) {
    LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
  }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
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

/*==============================================================================
 * Public Functions - Initialization & Connection
 *============================================================================*/

void start_bluetooth_adverts(void) {
  int rc;

  k_work_init(&advertise_work, advertise);
  rc = bt_enable(bt_ready);

  if (rc != 0) {
    LOG_ERR("Bluetooth enable failed: %d", rc);
  }
}


/*==============================================================================
 * Private Functions - BLE Write Thread
 *============================================================================*/

void ble_write_thread(void) {
  LOG_INF("BLE write thread waiting for initialization");
  k_sem_take(&ble_init_ok, K_FOREVER);
  LOG_INF("BLE write thread started after initialization. Wait 5s before sending data...");
  k_sleep(K_MSEC(5000));
  LOG_INF("Now you can send data over BLE NUS");

  for (;;) {
    k_sleep(K_MSEC(100));
  }
}

/*==============================================================================
 * Public Functions - Diagnostics
 *============================================================================*/

void print_ble_conn_info(void) {
  int err;
  struct bt_conn_info info = {0};

  err = bt_conn_get_info(current_conn, &info);
  if (err) {
    LOG_ERR("Failed to get connection info %d", err);
    return;
  }

  LOG_INF("TX max length: %u bytes", info.le.data_len->tx_max_len);
  LOG_INF("TX max time: %u us", info.le.data_len->tx_max_time);
  LOG_INF("RX max length: %u bytes", info.le.data_len->rx_max_len);
  LOG_INF("RX max time: %u us", info.le.data_len->rx_max_time);
  LOG_INF("Conn. interval is %u units", info.le.interval);
  LOG_INF("Conn. latency is %u", info.le.latency);
  LOG_INF("Supervision timeout is %u units", info.le.timeout);
}

void measure_throughput(void) {
  uint32_t start_time, end_time;
  size_t total_bytes_sent = 0;
  int transmissions = 0;

  init_test_data();

  start_time = k_uptime_get_32();

  for (transmissions = 0; transmissions < NUM_TRANSMISSIONS; ++transmissions) {
    if (bt_nus_send(NULL, test_data, sizeof(test_data))) {
      LOG_WRN("Failed to send data over BLE connection");
    } else {
      total_bytes_sent += sizeof(test_data);
    }
  }

  end_time = k_uptime_get_32();

  uint32_t time_taken_ms = end_time - start_time;
  float time_taken_s = time_taken_ms / 1000.0;
  float throughput = 8 * total_bytes_sent / time_taken_s / 1000000;

  LOG_INF("Time taken for %d transmissions: %u ms", NUM_TRANSMISSIONS, time_taken_ms);
  LOG_INF("Total bytes sent: %u", total_bytes_sent);
  LOG_INF("Throughput: %.2f Mbit/s", throughput);
}

/*==============================================================================
 * Private Functions - Test Helpers
 *============================================================================*/

/**
 * @brief Initialize test data buffer with repeating pattern
 */
static void init_test_data(void) {
  size_t offset = 0;

  while (offset < TEST_DATA_LEN) {
    size_t copy_len = (offset + PATTERN_LEN <= TEST_DATA_LEN) ? PATTERN_LEN : (TEST_DATA_LEN - offset);
    memcpy(&test_data[offset], PATTERN, copy_len);
    offset += copy_len;
  }
}

/*==============================================================================
 * Public Functions - Data Transmission
 *============================================================================*/

void send_data_ble(char *data_array, int16_t length) {
  if (bt_nus_send(NULL, data_array, length)) {
    LOG_WRN("Failed to send data over BLE connection");
    ble_packets_failed++;
  } else {
    /* Identify packet type by header byte */
    uint8_t header = (uint8_t)data_array[0];
    if (header == BLE_EEG_PACKET_HEADER) {
      ble_eeg_packets_sent++;
    } else if (header == BLE_IMU_PACKET_HEADER) {
      ble_imu_packets_sent++;
    } else if (header == BLE_MIC_PACKET_HEADER) {
      ble_mic_packets_sent++;
    } else {
      ble_other_packets_sent++;
    }
  }
}

/*==============================================================================
 * Public Functions - Packet Statistics
 *============================================================================*/

void ble_reset_packet_counters(void) {
  LOG_INF("Resetting BLE packet counters");
  ble_eeg_packets_sent = 0;
  ble_mic_packets_sent = 0;
  ble_imu_packets_sent = 0;
  ble_other_packets_sent = 0;
  ble_packets_failed = 0;
}

void ble_print_packet_stats(void) {
  LOG_INF("BLE packets: EEG=%u, IMU=%u, MIC=%u, other=%u, failed=%u",
          ble_eeg_packets_sent, ble_imu_packets_sent, ble_mic_packets_sent,
          ble_other_packets_sent, ble_packets_failed);
}

void ble_get_packet_stats(ble_packet_stats_t *stats) {
  if (stats != NULL) {
    stats->eeg_sent = ble_eeg_packets_sent;
    stats->mic_sent = ble_mic_packets_sent;
    stats->imu_sent = ble_imu_packets_sent;
    stats->other_sent = ble_other_packets_sent;
    stats->failed = ble_packets_failed;
  }
}

uint32_t ble_get_packets_sent(void) {
  return ble_eeg_packets_sent + ble_imu_packets_sent + ble_mic_packets_sent + ble_other_packets_sent;
}

uint32_t ble_get_packets_failed(void) {
  return ble_packets_failed;
}

uint32_t ble_get_eeg_packets_sent(void) {
  return ble_eeg_packets_sent;
}

uint32_t ble_get_mic_packets_sent(void) {
  return ble_mic_packets_sent;
}

uint32_t ble_get_imu_packets_sent(void) {
  return ble_imu_packets_sent;
}

/*==============================================================================
 * Public Functions - Connection Status
 *============================================================================*/

bool ble_is_connected(void) {
  return (current_conn != NULL);
}

/*==============================================================================
 * Thread Definition
 *============================================================================*/

K_THREAD_DEFINE(ble_write_thread_id, CONFIG_BT_NUS_THREAD_STACK_SIZE, ble_write_thread, NULL, NULL, NULL,
                PRIORITY_BLE_THREAD, 0, 0);

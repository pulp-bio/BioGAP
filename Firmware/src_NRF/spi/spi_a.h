/*
 * ----------------------------------------------------------------------
 *
 * File: spi_a.h
 *
 * Copyright (C) 2026, ETH Zurich
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
 * @file spi_a.h
 * @brief Shared SPI_A bus (nrfx SPIM4) initialization and transfer dispatch
 *
 * SPI_A is the BioGAP mainboard's high-speed SPI bus (SCK=P0.08, MOSI=P0.09,
 * MISO=P0.10, driven by nrfx SPIM instance 4). It is physically shared by
 * every device that sits on it -- currently the two ADS1298 AFEs (ExG
 * shield) and the ESP32-C6 / SD card (WiFi/SD shield). Each device gets its
 * own software-toggled CS GPIO; there is no hardware-managed CS on this
 * bus, since more than one device shares it.
 *
 * The nrfx SPIM driver only allows ONE event handler per peripheral
 * instance, but every consumer needs to know when *its* transfer
 * completed. This module resolves that with a single shared interrupt
 * handler that dispatches to whichever consumer kicked off the
 * in-flight transfer (tracked via spi_a_begin_transfer()). Since the bus
 * is physically serial and every consumer is expected to hold spi_a_mutex
 * for the duration of its transfer, at most one transfer -- and therefore
 * one owner -- is ever in flight at a time.
 */

#ifndef SPI_A_H
#define SPI_A_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_spim.h>

/** @brief SPIM instance index for SPI_A (must match spi_a_default pinctrl
 *  in the sensei-sdk board DTS: nrf5340_senseiv1_cpuapp.dts) */
#define SPI_A_INST_IDX 4

/** @brief SCK pin number for SPI_A (SPI_A_CLK = P0.08) */
#define SPI_A_SCK_PIN 8

/** @brief MOSI pin number for SPI_A (SPI_A_MOSI = P0.09) */
#define SPI_A_MOSI_PIN 9

/** @brief MISO pin number for SPI_A (SPI_A_MISO = P0.10) */
#define SPI_A_MISO_PIN 10

/** @brief SPI_A interrupt priority level */
#define SPI_A_INT_PRIO 1

/*
 * Mode the bus is brought up in. Mode 1 (CPOL 0 / CPHA 1) is what the ADS1298
 * needs. The BGT60TR13C radar speaks mode 0, and switches the bus for the
 * duration of each of its transactions, restoring whatever configuration it
 * found (see spi_a_save_config) rather than a fixed idle setting -- the clock
 * is a variable here, owned by whichever consumer is currently active.
 *
 * CONFIG_MMWAVE_SPI_STATIC_MODE is a bring-up diagnostic: it brings the bus up
 * in the radar's dialect and makes the radar skip its per-transaction
 * reconfigure, so the mode switching itself can be ruled in or out as the cause
 * of a non-responsive radar. It breaks the ADS1298, which needs mode 1.
 */
#if defined(CONFIG_MMWAVE_SPI_STATIC_MODE)
#define SPI_A_INIT_MODE NRF_SPIM_MODE_0
/** @brief SCLK frequency for init_spi_a_bus()'s initial bring-up */
#define SPI_A_INIT_FREQ NRFX_MHZ_TO_HZ(CONFIG_MMWAVE_SPI_FREQ_MHZ)
#else
#define SPI_A_INIT_MODE NRF_SPIM_MODE_1
/** @brief SCLK frequency for init_spi_a_bus()'s initial bring-up */
#define SPI_A_INIT_FREQ NRFX_MHZ_TO_HZ(2)
#endif

/**
 * @brief SCLK frequency safe for ADS1298 command decode
 *
 * The ADS1298 needs 4 tCLK periods (tSDECODE) to internally decode each
 * command byte (RESET, SDATAC, RREG, WREG, ...); sending a multi-byte
 * command faster than that -- as a single burst DMA transfer does -- can
 * outrun the decoder. Only applies to actual command bytes, not to reading
 * already-converted samples in RDATAC mode (see SPI_A_ADS_STREAMING_FREQ_HZ).
 */
#define SPI_A_ADS_CMD_SAFE_FREQ_HZ NRFX_MHZ_TO_HZ(4)

/**
 * @brief SCLK frequency for steady-state ADS1298 data streaming
 *
 * SPI rate when reading data from the ADS1298.
 * Safe once the ADS1298 is in RDATAC mode
 * Maxmimum value is 8 Mhz (theoretical 15 MHz, but NRF supports only 8 or 16 MHz)
 */
#define SPI_A_ADS_STREAMING_FREQ_HZ NRFX_MHZ_TO_HZ(8)

/**
 * @brief SCLK frequency for WiFi/SD (ESP32-C6) transfers
 *
 * The ESP side (as slave) can handle 40 MHz
 *
 * If this differs from SPI_A_ADS_STREAMING_FREQ_HZ, spi_master_transceive()
 * (wifi_sd_spi_functions.c) switches SPI_A to this rate for the duration of
 * each ESP transfer and back afterward; if they're equal, it skips both
 * switches. That's a plain runtime check, not #if -- NRFX_MHZ_TO_HZ()'s
 * expansion isn't guaranteed valid in a preprocessor constant expression.
 */
#define SPI_A_ESP_STREAMING_FREQ_HZ NRFX_MHZ_TO_HZ(32)

/** @brief nrfx SPIM driver instance shared by every device on SPI_A */
extern nrfx_spim_t spi_a_inst;

/**
 * @brief Serializes access to SPI_A across all consumers (ADS, WiFi/SD, ...)
 *
 * @note The ADS1298 driver releases this mutex right after kicking off an
 * async nrfx_spim_xfer(), rather than holding it until the transfer completes. 
 * This leaves a window where another consumer (ESP) could start a
 * transfer before the ADS one physically finishes. 
 */
extern struct k_mutex spi_a_mutex;

/**
 * @brief Identifies which consumer owns the in-flight SPI_A transfer
 *
 * Set via spi_a_begin_transfer() immediately before issuing an
 * nrfx_spim_xfer() on spi_a_inst, so the shared interrupt handler routes
 * the completion event to the right consumer.
 */
typedef enum {
  SPI_A_OWNER_NONE,
  SPI_A_OWNER_ADS,
  SPI_A_OWNER_WIFI_SD,
  SPI_A_OWNER_MMWAVE,
} spi_a_owner_t;

/**
 * @brief Diagnostic-only step markers for the current SPI_A transaction
 *
 * Purely informational -- exists only so the periodic status heartbeat
 * (wifi_sd_shield_appl.c) can show exactly which step was last reached
 * before a stall, since spi_a_current_owner() alone only says who has the
 * bus, not what they're doing with it.
 *
 * Tracked as TWO independent variables, one per subsystem (see
 * spi_a_set_ads_checkpoint()/spi_a_set_wifi_checkpoint() below) -- a single
 * shared variable would have whichever side runs more often (ADS reads fire
 * far more frequently than WiFi sends) constantly overwrite the other,
 * making "last value before a freeze" ambiguous as to which side actually
 * froze. Two variables let the heartbeat show both at once.
 */
typedef enum {
  SPI_A_CP_NONE = 0,

  /* ads_spi_comm.c (ads1298_read_spi / ads1298_read_samples / ads1298_write_spi) */
  SPI_A_CP_ADS_WAIT_MUTEX = 1,
  SPI_A_CP_ADS_GOT_MUTEX = 2,
  SPI_A_CP_ADS_IRQ_LOCKED = 3,
  SPI_A_CP_ADS_CS_SELECTED = 4,
  SPI_A_CP_ADS_XFER_STARTED = 5,
  SPI_A_CP_ADS_IRQ_UNLOCKED = 6,
  SPI_A_CP_ADS_DONE = 7,

  /* wifi_sd_spi_functions.c (spi_master_transceive) */
  SPI_A_CP_WIFI_WAIT_MUTEX = 10,
  SPI_A_CP_WIFI_GOT_MUTEX = 11,
  SPI_A_CP_WIFI_WAIT_ADS_OWNER = 12,
  SPI_A_CP_WIFI_DRAINING_SEM = 13,
  SPI_A_CP_WIFI_FREQ_UP = 14,
  SPI_A_CP_WIFI_XFER_STARTED = 15,
  SPI_A_CP_WIFI_WAIT_COMPLETION = 16,
  SPI_A_CP_WIFI_FREQ_DOWN = 17,
  SPI_A_CP_WIFI_DONE = 18,

  /* ads_spi_hw.c / ads_spi_data.c -- the SPI completion INTERRUPT handler
   * (ads_spim_transfer_complete() -> ads_spim_handler_done()), fired once
   * the DMA transfer kicked off at SPI_A_CP_ADS_XFER_STARTED actually
   * finishes. Runs in ISR context, not the calling thread -- this is the
   * gap that let the ISR-context queue-overflow halt hide from the
   * thread-side checkpoints above (they stayed frozen at ADS_DONE with no
   * visibility into what happened after). */
  SPI_A_CP_ADS_ISR_ENTERED = 20,
  SPI_A_CP_ADS_ISR_CS_DEASSERTED = 21,
  SPI_A_CP_ADS_ISR_HANDING_TO_ESP = 22,
  SPI_A_CP_ADS_ISR_DONE = 23,
} spi_a_checkpoint_t;

/** @brief Record the current step of an ADS-side SPI_A transaction (diagnostic only) */
void spi_a_set_ads_checkpoint(uint8_t cp);

/** @brief Read back the last-recorded ADS-side step (diagnostic only) */
uint8_t spi_a_get_ads_checkpoint(void);

/** @brief Record the current step of a WiFi-side SPI_A transaction (diagnostic only) */
void spi_a_set_wifi_checkpoint(uint8_t cp);

/** @brief Read back the last-recorded WiFi-side step (diagnostic only) */
uint8_t spi_a_get_wifi_checkpoint(void);

/**
 * @brief Bring up the shared SPI_A (SPIM4) peripheral
 *
 * Configures SPI Mode 1 (CPOL=0, CPHA=1), 4 MHz, MSB first, with no
 * hardware-managed CS (every device's CS is a plain GPIO toggled by its
 * own driver). Call exactly once at boot, regardless of which shields
 * (ExG, WiFi/SD, ...) are actually enabled -- each shield's own init
 * function should call this rather than initializing SPIM4 itself.
 *
 * @return 0 on success, -1 on nrfx init failure
 */
int init_spi_a_bus(void);

/**
 * @brief Declare the owner and CS line for the next SPI_A transfer
 *
 * Must be called with spi_a_mutex held, immediately before asserting CS
 * and issuing nrfx_spim_xfer() on spi_a_inst. Asserts *cs (if non-NULL)
 * and records it so the shared interrupt handler can deassert it
 * automatically on completion; pass NULL if the caller manages its own
 * CS deassertion (e.g. ADS toggles two CS lines from a single owner).
 *
 * @param owner Consumer identifier, used to route the completion callback
 * @param cs    CS GPIO to assert now and auto-deassert on completion, or NULL
 */
void spi_a_begin_transfer(spi_a_owner_t owner, const struct gpio_dt_spec *cs);

/**
 * @brief Returns the current owner of SPI_A, or SPI_A_OWNER_NONE if idle
 *
 * Lets a consumer holding spi_a_mutex confirm no other consumer's transfer
 * is still physically in flight before starting its own -- see
 * spi_a_mutex's doc comment for why the mutex alone isn't sufficient.
 */
spi_a_owner_t spi_a_current_owner(void);

/**
 * @brief Report whether a transfer is still in flight on the bus
 *
 * Convenience predicate over spi_a_current_owner() for consumers that only
 * need to know whether the bus is busy, not who has it. True between
 * spi_a_begin_transfer() and the completion event. Because the ADS1298 driver
 * releases spi_a_mutex as soon as it has kicked off its async transfer,
 * holding the mutex is not by itself proof that the bus is idle: a consumer
 * that rewrites the peripheral's configuration -- rather than only pushing
 * data -- must wait for this to go false first, or it would corrupt the
 * transfer still running.
 *
 * @return true if a transfer is in progress, false if the bus is idle
 */
bool spi_a_transfer_in_flight(void);

/**
 * @brief Reconfigure SPI_A's CLK frequency
 *
 * Briefly tears down and reinitializes the SPIM peripheral with the same
 * pin/mode config but a new frequency.
 *
 * Heavier than spi_a_reconfigure() -- prefer this when changing the clock for
 * a whole acquisition phase (ADS command vs streaming, ESP transfers), and
 * spi_a_reconfigure() when switching dialect for a single transaction.
 *
 * @param frequency_hz Desired CLK frequency in Hz -- must be one of the
 *  discrete values nrfx_spim supports (e.g. SPI_A_ADS_CMD_SAFE_FREQ_HZ,
 *  SPI_A_ADS_STREAMING_FREQ_HZ, SPI_A_ESP_STREAMING_FREQ_HZ); arbitrary
 *  values are rejected by nrfx_spim_init() with NRFX_ERROR_INVALID_PARAM.
 * @return 0 on success, -1 on nrfx re-init failure
 */
int spi_a_set_frequency(uint32_t frequency_hz);

/** @brief Saved CONFIG/FREQUENCY register pair, for save/restore around a
 *  transaction that needs a different SPI dialect. Opaque: treat only as
 *  something to hand back to spi_a_restore_config(). */
typedef struct {
  uint32_t config;
  uint32_t frequency;
} spi_a_config_t;

/**
 * @brief Capture the bus's current mode and clock
 *
 * Counterpart to spi_a_restore_config(). Consumers that switch dialect for a
 * single transaction must put back what they found rather than any fixed idle
 * setting: the clock is owned by whichever consumer is active (the ADS1298
 * runs at 4 or 8 MHz depending on phase, the ESP32 at 32 MHz), so restoring a
 * compile-time constant would silently reclock somebody else's session.
 *
 * Must be called with spi_a_mutex held and no transfer in flight.
 */
void spi_a_save_config(spi_a_config_t *out);

/**
 * @brief Put back a configuration captured by spi_a_save_config()
 *
 * Must be called with spi_a_mutex held and no transfer in flight.
 */
void spi_a_restore_config(const spi_a_config_t *cfg);

/**
 * @brief Switch the bus mode and clock for the current owner
 *
 * Devices on SPI_A do not all speak the same SPI dialect: the ADS1298 needs
 * Mode 1, the BGT60TR13C radar needs Mode 0 and runs faster. Only the CONFIG
 * and FREQUENCY registers are rewritten, so pin assignment, DMA and interrupt
 * setup from init_spi_a_bus() stay intact -- which is what makes this cheap
 * enough to do per transaction.
 *
 * Must be called with spi_a_mutex held and no transfer in flight. An owner
 * that changes the configuration is responsible for restoring the one it
 * captured with spi_a_save_config() before releasing the mutex, so that owners
 * which never touch the configuration keep seeing the bus as they left it.
 *
 * @param mode      SPI mode to switch to
 * @param frequency SPI clock to switch to
 */
void spi_a_reconfigure(nrf_spim_mode_t mode, nrf_spim_frequency_t frequency);

#endif // SPI_A_H

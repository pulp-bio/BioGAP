/*
 * -----------------------------------------------------------------------------
 *
 * File: static_distance.h
 *
 * Generated file.
 *
 * This file contains BGT60TR13C register configuration values generated with an
 * Infineon radar configuration tool.
 *
 * The file was generated for the BioGAP mmWave radar project and is used by the
 * firmware to configure the BGT60TR13C sensor.
 *
 * Generated/added by:
 * - Benjamin Löliger, ETH Zurich
 *
 * Tool/source:
 * - XENSIV Radar Fusion GUI
 *
 * License:
 * - See the Infineon Radar SDK / tool license terms for generated files.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef XENSIV_BGT60TRXX_CONF_H
#define XENSIV_BGT60TRXX_CONF_H

#define XENSIV_BGT60TRXX_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define XENSIV_BGT60TRXX_CONF_START_FREQ_HZ (59250000000)
#define XENSIV_BGT60TRXX_CONF_END_FREQ_HZ (62250000000)
#define XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP (64)
#define XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME (32)
#define XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS (1)
#define XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS (1)
#define XENSIV_BGT60TRXX_CONF_SAMPLE_RATE (1000000)
#define XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S (0.000411888)
#define XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S (0.199849)
#define XENSIV_BGT60TRXX_CONF_NUM_REGS (38)

#if defined(XENSIV_BGT60TRXX_CONF_IMPL)
const uint32_t register_list[] = {
    0x011e8270UL,
    0x03140210UL,
    0x09e967fdUL,
    0x0b0805b4UL,
    0x0d1023ffUL,
    0x0f010700UL,
    0x11000000UL,
    0x13000000UL,
    0x15000000UL,
    0x17000be0UL,
    0x19000000UL,
    0x1b000000UL,
    0x1d000000UL,
    0x1f000b60UL,
    0x21103c51UL,
    0x231ff41fUL,
    0x25703defUL,
    0x2d000490UL,
    0x3b000480UL,
    0x49000480UL,
    0x57000480UL,
    0x5911be0eUL,
    0x5b6f1c0aUL,
    0x5d01f000UL,
    0x5f787e1eUL,
    0x61c727deUL,
    0x63000393UL,
    0x650002b2UL,
    0x67000040UL,
    0x69000000UL,
    0x6b000000UL,
    0x6d000000UL,
    0x6f25a310UL,
    0x7f000100UL,
    0x8f000100UL,
    0x9f000100UL,
    0xad000000UL,
    0xb7000000UL,
};
#endif /* XENSIV_BGT60TRXX_CONF_IMPL */

#endif /* XENSIV_BGT60TRXX_CONF_H */

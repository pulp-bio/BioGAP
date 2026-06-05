/*
 * Minimal Silicon Labs sl_udelay shim for Zephyr/nRF build.
 * Delegates to Zephyr's k_usleep().
 */

#ifndef SL_UDELAY_H
#define SL_UDELAY_H

#include <stdint.h>
#include <zephyr/kernel.h>

static inline void sl_udelay_wait(uint32_t useconds)
{
    k_usleep((int32_t)useconds);
}

#endif /* SL_UDELAY_H */

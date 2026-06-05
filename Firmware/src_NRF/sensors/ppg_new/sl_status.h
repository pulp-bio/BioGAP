/*
 * Minimal Silicon Labs sl_status_t shim for Zephyr/nRF build.
 * Maps SL status codes to plain integers; no Silicon Labs SDK required.
 */

#ifndef SL_STATUS_H
#define SL_STATUS_H

#include <stdint.h>

typedef uint32_t sl_status_t;

#define SL_STATUS_OK                 ((sl_status_t)0x0000)
#define SL_STATUS_FAIL               ((sl_status_t)0x0001)
#define SL_STATUS_INVALID_PARAMETER  ((sl_status_t)0x0021)
#define SL_STATUS_ALLOCATION_FAILED  ((sl_status_t)0x0025)
#define SL_STATUS_NOT_INITIALIZED    ((sl_status_t)0x0014)
#define SL_STATUS_FULL               ((sl_status_t)0x0026)
#define SL_STATUS_EMPTY              ((sl_status_t)0x0027)

#endif /* SL_STATUS_H */

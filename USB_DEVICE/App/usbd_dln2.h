/**
 * @file    usbd_dln2.h
 * @brief   Vendor-specific bulk USB class for the Diolan DLN-2 protocol.
 */

#ifndef USBD_DLN2_H
#define USBD_DLN2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"

#define DLN2_IN_EP                      0x81U
#define DLN2_OUT_EP                     0x01U
#define DLN2_MAX_PACKET_SIZE            64U

extern USBD_ClassTypeDef USBD_DLN2;

/* Handles a received request, if any. Call from the main loop. */
void USBD_DLN2_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* USBD_DLN2_H */

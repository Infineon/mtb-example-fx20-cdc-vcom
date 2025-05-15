/***************************************************************************//**
* \file cy_usb_vcom_device.h
* \version 1.0
*
* Defines messages codes and other constants used in the USB CDC Virtual COM
* Port application.
*
*******************************************************************************
* \copyright
* (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef _CY_USB_VCOM_DEVICE_H_
#define _CY_USB_VCOM_DEVICE_H_

#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define USB3_DESC_ATTRIBUTES __attribute__ ((section(".descSection"), used))

#define CY_USB_VCOM_DEVICE_MSG_START_DATA_XFER      (0x02)
#define CY_USB_VCOM_DEVICE_MSG_READ_COMPLETE        (0x04)
#define CY_USB_VCOM_DEVICE_MSG_WRITE_COMPLETE       (0x05)
#define CY_USB_VCOM_DEVICE_MSG_SLP_OUT              (0x09)
#define CY_USB_UART_MSG_WRITE_COMPLETE              (0x0E)
#define CY_USB_UART_MSG_READ_COMPLETE               (0x0F)
#define CY_USB_VCOM_VBUS_CHANGE_INTR                (0x10)
#define CY_USB_VCOM_VBUS_CHANGE_DEBOUNCED           (0x11)
#define CY_USB_VCOM_SETLINECODING_CMD               (0x12)

#define CY_USB_VCOM_DEVICE_MSG_QUEUE_SIZE  (24)
#define CY_USB_MAX_DATA_BUFFER_SIZE        (1024)
#define CY_IFX_VCOM_MAX_QUEUE_SIZE         (2)
#define CY_USB_VCOM_DEVICE_MSG_SIZE        (sizeof (cy_stc_usbd_app_msg_t))

#define UART_WRITE                    1
#define UART_READ                     2
#define UART_WRITE_DMA_CHANNEL        18
#define UART_READ_DMA_CHANNEL         19

/********************************************************************************/
/*******************************************************************************/

/* Function which registers the USB descriptors corresponding to the VCOM
 * application with the USB stack.
 */
void Cy_USB_VcomDevice_RegisterDescriptors(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_en_usb_speed_t usbSpeed);

void Cy_USB_VcomDeviceTaskHandler(void *pTaskParam);
void Cy_Uart_ReadDma_ISR(void);
void Cy_Uart_WriteDma_ISR(void);
void Cy_USB_UARTDmaWriteCompletion(void *pApp);
void Cy_USB_UARTDmaReadCompletion(void *pApp);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_VCOM_DEVICE_H_ */

/* End of File */


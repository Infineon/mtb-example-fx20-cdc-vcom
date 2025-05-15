/***************************************************************************//**
* \file cy_usb_app.h
* \version 1.0
*
* Defines the interfaces used by the FX10 USB CDC Virtual COM Port application.
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

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "cy_pdl.h"
#include "cy_debug.h"
#include "timers.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define VCOM_RESPONSE_TIME              (2000)  /** UART RX buffer polling duration in milliseconds. */
#define VCOM_UART_MAX_RD_SIZE           (64u)   /** Maximum size of data read from UART RX FIFO in one burst. */
#define VCOM_APP_BUFFER_CNT             (4u)    /** Number of buffers used for USB->UART and UART->USB transfers. */

/* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PORT           (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN            (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR           (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE               (0u)

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

/* USBD layer return code shared between USBD layer and Application layer. */
typedef enum cy_en_usb_app_ret_code_ {
    CY_USB_APP_STATUS_SUCCESS=0,
    CY_USB_APP_STATUS_FAILURE,
}cy_en_usb_app_ret_code_t;


/*
 * USB application data structure which is bridge between USB system and device
 * functionality.
 * It maintains some USB system information which comes from USBD and it also
 * maintains info about functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    uint8_t firstInitDone;                                          /** Whether APP init is complete. */
    cy_en_usb_device_state_t devState;                              /** Current device state. */
    cy_en_usb_device_state_t prevDevState;                          /** Previous device state. */
    cy_en_usb_speed_t devSpeed;                                     /** Active USB connection speed. */
    uint8_t devAddr;                                                /** USB address assigned to the device. */
    uint8_t activeCfgNum;                                           /** Active configuration index. */
    uint8_t prevAltSetting;                                         /** Previous alternate setting number. */
    bool dataXferIntrEnabled;                                       /** Whether DMA interrupts are initialised. */

    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_MAX_ENDP_NUMBER];    /** Data transfer info for IN endpoints. */
    cy_stc_app_endp_dma_set_t endpOutDma[CY_USB_MAX_ENDP_NUMBER];   /** Data transfer info for OUT endpoints. */

    DMAC_Type *pCpuDmacBase;                                        /** Pointer to DMAC IP registers. */
    DW_Type *pCpuDw0Base;                                           /** Pointer to DataWire-0 IP registers. */
    DW_Type *pCpuDw1Base;                                           /** Pointer to DataWire-1 IP registers. */
    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;                      /** High BW DMA manager context pointer. */
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;                              /** USBD stack context pointer. */
    cy_stc_scb_uart_context_t *pUartCtxt;                           /** SCB-UART context pointer. */

    TaskHandle_t vcomDevicetaskHandle;                              /** VCOM application task handle. */
    QueueHandle_t vcomDeviceQueue;                                  /** VCOM application message queue. */
    TimerHandle_t vcomTimerHandle;                                  /** VCOM application timer handle. */
    TimerHandle_t vbusDebounceTimer;                                /** VBus change debounce timer handle. */
    uint32_t vcomtimerExpiry;                                       /** VCOM timer duration in milliseconds. */

    uint8_t  vcomOutEpNum;                                          /** Index of OUT endpoint used for VCOM i/f. */
    uint8_t  vcomInEpNum;                                           /** Index of IN endpoint used for VCOM i/f. */

    cy_stc_hbdma_channel_t *pUartToUsbChn;                          /** UART to USB DMA channel handle. */
    cy_stc_hbdma_channel_t *pUsbToUartChn;                          /** USB to UART DMA channel handle. */

    uint8_t *pUartRxBuffer[VCOM_APP_BUFFER_CNT];                    /** Pointer of buffers to read UART data into. */
    uint16_t uartRxDataLen[VCOM_APP_BUFFER_CNT];                    /** Amount of data in each UART RX buffer. */
    uint8_t  uartRxBufIndex;                                        /** Index of current UART read buffer. */
    uint8_t  nxtUsbTxBufIdx;                                        /** Index of next buffer to be sent on USB EP. */
    uint8_t  uartRxFreeBufCount;                                    /** Number of free UART RX buffers. */
    bool     usbInXferPending;                                      /** Whether a write has been queued on IN EP. */

    uint8_t *pUsbRxBuffer[VCOM_APP_BUFFER_CNT];                     /** Pointer of buffers to read USB data into. */
    uint16_t usbRxDataLen[VCOM_APP_BUFFER_CNT];                     /** Amount of data in each USB RX buffer. */
    uint16_t usbOutPktLen;                                          /** Length of current USB OUT packet. */
    uint8_t  nxtUsbRxBufIdx;                                        /** Index of next USB read buffer. */
    uint8_t  uartTxBufIndex;                                        /** Index of current UART write buffer. */
    uint8_t  usbRxBusyBufCount;                                     /** Number of used USB RX buffers. */

    bool vbusChangeIntr;                                            /** VBus change interrupt received flag. */
    bool vbusPresent;                                               /** VBus presence indicator flag. */
    bool usbConnected;                                              /** Whether USB connection is enabled. */
    bool blockUartReads;                                            /** Whether read from UART should be blocked. */
};

void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt,
                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    DMAC_Type *pCpuDmacBase,
                    DW_Type *pCpuDw0Base,
                    DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);
uint32_t *Cy_USB_CalculateEpmAddr(uint32_t endpNum,
                                  cy_en_usb_endp_dir_t endpDirection);

void Cy_USB_AppSetCfgCallback(void *pAppCtxt,
                              cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetCallback(void *pAppCtxt,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetupCallback(void *pAppCtxt,
                             cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSuspendCallback(void *pAppCtxt,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppResumeCallback (void *pAppCtxt,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetIntfCallback(void *pAppCtxt,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1SleepCallback(void *pUsbApp,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1ResumeCallback(void *pUsbApp,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetFeatureCallback(void *pUsbApp,
                                  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppTerminateUartDma(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                       uint8_t uart_read_write);
void Cy_USB_AppClearUartDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                       uint8_t uart_read_write);
void Cy_USB_AppUARTWrite(uint8_t *pBuffer,uint16_t dataSize);
void Cy_USB_AppUartRead(uint8_t *pBuffer,uint16_t dataSize);
void Cy_USB_AppUartInitsDmaIntr(uint8_t uart_read_write,cy_israddress userIsr);
bool Cy_USB_SSConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt);
void Cy_USB_SSConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);
void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                 uint32_t endpNumber,
                                 cy_en_usb_endp_dir_t endpDirection);
void Cy_USB_VcomDeviceDmaReadCompletion (void *pApp);
void Cy_USB_VcomDeviceDmaWriteCompletion (void *pApp);
void Cy_USB_AppQueueWrite(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                          uint8_t *pBuffer, uint16_t dataSize);
void Cy_USB_AppQueueRead(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                         uint8_t *pBuffer, uint16_t dataSize);
void Cy_USB_AppDisableEndpDma(cy_stc_usb_app_ctxt_t *pAppCtxt);

void Cy_USBHS_OutEp_DW_ISR(void);
void Cy_USBHS_InEp_DW_ISR(void);
void Cy_USB_EgressDma_ISR(void);
void Cy_USB_IngressDma_ISR(void);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */


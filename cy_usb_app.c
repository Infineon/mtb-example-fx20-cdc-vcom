/***************************************************************************//**
* \file cy_usb_app.c
* \version 1.0
*
* Implements the USB data handling part of the FX10 USB CDC Virtual COM Port
* application.
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

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usbss_cal_drv.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"
#include "cy_usb_vcom_device.h"
#include "cy_usb_app.h"

/* CDC Class specific requests to be handled by this application. */
#define SET_LINE_CODING        0x20
#define GET_LINE_CODING        0x21
#define SET_CONTROL_LINE_STATE 0x22

/* Default SCB-UART configuration. */
cy_stc_scb_uart_config_t glUartConfig = {
    .uartMode                   = CY_SCB_UART_STANDARD,
    .oversample                 = 8,
    .dataWidth                  = 8,
    .enableMsbFirst             = false,
    .stopBits                   = CY_SCB_UART_STOP_BITS_1,
    .parity                     = CY_SCB_UART_PARITY_NONE,
    .enableInputFilter          = false,
    .dropOnParityError          = false,
    .dropOnFrameError           = false,
    .enableMutliProcessorMode   = false,
    .receiverAddress            = 0,
    .receiverAddressMask        = 0,
    .acceptAddrInFifo           = false,
    .irdaInvertRx               = false,
    .irdaEnableLowPowerReceiver = false,
    .smartCardRetryOnNack       = false,
    .enableCts                  = false,
    .ctsPolarity                = CY_SCB_UART_ACTIVE_LOW,
    .rtsRxFifoLevel             = 0,
    .rtsPolarity                = CY_SCB_UART_ACTIVE_LOW,
    .breakWidth                 = 8,
    .rxFifoTriggerLevel         = VCOM_UART_MAX_RD_SIZE,
    .rxFifoIntEnableMask        = 0,
    .txFifoTriggerLevel         = VCOM_UART_MAX_RD_SIZE,
    .txFifoIntEnableMask        = 0
};

static cy_stc_dma_descriptor_t gUartWriteDmaDesc1, gUartWriteDmaDesc2;
static cy_stc_dma_descriptor_t gUartReadlDmaDesc;

static volatile bool cy_vcom_IsApplnActive = false;  /* Application Status Tracker */

uint32_t cy_timer_loop;

/* These two data length will assist timer functionality. */
static uint32_t uartRxFifoDataLenCurr = 0x00, uartRxFifoDataLenPrev = 0x00;

static uint8_t __attribute__ ((section(".descSection"), used)) __attribute__ ((aligned (32))) glVcomCfgData[7U] = {
    0x00,0x10,0x0E,0x00,0x02,0x00,0x08};
__attribute__ ((section(".descSection"), used)) uint32_t SetSelDataBuffer[8] __attribute__ ((aligned (32)));

/*
 * Function: Cy_USB_AppVcomSendUartData
 * Description: Function to send the data received from UART to the USBSS
 * endpoint using High BandWidth DMA channel.
 * Parameter:
 *     pAppCtxt  : VCOM application context handle.
 *     dataBuf_p : Pointer to buffer containing data to be sent.
 *     dataLength: Length of the data in bytes.
 * Return: void
 */
void
Cy_USB_AppVcomSendUartData (cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint8_t *dataBuf_p, uint32_t dataLength)
{
    cy_stc_hbdma_buff_status_t dmaBufStat;
    cy_en_hbdma_mgr_status_t   hbdma_stat;

    if (pAppCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        dmaBufStat.pBuffer = dataBuf_p;
        dmaBufStat.size    = VCOM_UART_MAX_RD_SIZE;
        dmaBufStat.count   = dataLength;
        dmaBufStat.status  = 0;
        hbdma_stat = Cy_HBDma_Channel_CommitBuffer(pAppCtxt->pUartToUsbChn, &dmaBufStat);
        if (hbdma_stat != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("Cy_USB_AppVcomSendUartData: DMA error=%x\r\n", hbdma_stat);
        }
    } else {
        if (!pAppCtxt->usbInXferPending) {
            /* Send the data on the IN endpoint if it is currently idle. */
            Cy_USB_AppQueueWrite(pAppCtxt, pAppCtxt->vcomInEpNum, dataBuf_p, dataLength);

            if ((pAppCtxt->devSpeed == CY_USBD_USB_DEV_FS) && (dataLength == VCOM_UART_MAX_RD_SIZE)) {
                /* When sending a full packet (only applicable in FS case), send a ZLP after this. */
                Cy_USBD_SendEgressZLP(pAppCtxt->pUsbdCtxt, pAppCtxt->vcomInEpNum);
            }

            pAppCtxt->nxtUsbTxBufIdx++;
            if (pAppCtxt->nxtUsbTxBufIdx >= VCOM_APP_BUFFER_CNT) {
                pAppCtxt->nxtUsbTxBufIdx = 0;
            }
        }
    }
}

/*
 * Function: Cy_VcomDev_QueueUartRead()
 * Description: Function which identifies the DMA buffer address where
 * the next data from UART RX_FIFO should be read and queues DataWire
 * transfer to read a specific amount of data into it.
 *
 * Parameter:
 *      pAppCtxt  : Pointer to the application context structure.
 *      dataLength: Length of data to be read from the SCB RX_FIFO.
 * return: void
 */
static void Cy_VcomDev_QueueUartRead (cy_stc_usb_app_ctxt_t *pAppCtxt,
                                      uint32_t dataLength)
{
    uint8_t *rxBuf = pAppCtxt->pUartRxBuffer[pAppCtxt->uartRxBufIndex];

    /* Save the current data size. */
    pAppCtxt->uartRxDataLen[pAppCtxt->uartRxBufIndex] = dataLength;

    /* Configure the DataWire channel to read specified amount of data
     * into the next Rx buffer.
     */
    Cy_USB_AppUartRead(rxBuf, dataLength);
}

/*
 * Function: Cy_USB_AppVcomTimerCallback()
 * Description: DMA transfer from the UART RX FIFO will only happen when
 * the amount of data received crosses the threshold. To handle the cases
 * where lesser amount of data is received, a timer is being run. If the
 * RX FIFO has valid data which is below the threshold level, the UART DMA
 * will be re-configured to read it out.
 *
 * Parameter:
 *      xTimer: RTOS timer handle.
 * return: void
 */
void
Cy_USB_AppVcomTimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);

    (void)pAppCtxt;

    if (pAppCtxt->blockUartReads) {
        /* If the flag to block UART reads is set, do nothing. */
        return;
    }

    DBG_APP_TRACE("Timer Callback >>\r\n");

    if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
        /*
         * Just wait for two consecutive timer expire. Both the time if available
         * data is same, then initiate transfer.
         */
        uartRxFifoDataLenPrev = uartRxFifoDataLenCurr;
        uartRxFifoDataLenCurr = Cy_SCB_GetNumInRxFifo(SCB1);

        DBG_APP_TRACE("uartRxFifoDataLenPrev =0x%x\r\n", uartRxFifoDataLenPrev);
        DBG_APP_TRACE("uartRxFifoDataLenCurr =0x%x\r\n", uartRxFifoDataLenCurr);

        if ((uartRxFifoDataLenCurr > 0x00) &&
                (uartRxFifoDataLenCurr == uartRxFifoDataLenPrev)) {

            DBG_APP_TRACE("Start RX_FIFO read len:%x\r\n", uartRxFifoDataLenCurr);

            /*
             * Just enable SHORT packet with HBWDMA before activating
             * CPUSS DMA. Later code wont get chance to enable short bit.
             */
            MAIN_REG->TR_ASSIST_GR[0].TRA_SLP_CTRL[0] =
                MAIN_REG_TR_ASSIST_GR_TRA_SLP_CTRL_TRA_SHORT_CONFIG_Msk |
                (uartRxFifoDataLenCurr <<
                 MAIN_REG_TR_ASSIST_GR_TRA_SLP_CTRL_TRA_BYTE_COUNT_Pos);

            /* Prepare the DataWire channel to read the available amount of data. */
            Cy_VcomDev_QueueUartRead(pAppCtxt, uartRxFifoDataLenCurr);

            /* SCB.RX_TRIGGER needs to be asserted high manually to get the
             * DMA transfer to start.
             */
            Cy_TrigMux_SwTrigger(TRIG_OUT_1TO1_0_SCB1_RX_TO_PDMA0_TR_IN19,
                    CY_TRIGGER_TWO_CYCLES);
            uartRxFifoDataLenPrev = 0x00;
            uartRxFifoDataLenCurr = 0x00;
        }
    } else {

        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_FS) {
            uartRxFifoDataLenPrev = uartRxFifoDataLenCurr;
            uartRxFifoDataLenCurr = Cy_SCB_GetNumInRxFifo(SCB1);

            if ((uartRxFifoDataLenCurr > 0x00) &&
                    (uartRxFifoDataLenCurr == uartRxFifoDataLenPrev) && (cy_timer_loop == 0x00)) {
                DBG_APP_TRACE("Start RX_FIFO read len:%x\r\n", uartRxFifoDataLenCurr);

                /* Prepare the DataWire channel to read the available amount of data. */
                Cy_VcomDev_QueueUartRead(pAppCtxt, uartRxFifoDataLenCurr);

                /* SCB.RX_TRIGGER needs to be asserted high manually to get the
                 * DMA transfer to start.
                 */
                Cy_TrigMux_SwTrigger(TRIG_OUT_1TO1_0_SCB1_RX_TO_PDMA0_TR_IN19, CY_TRIGGER_TWO_CYCLES);

                /* Set flag to indicate timer based transfer has been initialized. */
                cy_timer_loop = 0x01;
            }
        }
    }

    xTimerReset(xTimer, 0);
}   /* end of function  */

/*
 * Function: Cy_USB_VbusDebounceTimerCallback()
 * Description: Timer used to do debounce on VBus changed interrupt notification.
 *
 * Parameter:
 *      xTimer: RTOS timer handle.
 * return: void
 */
void
Cy_USB_VbusDebounceTimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;

    if (pAppCtxt->vbusChangeIntr) {
        /* Notify the VCOM task that VBus debounce is complete. */
        xMsg.type = CY_USB_VCOM_VBUS_CHANGE_DEBOUNCED;
        xQueueSendFromISR(pAppCtxt->vcomDeviceQueue, &(xMsg), &(xHigherPriorityTaskWoken));

        /* Clear and re-enable the interrupt. */
        pAppCtxt->vbusChangeIntr = false;
        Cy_GPIO_ClearInterrupt(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);
        Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 1);
    }
}   /* end of function  */



/*
 * Function: Cy_USB_VcomDeviceDmaReadCompletion()
 * Description:  Handler for DMA transfer completion on OUT endpoint
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: void
 */
void Cy_USB_VcomDeviceDmaReadCompletion (void *pApp)
{
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    pAppCtxt = (cy_stc_usb_app_ctxt_t*)pApp;

    xMsg.type = CY_USB_VCOM_DEVICE_MSG_READ_COMPLETE;
    status = xQueueSendFromISR(pAppCtxt->vcomDeviceQueue, &(xMsg),
                               &(xHigherPriorityTaskWoken));
    (void)status;
    return;
}   /* end of function */

/*
 * Function: Cy_USB_VcomDeviceDmaWriteCompletion()
 * Description:  Handler for DMA transfer completion on IN endpoint
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: void
 */
void Cy_USB_VcomDeviceDmaWriteCompletion (void *pApp)
{
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t xMsg;
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    pAppCtxt = (cy_stc_usb_app_ctxt_t*)pApp;
    pAppCtxt->usbInXferPending = false;

    xMsg = CY_USB_VCOM_DEVICE_MSG_WRITE_COMPLETE;
    status = xQueueSendFromISR(pAppCtxt->vcomDeviceQueue, &(xMsg),
                               &(xHigherPriorityTaskWoken));
    (void)status;
    return;
}   /* end of function */

/*
 * Function: Cy_USB_UARTDmaWriteCompletion()
 * Description:  Handler for DMA transfer completion on UART TX_FIFO
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: void
 */
void
Cy_USB_UARTDmaWriteCompletion(void *pApp)
{
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    pAppCtxt = (cy_stc_usb_app_ctxt_t*)pApp;

    xMsg.type = CY_USB_UART_MSG_WRITE_COMPLETE;
    status = xQueueSendFromISR(pAppCtxt->vcomDeviceQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
    (void)status;
    return;

}  /* end of function */


/*
 * Function: Cy_USB_UARTDmaReadCompletion()
 * Description:  Handler for DMA transfer completion on UART RX_FIFO
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: None
 */
void
Cy_USB_UARTDmaReadCompletion(void *pApp)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t*)pApp;
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;

    xMsg.type = CY_USB_UART_MSG_READ_COMPLETE;
    status = xQueueSendFromISR(pAppCtxt->vcomDeviceQueue, &(xMsg),
                               &(xHigherPriorityTaskWoken));
    (void)status;
    return;
}  /* end of function */

/*
 * Function: Cy_USB_AppUartInitsDmaIntr()
 * Description: Function to register an ISR for the DMA channel associated
 *              with an UART Read/Write
 * Parameters:
 *      endpNumber: UART READ/WRITE
 *      userIsr: ISR function pointer. Can be NULL if interrupt is to be
 *               disabled.
 * return: void
 */
void
Cy_USB_AppUartInitsDmaIntr(uint8_t uart_read_write,cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;

    DBG_APP_INFO("Init Uart_Dma_Intr\r\n");

    if (uart_read_write == UART_WRITE) {
        /* UART Write Interrupt Source */
#if CY_CPU_CORTEX_M4
        intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + UART_WRITE_DMA_CHANNEL);
        intrCfg.intrPriority = 5;
#else
        intrCfg.intrSrc = NvicMux5_IRQn;
        intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + UART_WRITE_DMA_CHANNEL);
        intrCfg.intrPriority = 2;
#endif /* CY_CPU_CORTEX_M4 */
    } else {
        /* UART Read Interrupt Source */
#if CY_CPU_CORTEX_M4
        intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + UART_READ_DMA_CHANNEL);
        intrCfg.intrPriority = 5;
#else
        intrCfg.intrSrc = NvicMux6_IRQn;
        intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + UART_READ_DMA_CHANNEL);
        intrCfg.intrPriority = 2;
#endif /* CY_CPU_CORTEX_M4 */
    }

    if (userIsr != NULL)  {
        /* If an ISR is provided, register it and enable the interrupt. */
        Cy_SysInt_Init(&intrCfg, userIsr);
        NVIC_EnableIRQ(intrCfg.intrSrc);
    } else {
        /* ISR is NULL. Disable the interrupt. */
        NVIC_DisableIRQ(intrCfg.intrSrc);
    }

    return;
} /* end of function. */



/*
 * Function: Cy_USB_AppClearUartDmaInterrupt()
 * Description: Function to clear the pending DMA interrupt associated with an
 *              UART.
 * Parameters:
 *      pAppCtxt: Pointer to USB application context structure.
 *      uart_read_write: UART Read/Write
 * return: void
 */
void
Cy_USB_AppClearUartDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,uint8_t uart_read_write)
{

    if ((pAppCtxt != NULL)) {
        if (uart_read_write == UART_WRITE) {
            Cy_DMA_Channel_ClearInterrupt(pAppCtxt->pCpuDw0Base, UART_WRITE_DMA_CHANNEL);
        } else if (uart_read_write == UART_READ) {
            Cy_DMA_Channel_ClearInterrupt(pAppCtxt->pCpuDw0Base, UART_READ_DMA_CHANNEL);
        } else {
            DBG_APP_ERR("ClearUartDmaInterrupt Err\r\n");
        }
    }

    return;
}  /* end of function. */


/*
 * Function: Cy_USB_AppUARTWrite()
 * Description: Function to write operation on an UART TX_FIFO.
 * Parameter: pBuffer, dataSize
 * return: void
 */
void
Cy_USB_AppUARTWrite (uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_dma_descriptor_config_t desc_cfg;
    cy_stc_dma_channel_config_t    chan_cfg;
    cy_en_dma_status_t stat;

    /* If the DMA channel is already enabled, disable it. */
    Cy_DMA_Channel_Disable(DW0, UART_WRITE_DMA_CHANNEL);

    if (dataSize > VCOM_UART_MAX_RD_SIZE) {
        /* Data may not fit in the SCB TX FIFO. Use 2-D DMA
         * with wait for TRIGGER from SCB after each 64 bytes are
         * transferred. A second descriptor is used to send any
         * residual portion of data at the end.
         */
        if ((dataSize % VCOM_UART_MAX_RD_SIZE) != 0) {
            desc_cfg.retrigger          = CY_DMA_RETRIG_16CYC;
            desc_cfg.interruptType      = CY_DMA_DESCR_CHAIN;
            desc_cfg.triggerOutType     = CY_DMA_DESCR_CHAIN;
            desc_cfg.triggerInType      = CY_DMA_DESCR;
            desc_cfg.dataSize           = CY_DMA_BYTE;
            desc_cfg.srcTransferSize    = CY_DMA_TRANSFER_SIZE_DATA;
            desc_cfg.dstTransferSize    = CY_DMA_TRANSFER_SIZE_WORD;
            desc_cfg.descriptorType     = CY_DMA_1D_TRANSFER;
            desc_cfg.srcAddress         = (void *)(pBuffer + (dataSize & (~(VCOM_UART_MAX_RD_SIZE - 1))));
            desc_cfg.dstAddress         = (void *)(&SCB1->TX_FIFO_WR);
            desc_cfg.srcXincrement      = 1;
            desc_cfg.dstXincrement      = 0;
            desc_cfg.xCount             = dataSize % VCOM_UART_MAX_RD_SIZE;
            desc_cfg.srcYincrement      = 0;
            desc_cfg.dstYincrement      = 0;
            desc_cfg.yCount             = 0;
            desc_cfg.channelState       = CY_DMA_CHANNEL_DISABLED;
            desc_cfg.nextDescriptor     = NULL;
            stat = Cy_DMA_Descriptor_Init (&gUartWriteDmaDesc2, &desc_cfg);
            if (stat != CY_DMA_SUCCESS) {
                DBG_APP_ERR("DMA descriptor init failed\r\n");
                return;
            }

            /* Descriptor 1 needs to be linked to descriptor 2. */
            desc_cfg.channelState       = CY_DMA_CHANNEL_ENABLED;
            desc_cfg.nextDescriptor     = &gUartWriteDmaDesc2;
        } else {
            desc_cfg.channelState       = CY_DMA_CHANNEL_DISABLED;
            desc_cfg.nextDescriptor     = NULL;
        }

        desc_cfg.retrigger          = CY_DMA_RETRIG_16CYC;
        desc_cfg.interruptType      = CY_DMA_DESCR_CHAIN;
        desc_cfg.triggerOutType     = CY_DMA_DESCR_CHAIN;
        desc_cfg.triggerInType      = CY_DMA_X_LOOP;
        desc_cfg.dataSize           = CY_DMA_BYTE;
        desc_cfg.srcTransferSize    = CY_DMA_TRANSFER_SIZE_DATA;
        desc_cfg.dstTransferSize    = CY_DMA_TRANSFER_SIZE_WORD;
        desc_cfg.descriptorType     = CY_DMA_2D_TRANSFER;
        desc_cfg.srcAddress         = (void *)pBuffer;
        desc_cfg.dstAddress         = (void *)(&SCB1->TX_FIFO_WR);
        desc_cfg.srcXincrement      = 1;
        desc_cfg.dstXincrement      = 0;
        desc_cfg.xCount             = VCOM_UART_MAX_RD_SIZE;
        desc_cfg.srcYincrement      = VCOM_UART_MAX_RD_SIZE;
        desc_cfg.dstYincrement      = 0;
        desc_cfg.yCount             = (dataSize / VCOM_UART_MAX_RD_SIZE);
    } else {
        /* All the available data will fill in the SCB TX FIFO.
         * Use 1-D DMA to transfer the entire data in one burst.
         */
        desc_cfg.retrigger          = CY_DMA_RETRIG_4CYC;
        desc_cfg.interruptType      = CY_DMA_DESCR;
        desc_cfg.triggerOutType     = CY_DMA_DESCR;
        desc_cfg.channelState       = CY_DMA_CHANNEL_DISABLED;
        desc_cfg.triggerInType      = CY_DMA_DESCR;
        desc_cfg.dataSize           = CY_DMA_BYTE;
        desc_cfg.srcTransferSize    = CY_DMA_TRANSFER_SIZE_DATA;
        desc_cfg.dstTransferSize    = CY_DMA_TRANSFER_SIZE_WORD;
        desc_cfg.descriptorType     = CY_DMA_1D_TRANSFER;
        desc_cfg.srcAddress         = (void *)pBuffer;
        desc_cfg.dstAddress         = (void *)(&SCB1->TX_FIFO_WR);
        desc_cfg.srcXincrement      = 1;
        desc_cfg.dstXincrement      = 0;
        desc_cfg.xCount             = dataSize;
        desc_cfg.srcYincrement      = 0;
        desc_cfg.dstYincrement      = 0;
        desc_cfg.yCount             = 1;
        desc_cfg.nextDescriptor     = NULL;
    }

    stat = Cy_DMA_Descriptor_Init (&gUartWriteDmaDesc1, &desc_cfg);
    if (stat != CY_DMA_SUCCESS) {
        DBG_APP_ERR("DMA descriptor init failed\r\n");
    } else {
        chan_cfg.descriptor  = &gUartWriteDmaDesc1;
        chan_cfg.preemptable = false;
        chan_cfg.priority    = 0;
        chan_cfg.enable      = false;
        chan_cfg.bufferable  = false;

        stat = Cy_DMA_Channel_Init (DW0, UART_WRITE_DMA_CHANNEL, &chan_cfg);
        if (stat != CY_DMA_SUCCESS) {
            DBG_APP_ERR("DMA channel init failed\r\n");
        } else {
            /* Make sure SLOW AHB read cache is evicted before new transfer is started. */
            Cy_HBDma_EvictReadCache(false);

            Cy_DMA_Channel_Enable(DW0, UART_WRITE_DMA_CHANNEL);
            Cy_DMA_Channel_SetInterruptMask(DW0,UART_WRITE_DMA_CHANNEL,CY_DMA_INTR_MASK);
        }
    }

    return;
}  /* end of function */

/*
 * Function: CY_AppUartHbwDmaCallback()
 * Description: HbwDMA callback whenever HBW-DMA transfer completes.
 * Parameter: cy_stc_hbdma_channel, cy_en_hbdma_cb_type_t,
 *            cy_stc_hbdma_buff_status_t, void* .
 * return: None.
 */
void
Cy_AppUartHbwDmaCallback (cy_stc_hbdma_channel_t *pHandle,
                          cy_en_hbdma_cb_type_t type,
                          cy_stc_hbdma_buff_status_t* pbufStat, void *userCtx)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    switch (type) {
        case CY_HBDMA_CB_CONS_EVENT:
            /*
             * This event means DATA is consumed and buffer is
             * available so next set of transfer can be initiated
             * from UART->HbwSRAM->USB
             */
            DBG_APP_TRACE("HBWDMA CONS event\r\n");

            /* Data has been consumed on USB side and buffer is free now. */
            if (pAppCtxt->uartRxFreeBufCount == 0) {
                /* Queue read from the UART RX FIFO into the next DMA buffer. */
                Cy_VcomDev_QueueUartRead(pAppCtxt, VCOM_UART_MAX_RD_SIZE);
            }
            pAppCtxt->uartRxFreeBufCount++;

            /*
             * CONS_EVENT can come for SHORT as well as for FULL packet
             * so always reset short related settings in TR_ASSIST logic.
             */
            MAIN_REG->TR_ASSIST_GR[0].TRA_SLP_CTRL[0] &=
                    (~MAIN_REG_TR_ASSIST_GR_TRA_SLP_CTRL_TRA_SHORT_CONFIG_Msk);
            MAIN_REG->TR_ASSIST_GR[0].TRA_SLP_CTRL[0] &=
                    (~MAIN_REG_TR_ASSIST_GR_TRA_SLP_CTRL_TRA_BYTE_COUNT_Msk);
            break;

        case CY_HBDMA_CB_PROD_EVENT:
            /* Increment the number of occupied data buffers and store
             * the size of the data received.
             */
            pAppCtxt->usbRxDataLen[pAppCtxt->nxtUsbRxBufIdx] = pbufStat->count;
            pAppCtxt->nxtUsbRxBufIdx++;
            if (pAppCtxt->nxtUsbRxBufIdx >= VCOM_APP_BUFFER_CNT) {
                pAppCtxt->nxtUsbRxBufIdx = 0;
            }
            pAppCtxt->usbRxBusyBufCount++;

            /* If this is the first occupied buffer, we can queue UART write. */
            if (pAppCtxt->usbRxBusyBufCount == 1) {
                MAIN_REG->TR_ASSIST_GR[0].TRA_CTRL[1] &=
                    (~MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_STREAM_EN_Msk);

                if (pbufStat->count < 0x400U) {
                    /* If this is a short packet, configure the count accordingly. */
                    MAIN_REG->TR_ASSIST_GR[0].TRA_SLP_CTRL[1] =
                        MAIN_REG_TR_ASSIST_GR_TRA_SLP_CTRL_TRA_SHORT_CONFIG_Msk |
                        (pbufStat->count << MAIN_REG_TR_ASSIST_GR_TRA_SLP_CTRL_TRA_BYTE_COUNT_Pos);
                } else {
                    MAIN_REG->TR_ASSIST_GR[0].TRA_SLP_CTRL[1] = 0;
                }

                MAIN_REG->TR_ASSIST_GR[0].TRA_CTRL[1] = pHandle->nextProdDscr |
                    MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_STREAM_EN_Msk |
                    MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_PKT_MODE_Msk;

                /* Configure the DMA channel to write the received data to the SCB
                 * transmit FIFO. */
                Cy_USB_AppUARTWrite((uint8_t *)pbufStat->pBuffer, pbufStat->count);
            }
            break;

        default:
            break;
    }
    return;
}

/*
 * Function: Cy_USB_AppUartFramework()
 * Description: This function creates setup for end to end transfer between
 *              UART to USBSS.
 * Parameter: pUsbApp, pEndpDscr,
 * Return: None.
 */
void
Cy_USB_AppUartFramework (cy_stc_usb_app_ctxt_t *pUsbApp,
                         uint8_t *pEndpDscr)
{
    cy_stc_hbdma_chn_config_t hbDmaChnConfig;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint32_t endpNumber, endpDirection, endpType;
    uint16_t maxPktSize;
    uint8_t *pCompDscr;
    uint8_t  burstSize = 1;
    cy_en_hbdma_mgr_status_t hbwDmaMgrStatus = CY_HBDMA_MGR_SUCCESS;
    uint16_t currDscrNum;

    /*
     * First configure HBWDMA  so that you can get buffer pointer
     * from HbwRam and use the same as destination while configuring
     * CPUSS DMA for IN transfer.
     */
    Cy_USBD_GetEndpType(pEndpDscr, &endpType);
    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &endpDirection);
    if (pUsbApp->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
        pCompDscr = Cy_USBD_GetSsEndpCompDscr(pUsbApp->pUsbdCtxt, pEndpDscr);
        Cy_USBD_GetEndpCompnMaxburst(pCompDscr, &burstSize);
    }

    /*
     * Config channel, Create Channel, Get buffer and use that
     * buffer for CPUSS DMA.
     */
    hbDmaChnConfig.bufferMode     = false;                      /* DMA buffer mode disabled */
    hbDmaChnConfig.prodHdrSize    = 0;                          /* No header being added. */
    hbDmaChnConfig.prodSckCount   = 1;                          /* No. of producer sockets */
    hbDmaChnConfig.consSckCount   = 1;                          /* No. of consumer Sockets */
    hbDmaChnConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;    /* Producer Socket ID: None */
    hbDmaChnConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;    /* Consumer Socket ID: None */
    hbDmaChnConfig.eventEnable    = 1;                          /* TR_ASSIST connects to events from the adapter. */
    hbDmaChnConfig.userCtx        = (void *)(pUsbApp);          /* Pass the application context as user context. */

    if (endpDirection != 0) {
        /*
         * Its IN endpoint which means DATA flow from
         * UART->USBSSDMA->HBWRAM->USBSSEPM->Host
         */
        pEndpDmaSet = &(pUsbApp->endpInDma[endpNumber]);

        /* Skip creating DMA channel for the Interrupt endpoint at present. */
        if (endpType != CY_USB_ENDP_TYPE_INTR) {
            if (pUsbApp->pUartToUsbChn != NULL) {
                DBG_APP_ERR("BULK-IN channel already created\r\n");
                Cy_HBDma_Channel_Disable(pUsbApp->pUartToUsbChn);
                Cy_HBDma_Channel_Destroy(pUsbApp->pUartToUsbChn);
                pUsbApp->pUartToUsbChn = NULL;
            }

            /* Create channel which will move data from SRAM to USBSS EPM. */
            hbDmaChnConfig.size           = VCOM_UART_MAX_RD_SIZE;      /* DMA Buffer size in bytes */
            hbDmaChnConfig.count          = VCOM_APP_BUFFER_CNT;        /* DMA Buffer Count */
            hbDmaChnConfig.prodBufSize    = VCOM_UART_MAX_RD_SIZE;
            hbDmaChnConfig.chType         = CY_HBDMA_TYPE_MEM_TO_IP;    /* DMA Channel type: from HB-RAM to USB3-IP */
            hbDmaChnConfig.prodSck[0]     = CY_HBDMA_VIRT_SOCKET_WR;
            hbDmaChnConfig.consSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + endpNumber);
            hbDmaChnConfig.cb             = Cy_AppUartHbwDmaCallback; /* HB-DMA callback */
            hbDmaChnConfig.intrEnable     = USB32DEV_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;

            hbwDmaMgrStatus = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                                                      &(pEndpDmaSet->hbDmaChannel),
                                                      &hbDmaChnConfig);

            if (hbwDmaMgrStatus != CY_HBDMA_MGR_SUCCESS) {
                /* Not able to create Channel then return. */
                DBG_APP_ERR("EP %x-IN: channel create failed 0x%x\r\n", endpNumber, hbwDmaMgrStatus);
                return;
            }

            /* Store the pointer to the DMA channel. */
            pUsbApp->pUartToUsbChn = &(pEndpDmaSet->hbDmaChannel);
            pUsbApp->vcomInEpNum   = endpNumber;

            /* Get the list of DMA buffers to be used for UART to USB transfers. */
            Cy_HBDma_Channel_GetBufferInfo(pUsbApp->pUartToUsbChn, pUsbApp->pUartRxBuffer, VCOM_APP_BUFFER_CNT);
            pUsbApp->uartRxBufIndex     = 0;
            pUsbApp->uartRxFreeBufCount = VCOM_APP_BUFFER_CNT;
            pUsbApp->nxtUsbTxBufIdx     = 0;
            pUsbApp->usbInXferPending   = false;

            /* Housekeeping and Make sure endpoint is marked valid */
            pEndpDmaSet->maxPktSize = maxPktSize;
            pEndpDmaSet->valid      = 0x01;
            DBG_APP_INFO("HBDMA BulkIn endpNum:%x ChnCreate status: %x\r\n",
                    endpNumber,hbwDmaMgrStatus);

            if (pUsbApp->devSpeed > CY_USBD_USB_DEV_HS) {
                /* Enable the channel for data transfer. */
                hbwDmaMgrStatus = Cy_HBDma_Channel_Enable(pUsbApp->pUartToUsbChn, 0);
                DBG_APP_INFO("HBDMA UART to USB channel enable status: %x\r\n", hbwDmaMgrStatus);

                /* Connect the output trigger of the DataWire channel to the TR_ASSIST input. */
                Cy_TrigMux_Connect(TRIG_IN_MUX_9_PDMA0_TR_OUT19,
                        TRIG_OUT_MUX_9_LVDSUSB32SS_TR_IN0,
                        false, TRIGGER_TYPE_EDGE);

                /* Connect TR_ASSIST trigger to the USB egress socket trigger input. */
                MAIN_REG->TR_GR[0].TR_CTL[0x10 + endpNumber] = 0x60;

                /* Configure the trigger control register to generate the event data required. */
                currDscrNum = pUsbApp->pUartToUsbChn->curConsDscrIndex[0];
                MAIN_REG->TR_ASSIST_GR[0].TRA_CTRL[0] = currDscrNum |
                    MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_HBWSS_IS_CONS_Msk |
                    MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_STREAM_EN_Msk |
                    MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_PKT_MODE_Msk;

                /* Enable DataWire channel to read VCOM_UART_MAX_RD_SIZE bytes from the UART RX_FIFO. */
                Cy_VcomDev_QueueUartRead(pUsbApp, VCOM_UART_MAX_RD_SIZE);
            }

            xTimerReset(pUsbApp->vcomTimerHandle, 0);
        }
    } else {
        /* Its OUT endpoint which means data Host->UART Device. */
        pEndpDmaSet = &(pUsbApp->endpOutDma[endpNumber]);

        /* Only one OUT endpoint is expected. Ensure that OUT channel has not been created. */
        if (pUsbApp->pUsbToUartChn != NULL) {
            DBG_APP_ERR("BULK-OUT channel already created\r\n");
            Cy_HBDma_Channel_Disable(pUsbApp->pUsbToUartChn);
            Cy_HBDma_Channel_Destroy(pUsbApp->pUsbToUartChn);
            pUsbApp->pUsbToUartChn = NULL;
        }

        /* Create channel which moves data from USB ingress endpoint into HBW SRAM. */
        hbDmaChnConfig.size        = maxPktSize * burstSize;    /* DMA Buffer Size in bytes */
        hbDmaChnConfig.count       = VCOM_APP_BUFFER_CNT;       /* DMA Buffer Count */
        hbDmaChnConfig.prodBufSize = maxPktSize * burstSize;
        hbDmaChnConfig.chType      = CY_HBDMA_TYPE_IP_TO_MEM;
        hbDmaChnConfig.prodSck[0]  = (cy_hbdma_socket_id_t)(CY_HBDMA_USBIN_SOCKET_00 + endpNumber);
        hbDmaChnConfig.consSck[0]  = CY_HBDMA_VIRT_SOCKET_RD;
        hbDmaChnConfig.cb          = Cy_AppUartHbwDmaCallback;  /* HB-DMA callback */
        hbDmaChnConfig.intrEnable  = USB32DEV_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk;

        hbwDmaMgrStatus = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                                                  &(pEndpDmaSet->hbDmaChannel),
                                                  &hbDmaChnConfig);
        if (hbwDmaMgrStatus != CY_HBDMA_MGR_SUCCESS) {
            /* Not able to create Channel then return. */
            DBG_APP_ERR("EP %x-OUT: channel create failed 0x%x\r\n", endpNumber, hbwDmaMgrStatus);
            return;
        }

        /* Store the DMA channel handle. */
        pUsbApp->pUsbToUartChn = &(pEndpDmaSet->hbDmaChannel);
        pUsbApp->vcomOutEpNum  = endpNumber;

        /* Make sure endpoint is marked valid */
        pEndpDmaSet->maxPktSize = maxPktSize;
        pEndpDmaSet->valid      = 0x01;

        /* Get the list of DMA buffers to be used for UART to USB transfers. */
        Cy_HBDma_Channel_GetBufferInfo(pUsbApp->pUsbToUartChn, pUsbApp->pUsbRxBuffer, VCOM_APP_BUFFER_CNT);
        pUsbApp->nxtUsbRxBufIdx    = 0;
        pUsbApp->uartTxBufIndex    = 0;
        pUsbApp->usbRxBusyBufCount = 0;

        if (pUsbApp->devSpeed > CY_USBD_USB_DEV_HS) {
            /* Connect the trigger output from the DataWire channel to the TR_ASSIST logic input. */
            Cy_TrigMux_Connect(TRIG_IN_MUX_9_PDMA0_TR_OUT18,
                    TRIG_OUT_MUX_9_LVDSUSB32SS_TR_IN1,
                    false, TRIGGER_TYPE_EDGE);

            /* Connect TR_ASSIST trigger to the trigger input of the ingress socket. */
            MAIN_REG->TR_GR[0].TR_CTL[endpNumber] = 0x61;

            /* Configure TR_ASSIST control register to update the correct descriptor. */
            currDscrNum = pUsbApp->pUsbToUartChn->curProdDscrIndex[0];
            MAIN_REG->TR_ASSIST_GR[0].TRA_CTRL[1] = currDscrNum |
                MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_STREAM_EN_Msk |
                MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_PKT_MODE_Msk;

            /* Enable the channel for data transfer. */
            hbwDmaMgrStatus = Cy_HBDma_Channel_Enable(pUsbApp->pUsbToUartChn, 0);
            DBG_APP_INFO("HBDMA BulkOut endpNumber:%x ChnEnab status: %x\r\n",endpNumber,hbwDmaMgrStatus);
        }
    }
}

/*
 * Function: Cy_USB_AppUartRead()
 * Description: This function activate CPUSS DMA to read from UART RX_FIFO and
 *              put dataSize length of data into SRAM buffer.
 * Parameter: pHbSramBuffer, dataSize
 * return: void
 */
void
Cy_USB_AppUartRead (uint8_t *pHbSramBuffer, uint16_t dataSize)
{
    cy_stc_dma_descriptor_config_t desc_cfg;
    cy_stc_dma_channel_config_t    chan_cfg;
    cy_en_dma_status_t stat;

    /* If the DMA channel is already enabled, disable it. */
    Cy_DMA_Channel_Disable (DW0, UART_READ_DMA_CHANNEL);

    desc_cfg.retrigger          = CY_DMA_RETRIG_4CYC;
    desc_cfg.interruptType      = CY_DMA_DESCR;
    desc_cfg.triggerOutType     = CY_DMA_DESCR;
    desc_cfg.channelState       = CY_DMA_CHANNEL_DISABLED;
    desc_cfg.triggerInType      = CY_DMA_DESCR_CHAIN;
    desc_cfg.dataSize           = CY_DMA_BYTE;
    desc_cfg.srcTransferSize    = CY_DMA_TRANSFER_SIZE_WORD;
    desc_cfg.dstTransferSize    = CY_DMA_TRANSFER_SIZE_DATA;
    desc_cfg.descriptorType     = CY_DMA_1D_TRANSFER;
    desc_cfg.srcAddress         = (void *)(&SCB1->RX_FIFO_RD);
    desc_cfg.dstAddress         = (void *)pHbSramBuffer;
    desc_cfg.srcXincrement      = 0u;
    desc_cfg.dstXincrement      = 1u;
    desc_cfg.xCount             = dataSize;
    desc_cfg.srcYincrement      = 0;
    desc_cfg.dstYincrement      = 0;
    desc_cfg.yCount             = 1;
    desc_cfg.nextDescriptor     = NULL;

    stat = Cy_DMA_Descriptor_Init (&gUartReadlDmaDesc, &desc_cfg);
    if (stat != CY_DMA_SUCCESS) {
        DBG_APP_ERR("DMA descriptor init failed\r\n");
    } else {
        chan_cfg.descriptor  = &gUartReadlDmaDesc;
        chan_cfg.preemptable = false;
        chan_cfg.priority    = 0;
        chan_cfg.enable      = false;
        chan_cfg.bufferable  = false;

        stat = Cy_DMA_Channel_Init (DW0, UART_READ_DMA_CHANNEL, &chan_cfg);
        if (stat != CY_DMA_SUCCESS) {
            DBG_APP_ERR("DMA channel init failed\r\n");
        } else {
            Cy_DMA_Channel_SetInterruptMask(DW0,UART_READ_DMA_CHANNEL,CY_DMA_INTR_MASK);
            Cy_DMA_Channel_Enable (DW0, UART_READ_DMA_CHANNEL);
        }
    }

    return;
}  /* end of function */

/*
 * Function: Cy_USB_AppInit()
 * Description: This function Initializes application related data structures,
 *              register callback and creates queue and task for device
 *              function.
 * Parameter: cy_stc_usb_app_ctxt_t, cy_stc_usb_usbd_ctxt_t, DMAC_Type
 *            DW_Type, DW_Type, cy_stc_hbdma_mgr_context_t*
 * return: None.
 * Note: This function should be called after USBD_Init()
 */
void
Cy_USB_AppInit (cy_stc_usb_app_ctxt_t *pAppCtxt,
                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, DMAC_Type *pCpuDmacBase,
                DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
                cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt)
{
    uint32_t index;
    BaseType_t status = pdFALSE;
    cy_stc_app_endp_dma_set_t *pEndpInDma;
    cy_stc_app_endp_dma_set_t *pEndpOutDma;

    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pAppCtxt->devAddr = 0x00;
    pAppCtxt->activeCfgNum = 0x00;
    pAppCtxt->prevAltSetting = 0x00;
    pAppCtxt->pHbDmaMgrCtxt = pHbDmaMgrCtxt;

    for (index=0x00; index < CY_USB_MAX_ENDP_NUMBER; index++) {
        pEndpInDma = &(pAppCtxt->endpInDma[index]);
        memset((void *)pEndpInDma, 0, sizeof(cy_stc_app_endp_dma_set_t));

        pEndpOutDma = &(pAppCtxt->endpOutDma[index]);
        memset((void *)pEndpOutDma, 0, sizeof(cy_stc_app_endp_dma_set_t));
    }

    pAppCtxt->pCpuDmacBase = pCpuDmacBase;
    pAppCtxt->pCpuDw0Base = pCpuDw0Base;
    pAppCtxt->pCpuDw1Base = pCpuDw1Base;
    pAppCtxt->pUsbdCtxt = pUsbdCtxt;
    pAppCtxt->dataXferIntrEnabled = false;

    /*
     * Callbacks registered with USBD layer. These callbacks will be called
     * based on appropriate event.
     */
    Cy_USB_AppRegisterCallback(pAppCtxt);

    if (!(pAppCtxt->firstInitDone)) {
        pAppCtxt->pUartToUsbChn  = NULL;
        pAppCtxt->pUsbToUartChn  = NULL;
        pAppCtxt->vbusChangeIntr = false;
        pAppCtxt->vbusPresent    = false;
        pAppCtxt->usbConnected   = false;

        /* create queue and register it to kernel. */
        pAppCtxt->vcomDeviceQueue = xQueueCreate(CY_USB_VCOM_DEVICE_MSG_QUEUE_SIZE,
                                        CY_USB_VCOM_DEVICE_MSG_SIZE);

        if (pAppCtxt->vcomDeviceQueue == NULL)
        {
            DBG_APP_ERR("QueuecreateFail\r\n");
            return;
        }
        DBG_APP_INFO("createdVcomQueue\r\n");
        vQueueAddToRegistry(pAppCtxt->vcomDeviceQueue, "VCOMDeviceMsgQueue");

        /* Create task and check status to confirm task created properly. */
        status = xTaskCreate(Cy_USB_VcomDeviceTaskHandler, "VCOMDeviceTask", 2048,
                        (void *)pAppCtxt, 5, &(pAppCtxt->vcomDevicetaskHandle));

        if (status != pdPASS) {
            DBG_APP_ERR("TaskcreateFail\r\n");
            return;
        }

        pAppCtxt->vcomtimerExpiry = VCOM_RESPONSE_TIME;
        pAppCtxt->vcomTimerHandle = xTimerCreate("VcomTimer", pAppCtxt->vcomtimerExpiry, pdFALSE,
                (void *)pAppCtxt, Cy_USB_AppVcomTimerCallback);
        pAppCtxt->vbusDebounceTimer = xTimerCreate("VbusDebounceTimer", 200, pdFALSE,
                (void *)pAppCtxt, Cy_USB_VbusDebounceTimerCallback);

        if ((pAppCtxt->vcomTimerHandle == NULL) || (pAppCtxt->vbusDebounceTimer == NULL)) {
            DBG_APP_ERR("TimerCreateFail\r\n");
            return;
        }

        DBG_APP_INFO("xTimer Created\r\n");
        pAppCtxt->firstInitDone = 0x01;
    }

    return;
}   /* end of function. */

/*
 * Function: Cy_USB_AppSetAddressCallback()
 * Description: This Function will be called by USBD layer when
 *              a USB address has been assigned to the device.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppSetAddressCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    /* Update the state variables. */
    pAppCtxt->devState     = CY_USB_DEVICE_STATE_ADDRESS;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DEFAULT;
    pAppCtxt->devAddr      = pUsbdCtxt->devAddr;
    pAppCtxt->devSpeed     = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);

    /* Check the type of USB connection and register appropriate descriptors. */
    Cy_USB_VcomDevice_RegisterDescriptors(pAppCtxt->pUsbdCtxt, pAppCtxt->devSpeed);

#if (!CY_CPU_CORTEX_M4)
    /*
     * Map NvicMux2 and NvicMux3 to High BandWidth DMA adapter interrupts by default.
     * If we are about to start transfers in HS mode, it will be switched to DW interrupts.
     */
    Cy_SysInt_SetVector(NvicMux2_IRQn, Cy_USB_EgressDma_ISR);
    Cy_SysInt_SetVector(NvicMux3_IRQn, Cy_USB_IngressDma_ISR);
#endif /* (!CY_CPU_CORTEX_M4) */
}

/*
 * Function: Cy_USB_AppRegisterCallback()
 * Description: This function will register all calback with USBD layer.
 * Parameter: cy_stc_usb_app_ctxt_t.
 * return: None.
 */
void
Cy_USB_AppRegisterCallback (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET,
                                               Cy_USB_AppBusResetCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE,
                                           Cy_USB_AppBusResetDoneCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED,
                                               Cy_USB_AppBusSpeedCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP,
                                                  Cy_USB_AppSetupCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND,
                                                Cy_USB_AppSuspendCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME,
                                                 Cy_USB_AppResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG,
                                                 Cy_USB_AppSetCfgCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETADDR,
                                                 Cy_USB_AppSetAddressCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF,
                                                Cy_USB_AppSetIntfCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_SLEEP,
                                                Cy_USB_AppL1SleepCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_RESUME,
                                                Cy_USB_AppL1ResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_ZLP,
                                                Cy_USB_AppZlpCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SLP,
                                                Cy_USB_AppSlpCallback);
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppSetupEndpDmaParamsHs()
 * Description: This Function will setup Endpoint and DMA related parameters
 *             for high speed device before transfer initiated.
 * Parameter: cy_stc_usb_app_ctxt_t, pEndpDscr
 * return: void
 */
static void
Cy_USB_AppSetupEndpDmaParamsHs (cy_stc_usb_app_ctxt_t *pUsbApp,
                              uint8_t *pEndpDscr)
{
    uint32_t endpNumber;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    DW_Type *pDW;
    uint16_t maxPktSize = 0x00;
    cy_en_usb_endp_dir_t endpDirection;
    bool stat;

    endpNumber = ((*(pEndpDscr+CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);
    Cy_USBD_GetEndpMaxPktSize(pEndpDscr, &maxPktSize);

    if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80) {
        endpDirection = CY_USB_ENDP_DIR_IN;
        pEndpDmaSet   = &(pUsbApp->endpInDma[endpNumber]);
        pDW           = pUsbApp->pCpuDw1Base;
    } else {
        endpDirection = CY_USB_ENDP_DIR_OUT;
        pEndpDmaSet   = &(pUsbApp->endpOutDma[endpNumber]);
        pDW           = pUsbApp->pCpuDw0Base;
    }

    DBG_APP_INFO("AppSetupEndpDmaParamsHs: endpNum:0x%x maxPktSize:0x%x "
                 "dir:0x%x \r\n", endpNumber, maxPktSize, endpDirection);

    stat = Cy_USBHS_App_EnableEpDmaSet(pEndpDmaSet, pDW, endpNumber, endpNumber, endpDirection, maxPktSize);
    DBG_APP_INFO("Enable EPDmaSet: endp=%x dir=%x stat=%x\r\n", endpNumber, endpDirection, stat);

}   /* end of function  */

/*
 * Function: Cy_USB_AppSetupEndpDmaParams()
 * Description: This Function will setup Endpoint and DMA related parameters
 *              before transfer initiated.
 * Parameter: cy_stc_usb_app_ctxt_t, pEndpDscr
 * return: void
 */
void
Cy_USB_AppSetupEndpDmaParams (cy_stc_usb_app_ctxt_t *pUsbApp,
                              uint8_t *pEndpDscr)
{
    if (pUsbApp->devSpeed <= CY_USBD_USB_DEV_HS) {
        Cy_USB_AppSetupEndpDmaParamsHs (pUsbApp, pEndpDscr);
    }

    /* The High BandWidth Channel Creation code needs to be run in all cases
     * as the DMA buffers are allocated as part of it. */
    Cy_USB_AppUartFramework(pUsbApp, pEndpDscr);
}


/*
 * Function: Cy_USB_AppConfigureEndp()
 * Description: This Function is used by application to configure endpoints
 *              after set configuration.  This function should be used for
 *              all endpoints except endp0.
 * Parameter: cy_stc_usb_usbd_ctxt_t, pEndpDscr
 * return: void
 */
void
Cy_USB_AppConfigureEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr)
{
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_endp_dir_t endpDirection;
    bool valid;
    uint32_t endpType;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;
    uint8_t isoPkts = 0x00;
    uint8_t burstSize = 0x00;
    uint8_t maxStream = 0x00;
    uint8_t interval = 0x00;
    uint8_t *pCompDscr = NULL;
    cy_en_usbd_ret_code_t usbdRetCode;


    /* If it is not endpoint descriptor then return */
    if (!Cy_USBD_EndpDscrValid(pEndpDscr)) {
        return;
    }
    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

    if (dir) {
        endpDirection = CY_USB_ENDP_DIR_IN;
    } else {
        endpDirection = CY_USB_ENDP_DIR_OUT;
    }
    Cy_USBD_GetEndpType(pEndpDscr, &endpType);

    if ((CY_USB_ENDP_TYPE_ISO == endpType) || (CY_USB_ENDP_TYPE_INTR == endpType)) {
        /* The ISOINPKS setting in the USBHS register is the actual packets per microframe value. */
        isoPkts = (
                (*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK)
                >> CY_USB_ENDP_ADDL_XN_POS) + 1;
    }

    valid = 0x01;
    if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        /* Get companion descriptor and from there get burstSize. */
        pCompDscr = Cy_USBD_GetSsEndpCompDscr(pUsbdCtxt, pEndpDscr);
        Cy_USBD_GetEndpCompnMaxburst(pCompDscr, &burstSize);
        Cy_USBD_GetEndpCompnMaxStream(pCompDscr, &maxStream);
        if (CY_USB_ENDP_TYPE_ISO == endpType) {
            Cy_USBD_GetEndpCompnAttribute(pCompDscr, &isoPkts);
            isoPkts = (isoPkts & 0x03U) + 0x01U;
            isoPkts *= burstSize;

            /* Fetch the endpoint service interval. */
            Cy_USBD_GetEndpInterval(pEndpDscr, &interval);
        }
    }

    /* Prepare endpointConfig parameter. */
    endpConfig.endpType = (cy_en_usb_endp_type_t)endpType;
    endpConfig.endpDirection = endpDirection;
    endpConfig.valid = valid;
    endpConfig.endpNumber = endpNumber;
    endpConfig.maxPktSize = (uint32_t)maxPktSize;
    endpConfig.isoPkts = isoPkts;
    endpConfig.burstSize = burstSize;
    endpConfig.streamID = maxStream;
    endpConfig.interval = interval;

    /*
       Since we will mostly be receiving short packets in the
       VCOM application, it is preferred to block the reception
       of a new packet until the previous short packet has been
       read out from the EPM.
     */
    endpConfig.allowNakTillDmaRdy = true;
    usbdRetCode = Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

    /* Print status of the endpoint configuration to help debug. */
    DBG_APP_INFO("#ENDPCFG: %d, %d\r\n", endpNumber, usbdRetCode);

    return;
}   /* end of function */

/*
 * Function: Cy_USB_AppSetCfgCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppSetCfgCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_cal_msg_t *pMsg)
{

    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
    uint8_t index, numOfIntf, numOfEndp;
    cy_en_usb_speed_t devSpeed;

    DBG_APP_INFO ("AppSetCfgCbStart\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    devSpeed = pUsbApp->devSpeed;

    /*
     * Based on type of application as well as how data flows,
     * data wire can be used so initialize datawire.
     */
    Cy_DMA_Enable(pUsbApp->pCpuDw0Base);
    Cy_DMA_Enable(pUsbApp->pCpuDw1Base);

    pActiveCfg = Cy_USB_USBD_GetActiveCfgDscr(pUsbdCtxt);
    if (!pActiveCfg) {
        /* Set config should be called when active config value > 0x00. */
        return;
    }
    numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
    if (numOfIntf == 0x00) {
        return;
    }

    for (index = 0x00; index < numOfIntf; index++) {
        /* During Set Config command always altSetting 0 will be active. */
        pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
        if (pIntfDscr == NULL) {
            DBG_APP_INFO ("pIntfDscrNull\r\n");
            return;
        }

        numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
        if (numOfEndp == 0x00) {
            DBG_APP_INFO ("numOfEndp 0\r\n");
            continue;
        }

        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00) {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParams(pAppCtxt, pEndpDscr);
            numOfEndp--;
            if(devSpeed > CY_USBD_USB_DEV_HS) {
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)) + 6);
            } else {
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
            }
        }
    }

    /* In USB 2.0 connections, send a message to the VCOM thread to queue
     * read requests on USB and UART side.
     */
    if (pUsbApp->devSpeed <= CY_USBD_USB_DEV_HS) {
        BaseType_t xHigherPriorityTaskWoken;
        cy_stc_usbd_app_msg_t xMsg;

        xMsg.type = CY_USB_VCOM_DEVICE_MSG_START_DATA_XFER;
        xQueueSendFromISR(pUsbApp->vcomDeviceQueue, &(xMsg), &(xHigherPriorityTaskWoken));
    }

    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
    pUsbApp->devState = CY_USB_DEVICE_STATE_CONFIGURED;

    DBG_APP_INFO("AppSetCfgCbEnd Done\r\n");
    return;
}   /* end of function */


/*
 * Function: Cy_USB_AppBusResetCallback()
 * Description: This Function will be called by USBD when bus detects RESET.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppBusResetCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t i;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("AppBusResetCallback\r\n");

    for (i = 0; i < CY_USB_MAX_ENDP_NUMBER; i++) {
        if (pUsbApp->endpInDma[i].valid) {
            DBG_APP_INFO("HBDMA destroy EP%d-In\r\n", i);
            Cy_HBDma_Channel_Disable(&(pUsbApp->endpInDma[i].hbDmaChannel));
            Cy_HBDma_Channel_Destroy(&(pUsbApp->endpInDma[i].hbDmaChannel));
            pUsbApp->endpInDma[i].valid = false;
        }

        if (pUsbApp->endpOutDma[i].valid) {
            DBG_APP_INFO("HBDMA destroy EP%d-Out\r\n", i);
            Cy_HBDma_Channel_Disable(&(pUsbApp->endpOutDma[i].hbDmaChannel));
            Cy_HBDma_Channel_Destroy(&(pUsbApp->endpOutDma[i].hbDmaChannel));
            pUsbApp->endpOutDma[i].valid = false;
        }
    }

    /*
     * USBD layer takes care of reseting its own data structure as well as
     * takes care of calling CAL reset APIs. Application needs to take care
     * of reseting its own data structure as well as "device function".
     */
    Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase,
                   pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base,
                   pUsbApp->pHbDmaMgrCtxt);
    pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;

    if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
        Cy_USBSS_Cal_LPMDisable(pUsbdCtxt->pSsCalCtxt);
    }

#if (!CY_CPU_CORTEX_M4)
    /*
     * Map NvicMux2 and NvicMux3 to High BandWidth DMA adapter interrupts by default.
     * If we are about to start transfers in HS mode, it will be switched to DW interrupts.
     */
    Cy_SysInt_SetVector(NvicMux2_IRQn, Cy_USB_EgressDma_ISR);
    Cy_SysInt_SetVector(NvicMux3_IRQn, Cy_USB_IngressDma_ISR);
#endif /* (!CY_CPU_CORTEX_M4) */

    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppBusResetDoneCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppBusResetDoneCallback (void *pAppCtxt,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    DBG_APP_INFO("ppBusResetDoneCallback\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->prevDevState = pUsbApp->devState;
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppBusSpeedCallback()
 * Description: This Function will be called by USBD  layer when
 *              speed is identified or speed change is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppBusSpeedCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppSetupCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppSetupCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue,wLength,wIndex;
    bool isReqHandled = false;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    cy_en_usb_endp_dir_t epDir = CY_USB_ENDP_DIR_INVALID;
    BaseType_t xHigherPriorityTaskWoken;
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("SetupCallback\r\n");

    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function. */

    /* Decode the fields from the setup request. */
    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bType    = ((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS);
    bTarget  = (bReqType & CY_USB_CTRL_REQ_RECIPENT_OTHERS);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wValue   = pUsbdCtxt->setupReq.wValue;
    wIndex   = pUsbdCtxt->setupReq.wIndex;
    wLength  = pUsbdCtxt->setupReq.wLength;

    if (wLength > 0x07)
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,CY_USB_ENDP_DIR_IN, TRUE);

    if (bType == CY_USB_CTRL_REQ_STD)
    {
        DBG_APP_INFO("CY_USB_CTRL_REQ_STD\r\n");
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && ((bRequest == CY_USB_SC_SET_FEATURE)
                    || (bRequest == CY_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (pUsbApp->devState == CY_USB_DEVICE_STATE_CONFIGURED)
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            else
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,CY_USB_ENDP_DIR_IN, TRUE);

            isReqHandled = true;
        }

        if ((bRequest == CY_USB_SC_SET_FEATURE) &&
             (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
             (wValue == CY_USB_FEATURE_ENDP_HALT))
        {
            epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) :
                    (CY_USB_ENDP_DIR_OUT));
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                                     epDir, true);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            isReqHandled = true;
        }

        if ((bRequest == CY_USB_SC_CLEAR_FEATURE) &&
             (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
             (wValue == CY_USB_FEATURE_ENDP_HALT))
        {
            epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) :
                    (CY_USB_ENDP_DIR_OUT));
            Cy_USBD_FlushEndp(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL), epDir);
            Cy_USBD_ResetEndp(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL), epDir,false);
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                                     epDir, false);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            isReqHandled = true;
        }
    }

    /* Check for CDC Class Requests */
    if (bType == CY_USB_CTRL_REQ_CLASS)
    {
        DBG_APP_INFO("CY_USB_CTRL_REQ_CLASS\r\n");
        isReqHandled = true;

        /* CDC Specific Requests */
        /* set_line_coding */
        if (bRequest == SET_LINE_CODING)
        {
            xMsg.type    = CY_USB_VCOM_SETLINECODING_CMD;
            xMsg.data[0] = wLength;
            xQueueSendFromISR(pUsbApp->vcomDeviceQueue, &(xMsg), &(xHigherPriorityTaskWoken));
        }
        /* get_line_coding */
        else if (bRequest == GET_LINE_CODING )
        {
            DBG_APP_INFO("GET_LINE_CODING\r\n");

            retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt,(uint8_t *)glVcomCfgData,CY_USB_MIN(wLength,0x07));
            if (retStatus != CY_USBD_STATUS_SUCCESS)
            {
                isReqHandled = false;
            }
        }
        /* SET_CONTROL_LINE_STATE */
        else if (bRequest == SET_CONTROL_LINE_STATE)
        {
            DBG_APP_INFO("SET_CONTROL_LINE_STATE\r\n");
            if (pUsbApp->devState == CY_USB_DEVICE_STATE_CONFIGURED) {
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            } else {
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,CY_USB_ENDP_DIR_IN, TRUE);
            }
        }
        else
        {
            DBG_APP_ERR("Error SetupCallback\r\n");
            isReqHandled = false;
        }
    }

    /* SET_SEL request is supposed to have an OUT data phase of 6 bytes. */
    if ((bRequest == CY_USB_SC_SET_SEL) && (wLength == 6)) {
        /* SET_SEL request is only received in USBSS case and the Cy_USB_USBD_RecvEndp0Data is blocking. */
        retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, (uint8_t *)SetSelDataBuffer, wLength);
        DBG_APP_INFO("SET_SEL: EP0 recv stat = %d, Data=%x:%x\r\n",
                retStatus, SetSelDataBuffer[0], SetSelDataBuffer[1]);
        isReqHandled = true;
    }

    /* If Request is not handled by the callback, Stall the command. */
    if(!isReqHandled) {

        DBG_APP_ERR("SetupRequest is not handled\r\n");
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                      CY_USB_ENDP_DIR_IN, TRUE);
    }

}   /* end of function. */

/*
 * Function: Cy_USB_AppSuspendCallback()
 * Description: This Function will be called by USBD  layer when
 *              Suspend signal/message is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppSuspendCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->prevDevState = pUsbApp->devState;
    pUsbApp->devState = CY_USB_DEVICE_STATE_SUSPEND;
}   /* end of function. */

/*
 * Function: Cy_USB_AppResumeCallback()
 * Description: This Function will be called by USBD  layer when
 *              Resume signal/message is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppResumeCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    cy_en_usb_device_state_t tempState;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    tempState =  pUsbApp->devState;
    pUsbApp->devState = pUsbApp->prevDevState;
    pUsbApp->prevDevState = tempState;
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppSetIntfCallback()
 * Description: This Function will be called by USBD  layer when
 *              set interface is called.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppSetIntfCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppSetIntfCallback\r\n");

    /* We can stall the SET_INTERFACE request as only one alternate
     * setting is supported per interface by this application.
     */
    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, true);
}   /* end of function. */


/*
 * Function: Cy_USB_AppZlpCallback()
 * Description: This Function will be called by USBD layer when
 *              ZLP message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppZlpCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    /* OUT-ZLP is not expected in VCOM use case. Even if we do receive it, we
     * can simply discard it and move on. */
    if (pMsg->type == CY_USB_CAL_MSG_OUT_ZLP) {
        Cy_USBD_ClearZlpSlpIntrEnableMask(pUsbdCtxt, (uint8_t)pMsg->data[0], CY_USB_ENDP_DIR_OUT, true);
    }

    return;
}   /* end of function. */

/*
 * Function: Cy_USB_AppL1SleepCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Sleep message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppL1SleepCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppL1SleepCb\r\n");
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppL1ResumeCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Resume message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppL1ResumeCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppL1ResumeCb\r\n");
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppSlpCallback()
 * Description: This Function will be called by USBD layer when
 *              SLP message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppSlpCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    /* We need to notify the VCOM task in case of an OUT ZLP. */
    if (pMsg->type == CY_USB_CAL_MSG_OUT_SLP) {
        xMsg.type = CY_USB_VCOM_DEVICE_MSG_SLP_OUT;
        xMsg.data[0] = pMsg->data[0];
        xMsg.data[1] = pMsg->data[1];
        status = xQueueSendFromISR(pAppCtxt->vcomDeviceQueue, &(xMsg),
                                   &(xHigherPriorityTaskWoken));
    }

    (void)status;
    return;
}   /* end of function. */

/*
 * Function: Cy_USB_AppClearDmaInterrupt()
 * Description: Function to clear the pending DMA interrupt associated with an
 *              endpoint.
 * Parameters:
 *      pAppCtxt: Pointer to USB application context structure.
 *      endpNumber: Endpoint number
 *      endpDirection: Endpoint direction.
 * Return: void
 */
void Cy_USB_AppClearDmaInterrupt (cy_stc_usb_app_ctxt_t *pAppCtxt,
                                  uint32_t endpNumber,
                                  cy_en_usb_endp_dir_t endpDirection)
{
    if ((pAppCtxt != NULL) && (endpNumber > 0) &&
            (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {
        if (endpDirection == CY_USB_ENDP_DIR_IN) {
            Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpInDma[endpNumber]));
        } else  {
            Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpOutDma[endpNumber]));
        }
    }

    return;
}  /* end of function. */

/*
 * Function: Cy_USB_AppInitDmaIntr()
 * Description: Function to register an ISR for the DMA channel associated
 *              with an endpoint.
 * Parameters:
 *      endpNumber: Endpoint number
 *      endpDirection: Endpoint direction.
 *      userIsr: ISR function pointer. Can be NULL if interrupt is to be
 *               disabled.
 * Return: void
 */
void Cy_USB_AppInitDmaIntr (uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection,
                            cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;

    if ((endpNumber > 0) && (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {
        if (endpDirection == CY_USB_ENDP_DIR_IN) {
            /* DW1 channels 0 onwards are used for IN endpoints. */
#if CY_CPU_CORTEX_M4
            intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
            intrCfg.intrPriority = 5;
#else
            intrCfg.intrSrc = NvicMux2_IRQn;
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
            intrCfg.intrPriority = 1;
#endif /* CY_CPU_CORTEX_M4 */
        } else {
            /* DW0 channels 0 onwards are used for OUT endpoints. */
#if CY_CPU_CORTEX_M4
            intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
            intrCfg.intrPriority = 5;
#else
            intrCfg.intrSrc = NvicMux3_IRQn;
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
            intrCfg.intrPriority = 1;
#endif /* CY_CPU_CORTEX_M4 */
        }

        if (userIsr != NULL)  {
            /* If an ISR is provided, register it and enable the interrupt. */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        } else {
            /* ISR is NULL. Disable the interrupt. */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }

    return;
} /* end of function. */

/*
 * Function: Cy_USB_AppQueueRead()
 * Description: Function to queue read operation on an OUT endpoint.
 * Parameter: pAppCtxt, endpNumber, pBuffer, dataSize
 * return: void
 */
void
Cy_USB_AppQueueRead (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                     uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t *dmaset_p;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
            (pAppCtxt->pCpuDw0Base == NULL) || (pBuffer == NULL))
    {
        DBG_APP_ERR("QueueRead Err0\r\n");
        return;
    }

    /*
     * Verify that the selected endpoint is valid and
     * dataSize is non-zero.
     */
    dmaset_p = &(pAppCtxt->endpOutDma[endpNumber]);
    if ((dmaset_p->valid == 0) || (dataSize == 0))
    {
        DBG_APP_ERR("QueueRead Err1\r\n");
        return;
    }

    if (Cy_USBHS_App_QueueRead(dmaset_p, pBuffer, dataSize))
    {
        /* Set transfer size for the endpoint and clear NAK status to allow data to be received. */
        if (dataSize < dmaset_p->maxPktSize) {
            /* We are trying to read out data that has already been received. Force EP NAK. */
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
        } else {
            /* Set the NAK bit so that the IP can see the bit transition from 1->0 after XFER_CNT is set. */
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
            Cy_USBD_UpdateXferCount(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, dataSize);
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, false);
        }
    }
} /* end of function */


/*
 * Function: Cy_USB_AppQueueWrite()
 * Description: Function to queue write operation on an IN endpoint.
 * Parameter: pAppCtxt, endpNumber, pBuffer, dataSize
 * return: void
 */
void
Cy_USB_AppQueueWrite (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                      uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t *dmaset_p;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
            (pAppCtxt->pCpuDw1Base == NULL) || (pBuffer == NULL)) {
        DBG_APP_ERR("QueueWrite Err0\r\n");
        return;
    }

    /*
     * Verify that the selected endpoint is valid and the dataSize
     * is non-zero.
     */
    dmaset_p = &(pAppCtxt->endpInDma[endpNumber]);
    if ((dmaset_p->valid == 0) || (dataSize == 0)) {
        DBG_APP_ERR("QueueWrite Err1\r\n");
        return;
    }

    Cy_USBHS_App_QueueWrite(dmaset_p, pBuffer, dataSize);
} /* end of function */

/*
 * Function: Cy_USB_AppDisableEndpDma()
 * Description: This function de-inits all active USB DMA channels as part of USB disconnect process.
 * Parameter: cy_stc_usb_app_ctxt_t *
 * return: void
 */
void
Cy_USB_AppDisableEndpDma (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    uint8_t i;

    /* Abort any ongoing transfers from/to the UART FIFOs and then disable the SCB-UART. */
    pAppCtxt->blockUartReads = true;
    Cy_DMA_Channel_Disable(pAppCtxt->pCpuDw0Base, UART_WRITE_DMA_CHANNEL);
    Cy_USB_AppClearUartDmaInterrupt(pAppCtxt, UART_WRITE);
    Cy_DMA_Channel_Disable(pAppCtxt->pCpuDw0Base, UART_READ_DMA_CHANNEL);
    Cy_USB_AppClearUartDmaInterrupt(pAppCtxt, UART_READ);
    Cy_SCB_ClearRxFifo(SCB1);
    Cy_SCB_UART_DeInit(SCB1);

    if (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
        for (i = 1; i < CY_USB_MAX_ENDP_NUMBER; i++) {
            if (pAppCtxt->endpInDma[i].valid) {
                /* DeInit the DMA channel and disconnect the triggers. */
                Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpInDma[i]));
            }

            if (pAppCtxt->endpOutDma[i].valid) {
                /* DeInit the DMA channel and disconnect the triggers. */
                Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpOutDma[i]));
            }
        }
    }

    /* Disable and destroy the High BandWidth DMA channels. */
    if (pAppCtxt->pUsbToUartChn != NULL) {
        Cy_HBDma_Channel_Disable(pAppCtxt->pUsbToUartChn);
        Cy_HBDma_Channel_Destroy(pAppCtxt->pUsbToUartChn);
        pAppCtxt->pUsbToUartChn = NULL;

        MAIN_REG->TR_ASSIST_GR[0].TRA_CTRL[1] = 0x00;
        MAIN_REG->TR_GR[0].TR_CTL[pAppCtxt->vcomOutEpNum] = 0x00;
        Cy_TrigMux_Connect((TRIG_IN_MUX_9_PDMA0_TR_OUT18 & 0xFFFFFF00),
                TRIG_OUT_MUX_9_LVDSUSB32SS_TR_IN1, false, TRIGGER_TYPE_LEVEL);

        /* Flush the EPM in case of USB 3.x connection. */
        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, pAppCtxt->vcomOutEpNum, CY_USB_ENDP_DIR_OUT);
        }
    }

    if (pAppCtxt->pUartToUsbChn != NULL) {
        Cy_HBDma_Channel_Disable(pAppCtxt->pUartToUsbChn);
        Cy_HBDma_Channel_Destroy(pAppCtxt->pUartToUsbChn);
        pAppCtxt->pUartToUsbChn = NULL;

        MAIN_REG->TR_ASSIST_GR[0].TRA_CTRL[0] = 0x00;
        MAIN_REG->TR_GR[0].TR_CTL[0x10 + pAppCtxt->vcomInEpNum] = 0x00;
        Cy_TrigMux_Connect((TRIG_IN_MUX_9_PDMA0_TR_OUT19 & 0xFFFFFF00),
                TRIG_OUT_MUX_9_LVDSUSB32SS_TR_IN0, false, TRIGGER_TYPE_LEVEL);

        /* Flush the EPM in case of USB 3.x connection. */
        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, pAppCtxt->vcomInEpNum, CY_USB_ENDP_DIR_IN);
        }
    }
}

static void
Cy_USB_VcomDeviceSetLineCodingHandler (cy_stc_usb_app_ctxt_t *pAppCtxt, uint16_t wLength)
{
    cy_stc_scb_uart_config_t uartCfg;
    cy_en_usbd_ret_code_t    retStatus = CY_USBD_STATUS_SUCCESS;
    uint16_t loopCnt = 250u;

    DBG_APP_INFO("SET_LINE_CODING\r\n");
    retStatus = Cy_USB_USBD_RecvEndp0Data(pAppCtxt->pUsbdCtxt, ((uint8_t *)glVcomCfgData), wLength);
    if (retStatus != CY_USBD_STATUS_SUCCESS)
    {
        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
    else
    {
        /* Wait until receive DMA transfer has been completed. */
        while ((!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt)) && (loopCnt--)) {
            Cy_SysLib_DelayUs(10);
        }

        if (!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt)) {
            Cy_USB_USBD_RetireRecvEndp0Data(pAppCtxt->pUsbdCtxt);
            Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
            return;
        }

        /* The parameters passed from the host need to be applied on SCB1. */
        memset((uint8_t *)&uartCfg, 0, sizeof (cy_stc_scb_uart_config_t));
        uartCfg.uartMode            = CY_SCB_UART_STANDARD;
        uartCfg.oversample          = 9;
        uartCfg.stopBits            = CY_SCB_UART_STOP_BITS_1;
        uartCfg.parity              = CY_SCB_UART_PARITY_NONE;
        uartCfg.dataWidth           = 8;
        uartCfg.enableMsbFirst      = false;
        uartCfg.breakWidth          = 12;
        uartCfg.rxFifoTriggerLevel  = VCOM_UART_MAX_RD_SIZE;
        uartCfg.txFifoTriggerLevel  = VCOM_UART_MAX_RD_SIZE;
        uartCfg.rxFifoIntEnableMask = 0;

        Cy_SCB_UART_DeInit(SCB1);
        Cy_SCB_UART_Init(SCB1, &uartCfg, pAppCtxt->pUartCtxt);
        Cy_SCB_UART_Enable(SCB1);
        Cy_SCB_ClearTxFifo(SCB1);
        Cy_SCB_ClearRxFifo(SCB1);
        pAppCtxt->blockUartReads = false;

        /* Set the uart configuration */
        memcpy((uint8_t *)&glUartConfig,(uint8_t *)&uartCfg,sizeof (uartCfg));
    }
}

void
Cy_USB_VcomDeviceTaskHandler (void *pTaskParam)
{
    cy_stc_usbd_app_msg_t queueMsg;
    BaseType_t xStatus;
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;
    uint32_t intMask;

    DBG_APP_INFO("VCOMDeviceThreadCreated\r\n");

    /* If VBus is present, enable the USB connection. */
    pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
    if (pAppCtxt->vbusPresent) {
        (void)Cy_USB_SSConnectionEnable(pAppCtxt);
    }

    Cy_USB_AppUartInitsDmaIntr(UART_WRITE, Cy_Uart_WriteDma_ISR);
    Cy_USB_AppUartInitsDmaIntr(UART_READ, Cy_Uart_ReadDma_ISR);
    DBG_APP_INFO("UART DMA interrupts initialized\r\n");

    do {
        /*
         * Wait until some data is received from the queue.
         * Timeout after 100 ms.
         */
        xStatus = xQueueReceive(pAppCtxt->vcomDeviceQueue, &queueMsg, 100);
        if (xStatus != pdPASS) {
            continue;
        }

        switch (queueMsg.type) {
            case CY_USB_VCOM_DEVICE_MSG_START_DATA_XFER:
                /* Note: Message is sent only in case of USBHS. Not checking explicitly for speed here. */

                /* Enable DMA interrupts for the VCOM OUT and IN endpoints. */
                Cy_USB_AppInitDmaIntr(pAppCtxt->vcomOutEpNum,
                                      CY_USB_ENDP_DIR_OUT, Cy_USBHS_OutEp_DW_ISR);
                Cy_USB_AppInitDmaIntr(pAppCtxt->vcomInEpNum,
                                      CY_USB_ENDP_DIR_IN, Cy_USBHS_InEp_DW_ISR);

                /* Queue read on USB OUT endpoint for one MAX_PKT_SIZE worth of data. */
                pAppCtxt->usbOutPktLen = pAppCtxt->endpOutDma[pAppCtxt->vcomOutEpNum].maxPktSize;
                Cy_USB_AppQueueRead(pAppCtxt, pAppCtxt->vcomOutEpNum,
                                    pAppCtxt->pUsbRxBuffer[pAppCtxt->nxtUsbRxBufIdx],
                                    pAppCtxt->usbOutPktLen);

                /* Enable DataWire channel to read VCOM_UART_MAX_RD_SIZE bytes from the UART RX_FIFO. */
                Cy_VcomDev_QueueUartRead(pAppCtxt, VCOM_UART_MAX_RD_SIZE);
                break;

            case CY_USB_VCOM_DEVICE_MSG_READ_COMPLETE:
                /* Note: Message is sent only in case of USBHS. Not checking explicitly for speed here. */

                intMask = Cy_SysLib_EnterCriticalSection();

                /* Increment the number of occupied data buffers and store
                 * the size of the data received.
                 */
                pAppCtxt->usbRxDataLen[pAppCtxt->nxtUsbRxBufIdx] = pAppCtxt->usbOutPktLen;
                pAppCtxt->nxtUsbRxBufIdx++;
                if (pAppCtxt->nxtUsbRxBufIdx >= VCOM_APP_BUFFER_CNT) {
                    pAppCtxt->nxtUsbRxBufIdx = 0;
                }
                pAppCtxt->usbRxBusyBufCount++;

                /* If this is the first occupied buffer, we can queue UART write. */
                if (pAppCtxt->usbRxBusyBufCount == 1) {
                    Cy_SysLib_ExitCriticalSection(intMask);

                    /* Configure the DMA channel to write the received data to the SCB
                     * transmit FIFO. */
                    Cy_USB_AppUARTWrite(pAppCtxt->pUsbRxBuffer[pAppCtxt->uartTxBufIndex],
                            pAppCtxt->usbRxDataLen[pAppCtxt->uartTxBufIndex]);
                } else {
                    Cy_SysLib_ExitCriticalSection(intMask);
                }

                /* If there is space, queue read on USB OUT endpoint for one MAX_PKT_SIZE worth of data. */
                if (pAppCtxt->usbRxBusyBufCount < VCOM_APP_BUFFER_CNT) {
                    pAppCtxt->usbOutPktLen = pAppCtxt->endpOutDma[pAppCtxt->vcomOutEpNum].maxPktSize;
                    Cy_USB_AppQueueRead(pAppCtxt, pAppCtxt->vcomOutEpNum,
                            pAppCtxt->pUsbRxBuffer[pAppCtxt->nxtUsbRxBufIdx],
                            pAppCtxt->usbOutPktLen);
                }
                break;

            case CY_USB_VCOM_DEVICE_MSG_SLP_OUT:
                /* Note: Message is sent only in case of USBHS. Not checking explicitly for speed here. */

                /* Queue fresh read to retrieve the correct amount of data from Ingress EPM. */
                pAppCtxt->usbOutPktLen = queueMsg.data[1];
                Cy_USB_AppQueueRead(pAppCtxt, pAppCtxt->vcomOutEpNum,
                                    pAppCtxt->pUsbRxBuffer[pAppCtxt->nxtUsbRxBufIdx],
                                    pAppCtxt->usbOutPktLen);

                /* Manually assert the trigger to the DMA channel. */
                Cy_TrigMux_SwTrigger(TRIG_IN_MUX_0_USBHSDEV_TR_OUT0 + pAppCtxt->vcomOutEpNum,
                        CY_TRIGGER_TWO_CYCLES);
                break;

            case CY_USB_VCOM_DEVICE_MSG_WRITE_COMPLETE:
                /* Note: Message is sent only in case of USBHS. Not checking explicitly for speed here. */

                /* Data has been consumed on USB side and buffer is free now. */
                if (pAppCtxt->uartRxFreeBufCount == 0) {
                    /* Queue read from the UART RX FIFO into the next DMA buffer. */
                    Cy_VcomDev_QueueUartRead(pAppCtxt, VCOM_UART_MAX_RD_SIZE);
                }
                pAppCtxt->uartRxFreeBufCount++;

                /* If we have at least one occupied buffer, queue the next write operation. */
                if (pAppCtxt->uartRxFreeBufCount < VCOM_APP_BUFFER_CNT) {
                    Cy_USB_AppVcomSendUartData(pAppCtxt,
                                               pAppCtxt->pUartRxBuffer[pAppCtxt->nxtUsbTxBufIdx],
                                               pAppCtxt->uartRxDataLen[pAppCtxt->nxtUsbTxBufIdx]);
                }
                break;

            case CY_USB_UART_MSG_WRITE_COMPLETE:
                DBG_APP_TRACE("MEM_TO_UART_WRITE_COMPLETE\r\n");
                intMask = Cy_SysLib_EnterCriticalSection();

                /* Move to the next TX buffer for the next transfer. */
                pAppCtxt->uartTxBufIndex++;
                if (pAppCtxt->uartTxBufIndex >= VCOM_APP_BUFFER_CNT) {
                    pAppCtxt->uartTxBufIndex = 0;
                }

                /* Decrement the number of used DMA buffers on the USB RX side.
                 * If we still have an used buffer, we need to queue the
                 * corresponding data transfer on the UART interface.
                 */
                pAppCtxt->usbRxBusyBufCount--;
                if (pAppCtxt->usbRxBusyBufCount > 0) {
                    Cy_SysLib_ExitCriticalSection(intMask);

                    if (pAppCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                        uint16_t xferCnt = pAppCtxt->usbRxDataLen[pAppCtxt->uartTxBufIndex];

                        MAIN_REG->TR_ASSIST_GR[0].TRA_CTRL[1] &=
                            (~MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_STREAM_EN_Msk);

                        if (xferCnt < 0x400U) {
                            /* If this is a short packet, configure the count accordingly. */
                            MAIN_REG->TR_ASSIST_GR[0].TRA_SLP_CTRL[1] =
                                MAIN_REG_TR_ASSIST_GR_TRA_SLP_CTRL_TRA_SHORT_CONFIG_Msk |
                                (xferCnt << MAIN_REG_TR_ASSIST_GR_TRA_SLP_CTRL_TRA_BYTE_COUNT_Pos);
                        } else {
                            MAIN_REG->TR_ASSIST_GR[0].TRA_SLP_CTRL[1] = 0;
                        }

                        MAIN_REG->TR_ASSIST_GR[0].TRA_CTRL[1] = pAppCtxt->pUsbToUartChn->nextProdDscr |
                            MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_STREAM_EN_Msk |
                            MAIN_REG_TR_ASSIST_GR_TRA_CTRL_TRA_PKT_MODE_Msk;
                    } else {
                        /* Queue read on USB OUT endpoint if it is not pending. */
                        if (pAppCtxt->usbRxBusyBufCount == (VCOM_APP_BUFFER_CNT - 1)) {
                            pAppCtxt->usbOutPktLen = pAppCtxt->endpOutDma[pAppCtxt->vcomOutEpNum].maxPktSize;
                            Cy_USB_AppQueueRead(pAppCtxt, pAppCtxt->vcomOutEpNum,
                                    pAppCtxt->pUsbRxBuffer[pAppCtxt->nxtUsbRxBufIdx],
                                    pAppCtxt->usbOutPktLen);
                        }
                    }

                    Cy_USB_AppUARTWrite(pAppCtxt->pUsbRxBuffer[pAppCtxt->uartTxBufIndex],
                            pAppCtxt->usbRxDataLen[pAppCtxt->uartTxBufIndex]);
                } else {
                    Cy_SysLib_ExitCriticalSection(intMask);
                }
                break;

            case CY_USB_UART_MSG_READ_COMPLETE:
                DBG_APP_TRACE("UART_MSG_READ_COMPLETE\r\n");
                if (pAppCtxt->blockUartReads) {
                    DBG_APP_INFO("Block transfer on UART read complete\r\n");
                    break;
                }
                xTimerReset(pAppCtxt->vcomTimerHandle, 0);

                /* FW based commit is not required in SS mode as TR_ASSIST takes care of this. */
                if ((pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS)) {
                    /* Send the data that has been received on the USB IN endpoint. */
                    if (cy_timer_loop == 0x01){
                        Cy_USB_AppVcomSendUartData(pAppCtxt,
                                pAppCtxt->pUartRxBuffer[pAppCtxt->uartRxBufIndex],
                                uartRxFifoDataLenCurr);

                        /* Clear the data size tracking variables. */
                        cy_timer_loop = 0x00;
                        uartRxFifoDataLenCurr = 0x00;
                        uartRxFifoDataLenPrev = 0x00;
                    }
                    else{
                        Cy_USB_AppVcomSendUartData(pAppCtxt,
                                pAppCtxt->pUartRxBuffer[pAppCtxt->uartRxBufIndex],
                                VCOM_UART_MAX_RD_SIZE);
                    }
                }

                intMask = Cy_SysLib_EnterCriticalSection();

                /* Move to the next RX buffer for the next read. */
                pAppCtxt->uartRxBufIndex++;
                if (pAppCtxt->uartRxBufIndex >= VCOM_APP_BUFFER_CNT) {
                    pAppCtxt->uartRxBufIndex = 0;
                }

                pAppCtxt->uartRxFreeBufCount--;
                if (pAppCtxt->uartRxFreeBufCount != 0) {
                    Cy_SysLib_ExitCriticalSection(intMask);

                    /* Queue read from the UART RX FIFO into the next DMA buffer. */
                    Cy_VcomDev_QueueUartRead(pAppCtxt, VCOM_UART_MAX_RD_SIZE);
                } else {
                    Cy_SysLib_ExitCriticalSection(intMask);
                }
                break;

            case CY_USB_VCOM_VBUS_CHANGE_INTR:
                /* Start the debounce timer. */
                xTimerStart(pAppCtxt->vbusDebounceTimer, 0);
                break;

            case CY_USB_VCOM_VBUS_CHANGE_DEBOUNCED:
                /* Check whether VBus state has changed. */
                pAppCtxt->vbusPresent =
                    (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);

                if (pAppCtxt->vbusPresent) {
                    if (!pAppCtxt->usbConnected) {
                        DBG_APP_INFO("Enabling USB connection due to VBus detect\r\n");
                        (void)Cy_USB_SSConnectionEnable(pAppCtxt);
                    }
                } else {
                    if (pAppCtxt->usbConnected) {
                        Cy_USB_AppDisableEndpDma(pAppCtxt);
                        DBG_APP_INFO("Disabling USB connection due to VBus removal\r\n");
                        Cy_USB_SSConnectionDisable(pAppCtxt);
                    }
                }
                break;

            case CY_USB_VCOM_SETLINECODING_CMD:
                Cy_USB_VcomDeviceSetLineCodingHandler(pAppCtxt, queueMsg.data[0]);
                break;

            default:
                DBG_APP_INFO("VcomMsgDefault\r\n");
                break;
        }/* end of switch() */

    } while (1);
}

/*[EOF]*/


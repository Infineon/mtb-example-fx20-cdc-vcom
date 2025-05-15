/***************************************************************************//**
* \file main.c
* \version 1.0
*
* Main source file of the FX10/FX20 USB CDC Virtual COM Port application.
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

#include "cy_pdl.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>
#include "cy_usb_common.h"
#include "cy_usbss_cal_drv.h"
#include "cy_usb_vcom_device.h"
#include "cy_usb_usbd.h"
#include "cy_usb_app.h"
#include "cy_debug.h"
#include "cy_usbfs_cdc.h"
#include "cy_usbd_version.h"
#include "cy_hbdma_version.h"
#include "app_version.h"
#include "cybsp.h"

cy_stc_usbss_cal_ctxt_t ssCalCtxt;
cy_stc_usb_cal_ctxt_t   hsCalCtxt;

/* Global variables associated with High BandWidth DMA setup. */
cy_stc_hbdma_context_t HBW_DrvCtxt;             /* High BandWidth DMA driver context. */
cy_stc_hbdma_dscr_list_t HBW_DscrList;          /* High BandWidth DMA descriptor free list. */
cy_stc_hbdma_buf_mgr_t HBW_BufMgr;              /* High BandWidth DMA buffer manager. */
cy_stc_hbdma_mgr_context_t HBW_MgrCtxt;         /* High BandWidth DMA manager context. */

/* CPU DMA register pointers. */
DMAC_Type *pCpuDmacBase;
DW_Type *pCpuDw0Base, *pCpuDw1Base;

cy_stc_usb_usbd_ctxt_t    usbdCtxt;             /* USBD context structure. */
cy_stc_usb_app_ctxt_t     appCtxt;              /* VCOM application context structure. */
cy_stc_scb_uart_context_t uartCtxt;             /* SCB-UART context structure. */

extern void xPortPendSVHandler( void );
extern void xPortSysTickHandler( void );
extern void vPortSVCHandler( void );

#if DEBUG_INFRA_EN
/* Debug log related initilization */
#define LOGBUF_SIZE     (1024u)

/* Set the debug log verbosity. */
#define DEBUG_LEVEL     (3u)

static uint8_t logBuff[LOGBUF_SIZE];
static uint8_t dbgUartRcvBuffer[128];

/* USB-FS port is always used for logging. */
cy_stc_debug_config_t dbgCfg = {
    .pBuffer         = logBuff,
    .traceLvl        = DEBUG_LEVEL,
    .bufSize         = LOGBUF_SIZE,
    .dbgIntfce       = CY_DEBUG_INTFCE_USBFS_CDC,
    .printNow        = true
};

TaskHandle_t printLogTaskHandle;
#endif /* DEBUG_INFRA_EN */

void SysTickIntrWrapper (void)
{
    Cy_USBD_TickIncrement(&usbdCtxt);
    xPortSysTickHandler();
}

void vPortSetupTimerInterrupt( void )
{
    /* Register the exception vectors. */
    Cy_SysInt_SetVector(PendSV_IRQn, xPortPendSVHandler);
    Cy_SysInt_SetVector(SVCall_IRQn, vPortSVCHandler);
    Cy_SysInt_SetVector(SysTick_IRQn, SysTickIntrWrapper);

    /* Start the SysTick timer with a period of 1 ms. */
    Cy_SysTick_SetClockSource(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU);
    Cy_SysTick_SetReload(Cy_SysClk_ClkFastGetFrequency() / 1000U);
    Cy_SysTick_Clear();
    Cy_SysTick_Enable();
}

#if DEBUG_INFRA_EN
void PrintTaskHandler(void *pTaskParam)
{
    while(1)
    {
        /* Print any pending logs to the output console. */
        Cy_Debug_PrintLog();

        /* Put the thread to sleep for 5 ms */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif /* DEBUG_INFRA_EN */

/*****************************************************************************
 * Function Name: PrintVersionInfo
 ******************************************************************************
 * Summary:
 *  Function to print version information to UART console.
 *
 * Parameters:
 *  type: Type of version string.
 *  version: Version number including major, minor, patch and build number.
 *
 * Return:
 *  None
 *****************************************************************************/
void PrintVersionInfo (const char *type, uint32_t version)
{
    char tString[32];
    uint16_t vBuild;
    uint8_t vMajor, vMinor, vPatch;
    uint8_t typeLen = strlen(type);

    vMajor = (version >> 28U);
    vMinor = ((version >> 24U) & 0x0FU);
    vPatch = ((version >> 16U) & 0xFFU);
    vBuild = (uint16_t)(version & 0xFFFFUL);

    memcpy(tString, type, typeLen);
    tString[typeLen++] = '0' + (vMajor / 10);
    tString[typeLen++] = '0' + (vMajor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vMinor / 10);
    tString[typeLen++] = '0' + (vMinor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vPatch / 10);
    tString[typeLen++] = '0' + (vPatch % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vBuild / 1000);
    tString[typeLen++] = '0' + ((vBuild % 1000) / 100);
    tString[typeLen++] = '0' + ((vBuild % 100) / 10);
    tString[typeLen++] = '0' + (vBuild % 10);
    tString[typeLen++] = '\r';
    tString[typeLen++] = '\n';
    tString[typeLen]   = 0;

    DBG_APP_INFO("%s", tString);
}

/*******************************************************************************
 * Function name: Cy_Fx3G2_OnResetInit
 ****************************************************************************//**
 * This function performs initialization that is required to enable scatter
 * loading of data into the High BandWidth RAM during device boot-up. The FX10/FX20
 * device comes up with the High BandWidth RAM disabled and hence any attempt
 * to read/write the RAM will cause the processor to hang. The RAM needs to
 * be enabled with default clock settings to allow scatter loading to work.
 * This function needs to be called from Cy_OnResetUser.
 *******************************************************************************/
void
Cy_Fx3G2_OnResetInit (
        void)
{
    /* Enable clk_hf4 with IMO as input. */
    SRSS->CLK_ROOT_SELECT[4] = SRSS_CLK_ROOT_SELECT_ENABLE_Msk;

    /* Enable LVDS2USB32SS IP and select clk_hf[4] as clock input. */
    MAIN_REG->CTRL = (
            MAIN_REG_CTRL_IP_ENABLED_Msk |
            (1UL << MAIN_REG_CTRL_NUM_FAST_AHB_STALL_CYCLES_Pos) |
            (1UL << MAIN_REG_CTRL_NUM_SLOW_AHB_STALL_CYCLES_Pos) |
            (3UL << MAIN_REG_CTRL_DMA_SRC_SEL_Pos));
}

/*******************************************************************************
 * Function name: Cy_Fx3g2_InitPeripheralClocks
 ****************************************************************************//**
 *
 * Function used to enable clocks to different peripherals on the FX10/FX20 device.
 *
 * \param adcClkEnable
 * Whether to enable clock to the ADC in the USBSS block.
 *
 * \param usbfsClkEnable
 * Whether to enable bus reset detect clock input to the USBFS block.
 *
 *******************************************************************************/
void Cy_Fx3g2_InitPeripheralClocks (
        bool adcClkEnable,
        bool usbfsClkEnable)
{
    if (adcClkEnable) {
        /* Divide PERI clock at 75 MHz by 75 to get 1 MHz clock using 16-bit divider #1. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 1, 74);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 1);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_LVDS2USB32SS_CLOCK_SAR, CY_SYSCLK_DIV_16_BIT, 1);
    }

    if (usbfsClkEnable) {
        /* Divide PERI clock at 75 MHz by 750 to get 100 KHz clock using 16-bit divider #2. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 2, 749);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 2);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_USB_CLOCK_DEV_BRS, CY_SYSCLK_DIV_16_BIT, 2);
    }

    /* Use PERI 8-bit divider #0 to divide 75 MHz HFCLK by 9 to get 8.333 MHz. */
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_8_BIT, 0, 8);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_8_BIT, 0);
    Cy_SysLib_DelayUs(10);
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB1_CLOCK, CY_SYSCLK_DIV_8_BIT, 0);
}

/*****************************************************************************
 * Function Name: VbusDetGpio_ISR
 *****************************************************************************
 * Summary
 *  Interrupt handler for the Vbus detect GPIO transition detection.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void VbusDetGpio_ISR(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    cy_stc_usbd_app_msg_t xMsg;

    /* Send VBus changed message to the task thread. */
    xMsg.type = CY_USB_VCOM_VBUS_CHANGE_INTR;
    xQueueSendFromISR(appCtxt.vcomDeviceQueue, &(xMsg), &(xHigherPriorityTaskWoken));

    /* Remember that VBus change has happened and disable the interrupt. */
    appCtxt.vbusChangeIntr = true;
    Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 0);
}

/*****************************************************************************
 * Function Name: SCB1Init
 *****************************************************************************
 * Summary
 *  Initialize SCB1 as UART.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 ****************************************************************************/
void SCB1Init(void)
{
    cy_stc_gpio_pin_config_t pinCfg;
    cy_stc_scb_uart_config_t uartCfg;
    cy_stc_sysint_t intrCfg;

    /* Configure the clock input to the SCB, USBFS and USB ADC blocks. */
    Cy_Fx3g2_InitPeripheralClocks(true, true);

    memset((void *)&pinCfg, 0, sizeof(pinCfg));

    /* Configure SCB1.UART_RX (P8.0) pin as input. */
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom = P8_0_SCB1_UART_RX;
    Cy_GPIO_Pin_Init(P8_0_PORT, P8_0_PIN, &pinCfg);

    /* Configure the SCB1.UART_TX (P8.1) pin as strong drive output. */
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pinCfg.hsiom = P8_1_SCB1_UART_TX;
    Cy_GPIO_Pin_Init(P8_1_PORT, P8_1_PIN, &pinCfg);

    /* Configure VBus detect GPIO. */
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    pinCfg.intEdge   = CY_GPIO_INTR_BOTH;
    pinCfg.intMask   = 0x01UL;
    Cy_GPIO_Pin_Init(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, &pinCfg);

    /* Register edge detect interrupt for Vbus detect GPIO. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = VBUS_DETECT_GPIO_INTR;
    intrCfg.intrPriority = 7;
#else
    intrCfg.intrSrc = NvicMux4_IRQn;
    intrCfg.cm0pSrc = VBUS_DETECT_GPIO_INTR;
    intrCfg.intrPriority = 3;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, VbusDetGpio_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    memset((void *)&uartCfg, 0, sizeof(uartCfg));
    uartCfg.uartMode = CY_SCB_UART_STANDARD;

    /* Divide 8.33 MHz clock by 9 to get baud rate of ~921600. */
    uartCfg.oversample         = 9;
    uartCfg.dataWidth          = 8;
    uartCfg.enableMsbFirst     = false;
    uartCfg.stopBits           = CY_SCB_UART_STOP_BITS_1;
    uartCfg.parity             = CY_SCB_UART_PARITY_NONE;
    uartCfg.breakWidth         = 12;
    uartCfg.rxFifoTriggerLevel = VCOM_UART_MAX_RD_SIZE;
    uartCfg.txFifoTriggerLevel = VCOM_UART_MAX_RD_SIZE;

    /* Configure connection from SCB1 TX Trigger to DW0 Channel 18 Input Trigger. */
    Cy_TrigMux_Select(TRIG_OUT_1TO1_0_SCB1_TX_TO_PDMA0_TR_IN18, false, TRIGGER_TYPE_LEVEL);

    /* Configure connection from SCB1 RX Trigger to DW0 Channel 19 Input Trigger. */
    Cy_TrigMux_Select(TRIG_OUT_1TO1_0_SCB1_RX_TO_PDMA0_TR_IN19, false, TRIGGER_TYPE_LEVEL);
} /* end of function. */

/*****************************************************************************
 * Function Name: Cy_USB_HS_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB-HS Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_HS_ISR (void)
{
    if (Cy_USBHS_Cal_IntrHandler(&hsCalCtxt))
    {
        portYIELD_FROM_ISR(true);
    }
}

/*****************************************************************************
 * Function Name: Cy_USB_SS_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB-SS Interrupts. Re-invoke the RTOS schedule once ISR
 *  execution is complete.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_SS_ISR (void)
{
    /* Call the USB32DEV interrupt handler. */
    Cy_USBSS_Cal_IntrHandler(&ssCalCtxt);
    portYIELD_FROM_ISR(true);
}

/*****************************************************************************
 * Function Name: Cy_USB_IngressDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USBSS ingress DMA interrupts. Calls the respective driver
 *  function and then invokes the RTOS scheduler to switch to the highest
 *  priority active task.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_IngressDma_ISR (void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_USB_IN);
    portYIELD_FROM_ISR(true);
}

/*****************************************************************************
 * Function Name: Cy_USB_EgressDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USBSS egress DMA interrupts. Calls the respective driver
 *  function and then invokes the RTOS scheduler to switch to the highest
 *  priority active task.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_EgressDma_ISR (void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_USB_EG);
    portYIELD_FROM_ISR(true);
}

/*****************************************************************************
 * Function Name: Cy_Uart_WriteDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for DataWire transfer complete interrupts for transfers to the
 *  UART TX FIFO.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *****************************************************************************/
void Cy_Uart_WriteDma_ISR (void)
{
    Cy_USB_AppClearUartDmaInterrupt(&appCtxt, UART_WRITE);
    Cy_USB_UARTDmaWriteCompletion(&appCtxt);
}  /* end of function. */


/*****************************************************************************
 * Function Name: Cy_Uart_ReadDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for DataWire transfer complete interrupts for transfers which
 *  read from the UART RX FIFO.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *****************************************************************************/
void Cy_Uart_ReadDma_ISR (void)
{
    Cy_USB_AppClearUartDmaInterrupt(&appCtxt, UART_READ);
    Cy_USB_UARTDmaReadCompletion(&appCtxt);
    portYIELD_FROM_ISR(true);
}  /* end of function. */

/*****************************************************************************
 * Function Name: Cy_USBHS_OutEp_DW_ISR
 ******************************************************************************
 * Summary:
 *  Handler for DMA transfer completion on USB-HS OUT endpoint.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *****************************************************************************/
void Cy_USBHS_OutEp_DW_ISR (void)
{
    Cy_USB_AppClearDmaInterrupt(&appCtxt, appCtxt.vcomOutEpNum, CY_USB_ENDP_DIR_OUT);
    Cy_USB_VcomDeviceDmaReadCompletion(&appCtxt);
    portYIELD_FROM_ISR(true);
}  /* end of function. */


/*****************************************************************************
 * Function Name: Cy_USBHS_InEp_DW_ISR
 ******************************************************************************
 * Summary:
 *  Handler for DMA transfer completion on USB-HS IN endpoint.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *****************************************************************************/
void Cy_USBHS_InEp_DW_ISR (void)
{
    Cy_USB_AppClearDmaInterrupt(&appCtxt, appCtxt.vcomInEpNum, CY_USB_ENDP_DIR_IN);
    Cy_USB_VcomDeviceDmaWriteCompletion(&appCtxt);
    portYIELD_FROM_ISR(true);
}  /* end of function. */

/*****************************************************************************
 * Function Name: UsbDevInit
 *****************************************************************************
 * Summary
 *  Initialize the USB IP blocks and register interrupt handlers.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void UsbDevInit (void)
{
    cy_stc_sysint_t intrCfg;

    memset((uint8_t *)&appCtxt, 0, sizeof(appCtxt));
    memset((uint8_t *)&ssCalCtxt, 0, sizeof(ssCalCtxt));
    memset((uint8_t *)&hsCalCtxt, 0, sizeof(hsCalCtxt));
    memset((uint8_t *)&usbdCtxt, 0, sizeof(usbdCtxt));

    /* Register the USBSS ISR and enable the interrupt. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = lvds2usb32ss_usb32_int_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.intrSrc      = NvicMux1_IRQn;
    intrCfg.cm0pSrc      = lvds2usb32ss_usb32_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_USB_SS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = lvds2usb32ss_usb32_wakeup_int_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.intrSrc      = NvicMux1_IRQn;
    intrCfg.cm0pSrc      = lvds2usb32ss_usb32_wakeup_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_USB_SS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

#if CY_CPU_CORTEX_M4
    /* Register the ISR and enable the interrupt. */
    intrCfg.intrSrc      = usbhsdev_interrupt_u2d_active_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.intrSrc      = NvicMux0_IRQn;
    intrCfg.cm0pSrc      = usbhsdev_interrupt_u2d_active_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, Cy_USB_HS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = usbhsdev_interrupt_u2d_dpslp_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.cm0pSrc      = usbhsdev_interrupt_u2d_dpslp_o_IRQn;
    intrCfg.intrSrc      = NvicMux0_IRQn;
    intrCfg.intrPriority = 2;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, Cy_USB_HS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* ISR for the USB Ingress DMA adapter */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 3;
#else
    intrCfg.intrSrc      = NvicMux3_IRQn;
    intrCfg.cm0pSrc      = lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_USB_IngressDma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

/* ISR for the USB Egress DMA adapter */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = lvds2usb32ss_usb32_egrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 3;
#else
    intrCfg.intrSrc      = NvicMux2_IRQn;
    intrCfg.cm0pSrc      = lvds2usb32ss_usb32_egrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_USB_EgressDma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);
}

/*****************************************************************************
 * Function Name: Cy_InitHbDma
 *****************************************************************************
 * Summary
 *  Initialize the High BandWidth DMA adapters and the channel manager.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  true if initialization is successful.
 ****************************************************************************/
bool Cy_InitHbDma (void)
{
    cy_en_hbdma_status_t      drvstat;
    cy_en_hbdma_mgr_status_t  mgrstat;

    /* Initialize the HBW DMA driver layer. */
    /* Leave 0 as value for USB_EGRESS FQ_DEPTH and RQ_CTRL. */
    drvstat = Cy_HBDma_Init(LVDSSS_LVDS, USB32DEV, &HBW_DrvCtxt, 0, 0);
    if (drvstat != CY_HBDMA_SUCCESS)
    {
        return false;
    }

    /* Setup a HBW DMA descriptor list. */
    mgrstat = Cy_HBDma_DscrList_Create(&HBW_DscrList, 256U);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    /* Initialize the DMA buffer manager. We will use 256 KB of space from 0x1C030000 onwards. */
    mgrstat = Cy_HBDma_BufMgr_Create(&HBW_BufMgr, (uint32_t *)0x1C030000UL, 0x40000UL);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    /* Initialize the HBW DMA channel manager. */
    mgrstat = Cy_HBDma_Mgr_Init(&HBW_MgrCtxt, &HBW_DrvCtxt, &HBW_DscrList, &HBW_BufMgr);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    return true;
}

/*****************************************************************************
 * Function Name: Cy_USBSS_DeInit
 *****************************************************************************
 * Summary
 *  PSVP specific code to make sure that USB32 IP and PHY are disabled
 *  before a connection is attempted.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_USBSS_DeInit (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type  *USB32DEV_MAIN = &base->USB32DEV_MAIN;

    /* Disable the clock for USB3.2 function */
    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_CLK_EN_Msk;

    /* Disable PHYSS */
    base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 &=
                    ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk);

    base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.TOP_CTRL_0 &=
        ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk);

    /* Disable the SuperSpeed Device function */
    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_SSDEV_ENABLE_Msk;
}

/*****************************************************************************
 * Function Name: Cy_USB_DisableUsbBlock
 ******************************************************************************
 * Summary:
 *  Function to disable the USB32DEV IP block after terminating current
 *  connection.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_DisableUsbBlock (void)
{
    /* Disable the USB32DEV IP. */
    USB32DEV->USB32DEV_MAIN.CTRL &= ~USB32DEV_MAIN_CTRL_IP_ENABLED_Msk;

    /* Disable HBDMA adapter interrupts and the adapter itself. */
    NVIC_DisableIRQ(lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn);
    NVIC_DisableIRQ(lvds2usb32ss_usb32_egrs_dma_int_o_IRQn);
    Cy_HBDma_DeInit(&HBW_DrvCtxt);

    DBG_APP_TRACE("Disabled HBWSS DMA adapters\r\n");
}

/*****************************************************************************
 * Function Name: Cy_USB_EnableUsbBlock
 ******************************************************************************
 * Summary:
 *  Function to enable the USB32DEV IP block before enabling a new USB
 *  connection.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_EnableUsbBlock (void)
{
    /* Enable the USB DMA adapters and respective interrupts. */
    Cy_HBDma_Init(NULL, USB32DEV, &HBW_DrvCtxt, 0, 0);

    NVIC_EnableIRQ(lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn);
    NVIC_EnableIRQ(lvds2usb32ss_usb32_egrs_dma_int_o_IRQn);

    /* Make sure to enable USB32DEV IP. */
    USB32DEV->USB32DEV_MAIN.CTRL |= USB32DEV_MAIN_CTRL_IP_ENABLED_Msk;
}

/*****************************************************************************
 * Function Name: Cy_USB_SSConnectionEnable
 *****************************************************************************
 * Summary
 *  PSVP specific USB connect function.
 *
 * Parameters:
 *  pAppCtxt: Pointer to VCOM application context structure.
 *
 * Return:
 *  Always returns true.
 ****************************************************************************/
bool Cy_USB_SSConnectionEnable (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    /* Enable USB connection with desired rate set based on make parameter. */
    Cy_USBD_ConnectDevice(pAppCtxt->pUsbdCtxt, USB_CONN_TYPE);
    pAppCtxt->usbConnected = true;

    return true;
}

/*****************************************************************************
 * Function Name: Cy_USB_SSConnectionDisable
 *****************************************************************************
 * Summary
 *  PSVP specific USB disconnect function.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_USB_SSConnectionDisable (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    Cy_USBD_DisconnectDevice(pAppCtxt->pUsbdCtxt);
    Cy_USBSS_DeInit(pAppCtxt->pUsbdCtxt->pSsCalCtxt);
    pAppCtxt->usbConnected = false;
    pAppCtxt->devState     = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
}

/*****************************************************************************
* Function Name: DebugRcvCallback
******************************************************************************
* Summary:
*  Callback function which handles data received through the debug UART.
*
* Parameters:
*  pBuf:        Pointer to read data buffer.
*  size:        Size of data received.
*  pAppCtxt:    Pointer to application context.
*
* Return:
*  void
*****************************************************************************/
void DebugRcvCallback (uint8_t *pBuf, uint16_t size, void *pAppCtxt)
{
    DBG_APP_INFO("Received %d bytes from the debug UART\r\n", size);
    Cy_Debug_QueueDataRead(dbgUartRcvBuffer, 128, DebugRcvCallback, pAppCtxt);
}

/*****************************************************************************
 * Function Name: Cy_Set_BasePri
 *****************************************************************************
 * Summary
 *  This function sets the BASEPRI value for the active Cortex-M CPU core.
 *
 * Parameters:
 *  val: BASEPRI value to be set.
 ****************************************************************************/
void Cy_Set_BasePri(uint32_t val)
{
#if (CY_CPU_CORTEX_M4)
    __set_BASEPRI(val);
#endif /* (CY_CPU_CORTEX_M4) */
}

/*****************************************************************************
* Function Name: main(void)
******************************************************************************
* Summary:
*  Entry to the application.
*
* Parameters:
*  void

* Return:
*  Does not return.
*****************************************************************************/
int main (void)
{
    const char startup_string[] = "***** FX20: CDC VCOM application *****\r\n";

    pCpuDmacBase = ((DMAC_Type *)DMAC_BASE);
    pCpuDw0Base  = ((DW_Type *)DW0_BASE);
    pCpuDw1Base  = ((DW_Type *)DW1_BASE);

    /* Initialize the PDL driver library and set the clock variables. */
    Cy_PDL_Init(&cy_deviceIpBlockCfgFX3G2);
    cybsp_init();

    /* Unlock and then disable the watchdog. */
    Cy_WDT_Unlock();
    Cy_WDT_Disable();

    /* Set BASEPRI value to 0 to ensure all exceptions can run and then enable interrupts. */
    Cy_Set_BasePri(0);
    __enable_irq();

    /* Initialize SCB-1 as UART for the VCOM interface. */
    SCB1Init();

    /* Initialize the USB blocks. */
    UsbDevInit();

#if DEBUG_INFRA_EN
    /* Enable forwarding of CDC OUT data to application callback. */
    CyUsbFsCdc_ControlDataReceive(true);

    /* Log through USBFS CDC port on FX10 DVK. */
    dbgCfg.dbgIntfce = CY_DEBUG_INTFCE_USBFS_CDC;
    Cy_Debug_LogInit(&dbgCfg);

    /* Give some time for USBFS enumeration to complete. */
    Cy_SysLib_Delay(5000U);

    /* Print start-up string to the logging UART. */
    DBG_APP_INFO("%s", startup_string);

    /* Print application, USBD stack and HBDMA version information. */
    PrintVersionInfo("APP_VERSION: ", APP_VERSION_NUM);
    PrintVersionInfo("USBD_VERSION: ", USBD_VERSION_NUM);
    PrintVersionInfo("HBDMA_VERSION: ", HBDMA_VERSION_NUM);

    /* Create task for printing logs and check status. */
    xTaskCreate(PrintTaskHandler, "PrintLogTask", 512, NULL, 5, &printLogTaskHandle);
#endif /* DEBUG_INFRA_EN */

    memset((uint8_t *)&appCtxt, 0, sizeof(appCtxt));
    memset((uint8_t *)&ssCalCtxt, 0, sizeof(ssCalCtxt));
    memset((uint8_t *)&hsCalCtxt, 0, sizeof(hsCalCtxt));
    memset((uint8_t *)&usbdCtxt, 0, sizeof(usbdCtxt));

    /* Store IP base address in CAL context. */
    ssCalCtxt.regBase  = USB32DEV;
    hsCalCtxt.pCalBase = MXS40USBHSDEV_USBHSDEV;
    hsCalCtxt.pPhyBase = MXS40USBHSDEV_USBHSPHY;

    /*
     * Make sure any previous USB connection state is cleared. Give some delay to allow the host to process
     * disconnection.
     */
    Cy_USBSS_DeInit(&ssCalCtxt);
    Cy_SysLib_Delay(500);

    /* Initialize the HbDma IP and DMA Manager */
    Cy_InitHbDma();
    DBG_APP_INFO("Cy_InitHbDma done\r\n");

    /* Initialize the USBD layer */
    Cy_USB_USBD_Init(&appCtxt, &usbdCtxt, pCpuDmacBase, &hsCalCtxt, &ssCalCtxt, &HBW_MgrCtxt);
    DBG_APP_INFO("USBD_Init done\r\n");

    /* Specify that DMA clock should be set to 240 MHz once USB 3.x connection is active. */
    Cy_USBD_SetDmaClkFreq(&usbdCtxt, CY_HBDMA_CLK_240_MHZ);

    /* Register USB descriptors with the stack. */
    Cy_USB_VcomDevice_RegisterDescriptors(&usbdCtxt, CY_USBD_USB_DEV_SS_GEN1);

    /* Initialize the application and create VCOM application task. */
    Cy_USB_AppInit(&appCtxt, &usbdCtxt, pCpuDmacBase, pCpuDw0Base, pCpuDw1Base, &HBW_MgrCtxt);
    appCtxt.pUartCtxt = &uartCtxt;

    /* Queue read operation to get data from the logging CDC interface. */
    Cy_Debug_QueueDataRead(dbgUartRcvBuffer, 128, DebugRcvCallback, &appCtxt);

    /* Invokes scheduler: Not expected to return. */
    vTaskStartScheduler();
    while (1);

    return 0;
}

/*****************************************************************************
 * Function Name: Cy_OnResetUser(void)
 ******************************************************************************
 * Summary:
 *  Init function which is executed before the load regions in RAM are updated.
 *  The High BandWidth subsystem needs to be enable here to allow variables
 *  placed in the High BandWidth SRAM to be updated.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *****************************************************************************/
void Cy_OnResetUser (void)
{
    Cy_Fx3G2_OnResetInit();
}

/* [] END OF FILE */

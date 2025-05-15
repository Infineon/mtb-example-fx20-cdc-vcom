/***************************************************************************//**
* \file cy_usb_descriptors.c
* \version 1.0
*
* This file contains the USB descriptors for the FX10 USB CDC Virtual COM Port
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

#include "cy_pdl.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_usb_vcom_device.h"
#include "cy_usb_app.h"

/* Endpoint definitions for the USB-UART application */
#define BULK_IN_ENDPOINT        (2u)       /* Bulk endpoint used for UART to USB transfers: 2-IN. */
#define BULK_OUT_ENDPOINT       (1u)       /* Bulk endpoint used for USB to UART transfers: 1-OUT. */
#define VCOM_INTR_ENDPOINT      (3u)       /* Interrupt endpoint used for VCOM notifications: 3-IN. */

/* Device Capability Type Codes */
#define CY_WIRELESS_USB_CAPB_TYPE    0x01
#define CY_USB2_EXTN_CAPB_TYPE       0x02
#define CY_SS_USB_CAPB_TYPE          0x03
#define CY_CONTAINER_ID_CAPBD_TYPE   0x04

/* Standard device descriptor for USB 3.1 */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSB30DeviceDscr[] __attribute__ ((aligned (32))) =
{
    0x12,                           /* Descriptor size */
    0x01,                           /* Device descriptor type */
    0x10,0x03,                      /* USB 3.1 */
    0x02,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x09,                           /* Maxpacket size for EP0 */
    0xB4,0x04,                      /* Vendor ID */
    0x11,0x00,                      /* Product ID - Using the Cypress USB-UART PID for driver binding */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Standard device descriptor for USB 2.0 */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSB20DeviceDscr[] __attribute__ ((aligned (32))) =
{
    0x12,                           /* Descriptor size */
    0x01,                           /* Device descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0x02,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0  */
    0xB4,0x04,                      /* Vendor ID */
    0x11,0x00,                      /* Product ID  - Using the Cypress USB-UART PID for driver binding */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Binary device object store descriptor for USB 2.x operation. */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBBOSDscr_HS[] __attribute__ ((aligned (32))) =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* BOS descriptor type */
    0x0C,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x01,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    CY_USB2_EXTN_CAPB_TYPE,         /* USB 2.0 extension capability type */
    0x02,0x00,0x00,0x00            /* Supported device level features: LPM support  */
};

/* Binary device object store descriptor for SuperSpeed and SuperSpeedPlus operation. */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBBOSDscr_SS[] __attribute__ ((aligned (32))) =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* BOS descriptor type */
    0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    CY_USB2_EXTN_CAPB_TYPE,         /* USB 2.0 extension capability type */
    0x02,0x00,0x00,0x00,            /* Supported device level features: LPM support  */

    /* SuperSpeed device capability */
    0x0A,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    CY_SS_USB_CAPB_TYPE,            /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features  */
    0x0E,0x00,                      /* Speeds supported by the device : SS, HS and FS */
    0x03,                           /* Functionality support */
    0x00,                           /* U1 Device Exit latency */
    0x00,0x00                       /* U2 Device Exit latency */
};

/* Binary Object Store (BOS) Descriptor to be used in SuperSpeedPlus connection. */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBBOSDscr_SSP[] __attribute__ ((aligned (32))) =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* BOS descriptor. */
    0x2A,0x00,                      /* Length of this descriptor and all sub-descriptors */
    0x03,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

    /* SuperSpeed device capability */
    0x0A,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x03,                           /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features: Not LTM capable.  */
    0x0E,0x00,                      /* Speeds supported by the device : SS Gen1, HS and FS */
    0x03,                           /* Functionality support */
    0x0A,                           /* U1 Device Exit latency */
    0xFF,0x07,                      /* U2 Device Exit latency */

    /* SuperSpeedPlus USB device capability */
    0x14,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x0A,                           /* SuperSpeedPlus Device capability */
    0x00,                           /* Reserved */
    0x01,0x00,0x00,0x00,            /* SSAC=1, SSIC=0 */
    0x00,0x11,                      /* SSID=0, Min. RX Lane = 1, Min. Tx Lane = 1 */
    0x00,0x00,                      /* Reserved */
    0x30,0x40,0x0A,0x00,            /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Rx), LP=1(SSPlus), LSM=10 */
    0xB0,0x40,0x0A,0x00             /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Tx), LP=1(SSPlus), LSM=10 */
};

/* Standard device qualifier descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBDeviceQualDscr[] __attribute__ ((aligned (32))) =
{
    0x0A,                           /* Descriptor size */
    0x06,                           /* Device qualifier descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0x02,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x08,                           /* Maxpacket size for EP0 */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};

/* Standard super speed configuration descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBSSConfigDscr[] __attribute__ ((aligned (32))) =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
    0x55,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x03,                           /* COnfiguration string index */
    0x80,                           /* Config characteristics - bus powered */
    0x19,                           /* Max power consumption of device (in 8mA unit) : 200mA */

    /* Communication Interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of endpoints */
    0x02,                           /* Interface class : Communication interface */
    0x02,                           /* Interface sub class */
    0x01,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* CDC Class-specific Descriptors */
    /* Header functional Descriptor */
    0x05,                           /* Descriptors length(5) */
    0x24,                           /* Descriptor type : CS_Interface */
    0x00,                           /* DescriptorSubType : Header Functional Descriptor */
    0x10,0x01,                      /* bcdCDC : CDC Release Number */

    /* Abstract Control Management Functional Descriptor */
    0x04,                           /* Descriptors Length (4) */
    0x24,                           /* bDescriptorType: CS_INTERFACE */
    0x02,                           /* bDescriptorSubType: Abstract Control Model Functional Descriptor */
    0x02,                           /* bmCapabilities: Supports the request combination of Set_Line_Coding,
                                       Set_Control_Line_State, Get_Line_Coding and the notification Serial_State */

    /* Union Functional Descriptor */
    0x05,                           /* Descriptors Length (5) */
    0x24,                           /* bDescriptorType: CS_INTERFACE */
    0x06,                           /* bDescriptorSubType: Union Functional Descriptor */
    0x00,                           /* bControlInterface */
    0x01,                           /* bSubordinateInterface0 */

    /* Call Management Functional Descriptor */
    0x05,                           /*  Descriptors Length (5) */
    0x24,                           /*  bDescriptorType: CS_INTERFACE */
    0x01,                           /*  bDescriptorSubType: Call Management Functional Descriptor */
    0x00,                           /*  bmCapabilities: Device sends/receives call management information
                                        only over the Communication Class Interface. */
    0x01,                           /*  Interface Number of Data Class interface */

    /* Endpoint Descriptor(Interrupt) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | VCOM_INTR_ENDPOINT,      /* Endpoint address and description */
    0x03,                           /* Interrupt endpoint type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x01,                           /* Servicing interval for data transfers */

    /* Super speed endpoint companion descriptor for interrupt endpoint */
    0x06,                           /* Descriptor size */
    0x30,                           /* SS endpoint companion descriptor type */
    0x00,                           /* Max no. of packets in a Burst : 1 */
    0x00,                           /* Mult.: Max number of packets : 1 */
    0x40,0x00,                      /* Bytes per interval: 64 */

    /* Data Interface Descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x02,                           /* Number of endpoints */
    0x0A,                           /* Interface class: Data interface */
    0x00,                           /* Interface sub class */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* Endpoint Descriptor(BULK-PRODUCER) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    BULK_OUT_ENDPOINT,              /* Endpoint address and description */
    0x02,                           /* BULK endpoint type */
    0x00,0x04,                      /* Max packet size = 1024 bytes */
    0x00,                           /* Servicing interval for data transfers */

    /* Super speed endpoint companion descriptor for producer ep */
    0x06,                           /* Descriptor size */
    0x30,                           /* SS endpoint companion descriptor type */
    0x00,                           /* Max no. of packets in a burst : 1 */
    0x00,                           /* Mult.: Max number of packets : 1 */
    0x00,0x00,                      /* Bytes per interval : N/A for bulk */

    /* Endpoint Descriptor(BULK- CONSUMER) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | BULK_IN_ENDPOINT,        /* Endpoint address and description */
    0x02,                           /* BULK endpoint type */
    0x00,0x04,                      /* Max packet size = 1024 bytes */
    0x00,                           /* Servicing interval for data transfers */

    /* Super speed endpoint companion descriptor for consumer ep */
    0x06,                           /* Descriptor size */
    0x30,                           /* SS endpoint companion descriptor type */
    0x00,                           /* Max no. of packets in a burst : 1 */
    0x00,                           /* Mult.: Max number of packets : 1 */
    0x00,0x00                       /* Bytes per interval : N/A for bulk */
};

/* Standard high speed configuration descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBHSConfigDscr[] __attribute__ ((aligned (32))) =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
    0x43,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x03,                           /* COnfiguration string index */
    0x80,                           /* Config characteristics - bus powered */
    0x64,                           /* Max power consumption of device (in 2mA unit) : 200mA */

    /* Communication Interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of endpoints */
    0x02,                           /* Interface class : Communication Interface */
    0x02,                           /* Interface sub class */
    0x01,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* CDC Class-specific Descriptors */
    /* Header functional Descriptor */
    0x05,                           /* Descriptors length(5) */
    0x24,                           /* Descriptor type : CS_Interface */
    0x00,                           /* DescriptorSubType : Header Functional Descriptor */
    0x10,0x01,                      /* bcdCDC : CDC Release Number */

    /* Abstract Control Management Functional Descriptor */
    0x04,                           /* Descriptors Length (4) */
    0x24,                           /* bDescriptorType: CS_INTERFACE */
    0x02,                           /* bDescriptorSubType: Abstract Control Model Functional Descriptor */
    0x02,                           /* bmCapabilities: Supports the request combination of Set_Line_Coding,
                                       Set_Control_Line_State, Get_Line_Coding and the notification Serial_State */

    /* Union Functional Descriptor */
    0x05,                           /* Descriptors Length (5) */
    0x24,                           /* bDescriptorType: CS_INTERFACE */
    0x06,                           /* bDescriptorSubType: Union Functional Descriptor */
    0x00,                           /* bControlInterface */
    0x01,                           /* bSubordinateInterface0 */

    /* Call Management Functional Descriptor */
    0x05,                           /*  Descriptors Length (5) */
    0x24,                           /*  bDescriptorType: CS_INTERFACE */
    0x01,                           /*  bDescriptorSubType: Call Management Functional Descriptor */
    0x00,                           /*  bmCapabilities: Device sends/receives call management information
                                        only over the Communication Class Interface. */
    0x01,                           /*  Interface Number of Data Class interface */

    /* Endpoint Descriptor(Interrupt) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | VCOM_INTR_ENDPOINT,      /* Endpoint address and description */
    0x03,                           /* Interrupt endpoint type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x02,                           /* Servicing interval for data transfers */

    /* Data Interface Descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x02,                           /* Number of endpoints */
    0x0A,                           /* Interface class: Data interface */
    0x00,                           /* Interface sub class */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* Endpoint Descriptor(BULK-PRODUCER) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    BULK_OUT_ENDPOINT,              /* Endpoint address and description */
    0x02,                           /* BULK endpoint type */
    0x00,0x02,                      /* Max packet size = 512 bytes */
    0x00,                           /* Servicing interval for data transfers */

    /* Endpoint Descriptor(BULK- CONSUMER) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | BULK_IN_ENDPOINT,        /* Endpoint address and description */
    0x02,                           /* Bulk endpoint type */
    0x00,0x02,                      /* Max packet size = 512 bytes */
    0x00,                           /* Servicing interval for data transfers */
};

/* Standard full speed configuration descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBFSConfigDscr[] __attribute__ ((aligned (32))) =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
    0x43,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x03,                           /* COnfiguration string index */
    0x80,                           /* Config characteristics - bus powered */
    0x64,                           /* Max power consumption of device (in 2mA unit) : 200mA */

    /* Communication Interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of endpoints */
    0x02,                           /* Interface class: Communication interface*/
    0x02,                           /* Interface sub class */
    0x01,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* CDC Class-specific Descriptors */
    /* Header functional Descriptor */
    0x05,                           /* Descriptors length(5) */
    0x24,                           /* Descriptor type : CS_Interface */
    0x00,                           /* DescriptorSubType : Header Functional Descriptor */
    0x10,0x01,                      /* bcdCDC : CDC Release Number */

    /* Abstract Control Management Functional Descriptor */
    0x04,                           /* Descriptors Length (4) */
    0x24,                           /* bDescriptorType: CS_INTERFACE */
    0x02,                           /* bDescriptorSubType: Abstract Control Model Functional Descriptor */
    0x02,                           /* bmCapabilities: Supports the request combination of Set_Line_Coding,
                                       Set_Control_Line_State, Get_Line_Coding and the notification Serial_State */

    /* Union Functional Descriptor */
    0x05,                           /* Descriptors Length (5) */
    0x24,                           /* bDescriptorType: CS_INTERFACE */
    0x06,                           /* bDescriptorSubType: Union Functional Descriptor */
    0x00,                           /* bControlInterface */
    0x01,                           /* bSubordinateInterface0 */

    /* Call Management Functional Descriptor */
    0x05,                           /*  Descriptors Length (5) */
    0x24,                           /*  bDescriptorType: CS_INTERFACE */
    0x01,                           /*  bDescriptorSubType: Call Management Functional Descriptor */
    0x00,                           /*  bmCapabilities: Device sends/receives call management information only over
                                        the Communication Class Interface. */
    0x01,                           /*  Interface Number of Data Class interface */

    /* Endpoint Descriptor(Interrupt) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | VCOM_INTR_ENDPOINT,      /* Endpoint address and description */
    0x03,                           /* Interrupt endpoint type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x02,                           /* Servicing interval for data transfers */

    /* Data Interface Descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x02,                           /* Number of endpoints */
    0x0A,                           /* Interface class: Data interface */
    0x00,                           /* Interface sub class */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* Endpoint Descriptor(BULK-PRODUCER) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    BULK_OUT_ENDPOINT,              /* Endpoint address and description */
    0x02,                           /* BULK endpoint type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x00,                           /* Servicing interval for data transfers */

    /* Endpoint Descriptor(BULK- CONSUMER) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | BULK_IN_ENDPOINT,        /* Endpoint address and description */
    0x02,                           /* Bulk endpoint type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x00                            /* Servicing interval for data transfers */
};

/* Standard language ID string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBStringLangIDDscr[] __attribute__ ((aligned (32))) =
{
    0x04,                           /* Descriptor size */
    0x03,                           /* Device descriptor type */
    0x09,0x04                       /* Language ID supported */
};

/* Standard manufacturer string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBManufactureDscr[32] __attribute__ ((aligned (32))) =
{
    0x08,
    0x03,
    'I',
    0x00,
    'F',
    0x00,
    'X',
    0x00
};

/* Standard Product String desciptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBProductDscr[32] __attribute__ ((aligned (32))) =
{
    0x14,
    0x03,
    'F',
    0x00,
    'X',
    0x00,
    '3',
    0x00,
    'G',
    0x00,
    '2',
    0x00,
    'V',
    0x00,
    'C',
    0x00,
    'O',
    0x00,
    'M',
    0x00
};

void Cy_USB_VcomDevice_RegisterDescriptors (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_en_usb_speed_t usbSpeed)
{
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_SS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    if (usbSpeed == CY_USBD_USB_DEV_SS_GEN2) {
        Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr_SSP);
    } else {
        Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr_SS);
    }
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_HS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr_HS);
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_DEVICE_QUAL_DSCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_SS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_HS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_FS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_STRING_DSCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_STRING_DSCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_STRING_DSCR, 2, (uint8_t *)CyFxUSBProductDscr);
}

/* [ ] */

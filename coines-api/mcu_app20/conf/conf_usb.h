/**
 * \file
 *
 * \brief USB configuration file for CDC application
 *
 * Copyright (c) 2009-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef _CONF_USB_H_
#define _CONF_USB_H_

#include <compiler.h>
#include <usb_protocol_cdc.h>
/**
 * USB Device Configuration
 * @{
 */

//! Device definition
#define  USB_DEVICE_VENDOR_ID             0x108C      // Robert Bosch GmbH USB VID
#define  USB_DEVICE_PRODUCT_ID            0xAB2C      // Temporary.Need an official PID !
#define  USB_DEVICE_MAJOR_VERSION         1
#define  USB_DEVICE_MINOR_VERSION         0
#define  USB_DEVICE_POWER                 100
#define  USB_DEVICE_MANUFACTURE_NAME      "Bosch Sensortec GmbH"
#define  USB_DEVICE_PRODUCT_NAME          "Application Board 2.0"

#define  USB_DEVICE_ATTR                  \
	(USB_CONFIG_ATTR_SELF_POWERED)
// (USB_CONFIG_ATTR_BUS_POWERED)
//	(USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_SELF_POWERED)
//	(USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_BUS_POWERED)


/**
 * Device speeds support
 * Low speed not supported by CDC
 * @{
 */
//! To authorize the High speed
#if (UC3A3||UC3A4)
//#define  USB_DEVICE_HS_SUPPORT
#endif
//@}


/**
 * USB Device Callbacks definitions
 * @{
 */
#define  UDC_VBUS_EVENT(b_vbus_high)
#define  UDC_SOF_EVENT()
#define  UDC_SUSPEND_EVENT()
#define  UDC_RESUME_EVENT()
//@}

//@}


/**
 * USB Interface Configuration
 * @{
 */
/**
 * Configuration of CDC interface
 * @{
 */
//! Interface callback definition
#define  UDI_CDC_ENABLE_EXT(port)          stdio_usb_enable()
#define  UDI_CDC_DISABLE_EXT(port)         stdio_usb_disable()
#define  UDI_CDC_RX_NOTIFY(port)
#define  UDI_CDC_TX_EMPTY_NOTIFY(port)
#define  UDI_CDC_SET_CODING_EXT(port,cfg)  get_connection_parameters(cfg)
#define  UDI_CDC_SET_DTR_EXT(port,set)     check_com_port_connection(set)
#define  UDI_CDC_SET_RTS_EXT(port,set)

//! Default configuration of communication port
#define  UDI_CDC_DEFAULT_RATE             115200
#define  UDI_CDC_DEFAULT_STOPBITS         CDC_STOP_BITS_1
#define  UDI_CDC_DEFAULT_PARITY           CDC_PAR_NONE
#define  UDI_CDC_DEFAULT_DATABITS         8
//@}
//@}

//Callback declarations
/*!
 *
 * @brief       : Communication port connection callback handler
 *
 * @param[in]   : Set
 *
 * @return      : None
 */
void check_com_port_connection(int set);
 /*!
  *
  * @brief       : This API is to get connection parameters
  *
  * @param[in]   : usb_cdc_line_coding_t *cfg
  *
  * @return      : None
  */
void get_connection_parameters(usb_cdc_line_coding_t *cfg);
/**
 * USB Device Driver Configuration
 * @{
 */
//@}

//! The includes of classes and other headers must be done at the end of this file to avoid compile error
#include <udi_cdc_conf.h>
#include <stdio_usb.h>

#endif // _CONF_USB_H_

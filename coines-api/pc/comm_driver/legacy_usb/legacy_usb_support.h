/**
 *
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 * BSD-3-Clause
 * Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    legacy_usb_support.h
 * @date    Dec-10-2018
 * @version 1.1
 * @brief   This file contains legacy support for APP2.0 Board Thesycon USBIO driver
 *
 */

#ifndef LEGACY_USB_SUPPORT_H
#define LEGACY_USB_SUPPORT_H

/* header files */
#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <winioctl.h>
#include <setupapi.h>
#include "coines.h"

/* macro definitions */

/*! Size of the I/O buffer in bytes */
#define BUFFER_SIZE                    (COINES_DATA_BUF_SIZE)

/*! Index to be used for "GetConfigurationDescriptor" */
#define CONFIG_INDEX                   0

/*! Number of interfaces, all possible interfaces must be activated */
#define CONFIG_NB_OF_INTERFACES        1

/*! Settings for usb interface */
#define CONFIG_INTERFACE               0

/*! Settings for usb interface */
#define CONFIG_ALT_SETTING             0

/*! Transmission size for read operation */
#define CONFIG_TRAN_SIZE               4096

/*! Transmission size for write operation */
#define CONFIG_TRAN_SIZE_WRITE         64

#define IOCTL_USBIO_BIND_PIPE          0x80942078
#define IOCTL_USBIO_SET_CONFIGURATION  0x80942024

#define USBIO_IID                      { 0x325ddf96, 0x938c, 0x11d3, { 0x9e, 0x34, 0x00, 0x80, 0xc8, 0x27, 0x27, 0xf4 } \
}

/*
 * @brief   USB interface setting
 */
typedef struct _USBIO_INTERFACE_SETTING
{
    USHORT InterfaceIndex; /*<interfave index */
    USHORT AlternateSettingIndex; /*< alternate setting index */
    ULONG MaximumTransferSize; /*< maximum tranfer size */
} USBIO_INTERFACE_SETTING;

/*
 * @brief To USB configuration setting
 */
typedef struct _USBIO_SET_CONFIGURATION
{
    USHORT ConfigurationIndex; /*< Configuration index */
    USHORT NbOfInterfaces; /*< No. of interfaces */
    USBIO_INTERFACE_SETTING InterfaceList[32]; /*<List of various interfaces */
} USBIO_SET_CONFIGURATION;

/*
 * @ brief USB BIND_PIPE struct
 */
typedef struct _USBIO_BIND_PIPE
{
    UCHAR EndpointAddress; /* including direction bit */
} USBIO_BIND_PIPE;

/*!
 *  @brief This API is used to establish the USB communication
 *
 *  @param[in] void
 *
 *  @return Result of USB connection status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int16_t legacy_open_usb_connection(void);

/*!
 *  @brief This API is used to initialize and configure a pipe for read functionality
 *
 *  @param[in] void
 *
 *  @return Result of configuring USB handles for read operation
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int16_t legacy_configure_usb_read(void);

/*!
 *  @brief This API is used to initialize and configure a pipe for write functionality
 *
 *  @param[in] void
 *
 *  @return Result of configuring USB handles for write operation
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int16_t legacy_configure_usb_write(void);

/*!
 *  @brief This API is used to send commands to the board
 *
 *  @param[in] data  : Variable used to store the data
 *
 *  @param[in] length  : Variable used to store the length of the command
 *
 *  @return Indicates whether the command is sent or not
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int16_t legacy_send_usb_command(uint8_t data[], int32_t length);

/*!
 *  @brief This API is used to read the data from board
 *
 *  @param[in] data  : Variable used to store the data
 *
 *  @return Indicates whether the response is received or not
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int16_t legacy_read_usb_response(uint8_t data[]);

/*!
 *  @brief This API is used to close the USB handles
 *
 *  @return Indicates whether the USB device is closed or not
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
void legacy_close_usb_device(void);

#endif

/** @}*/

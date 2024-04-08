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
 * @file    legacy_usb_support.c
 * @brief   This file contains legacy support for APP2.0 Board Thesycon USBIO driver
 *
 */

/*********************************************************************/
/* header files */
/*********************************************************************/
#include "legacy_usb_support.h"

/* For shorter and faster windows.h */
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN  1
#endif

/*! Interface ID (IID) of the USBIO driver */
GUID g_UsbioID = USBIO_IID;

/*! Variable to hold the enumerated devices */
HDEVINFO device_list;

/*! Variable to hold device information data */
SP_DEVINFO_DATA device_info_data;

/*! Variable to hold the device interface data */
SP_DEVICE_INTERFACE_DATA device_interface_data;

/*! Variable to hold the enumerated device detail */
SP_INTERFACE_DEVICE_DETAIL_DATA* device_detail = NULL;

/*! Variable to hold the request length */
DWORD req_len;

/*! Variable to hold the handle for read operation*/
HANDLE device_handle;

/*! Structure to store the configuration information of each read interface */
USBIO_SET_CONFIGURATION usb_config;

/*! Structure to store the pipe related information for read operation */
USBIO_BIND_PIPE usb_pipe;

/*! Structure to store the information for asynchronous I/O operations */
OVERLAPPED overlapped;

/*! Variable to hold the handle for write operation */
HANDLE device_handle_write;

/*! Structure to store the configuration information of each write interface */
USBIO_SET_CONFIGURATION usb_config_write;

/*! Structure to store the pipe related information for write operation */
USBIO_BIND_PIPE usb_pipe_write;

/*! Array to hold the data received during read operation */
UCHAR Buffer[BUFFER_SIZE];

/*! Holds the result of I/O control request */
DWORD status;

/*! End point address for read operation */
DWORD read_end_point_address;

/*! End point address for write operation */
DWORD write_end_point_address;

/*! Holds the number of bytes transferred */
DWORD bytes_transferred;

/*! Holds the value to indicate whether the output of API execution is
 * success or failure */
BOOL result;

/*!
 * @brief io_ctl_sync routine, executes an I/O control request synchronously
 */
DWORD io_ctl_sync(HANDLE file_handle,
                DWORD ioctl_code,
                const void *in_buffer,
                DWORD in_buffer_size,
                void *out_buffer,
                DWORD out_buffer_size,
                DWORD *bytes_returned)
{
    DWORD status;
    DWORD bytes_ret = 0;

    /* Call the device driver*/
    result = DeviceIoControl(file_handle, /* Driver handle */ ioctl_code, /* IOCTL code */
                             (void*)in_buffer, /* Input buffer */ in_buffer_size, /* Input buffer size */
                             out_buffer, /* Output buffer */ out_buffer_size, /* Output buffer size */
                             &bytes_ret, /* Number of bytes returned */ NULL /* OVERLAPPED structure */);

    if (result)
    {
        /* ioctl completed successfully */
        status = COINES_SUCCESS;
    }
    else
    {
        status = GetLastError();
    }

    if (bytes_returned != NULL)
    {
        *bytes_returned = bytes_ret;
    }

    return status;
}

/*!
 * @brief This API is used to establish the USB communication
 */
int16_t legacy_open_usb_connection(void)
{
    int16_t error_code = COINES_SUCCESS;

    /* Create a list of attached devices */
    device_list = SetupDiGetClassDevs(&g_UsbioID, /* LPGUID ClassGuid,*/ NULL, /* PCTSTR Enumerator*/
                                      NULL, /* HWND hwndParent,*/
                                      DIGCF_DEVICEINTERFACE | DIGCF_PRESENT /* DWORD Flags*/);
    if (device_list == NULL)
    {
        error_code = COINES_E_DEVICE_NOT_FOUND;

        return error_code;
    }

    /* Enumerate device interfaces, for simplicity we use index 0 only */
    ZeroMemory(&device_interface_data, sizeof(device_interface_data));
    device_interface_data.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
    device_info_data.cbSize = sizeof(SP_DEVINFO_DATA);

    for (int index = 0;; index++)
    {
        if (!SetupDiEnumDeviceInfo(device_list, index, &device_info_data))
        {
            return COINES_E_DEVICE_NOT_FOUND; /* no match */
        }

        result = SetupDiEnumDeviceInterfaces(device_list, &device_info_data, &g_UsbioID, 0, &device_interface_data);
        if (!result)
        {
            error_code = COINES_E_DEVICE_NOT_FOUND;
            continue;
        }
        else
        {
            /* Get length of INTERFACE_DEVICE_DETAIL_DATA, allocate buffer */
            SetupDiGetDeviceInterfaceDetail(device_list, &device_interface_data, NULL, 0, &req_len, NULL);
            device_detail = (SP_INTERFACE_DEVICE_DETAIL_DATA*)malloc(req_len);
            if (device_detail == NULL)
            {
                error_code = COINES_E_MEMORY_ALLOCATION;
            }

            /* Get the INTERFACE_DEVICE_DETAIL_DATA struct */
            ZeroMemory(device_detail, req_len);
            device_detail->cbSize = sizeof(SP_INTERFACE_DEVICE_DETAIL_DATA);
            result = SetupDiGetDeviceInterfaceDetail(device_list,
                                                     &device_interface_data,
                                                     device_detail,
                                                     req_len,
                                                     &req_len,
                                                     NULL);
            if (!result)
            {
                error_code = COINES_SUCCESS;
            }

            /* Open the device in overlapped mode */
            device_handle = CreateFile(device_detail->DevicePath, /* Device name*/
                                       GENERIC_READ | GENERIC_WRITE, /* Access mode*/
                                       FILE_SHARE_WRITE | FILE_SHARE_READ, /*Share mode*/
                                       NULL, /*Security descriptor*/ OPEN_EXISTING, /* How to create*/
                                       0, /* File attributes*/ NULL /* Template file*/);

            device_handle_write = CreateFile(device_detail->DevicePath, /* Device name*/
                                             GENERIC_READ | GENERIC_WRITE, /* Access mode*/
                                             FILE_SHARE_WRITE | FILE_SHARE_READ, /*Share mode*/
                                             NULL, /*Security descriptor*/ OPEN_EXISTING, /* How to create*/
                                             0, /* File attributes*/ NULL /* Template file*/);

            if (device_handle == INVALID_HANDLE_VALUE || device_handle_write == INVALID_HANDLE_VALUE)
            {
                error_code = COINES_E_UNABLE_OPEN_DEVICE;
                continue;
            }

            if (legacy_configure_usb_read() != COINES_SUCCESS || legacy_configure_usb_write() != COINES_SUCCESS)
            {
                error_code = COINES_E_UNABLE_OPEN_DEVICE;
                continue;
            }
            else
            {
                error_code = COINES_SUCCESS;
                break;
            }
        }
    }

    /* Free resources */
    free(device_detail);
    device_detail = NULL;
    SetupDiDestroyDeviceInfoList(device_list);
    device_list = NULL;

    return error_code;
}

/*!
 * @brief This API is used to initialize and configure a pipe for read functionality
 */
int16_t legacy_configure_usb_read(void)
{
    int16_t error_code = COINES_SUCCESS;

    read_end_point_address = 0x81;

    /* Configure the USB device*/
    ZeroMemory(&usb_config, sizeof(usb_config));
    usb_config.ConfigurationIndex = CONFIG_INDEX;
    usb_config.NbOfInterfaces = CONFIG_NB_OF_INTERFACES;
    usb_config.InterfaceList[0].InterfaceIndex = CONFIG_INTERFACE;
    usb_config.InterfaceList[0].AlternateSettingIndex = CONFIG_ALT_SETTING;
    usb_config.InterfaceList[0].MaximumTransferSize = CONFIG_TRAN_SIZE;

    /* Send the configuration request*/
    status = io_ctl_sync(device_handle,
                       IOCTL_USBIO_SET_CONFIGURATION,
                       &usb_config,
                       sizeof(USBIO_SET_CONFIGURATION),
                       NULL,
                       0,
                       NULL);
    if (status != COINES_SUCCESS)
    {
        error_code = COINES_E_COMM_IO_ERROR;
    }

    /* Bind the handle to a pipe*/
    ZeroMemory(&usb_pipe, sizeof(usb_pipe));
    usb_pipe.EndpointAddress = (UCHAR)read_end_point_address;

    /* Send bind request to USBIO */
    status = io_ctl_sync(device_handle, IOCTL_USBIO_BIND_PIPE, &usb_pipe, sizeof(USBIO_BIND_PIPE), NULL, 0, NULL);
    if (status != COINES_SUCCESS)
    {
        error_code = COINES_E_COMM_IO_ERROR;
    }

    return error_code;
}

/*!
 * @brief This API is used to initialize and configure a pipe for write functionality
 */
int16_t legacy_configure_usb_write(void)
{
    int16_t error_code = COINES_SUCCESS;

    write_end_point_address = 0x02;

    /* Configure the USB device*/
    ZeroMemory(&usb_config_write, sizeof(usb_config_write));
    usb_config_write.ConfigurationIndex = CONFIG_INDEX;
    usb_config_write.NbOfInterfaces = CONFIG_NB_OF_INTERFACES;
    usb_config_write.InterfaceList[0].InterfaceIndex = CONFIG_INTERFACE;
    usb_config_write.InterfaceList[0].AlternateSettingIndex = CONFIG_ALT_SETTING;
    usb_config_write.InterfaceList[0].MaximumTransferSize = CONFIG_TRAN_SIZE_WRITE;

    /* Send the configuration request */
    io_ctl_sync(device_handle_write,
              IOCTL_USBIO_SET_CONFIGURATION,
              &usb_config_write,
              sizeof(USBIO_SET_CONFIGURATION),
              NULL,
              0,
              NULL);

    /* Bind the handle to a pipe */
    ZeroMemory(&usb_pipe_write, sizeof(usb_pipe_write));
    usb_pipe_write.EndpointAddress = (UCHAR)write_end_point_address;

    /* Send bind request to USBIO */
    status = io_ctl_sync(device_handle_write,
                       IOCTL_USBIO_BIND_PIPE,
                       &usb_pipe_write,
                       sizeof(USBIO_BIND_PIPE),
                       NULL,
                       0,
                       NULL);
    if (status != COINES_SUCCESS)
    {
        error_code = COINES_E_COMM_IO_ERROR;
    }

    return error_code;
}

/*!
 * @brief This API is used to send commands to the board
 */
int16_t legacy_send_usb_command(uint8_t data[], int32_t length)
{
    return WriteFile(device_handle_write, data, length, &bytes_transferred, NULL);
}

/*!
 * @brief This API is used to read the data from board
 */
int16_t legacy_read_usb_response(uint8_t data[])
{
    int16_t error_code = COINES_SUCCESS;

    /* Read data into the buffer */
    result = ReadFile(device_handle, /* Handle of file to read */ data, /* Pointer to buffer that receives data */
                      BUFFER_SIZE, /* Number of bytes to read */ &bytes_transferred, /* Number of bytes read */
                      NULL /* Pointer to OVERLAPPED struct */);
    if (result && (data[0] != 0))
    {
        error_code = COINES_SUCCESS;
    }
    else
    {
        error_code = COINES_E_COMM_IO_ERROR;
    }

    return error_code;
}

/*!
 * @brief This API is used to close the USB handles
 */
void legacy_close_usb_device(void)
{
    CloseHandle(device_handle);
    CloseHandle(device_handle_write);
}

/**
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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
 *  @file   sync_comm_windows.c
 *  @brief  This module defines Windows-specific serial communication APIs to be used by the interface layer.
 *          It includes functions for configuring and managing the serial port on a Windows system.
 *          This file is part of the platform-specific implementation for the sync_comm module.
 */

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <windows.h>
#include <setupapi.h>
#include <stdio.h>
#include "platform.h"
#include "sync_comm_windows.h"
#include "error_handling.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*********************************************************************/
/* static variables */
/*********************************************************************/
static HANDLE serial_handle = NULL;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static int configure_serial_port(const char *COM_PortName, uint32_t baud_rate);
static int8_t find_port_in_registry_hardware_devicemap(char *com_port_name, uint32_t baud_rate);
/*********************************************************************/
/* static functions */
/*********************************************************************/
/**
 * @brief Finds the COM port in the Windows registry.
 *
 * @param com_port_name The name of the COM port to search for.
 * @param baud_rate The baud rate for the serial communication.
 *
 * @return A platform_serial_error_code_t indicating the result of the operation.
 */
static int8_t find_port_in_registry_hardware_devicemap(char *com_port_name, uint32_t baud_rate)
{
    HKEY hKey;
    char found_com_port[256] = {0};
    DWORD dwSize = sizeof(found_com_port);
    DWORD dwType;
    int8_t ret_code = PLATFORM_SERIAL_DEV_NOT_FOUND;

    //lint -e{620}
    if (RegOpenKeyEx(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &hKey) == ERROR_SUCCESS)
    {
        DWORD index_t = 0;
        char value_name[256];
        DWORD value_size = sizeof(value_name);

        while (RegEnumValue(hKey, index_t, value_name, &value_size, NULL, &dwType, (LPBYTE)found_com_port, &dwSize) == ERROR_SUCCESS)
        {
            index_t++;
            value_size = sizeof(value_name);
            dwSize = sizeof(found_com_port);

            if (com_port_name == NULL || strcmp(com_port_name, found_com_port) == 0)
            {
                ret_code = (int8_t)configure_serial_port(found_com_port, baud_rate);
                if (ret_code == COINES_SUCCESS)
                {
                    break;
                }
            }
        }
        (void)RegCloseKey(hKey);
    }

    return ret_code;
}

/*********************************************************************/
/* functions */
/*********************************************************************/

/**
 * @brief Configures the serial port with the given parameters.
 *
 * @param COM_PortName The name of the COM port to configure.
 * @param baud_rate The baud rate for the serial communication.
 * @param serial_handle A pointer to the HANDLE for the serial port.
 * @return int Status code indicating success or failure.
 */
static int configure_serial_port(const char *COM_PortName, uint32_t baud_rate)
{
    char str[20];
    BOOL success;

    /*lint -e(534) Ignoring return value */
    snprintf(str, sizeof(str) - 1, "\\\\.\\%s", COM_PortName);
    serial_handle = CreateFile(str,

                               /*lint -e(620) Suspicious constant */ GENERIC_READ | GENERIC_WRITE,
                               0,
                               NULL,
                               OPEN_EXISTING,
                               0,
                               NULL);

    if (serial_handle != INVALID_HANDLE_VALUE)
    {
        /* Reference for the code below https://www.pololu.com/docs/0J73/15.6 */

        /* Flush away any bytes previously read or written. */
        success = FlushFileBuffers(serial_handle);
        if (success)
        {
            /* Set the baud rate and other options. */
            DCB state = { 0 };
            state.DCBlength = sizeof(DCB);
            state.BaudRate = baud_rate;
            state.ByteSize = 8;
            state.Parity = NOPARITY;
            state.StopBits = ONESTOPBIT;
            state.fDtrControl = DTR_CONTROL_ENABLE;

            success = SetCommState(serial_handle, &state);
            if (success)
            {
                /* Configure read and write operations to time out after 100 ms. */
                COMMTIMEOUTS timeouts = { 0 };

                /*
                 * success = GetCommTimeouts(serial_handle, &timeouts);
                 * printf("TO: %x %x %x %x %x\r\n", timeouts.ReadIntervalTimeout, timeouts.ReadTotalTimeoutConstant,
                 * timeouts.ReadTotalTimeoutMultiplier, timeouts.WriteTotalTimeoutConstant,
                 * timeouts.WriteTotalTimeoutMultiplier);
                 */

                timeouts.ReadIntervalTimeout = 0xFFFFFFFF;
                timeouts.ReadTotalTimeoutConstant = 0;
                timeouts.ReadTotalTimeoutMultiplier = 0;
                timeouts.WriteTotalTimeoutConstant = 100;
                timeouts.WriteTotalTimeoutMultiplier = 0;

                success = SetCommTimeouts(serial_handle, &timeouts);
                if (!success)
                {
                    CloseHandle(serial_handle);

                    return PLATFORM_SERIAL_CONFIG_FAILED;
                }

                return COINES_SUCCESS;
            }
        }

        return PLATFORM_SERIAL_CONFIG_FAILED;
    }

    return PLATFORM_SERIAL_PORT_IN_USE;
}

/**
 * @brief Opens a USB CDC ACM device.
 *
 * @param baud_rate The baud rate for the serial communication.
 * @param vid The vendor ID of the USB device.
 * @param pid The product ID of the USB device.
 * @param com_port_name The name of the COM port. If this is NULL, the function will open the first device that matches the VID and PID.
 *
 * @return A platform_serial_error_code_t indicating the result of the operation.
 */
int8_t serial_open(uint32_t baud_rate, uint16_t vid, uint16_t pid, char *com_port_name)
{
    unsigned index_t;
    HDEVINFO hd_dev_info;
    SP_DEVINFO_DATA device_info_data;
    char hardware_id[1024] = { 0 };
    char usb_id[20] = { 0 };
    BOOL found_device = FALSE;
    int8_t ret_code = PLATFORM_SERIAL_DEV_NOT_FOUND;

    sprintf(usb_id, "VID_%04X&PID_%04X", vid, pid);

    hd_dev_info = SetupDiGetClassDevs(NULL, TEXT("USB"), NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES);
    for (index_t = 0; index_t < 255 && !found_device; index_t++)
    {
        device_info_data.cbSize = sizeof(device_info_data);
        if (!SetupDiEnumDeviceInfo(hd_dev_info, index_t, &device_info_data))
        {
            break;
        }

        if (SetupDiGetDeviceRegistryProperty(hd_dev_info, &device_info_data, SPDRP_HARDWAREID, NULL,
                                             (BYTE *)hardware_id, sizeof(hardware_id), NULL))
        {
            /*lint -e(534) Ignoring return value */ fflush(stdout);
            if (strstr(hardware_id, usb_id))
            {
                char COM_PortName[20] = { 0 };
                DWORD dwSize = sizeof(COM_PortName);
                DWORD dwType = 0;
                HKEY hDeviceRegistryKey = SetupDiOpenDevRegKey(hd_dev_info,
                                                               &device_info_data,
                                                               DICS_FLAG_GLOBAL,
                                                               0,
                                                               DIREG_DEV,

                                                               /*lint -e(620) Suspicious constant */ KEY_READ);

                if (RegQueryValueEx(hDeviceRegistryKey, "PortName", NULL, &dwType, (LPBYTE)COM_PortName,
                                    &dwSize) == /*lint -e(620) Suspicious constant */ ERROR_SUCCESS && dwType == REG_SZ)
                {
                    /* Check if the PortName matches the requested COM port name */
                    if (com_port_name == NULL || strcmp(com_port_name, COM_PortName) == 0)
                    {
                        ret_code = (int8_t)configure_serial_port(COM_PortName, baud_rate);
                        if (ret_code == COINES_SUCCESS)
                        {
                            found_device = TRUE;
                        }
                    }
                    
                }
                /* If RegQueryValueEx fails, try searching in "HARDWARE\\DEVICEMAP\\SERIALCOMM" */
                else if(find_port_in_registry_hardware_devicemap(com_port_name, baud_rate) == COINES_SUCCESS)
                {
                    found_device = TRUE;
                    ret_code = COINES_SUCCESS;
                }
                else
                {
                    ret_code = PLATFORM_SERIAL_DEVICE_NOT_SERIAL;
                }

                (void)RegCloseKey(hDeviceRegistryKey);
                
            }
        }
    }

    (void)SetupDiDestroyDeviceInfoList(hd_dev_info);

    return ret_code;
}

/**
 * @brief Closes the USB CDC ACM device.
 *
 * @return A platform_serial_error_code_t indicating the result of the operation.
 */
int8_t serial_close(void)
{
    if (serial_handle)
    {
        CloseHandle(serial_handle);
        serial_handle = NULL;

        return COINES_SUCCESS;
    }
    else
    {
        return PLATFORM_SERIAL_PORT_NOT_OPEN;
    }
}

/**
 * @brief Clears the transmit and receive buffer.
 *
 * @return A platform_serial_error_code_t indicating the result of the operation.
 */
int8_t serial_clear_buffer(void)
{
    if (PurgeComm(serial_handle, PURGE_RXCLEAR | PURGE_TXCLEAR))
    {
        return COINES_SUCCESS;
    }

    return PLATFORM_SERIAL_FLUSH_FAILED;
}

/**
 * @brief Reads data from the serial port.
 *
 * @param buffer The buffer to store the read data.
 * @param n_bytes The number of bytes to read.
 * @param n_bytes_read The number of bytes actually read.
 *
 * @return A platform_serial_error_code_t indicating the result of the operation.
 */
int8_t serial_read(void *buffer, uint32_t n_bytes, uint32_t *n_bytes_read)
{
    if (serial_handle)
    {
        if (ReadFile(serial_handle, buffer, n_bytes, (PDWORD)n_bytes_read, NULL))
        {
            return COINES_SUCCESS;
        }
        else
        {
            return PLATFORM_SERIAL_READ_FAILED;
        }
    }
    else
    {
        return PLATFORM_SERIAL_PORT_NOT_OPEN;
    }
}

/**
 * @brief Writes data to the serial port.
 *
 * @param buffer The buffer containing the data to write.
 * @param n_bytes The number of bytes to write.
 *
 * @return A platform_serial_error_code_t indicating the result of the operation.
 */
int8_t serial_write(void *buffer, uint32_t n_bytes)
{
    DWORD bytes_written = 0;

    if (serial_handle)
    {
        if (WriteFile(serial_handle, buffer, n_bytes, &bytes_written, NULL))
        {
            if (bytes_written == n_bytes)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return PLATFORM_SERIAL_WRITE_FAILED;
            }
        }
        else
        {
            return PLATFORM_SERIAL_WRITE_FAILED;
        }
    }
    else
    {
        return PLATFORM_SERIAL_PORT_NOT_OPEN;
    }
}

/**
 * @brief Checks if the serial port is closed.
 *
 */
int8_t serial_is_port_closed(void)
{
    DWORD modemStatus;
    if (serial_handle == NULL)
    {
        return PLATFORM_SERIAL_DEV_NOT_FOUND; 
    }

    if (GetCommModemStatus(serial_handle, &modemStatus)) 
    {
        return COINES_SUCCESS;
    }
    else 
    {
        (void)serial_close();
        return PLATFORM_SERIAL_DEV_NOT_FOUND;
    }   
}


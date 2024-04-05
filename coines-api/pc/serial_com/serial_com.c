/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file   serial_com.c
 * @brief  This module provides communication interface layer between host and application board
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <libgen.h>

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#include <setupapi.h>
#endif

#ifdef PLATFORM_LINUX
#include <libusbp-1.0/libusbp.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

/*********************************************************************/
/* own header files */
/**********************************************************************/
#include "serial_com.h"

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/
#define ROBERT_BOSCH_USB_VID      (0x108C)
#define ARDUINO_USB_VID           (0x2341)

#define BST_APP31_CDC_USB_PID     (0xAB38)
#define BST_APP30_CDC_USB_PID     (0xAB3C)
#define BST_APP20_CDC_USB_PID     (0xAB2C)
#define ARDUINO_NICLA_USB_PID     (0x0060)

/*! USB COINES_SDK Development desktop device vendor ID*/
#define COINES_DEVICE_DD_VENDOR   (0x152A)

/*! USB COINES_SDK Development desktop device product ID*/
#define COINES_DEVICE_DD_PRODUCT  (0x80C0)

#define DEFAULT_BAUD_RATE         115200

/*********************************************************************/
/* static variables */
/*********************************************************************/

#ifdef PLATFORM_WINDOWS
static HANDLE serial_handle = NULL;
#else
static int serial_handle;
#endif

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static int8_t usb_cdc_acm_open(uint32_t baud_rate, uint16_t vid, uint16_t pid, char *com_port_name);
static int8_t usb_cdc_acm_close(void);

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 * @brief This API is used to open communication interface
 *
 */
int8_t scom_open_id(uint32_t baud_rate, uint16_t vendor_id, uint16_t product_id, char *com_port_name)
{
    return usb_cdc_acm_open(baud_rate, vendor_id, product_id, com_port_name);
}

/*!
 * @brief This API is used to open communication interface with default configuration
 *
 */
int8_t scom_open(void)
{
    int8_t com_rslt;

    com_rslt = scom_open_id(DEFAULT_BAUD_RATE, ROBERT_BOSCH_USB_VID, BST_APP30_CDC_USB_PID, NULL);

    if (com_rslt != SCOM_OK)
    {
        com_rslt = scom_open_id(DEFAULT_BAUD_RATE, ROBERT_BOSCH_USB_VID, BST_APP20_CDC_USB_PID, NULL);
    }

    if (com_rslt != SCOM_OK)
    {
        com_rslt = scom_open_id(DEFAULT_BAUD_RATE, ARDUINO_USB_VID, ARDUINO_NICLA_USB_PID, NULL);
    }

    if (com_rslt != SCOM_OK)
    {
        com_rslt = scom_open_id(DEFAULT_BAUD_RATE, ROBERT_BOSCH_USB_VID, BST_APP31_CDC_USB_PID, NULL);
    }

    if (com_rslt != SCOM_OK)
    {
        com_rslt = scom_open_id(DEFAULT_BAUD_RATE, COINES_DEVICE_DD_VENDOR, COINES_DEVICE_DD_PRODUCT, NULL);
    }

    return com_rslt;
}

/*!
 * @brief This API is used to close the serial communication.
 *
 */
int8_t scom_close(void)
{
    return usb_cdc_acm_close();
}

#ifdef PLATFORM_WINDOWS

/*!
 *  @brief This API is used to read the data.
 *
 */
int8_t scom_read(void *buffer, uint32_t n_bytes, uint32_t *n_bytes_read)
{
    if (serial_handle)
    {
        if (ReadFile(serial_handle, buffer, n_bytes, (PDWORD)n_bytes_read, NULL))
        {
            return SCOM_OK;
        }
        else
        {
            return SCOM_E_READ_FAILED;
        }
    }
    else
    {
        return SCOM_E_PORT_NOT_OPEN;
    }
}

/*!
 *  @brief This API is used to send the data.
 *
 */
int8_t scom_write(void *buffer, uint32_t n_bytes)
{
    DWORD bytes_written = 0;

    if (serial_handle)
    {
        if (WriteFile(serial_handle, buffer, n_bytes, &bytes_written, NULL))
        {
            if (bytes_written == n_bytes)
            {
                return SCOM_OK;
            }
            else
            {
                return SCOM_E_WRITE_FAILED;
            }
        }
        else
        {
            return SCOM_E_WRITE_FAILED;
        }
    }
    else
    {
        return SCOM_E_PORT_NOT_OPEN;
    }
}

/*!
 * @brief This API is used to initialize the serial communication.
 *
 */
static int8_t usb_cdc_acm_open(uint32_t baud_rate, uint16_t vid, uint16_t pid, char *com_port_name)
{
    unsigned index_t;
    HDEVINFO hd_dev_info;
    SP_DEVINFO_DATA device_info_data;
    char hardware_id[1024] = { 0 };
    char usb_id[20] = { 0 };
    BOOL found_device = FALSE;
    int8_t ret_code = SCOM_E_DEV_NOT_FOUND;

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
                        char str[20];

                        /*lint -e(534) Ignoring return value */ snprintf(str, sizeof(str) - 1, "\\\\.\\%s",
                                                                         COM_PortName);
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
                            BOOL success;
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
                                    timeouts.WriteTotalTimeoutConstant = 0;
                                    timeouts.WriteTotalTimeoutMultiplier = 0;

                                    success = SetCommTimeouts(serial_handle, &timeouts);
                                    if (!success)
                                    {
                                        CloseHandle(serial_handle);

                                        return SCOM_E_CONFIG_FAILED;
                                    }

                                    ret_code = SCOM_OK;
                                    found_device = TRUE;
                                }
                            }
                            else
                            {
                                ret_code = SCOM_E_CONFIG_FAILED;
                            }
                        }
                        else
                        {
                            ret_code = SCOM_E_PORT_IN_USE;
                        }
                    }
                }
                else
                {
                    ret_code = SCOM_E_DEV_NOT_SERIAL_COM;
                }

                (void)RegCloseKey(hDeviceRegistryKey);
            }
        }
    }

    (void)SetupDiDestroyDeviceInfoList(hd_dev_info);

    return ret_code;
}

/*!
 * @brief This API is used to close the serial communication.
 *
 */
static int8_t usb_cdc_acm_close(void)
{
    if (serial_handle)
    {
        CloseHandle(serial_handle);
        serial_handle = NULL;

        return SCOM_OK;
    }
    else
    {
        return SCOM_E_PORT_NOT_OPEN;
    }
}

/*!
 * @brief This API is used to clear transmit and receive buffer.
 *
 */
int8_t scom_clear_buffer(void)
{
    if (PurgeComm(serial_handle, PURGE_RXCLEAR | PURGE_TXCLEAR))
    {
        return SCOM_OK;
    }

    return SCOM_E_FLUSH_FAILED;
}
#endif /* PLATFORM_WINDOWS */

#ifdef PLATFORM_LINUX

static int8_t configure_port(uint32_t baudrate, char *port_name)
{
    int file_handle;

    file_handle = open(port_name, O_RDWR | O_NOCTTY);
    if (file_handle < 0)
    {
        return SCOM_E_PORT_IN_USE;
    }

    /* Flush away any bytes previously read or written. */
    tcflush(file_handle, TCIOFLUSH);

    /* Get the current configuration of the serial port. */
    struct termios options;
    if (tcgetattr(file_handle, &options) < 0)
    {
        close(file_handle);

        return SCOM_E_CONFIG_FAILED;
    }

    /*
     * Turn off any options that might interfere with our ability to send and
     * receive raw binary bytes.
     */
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    /* Configure timeout */
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;

    /* Configure baud rate */
    switch (baudrate)
    {
        case 4800:
            cfsetospeed(&options, B4800);
            break;
        case 9600:
            cfsetospeed(&options, B9600);
            break;
        case 19200:
            cfsetospeed(&options, B19200);
            break;
        case 38400:
            cfsetospeed(&options, B38400);
            break;
        case 115200:
            cfsetospeed(&options, B115200);
            break;
        default:
            cfsetospeed(&options, B115200);
            break;
    }
    cfsetispeed(&options, cfgetospeed(&options));

    if (tcsetattr(file_handle, TCSANOW, &options) < 0)
    {
        close(file_handle);

        return SCOM_E_CONFIG_FAILED;
    }

    serial_handle = file_handle;

    return SCOM_OK;
}

/*!
 * @brief This API is used to initialize the serial communication.
 *
 */
static int8_t usb_cdc_acm_open(uint32_t baud_rate, uint16_t vid, uint16_t pid, char *port_name)
{
    int8_t ret = SCOM_OK;
    libusbp_device **dev_list = NULL;
    libusbp_serial_port *port = NULL;
    size_t dev_count = 0;
    uint8_t index;
    libusbp_error *error;
    char *serial_port;
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;

    error = libusbp_list_connected_devices(&dev_list, &dev_count);
    (void)error;
    for (index = 0; index < dev_count; index++)
    {
        error = libusbp_device_get_vendor_id(dev_list[index], &vendor_id);
        (void)error;
        error = libusbp_device_get_product_id(dev_list[index], &product_id);
        (void)error;
        if ((vid == vendor_id) && (product_id == pid))
        {
            error = libusbp_serial_port_create(dev_list[index], 0, true, &port);
            if (error == NULL)
            {
                error = libusbp_serial_port_get_name(port, &serial_port);
                if ((port_name == NULL) || (strcmp(port_name, serial_port) == 0))
                {
                    ret = configure_port(baud_rate, serial_port);
                    libusbp_serial_port_free(port);
                    libusbp_string_free(serial_port);
                    if ((ret == SCOM_OK) || (port_name != NULL && ret == SCOM_E_PORT_IN_USE))
                    {
                        break;
                    }
                }
            }
            else
            {
                ret = SCOM_E_DEV_NOT_SERIAL_COM;
                break;
            }
        }
        else
        {
            ret = SCOM_E_DEV_NOT_FOUND;
        }
    }

    libusbp_list_free(dev_list);

    return ret;
}

/*!
 *  @brief This API is used to send the data.
 *
 */
int8_t scom_write(void * buffer, uint32_t n_bytes)
{
    ssize_t bytes_written = 0;

    if (serial_handle)
    {
        bytes_written = write(serial_handle, buffer, n_bytes);
        if (bytes_written == (ssize_t)n_bytes)
        {
            return SCOM_OK;
        }
        else
        {
            return SCOM_E_WRITE_FAILED;
        }
    }
    else
    {
        return SCOM_E_PORT_NOT_OPEN;
    }
}

/*!
 *  @brief This API is used to read the data.
 *
 */
int8_t scom_read(void * buffer, uint32_t n_bytes, uint32_t *n_bytes_read)
{
    ssize_t bytes_read = 0;

    if (serial_handle)
    {
        bytes_read = read(serial_handle, buffer, n_bytes);
        if (bytes_read < 0)
        {
            return SCOM_E_READ_FAILED;
        }

        *n_bytes_read = bytes_read;

        return SCOM_OK;
    }
    else
    {
        return SCOM_E_PORT_NOT_OPEN;
    }
}

/*!
 * @brief This API is used to close the serial communication.
 *
 */
static int8_t usb_cdc_acm_close(void)
{
    if (serial_handle)
    {
        close(serial_handle);
        serial_handle = 0;

        return SCOM_OK;
    }
    else
    {
        return SCOM_E_PORT_NOT_OPEN;
    }
}

/*!
 * @brief This API is used to clear transmit and receive buffer.
 *
 */
int8_t scom_clear_buffer(void)
{
    if (tcflush(serial_handle, TCIOFLUSH) == 0)
    {
        return SCOM_OK;
    }

    return SCOM_E_FLUSH_FAILED;
}
#endif /* PLATFORM_LINUX */

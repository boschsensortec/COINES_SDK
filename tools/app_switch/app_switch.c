/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    app_switch.c
 * @date    Jan 28, 2019
 * @brief   This file contains support for switching applications in APP2.0/APP3.X Board
 *
 */

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "coines.h"
#include "comm_intf.h"

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#include <setupapi.h>
#endif

#ifdef PLATFORM_LINUX
#include <libusb-1.0/libusb.h>
#include <unistd.h>
#endif
#include <libgen.h>

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/
#define APP20_DD_FW_ADDR       (0x00410000)
#define USB_DFU_BL_ADDR        (0) /* It is actually not at 0x0 ! */
#define APP20_EXAMPLE_ADDR     (0x00440000)
#define APP30_MTP_FW_ADDR      (0x28000)
#define APP31_MTP_FW_ADDR      (0x28000)

#define APP_SWITCH_FEATURE     (0x30)

#define ROBERT_BOSCH_USB_VID   (0x108C)

#define BST_APP20_CDC_USB_PID  (0xAB2C)
#define BST_APP20_DFU_USB_PID  (0xAB2D)

#define BST_APP30_CDC_USB_PID  (0xAB3C)
#define BST_APP30_DFU_USB_PID  (0xAB3D)

#define BST_APP31_CDC_USB_PID  (0xAB38)
#define BST_APP31_DFU_USB_PID  (0xAB39)

#define APP20_BOARD            (3)
#define APP30_BOARD            (5)
#define TIMEOUT                (1000)

/**********************************************************************************/
/* global variables */
/**********************************************************************************/
uint32_t app_address = -1;
char *port_name = NULL;

/**********************************************************************************/
/* static variables */
/**********************************************************************************/

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/
static int usb_connected(char *port_name, uint16_t vid, uint16_t pid);
static void usb_cdc_acm_open_close(uint32_t baud_rate, uint16_t vid, uint16_t pid);


#ifdef PLATFORM_WINDOWS
static void create_serial_handle(uint32_t baud_rate, char *COM_PortName);

#endif

static void set_app_address(uint32_t *app_address, char* argv[]);
static void jump_to(uint32_t addr);
static bool is_board_switched_dfu_mode(void);
static void wait_for_dfu_mode_entry(uint32_t app_address);
static int detect_connected_usb_device();
static void switch_mode(uint16_t pid);

int main(int argc, char *argv[])
{
    int16_t rslt;
    struct coines_board_info board_info;

    if (argc > 3 || argc == 1)
    {
        printf("\n Invalid/Insufficient arguments !!");
        printf("\n\n %s <application name / start address>", argv[0]);
        printf("\n\n Supported Application names in APP2.0 Board [ dd_fw | usb_dfu_bl | example ]");
        printf("\n\n Supported Application names in APP3.0 Board [ usb_dfu_bl | usb_mtp ]");
        printf("\n\n Eg: 1. %s 0x440000", argv[0]);
        printf("\n     2. %s example", argv[0]);
        printf("\n");
        exit(EXIT_FAILURE);
    }

    app_address = strtol(argv[1], NULL, 0);
    port_name = argv[2];
    set_app_address(&app_address, argv);

    /* If the APP board contains DD firmware, it will return success; otherwise, it will indicate failure for the coines
     * bridge firmware */
    rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
    if (rslt == 0)
    {
        /* DD Firmware */
        jump_to(app_address);

        coines_get_board_info(&board_info);

        if (board_info.software_id < 0x31 && board_info.board == APP20_BOARD)
        {
            printf("\n\n Application Switch Feature not supported !"
                   "\n Please upgrade to the latest DD2.0 firmware.\n");
            exit(-3);
        }
    }
    else if (rslt < 0)
    {
        /* COINES Bridge firmware */
        switch_mode(detect_connected_usb_device());
    }

    exit(EXIT_SUCCESS);

}

static void jump_to(uint32_t addr)
{
    comm_intf_init_command_header(COINES_DD_SET, APP_SWITCH_FEATURE);
    comm_intf_put_u32(addr);
    comm_intf_send_command(NULL); /*Response not required and hence passing NULL*/
}

static bool is_board_switched_dfu_mode(void)
{
    if (usb_connected(NULL, ROBERT_BOSCH_USB_VID, BST_APP20_DFU_USB_PID)
        || usb_connected(NULL, ROBERT_BOSCH_USB_VID, BST_APP30_DFU_USB_PID)
        || usb_connected(NULL, ROBERT_BOSCH_USB_VID, BST_APP31_DFU_USB_PID))
    {
        return true;
    }

    return false;
}

static void set_app_address(uint32_t *app_address, char* argv[])
{
    if (*app_address == 0)
    {
        if (strcmp(argv[1], "dd_fw") == 0)
        {
            *app_address = APP20_DD_FW_ADDR;
        }
        else if ((strcmp(argv[1], "usb_dfu_bl") == 0) || (strcmp(argv[1], "0") == 0))
        {
            *app_address = USB_DFU_BL_ADDR;
        }
        else if (strcmp(argv[1], "example") == 0)
        {
            *app_address = APP20_EXAMPLE_ADDR;
        }
        else if (strcmp(argv[1], "usb_mtp") == 0)
        {
            *app_address = APP30_MTP_FW_ADDR;
        }
        else
        {
            *app_address = -1;
        }
    }
}

static int detect_connected_usb_device()
{
    if (usb_connected(port_name, ROBERT_BOSCH_USB_VID, BST_APP31_CDC_USB_PID))
    {
        return BST_APP31_CDC_USB_PID;
    }
    else if (usb_connected(port_name, ROBERT_BOSCH_USB_VID, BST_APP30_CDC_USB_PID))
    {
        return BST_APP30_CDC_USB_PID;
    }
    else if (usb_connected(port_name, ROBERT_BOSCH_USB_VID, BST_APP20_CDC_USB_PID))
    {
        return BST_APP20_CDC_USB_PID;
    }
    else if (usb_connected(NULL, ROBERT_BOSCH_USB_VID, BST_APP20_DFU_USB_PID)
             || usb_connected(NULL, ROBERT_BOSCH_USB_VID, BST_APP30_DFU_USB_PID)
             || usb_connected(NULL, ROBERT_BOSCH_USB_VID, BST_APP31_DFU_USB_PID))
    {
        if ((app_address == APP30_MTP_FW_ADDR) || (app_address == APP31_MTP_FW_ADDR))
        {
            printf("\n Switching from DFU to MTP mode is not allowed ! \n");
            return EXIT_FAILURE;
        }

        exit(EXIT_SUCCESS);
    }
    else
    {
        printf("\n\n Application Board in use (or) seems to be unconnected !\n");
        return EXIT_FAILURE;
    }
}

static void wait_for_dfu_mode_entry(uint32_t app_address)
{
    uint32_t start_time;

    if (app_address == USB_DFU_BL_ADDR)
    {
        /*Wait till APP2.0/APP3.x Board switches to DFU mode*/
        start_time = coines_get_millis();
        while (coines_get_millis() <= (start_time + TIMEOUT))
        {
            if (is_board_switched_dfu_mode())
            {
                exit(EXIT_SUCCESS);
            }
        }
    }

    exit(EXIT_FAILURE);
}

static void switch_mode(uint16_t pid)
{
    if (app_address == USB_DFU_BL_ADDR)
    {
        usb_cdc_acm_open_close(1200, ROBERT_BOSCH_USB_VID, pid);
        wait_for_dfu_mode_entry(app_address);
    }
    else if (app_address == APP20_EXAMPLE_ADDR)
    {
        printf("\n Example already running ! \n");
        exit(EXIT_SUCCESS);
    }
    else if (app_address == APP30_MTP_FW_ADDR && pid == BST_APP30_CDC_USB_PID)
    {
        usb_cdc_acm_open_close(2400, ROBERT_BOSCH_USB_VID, BST_APP30_CDC_USB_PID);
        exit(EXIT_SUCCESS);
    }
    else if (app_address == APP31_MTP_FW_ADDR && pid == BST_APP31_CDC_USB_PID)
    {
        usb_cdc_acm_open_close(2400, ROBERT_BOSCH_USB_VID, BST_APP31_CDC_USB_PID);
        exit(EXIT_SUCCESS);
    }
    else
    {
        printf("\n Correct your Given APP address ! \n");
        exit(EXIT_SUCCESS);
    }
}

#ifdef PLATFORM_WINDOWS
static int usb_connected(char *port_name, uint16_t vid, uint16_t pid)
{

    char usb_id[20] = {};
    unsigned index;
    HDEVINFO hDevInfo;
    SP_DEVINFO_DATA DeviceInfoData;
    char HardwareID[1024] = {};

    sprintf(usb_id, "VID_%04X&PID_%04X", vid, pid);

    /* List all connected USB devices */
    hDevInfo = SetupDiGetClassDevs(NULL, TEXT("USB"), NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES);
    for (index = 0;; index++)
    {
        DeviceInfoData.cbSize = sizeof(DeviceInfoData);
        if (!SetupDiEnumDeviceInfo(hDevInfo, index, &DeviceInfoData))
        {
            return 0;     /* no match */
        }

        SetupDiGetDeviceRegistryProperty(hDevInfo, &DeviceInfoData, SPDRP_HARDWAREID, NULL, (BYTE*)HardwareID,
                                         sizeof(HardwareID), NULL);
        fflush(stdout);
        if (strstr(HardwareID, usb_id))
        {
            if (port_name != NULL)
            {
                char COM_PortName[20];
                DWORD dwSize = sizeof(COM_PortName);
                DWORD dwType = 0;
                HKEY hDeviceRegistryKey = SetupDiOpenDevRegKey(hDevInfo,
                                                               &DeviceInfoData,
                                                               DICS_FLAG_GLOBAL,
                                                               0,
                                                               DIREG_DEV,
                                                               KEY_READ);

                if ((RegQueryValueEx(hDeviceRegistryKey, "PortName", NULL, &dwType, (LPBYTE)COM_PortName,
                                     &dwSize) == ERROR_SUCCESS) && (dwType == REG_SZ))
                {
                    if (strcmp(port_name, COM_PortName) == 0)
                    {
                        coines_delay_msec(2000);

                        return 1;
                    }
                }
            }
            else
            {
                coines_delay_msec(2000);

                return 1;
            }
        }
    }

    return 0;
}

static void create_serial_handle(uint32_t baud_rate, char *COM_PortName)
{
    char str[20];
    DCB dcb;
    HANDLE serial_handle;

    snprintf(str, sizeof(str) - 1, "\\\\.\\%s", COM_PortName);
    serial_handle = CreateFile(str, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (serial_handle == INVALID_HANDLE_VALUE)
    {
        printf("\nSerial Port in use !\n");
        exit(EXIT_FAILURE);
    }

    GetCommState(serial_handle, &dcb);

    dcb.BaudRate = baud_rate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;

    SetCommState(serial_handle, &dcb);
    if (serial_handle)
    {
        CloseHandle(serial_handle);
    }

    coines_delay_msec(2000);

}

static void usb_cdc_acm_open_close(uint32_t baud_rate, uint16_t vid, uint16_t pid)
{

    unsigned index;
    HDEVINFO hDevInfo;
    SP_DEVINFO_DATA DeviceInfoData;
    char HardwareID[1024] = {};
    char usb_id[20] = {};

    sprintf(usb_id, "VID_%04X&PID_%04X", vid, pid);

    hDevInfo = SetupDiGetClassDevs(NULL, TEXT("USB"), NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES);
    for (index = 0;; index++)
    {
        DeviceInfoData.cbSize = sizeof(DeviceInfoData);
        if (!SetupDiEnumDeviceInfo(hDevInfo, index, &DeviceInfoData))
        {
            return;     /* no match */
        }

        SetupDiGetDeviceRegistryProperty(hDevInfo, &DeviceInfoData, SPDRP_HARDWAREID, NULL, (BYTE*)HardwareID,
                                         sizeof(HardwareID), NULL);
        fflush(stdout);
        if (strstr(HardwareID, usb_id))
        {
            char COM_PortName[20];
            DWORD dwSize = sizeof(COM_PortName);
            DWORD dwType = 0;
            HKEY hDeviceRegistryKey = SetupDiOpenDevRegKey(hDevInfo,
                                                           &DeviceInfoData,
                                                           DICS_FLAG_GLOBAL,
                                                           0,
                                                           DIREG_DEV,
                                                           KEY_READ);

            if ((RegQueryValueEx(hDeviceRegistryKey, "PortName", NULL, &dwType, (LPBYTE)COM_PortName,
                                 &dwSize) == ERROR_SUCCESS) && (dwType == REG_SZ))
            {
                if (port_name)
                {
                    if (strcmp(port_name, COM_PortName) == 0)
                    {
                        create_serial_handle(baud_rate, COM_PortName);

                        return;
                    }
                }
                else
                {
                    create_serial_handle(baud_rate, COM_PortName);

                    return;
                }
            }
        }
    }
}
#endif

#ifdef PLATFORM_LINUX
static int usb_connected(char *port_name, uint16_t vid, uint16_t pid)
{
    (void)port_name;
    libusb_init(NULL);
    libusb_device **list;
    struct libusb_device_descriptor desc;

    int cnt = libusb_get_device_list(NULL, &list);

    for (int i = 0; i < cnt; i++)
    {
        libusb_device *device = list[i];
        libusb_get_device_descriptor(device, &desc);

        if (desc.idVendor == vid && desc.idProduct == pid)
        {
            coines_delay_msec(2000);

            return 1;
        }
    }

    libusb_free_device_list(list, 1);

    return 0;

}

static void usb_cdc_acm_open_close(uint32_t baud_rate, uint16_t vid, uint16_t pid)
{
#define DTR  (1 << 0)
#define RTS  (1 << 1)

    struct libusb_device_handle *dev = NULL;
    libusb_init(NULL);
    dev = libusb_open_device_with_vid_pid(NULL, vid, pid);

    if (dev == NULL)
    {
        return;
    }

    for (int i = 0; i < 2; i++)
    {
        if (libusb_kernel_driver_active(dev, i))
        {
            libusb_detach_kernel_driver(dev, i);
        }

        libusb_claim_interface(dev, i);
    }

    /* 1200 8N1 , 1200 = 0x000004B0*/
    unsigned char data[] = { 0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x08 };

    memcpy(data, &baud_rate, 4);

    libusb_control_transfer(dev, 0x21, 0x22, DTR | RTS, 0, NULL, 0, 0);
    libusb_control_transfer(dev, 0x21, 0x20, 0, 0, data, sizeof(data), 0);
    libusb_control_transfer(dev, 0x21, 0x22, 0x00, 0, NULL, 0, 0);

    coines_delay_msec(1000);

    libusb_control_transfer(dev, 0x21, 0x22, DTR | RTS, 0, NULL, 0, 0);
    libusb_control_transfer(dev, 0x21, 0x20, 0, 0, data, sizeof(data), 0);
    libusb_control_transfer(dev, 0x21, 0x22, 0x00, 0, NULL, 0, 0);

    coines_delay_msec(1000);
}
#endif

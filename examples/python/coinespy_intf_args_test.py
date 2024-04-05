#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
(c) Bosch Sensortec GmbH, Reutlingen, Germany
Open Source as per the BSD-3 Clause
"""
# pylint: disable=no-member

import coinespy as cpy
from coinespy import ErrorCodes

# Please change the name and address with BLE name of the App board under test
BLE_NAME = "APP Board 3.0(B6-E5)"
BLE_ADDR = "dd:fc:ab:af:b6:e5"

COM_INTF = cpy.CommInterface.USB

ROBERT_BOSCH_USB_VID = 0x108C
ARDUINO_USB_VID = 0x2341
BST_APP30_CDC_USB_PID = 0xAB3C
BST_APP20_CDC_USB_PID = 0xAB2C
ARDUINO_NICLA_USB_PID = 0x0060

if __name__ == "__main__":

    board = cpy.CoinesBoard()

    print('coinespy version - %s' % cpy.__version__)

    if COM_INTF is cpy.CommInterface.BLE:
        ble_info, ble_count = board.scan_ble_devices(0)
        if board.error_code == ErrorCodes.COINES_SUCCESS and ble_count:
            print("BLE scan results:")
            for i in range(ble_count):
                print(f"[{i}] {ble_info[i].identifier} [{ble_info[i].address}]")

        ble_com_config = cpy.BleComConfig()
        ble_com_config.address = BLE_ADDR
        board.open_comm_interface(COM_INTF, ble_com_config=ble_com_config)
    elif COM_INTF is cpy.CommInterface.USB:
        serial_com_config = cpy.SerialComConfig()
        serial_com_config.com_port_name = "COM5"
        serial_com_config.baud_rate = 9600
        serial_com_config.vendor_id = ROBERT_BOSCH_USB_VID
        serial_com_config.product_id = BST_APP30_CDC_USB_PID
        serial_com_config.rx_buffer_size = 2048
        board.open_comm_interface(COM_INTF, serial_com_config=serial_com_config)
    if board.error_code != ErrorCodes.COINES_SUCCESS:
        print(f'Could not connect to board: {board.error_code}')
    else:
        b_info = board.get_board_info()
        print(f"coines lib version: {board.lib_version}")
        print(f'BoardInfo: HW/SW ID: {hex(b_info.HardwareId)}/{hex(b_info.SoftwareId)}')
        board.close_comm_interface()

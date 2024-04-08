# -*- coding: utf-8 -*-
"""

Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.

BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Created on Tue Jan 28 13:56:48 2020

PC based tool for performing BLE DFU on Bosch Sensortec Application Board 3.0

nRF5 devices with Adafruit / legacy (nRF SDK v11 or lesser) bootloaders are also supported
https://github.com/adafruit/Adafruit_nRF52_Bootloader

Works with latest Bluetooth v4.0 USB dongles and recent notebook PCs with Bluetooth.
Tested with CSR8510 dongle in Windows 10 (Build 16299 and above) and Ubuntu 18.04 LTS
"""

import argparse
import asyncio
import os
import struct
import sys

from bleak import BleakClient
from bleak import BleakScanner

DFU_SERVICE_UUID = "00001530-1212-efde-1523-785feabcd123"
DFU_CONTROL_POINT_UUID = "00001531-1212-efde-1523-785feabcd123"
DFU_PACKET_UUID = "00001532-1212-efde-1523-785feabcd123"
DFU_VERSION_UUID = "00001534-1212-efde-1523-785feabcd123"

BLE_GATT_WRITE_LEN = 20

INIT_DATA_PREFIX = bytearray([0x52, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0xfe, 0xff])
SIZE_PACKET_PREFIX = bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

PROGRESS_BAR_SIZE = 25

# Information from nRF5 SDK v11.0.0 documentation
# https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v11.0.0%2Fbledfu_transport_bleservice.html
START_DFU = 1
RECEIVE_INIT = 2
RECEIVE_FW = 3
VALIDATE = 4
ACTIVATE_N_RESET = 5
SYS_RESET = 6
IMAGE_SIZE_REQ = 7
PKT_RCPT_NOTIF_REQ = 8
RESPONSE = 16
PKT_RCPT_NOTIF = 17

RECEIVE_INIT_RDY = 0
RECEIVE_INIT_COMPLETE = 1

NO_IMAGE = 0
SOFTDEVICE = 1
BOOTLOADER = 2
SOFTDEVICE_BOOTLOADER = 3
APPLICATION = 4

SUCCESS = 1
INVALID_STATE = 2
NOT_SUPPORTED = 3
DATA_SIZE_LIMIT_EXCEEDED = 4
CRC_ERROR = 5
OPERATION_FAILED = 6


# callback for notification handler
def notify_dfu_controlpoint(sender, data):
    global notification_flag, dfu_response_code
    notification_flag = True
    if data[0] is RESPONSE:
        dfu_response_code = data[2]


# CRC16 calculation
def crc16(data, length):
    crc = 0xFFFF
    for i in range(0, length):
        crc  = (crc >> 8) | (crc << 8)
        crc &= 0xFFFF
        crc ^= data[i];
        crc ^= (crc & 0xFF) >> 4;
        crc ^= (crc << 8) << 4;
        crc &= 0xFFFF
        crc ^= ((crc & 0xFF) << 4) << 1;
        crc &= 0xFFFF
    return crc & 0xFFFF


async def scan_devices():
    """Scan for BLE devices and print the discovered devices."""
    print("Scanning for devices...")
    scanner = BleakScanner()
    await scanner.start()
    devices = await scanner.discover(return_adv=True)
    await scanner.stop()
    if devices:
        for dev, adv_data in devices.values():
            ble_name = dev.name if dev.name else "Unknown"
            print(f"{ble_name: <20} \t {dev.address: >18} \t {adv_data.rssi:+3d} dB")
    else:
        print("No devices found")


async def check_dfu_service(client):
    dfu_service_found = False

    connected = client.is_connected
    if connected is True:
        print("Connected to " + client.address)

    for service in client.services:
        if service.uuid == DFU_SERVICE_UUID:
            dfu_service_found = True

    # Read DFU version
    if dfu_service_found is True:
        value = await client.read_gatt_char(DFU_VERSION_UUID)
        print("nRF5 BLE DFU service found - v" + str(value[1]) + "." + str(value[0]) + "\n")
        await asyncio.sleep(.1)

    return dfu_service_found


async def start_application_dfu(client):
    value = bytearray([START_DFU, APPLICATION])
    await client.write_gatt_char(DFU_CONTROL_POINT_UUID, value, response=True)
    await asyncio.sleep(.1)


async def write_image_size(client, firmware_file_size):
    value = SIZE_PACKET_PREFIX + struct.pack('<L', firmware_file_size)
    await client.write_gatt_char(DFU_PACKET_UUID, value, response=True)


async def wait_for_flash_erase():
    sys.stdout.write("Erasing ...\r")
    while(notification_flag is False):
        await asyncio.sleep(1)
    sys.stdout.write("Erase complete !\r")


async def dfu_start_initialization(client, firmware_file, firmware_file_size):
    value = bytearray([RECEIVE_INIT, RECEIVE_INIT_RDY])
    await client.write_gatt_char(DFU_CONTROL_POINT_UUID, value, response=True)
    await asyncio.sleep(.2)

    with open(firmware_file, 'rb') as fh:
        firmware_data = fh.read()

    crc16_info = crc16(firmware_data, firmware_file_size)
    value = INIT_DATA_PREFIX + struct.pack('<H', crc16_info)
    await client.write_gatt_char(DFU_PACKET_UUID, value, response=True)
    await asyncio.sleep(.2)

    value = bytearray([RECEIVE_INIT, RECEIVE_INIT_COMPLETE])
    await client.write_gatt_char(DFU_CONTROL_POINT_UUID, value, response=True)
    await asyncio.sleep(.5)


async def request_firmware_image(client):
    value = bytearray([RECEIVE_FW])
    await client.write_gatt_char(DFU_CONTROL_POINT_UUID, value, response=True)
    await asyncio.sleep(.2)


async def upload_firmware(client, firmware_file, firmware_file_size):
    fo = open(firmware_file, 'rb')
    tail_byte_count = firmware_file_size % BLE_GATT_WRITE_LEN
    if tail_byte_count == 0:
        num_of_blocks = int(firmware_file_size / BLE_GATT_WRITE_LEN)
    else:
        num_of_blocks = int((firmware_file_size / BLE_GATT_WRITE_LEN) + 1)

    for i in range(0, num_of_blocks):
        line = fo.read(BLE_GATT_WRITE_LEN)
        value = bytearray(line)
        await client.write_gatt_char(DFU_PACKET_UUID, value)
        if dfu_response_code is not SUCCESS:
            print("\nDFU download error !")
            exit(-dfu_response_code)

        t = int((i + 1) * PROGRESS_BAR_SIZE / num_of_blocks)
        no_of_symbols = -1
        if (t != no_of_symbols):
            no_of_symbols = t
            sys.stdout.write("Download  [")
            sys.stdout.write('=' * no_of_symbols)
            sys.stdout.write(' ' * (PROGRESS_BAR_SIZE - no_of_symbols))
            sys.stdout.write("]  %d %% (%.2f kB/%.2f kB)" % ((i + 1) * 100 / num_of_blocks, (i * 20 + tail_byte_count) / 1024., firmware_file_size / 1024.))
            sys.stdout.write('\r')
            sys.stdout.flush()

        await asyncio.sleep(.025)

    print('\n')
    await asyncio.sleep(1)


async def validate_firmware(client):
    value = bytearray([VALIDATE])
    await client.write_gatt_char(DFU_CONTROL_POINT_UUID, value, response=True)
    await asyncio.sleep(.5)


async def activate_and_reset(client):
    value = bytearray([ACTIVATE_N_RESET])
    await client.write_gatt_char(DFU_CONTROL_POINT_UUID, value, response=True)
    await asyncio.sleep(.5)


async def run(address, loop, firmware_file, scan):
    global notification_flag, dfu_response_code
    notification_flag = False
    dfu_response_code = SUCCESS

    if scan:
        await scan_devices()
        exit(0)

    async with BleakClient(address, loop=loop) as client:
        dfu_service_found = await check_dfu_service(client)
        if not dfu_service_found:
            print("No DFU service! Disconnecting ...")
            await client.disconnect()
            exit(-1)

        await client.start_notify(DFU_CONTROL_POINT_UUID, notify_dfu_controlpoint)

        await start_application_dfu(client)

        firmware_file_size = os.path.getsize(firmware_file)
        await write_image_size(client, firmware_file_size)

        await wait_for_flash_erase()

        if notification_flag and dfu_response_code == SUCCESS:
            notification_flag = False
            await dfu_start_initialization(client, firmware_file, firmware_file_size)
        else:
            print("DFU Initialization error!")
            exit(-dfu_response_code)

        if notification_flag and dfu_response_code == SUCCESS:
            notification_flag = False
            await request_firmware_image(client)
            await upload_firmware(client, firmware_file, firmware_file_size)
        else:
            print("DFU download error!")
            exit(-dfu_response_code)

        if notification_flag and dfu_response_code == SUCCESS:
            notification_flag = False
            await validate_firmware(client)
        else:
            print("DFU download error!")
            exit(-dfu_response_code)

        if notification_flag and dfu_response_code == SUCCESS:
            print("DFU Success!")
            notification_flag = False
            await activate_and_reset(client)
        else:
            print("Firmware integrity check failed!")
            exit(-dfu_response_code)


if __name__ == "__main__":
    print("Application Board 3.0 BLE DFU tool")
    print("Bosch Sensortec GmbH (C) 2023")
    cmdline_parser = argparse.ArgumentParser()
    cmdline_parser.add_argument('-l', '--list', dest='scan', help='Scan for BLE devices', action='store_true')
    required_args = cmdline_parser.add_argument_group('required arguments')
    required_args.add_argument('-d', dest='device_mac_addr', help='Specify device MAC address')
    required_args.add_argument('-f', dest='firmware_bin_file', help='Specify firmware binary')
    args = cmdline_parser.parse_args()
    devices = args.device_mac_addr
    firmware = args.firmware_bin_file
    scan = args.scan

    if args.scan is False and (args.device_mac_addr is None or args.firmware_bin_file is None):
        cmdline_parser.error("-f and -d to be used together !")

    if firmware is not None and not os.path.isfile(firmware):
            print(f"'{firmware}' doesn't exist!")
            exit(1)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(devices, loop, firmware, scan))

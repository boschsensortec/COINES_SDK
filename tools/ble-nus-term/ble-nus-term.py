# -*- coding: utf-8 -*-
"""
Copyright (C) 2023 Bosch Sensortec GmbH

SPDX-License-Identifier: BSD-3-Clause

Date : Aug 14, 2020

PC based tool for interacting with BLE Nordic UART service.
Works with BLE enabled COINES_SDK examples compiled for Application Board 3.0.

Works with latest Bluetooth v4.0 USB dongles and recent notebook PCs with Bluetooth.
Tested with CSR8510 dongle in Windows 10 (Build 16299 and above) and Ubuntu 20.04 LTS
"""

import argparse
import asyncio
import sys
import signal
import aioconsole

from bleak import BleakClient
from bleak import BleakScanner

BLE_NUS_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLE_GATT_WRITE_LEN = 20


def nus_data_rcv_handler(sender, data):
    """Handle received NUS data."""
    sys.stdout.buffer.write(data)
    sys.stdout.flush()


def cleanup_handler(signum, frame):
    """Handle cleanup tasks on receiving a signal."""
    for task in asyncio.all_tasks():
        task.cancel()


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


async def connect_and_handle_notifications(client):
    """Connect to a BLE client and handle notifications."""
    ble_nus_found = False
    connected = client.is_connected
    if connected:
        print("Connected to " + client.address)
        print("")

    for service in client.services:
        if service.uuid == BLE_NUS_UUID:
            ble_nus_found = True

    if ble_nus_found:
        await client.start_notify(NUS_TX_UUID, nus_data_rcv_handler)
    else:
        print("Nordic UART Service (NUS) not found! Disconnecting ...")


async def handle_console_input(client):
    """Handle console input and write it to the NUS RX characteristic."""
    while True:
        try:
            line_input = await aioconsole.ainput()
            line_byte_arr = (line_input + '\n').encode()
            length = len(line_byte_arr)
            if length <= BLE_GATT_WRITE_LEN:
                await client.write_gatt_char(NUS_RX_UUID, line_byte_arr)
            else:
                for i in range(0, int(length / BLE_GATT_WRITE_LEN) + 1):
                    ble_gatt_byte_arr = line_byte_arr[i * BLE_GATT_WRITE_LEN:(i + 1) * BLE_GATT_WRITE_LEN]
                    await client.write_gatt_char(NUS_RX_UUID, ble_gatt_byte_arr)
            await asyncio.sleep(.1)
        except asyncio.CancelledError:
            await client.stop_notify(NUS_TX_UUID)
            break


async def run(address, loop, scan):
    """
    Run the BLE client application.

    Args:
        address (str): The address of the BLE device to connect to.
        loop (asyncio.AbstractEventLoop): The event loop to use.
        scan (bool): Flag indicating whether to scan for devices or connect directly.

    """
    if scan:
        await scan_devices()
        exit(0)

    async with BleakClient(address, loop=loop) as client:
        await connect_and_handle_notifications(client)
        await handle_console_input(client)


if __name__ == "__main__":
    print("Application Board 3.0 BLE NUS Terminal")
    print("Bosch Sensortec GmbH (C) 2023")
    print("")
    cmdline_parser = argparse.ArgumentParser()
    cmdline_parser.add_argument(
        '-l', '--list', dest='scan', help='Scan for BLE devices', action='store_true')
    required_args = cmdline_parser.add_argument_group('required arguments')
    required_args.add_argument(
        '-d', dest='device_mac_addr', help='Specify device MAC address')
    args = cmdline_parser.parse_args()
    devices = args.device_mac_addr
    scan = args.scan

    if args.scan is False and args.device_mac_addr is None:
        cmdline_parser.print_usage()
        exit(1)

    loop = asyncio.get_event_loop()
    signal.signal(signal.SIGINT, cleanup_handler)
    signal.signal(signal.SIGINT, cleanup_handler)
    loop.run_until_complete(run(devices, loop, scan))

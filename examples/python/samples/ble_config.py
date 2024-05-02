#!/usr/bin/env python3
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
"""
# pylint: disable=no-member

import coinespy as cpy
from coinespy import ErrorCodes

# Please change the name and address with BLE name and address of the App board under test
BLE_NAME = "APP Board 3.1(C3-F4)"
BLE_ADDR = "c1:c5:2f:20:c3:f4"

COM_INTF = cpy.CommInterface.BLE

if __name__ == "__main__":

    board = cpy.CoinesBoard()

    print("COINESPY version - %s" % cpy.__version__)

    # Scan for BLE devices
    ble_info, ble_count = board.scan_ble_devices(0)

    # Configure BLE communication
    ble_com_config = cpy.BleComConfig()
    ble_com_config.address = BLE_ADDR
    # ble_com_config.identifier = BLE_NAME

    # Open communication
    board.open_comm_interface(COM_INTF, ble_com_config=ble_com_config)

    if board.error_code != ErrorCodes.COINES_SUCCESS:
        print(f"Could not connect to board: {board.error_code}")
    else:
        # Get and print board info
        board_info = board.get_board_info()
        print(f"COINES SDK version: {board.lib_version}")
        print(
            f"BoardInfo: HW/SW ID: {hex(board_info.HardwareId)}/{hex(board_info.SoftwareId)}"
        )
        board.soft_reset()

        # Close communication
        board.close_comm_interface()

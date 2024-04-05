#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
(c) Bosch Sensortec GmbH, Reutlingen, Germany
Open Source as per the BSD-3 Clause
"""
# pylint: disable=no-member

import coinespy as cpy
from coinespy import ErrorCodes

COM_INTF = cpy.CommInterface.USB

if __name__ == "__main__":

    board = cpy.CoinesBoard()

    print('coinespy version - %s' % cpy.__version__)

    board.open_comm_interface(COM_INTF)

    if board.error_code != ErrorCodes.COINES_SUCCESS:
        print(f'Could not connect to board: {board.error_code}')
    else:
        b_info = board.get_board_info()
        print(f"coines lib version: {board.lib_version}")
        print(f'BoardInfo: HW/SW ID: {hex(b_info.HardwareId)}/{hex(b_info.SoftwareId)}')
        board.close_comm_interface()

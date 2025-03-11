#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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
# This example works with Application Board 3.X with BMA400 shuttle board
# This is a simple example of BMA400 data read using coinespy
# pylint: disable=no-member, too-few-public-methods, protected-access

import sys
import time
import coinespy as cpy
from coinespy import ErrorCodes

# Dictionary mapping output data rate (ODR) names to their corresponding values
odr = {"ODR_25_HZ": 0x06, "ODR_50_HZ": 0x07, "ODR_100_HZ": 0x08,"ODR_200_HZ": 0x09,"ODR_400_HZ": 0x0a,"ODR_800_HZ": 0x0a}

# The selected output data rate
SELECTED_ODR= odr["ODR_800_HZ"]

# The number of samples to collect
NUM_OF_SAMPLES = 10000

# The length of the data to be streamed
DATA_LENGTH = 6

# The data that has been streamed
streamed_data = []

# Settings for the data stream
stream_settings = dict(
            # The primary I2C address
            I2C_ADDR_PRIMARY=0x14,
            # The number of blocks of data
            NO_OF_BLOCKS=1,
            # The least significant byte of the X register
            REG_X_LSB= 0x04,
            # The number of data bytes
            NO_OF_DATA_BYTES= DATA_LENGTH,
            # The channel ID
            CHANNEL_ID=1,
            # The chip select pin
            CS_PIN=cpy.MultiIOPin.SHUTTLE_PIN_7.value,
            # The interrupt pin
            INT_PIN=cpy.MultiIOPin.SHUTTLE_PIN_20.value,
            # The interrupt timestamp
            INT_TIME_STAMP=0,
            # The hardware pin state
            HW_PIN_STATE=1)
           
class BMA400:
    """ Test api for the shuttle bma400 with app30 board """
    CS_PIN = cpy.MultiIOPin.SHUTTLE_PIN_7
    SHUTTLE_ID = 0x1A1
    I2C_ADDR = 0x14
    SPI_DUMMY_BYTE = 1
   
    def __init__(self, bus, interface=cpy.SensorInterface.SPI):
        print(f"++++++++ {type(bus).__name__} ++++++++")
        self.interface = interface
        self.board = cpy.CoinesBoard()
        self.bst_get_board_details()
        self.counter = 1
        self.missed_packet_count = 0

        if interface == cpy.SensorInterface.SPI:
            self.read_board = self.board.read_spi
            self.write_board = self.board.write_spi
            if isinstance(bus, cpy.SPIBus):
                self.bus_instance = bus
            else:
                raise ValueError("Wrong bus instance")
        else:
            self.read_board = self.board.read_i2c
            self.write_board = self.board.write_i2c
            if isinstance(bus, cpy.I2CBus):
                self.bus_instance = bus
            else:
                raise ValueError("Wrong bus instance")

    def bst_get_board_details(self):
        """ Initial checking of the connected board """
        self.board.open_comm_interface(cpy.CommInterface.USB)
        self.verify_error(keyword="Open Communication interface", exit_flag=True)
        time.sleep(0.2)
        board_info = self.board.get_board_info()
        print(f'BoardInfo: HW/SW ID: {hex(board_info.HardwareId)}/{hex(board_info.SoftwareId)}')
        print(f"ShuttleID: {hex(board_info.ShuttleID)}")
        if board_info.HardwareId == 0:
            print('Seems like there is no board communication. Stop')
            self.board.close_comm_interface()
            sys.exit()
        if board_info.ShuttleID != BMA400.SHUTTLE_ID:
            print('Seems like you are not using BMA400 shuttle board. Stop')
            self.board.close_comm_interface()
            sys.exit()

    def set_vdd_vddio(self, vdd, vddio):
        """ Sets the vdd and/or vddio """
        self.board.set_shuttleboard_vdd_vddio_config(vdd_val=vdd, vddio_val=vddio)
        self.verify_error(keyword="set vdd, vddio", exit_flag=True)

    def read_chip_id(self):
        """Reads the chip id"""
        chip_id = self.read(0x00, 1)
        print(f'Chip ID accel: %#02x' % (chip_id[0 + BMA400.SPI_DUMMY_BYTE]))

    def fetch_power_setup(self):
        """Power up accelerometer and change power mode from sleep to normal mode"""
        pwr_before = self.read(0x19, 1)[0 + BMA400.SPI_DUMMY_BYTE]
        print('Power settings before power-up (expected: 0x00): 0x%02x' % pwr_before)
        # Change power mode from sleep to normal mode
        self.write(0x19, 0x02)
        time.sleep(.2)
        pwr_after = self.read(0x19, 1)[0 + BMA400.SPI_DUMMY_BYTE]
        print('ACC: Power settings after power-up (expected: 0x02): 0x%02x' % pwr_after)

    def meas_range_4g(self):
        """Change to measurement range 4G"""
        acc_config1 = self.read(0x1A, 1)
        acc_config1[0] = acc_config1[0 + BMA400.SPI_DUMMY_BYTE] & 0x3F  # clear bit 7
        acc_config1[0] = acc_config1[0] | 0x40  # set bit 6
        accel_range = acc_config1[0] >> 6
        self.write(0x1A, SELECTED_ODR)
        time.sleep(.2)
        print('\nRange: %dG' % (2 ** (accel_range + 1)))
        return accel_range

    def read_temperature(self):
        """Read temperature sensor data. This may be surprisingly high
        due to self-heating of the ApplicationBoard2"""
        data = self.read(0x11, 1)
        temp = twos_comp(data[0 + BMA400.SPI_DUMMY_BYTE], 8) * 0.5 + 23
        print('\nTemperature: %.2f' % temp)

    def read(self, register_address, number_of_reads):
        ret = self.read_board(self.bus_instance, register_address, number_of_reads + BMA400.SPI_DUMMY_BYTE)
        self.verify_error(keyword="Device read", exit_flag=False)
        return ret

    def write(self, register_address, register_value):
        self.write_board(self.bus_instance, register_address, register_value)
        self.verify_error(keyword="Device write", exit_flag=False)

    def config_bus(self):
        """ Decide to do the bus configuration"""
        if self.interface == cpy.SensorInterface.SPI:
            self.config_spi()
        else:
            self.config_i2c()

    def config_spi(self):
        """ Configuration for SPI """
        self.board.config_spi_bus(self.bus_instance, BMA400.CS_PIN, cpy.SPISpeed.SPI_10_MHZ, cpy.SPIMode.MODE0)
        self.verify_error(keyword="configuring spi bus", exit_flag=True)
        self.board.set_pin_config(BMA400.CS_PIN, cpy.PinDirection.OUTPUT, cpy.PinValue.HIGH)
        self.verify_error(keyword="configuring Pin", exit_flag=True)
        BMA400.SPI_DUMMY_BYTE = 1

    def config_i2c(self):
        """ Configuration for I2C """
        self.board.config_i2c_bus(self.bus_instance, BMA400.I2C_ADDR, cpy.I2CMode.STANDARD_MODE)
        self.verify_error(keyword="configuring i2c bus", exit_flag=True)
        self.board.set_pin_config(BMA400.CS_PIN, cpy.PinDirection.OUTPUT, cpy.PinValue.HIGH)
        self.verify_error(keyword="configuring Pin", exit_flag=True)
        BMA400.SPI_DUMMY_BYTE = 0

    def read_accel_data(self, accel_range):
        """"Read samples from the sensor"""
        a_lsb = [0] * 3
        data = self.read(0x04, 6)
        if self.interface == cpy.SensorInterface.SPI:
            data.pop(0)
        a_lsb[0] = twos_comp(data[1] * 256 + data[0], 12)
        a_lsb[1] = twos_comp(data[3] * 256 + data[2], 12)
        a_lsb[2] = twos_comp(data[5] * 256 + data[4], 12)
        return [(val / 2048. * 2 ** (accel_range + 1)) for val in a_lsb]

    def verify_error(self, keyword=None, exit_flag=False):
        if self.board.error_code != ErrorCodes.COINES_SUCCESS:
            print(f"{keyword} failure: {self.board.error_code}")
            if exit_flag:
                self.board.close_comm_interface()
                sys.exit()

    def enable_interrupt(self):
        self.write(0x1F,0x80)
        self.write(0x21,0x80)

    def streaming_config(self):
        stream_config = cpy.StreamingConfig()
        data_blocks = cpy.StreamingBlocks()
        if self.interface == cpy.SensorInterface.I2C:
            stream_config.Intf = cpy.SensorInterface.I2C.value
            stream_config.I2CBus = cpy.I2CBus.BUS_I2C_0.value
            stream_config.DevAddr = stream_settings["I2C_ADDR_PRIMARY"]

        elif self.interface == cpy.SensorInterface.SPI:
            stream_config.Intf = cpy.SensorInterface.SPI.value
            stream_config.SPIBus = cpy.SPIBus.BUS_SPI_0.value
            stream_config.CSPin = stream_settings["CS_PIN"]

        data_blocks.NoOfBlocks = stream_settings["NO_OF_BLOCKS"]
        for i in range(0, data_blocks.NoOfBlocks):
            data_blocks.RegStartAddr[i] = stream_settings["REG_X_LSB"]
            data_blocks.NoOfDataBytes[i] = stream_settings["NO_OF_DATA_BYTES"] + BMA400.SPI_DUMMY_BYTE

        stream_config.IntTimeStamp = stream_settings["INT_TIME_STAMP"]
        stream_config.IntPin = stream_settings["INT_PIN"]

        stream_config.HwPinState = stream_settings["HW_PIN_STATE"]

        ret = self.board.config_streaming(1, stream_config, data_blocks)

    def analysis_data_loss(self, streamed_data):
        """
        This function parse the response and check for data loss
        """
        lost_packets = []
        for data in streamed_data:
            packet_cnt = int.from_bytes(data[0:4], "big", signed=False)

            if packet_cnt == self.counter:
                self.counter += 1
            else:
                self.missed_packet_count += packet_cnt - \
                    self.counter
                for i in range(packet_cnt - self.counter):
                    lost_packets.append(
                        self.counter + i)
                # Data loss increment the counter to check next packet
                self.counter = packet_cnt + 1


        print("<-----------------Steaming beanchmark result------------------>")
        if self.counter > 1:
            loss_percent = (self.missed_packet_count) / \
                self.counter * 100
            print(
                f'Data loss - {loss_percent:.5f}%, '
                f'Total packets received - {self.counter}, '
                f'Packets lost - {self.missed_packet_count},'
                f'Missed packets - {lost_packets}'
            )
        else:
            print("Streaming not configured")
        print("<------------------------------------------------------------->")
        

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)  # compute negative value
    return val  # return positive value as is


if __name__ == "__main__":
    try:
        BMA400.CS_PIN = cpy.MultiIOPin.SHUTTLE_PIN_7
        bma400 = BMA400(bus=cpy.SPIBus.BUS_SPI_0, interface=cpy.SensorInterface.SPI)
        bma400.set_vdd_vddio(vdd=0, vddio=0)
        bma400.config_bus()
        bma400.set_vdd_vddio(vdd=3.3, vddio=3.3)
        time.sleep(0.2)
        
        if bma400.interface == cpy.SensorInterface.SPI:
            # Dummy-read for accelerometer to switch to SPI mode
            bma400.read_board(bma400.bus_instance, 0x00, 1)

        bma400.read_chip_id()
        bma400.fetch_power_setup()
        acc_range = bma400.meas_range_4g()

        bma400.streaming_config()

        bma400.board.start_stop_streaming(cpy.StreamingMode.STREAMING_MODE_INTERRUPT.value, cpy.StreamingState.STREAMING_START.value)
        bma400.verify_error("Start Interrupt streaming")

        bma400.enable_interrupt()

        for i in range(1, NUM_OF_SAMPLES):
            (error_code, stream_buffer, valid_sample_count) = bma400.board.read_stream_sensor_data(1, 1)
            streamed_data.append(stream_buffer[0: BMA400.SPI_DUMMY_BYTE + stream_settings["NO_OF_DATA_BYTES"] + 10]) # 4bytes packet counter + data + 6bytes timestamp

        bma400.board.start_stop_streaming(cpy.StreamingMode.STREAMING_MODE_INTERRUPT.value, cpy.StreamingState.STREAMING_STOP.value)

        print('Streamed data')
        for data in streamed_data:
            print(f'{data}')

        bma400.analysis_data_loss(streamed_data)

        bma400.board.flush_interface()
        
        bma400.read_temperature()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        bma400.set_vdd_vddio(vdd=0, vddio=0)
        bma400.board.soft_reset()
        bma400.verify_error("soft reset")
        bma400.board.close_comm_interface()

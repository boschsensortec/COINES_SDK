#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This example works with Application Board 3.0 with BMA400 shuttle board
# pylint: disable=no-member, too-few-public-methods, protected-access
"""
This is a simple example of BMA400 data read using coinespy
"""
import sys
import time
import coinespy as cpy
from coinespy import ErrorCodes


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
        print('Chip ID accel: %#02x' % (chip_id[0 + BMA400.SPI_DUMMY_BYTE]))

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
        self.write(0x1A, acc_config1[0])
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
        self.board.config_spi_bus(self.bus_instance, BMA400.CS_PIN, cpy.SPISpeed.SPI_1_MHZ, cpy.SPIMode.MODE0)
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


def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)  # compute negative value
    return val  # return positive value as is


if __name__ == "__main__":
    BMA400.CS_PIN = cpy.MultiIOPin.SHUTTLE_PIN_7
    # Note: The I2C address could be changed to 0x15 by setting a HIGH signal to the SDO pin of
    #   BM400, however this is not possible with the current implementation of the COINES_SDK layer
    
    # bma400 = BMA400(bus=cpy.I2CBus.BUS_I2C_0, interface=cpy.SensorInterface.I2C)
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

    # Read some samples from the sensor
    print('\n  ax\t   ay\t   az')
    for i in range(10):
        a_g = bma400.read_accel_data(acc_range)
        print('%+.3f\t %+.3f\t %+.3f' % (a_g[0], a_g[1], a_g[2]))
        time.sleep(0.05)  # 200Hz ODR

    bma400.read_temperature()
    bma400.set_vdd_vddio(vdd=0, vddio=0)
    bma400.board.soft_reset()
    bma400.verify_error("soft reset")
    bma400.board.close_comm_interface()

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
# This is a simple example of BMI085 data read using coinespy
# This example works with Application Board 3.X with BMI085 shuttle board
# pylint: disable=no-member, too-few-public-methods, protected-access

import coinespy as cpy
import bmi08x_common as bmi08x
from bmi08x_common import BMI08X as BMI08X_CLS
import helper_functions as hfunc


class BMI085(BMI08X_CLS):
    "Child class of BMI08X_CLS with methods for sensor data readout display"
    def __init__(self, **kwargs):
        BMI08X_CLS.__init__(self, kwargs["bus"], kwargs["interface"])
        # ACCEL Range set to 2G
        self.accel_full_range = 2
        self.accel_cfg['RANGE'] = bmi08x.BMI085_ACCEL_RANGE_2G
        #  GYRO Range set to 2000 dps
        self.gyro_full_range = 2000
        self.gyro_cfg['RANGE'] = bmi08x.BMI08X_GYRO_RANGE_2000_DPS


    def read_sensor_data(self, accel_range, gyro_range):
        """"Read samples from the sensor"""
        a_lsb = [0] * 3
        g_lsb = [0] * 3

        data = self.read(bmi08x.SensorType.ACCEL, self.ACCEL_DATA_ADDR, self.ACCEL_DATA_REG_LEN)
        a_lsb[0] = hfunc.twos_comp(data[1] * 256 + data[0], 16)
        a_lsb[1] = hfunc.twos_comp(data[3] * 256 + data[2], 16)
        a_lsb[2] = hfunc.twos_comp(data[5] * 256 + data[4], 16)
        acc_g = [(val / 32768. * self.accel_full_range * 2 ** accel_range)
                 for val in a_lsb]

        data = self.read(bmi08x.SensorType.GYRO, self.GYRO_DATA_ADDR, self.GYRO_DATA_REG_LEN)
        g_lsb[0] = hfunc.twos_comp(data[1] * 256 + data[0], 16)
        g_lsb[1] = hfunc.twos_comp(data[3] * 256 + data[2], 16)
        g_lsb[2] = hfunc.twos_comp(data[5] * 256 + data[4], 16)
        gyr_dps = [(val / 32768. * self.gyro_full_range * 2 ** gyro_range)
                   for val in g_lsb]

        return acc_g, gyr_dps


if __name__ == "__main__":
    bmi085 = BMI085(bus=cpy.I2CBus.BUS_I2C_0,
                    interface=cpy.SensorInterface.I2C)
    # bmi085 = BMI085(bus=cpy.SPIBus.BUS_SPI_0, interface=cpy.SensorInterface.SPI)
    bmi085.init_board()
    
    # Set Accel power, ODR and range settings
    bmi085.set_accel_power_mode()
    acc_range = bmi085.set_accel_meas_conf()

    # Set Gyro power, ODR and range settings
    bmi085.set_gyro_power_mode()
    gyr_range = bmi085.set_gyro_meas_conf()
    print(f"Accel range: {acc_range} Gyro range: {gyr_range}")

    print('\n  ax\t   ay\t   az\t    gx\t     gy\t     gz')
    for i in range(10):
        a_g, g_dps = bmi085.read_sensor_data(acc_range, gyr_range)
        print('%+.3f\t %+.3f\t %+.3f\t  %+07.1f  %+07.1f  %+07.1f' % (a_g[0], a_g[1], a_g[2],
                                                                      g_dps[0], g_dps[1], g_dps[2]))
    bmi085.read_temperature()

    bmi085.close_comm(verify_reset=True)

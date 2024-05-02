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

# This module has constants and common APIs for BMI08x
# pylint: disable=no-member, too-few-public-methods, protected-access, too-many-lines

import sys
import time
from enum import Enum
import coinespy as cpy
from coinespy import ErrorCodes
import helper_functions as hfunc

GRAVITY_EARTH = 9.80665

# *************************** Macro definitions *****************************

BMI08X_READ_WRITE_LEN = 44

# *************************** BMI08 Accelerometer Macros *****************************

# Register map
# Accel registers


class Bmi08xAccelRegs(Enum):
    """ Used to store Accel register addresses"""
    #    Accel Chip Id register
    BMI08X_REG_ACCEL_CHIP_ID = 0x00

    #    Accel Error condition register
    BMI08X_REG_ACCEL_ERR = 0x02

    #    Accel Status flag register
    BMI08X_REG_ACCEL_STATUS = 0x03

    #    Accel X LSB data register
    BMI08X_REG_ACCEL_X_LSB = 0x12

    #    Accel X MSB data register
    BMI08X_REG_ACCEL_X_MSB = 0x13

    #    Accel Y LSB data register
    BMI08X_REG_ACCEL_Y_LSB = 0x14

    #    Accel Y MSB data register
    BMI08X_REG_ACCEL_Y_MSB = 0x15

    #    Accel Z LSB data register
    BMI08X_REG_ACCEL_Z_LSB = 0x16

    #    Accel Z MSB data register
    BMI08X_REG_ACCEL_Z_MSB = 0x17

    #    Sensor time byte 0 register
    BMI08X_REG_ACCEL_SENSORTIME_0 = 0x18

    #    Sensor time byte 1 register
    BMI08X_REG_ACCEL_SENSORTIME_1 = 0x19

    #    Sensor time byte 2 register
    BMI08X_REG_ACCEL_SENSORTIME_2 = 0x1A

    #    Accel Interrupt status0 register
    BMI08X_REG_ACCEL_INT_STAT_0 = 0x1C

    #    Accel Interrupt status1 register
    BMI08X_REG_ACCEL_INT_STAT_1 = 0x1D

    #    Accel general purpose register 0
    BMI08X_REG_ACCEL_GP_0 = 0x1E

    #    Sensor temperature MSB data register
    BMI08X_REG_TEMP_MSB = 0x22

    #    Sensor temperature LSB data register
    BMI08X_REG_TEMP_LSB = 0x23

    #    Accel general purpose register 4
    BMI08X_REG_ACCEL_GP_4 = 0x27

    #    Accel Internal status register
    BMI08X_REG_ACCEL_INTERNAL_STAT = 0x2A

    #    Accel configuration register
    BMI08X_REG_ACCEL_CONF = 0x40

    #    Accel range setting register
    BMI08X_REG_ACCEL_RANGE = 0x41

    #    Accel Interrupt pin 1 configuration register
    BMI08X_REG_ACCEL_INT1_IO_CONF = 0x53

    #    Accel Interrupt pin 2 configuration register
    BMI08X_REG_ACCEL_INT2_IO_CONF = 0x54

    #    Accel Interrupt latch configuration register
    BMI08X_REG_ACCEL_INT_LATCH_CONF = 0x55

    #    Accel Interrupt pin1 mapping register
    BMI08X_REG_ACCEL_INT1_MAP = 0x56

    #    Accel Interrupt pin2 mapping register
    BMI08X_REG_ACCEL_INT2_MAP = 0x57

    #    Accel Interrupt map register
    BMI08X_REG_ACCEL_INT1_INT2_MAP_DATA = 0x58

    #    Accel Init control register
    BMI08X_REG_ACCEL_INIT_CTRL = 0x59

    #    Accel Self test register
    BMI08X_REG_ACCEL_SELF_TEST = 0x6D

    #    Accel Power mode configuration register
    BMI08X_REG_ACCEL_PWR_CONF = 0x7C

    #    Accel Power control (switch on or off  register
    BMI08X_REG_ACCEL_PWR_CTRL = 0x7D

    #    Accel Soft reset register
    BMI08X_REG_ACCEL_SOFTRESET = 0x7E

    #    Feature Config related Registers
    BMI08X_REG_ACCEL_RESERVED_5B = 0x5B
    BMI08X_REG_ACCEL_RESERVED_5C = 0x5C
    BMI08X_REG_ACCEL_FEATURE_CFG = 0x5E


#    BMI085 Accel unique chip identifier
BMI085_ACCEL_CHIP_ID = 0x1F

#    BMI088 Accel unique chip identifier
BMI088_ACCEL_CHIP_ID = 0x1E

#    Accel I2C slave address
BMI08X_ACCEL_I2C_ADDR_PRIMARY = 0x18
BMI08X_ACCEL_I2C_ADDR_SECONDARY = 0x19

#    Interrupt masks
BMI08X_ACCEL_DATA_READY_INT = 0x80
BMI08X_ACCEL_FIFO_WM_INT = 0x02
BMI08X_ACCEL_FIFO_FULL_INT = 0x01

BMI08X_GYRO_DATA_READY_INT = 0x80
BMI08X_GYRO_FIFO_WM_INT = 0x10
BMI08X_GYRO_FIFO_FULL_INT = 0x10

#    Accel Bandwidth
BMI08X_ACCEL_BW_OSR4 = 0x08
BMI08X_ACCEL_BW_OSR2 = 0x09
BMI08X_ACCEL_BW_NORMAL = 0x0A

#    BMI085 Accel Range
BMI085_ACCEL_RANGE_2G = 0x00
BMI085_ACCEL_RANGE_4G = 0x01
BMI085_ACCEL_RANGE_8G = 0x02
BMI085_ACCEL_RANGE_16G = 0x03

#  BMI088 Accel Range
BMI088_ACCEL_RANGE_3G = 0x00
BMI088_ACCEL_RANGE_6G = 0x01
BMI088_ACCEL_RANGE_12G = 0x02
BMI088_ACCEL_RANGE_24G = 0x03

#    Accel Output data rate
BMI08X_ACCEL_ODR_12_5_HZ = 0x05
BMI08X_ACCEL_ODR_25_HZ = 0x06
BMI08X_ACCEL_ODR_50_HZ = 0x07
BMI08X_ACCEL_ODR_100_HZ = 0x08
BMI08X_ACCEL_ODR_200_HZ = 0x09
BMI08X_ACCEL_ODR_400_HZ = 0x0A
BMI08X_ACCEL_ODR_800_HZ = 0x0B
BMI08X_ACCEL_ODR_1600_HZ = 0x0C

#    Accel Self test
BMI08X_ACCEL_SWITCH_OFF_SELF_TEST = 0x00
BMI08X_ACCEL_POSITIVE_SELF_TEST = 0x0D
BMI08X_ACCEL_NEGATIVE_SELF_TEST = 0x09

#    Accel Power mode
BMI08X_ACCEL_PM_ACTIVE = 0x00
BMI08X_ACCEL_PM_SUSPEND = 0x03

#    Accel Power control settings
BMI08X_ACCEL_POWER_DISABLE = 0x00
BMI08X_ACCEL_POWER_ENABLE = 0x04

#    Accel internal interrupt pin mapping
BMI08X_ACCEL_INTA_DISABLE = 0x00
BMI08X_ACCEL_INTA_ENABLE = 0x01
BMI08X_ACCEL_INTB_DISABLE = 0x00
BMI08X_ACCEL_INTB_ENABLE = 0x02
BMI08X_ACCEL_INTC_DISABLE = 0x00
BMI08X_ACCEL_INTC_ENABLE = 0x04

#    Accel Soft reset delay
BMI08X_ACCEL_SOFTRESET_DELAY_MS = 1

#    Mask definitions for ACCEL_ERR_REG register
BMI08X_FATAL_ERR_MASK = 0x01
BMI08X_ERR_CODE_MASK = 0x1C

#    Position definitions for ACCEL_ERR_REG register
BMI08X_CMD_ERR_POS = 1
BMI08X_ERR_CODE_POS = 2

#    Mask definition for ACCEL_STATUS_REG register
BMI08X_ACCEL_STATUS_MASK = 0x80

#    Position definitions for ACCEL_STATUS_REG
BMI08X_ACCEL_STATUS_POS = 7

#    Mask definitions for odr, bandwidth and range
BMI08X_ACCEL_ODR_MASK = 0x0F
BMI08X_ACCEL_BW_MASK = 0xF0
BMI08X_ACCEL_RANGE_MASK = 0x03

#    Position definitions for odr, bandwidth and range
BMI08X_ACCEL_BW_POS = 4

#    Mask definitions for INT1_IO_CONF register
BMI08X_ACCEL_INT_EDGE_MASK = 0x01
BMI08X_ACCEL_INT_LVL_MASK = 0x02
BMI08X_ACCEL_INT_OD_MASK = 0x04
BMI08X_ACCEL_INT_IO_MASK = 0x08
BMI08X_ACCEL_INT_IN_MASK = 0x10

#    Position definitions for INT1_IO_CONF register
BMI08X_ACCEL_INT_EDGE_POS = 0
BMI08X_ACCEL_INT_LVL_POS = 1
BMI08X_ACCEL_INT_OD_POS = 2
BMI08X_ACCEL_INT_IO_POS = 3
BMI08X_ACCEL_INT_IN_POS = 4

#    Mask definitions for INT1/INT2 mapping register
BMI08X_ACCEL_MAP_INTA_MASK = 0x01

#    Mask definitions for INT1/INT2 mapping register
BMI08X_ACCEL_MAP_INTA_POS = 0x00

#    Mask definitions for INT1_INT2_MAP_DATA register
BMI08X_ACCEL_INT1_DRDY_MASK = 0x04
BMI08X_ACCEL_INT2_DRDY_MASK = 0x40

#    Position definitions for INT1_INT2_MAP_DATA register
BMI08X_ACCEL_INT1_DRDY_POS = 2
BMI08X_ACCEL_INT2_DRDY_POS = 6

#    Asic Initialization value
BMI08X_ASIC_INITIALIZED = 0x01

# ************************** BMI08 Gyroscope Macros ****************************
# * Register map
# Gyro registers

#    Gyro Chip Id register
BMI08X_REG_GYRO_CHIP_ID = 0x00


class Bmi08xGyroRegs(Enum):
    """ Used to store Gyro register addresses """
    #    Gyro X LSB data register
    BMI08X_REG_GYRO_X_LSB = 0x02

    #    Gyro X MSB data register
    BMI08X_REG_GYRO_X_MSB = 0x03

    #    Gyro Y LSB data register
    BMI08X_REG_GYRO_Y_LSB = 0x04

    #    Gyro Y MSB data register
    BMI08X_REG_GYRO_Y_MSB = 0x05

    #    Gyro Z LSB data register
    BMI08X_REG_GYRO_Z_LSB = 0x06

    #    Gyro Z MSB data register
    BMI08X_REG_GYRO_Z_MSB = 0x07

    #    Gyro Interrupt status register
    BMI08X_REG_GYRO_INT_STAT_1 = 0x0A

    #    Gyro FIFO status register
    BMI08X_REG_GYRO_FIFO_STATUS = 0x0E

    #    Gyro Range register
    BMI08X_REG_GYRO_RANGE = 0x0F

    #    Gyro Bandwidth register
    BMI08X_REG_GYRO_BANDWIDTH = 0x10

    #    Gyro Power register
    BMI08X_REG_GYRO_LPM1 = 0x11

    #    Gyro Soft reset register
    BMI08X_REG_GYRO_SOFTRESET = 0x14

    #    Gyro Interrupt control register
    BMI08X_REG_GYRO_INT_CTRL = 0x15

    #    Gyro Interrupt Pin configuration register
    BMI08X_REG_GYRO_INT3_INT4_IO_CONF = 0x16

    #    Gyro Interrupt Map register
    BMI08X_REG_GYRO_INT3_INT4_IO_MAP = 0x18

    #    Gyro FIFO watermark enable register
    BMI08X_REG_GYRO_FIFO_WM_ENABLE = 0x1E

    #    Gyro Self test register
    BMI08X_REG_GYRO_SELF_TEST = 0x3C

    #    Gyro Fifo Config 0 register
    BMI08X_REG_GYRO_FIFO_CONFIG0 = 0x3D

    #    Gyro Fifo Config 1 register
    BMI08X_REG_GYRO_FIFO_CONFIG1 = 0x3E

    #    Gyro Fifo Data register
    BMI08X_REG_GYRO_FIFO_DATA = 0x3F


#    Gyro unique chip identifier
BMI08X_GYRO_CHIP_ID = 0x0F

#    Gyro I2C slave address
BMI08X_GYRO_I2C_ADDR_PRIMARY = 0x68
BMI08X_GYRO_I2C_ADDR_SECONDARY = 0x69

#    Gyro Range
BMI08X_GYRO_RANGE_2000_DPS = 0x00
BMI08X_GYRO_RANGE_1000_DPS = 0x01
BMI08X_GYRO_RANGE_500_DPS = 0x02
BMI08X_GYRO_RANGE_250_DPS = 0x03
BMI08X_GYRO_RANGE_125_DPS = 0x04

#    Gyro Output data rate and bandwidth
BMI08X_GYRO_BW_532_ODR_2000_HZ = 0x00
BMI08X_GYRO_BW_230_ODR_2000_HZ = 0x01
BMI08X_GYRO_BW_116_ODR_1000_HZ = 0x02
BMI08X_GYRO_BW_47_ODR_400_HZ = 0x03
BMI08X_GYRO_BW_23_ODR_200_HZ = 0x04
BMI08X_GYRO_BW_12_ODR_100_HZ = 0x05
BMI08X_GYRO_BW_64_ODR_200_HZ = 0x06
BMI08X_GYRO_BW_32_ODR_100_HZ = 0x07
BMI08X_GYRO_ODR_RESET_VAL = 0x80

#    Gyro Power mode
BMI08X_GYRO_PM_NORMAL = 0x00
BMI08X_GYRO_PM_DEEP_SUSPEND = 0x20
BMI08X_GYRO_PM_SUSPEND = 0x80

#    Gyro data ready interrupt enable value
BMI08X_GYRO_DRDY_INT_DISABLE_VAL = 0x00
BMI08X_GYRO_DRDY_INT_ENABLE_VAL = 0x80
BMI08X_GYRO_FIFO_INT_DISABLE_VAL = 0x00
BMI08X_GYRO_FIFO_INT_ENABLE_VAL = 0x40
BMI08X_GYRO_FIFO_WM_ENABLE_VAL = 0x80
BMI08X_GYRO_FIFO_WM_DISABLE_VAL = 0x00

#    Gyro data ready map values
BMI08X_GYRO_MAP_DRDY_TO_INT3 = 0x01
BMI08X_GYRO_MAP_DRDY_TO_INT4 = 0x80
BMI08X_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4 = 0x81
BMI08X_GYRO_MAP_FIFO_INT3 = 0x04
BMI08X_GYRO_MAP_FIFO_INT4 = 0x20
BMI08X_GYRO_MAP_FIFO_BOTH_INT3_INT4 = 0x24

#    Gyro Soft reset delay
BMI08X_GYRO_SOFTRESET_DELAY = 30

#    Gyro power mode config delay
BMI08X_GYRO_POWER_MODE_CONFIG_DELAY = 30

# * Mask definitions for range, bandwidth and power
BMI08X_GYRO_RANGE_MASK = 0x07
BMI08X_GYRO_BW_MASK = 0x0F
BMI08X_GYRO_POWER_MASK = 0xA0

# * Position definitions for range, bandwidth and power
BMI08X_GYRO_POWER_POS = 5

#    Mask definitions for BMI08X_GYRO_INT_CTRL_REG register
BMI08X_GYRO_DATA_EN_MASK = 0x80

#    Position definitions for BMI08X_GYRO_INT_CTRL_REG register
BMI08X_GYRO_DATA_EN_POS = 7

#    Mask definitions for BMI08X_GYRO_INT3_INT4_IO_CONF_REG register
BMI08X_GYRO_INT3_LVL_MASK = 0x01
BMI08X_GYRO_INT3_OD_MASK = 0x02
BMI08X_GYRO_INT4_LVL_MASK = 0x04
BMI08X_GYRO_INT4_OD_MASK = 0x08

#    Position definitions for BMI08X_GYRO_INT3_INT4_IO_CONF_REG register
BMI08X_GYRO_INT3_OD_POS = 1
BMI08X_GYRO_INT4_LVL_POS = 2
BMI08X_GYRO_INT4_OD_POS = 3

#    Mask definitions for BMI08X_GYRO_INT_EN_REG register
BMI08X_GYRO_INT_EN_MASK = 0x80

#    Position definitions for BMI08X_GYRO_INT_EN_REG register
BMI08X_GYRO_INT_EN_POS = 7

#    Mask definitions for BMI088_GYRO_INT_MAP_REG register
BMI08X_GYRO_INT3_MAP_MASK = 0x01
BMI08X_GYRO_INT4_MAP_MASK = 0x80

#    Position definitions for BMI088_GYRO_INT_MAP_REG register
BMI08X_GYRO_INT3_MAP_POS = 0
BMI08X_GYRO_INT4_MAP_POS = 7

#    Mask definitions for BMI088_GYRO_INT_MAP_REG register
BMI088_GYRO_INT3_MAP_MASK = 0x01
BMI088_GYRO_INT4_MAP_MASK = 0x80

#    Position definitions for BMI088_GYRO_INT_MAP_REG register
BMI088_GYRO_INT3_MAP_POS = 0
BMI088_GYRO_INT4_MAP_POS = 7

#    Mask definitions for GYRO_SELF_TEST register
BMI08X_GYRO_SELF_TEST_EN_MASK = 0x01
BMI08X_GYRO_SELF_TEST_RDY_MASK = 0x02
BMI08X_GYRO_SELF_TEST_RESULT_MASK = 0x04
BMI08X_GYRO_SELF_TEST_FUNCTION_MASK = 0x08

#    Position definitions for GYRO_SELF_TEST register
BMI08X_GYRO_SELF_TEST_RDY_POS = 1
BMI08X_GYRO_SELF_TEST_RESULT_POS = 2
BMI08X_GYRO_SELF_TEST_FUNCTION_POS = 3

#    Gyro Fifo configurations
BMI08X_GYRO_FIFO_OVERRUN_MASK = 0x80
BMI08X_GYRO_FIFO_OVERRUN_POS = 0x07
BMI08X_GYRO_FIFO_MODE_MASK = 0xC0
BMI08X_GYRO_FIFO_MODE_POS = 0x06
BMI08X_GYRO_FIFO_TAG_MASK = 0x80
BMI08X_GYRO_FIFO_TAG_POS = 0x07
BMI08X_GYRO_FIFO_DATA_SELECT_MASK = 0x03
BMI08X_GYRO_FIFO_FRAME_COUNT_MASK = 0x7F
BMI08X_GYRO_FIFO_WM_LEVEL_MASK = 0x7F

#  Gyro Fifo interrupt map
BMI08X_GYRO_FIFO_INT3_MASK = 0x04
BMI08X_GYRO_FIFO_INT3_POS = 0x02
BMI08X_GYRO_FIFO_INT4_MASK = 0x20
BMI08X_GYRO_FIFO_INT4_POS = 0x05

#   Gyro FIFO definitions
BMI08X_GYRO_FIFO_TAG_ENABLED = 0x01
BMI08X_GYRO_FIFO_TAG_DISABLED = 0x00
BMI08X_GYRO_FIFO_MODE_BYPASS = 0x00
BMI08X_GYRO_FIFO_MODE = 0x01
BMI08X_GYRO_FIFO_MODE_STREAM = 0x02
BMI08X_GYRO_FIFO_XYZ_AXIS_ENABLED = 0x00
BMI08X_GYRO_FIFO_X_AXIS_ENABLED = 0x01
BMI08X_GYRO_FIFO_Y_AXIS_ENABLED = 0x02
BMI08X_GYRO_FIFO_Z_AXIS_ENABLED = 0x03
BMI08X_GYRO_FIFO_XYZ_AXIS_FRAME_SIZE = 0x06
BMI08X_GYRO_FIFO_SINGLE_AXIS_FRAME_SIZE = 0x02


# *************************** Common Macros for both Accel and Gyro *****************************
#    SPI read/write mask to configure address
BMI08X_SPI_RD_MASK = 0x80
BMI08X_SPI_WR_MASK = 0x7F


# API warning codes
BMI08X_W_SELF_TEST_FAIL = 1

# **\    Soft reset Value
BMI08X_SOFT_RESET_CMD = 0xB6

#    Enable/disable macros
BMI08X_DISABLE = 0
BMI08X_ENABLE = 1

#  To define warnings for FIFO activity
BMI08X_W_FIFO_EMPTY = 1
BMI08X_W_PARTIAL_READ = 2

#  Maximum length to read
BMI08X_MAX_LEN = 128

#  Sensortime resolution in seconds
BMI08X_SENSORTIME_RESOLUTION = 0.0000390625

#    Constant values macros
BMI08X_SENSOR_DATA_SYNC_TIME_MS = 1
BMI08X_DELAY_BETWEEN_WRITES_MS = 1
BMI08X_SELF_TEST_DELAY_MS = 3
BMI08X_POWER_CONFIG_DELAY = 5
BMI08X_SENSOR_SETTLE_TIME_MS = 30
BMI08X_SELF_TEST_DATA_READ_MS = 50
BMI08X_ASIC_INIT_TIME_MS = 150

BMI08X_CONFIG_STREAM_SIZE = 6144

#    Sensor time array parameter definitions
BMI08X_SENSOR_TIME_MSB_BYTE = 2
BMI08X_SENSOR_TIME_XLSB_BYTE = 1
BMI08X_SENSOR_TIME_LSB_BYTE = 0

#   int pin active state
BMI08X_INT_ACTIVE_LOW = 0
BMI08X_INT_ACTIVE_HIGH = 1

#   interrupt pin output definition
BMI08X_INT_MODE_PUSH_PULL = 0
BMI08X_INT_MODE_OPEN_DRAIN = 1

#    Sensor bit resolution
BMI08X_16_BIT_RESOLUTION = 16


# ********************************BMI08X FIFO Macros*********************************
# Register map
#  FIFO Header Mask definitions
BMI08X_FIFO_HEADER_ACC_FRM = 0x84
BMI08X_FIFO_HEADER_ALL_FRM = 0x9C
BMI08X_FIFO_HEADER_SENS_TIME_FRM = 0x44
BMI08X_FIFO_HEADER_SKIP_FRM = 0x40
BMI08X_FIFO_HEADER_INPUT_CFG_FRM = 0x48
BMI08X_FIFO_HEAD_OVER_READ_MSB = 0x80
BMI08X_FIFO_SAMPLE_DROP_FRM = 0x50

# Accel registers
BMI08X_FIFO_LENGTH_0_ADDR = 0x24
BMI08X_FIFO_LENGTH_1_ADDR = 0x25
BMI08X_FIFO_DATA_ADDR = 0x26
BMI08X_FIFO_DOWNS_ADDR = 0x45
BMI08X_FIFO_WTM_0_ADDR = 0x46
BMI08X_FIFO_WTM_1_ADDR = 0x47
BMI08X_FIFO_CONFIG_0_ADDR = 0x48
BMI08X_FIFO_CONFIG_1_ADDR = 0x49

# FIFO sensor data lengths
BMI08X_FIFO_ACCEL_LENGTH = 6
BMI08X_FIFO_WTM_LENGTH = 2
BMI08X_FIFO_LENGTH_MSB_BYTE = 1
BMI08X_FIFO_DATA_LENGTH = 2
BMI08X_FIFO_CONFIG_LENGTH = 2
BMI08X_SENSOR_TIME_LENGTH = 3
BMI08X_FIFO_SKIP_FRM_LENGTH = 1
BMI08X_FIFO_INPUT_CFG_LENGTH = 1

# FIFO byte counter mask definition
BMI08X_FIFO_BYTE_COUNTER_MSB_MASK = 0x3F

# FIFO frame masks
BMI08X_FIFO_LSB_CONFIG_CHECK = 0x00
BMI08X_FIFO_MSB_CONFIG_CHECK = 0x80
BMI08X_FIFO_INTR_MASK = 0x5C

# FIFO config modes
BMI08X_ACC_STREAM_MODE = 0x00
BMI08X_ACC_FIFO_MODE = 0x01

# Mask definitions for FIFO configuration modes
BMI08X_ACC_FIFO_MODE_CONFIG_MASK = 0x01

# Mask definitions for FIFO_CONFIG_1 register
BMI08X_ACCEL_EN_MASK = 0x40
BMI08X_ACCEL_INT1_EN_MASK = 0x08
BMI08X_ACCEL_INT2_EN_MASK = 0x04

# Position definitions for FIFO_CONFIG_1 register
BMI08X_ACCEL_EN_POS = 6
BMI08X_ACCEL_INT1_EN_POS = 3
BMI08X_ACCEL_INT2_EN_POS = 2

# Position definitions for FIFO_DOWNS register
BMI08X_ACC_FIFO_DOWNS_MASK = 0xF0

# FIFO down sampling bit positions
BMI08X_ACC_FIFO_DOWNS_POS = 0x04

# FIFO down sampling user macros
BMI08X_ACC_FIFO_DOWN_SAMPLE_0 = 0
BMI08X_ACC_FIFO_DOWN_SAMPLE_1 = 1
BMI08X_ACC_FIFO_DOWN_SAMPLE_2 = 2
BMI08X_ACC_FIFO_DOWN_SAMPLE_3 = 3
BMI08X_ACC_FIFO_DOWN_SAMPLE_4 = 4
BMI08X_ACC_FIFO_DOWN_SAMPLE_5 = 5
BMI08X_ACC_FIFO_DOWN_SAMPLE_6 = 6
BMI08X_ACC_FIFO_DOWN_SAMPLE_7 = 7

# Mask definitions for INT1_INT2_MAP_DATA register
BMI08X_ACCEL_INT2_FWM_MASK = 0x20
BMI08X_ACCEL_INT2_FFULL_MASK = 0x10
BMI08X_ACCEL_INT1_FWM_MASK = 0x02
BMI08X_ACCEL_INT1_FFULL_MASK = 0x01

# Positions definitions for INT1_INT2_MAP_DATA register
BMI08X_ACCEL_INT1_FWM_POS = 1
BMI08X_ACCEL_INT2_FFULL_POS = 4
BMI08X_ACCEL_INT2_FWM_POS = 5

# *************************** Data structures *****************************

class ApiErrorCodes(Enum):
    """ Enum to represent API error code """
    # API success code
    BMI08X_OK = 0
    # API error codes
    BMI08X_E_NULL_PTR = -1
    BMI08X_E_COM_FAIL = -2
    BMI08X_E_DEV_NOT_FOUND = -3
    BMI08X_E_OUT_OF_RANGE = -4
    BMI08X_E_INVALID_INPUT = -5
    BMI08X_E_CONFIG_STREAM_ERROR = -6
    BMI08X_E_RD_WR_LENGTH_INVALID = -7
    BMI08X_E_INVALID_CONFIG = -8
    BMI08X_E_FEATURE_NOT_SUPPORTED = -9


class SensorType(Enum):
    """ Used to define Sensor type"""
    ACCEL = 0
    GYRO = 1


class Bmi08xAccelIntChannel(Enum):
    """ Enum to select accelerometer Interrupt pins """
    # interrupt Channel 1 for accel sensor
    BMI08X_INT_CHANNEL_1 = 0
    # interrupt Channel 2 for accel sensor
    BMI08X_INT_CHANNEL_2 = 1


class Bmi08xGyroIntChannel(Enum):
    """ Enum to select gyroscope Interrupt pins """
    # interrupt Channel 3 for gyro sensor
    BMI08X_INT_CHANNEL_3 = 0
    # interrupt Channel 4 for gyro sensor
    BMI08X_INT_CHANNEL_4 = 1


class Bmi08xAccelIntTypes(Enum):
    """ Enum to select accelerometer interrupts """
    BMI08X_ACCEL_INT_DATA_RDY = 0
    # Accel data ready interrupt
    BMI08X_ACCEL_INT_SYNC_DATA_RDY = 1
    # Accel synchronized data ready interrupt
    BMI08X_ACCEL_SYNC_INPUT = 2
    # Accel FIFO watermark interrupt
    BMI08X_ACCEL_INT_FIFO_WM = 3
    # Accel FIFO full interrupt
    BMI08X_ACCEL_INT_FIFO_FULL = 4


class Bmi08xGyroIntTypes(Enum):
    """ Enum to select gyroscope interrupts """
    BMI08X_GYRO_INT_DATA_RDY = 0  # Gyro data ready interrupt
    BMI08X_GYRO_INT_FIFO_WM = 1   # Gyro FIFO watermark interrupt
    BMI08X_GYRO_INT_FIFO_FULL = 2  # Gyro FIFO full interrupt


# *************************** Unit conversion functions *****************************


def lsb_to_dps(val, dps, bit_width):
    """
    This function converts lsb to degree per second for 16 bit gyro at
    range 125, 250, 500, 1000 or 2000dps.

    :param val       : LSB from each axis.
    :param dps       : Degree per second.
    :param bit_width : Resolution for gyro.

    :return: Degree per second.
    """
    half_scale = (1 << bit_width) / 2
    return (dps / ((half_scale) + BMI08X_GYRO_RANGE_2000_DPS)) * (val)


def lsb_to_mps2(val, accel_range, bit_width):
    """
    This internal function converts lsb to meter per second squared for 16 bit accelerometer for
    range 2G, 4G, 8G or 16G.

    :param val       : LSB from each axis.
    :param accel_range   : Gravity range.
    :param bit_width : Resolution for accel.

    :return: Meter per second square
    """
    half_scale = (1 << bit_width) / 2
    return (GRAVITY_EARTH * val * accel_range) / half_scale


# *************************** Accel Interrupt functions *****************************

def bmi08a_set_int_pin_config(self, int_config: dict) -> ErrorCodes:
    """This API configures the pins which fire the interrupt signal
    when any accel interrupt occurs.

    Args:
        int_config (dict): interrupt configurations

    Returns:
        ErrorCodes (Enum): 0 -> Success; < 0 -> Fail
    """
    rslt = ApiErrorCodes.BMI08X_OK
    int_channel = int_config['int_channel']
    is_channel_valid = True
    int_types = Bmi08xAccelIntTypes
    int_channels = Bmi08xAccelIntChannel

    # update reg_addr based on channel inputs
    if int_channel == int_channels.BMI08X_INT_CHANNEL_1:
        reg_addr = Bmi08xAccelRegs.BMI08X_REG_ACCEL_INT1_IO_CONF.value
    elif int_channel == int_channels.BMI08X_INT_CHANNEL_2:
        reg_addr = Bmi08xAccelRegs.BMI08X_REG_ACCEL_INT2_IO_CONF.value
    else:
        is_channel_valid = False

    if is_channel_valid:
        # Read interrupt pin configuration register
        data = self.read(SensorType.ACCEL, reg_addr, 1)[0]

        data = hfunc.set_bits(data, BMI08X_ACCEL_INT_LVL_MASK,
                              int_config['int_pin_cfg']['lvl'],  BMI08X_ACCEL_INT_LVL_POS)
        data = hfunc.set_bits(data, BMI08X_ACCEL_INT_OD_MASK,
                              int_config['int_pin_cfg']['output_mode'], BMI08X_ACCEL_INT_OD_POS)

        if int_config['int_type'] == int_types.BMI08X_ACCEL_SYNC_INPUT:
            data = hfunc.set_bits_pos_0(
                data, BMI08X_ACCEL_INT_EDGE_MASK, BMI08X_ENABLE)
            data = hfunc.set_bits(data, BMI08X_ACCEL_INT_IN_MASK,
                                  int_config['int_pin_cfg']['enable_int_pin'], BMI08X_ACCEL_INT_IN_POS)
            data = hfunc.set_bit_val_0(data, BMI08X_ACCEL_INT_IN_MASK)

        else:
            data = hfunc.set_bits(data, BMI08X_ACCEL_INT_IO_MASK,
                                  int_config['int_pin_cfg']['enable_int_pin'], BMI08X_ACCEL_INT_IO_POS)
            data = hfunc.set_bit_val_0(data, BMI08X_ACCEL_INT_IN_MASK)

        # Write to interrupt pin configuration register
        self.write(self.bus_instance, reg_addr, data, SensorType.ACCEL)

    else:
        rslt = ApiErrorCodes.BMI08X_E_INVALID_INPUT

    return rslt


def set_accel_data_ready_int(self, int_config:dict) -> ErrorCodes:
    """This API sets the data ready interrupt for accel sensor

    Args:
        int_config (dict): interrupt configurations

    Returns:
        ErrorCodes (Enum): 0 -> Success; < 0 -> Fail
    """
    rslt = ApiErrorCodes.BMI08X_OK
    data = 0
    conf = int_config['int_pin_cfg']['enable_int_pin']
    int_channels = Bmi08xAccelIntChannel

    # Read interrupt map register
    data = self.read(
        SensorType.ACCEL, Bmi08xAccelRegs.BMI08X_REG_ACCEL_INT1_INT2_MAP_DATA.value, 1)[0]
    int_channel = int_config['int_channel']
    if int_channel == int_channels.BMI08X_INT_CHANNEL_1:
        data = hfunc.set_bits(data, BMI08X_ACCEL_INT1_DRDY_MASK,
                              conf, BMI08X_ACCEL_INT1_DRDY_POS)
    elif int_channel == int_channels.BMI08X_INT_CHANNEL_2:
        data = hfunc.set_bits(data, BMI08X_ACCEL_INT2_DRDY_MASK,
                              conf, BMI08X_ACCEL_INT2_DRDY_POS)
    else:
        rslt = ApiErrorCodes.BMI08X_E_INVALID_INPUT

    # Configure interrupt pins
    rslt = bmi08a_set_int_pin_config(self, int_config)

    # Write to interrupt map register
    self.write(self.bus_instance,
               Bmi08xAccelRegs.BMI08X_REG_ACCEL_INT1_INT2_MAP_DATA.value, data, SensorType.ACCEL)

    return rslt


def bmi08a_set_int_config(self, int_config):
    """This API configures the necessary accel interrupt
    based on the user settings in the int_config.

    Args:
        int_config (dict): interrupt configurations

    Returns:
        ErrorCodes (Enum): 0 -> Success; < 0 -> Fail
    """
    rslt = ApiErrorCodes.BMI08X_OK
    int_type = int_config['int_type']
    int_types = Bmi08xAccelIntTypes
    if int_type == int_types.BMI08X_ACCEL_INT_DATA_RDY:
        # Data ready interrupt
        rslt = set_accel_data_ready_int(self, int_config)
    else:
        raise Exception(
            f"{int_type} Interrupt type not supported in the API!\n")

    return rslt

# *************************** Gyro Interrupt functions *****************************


def bmi08g_set_int_pin_config(self, int_config):
    """This API configures the pins which fire the
    interrupt signal when any gyro interrupt occurs.

    Args:
        int_config (dict): interrupt configurations

    Returns:
        ErrorCodes (Enum): 0 -> Success; < 0 -> Fail
    """
    int_channel = int_config['int_channel']
    int_channels = Bmi08xGyroIntChannel

    # Read interrupt configuration register
    data = self.read(SensorType.ACCEL,
                     Bmi08xGyroRegs.BMI08X_REG_GYRO_INT3_INT4_IO_CONF.value, 1)[0]

    #  Interrupt pin or channel 3
    if int_channel == int_channels.BMI08X_INT_CHANNEL_3:
        # Update data with user configured bmi08x_int_cfg structure
        data = hfunc.set_bits_pos_0(
            data, BMI08X_GYRO_INT3_LVL_MASK, int_config['int_pin_cfg']['lvl'])
        data = hfunc.set_bits(data, BMI08X_GYRO_INT3_OD_MASK,
                              int_config['int_pin_cfg']['output_mode'], BMI08X_GYRO_INT3_OD_POS)
    elif int_channel == int_channels.BMI08X_INT_CHANNEL_4:
        # Update data with user configured bmi08x_int_cfg structure
        data = hfunc.set_bits(data, BMI08X_GYRO_INT4_LVL_MASK,
                              int_config['int_pin_cfg']['lvl'], BMI08X_GYRO_INT4_LVL_POS)
        data = hfunc.set_bits(data, BMI08X_GYRO_INT4_OD_MASK,
                              int_config['int_pin_cfg']['output_mode'], BMI08X_GYRO_INT4_OD_POS)

    #  write to interrupt configuration register
    self.write(self.bus_instance,
               Bmi08xGyroRegs.BMI08X_REG_GYRO_INT3_INT4_IO_CONF.value, data, SensorType.GYRO)


def set_gyro_data_ready_int(self, int_config):
    """This API sets the data ready interrupt for gyro sensor.

    Args:
        int_config (dict): interrupt configurations

    Returns:
        ErrorCodes (Enum): 0 -> Success; < 0 -> Fail
    """
    rslt = ApiErrorCodes.BMI08X_OK
    data = [0, 0]
    conf = int_config['int_pin_cfg']['enable_int_pin']
    int_channels = Bmi08xGyroIntChannel

    # read interrupt map register
    data[0] = self.read(
        SensorType.GYRO, Bmi08xGyroRegs.BMI08X_REG_GYRO_INT3_INT4_IO_MAP.value, 1)[0]
    int_channel = int_config['int_channel']

    # Data to enable new data ready interrupt
    if int_channel == int_channels.BMI08X_INT_CHANNEL_3:
        data[0] = hfunc.set_bits_pos_0(
            data[0], BMI08X_GYRO_INT3_MAP_MASK, conf)
    elif int_channel == int_channels.BMI08X_INT_CHANNEL_4:
        data[0] = hfunc.set_bits(
            data[0], BMI08X_GYRO_INT3_MAP_MASK, conf, BMI08X_GYRO_INT3_MAP_POS)
    else:
        rslt = ApiErrorCodes.BMI08X_E_INVALID_INPUT

    # condition to check disabling the interrupt in single channel
    # when both interrupts channels are enable
    if data[0] & BMI08X_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4:
        data[1] = BMI08X_GYRO_DRDY_INT_ENABLE_VAL
    else:
        data[1] = BMI08X_GYRO_DRDY_INT_DISABLE_VAL

    # Write to interrupt map register
    self.write(self.bus_instance,
               Bmi08xGyroRegs.BMI08X_REG_GYRO_INT3_INT4_IO_MAP.value, data[0], SensorType.GYRO)

    # set input interrupt configuration
    bmi08g_set_int_pin_config(self, int_config)

    # Write data to interrupt control register
    self.write(self.bus_instance,
               Bmi08xGyroRegs.BMI08X_REG_GYRO_INT_CTRL.value, data[1], SensorType.GYRO)

    return rslt


def bmi08g_set_int_config(self, int_config):
    """This API configures the necessary gyro interrupt
    based on the user settings in the int_config.

    Args:
        int_config (dict): interrupt configurations

    Returns:
        ErrorCodes (Enum): 0 -> Success; < 0 -> Fail
    """
    rslt = ApiErrorCodes.BMI08X_OK
    int_type = int_config['int_type']
    int_types = Bmi08xGyroIntTypes

    if int_type == int_types.BMI08X_GYRO_INT_DATA_RDY:
        # Data ready interrupt
        rslt = set_gyro_data_ready_int(self, int_config)
    else:
        raise Exception(
            f"{int_type} Interrupt type not supported in the API!\n")

    return rslt


class BMI08X:
    """ Test api for the shuttle bmi085 with app30 board """

    SHUTTLE_ID = 0x46
    I2C_ADDRESS_ACCEL = 0x18
    I2C_ADDRESS_GYRO = 0x68
    SPI_CS_ACCEL = cpy.MultiIOPin.SHUTTLE_PIN_8
    SPI_CS_GYRO = cpy.MultiIOPin.SHUTTLE_PIN_14
    ACCEL_DATA_ADDR = Bmi08xAccelRegs.BMI08X_REG_ACCEL_X_LSB.value
    GYRO_DATA_ADDR = Bmi08xGyroRegs.BMI08X_REG_GYRO_X_LSB.value
    TEMP_DATA_ADDR = Bmi08xAccelRegs.BMI08X_REG_TEMP_MSB.value
    ACCEL_DATA_REG_LEN = 6
    GYRO_DATA_REG_LEN = 6
    TEMP_DATA_REG_LEN = 2


    def __init__(self, bus, interface):
        print(f"++++++++ {type(bus).__name__} ++++++++")
        self.interface = interface
        self.board = cpy.CoinesBoard()
        self.api_error_code = 0
        self.accel_cfg = dict(
            ODR=BMI08X_ACCEL_ODR_1600_HZ,
            RANGE=BMI085_ACCEL_RANGE_16G,
            BANDWIDTH=BMI08X_ACCEL_BW_NORMAL,
            POWER_MODE=BMI08X_ACCEL_PM_ACTIVE,
            POWER_ENABLE=BMI08X_ACCEL_POWER_ENABLE
        )
        self.gyro_cfg = dict(
            ODR=BMI08X_GYRO_BW_230_ODR_2000_HZ,
            RANGE=BMI08X_GYRO_RANGE_250_DPS,
            BANDWIDTH=BMI08X_GYRO_BW_230_ODR_2000_HZ,
            POWER_MODE=BMI08X_GYRO_PM_NORMAL
        )

        self.bst_get_board_details()

        if interface == cpy.SensorInterface.SPI:
            self.read_board = self.board.read_spi
            self.write_board = self.board.write_spi
            self.sensor_selection = [self.SPI_CS_ACCEL, self.SPI_CS_GYRO]
            if isinstance(bus, cpy.SPIBus):
                self.bus_instance = bus
            else:
                raise ValueError("Wrong bus instance")
        else:
            self.read_board = self.board.read_i2c
            self.write_board = self.board.write_i2c
            self.sensor_selection = [
                self.I2C_ADDRESS_ACCEL, self.I2C_ADDRESS_GYRO]
            if isinstance(bus, cpy.I2CBus):
                self.bus_instance = bus
            else:
                raise ValueError("Wrong bus instance")

    def init_board(self):
        """ Initialize board for Read and write """
        self.set_vdd_vddio(vdd=0, vddio=0)
        self.config_bus()
        self.set_vdd_vddio(vdd=3.3, vddio=3.3)
        time.sleep(0.1)

        if self.interface == cpy.SensorInterface.SPI:
            # Dummy-read for accelerometer to switch to spi mode
            self.read(SensorType.ACCEL, 0, 1)
        self.read_chip_id()

    def bst_get_board_details(self):
        """ Initial checking of the connected board """
        self.board.open_comm_interface(cpy.CommInterface.USB)
        self.verify_error(
            keyword="Open Communication interface", exit_flag=True)
        board_info = self.board.get_board_info()
        print(
            f'BoardInfo: HW/SW ID: {hex(board_info.HardwareId)}/{hex(board_info.SoftwareId)}')
        print(f"ShuttleID: {hex(board_info.ShuttleID)}")
        if board_info.HardwareId == 0:
            print('Seems like there is no board communication. Stop')
            self.board.close_comm_interface()
            sys.exit()
        if board_info.ShuttleID != self.SHUTTLE_ID:
            print('Seems like you are not using BMI085 shuttle board. Stop')
            self.board.close_comm_interface()
            sys.exit()

    def set_vdd_vddio(self, vdd, vddio):
        """ Sets the vdd and/or vddio """
        self.board.set_shuttleboard_vdd_vddio_config(
            vdd_val=vdd, vddio_val=vddio)
        self.verify_error(keyword="set vdd, vddio", exit_flag=True)

    def read_chip_id(self):
        """Reads the chip id"""
        chip_id = self.read(SensorType.ACCEL, 0, 1)
        print('Chip ID accel: ' + hex(chip_id[0]))
        chip_id = self.read(SensorType.GYRO, 0, 1)
        print('Chip ID gyro: ' + hex(chip_id[0]))

    def read_temperature(self):
        """
        Read temperature sensor data.
        Note: This may be surprisingly high
        due to self-heating of the ApplicationBoard2
        """
        data = self.read(SensorType.ACCEL, self.TEMP_DATA_ADDR, self.TEMP_DATA_REG_LEN)
        # Refer datasheet for temperature formula
        temp = hfunc.twos_comp(data[0] * 8 + (data[1] >> 5), 11) * 0.125 + 23
        print('\nTemperature: %.2f' % temp)

    def set_accel_power_mode(self):
        """ This API sets the power mode of the accel sensor """
        power_mode = self.accel_cfg['POWER_MODE']
        power_enable = self.accel_cfg['POWER_ENABLE']

        # write to register
        self.write(self.bus_instance, Bmi08xAccelRegs.BMI08X_REG_ACCEL_PWR_CONF.value,
                   power_mode, SensorType.ACCEL)
        self.board.delay_milli_sec(5)
        self.write(self.bus_instance, Bmi08xAccelRegs.BMI08X_REG_ACCEL_PWR_CTRL.value,
                   power_enable, SensorType.ACCEL)
        self.board.delay_milli_sec(5)

    def set_gyro_power_mode(self):
        """ This API sets the power mode of the gyro sensor """
        power_mode = self.gyro_cfg['POWER_MODE']

        # write to register
        self.write(self.bus_instance, Bmi08xGyroRegs.BMI08X_REG_GYRO_LPM1.value,
                   power_mode, SensorType.GYRO)
        self.board.delay_milli_sec(30)

    def config_bus(self):
        """ Decide to do the bus configuration"""
        if self.interface == cpy.SensorInterface.SPI:
            self.deconfig_spi()
            self.config_spi()
        else:
            self.deconfig_i2c()
            self.config_i2c()

    def config_spi(self):
        """ Configuration for SPI """

        self.board.set_pin_config(
            self.SPI_CS_ACCEL, cpy.PinDirection.OUTPUT, cpy.PinValue.HIGH)
        self.verify_error(keyword="configuring Pin", exit_flag=True)
        self.board.set_pin_config(
            self.SPI_CS_GYRO, cpy.PinDirection.OUTPUT, cpy.PinValue.HIGH)
        self.verify_error(keyword="configuring Pin", exit_flag=True)
        # Set PS pin of gyro to LOW for proper protocol selection
        self.board.set_pin_config(
            cpy.MultiIOPin.SHUTTLE_PIN_9, cpy.PinDirection.OUTPUT, cpy.PinValue.LOW)
        self.verify_error(keyword="configuring Pin", exit_flag=True)

        self.board.config_spi_bus(
            self.bus_instance, self.SPI_CS_ACCEL, cpy.SPISpeed.SPI_5_MHZ, cpy.SPIMode.MODE3)
        self.verify_error(keyword="configuring spi bus", exit_flag=True)

    def config_i2c(self):
        """ Configuration for I2C """
        #  CSB1 pin is made low for selecting I2C protocol*/
        self.board.set_pin_config(
            cpy.MultiIOPin.SHUTTLE_PIN_8, cpy.PinDirection.OUTPUT, cpy.PinValue.LOW)
        # SDO pin is made low for selecting I2C address 0x76*/
        self.board.set_pin_config(
            cpy.MultiIOPin.SHUTTLE_PIN_SDO, cpy.PinDirection.OUTPUT, cpy.PinValue.LOW)

        self.board.config_i2c_bus(
            self.bus_instance, self.I2C_ADDRESS_ACCEL, cpy.I2CMode.STANDARD_MODE)
        self.verify_error(keyword="configuring i2c bus", exit_flag=True)

        # Set PS pin of gyro to HIGH for proper protocol selection
        self.board.set_pin_config(
            cpy.MultiIOPin.SHUTTLE_PIN_9, cpy.PinDirection.OUTPUT, cpy.PinValue.HIGH)
        self.verify_error(keyword="configuring Pin", exit_flag=True)

    def deconfig_i2c(self):
        """ De-configuration I2C """
        self.board.deconfig_i2c_bus(self.bus_instance)
    
    def deconfig_spi(self):
        """ De-configuration SPI """
        self.board.deconfig_spi_bus(self.bus_instance)

    def set_accel_meas_conf(self):
        """ This API sets the output data rate, range and bandwidth of accel sensor."""
        odr = self.accel_cfg['ODR']
        bandwidth = self.accel_cfg['BANDWIDTH']
        accel_range = self.accel_cfg['RANGE']

        accel_conf_reg_data = self.read(
            SensorType.ACCEL, Bmi08xAccelRegs.BMI08X_REG_ACCEL_CONF.value, 1)[0]
        accel_range_reg_data = self.read(
            SensorType.ACCEL, Bmi08xAccelRegs.BMI08X_REG_ACCEL_RANGE.value, 1)[0]
        data = [accel_conf_reg_data, accel_range_reg_data]

        # Update data with new ODR and bw values
        data[0] = hfunc.set_bits_pos_0(data[0], BMI08X_ACCEL_ODR_MASK, odr)
        data[0] = hfunc.set_bits(
            data[0], BMI08X_ACCEL_BW_MASK, bandwidth, BMI08X_ACCEL_BW_POS)

        #  Update data with current range values
        data[1] = hfunc.set_bits_pos_0(data[1], BMI08X_ACCEL_RANGE_MASK, accel_range)

        # write to register
        self.write(self.bus_instance, Bmi08xAccelRegs.BMI08X_REG_ACCEL_CONF.value,
                   data[0], SensorType.ACCEL)
        self.write(self.bus_instance, Bmi08xAccelRegs.BMI08X_REG_ACCEL_RANGE.value,
                   data[1], SensorType.ACCEL)

        # check if written value is same as read value
        write_val = data
        [accel_conf, accel_range] = self.read(
            SensorType.ACCEL, Bmi08xAccelRegs.BMI08X_REG_ACCEL_CONF.value, 2)
        assert write_val == [accel_conf, accel_range]

        return accel_range

    def set_gyro_meas_conf(self):
        """ This API sets the output data rate, range and bandwidth of gyro sensor."""
        odr = self.gyro_cfg['ODR']
        gyro_range = self.gyro_cfg['RANGE']

        gyro_range_reg_data = self.read(
            SensorType.GYRO, Bmi08xGyroRegs.BMI08X_REG_GYRO_RANGE.value, 1)[0]
        gyro_bw_reg_data = self.read(
            SensorType.GYRO, Bmi08xGyroRegs.BMI08X_REG_GYRO_BANDWIDTH.value, 1)[0]
        data = [gyro_range_reg_data, gyro_bw_reg_data]

        #  Update data with current range values
        data[0] = hfunc.set_bits_pos_0(data[0], BMI08X_GYRO_RANGE_MASK, gyro_range)

        # Update data with new ODR value
        data[1] = hfunc.set_bits_pos_0(data[1], BMI08X_GYRO_BW_MASK, odr)

        # write to register
        self.write(self.bus_instance,
                   Bmi08xGyroRegs.BMI08X_REG_GYRO_RANGE.value, data[0], SensorType.GYRO)
        self.write(self.bus_instance,
                   Bmi08xGyroRegs.BMI08X_REG_GYRO_BANDWIDTH.value, data[1], SensorType.GYRO)

        # check if written value is same as read value
        write_val = data
        [gyro_range, gyro_bw] = self.read(
            SensorType.GYRO, Bmi08xGyroRegs.BMI08X_REG_GYRO_RANGE.value, 2)
        assert write_val == [gyro_range, gyro_bw]

        return gyro_range

    def read(self, sensor: SensorType, reg, num_of_bytes):
        """
        Read data,
        Consider the dummy byte in case of SPI mode for the accelerometer
        Remove the dummy byte in case of SPI mode for the accelerometer after reading
        """
        sensor_selection = self.sensor_selection[sensor.value]
        if sensor_selection == self.SPI_CS_ACCEL:
            num_of_bytes += 1

        data = self.read_board(self.bus_instance, reg &
                               0x7F, num_of_bytes, sensor_selection)
        self.verify_error(keyword="Device read", exit_flag=False)

        if sensor_selection == self.SPI_CS_ACCEL:
            data.pop(0)
        return data

    def write(self, bus, register_address, register_value,  sensor):
        """ Use to Write data """
        sensor_selection = self.sensor_selection[sensor.value]
        self.write_board(bus, register_address,
                         register_value, sensor_selection)
        self.verify_error(keyword="Device write", exit_flag=False)

    def verify_error(self, keyword=None, exit_flag=False):
        """ Display error code and Close the communication interface if error occurs """
        if self.board.error_code != ErrorCodes.COINES_SUCCESS:
            print(f"{keyword} failure: {self.board.error_code}")
            if exit_flag:
                self.board.close_comm_interface()
                sys.exit()
    def verify_api_error(self, keyword=None, exit_flag=False):
        """ Display API error code and Close the communication interface if error occurs """
        if self.api_error_code != ApiErrorCodes.BMI08X_OK:
            print(f"{keyword} failure: {self.api_error_code}")
            if exit_flag:
                self.board.close_comm_interface()
                sys.exit()

    def close_comm(self, verify_reset=False):
        """ Close the communication interface """
        self.set_vdd_vddio(vdd=0, vddio=0)
        self.board.delay_milli_sec(100)
        self.board.soft_reset()
        if verify_reset:
            self.verify_error("soft reset")
        self.board.delay_milli_sec(100)
        self.board.close_comm_interface()

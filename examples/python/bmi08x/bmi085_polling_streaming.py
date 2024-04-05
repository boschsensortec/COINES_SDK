#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This example works with Application Board 3.0 with BMI085 shuttle board
# pylint: disable=no-member, too-few-public-methods, protected-access, broad-except
"""
This is a simple example of BMI085 Polling streaming using coinespy
"""
from enum import Enum
import coinespy as cpy
import bmi08x_common as bmi08x
from bmi08x_common import BMI08X as BMI08X_CLS
import helper_functions as hfunc


class BMI085(BMI08X_CLS):
    """ Child class of BMI08X_CLS with methods for polling streaming """
    def __init__(self, **kwargs):
        BMI08X_CLS.__init__(self, kwargs["bus"], kwargs["interface"])
        self.accel_stream_settings = dict(
            SAMPLING_TIME=625,  # 1.6KHZ
            I2C_ADDR_PRIMARY=0x18,
            REG_X_LSB=0x12,
            CHANNEL_ID=1,
            CS_PIN=cpy.MultiIOPin.SHUTTLE_PIN_8.value,
            NO_OF_DATA_BYTES = 6
        )
        self.gyro_stream_settings = dict(
            SAMPLING_TIME=500,  # 2KHZ
            I2C_ADDR_PRIMARY=0x68,
            REG_X_LSB=0x02,
            CHANNEL_ID=2,
            CS_PIN=cpy.MultiIOPin.SHUTTLE_PIN_14.value,
            NO_OF_DATA_BYTES = 6
        )
        # ACCEL Range set to 16G
        self.accel_full_range = 16
        #  GYRO Range set to 250 dps
        self.gyro_full_range = 250
        self.accel_stream_config, self.accel_data_blocks = self.set_stream_settings(
            self.accel_stream_settings)
        self.gyro_stream_config, self.gyro_data_blocks = self.set_stream_settings(
            self.gyro_stream_settings)
        self.number_of_samples = 10

    def set_stream_settings(self, sensor: dict):
        """ API to configure Stream and Data blocks """
        stream_config = cpy.StreamingConfig()
        data_blocks = cpy.StreamingBlocks()
        if self.interface == cpy.SensorInterface.I2C:
            stream_config.Intf = cpy.SensorInterface.I2C.value
            stream_config.I2CBus = cpy.I2CBus.BUS_I2C_0.value
            stream_config.DevAddr = sensor["I2C_ADDR_PRIMARY"]
            data_blocks.NoOfDataBytes[0] = sensor["NO_OF_DATA_BYTES"]

        elif self.interface == cpy.SensorInterface.SPI:
            stream_config.Intf = cpy.SensorInterface.SPI.value
            stream_config.SPIBus = cpy.SPIBus.BUS_SPI_0.value
            stream_config.CSPin = sensor["CS_PIN"]
             # extra dummy byte for SPI
            data_blocks.NoOfDataBytes[0] = sensor["NO_OF_DATA_BYTES"] + 1

        stream_config.SamplingUnits = cpy.SamplingUnits.SAMPLING_TIME_IN_MICRO_SEC.value
        stream_config.SamplingTime = sensor["SAMPLING_TIME"]
        data_blocks.NoOfBlocks = 1
        data_blocks.RegStartAddr[0] = sensor["REG_X_LSB"]
        return (stream_config, data_blocks)

    def send_stream_settings(self, sensor: dict, sensor_type: Enum):
        """ API to send streaming settings to board based on Sensor type"""
        if sensor_type == bmi08x.SensorType.ACCEL:
            ret = self.board.config_streaming(
                sensor["CHANNEL_ID"], self.accel_stream_config, self.accel_data_blocks)
        else:
            ret = self.board.config_streaming(
                sensor["CHANNEL_ID"], self.gyro_stream_config, self.gyro_data_blocks)
        return ret

    def print_accel_gyro_data(self, sensor_type, sensor_data):
        """ To Display Accel and Gyro data after unit conversion"""
        if sensor_data:
            stream_buffer, valid_sample_count = sensor_data
            data = stream_buffer
            if data:
                print(f'\n{"ACCEL" if sensor_type == bmi08x.SensorType.ACCEL else "GYRO"} DATA \n')
                buffer_index = 0
                for idx in range(0, valid_sample_count):
                    if (idx + 5) < len(data):
                        if self.interface == cpy.SensorInterface.SPI and sensor_type == bmi08x.SensorType.ACCEL:
                            # dummy byte
                            buffer_index += 1

                        # Next 6 bytes contain sensor info
                        x_data = hfunc.twos_comp(
                            (data[1 + buffer_index] << 8) | data[0 + buffer_index], 16)
                        y_data = hfunc.twos_comp(
                            (data[3 + buffer_index] << 8) | data[2 + buffer_index], 16)
                        z_data = hfunc.twos_comp(
                            (data[5 + buffer_index] << 8) | data[4 + buffer_index], 16)
                        buffer_index += 6

                        if sensor_type == bmi08x.SensorType.ACCEL:
                            unit_converted_x_data = bmi08x.lsb_to_mps2(
                                x_data, self.accel_full_range, 16)
                            unit_converted_y_data = bmi08x.lsb_to_mps2(
                                y_data, self.accel_full_range, 16)
                            unit_converted_z_data = bmi08x.lsb_to_mps2(
                                z_data, self.accel_full_range, 16)

                        elif sensor_type == bmi08x.SensorType.GYRO:
                            unit_converted_x_data = bmi08x.lsb_to_dps(x_data, self.gyro_full_range, 16)
                            unit_converted_y_data = bmi08x.lsb_to_dps(y_data, self.gyro_full_range, 16)
                            unit_converted_z_data = bmi08x.lsb_to_dps(z_data, self.gyro_full_range, 16)

                        if sensor_type == bmi08x.SensorType.ACCEL:
                            print(f"Accel[{idx}] Acc_ms2_X : {unit_converted_x_data:+.3f}"
                                  f"\tAcc_ms2_Y : {unit_converted_y_data:+.3f}"
                                  f"\tAcc_ms2_Z : {unit_converted_z_data:+.3f}")
                        elif sensor_type == bmi08x.SensorType.GYRO:
                            print(f"Gyro[{idx}] Gyr_DPS_X : {unit_converted_x_data:+3.2f}"
                                  f"\tGyr_DPS_Y : {unit_converted_y_data:+3.2f}"
                                  f"\tGyr_DPS_Z : {unit_converted_z_data:+3.2f}")      

    def polling_streaming(self):
        """ API to execute polling streaming sequence"""
        # Send streaming settings
        self.board.error_code = self.send_stream_settings(self.accel_stream_settings, bmi08x.SensorType.ACCEL)
        self.verify_error("Accel Stream settings")
        self.board.error_code = self.send_stream_settings(self.gyro_stream_settings, bmi08x.SensorType.GYRO)
        self.verify_error("Gyro Stream settings")

        # Start polling streaming
        self.board.error_code = self.board.start_stop_streaming(
            cpy.StreamingMode.STREAMING_MODE_POLLING.value, cpy.StreamingState.STREAMING_START.value)
        self.verify_error("Start Polling streaming")

        # Read sensor data via polling streaming
        (self.board.error_code, accel_stream_buffer, accel_valid_sample_count) = self.board.read_stream_sensor_data(
            self.accel_stream_settings["CHANNEL_ID"], self.number_of_samples)
        self.verify_error("Read Accel stream data")
        (self.board.error_code, gyro_stream_buffer, gyro_valid_sample_count) = self.board.read_stream_sensor_data(
            self.gyro_stream_settings["CHANNEL_ID"], self.number_of_samples)
        self.verify_error("Read Gyro stream data")

        # Stop polling streaming
        self.board.error_code = self.board.start_stop_streaming(
            cpy.StreamingMode.STREAMING_MODE_POLLING.value, cpy.StreamingState.STREAMING_STOP.value)
        self.verify_error("Stop Poling streaming")

        # print the streamed accel & gyro data
        accel_data = [accel_stream_buffer,
                      accel_valid_sample_count]
        gyro_data = [gyro_stream_buffer, gyro_valid_sample_count]
        self.print_accel_gyro_data(bmi08x.SensorType.ACCEL, accel_data)
        self.print_accel_gyro_data(bmi08x.SensorType.GYRO, gyro_data)

        # flush the buffer
        self.board.flush_interface()

        # Wait for 100 ms
        self.board.delay_milli_sec(100)


if __name__ == "__main__":
    bmi085 = BMI085(bus=cpy.I2CBus.BUS_I2C_0,
                    interface=cpy.SensorInterface.I2C)
    # bmi085 = BMI085(bus=cpy.SPIBus.BUS_SPI_0,interface=cpy.SensorInterface.SPI)
    bmi085.init_board()

    # Change sensor config settings as shown below
    # bmi085.accel_stream_config.I2CBus = cpy.I2CBus.BUS_I2C_0.value
    # bmi085.number_of_samples = 30

    # Set Accel power, ODR and range settings
    bmi085.set_accel_power_mode()
    acc_range = bmi085.set_accel_meas_conf()

    # Set Gyro power, ODR and range settings
    bmi085.set_gyro_power_mode()
    gyr_range = bmi085.set_gyro_meas_conf()
    print(f"Accel range: {acc_range} Gyro range: {gyr_range}")

    try:
        bmi085.polling_streaming()
    except Exception as e:
        print(f"Exception:{e}")
        bmi085.close_comm()
    bmi085.close_comm()

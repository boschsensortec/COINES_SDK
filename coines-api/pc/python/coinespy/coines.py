#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
(c) Bosch Sensortec GmbH, Reutlingen, Germany
Open Source as per the BSD-3 Clause
"""
# pylint: disable=too-few-public-methods, attribute-defined-outside-init,  # NOSONAR
# pylint: disable=too-many-instance-attributes, too-many-arguments  # NOSONAR

from __future__ import print_function
from typing import List, Tuple
from enum import Enum
import ctypes as ct
import math
import os
import os.path
import platform
import sys

STREAM_RSP_BUF_SIZE = 1048576

""" COINES_SDK maximum interrupt line count """
COINES_MAX_INT_LINE = 2


class ErrorCodes(Enum):
    """ Used to define the error codes"""
    COINES_SUCCESS = 0
    COINES_E_FAILURE = -1
    COINES_E_COMM_IO_ERROR = -2
    COINES_E_COMM_INIT_FAILED = -3
    COINES_E_UNABLE_OPEN_DEVICE = -4
    COINES_E_DEVICE_NOT_FOUND = -5
    COINES_E_UNABLE_CLAIM_INTERFACE = -6
    COINES_E_MEMORY_ALLOCATION = -7
    COINES_E_NOT_SUPPORTED = -8
    COINES_E_NULL_PTR = -9
    COINES_E_COMM_WRONG_RESPONSE = -10
    COINES_E_SPI16BIT_NOT_CONFIGURED = -11
    COINES_E_SPI_INVALID_BUS_INTERFACE = -12
    COINES_E_SPI_CONFIG_EXIST = -13
    COINES_E_SPI_BUS_NOT_ENABLED = -14
    COINES_E_SPI_CONFIG_FAILED = -15
    COINES_E_I2C_INVALID_BUS_INTERFACE = -16
    COINES_E_I2C_BUS_NOT_ENABLED = -17
    COINES_E_I2C_CONFIG_FAILED = -18
    COINES_E_I2C_CONFIG_EXIST = -19
    COINES_E_TIMER_INIT_FAILED = -20
    COINES_E_TIMER_INVALID_INSTANCE = -21
    COINES_E_TIMER_CC_CHANNEL_NOT_AVAILABLE = -22
    COINES_E_EEPROM_RESET_FAILED = -23
    COINES_E_EEPROM_READ_FAILED = -24
    COINES_E_INIT_FAILED = -25
    COINES_E_STREAM_NOT_CONFIGURED = -26
    COINES_E_STREAM_INVALID_BLOCK_SIZE = -27
    COINES_E_STREAM_SENSOR_ALREADY_CONFIGURED = -28
    COINES_E_STREAM_CONFIG_MEMORY_FULL = -29
    COINES_E_INVALID_PAYLOAD_LEN = -30
    COINES_E_CHANNEL_ALLOCATION_FAILED = -31
    COINES_E_CHANNEL_DE_ALLOCATION_FAILED = -32
    COINES_E_CHANNEL_ASSIGN_FAILED = -33
    COINES_E_CHANNEL_ENABLE_FAILED = -34
    COINES_E_CHANNEL_DISABLE_FAILED = -35
    COINES_E_INVALID_PIN_NUMBER = -36
    COINES_E_MAX_SENSOR_COUNT_REACHED = -37
    COINES_E_EEPROM_WRITE_FAILED = -38
    COINES_E_INVALID_EEPROM_RW_LENGTH = -39
    COINES_E_INVALID_SCOM_CONFIG = -40
    COINES_E_INVALID_BLE_CONFIG = -41
    COINES_E_SCOM_PORT_IN_USE = -42
    COINES_E_UART_INIT_FAILED = -43
    COINES_E_UART_WRITE_FAILED = -44
    COINES_E_UART_INSTANCE_NOT_SUPPORT = -45
    COINES_E_BLE_ADAPTOR_NOT_FOUND = -46
    COINES_E_ADAPTER_BLUETOOTH_NOT_ENABLED = -47
    COINES_E_BLE_PERIPHERAL_NOT_FOUND = -48
    COINES_E_BLE_LIBRARY_NOT_LOADED = -49
    COINES_E_APP_BOARD_BLE_NOT_FOUND = -50
    COINES_E_BLE_COMM_FAILED = -51
    COINES_E_INCOMPATIBLE_FIRMWARE = -52
    COINES_E_UNDEFINED_CODE = -100


class PinDirection(Enum):
    """ Used to define the pin mode"""
    INPUT = 0  # COINES_PIN_DIRECTION_IN
    OUTPUT = 1


class PinValue(Enum):
    """ Used to define the pin level"""
    LOW = 0  # COINES_PIN_VALUE_LOW
    HIGH = 1


class CommInterface(Enum):
    """ Used to define the Communication interface as USB or Serial """
    USB = 0
    SERIAL = 1
    BLE = 2


class I2CMode(Enum):
    """ Used to define the speed of I2C bus """
    STANDARD_MODE = 0    # Standard mode - 100kHz
    FAST_MODE = 1        # Fast mode - 400kHz
    SPEED_3_4_MHZ = 2          # High Speed mode - 3.4 MHz
    SPEED_1_7_MHZ = 3         # High Speed mode 2 - 1.7 MHz


class SPISpeed(Enum):
    """ Used to define the speed of SPI bus"""
    SPI_10_MHZ = 6
    SPI_7_5_MHZ = 8
    SPI_6_MHZ = 10
    SPI_5_MHZ = 12
    SPI_3_75_MHZ = 16
    SPI_3_MHZ = 20
    SPI_2_5_MHZ = 24
    SPI_2_MHZ = 30
    SPI_1_5_MHZ = 40
    SPI_1_25_MHZ = 48
    SPI_1_2_MHZ = 50
    SPI_1_MHZ = 60
    SPI_750_KHZ = 80
    SPI_600_KHZ = 100
    SPI_500_KHZ = 120
    SPI_400_KHZ = 150
    SPI_300_KHZ = 200
    SPI_250_KHZ = 240


class SPITransferBits(Enum):
    """ Used to define the spi bits either 8 or 16 """
    SPI8BIT = 8     # 8 bit register read/write
    SPI16BIT = 16   # 16 bit register read/write


class SPIMode(Enum):
    """ Used to define the spi mode """
    MODE0 = 0x00   # SPI Mode 0: CPOL=0; CPHA=0
    MODE1 = 0x01   # SPI Mode 1: CPOL=0; CPHA=1
    MODE2 = 0x02   # SPI Mode 2: CPOL=1; CPHA=0
    MODE3 = 0x03   # SPI Mode 3: CPOL=1; CPHA=1


class MultiIOPin(Enum):
    """
    Used to define the shuttle board pin(s).
    Replication of coines_multi_io_pin from coines.h
    """
    SHUTTLE_PIN_7 = 0x09    # CS pin
    SHUTTLE_PIN_8 = 0x05    # Multi-IO 5
    SHUTTLE_PIN_9 = 0x00    # Multi-IO 0
    SHUTTLE_PIN_14 = 0x01   # Multi-IO 1
    SHUTTLE_PIN_15 = 0x02   # Multi-IO 2
    SHUTTLE_PIN_16 = 0x03   # Multi-IO 3
    SHUTTLE_PIN_19 = 0x08   # Multi-IO 8
    SHUTTLE_PIN_20 = 0x06   # Multi-IO 6
    SHUTTLE_PIN_21 = 0x07   # Multi-IO 7
    SHUTTLE_PIN_22 = 0x04   # Multi-IO 4
    SHUTTLE_PIN_SDO = 0x1F

    # APP3.0 pins
    MINI_SHUTTLE_PIN_1_4 = 0x10  # GPIO0
    MINI_SHUTTLE_PIN_1_5 = 0x11  # GPIO1
    MINI_SHUTTLE_PIN_1_6 = 0x12  # GPIO2/INT1
    MINI_SHUTTLE_PIN_1_7 = 0x13  # GPIO3/INT2
    MINI_SHUTTLE_PIN_2_5 = 0x14  # GPIO4
    MINI_SHUTTLE_PIN_2_6 = 0x15  # GPIO5
    MINI_SHUTTLE_PIN_2_1 = 0x16  # CS
    MINI_SHUTTLE_PIN_2_3 = 0x17  # SDO
    MINI_SHUTTLE_PIN_2_7 = 0x1D  # GPIO6
    MINI_SHUTTLE_PIN_2_8 = 0x1E  # GPIO7


class SensorInterface(Enum):
    """ To define Sensor interface """
    SPI = 0
    I2C = 1


class I2CBus(Enum):
    """ Used to define the I2C type """
    BUS_I2C_0 = 0
    BUS_I2C_1 = 1
    BUS_I2C_MAX = 2


class SPIBus(Enum):
    """ Used to define the SPI type """
    BUS_SPI_0 = 0
    BUS_SPI_1 = 1
    BUS_SPI_MAX = 2


class StreamingMode(Enum):
    """ Defines streaming modes """
    STREAMING_MODE_POLLING = 0    # Polling mode streaming
    STREAMING_MODE_INTERRUPT = 1  # Interrupt mode streaming


class StreamingState(Enum):
    """ Defines streaming states """
    STREAMING_START = 1
    STREAMING_STOP = 0


class PinInterruptMode(Enum):
    """ Defines Pin interrupt modes """
    # Trigger interrupt on pin state change
    PIN_INTERRUPT_CHANGE = 0
    # Trigger interrupt when pin changes from low to high
    PIN_INTERRUPT_RISING_EDGE = 1
    # Trigger interrupt when pin changes from high to low
    PIN_INTERRUPT_FALLING_EDGE = 2
    PIN_INTERRUPT_MODE_MAXIMUM = 4


class TimerConfig(Enum):
    """ Defines timer configuration """
    TIMER_STOP = 0  # TIMER Stop
    TIMER_START = 1  # TIMER Start
    TIMER_RESET = 2  # TIMER Reset


class TimerStampConfig(Enum):
    """ Defines timer stamp configuration"""
    TIMESTAMP_ENABLE = 0x03   # TIMESTAMP Enable
    TIMESTAMP_DISABLE = 0x04  # TIMESTAMP Disable


class SamplingUnits(Enum):
    """ Defines sampling unit"""
    SAMPLING_TIME_IN_MICRO_SEC = 0x01  # sampling unit in micro second
    SAMPLING_TIME_IN_MILLI_SEC = 0x02  # sampling unit in milli second


class PinConfigInfo(ct.Structure):
    """
    Assign the field values inside the corresponding
    streaming setting function
    """
    _fields_ = [('direction', ct.c_uint16),
                ('switch_state', ct.c_uint16),
                ('level', ct.c_uint16)]


class BoardInfo(ct.Structure):
    """ Used to represent the Board details """
    _fields_ = [  # variable for storing the hardware ID.
                  ('HardwareId', ct.c_uint16),
                  # variable for storing the software ID.
                  ('SoftwareId', ct.c_uint16),
                  # variable for storing the board information.
                  ('Board', ct.c_uint8),
                  # variable to store the shuttle ID.
                  ('ShuttleID', ct.c_uint16)]


class StreamingClearOnWriteConfig(ct.Structure):
    """ Variables to store the streaming clear on write settings """
    _fields_ = [('DummyByte', ct.c_uint8),
                ('StartAddress', ct.c_uint8),
                ('NumBytesToClear', ct.c_uint16),
                ('DataBuf', ct.c_uint8 * 255)]


class StreamingBlocks(ct.Structure):
    """ Variables to store the streaming address blocks """
    _fields_ = [('NoOfBlocks', ct.c_uint16),        # Number of blocks.
                # Register start address.
                ('RegStartAddr', ct.c_uint8 * 10),
                ('NoOfDataBytes', ct.c_uint16 * 10)]     # Number of data bytes.


class StreamingConfig(ct.Structure):
    """ Variables to store the streaming config settings """
    _fields_ = [('Intf', ct.c_int),               # Sensor Interface
                ('I2CBus', ct.c_int),             # I2C bus
                ('SPIBus', ct.c_int),             # SPI bus
                ('DevAddr', ct.c_uint8),          # I2C - Device address
                ('CSPin', ct.c_uint8),            # Chip select
                ('SamplingTime', ct.c_uint16),    # Sampling time
                # micro second / milli second - Sampling unit
                ('SamplingUnits', ct.c_int),
                ('IntPin', ct.c_int),             # Interrupt pin
                # 1- enable /0- disable time stamp for corresponding sensor
                ('IntTimeStamp', ct.c_uint8),
                ('SPIType', ct.c_uint8),
                ('ClearOnWrite', ct.c_uint8),
                ('HwPinState', ct.c_uint8),
                ('ClearOnWriteConfig', StreamingClearOnWriteConfig),
                ('IntlineCount', ct.c_uint8),
                ('IntlineInfo', ct.c_uint8 * COINES_MAX_INT_LINE)]


class CBleComConfig(ct.Structure):
    """ Variables to store ctype BLE com config settings """
    _fields_ = [("Address", ct.c_char * 200),
                ("Identifier", ct.c_char * 200),
                ]


class CSerialComConfig(ct.Structure):
    """ Variables to store ctype Serial com config settings """
    _fields_ = [
        ('BaudRate', ct.c_uint32),
        ('VendorId', ct.c_uint16),
        ('ProductId', ct.c_uint16),
        ('ComPortName', ct.c_char_p),
        ('RxBufferSize', ct.c_uint16),
    ]


class BleComConfig():
    """ Class to store BLE com config settings"""

    def __init__(self, address: str = None, identifier: str = None):
        self.address = address
        self.identifier = identifier


class SerialComConfig:
    """ Class to store Serial com config settings """

    def __init__(self, baud_rate=0, vendor_id=0x00, product_id=0x00, com_port_name: str = None, rx_buffer_size=0):
        self.baud_rate = baud_rate
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.com_port_name = com_port_name
        self.rx_buffer_size = rx_buffer_size


class CoinesBoard:
    """
    API to utilize the functions of COINES_SDK header file through .dll/.so loaded
    """
    @staticmethod
    def __my_os_arch():
        """ returns os architecture """
        os_arch = platform.architecture()[0]
        return os_arch

    @staticmethod
    def _load_coines_lib() -> ct.cdll.LoadLibrary:
        """ Loads the dll as per the os platform """
        path_list = []
        coinespy_path = os.path.dirname(__file__)
        if platform.system() == 'Windows':
            path_list.append('libcoines.dll')
            path_list.append(
                CoinesBoard.__coines_lib_for_windows(coinespy_path))
        elif platform.system() == 'Linux':
            path_list.append('libcoines.so')
            path_list.append(CoinesBoard.__coines_lib_for_linux(coinespy_path))
        else:  # macOS library loading
            path_list.append('libcoines.dylib')
            path_list.append(CoinesBoard.__coines_lib_for_mac(coinespy_path))

        return CoinesBoard.__load_status(path_list)

    @staticmethod
    def __coines_lib_for_windows(coinespy_path):
        if CoinesBoard.__my_os_arch() == '64bit':
            temp = os.path.join(coinespy_path, 'libcoines_64.dll')
        else:
            temp = os.path.join(coinespy_path, 'libcoines_32.dll')
        return temp

    @staticmethod
    def __coines_lib_for_linux(coinespy_path):
        if 'armv7l' in platform.uname():
            temp = os.path.join(coinespy_path, 'libcoines_armv7_32.so')
        elif CoinesBoard.__my_os_arch() == '64bit':
            temp = os.path.join(coinespy_path, 'libcoines_64.so')
        else:  # 32bit
            temp = os.path.join(coinespy_path, 'libcoines_32.so')
        return temp

    @staticmethod
    def __coines_lib_for_mac(coinespy_path):
        if platform.processor() == 'i386':
            if CoinesBoard.__my_os_arch() == '64bit':
                temp = os.path.join(coinespy_path, 'libcoines_i386_64.dylib')
            else:  # 32bit
                temp = os.path.join(coinespy_path, 'libcoines_i386_32.dylib')
        else:
            if CoinesBoard.__my_os_arch() == '64bit':
                temp = os.path.join(coinespy_path, 'libcoines_arm_64.dylib')
            else:  # 32bit
                temp = os.path.join(coinespy_path, 'libcoines_arm_32.dylib')

        return temp

    @staticmethod
    def __load_status(path_list):
        lib = None
        for path in path_list:
            try:
                lib = ct.cdll.LoadLibrary(path)
            except OSError:
                pass
            if lib is not None:
                break
        return lib

    @staticmethod
    def convert_to_signed_error_code(err_code: int):
        """
        Function to convert the error code to signed integer.
        As per the ErrorCodes class, the values mentioned from -36 to 0,
        so this function helps to convert the value to appropriate code
        Values other than -36 to 0 are considered as undefined error codes

        :param err_code: integer
        :return: ErrorCodes
        """
        err_values = [err.value for err in ErrorCodes]
        signed_int = ct.c_int16(err_code)
        if signed_int.value > 0:
            signed_int = ct.c_int8(err_code)
        if signed_int.value not in err_values:
            return ErrorCodes.COINES_E_UNDEFINED_CODE
        else:
            return ErrorCodes(signed_int.value)

    def __init__(self, path_to_coines_lib: str = None):
        """
        Creates the class attributes and loads the library file loaded or
        from the installation directory as per os platform and bits of the processor
        """
        self._lib = None
        self.__pc_interface = CommInterface.USB.value
        self.error_code = 0
        self.__current_vdd = 0
        self.__current_vddio = 0
        self.__sensor_interface_detail = -1
        self._sensor_interface = SensorInterface.I2C.value
        self.__spi_16bit_enable = False

        if path_to_coines_lib:
            try:
                self._lib = ct.cdll.LoadLibrary(path_to_coines_lib)
            except OSError:
                print(
                    f'Could not load coines lib shared library: {path_to_coines_lib}')
                sys.exit(-1)

        if self._lib is None:
            self._lib = self._load_coines_lib()

        if self._lib is None:
            print(r"""
            Could not load COINES_SDK lib shared library. Please initialize the
            UserApplicationBoard object with the path to the library, e.g.
            board = BST.UserApplicationBoard(r'libcoines.dll')
            """)
            sys.exit(-1)

        coines_get_version = self._lib.coines_get_version
        coines_get_version.restype = ct.c_char_p
        self.lib_version = coines_get_version().decode("utf-8")

    def wrap_function(self, func_name: str, restype, argtypes):
        """ wrapping the func_name function equivalent to COINES_SDK header file with same name

        :param func_name: name of the function to be wrapped
        :param restype: eg, ct.u_int8
        :param argtypes: list of arguments eg, [ct.c_uint8, ct.c_uint8, ct.c_uint8]
        """
        func = self._lib.__getattr__(func_name)
        func.restype = restype
        func.argtypes = argtypes
        return func

    def open_comm_interface(self, interface=CommInterface.USB, serial_com_config: SerialComConfig = None,
                            ble_com_config: BleComConfig = None) -> ErrorCodes:
        """
        Opens the communication interface

        :param interface: Use the class CommInterface
        :param serial_com_config: Used to set the serial com settings
        :param ble_com_config : Used to set the BLE com settings
        :return: ErrorCodes
        """
        self.__pc_interface = interface.value
        arg_ptr = None
        if interface is CommInterface.USB and serial_com_config:
            c_serial_com_config = CSerialComConfig()
            c_serial_com_config.BaudRate = serial_com_config.baud_rate
            c_serial_com_config.VendorId = serial_com_config.vendor_id
            c_serial_com_config.ProductId = serial_com_config.product_id
            c_serial_com_config.ComPortName = bytes(
                str(serial_com_config.com_port_name), 'utf-8')
            c_serial_com_config.RxBufferSize = serial_com_config.rx_buffer_size
            arg_ptr = ct.pointer(c_serial_com_config)
        elif interface is CommInterface.BLE and ble_com_config:
            c_ble_com_config = CBleComConfig()
            c_ble_com_config.Address = bytes(
                str(ble_com_config.address), 'utf-8')
            c_ble_com_config.Identifier = bytes(
                str(ble_com_config.identifier), 'utf-8')
            arg_ptr = ct.pointer(c_ble_com_config)
        ret = self._lib.coines_open_comm_intf(self.__pc_interface, arg_ptr)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def scan_ble_devices(self, scan_timeout_ms=0) -> Tuple[list, int]:
        """
        Scans for BLE devices and returns a list of found BLE peripherals' info
        like Address and Identifier

        :param: scan_timeout_ms: timeout for BLE scan function
        :return: A tuple containing list of BLE peripheral information and peripheral count
        """
        self._lib.coines_scan_ble_devices.argtypes = [
            ct.POINTER(CBleComConfig), ct.POINTER(ct.c_uint8), ct.c_size_t]
        self._lib.coines_scan_ble_devices.restype = ct.c_int16

        ble_com_config = (CBleComConfig * 100)()
        peripheral_count = ct.c_uint8(0)

        ret = self._lib.coines_scan_ble_devices(
            ble_com_config[0], ct.byref(peripheral_count), scan_timeout_ms)

        ble_com_config_list = []
        for i in range(peripheral_count.value):
            address = ble_com_config[i].Address.decode('utf-8').rstrip('\x00')
            identifier = ble_com_config[i].Identifier.decode(
                'utf-8').rstrip('\x00')
            ble_com_config_list.append(BleComConfig(address, identifier))

        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return (ble_com_config_list, peripheral_count.value)

    def close_comm_interface(self, arg=None) -> ErrorCodes:
        """Closes the communication interface

        :param arg:
        :return: ErrorCodes
        """
        ret = self._lib.coines_close_comm_intf(self.__pc_interface, arg)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def get_board_info(self):
        """ To Fetch the board information

        :return: BoardInfo"""
        brd_info = BoardInfo()
        ret = self._lib.coines_get_board_info(ct.pointer(brd_info))
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return brd_info

    def set_pin_config(self, pin_number: MultiIOPin, direction: PinDirection,
                       output_state: PinValue) -> ErrorCodes:
        """ Sets the pin configuration

        :param pin_number: use var from MultiIOPin
        :param direction: use PinDirection
        :param output_state: use PinValue
        :return: ErrorCodes
        """
        pin_conf_func = self.wrap_function('coines_set_pin_config', ct.c_uint8,
                                           [ct.c_uint8, ct.c_uint8, ct.c_uint8])
        ret = pin_conf_func(
            pin_number.value, direction.value, output_state.value)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def get_pin_config(self, pin_number: MultiIOPin):
        """ Gets and returns the pin configuration

        :param pin_number: use MultiIOPin
        :return : PinConfigInfo
        """
        pin_info = PinConfigInfo()
        pt_direction = ct.c_uint16()
        pt_level = ct.c_uint16()
        getpin_func = self.wrap_function('coines_get_pin_config', ct.c_uint8,
                                         [ct.c_int, ct.POINTER(ct.c_uint16), ct.POINTER(ct.c_uint16)])
        ret = getpin_func(pin_number.value, ct.byref(
            pt_direction), ct.byref(pt_level))
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        pin_info.level = pt_level
        pin_info.direction = pt_direction
        if pin_info.level == PinValue.LOW and pin_info.direction == PinDirection.INPUT:
            pin_info.switch_state = 0
        else:
            pin_info.switch_state = 1
        return pin_info

    def set_shuttleboard_vdd_vddio_config(self, vdd_val: float = None, vddio_val: float = None) -> ErrorCodes:
        """ Sets the vdd/vddio level

        :param vdd_val: vdd value
        :param vddio_val: vddio value
        :return: ErrorCodes
        """
        if vdd_val is not None:
            self.__current_vdd = ct.c_ushort(int(vdd_val * 1000))
        if vddio_val is not None:
            self.__current_vddio = ct.c_ushort(int(vddio_val * 1000))
        ret = self._lib.coines_set_shuttleboard_vdd_vddio_config(
            self.__current_vdd, self.__current_vddio)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def set_vdd(self, vdd_val: float = None) -> ErrorCodes:
        """ Sets the Vdd level by keeping the vddio level as previous value

        :param vdd_val: vdd value
        :return: ErrorCodes
        """
        if vdd_val is not None:
            self.__current_vdd = ct.c_ushort(int(vdd_val * 1000))
        ret = self._lib.coines_set_shuttleboard_vdd_vddio_config(
            self.__current_vdd, self.__current_vddio)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def set_vddio(self, vddio_val: float = None) -> ErrorCodes:
        """ Sets the Vddio level by keeping the vdd level as previous value

        :param vddio_val: vddio value
        :return: ErrorCodes
        """
        if vddio_val is not None:
            self.__current_vddio = ct.c_ushort(int(vddio_val * 1000))
        ret = self._lib.coines_set_shuttleboard_vdd_vddio_config(
            self.__current_vdd, self.__current_vddio)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def config_i2c_bus(self, bus: I2CBus, i2c_address: int, i2c_mode: I2CMode) -> ErrorCodes:
        """ Configures the I2C Bus

        :param bus: use I2CBus
        :param i2c_address:
        :param i2c_mode: use I2CMode
        :return: ErrorCodes
        """
        self.__sensor_interface_detail = i2c_address
        self._sensor_interface = SensorInterface.I2C.value
        self.__spi_16bit_enable = False
        ret = self._lib.coines_config_i2c_bus(
            bus.value, i2c_mode.value)  # Always use bus COINES_I2C_BUS_0,
        # more are not available on APP2.0
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def deconfig_i2c_bus(self, bus: I2CBus) -> ErrorCodes:
        """ Method to de-configure the I2C BUS

        :param bus: use I2CBus
        :return: ErrorCodes
        """
        ret = self._lib.coines_deconfig_i2c_bus(bus.value)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def config_spi_bus(self, bus: SPIBus, cs_pin: MultiIOPin,
                       spi_speed=SPISpeed.SPI_1_MHZ, spi_mode=SPIMode.MODE0) -> ErrorCodes:
        """ Configures the SPI Bus

        :param bus: use SPIBus
        :param cs_pin: use MultiIOPin
        :param spi_speed: use SPISpeed
        :param spi_mode: use SPIMode
        :return: ErrorCodes
        """
        self.__sensor_interface_detail = cs_pin
        self._sensor_interface = SensorInterface.SPI.value
        self.__spi_16bit_enable = False
        ret = self._lib.coines_config_spi_bus(
            bus.value, spi_speed.value, spi_mode.value)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def deconfig_spi_bus(self, bus: SPIBus) -> ErrorCodes:
        """ Method to de-configure the SPI BUS

        :param bus: use SPIBus
        :return: ErrorCodes
        """
        ret = self._lib.coines_deconfig_spi_bus(bus.value)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def config_word_spi_bus(self, bus: SPIBus, cs_pin: MultiIOPin, spi_speed=SPISpeed.SPI_1_MHZ,
                            spi_mode=SPIMode.MODE0, spi_bits=SPITransferBits.SPI16BIT) -> ErrorCodes:
        """ 16-bit spi configuration

        :param bus: use SPIBus
        :param cs_pin: use MultiIOPin
        :param spi_speed: use SPISpeed
        :param spi_mode: use SPIMode
        :param spi_bits: use SPITransferBits
        :return: ErrorCodes
        """
        self.__sensor_interface_detail = cs_pin
        self._sensor_interface = SensorInterface.SPI.value
        if spi_bits == SPITransferBits.SPI16BIT:
            self.__spi_16bit_enable = True
        else:
            self.__spi_16bit_enable = False
        ret = self._lib.coines_config_word_spi_bus(
            bus.value, spi_speed.value, spi_mode.value, spi_bits.value)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def custom_spi_config(self, bus: SPIBus, cs_pin: MultiIOPin,
                          spi_speed: SPISpeed, spi_mode=SPIMode.MODE0) -> ErrorCodes:
        """
        Configures the customized SPI Bus

        :param bus: use SPIBus
        :param cs_pin: use MultiIOPin
        :param spi_speed: use SPISpeed
        :param spi_mode: use SPIMode
        :return: ErrorCodes
        """
        self.__sensor_interface_detail = cs_pin
        custom_spi_speed = math.ceil(60 * (10 ** 6) / spi_speed.value)
        ret = self._lib.coines_config_spi_bus(
            bus.value, custom_spi_speed, spi_mode.value)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    # -------------------------------------------- Read ----------------------------------------------
    def read_i2c(self, bus: I2CBus, register_address: int,
                 number_of_reads=1, sensor_interface_detail: int = None):
        """ Reads from I2C bus and returns it

        :param bus: use I2CBus
        :param register_address:
        :param number_of_reads:
        :param sensor_interface_detail:
        :return : List of data
        """
        reg_data = ct.create_string_buffer(number_of_reads)
        interface = self.__interface(sensor_interface_detail)
        ret = self._lib.coines_read_i2c(bus.value, interface, int(register_address),
                                        ct.byref(reg_data), int(number_of_reads))
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        reg_data = [int.from_bytes(i, byteorder='big') for i in reg_data]
        return reg_data

    def read_16bit_spi(self, bus: SPIBus, register_address: int, number_of_reads=2,
                       sensor_interface_detail: int = None):
        """ Reads 16 bits from SPI bus and returns it

        :param bus: use SPIBus
        :param register_address:
        :param number_of_reads:
        :param sensor_interface_detail:
        :return : Array of data
        """
        interface = self.__interface(sensor_interface_detail)
        array_type = (ct.c_int16 * number_of_reads)()
        self._lib.coines_read_16bit_spi.argtypes = [
            ct.c_char, ct.c_int16, ct.POINTER(ct.c_int16), ct.c_int16]
        if self._sensor_interface == SensorInterface.SPI.value:  # SPI bus configured
            ret = self._lib.coines_read_16bit_spi(bus.value, interface, int(register_address),
                                                  ct.cast(array_type, ct.POINTER(ct.c_int16)), int(number_of_reads))
        else:
            raise RuntimeError('No bus configured')

        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)

        return array_type

    def read_spi(self, bus: SPIBus, register_address: int,
                 number_of_reads=1, sensor_interface_detail: int = None):
        """ Reads from SPI bus and returns it

        :param bus: use SPIBus
        :param register_address:
        :param number_of_reads:
        :param sensor_interface_detail:
        :return : List of data
        """
        reg_data = ct.create_string_buffer(number_of_reads)
        interface = self.__interface(sensor_interface_detail)
        ret = self._lib.coines_read_spi(bus.value, interface, int(register_address | 0x80),
                                        ct.byref(reg_data), int(number_of_reads))
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        reg_data = [int.from_bytes(i, byteorder='big') for i in reg_data]
        return reg_data

    # -------------------------------------- Write -------------------------------------------------
    # Backward compatibility with GenericAPI: allow registerValue to be
    # either a list or an integer. However, the user should prefer to write one
    # byte at a time or consider a long delay after the write and check for
    # success by reading back.
    def write_i2c(self, bus: I2CBus, register_address: int,
                  register_value: int, sensor_interface_detail: int = None) -> ErrorCodes:
        """ Writes to I2C bus

        :param bus: use I2CBus
        :param register_address:
        :param register_value:
        :param sensor_interface_detail:
        :return: ErrorCodes
        """
        write_data, reg_length = self.__write_data_and_reg_length(
            register_value)
        interface = self.__interface(sensor_interface_detail)
        ret = self._lib.coines_write_i2c(
            bus.value, interface, register_address, write_data, reg_length)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def write_spi(self, bus: SPIBus, register_address: int,
                  register_value: int, sensor_interface_detail: int = None) -> ErrorCodes:
        """ Writes to SPI bus

        :param bus: use SPIBus
        :param register_address:
        :param register_value:
        :param sensor_interface_detail:
        :return: ErrorCodes
        """
        write_data, reg_length = self.__write_data_and_reg_length(
            register_value)
        interface = self.__interface(sensor_interface_detail)
        ret = self._lib.coines_write_spi(
            bus.value, interface, register_address, write_data, reg_length)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def write_16bit_spi(self, bus: SPIBus, register_address: int,
                        register_value: List[int], sensor_interface_detail: int = None) -> ErrorCodes:
        """ Writes 16 bits to SPI bus

        :param bus: use SPIBus
        :param register_address:
        :param register_value:
        :param sensor_interface_detail:
        :return: ErrorCodes
        """
        write_data, reg_length = self.__write_data_and_reg_length(
            register_value)
        interface = self.__interface(sensor_interface_detail)
        ret = self._lib.coines_write_16bit_spi(bus.value, interface, register_address,
                                               ct.cast(write_data, ct.POINTER(ct.c_int16)), reg_length)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code

    def soft_reset(self):
        """ Reset the board """
        self._lib.coines_soft_reset()

    def delay_milli_sec(self, time_in_milli_sec=100):
        """ Method to make delay with requested time in milli-seconds

        :param time_in_milli_sec: integer
        """
        self._lib.coines_delay_msec(time_in_milli_sec)

    def delay_micro_sec(self, time_in_micro_sec=1):
        """ Method to make delay with requested time in micro-seconds

        :param time_in_micro_sec: integer
        """
        self._lib.coines_delay_usec(time_in_micro_sec)

    def flush_interface(self):
        """ Method to perform flush. Recommended at end of the reading or testcase """
        self._lib.coines_flush_intf(self.__pc_interface)

    def echo_test(self, data: List[int]) -> ErrorCodes:
        """ Method to test the communication"""
        write_data, length = self.__write_data_and_reg_length(data)
        ret = self._lib.coines_echo_test(write_data, length)
        self.error_code = CoinesBoard.convert_to_signed_error_code(ret)
        return self.error_code
        # ------------------------------------------------------------------------------------------------

    # -------------------------------------------- Streaming ----------------------------------------------
    def config_streaming(self, sensor_id: int,
                         stream_config: StreamingConfig, data_blocks: StreamingBlocks) -> ErrorCodes:
        """ Sends the streaming settings to the board

        :param channel_id: channel identifier (Possible values - 1,2)
        :param stream_config:
        :param data_blocks:
        :return: ErrorCodes
        """
        ret = self._lib.coines_config_streaming(ct.c_uint8(
            sensor_id), ct.pointer(stream_config), ct.pointer(data_blocks))
        self.error_code = ErrorCodes(ct.c_int16(ret).value)
        return self.error_code

    def start_stop_streaming(self, stream_mode: StreamingMode, start_stop: StreamingState) -> ErrorCodes:
        """ Starts or stops the streaming

        :param stream_mode:
        :param start_stop:
        :return: ErrorCodes
        """
        ret = self._lib.coines_start_stop_streaming(
            stream_mode, ct.c_uint8(start_stop))
        self.error_code = ErrorCodes(ct.c_int16(ret).value)
        return self.error_code

    def read_stream_sensor_data(self, sensor_id: int, number_of_samples: int,
                                buffer_size=STREAM_RSP_BUF_SIZE) -> Tuple[ErrorCodes, list, int]:
        """ Reads the streaming sensor data

        :param sensor_id            :  Sensor Identifier.
        :param number_of_samples    :  Number of samples to be read.
        :param buffer_size          :  Buffer size of sensor data

        :data               :  Buffer to retrieve the sensor data
        :valid_samples_count:  Count of valid samples available.
        :return: tuple of error_code, data and valid_samples_count
        """
        data = (ct.c_uint8 * buffer_size)()
        valid_samples_count = ct.c_uint32(0)
        wrapper_params_types = [ct.c_uint8, ct.c_uint32,
                                ct.POINTER(ct.c_uint8), ct.POINTER(ct.c_uint32)]
        wrapper_ret_type = ct.c_int16
        coines_read_stream_sensor_data = self.wrap_function(
            'coines_read_stream_sensor_data', wrapper_ret_type, wrapper_params_types)

        sensor_id = ct.c_uint8(sensor_id)
        number_of_samples = ct.c_uint32(number_of_samples)
        ret = coines_read_stream_sensor_data(
            sensor_id, number_of_samples, data, valid_samples_count)
        self.error_code = ErrorCodes(ct.c_int16(ret).value)
        return (self.error_code, data, valid_samples_count.value)

    def trigger_timer(self, tmr_cfg: TimerConfig, ts_cfg: TimerStampConfig) -> ErrorCodes:
        """ Triggers the timer in firmware and enables or disables system time stamp

        :param tmr_cfg : timer config value
        :param ts_cfg : timer stamp cfg value
        :return: ErrorCodes
        """
        ret = self._lib.coines_trigger_timer(tmr_cfg, ts_cfg)
        self.error_code = ErrorCodes(ct.c_int16(ret).value)
        return self.error_code

        # ------------------------------------------------------------------------------------------------

    # --------------------------------- Internal support functions -----------------------------------
    def __write_data(self, reg_length):
        if self.__spi_16bit_enable is False:
            write_data = ct.create_string_buffer(reg_length)
        else:
            write_data = (ct.c_int16 * 1)()
        return write_data

    def __write_data_and_reg_length(self, register_value):
        write_data = []
        reg_length = 0
        if isinstance(register_value, int):
            reg_length = 1
            write_data = self.__write_data(reg_length)
            write_data[0] = register_value

        elif isinstance(register_value, list):
            reg_length = len(register_value)
            write_data = self.__write_data(reg_length)
            for i in range(reg_length):
                write_data[i] = register_value[i]

        return write_data, reg_length

    def __interface(self, sensor_interface_detail):
        if sensor_interface_detail is not None:
            interface = sensor_interface_detail
        else:
            interface = self.__sensor_interface_detail

        if isinstance(interface, MultiIOPin):
            interface = interface.value
        return interface

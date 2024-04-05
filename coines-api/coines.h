/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    coines.h
 * @brief   This file contains COINES_SDK layer function prototypes, variable declarations and Macro definitions
 *
 */

/*!
 * @addtogroup coines_api
 * @{*/

#ifndef COINES_H_
#define COINES_H_

#if !defined(COINES_VERSION)
#define COINES_VERSION  "v2.9.1"
#endif

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**********************************************************************************/
/* other includes */
/**********************************************************************************/
#include "coines_common.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! Streaming states */
#define COINES_STREAMING_START                     1
#define COINES_STREAMING_STOP                      0

/*! COINES_SDK USB buffer max size */
#define COINES_DATA_BUF_SIZE                       (1024)

/*! maximum no of sensor support */
#define COINES_MIN_SENSOR_ID                       1
#define COINES_MAX_SENSOR_ID                       2
#define COINES_MAX_SENSOR_COUNT                    (COINES_MAX_SENSOR_ID + 1)

/*! COINES_SDK stream response buffer size */
#define COINES_STREAM_RSP_BUF_SIZE                 2048

/*! COINES_SDK success code */
#define COINES_SUCCESS                             0

/*! COINES_SDK error code - failure */
#define COINES_E_FAILURE                           -1

/*! COINES_SDK error code - IO error */
#define COINES_E_COMM_IO_ERROR                     -2

/*! COINES_SDK error code - Init failure */
#define COINES_E_COMM_INIT_FAILED                  -3

/*! COINES_SDK error code - failure to open device */
#define COINES_E_UNABLE_OPEN_DEVICE                -4

/*! COINES_SDK error code - Device not found */
#define COINES_E_DEVICE_NOT_FOUND                  -5

/*! COINES_SDK error code - failure to claim interface */
#define COINES_E_UNABLE_CLAIM_INTF                 -6

/*! COINES_SDK error code - failure to allocate memory */
#define COINES_E_MEMORY_ALLOCATION                 -7

/*! COINES_SDK error code - Feature not supported */
#define COINES_E_NOT_SUPPORTED                     -8

/*! COINES_SDK error code - Null pointer */
#define COINES_E_NULL_PTR                          -9

/*! COINES_SDK error code - Wrong response */
#define COINES_E_COMM_WRONG_RESPONSE               -10

/*! COINES_SDK error code - Not configured */
#define COINES_E_SPI16BIT_NOT_CONFIGURED           -11

/*! COINES_SDK error code - SPI invalid bus interface */
#define COINES_E_SPI_INVALID_BUS_INTF              -12

/*! COINES_SDK error code - SPI instance configured already */
#define COINES_E_SPI_CONFIG_EXIST                  -13

/*! COINES_SDK error code - SPI bus not enabled */
#define COINES_E_SPI_BUS_NOT_ENABLED               -14

/*! COINES_SDK error code - SPI instance configuration failed */
#define COINES_E_SPI_CONFIG_FAILED                 -15

/*! COINES_SDK error code - I2C invalid bus interface */
#define COINES_E_I2C_INVALID_BUS_INTF              -16

/*! COINES_SDK error code - I2C bus not enabled */
#define COINES_E_I2C_BUS_NOT_ENABLED               -17

/*! COINES_SDK error code - I2C instance configuration failed */
#define COINES_E_I2C_CONFIG_FAILED                 -18

/*! COINES_SDK error code - I2C instance configured already */
#define COINES_E_I2C_CONFIG_EXIST                  -19

/*! COINES_SDK error code - Timer initialization failed */
#define COINES_E_TIMER_INIT_FAILED                 -20

/*! COINES_SDK error code - Invalid timer instance */
#define COINES_E_TIMER_INVALID_INSTANCE            -21

/*! COINES_SDK error code - Invalid timer instance */
#define COINES_E_TIMER_CC_CHANNEL_NOT_AVAILABLE    -22

/*! COINES_SDK error code - EEPROM reset failed */
#define COINES_E_EEPROM_RESET_FAILED               -23

/*! COINES_SDK error code - EEPROM read failed */
#define COINES_E_EEPROM_READ_FAILED                -24

/*! COINES_SDK error code - Initialization failed */
#define COINES_E_INIT_FAILED                       -25

/*! COINES_SDK error code - Streaming not configure */
#define COINES_E_STREAM_NOT_CONFIGURED             -26

/*! COINES_SDK error code - Streaming invalid block size */
#define COINES_E_STREAM_INVALID_BLOCK_SIZE         -27

/*! COINES_SDK error code - Streaming sensor already configured */
#define COINES_E_STREAM_SENSOR_ALREADY_CONFIGURED  -28

/*! COINES_SDK error code - Streaming sensor config memory full */
#define COINES_E_STREAM_CONFIG_MEMORY_FULL         -29

/*! COINES_SDK error code - Invalid payload length */
#define COINES_E_INVALID_PAYLOAD_LEN               -30

/*! COINES_SDK error code - channel allocation failed */
#define COINES_E_CHANNEL_ALLOCATION_FAILED         -31

/*! COINES_SDK error code - channel de-allocation failed */
#define COINES_E_CHANNEL_DEALLOCATION_FAILED       -32

/*! COINES_SDK error code - channel assignment failed */
#define COINES_E_CHANNEL_ASSIGN_FAILED             -33

/*! COINES_SDK error code - channel enable failed */
#define COINES_E_CHANNEL_ENABLE_FAILED             -34

/*! COINES_SDK error code - channel disable failed */
#define COINES_E_CHANNEL_DISABLE_FAILED            -35

/*! COINES_SDK error code - GPIO invalid pin number */
#define COINES_E_INVALID_PIN_NUMBER                -36

/*! COINES_SDK error code - GPIO invalid pin number */
#define COINES_E_MAX_SENSOR_COUNT_REACHED          -37

/*! COINES_SDK error code - EEPROM write failed */
#define COINES_E_EEPROM_WRITE_FAILED               -38

/*! COINES_SDK error code - Invalid EEPROM write length */
#define COINES_E_INVALID_EEPROM_RW_LENGTH          -39

/*! COINES_SDK error code - Invalid serial com config */
#define COINES_E_INVALID_SCOM_CONFIG               -40

/*! COINES_SDK error code - Invalid BLE config */
#define COINES_E_INVALID_BLE_CONFIG                -41

/*! COINES_SDK error code - Serial com port in use */
#define COINES_E_SCOM_PORT_IN_USE                  -42

/*! COINES_SDK error code - UART initialization failed  */
#define COINES_E_UART_INIT_FAILED                  -43

/*! COINES_SDK error code - UART write operation failed  */
#define COINES_E_UART_WRITE_FAILED                 -44

/*! COINES_SDK error code - UART instance check failed  */
#define COINES_E_UART_INSTANCE_NOT_SUPPORT         -45

/*! coines error code - BLE Adaptor not found  */
#define COINES_E_BLE_ADAPTOR_NOT_FOUND             -46

/*! coines error code - BLE not enabled  */
#define COINES_E_ADAPTER_BLUETOOTH_NOT_ENABLED     -47

/*! coines error code - BLE peripheral not found  */
#define COINES_E_BLE_PERIPHERAL_NOT_FOUND          -48

/*! coines error code - BLE library not loaded  */
#define COINES_E_BLE_LIBRARY_NOT_LOADED            -49

/*! coines error code - APP board BLE not found  */
#define COINES_E_APP_BOARD_BLE_NOT_FOUND           -50

/*! coines error code - BLE COMM failure  */
#define COINES_E_BLE_COMM_FAILED                   -51

/*! coines error code - incompatible firmware for the selected comm type */
#define COINES_E_INCOMPATIBLE_FIRMWARE             -52

/*! Variable to hold the number of error codes */
#define NUM_ERROR_CODES                             53

/**
 * EEPROM ID size in bytes
 */
#define EEPROM_ID_SIZE                             8

/*! EEPROM content size */
#define EEPROM_RW_CONTENT_SIZE                     112

/*! uart baud rate */
/*!< 1200 baud */
#define COINES_UART_BAUD_RATE_1200                 1200

/*!< 2400 baud */
#define COINES_UART_BAUD_RATE_2400                 2400

/*!< 4800 baud */
#define COINES_UART_BAUD_RATE_4800                 4800

/*!< 9600 baud */
#define COINES_UART_BAUD_RATE_9600                 9600

/*!< 14400 baud */
#define COINES_UART_BAUD_RATE_14400                14400

/*!< 19200 baud */
#define COINES_UART_BAUD_RATE_19200                19200

/*!< 28800 baud */
#define COINES_UART_BAUD_RATE_28800                28800

/*!< 31250 baud */
#define COINES_UART_BAUD_RATE_31250                31250

/*!< 38400 baud */
#define COINES_UART_BAUD_RATE_38400                38400

/*!< 56000 baud */
#define COINES_UART_BAUD_RATE_56000                56000

/*!< 57600 baud */
#define COINES_UART_BAUD_RATE_57600                57600

/*!< 76800 baud */
#define COINES_UART_BAUD_RATE_76800                76800

/*!< 115200 baud */
#define COINES_UART_BAUD_RATE_115200               115200

/*!< 230400 baud */
#define COINES_UART_BAUD_RATE_230400               230400

/*!< 250000 baud */
#define COINES_UART_BAUD_RATE_250000               250000

/*!< 460800 baud */
#define COINES_UART_BAUD_RATE_460800               460800

/*!< 921600 baud */
#define COINES_UART_BAUD_RATE_921600               921600

/*!< 1Mega baud */
#define COINES_UART_BAUD_RATE_1000000              1000000

/*! COINES_SDK maximum interrupt line count */
#define COINES_MAX_INT_LINE                        2

/*! COINES_SDK maximum blocks to read in streaming */
#define COINES_MAX_BLOCKS                          10

/*! COINES_SDK BLE char max len */
#define COINES_CHAR_MAX_LEN                        250

#if (defined(MCU_APP30) || defined(MCU_APP31))
#include <stdio.h>
extern FILE *bt_w, *bt_r;

/* Simplified content for missing 'dirent.h'
 * in arm-none-eabi-gcc toolchain
 */
struct dirent
{
    size_t d_namlen;
    char d_name[41];
};

typedef struct
{
    struct dirent dd_dir;
} DIR;

DIR *opendir(const char *dirname);
struct dirent *readdir(DIR *dirp);
int closedir(DIR *dirp);

#ifndef COINES_TDM_BUFFER_SIZE_WORDS
#define COINES_TDM_BUFFER_SIZE_WORDS  600
#endif

#endif

typedef void (*coines_tdm_callback)(uint32_t const *data);

typedef void (*coines_critical_callback)(void);

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/

/*!
 * @brief communication interface type
 */
enum coines_comm_intf {
    COINES_COMM_INTF_USB,
    /*< communication interface USB */
    COINES_COMM_INTF_VCOM,
    /*< communication interface VCOM */
    COINES_COMM_INTF_BLE
    /*< communication interface BLE */
};

/*!
 * @brief MULTIO Pin direction
 */
enum coines_pin_direction {
    COINES_PIN_DIRECTION_IN = 0,
    /*< PIN direction IN */
    COINES_PIN_DIRECTION_OUT /*< PIN direction OUT */
};

/*!
 * @brief MULTIO Pin value
 */
enum coines_pin_value {
    COINES_PIN_VALUE_LOW = 0,
    /*< PIN value LOW */
    COINES_PIN_VALUE_HIGH /*< PIN value HIGH */
};

enum coines_pin_polarity {
    COINES_PIN_INT_POLARITY_LOW_TO_HIGH = 1UL,
    /*< PIN interrupt trigger polarity when Low to High */
    COINES_PIN_INT_POLARITY_HIGH_TO_LOW = 2UL,
    /*< PIN interrupt trigger polarity when High to Low */
    COINES_PIN_INT_POLARITY_TOGGLE = 3UL /*< PIN interrupt trigger polarity toggle */
};

/*!
 * @brief I2C speed mode settings
 */
enum coines_i2c_mode {
    COINES_I2C_STANDARD_MODE = 0x00,
    /*< I2C speed in standard mode */
    COINES_I2C_FAST_MODE = 0x01,
    /*< I2C speed in fast mode */
    COINES_I2C_SPEED_3_4_MHZ = 0x02,
    /*< I2C speed in 3.4 MHz */
    COINES_I2C_SPEED_1_7_MHZ = 0x03 /*< I2C speed in 1.7 MHz */
};

/*!
 * @brief COINES_SDK sampling unit
 */
enum coines_sampling_unit {
    COINES_SAMPLING_TIME_IN_MICRO_SEC = 0x01,
    /*< sampling unit in micro second */
    COINES_SAMPLING_TIME_IN_MILLI_SEC = 0x02 /*< sampling unit in milli second */
};

/*!
 * @brief SPI mode settings
 *
 * > Note - don't change the values
 */
enum coines_spi_speed {
    COINES_SPI_SPEED_10_MHZ = 6,
    /*< 10 MHz */
    COINES_SPI_SPEED_7_5_MHZ = 8,
    /*< 7.5 MHz */
    COINES_SPI_SPEED_6_MHZ = 10,
    /*< 6 MHz */
    COINES_SPI_SPEED_5_MHZ = 12,
    /*< 5 MHz */
    COINES_SPI_SPEED_3_75_MHZ = 16,
    /*< 3.75 MHz */
    COINES_SPI_SPEED_3_MHZ = 20,
    /*< 3 MHz */
    COINES_SPI_SPEED_2_5_MHZ = 24,
    /*< 2.5 MHz */
    COINES_SPI_SPEED_2_MHZ = 30,
    /*< 2 MHz */
    COINES_SPI_SPEED_1_5_MHZ = 40,
    /*< 1.5 MHz */
    COINES_SPI_SPEED_1_25_MHZ = 48,
    /*< 1.25 MHz */
    COINES_SPI_SPEED_1_2_MHZ = 50,
    /*< 1.2 MHz */
    COINES_SPI_SPEED_1_MHZ = 60,
    /*< 1 MHz */
    COINES_SPI_SPEED_750_KHZ = 80,
    /*< 750 kHz */
    COINES_SPI_SPEED_600_KHZ = 100,
    /*< 600 kHz */
    COINES_SPI_SPEED_500_KHZ = 120,
    /*< 500 kHz */
    COINES_SPI_SPEED_400_KHZ = 150,
    /*< 400 kHz */
    COINES_SPI_SPEED_300_KHZ = 200,
    /*< 300 kHz */
    COINES_SPI_SPEED_250_KHZ = 240 /*< 250 kHz */
};

/*!
 * @brief ble transmit power
 *
 * > Note - don't change the values
 */
enum coines_tx_power {
    COINES_TX_POWER_MINUS_40_DBM = -40,
    /*< -40 dBm */
    COINES_TX_POWER_MINUS_20_DBM = -20,
    /*< -20 dBm */
    COINES_TX_POWER_MINUS_16_DBM = -16,
    /*< -16 dBm */
    COINES_TX_POWER_MINUS_12_DBM = -12,
    /*< -12 dBm */
    COINES_TX_POWER_MINUS_8_DBM = -8,
    /*< -8 dBm */
    COINES_TX_POWER_MINUS_4_DBM = -4,
    /*< -4 dBm */
    COINES_TX_POWER_0_DBM = 0,
    /*< 0 dBm */
    COINES_TX_POWER_2_DBM = 2,
    /*< 2 dBm */
    COINES_TX_POWER_3_DBM = 3,
    /*< 3 dBm */
    COINES_TX_POWER_4_DBM = 4,
    /*< 4 dBm */
    COINES_TX_POWER_5_DBM = 5,
    /*< 5 dBm */
    COINES_TX_POWER_6_DBM = 6,
    /*< 6 dBm */
    COINES_TX_POWER_7_DBM = 7,
    /*< 7 dBm */
    COINES_TX_POWER_8_DBM = 8,
    /*< 8 dBm */
};

/*!
 *  @brief user configurable Shuttle board pin description
 */
enum coines_multi_io_pin {
    COINES_SHUTTLE_PIN_7 = 0x09,
    /*<  CS pin*/
    COINES_SHUTTLE_PIN_8 = 0x05,
    /*<  Multi-IO 5*/
    COINES_SHUTTLE_PIN_9 = 0x00,
    /*<  Multi-IO 0*/
    COINES_SHUTTLE_PIN_14 = 0x01,
    /*<  Multi-IO 1*/
    COINES_SHUTTLE_PIN_15 = 0x02,
    /*<  Multi-IO 2*/
    COINES_SHUTTLE_PIN_16 = 0x03,
    /*<  Multi-IO 3*/
    COINES_SHUTTLE_PIN_19 = 0x08,
    /*<  Multi-IO 8*/
    COINES_SHUTTLE_PIN_20 = 0x06,
    /*<  Multi-IO 6*/
    COINES_SHUTTLE_PIN_21 = 0x07,
    /*<  Multi-IO 7*/
    COINES_SHUTTLE_PIN_22 = 0x04,
    /*<  Multi-IO 4*/

#if !defined(MCU_APP20)
    COINES_MINI_SHUTTLE_PIN_1_4 = 0x10,
    /*<  GPIO0 */
    COINES_MINI_SHUTTLE_PIN_1_5 = 0x11,
    /*<  GPIO1 */
    COINES_MINI_SHUTTLE_PIN_1_6 = 0x12,
    /*<  GPIO2/INT1 */
    COINES_MINI_SHUTTLE_PIN_1_7 = 0x13,
    /*<  GPIO3/INT2 */
    COINES_MINI_SHUTTLE_PIN_2_5 = 0x14,
    /*<  GPIO4 */
    COINES_MINI_SHUTTLE_PIN_2_6 = 0x15,
    /*<  GPIO5 */
    COINES_MINI_SHUTTLE_PIN_2_1 = 0x16,
    /*<  CS */
    COINES_MINI_SHUTTLE_PIN_2_3 = 0x17,
    /*<  SDO */
    COINES_MINI_SHUTTLE_PIN_2_7 = 0x1D,
    /*<  GPIO6 */
    COINES_MINI_SHUTTLE_PIN_2_8 = 0x1E,
    /*<  GPIO7 */
#endif
#if defined(MCU_APP30)
    COINES_APP30_LED_R = 0x18,
    /*<  red LED */
    COINES_APP30_LED_G = 0x19,
    /*<  green LED */
    COINES_APP30_LED_B = 0x1A,
    /*<  blue LED */
    COINES_APP30_BUTTON_1 = 0x1B,
    /*< button 1 */
    COINES_APP30_BUTTON_2 = 0x1C,
    /*< button 2 */
#endif
#if defined(MCU_NICLA)
    COINES_NICLA_BUTTON_RESET = 0x1B,
    /*< button 1 */
    COINES_NICLA_CD_PIN = 0x1C,
    /*CD line*/
#endif
    COINES_SHUTTLE_PIN_SDO = 0x1F,
#if defined(MCU_APP31)
    COINES_APP31_LED_R = 0x18,
    /*<  red LED */
    COINES_APP31_LED_G = 0x19,
    /*<  green LED */
    COINES_APP31_LED_B = 0x1A,
    /*<  blue LED */
    COINES_APP31_BUTTON_1 = 0x1B,
    /*< button 1 */
    COINES_APP31_BUTTON_2 = 0x1C,
    /*< button 2 */
    COINES_APP31_RESET_INT = 0x20,
    /*< Reset from PMIC */
    COINES_APP31_LSLDO = 0x21,
    /*< Load switch LDO pin */
    COINES_APP31_CD = 0x22,
    /*< PMIC chip disable pin */
    COINES_APP31_P_INT = 0x23,
    /*< Interrupt line from PMIC */
    COINES_APP31_VDDIO_EN = 0x24,
    /*< VDDIO_Sensor pin */
    COINES_APP31_VDD_EN = 0x25,
    /*< VDD_Sensor pin */
    COINES_APP31_LS_EN = 0x26,
    /*< Level Shifter pin*/
    COINES_APP31_VIN_DEC = 0x27,
    /*< VIN detection pin*/
    COINES_SHUTTLE_PIN_MAX = 0x28
#else
    COINES_SHUTTLE_PIN_MAX = 0x21
#endif
};

/*!
 * @brief SPI mode settings
 */
enum coines_spi_mode {
    COINES_SPI_MODE0 = 0x00,
    /*< SPI Mode 0: CPOL=0; CPHA=0 */
    COINES_SPI_MODE1 = 0x01,
    /*< SPI Mode 1: CPOL=0; CPHA=1 */
    COINES_SPI_MODE2 = 0x02,
    /*< SPI Mode 2: CPOL=1; CPHA=0 */
    COINES_SPI_MODE3 = 0x03 /*< SPI Mode 3: CPOL=1; CPHA=1 */
};

/*!
 * @brief SPI Number of bits per transfer
 */
enum coines_spi_transfer_bits {
    COINES_SPI_TRANSFER_8BIT = 8,
    /**< Transfer 8 bit */

    /* Before selecting the below 16 bit, ensure that,
     * the intended sensor supports 16bit register read/write
     */
    COINES_SPI_TRANSFER_16BIT = 16 /**< Transfer 16 bit */
};

/*!
 * @brief interface type
 */
enum coines_sensor_intf {
    COINES_SENSOR_INTF_SPI,
    /*< SPI Interface */
    COINES_SENSOR_INTF_I2C /*< I2C Interface */
};

/*!
 * @brief i2c bus
 */
enum coines_i2c_bus {
    COINES_I2C_BUS_0,
    /*< I2C bus 0 */
    COINES_I2C_BUS_1,
    /*< I2C bus 1 */
    COINES_I2C_BUS_MAX
};

/*!
 * @brief spi bus
 */
enum coines_spi_bus {
    COINES_SPI_BUS_0,
    /*< SPI bus 0 */
    COINES_SPI_BUS_1,
    /*< SPI bus 1 */
    COINES_SPI_BUS_MAX
};

/*!
 * @brief UART instance
 */
enum coines_uart_instance {
    COINES_UART_0,
    /*< UART instance 0 */
    COINES_UART_1
    /*< UART instance 1 */
};

/*!
 * @brief timer instance
 */
enum coines_timer_instance {
    COINES_TIMER_INSTANCE_0,
    /*< instance 0 */
    COINES_TIMER_INSTANCE_1,
#if defined(MCU_NICLA)

    /*< instance 1 */
    COINES_TIMER_INSTANCE_2,
#endif

    /*< instance 2 */
    COINES_TIMER_INSTANCE_MAX
};

/*!
 * @brief timer configuration
 */
enum coines_timer_config {
    COINES_TIMER_STOP,
    /*< TIMER Stop */
    COINES_TIMER_START,
    /*< TIMER Start */
    COINES_TIMER_RESET /*< TIMER Reset */
};

/*!
 * @brief times stamp config
 */
enum coines_time_stamp_config {
    COINES_TIMESTAMP_ENABLE = 0x03,
    /*< TIMESTAMP Enable */
    COINES_TIMESTAMP_DISABLE = 0x04 /*< TIMESTAMP Disable */
};

/*!
 * @brief COINES_SDK streaming mode
 */
enum coines_streaming_mode {
    COINES_STREAMING_MODE_POLLING,
    /*< Polling mode streaming */
    COINES_STREAMING_MODE_INTERRUPT /*< Interrupt mode streaming */
};

/*!
 * @brief COINES_SDK led state
 */
enum coines_led_state {
    COINES_LED_STATE_ON = 0,
    /*< Led state ON*/
    COINES_LED_STATE_OFF /*< Led state OFF */
};

/*!
 * @brief COINES_SDK led
 */
enum coines_led {
    COINES_LED_RED,
    /*< Red Led */
    COINES_LED_GREEN,
    /*< Green Led */
    COINES_LED_BLUE /*< Blue Led */
};

/*!
 * @brief Structure to store the board related information
 */
struct coines_board_info
{
    uint16_t hardware_id; /*< Board hardware ID */
    uint16_t software_id; /*< Board software ID */
    uint8_t board; /*< Type of the board like APP2.0, Arduino Due */
    uint16_t shuttle_id; /*< Shuttle ID of the sensor connected */
    uint8_t eeprom_id[EEPROM_ID_SIZE]; /*< EEPROM ID of the sensor shuttle connected */
};

/*!
 * @brief streaming address block
 */
struct coines_streaming_blocks
{
    uint16_t no_of_blocks; /*< Number of blocks */
    uint8_t reg_start_addr[COINES_MAX_BLOCKS]; /*< Register start address */
    uint16_t no_of_data_bytes[COINES_MAX_BLOCKS]; /*< Number of data bytes */
};

/*!
 * @brief streaming clear on write settings
 */
struct coines_streaming_clear_on_write
{
    uint8_t dummy_byte; /*< dummy byte count */
    uint8_t startaddress; /*< starting address */
    uint16_t num_bytes_to_clear; /* < No. of bytes to clear */
    uint8_t data_buf[255]; /*< data chunks */
};

/*!
 * @brief streaming config settings
 */
struct coines_streaming_config
{
    enum coines_sensor_intf intf; /*< Sensor Interface */
    enum coines_i2c_bus i2c_bus; /*< I2C bus */
    enum coines_spi_bus spi_bus; /*< SPI bus */
    uint8_t dev_addr; /*< I2C -Device address */
    uint8_t cs_pin; /*< Chip select */
    uint16_t sampling_time; /*< Poll stream - Sampling time */
    enum coines_sampling_unit sampling_units; /*< Poll stream - micro/milli second */
    enum coines_multi_io_pin int_pin; /*< Int stream - Interrupt pin */
    uint8_t int_timestamp; /*< Int stream - (1- enable /0- disable) time stamp for corresponding sensor */
    uint8_t spi_type; /*< spi type */
    uint8_t clear_on_write; /*< clear on write */
    uint8_t hw_pin_state; /*< hardware pin state active low/high */
    struct coines_streaming_clear_on_write clear_on_write_config;
    uint8_t intline_count; /*< interrupt line count */
    uint8_t intline_info[COINES_MAX_INT_LINE]; /*< interrupt line number */
};

/*!
 * @brief Structure to store the ble configuration (name and power)
 */
struct coines_ble_config
{
    char * name; /*< ble device name */
    enum coines_tx_power tx_power; /*<  Radio transmit power in dBm. */
};

/*!
 * @brief Structure to store the serial com configuration
 */
struct coines_serial_com_config
{
    uint32_t baud_rate; /*< Baud rate */
    uint16_t vendor_id; /*< vendor Id */
    uint16_t product_id; /*< Product Id */
    char* com_port_name; /*< serial com port name */
    uint16_t rx_buffer_size; /*< RX response buffer size */
};

/*!
 * @brief Structure to store the ble com configuration
 */
struct ble_peripheral_info
{
    char ble_address[COINES_CHAR_MAX_LEN]; /*< BLE device address */
    char ble_identifier[COINES_CHAR_MAX_LEN]; /*< BLE device identifier */
};

/*!
 * @brief Pin interrupt modes
 */
enum coines_pin_interrupt_mode {
    COINES_PIN_INTERRUPT_CHANGE,
    /*< Trigger interrupt on pin state change */
    COINES_PIN_INTERRUPT_RISING_EDGE,
    /*< Trigger interrupt when pin changes from low to high */
    COINES_PIN_INTERRUPT_FALLING_EDGE,
    /*< Trigger interrupt when pin changes from high to low */
    COINES_PIN_INTERRUPT_MODE_MAXIMUM = 4
};

struct coines_comm_intf_config
{
    uint32_t uart_baud_rate;
};

/*!
 * @brief uart parity bit
 */
enum coines_uart_parity
{
    COINES_UART_PARITY_NONE,
    /* Parity not included */
    COINES_UART_PARITY_EVEN
    /* Parity included */
};

/**@brief UART Flow Control modes for the peripheral.
 */
enum coines_uart_flow_control
{
    COINES_UART_FLOW_CONTROL_DISABLED,
    /*< UART Hw Flow Control is disabled. */
    COINES_UART_FLOW_CONTROL_ENABLED
    /*< Standard UART Hw Flow Control is enabled. */
};


/**********************************************************************************/
/* function prototype declarations */
/**********************************************************************************/
#if defined(PC)

/*!
 * @brief This API is used to COINES_SDK error codes to error strings
 *
 * @param[in]  error_code     : error_code
 *
 * @return Error string for the given error code
 */
const char *get_coines_error_str(int16_t error_code);

/*!
 * @brief This API is used to scan BLE devices
 *
 * @param[out] ble_info            : Array of structure containing found BLE peripherals' info like Address and Identifier
 * @param[out] peripheral_count    : The number of found BLE peripherals
 * @param[in]  scan_timeout_ms     : Timeout for BLE scan
 *
 * @return Result of API execution status.
 * @retval 0 -> Success.
 * @retval Any non zero value -> Fail.
 */
int16_t coines_scan_ble_devices(struct ble_peripheral_info *ble_info, uint8_t *peripheral_count, size_t scan_timeout_ms);

#endif

/*!
 * @brief This API is used to initialize the communication according to interface type.
 *
 * @param[in] intf_type : Type of interface(USB, COM, or BLE).
 * @param[in] arg       : Void pointer to store ble_peripheral_name for BLE communication
 *
 * @return Result of API execution status
 * @retval Zero -> Success
 * @retval Negative -> Error
 */
int16_t coines_open_comm_intf(enum coines_comm_intf intf_type, void *arg);

/*!
 * @brief This API is used to close the active communication(USB,COM or BLE).
 *
 * @param[in] intf_type : Type of interface(USB, COM, or BLE).
 * @param[in] arg       : Void pointer to store ble_peripheral_name for BLE communication
 *
 * @return Result of API execution status
 * @retval Zero -> Success
 * @retval Negative -> Error
 */
int16_t coines_close_comm_intf(enum coines_comm_intf intf_type, void *arg);

/*!
 *  @brief This API is used to test the communication.
 *
 *  @param[in] data  : Data to be sent for testing
 *  @param[in] length  : Length of the data
 *
 *  @return Result of API execution status.
 *  @retval 0 -> Success.
 *  @retval Any non zero value -> Fail.
 *
 */
int16_t coines_echo_test(uint8_t *data, uint16_t length);

/*!
 *  @brief This API is used to get the board information.
 *
 *  @param[out] data  : Board information details.
 *
 *  @return Result of API execution status.
 *  @retval 0 -> Success.
 *  @retval Any non zero value -> Fail.
 *
 */

int16_t coines_get_board_info(struct coines_board_info *data);

/*!
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 *
 *  @param[in] pin_number : pin to be configured.
 *  @param[in] direction : pin direction information(COINES_PIN_DIRECTION_IN or COINES_PIN_DIRECTION_OUT) *
 *  @param[in] pin_value : pin value information(COINES_PIN_VALUE_LOW or COINES_PIN_VALUE_HIGH)
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_set_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction direction,
                              enum coines_pin_value pin_value);

/*!
 *  @brief This API function is used to get the pin direction and pin state.
 *
 *  @param[in] pin_number : pin number for getting the status.
 *  @param[out] pin_direction : pin direction information(COINES_PIN_DIRECTION_IN or COINES_PIN_DIRECTION_OUT)
 *  @param[out] pin_value : pin value information(COINES_PIN_VALUE_LOW or COINES_PIN_VALUE_HIGH)*
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_get_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction *pin_direction,
                              enum coines_pin_value *pin_value);

/*!
 *  @brief This API is used to configure the VDD and VDDIO of the sensor.
 *
 *  @param[in] vdd_millivolt     : VDD voltage to be set in sensor.
 *  @param[in] vddio_millivolt   : VDDIO voltage to be set in sensor.
 *
 *  @note In APP2.0 board, voltage level of 0 or 3300mV is supported.
 *        In APP3.0 board, voltage levels of 0, 1800mV and 2800mV are supported.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt);

/*!
 *  @brief This API is used to configure the SPI bus
 *
 *  @param[in] bus         : bus
 *  @param[in] spi_speed   : SPI speed
 *  @param[in] spi_mode    : SPI mode
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_config_spi_bus(enum coines_spi_bus bus, enum coines_spi_speed spi_speed, enum coines_spi_mode spi_mode);

/*!
 *  @brief This API is used to de-configure the SPI bus
 *
 *  @param[in] bus         : bus
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_deconfig_spi_bus(enum coines_spi_bus bus);

/*!
 *  @brief This API is used to configure the SPI bus as either 8 bit or 16 bit length
 *
 *  @param[in] bus         : bus
 *  @param[in] spi_speed   : spi speed
 *  @param[in] spi_mode    : spi mode
 *  @param[in] spi_transfer_bits    : bits to transfer
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_config_word_spi_bus(enum coines_spi_bus bus,
                                   enum coines_spi_speed spi_speed,
                                   enum coines_spi_mode spi_mode,
                                   enum coines_spi_transfer_bits spi_transfer_bits);

/*!
 *  @brief This API is used to configure the I2C bus
 *
 *  @param[in] bus : i2c bus
 *  @param[in] i2c_mode   : i2c_mode
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_config_i2c_bus(enum coines_i2c_bus bus, enum coines_i2c_mode i2c_mode);

/*!
 *  @brief This API is used to de-configure the I2C bus
 *
 *  @param[in] bus : i2c bus
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_deconfig_i2c_bus(enum coines_i2c_bus bus);

/*!
 *  @brief This API is used to write 8-bit register data on the I2C device.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr : Device address for I2C write.
 *  @param[in] reg_addr : Starting address for writing the data.
 *  @param[in] reg_data : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_write_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

/*!
 *  @brief This API is used to read 8-bit register data from the I2C device.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr  : Device address for I2C read.
 *  @param[in] reg_addr  : Starting address for reading the data.
 *  @param[out] reg_data : Data read from the sensor.
 *  @param[in] count     : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_read_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

/*!
 *  @brief This API is used to write 16-bit register data on the SPI device.
 *
 *  @param[in] bus      : spi bus.
 *  @param[in] cs       : Chip select pin number for SPI write.
 *  @param[in] reg_addr : Starting address for writing the data.
 *  @param[in] reg_data : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_write_16bit_spi(enum coines_spi_bus bus, uint8_t cs, uint16_t reg_addr, void *reg_data, uint16_t count);

/*!
 *  @brief This API is used to write 8-bit register data on the SPI device.
 *
 *  @param[in] bus      : spi bus.
 *  @param[in] dev_addr : Chip select pin number for SPI write.
 *  @param[in] reg_addr : Starting address for writing the data.
 *  @param[in] reg_data : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_write_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

/*!
 *  @brief This API is used to read 16-bit register data from the SPI device.
 *
 *  @param[in] bus       : spi bus.
 *  @param[in] cs        : Chip select pin number for SPI read.
 *  @param[in] reg_addr  : Starting address for reading the data.
 *  @param[out] reg_data : Data read from the sensor.
 *  @param[in] count     : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_read_16bit_spi(enum coines_spi_bus bus, uint8_t cs, uint16_t reg_addr, void *reg_data, uint16_t count);

/*!
 *  @brief This API is used to read the data in SPI communication.
 *
 *  @param[in] bus      : spi bus.
 *  @param[in] dev_addr : Chip select pin number for SPI read.
 *  @param[in] reg_addr : Starting address for reading the data.
 *  @param[out] reg_data : Data read from the sensor.
 *  @param[in] count    : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_read_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

/*!
 *  @brief This API is used for introducing a delay in milliseconds
 *
 *  @param[in] delay_ms   :  delay in milliseconds.
 *
 *  @return void
 */
void coines_delay_msec(uint32_t delay_ms);

/*!
 *  @brief This API is used for introducing a delay in microseconds
 *
 *  @param[in] delay_us   :  delay in microseconds.
 *
 *  @return void
 */
void coines_delay_usec(uint32_t delay_us);

#if !defined(MCU_NICLA) && !defined(MCU_APP20)

/*!
 * @brief This API is used to send the streaming settings to the board.
 *
 * @param[in] channel_id    :  channel identifier (Possible values - 1,2)
 * @param[in] stream_config :  stream_config
 * @param[in] data_blocks   :  data_blocks
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_config_streaming(uint8_t channel_id,
                                struct coines_streaming_config *stream_config,
                                struct coines_streaming_blocks *data_blocks);

/*!
 * @brief This API is used to start or stop the streaming.
 *
 * @param[in] stream_mode :  stream_mode
 * @param[in] samples     :  samples
 * @param[in] start_stop  :  start or stop steaming
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_start_stop_streaming(enum coines_streaming_mode stream_mode, uint8_t start_stop);

/*!
 * @brief This API is used to read the streaming sensor data.
 *
 * @param[in] sensor_id             :  Sensor Identifier.
 * @param[in] number_of_samples     :  Number of samples to be read.
 * @param[out] data                 :  Buffer to retrieve the sensor data.
 * @param[out] valid_samples_count  :  Count of valid samples available.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_read_stream_sensor_data(uint8_t sensor_id,
                                       uint32_t number_of_samples,
                                       uint8_t *data,
                                       uint32_t *valid_samples_count);

#endif

/*!
 * @brief This API is used to configure the hardware timer
 *
 * @param[in] instance : timer instance
 * @param[in] handler : callback to be called when timer expires.
 *                      use the function pointer for timer event handler as per the TARGET
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_timer_config(enum coines_timer_instance instance, void* handler);

/*!
 * @brief This API is used to de-configure the hardware timer
 *
 * @param[in] instance : timer instance
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_timer_deconfig(enum coines_timer_instance instance);

/*!
 * @brief This API is used to start the configured hardware timer
 *
 * @param[in] instance : timer instance
 * @param[in] timeout : in microsecond
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_timer_start(enum coines_timer_instance instance, uint32_t timeout);

/*!
 * @brief This API is used to stop the hardware timer
 *
 * @param[in] instance : timer instance
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_timer_stop(enum coines_timer_instance instance);

/*!
 * @brief This API is used to trigger the timer in firmware and enable or disable system time stamp
 *
 * @param[in] tmr_cfg : timer config value
 * @param[in] ts_cfg : timer stamp cfg value
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_trigger_timer(enum coines_timer_config tmr_cfg, enum coines_time_stamp_config ts_cfg);

/*!
 * @brief This API returns the number of milliseconds passed since the program started
 *
 * @return Time in milliseconds
 */
uint32_t coines_get_millis();

/*!
 * @brief This API returns the number of microseconds passed since the program started
 *
 * @return Time in microseconds
 */
uint64_t coines_get_micro_sec();

/*!
 * @brief This API is used to introduce delay based on high precision RTC(LFCLK crystal)
 * with the resolution of 30.517 usec
 *
 * @param[in]   : required delay in microseconds
 * @return      : None
 */
void coines_delay_realtime_usec(uint32_t period);

/*!
 * @brief This API is used to get the current counter(RTC) reference time in usec
 *
 * @param[in]   : None
 * @return      : counter(RTC) reference time in usec
 * */
uint32_t coines_get_realtime_usec(void);

/*!
 * @brief Attaches a interrupt to a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 * @param[in] interrupt_cb : Name of the function to be called on detection of interrupt
 * @param[in] mode : Trigger modes - change,rising edge,falling edge
 *
 * @return void
 */
void coines_attach_interrupt(enum coines_multi_io_pin pin_number,
                             void (*interrupt_cb)(uint32_t, uint32_t),
                             enum coines_pin_interrupt_mode int_mode);

/*!
 * @brief Detaches a interrupt from a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 *
 * @return void
 */
void coines_detach_interrupt(enum coines_multi_io_pin pin_number);

/*!
 * @brief Attaches a timed interrupt to a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 * @param[in] timed_interrupt_cb : Name of the function to be called on detection of interrupt
 * @param[in] mode : Trigger modes - change,rising edge,falling edge
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_attach_timed_interrupt(enum coines_multi_io_pin pin_number,
                                      void (*timed_interrupt_cb)(uint64_t,
                                                                 uint32_t,
                                                                 uint32_t),
                                      enum coines_pin_interrupt_mode int_mode);

/*!
 * @brief Detaches a timed interrupt from a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_detach_timed_interrupt(enum coines_multi_io_pin pin_number);

/*!
 * @brief Get COINES_SDK library version
 *
 * @return pointer to version string
 */
const char* coines_get_version();

/*!
 * @brief Return the number of bytes available in the read buffer of the interface
 *
 * @param[in] intf : Type of interface(USB, COM, or BLE).
 *
 * @return number of bytes in the read buffer
 */
uint16_t coines_intf_available(enum coines_comm_intf intf);

/*!
 * @brief Check if the interface is connected
 *
 * @param[in] intf : Type of interface(USB, COM, or BLE).
 *
 * @return true if connected, false otherwise
 */
bool coines_intf_connected(enum coines_comm_intf intf);

/*!
 * @brief Read data over the specified interface
 *
 * @param[in] intf    : Type of interface(USB, COM, or BLE).
 * @param[out] buffer : Pointer to the buffer to store the data
 * @param[in] len     : Length of the buffer
 *
 * @return number of bytes read
 */
uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len);

/*!
 * @brief Write data over the specified interface
 *
 * @param[in] intf    : Type of interface(USB, COM, or BLE).
 * @param[out] buffer : Pointer to the buffer storing the data
 * @param[in] len     : Length of the buffer
 *
 * @return void
 */
void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len);

/*!
 * @brief This API is used to read the temperature sensor data.
 *
 * @param[out] temp_conv_data       :  Buffer to retrieve the sensor data in degC.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_read_temp_data(float *temp_data);

/*!
 * @brief This API is used to read the battery status .
 *
 * @param[out] bat_status_mv            :  Buffer to retrieve the battery status in millivolt.
 *
 * @param[out] bat_status_percent       :  Buffer to retrieve the battery status in %.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_read_bat_status(uint16_t *bat_status_mv, uint8_t *bat_status_percent);

/*!
 * @brief Resets the device
 *
 * @note  After reset device jumps to the address specified in makefile (APP_START_ADDRESS).
 *
 * @return void
 */
void coines_soft_reset(void);

/*!
 *  @brief This API is used to write the data in I2C communication.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr : Device address for I2C write.
 *  @param[in] data     : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_i2c_set(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count);

/*!
 *  @brief This API is used to read the data in I2C communication.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr : Device address for I2C read.
 *  @param[out] data    : Data read from the sensor.
 *  @param[in] count    : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_i2c_get(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count);

/*!
 *  @brief This API is used to configure BLE name and power.This API should be called
 *         before calling coines_open_comm_intf().
 *
 *  @param[in] ble_config : structure holding ble name and power details
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int16_t coines_ble_config(struct coines_ble_config *ble_config);

/*!
 *  @brief This API is used to set led state(on or off).
 *
 *  @param[in] led             : led to which the state has to be set
 *  @param[in] led_state       : state to be set to the given led
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_set_led(enum coines_led led, enum coines_led_state led_state);

/*!
 * @brief Flush the buffer
 *
 * @param[in] intf    : Type of interface(USB, COM, or BLE).
 *
 * @return void
 */
void coines_flush_intf(enum coines_comm_intf intf);

#if !defined(MCU_APP20)

/*!
 *  @brief This API is used to write the content into shuttle eeprom.
 *
 *  @param[in]  start_addr  : EEPROM write address.
 *  @param[in]  buffer      : Pointer to the buffer.
 *  @param[out] length      : Length of the buffer.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_shuttle_eeprom_write(uint16_t start_addr, uint8_t *buffer, uint16_t length);

/*!
 *  @brief This API is used to execute the function inside critical region.
 *
 *  @param[in]  start_addr  : EEPROM read address.
 *  @param[in]  buffer      : Pointer to the buffer.
 *  @param[out] length      : Length of the buffer.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_shuttle_eeprom_read(uint16_t start_addr, uint8_t *buffer, uint16_t length);

/**
 * @brief This API can be defined to perform a task when yielded from an ongoing blocking call
 */
void coines_yield(void);

#endif

#if (defined(MCU_APP30) || defined(MCU_APP31))

/**
 * @brief This API is used to configure the I2S bus to match the TDM configuration
 *
 * @param[in] data_words : number of words to use in the buffer. Max is set at COINES_TDM_BUFFER_SIZE_WORDS
 * @param[in] callback   : register a callback to be called to process and copy the data over
 *
 * @return Results of API execution status.
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_config_i2s_bus(uint16_t data_words, coines_tdm_callback callback);

/**
 * @brief This API is used to stop the I2S/TDM interface from reading data from the sensor
 */
void coines_deconfig_i2s_bus(void);

/**
 * @brief This API is used to initialize UART communication
 *
 * @param[in] uart_instance : specifies the UART instance
 * @param[in] parity        : UART parity
 * @param[in] flow_control  : UART flow control mode
 * @param[in] baud_rate     : UART baud rate
 * @return Results of API execution status.
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int8_t coines_uart_init(enum coines_uart_instance uart_instance,
                        enum coines_uart_parity parity,
                        enum coines_uart_flow_control flow_control,
                        uint32_t baud_rate);
/*!
 * @brief This API is used to read the data in UART communication
 *
 * @param[in] uart_instance : specifies the UART instance
 * @param[out] buffer       : Pointer to the buffer to store the data
 * @param[in] length        : Length of the buffer
 *
 * @return Number of bytes read
 */
uint16_t coines_uart_read(enum coines_uart_instance uart_instance, uint8_t *buffer, uint16_t length);
/*!
 *  @brief This API is used to write the data in UART communication
 *
 * @param[in] uart_instance : specifies the UART instance
 * @param[in] buffer        : Pointer to the data buffer which need to be written
 * @param[in] length        : Length of the buffer
 *
 * @return Results of API execution status.
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int8_t coines_uart_write(enum coines_uart_instance uart_instance, uint8_t *buffer, uint16_t length);


#endif

/*!
 *  @brief This API is used to execute the function inside critical region.
 *
 *  @param[in] callback   : function to execute
 *
 *  @return void
 */
void coines_execute_critical_region(coines_critical_callback callback);

#if defined(MCU_NICLA)

/*!
 * @brief This API is used to switch off the board
 */
void coines_ship_mode(void);

#endif



#ifdef __cplusplus
}
#endif

#endif /* COINES_H_ */

/** @}*/

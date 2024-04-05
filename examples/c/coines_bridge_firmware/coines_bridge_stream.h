/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    coines_bridge_stream.h
 * @brief   This file contains function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef COINES_BRIDGE_STREAM_H_
#define COINES_BRIDGE_STREAM_H_

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! COINES_SDK maximum timestamp size */
#define TIMESTAMP_SIZE                    UINT8_C(6)

/*! COINES_SDK maximum sensor for streaming */
#define STREAM_MAX_COUNT_T                3

/*! COINES_SDK streaming hardware pin state mask */
#define COINES_STREAM_INT_PIN_MASK        0x0F

/*! COINES_SDK streaming hardware pin state mask */
#define COINES_STREAM_INT_PIN_STATE_MASK  0x10

#define STREAM_BUFF_SIZE                  UINT16_C(COM_READ_BUFF_SIZE + COINES_MAX_HEADER_LEN + TIMESTAMP_SIZE)

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/

/*!
 * @brief timestamp config
 */
enum coines_stream_timestamp {
    COINES_STREAM_NO_TIMESTAMP = 0, /*< no timestamp */
    COINES_STREAM_USE_TIMESTAMP = 1 /*< Timestamp is present */
};

/*!
 * @brief streaming mode
 */
enum coines_stream_mode {
    COINES_STREAM_MODE_INTERRUPT, /*< interrupt*/
    COINES_STREAM_MODE_POLLING, /*< polling*/
    COINES_STREAM_MODE_FIFO_POLLING /*<fifo polling*/
};

struct coines_stream_settings
{
    uint32_t gst_period_us; /*< Global Sampling Timer period (in polling mode, all sensor sampling periods are multiple
                             * of this period)*/
    enum coines_stream_timestamp ts_mode; /*< ts mode */
    enum coines_stream_mode stream_mode; /*< stream mode */
};

/*!
 * @brief streaming clear on write settings
 */
struct coines_stream_clear_on_write
{
    uint8_t dummy_byte; /*< dummy byte count */
    uint8_t startaddress; /*< starting address */
    uint16_t num_bytes_to_clear; /* < No. of bytes to clear */
    uint8_t data_buf[255]; /*< data chunks */
};

/*!
 * @brief polling streaming config settings
 */
struct coines_poll_streaming
{
    uint8_t sensor_id; /*< streaming sensor id */
    uint8_t timestamp;  /*< 1- enable /0- disable time stamp for corresponding sensor */
    enum coines_sensor_intf intf; /*< Sensor Interface */
    uint8_t intf_bus; /*< Bus Interface */
    uint8_t intf_addr; /*< I2C address/SPI CS pin */
    uint16_t sampling_time; /*< Sampling time */
    enum coines_sampling_unit sampling_units; /*< micro second / milli second - Sampling unit */
    uint16_t no_of_blocks; /*< Number of blocks to read*/
    uint8_t reg_start_addr[COINES_MAX_BLOCKS]; /*< Register start address */
    uint8_t no_of_data_bytes[COINES_MAX_BLOCKS]; /*< Number of data bytes */
    uint8_t spi_type; /*< spi type */
    uint8_t clear_on_write; /*< clear on write */
    struct coines_stream_clear_on_write clear_on_write_config;
    uint8_t intline_count; /*< interrupt line count */
    uint8_t intline_info[COINES_MAX_INT_LINE]; /*< interrupt line number */
    uint8_t DATA_intline[COINES_MAX_INT_LINE]; /*< interrupt line state */
    uint64_t packet_timestamp_us; /*< packet timestamp */
    uint32_t sampling_period_us;
    uint32_t gst_ticks_counter;
    uint32_t gst_multiplier;
};

/*!
 * @brief interrupt streaming config settings
 */
struct coines_int_streaming
{
    uint8_t sensor_id; /*< streaming sensor id */
    uint8_t timestamp;
    enum coines_sensor_intf intf; /*< Sensor Interface SPI/I2C */
    uint8_t intf_bus; /*< Bus Interface */
    uint8_t intf_addr; /*< I2C address/SPI CS pin */
    uint8_t int_pin; /*< data ready initialization */
    uint16_t no_of_blocks; /*< Number of blocks to read*/
    uint8_t reg_start_addr[COINES_MAX_BLOCKS]; /*< Register start address */
    uint8_t no_of_data_bytes[COINES_MAX_BLOCKS]; /*< Number of data bytes */
    uint8_t spi_type; /*< spi type */
    uint8_t clear_on_write; /*< clear on write */
    uint8_t hw_pin_state; /*< hardware pin state active low/high */
    struct coines_stream_clear_on_write clear_on_write_config;
    uint8_t intline_count; /*< intterupt line count */
    uint8_t intline_info[COINES_MAX_INT_LINE]; /*< interrupt line number */
    uint8_t DATA_intline[COINES_MAX_INT_LINE]; /*< interrupt line state */
    uint64_t packet_timestamp_us; /*< packet timestamp */
};

union coines_streaming
{
    struct coines_poll_streaming poll_config;
    struct coines_int_streaming int_config;
};

/*!
 * @brief data stored and retrived from job queue
 */
typedef struct
{
    uint32_t multiio_pin;
    uint32_t packet_no;
    uint64_t timestamp_us;
} coines_bridge_job_data_t;

/**********************************************************************************/
/* functions */
/**********************************************************************************/
/*!
 * @brief This API is used to configure global sampling timer period for all polling streaming.
 *
 */
int16_t poll_streaming_common(uint8_t cmd,
                              uint8_t *payload,
                              uint16_t payload_length,
                              uint8_t *resp,
                              uint16_t *resp_length);

/*!
 * @brief This API is used to get and configure polling stream settings.
 *
 */
int16_t poll_streaming_config(uint8_t cmd,
                              uint8_t *payload,
                              uint16_t payload_length,
                              uint8_t *resp,
                              uint16_t *resp_length);

/*!
 * @brief This API is used to get and configure interrupt stream settings
 *
 */
int16_t int_streaming_config(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);

/*!
 * @brief This API is used to start/stop the streaming.
 *
 */
int16_t streaming_start_stop(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);

/*!
 * @brief This API is used to check streaming is configured and triggered
 *
 */
void send_streaming_response(void);

/*!
 * @brief This API is used to transmit the buffered streaming data
 *
 */
void transmit_streaming_rsp(void);

#ifdef __cplusplus
}
#endif

#endif /* COINES_BRIDGE_STREAM_H_ */

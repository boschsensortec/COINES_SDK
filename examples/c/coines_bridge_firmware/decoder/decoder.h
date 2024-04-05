/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file        decoder.h
 *
 * @brief       This file decodes the commands and process accordingly
 *
 */

#ifndef APPLICATION_DECODER_H_
#define APPLICATION_DECODER_H_
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! Header position in responses */
#define DECODER_BYTEPOS_HEADER               UINT8_C(0)

/*! Size position in responses */
#define DECODER_BYTEPOS_SIZE                 UINT8_C(1)

/*! Direction position in responses */
#define DECODER_BYTEPOS_DIR                  UINT8_C(2)

/*! Command feature position in responses */
#define DECODER_BYTEPOS_FEATURE              UINT8_C(3)

/*! Header value in commands and responses */
#define DECODER_HEADER_VALUE                 UINT8_C(0xAA)

/*! Unknown command size */
#define DECODER_UNKNOWN_SIZE                 UINT8_C(0)

/*! Maximum command or response packet size */
#define DECODER_MAX_PKT_SZ                   UINT8_C(255)

/*! Maximum allowed cs pin, it is based in IO's */
#define DECODER_MAX_CS_PIN                   UINT8_C(9)

/*! Burst mode value in read and write commands */
#define DECODER_RW_MODE_BURST                UINT8_C(1)

/*! Nomal mode value in read and write commands */
#define DECODER_RW_MODE_NORMAL               UINT8_C(2)

/*! Burst accumulate mode value in read and write commands */
#define DECODER_RW_MODE_BURST_ACCUMULATE     UINT8_C(3)

/*! CR value in response and command packets */
#define DECODER_CR_MACRO                     UINT8_C(0x0D)

/*! LF value in response and command packets */
#define DECODER_LF_MACRO                     UINT8_C(0x0A)

/*! Command id mask in response */
#define DECODER_RSP_ID_MASK                  UINT8_C(0x40)

/*! Minimum size of the command packet */
#define DECODER_END_INDICATORS_SIZE          UINT8_C(0x02)            /*CR and LF */

/*! Over head bytes in polling streaming response packet */
#define DECODER_STRM_POLL_RSP_OVERHEAD       UINT8_C(0x05)

/*! Over head bytes in interrupt streaming response packet */
#define DECODER_STRM_INT_RSP_OVERHEAD        UINT8_C(0x0A)

/*! Over head bytes in FIFO polling streaming response packet */
#define DECODER_STRM_FIFO_POLL_RSP_OVERHEAD  UINT8_C(0x05)

/*! Over head bytes in FIFO streaming response packet */
#define DECODER_STRM_FIFO_RSP_OVERHEAD       UINT8_C(0x0E)

/*! Actual data in polling streaming response */
#define DECODER_STRM_POLL_RSP_PL_SZ          (UINT8_C(DECODER_MAX_PKT_SZ - \
                                                      (DECODER_STRM_POLL_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE)))

/*! Actual data in interrupt streaming response */
#define DECODER_STRM_INT_RSP_PL_SZ           (UINT8_C(DECODER_MAX_PKT_SZ - \
                                                      (DECODER_STRM_INT_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE)))

/*! Actual data in fifo polling streaming response */
#define DECODER_STRM_FIFO_POLL_RSP_PL_SZ     (UINT8_C(DECODER_MAX_PKT_SZ - \
                                                      (DECODER_STRM_FIFO_POLL_RSP_OVERHEAD + \
                                                       DECODER_END_INDICATORS_SIZE)))

/*! Actual data in fifo streaming response */
#define DECODER_STRM_FIFO_RSP_PL_SZ          (UINT8_C(DECODER_MAX_PKT_SZ - \
                                                      (DECODER_STRM_FIFO_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE)))

/*! FIFO polling streaming response identifier */
#define DECODER_STRM_FIFO_POLL_RSP_ID        UINT8_C(0x88)

/*! Polling streaming response identifier */
#define DECODER_STRM_POLL_RSP_ID             UINT8_C(0x87)

/*! Old polling streaming response identifier */
#define DECODER_STRM_OLD_POLL_RSP_ID         UINT8_C(0x86)

/*! Interrupt streaming response identifier */
#define DECODER_STRM_INT_RSP_ID              UINT8_C(0x8A)

/*! vdd voltage for 1800*/
#define VDD_VDDIO_1V8                        UINT16_C(1800)

/*! vdd voltage for 2800*/
#define VDD_VDDIO_2V8                        UINT16_C(2800)

/*! vdd voltage for 3300*/
#define VDD_VDDIO_3V3                        UINT16_C(3300)

/* Macros of memory locations used for performing application switch */
/*! Ram location to keep the magic data for BTL */
#define  DECODER_BTL_MAGIC_LOCATION          (0x2003FFF4)
#define  DECODER_BTL_MAGIC_INFO_ADDR         ((int8_t *)(DECODER_BTL_MAGIC_LOCATION))

/*! Magic data which is used by BTL to stay and upload the FW */
#define  DECODER_BTL_MAGIC_DATA              "COIN"

/* Location in which application start address saved before switching */
#define  DECODER_BTL_APP_START_ADDR          (*(uint32_t *)(DECODER_BTL_MAGIC_LOCATION + 4))

/*! Application stack pointer value */
#define  DECODER_APP_SP_VALUE                (*(uint32_t *)DECODER_BTL_APP_START_ADDR)

/* Application reset handler address */
#define  DECODER_APP_RESET_HANDLER_ADDR      (*(uint32_t *)(DECODER_BTL_APP_START_ADDR + 4))

/*! APP2.0 - 3, BNO USB stick - 4, APP3.0 - 5 */
#define  DECODER_BOARD_TYPE_APP30            (5)

/*! APP3.0 Board version - v1.0 */
#define  DECODER_APP30_BOARD_VERSION         (0x10)

/*! APP3.0 native shuttle pin identification bit */
#define  APP30_SHUTTLE_PIN_ID                (1 << 15)

/*! Clear on write mask value */
#define  DECODER_CLEAR_ON_WRITE_MASK         (0x80)

/*! hardware pin state mask value */
#define  DECODER_HW_PIN_STATE_MASK           (0x40)

/*! Dummy byte count mask value */
#define  DECODER_DUMMY_BYTE_COUNT_MASK       (0x3F)

#define TIMESTAMP_SIZE                  	UINT8_C(6)

#define DECODER_RET_INSUFFICIENT_MEMORY      1       /*< insufficient memory */
#define DECODER_RET_COMM_FAILED              2       /*< failed */
#define DECODER_RET_NULL_PARAM               3       /*< parameter is null */
#define DECODER_RET_INVALID_PARAM            4       /*< parameter is invalid */

/**********************************************************************************/
/* type definitions */
/**********************************************************************************/

/*!
 *
 * @brief : Enum for write / read
 *
 */
typedef enum
{
    DECODER_DIR_SET = 1, /*< direction for write */
    DECODER_DIR_GET = 2 /*< direction for read*/
} decoder_dir_t;

/*!
 *
 * @brief : Enum for the response status
 *
 */
typedef enum
{
    DECODER_RSP_SUCCESS = 0, /*< success response */
    DECODER_RSP_FAIL = -1, /*< failure response */
    DECODER_RSP_PARAM_OUT_OF_RANGE = -2, /*<parameters out of range */
    DECODER_RSP_MISSING_PARAM = -3, /*<missing parameters */
    DECODER_RSP_NO_DATA = -4, /*< data unavailable */
    DECODER_RSP_UNKNOWN_INSTR = -5, /*< unknown instruction */
    DECODER_RSP_SIZE_MISMATCH = -6, /*< size mismatch */
    DECODER_RSP_USB_FAILURE = -7, /*< USB failure */
    DECODER_RSP_NO_FREE_STREAM_CHANNEL = -8, /*< stream channel is unavailable */
    DECODER_RSP_NO_CONFIGURED_STREAM_CHANNEL = -9, /*< configured channel is unavailable */
    DECODER_RSP_EEPROM_ERROR = -10, /*< EEPROM error */
    DECODER_RSP_INVALID_ADDRESS = 1, /*< invalid address */
    DECODER_RSP_INVALID_APPLICATION = 2, /*< invalid application */
    DECODER_RSP_NULL_PARAM = 3 /* parameter is null*/
} decoder_rsp_t;

typedef decoder_rsp_t (*decoder_cmd_hdlr_t)(const uint8_t*, decoder_dir_t);

/*!
 *
 * @brief : Structure for command processing
 *
 */
typedef struct decoder_cmd_type
{
    decoder_cmd_hdlr_t handler;/*<handler*/
    uint8_t getter_length; /* getter_length */
    uint8_t setter_length; /* setter_length */
} decoder_cmd_t;

/*!
 *
 * @brief : Enum for command id's
 *
 */
typedef enum
{
    DECODER_ID_VDD = 0x01, /*< decoder ID for VDD */
    DECODER_ID_VDDIO = 0x02, /*< decoder ID for VDDIO */
    DECODER_ID_ASW = 0x03, /*< decoder ID for ASW */
    DECODER_ID_IODIR = 0x04, /*< decoder ID for IO direction*/
    DECODER_ID_IO = 0x05, /*< decoder ID for IO */

    DECODER_ID_I2CSPEED = 0x09, /*< decoder ID for I2C speed */
    DECODER_ID_SPICS = 0x0A, /*< decoder ID for SPI chip select */
    DECODER_ID_INTERFACE = 0x11, /*< decoder ID for interface */
    DECODER_ID_LED_CONTROL = 0x0B, /*< decoder ID for led control */

    DECODER_ID_SHUTTLE_PWR_CFG = 0x14, /*< decoder ID for shuttle configuration */

    DECODER_ID_MULTIIOCONFIG = 0x15, /*< decoder ID for multi IO configuration */
    DECODER_ID_WRITEREAD = 0x16, /*< decoder ID for write read */
    DECODER_ID_SPICONFIG = 0x19, /*< decoder ID for SPI configuration */
    DECODER_ID_DELAY = 0x1A, /*< decoder ID for delay */
    DECODER_ID_BOARDTYPE = 0x1E, /*< decoder ID for board type */
    DECODER_ID_BOARDINFO = 0x1F, /*< decoder ID for board information */

    DECODER_ID_WRITEDELAYREAD = 0x22, /*< decoder ID for write delay read */
    DECODER_ID_THIRDPARTYWRITEREAD = 0x28, /*< decoder ID for third party read */

    DECODER_ID_TIMESTAMP = 0x29, /*< decoder ID for timestamp */
    DECODER_ID_APP_SWITCH = 0x30, /*< decoder ID for app switch */
    DECODER_ID_INVOKE_BTL = 0x31, /*< decoder ID for invoke BTL */
    DECODER_ID_SHUTTLE_EEPROM_RW = 0x32, /*< decoder ID for shuttle eeprom */
    _DECODER_EXT_ID_BASE_ = 0x33, /*< decoder exit ID for base */

    DECODER_EXTID_POLLSTREAM_COMMON = _DECODER_EXT_ID_BASE_ + 0x03,

    /*< decoder exit ID for communication in poll
     * streaming */
    DECODER_EXTID_POLLSTREAM_OLD_READ = _DECODER_EXT_ID_BASE_ + 0x04, /*< decoder exit ID for read in poll streaming */
    DECODER_EXTID_POLLSTREAM_STARTSTOP = _DECODER_EXT_ID_BASE_ + 0x06,

    /*< decoder exit ID for start and stop in poll
     * stream */
    DECODER_EXTID_FIFOSTREAM_READ = _DECODER_EXT_ID_BASE_ + 0x07, /*< decoder exit ID for fifo stream read */
    DECODER_EXTID_FIFOSTREAM_STARTSTOP = _DECODER_EXT_ID_BASE_ + 0x08,

    /*< decoder exit ID for start and stop in fifo
     * stream */
    DECODER_EXTID_INTSTREAM_OLD_READ = _DECODER_EXT_ID_BASE_ + 0x09,

    /*< decoder exit ID for read in interrupt streaming
    */
    DECODER_EXTID_INTSTREAM_STARTSTOP = _DECODER_EXT_ID_BASE_ + 0x0A,

    /*< decoder exit ID for start and stop in
     * interrupt streaming */
    DECODER_EXTID_INTSTREAM_READ = _DECODER_EXT_ID_BASE_ + 0x0E, /*< decoder exit ID for read in interrupt streaming */
    DECODER_EXTID_POLLSTREAM_READ = _DECODER_EXT_ID_BASE_ + 0x0F, /*< decoder exit ID for read in poll streaming  */

    _DECODER_ID_MAX_
} decoder_id_t;

/**********************************************************************************/
/* (extern) variable declarations */
/**********************************************************************************/
extern uint8_t decoder_stream_samples_count;

/*! Buffer used to update the command responses to transmit to the host */
extern uint8_t xmit_buffer[DECODER_MAX_PKT_SZ];

/**********************************************************************************/
/* function prototype declarations */
/**********************************************************************************/

/*!
 *
 * @brief       : API will decodes the command from host and call appropriate handlers to process
 *
 * @param[in]   : cmd - pointer which points the command buffer
 *
 * @return      : None
 */
void decoder_process_cmds(const uint8_t *cmd);

/*!
 * @brief : Function to write the response data back into the interface
 */
void decoder_write_resp(void *buff, uint16_t len);

/**********************************************************************************/
/* inline function definitions */
/**********************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DECODER_H_ */

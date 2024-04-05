/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    app20-flash.c
 * @date    March 6, 2019
 * @version 1.0
 * @brief   Firmware flash tool for APP2.0 and BNO USB stick
 *
 */

#include "app20-flash.h"

coines_rsp_buffer_t coines_rsp_buffer;

int main(int argc, char *argv[])
{
    FILE *file_h = NULL;
    uint8_t fw_data[WRITE_SIZE] = {};
    uint8_t data[PAYLOAD_SIZE] = {};

    printf("Application Board 2.0 firmware flash tool\n");
    printf("Bosch Sensortec GmbH (C) 2019\n");

    if (argc > 2 || argc == 1)
    {
        printf("\nInvalid/Insufficient arguments !!");
        printf("\n\n%s <firmware_file>", argv[0]);
        printf("\n\nEg:  %s DD2_0.fwu2", argv[0]);
        printf("\n");
        exit(EXIT_FAILURE);
    }

    file_h = fopen(argv[1], "rb");

    if (file_h == NULL)
    {
        printf("\nUnable to open file - %s\n", argv[1]);
        exit(EXIT_FAILURE);
    }

    coines_board_t board_type;

    if (comm_intf_open(COINES_COMM_INTF_USB, &board_type) < 0)
    {
        printf("\nUnable to connect to device !\n");
        exit(EXIT_FAILURE);
    }

    uint32_t file_size = get_file_size(file_h);
    uint32_t num_of_blocks = file_size / WRITE_SIZE;

    app2_check_mode(); /*Check if board is in boot mode*/
    uint8_t board = app2_read_board_type(); /*Check if APP2.0 (or) BNO USB stick*/

    if (board == APP20_BOARD && file_size > MAX_FILE_SIZE_APP20)
    {
        printf("Maximum file size for APP2.0 can be %d kB.\n", MAX_FILE_SIZE_APP20 / 1024);
        exit(EXIT_FAILURE);
    }
    
    /* File size check is mandatory for BNO USB stick. 
     * There is a bug in BNO USB stick bootloader. File size > 64kB can brick the device. 
     * I already bricked one ! :(
     */ 
    if (board == BNO_USB_STICK && file_size > MAX_FILE_SIZE_BNO_STICK)
    {
        printf("Maximum file size for BNO USB stick can be %d kB.\n", MAX_FILE_SIZE_BNO_STICK / 1024);
        exit(EXIT_FAILURE);
    }

    app2_erase(); /*Erase flash*/
    app2_flash(); /*Start flash*/

    /*Send firmware*/
    for (int i = 0; i <= num_of_blocks; i++)
    {
        file_read_by_len(file_h, fw_data, WRITE_SIZE);
        uint16_t x = 0;

        for (uint8_t packet_no = 0x01; packet_no < NUM_OF_PACKETS; packet_no++)
        {
            memcpy(data, &fw_data[x], PAYLOAD_SIZE);
            x += PAYLOAD_SIZE;
            app2_send_fw_data(data, packet_no, PAYLOAD_SIZE, false);
        }

        for (int i = 0; i <= (WRITE_SIZE - x + 0x0A); i++, x++)
        {
            data[i] = fw_data[x];
        }

        data[PAYLOAD_SIZE - 1] = get_checksum(fw_data, WRITE_SIZE);

        app2_send_fw_data(data, NUM_OF_PACKETS, PAYLOAD_SIZE, true);

        /*Progress bar*/
        int t = i * PROGRESS_BAR_SIZE / num_of_blocks;
        int no_of_symbols = 0;
        if (t != no_of_symbols)
        {
            no_of_symbols = t;
            printf("Download  [");
            int u = no_of_symbols, v = PROGRESS_BAR_SIZE - no_of_symbols;
            while (u--)
                putchar('=');
            while (v--)
                putchar(' ');
            printf("]  %d %%", i * 100 / num_of_blocks);
            putchar('\r');
            fflush(stdout);
        }
    }

    printf("\n\nDownload complete !\n");

    app2_update_success(); /*Complete firmware update*/

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    fclose(file_h);

    exit(EXIT_SUCCESS);
}

/*!
 *  @brief Erase flash memory
 */

static void app2_erase()
{
    printf("Erasing flash memory ...\r");
    fflush(stdout);
    comm_intf_init_command_header(BOOT_MODE_SET, ERASE_CMD);
    comm_intf_put_u8(9); /*Historic fields from APP1.0 !! :-D */
    comm_intf_put_u8(26); /* "  "  " */
    comm_intf_send_command(&coines_rsp_buffer);
    coines_delay_msec(4000); /*Wait for erase to complete*/
}

/*!
 *  @brief Initiate flash memory write
 */

static void app2_flash()
{
    comm_intf_init_command_header(BOOT_MODE_SET, FLASH_CMD);
    comm_intf_put_u8(9); /*Historic fields from APP1.0 !! :-D */
    comm_intf_put_u8(26);/* "  "  "*/
    comm_intf_put_u8(2); // 2*256B
    comm_intf_send_command(&coines_rsp_buffer);
}

/*!
 *  @brief Send partial firmware image
 */

static void app2_send_fw_data(uint8_t *data, uint8_t packet_no, uint8_t len, bool is_last)
{
    comm_intf_init_command_header(BOOT_MODE_SET, DATA_CMD);
    comm_intf_put_u8(NUM_OF_PACKETS); /*No of packets*/
    comm_intf_put_u8(packet_no);
    for (uint8_t i = 0; i < len; i++)
        comm_intf_put_u8(data[i]);

    if (comm_intf_send_command(NULL) != COINES_SUCCESS)
    {
        printf("\n\nDownload error !\n");
        exit(EXIT_FAILURE);
    }

    if (is_last == true)
        coines_delay_msec(10);

    memset(data, -1, len);

}

/*!
 *  @brief Complete firmware update
 */

static void app2_update_success()
{
    comm_intf_init_command_header(BOOT_MODE_SET, UPDATE_SUCCESS_CMD);
    comm_intf_send_command(NULL);
    coines_delay_msec(100);
}

/*!
 *  @brief Check if board is in bootloader mode
 */

static void app2_check_mode()
{
    comm_intf_init_command_header(BOOT_MODE_SET, MODE_CMD);
    comm_intf_send_command(&coines_rsp_buffer);

    if (coines_rsp_buffer.buffer[6] != 1)
    {
        printf("\nDevice not in bootloader mode !");
        printf("\nPut the device to bootloader mode.\n");
        printf("\nAPP2.0 - Turn on board with switch-2 pressed.");
        printf("\nBNO USB stick - Slide the switch to right side before connecting to PC.\n");
        exit(EXIT_FAILURE);
    }
}

/*!
 *  @brief Check board type - APP2.0,BNO USB stick
 */

static uint8_t app2_read_board_type()
{
    comm_intf_init_command_header(BOOT_MODE_GET, GETBOARDINFO_CMD);
    comm_intf_send_command(&coines_rsp_buffer);

    if (coines_rsp_buffer.buffer[13] == APP20_BOARD)
    {
        printf("\nAPP2.0 detected.\n\n");
        return APP20_BOARD;
    }
    else // (coines_rsp_buffer.buffer[13] == BNO_USB_STICK) /* Won't work because boot mode response is incorrect for BNO USB stick */
    {
        printf("\nBNO USB stick detected.\n\n");
        return BNO_USB_STICK;
    }
}

/*!
 *  @brief Read firmware file
 */

static void file_read_by_len(FILE *file_h, uint8_t *fw_data, uint32_t len)
{
    memset(fw_data, 0xFF, WRITE_SIZE);
    size_t result = fread(fw_data, WRITE_SIZE, 1, file_h);
    (void)result;
}

/*!
 *  @brief Get firmware file size
 */

static int get_file_size(FILE *file_h)
{
    int file_size,prev_pos;
    prev_pos = ftell(file_h);
    fseek(file_h, 0, SEEK_END);
    file_size = ftell(file_h);
    fseek(file_h, prev_pos, SEEK_SET);
    return file_size;
}

/*!
 *  @brief Calculate checksum byte
 */

static uint8_t get_checksum(uint8_t *fw_data, uint32_t len)
{
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < len; i++)
        checksum ^= fw_data[i];

    return checksum;
}

/**
 *
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 * BSD-3-Clause
 * Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    app30_eeprom.c
 * @date    Sep 25, 2018
 * @brief   This file contains read/write support for 1-wire EEPROM
 *
 */

/**********************************************************************************/
/* header files */
/**********************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include "ds28e05.h"

/**********************************************************************************/
/* functions */
/**********************************************************************************/

/*!
 *
 * @brief       : API to initiatialize APP3.0 EEPROM
 */

void app30_eeprom_init(void)
{
    ds28e05_Init();
}

/*!
 *
 * @brief       : API to read the internal ROM ID, and checks whether the read id is correct or not
 *
 */

bool app30_eeprom_romid(uint8_t *buffer)
{
    if (buffer != NULL)
    {

        return ds28e05_read_rom_id(buffer);
    }
    else
    {
        return false;
    }
}

/*!
 *
 * @brief       : API to read APP3.0 EEPROM
 */
bool app30_eeprom_read(uint16_t address, uint8_t *buffer, uint8_t length)
{
    if (buffer != NULL)
    {
        uint8_t i;

        if (ds28e05_reset() == 0)
        {

            return false;
        }

        /* Skip Rom command, sending this command is compulsory as per the 1-wire protocol */
        (void)ds28e05_writebyte(DS28E05_SKIP_ROM_CMD);

        (void)ds28e05_writebyte(DS28E05_READ_MEMORY_CMD);

        (void)ds28e05_writebyte((uint8_t)address);
        (void)ds28e05_writebyte(0x00);

        for (i = 0; i < length; i++)
        {
            buffer[i] = ds28e05_readbyte();
        }

        return true;
    }
    else
    {
        return false;
    }
}

/*!
 *
 * @brief       : API to write APP3.0 EEPROM
 */
bool app30_eeprom_write(uint8_t address, uint8_t *buffer, uint8_t length)
{
    if (buffer != NULL)
    {
        uint8_t databyte1=0, databyte2=0;
        uint8_t sf, ef;
        uint8_t is_command_success;
        uint8_t sb[2] = { 0 };
        uint8_t eb[2] = { 0 };
        uint8_t spage, epage, npage, wpage;
        uint8_t nseg, wseg = 0;
        uint8_t wBytes = 0, rBytes = 0, wAddr = 0;

        /* Calculate pages */
        spage = (address & DS28E05_PAGE_MASK) >> 4;
        epage = ((address + length) & DS28E05_PAGE_MASK) >> 4;
        if (epage == DS28E05_NUM_PAGES)
        {
            epage = DS28E05_NUM_PAGES - 1;
        }

        npage = epage - spage;

        /* This memory must be written respecting 16bits boundaries */
        sf = (address & 0x01) != 0;
        ef = ((address + length) & 0x01) != 0;

        if (ef)
        {
            (void)app30_eeprom_read(address + length, &eb[1], 1);
            eb[0] = buffer[length - 1];
            length++;
        }

        if (sf)
        {
            (void)app30_eeprom_read(address - 1, &sb[0], 1);
            sb[1] = buffer[0];
            length++;
            address--;
        }

        /* Write pages */
        for (wpage = 0; wpage <= npage; wpage++)
        {
            wAddr = address + wBytes;

            /* Calculate segments to write */
            if ((length - wBytes) > (DS28E05_BYTES_PER_PAGE))
            {
                /* Will we pass a page boundary */
                if (wAddr % (DS28E05_SEG_SIZE * DS28E05_BYTES_PER_SEG) == 0)
                {
                    nseg = DS28E05_SEG_SIZE;
                }
                else
                {
                    nseg = (DS28E05_BYTES_PER_PAGE - (wAddr % (DS28E05_BYTES_PER_PAGE))) >> 1;
                }
            }
            else
            {
                nseg = ((length - wBytes) & DS28E05_SEG_MASK) >> 1;
            }

            if (ds28e05_reset() == 0)
            {
                return false;
            }

            /* Skip Rom command, sending this command is compulsory as per the 1-wire protocol */
            (void)ds28e05_writebyte(DS28E05_SKIP_ROM_CMD);

            (void)ds28e05_writebyte(DS28E05_WRITE_MEMORY_CMD);
            (void)ds28e05_writebyte(wAddr);
            (void)ds28e05_writebyte(0xff);

            /* Write segments within page */
            for (wseg = 0; wseg < nseg; wseg++)
            {
                if (sf)
                {
                    (void)ds28e05_writebyte(sb[0]);
                    (void)ds28e05_writebyte(sb[1]);
                    wBytes += 2;
                    rBytes++;
                    sf = 0;
                }
                else if (ef && (length - wBytes) <= 2)
                {
                    (void)ds28e05_writebyte(eb[0]);
                    (void)ds28e05_writebyte(eb[1]);
                    wBytes += 2;
                    rBytes++;
                    ef = 0;
                }
                else
                {
                    (void)ds28e05_writebyte(buffer[rBytes]);
                    (void)ds28e05_writebyte(buffer[rBytes + 1]);
                    wBytes += 2;
                    rBytes += 2;
                }

                databyte1 = ds28e05_readbyte();
                databyte2 = ds28e05_readbyte();

                (void)ds28e05_writebyte(0xff);

                ds28e05_msDelay(DS28E05_tPROG);

                is_command_success = ds28e05_readbyte();

                if (is_command_success != DS28E05_COMMAND_SUCCESS)
                {
                    (void)ds28e05_reset();

                    return false;
                }
            }

            (void)ds28e05_reset();
        }

        (void)databyte1;
        (void)databyte2;

        return true;
    }
    else
    {
        return false;
    }
}

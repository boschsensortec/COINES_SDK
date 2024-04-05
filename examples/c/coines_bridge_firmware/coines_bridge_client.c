/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "coines_bridge_client.h"
#include "coines.h"

int8_t coines_process_packet(uint8_t *packet,
                             uint16_t packet_length,
                             uint8_t *resp,
                             uint16_t *resp_length,
                             struct coines_cbt *cbt)
{
    if ((cbt == NULL) || (packet == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    uint8_t cmd = packet[COINES_PROTO_CMD_POS];
    uint8_t *payload = &packet[COINES_PROTO_PAYLOAD_POS];
    uint16_t payload_length = packet_length - 4;

    if (cmd >= COINES_N_CMDS)
    {
        return COINES_E_NOT_SUPPORTED;
    }

    if (!cbt->cmd_callback[cmd])
    {
        return COINES_E_NULL_PTR;
    }

    return cbt->cmd_callback[cmd](cmd, payload, payload_length, resp, resp_length);
}

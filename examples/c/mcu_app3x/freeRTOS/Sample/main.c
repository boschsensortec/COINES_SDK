/**\
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Note: Parts of the code in this file are from GenAI GitHub Copilot​
 * 
 * Works for MCU_APP30 and MCU_APP31 targets
 **/

#include <stdio.h>
#include "coines.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// / Define the queue handle
QueueHandle_t xQueue;

// Task function prototypes
void sender_task(void *pvParameter);
void receiver_task(void *pvParameter);

int main(int argc, char **argv)
{
    // Initialize COINES
    int16_t coines_rslt;
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL);

    if (coines_rslt != COINES_SUCCESS)
    {
        printf("Failed to open communication interface\n");
        return -1;
    }

    // Create the queue
    xQueue = xQueueCreate(10, sizeof(int));

    if (xQueue == NULL)
    {
        printf("Failed to create queue\n");
        return -1;
    }

    // Create the sender and receiver tasks
    xTaskCreate(sender_task, "Sender Task", 256, NULL, 1, NULL);
    xTaskCreate(receiver_task, "Receiver Task", 256, NULL, 1, NULL);

    // Start the scheduler so the tasks start executing
    vTaskStartScheduler();

    // If all is well, the scheduler will now be running, and the following line will never be reached.
    for (;;)
        ;

    return 0;
}

void sender_task(void *pvParameter)
{
    int count = 0;

    while (1)
    {
        // Send the count value to the queue
        if (xQueueSend(xQueue, &count, portMAX_DELAY) == pdPASS)
        {
            printf("Sent: %d\n", count);
            count++;
        }

        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void receiver_task(void *pvParameter)
{
    int received_value;

    while (1)
    {
        // Receive the value from the queue
        if (xQueueReceive(xQueue, &received_value, portMAX_DELAY) == pdPASS)
        {
            printf("Received: %d\n", received_value);
        }
    }
}

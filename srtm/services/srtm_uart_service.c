/*
 * Copyright 2022-2023 NXP
 *
 * Copyright (c) 2025 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>

#include "srtm_heap.h"
#include "srtm_dispatcher.h"
#include "fsl_lpuart.h"
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "srtm/srtm_uart_service.h"
#include "srtm_message.h"
#include "srtm/srtm_rpmsg_endpoint.h"
#include "srtm_channel.h"
#include "board/board_uart.h"
#include "task.h"

static srtm_uart_adapter_t pAdapter;

struct srtm_uart_chan srtm_uart_channels[SRTM_UART_ENDPOINT_MAX_NUM] = {0};

void uart_rx_task(void *param)
{
    struct uart_iface *iface = (struct uart_iface *)param;
    srtm_uart_adapter_t adapter = pAdapter;
    struct uart_adapter *uart_adapter = adapter->uart_adapter;
    struct srtm_uart_chan *uart_chan = NULL;
    size_t len = 0;
    uint8_t data;
    uint8_t buffer[BOARD_UART_BUFFER_SIZE] = {0};
    int i;

    /* get uart channel */
    for (i = 0; i < SRTM_UART_ENDPOINT_MAX_NUM; i++) {
        if (srtm_uart_channels[i].chan && srtm_uart_channels[i].bus_id == iface->id){
            uart_chan = &srtm_uart_channels[i];
            break;
        }
    }

    if (!uart_chan)
        goto exit;

    while (uart_chan->read_enable)
    {
        size_t byte = sizeof(uint8_t);
        status_t status = uart_adapter->ops.read(adapter->uart_adapter, iface, &data, &byte);
        if (status != kStatus_Success)
            continue;

        buffer[len++] = data;

        if (len >= BOARD_UART_BUFFER_SIZE || data == '\n' || data == '\r')
        {
            adapter->sendNotify(adapter->service, uart_chan, TTY_RPMSG_COMMAND_READ, buffer, len);

            len = 0;
            memset(buffer, 0, BOARD_UART_BUFFER_SIZE);
        }

    }

    /* Send rest to Linux */
    if(len > 0){
        adapter->sendNotify(adapter->service, uart_chan, TTY_RPMSG_COMMAND_READ, buffer, len);
    }

exit:
    vTaskDelete(NULL);
}

srtm_status_t SRTM_UartService_ReceiveRequest(srtm_service_t service, srtm_request_t request)
{
    srtm_uart_service_t handle = (srtm_uart_service_t)(void *)service;
    srtm_uart_adapter_t adapter = handle->adapter;
    struct uart_adapter *uart_adapter = adapter->uart_adapter;
    srtm_channel_t channel = SRTM_CommMessage_GetChannel(request);
    uint8_t command    = SRTM_CommMessage_GetCommand(request);
    struct _srtm_uart_payload *uartReq = (void *)SRTM_CommMessage_GetPayload(request);
    struct uart_iface *iface = NULL;
    status_t ret;
    int i;

    if (uartReq == NULL)
    {
        return SRTM_Status_Error;
    }


    for (i = 0; i < SRTM_UART_ENDPOINT_MAX_NUM; i++)
    {
        if (srtm_uart_channels[i].chan == NULL)
        {
            srtm_uart_channels[i].chan = channel;
            srtm_uart_channels[i].bus_id = uartReq->bus_id;
            iface = uart_adapter->ops.get_iface_from_idx(uart_adapter, uartReq->bus_id);
            break;
        }
        else if (srtm_uart_channels[i].bus_id == uartReq->bus_id)
        {
            iface = uart_adapter->ops.get_iface_from_idx(uart_adapter, uartReq->bus_id);
            break;
        }
    }

    if(!iface || i >= SRTM_UART_ENDPOINT_MAX_NUM)
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_ERROR, "%s: iface not found for busId %d\r\n", __func__, uartReq->bus_id);
        return SRTM_Status_Error;
    }

        switch (command)
        {
            case TTY_RPMSG_COMMAND_WRITE:
                while (!uart_adapter->ops.is_transfer_complete(uart_adapter, iface))
                    vTaskDelay(pdMS_TO_TICKS(10));
                ret = uart_adapter->ops.write(uart_adapter, iface, uartReq->buf, uartReq->len);
                if (ret != kStatus_Success)
                    return SRTM_Status_Error;
                adapter->sendNotify(adapter->service, &srtm_uart_channels[i], TTY_RPMSG_COMMAND_WRITE, NULL, 0);
                break;
            case TTY_RPMSG_COMMAND_READ:
                srtm_uart_channels[i].read_enable = true;
                xTaskCreate(uart_rx_task, "UART RX", 1024, iface, 3, NULL);
                break;
            case TTY_RPMSG_COMMAND_STOP:
                srtm_uart_channels[i].read_enable = false;
                break;
            case TTY_RPMSG_COMMAND_SET_BAUD:
                iface->config.baudRate_Bps = uartReq->baudrate;
                iface->config.enableRxRTS = uartReq->ctrl ? true : false;
                iface->config.enableTxCTS = uartReq->ctrl ? true : false;
                uart_adapter->ops.deinit(uart_adapter, iface);
                uart_adapter->ops.init(uart_adapter, iface);
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s: set baudrate to %d bps\r\n", __func__, uartReq->baudrate);
                break;
            case TTY_RPMSG_COMMAND_INIT:
                /* Make sure to set new channel after reboot*/
                srtm_uart_channels[i].chan = channel;
                uart_adapter->ops.init(uart_adapter, iface);
                adapter->sendNotify(adapter->service, &srtm_uart_channels[i], TTY_RPMSG_COMMAND_INIT, NULL, 0);
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s: init uart iface %d\r\n", __func__, uartReq->bus_id);
                break;
            case TTY_RPMSG_COMMAND_DEINIT:
                uart_adapter->ops.deinit(uart_adapter, iface);
                adapter->sendNotify(adapter->service, &srtm_uart_channels[i], TTY_RPMSG_COMMAND_DEINIT, NULL, 0);
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s: deinit uart iface %d\r\n", __func__, uartReq->bus_id);
                break;
            default:
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
                return SRTM_Status_ServiceNotFound;
        }

    return SRTM_Status_Success;
}

srtm_status_t SRTM_UartService_SendNotify(srtm_service_t service, struct srtm_uart_chan *uart_chan, uint8_t command, uint8_t *data, uint16_t data_len)
{
    srtm_uart_service_t handle = (srtm_uart_service_t)service;
    srtm_notification_t notif;
    struct _srtm_uart_payload *payload;
    uint16_t payload_size = sizeof(struct _srtm_uart_payload);

    notif = SRTM_Notification_Create(uart_chan->chan, SRTM_UART_CATEGORY, SRTM_UART_VERSION, command, payload_size);
    if (notif == NULL)
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_ERROR, "%s: alloc notification failed.\r\n", __func__);
        return SRTM_Status_OutOfMemory;
    }

    payload        = (struct _srtm_uart_payload *)(void *)SRTM_CommMessage_GetPayload(notif);
    payload->bus_id  = uart_chan->bus_id;
    payload->len = data_len;
    memcpy(payload->buf, data, data_len);
    SRTM_Dispatcher_DeliverNotification(handle->service.dispatcher, notif);

    return SRTM_Status_Success;
}

void SRTM_UartService_Destroy(srtm_service_t service)
{
    srtm_uart_service_t handle = (srtm_uart_service_t)service;
    assert(service);

    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}

srtm_service_t SRTM_UartService_Create(srtm_uart_adapter_t adapter)
{
    srtm_uart_service_t handle;
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = SRTM_Heap_Malloc(sizeof(struct _srtm_uart_service));
    if(!handle)
        return NULL;
    adapter->service = &handle->service;
    adapter->sendNotify = SRTM_UartService_SendNotify;
    handle->adapter = adapter;
    pAdapter = adapter;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_UART_CATEGORY;
    handle->service.destroy    = SRTM_UartService_Destroy;
    handle->service.request    = SRTM_UartService_ReceiveRequest;

    return SRTM_Status_Success;
}

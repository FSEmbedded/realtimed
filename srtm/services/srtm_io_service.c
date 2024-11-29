/*
 * Copyright 2017, NXP
 * All rights reserved.
 *
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "srtm_heap.h"
#include "srtm_dispatcher.h"
#include "srtm_peercore.h"
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "srtm_channel.h"
#include "srtm_channel_struct.h"
#include <srtm/srtm_io_service.h>
#include "srtm_message.h"
#include "srtm_message_struct.h"

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_IO_CATEGORY (0x5U)

#define SRTM_IO_VERSION (0x0100U)

#define SRTM_IO_RETURN_CODE_SUCEESS     (0x0U)
#define SRTM_IO_RETURN_CODE_FAIL        (0x1U)
#define SRTM_IO_RETURN_CODE_UNSUPPORTED (0x2U)

#define SRTM_IO_CMD_CONF_INPUT_EVENT (0x00U)
#define SRTM_IO_CMD_SET_OUTPUT       (0x01U)
#define SRTM_IO_CMD_GET_INPUT        (0x02U)

#define SRTM_IO_NTF_INPUT_EVENT (0x00U)

/* IO pin list node */
struct srtm_io_pin
{
    srtm_list_t node;
    srtm_channel_t channel;
    srtm_notification_t notif; /* SRTM notification message for input event */
    uint8_t pinID;
	uint8_t ifaceID;
    void *param;
};

/* Service handle */
struct srtm_io_service
{
    struct _srtm_service service;
    struct srtm_io_adapter *adapter;
    srtm_list_t pins;
};

static void SRTM_IoService_FreePin(struct srtm_io_pin *pin)
{
    SRTM_Notification_Destroy(pin->notif);
    SRTM_Heap_Free(pin);
}

static void SRTM_IoService_RecycleMessage(srtm_message_t msg, void *param)
{
    uint32_t primask;
    struct srtm_io_pin *pin = param;

    assert(pin);
    assert(pin->notif == NULL);

    primask = DisableGlobalIRQ();
    /* Return msg to pin */
    pin->notif = msg;
    EnableGlobalIRQ(primask);
}

static struct srtm_io_pin *SRTM_IoService_FindPin(struct srtm_io_service *handle, uint8_t ifaceID, uint8_t pinID, bool rm, bool notify)
{
    struct srtm_io_pin *srtm_pin = NULL;
    srtm_list_t *list;
    srtm_notification_t notif = NULL;
    uint32_t primask;

    primask = DisableGlobalIRQ();
    for (list = handle->pins.next; list != &handle->pins; list = list->next)
    {
        srtm_pin = SRTM_LIST_OBJ(struct srtm_io_pin *, node, list);
        if (srtm_pin->ifaceID == ifaceID && srtm_pin->pinID == pinID)
        {
            if (rm)
            {
                SRTM_List_Remove(list);
            }
            if (notify)
            {
                notif = srtm_pin->notif;
                /* If channel is destoryed, the notif of pin should be recycled */
                if (srtm_pin->channel)
                {
                    srtm_pin->notif = NULL;
                }
            }
            break;
        }
    }
    EnableGlobalIRQ(primask);

    if (notify && notif && srtm_pin->channel)
    {
        /* If notification message exists, just deliver it. Otherwise it's on the way, no need
           to deliver again. */
        notif->channel = srtm_pin->channel;
        SRTM_Dispatcher_DeliverNotification(handle->service.dispatcher, notif);
    }

    return list == &handle->pins ? NULL : srtm_pin;
}

/* Both request and notify are called from SRTM dispatcher context */
static srtm_status_t SRTM_IoService_Request(srtm_service_t service, srtm_request_t request)
{
    struct srtm_io_service *handle = (void *)service;
    struct io_adapter *io_adapter = handle->adapter->io_adapter;
    struct srtm_io_pin *srtm_pin;
    struct gpio_rpmsg_data *payload;
    srtm_channel_t channel;
    uint8_t command, retCode;
    srtm_response_t response;
    enum io_value value = IO_ValueLow;
    uint32_t len;
    uint8_t ifaceID = 0xff;
    uint8_t pinID = 0xff;
    srtm_status_t status;

    assert(service->dispatcher);

    channel = SRTM_CommMessage_GetChannel(request);
    assert(channel);
    command = SRTM_CommMessage_GetCommand(request);
    payload = (void *)SRTM_CommMessage_GetPayload(request);
    len     = SRTM_CommMessage_GetPayloadLen(request);

    status = SRTM_Service_CheckVersion(service, request, SRTM_IO_VERSION);
    if ((status != SRTM_Status_Success) || (payload == NULL) || (len < 2U))
    {
        /* Either version mismatch or empty payload is not supported */
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: format error, len %d!\r\n", __func__, len);
        retCode = SRTM_IO_RETURN_CODE_UNSUPPORTED;
    }
    else
    {
        ifaceID = payload->port_idx;
        pinID = payload->pin_idx;
        srtm_pin  = SRTM_IoService_FindPin(handle, ifaceID, pinID, false, false);
        if (!srtm_pin)
        {
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Pin 0x%x on IFace 0x%x not registered!\r\n", __func__, pinID, ifaceID);
            retCode = SRTM_IO_RETURN_CODE_FAIL;
        }
        else
        {
            /* Record pin's channel for further input event */
            srtm_pin->channel = channel;
            switch (command)
            {
                case SRTM_IO_CMD_CONF_INPUT_EVENT:
                    if ((len >= 4U) && io_adapter->ops.confIRQEvent)
                    {
                        status = io_adapter->ops.confIRQEvent(io_adapter, ifaceID, pinID,
                                                (enum io_event)(payload->out.event),
                                                (bool)(payload->in.wakeup));
                        retCode =
                            status == SRTM_Status_Success ? SRTM_IO_RETURN_CODE_SUCEESS : SRTM_IO_RETURN_CODE_FAIL;
                    }
                    else
                    {
                        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN,
                                           "%s: Command ConfInputEvent not allowed or len %d error!\r\n", __func__,
                                           len);
                        retCode = SRTM_IO_RETURN_CODE_FAIL;
                    }
                    break;
                case SRTM_IO_CMD_SET_OUTPUT:
                    if ((len >= 3U) && (io_adapter->ops.set_output != NULL))
                    {
                        status = io_adapter->ops.set_output(io_adapter, ifaceID, pinID, (enum io_value)(payload->out.value));
                        retCode =
                            status == SRTM_Status_Success ? SRTM_IO_RETURN_CODE_SUCEESS : SRTM_IO_RETURN_CODE_FAIL;
                    }
                    else
                    {
                        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN,
                                           "%s: Command SetOutput not allowed or len %d error!\r\n", __func__, len);
                        retCode = SRTM_IO_RETURN_CODE_FAIL;
                    }
                    break;
                case SRTM_IO_CMD_GET_INPUT:
                    if (io_adapter->ops.get_input)
                    {
                        status = io_adapter->ops.get_input(io_adapter, ifaceID, pinID, &value);
                        retCode =
                            status == SRTM_Status_Success ? SRTM_IO_RETURN_CODE_SUCEESS : SRTM_IO_RETURN_CODE_FAIL;
                    }
                    else
                    {
                        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command %d function not registered!\r\n",
                                           __func__, SRTM_IO_CMD_GET_INPUT);
                        retCode = SRTM_IO_RETURN_CODE_FAIL;
                    }
                    break;
                default:
                    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
                    retCode = SRTM_IO_RETURN_CODE_UNSUPPORTED;
                    break;
            }
        }
    }

    response = SRTM_Response_Create(channel, SRTM_IO_CATEGORY, SRTM_IO_VERSION, command, 4U);
    if (!response)
    {
        return SRTM_Status_OutOfMemory;
    }
    payload = (void *)SRTM_CommMessage_GetPayload(response);
    payload->pin_idx = pinID;
    payload->port_idx = ifaceID;
    payload->out.retcode = retCode;
    payload->in.value = (uint8_t)value;

    /* Now the response is ready */
    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_IoService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}

srtm_status_t SRTM_IoService_Create(struct srtm_io_adapter *adapter)
{
    struct srtm_io_service *handle;

    handle = SRTM_Heap_Malloc(sizeof(struct srtm_io_service));
    if(!handle)
        return SRTM_Status_OutOfMemory;

    adapter->service = &handle->service;
    handle->adapter = adapter;

    SRTM_List_Init(&handle->pins);

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_IO_CATEGORY;
    handle->service.destroy    = SRTM_IoService_Destroy;
    handle->service.request    = SRTM_IoService_Request;
    handle->service.notify     = SRTM_IoService_Notify;
    return SRTM_Status_Success;
}

void SRTM_IoService_Destroy(srtm_service_t service)
{
    struct srtm_io_service *handle = (void *)service;
    srtm_list_t *list;
    struct srtm_io_pin *pin;

    assert(service);
    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    while (!SRTM_List_IsEmpty(&handle->pins))
    {
        list = handle->pins.next;
        SRTM_List_Remove(list);
        pin = SRTM_LIST_OBJ(struct srtm_io_pin *, node, list);
        SRTM_IoService_FreePin(pin);
    }

    SRTM_Heap_Free(handle);
}

void SRTM_IoService_Reset(srtm_service_t service, srtm_peercore_t core)
{
    struct srtm_io_service *handle = (void *)service;
    srtm_list_t *list;
    struct srtm_io_pin *pin;

    assert(service);

    /* Currently assume just one peer core, need to update all pins. */
    for (list = handle->pins.next; list != &handle->pins; list = list->next)
    {
        pin          = SRTM_LIST_OBJ(struct srtm_io_pin *, node, list);
        pin->channel = NULL;
    }
}

srtm_status_t SRTM_IoService_RegisterPin(srtm_service_t service,
                                         uint8_t ifaceID,
                                         uint8_t pinID,
                                         void *param)
{
    struct srtm_io_service *handle = (void *)service;
    struct srtm_io_pin *srtm_pin;
    struct gpio_rpmsg_data *payload;
    uint32_t primask;

    assert(service);
    /* Pin must be registered before service registration. */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    srtm_pin = SRTM_IoService_FindPin(handle, ifaceID, pinID, false, false);
    if (srtm_pin)
    {
        /* pin is already registered */
        return SRTM_Status_InvalidParameter;
    }

    srtm_pin = SRTM_Heap_Malloc(sizeof(struct srtm_io_pin));
    if (srtm_pin == NULL)
    {
        return SRTM_Status_OutOfMemory;
    }

    srtm_pin->ifaceID   = ifaceID;
    srtm_pin->pinID    = pinID;
    srtm_pin->param      = param;
    srtm_pin->notif      = SRTM_Notification_Create(NULL, SRTM_IO_CATEGORY, SRTM_IO_VERSION, SRTM_IO_NTF_INPUT_EVENT, 2U);
    assert(srtm_pin->notif);
    SRTM_Message_SetFreeFunc(srtm_pin->notif, SRTM_IoService_RecycleMessage, srtm_pin);
    payload = (void *)SRTM_CommMessage_GetPayload(srtm_pin->notif);
    /* Little endian IO pin ID */
    payload->pin_idx = pinID;
    payload->port_idx = ifaceID;

    primask = DisableGlobalIRQ();
    SRTM_List_AddTail(&handle->pins, &srtm_pin->node);
    EnableGlobalIRQ(primask);

    return SRTM_Status_Success;
}

srtm_status_t SRTM_IoService_UnregisterPin(srtm_service_t service, uint8_t ifaceID, uint8_t pinID)
{
    struct srtm_io_service *handle = (void *)service;
    struct srtm_io_pin *pin;

    assert(service);
    /* Pin must be unregistered when service is not running. */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    pin = SRTM_IoService_FindPin(handle, ifaceID, pinID, true, false);
    if (pin)
    {
        SRTM_IoService_FreePin(pin);
    }
    else
    {
        /* Not found */
        return SRTM_Status_ListRemoveFailed;
    }

    return SRTM_Status_Success;
}

/* Called in ISR */
srtm_status_t SRTM_IoService_NotifyInputEvent(srtm_service_t service, uint8_t ifaceID, uint8_t pinID)
{
    struct srtm_io_service *handle = (void *)service;
    struct srtm_io_pin *pin;

    assert(service);
    /* Service must be running in dispatcher when notifying input event */
    assert(!SRTM_List_IsEmpty(&service->node));

    pin = SRTM_IoService_FindPin(handle, ifaceID, pinID, false, true);
    if (!pin)
    {
        /* Pin not registered */
        return SRTM_Status_InvalidParameter;
    }

    return SRTM_Status_Success;
}

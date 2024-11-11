/*
 * Copyright 2018-2021, NXP
 * All rights reserved.
 *
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <assert.h>
#include <string.h>
#include <board/board.h>

#include "fsl_common.h"

#include "srtm_heap.h"
#include "srtm_list.h"
#include "srtm_dispatcher.h"
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include <srtm/srtm_i2c_service.h>
#include "srtm_message.h"
#include "srtm_message_struct.h"
#include "srtm_channel.h"
#include "srtm_channel_struct.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_I2C_CATEGORY (0x9U)
#define SRTM_I2C_VERSION  (0x0100U)

#define SRTM_I2C_RETURN_CODE_SUCEESS     (0x0U)
#define SRTM_I2C_RETURN_CODE_FAIL        (0x1U)
#define SRTM_I2C_RETURN_CODE_UNSUPPORTED (0x2U)

struct srtm_i2c_service
{
    struct _srtm_service service;
    struct srtm_i2c_adapter *adapter;
};

static srtm_status_t SRTM_I2CService_Request(srtm_service_t service, srtm_request_t request);
static srtm_status_t SRTM_I2CService_Notify(srtm_service_t service, srtm_notification_t notif);

static srtm_status_t SRTM_I2CService_ReadBus(
    srtm_service_t service, uint8_t busID, uint16_t slaveAddr, uint8_t *buf, uint16_t len, uint16_t srtm_flags)
{
    struct srtm_i2c_service *handle  = (struct srtm_i2c_service *)service;
    struct srtm_i2c_adapter *adapter = handle->adapter;
    struct i2c_adapter *i2c_adapter = adapter->i2c_adapter;
    struct i2c_bus *targetBus;
    uint8_t switch_index;
    uint16_t switch_addr;
    enum i2c_switch_channel switch_channel;
    srtm_status_t status;
    struct i2c_switch *switch_inst;
    uint16_t i2c_flags;

    targetBus    = i2c_adapter->ops.get_bus_from_idx(i2c_adapter, busID);
    switch_index = targetBus->switch_idx;

    /* Switch Channel */
    if (switch_index < i2c_adapter->num_switches){
        switch_inst    = &i2c_adapter->switches[switch_index];
        switch_addr    = switch_inst->slaveAddr;
        switch_channel = targetBus->switch_channel;

        if (switch_inst->cur_channel != switch_channel)
        {
            (void)i2c_adapter->ops.switchchannel(i2c_adapter, busID, switch_addr, switch_channel);
            switch_inst->cur_channel = switch_channel;
        }
    }

    i2c_flags = I2C_TransferDefaultFlag;
    i2c_flags |= (srtm_flags & SRTM_I2C_FLAG_NEED_STOP) ? 0 : I2C_TransferNoStopFlag;

    /* Read */
    status = i2c_adapter->ops.read(i2c_adapter, busID, slaveAddr, 0, 0, buf, len, i2c_flags);
    return status;
}

static srtm_status_t SRTM_I2CService_WriteBus(
    srtm_service_t service, uint8_t busID, uint16_t slaveAddr, uint8_t *buf, uint16_t len, uint16_t srtm_flags)
{
    struct srtm_i2c_service *handle  = (struct srtm_i2c_service *)service;
    struct srtm_i2c_adapter *adapter = handle->adapter;
    struct i2c_adapter *i2c_adapter = adapter->i2c_adapter;
    struct i2c_bus *targetBus;
    uint8_t switch_index;
    uint16_t switch_addr;
    enum i2c_switch_channel switch_channel;
    srtm_status_t status;
    struct i2c_switch *switch_inst;
    uint16_t i2c_flags;

    targetBus    = i2c_adapter->ops.get_bus_from_idx(i2c_adapter, busID);
    switch_index = targetBus->switch_idx;

    /* Switch Channel */
    if (switch_index < i2c_adapter->num_switches)
    {
        switch_inst    = &i2c_adapter->switches[switch_index];
        switch_addr    = switch_inst->slaveAddr;
        switch_channel = targetBus->switch_channel;
        if (switch_inst->cur_channel != switch_channel)
        {
            (void)i2c_adapter->ops.switchchannel(i2c_adapter, busID, switch_addr, switch_channel);
            switch_inst->cur_channel = switch_channel;
        }
    }

    i2c_flags = I2C_TransferDefaultFlag;
    i2c_flags |= (srtm_flags & SRTM_I2C_FLAG_NEED_STOP) ? 0 : I2C_TransferNoStopFlag;

    /* Write */
    status = i2c_adapter->ops.write(i2c_adapter, busID, slaveAddr, 0, 0, buf, len, i2c_flags);
    return status;
}

srtm_status_t SRTM_I2CService_Create(struct srtm_i2c_adapter *adapter)
{
    struct srtm_i2c_service *handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);
    handle = SRTM_Heap_Malloc(sizeof(struct srtm_i2c_service));
    if(!handle)
        return SRTM_Status_OutOfMemory;

    adapter->service = &handle->service;
    handle->adapter  = adapter;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_I2C_CATEGORY;
    handle->service.destroy    = SRTM_I2CService_Destroy;
    handle->service.request    = SRTM_I2CService_Request;
    handle->service.notify     = SRTM_I2CService_Notify;
    return SRTM_Status_Success;
}

void SRTM_I2CService_Destroy(srtm_service_t service)
{
    struct srtm_i2c_service *handle = (struct srtm_i2c_service *)service;

    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}

void SRTM_I2CService_Reset(srtm_service_t service, srtm_peercore_t core)
{
    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);
}

static srtm_status_t SRTM_I2CService_Request(srtm_service_t service, srtm_request_t request)
{
    srtm_status_t status;
    srtm_channel_t channel;
    uint8_t command;
    uint32_t payloadLen;
    srtm_response_t response;
    struct srtm_i2c_payload *i2cReq;
    struct srtm_i2c_payload *i2cResp;

    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel    = SRTM_CommMessage_GetChannel(request);
    command    = SRTM_CommMessage_GetCommand(request);
    i2cReq     = (void *)SRTM_CommMessage_GetPayload(request);
    payloadLen = SRTM_CommMessage_GetPayloadLen(request);
    (void)payloadLen; /* try to fix warning: variable 'payloadLen' set but not used */
    assert(i2cReq);
    assert((uint32_t)(i2cReq->len + sizeof(struct srtm_i2c_payload) - sizeof(i2cReq->data[0])) <= payloadLen);

    response =
        SRTM_Response_Create(channel, SRTM_I2C_CATEGORY, SRTM_I2C_VERSION, command,
                             (uint16_t)((sizeof(struct srtm_i2c_payload) - sizeof(i2cReq->data[0])) + i2cReq->len));
    if (response == NULL)
    {
        return SRTM_Status_OutOfMemory;
    }

    i2cResp = (struct srtm_i2c_payload *)(void *)SRTM_CommMessage_GetPayload(response);

    status = SRTM_Service_CheckVersion(service, request, SRTM_I2C_VERSION);
    if (status != SRTM_Status_Success)
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: format error!\r\n", __func__);
        i2cResp->retCode = SRTM_I2C_RETURN_CODE_UNSUPPORTED;
    }
    else
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO,
                           "SRTM receive I2C request:cmd=%x, busID %d, slaveAddr 0x%x!, data %d bytes\r\n", command,
                           i2cReq->busID, i2cReq->slaveAddr, i2cReq->len);
        (void)memcpy(i2cResp, i2cReq,
                     (sizeof(struct srtm_i2c_payload) - sizeof(i2cReq->data[0]) + (size_t)i2cReq->len));

        switch (command)
        {
            case (uint8_t)SRTM_I2C_CMD_READ:
                status = SRTM_I2CService_ReadBus(service, i2cResp->busID, i2cResp->slaveAddr, i2cResp->data,
                                                 i2cReq->len, i2cReq->flags);
                i2cResp->retCode =
                    status == SRTM_Status_Success ? SRTM_I2C_RETURN_CODE_SUCEESS : SRTM_I2C_RETURN_CODE_FAIL;
                break;
            case (uint8_t)SRTM_I2C_CMD_WRITE:
                status = SRTM_I2CService_WriteBus(service, i2cResp->busID, i2cResp->slaveAddr, i2cResp->data,
                                                  i2cReq->len, i2cReq->flags);
                i2cResp->retCode =
                    status == SRTM_Status_Success ? SRTM_I2C_RETURN_CODE_SUCEESS : SRTM_I2C_RETURN_CODE_FAIL;
                break;
            default:
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
                assert(false);
                break;
        }
    }

    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_I2CService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}

static void SRTM_I2C_HandleBusRead(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    srtm_status_t status;
    struct srtm_i2c_payload *i2c_payload = (struct srtm_i2c_payload *)param1;
    struct _srtm_service *service         = (struct _srtm_service *)param2;
    status = SRTM_I2CService_ReadBus(service, i2c_payload->busID, i2c_payload->slaveAddr, i2c_payload->data,
                                     (uint8_t)i2c_payload->len, i2c_payload->flags);
    i2c_payload->retCode = (status == SRTM_Status_Success) ? SRTM_I2C_RETURN_CODE_SUCEESS : SRTM_I2C_RETURN_CODE_FAIL;
}

static void SRTM_I2C_HandleBusWrite(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    srtm_status_t status;
    struct srtm_i2c_payload *i2c_payload = (struct srtm_i2c_payload *)param1;
    struct _srtm_service *service         = (struct _srtm_service *)param2;
    status = SRTM_I2CService_WriteBus(service, i2c_payload->busID, i2c_payload->slaveAddr, i2c_payload->data,
                                      (uint8_t)i2c_payload->len, i2c_payload->flags);
    i2c_payload->retCode = (status == SRTM_Status_Success) ? SRTM_I2C_RETURN_CODE_SUCEESS : SRTM_I2C_RETURN_CODE_FAIL;
}

srtm_status_t SRTM_I2C_RequestBusRead(
    srtm_service_t service, uint8_t busID, uint16_t slaveAddr, uint8_t *buf, uint16_t len)
{
    srtm_request_t request;
    srtm_status_t status;
    struct srtm_i2c_payload *i2cReq;
    srtm_procedure_t proc;
    /*
     * Allocate an SRTM message and copy necessary information
     */
    request           = SRTM_Request_Create(NULL, SRTM_I2C_CATEGORY, SRTM_I2C_VERSION, (uint8_t)SRTM_I2C_CMD_READ,
                                            (uint16_t)((sizeof(struct srtm_i2c_payload) - sizeof(uint8_t)) + len));
    i2cReq            = (struct srtm_i2c_payload *)(void *)SRTM_CommMessage_GetPayload(request);
    i2cReq->busID     = busID;
    i2cReq->slaveAddr = slaveAddr;
    i2cReq->len       = len;
    (void)memset(i2cReq->data, 0, len);
    /*
     * Call proc in sync manner
     */
    proc = SRTM_Procedure_Create(SRTM_I2C_HandleBusRead, i2cReq, service);
    (void)SRTM_Dispatcher_CallProc(service->dispatcher, proc, SRTM_WAIT_FOR_EVER); /*synchronized call*/
    /*
     * Save proc exec result
     */
    (void)memcpy(buf, i2cReq->data, len);
    status = (srtm_status_t)i2cReq->retCode;
    /*
     * Clean the allocated SRTM object
     */
    SRTM_Procedure_Destroy(proc);
    SRTM_Response_Destroy(request);

    return status;
}

srtm_status_t SRTM_I2C_RequestBusWrite(
    srtm_service_t service, uint8_t busID, uint16_t slaveAddr, uint8_t *buf, uint16_t len, uint8_t needStop)
{
    srtm_request_t request;
    srtm_status_t status;
    struct srtm_i2c_payload *i2cReq;
    srtm_procedure_t proc;
    /*
     * Allocate an SRTM message and copy necessary information
     */
    request           = SRTM_Request_Create(NULL, SRTM_I2C_CATEGORY, SRTM_I2C_VERSION, (uint8_t)SRTM_I2C_CMD_WRITE,
                                            (uint16_t)((sizeof(struct srtm_i2c_payload) - sizeof(uint8_t)) + len));
    i2cReq            = (void *)SRTM_CommMessage_GetPayload(request);
    i2cReq->busID     = busID;
    i2cReq->slaveAddr = slaveAddr;
    i2cReq->len       = len;
    i2cReq->flags     = needStop > 0U ? (SRTM_I2C_FLAG_NEED_STOP) : 0U;

    (void)memcpy(i2cReq->data, buf, len);
    /*
     * Call proc in sync manner
     */
    proc = SRTM_Procedure_Create(SRTM_I2C_HandleBusWrite, i2cReq, service);
    (void)SRTM_Dispatcher_CallProc(service->dispatcher, proc, SRTM_WAIT_FOR_EVER); /*synchronized call*/
    /*
     * Save proc exec result
     */
    status = (srtm_status_t)i2cReq->retCode;
    /*
     * Clean the allocated SRTM object
     */
    SRTM_Procedure_Destroy(proc);
    SRTM_Response_Destroy(request);

    return status;
}

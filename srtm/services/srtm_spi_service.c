#include <assert.h>
#include <string.h>
#include <stdio.h>

#include "srtm_heap.h"
#include "srtm_dispatcher.h"
#include "fsl_lpspi.h"
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "srtm/srtm_spi_service.h"
#include "srtm_message.h"
#include "srtm/srtm_rpmsg_endpoint.h"
#include "srtm_channel.h"
#include "board/board_spi.h"

struct srtm_spi_chan srtm_spi_channels[SRTM_SPI_ENDPOINT_MAX_NUM] = {0};

srtm_status_t SRTM_SpiService_HandleRequest(srtm_service_t service, srtm_request_t msg)
{
    struct _srtm_spi_service *handle = (struct _srtm_spi_service *)service;
    srtm_spi_adapter_t spi_adapter = handle->adapter;
    struct spi_adapter *adapter = spi_adapter->spi_adapter;
    srtm_channel_t channel = SRTM_CommMessage_GetChannel(msg);
    uint8_t command = SRTM_CommMessage_GetCommand(msg);
    struct spi_iface *iface = NULL;
    struct srtm_spi_payload *spiReq = (void *)SRTM_CommMessage_GetPayload(msg);
    uint8_t bus_id, ret;
    int i;

    spiReq = (struct srtm_spi_payload *)(void *)SRTM_CommMessage_GetPayload(msg);
    bus_id = spiReq->busID;

    for (i = 0; i < SRTM_SPI_ENDPOINT_MAX_NUM; i++)
    {
        if (srtm_spi_channels[i].chan == NULL)
        {
            srtm_spi_channels[i].chan = channel;
            srtm_spi_channels[i].bus_id = bus_id;
            iface = adapter->ops.get_iface_from_idx(adapter, spiReq->busID);
            break;
        }
        else if (srtm_spi_channels[i].bus_id == bus_id)
        {
            iface = adapter->ops.get_iface_from_idx(adapter, spiReq->busID);
            break;
        }
    }

    if(!iface){
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: can't find spi iface %d\r\n", __func__, bus_id);
        return SRTM_Status_ServiceNotFound;
    }

    switch(command)
    {
            case SPI_RPMSG_COMMAND_TRANSFER: {
                uint16_t len = spiReq->len;
                if (len > BOARD_SPI_MAX_BUFFER_SIZE) len = BOARD_SPI_MAX_BUFFER_SIZE;

                static uint8_t rxbuf[BOARD_SPI_MAX_BUFFER_SIZE];

                ret = adapter->ops.transfer(adapter, iface, spiReq->buf, rxbuf, len, spiReq->flags);

                if (ret != kStatus_Success) {
                    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Transfer FAILED, sending error to Linux\r\n", __func__);
                    spi_adapter->sendNotify(spi_adapter->service, &srtm_spi_channels[i], SPI_RPMSG_COMMAND_TRANSFER, NULL, 0);
                } else {
                    spi_adapter->sendNotify(spi_adapter->service, &srtm_spi_channels[i], SPI_RPMSG_COMMAND_TRANSFER, rxbuf, len);
                }
                break;
            }
            default:
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
                return SRTM_Status_ServiceNotFound;
        }

    return SRTM_Status_Success;
}

srtm_status_t SRTM_SpiService_SendNotify(srtm_service_t service, struct srtm_spi_chan *spi_chan, uint8_t command, const uint8_t *data, uint32_t len)
{
    struct _srtm_spi_service *handle = (struct _srtm_spi_service *)service;
    srtm_notification_t notif;
    struct srtm_spi_payload *payload;
    uint16_t payload_size = offsetof(struct srtm_spi_payload, buf) + len;

    notif = SRTM_Notification_Create(spi_chan->chan, SRTM_SPI_CATEGORY, SRTM_SPI_VERSION, command, payload_size);
    if (notif == NULL)
        return SRTM_Status_OutOfMemory;

    payload = (struct srtm_spi_payload *)(void *)SRTM_CommMessage_GetPayload(notif);
    payload->busID = spi_chan->bus_id;
    payload->slaveAddr = 0;
    payload->flags     = 0;
    payload->len = len;

    if (data && len > 0)
        memcpy(payload->buf, data, len);

    srtm_status_t st = SRTM_Dispatcher_DeliverNotification(handle->service.dispatcher, notif);

    return st;

}

void SRTM_SpiService_Destroy(srtm_service_t service)
{
    struct _srtm_spi_service *handle = (struct _srtm_spi_service *)service;
    assert(service);

    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}

srtm_service_t SRTM_SpiService_Create(srtm_spi_adapter_t adapter)
{
    struct _srtm_spi_service *handle;
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = SRTM_Heap_Malloc(sizeof(struct _srtm_spi_service));
    if(!handle)
        return NULL;
    adapter->service = &handle->service;
    adapter->sendNotify = SRTM_SpiService_SendNotify;
    handle->adapter = adapter;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_SPI_CATEGORY;
    handle->service.destroy    = SRTM_SpiService_Destroy;
    handle->service.request    = SRTM_SpiService_HandleRequest;

    return (srtm_service_t)handle;
}
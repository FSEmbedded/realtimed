#ifndef __SRTM_SPI_SERVICE_H__
#define __SRTM_SPI_SERVICE_H__
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "fsl_component_serial_manager.h"
#include <board/board_spi.h>


/*!
 * @addtogroup srtm_service
 * @{
 */
/*******************************************************************************
 * Definitions
 * ******************************************************************************/
/** @brief Switch to disable SPI service debugging messages. */
#ifndef SRTM_SPI_SERVICE_DEBUG_OFF
#define SRTM_SPI_SERVICE_DEBUG_OFF (0)
#endif
#if SRTM_SPI_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif
#ifndef SRTM_SPI_ENDPOINT_MAX_NUM
#define SRTM_SPI_ENDPOINT_MAX_NUM (2U)
#endif
#define BOARD_SPI_MAX_BUFFER_SIZE 475
/* Protocol definition */
#define SRTM_SPI_CATEGORY (0xCU)
#define SRTM_SPI_VERSION (0x0100U)
#define SPI_RPMSG_COMMAND_TRANSFER  0x1

struct srtm_spi_chan
{
    srtm_channel_t chan;
    uint8_t bus_id;
};
/**
 * @brief SRTM SPI adapter structure pointer.
 */
typedef struct _srtm_spi_adapter *srtm_spi_adapter_t;

/**
 * @brief SRTM SPI adapter structure
 *
 */
struct _srtm_spi_adapter
{
    struct spi_adapter *spi_adapter;
    srtm_service_t service;
    srtm_status_t (*sendNotify)(srtm_service_t service, struct srtm_spi_chan *spi_chan, uint8_t cmd, const uint8_t *data, uint32_t len);
};

struct _srtm_spi_service
{
    struct _srtm_service service;
    srtm_spi_adapter_t adapter;
};

SRTM_PACKED_BEGIN struct srtm_spi_payload
{
    uint8_t busID;
    uint16_t slaveAddr;
    uint8_t flags;
    uint32_t speed_hz;
    uint16_t len;
    uint8_t buf[1];
} SRTM_PACKED_END;
/*******************************************************************************
 * API
 ******************************************************************************/
/**
 * @brief Create a SPI service.
 *
 * @param adapter SPI adapter to be used by the service.
 * @return SRTM service handle on success, NULL on failure.
 */
srtm_service_t SRTM_SpiService_Create(srtm_spi_adapter_t adapter);

/**
 * @brief Destroy a SPI service.
 * @param service SRTM service to destroy.
 */
void SRTM_SpiService_Destroy(srtm_service_t service);

srtm_status_t SRTM_SpiService_SendNotify(srtm_service_t service, struct srtm_spi_chan *spi_chan, uint8_t cmd, const uint8_t *data, uint32_t len);

#endif /* __SRTM_SPI_SERVICE_H__ */
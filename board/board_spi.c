
#include "board/board_spi.h"
#include "fsl_lpspi.h"
#include "fsl_reset.h"
#include "board/board.h"
#include <board/board_dev.h>

static struct spi_iface *get_iface_from_idx(struct spi_adapter *spi_adapter, uint8_t ifaceId)
{
    struct spi_iface *spi_iface = spi_adapter->spi_iface;
    uint32_t num_iface = spi_adapter->num_iface;
    uint32_t i;

    for(i = 0; i < num_iface; i++){
        if(spi_iface[i].id == ifaceId)
            return &spi_iface[i];
    }

    return  NULL;
}

void lpspi_callback(LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData)
{
    struct spi_iface *iface = (struct spi_iface *)userData;
    iface->transfer_done = true;
}

static void __init_spi(struct spi_iface *spi_iface)
{
    CLOCK_SetIpSrc(spi_iface->dev.ip_name, spi_iface->dev.ip_src);
    CLOCK_EnableClock(spi_iface->dev.ip_name);
    RESET_PeripheralReset(spi_iface->dev.reset);
}

void BOARD_LPSPI_Init(struct spi_adapter *spi_adapter, struct spi_iface *spi_iface)
{
    lpspi_master_config_t config = spi_iface->config;

    __init_spi(spi_iface);
    LPSPI_Enable((LPSPI_Type *)spi_iface->dev.base_addr, true);

    LPSPI_MasterGetDefaultConfig(&config);
    config.baudRate = 2000000U;
    config.whichPcs = kLPSPI_Pcs0;
    config.pcsToSckDelayInNanoSec = 1000000000U / (config.baudRate * 2U);;
    config.lastSckToPcsDelayInNanoSec = 1000000000U / (config.baudRate * 2U);
    config.betweenTransferDelayInNanoSec = 1000000000U / (config.baudRate * 2U);
    config.cpol = kLPSPI_ClockPolarityActiveHigh;
    config.cpha = kLPSPI_ClockPhaseFirstEdge;
    config.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;
    config.pinCfg = kLPSPI_SdoInSdiOut;
    config.dataOutConfig = kLpspiDataOutRetained;

    LPSPI_MasterInit((LPSPI_Type *)spi_iface->dev.base_addr, &config, CLOCK_GetIpFreq(spi_iface->dev.ip_name));
    LPSPI_MasterTransferCreateHandle((LPSPI_Type *)spi_iface->dev.base_addr, &spi_iface->handle, lpspi_callback, spi_iface);
}

void BOARD_LPSPI_Deinit(struct spi_adapter *spi_adapter, struct spi_iface *spi_iface)
{
    struct dev *dev = &spi_iface->dev;

    LPSPI_Deinit((LPSPI_Type *)dev->base_addr);
}

status_t BOARD_LPSPI_Transfer(struct spi_adapter *spi_adapter, struct spi_iface *spi_iface, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len, uint8_t flags)
{
    struct dev *dev = &spi_iface->dev;
    lpspi_transfer_t transfer = {0};
    LPSPI_Type *base = (LPSPI_Type *)dev->base_addr;
    status_t ret;

    spi_iface->transfer_done = false;
    transfer.txData = tx_buf;
    transfer.rxData = rx_buf;
    transfer.dataSize = len;

    static bool last_was_continuous = false;

    if (!last_was_continuous) {
        LPSPI_FlushFifo(base, true, true);
        LPSPI_ClearStatusFlags(base, kLPSPI_AllStatusFlag);
    }

    if (flags & 0x02) {
        transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
        last_was_continuous = true;
    } else {
        transfer.configFlags = kLPSPI_MasterPcs0;
        last_was_continuous = false;
    }

    ret = LPSPI_MasterTransferNonBlocking(base, &spi_iface->handle, &transfer);

    vTaskDelay(pdMS_TO_TICKS(20));

    if (ret == kStatus_Success) {
        return kStatus_Success;
    } else {
        LPSPI_MasterInit((LPSPI_Type *)dev->base_addr, &spi_iface->config, CLOCK_GetIpFreq(dev->ip_name));
        return kStatus_Fail;
    }

    return ret;
}

static int __init_spi_iface(struct spi_iface *spi_iface, struct dev *spi_dev, uint8_t id)
{
    memcpy(&spi_iface->dev, spi_dev, sizeof(struct dev));
    spi_iface->id  = id;

    LPSPI_MasterGetDefaultConfig(&spi_iface->config);

    return 0;
}

int init_spi_adapter(struct spi_adapter *spi_adapter, struct dev *spi_devs, enum board_types btype)
{
    int ret = 0;

    switch(btype){
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
        case BT_PICOCOREMX8ULP:
            spi_adapter->spi_iface = pvPortMalloc(sizeof(struct spi_iface));
            if(!spi_adapter->spi_iface)
                return -ENOMEM;

            ret = __init_spi_iface(&spi_adapter->spi_iface[0], &spi_devs[0], 1);
            if(ret){
                vPortFree(spi_adapter->spi_iface);
                return ret;
            }
            spi_adapter->num_iface = 1;
            break;
#endif /* CONFIG_BOARD_OSMSFMX8ULP */
#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
        case BT_ARMSTONEMX8ULP:
            spi_adapter->spi_iface = pvPortMalloc(sizeof(struct spi_iface));
            if(!spi_adapter->spi_iface)
                return -ENOMEM;

            ret = __init_spi_iface(&spi_adapter->spi_iface[0], &spi_devs[0], 1);
            if(ret){
                vPortFree(spi_adapter->spi_iface);
                return ret;
            }

            spi_adapter->num_iface = 1;
            break;
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */
#ifdef CONFIG_BOARD_OSMSFMX8ULP
        case BT_OSMSFMX8ULP:
            spi_adapter->spi_iface = pvPortMalloc(sizeof(struct spi_iface));
            if(!spi_adapter->spi_iface)
                return -ENOMEM;

            ret = __init_spi_iface(&spi_adapter->spi_iface[0], &spi_devs[0], 1);
            if(ret){
                vPortFree(spi_adapter->spi_iface);
                return ret;
            }

            spi_adapter->num_iface = 1;
            break;
#endif /* CONFIG_BOARD_OSMSFMX8ULP */
        default:
            return -EINVAL;
    }

    spi_adapter->ops.get_iface_from_idx = &get_iface_from_idx;
    spi_adapter->ops.init = &BOARD_LPSPI_Init;
    spi_adapter->ops.deinit = &BOARD_LPSPI_Deinit;
    spi_adapter->ops.transfer = &BOARD_LPSPI_Transfer;

    return 0;
}

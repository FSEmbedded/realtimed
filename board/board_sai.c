
#include <string.h>
#include <srtm/srtm_sai_edma_adapter.h>
#include <srtm/srtm_audio_service.h>
#include <board/board_sai.h>
#include "fsl_reset.h"
#include <board/board.h>



int __init_sai_handle(struct sai_chip *sai_chip, struct dev *sai_dev)
{

    memcpy(&sai_chip->dev, sai_dev, sizeof(struct dev));

    CLOCK_SetIpSrc(sai_dev->ip_name, sai_dev->ip_src);
    RESET_PeripheralReset(sai_dev->reset);

    return 0;
}

int init_sai_edma_adapter(struct sai_adapter *sai_adapter, struct dev *sai_devs, enum board_types btype)
{
    struct sai_chip *sai_chip;

    switch (btype)
    {
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
        case BT_PICOCOREMX8ULP:
            sai_chip = pvPortMalloc(sizeof(struct sai_chip));
            if (!sai_chip )
                return -ENOMEM;

            sai_adapter->sai_chip = sai_chip;
            sai_adapter->num_chips = 1;
            sai_chip[0].edma_mux1 = kDmaRequestMux0SAI1Tx;
            sai_chip[0].edma_mux2 = kDmaRequestMux0SAI1Rx;
            __init_sai_handle(&sai_chip[0], &sai_devs[1]);
            break;
#endif
#ifdef CONFIG_BOARD_OSMSFMX8ULP
        case BT_OSMSFMX8ULP:
            sai_chip = pvPortMalloc(sizeof(struct sai_chip));
            if (!sai_chip )
                return -ENOMEM;
            
            sai_adapter->sai_chip = sai_chip;
            sai_adapter->num_chips = 1;
            sai_chip[0].edma_mux1 = kDmaRequestMux0SAI0Tx;
            sai_chip[0].edma_mux2 = kDmaRequestMux0SAI0Rx;
            __init_sai_handle(&sai_chip[0], &sai_devs[0]);
            break;
#endif
        default:
            break;
    }

    return 0;
}
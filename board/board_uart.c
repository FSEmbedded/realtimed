/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <board/board.h>
#include "fsl_lpuart.h"
#include "fsl_reset.h"
#include "fsl_common.h"

enum
{
    kLPUART_TxIdle, /*!< TX idle. */
    kLPUART_TxBusy, /*!< TX busy. */
    kLPUART_RxIdle, /*!< RX idle. */
    kLPUART_RxBusy  /*!< RX busy. */
};

static struct uart_iface *get_iface_from_idx(struct uart_adapter *uart_adapter, uint8_t ifaceId)
{
    struct uart_iface *uart_iface = uart_adapter->uart_iface;
    uint8_t num_iface = uart_adapter->num_iface;
    uint8_t i;

    for(i = 0; i < num_iface; i++){
        if(uart_iface[i].id == ifaceId)
            return &uart_iface[i];
    }

    return  NULL;
}

static void __init_uart(struct uart_iface *uart_iface)
{
    struct dev *dev = &uart_iface->dev;

    CLOCK_SetIpSrc(dev->ip_name, dev->ip_src);
    RESET_PeripheralReset(dev->reset);
}

void BOARD_InitDebugConsole(struct dbg_info *dbg_info)
{
    struct uart_iface *uart_iface = dbg_info->uart_iface;
    struct dev *dev = &uart_iface->dev;
    uint32_t uartClkSrcFreq;

    __init_uart(uart_iface);

    uartClkSrcFreq = CLOCK_GetLpuartClkFreq(dev->instance);

    RESET_PeripheralReset(dev->reset);
    DbgConsole_Init(dev->instance,
        uart_iface->config.baudRate_Bps,
        dbg_info->port_type,
        uartClkSrcFreq);
}

static void BOARD_LPUART_Init(struct uart_adapter *uart_adapter, struct uart_iface *uart_iface)
{
    struct dev *dev = &uart_iface->dev;
    lpuart_config_t *config = &uart_iface->config;

    __init_uart(uart_iface);
    config->enableTx     = true;
    config->enableRx     = true;

    LPUART_Init((LPUART_Type *)dev->base_addr, config, CLOCK_GetIpFreq(dev->ip_name));
    LPUART_TransferCreateHandle((LPUART_Type *)dev->base_addr, &uart_iface->handle, NULL, uart_iface);
}

static void BOARD_LPUART_Deinit(struct uart_adapter *uart_adapter, struct uart_iface *uart_iface)
{
    // struct dev *dev = &uart_iface->dev;

    /* TODO:
     * The Clock disable part seems to be not working as expected.
     * The CLK will be disabled, but after a short time, the system hangs.
     * It looks like someone tries to access the LPUART again. Maybe a caching issue?
     */
    // LPUART_Deinit((LPUART_Type *)dev->base_addr);
}

static status_t BOARD_LPUART_Write(struct uart_adapter *uart_adaper,
                        struct uart_iface *uart_iface,
                        uint8_t *buf, size_t len)
{
    struct dev *dev = &uart_iface->dev;
    lpuart_transfer_t xfer;

    xfer.data               = buf;
    xfer.dataSize           = len;

    return LPUART_TransferSendNonBlocking((LPUART_Type *)dev->base_addr, &uart_iface->handle, &xfer);
}

static status_t BOARD_LPUART_Read(struct uart_adapter *uart_adapter,
                        struct uart_iface *uart_iface,
                        uint8_t *buf, size_t *len)
{
    struct dev *dev = &uart_iface->dev;
    lpuart_transfer_t xfer;
    xfer.data               = buf;
    xfer.dataSize           = *len;
    return LPUART_TransferReceiveNonBlocking((LPUART_Type *)dev->base_addr, &uart_iface->handle, &xfer, len);
}

static int __init_uart_iface(struct uart_iface *uart_iface, struct dev *uart_dev, uint8_t id)
{
    memcpy(&uart_iface->dev, uart_dev, sizeof(struct dev));
    uart_iface->id  = id;

    LPUART_GetDefaultConfig(&uart_iface->config);
    uart_iface->config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;

    return 0;
}

static int is_transfer_complete(struct uart_adapter *uart_adapter, struct uart_iface *uart_iface)
{
    return (uart_iface->handle.txState != kLPUART_TxBusy);
}

int init_uart_adapter(struct uart_adapter *uart_adapter, struct dev *uart_devs, struct board_descr *bdescr)
{
    struct uart_iface *iuart;
    uint32_t bfeatures = bdescr->bfeatures;
    int ret = 0;

    switch(bdescr->btype){
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
        case BT_PICOCOREMX8ULP:
            iuart = pvPortMalloc(sizeof(struct uart_iface));
            if(!iuart)
                return -ENOMEM;

            if(bfeatures & FEAT_BLUETOOTH){
                ret = __init_uart_iface(&iuart[0], &uart_devs[0], 1);
            } else {
                ret = __init_uart_iface(&iuart[0], &uart_devs[1], 3);
            }

            if(ret){
                vPortFree(iuart);
                return ret;
            }

            uart_adapter->num_iface = 1;
            uart_adapter->uart_iface = iuart;
            break;
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */
#ifdef CONFIG_BOARD_OSMSFMX8ULP
        case BT_OSMSFMX8ULP:
            iuart = pvPortMalloc(sizeof(struct uart_iface) * 2);
            if(!iuart)
                return -ENOMEM;

            ret = __init_uart_iface(&iuart[0], &uart_devs[1], 3);
            if(ret){
                vPortFree(iuart);
                return ret;
            }

            ret = __init_uart_iface(&iuart[1], &uart_devs[2], 2);
            if(ret){
                vPortFree(iuart);
                return ret;
            }

            uart_adapter->num_iface = 2;
            uart_adapter->uart_iface = iuart;
            break;
#endif /* CONFIG_BOARD_OSMSFMX8ULP */
#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
        case BT_ARMSTONEMX8ULP:
            iuart = pvPortMalloc(sizeof(struct uart_iface));
            if(!iuart)
                return -ENOMEM;

            ret = __init_uart_iface(&iuart[0], &uart_devs[2], 2);
            if(ret){
                vPortFree(iuart);
                return ret;
            }

            uart_adapter->num_iface = 1;
            uart_adapter->uart_iface = iuart;
            break;
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */
        default:
            return -EINVAL;
    }

    uart_adapter->ops.get_iface_from_idx = &get_iface_from_idx;
    uart_adapter->ops.init = &BOARD_LPUART_Init;
    uart_adapter->ops.deinit = &BOARD_LPUART_Deinit;
    uart_adapter->ops.read = &BOARD_LPUART_Read;
    uart_adapter->ops.write = &BOARD_LPUART_Write;
    uart_adapter->ops.is_transfer_complete = &is_transfer_complete;

    return 0;
}

int init_dbg_info(struct dbg_info *dbg_info, struct dev *uart_devs, struct board_descr *bdescr)
{
    struct uart_iface *uart_iface = NULL;
    uint32_t bfeatures = bdescr->bfeatures;
    int ret = 0;

    uart_iface = pvPortMalloc(sizeof(struct uart_iface));
    if(!uart_iface){
        return -ENOMEM;
    }

    switch(bdescr->btype){
        case BT_PICOCOREMX8ULP:
            if(bfeatures & FEAT_BLUETOOTH){
                ret = __init_uart_iface(uart_iface, &uart_devs[1], 0);
            } else {
                ret = __init_uart_iface(uart_iface, &uart_devs[0], 0);
            }
            break;
        case BT_OSMSFMX8ULP:
            ret = __init_uart_iface(uart_iface, &uart_devs[0], 0);
            break;
        case BT_ARMSTONEMX8ULP:
            ret = __init_uart_iface(uart_iface, &uart_devs[0], 0);
            break;
        default:
            ret = -EINVAL;
            break;
    }

    if(ret)
        return ret;

    dbg_info->uart_iface = uart_iface;
    dbg_info->port_type = kSerialPort_Uart;

    return 0;
}

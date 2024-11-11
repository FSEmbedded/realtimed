/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <board/board.h>
#include "fsl_lpuart.h"
#include "fsl_reset.h"
#include "fsl_common.h"

static void __init_uart(struct uart_iface *uart_iface)
{
    struct dev *dev = &uart_iface->dev;

    CLOCK_SetIpSrc(dev->ip_name, dev->ip_src);
    RESET_PeripheralReset(dev->reset);
}

int init_dbg_info(struct dbg_info *dbg_info, struct dev *uart_devs, enum board_types btype);

void BOARD_InitDebugConsole(struct dbg_info *dbg_info)
{
    struct uart_iface *uart_iface = dbg_info->uart_iface;
    struct dev *dev = &uart_iface->dev;
    uint32_t uartClkSrcFreq;

    __init_uart(uart_iface);

    uartClkSrcFreq = CLOCK_GetLpuartClkFreq(dev->instance);

    RESET_PeripheralReset(dev->reset);
    DbgConsole_Init(dev->instance,
        uart_iface->baudrate,
        dbg_info->port_type,
        uartClkSrcFreq);
}

/* LPUART user callback */
static void LPUART_Callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    struct uart_iface *uart_iface = userData;

    if (status == kStatus_LPUART_TxIdle)
    {
        uart_iface->txBufferFull = false;
        uart_iface->txOnGoing    = false;
    }

    if (status == kStatus_LPUART_RxIdle)
    {
        uart_iface->rxBufferEmpty = false;
        uart_iface->rxOnGoing     = false;
    }
}

static void BOARD_LPUART_Init(struct uart_adapter *uart_adapter, struct uart_iface *uart_iface)
{
    struct dev *dev = &uart_iface->dev;
    lpuart_config_t config;

    __init_uart(uart_iface);

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = uart_iface->baudrate;
    config.enableTx     = true;
    config.enableRx     = true;

    uart_iface->rxBufferEmpty = true;
    uart_iface->txBufferFull  = false;
    uart_iface->txOnGoing     = false;
    uart_iface->rxOnGoing     = false;

    LPUART_Init((LPUART_Type *)dev->base_addr, &config, CLOCK_GetIpFreq(dev->ip_name));
    LPUART_TransferCreateHandle((LPUART_Type *)dev->base_addr, &uart_iface->handle, LPUART_Callback, uart_iface);
}

static status_t BOARD_LPUART_Write(struct uart_adapter *uart_adaper,
                        struct uart_iface *uart_iface,
                        uint8_t *buf, size_t len)
{
    struct dev *dev = &uart_iface->dev;
    lpuart_transfer_t xfer;

    xfer.data       = buf;
    xfer.dataSize   = len;
    uart_iface->txOnGoing = true;

    return LPUART_TransferSendNonBlocking((LPUART_Type *)dev->base_addr, &uart_iface->handle, &xfer);
}

static status_t BOARD_LPUART_Read(struct uart_adapter *uart_adapter,
                        struct uart_iface *uart_iface,
                        uint8_t *buf, size_t *len)
{
    struct dev *dev = &uart_iface->dev;
    lpuart_transfer_t xfer;
    
    xfer.data       = buf;
    xfer.dataSize   = *len;
    uart_iface->rxOnGoing = true;

    return LPUART_TransferReceiveNonBlocking((LPUART_Type *)dev->base_addr, &uart_iface->handle, &xfer, len);
}

static int __init_uart_iface(struct uart_iface *uart_iface, struct dev *uart_dev, uint8_t id)
{
    memcpy(&uart_iface->dev, uart_dev, sizeof(struct dev));
    uart_iface->id  = id;
    uart_iface->baudrate = BOARD_DEBUG_UART_BAUDRATE;
    
    return 0;
}

int init_uart_adapter(struct uart_adapter *uart_adapter, struct dev *uart_devs, enum board_types btype)
{
    struct uart_iface *iuart;
    int ret = 0;

    switch(btype){
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
        case BT_PICOCOREMX8ULP:
            iuart = pvPortMalloc(sizeof(struct uart_iface));
            if(!iuart)
                return -ENOMEM;

            ret = __init_uart_iface(&iuart[0], &uart_devs[1], 1);
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

            ret = __init_uart_iface(&iuart[0], &uart_devs[1], 1);
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

    uart_adapter->ops.init = &BOARD_LPUART_Init;
    uart_adapter->ops.read = &BOARD_LPUART_Read;
    uart_adapter->ops.write = &BOARD_LPUART_Write;

    return 0;
}

int init_dbg_info(struct dbg_info *dbg_info, struct dev *uart_devs, enum board_types btype)
{
    struct uart_iface *uart_iface = NULL;
    int ret = 0;

    uart_iface = pvPortMalloc(sizeof(struct uart_iface));
    if(!uart_iface){
        return -ENOMEM;
    }

    switch(btype){
        case BT_PICOCOREMX8ULP:
            ret = __init_uart_iface(uart_iface, &uart_devs[0], 0);
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

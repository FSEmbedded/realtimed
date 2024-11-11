/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __BOARD_UART_H
#define __BOARD_UART_H
#include <board/board_dev.h>
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"

/* Description of an UART instance */
struct uart_iface {
    struct dev dev;
    uint8_t id;                     // user defined Iface ID
    uint32_t baudrate;
    struct _lpuart_handle handle;   // driver Handle
    volatile bool rxBufferEmpty;
    volatile bool txBufferFull;
    volatile bool txOnGoing;
    volatile bool rxOnGoing;
};

struct uart_adapter;
/* UART Methods */
struct uart_ops {
    /**
     * @brief init uart, setup clocks and driver instance
     * 
     * @param uart_adapter: uart adapter description
     * @param uart_iface: a specific uart iface instance
     */
    void (*init)(struct uart_adapter *uart_adapter, struct uart_iface *uart_iface);
    
    /**
     * @brief read data with size len from uart instance
     * 
     * @param uart_adapter: uart adapter description
     * @param uart_iface: a specific uart iface instance
     * @param buf: ptr to buffer
     * @param len: ptr to lenght, lenght will be overridden, after data is read
     * 
     * @return status_t values
     */
    status_t (*read)(struct uart_adapter *uart_adapter,
                        struct uart_iface *uart_iface,
                        uint8_t *buf, size_t *len);
    /**
     * @brief write data with size len
     * @param uart_adapter: uart adapter description
     * @param uart_iface: a specific uart iface instance
     * @param buf: ptr to buffer
     * @param len: data size to send.
     */
    status_t (*write)(struct uart_adapter *uart_adaper,
                        struct uart_iface *uart_iface,
                        uint8_t *buf, size_t len);
};

struct uart_adapter {
    struct uart_iface *uart_iface; //Array of UART Instances
    uint32_t num_iface; // Array Size of UART Instances
    struct uart_ops ops;
};

struct dbg_info {
    struct uart_iface *uart_iface; // Single UART Instance for DBG
    enum _serial_port_type port_type;
};

/**
 * @brief initialise dbg info structure
 * 
 * @param dbg_info: dbg info to initialise
 * @param uart_devs: array of UART Instances
 * @param btype: board type
 * @return 0 or -ERRNO
 */
int init_dbg_info(struct dbg_info *dbg_info, struct dev *uart_devs, enum board_types btype);

/**
 * @brief initialise uart adapter structure
 * 
 * @param uart_adapter: adapter to initialise
 * @param uart_devs: array of UART Instances
 * @param btype: board type
 * @return 0 or -ERRNO
 */
int init_uart_adapter(struct uart_adapter *uart_adapter, struct dev *uart_devs, enum board_types btype);

/**
 * @brief Set and initialise Debug Interface.
 *  
 * @param dbg_info: struct, that holds debug information
 */
void BOARD_InitDebugConsole(struct dbg_info *dbg_info);

#endif /* __BOARD_UART_H */
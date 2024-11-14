/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __BOARD_DESCR_H
#define __BOARD_DESCR_H

#define BOARD_DEBUG_UART_BAUDRATE 115200u
#define CFG_FUS_BOARDCFG_ADDR   0x2100e000u

struct board_descr {
    enum board_types btype;
    struct dbg_info dbg_info;
    struct uart_adapter uart_adapter;
    struct i2c_adapter i2c_adapter;
    struct io_adapter io_adapter;
};

/**
 * @brief read BOARD-CFG to determine board-type
 * 
 * @return board_types
 */
int BOARD_get_type(void);

/**
 * @brief get ptr to board_descr
 * 
 * @return ptr to board_descr
 */
struct board_descr *get_board_description(void);

/**
 * @brief initialise Board-Description
 * @param btype: Board Type
 * 
 * @return 0 or -ERRNO
 */
int BOARD_InitBoardDescr(enum board_types btype);

/**
 * @brief init clock of Peripherie,
 * that is not handled by adapter descriptions
 * 
 * @param bdescr: ptr to board description
 */
void BOARD_InitPeripherie(struct board_descr *bdescr);

/**
 * @brief set GPIO for CARRIER_PWR_EN according to OSM Spec.
 *
 * @param bdescr: Board Description
 */
void BOARD_carrier_enable(struct board_descr *bdescr);

void print_board(enum board_types btype);

#endif /* __BOARD_DESCR_H */
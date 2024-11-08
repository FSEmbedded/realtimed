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
    /* TODO: */
};

int BOARD_get_type(void);
struct board_descr *get_board_description(void);
int BOARD_InitBoardDescr(enum board_types btype);
void BOARD_InitPeripherie(struct board_descr *bdescr);
void BOARD_carrier_en();
void print_board(enum board_types btype);

#endif /* __BOARD_DESCR_H */
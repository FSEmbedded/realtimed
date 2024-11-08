/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __BOARD_DEV_H
#define __BOARD_DEV_H
#include "fsl_clock.h"

enum board_types {
	BT_UNKNOWN = -1,
    BT_PICOCOREMX8ULP = 0,
	BT_OSMSFMX8ULP,
	BT_SOLDERCOREMX8ULP,
	BT_ARMSTONEMX8ULP,
};

struct dev {
    unsigned long base_addr;
    clock_ip_name_t ip_name;
    clock_ip_src_t ip_src;
    unsigned int instance;
    IRQn_Type irq;
    void (*irqHandler)(void);
    int reset;
};

#endif
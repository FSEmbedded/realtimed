/*
 * Copyright (c) 2026 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __BOARD_CAN_H
#define __BOARD_CAN_H

#include <board/board_dev.h>
#include "fsl_common.h"

struct can_bus {
    struct dev dev;
    uint8_t bus_id;
};

struct can_adapter;
struct can_ops {
    status_t (*init)(struct can_bus *can_bus);
    struct can_bus *(*get_bus_from_idx)(struct can_adapter *can_adapter, uint8_t busID);
};

struct can_adapter {
    struct can_bus *can_buses;
    uint8_t num_buses;
    struct can_ops ops;
};

int init_can_adapter(struct can_adapter *can_adapter,
        struct dev *can_devs, struct board_descr *bdescr);

#endif /* __BOARD_CAN_H */
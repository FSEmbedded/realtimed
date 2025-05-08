/*
 * Copyright 2017-2022 NXP
 * All rights reserved.
 *
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __BOARD_SAI_H
#define __BOARD_SAI_H

#include <board/board_dev.h>

struct sai_chip {
    struct dev dev;
    uint32_t edma_mux1;
    uint32_t edma_mux2;
};


struct sai_adapter {
    struct sai_chip *sai_chip;
    uint8_t num_chips;
};

int init_sai_edma_adapter(struct sai_adapter *sai_adapter, struct dev *sai_devs, enum board_types btype);

#endif /* __BOARD_SAI_H */
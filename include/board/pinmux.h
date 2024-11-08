/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"

#ifndef __PIN_MUX_H
#define __PIN_MUX_H

/**
 * @brief Calls initialization functions in early state.
 */
void BOARD_InitBootPins_pre(void);

/*!
 * @brief Calls initialization functions.
 * @param btype: board_type ID
 */
void BOARD_InitBootPins(enum board_types btype);

#endif /* __PIN_MUX_H */

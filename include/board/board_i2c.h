/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __BOARD_I2C_H
#define __BOARD_I2C_H

#include <board/board_dev.h>
#include "fsl_common.h"
#include "semphr.h"

enum i2c_type {
    I2C_TYPE_LPI2C  =   0U,
    I2C_TYPE_I2C,
    I2C_TYPE_I3C,
};

enum i2c_switch_channel
{
    I2C_SWITCH_CHANNEL0 = 0U,
    I2C_SWITCH_CHANNEL1,
    I2C_SWITCH_CHANNEL2,
    I2C_SWITCH_CHANNEL3,
    I2C_SWITCH_CHANNEL_UNSPECIFIED
};

#define I2C_SWITCH_NONE 1

enum master_transfer_flags
{
    I2C_TransferDefaultFlag       = 0x00U, /*!< Transfer starts with a start signal, stops with a stop signal. */
    I2C_TransferNoStartFlag       = 0x01U, /*!< Don't send a start condition, address, and sub address */
    I2C_TransferRepeatedStartFlag = 0x02U, /*!< Send a repeated start condition */
    I2C_TransferNoStopFlag        = 0x04U, /*!< Don't send a stop condition. */
    I2C_TransferWordsFlag         = 0x08U, /*!< Transfer in words, else transfer in bytes. */
    I2C_TransferDisableRxTermFlag = 0x10U, /*!< Disable Rx termination. Note: It's for I3C CCC transfer. */
    I2C_TransferRxAutoTermFlag    = 0x20U, /*!< Set Rx auto-termination. Note: It's adaptive based on Rx size(<=255 bytes) except in I3C_MasterReceive. */
    I2C_TransferStartWithBroadcastAddr = 0x40U, /*!< Start transfer with 0x7E, then read/write data with device address. */

};

struct i2c_bus {
    struct dev dev;
    uint8_t bus_id;
    enum i2c_type type;
    uint32_t baudRate_Hz;
    uint8_t switch_idx;
    SemaphoreHandle_t xSemaphore;
    enum i2c_switch_channel switch_channel;
};

struct i2c_switch {
    uint16_t slaveAddr;
    enum i2c_switch_channel cur_channel;
};

struct i2c_adapter;
struct i2c_ops {
    status_t (*init)(struct i2c_bus *i2c_bus);
    struct i2c_bus *(*get_bus_from_idx)(struct i2c_adapter *i2c_adapter, uint8_t busID);
    status_t (*read)(struct i2c_adapter *i2c_adapter,
                    uint8_t busID,
                    uint16_t slaveAddr,
                    uint16_t subAddress,
                    size_t subAddressSize,
                    uint8_t *buf,
                    uint16_t len,
                    uint32_t flags);
    status_t (*write)(struct i2c_adapter *i2c_adapter,
                    uint8_t busID,
                    uint16_t slaveAddr,
                    uint16_t subAddress,
                    size_t subAddressSize,
                    uint8_t *buf,
                    uint16_t len,
                    uint32_t flags);
    status_t (*switchchannel)(struct i2c_adapter *i2c_adapter,
                    uint8_t busID,
                    uint16_t slaveAddr,
                    enum i2c_switch_channel channel);
};

struct i2c_adapter {
    struct i2c_bus *i2c_buses;
    uint8_t num_buses;
    struct i2c_switch *switches;
    uint8_t num_switches;
    struct i2c_ops ops;
};

int init_i2c_adapter(struct i2c_adapter *i2c_adapter, struct dev *i2c_devs, enum board_types btype);

#endif /* __BOARD_I2C_H */
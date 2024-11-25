/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "fsl_lpi2c.h"
#include "fsl_i3c.h"
#include "fsl_reset.h"
#include <board/board_i2c.h>
#include <asm-generic/errno.h>

static struct i2c_bus *BOARD_I2C_SearchBus(struct i2c_adapter *i2c_adapter, uint8_t busID)
{
    uint8_t num_buses = i2c_adapter->num_buses;
    struct i2c_bus *i2c_buses = i2c_adapter->i2c_buses;
    uint8_t i;

    for(i = 0; i < num_buses; i++){
        if(i2c_buses[i].bus_id == busID)
            return &i2c_buses[i];
    }

    return  NULL;
}

static void _LPI2C_Init(struct i2c_bus *i2c_bus)
{
    struct dev *dev = &i2c_bus->dev;
    lpi2c_master_config_t lpi2c_config = {0};

    LPI2C_MasterGetDefaultConfig(&lpi2c_config);
    lpi2c_config.baudRate_Hz = i2c_bus->baudRate_Hz;

    LPI2C_MasterInit((LPI2C_Type *)dev->base_addr, &lpi2c_config, CLOCK_GetLpi2cClkFreq(dev->instance));
}

static void _I3C_Init(struct i2c_bus *i2c_bus)
{
    struct dev *dev = &i2c_bus->dev;
    i3c_master_config_t  i3c_config = {0};

    I3C_MasterGetDefaultConfig(&i3c_config);
    i3c_config.baudRate_Hz.i2cBaud = i2c_bus->baudRate_Hz;
    i3c_config.baudRate_Hz.i3cPushPullBaud = 4000000U;
    i3c_config.baudRate_Hz.i3cOpenDrainBaud = 1500000U;
    i3c_config.enableOpenDrainHigh          = false;
    i3c_config.enableOpenDrainStop          = false;

    I3C_MasterInit((I3C_Type *)dev->base_addr, &i3c_config, CLOCK_GetI3cClkFreq(dev->instance));
}

static status_t BOARD_I2C_Init(struct i2c_bus *i2c_bus)
{
    enum i2c_type type = i2c_bus->type;

    /* Init IP */
    CLOCK_SetIpSrc(i2c_bus->dev.ip_name, i2c_bus->dev.ip_src);
    RESET_PeripheralReset(i2c_bus->dev.reset);

    /* Init Mutex */
    i2c_bus->xSemaphore = xSemaphoreCreateMutex();
    if(!i2c_bus->xSemaphore)
        return kStatus_Fail;

    /* Init Driver Instance */
    switch(type){
        case I2C_TYPE_LPI2C:
            _LPI2C_Init(i2c_bus);
            break;
        case I2C_TYPE_I3C:
            _I3C_Init(i2c_bus);
            break;
        default:
            break;
    }

    return kStatus_Success;
}

static uint32_t _i2c_flags_lut(uint32_t flags, uint32_t type)
{
    uint32_t devFlags = 0;

    switch(type)
    {
        case I2C_TYPE_LPI2C:
            devFlags = kLPI2C_TransferDefaultFlag;
            devFlags |= (flags & I2C_TransferNoStartFlag) ? kLPI2C_TransferNoStartFlag : 0;
            devFlags |= (flags & I2C_TransferRepeatedStartFlag) ? kLPI2C_TransferRepeatedStartFlag : 0;
            devFlags |= (flags & I2C_TransferNoStopFlag) ? kLPI2C_TransferNoStopFlag : 0;
            break;
        case I2C_TYPE_I3C:
            devFlags = kI3C_TransferDefaultFlag;
            devFlags |= (flags & I2C_TransferNoStartFlag) ? kI3C_TransferNoStartFlag : 0;
            devFlags |= (flags & I2C_TransferRepeatedStartFlag) ? kI3C_TransferRepeatedStartFlag : 0;
            devFlags |= (flags & I2C_TransferNoStopFlag) ? kI3C_TransferNoStopFlag : 0;
            devFlags |= (flags & I2C_TransferWordsFlag) ? kI3C_TransferWordsFlag : 0;
            devFlags |= (flags & I2C_TransferDisableRxTermFlag) ? kI3C_TransferDisableRxTermFlag : 0;
            devFlags |= (flags & I2C_TransferRxAutoTermFlag) ? kI3C_TransferRxAutoTermFlag : 0;
            devFlags |= (flags & I2C_TransferStartWithBroadcastAddr) ? kI3C_TransferStartWithBroadcastAddr : 0;
            break;
        default:
            break;
    }

    return devFlags;
}

static status_t _LPI2C_Send(LPI2C_Type *base,
                          uint8_t deviceAddress,
                          uint32_t subAddress,
                          uint8_t subAddressSize,
                          uint8_t *txBuff,
                          uint16_t txBuffSize,
                          uint32_t flags)
{
    lpi2c_master_transfer_t xfer;

    xfer.flags          = _i2c_flags_lut(flags, I2C_TYPE_LPI2C);
    xfer.slaveAddress   = deviceAddress;
    xfer.direction      = kLPI2C_Write;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = txBuff;
    xfer.dataSize       = txBuffSize;

    return LPI2C_MasterTransferBlocking(base, &xfer);
}

static status_t _I3C_Send(I3C_Type *base,
                        uint8_t deviceAddress,
                        uint32_t subAddress,
                        size_t subAddressSize,
                        uint8_t *txBuff,
                        uint16_t txBuffSize,
                        uint32_t flags)
{
    i3c_master_transfer_t xfer;
    xfer.flags          = _i2c_flags_lut(flags, I2C_TYPE_I3C);
    xfer.slaveAddress   = deviceAddress;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = txBuff;
    xfer.dataSize       = txBuffSize;
    xfer.direction      = kI3C_Write;
    xfer.busType        = kI3C_TypeI2C;

    return I3C_MasterTransferBlocking(base, &xfer);
}

static status_t _LPI2C_Receive(LPI2C_Type *base,
                             uint8_t deviceAddress,
                             uint32_t subAddress,
                             size_t subAddressSize,
                             uint8_t *rxBuff,
                             uint16_t rxBuffSize,
                             uint32_t flags)
{
    lpi2c_master_transfer_t xfer;

    xfer.flags          = _i2c_flags_lut(flags, I2C_TYPE_LPI2C);
    xfer.slaveAddress   = deviceAddress;
    xfer.direction      = kLPI2C_Read;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = rxBuff;
    xfer.dataSize       = rxBuffSize;

    return LPI2C_MasterTransferBlocking(base, &xfer);
}

static status_t _I3C_Receive(I3C_Type *base,
                             uint8_t deviceAddress,
                             uint32_t subAddress,
                             size_t subAddressSize,
                             uint8_t *rxBuff,
                             uint16_t rxBuffSize,
                             uint32_t flags)
{
    i3c_master_transfer_t xfer;

    xfer.flags          = _i2c_flags_lut(flags, I2C_TYPE_I3C);
    xfer.slaveAddress   = deviceAddress;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = rxBuff;
    xfer.dataSize       = rxBuffSize;
    xfer.direction      = kI3C_Read;
    xfer.busType        = kI3C_TypeI2C;

    return I3C_MasterTransferBlocking(base, &xfer);
}

static status_t BOARD_I2C_Read(struct i2c_adapter *i2c_adapter,
                    uint8_t busID,
                    uint16_t slaveAddr,
                    uint16_t subAddress,
                    size_t subAddressSize,
                    uint8_t *buf,
                    uint16_t len,
                    uint32_t flags)
{
    struct i2c_bus *i2c_bus = NULL;
    enum i2c_type type;
    status_t ret = kStatus_InvalidArgument;

    i2c_bus = BOARD_I2C_SearchBus(i2c_adapter, busID);
    if(!i2c_bus)
        return kStatus_InvalidArgument;

    type = i2c_bus->type;

    if(xSemaphoreTake(i2c_bus->xSemaphore, ( TickType_t ) pdTICKS_TO_MS(CONFIG_I2C_MUTEX_TAKE_MS) ) != pdTRUE)
        return kStatus_Fail;

    switch(type){
        case I2C_TYPE_LPI2C:
            ret = _LPI2C_Receive((LPI2C_Type *)i2c_bus->dev.base_addr,
                                    slaveAddr, subAddress,
                                    subAddressSize, buf, len, flags);
            break;
        case I2C_TYPE_I3C:
            ret = _I3C_Receive((I3C_Type *)i2c_bus->dev.base_addr,
                                    slaveAddr, subAddress, subAddressSize,
                                    buf, len, flags);
            break;
        default:
            break;
    }

    xSemaphoreGive(i2c_bus->xSemaphore);
    return ret;
}

static status_t BOARD_I2C_Write(struct i2c_adapter *i2c_adapter,
                    uint8_t busID,
                    uint16_t slaveAddr,
                    uint16_t subAddress,
                    size_t subAddressSize,
                    uint8_t *buf,
                    uint16_t len,
                    uint32_t flags)
{
    struct i2c_bus *i2c_bus = NULL;
    enum i2c_type type;
    status_t ret = kStatus_InvalidArgument;

    i2c_bus = BOARD_I2C_SearchBus(i2c_adapter, busID);
    if(!i2c_bus)
        return kStatus_InvalidArgument;

    type = i2c_bus->type;

    if(xSemaphoreTake(i2c_bus->xSemaphore, ( TickType_t ) pdTICKS_TO_MS(CONFIG_I2C_MUTEX_TAKE_MS) ) != pdTRUE)
        return kStatus_Fail;

    switch(type){
        case I2C_TYPE_LPI2C:
            ret = _LPI2C_Send((LPI2C_Type *)i2c_bus->dev.base_addr,
                                slaveAddr, subAddress,
                                subAddressSize, buf, len, flags);
            break;
        case I2C_TYPE_I3C:
            ret = _I3C_Send((I3C_Type *)i2c_bus->dev.base_addr,
                                slaveAddr, subAddress, subAddressSize,
                                buf, len, flags);
            break;
        default:
            break;
    }

    xSemaphoreGive(i2c_bus->xSemaphore);
    return ret;
}

static status_t BOARD_I2C_SwitchChannel(struct i2c_adapter *i2c_adapter,
                    uint8_t busID,
                    uint16_t slaveAddr,
                    enum i2c_switch_channel channel)
{
    uint8_t txBuff[1];
    txBuff[0] = 1 << (uint8_t) channel;
    return i2c_adapter->ops.write(i2c_adapter, busID, slaveAddr, 0, 0, txBuff, sizeof(txBuff), I2C_TransferDefaultFlag);
}

static uint32_t _get_i2c_baudcfg(struct dev *i2c_dev, enum i2c_type i2c_type)
{
    if(i2c_type == I2C_TYPE_LPI2C){
        switch(i2c_dev->instance){
            case 0:
                return CONFIG_BOARD_LPI2C0_BAUDRATE_HZ;
            case 1:
                return CONFIG_BOARD_LPI2C1_BAUDRATE_HZ;
            case 2:
                return CONFIG_BOARD_LPI2C2_BAUDRATE_HZ;
            case 3:
                return CONFIG_BOARD_LPI2C3_BAUDRATE_HZ;
            default:
                return CONFIG_BOARD_I2C_DEFAULT_BAUD;
        }
    }
    else if (i2c_type == I2C_TYPE_I3C){
        switch(i2c_dev->instance){
            case 0:
                return CONFIG_BOARD_I3C0_BAUDRATE_HZ;
            case 1:
                return CONFIG_BOARD_I3C1_BAUDRATE_HZ;
            default:
                return CONFIG_BOARD_I3C_DEFAULT_BAUD;
        }
    }
    return CONFIG_BOARD_I2C_DEFAULT_BAUD;
}

static void _init_i2c_bus(struct i2c_bus *i2c_bus, struct dev *i2c_dev, uint8_t id, enum i2c_type i2c_type)
{
    memcpy(&i2c_bus->dev, i2c_dev, sizeof(struct dev));

    /* Default Values for i2c_bus */
    i2c_bus->bus_id = id;
    i2c_bus->switch_channel = I2C_SWITCH_CHANNEL_UNSPECIFIED;
    i2c_bus->switch_idx = I2C_SWITCH_NONE;
    i2c_bus->type = i2c_type;
    i2c_bus->baudRate_Hz = _get_i2c_baudcfg(i2c_dev, i2c_type);
}

int init_i2c_adapter(struct i2c_adapter *i2c_adapter, struct dev *i2c_devs, enum board_types btype)
{
    struct i2c_bus *i2c_bus;
    switch(btype){
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
        case BT_PICOCOREMX8ULP:
            i2c_bus = pvPortMalloc(sizeof(struct i2c_bus) * 2);
            if(!i2c_bus)
                return -ENOMEM;

            i2c_adapter->num_buses = 2;
            i2c_adapter->i2c_buses = i2c_bus;

            /* I2C_C*/
            _init_i2c_bus(&i2c_bus[0], &i2c_devs[0], 3, I2C_TYPE_LPI2C);
            /* I2C_D */
            _init_i2c_bus(&i2c_bus[1], &i2c_devs[4], 4, I2C_TYPE_I3C);
            break;
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */
#ifdef CONFIG_BOARD_OSMSFMX8ULP
        case BT_OSMSFMX8ULP:
            i2c_bus = pvPortMalloc(sizeof(struct i2c_bus) * 2);
            if(!i2c_bus)
                return -ENOMEM;

            i2c_adapter->num_buses = 2;
            i2c_adapter->i2c_buses = i2c_bus;

            /* I2C_B*/
            _init_i2c_bus(&i2c_bus[0], &i2c_devs[4], 2, I2C_TYPE_I3C);
            /* I2C_CAM */
            _init_i2c_bus(&i2c_bus[1], &i2c_devs[2], 3, I2C_TYPE_LPI2C);
            break;
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */
#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
        case BT_ARMSTONEMX8ULP:
            i2c_bus = pvPortMalloc(sizeof(struct i2c_bus) * 1);

            if(!i2c_bus)
                return -ENOMEM;

            i2c_adapter->num_buses = 1;
            i2c_adapter->i2c_buses = i2c_bus;

            /* I2C_B*/
            _init_i2c_bus(&i2c_bus[0], &i2c_devs[4], 3, I2C_TYPE_I3C);
            break;
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */
        default:
            return -EINVAL;
    }

    i2c_adapter->ops.get_bus_from_idx = &BOARD_I2C_SearchBus;
    i2c_adapter->ops.init = &BOARD_I2C_Init;
    i2c_adapter->ops.read = &BOARD_I2C_Read;
    i2c_adapter->ops.write = &BOARD_I2C_Write;
    i2c_adapter->ops.switchchannel = &BOARD_I2C_SwitchChannel;
    return 0;
}

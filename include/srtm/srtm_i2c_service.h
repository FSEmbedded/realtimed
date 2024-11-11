/*
 * Copyright 2018-2021, NXP
 * All rights reserved.
 *
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SRTM_I2C_SERVICE_H__
#define __SRTM_I2C_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable I2C service debugging messages. */
#ifndef SRTM_I2C_SERVICE_DEBUG_OFF
#define SRTM_I2C_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_I2C_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

/* Protocol definition */
#define SRTM_I2C_FLAG_NEED_STOP (0x200U) /* Linux: I2C_RPMSG_M_STOP */

/* I2C Service Notification Command definition */

typedef enum
{
    /* I2C Service Request Command definition */
    SRTM_I2C_CMD_READ = 0U,
    SRTM_I2C_CMD_WRITE,
} srtm_i2c_cmd_t;

/**
 * @brief SRTM I2C payload structure
 */
SRTM_ANON_DEC_BEGIN
SRTM_PACKED_BEGIN struct srtm_i2c_payload
{
    uint8_t busID;
    union
    {
        uint8_t reserved; /* used in request packet */
        uint8_t retCode;  /* used in response packet */
    };
    uint16_t slaveAddr;
    uint16_t flags;
    uint16_t len;
    uint8_t data[1]; /* data size is decided by uint16_t len */
} SRTM_PACKED_END;
SRTM_ANON_DEC_END

/**
 * @brief SRTM I2C adapter structure
 */
struct srtm_i2c_adapter
{
    /* Bound service */
    struct _srtm_service *service;
    struct i2c_adapter *i2c_adapter;
};

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
/*!
 * @brief Create I2C service.
 *
 * @param adapter I2C adapter to provide real I2C features.
 * @return SRTM_Status_Success, or error
 */
srtm_status_t SRTM_I2CService_Create(struct srtm_i2c_adapter *adapter);

/*!
 * @brief Destroy I2C service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_I2CService_Destroy(srtm_service_t service);

/*!
 * @brief Reset I2C service. This is used to stop sending events and return to initial state.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset.
 */
void SRTM_I2CService_Reset(srtm_service_t service, srtm_peercore_t core);

/*!
 * @brief Perfrom a local read of I2C bus
 */
srtm_status_t SRTM_I2C_RequestBusRead(
    srtm_service_t service, uint8_t busID, uint16_t slaveAddr, uint8_t *buf, uint16_t len);

/*!
 * @brief Perfrom a local write of I2C bus
 */
srtm_status_t SRTM_I2C_RequestBusWrite(
    srtm_service_t service, uint8_t busID, uint16_t slaveAddr, uint8_t *buf, uint16_t len, uint8_t needStop);
#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_I2C_SERVICE_H__ */

/*
* Copyright 2021 NXP
* All rights reserved.
*
* Copyright (c) 2024 F&S Elektronik Systeme GmbH
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef __SRTM_PWM_SERVICE_H__
#define __SRTM_PWM_SERVICE_H__

#include "srtm_service.h"

/*!
* @addtogroup srtm_service
* @{
*/
/** @brief Switch to disable PWM service debugging messages. */
#ifndef SRTM_PWM_SERVICE_DEBUG_OFF
#define SRTM_PWM_SERVICE_DEBUG_OFF (0)
#endif
#if SRTM_PWM_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

/**
* @brief SRTM PWM adapter structure
*/
struct srtm_pwm_adapter
{
   /* Bound service */
   struct _srtm_service *service;
   struct pwm_adapter *pwm_adapter;
};

/**
* @brief SRTM PWM payload structure
*/
SRTM_PACKED_BEGIN struct _srtm_pwm_payload
{
   uint8_t requestID;
   union
   {
      uint8_t reserved; /* used in request packet */
      uint8_t retCode;  /* used in response packet */
   };
   uint8_t chipId;
   uint8_t channelId;
   uint64_t period;
   uint64_t dutyCycle;
   uint8_t polarity;
   bool enable;
} SRTM_PACKED_END;

/*!
* @brief Create PWM service.
*
* @param adapter PWM adapter to handle real pwm operations.
* @return SRTM service handle on success and NULL on failure.
*/
srtm_status_t SRTM_PwmService_Create(struct srtm_pwm_adapter *adapter);

/*!
* @brief Destroy PWM service.
*
* @param service SRTM service to destroy.
*/
void SRTM_PwmService_Destroy(srtm_service_t service);

#endif /* __SRTM_PWM_SERVICE_H__ */

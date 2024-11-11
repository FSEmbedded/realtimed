/*
 * Copyright 2017, NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SRTM_IO_SERVICE_H__
#define __SRTM_IO_SERVICE_H__

#include <board/board.h>
#include "srtm_service.h"
#include "srtm_service_struct.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable IO service debugging messages. */
#ifndef SRTM_IO_SERVICE_DEBUG_OFF
#define SRTM_IO_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_IO_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

// /**
//  * @brief SRTM IO service set output value function type.
//  */
// typedef srtm_status_t (*srtm_io_service_set_output_t)(srtm_service_t service,
//                                                       srtm_peercore_t core,
//                                                       uint16_t ioId,
//                                                       enum io_value ioValue);

// /**
//  * @brief SRTM IO service get input value function type.
//  */
// typedef srtm_status_t (*srtm_io_service_get_input_t)(srtm_service_t service,
//                                                      srtm_peercore_t core,
//                                                      uint16_t ioId,
//                                                      enum io_value *pIoValue);

// /**
//  * @brief SRTM IO service configure input event function type.
//  */
// typedef srtm_status_t (*srtm_io_service_conf_input_t)(
//     srtm_service_t service, srtm_peercore_t core, uint16_t ioId, enum io_event event, bool wakeup);

SRTM_ANON_DEC_BEGIN
SRTM_PACKED_BEGIN struct gpio_rpmsg_data {
	uint8_t pin_idx;
	uint8_t port_idx;
	union {
		uint8_t event;
		uint8_t retcode;
		uint8_t value;
	} out;
	union {
		uint8_t wakeup;
		uint8_t value;
	} in;
}SRTM_PACKED_END;
SRTM_ANON_DEC_END

struct srtm_io_adapter
{
    /* Bound service */
    struct _srtm_service *service;
    struct io_adapter *io_adapter;
};

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create IO service.
 *
 * @return SRTM_Status_Success or Error.
 */
srtm_status_t SRTM_IoService_Create(struct srtm_io_adapter *adapter);

/*!
 * @brief Destroy IO service.
 */
void SRTM_IoService_Destroy(srtm_service_t service);

/*!
 * @brief Reset IO service.
 *  This is used to stop all IO operations and return to initial state for corresponding core.
 *  Registered pins are kept unchanged.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset
 */
void SRTM_IoService_Reset(srtm_service_t service, srtm_peercore_t core);

/*!
 * @brief Register IO service pin. Only registered pin will be serviced.
 *
 * @param service SRTM IO service handle.
 * @param ifaceID IO Interface identification.
 * @param pinID IO Pin identification.
 * @param param user callback parameter.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_IoService_RegisterPin(srtm_service_t service,
                                         uint8_t ifaceID,
										 uint8_t pinID,
                                         void *param);

/*!
 * @brief Unregister IO service pin. The operation cannot work when service is running.
 *
 * @param service SRTM IO service handle.
 * @param ioId IO pin identification.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_IoService_UnregisterPin(srtm_service_t service, uint8_t ifaceID, uint8_t pinID);

/*!
 * @brief Notify Input event to peer core. This function must be called by application after peer core configured
 *  input event.
 *
 * @param service SRTM IO service.
 * @param ioId IO pin identification.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_IoService_NotifyInputEvent(srtm_service_t service, uint8_t ifaceID, uint8_t pinID);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_IO_SERVICE_H__ */

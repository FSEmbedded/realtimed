/*
 * Copyright 2022-2023 NXP
 *
 * Copyright (c) 2025 F&S Elektronik Systeme GmbH
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SRTM_UART_SERVICE_H__
#define __SRTM_UART_SERVICE_H__

#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "fsl_component_serial_manager.h"
#include <board/board_uart.h>

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable UART service debugging messages. */

#ifndef SRTM_UART_ENDPOINT_MAX_NUM
#define SRTM_UART_ENDPOINT_MAX_NUM (32U)
#endif

#define SRTM_UART_INVALID_BUS_ID (0xFFU)

/* Protocol definition */
#define SRTM_UART_CATEGORY (0xBU)

#define SRTM_UART_VERSION (0x0100U)

#define TTY_RPMSG_COMMAND_INIT		0x0
#define TTY_RPMSG_COMMAND_DEINIT	0x1
#define TTY_RPMSG_COMMAND_WRITE		0x2
#define TTY_RPMSG_COMMAND_READ		0x3
#define TTY_RPMSG_COMMAND_STOP		0x4
#define TTY_RPMSG_COMMAND_SET_BAUD	0x5

struct srtm_uart_chan
{
    srtm_channel_t chan;
    uint8_t bus_id;
    volatile bool read_enable; /* true if read is enabled */
};


/**
 * @brief SRTM UART adapter structure pointer.
 */
typedef struct _srtm_uart_adapter *srtm_uart_adapter_t;


/**
 * @brief SRTM UART adapter structure
 *
 */
struct _srtm_uart_adapter
{
    struct uart_adapter *uart_adapter;
    srtm_service_t service;
    srtm_status_t (*sendNotify)(srtm_service_t service, struct srtm_uart_chan *uart_chan, uint8_t command, uint8_t *data, uint16_t data_len);
};


typedef struct _srtm_uart_service
{
    struct _srtm_service service;
    srtm_uart_adapter_t adapter;
} * srtm_uart_service_t;

/**
 * @brief SRTM UART payload structure
 */
SRTM_PACKED_BEGIN struct _srtm_uart_payload
{
	uint8_t bus_id;				/* Bus ID for the tty port */
	uint8_t ctrl;				/* Flow control: 0 = none, 1 = CRTSCTS */
	uint32_t baudrate;			/* Baudrate in bps */
	int8_t len;				/* Length of the data in buf */
	uint8_t buf[BOARD_UART_BUFFER_SIZE];	/* Data buffer for the message */
} SRTM_PACKED_END;

/*******************************************************************************
 * API
 ******************************************************************************/


/*!
 * @brief Create UART service.
 *
 * @param adapter UART adapter to handle real UART operations.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_UartService_Create(srtm_uart_adapter_t adapter);

/*!
 * @brief Destroy UART service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_UartService_Destroy(srtm_service_t service);


srtm_status_t SRTM_UartService_SendNotify(srtm_service_t service, struct srtm_uart_chan *uart_chan, uint8_t command, uint8_t *data, uint16_t data_len);


#endif /* __SRTM_UART_SERVICE_H__ */

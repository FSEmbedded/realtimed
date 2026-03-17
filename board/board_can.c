/*
 * Copyright (c) 2026 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include <board/board.h>
#include "fsl_reset.h"
#include <asm-generic/errno.h>

static struct can_bus *BOARD_CAN_SearchBus(struct can_adapter *can_adapter, uint8_t busID)
{
	uint8_t num_buses = can_adapter->num_buses;
	struct can_bus *can_buses = can_adapter->can_buses;
	uint8_t i;

	for(i = 0; i < num_buses; i++){
		if(can_buses[i].bus_id == busID)
			return &can_buses[i];
	}

	return  NULL;
}

static status_t BOARD_CAN_Init(struct can_bus *can_bus)
{
	/* Init IP */
	CLOCK_SetIpSrc(can_bus->dev.ip_name, can_bus->dev.ip_src);
	RESET_PeripheralReset(can_bus->dev.reset);

	return kStatus_Success;
};

static void _init_flexcan_bus(struct can_bus *can_bus, struct dev *can_dev, uint8_t bus_id)
{
	memcpy(&can_bus->dev, can_dev, sizeof(struct dev));
	can_bus->bus_id = bus_id;
};

int init_can_adapter(struct can_adapter *can_adapter, struct dev *can_devs, struct board_descr *bdescr)
{
	struct can_bus *can_bus;

	switch(bdescr->btype) {
		case BT_PICOCOREMX8ULP:
		case BT_OSMSFMX8ULP:
		case BT_ARMSTONEMX8ULP:

			if (!(bdescr->bfeatures & FEAT_CAN_IN_APD))
				break;

			can_bus = pvPortMalloc(sizeof(struct can_bus));
			if (!can_bus)
				return -ENOMEM;

			can_adapter->num_buses = 1;
			can_adapter->can_buses = can_bus;

			/* FLEXCAN */
			_init_flexcan_bus(&can_bus[0], &can_devs[0], 0);
			break;
		default:
			break;
	};

	can_adapter->ops.init = BOARD_CAN_Init;
	can_adapter->ops.get_bus_from_idx = BOARD_CAN_SearchBus;

	return 0;
}
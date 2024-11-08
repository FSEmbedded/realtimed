/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"

#include <board/pinmux.h>
#include <board/clock_config.h>
#include <board/board.h>
#include <srtm/app_srtm.h>
#include "fsl_debug_console.h"
#include "fsl_rgpio.h"
#include "fsl_iomuxc.h"
#include "fsl_reset.h"
#include "fsl_upower.h"
#include "fsl_fusion.h"

extern volatile app_srtm_state_t srtmState;

void initTask(void *pvParameters)
{
    struct board_descr *bdescr = NULL;
    enum board_types btype = BT_UNKNOWN;
    int ret;

   /**
    * Wait for A35-UBOOT become ready
    */
    while(!BOARD_HandshakeWithUboot())
        vTaskDelay(pdMS_TO_TICKS(100));

    /* Board-Specific Settings */
    btype = BOARD_get_type();
    BOARD_InitBootPins(btype);
    ret = BOARD_InitBoardDescr(btype);
    if(ret)
        while (1);

    bdescr = get_board_description();
    BOARD_InitPeripherie(bdescr);
    BOARD_carrier_en(btype);
    BOARD_InitDebugConsole(&bdescr->dbg_info);

    PRINTF("\r\n############  F&S realtime DAEMON ############\n\r\n");
    print_board(bdescr->btype);
    PRINTF("VERSION: %s\r\n", APP_VER);
#ifdef GIT_VER
    PRINTF("GIT: %s\r\n", GIT_VER);
#endif
    PRINTF("Build Time: %s--%s \r\n\n", __DATE__, __TIME__);

    /*
     * Wait For A35 Side Become Ready
     */
    PRINTF("********************************\r\n");
    PRINTF(" Wait the Linux kernel boot up to create the link between M core and A core.\r\n");
    PRINTF("\r\n");
    PRINTF("********************************\r\n\r\n");

    APP_SRTM_Init();
    APP_SRTM_StartCommunication();

    while (srtmState != APP_SRTM_StateLinkedUp)
        vTaskDelay(pdMS_TO_TICKS(100));

    PRINTF("********************************\r\n");
    PRINTF("The rpmsg channel between M core and A core created!\r\n");
    PRINTF("********************************\r\n");
    PRINTF("\r\n");

    vTaskDelete(NULL);
}

/*!
 * @brief Main function
 */
int main(void)
{
    BOARD_ConfigMPU();
    BOARD_InitBootPins_pre();
    BOARD_BootClockRUN();
    
    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);
    UPOWER_ReduceBuck23VoltInSTBY();
 
    Fusion_Init();

    /* RTD take LPAV master ownership to prevent APD putting DDR into retention when suspended */
    /* SIM_SEC->SYSCTRL0 &= ~SIM_SEC_SYSCTRL0_LPAV_MASTER_CTRL_MASK; */
    if (xTaskCreate(initTask, "Init Task", 256U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS)
    {
        while (1)
        {
        }
    }

    // /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Application should never reach this point. */
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    PRINTF("Malloc Failed!!!\r\n");
}

/*
 * Copyright 2021-2022 NXP
 * All rights reserved.
 * 
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 * 
 * This program includes portions of code licensed under the BSD-3-Clause License.
 * Modifications and extensions were made by F&S Elektronik Systeme GmbH in 2024.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "fsl_lpi2c_freertos.h"

#include "srtm_dispatcher.h"
#include "srtm_peercore.h"
#include "srtm_message.h"
#include <srtm/srtm_rpmsg_endpoint.h>
#include <srtm/srtm_lfcl_service.h>
#include <srtm/app_srtm.h>
#include <board/board.h>

#include "fsl_mu.h"
#include "fsl_wuu.h"
#include "fsl_upower.h"
#include "fsl_iomuxc.h"
#include "rsc_table.h"
#include "fsl_bbnsm.h"
#include "fsl_sentinel.h"

void APP_SRTM_WakeupCA35(void);

typedef enum
{
    CORE_ACT  = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x0U),
    CORE_STDB = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x1U),
    CORE_PD   = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x3U),
} core_low_power_mode_t; /* A35 core0/1 low power mode */

volatile app_srtm_state_t srtmState;
bool option_v_boot_flag          = false;
static bool need_reset_peer_core = false;
bool wake_acore_flag             = true;

pca9460_buck3ctrl_t buck3_ctrl;
pca9460_ldo1_cfg_t ldo1_cfg;

/* For CMC1_IRQHandler */
static int64_t apd_boot_cnt = 0; /* it's cold boot when apd_boot_cnt(Application Domain, A Core) == 1 */

/*
 * For Deep Sleep Mode of Application Processor Domain(APD)
 *                         +--> APD enter Power Down Mode
 *                        /
 * Linux suspend to memory
 *                        \
 *                         +--> APD enter Deep Sleep Mode(When the option IMX8ULP_DSL_SUPPORT is enabled in TF-A)
 * NOTE: cannot support them at the same time for Linux.
 */

static bool support_dsl_for_apd = false; /* true: support deep sleep mode; false: not support deep sleep mode */

static srtm_dispatcher_t disp;
static srtm_peercore_t core;
static SemaphoreHandle_t monSig;
static struct rpmsg_lite_instance *rpmsgHandle;
static app_rpmsg_monitor_t rpmsgMonitor;
static void *rpmsgMonitorParam;
static TimerHandle_t linkupTimer;
static TimerHandle_t refreshS400WdgTimer;
static TimerHandle_t restoreRegValOfMuTimer; /* use the timer to restore register's value of mu(To make sure that
                                                register's value of mu is restored if cmc1 interrupt is not comming) */

static TimerHandle_t
    chngModeFromActToDslForApdTimer; /* use the timer to change mode of APD from active mode to Deep Sleep Mode */

static app_irq_handler_t irqHandler;
static void *irqHandlerParam;

lpm_ad_power_mode_e AD_CurrentMode   = AD_UNKOWN;
lpm_ad_power_mode_e AD_WillEnterMode = AD_UNKOWN;

static MU_Type mu0_mua;
/*******************************************************************************
 * Code
 ******************************************************************************/
/* For Deep Sleep Mode of APD */
bool APP_SRTM_GetSupportDSLForApd(void)
{
    return support_dsl_for_apd;
}

void APP_SRTM_SetSupportDSLForApd(bool support)
{
    support_dsl_for_apd = support;
}

void MU0_MUA_Save(void)
{
    /* Make sure the clock is on */
    MU_Init(MU0_MUA);
    mu0_mua.RCR   = MU0_MUA->RCR;
    mu0_mua.CIER0 = MU0_MUA->CIER0;
}

void MU0_MUA_Restore(void)
{
    /* Make sure the clock is on */
    MU_Init(MU0_MUA);
    if (mu0_mua.RCR != 0)
    {
        MU0_MUA->RCR = mu0_mua.RCR;
    }
    if (mu0_mua.CIER0 != 0)
    {
        MU0_MUA->CIER0 = mu0_mua.CIER0;
    }
}

/* Real Time Domain save context */
void rtdCtxSave(void)
{
    MU0_MUA_Save();
}

/* Real Time Domain restore context */
void rtdCtxRestore(void)
{
    MU0_MUA_Restore();
}

void APP_WakeupACore(void)
{
    if (wake_acore_flag)
    {
        /*
         * For case: when RTD is the ower of LPAV and APD enter PD/DPD, Mcore don't enter lp mode, but wakes up
         * directly. RTD need restore related settings.
         */
        if (BOARD_IsLPAVOwnedByRTD())
        {
            UPOWER_ReadPmicReg(PCA9460_BUCK3CTRL_ADDR, &(buck3_ctrl.val));
            UPOWER_ReadPmicReg(PCA9460_LDO1_CFG_ADDR, &(ldo1_cfg.val));

            if (AD_CurrentMode == AD_PD || (support_dsl_for_apd == true && AD_CurrentMode == AD_DSL))
            {
                APP_SRTM_EnableLPAV();
            }
            else if (AD_CurrentMode == AD_DPD && !ldo1_cfg.reg.L1_ENMODE && !buck3_ctrl.reg.B3_ENMODE)
            {
                /* B3_ENMODE = 0x1, L1_ENMODE, ON at RUN State(default) */
                buck3_ctrl.reg.B3_ENMODE = 0x1;
                ldo1_cfg.reg.L1_ENMODE   = 0x1;
                UPOWER_SetPmicReg(PCA9460_LDO1_CFG_ADDR, ldo1_cfg.val);
                UPOWER_SetPmicReg(PCA9460_BUCK3CTRL_ADDR, buck3_ctrl.val);
            }
        }
        else /* owner of lpav is AD */
        {
            /* For RTD hold lpav, sai low power audio demo, we need enable lpav before wakeup APD */
#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
            if (R32(DDR_IN_SELFREFRESH_BASE))
            {
                DisableIRQ(BBNSM_IRQn);
                DisableIRQ(GPIOA_INT0_IRQn);
                DisableIRQ(GPIOA_INT1_IRQn);
                DisableIRQ(GPIOB_INT0_IRQn);
                DisableIRQ(GPIOB_INT1_IRQn);
                DisableIRQ(GPIOB_INT1_IRQn);
                DisableIRQ(WUU0_IRQn);

                PRINTF("Acore will enter avtive, Put ddr into active\r\n");
                /* ddr in retention state, need put ddr exit retention */
                APP_SRTM_EnableLPAV();

                W32(DDR_IN_SELFREFRESH_BASE, 0);

                EnableIRQ(GPIOA_INT0_IRQn);
                EnableIRQ(GPIOA_INT1_IRQn);
                EnableIRQ(GPIOB_INT0_IRQn);
                EnableIRQ(GPIOB_INT1_IRQn);
                EnableIRQ(BBNSM_IRQn);
                EnableIRQ(WUU0_IRQn);
            }
#endif
        }

        if (support_dsl_for_apd == true && AD_CurrentMode == AD_DSL)
        {
            MU_SendMsg(MU0_MUA, 1, 0xFFFFFFFF); /* MCore wakeup ACore with mu interrupt, 1: RPMSG_MU_CHANNEL */
        }
        else
        {
            UPOWER_PowerOnADInPDMode();
        }
    }
}

static void APP_ResetSRTM(app_srtm_state_t state)
{
    srtmState = state;
    /* Wake up monitor to reinitialize the SRTM communication with CA35 */
    xSemaphoreGive(monSig);
}

static void APP_SRTM_ControlCA35(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    app_srtm_state_t state = (app_srtm_state_t)(uint32_t)param1;

    switch (state)
    {
        case APP_SRTM_StateRun:
            /* Fresh power up: Need SRTM monitor to prepare communication */
            srtmState = APP_SRTM_StateRun;
            xSemaphoreGive(monSig);
            break;
        case APP_SRTM_StateReboot:
            /* Only when CA35 is active, we can reboot it. */
            if (!core || AD_CurrentMode != AD_ACT)
            {
                PRINTF("CA35 is not active, cannot reboot!\r\n");
            }
            else
            {
                /* Now prepare reboot */
                need_reset_peer_core = true; /* set a flag to check whether need reset peer core(don't need reset peer
                                                core when peer core is in reset) */
                APP_ResetSRTM(APP_SRTM_StateReboot);
            }
            break;
        case APP_SRTM_StateShutdown:
            /* Only when CA35 goes into DPD, we can shutdown it. */
            if (core && AD_CurrentMode == AD_DPD)
            {
                /* Now prepare shutdown */
                APP_ResetSRTM(APP_SRTM_StateShutdown);
            }
            else
            {
                PRINTF("CA35 isn't in PD mode, cannot shutdown!\r\n");
            }
            break;
        default:
            break;
    }
}

static void APP_SRTM_SetLPAV(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    lpm_ad_power_mode_e state = (lpm_ad_power_mode_e)(uint32_t)param1;

    switch (state)
    {
        case AD_UNKOWN:
            break;
        case AD_ACT:
            break;
        case AD_DSL:
        case AD_PD:
            if (BOARD_IsLPAVOwnedByRTD())
            {
                /* Power down lpav domain, put ddr into retention, poweroff LDO1, set BUCK3 to 0.73V */
                APP_SRTM_DisableLPAV();
#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
                W32(DDR_IN_SELFREFRESH_BASE, 1);
                /* In dualboot mode, RTD hold LPAV domain, set LPAV ownership to APD let ddr into retention */
                SIM_SEC->SYSCTRL0 |= SIM_SEC_SYSCTRL0_LPAV_MASTER_CTRL(1);
                SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0x7f;
                SIM_SEC->LPAV_SLAVE_ALLOC_CTRL |=
                    (SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI6(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI7(0x1) |
                     SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SEMA42(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_LPTPM8(0x1) |
                     SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SPDIF(0x1));
#endif
            }
            break;
        case AD_DPD:
            if (BOARD_IsLPAVOwnedByRTD())
            {
                upower_ps_mask_t ps_mask =
                    (upower_ps_mask_t)(kUPOWER_PS_HIFI4 | kUPOWER_PS_DDRC | kUPOWER_PS_MIPI_DSI | kUPOWER_PS_MIPI_CSI |
                                       kUPOWER_PS_AV_NIC | kUPOWER_PS_FUSION_AO);
                uint32_t mp0_mask =
                    (uint32_t)(kUPOWER_MP0_DCNANO_A | kUPOWER_MP0_DCNANO_B | kUPOWER_MP0_DMA2 | kUPOWER_MP0_HIFI4 |
                               kUPOWER_MP0_ISI | kUPOWER_MP0_MIPI_CSI | kUPOWER_MP0_MIPI_DSI | kUPOWER_MP0_AV_SYSTEM);

                if (BOARD_IsIpDisabled(IP_GPU) == false)
                {
                    ps_mask |= kUPOWER_PS_GPU3D;
                    mp0_mask |= kUPOWER_MP0_GPU2D_A | kUPOWER_MP0_GPU2D_B | kUPOWER_MP0_GPU3D_A | kUPOWER_MP0_GPU3D_B;
                }

                if (BOARD_IsIpDisabled(IP_EPDC) == false)
                {
                    ps_mask |= kUPOWER_PS_PXP_EPDC;
                    mp0_mask |= kUPOWER_MP0_EPDC_A | kUPOWER_MP0_EPDC_B | kUPOWER_MP0_PXP;
                }

                /*
                 * APD default hold LPAV domain device and needs change the ownership of the LPAV device
                 * from APD to RTD
                 */
                SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0;
                SIM_SEC->LPAV_SLAVE_ALLOC_CTRL &=
                    ~(SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI6(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI7(0x1) |
                      SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SEMA42(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_LPTPM8(0x1) |
                      SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SPDIF(0x1));

                UPOWER_PowerOffSwitches(ps_mask);

                UPOWER_PowerOffMemPart(mp0_mask, 0U);

                /*
                 * Workaround: After Mcore hold lpav in dualboot mode, Mcore can't resume from PD/DPD modes in some
                 * boards. Restore lpav master and slave devices can fix it.
                 */
                SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0x7f;
                SIM_SEC->LPAV_SLAVE_ALLOC_CTRL |=
                    (SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI6(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI7(0x1) |
                     SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SEMA42(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_LPTPM8(0x1) |
                     SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SPDIF(0x1));

                /* Configure BUCK3CTRL and LDO1_CFG according to PCA9460  */
                buck3_ctrl.reg.B3_ENMODE = 0x0; /* 00-OFF */
                buck3_ctrl.reg.B3_FPWM   = 0x0; /* Automatic PFM and PWM mode transition (default) */
                buck3_ctrl.reg.B3_AD     = 0x1; /* Enable Active discharge resistor when regulator is OFF (default) */
                buck3_ctrl.reg.B3_LPMODE = 0x3; /* Normal mode (default) */
                buck3_ctrl.reg.B3_RAMP   = 0x1; /* 25 mV (default) */

                ldo1_cfg.reg.L1_ENMODE = 0x0;   /* 00-OFF */
                ldo1_cfg.reg.L1_LPMODE = 0x3;   /* Normal mode (default) */
                ldo1_cfg.reg.L1_LLSEL  = 0x1;   /* 15 mw (default) */
                ldo1_cfg.reg.L1_CSEL   = 0x2;   /* Auto Cout detection (default) */

                /* Poweroff BUCK3 */
                UPOWER_SetPmicReg(PCA9460_BUCK3CTRL_ADDR, buck3_ctrl.val);
                /* Poweroff LDO1 */
                UPOWER_SetPmicReg(PCA9460_LDO1_CFG_ADDR, ldo1_cfg.val);
            }
            break;
    }
}

void APP_RebootCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateReboot, NULL);
    PRINTF("M33 reboot A35\r\n");
    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

void APP_SRTM_ShutdownCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateShutdown, NULL);

    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

/* WUU interrupt handler. */
void WUU0_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(WUU0_IRQn, irqHandlerParam);
    }
}

void BBNSM_IRQHandler(void)
{
    BaseType_t reschedule = pdFALSE;
    uint32_t status       = BBNSM_GetStatusFlags(BBNSM);

    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(BBNSM_IRQn, irqHandlerParam);
    }

    if (status & kBBNSM_EMG_OFF_InterruptFlag)
    {
        /* Clear emergency power off interrupt flag */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_EMG_OFF_InterruptFlag);
    }

    if (status & kBBNSM_PWR_OFF_InterruptFlag)
    {
        if (AD_CurrentMode == AD_DPD) /* Application Domain is in Deep Power Down Mode/Power Down Mode */
        {
            /* Wakeup A Core (A35) */
            APP_SRTM_WakeupCA35();
        }
        else if (AD_CurrentMode == AD_PD || (support_dsl_for_apd == true && AD_CurrentMode == AD_DSL))
        {
            /* Wakeup A Core (A35) */
            APP_WakeupACore();
        }
        /* Clear BBNSM button off interrupt */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_PWR_OFF_InterruptFlag);
    }
    else if (status & kBBNSM_PWR_ON_InterruptFlag)
    {
        /* Clear BBNSM button on interrupt */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_PWR_ON_InterruptFlag);
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

static void APP_SRTM_DoWakeup(void *param)
{
    APP_WakeupACore();
}

static void APP_SRTM_DoWakeupCA35(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    if (!core || (core && SRTM_PeerCore_GetState(core) == SRTM_PeerCore_State_Deactivated))
    {
        APP_SRTM_DoWakeup(param1);
        APP_SRTM_StartCommunication();
    }
}

static void APP_SRTM_PollLinkup(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    if (srtmState == APP_SRTM_StateRun)
    {
        if (rpmsg_lite_is_link_up(rpmsgHandle))
        {
            srtmState = APP_SRTM_StateLinkedUp;
            xSemaphoreGive(monSig);
        }
        else
        {
            /* Start timer to poll linkup status. */
            xTimerStart(linkupTimer, portMAX_DELAY);
        }
    }
}

static void APP_RefreshS400WdgTimerCallback(TimerHandle_t xTimer)
{
    SENTINEL_Ping();
    PRINTF("\r\n %s: %d ping s400 wdg timer ok\r\n", __func__, __LINE__);
    xTimerStart(refreshS400WdgTimer, portMAX_DELAY);
}

static void APP_RestoreRegValOfMuTimerCallback(TimerHandle_t xTimer)
{
    rtdCtxRestore();
    xTimerStart(restoreRegValOfMuTimer, portMAX_DELAY);
}

static void APP_ChngModeFromActToDslForApdTimerCallback(TimerHandle_t xTimer)
{
    if (core != NULL && support_dsl_for_apd == true && AD_WillEnterMode == AD_DSL)
    {
        AD_CurrentMode   = AD_DSL;
        AD_WillEnterMode = AD_ACT;
        SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Deactivated);
        PRINTF("AD entered Deep Sleep Mode\r\n");
    }
}

static void APP_LinkupTimerCallback(TimerHandle_t xTimer)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_PollLinkup, NULL, NULL);

    if (proc)
    {
        SRTM_Dispatcher_PostProc(disp, proc);
    }
}

static void APP_SRTM_NotifyPeerCoreReady(struct rpmsg_lite_instance *rpmsgHandle, bool ready)
{
    /* deinit and init app task(str_echo/pingpong rpmsg) in APP_SRTM_StateReboot only */
    if (rpmsgMonitor && (srtmState == APP_SRTM_StateReboot))
    {
        rpmsgMonitor(rpmsgHandle, ready, rpmsgMonitorParam);
    }
}

static void APP_SRTM_Linkup(void)
{
    srtm_channel_t chan;
    srtm_rpmsg_endpoint_config_t rpmsgConfig;

    /* Inform upower that m33 is using the ddr */
    UPOWER_SetRtdUseDdr(true);

    /* Create SRTM peer core */
    core = SRTM_PeerCore_Create(PEER_CORE_ID);
    /* Set peer core state to activated */
    SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Activated);

    /* Common RPMsg channel config */
    rpmsgConfig.localAddr   = RL_ADDR_ANY;
    rpmsgConfig.peerAddr    = RL_ADDR_ANY;
    rpmsgConfig.rpmsgHandle = rpmsgHandle;

    /* Create and add SRTM Life Cycle channel to peer core */
    rpmsgConfig.epName = APP_SRTM_LFCL_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    SRTM_Dispatcher_AddPeerCore(disp, core);
}

static void APP_SRTM_InitPeerCore(void)
{
    copyResourceTable();

    rpmsgHandle = rpmsg_lite_remote_init((void *)RPMSG_LITE_SRTM_SHMEM_BASE, RPMSG_LITE_SRTM_LINK_ID, RL_NO_FLAGS);
    assert(rpmsgHandle);

    /* save context, such as: MU0_MUA[RCR] */
    rtdCtxSave();

    APP_SRTM_NotifyPeerCoreReady(rpmsgHandle, true);

    if (rpmsg_lite_is_link_up(rpmsgHandle))
    {
        APP_SRTM_Linkup();
    }
    else
    {
        /* Start timer to poll linkup status. */
        xTimerStart(linkupTimer, portMAX_DELAY);
    }
}

static void APP_SRTM_ResetServices(void)
{
    /* When CA35 resets, we need to avoid async event to send to CA35. Audio and IO services have async events. */
}

static void APP_SRTM_DeinitPeerCore(void)
{
    /* Stop linkupTimer if it's started. */
    xTimerStop(linkupTimer, portMAX_DELAY);

    /* Notify application for the peer core disconnection. */
    APP_SRTM_NotifyPeerCoreReady(rpmsgHandle, false);

    if (core)
    {
        /* Need to let services know peer core is now down. */
        APP_SRTM_ResetServices();

        SRTM_Dispatcher_RemovePeerCore(disp, core);
        SRTM_PeerCore_Destroy(core);
        core = NULL;
    }

    if (rpmsgHandle)
    {
        rpmsg_lite_deinit(rpmsgHandle);
        rpmsgHandle = NULL;
    }

    /* Inform upower that m33 is not using the ddr(it's ready to reset ddr of lpavd) */
    UPOWER_SetRtdUseDdr(false);
}

/*
 * uboot-spl will assign LPAV domain owner to APD in singleboot and to RTD in dualboot/low power boot, and assign LPAV
 * devices to APD in three boot mode.
 * APP_SRTM_EnableLPAV and APP_SRTM_DisableLPAV function logic is for dualboot/low power boot mode.
 * After system exit low power mode, assurance that restore ownership of LPAV to initial cold start.
 */
void APP_SRTM_EnableLPAV()
{
    UPOWER_ReadPmicReg(PCA9460_BUCK3CTRL_ADDR, &(buck3_ctrl.val));
    UPOWER_ReadPmicReg(PCA9460_LDO1_CFG_ADDR, &(ldo1_cfg.val));

    if (!(ldo1_cfg.reg.L1_ENMODE))
    {
        upower_ps_mask_t ps_mask = (upower_ps_mask_t)(kUPOWER_PS_HIFI4 | kUPOWER_PS_DDRC | kUPOWER_PS_MIPI_DSI |
                                                      kUPOWER_PS_MIPI_CSI | kUPOWER_PS_AV_NIC | kUPOWER_PS_FUSION_AO);
        upower_ps_mask_t ps_mask_workaround =
            (upower_ps_mask_t)(kUPOWER_PS_HIFI4 | kUPOWER_PS_AV_NIC | kUPOWER_PS_FUSION_AO);
        uint32_t mp0_mask =
            (uint32_t)(kUPOWER_MP0_DCNANO_A | kUPOWER_MP0_DCNANO_B | kUPOWER_MP0_DMA2 | kUPOWER_MP0_HIFI4 |
                       kUPOWER_MP0_ISI | kUPOWER_MP0_MIPI_CSI | kUPOWER_MP0_MIPI_DSI | kUPOWER_MP0_AV_SYSTEM);

        if (BOARD_IsIpDisabled(IP_GPU) == false)
        {
            ps_mask |= kUPOWER_PS_GPU3D;
            ps_mask_workaround |= kUPOWER_PS_GPU3D;
            mp0_mask |= kUPOWER_MP0_GPU2D_A | kUPOWER_MP0_GPU2D_B | kUPOWER_MP0_GPU3D_A | kUPOWER_MP0_GPU3D_B;
        }

        if (BOARD_IsIpDisabled(IP_EPDC) == false)
        {
            ps_mask |= kUPOWER_PS_PXP_EPDC;
            mp0_mask |= kUPOWER_MP0_EPDC_A | kUPOWER_MP0_EPDC_B | kUPOWER_MP0_PXP;
        }

        UPOWER_SetRtdUseDdr(true);

        if (!BOARD_IsLPAVOwnedByRTD())
        {
            /* Set LPAV ownership to RTD */
            SIM_SEC->SYSCTRL0 &= ~SIM_SEC_SYSCTRL0_LPAV_MASTER_CTRL(1);
        }

        /* APD default hold LPAV domain device and needs change the ownership of the LPAV device from APD to RTD */
        SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0;
        SIM_SEC->LPAV_SLAVE_ALLOC_CTRL &=
            ~(SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI6(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI7(0x1) |
              SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SEMA42(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_LPTPM8(0x1) |
              SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SPDIF(0x1));

        /* Power on LPAV domain */
        UPOWER_PowerOnMemPart(mp0_mask, 0U);

        UPOWER_PowerOnSwitches(ps_mask);

        /* Restore BUCK3 voltage to 1.1V, please refer to PCA9460 for the specific data */
        UPOWER_SetPmicReg(PCA9460_BUCK3OUT_DVS0_ADDR, 0x28);

        ldo1_cfg.reg.L1_ENMODE = 0x1; /* 00-ON */
        ldo1_cfg.reg.L1_LPMODE = 0x3; /* Normal mode (default) */
        ldo1_cfg.reg.L1_LLSEL  = 0x1; /* 15 mw (default) */
        ldo1_cfg.reg.L1_CSEL   = 0x2; /* Auto Cout detection (default) */

        /* Poweron LDO1 */
        UPOWER_SetPmicReg(PCA9460_LDO1_CFG_ADDR, ldo1_cfg.val);

        /* Init LPAV clocks */
        BOARD_LpavInit();

        /* Workaround: If DDR controller register read failed, need to toggle pS7, PS8, PS13, PS14 */
        while (!LPDDR->DENALI_CTL[0])
        {
            /* Toggle pS7, PS8, PS13, PS14 */
            UPOWER_PowerOffSwitches(ps_mask_workaround);
            UPOWER_PowerOnSwitches(ps_mask_workaround);
        }

        /* Control DRAM exit from self-refesh */
        BOARD_DramExitRetention(dram_class, dram_timing_cfg);

        /*
         * Restore LPAV devices owner to APD, RTD need keep hold LPAV domain owner in SIM_SEC_SYSCTRL0_LPAV_MASTER_CTRL
         * to ensure DDR exit retention state after RTD exit Power down for dualboot and low power boot mode
         */
        SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0x7f;
        SIM_SEC->LPAV_SLAVE_ALLOC_CTRL |=
            (SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI6(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI7(0x1) |
             SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SEMA42(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_LPTPM8(0x1) |
             SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SPDIF(0x1));
    }
}

void APP_SRTM_DisableLPAV()
{
    UPOWER_ReadPmicReg(PCA9460_BUCK3CTRL_ADDR, &(buck3_ctrl.val));
    UPOWER_ReadPmicReg(PCA9460_LDO1_CFG_ADDR, &(ldo1_cfg.val));

    if (ldo1_cfg.reg.L1_ENMODE)
    {
        upower_ps_mask_t ps_mask = (upower_ps_mask_t)(kUPOWER_PS_HIFI4 | kUPOWER_PS_DDRC | kUPOWER_PS_MIPI_DSI |
                                                      kUPOWER_PS_MIPI_CSI | kUPOWER_PS_AV_NIC | kUPOWER_PS_FUSION_AO);
        uint32_t mp0_mask =
            (uint32_t)(kUPOWER_MP0_DCNANO_A | kUPOWER_MP0_DCNANO_B | kUPOWER_MP0_DMA2 | kUPOWER_MP0_HIFI4 |
                       kUPOWER_MP0_ISI | kUPOWER_MP0_MIPI_CSI | kUPOWER_MP0_MIPI_DSI | kUPOWER_MP0_AV_SYSTEM);

        if (BOARD_IsIpDisabled(IP_GPU) == false)
        {
            ps_mask |= kUPOWER_PS_GPU3D;
            mp0_mask |= kUPOWER_MP0_GPU2D_A | kUPOWER_MP0_GPU2D_B | kUPOWER_MP0_GPU3D_A | kUPOWER_MP0_GPU3D_B;
        }

        if (BOARD_IsIpDisabled(IP_EPDC) == false)
        {
            ps_mask |= kUPOWER_PS_PXP_EPDC;
            mp0_mask |= kUPOWER_MP0_EPDC_A | kUPOWER_MP0_EPDC_B | kUPOWER_MP0_PXP;
        }
        /* DDR will enter retention state when LPAV master domain enter Power down */
        BOARD_DramEnterRetention();

        ldo1_cfg.reg.L1_ENMODE = 0x0; /* 00-OFF */
        ldo1_cfg.reg.L1_LPMODE = 0x3; /* Normal mode (default) */
        ldo1_cfg.reg.L1_LLSEL  = 0x1; /* 15 mw (default) */
        ldo1_cfg.reg.L1_CSEL   = 0x2; /* Auto Cout detection (default) */

        /* Poweroff LDO1 */
        UPOWER_SetPmicReg(PCA9460_LDO1_CFG_ADDR, ldo1_cfg.val);

#ifndef SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
        /* For sai_low_power_audio, reduce buck3 power will impact fucntion */
        /* Set BUCK3 voltage to 0.7375V, please refer to PCA9460 for the specific data */
        UPOWER_SetPmicReg(PCA9460_BUCK3OUT_DVS0_ADDR, 0x0B);
#endif

        if (!BOARD_IsLPAVOwnedByRTD())
        {
            /* Set LPAV ownership to RTD */
            SIM_SEC->SYSCTRL0 &= ~SIM_SEC_SYSCTRL0_LPAV_MASTER_CTRL(1);
        }

        /* APD default hold LPAV domain device and needs change the ownership of the LPAV device from APD to RTD */
        /*
         * LPAV_MASTER_ALLOC_CTRL bit fields mismatch between REVC and REVD.
         * SIM_SEC->LPAV_MASTER_ALLOC_CTRL &= ~(SIM_SEC_LPAV_MASTER_ALLOC_CTRL_DCNANO(0x1) |
         * SIM_SEC_LPAV_MASTER_ALLOC_CTRL_MIPI_DSI(0x1));
         */
        SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0;
        SIM_SEC->LPAV_SLAVE_ALLOC_CTRL &=
            ~(SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI6(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI7(0x1) |
              SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SEMA42(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_LPTPM8(0x1) |
              SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SPDIF(0x1));

        UPOWER_PowerOffSwitches(ps_mask);
        UPOWER_PowerOffMemPart(mp0_mask, 0U);

        /*
         * Workaround: After Mcore hold lpav in dualboot mode, Mcore can't resume from PD/DPD modes in some boards.
         * Restore lpav master and slave devices can fix it.
         */
        SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0x7f;
        SIM_SEC->LPAV_SLAVE_ALLOC_CTRL |=
            (SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI6(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI7(0x1) |
             SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SEMA42(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_LPTPM8(0x1) |
             SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SPDIF(0x1));

        UPOWER_SetRtdUseDdr(false);
    }
}

#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
/*
 * For sai low power audio demo, when Acore enter Power down mode, Mcore will
 * put ddr into self-refresh-->transfer data by dma and play audio-->put ddr exit self-refresh-->memcpy data from ddr to
 * ssram
 */
void APP_SRTM_PreCopyCallback()
{
    if (R32(DDR_IN_SELFREFRESH_BASE) || (SRTM_PeerCore_GetState(core) == SRTM_PeerCore_State_Deactivated))
    {
        /* Disable GAIO BBNSM IRQ before operate LPAV */
        DisableIRQ(BBNSM_IRQn);
        DisableIRQ(GPIOA_INT0_IRQn);
        DisableIRQ(GPIOA_INT1_IRQn);
        DisableIRQ(GPIOB_INT0_IRQn);
        DisableIRQ(GPIOB_INT1_IRQn);
        DisableIRQ(WUU0_IRQn);

        /* Poweron LPAV domain, ddr exit retention */
        APP_SRTM_EnableLPAV();
        W32(DDR_IN_SELFREFRESH_BASE, 0);

        EnableIRQ(GPIOA_INT0_IRQn);
        EnableIRQ(GPIOA_INT1_IRQn);
        EnableIRQ(GPIOB_INT0_IRQn);
        EnableIRQ(GPIOB_INT1_IRQn);
        EnableIRQ(BBNSM_IRQn);
        EnableIRQ(WUU0_IRQn);
    }
}

void APP_SRTM_PostCopyCallback()
{
    if (SRTM_PeerCore_GetState(core) == SRTM_PeerCore_State_Deactivated)
    {
        /* Disable GAIO BBNSM IRQ before operate LPAV */
        DisableIRQ(BBNSM_IRQn);
        DisableIRQ(GPIOA_INT0_IRQn);
        DisableIRQ(GPIOA_INT1_IRQn);
        DisableIRQ(GPIOB_INT0_IRQn);
        DisableIRQ(GPIOB_INT1_IRQn);
        DisableIRQ(WUU0_IRQn);

        /* Poweroff LPAV domain, put ddr into retention */
        APP_SRTM_DisableLPAV();

        /* In dualboot mode, RTD hold LPAV domain, set LPAV ownership to APD let ddr into retention */
        SIM_SEC->SYSCTRL0 |= SIM_SEC_SYSCTRL0_LPAV_MASTER_CTRL(1);
        SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0x7f;
        SIM_SEC->LPAV_SLAVE_ALLOC_CTRL |=
            (SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI6(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SAI7(0x1) |
             SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SEMA42(0x1) | SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_LPTPM8(0x1) |
             SIM_SEC_LPAV_SLAVE_ALLOC_CTRL_SPDIF(0x1));

        W32(DDR_IN_SELFREFRESH_BASE, 1);

        EnableIRQ(GPIOA_INT0_IRQn);
        EnableIRQ(GPIOA_INT1_IRQn);
        EnableIRQ(GPIOB_INT0_IRQn);
        EnableIRQ(GPIOB_INT1_IRQn);
        EnableIRQ(BBNSM_IRQn);
        EnableIRQ(WUU0_IRQn);
    }
}
#endif

static void APP_SRTM_A35ResetHandler(void)
{
    portBASE_TYPE taskToWake = pdFALSE;

    /* disable interrupt */
    MU_DisableInterrupts(MU0_MUA, kMU_ResetAssertInterruptEnable);

    srtmState = APP_SRTM_StateReboot;

    /* Wake up monitor to reinitialize the SRTM communication with CA35 */
    if (pdPASS == xSemaphoreGiveFromISR(monSig, &taskToWake))
    {
        portYIELD_FROM_ISR(taskToWake);
    }
}

/* Make sure that XRDC has setup access permission for M Core(M Core can access registers of CMC_AD), unless M Core will
 * get hardfault(CMC_AD is belongs to Application Domain) */
static void CMC_ADClrAD_PSDORF(CMC_AD_Type *base, uint32_t flag)
{
    base->AD_PSDORF = flag; /* Write 1 to clear flag */
}

/*
 * MU Interrrupt RPMsg handler
 */
#ifdef MU0_A_IRQHandler
#undef MU0_A_IRQHandler
#endif

int32_t MU0_A_IRQHandler(void)
{
    uint32_t status = MU_GetStatusFlags(MU0_MUA);

    if (status & kMU_OtherSideEnterPowerDownInterruptFlag) /* PD/DPD mode */
    {
        SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Deactivated);

        PRINTF("AD entered PD(linux suspend to ram)/DPD(linux shutdown) mode\r\n");
        MU_ClearStatusFlags(MU0_MUA, (uint32_t)kMU_OtherSideEnterPowerDownInterruptFlag);

        if (AD_WillEnterMode == AD_DPD)
        {
            AD_CurrentMode = AD_WillEnterMode; /* AD entered Deep Power Down Mode */
            NVIC_ClearPendingIRQ(CMC1_IRQn);
            EnableIRQ(CMC1_IRQn);
            /* Help A35 to setup TRDC after A35 entered deep power down moade */
            BOARD_SetTrdcAfterApdReset();
            /*
             *  When RTD is the ower of LPAV and APD enter PD mode, RTD need poweroff LDO1, reduce BUCK3 to 0.73V,
             *  Set flag to put ddr into retention when RTD enter PD mode.
             *  otherwise APD side is responsible to control them in PD mode.
             */
            srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_SetLPAV, (void *)AD_DPD, NULL);

            assert(proc);
            SRTM_Dispatcher_PostProc(disp, proc);
        }
        else
        {
            /* Relase A Core */
            MU_BootOtherCore(
                MU0_MUA,
                (mu_core_boot_mode_t)0); /* Delete the code after linux supported sending suspend rpmsg to M Core */
            AD_CurrentMode = AD_PD;      /* AD entered Power Down Mode */
            /*
             *  When RTD is the ower of LPAV and APD enter PD mode, RTD need poweroff LDO1 and BUCK3
             *  otherwise APD side is responsible to control them in DPD mode
             */
            srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_SetLPAV, (void *)AD_PD, NULL);

            assert(proc);
            SRTM_Dispatcher_PostProc(disp, proc);
        }
        AD_WillEnterMode = AD_ACT;
    }

    return RPMsg_MU0_A_IRQHandler();
}

// clang-format off
/*
 * [AD_CurrentMode, AD_WillEnterMode] state machine:
 *       +----(uboot reset)---+
 *       |                    |
 *       |                    |
 *       v                    |
 * [AD_UNKOWN, AD_UNKOWN](A Core in uboot)<--+
 *    ^      |                               |
 *    |      |                        (linux reboot)
 *    |    (boot from uboot to linux)        |        +------------(linux resume from suspend)--+
 *    |      |                               |        |                                         |
 *    |      |                               |        v                                         |
 *    |      +--------------------->[AD_ACT, AD_UNKOWN] -----(linux suspend to mem)---->[AD_PD, AD_ACT]
 *    |                                        |
 *    |                                        |
 *    |                                  (linux poweroff)
 *    |                                        |
 *    |                                        v
 *    |                                [AD_DPD, AD_ACT]
 *    |                                        |
 *    |                                        |
 *    +---(A Core is woken by wakeup source)---+
 */   /* Note: When AD is [AD_DPD, AD_ACT],option V will not enter APP_SRTM_A35ResetHandler,execute reset logic,
       * but keep the process of boot A Core*/
// clang-format on
void CMC1_IRQHandler(void)
{
    apd_boot_cnt++;
    DisableIRQ(CMC1_IRQn);
    NVIC_ClearPendingIRQ(CMC1_IRQn);
    rtdCtxRestore();

    if ((AD_CurrentMode == AD_DPD && AD_WillEnterMode == AD_ACT && !option_v_boot_flag) ||
        (apd_boot_cnt > 1 && AD_CurrentMode == AD_UNKOWN && AD_WillEnterMode == AD_UNKOWN) ||
        (AD_CurrentMode == AD_ACT && AD_WillEnterMode == AD_UNKOWN))
    {
        APP_SRTM_A35ResetHandler();
    }
    if (AD_CurrentMode == AD_DPD && AD_WillEnterMode == AD_ACT)
    {
        AD_CurrentMode   = AD_UNKOWN;
        AD_WillEnterMode = AD_UNKOWN;
    }
    if (AD_CurrentMode == AD_PD && AD_WillEnterMode == AD_ACT)
    {
        PRINTF("\r\nAD resume from Power Down Mode\r\n");

        /* hold A core for next reboot */
        MU_HoldOtherCoreReset(MU0_MUA);
    }
}

static srtm_status_t APP_SRTM_LfclEventHandler(
    srtm_service_t service, srtm_peercore_t core, srtm_lfcl_event_t event, void *eventParam, void *userParam)
{
    switch (event)
    {
        case SRTM_Lfcl_Event_ShutdownReq: /* Notify M Core that Application Domain will enter Deep Power Down Mode */
            AD_WillEnterMode = AD_DPD;
            /* Relase A Core */
            MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
            PRINTF("\r\nAD will enter Deep Power Down Mode\r\n");
            break;
        case SRTM_Lfcl_Event_SuspendReq: /* Notify M Core that Application Domain will enter Power Down Mode */
            /* Save context(such as: MU0_MUA[RCR]) */
            rtdCtxSave();

            if (support_dsl_for_apd != true)
            {
                AD_WillEnterMode = AD_PD;
                PRINTF("\r\nAD will enter Power Down Mode\r\n");
            }
            else
            {
                AD_WillEnterMode = AD_DSL;
                xTimerStart(chngModeFromActToDslForApdTimer,
                            portMAX_DELAY); /* No way to check whether apd entered deep sleep mode, so start a timer to
                                               change mode from active mode to deep sleep mode for AD */
                PRINTF("\r\nAD Will enter Deep Sleep Mode\r\n");
            }

            /* Relase A Core */
            MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
            break;
        case SRTM_Lfcl_Event_WakeupReq:
            /* If already deactivated, power on CA35, else CA35 will not power off,
               and wakeup will defer until CA35 enter Power Down */
            APP_SRTM_DoWakeupCA35(NULL, NULL, NULL);
            break;
        case SRTM_Lfcl_Event_Running: /* Notify M Core that Application Domain entered Active Mode */
            /* enable CMC1 IRQ */
            CMC_ADClrAD_PSDORF(
                CMC_AD, CMC_AD_AD_PSDORF_AD_PERIPH(
                            1)); /* need clear it, unless A Core cannot reboot after A Core suspend and resume back */
            NVIC_ClearPendingIRQ(CMC1_IRQn);
            EnableIRQ(CMC1_IRQn);
            rtdCtxRestore();
            AD_CurrentMode   = AD_ACT;
            AD_WillEnterMode = AD_UNKOWN;
            PRINTF("\r\nAD entered active mode\r\n");
            break;
        default:
            PRINTF("\r\n%s: %d unsupported event: 0x%x\r\n", __func__, __LINE__, event);
            break;
    }

    return SRTM_Status_Success;
}

static void APP_SRTM_InitLfclService(struct board_descr *bdescr)
{
    srtm_service_t service;

    PRINTF("APP_SRTM: Start %s\r\n", __func__);
    /* Create and register Life Cycle service */
    service = SRTM_LfclService_Create();
    SRTM_LfclService_Subscribe(service, APP_SRTM_LfclEventHandler, NULL);
    SRTM_Dispatcher_RegisterService(disp, service);
}

static void APP_SRTM_InitServices(struct board_descr *bdescr)
{
    APP_SRTM_InitLfclService(bdescr);
}

void APP_PowerOffCA35(void)
{
    UPOWER_PowerOffSwitches((upower_ps_mask_t)(kUPOWER_PS_A35_0 | kUPOWER_PS_A35_1 | kUPOWER_PS_L2_CACHE |
                                               kUPOWER_PS_AD_NIC | kUPOWER_PS_AD_PER));

    AD_CurrentMode = AD_DPD;
}

static void APP_PowerOnCA35(void)
{
    MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
    UPOWER_PowerOnSwitches((upower_ps_mask_t)(kUPOWER_PS_A35_0 | kUPOWER_PS_A35_1 | kUPOWER_PS_L2_CACHE |
                                              kUPOWER_PS_AD_NIC | kUPOWER_PS_AD_PER));
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void SRTM_MonitorTask(void *pvParameters)
{
    app_srtm_state_t state = APP_SRTM_StateInit;
    struct board_descr *bdescr;

    PRINTF("APP_SRTM: %s is running\r\n", __func__);

    bdescr = get_board_description();

    /* Initialize services and add to dispatcher */
    APP_SRTM_InitServices(bdescr);
    /* Start SRTM dispatcher */
    SRTM_Dispatcher_Start(disp);

    /* Monitor peer core state change */
    while (true)
    {
        xSemaphoreTake(monSig, portMAX_DELAY);

        if (state == srtmState)
        {
            continue;
        }

        switch (srtmState)
        {
            case APP_SRTM_StateRun:
                assert(state == APP_SRTM_StateShutdown);
                PRINTF("Start SRTM communication\r\n");
                SRTM_Dispatcher_Stop(disp);
                /* Power On A Core when SoC is in low power boot type or option_v_boot_flag=true
                 * The purpose of using option_v_boot_flag is to avoid entering the APP_SRTM_A35ResetHandler
                 * in the CMC1_IRQHandler and interrupt the startup process during the process of starting Acore. */
                if (BOARD_IsLowPowerBootType() || option_v_boot_flag)
                {
                    DisableIRQ(CMC1_IRQn);
                    MU_Init(MU0_MUA);
                    if (option_v_boot_flag)
                    {
                        APP_WakeupACore();
                    }
                    else
                    {
                        APP_PowerOnCA35();
                    }
                }
 
                /*
                 * NODE: Only Single-BootFlow is supported.
                 * Therefore The U-Boot handshake is done,
                 * before SRTM starts up to get board-cfgs
                 */
                if (state != APP_SRTM_StateInit && BOARD_HandshakeWithUboot())
                {
                    /* CMC1(CMC_AD) is belongs to Application Domain, so if want to access these registers of CMC1, pls
                     * make sure that mcore can access CMC1(mcore can access CMC1 after BOARD_HandshakeWithUboot) */
                    CMC_ADClrAD_PSDORF(
                        CMC_AD,
                        CMC_AD_AD_PSDORF_AD_PERIPH(
                            1)); /* need clear it, unless A Core cannot reboot after A Core suspend and resume back */
                }

                /* enable CMC1 interrupt after handshake with uboot(M Core cannot access CMC1 that belongs to
                 * Application Domain when Power On Reset; M Core can access CMC1 after uboot(running on A Core) enable
                 * accessing permission of CMC1 by XRDC) */
                EnableIRQ(CMC1_IRQn);

                APP_SRTM_InitPeerCore();
                SRTM_Dispatcher_Start(disp);
                NVIC_ClearPendingIRQ(CMC1_IRQn);
                EnableIRQ(CMC1_IRQn);
                option_v_boot_flag = false;
                state              = APP_SRTM_StateRun;
                break;

            case APP_SRTM_StateLinkedUp:
                if (state == APP_SRTM_StateRun)
                {
                    PRINTF("Handle Peer Core Linkup\r\n\r\n");
                    SRTM_Dispatcher_Stop(disp);
                    APP_SRTM_Linkup();
                    AD_CurrentMode   = AD_ACT;
                    AD_WillEnterMode = AD_UNKOWN;
                    SRTM_Dispatcher_Start(disp);
                }
                break;
            case APP_SRTM_StateShutdown:
                PRINTF("#### Shutdown CA35 ####\r\n");
                assert(state == APP_SRTM_StateRun);

                SRTM_Dispatcher_Stop(disp);
                /* Remove peer core from dispatcher */
                APP_SRTM_DeinitPeerCore();
                /* dispatcher can still handle proc message during peer core shutdown */
                SRTM_Dispatcher_Start(disp);

                /* Shutdown CA35 domain power */
                PRINTF("#### Power off CA35 ####\r\n");
                APP_PowerOffCA35();
                state = APP_SRTM_StateShutdown;
                break;
            case APP_SRTM_StateReboot:
                assert(state == APP_SRTM_StateRun);

                PRINTF("Handle Peer Core Reboot\r\n");

                SRTM_Dispatcher_Stop(disp);
                /* Remove peer core from dispatcher */
                APP_SRTM_DeinitPeerCore();

                /* enable clock of MU0_MUA before accessing registers of MU0_MUA */
                MU_Init(MU0_MUA);

                /* Force peer core in reset */
                if (need_reset_peer_core)
                {
                    MU_HardwareResetOtherCore(MU0_MUA, true, true, kMU_CoreBootFromAddr0);
                    need_reset_peer_core = false;
                }

                /* Help A35 to setup TRDC before release A35 */
                BOARD_SetTrdcAfterApdReset();

                /* Release peer core from reset */
                MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);

                if (BOARD_HandshakeWithUboot() == true)
                {
                    /* CMC1(CMC_AD) is belongs to Application Domain, so if want to access these registers of CMC1, pls
                     * make sure that mcore can access CMC1(mcore can access CMC1 after BOARD_HandshakeWithUboot) */
                    CMC_ADClrAD_PSDORF(
                        CMC_AD,
                        CMC_AD_AD_PSDORF_AD_PERIPH(
                            1)); /* need clear it, unless A Core cannot reboot after A Core suspend and resume back */
                }
                /* Initialize peer core and add to dispatcher */
                APP_SRTM_InitPeerCore();

                /* Restore srtmState to Run. */
                srtmState = APP_SRTM_StateRun;

                SRTM_Dispatcher_Start(disp);

                NVIC_ClearPendingIRQ(CMC1_IRQn);
                EnableIRQ(CMC1_IRQn);

                /* hold A core for next reboot */
                MU_HoldOtherCoreReset(MU0_MUA);

                /* Do not need to change state. It's still Run. */
                break;

            default:
                assert(false);
                break;
        }
    }
}

void APP_ShutdownCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateShutdown, NULL);

    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

void APP_BootCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateRun, NULL);

    assert(proc);
    /* Fresh power up: Need SRTM monitor to prepare communication */
    SRTM_Dispatcher_PostProc(disp, proc);
}

static void SRTM_DispatcherTask(void *pvParameters)
{    
    PRINTF("APP_SRTM: %s is running\r\n", __func__);

    SRTM_Dispatcher_Run(disp);
}

static void APP_SRTM_InitPeriph(bool resume)
{
}

void APP_SRTM_Init(void)
{
    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);

    APP_SRTM_InitPeriph(false);

    monSig = xSemaphoreCreateBinary();
    assert(monSig);

    /* Note: Create a task to refresh S400(sentinel) watchdog timer to keep S400 alive, the task will be removed after
     * the bug is fixed in soc A1 */
    SENTINEL_Init();
    refreshS400WdgTimer = xTimerCreate("refreshS400WdgTimer", APP_MS2TICK(APP_REFRESH_S400_WDG_TIMER_PERIOD_MS),
                                       pdFALSE, NULL, APP_RefreshS400WdgTimerCallback);
    assert(refreshS400WdgTimer);
    xTimerStart(refreshS400WdgTimer, portMAX_DELAY);

    restoreRegValOfMuTimer =
        xTimerCreate("restoreRegValOfMuTimer", APP_MS2TICK(100), pdFALSE, NULL, APP_RestoreRegValOfMuTimerCallback);
    assert(restoreRegValOfMuTimer);
    xTimerStart(restoreRegValOfMuTimer, portMAX_DELAY);

    chngModeFromActToDslForApdTimer = xTimerCreate("chngModeFromActToDslForApdTimer", APP_MS2TICK(300), pdFALSE, NULL,
                                                   APP_ChngModeFromActToDslForApdTimerCallback);
    assert(chngModeFromActToDslForApdTimer);

    linkupTimer =
        xTimerCreate("Linkup", APP_MS2TICK(APP_LINKUP_TIMER_PERIOD_MS), pdFALSE, NULL, APP_LinkupTimerCallback);
    assert(linkupTimer);

    /* Create SRTM dispatcher */
    disp = SRTM_Dispatcher_Create();

    NVIC_SetPriority(CMC1_IRQn, APP_CMC1_IRQ_PRIO);

    MU_Init(MU0_MUA);
    MU_EnableInterrupts(MU0_MUA, kMU_OtherSideEnterPowerDownInterruptEnable);
    rtdCtxSave(); /* try to save CIRE0 */

    /* hold A core for next reboot */
    MU_HoldOtherCoreReset(MU0_MUA);

    xTaskCreate(SRTM_DispatcherTask, "SRTM dispatcher", 512U, NULL, APP_SRTM_DISPATCHER_TASK_PRIO, NULL);
    xTaskCreate(SRTM_MonitorTask, "SRTM monitor", 256U, NULL, APP_SRTM_MONITOR_TASK_PRIO, NULL);
}

void APP_SRTM_StartCommunication(void)
{
    srtmState = APP_SRTM_StateRun;
    xSemaphoreGive(monSig);
}

void APP_SRTM_Suspend(void)
{
}

void APP_SRTM_Resume(bool resume)
{
    /* TODO: */
}

void APP_SRTM_SetRpmsgMonitor(app_rpmsg_monitor_t monitor, void *param)
{
    rpmsgMonitor      = monitor;
    rpmsgMonitorParam = param;
}

void APP_SRTM_HandlePeerReboot(void)
{
    if (srtmState != APP_SRTM_StateShutdown)
    {
        srtmState = APP_SRTM_StateReboot;
        xSemaphoreGive(monSig);
    }
}

void APP_SRTM_SetIRQHandler(app_irq_handler_t handler, void *param)
{
    irqHandler      = handler;
    irqHandlerParam = param;
}

void APP_SRTM_WakeupCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_DoWakeupCA35, NULL, NULL);

    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

static void APP_SRTM_DoSetWakeupModule(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    uint32_t module                          = (uint32_t)param1;
    wuu_internal_wakeup_module_event_t event = (wuu_internal_wakeup_module_event_t)(uint32_t)param2;

    WUU_SetInternalWakeUpModulesConfig(WUU0, module, event);
}

void APP_SRTM_SetWakeupModule(uint32_t module, uint16_t event)
{
    srtm_procedure_t proc;

    proc = SRTM_Procedure_Create(APP_SRTM_DoSetWakeupModule, (void *)module, (void *)(uint32_t)event);
    assert(proc);

    SRTM_Dispatcher_CallProc(disp, proc, SRTM_WAIT_FOR_EVER);
    SRTM_Procedure_Destroy(proc);
}

static void APP_SRTM_DoClrWakeupModule(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    uint32_t module                          = (uint32_t)param1;
    wuu_internal_wakeup_module_event_t event = (wuu_internal_wakeup_module_event_t)(uint32_t)param2;

    WUU_ClearInternalWakeUpModulesConfig(WUU0, module, event);
}

void APP_SRTM_ClrWakeupModule(uint32_t module, uint16_t event)
{
    srtm_procedure_t proc;

    proc = SRTM_Procedure_Create(APP_SRTM_DoClrWakeupModule, (void *)module, (void *)(uint32_t)event);
    assert(proc);

    SRTM_Dispatcher_CallProc(disp, proc, SRTM_WAIT_FOR_EVER);
    SRTM_Procedure_Destroy(proc);
}

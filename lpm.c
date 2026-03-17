#include "lpm.h"
#include "fsl_upower.h"
#include "fsl_rtd_cmc.h"
#include <board/board.h>

/* RTD Power Down Power mode */
static ps_rtd_pwr_mode_cfgs_t rtd_pwr_mode_cfgs = {
    /* PD */
    [PD_RTD_PWR_MODE] =
        {
            .in_reg_cfg = IN_REG_CFG(0x00000000, 0x00000000),
            .pmic_cfg   = PMIC_CFG(0x00000023, 0x00000000),
            .pad_cfg    = PAD_CFG(0x00000003, 0x00000000, 0x00000000),
            .mon_cfg    = MON_CFG(0x00000000, 0x0, 0x0),
            /*
             * bias mode: 0b00 - NBB, 0b01 - RBB, 0b10 - AFBB, 0b11 - ARBB
             * 0x0000 0001
             *   |  | |  |
             *   +--+ +--+
             *   |    |
             *   |    +--> RTD bias mode: RBB
             *   +--> LPAV bias mode: NBB
             *
             * RTD rbbn/rbbp: 0b00010 - 100 mV
             *           ...
             *           0b11010 - 1300 mV
             * LPAV rbbn/rbbp: 0b0 - 1000 mV, 0b1 - 1300 mV
             * 0x0001 001a
             *   |  | |  |
             *   +--+ +--+
             *   |    |
             *   |    +--> RTD rbbn: 0b11010 - 1300 mV
             *   +--> LPAV rbbn: 0b1 - 1300 mV
             *
             * LPAV rbbn/rbbp: 0b0 - 1000 mV, 0b1 - 1300 mV
             * 0x0001 001a
             *   |  | |  |
             *   +--+ +--+
             *   |    |
             *   |    +--> RTD rbbp: 0b11010 - 1300 mV
             *   +--> LPAV rbbp: 0b1 - 1300 mV
             */
            .bias_cfg       = BIAS_CFG(0x00000001, 0x0001001a, 0x0001001a, 0x00000001),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
    /* DPD */
    [DPD_RTD_PWR_MODE] =
        {
            /* Reuse power down mode's settings */
            .in_reg_cfg = IN_REG_CFG(
                0x00000000, 0x00000000), /* vol = 0[595.833mv], mode = 0x0 [regulator is off] internal regulator ? */
            .pmic_cfg = PMIC_CFG(0x00000023, 0x00000000),
            .pad_cfg  = PAD_CFG(0x00000003, 0x00000000, 0x00000000), /* PAD_CLOSE0(PTA) ~ PAD_CLOSE1(PTB) is isolated */
            .mon_cfg  = MON_CFG(0x00000000, 0x0, 0x0),
            .bias_cfg = BIAS_CFG(0x00000001, 0x0001001a, 0x0001001a, 0x00000001),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
    /* DSL */
    [DSL_RTD_PWR_MODE] =
        {
            .in_reg_cfg = IN_REG_CFG(0x1c, 0),
            .pmic_cfg   = PMIC_CFG(0x23, 0x0),
            // clang-format off
	    /*
	     * For PTA/PTB/PTE/PTF
	     * control                 pad state
	     * pad_reset   pad_close
	     * 0           0           Functional
	     * 0           1           State Retention
	     * 1           0/1         Weak0 Drive
	     *
	     * For PTC/PTD
	     * control                 pad state
	     * pad_tqsleep
	     * 0                       Functional
	     * 1                       Functional(Degradated Performance ?)
	     */
             /*
	      * PAD_CLOSE0(PTA) = 1, PAD_RESET0 = 0: PTA is in State Retention,[PTA/PTB/PTE/PTF are Fail Safe GPIO, PTC/PTD are High Speed GPIO, PTA/PTB/PTC is used by RTD, PTD/PTE/PTF are used by APD/LPAV]
	      * PAD_CLOSE1(PTB) = 1, PAD_RESET1 = 0: PTB is in State Retention.
	      * PAD_TQ_SLEEP0(PTC)
	      * PAD_TQ_SLEEP1(PTD)
	      * PAD_CLOSE2(PTE)
	      * PAD_CLOSE3(PTF)
	      */
            // clang-format on
            .pad_cfg        = PAD_CFG(0x3, 0x0, 0x0), /* PTA: State Retention, PTB: State Retention */
            .mon_cfg        = MON_CFG(0x0, 0x0, 0x0),
            .bias_cfg       = BIAS_CFG(0x00000001, 0x0001001a, 0x0001001a, 0x00000001),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
    /* SLP */
    [SLP_RTD_PWR_MODE] =
        {
            .in_reg_cfg     = IN_REG_CFG(0x1c, 0x3),
            .pmic_cfg       = PMIC_CFG(0x23, 0x901),
            .pad_cfg        = PAD_CFG(0x0, 0x0, 0x0),
            .mon_cfg        = MON_CFG(0x0deb3a00, 0x0, 0x0),
            .bias_cfg       = BIAS_CFG(0x00000000, 0x00000002, 0x00000002, 0x00000000),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
    /* ACT */
    [ACT_RTD_PWR_MODE] =
        {
            .in_reg_cfg     = IN_REG_CFG(0x0000001c, 0x3),
            .pmic_cfg       = PMIC_CFG(0x23, 0x00000000),
            .pad_cfg        = PAD_CFG(0x0, 0x0, 0x0),
            .mon_cfg        = MON_CFG(0x0, 0x0, 0x0),
            .bias_cfg       = BIAS_CFG(0x00020002, 0x00000002, 0x00000002, 0x0000000),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
};

static ps_rtd_swt_cfgs_t rtd_swt_cfgs = {
    /* PD */
    [PD_RTD_PWR_MODE] =
        {
            .swt_board[0] = SWT_BOARD(0x0, 0x00060003),
            .swt_mem[0]   = SWT_MEM(0x003fe000, 0x0, 0x003ff3ff),
            .swt_mem[1]   = SWT_MEM(0x00000000, 0x00000000, 0x00000000),
        },
    /* DPD */
    [DPD_RTD_PWR_MODE] =
        {
            /*
             * Reuse Power Down Mode's settings
             * on = 0x0 = 0b0000 0000 0000 0000 0000
             * mask = 0x60003 = 0b0110 0000 0000 0000 0011
             * switch close(on Power Switch):
             * switch open(off Power Switch): PS0(RTD A, RTD B), PS1(Fusion), PS17(Fusion AO),PS18(FUSE)
             */
            .swt_board[0] = SWT_BOARD(0x0, 0x00060003),
            .swt_mem[0]   = SWT_MEM(0x003fe000, 0x0, 0x003ff3ff),
            .swt_mem[1]   = SWT_MEM(0x00000000, 0x00000000, 0x00000000),
        },
    /* DSL */
    [DSL_RTD_PWR_MODE] =
        {
            .swt_board[0] = SWT_BOARD(0x1, 0x60003),
            .swt_mem[0]   = SWT_MEM(0x0, 0x0, 0xfffe0000),
            .swt_mem[1]   = SWT_MEM(0x003fffff, 0x003fffff, 0x003ff3ff),
        },
    /* SLP */
    [SLP_RTD_PWR_MODE] =
        {
            .swt_board[0] = SWT_BOARD(0x00060003, 0x0007ff83),
            .swt_mem[0]   = SWT_MEM(0x0, 0x0, 0xfffe0000),
            .swt_mem[1]   = SWT_MEM(0x003fffff, 0x003fffff, 0x003ff3ff),
        },
    /* ACT */
    [ACT_RTD_PWR_MODE] =
        {
            /*
             * RTD ACT settings
             * swt_board bit can refer into upower_ps_mask_t, power on PS0(RTD A, RTD B), PS1(Fusion), PS17(Fusion AO),PS18(FUSE).
             * swt_mem[1] refer into upower_mp1_mask_t, power on RTD domain periperal which are needed.
             * such as PS1(DMA0) to assure PDM edma adapter service is available after RTD resume from low power mode.
             */
            .swt_board[0] = SWT_BOARD(0x00060003, 0x0007ff83),
            .swt_mem[0]   = SWT_MEM(0x0, 0x0, 0x0),
            .swt_mem[1]   = SWT_MEM(0x003fff2a, 0x003fff2a, 0x003fffff),
        },
};

/* Deep sleep mode, reduce BUCK2OUT_DVS0 to 0.75V */
static ps_rtd_pmic_reg_data_cfgs_t rtd_pmic_reg_cfgs_dsl = {
    /* RTD DeepSleep: set BUCK2OUT_DVS0 to 0.75V */
    [0] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = DSL_RTD_PWR_MODE,
            .i2c_addr   = 0x15, /* BUCK2OUT_DVS0 of PCA9460 */
            .i2c_data   = 0x0C, /* 0.75 */
        },
    /* RTD Active: set BUCK2OUT_DVS0 to 1.0V */
    [1] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = ACT_RTD_PWR_MODE,
            .i2c_addr   = 0x15,
            .i2c_data   = 0x20,
        },
};

/* In order to reduce power consumption, poweroff LSW1, reduce BUCK2OUT_DVS0 to 0.65 V */
ps_rtd_pmic_reg_data_cfgs_t rtd_pmic_reg_cfgs_pd = {
    [0] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = PD_RTD_PWR_MODE,
            .i2c_addr   = 0x15, /* BUCK2OUT_DVS0 of PCA9460 */
            .i2c_data   = 0x04, /* 0.65 V */
        },
    /* RTD Power Down: off LSW1 */
    [1] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = PD_RTD_PWR_MODE,
            .i2c_addr   = 0x40, /* LSW1_CTRL of PCA9460 */
            .i2c_data   = 0x0,  /* LSW1_EN[1:0] = 00b(OFF) */
        },

    /* RTD Active: BUCK2OUT_DVS0 to 1.0 V  */
    [2] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = ACT_RTD_PWR_MODE,
            .i2c_addr   = 0x15, /* BUCK2OUT_DVS0 of PCA9460 */
            .i2c_data   = 0x20, /* 1.0 V */
        },
    /* RTD Active: on LSW1 */
    [3] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = ACT_RTD_PWR_MODE,
            .i2c_addr   = 0x40, /* LSW1_CTRL of PCA9460 */
            .i2c_data   = 0x11, /* LSW1_EN[1:0] = 01b(ON at RUN state) */
        },
};

/* In order to reduce power consumption, poweroff LSW1, reduce BUCK2OUT_DVS0 to 0.65 V */
ps_rtd_pmic_reg_data_cfgs_t rtd_pmic_reg_cfgs_dpd = {
    /* RTD Deep Power Down: BUCK2OUT_DVS0 to 0.65 V */
    [0] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = DPD_RTD_PWR_MODE,
            .i2c_addr   = 0x15, /* BUCK2OUT_DVS0 of PCA9460 */
            .i2c_data   = 0x04, /* 0.65 V */
        },
    /* RTD Deep Power Down: off LSW1 */
    [1] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = DPD_RTD_PWR_MODE,
            .i2c_addr   = 0x40, /* LSW1_CTRL of PCA9460 */
            .i2c_data   = 0x0,  /* LSW1_EN[1:0] = 00b(OFF) */
        },
    /* RTD Active: BUCK2OUT_DVS0 to 1.0 V  */
    [2] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = ACT_RTD_PWR_MODE,
            .i2c_addr   = 0x15, /* BUCK2OUT_DVS0 of PCA9460 */
            .i2c_data   = 0x20, /* 1.0 V */
        },
    /* RTD Active: on LSW1 */
    [3] =
        {
            .tag        = PMIC_REG_VALID_TAG,
            .power_mode = ACT_RTD_PWR_MODE,
            .i2c_addr   = 0x40, /* LSW1_CTRL of PCA9460 */
            .i2c_data   = 0x11, /* LSW1_EN[1:0] = 01b(ON at RUN state) */
        },
};

bool LPM_SystemSleep(void)
{
    /* Initialize upower config data */
    struct ps_pwr_mode_cfg_t *pwr_sys_cfg = (struct ps_pwr_mode_cfg_t *)UPWR_DRAM_SHARED_BASE_ADDR;

    /* Copy upower config data to target memory */
    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[SLP_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[SLP_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[SLP_RTD_PWR_MODE], &rtd_swt_cfgs[SLP_RTD_PWR_MODE], sizeof(swt_config_t));

    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[ACT_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[ACT_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[ACT_RTD_PWR_MODE], &rtd_swt_cfgs[ACT_RTD_PWR_MODE], sizeof(swt_config_t));

    for (uint32_t rtd_mode = 0; rtd_mode < NUM_RTD_PWR_MODES; rtd_mode++)
    {
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_board =
            (struct upwr_switch_board_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_board);
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_mem =
            (struct upwr_mem_switches_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_mem);
    }

    RTDCMC_SetPowerModeProtection(CMC_RTD, CMC_RTD_PMPROT_AS_MASK);
    RTDCMC_EnterLowPowerMode(CMC_RTD, kRTDCMC_SleepMode);

    return true;
}

bool LPM_SystemDeepSleep(void)
{
    /* Initialize upower config data */
    struct ps_pwr_mode_cfg_t *pwr_sys_cfg = (struct ps_pwr_mode_cfg_t *)UPWR_DRAM_SHARED_BASE_ADDR;

    /* Copy upower config data to target memory */
    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[DSL_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[DSL_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[DSL_RTD_PWR_MODE], &rtd_swt_cfgs[DSL_RTD_PWR_MODE], sizeof(swt_config_t));

    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[ACT_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[ACT_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[ACT_RTD_PWR_MODE], &rtd_swt_cfgs[ACT_RTD_PWR_MODE], sizeof(swt_config_t));

    memcpy(&pwr_sys_cfg->ps_rtd_pmic_reg_data_cfg, &rtd_pmic_reg_cfgs_dsl, sizeof(ps_rtd_pmic_reg_data_cfgs_t));

    for (uint32_t rtd_mode = 0; rtd_mode < NUM_RTD_PWR_MODES; rtd_mode++)
    {
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_board =
            (struct upwr_switch_board_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_board);
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_mem =
            (struct upwr_mem_switches_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_mem);
    }

    /*
     * Switch clock source from PLL0/1 to FRO to save more power during Power Down,
     * then switch back after wakeup in function BOARD_ResumeClockInit()
     */
    BOARD_SwitchToFROClk(BOARD_GetRtdDriveMode());
    BOARD_DisablePlls();

    RTDCMC_SetClockMode(CMC_RTD, kRTDCMC_GateAllSystemClocks);
    RTDCMC_SetPowerModeProtection(CMC_RTD, CMC_RTD_PMPROT_ADS_MASK);
    RTDCMC_EnableDebugOperation(CMC_RTD, false);
    RTDCMC_EnterLowPowerMode(CMC_RTD, kRTDCMC_DeepSleepMode);

    /* Clear clock mode after exiting from deep sleep mode */
    RTDCMC_SetClockMode(CMC_RTD, kRTDCMC_GateNoneClock);

    return true;
}

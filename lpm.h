/*
 * Copyright 2021, NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _LPM_H_
#define _LPM_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef NOR_CMD_LUT_SEQ_IDX_CONFIG
#define NOR_CMD_LUT_SEQ_IDX_CONFIG 6
#endif

// #define SYSTICK_BASE       LPTMR0
// #define SYSTICK_IRQn       LPTMR0_IRQn
// #define SYSTICK_HANDLER    LPTMR0_IRQHandler
// #define SYSTICK_CLOCK_NAME kCLOCK_Lptmr0

#define IN_REG_CFG(v, m)          \
    {                             \
        .volt = (v), .mode = (m), \
    }

#define PMIC_CFG(v, m)            \
    {                             \
        .volt = (v), .mode = (m), \
    }
#define PAD_CFG(c, r, t)                                       \
    {                                                          \
        .pad_close = (c), .pad_reset = (r), .pad_tqsleep = (t) \
    }
#define MON_CFG(hvd_en, lvd_en, lvdlvl)                                        \
    {                                                                          \
        .mon_hvd_en = (hvd_en), .mon_lvd_en = (lvd_en), .mon_lvdlvl = (lvdlvl) \
    }
#define BIAS_CFG(m, n, p, mbias) \
    {                            \
        .dombias_cfg =           \
            {                    \
                .mode = (m),     \
                .rbbn = (n),     \
                .rbbp = (p),     \
            },                   \
        .membias_cfg = {mbias},  \
    }
#define PWRSYS_LPM_CFG(m) \
    {                     \
        .lpm_mode = (m),  \
    }
#define SWT_BOARD(swt_on, msk)         \
    {                                  \
        .on = (swt_on), .mask = (msk), \
    }
#define SWT_MEM(a, p, m)                         \
    {                                            \
        .array = (a), .perif = (p), .mask = (m), \
    }


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

bool LPM_SystemDeepSleep(void);

bool LPM_SystemSleep(void);


#if defined(__cplusplus)
}
#endif /* __cplusplus*/

#endif /* _LPM_H_ */

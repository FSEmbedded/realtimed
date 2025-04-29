/*
 *Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "board/board_pwm.h"
#include "fsl_common.h"
#include "fsl_reset.h"
#include <asm-generic/errno.h>
#include "fsl_tpm.h"
#include "fsl_debug_console.h"

static struct pwm_chip *get_chip_from_idx(struct pwm_adapter *pwm_adapter, uint8_t chipId)
{
    struct pwm_chip *pwm_chip = pwm_adapter->pwm_chip;
    uint8_t num_chips = pwm_adapter->num_chips;
    uint8_t i;

    for(i = 0; i < num_chips; i++){
        if(pwm_chip[i].chipId == chipId)
            return &pwm_chip[i];
    }

    PRINTF("ERROR: PWM chipId %d not registered\r\n", chipId);
    return  NULL;
}

static status_t _setPwm_hz(struct pwm_adapter *adapter,
                                uint8_t chipId,
                                uint8_t chnlNumber,
                                uint32_t pwmFreq_Hz,
                                uint32_t dutyCyclePercent,
                                tpm_pwm_level_select_t level,
                                bool enable) {
    tpm_pwm_mode_t mode = kTPM_CenterAlignedPwm;
    struct pwm_chip *pwm_chip;
    struct pwm_channel *channel;
    int ret;

    if(!adapter)
        return kStatus_InvalidArgument;

    pwm_chip = adapter->ops.get_chip_from_idx(adapter, chipId);
    if(!pwm_chip || chnlNumber >= pwm_chip->num_channels)
        return kStatus_InvalidArgument;

    channel = &pwm_chip->channels[chnlNumber];
    channel->pwmConfig.chnlNumber = chnlNumber;
    channel->pwmFreq_Hz = pwmFreq_Hz;
    channel->pwmConfig.dutyCyclePercent = dutyCyclePercent;
    channel->pwmConfig.level = level;
    channel->enable = enable;
    channel->mode = mode;
#if (defined(FSL_FEATURE_TPM_HAS_PAUSE_LEVEL_SELECT) && FSL_FEATURE_TPM_HAS_PAUSE_LEVEL_SELECT)
    channel->pwmConfig.pauseLevel = kTPM_ClearOnPause;
#endif

    /* Enable PWM */
    if(channel->enable){
        ret = TPM_SetupPwm((TPM_Type *)pwm_chip->dev.base_addr, &channel->pwmConfig,
                        1, mode, pwmFreq_Hz, CLOCK_GetTpmClkFreq(pwm_chip->dev.instance));
        if (ret != kStatus_Success){
            PRINTF("ERROR: TPM_SetupPwm failed %d\r\n", ret);
            return kStatus_InvalidArgument;
        }

        TPM_StartTimer((TPM_Type *)pwm_chip->dev.base_addr, kTPM_SystemClock);
        return kStatus_Success;
    }

    /* Disable PWM */
    TPM_StopTimer((TPM_Type *)pwm_chip->dev.base_addr);

    return kStatus_Success;
}


static status_t _setPwm_ns(struct pwm_adapter *adapter,
                                uint8_t chipId,
                                uint8_t chnlNumber,
                                uint32_t period_ns,
                                uint32_t dutyCycle_ns,
                                uint8_t polarity,
                                bool enable)
{
    uint64_t dutyCyclePercent_64;
    uint32_t dutyCyclePercent;
    uint32_t pwmFreq_Hz;
    tpm_pwm_level_select_t level;

    if(period_ns == 0)
        return kStatus_OutOfRange;

    dutyCyclePercent_64 = ((uint64_t)dutyCycle_ns * 100) / (uint64_t)period_ns;

    if (dutyCyclePercent_64 > UINT32_MAX)
        return kStatus_OutOfRange;

    dutyCyclePercent = (uint32_t) dutyCyclePercent_64;
    pwmFreq_Hz = SECOND_TO_NANOSECOND / period_ns;
    level = polarity ? kTPM_LowTrue : kTPM_HighTrue;
    return _setPwm_hz(adapter, chipId, chnlNumber, pwmFreq_Hz, dutyCyclePercent, level, enable);
}

static int _getPwm_hz(struct pwm_adapter *adapter,
                            uint8_t chipId,
                            uint8_t chnlNumber,
                            uint32_t *pwmFreq_Hz,
                            uint32_t *dutyCyclePercent,
                            tpm_pwm_level_select_t *level,
                            bool *enable)
{
    struct pwm_chip *pwm_chip;
    struct pwm_channel *channel;

    pwm_chip = adapter->ops.get_chip_from_idx(adapter, chipId);

    if(!pwm_chip || chnlNumber >= pwm_chip->num_channels)
        return kStatus_InvalidArgument;

    channel = &pwm_chip->channels[chnlNumber];

    *pwmFreq_Hz = channel->pwmFreq_Hz;
    *dutyCyclePercent = channel->pwmConfig.dutyCyclePercent;
    *level = channel->pwmConfig.level;
    *enable = channel->enable;

    return kStatus_Success;
}

static int _getPwm_ns(struct pwm_adapter *adapter,
                            uint8_t chipId,
                            uint8_t chnlNumber,
                            uint32_t *period_ns,
                            uint32_t *dutyCycle_ns,
                            uint8_t *polarity,
                            bool *enable)
{
    uint32_t pwmFreq_Hz, dutyCyclePercent;
    uint64_t dutyCycle_ns_64;
    tpm_pwm_level_select_t level;
    int ret;

    ret = _getPwm_hz(adapter, chipId, chnlNumber, &pwmFreq_Hz, &dutyCyclePercent, &level, enable);
    if (ret != kStatus_Success)
        return ret;

    if(!pwmFreq_Hz)
        return kStatus_InvalidArgument;

    switch(level){
        case kTPM_HighTrue:
            *polarity = PWM_POLARITY_NORMAL;
            break;
        case kTPM_LowTrue:
            *polarity = PWM_POLARITY_INVERSED;
            break;
        default:
            return kStatus_InvalidArgument;
            break;
    }

    *period_ns = SECOND_TO_NANOSECOND / pwmFreq_Hz;
    dutyCycle_ns_64 = ((uint64_t)dutyCyclePercent * (uint64_t)(*period_ns)) / (uint64_t)100;

    if (dutyCycle_ns_64 > UINT32_MAX){
        *period_ns = 0;
        *dutyCycle_ns = 0;
        return kStatus_OutOfRange;
    }

    *dutyCycle_ns = (uint32_t)dutyCycle_ns_64;
    return kStatus_Success;
}

int __init_pwm_handle(struct pwm_chip *pwm_chip, struct dev *tpm_dev, uint8_t chipId, uint8_t numChannels)
{
    tpm_config_t tpmInfo;

    memcpy(&pwm_chip->dev, tpm_dev, sizeof(struct dev));
    
    pwm_chip->chipId = chipId;
    pwm_chip->num_channels = numChannels;
    pwm_chip->channels = pvPortMalloc(sizeof(struct pwm_channel)*numChannels);
    if (!pwm_chip->channels)
            return -ENOMEM;

    CLOCK_SetIpSrcDiv(pwm_chip->dev.ip_name, pwm_chip->dev.ip_src, 1U, 0U);
  
    RESET_PeripheralReset(pwm_chip->dev.reset);
    
    TPM_GetDefaultConfig(&tpmInfo);

    /* Initialize TPM module */
    TPM_Init((TPM_Type *)pwm_chip->dev.base_addr, &tpmInfo);
    
    return 0;
}

int init_pwm_adapter(struct pwm_adapter *pwm_adapter, struct dev *tpm_devs, enum board_types btype)
{
    struct pwm_chip *pwm_chip;
    switch (btype)
    {
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
    case BT_PICOCOREMX8ULP:
    pwm_chip = pvPortMalloc(sizeof(struct pwm_chip));
        if (!pwm_chip)
            return -ENOMEM;

        pwm_adapter->num_chips = 1;
        pwm_adapter->pwm_chip = pwm_chip;
        __init_pwm_handle(&pwm_chip[0], &tpm_devs[3], 0, 6);
        break;
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */
#ifdef CONFIG_BOARD_OSMSFMX8ULP
    case BT_OSMSFMX8ULP:
    pwm_chip = pvPortMalloc(sizeof(struct pwm_chip) * 3);
        if (!pwm_chip)
        return -ENOMEM;
        pwm_adapter->num_chips = 3;
        pwm_adapter->pwm_chip = pwm_chip;
        /* PWM_2 */
        __init_pwm_handle(&pwm_chip[0], &tpm_devs[0], 2, 6);
        /* PWM_1 */
        __init_pwm_handle(&pwm_chip[1], &tpm_devs[1], 1, 2);
        /* PWM_3 & PWM_4 */
        __init_pwm_handle(&pwm_chip[2], &tpm_devs[2], 3, 2);
        break;
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */
#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
    case BT_ARMSTONEMX8ULP:
        break;
#endif
    default:
        break;
    }

    pwm_adapter->ops.get_chip_from_idx = &get_chip_from_idx;
    pwm_adapter->ops.setPwm_hz = &_setPwm_hz;
    pwm_adapter->ops.setPwm_ns = &_setPwm_ns;
    pwm_adapter->ops.getPwm_hz = &_getPwm_hz;
    pwm_adapter->ops.getPwm_ns = &_getPwm_ns;

    return 0;
}




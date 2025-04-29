/*
*Copyright (c) 2024 F&S Elektronik Systeme GmbH
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef __BOARD_PWM_H
#define __BOARD_PWM_H

#include <board/board_dev.h>
#include "fsl_tpm.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#define SECOND_TO_NANOSECOND  (1000000000ULL)

enum pwm_polarity {
    PWM_POLARITY_NORMAL = 0,
    PWM_POLARITY_INVERSED,
};

struct pwm_channel {
    tpm_chnl_pwm_signal_param_t pwmConfig;
    uint32_t pwmFreq_Hz;
    bool enable;
    tpm_pwm_mode_t mode;
};

struct pwm_chip {
    struct dev dev;
    uint8_t chipId;
    struct pwm_channel *channels;
    uint8_t num_channels;
};

struct pwm_adapter;
struct pwm_ops {

    /* TODO: Füge für jede Methode eine Dokumentation hinzu. (Doxygen) Kleines Bsp ist unten. Hover mit der Maus über die funktion, und sehe was passiert :)*/

    /**
     * @brief set the frequency in hearts and the DutyCicle in percent for a PWM channel
     * @param adapter ptr to pwm_adapter
     * @param chipId pwm chip ID
     * @param chnlNumber Channel Number to set
     * @param pwmFreq_Hz frequency in HZ
     * @param dutyCyclePercent duty cicle in percent
     * @param enable enable/disable PWM channel
     */
    status_t (*setPwm_hz)(struct pwm_adapter *adapter,
                           uint8_t chipId,
                           uint8_t chnlNumber,
                           uint32_t pwmFreq_Hz,
                           uint32_t dutyCyclePercent,
                           tpm_pwm_level_select_t level,
                           bool enable);

    /**
     * @brief set the period in ms and the DutyCicle in ms for a PWM channel
     * @param adapter ptr to pwm_adapter
     * @param chipId pwm chip ID
     * @param chnlNumber Channel Number to set
     * @param period_ms period in ms
     * @param dutyCycle_ms duty cicle in ms
     * @param polarity polarity of the signal (0 = normal, 1 = inverted)
     * @param enable enable/disable PWM channel
     */
    status_t (*setPwm_ns)(struct pwm_adapter *adapter,
                           uint8_t chipId,
                           uint8_t chnlNumber,
                           uint32_t period_ns,
                           uint32_t dutyCycle_ns,
                           uint8_t polarity,
                           bool enable);
    /**
     * @brief get the frequency in hearts and the DutyCicle in percent for a PWM channel
     * @param adapter ptr to pwm_adapter
     * @param chipId pwm chip ID
     * @param chnlNumber Channel Number to get
     * @param pwmFreq_Hz frequency in HZ
     * @param dutyCyclePercent duty cicle in percent
     * @param level level of the signal (0 = normal, 1 = inverted)
     * @param enable enable/disable PWM channel
     */
    int (*getPwm_hz)(struct pwm_adapter *adapter,
                    uint8_t chipId,
                    uint8_t chnlNumber,
                    uint32_t *pwmFreq_Hz,
                    uint32_t *dutyCyclePercent,
                    tpm_pwm_level_select_t *level,
                    bool *enable);

    /**
     * @brief get the period in ms and the DutyCicle in ms for a PWM channel
     * @param adapter ptr to pwm_adapter
     * @param chipId pwm chip ID
     * @param chnlNumber Channel Number to get
     * @param period_ms period in ms
     * @param dutyCycle_ms duty cicle in ms
     * @param polarity polarity of the signal (0 = normal, 1 = inverted)
     * @param enable enable/disable PWM channel
     */
    int (*getPwm_ns)(struct pwm_adapter *adapter,
                    uint8_t chipId,
                    uint8_t chnlNumber,
                    uint32_t *period_ns,
                    uint32_t *dutyCycle_ns,
                    uint8_t *polarity,
                    bool *enable);
    /**
     * @brief get the pwm chip from the adapter by chipId
     * @param adapter ptr to pwm_adapter
     * @param chipId pwm chip ID
     * @return pointer to pwm_chip
     */
    struct pwm_chip *(*get_chip_from_idx)(struct pwm_adapter *adapter, uint8_t chipId);
};


struct pwm_adapter {

    struct pwm_chip *pwm_chip;
    uint32_t num_chips;
    struct pwm_ops ops;
};

int init_pwm_adapter(struct pwm_adapter *pwm_adapter, struct dev *tpm_devs, enum board_types btype);

#endif
 
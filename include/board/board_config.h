/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

/* remove to disable Board-Support */
#define CONFIG_BOARD_PICOCOREMX8ULP
#define CONFIG_BOARD_OSMSFMX8ULP
#define CONFIG_BOARD_ARMSTONEMX8ULP

#ifdef CONFIG_BOARD_PICOCOREMX8ULP
#define PCORE_GPIOA_GPIO_J1_44  0
#define PCORE_GPIOA_I2C_IRQ_B   18
#define PCORE_GPIOA_I2C_RTC_IRQ 19
#define PCORE_GPIOA_GPIO_J1_46  24
#define PCORE_GPIOB_WLAN_HOST_WAKE  0
#define PCORE_GPIOB_WLAN_WAKE_HOST  1
#define PCORE_GPIOB_BT_IRQ          2
#define PCORE_GPIOC_GPIO_J2_84      0
#define PCORE_GPIOC_GPIO_J2_86      1
#define PCORE_GPIOC_GPIO_J2_88      5
#define PCORE_GPIOC_GPIO_J2_90      6
#define PCORE_GPIOC_GPIO_J2_92      11
#define PCORE_GPIOC_GPIO_J2_98      18
#define PCORE_GPIOC_GPIO_J2_100     19
#endif

/* Default settings */
#define GPIOA_NUM 25
#define GPIOB_NUM 16
#define GPIOC_NUM 25

#ifndef CONFIG_I2C_MUTEX_TAKE_MS
#define CONFIG_I2C_MUTEX_TAKE_MS 100
#endif

#define CONFIG_BOARD_I2C_DEFAULT_BAUD      100000U
#define CONFIG_BOARD_I3C_DEFAULT_BAUD      400000U

#ifndef CONFIG_BOARD_LPI2C0_BAUDRATE_HZ
#define CONFIG_BOARD_LPI2C0_BAUDRATE_HZ    CONFIG_BOARD_I2C_DEFAULT_BAUD
#endif

#ifndef CONFIG_BOARD_LPI2C1_BAUDRATE_HZ
#define CONFIG_BOARD_LPI2C1_BAUDRATE_HZ    CONFIG_BOARD_I2C_DEFAULT_BAUD
#endif

#ifndef CONFIG_BOARD_LPI2C2_BAUDRATE_HZ
#define CONFIG_BOARD_LPI2C2_BAUDRATE_HZ    CONFIG_BOARD_I2C_DEFAULT_BAUD
#endif

#ifndef CONFIG_BOARD_LPI2C3_BAUDRATE_HZ
#define CONFIG_BOARD_LPI2C3_BAUDRATE_HZ    CONFIG_BOARD_I2C_DEFAULT_BAUD
#endif

#ifndef CONFIG_BOARD_I3C0_BAUDRATE_HZ
#define CONFIG_BOARD_I3C0_BAUDRATE_HZ    CONFIG_BOARD_I3C_DEFAULT_BAUD
#endif

#ifndef CONFIG_BOARD_I3C1_BAUDRATE_HZ
#define CONFIG_BOARD_I3C1_BAUDRATE_HZ    CONFIG_BOARD_I3C_DEFAULT_BAUD
#endif

#ifndef I3C_RETRY_TIMES
#define I3C_RETRY_TIMES 500
#endif

#ifndef CONFIG_RGPIO_INTERRUPT_SEL
#define CONFIG_RGPIO_INTERRUPT_SEL kRGPIO_InterruptOutput2
#endif

#ifndef CONFIG_GPIO_INT_PRIO
#define CONFIG_GPIO_INT_PRIO (5U)
#endif

#ifndef CONFIG_GPIOA_IFACEID
#define CONFIG_GPIOA_IFACEID 0
#endif

#ifndef CONFIG_GPIOB_IFACEID
#define CONFIG_GPIOB_IFACEID 1
#endif

#ifndef CONFIG_GPIOC_IFACEID
#define CONFIG_GPIOC_IFACEID 2
#endif

#endif /* __BOARD_CONFIG_H */
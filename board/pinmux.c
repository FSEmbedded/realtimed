/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include <board/pinmux.h>

#ifdef CONFIG_BOARD_OSMSFMX8ULP
static void BOARD_InitLpuartOSM(void)
{
	/* UART-B*/
	IOMUXC_SetPinMux(IOMUXC_PTA2_LPUART0_TX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA2_LPUART0_TX,
				IOMUXC_PCR_PE_MASK |
				IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA3_LPUART0_RX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA3_LPUART0_RX,
				IOMUXC_PCR_PE_MASK |
				IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA16_LPUART0_CTS_B, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA16_LPUART0_CTS_B,
				IOMUXC_PCR_PE_MASK |
				IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA17_LPUART0_RTS_B, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA17_LPUART0_RTS_B,
				IOMUXC_PCR_PE_MASK |
				IOMUXC_PCR_PS_MASK);

	/* UART-C */
	IOMUXC_SetPinMux(IOMUXC_PTB2_LPUART2_TX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB2_LPUART2_TX,
				IOMUXC_PCR_PE_MASK |
				IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTB3_LPUART2_RX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB3_LPUART2_RX,
				IOMUXC_PCR_PE_MASK |
				IOMUXC_PCR_PS_MASK);

	/* UART-D */
	IOMUXC_SetPinMux(IOMUXC_PTA10_LPUART1_TX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA10_LPUART1_TX,
				IOMUXC_PCR_PE_MASK |
				IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA11_LPUART1_RX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA11_LPUART1_RX,
				IOMUXC_PCR_PE_MASK |
				IOMUXC_PCR_PS_MASK);
}
#endif /* CONFIG_BOARD_OSMSFMX8ULP */

#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
static void BOARD_InitLpuartArmStone(void)
{
	/* UART-B*/
	IOMUXC_SetPinMux(IOMUXC_PTA2_LPUART0_TX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA2_LPUART0_TX,
		IOMUXC_PCR_PE_MASK |
											IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA3_LPUART0_RX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA3_LPUART0_RX,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA16_LPUART0_CTS_B, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA16_LPUART0_CTS_B,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA17_LPUART0_RTS_B, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA17_LPUART0_RTS_B,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);

	/* UART-E */
	IOMUXC_SetPinMux(IOMUXC_PTB2_LPUART2_TX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB2_LPUART2_TX,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTB3_LPUART2_RX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB3_LPUART2_RX,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
}
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */

#ifdef CONFIG_BOARD_PICOCOREMX8ULP
static void BOARD_InitLpuartPCore(void)
{
	/* UART-B*/
	IOMUXC_SetPinMux(IOMUXC_PTA2_LPUART0_TX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA2_LPUART0_TX,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA3_LPUART0_RX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA3_LPUART0_RX,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA16_LPUART0_CTS_B, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA16_LPUART0_CTS_B,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA17_LPUART0_RTS_B, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA17_LPUART0_RTS_B,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);

/* UART-D */
	IOMUXC_SetPinMux(IOMUXC_PTA10_LPUART1_TX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA10_LPUART1_TX,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA11_LPUART1_RX, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA11_LPUART1_RX,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
}
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */

static void BOARD_InitLpuartPins(enum board_types btype) {
	switch(btype) {
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
		case BT_PICOCOREMX8ULP:
			BOARD_InitLpuartPCore();
			break;
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */
#ifdef CONFIG_BOARD_OSMSFMX8ULP
		case BT_OSMSFMX8ULP:
			BOARD_InitLpuartOSM();
			break;
#endif /* CONFIG_BOARD_OSMSFMX8ULP */
#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
		case BT_ARMSTONEMX8ULP:
			BOARD_InitLpuartArmStone();
			break;
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */
		default:
			break;
	}
}

#ifdef CONFIG_BOARD_OSMSFMX8ULP
static void BOARD_InitI2cOSM(void)
{
	/* I2C_B */
	IOMUXC_SetPinMux(IOMUXC_PTA14_I3C0_SCL, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA14_I3C0_SCL,
		IOMUXC_PCR_DSE_MASK |
		IOMUXC_PCR_SRE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA15_I3C0_SDA, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA15_I3C0_SDA,
		IOMUXC_PCR_DSE_MASK |
		IOMUXC_PCR_SRE_MASK);
	
	/* I2C_CAM */
	IOMUXC_SetPinMux(IOMUXC_PTB0_LPI2C2_SCL, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB0_LPI2C2_SCL,
		IOMUXC_PCR_ODE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTB1_LPI2C2_SDA, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB1_LPI2C2_SDA,
		IOMUXC_PCR_ODE_MASK);
}
#endif /* CONFIG_BOARD_OSMSFMX8ULP */

#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
static void BOARD_InitI2cArmStone(void)
{
	/* I2C_B */
	IOMUXC_SetPinMux(IOMUXC_PTA14_I3C0_SCL, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA14_I3C0_SCL,
		IOMUXC_PCR_DSE_MASK |
		IOMUXC_PCR_SRE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA15_I3C0_SDA, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA15_I3C0_SDA,
		IOMUXC_PCR_DSE_MASK |
		IOMUXC_PCR_SRE_MASK);
}
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */

#ifdef CONFIG_BOARD_PICOCOREMX8ULP
static void BOARD_InitI2cPCore(void)
{
	/* I2C_C*/
	IOMUXC_SetPinMux(IOMUXC_PTA8_LPI2C0_SCL, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA8_LPI2C0_SCL,
		IOMUXC_PCR_ODE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA9_LPI2C0_SDA, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA9_LPI2C0_SDA,
		IOMUXC_PCR_ODE_MASK);

	/* I2C_D */
	IOMUXC_SetPinMux(IOMUXC_PTA14_I3C0_SCL, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA14_I3C0_SCL,
		IOMUXC_PCR_DSE_MASK |
		IOMUXC_PCR_SRE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA15_I3C0_SDA, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA15_I3C0_SDA,
		IOMUXC_PCR_DSE_MASK |
		IOMUXC_PCR_SRE_MASK);
}
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */

static void BOARD_InitI2cPins(enum board_types btype) {
	switch(btype) {
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
		case BT_PICOCOREMX8ULP:
			BOARD_InitI2cPCore();
			break;
#endif
#ifdef CONFIG_BOARD_OSMSFMX8ULP
		case BT_OSMSFMX8ULP:
			BOARD_InitI2cOSM();
			break;
#endif /* CONFIG_BOARD_OSMSFMX8ULP */
#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
		case BT_ARMSTONEMX8ULP:
			BOARD_InitI2cArmStone();
			break;
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */
		default:
			break;
	}
}

static void BOARD_InitPmicI2cPins(void) {
	IOMUXC_SetPinMux(IOMUXC_PTB10_PMIC0_SDA, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB10_PMIC0_SDA,
		IOMUXC_PCR_ODE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTB11_PMIC0_SCL, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB11_PMIC0_SCL,
		IOMUXC_PCR_ODE_MASK);
}

static void BOARD_InitPmicModePins(void) {
	IOMUXC_SetPinMux(IOMUXC_PTB7_PMIC0_MODE2, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB7_PMIC0_MODE2,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTB8_PMIC0_MODE1, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB8_PMIC0_MODE1,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTB9_PMIC0_MODE0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTB9_PMIC0_MODE0,
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
}

#ifdef CONFIG_BOARD_OSMSFMX8ULP
static void BOARD_InitI2sOSM(void)
{
	IOMUXC_SetPinMux(IOMUXC_PTC10_I2S0_MCLK, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC10_I2S0_MCLK,
		IOMUXC_PCR_OBE_MASK |
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC9_I2S0_TX_FS, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC9_I2S0_TX_FS,
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC8_I2S0_TX_BCLK, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC8_I2S0_TX_BCLK,
		IOMUXC_PCR_DSE_MASK);

	/* I2S_A */
	IOMUXC_SetPinMux(IOMUXC_PTC4_I2S0_RXD0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC4_I2S0_RXD0,
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC7_I2S0_TXD0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC7_I2S0_TXD0,
		IOMUXC_PCR_DSE_MASK);

	/* I2S_B */
	IOMUXC_SetPinMux(IOMUXC_PTC5_I2S0_RXD1, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC5_I2S0_RXD1,
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC6_I2S0_TXD1, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC6_I2S0_TXD1,
		IOMUXC_PCR_DSE_MASK);
}
#endif /* CONFIG_BOARD_OSMSFMX8ULP */

#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
static void BOARD_InitI2sArmStone(void)
{
	IOMUXC_SetPinMux(IOMUXC_PTC10_I2S0_MCLK, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC10_I2S0_MCLK,
		IOMUXC_PCR_OBE_MASK |
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC9_I2S0_TX_FS, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC9_I2S0_TX_FS,
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC8_I2S0_TX_BCLK, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC8_I2S0_TX_BCLK,
		IOMUXC_PCR_DSE_MASK);

	/* I2S_A */
	IOMUXC_SetPinMux(IOMUXC_PTC4_I2S0_RXD0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC4_I2S0_RXD0,
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC7_I2S0_TXD0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC7_I2S0_TXD0,
		IOMUXC_PCR_DSE_MASK);
}
#endif

#ifdef CONFIG_BOARD_PICOCOREMX8ULP
static void BOARD_InitI2sPCore(void)
{
	/* Audio Intern */	 
	IOMUXC_SetPinMux(IOMUXC_PTC23_I2S1_MCLK, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC23_I2S1_MCLK,
		IOMUXC_PCR_OBE_MASK |
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC22_I2S1_TX_FS, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC22_I2S1_TX_FS,
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC21_I2S1_TX_BCLK, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC21_I2S1_TX_BCLK,
		IOMUXC_PCR_DSE_MASK);

	IOMUXC_SetPinMux(IOMUXC_PTC20_I2S1_TXD0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC20_I2S1_TXD0,
		IOMUXC_PCR_DSE_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC17_I2S1_RXD0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC17_I2S1_RXD0,
		IOMUXC_PCR_DSE_MASK);

	/* Audio_B */
	/**
	 * TODO:
	 */
}
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */

static void BOARD_InitI2sPins(enum board_types btype) {
		switch(btype) {
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
			case BT_PICOCOREMX8ULP:
				BOARD_InitI2sPCore();
				break;
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */
#ifdef CONFIG_BOARD_OSMSFMX8ULP
			case BT_OSMSFMX8ULP:
				BOARD_InitI2sOSM();
				break;
#endif /* CONFIG_BOARD_OSMSFMX8ULP */
#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
			case BT_ARMSTONEMX8ULP:
				BOARD_InitI2sArmStone();
				break;
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */
			default:
				break;
		}
}

#ifdef CONFIG_BOARD_OSMSFMX8ULP
static void BOARD_InitTpmOSM()
{
	/* PWM 1 */
	IOMUXC_SetPinMux(IOMUXC_PTA9_TPM1_CH1, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA9_TPM1_CH1,
		IOMUXC_PCR_DSE_MASK);
	
	/* PWM_2 */
	IOMUXC_SetPinMux(IOMUXC_PTA18_TPM0_CH3, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA18_TPM0_CH3,
		IOMUXC_PCR_DSE_MASK);
	
	/* PWM_3 */
	IOMUXC_SetPinMux(IOMUXC_PTC1_TPM2_CH0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC1_TPM2_CH0,
		IOMUXC_PCR_DSE_MASK);
	
	/* PWM_4 */
	IOMUXC_SetPinMux(IOMUXC_PTC2_TPM2_CH1, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC2_TPM2_CH1,
		IOMUXC_PCR_DSE_MASK);
	
	/* PWM_5 */
	IOMUXC_SetPinMux(IOMUXC_PTC12_TPM3_CH0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC12_TPM3_CH0,
		IOMUXC_PCR_DSE_MASK);
}
#endif /* CONFIG_BOARD_OSMSFMX8ULP */

#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
static void BOARD_InitTpmArmStone(void)
{
	/* PWM 1 */
	IOMUXC_SetPinMux(IOMUXC_PTA9_TPM1_CH1, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTA9_TPM1_CH1,
		IOMUXC_PCR_DSE_MASK);
	
	/* PWM 2 */
	IOMUXC_SetPinMux(IOMUXC_PTC2_TPM2_CH1, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC2_TPM2_CH1,
		IOMUXC_PCR_DSE_MASK);

	/* PWM 3 */
	IOMUXC_SetPinMux(IOMUXC_PTC12_TPM3_CH0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC12_TPM3_CH0,
		IOMUXC_PCR_DSE_MASK);
}
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */

#ifdef CONFIG_BOARD_PICOCOREMX8ULP
static void BOARD_InitTpmPCore(void)
{
	/* PWM */
	IOMUXC_SetPinMux(IOMUXC_PTC12_TPM3_CH0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_PTC12_TPM3_CH0,
		IOMUXC_PCR_DSE_MASK);
}
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */

static void BOARD_InitTpmPins(enum board_types btype)
{
	switch(btype) {
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
		case BT_PICOCOREMX8ULP:
			BOARD_InitTpmPCore();
			break;
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */
#ifdef CONFIG_BOARD_OSMSFMX8ULP
		case BT_OSMSFMX8ULP:
			BOARD_InitTpmOSM();
			break;
#endif /* CONFIG_BOARD_OSMSFMX8ULP */
#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
		case BT_ARMSTONEMX8ULP:
			BOARD_InitTpmArmStone();
			break;
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */
		default:
			break;
	}
}

#ifdef CONFIG_BOARD_PICOCOREMX8ULP
static void BOARD_InitIOPCore()
{
	/* PTA */
	IOMUXC_SetPinMux(IOMUXC_PTA0_PTA0, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTA0_PTA0, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA18_PTA18, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTA18_PTA18, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);

	IOMUXC_SetPinMux(IOMUXC_PTA19_PTA19, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTA19_PTA19, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTA24_PTA24, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTA24_PTA24, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);

	/* PTB */
	IOMUXC_SetPinMux(IOMUXC_PTB0_PTB0, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTB0_PTB0, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);

	IOMUXC_SetPinMux(IOMUXC_PTB1_PTB1, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTB1_PTB1, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTB2_PTB2, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTB2_PTB2, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	
	/* PTC */
	IOMUXC_SetPinMux(IOMUXC_PTC0_PTC0, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTC0_PTC0, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC1_PTC1, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTC1_PTC1, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);

	IOMUXC_SetPinMux(IOMUXC_PTC5_PTC5, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTC5_PTC5, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC6_PTC6, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTC6_PTC6, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC11_PTC11, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTC11_PTC11, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC18_PTC18, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTC18_PTC18, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
	IOMUXC_SetPinMux(IOMUXC_PTC19_PTC19, 0);
	IOMUXC_SetPinConfig(IOMUXC_PTC19_PTC19, 
		IOMUXC_PCR_PE_MASK |
		IOMUXC_PCR_PS_MASK);
}
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */

static void BOARD_InitIOPins(enum board_types btype)
{
	switch(btype) {
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
		case BT_PICOCOREMX8ULP:
			BOARD_InitIOPCore();
			break;
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */
#ifdef CONFIG_BOARD_OSMSFMX8ULP
		// case BT_OSMSFMX8ULP:
		// 	BOARD_InitIOOSM();
		// 	break;
#endif
#ifdef 
		// case BT_ARMSTONEMX8ULP:
		// 	BOARD_InitIOArmStone();
		// 	break;
#endif
		default:
			break;
	}
}

void BOARD_InitBootPins_pre(void)
{
	BOARD_InitPmicI2cPins();
	BOARD_InitPmicModePins();
}

void BOARD_InitBootPins(enum board_types btype)
{
	BOARD_InitLpuartPins(btype);
	BOARD_InitI2cPins(btype);
	BOARD_InitI2sPins(btype);
	BOARD_InitTpmPins(btype);
	BOARD_InitIOPins(btype);
}
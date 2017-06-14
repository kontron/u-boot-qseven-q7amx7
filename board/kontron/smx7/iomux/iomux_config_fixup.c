/*
 * Copyright (c) 2017, Kontron Europe GmbH
 *
 * IOMux configuration fixups for the Kontron Smarc SMX7 board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

#include "iMX7D_registers.h"

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_FixupPins
 * Description   : pin routing fixup settings
 *
 *END**************************************************************************/
void BOARD_FixupPins(void)
{
	/*
	 * UART fixups due to DTE configuration (default from Pins tool is DCE).
	 */
	HW_IOMUXC_UART4_RX_DATA_SELECT_INPUT_WR(IOMUXC_BASE, 0x00000005u);  /* IOMUXC_UART4_RX_DATA_SELECT_INPUT: Select SAI2_TX_BCLK pad for UART4_RX_DATA */
	HW_IOMUXC_UART5_RX_DATA_SELECT_INPUT_WR(IOMUXC_BASE, 0x00000001u);  /* IOMUXC_UART5_RX_DATA_SELECT_INPUT: Select I2C4_SDA pad for UART5_RX_DATA */
	HW_IOMUXC_UART6_RX_DATA_SELECT_INPUT_WR(IOMUXC_BASE, 0x00000003u);  /* IOMUXC_UART6_RX_DATA_SELECT_INPUT: Select ECSPI1_MOSI pad for UART6_RX_DATA */
	HW_IOMUXC_UART7_RX_DATA_SELECT_INPUT_WR(IOMUXC_BASE, 0x00000003u);  /* IOMUXC_UART7_RX_DATA_SELECT_INPUT: Select ECSPI2_MOSI pad for UART7_RX_DATA */

	/*
	 * Fix ENET2_RX_CLK_SELECT_INPUT DAISY register. Must select EPDC_SDCE1 pad
	 * for ENET2_RX_CLK. Output from Pins tool is wrong.
	 */
	HW_IOMUXC_ENET2_RX_CLK_SELECT_INPUT_WR(IOMUXC_BASE, 0x00000000u);
}

void hsic_1p2_regulator_out(void)
{
	/* 
	 * allow the GPC to override register settings.
	 * use XTALOSC_BASE which is 0x30360000 instead of PMU_BASE (0x30360200)
	 */
	HW_PMU_REG_HSIC_1P2_SET(XTALOSC_BASE, 0x80000000);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

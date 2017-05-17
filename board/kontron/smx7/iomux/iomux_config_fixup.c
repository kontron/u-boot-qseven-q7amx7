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
	 * UART fixups due to DTE configuration (default from Pins Tool is DCE).
	 */
	HW_IOMUXC_UART4_RX_DATA_SELECT_INPUT_WR(IOMUXC_BASE, 0x00000005u);  /* IOMUXC_UART4_RX_DATA_SELECT_INPUT: Select SAI2_TX_BCLK pad for UART4_RX_DATA */
	HW_IOMUXC_UART5_RX_DATA_SELECT_INPUT_WR(IOMUXC_BASE, 0x00000001u);  /* IOMUXC_UART5_RX_DATA_SELECT_INPUT: Select I2C4_SDA pad for UART5_RX_DATA */
	HW_IOMUXC_UART6_RX_DATA_SELECT_INPUT_WR(IOMUXC_BASE, 0x00000003u);  /* IOMUXC_UART6_RX_DATA_SELECT_INPUT: Select ECSPI1_MOSI pad for UART6_RX_DATA */
	HW_IOMUXC_UART7_RX_DATA_SELECT_INPUT_WR(IOMUXC_BASE, 0x00000003u);  /* IOMUXC_UART7_RX_DATA_SELECT_INPUT: Select ECSPI2_MOSI pad for UART7_RX_DATA */
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

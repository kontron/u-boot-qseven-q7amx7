/*
 * Copyright (C) 2017 Kontron Europe GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/arch/crm_regs.h>
#include <usb.h>
#include <usb/ehci-ci.h>
#include <dm.h>
#include <dm/platform_data/serial_mxc.h>

extern void BOARD_InitPins(void);
extern void BOARD_FixupPins(void);

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PAD_CTRL    (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU100KOHM)


#ifdef CONFIG_SYS_I2C_MXC
/* from ICMC: #define I2C_PAD MUX_PAD_CTRL(0x0001F858) */
#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX7D_PAD_UART1_RX_DATA__I2C1_SCL | I2C_PAD,
		.gpio_mode = MX7D_PAD_UART1_RX_DATA__GPIO4_IO0 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 0),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_UART1_TX_DATA__I2C1_SDA | I2C_PAD,
		.gpio_mode = MX7D_PAD_UART1_TX_DATA__GPIO4_IO1 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 1),
	},
};

/* I2C2 */
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C2_SCL__I2C2_SCL | I2C_PAD,
		.gpio_mode = MX7D_PAD_I2C2_SCL__GPIO4_IO10 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 10)
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C2_SDA__I2C2_SDA | I2C_PAD,
		.gpio_mode = MX7D_PAD_I2C2_SDA__GPIO4_IO11 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 11)
	}
};

/* I2C3 */
static struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C3_SCL__I2C3_SCL | I2C_PAD,
		.gpio_mode = MX7D_PAD_I2C3_SCL__GPIO4_IO12 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C3_SDA__I2C3_SDA | I2C_PAD,
		.gpio_mode = MX7D_PAD_I2C3_SDA__GPIO4_IO13 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

static struct i2c_pads_info i2c_pad_info4 = {
	.scl = {
		.i2c_mode = MX7D_PAD_LCD_DATA22__I2C4_SCL | I2C_PAD,
		.gpio_mode = MX7D_PAD_LCD_DATA22__GPIO3_IO27 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 27)
	},
	.sda = {
		.i2c_mode = MX7D_PAD_LCD_DATA23__I2C4_SDA | I2C_PAD,
		.gpio_mode = MX7D_PAD_LCD_DATA23__GPIO3_IO28 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 28)
	}
};

#endif

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
         return (bus == 2 && cs == 0) ? (IMX_GPIO_NR(6, 22)) : -1;
}

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

#if 0
static iomux_v3_cfg_t const usb_otg1_pads[] = {
	MX7D_PAD_GPIO1_IO05__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usb_otg2_pads[] = {
	MX7D_PAD_UART3_CTS_B__USB_OTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#endif

#ifdef CONFIG_VIDEO_MXS
static int setup_lcd(void)
{
	/* Reset LCD */
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 1);

	/* Set Brightness to high */
	gpio_direction_output(IMX_GPIO_NR(1, 1) , 1);

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC

#define USDHC1_CD_GPIO	IMX_GPIO_NR(5, 0)
#define USDHC1_PWR_GPIO	IMX_GPIO_NR(5, 2)
#define USDHC3_PWR_GPIO IMX_GPIO_NR(6, 11)

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = 1; /* Assume uSDHC3 emmc is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int i, ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc2                    USDHC3 (eMMC)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			gpio_request(USDHC1_CD_GPIO, "usdhc1_cd");
			gpio_direction_input(USDHC1_CD_GPIO);
			gpio_request(USDHC1_PWR_GPIO, "usdhc1_pwr");
			gpio_direction_output(USDHC1_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC1_PWR_GPIO, 1);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			gpio_request(USDHC3_PWR_GPIO, "usdhc3_pwr");
			gpio_direction_output(USDHC3_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC3_PWR_GPIO, 1);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}
#endif /* CONFIG_FSL_ESDHC */

#ifdef CONFIG_FEC_MXC
int board_eth_init(bd_t *bis)
{
	int ret;

	/* remove PHY reset */
	gpio_direction_output(IMX_GPIO_NR(3, 21), 1);
	
	ret = fecmxc_initialize_multi(bis, 0,
		CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
	if (ret)
		printf("FEC1 MXC: %s:failed\n", __func__);

	return ret;
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
		(IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK |
		 IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK), 0);

	return set_clk_enet(ENET_125MHz);
}


int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x21);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x7ea8);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x2f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x71b7);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#ifdef CONFIG_FSL_QSPI
int board_qspi_init(void)
{
	/* Set the clock */
	set_clk_qspi();

	return 0;
}
#endif

int board_early_init_f(void)
{
	BOARD_InitPins();
	BOARD_FixupPins();

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);
	setup_i2c(3, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info4);

#if 0
	imx_iomux_v3_setup_multiple_pads(usb_otg1_pads,
					 ARRAY_SIZE(usb_otg1_pads));
	imx_iomux_v3_setup_multiple_pads(usb_otg2_pads,
					 ARRAY_SIZE(usb_otg2_pads));
#endif

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_VIDEO_MXS
	/* setup_lcd(); */
#endif

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif

#ifdef CONFIG_MXC_SPI
       /* setup_spi(); */
#endif

	return 0;
}

int board_late_init(void)
{
	return 0;
}

int checkboard(void)
{
	printf("Board: Kontron SMX7 SMARC 2.0 Module\n");

	return 0;
}

#ifdef CONFIG_USB_EHCI_MX7
int board_usb_phy_mode(int port)
{
	if (port == 0)
		return USB_INIT_DEVICE;
	else
		return USB_INIT_HOST;
}
#endif

static struct mxc_serial_platdata mxc_serial_plat = {
	.reg = (struct mxc_uart *)CONFIG_MXC_UART_BASE,
	.use_dte = true,
};

U_BOOT_DEVICE(mxc_serial) = {
	.name = "serial_mxc",
	.platdata = &mxc_serial_plat,
};


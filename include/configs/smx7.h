/*
 * Copyright (C) 2017 Kontron Europe GmbH
 *
 * Configuration settings for the Kontron Smarc SMX7 board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SMX7_CONFIG_H
#define __SMX7_CONFIG_H

#ifdef CONFIG_SPL
#include "imx7_spl.h"

#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SYS_SPI_U_BOOT_OFFS      0x11000
#endif

#include "mx7_common.h"

#define CONFIG_MXC_UART_BASE            UART6_IPS_BASE_ADDR

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN           (32 * SZ_1M)


/******************************************************************************
 * Miscellaneous configurable options
 */
#define CONFIG_WATCHDOG
#define CONFIG_DBG_MONITOR

#define CONFIG_MISC_INIT_R
#ifndef CONFIG_SPL_BUILD
#define CONFIG_CMD_KBOARDINFO
#endif

#ifdef CONFIG_CMD_KBOARDINFO
#define CONFIG_KBOARDINFO_MODULE
/* #define CONFIG_KBOARDINFO_CARRIER */

#define CONFIG_EMB_EEP_SPI

#define CONFIG_EMB_EEP_SPI_SIZE         0x1000
#define CONFIG_EMB_EEP_SPI_OFFSET       0x0f0000
#define CONFIG_EMB_EEP_SPI_BUS          CONFIG_SF_DEFAULT_BUS
#define CONFIG_EMB_EEP_SPI_CS           CONFIG_SF_DEFAULT_CS
#define CONFIG_EMB_EEP_SPI_SPEED        CONFIG_SF_DEFAULT_SPEED
#define CONFIG_EMB_EEP_SPI_MODE         CONFIG_SF_DEFAULT_MODE

#define CONFIG_HAS_ETH0

#define CONFIG_SAP_NAME                 "SK-FIRM-UBOOT-SMX7"
#define CONFIG_SAP_NUM                  "1060-xxxx"
#endif

/******************************************************************************
 * Network
 */
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_ETHPRIME                 "FEC"
#define CONFIG_FEC_MXC_PHYADDR          0

#define CONFIG_PHYLIB
/* ENET1 */
#define IMX_FEC_BASE                    ENET_IPS_BASE_ADDR

/******************************************************************************
 * MMC Config
 */
#define CONFIG_SYS_FSL_ESDHC_ADDR       0

#ifndef CONFIG_SPL_BUILD
#define CONFIG_CMD_MMC_RAW_ECSD
#endif

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

/******************************************************************************
 * I2C Configs
 */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_I2C_MXC_I2C1         /* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2         /* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3         /* enable I2C bus 3 */
#define CONFIG_SYS_I2C_MXC_I2C4         /* enable I2C bus 4 */
#define CONFIG_SYS_I2C_SPEED            100000

#define CONFIG_SUPPORT_EMMC_BOOT        /* eMMC specific */
#define CONFIG_SYS_MMC_IMG_LOAD_PART    1

/******************************************************************************
 * USB Configs
 */
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC           (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS            0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 3
#define CONFIG_MX7_USB_HSIC_PORTSC      (PORT_PTS_HSIC | PORT_PTS_PTW)

#define CONFIG_USBD_HS
#define CONFIG_USB_FUNCTION_MASS_STORAGE

/******************************************************************************
 * Environment organization
 */

#if defined(CONFIG_SPI_FLASH) && !defined(CONFIG_MFG_TOOL)
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SECT_SIZE        (32 * 1024)

#define CONFIG_ENV_OFFSET           0x1c0000
#define CONFIG_ENV_SIZE             SZ_8K

#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_OFFSET_REDUND    0x1c8000
#define CONFIG_ENV_SIZE_REDUND      (CONFIG_ENV_SIZE)
#else
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE             SZ_8K
#endif


#ifdef CONFIG_IMX_BOOTAUX
/* Set to QSPI1 A flash at default */
#define CONFIG_SYS_AUXCORE_BOOTDATA 0x60000000

#define UPDATE_M4_ENV \
	"m4image=m4_qspi.bin\0" \
	"loadm4image=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${m4image}\0" \
	"update_m4_from_sd=" \
		"if sf probe 0:0; then " \
			"if run loadm4image; then " \
				"setexpr fw_sz ${filesize} + 0xffff; " \
				"setexpr fw_sz ${fw_sz} / 0x10000; "	\
				"setexpr fw_sz ${fw_sz} * 0x10000; "	\
				"sf erase 0x0 ${fw_sz}; " \
				"sf write ${loadaddr} 0x0 ${filesize}; " \
			"fi; " \
		"fi\0" \
	"m4boot=sf probe 0:0; bootaux "__stringify(CONFIG_SYS_AUXCORE_BOOTDATA)"\0"
#else
#define UPDATE_M4_ENV ""
#endif

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		"clk_ignore_unused "\
		"\0" \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \
	"bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \

#define CONFIG_DFU_ENV_SETTINGS \
	"dfu_alt_info=image raw 0 0x800000;"\
		"u-boot raw 0 0x4000;"\
		"bootimg part 0 1;"\
		"rootfs part 0 2\0" \

#define CONFIG_EXTRA_ENV_SETTINGS \
	UPDATE_M4_ENV \
	CONFIG_MFG_ENV_SETTINGS \
	CONFIG_DFU_ENV_SETTINGS \
	"autoload=no" "\0" \
	"bootm_boot_mode=sec" "\0" \
	"boot_fdt=try" "\0" \
	"bootfdt=if test ${boot_fdt} = try; then bootz; else echo WARN: Cannot load the DT; fi" "\0" \
	"clear_env=sf probe 0 && sf erase " __stringify(CONFIG_ENV_OFFSET) " 10000" "\0" \
	"console=ttymxc0" "\0" \
	"fdt_addr=0x83000000" "\0" \
	"fdt_file=imx7d-samx7.dtb" "\0" \
	"fdt_high=0xffffffff" "\0" \
	"image=zImage" "\0" \
	"initrd_high=0xffffffff" "\0" \
	"loadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};" "\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} root=${mmcroot}" "\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw" "\0" \
	"nfsroot=/srv/export/tschaefer/samx7" "\0" \
	"bootscript=echo Running bootscript from mmc ...; source" "\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}" "\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}" "\0" \
	"script=boot.scr\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"run bootfdt " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} root=/dev/nfs ip=dhcp " \
		"nfsroot=${serverip}:${nfsroot},v3,tcp mipi_dsi_samsung.lvds_freq=50" "\0" \
	"netboot=echo Booting from net ...; " \
		"bootp && run netargs && tftp ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if tftp ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"run bootfdt; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;" "\0" \
	"qspi_header_file=qspi-header.bin" "\0" \
	"uboot_update_file=u-boot-smx7-spl.imx" "\0" \
	"uboot_install=bootp && tftp 80800000 ${qspi_header_file} && tftp 88000000 ${uboot_update_file} && " \
		"sf probe 0 && sf erase 0 80000 && sf write 80800000 0 200 && sf write 88000000 400 ${filesize}" "\0" \
	"uboot_update=bootp && tftp 88000000 ${uboot_update_file} && " \
		"sf probe 0 && sf read 80800000 0 200 && sf erase 0 80000 && " \
		"sf write 80800000 0 200 && sf write 88000000 400 ${filesize}" "\0"

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev}; " \
	"if mmc rescan; then " \
		"if run loadbootscript; then run bootscript; " \
		"else " \
			"if run loadimage; then run mmcboot; else run netboot; fi; " \
		"fi; " \
	"else run netboot; fi"

#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x20000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* MXC SPI driver support */
/* #define CONFIG_MXC_SPI */

#ifdef CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS       2
#define CONFIG_SF_DEFAULT_CS        0
#define CONFIG_SF_DEFAULT_SPEED     20000000
#define CONFIG_SF_DEFAULT_MODE      SPI_MODE_0
#endif

#define CONFIG_SYS_FSL_USDHC_NUM    2

#define CONFIG_SYS_MMC_ENV_DEV      0                   /* USDHC1 */
#define CONFIG_SYS_MMC_ENV_PART     0                   /* user area */
#define CONFIG_MMCROOT              "/dev/mmcblk0p2"    /* USDHC1 */

#define CONFIG_IMX_THERMAL

#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#endif

#ifdef CONFIG_FSL_QSPI
#define CONFIG_SPI_FLASH_BAR
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		40000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define FSL_QSPI_FLASH_NUM		2
#define FSL_QSPI_FLASH_SIZE		SZ_16M
#define QSPI0_BASE_ADDR			QSPI1_IPS_BASE_ADDR
#define QSPI0_AMBA_BASE			QSPI0_ARB_BASE_ADDR
#endif

#endif	/* __CONFIG_H */

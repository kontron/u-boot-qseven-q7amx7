/*
 * Copyright (C) 2017 Kontron Europe GmbH
 * Author: Thomas Schaefer <thomas.schaefer@kontron.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */
 
#ifndef __IMX7_SPL_CONFIG_H
#define __IMX7_SPL_CONFIG_H

#ifdef CONFIG_SPL

#define CONFIG_SPL_FRAMEWORK

/*
 * see Figure 6-22 in IMX7D/IMX7S Reference manuals:
 *  - IMX7D/IMX7S OCRAM (IRAM) is from 0x00910000 to 0x00946BFF
 *  - BOOT ROM stack is at 0x00946C00
 *  - Additionally the BOOT ROM loads what they consider the firmware image
 *    which consists of a 4K header in front of us that contains the IVT, DCD
 *    and some padding thus 'our' max size is really 0x00908000 - 0x00918000
 *    or 64KB
 */
#define CONFIG_SYS_THUMB_BUILD
#define CONFIG_SPL_LDSCRIPT	"arch/arm/mach-omap2/u-boot-spl.lds"
#define CONFIG_SPL_TEXT_BASE		0x00911000
#define CONFIG_SPL_MAX_SIZE		0x10000
#define CONFIG_SPL_STACK		0x00946C00
/*
 * Pad SPL to 68KB - 1KB (4KB header + 64KB max size - 1KB SPI offset).
 * This allows to write the SPL/U-Boot combination generated with
 * u-boot-with-spl.imx directly to a boot media (given that boot media
 * specific offset is configured properly).
 */
#define CONFIG_SPL_PAD_TO		0x10C00

/* NAND support */
#if defined(CONFIG_SPL_NAND_SUPPORT)
#define CONFIG_SPL_NAND_MXS
#endif

/* MMC support */
#if defined(CONFIG_SPL_MMC_SUPPORT)
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1
#define CONFIG_SYS_MONITOR_LEN			409600	/* 400 KB */
#define CONFIG_SPL_ABORT_ON_RAW_IMAGE
#endif

/* SATA support */
#if defined(CONFIG_SPL_SATA_SUPPORT)
#define CONFIG_SPL_SATA_BOOT_DEVICE		0
#define CONFIG_SYS_SATA_FAT_BOOT_PARTITION	1
#define CONFIG_SPL_ABORT_ON_RAW_IMAGE
#endif

/* Define the payload for FAT/EXT support */
#if defined(CONFIG_SPL_FAT_SUPPORT) || defined(CONFIG_SPL_EXT_SUPPORT)
#define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME  "u-boot.img"
#endif

#define CONFIG_SPL_BSS_START_ADDR      0x88200000
#define CONFIG_SPL_BSS_MAX_SIZE        0x100000		/* 1 MB */
#define CONFIG_SYS_SPL_MALLOC_START    0x88300000
#define CONFIG_SYS_SPL_MALLOC_SIZE     0x100000		/* 1 MB */
#define CONFIG_SYS_TEXT_BASE           0x87800000
#endif

#endif

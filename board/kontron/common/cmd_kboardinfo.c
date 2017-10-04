/*
 * Copyright 2013 Kontron Europe GmbH
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include "kboardinfo.h"
#include "../common/emb_vpd.h"


char *print_if_avail (char *text)
{
	int idx = 0;
	if (text != NULL){
		while (text[idx] == 0x20)
			idx++;

		return text+idx;
	}
	else
		return "na";
}

int do_kboardinfo (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv [])
{
	u32 cpurev;

#if defined(CONFIG_EMB_EEP_SPI)
	emb_vpd_init_r();
#endif

#ifdef CONFIG_KBOARDINFO_MODULE
	printf ("Module:\n");
	printf ("Manufacturer:        %s\n", print_if_avail (getManufacturer(1)));
	printf ("Product name:        %s\n", print_if_avail (getProductName(1)));
	printf ("Material number:     %s\n", print_if_avail (getSapId(1)));
	printf ("Serial number:       %s\n", getSerNo ());
	printf ("Manufacturer Date:   %s\n", print_if_avail (getManufacturerDate(1)));
	printf ("Revision:            %s\n", print_if_avail (getRevision(1)));
#endif
#ifdef CONFIG_KBOARDINFO_CARRIER
	printf ("\nCarrier:\n");
	printf ("Manufacturer:        %s\n", print_if_avail (getManufacturer(2)));
	printf ("Product name:        %s\n", print_if_avail (getProductName(2)));
	printf ("Material number:     %s\n", print_if_avail (getSapId(2)));
	printf ("Serial number:       %s\n", getSerNoEeprom (2));
	printf ("Manufacturer Date:   %s\n", print_if_avail (getManufacturerDate(2)));
	printf ("Revision:            %s\n", print_if_avail (getRevision(2)));
#endif
#ifdef CONFIG_KBOARDINFO_LOGIC
	printf ("\n");
	printf ("Logic rev.:          0x%x\n", getLogicRev());
	printf ("Boot flash:          %s Flash\n",
		getBootFlash () == 0 ? "Standard" : getBootFlash () == 1 ? "Rescue" : getBootFlash () ? "External" : "Unknown");
#endif
	/*
	 * Needed for test design department
	 */
	cpurev = get_cpu_rev();
	printf("CPU:                 Freescale i.MX%s rev%d.%d at %d MHz\n",
		get_imx_type((cpurev & 0xFF000) >> 12),
		(cpurev & 0x000F0) >> 4,
		(cpurev & 0x0000F) >> 0,
		mxc_get_clock(MXC_ARM_CLK) / 1000000);

	/*
	 * U-Boot for Solo does not have an SAP Article
	 * So we leave this information for all U-Boot images
	 * for AMX6
	 */
#if 0
	printf ("U-Boot article name: %s\n", CONFIG_SAP_NAME);
	printf ("U-Boot material num: %s\n", CONFIG_SAP_NUM);
#endif
#ifdef CONFIG_KBOARDINFO_DEBUG
	if ((argc == 2) && (!strcmp(argv[1], "debug_infos"))) {
#ifdef CONFIG_KBOARDINFO_MODULE
		printf ("Module:\r\n");
		printf ("DMI 2: String 1: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 2, 1)));
		printf ("DMI 2: String 2: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 2, 2)));
		printf ("DMI 2: String 3: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 2, 3)));
		printf ("DMI 2: String 4: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 2, 4)));
		printf ("DMI 2: String 5: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 2, 5)));

		printf ("DMI 160: String 1: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 160, 1)));
		printf ("DMI 160: String 2: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 160, 2)));
		printf ("DMI 160: String 3: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 160, 3)));
		printf ("DMI 160: String 4: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 160, 4)));
		printf ("DMI 160: String 5: %s\n", print_if_avail(emb_eep_find_string_in_dmi (1, 160, 5)));
#endif
#ifdef CONFIG_KBOARDINFO_CARRIER
		printf ("Carrier:\r\n");
		printf ("DMI 2: String 1: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 2, 1)));
		printf ("DMI 2: String 2: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 2, 2)));
		printf ("DMI 2: String 3: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 2, 3)));
		printf ("DMI 2: String 4: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 2, 4)));
		printf ("DMI 2: String 5: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 2, 5)));

		printf ("DMI 160: String 1: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 160, 1)));
		printf ("DMI 160: String 2: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 160, 2)));
		printf ("DMI 160: String 3: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 160, 3)));
		printf ("DMI 160: String 4: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 160, 4)));
		printf ("DMI 160: String 5: %s\r\n", print_if_avail(emb_eep_find_string_in_dmi (2, 160, 5)));
#endif
	}
#endif
	return 0;
}


U_BOOT_CMD (
	kboardinfo,	3,	0,	do_kboardinfo,
	"show board data",
	"kboardinfo - show board data like id's"
);

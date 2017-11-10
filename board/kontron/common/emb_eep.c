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
#include <i2c.h>
#include "emb_eep.h"
#include "crc.h"
#define CONFIG_EMB_EEP_I2C_EEPROM

#ifdef CONFIG_EMB_EEP_I2C_EEPROM


int EMB_EEP_I2C_EEPROM_BUS_NUM_1;
int EMB_EEP_I2C_EEPROM_BUS_NUM_2;

static int emb_eep_init (emb_eep_info *vpdi);
static char *emb_eep_find_mac_in_dmi_164 (emb_eep_info *vpdi, int string_num);

static int i2c_read_emb (emb_eep_info *vpdi, int offset, unsigned char *buffer, int len)
{
	switch (vpdi->eeprom_num) {
#ifdef CONFIG_EMB_EEP_I2C_EEPROM_ADDR_1
		case 1:
			i2c_read (CONFIG_EMB_EEP_I2C_EEPROM_ADDR_1,
				CONFIG_EMB_EEP_I2C_EEPROM_OFFSET_1 + offset,
				CONFIG_EMB_EEP_I2C_EEPROM_ADDR_LEN_1,
				(unsigned char *) &buffer[0],
				len);
			break;
#endif
#ifdef CONFIG_EMB_EEP_I2C_EEPROM_ADDR_2
		case 2:
			i2c_read (CONFIG_EMB_EEP_I2C_EEPROM_ADDR_2,
				CONFIG_EMB_EEP_I2C_EEPROM_OFFSET_2 + offset,
				CONFIG_EMB_EEP_I2C_EEPROM_ADDR_LEN_2,
				(unsigned char *) &buffer[0],
				len);
			break;
#endif
		default:
			printf ("Warning: EEPROM number %d not supported\n", vpdi->eeprom_num);
	}
	return 0;
}

static int emb_eep_get_block_size (emb_eep_info *vpdi, int idx)
{
	int block_size;
	uchar buffer[2];

	i2c_read_emb (vpdi, idx+1, (unsigned char *) &buffer[0],2);

	block_size = buffer[0] * 16 + buffer[1];

	block_size *= 2; /* block size in 2 bytes words */

	if (idx + block_size <= vpdi->max_size)
		return block_size;
	else
		return 0;
}

/*
 * We only search blocks before the CRC block.
 */
static int emb_eep_find_block(emb_eep_info *vpdi, int idx, int block_id)
{
	int block_size;
	uchar buffer[2];
	int block_len;

	if (idx == 0) {

		/* Calculate offset to first block */
		idx = 2 * (*(vpdi->header + 4));

		memset(buffer, 0, sizeof(buffer));
		i2c_read_emb (vpdi, idx, (unsigned char *) &buffer[0],1);

		if (buffer[0] == block_id) {
			block_len = emb_eep_get_block_size(vpdi, idx);
			i2c_read_emb (vpdi, idx, (unsigned char *) (unsigned char *) vpdi->block, block_len);

			return idx;
		}

		if (buffer[0] == BLOCK_ID_CRC)
			return -1;
	}

	do {
		block_size = emb_eep_get_block_size(vpdi, idx);

		if (block_size <= 0)
			return -1;

		idx += block_size;

		i2c_read_emb (vpdi, idx,(unsigned char *) &buffer[0],1);

		if (buffer[0] == block_id)
			break;

		if (buffer[0] == BLOCK_ID_CRC)
			return -1;

	} while (idx < vpdi->max_size);

	block_size = emb_eep_get_block_size(vpdi, idx);
	if ((idx + block_size) < vpdi->max_size) {
		i2c_read_emb (vpdi, idx, (unsigned char *) vpdi->block,	block_size);

		return idx;
	}
	else
		return -1;
}

char * emb_eep_find_mac_in_dmi (int eeprom_num, int eth_num)
{
	char vpd_header[0x10];
	char vpd_block[CONFIG_EMB_EEP_I2C_EEPROM_SIZE];
	emb_eep_info vpdi;

	vpdi.eeprom_num = eeprom_num;
	vpdi.block = &vpd_block[0];
	vpdi.header = &vpd_header[0];

	if (emb_eep_init (&vpdi) != 0) {
		return NULL;
	}

	return emb_eep_find_mac_in_dmi_164(&vpdi, eth_num);
}

char * emb_eep_find_string_in_dmi (int eeprom_num, int dmi_num, int string_num)
{
	char vpd_header[0x10];
	char vpd_block[CONFIG_EMB_EEP_I2C_EEPROM_SIZE];
	emb_eep_info vpdi;
	int idx = 0, num, tmp_num;
	char *ptr, * block_end;
	int offset_nums, offset_strings, max_strings;
	int found = 0;

	vpdi.eeprom_num = eeprom_num;
	vpdi.block = &vpd_block[0];
	vpdi.header = &vpd_header[0];

	if (emb_eep_init (&vpdi) != 0) {
		return NULL;
	}

	switch (dmi_num) {
		case 2:
			offset_nums = 6;
			offset_strings = 18;
			max_strings = 5;
			break;
		case 160:
			offset_nums = 11;
			offset_strings = 15;
			max_strings = 5;
			break;
		default:
			return NULL;
	}

	if ((string_num < 1) || (string_num > max_strings))
		return NULL;

	/*
	 * find SMBIOS Block with type dmi_num
	 */
	do {
		idx = emb_eep_find_block (&vpdi, idx, BLOCK_ID_SMBIOS);
		debug("SMBIOS block found with index %d\n", idx);

		if (idx < 0)
			return NULL;

		if ((*(vpdi.block + 3)) == dmi_num) {
			found = 1;
			break;
		}

	} while (idx < vpdi.max_size);

	if (found == 0)
		return NULL;
	/*
	 * find string index of string_num
	 */

	num = *(vpdi.block + offset_nums + string_num);
	if ((num == 0) | (num > max_strings))
		return NULL;

	/*
	 * get offset to string
	 */
	ptr = vpdi.block + offset_strings;
	tmp_num = 1;
	while (tmp_num < num) {
		ptr += strlen (ptr) + 1;
		tmp_num ++;
	}

	/*
	 * check for plausibility
	 */
	block_end = vpdi.block + emb_eep_get_block_size(&vpdi, idx);

	if (ptr < block_end)
		return ptr;
	else
		return NULL;

	return NULL;
}


static char *emb_eep_find_mac_in_dmi_164 (emb_eep_info *vpdi, int string_num)
{
	int idx = 0, tmp_num, num;
	char *ptr, * block_end;
	int numOfMacs;
	/* find string num */
	if (string_num < 1)
		return NULL;

	/*
	 * find SMBIOS Block with type 164
	 */
	do {
		idx = emb_eep_find_block (vpdi, idx, BLOCK_ID_SMBIOS);

		if (idx < 0)
			return NULL;

		else if ((*(vpdi->block + 3)) == 164)
			break;

	} while (idx < vpdi->max_size);

	numOfMacs = *(vpdi->block + 12);
	if (string_num > numOfMacs)
		return NULL;

	/*
	 * get offset to string
	 */
	num = *(vpdi->block + 12 + string_num);

	ptr = vpdi->block + 12 + string_num + numOfMacs;
	tmp_num = 1;
	while (tmp_num < num) {
		ptr += strlen (ptr) + 1;
		tmp_num ++;
	}

	/*
	 * check for plausibility
	 */
	block_end = vpdi->block + emb_eep_get_block_size(vpdi, idx);

	if (ptr < block_end)
		return ptr;
	else
		return NULL;

	return ptr;
}


static void emb_eep_default_ethaddr (void)
{
	char *e_ethaddr;

	e_ethaddr = getenv("ethaddr");
	if (e_ethaddr == NULL) {
		printf ("WARNING: ethaddr not found in environment, using default value\n");
		setenv  ("ethaddr", D_ETHADDR);
	}
}

static void emb_eep_import_ethaddr (emb_eep_info *vpdi, const char *eth_x_addr, int num, const char *d_ethaddr)
{
	char *e_ethaddr;
	char *v_ethaddr;

	v_ethaddr = emb_eep_find_mac_in_dmi_164(vpdi, num);

	if ((v_ethaddr[2] != ':') || (v_ethaddr[5] != ':') ||
		(v_ethaddr[8] != ':') || (v_ethaddr[11] != ':')  ||
		(v_ethaddr[14] != ':') ) {
		printf ("Error in MAC Address, no import possible\n");
		return;
	}
	
	e_ethaddr = getenv((char*)eth_x_addr);

	if (v_ethaddr == NULL) {
		printf ("WARNING: %s not found in embedded EEPROM\n", eth_x_addr);
		return;
	}

	/* Check if ethaddr already exists in Environment */
	if (e_ethaddr) {
		if (!strcmp(d_ethaddr, e_ethaddr)) {
			printf ("Embedded EEPROM: Overwrite default %s to %s\n", eth_x_addr, v_ethaddr);
			setenv((char*)eth_x_addr, v_ethaddr);
			return;
		}
		else {
			if (strcmp (e_ethaddr, v_ethaddr)) {
				printf ("Embedded EEPROM: Not overwriting existing %s\n", eth_x_addr);
			}
			return;
		}
	}

	debug ("Setenv %s\n", eth_x_addr);
	setenv((char*)eth_x_addr, v_ethaddr);
}


static int emb_eep_check_header (emb_eep_info *vpdi)
{
	if (*(vpdi->header + 1) != '3'){
		printf ("emb_eep_check_header: 0x%x instead of 3\n",*(vpdi->header + 1));
		return 0;
	}
	if (*(vpdi->header + 2) != 'P'){
		printf ("emb_eep_check_header: 0x%x instead of P\n",*(vpdi->header + 2));
		return 0;
	}
#if 0	
	if (*(vpdi->header + 3) != 0x10)
		return 0;
#endif

	return 1;
}

static int emb_eep_init (emb_eep_info *vpdi)
{

	debug ("Initialize vpd_init, vpdi = 0x%x\n", (int)vpdi);

	/* read 6 bytes first */
	if (vpdi->eeprom_num == 2)
		i2c_set_bus_num (EMB_EEP_I2C_EEPROM_BUS_NUM_2);
	else
		i2c_set_bus_num (EMB_EEP_I2C_EEPROM_BUS_NUM_1);

	i2c_read_emb (vpdi, 0, (unsigned char *)&vpdi->header[0], 6);

	if (!emb_eep_check_header (vpdi)) {
		printf ("WARNING: Embedded EEPROM header not valid, abort\n");
		return -1;
	}

	vpdi->max_size = 256 << (vpdi->header[5] & 0x7);
	if ( vpdi->max_size > CONFIG_EMB_EEP_I2C_EEPROM_SIZE) {
		printf ("Embedded EEPROM contents too large\n");
		return -1;
	}
		
	return 0;
}

/*
 * initialize environment from embedded EEPROM
 */
void emb_eep_init_r(int eeprom_num_serial, int eeprom_num_eth, int num_of_macs)
{
	char vpd_header[0x10];
	char vpd_block[CONFIG_EMB_EEP_I2C_EEPROM_SIZE];
	emb_eep_info vpdi;
	char *val;

	vpdi.block = &vpd_block[0];
	vpdi.header = &vpd_header[0];
	vpdi.eeprom_num = eeprom_num_serial;

	if (emb_eep_init (&vpdi) == 0) {
		/*
		* Import serial number to environment
		* Block SMBIOS, type 2, string number 4
		*/
		val = emb_eep_find_string_in_dmi (eeprom_num_serial, 2, 4);
		if (val != NULL)
			setenv("serial#", val);
	}

	vpdi.eeprom_num = eeprom_num_eth;

	if (emb_eep_init (&vpdi) != 0) {
		emb_eep_default_ethaddr ();
		return;
	}

	/*
	 * Import eth addresses to environment
	 */
	emb_eep_import_ethaddr (&vpdi, "ethaddr", 1, D_ETHADDR);

#if defined(CONFIG_HAS_ETH1)
	if (num_of_macs >= 2)
		emb_eep_import_ethaddr (&vpdi, "eth1addr", 2, D_ETH1ADDR);
#endif

#if defined(CONFIG_HAS_ETH2)
	if (num_of_macs >= 3)
		emb_eep_import_ethaddr (&vpdi, "eth2addr", 3, D_ETH1ADDR);
#endif

#if defined(CONFIG_HAS_ETH3)
	if (num_of_macs >= 4)
		emb_eep_import_ethaddr (&vpdi, "eth3addr", 4, D_ETH1ADDR);
#endif

#if defined(CONFIG_HAS_ETH4)
	if (num_of_macs >= 5)
		emb_eep_import_ethaddr (&vpdi, "eth4addr", 5, D_ETH1ADDR);
#endif

	return;
}

#endif /* CONFIG_EMB_EEP_I2C_EEPROM */

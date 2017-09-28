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
#ifndef EMB_EEP_H
#define EMB_EEP_H

typedef struct {
	char		* header;
	char		* block;
	unsigned int 	max_size;
	int		eeprom_num;
} emb_eep_info;

#define BLOCK_ID_CRC	(0xF2)
#define BLOCK_ID_SMBIOS	(0xD0)

extern void emb_eep_init_r(int eeprom_num_serial, int eeprom_num_eth);
extern char * emb_eep_find_string_in_dmi (int eeprom_num, int dmi_num, int string_num);

#endif /* EMB_EEP_H */

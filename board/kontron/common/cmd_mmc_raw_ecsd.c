/*
 * Copyright 2013 Kontron Modular Computers GmbH
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <mmc.h>
#include <memalign.h>

int mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd, struct mmc_data *data);
int mmc_send_status(struct mmc *mmc, int timeout);
int mmc_switch(struct mmc *mmc, u8 set, u8 index, u8 value);

int globvar_mmc_trace_enabled = 0;
static int mmc_raw_ecsd_write_activate = 1;

#define MMC_MODE_HC     (1 << 5)

int mmc_go_idle(struct mmc* mmc)
{
	struct mmc_cmd cmd;
	int err;

	udelay(1000);

	cmd.cmdidx = MMC_CMD_GO_IDLE_STATE;
	cmd.cmdarg = 0;
	cmd.resp_type = MMC_RSP_NONE;

	err = mmc_send_cmd(mmc, &cmd, NULL);

	if (err)
		return err;

	udelay(2000);

	return 0;
}

int mmc_send_op_cond(struct mmc *mmc)
{
	int timeout = 10000;
	struct mmc_cmd cmd;
	int err;

	/* Some cards seem to need this */
	mmc_go_idle(mmc);

 	/* Asking to the card its capabilities */
 	cmd.cmdidx = MMC_CMD_SEND_OP_COND;
 	cmd.resp_type = MMC_RSP_R3;
 	cmd.cmdarg = 0;

 	err = mmc_send_cmd(mmc, &cmd, NULL);

 	if (err)
 		return err;

 	udelay(1000);

	do {
		cmd.cmdidx = MMC_CMD_SEND_OP_COND;
		cmd.resp_type = MMC_RSP_R3;
		cmd.cmdarg = (mmc_host_is_spi(mmc) ? 0 :
				(mmc->cfg->voltages &
				(cmd.response[0] & OCR_VOLTAGE_MASK)) |
				(cmd.response[0] & OCR_ACCESS_MODE));

		if (mmc->cfg->host_caps & MMC_MODE_HC)
			cmd.cmdarg |= OCR_HCS;

		err = mmc_send_cmd(mmc, &cmd, NULL);

		if (err)
			return err;

		udelay(1000);
	} while (!(cmd.response[0] & OCR_BUSY) && timeout--);

	if (timeout <= 0)
		return -EOPNOTSUPP;

	if (mmc_host_is_spi(mmc)) { /* read OCR for spi */
		cmd.cmdidx = MMC_CMD_SPI_READ_OCR;
		cmd.resp_type = MMC_RSP_R3;
		cmd.cmdarg = 0;

		err = mmc_send_cmd(mmc, &cmd, NULL);

		if (err)
			return err;
	}

	mmc->version = MMC_VERSION_UNKNOWN;
	mmc->ocr = cmd.response[0];

	mmc->high_capacity = ((mmc->ocr & OCR_HCS) == OCR_HCS);
	mmc->rca = 0;

	return 0;
}

int mmc_send_ext_csd(struct mmc *mmc, u8 *ext_csd)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	int err;

	/* Get the Card Status Register */
	cmd.cmdidx = MMC_CMD_SEND_EXT_CSD;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = 0;

	data.dest = (char *)ext_csd;
	data.blocks = 1;
	data.blocksize = 512;
	data.flags = MMC_DATA_READ;

	err = mmc_send_cmd(mmc, &cmd, &data);

	return err;
}

void mmc_set_ios(struct mmc *mmc)
{
	mmc->cfg->ops->set_ios(mmc);
}

void mmc_set_bus_width(struct mmc *mmc, uint width)
{
	mmc->bus_width = width;

	mmc_set_ios(mmc);
}

int do_mmc_raw_ecsd_init (struct mmc *mmc)
{
	int err;
	struct mmc_cmd cmd;

	if (mmc_getcd(mmc) == 0) {
		mmc->has_init = 0;
		printf("MMC: no card present\n");
		return -ENOMEDIUM;
	}

	err = mmc_init(mmc);

	if (err) {
		printf ("mmc->init returned error %d\n", err);
		return err;
	}

	mmc_set_bus_width(mmc, 1);

	mmc_set_clock(mmc, 1);

	/* Reset the Card */
	err = mmc_go_idle(mmc);
	if (err) {
		printf ("mmc_go_idle failed with err %d\n", err);
		return err;
	}

	/* The internal partition reset to user partition(0) at every CMD0 */
	mmc->block_dev.hwpart = 0;

	err = mmc_send_op_cond(mmc);

	if (err) {
		printf("Card did not respond to voltage select!\n");
		return -EOPNOTSUPP;
	}

	/* Put the Card in Identify Mode */
	cmd.cmdidx = mmc_host_is_spi(mmc) ? MMC_CMD_SEND_CID :
		MMC_CMD_ALL_SEND_CID; /* cmd not supported in spi */
	cmd.resp_type = MMC_RSP_R2;
	cmd.cmdarg = 0;

	err = mmc_send_cmd(mmc, &cmd, NULL);

	if (err) {
		printf ("Error after Identify Mode\n");
		return err;
	}

	memcpy(mmc->cid, cmd.response, 16);

	/*
	 * For MMC cards, set the Relative Address.
	 * For SD cards, get the Relatvie Address.
	 * This also puts the cards into Standby State
	 */
	if (!mmc_host_is_spi(mmc)) { /* cmd not supported in spi */
		cmd.cmdidx = SD_CMD_SEND_RELATIVE_ADDR;
		cmd.cmdarg = mmc->rca << 16;
		cmd.resp_type = MMC_RSP_R6;

		err = mmc_send_cmd(mmc, &cmd, NULL);

		if (err) {
			printf ("Error after SD_CMD_SEND_RELATIVE_ADDR\n");
			return err;
		}

		if (IS_SD(mmc))
			mmc->rca = (cmd.response[0] >> 16) & 0xffff;
	}

	if (err) {
		mmc->has_init = 0;
	}
	else
		mmc->has_init = 1;


	return err;
}


int do_mmc_raw_ecsd_read (struct mmc *mmc, int regnum)
{
	ALLOC_CACHE_ALIGN_BUFFER(u8, ext_csd, 512);
	struct mmc_cmd cmd;
	int idx1, idx2;
	int err;
	char env_var_tmp [10];

	for (idx1 = 0; idx1 < 512; idx1 ++)
		ext_csd[idx1] = 0;


	/*
	 * Select the card, and put it into Transfer Mode
	 * depending on previous state, this might not be necessary
	 * and return error, error can be ignored in this case...
	 */
	if (!mmc_host_is_spi(mmc)) { /* cmd not supported in spi */
		cmd.cmdidx = MMC_CMD_SELECT_CARD;
		cmd.resp_type = MMC_RSP_R1;
		cmd.cmdarg = mmc->rca << 16;

		err = mmc_send_cmd(mmc, &cmd, NULL);
	}


	err = mmc_send_ext_csd(mmc, ext_csd);

	if (err) {
		printf ("ecsd read returned err = %d\n", err);
		return (-1);
	} else {
		if ((regnum == -1) || (regnum > 512)) {
			for (idx1 = 0; idx1 < 512; idx1 += 16) {
				printf ("\n%3x   ", idx1);
				for (idx2 = 0; idx2 < 16; idx2 ++) {
					if (!(idx2%4)) printf (" ");
					printf("%2x ",ext_csd [idx1+idx2]);
				}
			}
			printf ("\n");
			return (0);
		} else {
			printf ("0x%x\n", ext_csd [regnum]);
			sprintf (env_var_tmp, "%x",ext_csd[regnum]);
			setenv  ("raw_ecsd_tmp", env_var_tmp);
			return (ext_csd [regnum]);
		}
	}
}


int do_mmc_raw_ecsd_ops(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	static struct mmc *mmc = 0;
	int dev;
	int err;

#if 0
	int i;
	printf("argc: %d\n", argc);
	for (i=0 ; i<argc; i++)
		printf("argv[%d]: %s\n", i, argv[i]);
#endif

	if (strcmp(argv[1], "init") == 0) {

		if (argc != 3) {
			printf ("Invalid syntax\n");
			return 0;
		}
			
		dev = simple_strtoul(argv[2], NULL, 10);

		mmc = find_mmc_device(dev);
		if (!mmc) {
			printf("no mmc device at slot %x\n", dev);
			return 1;
		}
		/* force init */
		mmc->has_init = 0;

		err = do_mmc_raw_ecsd_init (mmc);

		return err;

		
	} else if (strcmp(argv[1], "read") == 0) {

		uint regnum;

		if (argc < 2) {
			printf ("Invalid syntax\n");
			return -1;
		}
		if (!mmc) {
			printf ("No mmc device selected, call mmc raw_ecsd init <devnum> first\n");
			return 0;
		}

		if (argc > 2){
			regnum = (ushort)simple_strtoul(argv[2], NULL, 16);
		} else {
			regnum = -1;
		}

		do_mmc_raw_ecsd_read(mmc, regnum);

		return 0;

	} else if (strcmp(argv[1], "cmp") == 0) {

		uint regnum;
		ushort cmpval;
		int val;

		if (argc != 4) {
			printf ("Invalid syntax\n");
			return -1;
		}
		if (!mmc) {
			printf ("No mmc device selected, call mmc raw_ecsd init <devnum> first\n");
			return 0;
		}

		regnum = (ushort)simple_strtoul(argv[2], NULL, 16);
		cmpval = (ushort)simple_strtoul(argv[3], NULL, 16);

		val = do_mmc_raw_ecsd_read(mmc, regnum);

		if (val == (int) cmpval) {
			printf ("ECSD 0x%x: Correct value 0x%x found\n", regnum, val);
			return 0;
		} else {
			printf ("ECSD 0x%x: Found value 0x%x, but expected 0x%x\n", regnum, val, cmpval);
			return 1;
		}


	} else if (strcmp(argv[1], "write") == 0) {

		uint regnum, value;
		
		if (argc != 4) {
			printf ("Invalid syntax\n");
			return -1;
		}

		if (!mmc) {
			printf ("No mmc device selected, call mmc raw_ecsd init <devnum> first\n");
			return 0;
		}

		regnum = (ushort)simple_strtoul(argv[2], NULL, 16);
		value = (ushort)simple_strtoul(argv[3], NULL, 16);

		if (mmc_raw_ecsd_write_activate == 0) {
			printf ("Write not active, would write addr 0x%x, val 0x%x\n", regnum, value);
			err = 0;
		} else {
			printf ("Write active: addr 0x%x, val 0x%x\n", regnum, value);
			err = mmc_switch(mmc, 1, regnum, value);
		}
		if (err)
			printf ("mmc_switch returned error err = %d\n", err);
		
		return err;
	} else if (strcmp(argv[1], "trace") == 0) {

		if (argc > 2) {
			globvar_mmc_trace_enabled = (int)simple_strtoul(argv[2], NULL, 10);
			printf ("MMC Trace set to %d\n", globvar_mmc_trace_enabled);
		}
		return 0;
	} else if (strcmp(argv[1], "activate") == 0) {

		if (argc > 2) {
			mmc_raw_ecsd_write_activate = (int)simple_strtoul(argv[2], NULL, 10);
			printf ("MMC ECSD Register write activate set to %d\n", mmc_raw_ecsd_write_activate);
		}
		return 0;

	} else {
		printf ("Syntax:\n");
		printf ("mmc raw_ecsd init <device_num>\n");
		printf ("mmc raw_ecsd activate <0|1>\n");
		printf ("mmc raw_ecsd read [<addr>]\n");
		printf ("mmc raw_ecsd cmp <addr> <cmpval>\n");
		printf ("mmc raw_ecsd write <addr> <val>\n");
		return 0;
	}
}

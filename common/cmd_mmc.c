/*
 * (C) Copyright 2003
 * Kyle Harris, kharris@nexus-tech.net
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

#ifndef CONFIG_GENERIC_MMC
static int curr_device = -1;

int do_mmc (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int dev;

	if (argc < 2) {
		cmd_usage(cmdtp);
		return 1;
	}

	if (strcmp(argv[1], "init") == 0) {
		if (argc == 2) {
			if (curr_device < 0)
				dev = 1;
			else
				dev = curr_device;
		} else if (argc == 3) {
			dev = (int)simple_strtoul(argv[2], NULL, 10);
		} else {
			cmd_usage(cmdtp);
			return 1;
		}

		if (mmc_legacy_init(dev) != 0) {
			puts("No MMC card found\n");
			return 1;
		}

		curr_device = dev;
		printf("mmc%d is available\n", curr_device);
	} else if (strcmp(argv[1], "device") == 0) {
		if (argc == 2) {
			if (curr_device < 0) {
				puts("No MMC device available\n");
				return 1;
			}
		} else if (argc == 3) {
			dev = (int)simple_strtoul(argv[2], NULL, 10);

#ifdef CONFIG_SYS_MMC_SET_DEV
			if (mmc_set_dev(dev) != 0)
				return 1;
#endif
			curr_device = dev;
		} else {
			cmd_usage(cmdtp);
			return 1;
		}

		printf("mmc%d is current device\n", curr_device);
	} else {
		cmd_usage(cmdtp);
		return 1;
	}

	return 0;
}

U_BOOT_CMD(
	mmc, 3, 1, do_mmc,
	"MMC sub-system",
	"init [dev] - init MMC sub system\n"
	"mmc device [dev] - show or set current device"
);
#else /* !CONFIG_GENERIC_MMC */

static void print_mmcinfo(struct mmc *mmc)
{
	printf("Device: %s\n", mmc->name);
	printf("Manufacturer ID: %x\n", mmc->cid[0] >> 24);
	printf("OEM: %x\n", (mmc->cid[0] >> 8) & 0xffff);
	printf("Name: %c%c%c%c%c \n", mmc->cid[0] & 0xff,
			(mmc->cid[1] >> 24), (mmc->cid[1] >> 16) & 0xff,
			(mmc->cid[1] >> 8) & 0xff, mmc->cid[1] & 0xff);

	printf("Tran Speed: %d\n", mmc->tran_speed);
	printf("Rd Block Len: %d\n", mmc->read_bl_len);

	printf("%s version %d.%d\n", IS_SD(mmc) ? "SD" : "MMC",
			(mmc->version >> 4) & 0xf, mmc->version & 0xf);

	printf("High Capacity: %s\n", mmc->high_capacity ? "Yes" : "No");
	printf("Capacity: %lld\n", mmc->capacity);

	printf("Bus Width: %d-bit\n", mmc->bus_width);
}

static void print_csd(struct mmc *mmc)
{
	printf("mmc->csd[0] = 0x%08X\n", mmc->csd[0]);
	printf("mmc->csd[1] = 0x%08X\n", mmc->csd[1]);
	printf("mmc->csd[2] = 0x%08X\n", mmc->csd[2]);
	printf("mmc->csd[3] = 0x%08X\n", mmc->csd[3]);

	printf("csd->csd_structure = 0x%02X\n", CSD_STRUCTURE(mmc->csd));
	printf("csd->spec_vers = 0x%02X\n", CSD_SPEC_VERS(mmc->csd));
	printf("csd->taac = 0x%02X\n", CSD_TAAC(mmc->csd));
	printf("csd->nsac = 0x%02X\n", CSD_NSAC(mmc->csd));
	printf("csd->tran_speed = 0x%02X\n", CSD_TRAN_SPEED(mmc->csd));
	printf("csd->ccc = 0x%04X\n", CSD_CCC(mmc->csd));
	printf("csd->read_bl_len = 0x%02X\n", CSD_READ_BL_LEN(mmc->csd));
	printf("csd->read_bl_partial = 0x%02X\n",
		CSD_READ_BL_PARTIAL(mmc->csd));
	printf("csd->write_blk_misalign = 0x%02X\n",
		CSD_WRITE_BLK_MISALIGN(mmc->csd));
	printf("csd->read_blk_misalign = 0x%02X\n",
		CSD_READ_BLK_MISALIGN(mmc->csd));
	printf("csd->dsr_imp = 0x%02X\n", CSD_DSR_IMP(mmc->csd));
	printf("csd->c_size = 0x%04X\n", CSD_C_SIZE(mmc->csd));
	printf("csd->vdd_r_curr_min = 0x%04X\n", CSD_VDD_R_CURR_MIN(mmc->csd));
	printf("csd->vdd_r_curr_max = 0x%04X\n", CSD_VDD_R_CURR_MAX(mmc->csd));
	printf("csd->vdd_w_curr_min = 0x%04X\n", CSD_VDD_W_CURR_MIN(mmc->csd));
	printf("csd->vdd_w_curr_max = 0x%04X\n", CSD_VDD_W_CURR_MAX(mmc->csd));
	printf("csd->c_size_mult = 0x%04X\n", CSD_C_SIZE_MULT(mmc->csd));
	printf("csd->erase_grp_size = 0x%04X\n", CSD_ERASE_GRP_SIZE(mmc->csd));
	printf("csd->erase_grp_mult = 0x%04X\n", CSD_ERASE_GRP_MULT(mmc->csd));
	printf("csd->wp_grp_size = 0x%04X\n", CSD_WP_GRP_SIZE(mmc->csd));
	printf("csd->wp_grp_enable = 0x%02X\n", CSD_WP_GRP_ENABLE(mmc->csd));
	printf("csd->default_ecc = 0x%02X\n", CSD_DEFAULT_ECC(mmc->csd));
	printf("csd->r2w_factor = 0x%04X\n", CSD_R2W_FACTOR(mmc->csd));
	printf("csd->write_bl_len = 0x%04X\n", CSD_WRITE_BL_LEN(mmc->csd));
	printf("csd->write_bl_partial = 0x%02X\n",
		CSD_WRITE_BL_PARTIAL(mmc->csd));
	printf("csd->file_format_grp = 0x%02X\n",
		CSD_FILE_FORMAT_GRP(mmc->csd));
	printf("csd->copy = 0x%02X\n", CSD_COPY(mmc->csd));
	printf("csd->perm_write_protect = 0x%02X\n",
		CSD_PERM_WRITE_PROTECT(mmc->csd));
	printf("csd->tmp_write_protect = 0x%02X\n",
		CSD_TMP_WRITE_PROTECT(mmc->csd));
	printf("csd->ecc = 0x%02X\n", CSD_ECC(mmc->csd));
	printf("csd->crc = 0x%02X\n", CSD_CRC(mmc->csd));
	printf("csd->one = 0x%02X\n", CSD_ONE(mmc->csd));
}

static void print_ext_csd(char *ext_csd)
{
	printf("ext_csd->s_cmd_set = 0x%02X\n", ext_csd[504]);
	printf("ext_csd->hpl_features = 0x%02X\n", ext_csd[503]);
	printf("ext_csd->bkops_support = 0x%02X\n", ext_csd[502]);
	printf("ext_csd->bkops_status = 0x%02X\n", ext_csd[246]);
	printf("ext_csd->correctly_prg_sectors_num = 0x%08X\n",
		ext_csd[245] * (1 << 24) | ext_csd[244] * (1 << 16) |
		ext_csd[243] * (1 << 8) | ext_csd[242]);
	printf("ext_csd->ini_timeout_ap = 0x%02X\n", ext_csd[241]);
	printf("ext_csd->pwr_cl_ddr_52_360 = 0x%02X\n", ext_csd[239]);
	printf("ext_csd->pwr_cl_ddr_52_195 = 0x%02X\n", ext_csd[238]);
	printf("ext_csd->min_perf_ddr_w_8_52 = 0x%02X\n", ext_csd[235]);
	printf("ext_csd->min_perf_ddr_r_8_52 = 0x%02X\n", ext_csd[234]);
	printf("ext_csd->trim_mult = 0x%02X\n", ext_csd[232]);
	printf("ext_csd->sec_feature_support = 0x%02X\n", ext_csd[231]);
	printf("ext_csd->sec_erase_mult = 0x%02X\n", ext_csd[230]);
	printf("ext_csd->sec_trim_mult = 0x%02X\n", ext_csd[229]);
	printf("ext_csd->boot_info = 0x%02X\n", ext_csd[228]);
	printf("ext_csd->boot_size_mult = 0x%02X\n", ext_csd[226]);
	printf("ext_csd->acc_size = 0x%02X\n", ext_csd[225]);
	printf("ext_csd->hc_erase_gp_size = 0x%02X\n", ext_csd[224]);
	printf("ext_csd->erase_timeout_mult = 0x%02X\n", ext_csd[223]);
	printf("ext_csd->rel_wr_sec_c = 0x%02X\n", ext_csd[222]);
	printf("ext_csd->hc_wp_grp_size = 0x%02X\n", ext_csd[221]);
	printf("ext_csd->s_c_vcc = 0x%02X\n", ext_csd[220]);
	printf("ext_csd->s_c_vccq = 0x%02X\n", ext_csd[219]);
	printf("ext_csd->s_a_timeout = 0x%02X\n", ext_csd[217]);
	printf("ext_csd->sec_count = 0x%08X\n", ext_csd[215] * (1 << 24) |
		ext_csd[214] * (1 << 16)|ext_csd[213]*(1 << 8)|ext_csd[212]);
	printf("ext_csd->min_perf_w_8_52 = 0x%02X\n", ext_csd[210]);
	printf("ext_csd->min_perf_r_8_52 = 0x%02X\n", ext_csd[209]);
	printf("ext_csd->min_perf_w_8_26_4_52 = 0x%02X\n", ext_csd[208]);
	printf("ext_csd->min_perf_r_8_26_4_52 = 0x%02X\n", ext_csd[207]);
	printf("ext_csd->min_perf_w_4_26 = 0x%02X\n", ext_csd[206]);
	printf("ext_csd->min_perf_r_4_26 = 0x%02X\n", ext_csd[205]);
	printf("ext_csd->pwr_cl_26_360 = 0x%02X\n", ext_csd[203]);
	printf("ext_csd->pwr_cl_52_360 = 0x%02X\n", ext_csd[202]);
	printf("ext_csd->pwr_cl_26_195 = 0x%02X\n", ext_csd[201]);
	printf("ext_csd->pwr_cl_52_195 = 0x%02X\n", ext_csd[200]);
	printf("ext_csd->partition_switch_time = 0x%02X\n", ext_csd[199]);
	printf("ext_csd->out_of_interrupt_time = 0x%02X\n", ext_csd[198]);
	printf("ext_csd->card_type = 0x%02X\n", ext_csd[196]);
	printf("ext_csd->csd_structure = 0x%02X\n", ext_csd[194]);
	printf("ext_csd->ext_csd_rev = 0x%02X\n", ext_csd[192]);
	printf("ext_csd->cmd_set = 0x%02X\n", ext_csd[191]);
	printf("ext_csd->cmd_set_rev = 0x%02X\n", ext_csd[189]);
	printf("ext_csd->power_class = 0x%02X\n", ext_csd[187]);
	printf("ext_csd->hs_timing = 0x%02X\n", ext_csd[185]);
	printf("ext_csd->bus_width = 0x%02X\n", ext_csd[183]);
	printf("ext_csd->erased_mem_cont = 0x%02X\n", ext_csd[181]);
	printf("ext_csd->partition_config = 0x%02X\n", ext_csd[179]);
	printf("ext_csd->boot_config_prot = 0x%02X\n", ext_csd[178]);
	printf("ext_csd->boot_bus_width = 0x%02X\n", ext_csd[177]);
	printf("ext_csd->erase_group_def = 0x%02X\n", ext_csd[175]);
	printf("ext_csd->boot_wp = 0x%02X\n", ext_csd[173]);
	printf("ext_csd->user_wp = 0x%02X\n", ext_csd[171]);
	printf("ext_csd->fw_config = 0x%02X\n", ext_csd[169]);
	printf("ext_csd->rpmb_size_mult = 0x%02X\n", ext_csd[168]);
	printf("ext_csd->wr_rel_set = 0x%02X\n", ext_csd[167]);
	printf("ext_csd->wr_rel_param = 0x%02X\n", ext_csd[166]);
	printf("ext_csd->bkops_start = 0x%02X\n", ext_csd[164]);
	printf("ext_csd->bkops_en = 0x%02X\n", ext_csd[163]);
	printf("ext_csd->rst_n_function = 0x%02X\n", ext_csd[162]);
	printf("ext_csd->hpi_mgmt = 0x%02X\n", ext_csd[161]);
	printf("ext_csd->partitioning_support = 0x%02X\n", ext_csd[160]);
	printf("ext_csd->max_en_size_mult = 0x%08X\n", ext_csd[159] * (1 << 16)
		| ext_csd[158] * (1 << 8) | ext_csd[157]);
	printf("ext_csd->partitions_attribute = 0x%02X\n", ext_csd[156]);
	printf("ext_csd->partition_setting_completed = 0x%02X\n", ext_csd[155]);
	printf("ext_csd->gp_size_mult = 0x%08X\n",  ext_csd[146] * (1 << 24) |
		ext_csd[145] * (1 << 16) | ext_csd[144] * (1 << 8) |
		ext_csd[143]);
	printf("ext_csd->enh_size_mult = 0x%08X\n", ext_csd[142] * (1 << 16) |
		ext_csd[141]*(1 << 8) | ext_csd[140]);
	printf("ext_csd->enh_start_addr = 0x%08X\n", ext_csd[139] * (1 << 24) |
		ext_csd[138] * (1 << 16) | ext_csd[137] * (1 << 8) |
		ext_csd[136]);
	printf("ext_csd->sec_bad_blk_mgmnt = 0x%02X\n", ext_csd[134]);
}

int do_mmcinfo (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	struct mmc *mmc;
	int dev_num;

	if (argc < 2)
		dev_num = 0;
	else
		dev_num = simple_strtoul(argv[1], NULL, 0);

	mmc = find_mmc_device(dev_num);

	if (mmc) {
		mmc_init(mmc);

		print_mmcinfo(mmc);

		if ((argc > 2) && (strcmp(argv[2], "csd") == 0))
			print_csd(mmc);

		/* Print Ext CSD */
		if ((argc > 2) && (strcmp(argv[2], "ext_csd") == 0)) {
			char ext_csd[512];
			struct mmc_cmd cmd;
			struct mmc_data data;

			/* Get the Card Status Register */
			cmd.cmdidx = MMC_CMD_SEND_EXT_CSD;
			cmd.resp_type = MMC_RSP_R1;
			cmd.cmdarg = 0;
			cmd.flags = 0;

			data.dest = ext_csd;
			data.blocks = 1;
			data.blocksize = 512;
			data.flags = MMC_DATA_READ;

			if (!mmc->send_cmd(mmc, &cmd, &data))
				print_ext_csd(ext_csd);
		}
	}

	return 0;
}

U_BOOT_CMD(mmcinfo, 3, 0, do_mmcinfo,
	"mmcinfo <dev num> [csd | ext_csd] -- display MMC info\n",
	""
);

int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int rc = 0;

	switch (argc) {
	case 3:
		if (strcmp(argv[1], "rescan") == 0) {
			int dev = simple_strtoul(argv[2], NULL, 10);
			struct mmc *mmc = find_mmc_device(dev);

			if (!mmc)
				return 1;

			mmc_init(mmc);

			return 0;
		}

	case 0:
	case 1:
	case 4:
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;

	case 2:
		if (!strcmp(argv[1], "list")) {
			print_mmc_devices('\n');
			return 0;
		}
		return 1;
	default: /* at least 5 args */
		if (strcmp(argv[1], "read") == 0) {
			int dev = simple_strtoul(argv[2], NULL, 10);
			void *addr = (void *)simple_strtoul(argv[3], NULL, 16);
			u32 cnt = simple_strtoul(argv[5], NULL, 16);
			u32 n;
			u32 blk = simple_strtoul(argv[4], NULL, 16);
			struct mmc *mmc = find_mmc_device(dev);

			if (!mmc)
				return 1;

			printf("\nMMC read: dev # %d, block # %d, count %d\n",
				dev, blk, cnt);

			mmc_init(mmc);

			n = mmc->block_dev.block_read(dev, blk, cnt, addr);

			/* flush cache after read */
			flush_cache((ulong)addr, cnt * 512); /* FIXME */

			printf("%d blocks read: %s\n",
				n, (n==cnt) ? "OK" : "ERROR");
			return (n == cnt) ? 0 : 1;
		} else if (strcmp(argv[1], "write") == 0) {
			int dev = simple_strtoul(argv[2], NULL, 10);
			void *addr = (void *)simple_strtoul(argv[3], NULL, 16);
			u32 cnt = simple_strtoul(argv[5], NULL, 16);
			u32 n;
			struct mmc *mmc = find_mmc_device(dev);

			int blk = simple_strtoul(argv[4], NULL, 16);

			if (!mmc)
				return 1;

			printf("\nMMC write: dev # %d, block # %d, count %d\n",
				dev, blk, cnt);

			mmc_init(mmc);

			n = mmc->block_dev.block_write(dev, blk, cnt, addr);

			printf("%d blocks written: %s\n",
				n, (n == cnt) ? "OK" : "ERROR");
			return (n == cnt) ? 0 : 1;
		} else {
			printf("Usage:\n%s\n", cmdtp->usage);
			rc = 1;
		}

		return rc;
	}
}

U_BOOT_CMD(
	mmc, 6, 1, do_mmcops,
	"MMC sub system",
	"read <device num> addr blk# cnt\n"
	"mmc write <device num> addr blk# cnt\n"
	"mmc rescan <device num>\n"
	"mmc list - lists available devices");
#endif

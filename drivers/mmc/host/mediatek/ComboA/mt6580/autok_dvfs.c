/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/delay.h>
#include <linux/fs.h>

#include "autok_dvfs.h"
#include "mtk_sd.h"
#include "dbg.h"
#include <mmc/core/sdio_ops.h>

static char const * const sdio_autok_res_path[] = {
	"/data/sdio_autok_0", "/data/sdio_autok_1",
	"/data/sdio_autok_2", "/data/sdio_autok_3",
};

#define AUTOK_MERGE_MIN_WIN     10
#define SDIO_AUTOK_DIFF_MARGIN  3

int sdio_autok_res_apply(struct msdc_host *host, int vcore)
{
	return 0;
}

int sdio_autok_res_save(struct msdc_host *host, int vcore, u8 *res)
{
	return 0;
}

void msdc_dvfs_reg_restore(struct msdc_host *host)
{
	/* MT6580 no DVFS register */
}

void sdio_dvfs_reg_restore(struct msdc_host *host)
{
	/* MT6580 no DVFS register */
}

int sd_execute_dvfs_autok(struct msdc_host *host, u32 opcode)
{
	int ret = 0;
	int vcore = 0;
	u8 *res;

	res = host->autok_res[vcore];

	if (host->mmc->ios.timing == MMC_TIMING_UHS_SDR104 ||
	    host->mmc->ios.timing == MMC_TIMING_UHS_SDR50) {
		if (host->is_autok_done == 0) {
			pr_notice("[AUTOK]SDcard autok\n");
			ret = autok_execute_tuning(host, res);
			host->is_autok_done = 1;
		} else {
			autok_init_sdr104(host);
			autok_tuning_parameter_init(host, res);
		}
	}

	return ret;
}


int emmc_execute_dvfs_autok(struct msdc_host *host, u32 opcode)
{
	int ret = 0;
	int vcore = 0;
	u8 *res;

	res = host->autok_res[vcore];

	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS200) {
#ifdef MSDC_HQA
		msdc_HQA_set_voltage(host);
#endif
		if (opcode == MMC_SEND_STATUS) {
			pr_notice("[AUTOK]eMMC HS200 Tune CMD only\n");
			ret = hs200_execute_tuning_cmd(host, res);
		} else {
			pr_notice("[AUTOK]eMMC HS200 Tune\n");
			ret = hs200_execute_tuning(host, res);
		}

		host->is_autok_done = 1;
		complete(&host->autok_done);
	}

	return ret;
}

void msdc_dvfs_reg_backup_init(struct msdc_host *host)
{
	pr_debug("msdc%d:notice msdc_dvfs_reg_backup_init info", host->id);
}

void sdio_execute_dvfs_autok(struct msdc_host *host)
{
}

/*
 * Vcore dvfs module MUST ensure having executed
 * the function before mmcblk0 inited + 3s,
 * otherwise will fail because of entering runtime
 * supsend.
 */
int emmc_autok(void)
{
	struct msdc_host *host = mtk_msdc_host[0];

	if (!host || !host->mmc) {
		pr_notice("eMMC device not ready\n");
		return -1;
	}

	if (!(host->mmc->caps2 & MMC_CAP2_HS400_1_8V)
	 && !(host->mmc->caps2 & MMC_CAP2_HS200_1_8V_SDR))
		return 0;

	/* Wait completion of AUTOK triggered by eMMC initialization */
	if (!wait_for_completion_timeout(&host->autok_done, 10 * HZ)) {
		pr_notice("eMMC 1st autok not done\n");
		return -1;
	}

	pr_info("emmc autok\n");

	return 0;
}
EXPORT_SYMBOL(emmc_autok);

/* FIX ME: Since card can be insert at any time but this is invoked only once
 * when DVFS ready, this function is not suitable for card insert after boot
 */
int sd_autok(void)
{
	struct msdc_host *host = mtk_msdc_host[1];

	if (!host || !host->mmc) {
		pr_notice("SD card not ready\n");
		return -1;
	}

	pr_info("sd autok\n");

	return 0;
}
EXPORT_SYMBOL(sd_autok);

int sdio_autok(void)
{
	return 0;
}
EXPORT_SYMBOL(sdio_autok);

void msdc_dump_autok(char **buff, unsigned long *size,
	struct seq_file *m, struct msdc_host *host)
{
	int i, j;
	int bit_pos, byte_pos, start;
	char buf[65];

	SPREAD_PRINTF(buff, size, m, "[AUTOK]VER : 0x%02x%02x%02x%02x\r\n",
		host->autok_res[0][AUTOK_VER3],
		host->autok_res[0][AUTOK_VER2],
		host->autok_res[0][AUTOK_VER1],
		host->autok_res[0][AUTOK_VER0]);

	for (i = AUTOK_VCORE_LEVEL0; i >= AUTOK_VCORE_LEVEL0; i--) {
		start = CMD_SCAN_R0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]CMD Rising \t: %s\r\n", buf);

		start = CMD_SCAN_F0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]CMD Falling \t: %s\r\n", buf);

		start = DAT_SCAN_R0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]DAT Rising \t: %s\r\n", buf);

		start = DAT_SCAN_F0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]DAT Falling \t: %s\r\n", buf);

		/* cmd response use ds pin, but window is
		 * different with data pin, because cmd response is SDR.
		 */
		start = DS_CMD_SCAN_0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]DS CMD Window \t: %s\r\n", buf);

		start = DS_DAT_SCAN_0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]DS DAT Window \t: %s\r\n", buf);

		start = D_DATA_SCAN_0;
		for (j = 0; j < 32; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]Device Data RX \t: %s\r\n", buf);

		start = H_DATA_SCAN_0;
		for (j = 0; j < 32; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]Host   Data TX \t: %s\r\n", buf);

		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]CMD [EDGE:%d CMD_FIFO_EDGE:%d DLY1:%d DLY2:%d]\r\n",
			host->autok_res[i][0], host->autok_res[i][1],
			host->autok_res[i][5], host->autok_res[i][7]);
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]DAT [RDAT_EDGE:%d RD_FIFO_EDGE:%d WD_FIFO_EDGE:%d]\r\n",
			host->autok_res[i][2], host->autok_res[i][3],
			host->autok_res[i][4]);
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]DAT [LATCH_CK:%d DLY1:%d DLY2:%d]\r\n",
			host->autok_res[i][13], host->autok_res[i][9],
			host->autok_res[i][11]);
		SPREAD_PRINTF(buff, size, m,
			"[AUTOK]DS  [DLY1:%d DLY2:%d DLY3:%d]\r\n",
			host->autok_res[i][14], host->autok_res[i][16],
			host->autok_res[i][18]);
		SPREAD_PRINTF(buff, size, m, "[AUTOK]DAT [TX SEL:%d]\r\n",
			host->autok_res[i][20]);
	}
}


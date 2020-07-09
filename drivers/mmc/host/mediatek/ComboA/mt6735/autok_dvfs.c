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
#include <linux/regulator/consumer.h>
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
#if defined(VCOREFS_READY)
	if (host->use_hw_dvfs == 0) {
		vcore = AUTOK_VCORE_MERGE;
	} else {
		pr_err("[AUTOK]Need change para when dvfs\n");
	}
#endif
	res = host->autok_res[vcore];
	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS200) {

		if (opcode == MMC_SEND_STATUS) {
			pr_notice("[AUTOK]eMMC HS200 Tune CMD only\n");
			ret = hs200_execute_tuning_cmd(host, res);
		} else {
			pr_notice("[AUTOK]eMMC HS200 Tune\n");
			ret = hs200_execute_tuning(host, res);
		}

		if (host->mmc->card &&
				!(host->mmc->card->mmc_avail_type
					& EXT_CSD_CARD_TYPE_HS400)) {
			host->is_autok_done = 1;
			complete(&host->autok_done);
		}
	} else if (host->mmc->ios.timing == MMC_TIMING_MMC_HS400) {

		if (opcode == MMC_SEND_STATUS) {
			pr_notice("[AUTOK]eMMC HS400 Tune CMD only\n");
			ret = hs400_execute_tuning_cmd(host, res);
		} else {
			pr_notice("[AUTOK]eMMC HS400 Tune\n");
			ret = hs400_execute_tuning(host, res);
		}
		host->is_autok_done = 1;
		complete(&host->autok_done);
	}

	return ret;
}

#ifdef EMMC_RUNTIME_AUTOK_MERGE
int emmc_runtime_autok_merge(u32 opcode)
{
#if !defined(FPGA_PLATFORM)
	struct msdc_host *host = mtk_msdc_host[0];
	void __iomem *base;
	int merge_result, merge_mode, merge_window;
	int i, ret = 0;

	if (!(host->mmc->caps2 & MMC_CAP2_HS400_1_8V)
	 && !(host->mmc->caps2 & MMC_CAP2_HS200_1_8V_SDR)) {
		return ret;
	}
	pr_info("emmc runtime autok merge\n");
	base = host->base;

	memcpy(host->autok_res[AUTOK_VCORE_LEVEL0],
		host->autok_res[AUTOK_VCORE_MERGE],
			TUNING_PARA_SCAN_COUNT);

	ret = emmc_execute_dvfs_autok(host, opcode);
	if (host->use_hw_dvfs == 0)
		memcpy(host->autok_res[AUTOK_VCORE_LEVEL1],
			host->autok_res[AUTOK_VCORE_MERGE],
				TUNING_PARA_SCAN_COUNT);

	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS400)
		merge_mode = MERGE_HS400;
	else
		merge_mode = MERGE_HS200_SDR104;

	merge_result = autok_vcore_merge_sel(host, merge_mode);
	for (i = CMD_MAX_WIN; i <= H_CLK_TX_MAX_WIN; i++) {
		merge_window = host->autok_res[AUTOK_VCORE_MERGE][i];
		if (merge_window < AUTOK_MERGE_MIN_WIN) {
			merge_result = -2;
			pr_info("[AUTOK]%s:merge_window[%d] less than %d\n",
				__func__, i, AUTOK_MERGE_MIN_WIN);
		}
		if (merge_window != 0xFF)
			pr_info("[AUTOK]merge_value = %d\n", merge_window);
	}

	if (merge_result == 0) {
		autok_tuning_parameter_init(host,
			host->autok_res[AUTOK_VCORE_MERGE]);
		pr_info("[AUTOK]No need change para when dvfs\n");
	} else if (host->use_hw_dvfs == 1) {
		pr_info("[AUTOK]Need change para when dvfs\n");
	} else if (host->use_hw_dvfs == 0) {
		if (merge_result == -1)
			autok_tuning_parameter_init(host,
				host->autok_res[AUTOK_VCORE_LEVEL0]);
		else if (merge_result == -2) {
			autok_tuning_parameter_init(host,
				host->autok_res[AUTOK_VCORE_LEVEL1]);
			memcpy(host->autok_res[AUTOK_VCORE_MERGE],
				host->autok_res[AUTOK_VCORE_LEVEL1],
					TUNING_PARA_SCAN_COUNT);
		}
		pr_info("[AUTOK]restore legacy window\n");
	}
#endif

	return ret;
}
#endif


void msdc_dvfs_reg_backup_init(struct msdc_host *host)
{
	pr_debug("msdc%d:notice msdc_dvfs_reg_backup_init info", host->id);
}

void sdio_execute_dvfs_autok(struct msdc_host *host)
{
}

#if defined(VCOREFS_READY)
static int autok_opp[AUTOK_VCORE_NUM] = {
	OPPI_PERF_ULTRA, /* 0.8V */  //is these vaule corrcet? 0.8V?
	OPPI_PERF, /* 0.7V */
	OPPI_LOW_PWR, /* 0.675V or 0.7V */
};
#endif

/*
 * Vcore dvfs module MUST ensure having executed
 * the function before mmcblk0 inited + 3s,
 * otherwise will fail because of entering runtime
 * supsend.
 */
int emmc_autok(void)
{
#if !defined(FPGA_PLATFORM) && defined(VCOREFS_READY)
	struct msdc_host *host = mtk_msdc_host[0];
	void __iomem *base;
	int merge_result, merge_mode, merge_window;
	int i, vcore_step1 = -1, vcore_step2 = 0;

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
	base = host->base;
	mmc_claim_host(host->mmc);

	for (i = 0; i < AUTOK_VCORE_NUM; i++) {

          pr_debug("[AUTOK] emmc_autok VCORE= %d \n", i);

         if (vcorefs_request_dvfs_opp(KIR_AUTOK_EMMC, autok_opp[i]) != 0)
			pr_err("vcorefs_request_dvfs_opp@L%d fail!\n", i);

          vcore_step2 = vcore_pmic_to_uv(pmic_get_register_value(PMIC_VCORE1_VOSEL_ON));

           pr_debug("[AUTOK] get regulator volt vcore_step1: %d vocre_step2: %d\n",
                    vcore_step1, vcore_step2);
		if (vcore_step2 == vcore_step1) {
			pr_info("skip duplicated vcore autok\n");
			memcpy(host->autok_res[i], host->autok_res[i-1],
				TUNING_PARA_SCAN_COUNT);
		} else {
			emmc_execute_dvfs_autok(host,
				MMC_SEND_TUNING_BLOCK_HS200);
			if (host->use_hw_dvfs == 0)
				memcpy(host->autok_res[i],
					host->autok_res[AUTOK_VCORE_MERGE],
					TUNING_PARA_SCAN_COUNT);
		}
		vcore_step1 = vcore_step2;
	}

	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS400)
		merge_mode = MERGE_HS400;
	else
		merge_mode = MERGE_HS200_SDR104;

	merge_result = autok_vcore_merge_sel(host, merge_mode);
	for (i = CMD_MAX_WIN; i <= H_CLK_TX_MAX_WIN; i++) {
		merge_window = host->autok_res[AUTOK_VCORE_MERGE][i];
		if (merge_window < AUTOK_MERGE_MIN_WIN)
			merge_result = -1;
		if (merge_window != 0xFF)
			pr_info("[AUTOK]merge_value = %d\n", merge_window);
	}

	if (merge_result == 0) {
		autok_tuning_parameter_init(host,
			host->autok_res[AUTOK_VCORE_MERGE]);
		pr_info("[AUTOK]No need change para when dvfs\n");
	} else if (host->use_hw_dvfs == 1) {
		pr_info("[AUTOK]Need change para when dvfs\n");
	} else if (host->use_hw_dvfs == 0) {
		autok_tuning_parameter_init(host,
			host->autok_res[AUTOK_VCORE_LEVEL0]);
		pr_info("[AUTOK]Need lock vcore\n");
		host->lock_vcore = 1;
	}

if (vcorefs_request_dvfs_opp(KIR_AUTOK_EMMC, OPPI_UNREQ) != 0)
	pr_notice("vcorefs_request_dvfs_opp@OPP_UNREQ fail!\n");

	mmc_release_host(host->mmc);
#endif

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

	for (i = AUTOK_VCORE_NUM; i >= AUTOK_VCORE_LEVEL0; i--) {
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


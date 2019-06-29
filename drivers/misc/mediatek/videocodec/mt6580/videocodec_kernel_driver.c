/*
 * Copyright (C) 2018 MediaTek Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <asm/page.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
/* #include <mach/irqs.h> */
/* #include <mach/x_define_irq.h> */
#include <linux/wait.h>
#include <linux/proc_fs.h>
#include <linux/semaphore.h>
#include <mt-plat/dma.h>
#include <linux/delay.h>
#include "mt-plat/sync_write.h"
#include "mt_clkmgr.h"

#ifdef CONFIG_MTK_HIBERNATION
#include <mtk_hibernate_dpm.h>
#endif

#include "videocodec_kernel_driver.h"
#include "../videocodec_kernel.h"
#include <asm/cacheflush.h>
#include <linux/io.h>
#include <asm/sizes.h>
#include "val_types_private.h"
#include "val_api_private.h"
#include "drv_api.h"

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

/* memory signature for memory protection */
#define MEM_SIGNATURE 0x56636F64

#define VDO_HW_WRITE(ptr, data)     mt_reg_sync_writel(data, ptr)
#define VDO_HW_READ(ptr)            (*((volatile unsigned int * const)(ptr)))

#define VCODEC_DEVNAME     "Vcodec"
#define VCODEC_DEV_MAJOR_NUMBER 160	/* 189 */

static dev_t vcodec_devno = MKDEV(VCODEC_DEV_MAJOR_NUMBER, 0);
static struct cdev *vcodec_cdev;
static struct class *vcodec_class;
static struct device *vcodec_device;

static DEFINE_MUTEX(IsOpenedLock);
static DEFINE_MUTEX(PWRLock);
/* static DEFINE_MUTEX(VdecHWLock); */
/* static DEFINE_MUTEX(VencHWLock); */
/* static DEFINE_MUTEX(HWLock); */
static DEFINE_MUTEX(EncEMILock);
static DEFINE_MUTEX(L2CLock);
static DEFINE_MUTEX(DecEMILock);
static DEFINE_MUTEX(DriverOpenCountLock);
static DEFINE_MUTEX(NonCacheMemoryListLock);
/* static DEFINE_MUTEX(DecHWLockEventTimeoutLock); */
/* static DEFINE_MUTEX(EncHWLockEventTimeoutLock); */
static DEFINE_MUTEX(HWLockEventTimeoutLock);

/* static DEFINE_MUTEX(VdecPWRLock); */
/* static DEFINE_MUTEX(VencPWRLock); */
static DEFINE_MUTEX(InitHWLock);

static DEFINE_SPINLOCK(OalHWContextLock);
static DEFINE_SPINLOCK(DecIsrLock);
static DEFINE_SPINLOCK(EncIsrLock);
static DEFINE_SPINLOCK(LockDecHWCountLock);
static DEFINE_SPINLOCK(LockEncHWCountLock);
static DEFINE_SPINLOCK(DecISRCountLock);
static DEFINE_SPINLOCK(EncISRCountLock);
static DEFINE_SPINLOCK(HWLock);
static DEFINE_SPINLOCK(VdecPWRLock);
static DEFINE_SPINLOCK(VencPWRLock);


/* static struct VAL_EVENT_T DecHWLockEvent; //mutex : HWLockEventTimeoutLock */
/* static struct VAL_EVENT_T EncHWLockEvent; //mutex : HWLockEventTimeoutLock */
/* mutex : HWLockEventTimeoutLock */
static struct VAL_EVENT_T HWLockEvent;
static struct VAL_EVENT_T DecIsrEvent;
static struct VAL_EVENT_T EncIsrEvent;
static int Driver_Open_Count;	/* mutex : DriverOpenCountLock */
static unsigned int gu4PWRCounter;	/* mutex : PWRLock */
static unsigned int gu4EncEMICounter;	/* mutex : EncEMILock */
static unsigned int gu4DecEMICounter;	/* mutex : DecEMILock */
static unsigned int gu4L2CCounter;	/* mutex : L2CLock */
static char bIsOpened = VAL_FALSE;	/* mutex : IsOpenedLock */
static unsigned int gu4HwVencIrqStatus;	/* hardware VENC IRQ status(VP8/H264) */

static unsigned int gu4VdecPWRCounter;	/* mutex : VdecPWRLock */
static unsigned int gu4VencPWRCounter;	/* mutex : VencPWRLock */
static int gu4HybPWRCounter;	/* no mutex, just for debug */

static unsigned int gLockTimeOutCount;

static unsigned int gu4VdecLockThreadId;

/* #define VCODEC_DEBUG */
#ifdef VCODEC_DEBUG
#undef VCODEC_DEBUG
#define VCODEC_DEBUG pr_info
#undef pr_debug
#define pr_debug  pr_info
#else
#define VCODEC_DEBUG(...)
#undef pr_debug
#define pr_debug(...)
#endif

/* VENC physical base address */
#undef VENC_BASE
#define VENC_BASE       0x17002000
#define VENC_REGION     0x1000

#undef MP4_VENC_BASE
#define MP4_VENC_BASE       0x15009000
#define MP4_VENC_REGION     0x1000

/* VDEC virtual base address */
#define VDEC_BASE_PHY   0x1500B000
#define VDEC_REGION     0x2000

#define HW_BASE         0x7FFF000
#define HW_REGION       0x2000

#define INFO_BASE       0x10000000
#define INFO_REGION     0x1000

#define VENC_IRQ_STATUS_SPS         0x1
#define VENC_IRQ_STATUS_PPS         0x2
#define VENC_IRQ_STATUS_FRM         0x4
#define VENC_IRQ_STATUS_DRAM        0x8
#define VENC_IRQ_STATUS_PAUSE       0x10
#define VENC_IRQ_STATUS_SWITCH      0x20

unsigned long KVA_VENC_IRQ_ACK_ADDR, KVA_VENC_IRQ_STATUS_ADDR, KVA_VENC_BASE;
unsigned long KVA_VENC_ZERO_COEF_COUNT_ADDR,
KVA_VENC_BYTE_COUNT_ADDR;	/* hybrid */
unsigned long KVA_VDEC_MISC_BASE, KVA_VDEC_VLD_BASE;
unsigned long KVA_VDEC_BASE, KVA_VDEC_GCON_BASE;
unsigned long KVA_VDEC_INT_STA_ADDR, KVA_VDEC_INT_ACK_ADDR;	/* hybrid */
unsigned int VENC_IRQ_ID, VDEC_IRQ_ID;
unsigned long KVA_VENC_MP4_IRQ_ENABLE_ADDR;

void vdec_power_on(void)
{
	unsigned long ulFlagsVdecPWRLock;

	spin_lock_irqsave(&VdecPWRLock, ulFlagsVdecPWRLock);
	gu4VdecPWRCounter++;
	spin_unlock_irqrestore(&VdecPWRLock, ulFlagsVdecPWRLock);
	++gu4HybPWRCounter;

	/* Central power on */
	pr_debug("[VCODEC] vdec_power_on + (%d) (%d) (%d)\n",
			gu4VdecPWRCounter, gu4HybPWRCounter, gu4VDecIRQCount);
	enable_clock(MT_CG_DISP0_SMI_COMMON, "VDEC");
	enable_clock(MT_CG_VCODEC_CKPDN, "VDEC");
	enable_clock(MT_CG_LARB1_SMI_CKPDN, "VDEC");
	pr_debug("[VCODEC] vdec_power_on -\n");
}

void vdec_power_off(void)
{
	unsigned long ulFlagsVdecPWRLock;

	spin_lock_irqsave(&VdecPWRLock, ulFlagsVdecPWRLock);
	if (gu4VdecPWRCounter != 0) {
		gu4VdecPWRCounter--;
		--gu4HybPWRCounter;
		/* Central power off */
		pr_debug("[VCODEC] vdec_power_off + (%d) (%d) (%d)\n",
				gu4VdecPWRCounter,
				gu4HybPWRCounter,
				gu4VDecIRQCount);
		disable_clock(MT_CG_LARB1_SMI_CKPDN, "VDEC");
		disable_clock(MT_CG_VCODEC_CKPDN, "VDEC");
		disable_clock(MT_CG_DISP0_SMI_COMMON, "VDEC");
		pr_debug("[VCODEC] vdec_power_off -\n");
	}
	spin_unlock_irqrestore(&VdecPWRLock, ulFlagsVdecPWRLock);
}

void venc_power_on(void)
{
	unsigned long ulFlagsVencPWRLock;

	spin_lock_irqsave(&VencPWRLock, ulFlagsVencPWRLock);
	gu4VencPWRCounter++;
	spin_unlock_irqrestore(&VencPWRLock, ulFlagsVencPWRLock);
	++gu4HybPWRCounter;

	pr_debug("[VCODEC] venc_power_on + (%d) (%d) (%d)\n",
			gu4VencPWRCounter, gu4HybPWRCounter, gu4VEncIRQCount);
	enable_clock(MT_CG_DISP0_SMI_COMMON, "VENC");
	enable_clock(MT_CG_VCODEC_CKPDN, "VENC");
	enable_clock(MT_CG_LARB1_SMI_CKPDN, "VENC");

	/* enable_clock(MT_CG_MM_CODEC_SW_CG, "VideoClock"); */
	/* larb_clock_on(0, "VideoClock"); */
	VDO_HW_WRITE(KVA_VENC_MP4_IRQ_ENABLE_ADDR, 0x1);
	pr_debug("[VCODEC] venc_power_on -\n");
}

void venc_power_off(void)
{
	unsigned long ulFlagsVencPWRLock;

	spin_lock_irqsave(&VencPWRLock, ulFlagsVencPWRLock);
	if (gu4VencPWRCounter != 0) {
		gu4VencPWRCounter--;
		--gu4HybPWRCounter;
		pr_debug("[VCODEC] venc_power_off + (%d) (%d) (%d)\n",
				gu4VencPWRCounter,
				gu4HybPWRCounter,
				gu4VEncIRQCount);
		disable_clock(MT_CG_LARB1_SMI_CKPDN, "VENC");
		disable_clock(MT_CG_VCODEC_CKPDN, "VENC");
		disable_clock(MT_CG_DISP0_SMI_COMMON, "VENC");

		/* disable_clock(MT_CG_MM_CODEC_SW_CG, "VideoClock"); */
		/* larb_clock_off(0, "VideoClock"); */
		pr_debug("[VCODEC] venc_power_off -\n");
	}
	spin_unlock_irqrestore(&VencPWRLock, ulFlagsVencPWRLock);
}

void dec_isr(void)
{
	enum VAL_RESULT_T eValRet;
	unsigned long ulFlags, ulFlagsISR, ulFlagsLockHW, ulFlagsHWLock;

	unsigned int u4TempDecISRCount = 0;
	unsigned int u4TempLockDecHWCount = 0;
	unsigned int u4DecDoneStatus = 0;
	unsigned int reg_val;
	unsigned int index, i, maxnum;
	unsigned int u4IRQStatus = 0;
	enum VAL_RESULT_T eValHWLockRet = VAL_RESULT_INVALID_ISR;

	if (VcodecHWLock.eDriverType != VAL_DRIVER_TYPE_H264_DEC
	    && VcodecHWLock.eDriverType != VAL_DRIVER_TYPE_VP8_DEC) {

		u4DecDoneStatus = VDO_HW_READ(KVA_VDEC_BASE + 0xA4);
		if ((u4DecDoneStatus & (0x1 << 16)) != 0x10000) {
			pr_info("[VCODEC][ERROR] DEC ISR, Dec done status is not 0x1 (0x%08x)",
					u4DecDoneStatus);
			return;
		}
	}


	spin_lock_irqsave(&DecISRCountLock, ulFlagsISR);
	gu4DecISRCount++;
	u4TempDecISRCount = gu4DecISRCount;
	spin_unlock_irqrestore(&DecISRCountLock, ulFlagsISR);

	spin_lock_irqsave(&LockDecHWCountLock, ulFlagsLockHW);
	u4TempLockDecHWCount = gu4LockDecHWCount;
	spin_unlock_irqrestore(&LockDecHWCountLock, ulFlagsLockHW);

	if (VcodecHWLock.eDriverType == VAL_DRIVER_TYPE_H264_DEC ||
	    VcodecHWLock.eDriverType == VAL_DRIVER_TYPE_VP8_DEC) {
		spin_lock_irqsave(&OalHWContextLock, ulFlags);
		index = search_slot_byHdl(0,
				(unsigned long) VcodecHWLock.pvHandle);

		/* in case, if the process is killed first,
		 * then receive an ISR from HW,
		 * the event information already cleared.
		 */
		if (index == -1)	{ /* Hybrid */
			pr_debug("[VCODEC][ERROR][ISR] Can't find any index in ISR\n");

			/* ACK interrupt */
			/* decoder */
			reg_val = VDO_HW_READ(KVA_VDEC_INT_STA_ADDR);
			VDO_HW_WRITE(KVA_VDEC_INT_ACK_ADDR, reg_val);
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

			return;
		}
		/* get address from context */

		maxnum = hw_ctx[index].u4NumOfRegister;
		if (hw_ctx[index].u4NumOfRegister
			> VCODEC_INST_NUM) {
			pr_info("[VCODEC][ERROR][ISR] hw_ctx[index].u4NumOfRegister =%d\n",
			hw_ctx[index].u4NumOfRegister);
			maxnum = VCODEC_INST_NUM;
		}

		if ((((volatile unsigned int *)
		    hw_ctx[index].kva_u4HWIsCompleted) == NULL)
		    || (((volatile unsigned int *)
			hw_ctx[index].kva_u4HWIsTimeout) ==
			NULL)) {
			pr_info("[VCODEC][ERROR][ISR] index = %d, please check\n",
					index);

			/* ACK interrupt */
			/* decoder */
			reg_val = VDO_HW_READ(KVA_VDEC_INT_STA_ADDR);
			VDO_HW_WRITE(KVA_VDEC_INT_ACK_ADDR, reg_val);
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

			return;
		}
		*((volatile unsigned int *)hw_ctx[index].kva_u4HWIsCompleted)
		 = 1;
		*((volatile unsigned int *)hw_ctx[index].kva_u4HWIsTimeout)
		 = 0;

		for (i = 0; i < maxnum; i++) {
			*((volatile unsigned int *)
			hw_ctx[index].kva_Oal_HW_mem_reg + i * 2 + 1) =
					*((volatile unsigned int *)
					hw_ctx[index].
					oalmem_status[i].
					u4ReadAddr);

			if (maxnum == 3) {
				if (i == 0) {
					u4IRQStatus =
						(*
						((volatile unsigned int *)
						hw_ctx[index].
						kva_Oal_HW_mem_reg +
						i * 2 + 1));
					if (u4IRQStatus != 2) {
						pr_info("[VCODEC][ERROR][ISR] IRQ status %d error\n",
								u4IRQStatus);
					}
				}

				if (u4IRQStatus != 2) {
					pr_info("[VCODEC][ERROR][ISR] %d, 0x%lx, %d, %d, %d, %d\n",
						i,
						(unsigned long) (
						(volatile unsigned int *)
						hw_ctx[index].
						oalmem_status[i].
						u4ReadAddr),
						(*
						((volatile unsigned int *)
						hw_ctx[index].
						kva_Oal_HW_mem_reg +
						i * 2 + 1)),
						VDO_HW_READ(
						KVA_VENC_IRQ_ACK_ADDR),
						VDO_HW_READ(
						KVA_VENC_ZERO_COEF_COUNT_ADDR),
						VDO_HW_READ(
						KVA_VENC_BYTE_COUNT_ADDR));
				}
			}

		}
		/* set handle event */
		eValRet = eVideoSetEvent(&hw_ctx[index].IsrEvent,
						sizeof(struct VAL_EVENT_T));
		spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

		if (eValRet != VAL_RESULT_NO_ERROR)
			pr_info("[VCODEC][ERROR] ISR set IsrEvent error\n");
		/* Clear interrupt */
		/* ACK interrupt */
		/* decoder */
		reg_val = VDO_HW_READ(KVA_VDEC_INT_STA_ADDR);
		VDO_HW_WRITE(KVA_VDEC_INT_ACK_ADDR, reg_val);

		/* Unlock */
		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		for (i = 0; i < VCODEC_THREAD_MAX_NUM; i++)
			VcodecHWLock.u4VCodecThreadID[i] = -1;

		VcodecHWLock.u4VCodecThreadNum = VCODEC_THREAD_MAX_NUM;
		VcodecHWLock.rLockedTime.u4Sec = 0;
		VcodecHWLock.rLockedTime.u4uSec = 0;
		VcodecHWLock.pvHandle = 0;
		disable_irq_nosync(VDEC_IRQ_ID);
		--gu4VDecIRQCount;
		/* turn vdec power off */
		vdec_power_off();

		eValHWLockRet = eVideoSetEvent(&HWLockEvent,
		sizeof(struct VAL_EVENT_T));
		if (eValHWLockRet != VAL_RESULT_NO_ERROR)
			pr_info("[VCODEC][ERROR] ISR set dec_isr error\n");

		spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);
		return;
	}

	/* Clear interrupt */
	VDO_HW_WRITE(KVA_VDEC_MISC_BASE + 41 * 4,
		VDO_HW_READ(KVA_VDEC_MISC_BASE + 41 * 4) | 0x11);
	VDO_HW_WRITE(KVA_VDEC_MISC_BASE + 41 * 4,
		VDO_HW_READ(KVA_VDEC_MISC_BASE + 41 * 4) & ~0x10);


	spin_lock_irqsave(&DecIsrLock, ulFlags);
	eValRet = eVideoSetEvent(&DecIsrEvent, sizeof(struct VAL_EVENT_T));
	if (eValRet != VAL_RESULT_NO_ERROR)
		pr_info("[VCODEC][ERROR] ISR set DecIsrEvent error\n");

	spin_unlock_irqrestore(&DecIsrLock, ulFlags);
}


void enc_isr(void)
{
	enum VAL_RESULT_T eValRet;
	unsigned int index, i, maxnum;
	unsigned long ulFlags, ulFlagsISR, ulFlagsLockHW, ulFlagsHWLock;
	unsigned int u4IRQStatus = 0;

	unsigned int u4TempEncISRCount = 0;
	unsigned int u4TempLockEncHWCount = 0;
	enum VAL_RESULT_T eValHWLockRet = VAL_RESULT_INVALID_ISR;

	spin_lock_irqsave(&EncISRCountLock, ulFlagsISR);
	gu4EncISRCount++;
	u4TempEncISRCount = gu4EncISRCount;
	spin_unlock_irqrestore(&EncISRCountLock, ulFlagsISR);

	spin_lock_irqsave(&LockEncHWCountLock, ulFlagsLockHW);
	u4TempLockEncHWCount = gu4LockEncHWCount;
	spin_unlock_irqrestore(&LockEncHWCountLock, ulFlagsLockHW);

	if (VcodecHWLock.pvHandle == 0) {
		pr_info("[VCODEC][ERROR][ISR] NO one Lock Enc HW, please check!!\n");

		/* Clear all status */
		if (VcodecHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC) {
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_PAUSE);
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_SWITCH);
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_DRAM);
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_SPS);
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_PPS);
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_FRM);
		} else if (VcodecHWLock.eDriverType ==
			VAL_DRIVER_TYPE_MP4_ENC) {
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, 1);
		}

		return;
	}
	/* hardwire */
	if (VcodecHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC) {
		gu4HwVencIrqStatus = VDO_HW_READ(KVA_VENC_IRQ_STATUS_ADDR);
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_PAUSE)
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_PAUSE);
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_SWITCH)
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_SWITCH);
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_DRAM)
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_DRAM);
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_SPS)
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_SPS);
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_PPS)
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_PPS);
		if (gu4HwVencIrqStatus & VENC_IRQ_STATUS_FRM)
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR,
					VENC_IRQ_STATUS_FRM);
	} else if (VcodecHWLock.eDriverType ==
				VAL_DRIVER_TYPE_HEVC_ENC) { /* hardwire */
		pr_info("[VCODEC][ISR] VAL_DRIVER_TYPE_HEVC_ENC\n");
	} else if (VcodecHWLock.eDriverType ==
				VAL_DRIVER_TYPE_MP4_ENC) {
		spin_lock_irqsave(&OalHWContextLock, ulFlags);
		index = search_slot_byHdl(0,
					(unsigned long) VcodecHWLock.pvHandle);

		/* in case, if the process is killed first, */
		/* then receive an ISR from HW,
		 * the event information already cleared.
		 */
		if (index == -1)	{ /* Hybrid */
			pr_info("[VCODEC][ERROR][ISR] Can't find any index in ISR\n");
			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, 1);
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

			return;
		}
		/* get address from context */

		maxnum = hw_ctx[index].u4NumOfRegister;
		if (hw_ctx[index].u4NumOfRegister > VCODEC_INST_NUM) {
			pr_info("[VCODEC][ERROR][ISR] hw_ctx[index].u4NumOfRegister =%d\n",
					hw_ctx[index].u4NumOfRegister);
			maxnum = VCODEC_INST_NUM;
		}

		if ((((volatile unsigned int *)
			hw_ctx[index].kva_u4HWIsCompleted) == NULL)
		    || (((volatile unsigned int *)
				hw_ctx[index].kva_u4HWIsTimeout) == NULL)) {
			pr_info("[VCODEC][ERROR][ISR] index = %d, please check!!\n",
					index);

			VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, 1);
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

			return;
		}
		*((volatile unsigned int *)
				hw_ctx[index].kva_u4HWIsCompleted) = 1;
		*((volatile unsigned int *)
				hw_ctx[index].kva_u4HWIsTimeout) = 0;

		for (i = 0; i < maxnum; i++) {
			*((volatile unsigned int *)
					hw_ctx[index].kva_Oal_HW_mem_reg +
			  i * 2 + 1) =
			*((volatile unsigned int *)
					hw_ctx[index].oalmem_status[i].
					u4ReadAddr);

			if (maxnum == 3) {
				if (i == 0) {
					u4IRQStatus =
						(*
						((volatile unsigned int *)
						hw_ctx[index].
						kva_Oal_HW_mem_reg +
						i * 2 + 1));
					if (u4IRQStatus != 2) {
						pr_info("[VCODEC][ERROR][ISR] IRQ status error u4IRQStatus = %d\n",
								u4IRQStatus);
					}
				}

				if (u4IRQStatus != 2) {
					pr_info("[VCODEC][ERROR][ISR] %d, 0x%lx, %d, %d, %d, %d\n",
						i,
						(unsigned long)
						((volatile unsigned int *)
						hw_ctx[index].
						oalmem_status[i].u4ReadAddr),
						(*
						((volatile unsigned int *)
						hw_ctx[index].
						kva_Oal_HW_mem_reg +
						i * 2 + 1)),
						VDO_HW_READ(
						KVA_VENC_IRQ_ACK_ADDR),
						VDO_HW_READ(
						KVA_VENC_ZERO_COEF_COUNT_ADDR
						),
						VDO_HW_READ(
						KVA_VENC_BYTE_COUNT_ADDR
						));
				}
			}
		}
		/* set handle event */
		eValRet = eVideoSetEvent(&hw_ctx[index].IsrEvent,
						sizeof(struct VAL_EVENT_T));
		spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

		/* Clear interrupt */
		/* ACK interrupt */
		/* encoder */
		VDO_HW_WRITE(KVA_VENC_IRQ_ACK_ADDR, 1);

		/* Unlock */
		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		for (i = 0; i < VCODEC_THREAD_MAX_NUM; i++)
			VcodecHWLock.u4VCodecThreadID[i] = -1;

		VcodecHWLock.u4VCodecThreadNum = VCODEC_THREAD_MAX_NUM;
		VcodecHWLock.rLockedTime.u4Sec = 0;
		VcodecHWLock.rLockedTime.u4uSec = 0;
		VcodecHWLock.pvHandle = 0;
		disable_irq_nosync(VENC_IRQ_ID);
		--gu4VEncIRQCount;
		/* turn venc power off */
		venc_power_off();

		eValHWLockRet = eVideoSetEvent(&HWLockEvent,
					sizeof(struct VAL_EVENT_T));
		if (eValHWLockRet != VAL_RESULT_NO_ERROR)
			pr_info("[VCODEC][ERROR] ISR set enc_isr error\n");

		spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);

		return;
	}
	if (VcodecHWLock.eDriverType != VAL_DRIVER_TYPE_H264_ENC &&
		VcodecHWLock.eDriverType != VAL_DRIVER_TYPE_HEVC_ENC) {
		pr_info("[VCODEC][ERROR][ISR] Invalid lock holder driver type = %d\n",
				VcodecHWLock.eDriverType);
	}

	spin_lock_irqsave(&EncIsrLock, ulFlags);
	eValRet = eVideoSetEvent(&EncIsrEvent,
					sizeof(struct VAL_EVENT_T));
	if (eValRet != VAL_RESULT_NO_ERROR)
		pr_info("[VCODEC][ERROR] ISR set EncIsrEvent error\n");

	spin_unlock_irqrestore(&EncIsrLock, ulFlags);
}

static irqreturn_t video_intr_dlr(int irq, void *dev_id)
{
	dec_isr();
	return IRQ_HANDLED;
}

static irqreturn_t video_intr_dlr2(int irq, void *dev_id)
{
	enc_isr();
	return IRQ_HANDLED;
}

static long vcodec_alloc_non_cache_buffer(unsigned long arg)
{
	unsigned char *user_data_addr;
	struct VAL_MEMORY_T rTempMem;
	long ret;
	unsigned int u4TempVCodecThreadNum;
	unsigned int u4TempVCodecThreadID[VCODEC_THREAD_MAX_NUM];
	unsigned long ulFlags;
	int i4I;
	int i4Index = -1;

	pr_info("VCODEC_ALLOC_NON_CACHE_BUFFER + tid = %d\n",
			current->pid);

	user_data_addr = (unsigned char *)arg;
	ret = copy_from_user(&rTempMem, user_data_addr,
				sizeof(struct VAL_MEMORY_T));
	if (ret) {
		pr_info("[ERROR] VCODEC_ALLOC_NCACHE_BUFFER, copy_from_user failed: %lu\n",
				ret);
		return -EFAULT;
	}

	rTempMem.u4MemSign = MEM_SIGNATURE;
	rTempMem.u4ReservedSize /*kernel va*/ =
			(unsigned long)dma_alloc_coherent(
					vcodec_device,
					rTempMem.u4MemSize,
					(dma_addr_t *)&rTempMem.pvMemPa,
					GFP_KERNEL);
	if ((rTempMem.u4ReservedSize == 0) || (rTempMem.pvMemPa == 0)) {
		pr_info("[ERROR] dma_alloc_coherent fail in VCODEC_ALLOC_NCACHE_BUFFER\n");
		return -EFAULT;
	}

	pr_debug("[VCODEC] kernel va = 0x%lx, kernel pa = 0x%lx, mem size = %lu\n",
			(unsigned long)rTempMem.u4ReservedSize,
			(unsigned long)rTempMem.pvMemPa,
			(unsigned long)rTempMem.u4MemSize);

	spin_lock_irqsave(&OalHWContextLock, ulFlags);
	i4Index = search_slot_byTID(0, current->pid);
	if (i4Index == -1) {
		pr_info("[VCODEC][ERROR] add_ncmem error, u4Index = -1\n");
				spin_unlock_irqrestore(&OalHWContextLock,
				ulFlags);
		return -EFAULT;
	}

	u4TempVCodecThreadNum =
			hw_ctx[i4Index].u4VCodecThreadNum;
	for (i4I = 0; i4I < u4TempVCodecThreadNum; i4I++) {
		u4TempVCodecThreadID[i4I] =
				hw_ctx[i4Index].u4VCodecThreadID[i4I];
	}
	spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

	mutex_lock(&NonCacheMemoryListLock);
	add_ncmem(rTempMem.u4ReservedSize,
			(unsigned long) rTempMem.pvMemPa,
			(unsigned long) rTempMem.u4MemSize,
			u4TempVCodecThreadNum, u4TempVCodecThreadID);
	mutex_unlock(&NonCacheMemoryListLock);

	ret = copy_to_user(user_data_addr, &rTempMem,
				sizeof(struct VAL_MEMORY_T));
	if (ret) {
		pr_info("[ERROR] VCODEC_ALLOC_NON_CACHE_BUFFER, copy_to_user failed: %lu\n",
				ret);
		return -EFAULT;
	}

	pr_info("VCODEC_ALLOC_NON_CACHE_BUFFER - tid = %d\n",
			current->pid);

	return 0;
}

static long vcodec_free_non_cache_buffer(unsigned long arg)
{
	unsigned char *user_data_addr;
	struct VAL_MEMORY_T rTempMem;
	long ret;

	pr_info("VCODEC_FREE_NON_CACHE_BUFFER + tid = %d\n",
			current->pid);

	user_data_addr = (unsigned char *)arg;
	ret = copy_from_user(&rTempMem, user_data_addr,
				sizeof(struct VAL_MEMORY_T));
	if (ret) {
		pr_info("[ERROR] VCODEC_FREE_NCACHE_BUFFER, copy_from_user failed: %lu",
				ret);
		return -EFAULT;
	}
	if (rTempMem.u4MemSign != MEM_SIGNATURE) {
		pr_info("[ERROR] VCODEC_FREE_NON_CACHE_BUFFER, memory illegal: %d\n",
				rTempMem.u4MemSign);
		return -EFAULT;
	}
	if (rTempMem.u4MemSize == 0 ||
		rTempMem.u4ReservedSize == 0) {
		pr_info("[ERROR] VCODEC_FREE_NON_CACHE_BUFFER, memory size illegal\n");
		return -EFAULT;
	}

	dma_free_coherent(vcodec_device, rTempMem.u4MemSize,
			(void *)rTempMem.u4ReservedSize,
			(dma_addr_t) rTempMem.pvMemPa);

	mutex_lock(&NonCacheMemoryListLock);
	free_ncmem((unsigned long) rTempMem.u4ReservedSize,
			(unsigned long) rTempMem.pvMemPa);
	mutex_unlock(&NonCacheMemoryListLock);

	rTempMem.u4ReservedSize = 0;
	rTempMem.pvMemPa = NULL;

	ret = copy_to_user(user_data_addr, &rTempMem,
				sizeof(struct VAL_MEMORY_T));
	if (ret) {
		pr_info("[ERROR] VCODEC_FREE_NCACHE_BUFFER, copy_to_user failed: %lu",
				ret);
		return -EFAULT;
	}

	pr_info("VCODEC_FREE_NON_CACHE_BUFFER - tid = %d\n",
			current->pid);

	return 0;
}

static long vcodec_lockhw_dec_fail(struct VAL_HW_LOCK_T rHWLock,
				unsigned int FirstUseDecHW)
{
	unsigned long ulFlagsHWLock;

	pr_info("[ERROR] VCODEC_LOCKHW, HWLockEvent TimeOut, CurrentTID = %d\n",
			current->pid);
	if (FirstUseDecHW != 1) {
		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		if (VcodecHWLock.pvHandle == 0)
			pr_info("[WARNING] VDEC_LOCKHW, maybe mediaserver restart before, please check\n");
		else
			pr_info("[WARNING] VDEC_LOCKHW, someone use HW\n");

		spin_unlock_irqrestore(&HWLock,
				       ulFlagsHWLock);
	}

	return 0;
}



static long vcodec_lockhw_enc_fail(struct VAL_HW_LOCK_T rHWLock,
				unsigned int FirstUseEncHW)
{
	unsigned long ulFlagsHWLock;

	pr_info("[ERROR] VCODEC_LOCKHW HWLockEvent TimeOut, CurrentTID = %d\n",
			current->pid);
	if (FirstUseEncHW != 1) {
		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		if (VcodecHWLock.pvHandle == 0) {
			pr_info("[WARNING] VENC_LOCKHW, maybe mediaserver restart before, please check\n");
		} else {
			pr_info("[WARNING] VENC_LOCKHW, someone use HW, and check timeout value (%d)\n",
					gLockTimeOutCount);
			++gLockTimeOutCount;

			if (gLockTimeOutCount > 30) {
				pr_info("[ERROR] VENC_LOCKHW - ID %d fail, someone locked HW time out > 30 times 0x%lx, %lx, 0x%lx, type:%d\n",
						current->pid,
						(unsigned long)
						VcodecHWLock.
						pvHandle,
						pmem_user_v2p_video(
						(unsigned long)
						rHWLock.pvHandle),
						(unsigned long)
						rHWLock.pvHandle,
						rHWLock.eDriverType);
				gLockTimeOutCount = 0;
				spin_unlock_irqrestore
				    (&HWLock,
				     ulFlagsHWLock);
				return -EFAULT;
			}

			if (rHWLock.u4TimeoutMs == 0) {
				pr_info("[ERROR] VENC_LOCKHW - ID %d fail, someone locked HW already 0x%lx, %lx, 0x%lx, type:%d\n",
						current->pid,
						(unsigned long)
						VcodecHWLock.
						pvHandle,
						pmem_user_v2p_video(
						(unsigned long)
						rHWLock.pvHandle),
						(unsigned long)
						rHWLock.pvHandle,
						rHWLock.eDriverType);
				gLockTimeOutCount = 0;
				spin_unlock_irqrestore
				    (&HWLock,
				     ulFlagsHWLock);
				return -EFAULT;
			}
		}
		spin_unlock_irqrestore(&HWLock,
				       ulFlagsHWLock);
	}

	return 0;
}

static long vcodec_lockhw_vdec(struct VAL_HW_LOCK_T *pHWLock, char *bLockedHW)
{
	unsigned int FirstUseDecHW = 0;
	unsigned long ulFlagsLockHW;
	unsigned long handle = 0;
	long ret = 0;
	struct VAL_TIME_T rCurTime;
	unsigned int u4TimeInterval;
	enum VAL_RESULT_T eValRet = VAL_RESULT_NO_ERROR;
	unsigned long ulFlags, ulFlagsHWLock;
	unsigned int u4Index = 0xff;
	int i;

	while ((*bLockedHW) == VAL_FALSE) {
		mutex_lock(&HWLockEventTimeoutLock);
		if (HWLockEvent.u4TimeoutMs == 1) {
			pr_info("VDEC_LOCKHW, First Use Dec HW!!\n");
			FirstUseDecHW = 1;
		} else {
			FirstUseDecHW = 0;
		}
		mutex_unlock(&HWLockEventTimeoutLock);

		if (FirstUseDecHW == 1) {
			eValRet = eVideoWaitEvent(&HWLockEvent,
						sizeof(struct VAL_EVENT_T));
		}
		mutex_lock(&HWLockEventTimeoutLock);
		if (HWLockEvent.u4TimeoutMs != 1000) {
			HWLockEvent.u4TimeoutMs = 1000;
			FirstUseDecHW = 1;
		} else {
			FirstUseDecHW = 0;
		}
		mutex_unlock(&HWLockEventTimeoutLock);

		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		/* one process try to lock twice */
		handle = (unsigned long)(pHWLock->pvHandle);
		if (VcodecHWLock.pvHandle ==
			(void *)pmem_user_v2p_video(handle)) {
			pr_info("[WARNING] VDEC_LOCKHW, one decoder inst try to lock twice, may cause lock HW timeout! inst = 0x%lx, TID = %d\n",
					(unsigned long)VcodecHWLock.pvHandle,
					current->pid);
		}
		spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);

		if (FirstUseDecHW == 0) {
			pr_debug("VDEC_LOCKHW, Not first time use HW, timeout = %d\n",
					HWLockEvent.u4TimeoutMs);
			eValRet = eVideoWaitEvent(&HWLockEvent,
						sizeof(struct VAL_EVENT_T));
		}

		if (eValRet == VAL_RESULT_INVALID_ISR) {
			ret = vcodec_lockhw_dec_fail(*pHWLock, FirstUseDecHW);
			if (ret) {
				pr_info("[VDEC] lockhw_dec_fail: %lu\n",
						ret);
				return -EFAULT;
			}
		} else if (eValRet == VAL_RESULT_RESTARTSYS) {
			pr_info("[WARNING] VDEC_LOCKHW, VAL_RESULT_RESTARTSYS return when HWLock\n");
			return -ERESTARTSYS;
		}

		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		/* No one holds dec hw lock now */
		if (VcodecHWLock.pvHandle == 0)	{
			if (pHWLock->eDriverType ==
				VAL_DRIVER_TYPE_H264_DEC ||
				pHWLock->eDriverType ==
				VAL_DRIVER_TYPE_VP8_DEC) {
				/* for H264 hybrid decoder */
				spin_lock_irqsave(&OalHWContextLock,
						  ulFlags);
				u4Index = search_slot_byTID(0, current->pid);
				spin_unlock_irqrestore(&OalHWContextLock,
								ulFlags);

				if (u4Index == -1) {
					pr_info("[ERROR] VDEC_LOCKHW, No process use enc HW, so current process can use HW, u4Index = -1\n");
					spin_unlock_irqrestore(&HWLock,
							       ulFlagsHWLock);
					return -EFAULT;
				}

				gu4VdecLockThreadId = current->pid;
				VcodecHWLock.pvHandle =
				    (void *)
				    pmem_user_v2p_video(handle);
				VcodecHWLock.u4ThreadID = current->pid;
				pr_debug("VDEC_LOCKHW, No process use enc HW, so current process can use HW, handle = 0x%lx\n",
						(unsigned long)
						VcodecHWLock.pvHandle);

				spin_lock_irqsave(&OalHWContextLock,
							ulFlags);
				hw_ctx[u4Index].pvHandle =
					(unsigned long) VcodecHWLock.pvHandle;
				/* WaitISR need thread ID info. */
				VcodecHWLock.u4VCodecThreadNum =
					hw_ctx[u4Index].
					u4VCodecThreadNum;
				for (i = 0;
					i < VcodecHWLock.u4VCodecThreadNum;
					i++) {
					VcodecHWLock.u4VCodecThreadID[i] =
						hw_ctx[u4Index].
						u4VCodecThreadID[i];
				}
				spin_unlock_irqrestore(&OalHWContextLock,
								ulFlags);

				VcodecHWLock.eDriverType =
						pHWLock->eDriverType;
				eVideoGetTimeOfDay(&VcodecHWLock.
						rLockedTime,
						sizeof(struct VAL_TIME_T));

				pr_debug("VDEC_LOCKHW, LockInst = 0x%lx TID = %d, rLockedTime(s, us) = %d, %d\n",
						(unsigned long)
						VcodecHWLock.pvHandle,
						current->pid,
						VcodecHWLock.rLockedTime.u4Sec,
						VcodecHWLock.rLockedTime.
						u4uSec);

				*bLockedHW = VAL_TRUE;
				if (eValRet == VAL_RESULT_INVALID_ISR
						&& FirstUseDecHW != 1) {
					vdec_power_off();
					/* disable_irq(VDEC_IRQ_ID); */
					/* --gu4VDecIRQCount; */
					pr_info("[WARNING] VDEC_LOCKHW, reset power when HWLock (%d)\n",
							gu4VDecIRQCount);
				}
				vdec_power_on();
				/* enable_irq(MT_VDEC_IRQ_ID); */
				enable_irq(VDEC_IRQ_ID);
				++gu4VDecIRQCount;
			} else	{ /* other codecs */
				gu4VdecLockThreadId = current->pid;
				VcodecHWLock.pvHandle =
				    (void *)
				    pmem_user_v2p_video(handle);
				VcodecHWLock.eDriverType =
				    pHWLock->eDriverType;
				VcodecHWLock.u4ThreadID = current->pid;
				eVideoGetTimeOfDay(&VcodecHWLock.
						rLockedTime,
						sizeof(struct VAL_TIME_T));

				pr_debug("VDEC_LOCKHW, No process use dec HW, so current process can use HW\n");
				pr_debug("VDEC_LOCKHW, LockInst = 0x%lx TID = %d, rLockedTime(s, us) = %d, %d\n",
						handle,
						current->pid,
						VcodecHWLock.rLockedTime.u4Sec,
						VcodecHWLock.
						rLockedTime.u4uSec);

				*bLockedHW = VAL_TRUE;
				if (eValRet == VAL_RESULT_INVALID_ISR &&
					FirstUseDecHW != 1) {
					pr_info("[WARNING] VDEC_LOCKHW, reset power when HWLock\n");
					vdec_power_off();
					/* disable_irq(VDEC_IRQ_ID); */
					/* --gu4VDecIRQCount; */
				}
				vdec_power_on();
				enable_irq(VDEC_IRQ_ID);
				++gu4VDecIRQCount;

			}

		} else { /* Another one holding dec hw now */
			pr_info("VDEC_LOCKHW E\n");
			eVideoGetTimeOfDay(&rCurTime,
						sizeof(struct VAL_TIME_T));
			u4TimeInterval = (((((rCurTime.u4Sec -
					VcodecHWLock.rLockedTime.u4Sec) *
					1000000) + rCurTime.u4uSec) -
					VcodecHWLock.rLockedTime.u4uSec) /
					1000);

			pr_debug("VDEC_LOCKHW, someone use dec HW, and check timeout value\n");
			pr_debug("VDEC_LOCKHW, Inst = 0x%lx TID = %d, TimeInterval(ms) = %d, TimeOutValue(ms)) = %d\n",
					(unsigned long) VcodecHWLock.pvHandle,
					current->pid,
					u4TimeInterval,
					pHWLock->u4TimeoutMs);
			pr_debug("VDEC_LOCKHW Lock Inst = 0x%lx, Lock TID = %d, CurrTID = %d, rLockedTime(%d s, %d us), rCurTime(%d s, %d us)\n",
					(unsigned long) VcodecHWLock.pvHandle,
					gu4VdecLockThreadId,
					current->pid,
					VcodecHWLock.rLockedTime.u4Sec,
					VcodecHWLock.rLockedTime.u4uSec,
					rCurTime.u4Sec, rCurTime.u4uSec);

			/* 2012/12/16. Cheng-Jung Never steal hardware lock */
		}
		spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);
		spin_lock_irqsave(&LockDecHWCountLock, ulFlagsLockHW);
		gu4LockDecHWCount++;
		spin_unlock_irqrestore(&LockDecHWCountLock, ulFlagsLockHW);
	}
	return 0;
}

static long vcodec_lockhw_venc(struct VAL_HW_LOCK_T *pHWLock, char *bLockedHW)
{
	unsigned int FirstUseEncHW = 0;
	unsigned long handle = 0;
	long ret = 0;
	struct VAL_TIME_T rCurTime;
	unsigned int u4TimeInterval;
	enum VAL_RESULT_T eValRet = VAL_RESULT_NO_ERROR;
	unsigned long ulFlags, ulFlagsHWLock;
	unsigned int u4Index = 0xff;
	int i;

	while ((*bLockedHW) == VAL_FALSE) {
		/* Early break for JPEG VENC */
		if (pHWLock->u4TimeoutMs == 0) {
			if (VcodecHWLock.pvHandle != 0)
				break;
		}
		/* Wait to acquire Enc HW lock */
		mutex_lock(&HWLockEventTimeoutLock);
		if (HWLockEvent.u4TimeoutMs == 1) {
			pr_info("VENC_LOCKHW, First Use Enc HW %d!!\n",
					pHWLock->eDriverType);
			FirstUseEncHW = 1;
		} else {
			FirstUseEncHW = 0;
		}
		mutex_unlock(&HWLockEventTimeoutLock);
		if (FirstUseEncHW == 1) {
			eValRet = eVideoWaitEvent(&HWLockEvent,
						sizeof(struct VAL_EVENT_T));
		}

		mutex_lock(&HWLockEventTimeoutLock);
		if (HWLockEvent.u4TimeoutMs == 1) {
			HWLockEvent.u4TimeoutMs = 1000;
			FirstUseEncHW = 1;
		} else {
			FirstUseEncHW = 0;
			if (pHWLock->u4TimeoutMs == 0)
				HWLockEvent.u4TimeoutMs = 0;	/* No wait */
			else
				/* Wait indefinitely */
				HWLockEvent.u4TimeoutMs = 1000;
		}
		mutex_unlock(&HWLockEventTimeoutLock);

		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		handle = (unsigned long)(pHWLock->pvHandle);
		/* one process try to lock twice */
		if (VcodecHWLock.pvHandle ==
			(void *)pmem_user_v2p_video(handle)) {
			pr_info("[WARNING] VENC_LOCKHW, one encoder inst try to lock twice, may cause lock HW timeout! inst = 0x%lx, TID = %d, type:%d\n",
					(unsigned long) VcodecHWLock.pvHandle,
					current->pid, pHWLock->eDriverType);
		}
		spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);

		if (FirstUseEncHW == 0) {
			eValRet = eVideoWaitEvent(&HWLockEvent,
						sizeof(struct VAL_EVENT_T));
		}

		if (eValRet == VAL_RESULT_INVALID_ISR) {
			ret = vcodec_lockhw_enc_fail(*pHWLock, FirstUseEncHW);
			if (ret) {
				pr_info("lockhw_enc_fail: %lu\n",
						ret);
				return -EFAULT;
			}
		} else if (eValRet == VAL_RESULT_RESTARTSYS) {
			return -ERESTARTSYS;
		}

		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		if (VcodecHWLock.pvHandle == 0)	{
			/* No process use HW, so current process can use HW */
			if (pHWLock->eDriverType == VAL_DRIVER_TYPE_MP4_ENC) {
				spin_lock_irqsave(&OalHWContextLock,
						ulFlags);
				u4Index = search_slot_byTID(0,
							    current->pid);
				spin_unlock_irqrestore(&OalHWContextLock,
						       ulFlags);

				if (u4Index == -1) {
					pr_info("[ERROR] VENC_LOCKHW, No process use enc HW, so current process can use HW, u4Index = -1\n");
					spin_unlock_irqrestore(&HWLock,
							ulFlagsHWLock);
					return -EFAULT;
				}

				VcodecHWLock.pvHandle =
						(void *)
						pmem_user_v2p_video(handle);
				VcodecHWLock.u4ThreadID = current->pid;
				pr_debug("VENC_LOCKHW, No process use enc HW, so current process can use HW, handle = 0x%lx\n",
						(unsigned long)
						VcodecHWLock.pvHandle);
				VcodecHWLock.eDriverType =
						pHWLock->eDriverType;
				spin_lock_irqsave(&OalHWContextLock,
							ulFlags);
				/* WaitISR need thread ID info. */
				VcodecHWLock.u4VCodecThreadNum =
						hw_ctx[u4Index].
						u4VCodecThreadNum;
				for (i = 0;
					i < VcodecHWLock.u4VCodecThreadNum;
					i++) {
					VcodecHWLock.u4VCodecThreadID[i] =
					hw_ctx[u4Index].
					u4VCodecThreadID[i];
				}
				hw_ctx[u4Index].pvHandle =
						(unsigned long)
						VcodecHWLock.pvHandle;
				spin_unlock_irqrestore(&OalHWContextLock,
						ulFlags);

				eVideoGetTimeOfDay(&VcodecHWLock.
						rLockedTime,
						sizeof(struct VAL_TIME_T));

				pr_debug("VENC_LOCKHW, LockInst = 0x%lx TID = %d, rLockedTime(s, us) = %d, %d\n",
						(unsigned long)
						VcodecHWLock.pvHandle,
						current->pid,
						VcodecHWLock.rLockedTime.u4Sec,
						VcodecHWLock.
						rLockedTime.u4uSec);

				*bLockedHW = VAL_TRUE;
				venc_power_on();
				enable_irq(VENC_IRQ_ID);
				++gu4VEncIRQCount;
			} else if (pHWLock->eDriverType ==
					VAL_DRIVER_TYPE_H264_ENC
					|| pHWLock->eDriverType ==
					VAL_DRIVER_TYPE_HEVC_ENC
					|| pHWLock->eDriverType ==
					VAL_DRIVER_TYPE_JPEG_ENC) {
				VcodecHWLock.pvHandle =
						(void *)
						pmem_user_v2p_video(handle);
				pr_debug("VENC_LOCKHW, No process use HW, so current process can use HW, handle = 0x%lx\n",
						(unsigned long)
						VcodecHWLock.pvHandle);
				VcodecHWLock.u4ThreadID = current->pid;
				VcodecHWLock.eDriverType =
						pHWLock->eDriverType;
				eVideoGetTimeOfDay(
					&VcodecHWLock.rLockedTime,
					sizeof(struct VAL_TIME_T));

				pr_debug("VENC_LOCKHW, No process use HW, so current process can use HW\n");
				pr_debug("VENC_LOCKHW, LockInst = 0x%lx TID = %d, rLockedTime(s, us) = %d, %d\n",
						(unsigned long)
						VcodecHWLock.pvHandle,
						current->pid,
						VcodecHWLock.rLockedTime.u4Sec,
						VcodecHWLock.
						rLockedTime.u4uSec);

				*bLockedHW = VAL_TRUE;
				if (pHWLock->eDriverType ==
					VAL_DRIVER_TYPE_H264_ENC ||
					pHWLock->eDriverType ==
						VAL_DRIVER_TYPE_HEVC_ENC) {
					venc_power_on();
					/* enable_irq(MT_VENC_IRQ_ID); */
					enable_irq(VENC_IRQ_ID);
					++gu4VEncIRQCount;
				}
			}
		} else { /* someone use HW, and check timeout value */
			if (pHWLock->u4TimeoutMs == 0) {
				*bLockedHW = VAL_FALSE;
				spin_unlock_irqrestore(&HWLock,
						       ulFlagsHWLock);
				break;
			}

			eVideoGetTimeOfDay(&rCurTime,
						sizeof(struct VAL_TIME_T));
			u4TimeInterval =
					(((((rCurTime.u4Sec -
					VcodecHWLock.rLockedTime.u4Sec) *
					1000000) + rCurTime.u4uSec) -
					VcodecHWLock.rLockedTime.u4uSec)
					/ 1000);

			pr_debug("VENC_LOCKHW, someone use enc HW, and check timeout value\n");
			pr_debug("VENC_LOCKHW, LockInst = 0x%lx, Inst = 0x%lx, TID = %d, TimeInterval(ms) = %d, TimeOutValue(ms)) = %d\n",
					(unsigned long)
					VcodecHWLock.pvHandle,
					pmem_user_v2p_video(handle),
					current->pid,
					u4TimeInterval, pHWLock->u4TimeoutMs);
			pr_debug("VENC_LOCKHW, LockInst = 0x%lx, Inst = 0x%lx, TID = %d, rLockedTime(s, us) = %d, %d, rCurTime(s, us) = %d, %d\n",
					(unsigned long)
					VcodecHWLock.pvHandle,
					pmem_user_v2p_video(handle),
					current->pid,
					VcodecHWLock.rLockedTime.u4Sec,
					VcodecHWLock.rLockedTime.u4uSec,
					rCurTime.u4Sec, rCurTime.u4uSec);

			++gLockTimeOutCount;
			if (gLockTimeOutCount > 30) {
				pr_info("[ERROR] VENC_LOCKHW - ID %d  fail, someone locked HW > 30 times without timeout 0x%lx, %lx, 0x%lx, type:%d\n",
						current->pid,
						(unsigned long)
						VcodecHWLock.pvHandle,
						pmem_user_v2p_video(handle),
						handle,
						pHWLock->eDriverType);
				gLockTimeOutCount = 0;
				spin_unlock_irqrestore(&HWLock,
						ulFlagsHWLock);
				return -EFAULT;
			}

			/* 2013/04/10. Cheng-Jung Never steal hardware lock */
		}

		if (*bLockedHW == VAL_TRUE) {
			pr_debug("VENC_LOCKHW, Lock ok VcodecHWLock.pvHandle = 0x%lx, va:%lx, type:%d",
					(unsigned long)
					VcodecHWLock.pvHandle,
					(unsigned long) handle,
					pHWLock->eDriverType);
			gLockTimeOutCount = 0;
		}
		spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);
	}
	return 0;
}

static long vcodec_lockhw(unsigned long arg)
{
	unsigned char *user_data_addr;
	struct VAL_HW_LOCK_T rHWLock;
	unsigned long handle;
	enum VAL_RESULT_T eValRet;
	long ret;
	char bLockedHW = VAL_FALSE;
	unsigned long ulFlags, ulFlagsLockHW;

	pr_debug("VCODEC_LOCKHW + tid = %d\n",
			current->pid);
	user_data_addr = (unsigned char *) arg;
	ret = copy_from_user(&rHWLock, user_data_addr,
				sizeof(struct VAL_HW_LOCK_T));
	if (ret) {
		pr_info("[ERROR] VCODEC_LOCKHW, copy_from_user failed: %lu\n",
			 ret);
		return -EFAULT;
	}

	pr_debug("[VCODEC] LOCKHW eDriverType = %d\n", rHWLock.eDriverType);
	eValRet = VAL_RESULT_INVALID_ISR;
	if (rHWLock.eDriverType == VAL_DRIVER_TYPE_MP4_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_MP1_MP2_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VC1_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VC1_ADV_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VP8_DEC) {
		return vcodec_lockhw_vdec(&rHWLock, &bLockedHW);

	} else if (rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
		   rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC ||
		   rHWLock.eDriverType == VAL_DRIVER_TYPE_JPEG_ENC ||
		   rHWLock.eDriverType == VAL_DRIVER_TYPE_MP4_ENC) {
		ret = vcodec_lockhw_venc(&rHWLock, &bLockedHW);
		if (ret != 0) {
			/* If there is error, return immediately */
			return ret;
		}

		if (bLockedHW == VAL_FALSE) {
			handle = (unsigned long)rHWLock.pvHandle;
			pr_info("[ERROR] VENC_LOCKHW - ID %d  fail, someone locked HW already , 0x%lx, %lx, 0x%lx, type:%d\n",
					current->pid,
					(unsigned long) VcodecHWLock.pvHandle,
					pmem_user_v2p_video(handle),
					handle,
					rHWLock.eDriverType);
			gLockTimeOutCount = 0;
			return -EFAULT;
		}

		spin_lock_irqsave(&LockEncHWCountLock, ulFlagsLockHW);
		gu4LockEncHWCount++;
		spin_unlock_irqrestore(&LockEncHWCountLock, ulFlagsLockHW);

		pr_debug("VCODEC_LOCKHWed - tid = %d\n", current->pid);

		if (rHWLock.eDriverType == VAL_DRIVER_TYPE_MP4_ENC) {
			/* add for debugging checking */
			spin_lock_irqsave(&OalHWContextLock, ulFlags);
			ret = search_slot_byTID(0, current->pid);
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

			if (ret == -1) {
				pr_info("VENC_LOCKHW - ID %d  fail, didn't call InitHWLock\n",
						current->pid);
				return -EFAULT;
			}
		}
	} else {
		pr_info("[WARNING] VCODEC_LOCKHW Unknown instance\n");
		return -EFAULT;
	}
	pr_debug("VCODEC_LOCKHW - tid = %d\n", current->pid);
	return 0;
}

static long vcodec_unlockhw(unsigned long arg)
{
	unsigned char *user_data_addr;
	struct VAL_HW_LOCK_T rHWLock;
	unsigned long handle = 0;
	enum VAL_RESULT_T eValRet;
	long ret;
	unsigned long ulFlagsHWLock;

	pr_debug("VCODEC_UNLOCKHW + tid = %d\n", current->pid);

	user_data_addr = (unsigned char *)arg;
	ret = copy_from_user(&rHWLock, user_data_addr,
				sizeof(struct VAL_HW_LOCK_T));
	if (ret) {
		pr_info("[ERROR] VCODEC_UNLOCKHW, copy_from_user failed: %lu\n",
				ret);
		return -EFAULT;
	}

	pr_debug("VCODEC_UNLOCKHW eDriverType = %d\n",
			rHWLock.eDriverType);
	eValRet = VAL_RESULT_INVALID_ISR;
	handle = (unsigned long)rHWLock.pvHandle;
	if (rHWLock.eDriverType == VAL_DRIVER_TYPE_MP4_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_MP1_MP2_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VC1_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VC1_ADV_DEC ||
	    rHWLock.eDriverType == VAL_DRIVER_TYPE_VP8_DEC) {
		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		if (VcodecHWLock.pvHandle ==
			(void *) pmem_user_v2p_video(handle))	{
			/* Current owner give up hw lock */
			VcodecHWLock.pvHandle = 0;
			VcodecHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
			disable_irq(VDEC_IRQ_ID);
			/* TODO: check if turning power off is ok */
			vdec_power_off();

		} else	{ /* Not current owner */
			pr_info("[ERROR] VDEC_UNLOCKHW, Not owner trying to unlock dec hardware 0x%lx\n",
					pmem_user_v2p_video(handle));
			spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);
			return -EFAULT;
		}
		spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);
		eValRet = eVideoSetEvent(&HWLockEvent,
					sizeof(struct VAL_EVENT_T));
	} else if (rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
			rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC ||
			rHWLock.eDriverType == VAL_DRIVER_TYPE_JPEG_ENC ||
			rHWLock.eDriverType == VAL_DRIVER_TYPE_MP4_ENC) {
		if (rHWLock.eDriverType == VAL_DRIVER_TYPE_MP4_ENC) {
			/* Debug */
			pr_info("Hybrid VCODEC_UNLOCKHW\n");
		} else if (rHWLock.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
			   rHWLock.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC) {
			spin_lock_irqsave(&HWLock, ulFlagsHWLock);
			if (VcodecHWLock.pvHandle ==
				(void *) pmem_user_v2p_video(
				(unsigned long) rHWLock.pvHandle)) {
				/* Current owner give up hw lock */
				VcodecHWLock.pvHandle = 0;
				VcodecHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
				/* disable_irq(MT_VENC_IRQ_ID); */
				disable_irq(VENC_IRQ_ID);
				/* turn venc power off */
				venc_power_off();
			} else	{ /* Not current owner */
				/* [TODO] error handling */
				pr_info("[ERROR] VENC_UNLOCKHW, Not owner trying to unlock enc hardware 0x%lx\n",
						pmem_user_v2p_video(handle));
				spin_unlock_irqrestore(&HWLock,
					ulFlagsHWLock);
				return -EFAULT;
			}
			spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);
			eValRet = eVideoSetEvent(&HWLockEvent,
						sizeof(struct VAL_EVENT_T));
		}
	} else {
		pr_info("[WARNING] VCODEC_UNLOCKHW Unknown instance\n");
		return -EFAULT;
	}

	pr_debug("VCODEC_UNLOCKHW - tid = %d\n", current->pid);

	return 0;
}

static long vcodec_waitisr(unsigned long arg)
{
	unsigned char *user_data_addr;
	struct VAL_ISR_T val_isr;
	char bLockedHW = VAL_FALSE;
	unsigned long ulFlags, ulFlagsISR, ulFlagsHWLock;
	unsigned long handle;
	long ret;
	enum VAL_RESULT_T eValRet;
	unsigned int u4Index = 0xff;
	int i, j;
	enum VAL_RESULT_T eValHWLockRet = VAL_RESULT_INVALID_ISR;

	pr_debug("VCODEC_WAITISR + tid = %d\n", current->pid);
	user_data_addr = (unsigned char *) arg;
	ret = copy_from_user(&val_isr, user_data_addr,
				sizeof(struct VAL_ISR_T));
	if (ret) {
		pr_info("[ERROR] VCODEC_WAITISR, copy_from_user failed: %lu\n",
				ret);
		return -EFAULT;
	}

	handle = (unsigned long)val_isr.pvHandle;
	if (val_isr.eDriverType == VAL_DRIVER_TYPE_MP4_DEC ||
		val_isr.eDriverType == VAL_DRIVER_TYPE_HEVC_DEC ||
		val_isr.eDriverType == VAL_DRIVER_TYPE_H264_DEC ||
		val_isr.eDriverType == VAL_DRIVER_TYPE_MP1_MP2_DEC ||
		val_isr.eDriverType == VAL_DRIVER_TYPE_VC1_DEC ||
		val_isr.eDriverType == VAL_DRIVER_TYPE_VC1_ADV_DEC ||
		val_isr.eDriverType == VAL_DRIVER_TYPE_VP8_DEC) {
		if (val_isr.eDriverType == VAL_DRIVER_TYPE_H264_DEC
		    || val_isr.eDriverType == VAL_DRIVER_TYPE_VP8_DEC) {
			pr_debug("VCODEC_WAITISR_H264_DEC + tid = %d\n",
					current->pid);
			spin_lock_irqsave(&OalHWContextLock, ulFlags);
			/* use search by thread ID,
			 * because handle may freed in user space
			 */
			u4Index = search_slot_byTID(0, current->pid);
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

			if (u4Index == -1) {
				pr_info("[ERROR] VDEC_WAITISR Fail, handle = 0x%lx, tid = %d, index = -1\n",
						pmem_user_v2p_video(handle),
						current->pid);
				return -EFAULT;
			}

			pr_debug("VDEC_WAITISR, index = %d, start wait VCODEC_WAITISR handle 0x%lx\n",
					u4Index,
					pmem_user_v2p_video(handle));

			spin_lock_irqsave(&OalHWContextLock, ulFlags);
			hw_ctx[u4Index].IsrEvent.u4TimeoutMs =
			    val_isr.u4TimeoutMs;
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
			eValRet = eVideoWaitEvent(
					&hw_ctx[u4Index].IsrEvent,
					sizeof(struct VAL_EVENT_T));
			pr_debug("waitdone VDEC_WAITISR handle 0x%lx\n",
				 pmem_user_v2p_video(handle));

			if (eValRet == VAL_RESULT_INVALID_ISR) {
				if (DecIsrEvent.u4TimeoutMs == 1
				    && val_isr.eDriverType ==
				    VAL_DRIVER_TYPE_VP8_DEC) {
					pr_info("[ERROR] VDEC_WAITISR, VP8 intend to let first WAITISE fail with timeOut value = %d (%d)\n",
							DecIsrEvent.u4TimeoutMs,
							gu4VDecIRQCount);
				} else {
					pr_info("[ERROR] VDEC_WAITISR, WAIT_ISR_CMD TimeOut (%d)\n",
							gu4VDecIRQCount);
				}

				spin_lock_irqsave(&OalHWContextLock, ulFlags);
				*((volatile unsigned int *)hw_ctx[u4Index].
				  kva_u4HWIsCompleted) = 0;
				*((volatile unsigned int *)hw_ctx[u4Index].
				  kva_u4HWIsTimeout) = 1;
				spin_unlock_irqrestore(&OalHWContextLock,
								ulFlags);

				spin_lock_irqsave(&DecISRCountLock, ulFlagsISR);
				gu4DecISRCount++;
				spin_unlock_irqrestore(&DecISRCountLock,
								ulFlagsISR);

				/* Unlock */
				spin_lock_irqsave(&HWLock, ulFlagsHWLock);
				/* use search by thread ID to avoid
				 * handle already freed in user
				 * space that will cause KE!
				 */
				for (i = 0; i < VcodecHWLock.u4VCodecThreadNum;
					i++) {
					if (VcodecHWLock.u4VCodecThreadID[i] ==
						current->pid) {
						/* normal case */
					for (j = 0;
					j < VCODEC_THREAD_MAX_NUM;
					j++) {
						VcodecHWLock.
						u4VCodecThreadID[j] =
						-1;
					}
						VcodecHWLock.u4VCodecThreadNum =
							VCODEC_THREAD_MAX_NUM;
						VcodecHWLock.pvHandle = 0;
						VcodecHWLock.u4ThreadID = 0;
						VcodecHWLock.rLockedTime.u4Sec =
							0;
						VcodecHWLock.rLockedTime.
							u4uSec = 0;
						disable_irq_nosync(VDEC_IRQ_ID);
						--gu4VDecIRQCount;
						/* turn venc power off */
						vdec_power_off();

						eValHWLockRet =
							eVideoSetEvent(
								&HWLockEvent,
								sizeof
								(struct
								VAL_EVENT_T));
					if (VAL_RESULT_NO_ERROR !=
						eValHWLockRet) {
						pr_info("[ERROR] VDEC_WAITISR, set HWLockEvent error\n");
					}
						break;
					}
				}
				spin_unlock_irqrestore(&HWLock,
								ulFlagsHWLock);

				/* TODO: power down hw? */
				return -2;
			}
		} else {
			spin_lock_irqsave(&HWLock, ulFlagsHWLock);
			if (VcodecHWLock.pvHandle ==
			    (void *) pmem_user_v2p_video(handle)) {
				bLockedHW = VAL_TRUE;
			} else {
			}
			spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);

			if (bLockedHW == VAL_FALSE) {
				pr_info("[ERROR] VDEC_WAITISR, DO NOT have HWLock, so return fail\n");
				return -EFAULT;
			}

			spin_lock_irqsave(&DecIsrLock, ulFlags);
			DecIsrEvent.u4TimeoutMs = val_isr.u4TimeoutMs;
			spin_unlock_irqrestore(&DecIsrLock, ulFlags);

			eValRet = eVideoWaitEvent(&DecIsrEvent,
					sizeof(struct VAL_EVENT_T));
			if (eValRet == VAL_RESULT_INVALID_ISR) {
				return -2;
			} else if (eValRet == VAL_RESULT_RESTARTSYS) {
				pr_info("[WARNING] VDEC_WAITISR, VAL_RESULT_RESTARTSYS return when WAITISR\n");
				return -ERESTARTSYS;
			}
		}
	} else if (val_isr.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
		   val_isr.eDriverType == VAL_DRIVER_TYPE_HEVC_ENC ||
		   val_isr.eDriverType == VAL_DRIVER_TYPE_MP4_ENC) {
		if (val_isr.eDriverType == VAL_DRIVER_TYPE_MP4_ENC)	{
			/* Hybrid */
			pr_debug("VENC_WAITISR_MP4_ENC + tid = %d\n",
					current->pid);
			spin_lock_irqsave(&OalHWContextLock, ulFlags);
			/* use search by thread ID,
			 * because handle may freed in user space
			 */
			u4Index = search_slot_byTID(0,
							current->pid);
			spin_unlock_irqrestore(&OalHWContextLock,
							ulFlags);

			if (u4Index == -1) {
				pr_info("[ERROR] VENC_WAITISR Fail, handle = 0x%lx, tid = %d, index = -1\n",
						pmem_user_v2p_video(handle),
						current->pid);
				return -EFAULT;
			}

			pr_debug("VENC_WAITISR, index = %d, start wait VCODEC_WAITISR handle 0x%lx\n",
					u4Index,
					pmem_user_v2p_video(handle));

			spin_lock_irqsave(&OalHWContextLock, ulFlags);
			hw_ctx[u4Index].IsrEvent.u4TimeoutMs =
			    val_isr.u4TimeoutMs;
			spin_unlock_irqrestore(&OalHWContextLock,
					ulFlags);
			eValRet =
			    eVideoWaitEvent(&hw_ctx[u4Index].IsrEvent,
					    sizeof(struct VAL_EVENT_T));
			pr_debug("waitdone VENC_WAITISR handle 0x%lx\n",
				 pmem_user_v2p_video(handle));

			if (eValRet == VAL_RESULT_INVALID_ISR) {
				pr_info("[ERROR] VENC_WAITISR, WAIT_ISR_CMD TimeOut\n");

				spin_lock_irqsave(&OalHWContextLock, ulFlags);
				*((volatile unsigned int *)hw_ctx[u4Index].
				  kva_u4HWIsCompleted) = 0;
				*((volatile unsigned int *)hw_ctx[u4Index].
				  kva_u4HWIsTimeout) = 1;
				spin_unlock_irqrestore(&OalHWContextLock,
						ulFlags);

				spin_lock_irqsave(&EncISRCountLock, ulFlagsISR);
				gu4EncISRCount++;
				spin_unlock_irqrestore(&EncISRCountLock,
						       ulFlagsISR);

				/* Unlock */
				spin_lock_irqsave(&HWLock, ulFlagsHWLock);
				/* use search by thread ID to avoid handle
				 * already freed in user space
				 */
				/* that will cause KE! */
				for (i = 0; i < VcodecHWLock.u4VCodecThreadNum;
					i++) {
					if (VcodecHWLock.u4VCodecThreadID[i] ==
						current->pid)
					/* normal case */
					{
					for (j = 0;
					j < VCODEC_THREAD_MAX_NUM;
					j++) {
						VcodecHWLock.
						u4VCodecThreadID[j] =
						-1;
					}
						VcodecHWLock.u4VCodecThreadNum =
						    VCODEC_THREAD_MAX_NUM;
						VcodecHWLock.pvHandle = 0;
						VcodecHWLock.u4ThreadID = 0;
						VcodecHWLock.rLockedTime.u4Sec =
						    0;
						VcodecHWLock.
						rLockedTime.u4uSec =
						    0;
						disable_irq_nosync(VENC_IRQ_ID);
						--gu4VEncIRQCount;
						/* turn venc power off */
						venc_power_off();

						eValHWLockRet =
						    eVideoSetEvent(&HWLockEvent,
							sizeof
							(struct VAL_EVENT_T));
					if (eValHWLockRet !=
					    VAL_RESULT_NO_ERROR) {
						pr_info("[ERROR] VENC_WAITISR, set HWLockEvent error\n");
					}
						break;
					}
				}
				spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);

				/* TODO: power down hw? */
				return -2;
			}
		} else if (val_isr.eDriverType == VAL_DRIVER_TYPE_H264_ENC ||
			val_isr.eDriverType == VAL_DRIVER_TYPE_VP8_ENC)	{
			/* Pure HW */
			spin_lock_irqsave(&HWLock, ulFlagsHWLock);
			if (VcodecHWLock.pvHandle ==
			    (void *) pmem_user_v2p_video(handle)) {
				bLockedHW = VAL_TRUE;
			} else {
			}
			spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);

			if (bLockedHW == VAL_FALSE) {
				pr_info("[ERROR] VENC_WAITISR, DO NOT have enc HWLock, so return fail\n");
				return -EFAULT;
			}

			spin_lock_irqsave(&EncIsrLock, ulFlags);
			EncIsrEvent.u4TimeoutMs = val_isr.u4TimeoutMs;
			spin_unlock_irqrestore(&EncIsrLock, ulFlags);

			eValRet = eVideoWaitEvent(&EncIsrEvent,
					sizeof(struct VAL_EVENT_T));
			if (eValRet == VAL_RESULT_INVALID_ISR) {
				return -2;
			} else if (eValRet == VAL_RESULT_RESTARTSYS) {
				pr_info("[WARNING] VENC_WAITISR, VAL_RESULT_RESTARTSYS return when WAITISR!!\n");
				return -ERESTARTSYS;
			}

			if (val_isr.u4IrqStatusNum > 0) {
				val_isr.u4IrqStatus[0] = gu4HwVencIrqStatus;
				ret = copy_to_user(user_data_addr, &val_isr,
						sizeof(struct VAL_ISR_T));
				if (ret) {
					pr_info("[ERROR] VENC_WAITISR, copy_to_user failed: %lu\n",
					     ret);
					return -EFAULT;
				}
			}
		}
	} else {
		pr_info("[WARNING] VCODEC_WAITISR Unknown instance\n");
		return -EFAULT;
	}
	pr_debug("VCODEC_WAITISR - tid = %d\n", current->pid);

	return 0;
}

static long vcodec_unlocked_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	long ret;
	unsigned char *user_data_addr;
	unsigned long ulFlags;
	struct VAL_VCODEC_CORE_LOADING_T rTempCoreLoading;
	struct VAL_VCODEC_CPU_OPP_LIMIT_T rCpuOppLimit;
	int temp_nr_cpu_ids;
	struct VAL_POWER_T rPowerParam;
	struct VAL_VCODEC_THREAD_ID_T rTempTID;
	unsigned int u4Index = 0xff;
	unsigned int *pu4TempKVA;
	unsigned long u8TempKPA;

	switch (cmd) {
	case VCODEC_SET_THREAD_ID:
	{
		pr_debug("VCODEC_SET_THREAD_ID + tid = %d\n",
				current->pid);
		user_data_addr = (unsigned char *) arg;
		ret = copy_from_user(&rTempTID, user_data_addr,
					sizeof(struct VAL_VCODEC_THREAD_ID_T));
		if (ret) {
			pr_info("[ERROR] VCODEC_SET_THREAD_ID, copy_from_user failed: %ld\n",
					ret);
			return -EFAULT;
		}

		spin_lock_irqsave(&OalHWContextLock, ulFlags);
		set_slot_TID(rTempTID, &u4Index);
		spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

		if (u4Index == 0xff) {
			pr_info("[ERROR] VCODEC_SET_THREAD_ID error, u4Index = %d\n",
					u4Index);
		}

		pr_debug("VCODEC_SET_THREAD_ID - tid = %d\n",
				current->pid);
	}
	break;

	case VCODEC_ALLOC_NON_CACHE_BUFFER:
	{
		ret = vcodec_alloc_non_cache_buffer(arg);
		if (ret) {
			pr_info("[ERROR] VCODEC_ALLOC_NON_CACHE_BUFFER failed! %lu\n",
					ret);
			return ret;
		}
	}
	break;

	case VCODEC_FREE_NON_CACHE_BUFFER:
	{
		ret = vcodec_free_non_cache_buffer(arg);
		if (ret) {
			pr_info("[ERROR] VCODEC_FREE_NON_CACHE_BUFFER failed! %lu\n",
					ret);
			return ret;
		}
	}
	break;

	case VCODEC_INC_DEC_EMI_USER:
	{
		pr_debug("VCODEC_INC_DEC_EMI_USER + tid = %d\n",
				current->pid);

		mutex_lock(&DecEMILock);
		gu4DecEMICounter++;
		pr_debug("[VCODEC] DEC_EMI_USER = %d\n",
				gu4DecEMICounter);
		user_data_addr = (unsigned char *)arg;
		ret = copy_to_user(user_data_addr, &gu4DecEMICounter,
				sizeof(unsigned int));
		if (ret) {
			pr_info("[ERROR] VCODEC_INC_DEC_EMI_USER, copy_to_user fail: %lu\n",
					ret);
			mutex_unlock(&DecEMILock);
			return -EFAULT;
		}
		mutex_unlock(&DecEMILock);

		pr_debug("VCODEC_INC_DEC_EMI_USER - tid = %d\n",
				current->pid);
	}
	break;

	case VCODEC_DEC_DEC_EMI_USER:
	{
		pr_debug("VCODEC_DEC_DEC_EMI_USER + tid = %d\n",
				current->pid);

		mutex_lock(&DecEMILock);
		gu4DecEMICounter--;
		pr_debug("[VCODEC] DEC_EMI_USER = %d\n",
				gu4DecEMICounter);
		user_data_addr = (unsigned char *)arg;
		ret = copy_to_user(user_data_addr, &gu4DecEMICounter,
				sizeof(unsigned int));
		if (ret) {
			pr_info("[ERROR] VCODEC_DEC_DEC_EMI_USER, copy_to_user fail: %lu\n",
					ret);
			mutex_unlock(&DecEMILock);
			return -EFAULT;
		}
		mutex_unlock(&DecEMILock);

		pr_debug("VCODEC_DEC_DEC_EMI_USER - tid = %d\n",
				current->pid);
	}
	break;

	case VCODEC_INC_ENC_EMI_USER:
	{
		pr_debug("VCODEC_INC_ENC_EMI_USER + tid = %d\n",
				current->pid);

		mutex_lock(&EncEMILock);
		gu4EncEMICounter++;
		pr_info("[VCODEC] ENC_EMI_USER = %d\n",
				gu4EncEMICounter);
		user_data_addr = (unsigned char *)arg;
		ret = copy_to_user(user_data_addr, &gu4EncEMICounter,
				sizeof(unsigned int));
		if (ret) {
			pr_info("[ERROR] VCODEC_INC_ENC_EMI_USER, copy_to_user fail: %lu\n",
					ret);
			mutex_unlock(&EncEMILock);
			return -EFAULT;
		}
		mutex_unlock(&EncEMILock);

		pr_debug("VCODEC_INC_ENC_EMI_USER - tid = %d\n",
				current->pid);
	}
	break;

	case VCODEC_DEC_ENC_EMI_USER:
	{
		pr_debug("VCODEC_DEC_ENC_EMI_USER + tid = %d\n",
				current->pid);

		mutex_lock(&EncEMILock);
		gu4EncEMICounter--;
		pr_info("[VCODEC] ENC_EMI_USER = %d\n",
				gu4EncEMICounter);
		user_data_addr = (unsigned char *)arg;
		ret = copy_to_user(user_data_addr, &gu4EncEMICounter,
					sizeof(unsigned int));
		if (ret) {
			pr_info("[ERROR] VCODEC_DEC_ENC_EMI_USER, copy_to_user fail: %lu\n",
					ret);
			mutex_unlock(&EncEMILock);
			return -EFAULT;
		}
		mutex_unlock(&EncEMILock);

		pr_debug("VCODEC_DEC_ENC_EMI_USER - tid = %d\n",
				current->pid);
	}
	break;

	case VCODEC_LOCKHW:
	{
		ret = vcodec_lockhw(arg);
		if (ret) {
			pr_info("[ERROR] VCODEC_LOCKHW failed! %lu\n",
					ret);
			return ret;
		}
	}
	break;

	case VCODEC_UNLOCKHW:
	{
		ret = vcodec_unlockhw(arg);
		if (ret) {
			pr_info("[ERROR] VCODEC_UNLOCKHW failed! %lu\n",
					ret);
			return ret;
		}
	}
	break;

	case VCODEC_INC_PWR_USER:
	{
		pr_debug("VCODEC_INC_PWR_USER + tid = %d\n",
				current->pid);
		user_data_addr = (unsigned char *)arg;
		ret = copy_from_user(&rPowerParam, user_data_addr,
					sizeof(struct VAL_POWER_T));
		if (ret) {
			pr_info("[ERROR] VCODEC_INC_PWR_USER, copy_from_user fail: %lu\n",
					ret);
			return -EFAULT;
		}
		pr_debug("[VCODEC] INC_PWR_USER eDriverType = %d\n",
				rPowerParam.eDriverType);
		pr_debug("VCODEC_INC_PWR_USER - tid = %d\n",
				current->pid);
		}
	break;

	case VCODEC_DEC_PWR_USER:
	{
		pr_debug("VCODEC_DEC_PWR_USER + tid = %d\n",
				current->pid);
		user_data_addr = (unsigned char *)arg;
		ret = copy_from_user(&rPowerParam, user_data_addr,
					sizeof(struct VAL_POWER_T));
		if (ret) {
			pr_info("[ERROR] VCODEC_DEC_PWR_USER, copy_from_user fail: %lu\n",
					ret);
			return -EFAULT;
		}
		pr_debug("[VCODEC] DEC_PWR_USER eDriverType = %d\n",
				rPowerParam.eDriverType);
		pr_debug("VCODEC_DEC_PWR_USER - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_WAITISR:
	{
		ret = vcodec_waitisr(arg);
		if (ret) {
			pr_info("[ERROR] VCODEC_WAITISR failed! %lu\n",
					ret);
			return ret;
		}
	}
	break;

	case VCODEC_INITHWLOCK:
	{
		struct VAL_VCODEC_OAL_HW_CONTEXT_T *context;
		struct VAL_VCODEC_OAL_HW_REGISTER_T hwoal_reg;
		struct VAL_VCODEC_OAL_HW_REGISTER_T *kva_TempReg;
		struct VAL_VCODEC_OAL_MEM_STAUTS_T
					oal_mem_status[OALMEM_STATUS_NUM];
		unsigned int ret, i, pa_u4HWIsCompleted, pa_u4HWIsTimeout;
		unsigned long addr_pa;

		pr_debug("VCODEC_INITHWLOCK + - tid = %d\n", current->pid);

		/* Start to get content */
		/* take struct VAL_VCODEC_OAL_HW_REGISTER_T content */
		user_data_addr = (unsigned char *) arg;
		ret = copy_from_user(&hwoal_reg, user_data_addr,
				   sizeof(struct VAL_VCODEC_OAL_HW_REGISTER_T));

		addr_pa = pmem_user_v2p_video(
						(unsigned long)user_data_addr);

		spin_lock_irqsave(&OalHWContextLock, ulFlags);
		context = set_slot(addr_pa, current->pid);
		if (context == NULL) {
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
			pr_info("get context fail\n");
			return -EFAULT;
		}
		context->Oal_HW_reg =
				(struct VAL_VCODEC_OAL_HW_REGISTER_T *) arg;

		context->Oal_HW_mem_reg =
					(unsigned int *)
					hwoal_reg.pHWStatus;

		if (hwoal_reg.u4NumOfRegister != 0) {
			context->pa_Oal_HW_mem_reg =
			    pmem_user_v2p_video(
						(unsigned long)
						context->Oal_HW_mem_reg);
			if (context->pa_Oal_HW_mem_reg == 0) {
				spin_unlock_irqrestore(&OalHWContextLock,
						ulFlags);
				pr_info("get Oal_HW_mem_reg fail\n");
				return -EFAULT;
			}
		}
		pa_u4HWIsCompleted =
		    pmem_user_v2p_video((unsigned long)
					&(((struct
					VAL_VCODEC_OAL_HW_REGISTER_T *)
					user_data_addr)->u4HWIsCompleted));
		if (pa_u4HWIsCompleted == 0) {
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
			pr_info("get pa_u4HWIsCompleted fail\n");
			return -EFAULT;
		}
		pa_u4HWIsTimeout = pmem_user_v2p_video(
					(unsigned long)
					&(((struct
					VAL_VCODEC_OAL_HW_REGISTER_T *)
					user_data_addr)->u4HWIsTimeout));
		if (pa_u4HWIsTimeout == 0) {
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
			pr_info("get pa_u4HWIsTimeout fail\n");
			return -EFAULT;
		}
		pr_debug("[VCODEC] user_data_addr->u4HWIsCompleted ua = 0x%lx pa= 0x%x\n",
				(unsigned long)
				&(((struct VAL_VCODEC_OAL_HW_REGISTER_T *)
				user_data_addr)->
				u4HWIsCompleted), pa_u4HWIsCompleted);
		pr_debug("[VCODEC] user_data_addr->u4HWIsTimeout ua = 0x%lx pa= 0x%x\n",
				(unsigned long)
				&(((struct VAL_VCODEC_OAL_HW_REGISTER_T *)
				user_data_addr)->
				u4HWIsTimeout), pa_u4HWIsTimeout);

		ret = copy_from_user(&oal_mem_status[0],
					hwoal_reg.pHWStatus,
					hwoal_reg.u4NumOfRegister *
					sizeof(struct
					VAL_VCODEC_OAL_MEM_STAUTS_T));
		context->u4NumOfRegister = hwoal_reg.u4NumOfRegister;
		pr_info("[VCODEC_INITHWLOCK] ToTal %d u4NumOfRegister\n",
			 hwoal_reg.u4NumOfRegister);

		if (hwoal_reg.u4NumOfRegister != 0) {
			u8TempKPA = context->pa_Oal_HW_mem_reg;
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
			mutex_lock(&NonCacheMemoryListLock);
			pu4TempKVA =
			    (unsigned int *) search_ncmem_byKPA(u8TempKPA);
			mutex_unlock(&NonCacheMemoryListLock);
			if (pu4TempKVA == NULL) {
				pr_info("search pu4TempKVA fail\n");
				return -EFAULT;
			}
			spin_lock_irqsave(&OalHWContextLock, ulFlags);
			context->kva_Oal_HW_mem_reg = pu4TempKVA;
			pr_debug("[VCODEC] context->ua = 0x%lx  pa_Oal_HW_mem_reg = 0x%lx\n",
			     (unsigned long) context->Oal_HW_mem_reg,
			     context->pa_Oal_HW_mem_reg);
		}
		spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
		mutex_lock(&NonCacheMemoryListLock);
		kva_TempReg =
		    (struct VAL_VCODEC_OAL_HW_REGISTER_T *)
		    search_ncmem_byKPA(addr_pa);
		mutex_unlock(&NonCacheMemoryListLock);
		if (kva_TempReg == NULL) {
			pr_info("search kva_TempReg fail\n");
			return -EFAULT;
		}
		spin_lock_irqsave(&OalHWContextLock, ulFlags);
		context->kva_u4HWIsCompleted =
		    (unsigned long) (&(kva_TempReg->u4HWIsCompleted));
		context->kva_u4HWIsTimeout =
					(unsigned long)
					(&(kva_TempReg->u4HWIsTimeout));
		pr_debug("[VCODEC] kva_TempReg = 0x%lx, kva_u4HWIsCompleted = 0x%lx, kva_u4HWIsTimeout = 0x%lx\n",
				(unsigned long) kva_TempReg,
				context->kva_u4HWIsCompleted,
				context->kva_u4HWIsTimeout);

		for (i = 0; i < hwoal_reg.u4NumOfRegister; i++) {
			unsigned long kva;

			pr_info("[VCODEC][REG_INFO_1] [%d] 0x%lx 0x%x\n", i,
					oal_mem_status[i].u4ReadAddr,
					oal_mem_status[i].u4ReadData);

			addr_pa = pmem_user_v2p_video(
							(unsigned long)
							oal_mem_status[i].
							u4ReadAddr);
			spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
			if (addr_pa == 0) {
				pr_info("get addr_pa fail\n");
				return -EFAULT;
			}
			/* need to remap addr + data addr */
			kva = (unsigned long) ioremap(addr_pa, 8);
			spin_lock_irqsave(&OalHWContextLock, ulFlags);
			pr_info("[VCODEC][REG_INFO_2] [%d] pa = 0x%lx  kva = 0x%lx\n",
					i,
					addr_pa, kva);
			/* oal_mem_status[i].u4ReadAddr; */
			context->oalmem_status[i].u4ReadAddr = kva;
		}
		spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

		pr_debug("VCODEC_INITHWLOCK addr1 0x%lx addr2 0x%lx\n",
				(unsigned long)arg,
				(unsigned long)context->Oal_HW_mem_reg);
		pr_debug("VCODEC_INITHWLOCK - - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_DEINITHWLOCK:
	{
		unsigned char *user_data_addr;
		unsigned long addr_pa;

		pr_debug("VCODEC_DEINITHWLOCK + - tid = %d\n",
				current->pid);

		user_data_addr = (unsigned char *) arg;
		addr_pa = pmem_user_v2p_video((unsigned long)user_data_addr);

		spin_lock_irqsave(&OalHWContextLock, ulFlags);
		free_slot(addr_pa);
		spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
		pr_debug("VCODEC_DEINITHWLOCK - - tid = %d\n", current->pid);
	}
	break;

	case VCODEC_GET_CPU_LOADING_INFO:
	{
		unsigned char *user_data_addr;
		struct VAL_VCODEC_CPU_LOADING_INFO_T _temp = {0};

		pr_debug("VCODEC_GET_CPU_LOADING_INFO +\n");
		user_data_addr = (unsigned char *) arg;

		ret = copy_to_user(user_data_addr, &_temp,
				sizeof(struct VAL_VCODEC_CPU_LOADING_INFO_T));
		if (ret) {
			pr_info("[ERROR] VCODEC_GET_CPU_LOADING_INFO, copy_to_user failed: %lu\n",
					ret);
			return -EFAULT;
		}

		pr_debug("VCODEC_GET_CPU_LOADING_INFO -\n");
	}
	break;

	case VCODEC_GET_CORE_LOADING:
	{
		pr_debug("VCODEC_GET_CORE_LOADING + - tid = %d\n",
				current->pid);

		user_data_addr = (unsigned char *) arg;
		ret = copy_from_user(&rTempCoreLoading, user_data_addr,
				sizeof(struct VAL_VCODEC_CORE_LOADING_T));
		if (ret) {
			pr_info("[ERROR] VCODEC_GET_CORE_LOADING, copy_from_user failed: %lu\n",
					ret);
			return -EFAULT;
		}
		if (rTempCoreLoading.CPUid > num_possible_cpus()) {
			pr_info("CPUid(%d) > num_possible_cpus(%d)\n",
					rTempCoreLoading.CPUid,
					num_possible_cpus());
			return -EFAULT;
		}
		if (rTempCoreLoading.CPUid < 0) {
			pr_info("CPUid(%d) < 0\n", rTempCoreLoading.CPUid);
			return -EFAULT;
		}
		/* tempory remark, must enable after function check-in */
		rTempCoreLoading.Loading = 0;
		/* get_cpu_load(rTempCoreLoading.CPUid); */
		ret = copy_to_user(user_data_addr, &rTempCoreLoading,
				sizeof(struct VAL_VCODEC_CORE_LOADING_T));
		if (ret) {
			pr_info("CORE_LOADING, copy_to_user failed: %lu\n",
					ret);
			return -EFAULT;
		}
		pr_debug("VCODEC_GET_CORE_LOADING - - tid = %d\n",
				current->pid);
	}
	break;

	case VCODEC_GET_CORE_NUMBER:
	{
		pr_debug("VCODEC_GET_CORE_NUMBER + - tid = %d\n",
				current->pid);

		user_data_addr = (unsigned char *)arg;
		temp_nr_cpu_ids = nr_cpu_ids;
		ret = copy_to_user(user_data_addr, &temp_nr_cpu_ids,
				sizeof(int));
		if (ret) {
			pr_info("[ERROR] VCODEC_GET_CORE_NUMBER, copy_to_user failed: %lu\n",
					ret);
			return -EFAULT;
		}
		pr_debug("VCODEC_GET_CORE_NUMBER - - tid = %d\n",
				current->pid);
	}
	break;

	case VCODEC_SET_CPU_OPP_LIMIT:
	{
		pr_info("VCODEC_SET_CPU_OPP_LIMIT [EMPTY] + - tid = %d\n",
				current->pid);
		user_data_addr = (unsigned char *) arg;
		ret = copy_from_user(&rCpuOppLimit, user_data_addr,
					   sizeof(
					   struct
					   VAL_VCODEC_CPU_OPP_LIMIT_T));
		if (ret) {
			pr_info("[ERROR] VCODEC_SET_CPU_OPP_LIMIT, copy_from_user failed: %lu\n",
					ret);
			return -EFAULT;
		}
		pr_info("+VCODEC_SET_CPU_OPP_LIMIT (%d, %d, %d), tid = %d\n",
				rCpuOppLimit.limited_freq,
				rCpuOppLimit.limited_cpu,
				rCpuOppLimit.enable,
				current->pid);
		/* TODO: Check if cpu_opp_limit is available */
		/* ret = cpu_opp_limit(EVENT_VIDEO,
		 * rCpuOppLimit.limited_freq,
		 * rCpuOppLimit.limited_cpu,
		 * rCpuOppLimit.enable);
		 * // 0: PASS, other: FAIL
		 */
		if (ret) {
			pr_info("[VCODEC][ERROR] cpu_opp_limit failed: %lu\n",
					ret);
			return -EFAULT;
		}
		pr_info("-VCODEC_SET_CPU_OPP_LIMIT tid = %d, ret = %lu\n",
				current->pid,
				ret);
		pr_info("VCODEC_SET_CPU_OPP_LIMIT [EMPTY] - - tid = %d\n",
				current->pid);
	}
	break;

	case VCODEC_MB:
	{
		/* For userspace to guarantee register setting order */
		mb();
	}
	break;

	default:
	{
		pr_info("[ERROR] vcodec_ioctl default case %u\n", cmd);
	}
	break;
	}
	return 0xFF;
}

#define vcodec_unlocked_compat_ioctl NULL

static int vcodec_open(struct inode *inode, struct file *file)
{
	pr_debug("vcodec_open\n");

	mutex_lock(&DriverOpenCountLock);
	Driver_Open_Count++;

	pr_info("vcodec_open pid = %d, Driver_Open_Count %d\n",
			current->pid, Driver_Open_Count);
	mutex_unlock(&DriverOpenCountLock);

	/* TODO: Check upper limit of concurrent users? */

	return 0;
}

static int vcodec_flush(struct file *file, fl_owner_t id)
{
	pr_debug("vcodec_flush, curr_tid =%d\n", current->pid);
	pr_debug("vcodec_flush pid = %d, Driver_Open_Count %d\n",
			current->pid, Driver_Open_Count);

	return 0;
}

static int vcodec_release(struct inode *inode, struct file *file)
{
	int i, j;
	unsigned long ulFlags, ulFlagsLockHW, ulFlagsISR, ulFlagsHWLock;

	/* dump_stack(); */
	pr_debug("vcodec_release, curr_tid =%d\n", current->pid);
	mutex_lock(&DriverOpenCountLock);
	pr_info("vcodec_release pid = %d, Driver_Open_Count %d\n",
			current->pid, Driver_Open_Count);
	Driver_Open_Count--;

	if (Driver_Open_Count == 0) {
		spin_lock_irqsave(&HWLock, ulFlagsHWLock);
		gu4VdecLockThreadId = 0;
		VcodecHWLock.pvHandle = 0;
		VcodecHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
		VcodecHWLock.rLockedTime.u4Sec = 0;
		VcodecHWLock.rLockedTime.u4uSec = 0;
		VcodecHWLock.u4ThreadID = 0;
		VcodecHWLock.u4VCodecThreadNum = VCODEC_THREAD_MAX_NUM;
		for (i = 0; i < VCODEC_THREAD_MAX_NUM; i++)
			VcodecHWLock.u4VCodecThreadID[i] = -1;

		spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);

		mutex_lock(&DecEMILock);
		gu4DecEMICounter = 0;
		mutex_unlock(&DecEMILock);

		mutex_lock(&EncEMILock);
		gu4EncEMICounter = 0;
		mutex_unlock(&EncEMILock);

		mutex_lock(&PWRLock);
		gu4PWRCounter = 0;
		mutex_unlock(&PWRLock);

		mutex_lock(&NonCacheMemoryListLock);
		for (i = 0; i < VCODEC_INST_NUM_x_10; i++) {
			ncache_mem_list[i].pvHandle = 0;
			for (j = 0; j < VCODEC_THREAD_MAX_NUM; j++)
				ncache_mem_list[i].u4VCodecThreadID[j] =
						0xffffffff;

			ncache_mem_list[i].ulKVA = -1L;
			ncache_mem_list[i].ulKPA = -1L;
		}
		mutex_unlock(&NonCacheMemoryListLock);

		spin_lock_irqsave(&OalHWContextLock, ulFlags);
		for (i = 0; i < VCODEC_INST_NUM; i++) {
			hw_ctx[i].Oal_HW_reg = (struct
					VAL_VCODEC_OAL_HW_REGISTER_T *)
					0;
			hw_ctx[i].ObjId = -1L;
			hw_ctx[i].slotindex = i;
			hw_ctx[i].u4VCodecThreadNum = VCODEC_THREAD_MAX_NUM;
			for (j = 0; j < VCODEC_THREAD_MAX_NUM; j++)
				hw_ctx[i].u4VCodecThreadID[j] = -1;

			hw_ctx[i].pvHandle = 0;
			hw_ctx[i].u4NumOfRegister = 0;

			for (j = 0; j < OALMEM_STATUS_NUM; j++) {
				hw_ctx[i].oalmem_status[j].u4ReadAddr = 0;
				hw_ctx[i].oalmem_status[j].u4ReadData = 0;
			}

			/* event part */
			hw_ctx[i].IsrEvent.pvHandle = "ISR_EVENT";
			hw_ctx[i].IsrEvent.u4HandleSize =
					sizeof("ISR_EVENT") + 1;
			/* 1000;*/
			hw_ctx[i].IsrEvent.u4TimeoutMs = 0xFFFFFFFF;
		}
		spin_unlock_irqrestore(&OalHWContextLock, ulFlags);

		spin_lock_irqsave(&LockDecHWCountLock, ulFlagsLockHW);
		gu4LockDecHWCount = 0;
		spin_unlock_irqrestore(&LockDecHWCountLock, ulFlagsLockHW);

		spin_lock_irqsave(&LockEncHWCountLock, ulFlagsLockHW);
		gu4LockEncHWCount = 0;
		spin_unlock_irqrestore(&LockEncHWCountLock, ulFlagsLockHW);

		spin_lock_irqsave(&DecISRCountLock, ulFlagsISR);
		gu4DecISRCount = 0;
		spin_unlock_irqrestore(&DecISRCountLock, ulFlagsISR);

		spin_lock_irqsave(&EncISRCountLock, ulFlagsISR);
		gu4EncISRCount = 0;
		spin_unlock_irqrestore(&EncISRCountLock, ulFlagsISR);

	}
	mutex_unlock(&DriverOpenCountLock);

	return 0;
}

void vcodec_vma_open(struct vm_area_struct *vma)
{
	pr_debug("vcodec VMA open, virt %lx, phys %lx\n", vma->vm_start,
			vma->vm_pgoff << PAGE_SHIFT);
}

void vcodec_vma_close(struct vm_area_struct *vma)
{
	pr_debug("vcodec VMA close, virt %lx, phys %lx\n", vma->vm_start,
			vma->vm_pgoff << PAGE_SHIFT);
}

static const struct vm_operations_struct vcodec_remap_vm_ops = {
	.open = vcodec_vma_open,
	.close = vcodec_vma_close,
};

static int vcodec_mmap(struct file *file, struct vm_area_struct *vma)
{
#if 1
	unsigned int u4I = 0;
	unsigned long length;
	unsigned long pfn;

	length = vma->vm_end - vma->vm_start;
	pfn = vma->vm_pgoff<<PAGE_SHIFT;

	if (((length > VENC_REGION) || (pfn < VENC_BASE) ||
		(pfn > VENC_BASE+VENC_REGION)) &&
		((length > VDEC_REGION) || (pfn < VDEC_BASE_PHY) ||
			(pfn > VDEC_BASE_PHY+VDEC_REGION)) &&
		((length > HW_REGION) || (pfn < HW_BASE) ||
			(pfn > HW_BASE+HW_REGION)) &&
		((length > INFO_REGION) || (pfn < INFO_BASE) ||
			(pfn > INFO_BASE + INFO_REGION)) &&
	    ((length > MP4_VENC_REGION) || (pfn < MP4_VENC_BASE) ||
			(pfn > MP4_VENC_BASE + MP4_VENC_REGION))) {
		unsigned long ulAddr, ulSize;

		for (u4I = 0; u4I < VCODEC_INST_NUM_x_10; u4I++) {
			if ((ncache_mem_list[u4I].ulKVA != -1L) &&
				(ncache_mem_list[u4I].ulKPA != -1L)) {
				ulAddr = ncache_mem_list[u4I].ulKPA;
				ulSize = (ncache_mem_list[u4I].ulSize +
						0x1000 - 1) & ~(0x1000 - 1);
				if ((length == ulSize) && (pfn == ulAddr)) {
					pr_debug("[VCODEC] cache idx %d\n",
							u4I);
					break;
				}
			}
		}

		if (u4I == VCODEC_INST_NUM_x_10) {
			pr_info("mmap region error: Len(0x%lx),pfn(0x%lx)",
				 (unsigned long)length, pfn);
			return -EAGAIN;
		}
	}
#endif
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	pr_debug("mmap vma->start 0x%lx, vma->end 0x%lx, vma->pgoff 0x%lx\n",
			(unsigned long)vma->vm_start,
			(unsigned long)vma->vm_end,
			(unsigned long)vma->vm_pgoff);
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	vma->vm_ops = &vcodec_remap_vm_ops;
	vcodec_vma_open(vma);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void vcodec_early_suspend(struct early_suspend *h)
{
	mutex_lock(&PWRLock);
	pr_info("vcodec_early_suspend, tid = %d, PWR_USER = %d\n",
			current->pid,
			gu4PWRCounter);
	mutex_unlock(&PWRLock);
	pr_debug("vcodec_early_suspend - tid = %d\n", current->pid);
}

static void vcodec_late_resume(struct early_suspend *h)
{
	mutex_lock(&PWRLock);
	pr_info("vcodec_late_resume, tid = %d, PWR_USER = %d\n",
			current->pid,
			gu4PWRCounter);
	mutex_unlock(&PWRLock);
	pr_debug("vcodec_late_resume - tid = %d\n", current->pid);
}

static struct early_suspend vcodec_early_suspend_handler = {
	.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1),
	.suspend = vcodec_early_suspend,
	.resume = vcodec_late_resume,
};
#endif

static const struct file_operations vcodec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vcodec_unlocked_ioctl,
	.open = vcodec_open,
	.flush = vcodec_flush,
	.release = vcodec_release,
	.mmap = vcodec_mmap,
};

static int vcodec_probe(struct platform_device *dev)
{
	int ret;

	pr_debug("+vcodec_probe\n");

	mutex_lock(&DecEMILock);
	gu4DecEMICounter = 0;
	mutex_unlock(&DecEMILock);

	mutex_lock(&EncEMILock);
	gu4EncEMICounter = 0;
	mutex_unlock(&EncEMILock);

	mutex_lock(&PWRLock);
	gu4PWRCounter = 0;
	mutex_unlock(&PWRLock);

	mutex_lock(&L2CLock);
	gu4L2CCounter = 0;
	mutex_unlock(&L2CLock);

	ret = register_chrdev_region(vcodec_devno, 1, VCODEC_DEVNAME);
	if (ret)
		pr_info("[VCODEC] Can't Get Major number\n");


	vcodec_cdev = cdev_alloc();
	vcodec_cdev->owner = THIS_MODULE;
	vcodec_cdev->ops = &vcodec_fops;

	ret = cdev_add(vcodec_cdev, vcodec_devno, 1);
	if (ret)
		pr_info("[VCODEC] Can't add Vcodec Device\n");


	vcodec_class = class_create(THIS_MODULE, VCODEC_DEVNAME);
	if (IS_ERR(vcodec_class)) {
		ret = PTR_ERR(vcodec_class);
		pr_info("[VCODEC] Unable to create class err = %d\n",
				ret);
		return ret;
	}

	vcodec_device = device_create(vcodec_class, NULL, vcodec_devno, NULL,
					VCODEC_DEVNAME);

	if (request_irq(VDEC_IRQ_ID, (irq_handler_t)video_intr_dlr,
			IRQF_TRIGGER_LOW, VCODEC_DEVNAME, NULL) < 0) {
		pr_info("[VCODEC][ERROR] error to request dec irq\n");
	} else {
		pr_debug("[VCODEC] success to request dec irq: %d\n",
				VDEC_IRQ_ID);
	}

	if (request_irq(VENC_IRQ_ID, (irq_handler_t)video_intr_dlr2,
			IRQF_TRIGGER_LOW, VCODEC_DEVNAME, NULL) < 0) {
		pr_debug("[VCODEC][ERROR] error to request enc irq\n");
	} else {
		pr_debug("[VCODEC] success to request enc irq: %d\n",
				VENC_IRQ_ID);
	}

	disable_irq(VDEC_IRQ_ID);
	disable_irq(VENC_IRQ_ID);

	vcodec_device->coherent_dma_mask = DMA_BIT_MASK(32);
	if (!vcodec_device->dma_mask)
		vcodec_device->dma_mask = &vcodec_device->coherent_dma_mask;

	pr_debug("vcodec_probe Done\n");

	return 0;
}

#ifdef CONFIG_MTK_HIBERNATION
static int vcodec_pm_restore_noirq(struct device *device)
{
	/* vdec: IRQF_TRIGGER_LOW */
	mt_irq_set_sens(VDEC_IRQ_ID, MT_LEVEL_SENSITIVE);
	mt_irq_set_polarity(VDEC_IRQ_ID, MT_POLARITY_LOW);
	/* venc: IRQF_TRIGGER_LOW */
	mt_irq_set_sens(VENC_IRQ_ID, MT_LEVEL_SENSITIVE);
	mt_irq_set_polarity(VENC_IRQ_ID, MT_POLARITY_LOW);

	return 0;
}
#endif

static int __init vcodec_driver_init(void)
{
	int i, j;
	enum VAL_RESULT_T eValHWLockRet, eValRet;
	unsigned long ulFlags, ulFlagsLockHW, ulFlagsISR, ulFlagsHWLock;
	unsigned long ulFlagsVdecPWRLock, ulFlagsVencPWRLock;

	pr_debug("+vcodec_driver_init !!\n");

	mutex_lock(&DriverOpenCountLock);
	Driver_Open_Count = 0;
	mutex_unlock(&DriverOpenCountLock);

	{
#ifdef CONFIG_MACH_MT6580
		struct device_node *node = NULL;

		node = of_find_compatible_node(NULL, NULL, "mediatek,VENC");
		if (node) {
			KVA_VENC_BASE = (unsigned long) of_iomap(node, 0);
			VENC_IRQ_ID = irq_of_parse_and_map(node, 0);
		} else {
			pr_debug("[VCODEC][DeviceTree] Not supported, use hard-code value");
			KVA_VENC_BASE = (unsigned long)
						ioremap(0x14016000, 0x800);
			/* VENC_IRQ_ID = MT_VENC_IRQ_ID; */
		}
		KVA_VENC_IRQ_STATUS_ADDR = KVA_VENC_BASE + 0x67C;
		KVA_VENC_IRQ_ACK_ADDR = KVA_VENC_BASE + 0x678;
		KVA_VENC_ZERO_COEF_COUNT_ADDR = KVA_VENC_BASE + 0x688;
		KVA_VENC_BYTE_COUNT_ADDR = KVA_VENC_BASE + 0x680;
		KVA_VENC_MP4_IRQ_ENABLE_ADDR = KVA_VENC_BASE + 0x668;
#else
		struct device_node *node = NULL;

		node = of_find_compatible_node(NULL, NULL, "mediatek,VENC");
		KVA_VENC_BASE = (unsigned long) of_iomap(node, 0);
		VENC_IRQ_ID = irq_of_parse_and_map(node, 0);
		KVA_VENC_IRQ_STATUS_ADDR = KVA_VENC_BASE + 0x05C;
		KVA_VENC_IRQ_ACK_ADDR = KVA_VENC_BASE + 0x060;
#endif
	}
#ifdef CONFIG_MACH_MT6580
	{
		struct device_node *node = NULL;

		node = of_find_compatible_node(NULL, NULL, "mediatek,VDEC");
		if (node) {
			KVA_VDEC_BASE = (unsigned long) of_iomap(node, 0);
			VDEC_IRQ_ID = irq_of_parse_and_map(node, 0);
		} else {
			pr_info("[VCODEC][DeviceTree] VDEC Not supported, use hard-code value");
			KVA_VDEC_BASE = (unsigned long)
					ioremap(VDEC_BASE_PHY, VDEC_REGION);
			VDEC_IRQ_ID = 137;
			/* 0x6c; //mt6580 Hybrid VDEC interrupt ID is 0x6c */
		}
		KVA_VDEC_MISC_BASE = KVA_VDEC_BASE + 0x0000;
		KVA_VDEC_VLD_BASE = KVA_VDEC_BASE + 0x1000;
		/* VDEC interrupt status */
		KVA_VDEC_INT_STA_ADDR = KVA_VDEC_BASE + 0x10;
		/* VDEC interrupt ACKnowledge */
		KVA_VDEC_INT_ACK_ADDR = KVA_VDEC_BASE + 0xc;
	}

	{
		pr_info("[VCODEC][DeviceTree] KVA_VENC_BASE(0x%lx), KVA_VDEC_BASE(0x%lx), KVA_VDEC_GCON_BASE(0x%lx)",
		     KVA_VENC_BASE, KVA_VDEC_BASE, KVA_VDEC_GCON_BASE);
		pr_debug("[VCODEC][DeviceTree] VDEC_IRQ_ID(%d), VENC_IRQ_ID(%d)",
			VDEC_IRQ_ID, VENC_IRQ_ID);
	}
#else
	{
		struct device_node *node = NULL;

		node = of_find_compatible_node(NULL, NULL,
					"mediatek,VDEC_FULL_TOP");
		KVA_VDEC_BASE = (unsigned long)of_iomap(node, 0);
		VDEC_IRQ_ID = irq_of_parse_and_map(node, 0);
		KVA_VDEC_MISC_BASE = KVA_VDEC_BASE + 0x0000;
		KVA_VDEC_VLD_BASE = KVA_VDEC_BASE + 0x1000;
	}
	{
		struct device_node *node = NULL;

		node = of_find_compatible_node(NULL, NULL,
					"mediatek,VDEC_GCON");
		KVA_VDEC_GCON_BASE = (unsigned long) of_iomap(node, 0);

		pr_debug("[VCODEC] VENC(0x%lx), VDEC(0x%lx), VDEC_GCON(0x%lx)",
			KVA_VENC_BASE, KVA_VDEC_BASE, KVA_VDEC_GCON_BASE);
		pr_debug("[VCODEC] VDEC_IRQ_ID(%d), VENC_IRQ_ID(%d)",
			VDEC_IRQ_ID, VENC_IRQ_ID);
	}
#endif

	gu4VDecIRQCount = 0;
	gu4VEncIRQCount = 0;

	spin_lock_irqsave(&LockDecHWCountLock, ulFlagsLockHW);
	gu4LockDecHWCount = 0;
	spin_unlock_irqrestore(&LockDecHWCountLock, ulFlagsLockHW);

	spin_lock_irqsave(&LockEncHWCountLock, ulFlagsLockHW);
	gu4LockEncHWCount = 0;
	spin_unlock_irqrestore(&LockEncHWCountLock, ulFlagsLockHW);

	spin_lock_irqsave(&DecISRCountLock, ulFlagsISR);
	gu4DecISRCount = 0;
	spin_unlock_irqrestore(&DecISRCountLock, ulFlagsISR);

	spin_lock_irqsave(&EncISRCountLock, ulFlagsISR);
	gu4EncISRCount = 0;
	spin_unlock_irqrestore(&EncISRCountLock, ulFlagsISR);

	spin_lock_irqsave(&VdecPWRLock, ulFlagsVdecPWRLock);
	gu4VdecPWRCounter = 0;
	spin_unlock_irqrestore(&VdecPWRLock, ulFlagsVdecPWRLock);

	spin_lock_irqsave(&VencPWRLock, ulFlagsVencPWRLock);
	gu4VencPWRCounter = 0;
	spin_unlock_irqrestore(&VencPWRLock, ulFlagsVencPWRLock);

	mutex_lock(&IsOpenedLock);
	if (bIsOpened == VAL_FALSE) {
		bIsOpened = VAL_TRUE;
		vcodec_probe(NULL);
	}
	mutex_unlock(&IsOpenedLock);

	spin_lock_irqsave(&HWLock, ulFlagsHWLock);
	gu4VdecLockThreadId = 0;
	VcodecHWLock.pvHandle = 0;
	VcodecHWLock.eDriverType = VAL_DRIVER_TYPE_NONE;
	VcodecHWLock.rLockedTime.u4Sec = 0;
	VcodecHWLock.rLockedTime.u4uSec = 0;
	VcodecHWLock.u4ThreadID = 0;
	VcodecHWLock.u4VCodecThreadNum = VCODEC_THREAD_MAX_NUM;
	for (i = 0; i < VCODEC_THREAD_MAX_NUM; i++)
		VcodecHWLock.u4VCodecThreadID[i] = -1;

	spin_unlock_irqrestore(&HWLock, ulFlagsHWLock);

	mutex_lock(&NonCacheMemoryListLock);
	for (i = 0; i < VCODEC_INST_NUM_x_10; i++) {
		ncache_mem_list[i].pvHandle = 0x0;
		for (j = 0; j < VCODEC_THREAD_MAX_NUM; j++)
			ncache_mem_list[i].u4VCodecThreadID[j] = 0xffffffff;

		ncache_mem_list[i].ulKVA = -1L;
		ncache_mem_list[i].ulKPA = -1L;
	}
	mutex_unlock(&NonCacheMemoryListLock);

	for (i = 0; i < VCODEC_INST_NUM; i++) {
		spin_lock_irqsave(&OalHWContextLock, ulFlags);
		hw_ctx[i].Oal_HW_reg = (struct
				VAL_VCODEC_OAL_HW_REGISTER_T *)
				0;
		hw_ctx[i].ObjId = -1L;
		hw_ctx[i].slotindex = i;
		hw_ctx[i].u4VCodecThreadNum = VCODEC_THREAD_MAX_NUM;
		for (j = 0; j < VCODEC_THREAD_MAX_NUM; j++)
			hw_ctx[i].u4VCodecThreadID[j] = -1;

		hw_ctx[i].pvHandle = 0;
		hw_ctx[i].u4NumOfRegister = 0;

		for (j = 0; j < OALMEM_STATUS_NUM; j++) {
			hw_ctx[i].oalmem_status[j].u4ReadAddr = 0;
			hw_ctx[i].oalmem_status[j].u4ReadData = 0;
		}

		/* event part */
		hw_ctx[i].IsrEvent.pvHandle = "ISR_EVENT";
		hw_ctx[i].IsrEvent.u4HandleSize = sizeof("ISR_EVENT") + 1;
		hw_ctx[i].IsrEvent.u4TimeoutMs = 0xFFFFFFFF;	/* 1000; */
		spin_unlock_irqrestore(&OalHWContextLock, ulFlags);
		eValRet = eVideoCreateEvent(&(hw_ctx[i].IsrEvent),
					sizeof(struct VAL_EVENT_T));
		if (eValRet != VAL_RESULT_NO_ERROR)
			pr_info("[MFV][ERROR] create isr event error\n");

	}

	/* HWLockEvent part */
	mutex_lock(&HWLockEventTimeoutLock);
	HWLockEvent.pvHandle = "HWLOCK_EVENT";
	HWLockEvent.u4HandleSize = sizeof("HWLOCK_EVENT") + 1;
	HWLockEvent.u4TimeoutMs = 1;
	mutex_unlock(&HWLockEventTimeoutLock);
	eValHWLockRet = eVideoCreateEvent(&HWLockEvent,
					sizeof(struct VAL_EVENT_T));
	if (eValHWLockRet != VAL_RESULT_NO_ERROR)
		pr_info("[VCODEC][ERROR] create hwlock event error\n");

	/* IsrEvent part */
	spin_lock_irqsave(&DecIsrLock, ulFlags);
	DecIsrEvent.pvHandle = "DECISR_EVENT";
	DecIsrEvent.u4HandleSize = sizeof("DECISR_EVENT") + 1;
	DecIsrEvent.u4TimeoutMs = 1;
	spin_unlock_irqrestore(&DecIsrLock, ulFlags);
	eValHWLockRet = eVideoCreateEvent(&DecIsrEvent,
					sizeof(struct VAL_EVENT_T));
	if (eValHWLockRet != VAL_RESULT_NO_ERROR)
		pr_info("[VCODEC][ERROR] create dec isr event error\n");

	spin_lock_irqsave(&EncIsrLock, ulFlags);
	EncIsrEvent.pvHandle = "ENCISR_EVENT";
	EncIsrEvent.u4HandleSize = sizeof("ENCISR_EVENT") + 1;
	EncIsrEvent.u4TimeoutMs = 1;
	spin_unlock_irqrestore(&EncIsrLock, ulFlags);
	eValHWLockRet = eVideoCreateEvent(&EncIsrEvent,
					sizeof(struct VAL_EVENT_T));
	if (eValHWLockRet != VAL_RESULT_NO_ERROR)
		pr_info("[VCODEC][ERROR] create enc isr event error\n");


	m4u_register_fault_callback(M4U_PORT_VENC_CDMA_VDEC_CDMA,
			vcodec_m4u_fault_callback, (void *)&VcodecHWLock);
	m4u_register_fault_callback(M4U_PORT_VENC_BSDMA_VDEC_POST0,
			vcodec_m4u_fault_callback, (void *)&VcodecHWLock);
	pr_debug("vcodec_driver_init Done\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&vcodec_early_suspend_handler);
#endif

#ifdef CONFIG_MTK_HIBERNATION
	register_swsusp_restore_noirq_func(ID_M_VCODEC,
					vcodec_pm_restore_noirq, NULL);
#endif

	return 0;
}

static void __exit vcodec_driver_exit(void)
{
	int i;
	enum VAL_RESULT_T eValRet;
	enum VAL_RESULT_T eValHWLockRet;

	pr_debug("vcodec_driver_exit\n");

	mutex_lock(&IsOpenedLock);
	if (bIsOpened == VAL_TRUE)
		bIsOpened = VAL_FALSE;

	mutex_unlock(&IsOpenedLock);

	cdev_del(vcodec_cdev);
	unregister_chrdev_region(vcodec_devno, 1);

	/* [TODO] iounmap the following? */
#if 0
	iounmap((void *)KVA_VENC_IRQ_STATUS_ADDR);
	iounmap((void *)KVA_VENC_IRQ_ACK_ADDR);
#endif

	free_irq(VENC_IRQ_ID, NULL);
	free_irq(VDEC_IRQ_ID, NULL);

	for (i = 0; i < OALMEM_STATUS_NUM; i++) {
		eValRet = eVideoCloseEvent(&(hw_ctx[i].IsrEvent),
					sizeof(struct VAL_EVENT_T));
		if (eValRet != VAL_RESULT_NO_ERROR)
			pr_info("[VCODEC][ERROR] close isr event error\n");
	}

	eValHWLockRet = eVideoCloseEvent(&HWLockEvent,
					sizeof(struct VAL_EVENT_T));
	if (eValHWLockRet != VAL_RESULT_NO_ERROR)
		pr_info("[VCODEC][ERROR] close hwlock event error\n");

	eValHWLockRet = eVideoCloseEvent(&DecIsrEvent,
					sizeof(struct VAL_EVENT_T));
	if (eValHWLockRet != VAL_RESULT_NO_ERROR)
		pr_info("[VCODEC][ERROR] close dec isr event error\n");

	eValHWLockRet = eVideoCloseEvent(&EncIsrEvent,
					sizeof(struct VAL_EVENT_T));
	if (eValHWLockRet != VAL_RESULT_NO_ERROR)
		pr_info("[VCODEC][ERROR] close enc isr event error\n");

	m4u_unregister_fault_callback(M4U_PORT_VENC_CDMA_VDEC_CDMA);
	m4u_unregister_fault_callback(M4U_PORT_VENC_BSDMA_VDEC_POST0);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&vcodec_early_suspend_handler);
#endif

#ifdef CONFIG_MTK_HIBERNATION
	unregister_swsusp_restore_noirq_func(ID_M_VCODEC);
#endif
}
module_init(vcodec_driver_init);
module_exit(vcodec_driver_exit);
MODULE_AUTHOR("Legis, Lu <legis.lu@mediatek.com>");
MODULE_DESCRIPTION("Rainier Vcodec Driver");
MODULE_LICENSE("GPL");

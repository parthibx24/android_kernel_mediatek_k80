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

#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include "val_types_private.h"
#include "drv_api.h"

/* ============================================================== */
/* For Hybrid HW */
/* spinlock : OalHWContextLock */
struct VAL_VCODEC_OAL_HW_CONTEXT_T hw_ctx[VCODEC_INST_NUM];
/* mutex : NonCacheMemoryListLock */
struct VAL_NON_CACHE_MEMORY_LIST_T ncache_mem_list[VCODEC_INST_NUM_x_10];
/* For both hybrid and pure HW */
/* struct VAL_VCODEC_HW_LOCK_T DecHWLock; //mutex : VdecHWLock */
/* struct VAL_VCODEC_HW_LOCK_T EncHWLock; //mutex : VencHWLock */
struct VAL_VCODEC_HW_LOCK_T VcodecHWLock;	/* mutex : HWLock */

unsigned int gu4LockDecHWCount;	/* spinlock : LockDecHWCountLock */
unsigned int gu4LockEncHWCount;	/* spinlock : LockEncHWCountLock */
unsigned int gu4DecISRCount;	/* spinlock : DecISRCountLock */
unsigned int gu4EncISRCount;	/* spinlock : EncISRCountLock */

int gu4VDecIRQCount;
int gu4VEncIRQCount;
/*
 * Search HWLockSlot by TID
 */
int search_slot_byTID(unsigned long ulpa, unsigned int curr_tid)
{
	int i;
	int j;

	for (i = 0; i < VCODEC_INST_NUM; i++) {
		if (hw_ctx[i].u4VCodecThreadNum != VCODEC_THREAD_MAX_NUM) {
			for (j = 0; j < hw_ctx[i].u4VCodecThreadNum; j++) {
				if (hw_ctx[i].u4VCodecThreadID[j] == curr_tid) {
					pr_debug("HWLocker id %d idx %d",
					curr_tid, i);
					return i;
				}
			}
		}
	}

	return -1;
}


/*
 * Search HWLockSlot by handle
 */
int search_slot_byHdl(unsigned long ulpa, unsigned long handle)
{
	int i;

	if (handle == (unsigned long)NULL) {
		pr_info("Get NULL Handle\n");
		return -1;
	}
	for (i = 0; i < VCODEC_INST_NUM; i++) {
		if (hw_ctx[i].pvHandle == handle)
			return i;
	}

	return -1;
}

/*
 * Set current HWLockSlot
 */
struct VAL_VCODEC_OAL_HW_CONTEXT_T *set_slot(unsigned long ulpa,
						unsigned int tid)
{

	int i, j;

	/* Dump current ObjId */
	for (i = 0; i < VCODEC_INST_NUM; i++)
		pr_debug("[VCODEC] Dump curr slot %d ObjId 0x%lx\n",
				i, hw_ctx[i].ObjId);

	/* check if current ObjId exist in hw_ctx[i].ObjId */
	for (i = 0; i < VCODEC_INST_NUM; i++) {
		if (hw_ctx[i].ObjId == ulpa) {
			pr_debug("[VCODEC] Curr exists in %d Slot", i);
			return &hw_ctx[i];
		}
	}

	/* if not exist in table,  find a new free slot and put it */
	for (i = 0; i < VCODEC_INST_NUM; i++) {
		if (hw_ctx[i].u4VCodecThreadNum != VCODEC_THREAD_MAX_NUM) {
			for (j = 0; j < hw_ctx[i].u4VCodecThreadNum; j++) {
				if (hw_ctx[i].u4VCodecThreadID[j] ==
					current->pid) {
					hw_ctx[i].ObjId = ulpa;
					pr_debug("[VCODEC] Set slot %d",
							i);
					return &hw_ctx[i];
				}
			}
		}
	}

	pr_warn("[VCODEC] set_slot All %d Slots unavaliable\n",
			VCODEC_INST_NUM);
	return NULL;
}


struct VAL_VCODEC_OAL_HW_CONTEXT_T
	*set_slot_TID(struct VAL_VCODEC_THREAD_ID_T a_prVcodecThreadID,
	unsigned int *a_prIndex)
{
	int i;
	int j;
	int k;

	if (a_prVcodecThreadID.u4VCodecThreadNum >= VCODEC_THREAD_MAX_NUM) {
		pr_info("set_slot_TID TNum %u > Max %u",
			a_prVcodecThreadID.u4VCodecThreadNum,
			VCODEC_THREAD_MAX_NUM);
		*a_prIndex = 0xff;
		return NULL;
	}

	for (i = 0; i < a_prVcodecThreadID.u4VCodecThreadNum; i++) {
		pr_debug("set_slot_TID TNum = %d, TID = %d\n",
				a_prVcodecThreadID.u4VCodecThreadNum,
				a_prVcodecThreadID.u4VCodecThreadID[i]);
	}

	/* check if current tids exist in hw_ctx[i].ObjId */
	for (i = 0; i < VCODEC_INST_NUM; i++) {
		if (hw_ctx[i].u4VCodecThreadNum !=
			VCODEC_THREAD_MAX_NUM) {
			for (j = 0; j < hw_ctx[i].u4VCodecThreadNum; j++) {
				for (k = 0;
				k < a_prVcodecThreadID.u4VCodecThreadNum;
				k++) {
				if (hw_ctx[i].u4VCodecThreadID[j] ==
				a_prVcodecThreadID.u4VCodecThreadID[k]) {
					pr_warn("already exists in %d slot",
							i);
					*a_prIndex = i;
					return &hw_ctx[i];
				}
				}
			}
		}
	}

	/* if not exist in table,  find a new free slot and put it */
	for (i = 0; i < VCODEC_INST_NUM; i++) {
		if (hw_ctx[i].u4VCodecThreadNum == VCODEC_THREAD_MAX_NUM) {
			hw_ctx[i].u4VCodecThreadNum =
					a_prVcodecThreadID.u4VCodecThreadNum;
			for (j = 0; j < a_prVcodecThreadID.u4VCodecThreadNum;
				j++) {
				hw_ctx[i].u4VCodecThreadID[j] =
				    a_prVcodecThreadID.u4VCodecThreadID[j];
				pr_debug("set_slot_TID %d Slot, %d\n",
				i, hw_ctx[i].u4VCodecThreadID[j]);
			}
			*a_prIndex = i;
			return &hw_ctx[i];
		}
	}

	{
		pr_warn("set_slot_TID All %d Slots unavaliable\n",
			 VCODEC_INST_NUM);
		/* Dump current tids */
		for (i = 0; i < VCODEC_INST_NUM; i++) {
			if (hw_ctx[i].u4VCodecThreadNum !=
				VCODEC_THREAD_MAX_NUM) {
				for (j = 0;
					j < hw_ctx[i].u4VCodecThreadNum;
					j++) {
					pr_debug("set_slot_TID Dump curr slot %d, TID[%d] = %d\n",
					     i, j,
						 hw_ctx[i].u4VCodecThreadID[j]);
				}
			}
		}

		hw_ctx[0].u4VCodecThreadNum =
				a_prVcodecThreadID.u4VCodecThreadNum;
		for (i = 0; i < hw_ctx[0].u4VCodecThreadNum; i++) {
			hw_ctx[0].u4VCodecThreadID[i] =
			    a_prVcodecThreadID.u4VCodecThreadID[i];
		}
		*a_prIndex = 0;
		return &hw_ctx[0];
	}
}


/*
 * free HWLockSlot
 */
struct VAL_VCODEC_OAL_HW_CONTEXT_T *free_slot(unsigned long ulpa)
{
	int i;
	int j;

	/* check if current ObjId exist in hw_ctx[i].ObjId */

	for (i = 0; i < VCODEC_INST_NUM; i++) {
		if (hw_ctx[i].ObjId == ulpa) {
			hw_ctx[i].ObjId = -1L;
			for (j = 0; j < hw_ctx[i].u4VCodecThreadNum; j++)
				hw_ctx[i].u4VCodecThreadID[j] = -1;
			hw_ctx[i].u4VCodecThreadNum = VCODEC_THREAD_MAX_NUM;
			hw_ctx[i].Oal_HW_reg =
				(struct VAL_VCODEC_OAL_HW_REGISTER_T *)0;
			pr_debug("[VCODEC] free_slot %d Slot", i);
			return &hw_ctx[i];
		}
	}

	pr_warn("[VCODEC][ERROR] free_slot can't find pid in HWLockSlot\n");
	return 0;
}


/*
 * Add non cache memory to list
 */
void add_ncmem(unsigned long a_ulKVA,
				unsigned long a_ulKPA,
				unsigned long a_ulSize,
				unsigned int a_u4VCodecThreadNum,
				unsigned int *a_pu4VCodecThreadID)
{
	unsigned int u4I = 0;
	unsigned int u4J = 0;

	pr_debug("add_ncmem +, KVA = 0x%lx, KPA = 0x%lx, Size = 0x%lx\n",
			a_ulKVA, a_ulKPA, a_ulSize);

	for (u4I = 0; u4I < VCODEC_INST_NUM_x_10; u4I++) {
		if ((ncache_mem_list[u4I].ulKVA == -1L)
		    && (ncache_mem_list[u4I].ulKPA == -1L)) {
			pr_debug("add_ncmem idx=%d, TNum=%d, tid=%d",
				u4I, a_u4VCodecThreadNum, current->pid);

			ncache_mem_list[u4I].u4VCodecThreadNum =
							a_u4VCodecThreadNum;
			for (u4J = 0;
			u4J < ncache_mem_list[u4I].u4VCodecThreadNum;
			u4J++) {
				ncache_mem_list[u4I].u4VCodecThreadID[u4J]
				= *(a_pu4VCodecThreadID + u4J);
				pr_debug("add_ncmem TNum = %d, TID = %d",
				ncache_mem_list[u4I].u4VCodecThreadNum,
				ncache_mem_list[u4I].u4VCodecThreadID[u4J]);
			}

			ncache_mem_list[u4I].ulKVA = a_ulKVA;
			ncache_mem_list[u4I].ulKPA = a_ulKPA;
			ncache_mem_list[u4I].ulSize = a_ulSize;
			break;
		}
	}

	if (u4I == VCODEC_INST_NUM_x_10)
		pr_warn("CAN'T ADD add_ncmem, List is FULL!!\n");

	pr_debug("add_ncmem -\n");
}

void free_ncmem(unsigned long a_ulKVA, unsigned long a_ulKPA)
{
	unsigned int u4I = 0;
	unsigned int u4J = 0;

	pr_debug("free_ncmem +, KVA = 0x%lx, KPA = 0x%lx\n",
			a_ulKVA, a_ulKPA);

	for (u4I = 0; u4I < VCODEC_INST_NUM_x_10; u4I++) {
		if ((ncache_mem_list[u4I].ulKVA == a_ulKVA)
		    && (ncache_mem_list[u4I].ulKPA == a_ulKPA)) {
			pr_debug("free_ncmem index = %d\n", u4I);
			ncache_mem_list[u4I].u4VCodecThreadNum =
							VCODEC_THREAD_MAX_NUM;
			for (u4J = 0; u4J < VCODEC_THREAD_MAX_NUM; u4J++)
				ncache_mem_list[u4I].u4VCodecThreadID[u4J] =
								0xffffffff;

			ncache_mem_list[u4I].ulKVA = -1L;
			ncache_mem_list[u4I].ulKPA = -1L;
			ncache_mem_list[u4I].ulSize = -1L;
			break;
		}
	}

	if (u4I == VCODEC_INST_NUM_x_10)
		pr_warn("CAN'T Free free_ncmem, Address is not find!!\n");

	pr_debug("free_ncmem -\n");
}


#define FFREE_LOG "idx=\%d,tid=\%d,KVA=0x\%lx,KPA=0x\%lx,Size=\%lu"
/*
 * Force free non cache memory of a tid
 */
void ffree_ncmem(unsigned int a_u4Tid)
{
	unsigned int u4I = 0;
	unsigned int u4J = 0;
	unsigned int u4K = 0;

	pr_debug("ffree_ncmem +, curr_id = %d", a_u4Tid);

	for (u4I = 0; u4I < VCODEC_INST_NUM_x_10; u4I++) {
		if (ncache_mem_list[u4I].u4VCodecThreadNum !=
			VCODEC_THREAD_MAX_NUM) {
			for (u4J = 0;
			u4J < ncache_mem_list[u4I].u4VCodecThreadNum;
			u4J++) {
				if (ncache_mem_list[u4I].
					u4VCodecThreadID[u4J] == a_u4Tid) {
					pr_debug(FFREE_LOG,
						u4I, a_u4Tid,
						ncache_mem_list[u4I].ulKVA,
						ncache_mem_list[u4I].ulKPA,
						ncache_mem_list[u4I].ulSize);
				dma_free_coherent(0,
				ncache_mem_list[u4I].ulSize,
				(void *)ncache_mem_list[u4I].ulKVA,
				(dma_addr_t) ncache_mem_list[u4I].ulKPA);

				ncache_mem_list[u4I].u4VCodecThreadNum =
							VCODEC_THREAD_MAX_NUM;
					for (u4K = 0;
						u4K < VCODEC_THREAD_MAX_NUM;
						u4K++) {
						ncache_mem_list[u4I].
							u4VCodecThreadID[u4K] =
						    0xffffffff;
					}
					ncache_mem_list[u4I].ulKVA = -1L;
					ncache_mem_list[u4I].ulKPA = -1L;
					ncache_mem_list[u4I].ulSize = -1L;
					break;
				}
			}
		}
	}

	pr_debug("ffree_ncmem -, curr_id = %d", a_u4Tid);
}


/*
 * Search non cache memory by KPA
 */
unsigned long search_ncmem_byKPA(unsigned long a_ulKPA)
{
	unsigned int u4I = 0;
	unsigned long ulVA_Offset = 0;

	ulVA_Offset = a_ulKPA & 0x0000000000000fff;

	pr_debug("search_ncmem_byKPA +, KPA=0x%lx, ulVA_Offset = 0x%lx",
			a_ulKPA, ulVA_Offset);

	for (u4I = 0; u4I < VCODEC_INST_NUM_x_10; u4I++) {
		if (ncache_mem_list[u4I].ulKPA ==
			(a_ulKPA - ulVA_Offset)) {
			pr_debug("search_ncmem_byKPA index = %d\n",
					u4I);
			break;
		}
	}

	if (u4I == VCODEC_INST_NUM_x_10) {
		pr_warn("CAN'T Find address search_ncmem_byKPA");
		return 0;
	}

	pr_debug("[VCODEC] search_ncmem_byKPA, ulVA = 0x%lx -\n",
			(ncache_mem_list[u4I].ulKVA + ulVA_Offset));

	return ncache_mem_list[u4I].ulKVA + ulVA_Offset;
}

#define M4UCB_LOG "m4u cb, port=%d, data=0x%lx, handle=0x%lx, type=%d, tid=%d"

m4u_callback_ret_t
	vcodec_m4u_fault_callback(int port,
				unsigned int mva, void *data)
{
	struct VAL_VCODEC_HW_LOCK_T *pLock =
				(struct VAL_VCODEC_HW_LOCK_T *) data;

	if (pLock != NULL) {
		/* if pLock not NULL, print info */
		pr_info(M4UCB_LOG,
		       port, (unsigned long)data,
			   (unsigned long)pLock->pvHandle,
		       pLock->eDriverType, pLock->u4ThreadID);
	} else {
		/* if pLock NULL, error */
		pr_info("m4u cb, port=%d, No Data (0x%lx)!\n",
				port, (unsigned long)data);
	}
	return M4U_CALLBACK_HANDLED;
}

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

#ifndef __VCODEC_DRIVER_H__
#define __VCODEC_DRIVER_H__

#define MFV_IOC_MAGIC    'M'

/* below is control message */
/* HAL_POWER_T * */
#define VCODEC_WAITISR			_IOW(MFV_IOC_MAGIC, 0x0b, unsigned int)
/* VAL_HW_LOCK_T * */
#define VCODEC_LOCKHW			_IOW(MFV_IOC_MAGIC, 0x0d, unsigned int)
/* VAL_UINT32_T * */
#define VCODEC_INC_ENC_EMI_USER		_IOW(MFV_IOC_MAGIC, 0x15, unsigned int)
/* VAL_UINT32_T * */
#define VCODEC_DEC_ENC_EMI_USER		_IOW(MFV_IOC_MAGIC, 0x16, unsigned int)
/* VAL_UINT32_T * */
#define VCODEC_INC_DEC_EMI_USER		_IOW(MFV_IOC_MAGIC, 0x17, unsigned int)
/* VAL_UINT32_T * */
#define VCODEC_DEC_DEC_EMI_USER		_IOW(MFV_IOC_MAGIC, 0x18, unsigned int)
/* VAL_VCODEC_OAL_HW_REGISTER_T * */
#define VCODEC_INITHWLOCK		_IOW(MFV_IOC_MAGIC, 0x20, unsigned int)
/* VAL_VCODEC_OAL_HW_REGISTER_T * */
#define VCODEC_DEINITHWLOCK		_IOW(MFV_IOC_MAGIC, 0x21, unsigned int)
/* VAL_MEMORY_T * */
#define VCODEC_ALLOC_NON_CACHE_BUFFER	_IOW(MFV_IOC_MAGIC, 0x22, unsigned int)
/* VAL_MEMORY_T * */
#define VCODEC_FREE_NON_CACHE_BUFFER	_IOW(MFV_IOC_MAGIC, 0x23, unsigned int)
/* VAL_VCODEC_THREAD_ID_T * */
#define VCODEC_SET_THREAD_ID		_IOW(MFV_IOC_MAGIC, 0x24, unsigned int)
/* HAL_POWER_T * */
#define VCODEC_INC_PWR_USER		_IOW(MFV_IOC_MAGIC, 0x27, unsigned int)
/* HAL_POWER_T * */
#define VCODEC_DEC_PWR_USER		_IOW(MFV_IOC_MAGIC, 0x28, unsigned int)
/* VAL_VCODEC_CPU_LOADING_INFO_T * */
#define VCODEC_GET_CPU_LOADING_INFO	_IOW(MFV_IOC_MAGIC, 0x29, unsigned int)
/* VAL_VCODEC_CORE_LOADING_T * */
#define VCODEC_GET_CORE_LOADING		_IOW(MFV_IOC_MAGIC, 0x30, unsigned int)
/* int * */
#define VCODEC_GET_CORE_NUMBER		_IOW(MFV_IOC_MAGIC, 0x31, unsigned int)
/* VAL_VCODEC_CPU_OPP_LIMIT_T * */
#define VCODEC_SET_CPU_OPP_LIMIT	_IOW(MFV_IOC_MAGIC, 0x32, unsigned int)
/* VAL_HW_LOCK_T * */
#define VCODEC_UNLOCKHW			_IOW(MFV_IOC_MAGIC, 0x33, unsigned int)
/* VAL_UINT32_T * */
#define VCODEC_MB			_IOW(MFV_IOC_MAGIC, 0x34, unsigned int)


/* #define MFV_GET_CACHECTRLADDR_CMD  _IOR(MFV_IOC_MAGIC, 0x06, int) */

#ifdef CONFIG_MTK_HIBERNATION
extern void mt_irq_set_sens(unsigned int irq, unsigned int sens);
extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity);
#endif

#endif				/* __VCODEC_DRIVER_H__ */

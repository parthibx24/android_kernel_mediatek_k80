/*
 * Copyright (c) 2015-2017 MICROTRUST Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <teei_trusty.h>
#include <teei_smcall.h>
#include <teei_ipc.h>
#include <imsg_log.h>
#include "teei_loader.h"

#define ROUND_UP(N, S)		((((N) + (S) - 1) / (S)) * (S))
#define MAX_SHM_LEN		(0x180000)
#define TEEI_LOADER_MAJOR	252

#define TEEI_IOC_MAGIC		'T'
#define TEEI_LOAD_TA		_IO(TEEI_IOC_MAGIC, 0x1)
#define TEEI_SEND_MODEL_INFO	_IO(TEEI_IOC_MAGIC, 0x2)
#define CMD_FP_GET_UUID		_IO(TEEI_IOC_MAGIC, 0x3)

static struct cdev teei_loader_cdev;
static int loader_major = TEEI_LOADER_MAJOR;
static struct class *driver_class;
static dev_t devno;

DEFINE_MUTEX(loader_mutex);

struct teei_loader_head {
	unsigned long long file_len;
};

static int teei_loader_open(struct inode *inode, struct file *filp)
{
	IMSG_DEBUG("%s %d ====================\n", __func__, __LINE__);
	return 0;
}

static int teei_loader_release(struct inode *inode, struct file *filp)
{
	IMSG_DEBUG("%s %d ====================\n", __func__, __LINE__);
	return 0;
}

static ssize_t teei_loader_read(struct file *filp, char __user *buf,
			size_t size, loff_t *ppos)
{
	return 0;
}

static ssize_t teei_loader_write(struct file *filp, const char __user *buf,
			size_t size, loff_t *ppos)
{
	return 0;
}

static loff_t teei_loader_llseek(struct file *filp, loff_t offset, int orig)
{
	return 0;
}

static long load_ta_fn(unsigned long arg)
{

	struct teei_loader_head head;
	void *shared_buff = NULL;
	unsigned long shared_buff_pa = 0;
	long retVal = 0;
	unsigned long shm_size = 0;

	if (copy_from_user((void *)&head, (void *)arg, sizeof(head))) {
		IMSG_ERROR("Can not copy the loader head from user space!\n");
		return -EFAULT;
	}

	shm_size = head.file_len;

	if (ROUND_UP(shm_size, PAGE_SIZE) > MAX_SHM_LEN) {
		IMSG_ERROR("Loader file is TOO large, can NOT transfer it\n");
		return -EINVAL;
	}

	shared_buff = alloc_pages_exact(ROUND_UP(shm_size, PAGE_SIZE),
					GFP_KERNEL | GFP_DMA);

	shared_buff_pa = (unsigned long)virt_to_phys(shared_buff);

	if (shared_buff == 0) {
		IMSG_ERROR("%s: Can NOT get enough memory!\n", __func__);
		return -ENOMEM;
	}


	if (copy_from_user(shared_buff, (void *)(arg + sizeof(head)),
								shm_size)) {
		IMSG_ERROR("%s: Failed to copy the file data!\n", __func__);
		retVal = -EFAULT;
		goto err_free_shm;
	}

	retVal = trusty_mtee_std_call32(SMC_SC_LOAD_TA, (u32)(shared_buff_pa),
				(u32)((u64)shared_buff_pa >> 32), shm_size);
	if (retVal != 0) {
		IMSG_ERROR("%s: Failed to send SMC Call(%ld)!\n",
							__func__, retVal);
	}

err_free_shm:
	free_pages_exact((void *)shared_buff, get_order(
					ROUND_UP(shm_size, PAGE_SIZE)));

	return retVal;
}

static long send_model_fn(unsigned long arg)
{
	struct teei_loader_head head;
	void *shared_buff = NULL;
	unsigned long shared_buff_pa = 0;
	long retVal = 0;
	unsigned long shm_size = 0;

	if (copy_from_user((void *)&head, (void *)arg, sizeof(head))) {
		IMSG_ERROR("Can not copy the loader head from user space!\n");
		return -EFAULT;
	}

	shm_size = head.file_len;

	if (ROUND_UP(shm_size, PAGE_SIZE) > MAX_SHM_LEN) {
		IMSG_ERROR("loader file is TOO large, can NOT transfer it\n");
		return -EINVAL;
	}

	shared_buff = alloc_pages_exact(ROUND_UP(shm_size, PAGE_SIZE),
					GFP_KERNEL | GFP_DMA);
	if (shared_buff == 0) {
		IMSG_ERROR("%s: Can NOT get enough memory!\n", __func__);
		return -ENOMEM;
	}

	shared_buff_pa = (unsigned long)virt_to_phys(shared_buff);

	if (copy_from_user(shared_buff, (void *)(arg + sizeof(head)),
								shm_size)) {
		IMSG_ERROR("%s: Failed to copy from user space!\n", __func__);
		retVal = -EFAULT;
		goto err_free_shm;
	}

	retVal = trusty_mtee_std_call32(SMC_SC_SEND_MODEL_INFO,
					(u32)(shared_buff_pa),
					(u32)((u64)shared_buff_pa >> 32),
					shm_size);
	if (retVal != 0)
		IMSG_ERROR("%s: Failed to send SMC Call(%ld)!\n",
							__func__, retVal);

err_free_shm:
	free_pages_exact((void *)shared_buff,
				get_order(ROUND_UP(shm_size, PAGE_SIZE)));

	return retVal;
}

static long teei_loader_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	long retVal = 0;

	mutex_lock(&loader_mutex);

	switch (cmd) {
	case TEEI_LOAD_TA:
		retVal = load_ta_fn(arg);
		break;

	case TEEI_SEND_MODEL_INFO:
		retVal = send_model_fn(arg);
		break;

	case CMD_FP_GET_UUID:
		retVal = copy_to_user((void *)arg, &uuid_fp,
					sizeof(struct TEEC_UUID));
		break;

	default:
		IMSG_ERROR("%s: Unsupport IOCTL command!\n", __func__);
		retVal = -EINVAL;
	}

	mutex_unlock(&loader_mutex);

	return retVal;

}


static const struct file_operations teei_loader_fops = {
	.owner =		THIS_MODULE,
	.llseek =		teei_loader_llseek,
	.read =			teei_loader_read,
	.write =		teei_loader_write,
	.unlocked_ioctl =	teei_loader_ioctl,
	.compat_ioctl =		teei_loader_ioctl,
	.open =			teei_loader_open,
	.release =		teei_loader_release,
};

static int teei_loader_init(void)
{
	int retVal = 0;
	struct device *class_dev = NULL;

	devno = MKDEV(loader_major, 0);

	retVal = alloc_chrdev_region(&devno, 0, 1, "teei_loader");
	loader_major = MAJOR(devno);

	if (retVal < 0)
		return retVal;

	driver_class = class_create(THIS_MODULE, "teei_loader");

	if (IS_ERR(driver_class)) {
		retVal = -ENOMEM;
		IMSG_ERROR("class_create failed %d.\n", retVal);
		goto unregister_chrdev_region;
	}

	class_dev = device_create(driver_class, NULL, devno,
						NULL, "teei_loader");

	if (!class_dev) {
		retVal = -ENOMEM;
		IMSG_ERROR("class_device_create failed %d.\n", retVal);
		goto class_destroy;
	}

	devno = MKDEV(loader_major, 0);
	cdev_init(&teei_loader_cdev, &teei_loader_fops);
	teei_loader_cdev.owner = THIS_MODULE;
	retVal = cdev_add(&teei_loader_cdev, devno, 1);
	if (retVal < 0) {
		IMSG_ERROR("Failed to add cdev %d\n", retVal);
		goto class_device_destroy;
	}

	goto return_fn;

class_device_destroy:
	device_destroy(driver_class, devno);
class_destroy:
	class_destroy(driver_class);
unregister_chrdev_region:
	unregister_chrdev_region(devno, 1);
return_fn:
	return retVal;
}

static void teei_loader_exit(void)
{
	device_destroy(driver_class, devno);
	class_destroy(driver_class);
	cdev_del(&teei_loader_cdev);
	unregister_chrdev_region(MKDEV(loader_major, 0), 1);
}

MODULE_AUTHOR("Microtrust");
MODULE_LICENSE("Dual BSD/GPL");

module_init(teei_loader_init);
module_exit(teei_loader_exit);

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
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/semaphore.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <teei_ipc.h>
#include <imsg_log.h>
#include "fp_func.h"

#define FP_TAG "[teei_fp]"

struct fp_dev {
	struct cdev cdev;
	unsigned char mem[FP_LIMIT_SIZE];
	struct semaphore sem;
};

static int fp_major = FP_MAJOR;
static dev_t devno;

static struct class *driver_class;
struct fp_dev *fp_devp;

unsigned long fp_trans_addr;
uint8_t *r_buf;

tipc_k_handle fp_handle;

struct TEEC_UUID uuid_fp = { 0x7778c03f, 0xc30c, 0x4dd0,
{ 0xa3, 0x19, 0xea, 0x29, 0x64, 0x3d, 0x4d, 0x4b } };

int trusty_fingerprint_call(uint32_t cmd, void *in, uint32_t in_size,
				uint8_t *out, uint32_t *out_size)
{

	size_t msg_size = 0;
	ssize_t rc = 0;
	uint32_t send_cmd = 0;
	uint32_t recv_cmd = 0;
	struct fingerprint_message *msg = NULL;
	uint32_t bytes_remaining;
	uint32_t bytes_sent = 0;
	uint32_t max_msg_size = MAX_MESSAGE_SIZE;

	if (fp_handle < 0) {
		IMSG_ERROR("not connected\n");
		return -EINVAL;
	}

	bytes_remaining = in_size;

	do {
		msg_size = min(max_msg_size, bytes_remaining);
		send_cmd = cmd | FP_RESP_BIT;

		if (msg_size == bytes_remaining) {
			IMSG_DEBUG("fp ca send last package\n");
			send_cmd = cmd | FP_STOP_BIT;
		}

		msg = kmalloc(msg_size + sizeof(struct fingerprint_message),
				GFP_KERNEL);
		msg->cmd = send_cmd;
		memcpy(msg->payload, in+bytes_sent, msg_size);

		rc = tipc_k_write(fp_handle, msg,
				msg_size + sizeof(struct fingerprint_message),
				O_SYNC);
		kfree(msg);
		if (rc < 0) {
			IMSG_ERROR("failed to send cmd (%d) to %s\n",
							cmd, FINGERPRINT_PORT);
			return -1;
		}
		if (rc != (msg_size + sizeof(struct fingerprint_message))) {
			IMSG_ERROR("failed to send want (%lu):(%lu)\n",
								msg_size, rc);
		}

		bytes_remaining -= msg_size;
		bytes_sent += msg_size;
	} while (bytes_remaining);

	*out_size = 0;

	while (true) {
		memset(r_buf, 0, sizeof(r_buf));

		rc = tipc_k_read(fp_handle, r_buf,
					FINGERPRINT_MAX_BUFFER_LENGTH, O_SYNC);
		if (rc < 0) {
			IMSG_ERROR("failed to get response for cmd (%d) %s\n",
							cmd, FINGERPRINT_PORT);
			return -EINVAL;
		}

		if (rc == 0) {
			IMSG_ERROR("No data read from ta, total len is %d",
							*out_size);
			return -EINVAL;
		}

		if ((size_t)rc < sizeof(struct fingerprint_message)) {
			IMSG_ERROR("invalid response size (%d)\n", (int)rc);
			return -EINVAL;
		}

		recv_cmd = r_buf[3] << 24 | r_buf[2] << 16 |
					r_buf[1] << 8 | r_buf[0];

		IMSG_DEBUG("cmd = %d, recv_cmd = %d\n", cmd, recv_cmd);

		if ((cmd | FP_RESP_BIT) != (recv_cmd & ~(FP_STOP_BIT))) {
			IMSG_ERROR("invalid command (%d)\n", recv_cmd);
			return -EINVAL;
		}

		memcpy(out + *out_size,
			r_buf + sizeof(struct fingerprint_message),
			(size_t)rc - sizeof(struct fingerprint_message));

		*out_size += ((size_t)rc -
					sizeof(struct fingerprint_message));

		if (recv_cmd & FP_STOP_BIT) {
			/*
			 * IMSG_DEBUG("*out_size = %d\n", *out_size);
			 * for (i = 0; i < *out_size; i++)
			 *	IMSG_DEBUG("out data[%d] = %x\n", i, *(out+i));
			 */
			break;
		}
	}
	return 0;
}

int fp_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	IMSG_DEBUG("%s start\n", __func__);

	ret = tipc_k_connect(&fp_handle, FINGERPRINT_PORT);
	if (ret < 0) {
		IMSG_ERROR("service connect failed\n");
		return -1;
	}

	IMSG_DEBUG("%s finished!\n", __func__);

	filp->private_data = fp_devp;
	return ret;
}

int fp_release(struct inode *inode, struct file *filp)
{
	int ret = 0;

	IMSG_DEBUG("%s start\n", __func__);

	ret = tipc_k_disconnect(fp_handle);
	if (ret < 0) {
		IMSG_ERROR("service disconnect failed\n");
		return -1;
	}

	IMSG_DEBUG("%s finished!\n", __func__);

	filp->private_data = NULL;
	return ret;
}

static long fp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int i = 0;
	unsigned int args_len = 0;
	unsigned int fp_trans_len = 0;
	unsigned char args[16] = {0};

	IMSG_DEBUG("%s Enter\n", __func__);

	switch (cmd) {
	case CMD_FP_MEM_CLEAR:
		IMSG_DEBUG("CMD FP MEM CLEAR.\n");
		break;
	case CMD_FP_CMD:
		if (copy_from_user((void *)args, (void *)arg, 16)) {
			IMSG_ERROR("copy args from user failed.\n");
			return -EFAULT;
		}


		/* TODO compute args length */
		/* [11-15] is the length of data */
		args_len = *((unsigned int *)(args + 12));

		if (args_len + 16 > FP_LIMIT_SIZE) {
			IMSG_ERROR("args_len is invalid!, args_len is %d\n",
								args_len);
			return -EFAULT;
		}

		if (copy_from_user((void *)fp_trans_addr, (void *)arg,
					args_len + 16)) {
			IMSG_ERROR("copy from user failed.\n");
			return -EFAULT;
		}

		/*
		 *for (i = 0; i < args_len + 16; i++)
		 *	IMSG_DEBUG("fp_send_addr[%d] = %x\n", i,
		 *		*(unsigned char *)(fp_send_addr+i));
		 */
		lock_system_sleep();
		ret = trusty_fingerprint_call(FP_REQ, (void *)fp_trans_addr,
						args_len + 16,
						(uint8_t *)fp_trans_addr,
						&fp_trans_len);
		unlock_system_sleep();
		if (ret < 0) {
			IMSG_ERROR("transfer data failed. ret = %d\n", ret);
			return -EFAULT;
		}

		if (copy_to_user((void *)arg, (void *)fp_trans_addr,
						args_len + 16)) {
			IMSG_ERROR("copy to user failed.\n");
			return -EFAULT;
		}
		break;

	default:
		return -EINVAL;

	}
	return 0;
}

static ssize_t fp_read(struct file *filp, char __user *buf,
		size_t size, loff_t *ppos)
{
	int ret = 0;
	return ret;
}

static ssize_t fp_write(struct file *filp, const char __user *buf,
		size_t size, loff_t *ppos)
{
	return 0;
}

static loff_t fp_llseek(struct file *filp, loff_t offset, int orig)
{
	return 0;
}
static const struct file_operations fp_fops = {
	.owner = THIS_MODULE,
	.llseek = fp_llseek,
	.read = fp_read,
	.write = fp_write,
	.unlocked_ioctl = fp_ioctl,
	.compat_ioctl = fp_ioctl,
	.open = fp_open,
	.release = fp_release,
};

static void fp_setup_cdev(struct fp_dev *dev, int index)
{
	int err = 0;
	int devno = MKDEV(fp_major, index);

	cdev_init(&dev->cdev, &fp_fops);
	dev->cdev.owner = fp_fops.owner;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		IMSG_ERROR("Error %d adding fp %d.\n", err, index);
}

int fp_init(void)
{
	int result = 0;
	struct device *class_dev = NULL;

	devno = MKDEV(fp_major, 0);
	result = alloc_chrdev_region(&devno, 0, 1, DEV_NAME);
	fp_major = MAJOR(devno);
	if (result < 0)
		return result;

	driver_class = NULL;
	driver_class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(driver_class)) {
		result = -ENOMEM;
		IMSG_ERROR("class_create failed %d.\n", result);
		goto unregister_chrdev_region;
	}

	class_dev = device_create(driver_class, NULL, devno, NULL, DEV_NAME);
	if (!class_dev) {
		result = -ENOMEM;
		IMSG_ERROR("class_device_create failed %d.\n", result);
		goto class_destroy;
	}
	fp_devp = NULL;
	fp_devp = kmalloc(sizeof(struct fp_dev), GFP_KERNEL);
	if (fp_devp == NULL) {
		result = -ENOMEM;
		goto class_device_destroy;
	}
	memset(fp_devp, 0, sizeof(struct fp_dev));
	fp_setup_cdev(fp_devp, 0);
	fp_trans_addr = (unsigned long)kmalloc(FP_LIMIT_SIZE, GFP_KERNEL);
	if (fp_trans_addr == 0) {
		IMSG_ERROR("fp_trans_addr malloc failed\n");
		return -EFAULT;
	}
	memset((void *)fp_trans_addr, 0, FP_LIMIT_SIZE);

	r_buf = kmalloc(FINGERPRINT_MAX_BUFFER_LENGTH, GFP_KERNEL);
	if (r_buf == NULL) {
		IMSG_ERROR("r_buf malloc failed\n");
		return -EFAULT;
	}

	IMSG_DEBUG("[%s][%d]create the teei_fp device node successfully!\n",
							__func__, __LINE__);
	goto return_fn;

class_device_destroy:
	device_destroy(driver_class, devno);
class_destroy:
	class_destroy(driver_class);
unregister_chrdev_region:
	unregister_chrdev_region(devno, 1);
return_fn:
	return result;
}

void fp_exit(void)
{
	device_destroy(driver_class, devno);
	class_destroy(driver_class);
	cdev_del(&fp_devp->cdev);
	kfree(fp_devp);
	kfree(fp_trans_addr);
	kfree(r_buf);
	unregister_chrdev_region(MKDEV(fp_major, 0), 1);
}

MODULE_AUTHOR("Microtrust");
MODULE_LICENSE("Dual BSD/GPL");

module_param(fp_major, int, 0444);

module_init(fp_init);
module_exit(fp_exit);

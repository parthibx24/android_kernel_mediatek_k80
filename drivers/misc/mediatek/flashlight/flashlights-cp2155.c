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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

#define TAG_NAME "[flashligh_cp2155_drv]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__, ##arg)
#define PK_ERR(fmt, arg...)         pr_err(TAG_NAME "%s: " fmt, __func__, ##arg)


/*#define DEBUG_LEDS_STROBE*/
#ifdef DEBUG_LEDS_STROBE
#define PK_INF(fmt, arg...)       pr_info(TAG_NAME "%s is called.\n", __func__)
#define PK_DBG         PK_DBG_FUNC
#else
#define PK_INF(fmt, arg...)       do {} while (0)
#define PK_DBG(a, ...)
#endif


/* define device tree */
#ifndef CP2155_DTNAME
#define CP2155_DTNAME "mediatek,flashlights_cp2155"
#endif

#define CP2155_NAME "flashlights_cp2155"

/* define registers */

/* define mutex and work queue */
static DEFINE_MUTEX(cp2155_mutex);
static struct work_struct cp2155_work;

/* define pinctrl */
#define CP2155_PINCTRL_PIN_HWEN 0
#define CP2155_PINCTRL_PIN_MODE 1
#define CP2155_PINCTRL_PINSTATE_LOW 0
#define CP2155_PINCTRL_PINSTATE_HIGH 1
#define CP2155_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define CP2155_PINCTRL_STATE_HWEN_LOW  "hwen_low"
#define CP2155_PINCTRL_STATE_MODE_HIGH "mode_high"
#define CP2155_PINCTRL_STATE_MODE_LOW  "mode_low"

static struct pinctrl *cp2155_pinctrl;
static struct pinctrl_state *cp2155_hwen_high;
static struct pinctrl_state *cp2155_hwen_low;
static struct pinctrl_state *cp2155_mode_high;
static struct pinctrl_state *cp2155_mode_low;

/* define usage count */
static int use_count;

static int g_flash_duty = -1;

/* platform data */
struct cp2155_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int cp2155_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	cp2155_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(cp2155_pinctrl)) {
		PK_ERR("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(cp2155_pinctrl);
	}

	/*  Flashlight pin initialization */
	cp2155_hwen_high = pinctrl_lookup_state(
			cp2155_pinctrl, CP2155_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(cp2155_hwen_high)) {
		PK_ERR("Failed to init (%s)\n", CP2155_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(cp2155_hwen_high);
	}
	cp2155_hwen_low = pinctrl_lookup_state(
			cp2155_pinctrl, CP2155_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(cp2155_hwen_low)) {
		PK_ERR("Failed to init (%s)\n", CP2155_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(cp2155_hwen_low);
	}

	cp2155_mode_high = pinctrl_lookup_state(
			cp2155_pinctrl, CP2155_PINCTRL_STATE_MODE_HIGH);
	if (IS_ERR(cp2155_mode_high)) {
		PK_ERR("Failed to init (%s)\n", CP2155_PINCTRL_STATE_MODE_HIGH);
		ret = PTR_ERR(cp2155_mode_high);
	}
	cp2155_mode_low = pinctrl_lookup_state(
			cp2155_pinctrl, CP2155_PINCTRL_STATE_MODE_LOW);
	if (IS_ERR(cp2155_mode_low)) {
		PK_ERR("Failed to init (%s)\n", CP2155_PINCTRL_STATE_MODE_LOW);
		ret = PTR_ERR(cp2155_mode_low);
	}

	return ret;
}

static int cp2155_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(cp2155_pinctrl)) {
		PK_ERR("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case CP2155_PINCTRL_PIN_HWEN:
		if (state == CP2155_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(cp2155_hwen_low))
			ret = pinctrl_select_state(
					cp2155_pinctrl, cp2155_hwen_low);
		else if (state == CP2155_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(cp2155_hwen_high))
			ret = pinctrl_select_state(
					cp2155_pinctrl, cp2155_hwen_high);
		else
			PK_ERR("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case CP2155_PINCTRL_PIN_MODE:
		if (state == CP2155_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(cp2155_mode_low))
			ret = pinctrl_select_state(
					cp2155_pinctrl, cp2155_mode_low);
		else if (state == CP2155_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(cp2155_mode_high))
			ret = pinctrl_select_state(
					cp2155_pinctrl, cp2155_mode_high);
		else
			PK_ERR("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		PK_ERR("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	PK_DBG("pin(%d) state(%d), ret:%d\n", pin, state, ret);

	return ret;
}


/******************************************************************************
 * cp2155 operations
 *****************************************************************************/
/* flashlight enable function */
static int cp2155_enable(void)
{
	int pin_en = CP2155_PINCTRL_PIN_HWEN;
	int pin_mode = CP2155_PINCTRL_PIN_MODE;

	if (g_flash_duty == 1)   /* torch mode */
		cp2155_pinctrl_set(pin_mode, CP2155_PINCTRL_PINSTATE_LOW);
	else                     /* flash mode */
		cp2155_pinctrl_set(pin_mode, CP2155_PINCTRL_PINSTATE_HIGH);
	cp2155_pinctrl_set(pin_en, CP2155_PINCTRL_PINSTATE_HIGH);

	return 0;
}

/* flashlight disable function */
static int cp2155_disable(void)
{
	int pin_en = CP2155_PINCTRL_PIN_HWEN;
	int pin_mode = CP2155_PINCTRL_PIN_MODE;
	int state = CP2155_PINCTRL_PINSTATE_LOW;

	cp2155_pinctrl_set(pin_mode, state);
	cp2155_pinctrl_set(pin_en, state);

	return 0;
}

/* set flashlight level */
static int cp2155_set_level(int level)
{
	g_flash_duty = level;
	return 0;
}

/* flashlight init */
static int cp2155_init(void)
{
	int pin_en = CP2155_PINCTRL_PIN_HWEN;
	int pin_mode = CP2155_PINCTRL_PIN_MODE;
	int state = CP2155_PINCTRL_PINSTATE_LOW;

	cp2155_pinctrl_set(pin_mode, state);
	cp2155_pinctrl_set(pin_en, state);

	return 0;
}

/* flashlight uninit */
static int cp2155_uninit(void)
{
	cp2155_disable();
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer cp2155_timer;
static unsigned int cp2155_timeout_ms;

static void cp2155_work_disable(struct work_struct *data)
{
	PK_DBG("work queue callback\n");
	cp2155_disable();
}

static enum hrtimer_restart cp2155_timer_func(struct hrtimer *timer)
{
	schedule_work(&cp2155_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int cp2155_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		cp2155_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		cp2155_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (cp2155_timeout_ms) {
				s = cp2155_timeout_ms / 1000;
				ns = cp2155_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&cp2155_timer, ktime,
						HRTIMER_MODE_REL);
			}
			cp2155_enable();
		} else {
			cp2155_disable();
			hrtimer_cancel(&cp2155_timer);
		}
		break;
	default:
		PK_INF("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int cp2155_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int cp2155_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int cp2155_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&cp2155_mutex);
	if (set) {
		if (!use_count)
			ret = cp2155_init();
		use_count++;
		PK_DBG("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = cp2155_uninit();
		if (use_count < 0)
			use_count = 0;
		PK_DBG("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&cp2155_mutex);

	return ret;
}

static ssize_t cp2155_strobe_store(struct flashlight_arg arg)
{
	cp2155_set_driver(1);
	cp2155_set_level(arg.level);
	cp2155_timeout_ms = 0;
	cp2155_enable();
	msleep(arg.dur);
	cp2155_disable();
	cp2155_set_driver(0);

	return 0;
}

static struct flashlight_operations cp2155_ops = {
	cp2155_open,
	cp2155_release,
	cp2155_ioctl,
	cp2155_strobe_store,
	cp2155_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int cp2155_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * cp2155_init();
	 */

	return 0;
}

static int cp2155_parse_dt(struct device *dev,
		struct cp2155_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		PK_INF("Parse no dt, node.\n");
		return 0;
	}
	PK_INF("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		PK_INF("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				CP2155_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		PK_INF("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int cp2155_probe(struct platform_device *pdev)
{
	struct cp2155_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	PK_DBG("Probe start.\n");

	/* init pinctrl */
	if (cp2155_pinctrl_init(pdev)) {
		PK_DBG("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = cp2155_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&cp2155_work, cp2155_work_disable);

	/* init timer */
	hrtimer_init(&cp2155_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cp2155_timer.function = cp2155_timer_func;
	cp2155_timeout_ms = 100;

	/* init chip hw */
	cp2155_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&cp2155_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(CP2155_NAME, &cp2155_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	PK_DBG("Probe done.\n");

	return 0;
err:
	return err;
}

static int cp2155_remove(struct platform_device *pdev)
{
	struct cp2155_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	PK_DBG("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(CP2155_NAME);

	/* flush work queue */
	flush_work(&cp2155_work);

	PK_DBG("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id cp2155_gpio_of_match[] = {
	{.compatible = CP2155_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, cp2155_gpio_of_match);
#else
static struct platform_device cp2155_gpio_platform_device[] = {
	{
		.name = CP2155_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, cp2155_gpio_platform_device);
#endif

static struct platform_driver cp2155_platform_driver = {
	.probe = cp2155_probe,
	.remove = cp2155_remove,
	.driver = {
		.name = CP2155_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = cp2155_gpio_of_match,
#endif
	},
};

static int __init flashlight_cp2155_init(void)
{
	int ret;

	PK_DBG("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&cp2155_gpio_platform_device);
	if (ret) {
		PK_ERR("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&cp2155_platform_driver);
	if (ret) {
		PK_ERR("Failed to register platform driver\n");
		return ret;
	}

	PK_DBG("Init done.\n");

	return 0;
}

static void __exit flashlight_cp2155_exit(void)
{
	PK_DBG("Exit start.\n");

	platform_driver_unregister(&cp2155_platform_driver);

	PK_DBG("Exit done.\n");
}

module_init(flashlight_cp2155_init);
module_exit(flashlight_cp2155_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight cp2155 GPIO Driver");


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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/types.h>
#ifdef CONFIG_MTK_HIBERNATION
#include <mtk_hibernate_dpm.h>
#endif
#include "mt_device_apc.h"
#include "mt_io.h"
#include "sync_write.h"
#include "mach/irqs.h"
#include "devapc.h"

/* false for early porting */
#define DEVAPC_TURN_ON  1

static struct cdev *g_devapc_ctrl;
static unsigned int devapc_irq;
static void __iomem *devapc_ao_base;
static void __iomem *devapc_pd_base;

static unsigned int enable_dynamic_one_core_violation_debug;

#if DEVAPC_TURN_ON
static struct DEVICE_INFO devapc_devices[] = {
	/* 0 */
	{"INFRA_AO_TOP_LEVEL_CLOCK_GENERATOR",      false},
	{"INFRA_AO_INFRASYS_CONFIG_REGS",           false},
	{"INFRA_AO_KPAD_CONTROL_REG",               true},
	{"INFRA_AO_PERISYS_CONFIG_REGS",            true},
	{"INFRA_AO_RESERVED_0",                     true},
	{"INFRA_AO_GPIO_CONTROLLER",                false},
	{"INFRA_AO_TOP_LEVEL_SLP_MANAGER",          true},
	{"INFRA_AO_TOP_LEVEL_RESET_GENERATOR",      true},
	{"INFRA_AO_GPT",                            true},
	{"INFRA_AO_EFUSE",                          false},

	/* 10 */
	{"INFRA_AO_SEJ",                            true},
	{"INFRA_AO_APMCU_EINT_CONTROLLER",          true},
	{"INFRA_AO_AP_CCIF",                        false},
	{"INFRA_AO_MD_CCIF",                        false},
	{"INFRA_AO_AES_TOP",                        false},
	{"INFRA_AO_PMIC_WRAP_CONTROL_REG",          false},
	{"INFRA_AO_DEVICE_APC_AO",                  true},
	{"INFRA_AO_MIPI_RX_CONFIG",                 true},
	{"INFRA_AO_MBIST_CONTROL_REG",              true},
	{"INFRA_AO_RESERVED_1",                     true},

	/* 20 */
	{"INFRA_AO_IO_CONFG_T",                     true},
	{"INFRA_AO_IO_CONFG_B",                     true},
	{"INFRA_AO_IO_CONFG_L",                     true},
	{"INFRA_AO_IO_CONFG_R",                     true},
	{"INFRA_AO_APMIXEDSYS",                     true},
	{"DEGBUGSYS",                               true},
	{"INFRASYS_MCUSYS_CONFIG_REG",              true},
	{"INFRASYS_CONTROL_REG",                    true},
	{"INFRASYS_SYSTEM_CIRQ",                    true},
	{"INFRASYS_M4U_CONFIGURATION",              true},

	/* 30 */
	{"INFRASYS_DEVICE_APC_MONITOR",             true},
	{"INFRASYS_EMI_BUS_INTERFACE",              false},
	{"INFRASYS_DRAMC_NAO_REGION_REG",           true},
	{"INFRASYS_DRAMC_CONFIGURATION",            true},
	{"INFRASYS_DDRPHY_CONFIG",                  true},
	{"INFRASYS_BOOTROM/SRAM",                   true},
	{"INFRASYS_GCE",                            true},
	{"INFRASYS_BUS_TRACKER",                    true},
	{"INFRASYS_TRNG",                           true},
	{"INFRASYS_RESERVED_1",                     true},

	/* 40 */
	{"DMA",                                     true},
	{"NFI",                                     true},
	{"NFI_ECC",                                 true},
	{"AUXADC",                                  false},
	{"FHCTL",                                   true},
	{"UART0",                                   true},
	{"UART1",                                   true},
	{"UART2",                                   true},
	{"PWM",                                     true},
	{"I2C0",                                    true},

	/* 50 */
	{"I2C1",                                    true},
	{"I2C2",                                    true},
	{"SPI0",                                    true},
	{"PTP",                                     true},
	{"BTIF",                                    true},
	{"PWM_DISP",                                true},
	{"USB2.0",                                  false},
	{"USBSIF",                                  false},
	{"MSDC0",                                   true},
	{"MSDC1",                                   true},

	/* 60 */
	{"AUDIO",                                   false},
	{"CONN_PERIPHERALS",                        false},
	{"MD1_PERIPHERALS",                         true},
	{"MFG_0",                                   true},
	{"MFG_1",                                   true},
	{"MMSYS_CONFIG",                            true},
	{"MDP_RDMA",                                true},
	{"MDP_RSZ0",                                true},
	{"MDP_RSZ1",                                true},
	{"MDP_WDMA",                                true},

	/* 70 */
	{"MDP_WROT",                                true},
	{"MDP_TDSHP",                               true},
	{"DISP_OVL0",                               true},
	{"DISP_OVL1_",                              true},
	{"DISP_RDMA0",                              true},
	{"DISP_RDMA1",                              true},
	{"DISP_WDMA",                               true},
	{"DISP_COLOR",                              true},
	{"DISP_CCORR",                              true},
	{"DISP_AAL",                                true},

	/* 80 */
	{"DISP_GAMMA",                              true},
	{"DISP_DITHER",                             true},
	{"DSI_UFOE",                                true},
	{"DSI0",                                    true},
	{"DPI0",                                    true},
	{"DISP_PWM0",                               true},
	{"MM_MUTEX",                                true},
	{"SMI_LARB0",                               true},
	{"SMI_COMMON",                              true},
	{"MIPI_TX_CONFIG",                          true},

	/* 90 */
	{"IMG_CONFIG",                              true},
	{"IMG_SMI_LARB2",                           true},
	{"IMG_CAM_0",                               true},
	{"IMG_CAM_1",                               true},
	{"IMG_SENINF_TOP",                          true},
	{"IMG_VENC",                                true},
	{"IMG_VDEC",                                true},
	{"UFOZIP",                                  true},
};
#endif

/*****************************************************************************
*FUNCTION DEFINITION
*****************************************************************************/
#if DEVAPC_TURN_ON
static int clear_vio_status(unsigned int module);
#endif
static int devapc_ioremap(void);
/**************************************************************************
*EXTERN FUNCTION
**************************************************************************/
int mt_devapc_check_emi_violation(void)
{
	if ((readl(IOMEM(DEVAPC0_D0_VIO_STA_3)) & ABORT_EMI) == 0)
		return -1;

	pr_err("EMI violation! It should be cleared by EMI MPU driver later!\n");
	return 0;
}

int mt_devapc_emi_initial(void)
{
	pr_warn("EMI_DAPC Init start\n");

	devapc_ioremap();

	mt_reg_sync_writel(readl(IOMEM(DEVAPC0_APC_CON)) & (0xFFFFFFFF ^ (1 << 2)), DEVAPC0_APC_CON);
	mt_reg_sync_writel(readl(IOMEM(DEVAPC0_PD_APC_CON)) & (0xFFFFFFFF ^ (1 << 2)), DEVAPC0_PD_APC_CON);
	mt_reg_sync_writel(ABORT_EMI, DEVAPC0_D0_VIO_STA_3);
	mt_reg_sync_writel(readl(IOMEM(DEVAPC0_D0_VIO_MASK_3)) & (0xFFFFFFFF ^ (ABORT_EMI)), DEVAPC0_D0_VIO_MASK_3);
	pr_warn("EMI_DAPC Init done\n");
	return 0;
}

int mt_devapc_clear_emi_violation(void)
{
	if ((readl(IOMEM(DEVAPC0_D0_VIO_STA_3)) & ABORT_EMI) != 0)
		mt_reg_sync_writel(ABORT_EMI, DEVAPC0_D0_VIO_STA_3);

	return 0;
}


 /*
 * mt_devapc_set_permission: set module permission on device apc.
 * @module: the moudle to specify permission
 * @domain_num: domain index number
 * @permission_control: specified permission
 * no return value.
 */
int mt_devapc_set_permission(unsigned int module, E_MASK_DOM domain_num, APC_ATTR permission)
{
	volatile unsigned int *base;
	unsigned int clr_bit = 0x3 << ((module % MOD_NO_IN_1_DEVAPC) * 2);
	unsigned int set_bit = permission << ((module % MOD_NO_IN_1_DEVAPC) * 2);

	if (module >= DEVAPC_DEVICE_NUMBER) {
		pr_warn("[DEVAPC] ERROR, device number %d exceeds the max number!\n", module);
		return -1;
	}

	if (domain_num == DEVAPC_DOMAIN_AP)
		base = (volatile unsigned int *)((unsigned int)DEVAPC0_D0_APC_0 + (module / MOD_NO_IN_1_DEVAPC) * 4);
	else if (domain_num == DEVAPC_DOMAIN_MD)
		base = (volatile unsigned int *)((unsigned int)DEVAPC0_D1_APC_0 + (module / MOD_NO_IN_1_DEVAPC) * 4);
	else if (domain_num == DEVAPC_DOMAIN_CONN)
		base = (volatile unsigned int *)((unsigned int)DEVAPC0_D2_APC_0 + (module / MOD_NO_IN_1_DEVAPC) * 4);
	else if (domain_num == DEVAPC_DOMAIN_MM)
		base = (volatile unsigned int *)((unsigned int)DEVAPC0_D3_APC_0 + (module / MOD_NO_IN_1_DEVAPC) * 4);
	else {
		pr_warn("[DEVAPC] ERROR, domain number %d exceeds the max number!\n", domain_num);
		return -2;
	}

	mt_reg_sync_writel(readl(base) & ~clr_bit, base);
	mt_reg_sync_writel(readl(base) | set_bit, base);
	return 0;
}
/**************************************************************************
*STATIC FUNCTION
**************************************************************************/

static int devapc_ioremap(void)
{
	struct device_node *node = NULL;
	/*IO remap*/
	node = of_find_compatible_node(NULL, NULL, "mediatek,DEVAPC_AO");
	if (node) {
		devapc_ao_base = of_iomap(node, 0);
		pr_warn("[DEVAPC] AO_ADDRESS=%p\n", devapc_ao_base);
	} else {
		pr_warn("[DEVAPC] can't find DAPC_AO compatible node\n");
		return -1;
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,DEVAPC");
	if (node) {
		devapc_pd_base = of_iomap(node, 0);
		devapc_irq = irq_of_parse_and_map(node, 0);
		pr_warn("[DEVAPC] PD_ADDRESS=%p, IRD: %d\n", devapc_pd_base, devapc_irq);
	} else {
		pr_warn("[DEVAPC] can't find DAPC_PD compatible node\n");
		return -1;
	}

	return 0;
}

#ifdef CONFIG_MTK_HIBERNATION
static int devapc_pm_restore_noirq(struct device *device)
{
	if (devapc_irq != 0) {
		mt_irq_set_sens(devapc_irq, MT_LEVEL_SENSITIVE);
		mt_irq_set_polarity(devapc_irq, MT_POLARITY_LOW);
	}

	return 0;
}
#endif

/*
 * set_module_apc: set module permission on device apc.
 * @module: the moudle to specify permission
 * @devapc_num: device apc index number (device apc 0 or 1)
 * @domain_num: domain index number (AP or MD domain)
 * @permission_control: specified permission
 * no return value.
 */
#if DEVAPC_TURN_ON
#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT)
static void start_devapc(void)
{
	pr_warn("[DEVAPC] Walk TEE path\n");

	mt_reg_sync_writel(readl(DEVAPC0_PD_APC_CON) & (0xFFFFFFFF ^ (1<<2)), DEVAPC0_PD_APC_CON);
}
#else
static void set_module_apc(unsigned int module, E_MASK_DOM domain_num, APC_ATTR permission_control)
{
	volatile unsigned int *base;
	unsigned int clr_bit = 0x3 << ((module % MOD_NO_IN_1_DEVAPC) * 2);
	unsigned int set_bit = permission_control << ((module % MOD_NO_IN_1_DEVAPC) * 2);

	if (module >= DEVAPC_DEVICE_NUMBER) {
		pr_warn("set_module_apc : device number %d exceeds the max number!\n", module);
		return;
	}

	if (domain_num == DEVAPC_DOMAIN_AP)
		base = (volatile unsigned int *)((unsigned int)DEVAPC0_D0_APC_0 + (module / MOD_NO_IN_1_DEVAPC) * 4);
	else if (domain_num == DEVAPC_DOMAIN_MD)
		base = (volatile unsigned int *)((unsigned int)DEVAPC0_D1_APC_0 + (module / MOD_NO_IN_1_DEVAPC) * 4);
	else if (domain_num == DEVAPC_DOMAIN_CONN)
		base = (volatile unsigned int *)((unsigned int)DEVAPC0_D2_APC_0 + (module / MOD_NO_IN_1_DEVAPC) * 4);
	else if (domain_num == DEVAPC_DOMAIN_MM)
		base = (volatile unsigned int *)((unsigned int)DEVAPC0_D3_APC_0 + (module / MOD_NO_IN_1_DEVAPC) * 4);
	else {
		pr_warn("set_module_apc : domain number %d exceeds the max number!\n", domain_num);
		return;
	}

	mt_reg_sync_writel(readl(base) & ~clr_bit, base);
	mt_reg_sync_writel(readl(base) | set_bit, base);
}

/*
 * unmask_module_irq: unmask device apc irq for specified module.
 * @module: the moudle to unmask
 * @devapc_num: device apc index number (device apc 0 or 1)
 * @domain_num: domain index number (AP or MD domain)
 * no return value.
 */
static int unmask_module_irq(unsigned int module)
{

	unsigned int apc_index = 0;
	unsigned int apc_bit_index = 0;

	if (module >= DEVAPC_MASK_NUMBER) {
		pr_warn("unmask_module_irq : module number %d exceeds the max number!\n", module);
		return -1;
	}

	apc_index = module / (MOD_NO_IN_1_DEVAPC*2);
	apc_bit_index = module % (MOD_NO_IN_1_DEVAPC*2);

	switch (apc_index) {
	case 0:
		*DEVAPC0_D0_VIO_MASK_0 &= ~(0x1 << apc_bit_index);
		break;
	case 1:
		*DEVAPC0_D0_VIO_MASK_1 &= ~(0x1 << apc_bit_index);
		break;
	case 2:
		*DEVAPC0_D0_VIO_MASK_2 &= ~(0x1 << apc_bit_index);
		break;
	case 3:
		*DEVAPC0_D0_VIO_MASK_3 &= ~(0x1 << apc_bit_index);
		break;
	default:
		break;
	}
	return 0;
}

static void init_devpac(void)
{
	/* clear violation*/
	mt_reg_sync_writel(0x80000000, DEVAPC0_VIO_DBG0);
	mt_reg_sync_writel(readl(DEVAPC0_APC_CON) & (0xFFFFFFFF ^ (1<<2)), DEVAPC0_APC_CON);
	mt_reg_sync_writel(readl(DEVAPC0_PD_APC_CON) & (0xFFFFFFFF ^ (1<<2)), DEVAPC0_PD_APC_CON);
}


/*
 * start_devapc: start device apc for MD
 */
static int start_devapc(void)
{
	unsigned int i = 0;

	init_devpac();
	for (i = 0; i < ARRAY_SIZE(devapc_devices); ++i) {
		clear_vio_status(i);
		unmask_module_irq(i);
		if (devapc_devices[i].forbidden == true) {
			set_module_apc(i, E_DOMAIN_1, E_L3);
		}
	}
	return 0;
}


#endif


/*
 * clear_vio_status: clear violation status for each module.
 * @module: the moudle to clear violation status
 * @devapc_num: device apc index number (device apc 0 or 1)
 * @domain_num: domain index number (AP or MD domain)
 * no return value.
 */
static int clear_vio_status(unsigned int module)
{

	unsigned int apc_index = 0;
	unsigned int apc_bit_index = 0;

	if (module >= DEVAPC_VIO_NUMBER) {
		pr_warn("clear_vio_status : module number %d exceeds the max number!\n", module);
		return -1;
	}

	apc_index = module / (MOD_NO_IN_1_DEVAPC*2);
	apc_bit_index = module % (MOD_NO_IN_1_DEVAPC*2);

	switch (apc_index) {
	case 0:
		*DEVAPC0_D0_VIO_STA_0 = (0x1 << apc_bit_index);
		break;
	case 1:
		*DEVAPC0_D0_VIO_STA_1 = (0x1 << apc_bit_index);
		break;
	case 2:
		*DEVAPC0_D0_VIO_STA_2 = (0x1 << apc_bit_index);
		break;
	case 3:
		*DEVAPC0_D0_VIO_STA_3 = (0x1 << apc_bit_index);
		break;
	default:
		break;
	}

	return 0;
}


static irqreturn_t devapc_violation_irq(int irq, void *dev_id)
{
	unsigned int dbg0 = 0, dbg1 = 0;
	unsigned int master_id;
	unsigned int domain_id;
	unsigned int r_w_violation;
	unsigned int i;
	struct pt_regs *regs;

	dbg0 = readl(DEVAPC0_VIO_DBG0);
	dbg1 = readl(DEVAPC0_VIO_DBG1);
	master_id = (dbg0 & VIO_DBG_MSTID) >> 0;
	domain_id = (dbg0 & VIO_DBG_DMNID) >> 14;
	r_w_violation = (dbg0 & VIO_DBG_W) >> 28;

	if (1 == r_w_violation) {
		pr_err("[DEVAPC] Violation(W) Process:%s PID:%i Vio Addr:0x%x, Master ID:0x%x, Dom ID:0x%x\n",
			current->comm, current->pid, dbg1, master_id, domain_id);
	} else {
		pr_err("[DEVAPC] Violation(R) Process:%s PID:%i Vio Addr:0x%x, Master ID:0x%x, Dom ID:0x%x\n",
			current->comm, current->pid, dbg1, master_id, domain_id);
	}

	pr_err("[DEVAPC] VIO_STA 0:0x%x, 1:0x%x, 2:0x%x, 3:0x%x\n",
		readl(DEVAPC0_D0_VIO_STA_0), readl(DEVAPC0_D0_VIO_STA_1), readl(DEVAPC0_D0_VIO_STA_2),
		readl(DEVAPC0_D0_VIO_STA_3));

	for (i = 0; i < ARRAY_SIZE(devapc_devices); ++i)
		clear_vio_status(i);

	if (DEVAPC_ENABLE_ONE_CORE_VIOLATION_DEBUG || enable_dynamic_one_core_violation_debug) {
		pr_err("[DEVAPC] ====== Start dumping Device APC violation tracing ======\n");

		pr_err("[DEVAPC] **************** [All IRQ Registers] ****************\n");
		regs = get_irq_regs();
		show_regs(regs);

		pr_err("[DEVAPC] **************** [All Current Task Stack] ****************\n");
		show_stack(current, NULL);

		pr_err("[DEVAPC] ====== End of dumping Device APC violation tracing ======\n");
	}

	mt_reg_sync_writel(VIO_DBG_CLR, DEVAPC0_VIO_DBG0);
	dbg0 = readl(DEVAPC0_VIO_DBG0);
	dbg1 = readl(DEVAPC0_VIO_DBG1);

	if ((dbg0 != 0) || (dbg1 != 0)) {
		pr_err("[DEVAPC] Multi-violation!\n");
		pr_err("[DEVAPC] DBG0 = %x, DBG1 = %x\n", dbg0, dbg1);
	}

	return IRQ_HANDLED;
}
#endif

static int devapc_probe(struct platform_device *dev)
{
#if DEVAPC_TURN_ON
	int ret;
#endif

	pr_warn("[DEVAPC] module probe.\n");
	/*IO remap*/
	devapc_ioremap();
	/*
	* Interrupts of violation (including SPC in SMI, or EMI MPU) are triggered by the device APC.
	* need to share the interrupt with the SPC driver.
	*/
#if DEVAPC_TURN_ON
	ret = request_irq(devapc_irq, (irq_handler_t)devapc_violation_irq,
	IRQF_TRIGGER_LOW | IRQF_SHARED, "devapc", &g_devapc_ctrl);
	if (ret) {
		pr_err("[DEVAPC] Failed to request irq! (%d)\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_MTK_HIBERNATION
	register_swsusp_restore_noirq_func(ID_M_DEVAPC, devapc_pm_restore_noirq, NULL);
#endif

#if DEVAPC_TURN_ON
	start_devapc();
#endif
	return 0;
}


static int devapc_remove(struct platform_device *dev)
{
	pr_info("[DEVAPC] module remove.\n");
	return 0;
}

static int devapc_suspend(struct platform_device *dev, pm_message_t state)
{
	pr_info("[DEVAPC] module suspend.\n");
	return 0;
}

static int devapc_resume(struct platform_device *dev)
{
	pr_info("[DEVAPC] module resume.\n");
	return 0;
}

static int check_debug_input_type(const char *str)
{
	if (sysfs_streq(str, "1"))
		return DAPC_INPUT_TYPE_DEBUG_ON;
	else if (sysfs_streq(str, "0"))
		return DAPC_INPUT_TYPE_DEBUG_OFF;
	else
		return 0;
}

static ssize_t devapc_dbg_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;
	int input_type;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	input_type = check_debug_input_type(desc);
	if (!input_type)
		return -EFAULT;

	if (input_type == DAPC_INPUT_TYPE_DEBUG_ON) {
		enable_dynamic_one_core_violation_debug = 1;
		pr_err("[DEVAPC] One-Core Debugging: Enabled\n");
	} else if (input_type == DAPC_INPUT_TYPE_DEBUG_OFF) {
		enable_dynamic_one_core_violation_debug = 0;
		pr_err("[DEVAPC] One-Core Debugging: Disabled\n");
	}

	return count;
}

static int devapc_dbg_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations devapc_dbg_fops = {
	.owner = THIS_MODULE,
	.open  = devapc_dbg_open,
	.write = devapc_dbg_write,
	.read = NULL,
};

struct platform_device devapc_device = {
	.name = "devapc",
	.id = -1,
};

static struct platform_driver devapc_driver = {
	.probe = devapc_probe,
	.remove = devapc_remove,
	.suspend = devapc_suspend,
	.resume = devapc_resume,
	.driver = {
	.name = "devapc",
	.owner = THIS_MODULE,
	},
};

/*
 * devapc_init: module init function.
 */
static int __init devapc_init(void)
{
	int ret;

	/*You do not need to open the clock for Rainier*/
	/* enable_clock(MT_CG_INFRA_DEVICE_APC,"DEVAPC"); */

	pr_warn("[DEVAPC] module init.\n");

	ret = platform_device_register(&devapc_device);
	if (ret) {
		pr_err("[DEVAPC] Unable to do device register(%d)\n", ret);
		return ret;
	}

	ret = platform_driver_register(&devapc_driver);
	if (ret) {
		pr_err("[DEVAPC] Unable to register driver (%d)\n", ret);
		platform_device_unregister(&devapc_device);
		return ret;
	}

	g_devapc_ctrl = cdev_alloc();
	if (!g_devapc_ctrl) {
		pr_err("[DEVAPC] Failed to add devapc device! (%d)\n", ret);
		platform_driver_unregister(&devapc_driver);
		platform_device_unregister(&devapc_device);
		return ret;
	}
	g_devapc_ctrl->owner = THIS_MODULE;

	proc_create("devapc_dbg", (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH), NULL,
			&devapc_dbg_fops);

	return 0;
}

/*
 * devapc_exit: module exit function.
 */
static void __exit devapc_exit(void)
{
	pr_info("[DEVAPC] DEVAPC module exit\n");
#ifdef CONFIG_MTK_HIBERNATION
	unregister_swsusp_restore_noirq_func(ID_M_DEVAPC);
#endif
}

late_initcall(devapc_init);
module_exit(devapc_exit);
MODULE_LICENSE("GPL");

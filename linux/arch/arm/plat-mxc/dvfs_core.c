/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file dvfs_core.c
 *
 * @brief A simplied driver for the Freescale Semiconductor MXC DVFS module.
 *
 * Upon initialization, the DVFS driver initializes the DVFS hardware
 * sets up driver nodes attaches to the DVFS interrupt and initializes internal
 * data structures. When the DVFS interrupt occurs the driver checks the cause
 * of the interrupt (lower frequency, increase frequency or emergency) and
 * changes the CPU voltage according to translation table that is loaded into
 * the driver.
 *
 * @ingroup PM
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/regulator.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#define MXC_DVFSTHRS_UPTHR_MASK               0x0FC00000
#define MXC_DVFSTHRS_UPTHR_OFFSET             22
#define MXC_DVFSTHRS_DNTHR_MASK               0x003F0000
#define MXC_DVFSTHRS_DNTHR_OFFSET             16
#define MXC_DVFSTHRS_PNCTHR_MASK              0x0000003F
#define MXC_DVFSTHRS_PNCTHR_OFFSET            0

#define MXC_DVFSCOUN_DNCNT_MASK               0x00FF0000
#define MXC_DVFSCOUN_DNCNT_OFFSET             16
#define MXC_DVFSCOUN_UPCNT_MASK              0x000000FF
#define MXC_DVFSCOUN_UPCNT_OFFSET            0

#define MXC_DVFSEMAC_EMAC_MASK               0x000001FF
#define MXC_DVFSEMAC_EMAC_OFFSET             0

#define MXC_DVFSCNTR_DVFEV                   0x10000000
#define MXC_DVFSCNTR_LBMI                    0x08000000
#define MXC_DVFSCNTR_LBFL                    0x06000000
#define MXC_DVFSCNTR_DVFIS                   0x01000000
#define MXC_DVFSCNTR_FSVAIM                  0x00400000
#define MXC_DVFSCNTR_FSVAI_MASK              0x00300000
#define MXC_DVFSCNTR_FSVAI_OFFSET            20
#define MXC_DVFSCNTR_WFIM                    0x00080000
#define MXC_DVFSCNTR_WFIM_OFFSET             19
#define MXC_DVFSCNTR_MAXF_MASK               0x00040000
#define MXC_DVFSCNTR_MAXF_OFFSET             18
#define MXC_DVFSCNTR_MINF_MASK               0x00020000
#define MXC_DVFSCNTR_MINF_OFFSET             17
#define MXC_DVFSCNTR_LTBRSR_MASK             0x00000018
#define MXC_DVFSCNTR_LTBRSR_OFFSET           3
#define MXC_DVFSCNTR_DVFEN                   0x00000001

#define MXC_GPCCNTR_GPCIRQ                   0x00100000
#define MXC_GPCCNTR_DVFS0CR                  0x00010000
#define MXC_GPCCNTR_ADU                      0x00008000
#define MXC_GPCCNTR_STRT                     0x00004000
#define MXC_GPCCNTR_FUPD                     0x00002000
#define MXC_GPCCNTR_HTRI_MASK                0x0000000F
#define MXC_GPCCNTR_HTRI_OFFSET              0

#define MXC_GPCVCR_VINC_MASK                 0x00020000
#define MXC_GPCVCR_VINC_OFFSET               17
#define MXC_GPCVCR_VCNTU_MASK                0x00010000
#define MXC_GPCVCR_VCNTU_OFFSET              16
#define MXC_GPCVCR_VCNT_MASK                 0x00007FFF
#define MXC_GPCVCR_VCNT_OFFSET               0

static struct delayed_work dvfs_core_work;
static struct mxc_dvfs_platform_data *dvfs_data;
static struct device *dvfs_dev;
static struct cpu_wp *cpu_wp_tbl;
int dvfs_core_resume;
int curr_wp;
int dvfs_core_is_active;

/*
 * Clock structures
 */
static struct clk *cpu_clk;
static struct clk *dvfs_clk;
static struct regulator *core_regulator;

enum {
	FSVAI_FREQ_NOCHANGE = 0x0,
	FSVAI_FREQ_INCREASE,
	FSVAI_FREQ_DECREASE,
	FSVAI_FREQ_EMERG,
};

/*
 * Load tracking buffer source: 1 for ld_add; 0 for pre_ld_add; 2 for after EMA
 */
#define DVFS_LTBRSR		(2 << MXC_DVFSCNTR_LTBRSR_OFFSET)

DEFINE_SPINLOCK(mxc_dvfs_core_lock);

static void dvfs_load_config(void)
{
	u32 reg;

	reg = 0;
	reg |= dvfs_data->upthr_val << MXC_DVFSTHRS_UPTHR_OFFSET;
	reg |= dvfs_data->dnthr_val << MXC_DVFSTHRS_DNTHR_OFFSET;
	reg |= dvfs_data->pncthr_val;
	__raw_writel(reg, dvfs_data->dvfs_thrs_reg_addr);

	reg = 0;
	reg |= dvfs_data->dncnt_val << MXC_DVFSCOUN_DNCNT_OFFSET;
	reg |= dvfs_data->upcnt_val << MXC_DVFSCOUN_UPCNT_OFFSET;
	__raw_writel(reg, dvfs_data->dvfs_coun_reg_addr);
}

static int start_dvfs(void)
{
	u32 reg, flags;

	if (dvfs_core_is_active)
		return 0;

	spin_lock_irqsave(&mxc_dvfs_core_lock, flags);

	clk_enable(dvfs_clk);

	/* config reg GPC_CNTR */
	reg = __raw_readl(dvfs_data->gpc_cntr_reg_addr);

	/* GPCIRQ=1, select ARM IRQ */
	reg |= MXC_GPCCNTR_GPCIRQ;
	/* ADU=1, select ARM domain */
	reg |= MXC_GPCCNTR_ADU;
	__raw_writel(reg, dvfs_data->gpc_cntr_reg_addr);

	/* Enable DVFS interrupt */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	/* FSVAIM=0 */
	reg = (reg & ~MXC_DVFSCNTR_FSVAIM);
	/* Set MAXF, MINF */
	reg = (reg & ~(MXC_DVFSCNTR_MAXF_MASK | MXC_DVFSCNTR_MINF_MASK));
	reg |= 1 << MXC_DVFSCNTR_MAXF_OFFSET;
	/* Select ARM domain */
	reg |= MXC_DVFSCNTR_DVFIS;
	/* Enable DVFS frequency adjustment interrupt */
	reg = (reg & ~MXC_DVFSCNTR_FSVAIM);
	/* Set load tracking buffer register source */
	reg = (reg & ~MXC_DVFSCNTR_LTBRSR_MASK);
	reg |= DVFS_LTBRSR;
	/* Set DIV3CK */
	reg = (reg & ~(dvfs_data->div3ck_mask));
	reg |= (dvfs_data->div3ck_val) << (dvfs_data->div3ck_offset);
	/* Enable DVFS */
	reg |= MXC_DVFSCNTR_DVFEN;
	__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);

	dvfs_core_is_active = 1;

	spin_unlock_irqrestore(&mxc_dvfs_core_lock, flags);

	printk(KERN_DEBUG "DVFS is started\n");

	return 0;
}

/*!
 * This function is called for module initialization.
 * It sets up the DVFS hardware.
 * It sets default values for DVFS thresholds and counters. The default
 * values was chosen from a set of different reasonable values. They was tested
 * and the default values in the driver gave the best results.
 * More work should be done to find optimal values.
 *
 * @return   0 if successful; non-zero otherwise.
 *
 */
static int init_dvfs_controller(void)
{
	/* DVFS loading config */
	dvfs_load_config();

	/* Set EMAC value */
	__raw_writel((dvfs_data->emac_val << MXC_DVFSEMAC_EMAC_OFFSET),
				 dvfs_data->dvfs_emac_reg_addr);

	return 0;
}

static irqreturn_t dvfs_irq(int irq, void *dev_id)
{
	u32 reg;

	/* Check if DVFS0 (ARM) id requesting for freqency/voltage update */
	if ((__raw_readl(dvfs_data->gpc_cntr_reg_addr) & MXC_GPCCNTR_DVFS0CR) ==
	    0)
		return IRQ_NONE;

	/* Mask DVFS irq */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	/* FSVAIM=1 */
	reg |= MXC_DVFSCNTR_FSVAIM;
	__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);

	schedule_delayed_work(&dvfs_core_work, 0);

	return IRQ_HANDLED;
}

static void dvfs_core_workqueue_handler(struct work_struct *work)
{
	u32 fsvai;
	u32 reg;
	u32 curr_cpu;
	unsigned long rate = 0;
	int ret = 0;
	int uvol;
	int maxf = 0, minf = 0;

	/* Check DVFS frequency adjustment interrupt status */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	fsvai = (reg & MXC_DVFSCNTR_FSVAI_MASK) >> MXC_DVFSCNTR_FSVAI_OFFSET;

	/* Check FSVAI, FSVAI=0 is error */
	if (fsvai == FSVAI_FREQ_NOCHANGE) {
		/* Do nothing. Freq change is not required */
		goto END;
	}

	curr_cpu = clk_get_rate(cpu_clk);

	/* If FSVAI indicate freq down,
	   check arm-clk is not in lowest frequency 200 MHz */
	if (fsvai == FSVAI_FREQ_DECREASE) {
		if (curr_cpu == cpu_wp_tbl[dvfs_data->num_wp - 1].cpu_rate) {
			minf = 1;
			goto END;
		} else {
			/* freq down */
			curr_wp++;
			if (curr_wp >= dvfs_data->num_wp) {
				curr_wp = dvfs_data->num_wp - 1;
				goto END;
			}

			rate = cpu_wp_tbl[curr_wp].cpu_rate;
			uvol = cpu_wp_tbl[curr_wp].cpu_voltage;
			if (curr_wp == dvfs_data->num_wp - 1)
				minf = 1;

			ret = clk_set_rate(cpu_clk, rate);
			if (ret != 0) {
				printk(KERN_DEBUG
				       "cannot set CPU clock rate\n");
				goto END;
			}

			/* START the GPC main control FSM */
			/* set VINC */
			reg = __raw_readl(dvfs_data->gpc_vcr_reg_addr);
			reg &=
			    ~(MXC_GPCVCR_VINC_MASK | MXC_GPCVCR_VCNTU_MASK |
			      MXC_GPCVCR_VCNT_MASK);
			reg |=
			    (1 << MXC_GPCVCR_VCNTU_OFFSET) |
			    (100 << MXC_GPCVCR_VCNT_OFFSET);
			__raw_writel(reg, dvfs_data->gpc_vcr_reg_addr);

			/* Set the voltage for the GP domain. */
			ret = regulator_set_voltage(core_regulator, uvol);
			if (ret < 0) {
				printk(KERN_DEBUG
				       "COULD NOT SET CORE VOLTAGE!!!!!\n");
				goto END;
			}
			udelay(dvfs_data->delay_time);
		}
	} else {
		if (curr_cpu == cpu_wp_tbl[0].cpu_rate) {
			maxf = 1;
			goto END;
		} else {
			/* freq up */
			curr_wp = 0;
			rate = cpu_wp_tbl[curr_wp].cpu_rate;
			/* START the GPC main control FSM */
			/* set VINC */
			reg = __raw_readl(dvfs_data->gpc_vcr_reg_addr);
			reg &=
			    ~(MXC_GPCVCR_VINC_MASK | MXC_GPCVCR_VCNTU_MASK |
			      MXC_GPCVCR_VCNT_MASK);
			reg |=
			    (1 << MXC_GPCVCR_VCNTU_OFFSET) |
			    (100 << MXC_GPCVCR_VCNT_OFFSET);
			__raw_writel(reg, dvfs_data->gpc_vcr_reg_addr);

			ret = regulator_set_voltage(core_regulator,
						    cpu_wp_tbl[curr_wp].
						    cpu_voltage);
			if (ret < 0) {
				printk(KERN_DEBUG
				       "COULD NOT SET CORE VOLTAGE!!!!\n");
				goto END;
			}
			udelay(dvfs_data->delay_time);

			ret =
			    clk_set_rate(cpu_clk, cpu_wp_tbl[curr_wp].cpu_rate);
			if (ret != 0)
				printk(KERN_DEBUG
				       "cannot set CPU clock rate\n");
			maxf = 1;
		}
	}

END:			/* Set MAXF, MINF */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	reg = (reg & ~(MXC_DVFSCNTR_MAXF_MASK | MXC_DVFSCNTR_MINF_MASK));
	reg |= maxf << MXC_DVFSCNTR_MAXF_OFFSET;
	reg |= minf << MXC_DVFSCNTR_MINF_OFFSET;

	/* Enable DVFS interrupt */
	/* FSVAIM=0 */
	reg = (reg & ~MXC_DVFSCNTR_FSVAIM);
	/* LBFL=1 */
	reg = (reg & ~MXC_DVFSCNTR_LBFL);
	reg |= MXC_DVFSCNTR_LBFL;
	__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);
}

/*!
 * This function disables the DVFS module.
 */
static void stop_dvfs(void)
{
	u32 reg = 0;
	u32 flags;
	u32 curr_cpu;

	if (dvfs_core_is_active) {
		spin_lock_irqsave(&mxc_dvfs_core_lock, flags);

		/* Mask dvfs irq, disable DVFS */
		reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
		/* FSVAIM=1 */
		reg |= MXC_DVFSCNTR_FSVAIM;
		reg = (reg & ~MXC_DVFSCNTR_DVFEN);
		__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);

		dvfs_core_is_active = 0;
		spin_unlock_irqrestore(&mxc_dvfs_core_lock, flags);

		curr_wp = 0;
		curr_cpu = clk_get_rate(cpu_clk);
		if (curr_cpu != cpu_wp_tbl[curr_wp].cpu_rate) {
			if (regulator_set_voltage(core_regulator,
						  cpu_wp_tbl[curr_wp].
						  cpu_voltage) == 0)
				clk_set_rate(cpu_clk,
					     cpu_wp_tbl[curr_wp].cpu_rate);
		}

		clk_disable(dvfs_clk);
	}

	printk(KERN_DEBUG "DVFS is stopped\n");
}

static ssize_t dvfs_enable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	if (dvfs_core_is_active)
		return sprintf(buf, "DVFS is enabled\n");
	else
		return sprintf(buf, "DVFS is disabled\n");
}


static ssize_t dvfs_enable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL) {
		if (start_dvfs() != 0)
			printk(KERN_ERR "Failed to start DVFS\n");
	} else if (strstr(buf, "0") != NULL)
		stop_dvfs();

	return size;
}

static DEVICE_ATTR(enable, 0644, dvfs_enable_show, dvfs_enable_store);

/*!
 * This is the probe routine for the DVFS driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 */
static int __devinit mxc_dvfs_core_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	int cpu_wp_nr;
	int irq;

	printk(KERN_INFO "mxc_dvfs_core_probe\n");
	dvfs_dev = &pdev->dev;
	dvfs_data = pdev->dev.platform_data;

	INIT_DELAYED_WORK(&dvfs_core_work, dvfs_core_workqueue_handler);

	cpu_clk = clk_get(NULL, dvfs_data->clk1_id);
	if (IS_ERR(cpu_clk)) {
		printk(KERN_ERR "%s: failed to get cpu clock\n", __func__);
		return PTR_ERR(cpu_clk);
	}

	dvfs_clk = clk_get(NULL, dvfs_data->clk2_id);
	if (IS_ERR(dvfs_clk)) {
		printk(KERN_ERR "%s: failed to get dvfs clock\n", __func__);
		return PTR_ERR(dvfs_clk);
	}

	core_regulator = regulator_get(NULL, dvfs_data->reg_id);
	if (IS_ERR(core_regulator)) {
		clk_put(cpu_clk);
		clk_put(dvfs_clk);
		printk(KERN_ERR "%s: failed to get gp regulator\n", __func__);
		return PTR_ERR(core_regulator);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		err = -ENODEV;
		goto err1;
	}

	/*
	 * Request the DVFS interrupt
	 */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err = irq;
		goto err1;
	}

	/* request the DVFS interrupt */
	err = request_irq(irq, dvfs_irq, IRQF_SHARED, "dvfs", dvfs_dev);
	if (err)
		printk(KERN_ERR
		       "DVFS: Unable to attach to DVFS interrupt,err = %d",
		       err);

	clk_enable(dvfs_clk);
	err = init_dvfs_controller();
	if (err) {
		printk(KERN_ERR "DVFS: Unable to initialize DVFS");
		return err;
	}
	clk_disable(dvfs_clk);

	err = sysfs_create_file(&dvfs_dev->kobj, &dev_attr_enable.attr);
	if (err) {
		printk(KERN_ERR
		       "DVFS: Unable to register sysdev entry for DVFS");
		return err;
	}

	/* Set the current working point. */
	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	curr_wp = 0;
	dvfs_core_resume = 0;

	return err;

err1:
	dev_err(&pdev->dev, "Failed to probe DVFS CORE\n");
	return err;
}

/*!
 * This function is called to put DVFS in a low power state.
 *
 * @param   pdev  the device structure
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_dvfs_core_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	if (dvfs_core_is_active) {
		dvfs_core_resume = 1;
		stop_dvfs();
	}

	return 0;
}

/*!
 * This function is called to resume the MU from a low power state.
 *
 * @param   dev   the device structure
 * @param   level the stage in device suspension process that we want the
 *                device to be put in
 *
 * @return  The function always returns 0.
 */
static int mxc_dvfs_core_resume(struct platform_device *pdev)
{
	if (dvfs_core_resume) {
		dvfs_core_resume = 0;
		start_dvfs();
	}

	return 0;
}

static struct platform_driver mxc_dvfs_core_driver = {
	.driver = {
		   .name = "mxc_dvfs_core",
		   },
	.probe = mxc_dvfs_core_probe,
	.suspend = mxc_dvfs_core_suspend,
	.resume = mxc_dvfs_core_resume,
};

static int __init dvfs_init(void)
{
	if (platform_driver_register(&mxc_dvfs_core_driver) != 0) {
		printk(KERN_ERR "mxc_dvfs_core_driver register failed\n");
		return -ENODEV;
	}

	printk(KERN_INFO "DVFS driver module loaded\n");

	return 0;
}

static void __exit dvfs_cleanup(void)
{
	stop_dvfs();

	/* release the DVFS interrupt */
	free_irq(MXC_INT_GPC1, NULL);

	sysfs_remove_file(&dvfs_dev->kobj, &dev_attr_enable.attr);

	/* Unregister the device structure */
	platform_driver_unregister(&mxc_dvfs_core_driver);

	clk_put(cpu_clk);
	clk_put(dvfs_clk);

	printk(KERN_INFO "DVFS driver module unloaded\n");

}

module_init(dvfs_init);
module_exit(dvfs_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("DVFS driver");
MODULE_LICENSE("GPL");

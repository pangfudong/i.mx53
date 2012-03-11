/*
 * Backup battery driver for Wolfson Microelectronics wm831x PMICs
 *
 * Copyright 2009 Wolfson Microelectronics PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/auxadc.h>
#include <linux/mfd/wm831x/pmu.h>
#include <linux/mfd/wm831x/pdata.h>
#include <mach/gpio.h>

#define MX53_SMD_CAP_TCH_INT1	(2*32 + 20)/* GPIO_3_20 */
#define MX53_SMD_DC_DET		(1*32 + 23)/* GPIO_2_23 */
#define WM831X_DELAY	1000
#define STATUS_CHARGING 0

struct wm831x_backup {
	struct wm831x *wm831x;
	struct power_supply backup;
	struct delayed_work work;
	int voltage_uV;
	int cap;
	int online;
	int status;
};

/*
static int wm831x_backup_read_voltage(struct wm831x *wm831x,
				     enum wm831x_auxadc src,
				     union power_supply_propval *val)
{
	int ret;

	ret = wm831x_auxadc_read_uv(wm831x, src);
	if (ret >= 0)
		val->intval = ret;

	return ret;
}
*/

/*********************************************************************
 *		Backup supply properties
 *********************************************************************/

static void wm831x_config_backup(struct wm831x *wm831x)
{
	struct wm831x_pdata *wm831x_pdata = wm831x->dev->platform_data;
	struct wm831x_backup_pdata *pdata;
	int ret, reg;

	if (!wm831x_pdata || !wm831x_pdata->backup) {
		dev_warn(wm831x->dev,
			 "No backup battery charger configuration\n");
		return;
	}

	pdata = wm831x_pdata->backup;

	reg = 0;

	if (pdata->charger_enable)
		reg |= WM831X_BKUP_CHG_ENA | WM831X_BKUP_BATT_DET_ENA;
	if (pdata->no_constant_voltage)
		reg |= WM831X_BKUP_CHG_MODE;

	switch (pdata->vlim) {
	case 2500:
		break;
	case 3100:
		reg |= WM831X_BKUP_CHG_VLIM;
		break;
	default:
		dev_err(wm831x->dev, "Invalid backup voltage limit %dmV\n",
			pdata->vlim);
	}

	switch (pdata->ilim) {
	case 100:
		break;
	case 200:
		reg |= 1;
		break;
	case 300:
		reg |= 2;
		break;
	case 400:
		reg |= 3;
		break;
	default:
		dev_err(wm831x->dev, "Invalid backup current limit %duA\n",
			pdata->ilim);
	}

	ret = wm831x_reg_unlock(wm831x);
	if (ret != 0) {
		dev_err(wm831x->dev, "Failed to unlock registers: %d\n", ret);
		return;
	}

	ret = wm831x_set_bits(wm831x, WM831X_BACKUP_CHARGER_CONTROL,
			      WM831X_BKUP_CHG_ENA_MASK |
			      WM831X_BKUP_CHG_MODE_MASK |
			      WM831X_BKUP_BATT_DET_ENA_MASK |
			      WM831X_BKUP_CHG_VLIM_MASK |
			      WM831X_BKUP_CHG_ILIM_MASK,
			      reg);
	if (ret != 0)
		dev_err(wm831x->dev,
			"Failed to set backup charger config: %d\n", ret);

	wm831x_reg_lock(wm831x);
}
#define BATT_EMPTY_MV          3300
#define BATT_FULL_MV           4200
static void wm831x_get_cap(struct wm831x_backup *devdata)
{
	int old_cap = devdata->cap;

	devdata->voltage_uV =  wm831x_auxadc_read_uv(devdata->wm831x, WM831X_AUX_BKUP_BATT);
	if (devdata->voltage_uV > BATT_EMPTY_MV*1000) {
		devdata->cap = (devdata->voltage_uV/1000 - BATT_EMPTY_MV) * 100/
				(BATT_FULL_MV - BATT_EMPTY_MV);
		if (devdata->cap > 100)
			devdata->cap = 100;
	} else
		devdata->cap = 0;

	if (devdata->cap != old_cap)
		power_supply_changed(&devdata->backup);
}

static irqreturn_t wm831x_update_status(int irq, void *dev_id)
{
	struct wm831x_backup *devdata = dev_id;
	int old_status = devdata->status;
	int flags = gpio_get_value(MX53_SMD_CAP_TCH_INT1);

	devdata->online = gpio_get_value(MX53_SMD_DC_DET);
	if (devdata->online) {
		if (flags == STATUS_CHARGING &&
				devdata->voltage_uV/1000 < BATT_FULL_MV) {
			devdata->status =
				POWER_SUPPLY_STATUS_CHARGING;
		}
		else
			devdata->status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else
		devdata->status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (devdata->voltage_uV/1000 >= BATT_FULL_MV)
		devdata->status = POWER_SUPPLY_STATUS_FULL;

	if (old_status != devdata->status)
		power_supply_changed(&devdata->backup);

	return IRQ_HANDLED;
}

static void wm831x_work(struct work_struct *work)
{
	struct wm831x_backup *devdata = container_of(work,
					struct wm831x_backup, work.work);

	wm831x_get_cap(devdata);
	schedule_delayed_work(&devdata->work, WM831X_DELAY);
}

static int wm831x_backup_get_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct wm831x_backup *devdata = dev_get_drvdata(psy->dev->parent);
	struct wm831x *wm831x = devdata->wm831x;
	int ret = 0;

	ret = wm831x_reg_read(wm831x, WM831X_BACKUP_CHARGER_CONTROL);
	if (ret < 0)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
			val->intval = devdata->status;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = devdata->voltage_uV;
		return ret;

	case POWER_SUPPLY_PROP_PRESENT:
		if (ret & WM831X_BKUP_CHG_STS)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = devdata->cap;
		return ret;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property wm831x_backup_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
};

/*********************************************************************
 *		Initialisation
 *********************************************************************/

static __devinit int wm831x_backup_probe(struct platform_device *pdev)
{
	struct wm831x *wm831x = dev_get_drvdata(pdev->dev.parent);
	struct wm831x_backup *devdata;
	struct power_supply *backup;
	int ret;
	int irq = gpio_to_irq(MX53_SMD_DC_DET);

	devdata = kzalloc(sizeof(struct wm831x_backup), GFP_KERNEL);
	if (devdata == NULL)
		return -ENOMEM;

	devdata->wm831x = wm831x;
	platform_set_drvdata(pdev, devdata);

	backup = &devdata->backup;

	/* We ignore configuration failures since we can still read
	 * back the status without enabling the charger (which may
	 * already be enabled anyway).
	 */
	wm831x_config_backup(wm831x);

	backup->name = "wm831x-backup";
	backup->type = POWER_SUPPLY_TYPE_BATTERY;
	backup->properties = wm831x_backup_props;
	backup->num_properties = ARRAY_SIZE(wm831x_backup_props);
	backup->get_property = wm831x_backup_get_prop;
	ret = power_supply_register(&pdev->dev, backup);
	if (ret)
		goto err_kmalloc;

	wm831x_update_status(irq, devdata);
	wm831x_get_cap(devdata);

	INIT_DELAYED_WORK_DEFERRABLE(&devdata->work, wm831x_work);
	schedule_delayed_work(&devdata->work, WM831X_DELAY);

	ret = request_irq(irq, wm831x_update_status, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, backup->name, devdata);
	if (ret != 0) {
		free_irq(irq, devdata);
		printk("request irq fail\n");
	}

	return ret;

err_kmalloc:
	kfree(devdata);
	return ret;
}

static __devexit int wm831x_backup_remove(struct platform_device *pdev)
{
	struct wm831x_backup *devdata = platform_get_drvdata(pdev);

	power_supply_unregister(&devdata->backup);
	kfree(devdata);

	return 0;
}

static struct platform_driver wm831x_backup_driver = {
	.probe = wm831x_backup_probe,
	.remove = __devexit_p(wm831x_backup_remove),
	.driver = {
		.name = "wm831x-backup",
	},
};

static int __init wm831x_backup_init(void)
{
	return platform_driver_register(&wm831x_backup_driver);
}
module_init(wm831x_backup_init);

static void __exit wm831x_backup_exit(void)
{
	platform_driver_unregister(&wm831x_backup_driver);
}
module_exit(wm831x_backup_exit);

MODULE_DESCRIPTION("Backup battery charger driver for WM831x PMICs");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:wm831x-backup");

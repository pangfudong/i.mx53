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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/regulator/regulator-platform.h>
#include <linux/regulator/regulator-drv.h>
#include <linux/ioctl.h>
#include <linux/pmic_status.h>
#include <linux/platform_device.h>
#include <linux/regulator/mcu_max8660-bus.h>

enum {
	MC9SDZ60_LCD,
	MC9SDZ60_WIFI,
	MC9SDZ60_HDD,
	MC9SDZ60_GPS,
	MC9SDZ60_SPKR,
} MC9SDZ60_regulator;

#define NUM_MC9SDZ60_REGULATORS 5

/* lcd */
static int mc9sdz60_lcd_enable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 6, 1);
}

static int mc9sdz60_lcd_disable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 6, 0);
}

static struct regulator_ops mc9sdz60_lcd_ops = {
	.enable = mc9sdz60_lcd_enable,
	.disable = mc9sdz60_lcd_disable,
};

/* wifi */
static int mc9sdz60_wifi_enable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 5, 1);
}

static int mc9sdz60_wifi_disable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 5, 0);
}

static struct regulator_ops mc9sdz60_wifi_ops = {
	.enable = mc9sdz60_wifi_enable,
	.disable = mc9sdz60_wifi_disable,
};

/* hdd */
static int mc9sdz60_hdd_enable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 4, 1);
}

static int mc9sdz60_hdd_disable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 4, 0);
}

static struct regulator_ops mc9sdz60_hdd_ops = {
	.enable = mc9sdz60_hdd_enable,
	.disable = mc9sdz60_hdd_disable,
};

/* gps */
static int mc9sdz60_gps_enable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 0, 1);
}

static int mc9sdz60_gps_disable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 0, 0);
}

static struct regulator_ops mc9sdz60_gps_ops = {
	.enable = mc9sdz60_gps_enable,
	.disable = mc9sdz60_gps_disable,
};

/* speaker */
static int mc9sdz60_speaker_enable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 0, 1);
}

static int mc9sdz60_speaker_disable(struct regulator *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 0, 0);
}

static struct regulator_ops mc9sdz60_speaker_ops = {
	.enable = mc9sdz60_speaker_enable,
	.disable = mc9sdz60_speaker_disable,
};

struct mc9sdz60_regulator {
	struct regulator regulator;
};

static struct mc9sdz60_regulator
    reg_mc9sdz60[NUM_MC9SDZ60_REGULATORS] = {
	{
	 .regulator = {
		       .name = "LCD",
		       .id = MC9SDZ60_LCD,
		       .ops = &mc9sdz60_lcd_ops,
		       },
	 },
	{
	 .regulator = {
		       .name = "WIFI",
		       .id = MC9SDZ60_WIFI,
		       .ops = &mc9sdz60_wifi_ops,
		       },
	 },
	{
	 .regulator = {
		       .name = "HDD",
		       .id = MC9SDZ60_HDD,
		       .ops = &mc9sdz60_hdd_ops,
		       },
	 },
	{
	 .regulator = {
		       .name = "GPS",
		       .id = MC9SDZ60_GPS,
		       .ops = &mc9sdz60_gps_ops,
		       },

	 },
	{
	 .regulator = {
		       .name = "SPKR",
		       .id = MC9SDZ60_SPKR,
		       .ops = &mc9sdz60_speaker_ops,
		       },

	 },

};


/*
 * Init and Exit
 */
int reg_mc9sdz60_probe(void)
{
	int ret11 = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(reg_mc9sdz60); i++) {
		ret11 = regulator_register(&reg_mc9sdz60[i].regulator);
		regulator_set_platform_constraints(reg_mc9sdz60[i].regulator.
						   name,
						   reg_mc9sdz60[i].regulator.
						   constraints);
		if (ret11 < 0) {
			i--;
			for (; i >= 0; i--)
				regulator_unregister(
					&reg_mc9sdz60[i].regulator);

			return ret11;
		}

	}
	return 0;
}
EXPORT_SYMBOL(reg_mc9sdz60_probe);

int reg_mc9sdz60_remove(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(reg_mc9sdz60); i++)
		regulator_unregister(&reg_mc9sdz60[i].regulator);
	return 0;
}
EXPORT_SYMBOL(reg_mc9sdz60_remove);



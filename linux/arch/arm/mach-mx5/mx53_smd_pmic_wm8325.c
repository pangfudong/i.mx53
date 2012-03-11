/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


/*
 * mx53_smd_pmic_wm8325.c  --  i.MX53 SMD driver for pmic wm8325
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/pdata.h>
#include <mach/irqs.h>
#include <mach/iomux-mx53.h>
#include <mach/gpio.h>


#define MX53_SMD_WM_8325_GPIO3		(2*32 + 29)	/* GPIO_3_29, DVS1 function */

/* VCC DC1 */
static struct regulator_init_data wm8325_dc1 = {
	.constraints = {
		.name = "DCDC1",
		.min_uV = 600000,
		.max_uV = 1300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.enabled = 1,
		.uV = 950000,
		},
	},
};

/* VDDGP DC2 */
static struct regulator_init_data wm8325_dc2 = {
	.constraints = {
		.name = "DCDC2",
		.min_uV =  600000,
		.max_uV = 1300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.enabled = 1,
		.uV = 850000,
		},
	},
};

/* VDDAL 1V3 DC3 */
static struct regulator_init_data wm8325_dc3 = {
	.constraints = {
		.name = "DCDC3",
		.min_uV = 850000,
		.max_uV = 3400000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.enabled = 1,
		.uV = 950000,
		},
	},
};

/* DDR_1V5 DC4 */
static struct regulator_init_data wm8325_dc4 = {
	.constraints = {
		.name = "DCDC4",
		.min_uV = 850000,
		.max_uV = 3400000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_NORMAL,
		.enabled = 1,
		.uV = 1550000,
		},
	},
};

/* TVDAC_2V75 LDO1 */
static struct regulator_init_data wm8325_ldo1 = {
	.constraints = {
		.name = "LDO1",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.disabled = 1,
		.uV = 900000,
		},
	},
};

/* VDD_REG_2V5 LDO2 */
static struct regulator_init_data wm8325_ldo2 = {
	.constraints = {
		.name = "LDO2",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.disabled = 1,
		.uV = 900000,
		},
	},
};

/* VUSB_2V5/LVDS_2V5/NVCC_XTAL_2V5 LDO3 */
static struct regulator_init_data wm8325_ldo3 = {
	.constraints = {
		.name = "LDO3",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.disabled = 1,
		.uV = 900000,
		},
	},
};

/* VDD_DIG_PLL 1V3 LDO4 */
static struct regulator_init_data wm8325_ldo4 = {
	.constraints = {
		.name = "LDO4",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_NORMAL,
		.enabled = 1,
		.uV = 1000000,
		},
	},
};

/* SATA_PHY_2V5 LDO5 */
static struct regulator_init_data wm8325_ldo5 = {
	.constraints = {
		.name = "LDO5",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.disabled = 1,
		.uV = 900000,
		},
	},
};

/* VDD_FUSE 3V3 LDO6 */
static struct regulator_init_data wm8325_ldo6 = {
	.constraints = {
		.name = "LDO6",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.disabled = 1,
		.uV = 900000,
		},
	},
};

/*
   NVCC_EIM_MAIN/NVCC_SD1&2/
   NVCC_PATA/NVCC_GPIO/NVCC_FEC
   3V3 LDO7
*/
static struct regulator_init_data wm8325_ldo7 = {
	.constraints = {
		.name = "LDO7",
		.min_uV = 1000000,
		.max_uV = 3500000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_NORMAL,
		.enabled = 1,
		.uV = 3300000,
		},
	},
};

/*
   NVCC_NANDF/NVCC_RESET/NVCC_CSI
   NVCC_JTAG/NVCC_CKIH/VDD_ANA_PLL
   1V8 LDO8
*/
static struct regulator_init_data wm8325_ldo8 = {
	.constraints = {
		.name = "LDO8",
		.min_uV = 1000000,
		.max_uV = 3500000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_NORMAL,
		.enabled = 1,
		.uV = 3300000,
		},
	},
};

/* SATA_1V3 LDO9 */
static struct regulator_init_data wm8325_ldo9 = {
	.constraints = {
		.name = "LDO9",
		.min_uV = 1000000,
		.max_uV = 3500000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.disabled = 1,
		.uV = 1000000,
		},
	},
};

/* NVCC_LCD 2V775 LDO10*/
static struct regulator_init_data wm8325_ldo10 = {
	.constraints = {
		.name = "LDO10",
		.min_uV = 1000000,
		.max_uV = 3500000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_IDLE,
		.disabled = 1,
		.uV = 1000000,
		},
	},
};

/* NVCC_SRTC 1V3 LDO11*/
static struct regulator_init_data wm8325_ldo11 = {
	.constraints = {
		.name = "LDO11",
		.min_uV = 800000,
		.max_uV = 1550000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
		.mode = REGULATOR_MODE_NORMAL,
		.enabled = 1,
		.uV = 1200000,
		},
	},
};

#define MX53_SMD_WM8325_IRQ			(6*32 + 11)	/* GPIO7_11 */
#define WM8325_I2C_ADDR				(0x68)
#define WM8325_I2C_DEVICE_NAME      "wm8325"

#define WM8325_GPIO1_CONFIG_VAL     (0x848a)
#define WM8325_GPIO2_CONFIG_VAL     (0xcc84)
#define WM8325_GPIO3_CONFIG_VAL     (0x8488)
#define WM831X_DC1_CONTROL_2_VAL    (0x0b00)
#define WM831X_DC1_DVS_CONTROL_VAL  (0x1024)
#define WM831X_DC2_CONTROL_2_VAL    (0x0b00)
#define WM831X_DC2_DVS_CONTROL_VAL  (0x101c)
#define WM831X_LDO1_CONTROL_VAL     (0x0A00)

static int wm8325_post_init(struct wm831x *wm831x)
{
	int ret;

	/* Set GPIO1 as input ,active high, Hardware Enable1 Function */
	ret = wm831x_reg_write(wm831x, WM831X_GPIO1_CONTROL, \
			WM8325_GPIO1_CONFIG_VAL);
	if (0 > ret) {
		printk("func:%s, write wm831x gpio1 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set GPIO2 as input ,active high, Hardware Control1 Function */
	ret = wm831x_reg_write(wm831x, WM831X_GPIO2_CONTROL, \
			WM8325_GPIO2_CONFIG_VAL);
	if (0 > ret) {
		printk("func:%s, write wm831x gpio2 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set GPIO3 as input ,active high, DVS1 Function */
	ret = wm831x_reg_write(wm831x, WM831X_GPIO3_CONTROL, \
			WM8325_GPIO3_CONFIG_VAL);
	if (0 > ret) {
		printk("func:%s, write wm831x gpio3 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set DCDC1 hardware controlled by Hardware Control1 */
	ret = wm831x_reg_write(wm831x, WM831X_DC1_CONTROL_2, \
			WM831X_DC1_CONTROL_2_VAL);
	if (0 > ret) {
		printk("func:%s, write wm831x dc1 ctrl2 reg error.\n", __func__);
		goto out;
	}

	/* Set DCDC2 hardware controlled by Hardware Control1 */
	ret = wm831x_reg_write(wm831x, WM831X_DC2_CONTROL_2, \
			WM831X_DC2_CONTROL_2_VAL);
	if (0 > ret) {
		printk("func:%s, write wm831x dc2 ctrl2 reg error.\n", __func__);
		goto out;
	}

	/* Set DCDC1 controlled by DVS1, DC1_DVS_VSEL=0.95V */
	ret = wm831x_reg_write(wm831x, WM831X_DC1_DVS_CONTROL, \
			WM831X_DC1_DVS_CONTROL_VAL);
	if (0 > ret) {
		printk("func:%s, write wm831x dc1 dvs reg error.\n", __func__);
		goto out;
	}

	/* Set DCDC2 controlled by DVS1, DC2_DVS_VSEL=0.85V */
	ret = wm831x_reg_write(wm831x, WM831X_DC2_DVS_CONTROL, \
			WM831X_DC2_DVS_CONTROL_VAL);
	if (0 > ret) {
		printk("func:%s, write wm831x dc2 dvs reg error.\n", __func__);
		goto out;
	}

	/* Set other LDOs hardware controlled by Hardware Control1,
	   and set other LDOs hardware enable function, accroding to board design
	*/
	/* Set LDO1 hardware controlled by Hardware Control1:low-pwr mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO1_CONTROL, \
			WM831X_LDO1_CONTROL_VAL);
	if (0 > ret) {
		printk("func:%s, write wm831x ldo1 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set LDO9 to 3.0V */
	ret = wm831x_reg_write(wm831x, WM831X_LDO9_ON_CONTROL, 0x1A);
	if (0 > ret) {
		printk("func:%s, write wm831x ldo9 ctrl reg error.\n", __func__);
		goto out;
	}
	return 0;

out:
	return ret;
}

static struct wm831x_pdata wm8325_plat= {
	.post_init = wm8325_post_init,
	.irq_base = MXC_BOARD_IRQ_START,

	.dcdc = {
		&wm8325_dc1,
		&wm8325_dc2,
		&wm8325_dc3,
		&wm8325_dc4,
	},
	.ldo = {
		 &wm8325_ldo1,
		 &wm8325_ldo2,
		 &wm8325_ldo3,
		 &wm8325_ldo4,
		 &wm8325_ldo5,
		 &wm8325_ldo6,
		 &wm8325_ldo7,
		 &wm8325_ldo8,
		 &wm8325_ldo9,
		 &wm8325_ldo10,
		 &wm8325_ldo11,
	},
};

static struct i2c_board_info __initdata wm8325_i2c_device = {
	I2C_BOARD_INFO(WM8325_I2C_DEVICE_NAME, WM8325_I2C_ADDR >> 1),
	.irq = gpio_to_irq(MX53_SMD_WM8325_IRQ),
	.platform_data = &wm8325_plat,
};

int __init mx53_smd_init_wm8325(void)
{
	return i2c_register_board_info(0, &wm8325_i2c_device, 1);
}

/*
 * Copyright 2005-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/regulator.h>

#include "mxc_v4l2_capture.h"

enum ov2640_mode {
	ov2640_mode_1600_1120,
	ov2640_mode_800_600
};

struct reg_value {
	u8 reg;
	u8 value;
	int delay_ms;
};

static struct reg_value ov2640_setting_1600_1120[] = {
	{0xff, 0x1, 0}, {0x12, 0x80, 1}, {0xff, 0, 0}, {0x2c, 0xff, 0},
	{0x2e, 0xdf, 0}, {0xff, 0x1, 0}, {0x3c, 0x32, 0}, {0x11, 0x01, 0},
	{0x09, 0x00, 0}, {0x04, 0x28, 0}, {0x13, 0xe5, 0}, {0x14, 0x48, 0},
	{0x2c, 0x0c, 0}, {0x33, 0x78, 0}, {0x3a, 0x33, 0}, {0x3b, 0xfb, 0},
	{0x3e, 0x00, 0}, {0x43, 0x11, 0}, {0x16, 0x10, 0}, {0x39, 0x82, 0},
	{0x35, 0x88, 0}, {0x22, 0x0a, 0}, {0x37, 0x40, 0}, {0x23, 0x00, 0},
	{0x34, 0xa0, 0}, {0x36, 0x1a, 0}, {0x06, 0x02, 0}, {0x07, 0xc0, 0},
	{0x0d, 0xb7, 0}, {0x0e, 0x01, 0}, {0x4c, 0x00, 0}, {0x4a, 0x81, 0},
	{0x21, 0x99, 0}, {0x24, 0x40, 0}, {0x25, 0x38, 0}, {0x26, 0x82, 0},
	{0x5c, 0x00, 0}, {0x63, 0x00, 0}, {0x46, 0x3f, 0}, {0x0c, 0x3c, 0},
	{0x5d, 0x55, 0}, {0x5e, 0x7d, 0}, {0x5f, 0x7d, 0}, {0x60, 0x55, 0},
	{0x61, 0x70, 0}, {0x62, 0x80, 0}, {0x7c, 0x05, 0}, {0x20, 0x80, 0},
	{0x28, 0x30, 0}, {0x6c, 0x00, 0}, {0x6d, 0x80, 0}, {0x6e, 00, 0},
	{0x70, 0x02, 0}, {0x71, 0x94, 0}, {0x73, 0xc1, 0}, {0x3d, 0x34, 0},
	{0x5a, 0x57, 0}, {0x4f, 0xbb, 0}, {0x50, 0x9c, 0}, {0xff, 0x00, 0},
	{0xe5, 0x7f, 0}, {0xf9, 0xc0, 0}, {0x41, 0x24, 0}, {0x44, 0x06, 0},
	{0xe0, 0x14, 0}, {0x76, 0xff, 0}, {0x33, 0xa0, 0}, {0x42, 0x20, 0},
	{0x43, 0x18, 0}, {0x4c, 0x00, 0}, {0x87, 0xd0, 0}, {0xd7, 0x03, 0},
	{0xd9, 0x10, 0}, {0xd3, 0x82, 0}, {0xc8, 0x08, 0}, {0xc9, 0x80, 0},
	{0x7c, 0x00, 0}, {0x7d, 0x00, 0}, {0x7c, 0x03, 0}, {0x7d, 0x48, 0},
	{0x7d, 0x48, 0}, {0x7c, 0x08, 0}, {0x7d, 0x20, 0}, {0x7d, 0x10, 0},
	{0x7d, 0x0e, 0}, {0x90, 0x00, 0}, {0x91, 0x0e, 0}, {0x91, 0x1a, 0},
	{0x91, 0x31, 0}, {0x91, 0x5a, 0}, {0x91, 0x69, 0}, {0x91, 0x75, 0},
	{0x91, 0x7e, 0}, {0x91, 0x88, 0}, {0x91, 0x8f, 0}, {0x91, 0x96, 0},
	{0x91, 0xa3, 0}, {0x91, 0xaf, 0}, {0x91, 0xc4, 0}, {0x91, 0xd7, 0},
	{0x91, 0xe8, 0}, {0x91, 0x20, 0}, {0x92, 0x00, 0}, {0x93, 0x06, 0},
	{0x93, 0xe3, 0}, {0x93, 0x03, 0}, {0x93, 0x03, 0}, {0x93, 0x00, 0},
	{0x93, 0x02, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x96, 0x00, 0}, {0x97, 0x08, 0}, {0x97, 0x19, 0}, {0x97, 0x02, 0},
	{0x97, 0x0c, 0}, {0x97, 0x24, 0}, {0x97, 0x30, 0}, {0x97, 0x28, 0},
	{0x97, 0x26, 0}, {0x97, 0x02, 0}, {0x97, 0x98, 0}, {0x97, 0x80, 0},
	{0x97, 0x00, 0}, {0x97, 0x00, 0}, {0xa4, 0x00, 0}, {0xa8, 0x00, 0},
	{0xc5, 0x11, 0}, {0xc6, 0x51, 0}, {0xbf, 0x80, 0}, {0xc7, 0x10, 0},
	{0xb6, 0x66, 0}, {0xb8, 0xa5, 0}, {0xb7, 0x64, 0}, {0xb9, 0x7c, 0},
	{0xb3, 0xaf, 0}, {0xb4, 0x97, 0}, {0xb5, 0xff, 0}, {0xb0, 0xc5, 0},
	{0xb1, 0x94, 0}, {0xb2, 0x0f, 0}, {0xc4, 0x5c, 0}, {0xa6, 0x00, 0},
	{0xa7, 0x20, 0}, {0xa7, 0xd8, 0}, {0xa7, 0x1b, 0}, {0xa7, 0x31, 0},
	{0xa7, 0x00, 0}, {0xa7, 0x18, 0}, {0xa7, 0x20, 0}, {0xa7, 0xd8, 0},
	{0xa7, 0x19, 0}, {0xa7, 0x31, 0}, {0xa7, 0x00, 0}, {0xa7, 0x18, 0},
	{0xa7, 0x20, 0}, {0xa7, 0xd8, 0}, {0xa7, 0x19, 0}, {0xa7, 0x31, 0},
	{0xa7, 0x00, 0}, {0xa7, 0x18, 0}, {0xc0, 0xc8, 0}, {0xc1, 0x96, 0},
	{0x86, 0x3d, 0}, {0x50, 0x00, 0}, {0x51, 0x90, 0}, {0x52, 0x18, 0},
	{0x53, 0x00, 0}, {0x54, 0x00, 0}, {0x55, 0x88, 0}, {0x57, 0x00, 0},
	{0x5a, 0x90, 0}, {0x5b, 0x18, 0}, {0x5c, 0x05, 0}, {0xc3, 0xef, 0},
	{0x7f, 0x00, 0}, {0xda, 0x01, 0}, {0xe5, 0x1f, 0}, {0xe1, 0x67, 0},
	{0xe0, 0x00, 0}, {0xdd, 0x7f, 0}, {0x05, 0x00, 0}
};

static struct reg_value ov2640_setting_800_600[] = {
	{0xff, 0, 0}, {0xff, 1, 0}, {0x12, 0x80, 1}, {0xff, 00, 0},
	{0x2c, 0xff, 0}, {0x2e, 0xdf, 0}, {0xff, 0x1, 0}, {0x3c, 0x32, 0},
	{0x11, 0x01, 0}, {0x09, 0x00, 0}, {0x04, 0x28, 0}, {0x13, 0xe5, 0},
	{0x14, 0x48, 0}, {0x2c, 0x0c, 0}, {0x33, 0x78, 0}, {0x3a, 0x33, 0},
	{0x3b, 0xfb, 0}, {0x3e, 0x00, 0}, {0x43, 0x11, 0}, {0x16, 0x10, 0},
	{0x39, 0x92, 0}, {0x35, 0xda, 0}, {0x22, 0x1a, 0}, {0x37, 0xc3, 0},
	{0x23, 0x00, 0}, {0x34, 0xc0, 0}, {0x36, 0x1a, 0}, {0x06, 0x88, 0},
	{0x07, 0xc0, 0}, {0x0d, 0x87, 0}, {0x0e, 0x41, 0}, {0x4c, 0x00, 0},
	{0x4a, 0x81, 0}, {0x21, 0x99, 0}, {0x24, 0x40, 0}, {0x25, 0x38, 0},
	{0x26, 0x82, 0}, {0x5c, 0x00, 0}, {0x63, 0x00, 0}, {0x46, 0x22, 0},
	{0x0c, 0x3c, 0}, {0x5d, 0x55, 0}, {0x5e, 0x7d, 0}, {0x5f, 0x7d, 0},
	{0x60, 0x55, 0}, {0x61, 0x70, 0}, {0x62, 0x80, 0}, {0x7c, 0x05, 0},
	{0x20, 0x80, 0}, {0x28, 0x30, 0}, {0x6c, 0x00, 0}, {0x6d, 0x80, 0},
	{0x6e, 00, 0}, {0x70, 0x02, 0}, {0x71, 0x94, 0}, {0x73, 0xc1, 0},
	{0x12, 0x40, 0}, {0x17, 0x11, 0}, {0x18, 0x43, 0}, {0x19, 0x00, 0},
	{0x1a, 0x4b, 0}, {0x32, 0x09, 0}, {0x37, 0xc0, 0}, {0x4f, 0xca, 0},
	{0x50, 0xa8, 0}, {0x6d, 0x00, 0}, {0x3d, 0x38, 0}, {0xff, 0x00, 0},
	{0xe5, 0x7f, 0}, {0xf9, 0xc0, 0}, {0x41, 0x24, 0}, {0x44, 0x06, 0},
	{0xe0, 0x14, 0}, {0x76, 0xff, 0}, {0x33, 0xa0, 0}, {0x42, 0x20, 0},
	{0x43, 0x18, 0}, {0x4c, 0x00, 0}, {0x87, 0xd0, 0}, {0x88, 0x3f, 0},
	{0xd7, 0x03, 0}, {0xd9, 0x10, 0}, {0xd3, 0x82, 0}, {0xc8, 0x08, 0},
	{0xc9, 0x80, 0}, {0x7c, 0x00, 0}, {0x7d, 0x00, 0}, {0x7c, 0x03, 0},
	{0x7d, 0x48, 0}, {0x7d, 0x48, 0}, {0x7c, 0x08, 0}, {0x7d, 0x20, 0},
	{0x7d, 0x10, 0}, {0x7d, 0x0e, 0}, {0x90, 0x00, 0}, {0x91, 0x0e, 0},
	{0x91, 0x1a, 0}, {0x91, 0x31, 0}, {0x91, 0x5a, 0}, {0x91, 0x69, 0},
	{0x91, 0x75, 0}, {0x91, 0x7e, 0}, {0x91, 0x88, 0}, {0x91, 0x8f, 0},
	{0x91, 0x96, 0}, {0x91, 0xa3, 0}, {0x91, 0xaf, 0}, {0x91, 0xc4, 0},
	{0x91, 0xd7, 0}, {0x91, 0xe8, 0}, {0x91, 0x20, 0}, {0x92, 0x00, 0},
	{0x93, 0x06, 0}, {0x93, 0xe3, 0}, {0x93, 0x03, 0}, {0x93, 0x03, 0},
	{0x93, 0x00, 0}, {0x93, 0x02, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x93, 0x00, 0}, {0x96, 0x00, 0}, {0x97, 0x08, 0}, {0x97, 0x19, 0},
	{0x97, 0x02, 0}, {0x97, 0x0c, 0}, {0x97, 0x24, 0}, {0x97, 0x30, 0},
	{0x97, 0x28, 0}, {0x97, 0x26, 0}, {0x97, 0x02, 0}, {0x97, 0x98, 0},
	{0x97, 0x80, 0}, {0x97, 0x00, 0}, {0x97, 0x00, 0}, {0xa4, 0x00, 0},
	{0xa8, 0x00, 0}, {0xc5, 0x11, 0}, {0xc6, 0x51, 0}, {0xbf, 0x80, 0},
	{0xc7, 0x10, 0}, {0xb6, 0x66, 0}, {0xb8, 0xa5, 0}, {0xb7, 0x64, 0},
	{0xb9, 0x7c, 0}, {0xb3, 0xaf, 0}, {0xb4, 0x97, 0}, {0xb5, 0xff, 0},
	{0xb0, 0xc5, 0}, {0xb1, 0x94, 0}, {0xb2, 0x0f, 0}, {0xc4, 0x5c, 0},
	{0xa6, 0x00, 0}, {0xa7, 0x20, 0}, {0xa7, 0xd8, 0}, {0xa7, 0x1b, 0},
	{0xa7, 0x31, 0}, {0xa7, 0x00, 0}, {0xa7, 0x18, 0}, {0xa7, 0x20, 0},
	{0xa7, 0xd8, 0}, {0xa7, 0x19, 0}, {0xa7, 0x31, 0}, {0xa7, 0x00, 0},
	{0xa7, 0x18, 0}, {0xa7, 0x20, 0}, {0xa7, 0xd8, 0}, {0xa7, 0x19, 0},
	{0xa7, 0x31, 0}, {0xa7, 0x00, 0}, {0xa7, 0x18, 0}, {0xc0, 0x64, 0},
	{0xc1, 0x4b, 0}, {0x86, 0x1d, 0}, {0x50, 0x00, 0}, {0x51, 0xc8, 0},
	{0x52, 0x96, 0}, {0x53, 0x00, 0}, {0x54, 0x00, 0}, {0x55, 0x00, 0},
	{0x57, 0x00, 0}, {0x5a, 0xc8, 0}, {0x5b, 0x96, 0}, {0x5c, 0x00, 0},
	{0xc3, 0xef, 0}, {0x7f, 0x00, 0}, {0xda, 0x01, 0}, {0xe5, 0x1f, 0},
	{0xe1, 0x67, 0}, {0xe0, 0x00, 0}, {0xdd, 0x7f, 0}, {0x05, 0x00, 0}
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
u32 mclk = 24000000;

struct i2c_client *ov2640_i2c_client;

static sensor_interface *interface_param;
static int reset_frame_rate = 30;
static int ov2640_probe(struct i2c_client *adapter,
			const struct i2c_device_id *id);
static int ov2640_remove(struct i2c_client *client);

static const struct i2c_device_id ov2640_id[] = {
	{"ov2640", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov2640_id);

static struct i2c_driver ov2640_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ov2640",
		   },
	.probe = ov2640_probe,
	.remove = ov2640_remove,
	.id_table = ov2640_id,
};

/*!
 * ov2640 I2C attach function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov2640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct mxc_camera_platform_data *plat_data = client->dev.platform_data;

	ov2640_i2c_client = client;
	mclk = plat_data->mclk;

	io_regulator = regulator_get(&client->dev, plat_data->io_regulator);
	core_regulator = regulator_get(&client->dev, plat_data->core_regulator);
	analog_regulator =
	    regulator_get(&client->dev, plat_data->analog_regulator);
	gpo_regulator = regulator_get(&client->dev, plat_data->gpo_regulator);

	interface_param = (sensor_interface *)
	    kmalloc(sizeof(sensor_interface), GFP_KERNEL);
	if (!interface_param) {
		dev_dbg(&ov2640_i2c_client->dev,
			"ov2640_probe: kmalloc failed \n");
		return -1;
	}

	return 0;
}

/*!
 * ov2640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov2640_remove(struct i2c_client *client)
{
	kfree(interface_param);
	interface_param = NULL;

	if (!IS_ERR_VALUE((unsigned long)io_regulator)) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator, NULL);
	}

	if (!IS_ERR_VALUE((unsigned long)core_regulator)) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator, NULL);
	}

	if (!IS_ERR_VALUE((unsigned long)gpo_regulator)) {
		regulator_disable(gpo_regulator);
		regulator_put(gpo_regulator, NULL);
	}

	if (!IS_ERR_VALUE((unsigned long)analog_regulator)) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator, NULL);
	}

	return 0;
}

static int ov2640_write_reg(u8 reg, u8 val)
{
	if (i2c_smbus_write_byte_data(ov2640_i2c_client, reg, val) < 0) {
		dev_dbg(&ov2640_i2c_client->dev,
			"%s:write reg errorr:reg=%x,val=%x\n", __func__, reg,
			val);
		return -1;
	}
	return 0;
}

static int ov2640_init_mode(enum ov2640_mode mode)
{
	struct reg_value *setting;
	int i, num;

	switch (mode) {
	case ov2640_mode_1600_1120:
		setting = ov2640_setting_1600_1120;
		num = ARRAY_SIZE(ov2640_setting_1600_1120);
		break;
	case ov2640_mode_800_600:
		setting = ov2640_setting_800_600;
		num = ARRAY_SIZE(ov2640_setting_800_600);
		break;
	default:
		return 0;
	}

	for (i = 0; i < num; i++) {
		ov2640_write_reg(setting[i].reg, setting[i].value);
		if (setting[i].delay_ms > 0)
			msleep(setting[i].delay_ms);
	}

	return 0;
}

/*!
 * ov2640 sensor interface Initialization
 * @param param            sensor_interface *
 * @param width            u32
 * @param height           u32
 * @return  None
 */
static void ov2640_interface(sensor_interface *param, u32 width, u32 height)
{
	param->Vsync_pol = 0x0;
	param->clk_mode = 0x0;	/*gated */
	param->pixclk_pol = 0x0;
	param->data_width = 0x1;
	param->data_pol = 0x0;
	param->ext_vsync = 0x0;
	param->Vsync_pol = 0x0;
	param->Hsync_pol = 0x0;
	param->width = width - 1;
	param->height = height - 1;
	param->active_width = width;
	param->active_height = height;
	param->pixel_fmt = IPU_PIX_FMT_UYVY;
	param->mclk = mclk;
}

static void ov2640_set_color(int bright, int saturation, int red, int green,
			     int blue)
{

}

static void ov2640_get_color(int *bright, int *saturation, int *red, int *green,
			     int *blue)
{

}
static void ov2640_set_ae_mode(int ae_mode)
{

}
static void ov2640_get_ae_mode(int *ae_mode)
{

}

extern void gpio_sensor_active(void);

static sensor_interface *ov2640_config(int *frame_rate, int high_quality)
{

	u32 out_width, out_height;

	/*set io votage */
	if (!IS_ERR_VALUE((unsigned long)io_regulator)) {
		regulator_set_voltage(io_regulator, 2800000);
		if (regulator_enable(io_regulator) != 0) {
			dev_dbg(&ov2640_i2c_client->dev,
				"%s:io set voltage error\n", __func__);
			return NULL;
		} else {
			dev_dbg(&ov2640_i2c_client->dev,
				"%s:io set voltage ok\n", __func__);
		}
	}

	/*core votage */
	if (!IS_ERR_VALUE((unsigned long)core_regulator)) {
		regulator_set_voltage(core_regulator, 1300000);
		if (regulator_enable(core_regulator) != 0) {
			dev_dbg(&ov2640_i2c_client->dev,
				"%s:core set voltage error\n", __func__);
			return NULL;
		} else {
			dev_dbg(&ov2640_i2c_client->dev,
				"%s:core set voltage ok\n", __func__);
		}
	}

	/*GPO 3 */
	if (!IS_ERR_VALUE((unsigned long)gpo_regulator)) {
		if (regulator_enable(gpo_regulator) != 0) {
			dev_dbg(&ov2640_i2c_client->dev,
				"%s:gpo3 enable error\n", __func__);
			return NULL;
		} else {
			dev_dbg(&ov2640_i2c_client->dev, "%s:gpo3 enable ok\n",
				__func__);
		}
	}

	if (!IS_ERR_VALUE((unsigned long)analog_regulator)) {
		regulator_set_voltage(analog_regulator, 2000000);
		if (regulator_enable(analog_regulator) != 0) {
			dev_dbg(&ov2640_i2c_client->dev,
				"%s:analog set voltage error\n", __func__);
			return NULL;
		} else {
			dev_dbg(&ov2640_i2c_client->dev,
				"%s:analog set voltage ok\n", __func__);
		}
	}

	gpio_sensor_active();

	if (high_quality) {
		out_width = 1600;
		out_height = 1120;
	} else {
		out_width = 800;
		out_height = 600;
	}
	ov2640_interface(interface_param, out_width, out_height);
	set_mclk_rate(&interface_param->mclk);

	if (high_quality)
		ov2640_init_mode(ov2640_mode_1600_1120);
	else
		ov2640_init_mode(ov2640_mode_800_600);

	msleep(300);

	return interface_param;
}

static sensor_interface *ov2640_reset(void)
{
	return ov2640_config(&reset_frame_rate, 0);
}

struct camera_sensor camera_sensor_if = {
	.set_color = ov2640_set_color,
	.get_color = ov2640_get_color,
	.set_ae_mode = ov2640_set_ae_mode,
	.get_ae_mode = ov2640_get_ae_mode,
	.config = ov2640_config,
	.reset = ov2640_reset,
};

EXPORT_SYMBOL(camera_sensor_if);

/*!
 * ov2640 init function
 *
 * @return  Error code indicating success or failure
 */
static __init int ov2640_init(void)
{
	u8 err;

	err = i2c_add_driver(&ov2640_i2c_driver);

	return err;
}

extern void gpio_sensor_inactive(void);
/*!
 * OV2640 cleanup function
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov2640_clean(void)
{
	i2c_del_driver(&ov2640_i2c_driver);

	gpio_sensor_inactive();
}

module_init(ov2640_init);
module_exit(ov2640_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV2640 Camera Driver");
MODULE_LICENSE("GPL");

/*
 *  mma8451.c - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor
 *
 *  Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>

#define MMA8452_DEVICE_ID    0x2a
#define MMA8452_NAME        "mma8452"    

#define POLL_INTERVAL_MIN    1
#define POLL_INTERVAL_MAX    500
#define POLL_INTERVAL        100 /* msecs */
#define INPUT_FUZZ        32
#define INPUT_FLAT        32
#define MODE_CHANGE_DELAY_MS    100

#define MMA8452_STATUS_ZYXDR    0x08
#define MMA8452_BUF_SIZE    6

/* mma8452 registers */
enum mma8452_registers {
    MMA8452_STATUS = 0x00,
    MMA8452_OUT_X_MSB,
    MMA8452_OUT_X_LSB,
    MMA8452_OUT_Y_MSB,
    MMA8452_OUT_Y_LSB,
    MMA8452_OUT_Z_MSB,
    MMA8452_OUT_Z_LSB,
    MMA8452_SYSMOD = 0x0B,
    MMA8452_INT_SOURCE,
    MMA8452_WHO_AM_I,
    MMA8452_XYZ_DATA_CFG,
    MMA8452_HP_FILTER_CUTOFF,
    MMA8452_PL_STATUS,
    MMA8452_PL_CFG,
    MMA8452_PL_COUNT,
    MMA8452_PL_BF_ZCOMP,
    MMA8452_P_L_THS_REG,
    MMA8452_FF_MT_CFG,
    MMA8452_FF_MT_SRC,
    MMA8452_FF_MT_THS,
    MMA8452_FF_MT_COUNT,
    MMA8452_TRANSIENT_CFG = 0x1D,
    MMA8452_TRANSIENT_SRC,
    MMA8452_TRANSIENT_THS,
    MMA8452_TRANSIENT_COUNT,
    MMA8452_PULSE_CFG,
    MMA8452_PULSE_SRC,
    MMA8452_PULSE_THSX,
    MMA8452_PULSE_THSY,
    MMA8452_PULSE_THSZ,
    MMA8452_PULSE_TMLT,
    MMA8452_PULSE_LTCY,
    MMA8452_PULSE_WIND,
    MMA8452_ASLP_COUNT,
    MMA8452_CTRL_REG1,
    MMA8452_CTRL_REG2,
    MMA8452_CTRL_REG3,
    MMA8452_CTRL_REG4,
    MMA8452_CTRL_REG5,
    MMA8452_OFF_X,
    MMA8452_OFF_Y,
    MMA8452_OFF_Z,
    MMA8452_REG_END,
};


/* MMA8452 have 3 different mode, each mode have different sensitivity
 * as below */
/* MODE_2G: sensitivity is 1024 counts/g
 * MODE_4G: sensitivity is 512 counts/g
 * MODE_8G: sensitivity is 256 counts/g
 */
enum mma8452_mode {
    MODE_2G = 0,
    MODE_4G,
    MODE_8G,
};

struct mma8452_info {
    u8 mode;
    u8 ctl_reg;
    struct i2c_client *client;
    struct input_polled_dev *idev;
    struct device *hwmon_dev;
};

static DEFINE_MUTEX(mma8452_lock);

/* Default use 2G mode */
#define DEFAULT_SENSTIVE_MODE MODE_2G

static int mma8452_change_mode(struct mma8452_info *priv)
{
    int ret;
    priv->ctl_reg |= 0x01;
    ret = i2c_smbus_write_byte_data(priv->client, MMA8452_CTRL_REG1, 0);
    if (ret < 0)
        goto out;
    ret = i2c_smbus_write_byte_data(priv->client,
                    MMA8452_XYZ_DATA_CFG, priv->mode);
    ret |= i2c_smbus_write_byte_data(priv->client,
                     MMA8452_CTRL_REG1, priv->ctl_reg);
    if (ret < 0) {
        dev_err(&priv->client->dev, "mma8452 init error");
        goto out;
    }

    mdelay(MODE_CHANGE_DELAY_MS);
    return 0;
out:
    return ret;
}

static int init_mma8452_chip(struct mma8452_info *priv)
{
    return mma8452_change_mode(priv);
}

static int mma8452_read_data(struct mma8452_info *priv, short *x,
             short *y, short *z)
{
    u8 buf[MMA8452_BUF_SIZE];
    int ret;

    ret = i2c_smbus_read_i2c_block_data(priv->client, MMA8452_OUT_X_MSB,
                     MMA8452_BUF_SIZE, buf);
    if (ret < MMA8452_BUF_SIZE) {
        dev_err(&priv->client->dev, "i2c block read failed\n");
        return -EIO;
    }

    *x = (buf[0] << 8) | buf[1];
    *y = (buf[2] << 8) | buf[3];
    *z = (buf[4] << 8) | buf[5];
    *x >>= 2;
    *y >>= 2;
    *z >>= 2;

    if (priv->mode == MODE_4G) {
        *x <<= 1;
        *y <<= 1;
        *z <<= 1;
    } else if (priv->mode == MODE_8G) {
        *x <<= 2;
        *y <<= 2;
        *z <<= 2;
    }

    return 0;
}

static void mma8452_dev_poll(struct input_polled_dev *dev)
{
    struct mma8452_info *priv = dev->private;

    short x = -1, y = -1, z = -1;
    int ret;

    mutex_lock(&mma8452_lock);

    do
        ret = i2c_smbus_read_byte_data(priv->client, MMA8452_STATUS);
    while (!(ret & MMA8452_STATUS_ZYXDR));

    ret = mma8452_read_data(priv, &x, &y, &z);
    if (!ret) {
        input_report_abs(priv->idev->input, ABS_X, x);
        input_report_abs(priv->idev->input, ABS_Y, y);
        input_report_abs(priv->idev->input, ABS_Z, z);
        input_sync(priv->idev->input);
    }

    mutex_unlock(&mma8452_lock);
}

/*
 * detecting whether mma8452 is on board.
 */
static int is_mma8452_device_id(struct i2c_client *client)
{
    int ret;

    ret = i2c_smbus_read_byte_data(client, MMA8452_WHO_AM_I);

    if (ret == MMA8452_DEVICE_ID)
        return 0;
    else
        return -EINVAL;
}

static int __devinit mma8452_probe(struct i2c_client *client,
                 const struct i2c_device_id *id)
{
    int ret;
    struct input_dev *input_idev;
    struct i2c_adapter *adapter;
    struct mma8452_info *priv;

    adapter = to_i2c_adapter(client->dev.parent);
    ret = i2c_check_functionality(adapter,
                     I2C_FUNC_SMBUS_BYTE |
                     I2C_FUNC_SMBUS_BYTE_DATA);
    if (!ret)
        goto err_out;

    priv = kzalloc(sizeof(struct mma8452_info), GFP_KERNEL);
    if (!priv) {
        dev_err(&client->dev, "failed to alloc driver info\n");
        goto err_out;
    }

    ret = is_mma8452_device_id(client);
    if(ret)
        goto err_out;

    priv->client = client;
    priv->mode = DEFAULT_SENSTIVE_MODE;

    dev_dbg(&client->dev, "found %s model accelerator\n",
         MMA8452_NAME);

    ret = init_mma8452_chip(priv);

    if (ret) {
        dev_err(&client->dev,
            "error when init %s chip:(%d)\n",
            MMA8452_NAME, ret);
        goto err_alloc_priv;
    }

    priv->hwmon_dev = hwmon_device_register(&client->dev);
    if (!priv->hwmon_dev) {
        ret = -ENOMEM;
        dev_err(&client->dev,
            "error register hwmon device\n");
        goto err_alloc_priv;
    }

    priv->idev = input_allocate_polled_device();
    if (!priv->idev) {
        ret = -ENOMEM;
        dev_err(&client->dev, "alloc poll device failed!\n");
        goto err_alloc_poll_device;
    }
    priv->idev->private = priv;
    priv->idev->poll = mma8452_dev_poll;
    priv->idev->poll_interval = POLL_INTERVAL;
    priv->idev->poll_interval_min = POLL_INTERVAL_MIN;
    priv->idev->poll_interval_max = POLL_INTERVAL_MAX;

    input_idev = priv->idev->input;
    input_idev->name = MMA8452_NAME;
    input_idev->id.bustype = BUS_I2C;
    input_idev->evbit[0] = BIT_MASK(EV_ABS);

    input_set_abs_params(input_idev, ABS_X, -8192, 8191,
             INPUT_FUZZ, INPUT_FLAT);
    input_set_abs_params(input_idev, ABS_Y, -8192, 8191,
             INPUT_FUZZ, INPUT_FLAT);
    input_set_abs_params(input_idev, ABS_Z, -8192, 8191,
             INPUT_FUZZ, INPUT_FLAT);

    ret = input_register_polled_device(priv->idev);
    if (ret) {
        dev_err(&client->dev, "register poll device failed!\n");
        goto err_register_polled_device;
    }

    i2c_set_clientdata(client, priv);
    dev_dbg(&client->dev, "%s accelerator init success\n",
        MMA8452_NAME);

    return 0;
err_register_polled_device:
    input_free_polled_device(priv->idev);
err_alloc_poll_device:
    hwmon_device_unregister(&client->dev);
err_alloc_priv:
    kfree(priv);
err_out:
    return ret;
}

static int mma8452_stop_chip(struct i2c_client *client)
{
    struct mma8452_info *priv = i2c_get_clientdata(client);
    int ret;

    priv->ctl_reg = i2c_smbus_read_byte_data(client,
                             MMA8452_CTRL_REG1);
    ret = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1,
                        priv->ctl_reg & 0xFE);
    return ret;
}

static int __devexit mma8452_remove(struct i2c_client *client)
{
    int ret;
    struct mma8452_info *priv = i2c_get_clientdata(client);
    ret = mma8452_stop_chip(client);
    input_free_polled_device(priv->idev);
    hwmon_device_unregister(priv->hwmon_dev);

    return ret;
}

#ifdef CONFIG_PM_SLEEP
static int mma8452_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);

    return mma8452_stop_chip(client);
}

static int mma8452_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mma8452_info *priv = i2c_get_clientdata(client);

    return init_mma8452_chip(priv);
}
#endif

static const struct i2c_device_id mma8452_id[] = {
    {"mma8452", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, mma8452_id);

static SIMPLE_DEV_PM_OPS(mma8452_pm_ops, mma8452_suspend, mma8452_resume);
static struct i2c_driver mma8452_driver = {
    .driver = {
         .name = "mma8452",
         .owner = THIS_MODULE,
         .pm = &mma8452_pm_ops,
         },
    .probe = mma8452_probe,
    .remove = __devexit_p(mma8452_remove),
    .id_table = mma8452_id,
};

static int __init mma8452_init(void)
{
    return i2c_add_driver(&mma8452_driver);
}

static void __exit mma8452_exit(void)
{
    i2c_del_driver(&mma8452_driver);
}

module_init(mma8452_init);
module_exit(mma8452_exit);

MODULE_AUTHOR("Peter Hu <peter.hu@xxxxxx>");
MODULE_DESCRIPTION("Freescale MMA8452 3-axis gravity accelerator sensors");
MODULE_LICENSE("GPL");

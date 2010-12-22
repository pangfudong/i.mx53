/*
 * KERNEL_DIR/drivers/power/bq27510_battery.c
 *
 * ONYX E-BOOK V2.0 battery driver
 *
 * Copyright (C) 2010 Onyx International, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>

#define ADC_I2C_ADDR			0x54
#define BQ27510_I2C_ADDR		0x55
#define ADC_REG_VOLT			0x00
#define BQ27x00_REG_VOLT		0x08
#define HIGH_BYTE(A)			((A) << 8)

static const char proc_filename[] = "driver/bq27510";

/* With TPS650244, V3.0_IO = 3.125V */
static int voltage_ref = 3125;

void set_reference_voltage(int ref)
{
	voltage_ref = ref;
}
EXPORT_SYMBOL(set_reference_voltage);

/* Address list to scan */
static unsigned short normal_i2c[] = { ADC_I2C_ADDR, BQ27510_I2C_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static int bq27510_read(u8 reg, int *rt_value, int b_single, struct i2c_client *client)
{
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = data[1] | HIGH_BYTE(data[0]);
			else
				*rt_value = data[0];

			return 0;
		} else
			return err;
	} else
		return err;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27510_battery_voltage(struct i2c_client *client)
{
	int ret = 0, volt = 0;

	if (client->addr == BQ27510_I2C_ADDR)
	{
		ret = bq27510_read(BQ27x00_REG_VOLT, &volt, 0, client);
		volt = be16_to_cpu(volt);
	}
	else if (client->addr == ADC_I2C_ADDR)
	{
		ret = bq27510_read(ADC_REG_VOLT, &volt, 0, client);
		volt = (volt >> 4) & 0x00FF;
		if (volt < 186)
		{
			/* Workaroud: reference voltage will be dropped as battery voltage < 3.3V */
			volt = 175;
		}
		volt = volt * voltage_ref / 179;
	}

	if (ret) {
		printk(KERN_ERR "error reading voltage\n");
		return ret;
	}

	return volt;
}

static int bq27510_proc_read(char *page, char **start, off_t off, int count, int *eof, void *_dev)
{
    int ret = 0;
    struct i2c_client *client = _dev;

    // The read offset must be 0
    if (off != 0 || count == 0)
    {
        return 0;
    }

    ret = snprintf(page, count, "Voltage: %dmV\n",
    	bq27510_battery_voltage(client));
    *eof = 1;
    return ret;
}

static int bq27510_probe(struct i2c_adapter *adap, int addr, int kind)
{
	int ret;
	struct i2c_client *client;

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client)
	{
		return -ENOMEM;
	}

	client->addr = addr;
	client->adapter = adap;
	ret = i2c_attach_client(client);
	if (ret)
	{
		printk(KERN_ERR "Failed to attach bq27510 i2c client.");
		return ret;
	}

	if (NULL == create_proc_read_entry(proc_filename, 0, NULL, bq27510_proc_read, client))
    {
        printk(KERN_ERR "Failed to create /proc/%s\n", proc_filename);
    }
	return 0;
}

static int bq27510_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, bq27510_probe);
}

static int bq27510_i2c_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	kfree(client);
	return 0;
}

static struct i2c_driver bq27510_i2c_driver = {
	.driver = {
		.name = "bq27510",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_BQ27510,
	.attach_adapter = bq27510_i2c_attach,
	.detach_client =  bq27510_i2c_detach,
	.command =        NULL,
};

static int __init bq27510_battery_init(void)
{
	return i2c_add_driver(&bq27510_i2c_driver);
}

static void __exit bq27510_battery_exit(void)
{
	i2c_del_driver(&bq27510_i2c_driver);
}

module_init(bq27510_battery_init);
module_exit(bq27510_battery_exit);

MODULE_AUTHOR("Ivan Li ivan@onyx-international.com");
MODULE_DESCRIPTION("BQ27510 battery gauge driver");
MODULE_LICENSE("GPL");

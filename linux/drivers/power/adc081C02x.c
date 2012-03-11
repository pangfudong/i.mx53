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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>

#define ADC_I2C_ADDR1			0x54
#define ADC_I2C_ADDR2			0x50
#define BQ27510_I2C_ADDR		0x55
#define ADC_REG_VOLT			0x00
#define bq27510_REG_VOLT		0x08
#define HIGH_BYTE(A)			((A) << 8)

struct bq27510_device_info
{
	struct i2c_client*  client;
	struct power_supply	bat;
};

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

	switch (client->addr)
	{
		case BQ27510_I2C_ADDR:
			ret = bq27510_read(bq27510_REG_VOLT, &volt, 0, client);
			volt = be16_to_cpu(volt);
			break;
		case ADC_I2C_ADDR1:
		case ADC_I2C_ADDR2:
			bq27510_read(ADC_REG_VOLT, &volt, 0, client);
			volt = ((volt >> 4) & 0x00FF) * 5069 / 256;
			break;
		default:
			break;
	}

	if (ret) {
		printk(KERN_ERR "error reading voltage\n");
		return ret;
	}

	return volt;
}

static enum power_supply_property bq27510_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

#define to_bq27510_device_info(x) container_of((x), \
				struct bq27510_device_info, bat);

static int bq27510_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27510_device_info *di = to_bq27510_device_info(psy);

	switch (psp)
	{
		case POWER_SUPPLY_PROP_STATUS:
			ret = -EINVAL;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			ret = -EINVAL;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = bq27510_battery_voltage(di->client) * 1000;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			ret = -EINVAL;
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static void bq27510_powersupply_init(struct power_supply *ps)
{
	ps->type = POWER_SUPPLY_TYPE_BATTERY;
	ps->properties = bq27510_battery_props;
	ps->num_properties = ARRAY_SIZE(bq27510_battery_props);
	ps->get_property = bq27510_battery_get_property;
	ps->external_power_changed = NULL;
}

static int __devinit bq27510_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct bq27510_device_info* di;

	int ret = i2c_smbus_read_word_data(client, 0x0);
	if (ret < 0)
	{
		printk(KERN_ERR "Read from 0x%x[0x0] failed.\n", client->addr);
		return ret;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	bq27510_powersupply_init(&di->bat);
	ret = power_supply_register(&client->dev, &di->bat);
	if (ret) {
		dev_err(&client->dev, "failed to register battery\n");
		kfree(di);
		return ret;
	}

	i2c_set_clientdata(client, di);
	di->client = client;
    return 0;
}

static int __devexit bq27510_i2c_remove(struct i2c_client *i2c)
{
	struct bq27510_device_info *di = i2c_get_clientdata(i2c);
	kfree(di);
	return 0;
}

static const struct i2c_device_id bq27510_id[] = {
       { "ADC081C021", 0 },
       { "ADC081C027", 0 },
       { "bq27510", 0 },
       { }
};

static struct i2c_driver bq27510_i2c_driver = {
	.driver = {
		.name = "BQ27510",
		.owner = THIS_MODULE,
	},
	.probe    = bq27510_i2c_probe,
	.remove   = __devexit_p(bq27510_i2c_remove),
	.id_table = bq27510_id,
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

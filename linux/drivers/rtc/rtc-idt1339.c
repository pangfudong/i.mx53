/*
 * rtc-idt1339.c - RTC driver for some mostly-compatible I2C chips.
 *
 *  Copyright (C) 2005 James Chapman (ds1337 core)
 *  Copyright (C) 2006 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/rtc.h>
#include <linux/bcd.h>

/* We can't determine type by probing, but if we expect pre-Linux code
 * to have set the chip up as a clock (turning on the oscillator and
 * setting the date and time), Linux can ignore the non-clock features.
 * That's a natural job for a factory or repair bench.
 *
 * This is currently a simple no-alarms driver.  If your board has the
 * alarm irq wired up on a ds1337 or ds1339, and you want to use that,
 * then look at the rtc-rs5c372 driver for code to steal...
 */
enum idt_type {
	idt_1339,	
};


/* RTC registers don't differ much, except for the century flag */
#	define IDT1339_REG_SECS		0x00	/* 00-59 */
#	define IDT1339_BIT_CH		0x80
#	define DS1340_BIT_nEOSC		0x80
#	define IDT1339_REG_MIN		0x01	/* 00-59 */
#	define IDT1339_REG_HOUR		0x02	/* 00-23, or 1-12{am,pm} */
#	define IDT1339_BIT_12HR		0x40	/* in REG_HOUR */
#	define IDT1339_BIT_PM		0x20	/* in REG_HOUR */
#	define DS1340_BIT_CENTURY_EN	0x80	/* in REG_HOUR */
#	define DS1340_BIT_CENTURY	0x40	/* in REG_HOUR */
#	define IDT1339_REG_WDAY		0x03	/* 01-07 */
#	define IDT1339_REG_MDAY		0x04	/* 01-31 */
#	define IDT1339_REG_MONTH		0x05	/* 01-12 */
#	define IDT1339_BIT_CENTURY	0x80	/* in REG_MONTH */
#	define IDT1339_REG_YEAR		0x06	/* 00-99 */

/* Other registers (control, status, alarms, trickle charge, NVRAM, etc)
 * start at 7, and they differ a LOT. Only control and status matter for
 * basic RTC date and time functionality; be careful using them.
 */
/*#	define IDT1339_REG_CONTROL	0x0e
#	define IDT1339_BIT_OUT		0x80
#	define DS1338_BIT_OSF		0x20
#	define IDT1339_BIT_SQWE		0x10
#	define IDT1339_BIT_RS1		0x02
#	define IDT1339_BIT_RS0		0x01
*/
#	define IDT1339_REG_CONTROL	0x0e
#	define IDT1339_BIT_nEOSC		0x80
#	define IDT1339_BIT_RS2		0x10
#	define IDT1339_BIT_RS1		0x08
#	define IDT1339_BIT_INTCN		0x04
#	define IDT1339_BIT_A2IE		0x02
#	define IDT1339_BIT_A1IE		0x01

#	define IDT1339_REG_STATUS		0x0f
#	define IDT1339_BIT_OSF		0x80
#	define IDT1339_BIT_A2I		0x02
#	define IDT1339_BIT_A1I		0x01

#	define IDT1339_REG_TRICKLE	0x10



struct idt1339 {
	u8			reg_addr;
	bool			has_nvram;
	u8			regs[8];
	enum idt_type		type;
	struct i2c_msg		msg[2];
	struct i2c_client	*client;
	struct i2c_client	dev;
	struct rtc_device	*rtc;
};

struct chip_desc {
	char		name[9];
	unsigned		nvram56:1;
	unsigned		alarm:1;
	enum idt_type		type;
};

static const struct chip_desc chips[] = { {
	.name		= "idt1339",
	.type		= idt_1339,
	.nvram56	= 1,
},};

static inline const struct chip_desc *find_chip(const char *s)
{
	unsigned i;

	for (i = 0; i < ARRAY_SIZE(chips); i++)
		if (strnicmp(s, chips[i].name, sizeof chips[i].name) == 0)
			return &chips[i];
	return NULL;
}

static int idt1339_get_time(struct device *dev, struct rtc_time *t)
{
	struct idt1339	*idt1339 = dev_get_drvdata(dev);
	int		tmp;

	/* read the RTC date and time registers all at once */
	idt1339->msg[1].flags = I2C_M_RD;
	idt1339->msg[1].len = 7;

	tmp = i2c_transfer(to_i2c_adapter(idt1339->client->dev.parent),
			idt1339->msg, 2);
	if (tmp != 2) {
		dev_err(dev, "%s error %d\n", "read", tmp);
		return -EIO;
	}

	dev_dbg(dev, "%s: %02x %02x %02x %02x %02x %02x %02x\n",
			"read",
			idt1339->regs[0], idt1339->regs[1],
			idt1339->regs[2], idt1339->regs[3],
			idt1339->regs[4], idt1339->regs[5],
			idt1339->regs[6]);

	t->tm_sec = BCD2BIN(idt1339->regs[IDT1339_REG_SECS] & 0x7f);
	t->tm_min = BCD2BIN(idt1339->regs[IDT1339_REG_MIN] & 0x7f);
	tmp = idt1339->regs[IDT1339_REG_HOUR] & 0x3f;
	t->tm_hour = BCD2BIN(tmp);
	t->tm_wday = BCD2BIN(idt1339->regs[IDT1339_REG_WDAY] & 0x07) - 1;
	t->tm_mday = BCD2BIN(idt1339->regs[IDT1339_REG_MDAY] & 0x3f);
	tmp = idt1339->regs[IDT1339_REG_MONTH] & 0x1f;
	t->tm_mon = BCD2BIN(tmp) - 1;

	/* assume 20YY not 19YY, and ignore DS1337_BIT_CENTURY */
	t->tm_year = BCD2BIN(idt1339->regs[IDT1339_REG_YEAR]) + 100;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"read", t->tm_sec, t->tm_min,
		t->tm_hour, t->tm_mday,
		t->tm_mon, t->tm_year, t->tm_wday);

	/* initial clock setting can be undefined */
	return rtc_valid_tm(t);
}

static int idt1339_set_time(struct device *dev, struct rtc_time *t)
{
	struct idt1339	*idt1339 = dev_get_drvdata(dev);
	int		result;
	int		tmp;
	u8		*buf = idt1339->regs;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"write", t->tm_sec, t->tm_min,
		t->tm_hour, t->tm_mday,
		t->tm_mon, t->tm_year, t->tm_wday);

	*buf++ = 0;		/* first register addr */
	buf[IDT1339_REG_SECS] = BIN2BCD(t->tm_sec);
	buf[IDT1339_REG_MIN] = BIN2BCD(t->tm_min);
	buf[IDT1339_REG_HOUR] = BIN2BCD(t->tm_hour);
	buf[IDT1339_REG_WDAY] = BIN2BCD(t->tm_wday + 1);
	buf[IDT1339_REG_MDAY] = BIN2BCD(t->tm_mday);
	buf[IDT1339_REG_MONTH] = BIN2BCD(t->tm_mon + 1);

	/* assume 20YY not 19YY */
	tmp = t->tm_year - 100;
	buf[IDT1339_REG_YEAR] = BIN2BCD(tmp);

	switch (idt1339->type) {
	case idt_1339:
		buf[IDT1339_REG_MONTH] |= IDT1339_BIT_CENTURY;
		break;
	default:
		break;
	}

	idt1339->msg[1].flags = 0;
	idt1339->msg[1].len = 8;

	dev_dbg(dev, "%s: %02x %02x %02x %02x %02x %02x %02x\n",
		"write", buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6]);

	result = i2c_transfer(to_i2c_adapter(idt1339->client->dev.parent),
			&idt1339->msg[1], 1);
	if (result != 1) {
		dev_err(dev, "%s error %d\n", "write", tmp);
		return -EIO;
	}
	return 0;
}

static const struct rtc_class_ops ds13xx_rtc_ops = {
	.read_time	= idt1339_get_time,
	.set_time	= idt1339_set_time,
};

/*----------------------------------------------------------------------*/

#define NVRAM_SIZE	56

static ssize_t
idt1339_nvram_read(struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct i2c_client	*client;
	struct idt1339		*idt1339;
	struct i2c_msg		msg[2];
	int			result;

	client = to_i2c_client(container_of(kobj, struct device, kobj));
	idt1339 = i2c_get_clientdata(client);

	if (unlikely(off >= NVRAM_SIZE))
		return 0;
	if ((off + count) > NVRAM_SIZE)
		count = NVRAM_SIZE - off;
	if (unlikely(!count))
		return count;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	buf[0] = 8 + off;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = count;
	msg[1].buf = buf;

	result = i2c_transfer(to_i2c_adapter(client->dev.parent), msg, 2);
	if (result != 2) {
		dev_err(&client->dev, "%s error %d\n", "nvram read", result);
		return -EIO;
	}

	return count;
}

static ssize_t
idt1339_nvram_write(struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct i2c_client	*client;
	u8			buffer[NVRAM_SIZE + 1];
	int			ret;

	client = to_i2c_client(container_of(kobj, struct device, kobj));

	if (unlikely(off >= NVRAM_SIZE))
		return -EFBIG;
	if ((off + count) > NVRAM_SIZE)
		count = NVRAM_SIZE - off;
	if (unlikely(!count))
		return count;

	buffer[0] = 8 + off;
	memcpy(buffer + 1, buf, count);

	ret = i2c_master_send(client, buffer, count + 1);
	return (ret < 0) ? ret : (ret - 1);
}

static struct bin_attribute nvram = {
	.attr = {
		.name	= "nvram",
		.mode	= S_IRUGO | S_IWUSR,
		.owner	= THIS_MODULE,
	},

	.read	= idt1339_nvram_read,
	.write	= idt1339_nvram_write,
	.size	= NVRAM_SIZE,
};

/*----------------------------------------------------------------------*/

static struct i2c_driver idt1339_driver;

static int __devinit idt1339_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct idt1339		*idt1339;
	int			err = -ENODEV;
	int			tmp;
	const struct chip_desc	*chip;
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);

	chip = find_chip("idt1339");
	if (!chip) {
		dev_err(&client->dev, "unknown chip type '%s'\n",
				client->name);
		return -ENODEV;
	}

	if (!i2c_check_functionality(adapter,
			I2C_FUNC_I2C | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -EIO;

	if (!(idt1339 = kzalloc(sizeof(struct idt1339), GFP_KERNEL)))
		return -ENOMEM;

	idt1339->client = client;
	i2c_set_clientdata(client, idt1339);

	idt1339->msg[0].addr = client->addr;
	idt1339->msg[0].flags = 0;
	idt1339->msg[0].len = 1;
	idt1339->msg[0].buf = &idt1339->reg_addr;

	idt1339->msg[1].addr = client->addr;
	idt1339->msg[1].flags = I2C_M_RD;
	idt1339->msg[1].len = sizeof(idt1339->regs);
	idt1339->msg[1].buf = idt1339->regs;

	idt1339->type = chip->type;

	switch (idt1339->type) {
	case idt_1339:

		idt1339->reg_addr = IDT1339_REG_CONTROL;
		idt1339->msg[1].len = 2;

		// get registers that the "rtc" read below won't read...
		tmp = i2c_transfer(adapter, idt1339->msg, 2);
		if (tmp != 2) {
			pr_debug("read error %d\n", tmp);
			err = -EIO;
			goto exit_free;
		}

		idt1339->reg_addr = 0;
		idt1339->msg[1].len = sizeof(idt1339->regs);

		// oscillator off?  turn it on, so clock can tick.
		if (idt1339->regs[0] & IDT1339_BIT_nEOSC)idt1339->regs[0] &= ~IDT1339_BIT_nEOSC;
		// disable int out
		if( (idt1339->regs[0] & 7) != 4 )idt1339->regs[0] = (idt1339->regs[0]&~7)|4;	
		i2c_smbus_write_byte_data(client, IDT1339_REG_CONTROL,idt1339->regs[0] );

		// oscillator fault?  clear flag, and warn
		if (idt1339->regs[1] & IDT1339_BIT_OSF) {
			i2c_smbus_write_byte_data(client, IDT1339_REG_STATUS,idt1339->regs[1] & ~IDT1339_BIT_OSF);
			dev_warn(&client->dev, "SET TIME!\n");
		}
		break;
	default:
		break;
	}

read_rtc:
	/* read RTC registers */

	tmp = i2c_transfer(adapter, idt1339->msg, 2);
	if (tmp != 2) {
		pr_debug("read error %d\n", tmp);
		err = -EIO;
		goto exit_free;
	}

	/* minimal sanity checking; some chips (like DS1340) don't
	 * specify the extra bits as must-be-zero, but there are
	 * still a few values that are clearly out-of-range.
	 */
	tmp = idt1339->regs[IDT1339_REG_SECS];
	switch (idt1339->type) {
	case idt_1339:
		/* clock halted?  turn it on, so clock can tick. */
		if (tmp & IDT1339_BIT_CH) {
			i2c_smbus_write_byte_data(client, IDT1339_REG_SECS, 0);
			dev_warn(&client->dev, "SET TIME!\n");
			goto read_rtc;
		}
		break;
	}

	tmp = idt1339->regs[IDT1339_REG_SECS];
	tmp = BCD2BIN(tmp & 0x7f);
	if (tmp > 60)
		goto exit_bad;
	tmp = BCD2BIN(idt1339->regs[IDT1339_REG_MIN] & 0x7f);
	if (tmp > 60)
		goto exit_bad;

	tmp = BCD2BIN(idt1339->regs[IDT1339_REG_MDAY] & 0x3f);
	if (tmp == 0 || tmp > 31)
		goto exit_bad;

	tmp = BCD2BIN(idt1339->regs[IDT1339_REG_MONTH] & 0x1f);
	if (tmp == 0 || tmp > 12)
		goto exit_bad;

	tmp = idt1339->regs[IDT1339_REG_HOUR];
	switch (idt1339->type) {
	case idt_1339:
	default:
		if (!(tmp & IDT1339_BIT_12HR))
			break;

		/* Be sure we're in 24 hour mode.  Multi-master systems
		 * take note...
		 */
		tmp = BCD2BIN(tmp & 0x1f);
		if (tmp == 12)
			tmp = 0;
		if (idt1339->regs[IDT1339_REG_HOUR] & IDT1339_BIT_PM)
			tmp += 12;
		i2c_smbus_write_byte_data(client,
				IDT1339_REG_HOUR,
				BIN2BCD(tmp));
	}

	idt1339->rtc = rtc_device_register(client->name, &client->dev,
				&ds13xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(idt1339->rtc)) {
		err = PTR_ERR(idt1339->rtc);
		dev_err(&client->dev,
			"unable to register the class device\n");
		goto exit_free;
	}

	if (chip->nvram56) {
		err = sysfs_create_bin_file(&client->dev.kobj, &nvram);
		if (err == 0) {
			idt1339->has_nvram = true;
			dev_info(&client->dev, "56 bytes nvram\n");
		}
	}

	return 0;

exit_bad:

	dev_dbg(&client->dev, "%s: %02x %02x %02x %02x %02x %02x %02x\n",
			"bogus register",
			idt1339->regs[0], idt1339->regs[1],
			idt1339->regs[2], idt1339->regs[3],
			idt1339->regs[4], idt1339->regs[5],
			idt1339->regs[6]);

exit_free:

	kfree(idt1339);
	return err;
}

static int __devexit idt1339_remove(struct i2c_client *client)
{
	struct idt1339	*idt1339 = i2c_get_clientdata(client);

	if (idt1339->has_nvram)
		sysfs_remove_bin_file(&client->dev.kobj, &nvram);

	rtc_device_unregister(idt1339->rtc);
	kfree(idt1339);
	return 0;
}

static const struct i2c_device_id idt1339_id[] = {
	{ "idt1339", 0 },
	{ },
};

static struct i2c_driver idt1339_driver = {
	.driver = {
		.name	= "idt1339",
		.owner	= THIS_MODULE,
	},
	.probe		= idt1339_probe,
	.remove		= __devexit_p(idt1339_remove),
	.id_table	= idt1339_id
};

static int __init idt1339_init(void)
{
	return i2c_add_driver(&idt1339_driver);
}
module_init(idt1339_init);

static void __exit idt1339_exit(void)
{
	i2c_del_driver(&idt1339_driver);
}
module_exit(idt1339_exit);

MODULE_DESCRIPTION("RTC driver for IDT1339 and similar chips");
MODULE_LICENSE("GPL");

/* Touch Screen driver for Renesas Raydium Platform
 *
 * Copyright (C) 2011 Raydium, Inc.
 *
 * Based on migor_ts.c and synaptics_i2c_rmi.c
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU  General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */


#include <linux/input.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

//#include "raydium_ts.h"
#include <linux/io.h>

#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <asm/io.h>


#define RM310XX_I2C_TS_NAME "raydium"
#define DRIVER_VERSION "v1.0"
#define DEVICE_NAME		"raydiumflash"


#define TS_MIN_X 0
#define TS_MAX_X 1024
#define TS_MIN_Y 0
#define TS_MAX_Y 768
#define MAX_BUFFER_SIZE	144


static int raydium_flash_major = 121;
static int raydium_flash_minor = 0;
static struct cdev raydium_flash_cdev;
static struct class *raydium_flash_class = NULL;
static dev_t raydium_flash_dev;

static int flag=0;

static struct workqueue_struct *raydium_wq;

struct raydium_ts_priv{
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct work;
	int irq;
};

struct raydium_flash_data {
	rwlock_t lock;
	unsigned char bufferIndex;
	unsigned int length;
	unsigned int buffer[MAX_BUFFER_SIZE];
	struct i2c_client *client;
};

struct raydium_flash_data *flash_priv;

static int
RAYDIUM_i2c_transfer(
	struct i2c_client *client, struct i2c_msg *msgs, int cnt)
{
	int ret, count=5;
	while(count >= 0){
		count-= 1;
		ret = i2c_transfer(client->adapter, msgs, cnt);

				if(ret < 0){
						udelay(500);
			continue;
				}
		break;
	}
	return ret;
}

static int
RAYDIUM_i2c_read(
	struct i2c_client *client,
	uint8_t cmd,
	uint8_t *data,
	int length)
{
	int ret;
		struct i2c_msg msgs[] = {
		{.addr = client->addr, .flags = 0, .len = 1, .buf = &cmd,},
		{.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = data,}
		};

		ret = RAYDIUM_i2c_transfer(client, msgs, 2);
	return ret;
}



int raydium_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
 struct i2c_msg msg[2];
 int ret;
 char *str;
 //struct raydium_flash_data *dev = file->private_data;
 file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
 str = file->private_data;
 ret=copy_from_user(str, buff, count);

 if(str[0]==0x11&&str[1]==0x22)
	{
	 flag=1;
	 udelay(100);
	 printk("Start!\n");
	 return 1;
	}
 else if(str[0]==0x33&&str[1]==0x44)
	{
	 flag=0;
	 udelay(50);
	 printk("End!\n");

	return 1;
	}

//for(i=0;i<5;i++)
	{
	udelay(5);
	}

	msg[0].addr = flash_priv->client->addr;
	msg[0].flags = 0;
	msg[0].len = str[0];
	msg[0].buf = &str[1];

	i2c_transfer(flash_priv->client->adapter, msg, 1);

// for(i=0;i<5;i++)
	{
//	udelay(str[str[0]+1]);
	}

/* printk("Write:");
 printk("lenth=%d, ", str[0]);
 for(ii=0;ii<str[0];ii++)
	 printk("data=%d, ", str[ii+1]);
 printk("\n");
*/
 return 1;
}

int raydium_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
 struct i2c_msg msg[2];
 char *str;
 //struct raydium_flash_data *dev = file->private_data;
 int ret;
 file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
 str = file->private_data;
 if(copy_from_user(str, buff, count))
	return -EFAULT;

// for(i=0;i<5;i++)
	{
	udelay(5);
	}
/*
	for(ii=0;ii<32;ii++)
		printk("%d, ", str[ii]);
	printk("\n");
*/

	msg[0].addr = flash_priv->client->addr;
	msg[0].flags = 0;
	msg[0].len = str[0];
	msg[0].buf = &str[1];
//	start_reg = 0x10;
	msg[1].addr = flash_priv->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = str[str[0]+1];
	msg[1].buf = &str[str[0]+2];

	i2c_transfer(flash_priv->client->adapter, msg, 2);

	ret=copy_to_user(buff, str, count);
/*	for(ii=0;ii<str[0]+2+str[str[0]+1];ii++)
		printk("%c, ", str[ii]);
	printk("\n");	*/
// for(i=0;i<5;i++)
	{
//	udelay(str[str[0]+str[2]+2]);
	}

/* printk("Read:");
 printk("length=%d, ", str[0]+str[2]+2);
 for(ii=0;ii<str[0]+str[2]+2;ii++)
	 printk("data=%d, ", buff[ii]);
 printk("\n");
*/

 return 1;
}


int raydium_flash_open(struct inode *inode, struct file *file)
{
	int i;
	struct raydium_flash_data *dev;

	dev = kmalloc(sizeof(struct raydium_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	/* initialize members */
	rwlock_init(&dev->lock);
	for(i = 0; i < MAX_BUFFER_SIZE; i++) {
		dev->buffer[i] = 0xFF;
	}
//	dev->client = flash_priv->client;
//	printk("%d\n", dev->client->addr);
	file->private_data = dev;


	return 0;   /* success */
}

int raydium_flash_close(struct inode *inode, struct file *file)
{
	struct raydium_flash_data *dev = file->private_data;

	if (dev) {
		kfree(dev);
	}
	return 0;   /* success */
}

struct file_operations raydium_flash_fops = {
	.owner = THIS_MODULE,
	.open = raydium_flash_open,
	.release = raydium_flash_close,
//	.ioctl = raydium_flash_ioctl,
	.write = raydium_flash_write,
	.read = raydium_flash_read,
};

static int raydium_flash_init(struct raydium_ts_priv *priv)
{
	dev_t dev = MKDEV(raydium_flash_major, 0);
	int alloc_ret = 0;
	int major;
	int cdev_err = 0;
	int input_err = 0;
	struct device *class_dev = NULL;

	// dynamic allocate driver handle
	alloc_ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if(alloc_ret) {
		goto error;
	}
	major = MAJOR(dev);
	raydium_flash_major = major;

	cdev_init(&raydium_flash_cdev, &raydium_flash_fops);
	raydium_flash_cdev.owner = THIS_MODULE;
	raydium_flash_cdev.ops = &raydium_flash_fops;
	cdev_err = cdev_add(&raydium_flash_cdev, MKDEV(raydium_flash_major, raydium_flash_minor), 1);
	if(cdev_err) {
		goto error;
	}

	// register class
	raydium_flash_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(raydium_flash_class)) {
		goto error;
	}
	raydium_flash_dev = MKDEV(raydium_flash_major, raydium_flash_minor);
	class_dev = device_create(raydium_flash_class, NULL, raydium_flash_dev, NULL, DEVICE_NAME);


/*
	input_dev = input_allocate_device();

	if (!input_dev) {
		input_err = -ENOMEM;
		goto error;
	}
*/
//	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

//	input_dev->name = DEVICE_NAME;

//	input_err = input_register_device(input_dev);



	printk("============================================================\n");
	printk("raydium_flash driver loaded\n");
	printk("============================================================\n");
	flash_priv=kzalloc(sizeof(*flash_priv),GFP_KERNEL);
	if (priv == NULL) {
		//dev_set_drvdata(&client->dev, NULL);
				input_err = -ENOMEM;
				goto error;
	}
	flash_priv->client = priv->client;
	return 0;

error:
	if(cdev_err == 0) {
		cdev_del(&raydium_flash_cdev);
	}

	if(alloc_ret == 0) {
		unregister_chrdev_region(dev, 1);
	}

	if(input_err != 0)
	{
	printk("flash_priv error!\n");
	}

	return -1;
}
/*
static void raydium_flash_exit(void)
{
	dev_t dev = MKDEV(raydium_flash_major, raydium_flash_minor);

	// unregister class
	device_destroy(raydium_flash_class, raydium_flash_dev);
	class_destroy(raydium_flash_class);

	// unregister driver handle
	cdev_del(&raydium_flash_cdev);
	unregister_chrdev_region(dev, 1);

	printk("============================================================\n");
	printk("raydium_flash driver unloaded\n");
	printk("============================================================\n");
}
*/

#define MS_TO_NS(x) (x*1E6L)


static enum hrtimer_restart raydium_ts_timer_func(struct hrtimer *timer)
{
	struct raydium_ts_priv *priv = container_of(timer, struct raydium_ts_priv, timer);
	/* printk("synaptics_ts_timer_func\n"); */
		queue_work(raydium_wq, &priv->work);
//		msleep(10);
		hrtimer_start(&priv->timer, ktime_set(0, MS_TO_NS(10)), HRTIMER_MODE_REL);
		return HRTIMER_NORESTART;

}


static void raydium_ts_poscheck(/*void* unused*/struct work_struct *work)
{
	struct raydium_ts_priv *priv = container_of(work,struct raydium_ts_priv,
						  work);

	unsigned char touching;
	struct i2c_msg msg[2];

	int posx[10];
	int posy[10];
	int pid[10];
	uint8_t start_reg;
	uint8_t Rdbuf[50];
	int ret;
	int z=5;
	int w=10;
	int i;
//	printk(KERN_INFO "raydium_ts_poscheck is working\n");
	if(flag==1)
		return;
/*	else
		flag=1;*/
/*	msg[0].addr = priv->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x10;
	msg[1].addr = priv->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = Wrbuf;
	//memset(Wrbuf, 0, sizeof(Wrbuf));
	//memset(Rdbuf, 0, sizeof(Rdbuf));


	//ret = GPIO_Read(0x10, 1, Wrbuf);
	ret = i2c_transfer(priv->client->adapter, msg, 2);
//	printk(KERN_INFO "raydium_ts_poscheck is working2\n");
	touching=Wrbuf[0]&0x0F;*/
//	printk(KERN_INFO "raydium_ts_poscheck is working2\n");

	msg[0].addr = priv->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x00;
	msg[1].addr = priv->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 8;
	msg[1].buf = Rdbuf;
	ret = i2c_transfer(priv->client->adapter, msg, 2);
	//ret = GPIO_Read(0x11, touching*5, Rdbuf);
	touching=0;
	for(i=0;i<2;i++)
		{
		posx[i] = (((Rdbuf[1+4*i]&0x03) << 8) | Rdbuf[0+4*i]);
		posy[i] = (((Rdbuf[3+4*i]&0x03) << 8) | Rdbuf[2+4*i]);
		pid[i] = ((Rdbuf[1+4*i]&0xF0) >> 4);
		if(posx[i]!=0||posy[i]!=0)
			touching++;
		}


	if(!(touching)){
		//printk(KERN_INFO "abc\n");
		input_report_key(priv->input_dev, BTN_TOUCH, 0);
		input_report_key(priv->input_dev, BTN_2, 0);
		//input_report_key(priv->input_dev, ABS_MT_TRACKING_ID, 0);
		input_report_abs(priv->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(priv->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		//input_report_abs(priv->input_dev, ABS_PRESSURE, 0);
		input_report_abs(priv->input_dev, ABS_MT_POSITION_X, 0);
		input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, 0);
		//input_report_key(priv->input_dev, ABS_MT_TRACKING_ID, 1);
		//input_report_abs(priv->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		//input_report_abs(priv->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		//input_report_abs(priv->input_dev, ABS_MT_POSITION_X, 0);
		//input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, 0);
		input_mt_sync(priv->input_dev);
		input_sync(priv->input_dev);

		hrtimer_cancel(&priv->timer);
		if (priv->use_irq)
			enable_irq(priv->client->irq);

		goto out;
	}
//	printk(KERN_INFO "oldtouch = %d, oldx1 = %d, oldy1 = %d, oldx2 = %d, oldy2 = %d\n", oldtouching, oldposx1, oldposy1, oldposx2, oldposy2);

	/*if(Wrbuf[0]==0)
		{
		input_sync(priv->input_dev);
		mod_timer(&timer, jiffies + HZ/500);
		goto out;
		}*/
//	if((Rdbuf[0]&0x0F)==0)
//		goto out;
/*	if(ret!=1){
		dev_err(&priv->client->dev, "Unable to write i2c index\n");
		dev_err(&priv->client->dev, "ret=%d\n", ret);
		goto out;
	}
*/
/*	if(ret!=sizeof(Rdbuf)){
		dev_err(&priv->client->dev, "Unable to read i2c page!\n");
		dev_err(&priv->client->dev, "ret=%d\n", ret);
		goto out;
	}
*/
	//oldtouching=Rdbuf[1];

	//printk(KERN_INFO "posx1 = %d\n", posx1);
	//printk(KERN_INFO "posy1 = %d\n", posy1);


	/*
	if(touching==2&&oldtouching==2)
		{
		if()
			{
			posx1=posx1+posx2;
			posx2=posx1-posx2;
			posx1=posx1-posx2;
			posy1=posy1+posy2;
			posy2=posy1-posy2;
			posy1=posy1-posy2;
			}
		}
*/	//input_report_key(priv->input_dev, BTN_TOUCH, 1);
	//input_report_abs(priv->input_dev, ABS_X, posx1);
		//input_report_abs(priv->input_dev, ABS_Y, posy1);
//	oldtouching = touching;
//	oldposx1=posx1;
//	oldposy1=posy1;
	//input_report_key(priv->input_dev, BTN_2, 1);
	//input_report_abs(priv->input_dev, ABS_X, posx1+100);
		//input_report_abs(priv->input_dev, ABS_Y, posy1+100);


//	input_report_key(priv->input_dev, ABS_MT_TRACKING_ID, 0);
	for(i=0;i<touching;i++)
		{
		input_report_abs(priv->input_dev, ABS_MT_TRACKING_ID, pid[i]);
		input_report_abs(priv->input_dev, ABS_MT_POSITION_X, posx[i]);
		input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, posy[i]);
		input_report_abs(priv->input_dev, ABS_MT_TOUCH_MAJOR, z*(i+1));
		input_report_abs(priv->input_dev, ABS_MT_WIDTH_MAJOR, w*(i+1));
		input_mt_sync(priv->input_dev);
		//printk(KERN_INFO "pid%d = %d\n", i,pid[i]);
		//printk(KERN_INFO "posx%d = %d\n", i,posx[i]);
		//printk(KERN_INFO "posy%d = %d\n", i,posy[i]);
		}

	input_sync(priv->input_dev);
	//printk("\n");
/*
	if(touching==2)
		mod_timer(&timer, jiffies + HZ/35);
	else
		mod_timer(&timer, jiffies + HZ/70);*/
//	udelay((touching-1)*5);

	out:
/*		if (priv->use_irq)
			enable_irq(priv->client->irq);
		else*/
			udelay((touching-1)*5);
//		return;


}

static irqreturn_t raydium_ts_isr(int irq,void *dev_id)
{
	struct raydium_ts_priv *priv=dev_id;
	disable_irq_nosync(priv->client->irq);
	hrtimer_start(&priv->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	return IRQ_HANDLED;
}


static int raydium_ts_open(struct input_dev *dev)
{
	return 0;
}

static void raydium_ts_close(struct input_dev *dev)
{
}



static int raydium_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct raydium_ts_priv *priv;
	int ret = 0;
	char buf[2];
	struct i2c_msg msg[1];
	priv=kzalloc(sizeof(*priv),GFP_KERNEL);
	if (priv == NULL)
	{
		dev_set_drvdata(&client->dev, NULL);
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&priv->work,raydium_ts_poscheck);

	priv->client = client;
	i2c_set_clientdata(client, priv);
	priv->use_irq=0;
	dev_set_drvdata(&client->dev, priv);

	buf[0]=0x7A;
	buf[1]=0x01;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = &buf;
	ret=i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("Show PID ... error\n");
		kfree(priv);
		return -EIO;
	}

	/* Calibration */
	buf[0]=0x78;
	buf[1]=0x03;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = &buf;
	ret=i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		kfree(priv);
		return -EIO;
	}

	msleep(500);

	priv->input_dev = input_allocate_device();
	if(priv->input_dev == NULL){
				ret = -ENOMEM;
				printk(KERN_ERR "raydium_ts_probe: Failed to allocate input device\n");
				goto err_input_dev_alloc_failed;
	}

	set_bit(EV_SYN, priv->input_dev->evbit);
	set_bit(EV_KEY, priv->input_dev->evbit);
	set_bit(BTN_TOUCH, priv->input_dev->keybit);
	set_bit(BTN_2, priv->input_dev->keybit);
	set_bit(ABS_MT_TRACKING_ID, priv->input_dev->keybit);
	set_bit(EV_ABS, priv->input_dev->evbit);
	input_set_abs_params(priv->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_X, TS_MIN_X, TS_MAX_X, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_HAT0X, TS_MIN_X, TS_MAX_X, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_HAT0Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_X, TS_MIN_X, TS_MAX_X, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_TOUCH_MAJOR, 0, 250, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_WIDTH_MAJOR, 0, 50, 0, 0);
	//input_set_abs_params(input_dev, ABS_PRESSURE, 0, 10, 0, 0);


	priv->input_dev->name = "raydium-touchscreen";
	priv->input_dev->id.bustype = BUS_I2C;
	priv->input_dev->dev.parent = &client->dev;

	priv->input_dev->open = raydium_ts_open;
	priv->input_dev->close = raydium_ts_close;





	/*setup_timer(&timer, raydium_ts_poscheck, NULL);
	timer.expires = jiffies +HZ * 30;
	add_timer(&timer);*/
	ret = input_register_device(priv->input_dev);
	priv->irq = client->irq;

		if (ret) {
				printk(KERN_ERR "raydium_ts_probe: Unable to register %s input device\n", priv->input_dev->name);
				goto err_input_register_device_failed;
		}
	//printk("client->irq=%d\n", client->irq);
		if (client->irq) {
		ret = request_irq(client->irq, raydium_ts_isr, IRQF_TRIGGER_FALLING, client->name, priv);
		//printk("request_irq ret=%d\n", ret);
		if (ret != 0) {
				free_irq(client->irq, priv);
		}
		if (ret == 0)
			priv->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}


//	if (!priv->use_irq)
	{
		hrtimer_init(&priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		priv->timer.function = raydium_ts_timer_func;
		if(!client->irq)
			hrtimer_start(&priv->timer, ktime_set(10, 0), HRTIMER_MODE_REL);
	}

	//device_init_wakeup(&client->dev, 1);
	printk(KERN_INFO "raydium_ts_probe: Start touchscreen %s is success\n", priv->input_dev->name);

	raydium_flash_init(priv);
	return 0;

err_input_register_device_failed:
		input_free_device(priv->input_dev);
err_input_dev_alloc_failed:
		kfree(priv);
err_alloc_data_failed:
return ret;

}


static int raydium_ts_remove(struct i2c_client *client)
{
	struct raydium_ts_priv *priv = i2c_get_clientdata(client);
	free_irq(priv->irq, priv);
	hrtimer_cancel(&priv->timer);
	input_unregister_device(priv->input_dev);
	kfree(priv);
	//dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static int raydium_ts_suspend(struct i2c_client *client, pm_message_t mesg)

{
	struct raydium_ts_priv *priv = i2c_get_clientdata(client);

	hrtimer_cancel(&priv->timer);
	cancel_work_sync(&priv->work);

	disable_irq(client->irq);

	return 0;
}

static int raydium_ts_resume(struct i2c_client *client)
{
	struct raydium_ts_priv *priv = i2c_get_clientdata(client);

	int ret = 0;
	char buf[2];
	struct i2c_msg msg[1];

	buf[0]=0x7A;
	buf[1]=0x01;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = &buf;
	ret=i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		printk("Show PID ... error\n");
	}

	enable_irq(client->irq);

	//hrtimer_start(&priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}

static const struct i2c_device_id raydium_ts_id[] = {
	{ RM310XX_I2C_TS_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, raydium_ts_id);

static struct i2c_driver raydium_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "raydium_ts",
	 },
	.probe = raydium_ts_probe,
	.remove = raydium_ts_remove,
	.suspend = raydium_ts_suspend,
	.resume = raydium_ts_resume,
	.id_table = raydium_ts_id,
};



static int __init raydium_ts_init(void)
{
//	u_int8_t Rdbuf[1];
	raydium_wq = create_singlethread_workqueue("raydium_wq");
//	Rdbuf[0]=0;
	if(!raydium_wq)
		return -ENOMEM;
	return i2c_add_driver(&raydium_ts_driver);
}

static void __exit raydium_ts_exit(void)
{
	i2c_del_driver(&raydium_ts_driver);
	if (raydium_wq)
		destroy_workqueue(raydium_wq);
}

MODULE_DESCRIPTION("Raydium Touchscreen Driver");
MODULE_AUTHOR("YH Chen<http://www.rad-ic.com>");
MODULE_LICENSE("Dual BSD/GPL");

module_init(raydium_ts_init);
module_exit(raydium_ts_exit);

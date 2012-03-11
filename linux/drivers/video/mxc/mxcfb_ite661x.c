/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxcfb_ite661x.c
 *
 * @brief MXC Frame buffer driver for ite661x
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mxcfb.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/mxc_edid.h>

#include "cat6611_drv.h"
#include "cat6611_sys.h"

#define IPU_DISP_PORT 0
#define SII_EDID_LEN	256
#define MXC_ENABLE	1
#define MXC_DISABLE	2
static int g_enable_hdmi;

struct ite661x_data {
	struct platform_device *pdev;
	struct i2c_client *client;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct mxc_edid_cfg edid_cfg;
	u8 cable_plugin;
	u8 edid[SII_EDID_LEN];
} ite661x;

static void (*ite661x_reset) (void);

static ssize_t ite661x_show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	strcpy(buf, ite661x.fbi->fix.id);
	sprintf(buf+strlen(buf), "\n");

	return strlen(buf);
}

static DEVICE_ATTR(fb_name, S_IRUGO, ite661x_show_name, NULL);

static ssize_t ite661x_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (ite661x.cable_plugin == 0)
		strcpy(buf, "plugout\n");
	else
		strcpy(buf, "plugin\n");

	return strlen(buf);
}

static DEVICE_ATTR(cable_state, S_IRUGO, ite661x_show_state, NULL);

static ssize_t ite661x_show_edid(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, j, len = 0;

	for (j = 0; j < SII_EDID_LEN/16; j++) {
		for (i = 0; i < 16; i++)
			len += sprintf(buf+len, "0x%02X ",
					ite661x.edid[j*16 + i]);
		len += sprintf(buf+len, "\n");
	}

	return len;
}

static DEVICE_ATTR(edid, S_IRUGO, ite661x_show_edid, NULL);

static void ite661x_setup(struct fb_info *fbi)
{
	// TODO.
}

#ifdef CONFIG_FB_MODE_HELPERS
static int ite661x_read_edid(struct fb_info *fbi)
{
	int ret;

	ret = ite661x_edid_read(GetEDIDData, ite661x.edid, &ite661x.edid_cfg, fbi);

	return ret;
}
#else
static int ite661x_read_edid(struct fb_info *fbi)
{
	return -1;
}
#endif

static void det_worker(struct work_struct *work)
{
	BYTE HPD, HPDChange ;
	char event_string[16];
	char *envp[] = { event_string, NULL };

	CheckHDMITX(&HPD, &HPDChange);
	if (HPDChange) {
		/* cable connection changes */
		if (HPD) {
			ite661x.cable_plugin = 1;
			sprintf(event_string, "EVENT=plugin");

			/* make sure fb is powerdown */
			acquire_console_sem();
			fb_blank(ite661x.fbi, FB_BLANK_POWERDOWN);
			release_console_sem();

			if (ite661x_read_edid(ite661x.fbi) < 0)
				dev_err(&ite661x.client->dev,
					"ite661x: read edid fail\n");
			else {
				ParseEDID(ite661x.edid + 128);
				if (ite661x.fbi->monspecs.modedb_len > 0) {
					int i;
					const struct fb_videomode *mode;
					struct fb_videomode m;

					fb_destroy_modelist(&ite661x.fbi->modelist);

					for (i = 0; i < ite661x.fbi->monspecs.modedb_len; i++) {
						/*FIXME now we do not support interlaced mode */
						if (!(ite661x.fbi->monspecs.modedb[i].vmode & FB_VMODE_INTERLACED))
							fb_add_videomode(&ite661x.fbi->monspecs.modedb[i],
									&ite661x.fbi->modelist);
					}

					fb_var_to_videomode(&m, &ite661x.fbi->var);
					mode = fb_find_nearest_mode(&m,
							&ite661x.fbi->modelist);

					fb_videomode_to_var(&ite661x.fbi->var, mode);

					ite661x.fbi->var.activate |= FB_ACTIVATE_FORCE;
					acquire_console_sem();
					ite661x.fbi->flags |= FBINFO_MISC_USEREVENT;
					fb_set_var(ite661x.fbi, &ite661x.fbi->var);
					ite661x.fbi->flags &= ~FBINFO_MISC_USEREVENT;
					release_console_sem();
				}

				acquire_console_sem();
				fb_blank(ite661x.fbi, FB_BLANK_UNBLANK);
				release_console_sem();
				HDMITX_SetOutput();
			}
		} else {
			ite661x.cable_plugin = 0;
			sprintf(event_string, "EVENT=plugout");
			acquire_console_sem();
			fb_blank(ite661x.fbi, FB_BLANK_POWERDOWN);
			release_console_sem();
			DisableAudioOutput();
			DisableVideoOutput();
		}
		kobject_uevent_env(&ite661x.pdev->dev.kobj, KOBJ_CHANGE, envp);
	}
	enable_irq(ite661x.client->irq);
}

static irqreturn_t ite661x_detect_handler(int irq, void *data)
{
	printk("Inside ite661x_detect_handler...\n");
	disable_irq_nosync(irq);
	if (ite661x.fbi)
		schedule_delayed_work(&(ite661x.det_work), msecs_to_jiffies(20));
	return IRQ_HANDLED;
}


static int ite661x_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;

	/* assume ite661x on DI0 only */
	if ((IPU_DISP_PORT)) {
		if (strcmp(event->info->fix.id, "DISP3 BG - DI1"))
			return 0;
	} else {
		if (strcmp(event->info->fix.id, "DISP3 BG"))
			return 0;
	}

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		if (ite661x.fbi != NULL)
			break;
		ite661x.fbi = fbi;
		break;
	case FB_EVENT_MODE_CHANGE:
		ite661x_setup(fbi);
		break;
	case FB_EVENT_BLANK:
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = ite661x_fb_event,
};

static int __devinit ite661x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct mxc_lcd_platform_data *plat = client->dev.platform_data;
	struct fb_info edid_fbi;

	if (plat->boot_enable &&
		!g_enable_hdmi)
		g_enable_hdmi = MXC_ENABLE;
	if (!g_enable_hdmi)
		g_enable_hdmi = MXC_DISABLE;

	if (g_enable_hdmi == MXC_DISABLE) {
		printk(KERN_WARNING "By setting, ITE driver will not be enabled\n");
		return -ENODEV;
	}

	ite661x.client = client;

	if (plat->reset) {
		ite661x_reset = plat->reset;
		ite661x_reset();
	}

	HDMITX_ChangeDisplayOption(HDMI_480p60, HDMI_RGB444) ;
	InitCAT6611(client);

	if (ite661x.client->irq) {
		ret = request_irq(ite661x.client->irq, ite661x_detect_handler,
				IRQF_TRIGGER_FALLING,
				"ite661x_det", &ite661x);
		if (ret < 0)
			dev_warn(&ite661x.client->dev,
				"ite661x: cound not request det irq %d\n",
				ite661x.client->irq);
		else {
			/*enable cable hot plug irq*/
			INIT_DELAYED_WORK(&(ite661x.det_work), det_worker);
		}
		ret = device_create_file(&ite661x.pdev->dev, &dev_attr_fb_name);
		if (ret < 0)
			dev_warn(&ite661x.client->dev,
				"ite661x: cound not create sys node for fb name\n");
		ret = device_create_file(&ite661x.pdev->dev, &dev_attr_cable_state);
		if (ret < 0)
			dev_warn(&ite661x.client->dev,
				"ite661x: cound not create sys node for cable state\n");
		ret = device_create_file(&ite661x.pdev->dev, &dev_attr_edid);
		if (ret < 0)
			dev_warn(&ite661x.client->dev,
				"ite661x: cound not create sys node for edid\n");
	}

	fb_register_client(&nb);

	return 0;
}

static int __devexit ite661x_remove(struct i2c_client *client)
{
	fb_unregister_client(&nb);
	DisableAudioOutput();
	DisableVideoOutput();
	return 0;
}

static int ite661x_suspend(struct i2c_client *client, pm_message_t message)
{
	/*TODO*/
	return 0;
}

static int ite661x_resume(struct i2c_client *client)
{
	/*TODO*/
	return 0;
}

static const struct i2c_device_id ite661x_id[] = {
	{ "ite661x", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ite661x_id);

static struct i2c_driver ite661x_i2c_driver = {
	.driver = {
		   .name = "ite661x",
		   },
	.probe = ite661x_probe,
	.remove = ite661x_remove,
	.suspend = ite661x_suspend,
	.resume = ite661x_resume,
	.id_table = ite661x_id,
};

static int __init ite661x_init(void)
{
	int ret;

	memset(&ite661x, 0, sizeof(ite661x));

	ite661x.pdev = platform_device_register_simple("ite661x", 0, NULL, 0);
	if (IS_ERR(ite661x.pdev)) {
		printk(KERN_ERR
				"Unable to register ite661x as a platform device\n");
		ret = PTR_ERR(ite661x.pdev);
		goto err;
	}

	return i2c_add_driver(&ite661x_i2c_driver);
err:
	return ret;
}

static void __exit ite661x_exit(void)
{
	i2c_del_driver(&ite661x_i2c_driver);
	platform_device_unregister(ite661x.pdev);
}

static int __init enable_hdmi_setup(char *options)
{
	if (!strcmp(options, "=off"))
		g_enable_hdmi = MXC_DISABLE;
	else
		g_enable_hdmi = MXC_ENABLE;

	return 1;
}
__setup("hdmi", enable_hdmi_setup);

module_init(ite661x_init);
module_exit(ite661x_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ITE661x HDMI driver");
MODULE_LICENSE("GPL");

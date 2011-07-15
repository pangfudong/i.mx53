/*
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <sound/soc.h>

#include <asm/mach/flash.h>
#endif

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/keypad.h>
#include <asm/arch/memory.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc.h>
#include <asm/arch/spba.h>
#include <asm/arch/pmic_power.h>

#include "board-mx3_3stack.h"
#include "crm_regs.h"
#include "iomux.h"
/*!
 * @file mach-mx3/mx3_3stack.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX31
 */

extern void mxc_map_io(void);
extern void mxc_init_irq(void);
extern void mxc_cpu_init(void) __init;
extern struct sys_timer mxc_timer;
extern void mxc_cpu_common_init(void);
extern int mxc_clocks_init(void);
extern void __init early_console_setup(char *);
extern int mxc_init_devices(void);

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

unsigned long board_get_ckih_rate(void)
{
	return 26000000;
}

#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)
static u16 keymapping[12] = {
	KEY_RIGHT,		KEY_LEFT,		KEY_DOWN,	KEY_UP,
	KEY_VOLUMEUP,	KEY_MENU,		KEY_ESC,	KEY_ENTER,
	KEY_PAGEDOWN,	KEY_PAGEUP,		KEY_F9,		KEY_VOLUMEDOWN,
};

static struct resource mxc_kpp_resources[] = {
	[0] = {
	       .start = MXC_INT_KPP,
	       .end = MXC_INT_KPP,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct keypad_data keypad_plat_data = {
	.rowmax = 3,
	.colmax = 4,
	.irq = MXC_INT_KPP,
	.learning = 0,
	.delay = 2,
	.matrix = keymapping,
};

/* mxc keypad driver */
static struct platform_device mxc_keypad_device = {
	.name = "mxc_keypad",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_kpp_resources),
	.resource = mxc_kpp_resources,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &keypad_plat_data,
		},
};

static void mxc_init_keypad(void)
{
	(void)platform_device_register(&mxc_keypad_device);
}
#else
static inline void mxc_init_keypad(void)
{
}
#endif

/* MTD NAND flash */
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) || defined(CONFIG_MTD_NAND_MXC_V2_MODULE)

static struct mtd_partition mxc_nand_partitions[] = {
	{
	 .name = "nand.bootloader",
	 .offset = 0,
	 .size = 1 * 1024 * 1024},
	{
	 .name = "nand.bootsplash",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 1 * 1024 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 2 * 1024 * 1024},
	{
	 .name = "nand.initramfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 3 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 160 * 1024 * 1024},
	{
	 .name = "nand.configure",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 32 * 1024 * 1024},
#if defined(CONFIG_MTD_8G_NAND_FLASH)
	{
	 .name = "nand.firmware",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 130 * 1024 * 1024},
#endif
	{
	 .name = "nand.userfs",
	 .offset = MTDPART_OFS_APPEND,
#if defined(CONFIG_MTD_8G_NAND_FLASH)
	 .size = 694 * 1024 * 1024},
#else
	 .size = 312 * 1024 * 1024},
#endif
};

static struct flash_platform_data mxc_nand_data = {
	.parts = mxc_nand_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nand_partitions),
	.width = 1,
};

static struct platform_device mxc_nand_mtd_device = {
	.name = "mxc_nand_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static struct platform_device mxc_nandv2_mtd_device = {
	.name = "mxc_nandv2_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static void mxc_init_nand_mtd(void)
{
	if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B) {
		mxc_nand_data.width = 2;
	}
	if (cpu_is_mx31()) {
		(void)platform_device_register(&mxc_nand_mtd_device);
	}
	if (cpu_is_mx32()) {
		(void)platform_device_register(&mxc_nandv2_mtd_device);
	}
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

static struct i2c_board_info mxc_i2c_board_info[] __initdata = {
	{
	 .type = "idt1339",
	 .addr = 0x68,
	 .platform_data = NULL},
	{
#if defined(CONFIG_SND_SOC_WM8731) || defined(CONFIG_SND_SOC_WM8731_MODULE)
	 .type = "wm8731",
#else
	 .type = "wm8711",
#endif
	 .addr = 0x1A,
	 .platform_data = NULL},
	{
	 .type = "lds6107",
	 .addr = 0x2C,
	 .platform_data = NULL},
	{
	 .type = "lp3971",
	 .addr = 0x34,
	 .platform_data = NULL},
};

/* MMC device data */

#if defined(CONFIG_MMC_MXC) || defined(CONFIG_MMC_MXC_MODULE)
static struct mxc_mmc_platform_data mmc0_data = {
	.ocr_mask = MMC_VDD_32_33,
	.min_clk = 150000,
	.max_clk = 25000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.power_mmc = NULL,
};

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc1_resources[] = {
	[0] = {
	       .start = MMC_SDHC1_BASE_ADDR,
	       .end = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC1,
	       .end = MXC_INT_MMC_SDHC1,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.min_clk = 150000,
	.max_clk = 25000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.power_mmc = NULL,
};

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc2_resources[] = {
	[0] = {
	       .start = MMC_SDHC2_BASE_ADDR,
	       .end = MMC_SDHC2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC2,
	       .end = MXC_INT_MMC_SDHC2,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Device Definition for MXC SDHC1 */
static struct platform_device mxcsdhc1_device = {
	.name = "mxcmci",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc0_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc1_resources),
	.resource = mxcsdhc1_resources,
};

/*! Device Definition for MXC SDHC2 */
static struct platform_device mxcsdhc2_device = {
	.name = "mxcmci",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc1_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc2_resources),
	.resource = mxcsdhc2_resources,
};

static inline void mxc_init_mmc(void)
{
	int cd_irq;

	cd_irq = sdhc_init_card_det(0);
	if (cd_irq) {
		mxcsdhc1_device.resource[2].start = cd_irq;
		mxcsdhc1_device.resource[2].end = cd_irq;
	}

	cd_irq = sdhc_init_card_det(1);
	if (cd_irq) {
		mxcsdhc2_device.resource[2].start = cd_irq;
		mxcsdhc2_device.resource[2].end = cd_irq;
	}

	spba_take_ownership(SPBA_SDHC1, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc1_device);

	spba_take_ownership(SPBA_SDHC2, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

#if defined(CONFIG_SND_SOC_WM8731) || defined(CONFIG_SND_SOC_WM8731_MODULE)
const char imx_3stack_audio[32] = "wm8731-imx-3stack-audio";
#else
const char imx_3stack_audio[32] = "wm8711-imx-3stack-audio";
#endif
EXPORT_SYMBOL_GPL(imx_3stack_audio);

static struct snd_soc_machine wm8711_machine_driver;
static struct mxc_audio_platform_data imx_3stack_audio_platform_data = {
	.ssi_num = 2,
	.src_port = 2,
	.ext_port = 4,
	.regulator1 = "DCDC6",
	.regulator2 = "DCDC3"
};

/*!
 * Register sound device.
 */
static int mxc_init_audio(void)
{
	struct platform_device* imx_snd_device = platform_device_alloc(imx_3stack_audio, -1);
	if (!imx_snd_device)
	{
		return -ENOMEM;
	}

#if defined(CONFIG_SND_SOC_WM8731) || defined(CONFIG_SND_SOC_WM8731_MODULE)
	/* Setup speaker enable pin */
	mxc_request_iomux(MX31_PIN_DTR_DCE2, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_set_gpio_direction(MX31_PIN_DTR_DCE2, 0);
#endif

	imx_snd_device->dev.platform_data = &imx_3stack_audio_platform_data;
	platform_set_drvdata(imx_snd_device, &wm8711_machine_driver);
	return platform_device_add(imx_snd_device);
}

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	mxc_cpu_init();

#ifdef CONFIG_DISCONTIGMEM
	do {
		int nid;
		mi->nr_banks = MXC_NUMNODES;
		for (nid = 0; nid < mi->nr_banks; nid++) {
			SET_NODE(mi, nid);
		}
	} while (0);
#endif
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	/* config CS5 for debug board */
	mxc_request_iomux(MX31_PIN_CS5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

	mxc_cpu_common_init();
	mxc_clocks_init();
	mxc_gpio_init();
	early_console_setup(saved_command_line);
	mxc_init_devices();

	/*Pull down MX31_PIN_USB_BYP to reset USB3317 */
	mxc_request_iomux(MX31_PIN_USB_BYP, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	mxc_set_gpio_direction(MX31_PIN_USB_BYP, 0);
	mxc_set_gpio_dataout(MX31_PIN_USB_BYP, 1);
	mxc_free_iomux(MX31_PIN_USB_BYP, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);

	mxc_init_keypad();
	mxc_init_nand_mtd();
	i2c_register_board_info(0, mxc_i2c_board_info,
				ARRAY_SIZE(mxc_i2c_board_info));
	mxc_init_mmc();
	mxc_init_audio();
}

#define PLL_PCTL_REG(pd, mfd, mfi, mfn)		\
	((((pd) - 1) << 26) + (((mfd) - 1) << 16) + ((mfi)  << 10) + mfn)

/* For 26MHz input clock */
#define PLL_532MHZ		PLL_PCTL_REG(1, 13, 10, 3)
#define PLL_399MHZ		PLL_PCTL_REG(1, 52, 7, 35)
#define PLL_133MHZ		PLL_PCTL_REG(2, 26, 5, 3)

#define PDR0_REG(mcu, max, hsp, ipg, nfc)	\
	(MXC_CCM_PDR0_MCU_DIV_##mcu | MXC_CCM_PDR0_MAX_DIV_##max | \
	 MXC_CCM_PDR0_HSP_DIV_##hsp | MXC_CCM_PDR0_IPG_DIV_##ipg | \
	 MXC_CCM_PDR0_NFC_DIV_##nfc)

/* working point(wp): 0 - 133MHz; 1 - 266MHz; 2 - 399MHz; 3 - 532MHz */
/* 26MHz input clock table */
static struct cpu_wp cpu_wp_26[] = {
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = PDR0_REG(4, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = PDR0_REG(2, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = PDR0_REG(1, 3, 3, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdr0_reg = PDR0_REG(1, 4, 4, 2, 6),},
};

struct cpu_wp *get_cpu_wp(int *wp)
{
	*wp = 4;
	return cpu_wp_26;
}

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX3_3STACK data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX31_3DS, "Freescale i.MX31L")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mxc_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END

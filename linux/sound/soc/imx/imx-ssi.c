/*
 * imx-ssi.c  --  SSI driver for Freescale IMX
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Based on mxc-alsa-mc13783 (C) 2006-2008 Freescale Semiconductor, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    29th Aug 2006   Initial version.
 *
 * TODO:
 *   Need to rework SSI register defs when new defs go into mainline.
 *   Add support for TDM and FIFO 1.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/arch/dma.h>
#include <asm/arch/clock.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>

#include "imx-ssi.h"
#include "imx-pcm.h"

/* debug */
#define IMX_SSI_DEBUG 0
#if IMX_SSI_DEBUG
#define dbg(format, arg...) printk(format, ## arg)
#else
#define dbg(format, arg...)
#endif

#define IMX_SSI_DUMP 0
#if IMX_SSI_DUMP
#define SSI_DUMP() \
	do { \
		printk(KERN_INFO "dump @ %s\n", __func__); \
		printk(KERN_INFO "scr %x\t, %x\n", SSI1_SCR, SSI2_SCR); \
		printk(KERN_INFO "sisr %x\t, %x\n", SSI1_SISR, SSI2_SISR); \
		printk(KERN_INFO "stcr %x\t, %x\n", SSI1_STCR, SSI2_STCR); \
		printk(KERN_INFO "srcr %x\t, %x\n", SSI1_SRCR, SSI2_SRCR); \
		printk(KERN_INFO "stccr %x\t, %x\n", SSI1_STCCR, SSI2_STCCR); \
		printk(KERN_INFO "srccr %x\t, %x\n", SSI1_SRCCR, SSI2_SRCCR); \
		printk(KERN_INFO "sfcsr %x\t, %x\n", SSI1_SFCSR, SSI2_SFCSR); \
		printk(KERN_INFO "stmsk %x\t, %x\n", SSI1_STMSK, SSI2_STMSK); \
		printk(KERN_INFO "srmsk %x\t, %x\n", SSI1_SRMSK, SSI2_SRMSK); \
		printk(KERN_INFO "sier %x\t, %x\n", SSI1_SIER, SSI2_SIER); \
	} while (0);
#else
#define SSI_DUMP()
#endif

#define SSI1_PORT	0
#define SSI2_PORT	1

static int ssi_active[2] = { 0, 0 };

/*
 * SSI system clock configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				  int clk_id, unsigned int freq, int dir)
{
	u32 scr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		scr = SSI1_SCR;
	else
		scr = SSI2_SCR;

	if (scr & SSI_SCR_SSIEN)
		return 0;

	switch (clk_id) {
	case IMX_SSP_SYS_CLK:
		if (dir == SND_SOC_CLOCK_OUT)
			scr |= SSI_SCR_SYS_CLK_EN;
		else
			scr &= ~SSI_SCR_SYS_CLK_EN;
		break;
	default:
		return -EINVAL;
	}

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		SSI1_SCR = scr;
	else
		SSI2_SCR = scr;

	return 0;
}

/*
 * SSI Clock dividers
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				  int div_id, int div)
{
	u32 stccr, srccr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		if (SSI1_SCR & SSI_SCR_SSIEN)
			return 0;

		srccr = SSI1_SRCCR;
		stccr = SSI1_STCCR;
	} else {
		if (SSI2_SCR & SSI_SCR_SSIEN)
			return 0;

		srccr = SSI2_SRCCR;
		stccr = SSI2_STCCR;
	}

	switch (div_id) {
	case IMX_SSI_TX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	case IMX_SSI_RX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	default:
		return -EINVAL;
	}

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_STCCR = stccr;
		SSI1_SRCCR = srccr;
	} else {
		SSI2_STCCR = stccr;
		SSI2_SRCCR = srccr;
	}
	return 0;
}

/*
 * SSI Network Mode or TDM slots configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
				    unsigned int mask, int slots)
{
	u32 stmsk, srmsk, stccr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		if (SSI1_SCR & SSI_SCR_SSIEN)
			return 0;
		stccr = SSI1_STCCR;
	} else {
		if (SSI2_SCR & SSI_SCR_SSIEN)
			return 0;
		stccr = SSI2_STCCR;
	}

	stmsk = srmsk = mask;
	stccr &= ~SSI_STCCR_DC_MASK;
	stccr |= SSI_STCCR_DC(slots - 1);

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_STMSK = stmsk;
		SSI1_SRMSK = srmsk;
		SSI1_SRCCR = SSI1_STCCR = stccr;
	} else {
		SSI2_STMSK = stmsk;
		SSI2_SRMSK = srmsk;
		SSI2_SRCCR = SSI2_STCCR = stccr;
	}

	return 0;
}

/*
 * SSI DAI format configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 * Note: We don't use the I2S modes but instead manually configure the
 * SSI for I2S.
 */
static int imx_ssi_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	u32 stcr = 0, srcr = 0, scr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		scr = SSI1_SCR & ~(SSI_SCR_SYN | SSI_SCR_NET);
	else
		scr = SSI2_SCR & ~(SSI_SCR_SYN | SSI_SCR_NET);

	if (scr & SSI_SCR_SSIEN)
		return 0;

	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* data on rising edge of bclk, frame low 1clk before data */
		stcr |= SSI_STCR_TFSI | SSI_STCR_TEFS | SSI_STCR_TXBIT0;
		srcr |= SSI_SRCR_RFSI | SSI_SRCR_REFS | SSI_SRCR_RXBIT0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* data on rising edge of bclk, frame high with data */
		stcr |= SSI_STCR_TXBIT0;
		srcr |= SSI_SRCR_RXBIT0;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* data on rising edge of bclk, frame high with data */
		stcr |= SSI_STCR_TFSL;
		srcr |= SSI_SRCR_RFSL;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* data on rising edge of bclk, frame high 1clk before data */
		stcr |= SSI_STCR_TFSL | SSI_STCR_TEFS;
		srcr |= SSI_SRCR_RFSL | SSI_SRCR_REFS;
		break;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		stcr &= ~(SSI_STCR_TSCKP | SSI_STCR_TFSI);
		srcr &= ~(SSI_SRCR_RSCKP | SSI_SRCR_RFSI);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		stcr |= SSI_STCR_TFSI;
		stcr &= ~SSI_STCR_TSCKP;
		srcr |= SSI_SRCR_RFSI;
		srcr &= ~SSI_SRCR_RSCKP;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		stcr &= ~SSI_STCR_TFSI;
		stcr |= SSI_STCR_TSCKP;
		srcr &= ~SSI_SRCR_RFSI;
		srcr |= SSI_SRCR_RSCKP;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		stcr |= SSI_STCR_TFSI | SSI_STCR_TSCKP;
		srcr |= SSI_SRCR_RFSI | SSI_SRCR_RSCKP;
		break;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		stcr |= SSI_STCR_TFDIR | SSI_STCR_TXDIR;
		srcr |= SSI_SRCR_RFDIR | SSI_SRCR_RXDIR;
		if (((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S)
		    && (fmt & SND_SOC_DAIFMT_TDM)) {
			scr &= ~SSI_SCR_I2S_MODE_MASK;
			scr |= SSI_SCR_I2S_MODE_MSTR;
		}
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		stcr |= SSI_STCR_TFDIR;
		srcr |= SSI_SRCR_RFDIR;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		stcr |= SSI_STCR_TXDIR;
		srcr |= SSI_SRCR_RXDIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		if (((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S)
		    && (fmt & SND_SOC_DAIFMT_TDM)) {
			scr &= ~SSI_SCR_I2S_MODE_MASK;
			scr |= SSI_SCR_I2S_MODE_SLAVE;
		}
		break;
	}

	/* sync */
	if (!(fmt & SND_SOC_DAIFMT_ASYNC))
		scr |= SSI_SCR_SYN;

	/* tdm - only for stereo atm */
	if (fmt & SND_SOC_DAIFMT_TDM)
		scr |= SSI_SCR_NET;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_STCR = stcr;
		SSI1_SRCR = srcr;
		SSI1_SCR = scr;
	} else {
		SSI2_STCR = stcr;
		SSI2_SRCR = srcr;
		SSI2_SCR = scr;
	}
	SSI_DUMP();
	return 0;
}

static struct clk *ssi1_clk;
static struct clk *ssi2_clk;

static int imx_ssi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;

	/* we cant really change any SSI values after SSI is enabled
	 * need to fix in software for max flexibility - lrg */
	if (cpu_dai->playback_active || cpu_dai->capture_active)
		return 0;

	/* reset the SSI port - Sect 45.4.4 */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {

		if (ssi_active[SSI1_PORT]++)
			return 0;

		SSI1_SCR = 0;
		ssi1_clk = clk_get(NULL, "ssi_clk.0");
		clk_enable(ssi1_clk);

		/* BIG FAT WARNING
		 * SDMA FIFO watermark must == SSI FIFO watermark for
		 * best results.
		 */
		SSI1_SFCSR = SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
		    SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
		    SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
		    SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK);
		SSI1_SIER = 0;
	} else {

		if (ssi_active[SSI2_PORT]++)
			return 0;

		SSI2_SCR = 0;
		ssi2_clk = clk_get(NULL, "ssi_clk.1");
		clk_enable(ssi2_clk);

		/* above warning applies here too */
		SSI2_SFCSR = SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
		    SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
		    SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
		    SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK);
		SSI2_SIER = 0;
	}

	SSI_DUMP();
	return 0;
}

static int imx_ssi_hw_tx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	u32 stccr, stcr, sier;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		stccr = SSI1_STCCR & ~SSI_STCCR_WL_MASK;
		stcr = SSI1_STCR;
		sier = SSI1_SIER;
	} else {
		stccr = SSI2_STCCR & ~SSI_STCCR_WL_MASK;
		stcr = SSI2_STCR;
		sier = SSI2_SIER;
	}

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		stccr |= SSI_STCCR_WL(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		stccr |= SSI_STCCR_WL(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		stccr |= SSI_STCCR_WL(24);
		break;
	}

	/* enable interrupts */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI2)
		stcr |= SSI_STCR_TFEN0;
	else
		stcr |= SSI_STCR_TFEN1;
	sier |= SSI_SIER_TDMAE | SSI_SIER_TIE | SSI_SIER_TUE0_EN;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_STCR = stcr;
		SSI1_STCCR = stccr;
		SSI1_SIER = sier;
	} else {
		SSI2_STCR = stcr;
		SSI2_STCCR = stccr;
		SSI2_SIER = sier;
	}

	return 0;
}

static int imx_ssi_hw_rx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	u32 srccr, srcr, sier;
	bool sync_mode;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		sync_mode = ((SSI1_SCR & SSI_SCR_SYN) ? true : false);
		srccr = sync_mode ? SSI1_STCCR : SSI1_SRCCR;
		srcr = SSI1_SRCR;
		sier = SSI1_SIER;
	} else {
		sync_mode = ((SSI2_SCR & SSI_SCR_SYN) ? true : false);
		srccr = sync_mode ? SSI2_STCCR : SSI2_SRCCR;
		srcr = SSI2_SRCR;
		sier = SSI2_SIER;
	}
	srccr &= ~SSI_SRCCR_WL_MASK;

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		srccr |= SSI_SRCCR_WL(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		srccr |= SSI_SRCCR_WL(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		srccr |= SSI_SRCCR_WL(24);
		break;
	}

	/* enable interrupts */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI2)
		srcr |= SSI_SRCR_RFEN0;
	else
		srcr |= SSI_SRCR_RFEN1;
	sier |= SSI_SIER_RDMAE | SSI_SIER_RIE | SSI_SIER_ROE0_EN;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_SRCR = srcr;
		if (sync_mode)
			SSI1_STCCR = srccr;
		else
			SSI1_SRCCR = srccr;
		SSI1_SIER = sier;
	} else {
		SSI2_SRCR = srcr;
		if (sync_mode)
			SSI2_STCCR = srccr;
		else
			SSI2_SRCCR = srccr;
		SSI2_SIER = sier;
	}
	return 0;
}

/*
 * Should only be called when port is inactive (i.e. SSIEN = 0),
 * although can be called multiple times by upper layers.
 */
static int imx_ssi_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	int id;

	id = cpu_dai->id;

	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* cant change any parameters when SSI is running */
		if (id == IMX_DAI_SSI0 || id == IMX_DAI_SSI1) {
			if ((SSI1_SCR & SSI_SCR_SSIEN) &&
			    (SSI1_SCR & SSI_SCR_TE))
				return 0;
		} else {
			if ((SSI2_SCR & SSI_SCR_SSIEN) &&
			    (SSI2_SCR & SSI_SCR_TE))
				return 0;
		}
		return imx_ssi_hw_tx_params(substream, params);
	} else {
		/* cant change any parameters when SSI is running */
		if (id == IMX_DAI_SSI0 || id == IMX_DAI_SSI1) {
			if ((SSI1_SCR & SSI_SCR_SSIEN) &&
			    (SSI1_SCR & SSI_SCR_RE))
				return 0;
		} else {
			if ((SSI2_SCR & SSI_SCR_SSIEN) &&
			    (SSI2_SCR & SSI_SCR_RE))
				return 0;
		}
		return imx_ssi_hw_rx_params(substream, params);
	}
}

static int imx_ssi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	u32 scr;

	/* enable the SSI port, note that no other port config
	 * should happen after SSIEN is set */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		scr = SSI1_SCR;
		SSI1_SCR = scr | SSI_SCR_SSIEN;
	} else {
		scr = SSI2_SCR;
		SSI2_SCR = scr | SSI_SCR_SSIEN;
	}
	SSI_DUMP();
	return 0;
}

static int imx_ssi_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	u32 scr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		scr = SSI1_SCR;
	else
		scr = SSI2_SCR;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr |= SSI_SCR_TE;
		else
			scr |= SSI_SCR_RE;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr &= ~SSI_SCR_TE;
		else
			scr &= ~SSI_SCR_RE;
		break;
	default:
		return -EINVAL;
	}

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		SSI1_SCR = scr;
	else
		SSI2_SCR = scr;

	SSI_DUMP();
	return 0;
}

static void imx_ssi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	int id;

	id = cpu_dai->id;

	/* shutdown SSI if neither Tx or Rx is active */
	if (cpu_dai->playback_active || cpu_dai->capture_active)
		return;

	if (id == IMX_DAI_SSI0 || id == IMX_DAI_SSI1) {

		if (--ssi_active[SSI1_PORT] > 1)
			return;

		SSI1_SCR = 0;

		clk_disable(ssi1_clk);
		clk_put(ssi1_clk);

	} else {
		if (--ssi_active[SSI2_PORT])
			return;
		SSI2_SCR = 0;
		clk_disable(ssi2_clk);
		clk_put(ssi2_clk);
	}
}

#ifdef CONFIG_PM
static int imx_ssi_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);

	if (!dai->active)
		return 0;

	// do we need to disable any clocks

	return 0;
}

static int imx_ssi_resume(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);

	if (!dai->active)
		return 0;

	// do we need to enable any clocks
	return 0;
}

#else
#define imx_ssi_suspend	NULL
#define imx_ssi_resume	NULL
#endif

#define IMX_SSI_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
	SNDRV_PCM_RATE_96000)

#define IMX_SSI_BITS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_pcm_stream imx_ssi_playback = {
	.stream_name = "Playback",
	.channels_min = 1,
	.channels_max = 2,
	.rates = IMX_SSI_RATES,
	.formats = IMX_SSI_BITS,
};

static const struct snd_soc_pcm_stream imx_ssi_capture = {
	.stream_name = "Capture",
	.channels_min = 1,
	.channels_max = 2,
	.rates = IMX_SSI_RATES,
	.formats = IMX_SSI_BITS,
};

/* dai ops, called by machine drivers */
static const struct snd_soc_dai_ops imx_ssi_dai_ops = {
	.set_sysclk = imx_ssi_set_dai_sysclk,
	.set_clkdiv = imx_ssi_set_dai_clkdiv,
	.set_fmt = imx_ssi_set_dai_fmt,
	.set_tdm_slot = imx_ssi_set_dai_tdm_slot,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops imx_ssi_audio_ops = {
	.startup = imx_ssi_startup,
	.shutdown = imx_ssi_shutdown,
	.trigger = imx_ssi_trigger,
	.prepare = imx_ssi_prepare,
	.hw_params = imx_ssi_hw_params,
};

const char imx_ssi_1[SND_SOC_DAI_NAME_SIZE] = {
	"imx-ssi-1"
};

EXPORT_SYMBOL_GPL(imx_ssi_1);

const char imx_ssi_2[SND_SOC_DAI_NAME_SIZE] = {
	"imx-ssi-2"
};

EXPORT_SYMBOL_GPL(imx_ssi_2);

const char imx_ssi_3[SND_SOC_DAI_NAME_SIZE] = {
	"imx-ssi-3"
};

EXPORT_SYMBOL_GPL(imx_ssi_3);

const char imx_ssi_4[SND_SOC_DAI_NAME_SIZE] = {
	"imx-ssi-4"
};

EXPORT_SYMBOL_GPL(imx_ssi_4);

static int fifo_err_counter;

static irqreturn_t ssi1_irq(int irq, void *dev_id)
{
	if (fifo_err_counter++ % 1000 == 0)
		printk(KERN_ERR "ssi1_irq SISR %x SIER %x fifo_errs=%d\n",
		       SSI1_SISR, SSI1_SIER, fifo_err_counter);
	SSI1_SISR = SSI_SIER_TUE0_EN | SSI_SIER_ROE0_EN;
	return IRQ_HANDLED;
}

static irqreturn_t ssi2_irq(int irq, void *dev_id)
{
	if (fifo_err_counter++ % 1000 == 0)
		printk(KERN_ERR "ssi2_irq SISR %x SIER %x fifo_errs=%d\n",
		       SSI2_SISR, SSI2_SIER, fifo_err_counter);
	SSI2_SISR = SSI_SIER_TUE0_EN | SSI_SIER_ROE0_EN;
	return IRQ_HANDLED;
}

static int imx_ssi_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);

	if (!strcmp(imx_ssi_1, dai->name))
		dai->id = IMX_DAI_SSI0;
	else if (!strcmp(imx_ssi_2, dai->name))
		dai->id = IMX_DAI_SSI1;
	else if (!strcmp(imx_ssi_3, dai->name))
		dai->id = IMX_DAI_SSI2;
	else if (!strcmp(imx_ssi_4, dai->name))
		dai->id = IMX_DAI_SSI3;
	else {
		printk(KERN_ERR "%s: invalid device %s\n", __func__, dai->name);
		return -ENODEV;
	}

	dai->type = SND_SOC_DAI_PCM;
	dai->ops = &imx_ssi_dai_ops;
	dai->audio_ops = &imx_ssi_audio_ops;
	dai->capture = &imx_ssi_capture;
	dai->playback = &imx_ssi_playback;
	snd_soc_register_cpu_dai(dai);

	if ((!strcmp(imx_ssi_1, dai->name)) || (!strcmp(imx_ssi_2, dai->name)))
		if (request_irq(MXC_INT_SSI1, ssi1_irq, 0, "ssi1", dai)) {
			printk(KERN_ERR "%s: failure requesting irq %s\n",
			       __func__, "ssi1");
			return -EBUSY;
		}

	return 0;
}

static int imx_ssi_remove(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);

	if ((!strcmp(imx_ssi_1, dai->name)) || (!strcmp(imx_ssi_2, dai->name)))
	{
		free_irq(MXC_INT_SSI1, dai);
	}
	if ((!strcmp(imx_ssi_3, dai->name)) || (!strcmp(imx_ssi_4, dai->name)))
	{
		free_irq(MXC_INT_SSI2, dai);
	}

	return 0;
}

static struct snd_soc_device_driver imx_ssi_driver = {
	.type = SND_SOC_BUS_TYPE_DAI,
	.driver = {
		   .name = "imx-ssi",
		   .owner = THIS_MODULE,
		   .bus = &asoc_bus_type,
		   .probe = imx_ssi_probe,
		   .remove = imx_ssi_remove,
		   .suspend = imx_ssi_suspend,
		   .resume = imx_ssi_resume,
		   },
};

static int __init imx_ssi_init(void)
{
	return driver_register(&imx_ssi_driver.driver);
}

static void __exit imx_ssi_exit(void)
{
	driver_unregister(&imx_ssi_driver.driver);
}

module_init(imx_ssi_init);
module_exit(imx_ssi_exit);

/* Module information */
MODULE_AUTHOR
    ("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("i.MX ASoC I2S driver");
MODULE_LICENSE("GPL");

/*
 * imx-3stack-wm8711.c  --  i.MX 3Stack Driver for Wolfson WM8711 Codec
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Copyright 2007-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    19th Jun 2007   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include "imx-ssi.h"
#include "imx-pcm.h"
#include "../codecs/wm8711.h"

extern void gpio_activate_audio_ports(void);

/* SSI BCLK and LRC master */
#define WM8711_SSI_MASTER	1

struct imx_3stack_pcm_state {
	int lr_clk_active;
};

static int imx_3stack_startup(struct snd_pcm_substream *substream)
{
	// TODO: What should I do here?
	printk("%s: %s was called.\n", __FILE__, __FUNCTION__);
	return 0;
}

static int imx_3stack_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	unsigned int channels = params_channels(params);
	u32 dai_format;

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_SYNC;
	if (channels == 2)
		dai_format |= SND_SOC_DAIFMT_TDM;

	/* set codec DAI configuration */
	codec_dai->ops->set_fmt(codec_dai, dai_format);

	/* set cpu DAI configuration */
	cpu_dai->ops->set_fmt(cpu_dai, dai_format);

	/* set the codec system clock for DAC and ADC */
	codec_dai->ops->set_sysclk(codec_dai, WM8711_SYSCLK, 11289600, SND_SOC_CLOCK_IN);

	/* set i.MX active slot mask */
	cpu_dai->ops->set_tdm_slot(cpu_dai,
				   channels == 1 ? 0xfffffffe : 0xfffffffc,
				   channels);

	/* set the SSI system clock as input (unused) */
	cpu_dai->ops->set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);
	return 0;
}

static void imx_3stack_shutdown(struct snd_pcm_substream *substream)
{
	// TODO: What should I do here?
	printk("%s: %s was called.\n", __FILE__, __FUNCTION__);
}

/*
 * imx_3stack WM8711 HiFi DAI opserations.
 */
static struct snd_soc_ops imx_3stack_hifi_ops = {
	.startup = imx_3stack_startup,
	.shutdown = imx_3stack_shutdown,
	.hw_params = imx_3stack_hifi_hw_params,
};

static void imx_3stack_init_dam(int ssi_port, int dai_port)
{
	/* WM8711 uses SSI1 or SSI2 via AUDMUX port dai_port for audio */

	/* reset port ssi_port & dai_port */
	DAM_PTCR(ssi_port) = 0;
	DAM_PDCR(ssi_port) = 0;
	DAM_PTCR(dai_port) = 0;
	DAM_PDCR(dai_port) = 0;

	/* set to synchronous */
	DAM_PTCR(ssi_port) |= AUDMUX_PTCR_SYN;
	DAM_PTCR(dai_port) |= AUDMUX_PTCR_SYN;

	/* set Rx sources ssi_port <--> dai_port */
	DAM_PDCR(ssi_port) |= AUDMUX_PDCR_RXDSEL(dai_port);
	DAM_PDCR(dai_port) |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  dai_port--> ssi_port output */
	DAM_PTCR(ssi_port) |= AUDMUX_PTCR_TFSDIR;
	DAM_PTCR(ssi_port) |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, dai_port);

	/* set Tx Clock direction and source dai_port--> ssi_port output */
	DAM_PTCR(ssi_port) |= AUDMUX_PTCR_TCLKDIR;
	DAM_PTCR(ssi_port) |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, dai_port);
}

static int imx_3stack_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	struct imx_3stack_pcm_state *state;
	int ret;

	state = kzalloc(sizeof(struct imx_3stack_pcm_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	pcm_link->audio_ops = &imx_3stack_hifi_ops;
	pcm_link->private_data = state;

	ret = snd_soc_pcm_new(pcm_link, 1, 1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create hifi pcm\n", __func__);
		kfree(state);
		return ret;
	}

	printk(KERN_INFO "i.MX 3STACK WM8711 Audio Driver\n");

	return 0;
}

static int imx_3stack_pcm_free(struct snd_soc_pcm_link *pcm_link)
{
	kfree(pcm_link->private_data);
	return 0;
}

static const struct snd_soc_pcm_link_ops imx_3stack_pcm_ops = {
	.new = imx_3stack_pcm_new,
	.free = imx_3stack_pcm_free,
};

/* imx_3stack machine dapm widgets */
static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
    SND_SOC_DAPM_HP("Headphone Jack", NULL),
    SND_SOC_DAPM_MIC("Mic Jack", NULL),
    SND_SOC_DAPM_SPK("Ext Spk", NULL),
    SND_SOC_DAPM_LINE("Line Jack", NULL),
    SND_SOC_DAPM_HP("Headset Jack", NULL),
};

/* imx_3stack machine audio map */
static const char *audio_map[][3] = {

	/* headset Jack  - in = micin, out = LHPOUT*/
	{"Headset Jack", NULL, "LHPOUT"},

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},

	/* speaker connected to LOUT, ROUT */
	{"Ext Spk", NULL, "ROUT"},
	{"Ext Spk", NULL, "LOUT"},

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{"MICIN", NULL, "Mic Jack"},

	/* Same as the above but no mic bias for line signals */
	{"MICIN", NULL, "Line Jack"},

	{NULL, NULL, NULL},
};

#ifdef CONFIG_PM
static int imx_3stack_wm8711_audio_suspend(struct platform_device *dev,
					   pm_message_t state)
{

	int ret = 0;

	return ret;
}

static int imx_3stack_wm8711_audio_resume(struct platform_device *dev)
{

	int ret = 0;

	return ret;
}

#else
#define imx_3stack_wm8711_audio_suspend	NULL
#define imx_3stack_wm8711_audio_resume	NULL
#endif

static int mach_probe(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_link *pcm_link;

	int i, ret;

	pcm_link = list_first_entry(&machine->active_list,
				    struct snd_soc_pcm_link, active_list);

	codec = pcm_link->codec;
	codec->ops->io_probe(codec, machine);

	/* Add imx_3stack specific widgets */
	for (i = 0; i < ARRAY_SIZE(imx_3stack_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec,
					 &imx_3stack_dapm_widgets[i]);
	}

	/* set up imx_3stack specific audio path audio map */
	for (i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0],
					   audio_map[i][1], audio_map[i][2]);
	}

	/* connect and enable all imx_3stack WM8711 jacks (for now) */
	snd_soc_dapm_set_endpoint(machine, "Headphone Jack", 1);
	snd_soc_dapm_set_endpoint(machine, "Mic Jack", 1);
	snd_soc_dapm_set_endpoint(machine, "Ext Spk", 1);
	snd_soc_dapm_set_endpoint(machine, "Line Jack", 1);
	snd_soc_dapm_set_endpoint(machine, "Headset Jack", 1);

	snd_soc_dapm_set_policy(machine, SND_SOC_DAPM_POLICY_STREAM);
	snd_soc_dapm_sync_endpoints(machine);

	/* register card with ALSA upper layers */
	ret = snd_soc_register_card(machine);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register sound card\n",
		       __FUNCTION__);
		snd_soc_machine_free(machine);
		return ret;
	}
	return 0;
}

struct snd_soc_machine_ops machine_ops = {
	.mach_probe = mach_probe,
};

static int __devinit imx_3stack_wm8711_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = platform_get_drvdata(pdev);
	struct snd_soc_pcm_link *hifi;
	int ret;
	struct mxc_audio_platform_data *tmp;

	machine->owner = THIS_MODULE;
	machine->pdev = pdev;
	machine->name = "i.MX_3STACK";
	machine->longname = "WM8711";
	machine->ops = &machine_ops;

	/* register card */
	ret =
	    snd_soc_new_card(machine, 1, SNDRV_DEFAULT_IDX1,
			     SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create pcms\n", __func__);
		return ret;
	}

	/* WM8711 hifi interface */
	ret = -ENODEV;
	tmp = pdev->dev.platform_data;

	if (tmp->src_port == 2)
		hifi = snd_soc_pcm_link_new(machine, "imx_3stack-hifi",
					    &imx_3stack_pcm_ops, imx_pcm,
					    wm8711_codec_name, wm8711_dai_name,
					    imx_ssi_3);
	else
		hifi = snd_soc_pcm_link_new(machine, "imx_3stack-hifi",
					    &imx_3stack_pcm_ops, imx_pcm,
					    wm8711_codec_name, wm8711_dai_name,
					    imx_ssi_1);
	if (hifi == NULL) {
		printk("failed to create HiFi PCM link\n");
		snd_soc_machine_free(machine);
		return ret;
	}
	ret = snd_soc_pcm_link_attach(hifi);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to attach hifi pcm\n", __func__);
		snd_soc_machine_free(machine);
		return ret;
	}
	gpio_activate_audio_ports();
	imx_3stack_init_dam(tmp->src_port, tmp->ext_port);

	return ret;

}

static int __devexit
imx_3stack_wm8711_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_link *pcm_link;

	pcm_link = list_first_entry(&machine->active_list,
				    struct snd_soc_pcm_link, active_list);

	codec = pcm_link->codec;
	codec->ops->io_remove(codec, machine);

	snd_soc_machine_free(machine);
	return 0;
}

/* Defined in mx3_3stack.c */
extern const char imx_3stack_audio[32];

static struct platform_driver imx_3stack_wm8711_audio_driver = {
	.probe = imx_3stack_wm8711_audio_probe,
	.remove = __devexit_p(imx_3stack_wm8711_audio_remove),
	.suspend = imx_3stack_wm8711_audio_suspend,
	.resume = imx_3stack_wm8711_audio_resume,
	.driver = {
		   .name = imx_3stack_audio,
		   },
};

static int __init imx_3stack_wm8711_audio_init(void)
{
	return platform_driver_register(&imx_3stack_wm8711_audio_driver);
}

static void __exit imx_3stack_wm8711_audio_exit(void)
{
	platform_driver_unregister(&imx_3stack_wm8711_audio_driver);
}

module_init(imx_3stack_wm8711_audio_init);
module_exit(imx_3stack_wm8711_audio_exit);

MODULE_AUTHOR("Liam Girdwood");
MODULE_DESCRIPTION("WM8711 Machine Driver for i.MX 3STACK");
MODULE_LICENSE("GPL");

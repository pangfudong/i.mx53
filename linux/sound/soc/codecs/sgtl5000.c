/*
 * sgtl5000.c  --  SGTL5000 ALSA SoC Audio driver
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "sgtl5000.h"

struct sgtl5000_priv {
	int sysclk;
	int master;
	int fmt;
	int playback_active;
	int capture_active;
	int rev;
};

static int sgtl5000_dapm_event(struct snd_soc_codec *codec, int event);

struct i2c_client *sgtl5000_i2c_client;

static unsigned int sgtl5000_read(struct snd_soc_codec *codec, unsigned int reg)
{
	int i2c_ret;
	u16 value;
	u16 addr = sgtl5000_i2c_client->addr;
	u16 flags = sgtl5000_i2c_client->flags;
	u8 buf0[2];
	u8 buf1[2];
	struct i2c_msg msg[2] = {
					{addr, flags, 2, buf0},
					{addr, flags | I2C_M_RD, 2, buf1},
				};

	buf0[0] = (reg & 0xff00) >> 8;
	buf0[1] = reg & 0xff;
	i2c_ret = i2c_transfer(sgtl5000_i2c_client->adapter, msg, 2);
	if (i2c_ret < 0) {
		pr_err("%s: read reg error : reg=%x\n", __func__, reg);
		return 0;
	}

	value = buf1[0] << 8 | buf1[1];

	pr_debug("r r:%02x,v:%04x\n", reg, value);
	return value;
}

static int sgtl5000_write(struct snd_soc_codec *codec, unsigned int reg,
			  unsigned int value)
{
	int i2c_ret;
	u16 addr = sgtl5000_i2c_client->addr;
	u16 flags = sgtl5000_i2c_client->flags;
	u8 buf[4];
	struct i2c_msg msg = {addr, flags, 4, buf};

	pr_debug("w r:%02x,v:%04x\n", reg, value);
	buf[0] = (reg & 0xff00) >> 8;
	buf[1] = reg & 0xff;
	buf[2] = (value & 0xff00) >> 8;
	buf[3] = value & 0xff;

	i2c_ret = i2c_transfer(sgtl5000_i2c_client->adapter, &msg, 1);
	if (i2c_ret < 0) {
		pr_err("%s: write reg error : R%02d = 0x%04x\n",
		       __func__, reg, value);
		return -EIO;
	}

	return i2c_ret;
}

#ifdef DEBUG
static int all_reg[] = {
SGTL5000_CHIP_ID,
SGTL5000_CHIP_DIG_POWER,
SGTL5000_CHIP_CLK_CTRL,
SGTL5000_CHIP_I2S_CTRL,
SGTL5000_CHIP_SSS_CTRL,
SGTL5000_CHIP_ADCDAC_CTRL,
SGTL5000_CHIP_DAC_VOL,
SGTL5000_CHIP_PAD_STRENGTH,
SGTL5000_CHIP_ANA_ADC_CTRL,
SGTL5000_CHIP_ANA_HP_CTRL,
SGTL5000_CHIP_ANA_CTRL,
SGTL5000_CHIP_LINREG_CTRL,
SGTL5000_CHIP_REF_CTRL,
SGTL5000_CHIP_MIC_CTRL,
SGTL5000_CHIP_LINE_OUT_CTRL,
SGTL5000_CHIP_LINE_OUT_VOL,
SGTL5000_CHIP_ANA_POWER,
SGTL5000_CHIP_PLL_CTRL,
SGTL5000_CHIP_CLK_TOP_CTRL,
SGTL5000_CHIP_ANA_STATUS,
SGTL5000_CHIP_SHORT_CTRL,
};

static void dump_reg(struct snd_soc_codec *codec)
{
	int i, reg;
	printk(KERN_DEBUG "dump begin\n");
	for (i = 0; i < 21; i++) {
		reg = sgtl5000_read(codec, all_reg[i]);
		printk(KERN_DEBUG "d r %04x, v %04x\n", all_reg[i], reg);
	}
	printk(KERN_DEBUG "dump end\n");
}
#else
static void dump_reg(struct snd_soc_codec *codec)
{
}
#endif

static int dac_mux_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = widget->codec;
	unsigned int reg;

	if (ucontrol->value.enumerated.item[0]) {
		reg = sgtl5000_read(codec, SGTL5000_CHIP_CLK_TOP_CTRL);
		reg |= SGTL5000_INT_OSC_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, reg);

		if (codec->dapm_state != SNDRV_CTL_POWER_D0) {
			sgtl5000_dapm_event(codec, SNDRV_CTL_POWER_D1);
			snd_soc_dapm_put_enum_double(kcontrol, ucontrol);
			sgtl5000_dapm_event(codec, SNDRV_CTL_POWER_D0);
		} else
			snd_soc_dapm_put_enum_double(kcontrol, ucontrol);
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_CTRL);
		reg &= ~(SGTL5000_LINE_OUT_MUTE | SGTL5000_HP_MUTE);
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg);
	} else {
		reg = sgtl5000_read(codec, SGTL5000_CHIP_CLK_TOP_CTRL);
		reg &= ~SGTL5000_INT_OSC_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, reg);

		snd_soc_dapm_put_enum_double(kcontrol, ucontrol);
		sgtl5000_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	}
	return 0;
}


static const char *adc_mux_text[] = {
	"MIC_IN", "LINE_IN"
};

static const char *dac_mux_text[] = {
	"DAC", "LINE_IN"
};

static const struct soc_enum adc_enum =
	SOC_ENUM_SINGLE(SGTL5000_CHIP_ANA_CTRL, 2, 2, adc_mux_text);

static const struct soc_enum dac_enum =
	SOC_ENUM_SINGLE(SGTL5000_CHIP_ANA_CTRL, 6, 2, dac_mux_text);

static const struct snd_kcontrol_new adc_mux =
	SOC_DAPM_ENUM("ADC Mux", adc_enum);

static const struct snd_kcontrol_new dac_mux = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = "DAC Mux",
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE
		  | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.info = snd_soc_info_enum_double,
	.get = snd_soc_dapm_get_enum_double,
	.put = dac_mux_put,
	.private_value = (unsigned long)&dac_enum,
};

static const struct snd_soc_dapm_widget sgtl5000_dapm_widgets[] = {
SND_SOC_DAPM_INPUT("LINE_IN"),
SND_SOC_DAPM_INPUT("MIC_IN"),

SND_SOC_DAPM_OUTPUT("HP_OUT"),
SND_SOC_DAPM_OUTPUT("LINE_OUT"),

SND_SOC_DAPM_MUX("ADC Mux", SND_SOC_NOPM, 0, 0, &adc_mux),
SND_SOC_DAPM_MUX("DAC Mux", SND_SOC_NOPM, 0, 0, &dac_mux),

SND_SOC_DAPM_ADC("ADC", "Capture", SGTL5000_CHIP_DIG_POWER, 6, 0),
SND_SOC_DAPM_DAC("DAC", "Playback", SGTL5000_CHIP_DIG_POWER, 5, 0),
};

static const char *audio_map[][3] = {
	{ "ADC Mux", "LINE_IN", "LINE_IN" },
	{ "ADC Mux", "MIC_IN", "MIC_IN" },
	{ "ADC", NULL, "ADC Mux"},
	{ "DAC Mux", "DAC", "DAC" },
	{ "DAC Mux", "LINE_IN", "LINE_IN" },
	{ "LINE_OUT", NULL, "DAC" },
	{ "HP_OUT", NULL, "DAC Mux" },
	{}
};

static int sgtl5000_add_widgets(struct snd_soc_codec *codec,
			      struct snd_soc_machine *machine)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sgtl5000_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec,
					 &sgtl5000_dapm_widgets[i]);
	}

	/* set up audio path audio_mapnects */
	for (i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0],
					   audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_new_widgets(machine);
	return 0;
}

static int dac_info_volsw(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xfc - 0x3c;
	return 0;
}

static int dac_get_volsw(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;

	reg = sgtl5000_read(codec, SGTL5000_CHIP_DAC_VOL);
	l = (reg & SGTL5000_DAC_VOL_LEFT_MASK) << SGTL5000_DAC_VOL_LEFT_SHIFT;
	r = (reg & SGTL5000_DAC_VOL_RIGHT_MASK) << SGTL5000_DAC_VOL_RIGHT_SHIFT;
	l = l < 0x3c ? 0x3c : l;
	l = l > 0xfc ? 0xfc : l;
	r = r < 0x3c ? 0x3c : r;
	r = r > 0xfc ? 0xfc : r;
	l = 0xfc - l;
	r = 0xfc - r;

	ucontrol->value.integer.value[0] = l;
	ucontrol->value.integer.value[1] = l;

	return 0;
}

static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;

	l = ucontrol->value.integer.value[0];
	r = ucontrol->value.integer.value[1];

	l = l < 0x3c ? 0x3c : l;
	l = l > 0xfc ? 0xfc : l;
	r = r < 0x3c ? 0x3c : r;
	r = r > 0xfc ? 0xfc : r;
	l = 0xfc - l;
	r = 0xfc - r;

	reg = l << SGTL5000_DAC_VOL_LEFT_SHIFT |
	      r << SGTL5000_DAC_VOL_RIGHT_SHIFT;

	sgtl5000_write(codec, SGTL5000_CHIP_DAC_VOL, reg);

	return 0;
}

static const char *mic_gain_text[] = {
	"0dB", "20dB", "30dB", "40dB"
};

static const char *adc_m6db_text[] = {
	"No Change", "Reduced by 6dB"
};

static const struct soc_enum mic_gain =
	SOC_ENUM_SINGLE(SGTL5000_CHIP_MIC_CTRL, 0, 4, mic_gain_text);

static const struct soc_enum adc_m6db =
	SOC_ENUM_SINGLE(SGTL5000_CHIP_ANA_ADC_CTRL, 8, 2, adc_m6db_text);

static const struct snd_kcontrol_new sgtl5000_snd_controls[] = {
SOC_ENUM("MIC GAIN", mic_gain),
SOC_DOUBLE("Capture Volume", SGTL5000_CHIP_ANA_ADC_CTRL, 0, 4, 0xf, 0),
SOC_ENUM("Capture Vol Reduction", adc_m6db),
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = "Playback Volume",
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		  SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.info = dac_info_volsw, .get = dac_get_volsw,
	.put = dac_put_volsw,
},
SOC_DOUBLE("Headphone Volume", SGTL5000_CHIP_ANA_HP_CTRL, 0, 8, 0x7f, 1),
};

static int sgtl5000_add_controls(struct snd_soc_codec *codec,
			       struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(sgtl5000_snd_controls); i++) {
		err = snd_ctl_add(card,
				  snd_soc_cnew(&sgtl5000_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

static int sgtl5000_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg1, reg2;

	reg1 = sgtl5000_read(codec, SGTL5000_CHIP_ANA_CTRL);
	reg2 = sgtl5000_read(codec, SGTL5000_CHIP_ADCDAC_CTRL);

	if (mute) {
		reg1 |= SGTL5000_LINE_OUT_MUTE;
		reg1 |= SGTL5000_HP_MUTE;
		reg1 |= SGTL5000_ADC_MUTE;
		reg2 |= SGTL5000_DAC_MUTE_LEFT;
		reg2 |= SGTL5000_DAC_MUTE_RIGHT;
	} else {
		reg1 &= ~SGTL5000_LINE_OUT_MUTE;
		reg1 &= ~SGTL5000_HP_MUTE;
		reg1 &= ~SGTL5000_ADC_MUTE;
		reg2 &= ~SGTL5000_DAC_MUTE_LEFT;
		reg2 &= ~SGTL5000_DAC_MUTE_RIGHT;
	}

	sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg1);
	sgtl5000_write(codec, SGTL5000_CHIP_ADCDAC_CTRL, reg2);
	if (!mute)
		dump_reg(codec);
	return 0;
}

static int sgtl5000_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;
	u16 i2sctl = 0;
	pr_debug("%s:fmt=%08x\n", __func__, fmt);
	sgtl5000->master = 0;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		i2sctl |= SGTL5000_I2S_MASTER;
		sgtl5000->master = 1;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		i2sctl |= SGTL5000_I2S_MODE_PCM;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		i2sctl |= SGTL5000_I2S_MODE_PCM;
		i2sctl |= SGTL5000_I2S_LRALIGN;
		break;
	case SND_SOC_DAIFMT_I2S:
		i2sctl |= SGTL5000_I2S_MODE_I2S_LJ;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		i2sctl |= SGTL5000_I2S_MODE_RJ;
		i2sctl |= SGTL5000_I2S_LRPOL;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		i2sctl |= SGTL5000_I2S_MODE_I2S_LJ;
		i2sctl |= SGTL5000_I2S_LRALIGN;
		break;
	default:
		return -EINVAL;
	}
	sgtl5000->fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	/* Clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
	case SND_SOC_DAIFMT_NB_IF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
	case SND_SOC_DAIFMT_IB_NF:
		i2sctl |= SGTL5000_I2S_SCLK_INV;
		break;
	default:
		return -EINVAL;
	}
	sgtl5000_write(codec, SGTL5000_CHIP_I2S_CTRL, i2sctl);

	return 0;
}

static int sgtl5000_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;

	sgtl5000->sysclk = freq;

	return 0;
}

/* dai ops, called by machine drivers */
static const struct snd_soc_dai_ops sgtl5000_dai_ops = {
	.digital_mute = sgtl5000_digital_mute,
	.set_fmt = sgtl5000_set_dai_fmt,
	.set_sysclk = sgtl5000_set_dai_sysclk,
};

static int sgtl5000_pcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sgtl5000->playback_active++;
	else
		sgtl5000->capture_active++;

	return 0;
}

static void sgtl5000_pcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;
	int reg;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sgtl5000->playback_active--;
	else
		sgtl5000->capture_active--;

	reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
	reg &= ~(SGTL5000_I2S_IN_POWERUP | SGTL5000_I2S_OUT_POWERUP);
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);
}

/*
 * Set PCM DAI bit size and sample rate.
 * input: params_rate, params_fmt
 */
static int sgtl5000_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;
	int fs = params_rate(params);
	int channels = params_channels(params);
	int clk_ctl = 0;
	int pll_ctl = 0;
	int i2s_ctl;
	int div2 = 0;
	int reg;

	if (!sgtl5000->sysclk) {
		pr_err("%s: set sysclk first!\n", __func__);
		return -EFAULT;
	}

	if (sgtl5000->rev != 0x00) { /* rev 1 not support mono playback */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_TEST2);
		if (channels == 1)
			reg |= SGTL5000_MONO_DAC;
		else
			reg &= ~SGTL5000_MONO_DAC;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_TEST2, reg);
	}


	switch (fs) {
	case 32000:
		clk_ctl |= SGTL5000_SYS_FS_32k << SGTL5000_SYS_FS_SHIFT;
		break;
	case 44100:
		clk_ctl |= SGTL5000_SYS_FS_44_1k << SGTL5000_SYS_FS_SHIFT;
		break;
	case 48000:
		clk_ctl |= SGTL5000_SYS_FS_48k << SGTL5000_SYS_FS_SHIFT;
		break;
	case 96000:
		clk_ctl |= SGTL5000_SYS_FS_96k << SGTL5000_SYS_FS_SHIFT;
		break;
	default:
		pr_err("%s: sample rate %d not supported\n", __func__, fs);
		return -EFAULT;
	}

#if 0 /*SGTL5000 rev1 has a IC bug to prevent switching to MCLK from PLL. */
	if (fs*256 == sgtl5000->sysclk)
		clk_ctl |= SGTL5000_MCLK_FREQ_256FS << SGTL5000_MCLK_FREQ_SHIFT;
	else if (fs*384 == sgtl5000->sysclk && fs != 96000)
		clk_ctl |= SGTL5000_MCLK_FREQ_384FS << SGTL5000_MCLK_FREQ_SHIFT;
	else if (fs*512 == sgtl5000->sysclk && fs != 96000)
		clk_ctl |= SGTL5000_MCLK_FREQ_512FS << SGTL5000_MCLK_FREQ_SHIFT;
	else
#endif
	{
		if (!sgtl5000->master) {
			pr_err("%s: PLL not supported in slave mode\n",
			       __func__);
			return -EINVAL;
		}
		clk_ctl |= SGTL5000_MCLK_FREQ_PLL << SGTL5000_MCLK_FREQ_SHIFT;
	}

	if ((clk_ctl & SGTL5000_MCLK_FREQ_MASK) == SGTL5000_MCLK_FREQ_PLL) {
		u64 out, t;
		unsigned int in, int_div, frac_div;
		if (sgtl5000->sysclk > 17000000) {
			div2 = 1;
			in = sgtl5000->sysclk / 2;
		} else {
			div2 = 0;
			in = sgtl5000->sysclk;
		}
		if (fs == 44100)
			out = 180633600;
		else
			out = 196608000;
		t = do_div(out, in);
		int_div = out;
		t *= 2048;
		do_div(t, in);
		frac_div = t;
		pll_ctl = int_div << SGTL5000_PLL_INT_DIV_SHIFT |
			  frac_div << SGTL5000_PLL_FRAC_DIV_SHIFT;
	}

	i2s_ctl = sgtl5000_read(codec, SGTL5000_CHIP_I2S_CTRL);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (sgtl5000->fmt == SND_SOC_DAIFMT_RIGHT_J)
			return -EINVAL;
		i2s_ctl |= SGTL5000_I2S_DLEN_16 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_32FS <<
			    SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		i2s_ctl |= SGTL5000_I2S_DLEN_20 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_64FS <<
			   SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		i2s_ctl |= SGTL5000_I2S_DLEN_24 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_64FS <<
			   SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		if (sgtl5000->fmt == SND_SOC_DAIFMT_RIGHT_J)
			return -EINVAL;
		i2s_ctl |= SGTL5000_I2S_DLEN_32 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_64FS <<
			   SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	pr_debug("fs=%d,clk_ctl=%d,pll_ctl=%d,i2s_ctl=%d,div2=%d\n",
		 fs, clk_ctl, pll_ctl, i2s_ctl, div2);

	if ((clk_ctl & SGTL5000_MCLK_FREQ_MASK) == SGTL5000_MCLK_FREQ_PLL) {
		sgtl5000_write(codec, SGTL5000_CHIP_PLL_CTRL, pll_ctl);
		reg = sgtl5000_read(codec, SGTL5000_CHIP_CLK_TOP_CTRL);
		if (div2)
			reg |= SGTL5000_INPUT_FREQ_DIV2;
		else
			reg &= ~SGTL5000_INPUT_FREQ_DIV2;
		sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, reg);
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg |= SGTL5000_PLL_POWERUP | SGTL5000_VCOAMP_POWERUP; /*vco?*/
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
	}
	sgtl5000_write(codec, SGTL5000_CHIP_CLK_CTRL, clk_ctl);
	sgtl5000_write(codec, SGTL5000_CHIP_I2S_CTRL, i2s_ctl);
	reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
	reg |= SGTL5000_I2S_IN_POWERUP | SGTL5000_I2S_OUT_POWERUP;
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);
	return 0;
}

#define SGTL5000_RATES (SNDRV_PCM_RATE_32000 |\
		      SNDRV_PCM_RATE_44100 |\
		      SNDRV_PCM_RATE_48000 |\
		      SNDRV_PCM_RATE_96000)

#define SGTL5000_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_pcm_stream sgtl5000_dai_playback = {
	.stream_name = "Playback",
	.channels_min = 1,
	.channels_max = 2,
	.rates = SGTL5000_RATES,
	.formats = SGTL5000_FORMATS,
};

static const struct snd_soc_pcm_stream sgtl5000_dai_capture = {
	.stream_name = "Capture",
	.channels_min = 1,
	.channels_max = 1,
	.rates = SGTL5000_RATES,
	.formats = SGTL5000_FORMATS,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops sgtl5000_dai_audio_ops = {
	.startup = sgtl5000_pcm_startup,
	.shutdown = sgtl5000_pcm_shutdown,
	.hw_params = sgtl5000_pcm_hw_params,
};

static int sgtl5000_dapm_event(struct snd_soc_codec *codec, int event)
{
	u16 reg;
	pr_debug("dapm event %d\n", event);
	switch (event) {
	case SNDRV_CTL_POWER_D0:	/* full On */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg |= SGTL5000_HP_POWERUP;
		reg |= SGTL5000_LINE_OUT_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);


		reg = sgtl5000_read(codec, SGTL5000_CHIP_MIC_CTRL);
		reg &= ~SGTL5000_BIAS_R_MASK;
		reg |= SGTL5000_BIAS_R_4k << SGTL5000_BIAS_R_SHIFT;
		sgtl5000_write(codec, SGTL5000_CHIP_MIC_CTRL, reg);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_CTRL);
		reg |= SGTL5000_HP_ZCD_EN;
		reg |= SGTL5000_ADC_ZCD_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg);
		break;

	case SNDRV_CTL_POWER_D1:	/* partial On */
	case SNDRV_CTL_POWER_D2:	/* partial On */
		break;

	case SNDRV_CTL_POWER_D3hot:	/* Off, with power */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_MIC_CTRL);
		reg &= ~SGTL5000_BIAS_R_MASK;
		reg |= SGTL5000_BIAS_R_off;
		sgtl5000_write(codec, SGTL5000_CHIP_MIC_CTRL, reg);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_CTRL);
		reg &= ~SGTL5000_HP_ZCD_EN;
		reg &= ~SGTL5000_ADC_ZCD_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		if (codec->dapm_state == SNDRV_CTL_POWER_D3cold) {
			reg |= SGTL5000_VAG_POWERUP;
			reg |= SGTL5000_REFTOP_POWERUP;
			reg |= SGTL5000_DAC_POWERUP;
			reg |= SGTL5000_ADC_POWERUP;
		}
		/*reg &= ~SGTL5000_PLL_POWERUP;
		reg &= ~SGTL5000_VCOAMP_POWERUP;*/
		reg &= ~SGTL5000_HP_POWERUP;
		reg &= ~SGTL5000_CAPLESS_HP_POWERUP;
		reg &= ~SGTL5000_LINE_OUT_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);

		msleep(400);

		break;

	case SNDRV_CTL_POWER_D3cold:	/* Off, without power */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg &= ~SGTL5000_VAG_POWERUP;
		reg &= ~SGTL5000_REFTOP_POWERUP;
		reg &= ~SGTL5000_DAC_POWERUP;
		reg &= ~SGTL5000_ADC_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
		break;
	}

	codec->dapm_state = event;

	return 0;
}

static int sgtl5000_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	sgtl5000_dapm_event(codec, SNDRV_CTL_POWER_D3cold);

	return 0;
}

static int sgtl5000_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	/* Bring the codec back up to standby first to minimise pop/clicks */
	sgtl5000_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	sgtl5000_dapm_event(codec, codec->suspend_dapm_state);

	return 0;
}

static int sgtl5000_codec_io_probe(struct snd_soc_codec *codec,
				 struct snd_soc_machine *machine)
{
	u16 reg, ana_pwr, lreg_ctrl, ref_ctrl, lo_ctrl, short_ctrl, sss;
	int vag;
	unsigned int val;
	struct sgtl5000_platform_data *plat = codec->platform_data;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;

	val = sgtl5000_read(NULL, SGTL5000_CHIP_ID);
	if (((val & SGTL5000_PARTID_MASK) >> SGTL5000_PARTID_SHIFT) !=
		SGTL5000_PARTID_PART_ID) {
		sgtl5000_i2c_client = NULL;
		pr_err("Device with ID register %x is not a SGTL5000\n", val);
		return -ENODEV;
	}

	sgtl5000->rev = (val & SGTL5000_REVID_MASK) >> SGTL5000_REVID_SHIFT;
	dev_info(&sgtl5000_i2c_client->dev, "SGTL5000 revision %d\n",
		 sgtl5000->rev);
	if (sgtl5000->rev == 0x00) /* if chip is rev 1 */
		sgtl5000_dai_playback.channels_min = 2;

	/* reset value */
	ana_pwr = SGTL5000_DAC_STERO |
		  SGTL5000_LINREG_SIMPLE_POWERUP |
		  SGTL5000_STARTUP_POWERUP |
		  SGTL5000_ADC_STERO |
		  SGTL5000_REFTOP_POWERUP;
	lreg_ctrl = 0;
	ref_ctrl = 0;
	lo_ctrl = 0;
	short_ctrl = 0;
	sss = SGTL5000_DAC_SEL_I2S_IN << SGTL5000_DAC_SEL_SHIFT;

	if (!plat->vddd) {
		/* set VDDD to 1.2v */
		lreg_ctrl |= 0x8 << SGTL5000_LINREG_VDDD_SHIFT;
		/* power internal linear regulator */
		ana_pwr |= SGTL5000_LINEREG_D_POWERUP;
	} else {
		/* turn of startup power */
		ana_pwr &= ~SGTL5000_STARTUP_POWERUP;
		ana_pwr &= ~SGTL5000_LINREG_SIMPLE_POWERUP;
	}
	if (plat->vddio < 3100 && plat->vdda < 3100) {
		/* Enable VDDC charge pump */
		ana_pwr |= SGTL5000_VDDC_CHRGPMP_POWERUP;
	}
	if (plat->vddio >= 3100 && plat->vdda >= 3100) {
		/* VDDC use VDDIO rail */
		lreg_ctrl |= SGTL5000_VDDC_ASSN_OVRD;
		if (plat->vddio >= 3100)
			lreg_ctrl |= SGTL5000_VDDC_MAN_ASSN_VDDIO <<
				     SGTL5000_VDDC_MAN_ASSN_SHIFT;
	}
	/* If PLL is powered up (such as on power cycle) leave it on. */
	reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
	ana_pwr |=  reg & (SGTL5000_PLL_POWERUP | SGTL5000_VCOAMP_POWERUP);

	/* set ADC/DAC ref voltage to vdda/2 */
	vag = plat->vdda/2;
	if (vag <= SGTL5000_ANA_GND_BASE)
		vag = 0;
	else if (vag >= SGTL5000_ANA_GND_BASE + SGTL5000_ANA_GND_STP *
		 (SGTL5000_ANA_GND_MASK >> SGTL5000_ANA_GND_SHIFT))
		vag = SGTL5000_ANA_GND_MASK >> SGTL5000_ANA_GND_SHIFT;
	else
		vag = (vag - SGTL5000_ANA_GND_BASE) / SGTL5000_ANA_GND_STP;
	ref_ctrl |= vag << SGTL5000_ANA_GND_SHIFT;

	/* set line out ref voltage to vddio/2 */
	vag = plat->vddio/2;
	if (vag <= SGTL5000_LINE_OUT_GND_BASE)
		vag = 0;
	else if (vag >= SGTL5000_LINE_OUT_GND_BASE + SGTL5000_LINE_OUT_GND_STP *
		 SGTL5000_LINE_OUT_GND_MAX)
		vag = SGTL5000_LINE_OUT_GND_MAX;
	else
		vag = (vag - SGTL5000_LINE_OUT_GND_BASE) /
		      SGTL5000_LINE_OUT_GND_STP;
	lo_ctrl |= vag << SGTL5000_LINE_OUT_GND_SHIFT;

	/* enable small pop */
	ref_ctrl |= SGTL5000_SMALL_POP;

	/* set short detect */
	/* keep default */

	/* set routing */
	/* keep default, bypass DAP */

	sgtl5000_write(codec, SGTL5000_CHIP_LINREG_CTRL, lreg_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, ana_pwr);
	msleep(10);
	sgtl5000_write(codec, SGTL5000_CHIP_REF_CTRL, ref_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_LINE_OUT_CTRL, lo_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_SHORT_CTRL, short_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_SSS_CTRL, sss);
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, 0);

	reg = SGTL5000_DAC_VOL_RAMP_EN |
	      SGTL5000_DAC_MUTE_RIGHT |
	      SGTL5000_DAC_MUTE_LEFT;
	sgtl5000_write(codec, SGTL5000_CHIP_ADCDAC_CTRL, reg);

	sgtl5000_write(codec, SGTL5000_CHIP_PAD_STRENGTH, 0x015f);

	reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_ADC_CTRL);
	reg &= ~SGTL5000_ADC_VOL_M6DB;
	reg &= ~(SGTL5000_ADC_VOL_LEFT_MASK | SGTL5000_ADC_VOL_RIGHT_MASK);
	reg |= (0xf << SGTL5000_ADC_VOL_LEFT_SHIFT)
	       | (0xf << SGTL5000_ADC_VOL_RIGHT_SHIFT);
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_ADC_CTRL, reg);

	reg = SGTL5000_LINE_OUT_MUTE |
	      SGTL5000_HP_MUTE |
	      SGTL5000_ADC_MUTE;
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg);

	sgtl5000_write(codec, SGTL5000_CHIP_MIC_CTRL, 0);
	sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, 0);
	/* disable DAP */
	sgtl5000_write(codec, SGTL5000_DAP_CTRL, 0);
	/* TODO: initialize DAP */

	sgtl5000_add_controls(codec, machine->card);
	sgtl5000_add_widgets(codec, machine);

	codec->dapm_state = SNDRV_CTL_POWER_D3cold;
	sgtl5000_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	return 0;
}

static int sgtl5000_codec_io_remove(struct snd_soc_codec *codec,
				  struct snd_soc_machine *machine)
{
	if (codec->dapm_state != SNDRV_CTL_POWER_D3hot)
		sgtl5000_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	sgtl5000_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static const struct snd_soc_codec_ops sgtl5000_codec_ops = {
	.dapm_event = sgtl5000_dapm_event,
	.read = sgtl5000_read,
	.write = sgtl5000_write,
	.io_probe = sgtl5000_codec_io_probe,
	.io_remove = sgtl5000_codec_io_remove,
};

static int sgtl5000_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	struct sgtl5000_priv *sgtl5000;

	codec->owner = THIS_MODULE;
	codec->ops = &sgtl5000_codec_ops;

	sgtl5000 = kzalloc(sizeof(struct sgtl5000_priv), GFP_KERNEL);
	if (sgtl5000 == NULL)
		return -ENOMEM;
	codec->private_data = sgtl5000;

	snd_soc_register_codec(codec);

	return 0;
}

static int sgtl5000_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	kfree(codec->private_data);
	return 0;
}

static int sgtl5000_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);

	dai->ops = &sgtl5000_dai_ops;
	dai->audio_ops = &sgtl5000_dai_audio_ops;
	dai->capture = &sgtl5000_dai_capture;
	dai->playback = &sgtl5000_dai_playback;
	snd_soc_register_codec_dai(dai);

	return 0;
}

const char sgtl5000_codec[SND_SOC_CODEC_NAME_SIZE] = "sgtl5000-codec";
EXPORT_SYMBOL_GPL(sgtl5000_codec);

static struct snd_soc_device_driver sgtl5000_codec_driver = {
	.type = SND_SOC_BUS_TYPE_CODEC,
	.driver = {
		   .name = sgtl5000_codec,
		   .owner = THIS_MODULE,
		   .bus = &asoc_bus_type,
		   .probe = sgtl5000_codec_probe,
		   .remove = __devexit_p(sgtl5000_codec_remove),
		   .suspend = sgtl5000_suspend,
		   .resume = sgtl5000_resume,
		   },
};

const char sgtl5000_dai[SND_SOC_CODEC_NAME_SIZE] = "sgtl5000-dai";
EXPORT_SYMBOL_GPL(sgtl5000_dai);

static struct snd_soc_device_driver sgtl5000_dai_driver = {
	.type = SND_SOC_BUS_TYPE_DAI,
	.driver = {
		   .name = sgtl5000_dai,
		   .owner = THIS_MODULE,
		   .bus = &asoc_bus_type,
		   .probe = sgtl5000_dai_probe,
		   },
};

static int sgtl5000_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;

	sgtl5000_i2c_client = client;

	ret = driver_register(&sgtl5000_codec_driver.driver);
	if (ret < 0)
		return ret;

	ret = driver_register(&sgtl5000_dai_driver.driver);
	if (ret < 0) {
		driver_unregister(&sgtl5000_codec_driver.driver);
		return ret;
	}

	return ret;
}

static int sgtl5000_i2c_remove(struct i2c_client *client)
{
	driver_unregister(&sgtl5000_dai_driver.driver);
	driver_unregister(&sgtl5000_codec_driver.driver);

	return 0;
}

static const struct i2c_device_id sgtl5000_id[] = {
	{ "sgtl5000-i2c", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgtl5000_id);

static struct i2c_driver sgtl5000_i2c_driver = {
	.driver = {
		   .name = "sgtl5000-i2c",
		   .owner = THIS_MODULE,
		   },
	.probe = sgtl5000_i2c_probe,
	.remove = sgtl5000_i2c_remove,
	.id_table = sgtl5000_id,
};

static __init int sgtl5000_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&sgtl5000_i2c_driver);
	if (ret) {
		pr_err("sgtl5000 i2c driver register failed");
		return ret;
	}

	return ret;
}

static __exit void sgtl5000_exit(void)
{
	i2c_del_driver(&sgtl5000_i2c_driver);
}


subsys_initcall(sgtl5000_init);
module_exit(sgtl5000_exit);

MODULE_DESCRIPTION("ASoC SGTL5000 driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");

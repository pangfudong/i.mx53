/*
 * wm8350.c -- WM8350 ALSA SoC audio driver
 *
 * Copyright (C) 2007 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood <lg@opensource.wolfsonmicro.com>
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
#include <linux/platform_device.h>
#include <linux/regulator/wm8350/wm8350.h>
#include <linux/regulator/wm8350/wm8350-bus.h>
#include <linux/regulator/wm8350/wm8350-audio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#define AUDIO_NAME "WM8350"
#define WM8350_VERSION "0.4"

#define WM8350_OUTn_0dB 0x39

/*
 * Debug
 */

#define WM8350_DEBUG 0

#ifdef WM8350_DEBUG
#define dbg(format, arg...) \
	printk(KERN_DEBUG AUDIO_NAME ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)

#define WM8350_RAMP_NONE	0
#define WM8350_RAMP_UP	1
#define WM8350_RAMP_DOWN	2

struct wm8350_output {
	u16 left_vol;
	u16 right_vol;
	u16 ramp;
	u16 mute;
};

struct wm8350_out_ramp {
	struct wm8350_output out1;
	struct wm8350_output out2;
};

static int wm8350_dapm_event(struct snd_soc_codec *codec, int event);

static unsigned int wm8350_codec_cache_read(struct snd_soc_codec *codec,
					    unsigned int reg)
{
	struct wm8350 *wm8350 = codec->control_data;
	return wm8350->reg_cache[reg];
}

static unsigned int wm8350_codec_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	struct wm8350* wm8350 = codec->control_data;
	return wm8350_reg_read(wm8350, reg);
}

static int wm8350_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	struct wm8350* wm8350 = codec->control_data;
	return wm8350_reg_write(wm8350, reg, value);
}

/*
 * Ramp OUT1 PGA volume to minimise pops at stream startup and shutdown.
 */
static inline int wm8350_out1_ramp_step(struct snd_soc_codec *codec)
{
	struct wm8350_out_ramp *or = codec->private_data;
	struct wm8350_output *output = &or->out1;
	struct wm8350 *wm8350 = codec->control_data;
	int left_complete = 0, right_complete = 0;
	u16 reg, val;

	/* left channel */
	reg = wm8350_reg_read(wm8350, WM8350_LOUT1_VOLUME);
	val = (reg & WM8350_OUT1L_VOL_MASK) >> WM8350_OUT1L_VOL_SHIFT;

	if (output->ramp == WM8350_RAMP_UP) {
		/* ramp step up */
		if (val < output->left_vol) {
			val++;
			reg &= ~WM8350_OUT1L_VOL_MASK;
			wm8350_reg_write(wm8350, WM8350_LOUT1_VOLUME,
				reg | (val << WM8350_OUT1L_VOL_SHIFT));
		} else
			left_complete = 1;
	} else if (output->ramp == WM8350_RAMP_DOWN) {
		/* ramp step down */
		if (val > 0) {
			val--;
			reg &= ~WM8350_OUT1L_VOL_MASK;
			wm8350_reg_write(wm8350, WM8350_LOUT1_VOLUME,
				reg | (val << WM8350_OUT1L_VOL_SHIFT));
		} else
			left_complete = 1;
	} else
		return 1;

	/* right channel */
	reg = wm8350_reg_read(wm8350, WM8350_ROUT1_VOLUME);
	val = (reg & WM8350_OUT1R_VOL_MASK) >> WM8350_OUT1R_VOL_SHIFT;
	if (output->ramp == WM8350_RAMP_UP) {
		/* ramp step up */
		if (val < output->right_vol) {
			val++;
			reg &= ~WM8350_OUT1R_VOL_MASK;
			wm8350_reg_write(wm8350, WM8350_ROUT1_VOLUME,
				reg | (val << WM8350_OUT1R_VOL_SHIFT));
		} else
			right_complete = 1;
	} else if (output->ramp == WM8350_RAMP_DOWN) {
		/* ramp step down */
		if (val > 0) {
			val--;
			reg &= ~WM8350_OUT1R_VOL_MASK;
			wm8350_reg_write(wm8350, WM8350_ROUT1_VOLUME,
				reg | (val << WM8350_OUT1R_VOL_SHIFT));
		} else
			right_complete = 1;
	}

	/* only hit the update bit if either volume has changed this step */
	if (!left_complete || !right_complete)
		wm8350_set_bits(wm8350, WM8350_LOUT1_VOLUME, WM8350_OUT1_VU);

	return left_complete & right_complete;
}

/*
 * Ramp OUT2 PGA volume to minimise pops at stream startup and shutdown.
 */
static inline int wm8350_out2_ramp_step(struct snd_soc_codec *codec)
{
	struct wm8350_out_ramp *or = codec->private_data;
	struct wm8350_output *output = &or->out2;
	struct wm8350 *wm8350 = codec->control_data;
	int left_complete = 0, right_complete = 0;
	u16 reg, val;

	/* left channel */
	reg = wm8350_reg_read(wm8350, WM8350_LOUT2_VOLUME);
	val = (reg & WM8350_OUT2L_VOL_MASK) >> WM8350_OUT1L_VOL_SHIFT;
	if (output->ramp == WM8350_RAMP_UP) {
		/* ramp step up */
		if (val < output->left_vol) {
			val++;
			reg &= ~WM8350_OUT2L_VOL_MASK;
			wm8350_reg_write(wm8350, WM8350_LOUT2_VOLUME,
				reg | (val << WM8350_OUT1L_VOL_SHIFT));
		} else
			left_complete = 1;
	} else if (output->ramp == WM8350_RAMP_DOWN) {
		/* ramp step down */
		if (val > 0) {
			val--;
			reg &= ~WM8350_OUT2L_VOL_MASK;
			wm8350_reg_write(wm8350, WM8350_LOUT2_VOLUME,
				reg | (val << WM8350_OUT1L_VOL_SHIFT));
		} else
			left_complete = 1;
	} else
		return 1;

	/* right channel */
	reg = wm8350_reg_read(wm8350, WM8350_ROUT2_VOLUME);
	val = (reg & WM8350_OUT2R_VOL_MASK) >> WM8350_OUT1R_VOL_SHIFT;
	if (output->ramp == WM8350_RAMP_UP) {
		/* ramp step up */
		if (val < output->right_vol) {
			val++;
			reg &= ~WM8350_OUT2R_VOL_MASK;
			wm8350_reg_write(wm8350, WM8350_ROUT2_VOLUME,
				reg | (val << WM8350_OUT1R_VOL_SHIFT));
		} else
			right_complete = 1;
	} else if (output->ramp == WM8350_RAMP_DOWN) {
		/* ramp step down */
		if (val > 0) {
			val--;
			reg &= ~WM8350_OUT2R_VOL_MASK;
			wm8350_reg_write(wm8350, WM8350_ROUT2_VOLUME,
				reg | (val << WM8350_OUT1R_VOL_SHIFT));
		} else
			right_complete = 1;
	}

	/* only hit the update bit if either volume has changed this step */
	if (!left_complete || !right_complete)
		wm8350_set_bits(wm8350, WM8350_LOUT2_VOLUME, WM8350_OUT2_VU);

	return left_complete & right_complete;
}

/*
 * This work ramps both output PGA's at stream start/stop time to
 * minimise pop associated with DAPM power switching.
 * It's best to enable Zero Cross when ramping occurs to minimise any
 * zipper noises.
 */
static void wm8350_pga_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
		container_of(work, struct snd_soc_codec, delayed_work.work);
	struct wm8350_out_ramp *or = codec->private_data;
	struct wm8350_output *out1 = &or->out1, *out2 = &or->out2;
	int i, out1_complete, out2_complete;

	/* do we need to ramp at all ? */
	if (out1->ramp == WM8350_RAMP_NONE && out2->ramp == WM8350_RAMP_NONE)
		return;

	/* PGA volumes have 6 bits of resolution to ramp */
	for (i = 0; i <= 63; i++) {

		out1_complete = 1, out2_complete = 1;
		if (out1->ramp != WM8350_RAMP_NONE)
			out1_complete = wm8350_out1_ramp_step(codec);
		if (out2->ramp != WM8350_RAMP_NONE)
			out2_complete = wm8350_out2_ramp_step(codec);

		/* ramp finished ? */
		if (out1_complete && out2_complete)
			break;

		/* we need to delay longer on the up ramp */
		if (out1->ramp == WM8350_RAMP_UP ||
			out2->ramp == WM8350_RAMP_UP) {
			/* delay is longer over 0dB as increases are larger */
			if (i >= WM8350_OUTn_0dB)
				schedule_timeout_interruptible(
					msecs_to_jiffies(2));
			else
				schedule_timeout_interruptible(
					msecs_to_jiffies(1));
		} else
			udelay(50); /* doesn't matter if we delay longer */
	}

	out1->ramp = WM8350_RAMP_NONE;
	out2->ramp = WM8350_RAMP_NONE;
}

/*
 * WM8350 Controls
 */

static int pga_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct wm8350_out_ramp *or = codec->private_data;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* ramp vol up */
		switch (w->shift) {
		case 0:
		case 1:
			or->out1.ramp = WM8350_RAMP_UP;
			break;
		case 2:
		case 3:
			or->out2.ramp = WM8350_RAMP_UP;
			break;
		}
		if (!delayed_work_pending(&codec->delayed_work))
			schedule_delayed_work(&codec->delayed_work,
				msecs_to_jiffies(1));
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* ramp down */
		switch (w->shift) {
		case 0:
		case 1:
			or->out1.ramp = WM8350_RAMP_DOWN;
			break;
		case 2:
		case 3:
			or->out2.ramp = WM8350_RAMP_DOWN;
			break;
		}
		if (!delayed_work_pending(&codec->delayed_work))
			schedule_delayed_work(&codec->delayed_work,
				msecs_to_jiffies(1));
		break;
	}
	return 0;
}

static int wm8350_put_volsw_2r_vu(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct wm8350_out_ramp *or = codec->private_data;
	struct wm8350_output *out1 = &or->out1, *out2 = &or->out2;
	int ret, reg = kcontrol->private_value & 0xff;
	u16 val;

	/* we cache the OUT1/2 volumes when the codec is inactive to keep
	 * pops to a low during ramping */
	if (reg == WM8350_LOUT1_VOLUME) {
		out1->left_vol = ucontrol->value.integer.value[0];
		out1->right_vol = ucontrol->value.integer.value[1];
		if (!codec->active)
			return 1;
	} else if (reg == WM8350_LOUT2_VOLUME) {
		out2->left_vol = ucontrol->value.integer.value[0];
		out2->right_vol = ucontrol->value.integer.value[1];
		if (!codec->active)
			return 1;
	}

	ret = snd_soc_put_volsw_2r(kcontrol, ucontrol);
	if (ret < 0)
		return ret;

	/* now hit the volume update bits (always bit 8) */
	val = wm8350_codec_read(codec, reg);
	wm8350_codec_write(codec, reg, val | WM8350_OUT1_VU);
	return 1;
}

static int wm8350_get_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct wm8350_out_ramp *or = codec->private_data;
	struct wm8350_output *out1 = &or->out1, *out2 = &or->out2;
	int reg = kcontrol->private_value & 0xff;
	int reg2 = (kcontrol->private_value >> 24) & 0xff;
	int shift = (kcontrol->private_value >> 8) & 0x0f;
	int max = (kcontrol->private_value >> 12) & 0xff;
	int mask = (1 << fls(max)) - 1;

	/* if the codec is inactive then read back the cached values */
	if (!codec->active) {
		if (reg == WM8350_LOUT1_VOLUME) {
			ucontrol->value.integer.value[0] = out1->left_vol;
			ucontrol->value.integer.value[1] = out1->right_vol;
			return 0;
		} else if (reg == WM8350_LOUT2_VOLUME) {
			ucontrol->value.integer.value[0] = out2->left_vol;
			ucontrol->value.integer.value[1] = out2->right_vol;
			return 0;
		}
	}

	ucontrol->value.integer.value[0] =
		(snd_soc_read(codec, reg) >> shift) & mask;
	ucontrol->value.integer.value[1] =
		(snd_soc_read(codec, reg2) >> shift) & mask;

	return 0;
}

/* double control with volume update */
#define SOC_WM8350_DOUBLE_R_TLV(xname, reg_left, reg_right, shift, mask, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE |\
	 SNDRV_CTL_ELEM_ACCESS_VOLATILE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_2r, \
	.get = wm8350_get_volsw_2r, .put = wm8350_put_volsw_2r_vu, \
	.private_value = (reg_left) | ((shift) << 8)  | \
		((mask) << 12) | ((invert) << 20) | ((reg_right) << 24) }

static const char *wm8350_deemp[] = {"None", "32kHz", "44.1kHz", "48kHz"};
static const char *wm8350_pol[] = {"Normal", "Inv R", "Inv L", "Inv L & R"};
static const char *wm8350_dacmutem[] = {"Normal", "Soft"};
static const char *wm8350_dacmutes[] = {"Fast", "Slow"};
static const char *wm8350_dacfilter[] = {"Normal", "Sloping"};
static const char *wm8350_adcfilter[] = {"None", "High Pass"};
static const char *wm8350_adchp[] = {"44.1kHz", "8kHz", "16kHz", "32kHz"};
static const char *wm8350_lr[] = {"Left", "Right"};

static const struct soc_enum wm8350_enum[] = {
SOC_ENUM_SINGLE(WM8350_DAC_CONTROL, 4, 4, wm8350_deemp),
SOC_ENUM_SINGLE(WM8350_DAC_CONTROL, 0, 4, wm8350_pol),
SOC_ENUM_SINGLE(WM8350_DAC_MUTE_VOLUME, 14, 2, wm8350_dacmutem),
SOC_ENUM_SINGLE(WM8350_DAC_MUTE_VOLUME, 13, 2, wm8350_dacmutes),
SOC_ENUM_SINGLE(WM8350_DAC_MUTE_VOLUME, 12, 2, wm8350_dacfilter),
SOC_ENUM_SINGLE(WM8350_ADC_CONTROL, 15, 2, wm8350_adcfilter),
SOC_ENUM_SINGLE(WM8350_ADC_CONTROL, 8, 4, wm8350_adchp),
SOC_ENUM_SINGLE(WM8350_ADC_CONTROL, 0, 4, wm8350_pol),
SOC_ENUM_SINGLE(WM8350_INPUT_MIXER_VOLUME, 15, 2, wm8350_lr),
};

DECLARE_TLV_DB_LINEAR(pre_amp_tlv, -1200, 3525);
DECLARE_TLV_DB_LINEAR(out_pga_tlv, -5700, 600);
DECLARE_TLV_DB_SCALE(dac_pcm_tlv, -7163, 36, 1);
DECLARE_TLV_DB_SCALE(adc_pcm_tlv, -12700, 50, 1);
DECLARE_TLV_DB_SCALE(out_mix_tlv, -1200, 300, 1);

static const unsigned int capture_sd_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0,12, TLV_DB_SCALE_ITEM(-3600, 300, 1),
	13,15, TLV_DB_SCALE_ITEM(0, 0, 0),
};

static const struct snd_kcontrol_new wm8350_snd_controls[] = {
SOC_ENUM("Playback Deemphasis", wm8350_enum[0]),
SOC_ENUM("Playback DAC Inversion", wm8350_enum[1]),
SOC_WM8350_DOUBLE_R_TLV("Playback PCM Volume", WM8350_DAC_DIGITAL_VOLUME_L,
	WM8350_DAC_DIGITAL_VOLUME_R, 0, 255, 0, dac_pcm_tlv),
SOC_ENUM("Playback PCM Mute Function", wm8350_enum[2]),
SOC_ENUM("Playback PCM Mute Speed", wm8350_enum[3]),
SOC_ENUM("Playback PCM Filter", wm8350_enum[4]),
SOC_ENUM("Capture PCM Filter", wm8350_enum[5]),
SOC_ENUM("Capture PCM HP Filter", wm8350_enum[6]),
SOC_ENUM("Capture ADC Inversion", wm8350_enum[7]),
SOC_WM8350_DOUBLE_R_TLV("Capture PCM Volume", WM8350_ADC_DIGITAL_VOLUME_L,
	WM8350_ADC_DIGITAL_VOLUME_R, 0, 255, 0, adc_pcm_tlv),
SOC_DOUBLE_TLV("Capture Sidetone Volume", WM8350_ADC_DIVIDER, 8, 4, 15, 1, capture_sd_tlv),
SOC_WM8350_DOUBLE_R_TLV("Capture Volume", WM8350_LEFT_INPUT_VOLUME,
	WM8350_RIGHT_INPUT_VOLUME, 2, 63, 0, pre_amp_tlv),
SOC_DOUBLE_R("Capture ZC Switch", WM8350_LEFT_INPUT_VOLUME,
	WM8350_RIGHT_INPUT_VOLUME, 13, 1, 0),
SOC_SINGLE_TLV("Left Input Left Sidetone Volume", WM8350_OUTPUT_LEFT_MIXER_VOLUME,
	1, 7, 0, out_mix_tlv),
SOC_SINGLE_TLV("Left Input Right Sidetone Volume", WM8350_OUTPUT_LEFT_MIXER_VOLUME,
	5, 7, 0, out_mix_tlv),
SOC_SINGLE_TLV("Left Input Bypass Volume", WM8350_OUTPUT_LEFT_MIXER_VOLUME,
	9, 7, 0, out_mix_tlv),
SOC_SINGLE_TLV("Right Input Left Sidetone Volume", WM8350_OUTPUT_RIGHT_MIXER_VOLUME,
	1, 7, 0, out_mix_tlv),
SOC_SINGLE_TLV("Right Input Right Sidetone Volume", WM8350_OUTPUT_RIGHT_MIXER_VOLUME,
	5, 7, 0, out_mix_tlv),
SOC_SINGLE_TLV("Right Input Bypass Volume", WM8350_OUTPUT_RIGHT_MIXER_VOLUME,
	13, 7, 0, out_mix_tlv),
SOC_SINGLE("Left Input Mixer +20dB Switch", WM8350_INPUT_MIXER_VOLUME_L,
	0, 1, 0),
SOC_SINGLE("Right Input Mixer +20dB Switch", WM8350_INPUT_MIXER_VOLUME_R,
	0, 1, 0),
SOC_SINGLE_TLV("Out4 Capture Volume", WM8350_INPUT_MIXER_VOLUME, 1, 7, 0, out_mix_tlv),
SOC_WM8350_DOUBLE_R_TLV("Out1 Playback Volume", WM8350_LOUT1_VOLUME,
	WM8350_ROUT1_VOLUME, 2, 63, 0, out_pga_tlv),
SOC_DOUBLE_R("Out1 Playback ZC Switch", WM8350_LOUT1_VOLUME,
	WM8350_ROUT1_VOLUME, 13, 1, 0),
SOC_WM8350_DOUBLE_R_TLV("Out2 Playback Volume", WM8350_LOUT2_VOLUME,
	WM8350_ROUT2_VOLUME, 2, 63, 0, out_pga_tlv),
SOC_DOUBLE_R("Out2 Playback ZC Switch", WM8350_LOUT2_VOLUME,
	WM8350_ROUT2_VOLUME, 13, 1, 0),
SOC_SINGLE("Out2 Right Invert Switch", WM8350_ROUT2_VOLUME, 10, 1, 0),
SOC_SINGLE_TLV("Out2 Beep Volume", WM8350_BEEP_VOLUME, 5, 7, 0, out_mix_tlv),

SOC_DOUBLE_R("Out1 Playback Switch", WM8350_LOUT1_VOLUME, WM8350_ROUT1_VOLUME,
	14, 1, 1),
SOC_DOUBLE_R("Out2 Playback Switch", WM8350_LOUT2_VOLUME, WM8350_ROUT2_VOLUME,
	14, 1, 1),
};

/* add non dapm controls */
static int wm8350_add_controls(struct snd_soc_codec *codec,
	struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8350_snd_controls); i++) {
		err = snd_ctl_add(card,
				snd_soc_cnew(&wm8350_snd_controls[i],
					codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

/* Same as snd_soc_dapm_put_volsw, but change power state */
static int wm8350_dapm_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = widget->codec;
	if (ucontrol->value.integer.value[0]) {
		if (codec->dapm_state != SNDRV_CTL_POWER_D0) {
			wm8350_dapm_event(codec, SNDRV_CTL_POWER_D1);
			snd_soc_dapm_put_volsw(kcontrol, ucontrol);
			wm8350_dapm_event(codec, SNDRV_CTL_POWER_D0);
		} else
			snd_soc_dapm_put_volsw(kcontrol, ucontrol);
	} else {
		snd_soc_dapm_put_volsw(kcontrol, ucontrol);
		wm8350_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	}
}

/*
 * DAPM Controls
 */

/* Left Playback Mixer */
static const struct snd_kcontrol_new wm8350_left_play_mixer_controls[] = {
SOC_DAPM_SINGLE("Playback Switch", WM8350_LEFT_MIXER_CONTROL, 11, 1, 0),
SOC_SINGLE_EXT("Left Bypass Switch", WM8350_LEFT_MIXER_CONTROL, 2, 1, 0,
		snd_soc_dapm_get_volsw, wm8350_dapm_put_volsw),
SOC_DAPM_SINGLE("Right Playback Switch", WM8350_LEFT_MIXER_CONTROL, 12, 1, 0),
SOC_DAPM_SINGLE("Left Sidetone Switch", WM8350_LEFT_MIXER_CONTROL, 0, 1, 0),
SOC_DAPM_SINGLE("Right Sidetone Switch", WM8350_LEFT_MIXER_CONTROL, 1, 1, 0),
};

/* Right Playback Mixer */
static const struct snd_kcontrol_new wm8350_right_play_mixer_controls[] = {
SOC_DAPM_SINGLE("Playback Switch", WM8350_RIGHT_MIXER_CONTROL, 12, 1, 0),
SOC_SINGLE_EXT("Right Bypass Switch", WM8350_RIGHT_MIXER_CONTROL, 3, 1, 0,
		snd_soc_dapm_get_volsw, wm8350_dapm_put_volsw),
SOC_DAPM_SINGLE("Left Playback Switch", WM8350_RIGHT_MIXER_CONTROL, 11, 1, 0),
SOC_DAPM_SINGLE("Left Sidetone Switch", WM8350_RIGHT_MIXER_CONTROL, 0, 1, 0),
SOC_DAPM_SINGLE("Right Sidetone Switch", WM8350_RIGHT_MIXER_CONTROL, 1, 1, 0),
};

/* Out4 Mixer */
static const struct snd_kcontrol_new wm8350_out4_mixer_controls[] = {
SOC_DAPM_SINGLE("Right Playback Switch", WM8350_OUT4_MIXER_CONTROL, 12, 1, 0),
SOC_DAPM_SINGLE("Left Playback Switch", WM8350_OUT4_MIXER_CONTROL, 11, 1, 0),
SOC_DAPM_SINGLE("Right Capture Switch", WM8350_OUT4_MIXER_CONTROL, 9, 1, 0),
SOC_DAPM_SINGLE("Out3 Playback Switch", WM8350_OUT4_MIXER_CONTROL, 2, 1, 0),
SOC_DAPM_SINGLE("Right Mixer Switch", WM8350_OUT4_MIXER_CONTROL, 1, 1, 0),
SOC_DAPM_SINGLE("Left Mixer Switch", WM8350_OUT4_MIXER_CONTROL, 0, 1, 0),
};

/* Out3 Mixer */
static const struct snd_kcontrol_new wm8350_out3_mixer_controls[] = {
SOC_DAPM_SINGLE("Left Playback Switch", WM8350_OUT3_MIXER_CONTROL, 11, 1, 0),
SOC_DAPM_SINGLE("Left Capture Switch", WM8350_OUT3_MIXER_CONTROL, 8, 1, 0),
SOC_DAPM_SINGLE("Out4 Playback Switch", WM8350_OUT3_MIXER_CONTROL, 3, 1, 0),
SOC_DAPM_SINGLE("Left Mixer Switch", WM8350_OUT3_MIXER_CONTROL, 0, 1, 0),
};

/* Left Input Mixer */
static const struct snd_kcontrol_new wm8350_left_capt_mixer_controls[] = {
SOC_DAPM_SINGLE_TLV("L2 Capture Volume",
	WM8350_INPUT_MIXER_VOLUME_L, 1, 7, 0, out_mix_tlv),
SOC_DAPM_SINGLE_TLV("AUX Capture Volume",
	WM8350_INPUT_MIXER_VOLUME_L, 9, 7, 0, out_mix_tlv),
SOC_DAPM_SINGLE("PGA Capture Switch",
	WM8350_LEFT_INPUT_VOLUME, 14, 1, 1),
};

/* Right Input Mixer */
static const struct snd_kcontrol_new wm8350_right_capt_mixer_controls[] = {
SOC_DAPM_SINGLE_TLV("L2 Capture Volume",
	WM8350_INPUT_MIXER_VOLUME_R, 5, 7, 0, out_mix_tlv),
SOC_DAPM_SINGLE_TLV("AUX Capture Volume",
	WM8350_INPUT_MIXER_VOLUME_R, 13, 7, 0, out_mix_tlv),
SOC_DAPM_SINGLE("PGA Capture Switch",
	WM8350_RIGHT_INPUT_VOLUME, 14, 1, 1),
};

/* Left Mic Mixer */
static const struct snd_kcontrol_new wm8350_left_mic_mixer_controls[] = {
SOC_DAPM_SINGLE("INN Capture Switch", WM8350_INPUT_CONTROL, 1, 1, 0),
SOC_DAPM_SINGLE("INP Capture Switch", WM8350_INPUT_CONTROL, 0, 1, 0),
SOC_DAPM_SINGLE("IN2 Capture Switch", WM8350_INPUT_CONTROL, 2, 1, 0),
};

/* Right Mic Mixer */
static const struct snd_kcontrol_new wm8350_right_mic_mixer_controls[] = {
SOC_DAPM_SINGLE("INN Capture Switch", WM8350_INPUT_CONTROL, 9, 1, 0),
SOC_DAPM_SINGLE("INP Capture Switch", WM8350_INPUT_CONTROL, 8, 1, 0),
SOC_DAPM_SINGLE("IN2 Capture Switch", WM8350_INPUT_CONTROL, 10, 1, 0),
};

/* Beep Switch */
static const struct snd_kcontrol_new wm8350_beep_switch_controls =
SOC_DAPM_SINGLE("Switch", WM8350_BEEP_VOLUME, 15, 1, 1);

/* Out4 Capture Mux */
static const struct snd_kcontrol_new wm8350_out4_capture_controls =
SOC_DAPM_ENUM("Route", wm8350_enum[8]);


static const struct snd_soc_dapm_widget wm8350_dapm_widgets[] = {

	SND_SOC_DAPM_PGA("Right Aux PGA", WM8350_POWER_MGMT_2, 11, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Aux PGA", WM8350_POWER_MGMT_2, 10, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Input PGA", WM8350_POWER_MGMT_2, 9, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Input PGA", WM8350_POWER_MGMT_2, 8, 0, NULL, 0),
	SND_SOC_DAPM_PGA_E("Right Out2 PGA", WM8350_POWER_MGMT_3, 3, 0, NULL, 0,
		pga_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("Left Out2 PGA", WM8350_POWER_MGMT_3, 2, 0, NULL, 0,
		pga_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("Right Out1 PGA", WM8350_POWER_MGMT_3, 1, 0, NULL, 0,
		pga_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("Left Out1 PGA", WM8350_POWER_MGMT_3, 0, 0, NULL, 0,
		pga_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MIXER("Right Capture Mixer", WM8350_POWER_MGMT_2,
		7, 0, &wm8350_right_capt_mixer_controls[0],
		ARRAY_SIZE(wm8350_right_capt_mixer_controls)),

	SND_SOC_DAPM_MIXER("Left Capture Mixer", WM8350_POWER_MGMT_2,
		6, 0, &wm8350_left_capt_mixer_controls[0],
		ARRAY_SIZE(wm8350_left_capt_mixer_controls)),

	SND_SOC_DAPM_MIXER("Out4 Mixer", WM8350_POWER_MGMT_2, 5, 0,
		&wm8350_out4_mixer_controls[0],
		ARRAY_SIZE(wm8350_out4_mixer_controls)),

	SND_SOC_DAPM_MIXER("Out3 Mixer", WM8350_POWER_MGMT_2, 4, 0,
		&wm8350_out3_mixer_controls[0],
		ARRAY_SIZE(wm8350_out3_mixer_controls)),

	SND_SOC_DAPM_MIXER("Right Playback Mixer", WM8350_POWER_MGMT_2, 1, 0,
		&wm8350_right_play_mixer_controls[0],
		ARRAY_SIZE(wm8350_right_play_mixer_controls)),

	SND_SOC_DAPM_MIXER("Left Playback Mixer", WM8350_POWER_MGMT_2, 0, 0,
		&wm8350_left_play_mixer_controls[0],
		ARRAY_SIZE(wm8350_left_play_mixer_controls)),

	SND_SOC_DAPM_MIXER("Left Mic Mixer", WM8350_POWER_MGMT_2, 8, 0,
		&wm8350_left_mic_mixer_controls[0],
		ARRAY_SIZE(wm8350_left_mic_mixer_controls)),

	SND_SOC_DAPM_MIXER("Right Mic Mixer", WM8350_POWER_MGMT_2, 9, 0,
		&wm8350_right_mic_mixer_controls[0],
		ARRAY_SIZE(wm8350_right_mic_mixer_controls)),

	/* virtual mixer for Beep and Out2R */
	SND_SOC_DAPM_MIXER("Out2 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SWITCH("Beep", WM8350_POWER_MGMT_3, 7, 0,
		&wm8350_beep_switch_controls),

	SND_SOC_DAPM_ADC("Right ADC", "Right Capture",
		WM8350_POWER_MGMT_4, 3, 0),
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture",
		WM8350_POWER_MGMT_4, 2, 0),
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback",
		WM8350_POWER_MGMT_4, 5, 0),
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback",
		WM8350_POWER_MGMT_4, 4, 0),

	SND_SOC_DAPM_MICBIAS("Mic Bias", WM8350_POWER_MGMT_1, 4, 0),

	SND_SOC_DAPM_MUX("Out4 Capture Channel", SND_SOC_NOPM, 0, 0,
		&wm8350_out4_capture_controls),

	SND_SOC_DAPM_OUTPUT("OUT1R"),
	SND_SOC_DAPM_OUTPUT("OUT1L"),
	SND_SOC_DAPM_OUTPUT("OUT2R"),
	SND_SOC_DAPM_OUTPUT("OUT2L"),
	SND_SOC_DAPM_OUTPUT("OUT3"),
	SND_SOC_DAPM_OUTPUT("OUT4"),

	SND_SOC_DAPM_INPUT("IN1RN"),
	SND_SOC_DAPM_INPUT("IN1RP"),
	SND_SOC_DAPM_INPUT("IN2R"),
	SND_SOC_DAPM_INPUT("IN1LP"),
	SND_SOC_DAPM_INPUT("IN1LN"),
	SND_SOC_DAPM_INPUT("IN2L"),
	SND_SOC_DAPM_INPUT("IN3R"),
	SND_SOC_DAPM_INPUT("IN3L"),
};

static const char *audio_map[][3] = {

	/* left playback mixer */
	{"Left Playback Mixer", "Playback Switch", "Left DAC"},
	{"Left Playback Mixer", "Left Bypass Switch", "Left Aux PGA"},
	{"Left Playback Mixer", "Right Playback Switch", "Right DAC"},
	{"Left Playback Mixer", "Left Sidetone Switch", "Left Input PGA"},
	{"Left Playback Mixer", "Right Sidetone Switch", "Right Input PGA"},

	/* right playback mixer */
	{"Right Playback Mixer", "Playback Switch", "Right DAC"},
	{"Right Playback Mixer", "Right Bypass Switch", "Right Aux PGA"},
	{"Right Playback Mixer", "Left Playback Switch", "Left DAC"},
	{"Right Playback Mixer", "Left Sidetone Switch", "Left Input PGA"},
	{"Right Playback Mixer", "Right Sidetone Switch", "Right Input PGA"},

	/* out4 playback mixer */
	{"Out4 Mixer", "Right Playback Switch", "Right DAC"},
	{"Out4 Mixer", "Left Playback Switch", "Left DAC"},
	{"Out4 Mixer", "Right Capture Switch", "Right Capture Mixer"},
	{"Out4 Mixer", "Out3 Playback Switch", "Out3 Mixer"},
	{"Out4 Mixer", "Right Mixer Switch", "Right Playback Mixer"},
	{"Out4 Mixer", "Left Mixer Switch", "Left Playback Mixer"},
	{"OUT4", NULL, "Out4 Mixer"},

	/* out3 playback mixer */
	{"Out3 Mixer", "Left Playback Switch", "Left DAC"},
	{"Out3 Mixer", "Left Capture Switch", "Left Capture Mixer"},
	{"Out3 Mixer", "Left Mixer Switch", "Left Playback Mixer"},
	{"Out3 Mixer", "Out4 Playback Switch", "Out4 Mixer"},
	{"OUT3", NULL, "Out3 Mixer"},

	/* out2 */
	{"Right Out2 PGA", NULL, "Right Playback Mixer"},
	{"Left Out2 PGA", NULL, "Left Playback Mixer"},
	{"OUT2L", NULL, "Left Out2 PGA"},
	{"OUT2R", NULL, "Right Out2 PGA"},

	/* out1 */
	{"Right Out1 PGA", NULL, "Right Playback Mixer"},
	{"Left Out1 PGA", NULL, "Left Playback Mixer"},
	{"OUT1L", NULL, "Left Out1 PGA"},
	{"OUT1R", NULL, "Right Out1 PGA"},

	/* ADC's */
	{"Left ADC", NULL, "Left Capture Mixer"},
	{"Right ADC", NULL, "Right Capture Mixer"},

	/* Left capture mixer */
	{"Left Capture Mixer", "L2 Capture Volume", "IN2L"},
	{"Left Capture Mixer", "AUX Capture Volume", "Left Aux PGA"},
	{"Left Capture Mixer", "PGA Capture Switch", "Left Mic Mixer"},
	{"Left Capture Mixer", NULL, "Out4 Capture Channel"},

	/* Right capture mixer */
	{"Right Capture Mixer", "L2 Capture Volume", "IN2R"},
	{"Right Capture Mixer", "AUX Capture Volume", "Right Aux PGA"},
	{"Right Capture Mixer", "PGA Capture Switch", "Right Mic Mixer"},
	{"Right Capture Mixer", NULL, "Out4 Capture Channel"},

	/* AUX Inputs */
	{"Left Aux PGA", NULL, "IN3L"},
	{"Right Aux PGA", NULL, "IN3R"},

	/* MIC Inputs */
	{"Left Input PGA", NULL, "IN1LN"},
	{"Left Input PGA", NULL, "IN1LP"},
	{"Left Input PGA", NULL, "IN2L"},
	{"Left Capture Mixer", NULL, "Left Input PGA"},
	{"Right Input PGA", NULL, "IN1RN"},
	{"Right Input PGA", NULL, "IN1RP"},
	{"Right Input PGA", NULL, "IN2R"},
	{"Right Capture Mixer", NULL, "Right Input PGA"},

	/* Left Mic mixer */
	{"Left Mic Mixer", "INN Capture Switch", "IN1LN"},
	{"Left Mic Mixer", "INP Capture Switch", "IN1LP"},
	{"Left Mic Mixer", "IN2 Capture Switch", "IN2L"},

	/* Right Mic mixer */
	{"Right Mic Mixer", "INN Capture Switch", "IN1RN"},
	{"Right Mic Mixer", "INP Capture Switch", "IN1RP"},
	{"Right Mic Mixer", "IN2 Capture Switch", "IN2R"},

	/* out 4 capture */
	{"Out4 Capture Channel", NULL, "Out4 Mixer"},

	/* Beep */
	{"Beep", NULL, "Right Aux PGA"},

	/* terminator */
	{NULL, NULL, NULL},
};

static int wm8350_add_widgets(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(wm8350_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec,
			&wm8350_dapm_widgets[i]);
	}

	/* set up audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_new_widgets(machine);
	return 0;
}

static int wm8350_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct wm8350* wm8350 = codec->control_data;
	u16 fll_4;

	switch(clk_id) {
	case WM8350_MCLK_SEL_MCLK:
		wm8350_clear_bits(wm8350, WM8350_CLOCK_CONTROL_1,
			WM8350_MCLK_SEL);
		break;
	case WM8350_MCLK_SEL_PLL_MCLK:
	case WM8350_MCLK_SEL_PLL_DAC:
	case WM8350_MCLK_SEL_PLL_ADC:
	case WM8350_MCLK_SEL_PLL_32K:
		wm8350_set_bits(wm8350, WM8350_CLOCK_CONTROL_1,
			WM8350_MCLK_SEL);
		fll_4 = wm8350_codec_read(codec, WM8350_FLL_CONTROL_4) &
			~WM8350_FLL_CLK_SRC_MASK;
		wm8350_codec_write(codec, WM8350_FLL_CONTROL_4,
			fll_4 | clk_id);
		break;
	}

	/* MCLK direction */
	if (dir == WM8350_MCLK_DIR_OUT)
		wm8350_set_bits(wm8350, WM8350_CLOCK_CONTROL_2,
			WM8350_MCLK_DIR);
	else
		wm8350_clear_bits(wm8350, WM8350_CLOCK_CONTROL_2,
			WM8350_MCLK_DIR);

	return 0;
}

static int wm8350_set_clkdiv(struct snd_soc_dai *codec_dai,
	int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 val;

	switch (div_id) {
	case  WM8350_ADC_CLKDIV:
		val = wm8350_codec_read(codec, WM8350_ADC_DIVIDER) &
			~WM8350_ADC_CLKDIV_MASK;
		wm8350_codec_write(codec, WM8350_ADC_DIVIDER, val | div);
	break;
	case WM8350_DAC_CLKDIV:
		val = wm8350_codec_read(codec, WM8350_DAC_CLOCK_CONTROL) &
			~WM8350_DAC_CLKDIV_MASK;
		wm8350_codec_write(codec, WM8350_DAC_CLOCK_CONTROL, val | div);
	break;
	case WM8350_BCLK_CLKDIV:
		val = wm8350_codec_read(codec, WM8350_CLOCK_CONTROL_1) &
			~WM8350_BCLK_DIV_MASK;
		wm8350_codec_write(codec, WM8350_CLOCK_CONTROL_1, val | div);
	break;
	case WM8350_OPCLK_CLKDIV:
		val = wm8350_codec_read(codec, WM8350_CLOCK_CONTROL_1) &
			~WM8350_OPCLK_DIV_MASK;
		wm8350_codec_write(codec, WM8350_CLOCK_CONTROL_1, val | div);
	break;
	case WM8350_SYS_CLKDIV:
		val = wm8350_codec_read(codec, WM8350_CLOCK_CONTROL_1) &
			~WM8350_MCLK_DIV_MASK;
		wm8350_codec_write(codec, WM8350_CLOCK_CONTROL_1, val | div);
	break;
	case WM8350_DACLR_CLKDIV:
		val = wm8350_codec_read(codec, WM8350_DAC_LR_RATE) &
			~WM8350_DACLRC_RATE_MASK;
		wm8350_codec_write(codec, WM8350_DAC_LR_RATE, val | div);
	break;
	case WM8350_ADCLR_CLKDIV:
		val = wm8350_codec_read(codec, WM8350_ADC_LR_RATE) &
			~WM8350_ADCLRC_RATE_MASK;
		wm8350_codec_write(codec, WM8350_ADC_LR_RATE, val | div);
	break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wm8350_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = wm8350_codec_read(codec, WM8350_AI_FORMATING) &
		~(WM8350_AIF_BCLK_INV | WM8350_AIF_LRCLK_INV |
		WM8350_AIF_FMT_MASK);
	u16 master = wm8350_codec_read(codec, WM8350_AI_DAC_CONTROL) &
		~WM8350_BCLK_MSTR;
	u16 dac_lrc = wm8350_codec_read(codec, WM8350_DAC_LR_RATE) &
		~WM8350_DACLRC_ENA;
	u16 adc_lrc = wm8350_codec_read(codec, WM8350_ADC_LR_RATE) &
		~WM8350_ADCLRC_ENA;


	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		master |= WM8350_BCLK_MSTR;
		dac_lrc |= WM8350_DACLRC_ENA;
		adc_lrc |= WM8350_ADCLRC_ENA;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x2 << 8;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x1 << 8;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x3 << 8;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x3 << 8; // lg not sure which mode
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= WM8350_AIF_LRCLK_INV | WM8350_AIF_BCLK_INV;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= WM8350_AIF_BCLK_INV;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= WM8350_AIF_LRCLK_INV;
		break;
	default:
		return -EINVAL;
	}

	wm8350_codec_write(codec, WM8350_AI_FORMATING, iface);
	wm8350_codec_write(codec, WM8350_AI_DAC_CONTROL, master);
	wm8350_codec_write(codec, WM8350_DAC_LR_RATE, dac_lrc);
	wm8350_codec_write(codec, WM8350_ADC_LR_RATE, adc_lrc);
	return 0;
}

static int wm8350_pcm_trigger(struct snd_pcm_substream *substream,
			      int cmd)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;

	int master = wm8350_codec_cache_read(codec, WM8350_AI_DAC_CONTROL) &
	    WM8350_BCLK_MSTR;
	int enabled = 0;

	/* Check that the DACs or ADCs are enabled since they are
	 * required for LRC in master mode. The DACs or ADCs need a
	 * valid audio path i.e. pin -> ADC or DAC -> pin before
	 * the LRC will be enabled in master mode. */
	if (!master && cmd != SNDRV_PCM_TRIGGER_START)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		enabled = wm8350_codec_cache_read(codec, WM8350_POWER_MGMT_4) &
		    (WM8350_ADCR_ENA | WM8350_ADCL_ENA);
	} else {
		enabled = wm8350_codec_cache_read(codec, WM8350_POWER_MGMT_4) &
		    (WM8350_DACR_ENA | WM8350_DACL_ENA);
	}

	if (!enabled) {
		printk(KERN_ERR
		       "%s: invalid audio path - no clocks available\n",
		       __func__);
		return -EINVAL;
	}
	return 0;
}

static int wm8350_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 iface = wm8350_codec_read(codec, WM8350_AI_FORMATING) &
		~WM8350_AIF_WL_MASK;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x1 << 10;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x2 << 10;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x3 << 10;
		break;
	}

	wm8350_codec_write(codec, WM8350_AI_FORMATING, iface);
	return 0;
}

static int wm8350_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct wm8350* wm8350 = codec->control_data;

	if (mute)
		wm8350_set_bits(wm8350, WM8350_DAC_MUTE, WM8350_DAC_MUTE_ENA);
	else
		wm8350_clear_bits(wm8350, WM8350_DAC_MUTE, WM8350_DAC_MUTE_ENA);
	return 0;
}

/* FLL divisors */
struct _fll_div {
	int div; /* FLL_OUTDIV */
	int n;
	int k;
	int ratio; /* FLL_FRATIO */
};

/* The size in bits of the fll divide multiplied by 10
 * to allow rounding later */
#define FIXED_FLL_SIZE ((1 << 16) * 10)

static inline int fll_factors(struct _fll_div *fll_div, unsigned int input,
	unsigned int output)
{
	u64 Kpart;
	unsigned int t1, t2, K, Nmod;

	if (output >= 2815250 && output <= 3125000)
		fll_div->div = 0x4;
	else if (output >= 5625000 && output <= 6250000)
		fll_div->div = 0x3;
	else if (output >= 11250000 && output <= 12500000)
		fll_div->div = 0x2;
	else if (output >= 22500000 && output <= 25000000)
		fll_div->div = 0x1;
	else {
		printk(KERN_ERR "wm8350: fll freq %d out of range\n", output);
		return -EINVAL;
	}

	if (input > 48000)
		fll_div->ratio = 1;
	else
		fll_div->ratio = 8;

	t1 = output * (1 << (fll_div->div + 1));
	t2 = input * fll_div->ratio;

	fll_div->n = t1 / t2;
	Nmod = t1 % t2;

	if (Nmod) {
		Kpart = FIXED_FLL_SIZE * (long long)Nmod;
		do_div(Kpart, t2);
		K = Kpart & 0xFFFFFFFF;

		/* Check if we need to round */
		if ((K % 10) >= 5)
			K += 5;

		/* Move down to proper range now rounding is done */
		K /= 10;
		fll_div->k = K;
	} else
		fll_div->k = 0;

	return 0;
}


static int wm8350_set_fll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct wm8350* wm8350 = codec->control_data;
	struct _fll_div fll_div;
	int ret = 0;
	u16 fll_1, fll_4;

	if (freq_out == 0 || freq_in == 0) {
		/* power down FLL */
		wm8350_clear_bits(wm8350, WM8350_POWER_MGMT_4,
			WM8350_FLL_ENA | WM8350_FLL_OSC_ENA);
		return ret;
	}

	ret = fll_factors(&fll_div, freq_in, freq_out);
	if (ret < 0)
		return ret;

	/* set up N.K & dividers */
	fll_1 = wm8350_codec_read(codec, WM8350_FLL_CONTROL_1) &
		~(WM8350_FLL_OUTDIV_MASK | 0xc000);
	wm8350_codec_write(codec, WM8350_FLL_CONTROL_1,
		fll_1 | (fll_div.div << 8));
	wm8350_codec_write(codec, WM8350_FLL_CONTROL_2,
		 (fll_div.ratio << 11) | (fll_div.n & WM8350_FLL_N_MASK));
	wm8350_codec_write(codec, WM8350_FLL_CONTROL_3, fll_div.k);
	fll_4 = wm8350_codec_read(codec, WM8350_FLL_CONTROL_4) &
		~(WM8350_FLL_FRAC | WM8350_FLL_SLOW_LOCK_REF);
	wm8350_codec_write(codec, WM8350_FLL_CONTROL_4,
		fll_4 | (fll_div.k ? WM8350_FLL_FRAC : 0) |
		(fll_div.ratio == 8 ? WM8350_FLL_SLOW_LOCK_REF : 0));

	/* power FLL on */
	wm8350_set_bits(wm8350, WM8350_POWER_MGMT_4, WM8350_FLL_OSC_ENA);
	/* do we need to wait here ? */
	wm8350_set_bits(wm8350, WM8350_POWER_MGMT_4, WM8350_FLL_ENA);
	return 0;
}

static int wm8350_set_tdm_slot(struct snd_soc_dai *codec_dai,
		unsigned int mask, int slots)
{
	//struct snd_soc_codec *codec = codec_dai->codec;

	return 0;
}


static int wm8350_set_tristate(struct snd_soc_dai *codec_dai,
	int tristate)
{
	//struct snd_soc_codec *codec = codec_dai->codec;

	return 0;
}

static int wm8350_dapm_event(struct snd_soc_codec *codec, int event)
{
	struct wm8350 *wm8350 = codec->control_data;
	struct wm8350_audio_platform_data *platform = codec->platform_data;
	u16 pm1;

	snd_assert(wm8350 != NULL, return -EINVAL);
	snd_assert(platform != NULL, return -EINVAL);

	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* set vmid to 10k and current to 1.0x */
		pm1 = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_1) &
			~(WM8350_VMID_MASK | WM8350_CODEC_ISEL_MASK);
		wm8350_reg_write(wm8350, WM8350_POWER_MGMT_1,
			pm1 | WM8350_VMID_10K |
			platform->codec_current_d0 << 14);
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		/* set vmid to 40k for quick power up */
		if (codec->dapm_state == SNDRV_CTL_POWER_D3hot) {
			/* D3hot --> D0 */
			/* enable bias */
			pm1 = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_1);
			wm8350_reg_write(wm8350, WM8350_POWER_MGMT_1,
				pm1 | WM8350_BIASEN);
		}
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		if (codec->dapm_state == SNDRV_CTL_POWER_D3cold) {
			/* D3cold --> D3hot */
			/* mute DAC & outputs */
			wm8350_set_bits(wm8350, WM8350_DAC_MUTE,
				WM8350_DAC_MUTE_ENA);

			/* discharge cap memory */
			wm8350_reg_write(wm8350, WM8350_ANTI_POP_CONTROL,
				platform->dis_out1 |
				(platform->dis_out2 << 2) |
				(platform->dis_out3 << 4) |
				(platform->dis_out4 << 6));

			/* wait for discharge */
			schedule_timeout_interruptible(msecs_to_jiffies(
				platform->cap_discharge_msecs));

			/* enable antipop */
			wm8350_reg_write(wm8350, WM8350_ANTI_POP_CONTROL,
				(platform->vmid_s_curve << 8));

			/* ramp up vmid */
			wm8350_reg_write(wm8350, WM8350_POWER_MGMT_1,
				(platform->codec_current_charge << 14) |
				WM8350_VMID_10K | WM8350_VMIDEN | WM8350_VBUFEN);

			/* wait for vmid */
			schedule_timeout_interruptible(msecs_to_jiffies(
				platform->vmid_charge_msecs));

			/* turn on vmid 500k  */
			pm1 = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_1) &
				~(WM8350_VMID_MASK | WM8350_CODEC_ISEL_MASK);
			wm8350_reg_write(wm8350, WM8350_POWER_MGMT_1,
				pm1 | WM8350_VMID_500K |
				(platform->codec_current_d3 << 14));

			/* disable antipop */
			wm8350_reg_write(wm8350, WM8350_ANTI_POP_CONTROL, 0);

		} else {
			/* D1,D2 --> D3hot */
			/* turn on vmid 500k and reduce current */
			pm1 = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_1) &
			~(WM8350_VMID_MASK | WM8350_CODEC_ISEL_MASK);
			wm8350_reg_write(wm8350, WM8350_POWER_MGMT_1,
				pm1 | WM8350_VMID_500K |
				(platform->codec_current_d3 << 14));

			/* disable bias */
			pm1 = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_1) &
				~WM8350_BIASEN;
			wm8350_reg_write(wm8350, WM8350_POWER_MGMT_1,
				pm1);
		}

		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */

		/* mute DAC & enable outputs */
		wm8350_set_bits(wm8350, WM8350_DAC_MUTE,
			WM8350_DAC_MUTE_ENA);

		wm8350_set_bits(wm8350, WM8350_POWER_MGMT_3,
			WM8350_OUT1L_ENA | WM8350_OUT1R_ENA |
			WM8350_OUT2L_ENA | WM8350_OUT2R_ENA);

		/* enable anti pop S curve */
		wm8350_reg_write(wm8350, WM8350_ANTI_POP_CONTROL,
			(platform->vmid_s_curve << 8));

		/* turn off vmid  */
		pm1 = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_1) &
				~WM8350_VMIDEN;
		wm8350_reg_write(wm8350, WM8350_POWER_MGMT_1, pm1);

		/* wait */
		schedule_timeout_interruptible(msecs_to_jiffies(
			platform->vmid_discharge_msecs));

		wm8350_reg_write(wm8350, WM8350_ANTI_POP_CONTROL,
			(platform->vmid_s_curve << 8) |
			platform->dis_out1 |
			(platform->dis_out2 << 2) |
			(platform->dis_out3 << 4) |
			(platform->dis_out4 << 6));

		/* turn off VBuf and drain */
		pm1 = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_1) &
				~(WM8350_VBUFEN | WM8350_VMID_MASK);
		wm8350_reg_write(wm8350, WM8350_POWER_MGMT_1,
			pm1 | WM8350_OUTPUT_DRAIN_EN);

		/* wait */
		schedule_timeout_interruptible(msecs_to_jiffies(
			platform->drain_msecs));

		/* disable anti-pop */
		wm8350_reg_write(wm8350, WM8350_ANTI_POP_CONTROL, 0);

		wm8350_clear_bits(wm8350, WM8350_LOUT1_VOLUME,
			WM8350_OUT1L_ENA);
		wm8350_clear_bits(wm8350, WM8350_ROUT1_VOLUME,
			WM8350_OUT1R_ENA);
		wm8350_clear_bits(wm8350, WM8350_LOUT2_VOLUME,
			WM8350_OUT2L_ENA);
		wm8350_clear_bits(wm8350, WM8350_ROUT2_VOLUME,
			WM8350_OUT2R_ENA);

		/* disable clock gen */
		wm8350_clear_bits(wm8350, WM8350_POWER_MGMT_4, WM8350_SYSCLK_ENA);

		break;
	}
	codec->dapm_state = event;
	return 0;
}

#define WM8350_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
        SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_32000 | \
        SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
        SNDRV_PCM_RATE_96000)


#define WM8350_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_pcm_stream wm8350_hifi_dai_playback = {
	.stream_name	= "Playback",
	.channels_min	= 2,
	.channels_max	= 2,
	.rates		= WM8350_RATES,
	.formats	= WM8350_FORMATS,
};

static const struct snd_soc_pcm_stream wm8350_hifi_dai_capture = {
	.stream_name	= "Capture",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM8350_RATES,
	.formats	= WM8350_FORMATS,
};

/* dai ops, called by machine drivers */
static const struct snd_soc_dai_ops wm8350_hifi_dai_ops = {
	.digital_mute = wm8350_mute,
	.set_fmt = wm8350_set_dai_fmt,
	.set_sysclk = wm8350_set_dai_sysclk,
	.set_tristate = wm8350_set_tristate,
	.set_pll = wm8350_set_fll,
	.set_tdm_slot = wm8350_set_tdm_slot,
	.set_clkdiv = wm8350_set_clkdiv,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm8350_hifi_dai_audio_ops = {
	.hw_params = wm8350_pcm_hw_params,
	.trigger = wm8350_pcm_trigger,
};

static int wm8350_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
// lg save status
	wm8350_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static int wm8350_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	wm8350_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	/* charge wm8350 caps */
	if (codec->suspend_dapm_state == SNDRV_CTL_POWER_D0) {
		wm8350_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
		codec->dapm_state = SNDRV_CTL_POWER_D0;
	}

	return 0;
}

static int wm8350_codec_io_probe(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	struct wm8350* wm8350 = codec->control_data;
	struct wm8350_out_ramp *or = codec->private_data;
	struct wm8350_output *out1 = &or->out1, *out2 = &or->out2;

	snd_assert(wm8350 != NULL, return -EINVAL);

	/* reset codec */
	wm8350_clear_bits(wm8350, WM8350_POWER_MGMT_5, WM8350_CODEC_ENA);
	wm8350_set_bits(wm8350, WM8350_POWER_MGMT_5, WM8350_CODEC_ENA);

	/* enable clock gen - lg need to move in new silicon */
	wm8350_set_bits(wm8350, WM8350_POWER_MGMT_4, WM8350_SYSCLK_ENA);

	/* charge output caps */
	codec->dapm_state = SNDRV_CTL_POWER_D3cold;
	wm8350_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	wm8350_add_controls(codec, machine->card);
	wm8350_add_widgets(codec, machine);

	/* read OUT1 & OUT2 volumes */
	out1->left_vol = (wm8350_reg_read(wm8350, WM8350_LOUT1_VOLUME) &
		WM8350_OUT1L_VOL_MASK) >> WM8350_OUT1L_VOL_SHIFT;
	out1->right_vol = (wm8350_reg_read(wm8350, WM8350_ROUT1_VOLUME) &
		WM8350_OUT1R_VOL_MASK) >> WM8350_OUT1R_VOL_SHIFT;
	out2->left_vol = (wm8350_reg_read(wm8350, WM8350_LOUT2_VOLUME) &
		WM8350_OUT2L_VOL_MASK) >> WM8350_OUT1L_VOL_SHIFT;
	out2->right_vol = (wm8350_reg_read(wm8350, WM8350_ROUT2_VOLUME) &
		WM8350_OUT2R_VOL_MASK) >> WM8350_OUT1R_VOL_SHIFT;
	wm8350_reg_write(wm8350, WM8350_LOUT1_VOLUME, 0);
	wm8350_reg_write(wm8350, WM8350_ROUT1_VOLUME, 0);
	wm8350_set_bits(wm8350, WM8350_LOUT1_VOLUME, WM8350_OUT1_VU);
	wm8350_reg_write(wm8350, WM8350_LOUT2_VOLUME, 0);
	wm8350_reg_write(wm8350, WM8350_ROUT2_VOLUME, 0);
	wm8350_set_bits(wm8350, WM8350_LOUT2_VOLUME, WM8350_OUT2_VU);

	return 0;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

static int wm8350_codec_io_remove(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	struct wm8350* wm8350 = codec->control_data;

	snd_assert(wm8350 != NULL, return -EINVAL);
	run_delayed_work(&codec->delayed_work);
	wm8350_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static const struct snd_soc_codec_ops wm8350_codec_ops = {
	.dapm_event	= wm8350_dapm_event,
	.read		= wm8350_codec_read,
	.write		= wm8350_codec_write,
	.io_probe	= wm8350_codec_io_probe,
	.io_remove	= wm8350_codec_io_remove,
};

static int wm8350_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	struct wm8350_out_ramp *or;
	info("WM8350 Audio Codec %s", WM8350_VERSION);

	or = kzalloc(sizeof(struct wm8350_out_ramp), GFP_KERNEL);
	if (or == NULL)
		return -ENOMEM;

	codec->owner = THIS_MODULE;
	codec->ops = &wm8350_codec_ops;
	codec->reg_cache_size = WM8350_MAX_REGISTER;
	codec->reg_cache_step = 1;
	codec->private_data = or;
	INIT_DELAYED_WORK(&codec->delayed_work, wm8350_pga_work);
 	snd_soc_register_codec(codec);
	return 0;
}

static int wm8350_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	kfree(codec->private_data);
	return 0;
}

static int wm8350_hifi_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);

	dai->ops = &wm8350_hifi_dai_ops;
	dai->audio_ops = &wm8350_hifi_dai_audio_ops;
	dai->capture = &wm8350_hifi_dai_capture;
	dai->playback = &wm8350_hifi_dai_playback;
	snd_soc_register_codec_dai(dai);
	return 0;
}

const char wm8350_codec[SND_SOC_CODEC_NAME_SIZE] = "wm8350-codec";
EXPORT_SYMBOL_GPL(wm8350_codec);

static struct snd_soc_device_driver wm8350_codec_driver = {
	.type	= SND_SOC_BUS_TYPE_CODEC,
	.driver	= {
		.name 		= wm8350_codec,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8350_codec_probe,
		.remove		= __devexit_p(wm8350_codec_remove),
		.suspend	= wm8350_suspend,
		.resume		= wm8350_resume,
	},
};

const char wm8350_hifi_dai[SND_SOC_CODEC_NAME_SIZE] = "wm8350-hifi-dai";
EXPORT_SYMBOL_GPL(wm8350_hifi_dai);

static struct snd_soc_device_driver wm8350_hifi_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm8350_hifi_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8350_hifi_dai_probe,
	},
};

static __init int wm8350_init(void)
{
	int ret = 0;

	ret = driver_register(&wm8350_codec_driver.driver);
	if (ret < 0)
		return ret;
	ret = driver_register(&wm8350_hifi_dai_driver.driver);
	if (ret < 0) {
		driver_unregister(&wm8350_codec_driver.driver);
		return ret;
	}
	return ret;
}

static __exit void wm8350_exit(void)
{
	driver_unregister(&wm8350_hifi_dai_driver.driver);
	driver_unregister(&wm8350_codec_driver.driver);
}

module_init(wm8350_init);
module_exit(wm8350_exit);

MODULE_DESCRIPTION("ASoC WM8350 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");

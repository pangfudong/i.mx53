/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file bluetooth.c
 * @brief Driver for bluetooth PCM interface
 *
 * @ingroup Sound
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#define AUDIO_NAME "bluetooth"

#define BLUETOOTH_RATES SNDRV_PCM_RATE_8000

#define BLUETOOTH_FORMATS SNDRV_PCM_FMTBIT_S16_LE

static const struct snd_soc_pcm_stream bt_dai_playback = {
	.stream_name = "Playback",
	.channels_min = 1,
	.channels_max = 2,
	.rates = BLUETOOTH_RATES,
	.formats = BLUETOOTH_FORMATS,
};

static const struct snd_soc_pcm_stream bt_dai_capture = {
	.stream_name = "Capture",
	.channels_min = 1,
	.channels_max = 2,
	.rates = BLUETOOTH_RATES,
	.formats = BLUETOOTH_FORMATS,
};

static int bt_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	codec->owner = THIS_MODULE;
	snd_soc_register_codec(codec);
	return 0;
}

static int bt_codec_remove(struct device *dev)
{
	return 0;
}

static int bt_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
	dai->capture = &bt_dai_capture;
	dai->playback = &bt_dai_playback;
	snd_soc_register_codec_dai(dai);
	return 0;
}

const char bt_codec[SND_SOC_CODEC_NAME_SIZE] = "bt-codec";
EXPORT_SYMBOL_GPL(bt_codec);

static struct snd_soc_device_driver bt_codec_driver = {
	.type = SND_SOC_BUS_TYPE_CODEC,
	.driver = {
		   .name = bt_codec,
		   .owner = THIS_MODULE,
		   .bus = &asoc_bus_type,
		   .probe = bt_codec_probe,
		   .remove = __devexit_p(bt_codec_remove),
		   },
};

const char bt_dai[SND_SOC_CODEC_NAME_SIZE] = "bt-dai";
EXPORT_SYMBOL_GPL(bt_dai);

static struct snd_soc_device_driver bt_dai_driver = {
	.type = SND_SOC_BUS_TYPE_DAI,
	.driver = {
		   .name = bt_dai,
		   .owner = THIS_MODULE,
		   .bus = &asoc_bus_type,
		   .probe = bt_dai_probe,
		   },
};

static __init int bt_codec_init(void)
{
	int ret = 0;

	ret = driver_register(&bt_codec_driver.driver);
	if (ret < 0)
		return ret;
	ret = driver_register(&bt_dai_driver.driver);
	if (ret < 0) {
		driver_unregister(&bt_codec_driver.driver);
		return ret;
	}

	return ret;
}

static __exit void bt_codec_exit(void)
{
	driver_unregister(&bt_dai_driver.driver);
	driver_unregister(&bt_codec_driver.driver);
}

subsys_initcall(bt_codec_init);
module_exit(bt_codec_exit);

MODULE_DESCRIPTION("ASoC bluetooth codec driver");
MODULE_LICENSE("GPL");

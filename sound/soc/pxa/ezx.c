/*
 * ezx.c - Machine specific code for EZX phones
 *
 *	Copyright (C) 2007-2008 Daniel Ribeiro <drwyrm@gmail.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/pxa27x.h>
#include <mach/pxa2xx-regs.h>
#include <mach/regs-ssp.h>

#include <linux/mfd/ezx-pcap.h>

#include "../codecs/pcap2.h"
#include "pxa2xx-pcm.h"
#include "pxa-ssp.h"

#define AUDIO_OFF		0
#define LOUDSPEAKER		1
#define EARPIECE		2
#define HEADSET			3

static struct snd_soc_codec *control_codec;

static void ezx_ext_control(struct snd_soc_codec *codec)
{
/*	u32 tmp;

	ezx_pcap_read(PCAP_REG_PSTAT, &tmp);

	if (tmp & PCAP_IRQ_A1)
		snd_soc_dapm_enable_pin(codec, "Headset");
	else
		snd_soc_dapm_disable_pin(codec, "Headset");

	if (tmp & PCAP_IRQ_MB2)
		snd_soc_dapm_enable_pin(codec, "External Mic");
	else
		snd_soc_dapm_disable_pin(codec, "External Mic");

	snd_soc_dapm_sync(codec);
*/
}

static irqreturn_t jack_irq(struct work_struct *unused)
{
	ezx_ext_control(control_codec);
	return IRQ_HANDLED;
}

static int ezx_scenario;
static int ezx_get_scenario(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ezx_scenario;
	return 0;
}

static int ezx_set_scenario_endpoints(struct snd_soc_codec *codec, int mode)
{
	switch (mode) {
	case AUDIO_OFF:
		snd_soc_dapm_disable_pin(codec, "Earpiece");
		snd_soc_dapm_disable_pin(codec, "Loudspeaker");
		snd_soc_dapm_disable_pin(codec, "Headset");
		snd_soc_dapm_disable_pin(codec, "Built-in Mic");
		snd_soc_dapm_disable_pin(codec, "External Mic");
		break;
	case EARPIECE:
		snd_soc_dapm_enable_pin(codec, "Earpiece");
		snd_soc_dapm_disable_pin(codec, "Loudspeaker");
		snd_soc_dapm_disable_pin(codec, "Headset");
		snd_soc_dapm_enable_pin(codec, "Built-in Mic");
		snd_soc_dapm_disable_pin(codec, "External Mic");
		break;
	case LOUDSPEAKER:
		snd_soc_dapm_disable_pin(codec, "Earpiece");
		snd_soc_dapm_enable_pin(codec, "Loudspeaker");
		snd_soc_dapm_disable_pin(codec, "Headset");
		snd_soc_dapm_enable_pin(codec, "Built-in Mic");
		snd_soc_dapm_disable_pin(codec, "External Mic");
		break;
	case HEADSET:
		snd_soc_dapm_disable_pin(codec, "Earpiece");
		snd_soc_dapm_disable_pin(codec, "Loudspeaker");
		snd_soc_dapm_enable_pin(codec, "Headset");
		snd_soc_dapm_disable_pin(codec, "Built-in Mic");
		snd_soc_dapm_enable_pin(codec, "External Mic");
		break;
	}

	snd_soc_dapm_sync(codec);

	return 0;
}
static int ezx_set_scenario(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (ezx_scenario == ucontrol->value.integer.value[0])
		return 0;

	ezx_scenario = ucontrol->value.integer.value[0];

	ezx_set_scenario_endpoints(codec, ezx_scenario);

	return 1;
}

static const char *ezx_scenario_select[] = {
	"Off",
	"Loudspeaker",
	"Earpiece",
	"Headset",
};

static const struct soc_enum ezx_scenario_enum[] = {
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ezx_scenario_select), ezx_scenario_select),
};

static const struct snd_kcontrol_new ezx_snd_controls[] = {
SOC_ENUM_EXT("Output mode", ezx_scenario_enum[0], ezx_get_scenario,
							ezx_set_scenario),
};

/*
 * Alsa operations
 * Only implement the required operations for your platform.
 * These operations are specific to the machine only.
 */

/*
 * Called by ALSA when a PCM substream is opened, private data can be allocated.
 */
static int ezx_machine_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->card->codec;

	/* check the jack status at stream startup */
	ezx_ext_control(codec);
	return 0;
}

/*
 * Called by ALSA when the hardware params are set by application. This
 * function can also be called multiple times and can allocate buffers
 * (using snd_pcm_lib_* ). It's non-atomic.
 */
static int ezx_machine_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_B |
			SND_SOC_DAIFMT_IB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* Turn on clock output on CLK_PIO */
	OSCC |= 0x8;

	/* set clock source */
	ret = snd_soc_dai_set_sysclk(codec_dai, PCAP2_CLK_AP,
					13000000, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* setup TDM slots */
	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 1, 1);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_B |
			SND_SOC_DAIFMT_IB_IF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_tristate(cpu_dai, 0);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA_SSP_CLK_PLL,
						0, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * Free's resources allocated by hw_params, can be called multiple times
 */
static int ezx_machine_hw_free(struct snd_pcm_substream *substream)
{
	OSCC &= ~0x8; /* turn off clock output on CLK_PIO */

	return 0;
}

/* machine Alsa PCM operations */
static struct snd_soc_ops ezx_ops = {
	.startup = ezx_machine_startup,
	.hw_free = ezx_machine_hw_free,
	.hw_params = ezx_machine_hw_params,
};

static int bp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret = 0;
	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_B |
		SND_SOC_DAIFMT_IB_IF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set clock source */
	ret = snd_soc_dai_set_sysclk(codec_dai, PCAP2_CLK_BP,
					13000000, SND_SOC_CLOCK_IN);

	return ret;
}

/* machine dapm widgets */
static const struct snd_soc_dapm_widget ezx_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headset", NULL),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_SPK("Loudspeaker", NULL),
	SND_SOC_DAPM_MIC("Built-in Mic", NULL),
	SND_SOC_DAPM_MIC("External Mic", NULL),
};

/* machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map[] = {
	{ "Headset", NULL, "AR" },
	{ "Headset", NULL, "AL" },
	{ "Earpiece", NULL, "A1" },
	{ "Loudspeaker", NULL, "A2" },

	{ "Built-in Mic", NULL, "A5" },
	{ "External Mic", NULL, "A3" },
};

/*
 * Initialise the machine audio subsystem.
 */
static int ezx_machine_init(struct snd_soc_codec *codec)
{
	int i, err;

	control_codec = codec;

	/* Add ezx specific widgets */
	snd_soc_dapm_new_controls(codec, ezx_dapm_widgets,
						ARRAY_SIZE(ezx_dapm_widgets));

	for (i = 0; i < ARRAY_SIZE(ezx_snd_controls); i++) {
		if ((err = snd_ctl_add(codec->card,
				snd_soc_cnew(&ezx_snd_controls[i], codec,
				NULL))) < 0)
			return err;
	}

	/* Set up ezx specific audio path interconnects */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

//	snd_soc_dapm_new_widgets(codec);

	ezx_scenario = AUDIO_OFF;
	ezx_set_scenario_endpoints(codec, ezx_scenario);

	/* synchronise subsystem */
	snd_soc_dapm_sync(codec);
	return 0;
}

/* template digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ezx_dai[] = {
{
	.name = "PCAP2 STEREO",
	.stream_name = "Stereo playback",
	.cpu_dai = &pxa_ssp_dai[PXA_DAI_SSP3],
	.codec_dai = &pcap2_dai[0],
	.init = ezx_machine_init,
	.ops = &ezx_ops,
},
};

/* template audio machine driver */
static struct snd_soc_card snd_soc_machine_ezx = {
	.name = "Motorola EZX",
	.platform = &pxa2xx_soc_platform,
//	.probe
//	.remove
//	.suspend_pre
//	.resume_post
	.dai_link = ezx_dai,
	.num_links = ARRAY_SIZE(ezx_dai),
};

/* template audio subsystem */
static struct snd_soc_device ezx_snd_devdata = {
	.card = &snd_soc_machine_ezx,
	.codec_dev = &soc_codec_dev_pcap2,
};

static struct platform_device *ezx_snd_device;

static int __init ezx_init(void)
{
	int ret;

	ezx_snd_device = platform_device_alloc("soc-audio", -1);
	if (!ezx_snd_device)
		return -ENOMEM;

	platform_set_drvdata(ezx_snd_device, &ezx_snd_devdata);
	ezx_snd_devdata.dev = &ezx_snd_device->dev;
	ret = platform_device_add(ezx_snd_device);

	if (ret)
		platform_device_put(ezx_snd_device);

#ifdef CONFIG_PXA_EZX_A780
	if (machine_is_ezx_a780())
		gpio_direction_output(96, 1);
#endif

	/* request jack event */
	/* not deal this event temporarily */
	/*
	ezx_pcap_register_event(PCAP_IRQ_MB2 | PCAP_IRQ_A1, jack_irq,
							NULL, "HP/MIC");
	*/
	return ret;
}

static void __exit ezx_exit(void)
{
	/*
	ezx_pcap_unregister_event(PCAP_IRQ_MB2 | PCAP_IRQ_A1);
	*/
	platform_device_unregister(ezx_snd_device);
}

module_init(ezx_init);
module_exit(ezx_exit);
MODULE_LICENSE("GPL");

/*
 * ezx.c - Machine specific code for EZX phones
 *
 *	Copyright (C) 2007 Daniel Ribeiro <wyrm@openezx.org>
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
#include <mach/pxa-regs.h>
#include <mach/pxa2xx-regs.h>
#include <mach/regs-ssp.h>
//#include <asm/arch/hardware.h>
//#include <asm/arch/gpio.h>

#include <linux/mfd/ezx-pcap.h>

#include "../codecs/pcap2.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-ssp.h"

#define GPIO_HW_ATTENUATE_A780  96

static struct snd_soc_codec *control_codec;

static void ezx_ext_control(struct snd_soc_codec *codec)
{
	u32 tmp;

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
}

static irqreturn_t jack_irq(struct work_struct *unused)
{
	ezx_ext_control(control_codec);
	return IRQ_HANDLED;
}


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
	struct snd_soc_codec *codec = rtd->socdev->codec;

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
	if (codec_dai->id == PCAP2_STEREO_DAI)
		ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_B |
			SND_SOC_DAIFMT_IB_NF | SND_SOC_DAIFMT_CBM_CFM);
	else
		ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_B |
			SND_SOC_DAIFMT_IB_IF | SND_SOC_DAIFMT_CBM_CFM);
	if(ret < 0)
		return ret;

	/* Turn on clock output on CLK_PIO */
	OSCC |= 0x8;

	/* set clock source */
	ret = codec_dai->dai_ops.set_sysclk(codec_dai, PCAP2_CLK_AP,
					13000000, SND_SOC_CLOCK_IN);
	if(ret < 0)
		return ret;

	/* set cpu DAI configuration */
	if (codec_dai->id == PCAP2_STEREO_DAI)
		ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_MSB |
				SND_SOC_DAIFMT_IB_IF | SND_SOC_DAIFMT_CBM_CFM);
	else
		ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_B |
				SND_SOC_DAIFMT_IB_IF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = cpu_dai->dai_ops.set_tristate(cpu_dai, 0);
	if (ret < 0)
		return ret;

	ret = cpu_dai->dai_ops.set_sysclk(cpu_dai,PXA2XX_SSP_CLK_EXT,
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
//	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_B |
		SND_SOC_DAIFMT_IB_IF | SND_SOC_DAIFMT_CBM_CFM);
	if(ret < 0)
		return ret;

	/* set clock source */
	ret = codec_dai->dai_ops.set_sysclk(codec_dai, PCAP2_CLK_BP,
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

	{NULL, NULL, NULL},
};

/*
 * Initialise the machine audio subsystem.
 */
static int ezx_machine_init(struct snd_soc_codec *codec)
{
	int i;
	/* mark unused codec pins as NC */
//	snd_soc_dapm_set_endpoint(codec, "FIXME", 0);
	control_codec = codec;

        /* Add ezx specific controls */
//	for (i = 0; i < ARRAY_SIZE(ezx_controls); i++) {
//		if ((err = snd_ctl_add(codec->card, snd_soc_cnew(&ezx_controls[i], codec, NULL))) < 0)
//			return err;
//	}

	/* Add ezx specific widgets */
	for(i = 0; i < ARRAY_SIZE(ezx_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &ezx_dapm_widgets[i]);
	}
	/* Set up ezx specific audio path interconnects */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* synchronise subsystem */
	snd_soc_dapm_sync(codec);
	return 0;
}

static struct snd_soc_dai bp_dai =
{
	.name = "Baseband",
	.id = 0,
	.type = SND_SOC_DAI_PCM,
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = {
//		.startup = bp_startup,
//		.shutdown = bp_shutdown,
		.hw_params = bp_hw_params,
//		.hw_free = bp_hw_free,
	},
};

/* template digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ezx_dai[] = {
{
	.name = "PCAP2 STEREO",
	.stream_name = "stereo playback",
	.cpu_dai = &pxa_ssp_dai[PXA2XX_DAI_SSP3],
	.codec_dai = &pcap2_dai[PCAP2_STEREO_DAI],
	.init = ezx_machine_init,
	.ops = &ezx_ops,
},
{
	.name = "PCAP2 MONO",
	.stream_name = "mono playback",
	.cpu_dai = &pxa_ssp_dai[PXA2XX_DAI_SSP3],
	.codec_dai = &pcap2_dai[PCAP2_MONO_DAI],
//	.init = ezx_machine_init, /* the stereo call already registered our controls */
	.ops = &ezx_ops,
},
{
	.name = "PCAP2 BP",
	.stream_name = "BP Audio",
	.cpu_dai = &bp_dai,
	.codec_dai = &pcap2_dai[PCAP2_BP_DAI],
},
};

/* template audio machine driver */
static struct snd_soc_machine snd_soc_machine_ezx = {
	.name = "Motorola EZX",
//	.probe
//	.remove
//	.suspend_pre
//	.resume_post
	.dai_link = ezx_dai,
	.num_links = ARRAY_SIZE(ezx_dai),
};

/* template audio subsystem */
static struct snd_soc_device ezx_snd_devdata = {
	.machine = &snd_soc_machine_ezx,
	.platform = &pxa2xx_soc_platform,
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

	/* configure gpio for ssp3 */
//	pxa_gpio_mode(GPIO83_SFRM3_MD);	/* SFRM */
//	pxa_gpio_mode(GPIO81_STXD3_MD);	/* TXD  */
//	pxa_gpio_mode(GPIO52_SCLK3_MD);	/* SCLK */
//	pxa_gpio_mode(GPIO89_SRXD3_MD);	/* RXD  */

	/* configure gpio for ssp2 */
//	pxa_gpio_mode(37 | GPIO_IN);	/* SFRM */
//	pxa_gpio_mode(38 | GPIO_IN);	/* TXD  */
//	pxa_gpio_mode(22 | GPIO_IN);	/* SCLK */
//	pxa_gpio_mode(88 | GPIO_IN);	/* RXD  */

#if 0 //CONFIG_PXA_EZX_A780
	pxa_gpio_mode(GPIO_HW_ATTENUATE_A780 | GPIO_OUT);
	gpio_set_value(GPIO_HW_ATTENUATE_A780, 1);
#endif

	/* request jack event */
	ezx_pcap_register_event(PCAP_IRQ_MB2 | PCAP_IRQ_A1, jack_irq, "HP/MIC");

	return ret;
}

static void __exit ezx_exit(void)
{
	ezx_pcap_unregister_event(PCAP_IRQ_MB2 | PCAP_IRQ_A1);
	platform_device_unregister(ezx_snd_device);
}

module_init(ezx_init);
module_exit(ezx_exit);


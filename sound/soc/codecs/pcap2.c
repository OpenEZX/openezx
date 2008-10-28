/*
 * pcap2.c - PCAP2 ASIC Audio driver
 *
 * 	Copyright (C) 2007 Daniel Ribeiro <wyrm@openezx.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
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
//#include <mach/pxa-regs.h>
#include <linux/mfd/ezx-pcap.h>
//#include <asm/arch/hardware.h>

#include "pcap2.h"

#define AUDIO_NAME "pcap2-codec"
#define PCAP2_VERSION "0.1"

extern int ezx_pcap_write(u_int8_t, u_int32_t);
extern int ezx_pcap_read(u_int8_t, u_int32_t *);
static struct snd_soc_device *pcap2_codec_socdev;

/*
 * Debug
 */

//#define PCAP2_DEBUG

#ifdef PCAP2_DEBUG
#define dbg(format, arg...) \
	printk(KERN_DEBUG AUDIO_NAME ": " format "\n" , ## arg)
#else
#define dbg(format, arg...)
#endif

#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)

#define dump_registers()	pcap2_codec_read(NULL, 13); \
				pcap2_codec_read(NULL, 12); \
				pcap2_codec_read(NULL, 11); \
				pcap2_codec_read(NULL, 26);




/*
 * ASoC limits register value to 16 bits and pcap uses 32 bit registers
 * to work around this, we get 16 bits from low, mid or high positions.
 * ASoC limits register number to 8 bits we use 0x1f for register
 * number and 0xe0 for register offset. -WM
 */
static int pcap2_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	unsigned int tmp;

	ezx_pcap_read((reg & 0x1f), &tmp);

	if (reg & SL) {
		tmp &= 0xffff0000;
		tmp |= (value & 0xffff);
	}
	else if (reg & SM) {
		tmp &= 0xff0000ff;
		tmp |= ((value << 8) & 0x00ffff00);
	}
	else if (reg & SH) {
		tmp &= 0xffff;
		tmp |= ((value << 16) & 0xffff0000);
	}
	else
		tmp = value;

	dbg("codec_write reg=%x, rval=%08x, fval=%08x", reg, tmp,  value);
	ezx_pcap_write((reg & 0x1f), tmp);
	return 0;

}

static unsigned int pcap2_codec_read(struct snd_soc_codec *codec, unsigned int reg)
{
	unsigned int tmp, ret;

	ezx_pcap_read((reg & 0x1f), &tmp);
	ret = tmp;
	if (reg & SL)
		ret = (tmp & 0xffff);
	else if (reg & SM)
		ret = ((tmp >> 8) & 0xffff);
	else if (reg & SH)
		ret = ((tmp >> 16) & 0xffff);

	dbg("codec_read  reg=%x, rval=%08x, fval=%08x", reg, tmp, ret);
	return(ret);

}

static const char *pcap2_output_select[] = {"2ch", "2->1ch", "2->1ch -3db", "2->1ch -6db"};

static const struct soc_enum pcap2_enum[] = {
SOC_ENUM_SINGLE((PCAP2_OUTPUT_AMP|SH), 3, 4, pcap2_output_select),
};

static const struct snd_kcontrol_new pcap2_input_mixer_controls[] = {
SOC_DAPM_SINGLE("A3 Switch", (PCAP2_INPUT_AMP|SL), 6, 1, 0),
SOC_DAPM_SINGLE("A5 Switch", (PCAP2_INPUT_AMP|SL), 8, 1, 0),
};

static const struct snd_kcontrol_new pcap2_output_mixer_controls[] = {
SOC_DAPM_SINGLE("A1 Switch", (PCAP2_OUTPUT_AMP|SL), 0, 1, 0),
SOC_DAPM_SINGLE("A2 Switch", (PCAP2_OUTPUT_AMP|SL), 1, 1, 0),
SOC_DAPM_SINGLE("AR Switch", (PCAP2_OUTPUT_AMP|SL), 5, 1, 0),
SOC_DAPM_SINGLE("AL Switch", (PCAP2_OUTPUT_AMP|SL), 6, 1, 0),
};

/* pcap2 codec non DAPM controls */
static const struct snd_kcontrol_new pcap2_codec_snd_controls[] = {
SOC_SINGLE("Master Playback Volume", (PCAP2_OUTPUT_AMP|SM),  5, 15, 0),
SOC_SINGLE("Capture Volume", (PCAP2_INPUT_AMP|SL),   0, 31, 0),
};

static const struct snd_kcontrol_new pcap2_codec_dm_mux_control[] = {
	SOC_DAPM_ENUM("Output Mode",	pcap2_enum[0]),
};

/* add non dapm controls */
static int pcap2_codec_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(pcap2_codec_snd_controls); i++) {
		if ((err = snd_ctl_add(codec->card,
				snd_soc_cnew(&pcap2_codec_snd_controls[i],codec, NULL))) < 0)
			return err;
	}

	return 0;
}

/* pcap2 codec DAPM controls */
static const struct snd_soc_dapm_widget pcap2_codec_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("ST_DAC", "ST_DAC playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("CDC_DAC", "CDC_DAC playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("CDC_ADC", "CDC_DAC capture", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA("PGA_ST", (PCAP2_OUTPUT_AMP|SL), 9, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA_CDC", (PCAP2_OUTPUT_AMP|SL), 8, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA_R", (PCAP2_OUTPUT_AMP|SL), 11, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA_L", (PCAP2_OUTPUT_AMP|SL), 12, 0, NULL, 0),
	SND_SOC_DAPM_MUX("Downmixer", SND_SOC_NOPM, 0, 0, pcap2_codec_dm_mux_control),
	SND_SOC_DAPM_PGA("PGA_A1CTRL", (PCAP2_OUTPUT_AMP|SH), 1, 1, NULL, 0),
	SND_SOC_DAPM_MIXER("Output Mixer", SND_SOC_NOPM, 0, 0, &pcap2_output_mixer_controls[0], ARRAY_SIZE(pcap2_output_mixer_controls)),
	SND_SOC_DAPM_OUTPUT("A1"), /* Earpiece */
	SND_SOC_DAPM_OUTPUT("A2"), /* LoudSpeaker */
	SND_SOC_DAPM_OUTPUT("AR"), /* headset right */
	SND_SOC_DAPM_OUTPUT("AL"), /* headset left */

	SND_SOC_DAPM_MICBIAS("BIAS1", (PCAP2_INPUT_AMP|SL), 10, 0),
	SND_SOC_DAPM_MICBIAS("BIAS2", (PCAP2_INPUT_AMP|SL), 11, 0),
	SND_SOC_DAPM_MIXER("Input Mixer", SND_SOC_NOPM, 0, 0, &pcap2_input_mixer_controls[0], ARRAY_SIZE(pcap2_input_mixer_controls)),
	SND_SOC_DAPM_INPUT("A3"), /* Headset Mic */
	SND_SOC_DAPM_INPUT("A5"), /* Builtin Mic */
};

static const struct snd_soc_dapm_route audio_map[] = {
	{ "A1", NULL, "Output Mixer" },
	{ "A2", NULL, "Output Mixer" },
	{ "AR", NULL, "Output Mixer" },
	{ "AL", NULL, "Output Mixer" },

	{ "Output Mixer", "A1 Switch", "PGA_A1CTRL" },
	{ "Output Mixer", "A2 Switch", "Downmixer" },
	{ "Output Mixer", "AR Switch", "PGA_R" },
	{ "Output Mixer", "AL Switch", "PGA_L" },

	{ "PGA_A1CTRL", NULL, "Downmixer" },

	{ "Downmixer", "2->1ch", "PGA_L" },
	{ "Downmixer", "2->1ch", "PGA_R" },
	{ "Downmixer", "2->1ch -3db", "PGA_L" },
	{ "Downmixer", "2->1ch -3db", "PGA_R" },
	{ "Downmixer", "2->1ch -6db", "PGA_L" },
	{ "Downmixer", "2->1ch -6db", "PGA_R" },
	{ "Downmixer", "2ch", "PGA_R" },

	{ "PGA_R", NULL, "PGA_ST" },
	{ "PGA_L", NULL, "PGA_ST" },
	{ "PGA_R", NULL, "PGA_CDC" },

	{ "PGA_ST", NULL, "ST_DAC" },
	{ "PGA_CDC", NULL, "CDC_DAC" },

	/* input path */
	{ "BIAS1", NULL, "A3" },
	{ "BIAS2", NULL, "A5" },

	{ "Input Mixer", "A3 Switch", "BIAS1" },
	{ "Input Mixer", "A5 Switch", "BIAS2" },

	{ "PGA_R", NULL, "Input Mixer" },

	{ "PGA_CDC", NULL, "PGA_R" },
	{ "CDC_ADC", NULL, "PGA_CDC" },
};

static int pcap2_codec_add_widgets(struct snd_soc_codec *codec)
{
//	int i;

//	for(i = 0; i < ARRAY_SIZE(pcap2_codec_dapm_widgets); i++) {
//		snd_soc_dapm_new_control(codec, &pcap2_codec_dapm_widgets[i]);
//	}
	snd_soc_dapm_new_controls(codec, pcap2_codec_dapm_widgets,
				ARRAY_SIZE(pcap2_codec_dapm_widgets));

	/* set up audio path interconnects */
//	for(i = 0; audio_map[i][0] != NULL; i++) {
//		snd_soc_dapm_connect_input(codec, audio_map[i][0],
//			audio_map[i][1], audio_map[i][2]);
//	}
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int pcap2_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	unsigned int input = pcap2_codec_read(codec, PCAP2_INPUT_AMP);

	input &= ~PCAP2_INPUT_AMP_LOWPWR;

	switch (level) {

	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		input |= PCAP2_INPUT_AMP_LOWPWR;
		break;
	}
	codec->bias_level = level;
	pcap2_codec_write(codec, PCAP2_INPUT_AMP, input);
	return 0;
}

static int pcap2_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int tmp;

	if (codec_dai->id == PCAP2_STEREO_DAI) {
		tmp = pcap2_codec_read(codec, PCAP2_ST_DAC);

		tmp &= ~PCAP2_ST_DAC_RATE_MASK;
		switch(params_rate(params)) {
		case 8000:
			break;
		case 11025:
			tmp |= PCAP2_ST_DAC_RATE_11025;
			break;
		case 12000:
			tmp |= PCAP2_ST_DAC_RATE_12000;
			break;
		case 16000:
			tmp |= PCAP2_ST_DAC_RATE_16000;
			break;
		case 22050:
			tmp |= PCAP2_ST_DAC_RATE_22050;
			break;
		case 24000:
			tmp |= PCAP2_ST_DAC_RATE_24000;
			break;
		case 32000:
			tmp |= PCAP2_ST_DAC_RATE_32000;
			break;
		case 44100:
			tmp |= PCAP2_ST_DAC_RATE_44100;
			break;
		case 48000:
			tmp |= PCAP2_ST_DAC_RATE_48000;
			break;
		default:
			return -EINVAL;
		}
		tmp |= PCAP2_ST_DAC_RESET_DF;
		pcap2_codec_write(codec, PCAP2_ST_DAC, tmp);
	}
	else {
		tmp = pcap2_codec_read(codec, PCAP2_CODEC);

		tmp &= ~PCAP2_CODEC_RATE_MASK;
		switch(params_rate(params)) {
		case 8000:
			break;
		case 16000:
			tmp |= PCAP2_CODEC_RATE_16000;
			break;
		default:
			return -EINVAL;
		}
		tmp |= PCAP2_CODEC_RESET_DF;
		pcap2_codec_write(codec, PCAP2_CODEC, tmp);
	}

	return 0;
}

static int pcap2_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_dapm_widget *w;
	unsigned int tmp;

	if (codec_dai->id == PCAP2_STEREO_DAI) {
		snd_soc_dapm_disable_pin(codec, "ST_DAC");
		tmp = pcap2_codec_read(codec, PCAP2_ST_DAC);
		tmp &= ~(PCAP2_ST_DAC_EN | PCAP2_ST_DAC_CLK_EN);
		pcap2_codec_write(codec, PCAP2_ST_DAC, tmp);
	}
	else {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			snd_soc_dapm_disable_pin(codec, "CDC_DAC");
		else
			snd_soc_dapm_disable_pin(codec, "CDC_ADC");
		list_for_each_entry(w, &codec->dapm_widgets, list) {
			if ((!strcmp(w->name, "CDC_DAC") || !strcmp(w->name, "CDC_ADC")) && w->connected)
				goto in_use;
		}
		tmp = pcap2_codec_read(codec, PCAP2_CODEC);
		tmp &= ~(PCAP2_CODEC_EN | PCAP2_CODEC_CLK_EN);
		pcap2_codec_write(codec, PCAP2_CODEC, tmp);
	}
in_use:
	snd_soc_dapm_sync(codec);

	return 0;
}

static int pcap2_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	unsigned int tmp;
	if (codec_dai->id == PCAP2_STEREO_DAI) {
		/* ST_DAC */

		tmp = pcap2_codec_read(codec, PCAP2_ST_DAC);

		tmp &= ~PCAP2_ST_DAC_CLKSEL_MASK;
		switch (clk_id) {
		case PCAP2_CLK_AP:
			tmp |= PCAP2_ST_DAC_CLKSEL_AP;
			break;
		case PCAP2_CLK_BP:
			break;
		default:
			return -ENODEV;
		}

		tmp &= ~PCAP2_ST_DAC_CLK_MASK;
		switch (freq) {
		case 13000000:
			break;
/*		case 15M36:
			tmp |= PCAP2_ST_DAC_CLK_15M36;
			break;
		case 16M8:
			tmp |= PCAP2_ST_DAC_CLK_16M8;
			break;
		case 19M44:
			tmp |= PCAP2_ST_DAC_CLK_19M44;
			break;
*/		case 26000000:
			tmp |= PCAP2_ST_DAC_CLK_26M;
			break;
/*		case EXT_MCLK:
			tmp |= PCAP2_ST_DAC_CLK_MCLK;
			break;
		case FSYNC:
			tmp |= PCAP2_ST_DAC_CLK_FSYNC;
			break;
		case BITCLK:
			tmp |= PCAP2_ST_DAC_CLK_BITCLK;
			break;
*/		default:
			return -EINVAL;
		}
		pcap2_codec_write(codec, PCAP2_ST_DAC, tmp);
	}
	else {
		/* MONO_DAC */
		tmp = pcap2_codec_read(codec, PCAP2_CODEC);

		tmp &= ~PCAP2_CODEC_CLKSEL_MASK;
		switch (clk_id) {
		case PCAP2_CLK_AP:
			tmp |= PCAP2_CODEC_CLKSEL_AP;
			break;
		case PCAP2_CLK_BP:
			break;
		default:
			return -ENODEV;
		}

		tmp &= ~PCAP2_CODEC_CLK_MASK;
		switch (freq) {
		case 13000000:
			break;
/*		case 15M36:
			tmp |= PCAP2_CODEC_CLK_15M36;
			break;
		case 16M8:
			tmp |= PCAP2_CODEC_CLK_16M8;
			break;
		case 19M44:
			tmp |= PCAP2_CODEC_CLK_19M44;
			break;
*/		case 26000000:
			tmp |= PCAP2_CODEC_CLK_26M;
			break;
		default:
			return -EINVAL;
		}
		pcap2_codec_write(codec, PCAP2_CODEC, tmp);
	}
	return 0;
}

static int pcap2_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int tmp = 0;

	if (codec_dai->id == PCAP2_STEREO_DAI) {
		/* ST_DAC */

		/* disable CODEC */
		pcap2_codec_write(codec, PCAP2_CODEC, 0);

		switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			tmp |= 0x1;
			break;
		default:
			return -EINVAL;
		}

		switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			tmp |= 0x4000;
			break;
/*		case SND_SOC_NET:
			tmp |= 0x2000;
			break;
*/		case SND_SOC_DAIFMT_DSP_B:
			break;
		default:
			return -EINVAL;
		}

		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_IB_IF:
			break;
		case SND_SOC_DAIFMT_NB_NF:
			tmp |= 0x60000;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			tmp |= 0x40000;
			break;
		case SND_SOC_DAIFMT_NB_IF:
			tmp |= 0x20000;
			break;
		}
		/* set dai to AP */
		tmp |= 0x1000;

		/* set BCLK */
		tmp |= 0x18000;

		pcap2_codec_write(codec, PCAP2_ST_DAC, tmp);
	}
	else {
		/* MONO_DAC */

		/* disable ST_DAC */
		pcap2_codec_write(codec, PCAP2_ST_DAC, 0);

		switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			tmp |= 0x2;
			break;
		default:
			return -EINVAL;
		}

		switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_DSP_B:
			break;
		default:
			return -EINVAL;
		}

		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_IB_IF:
			break;
		case SND_SOC_DAIFMT_NB_NF:
			tmp |= 0x600;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			tmp |= 0x400;
			break;
		case SND_SOC_DAIFMT_NB_IF:
			tmp |= 0x200;
			break;
		}
		if (codec_dai->id == PCAP2_MONO_DAI)
			/* set dai to AP */
			tmp |= 0x8000;

		tmp |= 0x5; /* IHF / OHF */

		pcap2_codec_write(codec, PCAP2_CODEC, tmp);
	}
	return 0;
}

static int pcap2_prepare(struct snd_pcm_substream *substream)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int tmp;
	/* FIXME enable clock only if codec is master */
	if (codec_dai->id == PCAP2_STEREO_DAI) {
		snd_soc_dapm_enable_pin(codec, "ST_DAC");
		snd_soc_dapm_disable_pin(codec, "CDC_DAC");
		snd_soc_dapm_disable_pin(codec, "CDC_ADC");
		tmp = pcap2_codec_read(codec, PCAP2_ST_DAC);
		tmp |= (PCAP2_ST_DAC_EN | PCAP2_ST_DAC_CLK_EN);
		pcap2_codec_write(codec, PCAP2_ST_DAC, tmp);
	}
	else {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			snd_soc_dapm_enable_pin(codec, "CDC_DAC");
		else
			snd_soc_dapm_enable_pin(codec, "CDC_ADC");
		snd_soc_dapm_disable_pin(codec, "ST_DAC");
		tmp = pcap2_codec_read(codec, PCAP2_CODEC);
		tmp |= (PCAP2_CODEC_EN | PCAP2_CODEC_CLK_EN);
		pcap2_codec_write(codec, PCAP2_CODEC, tmp);
	}
	snd_soc_dapm_sync(codec);
	mdelay(1);
#ifdef PCAP2_DEBUG
	dump_registers();
#endif
	return 0;
}

/*
 * Define codec DAI.
 */
struct snd_soc_dai pcap2_dai[] = {
{
	.name = "PCAP2 MONO",
	.id = 0,
	.playback = {
		.stream_name = "CDC_DAC playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "CDC_DAC capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = {
		.prepare = pcap2_prepare,
		.hw_params = pcap2_hw_params,
		.hw_free = pcap2_hw_free,
	},
	.dai_ops = {
//		.digital_mute = pcap2_mute,
		.set_fmt = pcap2_set_dai_fmt,
		.set_sysclk = pcap2_set_dai_sysclk,
	},
},
{
	.name = "PCAP2 STEREO",
	.id = 1,
	.playback = {
		.stream_name = "ST_DAC playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = { /* FIXME: PCAP support this?? */
		.stream_name = "ST_DAC capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = {
		.prepare = pcap2_prepare,
		.hw_params = pcap2_hw_params,
		.hw_free = pcap2_hw_free,
	},
	.dai_ops = {
//		.digital_mute = pcap2_mute,
		.set_fmt = pcap2_set_dai_fmt,
		.set_sysclk = pcap2_set_dai_sysclk,
	},
},
{
	.name = "PCAP2 BP",
	.id = 2,
	.playback = {
		.stream_name = "BP playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = {
		.prepare = pcap2_prepare,
		.hw_params = pcap2_hw_params,
		.hw_free = pcap2_hw_free,
	},
	.dai_ops = {
//		.digital_mute = pcap2_mute,
		.set_fmt = pcap2_set_dai_fmt,
		.set_sysclk = pcap2_set_dai_sysclk,
	},
},
};
EXPORT_SYMBOL_GPL(pcap2_dai);

static int pcap2_codec_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	dbg("pcap2_codec_suspend");
	pcap2_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int pcap2_codec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	dbg("pcap2_codec_resume");
	pcap2_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	pcap2_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

/*
 * initialise the PCAP2 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int pcap2_codec_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	dbg("pcap2_codec_init");
	codec->name = "PCAP2 Audio";
	codec->owner = THIS_MODULE;
	codec->read = pcap2_codec_read;
	codec->write = pcap2_codec_write;
	codec->set_bias_level = pcap2_set_bias_level;
	codec->dai = pcap2_dai;
	codec->num_dai = ARRAY_SIZE(pcap2_dai);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		return ret;
	}
	/* power on device */
	pcap2_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	pcap2_codec_add_controls(codec);
	pcap2_codec_add_widgets(codec);
	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		snd_soc_free_pcms(socdev);
		snd_soc_dapm_free(socdev);
	}

	return ret;
}

static int pcap2_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct pcap2_codec_setup_data *setup;
	struct snd_soc_codec *codec;
	int ret = 0;
	info("PCAP2 Audio Codec %s", PCAP2_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	pcap2_codec_socdev = socdev;

	ret = pcap2_codec_init(socdev);
	return ret;
}

/* power down chip and remove */
static int pcap2_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	if (codec->control_data)
		pcap2_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	kfree(codec);

	return 0;
}

/* codec device ops */
struct snd_soc_codec_device soc_codec_dev_pcap2 = {
	.probe = 	pcap2_codec_probe,
	.remove = 	pcap2_codec_remove,
	.suspend = 	pcap2_codec_suspend,
	.resume =	pcap2_codec_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_pcap2);

MODULE_DESCRIPTION("ASoC PCAP2 codec");
MODULE_AUTHOR("Daniel Ribeiro");
MODULE_LICENSE("GPL");

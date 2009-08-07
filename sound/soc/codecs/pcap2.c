/*
 * pcap2.c - PCAP2 PMIC Audio driver
 *
 * 	Copyright (C) 2009 Daniel Ribeiro <drwyrm@gmail.com>
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
#include <sound/jack.h>
#include <linux/mfd/ezx-pcap.h>

#include "pcap2.h"

static int pcap2_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	struct pcap_chip *pcap = codec->dai->private_data;

	ezx_pcap_write(pcap, reg, value);

	return 0;
}

static unsigned int pcap2_codec_read(struct snd_soc_codec *codec,
							unsigned int reg)
{
	struct pcap_chip *pcap = codec->dai->private_data;
	unsigned int tmp;

	ezx_pcap_read(pcap, reg, &tmp);

	return tmp;
}

static const char *pcap2_downmix_select[] = {
	"Off",
	"2->1ch",
	"2->1ch -3db",
	"2->1ch -6db"
};

static const struct soc_enum pcap2_downmixer_enum[] = {
SOC_ENUM_SINGLE(PCAP2_OUTPUT_AMP, 19, 4, pcap2_downmix_select),
};

/* pcap2 codec non DAPM controls */
static const struct snd_kcontrol_new pcap2_codec_snd_controls[] = {
SOC_SINGLE("Master Playback Volume", PCAP2_OUTPUT_AMP, 13, 15, 0),
SOC_SINGLE("Capture Volume", PCAP2_INPUT_AMP, 0, 31, 0),
};

/* pcap2 codec DAPM controls */
static const struct snd_kcontrol_new pcap2_codec_dm_mux_control[] = {
SOC_DAPM_ENUM("Downmixer Mode",	pcap2_downmixer_enum[0]),
};

static const struct snd_kcontrol_new pcap2_input_mixer_controls[] = {
SOC_DAPM_SINGLE("A3 Switch", PCAP2_INPUT_AMP, 6, 1, 0),
SOC_DAPM_SINGLE("A4 Switch", PCAP2_OUTPUT_AMP, 10, 1, 0),
SOC_DAPM_SINGLE("A5 Switch", PCAP2_INPUT_AMP, 8, 1, 0),
};

static const struct snd_kcontrol_new pcap2_output_mixer_controls[] = {
SOC_DAPM_SINGLE("A1 Switch", PCAP2_OUTPUT_AMP, 0, 1, 0),
SOC_DAPM_SINGLE("A2 Switch", PCAP2_OUTPUT_AMP, 1, 1, 0),
SOC_DAPM_SINGLE("AR Switch", PCAP2_OUTPUT_AMP, 5, 1, 0),
SOC_DAPM_SINGLE("AL Switch", PCAP2_OUTPUT_AMP, 6, 1, 0),
};

static const struct snd_soc_dapm_widget pcap2_codec_dapm_widgets[] = {
SND_SOC_DAPM_DAC("ST_DAC", "ST_DAC playback", PCAP2_OUTPUT_AMP, 9, 0),
SND_SOC_DAPM_DAC("CDC_DAC", "MONO_DAC playback", PCAP2_OUTPUT_AMP, 8, 0),
SND_SOC_DAPM_ADC("CDC_ADC", "MONO_DAC capture", PCAP2_OUTPUT_AMP, 8, 0),
SND_SOC_DAPM_PGA("PGA_R", PCAP2_OUTPUT_AMP, 11, 0, NULL, 0),
SND_SOC_DAPM_PGA("PGA_L", PCAP2_OUTPUT_AMP, 12, 0, NULL, 0),
SND_SOC_DAPM_MUX("Downmixer", SND_SOC_NOPM, 0, 0,
					pcap2_codec_dm_mux_control),
SND_SOC_DAPM_PGA("PGA_A1CTRL", PCAP2_OUTPUT_AMP, 17, 1, NULL, 0),
SND_SOC_DAPM_MIXER("Output Mixer", SND_SOC_NOPM, 0, 0,
		pcap2_output_mixer_controls,
		ARRAY_SIZE(pcap2_output_mixer_controls)),
SND_SOC_DAPM_OUTPUT("A1"), /* Earpiece */
SND_SOC_DAPM_OUTPUT("A2"), /* Loudspeaker */
SND_SOC_DAPM_OUTPUT("AR"), /* Headphone Right */
SND_SOC_DAPM_OUTPUT("AL"), /* Headphone Left */

SND_SOC_DAPM_MICBIAS("BIAS1", PCAP2_INPUT_AMP, 10, 0),
SND_SOC_DAPM_MICBIAS("BIAS2", PCAP2_INPUT_AMP, 11, 0),
SND_SOC_DAPM_MIXER("Input Mixer", SND_SOC_NOPM, 0, 0,
		pcap2_input_mixer_controls,
		ARRAY_SIZE(pcap2_input_mixer_controls)),
SND_SOC_DAPM_INPUT("A3"), /* Headset Mic */
SND_SOC_DAPM_INPUT("A4"), /* FM Chip */
SND_SOC_DAPM_INPUT("A5"), /* Built-in Mic */
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
	{ "Downmixer", "Off", "PGA_R" },

	{ "PGA_R", NULL, "ST_DAC" },
	{ "PGA_L", NULL, "ST_DAC" },
	{ "PGA_R", NULL, "CDC_DAC" },

	/* input path */
	{ "BIAS1", NULL, "A3" },
	{ "BIAS2", NULL, "A5" },

	{ "Input Mixer", "A3 Switch", "BIAS1" },
	{ "Input Mixer", "A4 Switch", "A4" },
	{ "Input Mixer", "A5 Switch", "BIAS2" },

	{ "PGA_R", NULL, "Input Mixer" },
	{ "PGA_L", NULL, "Input Mixer" },

	{ "CDC_ADC", NULL, "PGA_R" },
};

static int pcap2_codec_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, pcap2_codec_dapm_widgets,
				ARRAY_SIZE(pcap2_codec_dapm_widgets));
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
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int st_dac, mono_dac;

	st_dac = pcap2_codec_read(codec, PCAP2_ST_DAC);
	mono_dac = pcap2_codec_read(codec, PCAP2_CODEC);

	if (st_dac & PCAP2_ST_DAC_EN || mono_dac & PCAP2_CODEC_EN)
		return -EBUSY;

	switch (codec_dai->id) {
	case PCAP2_ID_ST_DAC:
		if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
			return -EINVAL;

		st_dac &= ~PCAP2_ST_DAC_RATE_MASK;
		switch (params_rate(params)) {
		case 8000:
			break;
		case 11025:
			st_dac |= PCAP2_ST_DAC_RATE_11025;
			break;
		case 12000:
			st_dac |= PCAP2_ST_DAC_RATE_12000;
			break;
		case 16000:
			st_dac |= PCAP2_ST_DAC_RATE_16000;
			break;
		case 22050:
			st_dac |= PCAP2_ST_DAC_RATE_22050;
			break;
		case 24000:
			st_dac |= PCAP2_ST_DAC_RATE_24000;
			break;
		case 32000:
			st_dac |= PCAP2_ST_DAC_RATE_32000;
			break;
		case 44100:
			st_dac |= PCAP2_ST_DAC_RATE_44100;
			break;
		case 48000:
			st_dac |= PCAP2_ST_DAC_RATE_48000;
			break;
		default:
			return -EINVAL;
		}

		st_dac |= PCAP2_ST_DAC_RESET_DF;

		pcap2_codec_write(codec, PCAP2_ST_DAC, st_dac);
		break;
	case PCAP2_ID_MONO_DAC:
		mono_dac &= ~PCAP2_CODEC_RATE_MASK;
		switch (params_rate(params)) {
		case 8000:
			break;
		case 16000:
			mono_dac |= PCAP2_CODEC_RATE_16000;
			break;
		default:
			return -EINVAL;
		}

		mono_dac |= PCAP2_CODEC_RESET_DF;

		pcap2_codec_write(codec, PCAP2_CODEC, mono_dac);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pcap2_hw_free(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_dapm_widget *w;
	unsigned int tmp;

	switch (codec_dai->id) {
	case PCAP2_ID_ST_DAC:
		tmp = pcap2_codec_read(codec, PCAP2_ST_DAC);
		tmp &= ~(PCAP2_ST_DAC_EN | PCAP2_ST_DAC_CLK_EN);
		pcap2_codec_write(codec, PCAP2_ST_DAC, tmp);
		break;
	case PCAP2_ID_MONO_DAC:
		list_for_each_entry(w, &codec->dapm_widgets, list) {
			if ((!strcmp(w->name, "CDC_DAC") ||
				!strcmp(w->name, "CDC_ADC")) && w->connected)
				goto in_use;
		}
		tmp = pcap2_codec_read(codec, PCAP2_CODEC);
		tmp &= ~(PCAP2_CODEC_EN | PCAP2_CODEC_CLK_EN);
		pcap2_codec_write(codec, PCAP2_CODEC, tmp);
		break;
	default:
		return -EINVAL;
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

	switch (codec_dai->id) {
	case PCAP2_ID_ST_DAC:
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
		case 26000000:
			tmp |= PCAP2_ST_DAC_CLK_26M;
			break;
		default:
			return -EINVAL;
		}
		pcap2_codec_write(codec, PCAP2_ST_DAC, tmp);
		break;
	case PCAP2_ID_MONO_DAC:
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
		case 26000000:
			tmp |= PCAP2_CODEC_CLK_26M;
			break;
		default:
			return -EINVAL;
		}
		pcap2_codec_write(codec, PCAP2_CODEC, tmp);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int pcap2_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int st_dac, mono_dac;

	st_dac = pcap2_codec_read(codec, PCAP2_ST_DAC);
	mono_dac = pcap2_codec_read(codec, PCAP2_CODEC);

	if (st_dac & PCAP2_ST_DAC_EN || mono_dac & PCAP2_CODEC_EN)
		return -EBUSY;

	/* reset both dacs */
	st_dac = mono_dac = 0;
	switch (codec_dai->id) {
	case PCAP2_ID_ST_DAC:
		switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			st_dac |= PCAP2_ST_DAC_SLAVE;
			break;
		default:
			return -EINVAL;
		}

		switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			st_dac |= PCAP2_ST_DAC_DAI_I2S;
			break;
		case SND_SOC_DAIFMT_DSP_B:
			break;
		default:
			return -EINVAL;
		}

		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_IB_IF:
			break;
		case SND_SOC_DAIFMT_NB_NF:
			st_dac |= PCAP2_ST_DAC_BCLK_INV |
				PCAP2_ST_DAC_FRAME_INV;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			st_dac |= PCAP2_ST_DAC_FRAME_INV;
			break;
		case SND_SOC_DAIFMT_NB_IF:
			st_dac |= PCAP2_ST_DAC_BCLK_INV;
			break;
		}
		/* FIXME set dai to AP */
		st_dac |= 0x1000;

		/* FIXME set BCLK */
		st_dac |= 0x18000;
		break;
	case PCAP2_ID_MONO_DAC:
		switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			mono_dac |= PCAP2_CODEC_SLAVE;
			break;
		default:
			return -EINVAL;
		}

		switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_DSP_A:
			break;
		default:
			return -EINVAL;
		}

		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_IB_IF:
			break;
		case SND_SOC_DAIFMT_NB_NF:
			mono_dac |= PCAP2_CODEC_FRAME_INV |
				PCAP2_CODEC_BCLK_INV;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			mono_dac |= PCAP2_CODEC_FRAME_INV;
			break;
		case SND_SOC_DAIFMT_NB_IF:
			mono_dac |= PCAP2_CODEC_BCLK_INV;
			break;
		}
//		if (pcap2_dai_mode == DAI_AP_MONO)
			/* FIXME set dai to AP */
		mono_dac |= 0x8000;

		mono_dac |= 0x5; /* IHF / OHF */
		break;
	default:
		return -EINVAL;
	}

	pcap2_codec_write(codec, PCAP2_ST_DAC, st_dac);
	pcap2_codec_write(codec, PCAP2_CODEC, mono_dac);

	return 0;
}

static int pcap2_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int st_dac, mono_dac;

	st_dac = pcap2_codec_read(codec, PCAP2_ST_DAC);
	mono_dac = pcap2_codec_read(codec, PCAP2_CODEC);

	if (st_dac & PCAP2_ST_DAC_EN || mono_dac & PCAP2_CODEC_EN)
		return -EBUSY;

	switch (codec_dai->id) {
	case PCAP2_ID_ST_DAC:
		st_dac |= PCAP2_ST_DAC_EN;
		if (!(st_dac & PCAP2_ST_DAC_SLAVE))
			st_dac |= PCAP2_ST_DAC_CLK_EN;
		pcap2_codec_write(codec, PCAP2_ST_DAC, st_dac);
		break;
	case PCAP2_ID_MONO_DAC:
		mono_dac |= PCAP2_CODEC_EN;
		if (!(mono_dac & PCAP2_CODEC_SLAVE))
			mono_dac |= PCAP2_CODEC_CLK_EN;
		pcap2_codec_write(codec, PCAP2_CODEC, mono_dac);
		break;
	default:
		return -EINVAL;
	}
	snd_soc_dapm_sync(codec);
#ifdef PCAP2_DEBUG
	dump_registers();
#endif
	return 0;
}

/*
 * Define codec DAI.
 */

static struct snd_soc_dai_ops pcap2_dai_ops = {
	.prepare = pcap2_prepare,
	.hw_params = pcap2_hw_params,
	.hw_free = pcap2_hw_free,
	/* .digital_mute = pcap2_mute, */
	.set_fmt = pcap2_set_dai_fmt,
	.set_sysclk = pcap2_set_dai_sysclk,
};

struct snd_soc_dai pcap2_dai[] = {
	{
		.name = "PCAP2 ST_DAC",
		.id = PCAP2_ID_ST_DAC,
		.playback = {
			.stream_name = "ST_DAC playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &pcap2_dai_ops,
	}, {
		.name = "PCAP2 MONO_DAC",
		.id = PCAP2_ID_MONO_DAC,
		.playback = {
			.stream_name = "MONO_DAC playback",
			.channels_min = 1,
			.channels_max = 1,
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "MONO_DAC capture",
			.channels_min = 1,
			.channels_max = 1,
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &pcap2_dai_ops,
	},
};
EXPORT_SYMBOL_GPL(pcap2_dai);

static int pcap2_codec_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	pcap2_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int pcap2_codec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	pcap2_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	pcap2_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}


/* headset jack setup */
struct pcap_jack {
	struct pcap_chip *pcap;
	struct snd_soc_jack *jack;
};

static struct snd_soc_jack_pin pcap2_jack_pins[] = {
	{
		.pin = "External Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headset",
		.mask = SND_JACK_HEADPHONE,
	},
};

static irqreturn_t pcap2_hs_jack_irq(int irq, void *data)
{
	struct pcap_jack *pcap_jack = data;
	struct snd_soc_jack *jack = pcap_jack->jack;
	int pirq = irq_to_pcap(pcap_jack->pcap, irq);
	u32 pstat;

	ezx_pcap_read(pcap_jack->pcap, PCAP_REG_PSTAT, &pstat);
	pstat &= 1 << pirq;

	printk(KERN_DEBUG "IRQ HS.\n");
	if (pstat)
		snd_soc_jack_report(jack, SND_JACK_HEADSET, SND_JACK_HEADSET);
	else
		snd_soc_jack_report(jack, 0, SND_JACK_HEADSET);

	return IRQ_HANDLED;
}

static int pcap2_jack_init(struct snd_soc_device *socdev)
{
	int err = -ENOMEM;
	struct snd_soc_card *card = socdev->card;
	struct pcap_chip *pcap = socdev->card->codec->dai->private_data;
	struct snd_soc_jack *jack;
	struct pcap_jack *pcap_jack;

	pcap_jack = kmalloc(sizeof(*pcap_jack), GFP_KERNEL);
	if (!pcap_jack)
		goto out;

	pcap_jack->pcap = pcap;

	jack = kmalloc(sizeof(*jack), GFP_KERNEL);
	if (!jack)
		goto fail_jack_alloc;

	pcap_jack->jack = jack;

	err = snd_soc_jack_new(card, "Headset", SND_JACK_HEADSET, jack);
	if (err)
		goto fail;

	err = snd_soc_jack_add_pins(jack, ARRAY_SIZE(pcap2_jack_pins),
				pcap2_jack_pins);
	if (err)
		goto fail;


	err = request_irq(pcap_to_irq(pcap, PCAP_IRQ_HS), pcap2_hs_jack_irq, 0,
			"Headset jack", pcap_jack);
	if (err)
		goto fail;

	return 0;

fail:
	kfree(jack);
fail_jack_alloc:
	kfree(pcap_jack);
out:
	return err;
}

/*
 * initialise the PCAP2 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int pcap2_codec_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret = 0;

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

	snd_soc_add_controls(codec, pcap2_codec_snd_controls,
			ARRAY_SIZE(pcap2_codec_snd_controls));
	pcap2_codec_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		snd_soc_free_pcms(socdev);
		snd_soc_dapm_free(socdev);
	}

/*	pcap2_dai_mode = DAI_AP_ST;
	pcap2_set_dai_mode(codec, pcap2_dai_mode); */


	ret = pcap2_jack_init(socdev);

	// FIXME, here just to make sure snd_jack_dev_register() gets called
	snd_device_register_all(socdev->card->codec->card);


	return ret;
}

static int pcap2_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct pcap2_codec_setup_data *setup;
	struct snd_soc_codec *codec;
	int ret = 0;

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->card->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	ret = pcap2_codec_init(socdev);
	return ret;
}

/* power down chip and remove */
static int pcap2_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	if (codec->control_data)
		pcap2_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	kfree(codec);

	return 0;
}

static int pcap2_driver_probe(struct platform_device *pdev)
{
	pcap2_dai[0].private_data = dev_get_drvdata(pdev->dev.parent);

	return snd_soc_register_dai(&pcap2_dai[0]);
}

static int __devexit pcap2_driver_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pcap2_dai[0]);
	return 0;
}

/* codec device ops */
struct snd_soc_codec_device soc_codec_dev_pcap2 = {
	.probe =	pcap2_codec_probe,
	.remove =	pcap2_codec_remove,
	.suspend =	pcap2_codec_suspend,
	.resume =	pcap2_codec_resume,
};

static struct platform_driver pcap2_driver = {
	.probe		= pcap2_driver_probe,
	.remove		= __devexit_p(pcap2_driver_remove),
	.driver		= {
		.name		= "pcap-audio",
		.owner		= THIS_MODULE,
	},
};


static int __devinit pcap2_init(void)
{
	return platform_driver_register(&pcap2_driver);
}

static void __exit pcap2_exit(void)
{
	platform_driver_unregister(&pcap2_driver);
}

module_init(pcap2_init);
module_exit(pcap2_exit);

EXPORT_SYMBOL_GPL(soc_codec_dev_pcap2);

MODULE_DESCRIPTION("ASoC PCAP2 codec");
MODULE_AUTHOR("Daniel Ribeiro");
MODULE_LICENSE("GPL");

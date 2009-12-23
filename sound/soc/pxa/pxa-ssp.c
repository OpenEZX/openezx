/*
 * pxa-ssp.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2005,2008 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 * TODO:
 *  o Test network mode for > 16bit sample size
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/irq.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/pxa2xx-lib.h>

#include <mach/hardware.h>
#include <mach/dma.h>
#include <mach/regs-ssp.h>
#include <mach/audio.h>
#include <mach/ssp.h>

#include "pxa2xx-pcm.h"
#include "pxa-ssp.h"

/*
 * SSP audio private data
 */
struct ssp_priv {
	struct ssp_dev dev;
	unsigned int sysclk;
	int dai_fmt;
#ifdef CONFIG_PM
	struct ssp_state state;
#endif
};

static void dump_registers(struct ssp_device *ssp)
{
	dev_dbg(&ssp->pdev->dev, "SSCR0 0x%08x SSCR1 0x%08x SSTO 0x%08x\n",
		 ssp_read_reg(ssp, SSCR0), ssp_read_reg(ssp, SSCR1),
		 ssp_read_reg(ssp, SSTO));

	dev_dbg(&ssp->pdev->dev, "SSPSP 0x%08x SSSR 0x%08x SSACD 0x%08x\n",
		 ssp_read_reg(ssp, SSPSP), ssp_read_reg(ssp, SSSR),
		 ssp_read_reg(ssp, SSACD));
}

struct pxa2xx_pcm_dma_data {
	struct pxa2xx_pcm_dma_params params;
	char name[20];
};

static struct pxa2xx_pcm_dma_params *
ssp_get_dma_params(struct ssp_device *ssp, int width4, int out)
{
	struct pxa2xx_pcm_dma_data *dma;

	dma = kzalloc(sizeof(struct pxa2xx_pcm_dma_data), GFP_KERNEL);
	if (dma == NULL)
		return NULL;

	snprintf(dma->name, 20, "SSP%d PCM %s %s", ssp->port_id,
			width4 ? "32-bit" : "16-bit", out ? "out" : "in");

	dma->params.name = dma->name;
	dma->params.drcmr = &DRCMR(out ? ssp->drcmr_tx : ssp->drcmr_rx);
	dma->params.dcmd = (out ? (DCMD_INCSRCADDR | DCMD_FLOWTRG) :
				  (DCMD_INCTRGADDR | DCMD_FLOWSRC)) |
			(width4 ? DCMD_WIDTH4 : DCMD_WIDTH2) | DCMD_BURST16;
	dma->params.dev_addr = ssp->phys_base + SSDR;

	return &dma->params;
}

static int pxa_ssp_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	int ret = 0;

	if (!cpu_dai->active) {
		priv->dev.port = cpu_dai->id + 1;
		priv->dev.irq = NO_IRQ;
		clk_enable(priv->dev.ssp->clk);
		ssp_disable(&priv->dev);
	}

	if (cpu_dai->dma_data) {
		kfree(cpu_dai->dma_data);
		cpu_dai->dma_data = NULL;
	}
	return ret;
}

static void pxa_ssp_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;

	if (!cpu_dai->active) {
		ssp_disable(&priv->dev);
		clk_disable(priv->dev.ssp->clk);
	}

	if (cpu_dai->dma_data) {
		kfree(cpu_dai->dma_data);
		cpu_dai->dma_data = NULL;
	}
}

#ifdef CONFIG_PM

static int pxa_ssp_suspend(struct snd_soc_dai *cpu_dai)
{
	struct ssp_priv *priv = cpu_dai->private_data;

	if (!cpu_dai->active)
		return 0;

	ssp_save_state(&priv->dev, &priv->state);
	clk_disable(priv->dev.ssp->clk);
	return 0;
}

static int pxa_ssp_resume(struct snd_soc_dai *cpu_dai)
{
	struct ssp_priv *priv = cpu_dai->private_data;

	if (!cpu_dai->active)
		return 0;

	clk_enable(priv->dev.ssp->clk);
	ssp_restore_state(&priv->dev, &priv->state);
	ssp_enable(&priv->dev);

	return 0;
}

#else
#define pxa_ssp_suspend	NULL
#define pxa_ssp_resume	NULL
#endif

/**
 * ssp_set_clkdiv - set SSP clock divider
 * @div: serial clock rate divider
 */
static void ssp_set_scr(struct ssp_device *ssp, u32 div)
{
	u32 sscr0 = ssp_read_reg(ssp, SSCR0);

	if (cpu_is_pxa25x() && ssp->type == PXA25x_SSP) {
		sscr0 &= ~0x0000ff00;
		sscr0 |= ((div - 2)/2) << 8; /* 2..512 */
	} else {
		sscr0 &= ~0x000fff00;
		sscr0 |= (div - 1) << 8;     /* 1..4096 */
	}
	ssp_write_reg(ssp, SSCR0, sscr0);
}

/*
 * Set the SSP ports SYSCLK.
 */
static int pxa_ssp_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->dev.ssp;
	int val;

	u32 sscr0 = ssp_read_reg(ssp, SSCR0) &
		~(SSCR0_ECS |  SSCR0_NCS | SSCR0_MOD | SSCR0_ACS);

	dev_dbg(&ssp->pdev->dev,
		"pxa_ssp_set_dai_sysclk id: %d, clk_id %d, freq %u\n",
		cpu_dai->id, clk_id, freq);

	switch (clk_id) {
	case PXA_SSP_CLK_NET_PLL:
		sscr0 |= SSCR0_MOD;
		break;
	case PXA_SSP_CLK_PLL:
		/* Internal PLL is fixed */
		if (cpu_is_pxa25x())
			priv->sysclk = 1843200;
		else
			priv->sysclk = 13000000;
		break;
	case PXA_SSP_CLK_EXT:
		priv->sysclk = freq;
		sscr0 |= SSCR0_ECS;
		break;
	case PXA_SSP_CLK_NET:
		priv->sysclk = freq;
		sscr0 |= SSCR0_NCS | SSCR0_MOD;
		break;
	case PXA_SSP_CLK_AUDIO:
		priv->sysclk = 0;
		ssp_set_scr(ssp, 1);
		sscr0 |= SSCR0_ACS;
		break;
	default:
		return -ENODEV;
	}

	/* The SSP clock must be disabled when changing SSP clock mode
	 * on PXA2xx.  On PXA3xx it must be enabled when doing so. */
	if (!cpu_is_pxa3xx())
		clk_disable(priv->dev.ssp->clk);
	val = ssp_read_reg(ssp, SSCR0) | sscr0;
	ssp_write_reg(ssp, SSCR0, val);
	if (!cpu_is_pxa3xx())
		clk_enable(priv->dev.ssp->clk);

	return 0;
}

/*
 * Set the SSP clock dividers.
 */
static int pxa_ssp_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
	int div_id, int div)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->dev.ssp;
	int val;

	switch (div_id) {
	case PXA_SSP_AUDIO_DIV_ACDS:
		val = (ssp_read_reg(ssp, SSACD) & ~0x7) | SSACD_ACDS(div);
		ssp_write_reg(ssp, SSACD, val);
		break;
	case PXA_SSP_AUDIO_DIV_SCDB:
		val = ssp_read_reg(ssp, SSACD);
		val &= ~SSACD_SCDB;
#if defined(CONFIG_PXA3xx)
		if (cpu_is_pxa3xx())
			val &= ~SSACD_SCDX8;
#endif
		switch (div) {
		case PXA_SSP_CLK_SCDB_1:
			val |= SSACD_SCDB;
			break;
		case PXA_SSP_CLK_SCDB_4:
			break;
#if defined(CONFIG_PXA3xx)
		case PXA_SSP_CLK_SCDB_8:
			if (cpu_is_pxa3xx())
				val |= SSACD_SCDX8;
			else
				return -EINVAL;
			break;
#endif
		default:
			return -EINVAL;
		}
		ssp_write_reg(ssp, SSACD, val);
		break;
	case PXA_SSP_DIV_SCR:
		ssp_set_scr(ssp, div);
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

/*
 * Configure the PLL frequency pxa27x and (afaik - pxa320 only)
 */
static int pxa_ssp_set_dai_pll(struct snd_soc_dai *cpu_dai, int pll_id,
	int source, unsigned int freq_in, unsigned int freq_out)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->dev.ssp;
	u32 ssacd = ssp_read_reg(ssp, SSACD) & ~0x70;

#if defined(CONFIG_PXA3xx)
	if (cpu_is_pxa3xx())
		ssp_write_reg(ssp, SSACDD, 0);
#endif

	switch (freq_out) {
	case 5622000:
		break;
	case 11345000:
		ssacd |= (0x1 << 4);
		break;
	case 12235000:
		ssacd |= (0x2 << 4);
		break;
	case 14857000:
		ssacd |= (0x3 << 4);
		break;
	case 32842000:
		ssacd |= (0x4 << 4);
		break;
	case 48000000:
		ssacd |= (0x5 << 4);
		break;
	case 0:
		/* Disable */
		break;

	default:
#ifdef CONFIG_PXA3xx
		/* PXA3xx has a clock ditherer which can be used to generate
		 * a wider range of frequencies - calculate a value for it.
		 */
		if (cpu_is_pxa3xx()) {
			u32 val;
			u64 tmp = 19968;
			tmp *= 1000000;
			do_div(tmp, freq_out);
			val = tmp;

			val = (val << 16) | 64;
			ssp_write_reg(ssp, SSACDD, val);

			ssacd |= (0x6 << 4);

			dev_dbg(&ssp->pdev->dev,
				"Using SSACDD %x to supply %uHz\n",
				val, freq_out);
			break;
		}
#endif

		return -EINVAL;
	}

	ssp_write_reg(ssp, SSACD, ssacd);

	return 0;
}

/*
 * Set the active slots in TDM/Network mode
 */
static int pxa_ssp_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->dev.ssp;
	u32 sscr0;

	sscr0 = ssp_read_reg(ssp, SSCR0);
	sscr0 &= ~(SSCR0_MOD | SSCR0_SlotsPerFrm(8) | SSCR0_EDSS | SSCR0_DSS);

	/* set slot width */
	if (slot_width > 16)
		sscr0 |= SSCR0_EDSS | SSCR0_DataSize(slot_width - 16);
	else
		sscr0 |= SSCR0_DataSize(slot_width);

	if (slots > 1) {
		/* enable network mode */
		sscr0 |= SSCR0_MOD;

		/* set number of active slots */
		sscr0 |= SSCR0_SlotsPerFrm(slots);

		/* set active slot mask */
		ssp_write_reg(ssp, SSTSA, tx_mask);
		ssp_write_reg(ssp, SSRSA, rx_mask);
	}
	ssp_write_reg(ssp, SSCR0, sscr0);

	return 0;
}

/*
 * Tristate the SSP DAI lines
 */
static int pxa_ssp_set_dai_tristate(struct snd_soc_dai *cpu_dai,
	int tristate)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->dev.ssp;
	u32 sscr1;

	sscr1 = ssp_read_reg(ssp, SSCR1);
	if (tristate)
		sscr1 &= ~SSCR1_TTE;
	else
		sscr1 |= SSCR1_TTE;
	ssp_write_reg(ssp, SSCR1, sscr1);

	return 0;
}

/*
 * Set up the SSP DAI format.
 * The SSP Port must be inactive before calling this function as the
 * physical interface format is changed.
 */
static int pxa_ssp_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->dev.ssp;
	u32 sscr0;
	u32 sscr1;
	u32 sspsp;

	/* check if we need to change anything at all */
	if (priv->dai_fmt == fmt)
		return 0;

	/* we can only change the settings if the port is not in use */
	if (ssp_read_reg(ssp, SSCR0) & SSCR0_SSE) {
		dev_err(&ssp->pdev->dev,
			"can't change hardware dai format: stream is in use");
		return -EINVAL;
	}

	/* reset port settings */
	sscr0 = ssp_read_reg(ssp, SSCR0) & (SSCR0_ECS | SSCR0_NCS | SSCR0_MOD |
			SSCR0_ACS | SSCR0_DSS | SSCR0_EDSS);
	sscr1 = SSCR1_RxTresh(8) | SSCR1_TxTresh(7);
	sspsp = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		sscr1 |= SSCR1_SCLKDIR | SSCR1_SFRMDIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		sscr1 |= SSCR1_SCLKDIR;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		sspsp |= SSPSP_SFRMP;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		sspsp |= SSPSP_SCMODE(2);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		sspsp |= SSPSP_SCMODE(2) | SSPSP_SFRMP;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		sspsp |= SSPSP_FSRT;
	case SND_SOC_DAIFMT_DSP_B:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_I2S:
		sscr0 |= SSCR0_PSP;
		sscr1 |= SSCR1_TRAIL | SSCR1_RWOT;
		/* See hw_params() for I2S and LEFT_J */
		break;

	default:
		return -EINVAL;
	}

	ssp_write_reg(ssp, SSCR0, sscr0);
	ssp_write_reg(ssp, SSCR1, sscr1);
	ssp_write_reg(ssp, SSPSP, sspsp);

	dump_registers(ssp);

	/* Since we are configuring the timings for the format by hand
	 * we have to defer some things until hw_params() where we
	 * know parameters like the sample size.
	 */
	priv->dai_fmt = fmt;

	return 0;
}

/*
 * Set the SSP audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int pxa_ssp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->dev.ssp;
	int chn = params_channels(params);
	u32 sscr0, sscr1, sspsp;
	int width = snd_pcm_format_physical_width(params_format(params));
	int slot_width, frame_width = 0;

	/* check if the user explicitly set a slot_width */
	sscr0 = ssp_read_reg(ssp, SSCR0);

	if (sscr0 & (SSCR0_EDSS | SSCR0_DSS))
		slot_width = (sscr0 & SSCR0_DSS) +
			(sscr0 & SSCR0_EDSS ? 17 : 1);
	else
		frame_width = slot_width = width * chn;

	/* generate correct DMA params */
	if (cpu_dai->dma_data)
		kfree(cpu_dai->dma_data);

	cpu_dai->dma_data = ssp_get_dma_params(ssp, slot_width > 16,
			substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	/* we can only change the settings if the port is not in use */
	if (sscr0 & SSCR0_SSE)
		return 0;

#ifdef CONFIG_PXA3xx
	if (slot_width == 16 && cpu_is_pxa3xx())
		sscr0 |= SSCR0_FPCKE;
#endif

	sspsp = ssp_read_reg(ssp, SSPSP);
	switch (priv->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/*
		 * I2S and LEFT_J are stereo only, we have to send data for
		 * both channels.
		 */
		if (chn == 1)
			frame_width *= 2;

		/*
		 * If the user did not use network mode, we assume the codec
		 * is I2S compliant.
		 */
		if (frame_width > 0) {
			sspsp |= SSPSP_SFRMWDTH(frame_width / 2);
			sspsp |= SSPSP_FSRT;
		} else {
			/*
			 * Otherwise we assume that it is a single TDM slot, and
			 * the user is abusing set_tdm_slot to support an
			 * out of spec codec.
			 */
			int slots = ((sscr0 & SSCR0_SlotsPerFrm(8)) >> 24) + 1;

			if (slots == 1 && slot_width == 16) {
				if (!cpu_is_pxa3xx())
					return -EINVAL;

				sspsp |= SSPSP_DMYSTRT(1);
				sspsp |= SSPSP_DMYSTOP(
						slot_width * 2 - width - 1);
				sspsp |= SSPSP_SFRMWDTH(slot_width * 2);
			} else {
				sspsp |= SSPSP_SFRMWDTH(slot_width * slots / 2);
				sspsp |= SSPSP_FSRT;
			}
		}
		break;

	case SND_SOC_DAIFMT_LEFT_J:
		if (chn == 1)
			frame_width *= 2;

		if (frame_width > 0) {
			sspsp |= SSPSP_SFRMWDTH(frame_width / 2);
		} else {
			int slots = ((sscr0 & SSCR0_SlotsPerFrm(8)) >> 24) + 1;

			sspsp |= SSPSP_SFRMWDTH((slot_width * slots) / 2);
		}
		break;
	default:
		break;
	}
	ssp_write_reg(ssp, SSPSP, sspsp);

	if (frame_width > 0) {
		/* Not using network mode */
		if (frame_width > 16)
			sscr0 |= SSCR0_EDSS | SSCR0_DataSize(frame_width - 16);
		else
			sscr0 |= SSCR0_DataSize(frame_width);

		if (frame_width > 32) {
			/*
			 * Network mode is needed to support this frame width.
			 * We assume the wire is not networked and setup a
			 * fake network mode here. Use as many slots as needed
			 * each with 32 bits.
			 */
			int slots = frame_width / 32;

			sscr0 |= SSCR0_MOD;
			sscr0 |= SSCR0_SlotsPerFrm(slots);

			/*
			 * Set active slots. Only set an active TX slot
			 * if we are going to use it.
			 */
			ssp_write_reg(ssp, SSRSA, slots - 1);
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				ssp_write_reg(ssp, SSTSA, slots - 1);
		}
	}

	/* If SSCR0_MOD is set we can't use SSCR1_RWOT */
	if (sscr0 & SSCR0_MOD) {
		sscr1 = ssp_read_reg(ssp, SSCR1);
		ssp_write_reg(ssp, SSCR1, sscr1 & ~SSCR1_RWOT);
	}

	ssp_write_reg(ssp, SSCR0, sscr0);

	dump_registers(ssp);

	return 0;
}

static int pxa_ssp_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->dev.ssp;
	int val;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
		ssp_enable(&priv->dev);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		val = ssp_read_reg(ssp, SSCR1);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val |= SSCR1_TSRE;
		else
			val |= SSCR1_RSRE;
		ssp_write_reg(ssp, SSCR1, val);
		val = ssp_read_reg(ssp, SSSR);
		ssp_write_reg(ssp, SSSR, val);
		break;
	case SNDRV_PCM_TRIGGER_START:
		val = ssp_read_reg(ssp, SSCR1);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val |= SSCR1_TSRE;
		else
			val |= SSCR1_RSRE;
		ssp_write_reg(ssp, SSCR1, val);
		ssp_enable(&priv->dev);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		val = ssp_read_reg(ssp, SSCR1);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val &= ~SSCR1_TSRE;
		else
			val &= ~SSCR1_RSRE;
		ssp_write_reg(ssp, SSCR1, val);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		ssp_disable(&priv->dev);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		val = ssp_read_reg(ssp, SSCR1);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val &= ~SSCR1_TSRE;
		else
			val &= ~SSCR1_RSRE;
		ssp_write_reg(ssp, SSCR1, val);
		break;

	default:
		ret = -EINVAL;
	}

	dump_registers(ssp);

	return ret;
}

static int pxa_ssp_probe(struct platform_device *pdev,
			    struct snd_soc_dai *dai)
{
	struct ssp_priv *priv;
	int ret;

	priv = kzalloc(sizeof(struct ssp_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev.ssp = ssp_request(dai->id + 1, "SoC audio");
	if (priv->dev.ssp == NULL) {
		ret = -ENODEV;
		goto err_priv;
	}

	priv->dai_fmt = (unsigned int) -1;
	dai->private_data = priv;

	return 0;

err_priv:
	kfree(priv);
	return ret;
}

static void pxa_ssp_remove(struct platform_device *pdev,
			      struct snd_soc_dai *dai)
{
	struct ssp_priv *priv = dai->private_data;
	ssp_free(priv->dev.ssp);
}

#define PXA_SSP_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
			  SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |	\
			  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |	\
			  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define PXA_SSP_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			    SNDRV_PCM_FMTBIT_S24_LE |	\
			    SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops pxa_ssp_dai_ops = {
	.startup	= pxa_ssp_startup,
	.shutdown	= pxa_ssp_shutdown,
	.trigger	= pxa_ssp_trigger,
	.hw_params	= pxa_ssp_hw_params,
	.set_sysclk	= pxa_ssp_set_dai_sysclk,
	.set_clkdiv	= pxa_ssp_set_dai_clkdiv,
	.set_pll	= pxa_ssp_set_dai_pll,
	.set_fmt	= pxa_ssp_set_dai_fmt,
	.set_tdm_slot	= pxa_ssp_set_dai_tdm_slot,
	.set_tristate	= pxa_ssp_set_dai_tristate,
};

struct snd_soc_dai pxa_ssp_dai[] = {
	{
		.name = "pxa2xx-ssp1",
		.id = 0,
		.probe = pxa_ssp_probe,
		.remove = pxa_ssp_remove,
		.suspend = pxa_ssp_suspend,
		.resume = pxa_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		},
		.capture = {
			 .channels_min = 1,
			 .channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		 },
		.ops = &pxa_ssp_dai_ops,
	},
	{	.name = "pxa2xx-ssp2",
		.id = 1,
		.probe = pxa_ssp_probe,
		.remove = pxa_ssp_remove,
		.suspend = pxa_ssp_suspend,
		.resume = pxa_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		 },
		.ops = &pxa_ssp_dai_ops,
	},
	{
		.name = "pxa2xx-ssp3",
		.id = 2,
		.probe = pxa_ssp_probe,
		.remove = pxa_ssp_remove,
		.suspend = pxa_ssp_suspend,
		.resume = pxa_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		 },
		.ops = &pxa_ssp_dai_ops,
	},
	{
		.name = "pxa2xx-ssp4",
		.id = 3,
		.probe = pxa_ssp_probe,
		.remove = pxa_ssp_remove,
		.suspend = pxa_ssp_suspend,
		.resume = pxa_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = PXA_SSP_RATES,
			.formats = PXA_SSP_FORMATS,
		 },
		.ops = &pxa_ssp_dai_ops,
	},
};
EXPORT_SYMBOL_GPL(pxa_ssp_dai);

static int __init pxa_ssp_init(void)
{
	return snd_soc_register_dais(pxa_ssp_dai, ARRAY_SIZE(pxa_ssp_dai));
}
module_init(pxa_ssp_init);

static void __exit pxa_ssp_exit(void)
{
	snd_soc_unregister_dais(pxa_ssp_dai, ARRAY_SIZE(pxa_ssp_dai));
}
module_exit(pxa_ssp_exit);

/* Module information */
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_DESCRIPTION("PXA SSP/PCM SoC Interface");
MODULE_LICENSE("GPL");

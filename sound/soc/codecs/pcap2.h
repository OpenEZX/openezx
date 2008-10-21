/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PCAP2_H
#define _PCAP2_H

/* 16 bit reads/writes on pcap registers (ugly workaround) */
#define SL (1 << 5)	/* lower 16 bits */
#define SM (1 << 6)	/* mid 16 bits */
#define SH (1 << 7)	/* higher 16 bits */

/* PCAP2 register space */
#define PCAP2_CODEC			0x0b
#define PCAP2_OUTPUT_AMP		0x0c
#define PCAP2_ST_DAC			0x0d
#define PCAP2_INPUT_AMP			0x1a

#define PCAP2_MONO_DAI			0
#define PCAP2_STEREO_DAI		1
#define PCAP2_BP_DAI			2

#define PCAP2_CLK_BP			0
#define PCAP2_CLK_AP			1

#define PCAP2_CODEC_EN			0x2000
#define PCAP2_CODEC_CLK_EN		0x1000
#define PCAP2_CODEC_RESET_DF		0x800
#define PCAP2_CODEC_RATE_MASK		0x4000
#define PCAP2_CODEC_RATE_8000		0x0
#define PCAP2_CODEC_RATE_16000		0x4000
#define PCAP2_CODEC_CLKSEL_MASK		0x10000
#define PCAP2_CODEC_CLKSEL_AP		0x10000
#define PCAP2_CODEC_CLKSEL_BP		0x0
#define PCAP2_CODEC_CLK_MASK		0x1c0
#define PCAP2_CODEC_CLK_13M		0x0
#define PCAP2_CODEC_CLK_15M36		0x40
#define PCAP2_CODEC_CLK_16M8		0x80
#define PCAP2_CODEC_CLK_19M44		0xc0
#define PCAP2_CODEC_CLK_26M		0x100

#define PCAP2_ST_DAC_EN			0x80
#define PCAP2_ST_DAC_CLK_EN		0x20
#define PCAP2_ST_DAC_RESET_DF		0x40
#define PCAP2_ST_DAC_RATE_MASK		0xf00
#define PCAP2_ST_DAC_RATE_8000		0x0
#define PCAP2_ST_DAC_RATE_11025		0x100
#define PCAP2_ST_DAC_RATE_12000		0x200
#define PCAP2_ST_DAC_RATE_16000		0x300
#define PCAP2_ST_DAC_RATE_22050		0x400
#define PCAP2_ST_DAC_RATE_24000		0x500
#define PCAP2_ST_DAC_RATE_32000		0x600
#define PCAP2_ST_DAC_RATE_44100		0x700
#define PCAP2_ST_DAC_RATE_48000		0x800
#define PCAP2_ST_DAC_CLKSEL_MASK	0x80000
#define PCAP2_ST_DAC_CLKSEL_AP		0x80000
#define PCAP2_ST_DAC_CLKSEL_BP		0x0
#define PCAP2_ST_DAC_CLK_MASK		0x1c
#define PCAP2_ST_DAC_CLK_13M		0x0
#define PCAP2_ST_DAC_CLK_15M36		0x4
#define PCAP2_ST_DAC_CLK_16M8		0x8
#define PCAP2_ST_DAC_CLK_19M44		0xc
#define PCAP2_ST_DAC_CLK_26M		0x10
#define PCAP2_ST_DAC_CLK_MCLK		0x14
#define PCAP2_ST_DAC_CLK_FSYNC		0x18
#define PCAP2_ST_DAC_CLK_BITCLK		0x1c

#define PCAP2_INPUT_AMP_LOWPWR		0x80000
#define PCAP2_INPUT_AMP_V2EN2		0x200000

#define PCAP2_OUTPUT_AMP_PGAR_EN	0x800
#define PCAP2_OUTPUT_AMP_PGAL_EN	0x1000
#define PCAP2_OUTPUT_AMP_CDC_SW		0x100
#define PCAP2_OUTPUT_AMP_ST_DAC_SW	0x200

extern struct snd_soc_dai pcap2_dai[];
extern struct snd_soc_codec_device soc_codec_dev_pcap2;


#endif

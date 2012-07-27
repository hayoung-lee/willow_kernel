/*
 * wm8985.c  --  WM8985 ALSA SoC Audio driver
 *
 * Copyright 2010 Wolfson Microelectronics plc
 *
 * Author: Dimitris Papastamos <dp@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * TODO:
 *  o Add OUT3/OUT4 mixer controls.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <linux/gpio.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
//#include <linux/sec_jack.h>
#include <mach/gpio-willow.h>

#include "wm8985.h"

static unsigned int save_reg_power1;
static unsigned int save_reg_power2;
static unsigned int save_reg_power3;

int g_dacmute_firstseted = 0;
struct snd_soc_codec *codecwrite;

#define DEBUG_MSG(f, a...)
//#define DEBUG_MSG(f, a...)  printk(f, ## a)

extern int t10_jack_get_type(void);
void wm8985_set_path_prev(int mode, int jack_type);
void wm8985_change_path(struct snd_soc_codec *codec, int lrmode, int pathmode);

#define WM8985_NUM_SUPPLIES 4
static const char *wm8985_supply_names[WM8985_NUM_SUPPLIES] = {
	"DCVDD",
	"DBVDD",
	"AVDD1",
	"AVDD2"
};

static const u16 wm8985_reg_defs[] = {
	0x0000,     /* R0  - Software Reset */
	0x0000,     /* R1  - Power management 1 */
	0x0000,     /* R2  - Power management 2 */
	0x0000,     /* R3  - Power management 3 */
	0x0050,     /* R4  - Audio Interface */
	0x0000,     /* R5  - Companding control */
	0x0140,     /* R6  - Clock Gen control */
	0x0000,     /* R7  - Additional control */
	0x0000,     /* R8  - GPIO Control */
	0x0000,     /* R9  - Jack Detect Control 1 */
	0x0000,     /* R10 - DAC Control */
	0x00FF,     /* R11 - Left DAC digital Vol */
	0x00FF,     /* R12 - Right DAC digital vol */
	0x0000,     /* R13 - Jack Detect Control 2 */
	0x0100,     /* R14 - ADC Control */
	0x00FF,     /* R15 - Left ADC Digital Vol */
	0x00FF,     /* R16 - Right ADC Digital Vol */
	0x0000,     /* R17 */
	0x012C,     /* R18 - EQ1 - low shelf */
	0x002C,     /* R19 - EQ2 - peak 1 */
	0x002C,     /* R20 - EQ3 - peak 2 */
	0x002C,     /* R21 - EQ4 - peak 3 */
	0x002C,     /* R22 - EQ5 - high shelf */
	0x0000,     /* R23 */
	0x0032,     /* R24 - DAC Limiter 1 */
	0x0000,     /* R25 - DAC Limiter 2 */
	0x0000,     /* R26 */
	0x0000,     /* R27 - Notch Filter 1 */
	0x0000,     /* R28 - Notch Filter 2 */
	0x0000,     /* R29 - Notch Filter 3 */
	0x0000,     /* R30 - Notch Filter 4 */
	0x0000,     /* R31 */
	0x0038,     /* R32 - ALC control 1 */
	0x000B,     /* R33 - ALC control 2 */
	0x0032,     /* R34 - ALC control 3 */
	0x0000,     /* R35 - Noise Gate */
	0x0008,     /* R36 - PLL N */
	0x000C,     /* R37 - PLL K 1 */
	0x0093,     /* R38 - PLL K 2 */
	0x00E9,     /* R39 - PLL K 3 */
	0x0000,     /* R40 */
	0x0000,     /* R41 - 3D control */
	0x0000,     /* R42 - OUT4 to ADC */
	0x0000,     /* R43 - Beep control */
	0x0033,     /* R44 - Input ctrl */
	0x0010,     /* R45 - Left INP PGA gain ctrl */
	0x0010,     /* R46 - Right INP PGA gain ctrl */
	0x0100,     /* R47 - Left ADC BOOST ctrl */
	0x0100,     /* R48 - Right ADC BOOST ctrl */
	0x0002,     /* R49 - Output ctrl */
	0x0001,     /* R50 - Left mixer ctrl */
	0x0001,     /* R51 - Right mixer ctrl */
	0x0039,     /* R52 - LOUT1 (HP) volume ctrl */
	0x0039,     /* R53 - ROUT1 (HP) volume ctrl */
	0x0039,     /* R54 - LOUT2 (SPK) volume ctrl */
	0x0039,     /* R55 - ROUT2 (SPK) volume ctrl */
	0x0001,     /* R56 - OUT3 mixer ctrl */
	0x0001,     /* R57 - OUT4 (MONO) mix ctrl */
	0x0001,     /* R58 */
	0x0000,     /* R59 */
	0x0004,     /* R60 - OUTPUT ctrl */
	0x0000,     /* R61 - BIAS CTRL */
	0x0180,     /* R62 */
	0x0000      /* R63 */
};

/*
 * latch bit 8 of these registers to ensure instant
 * volume updates
 */
static const int volume_update_regs[] = {
	WM8985_LEFT_DAC_DIGITAL_VOL,
	WM8985_RIGHT_DAC_DIGITAL_VOL,
	WM8985_LEFT_ADC_DIGITAL_VOL,
	WM8985_RIGHT_ADC_DIGITAL_VOL,
	WM8985_LOUT2_SPK_VOLUME_CTRL,
	WM8985_ROUT2_SPK_VOLUME_CTRL,
	WM8985_LOUT1_HP_VOLUME_CTRL,
	WM8985_ROUT1_HP_VOLUME_CTRL,
	WM8985_LEFT_INP_PGA_GAIN_CTRL,
	WM8985_RIGHT_INP_PGA_GAIN_CTRL
};


static const struct {
	int div;
	int ratio;
} fs_ratios[] = {
	{ 10, 128 },
	{ 15, 192 },
	{ 20, 256 },
	{ 30, 384 },
	{ 40, 512 },
	{ 60, 768 },
	{ 80, 1024 },
	{ 120, 1536 }
};

static const int srates[] = { 48000, 32000, 24000, 16000, 12000, 8000 };

static const int bclk_divs[] = {
	1, 2, 4, 8, 16, 32
};

static int eqmode_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol);
static int eqmode_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol);

static const DECLARE_TLV_DB_SCALE(dac_tlv, -12700, 50, 1);
static const DECLARE_TLV_DB_SCALE(adc_tlv, -12700, 50, 1);
static const DECLARE_TLV_DB_SCALE(out_tlv, -5700, 100, 0);
static const DECLARE_TLV_DB_SCALE(lim_thresh_tlv, -600, 100, 0);
static const DECLARE_TLV_DB_SCALE(lim_boost_tlv, 0, 100, 0);
static const DECLARE_TLV_DB_SCALE(alc_min_tlv, -1200, 600, 0);
static const DECLARE_TLV_DB_SCALE(alc_max_tlv, -675, 600, 0);
static const DECLARE_TLV_DB_SCALE(alc_tar_tlv, -2250, 150, 0);
static const DECLARE_TLV_DB_SCALE(pga_vol_tlv, -1200, 75, 0);
static const DECLARE_TLV_DB_SCALE(boost_tlv, -1200, 300, 1);
static const DECLARE_TLV_DB_SCALE(eq_tlv, -1200, 100, 0);
static const DECLARE_TLV_DB_SCALE(aux_tlv, -1500, 300, 0);
static const DECLARE_TLV_DB_SCALE(bypass_tlv, -1500, 300, 0);
static const DECLARE_TLV_DB_SCALE(pga_boost_tlv, 0, 2000, 0);

static const char *alc_sel_text[] = { "Off", "Right", "Left", "Stereo" };
static const SOC_ENUM_SINGLE_DECL(alc_sel, WM8985_ALC_CONTROL_1, 7,
				  alc_sel_text);

static const char *alc_mode_text[] = { "ALC", "Limiter" };
static const SOC_ENUM_SINGLE_DECL(alc_mode, WM8985_ALC_CONTROL_3, 8,
				  alc_mode_text);

static const char *filter_mode_text[] = { "Audio", "Application" };
static const SOC_ENUM_SINGLE_DECL(filter_mode, WM8985_ADC_CONTROL, 7,
				  filter_mode_text);

static const char *eq_bw_text[] = { "Narrow", "Wide" };
static const char *eqmode_text[] = { "Capture", "Playback" };
static const SOC_ENUM_SINGLE_EXT_DECL(eqmode, eqmode_text);

static const char *eq1_cutoff_text[] = {
	"80Hz", "105Hz", "135Hz", "175Hz"
};
static const SOC_ENUM_SINGLE_DECL(eq1_cutoff, WM8985_EQ1_LOW_SHELF, 5,
				  eq1_cutoff_text);
static const char *eq2_cutoff_text[] = {
	"230Hz", "300Hz", "385Hz", "500Hz"
};
static const SOC_ENUM_SINGLE_DECL(eq2_bw, WM8985_EQ2_PEAK_1, 8, eq_bw_text);
static const SOC_ENUM_SINGLE_DECL(eq2_cutoff, WM8985_EQ2_PEAK_1, 5,
				  eq2_cutoff_text);
static const char *eq3_cutoff_text[] = {
	"650Hz", "850Hz", "1.1kHz", "1.4kHz"
};
static const SOC_ENUM_SINGLE_DECL(eq3_bw, WM8985_EQ3_PEAK_2, 8, eq_bw_text);
static const SOC_ENUM_SINGLE_DECL(eq3_cutoff, WM8985_EQ3_PEAK_2, 5,
				  eq3_cutoff_text);
static const char *eq4_cutoff_text[] = {
	"1.8kHz", "2.4kHz", "3.2kHz", "4.1kHz"
};
static const SOC_ENUM_SINGLE_DECL(eq4_bw, WM8985_EQ4_PEAK_3, 8, eq_bw_text);
static const SOC_ENUM_SINGLE_DECL(eq4_cutoff, WM8985_EQ4_PEAK_3, 5,
				  eq4_cutoff_text);
static const char *eq5_cutoff_text[] = {
	"5.3kHz", "6.9kHz", "9kHz", "11.7kHz"
};
static const SOC_ENUM_SINGLE_DECL(eq5_cutoff, WM8985_EQ5_HIGH_SHELF, 5,
				  eq5_cutoff_text);

static const char *speaker_mode_text[] = { "Class A/B", "Class D" };
static const SOC_ENUM_SINGLE_DECL(speaker_mode, 0x17, 8, speaker_mode_text);

static const char *depth_3d_text[] = {
	"Off",
	"6.67%",
	"13.3%",
	"20%",
	"26.7%",
	"33.3%",
	"40%",
	"46.6%",
	"53.3%",
	"60%",
	"66.7%",
	"73.3%",
	"80%",
	"86.7%",
	"93.3%",
	"100%"
};
static const SOC_ENUM_SINGLE_DECL(depth_3d, WM8985_3D_CONTROL, 0,
				  depth_3d_text);

typedef void (*select_route)(struct snd_soc_codec *);
typedef void (*select_mic_route)(struct snd_soc_codec *);

#define MAX_PLAYBACK_PATHS 16
#define MAX_VOICECALL_PATH 9
#define MAX_FACTRYTEST_PATH_PLAYBACK_PATHS 3

static const char *playback_path[] = {
  "OFF",            // 0
  "RCV",            // 1
  "SPK",            // 2
  "HP",             // 3
  "HP_NO_MIC", // 4
  "BT",             //5
  "SPK_HP",         //6
  "CRADLE",         //7
  "CRADLE_HP",      //8
  "SPK_BT",         //9
  "RING_SPK",       //10
  "RING_HP",        //11
  "RING_NO_MIC",    //12
  "RING_SPK_HP",    //13
  "RING_CRADLE",    //14
  "RING_CRADLE_HP", //15
  "RING_SPK_BT"     //16
};
static const char *voicecall_path[] = {
  "OFF",            //0
  "RCV",            // 1
  "SPK",            // 2
  "HP",             // 3
  "HP_NO_MIC",  // 4
  "BT",             //5
  "SPK_HP",         //6
  "CRADLE",         //7
  "CRADLE_HP",      //8
  "SPK_BT",         //9
};
static const char *mic_path[] = {
  "MIC OFF",
  "Main Mic",
  "Hands Free Mic",
  "BT Sco Mic",
  "Main Mic in Call",
  "Hands Free Mic in Call",
};
static const char *input_source[] = {
  "Default",
  "Voice Recognition",
  "Camcorder"
};
static const char *factorytest_path[] = {
  "FP_OFF",	// 0
  "FP_LEFT",  // 1
  "FP_RIGHT",  // 2
  "FP_BOTH",  // 3
};

enum output_path  {
  OFF               = 0,
  RCV               = 1,
  SPK               = 2,
  HP                = 3,
  HP_NO_MIC         = 4,
  BT                = 5,
  SPK_HP            = 6,
  CRADLE            = 7,
  CRADLE_HP         = 8,
  SPK_BT            = 9,
  RING_SPK          = 10,
  RING_HP           = 11,
  RING_NO_MIC       = 12,
  RING_SPK_HP       = 13,
  RING_CRADLE       = 14,
  RING_CRADLE_HP    = 15,
  RING_SPK_BT       = 16,
};

enum input_path    {
  MIC_OFF                   = 0,
  MAIN_MIC                  = 1,
  HAND_FREE_MIC             = 2,
  BT_SOC_MIC                = 3,
  MAIN_MIC_IN_CALL          = 4,
  HAND_FREE_MIC_IN_CALL     = 5,
};

enum ringtone_state  {
  RING_OFF          = 0,
  RING_ON           = 1,
};

enum input_source_state  {
  DEFAULT           = 0,
  RECOGNITION       = 1,
  CAMCORDER         = 2,
};

enum factorytest_path  {
  FP_OFF	= 0,
  FP_LEFT	= 1,
  FP_RIGHT	= 2,
  FP_BOTH	=3,
};

struct wm8985_priv {
	struct snd_soc_codec codec;
	enum snd_soc_control_type control_type;
	u16 reg_cache[WM8985_REGCACHENUM];
	struct regulator_bulk_data supplies[WM8985_NUM_SUPPLIES];
	unsigned int sysclk;
	unsigned int bclk;
	enum output_path cur_path;
	enum input_path rec_path;
	enum input_source_state input_source;
	enum ringtone_state ringtone_active;
	select_route *universal_playback_path;
	select_route *universal_voicecall_path;
	select_mic_route *universal_mic_path;
};

static const struct soc_enum path_control_enum[] = {
  SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(playback_path), playback_path),
  SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voicecall_path), voicecall_path),
  SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mic_path), mic_path),
  SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(input_source), input_source),
};

static const struct soc_enum factorytest_path_enum[] = {
  SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(factorytest_path), factorytest_path)
};

void wm8985_set_off(struct snd_soc_codec *codec)
{
	//DEBUG_MSG("[%s]",__func__);
	wm8985_change_path(codec, 3, 0);
}

void wm8985_set_playback_receiver(struct snd_soc_codec *codec)
{
	//struct wm8985_priv *wm8985 = codec->drvdata;
	//DEBUG_MSG("[%s]",__func__);
	wm8985_change_path(codec, 3, 0);
}

void wm8985_set_playback_headset(struct snd_soc_codec *codec)
{
	//struct wm8985_priv *wm8985 = codec->drvdata;
	//DEBUG_MSG("[%s]",__func__);
	wm8985_change_path(codec, 3, 1);
}

void wm8985_set_playback_speaker(struct snd_soc_codec *codec)
{
	//struct wm8985_priv *wm8985 = codec->drvdata;
	//DEBUG_MSG("[%s]",__func__);
	wm8985_change_path(codec, 3, 2);
}

void wm8985_set_playback_speaker_headset(struct snd_soc_codec *codec)
{
	//struct wm8985_priv *wm8985 = codec->drvdata;
	//DEBUG_MSG("[%s]",__func__);
	wm8985_change_path(codec, 3, 3);
}

//------------------------------------------------
// Definitions of sound path
//------------------------------------------------
select_route universal_wm8985_playback_paths[] =
	{wm8985_set_off, wm8985_set_playback_receiver,
	wm8985_set_playback_speaker, wm8985_set_playback_headset,
	wm8985_set_playback_headset, wm8985_set_off,//BT -> OFF
	wm8985_set_playback_speaker_headset};

static int wm8985_get_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct wm8985_priv *wm8985 = snd_soc_codec_get_drvdata(codec);
	ucontrol->value.integer.value[0] = wm8985->cur_path;
	return 0;
}

static int wm8985_set_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct wm8985_priv *wm8985 = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *mc = (struct soc_enum *)kcontrol->private_value;
	int path_num = ucontrol->value.integer.value[0];

	if (strcmp(mc->texts[path_num], playback_path[path_num])) {
		printk(KERN_ERR "Unknown path %s\n", mc->texts[path_num]);
		return -ENODEV;
	}

	if (path_num > MAX_PLAYBACK_PATHS) {
		printk(KERN_ERR "Unknown Path\n");
		return -ENODEV;
	}

	switch (path_num) {
	case OFF:// 0
		DEBUG_MSG("Switching off output path\n");
		break;
	case RCV:// 1
	case SPK:// 2
	case HP:// 3
	case HP_NO_MIC:// 4
		DEBUG_MSG("routing to %s\n", mc->texts[path_num]);
		break;
	case BT:// 5
		path_num = 0;
		break;
	case SPK_HP:// 6
		DEBUG_MSG("routing to %s\n", mc->texts[path_num]);
		break;

	/*Not supported*/
	case CRADLE:// 7
	case CRADLE_HP:// 8
	case SPK_BT:// 9
		printk(KERN_ERR "audio path[%d] Not supported!!\n", path_num);
		return -ENODEV;
		break;

	case RING_SPK:// 10
	case RING_HP:// 11
	case RING_NO_MIC:// 12
	case RING_SPK_HP:// 13
		DEBUG_MSG("routing to %s\n", mc->texts[path_num]);
		path_num -= 8;
		break;

	/*Not supported*/
	case RING_CRADLE:// 14
	case RING_CRADLE_HP:// 15
	case RING_SPK_BT://16
		printk(KERN_ERR "audio path[%d] Not supported!!\n", path_num);
		return -ENODEV;
		break;

	default:
		printk(KERN_ERR "audio path[%d] does not exists!!\n", path_num);
		return -ENODEV;
		break;
	}

	DEBUG_MSG("[%s]# path_num : %d,  jack_type : %d  \n", __func__,path_num);
	wm8985->cur_path = path_num;
	wm8985->universal_playback_path[wm8985->cur_path] (codec);

	return 0;
}

static int factorytest_get_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct wm8985_priv *wm8985 = snd_soc_codec_get_drvdata(codec);
	ucontrol->value.integer.value[0] = wm8985->cur_path;
	return 0;
}

static int factorytest_set_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	//struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	//struct wm8985_priv *wm8985 = snd_soc_codec_get_drvdata(codec);
	//struct soc_enum *mc = (struct soc_enum *)kcontrol->private_value;
	int path_num = ucontrol->value.integer.value[0];
	DEBUG_MSG("[%s] (%s)\n",__func__,factorytest_path[path_num]);

	switch(path_num)
	{
	case FP_OFF:
		wm8985_set_path_prev(0,0);
		break;
//TODO : impletent jack type
/*
	case FP_LEFT:
		wm8985_set_path_prev(1,t10_jack_get_type());
		break;
	case FP_RIGHT:
		wm8985_set_path_prev(2,t10_jack_get_type());
		break;
	case FP_BOTH:
		wm8985_set_path_prev(3,t10_jack_get_type());
		break;
*/
	default:
		printk(KERN_ERR "factorytest path[%d] does not exists!!\n", path_num);
		return -ENODEV;
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new wm8985_snd_controls[] = {
	SOC_SINGLE("Digital Loopback Switch", WM8985_COMPANDING_CONTROL,
		0, 1, 0),

	SOC_ENUM("ALC Capture Function", alc_sel),
	SOC_SINGLE_TLV("ALC Capture Max Volume", WM8985_ALC_CONTROL_1,
		3, 7, 0, alc_max_tlv),
	SOC_SINGLE_TLV("ALC Capture Min Volume", WM8985_ALC_CONTROL_1,
		0, 7, 0, alc_min_tlv),
	SOC_SINGLE_TLV("ALC Capture Target Volume", WM8985_ALC_CONTROL_2,
		0, 15, 0, alc_tar_tlv),
	SOC_SINGLE("ALC Capture Attack", WM8985_ALC_CONTROL_3, 0, 10, 0),
	SOC_SINGLE("ALC Capture Hold", WM8985_ALC_CONTROL_2, 4, 10, 0),
	SOC_SINGLE("ALC Capture Decay", WM8985_ALC_CONTROL_3, 4, 10, 0),
	SOC_ENUM("ALC Mode", alc_mode),
	SOC_SINGLE("ALC Capture NG Switch", WM8985_NOISE_GATE,
		3, 1, 0),
	SOC_SINGLE("ALC Capture NG Threshold", WM8985_NOISE_GATE,
		0, 7, 1),

	SOC_DOUBLE_R_TLV("Capture Volume", WM8985_LEFT_ADC_DIGITAL_VOL,
		WM8985_RIGHT_ADC_DIGITAL_VOL, 0, 255, 0, adc_tlv),
	SOC_DOUBLE_R("Capture PGA ZC Switch", WM8985_LEFT_INP_PGA_GAIN_CTRL,
		WM8985_RIGHT_INP_PGA_GAIN_CTRL, 7, 1, 0),
	SOC_DOUBLE_R_TLV("Capture PGA Volume", WM8985_LEFT_INP_PGA_GAIN_CTRL,
		WM8985_RIGHT_INP_PGA_GAIN_CTRL, 0, 63, 0, pga_vol_tlv),

	SOC_DOUBLE_R_TLV("Capture PGA Boost Volume",
		WM8985_LEFT_ADC_BOOST_CTRL, WM8985_RIGHT_ADC_BOOST_CTRL,
		8, 1, 0, pga_boost_tlv),

	SOC_DOUBLE("ADC Inversion Switch", WM8985_ADC_CONTROL, 0, 1, 1, 0),
	SOC_SINGLE("ADC 128x Oversampling Switch", WM8985_ADC_CONTROL, 8, 1, 0),

	SOC_DOUBLE_R_TLV("Playback Volume", WM8985_LEFT_DAC_DIGITAL_VOL,
		WM8985_RIGHT_DAC_DIGITAL_VOL, 0, 255, 0, dac_tlv),

	SOC_SINGLE("DAC Playback Limiter Switch", WM8985_DAC_LIMITER_1, 8, 1, 0),
	SOC_SINGLE("DAC Playback Limiter Decay", WM8985_DAC_LIMITER_1, 4, 10, 0),
	SOC_SINGLE("DAC Playback Limiter Attack", WM8985_DAC_LIMITER_1, 0, 11, 0),
	SOC_SINGLE_TLV("DAC Playback Limiter Threshold", WM8985_DAC_LIMITER_2,
		4, 7, 1, lim_thresh_tlv),
	SOC_SINGLE_TLV("DAC Playback Limiter Boost Volume", WM8985_DAC_LIMITER_2,
		0, 12, 0, lim_boost_tlv),
	SOC_DOUBLE("DAC Inversion Switch", WM8985_DAC_CONTROL, 0, 1, 1, 0),
	SOC_SINGLE("DAC Auto Mute Switch", WM8985_DAC_CONTROL, 2, 1, 0),
	SOC_SINGLE("DAC 128x Oversampling Switch", WM8985_DAC_CONTROL, 3, 1, 0),

	SOC_DOUBLE_R_TLV("Headphone Playback Volume", WM8985_LOUT1_HP_VOLUME_CTRL,
		WM8985_ROUT1_HP_VOLUME_CTRL, 0, 63, 0, out_tlv),
	SOC_DOUBLE_R("Headphone Playback ZC Switch", WM8985_LOUT1_HP_VOLUME_CTRL,
		WM8985_ROUT1_HP_VOLUME_CTRL, 7, 1, 0),
	SOC_DOUBLE_R("Headphone Switch", WM8985_LOUT1_HP_VOLUME_CTRL,
		WM8985_ROUT1_HP_VOLUME_CTRL, 6, 1, 1),

	SOC_DOUBLE_R_TLV("Speaker Playback Volume", WM8985_LOUT2_SPK_VOLUME_CTRL,
		WM8985_ROUT2_SPK_VOLUME_CTRL, 0, 63, 0, out_tlv),
	SOC_DOUBLE_R("Speaker Playback ZC Switch", WM8985_LOUT2_SPK_VOLUME_CTRL,
		WM8985_ROUT2_SPK_VOLUME_CTRL, 7, 1, 0),
	SOC_DOUBLE_R("Speaker Switch", WM8985_LOUT2_SPK_VOLUME_CTRL,
		WM8985_ROUT2_SPK_VOLUME_CTRL, 6, 1, 1),

	SOC_SINGLE("High Pass Filter Switch", WM8985_ADC_CONTROL, 8, 1, 0),
	SOC_ENUM("High Pass Filter Mode", filter_mode),
	SOC_SINGLE("High Pass Filter Cutoff", WM8985_ADC_CONTROL, 4, 7, 0),

	SOC_DOUBLE_R_TLV("Aux Bypass Volume",
		WM8985_LEFT_MIXER_CTRL, WM8985_RIGHT_MIXER_CTRL, 6, 7, 0,
		aux_tlv),

	SOC_DOUBLE_R_TLV("Input PGA Bypass Volume",
		WM8985_LEFT_MIXER_CTRL, WM8985_RIGHT_MIXER_CTRL, 2, 7, 0,
		bypass_tlv),

	SOC_ENUM_EXT("Equalizer Function", eqmode, eqmode_get, eqmode_put),
	SOC_ENUM("EQ1 Cutoff", eq1_cutoff),
	SOC_SINGLE_TLV("EQ1 Volume", WM8985_EQ1_LOW_SHELF,  0, 24, 1, eq_tlv),
	SOC_ENUM("EQ2 Bandwith", eq2_bw),
	SOC_ENUM("EQ2 Cutoff", eq2_cutoff),
	SOC_SINGLE_TLV("EQ2 Volume", WM8985_EQ2_PEAK_1, 0, 24, 1, eq_tlv),
	SOC_ENUM("EQ3 Bandwith", eq3_bw),
	SOC_ENUM("EQ3 Cutoff", eq3_cutoff),
	SOC_SINGLE_TLV("EQ3 Volume", WM8985_EQ3_PEAK_2, 0, 24, 1, eq_tlv),
	SOC_ENUM("EQ4 Bandwith", eq4_bw),
	SOC_ENUM("EQ4 Cutoff", eq4_cutoff),
	SOC_SINGLE_TLV("EQ4 Volume", WM8985_EQ4_PEAK_3, 0, 24, 1, eq_tlv),
	SOC_ENUM("EQ5 Cutoff", eq5_cutoff),
	SOC_SINGLE_TLV("EQ5 Volume", WM8985_EQ5_HIGH_SHELF, 0, 24, 1, eq_tlv),

	SOC_ENUM("3D Depth", depth_3d),

	SOC_ENUM_EXT("Playback Path",     path_control_enum[0], wm8985_get_path,wm8985_set_path),
	SOC_ENUM_EXT("Factorytest Path",     factorytest_path_enum[0], factorytest_get_path,factorytest_set_path),

	SOC_ENUM("Speaker Mode", speaker_mode)
};

static const struct snd_kcontrol_new left_out_mixer[] = {
	SOC_DAPM_SINGLE("Line Switch", WM8985_LEFT_MIXER_CTRL, 1, 1, 0),
	SOC_DAPM_SINGLE("Aux Switch", WM8985_LEFT_MIXER_CTRL, 5, 1, 0),
	SOC_DAPM_SINGLE("PCM Switch", WM8985_LEFT_MIXER_CTRL, 0, 1, 0),
};

static const struct snd_kcontrol_new right_out_mixer[] = {
	SOC_DAPM_SINGLE("Line Switch", WM8985_RIGHT_MIXER_CTRL, 1, 1, 0),
	SOC_DAPM_SINGLE("Aux Switch", WM8985_RIGHT_MIXER_CTRL, 5, 1, 0),
	SOC_DAPM_SINGLE("PCM Switch", WM8985_RIGHT_MIXER_CTRL, 0, 1, 0),
};

static const struct snd_kcontrol_new left_input_mixer[] = {
	SOC_DAPM_SINGLE("L2 Switch", WM8985_INPUT_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("MicN Switch", WM8985_INPUT_CTRL, 1, 1, 0),
	SOC_DAPM_SINGLE("MicP Switch", WM8985_INPUT_CTRL, 0, 1, 0),
};

static const struct snd_kcontrol_new right_input_mixer[] = {
	SOC_DAPM_SINGLE("R2 Switch", WM8985_INPUT_CTRL, 6, 1, 0),
	SOC_DAPM_SINGLE("MicN Switch", WM8985_INPUT_CTRL, 5, 1, 0),
	SOC_DAPM_SINGLE("MicP Switch", WM8985_INPUT_CTRL, 4, 1, 0),
};

static const struct snd_kcontrol_new left_boost_mixer[] = {
	SOC_DAPM_SINGLE_TLV("L2 Volume", WM8985_LEFT_ADC_BOOST_CTRL,
		4, 7, 0, boost_tlv),
	SOC_DAPM_SINGLE_TLV("AUXL Volume", WM8985_LEFT_ADC_BOOST_CTRL,
		0, 7, 0, boost_tlv)
};

static const struct snd_kcontrol_new right_boost_mixer[] = {
	SOC_DAPM_SINGLE_TLV("R2 Volume", WM8985_RIGHT_ADC_BOOST_CTRL,
		4, 7, 0, boost_tlv),
	SOC_DAPM_SINGLE_TLV("AUXR Volume", WM8985_RIGHT_ADC_BOOST_CTRL,
		0, 7, 0, boost_tlv)
};

static const struct snd_soc_dapm_widget wm8985_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", WM8985_POWER_MANAGEMENT_3,
		0, 0),
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", WM8985_POWER_MANAGEMENT_3,
		1, 0),
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", WM8985_POWER_MANAGEMENT_2,
		0, 0),
	SND_SOC_DAPM_ADC("Right ADC", "Right Capture", WM8985_POWER_MANAGEMENT_2,
		1, 0),

	SND_SOC_DAPM_MIXER("Left Output Mixer", WM8985_POWER_MANAGEMENT_3,
		2, 0, left_out_mixer, ARRAY_SIZE(left_out_mixer)),
	SND_SOC_DAPM_MIXER("Right Output Mixer", WM8985_POWER_MANAGEMENT_3,
		3, 0, right_out_mixer, ARRAY_SIZE(right_out_mixer)),

	SND_SOC_DAPM_MIXER("Left Input Mixer", WM8985_POWER_MANAGEMENT_2,
		2, 0, left_input_mixer, ARRAY_SIZE(left_input_mixer)),
	SND_SOC_DAPM_MIXER("Right Input Mixer", WM8985_POWER_MANAGEMENT_2,
		3, 0, right_input_mixer, ARRAY_SIZE(right_input_mixer)),

	SND_SOC_DAPM_MIXER("Left Boost Mixer", WM8985_POWER_MANAGEMENT_2,
		4, 0, left_boost_mixer, ARRAY_SIZE(left_boost_mixer)),
	SND_SOC_DAPM_MIXER("Right Boost Mixer", WM8985_POWER_MANAGEMENT_2,
		5, 0, right_boost_mixer, ARRAY_SIZE(right_boost_mixer)),

	SND_SOC_DAPM_PGA("Left Capture PGA", WM8985_LEFT_INP_PGA_GAIN_CTRL,
		6, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right Capture PGA", WM8985_RIGHT_INP_PGA_GAIN_CTRL,
		6, 1, NULL, 0),

	SND_SOC_DAPM_PGA("Left Headphone Out", WM8985_POWER_MANAGEMENT_2,
		7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Headphone Out", WM8985_POWER_MANAGEMENT_2,
		8, 0, NULL, 0),

	SND_SOC_DAPM_PGA("Left Speaker Out", WM8985_POWER_MANAGEMENT_3,
		5, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Speaker Out", WM8985_POWER_MANAGEMENT_3,
		6, 0, NULL, 0),

	SND_SOC_DAPM_MICBIAS("Mic Bias", WM8985_POWER_MANAGEMENT_1, 4, 0),

	SND_SOC_DAPM_INPUT("LIN"),
	SND_SOC_DAPM_INPUT("LIP"),
	SND_SOC_DAPM_INPUT("RIN"),
	SND_SOC_DAPM_INPUT("RIP"),
	SND_SOC_DAPM_INPUT("AUXL"),
	SND_SOC_DAPM_INPUT("AUXR"),
	SND_SOC_DAPM_INPUT("L2"),
	SND_SOC_DAPM_INPUT("R2"),
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("SPKL"),
	SND_SOC_DAPM_OUTPUT("SPKR")
};

static const struct snd_soc_dapm_route audio_map[] = {
	{ "Right Output Mixer", "PCM Switch", "Right DAC" },
	{ "Right Output Mixer", "Aux Switch", "AUXR" },
	{ "Right Output Mixer", "Line Switch", "Right Boost Mixer" },

	{ "Left Output Mixer", "PCM Switch", "Left DAC" },
	{ "Left Output Mixer", "Aux Switch", "AUXL" },
	{ "Left Output Mixer", "Line Switch", "Left Boost Mixer" },

	{ "Right Headphone Out", NULL, "Right Output Mixer" },
	{ "HPR", NULL, "Right Headphone Out" },

	{ "Left Headphone Out", NULL, "Left Output Mixer" },
	{ "HPL", NULL, "Left Headphone Out" },

	{ "Right Speaker Out", NULL, "Right Output Mixer" },
	{ "SPKR", NULL, "Right Speaker Out" },

	{ "Left Speaker Out", NULL, "Left Output Mixer" },
	{ "SPKL", NULL, "Left Speaker Out" },

	{ "Right ADC", NULL, "Right Boost Mixer" },

	{ "Right Boost Mixer", "AUXR Volume", "AUXR" },
	{ "Right Boost Mixer", NULL, "Right Capture PGA" },
	{ "Right Boost Mixer", "R2 Volume", "R2" },

	{ "Left ADC", NULL, "Left Boost Mixer" },

	{ "Left Boost Mixer", "AUXL Volume", "AUXL" },
	{ "Left Boost Mixer", NULL, "Left Capture PGA" },
	{ "Left Boost Mixer", "L2 Volume", "L2" },

	{ "Right Capture PGA", NULL, "Right Input Mixer" },
	{ "Left Capture PGA", NULL, "Left Input Mixer" },

	{ "Right Input Mixer", "R2 Switch", "R2" },
	{ "Right Input Mixer", "MicN Switch", "RIN" },
	{ "Right Input Mixer", "MicP Switch", "RIP" },

	{ "Left Input Mixer", "L2 Switch", "L2" },
	{ "Left Input Mixer", "MicN Switch", "LIN" },
	{ "Left Input Mixer", "MicP Switch", "LIP" },
};

static int eqmode_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg;

	reg = snd_soc_read(codec, WM8985_EQ1_LOW_SHELF);
	if (reg & WM8985_EQ3DMODE)
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	return 0;
}

static int eqmode_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int regpwr2, regpwr3;
	unsigned int reg_eq;

	if (ucontrol->value.integer.value[0] != 0
			&& ucontrol->value.integer.value[0] != 1)
		return -EINVAL;

	reg_eq = snd_soc_read(codec, WM8985_EQ1_LOW_SHELF);
	switch ((reg_eq & WM8985_EQ3DMODE) >> WM8985_EQ3DMODE_SHIFT) {
	case 0:
		if (!ucontrol->value.integer.value[0])
			return 0;
		break;
	case 1:
		if (ucontrol->value.integer.value[0])
			return 0;
		break;
	}

	regpwr2 = snd_soc_read(codec, WM8985_POWER_MANAGEMENT_2);
	regpwr3 = snd_soc_read(codec, WM8985_POWER_MANAGEMENT_3);
	/* disable the DACs and ADCs */
	snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_2,
			    WM8985_ADCENR_MASK | WM8985_ADCENL_MASK, 0);
	snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_3,
			    WM8985_DACENR_MASK | WM8985_DACENL_MASK, 0);
	snd_soc_update_bits(codec, WM8985_ADDITIONAL_CONTROL,
			    WM8985_M128ENB_MASK, WM8985_M128ENB);
	/* set the desired eqmode */
	snd_soc_update_bits(codec, WM8985_EQ1_LOW_SHELF,
			    WM8985_EQ3DMODE_MASK,
			    ucontrol->value.integer.value[0]
			    << WM8985_EQ3DMODE_SHIFT);
	/* restore DAC/ADC configuration */
	snd_soc_write(codec, WM8985_POWER_MANAGEMENT_2, regpwr2);
	snd_soc_write(codec, WM8985_POWER_MANAGEMENT_3, regpwr3);
	return 0;
}

static int wm8985_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_new_controls(dapm, wm8985_dapm_widgets,
				  ARRAY_SIZE(wm8985_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_map,
				ARRAY_SIZE(audio_map));
	return 0;
}

static int wm8985_reset(struct snd_soc_codec *codec)
{
	return snd_soc_write(codec, WM8985_SOFTWARE_RESET, 0x0);
}


/*
	Control dac path
	@mode	   	0:none, 1:left, 2:right, 3: both
	@jack_type 	0:None, otherwise : jack value
*/
void wm8985_set_path_prev(int mode, int jack_type)
{
	int value=0, out_value=0, maskval=0;

	if(!codecwrite)
		return;

	DEBUG_MSG("[%s] # mode : %d, jack_type : %d\n", __func__, mode, jack_type);

	//TODO : impletent jack type
	/*
	switch(jack_type){
		case SEC_JACK_NO_DEVICE:
			out_value = 0;
			break;
		case SEC_HEADSET_3POLE:
		case SEC_HEADSET_4POLE:
			out_value = 1;
			break;
		//TODO : other value.
		default:
			break;
	}
	*/

	//set value
	maskval = 0;
	value = (mode==0 || 	mode==((maskval==1)?1:2)) || out_value;
		snd_soc_update_bits(codecwrite, WM8985_LOUT2_SPK_VOLUME_CTRL,
				   WM8985_LOUT2MUTE_MASK,
			   !!value << WM8985_LOUT2MUTE_SHIFT);	
	value = (mode==0 || mode==((maskval==1)?2:1)) || out_value;
		snd_soc_update_bits(codecwrite, WM8985_ROUT2_SPK_VOLUME_CTRL,
				   WM8985_ROUT2MUTE_MASK,
			   !!value << WM8985_ROUT2MUTE_SHIFT);

	maskval = 1;
	value = (mode==0 || mode==((maskval==1)?1:2)) || !out_value;
	snd_soc_update_bits(codecwrite, WM8985_LOUT1_HP_VOLUME_CTRL,
				   WM8985_LOUT1MUTE_MASK,
			   !!value << WM8985_LOUT1MUTE_SHIFT);
	value = (mode==0 || mode==((maskval==1)?2:1))|| !out_value;
	snd_soc_update_bits(codecwrite, WM8985_ROUT1_HP_VOLUME_CTRL,
				   WM8985_ROUT1MUTE_MASK,
			   !!value << WM8985_ROUT1MUTE_SHIFT);
}
EXPORT_SYMBOL(wm8985_set_path_prev);

/*
	Change dac path
	@mode	   	0:none, 1:left, 2:right, 3: both
	@pathmode 	0:none, 1:hp, 2:spk, 3:both
*/
void wm8985_change_path(struct snd_soc_codec *codec, int lrmode, int pathmode)
{
	int value, hpspk_invert=0, allenable=0, alldisabel=0, lrinvert=0;

	DEBUG_MSG("[%s] # lrmode : %d,  pathmode : %d\n", __func__, lrmode, pathmode);

	switch(pathmode){
		case 0://All disable
			alldisabel = 1;
			break;
		case 1:
			hpspk_invert = 1;
			break;
		case 2:
			hpspk_invert = 0;
			break;
		case 3://All enable
			allenable =1;
			break;
		default:
			break;
	}

	//set delay to avoid booting pop noise when path is changed before dac_mute routine is called.
	if(!g_dacmute_firstseted){
		msleep(100);
	}

	//set value
	lrinvert = 0;
	value = ((lrmode==0 || lrmode==((lrinvert==1)?1:2)) || hpspk_invert || alldisabel) && !allenable;
	snd_soc_update_bits(codec, WM8985_LOUT2_SPK_VOLUME_CTRL,
			   WM8985_LOUT2MUTE_MASK,
			   !!value << WM8985_LOUT2MUTE_SHIFT);	
	value = ((lrmode==0 || lrmode==((lrinvert==1)?2:1)) || hpspk_invert || alldisabel) && !allenable;
	snd_soc_update_bits(codec, WM8985_ROUT2_SPK_VOLUME_CTRL,
			   WM8985_ROUT2MUTE_MASK,
			   !!value << WM8985_ROUT2MUTE_SHIFT);

	lrinvert = 1;
	value = ((lrmode==0 || lrmode==((lrinvert==1)?1:2)) || !hpspk_invert || alldisabel) && !allenable;
	snd_soc_update_bits(codec, WM8985_LOUT1_HP_VOLUME_CTRL,
				   WM8985_LOUT1MUTE_MASK,
			   !!value << WM8985_LOUT1MUTE_SHIFT);
	value = ((lrmode==0 || lrmode==((lrinvert==1)?2:1))|| !hpspk_invert || alldisabel) && !allenable;
	snd_soc_update_bits(codec, WM8985_ROUT1_HP_VOLUME_CTRL,
				   WM8985_ROUT1MUTE_MASK,
			   !!value << WM8985_ROUT1MUTE_SHIFT);
}

EXPORT_SYMBOL(wm8985_change_path);


int wm8985_get_register(u32 reg)
{
	u32 retval = 0;

	if(!codecwrite)
		return -1;

	retval = snd_soc_read(codecwrite, reg);
	DEBUG_MSG("[%s] reg : 0x%x, retval : 0x%x\n", __func__,reg, retval);
	return retval;
}
EXPORT_SYMBOL(wm8985_get_register);

int wm8985_set_register(u32 reg, u32 val)
{
	u32 retval = 0;

	if(!codecwrite)
		return -1;

	retval = snd_soc_write(codecwrite, reg, val);
	DEBUG_MSG("[%s] reg : 0x%x, val : 0x%x, retval : 0x%x\n",__func__, reg, val, retval);
	return retval;
}
EXPORT_SYMBOL(wm8985_set_register);

static int wm8985_dac_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret;

	if(!g_dacmute_firstseted)
		g_dacmute_firstseted = 1;

	if(mute){
		gpio_set_value(GPIO_SPEAKER_AMP_OFF, !mute);
		gpio_set_value(GPIO_POP_DISABLE, !mute);
		//msleep(50);
		ret = snd_soc_update_bits(codec, WM8985_DAC_CONTROL,
					   WM8985_SOFTMUTE_MASK,
					   !!mute << WM8985_SOFTMUTE_SHIFT);
	}else{
		ret = snd_soc_update_bits(codec, WM8985_DAC_CONTROL,
				   WM8985_SOFTMUTE_MASK,
				   !!mute << WM8985_SOFTMUTE_SHIFT);
		gpio_set_value(GPIO_SPEAKER_AMP_OFF, !mute);
		gpio_set_value(GPIO_POP_DISABLE, !mute);
		msleep(100);
	}

	return ret;
}

static int wm8985_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec;
	u16 format, master, bcp, lrp;

	codec = dai->codec;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format = 0x2;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		format = 0x0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format = 0x1;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		format = 0x3;
		break;
	default:
		dev_err(dai->dev, "Unknown dai format\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, WM8985_AUDIO_INTERFACE,
			    WM8985_FMT_MASK, format << WM8985_FMT_SHIFT);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		master = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		master = 0;
		break;
	default:
		dev_err(dai->dev, "Unknown master/slave configuration\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, WM8985_CLOCK_GEN_CONTROL,
			    WM8985_MS_MASK, master << WM8985_MS_SHIFT);

	/* frame inversion is not valid for dsp modes */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_IB_IF:
		case SND_SOC_DAIFMT_NB_IF:
			return -EINVAL;
		default:
			break;
		}
		break;
	default:
		break;
	}

	bcp = lrp = 0;
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		bcp = lrp = 1;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		bcp = 1;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		lrp = 1;
		break;
	default:
		dev_err(dai->dev, "Unknown polarity configuration\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, WM8985_AUDIO_INTERFACE,
			    WM8985_LRP_MASK, lrp << WM8985_LRP_SHIFT);
	snd_soc_update_bits(codec, WM8985_AUDIO_INTERFACE,
			    WM8985_BCP_MASK, bcp << WM8985_BCP_SHIFT);
	return 0;
}

static int wm8985_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{

	struct snd_soc_codec *codec;
	struct wm8985_priv *wm8985;
	u16 blen;

	codec = dai->codec;
	codecwrite = dai->codec;
	wm8985 = snd_soc_codec_get_drvdata(codec);

	wm8985->bclk = snd_soc_params_to_bclk(params);
	if ((int)wm8985->bclk < 0)
		return wm8985->bclk;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		blen = 0x0;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		blen = 0x1;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		blen = 0x2;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		blen = 0x3;
		break;
	default:
		dev_err(dai->dev, "Unsupported word length %u\n",
			params_format(params));
		return -EINVAL;
	}

	snd_soc_update_bits(codec, WM8985_AUDIO_INTERFACE,
			    WM8985_WL_MASK, blen << WM8985_WL_SHIFT);

	/*
	 * match to the nearest possible sample rate and rely
	 * on the array index to configure the SR register
	 */
/*	srate_idx = 0;
	srate_best = abs(srates[0] - params_rate(params));
	for (i = 1; i < ARRAY_SIZE(srates); ++i) {
		if (abs(srates[i] - params_rate(params)) >= srate_best)
			continue;
		srate_idx = i;
		srate_best = abs(srates[i] - params_rate(params));
	}

	dev_dbg(dai->dev, "Selected SRATE = %d\n", srates[srate_idx]);
	snd_soc_update_bits(codec, WM8985_ADDITIONAL_CONTROL,
			    WM8985_SR_MASK, srate_idx << WM8985_SR_SHIFT);

	dev_dbg(dai->dev, "Target BCLK = %uHz\n", wm8985->bclk);
	dev_dbg(dai->dev, "SYSCLK = %uHz\n", wm8985->sysclk);

	for (i = 0; i < ARRAY_SIZE(fs_ratios); ++i) {
		if (wm8985->sysclk / params_rate(params)
				== fs_ratios[i].ratio)
			break;
	}

	if (i == ARRAY_SIZE(fs_ratios)) {
		dev_err(dai->dev, "Unable to configure MCLK ratio %u/%u\n",
			wm8985->sysclk, params_rate(params));
		return -EINVAL;
	}

	dev_dbg(dai->dev, "MCLK ratio = %dfs\n", fs_ratios[i].ratio);
	snd_soc_update_bits(codec, WM8985_CLOCK_GEN_CONTROL,
			    WM8985_MCLKDIV_MASK, i << WM8985_MCLKDIV_SHIFT);
*/
	/* select the appropriate bclk divider */
/*	tmp = (wm8985->sysclk / fs_ratios[i].div) * 10;
	for (i = 0; i < ARRAY_SIZE(bclk_divs); ++i) {
		if (wm8985->bclk == tmp / bclk_divs[i])
			break;
	}

	if (i == ARRAY_SIZE(bclk_divs)) {
		dev_err(dai->dev, "No matching BCLK divider found\n");
		return -EINVAL;
	}

	dev_dbg(dai->dev, "BCLK div = %d\n", i);
	snd_soc_update_bits(codec, WM8985_CLOCK_GEN_CONTROL,
			    WM8985_BCLKDIV_MASK, i << WM8985_BCLKDIV_SHIFT);
*/

	/* disable the PLL before reprogramming it */
	snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
			    WM8985_PLLEN_MASK, 0);

	/* set PLLN and PRESCALE */
	snd_soc_write(codec,WM8985_PLL_N,0x007); 
	/* set PLLK */
	snd_soc_write(codec,WM8985_PLL_K_1,0x016);
	snd_soc_write(codec,WM8985_PLL_K_2,0x0CC);
// THINKWARE_SIJUN(BSKIM) 2011-06-01 <<<START>>>
//	snd_soc_write(codec,WM8985_PLL_K_3,0x199);
	snd_soc_write(codec,WM8985_PLL_K_3,0x19A);
// THINKWARE_SIJUN(BSKIM) 2011-06-01 <<<END>>>
	/* set the source of the clock to be the PLL */
	snd_soc_update_bits(codec, WM8985_CLOCK_GEN_CONTROL,
			    WM8985_CLKSEL_MASK, 0);
	/* enable the PLL */
	snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
			    WM8985_PLLEN_MASK, WM8985_PLLEN);

	return 0;
}

struct pll_div {
	u32 div2:1;
	u32 n:4;
	u32 k:24;
};

#define FIXED_PLL_SIZE ((1ULL << 24) * 10)
static int pll_factors(struct pll_div *pll_div, unsigned int target,
		       unsigned int source)
{
	u64 Kpart;
	unsigned long int K, Ndiv, Nmod;

	pll_div->div2 = 0;
	Ndiv = target / source;
	if (Ndiv < 6) {
		source >>= 1;
		pll_div->div2 = 1;
		Ndiv = target / source;
	}

	if (Ndiv < 6 || Ndiv > 12) {
		printk(KERN_ERR "%s: WM8985 N value is not within"
		       " the recommended range: %lu\n", __func__, Ndiv);
		return -EINVAL;
	}
	pll_div->n = Ndiv;

	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (u64)Nmod;

	do_div(Kpart, source);

	K = Kpart & 0xffffffff;
	if ((K % 10) >= 5)
		K += 5;
	K /= 10;
	pll_div->k = K;

	return 0;
}

static int wm8985_set_pll(struct snd_soc_dai *dai, int pll_id,
			  int source, unsigned int freq_in,
			  unsigned int freq_out)
{
	int ret;
	struct snd_soc_codec *codec;
	struct pll_div pll_div;

	memset(&pll_div, 0, sizeof(pll_div));

	codec = dai->codec;
	if (freq_in && freq_out) {
		ret = pll_factors(&pll_div, freq_out * 4 * 2, freq_in);
		if (ret)
			return ret;
	}

	/* disable the PLL before reprogramming it */
	snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
			    WM8985_PLLEN_MASK, 0);
	
	if (!freq_in || !freq_out)
		return 0;

	/* set PLLN and PRESCALE */
	snd_soc_write(codec, WM8985_PLL_N,
		      (pll_div.div2 << WM8985_PLL_PRESCALE_SHIFT)
		      | pll_div.n);
	/* set PLLK */
	snd_soc_write(codec, WM8985_PLL_K_3, pll_div.k & 0x1ff);
	snd_soc_write(codec, WM8985_PLL_K_2, (pll_div.k >> 9) & 0x1ff);
	snd_soc_write(codec, WM8985_PLL_K_1, (pll_div.k >> 18));
	/* set the source of the clock to be the PLL */
	snd_soc_update_bits(codec, WM8985_CLOCK_GEN_CONTROL,
			    WM8985_CLKSEL_MASK, WM8985_CLKSEL);
	/* enable the PLL */
	snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
			    WM8985_PLLEN_MASK, WM8985_PLLEN);
	return 0;
}

static int wm8985_set_sysclk(struct snd_soc_dai *dai,
			     int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec;
	struct wm8985_priv *wm8985;

	codec = dai->codec;
	wm8985 = snd_soc_codec_get_drvdata(codec);

	switch (clk_id) {
	case WM8985_CLKSRC_MCLK:
		snd_soc_update_bits(codec, WM8985_CLOCK_GEN_CONTROL,
				    WM8985_CLKSEL_MASK, 0);
		snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
				    WM8985_PLLEN_MASK, 0);
		break;
	case WM8985_CLKSRC_PLL:
		snd_soc_update_bits(codec, WM8985_CLOCK_GEN_CONTROL,
				    WM8985_CLKSEL_MASK, WM8985_CLKSEL);
		break;
	default:
		dev_err(dai->dev, "Unknown clock source %d\n", clk_id);
		return -EINVAL;
	}

	wm8985->sysclk = freq;
	return 0;
}

static void wm8985_sync_cache(struct snd_soc_codec *codec)
{
	short i;
	u16 *cache;

	if (!codec->cache_sync)
		return;
	codec->cache_only = 0;
	/* restore cache */
	cache = codec->reg_cache;
	for (i = 0; i < codec->driver->reg_cache_size; i++) {
		if (i == WM8985_SOFTWARE_RESET
				|| cache[i] == wm8985_reg_defs[i])
			continue;
		snd_soc_write(codec, i, cache[i]);
	}
	codec->cache_sync = 0;
}

static int wm8985_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	int ret;
	struct wm8985_priv *wm8985;

	wm8985 = snd_soc_codec_get_drvdata(codec);
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		/* VMID at 75k */
		snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
				    WM8985_VMIDSEL_MASK,
				    1 << WM8985_VMIDSEL_SHIFT);
		snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
				    WM8985_MICBEN_MASK,
				    1 << WM8985_MICBEN_SHIFT);
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			ret = regulator_bulk_enable(ARRAY_SIZE(wm8985->supplies),
						    wm8985->supplies);
			if (ret) {
				dev_err(codec->dev,
					"Failed to enable supplies: %d\n",
					ret);
				return ret;
			}

			wm8985_sync_cache(codec);

			/* enable anti-pop features */
			snd_soc_update_bits(codec, WM8985_OUT4_TO_ADC,
					    WM8985_POBCTRL_MASK,
					    WM8985_POBCTRL);
			/* enable thermal shutdown */
			snd_soc_update_bits(codec, WM8985_OUTPUT_CTRL0,
					    WM8985_TSDEN_MASK, WM8985_TSDEN);
			snd_soc_update_bits(codec, WM8985_OUTPUT_CTRL0,
					    WM8985_TSOPCTRL_MASK,
					    WM8985_TSOPCTRL);
			/* enable BIASEN */
			snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
					    WM8985_BIASEN_MASK, WM8985_BIASEN);
			/* VMID at 75k */
			snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
					    WM8985_VMIDSEL_MASK,
					    1 << WM8985_VMIDSEL_SHIFT);
			msleep(500);
			/* disable anti-pop features */
			snd_soc_update_bits(codec, WM8985_OUT4_TO_ADC,
					    WM8985_POBCTRL_MASK, 0);
		}
	
		/* VMID at 300k */
		snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
				    WM8985_VMIDSEL_MASK,
				    2 << WM8985_VMIDSEL_SHIFT);
		break;
	case SND_SOC_BIAS_OFF:
		/* disable thermal shutdown */
		snd_soc_update_bits(codec, WM8985_OUTPUT_CTRL0,
				    WM8985_TSOPCTRL_MASK, 0);
		snd_soc_update_bits(codec, WM8985_OUTPUT_CTRL0,
				    WM8985_TSDEN_MASK, 0);
		/* disable VMIDSEL and BIASEN */
		snd_soc_update_bits(codec, WM8985_POWER_MANAGEMENT_1,
				    WM8985_VMIDSEL_MASK | WM8985_BIASEN_MASK,
				    0);
		snd_soc_write(codec, WM8985_POWER_MANAGEMENT_1, 0);
		snd_soc_write(codec, WM8985_POWER_MANAGEMENT_2, 0);
		snd_soc_write(codec, WM8985_POWER_MANAGEMENT_3, 0);

		codec->cache_sync = 1;

		regulator_bulk_disable(ARRAY_SIZE(wm8985->supplies),
				       wm8985->supplies);
		break;
	}

	codec->dapm.bias_level = level;
	return 0;
}

static void save_power_reg(struct snd_soc_codec *codec)
{
	save_reg_power1 = snd_soc_read(codec, WM8985_POWER_MANAGEMENT_1);
	save_reg_power2 = snd_soc_read(codec, WM8985_POWER_MANAGEMENT_2);
	save_reg_power3 = snd_soc_read(codec, WM8985_POWER_MANAGEMENT_3);
}

static void restore_power_reg(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, WM8985_POWER_MANAGEMENT_1, save_reg_power1);
	snd_soc_write(codec, WM8985_POWER_MANAGEMENT_2, save_reg_power2);
	snd_soc_write(codec, WM8985_POWER_MANAGEMENT_3, save_reg_power3);

	save_reg_power1 = save_reg_power2 = save_reg_power3 = 0;
}

#ifdef CONFIG_PM
static int wm8985_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	save_power_reg(codec);

	wm8985_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm8985_resume(struct snd_soc_codec *codec)
{
	restore_power_reg(codec);

	wm8985_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}
#else
#define wm8985_suspend NULL
#define wm8985_resume NULL
#endif

static void wm8985_register_setting(struct snd_soc_codec *codec )
{
        snd_soc_update_bits(codec, WM8985_LEFT_MIXER_CTRL,
                           WM8985_DACL2LMIX_MASK,
                           !!0 << WM8985_DACL2LMIX_SHIFT);
        snd_soc_update_bits(codec, WM8985_RIGHT_MIXER_CTRL,
                           WM8985_DACR2RMIX_MASK,
                           !!0 << WM8985_DACR2RMIX_SHIFT);

        snd_soc_update_bits(codec, WM8985_LOUT1_HP_VOLUME_CTRL,
                           WM8985_LOUT1MUTE_MASK,
                           !!1 << WM8985_LOUT1MUTE_SHIFT);
        snd_soc_update_bits(codec, WM8985_ROUT1_HP_VOLUME_CTRL,
                           WM8985_ROUT1MUTE_MASK,
                           !!1 << WM8985_ROUT1MUTE_SHIFT);

        snd_soc_update_bits(codec, WM8985_LOUT2_SPK_VOLUME_CTRL,
                           WM8985_LOUT2MUTE_MASK,
                           !!0 << WM8985_LOUT2MUTE_SHIFT);
        snd_soc_update_bits(codec, WM8985_ROUT2_SPK_VOLUME_CTRL,
                           WM8985_ROUT2MUTE_MASK,
                           !!0 << WM8985_ROUT2MUTE_SHIFT);


        snd_soc_write(codec,WM8985_BIAS_CTRL,0x100);
        snd_soc_write(codec,WM8985_OUTPUT_CTRL0,0x000);

        msleep(500);
        snd_soc_write(codec,WM8985_POWER_MANAGEMENT_1,0x0FD);
        msleep(500);

	snd_soc_write(codec,WM8985_POWER_MANAGEMENT_2,0x180);
	snd_soc_write(codec,WM8985_POWER_MANAGEMENT_3,0x1EF);

	snd_soc_write(codec,WM8985_AUDIO_INTERFACE,0x012);
	snd_soc_write(codec,WM8985_COMPANDING_CONTROL,0x000);
	snd_soc_write(codec,WM8985_ADDITIONAL_CONTROL,0X180);
	snd_soc_write(codec,WM8985_GPIO_CONTROL,0x000);
	snd_soc_write(codec,WM8985_JACK_DETECT_CONTROL_2,0x000);
	snd_soc_write(codec,WM8985_CLOCK_GEN_CONTROL,0x000);//R6

	//DAC
	snd_soc_write(codec,WM8985_DAC_CONTROL,0x048);//R10(0xA)
	snd_soc_write(codec,WM8985_LEFT_DAC_DIGITAL_VOL,0x1FD);//R11(0x0B)
	snd_soc_write(codec,WM8985_RIGHT_DAC_DIGITAL_VOL,0x1FD);//R12(0x0C)
	snd_soc_write(codec,WM8985_LOUT1_HP_VOLUME_CTRL,0x17A);//R52(0x34)
	snd_soc_write(codec,WM8985_ROUT1_HP_VOLUME_CTRL,0x17A);//R53(0x35)
	snd_soc_write(codec,WM8985_LOUT2_SPK_VOLUME_CTRL,0x138);//R54(0x36)
	snd_soc_write(codec,WM8985_ROUT2_SPK_VOLUME_CTRL,0x138);//R55(0x37)

	//ADC-EQ
	snd_soc_write(codec,WM8985_EQ1_LOW_SHELF,0x06C);//R18(0x12)
	snd_soc_write(codec,WM8985_EQ2_PEAK_1,0x02C);//R19(0x13)
	snd_soc_write(codec,WM8985_EQ3_PEAK_2,0x02C);//R20(0x14)
	snd_soc_write(codec,WM8985_EQ4_PEAK_3,0x02C);//R21(0x15)
	snd_soc_write(codec,WM8985_EQ5_HIGH_SHELF,0x02C);//R22(0x16)

	//ADC-Vol
	snd_soc_write(codec,WM8985_ADC_CONTROL,0x1A0);//R14(0x0E)
	snd_soc_write(codec,WM8985_LEFT_ADC_DIGITAL_VOL,0x1F5);//R15(0x0F)
	snd_soc_write(codec,WM8985_RIGHT_ADC_DIGITAL_VOL,0x1F5);//R16(0x10)
	snd_soc_write(codec,WM8985_LEFT_INP_PGA_GAIN_CTRL,0x13A);//R45(0x2D)
	snd_soc_write(codec,WM8985_RIGHT_INP_PGA_GAIN_CTRL,0x17A);//R46(0x2E)
	//snd_soc_write(codec,WM8985_LEFT_ADC_BOOST_CTRL,0x000);//R47(0x2F)
	//snd_soc_write(codec,WM8985_RIGHT_ADC_BOOST_CTRL,0x000);//R48(0x30)

	//ADC-Howling
	snd_soc_write(codec,WM8985_LEFT_MIXER_CTRL,0x8);//0x32 (50) - BYPL2LMIX[1] : ON,	BYPLMIX VOL[4:2] : 2
	//snd_soc_write(codec,WM8985_BEEP_CONTROL,0x000);	//0x2B (43) - BYPL2RMIX[8]  ; ON, 	BYPR2LMIX[7]  ; OFF
	snd_soc_write(codec,WM8985_RIGHT_MIXER_CTRL,0x0);//0x33 (51) - BYPR2RMIX[8] : OFF, - BYPRMIX VOL [4:2] : 0

}

static void amp_on(struct snd_soc_codec *codec)
{
        msleep(500);
	snd_soc_update_bits(codec, WM8985_LEFT_MIXER_CTRL,
			        WM8985_DACL2LMIX_MASK,
                                !!1 << WM8985_DACL2LMIX_SHIFT);
        snd_soc_update_bits(codec, WM8985_RIGHT_MIXER_CTRL,
                                WM8985_DACR2RMIX_MASK,
                                !!1 << WM8985_DACR2RMIX_SHIFT);
}

static int wm8985_remove(struct snd_soc_codec *codec)
{
	struct wm8985_priv *wm8985;

	wm8985 = snd_soc_codec_get_drvdata(codec);
	wm8985_set_bias_level(codec, SND_SOC_BIAS_OFF);
	regulator_bulk_free(ARRAY_SIZE(wm8985->supplies), wm8985->supplies);

	gpio_free(GPIO_SPEAKER_AMP_OFF);
	gpio_free(GPIO_POP_DISABLE);

	return 0;
}

static int wm8985_probe(struct snd_soc_codec *codec)
{
	size_t i;
	struct wm8985_priv *wm8985;
	int ret;
	u16 *cache;

	wm8985 = snd_soc_codec_get_drvdata(codec);

	ret = snd_soc_codec_set_cache_io(codec, 7, 9, wm8985->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache i/o: %d\n", ret);
		return ret;
	}

	ret = gpio_request(GPIO_SPEAKER_AMP_OFF, "SPEAKER_AMP_OFF");
	if (ret) {
		dev_err(codec->dev,"[%s] SPEAKER_AMP_OFF gpio request error : %d\n",__func__,ret);
		goto exit_spk_amp_off_gpio_request_failed;
	} 
	gpio_direction_output(GPIO_SPEAKER_AMP_OFF, 0);
	
	ret = gpio_request(GPIO_POP_DISABLE, "POP_DISABLE");
	if (ret) {
		dev_err(codec->dev,"[%s] GPIO_POP_DISABLE gpio request error : %d\n",__func__,ret);
		goto exit_pop_disable_gpio_request_failed;
	} 
	gpio_direction_output(GPIO_POP_DISABLE, 0);

	for (i = 0; i < ARRAY_SIZE(wm8985->supplies); i++)
		wm8985->supplies[i].supply = wm8985_supply_names[i];

	ret = regulator_bulk_get(codec->dev, ARRAY_SIZE(wm8985->supplies),
				 wm8985->supplies);
	if (ret) {
		dev_err(codec->dev, "Failed to request supplies: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(wm8985->supplies),
				    wm8985->supplies);
	if (ret) {
		dev_err(codec->dev, "Failed to enable supplies: %d\n", ret);
		goto err_reg_get;
	}

	ret = wm8985_reset(codec);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to issue reset: %d\n", ret);
		goto err_reg_enable;
	}

	cache = codec->reg_cache;
	/* latch volume update bits */
	for (i = 0; i < ARRAY_SIZE(volume_update_regs); ++i)
		cache[volume_update_regs[i]] |= 0x100;
	/* enable BIASCUT */
	cache[WM8985_BIAS_CTRL] |= WM8985_BIASCUT;
	codec->cache_sync = 1;

	snd_soc_add_controls(codec, wm8985_snd_controls,
			     ARRAY_SIZE(wm8985_snd_controls));
	wm8985_add_widgets(codec);

	wm8985_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	wm8985_register_setting(codec);

	amp_on(codec);
	return 0;


err_reg_enable:
	regulator_bulk_disable(ARRAY_SIZE(wm8985->supplies), wm8985->supplies);
err_reg_get:
	regulator_bulk_free(ARRAY_SIZE(wm8985->supplies), wm8985->supplies);
exit_pop_disable_gpio_request_failed:	
	gpio_free(GPIO_POP_DISABLE);
exit_spk_amp_off_gpio_request_failed:
	gpio_free(GPIO_SPEAKER_AMP_OFF);
	
	return ret;
}

static struct snd_soc_dai_ops wm8985_dai_ops = {
	.digital_mute = wm8985_dac_mute,
	.hw_params = wm8985_hw_params,
	.set_fmt = wm8985_set_fmt,
	.set_sysclk = wm8985_set_sysclk,
	.set_pll = wm8985_set_pll
};

#define WM8985_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver wm8985_dai = {
	.name = "wm8985-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = WM8985_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = WM8985_FORMATS,
	},
	.ops = &wm8985_dai_ops,
	.symmetric_rates = 0
};

static struct snd_soc_codec_driver soc_codec_dev_wm8985 = {
	.probe = wm8985_probe,
	.remove = wm8985_remove,
	.suspend = wm8985_suspend,
	.resume = wm8985_resume,
	.set_bias_level = wm8985_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(wm8985_reg_defs),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = wm8985_reg_defs
};

static void wm8985_set_priv_data(struct wm8985_priv *wm8985){
	wm8985->universal_playback_path = universal_wm8985_playback_paths;
	wm8985->cur_path = OFF;
	wm8985->rec_path = MIC_OFF;	
}

#if defined(CONFIG_SPI_MASTER)
static int __devinit wm8985_spi_probe(struct spi_device *spi)
{
	struct wm8985_priv *wm8985;
	int ret;

	wm8985 = kzalloc(sizeof *wm8985, GFP_KERNEL);
	if (!wm8985)
		return -ENOMEM;

	wm8985->control_type = SND_SOC_SPI;
	wm8985_set_priv_data(wm8985);
	
	spi_set_drvdata(spi, wm8985);

	ret = snd_soc_register_codec(&spi->dev,
				     &soc_codec_dev_wm8985, &wm8985_dai, 1);
	if (ret < 0)
		kfree(wm8985);
	return ret;
}

static int __devexit wm8985_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_codec(&spi->dev);
	kfree(spi_get_drvdata(spi));
	return 0;
}

static struct spi_driver wm8985_spi_driver = {
	.driver = {
		.name = "wm8985",
		.owner = THIS_MODULE,
	},
	.probe = wm8985_spi_probe,
	.remove = __devexit_p(wm8985_spi_remove)
};
#endif

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static __devinit int wm8985_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct wm8985_priv *wm8985;
	int ret;

	wm8985 = kzalloc(sizeof *wm8985, GFP_KERNEL);
	if (!wm8985)
		return -ENOMEM;

	wm8985->control_type = SND_SOC_I2C;
	wm8985_set_priv_data(wm8985);

	i2c_set_clientdata(i2c, wm8985);

	ret = snd_soc_register_codec(&i2c->dev,
				     &soc_codec_dev_wm8985, &wm8985_dai, 1);
	if (ret < 0)
		kfree(wm8985);

	return ret;
}

static __devexit int wm8985_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id wm8985_i2c_id[] = {
	{ "wm8985", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wm8985_i2c_id);

static struct i2c_driver wm8985_i2c_driver = {
	.driver = {
		.name = "wm8985",
		.owner = THIS_MODULE,
	},
	.probe = wm8985_i2c_probe,
	.remove = __devexit_p(wm8985_i2c_remove),
	.id_table = wm8985_i2c_id
};
#endif

static int __init wm8985_modinit(void)
{
	int ret = 0;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&wm8985_i2c_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register wm8985 I2C driver: %d\n",
		       ret);
	}
	return ret;	
#endif
#if defined(CONFIG_SPI_MASTER)
	ret = spi_register_driver(&wm8985_spi_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register wm8985 SPI driver: %d\n",
		       ret);
	}
#endif
	return ret;
}
module_init(wm8985_modinit);

static void __exit wm8985_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8985_i2c_driver);
#endif
#if defined(CONFIG_SPI_MASTER)
	spi_unregister_driver(&wm8985_spi_driver);
#endif
}
module_exit(wm8985_exit);

MODULE_DESCRIPTION("ASoC WM8985 driver");
MODULE_AUTHOR("Dimitris Papastamos <dp@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");

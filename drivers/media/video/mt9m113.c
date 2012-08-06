/* linux/drivers/media/video/mt9m113.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Driver for MT9M113 (SXGA camera) from Samsung Electronics
 * 1/6" 1.3Mp CMOS Image Sensor SoC with an Embedded Image Processor
 * supporting MIPI CSI-2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/mt9m113_platform.h>

#include <linux/videodev2_samsung.h>

#include "mt9m113.h"

#define MT9M113_DEBUG
#ifdef MT9M113_DEBUG
#define mt9m113_info dev_info
#define mt9m113_err dev_err
#else
#define mt9m113_info(dev, format, arg...) do { } while (0)
#define mt9m113_err dev_err
#endif

#define MT9M113_DRIVER_NAME	"MT9M113"

/* Default resolution & pixelformat. plz ref mt9m113_platform.h */
#define DEFAULT_RES		WVGA	/* Index of resoultion */
#define DEFAUT_FPS_INDEX	MT9M113_15FPS
#define DEFAULT_FMT		V4L2_PIX_FMT_UYVY	/* YUV422 */

/*
 * Specification
 * Parallel : ITU-R. 656/601 YUV422, RGB565, RGB888 (Up to VGA), RAW10
 * Serial : MIPI CSI2 (single lane) YUV422, RGB565, RGB888 (Up to VGA), RAW10
 * Resolution : 1280 (H) x 1024 (V)
 * Image control : Brightness, Contrast, Saturation, Sharpness, Glamour
 * Effect : Mono, Negative, Sepia, Aqua, Sketch
 * FPS : 15fps @full resolution, 30fps @VGA, 24fps @720p
 * Max. pixel clock frequency : 48MHz(upto)
 * Internal PLL (6MHz to 27MHz input frequency)
 */

/* Camera functional setting values configured by user concept */
struct mt9m113_userset {
	signed int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int ae_lock;
	unsigned int awb_lock;
	unsigned int auto_wb;	/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int manual_wb;	/* V4L2_CID_WHITE_BALANCE_PRESET */
	unsigned int wb_temp;	/* V4L2_CID_WHITE_BALANCE_TEMPERATURE */
	unsigned int effect;	/* Color FX (AKA Color tone) */
	unsigned int contrast;	/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
	unsigned int glamour;
};

struct mt9m113_state {
	struct mt9m113_mbus_platform_data *pdata;
	struct v4l2_subdev sd;
	struct v4l2_mbus_framefmt 	fmt;
	struct v4l2_fract 		timeperframe;
	struct mt9m113_userset 		userset;
	int 				freq;	/* MCLK in KHz */
	int 				is_mipi;
	int 				isize;
	int 				ver;
	int 				fps;
	int 				fmt_index;
	unsigned short 			devid_mask;
	
	enum mt9m113_runmode runmode;

	int default_width;
	int default_height;
	int capture_width;
	int capture_height;
	int set_vhflip;
	int check_dataline;	
};

static inline struct mt9m113_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9m113_state, sd);
}

static int mt9m113_i2c_write(struct v4l2_subdev *sd, unsigned char *txdata, int length)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[] = {
		{
			.addr = client->addr >> 1,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	int I2C_RETRY = 0;

	for (; I2C_RETRY < 2; I2C_RETRY++) {
		if (i2c_transfer(client->adapter, msg, 1) < 0) {
			if (1 == I2C_RETRY) {
				mt9m113_err(&client->dev, "%s: failed to transfer to addr:0x%x\n", __func__, client->addr);
				return -EIO;
			}
			mdelay(10);
		} else {
			break;
		}
	}

	return 0;
}

static int mt9m113_i2c_write_regs(struct v4l2_subdev *sd,
	struct mt9m113_reg const *reg_tbl, int num_of_items_in_table) {
	int i;
	int err = -EIO;
	unsigned char buf[4];

	for (i = 0; i < num_of_items_in_table; i++) {
		buf[0] = (reg_tbl->waddr & 0xFF00) >> 8;
		buf[1] = (reg_tbl->waddr & 0x00FF);
		buf[2] = (reg_tbl->wdata & 0xFF00) >> 8;
		buf[3] = (reg_tbl->wdata & 0x00FF);

		err = mt9m113_i2c_write(sd, buf, 4);
		if (err < 0)
			break;

		if (reg_tbl->mdelay_time != 0)
			mdelay(reg_tbl->mdelay_time);

		reg_tbl++;
	}
	return err;
}
/*
 * MT9M113 register structure : 2bytes address, 2bytes value
 * retry on write failure up-to 5 times
 */
static inline int mt9m113_write(struct v4l2_subdev *sd, u16 addr, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[1];
	unsigned char reg[4];
	int err = 0;
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = reg;

	reg[0] = addr >> 8;
	reg[1] = addr & 0xff;
	reg[2] = val >> 8;
	reg[3] = val & 0xff;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return err;	/* Returns here on success */

	/* abnormal case: retry 5 times */
	if (retry < 5) {
		dev_err(&client->dev, "%s: address: 0x%02x%02x, " \
			"value: 0x%02x%02x error..\n", __func__, \
			reg[0], reg[1], reg[2], reg[3]);
		retry++;
		goto again;
	}

	return err;
}

static const char *mt9m113_querymenu_wb_preset[] = {
	"WB Tungsten", "WB Fluorescent", "WB sunny", "WB cloudy", NULL
};

static const char *mt9m113_querymenu_effect_mode[] = {
	"Effect Sepia", "Effect Aqua", "Effect Monochrome",
	"Effect Negative", "Effect Sketch", NULL
};

static const char *mt9m113_querymenu_ev_bias_mode[] = {
	"-3EV",	"-2,1/2EV", "-2EV", "-1,1/2EV",
	"-1EV", "-1/2EV", "0", "1/2EV",
	"1EV", "1,1/2EV", "2EV", "2,1/2EV",
	"3EV", NULL
};

static struct v4l2_queryctrl mt9m113_controls[] = {
	{
		/*
		 * For now, we just support in preset type
		 * to be close to generic WB system,
		 * we define color temp range for each preset
		 */
		.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "White balance in kelvin",
		.minimum = 0,
		.maximum = 10000,
		.step = 1,
		.default_value = 0,	/* FIXME */
	},
	{
		.id = V4L2_CID_WHITE_BALANCE_PRESET,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "White balance preset",
		.minimum = 0,
		.maximum = ARRAY_SIZE(mt9m113_querymenu_wb_preset) - 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Auto white balance",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Exposure bias",
		.minimum = 0,
		.maximum = ARRAY_SIZE(mt9m113_querymenu_ev_bias_mode) - 2,
		.step = 1,
		.default_value = (ARRAY_SIZE(mt9m113_querymenu_ev_bias_mode) \
				- 2) / 2,	/* 0 EV */
	},
	{
		.id = V4L2_CID_COLORFX,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Image Effect",
		.minimum = 0,
		.maximum = ARRAY_SIZE(mt9m113_querymenu_effect_mode) - 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
	{
		.id = V4L2_CID_SATURATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Saturation",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
	{
		.id = V4L2_CID_SHARPNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Sharpness",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
};

static int mt9m113_set_preview_start(struct v4l2_subdev *sd)
{
#ifdef MT9M113_DEBUG
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#endif
	struct mt9m113_state *state = to_state(sd);

	int err = 0;
	mt9m113_info(&client->dev, "%s: set_Preview_start\n", __func__);
    msleep(120);

    if (!state->fmt.width || !state->fmt.height)
		return -EINVAL;

#if 0
	if(state->check_dataline) {
//		err = mt9m113_write_regs(sd, mt9m113_pattern_on,
//				sizeof(mt9m113_pattern_on) / sizeof(mt9m113_pattern_on[0]));
//		msleep(100);
	} else {
//		if(state->set_vhflip == 1) {
//			err = mt9m113_write_regs(sd, mt9m113_vhflip_on,
//					sizeof(mt9m113_vhflip_on) / sizeof(mt9m113_vhflip_on[0]));
//		} else {
//			err = mt9m113_write_regs(sd, mt9m113_vhflip_off,
//					sizeof(mt9m113_vhflip_off) / sizeof(mt9m113_vhflip_off[0]));
//		}
#endif
		/* set initial regster value */
		err = mt9m113_i2c_write_regs(sd, &mt9m113_preview_regs[0], ARRAY_SIZE(mt9m113_preview_regs));
		msleep(50);

		//0006672: 카메라 연속 촬영 시 촬영된 이미지 깨져 비정상적으로 보여짐
		err = mt9m113_i2c_write_regs(sd, &mt9m113_preview_regs[0], ARRAY_SIZE(mt9m113_preview_regs));

       msleep(50);

		if (unlikely(err)) {
			mt9m113_info(&client->dev, "%s: failed to make preview\n", __func__);
			return err;
		}
		mt9m113_info(&client->dev, "%s: set_Preview_Setting end ...\n", __func__);

		state->runmode = MT9M113_RUNMODE_RUNNING;
	//}
	
	return err;
}

static int mt9m113_set_preview_stop(struct v4l2_subdev *sd)
{
	struct mt9m113_state *state = to_state(sd);

	int err = 0;
	
	if (state->runmode == MT9M113_RUNMODE_RUNNING) {
		state->runmode = MT9M113_RUNMODE_IDLE;
		state->set_vhflip = 0;
	}

	return err;
}

const char * const *mt9m113_ctrl_get_menu(u32 id)
{
	switch (id) {
	case V4L2_CID_WHITE_BALANCE_PRESET:
		return mt9m113_querymenu_wb_preset;

	case V4L2_CID_COLORFX:
		return mt9m113_querymenu_effect_mode;

	case V4L2_CID_EXPOSURE:
		return mt9m113_querymenu_ev_bias_mode;

	default:
		return v4l2_ctrl_get_menu(id);
	}
}

static inline struct v4l2_queryctrl const *mt9m113_find_qctrl(int id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mt9m113_controls); i++)
		if (mt9m113_controls[i].id == id)
			return &mt9m113_controls[i];

	return NULL;
}

static int mt9m113_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mt9m113_controls); i++) {
		if (mt9m113_controls[i].id == qc->id) {
			memcpy(qc, &mt9m113_controls[i], \
				sizeof(struct v4l2_queryctrl));
			return 0;
		}
	}

	return -EINVAL;
}

static int mt9m113_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	struct v4l2_queryctrl qctrl;

	qctrl.id = qm->id;
	mt9m113_queryctrl(sd, &qctrl);

	return v4l2_ctrl_query_menu(qm, &qctrl, mt9m113_ctrl_get_menu(qm->id));
}

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 *	freq : in Hz
 *	flag : not supported for now
 */
static int mt9m113_s_crystal_freq(struct v4l2_subdev *sd, u32  freq, u32 flags)
{
	int err = -EINVAL;

	return err;
}

static int mt9m113_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	struct mt9m113_state *state = to_state(sd);
	int err = 0;

	*fmt = state->fmt;
	return err;
}

static int mt9m113_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m113_state *state = to_state(sd);
	int err = 0;

	dev_dbg(&client->dev, "requested res(%d, %d)\n",
		fmt->width, fmt->height);

	if (!state->fmt.width ||
		!state->fmt.height ||
		!state->fmt.code)
		state->fmt = *fmt;
	else
		*fmt = state->fmt;

	return err;
}
static int mt9m113_enum_framesizes(struct v4l2_subdev *sd,
					struct v4l2_frmsizeenum *fsize)
{
	int err = 0;

	return err;
}

static int mt9m113_enum_frameintervals(struct v4l2_subdev *sd,
					struct v4l2_frmivalenum *fival)
{
	int err = 0;

	return err;
}

static int mt9m113_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	int err = 0;

	return err;
}

static int mt9m113_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	int err = 0;

	return err;
}

static int mt9m113_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m113_state *state = to_state(sd);
	struct mt9m113_userset userset = state->userset;
	int err = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ctrl->value = userset.exposure_bias;
		err = 0;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->value = userset.auto_wb;
		err = 0;
		break;
	case V4L2_CID_WHITE_BALANCE_PRESET:
		ctrl->value = userset.manual_wb;
		err = 0;
		break;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		ctrl->value = userset.wb_temp;
		err = 0;
		break;
	case V4L2_CID_COLORFX:
		ctrl->value = userset.effect;
		err = 0;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = userset.contrast;
		err = 0;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = userset.saturation;
		err = 0;
		break;
	case V4L2_CID_SHARPNESS:
		ctrl->value = userset.saturation;
		err = 0;
		break;
	default:
		dev_err(&client->dev, "%s: no such ctrl\n", __func__);
		break;
	}

	return err;
}

static int mt9m113_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
#ifdef MT9M113_COMPLETE
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m113_state *state = to_state(sd);
	struct mt9m113_userset userset = state->userset;
	int err = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_CAM_PREVIEW_ONOFF:
		if (ctrl->value)
			err = mt9m113_set_preview_start(sd);
		else
			err = mt9m113_set_preview_stop(sd);
		mt9m113_info(&client->dev, "V4L2_CID_CAM_PREVIEW_ONOFF [%d] \n", ctrl->value);
		break;
		
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "%s: V4L2_CID_EXPOSURE\n", \
			__func__);
		//err = mt9m113_write_regs(sd, mt9m113_regs_ev_bias[ctrl->value]);
		break;
#if 0
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&client->dev, "%s: V4L2_CID_AUTO_WHITE_BALANCE\n", \
			__func__);
		//err = mt9m113_write_regs(sd, \
		//	mt9m113_regs_awb_enable[ctrl->value]);
		break;
	case V4L2_CID_WHITE_BALANCE_PRESET:
		dev_dbg(&client->dev, "%s: V4L2_CID_WHITE_BALANCE_PRESET\n", \
			__func__);
		err = mt9m113_write_regs(sd, \
			mt9m113_regs_wb_preset[ctrl->value]);
		break;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		dev_dbg(&client->dev, \
			"%s: V4L2_CID_WHITE_BALANCE_TEMPERATURE\n", __func__);
		err = mt9m113_write_regs(sd, \
			mt9m113_regs_wb_temperature[ctrl->value]);
		break;
	case V4L2_CID_COLORFX:
		dev_dbg(&client->dev, "%s: V4L2_CID_COLORFX\n", __func__);
		err = mt9m113_write_regs(sd, \
			mt9m113_regs_color_effect[ctrl->value]);
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&client->dev, "%s: V4L2_CID_CONTRAST\n", __func__);
		err = mt9m113_write_regs(sd, \
			mt9m113_regs_contrast_bias[ctrl->value]);
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&client->dev, "%s: V4L2_CID_SATURATION\n", __func__);
		err = mt9m113_write_regs(sd, \
			mt9m113_regs_saturation_bias[ctrl->value]);
		break;
	case V4L2_CID_SHARPNESS:
		dev_dbg(&client->dev, "%s: V4L2_CID_SHARPNESS\n", __func__);
		err = mt9m113_write_regs(sd, \
			mt9m113_regs_sharpness_bias[ctrl->value]);
		break;
#endif		
	default:
		dev_err(&client->dev, "%s: no such control\n", __func__);
		break;
	}

	if (err < 0)
		goto out;
	else
		return 0;

out:
	dev_dbg(&client->dev, "%s: vidioc_s_ctrl failed\n", __func__);
	return err;
#else
	return 0;
#endif
}



static int mt9m113_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;

	v4l_info(client, "%s: camera initialization start\n", __func__);


	err = mt9m113_i2c_write_regs(sd, \
				&mt9m113_init_regs[0], ARRAY_SIZE(mt9m113_init_regs));
    msleep(20);

	if (err < 0) {
		v4l_err(client, "%s: camera initialization failed\n", \
			__func__);
		return -EIO;	/* FIXME */
	}
	v4l_info(client, "%s: camera initialization end\n", __func__);

	return 0;
}

static int mt9m113_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m113_state *state = to_state(sd);
	struct mt9m113_mbus_platform_data *pdata = state->pdata;
	int ret;

	/* bug report */
	BUG_ON(!pdata);
	if(pdata->set_clock) {
		ret = pdata->set_clock(&client->dev, on);
		if(ret)
			return -EIO;
	}

	/* setting power */
	if(pdata->set_power) {
		ret = pdata->set_power(on);
		if(ret)
			return -EIO;
		if(on)
			return mt9m113_init(sd, 0);
	}

	return 0;
}

static int mt9m113_sleep(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL, i;

	v4l_info(client, "%s: sleep mode\n", __func__);
#if 0
	for (i = 0; i < MT9M113_SLEEP_REGS; i++) {
		if (mt9m113_sleep_reg[i][0] == REG_DELAY) {
			mdelay(mt9m113_sleep_reg[i][1]);
			err = 0;
		} else {
			err = mt9m113_write(sd, mt9m113_sleep_reg[i][0], \
				mt9m113_sleep_reg[i][1]);
		}

		if (err < 0)
			v4l_info(client, "%s: register set failed\n", __func__);
	}

	if (err < 0) {
		v4l_err(client, "%s: sleep failed\n", __func__);
		return -EIO;
	}
#endif 
	return 0;
}

static int mt9m113_wakeup(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL, i;

	v4l_info(client, "%s: wakeup mode\n", __func__);
#if 0

	for (i = 0; i < MT9M113_WAKEUP_REGS; i++) {
		if (mt9m113_wakeup_reg[i][0] == REG_DELAY) {
			mdelay(mt9m113_wakeup_reg[i][1]);
			err = 0;
		} else {
			err = mt9m113_write(sd, mt9m113_wakeup_reg[i][0], \
				mt9m113_wakeup_reg[i][1]);
		}

		if (err < 0)
			v4l_info(client, "%s: register set failed\n", __func__);
	}

	if (err < 0) {
		v4l_err(client, "%s: wake up failed\n", __func__);
		return -EIO;
	}
#endif
	return 0;
}

static int mt9m113_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt9m113_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	int err=0;
	printk("%s %d \n",__func__,enable);
#if 0
	switch (enable) {
		case STREAM_MODE_CAM_ON:
			printk("%s STREAM_MODE_CAM_ON\n",__func__);
			// 

			err = __mt9m113_init_2byte(sd, \
				(unsigned short *) Viewfinder_ON, MT9M113_Viewfinder_ON);
			if (err < 0) {
				v4l_err(client, "%s: camera i2c setting failed\n", \
					__func__);
				return -EIO;	/* FIXME */
			}

			break;
		case STREAM_MODE_CAM_OFF:
			printk("%s STREAM_MODE_CAM_OFF\n",__func__);
			err = __mt9m113_init_2byte(sd, \
				(unsigned short *) Viewfinder_OFF, MT9M113_Viewfinder_OFF);
			if (err < 0) {
				v4l_err(client, "%s: camera i2c setting failed\n", \
					__func__);
				return -EIO;	/* FIXME */
			}
			break;
		default:
			
			break;
		}
#endif

return err;
//	return enable ? mt9m113_wakeup(sd) : mt9m113_sleep(sd);

}

static const struct v4l2_subdev_core_ops mt9m113_core_ops = {
	.init = mt9m113_init,	/* initializing API */
	.s_power = mt9m113_s_power,
	.queryctrl = mt9m113_queryctrl,
	.querymenu = mt9m113_querymenu,
	.g_ctrl = mt9m113_g_ctrl,
	.s_ctrl = mt9m113_s_ctrl,
};

static const struct v4l2_subdev_video_ops mt9m113_video_ops = {
	.s_crystal_freq = mt9m113_s_crystal_freq,
	.g_mbus_fmt = mt9m113_g_fmt,
	.s_mbus_fmt = mt9m113_s_fmt,
	.enum_framesizes = mt9m113_enum_framesizes,
	.enum_frameintervals = mt9m113_enum_frameintervals,
	.g_parm = mt9m113_g_parm,
	.s_parm = mt9m113_s_parm,
	.s_stream = mt9m113_s_stream,
};

static const struct v4l2_subdev_ops mt9m113_ops = {
	.core = &mt9m113_core_ops,
	.video = &mt9m113_video_ops,
};

/*
 * mt9m113_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int mt9m113_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct mt9m113_state *state;
	struct v4l2_subdev *sd;
#if	0
	struct mt9m113_mbus_platform_data *pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err( &client->dev, "null platform data");
		return -EIO;
	}
#endif
	state = kzalloc(sizeof(struct mt9m113_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, MT9M113_DRIVER_NAME);
	//state->pdata = client->dev.platform_data;

	/* set default data from sensor specific value */
#if 1
	state->fmt.width = 640;
	state->fmt.height =480;
	state->runmode = MT9M113_RUNMODE_NOTREADY;
	state->default_width =  -1;
	state->default_height = -1;
	state->capture_width =  -1;
	state->capture_height = -1;
#else
	state->fmt.width = pdata->fmt.width;
	state->fmt.height = pdata->fmt.height;
#endif
	printk(" mt9m113_probe state->fmt.width =%d state->fmt.height=%d \n",state->fmt.width ,state->fmt.height);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &mt9m113_ops);

	/* needed for acquiring subdevice by this module name */
	snprintf(sd->name, sizeof(sd->name), MT9M113_DRIVER_NAME);

	dev_info(&client->dev, "mt9m113 has been probed\n");

	return 0;
}


static int mt9m113_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id mt9m113_id[] = {
	{ MT9M113_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mt9m113_id);

static struct i2c_driver mt9m113_i2c_driver = {
	.driver = {
		.name	= MT9M113_DRIVER_NAME,
	},
	.probe		= mt9m113_probe,
	.remove		= mt9m113_remove,
	.id_table	= mt9m113_id,
};

static int __init mt9m113_mod_init(void)
{
	return i2c_add_driver(&mt9m113_i2c_driver);
}

static void __exit mt9m113_mod_exit(void)
{
	i2c_del_driver(&mt9m113_i2c_driver);
}
module_init(mt9m113_mod_init);
module_exit(mt9m113_mod_exit);

MODULE_DESCRIPTION("Samsung Electronics MT9M113 SXGA camera driver");
MODULE_AUTHOR("Dongsoo Nathaniel Kim<dongsoo45.kim@samsung.com>");
MODULE_LICENSE("GPL");


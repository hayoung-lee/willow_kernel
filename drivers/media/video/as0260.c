/* linux/drivers/media/video/as0260.c
 *
 *
 * Driver for as0260 (SXGA camera) from ThinkWare
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
#include <media/as0260_platform.h>

#include <linux/videodev2_samsung.h>

#include "as0260.h"

#define AS0260_DEBUG
#ifdef AS0260_DEBUG
#define as0260_info dev_info
#define as0260_err dev_err
#else
#define as0260_info(dev, format, arg...) do { } while (0)
#define as0260_err dev_err
#endif

/* Default resolution & pixelformat. plz ref as0260_platform.h */
#define DEFAULT_RES		WVGA	/* Index of resoultion */
#define DEFAUT_FPS_INDEX	as0260_15FPS
#define DEFAULT_FMT		V4L2_PIX_FMT_UYVY	/* YUV422 */

#define as0260_JPEG_MAXSIZE	0x3A0000
#define as0260_THUMB_MAXSIZE	0xFC00
#define as0260_POST_MAXSIZE	0xBB800

//extern int as0260_stanby(void);
extern int willow_capture_status;

struct i2c_client *backup_client;
	

/* Camera functional setting values configured by user concept */
struct as0260_userset {
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

struct as0260_state {
	struct as0260_mbus_platform_data *pdata;
	struct v4l2_subdev sd;
	enum as0260_runmode runmode;
	struct v4l2_mbus_framefmt 	fmt;
	struct v4l2_pix_format req_fmt;
	struct v4l2_pix_format set_fmt;

	struct v4l2_fract 		timeperframe;
	struct as0260_userset 		userset;
	int 				freq;	/* MCLK in KHz */
	int 				is_mipi;
	int 				isize;
	int 				ver;
	int 				fps;
	int 				fmt_index;
	unsigned short 			devid_mask;
	
	int framesize_index;
	unsigned int pixelformat;
	int default_width;
	int default_height;
	int capture_width;
	int capture_height;
	int set_vhflip;
	int check_dataline;	
};

struct as0260_enum_framesize {
        int index;
        unsigned int width;
        unsigned int height;
};

enum AS0260_frame_size {
        AS0260_PREVIEW_QVGA = 0,      /* 320x240 */
        AS0260_PREVIEW_VGA,            /* 640x480 */
        AS0260_PREVIEW_1M,          /* 1280x720 */
        AS0260_PREVIEW_1P3M,          /* 1280x960 */
        AS0260_PREVIEW_2M,         /* 1920x1280*/
};

static struct as0260_enum_framesize as0260_framesize_list[] = {
        {AS0260_PREVIEW_QVGA,         320,  240 },
        {AS0260_PREVIEW_VGA,         640,  480 },
        {AS0260_PREVIEW_1M,         1280, 720 },
        {AS0260_PREVIEW_1P3M,         1280, 960 },
        {AS0260_PREVIEW_2M,         1920, 1080 },
};

static DEFINE_MUTEX(buf_lock);

static int as0260_i2c_write_reg_8(struct i2c_client *client,
		u16 reg_address,
		u8 value)
{
	int ret;
	u8 buf[3];
	mutex_lock(&buf_lock);

	buf[0] = (reg_address >> 8) & 0xFF;
	buf[1] = reg_address & 0xFF;
	buf[2] = value;

	ret = i2c_master_send(client, buf, 3);
	mutex_unlock(&buf_lock);

	return ret;
}

static int as0260_i2c_write_reg_16(struct i2c_client *client,
		u16 reg_address,
		u16 value)
{
	int ret;
	u8 buf[4];
	mutex_lock(&buf_lock);
	
	buf[0] = (reg_address >> 8) & 0xFF;
	buf[1] = reg_address & 0xFF;
	buf[2] = (value >> 8) & 0xFF;
	buf[3] = value & 0xFF;

	ret = i2c_master_send(client, buf, 4);
	mutex_unlock(&buf_lock);

	return ret;
}
#if defined(AS0260_NOT_USE)

static int as0260_i2c_write_reg_24(struct i2c_client *client,
		u16 reg_address,
		u32 value)
{
	int ret;
	u8 buf[5];

	mutex_lock(&buf_lock);

	buf[0] = (reg_address >> 8) & 0xFF;
	buf[1] = reg_address & 0xFF;
	buf[2] = (value >> 16) & 0xFF;
	buf[3] = (value >> 8) & 0xFF;
	buf[4] = value & 0xFF;

	ret = i2c_master_send(client, buf, 5);
	mutex_unlock(&buf_lock);

	return ret;
}
#endif
static int as0260_i2c_write_reg_32(struct i2c_client *client,
		u16 reg_address,
		u32 value)
{
	int ret;
	u8 buf[6];
	mutex_lock(&buf_lock);

	buf[0] = (reg_address >> 8) & 0xFF;
	buf[1] = reg_address & 0xFF;
	buf[2] = (value >> 24) & 0xFF;
	buf[3] = (value >> 16) & 0xFF;
	buf[4] = (value >> 8) & 0xFF;
	buf[5] = value & 0xFF;
	ret = i2c_master_send(client, buf, 6);
	mutex_unlock(&buf_lock);
	

	return ret;
}
#if defined(AS0260_NOT_USE)
static int as0260_i2c_read_reg_8(struct i2c_client *client,
		u16 reg_address,
		u8 *val)
{
	int ret;
	u8 tbuf[2];
	u8 rbuf[1];
	
	mutex_lock(&buf_lock);
	tbuf[0] = (reg_address >> 8) & 0xFF;
	tbuf[1] = reg_address & 0xFF;

	ret = i2c_master_send(client,tbuf, 2);
	if (ret)
		goto out;

	ret = i2c_master_recv(client,rbuf, 1);
	if (ret)
		goto out;

	*val = rbuf[0];

out:
	printk("as0260_i2c_read_reg_8 err \n");
	mutex_unlock(&buf_lock);
	return ret;
}

static int as0260_i2c_read_reg_16(struct i2c_client *client,
		u16 reg_address,
		u16 *val)
{
	int ret;
	u8 tbuf[2];
	u8 rbuf[2];

	mutex_lock(&buf_lock);
	tbuf[0] = (reg_address >> 8) & 0xFF;
	tbuf[1] = reg_address & 0xFF;

	ret = i2c_master_send(client, tbuf, 2);
	if (ret)
		goto out;

	ret = i2c_master_recv(client, rbuf, 2);
	if (ret)
		goto out;

	*val = (rbuf[0] << 8) | rbuf[1];
out:
	printk("as0260_i2c_read_reg_8 err \n");
	mutex_unlock(&buf_lock);
	
	return ret;
}

static int as0260_i2c_read_reg_24(struct i2c_client *client,
		u16 reg_address,
		u32 *val)
{

	int ret;
	u8 tbuf[2];
	u8 rbuf[3];

	mutex_lock(&buf_lock);
	tbuf[0] = (reg_address >> 8) & 0xFF;
	tbuf[1] = reg_address & 0xFF;

	ret = i2c_master_send(client, tbuf, 2);
	if (ret)
		goto out;

	ret = i2c_master_recv(client,rbuf, 3);
	if (ret)
		goto out;

	*val = (rbuf[0] << 16) | (rbuf[1] << 8) |rbuf[2];
out:
	printk("as0260_i2c_read_reg_8 err \n");
	mutex_unlock(&buf_lock);
	return ret;
}

static int as0260_i2c_read_reg_32(struct i2c_client *client,
		u16 reg_address,
		u32 *val)
{
	int ret;
	u8 tbuf[2];
	u8 rbuf[4];
	
	mutex_lock(&buf_lock);
	tbuf[0] = (reg_address >> 8) & 0xFF;
	tbuf[1] = reg_address & 0xFF;

	ret = i2c_master_send(client, tbuf, 2);
	if (ret)
		goto out;

	ret = i2c_master_recv(client,rbuf, 4);
	if (ret)
		goto out;

	*val = (rbuf[0] << 24) | (rbuf[1] << 16) | (rbuf[2] << 8) | rbuf[3];
out:
	printk("as0260_i2c_read_reg_8 err \n");
	mutex_unlock(&buf_lock);
	return ret;
}
#endif

static inline struct as0260_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct as0260_state, sd);
}

static int as0260_i2c_write(struct v4l2_subdev *sd, unsigned char *txdata, int length)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[] = {
		{
			//.addr = client->addr >> 1,
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	int I2C_RETRY = 0;

	for (; I2C_RETRY < 2; I2C_RETRY++) {
		if (i2c_transfer(client->adapter, msg, 1) < 0) {
			if (1 == I2C_RETRY) {
				as0260_err(&client->dev, "%s: failed to transfer to addr:0x%x\n", __func__, client->addr);
				return -EIO;
			}
			mdelay(10);
		} else {
			break;
		}
	}

	return 0;
}

int as0260_32_write_reg(struct v4l2_subdev *sd,u16 cmd, u32 val)
{
	unsigned char buf[4];
	int err=0;
	buf[0] = (cmd& 0xFF00) >> 8;
	buf[1] = (cmd & 0x00FF);			
	buf[2] = (val & 0xFF000000) >> 24;
	buf[3] = (val & 0x00FF0000) >> 16;
	//printk("[AS0260] ____init Reg cmd =%x %x data %x %x \n", buf[0],buf[1],buf[2],buf[3]);
	err = as0260_i2c_write(sd, buf, 4);

	buf[0] = (cmd& 0xFF00) >> 8;
	buf[1] = (cmd & 0x00FF)+2;			
	buf[2] = (val & 0x0000FF00) >> 8;
	buf[3] = (val & 0x000000FF);
	err = as0260_i2c_write(sd, buf, 4);
	//printk("[AS0260] ____init Reg cmd =%x %x data %x %x \n", buf[0],buf[1],buf[2],buf[3]);
	return 0;
}


static int as0260_i2c_w_write_regs(struct v4l2_subdev *sd,
	struct as0260_reg const *reg_tbl, int num_of_items_in_table) {
	int i;
	int err = -EIO;

	struct i2c_client *client = v4l2_get_subdevdata(sd);

	for (i = 0; i < num_of_items_in_table; i++) {
		if (reg_tbl->waddr ==0xffff)
			mdelay(reg_tbl->wdata);
		else
		{

			if(reg_tbl->len==4) {
				printk("as0260_i2c_w_write_regs 32bit reg add=%x data=%x \n", reg_tbl->waddr, reg_tbl->wdata);
				err=as0260_i2c_write_reg_32(client, reg_tbl->waddr, reg_tbl->wdata);
			} else {
				if(reg_tbl->len==2){
					err=as0260_i2c_write_reg_16(client, reg_tbl->waddr, reg_tbl->wdata);
				} else {
					err=as0260_i2c_write_reg_8(client, reg_tbl->waddr, reg_tbl->wdata);
				}
			}
			if (err < 0)
				break;
		}

		reg_tbl++;
	}

	return err;
}



static inline int as0260_i2c_32_read(struct i2c_client *client,
        u16 subaddr,  u32 *data)
{
	unsigned char buf[4];
	struct i2c_msg msg = {client->addr, 0, 2, buf};

	int err = 0;

	if (!client->adapter)
	{
		dev_err(&client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	buf[0] = subaddr>>8;
	buf[1] = subaddr &0xff;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
		dev_err(&client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	msg.flags = I2C_M_RD;
	msg.len = 4;
	memset(buf, 0, sizeof(buf));
	
	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
		//printk(&client->dev, "%s: %d register read fail\ n", __func__, __LINE__);
		return -EIO;
	}

	*data = ((buf[0] << 24) | (buf[1] << 16) | (buf[2] <<8) | buf[3] <<0);
	//printk("as0260_i2c_32_read buf[0]=%x buf[1]=%x buf[2]=%x,buf[3]=%x \n", buf[0] ,buf[1],buf[2],buf[3]);
	return err;
}

static inline int as0260_i2c_16_read(struct i2c_client *client,
        u16 regadd, u16 *data)
{
	unsigned char buf[2];
	struct i2c_msg msg = {client->addr, 0, 2, buf};

	int err = 0;

	if (!client->adapter)
	{
		dev_err(&client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	buf[0] = regadd>>8;
	buf[1] = regadd &0xff;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
		dev_err(&client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	msg.flags = I2C_M_RD;
	msg.len = 2;
	buf[0] = 0;
	buf[1] = 0;
	
	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
		//printk(&client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	*data = ((buf[0] << 8) | (buf[1] ));
	//printk("as0260_i2c_16_read buf[0]=%x buf[1]=%x \n", buf[0] ,buf[1]);
	return err;
}

static inline int as0260_i2c_8_read(struct i2c_client *client,
        u16 subaddr,u8 *data)
{
	unsigned char buf[2];
	struct i2c_msg msg = {client->addr, 0, 2, buf};

	int err = 0;

	if (!client->adapter)
	{
		dev_err(&client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	buf[0] = subaddr>>8;
	buf[1] = subaddr &0xff;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
		dev_err(&client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	msg.flags = I2C_M_RD;
	msg.len = 1;

	buf[0] = 0;
	buf[1] = 0;
	
	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
		//printk(&client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	*data = buf[0] ;
	//printk("as0260_i2c_8_read buf[0]=%x \n", buf[0]);
	return err;
}

#if 0 // Test code
static int as0260_i2c_test_read_regs(struct v4l2_subdev *sd,
	struct as0260_reg const *reg_tbl, int num_of_items_in_table) {
	int i;
	int err = -EIO;
	u8 buf8=0;
	u16 buf16=0;
	u32 buf32=0;
	
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	for (i = 0; i < num_of_items_in_table; i++) {
		if (reg_tbl->waddr ==0xffff)
			mdelay(reg_tbl->wdata);
		else
		{
			if(reg_tbl->waddr !=0x098E)
			{	
				printk("[AS0260] as0260_i2c_test_read_regs address=%x  data=", reg_tbl->waddr);
				if(reg_tbl->len==4) {
					err=	as0260_i2c_32_read(client, reg_tbl->waddr,&buf32);
					
					if( reg_tbl->wdata==buf32)
						printk("%x  read data=%x OK \n", reg_tbl->wdata, buf32);
					else
						printk("%x  read data=%x FAIL \n", reg_tbl->wdata, buf32);				
				} else {
					if(reg_tbl->len==2){
						err=	as0260_i2c_16_read(client, reg_tbl->waddr,&buf16);
						if( reg_tbl->wdata==buf16)
							printk("%x  read data=%x OK \n", reg_tbl->wdata, buf16);
						else
							printk("%x  read data=%x FAIL \n", reg_tbl->wdata, buf16);				

					} else {
						err=	as0260_i2c_8_read(client, reg_tbl->waddr,&buf8);
						if( reg_tbl->wdata==buf8)
							printk("%x  read data=%x OK \n", reg_tbl->wdata, buf8);
						else
							printk("%x  read data=%x FAIL \n", reg_tbl->wdata, buf8);	
					}
				}
				if (err < 0)
					break;
			}
		}

		reg_tbl++;
	}

	return err;
}
#endif

int issueCommand(struct v4l2_subdev *sd, const HOST_COMMAND_E command)
{
	int cnt=0;
	u16 buf=0;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

	// issue the command (and reset the OK bit)
	as0260_i2c_write_reg_16(client, COMMAND_REG, (command | HC_OK));
	// and wait for command to complete
	cnt=0;
	while (true)
	{
		as0260_i2c_16_read(client,0x0080,&buf);
		if (0 == (buf & command))
		{
			printk("issue command return =%d\n",buf);

			return (buf & HC_OK);
		}
		if(cnt++>100)
		{
			printk("issue command time out \n");
			break;
		}
		else
		msleep(10);	
	}
	return -1;
}

int checkCommand(struct v4l2_subdev *sd, u16 cmd)
{
	int cnt=0;
	u16 buf=0;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

	// issue the command (and reset the OK bit)
	as0260_i2c_write_reg_16(client, COMMAND_REG, cmd);
	// and wait for command to complete
	cnt=0;
	while (true)
	{
		as0260_i2c_16_read(client,0x0080,&buf);
		if(cmd==0xfff0 || cmd==0xfff1){
			if(buf==0xfff0)
			{
				printk("checkCommand return =%d\n",buf);
				return 1;
			}

		}else{
		
			if (0 == (buf & cmd))
			{
				printk("checkCommand return =%d\n",buf);

				return (buf & HC_OK);
			}
		}
		
		if(cnt++>100)
		{
			printk("issue command time out \n");
			break;
		}
		else
		msleep(10);	
	}
	return -1;
}

SYSTEM_STATE_E getSystemState(struct v4l2_subdev *sd)
{
	//unsigned long read_value=0;
	int err = -EINVAL;
	u8 buf=0;
	//u16 cmd=0;
	
    struct i2c_client *client = v4l2_get_subdevdata(sd);

	err=as0260_i2c_8_read(client,0xdc01,&buf);
	printk("getSystemState ______ AS0260 current state =0x%x\n",buf);

	switch(buf)
	{
		case 0x28:
		printk("getSystemState ______ AS0260 current state =SS_ENTER_CONFIG_CHANGE\n");
		break;
		case 0x31:
		printk("getSystemState ______ AS0260 current state =SS_STREAMING\n");
		break;
		case 0x34:
		printk("getSystemState ______ AS0260 current state =SS_START_STREAMING\n");
		break;
		case 0x40:
		printk("getSystemState ______ AS0260 current state =SS_ENTER_SUSPEND\n");
		break;
		case 0x41:
		printk("getSystemState ______ AS0260 current state =SS_SUSPENDED\n");
		break;
		case 0x50:
		printk("getSystemState ______ AS0260 current state =SS_ENTER_STANDBY\n");
		break;
		case 0x52:
		printk("getSystemState ______ AS0260 current state =STANDBY\n");
		break;		
		case 0x54:
		printk("getSystemState ______ AS0260 current state =SS_LEAVE_STANDBY\n");
		break;		
		default:
		printk("getSystemState ______ AS0260 current state =unknown \n");
		break;
	}
	return (SYSTEM_STATE_E)buf;
}


int changestate(struct v4l2_subdev *sd,SYSTEM_STATE_E state_d, HOST_COMMAND_E cmd_d)
{
	//int res;
	int err = -EINVAL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	u16 buf16=0;
	int i=0;

	err=as0260_i2c_write_reg_8(client, 0xdc00, state_d);  
	err=as0260_i2c_write_reg_16(client, 0x0080, 0x8000 | cmd_d);  
	do {
		as0260_i2c_16_read(client, 0x0080,&buf16);
		if (0 == (buf16 & cmd_d))
		{
			printk("issue command return =%x \n",buf16);
			return 1;		
		}
		i++;		
		msleep(10);
	}while(i++<10);

	return 0;	
}

int changeConfig(struct v4l2_subdev *sd)
{
	int res;
	int err = -EINVAL;

#if 1
	err =as0260_i2c_w_write_regs(sd,&as0260_configChange_regs[0], ARRAY_SIZE(as0260_configChange_regs));

	res = issueCommand(sd,HC_SET_STATE);

	if (res>0)
	{
		printk("setSystemState error \n");
		return -1;
	}
	else
	return getSystemState(sd);

#else
	u16 buf16=0;
	int i=0;

	err=as0260_i2c_write_reg_16(client, 0xdc00, 0x0028);  
	err=as0260_i2c_write_reg_16(client, 0x0080, 0x8002);  
	do {
		as0260_i2c_16_read(client, 0x0080,&buf16);
		printk("as0260_init ______ AS0260 0x0018 standby_control_and_status=0x%x\n",buf16);
		i++;
		msleep(10);
	}while(i++<5);

#endif
	
}


// requests system enter standby
int enterStandby(struct v4l2_subdev *sd)
{
	int res;
	int err = -EINVAL;
	
	err =as0260_i2c_w_write_regs(sd,&as0260_enterStanby_regs[0], ARRAY_SIZE(as0260_enterStanby_regs));

	res = issueCommand(sd,HC_SET_STATE);

	if (res>0)
	{
		printk("setSystemState error \n");
		return -1;
	}
	else
	return getSystemState(sd);
}

int startStreaming(struct v4l2_subdev *sd)
{
	int res;
	int err = -EINVAL;
	err =as0260_i2c_w_write_regs(sd,&as0260_enterStreaming_regs[0], ARRAY_SIZE(as0260_enterStreaming_regs));

	res = issueCommand(sd,HC_SET_STATE);

	if (res>0)
	{
		printk("AS0260_________ startStreaming error \n");
		return -1;
	}
	else
	return getSystemState(sd);
}

	
// requests system leave standby
int leaveStandby(struct v4l2_subdev *sd)
{
	int res;
	int err = -EINVAL;
	
	err =as0260_i2c_w_write_regs(sd,&as0260_exitStanby_regs[0], ARRAY_SIZE(as0260_exitStanby_regs));

	res = issueCommand(sd,HC_SET_STATE);

	if (res>0)
	{
		printk("setSystemState error \n");
		return -1;
	}
	else
	return getSystemState(sd);
}

// requests a Refresh operation
int refresh(struct v4l2_subdev *sd)
{
    //struct i2c_client *client = v4l2_get_subdevdata(sd);

	int res;
	res = issueCommand(sd,HC_REFRESH);
	if (res==-1){
		printk("[AS0260] Refresh .. error \n");
		return -1;
	}else{
		printk("[AS0260] Refresh .. OK \n");
		return 1;
	}
}

static const char *as0260_querymenu_wb_preset[] = {
	"WB Tungsten", "WB Fluorescent", "WB sunny", "WB cloudy", NULL
};

static const char *as0260_querymenu_effect_mode[] = {
	"Effect Sepia", "Effect Aqua", "Effect Monochrome",
	"Effect Negative", "Effect Sketch", NULL
};

static const char *as0260_querymenu_ev_bias_mode[] = {
	"-3EV",	"-2,1/2EV", "-2EV", "-1,1/2EV",
	"-1EV", "-1/2EV", "0", "1/2EV",
	"1EV", "1,1/2EV", "2EV", "2,1/2EV",
	"3EV", NULL
};

static struct v4l2_queryctrl as0260_controls[] = {
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
		.maximum = ARRAY_SIZE(as0260_querymenu_wb_preset) - 2,
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
		.maximum = ARRAY_SIZE(as0260_querymenu_ev_bias_mode) - 2,
		.step = 1,
		.default_value = (ARRAY_SIZE(as0260_querymenu_ev_bias_mode) \
				- 2) / 2,	/* 0 EV */
	},
	{
		.id = V4L2_CID_COLORFX,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Image Effect",
		.minimum = 0,
		.maximum = ARRAY_SIZE(as0260_querymenu_effect_mode) - 2,
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


static ssize_t as0260_regread_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	u16 wmreg=0 ,regvalue=0;
	u8 valbuf[6];
	u32 regvalue32=0;
	
	memset(valbuf, 0, sizeof(valbuf));

	num_read_chars = count - 1;
	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, (long *)&wmreg);
	//printk("valbuf=%s wmreg=%x\n", valbuf, wmreg);
	//if (0 != retval)
	{
		if(wmreg==0xc808 || wmreg==0xE004)
		{
			as0260_i2c_32_read(client, wmreg,&regvalue32);
			//num_read_chars = snprintf(buf, PAGE_SIZE, "0x%X\n", regvalue32);
			printk("[AS0260] as0260_read reg=0x%x  val=%x \n", wmreg,regvalue32);
		}
		else
		{
			as0260_i2c_16_read(client, wmreg,&regvalue);
			//num_read_chars = snprintf(buf, PAGE_SIZE, "0x%X\n", regvalue);
			printk("[AS0260] as0260_read reg=0x%x  val=%x \n", wmreg,regvalue);
		}
	}

	return num_read_chars;
}

static ssize_t as0260_regread_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	//ssize_t num_read_chars = 0;
	u32 read_value=0;
	//int err=0;

	as0260_i2c_32_read(client, 0x0000,&read_value);
#if 0
	if(err < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "[AS0260] Read Firmware Version Fail l!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "0x%X\n", read_value);
#endif
	printk("[AS0260] Firmware Version =0x%x \n", read_value);
	return sprintf(buf, "0x%x\n", read_value);;
}


static ssize_t as0260_regwrite_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	u16 wmreg=0 ,regvalue=0;
	u32 tmpreg=0;
	u8 regvalue1=0;
	
	//mutex_lock(&buf_lock);
	num_read_chars = count - 1;
	if(num_read_chars>=8)
	{
	 	retval = strict_strtoul(buf, 16, (long *)&tmpreg);
		wmreg=(tmpreg&0xffff0000) >>16;
		regvalue=(tmpreg&0xffff);
		printk("[AS0260] ================================ \n");		
		printk("[as0260] write..  reg=%x value=%x \n", wmreg,regvalue);
		//if (0 != retval)
		{

			if(regvalue>0xff){
				as0260_i2c_write_reg_16(client, wmreg,regvalue);
				regvalue=0;
				as0260_i2c_16_read(client, wmreg,&regvalue);			

			}else{
				as0260_i2c_write_reg_8(client, wmreg,(u8)regvalue);
				regvalue1=0;
				as0260_i2c_8_read(client, wmreg,&regvalue1);			
			}
			//num_read_chars = snprintf(buf, PAGE_SIZE, "0x%X\n", regvalue);
			printk("[AS0260] read  reg=0x%x  val=%x \n", wmreg,regvalue);
			printk("[AS0260] ================================ \n");
		}
		//mutex_unlock(&buf_lock);
	}
	return count;
}

static DEVICE_ATTR(regread, S_IRUGO|S_IWUSR, as0260_regread_show, as0260_regread_store);
static DEVICE_ATTR(regwrite, S_IRUGO|S_IWUSR, NULL, as0260_regwrite_store);

static struct attribute *as0260_attributes[] = {
	&dev_attr_regread.attr,
	&dev_attr_regwrite.attr,
	NULL
};

static struct attribute_group as0260_attribute_group = {
	.attrs = as0260_attributes
};


static int as0260_set_resolution_reg(struct v4l2_subdev *sd,int index)
{
	int err=0;
	switch(index)
	{
		case AS0260_PREVIEW_QVGA:
			err =as0260_i2c_w_write_regs(sd,&as0260_320p30_regs[0], ARRAY_SIZE(as0260_320p30_regs));
			break;
		case AS0260_PREVIEW_VGA:
			err =as0260_i2c_w_write_regs(sd,&as0260_640p30_regs[0], ARRAY_SIZE(as0260_640p30_regs));
			break;	
		case AS0260_PREVIEW_1M:
			err =as0260_i2c_w_write_regs(sd,&as0260_720p30_regs[0], ARRAY_SIZE(as0260_720p30_regs));
			
		case AS0260_PREVIEW_1P3M:
			err =as0260_i2c_w_write_regs(sd,&as0260_960p30_regs[0], ARRAY_SIZE(as0260_960p30_regs));
			break;		
		case AS0260_PREVIEW_2M:
			err =as0260_i2c_w_write_regs(sd,&as0260_1080p30_regs[0], ARRAY_SIZE(as0260_640p30_regs));
			break;		
	}

	return err;
}
static int as0260_set_preview_start(struct v4l2_subdev *sd)
{
	int err = 0;
#if 1
#ifdef AS0260_DEBUG
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#endif
	struct as0260_state *state = to_state(sd);
	u32 read_value=0;

#ifdef AS0260_DEBUG
	as0260_info(&client->dev, "[AS0260] %s: ________set_Preview_start\n", __func__);
#endif

	as0260_set_resolution_reg(sd,state->framesize_index);
	err=changestate(sd,SS_ENTER_CONFIG_CHANGE,HC_SET_STATE);
	if(err==0)
	{
		as0260_set_resolution_reg(sd,state->framesize_index);
		msleep(10);
		err=changestate(sd,SS_ENTER_CONFIG_CHANGE,HC_SET_STATE);
	}
	printk("as0260_init ______ AS0260 0xc808 CAM_SENSOR_CFG_PIXCLK Ver =0x%x\n",read_value);
	
	state->runmode = as0260_RUNMODE_RUNNING;
#endif	
	return err;
}


static int as0260_set_preview_stop(struct v4l2_subdev *sd)
{
	struct as0260_state *state = to_state(sd);

	int err = 0;
	
	if (state->runmode == as0260_RUNMODE_RUNNING) {
		state->runmode = as0260_RUNMODE_IDLE;
		state->set_vhflip = 0;
	}

	return err;
}

const char * const *as0260_ctrl_get_menu(u32 id)
{
	switch (id) {
	case V4L2_CID_WHITE_BALANCE_PRESET:
		return as0260_querymenu_wb_preset;

	case V4L2_CID_COLORFX:
		return as0260_querymenu_effect_mode;

	case V4L2_CID_EXPOSURE:
		return as0260_querymenu_ev_bias_mode;

	default:
		return v4l2_ctrl_get_menu(id);
	}
}

static inline struct v4l2_queryctrl const *as0260_find_qctrl(int id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(as0260_controls); i++)
		if (as0260_controls[i].id == id)
			return &as0260_controls[i];

	return NULL;
}

static int as0260_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(as0260_controls); i++) {
		if (as0260_controls[i].id == qc->id) {
			memcpy(qc, &as0260_controls[i], \
				sizeof(struct v4l2_queryctrl));
			return 0;
		}
	}
	printk(" as0260_queryctrl error   \n");
	return -EINVAL;
}

static int as0260_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	struct v4l2_queryctrl qctrl;

	qctrl.id = qm->id;
	as0260_queryctrl(sd, &qctrl);

	return v4l2_ctrl_query_menu(qm, &qctrl, as0260_ctrl_get_menu(qm->id));
}

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 *	freq : in Hz
 *	flag : not supported for now
 */
static int as0260_s_crystal_freq(struct v4l2_subdev *sd, u32  freq, u32 flags)
{
	int err = -EINVAL;

	return err;
}

static int as0260_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	struct as0260_state *state = to_state(sd);
	int err = 0;

	*fmt = state->fmt;
	printk( "as0260_g_fmt state requested res(%d, %d)\n",
		state->fmt.width, state->fmt.height);
	return err;
}

static int as0260_set_capture_start(struct v4l2_subdev *sd)
{
	int err=0;
	struct as0260_state *state = to_state(sd);

	err=as0260_set_resolution_reg(sd,state->framesize_index);
	if(err==0)
	{
		as0260_set_resolution_reg(sd,state->framesize_index);
		msleep(10);
		err=changestate(sd,SS_ENTER_CONFIG_CHANGE,HC_SET_STATE);
	}
	//printk(" as0260_set_capture_start  state->req_fmt.width=%d state->req_fmt.height %d index \n",
	//	state->req_fmt.width,state->req_fmt.height ,state->framesize_index);

	return err;
}

static int as0260_set_effect(struct v4l2_subdev *sd,int value)
{
	int err=0;
	struct as0260_state *state = to_state(sd);

	switch(value)
	{
		case IMAGE_EFFECT_NONE:
		err =as0260_i2c_w_write_regs(sd,&as0260_effect_off_regs[0], ARRAY_SIZE(as0260_effect_off_regs));
		break;

		case IMAGE_EFFECT_BNW:
		err =as0260_i2c_w_write_regs(sd,&as0260_effect_mono_regs[0], ARRAY_SIZE(as0260_effect_mono_regs));
		break;
		
		case IMAGE_EFFECT_SEPIA:
		err =as0260_i2c_w_write_regs(sd,&as0260_effect_sepia_regs[0], ARRAY_SIZE(as0260_effect_sepia_regs));
		break;

		case IMAGE_EFFECT_NEGATIVE:
		err =as0260_i2c_w_write_regs(sd,&as0260_effect_Negative_regs[0], ARRAY_SIZE(as0260_effect_Negative_regs));
		break;

		case IMAGE_EFFECT_AQUA:
		err =as0260_i2c_w_write_regs(sd,&as0260_effect_solarize_regs[0], ARRAY_SIZE(as0260_effect_solarize_regs));
		break;
	}

	//as0260_i2c_32_read(client, 0xc808,&read_value);
	printk("as0260_set ______ AS0260 0xc808 as0260_set_effect =%d \n",value);

	changestate(sd,SS_ENTER_CONFIG_CHANGE,HC_REFRESH);	
	state->runmode = as0260_RUNMODE_RUNNING;

	return err;
}

static int as0260_set_brightness(struct v4l2_subdev *sd,int value)
{
	int err=0;
	struct as0260_state *state = to_state(sd);

	switch(value)
	{
		case EV_MINUS_2:
		err =as0260_i2c_w_write_regs(sd,&as0260_brightness_M02_regs[0], ARRAY_SIZE(as0260_brightness_M02_regs));
		break;

		case EV_MINUS_1:
		err =as0260_i2c_w_write_regs(sd,&as0260_brightness_M01_regs[0], ARRAY_SIZE(as0260_brightness_M01_regs));
		break;
		
		case EV_DEFAULT:
		err =as0260_i2c_w_write_regs(sd,&as0260_brightness_P00_regs[0], ARRAY_SIZE(as0260_brightness_P00_regs));
		break;

		case EV_PLUS_1:
		err =as0260_i2c_w_write_regs(sd,&as0260_brightness_P01_regs[0], ARRAY_SIZE(as0260_brightness_P01_regs));
		break;

		case EV_PLUS_2:
		err =as0260_i2c_w_write_regs(sd,&as0260_brightness_P02_regs[0], ARRAY_SIZE(as0260_brightness_P02_regs));
		break;
	}

	//as0260_i2c_32_read(client, 0xc808,&read_value);
	printk("as0260_set ______ AS0260 0xc808 as0260_set_brightness =%d \n",value);

	changestate(sd,SS_ENTER_CONFIG_CHANGE,HC_REFRESH);	
	state->runmode = as0260_RUNMODE_RUNNING;

	return err;
}

static int as0260_get_framesize_index(struct v4l2_subdev *sd)
{
        struct as0260_state *state = to_state(sd);
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct as0260_enum_framesize *frmsize;

        int i = 0;

        printk("%s: ___________________Requested Res: %d %d \n",
			__func__, state->req_fmt.width, state->req_fmt.height);

        /* Check for video/image mode */
        for(i = 0; i < (sizeof(as0260_framesize_list)/sizeof(struct as0260_enum_framesize)); i++)
        {
                frmsize = &as0260_framesize_list[i];

                /* In case of image capture mode, if the given image resolution is not supported,
                 * return the next higher image resolution. */
                //must search wide
                if(frmsize->width == state->req_fmt.width  && frmsize->height == state->req_fmt.height)
                 {
                        as0260_err(&client->dev, "%s: as0260_get_framesize_index  width %d  height=%d  index=%d \n", __func__,
                          frmsize->width, frmsize->height , frmsize->index);
                        return frmsize->index;

                 }
        }

        as0260_err(&client->dev, "%s:  don't chekc....  %d %d \n", __func__, state->req_fmt.width, state->req_fmt.height);
        /* If it fails, return the default value. */

	return 2; //VGA
}

static int as0260_set_framesize_index(struct v4l2_subdev *sd, unsigned int index)
{

        int i = 0;
        struct as0260_state *state = to_state(sd);
        struct i2c_client *client = v4l2_get_subdevdata(sd);

        /* Check for video/image mode */
        for(i = 0; i < (sizeof(as0260_framesize_list)/sizeof(struct as0260_enum_framesize)); i++)
        {
                if(as0260_framesize_list[i].index == index){
                        state->framesize_index = as0260_framesize_list[i].index;
                        state->req_fmt.width = as0260_framesize_list[i].width;
                        state->req_fmt.height = as0260_framesize_list[i].height;
                        as0260_err(&client->dev, "%s: Camera Res: %dx%d\n", __func__, state->req_fmt.width, state->req_fmt.height);
                        return 0;
                }
        }

        return -EINVAL;
}

static int as0260_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	struct as0260_state *state = to_state(sd);
	int err = 0;
	int framesize_index=0;
	
	printk( "as0260_s_fmt__________________ requested res(%d, %d)\n",
		fmt->width, fmt->height);
	printk( "as0260_s_fmt__________________ requested res(%d, %d)\n",fmt->width, fmt->height);


	state->req_fmt.width = fmt->width;
	state->req_fmt.height = fmt->height;

        framesize_index = as0260_get_framesize_index(sd);
        err = as0260_set_framesize_index(sd, framesize_index);

	//state->req_fmt.pixelformat = fmt->fmt.pix.pixelformat;
	state->req_fmt.colorspace = fmt->colorspace;
	state->req_fmt.pixelformat = DEFAULT_FMT;
	
	printk( "as0260_s_fmt state requested res(%d, %d)\n",
		state->fmt.width, state->fmt.height);
	return err;
}

static int as0260_enum_framesizes(struct v4l2_subdev *sd,
					struct v4l2_frmsizeenum *fsize)
{
	int err = 0;
	struct as0260_state *state = to_state(sd);
	int num_entries = sizeof(as0260_framesize_list)/sizeof(struct as0260_enum_framesize);
	struct as0260_enum_framesize *elem;
	int index=0;
	int i=0;	
	/*
	* Return the actual output settings programmed to the camera
	*/
	printk("%s________________________: width - %d , height - %d\n", __func__, fsize->discrete.width, fsize->discrete.height);

	index = state->framesize_index;

	for(i = 0; i < num_entries; i++){
		elem = &as0260_framesize_list[i];
		if(elem->index == index){
			fsize->discrete.width = as0260_framesize_list[index].width;
			fsize->discrete.height = as0260_framesize_list[index].height;
			printk("%s OK ___________: width - %d , height - %d\n",
				__func__, fsize->discrete.width, fsize->discrete.height);
			return 0;
		}
	}
	fsize->discrete.width = state->default_width;
	fsize->discrete.height = state->default_height;

	printk("%s Fail default : width - %d , height - %d\n", __func__, fsize->discrete.width, fsize->discrete.height);
	return err;
}

static int as0260_enum_frameintervals(struct v4l2_subdev *sd,
					struct v4l2_frmivalenum *fival)
{
	int err = 0;

	return err;
}

static int as0260_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	int err = 0;

	return err;
}

static int as0260_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	int err = 0;

	return err;
}

static int as0260_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct as0260_state *state = to_state(sd);
	struct as0260_userset userset = state->userset;
	int err = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_CAM_JPEG_MEMSIZE:
		ctrl->value = as0260_JPEG_MAXSIZE +
			as0260_THUMB_MAXSIZE + as0260_POST_MAXSIZE;
		err = 0;
		break;	

	case V4L2_CID_CAM_JPEG_MAIN_OFFSET:
		ctrl->value = as0260_JPEG_MAXSIZE +
			as0260_THUMB_MAXSIZE + as0260_POST_MAXSIZE;
		err = 0;
		break;	

	case V4L2_CID_CAM_JPEG_THUMB_SIZE:
		ctrl->value = as0260_JPEG_MAXSIZE +
			as0260_THUMB_MAXSIZE + as0260_POST_MAXSIZE;
		err = 0;
		break;			
		
	case V4L2_CID_CAM_JPEG_THUMB_OFFSET:
		ctrl->value = as0260_JPEG_MAXSIZE +
			as0260_THUMB_MAXSIZE + as0260_POST_MAXSIZE;
		err = 0;
		break;	

	case V4L2_CID_CAM_JPEG_POSTVIEW_OFFSET:
		ctrl->value = as0260_JPEG_MAXSIZE +
			as0260_THUMB_MAXSIZE + as0260_POST_MAXSIZE;
		err = 0;
		break;			

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

static int as0260_set_jpeg_quality(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	//struct v4l2_queryctrl qc = {0,};
	//int val = ctrl->value;

	printk(" as0260_set_jpeg_quality =%d \n", ctrl->value);
	
	return 0;
}

static int as0260_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//struct as0260_state *state = to_state(sd);
	//struct as0260_userset userset = state->userset;
	int err = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_CAM_PREVIEW_ONOFF:
		if (ctrl->value)
			err = as0260_set_preview_start(sd);
		else
			err = as0260_set_preview_stop(sd);
		printk( "V4L2_CID_CAM_PREVIEW_ONOFF [%d] \n", ctrl->value);
		break;
		
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "%s: V4L2_CID_EXPOSURE\n", \
			__func__);
		//err = as0260_write_regs(sd, as0260_regs_ev_bias[ctrl->value]);
		break;

	case V4L2_CID_CAM_CAPTURE:
		err = as0260_set_capture_start(sd);
		as0260_info(&client->dev, "V4L2_CID_CAM_CAPTURE [%d] \n", ctrl->value);
		printk( "V4L2_CID_CAM_CAPTURE \n");
		break;

	case V4L2_CID_CAM_JPEG_QUALITY:
		err = as0260_set_jpeg_quality(sd, ctrl);
		break;

    case V4L2_CID_CAMERA_EFFECT://V4L2_CID_COLORFX:
		err =as0260_set_effect(sd, ctrl->value);
		break;

    case V4L2_CID_CAMERA_BRIGHTNESS://V4L2_CID_COLORFX:
		err =as0260_set_brightness(sd, ctrl->value);
		break;
		
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

}
//extern int smdk4x12_cam1_reset(int dummy);

static int as0260_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;
	struct as0260_platform_data *pdata;
	struct as0260_state *state = to_state(sd);
	u32 read_value=0;
	u16 buf16=0;
	//u8 buf=0;
	
	v4l_info(client, "%s: camera initialization start\n", __func__);
	pdata = client->dev.platform_data;

	as0260_i2c_16_read(client, 0x0000,&buf16);
	printk("as0260_init ______ AS0260 chip ID =0x%x\n",buf16);

	as0260_i2c_16_read(client, 0x001c,&buf16);
	printk("as0260_init ______ AS0260 MCU BOOT MODE =0x%x\n",buf16);
		
	err =as0260_i2c_w_write_regs(sd,&as0260_init_regs[0], ARRAY_SIZE(as0260_init_regs));

	//err =as0260_i2c_w_write_regs(sd,&as0260_init_regs[0], ARRAY_SIZE(as0260_init_regs));
	err=changestate(sd,SS_ENTER_CONFIG_CHANGE,HC_SET_STATE);
	if(err==0)
	{
		as0260_i2c_w_write_regs(sd,&as0260_init_low_regs[0], ARRAY_SIZE(as0260_init_regs));
		changestate(sd,SS_ENTER_CONFIG_CHANGE,HC_SET_STATE);
	}
	//changestate(sd,SS_STREAMING,HC_SET_STATE);
	//refresh(sd);

	backup_client=client;

#if 0//#if !defined(AS0260_DEBUG)
	//err=as0260_i2c_write_reg_32(client, reg_tbl->waddr, reg_tbl->wdata);
	as0260_i2c_32_read(client, 0xe004,&read_value);
	printk("as0260_init ______ AS0260 0xe004 Firmware Ver =0x%x\n",read_value);
	as0260_i2c_32_read(client, 0xc808,&read_value);
	printk("as0260_init ______ AS0260 0xc808 CAM_SENSOR_CFG_PIXCLK Ver =0x%x\n",read_value);

	as0260_i2c_8_read(client, 0xdc01,&buf);
	printk("as0260_init ______ AS0260 0xdc01 current state =0x%x\n",buf);

	as0260_i2c_8_read(client, 0xdc00,&buf);
	printk("as0260_init ______ AS0260 0xdc00 Next state =0x%x\n",buf);

	as0260_i2c_16_read(client, 0x0012,&buf16);
	printk("as0260_init ______ AS0260 0x0012 PLL_P divider =0x%x\n",buf16);

	as0260_i2c_16_read(client, 0x3c42,&buf16);
	printk("as0260_init ______ AS0260 0x3c42 MIPI Status =0x%x\n",buf16);

	as0260_i2c_16_read(client, 0x0874,&buf16);
	printk("as0260_init ______ AS0260 0x0874 UDI status =0x%x\n",buf16);  // 2 udi_streaming en

	as0260_i2c_16_read(client, 0x301A,&buf16);
	printk("as0260_init ______ AS0260 0x301A reset_register =0x%x\n",buf16);

	as0260_i2c_16_read(client, 0x0014,&buf16);
	printk("as0260_init ______ AS0260 0x0014 PLL LOCK=0x%x\n",buf16);

	as0260_i2c_16_read(client, 0x0016,&buf16);
	printk("as0260_init ______ AS0260 0x0016 clocks_control=0x%x\n",buf16);

	as0260_i2c_16_read(client, 0x0018,&buf16);
	printk("as0260_init ______ AS0260 0x0018 standby_control_and_status=0x%x\n",buf16);

	as0260_i2c_16_read(client, 0xc870,&buf16);
	printk("as0260_init ______ AS0260 0x0018 out format =0x%x\n",buf16);

#endif
	//changestate(sd,SS_ENTER_CONFIG_CHANGE,HC_SET_STATE);
	//changestate(sd,SS_STREAMING,HC_SET_STATE);
	//refresh(sd);
	as0260_i2c_16_read(client, 0xc870,&buf16);
	printk("as0260_init ______ AS0260 0x0018 out format =0x%x\n",buf16);
	
#if defined(AS0260_DEBUG)
	/* 0x3070 test pattern
		0: Normal operation: Generate output data from pixel array
		1: Solid color test pattern.
		2: 100% color bar test pattern
		3: Fade-to-gray color bar test pattern
		256: Walking 1s test pattern (10-bit)
		257: Walking 1s test pattern (8-bit)
	*/
	//err=as0260_i2c_write_reg_16(client, 0x3070, 2);  // test pattern
#endif

	printk(" 	as0260_init d_width=%d d_height=%d c_width=%d c_height=%d \n", \
		state->default_width,state->default_height,state->capture_width,state->capture_height );

	
	if (err < 0) {
		v4l_err(client, "%s: camera initialization failed\n", \
			__func__);
		return -EIO;	/* FIXME */
	}
	v4l_info(client, "%s: camera initialization end\n", __func__);
	msleep(100);
	read_value=0;
	//printk("as0260_init === read status=%x \n", read_value);
	
	return 0;
}

static int as0260_s_power(struct v4l2_subdev *sd, int on)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	//struct as0260_state *state = to_state(sd);
	//struct as0260_mbus_platform_data *pdata = state->pdata;
	//int ret;
	printk(" as0260_s_power on/off =%d \n", on);
	/* bug report */

	return 0;
}
#if defined(AS0260_NOT_USE)
static int as0260_sleep(struct v4l2_subdev *sd)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	//int err = -EINVAL, i;

	//v4l_info(client, "%s: sleep mode\n", __func__);
	return 0;
}

static int as0260_wakeup(struct v4l2_subdev *sd)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	//int err = -EINVAL, i;

	//v4l_info(client, "%s: wakeup mode\n", __func__);
	return 0;
}
#endif
static int as0260_s_stream(struct v4l2_subdev *sd, int enable)
{
	//struct as0260_state *state = to_state(sd);
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	int err=0;
	printk("%s %d \n",__func__,enable);
#if 1
	switch(enable) {
		case 1:  //PREVIEW
		err = as0260_set_preview_start(sd);
		break;
		case 3: //Snapshot
		err = as0260_set_capture_start(sd);
		break;
		case 6: //stanby reset
		break;
		//as0260_stanby();
		default:
		break;
	}
	
#endif
return err;
//	return enable ? as0260_wakeup(sd) : as0260_sleep(sd);

}
#if defined(AS0260_NOT_USE)

/*
 * s_config subdev ops
 * With camera device, we need to re-initialize every single opening time
 * therefor, it is not necessary to be initialized on probe time.
 * except for version checking.
 * NOTE: version checking is optional
 */
 
static int as0260_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct as0260_state *state = to_state(sd);
	struct as0260_platform_data *pdata;

	dev_info(&client->dev, "fetching platform data\n");

	pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -ENODEV;
	}

	/*
	 * Assign default format and resolution
	 * Use configured default information in platform data
	 * or without them, use default information in driver
	 */
	if (!(pdata->default_width && pdata->default_height)) {
		/* TODO: assign driver default resolution */
	} else {

		state->default_width =  pdata->default_width;;
		state->default_height = pdata->default_height;;
	}

	if (!pdata->pixelformat)
		state->pixelformat = DEFAULT_FMT;
	else
		state->pixelformat = pdata->pixelformat;

	if (!pdata->freq)
		state->freq = 24000000;	/* 24MHz default */
	else
		state->freq = pdata->freq;

	if (!pdata->is_mipi) {
		state->is_mipi = 0;
		dev_info(&client->dev, "parallel mode\n");
	} else
		state->is_mipi = pdata->is_mipi;

	return 0;
}
#endif

static const struct v4l2_subdev_core_ops as0260_core_ops = {
	.init = as0260_init,	/* initializing API */
	.s_power = as0260_s_power,
	.queryctrl = as0260_queryctrl,
	.querymenu = as0260_querymenu,
	.g_ctrl = as0260_g_ctrl,
	.s_ctrl = as0260_s_ctrl,
};

static const struct v4l2_subdev_video_ops as0260_video_ops = {
	.s_crystal_freq = as0260_s_crystal_freq,
	.g_mbus_fmt = as0260_g_fmt,
	.s_mbus_fmt = as0260_s_fmt,
	.enum_framesizes = as0260_enum_framesizes,
	.enum_frameintervals = as0260_enum_frameintervals,
	.g_parm = as0260_g_parm,
	.s_parm = as0260_s_parm,
	.s_stream = as0260_s_stream,
};

static const struct v4l2_subdev_ops as0260_ops = {
	.core = &as0260_core_ops,
	.video = &as0260_video_ops,
};

/*
 * as0260_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int as0260_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct as0260_state *state;
	struct v4l2_subdev *sd;
	struct as0260_platform_data *pdata;
	int err=0;
	pdata = client->dev.platform_data;

	state = kzalloc(sizeof(struct as0260_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, as0260_DRIVER_NAME);
	//state->pdata = client->dev.platform_data;

	/* set default data from sensor specific value */

	if (!(pdata->default_width && pdata->default_height)) {
		/* TODO: assign driver default resolution */
	} else {

		state->default_width =  pdata->default_width;
		state->default_height = pdata->default_height;
		state->fmt.width = pdata->default_width;
		state->fmt.height =pdata->default_height;
	}

	if (!pdata->pixelformat)
		state->pixelformat = DEFAULT_FMT;
	else
		state->pixelformat = pdata->pixelformat;

	if (!pdata->freq)
		state->freq = 24000000;	/* 24MHz default */
	else
		state->freq = pdata->freq;

	if (!pdata->is_mipi) {
		state->is_mipi = 0;
		dev_info(&client->dev, "parallel mode\n");
	} else
		state->is_mipi = pdata->is_mipi;

	printk(" as0260_probe state->fmt.width =%d state->fmt.height=%d \n",state->fmt.width ,state->fmt.height);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &as0260_ops);

	/* needed for acquiring subdevice by this module name */
	snprintf(sd->name, sizeof(sd->name), as0260_DRIVER_NAME);

	dev_info(&client->dev, "as0260 has been probed\n");

	err = sysfs_create_group(&client->dev.kobj, &as0260_attribute_group);
	if (0 != err)
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, err);
		sysfs_remove_group(&client->dev.kobj, &as0260_attribute_group);
	}
	else
	{
		printk("[AS0260] :%s() - sysfs_create_group() succeeded.\n", __FUNCTION__);
	}

	return 0;
}


static int as0260_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id as0260_id[] = {
	{ as0260_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, as0260_id);

static struct i2c_driver as0260_i2c_driver = {
	.driver = {
		.name	= as0260_DRIVER_NAME,
	},
	.probe		= as0260_probe,
	.remove		= as0260_remove,
	.id_table	= as0260_id,
};

static int __init as0260_mod_init(void)
{
	return i2c_add_driver(&as0260_i2c_driver);
}

static void __exit as0260_mod_exit(void)
{
	i2c_del_driver(&as0260_i2c_driver);
}
module_init(as0260_mod_init);
module_exit(as0260_mod_exit);

MODULE_DESCRIPTION("Samsung Electronics as0260 SXGA camera driver");
MODULE_AUTHOR("Dongsoo Nathaniel Kim<dongsoo45.kim@samsung.com>");
MODULE_LICENSE("GPL");


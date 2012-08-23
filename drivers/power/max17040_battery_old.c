/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>
#include <linux/android_alarm.h>

#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <mach/gpio-willow.h>

#include <linux/delay.h>
#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_DELAY			msecs_to_jiffies(1000 * 10)
//#define MAX17040_BATTERY_SOC_FULL	99//97.34
#define MAX17040_BATTERY_SOC_FULL	  98//T10S ?‘ì˜?¬í•­: 98 ?´ìƒ????100 ?¼ë¡œ ?¤ì •
#define MAX17040_BATTERY_SOC_FULL_REAL 100
#define MAX17040_BATTERY_SOC_EMPTY  1 //1.38

//#define T9_FUEL_GAUGE_INI
//#define USE_MAX17040_ALARM
#define	POWER_OFF_VOLTAGE	  3400000
#define	CHARGE_OFF_VOLTAGE	4200000
#define T9_BATT_TEST
#define FAST_POLL			(20)
#define SLOW_POLL			(10 * 60)

#define MAX17040_T9_RCOMP 0x6c00//0xf700

static int batt_debug_enable = 0;
static struct wake_lock vbus_wake_lock;

#ifdef T9_FUEL_GAUGE_INI
u16 EmptyAdjustment = 0;
u16 FullAdjustment= 100;
u16 RCOMP0=108;
u16 TempCoUp =-0.05;
u16 TempCoDown = -4.925;
u16 OCVTest = 55968;
u16 SOCCheckA = 244;
u16 SOCCheckB = 246;
u16 bits= 19;
u8 model_data[64] = {
	0x7D, //--(0x40h) Model data begins here                                                                    
	0x00,
	0xB6,
	0xD0,
	0xB9,                                                                                                             
	0x00,
	0xBB,
	0x80,
	0xBC,
	0x60,
	0xBC,
	0xC0,
	0xBD,
	0x30,
	0xBD,
	0xA0,  //(0x4Fh)                                                                                           
	0xBD, //data begins here                                                                          
	0xF0,
	0xBF,
	0x10,
	0xC0,
	0x70,
	0xC2,
	0xD0,
	0xC5,
	0x40,
	0xC7,
	0xB0,
	0xCB,
	0xF0,
	0xD0,
	0xA0,//  --(0x5Fh)                                                                                           
	0x00, // --(0x60h) data begins here                                                                          
	0x20,
	0x24,
	0xC0,
	0x1C,
	0xC0,
	0x2C,
	0xF0,
	0x92,
	0xA0,
	0x7C,
	0x00,
	0x5D,
	0xC0,
	0x73,
	0xD0,//--(0x6Fh)
	0x19, // --(0x70h) data begins here
	0xF0,
	0x1F,
	0xF0,
	0x19,
	0x00,
	0x1B,
	0x00,
	0x1C,
	0x00,
	0x0F,
	0x90,
	0x12,
	0x00,
	0x12,
	0x00//--(0x7Fh) Model data ends here;
};
static void max17040_reset(struct i2c_client *client);
int max17040_load_model_init(struct i2c_client *client);
#else
u16 bits= 19;
#endif /* T9_FUEL_GAUGE_INI */
#ifdef FEATURE_T9_BATT_THERM
struct adc_sample_info {
	unsigned int cnt;
	int total_adc;
	int average_adc;
	int adc_arr[ADC_TOTAL_COUNT];
	int index;
};

struct batt_adc_table_data {
	int adc_value;
	int temperature;
};


static struct batt_adc_table_data temper_table[] =  {
	/* ADC, Temperature (C/10) */
	{3889,   -400},
	{3830,   -350},
	{3758,   -300},
	{3671,   -250},
	{3569,   -200},
	{3450,   -150},
	{3314,   -100},
	{3163,    -50},
	{2996,     00},
	{2818,     50},
	{2630,    100},
	{2437,    150}, 
	{2241,    200},
	{2202,    210}, 
	{2163,    220}, 
	{2125,    230}, 
	{2086,    240}, 
	{2048,    250}, 
	{2009,    260}, 
	{1971,    270}, 
	{1933,    280}, 
	{1896,    290}, 
	{1859,    300}, 
	{1678,    350}, 
	{1508,    400}, 
	{1349,    450}, 
	{1203,    500}, 
	{1070,    550}, 
	{ 950,    600}, 
	{ 842,    650}, 
	{ 746,    700}, 
	{ 661,    750},
	{ 585,    800}, 
	{ 519,    850}, 
	{ 460,    900}, 
	{ 408,    950}, 
	{ 363,   1000},
};

#endif /* FEATURE_T9_BATT_THERM */

struct max17040_chip {
	struct i2c_client		*client;
#ifdef USE_MAX17040_ALARM
	struct work_struct		work;
#else
	struct delayed_work		work;
#endif
	struct power_supply		battery;
	struct power_supply		ac;
	struct power_supply		usb;
	struct max17040_platform_data	*pdata;  
	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery level test */
	int level_test;
	/* battery capacity */
	int soc;
	/* battery capacity real */
	int soc_real;
	/* State Of Charge */
	int status;
	/* usb online */
	int usb_online;

#ifdef FEATURE_T9_BATT_THERM
	struct adc_sample_info	therm_adc_sample;
	u32 batt_temp;		/* Battery Temperature (C) from ADC */
	u32 batt_temp_adc;	/* Battery Temperature ADC value */
	u32 batt_health;	/* Battery Health (Authority) */
	u32 dis_reason;

	struct adc_sample_info	soc_sample;

#ifdef USE_MAX17040_ALARM
	struct alarm		alarm;
	int			timestamp;
	int                     slow_poll;
	ktime_t                 last_poll;
#endif
#endif /* FEATURE_T9_BATT_THERM */
};

bool usb_is_connected = 0;
int dc_is_connected = 0;

struct max17040_chip *chip = NULL;

void isUSBconnected(bool usb_connect)
{
	if ( usb_connect ) {
		wake_lock(&vbus_wake_lock);
	} else {
		wake_unlock(&vbus_wake_lock);
		wake_lock_timeout(&vbus_wake_lock, 2*HZ);
	}
}
EXPORT_SYMBOL(isUSBconnected);
EXPORT_SYMBOL(dc_is_connected);

#ifdef T9_BATT_TEST
int batt_test_enable = 0;
static ssize_t max17040_proc_read(struct file *file, char *buf, size_t nbytes, loff_t * ppos)
{
	struct inode   *my_inode = file->f_dentry->d_inode;
	struct proc_dir_entry *dp;
	char            outputbuf[15];
	int             count;

	if (*ppos > 0)
		return 0;

	dp = PDE(my_inode);

	count = sprintf(&outputbuf[0], "batt_test  : %d\n", batt_test_enable);

	printk("[max9888_read_registers] batt_test_enable = %d\n",batt_test_enable);

	*ppos += count;

	if (count > nbytes)
		return -EINVAL;
	if (copy_to_user(buf, &outputbuf[0], count))
		return -EFAULT;

	return count;
}


static ssize_t max17040_proc_write(struct file *file, const char *buffer, size_t count, loff_t * ppos)
{
	char           *endp;

	batt_test_enable = (u32)(simple_strtoul(buffer, &endp, 10));

	printk("[max17040_proc_write]batt_test_enable = %d\n",batt_test_enable);

	return (count + endp - buffer);
}

static struct file_operations max17040_proc_reg_fops = {
	.read = max17040_proc_read,
	.write = max17040_proc_write,
};

static ssize_t max17040_proc_debug_read(struct file *file, char *buf, size_t nbytes, loff_t * ppos)
{
	struct inode   *my_inode = file->f_dentry->d_inode;
	struct proc_dir_entry *dp;
	char            outputbuf[15];
	int             count;

	if (*ppos > 0)
		return 0;

	dp = PDE(my_inode);

	count = sprintf(&outputbuf[0], "batt_test  : %d\n", batt_debug_enable);

	printk("[max17040_proc_debug_read] batt_debug_enable = %d\n",batt_debug_enable);

	*ppos += count;

	if (count > nbytes)
		return -EINVAL;
	if (copy_to_user(buf, &outputbuf[0], count))
		return -EFAULT;

	return count;
}


static ssize_t max17040_proc_debug_write(struct file *file, const char *buffer, size_t count, loff_t * ppos)
{
	char           *endp;

	batt_debug_enable = (u32)(simple_strtoul(buffer, &endp, 10));

	printk("[max17040_proc_debug_write]batt_debug_enable = %d\n",batt_debug_enable);

	return (count + endp - buffer);
}
//debug msg enable/disable.
static struct file_operations max17040_proc_batt_debug_fops = {
	.read = max17040_proc_debug_read,
	.write = max17040_proc_debug_write,
};
#endif


#if defined(CONFIG_INPUT_BMA150_SENSOR)
extern int bma150_sensor_get_air_temp(void);
#endif

static int max17040_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max17040_chip *chip = container_of(psy,
			struct max17040_chip, battery);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = chip->status;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = chip->online;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = chip->vcell;
			if(psp  == POWER_SUPPLY_PROP_PRESENT)
				val->intval = 1; /* You must never run Odrioid1 without Battery. */
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = chip->soc;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			if(chip->vcell  < 2850)
				val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			else
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case POWER_SUPPLY_PROP_TEMP:
#ifdef FEATURE_T9_BATT_THERM
			val->intval = chip->batt_temp;
#else
#if defined(CONFIG_INPUT_BMA150_SENSOR)
			val->intval = bma150_sensor_get_air_temp();
#else
			val->intval = 365;
#endif
#endif
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static int max17040_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct max17040_chip *chip = container_of(psy,
			struct max17040_chip, battery);  

	switch (psp) {
		case POWER_SUPPLY_PROP_CAPACITY:
			chip->level_test = 1;
			if( val->intval > 100)
			{
				chip->soc = 100;
			}
			else if( val->intval < 0 )
			{
				chip->soc = 0;
			}
			else
			{
				chip->soc = val->intval;
			}
			break;

		default:
			return -EINVAL;
	}
	return 0;
}


static int usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	struct max17040_chip *chip = container_of(psy,
			struct max17040_chip, usb);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			//TODO:
			val->intval =  chip->usb_online;
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static int adapter_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
#if defined(FEATURE_T10S_MAX17040)
	struct max17040_chip *chip = container_of(psy,
			struct max17040_chip, ac);
#endif
	int ret = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_ONLINE:
			//TODO:
#if defined(FEATURE_T10S_MAX17040)
			if (chip->pdata->charger_online)
				val->intval = chip->pdata->charger_online() ? (chip->usb_online ? 0 : 1) : 0;
#else
			val->intval = 0;
#endif
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

#if 0 //bootloader
static int max17040_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
#endif

static int max17040_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
static int max17040_write_word(struct i2c_client *client, int reg, u16 value)
{
	int ret;
	ret = i2c_smbus_write_word_data(client, reg, value);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	return ret;
}
static int max17040_read_word(struct i2c_client *client, int reg)
{
	int ret;
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	return ret;
}
int max17040_block_read(struct i2c_client *client, u8 reg, u8 length, u8 *values)
{
	int ret;
	ret = i2c_smbus_read_i2c_block_data(client, reg, length, values);
	if (ret < 0)
		printk("failed to read regs %#x: %d\n",reg, ret);
	return ret;
}
int max17040_block_write(struct i2c_client *client, u8 reg, u8 length,
		const u8 *values)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, length,
			values);
	if (ret < 0)
		printk("failed to write regs %#x: %d\n",reg, ret);

	return ret;
}

unsigned int max17040_get_rcomp(struct i2c_client *client)
{
	unsigned int rcomp=0;

	rcomp = max17040_read_word(client, MAX17040_RCOMP_MSB);
	printk("MAX17040 Fuel-Gauge RCOMP 0x%x\n", rcomp);

	return rcomp;

}

#if 0 //bootloader
static void max17040_set_rcomp(struct i2c_client *client, u16 value)
{
	max17040_write_word(client, MAX17040_RCOMP_MSB,value); 
}
#endif

#ifdef T9_FUEL_GAUGE_INI
/* 
	 1. max17040_unlock(Unlock Model Access)
	 To unlock access to the model the host software must write 0x4Ah to memory location 0x3Eh and write 0x57h to
	 memory location 0x3Fh.
//Unlock Model Access
 */
void  max17040_unlock(struct i2c_client *client)
{  
	/*  
			START
			Send Slave Address (0x6Ch)
			Send Memory Location (0x3Eh)
			Send Data Byte (0x4Ah)
			Send Data Byte (0x57h)
			STOP
	 */
	max17040_write_word(client, 0x3e, 0x4A57);
}

/*
	 Lock Model Access
	 To lock access to the model the host software must write 0x00h to memory location 0x3Eh and 0x00h to memory
	 location 0x3Fh
 */
void max17040_lock(struct i2c_client *client)
{  
	/*  
	//Lock Model Access
	START
	Send Slave Address (0x6Ch)
	Send Memory Location (0x3Eh)
	Send Data Byte (0x00h)
	Send Data Byte (0x00h)
	STOP
	 */
	max17040_write_word(client, 0x3e, 0x0000);
}

/*
	 The OCV Register will be modified during the process of loading the custom model. Read and store
	 this value so that it can be written back to the device after the model has been loaded.
 */
unsigned int  max17040_read_ocv(struct i2c_client *client)
{
	return max17040_read_word(client, 0x0E);
}


void max17040_write_ocv(struct i2c_client *client, u16 value)
{  
	max17040_write_word(client, 0x0E, value);
	return;
}

/*  Write the Model
		Once the model is unlocked, the host software must write the 64 byte model to the MAX17040/1/3. The model is
		located between memory locations 0x40h and 0x7Fh.
		Write 64 byte model
 */
void max17040_write_model(struct i2c_client *client)
{ 
	max17040_block_write(client, 0x40, 16, &model_data[0]);
	max17040_block_write(client, 0x50, 16, &model_data[0x10]);
	max17040_block_write(client, 0x60, 16, &model_data[0x20]);
	max17040_block_write(client, 0x70, 16, &model_data[0x30]);  
}

int max17040_compare_to_expect_soc(struct i2c_client *client)
{   
	u16 soc_1;
	soc_1 = max17040_read_word(client, MAX17040_SOC_MSB);
	soc_1 = soc_1 >> 8;

	if(soc_1 >= SOCCheckA && soc_1 <= SOCCheckB){
		// model was loaded successfully
		printk("%s : valid : %d\n", __func__, soc_1);
		return 0;

	}
	else{
		// model was NOT loaded successfully
		printk("%s : invalid : %d\n", __func__, soc_1);
		return -1;  
	}
}


int max17040_load_model_init(struct i2c_client *client)
{
	u16 original_OCV = 0;
	u16 OriginalRCOMP =0;
	// 1. Unlock Model Access
	max17040_unlock(client);

	//2. Read OCV
	original_OCV = max17040_read_ocv(client);
	OriginalRCOMP = max17040_get_rcomp(client);

	//3. Write OCV (MAX17040/1/3/4 only)
	max17040_write_ocv(client, OCVTest);

	//4. Write RCOMP to its Maximum Value (MAX17040/1/3/4 only)
	max17040_set_rcomp(client, 0xff00);

	//5. Write the Model
	// Once the model is unlocked, the host software must write the 64 byte model to the MAX17040/1/3. The model is
	// located between memory locations 0x40h and 0x7Fh.
	//Write 64 byte model
	max17040_write_model(client);

	//6. Delay at least 150ms (MAX17040/1/3/4 only)
	mdelay(200);

	//7. Write OCV
	max17040_write_ocv(client, OCVTest);

	//7.1. Lock Model Access (MAX17048/9 only)
	//max17040_lock(client);

	//8. Delay between 150ms and 600ms
	mdelay(400);

	//9. Read SOC Register and compare to expected result
	if(max17040_compare_to_expect_soc(client) < 0)
	{
		printk("%s : invalid : \n", __func__);
	}

	//9.1. Unlock Model Access (MAX17048/9 only)
	//max17040_unlock(client);

	//10. Restore CONFIG and OCV
	max17040_write_word(client, MAX17040_RCOMP_MSB, OriginalRCOMP);
	max17040_write_word(client, 0x0E, original_OCV);

	//11. Lock Model Access
	max17040_lock(client);

	//12. Delay at least 150mS
	mdelay(200);

	max17040_set_rcomp(client,MAX17040_T9_RCOMP);

}
#endif /* T9_FUEL_GAUGE_INI */

#ifdef FEATURE_T9_BATT_THERM
static int s3c_bat_get_adc_data(enum batt_adc_channel_type adc_ch)
{
	int adc_data;
	int adc_max = 0;
	int adc_min = 0;
	int adc_total = 0;
	int i;

	for (i = 0; i < ADC_DATA_ARR_SIZE; i++) {
		adc_data = s3c_adc_get_adc_data(adc_ch);

		if (i != 0) {
			if (adc_data > adc_max)
				adc_max = adc_data;
			else if (adc_data < adc_min)
				adc_min = adc_data;
		} else {
			adc_max = adc_data;
			adc_min = adc_data;
		}
		adc_total += adc_data;
	}

	return (adc_total - adc_max - adc_min) / (ADC_DATA_ARR_SIZE - 2);
}

static unsigned long calculate_average_adc(int adc, struct max17040_chip *chg)
{
	unsigned int cnt = 0;
	int total_adc = 0;
	int average_adc = 0;
	int index = 0;

	cnt = chg->therm_adc_sample.cnt;
	total_adc = chg->therm_adc_sample.total_adc;

	if (adc <= 0) {
		pr_err("%s : invalid adc : %d\n", __func__, adc);
		adc = chg->therm_adc_sample.average_adc;
	}

	if (cnt < ADC_TOTAL_COUNT) {
		chg->therm_adc_sample.adc_arr[cnt] = adc;
		chg->therm_adc_sample.index = cnt;
		chg->therm_adc_sample.cnt = ++cnt;

		total_adc += adc;
		average_adc = total_adc / cnt;
	} else {
		index = chg->therm_adc_sample.index;
		if (++index >= ADC_TOTAL_COUNT)
			index = 0;

		total_adc = total_adc - chg->therm_adc_sample.adc_arr[index]
			+ adc;
		average_adc = total_adc / ADC_TOTAL_COUNT;

		chg->therm_adc_sample.adc_arr[index] = adc;
		chg->therm_adc_sample.index = index;
	}

	chg->therm_adc_sample.total_adc = total_adc;
	chg->therm_adc_sample.average_adc = average_adc;

	chg->batt_temp_adc = average_adc;

	return average_adc;
}


unsigned long s3c_read_temp2(struct max17040_chip *client)
{
	int adc = 0;

	adc = s3c_bat_get_adc_data(ADC_BATT_TH2);

	return calculate_average_adc(adc, client);
}

int s3c_get_bat_temp(struct i2c_client *client)
{
	int temp = 0;
	struct max17040_chip *chip = i2c_get_clientdata(client);
	int temp_adc = s3c_read_temp2(chip);
	int health = chip->batt_health;
	int left_side = 0;
	int right_side = ARRAY_SIZE(temper_table) - 1;

	while (left_side <= right_side) {
		if(temper_table[left_side].adc_value >= temp_adc &&
				temper_table[left_side+1].adc_value < temp_adc) {
			temp = temper_table[left_side].temperature;
			break;
		}
		else {
			left_side++;
		}
	}

	chip->batt_temp = temp;
	if (temp >= HIGH_BLOCK_TEMP) {
		if (health != POWER_SUPPLY_HEALTH_OVERHEAT &&
				health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			chip->batt_health =
				POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (temp <= HIGH_RECOVER_TEMP && temp >= LOW_RECOVER_TEMP) {
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT ||
				health == POWER_SUPPLY_HEALTH_COLD)
			chip->batt_health =
				POWER_SUPPLY_HEALTH_GOOD;
	} else if (temp <= LOW_BLOCK_TEMP) {
		if (health != POWER_SUPPLY_HEALTH_COLD &&
				health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			chip->batt_health =
				POWER_SUPPLY_HEALTH_COLD;
	}

	if(batt_debug_enable)
	{
		printk("%s : temp = %d, adc = %d\n", __func__, temp, temp_adc);
	}

	return temp;
}


static unsigned long calculate_average_soc(int soc, struct max17040_chip *chg)
{
	unsigned int cnt = 0;
	int total_adc = 0;
	int average_adc = 0;
	int index = 0;

	cnt = chg->soc_sample.cnt;
	total_adc = chg->soc_sample.total_adc;

	if (soc <= 0) {
		pr_err("%s : invalid adc : %d\n", __func__, soc);
		soc = chg->soc_sample.average_adc;
	}

	if (cnt < ADC_TOTAL_COUNT) {
		chg->soc_sample.adc_arr[cnt] = soc;
		chg->soc_sample.index = cnt;
		chg->soc_sample.cnt = ++cnt;

		total_adc += soc;
		average_adc = total_adc / cnt;
	} else {
		index = chg->soc_sample.index;
		if (++index >= ADC_TOTAL_COUNT)
			index = 0;

		total_adc = total_adc - chg->soc_sample.adc_arr[index]
			+ soc;
		average_adc = total_adc / ADC_TOTAL_COUNT;

		chg->soc_sample.adc_arr[index] = soc;
		chg->soc_sample.index = index;
	}

	chg->soc_sample.total_adc = total_adc;
	chg->soc_sample.average_adc = average_adc;

	return average_adc;
}
#endif /* FEATURE_T9_BATT_THERM */

#if 0 //bootloader
static void max17040_reset(struct i2c_client *client)
{
	max17040_write_word(client, MAX17040_CMD_MSB, 0x5400);
}
#endif

static void max17040_get_vcell(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(client, MAX17040_VCELL_LSB);

	chip->vcell = (msb << 4) + (lsb >> 4);
	chip->vcell = (chip->vcell * 125) * 10;

	if(batt_debug_enable)
	{
		printk("BATTERY(%s) : Soc = %d, Vcell = %d\n", __FUNCTION__, chip->soc, chip->vcell);
	}
}


/*static*/ void max17040_get_soc(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
#if 0
	u8 msb;
	u8 lsb;
	static int old_soc=-1;

	msb = max17040_read_reg(client, MAX17040_SOC_MSB);
	lsb = max17040_read_reg(client, MAX17040_SOC_LSB);

	msb = (msb < 100) ? msb : 100;  

	if(chip->level_test)
	{    
		if(batt_debug_enable)
			printk("BATTERY(%s) : level_test(avr_soc=%d, soc=%d) !!!\n", __FUNCTION__, chip->soc, msb);

		return;
	}

	chip->soc = calculate_average_soc((int)msb, chip);
#else
	u8 SOC_1, SOC_2;
	int SOC_percent = 0;
	static int old_soc = -1;

	if(chip->level_test)
	{    
		if(batt_debug_enable)
			printk("BATTERY(%s) : level_test(soc=%d) !!!\n", __FUNCTION__, chip->soc);

		return;
	}

	SOC_1 = max17040_read_reg(client, MAX17040_SOC_MSB);
	SOC_2 = max17040_read_reg(client, MAX17040_SOC_LSB);

	if(bits == 18){  
		SOC_percent = ((SOC_1 << 8) + SOC_2) / 256;
	}
	else if(bits == 19){
		SOC_percent = ((SOC_1 << 8) + SOC_2) / 512;
	}
	chip->soc_real = chip->soc = SOC_percent;
	if( chip->soc > 100 )
		chip->soc = 100;
#endif

	if(chip->vcell < POWER_OFF_VOLTAGE)	
	{    
		if(batt_debug_enable)
			printk("BATTERY(%s) : Battery  is Low(soc=%d, vcell=%d) !!!\n", __FUNCTION__, chip->soc, chip->vcell);

		chip->soc = 0;
	}
	else if(chip->soc >= MAX17040_BATTERY_SOC_FULL)
	{    
		if(batt_debug_enable)
			printk("BATTERY(%s) : Battery  is Full(soc=%d, vcell=%d) !!!\n", __FUNCTION__, chip->soc, chip->vcell);
		chip->soc = 100;
	}
	else
	{
		if(batt_debug_enable)
			printk("BATTERY(%s) : soc=%d, vcell=%d\n", __FUNCTION__, chip->soc, chip->vcell);
	}

	if(batt_test_enable)
	{
		if(old_soc != chip->soc)
		{
			printk("BATTERY(%s) : Soc = %d, Vcell = %d\n", __FUNCTION__, chip->soc, chip->vcell);
			old_soc = chip->soc;
		}
	}

	return;
}

static void max17040_get_version(struct i2c_client *client)
{
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VER_MSB);
	lsb = max17040_read_reg(client, MAX17040_VER_LSB);
}

static void max17040_get_online(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if ( chip->pdata->charger_online ) {
		chip->online = chip->pdata->charger_online();
	}

	if ( chip->online ) {
		usb_is_connected = gpio_get_value(nUSB_OK) ? false : true;
		chip->usb_online = (int)usb_is_connected;

		if ( chip->usb_online )
			chip->online = 0;
	} else {
		chip->usb_online = 0;
	}

	dc_is_connected = chip->online;

	if ( batt_debug_enable )
		printk("[max17040_get_online] online=%d, usb_online=%d\n",chip->online, chip->usb_online);
}

static void max17040_get_status(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	static int full_charged = 0;

#if defined(FEATURE_T10S_MAX17040)
	if( !chip->pdata->charger_online || !chip->pdata->charger_enable || !chip->pdata->charger_disable )
	{
		return;
	}

	if(batt_debug_enable)
		printk("[max17040_get_status] usb_online(%d) online(%d)!\n", chip->usb_online, chip->online);

	if((chip->usb_online)||(chip->online))
	{
		if(chip->pdata->charger_done())
		{
			chip->pdata->charger_disable();
			chip->status = POWER_SUPPLY_STATUS_FULL;
			full_charged = 1;
			if(!chip->level_test)
				chip->soc = 100;

			if(batt_debug_enable) {
				printk("CHARGER Done. soc_real(%d) vcell(%d) status(%d) nCHARGING(%d)\n", chip->soc_real, chip->vcell, chip->status, chip->pdata->charger_done());

				if(chip->soc_real < MAX17040_BATTERY_SOC_FULL)
					printk("[TEST] CHARGER Done. soc_real(%d) vcell(%d) status(%d) nCHARGING(%d)\n", chip->soc_real, chip->vcell, chip->status, chip->pdata->charger_done());
			}
		}
		else
		{
			//      if((chip->soc > MAX17040_BATTERY_SOC_FULL) && (chip->vcell > CHARGE_OFF_VOLTAGE))
			if((chip->soc_real >= MAX17040_BATTERY_SOC_FULL_REAL) && (chip->vcell >= CHARGE_OFF_VOLTAGE))
			{
				chip->pdata->charger_disable();
				chip->status = POWER_SUPPLY_STATUS_FULL;  
				full_charged = 1;
				if(!chip->level_test)
					chip->soc = 100;

				if(batt_debug_enable)
					printk("CHARGER is now Done.! CHARGER Disable!\n");
			}
			else
			{
				if ( chip->soc_real >= MAX17040_BATTERY_SOC_FULL && full_charged ) {
					chip->pdata->charger_disable();
					if(batt_debug_enable)
						printk("FULL CHARGED soc_real(%d) vcell(%d) status(%d) nCHARGING(%d)\n", chip->soc_real, chip->vcell, chip->status, chip->pdata->charger_done());
				} else {
					full_charged = 0;
					if (chip->pdata->charger_enable())
					{
						chip->status = POWER_SUPPLY_STATUS_CHARGING;
						if(batt_debug_enable)
							printk("CHARGER is Charing!\n");
					}
					else
					{
						if(batt_debug_enable)
							printk("CHARGER isn't Charing! in online!\n");

						chip->status = POWER_SUPPLY_STATUS_CHARGING;
					}
				}
			}
		}
	}
	else
	{
		if((chip->status == POWER_SUPPLY_STATUS_CHARGING)||(chip->status == POWER_SUPPLY_STATUS_FULL))
		{
			chip->pdata->charger_disable();
		}
		if(batt_debug_enable)
			printk("CHARGER isn't Charing! not online!\n");

		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

#else
	chip->usb_online = usb_is_connected;
	if(chip->soc > 101) {
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if(chip->usb_online)
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
	else
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (chip->soc > MAX17040_BATTERY_FULL)
		chip->status = POWER_SUPPLY_STATUS_FULL;
#endif

}
#ifdef USE_MAX17040_ALARM
static void s3c_program_alarm(struct max17040_chip *chip, int seconds)
{
	ktime_t low_interval = ktime_set(seconds - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	next = ktime_add(chip->last_poll, low_interval);
	alarm_start_range(&chip->alarm, next, ktime_add(next, slack));
}


static void s3c_battery_alarm(struct alarm *alarm)
{
	struct max17040_chip *chip =
		container_of(alarm, struct max17040_chip, alarm);

	schedule_work(&chip->work);
}
#endif

#ifdef USE_MAX17040_ALARM
static void max17040_work(struct work_struct *work)
#else
static void max17040_work(struct delayed_work *work)
#endif
{
	struct max17040_chip *chip;
#ifdef USE_MAX17040_ALARM
	struct timespec ts;
#endif
	static int msg_update_cnt =0;
	static int old_usb_online = -1, old_soc=1, old_online=-1, old_vcell=-1, old_status=-1;
	unsigned long flags;

	chip = container_of(work, struct max17040_chip, work);

	max17040_get_online(chip->client);
	max17040_get_vcell(chip->client);
#ifdef FEATURE_T9_BATT_THERM
	s3c_get_bat_temp(chip->client);
#endif
	max17040_get_soc(chip->client);
	max17040_get_status(chip->client);

	if(chip->usb_online)
	{
		chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	if(chip->online || chip->usb_online)
	{
		if(msg_update_cnt > 5)
		{
#ifdef FEATURE_T9_BATT_THERM
			printk("[BATTERY] soc=%d%c, vcell=%duV, temp=%dï¿½ï¿½, dc=%d, usb=%d\n",chip->soc,'%',chip->vcell,chip->batt_temp,chip->online,chip->usb_online);
#else
			printk("[BATTERY] soc=%d%c, vcell=%duV, dc=%d, usb=%d\n",chip->soc,'%',chip->vcell,chip->online,chip->usb_online);
#endif
			msg_update_cnt = 0;
		}
		else
		{
			msg_update_cnt++;
		}
	}
	else
	{
		if(msg_update_cnt)
		{
			msg_update_cnt = 0;
		}
	}

	if(old_usb_online != chip->usb_online
			|| old_soc != chip->soc
			|| old_online != chip->online
			|| old_vcell != chip->vcell/100000
			|| old_status != chip->status)
	{
		if(batt_debug_enable)
		{
			printk("BATTERY(%s) : battery is changed!!!!!!\n", __FUNCTION__);
		}

		if(old_usb_online != chip->usb_online)
		{
			power_supply_changed(&chip->usb);
			if(batt_debug_enable)
			{
				printk("BATTERY(%s) : usb is changed(%d)\n", __FUNCTION__,chip->usb_online);
			}      
		}

		if(old_online != chip->online)
		{     
			power_supply_changed(&chip->ac);
			if(batt_debug_enable)
			{
				printk("BATTERY(%s) : ac is changed(%d)\n", __FUNCTION__,chip->online);
			}
		}   

		power_supply_changed(&chip->battery);  
		old_usb_online = chip->usb_online;
		old_soc = chip->soc;
		old_online = chip->online;
		old_vcell = chip->vcell/100000;
		old_status = chip->status;
	}
#ifdef USE_MAX17040_ALARM
	chip->last_poll = alarm_get_elapsed_realtime();
	ts = ktime_to_timespec(chip->last_poll);
	chip->timestamp = ts.tv_sec;    
#endif

	/* prevent suspend before starting the alarm */
	local_irq_save(flags);
#ifdef USE_MAX17040_ALARM
	if(chip->slow_poll)
		s3c_program_alarm(chip, SLOW_POLL);
	else
		s3c_program_alarm(chip, FAST_POLL);
#else
	schedule_delayed_work(&chip->work, MAX17040_DELAY);
#endif

	local_irq_restore(flags);

}

static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	/*POWER_SUPPLY_PROP_ONLINE,*/
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property adapter_get_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_get_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

#if 0//defined(FEATURE_T10S_MAX17040)
static struct {
	int gpio;
	char* name;
	bool output;
	int value;
} gpios[] = {
	{ GPIO_CHARGER_ONLINE,	"charger online", 0, 0 },
	{ GPIO_CHARGER_STATUS,	"is charging", 0, 0 },
	{ GPIO_CHARGER_ENABLE,	"charger enable", 1, 0 },
};
#endif
static irqreturn_t max8903_int_work_func(int irq, void *max8903_chg)
{
	struct max17040_chip *chg = NULL;	
	int i=0;

	chg = max8903_chg;

	if( batt_debug_enable )
	{
		for(i=0; i< chg->pdata->nOutputs;i++)
		{
			struct max8903_output_desc *output_desc = &chg->pdata->output_desc[i];

			if(irq == gpio_to_irq(output_desc->gpio))
			{
				printk("[max8903_int_work_func] %s !!!\n",output_desc->desc);

				if(strcmp(output_desc->desc, "vcharge_det_n") == 0)
				{
					printk("~~~~~~~~~ TA detect (%d)\n",gpio_get_value(nDC_OK));
				}
				break;
			}
		}
	}

	cancel_delayed_work_sync(&chg->work);
	schedule_delayed_work(&chg->work, msecs_to_jiffies(300));
	return IRQ_HANDLED;
}


static int max8903_gpio_setup_outputs(struct max17040_chip *bdata,
		struct max8903_output_desc *gpio_desc)
{
	char *desc = gpio_desc->desc ? gpio_desc->desc : "gpio_keys";
	unsigned long irqflags;
	int irq, error;

	error = gpio_request(gpio_desc->gpio, desc);
	if (error < 0) {
		printk("failed to request GPIO %d, error %d\n",
				gpio_desc->gpio, error);
		goto fail2;
	}

	error = gpio_direction_input(gpio_desc->gpio);
	if (error < 0) {
		printk("failed to configure"
				" direction for GPIO %d, error %d\n",
				gpio_desc->gpio, error);
		goto fail3;
	}

	s3c_gpio_setpull(gpio_desc->gpio, S3C_GPIO_PULL_UP);

	if(gpio_desc->enable_int)
	{
		irq = gpio_to_irq(gpio_desc->gpio);

		gpio_desc->irq = irq;

		if (irq < 0) {
			error = irq;
			printk("Unable to get irq number for GPIO %d, error %d\n",
					gpio_desc->gpio, error);
			goto fail3;
		}

		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
		/*
		 * If platform has specified that the button can be disabled,
		 * we don't want it to share the interrupt line.
		 */
		if (!gpio_desc->can_disable)
			irqflags |= IRQF_SHARED;

		error = request_irq(irq, max8903_int_work_func, irqflags, desc, bdata);
		if (error) {
			printk("Unable to claim irq %d; error %d\n",
					irq, error);
			goto fail3;
		}
	}

	return 0;

fail3:
	gpio_free(gpio_desc->gpio);
fail2:
	return error;
}

static int max8903_gpio_setup_chgen(int gpio, char *desc)
{
	int error;

	desc = desc ? desc : "chg_en_n";

	error = gpio_request(gpio, desc);
	if (error < 0) {
		printk("failed to request GPIO %d, error %d\n",
				gpio, error);
		goto fail2;
	}

	error = gpio_direction_output(gpio,1);
	if (error < 0) {
		printk("failed to configure"
				" direction for GPIO %d, error %d\n",
				gpio, error);
		goto fail3;
	}

	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);	

	return 0;

fail3:
	gpio_free(gpio);
fail2:
	return error;
}
static int __devinit max17040_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret, i=0, error;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17040_get_property;
	chip->battery.set_property	= max17040_set_property;
	chip->battery.properties	= max17040_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17040_battery_props);
	chip->battery.external_power_changed = NULL;

	chip->ac.name		= "ac";
	chip->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	chip->ac.get_property	= adapter_get_property;
	chip->ac.properties	= adapter_get_props;
	chip->ac.num_properties	= ARRAY_SIZE(adapter_get_props);
	chip->ac.external_power_changed = NULL;

	chip->usb.name		= "usb";
	chip->usb.type		= POWER_SUPPLY_TYPE_USB;
	chip->usb.get_property	= usb_get_property;
	chip->usb.properties	= usb_get_props;
	chip->usb.num_properties	= ARRAY_SIZE(usb_get_props);
	chip->usb.external_power_changed = NULL;

	chip->online = 0;
	chip->vcell = 0;
	chip->level_test = 0;
	chip->soc = 0;
	chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->usb_online = 0;

	ret = power_supply_register(&client->dev, &chip->battery);

	if (ret)
		goto err_battery_failed;

	ret = power_supply_register(&client->dev, &chip->ac);
	if(ret)
		goto err_ac_failed;

	ret = power_supply_register(&client->dev, &chip->usb);
	if(ret)
		goto err_usb_failed;

#ifdef T9_FUEL_GAUGE_INI
	//	max17040_reset(client);
	max17040_load_model_init(client);
#else
	//max17040_set_rcomp(client,MAX17040_T9_RCOMP);
	//max17040_get_rcomp(client);
#endif /* T9_FUEL_GAUGE_INI */  

	max17040_get_version(client);  

	max8903_gpio_setup_chgen(chip->pdata->chg_en_gpio,NULL);

	error = gpio_request(nUSB_OK, "chg_usb_mode_n");
	if (error < 0) {
		printk("failed to request GPIO %d, error %d\n",
				nUSB_OK, error);
		goto err_gpio_failed;
	}

	error = gpio_direction_input(nUSB_OK);
	if (error < 0) {
		printk("failed to configure"
				" direction for GPIO %d, error %d\n",
				nUSB_OK, error);
		goto err_gpio_failed;
	}
	s3c_gpio_setpull(nUSB_OK, S3C_GPIO_PULL_NONE);

	for (i = 0; i < chip->pdata->nOutputs; i++) {
		struct max8903_output_desc *output_desc = &chip->pdata->output_desc[i];

		error = max8903_gpio_setup_outputs(chip, output_desc);
		if (error)
			goto err_gpio_failed;
	}

#ifdef USE_MAX17040_ALARM  
	chip->last_poll = alarm_get_elapsed_realtime();
	alarm_init(&chip->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			s3c_battery_alarm);  

	INIT_WORK(&chip->work, max17040_work);
#else
	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17040_work);
#endif 

	if(chip->pdata->charger_disable)
	{
		chip->pdata->charger_disable();
	}

#ifdef USE_MAX17040_ALARM  
	chip->slow_poll = 0;
	schedule_work(&chip->work);
#else
	schedule_delayed_work(&chip->work, MAX17040_DELAY);
#endif  
#ifdef T9_BATT_TEST
	proc_create("batt_test", S_IRWXUGO, NULL, &max17040_proc_reg_fops);
	proc_create("batt_debug", S_IRWXUGO, NULL, &max17040_proc_batt_debug_fops);
#endif

	return 0;

#if defined(FEATURE_T10S_MAX17040)
err_gpio_failed:
	while (--i >= 0) {
		free_irq(gpio_to_irq(chip->pdata->output_desc[i].gpio), chip->pdata);
		gpio_free(chip->pdata->output_desc[i].gpio);
	}
#endif
err_usb_failed:
	power_supply_unregister(&chip->ac);
err_ac_failed:
	power_supply_unregister(&chip->battery);
err_battery_failed:
	dev_err(&client->dev, "failed: power supply register\n");
	i2c_set_clientdata(client, NULL);	
#ifdef USE_MAX17040_ALARM
	alarm_cancel(&chip->alarm);
#endif
	kfree(chip);
	return ret;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

#if defined(FEATURE_T10S_MAX17040)
	int i;

	for (i = 0; i < chip->pdata->nOutputs; i++) {
		free_irq(gpio_to_irq(chip->pdata->output_desc[i].gpio), chip->pdata);
		gpio_free(chip->pdata->output_desc[i].gpio);
	}
#ifdef USE_MAX17040_ALARM
	alarm_cancel(&chip->alarm);
#endif
#endif
	power_supply_unregister(&chip->usb);
	power_supply_unregister(&chip->ac);
	power_supply_unregister(&chip->battery);
#ifdef USE_MAX17040_ALARM
	flush_work(&chip->work);
#else
	cancel_delayed_work(&chip->work);
#endif
	i2c_set_clientdata(client, NULL);  
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17040_suspend(struct i2c_client *client,
		pm_message_t state)
{
	int i;
	struct max17040_chip *chip = i2c_get_clientdata(client);
	struct max17040_platform_data *pdata = chip->pdata;
#ifdef USE_MAX17040_ALARM
	s3c_program_alarm(chip, SLOW_POLL);
	chip->slow_poll = 1;	
#else
	cancel_delayed_work(&chip->work);
#endif

	if ( !(chip->usb_online) && !(chip->online) ) {
		chip->pdata->charger_enable();

		if(batt_debug_enable)
			printk("%s() : charger_enable() nCHG_EN(%d) \n", __FUNCTION__, gpio_get_value(nCHG_EN));
	}

	for (i = 0; i < pdata->nOutputs; i++) {
		struct max8903_output_desc *output_desc = &pdata->output_desc[i];
		if (output_desc->wakeup) {
			int irq = gpio_to_irq(output_desc->gpio);
			enable_irq_wake(irq);
		}
	}

	return 0;
}

static int max17040_resume(struct i2c_client *client)
{
	int i;
	struct max17040_chip *chip = i2c_get_clientdata(client);
	struct max17040_platform_data *pdata = chip->pdata;

	for (i = 0; i < pdata->nOutputs; i++) {

		struct max8903_output_desc *output_desc = &pdata->output_desc[i];
		if (output_desc->wakeup){
			int irq = gpio_to_irq(output_desc->gpio);
			disable_irq_wake(irq);
		}
	}
#ifdef USE_MAX17040_ALARM
	s3c_program_alarm(chip, FAST_POLL);
	chip->slow_poll = 0;
#else
	schedule_delayed_work(&chip->work, MAX17040_DELAY);
#endif 
	max17040_work(&chip->work);

	return 0;
}
#else

#define max17040_suspend NULL
#define max17040_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17040_id[] = {
	{ "max17040", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17040",
	},
	.probe		= max17040_probe,
	.remove		= __devexit_p(max17040_remove),
	.suspend	= max17040_suspend,
	.resume		= max17040_resume,
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	return i2c_add_driver(&max17040_i2c_driver);
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");

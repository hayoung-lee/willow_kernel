/* linux/arch/arm/mach-exynos/mach-willow.c
 *
 * Copyright (c) 2012 Thinkware Co., Ltd.
 *		http://www.Thinkware.co.kr
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/clk.h>
#include <linux/lcd.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/pwm_backlight.h>
#include <linux/input.h>
#include <linux/mmc/host.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max8649.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/max8997.h>
#include <linux/mfd/max77686.h>
#include <linux/v4l2-mediabus.h>
#include <linux/memblock.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/exynos4.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/keypad.h>
#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/fb-s5p.h>
#include <plat/fb-core.h>
#include <plat/regs-fb-v4.h>
#include <plat/backlight.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-adc.h>
#include <plat/adc.h>
#include <plat/iic.h>
#include <plat/pd.h>
#include <plat/sdhci.h>
#include <plat/mshci.h>
#include <plat/ehci.h>
#include <plat/usbgadget.h>
#include <plat/s3c64xx-spi.h>
#if defined(CONFIG_VIDEO_FIMC)
#include <plat/fimc.h>
#elif defined(CONFIG_VIDEO_SAMSUNG_S5P_FIMC)
#include <plat/fimc-core.h>
#include <media/s5p_fimc.h>
#endif
#if defined(CONFIG_VIDEO_FIMC_MIPI)
#include <plat/csis.h>
#elif defined(CONFIG_VIDEO_S5P_MIPI_CSIS)
#include <plat/mipi_csis.h>
#endif
#include <plat/tvout.h>
#include <plat/media.h>
#include <plat/regs-srom.h>
#include <plat/sysmmu.h>
#include <plat/tv-core.h>
#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC) || defined(CONFIG_VIDEO_MFC5X)
#include <plat/s5p-mfc.h>
#endif

#include <media/s5k4ba_platform.h>
#include <media/s5k4ea_platform.h>
#include <media/exynos_flite.h>
#include <media/exynos_fimc_is.h>
#include <video/platform_lcd.h>
#include <media/m5mo_platform.h>
#include <media/m5mols.h>
#include <mach/board_rev.h>
#include <mach/map.h>
#include <mach/spi-clocks.h>
#include <mach/exynos-ion.h>
#include <mach/regs-pmu.h>
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
#include <mach/dwmci.h>
#endif
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
#include <mach/secmem.h>
#endif
#include <mach/dev.h>
#include <mach/ppmu.h>
#ifdef CONFIG_EXYNOS_C2C
#include <mach/c2c.h>
#endif
#ifdef CONFIG_FB_S5P_MIPI_DSIM
#include <mach/mipi_ddi.h>
#include <mach/dsim.h>
#include <../../../drivers/video/samsung/s3cfb.h>
#endif
#include <plat/fimg2d.h>
#include <mach/dev-sysmmu.h>

#ifdef CONFIG_VIDEO_SAMSUNG_S5P_FIMC
#include <plat/fimc-core.h>
#include <media/s5p_fimc.h>
#endif

#ifdef CONFIG_VIDEO_JPEG_V2X
#include <plat/jpeg.h>
#endif

#include <linux/mfd/s5m87xx/s5m-core.h>
#include <linux/mfd/s5m87xx/s5m-pmic.h>

#if defined(CONFIG_EXYNOS_THERMAL)
#include <mach/tmu.h>
#endif

#include <mach/gpio-willow.h>

#define WILLOW_BOOT_NORMAL			1
#define WILLOW_BOOT_RECOVERY		2
#define WILLOW_BOOT_MMC_RECOVERY	3
#define WILLOW_BOOT_FACTORYTEST_L	4
#define WILLOW_BOOT_FACTORYTEST_H	5
#define REG_INFORM4            (S5P_INFORM4)

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define WILLOW_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define WILLOW_ULCON_DEFAULT	S3C2410_LCON_CS8

#define WILLOW_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg willow_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= WILLOW_UCON_DEFAULT,
		.ulcon		= WILLOW_ULCON_DEFAULT,
		.ufcon		= WILLOW_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= WILLOW_UCON_DEFAULT,
		.ulcon		= WILLOW_ULCON_DEFAULT,
		.ufcon		= WILLOW_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= WILLOW_UCON_DEFAULT,
		.ulcon		= WILLOW_ULCON_DEFAULT,
		.ufcon		= WILLOW_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= WILLOW_UCON_DEFAULT,
		.ulcon		= WILLOW_ULCON_DEFAULT,
		.ufcon		= WILLOW_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_EXYNOS_MEDIA_DEVICE
struct platform_device exynos_device_md0 = {
	.name = "exynos-mdev",
	.id = -1,
};
#endif

#define WRITEBACK_ENABLED

#if defined(CONFIG_VIDEO_FIMC) || defined(CONFIG_VIDEO_SAMSUNG_S5P_FIMC)
/*
 * External camera reset
 * Because the most of cameras take i2c bus signal, so that
 * you have to reset at the boot time for other i2c slave devices.
 * This function also called at fimc_init_camera()
 * Do optimization for cameras on your platform.
*/
#if defined(CONFIG_ITU_A) || defined(CONFIG_CSI_C) \
	|| defined(CONFIG_S5K3H2_CSI_C) || defined(CONFIG_S5K3H7_CSI_C) \
	|| defined(CONFIG_S5K4E5_CSI_C) || defined(CONFIG_S5K6A3_CSI_C)
static int smdk4x12_cam0_reset(int dummy)
{
	int err;
	/* Camera A */
	err = gpio_request(EXYNOS4_GPX1(2), "GPX1");
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_2 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPX1(2), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX1(2), 0);
	gpio_direction_output(EXYNOS4_GPX1(2), 1);
	gpio_free(EXYNOS4_GPX1(2));

	return 0;
}
#endif
#if defined(CONFIG_ITU_B) || defined(CONFIG_CSI_D) \
	|| defined(CONFIG_S5K3H2_CSI_D) || defined(CONFIG_S5K3H7_CSI_D) \
	|| defined(CONFIG_S5K4E5_CSI_D) || defined(CONFIG_S5K6A3_CSI_D)
static int smdk4x12_cam1_reset(int dummy)
{
	int err;

	/* Camera B */
	err = gpio_request(EXYNOS4_GPX1(0), "GPX1");
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_0 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPX1(0), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX1(0), 0);
	gpio_direction_output(EXYNOS4_GPX1(0), 1);
	gpio_free(EXYNOS4_GPX1(0));

	return 0;
}
#endif
#endif

#ifdef CONFIG_VIDEO_FIMC
#ifdef CONFIG_VIDEO_S5K4BA
static struct s5k4ba_platform_data s5k4ba_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info s5k4ba_i2c_info = {
	I2C_BOARD_INFO("S5K4BA", 0x2d),
	.platform_data = &s5k4ba_plat,
};

static struct s3c_platform_camera s5k4ba = {
#ifdef CONFIG_ITU_A
	.id		= CAMERA_PAR_A,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 4,
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_ITU_B
	.id		= CAMERA_PAR_B,
	.clk_name	= "sclk_cam1",
	.i2c_busnum	= 5,
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &s5k4ba_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUYV,
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.line_length	= 1920,
	.width		= 1600,
	.height		= 1200,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1600,
		.height	= 1200,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 1,
	.initialized	= 0,
};
#endif

/* 2 MIPI Cameras */
#ifdef CONFIG_VIDEO_S5K4EA
static struct s5k4ea_platform_data s5k4ea_plat = {
	.default_width = 1920,
	.default_height = 1080,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
};

static struct i2c_board_info s5k4ea_i2c_info = {
	I2C_BOARD_INFO("S5K4EA", 0x2d),
	.platform_data = &s5k4ea_plat,
};

static struct s3c_platform_camera s5k4ea = {
#ifdef CONFIG_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 4,
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.i2c_busnum	= 5,
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.info		= &s5k4ea_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized	= 0,
};
#endif

#ifdef WRITEBACK_ENABLED
static struct i2c_board_info writeback_i2c_info = {
	I2C_BOARD_INFO("WriteBack", 0x0),
};

static struct s3c_platform_camera writeback = {
	.id		= CAMERA_WB,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &writeback_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUV444,
	.line_length	= 800,
	.width		= 480,
	.height		= 800,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 480,
		.height	= 800,
	},

	.initialized	= 0,
};
#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
#ifdef CONFIG_VIDEO_S5K3H2
static struct i2c_board_info s5k3h2_sensor_info = {
	.type = "S5K3H2",
};

static struct s3c_platform_camera s5k3h2 = {
#ifdef CONFIG_S5K3H2_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_RAW10,
	.info		= &s5k3h2_sensor_info,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	.initialized	= 0,
#ifdef CONFIG_S5K3H2_CSI_C
	.flite_id	= FLITE_IDX_A,
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	.flite_id	= FLITE_IDX_B,
#endif
	.use_isp	= true,
#ifdef CONFIG_S5K3H2_CSI_C
	.sensor_index	= 1,
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	.sensor_index	= 101,
#endif
};
#endif

#ifdef CONFIG_VIDEO_S5K3H7
static struct i2c_board_info s5k3h7_sensor_info = {
	.type = "S5K3H7",
};

static struct s3c_platform_camera s5k3h7 = {
#ifdef CONFIG_S5K3H7_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_RAW10,
	.info		= &s5k3h7_sensor_info,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	.initialized	= 0,
#ifdef CONFIG_S5K3H7_CSI_C
	.flite_id	= FLITE_IDX_A,
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	.flite_id	= FLITE_IDX_B,
#endif
	.use_isp	= true,
#ifdef CONFIG_S5K3H7_CSI_C
	.sensor_index	= 4,
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	.sensor_index	= 104,
#endif
};
#endif

#ifdef CONFIG_VIDEO_S5K4E5
static struct i2c_board_info s5k4e5_sensor_info = {
	.type = "S5K4E5",
};

static struct s3c_platform_camera s5k4e5 = {
#ifdef CONFIG_S5K4E5_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_RAW10,
	.info		= &s5k4e5_sensor_info,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	.initialized	= 0,
#ifdef CONFIG_S5K4E5_CSI_C
	.flite_id	= FLITE_IDX_A,
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	.flite_id	= FLITE_IDX_B,
#endif
	.use_isp	= true,
#ifdef CONFIG_S5K4E5_CSI_C
	.sensor_index	= 3,
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	.sensor_index	= 103,
#endif
};
#endif


#ifdef CONFIG_VIDEO_S5K6A3
static struct i2c_board_info s5k6a3_sensor_info = {
	.type = "S5K6A3",
};

static struct s3c_platform_camera s5k6a3 = {
#ifdef CONFIG_S5K6A3_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_RAW10,
	.info		= &s5k6a3_sensor_info,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.mipi_lanes	= 1,
	.mipi_settle	= 18,
	.mipi_align	= 24,

	.initialized	= 0,
#ifdef CONFIG_S5K6A3_CSI_C
	.flite_id	= FLITE_IDX_A,
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	.flite_id	= FLITE_IDX_B,
#endif
	.use_isp	= true,
#ifdef CONFIG_S5K6A3_CSI_C
	.sensor_index	= 2,
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	.sensor_index	= 102,
#endif
};
#endif

#if defined(CONFIG_VIDEO_S5K6A3) && defined(CONFIG_S5K6A3_CSI_D)
static struct i2c_board_info s5k6a3_fd_sensor_info = {
	.type = "S5K6A3_FD",
};

static struct s3c_platform_camera s5k6a3_fd = {
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.cam_power	= smdk4x12_cam1_reset,

	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_RAW10,
	.info		= &s5k6a3_fd_sensor_info,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.mipi_lanes	= 1,
	.mipi_settle	= 18,
	.mipi_align	= 24,

	.initialized	= 0,
	.flite_id	= FLITE_IDX_B,
	.use_isp	= true,
	.sensor_index	= 200
};
#endif

#endif

/* legacy M5MOLS Camera driver configuration */
#ifdef CONFIG_VIDEO_M5MO
#define CAM_CHECK_ERR_RET(x, msg)	\
	if (unlikely((x) < 0)) { \
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x); \
		return x; \
	}
#define CAM_CHECK_ERR(x, msg)	\
		if (unlikely((x) < 0)) { \
			printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x); \
		}

static int m5mo_config_isp_irq(void)
{
	s3c_gpio_cfgpin(EXYNOS4_GPX3(3), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX3(3), S3C_GPIO_PULL_NONE);
	return 0;
}

static struct m5mo_platform_data m5mo_plat = {
	.default_width = 640, /* 1920 */
	.default_height = 480, /* 1080 */
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
	.config_isp_irq = m5mo_config_isp_irq,
	.irq = IRQ_EINT(27),
};

static struct i2c_board_info m5mo_i2c_info = {
	I2C_BOARD_INFO("M5MO", 0x1F),
	.platform_data = &m5mo_plat,
	.irq = IRQ_EINT(27),
};

static struct s3c_platform_camera m5mo = {
#ifdef CONFIG_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 4,
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.i2c_busnum	= 5,
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.info		= &m5mo_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti", /* "mout_mpll" */
	.clk_rate	= 24000000, /* 48000000 */
	.line_length	= 1920,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 1,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
};
#endif

/* Interface setting */
static struct s3c_platform_fimc fimc_plat = {
#ifdef CONFIG_ITU_A
	.default_cam	= CAMERA_PAR_A,
#endif
#ifdef CONFIG_ITU_B
	.default_cam	= CAMERA_PAR_B,
#endif
#ifdef CONFIG_CSI_C
	.default_cam	= CAMERA_CSI_C,
#endif
#ifdef CONFIG_CSI_D
	.default_cam	= CAMERA_CSI_D,
#endif
#ifdef WRITEBACK_ENABLED
	.default_cam	= CAMERA_WB,
#endif
	.camera		= {
#ifdef CONFIG_VIDEO_S5K4BA
		&s5k4ba,
#endif
#ifdef CONFIG_VIDEO_S5K4EA
		&s5k4ea,
#endif
#ifdef CONFIG_VIDEO_M5MO
		&m5mo,
#endif
#ifdef CONFIG_VIDEO_S5K3H2
		&s5k3h2,
#endif
#ifdef CONFIG_VIDEO_S5K3H7
		&s5k3h7,
#endif
#ifdef CONFIG_VIDEO_S5K4E5
		&s5k4e5,
#endif
#ifdef CONFIG_VIDEO_S5K6A3
		&s5k6a3,
#endif
#ifdef WRITEBACK_ENABLED
		&writeback,
#endif
#if defined(CONFIG_VIDEO_S5K6A3) && defined(CONFIG_S5K6A3_CSI_D)
		&s5k6a3_fd,
#endif
	},
	.hw_ver		= 0x51,
};
#endif /* CONFIG_VIDEO_FIMC */

/* for mainline fimc interface */
#ifdef CONFIG_VIDEO_SAMSUNG_S5P_FIMC
#ifdef WRITEBACK_ENABLED
struct writeback_mbus_platform_data {
	int id;
	struct v4l2_mbus_framefmt fmt;
};

static struct i2c_board_info __initdata writeback_info = {
	I2C_BOARD_INFO("writeback", 0x0),
};
#endif

#ifdef CONFIG_VIDEO_S5K4BA
static struct s5k4ba_mbus_platform_data s5k4ba_mbus_plat = {
	.id		= 0,
	.fmt = {
		.width	= 1600,
		.height	= 1200,
		/*.code	= V4L2_MBUS_FMT_UYVY8_2X8, */
		.code	= V4L2_MBUS_FMT_VYUY8_2X8,
	},
	.clk_rate	= 24000000UL,
#ifdef CONFIG_ITU_A
	.set_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_ITU_B
	.set_power	= smdk4x12_cam1_reset,
#endif
};

static struct i2c_board_info s5k4ba_info = {
	I2C_BOARD_INFO("S5K4BA", 0x2d),
	.platform_data = &s5k4ba_mbus_plat,
};
#endif

/* 2 MIPI Cameras */
#ifdef CONFIG_VIDEO_S5K4EA
static struct s5k4ea_mbus_platform_data s5k4ea_mbus_plat = {
#ifdef CONFIG_CSI_C
	.id		= 0,
	.set_power = smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_CSI_D
	.id		= 1,
	.set_power = smdk4x12_cam1_reset,
#endif
	.fmt = {
		.width	= 1920,
		.height	= 1080,
		.code	= V4L2_MBUS_FMT_VYUY8_2X8,
	},
	.clk_rate	= 24000000UL,
};

static struct i2c_board_info s5k4ea_info = {
	I2C_BOARD_INFO("S5K4EA", 0x2d),
	.platform_data = &s5k4ea_mbus_plat,
};
#endif

#ifdef CONFIG_VIDEO_M5MOLS
static struct m5mols_platform_data m5mols_platdata = {
#ifdef CONFIG_CSI_C
	.gpio_rst = EXYNOS4_GPX1(2), /* ISP_RESET */
#endif
#ifdef CONFIG_CSI_D
	.gpio_rst = EXYNOS4_GPX1(0), /* ISP_RESET */
#endif
	.enable_rst = true, /* positive reset */
	.irq = IRQ_EINT(27),
};

static struct i2c_board_info m5mols_board_info = {
	I2C_BOARD_INFO("M5MOLS", 0x1F),
	.platform_data = &m5mols_platdata,
};

#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
#ifdef CONFIG_VIDEO_S5K3H2
static struct i2c_board_info s5k3h2_sensor_info = {
	.type = "S5K3H2",
};
#endif
#ifdef CONFIG_VIDEO_S5K3H7
static struct i2c_board_info s5k3h7_sensor_info = {
	.type = "S5K3H7",
};
#endif
#ifdef CONFIG_VIDEO_S5K4E5
static struct i2c_board_info s5k4e5_sensor_info = {
	.type = "S5K4E5",
};
#endif
#ifdef CONFIG_VIDEO_S5K6A3
static struct i2c_board_info s5k6a3_sensor_info = {
	.type = "S5K6A3",
};
#endif
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
/* This is for platdata of fimc-lite */
#ifdef CONFIG_VIDEO_S5K3H2
static struct s3c_platform_camera s5k3h2 = {
	.type  = CAM_TYPE_MIPI,
	.use_isp = true,
	.inv_pclk = 0,
	.inv_vsync = 0,
	.inv_href = 0,
	.inv_hsync = 0,
};
#endif

#ifdef CONFIG_VIDEO_S5K3H7
static struct s3c_platform_camera s5k3h7 = {
	.type  = CAM_TYPE_MIPI,
	.use_isp = true,
	.inv_pclk = 0,
	.inv_vsync = 0,
	.inv_href = 0,
	.inv_hsync = 0,
};
#endif

#ifdef CONFIG_VIDEO_S5K4E5
static struct s3c_platform_camera s5k4e5 = {
	.type  = CAM_TYPE_MIPI,
	.use_isp = true,
	.inv_pclk = 0,
	.inv_vsync = 0,
	.inv_href = 0,
	.inv_hsync = 0,
};
#endif


#ifdef CONFIG_VIDEO_S5K6A3
static struct s3c_platform_camera s5k6a3 = {
	.type  = CAM_TYPE_MIPI,
	.use_isp = true,
	.inv_pclk = 0,
	.inv_vsync = 0,
	.inv_href = 0,
	.inv_hsync = 0,
};
#endif
#endif
#endif /* CONFIG_VIDEO_SAMSUNG_S5P_FIMC */

#ifdef CONFIG_S3C64XX_DEV_SPI
static struct s3c64xx_spi_csinfo spi0_csi[] = {
	[0] = {
		.line = EXYNOS4_GPB(1),
		.set_level = gpio_set_value,
		.fb_delay = 0x2,
	},
};

static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi0_csi[0],
	}
};

static struct s3c64xx_spi_csinfo spi2_csi[] = {
	[0] = {
		.line = EXYNOS4_GPC1(2),
		.set_level = gpio_set_value,
		.fb_delay = 0x2,
	},
};

static struct spi_board_info spi2_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi2_csi[0],
	}
};
#endif

#ifdef CONFIG_FB_S3C
#ifdef CONFIG_LCD_LTN101AL03
static void lcd_ltn101al03_set_power(struct plat_lcd_data *pd,
				   unsigned int power)
{
	if (power) {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(EXYNOS4_GPD0(1), GPIOF_OUT_INIT_HIGH, "GPD0");
		gpio_free(EXYNOS4_GPD0(1));
#endif
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(EXYNOS4_GPD0(1), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_free(EXYNOS4_GPD0(1));
#endif
	}
}

static struct plat_lcd_data willow_lcd_ltn101al03_data = {
	.set_power		= lcd_ltn101al03_set_power,
};

static struct platform_device willow_lcd_ltn101al03 = {
	.name			= "platform-lcd",
	.dev.parent		= &s5p_device_fimd0.dev,
	.dev.platform_data      = &willow_lcd_ltn101al03_data,
};

static struct s3c_fb_pd_win willow_fb_win0 = {
	.win_mode = {
		.left_margin	= 64,
		.right_margin	= 16,
		.upper_margin	= 12,
		.lower_margin	= 1,
		.hsync_len	= 48,
		.vsync_len	= 3,
		.xres		= 1280, /* real size : 1280 */
		.yres		= 800,
	},
	.virtual_x		= 1280, /* real size : 1280 */
	.virtual_y		= 800 * 2,
	.width			= 223,	//?
	.height			= 125,	//?
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win willow_fb_win1 = {
	.win_mode = {
		.left_margin	= 64,
		.right_margin	= 16,
		.upper_margin	= 12,
		.lower_margin	= 1,
		.hsync_len		= 48,
		.vsync_len		= 3,
		.xres			= 1280, /* real size : 1280 */
		.yres			= 800,
	},
	.virtual_x			= 1280, /* real size : 1280 */
	.virtual_y			= 800 * 2,
	.width				= 223,  //?
	.height				= 125,	//?
	.max_bpp			= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win willow_fb_win2 = {
	.win_mode = {
		.left_margin	= 64,
		.right_margin	= 16,
		.upper_margin	= 12,
		.lower_margin	= 1,
		.hsync_len		= 48,
		.vsync_len		= 3,
		.xres			= 1280, /* real size : 1280 */
		.yres			= 800,
	},
	.virtual_x			= 1280, /* real size : 1280 */
	.virtual_y			= 800 * 2,
	.width				= 223,	//?
	.height				= 125,	//?
	.max_bpp			= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_platdata willow_lcd0_pdata __initdata = {
	.win[0]		= &willow_fb_win0,
	.win[1]		= &willow_fb_win1,
	.win[2]		= &willow_fb_win2,
	.default_win	= 2,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_VCLK | VIDCON1_INV_HSYNC |
			  VIDCON1_INV_VSYNC,
	.setup_gpio	= exynos4_fimd0_gpio_setup_24bpp,
};
#endif
#endif

static void willow_power_off(void)
{
//	if (dc_is_connected ||usb_is_connected) {
		// reboot
//		writel(2, S5P_INFORM6);
//		arm_machine_restart('r', NULL);
//	} else {
		// shutdown
		pr_info("%s: set PS_HOLD low\n", __func__);
		writel(readl(EXYNOS4_PS_HOLD_CONTROL) & 0xFFFFFEFF, EXYNOS4_PS_HOLD_CONTROL);
		while(1);
//	}
}

static void willow_pm_restart(char mode, const char *cmd)
{
  if (cmd!=0){
	  printk("%s : %s\n",__func__,cmd);
    if(strncmp(cmd,"recovery",7)==0){
      writel(WILLOW_BOOT_RECOVERY, S5P_INFORM5);
    }
    else if(strncmp(cmd,"mmc_recovery",11)==0){
      writel(WILLOW_BOOT_MMC_RECOVERY, S5P_INFORM5);
    }
    else if(strncmp(cmd,"FACTORYTEST_L",13)==0){
      writel(WILLOW_BOOT_FACTORYTEST_L, S5P_INFORM5);
    }
    else if(strncmp(cmd,"FACTORYTEST_H",13)==0){
      writel(WILLOW_BOOT_FACTORYTEST_H, S5P_INFORM5);
    }
    //unknown reboot command, anyway set register to normal boot
    else
      writel(WILLOW_BOOT_NORMAL, S5P_INFORM5);
  }
  else
    writel(WILLOW_BOOT_NORMAL, S5P_INFORM5);

	/*
	 * Code from t10s gingerbrad kernel 'arch/arm/kernel/process.c.'
	 * If INFORM6 has '3', bootloader will not enter the charger mode.
	 */
	writel(3, S5P_INFORM6);
	mdelay(50);

	arm_machine_restart(mode, cmd);
}
static int exynos4_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	int mode = 0;

	if ((code == SYS_RESTART) && _cmd)
		if (!strcmp((char *)_cmd, "recovery"))
			mode = 0xf;

	__raw_writel(mode, REG_INFORM4);

	return NOTIFY_DONE;
}

static struct notifier_block exynos4_reboot_notifier = {
	.notifier_call = exynos4_notifier_call,
};

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
static void exynos_dwmci_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
	case 4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
		break;
	case 1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	default:
		break;
	}
}

static struct dw_mci_board exynos_dwmci_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
				MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci_cfg_gpio,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC
static struct s3c_sdhci_platdata willow_hsmmc0_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH0_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC1
static struct s3c_sdhci_platdata willow_hsmmc1_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC2
static struct s3c_sdhci_platdata willow_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC3
static struct s3c_sdhci_platdata willow_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

#ifdef CONFIG_S5P_DEV_MSHC
static struct s3c_mshci_platdata exynos4_mshc_pdata __initdata = {
	.cd_type		= S3C_MSHCI_CD_PERMANENT,
	.has_wp_gpio		= true,
	.wp_gpio		= 0xffffffff,
#if defined(CONFIG_EXYNOS4_MSHC_8BIT) && \
	defined(CONFIG_EXYNOS4_MSHC_DDR)
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR |
				  MMC_CAP_UHS_DDR50,
#elif defined(CONFIG_EXYNOS4_MSHC_8BIT)
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#elif defined(CONFIG_EXYNOS4_MSHC_DDR)
	.host_caps		= MMC_CAP_1_8V_DDR | MMC_CAP_UHS_DDR50,
#endif
};
#endif

#ifdef CONFIG_USB_EHCI_S5P
static struct s5p_ehci_platdata willow_ehci_pdata;

static void __init willow_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &willow_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}
#endif

#ifdef CONFIG_USB_OHCI_S5P
static struct s5p_ohci_platdata willow_ohci_pdata;

static void __init willow_ohci_init(void)
{
	struct s5p_ohci_platdata *pdata = &willow_ohci_pdata;

	s5p_ohci_set_platdata(pdata);
}
#endif

/* USB GADGET */
#ifdef CONFIG_USB_GADGET
static struct s5p_usbgadget_platdata willow_usbgadget_pdata;

static void __init willow_usbgadget_init(void)
{
	struct s5p_usbgadget_platdata *pdata = &willow_usbgadget_pdata;

	s5p_usbgadget_set_platdata(pdata);
}
#endif

/* max77686 */
static struct regulator_consumer_supply max77686_buck1 =
REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply max77686_buck2 =
REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply max77686_buck3 =
REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply max77686_buck4 =
REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply max77686_buck8 =
REGULATOR_SUPPLY("vmmc", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo1_consumer =
REGULATOR_SUPPLY("vdd_alive", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo2_consumer =
REGULATOR_SUPPLY("vdd_m12", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo3_consumer =
REGULATOR_SUPPLY("vdd_io", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo4_consumer =
REGULATOR_SUPPLY("vdd_sd", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo5_consumer =
REGULATOR_SUPPLY("vdd_cam_io", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo6_consumer =
REGULATOR_SUPPLY("vdd_ldo6", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo7_consumer =
REGULATOR_SUPPLY("vdd_pll", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo8_consumer =
REGULATOR_SUPPLY("vdd_mipi_1v0", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo9_consumer =
REGULATOR_SUPPLY("vdd_gps_1v8", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo10_consumer =
REGULATOR_SUPPLY("vdd_mipi_1v8", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo11_consumer =
REGULATOR_SUPPLY("vdd_abb", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo12_consumer =
REGULATOR_SUPPLY("vdd_uotg_3v3", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo13_consumer =
REGULATOR_SUPPLY("vdd_c2c_1v8", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo14_consumer =
REGULATOR_SUPPLY("vdd_mmc_1v8", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo15_consumer =
REGULATOR_SUPPLY("vdd_uotg_1v0", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo16_consumer =
REGULATOR_SUPPLY("vdd_hsic", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo17_consumer =
REGULATOR_SUPPLY("vdd_cam", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo18_consumer =
REGULATOR_SUPPLY("vdd_gps_2v8", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo19_consumer =
REGULATOR_SUPPLY("vdd_ldo19", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo20_consumer =
REGULATOR_SUPPLY("vdd_bt_wifi_io", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo21_consumer =
REGULATOR_SUPPLY("vdd_ldo21", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo22_consumer =
REGULATOR_SUPPLY("vdd_bt_wifi", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo23_consumer =
REGULATOR_SUPPLY("vdd_lcd", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo24_consumer =
REGULATOR_SUPPLY("vdd_sensor", NULL);

static struct regulator_consumer_supply max77686_ldo25_consumer[] = {
	REGULATOR_SUPPLY("DCVDD", "1-001a"),
	REGULATOR_SUPPLY("DBVDD", "1-001a"),
	REGULATOR_SUPPLY("AVDD1", "1-001a"),
	REGULATOR_SUPPLY("AVDD2", "1-001a"),
};

static struct regulator_consumer_supply __initdata max77686_ldo26_consumer =
REGULATOR_SUPPLY("vdd_tsp", NULL);

static struct regulator_init_data max77686_buck1_data = {
	.constraints = {
		.name = "vdd_mif range",
		.min_uV = 800000,
		.max_uV = 1050000,
		.boot_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck1,
};

static struct regulator_init_data max77686_buck2_data = {
	.constraints = {
		.name = "vdd_arm range",
		.min_uV = 800000,
		.max_uV = 1350000,
		.boot_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck2,
};

static struct regulator_init_data max77686_buck3_data = {
	.constraints = {
		.name = "vdd_int range",
		.min_uV = 800000,
		.max_uV = 1150000,
		.boot_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck3,
};

static struct regulator_init_data max77686_buck4_data = {
	.constraints = {
		.name = "vdd_g3d range",
		.min_uV = 850000,
		.max_uV = 1200000,
		.boot_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck4,
};

static struct regulator_init_data max77686_buck8_data = {
	.constraints = { // temporarily boot_on
		.name = "vdd_emmc_2v8 range",
		.min_uV = 2800000,
		.max_uV = 2800000,
		.boot_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled 	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck8,
};

static struct regulator_init_data __initdata max77686_ldo1_data = {
	.constraints	= {
		.name		= "vdd_alive range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.boot_on 	= 1,
		.state_mem	= {
			.uV 		= 1000000,
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo1_consumer,
};

static struct regulator_init_data __initdata max77686_ldo2_data = {
	.constraints	= {
		.name		= "vdd_m12 range",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.boot_on 	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo2_consumer,
};

static struct regulator_init_data __initdata max77686_ldo3_data = {
	.constraints	= {
		.name		= "vdd_io range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.boot_on 	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo3_consumer,
};

static struct regulator_init_data __initdata max77686_ldo4_data = {
	.constraints	= {
		.name		= "vdd_sd range",
		.min_uV		= 800000,
		.max_uV		= 3950000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						  REGULATOR_CHANGE_STATUS,
		.boot_on 	= 1,
		.state_mem	= {
			.uV		= 2800000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo4_consumer,
};

static struct regulator_init_data __initdata max77686_ldo5_data = {
	.constraints	= {
		.name		= "vdd_cam_io range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo5_consumer,
};

static struct regulator_init_data __initdata max77686_ldo6_data = {
	.constraints	= {
		.name		= "vdd_ldo6 range",
		.min_uV		= 800000,
		.max_uV		= 2375000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
							REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo6_consumer,
};

static struct regulator_init_data __initdata max77686_ldo7_data = {
	.constraints	= {
		.name		= "vdd_pll range",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.apply_uV	= 1,
		.boot_on 	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled 	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo7_consumer,
};

static struct regulator_init_data __initdata max77686_ldo8_data = {
	.constraints	= {
		.name		= "vdd_mipi_1v0 range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo8_consumer,
};

static struct regulator_init_data __initdata max77686_ldo9_data = {
	.constraints	= {
		.name		= "vdd_gps_1v8 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo9_consumer,
};

static struct regulator_init_data __initdata max77686_ldo10_data = {
	.constraints	= { //VDDQ_PRE,VDD_ADC
		.name		= "vdd_mipi_1v8 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo10_consumer,
};

static struct regulator_init_data __initdata max77686_ldo11_data = {
	.constraints	= {
		.name		= "vdd_abb_1v8 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.boot_on 	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo11_consumer,
};

static struct regulator_init_data __initdata max77686_ldo12_data = {
	.constraints	= {
		.name		= "vdd_uotg_3v3 range",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo12_consumer,
};

static struct regulator_init_data __initdata max77686_ldo13_data = {
	.constraints	= {
		.name		= "vdd_c2c range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo13_consumer,
};

static struct regulator_init_data __initdata max77686_ldo14_data = {
	.constraints	= { // temporarily always_on
		.name		= "vdd_mmc_1v8 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.boot_on 	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo14_consumer,
};

static struct regulator_init_data __initdata max77686_ldo15_data = {
	.constraints	= {
		.name		= "vdd_uotg_1v0 range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo15_consumer,
};

static struct regulator_init_data __initdata max77686_ldo16_data = {
	.constraints	= {
		.name		= "vdd_hsic range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo16_consumer,
};

static struct regulator_init_data __initdata max77686_ldo17_data = {
	.constraints	= {
		.name		= "vdd_cam range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo17_consumer,
};

static struct regulator_init_data __initdata max77686_ldo18_data = {
	.constraints	= {
		.name		= "vdd_gps_2v8 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo18_consumer,
};

static struct regulator_init_data __initdata max77686_ldo19_data = {
	.constraints	= {
		.name		= "vdd_ldo19 range",
		.min_uV		= 800000,
		.max_uV		= 3950000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
							REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo19_consumer,
};

static struct regulator_init_data __initdata max77686_ldo20_data = {
	.constraints	= {
		.name		= "vdd_bt_wifi_io range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo20_consumer,
};

static struct regulator_init_data __initdata max77686_ldo21_data = {
	.constraints	= {
		.name		= "vdd_ldo21 range",
		.min_uV		= 800000,
		.max_uV		= 3950000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo21_consumer,
};

static struct regulator_init_data __initdata max77686_ldo22_data = {
	.constraints	= {
		.name		= "vdd_bt_wifi range",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo22_consumer,
};

static struct regulator_init_data __initdata max77686_ldo23_data = {
	.constraints	= {
		.name		= "vdd_lcd range",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo23_consumer,
};

static struct regulator_init_data __initdata max77686_ldo24_data = {
	.constraints	= {
		.name		= "vdd_sensor range",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo24_consumer,
};

static struct regulator_init_data __initdata max77686_ldo25_data = {
	.constraints	= {
		.name		= "vdd_audio range",
		.min_uV		= 2900000,
		.max_uV		= 2900000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(max77686_ldo25_consumer),
	.consumer_supplies	= &max77686_ldo25_consumer,
};

static struct regulator_init_data __initdata max77686_ldo26_data = {
	.constraints	= {
		.name		= "vdd_tsp range",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo26_consumer,
};

static struct max77686_regulator_data max77686_regulators[] = {
	{MAX77686_BUCK1, &max77686_buck1_data,},
	{MAX77686_BUCK2, &max77686_buck2_data,},
	{MAX77686_BUCK3, &max77686_buck3_data,},
	{MAX77686_BUCK4, &max77686_buck4_data,},
	{MAX77686_BUCK8, &max77686_buck8_data,},
	{MAX77686_LDO1, &max77686_ldo1_data,},
	{MAX77686_LDO2, &max77686_ldo2_data,},
	{MAX77686_LDO3, &max77686_ldo3_data,},
	{MAX77686_LDO4, &max77686_ldo4_data,},
	{MAX77686_LDO5, &max77686_ldo5_data,},
	{MAX77686_LDO6, &max77686_ldo6_data,},
	{MAX77686_LDO7, &max77686_ldo7_data,},
	{MAX77686_LDO8, &max77686_ldo8_data,},
	{MAX77686_LDO9, &max77686_ldo9_data,},
	{MAX77686_LDO10, &max77686_ldo10_data,},
	{MAX77686_LDO11, &max77686_ldo11_data,},
	{MAX77686_LDO12, &max77686_ldo12_data,},
	{MAX77686_LDO13, &max77686_ldo13_data,},
	{MAX77686_LDO14, &max77686_ldo14_data,},
	{MAX77686_LDO15, &max77686_ldo15_data,},
	{MAX77686_LDO16, &max77686_ldo16_data,},
	{MAX77686_LDO17, &max77686_ldo17_data,},
	{MAX77686_LDO18, &max77686_ldo18_data,},
	{MAX77686_LDO19, &max77686_ldo19_data,},
	{MAX77686_LDO20, &max77686_ldo20_data,},
	{MAX77686_LDO21, &max77686_ldo21_data,},
	{MAX77686_LDO22, &max77686_ldo22_data,},
	{MAX77686_LDO23, &max77686_ldo23_data,},
	{MAX77686_LDO24, &max77686_ldo24_data,},
	{MAX77686_LDO25, &max77686_ldo25_data,},
	{MAX77686_LDO26, &max77686_ldo26_data,},
};

struct max77686_opmode_data max77686_opmode_data[MAX77686_REG_MAX] = {
	[MAX77686_LDO1] = {MAX77686_LDO1, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO2] = {MAX77686_LDO2, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO3] = {MAX77686_LDO3, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO4] = {MAX77686_LDO4, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO5] = {MAX77686_LDO5, MAX77686_OPMODE_LP},
	[MAX77686_LDO6] = {MAX77686_LDO6, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO7] = {MAX77686_LDO7, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO8] = {MAX77686_LDO8, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO9] = {MAX77686_LDO9, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO10] = {MAX77686_LDO10, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO11] = {MAX77686_LDO11, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO12] = {MAX77686_LDO12, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO13] = {MAX77686_LDO13, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO14] = {MAX77686_LDO14, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO15] = {MAX77686_LDO15, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO16] = {MAX77686_LDO16, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO17] = {MAX77686_LDO17, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO18] = {MAX77686_LDO18, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO19] = {MAX77686_LDO19, MAX77686_OPMODE_LP},
	[MAX77686_LDO20] = {MAX77686_LDO20, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO21] = {MAX77686_LDO21, MAX77686_OPMODE_LP},
	[MAX77686_LDO22] = {MAX77686_LDO22, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO23] = {MAX77686_LDO23, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO24] = {MAX77686_LDO24, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO25] = {MAX77686_LDO25, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO26] = {MAX77686_LDO26, MAX77686_OPMODE_NORMAL},
	[MAX77686_BUCK1] = {MAX77686_BUCK1, MAX77686_OPMODE_STANDBY},
	[MAX77686_BUCK2] = {MAX77686_BUCK2, MAX77686_OPMODE_STANDBY},
	[MAX77686_BUCK3] = {MAX77686_BUCK3, MAX77686_OPMODE_STANDBY},
	[MAX77686_BUCK4] = {MAX77686_BUCK4, MAX77686_OPMODE_STANDBY},
	[MAX77686_BUCK8] = {MAX77686_BUCK8, MAX77686_OPMODE_NORMAL},
};

static struct max77686_platform_data exynos4_max77686_info = {
	.num_regulators = ARRAY_SIZE(max77686_regulators),
	.regulators = max77686_regulators,
	.irq_gpio	= 0,
	.irq_base	= 0,
	.wakeup		= 1,

	.opmode_data = max77686_opmode_data,
	.ramp_rate = MAX77686_RAMP_RATE_27MV,

	.buck234_gpio_dvs[0] = PMIC_SET2,
	.buck234_gpio_dvs[1] = PMIC_SET3,
	.buck234_gpio_dvs[2] = PMIC_SET4,

	.buck234_gpio_selb[0] = PMIC_DVS1,
	.buck234_gpio_selb[1] = PMIC_DVS2,
	.buck234_gpio_selb[2] = PMIC_DVS3,

	.buck2_voltage[0] = 1300000,	/* 1.3V */
	.buck2_voltage[1] = 1000000,	/* 1.0V */
	.buck2_voltage[2] = 950000,	/* 0.95V */
	.buck2_voltage[3] = 900000,	/* 0.9V */
	.buck2_voltage[4] = 1000000,	/* 1.0V */
	.buck2_voltage[5] = 1000000,	/* 1.0V */
	.buck2_voltage[6] = 950000,	/* 0.95V */
	.buck2_voltage[7] = 900000,	/* 0.9V */

	.buck3_voltage[0] = 1037500,	/* 1.0375V */
	.buck3_voltage[1] = 1000000,	/* 1.0V */
	.buck3_voltage[2] = 950000,	/* 0.95V */
	.buck3_voltage[3] = 900000,	/* 0.9V */
	.buck3_voltage[4] = 1000000,	/* 1.0V */
	.buck3_voltage[5] = 1000000,	/* 1.0V */
	.buck3_voltage[6] = 950000,	/* 0.95V */
	.buck3_voltage[7] = 900000,	/* 0.9V */

	.buck4_voltage[0] = 1100000,	/* 1.1V */
	.buck4_voltage[1] = 1000000,	/* 1.0V */
	.buck4_voltage[2] = 950000,	/* 0.95V */
	.buck4_voltage[3] = 900000,	/* 0.9V */
	.buck4_voltage[4] = 1000000,	/* 1.0V */
	.buck4_voltage[5] = 1000000,	/* 1.0V */
	.buck4_voltage[6] = 950000,	/* 0.95V */
	.buck4_voltage[7] = 900000,	/* 0.9V */
};

#ifdef CONFIG_VIDEO_S5P_MIPI_CSIS
static struct regulator_consumer_supply mipi_csi_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.0"),
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.1"),
};

static struct regulator_init_data mipi_csi_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(mipi_csi_fixed_voltage_supplies),
	.consumer_supplies	= mipi_csi_fixed_voltage_supplies,
};

static struct fixed_voltage_config mipi_csi_fixed_voltage_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &mipi_csi_fixed_voltage_init_data,
};

static struct platform_device mipi_csi_fixed_voltage = {
	.name		= "reg-fixed-voltage",
	.id		= 3,
	.dev		= {
		.platform_data	= &mipi_csi_fixed_voltage_config,
	},
};
#endif

#ifdef CONFIG_VIDEO_M5MOLS
static struct regulator_consumer_supply m5mols_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("core", NULL),
	REGULATOR_SUPPLY("dig_18", NULL),
	REGULATOR_SUPPLY("d_sensor", NULL),
	REGULATOR_SUPPLY("dig_28", NULL),
	REGULATOR_SUPPLY("a_sensor", NULL),
	REGULATOR_SUPPLY("dig_12", NULL),
};

static struct regulator_init_data m5mols_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(m5mols_fixed_voltage_supplies),
	.consumer_supplies	= m5mols_fixed_voltage_supplies,
};

static struct fixed_voltage_config m5mols_fixed_voltage_config = {
	.supply_name	= "CAM_SENSOR",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &m5mols_fixed_voltage_init_data,
};

static struct platform_device m5mols_fixed_voltage = {
	.name		= "reg-fixed-voltage",
	.id		= 4,
	.dev		= {
		.platform_data	= &m5mols_fixed_voltage_config,
	},
};
#endif


static struct i2c_board_info i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("max77686", (0x12 >> 1)),
		.platform_data	= &exynos4_max77686_info,
	},
};

static struct i2c_board_info i2c_devs1[] __initdata = {
	{
		I2C_BOARD_INFO("wm8985", 0x1a),
	},
};
static void sensor_gpio_init(void){
#if 0
    int err;
	err = gpio_request(EXYNOS4_GPX3(6), "ACC_INT");
    if(err) {
      printk(KERN_ERR "failed to request ACC_INT\n");
    }
#endif 
    gpio_direction_input(EXYNOS4_GPX3(6));
    s3c_gpio_setpull(EXYNOS4_GPX3(6), S3C_GPIO_PULL_DOWN);
    gpio_free(EXYNOS4_GPX3(6));
}

static struct i2c_board_info i2c_devs2[] __initdata = {
#ifdef CONFIG_VIDEO_TVOUT
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
	},
#endif

#ifdef CONFIG_INPUT_YAS_ACCELEROMETER
    {
        I2C_BOARD_INFO("accelerometer", 0x38),
    }
#endif
};

static struct i2c_board_info i2c_devs3[] __initdata = {
	{
	},
};

static struct i2c_board_info i2c_devs5[] __initdata = {
	{
#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_I2C
        I2C_BOARD_INFO("ft5x0x_ts", (0x70>>1)),
#endif
	},
};

static struct i2c_board_info i2c_devs7[] __initdata = {
	{
	},
};

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
};
#endif

static struct gpio_keys_button willow_button[] = {
	{
		.code               = KEY_POWER,
        .gpio               = WILLOW_POWER_KEY,
		.active_low         = 1,
       	.desc               = "gpio-keys: KEY_POWER",
		.type               = EV_KEY,
        .wakeup             = 1,
        .debounce_interval	= 1,
	},
	{
		.code               = KEY_VOLUMEDOWN,
		.gpio               = WILLOW_VOLUM_DOWN,
		.active_low         = 1,
		.desc               = "gpio-keys: KEY_VOLUMEDOWN",
		.type               = EV_KEY,
		.debounce_interval	= 1,
	},
	{
		.code               = KEY_VOLUMEUP,
		.gpio               = WILLOW_VOLUM_UP,
		.active_low         = 1,
		.desc               = "gpio-keys: KEY_VOLUMEUP" ,
		.type               = EV_KEY,
		.debounce_interval	= 1,
	},
};

static struct gpio_keys_platform_data willow_gpiokeys_platform_data = {
	.buttons		= willow_button,
	.nbuttons		= ARRAY_SIZE(willow_button),
};

static struct platform_device willow_gpio_keys = {
	.name	= "gpio-keys",
	.dev	= {
		.platform_data = &willow_gpiokeys_platform_data,
	},
};

static void wilow_gpio_key_cfg(void)
{
	s3c_gpio_cfgpin(WILLOW_POWER_KEY, S3C_GPIO_SFN(0x0));
	s3c_gpio_setpull(WILLOW_POWER_KEY, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(WILLOW_VOLUM_DOWN, S3C_GPIO_SFN(0x0));
	s3c_gpio_setpull(WILLOW_VOLUM_DOWN, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(WILLOW_VOLUM_UP, S3C_GPIO_SFN(0x0));
	s3c_gpio_setpull(WILLOW_VOLUM_UP, S3C_GPIO_PULL_NONE);
}

#ifdef CONFIG_WAKEUP_ASSIST
static struct platform_device wakeup_assist_device = {
	.name   = "wakeup_assist",
};
#endif

#ifdef CONFIG_VIDEO_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.hw_ver = 0x41,
	.parent_clkname = "mout_g2d0",
	.clkname = "sclk_fimg2d",
	.gate_clkname = "fimg2d",
	.clkrate = 201 * 1000000,	/* 200 Mhz */
};
#endif

#ifdef CONFIG_EXYNOS_C2C
struct exynos_c2c_platdata willow_c2c_pdata = {
	.setup_gpio	= NULL,
	.shdmem_addr	= C2C_SHAREDMEM_BASE,
	.shdmem_size	= C2C_MEMSIZE_64,
	.ap_sscm_addr	= NULL,
	.cp_sscm_addr	= NULL,
	.rx_width	= C2C_BUSWIDTH_16,
	.tx_width	= C2C_BUSWIDTH_16,
	.clk_opp100	= 400,
	.clk_opp50	= 266,
	.clk_opp25	= 0,
	.default_opp_mode	= C2C_OPP50,
	.get_c2c_state	= NULL,
	.c2c_sysreg	= S5P_VA_CMU + 0x12000,
};
#endif

#ifdef CONFIG_USB_EXYNOS_SWITCH
static struct s5p_usbswitch_platdata willow_usbswitch_pdata;

static void __init willow_usbswitch_init(void)
{
	struct s5p_usbswitch_platdata *pdata = &willow_usbswitch_pdata;
	int err;

	pdata->gpio_host_detect = EXYNOS4_GPX3(5); /* low active */
	err = gpio_request_one(pdata->gpio_host_detect, GPIOF_IN, "HOST_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request gpio_host_detect\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_host_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_host_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_host_detect);

	pdata->gpio_device_detect = EXYNOS4_GPX3(4); /* high active */
	err = gpio_request_one(pdata->gpio_device_detect, GPIOF_IN, "DEVICE_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request gpio_host_detect for\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_device_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_device_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_device_detect);

	if (samsung_board_rev_is_0_0())
		pdata->gpio_host_vbus = 0;
	else {
		pdata->gpio_host_vbus = EXYNOS4_GPL2(0);
		err = gpio_request_one(pdata->gpio_host_vbus, GPIOF_OUT_INIT_LOW, "HOST_VBUS_CONTROL");
		if (err) {
			printk(KERN_ERR "failed to request gpio_host_vbus\n");
			return;
		}

		s3c_gpio_setpull(pdata->gpio_host_vbus, S3C_GPIO_PULL_NONE);
		gpio_free(pdata->gpio_host_vbus);
	}

	s5p_usbswitch_set_platdata(pdata);
}
#endif

#ifdef CONFIG_BUSFREQ_OPP
/* BUSFREQ to control memory/bus*/
static struct device_domain busfreq;
#endif

static struct platform_device exynos4_busfreq = {
	.id = -1,
	.name = "exynos-busfreq",
};

static struct platform_device *willow_devices[] __initdata = {
	&s3c_device_adc,
	/* Samsung Power Domain */
	&exynos4_device_pd[PD_MFC],
	&exynos4_device_pd[PD_G3D],
	&exynos4_device_pd[PD_LCD0],
	&exynos4_device_pd[PD_CAM],
	&exynos4_device_pd[PD_TV],
	&exynos4_device_pd[PD_GPS],
	&exynos4_device_pd[PD_GPS_ALIVE],
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	&exynos4_device_pd[PD_ISP],
#endif
#ifdef CONFIG_FB_MIPI_DSIM
	&s5p_device_mipi_dsim,
#endif
/* mainline fimd */
#ifdef CONFIG_FB_S3C
	&s5p_device_fimd0,
#ifdef defined(CONFIG_LCD_LTN101AL03)
	&willow_lcd_ltn101al03,
#endif
#endif
#ifdef CONFIG_FB_S5P
	&s3c_device_fb,
#endif
	&s3c_device_wdt,
	&s3c_device_rtc,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,
	&s3c_device_i2c3,
	&s3c_device_i2c4,
	&s3c_device_i2c5,
	&s3c_device_i2c7,
#ifdef CONFIG_USB_EHCI_S5P
	&s5p_device_ehci,
#endif
#ifdef CONFIG_USB_OHCI_S5P
	&s5p_device_ohci,
#endif
#ifdef CONFIG_USB_GADGET
	&s3c_device_usbgadget,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&s3c_device_rndis,
#endif
#ifdef CONFIG_USB_ANDROID
	&s3c_device_android_usb,
	&s3c_device_usb_mass_storage,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	&s3c_device_hsmmc1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif
#ifdef CONFIG_S5P_DEV_MSHC
	&s3c_device_mshci,
#endif
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	&exynos_device_dwmci,
#endif
#ifdef CONFIG_SND_SAMSUNG_AC97
	&exynos_device_ac97,
#endif
#ifdef CONFIG_SND_SAMSUNG_I2S
	&exynos_device_i2s0,
#endif
#ifdef CONFIG_SND_SAMSUNG_PCM
	&exynos_device_pcm0,
#endif
#ifdef CONFIG_SND_SAMSUNG_SPDIF
	&exynos_device_spdif,
#endif
#if defined(CONFIG_SND_SAMSUNG_RP) || defined(CONFIG_SND_SAMSUNG_ALP)
	&exynos_device_srp,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	&exynos4_device_fimc_is,
#endif
#ifdef CONFIG_VIDEO_TVOUT
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_TV
	&s5p_device_i2c_hdmiphy,
	&s5p_device_hdmi,
	&s5p_device_sdo,
	&s5p_device_mixer,
	&s5p_device_cec,
#endif
#if defined(CONFIG_VIDEO_FIMC)
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_fimc3,
/* CONFIG_VIDEO_SAMSUNG_S5P_FIMC is the feature for mainline */
#elif defined(CONFIG_VIDEO_SAMSUNG_S5P_FIMC)
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
#endif
#if defined(CONFIG_VIDEO_FIMC_MIPI)
	&s3c_device_csis0,
	&s3c_device_csis1,
#elif defined(CONFIG_VIDEO_S5P_MIPI_CSIS)
	&s5p_device_mipi_csis0,
	&s5p_device_mipi_csis1,
#endif
#ifdef CONFIG_VIDEO_S5P_MIPI_CSIS
	&mipi_csi_fixed_voltage,
#endif
#ifdef CONFIG_VIDEO_M5MOLS
	&m5mols_fixed_voltage,
#endif

#if defined(CONFIG_VIDEO_MFC5X) || defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
	&s5p_device_mfc,
#endif
#ifdef CONFIG_S5P_SYSTEM_MMU
	&SYSMMU_PLATDEV(g2d_acp),
	&SYSMMU_PLATDEV(fimc0),
	&SYSMMU_PLATDEV(fimc1),
	&SYSMMU_PLATDEV(fimc2),
	&SYSMMU_PLATDEV(fimc3),
	&SYSMMU_PLATDEV(jpeg),
	&SYSMMU_PLATDEV(mfc_l),
	&SYSMMU_PLATDEV(mfc_r),
	&SYSMMU_PLATDEV(tv),
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	&SYSMMU_PLATDEV(is_isp),
	&SYSMMU_PLATDEV(is_drc),
	&SYSMMU_PLATDEV(is_fd),
	&SYSMMU_PLATDEV(is_cpu),
#endif
#endif /* CONFIG_S5P_SYSTEM_MMU */
#ifdef CONFIG_ION_EXYNOS
	&exynos_device_ion,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	&exynos_device_flite0,
	&exynos_device_flite1,
#endif
#ifdef CONFIG_VIDEO_FIMG2D
	&s5p_device_fimg2d,
#endif
#ifdef CONFIG_EXYNOS_MEDIA_DEVICE
	&exynos_device_md0,
#endif
#ifdef CONFIG_VIDEO_JPEG_V2X
	&s5p_device_jpeg,
#endif
	&samsung_asoc_dma,
	&samsung_asoc_idma,
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif
	&samsung_device_keypad,
#ifdef CONFIG_EXYNOS_C2C
	&exynos_device_c2c,
#endif
#ifdef CONFIG_WAKEUP_ASSIST
	&wakeup_assist_device,
#endif
	&willow_gpio_keys,
#ifdef CONFIG_S3C64XX_DEV_SPI
	&exynos_device_spi0,
#ifndef CONFIG_FB_S5P_LMS501KF03
	&exynos_device_spi1,
#endif
	&exynos_device_spi2,
#endif
#ifdef CONFIG_EXYNOS_THERMAL
	&exynos_device_tmu,
#endif
#ifdef CONFIG_S5P_DEV_ACE
	&s5p_device_ace,
#endif
	&exynos4_busfreq,
};

#ifdef CONFIG_EXYNOS_THERMAL
/* below temperature base on the celcius degree */
struct tmu_data exynos_tmu_data __initdata = {
	.ts = {
		.stop_throttle  = 82,
		.start_throttle = 85,
		.stop_warning  = 102,
		.start_warning = 105,
		.start_tripping = 110, /* temp to do tripping */
		.stop_mem_throttle = 80,
		.start_mem_throttle = 85,

		.stop_tc = 13,
		.start_tc = 10,
	},
	.cpulimit = {
		.throttle_freq = 800000,
		.warning_freq = 200000,
	},
	.temp_compensate = {
		.arm_volt = 925000, /* vdd_arm in uV for temperature compensation */
		.bus_volt = 900000, /* vdd_bus in uV for temperature compensation */
		.g3d_volt = 900000, /* vdd_g3d in uV for temperature compensation */
	},
	.efuse_value = 55,
	.slope = 0x10008802,
	.mode = 0,
};
#endif

#if defined(CONFIG_VIDEO_TVOUT)
static struct s5p_platform_hpd hdmi_hpd_data __initdata = {

};
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif

#ifdef CONFIG_VIDEO_EXYNOS_HDMI_CEC
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif

#ifdef CONFIG_VIDEO_SAMSUNG_S5P_FIMC
static struct s5p_fimc_isp_info isp_info[] = {
#if defined(CONFIG_VIDEO_S5K4BA)
	{
		.board_info	= &s5k4ba_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_ITU_601,
#ifdef CONFIG_ITU_A
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
#endif
#ifdef CONFIG_ITU_B
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
#endif
		.flags		= FIMC_CLK_INV_VSYNC,
	},
#endif
#if defined(CONFIG_VIDEO_S5K4EA)
	{
		.board_info	= &s5k4ea_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
#endif
#ifdef CONFIG_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
#endif
		.flags		= FIMC_CLK_INV_VSYNC,
		.csi_data_align = 32,
	},
#endif
#if defined(CONFIG_VIDEO_M5MOLS)
	{
		.board_info	= &m5mols_board_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_CSI_C
		.i2c_bus_num	= 4,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
#endif
#ifdef CONFIG_CSI_D
		.i2c_bus_num	= 5,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
#endif
		.flags		= FIMC_CLK_INV_PCLK | FIMC_CLK_INV_VSYNC,
		.csi_data_align = 32,
	},
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
#if defined(CONFIG_VIDEO_S5K3H2)
	{
		.board_info	= &s5k3h2_sensor_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_S5K3H2_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_A,
		.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K3H2_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_B,
		.cam_power	= smdk4x12_cam1_reset,
#endif
		.flags		= 0,
		.csi_data_align = 24,
		.use_isp	= true,
	},
#endif
#if defined(CONFIG_VIDEO_S5K3H7)
	{
		.board_info	= &s5k3h7_sensor_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_S5K3H7_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_A,
		.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K3H7_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_B,
		.cam_power	= smdk4x12_cam1_reset,
#endif
		.csi_data_align = 24,
		.use_isp	= true,
	},
#endif
#if defined(CONFIG_VIDEO_S5K4E5)
	{
		.board_info	= &s5k4e5_sensor_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_S5K4E5_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_A,
		.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K4E5_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_B,
		.cam_power	= smdk4x12_cam1_reset,
#endif
		.csi_data_align = 24,
		.use_isp	= true,
	},
#endif
#if defined(CONFIG_VIDEO_S5K6A3)
	{
		.board_info	= &s5k6a3_sensor_info,
		.clk_frequency  = 12000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_S5K6A3_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_A,
		.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K6A3_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_B,
		.cam_power	= smdk4x12_cam1_reset,
#endif
		.flags		= 0,
		.csi_data_align = 12,
		.use_isp	= true,
	},
#endif
#endif
#if defined(WRITEBACK_ENABLED)
	{
		.board_info	= &writeback_info,
		.bus_type	= FIMC_LCD_WB,
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flags		= FIMC_CLK_INV_VSYNC,
	},
#endif
};

static void __init smdk4x12_subdev_config(void)
{
	s3c_fimc0_default_data.isp_info[0] = &isp_info[0];
	s3c_fimc0_default_data.isp_info[0]->use_cam = true;
	s3c_fimc0_default_data.isp_info[1] = &isp_info[1];
	s3c_fimc0_default_data.isp_info[1]->use_cam = true;
	/* support using two fimc as one sensore */
	{
		static struct s5p_fimc_isp_info camcording1;
		static struct s5p_fimc_isp_info camcording2;
		memcpy(&camcording1, &isp_info[0], sizeof(struct s5p_fimc_isp_info));
		memcpy(&camcording2, &isp_info[1], sizeof(struct s5p_fimc_isp_info));
		s3c_fimc1_default_data.isp_info[0] = &camcording1;
		s3c_fimc1_default_data.isp_info[0]->use_cam = false;
		s3c_fimc1_default_data.isp_info[1] = &camcording2;
		s3c_fimc1_default_data.isp_info[1]->use_cam = false;
	}
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
#ifdef CONFIG_VIDEO_S5K3H2
#ifdef CONFIG_S5K3H2_CSI_C
	s5p_mipi_csis0_default_data.clk_rate	= 160000000;
	s5p_mipi_csis0_default_data.lanes	= 2;
	s5p_mipi_csis0_default_data.alignment	= 24;
	s5p_mipi_csis0_default_data.hs_settle	= 12;
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	s5p_mipi_csis1_default_data.clk_rate	= 160000000;
	s5p_mipi_csis1_default_data.lanes	= 2;
	s5p_mipi_csis1_default_data.alignment	= 24;
	s5p_mipi_csis1_default_data.hs_settle	= 12;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K3H7
#ifdef CONFIG_S5K3H7_CSI_C
	s5p_mipi_csis0_default_data.clk_rate	= 160000000;
	s5p_mipi_csis0_default_data.lanes	= 2;
	s5p_mipi_csis0_default_data.alignment	= 24;
	s5p_mipi_csis0_default_data.hs_settle	= 12;
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	s5p_mipi_csis1_default_data.clk_rate	= 160000000;
	s5p_mipi_csis1_default_data.lanes	= 2;
	s5p_mipi_csis1_default_data.alignment	= 24;
	s5p_mipi_csis1_default_data.hs_settle	= 12;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K4E5
#ifdef CONFIG_S5K4E5_CSI_C
	s5p_mipi_csis0_default_data.clk_rate	= 160000000;
	s5p_mipi_csis0_default_data.lanes	= 2;
	s5p_mipi_csis0_default_data.alignment	= 24;
	s5p_mipi_csis0_default_data.hs_settle	= 12;
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	s5p_mipi_csis1_default_data.clk_rate	= 160000000;
	s5p_mipi_csis1_default_data.lanes	= 2;
	s5p_mipi_csis1_default_data.alignment	= 24;
	s5p_mipi_csis1_default_data.hs_settle	= 12;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K6A3
#ifdef CONFIG_S5K6A3_CSI_C
	s5p_mipi_csis0_default_data.clk_rate	= 160000000;
	s5p_mipi_csis0_default_data.lanes 	= 1;
	s5p_mipi_csis0_default_data.alignment	= 24;
	s5p_mipi_csis0_default_data.hs_settle	= 12;
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	s5p_mipi_csis1_default_data.clk_rate	= 160000000;
	s5p_mipi_csis1_default_data.lanes 	= 1;
	s5p_mipi_csis1_default_data.alignment	= 24;
	s5p_mipi_csis1_default_data.hs_settle	= 12;
#endif
#endif
#endif
}
static void __init smdk4x12_camera_config(void)
{
	/* CAM A port(b0010) : PCLK, VSYNC, HREF, DATA[0-4] */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPJ0(0), 8, S3C_GPIO_SFN(2));
	/* CAM A port(b0010) : DATA[5-7], CLKOUT(MIPI CAM also), FIELD */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPJ1(0), 5, S3C_GPIO_SFN(2));
	/* CAM B port(b0011) : PCLK, DATA[0-6] */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPM0(0), 8, S3C_GPIO_SFN(3));
	/* CAM B port(b0011) : FIELD, DATA[7]*/
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPM1(0), 2, S3C_GPIO_SFN(3));
	/* CAM B port(b0011) : VSYNC, HREF, CLKOUT*/
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPM2(0), 3, S3C_GPIO_SFN(3));

	/* note : driver strength to max is unnecessary */
#ifdef CONFIG_VIDEO_M5MOLS
	s3c_gpio_cfgpin(EXYNOS4_GPX2(6), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX2(6), S3C_GPIO_PULL_NONE);
#endif
}
#endif /* CONFIG_VIDEO_SAMSUNG_S5P_FIMC */

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
static void __set_flite_camera_config(struct exynos_platform_flite *data,
					u32 active_index, u32 max_cam)
{
	data->active_cam_index = active_index;
	data->num_clients = max_cam;
}

static void __init smdk4x12_set_camera_flite_platdata(void)
{
	int flite0_cam_index = 0;
	int flite1_cam_index = 0;
#ifdef CONFIG_VIDEO_S5K3H2
#ifdef CONFIG_S5K3H2_CSI_C
	exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k3h2;
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k3h2;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K3H7
#ifdef CONFIG_S5K3H7_CSI_C
	exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k3h7;
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k3h7;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K4E5
#ifdef CONFIG_S5K4E5_CSI_C
	exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k4e5;
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k4e5;
#endif
#endif

#ifdef CONFIG_VIDEO_S5K6A3
#ifdef CONFIG_S5K6A3_CSI_C
	exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k6a3;
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k6a3;
#endif
#endif
	__set_flite_camera_config(&exynos_flite0_default_data, 0, flite0_cam_index);
	__set_flite_camera_config(&exynos_flite1_default_data, 0, flite1_cam_index);
}
#endif

#if defined(CONFIG_CMA)
static void __init exynos4_reserve_mem(void)
{
	static struct cma_region regions[] = {
#ifndef CONFIG_VIDEOBUF2_ION
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_TV
		{
			.name = "tv",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_TV * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG
		{
			.name = "jpeg",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP
		{
			.name = "srp",
			.size = CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D
		{
			.name = "fimg2d",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD
		{
			.name = "fimd",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0
		{
			.name = "fimc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2
		{
			.name = "fimc2",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2 * SZ_1K,
			.start = 0
		},
#endif
#if !defined(CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION) && \
	defined(CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3)
		{
			.name = "fimc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3 * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1
		{
			.name = "fimc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC_NORMAL
		{
			.name = "mfc-normal",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC_NORMAL * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1
		{
			.name = "mfc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1 * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0
		{
			.name = "mfc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0 * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC
		{
			.name = "mfc",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
		{
			.name = "fimc_is",
			.size = CONFIG_VIDEO_EXYNOS_MEMSIZE_FIMC_IS * SZ_1K,
			{
				.alignment = 1 << 26,
			},
			.start = 0
		},
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS_BAYER
		{
			.name = "fimc_is_isp",
			.size = CONFIG_VIDEO_EXYNOS_MEMSIZE_FIMC_IS_ISP * SZ_1K,
			.start = 0
		},
#endif
#endif
#if !defined(CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION) && \
	defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
		{
			.name		= "b2",
			.size		= 32 << 20,
			{ .alignment	= 128 << 10 },
		},
		{
			.name		= "b1",
			.size		= 32 << 20,
			{ .alignment	= 128 << 10 },
		},
		{
			.name		= "fw",
			.size		= 1 << 20,
			{ .alignment	= 128 << 10 },
		},
#endif
#else /* !CONFIG_VIDEOBUF2_ION */
#ifdef CONFIG_FB_S5P
#error CONFIG_FB_S5P is defined. Select CONFIG_FB_S3C, instead
#endif
		{
			.name	= "ion",
			.size	= CONFIG_ION_EXYNOS_CONTIGHEAP_SIZE * SZ_1K,
		},
#endif /* !CONFIG_VIDEOBUF2_ION */
		{
			.size = 0
		},
	};
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
	static struct cma_region regions_secure[] = {
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3
		{
			.name = "fimc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3 * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD_VIDEO
		{
			.name = "video",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD_VIDEO * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC_SECURE
		{
			.name = "mfc-secure",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC_SECURE * SZ_1K,
		},
#endif
		{
			.name = "sectbl",
			.size = SZ_1M,
		},
		{
			.size = 0
		},
	};
#else /* !CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION */
	struct cma_region *regions_secure = NULL;
#endif
	static const char map[] __initconst =
#ifdef CONFIG_EXYNOS_C2C
		"samsung-c2c=c2c_shdmem;"
#endif
		"s3cfb.0/fimd=fimd;exynos4-fb.0/fimd=fimd;"
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
		"s3cfb.0/video=video;exynos4-fb.0/video=video;"
#endif
		"s3c-fimc.0=fimc0;s3c-fimc.1=fimc1;s3c-fimc.2=fimc2;s3c-fimc.3=fimc3;"
		"exynos4210-fimc.0=fimc0;exynos4210-fimc.1=fimc1;exynos4210-fimc.2=fimc2;exynos4210-fimc.3=fimc3;"
#ifdef CONFIG_VIDEO_MFC5X
		"s3c-mfc/A=mfc0,mfc-secure;"
		"s3c-mfc/B=mfc1,mfc-normal;"
		"s3c-mfc/AB=mfc;"
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_S5P_MFC
		"s5p-mfc/f=fw;"
		"s5p-mfc/a=b1;"
		"s5p-mfc/b=b2;"
#endif
		"samsung-rp=srp;"
		"s5p-jpeg=jpeg;"
		"exynos4-fimc-is/f=fimc_is;"
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS_BAYER
		"exynos4-fimc-is/i=fimc_is_isp;"
#endif
		"s5p-mixer=tv;"
		"s5p-fimg2d=fimg2d;"
		"ion-exynos=ion,fimd,fimc0,fimc1,fimc2,fimc3,fw,b1,b2;"
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
		"s5p-smem/video=video;"
		"s5p-smem/sectbl=sectbl;"
#endif
		"s5p-smem/mfc=mfc0,mfc-secure;"
		"s5p-smem/fimc=fimc3;"
		"s5p-smem/mfc-shm=mfc1,mfc-normal;"
		"s5p-smem/fimd=fimd;";

	s5p_cma_region_reserve(regions, regions_secure, 0, map);
}
#else
static inline void exynos4_reserve_mem(void)
{
}
#endif /* CONFIG_CMA */

/* LCD Backlight data */
static struct samsung_bl_gpio_info willow_bl_gpio_info = {
	.no = BACKLIGHT_PWM_GPIO,
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data willow_bl_data = {
	.pwm_id = 1,
#ifdef CONFIG_FB_S5P_LMS501KF03
	.pwm_period_ns  = 1000,
#endif
};


static void __init willow_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(willow_uartcfgs, ARRAY_SIZE(willow_uartcfgs));

	exynos4_reserve_mem();
    sensor_gpio_init();
}

static void __init exynos_sysmmu_init(void)
{
	ASSIGN_SYSMMU_POWERDOMAIN(fimc0, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc1, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc2, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc3, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(jpeg, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(mfc_l, &exynos4_device_pd[PD_MFC].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(mfc_r, &exynos4_device_pd[PD_MFC].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(tv, &exynos4_device_pd[PD_TV].dev);
#ifdef CONFIG_VIDEO_FIMG2D
	sysmmu_set_owner(&SYSMMU_PLATDEV(g2d_acp).dev, &s5p_device_fimg2d.dev);
#endif
#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC) || defined(CONFIG_VIDEO_MFC5X)
	sysmmu_set_owner(&SYSMMU_PLATDEV(mfc_l).dev, &s5p_device_mfc.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(mfc_r).dev, &s5p_device_mfc.dev);
#endif
#if defined(CONFIG_VIDEO_FIMC)
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc0).dev, &s3c_device_fimc0.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc1).dev, &s3c_device_fimc1.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc2).dev, &s3c_device_fimc2.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc3).dev, &s3c_device_fimc3.dev);
#elif defined(CONFIG_VIDEO_SAMSUNG_S5P_FIMC)
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc0).dev, &s5p_device_fimc0.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc1).dev, &s5p_device_fimc1.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc2).dev, &s5p_device_fimc2.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc3).dev, &s5p_device_fimc3.dev);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_TV
	sysmmu_set_owner(&SYSMMU_PLATDEV(tv).dev, &s5p_device_mixer.dev);
#endif
#ifdef CONFIG_VIDEO_TVOUT
	sysmmu_set_owner(&SYSMMU_PLATDEV(tv).dev, &s5p_device_tvout.dev);
#endif
#ifdef CONFIG_VIDEO_JPEG_V2X
	sysmmu_set_owner(&SYSMMU_PLATDEV(jpeg).dev, &s5p_device_jpeg.dev);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	ASSIGN_SYSMMU_POWERDOMAIN(is_isp, &exynos4_device_pd[PD_ISP].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(is_drc, &exynos4_device_pd[PD_ISP].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(is_fd, &exynos4_device_pd[PD_ISP].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(is_cpu, &exynos4_device_pd[PD_ISP].dev);

	sysmmu_set_owner(&SYSMMU_PLATDEV(is_isp).dev,
						&exynos4_device_fimc_is.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(is_drc).dev,
						&exynos4_device_fimc_is.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(is_fd).dev,
						&exynos4_device_fimc_is.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(is_cpu).dev,
						&exynos4_device_fimc_is.dev);
#endif
}


#define SMDK4412_REV_0_0_ADC_VALUE 0
#define SMDK4412_REV_0_1_ADC_VALUE 443
int samsung_board_rev;

static int get_samsung_board_rev(void)
{
	int 		adc_val = 0;
	struct clk	*adc_clk;
	struct resource	*res;
	void __iomem	*adc_regs;
	unsigned int	con;
	int		ret;

	if ((soc_is_exynos4412() && samsung_rev() < EXYNOS4412_REV_1_0) ||
		(soc_is_exynos4212() && samsung_rev() < EXYNOS4212_REV_1_0))
		return SAMSUNG_BOARD_REV_0_0;

	adc_clk = clk_get(NULL, "adc");
	if (unlikely(IS_ERR(adc_clk)))
		return SAMSUNG_BOARD_REV_0_0;

	clk_enable(adc_clk);

	res = platform_get_resource(&s3c_device_adc, IORESOURCE_MEM, 0);
	if (unlikely(!res))
		goto err_clk;

	adc_regs = ioremap(res->start, resource_size(res));
	if (unlikely(!adc_regs))
		goto err_clk;

	writel(S5PV210_ADCCON_SELMUX(3), adc_regs + S5PV210_ADCMUX);

	con = readl(adc_regs + S3C2410_ADCCON);
	con &= ~S3C2410_ADCCON_MUXMASK;
	con &= ~S3C2410_ADCCON_STDBM;
	con &= ~S3C2410_ADCCON_STARTMASK;
	con |=  S3C2410_ADCCON_PRSCEN;

	con |= S3C2410_ADCCON_ENABLE_START;
	writel(con, adc_regs + S3C2410_ADCCON);

	udelay (50);

	adc_val = readl(adc_regs + S3C2410_ADCDAT0) & 0xFFF;
	writel(0, adc_regs + S3C64XX_ADCCLRINT);

	iounmap(adc_regs);
err_clk:
	clk_disable(adc_clk);
	clk_put(adc_clk);

	ret = (adc_val < SMDK4412_REV_0_1_ADC_VALUE/2) ?
			SAMSUNG_BOARD_REV_0_0 : SAMSUNG_BOARD_REV_0_1;

	pr_info ("SMDK MAIN Board Rev 0.%d (ADC value:%d)\n", ret, adc_val);
	return ret;
}

static void __init willow_machine_init(void)
{
	arm_pm_restart = willow_pm_restart;
	pm_power_off = willow_power_off;

#ifdef CONFIG_S3C64XX_DEV_SPI
	struct clk *sclk = NULL;
	struct clk *prnt = NULL;
	struct device *spi0_dev = &exynos_device_spi0.dev;
#ifndef CONFIG_FB_S5P_LMS501KF03
	struct device *spi1_dev = &exynos_device_spi1.dev;
#endif
	struct device *spi2_dev = &exynos_device_spi2.dev;
#endif
	samsung_board_rev = get_samsung_board_rev();
#if defined(CONFIG_EXYNOS_DEV_PD) && defined(CONFIG_PM_RUNTIME)
	exynos_pd_disable(&exynos4_device_pd[PD_MFC].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_G3D].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_LCD0].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_CAM].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_TV].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_GPS].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_GPS_ALIVE].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_ISP].dev);
#elif defined(CONFIG_EXYNOS_DEV_PD)
	/*
	 * These power domains should be always on
	 * without runtime pm support.
	 */
	exynos_pd_enable(&exynos4_device_pd[PD_MFC].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_G3D].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_LCD0].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_CAM].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_TV].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_GPS].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_GPS_ALIVE].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_ISP].dev);
#endif
	s3c_i2c0_set_platdata(NULL);

	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));

	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));

	s3c_i2c4_set_platdata(NULL);

	s3c_i2c5_set_platdata(NULL);
	i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));

	s3c_i2c7_set_platdata(NULL);
	i2c_devs7[0].irq = samsung_board_rev_is_0_0() ? IRQ_EINT(15) : IRQ_EINT(22);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));

#if defined(CONFIG_FB_S5P_MIPI_DSIM)
	mipi_fb_init();
#endif
#ifdef CONFIG_FB_S3C
	dev_set_name(&s5p_device_fimd0.dev, "s3cfb.0");
	clk_add_alias("lcd", "exynos4-fb.0", "lcd", &s5p_device_fimd0.dev);
	clk_add_alias("sclk_fimd", "exynos4-fb.0", "sclk_fimd", &s5p_device_fimd0.dev);
	s5p_fb_setname(0, "exynos4-fb");
#if defined(CONFIG_LCD_AMS369FG06) || defined(CONFIG_LCD_LMS501KF03)
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
	s5p_fimd0_set_platdata(&smdk4x12_lcd0_pdata);
#ifdef CONFIG_FB_MIPI_DSIM
	s5p_device_mipi_dsim.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_fimd0.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif
#ifdef CONFIG_FB_S5P
#ifdef CONFIG_FB_S5P_LMS501KF03
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	s3cfb_set_platdata(&lms501kf03_data);
#else
	s3cfb_set_platdata(NULL);
#endif
#ifdef CONFIG_FB_S5P_MIPI_DSIM
	s5p_device_dsim.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_fb.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif
#ifdef CONFIG_USB_EHCI_S5P
	willow_ehci_init();
#endif
#ifdef CONFIG_USB_OHCI_S5P
	willow_ohci_init();
#endif
#ifdef CONFIG_USB_GADGET
	willow_usbgadget_init();
#endif
#ifdef CONFIG_USB_EXYNOS_SWITCH
	willow_usbswitch_init();
#endif

	samsung_bl_set(&willow_bl_gpio_info, &willow_bl_data);

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos_dwmci_set_platdata(&exynos_dwmci_pdata);
#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	exynos4_fimc_is_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	exynos4_device_fimc_is.dev.parent = &exynos4_device_pd[PD_ISP].dev;
#endif
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&willow_hsmmc0_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s3c_sdhci1_set_platdata(&willow_hsmmc1_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&willow_hsmmc2_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s3c_sdhci3_set_platdata(&willow_hsmmc3_pdata);
#endif
#ifdef CONFIG_S5P_DEV_MSHC
	s3c_mshci_set_platdata(&exynos4_mshc_pdata);
#endif
#if defined(CONFIG_VIDEO_EXYNOS_TV) && defined(CONFIG_VIDEO_EXYNOS_HDMI)
	dev_set_name(&s5p_device_hdmi.dev, "exynos4-hdmi");
	clk_add_alias("hdmi", "s5p-hdmi", "hdmi", &s5p_device_hdmi.dev);
	clk_add_alias("hdmiphy", "s5p-hdmi", "hdmiphy", &s5p_device_hdmi.dev);

	s5p_tv_setup();

	/* setup dependencies between TV devices */
	s5p_device_hdmi.dev.parent = &exynos4_device_pd[PD_TV].dev;
	s5p_device_mixer.dev.parent = &exynos4_device_pd[PD_TV].dev;

	s5p_i2c_hdmiphy_set_platdata(NULL);
#ifdef CONFIG_VIDEO_EXYNOS_HDMI_CEC
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
#endif
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	smdk4x12_set_camera_flite_platdata();
	s3c_set_platdata(&exynos_flite0_default_data,
			sizeof(exynos_flite0_default_data), &exynos_device_flite0);
	s3c_set_platdata(&exynos_flite1_default_data,
			sizeof(exynos_flite1_default_data), &exynos_device_flite1);
#ifdef CONFIG_EXYNOS_DEV_PD
	exynos_device_flite0.dev.parent = &exynos4_device_pd[PD_ISP].dev;
	exynos_device_flite1.dev.parent = &exynos4_device_pd[PD_ISP].dev;
#endif
#endif
#ifdef CONFIG_EXYNOS_THERMAL
	exynos_tmu_set_platdata(&exynos_tmu_data);
#endif
#ifdef CONFIG_VIDEO_FIMC
	s3c_fimc0_set_platdata(&fimc_plat);
	s3c_fimc1_set_platdata(&fimc_plat);
	s3c_fimc2_set_platdata(&fimc_plat);
	s3c_fimc3_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
	secmem.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif
#ifdef CONFIG_VIDEO_FIMC_MIPI
	s3c_csis0_set_platdata(NULL);
	s3c_csis1_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_csis0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_csis1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif

#if defined(CONFIG_ITU_A) || defined(CONFIG_CSI_C) \
	|| defined(CONFIG_S5K3H1_CSI_C) || defined(CONFIG_S5K3H2_CSI_C) \
	|| defined(CONFIG_S5K6A3_CSI_C)
	smdk4x12_cam0_reset(1);
#endif
#if defined(CONFIG_ITU_B) || defined(CONFIG_CSI_D) \
	|| defined(CONFIG_S5K3H1_CSI_D) || defined(CONFIG_S5K3H2_CSI_D) \
	|| defined(CONFIG_S5K6A3_CSI_D)
	smdk4x12_cam1_reset(1);
#endif
#endif /* CONFIG_VIDEO_FIMC */

#ifdef CONFIG_VIDEO_SAMSUNG_S5P_FIMC
	smdk4x12_camera_config();
	smdk4x12_subdev_config();

	dev_set_name(&s5p_device_fimc0.dev, "s3c-fimc.0");
	dev_set_name(&s5p_device_fimc1.dev, "s3c-fimc.1");
	dev_set_name(&s5p_device_fimc2.dev, "s3c-fimc.2");
	dev_set_name(&s5p_device_fimc3.dev, "s3c-fimc.3");

	clk_add_alias("fimc", "exynos4210-fimc.0", "fimc", &s5p_device_fimc0.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.0", "sclk_fimc",
			&s5p_device_fimc0.dev);
	clk_add_alias("fimc", "exynos4210-fimc.1", "fimc", &s5p_device_fimc1.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.1", "sclk_fimc",
			&s5p_device_fimc1.dev);
	clk_add_alias("fimc", "exynos4210-fimc.2", "fimc", &s5p_device_fimc2.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.2", "sclk_fimc",
			&s5p_device_fimc2.dev);
	clk_add_alias("fimc", "exynos4210-fimc.3", "fimc", &s5p_device_fimc3.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.3", "sclk_fimc",
			&s5p_device_fimc3.dev);

	s3c_fimc_setname(0, "exynos4210-fimc");
	s3c_fimc_setname(1, "exynos4210-fimc");
	s3c_fimc_setname(2, "exynos4210-fimc");
	s3c_fimc_setname(3, "exynos4210-fimc");
	/* FIMC */
	s3c_set_platdata(&s3c_fimc0_default_data,
			 sizeof(s3c_fimc0_default_data), &s5p_device_fimc0);
	s3c_set_platdata(&s3c_fimc1_default_data,
			 sizeof(s3c_fimc1_default_data), &s5p_device_fimc1);
	s3c_set_platdata(&s3c_fimc2_default_data,
			 sizeof(s3c_fimc2_default_data), &s5p_device_fimc2);
	s3c_set_platdata(&s3c_fimc3_default_data,
			 sizeof(s3c_fimc3_default_data), &s5p_device_fimc3);
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#ifdef CONFIG_VIDEO_S5P_MIPI_CSIS
	dev_set_name(&s5p_device_mipi_csis0.dev, "s3c-csis.0");
	dev_set_name(&s5p_device_mipi_csis1.dev, "s3c-csis.1");
	clk_add_alias("csis", "s5p-mipi-csis.0", "csis",
			&s5p_device_mipi_csis0.dev);
	clk_add_alias("sclk_csis", "s5p-mipi-csis.0", "sclk_csis",
			&s5p_device_mipi_csis0.dev);
	clk_add_alias("csis", "s5p-mipi-csis.1", "csis",
			&s5p_device_mipi_csis1.dev);
	clk_add_alias("sclk_csis", "s5p-mipi-csis.1", "sclk_csis",
			&s5p_device_mipi_csis1.dev);
	dev_set_name(&s5p_device_mipi_csis0.dev, "s5p-mipi-csis.0");
	dev_set_name(&s5p_device_mipi_csis1.dev, "s5p-mipi-csis.1");

	s3c_set_platdata(&s5p_mipi_csis0_default_data,
			sizeof(s5p_mipi_csis0_default_data), &s5p_device_mipi_csis0);
	s3c_set_platdata(&s5p_mipi_csis1_default_data,
			sizeof(s5p_mipi_csis1_default_data), &s5p_device_mipi_csis1);
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_mipi_csis0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_mipi_csis1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif
#if defined(CONFIG_ITU_A) || defined(CONFIG_CSI_C) \
	|| defined(CONFIG_S5K3H1_CSI_C) || defined(CONFIG_S5K3H2_CSI_C) \
	|| defined(CONFIG_S5K6A3_CSI_C)
	smdk4x12_cam0_reset(1);
#endif
#if defined(CONFIG_ITU_B) || defined(CONFIG_CSI_D) \
	|| defined(CONFIG_S5K3H1_CSI_D) || defined(CONFIG_S5K3H2_CSI_D) \
	|| defined(CONFIG_S5K6A3_CSI_D)
	smdk4x12_cam1_reset(1);
#endif
#endif

#if defined(CONFIG_VIDEO_TVOUT)
	s5p_hdmi_hpd_set_platdata(&hdmi_hpd_data);
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_tvout.dev.parent = &exynos4_device_pd[PD_TV].dev;
	exynos4_device_pd[PD_TV].dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif

#ifdef CONFIG_VIDEO_JPEG_V2X
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_jpeg.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	exynos4_jpeg_setup_clock(&s5p_device_jpeg.dev, 160000000);
#endif
#endif

#ifdef CONFIG_ION_EXYNOS
	exynos_ion_set_platdata();
#endif

#if defined(CONFIG_VIDEO_MFC5X) || defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_mfc.dev.parent = &exynos4_device_pd[PD_MFC].dev;
#endif
	if (soc_is_exynos4412() && samsung_rev() >= EXYNOS4412_REV_1_0)
		exynos4_mfc_setup_clock(&s5p_device_mfc.dev, 200 * MHZ);
	else
		exynos4_mfc_setup_clock(&s5p_device_mfc.dev, 267 * MHZ);
#endif

#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
	dev_set_name(&s5p_device_mfc.dev, "s3c-mfc");
	clk_add_alias("mfc", "s5p-mfc", "mfc", &s5p_device_mfc.dev);
	s5p_mfc_setname(&s5p_device_mfc, "s5p-mfc");
#endif

#ifdef CONFIG_VIDEO_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#endif

#ifdef CONFIG_EXYNOS_C2C
	exynos_c2c_set_platdata(&willow_c2c_pdata);
#endif

	exynos_sysmmu_init();

	platform_add_devices(willow_devices, ARRAY_SIZE(willow_devices));

#ifdef CONFIG_FB_S3C
	exynos4_fimd0_setup_clock(&s5p_device_fimd0.dev, "mout_mpll_user",
				800 * MHZ);
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI
	sclk = clk_get(spi0_dev, "dout_spi0");
	if (IS_ERR(sclk))
		dev_err(spi0_dev, "failed to get sclk for SPI-0\n");
	prnt = clk_get(spi0_dev, "mout_mpll_user");
	if (IS_ERR(prnt))
		dev_err(spi0_dev, "failed to get prnt\n");
	if (clk_set_parent(sclk, prnt))
		printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
				prnt->name, sclk->name);

	clk_set_rate(sclk, 800 * 1000 * 1000);
	clk_put(sclk);
	clk_put(prnt);

	if (!gpio_request(EXYNOS4_GPB(1), "SPI_CS0")) {
		gpio_direction_output(EXYNOS4_GPB(1), 1);
		s3c_gpio_cfgpin(EXYNOS4_GPB(1), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(EXYNOS4_GPB(1), S3C_GPIO_PULL_UP);
		exynos_spi_set_info(0, EXYNOS_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi0_csi));
	}

	spi_register_board_info(spi0_board_info, ARRAY_SIZE(spi0_board_info));

#ifndef CONFIG_FB_S5P_LMS501KF03
	sclk = clk_get(spi1_dev, "dout_spi1");
	if (IS_ERR(sclk))
		dev_err(spi1_dev, "failed to get sclk for SPI-1\n");
	prnt = clk_get(spi1_dev, "mout_mpll_user");
	if (IS_ERR(prnt))
		dev_err(spi1_dev, "failed to get prnt\n");
	if (clk_set_parent(sclk, prnt))
		printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
				prnt->name, sclk->name);

	clk_set_rate(sclk, 800 * 1000 * 1000);
	clk_put(sclk);
	clk_put(prnt);

	if (!gpio_request(EXYNOS4_GPB(5), "SPI_CS1")) {
		gpio_direction_output(EXYNOS4_GPB(5), 1);
		s3c_gpio_cfgpin(EXYNOS4_GPB(5), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(EXYNOS4_GPB(5), S3C_GPIO_PULL_UP);
		exynos_spi_set_info(1, EXYNOS_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi1_csi));
	}

	spi_register_board_info(spi1_board_info, ARRAY_SIZE(spi1_board_info));
#endif

	sclk = clk_get(spi2_dev, "dout_spi2");
	if (IS_ERR(sclk))
		dev_err(spi2_dev, "failed to get sclk for SPI-2\n");
	prnt = clk_get(spi2_dev, "mout_mpll_user");
	if (IS_ERR(prnt))
		dev_err(spi2_dev, "failed to get prnt\n");
	if (clk_set_parent(sclk, prnt))
		printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
				prnt->name, sclk->name);

	clk_set_rate(sclk, 800 * 1000 * 1000);
	clk_put(sclk);
	clk_put(prnt);

	if (!gpio_request(EXYNOS4_GPC1(2), "SPI_CS2")) {
		gpio_direction_output(EXYNOS4_GPC1(2), 1);
		s3c_gpio_cfgpin(EXYNOS4_GPC1(2), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(EXYNOS4_GPC1(2), S3C_GPIO_PULL_UP);
		exynos_spi_set_info(2, EXYNOS_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi2_csi));
	}

	spi_register_board_info(spi2_board_info, ARRAY_SIZE(spi2_board_info));
#endif
#ifdef CONFIG_BUSFREQ_OPP
	dev_add(&busfreq, &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC0], &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC1], &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_CPU], &exynos4_busfreq.dev);
#endif

	wilow_gpio_key_cfg();

	register_reboot_notifier(&exynos4_reboot_notifier);
}

#ifdef CONFIG_EXYNOS_C2C
static void __init exynos_c2c_reserve(void)
{
	static struct cma_region region[] = {
		{
			.name = "c2c_shdmem",
			.size = 64 * SZ_1M,
			{ .alignment    = 64 * SZ_1M },
			.start = C2C_SHAREDMEM_BASE
		}, {
		.size = 0,
		}
	};

	s5p_cma_region_reserve(region, NULL, 0, NULL);
}
#endif

MACHINE_START(WILLOW, "WILLOW")
	/* Maintainer: KooHyun Yu <koohyun@thinkware.co.kr>,<koohyun.yu@gmail.com> */
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= willow_map_io,
	.init_machine	= willow_machine_init,
	.timer		= &exynos4_timer,
#ifdef CONFIG_EXYNOS_C2C
	.reserve	= &exynos_c2c_reserve,
#endif
MACHINE_END

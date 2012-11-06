#ifndef USB3503_H
#define USB3503_H

#define USB3503_I2C_CONTROL 0
#define USB3503_I2C_NAME "usb3503"
#define HUB_TAG "usb3503: "

#define CFG1_REG		0x06
#define CFG1_SELF_BUS_PWR	(0x1 << 7)

#define SP_ILOCK_REG		0xE7
#define SPILOCK_CONNECT_N	(0x1 << 1)
#define SPILOCK_CONFIG_N	(0x1 << 0)

#define CFGP_REG		0xEE
#define CFGP_CLKSUSP		(0x1 << 7)

#define PDS_REG			0x0A
#define PDS_PORT1	(0x1 << 1)
#define PDS_PORT2	(0x1 << 2)
#define PDS_PORT3	(0x1 << 3)

enum usb3503_mode {
	USB3503_MODE_UNKNOWN,
	USB3503_MODE_HUB,
	USB3503_MODE_STANDBY,
};

enum dock_status {
	DOCK_STATE_ATTACHED,
	DOCK_STATE_DETACHED,
	DOCK_STATE_UNKNOWN,
};

struct usb3503_platform_data {
#if USB3503_I2C_CONTROL
	char initial_mode;
	int (*register_hub_handler)(void (*)(void), void *);
	int (*port_enable)(int, int);
#endif
	int (*reset_n)(int);
	unsigned int usb_doc_det;
	int cur_dock_status;
	int new_dock_status;
};

struct usb3503_hubctl {
	struct workqueue_struct *workqueue;
	struct work_struct dock_work;
	int dock_irq;
	int cur_dock_status;
	int new_dock_status;
	unsigned int usb_doc_det;
	int (*reset_n)(int);
#if USB3503_I2C_CONTROL
	int mode;
	int (*port_enable)(int, int);
#endif
	struct i2c_client *i2c_dev;
};
#endif

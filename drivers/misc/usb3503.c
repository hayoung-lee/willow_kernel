/*
 * drivers/misc/usb3503.c - usb3503 usb hub driver
 */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/platform_data/usb3503.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/pm_runtime.h>
#include <plat/devs.h>
#include <plat/ehci.h>

#if USB3503_I2C_CONTROL
static int usb3503_register_write(struct i2c_client *i2c_dev, char reg,
	char data)
{
	int ret;
	char buf[2];
	struct i2c_msg msg[] = {
		{
			.addr = i2c_dev->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
	};

	buf[0] = reg;
	buf[1] = data;

	ret = i2c_transfer(i2c_dev->adapter, msg, 1);
	if (ret < 0)
		pr_err(HUB_TAG "%s: reg: %x data: %x write failed\n",
		__func__, reg, data);

	return ret;
}

static int usb3503_register_read(struct i2c_client *i2c_dev, char reg,
	char *data)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = i2c_dev->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = i2c_dev->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = data,
		},
	};

	ret = i2c_transfer(i2c_dev->adapter, msgs, 2);
	if (ret < 0)
		pr_err(HUB_TAG "%s: reg: %x read failed\n", __func__, reg);

	return ret;
}

static int reg_write(struct i2c_client *i2c_dev, char reg, char req, int retry)
{
	int cnt = retry, err;
	char data = 0;

	pr_debug(HUB_TAG "%s: write %02X, data: %02x\n", __func__, reg, req);
	do {
		err = usb3503_register_write(i2c_dev, reg, req);
		if (err < 0) {
			pr_err(HUB_TAG "%s: usb3503_register_write failed"
					" - retry(%d)", __func__, cnt);
			continue;
		}

		err = usb3503_register_read(i2c_dev, reg, &data);
		if (err < 0)
			pr_err(HUB_TAG "%s: usb3503_register_read failed"
					" - retry(%d)", __func__, cnt);
	} while (data != req && cnt--);

	pr_info(HUB_TAG "%s: write %02X, req:%02x, val:%02x\n", __func__, reg,
		req, data);

	return err;
}

static int reg_update(struct i2c_client *i2c_dev, char reg, char req, int retry)
{
	int cnt = retry, err;
	char data;

	pr_debug(HUB_TAG "%s: update %02X, data: %02x\n", __func__, reg, req);
	do {
		err = usb3503_register_read(i2c_dev, reg, &data);
		if (err < 0) {
			pr_err(HUB_TAG "%s: usb3503_register_read failed"
					" - retry(%d)", __func__, cnt);
			continue;
		}

		pr_debug(HUB_TAG "%s: read %02X, data: %02x\n", __func__, reg,
			data);
		if ((data & req) == req) {
			pr_debug(HUB_TAG "%s: aleady set data: %02x\n",
				__func__, data);
			break;
		}
		err = usb3503_register_write(i2c_dev, reg, data | req);
		if (err < 0)
			pr_err(HUB_TAG "%s: usb3503_register_write failed"
					" - retry(%d)", __func__, cnt);
	} while (cnt--);

	pr_info(HUB_TAG "%s: update %02X, req:%02x, val:%02x\n", __func__, reg,
		req, data);
	return err;
}

static int reg_clear(struct i2c_client *i2c_dev, char reg, char req, int retry)
{
	int cnt = retry, err;
	char data;

	pr_debug(HUB_TAG "%s: clear %X, data %x\n", __func__, reg, req);
	do {
		err = usb3503_register_read(i2c_dev, reg, &data);
		if (err < 0)
			goto exit;
		pr_debug(HUB_TAG "%s: read %02X, data %02x\n", __func__, reg,
			data);
		if (!(data & req)) {
			pr_err(HUB_TAG "%s: aleady cleared data = %02x\n",
				__func__, data);
			break;
		}
		err = usb3503_register_write(i2c_dev, reg, data & ~req);
		if (err < 0)
			goto exit;
	} while (cnt--);
exit:
	pr_info(HUB_TAG "%s: clear %02X, req:%02x, val:%02x\n", __func__, reg,
		req, data);
	return err;
}
#endif

static void usb3503_change_status(struct usb3503_hubctl *hc, int force_detached) {
	hc->new_dock_status = gpio_get_value(hc->usb_doc_det);
	if (force_detached)
		hc->cur_dock_status = DOCK_STATE_DETACHED;
	else if (hc->cur_dock_status == hc->new_dock_status)
		return;
    else
		hc->cur_dock_status = hc->new_dock_status;

	if (hc->cur_dock_status == DOCK_STATE_ATTACHED) {
		hc->reset_n(0); s5p_ehci_port_control(&s5p_device_ehci, 2, 0);
		hc->reset_n(1); s5p_ehci_port_control(&s5p_device_ehci, 2, 1);
		pm_runtime_get_sync(&s5p_device_ehci.dev);
		pm_runtime_get_sync(&s5p_device_ohci.dev);
	} else {
		pm_runtime_put(&s5p_device_ohci.dev);
		pm_runtime_put(&s5p_device_ehci.dev);
		s5p_ehci_port_control(&s5p_device_ehci, 2, 0);
		hc->reset_n(0);
	}
}

static void usb3503_dock_worker(struct work_struct *work)
{
	struct usb3503_hubctl *hc = container_of(work, struct usb3503_hubctl, dock_work);

	usb3503_change_status(hc, 0);
	enable_irq(hc->dock_irq);
}

static irqreturn_t usb3503_dock_irq_thread(int irq, void *data) {
	struct usb3503_hubctl *hc = data;

	disable_irq_nosync(hc->dock_irq);
	queue_work(hc->workqueue, &hc->dock_work);

	return IRQ_HANDLED;
}

#if USB3503_I2C_CONTROL
static int usb3503_set_mode(struct usb3503_hubctl *hc, int mode)
{
	int err = 0;
	struct i2c_client *i2c_dev = hc->i2c_dev;

	pr_info(HUB_TAG "%s: mode = %d\n", __func__, mode);

	switch (mode) {
	case USB3503_MODE_HUB:
		hc->reset_n(1);

		/* SP_ILOCK: set connect_n, config_n for config */
		err = reg_write(i2c_dev, SP_ILOCK_REG,
			(SPILOCK_CONNECT_N | SPILOCK_CONFIG_N), 3);
		if (err < 0) {
			pr_err(HUB_TAG "SP_ILOCK write fail err = %d\n", err);
			goto exit;
		}
#ifdef USB3503_ES_VER
/* ES version issue
 * USB3503 can't PLL power up under cold circumstance, so enable
 * the Force suspend clock bit
 */
		err = reg_update(i2c_dev, CFGP_REG, CFGP_CLKSUSP, 1);
		if (err < 0) {
			pr_err(HUB_TAG "CFGP update fail err = %d\n", err);
			goto exit;
		}
#endif
		/* PDS : Port1,3 Disable For Self Powered Operation */
		err = reg_update(i2c_dev, PDS_REG, (PDS_PORT1 | PDS_PORT3), 1);
		if (err < 0) {
			pr_err(HUB_TAG "PDS update fail err = %d\n", err);
			goto exit;
		}
		/* CFG1 : SELF_BUS_PWR -> Self-Powerd operation */
		err = reg_update(i2c_dev, CFG1_REG, CFG1_SELF_BUS_PWR, 1);
		if (err < 0) {
			pr_err(HUB_TAG "CFG1 update fail err = %d\n", err);
			goto exit;
		}
		/* SP_LOCK: clear connect_n, config_n for hub connect */
		err = reg_clear(i2c_dev, SP_ILOCK_REG,
			(SPILOCK_CONNECT_N | SPILOCK_CONFIG_N), 1);
		if (err < 0) {
			pr_err(HUB_TAG "SP_ILOCK clear err = %d\n", err);
			goto exit;
		}
		hc->mode = mode;

		/* Should be enable the HSIC port1 */

		break;

	case USB3503_MODE_STANDBY:
		hc->reset_n(0);
		hc->mode = mode;
		break;

	default:
		pr_err(HUB_TAG "%s: Invalid mode %d\n", __func__, mode);
		err = -EINVAL;
		goto exit;
		break;
	}
exit:
	return err;
}

/* sysfs for control */
static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct usb3503_hubctl *hc = dev_get_drvdata(dev);

	if (hc->mode == USB3503_MODE_HUB)
		return sprintf(buf, "%s", "hub");
	else if (hc->mode == USB3503_MODE_STANDBY)
		return sprintf(buf, "%s", "standby");

	return 0;
}

static ssize_t mode_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct usb3503_hubctl *hc = dev_get_drvdata(dev);

	if (!strncmp(buf, "hub", 3)) {
		if (hc->port_enable)
			hc->port_enable(1, 1);
		pr_debug(HUB_TAG "mode set to hub\n");
	} else if (!strncmp(buf, "standby", 7)) {
		if (hc->port_enable)
			hc->port_enable(1, 0);
		pr_debug(HUB_TAG "mode set to standby\n");
	}
	return size;
}
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
#endif

int usb3503_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct usb3503_hubctl *hc = i2c_get_clientdata(client);
	usb3503_change_status(hc, 1);

	pr_info(HUB_TAG "suspended\n");

	return 0;
}

int usb3503_resume(struct i2c_client *client)
{
	struct usb3503_hubctl *hc = i2c_get_clientdata(client);
	usb3503_change_status(hc, 0);

#if USB3503_I2C_CONTROL
	if (hc->mode == USB3503_MODE_HUB)
		usb3503_set_mode(hc, USB3503_MODE_HUB);

	pr_info(HUB_TAG "resume mode=%s", (hc->mode == USB3503_MODE_HUB) ?
		"hub" : "standny");
#endif

	return 0;
}

int usb3503_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct usb3503_hubctl *hc;
	struct usb3503_platform_data *pdata;

	pr_info(HUB_TAG "%s:%d\n", __func__, __LINE__);

	hc = kzalloc(sizeof(struct usb3503_hubctl), GFP_KERNEL);
	if (!hc) {
		pr_err(HUB_TAG "private data alloc fail\n");
		err = -ENOMEM;
		goto exit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		pr_err(HUB_TAG "device's platform data is NULL!\n");
		err = -ENODEV;
		goto exit;
	}

	hc->cur_dock_status = DOCK_STATE_UNKNOWN;
	hc->new_dock_status = DOCK_STATE_UNKNOWN;
	hc->usb_doc_det = pdata->usb_doc_det;

	hc->i2c_dev = client;
	hc->reset_n = pdata->reset_n;
#if USB3503_I2C_CONTROL
	hc->port_enable = pdata->port_enable;
	if (pdata->initial_mode) {
		usb3503_set_mode(hc, pdata->initial_mode);
		hc->mode = pdata->initial_mode;
	}

	if (pdata->register_hub_handler)
		pdata->register_hub_handler((void (*)(void))usb3503_set_mode,
			(void *)hc);
#endif

	hc->workqueue = create_singlethread_workqueue(USB3503_I2C_NAME);
	INIT_WORK(&hc->dock_work, usb3503_dock_worker);

	hc->dock_irq = gpio_to_irq(pdata->usb_doc_det);
	if (!hc->dock_irq) {
		pr_err(HUB_TAG "Failed to get USB_DOCK_DET IRQ\n");
		err = -ENODEV;
		goto exit;
	}
	err = request_irq(hc->dock_irq, usb3503_dock_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING|IRQF_ONESHOT, "USB_DOCK_DET", hc);
	if (err) {
		pr_err(HUB_TAG "Failed to allocate an USB_DOCK_DET interrupt(%d)\n",
				hc->dock_irq);
		goto exit;
	}

	i2c_set_clientdata(client, hc);

#if USB3503_I2C_CONTROL
	err = device_create_file(&client->dev, &dev_attr_mode);
	pr_info(HUB_TAG "%s: probed on  %s mode\n", __func__,
		(hc->mode == USB3503_MODE_HUB) ? "hub" : "standby");
#endif

	return 0;

exit:
	cancel_work_sync(&hc->dock_work);
	return err;
}

static int usb3503_remove(struct i2c_client *client)
{
	struct usb3503_hubctl *hc = i2c_get_clientdata(client);

	pr_debug(HUB_TAG "%s\n", __func__);
	free_irq(hc->dock_irq, hc->i2c_dev);
	cancel_work_sync(&hc->dock_work);
	kfree(hc);

	return 0;
}

static const struct i2c_device_id usb3503_id[] = {
	{ USB3503_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver usb3503_driver = {
	.probe = usb3503_probe,
	.remove = usb3503_remove,
	.suspend = usb3503_suspend,
	.resume = usb3503_resume,
	.id_table = usb3503_id,
	.driver = {
		.name = USB3503_I2C_NAME,
	},
};

static int __init usb3503_init(void)
{
	pr_info(HUB_TAG "USB HUB driver init\n");
	return i2c_add_driver(&usb3503_driver);
}

static void __exit usb3503_exit(void)
{
	pr_info(HUB_TAG "USB HUB driver exit\n");
	i2c_del_driver(&usb3503_driver);
}
late_initcall(usb3503_init);
module_exit(usb3503_exit);

MODULE_DESCRIPTION("USB3503 USB HUB driver");
MODULE_LICENSE("GPL");

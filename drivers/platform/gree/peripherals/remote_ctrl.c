/*
 * RAON remote / IR controller for the Gree G0245D.
 *
 * Reconstructed from the stock 3.18.20 kernel's
 * drivers/misc/remote_ctrl.c (QFIL/vmlinux DWARF + disassembly):
 *   struct remote_ctrl_data (size 32), struct remote_ctrl_s (size 32),
 *   global _i2c_buf[2048], gDeviceData, rx_lock / tx_lock.
 *
 * The stock driver is an i2c_driver that exposes a raw I2C pass-through
 * character device.  init.qcom.rc chmods /dev/remote_ctrl_dev to 0666 and
 * chowns it to system, so the node must exist with exactly that name.
 *
 * Evidence: work/reverse_drivers/dwarf/dwarf_remote_ctrl.txt,
 *           work/reverse_drivers/raon_disas.txt, raon_strings.txt.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/pm.h>

#define RC_I2C_ADDR		0x50
#define RC_BUF_SIZE		2048
#define RC_CLASS_NAME		"remote_ctrl_class"
#define RC_DEVICE_NAME		"remote_ctrl_dev"
#define RC_CHRDEV_NAME		"remote_ctrl"

#define RC_DT_EN_GPIO		"remote-ctrl,en-gpio"
#define RC_DT_INIT_GPIO		"remote-ctrl,init-gpio"
#define RC_DT_RESET_GPIO	"remote-ctrl,reset-gpio"
#define RC_PINCTRL_DEFAULT	"raon_remote_default"
#define RC_PINCTRL_SLEEP	"raon_remote_sleeps"

/* Vendor ioctl numbers, recovered from remote_ctrl_ioctl(). */
#define RC_IOC_GPIO_1001	0x1001
#define RC_IOC_GPIO_1002	0x1002
#define RC_IOC_GPIO_1003	0x1003
#define RC_IOC_GPIO_1004	0x1004
#define RC_IOC_POWER_ON		0x7205
#define RC_IOC_POWER_OFF	0x7206
#define RC_IOC_I2C_WRITE	0x7207
#define RC_IOC_I2C_WRRD		0x7208

struct remote_ctrl_data {
	struct i2c_client	*client;
	unsigned int		en_gpio;
	u32			en_gpio_flags;
	unsigned int		init_gpio;
	u32			init_gpio_flags;
	unsigned int		reset_gpio;
	u32			reset_gpio_flags;
	struct pinctrl		*pctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;
};

struct remote_ctrl_s {
	struct class	*dev_class;
	dev_t		dev_num;
	struct device	*dev;
	struct cdev	cdev;
};

static struct remote_ctrl_data *gDeviceData;
static struct remote_ctrl_s remote_ctrl;
static u8 _i2c_buf[RC_BUF_SIZE];
static DEFINE_MUTEX(rx_lock);
static DEFINE_MUTEX(tx_lock);

/* ------------------------------------------------------------------ */
/* Character device                                                    */
/* ------------------------------------------------------------------ */

/*
 * remote_ctrl_read(): one I2C_M_RD message with addr 0x50, len = count,
 * then copy the whole _i2c_buf back to userspace.
 */
static ssize_t remote_ctrl_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	struct i2c_msg msg;
	int ret;

	if (!gDeviceData || count == 0)
		return -EINVAL;
	if (count > RC_BUF_SIZE)
		count = RC_BUF_SIZE;

	mutex_lock(&rx_lock);
	memset(_i2c_buf, 0, RC_BUF_SIZE);
	msg.addr = RC_I2C_ADDR;
	msg.flags = I2C_M_RD;
	msg.len = count;
	msg.buf = _i2c_buf;
	ret = i2c_transfer(gDeviceData->client->adapter, &msg, 1);
	if (ret < 0) {
		pr_err("read failed ...\n");
		mutex_unlock(&rx_lock);
		return ret;
	}
	if (copy_to_user(buf, _i2c_buf, count))
		ret = -EFAULT;
	mutex_unlock(&rx_lock);

	return ret;
}

/*
 * remote_ctrl_write(): wake the controller, copy the user payload into
 * _i2c_buf and send it verbatim with i2c_master_send().
 */
static ssize_t remote_ctrl_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	int ret;

	if (!gDeviceData || count == 0)
		return -EINVAL;
	if (count > RC_BUF_SIZE)
		count = RC_BUF_SIZE;

	mutex_lock(&tx_lock);
	memset(_i2c_buf, 0, RC_BUF_SIZE);
	if (copy_from_user(_i2c_buf, buf, count)) {
		mutex_unlock(&tx_lock);
		return -EFAULT;
	}
	ret = i2c_master_send(gDeviceData->client, _i2c_buf, count);
	if (ret >= 0)
		pr_debug("[remote_ir]:remote_ctrl_write   i2c_master_send=%d  success!\n",
			 ret);
	else
		pr_err("[remote_ir]:remote_ctrl_write   i2c_master_send=%d  fail!\n",
		       ret);
	mutex_unlock(&tx_lock);

	return ret;
}

/* Vendor power sequence used by the 0x7205 / 0x7206 ioctls. */
static void rc_power_sequence(bool on)
{
	struct remote_ctrl_data *pdata = gDeviceData;

	if (!pdata)
		return;

	if (on) {
		gpio_direction_output(pdata->init_gpio, 1);
		gpio_set_value(pdata->en_gpio, 1);
		msleep(1);
		gpio_set_value(pdata->reset_gpio, 1);
	} else {
		gpio_direction_output(pdata->init_gpio, 0);
		gpio_set_value(pdata->reset_gpio, 0);
		msleep(1);
		gpio_set_value(pdata->en_gpio, 0);
		msleep(10);
		gpio_set_value(pdata->reset_gpio, 1);
		msleep(64);
		gpio_direction_input(pdata->init_gpio);
	}
}

static long remote_ctrl_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct i2c_msg msg[2];
	int ret = 0;
	int val;
	size_t len;

	if (!gDeviceData)
		return -ENODEV;

	val = (int)(arg & 0x1);
	len = (size_t)(arg & 0xffff);
	if (len == 0 || len > RC_BUF_SIZE)
		len = RC_BUF_SIZE;

	switch (cmd) {
	case RC_IOC_GPIO_1001:
		/* Direction input, sample, optionally drive high again. */
		gpio_direction_input(gDeviceData->init_gpio);
		ret = gpio_get_value(gDeviceData->init_gpio);
		if (val)
			gpio_direction_output(gDeviceData->init_gpio, 1);
		return ret;

	case RC_IOC_GPIO_1002:
		return gpio_get_value(gDeviceData->init_gpio);

	case RC_IOC_GPIO_1003:
		gpio_set_value(gDeviceData->en_gpio, val);
		return 0;

	case RC_IOC_GPIO_1004:
		gpio_set_value(gDeviceData->reset_gpio, val);
		return 0;

	case RC_IOC_POWER_ON:
		rc_power_sequence(true);
		return 0;

	case RC_IOC_POWER_OFF:
		rc_power_sequence(false);
		return 0;

	case RC_IOC_I2C_WRITE:
		mutex_lock(&tx_lock);
		memset(_i2c_buf, 0, RC_BUF_SIZE);
		if (copy_from_user(_i2c_buf, (void __user *)arg, len))
			ret = -EFAULT;
		else
			ret = i2c_master_send(gDeviceData->client,
					      _i2c_buf, len);
		mutex_unlock(&tx_lock);
		return ret;

	case RC_IOC_I2C_WRRD:
		/* Two-message write-then-read transaction on addr 0x50. */
		mutex_lock(&rx_lock);
		memset(_i2c_buf, 0, RC_BUF_SIZE);
		if (copy_from_user(_i2c_buf, (void __user *)arg, len)) {
			mutex_unlock(&rx_lock);
			return -EFAULT;
		}
		msg[0].addr = RC_I2C_ADDR;
		msg[0].flags = 0;
		msg[0].len = len;
		msg[0].buf = _i2c_buf;
		msg[1].addr = RC_I2C_ADDR;
		msg[1].flags = I2C_M_RD;
		msg[1].len = len;
		msg[1].buf = _i2c_buf;
		ret = i2c_transfer(gDeviceData->client->adapter, msg, 2);
		if (ret >= 0 && copy_to_user((void __user *)arg,
					     _i2c_buf, len))
			ret = -EFAULT;
		mutex_unlock(&rx_lock);
		return ret;

	default:
		/* The vendor driver just returns 0 for unknown commands. */
		pr_debug("[remote_ctrl] unsupported ioctl 0x%08x\n", cmd);
		return 0;
	}
}

static int remote_ctrl_open(struct inode *inode, struct file *file)
{
	return gDeviceData ? 0 : -ENODEV;
}

static int remote_ctrl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations remote_ctrl_fops = {
	.owner		= THIS_MODULE,
	.open		= remote_ctrl_open,
	.release	= remote_ctrl_release,
	.read		= remote_ctrl_read,
	.write		= remote_ctrl_write,
	.unlocked_ioctl	= remote_ctrl_ioctl,
	.llseek		= no_llseek,
};

/* ------------------------------------------------------------------ */
/* DT / GPIO / pinctrl                                                 */
/* ------------------------------------------------------------------ */

static int raon_rc_parse_dt(struct device *dev,
			    struct remote_ctrl_data *pdata)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int ret;

	if (!np)
		return -EINVAL;

	ret = of_get_named_gpio_flags(np, RC_DT_EN_GPIO, 0, &flags);
	if (ret < 0)
		return ret;
	pdata->en_gpio = ret;
	pdata->en_gpio_flags = flags;

	ret = of_get_named_gpio_flags(np, RC_DT_INIT_GPIO, 0, &flags);
	if (ret < 0)
		return ret;
	pdata->init_gpio = ret;
	pdata->init_gpio_flags = flags;

	ret = of_get_named_gpio_flags(np, RC_DT_RESET_GPIO, 0, &flags);
	if (ret < 0)
		return ret;
	pdata->reset_gpio = ret;
	pdata->reset_gpio_flags = flags;

	pr_info("[remote_ir]: en_gpio = %d,init_gpio:%d,reset_gpio:%d.\n",
		pdata->en_gpio, pdata->init_gpio, pdata->reset_gpio);
	return 0;
}

static int remote_pinctrl_init(struct i2c_client *client,
			       struct remote_ctrl_data *pdata)
{
	struct device *dev = &client->dev;

	pdata->pctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pdata->pctrl)) {
		pr_err("[remote_ir]:Device does not use pinctrl\n");
		pdata->pctrl = NULL;
		return 0;
	}
	pdata->pins_default = pinctrl_lookup_state(pdata->pctrl,
						   RC_PINCTRL_DEFAULT);
	if (IS_ERR(pdata->pins_default))
		pr_err("[remote_ir]:Can not get ts default pinstate\n");
	pdata->pins_sleep = pinctrl_lookup_state(pdata->pctrl,
						 RC_PINCTRL_SLEEP);
	if (IS_ERR(pdata->pins_sleep))
		pr_err("[remote_ir]:Can not get ts sleep pinstate\n");
	return 0;
}

static int remote_pinctrl_select(bool on)
{
	struct remote_ctrl_data *pdata = gDeviceData;
	struct pinctrl_state *state;
	int ret;

	if (!pdata || !pdata->pctrl)
		return 0;

	state = on ? pdata->pins_default : pdata->pins_sleep;
	if (IS_ERR_OR_NULL(state))
		return 0;
	ret = pinctrl_select_state(pdata->pctrl, state);
	if (ret)
		pr_err("[remote_ir]:can not set %s pins\n",
		       on ? RC_PINCTRL_DEFAULT : RC_PINCTRL_SLEEP);
	return ret;
}

static int rc_request_gpios(struct remote_ctrl_data *pdata)
{
	int ret;

	ret = gpio_request(pdata->en_gpio, "raon_rc_en");
	if (ret)
		return ret;
	ret = gpio_request(pdata->init_gpio, "raon_rc_int");
	if (ret)
		goto free_en;
	ret = gpio_request(pdata->reset_gpio, "raon_rc_reset");
	if (ret)
		goto free_init;

	gpio_direction_output(pdata->en_gpio, 1);
	gpio_direction_output(pdata->init_gpio, 1);
	gpio_direction_output(pdata->reset_gpio, 1);
	gpio_set_value(pdata->reset_gpio, 0);
	msleep(10);
	gpio_set_value(pdata->reset_gpio, 1);
	msleep(10);
	return 0;

free_init:
	gpio_free(pdata->init_gpio);
free_en:
	gpio_free(pdata->en_gpio);
	return ret;
}

static void rc_free_gpios(struct remote_ctrl_data *pdata)
{
	gpio_free(pdata->reset_gpio);
	gpio_free(pdata->init_gpio);
	gpio_free(pdata->en_gpio);
}

/* ------------------------------------------------------------------ */
/* I2C driver                                                          */
/* ------------------------------------------------------------------ */

static int remote_ctrl_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct remote_ctrl_data *pdata;
	int ret;

	pr_info("[remote_ctrl] remote_ctrl_probe \n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[remote_ir]:i2c_check_functionality failed.\n");
		return -ENODEV;
	}

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	pdata->client = client;

	ret = raon_rc_parse_dt(dev, pdata);
	if (ret)
		goto err_free;
	ret = remote_pinctrl_init(client, pdata);
	if (ret)
		goto err_free;

	gDeviceData = pdata;
	remote_pinctrl_select(true);

	ret = rc_request_gpios(pdata);
	if (ret)
		goto err_free_data;

	ret = alloc_chrdev_region(&remote_ctrl.dev_num, 0, 1, RC_CHRDEV_NAME);
	if (ret)
		goto err_gpios;

	remote_ctrl.dev_class = class_create(THIS_MODULE, RC_CLASS_NAME);
	if (IS_ERR(remote_ctrl.dev_class)) {
		ret = PTR_ERR(remote_ctrl.dev_class);
		goto err_unreg;
	}

	remote_ctrl.dev = device_create(remote_ctrl.dev_class, NULL,
					remote_ctrl.dev_num, NULL,
					RC_DEVICE_NAME);
	if (IS_ERR(remote_ctrl.dev)) {
		ret = PTR_ERR(remote_ctrl.dev);
		goto err_class;
	}

	cdev_init(&remote_ctrl.cdev, &remote_ctrl_fops);
	remote_ctrl.cdev.owner = THIS_MODULE;
	ret = cdev_add(&remote_ctrl.cdev, remote_ctrl.dev_num, 1);
	if (ret)
		goto err_device;

	i2c_set_clientdata(client, pdata);
	return 0;

err_device:
	device_destroy(remote_ctrl.dev_class, remote_ctrl.dev_num);
err_class:
	class_destroy(remote_ctrl.dev_class);
err_unreg:
	unregister_chrdev_region(remote_ctrl.dev_num, 1);
err_gpios:
	rc_free_gpios(pdata);
err_free_data:
	gDeviceData = NULL;
err_free:
	kfree(pdata);
	return ret;
}

static int remote_ctrl_remove(struct i2c_client *client)
{
	struct remote_ctrl_data *pdata = i2c_get_clientdata(client);

	cdev_del(&remote_ctrl.cdev);
	device_destroy(remote_ctrl.dev_class, remote_ctrl.dev_num);
	class_destroy(remote_ctrl.dev_class);
	unregister_chrdev_region(remote_ctrl.dev_num, 1);
	if (pdata) {
		rc_free_gpios(pdata);
		kfree(pdata);
	}
	gDeviceData = NULL;
	i2c_set_clientdata(client, NULL);
	return 0;
}

static int remote_ctrl_suspend(struct device *dev)
{
	pr_info("[remote_ir]: IR remote_ctrl_suspend\n");
	remote_pinctrl_select(false);
	return 0;
}

static int remote_ctrl_resume(struct device *dev)
{
	pr_info("[remote_ir]: IR remote_ctrl_resume\n");
	remote_pinctrl_select(true);
	return 0;
}

static const struct dev_pm_ops i2c_device_rc_pm_ops = {
	.suspend	= remote_ctrl_suspend,
	.resume		= remote_ctrl_resume,
};

static const struct i2c_device_id RC_ID[] = {
	{ "raon_remote_ctrl", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, RC_ID);

static const struct of_device_id rc_match_table[] = {
	{ .compatible = "raon,remote-ctrl", },
	{ },
};
MODULE_DEVICE_TABLE(of, rc_match_table);

static struct i2c_driver rc_i2c_driver = {
	.driver = {
		.name		= "raon_remote_ctrl",
		.owner		= THIS_MODULE,
		.of_match_table	= rc_match_table,
		.pm		= &i2c_device_rc_pm_ops,
	},
	.probe		= remote_ctrl_probe,
	.remove		= remote_ctrl_remove,
	.id_table	= RC_ID,
};

static int __init remote_ctrl_init(void)
{
	int ret;

	pr_info("[remote_ctrl] remote_ctrl_init \n");
	ret = i2c_add_driver(&rc_i2c_driver);
	if (ret)
		pr_err("remote_ctrl======i2c failed \n ");
	return ret;
}

static void __exit remote_ctrl_exit(void)
{
	i2c_del_driver(&rc_i2c_driver);
}

module_init(remote_ctrl_init);
module_exit(remote_ctrl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("G0245D CAF bring-up");
MODULE_DESCRIPTION("RAON remote / IR controller (Gree G0245D)");

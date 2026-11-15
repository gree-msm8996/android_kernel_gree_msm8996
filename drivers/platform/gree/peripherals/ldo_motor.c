/*
 * LDO motor vibrator for the Gree G0245D board.
 *
 * Rebuilt from the stock 3.18.20-perf kernel.  See
 * Documentation/g0245d/peripherals.md for the DWARF struct layout, function
 * list, the hard-coded GPIO number recovered from ldo_motor_init and the
 * stock probe messages this driver reproduces.
 *
 * The vendor driver has no DT node: it drives GPIO 85 directly and exposes
 * an Android timed_output device ("vibrator"), a char device ("motordata")
 * and a switch class node.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <linux/switch.h>
#include "../../../staging/android/timed_output.h"

#define LDO_MOTOR_GPIO		85
#define LDO_MOTOR_DEV_NAME	"motordata"
#define LDO_MOTOR_TIMED_NAME	"vibrator"

struct ldo_motor_data {
	unsigned int GpioEnable;
	struct class *class;
	struct device *device;
	dev_t version;
	struct cdev cdev;
	struct switch_dev sw_dev;
	struct wake_lock wklock;
	struct hrtimer timer;
	struct mutex lock;
	struct timed_output_dev to_dev;
};

static struct ldo_motor_data *motordata_store;

static void vibrator_gpio_set_value(unsigned int gpio, int value)
{
	gpio_set_value(gpio, value);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct ldo_motor_data *data = motordata_store;

	if (!data)
		return;

	hrtimer_cancel(&data->timer);
	mutex_lock(&data->lock);

	if (value <= 0) {
		vibrator_gpio_set_value(data->GpioEnable, 0);
		wake_unlock(&data->wklock);
	} else {
		wake_lock(&data->wklock);
		vibrator_gpio_set_value(data->GpioEnable, 1);
		hrtimer_start(&data->timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}

	mutex_unlock(&data->lock);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct ldo_motor_data *data = motordata_store;
	ktime_t rem;

	if (!data || !hrtimer_active(&data->timer))
		return 0;

	rem = hrtimer_get_remaining(&data->timer);
	return (int)ktime_to_ms(rem);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct ldo_motor_data *data = motordata_store;

	if (data) {
		vibrator_gpio_set_value(data->GpioEnable, 0);
		wake_unlock(&data->wklock);
	}

	return HRTIMER_NORESTART;
}

static int dev2605_open(struct inode *i_node, struct file *filp)
{
	return 0;
}

static const struct file_operations fops = {
	.owner	= THIS_MODULE,
	.open	= dev2605_open,
};

static int Haptics_init(void)
{
	struct ldo_motor_data *data = motordata_store;
	int ret;

	ret = alloc_chrdev_region(&data->version, 0, 1, LDO_MOTOR_DEV_NAME);
	if (ret) {
		pr_err("ldo_motor: alloc_chrdev_region failed %d\n", ret);
		return ret;
	}

	data->class = class_create(THIS_MODULE, LDO_MOTOR_DEV_NAME);
	if (IS_ERR(data->class)) {
		pr_err("ldo_motor: class_create failed\n");
		ret = PTR_ERR(data->class);
		goto err_chrdev;
	}

	data->device = device_create(data->class, NULL, data->version, NULL,
				     LDO_MOTOR_DEV_NAME);
	if (IS_ERR(data->device)) {
		pr_err("ldo_motor: device_create failed\n");
		ret = PTR_ERR(data->device);
		goto err_class;
	}

	cdev_init(&data->cdev, &fops);
	data->cdev.owner = THIS_MODULE;
	ret = cdev_add(&data->cdev, data->version, 1);
	if (ret) {
		pr_err("ldo_motor: cdev_add failed %d\n", ret);
		goto err_device;
	}

	data->sw_dev.name = LDO_MOTOR_DEV_NAME;
	ret = switch_dev_register(&data->sw_dev);
	if (ret) {
		pr_err("ldo_motor: switch_dev_register failed %d\n", ret);
		goto err_cdev;
	}

	wake_lock_init(&data->wklock, WAKE_LOCK_SUSPEND, "ldo_motor");
	mutex_init(&data->lock);
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = vibrator_timer_func;

	data->to_dev.name = LDO_MOTOR_TIMED_NAME;
	data->to_dev.enable = vibrator_enable;
	data->to_dev.get_time = vibrator_get_time;
	ret = timed_output_dev_register(&data->to_dev);
	if (ret) {
		pr_err("ldo_motor: timed_output_dev_register failed %d\n", ret);
		goto err_switch;
	}

	return 0;

err_switch:
	switch_dev_unregister(&data->sw_dev);
	wake_lock_destroy(&data->wklock);
err_cdev:
	cdev_del(&data->cdev);
err_device:
	device_destroy(data->class, data->version);
err_class:
	class_destroy(data->class);
err_chrdev:
	unregister_chrdev_region(data->version, 1);
	return ret;
}

static int __init ldo_motor_init(void)
{
	struct ldo_motor_data *data;
	int ret;

	pr_info("ldo_motor_init\n");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	motordata_store = data;
	data->GpioEnable = LDO_MOTOR_GPIO;

	ret = gpio_request(data->GpioEnable, "ldo_motorEnable");
	if (ret) {
		pr_err("ldo_motor: gpio_request failed %d\n", ret);
		goto err_free;
	}
	gpio_direction_output(data->GpioEnable, 0);

	ret = Haptics_init();
	if (ret)
		goto err_gpio;

	pr_info("ldo_motor_init_succeed\n");
	return 0;

err_gpio:
	gpio_free(data->GpioEnable);
err_free:
	motordata_store = NULL;
	kfree(data);
	return ret;
}

static void __exit ldo_motor_exit(void)
{
	struct ldo_motor_data *data = motordata_store;

	if (!data)
		return;

	timed_output_dev_unregister(&data->to_dev);
	switch_dev_unregister(&data->sw_dev);
	wake_lock_destroy(&data->wklock);
	hrtimer_cancel(&data->timer);
	cdev_del(&data->cdev);
	device_destroy(data->class, data->version);
	class_destroy(data->class);
	unregister_chrdev_region(data->version, 1);
	gpio_free(data->GpioEnable);
	motordata_store = NULL;
	kfree(data);
}

module_init(ldo_motor_init);
module_exit(ldo_motor_exit);

MODULE_AUTHOR("G0245D adaptation");
MODULE_DESCRIPTION("LDO motor vibrator (Gree G0245D)");
MODULE_LICENSE("GPL v2");

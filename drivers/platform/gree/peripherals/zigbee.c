/*
 * TI CC2630 Zigbee radio GPIO/character interface for the Gree G0245D board.
 *
 * Rebuilt from the stock 3.18.20-perf kernel.  See
 * Documentation/g0245d/peripherals.md for the DWARF struct layout, function
 * list, DT properties and the stock probe messages this driver reproduces.
 *
 * The stock driver exposes a misc device whose read/write operations report
 * and drive the radio enable GPIO, and switches the two pin groups between
 * "zigbee_active" and "zigbee_suspend" pinctrl states.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/pm.h>

#define ZIGBEE_MISC_NAME	"zigbee"
#define ZIGBEE_PINCTRL_ACTIVE	"zigbee_active"
#define ZIGBEE_PINCTRL_SUSPEND	"zigbee_suspend"

struct zigbee_data {
	struct miscdevice mdev;
	int  en_gpio;
	u32  en_gpio_flags;
	int  reset_gpio;
	u32  reset_gpio_flags;
};

static struct zigbee_data *g_zigbee_data;
static struct semaphore cc2630_zigbee_sem;
static struct pinctrl *zigbee_pinctrl;
static struct pinctrl_state *zigbee_state_active;
static struct pinctrl_state *zigbee_state_suspend;
static int open_zigbee_flag;

static int zigbee_pinctrl_select(bool on)
{
	if (IS_ERR_OR_NULL(zigbee_pinctrl))
		return 0;
	if (IS_ERR_OR_NULL(zigbee_state_active) ||
	    IS_ERR_OR_NULL(zigbee_state_suspend))
		return 0;

	return pinctrl_select_state(zigbee_pinctrl,
			on ? zigbee_state_active : zigbee_state_suspend);
}

static int zigbee_pinctrl_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	zigbee_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(zigbee_pinctrl)) {
		dev_err(dev, "zigbee_pinctrl_init.Can not get ts default pinstate\n");
		return PTR_ERR(zigbee_pinctrl);
	}

	zigbee_state_active = pinctrl_lookup_state(zigbee_pinctrl,
						   ZIGBEE_PINCTRL_ACTIVE);
	if (IS_ERR_OR_NULL(zigbee_state_active))
		dev_err(dev,
			"zigbee_pinctrl_init.Can not get ts default pinstate\n");

	zigbee_state_suspend = pinctrl_lookup_state(zigbee_pinctrl,
						    ZIGBEE_PINCTRL_SUSPEND);
	if (IS_ERR_OR_NULL(zigbee_state_suspend))
		dev_err(dev,
			"zigbee_pinctrl_init.Can not get ts sleep pinstate\n");

	return 0;
}

static int zigbee_open(struct inode *inode, struct file *file)
{
	pr_info("open zigbee.\n");
	open_zigbee_flag = 1;
	return 0;
}

static int zigbee_close(struct inode *inode, struct file *file)
{
	pr_info("close zigbee.\n");
	open_zigbee_flag = 0;
	return 0;
}

static ssize_t zigbee_read(struct file *file, char __user *buf, size_t count,
			   loff_t *offset)
{
	struct zigbee_data *data = g_zigbee_data;
	char tmp[128];
	int len, value;

	if (!data)
		return -ENODEV;

	down(&cc2630_zigbee_sem);
	pr_info("zigbee read\n");
	value = gpio_get_value(data->en_gpio);
	len = snprintf(tmp, sizeof(tmp),
		       "%s en zigbee=%d,read data = %d.\n",
		       __func__, data->en_gpio, value);
	if (len > count)
		len = count;
	if (copy_to_user(buf, tmp, len))
		len = -EFAULT;
	up(&cc2630_zigbee_sem);

	return len;
}

static ssize_t zigbee_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *offset)
{
	struct zigbee_data *data = g_zigbee_data;
	char value;

	if (!data || !count)
		return -ENODEV;

	if (copy_from_user(&value, buf, 1))
		return -EFAULT;

	down(&cc2630_zigbee_sem);
	pr_info("zigbee write\n");
	gpio_direction_output(data->en_gpio, value ? 1 : 0);
	pr_info("%s en zigbee=%d, write '%c'\n",
		__func__, data->en_gpio, value);
	up(&cc2630_zigbee_sem);

	return count;
}

static const struct file_operations zigbee_fops = {
	.owner		= THIS_MODULE,
	.open		= zigbee_open,
	.release	= zigbee_close,
	.read		= zigbee_read,
	.write		= zigbee_write,
};

static struct miscdevice zigbee_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= ZIGBEE_MISC_NAME,
	.fops	= &zigbee_fops,
};

static const struct of_device_id zigbee_of_match[] = {
	{ .compatible = "TI,cc2630" },
	{ }
};
MODULE_DEVICE_TABLE(of, zigbee_of_match);

static int zigbee_platform_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct zigbee_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "zigbee: no memory for device data\n");
		return -ENOMEM;
	}

	data->mdev = zigbee_device;
	g_zigbee_data = data;
	sema_init(&cc2630_zigbee_sem, 1);

	data->en_gpio = of_get_named_gpio_flags(np, "cc2630,en-gpio", 0, NULL);
	data->reset_gpio = of_get_named_gpio_flags(np, "cc2630,reset-gpio", 0,
						   NULL);
	pr_info("huangjq:zigbee en_gpio = %d,reset_gpio = %d.\n",
		data->en_gpio, data->reset_gpio);

	if (!gpio_is_valid(data->en_gpio) ||
	    !gpio_is_valid(data->reset_gpio)) {
		dev_err(&pdev->dev, "zigbee: invalid dt gpios\n");
		ret = -EINVAL;
		goto err_data;
	}

	ret = zigbee_pinctrl_init(pdev);
	if (ret)
		dev_err(&pdev->dev,
			"zigbee_pinctrl_init.Device does not use pinctrl\n");

	zigbee_pinctrl_select(true);

	ret = gpio_request(data->en_gpio, "zigbee_en");
	if (ret) {
		dev_err(&pdev->dev, "zigbee: request en_gpio %d failed\n",
			data->en_gpio);
		goto err_data;
	}
	gpio_direction_output(data->en_gpio, 0);

	ret = gpio_request(data->reset_gpio, "zigbee_rst");
	if (ret) {
		dev_err(&pdev->dev, "zigbee: request reset_gpio %d failed\n",
			data->reset_gpio);
		goto err_en_gpio;
	}
	gpio_direction_output(data->reset_gpio, 0);
	msleep(10);

	ret = misc_register(&data->mdev);
	if (ret) {
		dev_err(&pdev->dev, "zigbee: misc_register failed\n");
		goto err_reset_gpio;
	}

	platform_set_drvdata(pdev, data);
	pr_info("zigbee_platform_probe:probe successfully!\n");
	return 0;

err_reset_gpio:
	gpio_free(data->reset_gpio);
err_en_gpio:
	gpio_free(data->en_gpio);
err_data:
	g_zigbee_data = NULL;
	return ret;
}

static int zigbee_platform_remove(struct platform_device *pdev)
{
	struct zigbee_data *data = platform_get_drvdata(pdev);

	if (!data)
		return 0;

	misc_deregister(&data->mdev);
	gpio_free(data->reset_gpio);
	gpio_free(data->en_gpio);
	platform_set_drvdata(pdev, NULL);
	g_zigbee_data = NULL;
	return 0;
}

static int zigbee_platform_suspend(struct platform_device *dev,
				   pm_message_t state)
{
	return zigbee_pinctrl_select(false);
}

static int zigbee_platform_resume(struct platform_device *dev)
{
	return zigbee_pinctrl_select(true);
}

static struct platform_driver zigbee_platform_driver = {
	.probe		= zigbee_platform_probe,
	.remove		= zigbee_platform_remove,
	.suspend	= zigbee_platform_suspend,
	.resume		= zigbee_platform_resume,
	.driver		= {
		.name		= "zigbee",
		.owner		= THIS_MODULE,
		.of_match_table	= zigbee_of_match,
	},
};

static int __init zigbee_init(void)
{
	return platform_driver_register(&zigbee_platform_driver);
}

static void __exit zigbee_exit(void)
{
	platform_driver_unregister(&zigbee_platform_driver);
}

late_initcall(zigbee_init);
module_exit(zigbee_exit);

MODULE_AUTHOR("G0245D adaptation");
MODULE_DESCRIPTION("TI CC2630 Zigbee radio interface (Gree G0245D)");
MODULE_LICENSE("GPL v2");

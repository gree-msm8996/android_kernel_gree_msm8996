/*
 * TI TUSB320 Type-C controller driver for the Gree G0245D board.
 *
 * Rebuilt from the stock 3.18.20-perf kernel.  The reconstruction evidence
 * (DWARF struct/function data, disassembly of tusb320_probe and the DT node)
 * is recorded in Documentation/g0245d/peripherals.md.
 *
 * Register behaviour recovered from the stock binary:
 *   probe : write(0x0a, read(0x0a) | 0x08)
 *   irq   : status = read(0x09); switch_usb_config(status); write(0x09, 0x10)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/delay.h>

#define TUSB320_REG_CTRL_CONTROL	0x0a
#define TUSB320_REG_INTERRUPT_STATUS	0x09

#define TUSB320_CTRL_DISABLED_INT_MASK	0x08
#define TUSB320_INT_STATUS_CLEAR	0x10

/* Bits 7:6 of the interrupt status register report the attach state. */
#define TUSB320_ATTACH_MASK		0xc0
/* Bit 5 selects the cable orientation on attach. */
#define TUSB320_ORIENT_MASK		0x20

#define TUSB320_AVDD_UV			3000000

static struct i2c_client *tusb320_client;
static int irq_gpio;
static int config_gpio;
static int enable_gpio;
static struct regulator *avdd;

void tusb320_write(u8 reg, u8 data)
{
	u8 buf[2];
	struct i2c_msg msg = {
		.addr	= tusb320_client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= buf,
	};

	if (!tusb320_client)
		return;

	buf[0] = reg;
	buf[1] = data;

	if (i2c_transfer(tusb320_client->adapter, &msg, 1) != 1)
		pr_err("write failed ...\n");
}
EXPORT_SYMBOL_GPL(tusb320_write);

u8 tusb320_read(u8 reg)
{
	u8 val = 0;
	u8 regbuf = reg;
	struct i2c_msg msgs[2] = {
		{
			.addr	= tusb320_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &regbuf,
		},
		{
			.addr	= tusb320_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= &val,
		},
	};

	if (!tusb320_client)
		return 0;

	if (i2c_transfer(tusb320_client->adapter, msgs, 2) != 2)
		pr_err("read failed ...\n");

	return val;
}
EXPORT_SYMBOL_GPL(tusb320_read);

void switch_usb_config(u8 num)
{
	if (!IS_ERR_OR_NULL(avdd) && regulator_enable(avdd))
		pr_err("avdd enable failed\n");

	if (num & TUSB320_ATTACH_MASK) {
		/* Attached: power the switch and drive the orientation. */
		gpio_direction_output(enable_gpio, 1);
		gpio_direction_output(config_gpio,
				      (num & TUSB320_ORIENT_MASK) ? 0 : 1);
	} else {
		/* Detached: put both switches back to high impedance. */
		gpio_direction_input(enable_gpio);
		gpio_direction_input(config_gpio);
	}
}
EXPORT_SYMBOL_GPL(switch_usb_config);

static irqreturn_t tusb320_irq_thread(int irq, void *_ts)
{
	u8 status = tusb320_read(TUSB320_REG_INTERRUPT_STATUS);

	switch_usb_config(status);
	tusb320_write(TUSB320_REG_INTERRUPT_STATUS, TUSB320_INT_STATUS_CLEAR);

	return IRQ_HANDLED;
}

static const struct of_device_id tusb320_match_table[] = {
	{ .compatible = "my,tusb320" },
	{ .compatible = "ti,tusb320" },
	{ }
};
MODULE_DEVICE_TABLE(of, tusb320_match_table);

static const struct i2c_device_id tusb320_i2c_idtable[] = {
	{ "tusb320", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tusb320_i2c_idtable);

static int tusb320_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	int ret, irq;

	tusb320_client = client;

	/* Unmask the attach interrupt, as the stock driver does. */
	tusb320_write(TUSB320_REG_CTRL_CONTROL,
		      tusb320_read(TUSB320_REG_CTRL_CONTROL) |
		      TUSB320_CTRL_DISABLED_INT_MASK);

	irq_gpio = of_get_named_gpio_flags(np, "tusb320,irq-gpio", 0, NULL);
	config_gpio = of_get_named_gpio_flags(np, "tusb320,select-gpio", 0,
					      NULL);
	enable_gpio = of_get_named_gpio_flags(np, "tusb320,enable-gpio", 0,
					      NULL);

	if (!gpio_is_valid(irq_gpio) || !gpio_is_valid(config_gpio) ||
	    !gpio_is_valid(enable_gpio)) {
		pr_err("irq=%d,config=%d,enable=%d\n",
		       irq_gpio, config_gpio, enable_gpio);
		return -EINVAL;
	}

	avdd = regulator_get(&client->dev, "avdd");
	if (IS_ERR(avdd)) {
		pr_err("avdd error\n");
		avdd = NULL;
	} else if (regulator_count_voltages(avdd) > 0 &&
		   regulator_set_voltage(avdd, TUSB320_AVDD_UV,
					 TUSB320_AVDD_UV)) {
		pr_err("avdd set voltage failed\n");
	}

	ret = gpio_request(irq_gpio, "tusb320");
	if (ret) {
		pr_err("request tusb320 irq gpio failed\n");
		goto err_regulator;
	}
	gpio_direction_input(irq_gpio);
	irq = gpio_to_irq(irq_gpio);
	if (irq < 0) {
		pr_err("gpio_to_irq failed\n");
		ret = irq;
		goto err_irq_gpio;
	}

	ret = gpio_request(config_gpio, "tusb320_select_gpio");
	if (ret) {
		pr_err("%s requset select error\n", __func__);
		goto err_irq_gpio;
	}
	gpio_direction_input(config_gpio);

	ret = gpio_request(enable_gpio, "tusb320_enable-gpio");
	if (ret) {
		pr_err("%s requset ebable error\n", __func__);
		goto err_config_gpio;
	}
	gpio_direction_input(enable_gpio);

	ret = request_threaded_irq(irq, NULL, tusb320_irq_thread,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "tusb320", client);
	if (ret) {
		pr_err("request_threaded_irq failed: %d\n", ret);
		goto err_enable_gpio;
	}

	pr_info("%s tusb320 successfully.\n", __func__);
	return 0;

err_enable_gpio:
	gpio_free(enable_gpio);
err_config_gpio:
	gpio_free(config_gpio);
err_irq_gpio:
	gpio_free(irq_gpio);
err_regulator:
	if (!IS_ERR_OR_NULL(avdd))
		regulator_put(avdd);
	avdd = NULL;
	tusb320_client = NULL;
	return ret;
}

static int tusb320_remove(struct i2c_client *client)
{
	int irq = gpio_to_irq(irq_gpio);

	if (irq > 0)
		free_irq(irq, client);
	gpio_free(enable_gpio);
	gpio_free(config_gpio);
	gpio_free(irq_gpio);
	if (!IS_ERR_OR_NULL(avdd))
		regulator_put(avdd);
	avdd = NULL;
	tusb320_client = NULL;
	return 0;
}

static void tusb320_shutdown(struct i2c_client *client)
{
	/* Put the switches back to high impedance on power-off. */
	switch_usb_config(0);
}

static struct i2c_driver tusb320_driver = {
	.driver = {
		.name		= "tusb320",
		.owner		= THIS_MODULE,
		.of_match_table	= tusb320_match_table,
	},
	.probe		= tusb320_probe,
	.remove		= tusb320_remove,
	.shutdown	= tusb320_shutdown,
	.id_table	= tusb320_i2c_idtable,
};

static int __init tusb320_init(void)
{
	return i2c_add_driver(&tusb320_driver);
}

static void __exit tusb320_exit(void)
{
	i2c_del_driver(&tusb320_driver);
}

module_init(tusb320_init);
module_exit(tusb320_exit);

MODULE_AUTHOR("G0245D adaptation");
MODULE_DESCRIPTION("TI TUSB320 Type-C controller (Gree G0245D)");
MODULE_LICENSE("GPL v2");

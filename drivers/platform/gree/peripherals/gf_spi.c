/*
 * Goodix GF816M fingerprint sensor driver for the Gree G0245D.
 *
 * This is a from-scratch reconstruction of the vendor driver
 * drivers/input/touchscreen/gf816m/gf-spi.c.  The vendor kernel is a CAF
 * 3.18.20 tree with the driver built in; only the ELF with DWARF is
 * available.  The character-device ABI, ioctl numbers, SPI wire format,
 * reset timing and DT bindings below were recovered from
 * QFIL/vmlinux (dwarf/dwarf_gf-spi.txt + gf_*_disas.txt) and cross-checked
 * against the stock dmesg and the ramdisk's gx_fpd service.
 *
 * Deliberately not implemented: on-line firmware upgrade (gf-update.c),
 * hw_config() register tables, debugfs/sysfs attribute group and the
 * download/ICE path.  The sensor already carries its factory firmware on
 * this device, and the HAL only needs a working /dev/goodix_fp.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/fb.h>
#include <linux/wakelock.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/ioctl.h>

/* ------------------------------------------------------------------ */
/* Userspace ABI (must match /system/bin/gx_fpd exactly)               */
/* ------------------------------------------------------------------ */

#define GF_IOC_MAGIC		'g'
#define GF_IOC_RESET		_IO(GF_IOC_MAGIC, 0)			/* 0x00006700 */
#define GF_IOC_TRANSFER		_IOWR(GF_IOC_MAGIC, 1, struct gf_ioc_transfer)
								/* 0xc0106701 */
#define GF_IOC_SPEED		_IOW(GF_IOC_MAGIC, 2, u32)		/* 0x40046702 */

struct gf_ioc_transfer {
	u8	cmd;		/* 0xF0 = write, 0xF1 = read */
	u8	reserve;
	u16	addr;
	u32	len;
	u8	*buf;
};

#define GF_CHRDEV_MAJOR		154
#define GF_CHRDEV_MINOR		0
#define GF_CHRDEV_COUNT		256
#define GF_CHRDEV_NAME		"goodix"
#define GF_CLASS_NAME		"goodix-spi"
#define GF_DEVICE_NAME		"goodix_fp"
#define GF_INPUT_NAME		"tiny4412-key"

/* Vendor bufsiz global (DWARF g_debug/bufsiz section). */
#define GF_BUFSIZ		16424
/* A transfer needs 3 header bytes + 1 dummy + 1 read-command byte. */
#define GF_BUFFER_LEN		(GF_BUFSIZ + 5)

#define GF_SPI_MAX_SPEED	8000000		/* GF_IOC_SPEED upper bound */
#define GF_SPEED_PROBE		1000000
#define GF_SPEED_READ		4000000
#define GF_SPEED_WRITE		2000000
#define GF_SPEED_TRANSFER	1000000

/* Sensor registers recovered from gf_probe / gf_irq / gf_read. */
#define GF_REG_IRQ_STATUS	0x8140
#define GF_REG_FIFO		0x8141
#define GF_REG_MODE		0x8043
#define GF_REG_CHIP_ID		0x41E4
#define GF_REG_VERSION		0x8000
#define GF_CHIP_ID		0xBE

#define GF_CMD_WRITE		0xF0
#define GF_CMD_READ		0xF1

/* DT properties (gf_strings.txt, decoded from gf_probe adrp+add pairs). */
#define GF_DT_IRQ_GPIO		"goodix,spi_irq_gpio"
#define GF_DT_RESET_GPIO	"goodix,spi_reset_gpio"
#define GF_PINCTRL_ACTIVE	"pmx_fp_active"
#define GF_PINCTRL_SUSPEND	"pmx_fp_suspend"
#define GF_PINCTRL_RELEASE	"pmx_fp_release"

struct gf_platform_data {
	u32	irq_gpio;
	u32	irq_gpio_flags;
	u32	reset_gpio;
	u32	reset_gpio_flags;
	struct pinctrl		*fp_pinctrl;
	struct pinctrl_state	*active;
	struct pinctrl_state	*suspend;
	struct pinctrl_state	*release;
};

struct gf_dev {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct regulator	*vdd;
	struct spi_device	*spi;
	struct wake_lock	fp_wake_lock;
	struct list_head	device_entry;
	struct gf_platform_data	pdata;
	struct input_dev	*input;
	struct mutex		buf_lock;
	unsigned int		users;
	u8			*buffer;
	u8			buf_status;
	u8			mode;
	struct fasync_struct	*async;
	bool			device_available;
	struct notifier_block	notifier;
	bool			fb_black;
	int			irq;
};

static LIST_HEAD(gf_device_list);
static DEFINE_MUTEX(gf_device_list_lock);
static struct class *gf_spi_class;
static DECLARE_WAIT_QUEUE_HEAD(gf_waitq);

/* ------------------------------------------------------------------ */
/* SPI wire format                                                     */
/* ------------------------------------------------------------------ */

/*
 * gf_spi_setup(): stock writes spi->mode = 0 (SPI_MODE_0),
 * spi->max_speed_hz = speed, spi->bits_per_word = 8, then spi_setup().
 */
static void gf_spi_setup(struct gf_dev *gdev, u32 speed)
{
	gdev->spi->mode = SPI_MODE_0;
	gdev->spi->max_speed_hz = speed;
	gdev->spi->bits_per_word = 8;
	spi_setup(gdev->spi);
}

/*
 * Read frame, recovered from gf_spi_read_bytes():
 *
 *   transfer 1 (tx only, 3 bytes): [0xF0, addr>>8, addr & 0xff]
 *   transfer 2 (full duplex, len+1 bytes starting at buf+4):
 *       MOSI  [0xF1, 0 ...]      MISO [dummy, data0 ... dataN]
 *
 * The vendor driver issues two *separate* SPI transactions (the first via
 * spi_sync, the second via spi_async + wait_for_completion), so chip select
 * is deasserted between the header and the data phase.  Keep that split.
 * Valid data therefore lands at buf[5 .. 5+len-1].
 */
static int gf_spi_read_bytes(struct gf_dev *gdev, u16 addr, u32 len, u8 *buf)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	int ret;

	if (!buf || len + 5 > GF_BUFFER_LEN)
		return -EINVAL;

	buf[0] = GF_CMD_WRITE;
	buf[1] = addr >> 8;
	buf[2] = addr & 0xff;

	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = buf;
	xfer.len = 3;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 5;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(gdev->spi, &msg);
	if (ret)
		return ret;

	buf[4] = GF_CMD_READ;
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = buf + 4;
	xfer.rx_buf = buf + 4;
	xfer.len = len + 1;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 5;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(gdev->spi, &msg);
	if (ret)
		return ret;

	return len;
}

/* Write frame: [0xF0, addr>>8, addr & 0xff, data ...], one transaction. */
static int gf_spi_write_bytes(struct gf_dev *gdev, u16 addr, u32 len, u8 *buf)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	int ret;

	if (!buf || len + 3 > GF_BUFFER_LEN)
		return -EINVAL;

	buf[0] = GF_CMD_WRITE;
	buf[1] = addr >> 8;
	buf[2] = addr & 0xff;

	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = buf;
	xfer.len = len + 3;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 5;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(gdev->spi, &msg);
	if (ret)
		return ret;

	return len;
}

static int gf_spi_read_byte(struct gf_dev *gdev, u16 addr, u8 *value)
{
	int ret;

	mutex_lock(&gdev->buf_lock);
	ret = gf_spi_read_bytes(gdev, addr, 1, gdev->buffer);
	if (ret > 0) {
		*value = gdev->buffer[5];
		ret = 0;
	}
	mutex_unlock(&gdev->buf_lock);
	return ret;
}

static int gf_spi_write_byte(struct gf_dev *gdev, u16 addr, u8 value)
{
	int ret;

	mutex_lock(&gdev->buf_lock);
	gdev->buffer[3] = value;
	ret = gf_spi_write_bytes(gdev, addr, 1, gdev->buffer);
	mutex_unlock(&gdev->buf_lock);
	return ret < 0 ? ret : 0;
}

/*
 * gf_hw_reset.isra.8(): high 5 ms, low ~16 ms, high ~70 ms.  The stock
 * driver uses __const_udelay(0x147AEB8) / __const_udelay(0x418958), i.e.
 * udelay(5000) and 1 ms steps.  probe() additionally sleeps 300 ms after
 * this before the first register read.
 */
static void gf_hw_reset(struct gf_dev *gdev)
{
	int i;

	gpio_set_value(gdev->pdata.reset_gpio, 1);
	udelay(5000);
	gpio_set_value(gdev->pdata.reset_gpio, 0);
	for (i = 0; i < 16; i++)
		udelay(1000);
	gpio_set_value(gdev->pdata.reset_gpio, 1);
	for (i = 0; i < 70; i++)
		udelay(1000);
}

/* ------------------------------------------------------------------ */
/* File operations                                                     */
/* ------------------------------------------------------------------ */

static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gdev = NULL, *pos;

	mutex_lock(&gf_device_list_lock);
	list_for_each_entry(pos, &gf_device_list, device_entry) {
		if (pos->devt == inode->i_rdev) {
			gdev = pos;
			break;
		}
	}
	mutex_unlock(&gf_device_list_lock);

	if (!gdev) {
		pr_err("goodix_fp: no device for %d:%d\n",
		       MAJOR(inode->i_rdev), MINOR(inode->i_rdev));
		return -ENODEV;
	}

	mutex_lock(&gdev->buf_lock);
	if (!gdev->buffer) {
		gdev->buffer = kmalloc(GF_BUFFER_LEN, GFP_KERNEL);
		if (!gdev->buffer) {
			mutex_unlock(&gdev->buf_lock);
			return -ENOMEM;
		}
	}
	mutex_unlock(&gdev->buf_lock);

	gdev->users++;
	filp->private_data = gdev;
	gdev->device_available = true;
	irq_set_irq_wake(gdev->irq, 1);

	pr_debug("goodix_fp: open, spi->irq = %d\n", gdev->spi->irq);
	return nonseekable_open(inode, filp);
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gdev = filp->private_data;

	if (!gdev)
		return 0;

	if (gdev->users > 0)
		gdev->users--;
	gdev->device_available = false;
	irq_set_irq_wake(gdev->irq, 0);
	return 0;
}

/*
 * gf_read(): speed 4 MHz, one burst read from GF_REG_FIFO (0x8141),
 * copy count bytes from buffer+5 to userspace.
 */
static ssize_t gf_read(struct file *filp, char __user *ubuf,
		       size_t count, loff_t *ppos)
{
	struct gf_dev *gdev = filp->private_data;
	int ret;

	if (!gdev || count == 0 || count > GF_BUFSIZ)
		return -EMSGSIZE;

	mutex_lock(&gdev->buf_lock);
	gf_spi_setup(gdev, GF_SPEED_READ);
	ret = gf_spi_read_bytes(gdev, GF_REG_FIFO, count, gdev->buffer);
	if (ret > 0) {
		if (copy_to_user(ubuf, gdev->buffer + 5, count))
			ret = -EFAULT;
		else
			gdev->buf_status = 0;
	}
	mutex_unlock(&gdev->buf_lock);

	return ret;
}

/*
 * gf_write(): speed 2 MHz, copy count bytes to buffer+3 and issue one
 * write burst to GF_REG_FIFO (0x8141).
 */
static ssize_t gf_write(struct file *filp, const char __user *ubuf,
			size_t count, loff_t *ppos)
{
	struct gf_dev *gdev = filp->private_data;
	int ret;

	if (!gdev || count == 0 || count > GF_BUFSIZ)
		return -EMSGSIZE;

	mutex_lock(&gdev->buf_lock);
	if (copy_from_user(gdev->buffer + 3, ubuf, count)) {
		mutex_unlock(&gdev->buf_lock);
		return -EFAULT;
	}
	gf_spi_setup(gdev, GF_SPEED_WRITE);
	ret = gf_spi_write_bytes(gdev, GF_REG_FIFO, count, gdev->buffer);
	mutex_unlock(&gdev->buf_lock);

	return ret;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gdev = filp->private_data;
	struct gf_ioc_transfer tr;
	u32 speed;
	u8 mode;
	u8 __user *ubuf;
	int ret = 0;

	if (!gdev)
		return -ENODEV;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case GF_IOC_RESET:
		irq_set_irq_wake(gdev->irq, 0);
		gf_hw_reset(gdev);
		irq_set_irq_wake(gdev->irq, 1);
		pr_debug("goodix_fp: hw reset done\n");
		return 0;

	case GF_IOC_TRANSFER:
		if (copy_from_user(&tr, (void __user *)arg, sizeof(tr)))
			return -EFAULT;
		if (tr.len == 0 || tr.len > GF_BUFSIZ)
			return -EMSGSIZE;

		mutex_lock(&gdev->buf_lock);
		gf_spi_setup(gdev, GF_SPEED_TRANSFER);

		if (tr.cmd == GF_CMD_READ) {
			ret = gf_spi_read_bytes(gdev, tr.addr, tr.len,
						gdev->buffer);
			if (ret > 0) {
				if (copy_to_user(tr.buf, gdev->buffer + 5,
						 tr.len))
					ret = -EFAULT;
			}
		} else if (tr.cmd == GF_CMD_WRITE) {
			ubuf = tr.buf;
			if (get_user(mode, ubuf)) {
				mutex_unlock(&gdev->buf_lock);
				return -EFAULT;
			}
			/* 0x8043 selects the chip operating mode. */
			if (tr.addr == GF_REG_MODE)
				gdev->mode = mode;
			if (copy_from_user(gdev->buffer + 3, ubuf, tr.len)) {
				mutex_unlock(&gdev->buf_lock);
				return -EFAULT;
			}
			ret = gf_spi_write_bytes(gdev, tr.addr, tr.len,
						 gdev->buffer);
		} else {
			pr_err("goodix_fp: unknown transfer cmd 0x%02x\n",
			       tr.cmd);
			ret = -EINVAL;
		}
		mutex_unlock(&gdev->buf_lock);
		return ret;

	case GF_IOC_SPEED:
		if (get_user(speed, (u32 __user *)arg))
			return -EFAULT;
		if (speed > GF_SPI_MAX_SPEED)
			return -EMSGSIZE;
		gf_spi_setup(gdev, speed);
		pr_debug("goodix_fp: spi speed set to %u\n", speed);
		return 0;

	default:
		pr_warn("goodix_fp: unsupported ioctl 0x%08x\n", cmd);
		return 0;
	}
}

static long gf_compat_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	/* The vendor driver literally forwards compat_ioctl to gf_ioctl. */
	return gf_ioctl(filp, cmd, arg);
}

static unsigned int gf_poll(struct file *filp, poll_table *wait)
{
	struct gf_dev *gdev = filp->private_data;
	unsigned int mask = 0;
	u8 status = 0;

	if (!gdev)
		return 0;

	poll_wait(filp, &gf_waitq, wait);
	gf_spi_read_byte(gdev, GF_REG_IRQ_STATUS, &status);
	if (gdev->buf_status & 0x80)
		mask = POLLIN | POLLRDNORM;

	return mask;
}

static int gf_fasync(int fd, struct file *filp, int mode)
{
	struct gf_dev *gdev = filp->private_data;

	if (!gdev)
		return -ENODEV;
	return fasync_helper(fd, filp, mode, &gdev->async);
}

static const struct file_operations gf_fops = {
	.owner		= THIS_MODULE,
	.open		= gf_open,
	.release	= gf_release,
	.read		= gf_read,
	.write		= gf_write,
	.unlocked_ioctl	= gf_ioctl,
	.compat_ioctl	= gf_compat_ioctl,
	.poll		= gf_poll,
	.fasync		= gf_fasync,
	.llseek		= no_llseek,
};

/* ------------------------------------------------------------------ */
/* Interrupt + framebuffer notifier                                    */
/* ------------------------------------------------------------------ */

/*
 * gf_irq() in the vendor driver reads 0x8140 (status), optionally 0x8043
 * (mode), reports navigation keys through an input device, clears 0x8140,
 * and signals userspace.  We keep the notification path and the key codes
 * (KEY_HOME 0x66 / KEY_MENU 0x8b / KEY_BACK 0x9e seen in the disassembly)
 * but leave the vendor's fingerprint algorithm to gx_fpd, which fetches
 * the raw frame through read().
 */
static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_dev *gdev = handle;
	u8 status = 0, mode = 0;

	wake_lock(&gdev->fp_wake_lock);
	pm_wakeup_event(&gdev->spi->dev, 200);

	if (gf_spi_read_byte(gdev, GF_REG_IRQ_STATUS, &status) == 0 &&
	    (status & 0x80))
		gf_spi_read_byte(gdev, GF_REG_MODE, &mode);

	gdev->mode = mode;
	/* Vendor clears the interrupt status register after handling. */
	gf_spi_write_byte(gdev, GF_REG_IRQ_STATUS, 0);

	gdev->buf_status = (s8)(0x80 | status);
	kill_fasync(&gdev->async, SIGIO, POLL_IN);
	wake_up_interruptible(&gf_waitq);

	pr_debug("goodix_fp: irq status=0x%02x mode=0x%02x\n", status, mode);
	wake_unlock(&gdev->fp_wake_lock);
	return IRQ_HANDLED;
}

/* Screen off -> enable irq as a wake source; screen on -> disable it. */
static int gf_fb_notifier(struct notifier_block *nb,
			  unsigned long action, void *data)
{
	struct gf_dev *gdev = container_of(nb, struct gf_dev, notifier);
	struct fb_event *event = data;
	int blank;

	if (!event || !event->data || action != FB_EVENT_BLANK)
		return NOTIFY_OK;

	blank = *(int *)event->data;
	if (blank == FB_BLANK_UNBLANK) {
		gdev->fb_black = false;
		irq_set_irq_wake(gdev->irq, 0);
	} else if (blank == FB_BLANK_POWERDOWN) {
		gdev->fb_black = true;
		irq_set_irq_wake(gdev->irq, 1);
	}
	return NOTIFY_OK;
}

/* ------------------------------------------------------------------ */
/* Platform data / pin control / power                                 */
/* ------------------------------------------------------------------ */

static int gf_parse_dt(struct gf_dev *gdev)
{
	struct device *dev = &gdev->spi->dev;
	struct device_node *np = dev->of_node;
	struct pinctrl *pctrl;
	enum of_gpio_flags flags;
	int ret;

	if (!np) {
		dev_err(dev, "no device tree node\n");
		return -EINVAL;
	}

	ret = of_get_named_gpio_flags(np, GF_DT_RESET_GPIO, 0, &flags);
	if (ret < 0) {
		dev_err(dev, "reset gpio not provided\n");
		return ret;
	}
	gdev->pdata.reset_gpio = ret;
	gdev->pdata.reset_gpio_flags = flags;

	ret = of_get_named_gpio_flags(np, GF_DT_IRQ_GPIO, 0, &flags);
	if (ret < 0) {
		dev_err(dev, "irq gpio not provided\n");
		return ret;
	}
	gdev->pdata.irq_gpio = ret;
	gdev->pdata.irq_gpio_flags = flags;

	pr_info("goodix_fp: pdata->irq_gpio = %d, reset_gpio = %d\n",
		gdev->pdata.irq_gpio, gdev->pdata.reset_gpio);

	pctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pctrl)) {
		dev_warn(dev, "no pinctrl handle, rely on bootloader mux\n");
		return 0;
	}
	gdev->pdata.fp_pinctrl = pctrl;

	gdev->pdata.active = pinctrl_lookup_state(pctrl, GF_PINCTRL_ACTIVE);
	gdev->pdata.suspend = pinctrl_lookup_state(pctrl, GF_PINCTRL_SUSPEND);
	gdev->pdata.release = pinctrl_lookup_state(pctrl, GF_PINCTRL_RELEASE);
	if (!IS_ERR(gdev->pdata.active)) {
		ret = pinctrl_select_state(pctrl, gdev->pdata.active);
		if (ret)
			dev_warn(dev, "Failed to select %s pinctrl state %d\n",
				 GF_PINCTRL_ACTIVE, ret);
	}

	return 0;
}

static int gf_request_gpios(struct gf_dev *gdev)
{
	struct device *dev = &gdev->spi->dev;
	int ret;

	ret = gpio_request(gdev->pdata.irq_gpio, "gf816m_irq_gpio");
	if (ret) {
		dev_err(dev, "Failed to request irq GPIO[%d], %d\n",
			gdev->pdata.irq_gpio, ret);
		return ret;
	}
	ret = gpio_direction_input(gdev->pdata.irq_gpio);
	if (ret) {
		dev_err(dev, "Failed to set direction for irq gpio[%d], %d\n",
			gdev->pdata.irq_gpio, ret);
		goto free_irq_gpio;
	}

	ret = gpio_request(gdev->pdata.reset_gpio, "gf816m_reset_gpio");
	if (ret) {
		dev_err(dev, "Failed to request reset GPIO[%d], %d\n",
			gdev->pdata.reset_gpio, ret);
		goto free_irq_gpio;
	}
	ret = gpio_direction_output(gdev->pdata.reset_gpio, 1);
	if (ret) {
		dev_err(dev, "Failed to set direction for reset GPIO[%d], %d\n",
			gdev->pdata.reset_gpio, ret);
		gpio_free(gdev->pdata.reset_gpio);
		goto free_irq_gpio;
	}

	gdev->irq = gpio_to_irq(gdev->pdata.irq_gpio);
	if (gdev->irq < 0) {
		dev_err(dev, "Unable to get irq number for GPIO %d, ret %d\n",
			gdev->pdata.irq_gpio, gdev->irq);
		ret = gdev->irq;
		goto free_all;
	}

	/* Stock dmesg: "gf_data->spi->irq=505". */
	pr_info("gf_data->spi->irq=%d\n", gdev->irq);
	return 0;

free_all:
	gpio_free(gdev->pdata.reset_gpio);
free_irq_gpio:
	gpio_free(gdev->pdata.irq_gpio);
	return ret;
}

static void gf_free_gpios(struct gf_dev *gdev)
{
	gpio_free(gdev->pdata.reset_gpio);
	gpio_free(gdev->pdata.irq_gpio);
}

static int gf_power_on(struct gf_dev *gdev)
{
	struct device *dev = &gdev->spi->dev;
	int ret;

	gdev->vdd = regulator_get(dev, "vdd");
	if (IS_ERR(gdev->vdd)) {
		ret = PTR_ERR(gdev->vdd);
		dev_err(dev, "Regulator get failed vdd rc=%d\n", ret);
		gdev->vdd = NULL;
		return ret;
	}

	if (regulator_count_voltages(gdev->vdd) > 0) {
		ret = regulator_set_voltage(gdev->vdd, 2800000, 3300000);
		if (ret) {
			dev_err(dev, "Regulator set_vtg failed vdd rc=%d\n", ret);
			regulator_put(gdev->vdd);
			gdev->vdd = NULL;
			return ret;
		}
	}

	ret = regulator_enable(gdev->vdd);
	if (ret) {
		dev_err(dev, "Regulator vdd enable failed rc=%d\n", ret);
		regulator_put(gdev->vdd);
		gdev->vdd = NULL;
		return ret;
	}

	msleep(10);
	return 0;
}

static void gf_power_off(struct gf_dev *gdev)
{
	if (!gdev->vdd)
		return;
	regulator_disable(gdev->vdd);
	regulator_put(gdev->vdd);
	gdev->vdd = NULL;
}

/* ------------------------------------------------------------------ */
/* SPI probe / remove                                                  */
/* ------------------------------------------------------------------ */

static int gf_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct device *devobj;
	struct gf_dev *gdev;
	u8 chip_id = 0;
	u8 version[16];
	int i, ret;

	gdev = kzalloc(sizeof(*gdev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	gdev->spi = spi;
	gdev->devt = MKDEV(GF_CHRDEV_MAJOR, GF_CHRDEV_MINOR);
	spin_lock_init(&gdev->spi_lock);
	mutex_init(&gdev->buf_lock);
	INIT_LIST_HEAD(&gdev->device_entry);
	spi_set_drvdata(spi, gdev);

	wake_lock_init(&gdev->fp_wake_lock, WAKE_LOCK_SUSPEND,
		       "goodix_fingerprint_lock");

	ret = gf_parse_dt(gdev);
	if (ret)
		goto err_wake_lock;

	ret = gf_power_on(gdev);
	if (ret)
		goto err_wake_lock;

	ret = gf_request_gpios(gdev);
	if (ret)
		goto err_power;

	/* Character device: /dev/goodix_fp (SELinux labels it gx_fpd_device). */
	devobj = device_create(gf_spi_class, dev, gdev->devt, gdev,
			       GF_DEVICE_NAME);
	if (IS_ERR(devobj)) {
		ret = PTR_ERR(devobj);
		dev_err(dev, "failed to create %s: %d\n", GF_DEVICE_NAME, ret);
		goto err_gpios;
	}
	ret = 0;

	/* Navigation / gesture keys. */
	gdev->input = input_allocate_device();
	if (!gdev->input) {
		ret = -ENOMEM;
		goto err_device;
	}
	gdev->input->name = GF_INPUT_NAME;
	gdev->input->id.bustype = BUS_SPI;
	input_set_capability(gdev->input, EV_KEY, KEY_HOME);
	input_set_capability(gdev->input, EV_KEY, KEY_MENU);
	input_set_capability(gdev->input, EV_KEY, KEY_BACK);
	input_set_capability(gdev->input, EV_KEY, KEY_POWER);
	ret = input_register_device(gdev->input);
	if (ret) {
		dev_err(dev, "input_register_device failed %d\n", ret);
		input_free_device(gdev->input);
		gdev->input = NULL;
		goto err_device;
	}

	/* Buffer is allocated lazily on first open, but probe reads registers. */
	gdev->buffer = kzalloc(GF_BUFFER_LEN, GFP_KERNEL);
	if (!gdev->buffer) {
		ret = -ENOMEM;
		goto err_input;
	}

	/* Initial sensor handshake, mirroring gf_probe() at line 1562. */
	gf_spi_setup(gdev, GF_SPEED_PROBE);
	gf_hw_reset(gdev);
	msleep(300);

	ret = gf_spi_read_byte(gdev, GF_REG_CHIP_ID, &chip_id);
	if (ret)
		dev_warn(dev, "chip id read failed %d\n", ret);
	if (chip_id != GF_CHIP_ID)
		dev_warn(dev, "unexpected chip id 0x%02x (want 0x%02x)\n",
			 chip_id, GF_CHIP_ID);
	else
		pr_info("goodix_fp: chip id = 0x%02x\n", chip_id);

	ret = gf_spi_read_bytes(gdev, GF_REG_VERSION, 16, gdev->buffer);
	if (ret > 0) {
		memcpy(version, gdev->buffer + 5, sizeof(version));
		pr_info("==pjh version \n");
		for (i = 0; i < 16; i++)
			pr_info("version[%d] = %x \n", i, version[i]);
	}

	gdev->notifier.notifier_call = gf_fb_notifier;
	fb_register_client(&gdev->notifier);

	/* IRQF_TRIGGER_RISING | IRQF_ONESHOT == stock's 0x2001. */
	ret = request_threaded_irq(gdev->irq, NULL, gf_irq,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				   "goodix_fp", gdev);
	if (ret) {
		dev_err(dev, "request_threaded_irq %d failed %d\n",
			gdev->irq, ret);
		goto err_fb;
	}
	irq_set_irq_wake(gdev->irq, 1);

	mutex_lock(&gf_device_list_lock);
	list_add(&gdev->device_entry, &gf_device_list);
	mutex_unlock(&gf_device_list_lock);

	gdev->device_available = true;
	dev_info(dev, "goodix_fp probed successfully\n");
	return 0;

err_fb:
	fb_unregister_client(&gdev->notifier);
err_input:
	if (gdev->input)
		input_unregister_device(gdev->input);
err_device:
	device_destroy(gf_spi_class, gdev->devt);
err_gpios:
	gf_free_gpios(gdev);
err_power:
	gf_power_off(gdev);
err_wake_lock:
	wake_lock_destroy(&gdev->fp_wake_lock);
	kfree(gdev->buffer);
	kfree(gdev);
	spi_set_drvdata(spi, NULL);
	return ret;
}

static int gf_remove(struct spi_device *spi)
{
	struct gf_dev *gdev = spi_get_drvdata(spi);

	if (!gdev)
		return 0;

	free_irq(gdev->irq, gdev);
	fb_unregister_client(&gdev->notifier);

	mutex_lock(&gf_device_list_lock);
	list_del(&gdev->device_entry);
	mutex_unlock(&gf_device_list_lock);

	device_destroy(gf_spi_class, gdev->devt);
	if (gdev->input)
		input_unregister_device(gdev->input);
	gf_free_gpios(gdev);
	gf_power_off(gdev);
	wake_lock_destroy(&gdev->fp_wake_lock);
	kfree(gdev->buffer);
	kfree(gdev);
	spi_set_drvdata(spi, NULL);
	return 0;
}

static const struct of_device_id gf_match_table[] = {
	{ .compatible = "goodix,gf", },
	{ .compatible = "goodix,gf816m", },
	{ },
};
MODULE_DEVICE_TABLE(of, gf_match_table);

static struct spi_driver gf_spi_driver = {
	.driver = {
		.name		= "goodix,gf",
		.owner		= THIS_MODULE,
		.of_match_table	= gf_match_table,
	},
	.probe	= gf_probe,
	.remove	= gf_remove,
};

static int __init gf_init(void)
{
	int ret;

	ret = register_chrdev(GF_CHRDEV_MAJOR, GF_CHRDEV_NAME, &gf_fops);
	if (ret < 0) {
		pr_err("goodix_fp: failed to register chrdev %d: %d\n",
		       GF_CHRDEV_MAJOR, ret);
		return ret;
	}

	gf_spi_class = class_create(THIS_MODULE, GF_CLASS_NAME);
	if (IS_ERR(gf_spi_class)) {
		ret = PTR_ERR(gf_spi_class);
		pr_err("goodix_fp: class_create failed %d\n", ret);
		unregister_chrdev(GF_CHRDEV_MAJOR, GF_CHRDEV_NAME);
		return ret;
	}

	ret = spi_register_driver(&gf_spi_driver);
	if (ret) {
		pr_err("goodix_fp: spi_register_driver failed %d\n", ret);
		class_destroy(gf_spi_class);
		unregister_chrdev(GF_CHRDEV_MAJOR, GF_CHRDEV_NAME);
		return ret;
	}

	pr_info("goodix_fp: GF816M driver registered (major %d)\n",
		GF_CHRDEV_MAJOR);
	return 0;
}

static void __exit gf_exit(void)
{
	spi_unregister_driver(&gf_spi_driver);
	class_destroy(gf_spi_class);
	unregister_chrdev(GF_CHRDEV_MAJOR, GF_CHRDEV_NAME);
}

module_init(gf_init);
module_exit(gf_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("G0245D CAF bring-up");
MODULE_DESCRIPTION("Goodix GF816M fingerprint sensor (Gree G0245D)");

/*
 * FocalTech FT8716 touchscreen driver for the Gree G0245D (MSM8996).
 *
 * Rebuilt from the stock 3.18.20-perf kernel.  The stock vmlinux is unstripped
 * and carries full DWARF for drivers/input/touchscreen/ft8716/, so every
 * register, packet field, ABS range and DT property below was recovered from
 * the original binary rather than guessed:
 *
 *   chip id register      0xA3  -> "Device ID = 0x87"
 *   report rate register  0x88  -> value * 10 Hz ("report rate = 60Hz")
 *   threshold register    0x80
 *   firmware version      0xA6 / 0xB2 / 0xB3 ("Firmware version = 13.0.0")
 *   vendor id register    0xA8
 *   touch packet          one 1-byte write of 0x00, then 63 bytes read
 *   IRQ                   falling edge, threaded request with NULL thread_fn
 *
 * The stock driver additionally carries firmware upgrade, APK debug channel,
 * sysfs/debugfs, gesture and fb notifier code.  None of that is needed to move
 * pointer events; this file implements the data path only.  Everything else
 * can be layered on later without touching the protocol code.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>

/*
 * Touch coordinate diagnostic aid (G0245D drift investigation).
 *
 * The FT8716 may report either display-resolved coordinates (0..xres-1 /
 * 0..yres-1, i.e. ~0..1079 / 0..1919 for this 1080x1920 panel) or raw
 * native 12-bit coordinates (up to 0..4095) depending on how the controller
 * is initialised.  A raw range reported against an input device whose ABS max
 * is 1080/1920 makes Android scale the point way off-screen -> the "drift"
 * symptom.  These counters let a scrcpy re-test surface the true range from
 * dmesg so the exact fix (range scaling vs axis/prop) can be applied.
 */
static unsigned int fts_dbg_log_count;
static unsigned int fts_dbg_max_x;
static unsigned int fts_dbg_max_y;
static bool fts_dbg_range_warn;

#define FTS_DRIVER_NAME		"fts_ts"
#define FTS_MAX_TOUCHES		10
#define FTS_PACKET_LEN		63

/* Register map recovered from fts_ts_probe / fts_update_fw_ver disassembly. */
#define FTS_REG_TOUCH_DATA	0x00
#define FTS_REG_THRESHOLD	0x80
#define FTS_REG_REPORT_RATE	0x88
#define FTS_REG_CHIP_ID		0xA3
#define FTS_REG_FW_VER_MAJOR	0xA6
#define FTS_REG_VENDOR_ID	0xA8
#define FTS_REG_FW_VER_MINOR	0xB2
#define FTS_REG_FW_VER_SUB	0xB3

/* Stock power sequence: vdd 2.6V..3.3V, vcc_i2c 1.8V. */
#define FTS_VDD_MIN_UV		2600000
#define FTS_VDD_MAX_UV		3300000
#define FTS_VCC_I2C_UV		1800000

struct fts_ts_platform_data {
	u32 irq_gpio;
	u32 reset_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio_flags;
	u32 family_id;
	u32 x_min;
	u32 y_min;
	u32 x_max;
	u32 y_max;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	const char *name;
	const char *fw_name;
};

struct fts_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct fts_ts_platform_data *pdata;
	struct work_struct touch_event_work;
	struct workqueue_struct *ts_workqueue;
	struct mutex i2c_mutex;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_suspend;
	int irq;
	unsigned int touchs;	/* bitmask of currently down finger ids */
	u8 fw_ver[3];
	u8 fw_vendor_id;
};

/* ------------------------------------------------------------------------- */
/* I2C transport                                                             */
/* ------------------------------------------------------------------------- */

/*
 * Stock fts_i2c_read(client, writebuf, writelen, readbuf, readlen):
 *   writelen > 0 -> two messages (write register address, then read)
 *   writelen = 0 -> one message (read only)
 * guarded by a mutex, returns the i2c_transfer result.
 */
static int fts_i2c_read(struct i2c_client *client, u8 *writebuf, int writelen,
			u8 *readbuf, int readlen)
{
	struct fts_ts_data *data = i2c_get_clientdata(client);
	struct i2c_msg msgs[2];
	int ret = 0;

	mutex_lock(&data->i2c_mutex);

	if (readlen <= 0)
		goto out;

	if (writelen > 0) {
		msgs[0].addr = client->addr;
		msgs[0].flags = 0;
		msgs[0].len = writelen;
		msgs[0].buf = writebuf;
		msgs[1].addr = client->addr;
		msgs[1].flags = I2C_M_RD;
		msgs[1].len = readlen;
		msgs[1].buf = readbuf;
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n", __func__);
	} else {
		msgs[0].addr = client->addr;
		msgs[0].flags = I2C_M_RD;
		msgs[0].len = readlen;
		msgs[0].buf = readbuf;
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}

out:
	mutex_unlock(&data->i2c_mutex);
	return ret < 0 ? ret : 0;
}

static int __maybe_unused fts_i2c_write(struct i2c_client *client,
					u8 *buf, int len)
{
	struct fts_ts_data *data = i2c_get_clientdata(client);
	struct i2c_msg msg;
	int ret;

	mutex_lock(&data->i2c_mutex);
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = buf;
	ret = i2c_transfer(client->adapter, &msg, 1);
	mutex_unlock(&data->i2c_mutex);

	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);
		return ret;
	}
	return 0;
}

static int fts_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	u8 addr = reg;

	return fts_i2c_read(client, &addr, 1, val, 1);
}

static int __maybe_unused fts_write_reg(struct i2c_client *client,
					u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };

	return fts_i2c_write(client, buf, sizeof(buf));
}

/* ------------------------------------------------------------------------- */
/* Touch packet decoding                                                     */
/* ------------------------------------------------------------------------- */

/*
 * One packet is 63 bytes starting at register 0x00:
 *
 *   buf[2] & 0x0f                      number of points
 *   point i at buf[3 + i*6]:
 *     p[0] >> 6                       event (0/2 down, 1/3 up)
 *     ((p[0] & 0x0f) << 8) | p[1]    x
 *     ((p[2] & 0x0f) << 8) | p[3]    y
 *     (p[2] >> 4) & 0x0f             finger id
 *     p[4]                           pressure (0 -> 1)
 *     p[5] >> 4                      area     (0 -> 9)
 *
 * Stock walks all ten slots, stops at the first id > 14, then reports the
 * filled entries through the type-B slot protocol.  A bitmask of live finger
 * ids is kept so slots that vanished from this packet get an explicit lift.
 */
static void fts_report_value(struct fts_ts_data *data, const u8 *buf)
{
	struct input_dev *input = data->input_dev;
	u16 x[FTS_MAX_TOUCHES], y[FTS_MAX_TOUCHES], pressure[FTS_MAX_TOUCHES];
	u8 event[FTS_MAX_TOUCHES], finger_id[FTS_MAX_TOUCHES], area[FTS_MAX_TOUCHES];
	int point_num = buf[2] & 0x0f;
	int n = 0, released = 0, i;
	unsigned int mask = 0;

	for (i = 0; i < FTS_MAX_TOUCHES; i++) {
		const u8 *p = &buf[3 + i * 6];
		u8 id = (p[2] >> 4) & 0x0f;
		u8 evt = p[0] >> 6;
		u8 press, sz;

		if (id > 14)
			break;

		/* Stock drops a frame that carries a down event but no points. */
		if (!(evt & 1) && point_num == 0)
			return;

		finger_id[n] = id;
		event[n] = evt;
		x[n] = ((p[0] & 0x0f) << 8) | p[1];
		y[n] = ((p[2] & 0x0f) << 8) | p[3];

		/* --- drift diagnostic (G0245D) --- */
		if (fts_dbg_log_count < 20) {
			dev_info(&data->client->dev,
				 "FTS raw touch[%u] id=%u x=%u y=%u\n",
				 fts_dbg_log_count, id, x[n], y[n]);
			fts_dbg_log_count++;
		}
		if (x[n] > fts_dbg_max_x) {
			fts_dbg_max_x = x[n];
			dev_info(&data->client->dev,
				 "FTS observed max x=%u (display max 1080)\n",
				 x[n]);
		}
		if (y[n] > fts_dbg_max_y) {
			fts_dbg_max_y = y[n];
			dev_info(&data->client->dev,
				 "FTS observed max y=%u (display max 1920)\n",
				 y[n]);
		}
		if (!fts_dbg_range_warn &&
		    (fts_dbg_max_x > 1080 || fts_dbg_max_y > 1920)) {
			fts_dbg_range_warn = true;
			dev_warn(&data->client->dev,
				 "FTS coordinate range EXCEEDS display resolution "
				 "(max x=%u y=%u) -- controller reports native "
				 "12-bit range, needs scaling to display-coords\n",
				 fts_dbg_max_x, fts_dbg_max_y);
		}
		/* --- end diagnostic --- */

		press = p[4];
		sz = p[5] >> 4;
		pressure[n] = press ? press : 1;
		area[n] = sz ? sz : 9;
		n++;
	}

	for (i = 0; i < n; i++) {
		bool up;

		input_event(input, EV_ABS, ABS_MT_SLOT, finger_id[i]);

		/* Stock clears bit 1 of the event code: 0/2 = down, 1/3 = up. */
		up = (event[i] & ~0x2) != 0;
		input_mt_report_slot_state(input, 0, !up);

		if (up) {
			mask &= ~(1U << finger_id[i]);
			released++;
			continue;
		}

		mask |= 1U << finger_id[i];
		input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, area[i]);
		input_event(input, EV_ABS, ABS_MT_PRESSURE, pressure[i]);
		input_event(input, EV_ABS, ABS_MT_POSITION_X, x[i]);
		input_event(input, EV_ABS, ABS_MT_POSITION_Y, y[i]);
	}

	if (data->touchs != mask) {
		unsigned int diff = data->touchs ^ mask;

		for (i = 0; i < FTS_MAX_TOUCHES; i++) {
			if (!(diff & (1U << i)))
				continue;
			input_event(input, EV_ABS, ABS_MT_SLOT, i);
			input_mt_report_slot_state(input, 0, false);
		}
		data->touchs = mask;
	}

	if (n == released)
		input_event(input, EV_KEY, BTN_TOUCH, 0);
	else
		input_event(input, EV_KEY, BTN_TOUCH, n != 0);

	input_sync(input);
}

static void fts_touch_irq_work(struct work_struct *work)
{
	struct fts_ts_data *data =
		container_of(work, struct fts_ts_data, touch_event_work);
	u8 buf[FTS_PACKET_LEN];
	int ret;

	memset(buf, 0, sizeof(buf));

	/* buf[0] = 0x00 is the register address; the read overwrites it. */
	ret = fts_i2c_read(data->client, buf, 1, buf, FTS_PACKET_LEN);
	if (ret < 0)
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
	else
		fts_report_value(data, buf);

	enable_irq(data->irq);
}

static irqreturn_t fts_ts_interrupt(int irq, void *dev_id)
{
	struct fts_ts_data *data = dev_id;

	disable_irq_nosync(data->irq);
	queue_work(data->ts_workqueue, &data->touch_event_work);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* Device tree, power and pin control                                        */
/* ------------------------------------------------------------------------- */

static void fts_get_dt_coords(struct device *dev, const char *name,
			      struct fts_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 coords[4];

	if (of_property_read_u32_array(np, name, coords, 4)) {
		dev_err(dev, "Unable to get %s\n", name);
		return;
	}

	if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	}
}

static int fts_parse_dt(struct device *dev,
			struct fts_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags = 0;
	u32 val;

	pdata->x_min = 0;
	pdata->y_min = 0;
	pdata->x_max = 1080;
	pdata->y_max = 1920;
	pdata->num_max_touches = FTS_MAX_TOUCHES;
	pdata->hard_rst_dly = 20;
	pdata->soft_rst_dly = 150;
	pdata->name = "fts";

	if (!np)
		return -EINVAL;

	of_property_read_string(np, "focaltech,name", &pdata->name);
	of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);

	fts_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	fts_get_dt_coords(dev, "focaltech,display-coords", pdata);

	if (!of_property_read_u32(np, "focaltech,num-max-touches", &val))
		pdata->num_max_touches = min_t(u32, val, FTS_MAX_TOUCHES);
	if (!of_property_read_u32(np, "focaltech,hard-reset-delay-ms", &val))
		pdata->hard_rst_dly = val;
	if (!of_property_read_u32(np, "focaltech,soft-reset-delay-ms", &val))
		pdata->soft_rst_dly = val;
	of_property_read_u32(np, "focaltech,group-id", &pdata->group_id);
	of_property_read_u32(np, "focaltech,family-id", &pdata->family_id);

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
						  0, &flags);
	pdata->irq_gpio_flags = flags;
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
						    0, &flags);
	pdata->reset_gpio_flags = flags;

	if (!gpio_is_valid(pdata->irq_gpio) ||
	    !gpio_is_valid(pdata->reset_gpio)) {
		dev_err(dev, "Invalid irq/reset GPIO in DT\n");
		return -EINVAL;
	}

	return 0;
}

static void fts_power_init(struct fts_ts_data *data)
{
	struct device *dev = &data->client->dev;

	/* Optional: some boards power the panel rail from the bootloader. */
	data->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(data->vdd)) {
		dev_warn(dev, "Regulator get failed vdd rc=%ld\n",
			 PTR_ERR(data->vdd));
		data->vdd = NULL;
	} else if (regulator_count_voltages(data->vdd) > 0) {
		regulator_set_voltage(data->vdd, FTS_VDD_MIN_UV,
				      FTS_VDD_MAX_UV);
	}

	data->vcc_i2c = devm_regulator_get(dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		dev_warn(dev, "Regulator get failed vcc_i2c rc=%ld\n",
			 PTR_ERR(data->vcc_i2c));
		data->vcc_i2c = NULL;
	} else if (regulator_count_voltages(data->vcc_i2c) > 0) {
		regulator_set_voltage(data->vcc_i2c, FTS_VCC_I2C_UV,
				      FTS_VCC_I2C_UV);
	}
}

static int fts_power_on(struct fts_ts_data *data, bool on)
{
	struct device *dev = &data->client->dev;
	int ret;

	if (on) {
		if (data->vdd) {
			ret = regulator_enable(data->vdd);
			if (ret)
				dev_err(dev, "Regulator vdd enable failed rc=%d\n",
					ret);
		}
		if (data->vcc_i2c) {
			ret = regulator_enable(data->vcc_i2c);
			if (ret)
				dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n",
					ret);
		}
	} else {
		if (data->vdd) {
			ret = regulator_disable(data->vdd);
			if (ret)
				dev_err(dev, "Regulator vdd disable failed rc=%d\n",
					ret);
		}
		if (data->vcc_i2c) {
			ret = regulator_disable(data->vcc_i2c);
			if (ret)
				dev_err(dev, "Regulator vcc_i2c disable failed rc=%d\n",
					ret);
		}
	}

	return 0;
}

static int fts_pinctrl_init(struct fts_ts_data *data)
{
	struct device *dev = &data->client->dev;

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		dev_warn(dev, "No pinctrl available, skipping\n");
		data->pinctrl = NULL;
		return 0;
	}

	data->pins_active = pinctrl_lookup_state(data->pinctrl,
						 "pmx_ts_active");
	if (IS_ERR(data->pins_active)) {
		dev_warn(dev, "Can not lookup pmx_ts_active pinstate %ld\n",
			 PTR_ERR(data->pins_active));
		data->pins_active = NULL;
	}

	data->pins_suspend = pinctrl_lookup_state(data->pinctrl,
						  "pmx_ts_suspend");
	if (IS_ERR(data->pins_suspend))
		data->pins_suspend = NULL;

	if (data->pins_active)
		return pinctrl_select_state(data->pinctrl, data->pins_active);

	return 0;
}

static int fts_gpio_configure(struct fts_ts_data *data)
{
	struct device *dev = &data->client->dev;
	struct fts_ts_platform_data *pdata = data->pdata;
	int ret;

	ret = gpio_request(pdata->irq_gpio, "fts_irq_gpio");
	if (ret) {
		dev_err(dev, "irq gpio request failed\n");
		return ret;
	}
	ret = gpio_direction_input(pdata->irq_gpio);
	if (ret) {
		dev_err(dev, "set_direction for irq gpio failed\n");
		goto free_irq_gpio;
	}

	ret = gpio_request(pdata->reset_gpio, "fts_reset_gpio");
	if (ret) {
		dev_err(dev, "reset gpio request failed\n");
		goto free_irq_gpio;
	}

	/* Reset: hold low for hard_rst_dly ms, then release high. */
	ret = gpio_direction_output(pdata->reset_gpio, 0);
	if (ret) {
		dev_err(dev, "set_direction for reset gpio failed\n");
		goto free_reset_gpio;
	}
	msleep(pdata->hard_rst_dly);
	gpio_set_value(pdata->reset_gpio, 1);

	return 0;

free_reset_gpio:
	gpio_free(pdata->reset_gpio);
free_irq_gpio:
	gpio_free(pdata->irq_gpio);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* Input device and probe                                                    */
/* ------------------------------------------------------------------------- */

static int fts_input_init(struct fts_ts_data *data)
{
	struct device *dev = &data->client->dev;
	struct fts_ts_platform_data *pdata = data->pdata;
	struct input_dev *input;
	int ret;

	input = input_allocate_device();
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input;
	input->name = FTS_DRIVER_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	input_set_capability(input, EV_KEY, BTN_TOUCH);

	/*
	 * INPUT_PROP_DIRECT marks the device as a direct touchscreen whose
	 * axes map 1:1 onto the screen.  The stock FocalTech drivers
	 * (ft5x06_ts.c, ft5x46_ts.c) all set this; it was dropped in the
	 * G0245D port and can make the Android input stack mishandle the
	 * coordinate mapping (drift when the pointer is mapped through a
	 * non-direct device profile).
	 */
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input_mt_init_slots(input, pdata->num_max_touches, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X,
			     pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
			     pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	input_set_drvdata(input, data);

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "Input device registration failed\n");
		input_free_device(input);
		data->input_dev = NULL;
		return ret;
	}

	return 0;
}

static void fts_read_diagnostics(struct fts_ts_data *data)
{
	struct device *dev = &data->client->dev;
	u8 val = 0;

	if (!fts_read_reg(data->client, FTS_REG_CHIP_ID, &val))
		dev_info(dev, "Device ID = 0x%02x\n", val);

	if (!fts_read_reg(data->client, FTS_REG_REPORT_RATE, &val))
		dev_info(dev, "report rate = %dHz\n", val * 10);
	else
		dev_err(dev, "report rate read failed\n");

	if (fts_read_reg(data->client, FTS_REG_THRESHOLD, &val))
		dev_err(dev, "threshold read failed\n");

	if (!fts_read_reg(data->client, FTS_REG_FW_VER_MAJOR,
			  &data->fw_ver[0]) &&
	    !fts_read_reg(data->client, FTS_REG_FW_VER_MINOR,
			  &data->fw_ver[1]) &&
	    !fts_read_reg(data->client, FTS_REG_FW_VER_SUB, &data->fw_ver[2]))
		dev_info(dev, "Firmware version = %d.%d.%d\n",
			 data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);

	fts_read_reg(data->client, FTS_REG_VENDOR_ID, &data->fw_vendor_id);
}

static int fts_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fts_ts_data *data;
	struct fts_ts_platform_data *pdata;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C not supported\n");
		return -EIO;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	data->client = client;
	data->pdata = pdata;
	mutex_init(&data->i2c_mutex);
	INIT_WORK(&data->touch_event_work, fts_touch_irq_work);
	i2c_set_clientdata(client, data);

	ret = fts_parse_dt(dev, pdata);
	if (ret) {
		dev_err(dev, "Invalid pdata\n");
		return ret;
	}

	ret = fts_input_init(data);
	if (ret)
		return ret;

	fts_power_init(data);
	ret = fts_power_on(data, true);
	if (ret) {
		dev_err(dev, "power on failed\n");
		goto err_unregister_input;
	}

	ret = fts_pinctrl_init(data);
	if (ret) {
		dev_err(dev, "failed to select pin to active state\n");
		goto err_power_off;
	}

	ret = fts_gpio_configure(data);
	if (ret) {
		dev_err(dev, "Failed to configure the gpios\n");
		goto err_power_off;
	}

	/*
	 * Stock waits soft_rst_dly (150 ms from DT) plus a hard-coded 200 ms
	 * after releasing reset before the first I2C access.  Reading earlier
	 * returns 0x00 / 0xEF garbage: the controller has not left its boot
	 * state yet.  This is also the reason the stock dmesg shows ~400 ms
	 * between "input: fts_ts" and "Device ID".
	 */
	msleep(pdata->soft_rst_dly);
	msleep(200);

	data->ts_workqueue = create_singlethread_workqueue("fts_wq");
	if (!data->ts_workqueue) {
		dev_err(dev, "failed to create fts_wq\n");
		ret = -ENOMEM;
		goto err_free_gpio;
	}

	fts_read_diagnostics(data);

	data->irq = client->irq;
	if (data->irq <= 0)
		data->irq = gpio_to_irq(pdata->irq_gpio);
	if (data->irq <= 0) {
		dev_err(dev, "failed to obtain touch IRQ\n");
		ret = -EINVAL;
		goto err_destroy_wq;
	}

	ret = request_threaded_irq(data->irq, fts_ts_interrupt, NULL,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "fts_ts_interrupt", data);
	if (ret) {
		dev_err(dev, "request irq failed\n");
		goto err_destroy_wq;
	}

	/* Stock disables the IRQ until probe finishes, then re-enables it. */
	disable_irq(data->irq);
	enable_irq(data->irq);

	dev_info(dev, "fts_ts probed successfully!\n");

	return 0;

err_destroy_wq:
	destroy_workqueue(data->ts_workqueue);
err_free_gpio:
	gpio_free(pdata->reset_gpio);
	gpio_free(pdata->irq_gpio);
err_power_off:
	fts_power_on(data, false);
err_unregister_input:
	input_unregister_device(data->input_dev);
	return ret;
}

static int fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *data = i2c_get_clientdata(client);
	struct fts_ts_platform_data *pdata = data->pdata;

	free_irq(data->irq, data);
	destroy_workqueue(data->ts_workqueue);
	input_unregister_device(data->input_dev);
	gpio_free(pdata->reset_gpio);
	gpio_free(pdata->irq_gpio);
	fts_power_on(data, false);

	return 0;
}

static const struct of_device_id fts_match_table[] = {
	{ .compatible = "focaltech,fts", },
	{ .compatible = "focaltech,ft8716", },
	{ },
};
MODULE_DEVICE_TABLE(of, fts_match_table);

static const struct i2c_device_id fts_ts_id[] = {
	{ FTS_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, fts_ts_id);

static struct i2c_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		.name = FTS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = fts_match_table,
	},
	.id_table = fts_ts_id,
};

static int __init fts_ts_init(void)
{
	return i2c_add_driver(&fts_ts_driver);
}

static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_DESCRIPTION("FocalTech FT8716 touchscreen (Gree G0245D)");
MODULE_AUTHOR("G0245D kernel reconstruction");
MODULE_LICENSE("GPL v2");

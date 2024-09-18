// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>


/*
 * FIXME: remove this when a subdev API becomes available
 * to set the MIPI CSI-2 virtual channel.
 */
static unsigned int virtual_channel;
module_param(virtual_channel, uint, 0444);
MODULE_PARM_DESC(virtual_channel,
		 "MIPI CSI-2 virtual channel (0..3), default 0");


/* regulator supplies */
static const char * const imx490_supply_name[] = {
	"DOVDD", /* Digital I/O (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
	"DVDD",  /* Digital Core (1.5V) supply */
};

#define IMX490_NUM_SUPPLIES ARRAY_SIZE(imx490_supply_name)

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct imx490_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to IMX490 */
	u32 xclk_freq;

	struct regulator_bulk_data supplies[IMX490_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	bool   upside_down;

	/* lock to protect all members below */
	struct mutex lock;
	bool streaming;
};

static inline struct imx490_dev *to_imx490_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx490_dev, sd);
}

static struct v4l2_mbus_framefmt imx490_default_fmt = {
	.code = MEDIA_BUS_FMT_UYVY8_1X16,
	.width = 2880,
	.height = 1860,
	.colorspace = V4L2_COLORSPACE_REC709,
	.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_REC709),
	.quantization = V4L2_QUANTIZATION_FULL_RANGE,
	.xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(V4L2_COLORSPACE_REC709),
	.field = V4L2_FIELD_NONE,
};

static int imx490_write_reg(struct imx490_dev *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
			__func__, reg, val);
		return ret;
	}

	return 0;
}

static int imx490_read_reg(struct imx490_dev *sensor, u16 reg, u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x\n",
			__func__, reg);
		return ret;
	}

	*val = buf[0];
	return 0;
}

/* restore the last set video mode after chip power-on */
static int imx490_init_base_settings(struct imx490_dev *sensor)
{
	return 0;
}

static void imx490_power(struct imx490_dev *sensor, bool enable)
{
	gpiod_set_value_cansleep(sensor->pwdn_gpio, enable ? 0 : 1);
}

/*
 * From section 2.7 power up sequence:
 * t0 + t1 + t2 >= 5ms	Delay from DOVDD stable to PWDN pull down
 * t3 >= 1ms		Delay from PWDN pull down to RESETB pull up
 * t4 >= 20ms		Delay from RESETB pull up to SCCB (i2c) stable
 *
 * Some modules don't expose RESETB/PWDN pins directly, instead providing a
 * "PWUP" GPIO which is wired through appropriate delays and inverters to the
 * pins.
 *
 * In such cases, this gpio should be mapped to pwdn_gpio in the driver, and we
 * should still toggle the pwdn_gpio below with the appropriate delays, while
 * the calls to reset_gpio will be ignored.
 */
static void imx490_powerup_sequence(struct imx490_dev *sensor)
{
	if (sensor->pwdn_gpio) {
		gpiod_set_value_cansleep(sensor->reset_gpio, 0);

		/* camera power cycle */
		imx490_power(sensor, false);
		usleep_range(5000, 10000);
		imx490_power(sensor, true);
		usleep_range(5000, 10000);

		gpiod_set_value_cansleep(sensor->reset_gpio, 1);
		usleep_range(1000, 2000);

		gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	}
	usleep_range(20000, 25000);

}

static int imx490_set_power_on(struct imx490_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		return ret;
	}

	ret = regulator_bulk_enable(IMX490_NUM_SUPPLIES,
				    sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		goto xclk_off;
	}

	imx490_powerup_sequence(sensor);

	return 0;

xclk_off:
	clk_disable_unprepare(sensor->xclk);
	return ret;
}

static void imx490_set_power_off(struct imx490_dev *sensor)
{
	imx490_power(sensor, false);
	regulator_bulk_disable(IMX490_NUM_SUPPLIES, sensor->supplies);
	clk_disable_unprepare(sensor->xclk);
}


static int imx490_set_power(struct imx490_dev *sensor, bool on)
{
	int ret = 0;

	if (on) {
		ret = imx490_set_power_on(sensor);
		if (ret)
			return ret;

		ret = imx490_init_base_settings(sensor);
		if (ret)
			goto power_off;
	}

	if (!on)
		imx490_set_power_off(sensor);

	return 0;

power_off:
	imx490_set_power_off(sensor);
	return ret;
}

static int imx490_sensor_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx490_dev *imx490 = to_imx490_dev(sd);

	return imx490_set_power(imx490, false);
}

static int imx490_sensor_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx490_dev *imx490 = to_imx490_dev(sd);

	return imx490_set_power(imx490, true);
}

/* --------------- Subdev Operations --------------- */
static int imx490_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct imx490_dev *sensor = to_imx490_dev(sd);
	dev_info(&sensor->i2c_client->dev, "zuojisi: %s \n",__func__);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);
	format->format = imx490_default_fmt;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int imx490_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct imx490_dev *sensor = to_imx490_dev(sd);
	int ret = 0;
	dev_info(&sensor->i2c_client->dev, "zuojisi: %s \n",__func__);
	dev_info(&sensor->i2c_client->dev, "zuojisi: which=%s", format->which ? "V4L2_SUBDEV_FORMAT_ACTIVE":"V4L2_SUBDEV_FORMAT_TRY");
	dev_info(&sensor->i2c_client->dev, "zuojisi: code=%d, colorspace=%d",format->format.code, format->format.colorspace);
	dev_info(&sensor->i2c_client->dev, "zuojisi: xfer_func=%d, ycbcr_enc=%d",format->format.xfer_func, format->format.ycbcr_enc);
	dev_info(&sensor->i2c_client->dev, "zuojisi: quantization=%d, field=%d",format->format.quantization, format->format.field);
	dev_info(&sensor->i2c_client->dev, "zuojisi: width=%d, height=%d",format->format.width, format->format.height);
	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}
	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, sd_state, 0) = format->format;
		goto out;
	}

	imx490_default_fmt = format->format;

	mutex_unlock(&sensor->lock);
	ret = 0;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int imx490_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct imx490_dev *sensor = to_imx490_dev(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		mutex_lock(&sensor->lock);
		sel->r.left = sel->r.top = 0;
		sel->r.width = 2880;
		sel->r.height = 1860;
		mutex_unlock(&sensor->lock);
		return 0;
	}

	return -EINVAL;
}

static int imx490_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int index = fse->index;
	struct imx490_dev *sensor = to_imx490_dev(sd);
	dev_info(&sensor->i2c_client->dev, "zuojisi: %s \n",__func__);

	if (fse->pad != 0)
		return -EINVAL;

	if (index != 0)
		return -EINVAL;

	fse->min_width = 2880;
	fse->max_width = fse->min_width;
	fse->min_height = 1860;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx490_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct v4l2_fract tpf;
	struct imx490_dev *sensor = to_imx490_dev(sd);
	dev_info(&sensor->i2c_client->dev, "zuojisi: %s \n",__func__);

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index != 0)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = 30;

	fie->interval = tpf;
	return 0;
}

static int imx490_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx490_dev *sensor = to_imx490_dev(sd);
	dev_info(&sensor->i2c_client->dev, "zuojisi: %s \n",__func__);

	mutex_lock(&sensor->lock);
	fi->interval.numerator = 1;
	fi->interval.denominator = 30;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int imx490_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx490_dev *sensor = to_imx490_dev(sd);
	int ret = 0;
	dev_info(&sensor->i2c_client->dev, "zuojisi: %s \n",__func__);

	if (fi->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mutex_unlock(&sensor->lock);
	return 0; //fix 30fps
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int imx490_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;

	return 0;
}

static int imx490_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx490_dev *sensor = to_imx490_dev(sd);
	int ret = 0;
	u16 addrbackup;

	dev_info(&sensor->i2c_client->dev, "zuojisi: s stream enable=%d\n",enable);

	if (enable) {
		ret = pm_runtime_resume_and_get(&sensor->i2c_client->dev);
		if (ret < 0)
			return ret;
	}

	mutex_lock(&sensor->lock);
	if (sensor->streaming == enable) {
		mutex_unlock(&sensor->lock);
		return 0;
	}

	if (enable) {
		addrbackup = sensor->i2c_client->addr;

		sensor->i2c_client->addr = 0x40;
		imx490_write_reg(sensor, 0x02d6, 0x10);
		imx490_write_reg(sensor, 0x02d4, 0xa0);
		usleep_range(1000 * 50, 1000 * 50 + 100);
		imx490_write_reg(sensor, 0x02d6, 0x00);
		imx490_write_reg(sensor, 0x02d4, 0xa0);
		usleep_range(1000 * 50, 1000 * 50 + 100);

		sensor->i2c_client->addr = 0x48;
		imx490_write_reg(sensor, 0x0010, 0x21);
		usleep_range(1000 * 100, 1000 * 100 + 100);

		sensor->i2c_client->addr = 0x40;
        imx490_write_reg(sensor, 0x0010, 0x21);
		usleep_range(1000 * 100, 1000 * 100 + 100);

		sensor->i2c_client->addr = addrbackup;
	} else {
		//
	}

	sensor->streaming = enable;
	mutex_unlock(&sensor->lock);

	if (!enable || ret)
		pm_runtime_put_autosuspend(&sensor->i2c_client->dev);

	return ret;
}

static int imx490_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt =
				v4l2_subdev_get_try_format(sd, state, 0);
	struct v4l2_rect *crop = v4l2_subdev_get_try_crop(sd, state, 0);

	*fmt = imx490_default_fmt;

	crop->left = 0;
	crop->top = 0;
	crop->width = 2880;
	crop->height = 1860;

	return 0;
}

static const struct v4l2_subdev_core_ops imx490_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx490_video_ops = {
	.g_frame_interval = imx490_g_frame_interval,
	.s_frame_interval = imx490_s_frame_interval,
	.s_stream = imx490_s_stream,
};

static const struct v4l2_subdev_pad_ops imx490_pad_ops = {
	.init_cfg = imx490_init_cfg,
	.enum_mbus_code = imx490_enum_mbus_code,
	.get_fmt = imx490_get_fmt,
	.set_fmt = imx490_set_fmt,
	.get_selection = imx490_get_selection,
	.enum_frame_size = imx490_enum_frame_size,
	.enum_frame_interval = imx490_enum_frame_interval,
};

static const struct v4l2_subdev_ops imx490_subdev_ops = {
	.core = &imx490_core_ops,
	.video = &imx490_video_ops,
	.pad = &imx490_pad_ops,
};

static int imx490_get_regulators(struct imx490_dev *sensor)
{
	int i;

	for (i = 0; i < IMX490_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = imx490_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       IMX490_NUM_SUPPLIES,
				       sensor->supplies);
}

static int imx490_setup_deserializer(struct imx490_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device *dev = &client->dev;
	u16 addrbackup = client->addr;
	u8 val = 0;
	int ret;
	client->addr = 0x48;

	ret = imx490_read_reg(sensor, 0x000d, &val);
	if(ret == 0 && val == 0x94){
		dev_info(dev, "zuojisi: deserializer detected\n");
	}else{
		dev_err(dev, "zuojisi: deserializer NOT detected\n");
		goto error;
	}

	imx490_write_reg(sensor, 0x0003, 0x40);
	imx490_write_reg(sensor, 0x02C2, 0x00);
	imx490_write_reg(sensor, 0x02C3, 0x00);
	imx490_write_reg(sensor, 0x0076, 0x70);
	imx490_write_reg(sensor, 0x007e, 0x70);
	imx490_write_reg(sensor, 0x0018, 0x0c);

	client->addr = 0x40;

	ret = imx490_read_reg(sensor, 0x000d, &val);
	if(ret == 0 && val == 0x91){
		dev_info(dev, "zuojisi: serializer detected\n");
	}else{
		dev_err(dev, "zuojisi: serializer NOT detected\n");
		goto error;
	}

	imx490_write_reg(sensor, 0x02be, 0x10);
	imx490_write_reg(sensor, 0x02bf, 0x60);
	imx490_write_reg(sensor, 0x0318, 0x5e);
	imx490_write_reg(sensor, 0x0311, 0xf0);
	imx490_write_reg(sensor, 0x0308, 0x64);
	imx490_write_reg(sensor, 0x0002, 0xf3);

	client->addr = 0x48;
	imx490_write_reg(sensor, 0x040b, 0x07);
	imx490_write_reg(sensor, 0x042d, 0x15);
	imx490_write_reg(sensor, 0x040d, 0x1e);
	imx490_write_reg(sensor, 0x040e, 0x1e);
	imx490_write_reg(sensor, 0x040f, 0x00);
	imx490_write_reg(sensor, 0x0410, 0x00);
	imx490_write_reg(sensor, 0x0411, 0x01);
	imx490_write_reg(sensor, 0x0412, 0x01);
	imx490_write_reg(sensor, 0x044a, 0x40);
	imx490_write_reg(sensor, 0x0320, 0x2f); //1500MHz
	// imx490_write_reg(sensor, 0x0320, 0x2c); //1200MHz
	imx490_write_reg(sensor, 0x0050, 0x02);
	
	usleep_range(1000 * 300, 1000 * 300 + 100);
	
	client->addr = addrbackup;
	return 0;
error:
	client->addr = addrbackup;
	return -EINVAL;
}

static int imx490_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct imx490_dev *sensor;
	int ret;

	dev_info(dev, "zuojisi: probing imx490...\n");

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	if (sensor->ep.bus_type != V4L2_MBUS_PARALLEL &&
	    sensor->ep.bus_type != V4L2_MBUS_CSI2_DPHY &&
	    sensor->ep.bus_type != V4L2_MBUS_BT656) {
		dev_err(dev, "Unsupported bus type %d\n", sensor->ep.bus_type);
		return -EINVAL;
	}

	/* get system clock (xclk) */
	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}

	/* request optional power down pin */
	sensor->pwdn_gpio = devm_gpiod_get_optional(dev, "powerdown",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->pwdn_gpio))
		return PTR_ERR(sensor->pwdn_gpio);

	/* request optional reset pin */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return PTR_ERR(sensor->reset_gpio);

	v4l2_i2c_subdev_init(&sensor->sd, client, &imx490_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	ret = imx490_get_regulators(sensor);
	if (ret)
		goto entity_cleanup;

	mutex_init(&sensor->lock);

	ret = imx490_setup_deserializer(sensor);
	if (ret) {
		dev_err(dev, "failed to setup deserializer\n");
		goto entity_cleanup;
	}

	ret = imx490_sensor_resume(dev);
	if (ret) {
		dev_err(dev, "failed to power on\n");
		goto entity_cleanup;
	}

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto err_pm_runtime;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	dev_info(dev, "zuojisi: probe imx490 success!\n");

	return 0;

err_pm_runtime:
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	imx490_sensor_suspend(dev);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static void imx490_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx490_dev *sensor = to_imx490_dev(sd);
	struct device *dev = &client->dev;

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		imx490_sensor_suspend(dev);
	pm_runtime_set_suspended(dev);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
}

static const struct dev_pm_ops imx490_pm_ops = {
	SET_RUNTIME_PM_OPS(imx490_sensor_suspend, imx490_sensor_resume, NULL)
};

static const struct i2c_device_id imx490_id[] = {
	{"imx490", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, imx490_id);

static const struct of_device_id imx490_dt_ids[] = {
	{ .compatible = "sony,z_imx490" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx490_dt_ids);

static struct i2c_driver imx490_i2c_driver = {
	.driver = {
		.name  = "z_imx490",
		.of_match_table	= imx490_dt_ids,
		.pm = &imx490_pm_ops,
	},
	.id_table = imx490_id,
	.probe_new = imx490_probe,
	.remove   = imx490_remove,
};

module_i2c_driver(imx490_i2c_driver);

MODULE_DESCRIPTION("IMX490 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");

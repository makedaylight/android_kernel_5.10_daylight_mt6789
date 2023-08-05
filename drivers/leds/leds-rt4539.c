// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 InnoComm.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME " %s#%d: " fmt "\n", __func__, __LINE__
#define dev_fmt(fmt) " %s#%d: " fmt "\n", __func__, __LINE__

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include <leds-mtk.h>

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
#include <linux/hrtimer.h>
#include "mtk_disp_notify.h"
#endif

#define MAX_FB	6

#define RT4539_REG_00	0x00
#define RT4539_REG_00_DEFAULT	0x6C
#define RT4539_REG_00_MASK	0xEF
#define RT4539_SPREAD_SPECTRUM	BIT(7)
#define RT4539_LX_RATE_CTRL_MASK	GENMASK(6, 5)
#define RT4539_MODE_MASK	GENMASK(2, 0)
#define RT4539_DCMODE	0x04

#define RT4539_REG_01	0x01
#define RT4539_REG_01_DEFAULT	0x17
#define RT4539_REG_01_MASK	GENMASK(4, 0)
#define RT4539_SWITCHING_CURRENT_LIMIT	BIT(4)
#define RT4539_SWITCHING_FREQ_MASK	GENMASK(3, 0)

#define RT4539_REG_CURRENT	0x02
#define RT4539_REG_CURRENT_MASK	0xFF
#define RT4539_REG_CURRENT_DEFAULT	0x82
#define RT4539_REG_CURRENT_MAX	0xFF

#define RT4539_REG_MSB_BRIGHTNESS	0x04
#define RT4539_BRIGHTNESS_MSB_MASK	GENMASK(11, 8)
#define RT4539_BRIGHTNESS_LSB_MASK	GENMASK(7, 0)
#define RT4539_BRIGHTNESS_MSB_SHIFT	8

#define RT4539_REG_06	0x06
#define RT4539_REG_06_MASK	0xFF
#define RT4539_REG_06_DEFAULT	0xA8
#define RT4539_REG_06_FADE_TIME_MASK	GENMASK(2, 0)
#define RT4539_REG_06_FADE_TIME_SHIFT	0
#define RT4539_REG_06_SLOPE_TIME_MASK	GENMASK(5, 3)
#define RT4539_REG_06_SLOPE_TIME_SHIFT	3

#define RT4539_REG_07	0x07
#define RT4539_ADVANCED_CONTROL_MASK	GENMASK(1, 0)
#define RT4539_CONTROL_PWM	0x00
#define RT4539_CONTROL_I2C	0x01
#define RT4539_CONTROL_PWM_AND_I2C	0x10
#define RT4539_PWM_SAMPLE_RATE_MASK	GENMASK(7, 5)
#define RT4539_PWM_SAMPLE_RATE_SHIFT	5
#define RT4539_PWM_SAMPLE_RATE_AUTO	0x00
#define RT4539_PWM_SAMPLE_RATE_MAX	0x07

#define RT4539_REG_08	0x08
#define RT4539_REG_08_MASK	0xE3
#define RT4539_REG_08_DEFAULT	0x00

#define RT4539_REG_09	0x09
#define RT4539_REG_09_DEFAULT	0x78
#define RT4539_REG_09_MASK	GENMASK(6, 0)
#define RT4539_PFM_EN	BIT(0)
#define RT4539_PFM_LOWEST_FREQ_MASK	GENMASK(6, 1)

#define RT4539_REG_EN	0x0B
#define RT4539_REG_0B_MASK	0xFE
#define RT4539_EN_MASK	BIT(7)
#define RT4539_FB6_MASK	BIT(6)
#define RT4539_FB5_MASK	BIT(5)
#define RT4539_FB4_MASK	BIT(4)
#define RT4539_FB3_MASK	BIT(3)
#define RT4539_FB2_MASK	BIT(2)
#define RT4539_FB1_MASK	BIT(1)

#define RT4539_REG_MTP	0xFF

#define EN_POWEN_ON_DELAY	20

struct rt4539_led {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;

	int curr_hw_brightness;
	int curr_i2c_brightness;

	struct mt_led_data m_led;
	struct pwm_device	*pwm;
	struct pwm_state	pwmstate;

	struct gpio_desc	*enable_gpio;
	bool enable;

	int control;
	int pwm_sample_rate;
	int max_current;
	int reg0x00; /* reg 0x00 */
	int reg0x01; /* reg 0x01 */
	int time_ctrl; /* reg 0x06 */
	int time_ctrl_for_off; /* reg 0x06 for brightness off */
	unsigned int fade_off_min_hw_brightness;
	unsigned int off_hw_brightness_threshold;
	int reg0x08; /* reg 0x08 */
	int reg0x09; /* reg 0x09 */
	int fbs[MAX_FB];

	unsigned int enable_post_delay_ms;
	unsigned int screen_on_backlight_off_pre_delay_ms;
	unsigned int screen_off_backlight_off_pre_delay_ms;
	struct delayed_work disable_delay_work;
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	bool display_off;
	unsigned int disp_off_to_on_delay_ms;
	unsigned int disp_off_to_on_delay_timeout_ms;
	struct notifier_block disp_notifier;
	bool disp_off_to_on_delay;
	struct hrtimer disp_off_to_on_delay_timeout;
#endif

};

static const unsigned int fade_time_us[] = {
	/* B000 */ 1, /* 0.5us */
	/* B001 */ 1,
	/* B010 */ 2,
	/* B011 */ 4,
	/* B100 */ 8,
	/* B101 */ 16,
	/* B110 */ 32,
	/* B111 */ 64,
};

static const unsigned int slop_time_ms[] = {
	/* B000 */ 0,
	/* B001 */ 1,
	/* B010 */ 8,
	/* B011 */ 128,
	/* B100 */ 256,
	/* B101 */ 512,
	/* B110 */ 768,
	/* B111 */ 1024,
};

static void rt4539_mdelay(unsigned int ms)
{
	if (ms > 0) {
		if (ms < 20)
			usleep_range(ms * 1000, ms * 1000 + 500);
		else
			msleep(ms);
	}
}

static int rt4539_disable(struct rt4539_led *priv, const char* log_prefix)
{
	/* no I2C access here because the enable gpio may be deactivated */
	int ret;

	ret = gpiod_direction_output(priv->enable_gpio, 0);
	if (ret)
		dev_notice(priv->dev, "[%s] Failed to set enable_gpio deactive, err %d", log_prefix, ret);
	else
		dev_alert(priv->dev, "[%s] off", log_prefix);

	return ret;
}

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
static enum hrtimer_restart rt4539_disp_off_to_on_delay_timeout(struct hrtimer *timer)
{
	struct rt4539_led *priv = container_of(timer,
			struct rt4539_led, disp_off_to_on_delay_timeout);

	// dev_info(priv->dev, "+%s()", __func__);
	priv->disp_off_to_on_delay = false;

	return HRTIMER_NORESTART;
}

static int disp_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	int *ev_data = (int *)data;
	struct rt4539_led *priv = container_of(self, struct rt4539_led, disp_notifier);

	// dev_info(priv->dev, "FB EVENT: %lu", event);

	if (!ev_data)
		return 0;

	if (event == MTK_DISP_EARLY_EVENT_BLANK) {
		if (*ev_data == MTK_DISP_BLANK_UNBLANK) {
			priv->display_off = false;
			priv->disp_off_to_on_delay = true;
	        // dev_info(priv->dev, "display is on");
		} else if (*ev_data == MTK_DISP_BLANK_POWERDOWN) {
			priv->display_off = true;
			priv->disp_off_to_on_delay = false;
	        // dev_info(priv->dev, "display is off");
			if (delayed_work_pending(&priv->disable_delay_work))
				cancel_delayed_work(&priv->disable_delay_work);
			else
				cancel_delayed_work_sync(&priv->disable_delay_work);
			rt4539_disable(priv, "display off");
			priv->enable = false; /* force disable */
		}
	} else if (event == MTK_DISP_EVENT_BLANK) {
		if (priv->disp_off_to_on_delay_ms && priv->disp_off_to_on_delay_timeout_ms) {
			if (*ev_data == MTK_DISP_BLANK_UNBLANK)
				hrtimer_start(&priv->disp_off_to_on_delay_timeout,
						ms_to_ktime(priv->disp_off_to_on_delay_timeout_ms),
						HRTIMER_MODE_REL);
			else if (*ev_data == MTK_DISP_BLANK_POWERDOWN)
				hrtimer_cancel(&priv->disp_off_to_on_delay_timeout);
		}
	}

	// dev_info(priv->dev, "-%s()", __func__);

	return 0;
}

static void register_disp_notifier(struct rt4539_led *priv)
{
	int ret;

	if (priv->disp_off_to_on_delay_ms && priv->disp_off_to_on_delay_timeout_ms) {
		hrtimer_init(&priv->disp_off_to_on_delay_timeout, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		priv->disp_off_to_on_delay_timeout.function = rt4539_disp_off_to_on_delay_timeout;
	}

	priv->disp_notifier.notifier_call = disp_notifier_callback;
	ret = mtk_disp_notifier_register(dev_name(priv->dev), &priv->disp_notifier);
	if (ret)
		dev_err(priv->dev, "reigster disp_notifier err %d", ret);
}

static void unregister_disp_notifier(struct rt4539_led *priv)
{
	int ret;

	ret = mtk_disp_notifier_unregister(&priv->disp_notifier);
	if (ret)
		dev_err(priv->dev, "unreigster disp_notifier err %d", ret);

	if (priv->disp_off_to_on_delay_ms && priv->disp_off_to_on_delay_timeout_ms)
		hrtimer_cancel(&priv->disp_off_to_on_delay_timeout);
}
#endif

static bool rt4539_is_accessible_reg(struct device *dev, unsigned int reg)
{
	if ((reg >= RT4539_REG_00 && reg <= RT4539_REG_EN) || reg == RT4539_REG_MTP)
		return true;
	return false;
}

static const struct regmap_config rt4539_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RT4539_REG_MTP,
	.readable_reg = rt4539_is_accessible_reg,
	.writeable_reg = rt4539_is_accessible_reg,
};

static int rt4539_pwm_brightness_set(struct rt4539_led *priv, unsigned int max_brightness, int brightness)
{
	unsigned long long duty = 0;
	int ret;

	if (brightness) {
		// set pwm brightness
		duty = priv->pwmstate.period;
		duty *= brightness;
		do_div(duty, max_brightness);

		priv->pwmstate.duty_cycle = duty;
		priv->pwmstate.enabled = duty > 0;
	} else {
		priv->pwmstate.duty_cycle = 0;
		priv->pwmstate.enabled = false;
	}

	ret = pwm_apply_state(priv->pwm, &priv->pwmstate);
	if (ret)
		dev_err(priv->dev, "pwm duty %llu (%d) apply error %d", duty, brightness, ret);
	else
		dev_info(priv->dev, "pwm duty: %llu (%d->%d)", duty, priv->curr_hw_brightness, brightness);

	return ret;
}

static int rt4539_i2c_brightness_set(struct rt4539_led *priv, int brightness)
{
	int ret;
	u8 val[2];

	val[0] = (brightness & RT4539_BRIGHTNESS_MSB_MASK) >> RT4539_BRIGHTNESS_MSB_SHIFT;
	val[1] = brightness & RT4539_BRIGHTNESS_LSB_MASK;

	ret = regmap_raw_write(priv->regmap, RT4539_REG_MSB_BRIGHTNESS, val, sizeof(val));
	if (ret) {
		dev_err(priv->dev, "write i2c brightness (%d) error %d", brightness, ret);
		return ret;
	}

	// dev_info(priv->dev, "write i2c brightness: v[0]: 0x%02x v[1]: 0x%02x", val[0], val[1]);
	dev_info(priv->dev, "%d->%d", priv->curr_i2c_brightness, brightness);
	priv->curr_i2c_brightness = brightness;

	return ret;
}

static ssize_t registers_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	int reg, val;
	ssize_t size = 0;

	for (reg = 0; reg <= RT4539_REG_EN; reg++) {
		if (!regmap_read(priv->regmap, reg, &val))
			size += sprintf(&buf[size], "%02x: %02x\n", reg, val);
		else
			size += sprintf(&buf[size], "%02x: XX\n", reg);
	}

	if (!regmap_read(priv->regmap, RT4539_REG_MTP, &val))
		size += sprintf(&buf[size], "%02x: %02x\n", RT4539_REG_MTP, val);
	else
		size += sprintf(&buf[size], "%02x: XX\n", RT4539_REG_MTP);

	return size;
}

static ssize_t registers_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	int ret;
	u8 reg = 0, mask = 0, val = 0;

	ret = sscanf(buf, "%hhx %hhx %hhx", &reg, &mask, &val);
	if (ret != 3){
		dev_err(priv->dev, "format wrong (%x %x %x), please enter 'reg mask val' (hex)", reg, mask, val);
		return -EINVAL;
	}

	if (mask == 0)
		mask = 0xFF;

	// control
	if (reg == RT4539_REG_07 && mask & (RT4539_ADVANCED_CONTROL_MASK | RT4539_PWM_SAMPLE_RATE_MASK)) {
		priv->control = val & RT4539_ADVANCED_CONTROL_MASK;
		if (priv->control != RT4539_CONTROL_I2C && !priv->pwm) {
			dev_info(priv->dev, "no pwm found: use I2C mode");
			priv->control = RT4539_CONTROL_I2C;
		}
		// dev_info(priv->dev, "change control to %d", priv->control);

		if (val & RT4539_PWM_SAMPLE_RATE_MASK) {
			priv->pwm_sample_rate = (val & RT4539_PWM_SAMPLE_RATE_MASK) >> RT4539_PWM_SAMPLE_RATE_SHIFT;
			// dev_info(priv->dev, "change PWM sample rate to %d", priv->pwm_sample_rate);
		}
	}

	// current
	if (reg == RT4539_REG_CURRENT && mask & RT4539_REG_CURRENT_MASK) {
		priv->max_current = val & RT4539_REG_CURRENT_MASK;
		// dev_info(priv->dev, "change max_current to %d", priv->max_current);
	}

	if (reg == RT4539_REG_06) {
		if (priv->time_ctrl < 0)
			priv->time_ctrl = RT4539_REG_06_DEFAULT;

		mask &= RT4539_REG_06_MASK;
		priv->time_ctrl &= ~mask;
		priv->time_ctrl |= val & mask;
		// dev_info(priv->dev, "change time_ctrl to 0x%x", priv->time_ctrl);
	} else if (reg == RT4539_REG_00) {
		if (priv->reg0x00 < 0)
			priv->reg0x00 = RT4539_REG_00_DEFAULT;

		mask &= RT4539_REG_00_MASK;
		priv->reg0x00 &= ~mask;
		priv->reg0x00 |= val & mask;
		// dev_info(priv->dev, "change reg 0x00 to 0x%x", priv->reg0x00);
	} else if (reg == RT4539_REG_01) {
		if (priv->reg0x01 < 0)
			priv->reg0x01 = RT4539_REG_01_DEFAULT;

		mask &= RT4539_REG_01_MASK;
		priv->reg0x01 &= ~mask;
		priv->reg0x01 |= val & mask;
		// dev_info(priv->dev, "change reg 0x01 to 0x%x", priv->reg0x01);
	} else if (reg == RT4539_REG_08) {
		if (priv->reg0x08 < 0)
			priv->reg0x08 = RT4539_REG_08_DEFAULT;

		mask &= RT4539_REG_08_MASK;
		priv->reg0x08 &= ~mask;
		priv->reg0x08 |= val & mask;
		// dev_info(priv->dev, "change reg 0x08 to 0x%x", priv->reg0x08);
	} else if (reg == RT4539_REG_09) {
		if (priv->reg0x09 < 0)
			priv->reg0x09 = RT4539_REG_09_DEFAULT;

		mask &= RT4539_REG_09_MASK;
		priv->reg0x09 &= ~mask;
		priv->reg0x09 |= val & mask;
		// dev_info(priv->dev, "change reg 0x09 to 0x%x", priv->reg0x09);
	}

	// dev_info(priv->dev, "0x%hhx 0x%hhx 0x%hhx", reg, mask, val);

	if (priv->enable) {
		unsigned int old_val = 0, new_val = 0;

		regmap_read(priv->regmap, reg, &old_val);

		ret = regmap_update_bits(priv->regmap, reg, mask, val);
		if (ret) {
			dev_err(priv->dev, "regmap write 0x%x 0x%x 0x%x failed(%d)", reg, mask, val, ret);
			return ret;
		}

		ret = regmap_read(priv->regmap, reg, &new_val);
		if (!ret)
			dev_info(priv->dev, "%02x: %02x -> %02x", reg, old_val, new_val);
		else
			dev_info(priv->dev, "%02x: %02x -> ??", reg, old_val);
	} else
		dev_info(priv->dev, "please turn on backlight");

	return size;
}

static ssize_t i2c_brightness_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	int ret;
	u16 val = 0;

	if (kstrtou16(buf, 0, &val))
		return -EINVAL;

	ret = rt4539_i2c_brightness_set(priv, val);
	if (ret)
		return ret;

	return size;
}

static ssize_t time_ctrl_for_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));

	if (priv->time_ctrl_for_off < 0)
		return sprintf(buf, "%d\n", priv->time_ctrl_for_off);
	else
		return sprintf(buf, "0x%02x\n", priv->time_ctrl_for_off);
}

static ssize_t time_ctrl_for_off_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	int val = 0;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if (val < 0)
		priv->time_ctrl_for_off = -1;
	else {
		/* Refer to kstrtou8() */
		if (val != (u8)val)
			return -ERANGE;

		priv->time_ctrl_for_off = val;
	}

	return size;
}

static DEVICE_ATTR_RW(registers);
static DEVICE_ATTR_WO(i2c_brightness);
static DEVICE_ATTR_RW(time_ctrl_for_off);

static struct attribute *rt4539_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_i2c_brightness.attr,
	&dev_attr_time_ctrl_for_off.attr,
	NULL,
};

static const struct attribute_group rt4539_attr_group = {
	.attrs = rt4539_attributes,
};

static int __maybe_unused led_pwm_get_conn_id(struct mt_led_data *mdev,
		       int flag)
{
	mdev->conf.connector_id = mtk_drm_get_conn_obj_id_from_idx(mdev->desp.index, flag);
	// dev_info(mdev->conf.cdev.dev, "disp_id: %d, connector id %d", mdev->desp.index, mdev->conf.connector_id);
	return 0;
}

static int rt4539_init(struct rt4539_led *priv)
{
	unsigned int mask, val;
	int ret = 0;
	int i;

	// dev_info(priv->dev, "rt4539 init begain +++");

	val = 0;
	mask = RT4539_REG_0B_MASK;
	if (priv->fbs[0] != -1) {
		for (i = 0; i < MAX_FB; i++) {
			if (priv->fbs[i]) {
				// dev_info(priv->dev, "enable fb%d", i+1);
				val |= RT4539_FB1_MASK << i;
			} else {
				// dev_info(priv->dev, "disable fb%d", i+1);
			}
		}
	} else
		val = RT4539_FB1_MASK | RT4539_FB2_MASK | RT4539_FB3_MASK |
			RT4539_FB4_MASK | RT4539_FB5_MASK | RT4539_FB6_MASK;

	// dev_info(priv->dev, "FB: mask=0x%x val=0x%x", mask, val);
	ret = regmap_update_bits(priv->regmap, RT4539_REG_EN, mask, val);
	if (ret) {
		/* retry */
		dev_err(priv->dev, "regmap write fb channel failed %d: retry", ret);

		rt4539_mdelay(priv->enable_post_delay_ms);
		ret = regmap_update_bits(priv->regmap, RT4539_REG_EN, mask, val);
		if (ret) {
			dev_err(priv->dev, "regmap write fb channel failed %d", ret);
			return ret;
		}
	}

	// max_current
	if (priv->max_current != -1) {
		// dev_info(priv->dev, "max_current 0x%x", priv->max_current);
		ret = regmap_update_bits(priv->regmap, RT4539_REG_CURRENT, RT4539_REG_CURRENT_MASK, priv->max_current);
		if (ret) {
			dev_err(priv->dev, "regmap write current (0x%x) failed %d", priv->max_current, ret);
			return ret;
		}
	}

	// advanced brightness control
	// dev_info(priv->dev, "control %d; pwm-sample-rate %d", priv->control, priv->pwm_sample_rate);
	ret = regmap_update_bits(priv->regmap, RT4539_REG_07,
			RT4539_ADVANCED_CONTROL_MASK | RT4539_PWM_SAMPLE_RATE_MASK,
			priv->control | (priv->pwm_sample_rate << RT4539_PWM_SAMPLE_RATE_SHIFT));
	if (ret) {
		dev_err(priv->dev, "regmap write control (0x%x) failed %d", priv->control, ret);
		return ret;
	}

	// Setup Slope Time, Smoothing and Fade IN/OUT
	if (priv->time_ctrl >= 0) {
		ret = regmap_update_bits(priv->regmap, RT4539_REG_06, RT4539_REG_06_MASK, priv->time_ctrl);
		if (ret) {
			dev_err(priv->dev, "regmap write time_ctrl (0x%x) failed %d", priv->time_ctrl, ret);
			return ret;
		}
	}

	// reg 0x00
	if (priv->reg0x00 >= 0) {
		ret = regmap_update_bits(priv->regmap, RT4539_REG_00, RT4539_REG_00_MASK, priv->reg0x00);
		if (ret) {
			dev_err(priv->dev, "regmap write reg 0x00 (0x%x) failed %d", priv->reg0x00, ret);
			return ret;
		}
		// dev_info(priv->dev, "reg 0x00: 0x%x", priv->reg0x00);
	} else {
		// reg 0x00: dc mode
		ret = regmap_update_bits(priv->regmap, RT4539_REG_00, RT4539_MODE_MASK, RT4539_DCMODE);
		if (ret) {
			dev_err(priv->dev, "Failed to write register: reg: %d mask: %d val: %d (%d)",
				RT4539_REG_00, RT4539_MODE_MASK, RT4539_DCMODE, ret);
			return ret;
		}
	}

	// reg 0x01
	if (priv->reg0x01 >= 0) {
		ret = regmap_update_bits(priv->regmap, RT4539_REG_01, RT4539_REG_01_MASK, priv->reg0x01);
		if (ret) {
			dev_err(priv->dev, "regmap write reg 0x01 (0x%x) failed %d", priv->reg0x01, ret);
			return ret;
		}
		// dev_info(priv->dev, "reg 0x01: 0x%x", priv->reg0x01);
	}

	// reg 0x08
	if (priv->reg0x08 >= 0) {
		ret = regmap_update_bits(priv->regmap, RT4539_REG_08, RT4539_REG_08_MASK, priv->reg0x08);
		if (ret) {
			dev_err(priv->dev, "regmap write reg 0x08 (0x%x) failed %d", priv->reg0x08, ret);
			return ret;
		}
		// dev_info(priv->dev, "reg 0x08: 0x%x", priv->reg0x08);
	}

	// reg 0x09
	if (priv->reg0x09 >= 0) {
		ret = regmap_update_bits(priv->regmap, RT4539_REG_09, RT4539_REG_09_MASK, priv->reg0x09);
		if (ret) {
			dev_err(priv->dev, "regmap write reg 0x09 (0x%x) failed %d", priv->reg0x09, ret);
			return ret;
		}
		// dev_info(priv->dev, "reg 0x09: 0x%x", priv->reg0x09);
	}

	// dev_info(priv->dev, "rt4539 init end ---");

	return ret;
}

static void rt4539_disable_delay_work(struct work_struct *work)
{
	struct rt4539_led *priv = container_of(work,
			struct rt4539_led, disable_delay_work.work);

	// dev_info(priv->dev, "+%s()", __func__);

	if (!priv->enable) /* check for the off-to-on case */
		rt4539_disable(priv, "delayed");
}

static int rt4539_brightness_set(struct mt_led_data *mdev,
		       int brightness)
{
	struct rt4539_led *priv = container_of(mdev, struct rt4539_led, m_led);
	int ret = 0;

	// dev_info(priv->dev, "%d->%d", priv->curr_hw_brightness, brightness);

	if (brightness > priv->off_hw_brightness_threshold) {
		bool off_to_on = false;

		if (!priv->enable) {
			priv->enable = true;

			if (delayed_work_pending(&priv->disable_delay_work))
				cancel_delayed_work(&priv->disable_delay_work);
			else
				cancel_delayed_work_sync(&priv->disable_delay_work);

			ret = gpiod_direction_output(priv->enable_gpio, 1);
			if (ret)
				dev_notice(priv->dev, "Failed to set enable_gpio active, err %d", ret);

			rt4539_mdelay(priv->enable_post_delay_ms);

			ret = rt4539_init(priv);
			if (ret)
				dev_notice(priv->dev, "rt4539 init failed (%d)", ret);

			off_to_on = true;
		}

		if (priv->control != RT4539_CONTROL_I2C /*&& priv->pwm*/) {
			// set pwm brightness
			rt4539_pwm_brightness_set(priv, mdev->conf.max_hw_brightness, brightness);

			if (off_to_on) {
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
				if (priv->disp_off_to_on_delay_ms && priv->disp_off_to_on_delay) {
					priv->disp_off_to_on_delay = false;
					dev_notice(priv->dev, "disp-on delay");
					rt4539_mdelay(priv->disp_off_to_on_delay_ms);
				}
#endif
				ret = regmap_update_bits(priv->regmap, RT4539_REG_EN, RT4539_EN_MASK, RT4539_EN_MASK);
				if (ret)
					dev_notice(priv->dev, "set BL_EN failed: %d", ret);

				dev_alert(priv->dev, "on");
			}
		} else {
			// set i2c brightness
			if (off_to_on) {
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
				if (priv->disp_off_to_on_delay_ms && priv->disp_off_to_on_delay) {
					priv->disp_off_to_on_delay = false;
					dev_notice(priv->dev, "disp-on delay");
					rt4539_mdelay(priv->disp_off_to_on_delay_ms);
				}
#endif
				ret = regmap_update_bits(priv->regmap, RT4539_REG_EN, RT4539_EN_MASK, RT4539_EN_MASK);
				if (ret)
					dev_notice(priv->dev, "set BL_EN failed: %d", ret);
			}

			rt4539_i2c_brightness_set(priv, brightness);

			if (off_to_on)
				dev_alert(priv->dev, "on");
		}
	} else if (priv->enable) {
		priv->enable = false;

		if (brightness > 0 || (priv->m_led.conf.cdev.flags & LED_SYSFS_DISABLE) || priv->display_off) {
			/* display already off, LED_SYSFS_DISABLE or display on backlight off */
			unsigned int disable_delay = priv->screen_on_backlight_off_pre_delay_ms;

			if (!priv->display_off) {
				int new_time_ctrl = -1; /* reg 0x06 */
				unsigned int slop_time, fade_time;

				if (priv->time_ctrl_for_off >= 0) {
					/* Setup New Time for Backlight off */
					ret = regmap_update_bits(priv->regmap, RT4539_REG_06, RT4539_REG_06_MASK, priv->time_ctrl_for_off);
					if (ret)
						dev_err(priv->dev, "regmap write time_ctrl_for_off (0x%x) failed %d", priv->time_ctrl_for_off, ret);
					else
						new_time_ctrl = priv->time_ctrl_for_off;
				}

				if (priv->control != RT4539_CONTROL_I2C /*&& priv->pwm*/)
					// set pwm brightness
					rt4539_pwm_brightness_set(priv, mdev->conf.max_hw_brightness, 0);
				else
					// set i2c brightness
					rt4539_i2c_brightness_set(priv, 0);

				if (new_time_ctrl < 0)
					new_time_ctrl = (priv->time_ctrl >= 0) ? priv->time_ctrl : RT4539_REG_06_DEFAULT;
				// dev_info(priv->dev, "new_time_ctrl for off: 0x%x", new_time_ctrl);

				slop_time = slop_time_ms[(new_time_ctrl & RT4539_REG_06_SLOPE_TIME_MASK) >> RT4539_REG_06_SLOPE_TIME_SHIFT];
				fade_time = fade_time_us[(new_time_ctrl & RT4539_REG_06_FADE_TIME_MASK) >> RT4539_REG_06_FADE_TIME_SHIFT];
				fade_time = DIV_ROUND_UP(fade_time * priv->curr_hw_brightness, 1000); /* ms */
				fade_time = max_t(unsigned int, fade_time, slop_time);

				disable_delay = max_t(unsigned int, disable_delay, fade_time);
				// dev_info(priv->dev, "display on backlight off waiting ms: %u", disable_delay);
			}

			if ((priv->m_led.conf.cdev.flags & LED_SYSFS_DISABLE) || priv->display_off) {
				//rt4539_mdelay(disable_delay); /* not required */
				rt4539_disable(priv, "immediate");
			} else {
				queue_delayed_work(system_highpri_wq,
						&priv->disable_delay_work, msecs_to_jiffies(disable_delay));
			}
		} else {
			/* display off backlight off */
			int new_time_ctrl = -1; /* reg 0x06 */

			if (priv->time_ctrl_for_off >= 0) {
				/* Setup New Time for Backlight off */
				ret = regmap_update_bits(priv->regmap, RT4539_REG_06, RT4539_REG_06_MASK, priv->time_ctrl_for_off);
				if (ret)
					dev_err(priv->dev, "regmap write time_ctrl_for_off (0x%x) failed %d", priv->time_ctrl_for_off, ret);
				else
					new_time_ctrl = priv->time_ctrl_for_off;
			}

			if (priv->control != RT4539_CONTROL_I2C /*&& priv->pwm*/)
				// set pwm brightness
				rt4539_pwm_brightness_set(priv, mdev->conf.max_hw_brightness, 0);
			else
				// set i2c brightness
				rt4539_i2c_brightness_set(priv, 0);

			if (new_time_ctrl < 0)
				new_time_ctrl = (priv->time_ctrl >= 0) ? priv->time_ctrl : RT4539_REG_06_DEFAULT;
			// dev_info(priv->dev, "new_time_ctrl for off: 0x%x", new_time_ctrl);

			/* Wait for turning backlight off */
			if (priv->curr_hw_brightness > priv->fade_off_min_hw_brightness) {
				unsigned int slop_time, fade_time;

				slop_time = slop_time_ms[(new_time_ctrl & RT4539_REG_06_SLOPE_TIME_MASK) >> RT4539_REG_06_SLOPE_TIME_SHIFT];
				if (slop_time >= priv->screen_off_backlight_off_pre_delay_ms)
					slop_time -= priv->screen_off_backlight_off_pre_delay_ms;
				else
					slop_time = 0;

				fade_time = fade_time_us[(new_time_ctrl & RT4539_REG_06_FADE_TIME_MASK) >> RT4539_REG_06_FADE_TIME_SHIFT];
				fade_time *= (priv->curr_hw_brightness - priv->fade_off_min_hw_brightness);
				fade_time = DIV_ROUND_UP(fade_time, 1000); /* ms */

				fade_time = max_t(unsigned int, fade_time, slop_time);
				// dev_info(priv->dev, "display off backlight off waiting ms: %u", fade_time);

				rt4539_mdelay(fade_time);
			}

			queue_delayed_work(system_highpri_wq,
					&priv->disable_delay_work,
					msecs_to_jiffies(priv->screen_off_backlight_off_pre_delay_ms));
		}
		brightness = 0;
	} else
		brightness = 0;

	priv->curr_hw_brightness = brightness;

	return ret;
}

__attribute__((nonnull))
static int rt4539_pwm_add(struct device *dev, struct rt4539_led *priv,
		       struct fwnode_handle *fwnode)
{
	int ret;

	priv->pwm = devm_fwnode_pwm_get(dev, fwnode, NULL);
	if (IS_ERR(priv->pwm)) {
		ret = PTR_ERR(priv->pwm);
		if (ret != -ENOENT)
			dev_err(priv->dev, "unable to request PWM: %d", ret);
		priv->pwm = NULL;
	} else
		pwm_init_state(priv->pwm, &priv->pwmstate);

	ret = mt_leds_classdev_register(dev, &priv->m_led);
	if (ret < 0) {
		dev_err(priv->dev, "failed to register MTK led: %d", ret);
		return ret;
	}
	priv->m_led.conf.cdev.flags &= ~(LED_CORE_SUSPENDRESUME); /* Disable suspend/resume feature */

	if (priv->m_led.conf.cdev.dev)
		priv->dev = priv->m_led.conf.cdev.dev;

	return 0;
}

static int rt4539_create_fwnode(struct device *dev, struct rt4539_led *priv)
{
	struct fwnode_handle *fwnode;
	int ret = 0;

	// dev_info(priv->dev, "create fwnode begain +++");

	fwnode = dev->fwnode;

	ret = mt_leds_parse_dt(&priv->m_led, fwnode);
	if (ret < 0) {
		fwnode_handle_put(fwnode);
		return -EINVAL;
	}

	priv->m_led.mtk_hw_brightness_set = rt4539_brightness_set;
	priv->m_led.mtk_conn_id_get = led_pwm_get_conn_id;

	priv->enable_post_delay_ms = EN_POWEN_ON_DELAY;
	fwnode_property_read_u32(fwnode, "enable-post-delay", &priv->enable_post_delay_ms);
	if (priv->enable_post_delay_ms < EN_POWEN_ON_DELAY)
		priv->enable_post_delay_ms = EN_POWEN_ON_DELAY;
	// dev_info(priv->dev, "enable-post-delay=%u", priv->enable_post_delay_ms);

	if (fwnode_property_read_u32(fwnode, "screen-on-backlight-off-pre-delay",
			&priv->screen_on_backlight_off_pre_delay_ms))
		priv->screen_on_backlight_off_pre_delay_ms = 20;
	// dev_info(priv->dev, "screen-on-backlight-off-pre-delay=%u", priv->screen_on_backlight_off_pre_delay_ms);

	if (fwnode_property_read_u32(fwnode, "screen-off-backlight-off-pre-delay",
			&priv->screen_off_backlight_off_pre_delay_ms))
		priv->screen_off_backlight_off_pre_delay_ms = 20;
	// dev_info(priv->dev, "screen-off-backlight-off-pre-delay=%u", priv->screen_off_backlight_off_pre_delay_ms);

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	if (fwnode_property_read_u32(fwnode, "disp-off-to-on-delay", &priv->disp_off_to_on_delay_ms))
		priv->disp_off_to_on_delay_ms = 0;
	// dev_info(priv->dev, "off-to-on-delay=%u", priv->disp_off_to_on_delay_ms);

	if (fwnode_property_read_u32(fwnode, "disp-off-to-on-delay-timeout", &priv->disp_off_to_on_delay_timeout_ms))
		priv->disp_off_to_on_delay_timeout_ms = 0;
	// dev_info(priv->dev, "off-to-on-delay-timeout=%u", priv->disp_off_to_on_delay_timeout_ms);
#endif

	if (fwnode_property_read_u32_array(fwnode, "fbs", priv->fbs, MAX_FB)) {
		dev_info(priv->dev, "use default fb channel");
		priv->fbs[0] = -1;
	}

	// reg 0x00
	if (!fwnode_property_read_u32(fwnode, "reg0x00", &priv->reg0x00)) {
		if (priv->reg0x00 < 0 || priv->reg0x00 > 0xff) {
			dev_err(priv->dev, "reg0x00 is out of range (0-0xff)");
			priv->reg0x00 = -1;
		} else {
			priv->reg0x00 &= RT4539_REG_00_MASK;
			// dev_info(priv->dev, "reg0x00: 0x%x", priv->reg0x00);
		}
	} else
		priv->reg0x00 = -1;

	// reg 0x01
	if (!fwnode_property_read_u32(fwnode, "reg0x01", &priv->reg0x01)) {
		if (priv->reg0x01 < 0 || priv->reg0x01 > 0xff) {
			dev_err(priv->dev, "reg0x01 is out of range (0-0xff)");
			priv->reg0x01 = -1;
		} else {
			priv->reg0x01 &= RT4539_REG_01_MASK;
			// dev_info(priv->dev, "reg0x01: 0x%x", priv->reg0x01);
		}
	} else
		priv->reg0x01 = -1;

	// reg 0x06
	if (!fwnode_property_read_u32(fwnode, "time-ctrl", &priv->time_ctrl)) {
		if (priv->time_ctrl < 0 || priv->time_ctrl > 0xff) {
			dev_err(priv->dev, "time-ctrl is out of range (0-0xff)");
			priv->time_ctrl = -1;
		} else {
			priv->time_ctrl &= RT4539_REG_06_MASK;
			// dev_info(priv->dev, "time-ctrl: 0x%x", priv->time_ctrl);
		}
	} else
		priv->time_ctrl = -1;

	/* hw brightness threshold for backlight off */
	if (fwnode_property_read_u32(fwnode, "off-hw-brightness-thresh", &priv->off_hw_brightness_threshold))
		priv->off_hw_brightness_threshold = 0;

	if (fwnode_property_read_u32(fwnode, "fade-off-min-hw-brightness", &priv->fade_off_min_hw_brightness))
		priv->fade_off_min_hw_brightness = 0;
	// dev_info(priv->dev, "fade_off_min_hw_brightness(DTS): %u", priv->fade_off_min_hw_brightness);

	// reg 0x06 for brightness off
	if (!fwnode_property_read_u32(fwnode, "time-ctrl-for-off", &priv->time_ctrl_for_off)) {
		if (priv->time_ctrl_for_off < 0 || priv->time_ctrl_for_off > 0xff) {
			dev_err(priv->dev, "time-ctrl-for-off is out of range (0-0xff)");
			priv->time_ctrl_for_off = -1;
		} else {
			unsigned int fade_time;

			priv->time_ctrl_for_off &= RT4539_REG_06_MASK;
			// dev_info(priv->dev, "time-ctrl-for-off: 0x%x", priv->time_ctrl_for_off);

			if (priv->fade_off_min_hw_brightness == 0) {
				fade_time = fade_time_us[(priv->time_ctrl_for_off & RT4539_REG_06_FADE_TIME_MASK) >> RT4539_REG_06_FADE_TIME_SHIFT];
				priv->fade_off_min_hw_brightness = priv->screen_off_backlight_off_pre_delay_ms * 1000 / fade_time;
				// dev_info(priv->dev, "fade_off_min_hw_brightness: %u", priv->fade_off_min_hw_brightness);
			}
		}
	} else
		priv->time_ctrl_for_off = -1;

	// reg 0x08
	if (!fwnode_property_read_u32(fwnode, "reg0x08", &priv->reg0x08)) {
		if (priv->reg0x08 < 0 || priv->reg0x08 > 0xff) {
			dev_err(priv->dev, "reg0x08 is out of range (0-0xff)");
			priv->reg0x08 = -1;
		} else {
			priv->reg0x08 &= RT4539_REG_08_MASK;
			// dev_info(priv->dev, "reg0x08: 0x%x", priv->reg0x08);
		}
	} else
		priv->reg0x08 = -1;

	// reg 0x09
	if (!fwnode_property_read_u32(fwnode, "reg0x09", &priv->reg0x09)) {
		if (priv->reg0x09 < 0 || priv->reg0x09 > 0xff) {
			dev_err(priv->dev, "reg0x09 is out of range (0-0xff)");
			priv->reg0x09 = -1;
		} else {
			priv->reg0x09 &= RT4539_REG_09_MASK;
			// dev_info(priv->dev, "reg0x09: 0x%x", priv->reg0x09);
		}
	} else
		priv->reg0x09 = -1;

	// current
	if (!fwnode_property_read_u32(fwnode, "current", &priv->max_current)) {
		if (priv->max_current < 0 || priv->max_current > RT4539_REG_CURRENT_MAX) {
			dev_info(priv->dev, "current is out of range(0~%d) use default current(20mA)", RT4539_REG_CURRENT_MAX);
			priv->max_current = RT4539_REG_CURRENT_DEFAULT;
		}
	} else
		priv->max_current = RT4539_REG_CURRENT_DEFAULT;

	if (!fwnode_property_read_u32(fwnode, "control", &priv->control)) {
		if (priv->control < RT4539_CONTROL_PWM || priv->control > RT4539_CONTROL_I2C) {
			dev_info(priv->dev, "control value is wrong use pwm");
			priv->control = RT4539_CONTROL_PWM;
		}
	} else
		priv->control = RT4539_CONTROL_PWM;

	if (!fwnode_property_read_u32(fwnode, "pwm-sample-rate", &priv->pwm_sample_rate)) {
		if (priv->pwm_sample_rate < RT4539_PWM_SAMPLE_RATE_AUTO ||
				priv->pwm_sample_rate > RT4539_PWM_SAMPLE_RATE_MAX) {
			dev_err(priv->dev, "invalid pwm-sample-rate: %d; use auto", priv->pwm_sample_rate);
			priv->pwm_sample_rate = RT4539_PWM_SAMPLE_RATE_AUTO;
		} else {
			// dev_info(priv->dev, "pwm-sample-rate: %d", priv->pwm_sample_rate);
		}
	} else
		priv->pwm_sample_rate = RT4539_PWM_SAMPLE_RATE_AUTO;

	ret = rt4539_pwm_add(dev, priv, fwnode);
	if (ret) {
		fwnode_handle_put(fwnode);
		dev_err(priv->dev, "led: %s add pwm failed", priv->m_led.conf.cdev.name);
		return ret;
	}

	if (priv->control != RT4539_CONTROL_I2C && !priv->pwm) {
		dev_info(priv->dev, "no pwm found: use I2C mode");
		priv->control = RT4539_CONTROL_I2C;
	}

	dev_info(priv->dev, "parse led: %s max: %d",
		priv->m_led.conf.cdev.name, priv->m_led.conf.cdev.max_brightness);

	return ret;
}

static int rt4539_probe(struct i2c_client *client)
{
	struct rt4539_led *priv;
	int ret = 0, val;

	// dev_info(&client->dev, "probe begain +++");

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto err;
	}

	priv->client = client;
	priv->dev = &client->dev;
	priv->curr_hw_brightness = 0;
	priv->curr_i2c_brightness = 0;

	i2c_set_clientdata(client, priv);

	ret = rt4539_create_fwnode(&client->dev, priv);
	if (ret < 0) {
		dev_err(priv->dev, "create fwnode failed %d", ret);
		goto err;
	}

	priv->enable_gpio = devm_gpiod_get_optional(&client->dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->enable_gpio)) {
		ret = PTR_ERR(priv->enable_gpio);
		dev_err(priv->dev, "failed to get 'enable' GPIO: %d", ret);
		goto err;
	}
	rt4539_mdelay(priv->enable_post_delay_ms);

	priv->regmap = devm_regmap_init_i2c(client, &rt4539_regmap_config);
	if (IS_ERR(priv->regmap)) {
		gpiod_direction_output(priv->enable_gpio, 0);
		ret = PTR_ERR(priv->regmap);
		dev_err(priv->dev, "Failed to initialize regmap: %d", ret);
		goto err;
	}

	ret = regmap_read(priv->regmap, RT4539_REG_00, &val);
	if (ret) {
		gpiod_direction_output(priv->enable_gpio, 0);
		dev_err(priv->dev, "Failed to read register: %d", ret);
		goto err;
	}
	// dev_info(priv->dev, "read register 0x%02x: 0x%02x(0x6B)", RT4539_REG_00, val);

	gpiod_direction_output(priv->enable_gpio, 0);

	INIT_DELAYED_WORK(&priv->disable_delay_work, rt4539_disable_delay_work);

	ret = rt4539_brightness_set(&priv->m_led, priv->m_led.conf.cdev.brightness);
	if (ret) {
		gpiod_direction_output(priv->enable_gpio, 0);
		dev_err(priv->dev, "set brightness failed(%d)", ret);
		goto err;
	}

	ret = sysfs_create_group(&client->dev.kobj, &rt4539_attr_group);
	if (ret) {
		gpiod_direction_output(priv->enable_gpio, 0);
		dev_err(priv->dev, "sysfs create failed(%d)", ret);
		goto err;
	}

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	register_disp_notifier(priv);
#endif

	dev_info(priv->dev, "probed");

	return 0;
err:
	dev_err(priv->dev, "Failed to probe: %d!", ret);
	return ret;
}

static int rt4539_remove(struct i2c_client *client)
{
	struct rt4539_led *priv = i2c_get_clientdata(client);
	int ret;

	/* Refer to led_sysfs_disable() */
	priv->m_led.conf.cdev.flags |= LED_SYSFS_DISABLE;

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	unregister_disp_notifier(priv);
#endif

	if (priv->enable) {
		ret = rt4539_brightness_set(&priv->m_led, 0);
		if (ret) {
			dev_err(priv->dev, "set brightness err %d", ret);
			rt4539_disable(priv, "removed");
		}
	} else {
		if (delayed_work_pending(&priv->disable_delay_work)) {
			cancel_delayed_work(&priv->disable_delay_work);
		} else
			cancel_delayed_work_sync(&priv->disable_delay_work);
		rt4539_disable(priv, "removed force");
	}

	mt_leds_classdev_unregister(priv->dev, &priv->m_led);

	sysfs_remove_group(&client->dev.kobj, &rt4539_attr_group);

	return 0;
}

static void rt4539_shutdown(struct i2c_client *client)
{
	struct rt4539_led *priv = i2c_get_clientdata(client);
	int ret;

	/* Refer to led_sysfs_disable() */
	priv->m_led.conf.cdev.flags |= LED_SYSFS_DISABLE;

	if (priv->enable) {
		ret = rt4539_brightness_set(&priv->m_led, 0);
		if (ret) {
			dev_err(priv->dev, "set brightness err %d", ret);
			rt4539_disable(priv, "shutdown");
		}
	} else {
		if (delayed_work_pending(&priv->disable_delay_work)) {
			cancel_delayed_work(&priv->disable_delay_work);
		} else
			cancel_delayed_work_sync(&priv->disable_delay_work);
		rt4539_disable(priv, "shutdown force");
	}
}

static const struct of_device_id __maybe_unused rt4539_of_match[] = {
	{ .compatible = "richtek,rt4539", },
	{}
};
MODULE_DEVICE_TABLE(of, rt4539_of_match);
static struct i2c_driver rt4539_driver = {
	.driver = {
		.name = "rt4539",
		.of_match_table = rt4539_of_match,
	},
	.probe_new = rt4539_probe,
	.remove = rt4539_remove,
	.shutdown = rt4539_shutdown,
};
module_i2c_driver(rt4539_driver);

MODULE_AUTHOR("InnoComm");
MODULE_DESCRIPTION("Richtek RT4539 PWM Backlight Driver");
MODULE_LICENSE("GPL v2");

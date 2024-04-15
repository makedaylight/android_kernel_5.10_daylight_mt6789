// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 InnoComm.
 *
 */

// #define RT4539_DISPLAY_BACKLIGHT_TIME_LATENCY_DEBUG
// #define RT4539_DISPLAY_BACKLIGHT_DEBUG
// #define RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
#define pr_fmt(fmt) KBUILD_MODNAME " %s#%d: " fmt "\n", __func__, __LINE__
#define dev_fmt(fmt) " %s#%d: " fmt "\n", __func__, __LINE__
#else
#define pr_fmt(fmt) KBUILD_MODNAME " %s: " fmt "\n", __func__
#define dev_fmt(fmt) " %s: " fmt "\n", __func__
#endif

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
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/sched/task.h>
#include <uapi/linux/sched/types.h>
#include <linux/completion.h>
#include <linux/ktime.h>
#include <linux/ratelimit_types.h>

#include <leds-mtk.h>

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
#include <linux/hrtimer.h>
#include "mtk_disp_notify.h"
#else
#error "Must enable CONFIG_DRM_MEDIATEK"
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
#define RT4539_REG_06_SLOPE_TIME_0ms	(0x0 << RT4539_REG_06_SLOPE_TIME_SHIFT)
#define RT4539_REG_06_SLOPE_TIME_1ms	(0x1 << RT4539_REG_06_SLOPE_TIME_SHIFT)
#define RT4539_REG_06_SLOPE_TIME_8ms	(0x2 << RT4539_REG_06_SLOPE_TIME_SHIFT)
#define RT4539_REG_06_SLOPE_TIME_128ms	(0x3 << RT4539_REG_06_SLOPE_TIME_SHIFT)
#define RT4539_REG_06_SLOPE_TIME_256ms	(0x4 << RT4539_REG_06_SLOPE_TIME_SHIFT)
#define RT4539_REG_06_SLOPE_TIME_512ms	(0x5 << RT4539_REG_06_SLOPE_TIME_SHIFT)
#define RT4539_REG_06_SLOPE_TIME_768ms	(0x6 << RT4539_REG_06_SLOPE_TIME_SHIFT)
#define RT4539_REG_06_SLOPE_TIME_1024ms	(0x7 << RT4539_REG_06_SLOPE_TIME_SHIFT)
#define RT4539_REG_06_SLOPE_FILTER_MASK	GENMASK(7, 6)
#define RT4539_REG_06_SLOPE_FILTER_SHIFT	6
#define RT4539_REG_06_SLOPE_FILTER_LIGHT	(0x01 << RT4539_REG_06_SLOPE_FILTER_SHIFT)
#define RT4539_REG_06_SLOPE_FILTER_MEDIUM	(0x02 << RT4539_REG_06_SLOPE_FILTER_SHIFT)
#define RT4539_REG_06_SLOPE_FILTER_HEAVY	(0x03 << RT4539_REG_06_SLOPE_FILTER_SHIFT)
#define RT4539_REG_06_SLOPE_LIGHT_0ms	(RT4539_REG_06_SLOPE_FILTER_LIGHT | RT4539_REG_06_SLOPE_TIME_0ms)
#define RT4539_REG_06_SLOPE_LIGHT_1ms	(RT4539_REG_06_SLOPE_FILTER_LIGHT | RT4539_REG_06_SLOPE_TIME_1ms)
#define RT4539_REG_06_SLOPE_LIGHT_8ms	(RT4539_REG_06_SLOPE_FILTER_LIGHT | RT4539_REG_06_SLOPE_TIME_8ms)
#define RT4539_REG_06_SLOPE_LIGHT_128ms	(RT4539_REG_06_SLOPE_FILTER_LIGHT | RT4539_REG_06_SLOPE_TIME_128ms)

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

#define RT4539_BACKLIGHT_OFF_FADING_OUT_MAX_TIMEOUT_MS	(1536)
#define RT4539_BACKLIGHT_ON_MAX_TIMEOUT_MS	(200)

#define RT4539_EVENT_STOP	0
#define RT4539_EVENT_BL_ON	1
#define RT4539_EVENT_BL_OFF	2
#define RT4539_EVENT_BL_FORCE_DISABLE	3

struct rt4539_led {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;

	int curr_hw_brightness;
	int curr_pwm_brightness;
	int curr_i2c_brightness;

	struct mt_led_data m_led;
	struct pwm_device *pwm;
	struct pwm_state pwmstate;

	struct gpio_desc *enable_gpio;
	const char* on_off_reason;
	bool hw_enabled;
	bool display_early_on_enable_backlight;
	bool backlight_off_waiting_for_off_fading_out;
	bool backlight_off_start_off_saftey_timer;
	bool enable;

	int control;
	int pwm_sample_rate;
	int max_current;
	int reg0x00; /* reg 0x00 */
	int reg0x01; /* reg 0x01 */
	int time_ctrl; /* reg 0x06 */
	int screen_off_backlight_off_time_ctrl; /* reg 0x06 for brightness off */
	unsigned int can_not_see_backlight_brightness_threshold;
	unsigned int can_not_see_backlight_brightness_threshold_hw;
	unsigned int hw_brightness_on_threshold;
	int soft_start_ctrl; /* reg 0x08 */
	int clk_pfm_ctrl; /* reg 0x09 */
	int fbs[MAX_FB];

	spinlock_t lock;
	struct task_struct *kthread;
	wait_queue_head_t wait_queue;
	unsigned long event;

	struct hrtimer timer_off_fading_out;
	struct hrtimer timer_off;
	struct completion backlight_on_complete;
	struct completion backlight_off_fading_out_complete;

	unsigned int enable_post_delay_ms;
	unsigned int display_on_backlight_on_delay_us;

	unsigned int screen_off_backlight_off_safety_delay_us;
	unsigned int screen_off_backlight_off_extra_latency_us;
	unsigned int hwcomposer_disabled_backlight_off_delay_us;
	bool hwcomposer_disabled_backlight_force_off;
	bool hwcomposer_disabled_backlight_off_bypass; /* for safety timer debugging */
#ifdef RT4539_DISPLAY_BACKLIGHT_TIME_LATENCY_DEBUG
	s64 screen_off_backlight_off_expected_fading_out_time; /* us */
	ktime_t screen_off_backlight_off_start;
	ktime_t screen_off_backlight_off_delay_start;
	ktime_t hwcomposer_disabled_backlight_off_delay_start;
	ktime_t screen_off_backlight_off_delay_end;
	ktime_t screen_off_backlight_off_end;
#endif

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	bool display_off;
	struct notifier_block disp_notifier;
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

static int rt4539_pwm_brightness_set(struct rt4539_led *priv, unsigned int max_brightness, int brightness);
static int rt4539_i2c_brightness_set(struct rt4539_led *priv, int brightness);
static void rt4539_start_off_fading_out_timer(struct rt4539_led *priv, unsigned int us);
static void rt4539_cancel_off_fading_out_timer(struct rt4539_led *priv);
static void rt4539_wait_for_off_fading_out(struct rt4539_led *priv);
static void rt4539_start_off_timer(struct rt4539_led *priv, const char* reason, unsigned int us);
static void rt4539_cancel_off_timer(struct rt4539_led *priv);
static void rt4539_start_on(struct rt4539_led *priv, const char* reason);
static void rt4539_wait_for_on(struct rt4539_led *priv);

/* true: allowed */
static bool rt4539_ratelimit(struct ratelimit_state *rs)
{
	/* Paired with WRITE_ONCE() in .proc_handler().
	 * Changing two values seperately could be inconsistent
	 * and some message could be lost.  (See: net_ratelimit_state).
	 */
	int interval = READ_ONCE(rs->interval);
	int burst = READ_ONCE(rs->burst);
#if 0
	unsigned long flags;
#endif
	bool ret;

	if (!interval)
		return true;

#if 0
	/*
	 * If we contend on this state's lock then almost
	 * by definition we are too busy to print a message,
	 * in addition to the one that will be printed by
	 * the entity that is holding the lock already:
	 */
	if (!raw_spin_trylock_irqsave(&rs->lock, flags))
		return false;
#endif

	if (!rs->begin)
		rs->begin = jiffies;

	if (time_is_before_jiffies(rs->begin + interval)) {
#if 0
		if (rs->missed) {
			if (!(rs->flags & RATELIMIT_MSG_ON_RELEASE))
				rs->missed = 0;
		}
#else
		rs->missed  = 0;
#endif
		rs->begin   = jiffies;
		rs->printed = 0;
	}
	if (burst && burst > rs->printed) {
		rs->printed++;
		ret = true;
	} else {
		rs->missed++;
		ret = false;
	}
#if 0
	raw_spin_unlock_irqrestore(&rs->lock, flags);
#endif

	return ret;
}

static inline void rt4539_udelay(unsigned int us)
{
	if (us >= 10)
		usleep_range(us, us);
	else if (us)
		udelay(us);
}

static inline void rt4539_mdelay(unsigned int ms)
{
	if (ms >= 20)
		msleep(ms);
	else if (ms)
		usleep_range(ms * USEC_PER_MSEC, ms * USEC_PER_MSEC + 500);
}

/* Refer to brightness_maptolevel() in leds-mtk.c */
static inline int brightness_maptolevel(struct led_conf_info *led_conf, int brightness)
{
	return (((led_conf->max_hw_brightness) * brightness
				+ ((led_conf->cdev.max_brightness) / 2))
				/ (led_conf->cdev.max_brightness));
}

static unsigned int rt4539_calc_fading_out_hw_brightness_threshold(struct rt4539_led *priv, unsigned int fade_time_us)
{
	unsigned int screen_off_backlight_off_delay_hw_brightness;
	unsigned int fading_out_hw_brightness_threshold;

	if (fade_time_us == 0)
		fade_time_us = 1; /* sanity check */

	screen_off_backlight_off_delay_hw_brightness = DIV_ROUND_UP(
			priv->screen_off_backlight_off_extra_latency_us,
			fade_time_us);

	/* WONT wait for backlight fading out if the HW brightness is less than fading_out_hw_brightness_threshold */
	fading_out_hw_brightness_threshold = screen_off_backlight_off_delay_hw_brightness +
				priv->can_not_see_backlight_brightness_threshold_hw;

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_alert(priv->dev, "%u (%uus->%u,%u)", fading_out_hw_brightness_threshold,
			priv->screen_off_backlight_off_extra_latency_us,
			screen_off_backlight_off_delay_hw_brightness,
			priv->can_not_see_backlight_brightness_threshold_hw);
#endif

	return fading_out_hw_brightness_threshold;
}

static inline unsigned int rt4539_get_slop_time_ms(int time_ctrl)
{
	unsigned int slop_time; /* ms */

	slop_time = slop_time_ms[(time_ctrl & RT4539_REG_06_SLOPE_TIME_MASK) >> RT4539_REG_06_SLOPE_TIME_SHIFT];
	switch (time_ctrl & RT4539_REG_06_SLOPE_FILTER_MASK) {
	case RT4539_REG_06_SLOPE_FILTER_MEDIUM:
		slop_time = slop_time * 120 / 100;
		break;
	case RT4539_REG_06_SLOPE_FILTER_HEAVY:
		slop_time = slop_time * 140 / 100;
		break;
	case RT4539_REG_06_SLOPE_FILTER_LIGHT:
	default:
		break;
	}
	
	return slop_time;
}

static void rt4539_enable(struct rt4539_led *priv, const char* log)
{
	int ret;

	if (!priv->hw_enabled) {
		priv->hw_enabled = true;

		ret = gpiod_direction_output(priv->enable_gpio, 1);
		if (ret)
			dev_err(priv->dev, "[%s] Failed to set enable_gpio active, err %d", log, ret);
		else
			dev_alert(priv->dev, "%s", log);

		priv->curr_hw_brightness = 0;
		priv->curr_pwm_brightness = 0;
		priv->curr_i2c_brightness = 0;

#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	} else {
		dev_alert(priv->dev, "%s (already on)", log);
#endif
	}
}

static void rt4539_disable(struct rt4539_led *priv, const char* log)
{
	/* no I2C access here because the enable gpio may be deactivated */
	int ret;

	if (priv->hw_enabled) {
		priv->hw_enabled = false;

		ret = gpiod_direction_output(priv->enable_gpio, 0);
		if (ret)
			dev_err(priv->dev, "[%s] Failed to set enable_gpio deactive, err %d", log, ret);
		else
			dev_alert(priv->dev, "%s", log);

		priv->curr_hw_brightness = 0;
		priv->curr_pwm_brightness = 0;
		priv->curr_i2c_brightness = 0;

#ifdef RT4539_DISPLAY_BACKLIGHT_TIME_LATENCY_DEBUG
		priv->screen_off_backlight_off_end = ktime_get();
#endif
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	} else {
		dev_alert(priv->dev, "%s (already off)", log);
#endif
	}
}

/* must check (priv->hw_enabled) before calling */
static void rt4539_immediately_backlight_off(struct rt4539_led *priv,
		int time_ctrl, bool force_disable, const char* log)
{
	unsigned long flags;

	if (time_ctrl < 0 && force_disable) {
		/* Display will be off immediately so we must turn off backlight right now! */
		spin_lock_irqsave(&priv->lock, flags);
		if (!priv->enable && priv->hw_enabled && !test_bit(RT4539_EVENT_BL_ON, &priv->event)) /* check for the off-to-on case */
			rt4539_disable(priv, log);
		spin_unlock_irqrestore(&priv->lock, flags);
	} else if (!priv->enable) { /* check for the off-to-on case */
		int ret;

		/* Setup New Time for Backlight off */
		ret = regmap_update_bits(priv->regmap, RT4539_REG_06, RT4539_REG_06_MASK, time_ctrl);
		if (ret)
			dev_err(priv->dev, "regmap write new time_ctrl (0x%x) failed %d", time_ctrl, ret);

		if (priv->control != RT4539_CONTROL_I2C /*&& priv->pwm*/)
			// set pwm brightness
			rt4539_pwm_brightness_set(priv, priv->m_led.conf.max_hw_brightness, 0);
		else
			// set i2c brightness
			rt4539_i2c_brightness_set(priv, 0);

		if (force_disable) {
			spin_lock_irqsave(&priv->lock, flags);
			rt4539_disable(priv, log);
			spin_unlock_irqrestore(&priv->lock, flags);
		}
	}
}

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
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

			if (priv->display_early_on_enable_backlight) {
#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
				dev_alert(priv->dev, "display early on; backlight turning on");
#endif

				rt4539_start_on(priv, "display early on");
			} else {
#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
				dev_alert(priv->dev, "display early on; backlight keep off");
#endif
			}
		} else if (*ev_data == MTK_DISP_BLANK_POWERDOWN) {
			priv->display_off = true;

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
			if (priv->hw_enabled)
				dev_alert(priv->dev, "display early off; backlight still on");
			else
				dev_alert(priv->dev, "display early off; backlight off");
#endif
		}
	} else if (event == MTK_DISP_EVENT_BLANK) {
		if (*ev_data == MTK_DISP_BLANK_UNBLANK) {
#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
			dev_alert(priv->dev, "display on");
#endif
		} else if (*ev_data == MTK_DISP_BLANK_POWERDOWN) {
			priv->backlight_off_waiting_for_off_fading_out = false;
			priv->backlight_off_start_off_saftey_timer = false;

#ifdef RT4539_DISPLAY_BACKLIGHT_TIME_LATENCY_DEBUG
			if (priv->screen_off_backlight_off_start) {
				s64 fading_out_time_us_total, screen_off_backlight_off_delay_total;
				s64 hwcomposer_disabled_backlight_off_delay_total;
				u64 fading_out_time_us_delta;

				fading_out_time_us_total = ktime_us_delta(
						priv->screen_off_backlight_off_delay_start, priv->screen_off_backlight_off_start);
				fading_out_time_us_delta = abs(fading_out_time_us_total -
						priv->screen_off_backlight_off_expected_fading_out_time);

				screen_off_backlight_off_delay_total = ktime_us_delta(
						priv->screen_off_backlight_off_delay_end, priv->screen_off_backlight_off_delay_start);

				if (priv->hwcomposer_disabled_backlight_off_delay_start)
					hwcomposer_disabled_backlight_off_delay_total = ktime_us_delta(
							priv->screen_off_backlight_off_delay_end, priv->hwcomposer_disabled_backlight_off_delay_start);
				else
					hwcomposer_disabled_backlight_off_delay_total = 0;

				dev_alert(priv->dev, "[time] fading-out start -> delay start: %lld us (<->%lld) (latency %llu)",
						fading_out_time_us_total,
						priv->screen_off_backlight_off_expected_fading_out_time,
						fading_out_time_us_delta);
				if (hwcomposer_disabled_backlight_off_delay_total)
					dev_alert(priv->dev, "[time] hwcomposer-disabled start -> delay end: %lld us (<->%u)",
							hwcomposer_disabled_backlight_off_delay_total,
							priv->hwcomposer_disabled_backlight_off_delay_us);

				dev_alert(priv->dev, "[time] delay start -> delay end: %lld us (<->%u)",
						screen_off_backlight_off_delay_total,
						priv->screen_off_backlight_off_extra_latency_us);
				dev_alert(priv->dev, "[time] delay start -> end: %lld us", ktime_us_delta(
						priv->screen_off_backlight_off_end, priv->screen_off_backlight_off_delay_start));
				dev_alert(priv->dev, "[time] total latency: %llu us (<->%u)",
						fading_out_time_us_delta + screen_off_backlight_off_delay_total,
						priv->screen_off_backlight_off_extra_latency_us);

				priv->screen_off_backlight_off_start = 0;
			}
#endif
#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
			if (priv->hw_enabled)
				dev_alert(priv->dev, "display off; backlight still on");
			else
				dev_alert(priv->dev, "display off; backlight off");
#endif
		}
	}

	// dev_info(priv->dev, "-");

	return 0;
}

static void register_disp_notifier(struct rt4539_led *priv)
{
	int ret;

	priv->disp_notifier.notifier_call = disp_notifier_callback;
	priv->disp_notifier.priority = INT_MAX;
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
	.cache_type = REGCACHE_NONE,
};

static int rt4539_pwm_brightness_set(struct rt4539_led *priv, unsigned int max_brightness, int brightness)
{
	int ret = 0;

	if (brightness != priv->curr_pwm_brightness) {
		unsigned long long duty = 0;

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
		else {
			dev_info(priv->dev, "pwm duty: %llu (%d->%d)", duty, priv->curr_pwm_brightness, brightness);
			priv->curr_pwm_brightness = brightness;
		}
	}

	return ret;
}

static int rt4539_i2c_brightness_set(struct rt4539_led *priv, int brightness)
{
	int ret = 0;

	if (brightness != priv->curr_i2c_brightness) {
		u8 val[2];

		val[0] = (brightness & RT4539_BRIGHTNESS_MSB_MASK) >> RT4539_BRIGHTNESS_MSB_SHIFT;
		val[1] = brightness & RT4539_BRIGHTNESS_LSB_MASK;

		ret = regmap_raw_write(priv->regmap, RT4539_REG_MSB_BRIGHTNESS, val, sizeof(val));
		if (ret)
			dev_err(priv->dev, "write i2c brightness (%d) error %d", brightness, ret);
		else {
			// dev_info(priv->dev, "write i2c brightness: v[0]: 0x%02x v[1]: 0x%02x", val[0], val[1]);
			dev_info(priv->dev, "%d->%d", priv->curr_i2c_brightness, brightness);
			priv->curr_i2c_brightness = brightness;
		}
	}

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
		if (priv->soft_start_ctrl < 0)
			priv->soft_start_ctrl = RT4539_REG_08_DEFAULT;

		mask &= RT4539_REG_08_MASK;
		priv->soft_start_ctrl &= ~mask;
		priv->soft_start_ctrl |= val & mask;
		// dev_info(priv->dev, "change reg 0x08 to 0x%x", priv->soft_start_ctrl);
	} else if (reg == RT4539_REG_09) {
		if (priv->clk_pfm_ctrl < 0)
			priv->clk_pfm_ctrl = RT4539_REG_09_DEFAULT;

		mask &= RT4539_REG_09_MASK;
		priv->clk_pfm_ctrl &= ~mask;
		priv->clk_pfm_ctrl |= val & mask;
		// dev_info(priv->dev, "change reg 0x09 to 0x%x", priv->clk_pfm_ctrl);
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

static ssize_t screen_off_backlight_off_time_ctrl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));

	if (priv->screen_off_backlight_off_time_ctrl < 0)
		return sprintf(buf, "%d\n", priv->screen_off_backlight_off_time_ctrl);
	else
		return sprintf(buf, "0x%02x\n", priv->screen_off_backlight_off_time_ctrl);
}

static ssize_t screen_off_backlight_off_time_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	int val = 0;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if (val < 0)
		priv->screen_off_backlight_off_time_ctrl = -1;
	else {
		/* Refer to kstrtou8() */
		if (val != (u8)val)
			return -ERANGE;

		priv->screen_off_backlight_off_time_ctrl = val;
	}

	return size;
}

static ssize_t can_not_see_backlight_brightness_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", priv->can_not_see_backlight_brightness_threshold);
}

static ssize_t can_not_see_backlight_brightness_threshold_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->can_not_see_backlight_brightness_threshold = val;

	priv->can_not_see_backlight_brightness_threshold_hw = brightness_maptolevel(&priv->m_led.conf,
			priv->can_not_see_backlight_brightness_threshold);

	return size;
}

static ssize_t can_not_see_backlight_brightness_threshold_hw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", priv->can_not_see_backlight_brightness_threshold_hw);
}

static ssize_t screen_off_backlight_off_safety_delay_us_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", priv->screen_off_backlight_off_safety_delay_us);
}

static ssize_t screen_off_backlight_off_safety_delay_us_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->screen_off_backlight_off_safety_delay_us = val;

	return size;
}

static ssize_t screen_off_backlight_off_extra_latency_us_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", priv->screen_off_backlight_off_extra_latency_us);
}

static ssize_t screen_off_backlight_off_extra_latency_us_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->screen_off_backlight_off_extra_latency_us = val;

	return size;
}

static ssize_t hwcomposer_disabled_backlight_off_delay_us_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", priv->hwcomposer_disabled_backlight_off_delay_us);
}

static ssize_t hwcomposer_disabled_backlight_off_delay_us_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->hwcomposer_disabled_backlight_off_delay_us = val;

	return size;
}

static ssize_t hwcomposer_disabled_backlight_off_bypass_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%d\n", priv->hwcomposer_disabled_backlight_off_bypass);
}

static ssize_t hwcomposer_disabled_backlight_off_bypass_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->hwcomposer_disabled_backlight_off_bypass = val ? true : false;

	return size;
}

static ssize_t display_on_backlight_on_delay_us_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", priv->display_on_backlight_on_delay_us);
}

static ssize_t display_on_backlight_on_delay_us_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct rt4539_led *priv = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->display_on_backlight_on_delay_us = val;

	return size;
}

static DEVICE_ATTR_RW(registers);
static DEVICE_ATTR_WO(i2c_brightness);
static DEVICE_ATTR_RW(screen_off_backlight_off_time_ctrl);
static DEVICE_ATTR_RW(can_not_see_backlight_brightness_threshold);
static DEVICE_ATTR_RO(can_not_see_backlight_brightness_threshold_hw);
static DEVICE_ATTR_RW(screen_off_backlight_off_safety_delay_us);
static DEVICE_ATTR_RW(screen_off_backlight_off_extra_latency_us);
static DEVICE_ATTR_RW(hwcomposer_disabled_backlight_off_delay_us);
static DEVICE_ATTR_RW(hwcomposer_disabled_backlight_off_bypass);
static DEVICE_ATTR_RW(display_on_backlight_on_delay_us);

static struct attribute *rt4539_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_i2c_brightness.attr,
	&dev_attr_screen_off_backlight_off_time_ctrl.attr,
	&dev_attr_can_not_see_backlight_brightness_threshold.attr,
	&dev_attr_can_not_see_backlight_brightness_threshold_hw.attr,
	&dev_attr_screen_off_backlight_off_safety_delay_us.attr,
	&dev_attr_screen_off_backlight_off_extra_latency_us.attr,
	&dev_attr_hwcomposer_disabled_backlight_off_delay_us.attr,
	&dev_attr_hwcomposer_disabled_backlight_off_bypass.attr,
	&dev_attr_display_on_backlight_on_delay_us.attr,
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
	if (priv->soft_start_ctrl >= 0) {
		ret = regmap_update_bits(priv->regmap, RT4539_REG_08, RT4539_REG_08_MASK, priv->soft_start_ctrl);
		if (ret) {
			dev_err(priv->dev, "regmap write reg 0x08 (0x%x) failed %d", priv->soft_start_ctrl, ret);
			return ret;
		}
		// dev_info(priv->dev, "reg 0x08: 0x%x", priv->soft_start_ctrl);
	}

	// reg 0x09
	if (priv->clk_pfm_ctrl >= 0) {
		ret = regmap_update_bits(priv->regmap, RT4539_REG_09, RT4539_REG_09_MASK, priv->clk_pfm_ctrl);
		if (ret) {
			dev_err(priv->dev, "regmap write reg 0x09 (0x%x) failed %d", priv->clk_pfm_ctrl, ret);
			return ret;
		}
		// dev_info(priv->dev, "reg 0x09: 0x%x", priv->clk_pfm_ctrl);
	}

	// dev_info(priv->dev, "rt4539 init end ---");

	return ret;
}

static int rt4539_kthread(void *data)
{
	struct rt4539_led *priv = (struct rt4539_led *)data;
	unsigned long flags;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	int ret;

	sched_setscheduler(current, SCHED_FIFO, &param);

	do {
		ret = wait_event_interruptible(priv->wait_queue,
				test_bit(RT4539_EVENT_BL_ON, &priv->event) ||
				test_bit(RT4539_EVENT_BL_OFF, &priv->event) ||
				test_bit(RT4539_EVENT_BL_FORCE_DISABLE, &priv->event) ||
				test_bit(RT4539_EVENT_STOP, &priv->event));

		if (ret == -ERESTARTSYS)
			continue;

		spin_lock_irqsave(&priv->lock, flags);

		if (test_bit(RT4539_EVENT_BL_ON, &priv->event)) {
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
			dev_info(priv->dev, "RT4539_EVENT_BL_ON+");
#endif
			clear_bit(RT4539_EVENT_BL_ON, &priv->event);

			rt4539_enable(priv, priv->on_off_reason ? priv->on_off_reason : "UNKNOWN");

			spin_unlock_irqrestore(&priv->lock, flags);

			rt4539_mdelay(priv->enable_post_delay_ms);

			ret = rt4539_init(priv);
			if (ret)
				dev_notice(priv->dev, "rt4539 init failed (%d)", ret);

			complete_all(&priv->backlight_on_complete);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
			dev_info(priv->dev, "RT4539_EVENT_BL_ON-");
#endif
		} else if (test_bit(RT4539_EVENT_BL_OFF, &priv->event)) {
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
			dev_info(priv->dev, "RT4539_EVENT_BL_OFF+");
#endif
			clear_bit(RT4539_EVENT_BL_OFF, &priv->event);

			//if (!priv->enable && priv->hw_enabled) { /* check for the off-to-on case */
			if (!priv->enable && priv->hw_enabled && !test_bit(RT4539_EVENT_BL_ON, &priv->event)) { /* check for the off-to-on case */
				spin_unlock_irqrestore(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_TIME_LATENCY_DEBUG
				priv->screen_off_backlight_off_delay_end = ktime_get();
#endif
				rt4539_immediately_backlight_off(priv, RT4539_REG_06_SLOPE_LIGHT_128ms, true,
						priv->on_off_reason ? priv->on_off_reason : "UNKNOWN");
			} else
				spin_unlock_irqrestore(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
			dev_info(priv->dev, "RT4539_EVENT_BL_OFF-");
#endif
		} else if (test_bit(RT4539_EVENT_BL_FORCE_DISABLE, &priv->event)) {
			clear_bit(RT4539_EVENT_BL_FORCE_DISABLE, &priv->event);
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
			dev_alert(priv->dev, "RT4539_EVENT_BL_FORCE_DISABLE+");
#endif

			rt4539_disable(priv, priv->on_off_reason ? priv->on_off_reason : "UNKNOWN");
			spin_unlock_irqrestore(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
			dev_alert(priv->dev, "RT4539_EVENT_BL_FORCE_DISABLE-");
#endif
		} else if (test_bit(RT4539_EVENT_STOP, &priv->event)) {
			clear_bit(RT4539_EVENT_STOP, &priv->event);
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
			dev_alert(priv->dev, "RT4539_EVENT_STOP+");
#endif

			rt4539_disable(priv, "EVENT-STOP");

			spin_unlock_irqrestore(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
			dev_alert(priv->dev, "RT4539_EVENT_STOP-");
#endif
		} else {
			// dev_info(priv->dev, "unknown event 0x%x", priv->event);
			spin_unlock_irqrestore(&priv->lock, flags);
		}
	} while (!kthread_should_stop());

	return 0;
}

static enum hrtimer_restart rt4539_fading_out_timer_func(struct hrtimer *timer)
{
	struct rt4539_led *priv = container_of(timer, struct rt4539_led, timer_off_fading_out);

#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	dev_info(priv->dev, "+");
#endif

	complete_all(&priv->backlight_off_fading_out_complete);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "-");
#endif

	return HRTIMER_NORESTART;
}

static void rt4539_start_off_fading_out_timer(struct rt4539_led *priv, unsigned int us)
{
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	dev_info(priv->dev, "+(%u)", us);
#endif

	if (us == 0) {
		hrtimer_cancel(&priv->timer_off_fading_out);
		complete_all(&priv->backlight_off_fading_out_complete);
	} else {
		reinit_completion(&priv->backlight_off_fading_out_complete);
		hrtimer_start(&priv->timer_off_fading_out, ns_to_ktime(us * NSEC_PER_USEC), HRTIMER_MODE_REL);
	}

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "-(%u)", us);
#endif
}

static void rt4539_cancel_off_fading_out_timer(struct rt4539_led *priv)
{
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	dev_info(priv->dev, "+");
#endif

	priv->backlight_off_waiting_for_off_fading_out = false;
	priv->backlight_off_start_off_saftey_timer = false;

	hrtimer_cancel(&priv->timer_off_fading_out);
	complete_all(&priv->backlight_off_fading_out_complete);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "-");
#endif
}

static void rt4539_wait_for_off_fading_out(struct rt4539_led *priv)
{
	int ret;
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	ktime_t start;
#endif

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "+");
#endif

#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	start = ktime_get();
#endif

	ret = wait_for_completion_timeout(&priv->backlight_off_fading_out_complete,
			msecs_to_jiffies(RT4539_BACKLIGHT_OFF_FADING_OUT_MAX_TIMEOUT_MS));
	if (ret == 0)
		dev_alert(priv->dev, "wait for off fading-out timedout");

#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	dev_alert(priv->dev, "[time] wait for off fading-out: %lld us", ktime_us_delta(ktime_get(), start));
#endif
}

static enum hrtimer_restart rt4539_off_timer_func(struct hrtimer *timer)
{
	struct rt4539_led *priv = container_of(timer, struct rt4539_led, timer_off);
	unsigned long flags;

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "+");
#endif

	spin_lock_irqsave(&priv->lock, flags);

	//if (!priv->enable && priv->hw_enabled) { /* check for the off-to-on case */
	if (!priv->enable && priv->hw_enabled && !test_bit(RT4539_EVENT_BL_ON, &priv->event)) { /* check for the off-to-on case */
		set_bit(RT4539_EVENT_BL_OFF, &priv->event);
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
		dev_info(priv->dev, "[%s] RT4539_EVENT_BL_OFF", priv->on_off_reason ? priv->on_off_reason : "UNKNOWN");
#endif
		wake_up(&priv->wait_queue);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "-");
#endif

	return HRTIMER_NORESTART;
}

static void rt4539_start_off_timer(struct rt4539_led *priv, const char* reason, unsigned int us)
{
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	dev_info(priv->dev, "+(%u): %s", us, reason);
#endif

	if (us == 0) {
		hrtimer_cancel(&priv->timer_off);
		priv->on_off_reason = reason;
		rt4539_off_timer_func(&priv->timer_off);
	} else {
		priv->on_off_reason = reason;
		hrtimer_start(&priv->timer_off, ns_to_ktime(us * NSEC_PER_USEC), HRTIMER_MODE_REL);
	}

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "-(%u)", ms);
#endif
}

static void rt4539_cancel_off_timer(struct rt4539_led *priv)
{
	unsigned long flags;

#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	dev_info(priv->dev, "+");
#endif

	hrtimer_cancel(&priv->timer_off);

	spin_lock_irqsave(&priv->lock, flags);

	priv->backlight_off_waiting_for_off_fading_out = false;
	priv->backlight_off_start_off_saftey_timer = false;

	clear_bit(RT4539_EVENT_BL_OFF, &priv->event);
	clear_bit(RT4539_EVENT_BL_FORCE_DISABLE, &priv->event);

	spin_unlock_irqrestore(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "-");
#endif
}

static void rt4539_start_on(struct rt4539_led *priv, const char* reason)
{
	unsigned long flags;

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "+: %s", reason);
#endif

	rt4539_cancel_off_timer(priv);
	rt4539_cancel_off_fading_out_timer(priv);

	spin_lock_irqsave(&priv->lock, flags);

	if (!priv->hw_enabled && !test_bit(RT4539_EVENT_BL_ON, &priv->event)) {
		reinit_completion(&priv->backlight_on_complete);
		priv->on_off_reason = reason;
		set_bit(RT4539_EVENT_BL_ON, &priv->event);
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
		dev_info(priv->dev, "[%s] RT4539_EVENT_BL_ON", reason ? reason : "");
#endif
		wake_up(&priv->wait_queue);
	} else if (!test_bit(RT4539_EVENT_BL_ON, &priv->event)) /* not queued */
		complete_all(&priv->backlight_on_complete);

	spin_unlock_irqrestore(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "-");
#endif
}

static void rt4539_wait_for_on(struct rt4539_led *priv)
{
	int ret;
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	ktime_t start;
#endif

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "+");
#endif

#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	start = ktime_get();
#endif

	ret = wait_for_completion_timeout(&priv->backlight_on_complete,
			msecs_to_jiffies(RT4539_BACKLIGHT_ON_MAX_TIMEOUT_MS));
	if (ret == 0)
		dev_alert(priv->dev, "wait for 'on' timedout");

#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	dev_alert(priv->dev, "[time] wait for 'on': %lld us", ktime_us_delta(ktime_get(), start));
#endif
}

static void rt4539_brightness_force_off(struct mt_led_data *mdev)
{
	struct rt4539_led *priv = container_of(mdev, struct rt4539_led, m_led);
	bool emergency_off = false;
	unsigned long flags;

#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
	dev_info(priv->dev, "+");
#endif

	if (priv->hwcomposer_disabled_backlight_off_bypass) {
		/* for safety timer debugging */
		static DEFINE_RATELIMIT_STATE(_rs, (1 * HZ), 1);

		if (rt4539_ratelimit(&_rs))
			dev_alert(priv->dev, "bypassed");

		return;
	}

	spin_lock_irqsave(&priv->lock, flags);

	if (!priv->enable && priv->hw_enabled && !test_bit(RT4539_EVENT_BL_ON, &priv->event)) { /* check for the off-to-on case */
		if (priv->hwcomposer_disabled_backlight_force_off) {
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
			dev_info(priv->dev, "was hwcomposer disabled backlight off");
#endif
			spin_unlock_irqrestore(&priv->lock, flags);
			return;
		} else
			priv->hwcomposer_disabled_backlight_force_off = true;

		spin_unlock_irqrestore(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_TIME_LATENCY_DEBUG
		priv->hwcomposer_disabled_backlight_off_delay_start = ktime_get();
#endif

		/* The safety off timer may be not started in stress testing */
		if (!hrtimer_active(&priv->timer_off))
			emergency_off = true;
		rt4539_cancel_off_timer(priv);

		/* The fading-out timer may be not expired in stress testing */
		if (!completion_done(&priv->backlight_off_fading_out_complete)) {
			rt4539_cancel_off_fading_out_timer(priv);
			emergency_off = true;
		}

		if (emergency_off) {
			spin_lock_irqsave(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_TIME_LATENCY_DEBUG
			priv->screen_off_backlight_off_delay_start = ktime_get();
#endif
			priv->on_off_reason = "HWC disabled backlight emerg off";
			set_bit(RT4539_EVENT_BL_FORCE_DISABLE, &priv->event);
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
			dev_info(priv->dev, "[%s] RT4539_EVENT_BL_FORCE_DISABLE", priv->on_off_reason);
#endif
			wake_up(&priv->wait_queue);

			spin_unlock_irqrestore(&priv->lock, flags);
		} else if (priv->hwcomposer_disabled_backlight_off_delay_us) {
			rt4539_start_off_timer(priv, "HWC disabled backlight delay off",
					priv->hwcomposer_disabled_backlight_off_delay_us); /* us */
		} else {
			spin_lock_irqsave(&priv->lock, flags);

			priv->on_off_reason = "HWC disabled backlight off";
			set_bit(RT4539_EVENT_BL_OFF, &priv->event);
#ifdef RT4539_DISPLAY_BACKLIGHT_DEBUG
			dev_info(priv->dev, "[%s] RT4539_EVENT_BL_OFF", priv->on_off_reason);
#endif
			wake_up(&priv->wait_queue);

			spin_unlock_irqrestore(&priv->lock, flags);
		}
	} else
		spin_unlock_irqrestore(&priv->lock, flags);

#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
	dev_info(priv->dev, "-");
#endif
}

static int rt4539_brightness_set(struct mt_led_data *mdev, int brightness)
{
	struct rt4539_led *priv = container_of(mdev, struct rt4539_led, m_led);
	int ret = 0;

	// dev_info(priv->dev, "%d->%d", priv->curr_hw_brightness, brightness);

	if (brightness > priv->hw_brightness_on_threshold) {
		/* screen on backlight on */
		bool off_to_on;

		if (!priv->enable) { /* the off-to-on case */
			priv->enable = true;

			priv->hwcomposer_disabled_backlight_force_off = false;
			rt4539_start_on(priv, "off-to-on");
			rt4539_wait_for_on(priv);

			if (priv->display_early_on_enable_backlight) {
				/* more delay to prevent a very short random backlight blink */
				priv->display_early_on_enable_backlight = false;
				rt4539_udelay(priv->display_on_backlight_on_delay_us);
			}

			off_to_on = true;
		} else
			off_to_on = false;

		if (priv->control != RT4539_CONTROL_I2C /*&& priv->pwm*/)
			// set pwm brightness
			rt4539_pwm_brightness_set(priv, mdev->conf.max_hw_brightness, brightness);
		else
			// set i2c brightness
			rt4539_i2c_brightness_set(priv, brightness);

		if (off_to_on) {
			ret = regmap_update_bits(priv->regmap, RT4539_REG_EN, RT4539_EN_MASK, RT4539_EN_MASK);
			if (ret)
				dev_notice(priv->dev, "set BL_EN failed: %d", ret);

			dev_alert(priv->dev, "on");
		}
	} else if (priv->enable) {
		priv->enable = false;

		rt4539_cancel_off_timer(priv);

		if (brightness > 0 || (priv->m_led.conf.cdev.flags & LED_SYSFS_DISABLE) || priv->display_off) {
			/* display already off, LED_SYSFS_DISABLE or screen on backlight off */
			if (priv->hw_enabled) {
				if ((priv->m_led.conf.cdev.flags & LED_SYSFS_DISABLE) || priv->display_off)
					rt4539_immediately_backlight_off(priv, RT4539_REG_06_SLOPE_LIGHT_8ms, true, "immediate");
				else
					rt4539_immediately_backlight_off(priv, RT4539_REG_06_SLOPE_LIGHT_8ms, true, "screen on backlight off");
			}
		} else {
			/* screen off backlight off */
			int time_ctrl = (priv->screen_off_backlight_off_time_ctrl >= 0) ?
					priv->screen_off_backlight_off_time_ctrl :
					(priv->time_ctrl >= 0) ? priv->time_ctrl : RT4539_REG_06_DEFAULT;
			int new_time_ctrl = time_ctrl;
			unsigned int slop_time, fade_time, fade_total_time; /* us */

			slop_time = rt4539_get_slop_time_ms(new_time_ctrl) * USEC_PER_MSEC; /* us */
			fade_time = fade_time_us[(new_time_ctrl & RT4539_REG_06_FADE_TIME_MASK) >> RT4539_REG_06_FADE_TIME_SHIFT];
			if (priv->curr_hw_brightness > priv->can_not_see_backlight_brightness_threshold_hw) {
				/* priv->curr_hw_brightness -> priv->can_not_see_backlight_brightness_threshold_hw */
				fade_total_time = fade_time *
						(priv->curr_hw_brightness - priv->can_not_see_backlight_brightness_threshold_hw);
			} else
				fade_total_time = 0;

			if (fade_total_time >= slop_time || (new_time_ctrl & RT4539_REG_06_SLOPE_FILTER_MASK) == 0) {
				/* Disable Slop Time and Using Fade Out Time */
				new_time_ctrl = new_time_ctrl & ~(RT4539_REG_06_SLOPE_TIME_MASK | RT4539_REG_06_SLOPE_FILTER_MASK);
				slop_time = 0; /* No Slop Time */
#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
				dev_alert(priv->dev, "new time_ctrl: 0x%x, slop %d us, fade %d us (%d)",
						new_time_ctrl, slop_time, fade_total_time, fade_time);
#endif
			} else {
#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
				dev_alert(priv->dev, "time_ctrl: 0x%x, slop %d us, fade %d us (%d)",
						new_time_ctrl, slop_time, fade_total_time, fade_time);
#endif
			}

			/* Calculate the waiting time for fading-out backlight off */
			fade_total_time = 0;
			if (priv->curr_hw_brightness > priv->hw_brightness_on_threshold &&
					priv->curr_hw_brightness > priv->can_not_see_backlight_brightness_threshold_hw) {
				/* priv->curr_hw_brightness -> priv->can_not_see_backlight_brightness_threshold_hw */
				unsigned int fading_out_hw_brightness_threshold;

				if (slop_time) {
					/* Using the slop time: New fade-out time per step */
					fade_time = slop_time / (priv->curr_hw_brightness - priv->can_not_see_backlight_brightness_threshold_hw);
					if (fade_time == 0)
						fade_time = 1; /* us */
				}

				fading_out_hw_brightness_threshold = rt4539_calc_fading_out_hw_brightness_threshold(priv, fade_time);

				if (priv->curr_hw_brightness > fading_out_hw_brightness_threshold) {
					fade_total_time = fade_time * (priv->curr_hw_brightness - fading_out_hw_brightness_threshold);
#ifdef RT4539_DISPLAY_BACKLIGHT_VERBOSE_DEBUG
					if (slop_time)
						dev_alert(priv->dev, "wait for sloping time: %u us (%u)", fade_total_time, fade_time);
					else
						dev_alert(priv->dev, "wait for fading time: %u us (%u)", fade_total_time, fade_time);
#endif
				}

				if (new_time_ctrl != time_ctrl || priv->screen_off_backlight_off_time_ctrl >= 0) {
					/* Setup New Time for Backlight off */
					ret = regmap_update_bits(priv->regmap, RT4539_REG_06, RT4539_REG_06_MASK, new_time_ctrl);
					if (ret)
						dev_err(priv->dev, "regmap write new time_ctrl (0x%x) failed %d", new_time_ctrl, ret);
				}

				if (priv->control != RT4539_CONTROL_I2C /*&& priv->pwm*/)
					// set pwm brightness
					rt4539_pwm_brightness_set(priv, mdev->conf.max_hw_brightness,
							priv->can_not_see_backlight_brightness_threshold_hw);
				else
					// set i2c brightness
					rt4539_i2c_brightness_set(priv,
							priv->can_not_see_backlight_brightness_threshold_hw);
			} else {
				/* Do not change the brightness => stay at the current brightness level */
				//dev_info(priv->dev, "%d (->0)", priv->curr_hw_brightness);
			}

#ifdef RT4539_DISPLAY_BACKLIGHT_TIME_LATENCY_DEBUG
			priv->screen_off_backlight_off_expected_fading_out_time = fade_total_time;
			priv->screen_off_backlight_off_delay_start = 0;
			priv->hwcomposer_disabled_backlight_off_delay_start = 0;
			priv->screen_off_backlight_off_delay_end = 0;
			priv->screen_off_backlight_off_end = 0;
			priv->screen_off_backlight_off_start = ktime_get();
#endif

			priv->hwcomposer_disabled_backlight_force_off = false;
			priv->display_early_on_enable_backlight = true;
			priv->backlight_off_waiting_for_off_fading_out = true;
			priv->backlight_off_start_off_saftey_timer = true;

			rt4539_start_off_fading_out_timer(priv, fade_total_time); /* us */
		}
		brightness = 0;
	} else if (priv->backlight_off_waiting_for_off_fading_out && brightness == 0) {
		priv->backlight_off_waiting_for_off_fading_out = false;
		rt4539_wait_for_off_fading_out(priv);
	} else if (priv->backlight_off_start_off_saftey_timer && brightness == 0) {
		priv->backlight_off_start_off_saftey_timer = false;
		/* we may receive the force-off from DRM in stress testing */
		if (!hrtimer_active(&priv->timer_off)) {
#ifdef RT4539_DISPLAY_BACKLIGHT_TIME_LATENCY_DEBUG
			priv->screen_off_backlight_off_delay_start = ktime_get();
#endif
			rt4539_start_off_timer(priv, "screen off backlight safety off",
					priv->screen_off_backlight_off_safety_delay_us); /* us */
		}
	} else {
		brightness = 0;
	}

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
	priv->m_led.mtk_hw_brightness_force_off = rt4539_brightness_force_off;
	priv->m_led.mtk_conn_id_get = led_pwm_get_conn_id;

	priv->enable_post_delay_ms = EN_POWEN_ON_DELAY;
	fwnode_property_read_u32(fwnode, "enable-post-delay", &priv->enable_post_delay_ms);
	if (priv->enable_post_delay_ms < EN_POWEN_ON_DELAY)
		priv->enable_post_delay_ms = EN_POWEN_ON_DELAY;
	// dev_info(priv->dev, "enable-post-delay=%u", priv->enable_post_delay_ms);

	if (fwnode_property_read_u32(fwnode, "screen-off-backlight-off-safety-delay-us",
			&priv->screen_off_backlight_off_safety_delay_us) ||
				priv->screen_off_backlight_off_safety_delay_us == 0)
		priv->screen_off_backlight_off_safety_delay_us = 100 * USEC_PER_MSEC; /* 100ms */
	// dev_info(priv->dev, "screen-off-backlight-off-safety-delay-us=%u", priv->screen_off_backlight_off_safety_delay_us);

	if (fwnode_property_read_u32(fwnode, "screen-off-backlight-off-extra-latency-us",
			&priv->screen_off_backlight_off_extra_latency_us) ||
				priv->screen_off_backlight_off_extra_latency_us == 0)
		priv->screen_off_backlight_off_extra_latency_us = 20000; /* 20ms */
	// dev_info(priv->dev, "screen-off-backlight-off-extra-latency-us=%u", priv->screen_off_backlight_off_extra_latency_us);

	if (fwnode_property_read_u32(fwnode, "hwcomposer-disabled-backlight-off-delay-us",
			&priv->hwcomposer_disabled_backlight_off_delay_us))
		priv->hwcomposer_disabled_backlight_off_delay_us = 0;
	// dev_info(priv->dev, "hwcomposer-disabled-backlight-off-delay-us=%u", priv->hwcomposer_disabled_backlight_off_delay_us);

	if (fwnode_property_read_u32(fwnode, "display-on-backlight-on-delay-us",
			&priv->display_on_backlight_on_delay_us) ||
				priv->display_on_backlight_on_delay_us == 0)
		priv->display_on_backlight_on_delay_us = DIV_ROUND_UP(1000UL * USEC_PER_MSEC, 60) * 3; /* ~ 3 frames@60Hz */
	// dev_info(priv->dev, "display-on-backlight-on-delay-us=%u", priv->display_on_backlight_on_delay_us);

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
	if (fwnode_property_read_u32(fwnode, "hw-brightness-on-threshold", &priv->hw_brightness_on_threshold))
		priv->hw_brightness_on_threshold = 0;

	if (fwnode_property_read_u32(fwnode, "can-not-see-backlight-brightness-threshold", &priv->can_not_see_backlight_brightness_threshold))
		priv->can_not_see_backlight_brightness_threshold = 0;
	// dev_info(priv->dev, "can-not-see-backlight-brightness-threshold(DTS): %u", priv->can_not_see_backlight_brightness_threshold);

	// reg 0x06 for brightness off
	if (!fwnode_property_read_u32(fwnode, "screen-off-backlight-off-time-ctrl", &priv->screen_off_backlight_off_time_ctrl)) {
		if (priv->screen_off_backlight_off_time_ctrl < 0 || priv->screen_off_backlight_off_time_ctrl > 0xff) {
			dev_err(priv->dev, "time-ctrl-for-off is out of range (0-0xff)");
			priv->screen_off_backlight_off_time_ctrl = -1;
		} else
			priv->screen_off_backlight_off_time_ctrl &= RT4539_REG_06_MASK;
	} else
		priv->screen_off_backlight_off_time_ctrl = -1;
	// dev_info(priv->dev, "screen-off-backlight-off-time-ctrl: 0x%x", priv->screen_off_backlight_off_time_ctrl);

	// reg 0x08
	if (!fwnode_property_read_u32(fwnode, "soft-start-ctrl", &priv->soft_start_ctrl)) {
		if (priv->soft_start_ctrl < 0 || priv->soft_start_ctrl > 0xff) {
			dev_err(priv->dev, "soft-start-ctrl is out of range (0-0xff)");
			priv->soft_start_ctrl = -1;
		} else {
			priv->soft_start_ctrl &= RT4539_REG_08_MASK;
			// dev_info(priv->dev, "soft-start-ctrl: 0x%x", priv->soft_start_ctrl);
		}
	} else
		priv->soft_start_ctrl = -1;

	// reg 0x09
	if (!fwnode_property_read_u32(fwnode, "clk-pfm-ctrl", &priv->clk_pfm_ctrl)) {
		if (priv->clk_pfm_ctrl < 0 || priv->clk_pfm_ctrl > 0xff) {
			dev_err(priv->dev, "clk-pfm-ctrl is out of range (0-0xff)");
			priv->clk_pfm_ctrl = -1;
		} else {
			priv->clk_pfm_ctrl &= RT4539_REG_09_MASK;
			// dev_info(priv->dev, "clk-pfm-ctrl: 0x%x", priv->clk_pfm_ctrl);
		}
	} else
		priv->clk_pfm_ctrl = -1;

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

	priv->can_not_see_backlight_brightness_threshold_hw = brightness_maptolevel(&priv->m_led.conf,
			priv->can_not_see_backlight_brightness_threshold);

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
#if 0
	priv->curr_hw_brightness = 0;
	priv->curr_pwm_brightness = 0;
	priv->curr_i2c_brightness = 0;
#endif

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

	spin_lock_init(&priv->lock);
	//priv->event = 0;
	init_waitqueue_head(&priv->wait_queue);
	init_completion(&priv->backlight_on_complete);
	init_completion(&priv->backlight_off_fading_out_complete);
	priv->kthread = kthread_create(rt4539_kthread, priv, dev_name(priv->dev));
	if (IS_ERR(priv->kthread)) {
		dev_err(priv->dev, "create thread err %ld", PTR_ERR(priv->kthread));
		ret = PTR_ERR(priv->kthread);
		priv->kthread = NULL;
		goto err;
	}
	wake_up_process(priv->kthread);

	/* fading out */
	hrtimer_init(&priv->timer_off_fading_out, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	priv->timer_off_fading_out.function = rt4539_fading_out_timer_func;

	/* disable/backlight off */
	hrtimer_init(&priv->timer_off, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	priv->timer_off.function = rt4539_off_timer_func;

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
	if (priv->kthread) {
		set_bit(RT4539_EVENT_STOP, &priv->event);
		wake_up(&priv->wait_queue);
		kthread_stop(priv->kthread);
	}
	dev_err(priv->dev, "Failed to probe: %d!", ret);
	return ret;
}

static int rt4539_remove(struct i2c_client *client)
{
	struct rt4539_led *priv = i2c_get_clientdata(client);

	/* Refer to led_sysfs_disable() */
	priv->m_led.conf.cdev.flags |= LED_SYSFS_DISABLE;

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	unregister_disp_notifier(priv);
#endif

	if (priv->hw_enabled) {
		rt4539_cancel_off_timer(priv);
		priv->enable = false;
		rt4539_immediately_backlight_off(priv, RT4539_REG_06_SLOPE_LIGHT_0ms, true, "rmmod");
	}

	if (priv->kthread) {
		set_bit(RT4539_EVENT_STOP, &priv->event);
		wake_up(&priv->wait_queue);
		kthread_stop(priv->kthread);
	}

	mt_leds_classdev_unregister(priv->dev, &priv->m_led);

	sysfs_remove_group(&client->dev.kobj, &rt4539_attr_group);

	return 0;
}

static void rt4539_shutdown(struct i2c_client *client)
{
	struct rt4539_led *priv = i2c_get_clientdata(client);

	/* Refer to led_sysfs_disable() */
	priv->m_led.conf.cdev.flags |= LED_SYSFS_DISABLE;

	if (priv->hw_enabled) {
		rt4539_cancel_off_timer(priv);
		priv->enable = false;
		rt4539_immediately_backlight_off(priv, RT4539_REG_06_SLOPE_LIGHT_0ms, true, "shutdown");
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

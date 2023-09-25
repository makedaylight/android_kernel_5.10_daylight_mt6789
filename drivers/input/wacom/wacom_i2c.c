/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2015 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

#include "wacom.h"

/* WACOM_PHYSICAL_ORIENTATION (0) is the default orientation of the origin (0,0) in WACOM FW */
/* 0/90/180/270 */
// #define WACOM_PHYSICAL_ORIENTATION 0
#ifdef WACOM_PHYSICAL_ORIENTATION
  #if (WACOM_PHYSICAL_ORIENTATION == 90)
    #define WACOM_SWAP_XY       1
    #define WACOM_INVERT_X      1
    #define WACOM_INVERT_Y      0
  #elif (WACOM_PHYSICAL_ORIENTATION == 180)
    #define WACOM_SWAP_XY       0
    #define WACOM_INVERT_X      1
    #define WACOM_INVERT_Y      1
  #elif (WACOM_PHYSICAL_ORIENTATION == 270)
    #define WACOM_SWAP_XY       1
    #define WACOM_INVERT_X      0
    #define WACOM_INVERT_Y      1
  #else
    #define WACOM_SWAP_XY       0
    #define WACOM_INVERT_X      0
    #define WACOM_INVERT_Y      0
  #endif
#else
  #define WACOM_SWAP_XY         0
  #define WACOM_INVERT_X        0
  #define WACOM_INVERT_Y        0
#endif

/* Resolutions */
#define XY_RESOLUTION       100   /* Distance : SI Linear Unit with exponent -3 */
#define DIST_RESOLUTION     10    /* Distance : SI Linear Unit with exponent -2. This covers 'Z' resolution too */
#define TILT_RESOLUTION     5730  /* Degrees : English Rotation  with exponent -2 */

static void wacom_delay(long long int ms) {
	if (ms < 0)
		usleep_range(1000, 1001);
	else if(ms < 20)
		usleep_range((ms + 1) * 1000, (ms + 1) * 1001);
	else
		msleep(ms);
}


static int wacom_query_device_direct(struct i2c_client *client,
			      struct wacom_features *features)
{
	int ret;
	u8 cmd_hid_desc[] = {HID_DESC_REG, 0x00};
	u8 cmd1[] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1,
			WACOM_CMD_QUERY2, WACOM_CMD_QUERY3,
			WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
	u8 data[WACOM_QUERY_SIZE];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};

	dev_dbg(&client->dev, "+%s()\n", __func__);

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev, "%s(): i2c_transfer addr %d, error %d\n", __func__, client->addr, ret);
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

#ifdef DEBUG_V
	for (ret = 0; ret < WACOM_QUERY_SIZE; ret++)
		dev_dbg(&client->dev ,"data%d: 0x%x \n", ret, data[ret]);
#endif

	features->x_max = get_unaligned_le16(&data[3]);
	features->y_max = get_unaligned_le16(&data[5]);
	features->pressure_max = get_unaligned_le16(&data[11]);
	features->fw_version = get_unaligned_le16(&data[13]);
	features->height_max = data[15];
	features->tilt_x_max = get_unaligned_le16(&data[17]);
	features->tilt_y_max =  get_unaligned_le16(&data[19]);

	if (features->height_max)
		features->support.height = true;
	else
		features->support.height = false;

	if (features->tilt_x_max && features->tilt_y_max)
		features->support.tilt = true;
	else
		features->support.tilt = false;

	ret = i2c_master_send(client, cmd_hid_desc, sizeof(cmd_hid_desc));
	if (ret < 0) {
		dev_err(&client->dev, "cannot send register info\n");
		return ret;
	}

	ret = i2c_master_recv(client, (char *)&features->hid_desc, sizeof(HID_DESC));
	if (ret < 0) {
		dev_err(&client->dev, "cannot receive register info\n");
		return ret;
	}

	dev_dbg(&client->dev, "input report length %d \n", features->hid_desc.wMaxInputLength);
	dev_dbg(&client->dev, "x_max:%d, y_max:%d, pressure:%d, fw:%d\n",
		features->x_max, features->y_max,
		features->pressure_max, features->fw_version);
	dev_dbg(&client->dev, "height:%d, tilt_x:%d, tilt_y:%d\n",
		features->height_max, features->tilt_x_max,
		features->tilt_y_max);

	return 0;
}

#if 0
/*Eliminating the digitizer AA offset; this makes the coordination exactly fit the LCD size*/
/*However, on the other hand, the difference in rotation of the screens against digitizers */
/*must be manually considered*/
static void set_offset(int *x, int *y, int x_max, int y_max)
{
	int temp_coord = *x - AA_OFFSET;

	if (temp_coord < 0)
		*x = 0;
	else if (temp_coord > x_max)
		*x = x_max;
	else
		*x = temp_coord;

	temp_coord = *y - AA_OFFSET;

	if (temp_coord < 0)
		*y = 0;
	else if (temp_coord > y_max)
		*y = y_max;
	else
		*y = temp_coord;
}
#endif


#ifdef INOCO_CPU_LATENCY_REQUEST
static inline void wacom_cpu_latency_request_nolock(struct wacom_i2c *wac_i2c, s32 new_value)
{
	dev_pm_qos_update_request(wac_i2c->cpu_dev->power.qos->resume_latency_req,
			new_value);
#if 0
	cpu_latency_qos_update_request(&wac_i2c->pm_qos_request,
			new_value != PM_QOS_RESUME_LATENCY_NO_CONSTRAINT ?
				new_value : PM_QOS_DEFAULT_VALUE);
#endif
}

static void wacom_cpu_latency_update_nolock(struct wacom_i2c *wac_i2c)
{
	if (!wac_i2c->cpu_dev)
		return;

	if (wac_i2c->fb_blank_powerdown) {
		wac_i2c->last_touched_time = 0;
		cancel_delayed_work(&wac_i2c->cpu_latency_check_work);

#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
		dev_info(&wac_i2c->client->dev, "wacom_cpu_latency_update: latency released (was suspend)\n");
#endif
		wacom_cpu_latency_request_nolock(wac_i2c, PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
		//cpu_latency_qos_update_request(&wac_i2c->pm_qos_request, PM_QOS_DEFAULT_VALUE);
	} else {
		wac_i2c->last_touched_time = 0;
		cancel_delayed_work(&wac_i2c->cpu_latency_check_work);

		if (wac_i2c->cpu_latency_display_on > 0) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
			dev_info(&wac_i2c->client->dev, "wacom_cpu_latency_update: latency display %d\n", wac_i2c->cpu_latency_display_on);
#endif
			wacom_cpu_latency_request_nolock(wac_i2c, wac_i2c->cpu_latency_display_on);
		} else {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
			dev_info(&wac_i2c->client->dev, "wacom_cpu_latency_update: latency released\n");
#endif
			wacom_cpu_latency_request_nolock(wac_i2c, PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
		}
	}
}

static void __maybe_unused wacom_cpu_latency_check_work(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c = container_of(to_delayed_work(work),
						struct wacom_i2c, cpu_latency_check_work);
	bool re_queue = false;

	mutex_lock(&wac_i2c->cpu_latency_mutex);

	if (wac_i2c->last_touched_time && wac_i2c->cpu_latency_check_time) {
		u64 next_check = wac_i2c->last_touched_time + wac_i2c->cpu_latency_check_time;
		u64 now = get_jiffies_64();

#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
		dev_info(&wac_i2c->client->dev, "wacom_cpu_latency_check: time %u; last_touched_time %llu; next_check %llu; now %llu\n",
				wac_i2c->cpu_latency_check_time, wac_i2c->last_touched_time, next_check, now);
#endif
		if (time_after_eq64(now, next_check)) {
			if (wac_i2c->cpu_latency_display_on > 0) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
				dev_info(&wac_i2c->client->dev, "wacom_cpu_latency_check: latency display %d\n", wac_i2c->cpu_latency_display_on);
#endif
				wacom_cpu_latency_request_nolock(wac_i2c, wac_i2c->cpu_latency_display_on);
			} else {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
				dev_info(&wac_i2c->client->dev, "wacom_cpu_latency_check: latency released\n");
#endif
				wacom_cpu_latency_request_nolock(wac_i2c, PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
			}
		} else {
			re_queue = true;
		}
	}

	mutex_unlock(&wac_i2c->cpu_latency_mutex);

	if (re_queue) {
		if (schedule_delayed_work_on(wac_i2c->affinity_hint_cpu,
				&wac_i2c->cpu_latency_check_work, wac_i2c->cpu_latency_check_time)) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
			dev_info(&wac_i2c->client->dev, "wacom_cpu_latency_check: queue work\n");
#endif
		} else {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
			dev_info(&wac_i2c->client->dev, "wacom_cpu_latency_check: work was queued\n");
#endif
		}
	}
}
#endif

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	u8 *data = wac_i2c->data;
#ifdef INOCO_CPU_LATENCY_REQUEST
	bool use_cpu_latency = false;
#endif
	unsigned int x, y, pressure;
	int tilt_x, tilt_y;
	unsigned char tsw, f1, f2, ers;
	int data_len = wac_i2c->features->hid_desc.wMaxInputLength;
	int error;
	int id = 0;
	struct i2c_msg msg = {
		.addr = wac_i2c->client->addr,
		.flags = wac_i2c->i2c_read_flags,
		.len = data_len,
		.buf = data,
	};

	dev_dbg(&wac_i2c->client->dev, "+%s()\n", __func__);

#ifdef INOCO_CPU_LATENCY_REQUEST
	mutex_lock(&wac_i2c->cpu_latency_mutex);

	if (wac_i2c->cpu_latency_touched > 0 &&
			(wac_i2c->cpu_latency_display_on == 0 || wac_i2c->cpu_latency_touched < wac_i2c->cpu_latency_display_on)) {
		use_cpu_latency = true;

		if (wac_i2c->cpu_latency_touched < dev_pm_qos_requested_resume_latency(wac_i2c->cpu_dev)) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
			dev_info(&wac_i2c->client->dev, "wacom_i2c_isr: latency touched %d\n", wac_i2c->cpu_latency_touched);
#endif
			wacom_cpu_latency_request_nolock(wac_i2c, wac_i2c->cpu_latency_touched);
		}
	}
#endif

#if 0
	error = i2c_master_recv(wac_i2c->client,
				data, data_len);

	if (error != data_len) {
		dev_dbg(&wac_i2c->client->dev, "%s(): data_len != %d\n", __func__, error);
		dev_dbg(&wac_i2c->client->dev, "data[%d]: %*ph\n", (data[1] << 8) | data[0], (data[1] << 8) | data[0], data);
		goto out;
	}
#else
	error = i2c_transfer(wac_i2c->client->adapter, &msg, 1);

	if (error != 1) {
		dev_dbg(&wac_i2c->client->dev, "%s(): data_len != %d\n", __func__, error);
		dev_dbg(&wac_i2c->client->dev, "data[%d]: %*ph\n", (data[1] << 8) | data[0], (data[1] << 8) | data[0], data);
		goto out;
	}
#endif

#ifdef DEBUG_V
	dev_dbg(&wac_i2c->client->dev, "data[%d]: %*ph\n", (data[1] << 8) | data[0], (data[1] << 8) | data[0], data);
#endif

	id = data[2];
	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
#if defined(WACOM_INVERT_X) && (WACOM_INVERT_X)
	x = wac_i2c->features->x_max - x /*+ x_min*/;
#endif
#if defined(WACOM_INVERT_Y) && (WACOM_INVERT_Y)
	y = wac_i2c->features->y_max - y /*+ x_min*/;
#endif
	pressure = le16_to_cpup((__le16 *)&data[8]);

	//set_offset(&x, &y, wac_i2c->features.x_max, wac_i2c->features.y_max);

#ifdef CALIBRATION
	/*Set calibration values*/
	if (wac_i2c->features->bCalibrationSet && wac_i2c->features->node_state == STATE_NORMAL)
		set_calib(&x, &y, wac_i2c->features->x_max, wac_i2c->features->y_max,
			  wac_i2c->features->calib_data.originX, wac_i2c->features->calib_data.originY,
			  wac_i2c->features->calib_data.extentX, wac_i2c->features->calib_data.extentY);
#endif

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;

	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
#if defined(WACOM_SWAP_XY) && (WACOM_SWAP_XY)
	input_report_abs(input, ABS_X, y);
	input_report_abs(input, ABS_Y, x);
#else
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
#endif
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_report_abs(input, ABS_MISC, id);
#ifdef ENABLE_SERIAL
	input_event(input, EV_MSC, MSC_SERIAL, id);
#endif

	if (data_len > WACOM_REGULAR_INPUT) {
		if (wac_i2c->features->support.height)
			input_report_abs(input, ABS_DISTANCE, data[10]);

		if (wac_i2c->features->support.tilt) {
			tilt_x = (int)le16_to_cpup((__le16 *)&data[11]);
			tilt_y = (int)le16_to_cpup((__le16 *)&data[13]);
			input_report_abs(input, ABS_TILT_X, tilt_x);
			input_report_abs(input, ABS_TILT_Y, tilt_y);
		}
	}

	input_sync(input);

out:
#ifdef INOCO_CPU_LATENCY_REQUEST
	if (use_cpu_latency) {
		if (wac_i2c->cpu_latency_check_time) {
			wac_i2c->last_touched_time = get_jiffies_64();
			if (!delayed_work_pending(&wac_i2c->cpu_latency_check_work)) {
				if (schedule_delayed_work_on(wac_i2c->affinity_hint_cpu,
						&wac_i2c->cpu_latency_check_work, wac_i2c->cpu_latency_check_time)) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
					dev_info(&wac_i2c->client->dev, "wacom_i2c_isr: queue work\n");
#endif
				} else {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
					dev_info(&wac_i2c->client->dev, "wacom_i2c_isr: work was queued\n");
#endif
				}
			}
		} else if (wac_i2c->cpu_latency_display_on > 0) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
			dev_info(&wac_i2c->client->dev, "wacom_i2c_isr: latency display %d\n", wac_i2c->cpu_latency_display_on);
#endif
			wacom_cpu_latency_request_nolock(wac_i2c, wac_i2c->cpu_latency_display_on);
		} else {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
			dev_info(&wac_i2c->client->dev, "wacom_i2c_isr: latency released\n");
#endif
			wacom_cpu_latency_request_nolock(wac_i2c, PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
		}
	}

	mutex_unlock(&wac_i2c->cpu_latency_mutex);
#endif

	return IRQ_HANDLED;
}


static int wacom_i2c_open(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	enable_irq(client->irq);

	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	disable_irq(client->irq);
}

static int wacom_pinctrl_select(struct wacom_i2c *wac_i2c, bool status)
{
	struct device *dev = &wac_i2c->client->dev;
	int ret = 0;

	dev_dbg(dev, "+%s(): pinctrl select %d\n", __func__, status);

	if (status) {
		ret = pinctrl_select_state(wac_i2c->pinctrl, wac_i2c->active);
		if (ret) {
			dev_err(dev, "failed to select pinctrl, err: %d\n", ret);
			return ret;
		}
	} else {
		ret = pinctrl_select_state(wac_i2c->pinctrl, wac_i2c->sleep);
		if (ret) {
			dev_err(dev, "failed to select pinctrl, err: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int wacom_pinctrl_get(struct wacom_i2c *wac_i2c)
{
	struct device *dev = &wac_i2c->client->dev;

	wac_i2c->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(wac_i2c->pinctrl)) {
		dev_err(dev, "unable to get pinctrl, err: %ld\n", PTR_ERR(wac_i2c->pinctrl));
		return PTR_ERR(wac_i2c->pinctrl);
	}
	wac_i2c->active = pinctrl_lookup_state(wac_i2c->pinctrl, "active");
	if (IS_ERR(wac_i2c->active)) {
		dev_err(dev, "unable to lookup pinctrl state, err: %ld\n", PTR_ERR(wac_i2c->active));
		return PTR_ERR(wac_i2c->active);
	}
	wac_i2c->sleep = pinctrl_lookup_state(wac_i2c->pinctrl, "sleep");
	if (IS_ERR(wac_i2c->sleep)) {
		dev_err(dev, "unable to lookup pinctrl state, err: %ld\n", PTR_ERR(wac_i2c->sleep));
		return PTR_ERR(wac_i2c->sleep);
	}

	return 0;
}

static int wacom_regulator_control(struct wacom_i2c *wac_i2c, bool status)
{
	struct device *dev = &wac_i2c->client->dev;
	int ret = 0;

	dev_dbg(dev, "+%s(): regulator control %d\n", __func__, status);

	if (status && !wac_i2c->vdd1v8_status && !wac_i2c->vdd3v3_status) {
		ret = regulator_enable(wac_i2c->vdd1v8);
		if (ret) {
			dev_err(dev, "failed to enable regulator WACOM-1V8, err: %d\n", ret);
			return ret;
		}
		wac_i2c->vdd1v8_status = true;

		ret = regulator_enable(wac_i2c->vdd3v3);
		if (ret) {
			dev_err(dev, "failed to enable regulator WACOM-3V3, err: %d\n", ret);
			return ret;
		}
		wac_i2c->vdd3v3_status = true;

		return ret;
	} else {
		ret = regulator_disable(wac_i2c->vdd1v8);
		if (ret) {
			dev_err(dev, "failed to disable regulator WACOM-1V8, err: %d\n", ret);
			return ret;
		}
		wac_i2c->vdd1v8_status = false;

		ret = regulator_disable(wac_i2c->vdd3v3);
		if (ret) {
			dev_err(dev, "failed to disable regulator WACOM-3V3, err: %d\n", ret);
			return ret;
		}
		wac_i2c->vdd3v3_status = false;

		return ret;
	}
}

static int wacom_regulator_get(struct wacom_i2c *wac_i2c)
{
	struct device *dev = &wac_i2c->client->dev;
	int ret = 0;

	wac_i2c->vdd1v8 = devm_regulator_get(dev, "vdd");
	if (IS_ERR(wac_i2c->vdd1v8)) {
		ret = PTR_ERR(wac_i2c->vdd1v8);
		dev_err(dev, "failed to get regulator WACOM-1V8, err: %d\n", ret);
		return ret;
	}
	wac_i2c->vdd1v8_status = false;

	wac_i2c->vdd3v3 = devm_regulator_get(dev, "vddl");
	if (IS_ERR(wac_i2c->vdd3v3)) {
		ret = PTR_ERR(wac_i2c->vdd3v3);
		dev_err(dev, "failed to get regulator WACOM-3V3, err: %d\n", ret);
		return ret;
	}
	wac_i2c->vdd3v3_status = false;

	return ret;
}

static void wacom_i2c_set_sleep(struct device *dev, bool into_sleep)
{
	int ret = -1;
	char cmd[4] = {0x04, 0x00, 0x01, 0x08};
	struct i2c_client *client = to_i2c_client(dev);

	if (!into_sleep)
		cmd[2] = 0x00;

	ret = i2c_master_send(client, cmd, 4);
	if (ret != 4) {
		dev_err(dev, "%s() Failed\n", __func__);
	} else {
		dev_dbg(dev, "%s() Succeeded\n", __func__);
	}

	return;
}

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
static int disp_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	int *ev_data = (int *)data;
	struct wacom_i2c *wac_i2c = container_of(self, struct wacom_i2c, fb_notif);

	dev_dbg(&wac_i2c->client->dev, "+%s(): FB EVENT: %lu\n",__func__, event);

	if (!ev_data)
		return 0;

	if (event == MTK_DISP_EARLY_EVENT_BLANK) {
		if (*ev_data == MTK_DISP_BLANK_POWERDOWN && !wac_i2c->fb_blank_powerdown) {
			dev_dbg(&wac_i2c->client->dev, "%s(): notify suspend\n", __func__);
			wac_i2c->fb_blank_powerdown = true;
			wacom_i2c_set_sleep(&wac_i2c->client->dev, true);
			disable_irq(wac_i2c->client->irq);
			wacom_pinctrl_select(wac_i2c, false);
			gpiod_set_value(wac_i2c->reset_gpiod, 1); /* reset */
			wacom_regulator_control(wac_i2c, false);

#ifdef INOCO_CPU_LATENCY_REQUEST
			if (wac_i2c->cpu_latency_touched > 0 || wac_i2c->cpu_latency_display_on > 0) {
				wac_i2c->last_touched_time = 0; /* clear out of mutex */

				mutex_lock(&wac_i2c->cpu_latency_mutex);
				if (wac_i2c->cpu_latency_check_time) {
					wac_i2c->last_touched_time = 0; /* clear again in mutex */
					cancel_delayed_work(&wac_i2c->cpu_latency_check_work); /* no sync in mutex */
				}
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
				dev_info(&wac_i2c->client->dev, "wacom_suspend: latency released\n");
#endif
				wacom_cpu_latency_request_nolock(wac_i2c, PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
				//cpu_latency_qos_update_request(&ts->pm_qos_request, PM_QOS_DEFAULT_VALUE);
				mutex_unlock(&wac_i2c->cpu_latency_mutex);
			}
#endif
		} else if (*ev_data == MTK_DISP_BLANK_UNBLANK && wac_i2c->fb_blank_powerdown) {
			dev_dbg(&wac_i2c->client->dev, "%s(): notify resume\n", __func__);
			gpiod_set_value(wac_i2c->reset_gpiod, 1); /* reset */
			wacom_regulator_control(wac_i2c, true);
			wacom_pinctrl_select(wac_i2c, true);
			gpiod_set_value(wac_i2c->reset_gpiod, 0); /* not reset */
			wac_i2c->poweron_kt = ktime_get();
		}
	} else if (event == MTK_DISP_EVENT_BLANK && *ev_data == MTK_DISP_BLANK_UNBLANK && wac_i2c->fb_blank_powerdown) {
		if (wac_i2c->post_power_delay_ms) {
			s64 delta_ms_kt = ktime_sub(ktime_get(), wac_i2c->poweron_kt);
			s64 post_power_delay_kt = ms_to_ktime(wac_i2c->post_power_delay_ms);
			dev_dbg(&wac_i2c->client->dev, "%s() delta_ms: %lld, post_power_delay_kt: %lld\n", __func__, delta_ms_kt, post_power_delay_kt);

			if (ktime_compare(delta_ms_kt, post_power_delay_kt) >= 0) {
				dev_dbg(&wac_i2c->client->dev, "%s(): delta_ms > post_power_delay, enable irq\n", __func__);
			} else {
				s64 delay_ms_kt = ktime_ms_delta(post_power_delay_kt, delta_ms_kt);
				dev_dbg(&wac_i2c->client->dev, "%s(): post_power_delay - delta_ms_kt = %lld(ms)\n", __func__, delay_ms_kt);
				wacom_delay(delay_ms_kt);
			}
		}

#ifdef INOCO_CPU_LATENCY_REQUEST
		if (wac_i2c->cpu_latency_display_on > 0) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
			dev_info(&wac_i2c->client->dev, "wacom_resume: latency display %d\n", wac_i2c->cpu_latency_display_on);
#endif
			mutex_lock(&wac_i2c->cpu_latency_mutex);
			wacom_cpu_latency_request_nolock(wac_i2c, wac_i2c->cpu_latency_display_on);
			mutex_unlock(&wac_i2c->cpu_latency_mutex);
		}
#endif

		enable_irq(wac_i2c->client->irq);
		wac_i2c->fb_blank_powerdown = false;
	}

	return 0;
}

static int register_notifier(struct wacom_i2c *wac_i2c)
{
	int ret = 0;

	dev_dbg(&wac_i2c->client->dev, "+%s()\n", __func__);
	wac_i2c->fb_blank_powerdown = false;
	wac_i2c->fb_notif.notifier_call = disp_notifier_callback;
	ret = mtk_disp_notifier_register("wacom", &wac_i2c->fb_notif);
	if (ret)
		dev_err(&wac_i2c->client->dev, "Unable to reigster fb_notifier: %d\n", ret);

	return ret;
}

static int release_notifier(struct wacom_i2c *wac_i2c)
{
	int ret = 0;

	dev_dbg(&wac_i2c->client->dev, "+%s()\n", __func__);
	ret = mtk_disp_notifier_unregister(&wac_i2c->fb_notif);
	if (ret)
		dev_err(&wac_i2c->client->dev, "Unable to release fb_notifier: %d\n", ret);

	return ret;
}
#endif // CONFIG_DRM_MEDIATEK

static ssize_t fwversion_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "firmware version: 0x%04x", le16_to_cpu(wac_i2c->features->hid_desc.wVersion));
}

static DEVICE_ATTR_RO(fwversion);

#ifdef INOCO_CPU_LATENCY_REQUEST
static ssize_t cpu_latency_check_time_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(to_i2c_client(dev));

	if (wac_i2c->cpu_dev == NULL)
		return sysfs_emit(buf, "unsupported\n");

	return sysfs_emit(buf, "%u\n", jiffies_to_msecs(wac_i2c->cpu_latency_check_time));
}

/* refer to pm_qos_resume_latency_us_store() */
static ssize_t cpu_latency_check_time_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t n)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(to_i2c_client(dev));
	s32 value = 0;

	if (wac_i2c->cpu_dev == NULL)
		return -EINVAL;

	if (kstrtos32(buf, 0, &value) || value < 0)
		return -EINVAL;

	dev_info(&wac_i2c->client->dev, "new cpu_latency_check_time: %d msec(s)\n", value);

	mutex_lock(&wac_i2c->cpu_latency_mutex);
	if (value == 0)
		wac_i2c->cpu_latency_check_time = 0;
	else
		wac_i2c->cpu_latency_check_time = msecs_to_jiffies(value);
	wacom_cpu_latency_update_nolock(wac_i2c);
	mutex_unlock(&wac_i2c->cpu_latency_mutex);

	return n;
}

static DEVICE_ATTR_RW(cpu_latency_check_time);

/* refer to pm_qos_resume_latency_us_show() */
static ssize_t cpu_latency_display_on_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(to_i2c_client(dev));
	s32 value = wac_i2c->cpu_latency_display_on;

	if (wac_i2c->cpu_dev == NULL)
		return sysfs_emit(buf, "unsupported\n");

	if (value == 0)
		return sysfs_emit(buf, "n/a\n");
	if (value == PM_QOS_RESUME_LATENCY_NO_CONSTRAINT)
		value = 0;

	return sysfs_emit(buf, "%d\n", value);
}

/* refer to pm_qos_resume_latency_us_store() */
static ssize_t cpu_latency_display_on_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t n)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(to_i2c_client(dev));
	s32 value;

	if (wac_i2c->cpu_dev == NULL)
		return -EINVAL;

	if (!kstrtos32(buf, 0, &value)) {
		/*
		 * Prevent users from writing negative or "no constraint" values
		 * directly.
		 */
		if (value < 0 || value == PM_QOS_RESUME_LATENCY_NO_CONSTRAINT)
			return -EINVAL;

		if (value == 0)
			value = PM_QOS_RESUME_LATENCY_NO_CONSTRAINT;
	} else if (sysfs_streq(buf, "n/a")) {
		value = 0;
	} else {
		return -EINVAL;
	}

	dev_info(&wac_i2c->client->dev, "new cpu_latency_display_on: %d usec(s)\n", value);

	mutex_lock(&wac_i2c->cpu_latency_mutex);
	wac_i2c->cpu_latency_display_on = value;
	wacom_cpu_latency_update_nolock(wac_i2c);
	mutex_unlock(&wac_i2c->cpu_latency_mutex);

	return n;
}

static DEVICE_ATTR_RW(cpu_latency_display_on);

/* refer to pm_qos_resume_latency_us_show() */
static ssize_t cpu_latency_touched_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(to_i2c_client(dev));
	s32 value = wac_i2c->cpu_latency_touched;

	if (wac_i2c->cpu_dev == NULL)
		return sysfs_emit(buf, "unsupported\n");

	if (value == 0)
		return sysfs_emit(buf, "n/a\n");
	if (value == PM_QOS_RESUME_LATENCY_NO_CONSTRAINT)
		value = 0;

	return sysfs_emit(buf, "%d\n", value);
}

/* refer to pm_qos_resume_latency_us_store() */
static ssize_t cpu_latency_touched_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t n)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(to_i2c_client(dev));
	s32 value;

	if (wac_i2c->cpu_dev == NULL)
		return -EINVAL;

	if (!kstrtos32(buf, 0, &value)) {
		/*
		 * Prevent users from writing negative or "no constraint" values
		 * directly.
		 */
		if (value < 0 || value == PM_QOS_RESUME_LATENCY_NO_CONSTRAINT)
			return -EINVAL;

		if (value == 0)
			value = PM_QOS_RESUME_LATENCY_NO_CONSTRAINT;
	} else if (sysfs_streq(buf, "n/a")) {
		value = 0;
	} else {
		return -EINVAL;
	}

	dev_info(&wac_i2c->client->dev, "new cpu_latency_touched: %d usec(s)\n", value);

	mutex_lock(&wac_i2c->cpu_latency_mutex);
	wac_i2c->cpu_latency_touched = value;
	wacom_cpu_latency_update_nolock(wac_i2c);
	mutex_unlock(&wac_i2c->cpu_latency_mutex);

	return n;
}

static DEVICE_ATTR_RW(cpu_latency_touched);
#endif

static struct attribute *fw_attributes[] = {
	&dev_attr_fwversion.attr,
#ifdef INOCO_CPU_LATENCY_REQUEST
	&dev_attr_cpu_latency_check_time.attr,
	&dev_attr_cpu_latency_display_on.attr,
	&dev_attr_cpu_latency_touched.attr,
#endif
	NULL,
};

static const struct attribute_group fw_attr_group = {
	.attrs = fw_attributes,
};

static int wacom_fwnode_probe(struct wacom_i2c *wac_i2c)
{
	struct device *dev = &wac_i2c->client->dev;
	u32 val;

	wac_i2c->reset_gpiod = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH); /* reset */
	if (IS_ERR(wac_i2c->reset_gpiod)) {
		dev_err(dev, "failed to get rest gpiod, err: %d\n", PTR_ERR(wac_i2c->reset_gpiod));
		return PTR_ERR(wac_i2c->reset_gpiod);
	}

	if (!device_property_read_u32(dev, "post-power-on-delay-ms", &val))
		wac_i2c->post_power_delay_ms = val;

	return 0;
}

static int wacom_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	int error = -1;

	dev_info(&client->dev, "+%s()\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	input = devm_input_allocate_device(&client->dev);
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device, err: %d\n", -ENOMEM);
		return -ENOMEM;
	}

	wac_i2c = devm_kzalloc(&client->dev, sizeof(*wac_i2c), GFP_KERNEL);
	if (!wac_i2c) {
		dev_err(&client->dev, "failed to preserve memory \n");
		return -ENOMEM;
	}

	wac_i2c->client = client;
	wac_i2c->input = input;

	wac_i2c->i2c_read_flags = I2C_M_RD;
	if (IS_ALIGNED((u64)wac_i2c->data, ARCH_KMALLOC_MINALIGN)) {
		// dev_info(&client->dev, "data is DMA aligned\n");
		wac_i2c->i2c_read_flags |= I2C_M_DMA_SAFE;
	}

	error = wacom_fwnode_probe(wac_i2c);
	if (error)
		return error;

	error = wacom_regulator_get(wac_i2c);
	if (error)
		return error;

	error = wacom_pinctrl_get(wac_i2c);
	if (error)
		return error;

	error = wacom_regulator_control(wac_i2c, true);
	if (error)
		return error;

	error = wacom_pinctrl_select(wac_i2c, true);
	if (error)
		goto err_disable_regulator;

	dev_dbg(&client->dev, "%s: power on reset\n", __func__);
	gpiod_set_value(wac_i2c->reset_gpiod, 0); /* not reset */

	if (wac_i2c->post_power_delay_ms)
		wacom_delay(wac_i2c->post_power_delay_ms);

	wac_i2c->features = devm_kzalloc(&client->dev, sizeof(struct wacom_features), GFP_KERNEL);
	if (!wac_i2c->features) {
		dev_err(&client->dev, "failed to preserve memory \n");
		goto err_disable_gpios;
	}

	error = wacom_query_device_direct(client, wac_i2c->features);
	if (error)
		goto err_disable_gpios;

	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x56a;
	input->id.version = wac_i2c->features->fw_version;
	input->dev.parent = &client->dev;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_MISC, 0, 255, 0, 0);
	__set_bit(ABS_MISC, input->absbit);
#ifdef ENABLE_SERIAL
	input_set_capability(input, EV_MSC, MSC_SERIAL);
	__set_bit(MSC_SERIAL, input->mscbit);
#endif

#if 0
	/*Setting maximum coordinate values  */
	/*eliminating 1mm offset on each side*/
	wac_i2c->features->x_max -= (AA_OFFSET * 2);
	wac_i2c->features->y_max -= (AA_OFFSET * 2);
	dev_dbg(&client->dev, "%s feature_xmax: %d feature_ymax: %d \n",
	       __func__, wac_i2c->features->x_max, wac_i2c->features->y_max);
#endif

#if defined(WACOM_SWAP_XY) && (WACOM_SWAP_XY)
	input_set_abs_params(input, ABS_X, 0, wac_i2c->features->y_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, wac_i2c->features->x_max, 0, 0);
	dev_dbg(&client->dev, "swapped: new input params: x_max:%d, y_max:%d\n",
		wac_i2c->features->y_max, wac_i2c->features->x_max);
#else
	input_set_abs_params(input, ABS_X, 0, wac_i2c->features->x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, wac_i2c->features->y_max, 0, 0);
#endif
	input_abs_set_res(input, ABS_X, XY_RESOLUTION);
	input_abs_set_res(input, ABS_Y, XY_RESOLUTION);

	input_set_abs_params(input, ABS_PRESSURE,
			     0, wac_i2c->features->pressure_max, 0, 0);

	if (wac_i2c->features->support.height) {
		input_set_abs_params(input, ABS_DISTANCE, 0, wac_i2c->features->height_max, 0, 0);
		input_abs_set_res(input, ABS_DISTANCE, DIST_RESOLUTION);
	}
	if (wac_i2c->features->support.tilt) {
		input_set_abs_params(input, ABS_TILT_X,
				     -wac_i2c->features->tilt_x_max, wac_i2c->features->tilt_x_max, 0, 0);
		input_set_abs_params(input, ABS_TILT_Y,
				     -wac_i2c->features->tilt_y_max, wac_i2c->features->tilt_y_max, 0, 0);
		input_abs_set_res(input, ABS_TILT_X, TILT_RESOLUTION);
		input_abs_set_res(input, ABS_TILT_Y, TILT_RESOLUTION);
	}

#ifdef CALIBRATION
	wac_i2c->features->calib_data.originX = wac_i2c->features->calib_data.originY = 0;
	wac_i2c->features->calib_data.extentX = wac_i2c->features->x_max;
	wac_i2c->features->calib_data.extentY = wac_i2c->features->y_max;
	wac_i2c->features->node_state = STATE_NORMAL;
	wac_i2c->features->bCalibrationSet = false;
#endif

	input_set_drvdata(input, wac_i2c);

#ifdef INOCO_SET_AFFINITY_HINT_CPU
	wac_i2c->affinity_hint_cpu = U32_MAX;
#endif

	if (client->dev.of_node) {
#if defined(INOCO_SET_AFFINITY_HINT_CPU) || defined(INOCO_CPU_LATENCY_REQUEST)
		u32 value = 0;
#endif

#ifdef INOCO_SET_AFFINITY_HINT_CPU
		if (of_property_read_u32(client->dev.of_node, "affinity-hint-cpu", &value) == 0) {
			error = irq_set_affinity_hint(client->irq, get_cpu_mask(value));
			if (error)
				dev_err(&client->dev, "set irq affinity hint err %d\n", error);
			else {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
				dev_info(&client->dev, "set irq affinity hint: cpu %u\n", value);
#else
				dev_dbg(&client->dev, "set irq affinity hint: cpu %u\n", value);
#endif
				wac_i2c->affinity_hint_cpu = value;
			}
		}
#endif

#ifdef INOCO_CPU_LATENCY_REQUEST
		if (wac_i2c->affinity_hint_cpu < CONFIG_NR_CPUS &&
			(wac_i2c->cpu_dev = get_cpu_device(wac_i2c->affinity_hint_cpu)) != NULL) {
			value = 0;
			if (of_property_read_u32(client->dev.of_node, "cpu-latency-display-on", &value) == 0) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
				dev_info(&client->dev, "cpu_latency_display_on: %u\n", value);
#endif
				wac_i2c->cpu_latency_display_on = value;
			}

			value = 0;
			if (of_property_read_u32(client->dev.of_node, "cpu-latency-touched", &value) == 0) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
				dev_info(&client->dev, "cpu_latency_touched: %u\n", value);
#endif
				wac_i2c->cpu_latency_touched = value;
			}

			value = 0;
			if (of_property_read_u32(client->dev.of_node, "cpu-latency-check-time", &value) == 0) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
				dev_info(&client->dev, "cpu_latency_check_time: %u ms\n", value);
#endif
				if (value >= 1000) { /* msecs */
					wac_i2c->cpu_latency_check_time = msecs_to_jiffies(value); /* jiffies */
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
					dev_info(&client->dev, "cpu_latency_check_time: %u jiffies\n", wac_i2c->cpu_latency_check_time);
#endif
				}
			}
			INIT_DELAYED_WORK(&wac_i2c->cpu_latency_check_work, wacom_cpu_latency_check_work);
		}
#endif
	}

#ifdef INOCO_CPU_LATENCY_REQUEST
	//cpu_latency_qos_add_request(&wac_i2c->pm_qos_request, PM_QOS_DEFAULT_VALUE);
	mutex_init(&wac_i2c->cpu_latency_mutex);
#endif

	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
	irq_set_status_flags(client->irq, IRQ_NOAUTOEN);
	error = devm_request_threaded_irq(&client->dev, client->irq, NULL, wacom_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "wacom_i2c", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to request IRQ, error: %d\n", error);
		goto err_clear_irq_affinity_hint;
	}

	error = input_register_device(wac_i2c->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
		goto err_clear_irq_affinity_hint;
	}

	i2c_set_clientdata(client, wac_i2c);

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	error = register_notifier(wac_i2c);
	if (error)
		goto err_unregister_input;
#endif

	error = sysfs_create_group(&client->dev.kobj, &fw_attr_group);
	if (error)
		dev_err(&client->dev, "sysfs create failed(%d)", error);

#ifdef INOCO_CPU_LATENCY_REQUEST
	if (wac_i2c->cpu_latency_display_on > 0) {
#ifdef INOCO_CPU_LATENCY_REQUEST_DEBUG
		dev_info(&client->dev, "wacom_i2c_probe: latency display %d\n", wac_i2c->cpu_latency_display_on);
#endif
		mutex_lock(&wac_i2c->cpu_latency_mutex);
		wacom_cpu_latency_request_nolock(wac_i2c, wac_i2c->cpu_latency_display_on);
		mutex_unlock(&wac_i2c->cpu_latency_mutex);
	}
#endif

	return 0;

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
err_unregister_input:
	input_unregister_device(wac_i2c->input);
#endif

err_clear_irq_affinity_hint:
#ifdef INOCO_SET_AFFINITY_HINT_CPU
	irq_set_affinity_hint(client->irq, NULL);
#else
	;
#endif
#ifdef INOCO_CPU_LATENCY_REQUEST
	//cpu_latency_qos_remove_request(&wac_i2c->pm_qos_request);
#endif

err_disable_gpios:
	gpiod_set_value(wac_i2c->reset_gpiod, 1); /* reset */
	wacom_pinctrl_select(wac_i2c, false);

err_disable_regulator:
	wacom_regulator_control(wac_i2c, false);

	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

#ifdef INOCO_SET_AFFINITY_HINT_CPU
	irq_set_affinity_hint(client->irq, NULL);
#endif
	free_irq(client->irq, wac_i2c);
#ifdef INOCO_CPU_LATENCY_REQUEST
	//cpu_latency_qos_remove_request(&wac_i2c->pm_qos_request);
#endif
	input_unregister_device(wac_i2c->input);
	sysfs_remove_group(&client->dev.kobj, &fw_attr_group);

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	release_notifier(wac_i2c);
#endif

	gpiod_set_value(wac_i2c->reset_gpiod, 1); /* reset */
	wacom_pinctrl_select(wac_i2c, false);
	wacom_regulator_control(wac_i2c, false);

	return 0;
}

static void wacom_i2c_shut_down(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "+%s()\n", __func__);
	if (!wac_i2c->fb_blank_powerdown) {
		wac_i2c->fb_blank_powerdown = true;
		wacom_i2c_set_sleep(&wac_i2c->client->dev, true);
		disable_irq(wac_i2c->client->irq);
		wacom_pinctrl_select(wac_i2c, false);
		gpiod_set_value(wac_i2c->reset_gpiod, 1); /* reset */
		wacom_regulator_control(wac_i2c, false);
	}
}

static int wacom_i2c_suspend(struct device *dev)
{
#if (!IS_ENABLED(CONFIG_DRM_MEDIATEK))
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);

	wacom_i2c_set_sleep(dev, true);
	disable_irq(wac_i2c->client->irq);
	wacom_pinctrl_select(wac_i2c, false);
	gpiod_set_value(wac_i2c->reset_gpiod, 1); /* reset */
	wacom_regulator_control(wac_i2c, false);
#endif

	return 0;
}

static int wacom_i2c_resume(struct device *dev)
{
#if (!IS_ENABLED(CONFIG_DRM_MEDIATEK))
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);

	gpiod_set_value(wac_i2c->reset_gpiod, 1); /* reset */
	wacom_regulator_control(wac_i2c, true);
	wacom_pinctrl_select(wac_i2c, true);
	gpiod_set_value(wac_i2c->reset_gpiod, 0); /* not reset */
	if (wac_i2c->post_power_delay_ms)
		wacom_delay(wac_i2c->post_power_delay_ms);
	enable_irq(wac_i2c->client->irq);
#endif

	return 0;
}

static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

#ifdef CONFIG_OF
#define WACOM_DT_ID  "emr,wacom_i2c"
static const struct of_device_id wacom_i2c_dt_ids[] = {
	{ .compatible = WACOM_DT_ID },
	//{ .compatible = "hid-over-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, wacom_i2c_dt_ids);
#endif

static const struct i2c_device_id wacom_i2c_id[] = {
	{ "WAC_I2C_EMR", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.owner	= THIS_MODULE,
		.pm	= &wacom_i2c_pm,
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(wacom_i2c_dt_ids),
#endif
	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.shutdown	= wacom_i2c_shut_down,
	.id_table	= wacom_i2c_id,
};
static int __init wacom_i2c_init(void)
{
	return i2c_add_driver(&wacom_i2c_driver);
}

static void __exit wacom_i2c_exit(void)
{
	i2c_del_driver(&wacom_i2c_driver);
}

module_init(wacom_i2c_init);
module_exit(wacom_i2c_exit);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");

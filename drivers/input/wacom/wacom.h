/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011-2015 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

#ifndef WACOM_I2C_H
#define WACOM_I2C_H

#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/device.h>
#include <linux/ktime.h>
#include <asm/unaligned.h>

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
#include "mtk_disp_notify.h"
#endif

/*Defining flags to ebable/disable a functionality*/
//#define CALIBRATION
#define OFFSET_SETTING
#define CHECK_MODE
//#define DEBUG_V
//#define ENABLE_SERIAL //enable MSC_SERIAL input event
/*---------------------*/

#define WACOM_INPUT_SIZE        10
#define AA_OFFSET               100

#define WACOM_MAX_SIZE          128
#define WACOM_EMR_INPUTSIZE     10

#define CMD_SET_FEATURE         0x03
#define CMD_GET_FEATURE         0x02

/*Added for using this prog. in Linux user-space program*/
#define RTYPE_FEATURE           0x03 /*: Report type -> feature(11b)*/
#define GET_FEATURE             0x02
#define SET_FEATURE             0x03
#define GFEATURE_SIZE           6
#define SFEATURE_SIZE           8

#ifdef OFFSET_SETTING
/*ReportID for GET/SET features*/
/*PEN*/
#define REPORT_CMD_SETOFFSET           22
#define RCSOFFSET_SIZE                 7
#define REPORT_CMD_GETOFFSET           REPORT_CMD_SETOFFSET
#define RCGOFFSET_SIZE                 RCSOFFSET_SIZE
#define WACOM_REGULAR_INPUT     10
#endif

/*HID over I2C spec*/
#define HID_DESC_REG            0x01
#define HID_DESC_REGISTER       0x01
#define USAGE_PAGE              0x05
#define USAGE_PAGE_EXTENDED     0x06
#define USAGE_PAGE_DIGITIZERS   0x0d
#define USAGE_PAGE_DESKTOP      0x01
#define USAGE_PAGE_VENDOR       0x00ff
#define USAGE                   0x09
#define USAGE_PEN               0x02
#define USAGE_MOUSE             0x02
#define USAGE_FINGER            0x22
#define USAGE_STYLUS            0x20
#define USAGE_TOUCHSCREEN       0x04
#define USAGE_X                 0x30
#define USAGE_TIPPRESSURE       0x30
#define USAGE_Y                 0x31
#define USAGE_VENDOR            0x04
#define ENDCOLLECTION           0xc0

#ifdef CHECK_MODE
#define WACOM_CMD_QUERY0	0x04
#define WACOM_CMD_QUERY1	0x00
#define WACOM_CMD_QUERY2	0x33
#define WACOM_CMD_QUERY3	0x02
#define WACOM_CMD_THROW0	0x05
#define WACOM_CMD_THROW1	0x00
#define WACOM_QUERY_SIZE	21

#define IN_USER_MODE            0
#define IN_BOOT_MODE            1
#define NOT_WORKING             -1

#define REPORT_CMD_SETBOOT      0x07
#define QUERY_BOOT              0x07
#define RCSBOOT_SIZE            3
#define REPORT_CMD_GETBOOT      0x08
#define RCGBOOT_SIZE            6
#endif

typedef struct hid_descriptor {
	u16 wHIDDescLength;
	u16 bcdVersion;
	u16 wReportDescLength;
	u16 wReportDescRegister;
	u16 wInputRegister;
	u16 wMaxInputLength;
	u16 wOutputRegister;
	u16 wMaxOutputLength;
	u16 wCommandRegister;
	u16 wDataRegister;
	u16 wVendorID;
	u16 wProductID;
	u16 wVersion;
	u16 RESERVED_HIGH;
	u16 RESERVED_LOW;
} HID_DESC;

#ifdef CALIBRATION
/*Added for Calibration purpose*/
typedef enum {
	STATE_NORMAL,
	STATE_QUERY,
	STATE_GETCAL,
	STATE_POINTS,
} NODE_STATE;

struct calibrationData {
  int   originX;
  int   originY;
  int   extentX;
  int   extentY;
};
#endif


struct feature_support {
	bool height;
	bool tilt;
};

struct wacom_features {
	struct feature_support support;
	int x_max;
	int y_max;
	int pressure_max;
	int height_max;
	int tilt_x_max;
	int tilt_y_max;
	unsigned int fw_version;
	HID_DESC hid_desc;
	int vendorId;
	int productId;
	unsigned int input_size;

#ifdef CALIBRATION
	struct calibrationData calib_data;
	NODE_STATE node_state;
	bool bCalibrationSet;
#endif

};

struct wacom_i2c {
	struct i2c_client *client;
	struct input_dev *input;
	struct wacom_features *features;
	struct class *class;
	struct device *dev;
	struct regulator *vdd1v8;
	struct regulator *vdd3v3;
	struct pinctrl *pinctrl;
	struct pinctrl_state *active;
	struct pinctrl_state *sleep;
	struct gpio_desc *reset_gpiod;

#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	struct notifier_block fb_notif;
	bool fb_blank_powerdown;
#endif

	bool vdd1v8_status;
	bool vdd3v3_status;

	int tool;
	u8 data[WACOM_MAX_SIZE];
	u8 cmd;
	bool prox;
	int post_power_delay_ms;
	ktime_t poweron_kt;
};

int wacom_query_device(struct i2c_client *client, struct wacom_features *features);

#ifdef OFFSET_SETTING
bool wacom_i2c_set_feature(struct i2c_client *client, u8 report_id, unsigned int buf_size, u8 *data,
			   u16 cmdreg, u16 datareg);
bool wacom_i2c_get_feature(struct i2c_client *client, u8 report_id, unsigned int buf_size, u8 *data,
			   u16 cmdreg, u16 datareg);
#endif

#ifdef CALIBRATION
void set_calib(int *x, int *y, int x_max, int y_max,
	       int originX, int originY, int extentX, int extentY);
#endif
#endif

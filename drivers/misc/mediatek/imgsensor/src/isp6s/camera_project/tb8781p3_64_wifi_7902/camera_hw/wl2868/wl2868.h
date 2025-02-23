/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#ifndef __IMGSENSOR_HW_WL2868_h__
#define __IMGSENSOR_HW_WL2868_h__

#include "imgsensor_hw.h"
#include "imgsensor_common.h"
#include "../../../../camera_ldo/camera_ldo.h"

//extern void camera_ldo_set_ldo_value(CAMERA_LDO_SELECT ldonum,unsigned int value);
//extern void camera_ldo_set_en_ldo(CAMERA_LDO_SELECT ldonum,unsigned int en);

typedef enum  {
	EXTLDO_REGULATOR_VOLTAGE_0    = 0,
	EXTLDO_REGULATOR_VOLTAGE_1000 = 1000,
	EXTLDO_REGULATOR_VOLTAGE_1050 = 1050,
	EXTLDO_REGULATOR_VOLTAGE_1100 = 1100,
	EXTLDO_REGULATOR_VOLTAGE_1150 = 1150,
	EXTLDO_REGULATOR_VOLTAGE_1200 = 1200,
	EXTLDO_REGULATOR_VOLTAGE_1210 = 1210,
	EXTLDO_REGULATOR_VOLTAGE_1220 = 1220,
	EXTLDO_REGULATOR_VOLTAGE_1500 = 1500,
	EXTLDO_REGULATOR_VOLTAGE_1800 = 1800,
	EXTLDO_REGULATOR_VOLTAGE_2500 = 2500,
	EXTLDO_REGULATOR_VOLTAGE_2800 = 2800,
	EXTLDO_REGULATOR_VOLTAGE_2900 = 2900,
}EXTLDO_REGULATOR_VOLTAGE;

struct WL2868_LDOMAP{
	enum IMGSENSOR_SENSOR_IDX idx;
	enum IMGSENSOR_HW_PIN hwpin;
	CAMERA_LDO_SELECT wl2868ldo;
};


enum IMGSENSOR_RETURN imgsensor_hw_wl2868_open(
	struct IMGSENSOR_HW_DEVICE **pdevice);

#endif


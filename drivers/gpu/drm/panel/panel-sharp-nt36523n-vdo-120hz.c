// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define SHARP_NAME "sharp-nt36523n-vdo-120hz"
#define SHARP_DRV_NAME "panel-"SHARP_NAME

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#ifdef CONFIG_DRM_MTK_ICOM_LCM_SET_DISPLAY_ON_DELAY
#include <linux/time64.h>
#include <linux/timekeeping.h>
#endif

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "panel-sharp-nt36523n-vdo-120hz.h"


// #define SHARP_VERBOSE_DEBUG
#ifdef SHARP_VERBOSE_DEBUG
#define LCM_DEV_LOGD(fmt, args...)	dev_info(ctx->dev, "[LCM][%s][%d]: " fmt, __func__, __LINE__, ##args)
#define LCM_LOGD(fmt, args...)		pr_info("[LCM][%s][%d]: " fmt, __func__, __LINE__, ##args)
#else
#define LCM_DEV_LOGD(fmt, args...)
#define LCM_LOGD(fmt, args...)
#endif

#define LCM_DEV_LOGE(fmt, args...)	dev_err(ctx->dev, "[LCM][%s][%d][ERR]: " fmt, __func__, __LINE__, ##args)
#define LCM_LOGE(fmt, args...)		pr_err("[LCM][%s][%d][ERR]: " fmt, __func__, __LINE__, ##args)

#define LCM_DEV_LOGI(fmt, args...)	dev_info(ctx->dev, "[LCM][%s][%d]: " fmt, __func__, __LINE__, ##args)
#define LCM_LOGI(fmt, args...)		pr_info("[LCM][%s][%d]: " fmt, __func__, __LINE__, ##args)

// ---------------------------------------------------------------------------

struct sharp {
	struct device *dev;
	struct drm_panel panel;
	struct regulator *disp_vddi;
	struct gpio_desc *reset_gpio;
	bool prepared;
	bool enabled;
#ifdef CONFIG_DRM_MTK_ICOM_LCM_SET_DISPLAY_ON_DELAY
	bool display_on;
	struct timespec64 lcm_init_time;
#endif

	struct regulator *disp_bias_pos;
	struct regulator *disp_bias_neg;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;

	unsigned int sample_id1;
	unsigned int sample_id2;
	unsigned int sample_id3;
	unsigned int lcm_config;

	unsigned int islcmfound;

	int error;
};

static int current_fps = 60;

static void sharp_check_sample_id(struct sharp *ctx)
{
	LCM_DEV_LOGI("+++");

	if (ctx->sample_id2 == SHARP_PRE_TS_ID2) {
		LCM_DEV_LOGI("Use Pre-TS Config\n");
		ctx->lcm_config = PRE_TS_CONFIG;
	} else {
		if (ctx->sample_id1 == SHARP_TS_ID1) {
			LCM_DEV_LOGI("Use TS Config\n");
			ctx->lcm_config = TS_CONFIG;
		} else if ((ctx->sample_id1 & 0xf0) == SHARP_ES_ID1) {
			if (ctx->sample_id3 == SHARP_PRE_ES_ID3) {
				LCM_DEV_LOGI("Use Pre-ES Config\n");
				ctx->lcm_config = PRE_ES_CONFIG;
			} else {
				if (ctx->sample_id1 == SHARP_ES_ID1) {
					LCM_DEV_LOGI("Use ES Config\n");
					ctx->lcm_config = ES_CONFIG;
				} else if (ctx->sample_id1 == SHARP_ES2_1_ID1) {
					LCM_DEV_LOGI("Use ES2-1 Config\n");
					ctx->lcm_config = ES2_1_CONFIG;
				} else if (ctx->sample_id1 == SHARP_ES2_2_ID1) {
					LCM_DEV_LOGI("Use ES2-2 Config\n");
					ctx->lcm_config = ES2_2_CONFIG;
				} else /*if (ctx->sample_id1 == SHARP_ES3_ID1)*/ {
					LCM_DEV_LOGI("Use ES3 Config\n");
					ctx->lcm_config = ES3_CONFIG;
				}
			}
		} else if ((ctx->sample_id1 & 0xf0) == SHARP_CS_ID1) {
			/*if (ctx->sample_id1 == SHARP_CS_ID1)*/ {
				LCM_LOGI("Use CS Config\n");
				ctx->lcm_config = CS_CONFIG;
			}
		} else if ((ctx->sample_id1 & 0xf0) == SHARP_MP_ID1) {
			/*if (ctx->sample_id1 == SHARP_MP_ID1)*/ {
				LCM_LOGI("Use MP Config\n");
				ctx->lcm_config = MP_CONFIG;
			}
		} else {
			LCM_LOGI("Unknown Sample ID (id1: 0x%02x, id2: 0x%02x)\n",
				ctx->sample_id1, ctx->sample_id2);
			LCM_LOGI("Use Pre-TS Config\n");
			ctx->lcm_config = PRE_TS_CONFIG;
		}
	}
	LCM_DEV_LOGI("ctx->lcm_config: 0x%X\n", ctx->lcm_config);

	LCM_DEV_LOGD("---");
}

static void panel_udelay(unsigned int us)
{
	if (us > 0) {
		if (us > 10)
			usleep_range(us, us);
		else
			udelay(us);
	}
}

static void panel_mdelay(unsigned int ms)
{
	if (ms > 0) {
		if (ms > 20)
			msleep(ms);
		else
			usleep_range(ms * 1000, ms * 1000);
	}
}
#define UDELAY(n) (panel_udelay(n))
#define MDELAY(n) (panel_mdelay(n))

static int bias_tps65132_enable(struct sharp *ctx, int vpos_val, int vneg_val)
{
	int ret = 0;
	int retval = 0;
	int volt;

	LCM_DEV_LOGI("+++");

	/* enable regulator */
	ret = regulator_enable(ctx->disp_bias_pos);
	if (ret < 0)
		LCM_DEV_LOGE("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;
	ret = regulator_enable(ctx->disp_bias_neg);
	if (ret < 0)
		LCM_DEV_LOGE("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;
	MDELAY(5);

	/* set voltage with min & max*/
	volt = regulator_get_voltage(ctx->disp_bias_pos);
	LCM_DEV_LOGI("Check disp_bias_pos voltage: %d", volt);
	if (volt != vpos_val) {
		ret = regulator_set_voltage(ctx->disp_bias_pos, volt, volt);
		if (ret < 0)
			LCM_DEV_LOGE("set voltage disp_bias_pos fail, voltage = %d ret = %d\n", volt, ret);
		retval |= ret;

		ret = regulator_set_voltage(ctx->disp_bias_pos, vpos_val, vpos_val);
		if (ret < 0)
			LCM_DEV_LOGE("set voltage disp_bias_pos fail, voltage = %d ret = %d\n", vpos_val, ret);
		retval |= ret;
	}

	volt = regulator_get_voltage(ctx->disp_bias_neg);
	LCM_DEV_LOGI("Check disp_bias_neg voltage: %d", volt);
	if (volt != vneg_val) {
		ret = regulator_set_voltage(ctx->disp_bias_neg, volt, volt);
		if (ret < 0)
			LCM_DEV_LOGE("set voltage disp_bias_neg fail, voltage = %d ret = %d\n", volt, ret);
		retval |= ret;

		ret = regulator_set_voltage(ctx->disp_bias_neg, vneg_val, vneg_val);
		if (ret < 0)
			LCM_DEV_LOGE("set voltage disp_bias_neg fail, voltage = %d ret = %d\n", vpos_val, ret);
		retval |= ret;
	}

	// /*
	//  * Tstartup = (Cout x Vneg) / Istartup
	//  * Tstartup = ((4.7 x 0.000001)F x (6.0)V) / (80 x 0.0001)A = 0.003525
	//  */
	// MDELAY(4);

	LCM_DEV_LOGD("---");
	return retval;
}

static int bias_tps65132_disable(struct sharp *ctx)
{
	int ret = 0;
	int retval = 0;

	LCM_DEV_LOGI("+++");

	ret = regulator_disable(ctx->disp_bias_neg);
	if (ret < 0)
		LCM_DEV_LOGE("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(ctx->disp_bias_pos);
	if (ret < 0)
		LCM_DEV_LOGE("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	LCM_DEV_LOGD("---");
	return retval;
}

static inline struct sharp *panel_to_sharp(struct drm_panel *panel)
{
	return container_of(panel, struct sharp, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int sharp_dcs_read(struct sharp *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	LCM_DEV_LOGI("+++");

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		LCM_DEV_LOGE("error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	LCM_DEV_LOGD("+++");
	return ret;
}

static void sharp_panel_get_data(struct sharp *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	LCM_DEV_LOGI("+++");

	if (ret == 0) {
		ret = sharp_dcs_read(ctx, 0x0A, buffer, 1);
		LCM_DEV_LOGI("0x%08x\n", buffer[0] | (buffer[1] << 8));
		LCM_DEV_LOGI("return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}

	LCM_DEV_LOGD("---");
}
#endif

static void sharp_dcs_write(struct sharp *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		LCM_DEV_LOGE("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void push_table(struct sharp *ctx,
		struct LCM_setting_table *table, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned int cmd;

		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_MDELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			LCM_DEV_LOGD("cmd: 0x%X, count: %d, para_list[0]: 0x%02X, para_list[1]: 0x%02X",cmd, table[i].count, table[i].para_list[0], table[i].para_list[1]);
			sharp_dcs_write(ctx, table[i].para_list, table[i].count);
			break;
		}
	}
}

static int sharp_panel_init(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);
	int ret;

	LCM_DEV_LOGI("+++");

	/* Reset = H */
	gpiod_set_value(ctx->reset_gpio, 1);
	UDELAY(10);
	/* Reset = L */
	gpiod_set_value(ctx->reset_gpio, 0);
	UDELAY(10);
	/* Reset = H */
	gpiod_set_value(ctx->reset_gpio, 1);
	MDELAY(90);
	/* [Automatic] MTP Auto load */
	/* [Automatic] Sleep mode ON */

	/* Initial code */
	switch (ctx->lcm_config & SAMPLE_MASK) {
	case SAMPLE_MP:
	case SAMPLE_CS:
	case SAMPLE_ES3:
	case SAMPLE_ES2_2:
		push_table(ctx, init_es2_2_120hz_3x_dsc, ARRAY_SIZE(init_es2_2_120hz_3x_dsc));
		break;

	case SAMPLE_ES2_1:
	case SAMPLE_ES:
		push_table(ctx, init_es_120hz_3x_dsc, ARRAY_SIZE(init_es_120hz_3x_dsc));
		break;

	case SAMPLE_PRE_ES:
	case SAMPLE_TS:
		if (ctx->lcm_config & VFP_CASE)
			push_table(ctx, init_ts_120hz_3x_dsc_vfp, ARRAY_SIZE(init_ts_120hz_3x_dsc_vfp));
		else
			push_table(ctx, init_ts_120hz_3x_dsc, ARRAY_SIZE(init_ts_120hz_3x_dsc));
		break;

	case SAMPLE_PRE_TS:
	default:
		if (ctx->lcm_config & PARAMS_DSC)
			push_table(ctx, init_pre_ts_120hz_3x_dsc, ARRAY_SIZE(init_pre_ts_120hz_3x_dsc));
		else /* No DSC */
			push_table(ctx, init_pre_ts_60hz_no_dsc, ARRAY_SIZE(init_pre_ts_60hz_no_dsc));
		break;
	}

#ifdef CONFIG_DRM_MTK_ICOM_LCM_SET_DISPLAY_ON_DELAY
	ktime_get_real_ts64(&ctx->lcm_init_time);
	ctx->display_on = false;
#else
	if (ctx->lcm_config & FRAMESKIP_CASE) {
		if ((current_fps == 120) && (ctx->lcm_config & FPS_120_FRAMESKIP))
			push_table(ctx, set_frameskip_120hz, ARRAY_SIZE(set_frameskip_120hz));
		else if ((current_fps == 60) && (ctx->lcm_config & FPS_60_FRAMESKIP))
			push_table(ctx, set_frameskip_60hz, ARRAY_SIZE(set_frameskip_60hz));
		else if ((current_fps == 30) && (ctx->lcm_config & FPS_30_FRAMESKIP))
			push_table(ctx, set_frameskip_30hz, ARRAY_SIZE(set_frameskip_30hz));
		else if ((current_fps == 10) && (ctx->lcm_config & FPS_10_FRAMESKIP))
			push_table(ctx, set_frameskip_10hz, ARRAY_SIZE(set_frameskip_10hz));
		else if ((current_fps == 1) && (ctx->lcm_config & FPS_1_FRAMESKIP))
			push_table(ctx, set_frameskip_1hz, ARRAY_SIZE(set_frameskip_1hz));
		else if ((current_fps == 1) && (ctx->lcm_config & FPS_1_VFP_FRAMESKIP))
			push_table(ctx, set_vfp_frameskip_1hz, ARRAY_SIZE(set_vfp_frameskip_1hz));
	}
	LCM_DEV_LOGI("current_fps: %d\n", current_fps);
#endif

	ret = ctx->error;

	LCM_DEV_LOGI("---");

	return ret;
}

#ifdef CONFIG_DRM_MTK_ICOM_LCM_SET_DISPLAY_ON_DELAY
#define cb_push_table(_dsi, _handle, _table, _count) \
({									\
	unsigned int i;					\
	for (i = 0; i < _count; i++) {	\
		unsigned int cmd;			\
		cmd = _table[i].cmd;			\
		switch (cmd) {				\
		case REGFLAG_MDELAY:		\
			MDELAY(_table[i].count);	\
			break;					\
		case REGFLAG_UDELAY:		\
			UDELAY(_table[i].count);	\
			break;					\
		case REGFLAG_END_OF_TABLE:	\
			break;					\
		default:					\
			LCM_DEV_LOGD("cmd: 0x%X, count: %d, para_list[0]: 0x%02X, para_list[1]: 0x%02X",\
							cmd, _table[i].count, _table[i].para_list[0], _table[i].para_list[1]); \
			cb(_dsi, _handle, _table[i].para_list, _table[i].count); \
			break; \
		} \
	} \
})

static int sharp_display_on(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle)
{
	struct sharp *ctx = panel_to_sharp(panel);
	int ret;
	struct timespec64 current_time, diff_time;
	s64 timeout_ns = (120 * 1000 * 1000), diff_time_ns;
	int i = 0;

	LCM_DEV_LOGI("+++");

	if (ctx->display_on) {
		LCM_DEV_LOGD("--- panel is display on");
		return 0;
	}

	ktime_get_real_ts64(&current_time);

	diff_time = timespec64_sub(current_time, ctx->lcm_init_time);
	diff_time_ns = timespec64_to_ns(&diff_time);

	// LCM_DEV_LOGI("lcm_init_time: tv_sec=%d tv_nsec=%ld", ctx->lcm_init_time.tv_sec, ctx->lcm_init_time.tv_nsec);
	// LCM_DEV_LOGI("current_time: tv_sec=%d tv_nsec=%ld", current_time.tv_sec, current_time.tv_nsec);
	// LCM_DEV_LOGI("diff_time: tv_sec=%d tv_nsec=%ld", diff_time.tv_sec, diff_time.tv_nsec);
	LCM_DEV_LOGI("diff_time_ns: %ld ns , %ld ms", diff_time_ns, (diff_time_ns / 1000 / 1000));

	while (diff_time_ns < timeout_ns) {
		i++;
		MDELAY(1);
		ktime_get_real_ts64(&current_time);
		diff_time = timespec64_sub(current_time, ctx->lcm_init_time);
		diff_time_ns = timespec64_to_ns(&diff_time);
		if (i % 1000 == 0) {
			LCM_DEV_LOGE("i=%d, lcm_init_time: %ld ns, cur_time: %ld ns\n",
				i, timespec64_to_ns(&ctx->lcm_init_time), timespec64_to_ns(&current_time));
		}
	}

	cb_push_table(dsi, handle, set_display_on, ARRAY_SIZE(set_display_on));

	if (ctx->lcm_config & FRAMESKIP_CASE) {
		if ((current_fps == 120) && (ctx->lcm_config & FPS_120_FRAMESKIP))
			cb_push_table(dsi, handle, set_frameskip_120hz, ARRAY_SIZE(set_frameskip_120hz));
		else if ((current_fps == 60) && (ctx->lcm_config & FPS_60_FRAMESKIP))
			cb_push_table(dsi, handle, set_frameskip_60hz, ARRAY_SIZE(set_frameskip_60hz));
		else if ((current_fps == 30) && (ctx->lcm_config & FPS_30_FRAMESKIP))
			cb_push_table(dsi, handle, set_frameskip_30hz, ARRAY_SIZE(set_frameskip_30hz));
		else if ((current_fps == 10) && (ctx->lcm_config & FPS_10_FRAMESKIP))
			cb_push_table(dsi, handle, set_frameskip_10hz, ARRAY_SIZE(set_frameskip_10hz));
		else if ((current_fps == 1) && (ctx->lcm_config & FPS_1_FRAMESKIP))
			cb_push_table(dsi, handle, set_frameskip_1hz, ARRAY_SIZE(set_frameskip_1hz));
		else if ((current_fps == 1) && (ctx->lcm_config & FPS_1_VFP_FRAMESKIP))
			cb_push_table(dsi, handle, set_vfp_frameskip_1hz, ARRAY_SIZE(set_vfp_frameskip_1hz));
	}
	LCM_DEV_LOGI("current_fps: %d\n", current_fps);

	ret = ctx->error;

	ctx->display_on = true;

	LCM_DEV_LOGI("---");

	return ret;
}

static void sharp_display_on_get_state(struct drm_panel *panel, bool *state)
{
	struct sharp *ctx = panel_to_sharp(panel);

	*state = ctx->display_on;
}
#endif

static int sharp_display_off(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);

	LCM_DEV_LOGI("+++");

	/* Command page select CMD1 */
	/* Display OFF */
	/* Sleep in */
	push_table(ctx, set_display_off, ARRAY_SIZE(set_display_off));

	LCM_DEV_LOGI("current_fps: %d\n", current_fps);

	LCM_DEV_LOGI("---");

	return 0;
}

static int sharp_unprepare(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);
	int ret;

	LCM_DEV_LOGI("+++");

	if (!ctx->prepared) {
		LCM_DEV_LOGD("--- panel is not prepared");
		return 0;
	}

#ifndef CONFIG_DRM_MTK_ICOM_DSI_POWER_SEQUENCE
	sharp_display_off(panel);
#endif
	/* Delay 60 ms */
	MDELAY(60);

	/* AVEE OFF */
	/* AVDD OFF */
	if (ctx->disp_bias_pos && ctx->disp_bias_neg) {
		bias_tps65132_disable(ctx);
		// MDELAY(0);
	}

	/* VDDI OFF */
	if (ctx->disp_vddi) {
		ret = regulator_disable(ctx->disp_vddi);
		if (ret < 0) {
			LCM_DEV_LOGE("regulator_disable vddi fail: %d\n", ret);
		}
		// MDELAY(0);
	}

	/* Reset = L */
	gpiod_set_value(ctx->reset_gpio, 0);

	LCM_DEV_LOGI("current_fps: %d\n", current_fps);

	ctx->error = 0;
	ctx->prepared = false;

	LCM_DEV_LOGD("---");

	return 0;
}

static int sharp_disable(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);

	LCM_DEV_LOGI("+++");

	if (!ctx->enabled) {
		LCM_DEV_LOGD("--- panel is not enabled");
		return 0;
	}

	LCM_DEV_LOGI("current_fps: %d\n", current_fps);

	ctx->enabled = false;

	LCM_DEV_LOGD("---");

	return 0;
}

static int sharp_prepare(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);
	int ret;

	LCM_DEV_LOGI("+++");

	if (ctx->prepared) {
		LCM_DEV_LOGD("--- panel is prepared");
		return 0;
	}

	/* Reset = L */
	gpiod_set_value(ctx->reset_gpio, 0);

	/* VDDI ON */
	if (ctx->disp_vddi) {
		ctx->error = regulator_enable(ctx->disp_vddi);
		if (ctx->error < 0) {
			LCM_DEV_LOGE("regulator_enable vddi fail: %d\n", ctx->error);
		}
		MDELAY(1);
	}

	/* AVDD ON */
	/* AVEE ON */
	if (ctx->disp_bias_pos && ctx->disp_bias_neg) {
		switch (ctx->lcm_config & SAMPLE_MASK) {
		case SAMPLE_MP:
		case SAMPLE_CS:
		case SAMPLE_ES3:
		case SAMPLE_ES2_2:
		case SAMPLE_ES2_1:
		case SAMPLE_ES:
		case SAMPLE_PRE_ES:
		case SAMPLE_TS:
			bias_tps65132_enable(ctx, 5400000, 5400000);
			break;

		case SAMPLE_PRE_TS:
		default:
			bias_tps65132_enable(ctx, 6000000, 6000000);
			break;
		}
		MDELAY(10);
	}

#ifndef CONFIG_DRM_MTK_ICOM_DSI_POWER_SEQUENCE
	sharp_panel_init(panel);
#endif

	ret = ctx->error;
	if (ret < 0) {
		LCM_DEV_LOGE("lcm init fail: %d\n", ret);
		sharp_unprepare(panel);
	}

	ctx->prepared = true;

	LCM_DEV_LOGD("---");

	return 0;
}

static int sharp_enable(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);

	LCM_DEV_LOGI("+++");

	if (ctx->enabled) {
		LCM_DEV_LOGD("--- panel is enabled");
		return 0;
	}

	ctx->enabled = true;

	LCM_DEV_LOGD("---");
	return 0;
}

#define _DRM_DISP_MODE(_name, _clock, _hac, _hfp, _hsa, _hbp, _vac, _vfp, _vsa, _vbp) \
struct drm_display_mode display_mode_##_name = { \
	.clock			= _clock, \
	.hdisplay		= _hac, \
	.hsync_start	= _hac + _hfp, \
	.hsync_end		= _hac + _hfp + _hsa, \
	.htotal			= _hac + _hfp + _hsa + _hbp, \
	.vdisplay		= _vac, \
	.vsync_start	= _vac + _vfp, \
	.vsync_end		= _vac + _vfp + _vsa, \
	.vtotal			= _vac + _vfp + _vsa + _vbp, \
}

#define DISP_MODE_60HZ_NO_DSC \
_DRM_DISP_MODE(60hz_no_dsc, CLOCK_KHZ_60HZ_NO_DSC, \
				HAC_60HZ_NO_DSC, HFP_60HZ_NO_DSC, HSA_60HZ_NO_DSC, HBP_60HZ_NO_DSC,\
				VAC_60HZ_NO_DSC, VFP_60HZ_NO_DSC, VSA_60HZ_NO_DSC, VBP_60HZ_NO_DSC)

#define DISP_MODE_120HZ_DSC(_name, _clock, _vfp) \
_DRM_DISP_MODE(_name, _clock, \
				HAC_120HZ_3X_DSC, HFP_120HZ_3X_DSC, HSA_120HZ_3X_DSC, HBP_120HZ_3X_DSC,\
				VAC_120HZ_3X_DSC, _vfp, VSA_120HZ_3X_DSC, VBP_120HZ_3X_DSC)

#define DISP_MODE_120HZ_DSC_VFP(_fps, _vfp) DISP_MODE_120HZ_DSC(_fps##hz_dsc_vfp, CLOCK_KHZ_120HZ_3X_DSC, _vfp)

#define DISP_MODE_120HZ_DSC_FRAMESKIP(_fps, _clock) DISP_MODE_120HZ_DSC(_fps##hz_dsc_frameskip, _clock, VFP_120HZ_3X_DSC)

// display_mode_60hz_no_dsc
static const DISP_MODE_60HZ_NO_DSC;
// display_mode_120hz_dsc
static const DISP_MODE_120HZ_DSC(120hz_dsc, CLOCK_KHZ_120HZ_3X_DSC, VFP_120HZ_3X_DSC);
// display_mode_90hz_dsc_vfp
static const DISP_MODE_120HZ_DSC_VFP(90, VFP_120HZ_3X_DSC_VFP_90HZ);
// display_mode_72hz_dsc_vfp
static const DISP_MODE_120HZ_DSC_VFP(72, VFP_120HZ_3X_DSC_VFP_72HZ);
// display_mode_60hz_dsc_vfp
static const DISP_MODE_120HZ_DSC_VFP(60, VFP_120HZ_3X_DSC_VFP_60HZ);
// display_mode_45hz_dsc_vfp
static const DISP_MODE_120HZ_DSC_VFP(45, VFP_120HZ_3X_DSC_VFP_45HZ);
// display_mode_30hz_dsc_vfp
static const DISP_MODE_120HZ_DSC_VFP(30, VFP_120HZ_3X_DSC_VFP_30HZ);
// display_mode_24hz_dsc_vfp
static const DISP_MODE_120HZ_DSC_VFP(24, VFP_120HZ_3X_DSC_VFP_24HZ);
// display_mode_15hz_dsc_vfp
static const DISP_MODE_120HZ_DSC_VFP(15, VFP_120HZ_3X_DSC_VFP_15HZ);
// display_mode_10hz_dsc_vfp
static const DISP_MODE_120HZ_DSC_VFP(10, VFP_120HZ_3X_DSC_VFP_10HZ);
// display_mode_6hz_dsc_vfp
static const DISP_MODE_120HZ_DSC_VFP(6, VFP_120HZ_3X_DSC_VFP_6HZ);
#define DSC_VFP_FRAMESKIP_1HZ_BASE		(24)
// display_mode_1hz_dsc_vfp_frameskip
#if (DSC_VFP_FRAMESKIP_1HZ_BASE == 10)
static const DISP_MODE_120HZ_DSC(1hz_dsc_vfp_frameskip, CLOCK_KHZ_120HZ_3X_DSC_10HZ_VFP_FRAMESKIP_1HZ, VFP_120HZ_3X_DSC_VFP_10HZ);
#elif (DSC_VFP_FRAMESKIP_1HZ_BASE == 24)
static const DISP_MODE_120HZ_DSC(1hz_dsc_vfp_frameskip, CLOCK_KHZ_120HZ_3X_DSC_24HZ_VFP_FRAMESKIP_1HZ, VFP_120HZ_3X_DSC_VFP_24HZ);
#else
static const DISP_MODE_120HZ_DSC(1hz_dsc_vfp_frameskip, CLOCK_KHZ_120HZ_3X_DSC_30HZ_VFP_FRAMESKIP_1HZ, VFP_120HZ_3X_DSC_VFP_30HZ);
#endif
// display_mode_60hz_dsc_frameskip
static const DISP_MODE_120HZ_DSC_FRAMESKIP(60, CLOCK_KHZ_120HZ_3X_DSC_FRAMESKIP_60HZ);
// display_mode_30hz_dsc_frameskip
static const DISP_MODE_120HZ_DSC_FRAMESKIP(30, CLOCK_KHZ_120HZ_3X_DSC_FRAMESKIP_30HZ);
// display_mode_10hz_dsc_frameskip
static const DISP_MODE_120HZ_DSC_FRAMESKIP(10, CLOCK_KHZ_120HZ_3X_DSC_FRAMESKIP_10HZ);
// display_mode_1hz_dsc_frameskip
static const DISP_MODE_120HZ_DSC_FRAMESKIP(1, CLOCK_KHZ_120HZ_3X_DSC_FRAMESKIP_1HZ);

#if defined(CONFIG_MTK_PANEL_EXT)
#ifdef USE_DSC
#define PANEL_EXT_DSC_PARAM(_en) \
	.dsc_params = { \
		.enable					= _en, \
		.slice_mode				= DSC_SLICE_MODE, \
		.dsc_cfg				= DSC_DSC_CFG, \
		.ver					= DSC_VER, \
		.rgb_swap				= DSC_RGB_SWAP, \
		.rct_on					= DSC_RCT_ON, \
		.bit_per_channel		= DSC_BIT_PER_CHANNEL, \
		.dsc_line_buf_depth		= DSC_DSC_LINE_BUF_DEPTH, \
		.bp_enable				= DSC_BP_ENABLE, \
		.bit_per_pixel			= DSC_BIT_PER_PIXEL, \
		.pic_height				= DSC_PIC_HEIGHT, \
		.pic_width				= DSC_PIC_WIDTH, \
		.slice_height			= DSC_SLICE_HEIGHT, \
		.slice_width			= DSC_SLICE_WIDTH, \
		.chunk_size				= DSC_CHUNK_SIZE, \
		.xmit_delay				= DSC_XMIT_DELAY, \
		.dec_delay				= DSC_DEC_DELAY, \
		.scale_value			= DSC_SCALE_VALUE, \
		.increment_interval		= DSC_INCREMENT_INTERVAL, \
		.decrement_interval		= DSC_DECREMENT_INTERVAL, \
		.line_bpg_offset		= DSC_LINE_BPG_OFFSET, \
		.nfl_bpg_offset			= DSC_NFL_BPG_OFFSET, \
		.slice_bpg_offset		= DSC_SLICE_BPG_OFFSET, \
		.initial_offset			= DSC_INITIAL_OFFSET, \
		.final_offset			= DSC_FINAL_OFFSET, \
		.flatness_minqp			= DSC_FLATNESS_MINQP, \
		.flatness_maxqp			= DSC_FLATNESS_MAXQP, \
		.rc_model_size			= DSC_RC_MODEL_SIZE, \
		.rc_edge_factor			= DSC_RC_EDGE_FACTOR, \
		.rc_quant_incr_limit0	= DSC_RC_QUANT_INCR_LIMIT0, \
		.rc_quant_incr_limit1	= DSC_RC_QUANT_INCR_LIMIT1, \
		.rc_tgt_offset_hi		= DSC_RC_TGT_OFFSET_HI, \
		.rc_tgt_offset_lo		= DSC_RC_TGT_OFFSET_LO, \
		.rc_buf_thresh[0]		= DSC_RC_BUF_THRESH_0, \
		.rc_buf_thresh[1]		= DSC_RC_BUF_THRESH_1, \
		.rc_buf_thresh[2]		= DSC_RC_BUF_THRESH_2, \
		.rc_buf_thresh[3]		= DSC_RC_BUF_THRESH_3, \
		.rc_buf_thresh[4]		= DSC_RC_BUF_THRESH_4, \
		.rc_buf_thresh[5]		= DSC_RC_BUF_THRESH_5, \
		.rc_buf_thresh[6]		= DSC_RC_BUF_THRESH_6, \
		.rc_buf_thresh[7]		= DSC_RC_BUF_THRESH_7, \
		.rc_buf_thresh[8]		= DSC_RC_BUF_THRESH_8, \
		.rc_buf_thresh[9]		= DSC_RC_BUF_THRESH_9, \
		.rc_buf_thresh[10]		= DSC_RC_BUF_THRESH_10, \
		.rc_buf_thresh[11]		= DSC_RC_BUF_THRESH_11, \
		.rc_buf_thresh[12]		= DSC_RC_BUF_THRESH_12, \
		.rc_buf_thresh[13]		= DSC_RC_BUF_THRESH_13, \
		.rc_range_parameters[0].range_min_qp		= DSC_RC_RANGE_0_RANGE_MIN_QP, \
		.rc_range_parameters[0].range_max_qp		= DSC_RC_RANGE_0_RANGE_MAX_QP, \
		.rc_range_parameters[0].range_bpg_offset	= DSC_RC_RANGE_0_RANGE_BPG_OFFSET, \
		.rc_range_parameters[1].range_min_qp		= DSC_RC_RANGE_1_RANGE_MIN_QP, \
		.rc_range_parameters[1].range_max_qp		= DSC_RC_RANGE_1_RANGE_MAX_QP, \
		.rc_range_parameters[1].range_bpg_offset	= DSC_RC_RANGE_1_RANGE_BPG_OFFSET, \
		.rc_range_parameters[2].range_min_qp		= DSC_RC_RANGE_2_RANGE_MIN_QP, \
		.rc_range_parameters[2].range_max_qp		= DSC_RC_RANGE_2_RANGE_MAX_QP, \
		.rc_range_parameters[2].range_bpg_offset	= DSC_RC_RANGE_2_RANGE_BPG_OFFSET, \
		.rc_range_parameters[3].range_min_qp		= DSC_RC_RANGE_3_RANGE_MIN_QP, \
		.rc_range_parameters[3].range_max_qp		= DSC_RC_RANGE_3_RANGE_MAX_QP, \
		.rc_range_parameters[3].range_bpg_offset	= DSC_RC_RANGE_3_RANGE_BPG_OFFSET, \
		.rc_range_parameters[4].range_min_qp		= DSC_RC_RANGE_4_RANGE_MIN_QP, \
		.rc_range_parameters[4].range_max_qp		= DSC_RC_RANGE_4_RANGE_MAX_QP, \
		.rc_range_parameters[4].range_bpg_offset	= DSC_RC_RANGE_4_RANGE_BPG_OFFSET, \
		.rc_range_parameters[5].range_min_qp		= DSC_RC_RANGE_5_RANGE_MIN_QP, \
		.rc_range_parameters[5].range_max_qp		= DSC_RC_RANGE_5_RANGE_MAX_QP, \
		.rc_range_parameters[5].range_bpg_offset	= DSC_RC_RANGE_5_RANGE_BPG_OFFSET, \
		.rc_range_parameters[6].range_min_qp		= DSC_RC_RANGE_6_RANGE_MIN_QP, \
		.rc_range_parameters[6].range_max_qp		= DSC_RC_RANGE_6_RANGE_MAX_QP, \
		.rc_range_parameters[6].range_bpg_offset	= DSC_RC_RANGE_6_RANGE_BPG_OFFSET, \
		.rc_range_parameters[7].range_min_qp		= DSC_RC_RANGE_7_RANGE_MIN_QP, \
		.rc_range_parameters[7].range_max_qp		= DSC_RC_RANGE_7_RANGE_MAX_QP, \
		.rc_range_parameters[7].range_bpg_offset	= DSC_RC_RANGE_7_RANGE_BPG_OFFSET, \
		.rc_range_parameters[8].range_min_qp		= DSC_RC_RANGE_8_RANGE_MIN_QP, \
		.rc_range_parameters[8].range_max_qp		= DSC_RC_RANGE_8_RANGE_MAX_QP, \
		.rc_range_parameters[8].range_bpg_offset	= DSC_RC_RANGE_8_RANGE_BPG_OFFSET, \
		.rc_range_parameters[9].range_min_qp		= DSC_RC_RANGE_9_RANGE_MIN_QP, \
		.rc_range_parameters[9].range_max_qp		= DSC_RC_RANGE_9_RANGE_MAX_QP, \
		.rc_range_parameters[9].range_bpg_offset	= DSC_RC_RANGE_9_RANGE_BPG_OFFSET, \
		.rc_range_parameters[10].range_min_qp		= DSC_RC_RANGE_10_RANGE_MIN_QP, \
		.rc_range_parameters[10].range_max_qp		= DSC_RC_RANGE_10_RANGE_MAX_QP, \
		.rc_range_parameters[10].range_bpg_offset	= DSC_RC_RANGE_10_RANGE_BPG_OFFSET, \
		.rc_range_parameters[11].range_min_qp		= DSC_RC_RANGE_11_RANGE_MIN_QP, \
		.rc_range_parameters[11].range_max_qp		= DSC_RC_RANGE_11_RANGE_MAX_QP, \
		.rc_range_parameters[11].range_bpg_offset	= DSC_RC_RANGE_11_RANGE_BPG_OFFSET, \
		.rc_range_parameters[12].range_min_qp		= DSC_RC_RANGE_12_RANGE_MIN_QP, \
		.rc_range_parameters[12].range_max_qp		= DSC_RC_RANGE_12_RANGE_MAX_QP, \
		.rc_range_parameters[12].range_bpg_offset	= DSC_RC_RANGE_12_RANGE_BPG_OFFSET, \
		.rc_range_parameters[13].range_min_qp		= DSC_RC_RANGE_13_RANGE_MIN_QP, \
		.rc_range_parameters[13].range_max_qp		= DSC_RC_RANGE_13_RANGE_MAX_QP, \
		.rc_range_parameters[13].range_bpg_offset	= DSC_RC_RANGE_13_RANGE_BPG_OFFSET, \
		.rc_range_parameters[14].range_min_qp		= DSC_RC_RANGE_14_RANGE_MIN_QP, \
		.rc_range_parameters[14].range_max_qp		= DSC_RC_RANGE_14_RANGE_MAX_QP, \
		.rc_range_parameters[14].range_bpg_offset	= DSC_RC_RANGE_14_RANGE_BPG_OFFSET \
	},
#else
#define PANEL_EXT_DSC_PARAM(_en)
#endif

#ifdef CONFIG_DRM_MTK_ICOM_DSI_POWER_SEQUENCE
#define PANEL_EXT_ICOM_PWR_SEQ(_on_en, _off_en) .use_icom_power_on_seq = _on_en, \
												.use_icom_power_off_seq = _off_en,
#else
#define PANEL_EXT_ICOM_PWR_SEQ(_on_en, _off_en)
#endif

#ifdef CONFIG_MTK_LCM_PHYSICAL_ROTATION_HW
#define PANEL_EXT_HW_RATATE_180(_en) .rotate = _en,
#else
#define PANEL_EXT_HW_RATATE_180(_en)
#endif

#ifdef USE_DISP_IDLE
#define PANEL_EXT_DISP_IDLE(_vfp) .vfp_low_power = _vfp,
#else
#define PANEL_EXT_DISP_IDLE(_vfp)
#endif

#ifdef USE_LFR
#define PANEL_EXT_LFR(_en, _min_fps) .lfr_enable = _en, \
									 .lfr_minimum_fps = _min_fps,
#else
#define PANEL_EXT_LFR(_en, _min_fps)
#endif

#ifdef USE_LP_PERLINE
#define PANEL_EXT_LP_PERLINE(_en) .lp_perline_en = _en,
#else
#define PANEL_EXT_LP_PERLINE(_en)
#endif

#ifdef USE_ESD_CHECK
#define PANEL_EXT_ESD_CHECK(_en) .esd_check_enable = _en,
#else
#define PANEL_EXT_ESD_CHECK(_en)
#endif

#ifdef USE_CUST_ESD
#define PANEL_EXT_CUST_ESD(_en) .cust_esd_check = _en, \
								.lcm_esd_check_table[0] = { \
									.cmd = 0x0A, \
									.count = 1, \
									.para_list[0] = 0x9C, \
								},
#else
#define PANEL_EXT_CUST_ESD(_en)
#endif

#ifdef CONFIG_DRM_MTK_ICOM_SKIP_FRAME
#define PANEL_EXT_DYN_FPS(_en, _max_fps, _base_fps, _cmd0_addr, _cmd0_val, _cmd1_addr, _cmd1_val, _cmd2_addr, _cmd2_val) \
.skip_frame_base_fps = _base_fps, \
.change_fps_by_vfp_send_cmd = (_cmd0_addr > 0) ? 1 : 0, \
.dyn_fps = { \
	.switch_en = _en, \
	.vact_timing_fps = _max_fps, \
	.dfps_cmd_table[0] = {0, 2, {_cmd0_addr, _cmd0_val} }, \
	.dfps_cmd_table[1] = {0, 2, {_cmd1_addr, _cmd1_val} }, \
	.dfps_cmd_table[2] = {0, 2, {_cmd2_addr, _cmd2_val} }, \
	.dfps_cmd_table[3] = {0, 2, {0xFF, 0x10} }, \
	.dfps_cmd_table[4] = {0, 2, {0xFB, 0x01} }, \
},
#else
#define PANEL_EXT_DYN_FPS(_en, _max_fps, _base_fps, _cmd0_addr, _cmd0_val, _cmd1_addr, _cmd1_val, _cmd2_addr, _cmd2_val) \
.change_fps_by_vfp_send_cmd = (_cmd0_addr > 0) ? 1 : 0, \
.dyn_fps = { \
	.switch_en = _en, \
	.vact_timing_fps = _max_fps, \
	.dfps_cmd_table[0] = {0, 2, {_cmd0_addr, _cmd0_val} }, \
	.dfps_cmd_table[1] = {0, 2, {_cmd1_addr, _cmd1_val} }, \
	.dfps_cmd_table[2] = {0, 2, {_cmd2_addr, _cmd2_val} }, \
	.dfps_cmd_table[3] = {0, 2, {0xFF, 0x10} }, \
	.dfps_cmd_table[4] = {0, 2, {0xFB, 0x01} }, \
},
#endif

#define _PANEL_EXT_PARAMS(_name, _datarate, _dyn_en, _hfp, _vfp, _vfp_lp, _max_fps, _base_fps, _cmd0, _data0, _cmd1, _data1, _cmd2, _data2, _lfr_fps, _dsc_en) \
struct mtk_panel_params ext_params_##_name = { \
	.pll_clk = _datarate >> 1, \
	.data_rate = _datarate, \
	.dyn = { \
		.switch_en = _dyn_en, \
		.pll_clk = _datarate >> 1, \
		.hfp = _hfp, \
		.vfp = _vfp, \
	}, \
	.physical_width_um = PHYSICAL_WIDTH_UM,\
	.physical_height_um = PHYSICAL_HEIGHT_UM,\
	PANEL_EXT_DYN_FPS(_dyn_en, _max_fps, _base_fps, _cmd0, _data0, _cmd1, _data1, _cmd2, _data2) \
	PANEL_EXT_DISP_IDLE(_vfp_lp) \
	PANEL_EXT_LFR(1, _lfr_fps) \
	PANEL_EXT_DSC_PARAM(_dsc_en) \
	PANEL_EXT_LP_PERLINE(1) \
	PANEL_EXT_ESD_CHECK(1) \
	PANEL_EXT_CUST_ESD(1) \
	PANEL_EXT_ICOM_PWR_SEQ(1, 1) \
	PANEL_EXT_HW_RATATE_180(1) \
}

#define PANEL_EXT_PARAMS_NO_DYN(_fps, _datarate, _hfp, _vfp, _vfp_lp, _lfr_fps, _dsc_en) \
_PANEL_EXT_PARAMS(_fps##hz_only, _datarate, 0, _hfp, _vfp, _vfp_lp, 0, 0, 0, 0, 0, 0, 0, 0, _lfr_fps, _dsc_en)

#define PANEL_EXT_PARAMS_120HZ_DSC_DYN(_name, _vfp, _vfp_lp, _base_fps, _cmd0, _data0, _cmd1, _data1, _cmd2, _data2, _lfr_fps) \
_PANEL_EXT_PARAMS(_name, DATA_RATE_120HZ_3X_DSC, 1, HFP_120HZ_3X_DSC, _vfp, _vfp_lp, 120, _base_fps, _cmd0, _data0, _cmd1, _data1, _cmd2, _data2, _lfr_fps, 1)

#define PANEL_EXT_PARAMS_120HZ_DSC_VFP(_fps, _vfp, _vfp_lp) \
PANEL_EXT_PARAMS_120HZ_DSC_DYN(_fps##hz_dsc_vfp, _vfp, _vfp_lp, 0, 0, 0, 0, 0, 0, 0, _fps/2)

#define PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(_fps, _vfp, _vfp_lp) \
PANEL_EXT_PARAMS_120HZ_DSC_DYN(_fps##hz_dsc_vfp_frameskip, _vfp, _vfp_lp, 0, 0xFF, 0x25, 0xF2, 0x00, 0xF3, 0x00, _fps/2)

#define PANEL_EXT_PARAMS_120HZ_DSC_FRAMESKIP(_fps, _base_fps, _cmd0, _data0, _cmd1, _data1, _cmd2, _data2) \
PANEL_EXT_PARAMS_120HZ_DSC_DYN(_fps##hz_dsc_frameskip, VFP_120HZ_3X_DSC, VFP_120HZ_3X_DSC, _base_fps, _cmd0, _data0, _cmd1, _data1, _cmd2, _data2, 0)

// ext_params_60hz_only
static PANEL_EXT_PARAMS_NO_DYN(60, DATA_RATE_60HZ_NO_DSC, HFP_60HZ_NO_DSC, VFP_60HZ_NO_DSC, VFP_60HZ_NO_DSC, 60, 0);
// ext_params_120hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(120, VFP_120HZ_3X_DSC, VFP_120HZ_3X_DSC);
// ext_params_90hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(90, VFP_120HZ_3X_DSC_VFP_90HZ, VFP_120HZ_3X_DSC_VFP_90HZ);
// ext_params_72hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(72, VFP_120HZ_3X_DSC_VFP_72HZ, VFP_120HZ_3X_DSC_VFP_72HZ);
// ext_params_60hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(60, VFP_120HZ_3X_DSC_VFP_60HZ, VFP_120HZ_3X_DSC_VFP_60HZ);
// ext_params_45hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(45, VFP_120HZ_3X_DSC_VFP_45HZ, VFP_120HZ_3X_DSC_VFP_45HZ);
// ext_params_30hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(30, VFP_120HZ_3X_DSC_VFP_30HZ, VFP_120HZ_3X_DSC_VFP_30HZ);
// ext_params_24hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(24, VFP_120HZ_3X_DSC_VFP_24HZ, VFP_120HZ_3X_DSC_VFP_24HZ);
// ext_params_15hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(15, VFP_120HZ_3X_DSC_VFP_15HZ, VFP_120HZ_3X_DSC_VFP_15HZ);
// ext_params_10hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(10, VFP_120HZ_3X_DSC_VFP_10HZ, VFP_120HZ_3X_DSC_VFP_10HZ);
// ext_params_6hz_dsc_vfp
static PANEL_EXT_PARAMS_120HZ_DSC_VFP(6, VFP_120HZ_3X_DSC_VFP_6HZ, VFP_120HZ_3X_DSC_VFP_6HZ);
// ext_params_120hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(120, VFP_120HZ_3X_DSC, VFP_120HZ_3X_DSC);
// ext_params_90hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(90, VFP_120HZ_3X_DSC_VFP_90HZ, VFP_120HZ_3X_DSC_VFP_90HZ);
// ext_params_72hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(72, VFP_120HZ_3X_DSC_VFP_72HZ, VFP_120HZ_3X_DSC_VFP_72HZ);
// ext_params_60hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(60, VFP_120HZ_3X_DSC_VFP_60HZ, VFP_120HZ_3X_DSC_VFP_60HZ);
// ext_params_45hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(45, VFP_120HZ_3X_DSC_VFP_45HZ, VFP_120HZ_3X_DSC_VFP_45HZ);
// ext_params_30hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(30, VFP_120HZ_3X_DSC_VFP_30HZ, VFP_120HZ_3X_DSC_VFP_30HZ);
// ext_params_24hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(24, VFP_120HZ_3X_DSC_VFP_24HZ, VFP_120HZ_3X_DSC_VFP_24HZ);
// ext_params_15hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(15, VFP_120HZ_3X_DSC_VFP_15HZ, VFP_120HZ_3X_DSC_VFP_15HZ);
// ext_params_10hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(10, VFP_120HZ_3X_DSC_VFP_10HZ, VFP_120HZ_3X_DSC_VFP_10HZ);
// ext_params_6hz_dsc_vfp_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_VFP_FRAMESKIP(6, VFP_120HZ_3X_DSC_VFP_6HZ, VFP_120HZ_3X_DSC_VFP_6HZ);
// ext_params_1hz_dsc_vfp_frameskip
#if (DSC_VFP_FRAMESKIP_1HZ_BASE == 10)
static PANEL_EXT_PARAMS_120HZ_DSC_DYN(1hz_dsc_vfp_frameskip, VFP_120HZ_3X_DSC_VFP_10HZ, VFP_120HZ_3X_DSC_VFP_10HZ, 10, 0xFF, 0x25, 0xF2,  9, 0xF3, 1, 0);
#elif (DSC_VFP_FRAMESKIP_1HZ_BASE == 24)
static PANEL_EXT_PARAMS_120HZ_DSC_DYN(1hz_dsc_vfp_frameskip, VFP_120HZ_3X_DSC_VFP_24HZ, VFP_120HZ_3X_DSC_VFP_24HZ, 24, 0xFF, 0x25, 0xF2, 23, 0xF3, 1, 0);
#else
static PANEL_EXT_PARAMS_120HZ_DSC_DYN(1hz_dsc_vfp_frameskip, VFP_120HZ_3X_DSC_VFP_30HZ, VFP_120HZ_3X_DSC_VFP_30HZ, 30, 0xFF, 0x25, 0xF2, 29, 0xF3, 1, 0);
#endif
// ext_params_60hz_dsc_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_FRAMESKIP(60, 120, 0xFF, 0x25, 0xF2, 0x01, 0xF3, 0x01);
// ext_params_30hz_dsc_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_FRAMESKIP(30, 120, 0xFF, 0x25, 0xF2, 0x03, 0xF3, 0x01);
// ext_params_10hz_dsc_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_FRAMESKIP(10, 120, 0xFF, 0x25, 0xF2, 0x0B, 0xF3, 0x01);
// ext_params_1hz_dsc_frameskip
static PANEL_EXT_PARAMS_120HZ_DSC_FRAMESKIP(1, 120, 0xFF, 0x25, 0xF2, 0x75, 0xF3, 0x03);

static struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct sharp *ctx = panel_to_sharp(panel);
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	int ret = 0, fps_mode;

	LCM_DEV_LOGI("+++");

	if (m == NULL) {
		LCM_DEV_LOGE("invalid display_mode\n");
		return -1;
	}

	fps_mode = drm_mode_vrefresh(m);

	if (ctx->lcm_config & FRAMESKIP_CASE) {
		if (fps_mode == 60) {
			if (ctx->lcm_config & FPS_60_VFP)
				ext->params = &ext_params_60hz_dsc_vfp_frameskip;
			else if (ctx->lcm_config & FPS_60_FRAMESKIP)
				ext->params = &ext_params_60hz_dsc_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 120) {
			if (ctx->lcm_config & (FPS_120_VFP | FPS_120_FRAMESKIP))
				ext->params = &ext_params_120hz_dsc_vfp_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 1) {
			if (ctx->lcm_config & FPS_1_VFP_FRAMESKIP)
				ext->params = &ext_params_1hz_dsc_vfp_frameskip;
			else if (ctx->lcm_config & FPS_1_FRAMESKIP)
				ext->params = &ext_params_1hz_dsc_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 6) {
			if (ctx->lcm_config & FPS_6_VFP)
				ext->params = &ext_params_6hz_dsc_vfp_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 10) {
			if (ctx->lcm_config & FPS_10_VFP)
				ext->params = &ext_params_10hz_dsc_vfp_frameskip;
			else if (ctx->lcm_config & FPS_10_FRAMESKIP)
				ext->params = &ext_params_10hz_dsc_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 15) {
			if (ctx->lcm_config & FPS_15_VFP)
				ext->params = &ext_params_15hz_dsc_vfp_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 24) {
			if (ctx->lcm_config & FPS_24_VFP)
				ext->params = &ext_params_24hz_dsc_vfp_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 30) {
			if (ctx->lcm_config & FPS_30_VFP)
				ext->params = &ext_params_30hz_dsc_vfp_frameskip;
			else if (ctx->lcm_config & FPS_30_FRAMESKIP)
				ext->params = &ext_params_30hz_dsc_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 45) {
			if (ctx->lcm_config & FPS_45_VFP)
				ext->params = &ext_params_45hz_dsc_vfp_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 72) {
			if (ctx->lcm_config & FPS_72_VFP)
				ext->params = &ext_params_72hz_dsc_vfp_frameskip;
			else
				ret = 1;
		} else if (fps_mode == 90) {
			if (ctx->lcm_config & FPS_90_VFP)
				ext->params = &ext_params_90hz_dsc_vfp_frameskip;
			else
				ret = 1;
		} else {
			ret = 1;
		}
	} else {
		if (fps_mode == 60) {
			ext->params = &ext_params_60hz_dsc_vfp;
		} else if (fps_mode == 120) {
			ext->params = &ext_params_120hz_dsc_vfp;
		} else if (fps_mode == 6) {
			ext->params = &ext_params_6hz_dsc_vfp;
		} else if (fps_mode == 10) {
			ext->params = &ext_params_10hz_dsc_vfp;
		} else if (fps_mode == 15) {
			ext->params = &ext_params_15hz_dsc_vfp;
		} else if (fps_mode == 24) {
			ext->params = &ext_params_24hz_dsc_vfp;
		} else if (fps_mode == 30) {
			ext->params = &ext_params_30hz_dsc_vfp;
		} else if (fps_mode == 45) {
			ext->params = &ext_params_45hz_dsc_vfp;
		} else if (fps_mode == 72) {
			ext->params = &ext_params_72hz_dsc_vfp;
		} else if (fps_mode == 90) {
			ext->params = &ext_params_90hz_dsc_vfp;
		} else {
			ret = 1;
		}
	}

	if (!ret) {
		current_fps = fps_mode;
		LCM_DEV_LOGI("current_fps: %d\n", current_fps);
	}

	LCM_DEV_LOGD("---");
	return ret;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct sharp *ctx = panel_to_sharp(panel);

	LCM_DEV_LOGI("+++");

	gpiod_set_value(ctx->reset_gpio, on);

	LCM_DEV_LOGD("---");
	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
#ifdef CONFIG_DRM_MTK_ICOM_DSI_POWER_SEQUENCE
	.panel_lp_enable = sharp_panel_init,
	.panel_lp_disable = sharp_display_off,
#endif
#ifdef CONFIG_DRM_MTK_ICOM_LCM_SET_DISPLAY_ON_DELAY
	.display_on = sharp_display_on,
	.display_on_get_state = sharp_display_on_get_state,
#endif
	.ext_param_set = mtk_panel_ext_param_set,
	.reset = panel_ext_reset,
};
#endif /* CONFIG_MTK_PANEL_EXT */

static int sharp_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct sharp *ctx = panel_to_sharp(panel);
	struct drm_display_mode *mode;
	int booton_fps;

	LCM_DEV_LOGI("+++");

	if (ctx->lcm_config & BOOT_ON_60HZ)
		booton_fps = 60;
	else if (ctx->lcm_config & (FPS_120_VFP | FPS_120_FRAMESKIP))
		booton_fps = 120;
	else if (ctx->lcm_config & (FPS_90_VFP))
		booton_fps = 90;
	else if (ctx->lcm_config & (FPS_72_VFP))
		booton_fps = 72;
	else if (ctx->lcm_config & (FPS_60_VFP | FPS_60_FRAMESKIP))
		booton_fps = 60;
	else if (ctx->lcm_config & (FPS_45_VFP))
		booton_fps = 45;
	else if (ctx->lcm_config & (FPS_30_VFP | FPS_30_FRAMESKIP))
		booton_fps = 30;
	else if (ctx->lcm_config & (FPS_24_VFP))
		booton_fps = 24;
	else if (ctx->lcm_config & (FPS_15_VFP))
		booton_fps = 15;
	else if (ctx->lcm_config & (FPS_10_VFP | FPS_10_FRAMESKIP))
		booton_fps = 10;
	else if (ctx->lcm_config & (FPS_6_VFP))
		booton_fps = 6;
	else if (ctx->lcm_config & (FPS_1_VFP_FRAMESKIP | FPS_1_FRAMESKIP))
		booton_fps = 1;


	if (ctx->lcm_config & (FPS_120_VFP | FPS_120_FRAMESKIP)) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_120hz_dsc);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_120hz_dsc.hdisplay, display_mode_120hz_dsc.vdisplay,
				drm_mode_vrefresh(&display_mode_120hz_dsc));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 120)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (ctx->lcm_config & FPS_90_VFP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_90hz_dsc_vfp);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_90hz_dsc_vfp.hdisplay, display_mode_90hz_dsc_vfp.vdisplay,
				drm_mode_vrefresh(&display_mode_90hz_dsc_vfp));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 90)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (ctx->lcm_config & FPS_72_VFP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_72hz_dsc_vfp);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_72hz_dsc_vfp.hdisplay, display_mode_72hz_dsc_vfp.vdisplay,
				drm_mode_vrefresh(&display_mode_72hz_dsc_vfp));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 72)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (!(ctx->lcm_config & PARAMS_DSC)) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_60hz_no_dsc);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_60hz_no_dsc.hdisplay, display_mode_60hz_no_dsc.vdisplay,
				drm_mode_vrefresh(&display_mode_60hz_no_dsc));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 60)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	} else if (ctx->lcm_config & FPS_60_VFP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_60hz_dsc_vfp);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_60hz_dsc_vfp.hdisplay, display_mode_60hz_dsc_vfp.vdisplay,
				drm_mode_vrefresh(&display_mode_60hz_dsc_vfp));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 60)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	} else if (ctx->lcm_config & FPS_60_FRAMESKIP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_60hz_dsc_frameskip);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_60hz_dsc_frameskip.hdisplay, display_mode_60hz_dsc_frameskip.vdisplay,
				drm_mode_vrefresh(&display_mode_60hz_dsc_frameskip));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 60)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (ctx->lcm_config & FPS_45_VFP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_45hz_dsc_vfp);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_45hz_dsc_vfp.hdisplay, display_mode_45hz_dsc_vfp.vdisplay,
				drm_mode_vrefresh(&display_mode_45hz_dsc_vfp));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 45)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (ctx->lcm_config & FPS_30_VFP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_30hz_dsc_vfp);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_30hz_dsc_vfp.hdisplay, display_mode_30hz_dsc_vfp.vdisplay,
				drm_mode_vrefresh(&display_mode_30hz_dsc_vfp));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 30)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	} else if (ctx->lcm_config & FPS_30_FRAMESKIP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_30hz_dsc_frameskip);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_30hz_dsc_frameskip.hdisplay, display_mode_30hz_dsc_frameskip.vdisplay,
				drm_mode_vrefresh(&display_mode_30hz_dsc_frameskip));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 30)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (ctx->lcm_config & FPS_24_VFP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_24hz_dsc_vfp);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_24hz_dsc_vfp.hdisplay, display_mode_24hz_dsc_vfp.vdisplay,
				drm_mode_vrefresh(&display_mode_24hz_dsc_vfp));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 24)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (ctx->lcm_config & FPS_15_VFP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_15hz_dsc_vfp);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_15hz_dsc_vfp.hdisplay, display_mode_15hz_dsc_vfp.vdisplay,
				drm_mode_vrefresh(&display_mode_15hz_dsc_vfp));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 15)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (ctx->lcm_config & FPS_10_VFP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_10hz_dsc_vfp);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_10hz_dsc_vfp.hdisplay, display_mode_10hz_dsc_vfp.vdisplay,
				drm_mode_vrefresh(&display_mode_10hz_dsc_vfp));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 10)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	} else if (ctx->lcm_config & FPS_10_FRAMESKIP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_10hz_dsc_frameskip);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_10hz_dsc_frameskip.hdisplay, display_mode_10hz_dsc_frameskip.vdisplay,
				drm_mode_vrefresh(&display_mode_10hz_dsc_frameskip));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 10)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (ctx->lcm_config & FPS_6_VFP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_6hz_dsc_vfp);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_6hz_dsc_vfp.hdisplay, display_mode_6hz_dsc_vfp.vdisplay,
				drm_mode_vrefresh(&display_mode_6hz_dsc_vfp));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 6)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}


	if (ctx->lcm_config & FPS_1_VFP_FRAMESKIP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_1hz_dsc_vfp_frameskip);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_1hz_dsc_vfp_frameskip.hdisplay, display_mode_1hz_dsc_vfp_frameskip.vdisplay,
				drm_mode_vrefresh(&display_mode_1hz_dsc_vfp_frameskip));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 1)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	} else if (ctx->lcm_config & FPS_1_FRAMESKIP) {
		mode = drm_mode_duplicate(connector->dev, &display_mode_1hz_dsc_frameskip);
		if (!mode) {
			LCM_DEV_LOGE("failed to add mode %ux%ux@%u\n",
				display_mode_1hz_dsc_frameskip.hdisplay, display_mode_1hz_dsc_frameskip.vdisplay,
				drm_mode_vrefresh(&display_mode_1hz_dsc_frameskip));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		if (booton_fps == 1)
			mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		else
			mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode = NULL;
	}

	connector->display_info.width_mm = PHYSICAL_WIDTH_UM / 1000;
	connector->display_info.height_mm = PHYSICAL_HEIGHT_UM / 1000;

	LCM_DEV_LOGD("---");

	return 1;
}

static const struct drm_panel_funcs sharp_drm_funcs = {
	.disable = sharp_disable,
	.unprepare = sharp_unprepare,
	.prepare = sharp_prepare,
	.enable = sharp_enable,
	.get_modes = sharp_get_modes,
};

static ssize_t sharp_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sharp *ctx = dev_get_drvdata(dev);
	char *s = buf;

	s += sprintf(s, "Found: %d , ", ctx->islcmfound);

	switch (ctx->lcm_config & SAMPLE_MASK) {
	case SAMPLE_MP:
		s += sprintf(s, "Panel: MP sample\n");
		break;
	case SAMPLE_CS:
		s += sprintf(s, "Panel: CS sample\n");
		break;
	case SAMPLE_ES3:
		s += sprintf(s, "Panel: ES3 sample\n");
		break;
	case SAMPLE_ES2_2:
		s += sprintf(s, "Panel: ES2-2 sample\n");
		break;
	case SAMPLE_ES2_1:
		s += sprintf(s, "Panel: ES2-1 sample\n");
		break;
	case SAMPLE_ES:
		s += sprintf(s, "Panel: ES sample\n");
		break;
	case SAMPLE_PRE_ES:
		s += sprintf(s, "Panel: Pre ES sample\n");
		break;
	case SAMPLE_TS:
		s += sprintf(s, "Panel: TS sample\n");
		break;
	case SAMPLE_PRE_TS:
	default:
		s += sprintf(s, "Panel: Pre TS sample\n");
		break;
	}

	return (s - buf);
}
static DEVICE_ATTR(state, 0444, sharp_state_show, NULL);

static int sharp_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct sharp *ctx;
	int ret;

	LCM_LOGI("+++\n");

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				LCM_LOGE("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			LCM_LOGI("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		LCM_LOGI("--- skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct sharp), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET;

#ifdef USE_DSI_CLOCK_NON_CONTINUOUS
	dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;
#endif

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		LCM_DEV_LOGE("cannot get reset-gpios %ld\n", PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	of_property_read_u32(dev->of_node, "sample-id1", &ctx->sample_id1);
	LCM_DEV_LOGI("sample-id1 = 0x%02x\n", ctx->sample_id1);

	of_property_read_u32(dev->of_node, "sample-id2", &ctx->sample_id2);
	LCM_DEV_LOGI("sample-id2 = 0x%02x\n", ctx->sample_id2);

	of_property_read_u32(dev->of_node, "sample-id3", &ctx->sample_id3);
	LCM_DEV_LOGI("sample-id3 = 0x%02x\n", ctx->sample_id3);

	of_property_read_u32(dev->of_node, "islcmfound", &ctx->islcmfound);
	LCM_DEV_LOGI("islcmfound = 0x%02x\n", ctx->islcmfound);

	sharp_check_sample_id(ctx);

	ctx->disp_vddi = devm_regulator_get_optional(dev, "vddi");
	if (IS_ERR(ctx->disp_vddi)) {
		ret = PTR_ERR(ctx->disp_vddi);
		LCM_DEV_LOGE("get vddi fail, error: %d\n", ret);
		ctx->disp_vddi = NULL;
	}

	if (ctx->disp_vddi) {
		ret = regulator_enable(ctx->disp_vddi);
		if (ret < 0) {
			LCM_DEV_LOGE("regulator_enable vddi fail: %d\n", ret);
		}
	}

	ctx->disp_bias_pos = devm_regulator_get_optional(dev, "vpos");
	if (IS_ERR(ctx->disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(ctx->disp_bias_pos);
		LCM_DEV_LOGE("get vpos fail, error: %d\n", ret);
		ctx->disp_bias_pos = NULL;
	}

	ctx->disp_bias_neg = devm_regulator_get_optional(dev, "vneg");
	if (IS_ERR(ctx->disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(ctx->disp_bias_neg);
		LCM_DEV_LOGE("get vneg fail, error: %d\n", ret);
		ctx->disp_bias_neg = NULL;
	}

	if (ctx->disp_bias_pos && ctx->disp_bias_neg) {
		switch (ctx->lcm_config & SAMPLE_MASK) {
		case SAMPLE_MP:
		case SAMPLE_CS:
		case SAMPLE_ES3:
		case SAMPLE_ES2_2:
		case SAMPLE_ES2_1:
		case SAMPLE_ES:
		case SAMPLE_PRE_ES:
		case SAMPLE_TS:
			bias_tps65132_enable(ctx, 5400000, 5400000);
			break;

		case SAMPLE_PRE_TS:
		default:
			bias_tps65132_enable(ctx, 6000000, 6000000);
			break;
		}
	}

#ifndef CONFIG_MTK_DISP_NO_LK
	ctx->prepared = true;
	ctx->enabled = true;
#ifdef CONFIG_DRM_MTK_ICOM_LCM_SET_DISPLAY_ON_DELAY
	ctx->display_on = true;
#endif
#endif

	drm_panel_init(&ctx->panel, dev, &sharp_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);

	if (!(ctx->lcm_config & PARAMS_DSC))
		ret = mtk_panel_ext_create(dev, &ext_params_60hz_only, &ext_funcs, &ctx->panel);
	else if ((ctx->lcm_config & BOOT_ON_60HZ) && (ctx->lcm_config & FPS_60_VFP))
		ret = mtk_panel_ext_create(dev, &ext_params_60hz_dsc_vfp, &ext_funcs, &ctx->panel);
	else if ((ctx->lcm_config & BOOT_ON_60HZ) && (ctx->lcm_config & FPS_60_FRAMESKIP))
		ret = mtk_panel_ext_create(dev, &ext_params_60hz_dsc_frameskip, &ext_funcs, &ctx->panel);
	else if (ctx->lcm_config & FRAMESKIP_CASE)
		ret = mtk_panel_ext_create(dev, &ext_params_120hz_dsc_vfp_frameskip, &ext_funcs, &ctx->panel);
	else
		ret = mtk_panel_ext_create(dev, &ext_params_120hz_dsc_vfp, &ext_funcs, &ctx->panel);

	if (ret < 0)
		return ret;
#endif

	device_create_file(ctx->dev, &dev_attr_state);

	LCM_DEV_LOGI("---");

	return ret;
}

static int sharp_remove(struct mipi_dsi_device *dsi)
{
	struct sharp *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id sharp_of_match[] = {
	{
		.compatible = SHARP_NT36523N_COMPATIBLE,
	},
	{}
};

MODULE_DEVICE_TABLE(of, sharp_of_match);

static struct mipi_dsi_driver sharp_driver = {
	.probe = sharp_probe,
	.remove = sharp_remove,
	.driver = {
		.name = SHARP_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sharp_of_match,
	},
};

module_mipi_dsi_driver(sharp_driver);

MODULE_AUTHOR("mike chang <mike.chang@innocomm.com>");
MODULE_DESCRIPTION("SHARP NT36523N VDO 120HZ Panel Driver");
MODULE_LICENSE("GPL v2");

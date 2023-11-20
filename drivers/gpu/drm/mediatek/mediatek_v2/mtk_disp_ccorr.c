// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#if defined(CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE) && IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif
#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
//#define CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
#endif
//#define CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG

#ifndef DRM_CMDQ_DISABLE
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#else
#include "mtk-cmdq-ext.h"
#endif

#include "mtk_drm_crtc.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_drv.h"
#include "mtk_disp_ccorr.h"
#include "mtk_disp_color.h"
#include "mtk_log.h"
#include "mtk_dump.h"
#include "mtk_drm_helper.h"
#include "platform/mtk_drm_6789.h"

#ifdef CONFIG_LEDS_MTK_MODULE
//#define CONFIG_LEDS_BRIGHTNESS_CHANGED
#include <linux/leds-mtk.h>
#else
#define mtk_leds_brightness_set(x, y) do { } while (0)
#endif

#define DISP_REG_CCORR_EN (0x000)
#define DISP_REG_CCORR_INTEN                     (0x008)
#define DISP_REG_CCORR_INTSTA                    (0x00C)
#define DISP_REG_CCORR_CFG (0x020)
#define DISP_REG_CCORR_SIZE (0x030)
#define DISP_REG_CCORR_COLOR_OFFSET_0	(0x100)
#define DISP_REG_CCORR_COLOR_OFFSET_1	(0x104)
#define DISP_REG_CCORR_COLOR_OFFSET_2	(0x108)
#define CCORR_COLOR_OFFSET_MASK	(0x3FFFFFF)

#define DISP_REG_CCORR_SHADOW (0x0A0)
#define CCORR_READ_WORKING		BIT(0)
#define CCORR_BYPASS_SHADOW		BIT(2)

#define CCORR_12BIT_MASK				0x0fff
#define CCORR_13BIT_MASK				0x1fff

#define CCORR_INVERSE_GAMMA   (0)
#define CCORR_BYASS_GAMMA      (1)

#define CCORR_REG(idx) (idx * 4 + 0x80)
#define CCORR_CLIP(val, min, max) (((val) >= (max)) ? \
	(max) : (((val) <= (min)) ? (min) : (val)))
#define CCORR_ABS(val) (((val) >= 0) ? (val) : (-(val)))

static unsigned int g_ccorr_8bit_switch[DISP_CCORR_TOTAL];
static unsigned int g_ccorr_relay_value[DISP_CCORR_TOTAL];
//#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
static unsigned int g_ccorr_size[DISP_CCORR_TOTAL];
//#endif

struct drm_mtk_ccorr_caps disp_ccorr_caps;
static int ccorr_offset_base = 1024;
static int ccorr_max_negative = -2048;
static int ccorr_max_positive = 2047;
static int ccorr_fullbit_mask = 0x0fff;
static int ccorr_offset_mask = 14;
unsigned int disp_ccorr_number;
unsigned int disp_ccorr_linear;
bool disp_aosp_ccorr;
static bool g_prim_ccorr_force_linear;
static bool g_prim_ccorr_pq_nonlinear;
static bool g_is_aibld_cv_mode;

#define index_of_ccorr(module) ((module == DDP_COMPONENT_CCORR0) ? 0 : \
		((module == DDP_COMPONENT_CCORR1) ? 1 : \
		((module == DDP_COMPONENT_CCORR2) ? 2 : 3)))

#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
static bool bypass_color0, bypass_color1;
#endif

static atomic_t g_ccorr_is_clock_on[DISP_CCORR_TOTAL] = {
	ATOMIC_INIT(0), ATOMIC_INIT(0), ATOMIC_INIT(0), ATOMIC_INIT(0) };

static atomic_t g_irq_backlight_change = ATOMIC_INIT(0);

#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
static struct DRM_DISP_CCORR_COEF_T *g_disp_ccorr_coef[DISP_CCORR_TOTAL] = {
	NULL };
static int g_ccorr_color_matrix[DISP_CCORR_TOTAL][3][3] = {
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	},
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	},
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	},
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	} };
static int g_ccorr_prev_matrix[DISP_CCORR_TOTAL][3][3] = {
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	},
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	},
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	},
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	} };
static int g_rgb_matrix[DISP_CCORR_TOTAL][3][3] = {
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	},
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	},
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	},
	{
		{1024, 0, 0},
		{0, 1024, 0},
		{0, 0, 1024}
	} };
static struct DRM_DISP_CCORR_COEF_T g_multiply_matrix_coef;
#endif /* !CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE */
static int g_disp_ccorr_without_gamma;
static int g_disp_ccorr_temp_linear;

static DECLARE_WAIT_QUEUE_HEAD(g_ccorr_get_irq_wq);
//static DEFINE_SPINLOCK(g_ccorr_get_irq_lock);
static DEFINE_SPINLOCK(g_ccorr_clock_lock);
static atomic_t g_ccorr_get_irq = ATOMIC_INIT(0);

/* FOR TRANSITION */
static DEFINE_SPINLOCK(g_pq_bl_change_lock);
static int g_old_pq_backlight;
static int g_pq_backlight;
static int g_pq_backlight_db;
static atomic_t g_ccorr_is_init_valid = ATOMIC_INIT(0);

static DEFINE_MUTEX(g_ccorr_global_lock);
// For color conversion bug fix
//static bool need_offset;
#define OFFSET_VALUE (1024)

/* TODO */
/* static ddp_module_notify g_ccorr_ddp_notify; */

// It's a work around for no comp assigned in functions.
static struct mtk_ddp_comp *default_comp;
static struct mtk_ddp_comp *ccorr1_default_comp;

static int disp_ccorr_write_coef_reg(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, int lock);
/* static void ccorr_dump_reg(void); */

enum CCORR_IOCTL_CMD {
	SET_CCORR = 0,
	SET_INTERRUPT,
	BYPASS_CCORR
};

struct mtk_disp_ccorr {
	struct mtk_ddp_comp ddp_comp;
	struct drm_crtc *crtc;
	const struct mtk_disp_ccorr_data *data;
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	int stored_color_matrix_rgb[3];
	int grayscale_color_matrix[3][3];
	struct DRM_DISP_CCORR_COEF_T grayscale_ccorr_coef;
#endif
};

static inline struct mtk_disp_ccorr *comp_to_ccorr(struct mtk_ddp_comp *comp)
{
	return container_of(comp, struct mtk_disp_ccorr, ddp_comp);
}

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
static int disp_ccorr_grayscale_calc(struct mtk_disp_ccorr *disp_ccorr,
		int color_matrix_r, int color_matrix_g, int color_matrix_b)
{
	int i;
	int coef_rgb[3], sum, coef;

	disp_ccorr->stored_color_matrix_rgb[0] = color_matrix_r;
	disp_ccorr->stored_color_matrix_rgb[1] = color_matrix_g;
	disp_ccorr->stored_color_matrix_rgb[2] = color_matrix_b;

	if (CCORR_ABS(disp_ccorr->stored_color_matrix_rgb[0]) > 20000 ||
			CCORR_ABS(disp_ccorr->stored_color_matrix_rgb[1]) > 20000 ||
			CCORR_ABS(disp_ccorr->stored_color_matrix_rgb[2]) > 20000) {
		pr_err("%s: ERR: -20000 <= (r'=%d)/(g'=%d)/(b'=%d) <= 20000\n", __func__,
			disp_ccorr->stored_color_matrix_rgb[0],
			disp_ccorr->stored_color_matrix_rgb[1],
			disp_ccorr->stored_color_matrix_rgb[2]);
		return -EINVAL;
	}

	sum = disp_ccorr->stored_color_matrix_rgb[0] +
			disp_ccorr->stored_color_matrix_rgb[1] +
			disp_ccorr->stored_color_matrix_rgb[2];
	if (sum <= 0) {
		pr_err("%s: ERR: ((r'=%d) + (g'=%d) + (b'=%d) = %d) > 0\n", __func__,
			disp_ccorr->stored_color_matrix_rgb[0],
			disp_ccorr->stored_color_matrix_rgb[1],
			disp_ccorr->stored_color_matrix_rgb[2],
			sum);
		return -EINVAL;
	}

	coef = DIV_ROUND_CLOSEST(color_matrix_r * ccorr_offset_base, 10000);
	coef_rgb[0] = CCORR_CLIP(coef, ccorr_max_negative, ccorr_max_positive);
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
	pr_info("%s: COEF[0] R = %d -> %d\n", __func__, coef, coef_rgb[0]);
#endif

	coef = DIV_ROUND_CLOSEST(color_matrix_g * ccorr_offset_base, 10000);
	coef_rgb[1] = CCORR_CLIP(coef, ccorr_max_negative, ccorr_max_positive);
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
	pr_info("%s: COEF[1] G = %d -> %d\n", __func__, coef, coef_rgb[1]);
#endif

	coef = DIV_ROUND_CLOSEST(color_matrix_b * ccorr_offset_base, 10000);
	coef_rgb[2] = CCORR_CLIP(coef, ccorr_max_negative, ccorr_max_positive);
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
	pr_info("%s: COEF[2] B = %d -> %d\n", __func__, coef, coef_rgb[2]);
#endif

	coef = coef_rgb[0] + coef_rgb[1] + coef_rgb[2];
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
	pr_info("%s: COEF SUM = %d (<-> base %d)\n", __func__, coef, ccorr_offset_base);
#endif
	/* 1.0 -> 2048 */
	if (sum == 10000 && coef != ccorr_offset_base) {
		int i, smallest = 0, largest = 0;
		int max, min;

		max = coef_rgb[0];
		min = coef_rgb[0];
		for (i = 1 ; i < 3 ; i++) {
			if (coef_rgb[i] < min) {
				min = coef_rgb[i];
				smallest = i;
			}
			if (coef_rgb[i] > max) {
				max = coef_rgb[i];
				largest = i;
			}
		}

		if (coef > ccorr_offset_base) {
			coef_rgb[largest] -= (coef - ccorr_offset_base);
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
			pr_info("%s: (largest) COEF[%d] - (%d) = %d\n", __func__,
					largest, coef - ccorr_offset_base, coef_rgb[largest]);
#endif
		} else {
			coef_rgb[smallest] += (ccorr_offset_base - coef);
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
			pr_info("%s: (smallest) COEF[%d] - (%d) = %d\n", __func__,
					smallest, ccorr_offset_base - coef, coef_rgb[smallest]);
#endif
		}
	}

	for (i = 0 ; i < 3 ; i++) {
		disp_ccorr->grayscale_color_matrix[0][i] = color_matrix_r;
		disp_ccorr->grayscale_color_matrix[1][i] = color_matrix_g;
		disp_ccorr->grayscale_color_matrix[2][i] = color_matrix_b;

		disp_ccorr->grayscale_ccorr_coef.coef[i][0] = ((u32)coef_rgb[0]) & ccorr_fullbit_mask;
		disp_ccorr->grayscale_ccorr_coef.coef[i][1] = ((u32)coef_rgb[1]) & ccorr_fullbit_mask;
		disp_ccorr->grayscale_ccorr_coef.coef[i][2] = ((u32)coef_rgb[2]) & ccorr_fullbit_mask;
	}

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	for (i = 0; i < 3; i++) {
		pr_info("grayscaleColorMatrix[%d][0-2] = {%c%d.%04d, %c%d.%04d, %c%d.%04d}\n", i,
			disp_ccorr->grayscale_color_matrix[i][0] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][0]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][0]) % 10000,
			disp_ccorr->grayscale_color_matrix[i][1] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][1]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][1]) % 10000,
			disp_ccorr->grayscale_color_matrix[i][2] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][2]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][2]) % 10000);
	}
	for (i = 0; i < 3; i += 1) {
		pr_info("grayscaleCCORRcoef[%d][0-2] = {0x%04x, 0x%04x, 0x%04x} = {%04d, %04d, %04d}\n", i,
			disp_ccorr->grayscale_ccorr_coef.coef[i][0],
			disp_ccorr->grayscale_ccorr_coef.coef[i][1],
			disp_ccorr->grayscale_ccorr_coef.coef[i][2],
			coef_rgb[0], coef_rgb[1], coef_rgb[2]);
	}
#else
	pr_info("grayscaleColorMatrix: %04d %04d %04d\n",
			disp_ccorr->grayscale_color_matrix[0][0],
			disp_ccorr->grayscale_color_matrix[1][1],
			disp_ccorr->grayscale_color_matrix[2][2]);
	pr_info("grayscaleCCORRcoef: {0x%04x, 0x%04x, 0x%04x} = {%04d, %04d, %04d}\n",
			disp_ccorr->grayscale_ccorr_coef.coef[0][0],
			disp_ccorr->grayscale_ccorr_coef.coef[0][1],
			disp_ccorr->grayscale_ccorr_coef.coef[0][2],
			coef_rgb[0], coef_rgb[1], coef_rgb[2]);
#endif

	return 0;
}
#endif /* CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE */

#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
static void disp_ccorr_multiply_3x3(unsigned int ccorrCoef[3][3],
	int color_matrix[3][3], unsigned int resultCoef[3][3])
{
	int temp_Result;
	int signedCcorrCoef[3][3];
	int i, j;

	/* convert unsigned 12 bit ccorr coefficient to signed 12 bit format */
	for (i = 0; i < 3; i += 1) {
		for (j = 0; j < 3; j += 1) {
			if (ccorrCoef[i][j] > ccorr_max_positive) {
				signedCcorrCoef[i][j] =
					(int)ccorrCoef[i][j] - (ccorr_offset_base<<2);
			} else {
				signedCcorrCoef[i][j] =
					(int)ccorrCoef[i][j];
			}
		}
	}

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
	for (i = 0; i < 3; i += 1) {
		pr_info("signedCcorrCoef[%d][0-2] = {%d, %d, %d}\n", i,
			signedCcorrCoef[i][0],
			signedCcorrCoef[i][1],
			signedCcorrCoef[i][2]);
	}
#endif

	temp_Result = (int)((signedCcorrCoef[0][0]*color_matrix[0][0] +
		signedCcorrCoef[0][1]*color_matrix[1][0] +
		signedCcorrCoef[0][2]*color_matrix[2][0]) / ccorr_offset_base);
	resultCoef[0][0] = CCORR_CLIP(temp_Result, ccorr_max_negative, ccorr_max_positive) &
		ccorr_fullbit_mask;

	temp_Result = (int)((signedCcorrCoef[0][0]*color_matrix[0][1] +
		signedCcorrCoef[0][1]*color_matrix[1][1] +
		signedCcorrCoef[0][2]*color_matrix[2][1]) / ccorr_offset_base);
	resultCoef[0][1] = CCORR_CLIP(temp_Result, ccorr_max_negative, ccorr_max_positive) &
		ccorr_fullbit_mask;

	temp_Result = (int)((signedCcorrCoef[0][0]*color_matrix[0][2] +
		signedCcorrCoef[0][1]*color_matrix[1][2] +
		signedCcorrCoef[0][2]*color_matrix[2][2]) / ccorr_offset_base);
	resultCoef[0][2] = CCORR_CLIP(temp_Result, ccorr_max_negative, ccorr_max_positive) &
		ccorr_fullbit_mask;

	temp_Result = (int)((signedCcorrCoef[1][0]*color_matrix[0][0] +
		signedCcorrCoef[1][1]*color_matrix[1][0] +
		signedCcorrCoef[1][2]*color_matrix[2][0]) / ccorr_offset_base);
	resultCoef[1][0] = CCORR_CLIP(temp_Result, ccorr_max_negative, ccorr_max_positive) &
		ccorr_fullbit_mask;

	temp_Result = (int)((signedCcorrCoef[1][0]*color_matrix[0][1] +
		signedCcorrCoef[1][1]*color_matrix[1][1] +
		signedCcorrCoef[1][2]*color_matrix[2][1]) / ccorr_offset_base);
	resultCoef[1][1] = CCORR_CLIP(temp_Result, ccorr_max_negative, ccorr_max_positive) &
		ccorr_fullbit_mask;

	temp_Result = (int)((signedCcorrCoef[1][0]*color_matrix[0][2] +
		signedCcorrCoef[1][1]*color_matrix[1][2] +
		signedCcorrCoef[1][2]*color_matrix[2][2]) / ccorr_offset_base);
	resultCoef[1][2] = CCORR_CLIP(temp_Result, ccorr_max_negative, ccorr_max_positive) &
		ccorr_fullbit_mask;

	temp_Result = (int)((signedCcorrCoef[2][0]*color_matrix[0][0] +
		signedCcorrCoef[2][1]*color_matrix[1][0] +
		signedCcorrCoef[2][2]*color_matrix[2][0]) / ccorr_offset_base);
	resultCoef[2][0] = CCORR_CLIP(temp_Result, ccorr_max_negative, ccorr_max_positive) &
		ccorr_fullbit_mask;

	temp_Result = (int)((signedCcorrCoef[2][0]*color_matrix[0][1] +
		signedCcorrCoef[2][1]*color_matrix[1][1] +
		signedCcorrCoef[2][2]*color_matrix[2][1]) / ccorr_offset_base);
	resultCoef[2][1] = CCORR_CLIP(temp_Result, ccorr_max_negative, ccorr_max_positive) &
		ccorr_fullbit_mask;

	temp_Result = (int)((signedCcorrCoef[2][0]*color_matrix[0][2] +
		signedCcorrCoef[2][1]*color_matrix[1][2] +
		signedCcorrCoef[2][2]*color_matrix[2][2]) / ccorr_offset_base);
	resultCoef[2][2] = CCORR_CLIP(temp_Result, ccorr_max_negative, ccorr_max_positive) &
		ccorr_fullbit_mask;

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
	for (i = 0; i < 3; i += 1) {
		pr_info("resultCoef[%d][0-2] = {0x%x, 0x%x, 0x%x}\n", i,
			resultCoef[i][0],
			resultCoef[i][1],
			resultCoef[i][2]);
	}
#endif
}
#endif

static int disp_ccorr_color_matrix_to_dispsys(struct drm_device *dev)
{
	int ret = 0;
	struct mtk_drm_private *private = dev->dev_private;

	// All Support 3*4 matrix on drm architecture
	if ((disp_ccorr_number == 1) && (disp_ccorr_linear&0x01)
		&& (!g_prim_ccorr_force_linear))
		ret = mtk_drm_helper_set_opt_by_name(private->helper_opt,
			"MTK_DRM_OPT_PQ_34_COLOR_MATRIX", 0);
	else
		ret = mtk_drm_helper_set_opt_by_name(private->helper_opt,
			"MTK_DRM_OPT_PQ_34_COLOR_MATRIX", 1);

	return ret;
}

static int disp_ccorr_write_coef_reg(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, int lock)
{
#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	struct DRM_DISP_CCORR_COEF_T *ccorr, *multiply_matrix;
#else
	struct mtk_disp_ccorr *disp_ccorr = comp_to_ccorr(comp);
	struct DRM_DISP_CCORR_COEF_T *ccorr;
#endif
	int ret = 0;
	unsigned int id = index_of_ccorr(comp->id);
#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	unsigned int temp_matrix[3][3];
#endif
	unsigned int cfg_val;
	int i, j;
	struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	if (lock)
		mutex_lock(&g_ccorr_global_lock);

#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	ccorr = g_disp_ccorr_coef[id];
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("%s:ccorr id:%d,aosp ccorr:%d,nonlinear:%d\n", __func__, id,
		disp_aosp_ccorr, g_disp_ccorr_without_gamma);
#endif
	if (ccorr == NULL) {
		DDPINFO("%s: [%d] is not initialized\n", __func__, id);
		ret = -EFAULT;
		goto ccorr_write_coef_unlock;
	}

	//if (id == 0) {
		multiply_matrix = &g_multiply_matrix_coef;
		if (((g_prim_ccorr_force_linear && (disp_ccorr_linear&0x01)) ||
			(g_prim_ccorr_pq_nonlinear && (disp_ccorr_linear == 0x0))) &&
			(disp_ccorr_number == 1)) {
			disp_ccorr_multiply_3x3(ccorr->coef, g_ccorr_color_matrix[id],
				temp_matrix);
			disp_ccorr_multiply_3x3(temp_matrix, g_rgb_matrix[id],
				multiply_matrix->coef);
		} else {
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
			for (i = 0; i < 3; i += 1) {
				pr_info("disp_ccorr_write_coef_reg:ccorr[%d][0-2] = {%d, %d, %d}\n",
						i, ccorr->coef[i][0], ccorr->coef[i][1], ccorr->coef[i][2]);
			}
#endif
			if (disp_aosp_ccorr) {
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
				for (i = 0; i < 3; i++) {
					pr_info("g_ccorr_color_matrix[%d][0-2] = {%d, %d, %d}\n",
							i,
							g_ccorr_color_matrix[id][i][0],
							g_ccorr_color_matrix[id][i][1],
							g_ccorr_color_matrix[id][i][2]);
				}
#endif
				disp_ccorr_multiply_3x3(ccorr->coef, g_ccorr_color_matrix[id],
					multiply_matrix->coef);//AOSP multiply
			} else {
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
				for (i = 0; i < 3; i++) {
					pr_info("g_rgb_matrix[%d][0-2] = {%d, %d, %d}\n",
							i,
							g_rgb_matrix[id][i][0],
							g_rgb_matrix[id][i][1],
							g_rgb_matrix[id][i][2]);
				}
#endif
				disp_ccorr_multiply_3x3(ccorr->coef, g_rgb_matrix[id],
					multiply_matrix->coef);//PQ service multiply
			}
		}
		ccorr = multiply_matrix;

		ccorr->offset[0] = g_disp_ccorr_coef[id]->offset[0];
		ccorr->offset[1] = g_disp_ccorr_coef[id]->offset[1];
		ccorr->offset[2] = g_disp_ccorr_coef[id]->offset[2];
	//}
#else
	ccorr = &disp_ccorr->grayscale_ccorr_coef;
#endif

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	for (i = 0; i < 3; i++) {
		pr_info("grayscaleColorMatrix[%d][0-2] = {%c%d.%04d, %c%d.%04d, %c%d.%04d}\n", i,
			disp_ccorr->grayscale_color_matrix[i][0] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][0]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][0]) % 10000,
			disp_ccorr->grayscale_color_matrix[i][1] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][1]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][1]) % 10000,
			disp_ccorr->grayscale_color_matrix[i][2] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][2]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][2]) % 10000);
	}
#endif
	for (i = 0; i < 3; i += 1) {
		pr_info("finalCCORRcoef[%d][0-2] = {0x%x, 0x%x, 0x%x}\n", i,
			ccorr->coef[i][0],
			ccorr->coef[i][1],
			ccorr->coef[i][2]);
	}
#endif

// For 6885 need to left shift one bit
	switch (priv->data->mmsys_id) {
	case MMSYS_MT6885:
	case MMSYS_MT6873:
	case MMSYS_MT6893:
	case MMSYS_MT6853:
	case MMSYS_MT6833:
	case MMSYS_MT6877:
	case MMSYS_MT6781:
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				ccorr->coef[i][j] = ccorr->coef[i][j]<<1;
		break;
	default:
		break;
	}

	if (handle == NULL) {
		/* use CPU to write */
		writel(1, comp->regs + DISP_REG_CCORR_EN);
		cfg_val = 0x2 | g_ccorr_relay_value[id] |
			     (g_disp_ccorr_without_gamma << 2) |
				(g_ccorr_8bit_switch[id] << 10);
		writel(cfg_val, comp->regs + DISP_REG_CCORR_CFG);
		writel(((ccorr->coef[0][0] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[0][1] & CCORR_13BIT_MASK),
			comp->regs + CCORR_REG(0));
		writel(((ccorr->coef[0][2] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][0] & CCORR_13BIT_MASK),
			comp->regs + CCORR_REG(1));
		writel(((ccorr->coef[1][1] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][2] & CCORR_13BIT_MASK),
			comp->regs + CCORR_REG(2));
		writel(((ccorr->coef[2][0] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[2][1] & CCORR_13BIT_MASK),
			comp->regs + CCORR_REG(3));
		writel(((ccorr->coef[2][2] & CCORR_13BIT_MASK) << 16),
			comp->regs + CCORR_REG(4));
		/* Ccorr Offset */
		writel(((ccorr->offset[0] & CCORR_COLOR_OFFSET_MASK) |
			(0x1 << 31)),
			comp->regs + DISP_REG_CCORR_COLOR_OFFSET_0);
		writel(((ccorr->offset[1] & CCORR_COLOR_OFFSET_MASK)),
			comp->regs + DISP_REG_CCORR_COLOR_OFFSET_1);
		writel(((ccorr->offset[2] & CCORR_COLOR_OFFSET_MASK)),
			comp->regs + DISP_REG_CCORR_COLOR_OFFSET_2);

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
		pr_info("DISP-CCORR: %s: DISP_REG_CCORR_EN=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_EN));
		pr_info("DISP-CCORR: DISP_REG_CCORR_SIZE=0x%08x<->0x%08x\n", readl(comp->regs + DISP_REG_CCORR_SIZE), g_ccorr_size[id]);
		pr_info("DISP-CCORR: DISP_REG_CCORR_CFG=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_CFG));
		pr_info("DISP-CCORR: CCORR_REG(0)=0x%08x\n", readl(comp->regs + CCORR_REG(0)));
		pr_info("DISP-CCORR: CCORR_REG(1)=0x%08x\n", readl(comp->regs + CCORR_REG(1)));
		pr_info("DISP-CCORR: CCORR_REG(2)=0x%08x\n", readl(comp->regs + CCORR_REG(2)));
		pr_info("DISP-CCORR: CCORR_REG(3)=0x%08x\n", readl(comp->regs + CCORR_REG(3)));
		pr_info("DISP-CCORR: CCORR_REG(4)=0x%08x\n", readl(comp->regs + CCORR_REG(4)));
		pr_info("DISP-CCORR: DISP_REG_CCORR_COLOR_OFFSET_0=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_COLOR_OFFSET_0));
		pr_info("DISP-CCORR: DISP_REG_CCORR_COLOR_OFFSET_1=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_COLOR_OFFSET_1));
		pr_info("DISP-CCORR: DISP_REG_CCORR_COLOR_OFFSET_2=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_COLOR_OFFSET_2));
#endif
	} else {
		/* use CMDQ to write */

		cfg_val = 0x2 | g_ccorr_relay_value[index_of_ccorr(comp->id)] |
				(g_disp_ccorr_without_gamma << 2 |
				(g_ccorr_8bit_switch[id] << 10));

		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + DISP_REG_CCORR_CFG, cfg_val, ~0);

		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(0),
			((ccorr->coef[0][0] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[0][1] & CCORR_13BIT_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(1),
			((ccorr->coef[0][2] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][0] & CCORR_13BIT_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(2),
			((ccorr->coef[1][1] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][2] & CCORR_13BIT_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(3),
			((ccorr->coef[2][0] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[2][1] & CCORR_13BIT_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(4),
			((ccorr->coef[2][2] & CCORR_13BIT_MASK) << 16), ~0);
		/* Ccorr Offset */
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + DISP_REG_CCORR_COLOR_OFFSET_0,
			(ccorr->offset[0] & CCORR_COLOR_OFFSET_MASK) |
			(0x1 << 31), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + DISP_REG_CCORR_COLOR_OFFSET_1,
			(ccorr->offset[1] & CCORR_COLOR_OFFSET_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + DISP_REG_CCORR_COLOR_OFFSET_2,
			(ccorr->offset[2] & CCORR_COLOR_OFFSET_MASK), ~0);

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
		pr_info("DISP-CCORR: %s: DISP_REG_CCORR_EN=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_EN));
		pr_info("DISP-CCORR: DISP_REG_CCORR_SIZE=0x%08x <-> 0x%08x\n", readl(comp->regs + DISP_REG_CCORR_SIZE), g_ccorr_size[id]);
		pr_info("DISP-CCORR: [Q] DISP_REG_CCORR_CFG=0x%08x\n", cfg_val);
		pr_info("DISP-CCORR: [Q] CCORR_REG(0)=0x%08x\n",
			((ccorr->coef[0][0] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[0][1] & CCORR_13BIT_MASK));
		pr_info("DISP-CCORR: [Q] CCORR_REG(1)=0x%08x\n",
			((ccorr->coef[0][2] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][0] & CCORR_13BIT_MASK));
		pr_info("DISP-CCORR: [Q] CCORR_REG(2)=0x%08x\n",
			((ccorr->coef[1][1] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][2] & CCORR_13BIT_MASK));
		pr_info("DISP-CCORR: [Q] CCORR_REG(3)=0x%08x\n",
			((ccorr->coef[2][0] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[2][1] & CCORR_13BIT_MASK));
		pr_info("DISP-CCORR: [Q] CCORR_REG(4)=0x%08x\n",
			((ccorr->coef[2][2] & CCORR_13BIT_MASK) << 16));
		pr_info("DISP-CCORR: [Q] DISP_REG_CCORR_COLOR_OFFSET_0=0x%08x\n",
			(ccorr->offset[0] & CCORR_COLOR_OFFSET_MASK) |
			(0x1 << 31));
		pr_info("DISP-CCORR: [Q] DISP_REG_CCORR_COLOR_OFFSET_1=0x%08x\n",
			(ccorr->offset[1] & CCORR_COLOR_OFFSET_MASK));
		pr_info("DISP-CCORR: [Q] DISP_REG_CCORR_COLOR_OFFSET_2=0x%08x\n",
			(ccorr->offset[2] & CCORR_COLOR_OFFSET_MASK));
#endif
	}

	DDPINFO("%s: finish\n", __func__);
#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
ccorr_write_coef_unlock:
#endif
	if (lock)
		mutex_unlock(&g_ccorr_global_lock);

	return ret;
}

void disp_ccorr_on_end_of_frame(struct mtk_ddp_comp *comp)
{
	unsigned int intsta;
	unsigned long flags;
	unsigned int index = index_of_ccorr(comp->id);

	spin_lock_irqsave(&g_ccorr_clock_lock, flags);

	if (atomic_read(&g_ccorr_is_clock_on[index]) != 1) {
		DDPINFO("%s: clock is off. enabled:%d\n", __func__, 0);

		spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
		return;
	}
	intsta = readl(comp->regs + DISP_REG_CCORR_INTSTA);

	if (intsta & 0x2) {	/* End of frame */
		// Clear irq
		writel(intsta & ~0x3, comp->regs
			+ DISP_REG_CCORR_INTSTA);

		if (index == 0) {
			atomic_set(&g_ccorr_get_irq, 1);
			wake_up_interruptible(&g_ccorr_get_irq_wq);
		}
	}
	spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
}

static void disp_ccorr_set_interrupt(struct mtk_ddp_comp *comp,
					int enabled)
{
	if (default_comp == NULL)
		default_comp = comp;

	if (!enabled && (g_old_pq_backlight != g_pq_backlight))
		g_old_pq_backlight = g_pq_backlight;
	else
		mtk_crtc_user_cmd(&(comp->mtk_crtc->base), comp,
			SET_INTERRUPT, &enabled);
}

static void disp_ccorr_clear_irq_only(struct mtk_ddp_comp *comp)
{
	unsigned int intsta;
	unsigned long flags;
	unsigned int index = index_of_ccorr(comp->id);

	DDPDBG("%s @ %d......... spin_trylock_irqsave ++ ",
		__func__, __LINE__);
	if (spin_trylock_irqsave(&g_ccorr_clock_lock, flags)) {
		DDPDBG("%s @ %d......... spin_trylock_irqsave -- ",
			__func__, __LINE__);
		if (atomic_read(&g_ccorr_is_clock_on[index]) != 1) {
			DDPINFO("%s: clock is off. enabled:%d\n", __func__, 0);

			spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
			DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
				__func__, __LINE__);
			return;
		}
		intsta = readl(comp->regs + DISP_REG_CCORR_INTSTA);

		DDPINFO("%s: intsta: 0x%x\n", __func__, intsta);

		if (intsta & 0x2) { /* End of frame */
			writel(intsta & ~0x3, comp->regs
					+ DISP_REG_CCORR_INTSTA);
		}
		spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
		DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
			__func__, __LINE__);
	} else {
		DDPINFO("%s @ %d......... Failed to spin_trylock_irqsave -- ",
			__func__, __LINE__);
	}


	/* disable interrupt */
	//disp_ccorr_set_interrupt(comp, 0);

	DDPDBG("%s @ %d......... spin_trylock_irqsave ++ ",
		__func__, __LINE__);
	if (spin_trylock_irqsave(&g_ccorr_clock_lock, flags)) {
		DDPDBG("%s @ %d......... spin_trylock_irqsave -- ",
			__func__, __LINE__);
		if (atomic_read(&g_ccorr_is_clock_on[index]) != 1) {
			DDPINFO("%s: clock is off. enabled:%d\n", __func__, 0);

			spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
			DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
				__func__, __LINE__);
			return;
		}

		{
			/* Disable output frame end interrupt */
			writel(0x0, comp->regs + DISP_REG_CCORR_INTEN);
			DDPINFO("%s: Interrupt disabled\n", __func__);
		}
			spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
			DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
				__func__, __LINE__);
	} else {
		DDPINFO("%s @ %d......... Failed to spin_trylock_irqsave -- ",
			__func__, __LINE__);
	}


}

static irqreturn_t mtk_disp_ccorr_irq_handler(int irq, void *dev_id)
{
	struct mtk_disp_ccorr *priv = dev_id;
	struct mtk_ddp_comp *ccorr = &priv->ddp_comp;

	disp_ccorr_on_end_of_frame(ccorr);

	return IRQ_HANDLED;
}

static int disp_ccorr_wait_irq(struct drm_device *dev, unsigned long timeout)
{
	int ret = 0;

	if (atomic_read(&g_ccorr_get_irq) == 0) {
		DDPDBG("%s: wait_event_interruptible ++ ", __func__);
		ret = wait_event_interruptible(g_ccorr_get_irq_wq,
			atomic_read(&g_ccorr_get_irq) == 1);
		DDPDBG("%s: wait_event_interruptible -- ", __func__);
		DDPINFO("%s: get_irq = 1, waken up", __func__);
		DDPINFO("%s: get_irq = 1, ret = %d", __func__, ret);
		if (atomic_read(&g_irq_backlight_change))
			atomic_set(&g_irq_backlight_change, 0);
	} else {
		/* If g_ccorr_get_irq is already set, */
		/* means PQService was delayed */
		DDPINFO("%s: get_irq = 0", __func__);
	}

	atomic_set(&g_ccorr_get_irq, 0);

	return ret;
}

static int disp_pq_copy_backlight_to_user(int *backlight)
{
	unsigned long flags;
	int ret = 0;

	/* We assume only one thread will call this function */
	spin_lock_irqsave(&g_pq_bl_change_lock, flags);
	g_pq_backlight_db = g_pq_backlight;
	spin_unlock_irqrestore(&g_pq_bl_change_lock, flags);

	memcpy(backlight, &g_pq_backlight_db, sizeof(int));

	DDPINFO("%s: %d\n", __func__, ret);

	return ret;
}

void disp_pq_notify_backlight_changed(int bl_1024)
{
	unsigned long flags;

	spin_lock_irqsave(&g_pq_bl_change_lock, flags);
	g_old_pq_backlight = g_pq_backlight;
	g_pq_backlight = bl_1024;
	spin_unlock_irqrestore(&g_pq_bl_change_lock, flags);

	if (atomic_read(&g_ccorr_is_init_valid) != 1)
		return;

	DDPINFO("%s: %d\n", __func__, bl_1024);

	if (m_new_pq_persist_property[DISP_PQ_CCORR_SILKY_BRIGHTNESS]) {
		if (default_comp != NULL &&
			g_ccorr_relay_value[index_of_ccorr(default_comp->id)] != 1) {
			disp_ccorr_set_interrupt(default_comp, 1);

			if (default_comp != NULL &&
					default_comp->mtk_crtc != NULL)
				mtk_crtc_check_trigger(default_comp->mtk_crtc, false,
					true);

			atomic_set(&g_irq_backlight_change, 1);
			DDPINFO("%s: trigger refresh when backlight changed", __func__);
		}
	} else {
		if (default_comp != NULL && (g_old_pq_backlight == 0 || bl_1024 == 0)) {
			disp_ccorr_set_interrupt(default_comp, 1);

			if (default_comp != NULL &&
					default_comp->mtk_crtc != NULL)
				mtk_crtc_check_trigger(default_comp->mtk_crtc, false,
					true);

			atomic_set(&g_irq_backlight_change, 1);
			DDPINFO("%s: trigger refresh when backlight ON/Off", __func__);
		}
	}
}
EXPORT_SYMBOL(disp_pq_notify_backlight_changed);

#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
static int disp_ccorr_set_coef(
	const struct DRM_DISP_CCORR_COEF_T *user_color_corr,
	struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle)
{
	int ret = 0;
	struct DRM_DISP_CCORR_COEF_T *ccorr, *old_ccorr;
	unsigned int id = index_of_ccorr(comp->id);

	ccorr = kmalloc(sizeof(struct DRM_DISP_CCORR_COEF_T), GFP_KERNEL);
	if (ccorr == NULL) {
		DDPPR_ERR("%s: no memory\n", __func__);
		return -EFAULT;
	}

	if (user_color_corr == NULL) {
		ret = -EFAULT;
		kfree(ccorr);
	} else {
		memcpy(ccorr, user_color_corr,
			sizeof(struct DRM_DISP_CCORR_COEF_T));

		if (id < DISP_CCORR_TOTAL) {
			mutex_lock(&g_ccorr_global_lock);

			old_ccorr = g_disp_ccorr_coef[id];
			g_disp_ccorr_coef[id] = ccorr;
			/* if ((g_disp_ccorr_coef[id]->offset[0] == 0) &&
				(g_disp_ccorr_coef[id]->offset[1] == 0) &&
				(g_disp_ccorr_coef[id]->offset[2] == 0) &&
				need_offset) {
				DDPINFO("%s:need change offset", __func__);
				g_disp_ccorr_coef[id]->offset[0] =
						(ccorr_offset_base << 1) << ccorr_offset_mask;
				g_disp_ccorr_coef[id]->offset[1] =
						(ccorr_offset_base << 1) << ccorr_offset_mask;
				g_disp_ccorr_coef[id]->offset[2] =
						(ccorr_offset_base << 1) << ccorr_offset_mask;
			}
			*/
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
			pr_info("%s: Set module(%d) coef", __func__, id);
#endif
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
{
			int i;
			for (i = 0; i < 3; i += 1) {
				pr_info("g_disp_ccorr_coef[%d][0-2] = {%d, %d, %d}\n",
						i,
						g_disp_ccorr_coef[id]->coef[i][0],
						g_disp_ccorr_coef[id]->coef[i][1],
						g_disp_ccorr_coef[id]->coef[i][2]);
			}
}
			pr_info("%s: disp_aosp_ccorr=%d->0", __func__, disp_aosp_ccorr);
#endif
			if (disp_aosp_ccorr)
				disp_aosp_ccorr = false;

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
			pr_info("%s: disp_aosp_ccorr=%d, g_disp_ccorr_without_gamma=%d, g_ccorr_relay_value=%d, g_ccorr_8bit_switch=%d\n",
				__func__, disp_aosp_ccorr, g_disp_ccorr_without_gamma,
				g_ccorr_relay_value[index_of_ccorr(comp->id)],
				g_ccorr_8bit_switch[index_of_ccorr(comp->id)]);
#endif
			ret = disp_ccorr_write_coef_reg(comp, handle, 0);

			mutex_unlock(&g_ccorr_global_lock);

			if (old_ccorr != NULL)
				kfree(old_ccorr);

			mtk_crtc_check_trigger(comp->mtk_crtc, false, false);
		} else {
			DDPPR_ERR("%s: invalid ID = %d\n", __func__, id);
			ret = -EFAULT;
			kfree(ccorr);
		}
	}

	return ret;
}
#endif

static int mtk_disp_ccorr_set_interrupt(struct mtk_ddp_comp *comp, void *data)
{
	int enabled = *((int *)data);
	unsigned long flags;
	unsigned int index = index_of_ccorr(comp->id);
	int ret = 0;

	DDPDBG("%s @ %d......... spin_lock_irqsave ++ %d\n", __func__, __LINE__, index);
	spin_lock_irqsave(&g_ccorr_clock_lock, flags);
	DDPDBG("%s @ %d......... spin_lock_irqsave -- ",
		__func__, __LINE__);
	if (atomic_read(&g_ccorr_is_clock_on[index]) != 1) {
		DDPINFO("%s: clock is off. enabled:%d\n",
			__func__, enabled);

		spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
		DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
			__func__, __LINE__);
		return ret;
	}

	if (enabled || g_old_pq_backlight != g_pq_backlight) {
		if (readl(comp->regs + DISP_REG_CCORR_EN) == 0) {
			/* Print error message */
			DDPINFO("[WARNING] DISP_REG_CCORR_EN not enabled!\n");
		}
		/* Enable output frame end interrupt */
		writel(0x2, comp->regs + DISP_REG_CCORR_INTEN);
		DDPINFO("%s: Interrupt enabled\n", __func__);
	} else {
		/* Disable output frame end interrupt */
		writel(0x0, comp->regs + DISP_REG_CCORR_INTEN);
		DDPINFO("%s: Interrupt disabled\n", __func__);
	}
	spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
	DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
		__func__, __LINE__);
	return ret;
}

int disp_ccorr_set_color_matrix(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, int32_t matrix[16], int32_t hint, bool fte_flag)
{
#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	int ret = 0;
	int i, j;
	int ccorr_without_gamma = 0;
	bool need_refresh = false;
	bool identity_matrix = true;
	unsigned int id = index_of_ccorr(comp->id);
	struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct DRM_DISP_CCORR_COEF_T *ccorr;

	if (handle == NULL) {
		DDPPR_ERR("%s: cmdq can not be NULL\n", __func__);
		return -EFAULT;
	}
	if (identity_matrix && (disp_ccorr_number == 1) &&
			(!g_prim_ccorr_force_linear) && (disp_ccorr_linear & 0x01)) {
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
		pr_info("%s: identity_matrix=%d, disp_ccorr_number=%d, g_prim_ccorr_force_linear=%d, disp_ccorr_linear=0x%x\n",
			__func__, identity_matrix, disp_ccorr_number, g_prim_ccorr_force_linear, disp_ccorr_linear);
#endif
		return ret;
	}

	if (g_disp_ccorr_coef[id] == NULL) {
		ccorr = kmalloc(sizeof(struct DRM_DISP_CCORR_COEF_T), GFP_KERNEL);
		if (ccorr == NULL) {
			DDPPR_ERR("%s: no memory\n", __func__);
			return -EFAULT;
		}
		g_disp_ccorr_coef[id] = ccorr;
	}

	mutex_lock(&g_ccorr_global_lock);


	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			/* Copy Color Matrix */
			g_ccorr_color_matrix[id][i][j] = matrix[j*4 + i];

			/* early jump out */
			if (ccorr_without_gamma == 1)
				continue;

			if (i == j && g_ccorr_color_matrix[id][i][j] != ccorr_offset_base) {
				ccorr_without_gamma = 1;
				identity_matrix = false;
			} else if (i != j && g_ccorr_color_matrix[id][i][j] != 0) {
				ccorr_without_gamma = 1;
				identity_matrix = false;
			}
		}
	}

	// hint: 0: identity matrix; 1: arbitraty matrix
	// fte_flag: true: gpu overlay && hwc not identity matrix
	// arbitraty matrix maybe identity matrix or color transform matrix;
	// only when set identity matrix and not gpu overlay, open display color
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("%s: hint: %d, identity: %d, fte_flag: %d, bypass: color0:%d color1:%d",
		__func__, hint, identity_matrix, fte_flag, bypass_color0, bypass_color1);
#endif
	if (((hint == 0) || ((hint == 1) && identity_matrix)) && (!fte_flag)) {
		if (id == 0) {
			if (bypass_color0 == true) {
				struct mtk_ddp_comp *comp_color0 =
					priv->ddp_comp[DDP_COMPONENT_COLOR0];
				ddp_color_bypass_color(comp_color0, false, handle);
				bypass_color0 = false;
			}
		} else if (id == 1 || id == 2) {
			if (bypass_color1 == true) {
				struct mtk_ddp_comp *comp_color1 =
					priv->ddp_comp[DDP_COMPONENT_COLOR1];
				ddp_color_bypass_color(comp_color1, false, handle);
				bypass_color1 = false;
			}
		} else {
			DDPINFO("%s, id is invalid!\n", __func__);
		}
	} else {
		if (id == 0) {
			if ((bypass_color0 == false) && (disp_ccorr_number == 1)
				&& (!(disp_ccorr_linear & 0x01))) {
				struct mtk_ddp_comp *comp_color0 =
					priv->ddp_comp[DDP_COMPONENT_COLOR0];
				ddp_color_bypass_color(comp_color0, true, handle);
				bypass_color0 = true;
			}
		} else if (id == 1 || id == 2) {
			if ((bypass_color1 == false) && (disp_ccorr_number == 1)
				&& (!(disp_ccorr_linear & 0x01))) {
				struct mtk_ddp_comp *comp_color1 =
					priv->ddp_comp[DDP_COMPONENT_COLOR1];
				ddp_color_bypass_color(comp_color1, true, handle);
				bypass_color1 = true;
			}
		} else {
			DDPINFO("%s, id is invalid!\n", __func__);
		}
	}
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("%s: bypass: color0:%d color1:%d", __func__, bypass_color0, bypass_color1);
#endif

	// offset part
/*	if ((matrix[12] != 0) || (matrix[13] != 0) || (matrix[14] != 0))
		need_offset = true;
	else
		need_offset = false;
*/

	g_disp_ccorr_coef[id]->offset[0] = (matrix[12] << 1) << ccorr_offset_mask;
	g_disp_ccorr_coef[id]->offset[1] = (matrix[13] << 1) << ccorr_offset_mask;
	g_disp_ccorr_coef[id]->offset[2] = (matrix[14] << 1) << ccorr_offset_mask;

	//if only ccorr0 hw exist and aosp forece linear or
	//pq force nonlinear,id should be 0, g_disp_ccorr_coef
	//should be PQ ioctl data, so no need to set value here

	if (!(((g_prim_ccorr_force_linear && (disp_ccorr_linear&0x01)) ||
		(g_prim_ccorr_pq_nonlinear && (disp_ccorr_linear == 0x0))) &&
		(disp_ccorr_number == 1))) {
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				g_disp_ccorr_coef[id]->coef[i][j] = 0;
				if (i == j)
					g_disp_ccorr_coef[id]->coef[i][j] = ccorr_offset_base;
		}
	}

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	for (i = 0; i < 3; i++) {
		pr_info("g_disp_ccorr_coef[%d][0-2] = {%d, %d, %d}\n",
				i,
				g_disp_ccorr_coef[id]->coef[i][0],
				g_disp_ccorr_coef[id]->coef[i][1],
				g_disp_ccorr_coef[id]->coef[i][2]);
	}

	for (i = 0; i < 4; i++) {
		pr_info("matrix[%d][0-3] = {%d, %d, %d, %d}\n",
			i, matrix[i*4 + 0], matrix[i*4 + 1], matrix[i*4 + 2], matrix[i*4 + 3]);
	}

	for (i = 0; i < 3; i++) {
		pr_info("g_ccorr_color_matrix[%d][0-2] = {%d, %d, %d}\n",
				i,
				g_ccorr_color_matrix[id][i][0],
				g_ccorr_color_matrix[id][i][1],
				g_ccorr_color_matrix[id][i][2]);
	}

	pr_info("g_ccorr_color_matrix offset {%d, %d, %d}, hint: %d\n",
		g_disp_ccorr_coef[id]->offset[0],
		g_disp_ccorr_coef[id]->offset[1],
		g_disp_ccorr_coef[id]->offset[2], hint);
#endif /* CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG */

	g_disp_ccorr_without_gamma = ccorr_without_gamma;
	g_disp_ccorr_temp_linear = g_disp_ccorr_without_gamma;

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("%s: disp_aosp_ccorr=%d, g_disp_ccorr_without_gamma=%d, g_ccorr_relay_value=%d, g_ccorr_8bit_switch=%d\n",
		__func__, disp_aosp_ccorr, g_disp_ccorr_without_gamma,
		g_ccorr_relay_value[index_of_ccorr(comp->id)],
		g_ccorr_8bit_switch[index_of_ccorr(comp->id)]);
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_SIZE=0x%08x <-> 0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_SIZE), g_ccorr_size[index_of_ccorr(comp->id)]);
#endif

	disp_aosp_ccorr = true;
	disp_ccorr_write_coef_reg(comp, handle, 0);

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			if (g_ccorr_prev_matrix[id][i][j]
				!= g_ccorr_color_matrix[id][i][j]) {
				/* refresh when matrix changed */
				need_refresh = true;
			}
			/* Copy Color Matrix */
			g_ccorr_prev_matrix[id][i][j] = g_ccorr_color_matrix[id][i][j];
		}
	}

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("g_disp_ccorr_without_gamma: [%d], need_refresh: [%d]\n",
		g_disp_ccorr_without_gamma, need_refresh);
#endif

	mutex_unlock(&g_ccorr_global_lock);

	if (need_refresh == true && comp->mtk_crtc != NULL)
		mtk_crtc_check_trigger(comp->mtk_crtc, false, false);

	return ret;
#else
	return -EFAULT;
#endif /* CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE */
}

int disp_ccorr_set_RGB_Gain(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle,
	int r, int g, int b)
{
#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	int ret;
	unsigned int id = index_of_ccorr(comp->id);

	mutex_lock(&g_ccorr_global_lock);
	g_rgb_matrix[id][0][0] = r;
	g_rgb_matrix[id][1][1] = g;
	g_rgb_matrix[id][2][2] = b;

	DDPINFO("%s: r[%d], g[%d], b[%d]", __func__, r, g, b);
	ret = disp_ccorr_write_coef_reg(comp, NULL, 0);
	mutex_unlock(&g_ccorr_global_lock);

	return ret;
#else
	return 0;
#endif /* CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE */
}

int mtk_drm_ioctl_set_ccorr(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct mtk_drm_private *private = dev->dev_private;
	struct mtk_ddp_comp *comp = private->ddp_comp[DDP_COMPONENT_CCORR0];
	struct drm_crtc *crtc = private->crtc[0];
	struct DRM_DISP_CCORR_COEF_T *ccorr_config = data;
	int ret;

	if (ccorr_config->hw_id == DRM_DISP_CCORR1) {
		comp = private->ddp_comp[DDP_COMPONENT_CCORR1];
		g_disp_ccorr_without_gamma = CCORR_INVERSE_GAMMA;
	} else if (disp_ccorr_linear&0x01) {
		g_disp_ccorr_without_gamma = CCORR_INVERSE_GAMMA;
	} else {
		g_disp_ccorr_without_gamma = CCORR_BYASS_GAMMA;
		g_prim_ccorr_pq_nonlinear = true;
	}

	if (m_new_pq_persist_property[DISP_PQ_CCORR_SILKY_BRIGHTNESS]) {

		ret = mtk_crtc_user_cmd(crtc, comp, SET_CCORR, data);

		if ((ccorr_config->silky_bright_flag) == 1 &&
			ccorr_config->FinalBacklight != 0) {
			DDPINFO("brightness = %d, silky_bright_flag = %d",
				ccorr_config->FinalBacklight,
				ccorr_config->silky_bright_flag);
			mtk_leds_brightness_set("lcd-backlight",
				ccorr_config->FinalBacklight);
		}

		mtk_crtc_check_trigger(comp->mtk_crtc, false, true);

		return ret;
	} else {
		return mtk_crtc_user_cmd(crtc, comp, SET_CCORR, data);
	}
}

#ifdef CONFIG_LEDS_BRIGHTNESS_CHANGED
static bool is_led_need_ccorr(int connector_id)
{
	unsigned int crtc0_connector_id = 0;
	struct mtk_ddp_comp *output_comp = NULL;

	if (connector_id <= 0) {
		DDPINFO("%s: invalid connector id\n", __func__);
		return true;
	}
	if (default_comp == NULL || default_comp->mtk_crtc == NULL) {
		DDPPR_ERR("%s: null pointer!\n", __func__);
		return false;
	}
	output_comp = mtk_ddp_comp_request_output(default_comp->mtk_crtc);
	if (output_comp == NULL) {
		DDPPR_ERR("%s: output_comp is null!\n", __func__);
		return false;
	}
	mtk_ddp_comp_io_cmd(output_comp, NULL, GET_CRTC0_CONNECTOR_ID, &crtc0_connector_id);
	return (connector_id == crtc0_connector_id);
}

int led_brightness_changed_event_to_pq(struct notifier_block *nb, unsigned long event,
	void *v)
{
	int trans_level;
	struct led_conf_info *led_conf;

	led_conf = (struct led_conf_info *)v;

	switch (event) {
	case LED_BRIGHTNESS_CHANGED:
		if (!is_led_need_ccorr(led_conf->connector_id)) {
			DDPINFO("connector id %d no need aal\n", led_conf->connector_id);
			led_conf->aal_enable = 0;
			break;
		}
		trans_level = led_conf->cdev.brightness;

		disp_pq_notify_backlight_changed(trans_level);
		DDPINFO("%s: brightness changed: %d(%d)\n",
			__func__, trans_level, led_conf->cdev.brightness);
		break;
	case LED_STATUS_SHUTDOWN:
		disp_pq_notify_backlight_changed(0);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block leds_init_notifier = {
	.notifier_call = led_brightness_changed_event_to_pq,
};
#endif

int mtk_drm_ioctl_ccorr_eventctl(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct mtk_drm_private *private = dev->dev_private;
	struct mtk_ddp_comp *comp = private->ddp_comp[DDP_COMPONENT_CCORR0];
	int ret = 0;
	/* TODO: dual pipe */
	int *enabled = data;

	if (enabled || g_old_pq_backlight != g_pq_backlight)
		mtk_crtc_check_trigger(comp->mtk_crtc, false, true);

	//mtk_crtc_user_cmd(crtc, comp, EVENTCTL, data);
	DDPINFO("ccorr_eventctl, enabled = %d\n", *enabled);

	if ((!atomic_read(&g_irq_backlight_change)) || (*enabled == 1))
		disp_ccorr_set_interrupt(comp, *enabled);

	return ret;
}

int mtk_drm_ioctl_ccorr_get_irq(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	int ret = 0;

	atomic_set(&g_ccorr_is_init_valid, 1);

	disp_ccorr_wait_irq(dev, 60);

	if (disp_pq_copy_backlight_to_user((int *) data) < 0) {
		DDPPR_ERR("%s: failed", __func__);
		ret = -EFAULT;
	}

	return ret;
}

int mtk_drm_ioctl_aibld_cv_mode(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	g_is_aibld_cv_mode = *(bool *)data;
	return 0;
}

int mtk_drm_ioctl_support_color_matrix(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	int ret = 0;
	struct DISP_COLOR_TRANSFORM *color_transform;
	bool support_matrix = true;
	bool identity_matrix = true;
	int i, j;

	if (data == NULL) {
		support_matrix = false;
		ret = -EFAULT;
		DDPINFO("unsupported matrix");
		return ret;
	}

	color_transform = data;

	// Support matrix:
	// AOSP is 4x3 matrix. Offset is located at 4th row (not zero)

	for (i = 0 ; i < 3; i++) {
		if (color_transform->matrix[i][3] != 0) {
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
			for (i = 0 ; i < 4; i++) {
				pr_info("%s: unsupported:[%d][0-3]:[%d %d %d %d]", __func__,
					i,
					color_transform->matrix[i][0],
					color_transform->matrix[i][1],
					color_transform->matrix[i][2],
					color_transform->matrix[i][3]);
			}
#endif
			support_matrix = false;
			ret = -EFAULT;
			return ret;
		}
	}
	if (support_matrix) {
		ret = 0; //Zero: support color matrix.
		for (i = 0 ; i < 3; i++)
			for (j = 0 ; j < 3; j++)
				if ((i == j) &&
					(color_transform->matrix[i][j] != ccorr_offset_base))
					identity_matrix = false;
	}

	//if only one ccorr and ccorr0 is linear, AOSP matrix unsupport
	if ((disp_ccorr_number == 1) && (disp_ccorr_linear&0x01)
		&& (!identity_matrix) && (!g_prim_ccorr_force_linear)) {
		ret = -EFAULT;
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_VDEBUG
		for (i = 0 ; i < 3; i++) {
			pr_info("%s: unsupported:[%d][0-2]:[%d %d %d]", __func__,
				i,
				color_transform->matrix[i][0],
				color_transform->matrix[i][1],
				color_transform->matrix[i][2]);
		}
#endif
	} else
		ret = 0;

	return ret;
}

int mtk_get_ccorr_caps(struct drm_mtk_ccorr_caps *ccorr_caps)
{
	memcpy(ccorr_caps, &disp_ccorr_caps, sizeof(disp_ccorr_caps));
	return 0;
}

int mtk_set_ccorr_caps(struct drm_mtk_ccorr_caps *ccorr_caps)
{
	if (ccorr_caps->ccorr_linear != disp_ccorr_caps.ccorr_linear) {
		disp_ccorr_caps.ccorr_linear = ccorr_caps->ccorr_linear;
		disp_ccorr_linear = disp_ccorr_caps.ccorr_linear;
		DDPINFO("%s:update ccorr 0 linear by DORA API\n", __func__);
	}
	return 0;
}

static void mtk_ccorr_config(struct mtk_ddp_comp *comp,
			     struct mtk_ddp_config *cfg,
			     struct cmdq_pkt *handle)
{
	unsigned int index = index_of_ccorr(comp->id);
	unsigned int width;

	if (comp->mtk_crtc->is_dual_pipe)
		width = cfg->w / 2;
	else
		width = cfg->w;

	DDPINFO("%s\n", __func__);

	if (cfg->source_bpc == 8)
		g_ccorr_8bit_switch[index_of_ccorr(comp->id)] = 1;
	else if (cfg->source_bpc == 10)
		g_ccorr_8bit_switch[index_of_ccorr(comp->id)] = 0;
	else
		DDPINFO("Disp CCORR's bit is : %u\n", cfg->bpc);

#if 0
	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_CCORR_SIZE,
		       (width << 16) | cfg->h, ~0);
#else
	g_ccorr_size[index] = (width << 16) | cfg->h;
	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_CCORR_SIZE,
		       g_ccorr_size[index], ~0);
#endif

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("DISP-CCORR: %s: width=0x%04x, cfg->w=0x%04x, cfg->h=0x%04x, cfg->source_bpc=%d => 0x%08x\n",
			__func__, width, cfg->w, cfg->h, cfg->source_bpc, g_ccorr_size[index]);
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_EN=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_EN));
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_SIZE=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_SIZE));
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_CFG=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_CFG));
	pr_info("DISP-CCORR: CCORR_REG(0)=0x%08x\n", readl(comp->regs + CCORR_REG(0)));
	pr_info("DISP-CCORR: CCORR_REG(1)=0x%08x\n", readl(comp->regs + CCORR_REG(1)));
	pr_info("DISP-CCORR: CCORR_REG(2)=0x%08x\n", readl(comp->regs + CCORR_REG(2)));
	pr_info("DISP-CCORR: CCORR_REG(3)=0x%08x\n", readl(comp->regs + CCORR_REG(3)));
	pr_info("DISP-CCORR: CCORR_REG(4)=0x%08x\n", readl(comp->regs + CCORR_REG(4)));
	pr_info("DISP-CCORR: DISP_REG_CCORR_COLOR_OFFSET_0=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_COLOR_OFFSET_0));
	pr_info("DISP-CCORR: DISP_REG_CCORR_COLOR_OFFSET_1=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_COLOR_OFFSET_1));
	pr_info("DISP-CCORR: DISP_REG_CCORR_COLOR_OFFSET_2=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_COLOR_OFFSET_2));
#endif

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	/* CAN NOT bypass CCORR */
	g_ccorr_relay_value[index_of_ccorr(comp->id)] = 0;
#endif /* CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE */
}

static void mtk_ccorr_start(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle)
{
	DDPINFO("%s\n", __func__);

	disp_aosp_ccorr = false;
	g_disp_ccorr_without_gamma = CCORR_INVERSE_GAMMA;

	if (disp_ccorr_number == 2) {
		if (comp->id == DDP_COMPONENT_CCORR0) {
			disp_aosp_ccorr = true;
			g_disp_ccorr_without_gamma = g_disp_ccorr_temp_linear;
		}

		if ((comp->mtk_crtc->is_dual_pipe) &&
			(comp->id == DDP_COMPONENT_CCORR2)) {
			disp_aosp_ccorr = true;
			g_disp_ccorr_without_gamma = g_disp_ccorr_temp_linear;
		}

	} else if (!(disp_ccorr_linear & 0x01)) {
		disp_aosp_ccorr = true;
		g_disp_ccorr_without_gamma = g_disp_ccorr_temp_linear;
	}

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("%s: disp_aosp_ccorr=%d, g_disp_ccorr_without_gamma=%d, g_ccorr_relay_value=%d, g_ccorr_8bit_switch=%d\n",
		__func__, disp_aosp_ccorr, g_disp_ccorr_without_gamma,
		g_ccorr_relay_value[index_of_ccorr(comp->id)],
		g_ccorr_8bit_switch[index_of_ccorr(comp->id)]);
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_SIZE=0x%08x <-> 0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_SIZE), g_ccorr_size[index_of_ccorr(comp->id)]);
#endif

	disp_ccorr_write_coef_reg(comp, handle, 1);

	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_CCORR_EN, 0x1, 0x1);
}

static void mtk_ccorr_bypass(struct mtk_ddp_comp *comp, int bypass,
	struct cmdq_pkt *handle)
{
	DDPINFO("%s\n", __func__);
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("DISP-CCORR: %s: bypass: 0 (arg bypass=%d)\n", __func__, bypass);
#endif
	/* CAN NOT bypass CCORR */
	bypass = 0;
#endif
	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_CCORR_CFG, bypass, 0x1);
	g_ccorr_relay_value[index_of_ccorr(comp->id)] = bypass;
}

static int mtk_ccorr_user_cmd(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, unsigned int cmd, void *data)
{
	DDPINFO("%s: cmd: %d\n", __func__, cmd);
	switch (cmd) {
	case SET_CCORR:
	{
#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
		struct DRM_DISP_CCORR_COEF_T *config = data;
		struct mtk_disp_ccorr *ccorr = comp_to_ccorr(comp);

		if (disp_ccorr_set_coef(config,
			comp, handle) < 0) {
			DDPPR_ERR("DISP_IOCTL_SET_CCORR: failed\n");
			return -EFAULT;
		}
		if (comp->mtk_crtc->is_dual_pipe) {
			struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
			struct drm_crtc *crtc = &mtk_crtc->base;
			struct mtk_drm_private *priv = crtc->dev->dev_private;
			struct mtk_ddp_comp *comp_ccorr1 = priv->ddp_comp[DDP_COMPONENT_CCORR1];
			if (ccorr->data->single_pipe_ccorr_num == 2)
				comp_ccorr1 = priv->ddp_comp[DDP_COMPONENT_CCORR3];

			if (disp_ccorr_set_coef(config, comp_ccorr1, handle) < 0) {
				DDPPR_ERR("DISP_IOCTL_SET_CCORR: failed\n");
				return -EFAULT;
			}
		}
#else
		if (atomic_read(&g_ccorr_is_clock_on[index_of_ccorr(comp->id)]) == 1) {
			disp_ccorr_write_coef_reg(comp, handle, 1);
			mtk_crtc_check_trigger(comp->mtk_crtc, false, false);
		}
#endif
	}
	break;

	case SET_INTERRUPT:
	{
		struct mtk_disp_ccorr *ccorr = comp_to_ccorr(comp);

		mtk_disp_ccorr_set_interrupt(comp, data);
		if (comp->mtk_crtc->is_dual_pipe) {
			struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
			struct drm_crtc *crtc = &mtk_crtc->base;
			struct mtk_drm_private *priv = crtc->dev->dev_private;
			struct mtk_ddp_comp *comp_ccorr1 = priv->ddp_comp[DDP_COMPONENT_CCORR1];
			if (ccorr->data->single_pipe_ccorr_num == 2)
				comp_ccorr1 = priv->ddp_comp[DDP_COMPONENT_CCORR2];

			mtk_disp_ccorr_set_interrupt(comp_ccorr1, data);
		}
	}
	break;

	case BYPASS_CCORR:
#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	{
		int *value = data;
		int i, ccorr_num = 1;
		struct mtk_ddp_comp *comp_ccorr[4] = { NULL };

		struct mtk_disp_ccorr *ccorr = comp_to_ccorr(comp);
		struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
		struct drm_crtc *crtc = &mtk_crtc->base;
		struct mtk_drm_private *priv = crtc->dev->dev_private;

		comp_ccorr[0] = comp;

		if ((ccorr->data->single_pipe_ccorr_num == 2) &&
			comp->mtk_crtc->is_dual_pipe) {
			ccorr_num = 4;
			comp_ccorr[1] = priv->ddp_comp[DDP_COMPONENT_CCORR1];
			comp_ccorr[2] = priv->ddp_comp[DDP_COMPONENT_CCORR2];
			comp_ccorr[3] = priv->ddp_comp[DDP_COMPONENT_CCORR3];
		} else if ((ccorr->data->single_pipe_ccorr_num == 2) ||
			comp->mtk_crtc->is_dual_pipe) {
			ccorr_num = 2;
			comp_ccorr[1] = priv->ddp_comp[DDP_COMPONENT_CCORR1];
		}

		for (i = 0; i < ccorr_num; i++) {
			if (comp_ccorr[i] != NULL)
				mtk_ccorr_bypass(comp_ccorr[i], *value, handle);
		}
	}
#endif
	break;

	default:
		DDPPR_ERR("%s: error cmd: %d\n", __func__, cmd);
		return -EINVAL;
	}
	return 0;
}

struct ccorr_backup {
	unsigned int REG_CCORR_CFG;
	unsigned int REG_CCORR_INTEN;
};
static struct ccorr_backup g_ccorr_backup[DISP_CCORR_TOTAL];
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
static bool ccorr_run_restore[DISP_CCORR_TOTAL];
#endif

static void ddp_ccorr_backup(struct mtk_ddp_comp *comp)
{
	unsigned int index = index_of_ccorr(comp->id);

	g_ccorr_backup[index].REG_CCORR_CFG =
			readl(comp->regs + DISP_REG_CCORR_CFG);
	g_ccorr_backup[index].REG_CCORR_INTEN =
			readl(comp->regs + DISP_REG_CCORR_INTEN);
//#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
//	g_ccorr_size[index] = readl(comp->regs + DISP_REG_CCORR_SIZE);
//#endif

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_SIZE=0x%08x\n", __func__, g_ccorr_size[index]);
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_CFG=0x%08x\n", __func__, g_ccorr_backup[index].REG_CCORR_CFG);
#endif
}

static void ddp_ccorr_restore(struct mtk_ddp_comp *comp)
{
	unsigned int index = index_of_ccorr(comp->id);

	writel(g_ccorr_backup[index].REG_CCORR_CFG,
			comp->regs + DISP_REG_CCORR_CFG);
	writel(g_ccorr_backup[index].REG_CCORR_INTEN,
			comp->regs + DISP_REG_CCORR_INTEN);
	if (g_ccorr_size[index])
		writel(g_ccorr_size[index], comp->regs + DISP_REG_CCORR_SIZE);
}

static void mtk_ccorr_prepare(struct mtk_ddp_comp *comp)
{
	struct mtk_disp_ccorr *ccorr = comp_to_ccorr(comp);

	DDPINFO("%s\n", __func__);

	mtk_ddp_comp_clk_prepare(comp);
	atomic_set(&g_ccorr_is_clock_on[index_of_ccorr(comp->id)], 1);

	/* Bypass shadow register and read shadow register */
	if (ccorr->data->need_bypass_shadow)
		mtk_ddp_write_mask_cpu(comp, CCORR_BYPASS_SHADOW,
			DISP_REG_CCORR_SHADOW, CCORR_BYPASS_SHADOW);

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("DISP-CCORR: %s[0]: DISP_REG_CCORR_EN=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_EN));
	pr_info("DISP-CCORR: %s[0]: DISP_REG_CCORR_SIZE=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_SIZE));
	pr_info("DISP-CCORR: %s[0]: DISP_REG_CCORR_CFG=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_CFG));
#endif

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	if (ccorr_run_restore[index_of_ccorr(comp->id)])
		ddp_ccorr_restore(comp);
	else {
		if (!(readl(comp->regs + DISP_REG_CCORR_EN) & 0x1))
			ddp_ccorr_restore(comp);
		ccorr_run_restore[index_of_ccorr(comp->id)] = true;
	}
#else
	ddp_ccorr_restore(comp);
#endif

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("DISP-CCORR: %s[1]: DISP_REG_CCORR_EN=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_EN));
	pr_info("DISP-CCORR: %s[1]: DISP_REG_CCORR_SIZE=0x%08x <-> 0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_SIZE), g_ccorr_size[index_of_ccorr(comp->id)]);
	pr_info("DISP-CCORR: %s[1]: DISP_REG_CCORR_CFG=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_CFG));
#endif
}

static void mtk_ccorr_unprepare(struct mtk_ddp_comp *comp)
{
	unsigned long flags;

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_EN=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_EN));
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_SIZE=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_SIZE));
	pr_info("DISP-CCORR: %s: DISP_REG_CCORR_CFG=0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_CFG));
	pr_info("DISP-CCORR: CCORR_REG(0)=0x%08x\n", readl(comp->regs + CCORR_REG(0)));
	pr_info("DISP-CCORR: CCORR_REG(1)=0x%08x\n", readl(comp->regs + CCORR_REG(1)));
	pr_info("DISP-CCORR: CCORR_REG(2)=0x%08x\n", readl(comp->regs + CCORR_REG(2)));
	pr_info("DISP-CCORR: CCORR_REG(3)=0x%08x\n", readl(comp->regs + CCORR_REG(3)));
	pr_info("DISP-CCORR: CCORR_REG(4)=0x%08x\n", readl(comp->regs + CCORR_REG(4)));
	pr_info("DISP-CCORR: DISP_REG_CCORR_COLOR_OFFSET_0=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_COLOR_OFFSET_0));
	pr_info("DISP-CCORR: DISP_REG_CCORR_COLOR_OFFSET_1=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_COLOR_OFFSET_1));
	pr_info("DISP-CCORR: DISP_REG_CCORR_COLOR_OFFSET_2=0x%08x\n", readl(comp->regs + DISP_REG_CCORR_COLOR_OFFSET_2));
#endif

	ddp_ccorr_backup(comp);
	disp_ccorr_clear_irq_only(comp);

	DDPINFO("%s @ %d......... spin_lock_irqsave ++ ", __func__, __LINE__);
	spin_lock_irqsave(&g_ccorr_clock_lock, flags);
	DDPINFO("%s @ %d......... spin_lock_irqsave -- ", __func__, __LINE__);
	atomic_set(&g_ccorr_is_clock_on[index_of_ccorr(comp->id)], 0);
	spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
	DDPDBG("%s @ %d......... spin_unlock_irqrestore ", __func__, __LINE__);
	wake_up_interruptible(&g_ccorr_get_irq_wq); // wake up who's waiting isr
	mtk_ddp_comp_clk_unprepare(comp);

	DDPINFO("%s\n", __func__);

}

static int mtk_ccorr_io_cmd(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle,
							enum mtk_ddp_io_cmd cmd, void *params)
{
	int enable = 1;

	if (comp->id != DDP_COMPONENT_CCORR0 || !g_is_aibld_cv_mode)
		return 0;

	if (cmd == FRAME_DIRTY) {
		DDPDBG("%s FRAME_DIRTY comp id:%d\n", __func__, comp->id);
		mtk_disp_ccorr_set_interrupt(comp, &enable);
	}
	DDPDBG("%s end\n", __func__);
	return 0;
}

static const struct mtk_ddp_comp_funcs mtk_disp_ccorr_funcs = {
	.config = mtk_ccorr_config,
	.start = mtk_ccorr_start,
	.bypass = mtk_ccorr_bypass,
	.user_cmd = mtk_ccorr_user_cmd,
	.io_cmd = mtk_ccorr_io_cmd,
	.prepare = mtk_ccorr_prepare,
	.unprepare = mtk_ccorr_unprepare,
};

static int mtk_disp_ccorr_bind(struct device *dev, struct device *master,
			       void *data)
{
	struct mtk_disp_ccorr *priv = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	int ret;

	DDPINFO("%s\n", __func__);

	ret = mtk_ddp_comp_register(drm_dev, &priv->ddp_comp);
	if (ret < 0) {
		dev_err(dev, "Failed to register component %s: %d\n",
			dev->of_node->full_name, ret);
		return ret;
	}

	disp_ccorr_color_matrix_to_dispsys(drm_dev);
	return 0;
}

static void mtk_disp_ccorr_unbind(struct device *dev, struct device *master,
				  void *data)
{
	struct mtk_disp_ccorr *priv = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;

	mtk_ddp_comp_unregister(drm_dev, &priv->ddp_comp);
}

static const struct component_ops mtk_disp_ccorr_component_ops = {
	.bind	= mtk_disp_ccorr_bind,
	.unbind = mtk_disp_ccorr_unbind,
};

void mtk_ccorr_dump(struct mtk_ddp_comp *comp)
{
	void __iomem *baddr = comp->regs;

	DDPDUMP("== %s REGS:0x%llx ==\n", mtk_dump_comp_str(comp), comp->regs_pa);
	mtk_cust_dump_reg(baddr, 0x0, 0x20, 0x30, -1);
	mtk_cust_dump_reg(baddr, 0x24, 0x28, -1, -1);
}

static int  mtk_update_ccorr_base(void)
{
#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	int g, i, j;
#endif

	if (disp_ccorr_caps.ccorr_bit == 12)
		return 0;

	ccorr_offset_base = 2048;
	ccorr_max_negative = ccorr_offset_base*(-2);
	ccorr_max_positive = (ccorr_offset_base*2)-1;
	ccorr_fullbit_mask = 0x1fff;
	ccorr_offset_mask = 13;

#ifndef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	for (g = 0; g < DISP_CCORR_TOTAL; g++)
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				if (i == j) {
					g_ccorr_color_matrix[g][i][j] = ccorr_offset_base;
					g_ccorr_prev_matrix[g][i][j] = ccorr_offset_base;
					g_rgb_matrix[g][i][j] = ccorr_offset_base;
				}
			}
#endif
	return 0;
}

static void mtk_get_ccorr_property(struct device_node *node)
{
	int ret;
	int ccorr0_force_linear = 0;

	ret = of_property_read_u32(node, "ccorr_bit", &disp_ccorr_caps.ccorr_bit);
	if (ret)
		DDPPR_ERR("read ccorr_bit failed\n");

	ret = of_property_read_u32(node, "ccorr_num_per_pipe", &disp_ccorr_caps.ccorr_number);
	if (ret)
		DDPPR_ERR("read ccorr_number failed\n");

	ret = of_property_read_u32(node, "ccorr_linear_per_pipe", &disp_ccorr_caps.ccorr_linear);
	if (ret)
		DDPPR_ERR("read ccorr_linear failed\n");

	ret = of_property_read_u32(node, "ccorr_prim_force_linear", &ccorr0_force_linear);
	if (ret)
		DDPPR_ERR("read ccorr_prim_force_linear failed\n");

	DDPINFO("%s:ccorr_bit:%d,ccorr_number:%d,ccorr_linear:%d,ccorr0 force linear:%d\n",
		__func__, disp_ccorr_caps.ccorr_bit, disp_ccorr_caps.ccorr_number,
		disp_ccorr_caps.ccorr_linear, ccorr0_force_linear);

	disp_ccorr_number = disp_ccorr_caps.ccorr_number;
	disp_ccorr_linear = disp_ccorr_caps.ccorr_linear;

	if (ccorr0_force_linear == 0x1)
		g_prim_ccorr_force_linear = true;
	else
		g_prim_ccorr_force_linear = false;

	mtk_update_ccorr_base();

}

#if defined(CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE) && IS_ENABLED(CONFIG_DEBUG_FS)
static int ccorr_grayscale_color_matrix_show(struct seq_file *file, void *v)
{
	struct mtk_disp_ccorr *disp_ccorr = (struct mtk_disp_ccorr *)file->private;
	int i;
	int coef_rgb[3], sum/*, coef*/;
	
	if (!disp_ccorr) {
		seq_puts(file, "can not get mtk_disp_ccorr!\n");
		return 0;
	}

	seq_puts(file,       "* RGB-to-Grayscale Color Transform Matrix:\n");
	seq_puts(file,       "            |r r r|\n");
	seq_puts(file,       "  |R G B| x |g g g| = |R' G' B'|\n");
	seq_puts(file,       "            |b b b|\n");
	seq_puts(file,       "  ** R' = G' = B' = r x R + g x G + b x B\n");
	seq_puts(file,       "  ** An example:\n");
	seq_puts(file,       "     *** R' = G' = B' = 0.299 x R + 0.587 x G + 0.114 x B\n");
	seq_puts(file,       "        **** r = 0.299 => r' = 10000 x r = 2990\n");
	seq_puts(file,       "        **** g = 0.587 => g' = 10000 x g = 5870\n");
	seq_puts(file,       "        **** b = 0.114 => b' = 10000 x b = 1140\n");
	seq_puts(file,       "        **** Using r'/g'/b' as the input integer values: \"r' g' b'\"\n");
	seq_puts(file,       "             # echo \"2990 5870 1140\" > /d/grayscale_color_matrix\n");
	seq_puts(file,       "        **** Checking the result:\n");
	seq_puts(file,       "             # cat /d/grayscale_color_matrix\n");
	seq_puts(file,       "  ** The integer value range of r'/g'/b': -20000 <= r'/g'/b' <= 20000\n");
	seq_puts(file,       "     *** The negative integer value means the color inversion\n");
	seq_puts(file,       "  ** (r' + g' + b') > 0\n");

	seq_printf(file, "\n\n* Last stored Color Matrix \"r' g' b'\": \"%04d %04d %04d\"\n",
			disp_ccorr->stored_color_matrix_rgb[0],
			disp_ccorr->stored_color_matrix_rgb[1],
			disp_ccorr->stored_color_matrix_rgb[2]);
	if (CCORR_ABS(disp_ccorr->stored_color_matrix_rgb[0]) > 20000)
		seq_printf(file, "  ** ERR: -20000 <= (r'=%d) <= 20000\n",
			disp_ccorr->stored_color_matrix_rgb[0]);
	if (CCORR_ABS(disp_ccorr->stored_color_matrix_rgb[1]) > 20000)
		seq_printf(file, "  ** ERR: -20000 <= (g'=%d) <= 20000\n",
			disp_ccorr->stored_color_matrix_rgb[1]);
	if (CCORR_ABS(disp_ccorr->stored_color_matrix_rgb[2]) > 20000)
		seq_printf(file, "  ** ERR: -20000 <= (b'=%d) <= 20000\n",
			disp_ccorr->stored_color_matrix_rgb[2]);

	sum = disp_ccorr->stored_color_matrix_rgb[0] +
			disp_ccorr->stored_color_matrix_rgb[1] +
			disp_ccorr->stored_color_matrix_rgb[2];
	if (sum <= 0)
		seq_printf(file, "  ** ERR: ((r'=%d) + (g'=%d) + (b'=%d) = %d) > 0\n",
			disp_ccorr->stored_color_matrix_rgb[0],
			disp_ccorr->stored_color_matrix_rgb[1],
			disp_ccorr->stored_color_matrix_rgb[2],
			sum);

	seq_puts(file,   "\n\n* Current active/working Color Matrix\n\n");
	seq_printf(file,     "  ** Color Matrix \"r' g' b'\" integer (rgb float value x 10000): \"%04d %04d %04d\"\n",
			disp_ccorr->grayscale_color_matrix[0][0],
			disp_ccorr->grayscale_color_matrix[1][1],
			disp_ccorr->grayscale_color_matrix[2][2]);
	seq_puts(file,       "  ** Color Transform:\n");
	seq_printf(file,     "     R' = G' = B' = %c%d.%04d x R + %c%d.%04d x G + %c%d.%04d x B\n",
			disp_ccorr->grayscale_color_matrix[0][0] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[0][0]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[0][0]) % 10000,
			disp_ccorr->grayscale_color_matrix[1][1] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[1][1]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[1][1]) % 10000,
			disp_ccorr->grayscale_color_matrix[2][2] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[2][2]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[2][2]) % 10000);
	seq_puts(file,       "  ** 3x3 Color Transform Matrix:\n");
	for (i = 0; i < 3; i++) {
		seq_printf(file, "     [%d][0-2] = {%c%d.%04d, %c%d.%04d, %c%d.%04d}\n", i,
			disp_ccorr->grayscale_color_matrix[i][0] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][0]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][0]) % 10000,
			disp_ccorr->grayscale_color_matrix[i][1] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][1]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][1]) % 10000,
			disp_ccorr->grayscale_color_matrix[i][2] < 0 ? '-' : ' ',
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][2]) / 10000,
			CCORR_ABS(disp_ccorr->grayscale_color_matrix[i][2]) % 10000);
	}

	coef_rgb[0] = (int)disp_ccorr->grayscale_ccorr_coef.coef[0][0];
	if (coef_rgb[0] > ccorr_max_positive) /* check negative */
		coef_rgb[0] -= (ccorr_offset_base<<2);
	coef_rgb[1] = (int)disp_ccorr->grayscale_ccorr_coef.coef[1][1];
	if (coef_rgb[1] > ccorr_max_positive) /* check negative */
		coef_rgb[1] -= (ccorr_offset_base<<2);
	coef_rgb[2] = (int)disp_ccorr->grayscale_ccorr_coef.coef[2][2];
	if (coef_rgb[2] > ccorr_max_positive) /* check negative */
		coef_rgb[2] -= (ccorr_offset_base<<2);

	seq_printf(file,     "  ** Color Matrix to MTK DISP-CCORR COEF Conversion: (float value 1.0 -> %d)\n",
			ccorr_offset_base);
	seq_printf(file,     "     ** COEF-R = DIV_ROUND_CLOSEST(r' x %d, 10000) = %d\n",
			ccorr_offset_base, coef_rgb[0]);
	seq_printf(file,     "     ** COEF-G = DIV_ROUND_CLOSEST(g' x %d, 10000) = %d\n",
			ccorr_offset_base, coef_rgb[1]);
	seq_printf(file,     "     ** COEF-B = DIV_ROUND_CLOSEST(b' x %d, 10000) = %d\n",
			ccorr_offset_base, coef_rgb[2]);

	seq_printf(file,     "  ** MTK DISP-CCORR 3x3 COEF: (%d <= COEF <= %d)\n",
		ccorr_max_negative, ccorr_max_positive, ccorr_offset_base);
	for (i = 0; i < 3; i += 1) {
		seq_printf(file, "     [%d][0-2] = {0x%04x, 0x%04x, 0x%04x} = {%04d, %04d, %04d}\n", i,
			disp_ccorr->grayscale_ccorr_coef.coef[i][0],
			disp_ccorr->grayscale_ccorr_coef.coef[i][1],
			disp_ccorr->grayscale_ccorr_coef.coef[i][2],
			coef_rgb[0], coef_rgb[1], coef_rgb[2]);
	}

	seq_puts(file, "\n\n");

	return 0;
}

static ssize_t ccorr_grayscale_color_matrix_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct mtk_disp_ccorr *disp_ccorr;
	char buf[64];
	struct mtk_ddp_comp *comp;
	struct device *dev;
	unsigned int index;
	int ret;
	int color_matrix_r = 0, color_matrix_g = 0, color_matrix_b = 0;

	disp_ccorr = (struct mtk_disp_ccorr *)
		(((struct seq_file *)file->private_data)->private);
	if (!disp_ccorr) {
		pr_err("%s: can not get mtk_disp_ccorr\n", __func__);
		return -EINVAL;
	}

	if (count == 0)
		return -EINVAL;
	if (count > 63)
		count = 63;

	if (copy_from_user(buf, ubuf, count))
		return -EINVAL;

	comp = &disp_ccorr->ddp_comp;
	index = index_of_ccorr(comp->id);
	dev = comp->dev;

	ret = sscanf(buf, "%d %d %d", &color_matrix_r, &color_matrix_g, &color_matrix_b);
	if (ret != 3) {
		dev_err(dev, "%s: invalid format (%d %d %d)!\n", __func__,
			color_matrix_r, color_matrix_g, color_matrix_b);
		return -EINVAL;
	}

	mutex_lock(&g_ccorr_global_lock);

	ret = disp_ccorr_grayscale_calc(disp_ccorr, color_matrix_r, color_matrix_g, color_matrix_b);
	if (ret) {
		mutex_unlock(&g_ccorr_global_lock);

		dev_err(dev, "%s: calculate the grayscale color matrix error: %d\n", __func__, ret);
		return ret;
	}

	if (atomic_read(&g_ccorr_is_clock_on[index]) == 1) {
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
		pr_info("%s: disp_aosp_ccorr=%d, g_disp_ccorr_without_gamma=%d, g_ccorr_relay_value=%d, g_ccorr_8bit_switch=%d\n",
			__func__, disp_aosp_ccorr, g_disp_ccorr_without_gamma,
			g_ccorr_relay_value[index_of_ccorr(comp->id)],
			g_ccorr_8bit_switch[index_of_ccorr(comp->id)]);
		pr_info("DISP-CCORR: %s: DISP_REG_CCORR_SIZE=0x%08x <-> 0x%08x\n", __func__, readl(comp->regs + DISP_REG_CCORR_SIZE), g_ccorr_size[index_of_ccorr(comp->id)]);
#endif
		(void)disp_ccorr_write_coef_reg(comp, NULL, 0);
	}

	mutex_unlock(&g_ccorr_global_lock);

	if (atomic_read(&g_ccorr_is_clock_on[index]) == 1)
		mtk_crtc_check_trigger(comp->mtk_crtc, false, false);

	return count;
}

static int ccorr_grayscale_color_matrix_open(struct inode *inode, struct file *file)
{
	return single_open(file, ccorr_grayscale_color_matrix_show, inode->i_private);
}

static struct dentry *debugfs_ccorr_grayscale_color_matrix;

static const struct file_operations ccorr_grayscale_color_matrix_fops = {
	.open		= ccorr_grayscale_color_matrix_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ccorr_grayscale_color_matrix_write,
};
#endif

static int mtk_disp_ccorr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_disp_ccorr *priv;
	enum mtk_ddp_comp_id comp_id;
	int irq;
	int ret;

	DDPINFO("%s+\n", __func__);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	comp_id = mtk_ddp_comp_get_id(dev->of_node, MTK_DISP_CCORR);
	if ((int)comp_id < 0) {
		DDPPR_ERR("Failed to identify by alias: %d\n", comp_id);
		return comp_id;
	}

	if (comp_id == DDP_COMPONENT_CCORR0) {
		disp_ccorr_caps.ccorr_bit = 12;
		disp_ccorr_caps.ccorr_number = 1;
		disp_ccorr_caps.ccorr_linear = 0x01;
		g_prim_ccorr_force_linear = false;
		g_prim_ccorr_pq_nonlinear = false;
		mtk_get_ccorr_property(dev->of_node);

#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
		/* after mtk_get_ccorr_property() & mtk_update_ccorr_base() */
		if (true/*comp_id == DDP_COMPONENT_CCORR0*/) {
			/**
			 * disp_ccorr0: disp_ccorr0@1400b000 {
			 * ......
			 *     ccorr_bit = <13>;
			 * ......
			 * };
			 * 
			 * disp_ccorr_caps.ccorr_bit = 13
			 *
			 * CCORR_Identity_Value = 2 ^ (ccorr_bit - 2) = 2^11 = 2048
			 *
			 * transFloatToIntForColorMatrix:
			 *     static_cast<int32_t>(val * CCORR_Identity_Value + 0.5f);
			 *
			 *
			 * Solution 1 (Android adopts this): Y = 0.2126 R + 0.7152 G + 0.0722 B
			 * 
			 * private static final float[] MATRIX_GRAYSCALE = new float[]{
			 *     .2126f, .2126f, .2126f, 0f,
			 *     .7152f, .7152f, .7152f, 0f,
			 *     .0722f, .0722f, .0722f, 0f,
			 *     0f, 0f, 0f, 1f
			 * };
			 *
			 * transFloatToIntForColorMatrix:
			 *     .2126f => RoundDown(0.2126 * 2048 + 0.5)=RoundDown(435.9048)=435=0x01b3
			 *     .7152f => RoundDown(0.7152 * 2048 + 0.5)=RoundDown(1465.2296)=1465=0x05b9
			 *     .0722f => RoundDown(0.0722 * 2048 + 0.5)=RoundDown(148.3656)=148=0x0094
			 *
			 * Sanity check: 435+1465+148=2048 => OK
			 *
			 * coef[0][0-2] = {435, 1465, 148} -> {0x01b3, 0x05b9, 0x0094}
			 * coef[1][0-2] = {435, 1465, 148} -> {0x01b3, 0x05b9, 0x0094}
			 * coef[2][0-2] = {435, 1465, 148} -> {0x01b3, 0x05b9, 0x0094}
			 *
			 *
			 * Solution 2: Y = 0.299 R + 0.587 G + 0.114 B
			 *
			 * transFloatToIntForColorMatrix:
			 *     .299f => RoundDown(0.299 * 2048 + 0.5)=RoundDown(612.852)=612=0x0264
			 *     .587f => RoundDown(0.587 * 2048 + 0.5)=RoundDown(1202.676)=1202=0x04b2
			 *     .114f => RoundDown(0.114 * 2048 + 0.5)=RoundDown(233.972)=233=0x00e9
			 *
			 * Sanity check: 612+1202+233=2047 => FAILED
			 *     Review the Int value again:
			 *          NEW: 612+1202+234=2048 => OK
			 *     .114f => RoundDown(0.114 * 2048 + 0.5)=RoundDown(233.972)=233 => 233+1=234=0x00ea
			 *
			 * coef[0][0-2] = {612, 1202, 234} -> {0x0264, 0x04b2, 0x00ea}
			 * coef[1][0-2] = {612, 1202, 234} -> {0x0264, 0x04b2, 0x00ea}
			 * coef[2][0-2] = {612, 1202, 234} -> {0x0264, 0x04b2, 0x00ea}
			 */

			/* Solution 1 (Android adopts this): Y = 0.2126 R + 0.7152 G + 0.0722 B */
			ret = disp_ccorr_grayscale_calc(priv, 2126, 7152, 722);
			if (ret) {
				DDPPR_ERR("calculate the grayscale color matrix error: %d\n", ret);
				return ret;
			}
		}
#endif
	}

	if (!default_comp && comp_id == DDP_COMPONENT_CCORR0)
		default_comp = &priv->ddp_comp;
	if (!ccorr1_default_comp &&
		comp_id == DDP_COMPONENT_CCORR1)
		ccorr1_default_comp = &priv->ddp_comp;

	ret = mtk_ddp_comp_init(dev, dev->of_node, &priv->ddp_comp, comp_id,
				&mtk_disp_ccorr_funcs);
	if (ret != 0) {
		DDPPR_ERR("Failed to initialize component: %d\n", ret);
		return ret;
	}

	priv->data = of_device_get_match_data(dev);

	platform_set_drvdata(pdev, priv);

	ret = devm_request_irq(dev, irq, mtk_disp_ccorr_irq_handler,
			       IRQF_TRIGGER_NONE | IRQF_SHARED,
			       dev_name(dev), priv);

	mtk_ddp_comp_pm_enable(&priv->ddp_comp);

	ret = component_add(dev, &mtk_disp_ccorr_component_ops);
	if (ret != 0) {
		dev_err(dev, "Failed to add component: %d\n", ret);
		mtk_ddp_comp_pm_disable(&priv->ddp_comp);
#if defined(CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE) && IS_ENABLED(CONFIG_DEBUG_FS)
	} else if (comp_id == DDP_COMPONENT_CCORR0) {
		debugfs_ccorr_grayscale_color_matrix =
			debugfs_create_file("grayscale_color_matrix",
					0644, NULL, priv,
					&ccorr_grayscale_color_matrix_fops);
		if (!debugfs_ccorr_grayscale_color_matrix)
			dev_err(dev, "failed to create debugfs\n");
#endif
	}

#ifdef CONFIG_LEDS_BRIGHTNESS_CHANGED
	if (comp_id == DDP_COMPONENT_CCORR0)
		mtk_leds_register_notifier(&leds_init_notifier);
#endif
	DDPINFO("%s-\n", __func__);

	return ret;
}

static int mtk_disp_ccorr_remove(struct platform_device *pdev)
{
	struct mtk_disp_ccorr *priv = dev_get_drvdata(&pdev->dev);

#if defined(CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE) && IS_ENABLED(CONFIG_DEBUG_FS)
	if (priv->ddp_comp.id == DDP_COMPONENT_CCORR0 && debugfs_ccorr_grayscale_color_matrix) {
		debugfs_remove(debugfs_ccorr_grayscale_color_matrix);
		debugfs_ccorr_grayscale_color_matrix = NULL;
	}
#endif

	component_del(&pdev->dev, &mtk_disp_ccorr_component_ops);
	mtk_ddp_comp_pm_disable(&priv->ddp_comp);

#ifdef CONFIG_LEDS_BRIGHTNESS_CHANGED
	if (priv->ddp_comp.id == DDP_COMPONENT_CCORR0)
		mtk_leds_unregister_notifier(&leds_init_notifier);
#endif
	return 0;
}
static const struct mtk_disp_ccorr_data mt6739_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = false,
	.single_pipe_ccorr_num = 1,
};

static const struct mtk_disp_ccorr_data mt6765_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = false,
	.single_pipe_ccorr_num = 1,
};

static const struct mtk_disp_ccorr_data mt6761_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = false,
	.single_pipe_ccorr_num = 1,
};

static const struct mtk_disp_ccorr_data mt6768_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = false,
	.single_pipe_ccorr_num = 1,
};

static const struct mtk_disp_ccorr_data mt6779_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = false,
	.single_pipe_ccorr_num = 1,
};

static const struct mtk_disp_ccorr_data mt6885_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = false,
	.single_pipe_ccorr_num = 1,
};

static const struct mtk_disp_ccorr_data mt6873_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = true,
	.single_pipe_ccorr_num = 1,
};

static const struct mtk_disp_ccorr_data mt6853_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = true,
	.single_pipe_ccorr_num = 2,
};

static const struct mtk_disp_ccorr_data mt6833_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = true,
	.single_pipe_ccorr_num = 1,
};

static const struct mtk_disp_ccorr_data mt6983_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = true,
	.single_pipe_ccorr_num = 2,
};

static const struct mtk_disp_ccorr_data mt6895_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = true,
	.single_pipe_ccorr_num = 2,
};

static const struct mtk_disp_ccorr_data mt6879_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = true,
	.single_pipe_ccorr_num = 2,
};

static const struct mtk_disp_ccorr_data mt6855_ccorr_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = true,
	.single_pipe_ccorr_num = 1,
};

static const struct of_device_id mtk_disp_ccorr_driver_dt_match[] = {
	{ .compatible = "mediatek,mt6739-disp-ccorr",
	  .data = &mt6739_ccorr_driver_data},
	{ .compatible = "mediatek,mt6765-disp-ccorr",
	  .data = &mt6765_ccorr_driver_data},
	{ .compatible = "mediatek,mt6761-disp-ccorr",
	  .data = &mt6761_ccorr_driver_data},
	{ .compatible = "mediatek,mt6768-disp-ccorr",
	  .data = &mt6768_ccorr_driver_data},
	{ .compatible = "mediatek,mt6779-disp-ccorr",
	  .data = &mt6779_ccorr_driver_data},
	{ .compatible = "mediatek,mt6789-disp-ccorr",
	  .data = &mt6789_ccorr_driver_data},
	{ .compatible = "mediatek,mt6885-disp-ccorr",
	  .data = &mt6885_ccorr_driver_data},
	{ .compatible = "mediatek,mt6873-disp-ccorr",
	  .data = &mt6873_ccorr_driver_data},
	{ .compatible = "mediatek,mt6853-disp-ccorr",
	  .data = &mt6853_ccorr_driver_data},
	{ .compatible = "mediatek,mt6833-disp-ccorr",
	  .data = &mt6833_ccorr_driver_data},
	{ .compatible = "mediatek,mt6983-disp-ccorr",
	  .data = &mt6983_ccorr_driver_data},
	{ .compatible = "mediatek,mt6895-disp-ccorr",
	  .data = &mt6895_ccorr_driver_data},
	{ .compatible = "mediatek,mt6879-disp-ccorr",
	  .data = &mt6879_ccorr_driver_data},
	{ .compatible = "mediatek,mt6855-disp-ccorr",
	  .data = &mt6855_ccorr_driver_data},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_disp_ccorr_driver_dt_match);

struct platform_driver mtk_disp_ccorr_driver = {
	.probe = mtk_disp_ccorr_probe,
	.remove = mtk_disp_ccorr_remove,
	.driver = {

			.name = "mediatek-disp-ccorr",
			.owner = THIS_MODULE,
			.of_match_table = mtk_disp_ccorr_driver_dt_match,
		},
};

void disp_ccorr_set_bypass(struct drm_crtc *crtc, int bypass)
{
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE
	/* CAN NOT bypass CCORR (internal used) */
#ifdef CONFIG_DRM_MTK_ICOM_FORCE_GRAYSCALE_DEBUG
	pr_info("DISP-CCORR: %s: bypass: 0 (internal used) (arg bypass=%d)\n", __func__, bypass);
#endif
#else
	int ret;

	if (g_ccorr_relay_value[index_of_ccorr(default_comp->id)] == bypass &&
		g_ccorr_relay_value[index_of_ccorr(ccorr1_default_comp->id)] == bypass)
		return;
	ret = mtk_crtc_user_cmd(crtc, default_comp, BYPASS_CCORR, &bypass);

	DDPINFO("%s : ret = %d", __func__, ret);
#endif
}

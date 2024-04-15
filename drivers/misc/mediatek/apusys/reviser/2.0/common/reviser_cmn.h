/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */


#ifndef __APUSYS_REVISER_COMMON_H__
#define __APUSYS_REVISER_COMMON_H__

#include <linux/printk.h>
#include <linux/seq_file.h>

extern u32 g_rvr_klog;
extern u32 g_rvr_aee_cnt;

enum {
	RVR_DBG_HW = 0x01,
	RVR_DBG_VLM = 0x02,
	RVR_DBG_TBL = 0x04,
	RVR_DBG_MEM = 0x08,
	RVR_DBG_FLW = 0x10,
};

#define REVISER_PREFIX "[reviser]"

static inline
int rvr_debug_on(int mask)
{
	return g_rvr_klog & mask;
}


#define LOG_ERR(x, args...) \
	pr_info(REVISER_PREFIX "[error] %s " x, __func__, ##args)
#define LOG_WARN(x, args...) \
	pr_info(REVISER_PREFIX "[warn] %s " x, __func__, ##args)
#define LOG_INFO(x, args...) \
	pr_info(REVISER_PREFIX "%s " x, __func__, ##args)
//#define LOG_DEBUG(x, args...) \
//	pr_info(REVISER_PREFIX "[debug] %s/%d " x, __func__, __LINE__, ##args)
#define LOG_DEBUG(x, args...)
#define rvr_debug(mask, x, ...) do { if (rvr_debug_on(mask)) \
		pr_info(REVISER_PREFIX " %s/%d " x, __func__, \
		__LINE__, ##__VA_ARGS__); \
	} while (0)

#define LOG_DBG_RVR_HW(x, ...) rvr_debug(RVR_DBG_HW, x, ##__VA_ARGS__)
#define LOG_DBG_RVR_VLM(x, ...) rvr_debug(RVR_DBG_VLM, x, ##__VA_ARGS__)
#define LOG_DBG_RVR_TBL(x, ...) rvr_debug(RVR_DBG_TBL, x, ##__VA_ARGS__)
#define LOG_DBG_RVR_MEM(x, ...) rvr_debug(RVR_DBG_MEM, x, ##__VA_ARGS__)
#define LOG_DBG_RVR_FLW(x, ...) rvr_debug(RVR_DBG_FLW, x, ##__VA_ARGS__)

#define DEBUG_TAG LOG_DEBUG("\n")
//#define DEBUG_TAG

#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#include <aee.h>
#define apu_reviser_aee(format, args...) \
	do { \
		if (g_rvr_aee_cnt++ < 3) {\
			pr_info("apusys reviser:" format, ##args); \
			aee_kernel_warning("APUSYS_REVISER_EXCEPTION", \
				"\nCRDISPATCH_KEY:APUSYS_REVISER\n" format, \
				##args); \
		}\
	} while (0)
#else
#define apu_reviser_aee(...)
#endif


/* print to console via seq file */
#define LOG_CON(s, x, args...) \
	{\
		if (s) \
			seq_printf(s, x, ##args); \
		else \
			LOG_INFO(x, ##args); \
	}
#endif

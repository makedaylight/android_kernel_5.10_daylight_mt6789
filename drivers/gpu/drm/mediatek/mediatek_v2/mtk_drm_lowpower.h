/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _MTK_DRM_LOWPOWER_H_
#define _MTK_DRM_LOWPOWER_H_

#include <drm/drm_crtc.h>

#ifndef DRM_CMDQ_DISABLE
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#else
#include "mtk-cmdq-ext.h"
#endif

#ifdef SHARE_WROT_SRAM
#include "mdp_event_common.h"
#define CMDQ_EVENT_ENUM cmdq_event
#endif

struct mtk_drm_idlemgr_context {
	unsigned long long idle_check_interval;
	unsigned long long idlemgr_last_kick_time;
	unsigned int enterulps;
	int session_mode_before_enter_idle;
	int is_idle;
	int cur_lp_cust_mode;
};

struct mtk_drm_idlemgr {
	struct task_struct *idlemgr_task;
	struct task_struct *kick_task;
	wait_queue_head_t idlemgr_wq;
	wait_queue_head_t kick_wq;
	atomic_t idlemgr_task_active;
	atomic_t kick_task_active;
	struct mtk_drm_idlemgr_context *idlemgr_ctx;
};

void mtk_drm_idlemgr_kick(const char *source, struct drm_crtc *crtc,
			  int need_lock);
bool mtk_drm_is_idle(struct drm_crtc *crtc);

int mtk_drm_idlemgr_init(struct drm_crtc *crtc, int index);
unsigned int mtk_drm_set_idlemgr(struct drm_crtc *crtc, unsigned int flag,
				 bool need_lock);
unsigned long long
mtk_drm_set_idle_check_interval(struct drm_crtc *crtc,
				unsigned long long new_interval);
unsigned long long
mtk_drm_get_idle_check_interval(struct drm_crtc *crtc);

void mtk_drm_idlemgr_kick_async(struct drm_crtc *crtc);

#ifdef SHARE_WROT_SRAM
unsigned int can_use_wrot_sram(void);
void set_share_sram(unsigned int share_sram);
unsigned int use_wrot_sram(void);
void mtk_drm_enter_share_sram(struct drm_crtc *crtc, bool first);
void mtk_drm_leave_share_sram(struct drm_crtc *crtc,
		struct cmdq_pkt *cmdq_handle);
int32_t _acquire_wrot_resource(enum CMDQ_EVENT_ENUM resourceEvent);
#endif

#endif

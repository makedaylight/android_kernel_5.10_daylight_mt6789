// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/delay.h>
#include <drm/mediatek_drm.h>
#include <drm/drm_vblank.h>
#include "mtk_drm_lowpower.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_drv.h"
#include "mtk_drm_ddp.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_mmp.h"

#ifdef SHARE_WROT_SRAM
#include "cmdq_helper_ext.h"
#endif

#define MAX_ENTER_IDLE_RSZ_RATIO 300

static void mtk_drm_idlemgr_enable_crtc(struct drm_crtc *crtc);
static void mtk_drm_idlemgr_disable_crtc(struct drm_crtc *crtc);

#ifdef SHARE_WROT_SRAM
static int register_share_sram;
static unsigned int is_wrot_sram_enabled;
static unsigned int is_wrot_sram_available;
static unsigned int is_wrot_sram;
static struct drm_crtc *share_sram_crtc;

unsigned int can_use_wrot_sram(void)
{
	return is_wrot_sram_available && is_wrot_sram_enabled;
}

void set_share_sram(unsigned int share_sram)
{
	if (is_wrot_sram != share_sram)
		is_wrot_sram = share_sram;
}

unsigned int use_wrot_sram(void)
{
	return is_wrot_sram;
}

void mtk_drm_update_rdma_golden_setting(struct cmdq_pkt *cmdq_handle,
	struct drm_crtc *crtc)
{
	/* Update RDMA golden setting */
	struct mtk_drm_crtc *mtk_crtc =
		to_mtk_crtc(crtc);
	unsigned int i, j;
	struct mtk_ddp_comp *comp;
	struct mtk_ddp_config cfg;

	cfg.w = crtc->state->adjusted_mode.hdisplay;
	cfg.h = crtc->state->adjusted_mode.vdisplay;
	cfg.vrefresh =
		drm_mode_vrefresh(&crtc->state->adjusted_mode);
	cfg.bpc = mtk_crtc->bpc;
	cfg.p_golden_setting_context =
		__get_golden_setting_context(mtk_crtc);
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_ddp_comp_io_cmd(comp, cmdq_handle,
			MTK_IO_CMD_RDMA_GOLDEN_SETTING, &cfg);
}

static void _share_wrot_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

int32_t _acquire_wrot_resource(enum CMDQ_EVENT_ENUM resourceEvent)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(share_sram_crtc);
	struct cmdq_pkt *handle;
	struct mtk_cmdq_cb_data *cb_data;

	DDPMSG("%s +\n", __func__);
	is_wrot_sram_available = 1;
	if (!share_sram_crtc || !mtk_crtc->enabled
		|| !(share_sram_crtc->state->active)) {
		DDPMSG("%s share_sram_crtc is NULL or not enable\n", __func__);
		return -EINVAL;
	}

	if (use_wrot_sram()) {
		DDPMSG("%s wrot sram already used.\n", __func__);
		return -EINVAL;
	}

	if (!is_wrot_sram_enabled) {
		DDPMSG("%s wrot sram is disabled.\n", __func__);
		return -EINVAL;
	}

	mtk_crtc_pkt_create(&handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);
	mtk_crtc_wait_frame_done(mtk_crtc, handle, DDP_FIRST_PATH, 0);
	mtk_drm_update_rdma_golden_setting(handle, share_sram_crtc);

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDPMSG("%s:cb data creation failed\n", __func__);
		return -EINVAL;
	}

	cb_data->cmdq_handle = handle;
	cmdq_pkt_flush_threaded(handle, _share_wrot_cb, cb_data);
	return 0;
}

static int32_t _release_wrot_resource(enum CMDQ_EVENT_ENUM resourceEvent)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(share_sram_crtc);
	struct cmdq_pkt *handle;
	struct mtk_cmdq_cb_data *cb_data;

	DDPMSG("%s +\n", __func__);
	is_wrot_sram_available = 0;
	if (!share_sram_crtc || !mtk_crtc->enabled
		|| !(share_sram_crtc->state->active)) {
		DDPMSG("%s share_sram_crtc is NULL or not enable\n", __func__);
		return -1;
	}
	if (!use_wrot_sram()) {
		DDPMSG("%s wrot sram already relased.\n", __func__);
		return -1;
	}

	if (!is_wrot_sram_enabled) {
		DDPMSG("%s wrot sram is disabled.\n", __func__);
		return -1;
	}
	mtk_crtc_pkt_create(&handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);
	mtk_crtc_wait_frame_done(mtk_crtc, handle, DDP_FIRST_PATH, 0);
	mtk_drm_update_rdma_golden_setting(handle, share_sram_crtc);

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDPMSG("%s:cb data creation failed\n", __func__);
		return -EINVAL;
	}

	cb_data->cmdq_handle = handle;
	cmdq_pkt_flush_threaded(handle, _share_wrot_cb, cb_data);
	return 0;
}

void mtk_drm_register_share_sram_callback(struct drm_crtc *crtc)
{
	if (!register_share_sram) {
		/* 1. register call back first */
		DDPINFO("%s mdp_set_resource_callback\n", __func__);
		mdp_set_resource_callback(CMDQ_SYNC_RESOURCE_WROT0,
				_acquire_wrot_resource, _release_wrot_resource);
		register_share_sram = 1;
		is_wrot_sram_available = 1;
	}
}

void mtk_drm_unregister_share_sram_callback(struct drm_crtc *crtc)
{
	if (register_share_sram) {
		/* 1. register call back first */
		DDPINFO("%s mdp_set_resource_callback\n", __func__);
		mdp_set_resource_callback(CMDQ_SYNC_RESOURCE_WROT0, NULL, NULL);
		register_share_sram = 0;
	}
}

/* vdo mode enter idle enter share sram */
void mtk_drm_enter_share_sram(struct drm_crtc *crtc, bool first)
{
	DDPMSG("%s +\n", __func__);
	share_sram_crtc = crtc;
	is_wrot_sram_enabled = 1;
	if (first) {
		/* 1. register call back first */
		mtk_drm_register_share_sram_callback(crtc);
		/* 2. try to allocate sram at the fisrt time */
		_acquire_wrot_resource(CMDQ_SYNC_RESOURCE_WROT0);
	}
}

void mtk_drm_leave_share_sram(struct drm_crtc *crtc,
	struct cmdq_pkt *cmdq_handle)
{
	DDPMSG("%s +\n", __func__);
	/* 1. unregister call back */
	is_wrot_sram_enabled = 0;
	/* try to release share sram */
	mtk_drm_update_rdma_golden_setting(cmdq_handle, share_sram_crtc);
}
#endif

static void mtk_drm_vdo_mode_enter_idle(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int i, j;
	struct cmdq_pkt *handle;
	struct cmdq_client *client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	struct mtk_ddp_comp *comp;

	mtk_crtc_pkt_create(&handle, crtc, client);

	if (mtk_drm_helper_get_opt(priv->helper_opt,
				   MTK_DRM_OPT_IDLEMGR_BY_REPAINT) &&
	    atomic_read(&state->plane_enabled_num) > 1) {
		atomic_set(&priv->idle_need_repaint, 1);
		drm_trigger_repaint(DRM_REPAINT_FOR_IDLE, crtc->dev);
	}

	if (mtk_drm_helper_get_opt(priv->helper_opt,
				   MTK_DRM_OPT_IDLEMGR_DISABLE_ROUTINE_IRQ)) {
		mtk_disp_mutex_inten_disable_cmdq(mtk_crtc->mutex[0], handle);
		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_ddp_comp_io_cmd(comp, handle, IRQ_LEVEL_IDLE, NULL);
	}

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (comp) {
#ifdef CONFIG_DRM_MTK_ICOM_ENABLE_LOW_POWER_IDLE_MODE
		mtk_ddp_comp_io_cmd(comp, handle, DSI_VFP_IDLE_MODE, NULL);
#else
		int en = 0;
		mtk_ddp_comp_io_cmd(comp, handle, DSI_VFP_IDLE_MODE, NULL);
		mtk_ddp_comp_io_cmd(comp, handle, DSI_LFR_SET, &en);
#endif
	}
	cmdq_pkt_flush(handle);
	cmdq_pkt_destroy(handle);
#ifndef CONFIG_DRM_MTK_ICOM_ENABLE_LOW_POWER_IDLE_MODE
	drm_crtc_vblank_off(crtc);
#endif
}

static void mtk_drm_cmd_mode_enter_idle(struct drm_crtc *crtc)
{
	mtk_drm_idlemgr_disable_crtc(crtc);
	lcm_fps_ctx_reset(crtc);
}

static void mtk_drm_vdo_mode_leave_idle(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int i, j;
	struct cmdq_pkt *handle;
	struct cmdq_client *client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	struct mtk_ddp_comp *comp;

	mtk_crtc_pkt_create(&handle, crtc, client);

	if (mtk_drm_helper_get_opt(priv->helper_opt,
				   MTK_DRM_OPT_IDLEMGR_DISABLE_ROUTINE_IRQ)) {
		mtk_disp_mutex_inten_enable_cmdq(mtk_crtc->mutex[0], handle);
		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_ddp_comp_io_cmd(comp, handle, IRQ_LEVEL_NORMAL, NULL);
	}

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (comp) {
#ifdef CONFIG_DRM_MTK_ICOM_ENABLE_LOW_POWER_IDLE_MODE
		mtk_ddp_comp_io_cmd(comp, handle, DSI_VFP_DEFAULT_MODE, NULL);
#else
		int en = 1;
		mtk_ddp_comp_io_cmd(comp, handle, DSI_VFP_DEFAULT_MODE, NULL);
		mtk_ddp_comp_io_cmd(comp, handle, DSI_LFR_SET, &en);
#endif
	}

	cmdq_pkt_flush(handle);
	cmdq_pkt_destroy(handle);
#ifndef CONFIG_DRM_MTK_ICOM_ENABLE_LOW_POWER_IDLE_MODE
	drm_crtc_vblank_on(crtc);
#endif
}

static void mtk_drm_cmd_mode_leave_idle(struct drm_crtc *crtc)
{
	mtk_drm_idlemgr_enable_crtc(crtc);
	lcm_fps_ctx_reset(crtc);
}

static void mtk_drm_idlemgr_enter_idle_nolock(struct drm_crtc *crtc)
{
	struct mtk_ddp_comp *output_comp;
	int index = drm_crtc_index(crtc);
	bool mode;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (!output_comp)
		return;

	mode = mtk_dsi_is_cmd_mode(output_comp);
	CRTC_MMP_EVENT_START(index, enter_idle, mode, 0);

	if (mode)
		mtk_drm_cmd_mode_enter_idle(crtc);
	else
		mtk_drm_vdo_mode_enter_idle(crtc);

	CRTC_MMP_EVENT_END(index, enter_idle, mode, 0);
}

static void mtk_drm_idlemgr_leave_idle_nolock(struct drm_crtc *crtc)
{
	struct mtk_ddp_comp *output_comp;
	int index = drm_crtc_index(crtc);
	bool mode;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (!output_comp)
		return;

	mode = mtk_dsi_is_cmd_mode(output_comp);
	CRTC_MMP_EVENT_START(index, leave_idle, mode, 0);

	if (mode)
		mtk_drm_cmd_mode_leave_idle(crtc);
	else
		mtk_drm_vdo_mode_leave_idle(crtc);

	CRTC_MMP_EVENT_END(index, leave_idle, mode, 0);
}

bool mtk_drm_is_idle(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_idlemgr *idlemgr = mtk_crtc->idlemgr;

	if (!idlemgr)
		return false;

	return idlemgr->idlemgr_ctx->is_idle;
}

void mtk_drm_idlemgr_kick_async(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct mtk_drm_idlemgr *idlemgr;

	if (crtc)
		mtk_crtc = to_mtk_crtc(crtc);

	if (mtk_crtc && mtk_crtc->idlemgr)
		idlemgr = mtk_crtc->idlemgr;
	else
		return;

	atomic_set(&idlemgr->kick_task_active, 1);
	wake_up_interruptible(&idlemgr->kick_wq);
}

void mtk_drm_idlemgr_kick(const char *source, struct drm_crtc *crtc,
			  int need_lock)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_idlemgr *idlemgr;
	struct mtk_drm_idlemgr_context *idlemgr_ctx;
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_IDLE_MGR))
		return;

	if (!mtk_crtc->idlemgr)
		return;
	idlemgr = mtk_crtc->idlemgr;
	idlemgr_ctx = idlemgr->idlemgr_ctx;

	/* get lock to protect idlemgr_last_kick_time and is_idle */
	if (need_lock)
		DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	if (idlemgr_ctx->is_idle) {
		DDPINFO("[LP] kick idle from [%s]\n", source);
		if (mtk_crtc->esd_ctx)
			atomic_set(&mtk_crtc->esd_ctx->target_time, 0);
		mtk_drm_idlemgr_leave_idle_nolock(crtc);
		idlemgr_ctx->is_idle = 0;

		/* wake up idlemgr process to monitor next idle state */
		wake_up_interruptible(&idlemgr->idlemgr_wq);
	}

	/* update kick timestamp */
	idlemgr_ctx->idlemgr_last_kick_time = sched_clock();

	if (need_lock)
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
}

unsigned int mtk_drm_set_idlemgr(struct drm_crtc *crtc, unsigned int flag,
				 bool need_lock)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_idlemgr *idlemgr = mtk_crtc->idlemgr;
	unsigned int old_flag;

	if (!idlemgr)
		return 0;

	old_flag = atomic_read(&idlemgr->idlemgr_task_active);

	if (flag) {
		DDPINFO("[LP] enable idlemgr\n");
		atomic_set(&idlemgr->idlemgr_task_active, 1);
		wake_up_interruptible(&idlemgr->idlemgr_wq);
	} else {
		DDPINFO("[LP] disable idlemgr\n");
		atomic_set(&idlemgr->idlemgr_task_active, 0);
		mtk_drm_idlemgr_kick(__func__, crtc, need_lock);
	}

	return old_flag;
}

unsigned long long
mtk_drm_set_idle_check_interval(struct drm_crtc *crtc,
				unsigned long long new_interval)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned long long old_interval = 0;

	if (!(mtk_crtc && mtk_crtc->idlemgr && mtk_crtc->idlemgr->idlemgr_ctx))
		return 0;

	old_interval = mtk_crtc->idlemgr->idlemgr_ctx->idle_check_interval;
	mtk_crtc->idlemgr->idlemgr_ctx->idle_check_interval = new_interval;

	return old_interval;
}

unsigned long long
mtk_drm_get_idle_check_interval(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!(mtk_crtc && mtk_crtc->idlemgr && mtk_crtc->idlemgr->idlemgr_ctx))
		return 0;

	return mtk_crtc->idlemgr->idlemgr_ctx->idle_check_interval;
}

static int mtk_drm_idlemgr_get_rsz_ratio(struct mtk_crtc_state *state)
{
	int src_w = state->rsz_src_roi.width;
	int src_h = state->rsz_src_roi.height;
	int dst_w = state->rsz_dst_roi.width;
	int dst_h = state->rsz_dst_roi.height;
	int ratio_w, ratio_h;

	if (src_w == 0 || src_h == 0)
		return 100;

	ratio_w = dst_w * 100 / src_w;
	ratio_h = dst_h * 100 / src_h;

	return ((ratio_w > ratio_h) ? ratio_w : ratio_h);
}

static bool is_yuv(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_YUV420:
	case DRM_FORMAT_YVU420:
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_YVYU:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_VYUY:
		return true;
	default:
		break;
	}

	return false;
}

static bool mtk_planes_is_yuv_fmt(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int i;

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i].base;
		struct mtk_plane_state *plane_state =
			to_mtk_plane_state(plane->state);
		struct mtk_plane_pending_state *pending = &plane_state->pending;
		unsigned int fmt = pending->format;

		if (pending->enable && is_yuv(fmt))
			return true;
	}

	return false;
}

static int mtk_drm_async_kick_idlemgr_thread(void *data)
{
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_idlemgr *idlemgr = mtk_crtc->idlemgr;
	int ret = 0;

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(
			idlemgr->kick_wq,
			atomic_read(&idlemgr->kick_task_active));
#ifdef CONFIG_DRM_MTK_ICOM_HANDLE_WAIT_EVENT_INTERRUPTIBLE
		if (ret < 0) {
			DDPMSG("[%s][%d] wait_event_interruptible return %pe !!!\n", __func__, __LINE__, ERR_PTR(ret));
			continue;
		}
#endif

		atomic_set(&idlemgr->kick_task_active, 0);

		mtk_drm_idlemgr_kick(__func__, crtc, true);
	}

	return 0;
}

static int mtk_drm_idlemgr_monitor_thread(void *data)
{
	int ret = 0;
	unsigned long long t_idle;
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_idlemgr *idlemgr = mtk_crtc->idlemgr;
	struct mtk_drm_idlemgr_context *idlemgr_ctx = idlemgr->idlemgr_ctx;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_crtc_state *mtk_state = NULL;
	struct drm_vblank_crtc *vblank = NULL;
	int crtc_id = drm_crtc_index(crtc);

	msleep(16000);
	while (1) {
		ret = wait_event_interruptible(
			idlemgr->idlemgr_wq,
			atomic_read(&idlemgr->idlemgr_task_active));

#ifdef CONFIG_DRM_MTK_ICOM_HANDLE_WAIT_EVENT_INTERRUPTIBLE
		if (ret < 0) {
			DDPMSG("[%s][%d] wait_event_interruptible return %pe !!!\n", __func__, __LINE__, ERR_PTR(ret));
			continue;
		}
#endif
		msleep_interruptible(idlemgr_ctx->idle_check_interval);

		DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

		if (!mtk_crtc->enabled) {
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
			ret = mtk_crtc_wait_status(crtc, 1, MAX_SCHEDULE_TIMEOUT);
			if (ret < 0)
				DDPPR_ERR("%s mtk_crtc_wait_status fail%d\n", __func__, ret);
			continue;
		}

		if (mtk_crtc_is_frame_trigger_mode(crtc) &&
				atomic_read(&priv->crtc_rel_present[crtc_id]) <
				atomic_read(&priv->crtc_present[crtc_id])) {
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
			continue;
		}

		if (crtc->state) {
			mtk_state = to_mtk_crtc_state(crtc->state);
			if (mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
				DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__,
						__LINE__);
				continue;
			}
			/* do not enter VDO idle when rsz ratio >= 2.5;
			 * And When layer fmt is YUV in VP scenario, it
			 * will flicker into idle repaint, so let it not
			 * into idle repaint as workaround.
			 */
			if (mtk_crtc_is_frame_trigger_mode(crtc) == 0 &&
				((mtk_drm_idlemgr_get_rsz_ratio(mtk_state) >=
				MAX_ENTER_IDLE_RSZ_RATIO) ||
				mtk_planes_is_yuv_fmt(crtc))) {
				DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__,
						__LINE__);
				continue;
			}
		}

		if (idlemgr_ctx->is_idle
			|| mtk_crtc_is_dc_mode(crtc)
			|| mtk_crtc->sec_on
			|| !priv->already_first_config) {
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
			continue;
		}

		t_idle = local_clock() - idlemgr_ctx->idlemgr_last_kick_time;
		if (t_idle < idlemgr_ctx->idle_check_interval * 1000 * 1000) {
			/* kicked in idle_check_interval msec, it's not idle */
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
			continue;
		}
		/* double check if dynamic switch on/off */
		if (atomic_read(&idlemgr->idlemgr_task_active)) {
			crtc_id = drm_crtc_index(crtc);
			vblank = &crtc->dev->vblank[crtc_id];

			/* enter idle state */
			if (!vblank || atomic_read(&vblank->refcount) == 0) {
				DDPINFO("[LP] enter idle\n");
				mtk_drm_idlemgr_enter_idle_nolock(crtc);
				idlemgr_ctx->is_idle = 1;
			} else {
				idlemgr_ctx->idlemgr_last_kick_time =
					sched_clock();
			}
		}

		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

#ifdef CONFIG_DRM_MTK_ICOM_HANDLE_WAIT_EVENT_INTERRUPTIBLE
		do {
			ret = wait_event_interruptible(idlemgr->idlemgr_wq,
						!idlemgr_ctx->is_idle);
			if (ret < 0) {
				DDPMSG("[%s][%d] wait_event_interruptible return %pe !!!\n", __func__, __LINE__, ERR_PTR(ret));
			}
		} while (ret < 0);
#else
		wait_event_interruptible(idlemgr->idlemgr_wq,
					 !idlemgr_ctx->is_idle);
#endif

		if (kthread_should_stop())
			break;
	}

	return 0;
}

int mtk_drm_idlemgr_init(struct drm_crtc *crtc, int index)
{
#define LEN 50
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_idlemgr *idlemgr;
	struct mtk_drm_idlemgr_context *idlemgr_ctx;
	char name[LEN];

	idlemgr = kzalloc(sizeof(struct mtk_drm_idlemgr), GFP_KERNEL);
	if (!idlemgr) {
		DDPPR_ERR("struct mtk_drm_idlemgr allocate fail\n");
		return -ENOMEM;
	}

	idlemgr_ctx = kzalloc(sizeof(struct mtk_drm_idlemgr_context), GFP_KERNEL);
	if (!idlemgr_ctx) {
		DDPPR_ERR("struct mtk_drm_idlemgr_context allocate fail\n");
		kfree(idlemgr);
		return -ENOMEM;
	}

	idlemgr->idlemgr_ctx = idlemgr_ctx;
	mtk_crtc->idlemgr = idlemgr;

	idlemgr_ctx->session_mode_before_enter_idle = MTK_DRM_SESSION_INVALID;
	idlemgr_ctx->is_idle = 0;
	idlemgr_ctx->enterulps = 0;
	idlemgr_ctx->idlemgr_last_kick_time = ~(0ULL);
	idlemgr_ctx->cur_lp_cust_mode = 0;
	idlemgr_ctx->idle_check_interval = 50;

	if (snprintf(name, LEN, "mtk_drm_disp_idlemgr-%d", index) < 0)
		DDPPR_ERR("%s:%d snprintf fail\n", __func__, __LINE__);
	idlemgr->idlemgr_task =
		kthread_create(mtk_drm_idlemgr_monitor_thread, crtc, name);
	init_waitqueue_head(&idlemgr->idlemgr_wq);
	atomic_set(&idlemgr->idlemgr_task_active, 1);

	wake_up_process(idlemgr->idlemgr_task);

	if (snprintf(name, LEN, "dis_ki-%d", index) < 0)
		DDPPR_ERR("%s:%d snprintf fail\n", __func__, __LINE__);
	idlemgr->kick_task =
		kthread_create(mtk_drm_async_kick_idlemgr_thread, crtc, name);
	init_waitqueue_head(&idlemgr->kick_wq);
	atomic_set(&idlemgr->kick_task_active, 0);

	wake_up_process(idlemgr->kick_task);

	return 0;
}

static void mtk_drm_idlemgr_disable_connector(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (output_comp)
		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_DISABLE, NULL);
}

static void mtk_drm_idlemgr_enable_connector(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (output_comp)
		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_ENABLE, NULL);
}

static void mtk_drm_idlemgr_disable_crtc(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned int crtc_id = drm_crtc_index(&mtk_crtc->base);
	bool mode = mtk_crtc_is_dc_mode(crtc);
	struct mtk_drm_private *priv =
				mtk_crtc->base.dev->dev_private;
	struct mtk_ddp_comp *output_comp = NULL;
	int en = 0;
	bool wait = true;

	DDPINFO("%s, crtc%d+\n", __func__, crtc_id);

	if (mode) {
		DDPINFO("crtc%d mode:%d bypass enter idle\n", crtc_id, mode);
		DDPINFO("crtc%d do %s-\n", crtc_id, __func__);
		return;
	}

	if (mtk_crtc->is_mml) {
		struct cmdq_pkt *cmdq_handle;

		mtk_crtc_pkt_create(&cmdq_handle, crtc, mtk_crtc->gce_obj.client[CLIENT_CFG]);
		if (cmdq_handle) {
			cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
			cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
			cmdq_pkt_flush(cmdq_handle);
			cmdq_pkt_destroy(cmdq_handle);
			wait = false;
			mtk_crtc->mml_ir_state = MML_IR_IDLE;
		}
	}

	/* 1. stop connector */
	mtk_drm_idlemgr_disable_connector(crtc);

	/* 2. stop CRTC */
	mtk_crtc_stop(mtk_crtc, wait);

	/* 3. disconnect addon module and recover config */
	mtk_crtc_disconnect_addon_module(crtc);

	/* 4. set HRT BW to 0 */
	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_MMQOS_SUPPORT))
		mtk_disp_set_hrt_bw(mtk_crtc, 0);

	/* 5. Release MMCLOCK request */
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (output_comp)
		mtk_ddp_comp_io_cmd(output_comp, NULL, SET_MMCLK_BY_DATARATE,
				&en);

	/* 6. disconnect path */
	mtk_crtc_disconnect_default_path(mtk_crtc);

	/* 7. power off all modules in this CRTC */
	mtk_crtc_ddp_unprepare(mtk_crtc);

	drm_crtc_vblank_off(crtc);

	mtk_crtc_vblank_irq(&mtk_crtc->base);
	mtk_gce_backup_slot_save(mtk_crtc, __func__);
	/* 8. power off MTCMOS */
	mtk_drm_top_clk_disable_unprepare(crtc->dev);

	/* 9. disable fake vsync if need */
	mtk_drm_fake_vsync_switch(crtc, false);

	/* 10. CMDQ power off */
	cmdq_mbox_disable(mtk_crtc->gce_obj.client[CLIENT_CFG]->chan);

	DDPINFO("crtc%d do %s-\n", crtc_id, __func__);
}

/* TODO: we should restore the current setting rather than default setting */
static void mtk_drm_idlemgr_enable_crtc(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned int crtc_id = drm_crtc_index(crtc);
	struct mtk_drm_private *priv =
			mtk_crtc->base.dev->dev_private;
	bool mode = mtk_crtc_is_dc_mode(crtc);
	struct mtk_ddp_comp *comp;
	unsigned int i, j;
	struct mtk_ddp_comp *output_comp = NULL;
	int en = 1;
	struct mtk_crtc_state *mtk_state = to_mtk_crtc_state(crtc->state);

	DDPINFO("crtc%d do %s+\n", crtc_id, __func__);

	if (mode) {
		DDPINFO("crtc%d mode:%d bypass exit idle\n", crtc_id, mode);
		DDPINFO("crtc%d do %s-\n", crtc_id, __func__);
		return;
	}

	/* 0. CMDQ power on */
	cmdq_mbox_enable(mtk_crtc->gce_obj.client[CLIENT_CFG]->chan);

	/* 1. power on mtcmos */
	mtk_drm_top_clk_prepare_enable(crtc->dev);
	mtk_gce_backup_slot_restore(mtk_crtc, __func__);

	/* 2. prepare modules would be used in this CRTC */
	mtk_drm_idlemgr_enable_connector(crtc);
	mtk_crtc_ddp_prepare(mtk_crtc);

	//mtk_gce_backup_slot_init(mtk_crtc);

#ifndef DRM_CMDQ_DISABLE
	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL)
		mtk_crtc_prepare_instr(crtc);
#endif

	/* 3. start trigger loop first to keep gce alive */
	if (crtc_id == 0) {
		if (mtk_crtc_with_sodi_loop(crtc) &&
			(!mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_start_sodi_loop(crtc);

		if (mtk_crtc_with_event_loop(crtc) &&
			(mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_start_event_loop(crtc);

		mtk_crtc_start_trig_loop(crtc);
		mtk_crtc_hw_block_ready(crtc);
	}

	/* 4. connect path */
	mtk_crtc_connect_default_path(mtk_crtc);

#ifdef SHARE_WROT_SRAM
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_SHARE_SRAM))
		mtk_drm_enter_share_sram(crtc, false);
#endif

	/* 5. config ddp engine & set dirty for cmd mode */
	mtk_crtc_config_default_path(mtk_crtc);

	/* 6. conect addon module and config */
	mtk_crtc_connect_addon_module(crtc);
	if (mtk_crtc->mml_ir_state == MML_IR_IDLE) {
		mtk_crtc_addon_connector_connect(crtc, NULL);
		mtk_crtc_alloc_sram(mtk_crtc, mtk_state->prop_val[CRTC_PROP_LYE_IDX]);
	} else {
		/* do not config mml addon module but dsc */
		mtk_crtc_connect_addon_module(crtc);
	}

	/* 7. restore OVL setting */
	mtk_crtc_restore_plane_setting(mtk_crtc);

	/* 8. Set QOS BW */
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_ddp_comp_io_cmd(comp, NULL, PMQOS_SET_BW, NULL);

	/* 9. restore HRT BW */
	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_MMQOS_SUPPORT))
		mtk_disp_set_hrt_bw(mtk_crtc,
			mtk_crtc->qos_ctx->last_hrt_req);

	/* 10. Request MMClock */
	mtk_crtc_attach_ddp_comp(crtc, mtk_crtc->ddp_mode, true);
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (output_comp)
		mtk_ddp_comp_io_cmd(output_comp, NULL, SET_MMCLK_BY_DATARATE,
				&en);

	/* 11. set vblank */
	drm_crtc_vblank_on(crtc);

	/* 12. enable fake vsync if need */
	mtk_drm_fake_vsync_switch(crtc, true);

	DDPINFO("crtc%d do %s-\n", crtc_id, __func__);
}

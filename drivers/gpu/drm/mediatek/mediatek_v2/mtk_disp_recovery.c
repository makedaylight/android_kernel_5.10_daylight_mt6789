// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/sched/clock.h>
#include <linux/delay.h>
#include <uapi/linux/sched/types.h>
#include <linux/pinctrl/consumer.h>

#ifndef DRM_CMDQ_DISABLE
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#else
#include "mtk-cmdq-ext.h"
#endif

#include "mtk_drm_drv.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_helper.h"
#include "mtk_drm_assert.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_trace.h"
#include "mtk_dump.h"
#include "mtk_disp_bdg.h"
#include "mtk_dsi.h"

#define ESD_TRY_CNT 5
#define ESD_CHECK_PERIOD 2000 /* ms */
static DEFINE_MUTEX(pinctrl_lock);

/* pinctrl implementation */
long _set_state(struct drm_crtc *crtc, const char *name)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct pinctrl_state *pState = 0;
	long ret = 0;

	mutex_lock(&pinctrl_lock);
	if (!priv->pctrl) {
		DDPPR_ERR("this pctrl is null\n");
		ret = -1;
		goto exit;
	}

	pState = pinctrl_lookup_state(priv->pctrl, name);
	if (IS_ERR(pState)) {
		DDPPR_ERR("lookup state '%s' failed\n", name);
		ret = PTR_ERR(pState);
		goto exit;
	}

	/* select state! */
	pinctrl_select_state(priv->pctrl, pState);

exit:
	mutex_unlock(&pinctrl_lock);
	return ret; /* Good! */
#else
	return 0; /* Good! */
#endif
}

long disp_dts_gpio_init(struct device *dev, struct mtk_drm_private *private)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
	long ret = 0;
	struct pinctrl *pctrl;

	/* retrieve */
	pctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pctrl)) {
		DDPPR_ERR("Cannot find disp pinctrl!");
		ret = PTR_ERR(pctrl);
		goto exit;
	}

	private->pctrl = pctrl;

exit:
	return ret;
#else
	return 0;
#endif
}

static inline int _can_switch_check_mode(struct drm_crtc *crtc,
					 struct mtk_panel_ext *panel_ext)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	int ret = 0;

	if (panel_ext->params->cust_esd_check == 0 &&
	    panel_ext->params->lcm_esd_check_table[0].cmd != 0 &&
	    mtk_drm_helper_get_opt(priv->helper_opt,
				   MTK_DRM_OPT_ESD_CHECK_SWITCH))
		ret = 1;

	return ret;
}

static inline int _lcm_need_esd_check(struct mtk_panel_ext *panel_ext)
{
	int ret = 0;

	if (panel_ext->params->esd_check_enable == 1)
		ret = 1;

	return ret;
}

static inline int need_wait_esd_eof(struct drm_crtc *crtc,
				    struct mtk_panel_ext *panel_ext)
{
	int ret = 1;

	/*
	 * 1.vdo mode
	 * 2.cmd mode te
	 */
	if (!mtk_crtc_is_frame_trigger_mode(crtc))
		ret = 0;

	if (panel_ext->params->cust_esd_check == 0)
		ret = 0;

	return ret;
}

static void esd_cmdq_timeout_cb(struct cmdq_cb_data data)
{
	struct drm_crtc *crtc = data.data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_ddp_comp *output_comp = NULL;

	if (!crtc) {
		DDPMSG("%s find crtc fail\n", __func__);
		return;
	}

	DDPMSG("[error]%s cmdq timeout out\n", __func__);
	DDPMSG("read flush fail\n");
	esd_ctx->chk_sta = 0xff;

	if (is_bdg_supported()) {
		if (mtk_crtc) {
			output_comp = mtk_ddp_comp_request_output(mtk_crtc);
			if (output_comp) {
				mtk_dump_analysis(output_comp);
				mtk_dump_reg(output_comp);
			}
		}
		bdg_dsi_dump_reg(DISP_BDG_DSI0);
	} else {
		mtk_drm_crtc_analysis(crtc);
		mtk_drm_crtc_dump(crtc);
	}
}


int _mtk_esd_check_read(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp = NULL;
	struct mtk_panel_ext *panel_ext = NULL;
	struct cmdq_pkt *cmdq_handle = NULL, *cmdq_handle2 = NULL;
	struct mtk_drm_esd_ctx *esd_ctx = NULL;
	int ret = 0;

	DDPINFO("[ESD%u]ESD read panel\n", drm_crtc_index(crtc));


	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s:invalid output comp\n", __func__);
		return -EINVAL;
	}

	if (mtk_drm_is_idle(crtc) && mtk_dsi_is_cmd_mode(output_comp))
		return 0;

	mtk_ddp_comp_io_cmd(output_comp, NULL, REQ_PANEL_EXT, &panel_ext);
	if (unlikely(!(panel_ext && panel_ext->params))) {
		DDPPR_ERR("%s:can't find panel_ext handle\n", __func__);
		return -EINVAL;
	}

	cmdq_handle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	cmdq_handle->err_cb.cb = esd_cmdq_timeout_cb;
	cmdq_handle->err_cb.data = crtc;

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 1);

	if (mtk_dsi_is_cmd_mode(output_comp)) {
		if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_SECOND_PATH, 0);
		else
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_FIRST_PATH, 0);

		cmdq_pkt_clear_event(cmdq_handle,
				     mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);

		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, ESD_CHECK_READ,
				    (void *)mtk_crtc);

		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	} else { /* VDO mode */
		if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_SECOND_PATH, 1);
		else
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_FIRST_PATH, 1);

		if (mtk_crtc->msync2.msync_on) {
			u32 vfp_early_stop = 1;

			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, DSI_VFP_EARLYSTOP,
							&vfp_early_stop);
		}

		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 2);

		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, DSI_STOP_VDO_MODE,
				    NULL);

		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 3);


		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, ESD_CHECK_READ,
				    (void *)mtk_crtc);

		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
				    DSI_START_VDO_MODE, NULL);

		mtk_disp_mutex_trigger(mtk_crtc->mutex[0], cmdq_handle);
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, COMP_REG_START,
				    NULL);
	}
	esd_ctx = mtk_crtc->esd_ctx;
	esd_ctx->chk_sta = 0;

	cmdq_pkt_flush(cmdq_handle);

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 4);


	mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_READ_EPILOG,
				    NULL);
	if (esd_ctx->chk_sta == 0xff) {
		ret = -1;
		if (need_wait_esd_eof(crtc, panel_ext)) {
			/* TODO: set ESD_EOF event through CPU is better */
			mtk_crtc_pkt_create(&cmdq_handle2, crtc,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);

			cmdq_pkt_set_event(
				cmdq_handle2,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
			cmdq_pkt_flush(cmdq_handle2);
			cmdq_pkt_destroy(cmdq_handle2);
		}
		goto done;
	}

	ret = mtk_ddp_comp_io_cmd(output_comp, NULL, ESD_CHECK_CMP,
				  (void *)mtk_crtc);
done:
	cmdq_pkt_destroy(cmdq_handle);
	return ret;
}

static irqreturn_t _esd_check_ext_te_irq_handler(int irq, void *data)
{
	struct mtk_drm_esd_ctx *esd_ctx = (struct mtk_drm_esd_ctx *)data;

	atomic_set(&esd_ctx->ext_te_event, 1);
	wake_up_interruptible(&esd_ctx->ext_te_wq);

	return IRQ_HANDLED;
}

static int _mtk_esd_check_eint(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	int ret = 1;

	DDPINFO("[ESD%u]ESD check eint\n", drm_crtc_index(crtc));

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s:invalid ESD context\n", __func__);
		return -EINVAL;
	}

	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_DUAL_TE) &&
			(atomic_read(&mtk_crtc->d_te.te_switched) == 1))
		atomic_set(&mtk_crtc->d_te.esd_te1_en, 1);
	else
		enable_irq(esd_ctx->eint_irq);

	/* check if there is TE in the last 2s, if so ESD check is pass */
	if (wait_event_interruptible_timeout(
		    esd_ctx->ext_te_wq,
		    atomic_read(&esd_ctx->ext_te_event),
		    HZ / 2) > 0)
		ret = 0;

	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_DUAL_TE) &&
			(atomic_read(&mtk_crtc->d_te.te_switched) == 1))
		atomic_set(&mtk_crtc->d_te.esd_te1_en, 0);
	else
		disable_irq(esd_ctx->eint_irq);
	atomic_set(&esd_ctx->ext_te_event, 0);

	return ret;
}

static void mtk_drm_release_esd_eint(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_ddp_comp *output_comp;

	if (unlikely(esd_ctx->eint_irq == -1)) {
		DDPPR_ERR("%s %u release eint_irq %d\n",
			__func__, drm_crtc_index(crtc), esd_ctx->eint_irq);
		return;
	}
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (IS_ERR_OR_NULL(output_comp)) {
		DDPPR_ERR("%s null output_comp\n", __func__);
		return;
	}

	free_irq(esd_ctx->eint_irq, esd_ctx);
	/*
	 * TE pinmux HW reg would be changed after free_irq, need to change pinctrl
	 * sw state as well. for next time set state to TE mode would not be skipped
	 * at the pinctrl_select_state.
	 */
	if (output_comp->id == DDP_COMPONENT_DSI0)
		_set_state(crtc, "mode_te_gpio");
	else
		_set_state(crtc, "mode_te_gpio1");

	esd_ctx->eint_irq = -1;
}

static int mtk_drm_request_eint(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_ddp_comp *output_comp;
	struct device_node *node;
	u32 ints[2] = {0, 0};
	char *compat_str = NULL;
	int ret = 0;

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s:invalid ESD context\n", __func__);
		return -EINVAL;
	}

	if (unlikely(esd_ctx->eint_irq != -1)) {
		DDPPR_ERR("%s: reentry with inited eint_irq %d\n", __func__, esd_ctx->eint_irq);
		return -EINVAL;
	}

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s:invalid output comp\n", __func__);
		return -EINVAL;
	}

	mtk_ddp_comp_io_cmd(output_comp, NULL, REQ_ESD_EINT_COMPAT,
			    &compat_str);
	if (unlikely(!compat_str)) {
		DDPPR_ERR("%s: invalid compat string\n", __func__);
		return -EINVAL;
	}
	node = of_find_compatible_node(NULL, NULL, compat_str);
	if (unlikely(!node)) {
		DDPPR_ERR("can't find ESD TE eint compatible node %s\n", compat_str);
		return -EINVAL;
	}

	of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
	esd_ctx->eint_irq = irq_of_parse_and_map(node, 0);

	ret = request_irq(esd_ctx->eint_irq, _esd_check_ext_te_irq_handler,
			  IRQF_TRIGGER_RISING, "ESD_TE-eint", esd_ctx);
	if (ret) {
		DDPPR_ERR("eint irq line %u not available! %d\n", esd_ctx->eint_irq, ret);
		return ret;
	}

	disable_irq(esd_ctx->eint_irq);

	/* mode_te_te1 mapping to non-primary display's TE */
	if (output_comp->id == DDP_COMPONENT_DSI0)
		_set_state(crtc, "mode_te_te");
	else
		_set_state(crtc, "mode_te_te1");

	/* in order to not all project DTS assign pinctrl mode 'mode_te_gpio', */
	/* add flag to decide need release eint during ESD check switch. */
	if (esd_ctx->need_release_eint == -1) {
		struct mtk_drm_private *priv = crtc->dev->dev_private;
		struct pinctrl_state *pState;

		if (!(priv && priv->pctrl)) {
			esd_ctx->need_release_eint = 0;
			return ret;
		}

		pState = pinctrl_lookup_state(priv->pctrl, "mode_te_gpio");
		if (IS_ERR(pState)) {
			esd_ctx->need_release_eint = 0;
			return ret;
		}

		pState = pinctrl_lookup_state(priv->pctrl, "mode_te_gpio1");
		if (IS_ERR(pState)) {
			esd_ctx->need_release_eint = 0;
			return ret;
		}
		esd_ctx->need_release_eint = 1;
	}

	return ret;
}

static int mtk_drm_esd_check(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_ext *panel_ext;
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	int ret = 0;

	CRTC_MMP_EVENT_START(drm_crtc_index(crtc), esd_check, 0, 0);

	if (mtk_crtc->enabled == 0) {
		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 0, 99);
		DDPINFO("[ESD] CRTC %d disable. skip esd check\n",
			drm_crtc_index(crtc));
		goto done;
	}

	panel_ext = mtk_crtc->panel_ext;
	if (unlikely(!(panel_ext && panel_ext->params))) {
		DDPPR_ERR("can't find panel_ext handle\n");
		ret = -EINVAL;
		goto done;
	}

	/* Check panel EINT */
	if (panel_ext->params->cust_esd_check == 0 &&
	    esd_ctx->chk_mode == READ_EINT) {
		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 1, 0);
		ret = _mtk_esd_check_eint(crtc);
	} else { /* READ LCM CMD  */
		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 0);
		ret = _mtk_esd_check_read(crtc);
	}

	/* switch ESD check mode */
	if (_can_switch_check_mode(crtc, panel_ext) &&
	    !mtk_crtc_is_frame_trigger_mode(crtc))
		esd_ctx->chk_mode =
			(esd_ctx->chk_mode == READ_EINT) ? READ_LCM : READ_EINT;

done:
	CRTC_MMP_EVENT_END(drm_crtc_index(crtc), esd_check, 0, ret);
	return ret;
}

static int mtk_drm_esd_recover(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	struct cmdq_pkt *cmdq_handle = NULL;
	int ret = 0;
	struct mtk_dsi *dsi = NULL;

	CRTC_MMP_EVENT_START(drm_crtc_index(crtc), esd_recovery, 0, 0);
	if (crtc->state && !crtc->state->active) {
		DDPMSG("%s: crtc is inactive\n", __func__);
		return 0;
	}
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s: invalid output comp\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 1);
	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		DDPMSG("%s, stop cancel all gce jobs\n", __func__);
		if (mtk_crtc->gce_obj.client[CLIENT_CFG])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_DSI_CFG])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_SUB_CFG])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SUB_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP]);
		if (mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP]);
	}
	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);
	if (IS_ERR_OR_NULL(cmdq_handle)) {
		DDPPR_ERR("%s: invalid output comp\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery,
		(unsigned long)cmdq_handle, 1);

	/* flush cmdq with stop_vdo_mode before it set DSI_START to 0 */
	if (mtk_crtc->is_mml) {
		DDPMSG("%s, %d mml stop\n", __func__, __LINE__);
		mtk_crtc_mml_racing_stop_sync(crtc, cmdq_handle);
	}

	if (mtk_crtc_is_frame_trigger_mode(crtc))
		mtk_ddp_comp_io_cmd(output_comp, NULL,
			CONNECTOR_PANEL_DISABLE_NOWAIT, NULL);
	else
		mtk_ddp_comp_io_cmd(output_comp, NULL,
			CONNECTOR_PANEL_DISABLE, NULL);

	mtk_gce_backup_slot_save(mtk_crtc, __func__);
	if (is_bdg_supported())
		bdg_common_deinit(DISP_BDG_DSI0, NULL);
	if (mtk_crtc_is_frame_trigger_mode(crtc))
		mtk_drm_crtc_disable(crtc, false, true);
	else
		mtk_drm_crtc_disable(crtc, true, true);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 2);

	if (mtk_drm_helper_get_opt(priv->helper_opt,
		MTK_DRM_OPT_MMQOS_SUPPORT)) {
		if (drm_crtc_index(crtc) == 0)
			mtk_disp_set_hrt_bw(mtk_crtc,
				mtk_crtc->qos_ctx->last_hrt_req);
	}

	mtk_drm_crtc_enable(crtc, true);
	mtk_gce_backup_slot_restore(mtk_crtc, __func__);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 3);

	if (mtk_crtc->is_mml)
		mtk_crtc_mml_racing_resubmit(crtc, NULL);
	if (is_bdg_supported()) {
		dsi = container_of(output_comp, struct mtk_dsi, ddp_comp);
		mtk_output_bdg_enable(dsi, false);
	}
	mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_PANEL_ENABLE, NULL);

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 4);

	mtk_crtc_hw_block_ready(crtc);
	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_ESD_EOF]);

		cmdq_pkt_flush(cmdq_handle);
	}
	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 5);

	cmdq_pkt_destroy(cmdq_handle);
done:
	CRTC_MMP_EVENT_END(drm_crtc_index(crtc), esd_recovery, 0, ret);

	return 0;
}

static int mtk_drm_esd_check_worker_kthread(void *data)
{
	struct sched_param param = {.sched_priority = 87};
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct mtk_drm_private *private = NULL;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct mtk_drm_esd_ctx *esd_ctx = NULL;
	int ret = 0;
	int i = 0;
	int recovery_flg = 0;
	bool check_te = false, te_timeout = false;
	unsigned int crtc_idx;

	sched_setscheduler(current, SCHED_RR, &param);

	if (!crtc) {
		DDPPR_ERR("%s invalid CRTC context, stop thread\n", __func__);

		return -EINVAL;
	}

	private = crtc->dev->dev_private;
	mtk_crtc = to_mtk_crtc(crtc);
	esd_ctx = mtk_crtc->esd_ctx;
	crtc_idx = drm_crtc_index(crtc);

	while (1) {
		msleep(ESD_CHECK_PERIOD);
		if (esd_ctx->chk_en == 0)
			continue;

		if (mtk_crtc_is_frame_trigger_mode(crtc) &&
			esd_ctx->chk_mode == READ_LCM)
			check_te = true;

		te_timeout = false;
		if (check_te == true) {
			ret = wait_event_interruptible(
				esd_ctx->check_task_wq,
				atomic_read(&esd_ctx->check_wakeup));
#ifdef CONFIG_DRM_MTK_ICOM_HANDLE_WAIT_EVENT_INTERRUPTIBLE
			if (ret < 0) {
				DDPMSG("[%s][%d] wait_event_interruptible return %pe !!!\n", __func__, __LINE__, ERR_PTR(ret));
				continue;
			}
#endif
			mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 1);
			atomic_set(&esd_ctx->int_te_event, 0);
			ret = wait_event_interruptible_timeout(
				esd_ctx->int_te_wq,
				atomic_read(&esd_ctx->int_te_event) ||
				atomic_read(&esd_ctx->check_wakeup) == 0,
				HZ);
			DDPINFO("%s, wait te time:%d, esd:%d\n",
				__func__, HZ - ret,  atomic_read(&esd_ctx->check_wakeup));
			if (ret < 0) {
#ifdef CONFIG_DRM_MTK_ICOM_HANDLE_WAIT_EVENT_INTERRUPTIBLE
				DDPMSG("[%s][%d] wait_event_interruptible_timeout return %pe !!!\n", __func__, __LINE__, ERR_PTR(ret));
#endif
				DDPINFO("[ESD]check thread waked up accidently\n");
				continue;
			}
			if (ret == 0 && esd_ctx->chk_active) {
#ifdef CONFIG_DRM_MTK_ICOM_HANDLE_WAIT_EVENT_INTERRUPTIBLE
				DDPMSG("[%s][%d] wait_event_interruptible_timeout time out !!!\n", __func__, __LINE__);
#endif
				DDPPR_ERR("%s: internal TE time out:%d, ret:%llu, esd:%d\n",
					__func__, HZ, ret,
					atomic_read(&esd_ctx->check_wakeup));
				te_timeout = true;
				DDPMSG("%s, stop cancel all gce jobs\n", __func__);
				if (mtk_crtc->gce_obj.client[CLIENT_CFG])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_CFG]);
				if (mtk_crtc->gce_obj.client[CLIENT_DSI_CFG])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);
				if (mtk_crtc->gce_obj.client[CLIENT_SUB_CFG])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SUB_CFG]);
				if (mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP]);
				if (mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP]);
			}
		} else {
			ret = wait_event_interruptible(
				esd_ctx->check_task_wq,
				atomic_read(&esd_ctx->check_wakeup) &&
				(atomic_read(&esd_ctx->target_time) ||
				 esd_ctx->chk_mode == READ_EINT));
			if (ret < 0) {
#ifdef CONFIG_DRM_MTK_ICOM_HANDLE_WAIT_EVENT_INTERRUPTIBLE
				DDPMSG("[%s][%d] wait_event_interruptible return %pe !!!\n", __func__, __LINE__, ERR_PTR(ret));
#endif
				DDPINFO("[ESD]check thread waked up accidently\n");
				continue;
			}
		}

		mutex_lock(&private->commit.lock);
		DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
		mtk_drm_trace_begin("esd");
		if (!mtk_drm_is_idle(crtc))
			atomic_set(&esd_ctx->target_time, 0);

		/* 1. esd check & recovery */
		if (!esd_ctx->chk_active) {
			DDPMSG("%s, %d, esd recover is disabled\n", __func__, __LINE__);
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
			mutex_unlock(&private->commit.lock);
			continue;
		}

		i = 0; /* repeat */
		do {
			if (te_timeout == false || i > 0) {
				ret = mtk_drm_esd_check(crtc);

				if (!ret) /* success */
					break;
			}

			DDPPR_ERR(
				"[ESD%u]esd check fail, will do esd recovery. te timeout:%d try=%d\n",
				crtc_idx, te_timeout, i);
			mtk_drm_esd_recover(crtc);
			recovery_flg = 1;
		} while (++i < ESD_TRY_CNT);

		if (ret != 0) {
			DDPPR_ERR(
				"[ESD%u]esd recover %d times failed, max:%d, disable esd check, ret:%d\n",
				crtc_idx, i, ESD_TRY_CNT, ret);
			mtk_disp_esd_check_switch(crtc, false);
			/*
			 * disable ESD check might release TE pin to GPIO mode when connector
			 * switch enabled, need restore TE pin back to TE mode.
			 */
			if (esd_ctx->need_release_eint == 1)
				mtk_drm_request_eint(crtc);
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
			mutex_unlock(&private->commit.lock);
			break;
		} else if (recovery_flg) {
			DDPINFO("[ESD%u] esd recovery success\n", crtc_idx);
			recovery_flg = 0;
		}
		mtk_drm_trace_end();
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		mutex_unlock(&private->commit.lock);

		/* 2. other check & recovery */
		if (kthread_should_stop())
			break;
	}
	return 0;
}

void mtk_disp_esd_check_switch(struct drm_crtc *crtc, bool enable)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;

	if (!mtk_drm_helper_get_opt(priv->helper_opt,
					   MTK_DRM_OPT_ESD_CHECK_RECOVERY))
		return;

	if (unlikely(!esd_ctx)) {
		DDPINFO("%s:invalid ESD context, crtc id:%d\n",
			__func__, drm_crtc_index(crtc));
		return;
	}
	DDPINFO("%s %u, esd chk active: %d\n", __func__, drm_crtc_index(crtc), enable);
	esd_ctx->chk_active = enable;

	/* release eint for connector switch; crtc might check differrent eint irq */
	if (esd_ctx->need_release_eint == 1) {
		if (!enable) /* release EINT if exist */
			mtk_drm_release_esd_eint(crtc);
		else /* request EINT before enable ESD check */
			mtk_drm_request_eint(crtc);
	}

	atomic_set(&esd_ctx->check_wakeup, enable);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check,
			0xffffffff, enable);
	if (enable)
		wake_up_interruptible(&esd_ctx->check_task_wq);
	else
		wake_up_interruptible(&esd_ctx->int_te_wq);
}

static void mtk_disp_esd_chk_deinit(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s:invalid ESD context\n", __func__);
		return;
	}

	/* Stop ESD task */
	mtk_disp_esd_check_switch(crtc, false);

	/* Stop ESD kthread */
	kthread_stop(esd_ctx->disp_esd_chk_task);

	kfree(esd_ctx);
}

static void mtk_disp_esd_chk_init(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_ext *panel_ext;
	struct mtk_drm_esd_ctx *esd_ctx;

	panel_ext = mtk_crtc->panel_ext;
	if (!(panel_ext && panel_ext->params)) {
		DDPMSG("can't find panel_ext handle\n");
		return;
	}

	if (_lcm_need_esd_check(panel_ext) == 0)
		return;

	DDPINFO("create ESD thread\n");
	/* primary display check thread init */
	esd_ctx = kzalloc(sizeof(*esd_ctx), GFP_KERNEL);
	if (!esd_ctx) {
		DDPPR_ERR("allocate ESD context failed!\n");
		return;
	}
	mtk_crtc->esd_ctx = esd_ctx;

	esd_ctx->eint_irq = -1;
	esd_ctx->need_release_eint = -1;
	esd_ctx->chk_en = 1;
	esd_ctx->disp_esd_chk_task = kthread_create(
		mtk_drm_esd_check_worker_kthread, crtc, "disp_echk");

	init_waitqueue_head(&esd_ctx->check_task_wq);
	init_waitqueue_head(&esd_ctx->ext_te_wq);
	init_waitqueue_head(&esd_ctx->int_te_wq);
	atomic_set(&esd_ctx->check_wakeup, 0);
	atomic_set(&esd_ctx->ext_te_event, 0);
	atomic_set(&esd_ctx->int_te_event, 0);
	atomic_set(&esd_ctx->target_time, 0);
	if (panel_ext->params->cust_esd_check == 1)
		esd_ctx->chk_mode = READ_LCM;
	else
		esd_ctx->chk_mode = READ_EINT;
	mtk_drm_request_eint(crtc);

	wake_up_process(esd_ctx->disp_esd_chk_task);
}

void mtk_disp_chk_recover_deinit(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;

	output_comp = (mtk_crtc) ? mtk_ddp_comp_request_output(mtk_crtc) : NULL;

	/* only support ESD check for DSI output interface */
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_ESD_CHECK_RECOVERY) &&
			output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI)
		mtk_disp_esd_chk_deinit(crtc);
}

void mtk_disp_chk_recover_init(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;

	output_comp = (mtk_crtc) ? mtk_ddp_comp_request_output(mtk_crtc) : NULL;

	/* only support ESD check for DSI output interface */
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_ESD_CHECK_RECOVERY) &&
			output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI)
		mtk_disp_esd_chk_init(crtc);
}

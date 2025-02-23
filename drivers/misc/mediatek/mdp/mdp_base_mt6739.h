/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/

#ifndef __MDP_BASE_H__
#define __MDP_BASE_H__

#define MDP_HW_CHECK

static const phys_addr_t mdp_base[ENGBASE_COUNT] = {
	[ENGBASE_MMSYS_CONFIG] = 0x14000000,
	[ENGBASE_MDP_RDMA0] = 0x14004000,
	[ENGBASE_MDP_RSZ0] = 0x14005000,
	[ENGBASE_MDP_RSZ1] = 0x14006000,
	[ENGBASE_MDP_WDMA] = 0x14007000,
	[ENGBASE_MDP_WROT0] = 0x14008000,
	[ENGBASE_MDP_TDSHP0] = 0x14009000,
	[ENGBASE_MDP_COLOR0] = 0x1400d000,
	[ENGBASE_MMSYS_MUTEX] = 0x14001000,
	[ENGBASE_IMGSYS] = 0x15000000,
	[ENGBASE_ISP_CAM_A] = 0x15004000,
	[ENGBASE_ISP_CAM_D] = 0x15007000,
};
#endif

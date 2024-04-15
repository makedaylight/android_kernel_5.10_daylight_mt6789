
# SPDX-License-Identifier: GPL-2.0
#
# Copyright (C) 2016 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dcm_v1/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/

ifneq (, $(filter $(strip $(CONFIG_MEDIATEK_DCM_MT6768)),m y))
obj-$(CONFIG_MEDIATEK_DCM_MT6768) += mediatek_dcm.o
mediatek_dcm-y += mtk_dcm.o
mediatek_dcm-y += mt6768_dcm_autogen.o
mediatek_dcm-y += mt6768_dcm_internal.o
else ifneq (, $(filter $(strip $(CONFIG_MEDIATEK_DCM_MT6885)),m y))
obj-$(CONFIG_MEDIATEK_DCM_MT6885) += mediatek_dcm.o
mediatek_dcm-y += mtk_dcm.o
mediatek_dcm-y += mt6885_dcm_autogen.o
mediatek_dcm-y += mt6885_dcm_internal.o
else ifneq (, $(filter $(strip $(CONFIG_MEDIATEK_DCM_V1)),m y))
obj-$(CONFIG_MEDIATEK_DCM_V1) += mediatek_dcm_v1.o
mediatek_dcm_v1-y += mtk_dcm.o
mediatek_dcm_v1-y += dcm_v1_autogen.o
mediatek_dcm_v1-y += dcm_v1_internal.o
endif
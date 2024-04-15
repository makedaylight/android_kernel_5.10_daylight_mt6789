# SPDX-License-Identifier: GPL-2.0

ifeq ($(CONFIG_MTK_GCOV_KERNEL),y)
	GCOV_PROFILE := y
endif

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/
ccflags-y += -I$(srctree)/include/memory/mediatek/
ccflags-y += -I$(srctree)/include/soc/mediatek/

NEED_TO_BE_REPORT :=

ifeq ($(CONFIG_MTK_DEVMPU_EMI),m)
NEED_TO_BE_REPORT := devmpu_emi working fine
else
NEED_TO_BE_REPORT := $(CONFIG_MTK_DEVMPU_EMI)
endif
all:
	@echo "devmpu_emi ="
	@echo $(NEED_TO_BE_REPORT)

obj-m += devmpu.o

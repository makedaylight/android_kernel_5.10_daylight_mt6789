# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2019 MediaTek Inc.

LOCAL_PATH := $(call my-dir)

ifeq ($(notdir $(LOCAL_PATH)),$(strip $(LINUX_KERNEL_VERSION)))

include $(LOCAL_PATH)/kenv.mk

ifeq ($(wildcard $(TARGET_PREBUILT_KERNEL)),)
##KERNEL_MAKE_DEPENDENCIES := $(shell find $(KERNEL_DIR) -name .git -prune -o -type f | sort)
KERNEL_MAKE_DEPENDENCIES := $(shell find -L $(KERNEL_DIR) -type f ! \( -path "*/*.swp" -prune -o -path "*/.vscode*" -prune -o -path "*/.git*" -prune -o -path "*/.svn" -prune -o -path "*/.vscode*" -prune -o -path "$(KERNEL_DIR)/arch/*" ! -path "$(KERNEL_DIR)/arch/arm64*" -prune -o -path "*/Documentation*" -prune -o -path "$(KERNEL_DIR)/samples*" -prune -o -path "$(KERNEL_DIR)/usr*" -prune -o -path "$(KERNEL_DIR)/virt*" -prune \) | sort)
##KERNEL_MAKE_DEPENDENCIES += $(shell find vendor/mediatek/kernel_modules -name .git -prune -o -type f | sort)
KERNEL_MAKE_DEPENDENCIES += $(shell find -L vendor/mediatek/kernel_modules -type f ! \( -path "*/*.swp" -prune -o -path "*/.vscode*" -prune -o -path "*/.git*" -prune -o -path "*/.svn" -prune -o -path "*/.vscode*" -prune \) | sort)
##$(foreach dep,$(KERNEL_MAKE_DEPENDENCIES),$(info $(dep)))
##$(error KERNEL_MAKE_DEPENDENCIES)

$(GEN_KERNEL_BUILD_CONFIG): PRIVATE_KERNEL_DEFCONFIG := $(KERNEL_DEFCONFIG)
$(GEN_KERNEL_BUILD_CONFIG): PRIVATE_KERNEL_DEFCONFIG_OVERLAYS := $(KERNEL_DEFCONFIG_OVERLAYS)
$(GEN_KERNEL_BUILD_CONFIG): PRIVATE_KERNEL_BUILD_CONFIG := $(REL_GEN_KERNEL_BUILD_CONFIG)
$(GEN_KERNEL_BUILD_CONFIG): PRIVATE_KERNEL_BUILD_CONFIG_OVERLAYS := $(addprefix $(KERNEL_DIR)/,$(KERNEL_BUILD_CONFIG_OVERLAYS))
$(GEN_KERNEL_BUILD_CONFIG): $(KERNEL_DIR)/kernel/configs/ext_modules.list
$(GEN_KERNEL_BUILD_CONFIG): $(KERNEL_DIR)/scripts/gen_build_config.py $(wildcard $(KERNEL_DIR)/build.config.*) $(build_config_file) $(KERNEL_CONFIG_FILE) $(LOCAL_PATH)/Android.mk
	$(hide) mkdir -p $(dir $@)
	$(hide) cd kernel && python $< --kernel-defconfig $(PRIVATE_KERNEL_DEFCONFIG) --kernel-defconfig-overlays "$(PRIVATE_KERNEL_DEFCONFIG_OVERLAYS)" --kernel-build-config-overlays "$(PRIVATE_KERNEL_BUILD_CONFIG_OVERLAYS)" -m $(TARGET_BUILD_VARIANT) -o $(PRIVATE_KERNEL_BUILD_CONFIG) && cd ..

$(TARGET_KERNEL_CONFIG): PRIVATE_KERNEL_OUT := $(REL_KERNEL_OUT)
$(TARGET_KERNEL_CONFIG): PRIVATE_DIST_DIR := $(REL_KERNEL_OUT)
$(TARGET_KERNEL_CONFIG): PRIVATE_CC_WRAPPER := $(CCACHE_EXEC)
$(TARGET_KERNEL_CONFIG): PRIVATE_KERNEL_BUILD_CONFIG := $(REL_GEN_KERNEL_BUILD_CONFIG)
$(TARGET_KERNEL_CONFIG): $(wildcard kernel/build/*.sh) $(GEN_KERNEL_BUILD_CONFIG) $(KERNEL_MAKE_DEPENDENCIES) | kernel-outputmakefile
	$(hide) mkdir -p $(dir $@)
	$(hide) cd kernel && ENABLE_GKI_CHECKER=$(ENABLE_GKI_CHECKER) CC_WRAPPER=$(PRIVATE_CC_WRAPPER) SKIP_MRPROPER=1 BUILD_CONFIG=$(PRIVATE_KERNEL_BUILD_CONFIG) OUT_DIR=$(PRIVATE_KERNEL_OUT) DIST_DIR=$(PRIVATE_DIST_DIR) POST_DEFCONFIG_CMDS="exit 0" ./build/build.sh && cd ..

ifeq (yes,$(strip $(BUILD_KERNEL)))
#.KATI_RESTAT: $(KERNEL_ZIMAGE_OUT)
$(KERNEL_ZIMAGE_OUT): PRIVATE_DIR := $(KERNEL_DIR)
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_OUT := $(REL_KERNEL_OUT)
$(KERNEL_ZIMAGE_OUT): PRIVATE_DIST_DIR := $(REL_KERNEL_OUT)
$(KERNEL_ZIMAGE_OUT): PRIVATE_CC_WRAPPER := $(CCACHE_EXEC)
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_BUILD_CONFIG := $(REL_GEN_KERNEL_BUILD_CONFIG)
ifeq (user,$(strip $(TARGET_BUILD_VARIANT)))
ifneq (,$(strip $(shell grep "^CONFIG_ABI_MONITOR\s*=\s*y" $(KERNEL_CONFIG_FILE))))
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_BUILD_SCRIPT := ./build/build_abi.sh
else
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_BUILD_SCRIPT := ./build/build.sh
endif
else
$(KERNEL_ZIMAGE_OUT): PRIVATE_KERNEL_BUILD_SCRIPT := ./build/build.sh
endif
ifneq (user,$(strip $(TARGET_BUILD_VARIANT)))
ifeq (yes,$(strip $(PCBA_BUILD)))
$(KERNEL_ZIMAGE_OUT): PRIVATE_PCBA_BUILD := PCBA_BUILD=yes
else
$(KERNEL_ZIMAGE_OUT): PRIVATE_PCBA_BUILD := PCBA_BUILD=no
endif
ifeq (yes,$(strip $(LTO_THIN_BUILD)))
$(KERNEL_ZIMAGE_OUT): PRIVATE_LTO_MODE := LTO=thin
else ifeq (yes,$(strip $(LTO_NONE_BUILD)))
$(KERNEL_ZIMAGE_OUT): PRIVATE_LTO_MODE := LTO=none
else
$(KERNEL_ZIMAGE_OUT): PRIVATE_LTO_MODE := LTO=full
endif
else
$(KERNEL_ZIMAGE_OUT): PRIVATE_LTO_MODE := LTO=full
endif
$(KERNEL_ZIMAGE_OUT): $(TARGET_KERNEL_CONFIG) $(KERNEL_MAKE_DEPENDENCIES)
	$(hide) mkdir -p $(dir $@)
	$(hide) cd kernel && CC_WRAPPER=$(PRIVATE_CC_WRAPPER) SKIP_MRPROPER=1 BUILD_CONFIG=$(PRIVATE_KERNEL_BUILD_CONFIG) OUT_DIR=$(PRIVATE_KERNEL_OUT) DIST_DIR=$(PRIVATE_DIST_DIR) SKIP_DEFCONFIG=1 $(PRIVATE_LTO_MODE) $(PRIVATE_PCBA_BUILD) $(PRIVATE_KERNEL_BUILD_SCRIPT) && cd ..
	$(hide) $(call fixup-kernel-cmd-file,$(KERNEL_OUT)/arch/$(KERNEL_TARGET_ARCH)/boot/compressed/.piggy.xzkern.cmd)

$(BUILT_KERNEL_TARGET): $(KERNEL_ZIMAGE_OUT) $(TARGET_KERNEL_CONFIG) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-target)

$(TARGET_PREBUILT_KERNEL): $(BUILT_KERNEL_TARGET) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-new-target)

endif#BUILD_KERNEL
endif #TARGET_PREBUILT_KERNEL is empty

ifeq (yes,$(strip $(BUILD_KERNEL)))
ifneq ($(strip $(TARGET_NO_KERNEL)),true)
$(INSTALLED_KERNEL_TARGET): $(BUILT_KERNEL_TARGET) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-target)
endif#TARGET_NO_KERNEL
endif#BUILD_KERNEL

.PHONY: kernel save-kernel kernel-savedefconfig kernel-menuconfig menuconfig-kernel savedefconfig-kernel clean-kernel
kernel: $(INSTALLED_KERNEL_TARGET)
save-kernel: $(TARGET_PREBUILT_KERNEL)

kernel-savedefconfig: $(TARGET_KERNEL_CONFIG)
	cp $(TARGET_KERNEL_CONFIG) $(KERNEL_CONFIG_FILE)

kernel-menuconfig:
	$(hide) mkdir -p $(KERNEL_OUT)
	$(PREBUILT_MAKE_PREFIX)/$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) menuconfig

menuconfig-kernel savedefconfig-kernel:
	$(hide) mkdir -p $(KERNEL_OUT)
	$(PREBUILT_MAKE_PREFIX)/$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) $(patsubst %config-kernel,%config,$@)

clean-kernel:
	$(hide) rm -rf $(KERNEL_OUT) $(INSTALLED_KERNEL_TARGET)

.PHONY: kernel-outputmakefile
kernel-outputmakefile: PRIVATE_DIR := $(KERNEL_DIR)
kernel-outputmakefile: PRIVATE_KERNEL_OUT := $(REL_KERNEL_OUT)/$(LINUX_KERNEL_VERSION)
kernel-outputmakefile:
	$(PREBUILT_MAKE_PREFIX)/$(MAKE) -C $(PRIVATE_DIR) O=$(PRIVATE_KERNEL_OUT) outputmakefile

### DTB build template
MTK_DTBIMAGE_DTS := $(addsuffix .dts,$(addprefix $(KERNEL_DIR)/arch/$(KERNEL_TARGET_ARCH)/boot/dts/,$(PLATFORM_DTB_NAME)))
include device/mediatek/build/core/build_dtbimage.mk

MTK_DTBOIMAGE_DTS := $(addsuffix .dts,$(addprefix $(KERNEL_DIR)/arch/$(KERNEL_TARGET_ARCH)/boot/dts/,$(PROJECT_DTB_NAMES)))
include device/mediatek/build/core/build_dtboimage.mk

endif #LINUX_KERNEL_VERSION

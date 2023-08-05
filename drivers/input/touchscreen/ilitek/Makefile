# MTK notifier
ifeq ($(CONFIG_DRM_MEDIATEK), y)
ccflags-$(CONFIG_DRM_MEDIATEK) += -I$(srctree)/drivers/gpu/drm/mediatek/
endif
ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

ccflags-y += -Wall

ccflags-y += -DILITEK_KERNEL_DRIVER

#Un-mark below item to enable FW upgrade on boot
ccflags-y += -DILITEK_BOOT_UPDATE

ILITEK_INTERFACE = i2c
#ILITEK_INTERFACE = spi


ifeq ($(ILITEK_INTERFACE), spi)
$(info start to build ilitek SPI driver)
ccflags-y += -DILITEK_SPI_INTERFACE
ilitek := ilitek_spi
else
$(info start to build ilitek I2C driver)
ilitek := ilitek_i2c
endif

obj-$(CONFIG_TOUCHSCREEN_ILITEK) += $(ilitek).o

$(ilitek)-y += \
	ilitek_main.o \
	ilitek_platform_init.o \
	ilitek_update.o \
	ilitek_tool.o \
	ilitek_protocol.o

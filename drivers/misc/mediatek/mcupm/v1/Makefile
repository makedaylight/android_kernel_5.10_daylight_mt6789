obj-$(CONFIG_MTK_TINYSYS_MCUPM_SUPPORT) += mcupm.o
mcupm-objs += mcupm_driver.o

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mcupm/include/v1
ccflags-y += -I$(srctree)/include/linux/soc/mediatek
ccflags-$(CONFIG_MTK_ENG_BUILD) += -DDEBUG
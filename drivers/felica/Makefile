#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2011 KYOCERA Corporation
# (C) 2012 KYOCERA Corporation
# (C) 2013 KYOCERA Corporation
#
KERNEL_DIR ?= $(ANDROID_BUILD_TOP)/../kernel
KERNEL_OUT ?= $(ANDROID_PRODUCT_OUT)/obj/KERNEL_OBJ

INSTALL_DIR ?= $(ANDROID_PRODUCT_OUT)/system/lib/modules

TARGET = felica
MFD_PATH = $(ANDROID_BUILD_TOP)/../kernel/drivers/felica

INC_PATH = $(MFD_PATH)
INC_PATH += $(ANDROID_BUILD_TOP)/../kernel/include
INC_PATH += $(ANDROID_BUILD_TOP)/../kernel/arch/arm/include/asm

SRC_DIR = $(MFD_PATH)
SOURCES = felica.c

obj-y := $(TARGET).o

MAKE_FLAGS := -C $(KERNEL_DIR) O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-
EXTRA_CFLAGS += $(addprefix -I, $(INC_PATH))


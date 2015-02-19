/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

/* Orient */
#define MXT_NORMAL		0x0
#define MXT_DIAGONAL		0x1
#define MXT_HORIZONTAL_FLIP	0x2
#define MXT_ROTATED_90_COUNTER	0x3
#define MXT_VERTICAL_FLIP	0x4
#define MXT_ROTATED_90		0x5
#define MXT_ROTATED_180		0x6
#define MXT_DIAGONAL_COUNTER	0x7

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	const u8 *config;
	size_t config_length;

	unsigned int x_size;
	unsigned int y_size;
/* H35 WS1 20120328 Touch A S */
	unsigned char orient;
/* H35 WS1 20120328 Touch A E */
	unsigned long irqflags;
	bool	i2c_pull_up;
	bool	digital_pwr_regulator;
	int reset_gpio;
	int irq_gpio;

/* H35 WS1 20120328 Touch M S */
#if 0
	u8(*read_chg) (void);
	int (*init_hw) (bool);
	int (*power_on) (bool);
#else /* 0 */
	int (*init_hw) (void);
	int (*reset_hw) (void);
#endif /* 0 */
/* H35 WS1 20120328 Touch M E */
};
/* H35 Diag 20120403 A S */
struct mxt_diag_event {
	int x;
	int y;
	int width;
};

struct mxt_diag_type {
	struct mxt_diag_event ts[10];
	int diag_count;
};
/* H35 Diag 20120403 A E */

/* H35 CDEV 20120328 A S */
#define IOC_MAGIC 't'

#define IOCTL_SET_CONF_STAT _IOW(IOC_MAGIC, 1, int)
#define IOCTL_GET_CONF_STAT _IOR(IOC_MAGIC, 2, int)
#define IOCTL_SET_LOG _IOW(IOC_MAGIC, 3, int)
/* H35 CDEV 20120328  A E */
/* H35 Diag 20120403 A S */
#define TS_NV_MAX_SIZE 1024
#define TS_NUM_OF_NV	3
struct mxt_nv_data {
	size_t size;
	u8 data[TS_NV_MAX_SIZE];
};

enum mxt_nv_type {
	MXT_CHARGE_C_NV = 0,
	MXT_CHARGE_A_NV,
	MXT_N_CHARGE_NV,
	MXT_NV_MAX,
};

#define IOCTL_LOAD_CHRG_C_NV _IOW(IOC_MAGIC, 4, struct mxt_nv_data)
#define IOCTL_LOAD_CHRG_A_NV _IOW(IOC_MAGIC, 5, struct mxt_nv_data)
#define IOCTL_LOAD_N_CHRG_NV _IOW(IOC_MAGIC, 6, struct mxt_nv_data)
#define IOCTL_SET_NV _IO(IOC_MAGIC, 7)
#define IOCTL_DIAG_START _IO(IOC_MAGIC, 0xA1)
#define IOCTL_MULTI_GET _IOR(IOC_MAGIC, 0xA2, struct mxt_diag_type)
#define IOCTL_COODINATE_GET _IOR(IOC_MAGIC, 0xA3, struct mxt_diag_type)
#define IOCTL_DIAG_END _IO(IOC_MAGIC, 0xA4)
#define IOCTL_DIAG_LOG_LEVEL _IOW(IOC_MAGIC, 0xA5, unsigned char)
#define IOCTL_DIAG_EVENT_CTRL _IOW(IOC_MAGIC, 0xA6, unsigned char)
/* H35 Diag 20120403 A E */

#endif /* __LINUX_ATMEL_MXT_TS_H */

/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/input/touchscreen/ts_ctrl.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/

#ifndef __LINUX_TS_CTRL_H
#define __LINUX_TS_CTRL_H

#include <linux/types.h>

#define TS_EVENT_ENABLE 0
#define TS_NV_MAX_SIZE 1024
#define TS_NUM_OF_NV	3

#define TS_N_CHARGE_NV_NAME	"mXT224S_non_charge_nv.bin"
#define TS_CHARGE_C_NV_NAME	"mXT224S_charge_c_nv.bin"
#define TS_CHARGE_A_NV_NAME	"mXT224S_charge_a_nv.bin"

enum ts_nv_type {
	TS_CHARGE_C_NV = 0,
	TS_CHARGE_A_S1_NV,
	TS_CHARGE_A_S2_NV,
	TS_DISCHARGE_NV,
	TS_WIRELESS_NV,
	TS_USB_OTG_NV,
	TS_INITIAL_VALUE_NV,
	TS_EXTENDED_NV,
	TS_NV_MAX,
};

enum ts_config_type {
	TS_CHARGE_CABLE = 0,
	TS_CHARGE_A_S1,
	TS_CHARGE_A_S2,
	TS_DISCHARGE,
	TS_WIRELESS,
	TS_USB_OTG,
	TS_INITIAL,
	TS_CONFIG_MAX,
};

struct ts_diag_event {
	int x;
	int y;
	int width;
};

struct ts_diag_type {
	struct ts_diag_event ts[10];
	int diag_count;
};

struct ts_nv_data {
	size_t size;
	char *data;
	enum ts_nv_type nv_type;
};

struct ts_config_nv {
	size_t size;
	u16 ver;
	u8 *data;
};
struct ts_log_data {
	int flag;
	int data;
};

extern struct ts_diag_type *diag_data;
extern struct mutex diag_lock;
extern struct mutex file_lock;
extern char ts_event_control;
extern char ts_log_level;
extern char ts_log_file_enable;
extern char ts_esd_recovery;
extern char ts_config_switching;

int ts_ctrl_init(struct cdev *device_cdev, const struct file_operations *fops);
int ts_ctrl_exit(struct cdev *device_cdev);

#define IOC_MAGIC 't'
#define IOCTL_SET_CONF_STAT _IOW(IOC_MAGIC, 1, enum ts_config_type)
#define IOCTL_SET_LOG _IOW(IOC_MAGIC, 3, struct ts_log_data)
#define IOCTL_SET_NV _IO(IOC_MAGIC, 8)
#define IOCTL_LOAD_NV _IOW(IOC_MAGIC, 17, struct ts_nv_data)
#define IOCTL_DIAG_START _IO(IOC_MAGIC, 0xA1)
#define IOCTL_MULTI_GET _IOR(IOC_MAGIC, 0xA2, struct ts_diag_type)
#define IOCTL_COODINATE_GET _IOR(IOC_MAGIC, 0xA3, struct ts_diag_type)
#define IOCTL_DIAG_END _IO(IOC_MAGIC, 0xA4)
#define IOCTL_DIAG_LOG_LEVEL _IOW(IOC_MAGIC, 0xA5, unsigned char)
#define IOCTL_DIAG_EVENT_CTRL _IOW(IOC_MAGIC, 0xA6, unsigned char)
#define IOCTL_DIAG_RESET_HW _IO(IOC_MAGIC, 0xA7)
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT_KC
#define IOCTL_GET_GOLDEN_REFERENCE _IOWR(IOC_MAGIC, 0xA8, unsigned char)
#endif
#define IOCTL_DIAG_GET_C_REFERENCE _IOWR(IOC_MAGIC, 0xA9, unsigned char)
#define IOCTL_DIAG_GET_DELTA _IOR(IOC_MAGIC, 0xA0, unsigned char)

#endif /* __LINUX_TS_CTRL_H */

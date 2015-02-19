/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */

#include "msm_sensor.h"
#include "msm.h"
#include "msm_ispif.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/workqueue.h>

#define SENSOR_NAME "rj6cba100"
#define PLATFORM_DRIVER_NAME "msm_camera_rj6cba100"
#define rj6cba100_obj rj6cba100_##obj

/* add special control */
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <mach/irqs.h>
#include <mach/rpm-regulator.h>
#include <mach/msm_memtypes.h>


DEFINE_MUTEX(rj6cba100_mut);
int32_t msm_sensor_rj6cba100_set_white_balance(struct msm_sensor_ctrl_t *s_ctrl, int8_t wb);
static struct msm_sensor_ctrl_t rj6cba100_s_ctrl;

static volatile int rj6cba100_power_on_flg = 0;

static int32_t pre_exp_setting = 0;
#define RJ6CBA100_EXPOSURE_SETTING_SIZE 2
#define RJ6CBA100_EXPOSURE_VALUE_STEP 1
#define RJ6CBA100_EXPOSURE_VALUE_MAX 6
#define RJ6CBA100_EXPOSURE_TBL_SIZE (((RJ6CBA100_EXPOSURE_VALUE_MAX) * 2) + 1)

#define RJ6CBA100_WB_AUTO            0
#define RJ6CBA100_WB_DAYLIGHT        1
#define RJ6CBA100_WB_FLUORESCENT     2
#define RJ6CBA100_WB_CLOUDY_DAYLIGHT 3
#define RJ6CBA100_WB_INCANDESCENT    4
#define RJ6CBA100_WB_MAX             5
#define RJ6CBA100_WB_NONE             0xFF
#define RJ6CBA100_WB_SETTING_SIZE 27

#define RJ6CBA100_EFFECT_OFF      0
#define RJ6CBA100_EFFECT_MONO     1
#define RJ6CBA100_EFFECT_NEGATIVE 2
#define RJ6CBA100_EFFECT_SEPIA    3
#define RJ6CBA100_EFFECT_POSTER   4
#define RJ6CBA100_EFFECT_MAX      5
#define RJ6CBA100_EFFECT_NONE     0xFF
#define RJ6CBA100_EFFECT_SETTING_SIZE 11

#define RJ6CBA100_AGC_THRESHOLD_5LX   0x03FF
#define RJ6CBA100_ADVF_THRESHOLD_5LX  0xFFFF
#define RJ6CBA100_AGC_THRESHOLD_50LX  0x03FF
#define RJ6CBA100_ADVF_THRESHOLD_50LX 0xFFFF

static int8_t rj6cba100_wb = RJ6CBA100_WB_NONE;
static int8_t rj6cba100_wb_wait = RJ6CBA100_WB_NONE;

static int8_t rj6cba100_effect = RJ6CBA100_EFFECT_NONE;

static uint32_t rj6cba100_g_exposure_time = 0;

/****************** mod special data for rj6cba100 ******************/
static struct msm_camera_i2c_reg_conf rj6cba100_reset_settings[] = {
	{ 0x12, 0x80 },
};
static struct msm_camera_i2c_reg_conf rj6cba100_recommend_settings[] = {
	{ 0x0E, 0x08 },
	{ 0x1E, 0xB3 },
	{ 0x48, 0x42 },
	{ 0xFF, 0x01 },
	{ 0xAE, 0xA0 },
	{ 0xA8, 0x25 },
	{ 0xB4, 0xC0 },
	{ 0xB5, 0x40 },
	{ 0x86, 0x48 },
	{ 0x87, 0x50 },
	{ 0xFF, 0x00 },
	{ 0x0C, 0xC0 },
	{ 0x62, 0x10 },
	{ 0x12, 0x00 },
	{ 0x17, 0x69 },
	{ 0x18, 0xA4 },
	{ 0x19, 0x0C },
	{ 0x1A, 0xF6 },
	{ 0x3E, 0x30 },
	{ 0x64, 0x0A },
	{ 0xFF, 0x01 },
	{ 0xB4, 0xC0 },
	{ 0xFF, 0x00 },
	{ 0x67, 0x20 },
	{ 0x81, 0x3F },
	{ 0xCC, 0x02 },
	{ 0xCD, 0x80 },
	{ 0xCE, 0x01 },
	{ 0xCF, 0xE0 },
	{ 0xC8, 0x02 },
	{ 0xC9, 0x80 },
	{ 0xCA, 0x01 },
	{ 0xCB, 0xE0 },
	{ 0xD0, 0x48 },
	{ 0x82, 0x03 },
	{ 0x70, 0x00 },
	{ 0x71, 0x34 },
	{ 0x74, 0x28 },
	{ 0x75, 0x98 },
	{ 0x76, 0x00 },
	{ 0x77, 0x64 },
	{ 0x78, 0x01 },
	{ 0x79, 0xC2 },
	{ 0x7A, 0x4E },
	{ 0x7B, 0x1F },
	{ 0x7C, 0x00 },
	{ 0x11, 0x00 },
	{ 0x20, 0x00 },
	{ 0x21, 0x23 },
	{ 0x50, 0x9A },
	{ 0x51, 0x80 },
	{ 0x4C, 0x7D },
	{ 0x80, 0x7F },
	{ 0x85, 0x90 },
	{ 0x86, 0x00 },
	{ 0x87, 0x00 },
	{ 0x88, 0x10 },
	{ 0x89, 0x22 },
	{ 0x8A, 0x16 },
	{ 0x8B, 0x10 },
	{ 0xBB, 0x80 },
	{ 0xBC, 0x62 },
	{ 0xBD, 0x1E },
	{ 0xBE, 0x26 },
	{ 0xBF, 0x7B },
	{ 0xC0, 0xAC },
	{ 0xC1, 0x1E },
	{ 0xB7, 0x02 },
	{ 0xB8, 0x0A },
	{ 0xB9, 0x02 },
	{ 0xBA, 0x20 },
	{ 0x5A, 0x14 },
	{ 0x5B, 0xA2 },
	{ 0x5C, 0x70 },
	{ 0x5D, 0x20 },
	{ 0x24, 0x78 },
	{ 0x25, 0x68 },
	{ 0x26, 0xB3 },
	{ 0xA3, 0x17 },
	{ 0xA4, 0x20 },
	{ 0xA5, 0x34 },
	{ 0xA6, 0x57 },
	{ 0xA7, 0x68 },
	{ 0xA8, 0x77 },
	{ 0xA9, 0x85 },
	{ 0xAA, 0x91 },
	{ 0xAB, 0x9C },
	{ 0xAC, 0xA7 },
	{ 0xAD, 0xB9 },
	{ 0xAE, 0xC8 },
	{ 0xAF, 0xDD },
	{ 0xB0, 0xEA },
	{ 0xB1, 0xF1 },
	{ 0xB2, 0x06 },
	{ 0x8C, 0x5E },
	{ 0x8D, 0x11 },
	{ 0x8E, 0x12 },
	{ 0x8F, 0x19 },
	{ 0x90, 0x50 },
	{ 0x91, 0x20 },
	{ 0x92, 0x96 },
	{ 0x93, 0x80 },
	{ 0x94, 0x13 },
	{ 0x95, 0x1B },
	{ 0x96, 0xFF },
	{ 0x97, 0x00 },
	{ 0x98, 0x3D },
	{ 0x99, 0x36 },
	{ 0x9A, 0x51 },
	{ 0x9B, 0x43 },
	{ 0x9C, 0xF0 },
	{ 0x9D, 0xF0 },
	{ 0x9E, 0xF0 },
	{ 0x9F, 0xFF },
	{ 0xA0, 0x68 },
	{ 0xA1, 0x62 },
	{ 0xA2, 0x0E },
	{ 0xB4, 0x26 },
	{ 0xB6, 0x04 },
	{ 0x81, 0x02 },
	{ 0xD2, 0x07 },
	{ 0xD3, 0x08 },
	{ 0xD4, 0x24 },
	{ 0xD5, 0x20 },
	{ 0x81, 0x04 },
	{ 0xDC, 0x00 },
	{ 0xD2, 0x04 },
	{ 0xD3, 0x00 },
	{ 0x81, 0x02 },
	{ 0x28, 0x00 },
	{ 0xD2, 0x06 },
	{ 0xDA, 0x80 },
	{ 0xDB, 0x80 },
	{ 0xD8, 0x50 },
	{ 0xD9, 0x50 },
	{ 0xB5, 0x06 },
	{ 0x14, 0x0B },
	{ 0x81, 0x3F },
	{ 0x30, 0xE7 },
	{ 0x31, 0x80 },
	{ 0x2A, 0x30 },
	{ 0x2B, 0x2E },
	{ 0x15, 0xA0 },
	{ 0xB3, 0x00 },
//	{ 0x0E, 0x00 },
};


static struct msm_camera_i2c_reg_conf rj6cba100_start_stream_settings[] = {
//	{0x38, 0x00},
	{0x0E, 0x00},
};

static struct msm_camera_i2c_reg_conf rj6cba100_stop_stream_settings[] = {
//	{0x38, 0x60},
	{0x0E, 0x08},
};

static struct v4l2_subdev_info rj6cba100_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array rj6cba100_init_conf[] = {
	{&rj6cba100_reset_settings[0],
	ARRAY_SIZE(rj6cba100_reset_settings), 10, MSM_CAMERA_I2C_BYTE_DATA},
	{&rj6cba100_recommend_settings[0],
	ARRAY_SIZE(rj6cba100_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};


static struct msm_camera_i2c_reg_conf rj6cba100_snap_settings[] = {
	{ 0x15, 0xA0 },
	{ 0x11, 0x00 },
	{ 0x21, 0x23 },
	{ 0x50, 0x9A },
	{ 0x51, 0x80 },
	{ 0x20, 0x00 },
	{ 0x2A, 0x30 },
	{ 0x2C, 0x00 },
	{ 0x2D, 0x00 },
	{ 0x2E, 0x00 },
};

static struct msm_camera_i2c_reg_conf rj6cba100_prev_settings[] = {
	{ 0x15, 0x00 },
	{ 0x11, 0x00 },
	{ 0x21, 0x23 },
	{ 0x50, 0x9A },
	{ 0x51, 0x80 },
	{ 0x20, 0x00 },
	{ 0x2A, 0x32 },
	{ 0x2C, 0x00 },
	{ 0x2D, 0x00 },
	{ 0x2E, 0x00 },
};


static struct msm_camera_i2c_conf_array rj6cba100_confs[] = {
	/* VGA */
	{&rj6cba100_snap_settings[0],
	ARRAY_SIZE(rj6cba100_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	/* VGA */
	{&rj6cba100_prev_settings[0],
	ARRAY_SIZE(rj6cba100_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t rj6cba100_dimensions[] = {
	{
		.x_output = 0x280, /* 640 */
		.y_output = 0x1E0, /* 480 */
		.line_length_pclk = 0x280, /* 640 */
		.frame_length_lines = 0x1E0, /* 480 */
		.op_pixel_clk = 128000000,
		.vt_pixel_clk = 9216000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x280, /* 640 */
		.y_output = 0x1E0, /* 480 */
		.line_length_pclk = 0x280, /* 640 */
		.frame_length_lines = 0x1E0, /* 480 */
		.op_pixel_clk = 128000000,
		.vt_pixel_clk = 9216000,
		.binning_factor = 1,
	},
};

static struct msm_camera_i2c_reg_conf rj6cba100_config_exposure_settings[RJ6CBA100_EXPOSURE_TBL_SIZE][RJ6CBA100_EXPOSURE_SETTING_SIZE] = {
	/* -6 */
	{
		{0x24, 0x48},
		{0x25, 0x38},
	},
	/* -5 */
	{
		{0x24, 0x4F},
		{0x25, 0x3F},
	},
	/* -4 */
	{
		{0x24, 0x58},
		{0x25, 0x48},
	},
	/* -3 */
	{
		{0x24, 0x5F},
		{0x25, 0x4F},
	},
	/* -2 */
	{
		{0x24, 0x68},
		{0x25, 0x58},
	},
	/* -1 */
	{
		{0x24, 0x6F},
		{0x25, 0x5F},
	},
	/* 0 */
	{
		{0x24, 0x78},
		{0x25, 0x68},
	},
	/* 1 */
	{
		{0x24, 0x7F},
		{0x25, 0x6F},
	},
	/* 2 */
	{
		{0x24, 0x88},
		{0x25, 0x78},
	},
	/* 3 */
	{
		{0x24, 0x8F},
		{0x25, 0x7F},
	},
	/* 4 */
	{
		{0x24, 0x98},
		{0x25, 0x88},
	},
	/* 5 */
	{
		{0x24, 0x9F},
		{0x25, 0x8F},
	},
	/* 6 */
	{
		{0x24, 0xA8},
		{0x25, 0x98},
	},
};

static struct msm_camera_i2c_reg_conf rj6cba100_config_white_balance_settings[RJ6CBA100_WB_MAX][RJ6CBA100_WB_SETTING_SIZE] = {
	/* RJ6CBA100_WB_AUTO */
	{
		{0x13, 0xF7},
		{0x8C, 0x5E},
		{0x8D, 0x11},
		{0x8E, 0x12},
		{0x8F, 0x19},
		{0x90, 0x50},
		{0x91, 0x20},
		{0x92, 0x96},
		{0x93, 0x80},
		{0x94, 0x13},
		{0x95, 0x1B},
		{0x96, 0xFF},
		{0x97, 0x00},
		{0x98, 0x3D},
		{0x99, 0x36},
		{0x9A, 0x51},
		{0x9B, 0x43},
		{0x9C, 0xF0},
		{0x9D, 0xF0},
		{0x9E, 0xF0},
		{0x9F, 0xFF},
		{0xA0, 0x68},
		{0xA1, 0x62},
		{0xA2, 0x0E},
	},
	/* RJ6CBA100_WB_DAYLIGHT */
	{
		{0x13, 0xF5},
		{0x8C, 0x5C},
		{0x8D, 0x11},
		{0x8E, 0x12},
		{0x8F, 0x19},
		{0x90, 0x50},
		{0x91, 0x20},
		{0x92, 0x85},
		{0x93, 0x87},
		{0x94, 0x0E},
		{0x95, 0x06},
		{0x96, 0xff},
		{0x97, 0x00},
		{0x98, 0x51},
		{0x99, 0x2B},
		{0x9A, 0x3F},
		{0x9B, 0x3F},
		{0x9C, 0xf0},
		{0x9D, 0xf0},
		{0x9E, 0xf0},
		{0x9F, 0xff},
		{0xA0, 0xA7},
		{0xA1, 0x4F},
		{0xA2, 0x0D},
		{0x01, 0x47},
		{0x02, 0x5E},
		{0x03, 0x40},
	},
	/* RJ6CBA100_WB_FLUORESCENT */
	{
		{0x13, 0xF5},
		{0x8C, 0x5C},
		{0x8D, 0x11},
		{0x8E, 0x12},
		{0x8F, 0x19},
		{0x90, 0x50},
		{0x91, 0x20},
		{0x92, 0x85},
		{0x93, 0x87},
		{0x94, 0x0E},
		{0x95, 0x06},
		{0x96, 0xff},
		{0x97, 0x00},
		{0x98, 0x51},
		{0x99, 0x2B},
		{0x9A, 0x3F},
		{0x9B, 0x3F},
		{0x9C, 0xf0},
		{0x9D, 0xf0},
		{0x9E, 0xf0},
		{0x9F, 0xff},
		{0xA0, 0xA7},
		{0xA1, 0x4F},
		{0xA2, 0x0D},
		{0x01, 0x5C},
		{0x02, 0x54},
		{0x03, 0x46},
	},
	/* RJ6CBA100_WB_CLOUDY_DAYLIGHT */
	{
		{0x13, 0xF5},
		{0x8C, 0x5C},
		{0x8D, 0x11},
		{0x8E, 0x12},
		{0x8F, 0x19},
		{0x90, 0x50},
		{0x91, 0x20},
		{0x92, 0x85},
		{0x93, 0x87},
		{0x94, 0x0E},
		{0x95, 0x06},
		{0x96, 0xff},
		{0x97, 0x00},
		{0x98, 0x51},
		{0x99, 0x2B},
		{0x9A, 0x3F},
		{0x9B, 0x3F},
		{0x9C, 0xf0},
		{0x9D, 0xf0},
		{0x9E, 0xf0},
		{0x9F, 0xff},
		{0xA0, 0xA7},
		{0xA1, 0x4F},
		{0xA2, 0x0D},
		{0x01, 0x46},
		{0x02, 0x56},
		{0x03, 0x40},
	},
	/* RJ6CBA100_WB_INCANDESCENT */
	{
		{0x13, 0xF5},
		{0x8C, 0x5C},
		{0x8D, 0x11},
		{0x8E, 0x12},
		{0x8F, 0x19},
		{0x90, 0x50},
		{0x91, 0x20},
		{0x92, 0x85},
		{0x93, 0x87},
		{0x94, 0x0E},
		{0x95, 0x06},
		{0x96, 0xff},
		{0x97, 0x00},
		{0x98, 0x51},
		{0x99, 0x2B},
		{0x9A, 0x3F},
		{0x9B, 0x3F},
		{0x9C, 0xf0},
		{0x9D, 0xf0},
		{0x9E, 0xf0},
		{0x9F, 0xff},
		{0xA0, 0xA7},
		{0xA1, 0x4F},
		{0xA2, 0x0D},
		{0x01, 0x62},
		{0x02, 0x38},
		{0x03, 0x40},
	},
};

static struct msm_camera_i2c_reg_conf rj6cba100_config_effect_settings[RJ6CBA100_EFFECT_MAX][RJ6CBA100_EFFECT_SETTING_SIZE] = {
	/* RJ6CBA100_EFFECT_OFF */
	{
		{0x81, 0x3F},
		{0x28, 0x00},
		{0xD2, 0x07},
		{0xD3, 0x08},
		{0xD4, 0x24},
		{0xD6, 0x80},
		{0xd8, 0x50},
		{0xd9, 0x50},
		{0xDA, 0x80},
		{0xDB, 0x80},
		{0x81, 0x3f},
	},
	/* RJ6CBA100_EFFECT_MONO */
	{
		{0x81, 0x3F},
		{0x28, 0x00},
		{0xD2, 0x1F},
		{0xD3, 0x00},
		{0xD4, 0x20},
		{0xD6, 0x00},
		{0xd8, 0x40},
		{0xd9, 0x40},
		{0xDA, 0x80},
		{0xDB, 0x80},
		{0x81, 0x3f},
	},
	/* RJ6CBA100_EFFECT_NEGATIVE */
	{
		{0x81, 0x3F},
		{0x28, 0x80},
		{0xD2, 0x47},
		{0xD3, 0x00},
		{0xD4, 0x20},
		{0xD6, 0x80},
		{0xd8, 0x40},
		{0xd9, 0x40},
		{0xDA, 0x80},
		{0xDB, 0x80},
		{0x81, 0x3f},
	},
	/* RJ6CBA100_EFFECT_SEPIA */
	{
		{0x81, 0x3F},
		{0x28, 0x00},
		{0xD2, 0x1F},
		{0xD3, 0x00},
		{0xD4, 0x20},
		{0xD6, 0x00},
		{0xd8, 0x40},
		{0xd9, 0x40},
		{0xDA, 0x70},
		{0xDB, 0x90},
		{0x81, 0x3f},
	},
	/* RJ6CBA100_EFFECT_POSTER */
	{
		{0x81, 0x3F},
		{0x28, 0x00},
		{0xD2, 0x07},
		{0xD3, 0x08},
		{0xD4, 0x24},
		{0xD6, 0x80},
		{0xd8, 0x80},
		{0xd9, 0x80},
		{0xDA, 0x80},
		{0xDB, 0x80},
		{0x81, 0x3f},
	},
};

/************************* add special ***********************/
// static bool camera_power_on = false;
static struct msm_cam_clk_info cam_clk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
};

static int32_t msm_sensor_rj6cba100_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl, uint16_t res)
{
	int32_t rc;

	CDBG("%s: E\n", __func__);

	rc = msm_sensor_write_all_conf_array(
		s_ctrl->sensor_i2c_client,
		&rj6cba100_confs[res],
		1);
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_sensor_evt_notify(s_ctrl, MSG_ID_ERROR_I2C);
	}

	CDBG("%s: X rc = %d\n", __func__, rc);
	return rc;
}
int32_t msm_sensor_rj6cba100_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	CDBG("%s: E\n", __func__);

	if (update_type == MSM_SENSOR_REG_INIT) {
		pr_err("Register INIT\n");
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
		msm_sensor_write_init_settings(s_ctrl);
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		pr_err("PERIODIC : %d\n", res);
		msm_sensor_rj6cba100_write_res_settings(s_ctrl, res);
		usleep_range(66000,66000);
		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);
		if(rj6cba100_power_on_flg == 0){
			rj6cba100_wb = RJ6CBA100_WB_NONE;
			msm_sensor_rj6cba100_set_white_balance(s_ctrl, rj6cba100_wb_wait);
        }
	}
	CDBG("%s: X rc = %d\n", __func__, rc);
	return rc;
}

int32_t msm_sensor_rj6cba100_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;

	pr_err("%s: %d\n", __func__, __LINE__);
	s_ctrl->reg_ptr = kzalloc(sizeof(struct regulator *)
			* data->sensor_platform_info->num_vreg, GFP_KERNEL);
	if (!s_ctrl->reg_ptr) {
		pr_err("%s: could not allocate mem for regulators\n",
			__func__);
		return -ENOMEM;
	}

	rc = msm_camera_request_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		goto request_gpio_failed;
	}

	rc = msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->vreg_seq,
			s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: regulator on failed\n", __func__);
		goto config_vreg_failed;
	}

	rc = msm_camera_config_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: config gpio failed\n", __func__);
		goto config_gpio_failed;
	}

	rc = regulator_enable(s_ctrl->reg_ptr[0]);   /* VSCAMD(1.8V) */
	if (rc < 0) {
		pr_err("%s: %s regulator enable failed\n", __func__, s_ctrl->sensordata->sensor_platform_info->cam_vreg[0].reg_name);
		goto enable_vreg_failed;
	}
	usleep(100);
	
	rc = regulator_enable(s_ctrl->reg_ptr[1]);   /* VSCAMA(2.8V) */
	if (rc < 0) {
		pr_err("%s: %s regulator enable failed\n", __func__, s_ctrl->sensordata->sensor_platform_info->cam_vreg[1].reg_name);
		goto enable_vreg_failed;
	}
	usleep_range(5000,5000);

	if (s_ctrl->clk_rate != 0)
		cam_clk_info->clk_rate = s_ctrl->clk_rate;

	rc = msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
	cam_clk_info, s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 1);
	if (rc < 0) {
		pr_err("%s: clk enable failed\n", __func__);
		goto enable_clk_failed;
	}

	rc = msm_camera_config_gpio_table(data, 0);
	if (rc < 0) {
		pr_err("%s: config gpio failed\n", __func__);
		goto config_gpio_failed2;
	}
	usleep_range(5000,5000);

	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(1);

	rj6cba100_wb = RJ6CBA100_WB_NONE;
	rj6cba100_wb_wait = RJ6CBA100_WB_NONE;
	rj6cba100_effect = RJ6CBA100_EFFECT_NONE;
	rj6cba100_power_on_flg = 0;
	return rc;


config_gpio_failed2:
enable_clk_failed:
enable_vreg_failed:
	msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->vreg_seq,
			s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 0);
	msm_camera_config_gpio_table(data, 0);
config_gpio_failed:
	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
config_vreg_failed:
request_gpio_failed:
	msm_camera_request_gpio_table(data, 0);
	kfree(s_ctrl->reg_ptr);
	return rc;
}

int32_t msm_sensor_rj6cba100_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	pr_err("%s\n", __func__);

//	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
//	msleep(20);
	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(0);

	msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
	cam_clk_info, s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 0);
	usleep_range(5000,5000);
	regulator_disable(s_ctrl->reg_ptr[1]);  /* VSCAMA(2.8V) */
	usleep_range(5000,5000);
	regulator_disable(s_ctrl->reg_ptr[0]);   /* VSCAMD(1.8V) */

	msm_camera_request_gpio_table(data, 0);
	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
	kfree(s_ctrl->reg_ptr);

	msleep(100);

	return 0;
}



static void msm_sensor_rj6cba100_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	CDBG("%s: E\n", __func__);

    if(rj6cba100_power_on_flg == 0)
		msm_sensor_start_stream(s_ctrl);

	rj6cba100_power_on_flg = 1;

	CDBG("%s: X\n", __func__);
}

static void msm_sensor_rj6cba100_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	CDBG("%s: E\n", __func__);
//	msm_sensor_stop_stream(s_ctrl);
	CDBG("%s: X\n", __func__);
}

static int32_t msm_sensor_rj6cba100_get_exp_time(struct msm_sensor_ctrl_t *s_ctrl, uint32_t *exposure_time)
{
	CDBG("%s: E\n", __func__);

	*exposure_time = rj6cba100_g_exposure_time;

	CDBG("%s: X exposure_time = %d\n", __func__, *exposure_time);
	return 0;
}

static int32_t msm_sensor_rj6cba100_set_note_takepic(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int32_t i;
	uint16_t addr[6]={0x10, 0x0F, 0x2D, 0x2E, 0x2B, 0x2A};
	uint16_t data[6];
	uint32_t exp_aec;
	uint32_t exp_advf;
	uint32_t exp_line;

	CDBG("%s: E\n", __func__);

	for (i=0; i<6; i++) {
		rc = msm_camera_i2c_read(
			s_ctrl->sensor_i2c_client,
			addr[i],
			&data[i],
			s_ctrl->msm_sensor_reg->default_data_type);
		pr_err("%s: data:%02x \n", __func__, data[i]);
		if (rc < 0) {
			pr_err("%s: i2c read err \n", __func__);
			msm_sensor_evt_notify(s_ctrl, MSG_ID_ERROR_I2C);
			goto failure;
		}
	}

	exp_aec  = data[0] + (data[1]<<8);
	exp_advf = data[2] + (data[3]<<8);
	exp_line = (data[4] + ((data[5]&0x70)<<4)) * 80;
	rj6cba100_g_exposure_time = (exp_line * (exp_aec+exp_advf)) / 1000000;

	pr_err("%s: %04x %04x %04x %04x\n", __func__, exp_aec, exp_advf, exp_line, rj6cba100_g_exposure_time);
	CDBG("%s: exposure_time = %d\n", __func__, rj6cba100_g_exposure_time);

failure:

	CDBG("%s: X rc = %d\n", __func__, rc);
	return rc;
}

int32_t msm_sensor_rj6cba100_write_exp(struct msm_sensor_ctrl_t *s_ctrl, int8_t exp_value)
{
	int32_t exp_setting;
	int32_t rc = 0;

	exp_setting = exp_value / RJ6CBA100_EXPOSURE_VALUE_STEP + RJ6CBA100_EXPOSURE_VALUE_MAX;
	if (exp_setting < 0) {
		exp_setting = 0;
	}
	else if (exp_setting > (RJ6CBA100_EXPOSURE_VALUE_MAX * 2)) {
		exp_setting = RJ6CBA100_EXPOSURE_VALUE_MAX * 2;
	}

	CDBG("%s: exp_value = %d, exp_setting = %d\n", __func__, exp_value, exp_setting);

	if (pre_exp_setting == exp_setting) {
		CDBG("%s: exp_setting no change \n", __func__);
		return 0;
	}
	pre_exp_setting = exp_setting;

	rc = msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		&rj6cba100_config_exposure_settings[exp_setting][0],
		ARRAY_SIZE(rj6cba100_config_exposure_settings[exp_setting]),
		s_ctrl->msm_sensor_reg->default_data_type);
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_sensor_evt_notify(s_ctrl, MSG_ID_ERROR_I2C);
	}

	CDBG("%s: rc = %d\n", __func__, rc);
	return rc;
}

int32_t msm_sensor_rj6cba100_set_white_balance(struct msm_sensor_ctrl_t *s_ctrl, int8_t wb)
{
	int32_t index = 0;
	int32_t rc = 0;
	uint16_t size = 0;

	CDBG("%s: wb = %d\n", __func__, wb);

	rj6cba100_wb_wait = wb;

	if (rj6cba100_wb == wb) {
		pr_err("%s: wb no change \n", __func__);
		return 0;
	}

	rj6cba100_wb = wb;

	switch (wb) {
	case CAMERA_WB_DAYLIGHT:
		index = RJ6CBA100_WB_DAYLIGHT;
		size = ARRAY_SIZE(rj6cba100_config_white_balance_settings[index]);
		break;

	case CAMERA_WB_FLUORESCENT:
		index = RJ6CBA100_WB_FLUORESCENT;
		size = ARRAY_SIZE(rj6cba100_config_white_balance_settings[index]);
		break;

	case CAMERA_WB_CLOUDY_DAYLIGHT:
		index = RJ6CBA100_WB_CLOUDY_DAYLIGHT;
		size = ARRAY_SIZE(rj6cba100_config_white_balance_settings[index]);
		break;

	case CAMERA_WB_INCANDESCENT:
		index = RJ6CBA100_WB_INCANDESCENT;
		size = ARRAY_SIZE(rj6cba100_config_white_balance_settings[index]);
		break;

	case CAMERA_WB_AUTO:
	default:
		index = RJ6CBA100_WB_AUTO;
		size = ARRAY_SIZE(rj6cba100_config_white_balance_settings[index])-3;
		break;
	}

	rc = msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		&rj6cba100_config_white_balance_settings[index][0],
		size,
		s_ctrl->msm_sensor_reg->default_data_type);
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_sensor_evt_notify(s_ctrl, MSG_ID_ERROR_I2C);
	}

	CDBG("%s: rc = %d\n", __func__, rc);
	return rc;
}

int32_t msm_sensor_rj6cba100_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int8_t effect)
{
	int32_t index = 0;
	int32_t rc = 0;

	CDBG("%s: effect = %d\n", __func__, effect);

    if ((rj6cba100_effect == effect) ||
        (effect != MSM_V4L2_EFFECT_OFF &&
         effect != MSM_V4L2_EFFECT_MONO &&
         effect != MSM_V4L2_EFFECT_NEGATIVE &&
         effect != MSM_V4L2_EFFECT_SEPIA &&
         effect != MSM_V4L2_EFFECT_POSTERAIZE)){
		pr_err("%s: effect no change \n", __func__);
		return 0;
	}

	rj6cba100_effect = effect;

	switch (effect) {
	case MSM_V4L2_EFFECT_MONO:
		index = RJ6CBA100_EFFECT_MONO;
		break;

	case MSM_V4L2_EFFECT_NEGATIVE:
		index = RJ6CBA100_EFFECT_NEGATIVE;
		break;

	case MSM_V4L2_EFFECT_SEPIA:
		index = RJ6CBA100_EFFECT_SEPIA;
		break;

	case MSM_V4L2_EFFECT_POSTERAIZE:
		index = RJ6CBA100_EFFECT_POSTER;
		break;

	case MSM_V4L2_EFFECT_OFF:
	default:
		index = RJ6CBA100_EFFECT_OFF;
		break;
	}

	rc = msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		&rj6cba100_config_effect_settings[index][0],
		ARRAY_SIZE(rj6cba100_config_effect_settings[index]),
		s_ctrl->msm_sensor_reg->default_data_type);
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_sensor_evt_notify(s_ctrl, MSG_ID_ERROR_I2C);
	}

	CDBG("%s: rc = %d\n", __func__, rc);
	return rc;
}

static int32_t msm_sensor_rj6cba100_get_low_light_info(struct msm_sensor_ctrl_t *s_ctrl, int32_t *lowlight_info)
{
	int32_t rc = 0;
	int32_t i;
	uint16_t addr[4]={0x00, 0x15, 0x2D, 0x2E};
	uint16_t data[4];
	uint32_t agc;
	uint32_t advf;

	CDBG("%s: E\n", __func__);
	
	*lowlight_info = 0;
	for (i=0; i<4; i++) {
		rc = msm_camera_i2c_read(
			s_ctrl->sensor_i2c_client,
			addr[i],
			&data[i],
			s_ctrl->msm_sensor_reg->default_data_type);
		CDBG("%s: data:%02x \n", __func__, data[i]);
		if (rc < 0) {
			pr_err("%s: i2c read err %d\n", __func__, rc);
			msm_sensor_evt_notify(s_ctrl, MSG_ID_ERROR_I2C);
			return rc;
		}
	}
	agc  = data[0] + ((data[1]&0x03)<<8);
	advf = data[2] + (data[3]<<8);

	CDBG("%s: agc = %d, advf = %d\n", __func__, agc, advf);

	if(agc >= RJ6CBA100_AGC_THRESHOLD_5LX && advf >= RJ6CBA100_ADVF_THRESHOLD_5LX) {
		*lowlight_info = 2;
	}
	else if(agc >= RJ6CBA100_AGC_THRESHOLD_50LX && advf >= RJ6CBA100_ADVF_THRESHOLD_50LX) {
		*lowlight_info = 1;
	}

	CDBG("%s: X rc = %d\n", __func__, rc);
	return rc;
}

static struct msm_sensor_id_info_t rj6cba100_id_info = {
	.sensor_id_reg_addr = 0x0A,
	.sensor_id = 0x7692,
};

static const struct i2c_device_id rj6cba100_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&rj6cba100_s_ctrl},
	{ }
};

static struct i2c_driver rj6cba100_i2c_driver = {
	.id_table = rj6cba100_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client rj6cba100_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&rj6cba100_i2c_driver);
}

static struct v4l2_subdev_core_ops rj6cba100_subdev_core_ops = {
	.s_ctrl = msm_sensor_v4l2_s_ctrl,
	.queryctrl = msm_sensor_v4l2_query_ctrl,
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops rj6cba100_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops rj6cba100_subdev_ops = {
	.core = &rj6cba100_subdev_core_ops,
	.video  = &rj6cba100_subdev_video_ops,
};

static struct msm_sensor_fn_t rj6cba100_func_tbl = {
	.sensor_start_stream = msm_sensor_rj6cba100_start_stream,
	.sensor_stop_stream = msm_sensor_rj6cba100_stop_stream,
//	.sensor_setting = msm_sensor_setting,
	.sensor_setting = msm_sensor_rj6cba100_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_rj6cba100_power_up, /* add special */
	.sensor_power_down = msm_sensor_rj6cba100_power_down, /* add special */
//	.sensor_match_id =  msm_sensor_rj6cba100_match_id,
//	.sensor_write_exp_gain = msm_sensor_rj6cba100_write_exp,
	.sensor_set_exposure_compensation = msm_sensor_rj6cba100_write_exp,
	.sensor_set_white_balance = msm_sensor_rj6cba100_set_white_balance,
	.sensor_get_exposure_time = msm_sensor_rj6cba100_get_exp_time,
	.sensor_set_effect = msm_sensor_rj6cba100_set_effect,
	.sensor_set_note_takepic = msm_sensor_rj6cba100_set_note_takepic,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
//	.sensor_get_frame_skip_flg = msm_sensor_rj6cba100_get_frame_skip_flg,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_get_low_light_info = msm_sensor_rj6cba100_get_low_light_info,
};

static struct msm_sensor_reg_t rj6cba100_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA, /* add special */
//	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA, /* kill org */
	.start_stream_conf = rj6cba100_start_stream_settings,
	.start_stream_conf_size = ARRAY_SIZE(rj6cba100_start_stream_settings),
	.stop_stream_conf = rj6cba100_stop_stream_settings,
	.stop_stream_conf_size = ARRAY_SIZE(rj6cba100_stop_stream_settings),
	.init_settings = &rj6cba100_init_conf[0],
	.init_size = ARRAY_SIZE(rj6cba100_init_conf),
//	.mode_settings = &rj6cba100_confs[0],
	.output_settings = &rj6cba100_dimensions[0],
//	.num_conf = ARRAY_SIZE(rj6cba100_confs),
	.num_conf = ARRAY_SIZE(rj6cba100_dimensions),
};

static struct msm_sensor_ctrl_t rj6cba100_s_ctrl = {
	.msm_sensor_reg = &rj6cba100_regs,
	.sensor_i2c_client = &rj6cba100_sensor_i2c_client,
	.sensor_i2c_addr = 0x78,
//	.sensor_output_reg_addr = &rj6cba100_reg_addr,
	.sensor_id_info = &rj6cba100_id_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex = &rj6cba100_mut,
	.sensor_i2c_driver = &rj6cba100_i2c_driver,
	.sensor_v4l2_subdev_info = rj6cba100_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(rj6cba100_subdev_info),
	.sensor_v4l2_subdev_ops = &rj6cba100_subdev_ops,
	.func_tbl = &rj6cba100_func_tbl,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Samsung 1.2MP YUV sensor driver");
MODULE_LICENSE("GPL v2");

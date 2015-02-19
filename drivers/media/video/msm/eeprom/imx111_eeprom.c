/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include "msm_camera_eeprom.h"
#include "msm_camera_i2c.h"
#include <media/msm_isp.h>

DEFINE_MUTEX(imx111_eeprom_mutex);
static struct msm_eeprom_ctrl_t imx111_eeprom_t;
#define IMX111_EEPROM_BANK_SEL_REG 0x34C9

static const struct i2c_device_id imx111_eeprom_i2c_id[] = {
	{"imx111_eeprom", (kernel_ulong_t)&imx111_eeprom_t},
	{ }
};

static struct i2c_driver imx111_eeprom_i2c_driver = {
	.id_table = imx111_eeprom_i2c_id,
	.probe  = msm_eeprom_i2c_probe,
	.remove = __exit_p(imx111_eeprom_i2c_remove),
	.driver = {
		.name = "imx111_eeprom",
	},
};

static int __init imx111_eeprom_i2c_add_driver(void)
{
	int rc = 0;
pr_err("%s called\n", __func__);
	rc = i2c_add_driver(imx111_eeprom_t.i2c_driver);
	return rc;
}

static struct v4l2_subdev_core_ops imx111_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops imx111_eeprom_subdev_ops = {
	.core = &imx111_eeprom_subdev_core_ops,
};

static uint8_t imx111_wbcalib_data[6];
static uint8_t imx111_afcalib_data[8];
static struct msm_calib_wb imx111_wb_data;
struct msm_calib_af imx111_af_data;
// uint16_t imx111_dac_1m = 0;
uint16_t imx111_dac_40cm = 0;

static struct msm_camera_eeprom_info_t imx111_calib_supp_info = {
	{TRUE, 8, 1, 1},
	{TRUE, 6, 0, 1024},
	{FALSE, 0, 0, 1},
	{FALSE, 0, 0, 1},
	{FALSE, 0, 0, 1},
};

static struct msm_camera_eeprom_read_t imx111_eeprom_read_tbl[] = {
	{0x3508, &imx111_wbcalib_data[0], 6, 0},
	{0x350E, &imx111_afcalib_data[0], 2, 0},
	{0x3510, &imx111_afcalib_data[2], 6, 0},
};


static struct msm_camera_eeprom_data_t imx111_eeprom_data_tbl[] = {
	{&imx111_wb_data, sizeof(struct msm_calib_wb)},
	{&imx111_af_data, sizeof(struct msm_calib_af)},
};

void imx111_set_dev_addr(struct msm_eeprom_ctrl_t *eclient,
	uint32_t *reg_addr) {
	uint16_t eprom_addr = *reg_addr;
	int32_t rc = 0;

pr_err("%s: E\n", __func__);

	if ((eprom_addr >= 0x3500) && (eprom_addr < 0x3508)) {
		rc = msm_camera_i2c_write(&eclient->i2c_client,
			IMX111_EEPROM_BANK_SEL_REG,
			0x00, MSM_CAMERA_I2C_BYTE_DATA);
	}
	if ((eprom_addr >= 0x3508) && (eprom_addr < 0x3510)) {
		rc = msm_camera_i2c_write(&eclient->i2c_client,
			IMX111_EEPROM_BANK_SEL_REG,
			0x01, MSM_CAMERA_I2C_BYTE_DATA);
	}
	if ((eprom_addr >= 0x3510) && (eprom_addr < 0x3518)) {
		rc = msm_camera_i2c_write(&eclient->i2c_client,
			IMX111_EEPROM_BANK_SEL_REG,
			0x02, MSM_CAMERA_I2C_BYTE_DATA);
	}
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_eeprom_evt_notify(eclient, MSG_ID_ERROR_I2C);
	}
}

static void imx111_format_wbdata(void)
{
	imx111_wb_data.r_over_g = (uint16_t)(imx111_wbcalib_data[0] << 8) |
		(imx111_wbcalib_data[1]);
	imx111_wb_data.b_over_g = (uint16_t)(imx111_wbcalib_data[2] << 8) |
		(imx111_wbcalib_data[3]);
	imx111_wb_data.gr_over_gb = (uint16_t)(imx111_wbcalib_data[4] << 8) |
		(imx111_wbcalib_data[5]);
}

static void imx111_format_afdata(void)
{
	imx111_af_data.inf_dac = (uint16_t)(imx111_afcalib_data[0] << 8) |
		imx111_afcalib_data[1];
	imx111_af_data.macro_dac = (uint16_t)(imx111_afcalib_data[4] << 8) |
		imx111_afcalib_data[5];
	imx111_af_data.start_dac = (uint16_t)(imx111_afcalib_data[6] << 8) |
		imx111_afcalib_data[7];
//	imx111_dac_1m = (uint16_t)(imx111_afcalib_data[2] << 8) | imx111_afcalib_data[3];
	imx111_dac_40cm = imx111_af_data.inf_dac + (imx111_af_data.macro_dac - imx111_af_data.inf_dac) * 189 / 865;
	CDBG("%s: imx111_dac_40cm %04x\n", __func__, imx111_dac_40cm);
}

void imx111_format_calibrationdata(void)
{
pr_err("%s: E\n", __func__);
	imx111_format_wbdata();
	imx111_format_afdata();
}
static struct msm_eeprom_ctrl_t imx111_eeprom_t = {
	.i2c_driver = &imx111_eeprom_i2c_driver,
//	.i2c_addr = 0x34,
	.i2c_addr = 0x6C,
	.eeprom_v4l2_subdev_ops = &imx111_eeprom_subdev_ops,

	.i2c_client = {
//		.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
		.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	},

	.eeprom_mutex = &imx111_eeprom_mutex,

	.func_tbl = {
		.eeprom_init = NULL,
		.eeprom_release = NULL,
		.eeprom_get_info = msm_camera_eeprom_get_info,
		.eeprom_get_data = msm_camera_eeprom_get_data,
		.eeprom_set_dev_addr = imx111_set_dev_addr,
		.eeprom_format_data = imx111_format_calibrationdata,
	},
	.info = &imx111_calib_supp_info,
	.info_size = sizeof(struct msm_camera_eeprom_info_t),
	.read_tbl = imx111_eeprom_read_tbl,
	.read_tbl_size = ARRAY_SIZE(imx111_eeprom_read_tbl),
	.data_tbl = imx111_eeprom_data_tbl,
	.data_tbl_size = ARRAY_SIZE(imx111_eeprom_data_tbl),
};

subsys_initcall(imx111_eeprom_i2c_add_driver);
MODULE_DESCRIPTION("imx111 EEPROM");
MODULE_LICENSE("GPL v2");

/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * arch/arm/mach-msm/board-8960-touch.h
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
#ifndef __ARCH_ARM_MACH_MSM_BOARD_MSM8960_TOUCH_H
#define __ARCH_ARM_MACH_MSM_BOARD_MSM8960_TOUCH_H

#ifdef CONFIG_TOUCHSCREEN_PIXART_KC
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c/pixart.h>


#define PIXART_TS_RESET_GPIO	6
#define PIXART_TS_GPIO_IRQ		7

#define PIXART_PANEL_WIDTH		1280
#define PIXART_PANEL_HEIGHT		720

#define PIXART_BOARD_INFO		I2C_BOARD_INFO(PIXART_I2C_NAME, 0x66 >> 1)

static inline int pixart_init_hw(void)
{
	int ret;

	ret = gpio_request(PIXART_TS_GPIO_IRQ, "pixart_ts_irq_gpio");
	if (ret) {
		printk("%s: unable to request pixart_ts_irq gpio [%d]\n"
						, __func__, PIXART_TS_GPIO_IRQ);
		goto err_irq_gpio_req;
	}
	ret = gpio_direction_input(PIXART_TS_GPIO_IRQ);
	if (ret) {
		printk("%s: unable to set_direction for pixart_ts_irq gpio [%d]\n"
						, __func__, PIXART_TS_GPIO_IRQ);
		goto err_irq_gpio_set;
	}
	ret = gpio_request(PIXART_TS_RESET_GPIO, "pixart_reset_gpio");
	if (ret) {
		printk("%s: unable to request pixart_reset gpio [%d]\n"
						, __func__, PIXART_TS_RESET_GPIO);
		goto err_reset_gpio_req;
	}
	ret = gpio_direction_output(PIXART_TS_RESET_GPIO, 0);
	if (ret) {
		printk("%s: unable to set_direction for pixart_reset gpio [%d]\n"
						, __func__, PIXART_TS_RESET_GPIO);
		goto err_reset_gpio_set;
	}

	return 0;

err_reset_gpio_set:
	gpio_free(PIXART_TS_RESET_GPIO);
err_reset_gpio_req:
err_irq_gpio_set:
	gpio_free(PIXART_TS_GPIO_IRQ);
err_irq_gpio_req:
	return ret;
}

static inline int pixart_reset_hw(void)
{
	gpio_set_value(PIXART_TS_RESET_GPIO, 0);
	msleep(10);
	gpio_set_value(PIXART_TS_RESET_GPIO, 1);
	msleep(10);

	return 0;
}

static inline int pixart_shutdown(void)
{
	gpio_set_value(PIXART_TS_RESET_GPIO, 0);
	msleep(1);

	return 0;
}

static const u8 pixart_config_data[] = {
 PIXART_REG_INTMASK_GEN2,		1,	0x17,
 PIXART_REG_STATUS_AUTO_CLEAR_GEN2,	1,	0x3F,
 PIXART_REG_INT_DEASSERT_GEN2,		1,	0xCF,
#ifndef PANEL_47INCH_CONFIG
 PIXART_NOISE_THRESH,				1,	0x03,
#endif
 PIXART_REG_CHEEK_PERCENTAGE_GEN2,	1,	0x19,
 PIXART_REG_CHEEK_TOTAL_CELLS_GEN2,	1,	0x30,
 PIXART_REG_CHEEK_DELAY_GEN2,		1,	0x0A,
 PIXART_REG_CHEEK_PROLONG_GEN2,		1,	0x05,
 PIXART_REG_FORCE_RUN_MODE_GEN2,	1,	0xA0,
 PIXART_REG_IDLE_DS_LO_GEN2,		1,	0x90,
 PIXART_REG_IDLE_DS_HI_GEN2,		1,	0x01,
 PIXART_REG_IDLE_FRAME_RATE_GEN2,	1,	0x50,
 PIXART_REG_REST1_DS_LO_GEN2,		1,	0x58,
 PIXART_REG_REST1_DS_HI_GEN2,		1,	0x02,
 PIXART_REG_REST1_FRAME_RATE_GEN2,	1,	0x46,
 PIXART_REG_REST2_DS_LO_GEN2,		1,	0xB0,
 PIXART_REG_REST2_DS_HI_GEN2,		1,	0x04,
 PIXART_REG_REST2_FRAME_RATE_GEN2,	1,	0x32,
 PIXART_REG_ONTOUCH_REPORT_RATE,	1,	0x78,
#ifdef PANEL_47INCH_CONFIG
 PIXART_REG_ROW_GEN2,			1,	0x0C,
 PIXART_REG_COL_GEN2,			1,	0x14,
#else
 PIXART_REG_ROW_GEN2,			1,	0x0A,
 PIXART_REG_COL_GEN2,			1,	0x12,
#endif
 PIXART_REG_TOUCH_ID_GEN2,		1,	0x0B,
 PIXART_REG_HEIGHT_LO_GEN2,		1,	0xD0,
 PIXART_REG_HEGHT_HI_GEN2,		1,	0x02,
 PIXART_REG_WIDTH_LO_GEN2,		1,	0x00,
 PIXART_REG_WIDTH_HI_GEN2,		1,	0x05,
 PIXART_REG_REPORT_POINTS_GEN2,		1,	0x0A,
 PIXART_REG_XOFFSET_LO_GEN2,		1,	0x00,
 PIXART_REG_XOFFSET_HI_GEN2,		1,	0x00,
 PIXART_REG_YOFFSET_LO_GEN2,		1,	0x00,
 PIXART_REG_YOFFSET_HI_GEN2,		1,	0x00,
 PIXART_REG_WIN_ORG_X_GEN2,		1,	0x00,
 PIXART_REG_WN_ORG_Y_GEN2,		1,	0x00,
#ifdef PANEL_47INCH_CONFIG
 PIXART_REG_WIN_MAX_X_GEN2,		1,	0x13,
 PIXART_REG_WIN_MAX_Y_GEN2,		1,	0x0B,
#else
 PIXART_REG_WIN_MAX_X_GEN2,		1,	0x11,
 PIXART_REG_WIN_MAX_Y_GEN2,		1,	0x09,
#endif
 PIXART_REG_NAV_CONFIG_GEN2,		1,	0x95,
 PIXART_REG_NAV_CONFIG_2_GEN2,		1,	0x02,
#ifdef PANEL_47INCH_CONFIG
 PIXART_REG_THRESH0_PCT_GEN2,		1,	0x16,
 PIXART_REG_THRESH1_PCT_GEN2,		1,	0x17,
 PIXART_REG_THRESH2_PCT_GEN2,		1,	0x11,
 PIXART_REG_THRESH3_PCT_GEN2,		1,	0x1A,
#else
 PIXART_REG_THRESH0_PCT_GEN2,		1,	0x15,
 PIXART_REG_THRESH1_PCT_GEN2,		1,	0x16,
 PIXART_REG_THRESH2_PCT_GEN2,		1,	0x10,
 PIXART_REG_THRESH3_PCT_GEN2,		1,	0x17,
#endif
 PIXART_REG_THRESH4_PCT_GEN2,		1,	0x1E,
 PIXART_REG_THRESH5_PCT_GEN2,		1,	0x21,
 PIXART_REG_TOUCH_REF_LO_GEN2,		1,	0xB8,
 PIXART_REG_TOUCH_REF_HI_GEN2,		1,	0x0B,
 PIXART_REG_HOVER_PROLONG_DELAY_GEN2,	1,	0x04,
};

static const u8 na_register[] = {
 PIXART_NA_DRIVE_FREQ_INIT, 0x4D,
 PIXART_NOISE_THRESH, 0x0F,
 PIXART_MAX_CORR_DIST_PER_REPORT_LSB, 0x64,
 PIXART_MAX_CORR_DIST_PER_REPORT_MSB, 0x00,
 PIXART_MAX_CORR_DIST_PER_REPORT_LSB_CHARG, 0x64,
 PIXART_MAX_CORR_DIST_PER_REPORT_MSB_CHARG, 0x00,
 PIXART_TOUCH_DELAY_T3, 0x03,
 PIXART_TOUCH_DELAY_T3_CHARG, 0x03,
 PIXART_TOUCH_DELAY_T4, 0x02,
 PIXART_TOUCH_DELAY_T4_CHARG, 0x02,
 PIXART_TOUCH_DELAY_T5, 0x01,
 PIXART_TOUCH_DELAY_T5_CHARG, 0x01,
 PIXART_PROLONG_DELAY, 0x02,
 PIXART_PROLONG_DELAY_CHARG, 0x02,
};

static struct pixart_platform_data pixart_platform_data = {
	.config			= pixart_config_data,
	.config_length	= ARRAY_SIZE(pixart_config_data),
	.na_config		= na_register,
	.na_config_length	= ARRAY_SIZE(na_register),
	.x_size			= PIXART_PANEL_WIDTH,
	.y_size			= PIXART_PANEL_HEIGHT,
	.irq_gpio		= PIXART_TS_GPIO_IRQ,
	.reset_gpio		= PIXART_TS_RESET_GPIO,
	.orient			= PIXART_DIAGONAL,
	.irqflags		= IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
	.init_hw		= pixart_init_hw,
	.reset_hw		= pixart_reset_hw,
	.shutdown		= pixart_shutdown,
};

#endif
#endif

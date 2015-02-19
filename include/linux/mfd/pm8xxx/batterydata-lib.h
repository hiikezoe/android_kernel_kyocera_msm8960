/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PM8XXX_BMS_BATTERYDATA_H
#define __PM8XXX_BMS_BATTERYDATA_H

#include <linux/errno.h>

#define FCC_CC_COLS		5
#define FCC_TEMP_COLS		8

#define PC_CC_ROWS             29
#define PC_CC_COLS             13

#define PC_TEMP_ROWS		29
#define PC_TEMP_COLS		8

#define RBATT_ROWS		22
#define RBATT_COLS		5

#define SOC_ADJUST_ROWS		6
#define SOC_ADJUST_COLS		8

#define MAX_SINGLE_LUT_COLS	20

struct single_row_lut {
	int x[MAX_SINGLE_LUT_COLS];
	int y[MAX_SINGLE_LUT_COLS];
	int cols;
};

/**
 * struct sf_lut -
 * @rows:	number of percent charge entries should be <= PC_CC_ROWS
 * @cols:	number of charge cycle entries should be <= PC_CC_COLS
 * @row_entries:	the charge cycles/temperature at which sf data
 *			is available in the table.
 *		The charge cycles must be in increasing order from 0 to rows.
 * @percent:	the percent charge at which sf data is available in the table
 *		The  percentcharge must be in decreasing order from 0 to cols.
 * @sf:		the scaling factor data
 */
struct sf_lut {
	int rows;
	int cols;
	int row_entries[PC_CC_COLS];
	int percent[PC_CC_ROWS];
	int sf[PC_CC_ROWS][PC_CC_COLS];
};

struct rbatt_lut {
	int rows;
	int cols;
	int temp[RBATT_COLS];
	int vbatt[RBATT_ROWS];
	int rbatt[RBATT_ROWS][RBATT_COLS];
};

struct soc_adjust_lut {
	int rows;
	int cols;
	int temp[SOC_ADJUST_COLS];
	int ibatt[SOC_ADJUST_ROWS];
	int soc_adjust[SOC_ADJUST_ROWS][SOC_ADJUST_COLS];
};

/**
 * struct pc_temp_ocv_lut -
 * @rows:	number of percent charge entries should be <= PC_TEMP_ROWS
 * @cols:	number of temperature entries should be <= PC_TEMP_COLS
 * @temp:	the temperatures at which ocv data is available in the table
 *		The temperatures must be in increasing order from 0 to rows.
 * @percent:	the percent charge at which ocv data is available in the table
 *		The  percentcharge must be in decreasing order from 0 to cols.
 * @ocv:	the open circuit voltage
 */
struct pc_temp_ocv_lut {
	int rows;
	int cols;
	int temp[PC_TEMP_COLS];
	int percent[PC_TEMP_ROWS];
	int ocv[PC_TEMP_ROWS][PC_TEMP_COLS];
};

enum battery_type {
	BATT_UNKNOWN = 0,
	BATT_PALLADIUM,
	BATT_DESAY,
};

/**
 * struct bms_battery_data -
 * @fcc:		full charge capacity (mAmpHour)
 * @fcc_temp_lut:	table to get fcc at a given temp
 * @pc_temp_ocv_lut:	table to get percent charge given batt temp and cycles
 * @pc_sf_lut:		table to get percent charge scaling factor given cycles
 *			and percent charge
 * @rbatt_sf_lut:	table to get battery resistance scaling factor given
 *			temperature and percent charge
 * @default_rbatt_mohm:	the default value of battery resistance to use when
 *			readings from bms are not available.
 * @delta_rbatt_mohm:	the resistance to be added towards lower soc to
 *			compensate for battery capacitance.
 * @rbatt_capacitve_mohm: the resistance to be added to compensate for
 *				battery capacitance
 */

struct bms_battery_data {
	unsigned int		fcc;
	struct single_row_lut	*fcc_temp_lut;
	struct single_row_lut	*fcc_sf_lut;
	struct pc_temp_ocv_lut	*pc_temp_ocv_lut;
	struct sf_lut		*pc_sf_lut;
	struct sf_lut		*rbatt_sf_lut;
	int			default_rbatt_mohm;
	int			delta_rbatt_mohm;
	int			rbatt_capacitive_mohm;
};

struct pm8921_bms_oem_battery_data {
	struct	rbatt_lut	*rbatt_initial_lut;
	struct	soc_adjust_lut	*soc_adjust_lut;
	struct	single_row_lut	*cycle_adjust_lut;
};

#if defined(CONFIG_PM8921_BMS) || \
	defined(CONFIG_PM8921_BMS_MODULE)
extern struct bms_battery_data  palladium_1500_data;
extern struct bms_battery_data  desay_5200_data;
extern struct pm8921_bms_oem_battery_data pm8921_bms_oem_data;

enum pm8921_bms_chg_state {
	CHG_STATE_NONCONNECTED,
	CHG_STATE_IDLE,
	CHG_STATE_TRICKLE,
	CHG_STATE_FAST_COOL,
	CHG_STATE_FAST_NORMAL,
	CHG_STATE_FAST_WARM,
	CHG_STATE_INTE_COOL,
	CHG_STATE_INTE_NORMAL,
	CHG_STATE_INTE_WARM,
	CHG_STATE_CHG_COMP,
	CHG_STATE_CHG_TIMEOUT,
	CHG_STATE_CHG_STAND,
	CHG_STATE_BATT_TEMP_COLD,
	CHG_STATE_BATT_TEMP_HOT,
	CHG_STATE_WAIT_TEMP,
	CHG_STATE_BATT_ID_ERROR,
	CHG_STATE_CHG_ERROR,
	CHG_STATE_INIT,
	CHG_STATE_MAX
};

enum pm8921_bms_chg_mode {
	CHG_MODE_DISCHARGE,
	CHG_MODE_CHARGING,
	CHG_MODE_FULL
};

enum pm8921_bms_chg_condition {
	CHG_CONDITION_4340MV,
	CHG_CONDITION_4240MV,
	CHG_CONDITION_4340MV_INTE,
	CHG_CONDITION_4240MV_INTE,
	CHG_CONDITION_MAX,
	CHG_CONDITION_NULL
};

struct pm8921_bms_correction {
	enum pm8921_bms_chg_state	chg_state;
	enum pm8921_bms_chg_mode	chg_mode;
	enum pm8921_bms_chg_condition	chg_condition_uim_valid;
	enum pm8921_bms_chg_condition	chg_condition_uim_invalid;
};

enum pm8921_bms_cyclecorrect_state {
	CHG_CYCLECORRECT_STATE_CHARGER_NO,
	CHG_CYCLECORRECT_STATE_CHARGER_DETECTED,
	CHG_CYCLECORRECT_STATE_OK1,
	CHG_CYCLECORRECT_STATE_OK2,
	CHG_CYCLECORRECT_STATE_PARAM_OBTAINED1,
	CHG_CYCLECORRECT_STATE_PARAM_OBTAINED2,
	CHG_CYCLECORRECT_STATE_CALCULATED
};

int interpolate_fcc(struct single_row_lut *fcc_temp_lut, int batt_temp);
int interpolate_scalingfactor(struct sf_lut *sf_lut, int row_entry, int pc);
int interpolate_scalingfactor_fcc(struct single_row_lut *fcc_sf_lut,
				int cycles);
int interpolate_pc(struct pc_temp_ocv_lut *pc_temp_ocv,
				int batt_temp_degc, int ocv);
int interpolate_ocv(struct pc_temp_ocv_lut *pc_temp_ocv,
				int batt_temp_degc, int pc);
int linear_interpolate(int y0, int x0, int y1, int x1, int x);
int is_between(int left, int right, int value);
#else
static inline int interpolate_fcc(struct single_row_lut *fcc_temp_lut,
			int batt_temp)
{
	return -EINVAL;
}
static inline int interpolate_scalingfactor(struct sf_lut *sf_lut,
			int row_entry, int pc)
{
	return -EINVAL;
}
static inline int interpolate_scalingfactor_fcc(
			struct single_row_lut *fcc_sf_lut, int cycles)
{
	return -EINVAL;
}
static inline int interpolate_pc(struct pc_temp_ocv_lut *pc_temp_ocv,
			int batt_temp_degc, int ocv)
{
	return -EINVAL;
}
static inline int interpolate_ocv(struct pc_temp_ocv_lut *pc_temp_ocv,
			int batt_temp_degc, int pc)
{
	return -EINVAL;
}
static inline int linear_interpolate(int y0, int x0, int y1, int x1, int x)
{
	return -EINVAL;
}
static inline int is_between(int left, int right, int value)
{
	return -EINVAL;
}
#endif

#endif

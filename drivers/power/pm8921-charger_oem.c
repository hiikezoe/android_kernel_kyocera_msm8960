/* 
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */

#include <linux/debugfs.h>
#include <linux/slab.h>
#include <mach/msm_smsm.h>
#include <pm8921-charger_oem.h>

pm8921_oem_chg_param_charger oem_param_charger = {
	0x1162,
	0xFFFF,
	0xFFFF,
	0xFFFF,

	0x01E0,
	0x01E0,
	0x0200,
	0x0040,
	0xFFFF,
	0xFFFF,

	0x0F,
	0x2D,
	0x34,
	0x2F,
	0x0A,
	0x32,
	0x28,
	0xFF,

	0x10CC,
	0x10CC,
	0x0FDC,
	0x0FDC,
	0xFFFF,
	0xFFFF,

	0x012C,
	0x000A,

	0x10CC,
	0xFFFF,
	0xFFFF,
	0xFFFF,

	0x0F3C,
	0x0DAC,
	0xFFFF,
	0xFFFF,

	0x02D5,
	0x0145,
	0x02D5,
	0x0064,
	0xFFFF,
	0xFFFF,

	0x0050,
	0x0001,
	0xFFFF,
	0xFFFF,

	0x0014,
	0x0028,

	0x03E8,
	0xFFFF,

	0x0320,
	0xFFFF,

	0x50,
	0x4D,
	0x4C,
	0x49,
	0x52,
	0x50,
	0x4E,
	0x4C,
	0x00,
	0xFF,
	0xFF,
	0xFF,

	0xFFFF,
	0xFFFF,

	0xFF,
	0xFF,
	0xFF,
	0xFF
};

pm8921_oem_chg_param_hkadc oem_param_hkadc = {
	0x0010C8E0,
	0x00155CC0,

	0xFFFF,
	0xFFFF,

	0xFF,
	0xFF,
	0xFF,
	0xFF
};

pm8921_oem_chg_param_bms oem_param_bms = {
	0x0D16,
	0x0D7A,
	0x0DDE,
	0x0E42,
	0x0000,
	0x0000,

	0x64,
	0x5A,
	0x55,
	0xFF,

	0xFFFF,
	0xFFFF,

	0xFF,
	0xFF,
	0xFF,
	0xFF
};

pm8921_oem_chg_param_share oem_param_share = {
	0x00,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF
};

pm8921_oem_chg_param_cycles oem_param_cycles = {
	0x00000000,
	0x00000000
};

typedef struct {
	pm8921_oem_chg_param_charger	oem_chg_param_charger;
	pm8921_oem_chg_param_hkadc	oem_chg_param_hkadc;
	pm8921_oem_chg_param_bms	oem_chg_param_bms;
	pm8921_oem_chg_param_share	oem_chg_param_share;
	pm8921_oem_chg_param_cycles	oem_chg_param_cycles;
}pm8921_oem_chg_param;

uint8_t oem_cmp_zero_flag = 0;
static void oem_param_charger_init(pm8921_oem_chg_param *ptr)
{

	if (ptr == NULL){
		pr_err("chg param charger read error.\n");
		return;
	}

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 65534, charger, chg_stop_volt);

	SET_CHG_PARAM(0xFFFF, charger, time_chg);
	SET_CHG_PARAM(0xFFFF, charger, time_cool_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 4, 512, charger, time_chg_pm);
	SET_CHG_PARAM_MINMAX(0xFFFF, 1, 64, charger, time_trkl_pm);

	SET_CHG_PARAM(0xFF, charger, chg_cool_tmp);
	SET_CHG_PARAM(0xFF, charger, chg_warm_tmp);
	SET_CHG_PARAM(0xFF, charger, wait_chg_on);
	SET_CHG_PARAM(0xFF, charger, wait_chg_off);
	SET_CHG_PARAM_MINMAX(0xFF, 0, 254, charger, chg_adp_tmp1);
	SET_CHG_PARAM_MINMAX(0xFF, 0, 254, charger, chg_adp_tmp2);
	SET_CHG_PARAM_MINMAX(0xFF, 0, 254, charger, chg_adp_tmp3);

	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, normal_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, cool_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, warm_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, uim_undete_chg);

	SET_CHG_PARAM(0xFFFF, charger, rechg_delta_volt);
	SET_CHG_PARAM(0xFFFF, charger, initial_delta_volt);

	SET_CHG_PARAM_MINMAX(0xFFFF, 4300, 6500, charger, maint_chg_vin);

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 65534, charger, maint_wait_chg_on_volt);
	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 65534, charger, maint_wait_chg_off_volt);

	SET_CHG_PARAM_MINMAX(0xFFFF, 325, 2000, charger, i_chg_norm);
	SET_CHG_PARAM_MINMAX(0xFFFF, 325, 2000, charger, i_chg_cool);
	SET_CHG_PARAM_MINMAX(0xFFFF, 325, 2000, charger, i_chg_warm);
	SET_CHG_PARAM_MINMAX(0xFFFF, 50, 200, charger, i_chg_finish);

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 400, charger, vdd_max_inc_mv);
	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 100, charger, rconn_mohm);

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 600, charger, maint_wait_chg_on_time);
	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 600, charger, maint_wait_chg_off_time);

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 65534, charger, i_chg_adp_chk);

	SET_CHG_PARAM(0xFFFF, charger, z_chg_adp_chk);

	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_cam_tmp_off);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_cam_tmp_on);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_inte_cam_on);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_inte_cam_off);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_phone_tmp_off);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_phone_tmp_on);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_inte_phone_on);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_inte_phone_off);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_adp_tmp_delta);

	SET_CHG_PARAM_TEST_FLG(0xFF, charger, chg_disable_test);
}

static void oem_param_hkadc_init(pm8921_oem_chg_param *ptr)
{
	if (ptr == NULL){
		pr_err("chg param hkadc read error.\n");
		return;
	}

	SET_CHG_PARAM(0xFFFFFFFF, hkadc, cal_vbat1);
	SET_CHG_PARAM(0xFFFFFFFF, hkadc, cal_vbat2);
}

static void oem_param_bms_init(pm8921_oem_chg_param *ptr)
{
	if (ptr == NULL){
		pr_err("chg param bms read error.\n");
		return;
	}

#ifdef OEM_BMS_USE_NV
	SET_CHG_PARAM(0xFFFF, 2500, 5675, bms, temp_normal_thresh_pow_off);
	SET_CHG_PARAM(0xFFFF, 2500, 5675, bms, temp_normal_thresh_low_batt);
	SET_CHG_PARAM(0xFFFF, 2500, 5675, bms, temp_low_thresh_pow_off);
	SET_CHG_PARAM(0xFFFF, 2500, 5675, bms, temp_low_thresh_low_batt);

	SET_CHG_PARAM(0xFF, 0, 100, bms, soc_rate_i_0);
	SET_CHG_PARAM(0xFF, 0, 100, bms, soc_rate_i_1);
	SET_CHG_PARAM(0xFF, 0, 100, bms, soc_rate_i_2);
#endif

	SET_CHG_PARAM_TEST_FLG(0xFF, bms, bms_dummy_soc_test);
	SET_CHG_PARAM_TEST_MINMAX(0xFF, 1, 100, bms, bms_dummy_soc);
}

static void oem_param_share_init(pm8921_oem_chg_param *ptr)
{
	if (ptr == NULL){
		pr_err("chg param share read error.\n");
		return;
	}

	memcpy(&oem_param_share, &ptr->oem_chg_param_share,
				sizeof(pm8921_oem_chg_param_share));
}

static void oem_param_cycles_init(pm8921_oem_chg_param *ptr)
{
	if (ptr == NULL){
		pr_err("chg param cycles read error.\n");
		return;
	}

	memcpy(&oem_param_cycles, &ptr->oem_chg_param_cycles,
				sizeof(pm8921_oem_chg_param_cycles));
}

static int init_flag = 0;
void oem_chg_param_init(void)
{
	uint32_t *smem_ptr = NULL;
	uint32_t *cmp_ptr  = NULL;

	if (init_flag) {
		return;
	}

	smem_ptr = (uint32_t *)kc_smem_alloc(SMEM_CHG_PARAM, (CHG_PARAM_SIZE + SMEM_CHG_PARAM_CYCLE));
	if (smem_ptr == NULL) {
		pr_err("chg param read error.\n");
		init_flag = 1;
		return;
	}

	cmp_ptr = kmalloc((CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE), GFP_KERNEL);
	if (cmp_ptr == NULL) {
		pr_err("kmalloc error.\n");
		init_flag = 1;
		return;
	}

	memset(cmp_ptr, 0x00, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE));
	if (0 == memcmp(smem_ptr, cmp_ptr, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE))) {
		pr_err("smem data all '0'\n");
		memset(smem_ptr, 0xFF, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE));
		oem_cmp_zero_flag = 1;
	}

	oem_param_charger_init((pm8921_oem_chg_param*)smem_ptr);
	oem_param_hkadc_init((pm8921_oem_chg_param*)smem_ptr);
	oem_param_bms_init((pm8921_oem_chg_param*)smem_ptr);
	oem_param_share_init((pm8921_oem_chg_param*)smem_ptr);
	oem_param_cycles_init((pm8921_oem_chg_param*)smem_ptr);

	kfree(cmp_ptr);
	init_flag = 1;

}

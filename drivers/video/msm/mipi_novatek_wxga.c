/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/video/msm/mipi_novatek_cmd.c
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_novatek_wxga.h"
#include "mipi_novatek_wxga_tbl.h"
#include <linux/disp_ext_blc.h>
#include "mdp.h"
#include "mdp4.h"
#include "mipi_dsi.h"
#ifdef CONFIG_DISP_EXT_BLC
#include <linux/leds-lm3533.h>
#endif /* CONFIG_DISP_EXT_BLC */
#include "disp_ext.h"

static struct mipi_dsi_panel_platform_data *mipi_novatek_wxga_pdata;

#define TM_GET_PID(id) (((id) & 0xff00)>>8)

static struct dsi_buf novatek_wxga_tx_buf;
static struct dsi_buf novatek_wxga_rx_buf;
static int mipi_novatek_wxga_lcd_init(void);
static int  mipi_novatek_wxga_start_off_seq = 0;
static int dsi_boot_on = 0;

extern void mdp4_dsi_refresh_screen_at_once( struct msm_fb_data_type *mfd );

extern struct platform_device msm_mipi_dsi1_device;
static struct regulator *reg_l8, *reg_l2, *reg_lsv5;
static struct msm_fb_data_type *mipi_novatek_wxga_mfd;

/* #undef CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
#define CONFIG_MIPI_NOVATEK_LOCAL_WAIT
#undef CONFIG_MIPI_NOVATEK_LOCAL_WAIT_QUEUE
/* #define CONFIG_MIPI_NOVATEK_LOCAL_WAIT_QUEUE */

#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
static struct workqueue_struct *mipi_novatek_wq;
static struct hrtimer w_timer;
static struct work_struct w_work;
static struct completion w_done;
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */

#ifdef CONFIG_DISP_EXT_PROPERTY
void disp_gamma_exttbl_set(struct fb_kcjprop_data* kcjprop_data)
{
	int i;
	char* tbl_ptr;

	DISP_LOCAL_LOG_EMERG("DISP %s S\n",__func__);

	for(i=0; i<MSMFB_GAMMA_KCJPROP_DATA_NUM; i++){
		tbl_ptr = set_gamma_R_P_tbl[i];
		tbl_ptr++;
		*tbl_ptr = kcjprop_data->rw_display_gamma_normal_r_plus[i];
		DISP_LOCAL_LOG_EMERG("DISP %s set_gamma_R_P_%d[0x%02X]\n",
								__func__, i, (int)(*tbl_ptr));
	}
	for(i=0; i<MSMFB_GAMMA_KCJPROP_DATA_NUM; i++){
		tbl_ptr = set_gamma_R_M_tbl[i];
		tbl_ptr++;
		*tbl_ptr = kcjprop_data->rw_display_gamma_normal_r_minus[i];
		DISP_LOCAL_LOG_EMERG("DISP %s set_gamma_R_M_%d[0x%02X]\n",
								__func__, i, (int)(*tbl_ptr));
	}

	for(i=0; i<MSMFB_GAMMA_KCJPROP_DATA_NUM; i++){
		tbl_ptr = set_gamma_G_P_tbl[i];
		tbl_ptr++;
		*tbl_ptr = kcjprop_data->rw_display_gamma_normal_g_plus[i];
		DISP_LOCAL_LOG_EMERG("DISP %s set_gamma_G_P_%d[0x%02X]\n",
								__func__, i, (int)(*tbl_ptr));
	}
	for(i=0; i<MSMFB_GAMMA_KCJPROP_DATA_NUM; i++){
		tbl_ptr = set_gamma_G_M_tbl[i];
		tbl_ptr++;
		*tbl_ptr = kcjprop_data->rw_display_gamma_normal_g_minus[i];
		DISP_LOCAL_LOG_EMERG("DISP %s set_gamma_G_M_%d[0x%02X]\n",
								__func__, i, (int)(*tbl_ptr));
	}

	for(i=0; i<MSMFB_GAMMA_KCJPROP_DATA_NUM; i++){
		tbl_ptr = set_gamma_B_P_tbl[i];
		tbl_ptr++;
		*tbl_ptr = kcjprop_data->rw_display_gamma_normal_b_plus[i];
		DISP_LOCAL_LOG_EMERG("DISP %s set_gamma_B_P_%d[0x%02X]\n",
								__func__, i, (int)(*tbl_ptr));
	}
	for(i=0; i<MSMFB_GAMMA_KCJPROP_DATA_NUM; i++){
		tbl_ptr = set_gamma_B_M_tbl[i];
		tbl_ptr++;
		*tbl_ptr = kcjprop_data->rw_display_gamma_normal_b_minus[i];
		DISP_LOCAL_LOG_EMERG("DISP %s set_gamma_B_M_%d[0x%02X]\n",
								__func__, i, (int)(*tbl_ptr));
	}

	DISP_LOCAL_LOG_EMERG("DISP %s E\n",__func__);

	return;
}
#endif/* CONFIG_DISP_EXT_PROPERTY */

#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
static void mipi_novatek_w_work(struct work_struct *work)
{
	pr_debug("%s is called.\n", __func__);
	complete_all(&w_done);
}

static void mipi_novatek_wait(int time)
{
	int err;

	pr_debug("%s: %dms wait start.\n", __func__, time);
	init_completion(&w_done);
	hrtimer_start(&w_timer,  ktime_set(0, time * 1000000), HRTIMER_MODE_REL);

	err = wait_for_completion_interruptible_timeout(&w_done,
						msecs_to_jiffies((time * 20) + 100));

	if (err < 0) {
		pr_info("%s: Error while waiting!(%d)\n",__func__, err);
		hrtimer_cancel(&w_timer);
		msleep(time);
	} else if (err == 0) {
		pr_info("%s: Timedout while waiting!\n",__func__);
		hrtimer_cancel(&w_timer);
	}
	pr_debug("%s: wait complete.\n", __func__);
}

enum hrtimer_restart mipi_novatek_w_handler(struct hrtimer *timer)
{
	pr_debug("%s() is called.\n", __func__);

#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT_QUEUE
	queue_work(mipi_novatek_wq, &w_work);
#else /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT_QUEUE */
	complete_all(&w_done);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT_QUEUE */

	return HRTIMER_NORESTART;
}
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */


void mipi_novatek_wxga_off_sequence( struct msm_fb_data_type *mfd )
{
    DISP_LOCAL_LOG_EMERG("DISP %s S\n",__func__);

	mipi_dsi_cmds_tx(&novatek_wxga_tx_buf, novatek_wxga_cmd1_set_cmds,
					ARRAY_SIZE(novatek_wxga_cmd1_set_cmds));
	mipi_dsi_cmds_tx(&novatek_wxga_tx_buf, novatek_wxga_display_off_cmds,
					ARRAY_SIZE(novatek_wxga_display_off_cmds));
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
	mipi_novatek_wait(15);
#else
	msleep(15);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
	mipi_dsi_cmds_tx(&novatek_wxga_tx_buf, novatek_wxga_display_off_cmds2,
					ARRAY_SIZE(novatek_wxga_display_off_cmds2));
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
	mipi_novatek_wait(32);
#else
	msleep(32);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
	DISP_LOCAL_LOG_EMERG("DISP %s DCDC_En Low\n",__func__);
	gpio_set_value(138, 0);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
	mipi_novatek_wait(20);
#else
	msleep(20);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */

    DISP_LOCAL_LOG_EMERG("DISP %s E\n",__func__);
}

void mipi_novatek_wxga_initial_sequence( struct msm_fb_data_type *mfd )
{
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_initial_sequence S\n");
	mipi_dsi_cmds_tx(&novatek_wxga_tx_buf,
					novatek_wxga_initialize_cmds,
					ARRAY_SIZE(novatek_wxga_initialize_cmds));
	pr_debug("%s:initialize sequence\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_initial_sequence E\n");
}

void mipi_novatek_wxga_pre_initial_sequence( struct msm_fb_data_type *mfd )
{
    DISP_LOCAL_LOG_EMERG("DISP %s S\n",__func__ );

	mipi_dsi_cmds_tx(&novatek_wxga_tx_buf,
					novatek_wxga_pre_initialize_cmds,
					ARRAY_SIZE(novatek_wxga_pre_initialize_cmds));

    DISP_LOCAL_LOG_EMERG("DISP %s E\n",__func__ );
}

void mipi_novatek_wxga_sleep_out_sequence( struct msm_fb_data_type *mfd )
{
    DISP_LOCAL_LOG_EMERG("DISP %s S\n",__func__ );

	/* SleepOut */
	mipi_dsi_cmds_tx(&novatek_wxga_tx_buf,
					novatek_wxga_sleep_out_cmds,
					ARRAY_SIZE(novatek_wxga_sleep_out_cmds));

    DISP_LOCAL_LOG_EMERG("DISP %s E\n",__func__ );
}

void mipi_novatek_wxga_display_direction_sequence( struct msm_fb_data_type *mfd )
{
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_display_direction_sequence S\n");
	mipi_dsi_cmds_tx(&novatek_wxga_tx_buf,
					novatek_wxga_display_direction_cmds,
					ARRAY_SIZE(novatek_wxga_display_direction_cmds));
	pr_debug("%s:display direction\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_display_direction_sequence E\n");
}

void mipi_novatek_wxga_display_on_sequence( struct msm_fb_data_type *mfd )
{
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_display_on_sequence S\n");
	mipi_dsi_cmds_tx(&novatek_wxga_tx_buf,
					novatek_wxga_display_on_cmds,
					ARRAY_SIZE(novatek_wxga_display_on_cmds));
	pr_debug("%s:dsplay on sequence\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_display_on_sequence E\n");
}

int mipi_novatek_wxga_panel_power( int on )
{
	int rc;
	local_disp_state_enum disp_state;

	disp_state = disp_ext_util_get_disp_state();

	DISP_LOCAL_LOG_EMERG("%s start:state=%d req=%d\n", __func__,disp_state, on);

	if (on == 1) {
		if(disp_state == LOCAL_DISPLAY_OFF){
			// VLCDIO(PM8921 LVS5) ON
			DISP_LOCAL_LOG_EMERG("%s: enable:LVS5\n",__func__);
			rc = regulator_enable(reg_lsv5);
			if (rc) {
				pr_err("enable 8921_lvs5 failed, rc=%d\n", rc);
				goto panel_power_err;
			}
			// VLCD(PM8921 L8) ON
			DISP_LOCAL_LOG_EMERG("%s: enable:L8\n",__func__);
			rc = regulator_enable(reg_l8);
			if (rc) {
				pr_err("enable l8 failed, rc=%d\n", rc);
				goto panel_power_err;
			}
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
			mipi_novatek_wait(20);
#else
			msleep(20);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
			DISP_LOCAL_LOG_EMERG("%s: set_optimum_mode l2\n",__func__);
			rc = regulator_set_optimum_mode(reg_l2, 40000);
			if (rc < 0) {
				pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
				goto panel_power_err;
			}
			DISP_LOCAL_LOG_EMERG("%s: set_optimum_mode l8\n",__func__);
			rc = regulator_set_optimum_mode(reg_l8, 40000);
			if (rc < 0) {
				pr_err("set_optimum_mode l8 failed, rc=%d\n", rc);
				goto panel_power_err;
			}
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
			mipi_novatek_wait(1);
#else
			msleep(1);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
			/* release HW reset */
			gpio_set_value(48, 1);
	        DISP_LOCAL_LOG_EMERG("DISP %s Reset High\n",__func__);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
			mipi_novatek_wait(20);
#else
			msleep(20);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */

			return 0;
		}
		if(disp_state > LOCAL_DISPLAY_POWER_OFF) {
			DISP_LOCAL_LOG_EMERG("%s:end Double call of power supply ON\n",__func__);
			return 0;
		}
	}
	else {
		if(disp_state == LOCAL_DISPLAY_ON){
			/* release HW reset */  
	        DISP_LOCAL_LOG_EMERG("DISP %s Reset Low\n",__func__);
			gpio_set_value(48, 0);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
			mipi_novatek_wait(12);
#else
			msleep(12);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
			// VLCD(VCI)(PM8921 L8) OFF
			DISP_LOCAL_LOG_EMERG("%s: disable:L8\n",__func__);
			rc = regulator_disable(reg_l8);
			if (rc) {
				pr_err("disable reg_l8 failed, rc=%d\n", rc);
				goto panel_power_err;
			}
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
			mipi_novatek_wait(1);
#else
			msleep(1);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
			// VLCDIO(VDDI)(PM8921 LVS5) OFF
			DISP_LOCAL_LOG_EMERG("%s: disable:LVS5\n",__func__);
			rc = regulator_disable(reg_lsv5);
			if (rc) {
				pr_err("disable 8921_lvs5 failed, rc=%d\n", rc);
				goto panel_power_err;
			}
			DISP_LOCAL_LOG_EMERG("%s: set_optimum_mode:L2 Low current\n",__func__);
			rc = regulator_set_optimum_mode(reg_l2, 100);
			if (rc < 0) {
				pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			}
			DISP_LOCAL_LOG_EMERG("%s: set_optimum_mode:L8 Low current\n",__func__);
			rc = regulator_set_optimum_mode(reg_l8, 100);
			if (rc < 0) {
				pr_err("set_optimum_mode l8 failed, rc=%d\n", rc);
				goto panel_power_err;
			}
			DISP_LOCAL_LOG_EMERG("%s:end\n",__func__);
			return 0;
		}
		if(disp_state < LOCAL_DISPLAY_POWER_ON) {
			DISP_LOCAL_LOG_EMERG("%s:end Double call of power supply OFF\n",__func__);
			return 0;
		}
	}

	if (disp_state == LOCAL_DISPLAY_DEF) {
		reg_lsv5 = regulator_get(NULL,"8921_lvs5");
		if (IS_ERR(reg_lsv5)) {
			pr_err("could not get 8921_lvs5, rc = %ld\n",
				PTR_ERR(reg_lsv5));
			goto panel_power_err;
		}
		reg_l8 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi_vdc");
		if (IS_ERR(reg_l8)) {
			pr_err("could not get 8921_l8, rc = %ld\n",
				PTR_ERR(reg_l8));
			goto panel_power_err;
		}
		reg_l2 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi_vdda");
		if (IS_ERR(reg_l2)) {
			pr_err("could not get 8921_l2, rc = %ld\n",
				PTR_ERR(reg_l2));
			goto panel_power_err;
		}
		rc = regulator_set_voltage(reg_l8, 3000000, 3000000);
		if (rc) {
			pr_err("set_voltage l8 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		rc = regulator_set_voltage(reg_l2, 1200000, 1200000);
		if (rc) {
			pr_err("set_voltage l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		rc = gpio_request(48, "disp_rst_n");
		if (rc) {
			pr_err("request gpio 48 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		rc = gpio_request(138, "disp_dcdc_en");
		if (rc) {
			pr_err("request gpio 138 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		rc = gpio_tlmm_config( GPIO_CFG( 138,0,GPIO_CFG_OUTPUT,
										GPIO_CFG_NO_PULL,GPIO_CFG_2MA),
								GPIO_CFG_ENABLE );
		if (rc) {
			pr_err("config gpio 138 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		if (gpio_get_value(48) == 1) {
			DISP_LOCAL_LOG_EMERG("%s already power on\n", __func__);
			dsi_boot_on = 1;
		}
		else {
			dsi_boot_on = 0;
			DISP_LOCAL_LOG_EMERG("%s: low gpio48\n",__func__);
			gpio_set_value(48, 0);
		}

		disp_ext_util_set_disp_state(LOCAL_DISPLAY_POWER_OFF);
	}

	if (on)
	{
		// VLCDIO(PM8921 LVS5) ON
		DISP_LOCAL_LOG_EMERG("%s: enable:LVS5\n",__func__);
		rc = regulator_enable(reg_lsv5);
		if (rc) {
			pr_err("enable 8921_lvs5 failed, rc=%d\n", rc);
			goto panel_power_err;
		}

		// VLCD(PM8921 L8) ON
		DISP_LOCAL_LOG_EMERG("%s: enable:L8\n",__func__);
		rc = regulator_enable(reg_l8);
		if (rc) {
			pr_err("enable l8 failed, rc=%d\n", rc);
			goto panel_power_err;
		}

#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
		mipi_novatek_wait(20);
#else
		msleep(20);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */

		DISP_LOCAL_LOG_EMERG("%s: enable:L2\n",__func__);
		rc = regulator_enable(reg_l2);
		if (rc) {
			pr_err("enable l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}

		DISP_LOCAL_LOG_EMERG("%s: set_optimum_mode l2\n",__func__);
		rc = regulator_set_optimum_mode(reg_l2, 40000);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		DISP_LOCAL_LOG_EMERG("%s: set_optimum_mode l8\n",__func__);
		rc = regulator_set_optimum_mode(reg_l8, 40000);
		if (rc < 0) {
			pr_err("set_optimum_mode l8 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
		mipi_novatek_wait(1);
#else
		msleep(1);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
		/* release HW reset */
		gpio_set_value(48, 1);
        DISP_LOCAL_LOG_EMERG("DISP %s Reset High\n",__func__);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
		mipi_novatek_wait(20);
#else
		msleep(20);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
		disp_ext_util_set_disp_state(LOCAL_DISPLAY_POWER_ON);
	}
	else
	{
		disp_ext_util_set_disp_state(LOCAL_DISPLAY_POWER_OFF);
		/* VMIPI OFF */
		DISP_LOCAL_LOG_EMERG("%s: Disable L2\n",__func__);
		rc = regulator_disable(reg_l2);
		if (rc) {
			pr_err("disable reg_l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		DISP_LOCAL_LOG_EMERG("%s: set_optimum_mode:L2 Low current\n",__func__);
		rc = regulator_set_optimum_mode(reg_l2, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
	}
	DISP_LOCAL_LOG_EMERG("%s:end\n", __func__);

	return 0;

panel_power_err:
	DISP_LOCAL_LOG_EMERG("%s:err end\n", __func__);
	return -ENODEV;
}

int mipi_novatek_wxga_lcd_on_exec(struct msm_fb_data_type *mfd)
{
	struct fb_info *fbi;
	local_disp_state_enum disp_state;

	pr_info("checkpoint: %s:start\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_lcd_on_exec S\n");

#ifdef CONFIG_DISP_EXT_UTIL
	disp_ext_util_disp_local_lock();
#endif /* CONFIG_DISP_EXT_UTIL */
	disp_state = disp_ext_util_get_disp_state();
	if (disp_state == LOCAL_DISPLAY_ON) {
#ifdef CONFIG_DISP_EXT_UTIL
		disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */
		DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_lcd_on_exec disp_state(%d) E\n",disp_state);
		return 0;
	}

 	mipi_dsi_mdp_busy_wait();
	if (TM_GET_PID(mfd->panel.id) == MIPI_DSI_PANEL_WXGA) {
		/* LowSpeed */
		mipi_set_tx_power_mode(1);
		/* panel power on */
		if ( mipi_novatek_wxga_panel_power( 1 ) ) {
#ifdef CONFIG_DISP_EXT_UTIL
			disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */
			return -ENODEV;
		}

#ifdef CONFIG_DISP_EXT_BOARD
		if( disp_ext_board_detect_board(mfd) == -1 ) {
			mipi_novatek_wxga_panel_power( 0 );
			pr_err("%s:disp_ext_board_detect_board err:\n", __func__);
#ifdef CONFIG_DISP_EXT_UTIL
			disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */
			return 0;
		}
#endif /* CONFIG_DISP_EXT_BOARD */

		fbi = mfd->fbi;
		memset(fbi->screen_base, 0x00, fbi->fix.smem_len);
		pr_debug("%s:Frame buffer clear addr=%#X size=%d\n", __func__,(uint)fbi->screen_base,fbi->fix.smem_len);

		if( (mipi_novatek_wxga_start_off_seq == 0) && ( dsi_boot_on == 1) ) {
			mipi_novatek_wxga_start_off_seq = 1;
			/* -- off Sequence S */
			DISP_LOCAL_LOG_EMERG("%s:display off sequence1\n", __func__);
			mipi_dsi_cmds_tx(&novatek_wxga_tx_buf, novatek_wxga_cmd1_set_cmds,
							ARRAY_SIZE(novatek_wxga_cmd1_set_cmds));
			DISP_LOCAL_LOG_EMERG("%s:display off sequence2\n", __func__);
			mipi_dsi_cmds_tx(&novatek_wxga_tx_buf, novatek_wxga_display_off_cmds,
							ARRAY_SIZE(novatek_wxga_display_off_cmds));
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
			mipi_novatek_wait(15);
#else
			msleep(15);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
			DISP_LOCAL_LOG_EMERG("%s:display off sequence3\n", __func__);
			mipi_dsi_cmds_tx(&novatek_wxga_tx_buf, novatek_wxga_display_off_cmds2,
							ARRAY_SIZE(novatek_wxga_display_off_cmds2));
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
			mipi_novatek_wait(32);
#else
			msleep(32);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
			/* Enable VSP,VSN */
			DISP_LOCAL_LOG_EMERG("DISP %s DCDC_En Low\n",__func__);
			gpio_set_value(138, 0);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
			mipi_novatek_wait(20);
#else
			msleep(20);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
			/* -- off Sequence E */
		}

		mipi_novatek_wxga_pre_initial_sequence( mfd );

#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
		mipi_novatek_wait(10);
#else
		msleep(10);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
        DISP_LOCAL_LOG_EMERG("DISP %s Reset Low\n",__func__);
		gpio_set_value(48, 0);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
		mipi_novatek_wait(1);
#else
		msleep(1);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
        DISP_LOCAL_LOG_EMERG("DISP %s Reset High\n",__func__);
		gpio_set_value(48, 1);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
		mipi_novatek_wait(20);
#else
		msleep(20);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */

		mipi_novatek_wxga_initial_sequence( mfd );
		mipi_novatek_wxga_sleep_out_sequence( mfd );
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
		mipi_novatek_wait(70);
#else
		msleep(70);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
		/* Enable VSP,VSN */
		DISP_LOCAL_LOG_EMERG("DISP %s DCDC_En High\n",__func__); 
		gpio_set_value(138, 1);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
		mipi_novatek_wait(72);
#else
		msleep(72);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
		/* HighSpeed */
		mipi_set_tx_power_mode(0);

		mipi_novatek_wxga_display_direction_sequence( mfd );

		mipi_dsi_mdp_busy_wait();
		mdp4_dsi_refresh_screen_at_once( mfd );

		mipi_novatek_wxga_display_on_sequence( mfd );

		disp_ext_util_set_disp_state(LOCAL_DISPLAY_ON);
#ifdef CONFIG_DISP_EXT_REFRESH
		disp_ext_reflesh_set_sw(0);
		disp_ext_refresh_te_monitor_timer_release();
		disp_ext_reflesh_before_te_run_count_init();
#endif /* CONFIG_DISP_EXT_REFRESH */
	}
	else {
#ifdef CONFIG_DISP_EXT_UTIL
		disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */
		pr_err("%s:err end\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_DISP_EXT_UTIL
	disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */
	pr_info("checkpoint: %s:end\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_lcd_on_exec E\n");
	return 0;
}

static int mipi_novatek_wxga_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	pr_debug("%s:start\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_lcd_on\n");

	mfd = platform_get_drvdata(pdev);
	mipi_novatek_wxga_mfd = mfd;

	if (!mfd) {
		return -ENODEV;
	}
	if (mfd->key != MFD_KEY) {
		return -EINVAL;
	}

	return mipi_novatek_wxga_lcd_on_exec(mfd);
}

int mipi_novatek_wxga_lcd_off_exec(struct msm_fb_data_type *mfd)
{
	pr_info("checkpoint: %s:start\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_lcd_off_exec S\n");

#ifdef CONFIG_DISP_EXT_UTIL
	disp_ext_util_disp_local_lock();
#endif /* CONFIG_DISP_EXT_UTIL */
	if (disp_ext_util_get_disp_state() != LOCAL_DISPLAY_ON) {
#ifdef CONFIG_DISP_EXT_UTIL
		disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */
		DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_lcd_off_exec disp_state(%d) E\n",disp_ext_util_get_disp_state());
		return 0;
	}
	mipi_dsi_mdp_busy_wait();
#ifdef CONFIG_DISP_EXT_REFRESH
	disp_ext_refresh_te_monitor_timer_release();
#endif /* CONFIG_DISP_EXT_REFRESH */

	mipi_novatek_wxga_off_sequence(mfd);
	mipi_novatek_wxga_panel_power(0);

	disp_ext_util_set_disp_state(LOCAL_DISPLAY_OFF);
#ifdef CONFIG_DISP_EXT_REFRESH
	disp_ext_reflesh_set_sw(0);
#endif /* CONFIG_DISP_EXT_REFRESH */
#ifdef CONFIG_DISP_EXT_UTIL
	disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */
	pr_info("checkpoint: %s:end\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_lcd_off_exec E\n");
	return 0;
}

static int mipi_novatek_wxga_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	pr_debug("%s:start\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_lcd_off\n");

	mfd = platform_get_drvdata(pdev);

	if (!mfd) {
		return -ENODEV;
	}
	if (mfd->key != MFD_KEY) {
		return -EINVAL;
	}

	return mipi_novatek_wxga_lcd_off_exec(mfd);
}

static void mipi_novatek_wxga_shutdown(struct platform_device *pdev)
{
	pr_debug("%s:ST\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_shutdown S\n");

#ifdef CONFIG_DISP_EXT_BLC
	(void)light_led_cabc_set(LIGHT_MAIN_WLED_CABC_OFF);
#endif /* CONFIG_DISP_EXT_BLC */

#ifdef CONFIG_DISP_EXT_UTIL
	disp_ext_util_mipitx_lock();
#endif /* CONFIG_DISP_EXT_UTIL */
	if (disp_ext_util_get_disp_state() == LOCAL_DISPLAY_POWER_OFF) {
		pr_debug("%s() Display shutdown Now\n", __func__);
		DISP_LOCAL_LOG_EMERG("%s() Display shutdown Now\n", __func__);
#ifdef CONFIG_DISP_EXT_UTIL
		disp_ext_util_mipitx_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */
		return;
	}

	/* Clock ON */
	mipi_dsi_clk_cfg(1);
	mdp_clk_ctrl(1);

	if( mipi_novatek_wxga_mfd != NULL ) {
		if( disp_ext_util_get_disp_state() == LOCAL_DISPLAY_ON){
			pr_debug("%s() Display deepstandby \n", __func__);
			DISP_LOCAL_LOG_EMERG("%s() Display deepstandby \n", __func__);
			mipi_novatek_wxga_lcd_off_exec(mipi_novatek_wxga_mfd);
		}
	}

	/* Clock OFF */
	mdp_clk_ctrl(0);
	mipi_dsi_clk_cfg(0);

#ifdef CONFIG_DISP_UTIL_DSI_CLK_OFF
	disp_ext_util_dsi_clk_off();
#endif /*CONFIG_DISP_UTIL_DSI_CLK_OFF*/

	pr_debug("%s() Display power off \n", __func__);
	DISP_LOCAL_LOG_EMERG("%s() Display power off \n", __func__);
#ifdef CONFIG_DISP_EXT_UTIL
	disp_ext_util_disp_local_lock();
#endif /* CONFIG_DISP_EXT_UTIL */
	mipi_novatek_wxga_panel_power( 0 );
#ifdef CONFIG_DISP_EXT_UTIL
	disp_ext_util_disp_local_unlock();
	disp_ext_util_mipitx_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */

	pr_debug("%s() ED\n", __func__);
    DISP_LOCAL_LOG_EMERG("DISP mipi_novatek_wxga_shutdown E\n");
}

#ifdef CONFIG_DISP_EXT_REFRESH
void mipi_novatek_wxga_refresh_exec(struct msm_fb_data_type *mfd)
{
	DISP_LOCAL_LOG_EMERG("DISP %s start\n",__func__);

#ifdef CONFIG_DISP_EXT_UTIL
	disp_ext_util_disp_local_lock();
#endif /* CONFIG_DISP_EXT_UTIL */

	disp_ext_refresh_te_monitor_timer_release();
	disp_ext_reflesh_set_start(0);

	DISP_LOCAL_LOG_EMERG("DISP %s OFF start\n",__func__);
    /* LowSpeed */
	mipi_set_tx_power_mode(1);
    /* Clock ON */
	mipi_dsi_clk_cfg(1);
	mdp_clk_ctrl(1);

	mipi_novatek_wxga_off_sequence(mfd);
	mipi_novatek_wxga_panel_power(0);
	disp_ext_util_set_disp_state(LOCAL_DISPLAY_OFF);
    /* Clock OFF */
	mdp_clk_ctrl(0);
	mipi_dsi_clk_cfg(0);
	DISP_LOCAL_LOG_EMERG("DISP %s OFF end\n",__func__);

	DISP_LOCAL_LOG_EMERG("DISP %s ON start\n",__func__);
    /* Clock ON */
	mipi_dsi_clk_cfg(1);
	mdp_clk_ctrl(1);
	mipi_novatek_wxga_panel_power( 1 );
	mipi_novatek_wxga_pre_initial_sequence( mfd );
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
	mipi_novatek_wait(10);
#else
	msleep(10);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
	DISP_LOCAL_LOG_EMERG("DISP %s Reset Low\n",__func__);
	gpio_set_value(48, 0);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
	mipi_novatek_wait(1);
#else
	msleep(1);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
	DISP_LOCAL_LOG_EMERG("DISP %s Reset High\n",__func__);
	gpio_set_value(48, 1);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
	mipi_novatek_wait(20);
#else
	msleep(20);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
	mipi_novatek_wxga_initial_sequence( mfd );
	mipi_novatek_wxga_sleep_out_sequence( mfd );
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
	mipi_novatek_wait(70);
#else
	msleep(70);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
	DISP_LOCAL_LOG_EMERG("DISP %s DCDC_En High\n",__func__); 
	gpio_set_value(138, 1);
#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
	mipi_novatek_wait(72);
#else
	msleep(72);
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */
    /* HighSpeed */
	mipi_set_tx_power_mode(0);
	mipi_novatek_wxga_display_direction_sequence( mfd );
	mipi_dsi_mdp_busy_wait();
	mdp4_dsi_refresh_screen_at_once( mfd );
	mipi_novatek_wxga_display_on_sequence( mfd );
	disp_ext_util_set_disp_state(LOCAL_DISPLAY_ON);
    /* Clock OFF */
	mdp_clk_ctrl(0);
	mipi_dsi_clk_cfg(0);
	DISP_LOCAL_LOG_EMERG("DISP %s ON end\n",__func__);

	disp_ext_reflesh_set_sw(0);
	disp_ext_reflesh_set_start(1);
	disp_ext_reflesh_before_te_run_count_init();
	disp_ext_refresh_te_monitor_timer_set();

#ifdef CONFIG_DISP_EXT_UTIL
	disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */

	DISP_LOCAL_LOG_EMERG("DISP %s end\n",__func__);
	return;
}
#endif /* CONFIG_DISP_EXT_REFRESH */

static void mipi_novatek_wxga_set_backlight(struct msm_fb_data_type *mfd)
{
	pr_debug("No Support %s: back light level %d\n", __func__, mfd->bl_level);
}

static int __devinit mipi_novatek_wxga_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_novatek_wxga_pdata = pdev->dev.platform_data;
		return 0;
	}

	if (mipi_novatek_wxga_pdata == NULL) {
		pr_err("%s.invalid platform data.\n", __func__);
		return -ENODEV;
	}

	mipi_novatek_wxga_panel_power(1);

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_novatek_wxga_lcd_probe,
	.shutdown = mipi_novatek_wxga_shutdown,
	.driver = {
		.name   = "mipi_novatek_wxga",
	},
};

static struct msm_fb_panel_data novatek_wxga_panel_data = {
	.on		= mipi_novatek_wxga_lcd_on,
	.off		= mipi_novatek_wxga_lcd_off,
	.set_backlight  = mipi_novatek_wxga_set_backlight,
#ifdef CONFIG_DISP_EXT_UTIL
#ifdef CONFIG_DISP_EXT_PROPERTY
	.set_nv		= disp_ext_util_set_kcjprop,
#endif /* CONFIG_DISP_EXT_PROPERTY */
#endif /* CONFIG_DISP_EXT_UTIL */
#ifdef CONFIG_DISP_EXT_REFRESH
	.refresh	= disp_ext_refresh_seq,
#endif /* CONFIG_DISP_EXT_REFRESH */
};

static int ch_used[3];
int mipi_novatek_wxga_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel]) {
		return -ENODEV;
	}

	ch_used[channel] = TRUE;

#ifdef CONFIG_MIPI_NOVATEK_LOCAL_WAIT
	pr_info("%s: use mipi novatek local timer\n", __func__);

	mipi_novatek_wq = alloc_workqueue("mipi_novatek_wq", WQ_MEM_RECLAIM, 1);
	if (!mipi_novatek_wq){
		pr_err("%s: Failed to allocate mipi_novatek_wq.\n", __func__);
		return -ENOMEM;
	}

	INIT_WORK(&w_work, mipi_novatek_w_work);

	hrtimer_init(&w_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	w_timer.function  = mipi_novatek_w_handler;
#endif /* CONFIG_MIPI_NOVATEK_LOCAL_WAIT */


	ret = mipi_novatek_wxga_lcd_init();
	if (ret) {
		pr_err("mipi_novatek_wxga_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_novatek_wxga", (panel << 8)|channel);
	if (!pdev) {
		return -ENOMEM;
	}

	novatek_wxga_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &novatek_wxga_panel_data,
		sizeof(novatek_wxga_panel_data));
	if (ret) {
		pr_err("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

#ifdef CONFIG_DISP_EXT_REFRESH
	disp_ext_refresh_set_te_monitor_init();
#endif /* CONFIG_DISP_EXT_REFRESH */

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_novatek_wxga_lcd_init(void)
{
	mipi_dsi_buf_alloc(&novatek_wxga_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&novatek_wxga_rx_buf, DSI_BUF_SIZE);
#ifdef CONFIG_DISP_EXT_REFRESH
	disp_ext_reflesh_init();
#endif /* CONFIG_DISP_EXT_REFRESH */
#ifdef CONFIG_DISP_EXT_BLC
	disp_ext_blc_init();
#endif /* CONFIG_DISP_EXT_BLC */
	mipi_novatek_wxga_mfd = NULL;

	return platform_driver_register(&this_driver);
}

struct msm_fb_data_type *mipi_novatek_wxga_get_mfd(void)
{
	return mipi_novatek_wxga_mfd;
}

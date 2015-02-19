/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/video/msm/disp_ext_util.c
 *
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/msm_mdp.h>
#include "disp_ext.h"
#include "mipi_dsi.h"
#include "mipi_novatek_wxga.h"

static struct local_disp_state_type local_state;

/* #define DISP_EXT_UTIL_PROPERTY_CABC */

#ifdef CONFIG_DISP_EXT_UTIL_VSYNC
#define DISP_EXT_UTIL_TOTAL_LINE  1651
#define DISP_EXT_UTIL_VSYNC_COUNT  290
#define DISP_EXT_UTIL_VSYNC_OFFSET  364
uint32 disp_ext_util_get_total_line( void )
{
	uint32 total_line;
	total_line = DISP_EXT_UTIL_TOTAL_LINE -1;
    DISP_LOCAL_LOG_EMERG("DISP: %s - total line[%d]\n",__func__,total_line);

	return total_line;
}
uint32 disp_ext_util_get_vsync_count( void )
{
	uint32 cnt;
	cnt = DISP_EXT_UTIL_VSYNC_COUNT;
    DISP_LOCAL_LOG_EMERG("DISP: %s - Vsync count[%d]\n",__func__,cnt);

	return cnt;
}
uint32 disp_ext_util_vsync_cal_start( uint32 start_y )
{
	uint32 cal_start_y;
	cal_start_y = start_y + DISP_EXT_UTIL_VSYNC_OFFSET;
    DISP_LOCAL_LOG_EMERG("DISP: %s - Calibration start_y[%d]\n",__func__,cal_start_y);

	return cal_start_y;
}
#endif /*CONFIG_DISP_EXT_UTIL_VSYNC*/

#ifdef CONFIG_DISP_EXT_UTIL_GET_RATE
#define DISP_EXT_UTIL_REFRESH_RATE  61
uint32 disp_ext_util_get_refresh_rate( void )
{
	uint32 rate;
	rate = DISP_EXT_UTIL_REFRESH_RATE;
    DISP_LOCAL_LOG_EMERG("DISP: %s - Refresh Rate[%d]\n",__func__,rate);

	return rate;
}
#endif /*CONFIG_DISP_EXT_UTIL_GET_RATE*/

#ifdef CONFIG_DISP_UTIL_DSI_CLK_OFF
void disp_ext_util_dsi_clk_off( void )
{
	DISP_LOCAL_LOG_EMERG("%s start\n",__func__);

	local_bh_disable();
	mipi_dsi_clk_disable();
	local_bh_enable();

	MIPI_OUTP(MIPI_DSI_BASE + 0x0000, 0);

	mipi_dsi_phy_ctrl(0);

	local_bh_disable();
	mipi_dsi_ahb_ctrl(0);
	local_bh_enable();

	DISP_LOCAL_LOG_EMERG("%s end\n",__func__);

	return;
}
#endif /*CONFIG_DISP_UTIL_DSI_CLK_OFF*/

local_disp_state_enum disp_ext_util_get_disp_state(void)
{
    return local_state.disp_state;
}

void disp_ext_util_set_disp_state(local_disp_state_enum state)
{
    if( state > LOCAL_DISPLAY_TERM ) {
        return;
    }
    local_state.disp_state = state;
}

struct local_disp_state_type *disp_ext_util_get_disp_info(void)
{
    return &local_state;
}

struct semaphore disp_local_mutex;
void disp_ext_util_disp_local_init( void )
{
    sema_init(&disp_local_mutex,1);
    memset((void *)&local_state, 0, sizeof(struct local_disp_state_type));
}

void disp_ext_util_disp_local_lock( void )
{
    down(&disp_local_mutex);
}

void disp_ext_util_disp_local_unlock( void )
{
    up(&disp_local_mutex);
}

DEFINE_SEMAPHORE(disp_ext_util_mipitx_sem);
void disp_ext_util_mipitx_lock( void )
{
    down(&disp_ext_util_mipitx_sem);
}

void disp_ext_util_mipitx_unlock( void )
{
    up(&disp_ext_util_mipitx_sem);
}

#ifdef CONFIG_DISP_EXT_DIAG
uint32_t disp_ext_util_get_crc_error(void)
{
    return local_state.crc_error_count;
}

void disp_ext_util_set_crc_error(uint32_t count)
{
    local_state.crc_error_count = count;
}

void disp_ext_util_crc_countup(void)
{
    local_state.crc_error_count++;
}
#endif /* CONFIG_DISP_EXT_DIAG */

#ifdef CONFIG_DISP_EXT_PROPERTY
void disp_ext_util_set_kcjprop( struct fb_kcjprop_data* kcjprop_data )
{
    DISP_LOCAL_LOG_EMERG("DISP disp_ext_util_set_kcjprop S\n");
    disp_ext_util_disp_local_lock();
    if( kcjprop_data->rw_display_gamma_normal_valid == 0) {
        disp_gamma_exttbl_set(kcjprop_data);
        DISP_LOCAL_LOG_EMERG("%s:gamma ext set\n", __func__);
    }
#ifdef DISP_EXT_UTIL_PROPERTY_CABC
    if( kcjprop_data->rw_display_cabc_valid == 0) {
        if(kcjprop_data->rw_display_cabc > 3) {
            disp_ext_blc_set_select_mode(0);
        }
        else {
            disp_ext_blc_set_select_mode(kcjprop_data->rw_display_cabc);
        }
        DISP_LOCAL_LOG_EMERG("%s:select_mode=%d\n", __func__,disp_ext_blc_get_select_mode());
    }
#endif /* DISP_EXT_UTIL_PROPERTY_CABC */

#ifdef CONFIG_DISP_EXT_REFRESH
    if( kcjprop_data->rw_display_reflesh_valid == 0) {
        if(kcjprop_data->rw_display_reflesh != 0) {
            disp_ext_reflesh_set_enable(0);
            disp_ext_reflesh_set_start(0);
            DISP_LOCAL_LOG_EMERG("%s:display_reflesh_enable=%d\n", __func__,disp_ext_reflesh_get_enable());
        }
        else {
            disp_ext_reflesh_start();
        }
    }
#endif /* CONFIG_DISP_EXT_REFRESH */
    disp_ext_util_disp_local_unlock();
    DISP_LOCAL_LOG_EMERG("DISP disp_ext_util_set_kcjprop E\n");
}
#endif /* CONFIG_DISP_EXT_PROPERTY */

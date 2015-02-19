/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/video/msm/disp_ext_refresh.c
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include "disp_ext.h"
#include "mipi_dsi.h"
#include "mdp4.h"
#include "mipi_novatek_wxga.h"

#define DISP_EXT_REFRESH_TE_MONITER_TIME 3   /* 30ms */
#define DISP_EXT_REFRESH_REGIST_TIME     400 /* 20s */

static struct timer_list te_monitor_timer;
static uint8 display_reflesh_enable = 0;
static uint8 display_reflesh_start = 0;
static uint32 before_te_run_count = 0;
static uint32 te_run_count = 0;
static int err_report_chek_count=0;
static struct dsi_buf refresh_dsi_buff_tp;
static struct dsi_buf refresh_dsi_buff_rp;
static uint8_t display_reflesh_sw = 0;
static struct workqueue_struct *disp_ext_reflesh_wq;
struct work_struct disp_ext_reflesh_work;

extern void mdp4_dsi_refresh_screen_at_once( struct msm_fb_data_type *mfd );
extern void mdp_hw_vsync_irq_reg(void);
#ifdef CONFIG_DISP_EXT_BOARD
static void disp_ext_reflesh_start_regist(struct work_struct *work);
#endif /* CONFIG_DISP_EXT_BOARD */


void disp_ext_refresh_te_monitor_timer_set( void )
{
    if( display_reflesh_start != 1 ) {
        pr_debug("%s:reflesh function not start\n", __func__);
        return;
    }

    del_timer(&te_monitor_timer);
    te_monitor_timer.data = (unsigned long)NULL;
    te_monitor_timer.expires = jiffies + DISP_EXT_REFRESH_TE_MONITER_TIME;
    add_timer(&te_monitor_timer);
}

void disp_ext_refresh_te_monitor_timer_release( void )
{
    if( display_reflesh_start != 1 ) {
        pr_debug("%s:reflesh function not start\n", __func__);
        return;
    }
    del_timer(&te_monitor_timer);
}

static void disp_ext_refresh_te_monitor(unsigned long data)
{
    if( disp_ext_util_get_disp_state() != LOCAL_DISPLAY_ON ) {
        pr_debug("%s:panel off\n", __func__);
        return;
    }
    if( display_reflesh_start != 1 ) {
        pr_debug("%s:reflesh function not start\n", __func__);
        return;
    }

    disp_ext_refresh_te_monitor_timer_set();
    if( before_te_run_count == te_run_count ) {
        DISP_LOCAL_LOG_EMERG("DISP disp_ext_refresh_te_monitor refresh flag ON\n");
        disp_ext_reflesh_set_sw(1);
        return;
    }
    before_te_run_count = te_run_count;
}

void disp_ext_refresh_set_te_monitor_init(void)
{
    init_timer(&te_monitor_timer);
    te_monitor_timer.function = disp_ext_refresh_te_monitor;
}

void disp_ext_refresh_seq( struct msm_fb_data_type *mfd, unsigned int cmd )
{
    struct msm_fb_data_type *mipi_novatek_wxga_mfd;

    DISP_LOCAL_LOG_EMERG("DISP disp_ext_refresh_seq S\n");

    mipi_novatek_wxga_mfd = mipi_novatek_wxga_get_mfd();
    if( mipi_novatek_wxga_mfd == NULL ) {
		pr_debug("%s:bot found device\n", __func__);
        return;
    }
    if( disp_ext_reflesh_get_start() != 1 ) {
		DISP_LOCAL_LOG_EMERG("%s:refresh not start\n", __func__);
        return;
    }
#ifdef CONFIG_DISP_EXT_BOARD
	if(disp_ext_board_get_panel_detect() == -1) {
		DISP_LOCAL_LOG_EMERG("%s: Notconnect panel\n", __func__);
        return;
	}
#endif /*CONFIG_DISP_EXT_BOARD*/

	mipi_dsi_mdp_busy_wait();

    if( cmd == MIPI_REFREH_SEQ_ALL ) {
		mipi_novatek_wxga_refresh_exec(mipi_novatek_wxga_mfd);
    }

    DISP_LOCAL_LOG_EMERG("DISP disp_ext_refresh_seq E\n");
}

void disp_ext_refresh_err_report_check(struct msm_fb_data_type *mfd)
{
    struct dsi_cmd_desc dm_dsi_cmds;
    char mipi_reg;
    int ret;

    DISP_LOCAL_LOG_EMERG("DISP %s S\n",__func__);

    if(err_report_chek_count < 1) { 
        err_report_chek_count++;
        return;
    }

    memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
    mipi_reg = 0x0A;
    dm_dsi_cmds.dtype   = DTYPE_DCS_READ;     /* Command type */
    dm_dsi_cmds.last    = 1; /* Last command */
    dm_dsi_cmds.vc      = 0; /* Virtual Channel */
    dm_dsi_cmds.ack     = 1; /* Don't care, dsi_host default ON set */
    dm_dsi_cmds.wait    = 0; /* wait response by msleep() */
    dm_dsi_cmds.dlen    = 1; /* Data length */
    dm_dsi_cmds.payload = &mipi_reg;   /* Data */
    if (mdp_rev >= MDP_REV_41) {
        mutex_lock(&mfd->dma->ov_mutex);
    } else {
        down(&mfd->dma->mutex);
    }
#ifdef CONFIG_DISP_EXT_UTIL
    disp_ext_util_disp_local_lock();
#endif /* CONFIG_DISP_EXT_UTIL */
    mipi_dsi_op_mode_config(DSI_CMD_MODE);
    mipi_dsi_mdp_busy_wait();
    /* LowSpeed */
    mipi_set_tx_power_mode(1);
	mipi_dsi_clk_cfg(1);
    ret = mipi_dsi_cmds_rx(mfd, &refresh_dsi_buff_tp, &refresh_dsi_buff_rp, &dm_dsi_cmds, 2);
    mipi_dsi_clk_cfg(0);
    /* HighSpeed */
    mipi_set_tx_power_mode(0);
    mipi_dsi_op_mode_config(mfd->panel_info.mipi.mode);
#ifdef CONFIG_DISP_EXT_UTIL
    disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */
    if (mdp_rev >= MDP_REV_41) {
        mutex_unlock(&mfd->dma->ov_mutex);
    } else {
        up(&mfd->dma->mutex);
    }

    DISP_LOCAL_LOG_EMERG("DISP %s ret[%d],rd_val_0[%d],rd_val_1[%d],\n",
							__func__,(int)ret,
							(int)(refresh_dsi_buff_rp.data[0]),
							(int)(refresh_dsi_buff_rp.data[1]));

    if( (ret == 0) || (refresh_dsi_buff_rp.data[0] != 0x9C) ) {
        disp_ext_reflesh_set_sw(1);
    }

    DISP_LOCAL_LOG_EMERG("DISP %s E\n",__func__);
}

irqreturn_t disp_ext_refresh_hw_vsync_irq(int irq, void *data)
{
    te_run_count++;
    return IRQ_HANDLED;
}

uint8 disp_ext_reflesh_get_enable(void)
{
    return display_reflesh_enable;
}

void disp_ext_reflesh_set_enable(uint8 flag)
{
    if( flag > 1 ) {
        return;
    }
    DISP_LOCAL_LOG_EMERG("DISP %s flag[%d]\n", __func__,(int)flag);
    display_reflesh_enable = flag;
}

uint8 disp_ext_reflesh_get_start(void)
{
    return display_reflesh_start;
}

void disp_ext_reflesh_set_start(uint8 flag)
{
    if( flag > 1 ) {
        return;
    }
    display_reflesh_start = flag;
}

void disp_ext_reflesh_before_te_run_count_init(void)
{
    before_te_run_count = te_run_count;
}

void disp_ext_reflesh_init(void)
{
    mipi_dsi_buf_alloc(&refresh_dsi_buff_tp, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&refresh_dsi_buff_rp, DSI_BUF_SIZE);

#ifdef CONFIG_DISP_EXT_BOARD
    disp_ext_reflesh_wq = create_singlethread_workqueue("disp_ext_reflesh_wq");
    if(disp_ext_reflesh_wq)
    {
        INIT_WORK(&disp_ext_reflesh_work, disp_ext_reflesh_start_regist);
    }
#endif /* CONFIG_DISP_EXT_BOARD */
}

uint8_t disp_ext_reflesh_get_sw(void)
{
    return display_reflesh_sw;
}

void disp_ext_reflesh_set_sw(uint8_t sw)
{
    if(sw > 1) {
        return;
    }
    DISP_LOCAL_LOG_EMERG("DISP %s sw[%d]\n",__func__,(int)sw);
    display_reflesh_sw= sw;
}

#ifdef CONFIG_DISP_EXT_BOARD
static void disp_ext_reflesh_start_regist(struct work_struct *work)
{
    int panel_detect;
    int i;

    DISP_LOCAL_LOG_EMERG("DISP disp_ext_reflesh_start_regist\n");

    for( i=0; i<DISP_EXT_REFRESH_REGIST_TIME; i++ ) {
        msleep(50);
        disp_ext_util_disp_local_lock();
        panel_detect = disp_ext_board_get_panel_detect();
        switch( panel_detect ) {
        case 0:
            DISP_LOCAL_LOG_EMERG("DISP disp_ext_reflesh_start_regist check next\n");
            break;
        case 1:
            DISP_LOCAL_LOG_EMERG("DISP disp_ext_reflesh_start_regist resist\n");
            disp_ext_reflesh_set_enable(1);
            disp_ext_reflesh_set_start(1);
            mdp_hw_vsync_irq_reg();
            disp_ext_refresh_te_monitor_timer_release();
            disp_ext_reflesh_before_te_run_count_init();
            disp_ext_refresh_te_monitor_timer_set();
            disp_ext_util_disp_local_unlock();
            return;
        default:    /* -1,etc... */
            DISP_LOCAL_LOG_EMERG("DISP disp_ext_reflesh_start_regist panel not found\n");
            disp_ext_util_disp_local_unlock();
            return;
        }
        disp_ext_util_disp_local_unlock();
    }
}
#endif /* CONFIG_DISP_EXT_BOARD */

void disp_ext_reflesh_start(void)
{
#ifdef CONFIG_DISP_EXT_BOARD
    DISP_LOCAL_LOG_EMERG("DISP disp_ext_reflesh_start1\n");
    queue_work(disp_ext_reflesh_wq, &disp_ext_reflesh_work);
#else /* CONFIG_DISP_EXT_BOARD */
    DISP_LOCAL_LOG_EMERG("DISP disp_ext_reflesh_start2\n");
    disp_ext_reflesh_set_enable(1);
    disp_ext_reflesh_set_start(1);
    mdp_hw_vsync_irq_reg();
    disp_ext_refresh_te_monitor_timer_release();
    disp_ext_reflesh_before_te_run_count_init();
    disp_ext_refresh_te_monitor_timer_set();
#endif /* CONFIG_DISP_EXT_BOARD */
}

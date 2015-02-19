/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/video/msm/disp_ext_diag.c
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
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mdp4.h"
#include "disp_ext.h"
#ifdef CONFIG_FB_MSM_MIPI_NOVATEK_CMD_WXGA_PT
#include "mipi_novatek_wxga.h"
#endif /* CONFIG_FB_MSM_MIPI_NOVATEK_CMD_WXGA_PT */

static struct dsi_buf dm_dsi_buff_tp;
static struct dsi_buf dm_dsi_buff_rp;

int disp_ext_diag_get_err_crc(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
    void __user *argp = (void __user *)arg;
    struct dsi_cmd_desc dm_dsi_cmds;
    char mipi_reg;
    uint32 err_data;
	uint32 rd_data,timeout_cnt = 0x00;
    int ret;

    /* MIPI 05H register read Start*/
    memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
    mipi_reg = 0x5; /* RDNUMED reg(CRC ERR count read) */
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
    /* LowSpeed */
    mipi_set_tx_power_mode(1);
	mipi_dsi_clk_cfg(1);
    mipi_dsi_cmds_rx(mfd, &dm_dsi_buff_tp, &dm_dsi_buff_rp, &dm_dsi_cmds, 1);
    mipi_dsi_clk_cfg(0);
    msleep(100);
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
    /* MIPI 05H register read End*/

	rd_data = (uint32)(*(dm_dsi_buff_rp.data));
	timeout_cnt = (uint32)disp_ext_util_get_crc_error();

	rd_data = ((rd_data&0xFF00)>>8)|((rd_data&0x00FF)<<8);
	timeout_cnt = ((timeout_cnt&0xFF00)>>8)|((timeout_cnt&0x00FF)<<8);
	err_data = ((0x0000FFFF&timeout_cnt)<<16)
				| (0x0000FFFF&rd_data);

    /* Set arg */
    ret = copy_to_user(argp, &err_data, 4);
    return ret;
}

int disp_ext_diag_reg_write(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
    void __user *argp = (void __user *)arg;
    int ret;
    struct dsi_cmd_desc dm_dsi_cmds;
    struct disp_diag_mipi_reg_type mipi_reg_data;

    /* Get arg */
    ret = copy_from_user(&mipi_reg_data, argp, sizeof(mipi_reg_data));
    if (ret) {
        /* Error */
        pr_err("MSMFB_MIPI_REG_WRITE: error 1[%d] \n", ret);
        return ret;
    }

    /* command check */
    if ((mipi_reg_data.type != DTYPE_DCS_WRITE) &&
        (mipi_reg_data.type != DTYPE_DCS_WRITE1) &&
        (mipi_reg_data.type != DTYPE_DCS_LWRITE) &&
        (mipi_reg_data.type != DTYPE_GEN_WRITE) &&
        (mipi_reg_data.type != DTYPE_GEN_WRITE1) &&
        (mipi_reg_data.type != DTYPE_GEN_WRITE2) &&
        (mipi_reg_data.type != DTYPE_GEN_LWRITE))   {
        pr_err("MSMFB_MIPI_REG_WRITE: error 2[%d] \n", mipi_reg_data.type);
        return -EINVAL;
    }

    /* Tx command send */
    memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
    dm_dsi_cmds.dtype = mipi_reg_data.type;     /* Command type */
    dm_dsi_cmds.last = 1;   /* Last command */
    dm_dsi_cmds.vc = 0;     /* Virtual Channel */
    dm_dsi_cmds.ack = 0;    /* No ACK trigger msg from peripeheral */
    dm_dsi_cmds.wait = mipi_reg_data.wait;      /* wait response by msleep() */
    dm_dsi_cmds.dlen = mipi_reg_data.len;       /* Data length */
    dm_dsi_cmds.payload = (char *)mipi_reg_data.data;   /* Data */

    if (mdp_rev >= MDP_REV_41) {
        mutex_lock(&mfd->dma->ov_mutex);
    } else {
        down(&mfd->dma->mutex);
    }
#ifdef CONFIG_DISP_EXT_UTIL
    disp_ext_util_disp_local_lock();
#endif /* CONFIG_DISP_EXT_UTIL */
    mipi_dsi_op_mode_config(DSI_CMD_MODE);

    /* LowSpeed */
    mipi_set_tx_power_mode(1);

	mipi_dsi_clk_cfg(1);
    ret = mipi_dsi_cmds_tx( &dm_dsi_buff_tp, &dm_dsi_cmds, 1);
	mipi_dsi_clk_cfg(0);
    if( ret == 1 ) {
        ret = 0;
    }
    else {
        ret = -1;
    }

    msleep(100);

    /* HighSpeed */
    mipi_set_tx_power_mode(0);

    mipi_dsi_op_mode_config(mfd->panel_info.mipi.mode);
#ifdef CONFIG_DISP_EXT_UTIL
    disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */

#ifdef CONFIG_MSM_BUS_SCALING
/*  mdp_bus_scale_update_request(2); */
#endif
    if (mdp_rev >= MDP_REV_41) {
        mutex_unlock(&mfd->dma->ov_mutex);
    } else {
        up(&mfd->dma->mutex);
    }
    return ret;
}

int disp_ext_diag_reg_read(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
    void __user *argp = (void __user *)arg;
    int ret;
    struct dsi_cmd_desc dm_dsi_cmds;
    struct disp_diag_mipi_reg_type mipi_reg_data;

    /* Get arg */
    ret = copy_from_user(&mipi_reg_data, argp, sizeof(mipi_reg_data));
    if (ret) {
        /* Error */
        pr_err("MSMFB_MIPI_REG_READ: error 1[%d] \n", ret);
        return ret;
    }

    /* command check */
    if ((mipi_reg_data.type != DTYPE_DCS_READ) &&
        (mipi_reg_data.type != DTYPE_GEN_READ) &&
        (mipi_reg_data.type != DTYPE_GEN_READ1) &&
        (mipi_reg_data.type != DTYPE_GEN_READ2))   {
        pr_err("MSMFB_MIPI_REG_READ: error 2[%d] \n", mipi_reg_data.type);
        return -EINVAL;
    }

    /* Rx command send */
    memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
    dm_dsi_cmds.dtype = mipi_reg_data.type;     /* Command type */
    dm_dsi_cmds.last = 1;   /* Last command */
    dm_dsi_cmds.vc = 0;     /* Virtual Channel */
    dm_dsi_cmds.ack = 1;    /* Don't care, dsi_host default ON set */
    dm_dsi_cmds.wait = mipi_reg_data.wait;   /* wait response by msleep() */
    dm_dsi_cmds.dlen = mipi_reg_data.len;   /* Data length */
    dm_dsi_cmds.payload = (char *)mipi_reg_data.data;   /* Data */

    if (mdp_rev >= MDP_REV_41) {
        mutex_lock(&mfd->dma->ov_mutex);
    } else {
        down(&mfd->dma->mutex);
    }
#ifdef CONFIG_DISP_EXT_UTIL
    disp_ext_util_disp_local_lock();
#endif /* CONFIG_DISP_EXT_UTIL */
    mipi_dsi_op_mode_config(DSI_CMD_MODE);

    /* LowSpeed */
    mipi_set_tx_power_mode(1);

	mipi_dsi_clk_cfg(1);
    mipi_dsi_cmds_rx(mfd, &dm_dsi_buff_tp, &dm_dsi_buff_rp, &dm_dsi_cmds, mipi_reg_data.r_size);
    mipi_dsi_clk_cfg(0);

    msleep(100);

    /* HighSpeed */
    mipi_set_tx_power_mode(0);

    mipi_dsi_op_mode_config(mfd->panel_info.mipi.mode);
#ifdef CONFIG_DISP_EXT_UTIL
    disp_ext_util_disp_local_unlock();
#endif /* CONFIG_DISP_EXT_UTIL */

#ifdef CONFIG_MSM_BUS_SCALING
/*  mdp_bus_scale_update_request(2); */
#endif
    if (mdp_rev >= MDP_REV_41) {
        mutex_unlock(&mfd->dma->ov_mutex);
    } else {
        up(&mfd->dma->mutex);
    }

    memcpy(mipi_reg_data.data, dm_dsi_buff_rp.data, dm_dsi_buff_rp.len);

    /* Set arg */
    ret = copy_to_user(argp, &mipi_reg_data, sizeof(mipi_reg_data));
    if (ret) {
        pr_err("MSMFB_MIPI_REG_READ: error 3[%d] \n", ret);
        /* Error */
        return ret;
    }
    return ret;
}

int disp_ext_diag_standby(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
    void __user *argp = (void __user *)arg;
    int ret;
    unsigned int display_standby_flag;

    /* Get arg */
    ret = copy_from_user(&display_standby_flag, argp, sizeof(unsigned int));
    if (ret) {
        /* Error */
        pr_err("MSMFB_DISPLAY_STANDBY: error 1[%d] \n", ret);
        return ret;
    }

    /* command check */
	mipi_dsi_clk_cfg(1);
    switch( display_standby_flag ) {
    case 0:
#ifdef CONFIG_FB_MSM_MIPI_NOVATEK_CMD_WXGA_PT
        ret = mipi_novatek_wxga_lcd_on_exec(mfd);
#endif /* CONFIG_FB_MSM_MIPI_NOVATEK_CMD_WXGA_PT */
        break;
    case 1:
#ifdef CONFIG_FB_MSM_MIPI_NOVATEK_CMD_WXGA_PT
        ret = mipi_novatek_wxga_lcd_off_exec(mfd);
#endif /* CONFIG_FB_MSM_MIPI_NOVATEK_CMD_WXGA_PT */
        break;
    default:
        pr_err("MSMFB_DISPLAY_STANDBY: error 2[%d] \n", display_standby_flag);
        ret = -EINVAL;
    }
	mipi_dsi_clk_cfg(0);
    return ret;
}

int disp_ext_diag_tx_rate(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
    void __user *argp = (void __user *)arg;
    int ret;
    uint8 bpp;
    uint8 lanes = 0;
    uint32 input_data;
    uint32 msmfb_rate;
    uint32 dsi_pclk_rate;

    /* Get arg */
    ret = copy_from_user(&input_data, argp, sizeof(uint));
    if (ret) {
        pr_err("MSMFB_CHANGE_TRANSFER_RATE: error 1[%d] \n", ret);
        /* Error */
        return ret;
    }
    msmfb_rate = ((input_data << 8) & 0xFF00) | ((input_data >> 8) & 0x00FF);

    if ((mfd->panel_info.mipi.dst_format == DSI_CMD_DST_FORMAT_RGB888)
        || (mfd->panel_info.mipi.dst_format == DSI_VIDEO_DST_FORMAT_RGB888)
        || (mfd->panel_info.mipi.dst_format == DSI_VIDEO_DST_FORMAT_RGB666_LOOSE)) {
        bpp = 3;
    }
    else if ((mfd->panel_info.mipi.dst_format == DSI_CMD_DST_FORMAT_RGB565)
         || (mfd->panel_info.mipi.dst_format == DSI_VIDEO_DST_FORMAT_RGB565)) {
        bpp = 2;
    }
    else {
        bpp = 3;        /* Default format set to RGB888 */
    }

    if (mfd->panel_info.mipi.data_lane3) {
        lanes += 1;
    }
    if (mfd->panel_info.mipi.data_lane2) {
        lanes += 1;
    }
    if (mfd->panel_info.mipi.data_lane1) {
        lanes += 1;
    }
    if (mfd->panel_info.mipi.data_lane0) {
        lanes += 1;
    }

    mfd->panel_info.clk_rate = msmfb_rate * 1000000;
    pll_divider_config.clk_rate = mfd->panel_info.clk_rate;

    ret = mipi_dsi_clk_div_config(bpp, lanes, &dsi_pclk_rate);
    if (ret) {
        pr_err("MSMFB_CHANGE_TRANSFER_RATE: error 2[%d] \n", ret);
        /* Error */
        return ret;
    }

    if ((dsi_pclk_rate < 3300000) || (dsi_pclk_rate > 103300000)) {
        dsi_pclk_rate = 35000000;
    }
    mfd->panel_info.mipi.dsi_pclk_rate = dsi_pclk_rate;
    return 0;
}

void disp_ext_diag_init(void)
{
    mipi_dsi_buf_alloc(&dm_dsi_buff_tp, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&dm_dsi_buff_rp, DSI_BUF_SIZE);
}

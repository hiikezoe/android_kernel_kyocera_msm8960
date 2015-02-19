/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/video/msm/mipi_novatek_wxga.h
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
#ifndef MIPI_NOVATEK_WXGA_H
#define MIPI_NOVATEK_WXGA_H

int mipi_novatek_wxga_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);

int mipi_novatek_wxga_lcd_on_exec(struct msm_fb_data_type *mfd);
int mipi_novatek_wxga_lcd_off_exec(struct msm_fb_data_type *mfd);
void mipi_novatek_wxga_initial_sequence( struct msm_fb_data_type *mfd );
void mipi_novatek_wxga_pre_initial_sequence( struct msm_fb_data_type *mfd );
void mipi_novatek_wxga_sleep_out_sequence( struct msm_fb_data_type *mfd );
void mipi_novatek_wxga_display_direction_sequence( struct msm_fb_data_type *mfd );
void mipi_novatek_wxga_display_on_sequence( struct msm_fb_data_type *mfd );
int mipi_novatek_wxga_panel_power( int on );
struct msm_fb_data_type *mipi_novatek_wxga_get_mfd(void);

#ifdef CONFIG_DISP_EXT_REFRESH
void mipi_novatek_wxga_refresh_exec(struct msm_fb_data_type *mfd);
#endif /* CONFIG_DISP_EXT_REFRESH */

#ifndef CONFIG_DISP_EXT_UTIL
#include <linux/msm_mdp.h>

#define disp_ext_util_get_disp_state() LOCAL_POWER_OFF
#define disp_ext_util_set_disp_state(state)   (void)0;
#endif /* CONFIG_DISP_EXT_UTIL */
#ifdef CONFIG_DISP_EXT_PROPERTY
void disp_gamma_exttbl_set(struct fb_kcjprop_data* kcjprop_data);
#endif /* CONFIG_DISP_EXT_REFRESH */
#endif  /* MIPI_NOVATEK_WXGA_H */

/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/video/msm/mipi_novatek_wxga_tbl.h
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
#ifndef MIPI_NOVATEK_WXGA_TBL_H
#define MIPI_NOVATEK_WXGA_TBL_H

/* DTYPE_DCS_WRITE */
static char exit_sleep[2]  = {0x11, 0x00};
static char display_on[2]  = {0x29, 0x00};
static char display_off[2] = {0x28, 0x00};
static char enter_sleep[2] = {0x10, 0x00};

/* DTYPE_DCS_WRITE1 */
static char cmd1_select[2]    = {0xFF, 0x00};
static char cmd2_select_01[2] = {0xFF, 0x01};
static char cmd2_select_02[2] = {0xFF, 0x02};
static char cmd2_select_EE[2] = {0xFF, 0xEE};

static char gamset_08[2] = {0x26, 0x08};
static char gamset_00[2] = {0x26, 0x00};

static char reload_cmd1_01[2] = {0xFB, 0x01};
/* Gamma settings data *//* DTYPE_GEN_WRITE2 */
/* R+ */
static char set_gamma_R_P_001[2] = { 0x75, 0x00 };
static char set_gamma_R_P_002[2] = { 0x76, 0x00 };
static char set_gamma_R_P_003[2] = { 0x77, 0x00 };
static char set_gamma_R_P_004[2] = { 0x78, 0x0A };
static char set_gamma_R_P_005[2] = { 0x79, 0x00 };
static char set_gamma_R_P_006[2] = { 0x7A, 0x1A };
static char set_gamma_R_P_007[2] = { 0x7B, 0x00 };
static char set_gamma_R_P_008[2] = { 0x7C, 0x2A };
static char set_gamma_R_P_009[2] = { 0x7D, 0x00 };
static char set_gamma_R_P_010[2] = { 0x7E, 0x36 };
static char set_gamma_R_P_011[2] = { 0x7F, 0x00 };
static char set_gamma_R_P_012[2] = { 0x80, 0x42 };
static char set_gamma_R_P_013[2] = { 0x81, 0x00 };
static char set_gamma_R_P_014[2] = { 0x82, 0x51 };
static char set_gamma_R_P_015[2] = { 0x83, 0x00 };
static char set_gamma_R_P_016[2] = { 0x84, 0x5C };
static char set_gamma_R_P_017[2] = { 0x85, 0x00 };
static char set_gamma_R_P_018[2] = { 0x86, 0x65 };
static char set_gamma_R_P_019[2] = { 0x87, 0x00 };
static char set_gamma_R_P_020[2] = { 0x88, 0x8B };
static char set_gamma_R_P_021[2] = { 0x89, 0x00 };
static char set_gamma_R_P_022[2] = { 0x8A, 0xAB };
static char set_gamma_R_P_023[2] = { 0x8B, 0x00 };
static char set_gamma_R_P_024[2] = { 0x8C, 0xDF };
static char set_gamma_R_P_025[2] = { 0x8D, 0x01 };
static char set_gamma_R_P_026[2] = { 0x8E, 0x09 };
static char set_gamma_R_P_027[2] = { 0x8F, 0x01 };
static char set_gamma_R_P_028[2] = { 0x90, 0x4C };
static char set_gamma_R_P_029[2] = { 0x91, 0x01 };
static char set_gamma_R_P_030[2] = { 0x92, 0x81 };
static char set_gamma_R_P_031[2] = { 0x93, 0x01 };
static char set_gamma_R_P_032[2] = { 0x94, 0x84 };
static char set_gamma_R_P_033[2] = { 0x95, 0x01 };
static char set_gamma_R_P_034[2] = { 0x96, 0xB2 };
static char set_gamma_R_P_035[2] = { 0x97, 0x01 };
static char set_gamma_R_P_036[2] = { 0x98, 0xE1 };
static char set_gamma_R_P_037[2] = { 0x99, 0x01 };
static char set_gamma_R_P_038[2] = { 0x9A, 0xF7 };
static char set_gamma_R_P_039[2] = { 0x9B, 0x02 };
static char set_gamma_R_P_040[2] = { 0x9C, 0x14 };
static char set_gamma_R_P_041[2] = { 0x9D, 0x02 };
static char set_gamma_R_P_042[2] = { 0x9E, 0x24 };
static char set_gamma_R_P_043[2] = { 0x9F, 0x02 };
static char set_gamma_R_P_044[2] = { 0xA0, 0x3E };
static char set_gamma_R_P_045[2] = { 0xA2, 0x02 };
static char set_gamma_R_P_046[2] = { 0xA3, 0x49 };
static char set_gamma_R_P_047[2] = { 0xA4, 0x02 };
static char set_gamma_R_P_048[2] = { 0xA5, 0x6A };
static char set_gamma_R_P_049[2] = { 0xA6, 0x02 };
static char set_gamma_R_P_050[2] = { 0xA7, 0x7D };
static char set_gamma_R_P_051[2] = { 0xA9, 0x02 };
static char set_gamma_R_P_052[2] = { 0xAA, 0x95 };
static char set_gamma_R_P_053[2] = { 0xAB, 0x02 };
static char set_gamma_R_P_054[2] = { 0xAC, 0xB8 };
static char set_gamma_R_P_055[2] = { 0xAD, 0x02 };
static char set_gamma_R_P_056[2] = { 0xAE, 0xED };
static char set_gamma_R_P_057[2] = { 0xAF, 0x03 };
static char set_gamma_R_P_058[2] = { 0xB0, 0x43 };
static char set_gamma_R_P_059[2] = { 0xB1, 0x03 };
static char set_gamma_R_P_060[2] = { 0xB2, 0xF9 };
/* R- */
static char set_gamma_R_M_001[2] = { 0xB3, 0x00 };
static char set_gamma_R_M_002[2] = { 0xB4, 0x00 };
static char set_gamma_R_M_003[2] = { 0xB5, 0x00 };
static char set_gamma_R_M_004[2] = { 0xB6, 0x0A };
static char set_gamma_R_M_005[2] = { 0xB7, 0x00 };
static char set_gamma_R_M_006[2] = { 0xB8, 0x1A };
static char set_gamma_R_M_007[2] = { 0xB9, 0x00 };
static char set_gamma_R_M_008[2] = { 0xBA, 0x2A };
static char set_gamma_R_M_009[2] = { 0xBB, 0x00 };
static char set_gamma_R_M_010[2] = { 0xBC, 0x37 };
static char set_gamma_R_M_011[2] = { 0xBD, 0x00 };
static char set_gamma_R_M_012[2] = { 0xBE, 0x43 };
static char set_gamma_R_M_013[2] = { 0xBF, 0x00 };
static char set_gamma_R_M_014[2] = { 0xC0, 0x51 };
static char set_gamma_R_M_015[2] = { 0xC1, 0x00 };
static char set_gamma_R_M_016[2] = { 0xC2, 0x5C };
static char set_gamma_R_M_017[2] = { 0xC3, 0x00 };
static char set_gamma_R_M_018[2] = { 0xC4, 0x65 };
static char set_gamma_R_M_019[2] = { 0xC5, 0x00 };
static char set_gamma_R_M_020[2] = { 0xC6, 0x8B };
static char set_gamma_R_M_021[2] = { 0xC7, 0x00 };
static char set_gamma_R_M_022[2] = { 0xC8, 0xAC };
static char set_gamma_R_M_023[2] = { 0xC9, 0x00 };
static char set_gamma_R_M_024[2] = { 0xCA, 0xE0 };
static char set_gamma_R_M_025[2] = { 0xCB, 0x01 };
static char set_gamma_R_M_026[2] = { 0xCC, 0x09 };
static char set_gamma_R_M_027[2] = { 0xCD, 0x01 };
static char set_gamma_R_M_028[2] = { 0xCE, 0x4C };
static char set_gamma_R_M_029[2] = { 0xCF, 0x01 };
static char set_gamma_R_M_030[2] = { 0xD0, 0x82 };
static char set_gamma_R_M_031[2] = { 0xD1, 0x01 };
static char set_gamma_R_M_032[2] = { 0xD2, 0x84 };
static char set_gamma_R_M_033[2] = { 0xD3, 0x01 };
static char set_gamma_R_M_034[2] = { 0xD4, 0xB2 };
static char set_gamma_R_M_035[2] = { 0xD5, 0x01 };
static char set_gamma_R_M_036[2] = { 0xD6, 0xE1 };
static char set_gamma_R_M_037[2] = { 0xD7, 0x01 };
static char set_gamma_R_M_038[2] = { 0xD8, 0xF8 };
static char set_gamma_R_M_039[2] = { 0xD9, 0x02 };
static char set_gamma_R_M_040[2] = { 0xDA, 0x14 };
static char set_gamma_R_M_041[2] = { 0xDB, 0x02 };
static char set_gamma_R_M_042[2] = { 0xDC, 0x24 };
static char set_gamma_R_M_043[2] = { 0xDD, 0x02 };
static char set_gamma_R_M_044[2] = { 0xDE, 0x3E };
static char set_gamma_R_M_045[2] = { 0xDF, 0x02 };
static char set_gamma_R_M_046[2] = { 0xE0, 0x49 };
static char set_gamma_R_M_047[2] = { 0xE1, 0x02 };
static char set_gamma_R_M_048[2] = { 0xE2, 0x6A };
static char set_gamma_R_M_049[2] = { 0xE3, 0x02 };
static char set_gamma_R_M_050[2] = { 0xE4, 0x7D };
static char set_gamma_R_M_051[2] = { 0xE5, 0x02 };
static char set_gamma_R_M_052[2] = { 0xE6, 0x95 };
static char set_gamma_R_M_053[2] = { 0xE7, 0x02 };
static char set_gamma_R_M_054[2] = { 0xE8, 0xB8 };
static char set_gamma_R_M_055[2] = { 0xE9, 0x02 };
static char set_gamma_R_M_056[2] = { 0xEA, 0xED };
static char set_gamma_R_M_057[2] = { 0xEB, 0x03 };
static char set_gamma_R_M_058[2] = { 0xEC, 0x43 };
static char set_gamma_R_M_059[2] = { 0xED, 0x03 };
static char set_gamma_R_M_060[2] = { 0xEE, 0xF9 };
/* G+ */
static char set_gamma_G_P_001[2] = { 0xEF, 0x00 };
static char set_gamma_G_P_002[2] = { 0xF0, 0x00 };
static char set_gamma_G_P_003[2] = { 0xF1, 0x00 };
static char set_gamma_G_P_004[2] = { 0xF2, 0x0A };
static char set_gamma_G_P_005[2] = { 0xF3, 0x00 };
static char set_gamma_G_P_006[2] = { 0xF4, 0x1F };
static char set_gamma_G_P_007[2] = { 0xF5, 0x00 };
static char set_gamma_G_P_008[2] = { 0xF6, 0x32 };
static char set_gamma_G_P_009[2] = { 0xF7, 0x00 };
static char set_gamma_G_P_010[2] = { 0xF8, 0x42 };
static char set_gamma_G_P_011[2] = { 0xF9, 0x00 };
static char set_gamma_G_P_012[2] = { 0xFA, 0x52 };
static char set_gamma_G_P_013[2] = { 0x00, 0x00 };
static char set_gamma_G_P_014[2] = { 0x01, 0x61 };
static char set_gamma_G_P_015[2] = { 0x02, 0x00 };
static char set_gamma_G_P_016[2] = { 0x03, 0x6F };
static char set_gamma_G_P_017[2] = { 0x04, 0x00 };
static char set_gamma_G_P_018[2] = { 0x05, 0x7B };
static char set_gamma_G_P_019[2] = { 0x06, 0x00 };
static char set_gamma_G_P_020[2] = { 0x07, 0xA6 };
static char set_gamma_G_P_021[2] = { 0x08, 0x00 };
static char set_gamma_G_P_022[2] = { 0x09, 0xC7 };
static char set_gamma_G_P_023[2] = { 0x0A, 0x00 };
static char set_gamma_G_P_024[2] = { 0x0B, 0xFB };
static char set_gamma_G_P_025[2] = { 0x0C, 0x01 };
static char set_gamma_G_P_026[2] = { 0x0D, 0x24 };
static char set_gamma_G_P_027[2] = { 0x0E, 0x01 };
static char set_gamma_G_P_028[2] = { 0x0F, 0x63 };
static char set_gamma_G_P_029[2] = { 0x10, 0x01 };
static char set_gamma_G_P_030[2] = { 0x11, 0x95 };
static char set_gamma_G_P_031[2] = { 0x12, 0x01 };
static char set_gamma_G_P_032[2] = { 0x13, 0x96 };
static char set_gamma_G_P_033[2] = { 0x14, 0x01 };
static char set_gamma_G_P_034[2] = { 0x15, 0xC2 };
static char set_gamma_G_P_035[2] = { 0x16, 0x01 };
static char set_gamma_G_P_036[2] = { 0x17, 0xEF };
static char set_gamma_G_P_037[2] = { 0x18, 0x02 };
static char set_gamma_G_P_038[2] = { 0x19, 0x05 };
static char set_gamma_G_P_039[2] = { 0x1A, 0x02 };
static char set_gamma_G_P_040[2] = { 0x1B, 0x22 };
static char set_gamma_G_P_041[2] = { 0x1C, 0x02 };
static char set_gamma_G_P_042[2] = { 0x1D, 0x35 };
static char set_gamma_G_P_043[2] = { 0x1E, 0x02 };
static char set_gamma_G_P_044[2] = { 0x1F, 0x54 };
static char set_gamma_G_P_045[2] = { 0x20, 0x02 };
static char set_gamma_G_P_046[2] = { 0x21, 0x60 };
static char set_gamma_G_P_047[2] = { 0x22, 0x02 };
static char set_gamma_G_P_048[2] = { 0x23, 0x6A };
static char set_gamma_G_P_049[2] = { 0x24, 0x02 };
static char set_gamma_G_P_050[2] = { 0x25, 0x7D };
static char set_gamma_G_P_051[2] = { 0x26, 0x02 };
static char set_gamma_G_P_052[2] = { 0x27, 0x95 };
static char set_gamma_G_P_053[2] = { 0x28, 0x02 };
static char set_gamma_G_P_054[2] = { 0x29, 0xB8 };
static char set_gamma_G_P_055[2] = { 0x2A, 0x02 };
static char set_gamma_G_P_056[2] = { 0x2B, 0xED };
static char set_gamma_G_P_057[2] = { 0x2D, 0x03 };
static char set_gamma_G_P_058[2] = { 0x2F, 0x43 };
static char set_gamma_G_P_059[2] = { 0x30, 0x03 };
static char set_gamma_G_P_060[2] = { 0x31, 0xF9 };
/* G- */
static char set_gamma_G_M_001[2] = { 0x32, 0x00 };
static char set_gamma_G_M_002[2] = { 0x33, 0x00 };
static char set_gamma_G_M_003[2] = { 0x34, 0x00 };
static char set_gamma_G_M_004[2] = { 0x35, 0x0A };
static char set_gamma_G_M_005[2] = { 0x36, 0x00 };
static char set_gamma_G_M_006[2] = { 0x37, 0x1F };
static char set_gamma_G_M_007[2] = { 0x38, 0x00 };
static char set_gamma_G_M_008[2] = { 0x39, 0x32 };
static char set_gamma_G_M_009[2] = { 0x3A, 0x00 };
static char set_gamma_G_M_010[2] = { 0x3B, 0x43 };
static char set_gamma_G_M_011[2] = { 0x3D, 0x00 };
static char set_gamma_G_M_012[2] = { 0x3F, 0x53 };
static char set_gamma_G_M_013[2] = { 0x40, 0x00 };
static char set_gamma_G_M_014[2] = { 0x41, 0x61 };
static char set_gamma_G_M_015[2] = { 0x42, 0x00 };
static char set_gamma_G_M_016[2] = { 0x43, 0x6F };
static char set_gamma_G_M_017[2] = { 0x44, 0x00 };
static char set_gamma_G_M_018[2] = { 0x45, 0x7B };
static char set_gamma_G_M_019[2] = { 0x46, 0x00 };
static char set_gamma_G_M_020[2] = { 0x47, 0xA5 };
static char set_gamma_G_M_021[2] = { 0x48, 0x00 };
static char set_gamma_G_M_022[2] = { 0x49, 0xC7 };
static char set_gamma_G_M_023[2] = { 0x4A, 0x00 };
static char set_gamma_G_M_024[2] = { 0x4B, 0xFC };
static char set_gamma_G_M_025[2] = { 0x4C, 0x01 };
static char set_gamma_G_M_026[2] = { 0x4D, 0x25 };
static char set_gamma_G_M_027[2] = { 0x4E, 0x01 };
static char set_gamma_G_M_028[2] = { 0x4F, 0x63 };
static char set_gamma_G_M_029[2] = { 0x50, 0x01 };
static char set_gamma_G_M_030[2] = { 0x51, 0x95 };
static char set_gamma_G_M_031[2] = { 0x52, 0x01 };
static char set_gamma_G_M_032[2] = { 0x53, 0x97 };
static char set_gamma_G_M_033[2] = { 0x54, 0x01 };
static char set_gamma_G_M_034[2] = { 0x55, 0xC2 };
static char set_gamma_G_M_035[2] = { 0x56, 0x01 };
static char set_gamma_G_M_036[2] = { 0x58, 0xEF };
static char set_gamma_G_M_037[2] = { 0x59, 0x02 };
static char set_gamma_G_M_038[2] = { 0x5A, 0x05 };
static char set_gamma_G_M_039[2] = { 0x5B, 0x02 };
static char set_gamma_G_M_040[2] = { 0x5C, 0x23 };
static char set_gamma_G_M_041[2] = { 0x5D, 0x02 };
static char set_gamma_G_M_042[2] = { 0x5E, 0x35 };
static char set_gamma_G_M_043[2] = { 0x5F, 0x02 };
static char set_gamma_G_M_044[2] = { 0x60, 0x54 };
static char set_gamma_G_M_045[2] = { 0x61, 0x02 };
static char set_gamma_G_M_046[2] = { 0x62, 0x60 };
static char set_gamma_G_M_047[2] = { 0x63, 0x02 };
static char set_gamma_G_M_048[2] = { 0x64, 0x6A };
static char set_gamma_G_M_049[2] = { 0x65, 0x02 };
static char set_gamma_G_M_050[2] = { 0x66, 0x7D };
static char set_gamma_G_M_051[2] = { 0x67, 0x02 };
static char set_gamma_G_M_052[2] = { 0x68, 0x95 };
static char set_gamma_G_M_053[2] = { 0x69, 0x02 };
static char set_gamma_G_M_054[2] = { 0x6A, 0xB8 };
static char set_gamma_G_M_055[2] = { 0x6B, 0x02 };
static char set_gamma_G_M_056[2] = { 0x6C, 0xED };
static char set_gamma_G_M_057[2] = { 0x6D, 0x03 };
static char set_gamma_G_M_058[2] = { 0x6E, 0x43 };
static char set_gamma_G_M_059[2] = { 0x6F, 0x03 };
static char set_gamma_G_M_060[2] = { 0x70, 0xF9 };
/* B+ */
static char set_gamma_B_P_001[2] = { 0x71, 0x00 };
static char set_gamma_B_P_002[2] = { 0x72, 0x00 };
static char set_gamma_B_P_003[2] = { 0x73, 0x00 };
static char set_gamma_B_P_004[2] = { 0x74, 0x3D };
static char set_gamma_B_P_005[2] = { 0x75, 0x00 };
static char set_gamma_B_P_006[2] = { 0x76, 0x66 };
static char set_gamma_B_P_007[2] = { 0x77, 0x00 };
static char set_gamma_B_P_008[2] = { 0x78, 0x80 };
static char set_gamma_B_P_009[2] = { 0x79, 0x00 };
static char set_gamma_B_P_010[2] = { 0x7A, 0x91 };
static char set_gamma_B_P_011[2] = { 0x7B, 0x00 };
static char set_gamma_B_P_012[2] = { 0x7C, 0xA2 };
static char set_gamma_B_P_013[2] = { 0x7D, 0x00 };
static char set_gamma_B_P_014[2] = { 0x7E, 0xB1 };
static char set_gamma_B_P_015[2] = { 0x7F, 0x00 };
static char set_gamma_B_P_016[2] = { 0x80, 0xBD };
static char set_gamma_B_P_017[2] = { 0x81, 0x00 };
static char set_gamma_B_P_018[2] = { 0x82, 0xC7 };
static char set_gamma_B_P_019[2] = { 0x83, 0x00 };
static char set_gamma_B_P_020[2] = { 0x84, 0xEA };
static char set_gamma_B_P_021[2] = { 0x85, 0x01 };
static char set_gamma_B_P_022[2] = { 0x86, 0x04 };
static char set_gamma_B_P_023[2] = { 0x87, 0x01 };
static char set_gamma_B_P_024[2] = { 0x88, 0x2D };
static char set_gamma_B_P_025[2] = { 0x89, 0x01 };
static char set_gamma_B_P_026[2] = { 0x8A, 0x4D };
static char set_gamma_B_P_027[2] = { 0x8B, 0x01 };
static char set_gamma_B_P_028[2] = { 0x8C, 0x80 };
static char set_gamma_B_P_029[2] = { 0x8D, 0x01 };
static char set_gamma_B_P_030[2] = { 0x8E, 0xA9 };
static char set_gamma_B_P_031[2] = { 0x8F, 0x01 };
static char set_gamma_B_P_032[2] = { 0x90, 0xAB };
static char set_gamma_B_P_033[2] = { 0x91, 0x01 };
static char set_gamma_B_P_034[2] = { 0x92, 0xD1 };
static char set_gamma_B_P_035[2] = { 0x93, 0x01 };
static char set_gamma_B_P_036[2] = { 0x94, 0xF9 };
static char set_gamma_B_P_037[2] = { 0x95, 0x02 };
static char set_gamma_B_P_038[2] = { 0x96, 0x0C };
static char set_gamma_B_P_039[2] = { 0x97, 0x02 };
static char set_gamma_B_P_040[2] = { 0x98, 0x27 };
static char set_gamma_B_P_041[2] = { 0x99, 0x02 };
static char set_gamma_B_P_042[2] = { 0x9A, 0x37 };
static char set_gamma_B_P_043[2] = { 0x9B, 0x02 };
static char set_gamma_B_P_044[2] = { 0x9C, 0x52 };
static char set_gamma_B_P_045[2] = { 0x9D, 0x02 };
static char set_gamma_B_P_046[2] = { 0x9E, 0x5D };
static char set_gamma_B_P_047[2] = { 0x9F, 0x02 };
static char set_gamma_B_P_048[2] = { 0xA0, 0x6A };
static char set_gamma_B_P_049[2] = { 0xA2, 0x02 };
static char set_gamma_B_P_050[2] = { 0xA3, 0x7D };
static char set_gamma_B_P_051[2] = { 0xA4, 0x02 };
static char set_gamma_B_P_052[2] = { 0xA5, 0x95 };
static char set_gamma_B_P_053[2] = { 0xA6, 0x02 };
static char set_gamma_B_P_054[2] = { 0xA7, 0xB8 };
static char set_gamma_B_P_055[2] = { 0xA9, 0x02 };
static char set_gamma_B_P_056[2] = { 0xAA, 0xED };
static char set_gamma_B_P_057[2] = { 0xAB, 0x03 };
static char set_gamma_B_P_058[2] = { 0xAC, 0x43 };
static char set_gamma_B_P_059[2] = { 0xAD, 0x03 };
static char set_gamma_B_P_060[2] = { 0xAE, 0xF9 };
/* B- */
static char set_gamma_B_M_001[2] = { 0xAF, 0x00 };
static char set_gamma_B_M_002[2] = { 0xB0, 0x00 };
static char set_gamma_B_M_003[2] = { 0xB1, 0x00 };
static char set_gamma_B_M_004[2] = { 0xB2, 0x3D };
static char set_gamma_B_M_005[2] = { 0xB3, 0x00 };
static char set_gamma_B_M_006[2] = { 0xB4, 0x66 };
static char set_gamma_B_M_007[2] = { 0xB5, 0x00 };
static char set_gamma_B_M_008[2] = { 0xB6, 0x80 };
static char set_gamma_B_M_009[2] = { 0xB7, 0x00 };
static char set_gamma_B_M_010[2] = { 0xB8, 0x92 };
static char set_gamma_B_M_011[2] = { 0xB9, 0x00 };
static char set_gamma_B_M_012[2] = { 0xBA, 0xA3 };
static char set_gamma_B_M_013[2] = { 0xBB, 0x00 };
static char set_gamma_B_M_014[2] = { 0xBC, 0xB1 };
static char set_gamma_B_M_015[2] = { 0xBD, 0x00 };
static char set_gamma_B_M_016[2] = { 0xBE, 0xBD };
static char set_gamma_B_M_017[2] = { 0xBF, 0x00 };
static char set_gamma_B_M_018[2] = { 0xC0, 0xC7 };
static char set_gamma_B_M_019[2] = { 0xC1, 0x00 };
static char set_gamma_B_M_020[2] = { 0xC2, 0xEA };
static char set_gamma_B_M_021[2] = { 0xC3, 0x01 };
static char set_gamma_B_M_022[2] = { 0xC4, 0x05 };
static char set_gamma_B_M_023[2] = { 0xC5, 0x01 };
static char set_gamma_B_M_024[2] = { 0xC6, 0x2E };
static char set_gamma_B_M_025[2] = { 0xC7, 0x01 };
static char set_gamma_B_M_026[2] = { 0xC8, 0x4E };
static char set_gamma_B_M_027[2] = { 0xC9, 0x01 };
static char set_gamma_B_M_028[2] = { 0xCA, 0x80 };
static char set_gamma_B_M_029[2] = { 0xCB, 0x01 };
static char set_gamma_B_M_030[2] = { 0xCC, 0xAA };
static char set_gamma_B_M_031[2] = { 0xCD, 0x01 };
static char set_gamma_B_M_032[2] = { 0xCE, 0xAB };
static char set_gamma_B_M_033[2] = { 0xCF, 0x01 };
static char set_gamma_B_M_034[2] = { 0xD0, 0xD1 };
static char set_gamma_B_M_035[2] = { 0xD1, 0x01 };
static char set_gamma_B_M_036[2] = { 0xD2, 0xF9 };
static char set_gamma_B_M_037[2] = { 0xD3, 0x02 };
static char set_gamma_B_M_038[2] = { 0xD4, 0x0D };
static char set_gamma_B_M_039[2] = { 0xD5, 0x02 };
static char set_gamma_B_M_040[2] = { 0xD6, 0x27 };
static char set_gamma_B_M_041[2] = { 0xD7, 0x02 };
static char set_gamma_B_M_042[2] = { 0xD8, 0x37 };
static char set_gamma_B_M_043[2] = { 0xD9, 0x02 };
static char set_gamma_B_M_044[2] = { 0xDA, 0x52 };
static char set_gamma_B_M_045[2] = { 0xDB, 0x02 };
static char set_gamma_B_M_046[2] = { 0xDC, 0x5C };
static char set_gamma_B_M_047[2] = { 0xDD, 0x02 };
static char set_gamma_B_M_048[2] = { 0xDE, 0x6A };
static char set_gamma_B_M_049[2] = { 0xDF, 0x02 };
static char set_gamma_B_M_050[2] = { 0xE0, 0x7D };
static char set_gamma_B_M_051[2] = { 0xE1, 0x02 };
static char set_gamma_B_M_052[2] = { 0xE2, 0x95 };
static char set_gamma_B_M_053[2] = { 0xE3, 0x02 };
static char set_gamma_B_M_054[2] = { 0xE4, 0xB8 };
static char set_gamma_B_M_055[2] = { 0xE5, 0x02 };
static char set_gamma_B_M_056[2] = { 0xE6, 0xED };
static char set_gamma_B_M_057[2] = { 0xE7, 0x03 };
static char set_gamma_B_M_058[2] = { 0xE8, 0x43 };
static char set_gamma_B_M_059[2] = { 0xE9, 0x03 };
static char set_gamma_B_M_060[2] = { 0xEA, 0xF9 };

#ifdef CONFIG_DISP_EXT_PROPERTY
static char* set_gamma_R_P_tbl[MSMFB_GAMMA_KCJPROP_DATA_NUM] = {
set_gamma_R_P_001,set_gamma_R_P_002,set_gamma_R_P_003,set_gamma_R_P_004,
set_gamma_R_P_005,set_gamma_R_P_006,set_gamma_R_P_007,set_gamma_R_P_008,
set_gamma_R_P_009,set_gamma_R_P_010,set_gamma_R_P_011,set_gamma_R_P_012,
set_gamma_R_P_013,set_gamma_R_P_014,set_gamma_R_P_015,set_gamma_R_P_016,
set_gamma_R_P_017,set_gamma_R_P_018,set_gamma_R_P_019,set_gamma_R_P_020,
set_gamma_R_P_021,set_gamma_R_P_022,set_gamma_R_P_023,set_gamma_R_P_024,
set_gamma_R_P_025,set_gamma_R_P_026,set_gamma_R_P_027,set_gamma_R_P_028,
set_gamma_R_P_029,set_gamma_R_P_030,set_gamma_R_P_031,set_gamma_R_P_032,
set_gamma_R_P_033,set_gamma_R_P_034,set_gamma_R_P_035,set_gamma_R_P_036,
set_gamma_R_P_037,set_gamma_R_P_038,set_gamma_R_P_039,set_gamma_R_P_040,
set_gamma_R_P_041,set_gamma_R_P_042,set_gamma_R_P_043,set_gamma_R_P_044,
set_gamma_R_P_045,set_gamma_R_P_046,set_gamma_R_P_047,set_gamma_R_P_048,
set_gamma_R_P_049,set_gamma_R_P_050,set_gamma_R_P_051,set_gamma_R_P_052,
set_gamma_R_P_053,set_gamma_R_P_054,set_gamma_R_P_055,set_gamma_R_P_056,
set_gamma_R_P_057,set_gamma_R_P_058,set_gamma_R_P_059,set_gamma_R_P_060,
};

static char* set_gamma_R_M_tbl[MSMFB_GAMMA_KCJPROP_DATA_NUM] = {
set_gamma_R_M_001,set_gamma_R_M_002,set_gamma_R_M_003,set_gamma_R_M_004,
set_gamma_R_M_005,set_gamma_R_M_006,set_gamma_R_M_007,set_gamma_R_M_008,
set_gamma_R_M_009,set_gamma_R_M_010,set_gamma_R_M_011,set_gamma_R_M_012,
set_gamma_R_M_013,set_gamma_R_M_014,set_gamma_R_M_015,set_gamma_R_M_016,
set_gamma_R_M_017,set_gamma_R_M_018,set_gamma_R_M_019,set_gamma_R_M_020,
set_gamma_R_M_021,set_gamma_R_M_022,set_gamma_R_M_023,set_gamma_R_M_024,
set_gamma_R_M_025,set_gamma_R_M_026,set_gamma_R_M_027,set_gamma_R_M_028,
set_gamma_R_M_029,set_gamma_R_M_030,set_gamma_R_M_031,set_gamma_R_M_032,
set_gamma_R_M_033,set_gamma_R_M_034,set_gamma_R_M_035,set_gamma_R_M_036,
set_gamma_R_M_037,set_gamma_R_M_038,set_gamma_R_M_039,set_gamma_R_M_040,
set_gamma_R_M_041,set_gamma_R_M_042,set_gamma_R_M_043,set_gamma_R_M_044,
set_gamma_R_M_045,set_gamma_R_M_046,set_gamma_R_M_047,set_gamma_R_M_048,
set_gamma_R_M_049,set_gamma_R_M_050,set_gamma_R_M_051,set_gamma_R_M_052,
set_gamma_R_M_053,set_gamma_R_M_054,set_gamma_R_M_055,set_gamma_R_M_056,
set_gamma_R_M_057,set_gamma_R_M_058,set_gamma_R_M_059,set_gamma_R_M_060,
};

static char* set_gamma_G_P_tbl[MSMFB_GAMMA_KCJPROP_DATA_NUM] = {
set_gamma_G_P_001,set_gamma_G_P_002,set_gamma_G_P_003,set_gamma_G_P_004,
set_gamma_G_P_005,set_gamma_G_P_006,set_gamma_G_P_007,set_gamma_G_P_008,
set_gamma_G_P_009,set_gamma_G_P_010,set_gamma_G_P_011,set_gamma_G_P_012,
set_gamma_G_P_013,set_gamma_G_P_014,set_gamma_G_P_015,set_gamma_G_P_016,
set_gamma_G_P_017,set_gamma_G_P_018,set_gamma_G_P_019,set_gamma_G_P_020,
set_gamma_G_P_021,set_gamma_G_P_022,set_gamma_G_P_023,set_gamma_G_P_024,
set_gamma_G_P_025,set_gamma_G_P_026,set_gamma_G_P_027,set_gamma_G_P_028,
set_gamma_G_P_029,set_gamma_G_P_030,set_gamma_G_P_031,set_gamma_G_P_032,
set_gamma_G_P_033,set_gamma_G_P_034,set_gamma_G_P_035,set_gamma_G_P_036,
set_gamma_G_P_037,set_gamma_G_P_038,set_gamma_G_P_039,set_gamma_G_P_040,
set_gamma_G_P_041,set_gamma_G_P_042,set_gamma_G_P_043,set_gamma_G_P_044,
set_gamma_G_P_045,set_gamma_G_P_046,set_gamma_G_P_047,set_gamma_G_P_048,
set_gamma_G_P_049,set_gamma_G_P_050,set_gamma_G_P_051,set_gamma_G_P_052,
set_gamma_G_P_053,set_gamma_G_P_054,set_gamma_G_P_055,set_gamma_G_P_056,
set_gamma_G_P_057,set_gamma_G_P_058,set_gamma_G_P_059,set_gamma_G_P_060,
};

static char* set_gamma_G_M_tbl[MSMFB_GAMMA_KCJPROP_DATA_NUM] = {
set_gamma_G_M_001,set_gamma_G_M_002,set_gamma_G_M_003,set_gamma_G_M_004,
set_gamma_G_M_005,set_gamma_G_M_006,set_gamma_G_M_007,set_gamma_G_M_008,
set_gamma_G_M_009,set_gamma_G_M_010,set_gamma_G_M_011,set_gamma_G_M_012,
set_gamma_G_M_013,set_gamma_G_M_014,set_gamma_G_M_015,set_gamma_G_M_016,
set_gamma_G_M_017,set_gamma_G_M_018,set_gamma_G_M_019,set_gamma_G_M_020,
set_gamma_G_M_021,set_gamma_G_M_022,set_gamma_G_M_023,set_gamma_G_M_024,
set_gamma_G_M_025,set_gamma_G_M_026,set_gamma_G_M_027,set_gamma_G_M_028,
set_gamma_G_M_029,set_gamma_G_M_030,set_gamma_G_M_031,set_gamma_G_M_032,
set_gamma_G_M_033,set_gamma_G_M_034,set_gamma_G_M_035,set_gamma_G_M_036,
set_gamma_G_M_037,set_gamma_G_M_038,set_gamma_G_M_039,set_gamma_G_M_040,
set_gamma_G_M_041,set_gamma_G_M_042,set_gamma_G_M_043,set_gamma_G_M_044,
set_gamma_G_M_045,set_gamma_G_M_046,set_gamma_G_M_047,set_gamma_G_M_048,
set_gamma_G_M_049,set_gamma_G_M_050,set_gamma_G_M_051,set_gamma_G_M_052,
set_gamma_G_M_053,set_gamma_G_M_054,set_gamma_G_M_055,set_gamma_G_M_056,
set_gamma_G_M_057,set_gamma_G_M_058,set_gamma_G_M_059,set_gamma_G_M_060,
};

static char* set_gamma_B_P_tbl[MSMFB_GAMMA_KCJPROP_DATA_NUM] = {
set_gamma_B_P_001,set_gamma_B_P_002,set_gamma_B_P_003,set_gamma_B_P_004,
set_gamma_B_P_005,set_gamma_B_P_006,set_gamma_B_P_007,set_gamma_B_P_008,
set_gamma_B_P_009,set_gamma_B_P_010,set_gamma_B_P_011,set_gamma_B_P_012,
set_gamma_B_P_013,set_gamma_B_P_014,set_gamma_B_P_015,set_gamma_B_P_016,
set_gamma_B_P_017,set_gamma_B_P_018,set_gamma_B_P_019,set_gamma_B_P_020,
set_gamma_B_P_021,set_gamma_B_P_022,set_gamma_B_P_023,set_gamma_B_P_024,
set_gamma_B_P_025,set_gamma_B_P_026,set_gamma_B_P_027,set_gamma_B_P_028,
set_gamma_B_P_029,set_gamma_B_P_030,set_gamma_B_P_031,set_gamma_B_P_032,
set_gamma_B_P_033,set_gamma_B_P_034,set_gamma_B_P_035,set_gamma_B_P_036,
set_gamma_B_P_037,set_gamma_B_P_038,set_gamma_B_P_039,set_gamma_B_P_040,
set_gamma_B_P_041,set_gamma_B_P_042,set_gamma_B_P_043,set_gamma_B_P_044,
set_gamma_B_P_045,set_gamma_B_P_046,set_gamma_B_P_047,set_gamma_B_P_048,
set_gamma_B_P_049,set_gamma_B_P_050,set_gamma_B_P_051,set_gamma_B_P_052,
set_gamma_B_P_053,set_gamma_B_P_054,set_gamma_B_P_055,set_gamma_B_P_056,
set_gamma_B_P_057,set_gamma_B_P_058,set_gamma_B_P_059,set_gamma_B_P_060,
};

static char* set_gamma_B_M_tbl[MSMFB_GAMMA_KCJPROP_DATA_NUM] = {
set_gamma_B_M_001,set_gamma_B_M_002,set_gamma_B_M_003,set_gamma_B_M_004,
set_gamma_B_M_005,set_gamma_B_M_006,set_gamma_B_M_007,set_gamma_B_M_008,
set_gamma_B_M_009,set_gamma_B_M_010,set_gamma_B_M_011,set_gamma_B_M_012,
set_gamma_B_M_013,set_gamma_B_M_014,set_gamma_B_M_015,set_gamma_B_M_016,
set_gamma_B_M_017,set_gamma_B_M_018,set_gamma_B_M_019,set_gamma_B_M_020,
set_gamma_B_M_021,set_gamma_B_M_022,set_gamma_B_M_023,set_gamma_B_M_024,
set_gamma_B_M_025,set_gamma_B_M_026,set_gamma_B_M_027,set_gamma_B_M_028,
set_gamma_B_M_029,set_gamma_B_M_030,set_gamma_B_M_031,set_gamma_B_M_032,
set_gamma_B_M_033,set_gamma_B_M_034,set_gamma_B_M_035,set_gamma_B_M_036,
set_gamma_B_M_037,set_gamma_B_M_038,set_gamma_B_M_039,set_gamma_B_M_040,
set_gamma_B_M_041,set_gamma_B_M_042,set_gamma_B_M_043,set_gamma_B_M_044,
set_gamma_B_M_045,set_gamma_B_M_046,set_gamma_B_M_047,set_gamma_B_M_048,
set_gamma_B_M_049,set_gamma_B_M_050,set_gamma_B_M_051,set_gamma_B_M_052,
set_gamma_B_M_053,set_gamma_B_M_054,set_gamma_B_M_055,set_gamma_B_M_056,
set_gamma_B_M_057,set_gamma_B_M_058,set_gamma_B_M_059,set_gamma_B_M_060,
};
#endif/* CONFIG_DISP_EXT_PROPERTY */

/* display direction data */
/*static char display_direction_001[2] = {0x36, 0xC0};*/
static char display_direction_001[2] = {0x36, 0xD4};

static char initialize_1[2] = { 0xFF, 0xEE};
static char initialize_2[2] = { 0x05, 0x09};
static char initialize_3[2] = { 0xFB, 0x01};
static char initialize_4[2] = { 0xFF, 0x00};
static char initialize_5[2] = { 0xFF, 0xEE};
static char initialize_6[2] = { 0x12, 0x50};
static char initialize_7[2] = { 0x13, 0x02};
static char initialize_8[2] = { 0x6A, 0x60};
static char initialize_9[2] = { 0xFB, 0x01};
static char initialize_10[2] = { 0xFF, 0x01};
static char initialize_11[2] = { 0xFB, 0x01};
static char initialize_12[2] = { 0x00, 0x4A};
static char initialize_13[2] = { 0x01, 0x33};
static char initialize_14[2] = { 0x02, 0x53};
static char initialize_15[2] = { 0x03, 0x55};
static char initialize_16[2] = { 0x04, 0x55};
static char initialize_17[2] = { 0x05, 0x33};
static char initialize_18[2] = { 0x06, 0x22};
static char initialize_19[2] = { 0x08, 0x16};
static char initialize_20[2] = { 0x09, 0x8F};
static char initialize_21[2] = { 0x0B, 0xB7};
static char initialize_22[2] = { 0x0C, 0xB7};
static char initialize_23[2] = { 0x0D, 0x2F};
static char initialize_24[2] = { 0x0E, 0x2E};
static char initialize_25[2] = { 0x11, 0x8A};
static char initialize_26[2] = { 0x12, 0x03};
static char initialize_27[2] = { 0x0F, 0x0A};
static char initialize_28[2] = { 0x6F, 0x03};
static char initialize_29[2] = { 0x71, 0x2C};
static char initialize_30[2] = { 0x0A, 0x23};
static char initialize_31[2] = { 0xFF, 0x05};
static char initialize_32[2] = { 0xFB, 0x01};
static char initialize_33[2] = { 0x01, 0x00};
static char initialize_34[2] = { 0x02, 0x82};
static char initialize_35[2] = { 0x03, 0x82};
static char initialize_36[2] = { 0x04, 0x82};
static char initialize_37[2] = { 0x05, 0x30};
static char initialize_38[2] = { 0x06, 0x33};
static char initialize_39[2] = { 0x07, 0x01};
static char initialize_40[2] = { 0x08, 0x00};
static char initialize_41[2] = { 0x09, 0x46};
static char initialize_42[2] = { 0x0A, 0x46};
static char initialize_43[2] = { 0x0D, 0x0B};
static char initialize_44[2] = { 0x0E, 0x1D};
static char initialize_45[2] = { 0x0F, 0x08};
static char initialize_46[2] = { 0x10, 0x53};
static char initialize_47[2] = { 0x11, 0x00};
static char initialize_48[2] = { 0x12, 0x00};
static char initialize_49[2] = { 0x14, 0x01};
static char initialize_50[2] = { 0x15, 0x00};
static char initialize_51[2] = { 0x16, 0x05};
static char initialize_52[2] = { 0x17, 0x02};
static char initialize_53[2] = { 0x19, 0x7F};
static char initialize_54[2] = { 0x1A, 0xFF};
static char initialize_55[2] = { 0x1B, 0xFF};
static char initialize_56[2] = { 0x1C, 0x00};
static char initialize_57[2] = { 0x1D, 0x00};
static char initialize_58[2] = { 0x1E, 0x10};
static char initialize_59[2] = { 0x1F, 0x07};
static char initialize_60[2] = { 0x20, 0x00};
static char initialize_61[2] = { 0x21, 0x06};
static char initialize_62[2] = { 0x22, 0x11};
static char initialize_63[2] = { 0x23, 0x09};
static char initialize_64[2] = { 0x2D, 0x02};
static char initialize_65[2] = { 0x83, 0x02};
static char initialize_66[2] = { 0x9E, 0x58};
static char initialize_67[2] = { 0x9F, 0x68};
static char initialize_68[2] = { 0xA0, 0x41};
static char initialize_69[2] = { 0xA2, 0x10};
static char initialize_70[2] = { 0xBB, 0x0A};
static char initialize_71[2] = { 0xBC, 0x0A};
static char initialize_72[2] = { 0x6C, 0x03};
static char initialize_73[2] = { 0x6D, 0x03};
static char initialize_74[2] = { 0x28, 0x01};
static char initialize_75[2] = { 0x2F, 0x02};
static char initialize_76[2] = { 0x32, 0x08};
static char initialize_77[2] = { 0x33, 0xB8};
static char initialize_78[2] = { 0x36, 0x01};
static char initialize_79[2] = { 0x37, 0x00};
static char initialize_80[2] = { 0x43, 0x00};
static char initialize_81[2] = { 0x4B, 0x21};
static char initialize_82[2] = { 0x4C, 0x03};
static char initialize_83[2] = { 0x50, 0x21};
static char initialize_84[2] = { 0x51, 0x03};
static char initialize_85[2] = { 0x58, 0x21};
static char initialize_86[2] = { 0x59, 0x03};
static char initialize_87[2] = { 0x5D, 0x21};
static char initialize_88[2] = { 0x5E, 0x03};
static char initialize_89[2] = { 0xFF, 0x00};
static char initialize_cabc_1[2] = { 0xFB, 0x01};
static char initialize_cabc_2[2] = { 0x51, 0xFF};
static char initialize_cabc_3[2] = { 0x53, 0x2C};
static char initialize_cabc_4[2] = { 0x55, 0x01};
static char initialize_90[2] = { 0xC2, 0x08};
static char initialize_91[2] = { 0xBA, 0x03};
static char initialize_TE[2] = { 0x35, 0x00};
/*
  ---------------------------------------------------------------------
*/
static struct dsi_cmd_desc novatek_wxga_pre_initialize_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(cmd2_select_EE), cmd2_select_EE},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(gamset_08), gamset_08},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(gamset_00), gamset_00},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(cmd1_select), cmd1_select}
};

/* initializeÅ` */
static struct dsi_cmd_desc novatek_wxga_initialize_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_1), initialize_1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_2), initialize_2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_3), initialize_3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_4), initialize_4},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_5), initialize_5},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_6), initialize_6},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_7), initialize_7},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_8), initialize_8},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_9), initialize_9},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_10), initialize_10},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_11), initialize_11},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_12), initialize_12},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_13), initialize_13},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_14), initialize_14},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_15), initialize_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_16), initialize_16},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_17), initialize_17},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_18), initialize_18},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_19), initialize_19},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_20), initialize_20},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_21), initialize_21},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_22), initialize_22},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_23), initialize_23},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_24), initialize_24},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_25), initialize_25},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_26), initialize_26},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_27), initialize_27},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_28), initialize_28},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_29), initialize_29},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_30), initialize_30},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_31), initialize_31},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_32), initialize_32},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_33), initialize_33},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_34), initialize_34},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_35), initialize_35},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_36), initialize_36},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_37), initialize_37},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_38), initialize_38},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_39), initialize_39},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_40), initialize_40},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_41), initialize_41},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_42), initialize_42},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_43), initialize_43},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_44), initialize_44},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_45), initialize_45},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_46), initialize_46},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_47), initialize_47},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_48), initialize_48},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_49), initialize_49},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_50), initialize_50},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_51), initialize_51},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_52), initialize_52},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_53), initialize_53},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_54), initialize_54},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_55), initialize_55},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_56), initialize_56},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_57), initialize_57},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_58), initialize_58},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_59), initialize_59},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_60), initialize_60},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_61), initialize_61},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_62), initialize_62},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_63), initialize_63},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_64), initialize_64},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_65), initialize_65},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_66), initialize_66},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_67), initialize_67},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_68), initialize_68},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_69), initialize_69},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_70), initialize_70},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_71), initialize_71},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_72), initialize_72},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_73), initialize_73},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_74), initialize_74},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_75), initialize_75},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_76), initialize_76},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_77), initialize_77},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_78), initialize_78},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_79), initialize_79},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_80), initialize_80},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_81), initialize_81},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_82), initialize_82},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_83), initialize_83},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_84), initialize_84},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_85), initialize_85},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_86), initialize_86},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_87), initialize_87},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_88), initialize_88},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_89), initialize_89},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_cabc_1), initialize_cabc_1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_cabc_2), initialize_cabc_2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_cabc_3), initialize_cabc_3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_cabc_4), initialize_cabc_4},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_90), initialize_90},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_91), initialize_91},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(initialize_TE), initialize_TE},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(cmd2_select_01), cmd2_select_01},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(reload_cmd1_01), reload_cmd1_01},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_001), set_gamma_R_P_001},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_002), set_gamma_R_P_002},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_003), set_gamma_R_P_003},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_004), set_gamma_R_P_004},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_005), set_gamma_R_P_005},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_006), set_gamma_R_P_006},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_007), set_gamma_R_P_007},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_008), set_gamma_R_P_008},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_009), set_gamma_R_P_009},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_010), set_gamma_R_P_010},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_011), set_gamma_R_P_011},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_012), set_gamma_R_P_012},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_013), set_gamma_R_P_013},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_014), set_gamma_R_P_014},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_015), set_gamma_R_P_015},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_016), set_gamma_R_P_016},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_017), set_gamma_R_P_017},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_018), set_gamma_R_P_018},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_019), set_gamma_R_P_019},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_020), set_gamma_R_P_020},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_021), set_gamma_R_P_021},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_022), set_gamma_R_P_022},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_023), set_gamma_R_P_023},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_024), set_gamma_R_P_024},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_025), set_gamma_R_P_025},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_026), set_gamma_R_P_026},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_027), set_gamma_R_P_027},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_028), set_gamma_R_P_028},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_029), set_gamma_R_P_029},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_030), set_gamma_R_P_030},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_031), set_gamma_R_P_031},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_032), set_gamma_R_P_032},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_033), set_gamma_R_P_033},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_034), set_gamma_R_P_034},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_035), set_gamma_R_P_035},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_036), set_gamma_R_P_036},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_037), set_gamma_R_P_037},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_038), set_gamma_R_P_038},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_039), set_gamma_R_P_039},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_040), set_gamma_R_P_040},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_041), set_gamma_R_P_041},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_042), set_gamma_R_P_042},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_043), set_gamma_R_P_043},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_044), set_gamma_R_P_044},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_045), set_gamma_R_P_045},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_046), set_gamma_R_P_046},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_047), set_gamma_R_P_047},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_048), set_gamma_R_P_048},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_049), set_gamma_R_P_049},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_050), set_gamma_R_P_050},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_051), set_gamma_R_P_051},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_052), set_gamma_R_P_052},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_053), set_gamma_R_P_053},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_054), set_gamma_R_P_054},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_055), set_gamma_R_P_055},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_056), set_gamma_R_P_056},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_057), set_gamma_R_P_057},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_058), set_gamma_R_P_058},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_059), set_gamma_R_P_059},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_P_060), set_gamma_R_P_060},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_001), set_gamma_R_M_001},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_002), set_gamma_R_M_002},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_003), set_gamma_R_M_003},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_004), set_gamma_R_M_004},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_005), set_gamma_R_M_005},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_006), set_gamma_R_M_006},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_007), set_gamma_R_M_007},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_008), set_gamma_R_M_008},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_009), set_gamma_R_M_009},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_010), set_gamma_R_M_010},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_011), set_gamma_R_M_011},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_012), set_gamma_R_M_012},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_013), set_gamma_R_M_013},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_014), set_gamma_R_M_014},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_015), set_gamma_R_M_015},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_016), set_gamma_R_M_016},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_017), set_gamma_R_M_017},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_018), set_gamma_R_M_018},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_019), set_gamma_R_M_019},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_020), set_gamma_R_M_020},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_021), set_gamma_R_M_021},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_022), set_gamma_R_M_022},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_023), set_gamma_R_M_023},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_024), set_gamma_R_M_024},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_025), set_gamma_R_M_025},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_026), set_gamma_R_M_026},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_027), set_gamma_R_M_027},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_028), set_gamma_R_M_028},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_029), set_gamma_R_M_029},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_030), set_gamma_R_M_030},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_031), set_gamma_R_M_031},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_032), set_gamma_R_M_032},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_033), set_gamma_R_M_033},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_034), set_gamma_R_M_034},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_035), set_gamma_R_M_035},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_036), set_gamma_R_M_036},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_037), set_gamma_R_M_037},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_038), set_gamma_R_M_038},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_039), set_gamma_R_M_039},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_040), set_gamma_R_M_040},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_041), set_gamma_R_M_041},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_042), set_gamma_R_M_042},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_043), set_gamma_R_M_043},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_044), set_gamma_R_M_044},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_045), set_gamma_R_M_045},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_046), set_gamma_R_M_046},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_047), set_gamma_R_M_047},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_048), set_gamma_R_M_048},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_049), set_gamma_R_M_049},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_050), set_gamma_R_M_050},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_051), set_gamma_R_M_051},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_052), set_gamma_R_M_052},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_053), set_gamma_R_M_053},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_054), set_gamma_R_M_054},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_055), set_gamma_R_M_055},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_056), set_gamma_R_M_056},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_057), set_gamma_R_M_057},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_058), set_gamma_R_M_058},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_059), set_gamma_R_M_059},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_R_M_060), set_gamma_R_M_060},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_001), set_gamma_G_P_001},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_002), set_gamma_G_P_002},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_003), set_gamma_G_P_003},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_004), set_gamma_G_P_004},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_005), set_gamma_G_P_005},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_006), set_gamma_G_P_006},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_007), set_gamma_G_P_007},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_008), set_gamma_G_P_008},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_009), set_gamma_G_P_009},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_010), set_gamma_G_P_010},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_011), set_gamma_G_P_011},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_012), set_gamma_G_P_012},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(cmd1_select), cmd1_select},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(cmd2_select_02), cmd2_select_02},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(reload_cmd1_01), reload_cmd1_01},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_013), set_gamma_G_P_013},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_014), set_gamma_G_P_014},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_015), set_gamma_G_P_015},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_016), set_gamma_G_P_016},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_017), set_gamma_G_P_017},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_018), set_gamma_G_P_018},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_019), set_gamma_G_P_019},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_020), set_gamma_G_P_020},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_021), set_gamma_G_P_021},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_022), set_gamma_G_P_022},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_023), set_gamma_G_P_023},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_024), set_gamma_G_P_024},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_025), set_gamma_G_P_025},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_026), set_gamma_G_P_026},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_027), set_gamma_G_P_027},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_028), set_gamma_G_P_028},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_029), set_gamma_G_P_029},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_030), set_gamma_G_P_030},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_031), set_gamma_G_P_031},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_032), set_gamma_G_P_032},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_033), set_gamma_G_P_033},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_034), set_gamma_G_P_034},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_035), set_gamma_G_P_035},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_036), set_gamma_G_P_036},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_037), set_gamma_G_P_037},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_038), set_gamma_G_P_038},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_039), set_gamma_G_P_039},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_040), set_gamma_G_P_040},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_041), set_gamma_G_P_041},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_042), set_gamma_G_P_042},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_043), set_gamma_G_P_043},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_044), set_gamma_G_P_044},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_045), set_gamma_G_P_045},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_046), set_gamma_G_P_046},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_047), set_gamma_G_P_047},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_048), set_gamma_G_P_048},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_049), set_gamma_G_P_049},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_050), set_gamma_G_P_050},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_051), set_gamma_G_P_051},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_052), set_gamma_G_P_052},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_053), set_gamma_G_P_053},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_054), set_gamma_G_P_054},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_055), set_gamma_G_P_055},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_056), set_gamma_G_P_056},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_057), set_gamma_G_P_057},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_058), set_gamma_G_P_058},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_059), set_gamma_G_P_059},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_P_060), set_gamma_G_P_060},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_001), set_gamma_G_M_001},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_002), set_gamma_G_M_002},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_003), set_gamma_G_M_003},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_004), set_gamma_G_M_004},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_005), set_gamma_G_M_005},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_006), set_gamma_G_M_006},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_007), set_gamma_G_M_007},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_008), set_gamma_G_M_008},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_009), set_gamma_G_M_009},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_010), set_gamma_G_M_010},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_011), set_gamma_G_M_011},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_012), set_gamma_G_M_012},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_013), set_gamma_G_M_013},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_014), set_gamma_G_M_014},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_015), set_gamma_G_M_015},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_016), set_gamma_G_M_016},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_017), set_gamma_G_M_017},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_018), set_gamma_G_M_018},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_019), set_gamma_G_M_019},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_020), set_gamma_G_M_020},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_021), set_gamma_G_M_021},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_022), set_gamma_G_M_022},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_023), set_gamma_G_M_023},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_024), set_gamma_G_M_024},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_025), set_gamma_G_M_025},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_026), set_gamma_G_M_026},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_027), set_gamma_G_M_027},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_028), set_gamma_G_M_028},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_029), set_gamma_G_M_029},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_030), set_gamma_G_M_030},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_031), set_gamma_G_M_031},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_032), set_gamma_G_M_032},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_033), set_gamma_G_M_033},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_034), set_gamma_G_M_034},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_035), set_gamma_G_M_035},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_036), set_gamma_G_M_036},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_037), set_gamma_G_M_037},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_038), set_gamma_G_M_038},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_039), set_gamma_G_M_039},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_040), set_gamma_G_M_040},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_041), set_gamma_G_M_041},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_042), set_gamma_G_M_042},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_043), set_gamma_G_M_043},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_044), set_gamma_G_M_044},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_045), set_gamma_G_M_045},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_046), set_gamma_G_M_046},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_047), set_gamma_G_M_047},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_048), set_gamma_G_M_048},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_049), set_gamma_G_M_049},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_050), set_gamma_G_M_050},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_051), set_gamma_G_M_051},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_052), set_gamma_G_M_052},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_053), set_gamma_G_M_053},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_054), set_gamma_G_M_054},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_055), set_gamma_G_M_055},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_056), set_gamma_G_M_056},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_057), set_gamma_G_M_057},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_058), set_gamma_G_M_058},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_059), set_gamma_G_M_059},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_G_M_060), set_gamma_G_M_060},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_001), set_gamma_B_P_001},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_002), set_gamma_B_P_002},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_003), set_gamma_B_P_003},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_004), set_gamma_B_P_004},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_005), set_gamma_B_P_005},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_006), set_gamma_B_P_006},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_007), set_gamma_B_P_007},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_008), set_gamma_B_P_008},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_009), set_gamma_B_P_009},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_010), set_gamma_B_P_010},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_011), set_gamma_B_P_011},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_012), set_gamma_B_P_012},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_013), set_gamma_B_P_013},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_014), set_gamma_B_P_014},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_015), set_gamma_B_P_015},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_016), set_gamma_B_P_016},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_017), set_gamma_B_P_017},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_018), set_gamma_B_P_018},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_019), set_gamma_B_P_019},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_020), set_gamma_B_P_020},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_021), set_gamma_B_P_021},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_022), set_gamma_B_P_022},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_023), set_gamma_B_P_023},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_024), set_gamma_B_P_024},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_025), set_gamma_B_P_025},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_026), set_gamma_B_P_026},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_027), set_gamma_B_P_027},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_028), set_gamma_B_P_028},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_029), set_gamma_B_P_029},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_030), set_gamma_B_P_030},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_031), set_gamma_B_P_031},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_032), set_gamma_B_P_032},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_033), set_gamma_B_P_033},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_034), set_gamma_B_P_034},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_035), set_gamma_B_P_035},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_036), set_gamma_B_P_036},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_037), set_gamma_B_P_037},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_038), set_gamma_B_P_038},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_039), set_gamma_B_P_039},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_040), set_gamma_B_P_040},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_041), set_gamma_B_P_041},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_042), set_gamma_B_P_042},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_043), set_gamma_B_P_043},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_044), set_gamma_B_P_044},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_045), set_gamma_B_P_045},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_046), set_gamma_B_P_046},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_047), set_gamma_B_P_047},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_048), set_gamma_B_P_048},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_049), set_gamma_B_P_049},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_050), set_gamma_B_P_050},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_051), set_gamma_B_P_051},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_052), set_gamma_B_P_052},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_053), set_gamma_B_P_053},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_054), set_gamma_B_P_054},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_055), set_gamma_B_P_055},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_056), set_gamma_B_P_056},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_057), set_gamma_B_P_057},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_058), set_gamma_B_P_058},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_059), set_gamma_B_P_059},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_P_060), set_gamma_B_P_060},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_001), set_gamma_B_M_001},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_002), set_gamma_B_M_002},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_003), set_gamma_B_M_003},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_004), set_gamma_B_M_004},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_005), set_gamma_B_M_005},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_006), set_gamma_B_M_006},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_007), set_gamma_B_M_007},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_008), set_gamma_B_M_008},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_009), set_gamma_B_M_009},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_010), set_gamma_B_M_010},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_011), set_gamma_B_M_011},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_012), set_gamma_B_M_012},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_013), set_gamma_B_M_013},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_014), set_gamma_B_M_014},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_015), set_gamma_B_M_015},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_016), set_gamma_B_M_016},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_017), set_gamma_B_M_017},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_018), set_gamma_B_M_018},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_019), set_gamma_B_M_019},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_020), set_gamma_B_M_020},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_021), set_gamma_B_M_021},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_022), set_gamma_B_M_022},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_023), set_gamma_B_M_023},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_024), set_gamma_B_M_024},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_025), set_gamma_B_M_025},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_026), set_gamma_B_M_026},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_027), set_gamma_B_M_027},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_028), set_gamma_B_M_028},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_029), set_gamma_B_M_029},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_030), set_gamma_B_M_030},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_031), set_gamma_B_M_031},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_032), set_gamma_B_M_032},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_033), set_gamma_B_M_033},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_034), set_gamma_B_M_034},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_035), set_gamma_B_M_035},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_036), set_gamma_B_M_036},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_037), set_gamma_B_M_037},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_038), set_gamma_B_M_038},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_039), set_gamma_B_M_039},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_040), set_gamma_B_M_040},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_041), set_gamma_B_M_041},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_042), set_gamma_B_M_042},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_043), set_gamma_B_M_043},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_044), set_gamma_B_M_044},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_045), set_gamma_B_M_045},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_046), set_gamma_B_M_046},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_047), set_gamma_B_M_047},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_048), set_gamma_B_M_048},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_049), set_gamma_B_M_049},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_050), set_gamma_B_M_050},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_051), set_gamma_B_M_051},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_052), set_gamma_B_M_052},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_053), set_gamma_B_M_053},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_054), set_gamma_B_M_054},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_055), set_gamma_B_M_055},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_056), set_gamma_B_M_056},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_057), set_gamma_B_M_057},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_058), set_gamma_B_M_058},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_059), set_gamma_B_M_059},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(set_gamma_B_M_060), set_gamma_B_M_060},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(cmd1_select), cmd1_select},
};

static struct dsi_cmd_desc novatek_wxga_sleep_out_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(exit_sleep), exit_sleep},
};
static struct dsi_cmd_desc novatek_wxga_cmd1_set_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(cmd1_select), cmd1_select},
};

/* display direction */
static struct dsi_cmd_desc novatek_wxga_display_direction_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,  sizeof(display_direction_001), display_direction_001},
};

static struct dsi_cmd_desc novatek_wxga_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,   sizeof(display_on), display_on},
};

static struct dsi_cmd_desc novatek_wxga_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,   sizeof(display_off), display_off},
};
static struct dsi_cmd_desc novatek_wxga_display_off_cmds2[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(enter_sleep), enter_sleep},
};
#endif  /* MIPI_NOVATEK_WXGA_TBL_H */

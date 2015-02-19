/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/input/touchscreen/pixart.h
 *
 * Copyright (C) 2012 Pixart Imaging Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* Revision history
 * 21Apr2011 mikeb added AMRI-5200 constants
 * 20May2011 jp deleted redundant #define
 * 20May2011 jp adjusted values for VALUE_TOUCH_REF - 4000(dec)
 */


#include <linux/types.h>
#include <linux/earlysuspend.h>
#include <linux/cdev.h>
#include "pixart_config.h"

#ifndef _LINUX_PIXART_PT_H
#define _LINUX_PIXART_I2C_H

#define PIXART_I2C_NAME "pixart-pt"
#define PIXART_I2C_ADDR	0x6B	/* MOSI_A0 = Z   NCS_A1 = Z */

/* #define PIXART_I2C_ADDR	0x33 */		/* MOSI_A0 = 0 NCS_A1 = 0 */

/* AMRI-5000 and AMRI-5100 and AMRI-5305 (GEN1,2) common register definitions */
#define PIXART_REG_PID			0x00
#define PIXART_REG_REV_ID        0x01
#define  PIXART_5000_PID			 0x86
#define  PIXART_5100_PID			 0x87
#define  PIXART_5305_PAP1100QN_PID	 0x89
#define  PIXART_5311_PID			 0x8B
#define PIXART_REG_REVID	        0x01
#define PIXART_REG_SHUTDOWN      0x7A
#define  PIXART_SHUTDOWN          0xAA
#define  PIXART_RESET             0xBB
#define  PIXART_RESET_TO_ROM_GEN2 0xCC
#define  PIXART_RESUME            0xDD
#define PIXART_REG_WD_DISABLE    0x7D
#define  PIXART_WD_DISABLE        0xAD
#define  PIXART_WD_ENABLE         0x00

/* AMRI-5000 and AMRI-5100 (GEN1) register definitions */
#define PIXART_REG_STATUS  			    0x02
#define  PIXART_REG_STATUS_ERROR			 0x01
#define  PIXART_REG_STATUS_TOUCH_DETECT	 0x02
#define  PIXART_REG_STATUS_STATUS_CHANGE	 0x04
#define PIXART_REG_ERROR_ID_GEN1          0x0B
#define  PIXART_REG_STATUS_MOTION_READY	 0x10
#define PIXART_REG_IODL_CTL              0x08
#define  PIXART_ENABLE_DL                 0x2b
#define PIXART_REG_IODL_DATA             0x09
#define PIXART_REG_ORIENTATION           0x0c
#define PIXART_REG_BOOT_STAT             0x0e
#define PIXART_REG_CAL_CONTROL			0x0f
#define PIXART_REG_POINTS			    0x48

#define  PIXART_REG_STATUS_ERROR			 0x01
#define  PIXART_REG_STATUS_TOUCH_DETECT	 0x02
#define  PIXART_REG_STATUS_STATUS_CHANGE	 0x04
#define  PIXART_REG_STATUS_MOTION_READY	 0x10

#define PIXART_REG_HEIGHT_HI              0x43
#define PIXART_REG_HEGHT_LO               0x44
#define PIXART_REG_WIDTH_HI               0x45
#define PIXART_REG_WIDTH_LO               0x46
#define PIXART_REG_POINTS			    0x48
#define PIXART_REG_TOUCH_1_HIGH			 0x5A
#define PIXART_REG_TOUCH_1_LO			 0x5B
#define PIXART_REG_TOUCH_2_HIGH		     0x5C
#define PIXART_REG_TOUCH_2_LO			 0x5D
#define PIXART_REG_TOUCH_3_HIGH			 0x5E
#define PIXART_REG_TOUCH_3_LO			 0x5F
#define  PIXART_VALUE_TOUCH_1_HIGH		  0x00
#define  PIXART_VALUE_TOUCH_1_LO			  0xC8
#define  PIXART_VALUE_TOUCH_2_HIGH		  0x01
#define  PIXART_VALUE_TOUCH_2_LO			  0x00
#define  PIXART_VALUE_TOUCH_3_HIGH		  0x01
#define  PIXART_VALUE_TOUCH_3_LO			  0x2C
#define  PIXART_VALUE_DS_MAP_DL			  0x2C
#define PIXART_REG_MOTION_ACCESS		    0x7b


/* AMRI-5305 (GEN2) register definitions */
#define PIXART_REG_FW_REVID_GEN2    	     0x02
#define PIXART_REG_BOOT_STAT_GEN2		 0x03
#define PIXART_REG_STATUS_GEN2			 0x04
#define  PIXART_REG_STATUS_GEN2_ERROR	       0x01
#define  PIXART_REG_STATUS_GEN2_TOUCH           0x02
#define  PIXART_REG_STATUS_GEN2_TOUCH_CHANGE    0x04
#define  PIXART_REG_STATUS_GEN2_THUMB           0x08
#define  PIXART_REG_STATUS_GEN2_DATA_READY	   0x10
#define  PIXART_REG_STATUS_GEN2_CHEEK           0x20
#define  PIXART_REG_STATUS_GEN2_IOTYPE          0x40
#define  PIXART_REG_STATUS_GEN2_WATCHDOG        0x80
#define PIXART_REG_INTMASK_GEN2			 0x05

#define PIXART_REG_STATUS_AUTO_CLEAR_GEN2 0x06
#define PIXART_REG_INT_DEASSERT_GEN2      0x07
#define PIXART_REG_EXTENDED_REG_GEN2      0x08
#define PIXART_REG_EXTENDED_VAL_GEN2      0x09
#define PIXART_REG_IODL_CTL_GEN2          0x0A
#define  PIXART_ENABLE_DL_GEN2             0x2e
#define PIXART_REG_IODL_DATA_GEN2         0x0B
#define PIXART_REG_TWI_ADDR_GEN2          0x0C
#define PIXART_REG_ERROR_ID_GEN2          0x0D
#define PIXART_REG_ORIENTATION_GEN2       0x0E
#define PIXART_REG_CAL_CONTROL_GEN2       0x0F
#define PIXART_REG_CHEEK_PERCENTAGE_GEN2  0x1A
#define PIXART_REG_CHEEK_TOTAL_CELLS_GEN2 0x1B
#define PIXART_REG_CHEEK_DELAY_GEN2       0x1C
#define PIXART_REG_CHEEK_PROLONG_GEN2     0x1D
#define PIXART_REG_MOTION_REPORT_CTL_GEN2 0x1E
#define  PIXART_MOTION_MARK_READ_GEN2     0x80
#define  PIXART_MOTION_DISABLE_TOUCH_GEN2 0x08
#define  PIXART_MOTION_DISABLE_HOVER_GEN2 0x04
#define  PIXART_MOTION_SECONDARY_SORT_GEN2 0x02
#define  PIXART_MOTION_SORT_BY_FORCE_GEN2 0x01
#define PIXART_REG_MOTION_REPORT_DATA_GEN2 0x1F
#define PIXART_REG_FORCE_RUN_MODE_GEN2    0x20
#define PIXART_REG_OBSERVED_RUN_MODE_GEN2 0x21
#define PIXART_REG_REPORT_RATE_GEN2       0x22
#define PIXART_REG_OBSERVED_REPORT_RATE_GEN2 0x23
#define PIXART_REG_IDLE_DS_LO_GEN2        0x24
#define PIXART_REG_IDLE_DS_HI_GEN2        0x25
#define PIXART_REG_IDLE_FRAME_RATE_GEN2   0x26
#define PIXART_REG_FLASH_CTL_GEN2         0x27
#define PIXART_REG_FLASH_SAVE_CUST_REGS   0x82
#define PIXART_REG_FLASH_SAVE_CUST_REGS_DS_MAP   0x86
#define PIXART_REG_FLASH_SAVE_DS_MAP   0x84
#define PIXART_REG_FLASH_NO_SAVE_CUST_REGS 0x02

#define PIXART_REG_REST1_DS_LO_GEN2       0x28
#define PIXART_REG_REST1_DS_HI_GEN2       0x29
#define PIXART_REG_REST1_FRAME_RATE_GEN2  0x2A
#define PIXART_REG_REST2_DS_LO_GEN2       0x2C
#define PIXART_REG_REST2_DS_HI_GEN2       0x2D
#define PIXART_REG_REST2_FRAME_RATE_GEN2  0x2E
#define PIXART_REG_NOISE_PROLONG_GEN2     0x2F
#define PIXART_REG_CHECHK_FW_CRC		0x30
#define PIXART_REG_CRC_HI_GEN2			 0x32
#define PIXART_REG_CRC_LO_GEN2			 0x33
#define PIXART_REG_OPEN_SHORT_SELECT	 0x32
#define PIXART_REG_OPEN_SHORT_RESULT	 0x33
#define PIXART_REG_HB_COUNT_GEN2          0x35
#define PIXART_REG_OSC_CTL_GEN2           0x37
#define PIXART_REG_ONTOUCH_REPORT_RATE    0x39
#define PIXART_REG_ROW_GEN2               0x41
#define PIXART_REG_COL_GEN2               0x42
#define PIXART_REG_TOUCH_ID_GEN2          0x43
#define PIXART_REG_HEIGHT_LO_GEN2         0x44
#define PIXART_REG_HEGHT_HI_GEN2          0x45
#define PIXART_REG_WIDTH_LO_GEN2          0x46
#define PIXART_REG_WIDTH_HI_GEN2          0x47
#define PIXART_REG_PIXEL_CONFIG			0x49
#define PIXART_REG_DEAD_ZONE_RAD_LO_GEN2  0x4A
#define PIXART_REG_DEAD_ZONE_RAD_HI_GEN2  0x4B
#define PIXART_REG_PIXEL_DATA_LO		0x4C
#define PIXART_REG_PIXEL_DATA_HI		0x4D
#define PIXART_REG_FLASH_CRC_CTL_GEN2	 0x4E
#define PIXART_REG_REPORT_POINTS_GEN2     0x4F
#define PIXART_REG_XOFFSET_LO_GEN2        0x50
#define PIXART_REG_XOFFSET_HI_GEN2        0x51
#define PIXART_REG_YOFFSET_LO_GEN2        0x52
#define PIXART_REG_YOFFSET_HI_GEN2        0x53
#define PIXART_REG_WIN_ORG_X_GEN2         0x54
#define PIXART_REG_WN_ORG_Y_GEN2          0x55
#define PIXART_REG_WIN_MAX_X_GEN2         0x56
#define PIXART_REG_WIN_MAX_Y_GEN2         0x57
#define PIXART_REG_NAV_CONFIG_GEN2        0x58
#define PIXART_REG_NAV_CONFIG_2_GEN2      0x59
#define PIXART_REG_THRESH0_PCT_GEN2       0x5A
#define PIXART_REG_THRESH1_PCT_GEN2       0x5B
#define PIXART_REG_THRESH2_PCT_GEN2       0x5C
#define PIXART_REG_THRESH3_PCT_GEN2       0x5D
#define PIXART_REG_THRESH4_PCT_GEN2       0x5E
#define PIXART_REG_THRESH5_PCT_GEN2       0x5F
#define  PIXART_VALUE_THRESH0_PCT_GEN2     0x17
#define  PIXART_VALUE_THRESH1_PCT_GEN2     0x18
#define  PIXART_VALUE_THRESH2_PCT_GEN2     0x0c
#define  PIXART_VALUE_THRESH3_PCT_GEN2     0x19
#define  PIXART_VALUE_THRESH4_PCT_GEN2     0x32
#define  PIXART_VALUE_THRESH5_PCT_GEN2     0x64
#define PIXART_REG_TOUCH_REF_LO_GEN2      0x60
#define PIXART_REG_TOUCH_REF_HI_GEN2      0x61
#define  PIXART_VALUE_TOUCH_REF_LO_GEN2    0xA0
#define  PIXART_VALUE_TOUCH_REF_HI_GEN2    0x0F
#define PIXART_REG_MIN_REF_LO_GEN2        0x62
#define PIXART_REG_MIN_REF_HI_GEN2        0x63
#define  PIXART_VALUE_MIN_REF_LO_GEN2      0x90
#define  PIXART_VALUE_MIN_REF_HI_GEN2      0x01
#define PIXART_REG_CONT_THRESH_PCT_GEN2   0x66
#define PIXART_REG_TOUCH_DELAY_LONG_GEN2  0x67
#define PIXART_REG_TOUCH_DELAY_SHORT_GEN2 0x68
#define PIXART_REG_PROLONG_DELAY_GEN2     0x69
#define PIXART_REG_HOVER_PROLONG_DELAY_GEN2 0x6A
#define PIXART_REG_GESTURE_SELECT_GEN2    0x6C
#define PIXART_REG_GEN_FW_CTL_GEN2        0x6D
#define PIXART_REG_NAV_FILTER_GEN2        0x75
#define PIXART_REG_PROLONG_FRAMES_GEN2    0x76
#define PIXART_REG_HAPTIC_GEN2            0x77
#define PIXART_REG_BUTTON_CONTROL_GEN2    0x78
#define PIXART_REG_FLASH_ENABLE           0x7C
#define PIXART_FLASH_ENABLE               0xFE

/* Extended register definitions */
#define PIXART_EX_NA_CHOOSE_ACTION_INDEX		0x17
#define PIXART_EX_NA_CHOOSE_NA				0x19

/* Noise Avoidance Extended Register index */
#define PIXART_NA_DISABLE				0x00
#define PIXART_NA_OPTIONS_MISC				0x04
#define PIXART_NA_DRIVE_FREQ_INIT			0x0C
#define PIXART_NOISE_THRESH				0x18
#define PIXART_K1LOG2					0x50
#define PIXART_K2LOG2					0x52
#define PIXART_MAX_CORR_DIST_PER_REPORT_LSB		0x57
#define PIXART_MAX_CORR_DIST_PER_REPORT_MSB		0x58
#define PIXART_MAX_CORR_DIST_PER_REPORT_LSB_CHARG	0x59
#define PIXART_MAX_CORR_DIST_PER_REPORT_MSB_CHARG	0x5A
#define PIXART_TOUCH_DELAY_T3				0x5B
#define PIXART_TOUCH_DELAY_T3_CHARG			0x5C
#define PIXART_TOUCH_DELAY_T4				0x5D
#define PIXART_TOUCH_DELAY_T4_CHARG			0x5E
#define PIXART_TOUCH_DELAY_T5				0x5F
#define PIXART_TOUCH_DELAY_T5_CHARG			0x60
#define PIXART_PROLONG_DELAY				0x61
#define PIXART_PROLONG_DELAY_CHARG			0x62
#define PIXART_NA_RESUME				0x64
#define PIXART_ALC_CONTROL_ENABLE_BITS			0x65
#define PIXART_ALC_THRESH_PCT				0x66
#define PIXART_ALC_THRESH_2				0x68
#define PIXART_ALC_THRESH_3				0x69
#define PIXART_ALC_FAST_ADAPT_RATE			0x6A
#define PIXART_ALC_VERY_FAST_ADAPT_RATE			0x6B

/* number of touch points to reports */
#define PIXART_MAX_FINGER					10

/* timeouts and delays */
#define PIXART_TIMEOUT_MS_FW_INIT_GEN1    	    80
#define PIXART_TIMEOUT_MS_FW_INIT_GEN2    	    2000
#define PIXART_TIMEOUT_MS_BOOT_GEN2    		    500
#define PIXART_DELAY_US_BETWEEN_FW_BURSTS_GEN2 	2000
#define PIXART_DELAY_US_BETWEEN_SPI_WRITES 	    15
#define PIXART_DELAY_US_BETWEEN_I2C_WRITES 	    15
#define PIXART_DELAY_US_BETWEEN_FLASH_WRITES 	    20
#define PIXART_DELAY_US_AFTER_FLASH_ENABLE_GEN2     30
#define PIXART_TIMEOUT_MS_AFTER_FLASH_ENABLE_GEN2   500
#define PIXART_DELAY_US_AFTER_BOOT_GEN2             100

/* I2C Retry */
#define PIXART_I2C_WAIT		25
#define PIXART_RW_RETRY		3

#define PIXART_NOTOUCH			0	/* lift off */
#define PIXART_TOUCH				1	/* touch down */
#define PIXART_TOUCH_ID_MIN		129
#define PIXART_TOUCH_ID_MAX		139

/* CRC Hi/Lo Byte for DS map block..may be different, depending on specific
	panel & fw vers */
#define CRC_HI_DS_MAP		0xCD
#define CRC_LO_DS_MAP		0x29

/* CRC Hi/Lo Byte for Customer Settings block..may be different, depending on
	specific panel & fw vers */
#define CRC_HI_CS_REGS		0x81
#define CRC_LO_CS_REGS		0xBB


#define PIXART_SET_TOUCH_THRESHOLD

/* Device dependent */
#define CTP_ROWS			12
#define CTP_COLS			20
#ifdef PANEL_47INCH_CONFIG
#define AMRI5K_FORCE_MAX	4000
#define AMRI5K_AREA_MAX		2000
#else
#define AMRI5K_FORCE_MAX	4000
#define AMRI5K_AREA_MAX		1500
#endif
#define Y_MAX				272
#define PIXART_DIAGONAL		85

/* Opens / Shorts Test */
#define MOTION_DATA_SIZE		92
#define MOTION_DATA_PAGE_MAX	40
#define MOTION_DATA_1BLOCK		10
#define PIXEL_DUMP				480

/* For SPI interface, PT_I2C is not set */
#ifdef CONFIG_TOUCHSCREEN_PIXART_I2C
#define PT_I2C
#endif

#define DO_PT_RESET

/* OMAP3530 GPIO175_SPI11_CS1, GPIO175 connects to NRST PIN to reset PT */
#define leswap(x) ( ((x << 8) & 0xff00) | ((x >> 8) & 0xff) )

#pragma pack(1)
struct pixart_point_data {
	uint8_t id;
	uint16_t x;
	uint16_t y;
	uint16_t force;
	uint16_t area;
};

struct pixart_slot {
	uint8_t id;
	bool tool_finger;
};

struct pixart_touch_data_gen {
	uint8_t status;
	uint8_t total_touch;
	struct pixart_point_data point_data[PIXART_MAX_FINGER];
	struct pixart_slot slot[PIXART_MAX_FINGER];
};
#pragma pack()

typedef enum touch_status {
    TOUCH_POWERON,
    TOUCH_POWEROFF,
    TOUCH_UPDATE
} touch_status;

/* sysfs operations */
enum {
	SYSFS_READ = 1,
	SYSFS_WRITE,
	SYSFS_BURST,
	SYSFS_DISABLE,
	SYSFS_ENABLE,
	SYSFS_READ3N = 6,
	SYSFS_VERSION,
	SYSFS_WRITE_READ,
	SYSFS_FIRMWARE,
	SYSFS_KDBGLEVEL,
	SYSFS_GPIO_CTRL = 11,
	SYSFS_READ2N,
	SYSFS_PIXEL_DUMP,
	SYSFS_STATUS,
	SYSFS_TOUCH_DATA,
	SYSFS_NA_READ,
};

#endif /*_LINUX_PIXART_PT_H */

struct pixart_platform_data {
	const u8 *config;
	size_t config_length;
	const u8 *na_config;
	size_t na_config_length;
	unsigned int x_size;
	unsigned int y_size;
	int irq_gpio;
	int reset_gpio;
	unsigned char orient;
	unsigned long irqflags;

	int (*init_hw) (void);
	int (*reset_hw) (void);
	int (*shutdown) (void);
};


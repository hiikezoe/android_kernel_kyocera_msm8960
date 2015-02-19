/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
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

//#define DISABLE_DISP_DETECT 1
//#define DISABLE_CABC_PWM 1

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h> 
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/wakelock.h>
#include <linux/leds-lm3533.h>
#include <linux/kcbms.h>
#ifndef DISABLE_CABC_PWM
#include <linux/disp_ext_blc.h>
#endif  /* DISABLE_CABC_PWM */

#define LED_DEBUG 0

#if LED_DEBUG
#define DEBUG_PRINT(arg...)	printk(KERN_INFO "[LEDDRV]:" arg)
#else
#define DEBUG_PRINT(arg...)
#endif

#define DEBUG_PRINT_ERR(arg...)	printk(KERN_ERR "[LEDDRV]:" arg)

#define LED_INDIVIDUAL_ON

#define LM3533_STATE_ENABLE(state) {					\
	DEBUG_PRINT("%s() enable_state = %d\n",__func__, state);		\
	DEBUG_PRINT("%s() before_status = %d\n",__func__, guc_light_status);		\
	guc_light_status |= state;					\
	DEBUG_PRINT("%s() after_status = %d\n",__func__, guc_light_status);		\
}

#define LM3533_STATE_DISABLE(state) {					\
	DEBUG_PRINT("%s() disable_state = %d\n",__func__, state);		\
	DEBUG_PRINT("%s() before_status = %d\n",__func__, guc_light_status);		\
	guc_light_status &= ~state;					\
	DEBUG_PRINT("%s() after_status = %d\n",__func__, guc_light_status);		\
}

enum light_index_enum{
	MOBILELIGHT_INDEX = 0,
	LEDLIGHT_INDEX,
	BACKLIGHT_INDEX,
	LIGHT_INDEX_MAX
};

#define LED_DRV_NAME			"LM3533"
#define LED_SLAVE_ADDR			(0x6C >> 1)
#define I2C_RETRIES_NUM 		5
#define LED_WRITE_BUF_LEN		2

#define MOBILELIGHT_INFO		"mobilelightinfo"
#define LED_INFO			"ledinfo"
#define BACKLIGHT_INFO			"backlightinfo"

#define MOBILELIGHT_MAX_BRIGHT_VAL	0xFFFFFFFFu
#define LEDLIGHT_MAX_BRIGHT_VAL		0xFFFFFFFFu
#define BACKLIGHT_MAX_BRIGHT_VAL	0xFFFFFFFFu

#define LM3533_RESET_GPIO		47

#define LM3533_REG_10			0x10
#define LM3533_REG_1A			0x1A
#define LM3533_REG_1B			0x1B
#define LM3533_REG_1C			0x1C
#define LM3533_REG_1D			0x1D
#define LM3533_REG_1E			0x1E
#define LM3533_REG_1F			0x1F
#define LM3533_REG_21			0x21
#define LM3533_REG_22			0x22
#define LM3533_REG_23			0x23
#define LM3533_REG_24			0x24
#define LM3533_REG_2C			0x2C
#define LM3533_REG_45			0x45

#define LM3533_REG_40			0x40
#define LM3533_REG_14			0x14
#define LM3533_REG_28			0x28
#define LM3533_REG_27			0x27
#define LM3533_REG_26			0x26
#define LM3533_REG_42			0x42
#define LM3533_REG_43			0x43
#define LM3533_REG_44			0x44

#define LM3533_REG_70			0x70
#define LM3533_REG_71			0x71
#define LM3533_REG_72			0x72
#define LM3533_REG_73			0x73
#define LM3533_REG_74			0x74
#define LM3533_REG_75			0x75

#define LM3533_REG_80			0x80
#define LM3533_REG_81			0x81
#define LM3533_REG_82			0x82
#define LM3533_REG_83			0x83
#define LM3533_REG_84			0x84
#define LM3533_REG_85			0x85

#define LM3533_REG_90			0x90
#define LM3533_REG_91			0x91
#define LM3533_REG_92			0x92
#define LM3533_REG_93			0x93
#define LM3533_REG_94			0x94
#define LM3533_REG_95			0x95

#define LM3533_REG10_INIT		0x92
#define LM3533_REG1A_INIT		0x02
#define LM3533_REG1B_INIT		0x04
#define LM3533_REG1C_INIT		0x04
#define LM3533_REG1D_INIT		0x04
#define LM3533_REG1E_INIT		0x04
#define LM3533_REG1F_INIT		0x12
#define LM3533_REG21_INIT		0x07
#define LM3533_REG22_INIT		0x07
#define LM3533_REG23_INIT		0x07
#define LM3533_REG24_INIT		0x13
#define LM3533_REG2C_INIT		0x0F
#define LM3533_REG45_INIT		0xFF

#define LM3533_REG28_OFF		0x00

#define LM3533_COL_BLACK		0x00000000

#define LM3533_LED_ENABLE		0x00
#define LM3533_LED_DISABLE		0x01

#define LM3533_LED_PWM_ENABLE		0x01
#define LM3533_LED_PWM_DISABLE		0x00

#define LM3533_LED_P_RED_ON_VAL		0x03
#define LM3533_LED_P_GREEN_ON_VAL	0x0C
#define LM3533_LED_P_BLUE_ON_VAL	0x30

#define LM3553_CURRENT_VALUE_MAX		0xFF
#define LM3553_CURRENT_VALUE_OFF		0x00
#define LM3553_CURRENT_VALUE_02     	0x02
#define LM3553_CURRENT_VALUE_03     	0x03
#define LM3553_CURRENT_VALUE_04     	0x04
#define LM3553_CURRENT_VALUE_08	    	0x08
#define LM3553_CURRENT_VALUE_0D     	0x0D
#define LM3553_CURRENT_VALUE_10	    	0x10
#define LM3553_CURRENT_VALUE_15     	0x15
#define LM3553_CURRENT_VALUE_1A	    	0x1A

#define LM3553_TEMP_COMPENSATING_CURVAL	0xFF
#define LM3553_THRESHOLD_TEMPVAL	0x3C
#define LM3553_THRESHOLD_CURVAL		0xFF
#define LM3553_CABC_TH_VAL		0x17

#define LM3533_DM_TEMPARTURE_HIGH	0x50
#define LM3533_DM_TEMPARTURE_LOW	0x0A

#define LM3533__BACKLIGHT_COLOR_OFF	0x00

#define ALL_OFF_STATE			0x00000000
#define BACK_LIGHT_STATE		0x00000001
#define LED_RED_STATE			0x00000004
#define LED_GREEN_STATE 		0x00000008
#define LED_BLUE_STATE			0x00000010
#define MOBILE_LIGHT_STATE		0x00000020
#define LED_RGB_STATE			(LED_RED_STATE|LED_GREEN_STATE|LED_BLUE_STATE)
#define LED_CHARGEPUMP_STATE		(LED_RGB_STATE|MOBILE_LIGHT_STATE)

#define LEDLIGHT_BLINK_NUM		(4)
#define LEDLIGHT_BLINK_NUM_DM		(1)
#define LEDLIGHT			'L'
#define LEDLIGHT_SET_BLINK		_IOW(LEDLIGHT, 0, T_LEDLIGHT_IOCTL)
#define LEDLIGHT_SET_TEMPERTURE_DM	_IOW(LEDLIGHT, 1, T_LEDLIGHT_IOCTL_DM)

#define LED_GPIO_HIGH_VAL		1
#define LED_GPIO_LOW_VAL		0

#define MOBILELIGHT_OFF			0x00
#define MOBILELIGHT_BATTTERM_LOW_VAL	0x13
#define MOBILELIGHT_BATTTERM_HIGH_VAL	0x07
#define MOBILELIGHT_THRESHOLD_VAL	0x41

#define LED_PT_RED_STATE		0x01
#define LED_PT_GREEN_STATE		0x04
#define LED_PT_BLUE_STATE		0x10

#define LED_COLOR_PATTERN_RGB		3

#define LM3533_LED_RESET_ON		0x01
#define LM3533_LED_RESET_OFF		0x00

enum blink_control_enum{
	NO_BLINK_REQUEST = 0,
	BLINK_REQUEST,
};

enum brightness_control_enum{
	BRIGHTNESS_USER = 0,
	BRIGHTNESS_SENSOR,
	BRIGHTNESS_LCD,
	BRIGHTNESS_MAX
};

enum color_pattern_enum{
	COLOR_PATTERN_1 = 0,
	COLOR_PATTERN_2,
	COLOR_PATTERN_3,
	COLOR_PATTERN_4,
	COLOR_PATTERN_5,
	COLOR_PATTERN_6,
	COLOR_PATTERN_7,
	COLOR_PATTERN_MAX
};

static uint32_t const gul_pattern_val[COLOR_PATTERN_MAX][LED_COLOR_PATTERN_RGB] = {
	{LM3553_CURRENT_VALUE_OFF     ,LM3553_CURRENT_VALUE_OFF       ,LM3553_CURRENT_VALUE_1A       },
	{LM3553_CURRENT_VALUE_10      ,LM3553_CURRENT_VALUE_OFF       ,LM3553_CURRENT_VALUE_OFF      },
	{LM3553_CURRENT_VALUE_OFF     ,LM3553_CURRENT_VALUE_08        ,LM3553_CURRENT_VALUE_OFF      },
	{LM3553_CURRENT_VALUE_08      ,LM3553_CURRENT_VALUE_OFF       ,LM3553_CURRENT_VALUE_0D       },
	{LM3553_CURRENT_VALUE_OFF     ,LM3553_CURRENT_VALUE_04        ,LM3553_CURRENT_VALUE_0D       },
	{LM3553_CURRENT_VALUE_15      ,LM3553_CURRENT_VALUE_03        ,LM3553_CURRENT_VALUE_OFF      },
	{LM3553_CURRENT_VALUE_0D      ,LM3553_CURRENT_VALUE_02        ,LM3553_CURRENT_VALUE_02       }
};

static uint32_t const gul_color_state[LED_COLOR_PATTERN_RGB] = {
	LED_RED_STATE,
	LED_GREEN_STATE,
	LED_BLUE_STATE
};

static uint32_t const gul_blink_pattern_onoff_val[COLOR_PATTERN_MAX][LED_COLOR_PATTERN_RGB] = {
	{ALL_OFF_STATE           ,ALL_OFF_STATE             ,LM3533_LED_P_BLUE_ON_VAL},
	{LM3533_LED_P_RED_ON_VAL ,ALL_OFF_STATE             ,ALL_OFF_STATE           },
	{ALL_OFF_STATE           ,LM3533_LED_P_GREEN_ON_VAL ,ALL_OFF_STATE           },
	{LM3533_LED_P_RED_ON_VAL ,ALL_OFF_STATE             ,LM3533_LED_P_BLUE_ON_VAL},
	{ALL_OFF_STATE           ,LM3533_LED_P_GREEN_ON_VAL ,LM3533_LED_P_BLUE_ON_VAL},
	{LM3533_LED_P_RED_ON_VAL ,LM3533_LED_P_GREEN_ON_VAL ,ALL_OFF_STATE           },
	{LM3533_LED_P_RED_ON_VAL ,LM3533_LED_P_GREEN_ON_VAL ,LM3533_LED_P_BLUE_ON_VAL}
};

struct light_led_data_type {
	struct led_classdev	st_cdev;
	struct i2c_client	*pst_client;
	uint32_t		ul_value;
	struct work_struct	work;
	struct mutex		lock;
	spinlock_t		value_lock;
	uint8_t 		uc_indled_val;
	struct wake_lock	work_wake_lock;
	uint32_t		blink_control; 
	uint32_t		blink_on_delay;
	uint32_t		blink_off_delay;
	uint32_t		blink_off_color;
	uint8_t			uc_led_red_val;
	uint8_t			uc_led_green_val;
	uint8_t			uc_led_blue_val;
};


static uint8_t guc_light_status = 0;
static uint8_t guc_cabc_keep_val = 0;
static uint8_t guc_chargepump_keep_val = 0xff;

static uint32_t guc_light_dm = 0;

static uint8_t guc_backlight_save_val = 0;
static uint32_t gul_value = 0;

static struct light_led_data_type *gpst_light_led_data = NULL;

static uint32_t const gul_pattern_value[COLOR_PATTERN_MAX] = {
	0x000000FF,
	0x00FF0000,
	0x0000FF00,
	0x00FF00FF,
	0x0000FFFF,
	0x00FFFF00,
	0x00C0C0C0
};

static atomic_t g_reset_err_status = ATOMIC_INIT(0);
static atomic_t g_cabc_status = ATOMIC_INIT(0);
#ifndef DISABLE_DISP_DETECT
static atomic_t g_disp_status = ATOMIC_INIT(0);
static atomic_t g_display_detect = ATOMIC_INIT(0);
static struct mutex led_disp_lock;
#endif  /* DISABLE_DISP_DETECT */
static struct mutex cabc_lock;

typedef struct _t_ledlight_ioctl {
	uint32_t data[LEDLIGHT_BLINK_NUM];
}T_LEDLIGHT_IOCTL;

typedef struct _t_ledlight_ioctl_dm {
	uint32_t dm_data;
}T_LEDLIGHT_IOCTL_DM;

static int32_t light_led_err_reset(struct light_led_data_type *pst_light_led_data);

static int32_t lm3533_i2c_write (struct i2c_client *pst_client, uint8_t uc_reg, uint8_t uc_val)
{
	int32_t lret = 0;
	uint32_t ul_tryn = 0;
	struct light_led_data_type *pst_light_led_data;
	u8 ucwritebuf[LED_WRITE_BUF_LEN];
	uint8_t uc_len;
	struct i2c_msg msg;

	pst_light_led_data = i2c_get_clientdata(pst_client);

	DEBUG_PRINT("%s() start uc_reg=0x%02x uc_val=0x%02x\n",
							__func__,
							uc_reg,
							uc_val);

	DEBUG_PRINT("%s() start pst_client=0x%08x pst_light_led_data=0x%08x\n",
					__func__, 
					(unsigned int)pst_client ,
					(unsigned int)pst_light_led_data);

	ucwritebuf[0] = uc_reg;
	ucwritebuf[1] = uc_val;
	uc_len    = sizeof(ucwritebuf);
	msg.addr  = pst_light_led_data->pst_client->addr;
	msg.flags = 0;
	msg.len   = uc_len;
	msg.buf   = &ucwritebuf[0];

	do {
		DEBUG_PRINT("%s() i2c_transfer() call\n", __func__);
		lret = i2c_transfer(pst_light_led_data->pst_client->adapter, &msg, 1);
		DEBUG_PRINT("%s() i2c_transfer() call end lret=%d\n",
							__func__,
							lret);
	} while ((lret != 1) && (++ul_tryn < I2C_RETRIES_NUM));

	if (lret != 1) {
		DEBUG_PRINT_ERR("%s(): uc_reg 0x%02x, uc_val 0x%02x lret %d\n",
								__func__,
								ucwritebuf[0],
								ucwritebuf[1],
								lret);
		lret = -1;
	}else
		lret = 0;

	DEBUG_PRINT("%s() end\n", __func__);
	return lret;
}

static int32_t lm3533_i2c_read_byte (struct i2c_client *pst_client, uint8_t uc_reg, uint8_t *buf)
{
	int32_t lret = 0;
	uint32_t ul_tryn = 0;
	struct light_led_data_type *pst_light_led_data;
	struct i2c_msg msg[2];
	uint8_t reg;

	pst_light_led_data = i2c_get_clientdata(pst_client);
	reg = uc_reg;

	DEBUG_PRINT("%s() start uc_reg=0x%02x \n",__func__,uc_reg);

	DEBUG_PRINT("%s() start pst_client=0x%08x pst_light_led_data=0x%08x\n",
					__func__, 
					(unsigned int)pst_client ,
					(unsigned int)pst_light_led_data);

	msg[0].addr = pst_light_led_data->pst_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = pst_light_led_data->pst_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;


	do {
		DEBUG_PRINT("%s() i2c_transfer() call\n", __func__);
		lret = i2c_transfer(pst_light_led_data->pst_client->adapter, &msg[0], 2);
		DEBUG_PRINT("%s() i2c_transfer() call end lret=%d\n",
							__func__,
							lret);
	} while ((lret != 2) && (++ul_tryn < I2C_RETRIES_NUM));

	if (lret != 2) {
		DEBUG_PRINT_ERR("%s(): uc_reg 0x%02x, buf 0x%02x lret %d\n",
								__func__,
								uc_reg,
								*buf,
								lret);
		lret = -1;
	}else
		lret = 0;

	DEBUG_PRINT("%s() end\n", __func__);
	return lret;
}

static int32_t lm3533_color_pattern_check(uint32_t ul_colorval)
{
	int32_t lret = COLOR_PATTERN_MAX;
	int32_t lmatch = 0;

	DEBUG_PRINT("%s() start \n", __func__);

	for (lmatch=0; lmatch<COLOR_PATTERN_MAX; lmatch++) {
		if (ul_colorval == gul_pattern_value[lmatch]) {
			DEBUG_PRINT("%s() color pattern match %d \n",__func__, lmatch);
			lret = lmatch;
			break;
		}
	}

	DEBUG_PRINT("%s() end\n", __func__);
	return lret;
}


static int32_t lm3533_set_chargepump(struct i2c_client *pst_client, uint8_t enable)
{
	int32_t lret = 0;
	DEBUG_PRINT("%s() start \n", __func__);
	DEBUG_PRINT("%s() guc_light_status=%d guc_chargepump_keep_val=%d enable=%d\n", __func__,guc_light_status,guc_chargepump_keep_val,enable);

	if (((guc_light_status & LED_CHARGEPUMP_STATE) == 0) &&
	     guc_chargepump_keep_val != enable ) {
		lret = lm3533_i2c_write(pst_client, LM3533_REG_26 , enable);
		guc_chargepump_keep_val = enable;
		DEBUG_PRINT("%s() lret=%d \n", __func__,lret);
	}

	DEBUG_PRINT("%s() end\n", __func__);
	return lret;
}

static int32_t lm3533_set_cabc(struct light_led_data_type *pst_light_led_data, uint8_t enable)
{
	int32_t lret = 0;
	DEBUG_PRINT("%s() start enable = %d keepval = %d\n",__func__,enable,guc_cabc_keep_val);

	if (guc_cabc_keep_val != enable) {
		lret = lm3533_i2c_write(pst_light_led_data->pst_client,
					LM3533_REG_14,
					enable);
		guc_cabc_keep_val = enable;
	}
	DEBUG_PRINT("%s() end\n", __func__);

	return lret;
}

static int32_t lm3533_set_delay(struct light_led_data_type *pst_light_led_data, uint8_t reg, uint8_t delay)
{
	int32_t lret = 0;
	DEBUG_PRINT("%s() start reg = %d delay = %d\n", __func__,reg,delay);
	lret = lm3533_i2c_write(pst_light_led_data->pst_client,
				reg,
				delay);

	if (lret != 0)
		DEBUG_PRINT_ERR("%s() failed to delay set \n", __func__);

	DEBUG_PRINT("%s() end\n", __func__);

	return lret;
}

static int32_t lm3533_set_tlow(struct light_led_data_type *pst_light_led_data, uint8_t reg, uint8_t tlow)
{
	int32_t lret = 0;
	DEBUG_PRINT("%s() start reg = %d tlow = %d\n", __func__,reg,tlow);
	lret = lm3533_i2c_write(pst_light_led_data->pst_client,
				reg,
				tlow);

	if (lret != 0)
		DEBUG_PRINT_ERR("%s() failed to off set(ChgR)\n", __func__);

	DEBUG_PRINT("%s() end\n", __func__);

	return lret;
}

static int32_t lm3533_set_ton(struct light_led_data_type *pst_light_led_data, uint8_t reg, uint8_t ton)
{
	int32_t lret = 0;
	 DEBUG_PRINT("%s() start reg = %d ton = %d\n", __func__,reg,ton);
	lret = lm3533_i2c_write(pst_light_led_data->pst_client,
				reg,
				ton);

	if (lret != 0)
		DEBUG_PRINT_ERR("%s() failed to on set(ChgR)\n", __func__);

	DEBUG_PRINT("%s() end\n", __func__);

	return lret;
}

static int32_t lm3533_set_curval(struct light_led_data_type *pst_light_led_data)
{
	int32_t lret = 0;
	lret = lm3533_i2c_write(pst_light_led_data->pst_client,
				LM3533_REG_42 ,
				pst_light_led_data->uc_led_red_val);
	if (lret != 0)
		DEBUG_PRINT_ERR("%s() failed to red val set\n", __func__);

	lret = lm3533_i2c_write(pst_light_led_data->pst_client,
				LM3533_REG_43 ,
				pst_light_led_data->uc_led_green_val);
	if (lret != 0)
		DEBUG_PRINT_ERR("%s() failed to green val set\n", __func__);

	lret = lm3533_i2c_write(pst_light_led_data->pst_client,
				LM3533_REG_44 ,
				pst_light_led_data->uc_led_blue_val);
	if (lret != 0)
		DEBUG_PRINT_ERR("%s() failed to bule val set\n", __func__);

	return lret;
}

static int32_t lm3533_blink_request(struct light_led_data_type *pst_light_led_data)
{
	uint32_t ul_pattern_state = 0;
	uint32_t ul_color_state =0;
	uint32_t lret;
	uint32_t i;
	uint32_t off_time = gpst_light_led_data[LEDLIGHT_INDEX].blink_off_delay;
	uint32_t on_time = gpst_light_led_data[LEDLIGHT_INDEX].blink_on_delay;
	uint32_t delay_time = gpst_light_led_data[LEDLIGHT_INDEX].blink_on_delay;

	DEBUG_PRINT("%s() delay_time = %x delay_on = %x delay_off = %x \n",
					__func__,
					delay_time,
					on_time,
					off_time);
	DEBUG_PRINT("%s() red = %x green = %x blue = %x \n",
					__func__,
					pst_light_led_data->uc_led_red_val,
					pst_light_led_data->uc_led_green_val,
					pst_light_led_data->uc_led_blue_val);


	if (gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color != LM3533_COL_BLACK) {
		if (gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color & 0x00FF0000) {
			lret = lm3533_set_delay(pst_light_led_data,LM3533_REG_70,delay_time);
			lret |= lm3533_set_tlow(pst_light_led_data,LM3533_REG_71,on_time);
			lret |= lm3533_set_ton(pst_light_led_data,LM3533_REG_72,off_time);
			if (lret != 0) {
				DEBUG_PRINT_ERR("%s() failed to patten param\n", __func__);
				goto fail_i2c_check;
			}
			ul_color_state |= LED_RED_STATE;
			ul_pattern_state |= LED_PT_RED_STATE;
		}
		
		if (gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color & 0x0000FF00) {
			lret = lm3533_set_delay(pst_light_led_data,LM3533_REG_80,delay_time);
			lret |= lm3533_set_tlow(pst_light_led_data,LM3533_REG_81,on_time);
			lret |= lm3533_set_ton(pst_light_led_data,LM3533_REG_82,off_time);
			if (lret != 0) {
				DEBUG_PRINT_ERR("%s() failed to patten param\n", __func__);
				goto fail_i2c_check;
			}
			ul_color_state |= LED_GREEN_STATE;
			ul_pattern_state |= LED_PT_GREEN_STATE;
		}

		if (gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color & 0x000000FF) {
			lret = lm3533_set_delay(pst_light_led_data,LM3533_REG_90,delay_time);
			lret |= lm3533_set_tlow(pst_light_led_data,LM3533_REG_91,on_time);
			lret |= lm3533_set_ton(pst_light_led_data,LM3533_REG_92,off_time);
			if (lret != 0) {
				DEBUG_PRINT_ERR("%s() failed to patten param\n", __func__);
				goto fail_i2c_check;
			}
			ul_color_state |= LED_BLUE_STATE;
			ul_pattern_state |= LED_PT_BLUE_STATE;
		}
	}

	if (pst_light_led_data->uc_led_red_val != LM3553_CURRENT_VALUE_OFF) {
		if ((ul_color_state & LED_RED_STATE) == 0) {
			ul_pattern_state |= LED_PT_RED_STATE;
			ul_color_state |= LED_RED_STATE;
			lret = lm3533_set_delay(pst_light_led_data,LM3533_REG_70,0);
			lret |= lm3533_set_tlow(pst_light_led_data,LM3533_REG_71,off_time);
			lret |= lm3533_set_ton(pst_light_led_data,LM3533_REG_72,on_time);
			if (lret != 0) {
				DEBUG_PRINT_ERR("%s() failed to patten param\n", __func__);
				goto fail_i2c_check;
			}
		}
		else {
			ul_pattern_state &= ~LED_PT_RED_STATE;
			if (on_time < off_time) {
				DEBUG_PRINT_ERR("%s() on_time < off_time red \n", __func__);
				pst_light_led_data->uc_led_red_val = (u8)((gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color>>(16)) & 0xff);
			}
		}
	}
	else {
		pst_light_led_data->uc_led_red_val = (u8)((gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color>>(16)) & 0xff);
	}

	if (pst_light_led_data->uc_led_green_val != LM3553_CURRENT_VALUE_OFF) {
		if ((ul_color_state & LED_GREEN_STATE) == 0) {
			ul_pattern_state |= LED_PT_GREEN_STATE;
			ul_color_state |= LED_GREEN_STATE;
			lret = lm3533_set_delay(pst_light_led_data,LM3533_REG_80,0);
			lret |= lm3533_set_tlow(pst_light_led_data,LM3533_REG_81,off_time);
			lret |= lm3533_set_ton(pst_light_led_data,LM3533_REG_82,on_time);
			if (lret != 0) {
				DEBUG_PRINT_ERR("%s() failed to patten param\n", __func__);
				goto fail_i2c_check;
			}
		}
		else {
			ul_pattern_state &= ~LED_PT_GREEN_STATE;
			if (on_time < off_time) {
				DEBUG_PRINT_ERR("%s() on_time < off_time green \n", __func__);
				pst_light_led_data->uc_led_green_val = (u8)((gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color>>(8)) & 0xff);
			}
		}
	}
	else {
		pst_light_led_data->uc_led_green_val = (u8)((gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color>>(8)) & 0xff);
	}

	if (pst_light_led_data->uc_led_blue_val != LM3553_CURRENT_VALUE_OFF) {
		if ((ul_color_state & LED_BLUE_STATE) == 0) {
			ul_pattern_state |= LED_PT_BLUE_STATE;
			ul_color_state |= LED_BLUE_STATE;
			lret = lm3533_set_delay(pst_light_led_data,LM3533_REG_90,0);
			lret |= lm3533_set_tlow(pst_light_led_data,LM3533_REG_91,off_time);
			lret |= lm3533_set_ton(pst_light_led_data,LM3533_REG_92,on_time);
			if (lret != 0) {
				DEBUG_PRINT_ERR("%s() failed to patten param\n", __func__);
				goto fail_i2c_check;
			}
		}
		else {
			ul_pattern_state &= ~LED_PT_BLUE_STATE;
			if (on_time < off_time) {
				DEBUG_PRINT_ERR("%s() on_time < off_time blue \n", __func__);
				pst_light_led_data->uc_led_blue_val = (u8)((gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color) & 0xff);
			}
		}
	}
	else {
		pst_light_led_data->uc_led_blue_val = (u8)((gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color) & 0xff);
	}

	lret = lm3533_i2c_write(pst_light_led_data->pst_client,
				LM3533_REG_28,
				ul_pattern_state);

	if (lret != 0) {
		DEBUG_PRINT_ERR("%s() failed to pattern set\n", __func__);
		goto fail_i2c_check;
	}

	lret = lm3533_set_curval(pst_light_led_data);
	if (lret != 0) {
		DEBUG_PRINT_ERR("%s() failed to curval reg set\n", __func__);
		goto fail_i2c_check;
	}

	for (i = 0 ;i > 2; i++ ) {
		lret |= lm3533_i2c_write(pst_light_led_data->pst_client,
					LM3533_REG_73 + i,
					0);
		lret |= lm3533_i2c_write(pst_light_led_data->pst_client,
					LM3533_REG_83 + i,
					0);
		lret |= lm3533_i2c_write(pst_light_led_data->pst_client,
					LM3533_REG_93 + i,
					0);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to pattern reg set\n", __func__);
			goto fail_i2c_check;
		}
	}

	LM3533_STATE_DISABLE(LED_RGB_STATE);
	LM3533_STATE_ENABLE(ul_color_state);
	lret = lm3533_i2c_write(pst_light_led_data->pst_client, 
					LM3533_REG_27,
					guc_light_status);
	if (lret != 0) {
		DEBUG_PRINT_ERR("%s() failed to ledlight on state = %d\n",
					__func__,
					guc_light_status);
		goto fail_i2c_check;
	}
	return lret;
fail_i2c_check:
	DEBUG_PRINT("%s() end  fail_i2c_check\n", __func__);
	return lret;
}

static int32_t lm3533_light_on(struct light_led_data_type *pst_light_led_data)
{

	int32_t lret;
	int32_t i;
	uint32_t ul_pattern_val;
	uint32_t ul_pattern_val_off;
	uint32_t ul_color_state = 0;
	
	ul_pattern_val = lm3533_color_pattern_check(pst_light_led_data->ul_value & 0x00FFFFFF);

	if (pst_light_led_data->blink_control == BLINK_REQUEST) {
		if (ul_pattern_val != COLOR_PATTERN_MAX) {
			pst_light_led_data->uc_led_red_val = gul_pattern_val[ul_pattern_val][0];
			pst_light_led_data->uc_led_green_val = gul_pattern_val[ul_pattern_val][1];
			pst_light_led_data->uc_led_blue_val = gul_pattern_val[ul_pattern_val][2];
		}
		ul_pattern_val_off = lm3533_color_pattern_check(gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color & 0x00FFFFFF);
		if (ul_pattern_val_off != COLOR_PATTERN_MAX) {
			gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color = ((uint32_t)gul_pattern_val[ul_pattern_val_off][0] << 16 & 0x00FF0000)
																+ ((uint32_t)gul_pattern_val[ul_pattern_val_off][1] << 8 & 0x0000FF00)
																+ ((uint32_t)gul_pattern_val[ul_pattern_val_off][2] & 0x000000FF);
		}
		DEBUG_PRINT("%s() red = %x green = %x blue = %x ,off_color = %x\n",
					__func__,
					pst_light_led_data->uc_led_red_val,
					pst_light_led_data->uc_led_green_val,
					pst_light_led_data->uc_led_blue_val,
					gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color);
		lret = lm3533_blink_request(pst_light_led_data);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to blink set\n", __func__);
			goto fail_i2c_check;
		}
	}
	else {
		if (ul_pattern_val != COLOR_PATTERN_MAX) {
			for(i =0; i < LED_COLOR_PATTERN_RGB; i++) {
				lret = lm3533_i2c_write(pst_light_led_data->pst_client,
						LM3533_REG_42 + i,
						gul_pattern_val[ul_pattern_val][i]);

				if (lret != 0) {
					DEBUG_PRINT_ERR("%s() failed to colorpattern val set\n", __func__);
					goto fail_i2c_check;
				}

				if (gul_pattern_val[ul_pattern_val][i] != LM3553_CURRENT_VALUE_OFF) {
					ul_color_state |= gul_color_state[i];
				}
			}
		}
		else {
			lret = lm3533_set_curval(pst_light_led_data);
			if (lret != 0) {
				DEBUG_PRINT_ERR("%s() failed to curval = %d\n",
							__func__,
							lret);
				goto fail_i2c_check;
			}

			if (pst_light_led_data->uc_led_red_val != LM3553_CURRENT_VALUE_OFF) {
				ul_color_state |= LED_RED_STATE;
			}

			if (pst_light_led_data->uc_led_green_val != LM3553_CURRENT_VALUE_OFF) {
				ul_color_state |= LED_GREEN_STATE;
			}

			if (pst_light_led_data->uc_led_blue_val != LM3553_CURRENT_VALUE_OFF) {
				ul_color_state |= LED_BLUE_STATE;
			}

		}
		LM3533_STATE_DISABLE(LED_RGB_STATE);
		LM3533_STATE_ENABLE(ul_color_state);
		lret = lm3533_i2c_write(pst_light_led_data->pst_client, 
						LM3533_REG_27,
						guc_light_status);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to ledlight on state = %d\n",
						__func__,
						guc_light_status);
			goto fail_i2c_check;
		}

	}
	return lret;
fail_i2c_check:
	DEBUG_PRINT("%s() end  fail_i2c_check\n", __func__);
	return lret;
}
static void led_set(struct light_led_data_type *pst_light_led_data)
{
	int32_t lret;
	uint8_t uc_color_red;
	uint8_t uc_color_green;
	uint8_t uc_color_blue;
	
	DEBUG_PRINT("%s() start \n", __func__);

	mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);

	if (LM3533_COL_BLACK == (pst_light_led_data->ul_value & 0x00FFFFFF)) {
		LM3533_STATE_DISABLE(LED_RGB_STATE);

		lret = lm3533_i2c_write(pst_light_led_data->pst_client,
						LM3533_REG_28,
						LM3533_REG28_OFF);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to pattern off state = %d \n",
						__func__,
						guc_light_status);
			goto fail_i2c_check;
		}

		lret = lm3533_i2c_write(pst_light_led_data->pst_client,
						LM3533_REG_27,
						guc_light_status);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to led off state = %d \n",
						__func__,
						guc_light_status);
			goto fail_i2c_check;
		}

		lret = lm3533_set_chargepump(pst_light_led_data->pst_client,
						LM3533_LED_DISABLE);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to lm3533_set_chargepump\n", __func__);
			goto fail_i2c_check;
		}

	}
	else {
		lret = lm3533_i2c_write(pst_light_led_data->pst_client,
						LM3533_REG_28,
						LM3533_REG28_OFF);
		if (lret != 0)
			DEBUG_PRINT_ERR("%s() failed to pattern off \n",__func__);

		lret = lm3533_set_chargepump(pst_light_led_data->pst_client,
						LM3533_LED_ENABLE);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to lm3533_set_chargepump\n", __func__);
			goto fail_i2c_check;
		}

		gul_value = (pst_light_led_data->ul_value&0x00FFFFFF);

		uc_color_red   = (u8)((pst_light_led_data->ul_value>>(16)) & 0xff);
		uc_color_green = (u8)((pst_light_led_data->ul_value>>(8)) & 0xff);
		uc_color_blue  = (u8)((pst_light_led_data->ul_value) & 0xff);

		if (uc_color_red <= LM3553_CURRENT_VALUE_MAX)
			pst_light_led_data->uc_led_red_val = uc_color_red;
		else
			pst_light_led_data->uc_led_red_val = LM3553_CURRENT_VALUE_MAX;

		if (uc_color_green <= LM3553_CURRENT_VALUE_MAX)
			pst_light_led_data->uc_led_green_val = uc_color_green;
		else
			pst_light_led_data->uc_led_green_val =
						LM3553_CURRENT_VALUE_MAX;

		if (uc_color_blue <= LM3553_CURRENT_VALUE_MAX)
			pst_light_led_data->uc_led_blue_val = uc_color_blue;
		else
			pst_light_led_data->uc_led_blue_val = LM3553_CURRENT_VALUE_MAX;

		DEBUG_PRINT("%s() color_red=0x%x red_val=0x%02x\n",
					__func__,
					uc_color_red,
					pst_light_led_data->uc_led_red_val);

		DEBUG_PRINT("%s() color_green=0x%02x green_val=0x%02x\n",
					__func__,
					uc_color_green,
					pst_light_led_data->uc_led_green_val);

		DEBUG_PRINT("%s() color_blue=0x%02x blue_val=0x%02x\n",
					__func__,
					uc_color_blue,
					pst_light_led_data->uc_led_blue_val);

		lret = lm3533_light_on(pst_light_led_data);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to led on\n", __func__);
			goto fail_i2c_check;
		}
	}
	mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
	DEBUG_PRINT("%s() end\n", __func__);
	return;
fail_i2c_check:
	mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
	if(atomic_read(&g_reset_err_status) == LM3533_LED_RESET_OFF) {
		DEBUG_PRINT("%s() reset_work call\n", __func__);
		light_led_err_reset(pst_light_led_data);
	}
	DEBUG_PRINT("%s() end  fail_i2c_check\n", __func__);
}


static void mobilelight_set(struct light_led_data_type *pst_light_led_data)
{
	int32_t lret;
	uint8_t uc_mobilelight_val;
	int32_t batt_temp = 0;

	DEBUG_PRINT("%s() start \n", __func__);

	mutex_lock(&gpst_light_led_data[MOBILELIGHT_INDEX].lock);

	if (LM3533_COL_BLACK == (pst_light_led_data->ul_value&0x00FFFFFF)) {
		LM3533_STATE_DISABLE(MOBILE_LIGHT_STATE);

		lret = lm3533_i2c_write(pst_light_led_data->pst_client,
						LM3533_REG_27,
						guc_light_status);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to mobilelight off state = %d \n",
						__func__,
						guc_light_status);
			goto fail_i2c_check;
		}

		lret = lm3533_set_chargepump(pst_light_led_data->pst_client,
						LM3533_LED_DISABLE);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to chargepump_set\n", __func__);
			goto fail_i2c_check;
		}

	}
	else {
		lret = lm3533_set_chargepump(pst_light_led_data->pst_client,
						LM3533_LED_ENABLE);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to chargepump_set\n", __func__);
			goto fail_i2c_check;
		}

		DEBUG_PRINT("%s() dm_val = %d\n",__func__,guc_light_dm);
		if (guc_light_dm == 1) {
			uc_mobilelight_val = MOBILELIGHT_BATTTERM_HIGH_VAL;
			DEBUG_PRINT("%s() mobilelight_val = %d\n", 
						__func__,
						uc_mobilelight_val);
		}
		else if(guc_light_dm == 2) {
			uc_mobilelight_val = MOBILELIGHT_BATTTERM_LOW_VAL;
			DEBUG_PRINT("%s() mobilelight_val = %d\n", 
						__func__,
						uc_mobilelight_val);
		}
		else {
			lret = kcbms_get_substrate_therm(&batt_temp);

			if (lret) {
				DEBUG_PRINT_ERR("%s() Failed to battery temp\n",__func__);
				batt_temp = MOBILELIGHT_THRESHOLD_VAL;
			}
			else {
				DEBUG_PRINT("%s() battery_temp = %d lret = %d\n",
							__func__,
							batt_temp,lret);
			}

			if (batt_temp >= MOBILELIGHT_THRESHOLD_VAL) {
				uc_mobilelight_val = MOBILELIGHT_BATTTERM_HIGH_VAL;
			}
			else {
				uc_mobilelight_val = MOBILELIGHT_BATTTERM_LOW_VAL;
			}

			DEBUG_PRINT("%s() mobilelight_val = %d\n", 
						__func__,
						uc_mobilelight_val);
		}

		lret = lm3533_i2c_write(pst_light_led_data->pst_client,
					LM3533_REG_24,
					uc_mobilelight_val);
		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to mobilelight val set\n", __func__);
			goto fail_i2c_check;
		}

		LM3533_STATE_ENABLE(MOBILE_LIGHT_STATE);

		lret = lm3533_i2c_write(pst_light_led_data->pst_client,
						LM3533_REG_27,
						guc_light_status);

		if (lret != 0) {
			DEBUG_PRINT_ERR("%s() failed to mobilelight on state = %d\n",
						__func__,
						guc_light_status);
			goto fail_i2c_check;
		}

	}
	DEBUG_PRINT("%s() end\n", __func__);
	mutex_unlock(&gpst_light_led_data[MOBILELIGHT_INDEX].lock);
	return;
fail_i2c_check:
	DEBUG_PRINT("%s() end  fail_i2c_check\n", __func__);
	mutex_unlock(&gpst_light_led_data[MOBILELIGHT_INDEX].lock);
	if(atomic_read(&g_reset_err_status) == LM3533_LED_RESET_OFF) {
		DEBUG_PRINT("%s() reset_work call\n", __func__);
		light_led_err_reset(pst_light_led_data);
	}
	DEBUG_PRINT("%s() end  fail_i2c_check\n", __func__);
}

static void backlight_set(struct light_led_data_type *pst_light_led_data)
{
	uint32_t ul_val;
	uint32_t lret = 0;
	uint8_t uc_backlightval;
	uint8_t uc_brightnessval;
	int32_t batt_temp = 0;
#ifndef DISABLE_DISP_DETECT
	int32_t display_detect;
#endif  /* DISABLE_DISP_DETECT */

	DEBUG_PRINT("%s() start \n", __func__);
	mutex_lock(&gpst_light_led_data[BACKLIGHT_INDEX].lock);

	ul_val = (pst_light_led_data->ul_value & 0xffffffff);
	uc_backlightval  = (u8)(ul_val & 0xff);
	uc_brightnessval = (u8)(ul_val>>(16) & 0xff);
#ifndef DISABLE_DISP_DETECT
	display_detect = atomic_read(&g_display_detect);
#endif  /* DISABLE_DISP_DETECT */

	DEBUG_PRINT("%s() backlight val = %d \n", __func__,uc_backlightval);
	if (uc_backlightval == LM3533__BACKLIGHT_COLOR_OFF) {
		if ((guc_light_status & BACK_LIGHT_STATE) != 0) {
			LM3533_STATE_DISABLE(BACK_LIGHT_STATE);
#ifndef DISABLE_DISP_DETECT
			if(display_detect == 0){
				lret = light_led_disp_set(LIGHT_MAIN_WLED_LED_DIS);
			}
			else if(display_detect == 1){
#endif  /* DISABLE_DISP_DETECT */
				lret = lm3533_i2c_write(pst_light_led_data->pst_client,
							LM3533_REG_27,
							guc_light_status);
#ifndef DISABLE_DISP_DETECT
			}
#endif  /* DISABLE_DISP_DETECT */
			if (lret != 0) {
				DEBUG_PRINT_ERR("%s() failed to backlight off state = %d \n",
						__func__,
						guc_light_status);
				goto fail_i2c_check;
			}

#ifndef DISABLE_CABC_PWM
			lret = light_led_cabc_set(LIGHT_MAIN_WLED_PWM_OFF);
#endif  /* DISABLE_CABC_PWM */

			if (lret != 0)
				DEBUG_PRINT_ERR("%s() failed to pwm off\n",
							__func__);

		}
	}
	else {
		DEBUG_PRINT("%s() backlight val = %d \n", __func__,uc_backlightval);
		if ((guc_light_status & BACK_LIGHT_STATE) == 0) {
			if (uc_backlightval >= LM3553_THRESHOLD_CURVAL) {
				lret = kcbms_get_substrate_therm(&batt_temp);
				if (lret) {
					DEBUG_PRINT("%s() Failed to battery temp\n",__func__);
					batt_temp = LM3553_THRESHOLD_TEMPVAL;
				}
				else{
					DEBUG_PRINT("%s() battery_temp = %d lret = %d\n",
								__func__,
								batt_temp,lret);
				}
				if (batt_temp >= LM3553_THRESHOLD_TEMPVAL) {
					uc_backlightval = LM3553_TEMP_COMPENSATING_CURVAL;
					DEBUG_PRINT_ERR("%s() Threshold over val = %d \n",
							__func__,
							uc_backlightval);
				}
				if (guc_light_dm == 1) {
					uc_backlightval = LM3553_TEMP_COMPENSATING_CURVAL;
					DEBUG_PRINT("%s() backlight val = %d dm = %d \n",
							__func__,
							uc_backlightval,
							guc_light_dm);
				}
				else if(guc_light_dm == 2) {
					uc_backlightval = batt_temp;
					DEBUG_PRINT("%s() backlight val = %d dm = %d \n",
							__func__,
							uc_backlightval,
							guc_light_dm);
				}
			}
		}

		guc_backlight_save_val = uc_backlightval;
		lret = lm3533_i2c_write(pst_light_led_data->pst_client,
						LM3533_REG_40,
						uc_backlightval);
		if (lret != 0)
			DEBUG_PRINT_ERR("%s() failed to backlight val\n", __func__);

#ifndef DISABLE_CABC_PWM
		lret = light_led_cabc_set(LIGHT_MAIN_WLED_PWM_ON);
#endif  /* DISABLE_CABC_PWM */

		if (lret != 0)
			DEBUG_PRINT_ERR("%s() failed to pwm on\n",
						__func__);

#ifndef DISABLE_DISP_DETECT
		if (((guc_light_status & BACK_LIGHT_STATE) == 0) || (display_detect == 0)) {
			if(display_detect == 0){
				lret = light_led_disp_set(LIGHT_MAIN_WLED_LED_EN);
				if (lret != 0) {
					DEBUG_PRINT_ERR("%s() failed to backlight on state = %d\n",
							__func__,
							guc_light_status);
					goto fail_i2c_check;
				}
			}
			else if(display_detect == 1){
#else   /* DISABLE_DISP_DETECT */
		if ((guc_light_status & BACK_LIGHT_STATE) == 0) {
#endif  /* DISABLE_DISP_DETECT */
				LM3533_STATE_ENABLE(BACK_LIGHT_STATE);
				lret = lm3533_i2c_write(pst_light_led_data->pst_client,
							LM3533_REG_27,
							guc_light_status);
				if (lret != 0) {
					DEBUG_PRINT_ERR("%s() failed to backlight on state = %d\n",
							__func__,
							guc_light_status);
					goto fail_i2c_check;
				}
#ifndef DISABLE_DISP_DETECT
			}
			else{
				DEBUG_PRINT_ERR("%s: No set display display_detect=%x\n",
						__func__,(int32_t)display_detect);
			}
#endif  /* DISABLE_DISP_DETECT */
		}
	}
	mutex_unlock(&gpst_light_led_data[BACKLIGHT_INDEX].lock);
	DEBUG_PRINT("%s() end \n", __func__);
	return;
fail_i2c_check:
	DEBUG_PRINT("%s() end  fail_i2c_check\n", __func__);
	mutex_unlock(&gpst_light_led_data[BACKLIGHT_INDEX].lock);
	if(atomic_read(&g_reset_err_status) == LM3533_LED_RESET_OFF) {
		DEBUG_PRINT("%s() reset_work call\n", __func__);
		light_led_err_reset(pst_light_led_data);
	}
}

static void light_led_work(struct work_struct *work)
{
	struct light_led_data_type *pst_light_led_data;
	
	pst_light_led_data = container_of(work,
					struct light_led_data_type,
					work);

	DEBUG_PRINT("%s() name=%s\n", __func__,
					pst_light_led_data->st_cdev.name);

	if (!strcmp(pst_light_led_data->st_cdev.name, MOBILELIGHT_INFO))
		mobilelight_set(pst_light_led_data);
	else if (!strcmp(pst_light_led_data->st_cdev.name, BACKLIGHT_INFO)) 
		backlight_set(pst_light_led_data);
	else if (!strcmp(pst_light_led_data->st_cdev.name, LED_INFO)) 
		led_set(pst_light_led_data);

}
static void light_led_set(struct led_classdev *pst_cdev, enum led_brightness value)
{
	struct light_led_data_type *pst_light_led_data;

	pst_light_led_data = container_of(pst_cdev,
					struct light_led_data_type,
					st_cdev);

	DEBUG_PRINT("%s() start name=%s value=0x%08x\n",
					__func__,
					pst_cdev->name,
					value);

	mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
	pst_light_led_data->ul_value = value;
	mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
	schedule_work(&pst_light_led_data->work);

}

static enum led_brightness light_led_get(struct led_classdev *pst_cdev)
{
	int32_t lret = 0;
	struct light_led_data_type *pst_light_led_data;
	DEBUG_PRINT("%s() start\n", __func__);

	pst_light_led_data = container_of(pst_cdev,
					struct light_led_data_type,
					st_cdev);
	lret = pst_light_led_data->ul_value;
	DEBUG_PRINT("%s() end\n", __func__);

	return lret;
}

static int32_t light_led_init_reg(struct i2c_client *pst_client)
{
	int32_t lret = 0;
	DEBUG_PRINT("%s() start\n", __func__);

	lret = lm3533_i2c_write(pst_client,
					LM3533_REG_10,
					LM3533_REG10_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_1A,
					LM3533_REG1A_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_1B,
					LM3533_REG1B_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_1C,
					LM3533_REG1C_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_1D,
					LM3533_REG1D_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_1E,
					LM3533_REG1E_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_1F,
					LM3533_REG1F_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_21,
					LM3533_REG21_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_22,
					LM3533_REG22_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_23,
					LM3533_REG23_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_24,
					LM3533_REG24_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_2C,
					LM3533_REG2C_INIT);
	lret |= lm3533_i2c_write(pst_client,
					LM3533_REG_45,
					LM3533_REG45_INIT);

	DEBUG_PRINT("%s() end lret[%d]\n", __func__,lret);

	return lret;
}

static int32_t light_led_err_reset(struct light_led_data_type *pst_light_led_data)
{
	int32_t lret = 0;
	
	DEBUG_PRINT("%s() start\n", __func__);
	atomic_set(&g_reset_err_status, LM3533_LED_RESET_ON);

	DEBUG_PRINT("%s() uc_backlight_save_val = 0x%08x \n", __func__,guc_backlight_save_val);

	gpio_set_value_cansleep(LM3533_RESET_GPIO, 0);
	mdelay(1);
	DEBUG_PRINT("%s() gpio47 = %d \n", __func__,gpio_get_value_cansleep(LM3533_RESET_GPIO));
	gpio_set_value_cansleep(LM3533_RESET_GPIO, 1);
	mdelay(1);
	DEBUG_PRINT("%s() gpio47 = %d \n", __func__,gpio_get_value_cansleep(LM3533_RESET_GPIO));
	
	lret = light_led_init_reg(pst_light_led_data->pst_client);
	if (lret != 0) {
		atomic_set(&g_reset_err_status, LM3533_LED_RESET_OFF);
		DEBUG_PRINT_ERR("%s() init_failed \n", __func__);
		return lret;
	}

	if ((guc_light_status & BACK_LIGHT_STATE) != 0) {
		DEBUG_PRINT("%s() BACK_LIGHT_STATE ON \n", __func__);
		pst_light_led_data->ul_value = guc_backlight_save_val;
		LM3533_STATE_DISABLE(BACK_LIGHT_STATE);
		backlight_set(pst_light_led_data);
	}

	if ((guc_light_status & MOBILE_LIGHT_STATE) != 0) {
		DEBUG_PRINT("%s() MOBILE_LIGHT_STATE ON \n", __func__);
		LM3533_STATE_DISABLE(MOBILE_LIGHT_STATE);
		pst_light_led_data->ul_value = 1;
		mobilelight_set(pst_light_led_data);
	}

	if ((guc_light_status & LED_RGB_STATE) != 0) {
		DEBUG_PRINT("%s() LED_RGB_STATE ON \n", __func__);
		LM3533_STATE_DISABLE(LED_RGB_STATE);
		pst_light_led_data->ul_value = gul_value;
		led_set(pst_light_led_data);
	}

	atomic_set(&g_reset_err_status, LM3533_LED_RESET_OFF);
	DEBUG_PRINT("%s() end\n", __func__);
	return lret;
}

static long leds_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int32_t ret = -1;
	T_LEDLIGHT_IOCTL st_ioctl;
	T_LEDLIGHT_IOCTL_DM st_ioctl_dm;

	DEBUG_PRINT("%s: start\n",__func__);
	switch (cmd) {
	case LEDLIGHT_SET_BLINK:
		DEBUG_PRINT("%s: LEDLIGHT_SET_BLINK\n",__func__);
		ret = copy_from_user(&st_ioctl,
					argp,
					sizeof(T_LEDLIGHT_IOCTL));
		if (ret) {
			DEBUG_PRINT_ERR("error : leds_ioctl(cmd = LEDLIGHT_SET_BLINK)\n");
			return -EFAULT;
		}
		DEBUG_PRINT("%s:%d,%d,%d,%d\n",__func__,
						st_ioctl.data[0],
						st_ioctl.data[1],
						st_ioctl.data[2],
						st_ioctl.data[3]);
		mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
		gpst_light_led_data[LEDLIGHT_INDEX].blink_control = st_ioctl.data[0];
		gpst_light_led_data[LEDLIGHT_INDEX].blink_on_delay = st_ioctl.data[1];
		gpst_light_led_data[LEDLIGHT_INDEX].blink_off_delay = st_ioctl.data[2];
		gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color = st_ioctl.data[3];
		mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
		break;
	case LEDLIGHT_SET_TEMPERTURE_DM:
		DEBUG_PRINT("%s: LEDLIGHT_SET_TEMPERTURE_DM\n",__func__);
		ret = copy_from_user(&st_ioctl_dm,
					argp,
					sizeof(T_LEDLIGHT_IOCTL_DM));
		if (ret) {
			DEBUG_PRINT_ERR("error : st_ioctl_dm(cmd = LEDLIGHT_SET_TEMPERTURE_DM)\n");
			return -EFAULT;
		}
		mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
		guc_light_dm = st_ioctl_dm.dm_data;
		DEBUG_PRINT("%s: guc_light_dm=%d\n",__func__,guc_light_dm);
		mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
		break;
	default:
		DEBUG_PRINT("%s: default\n",__func__);
		return -ENOTTY;
	}
	DEBUG_PRINT("%s: end\n",__func__);
	return 0;
}

int32_t light_led_cabc_set(e_light_main_wled cabc)
{
	int32_t ret = 0;
	e_light_main_wled status;
	DEBUG_PRINT("%s: cabc = 0x%x\n",__func__, (uint32_t)cabc);

	mutex_lock(&cabc_lock);
	status = (e_light_main_wled)atomic_read(&g_cabc_status);
	DEBUG_PRINT("%s: start status =0x%x cabc = 0x%x\n",__func__,(uint32_t)status, (uint32_t)cabc);

	switch(cabc) {
	case LIGHT_MAIN_WLED_CABC_ON:
		status |= cabc;
		if((status&LIGHT_MAIN_WLED_CABC_EN) || !(status&LIGHT_MAIN_WLED_PWM_ON))
			break;
	case LIGHT_MAIN_WLED_PWM_ON:
		status |= cabc;
		if(status&LIGHT_MAIN_WLED_CABC_ON && !(status&LIGHT_MAIN_WLED_CABC_EN)) {
#ifndef DISABLE_CABC_PWM
			if(!disp_ext_blc_mode_select(1))
				status |= LIGHT_MAIN_WLED_CABC_EN;
			else
				ret = -1;
			DEBUG_PRINT("%s: [end]cabc on status=%x\n",__func__,(uint32_t)status);
#endif  /* DISABLE_CABC_PWM */
		}
		if(status&LIGHT_MAIN_WLED_CABC_EN && !(status&LIGHT_MAIN_WLED_PWM_EN)) {
			if(!lm3533_set_cabc(&gpst_light_led_data[BACKLIGHT_INDEX],LM3533_LED_PWM_ENABLE))
				status |= LIGHT_MAIN_WLED_PWM_EN;
			else
				ret = -1;
			DEBUG_PRINT("%s: [end]pwm on status=%x\n",__func__,(uint32_t)status);
		}	
		break;
	case LIGHT_MAIN_WLED_CABC_OFF:
		status &= ~(cabc>>4);
	case LIGHT_MAIN_WLED_PWM_OFF:
		status &= ~(cabc>>4);
		if(status&LIGHT_MAIN_WLED_PWM_EN) {
			if(!lm3533_set_cabc(&gpst_light_led_data[BACKLIGHT_INDEX],LM3533_LED_PWM_DISABLE))
				status &= ~(LIGHT_MAIN_WLED_PWM_EN);
			else
				ret = -1;
			DEBUG_PRINT("%s: [end]pwm off status=%x\n",__func__,(uint32_t)status);
		}
		if(status&LIGHT_MAIN_WLED_CABC_EN && !(status&LIGHT_MAIN_WLED_PWM_EN)) {
#ifndef DISABLE_CABC_PWM
			if(!disp_ext_blc_mode_select(0))
				status &= ~(LIGHT_MAIN_WLED_CABC_EN);
			else
				ret = -1;
			DEBUG_PRINT("%s: [end]cabc off status=%x\n",__func__,(uint32_t)status);
#endif  /* DISABLE_CABC_PWM */
		}
		break;
	default:
		ret = -1;
		break;
	}
	DEBUG_PRINT("%s: status = 0x%x\n",__func__,(uint32_t)status);
	atomic_set(&g_cabc_status,(uint32_t)status);
	mutex_unlock(&cabc_lock);
	DEBUG_PRINT("%s: end ret=%d\n",__func__,ret);
	return ret;
}
EXPORT_SYMBOL(light_led_cabc_set);

e_light_main_wled light_led_cabc_get(void)
{
	e_light_main_wled ret = (e_light_main_wled)(atomic_read(&g_cabc_status)&0x00FF);
	DEBUG_PRINT("%s: start status = %d\n",__func__,(uint32_t)ret);
	DEBUG_PRINT("%s: end\n",__func__);
	return ret;
}
EXPORT_SYMBOL(light_led_cabc_get);

void light_led_threecolor_led_set(int32_t color,int32_t blink)
{
	DEBUG_PRINT("%s: start color = %x, blink = %d\n",__func__,color,blink);
	mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
	gpst_light_led_data[LEDLIGHT_INDEX].ul_value = color;

	if( blink == 0 ) {
		gpst_light_led_data[LEDLIGHT_INDEX].blink_control = NO_BLINK_REQUEST;
		gpst_light_led_data[LEDLIGHT_INDEX].blink_on_delay = 0;
		gpst_light_led_data[LEDLIGHT_INDEX].blink_off_delay = 0;
		gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color = 0;
	}
	else {
		gpst_light_led_data[LEDLIGHT_INDEX].blink_control = BLINK_REQUEST;
		gpst_light_led_data[LEDLIGHT_INDEX].blink_on_delay = 0x0E;
		gpst_light_led_data[LEDLIGHT_INDEX].blink_off_delay = 0x0E;
		gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color = 0;
	}
	mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);

	schedule_work(&gpst_light_led_data[LEDLIGHT_INDEX].work);
	DEBUG_PRINT("%s: end\n",__func__);
}
EXPORT_SYMBOL(light_led_threecolor_led_set);

#ifndef DISABLE_DISP_DETECT
int32_t light_led_disp_set(e_light_main_wled_disp disp_status)
{
	int32_t ret = 0;
	e_light_main_wled_disp status;

	mutex_lock(&led_disp_lock);
	status = (e_light_main_wled_disp)atomic_read(&g_disp_status);
	DEBUG_PRINT("%s: start status =0x%x disp_status = 0x%x\n",__func__,(uint32_t)status, (uint32_t)disp_status);

	if((atomic_read(&g_display_detect)) != 0){
		mutex_unlock(&led_disp_lock);
		DEBUG_PRINT("%s: [end]Already set g_display_detect=%d\n",__func__,(int32_t)atomic_read(&g_display_detect));
		return ret;
	}

	switch(disp_status) {
	case LIGHT_MAIN_WLED_LCD_EN:
	case LIGHT_MAIN_WLED_LED_EN:
		status |= disp_status;
		if(LIGHT_MAIN_WLED_EN == status) {
			LM3533_STATE_ENABLE(BACK_LIGHT_STATE);
			ret = lm3533_i2c_write(gpst_light_led_data[BACKLIGHT_INDEX].pst_client,
						LM3533_REG_27,
						guc_light_status);
			atomic_set(&g_display_detect,1);
			DEBUG_PRINT("%s: Set display detect status=%x\n",__func__,(uint32_t)status);
			if ((ret != 0) && (atomic_read(&g_reset_err_status) == LM3533_LED_RESET_OFF) &&
				(LIGHT_MAIN_WLED_LCD_EN == disp_status)) {
				DEBUG_PRINT_ERR("%s() failed to backlight on state = %d\n",
							__func__,
							guc_light_status);
				DEBUG_PRINT("%s() reset_work call\n", __func__);
				ret = light_led_err_reset(&gpst_light_led_data[BACKLIGHT_INDEX]);
			}
		}
		break;
	case LIGHT_MAIN_WLED_LCD_DIS:
		atomic_set(&g_display_detect,-1);
		DEBUG_PRINT_ERR("%s: No set display disp_status=%x\n",__func__,(uint32_t)disp_status);
	case LIGHT_MAIN_WLED_LED_DIS:
		status &= ~(disp_status>>4);
		DEBUG_PRINT("%s: status=%x\n",__func__,(uint32_t)status);
		break;
	default:
		break;
	}
	DEBUG_PRINT("%s: status = 0x%x g_display_detect=%d\n",
			__func__,(uint32_t)status,(int32_t)atomic_read(&g_display_detect));
	atomic_set(&g_disp_status,(uint32_t)status);
	mutex_unlock(&led_disp_lock);
	DEBUG_PRINT("%s: end ret=%d\n",__func__,ret);
	return ret;
}
EXPORT_SYMBOL(light_led_disp_set);
#endif  /* DISABLE_DISP_DETECT */

static int32_t leds_open(struct inode* inode, struct file* filp)
{
	DEBUG_PRINT("%s: start\n",__func__);
	DEBUG_PRINT("%s: end\n",__func__);
	return 0;
}

static int32_t leds_release(struct inode* inode, struct file* filp)
{
	DEBUG_PRINT("%s: start\n",__func__);
	DEBUG_PRINT("%s: end\n",__func__);
	return 0;
}

static struct file_operations leds_fops = {
	.owner		= THIS_MODULE,
	.open		= leds_open,
	.release	= leds_release,
	.unlocked_ioctl = leds_ioctl,
};

static struct miscdevice leds_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "leds-ledlight",
	.fops  = &leds_fops,
};

static void led_class_setinfo(struct light_led_data_type *pst_light_led_data, uint32_t ul_ledindex)
{
	
	switch (ul_ledindex) {
	case MOBILELIGHT_INDEX :
		pst_light_led_data[ul_ledindex].st_cdev.name   = MOBILELIGHT_INFO;
		pst_light_led_data[ul_ledindex].st_cdev.max_brightness = MOBILELIGHT_MAX_BRIGHT_VAL;
		break;
	case LEDLIGHT_INDEX :
		pst_light_led_data[ul_ledindex].st_cdev.name   = LED_INFO;
		pst_light_led_data[ul_ledindex].st_cdev.max_brightness = (unsigned int)LEDLIGHT_MAX_BRIGHT_VAL;
		break;
	case BACKLIGHT_INDEX :
		pst_light_led_data[ul_ledindex].st_cdev.name   = BACKLIGHT_INFO;
		pst_light_led_data[ul_ledindex].st_cdev.max_brightness = (unsigned int)BACKLIGHT_MAX_BRIGHT_VAL;
		break;
	}

	pst_light_led_data[ul_ledindex].st_cdev.brightness_set = light_led_set;
	pst_light_led_data[ul_ledindex].st_cdev.brightness_get = light_led_get;
	pst_light_led_data[ul_ledindex].st_cdev.brightness     = LM3533_COL_BLACK;
	pst_light_led_data[ul_ledindex].st_cdev.flags          = 0;
}

static int32_t __devinit light_led_probe(struct i2c_client *pst_client,
			const struct i2c_device_id *id)
{
	int32_t lret=0;
	struct light_led_data_type *pst_light_led_data;
	int32_t i;
	uint8_t ctl_buf;
	int32_t r_ret;

	DEBUG_PRINT("%s() start pst_client=0x%08x idname=%s\n",
					__func__,
					(unsigned int)pst_client,
					id->name);

	pst_light_led_data = kzalloc(
				sizeof(struct light_led_data_type)*LIGHT_INDEX_MAX,
				GFP_KERNEL);

	if (!pst_light_led_data) {
		DEBUG_PRINT_ERR("%s() failed to allocate driver data\n", __func__);
		return -1;
	}
	i2c_set_clientdata(pst_client, pst_light_led_data);
	gpst_light_led_data = pst_light_led_data;

	mutex_init(&cabc_lock);
#ifndef DISABLE_DISP_DETECT
	mutex_init(&led_disp_lock);
#endif  /* DISABLE_DISP_DETECT */

	for (i=0; i<LIGHT_INDEX_MAX; i++) {
		INIT_WORK(&pst_light_led_data[i].work, light_led_work);
		mutex_init(&pst_light_led_data[i].lock);
		spin_lock_init(&pst_light_led_data[i].value_lock);
		pst_light_led_data[i].pst_client = pst_client;
		if (i == 0) {
			lret = light_led_init_reg(pst_client);
			if (lret != 0) {
				DEBUG_PRINT("%s() reset_work call\n", __func__);
				lret = light_led_err_reset(pst_light_led_data);
				if (lret != 0) {
					DEBUG_PRINT_ERR("%s() light_led_init_reg()\n",
								__func__);
					goto fail_id_check;
				}
			}
		}

		led_class_setinfo(pst_light_led_data, i);

		lret = led_classdev_register(
				&pst_light_led_data[i].pst_client->dev,
				&pst_light_led_data[i].st_cdev);
		if (lret) {
			DEBUG_PRINT_ERR("%s() unable to register led %s\n",
					__func__,
					pst_light_led_data[i].st_cdev.name);
			goto fail_id_check;
		}
		pst_light_led_data[i].blink_control   = NO_BLINK_REQUEST; 
		pst_light_led_data[i].blink_on_delay  = 0;
		pst_light_led_data[i].blink_off_delay = 0;
		pst_light_led_data[i].blink_off_color = 0;
	}

	r_ret = lm3533_i2c_read_byte(pst_light_led_data->pst_client,LM3533_REG_27,&ctl_buf);
	DEBUG_PRINT("%s() r_ret=%d ctl_buf=%x \n", __func__,r_ret,ctl_buf);
	if((r_ret == 0) && (ctl_buf & BACK_LIGHT_STATE)){
		LM3533_STATE_ENABLE(BACK_LIGHT_STATE);
	}

	wake_lock_init(&pst_light_led_data[LEDLIGHT_INDEX].work_wake_lock,
						WAKE_LOCK_SUSPEND,
						"led-lights");

	misc_register(&leds_device);
	atomic_set(&g_reset_err_status, LM3533_LED_RESET_OFF);

	DEBUG_PRINT("%s() end\n", __func__);
	return lret;

fail_id_check:
	while (i--) {
		led_classdev_unregister(&pst_light_led_data[i].st_cdev);
	}
	kfree(pst_light_led_data);
	return lret;
}

static int32_t __exit light_led_remove(struct i2c_client *pst_client)
{
	int32_t lret = 0;
	DEBUG_PRINT("%s: start\n",__func__);
	DEBUG_PRINT("%s() end\n", __func__);
	return lret;
}

static int32_t light_led_suspend(struct i2c_client *pst_client, pm_message_t mesg)
{
	int32_t lret= 0;
	DEBUG_PRINT("%s: start\n",__func__);
	DEBUG_PRINT("%s() end\n", __func__);
	return lret;
}

static int32_t light_led_resume(struct i2c_client *pst_client)
{
	int32_t lret= 0;
	DEBUG_PRINT("%s() end\n", __func__);
	DEBUG_PRINT("%s: start\n",__func__);
	return lret;
}

#if 0
static void light_led_shutdown(struct i2c_client *pst_client)
{
	struct light_led_data_type *pst_light_led_data;
	int32_t i;

	DEBUG_PRINT("%s() start\n", __func__);

	pst_light_led_data = i2c_get_clientdata(pst_client);

	for (i=0; i<LIGHT_INDEX_MAX; i++)
		led_classdev_unregister(&pst_light_led_data[i].st_cdev);

	misc_deregister(&leds_device);
	kfree(pst_light_led_data);

	DEBUG_PRINT("%s() end\n", __func__);
	return;
}
#endif

static const struct i2c_device_id ledlight_id[] = {
	{ LED_DRV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ledlight_id);

static struct i2c_driver light_led_driver = {
	.driver = {
		.name	= LED_DRV_NAME,
	},
	.probe    = light_led_probe,
	.remove   = __exit_p(light_led_remove),
	.suspend  = light_led_suspend,
	.resume   = light_led_resume,
//	.shutdown = light_led_shutdown,
	.id_table = ledlight_id,

};

static int32_t __init light_led_init(void)
{

	int32_t rc;

	DEBUG_PRINT("%s() start\n", __func__);

	gpio_set_value_cansleep(LM3533_RESET_GPIO, 1);
	mdelay(1);
	DEBUG_PRINT("%s() gpio47 = %d \n", __func__,gpio_get_value_cansleep(LM3533_RESET_GPIO));

	rc = i2c_add_driver(&light_led_driver);

	if (rc != 0) {
		DEBUG_PRINT("can't add i2c driver\n");
		rc = -ENOTSUPP;
	}

	DEBUG_PRINT("%s() end\n", __func__);
	return rc;

}
module_init(light_led_init);

static void __exit light_led_exit(void)
{
	i2c_del_driver(&light_led_driver);
}
module_exit(light_led_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("ledlight driver");
MODULE_LICENSE("GPL");


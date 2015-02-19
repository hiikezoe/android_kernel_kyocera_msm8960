/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */
/*
 *  apds990x.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>

#include <linux/gpio.h>
#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include <mach/hs_io_ctl_a.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/apds990x.h>
#include <linux/sensor_power.h>

#include <linux/wakelock.h>

#define APDS990x_DRV_NAME   "APDS-9900"
#define DRIVER_VERSION      "1.0.4"

#define APDS990x_ALS_THRESHOLD_HSYTERESIS   20

#define APDS_PROXIMITY_SENSOR_GPIO          (46)
#define VSENSOR_GPIO                        (56)
#define GPIO_ON                              (1)
#define GPIO_OFF                             (0)

#define ABS_MISC_DUMMY_VAL                  (-1)

#define APDS_DEBUG        0

#if APDS_DEBUG
#define APDS_DEBUG_LOG( arg... )   printk("APDS:" arg )
#else
#define APDS_DEBUG_LOG( arg... )
#endif

#define APDS990x_ENABLE_REG 0x00
#define APDS990x_ATIME_REG  0x01
#define APDS990x_PTIME_REG  0x02
#define APDS990x_WTIME_REG  0x03
#define APDS990x_AILTL_REG  0x04
#define APDS990x_AILTH_REG  0x05
#define APDS990x_AIHTL_REG  0x06
#define APDS990x_AIHTH_REG  0x07
#define APDS990x_PILTL_REG  0x08
#define APDS990x_PILTH_REG  0x09
#define APDS990x_PIHTL_REG  0x0A
#define APDS990x_PIHTH_REG  0x0B
#define APDS990x_PERS_REG   0x0C
#define APDS990x_CONFIG_REG 0x0D
#define APDS990x_PPCOUNT_REG    0x0E
#define APDS990x_CONTROL_REG    0x0F
#define APDS990x_REV_REG    0x11
#define APDS990x_ID_REG     0x12
#define APDS990x_STATUS_REG 0x13
#define APDS990x_CDATAL_REG 0x14
#define APDS990x_CDATAH_REG 0x15
#define APDS990x_IRDATAL_REG    0x16
#define APDS990x_IRDATAH_REG    0x17
#define APDS990x_PDATAL_REG 0x18
#define APDS990x_PDATAH_REG 0x19

#define CMD_BYTE    0x80
#define CMD_WORD    0xA0
#define CMD_SPECIAL 0xE0

#define CMD_CLR_PS_INT  0xE5
#define CMD_CLR_ALS_INT 0xE6
#define CMD_CLR_PS_ALS_INT  0xE7

#define APDS990X_LUXVALUE_MAX       50000
#define APDS990X_LUXVALUE_TABLE_MAX 50

#define APDS990X_DEV_STATUS_INIT            0x00000000
#define APDS990X_DEV_STATUS_SUSPEND         0x00000001
#define APDS990X_DEV_STATUS_SUSPEND_INT     0x00000002
#define APDS990X_DEV_STATUS_RESUME          0x00000004

#define ALS_POLLING_CNT_RESET_NONE          0x00000000
#define ALS_POLLING_CNT_RESET_MAX_DATA      0x00000001
#define ALS_POLLING_CNT_RESET_DISABLE       0x00000002
#define ALS_POLLING_CNT_RESET_STORE_POLL    0x00000004
#define ALS_POLLING_CNT_RESET_STORE_TIME    0x00000008
#define ALS_POLLING_CNT_RESET_INIT          0x00000010
#define ALS_POLLING_CNT_RESET_RESUME        0x00000020

#define APDS990X_ALS_GAIN_1X    1
#define APDS990X_ALS_GAIN_8X    8
#define APDS990X_ALS_GAIN_16X   16
#define APDS990X_ALS_GAIN_120X  120

#define APDS_WAKE_LOCK_TIME                    (HZ / 10)
#define APDS_WAKE_LOCK_INPUT_TIME              (HZ)

#define APDS990X_ERR_MAX_CNT    5

#define APDS990X_COLOR_VARI     5

struct apds990x_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct delayed_work dwork;
	struct delayed_work    als_dwork;
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;

	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;

	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold;
	unsigned int ps_detection;
	unsigned int ps_data;

	unsigned int als_threshold_l;
	unsigned int als_threshold_h;
	unsigned int als_data;

	unsigned int als_gain;
	unsigned int als_poll_delay;
	unsigned int als_atime;

	uint32_t vreg_on;
	int32_t ps_irq;
	int32_t cdata;
	int32_t irdata;
	int32_t pdata;
	uint32_t luxValue_table[APDS990X_LUXVALUE_TABLE_MAX];
	uint32_t als_lux_ave;
	uint32_t als_polling_cnt;
	uint32_t als_mean_times;
	uint32_t als_polling_cnt_reset;
};

enum apds_type_enum{
	TYPE_BYTE_READ = 0,
	TYPE_WORD_READ,
	TYPE_BYTE_WRITE,
	TYPE_WORD_WRITE,
	TYPE_SP_WRITE
};

static struct sensor_power_callback apds_power_cb;

static struct i2c_client *client_apds = NULL;

static struct wake_lock apds_wake_lock;
static struct wake_lock apds_wake_lock_input;

static uint16_t guc_nv_proximity_sensor_near[5] = {0x03F0,0x03F0,0x03F0,0x03F0,0x03F0};
static uint16_t guc_nv_proximity_sensor_far[5]  = {0x03D0,0x03D0,0x03D0,0x03D0,0x03D0};
static uint16_t guc_nv_photo_sensor_beamish[1]  = {0xAAAA};
static uint16_t guc_nv_photo_sensor_dark[1]     = {0x9000};
static uint16_t guc_nv_photo_sensor_b[5]        = {0x07F6,0x07F6,0x07F6,0x07F6,0x07F6};
static uint16_t guc_nv_photo_sensor_c[5]        = {0x029E,0x029E,0x029E,0x029E,0x029E};
static uint16_t guc_nv_photo_sensor_d[5]        = {0x0518,0x0518,0x0518,0x0518,0x0518};
static uint16_t guc_nv_photo_sensor_ga[5]       = {0x0898,0x0898,0x0898,0x0898,0x0898};
static uint16_t guc_nv_prox_photo_colvar[1]     = {0x0000};
static uint32_t gun_nv_status                   = 0;
static atomic_t g_dev_status;
static struct workqueue_struct *als_polling_wq;
static struct workqueue_struct *ps_polling_wq;

static void apds990x_disable_ps_irq( struct apds990x_data *data );
static int apds990x_init_client2(struct i2c_client *client);
static int apds990x_init_client(struct i2c_client *client);
void apds990x_enable_ps_sensor(struct i2c_client *client);
void apds990x_enable_als_sensor(struct i2c_client *client);
static int apds990x_set_enable(struct i2c_client *client, int enable);

void apds990x_power_on( void )
{
	struct apds990x_data *data = i2c_get_clientdata(client_apds);
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	data->vreg_on = 1;
	apds990x_init_client2(client_apds);
	if(data->enable_ps_sensor > 0)
		apds990x_enable_ps_sensor(client_apds);
	
	if(data->enable_als_sensor > 0)
		apds990x_enable_als_sensor(client_apds);
	
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

void apds990x_power_off( void )
{
	unsigned long flags;
	struct apds990x_data *data = i2c_get_clientdata(client_apds);
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	apds990x_disable_ps_irq(data);
	apds990x_set_enable(client_apds,0);
	data->vreg_on = 0;
	spin_lock_irqsave(&data->update_lock.wait_lock, flags); 

	__cancel_delayed_work(&data->als_dwork);

	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags); 
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

void apds_vpro_voltage_set(int32_t enable)
{
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s enable[%d]\n",__func__,enable);
	if(enable) {
		sensor_power_on(SENSOR_INDEX_APDS);
		sensor_power_reg_cbfunc(&apds_power_cb);
	} else {
		sensor_power_unreg_cbfunc(&apds_power_cb);
		sensor_power_off(SENSOR_INDEX_APDS);
	}
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static int32_t apds990x_reg_read(struct i2c_client *client,int8_t type,int32_t reg)
{
	int32_t ret = 0;
	int8_t errcnt = 0;

	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s reg[%x] type[%x]\n",__func__,reg,type);
	switch (type) {
	case TYPE_BYTE_READ:
		do {
			ret = i2c_smbus_read_byte_data(client,CMD_BYTE|reg);
			APDS_DEBUG_LOG("%s read_byte ret[%x]\n",__func__,ret);
		}while(ret < 0 && (++errcnt < APDS990X_ERR_MAX_CNT));

		break;
	case TYPE_WORD_READ:
		do {
			ret = i2c_smbus_read_word_data(client,CMD_WORD|reg);
			APDS_DEBUG_LOG("%s read_word ret[%x]\n",__func__,ret);
		}while(ret < 0 && (++errcnt < APDS990X_ERR_MAX_CNT));
		break;
	default :
		APDS_DEBUG_LOG("%s parameter err\n",__func__);
	}

	APDS_DEBUG_LOG("[OUT]%s ret[%d]\n",__func__,ret);
	return ret;
}

static int32_t apds990x_reg_write(struct i2c_client *client, int8_t type, int32_t reg, int32_t value)
{
	int32_t ret = 0;
	int8_t errcnt = 0;
	
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s reg[%x] value[%x] type[%x]\n",__func__,reg,value,type);
	switch (type) {
	case TYPE_BYTE_WRITE:
		do {
			ret = i2c_smbus_write_byte_data(client,CMD_BYTE|reg,value);
			APDS_DEBUG_LOG("%s write_byte ret[%d]\n",__func__,ret);
		}while(ret != 0 && (++errcnt < APDS990X_ERR_MAX_CNT));
		break;
	case TYPE_WORD_WRITE:
		do {
			ret = i2c_smbus_write_word_data(client,CMD_WORD|reg,value);
			APDS_DEBUG_LOG("%s write_word ret[%d]\n",__func__,ret);
		}while(ret != 0 && (++errcnt < APDS990X_ERR_MAX_CNT));
		break;
	case TYPE_SP_WRITE:
		do {
			ret = i2c_smbus_write_byte(client,CMD_SPECIAL|reg);
			APDS_DEBUG_LOG("%s write_sp ret[%d]\n",__func__,ret);
		}while(ret != 0 && (++errcnt < APDS990X_ERR_MAX_CNT));
		break;
	default :
		APDS_DEBUG_LOG("%s parameter err\n",__func__);
	}
	APDS_DEBUG_LOG("[OUT]%s ret[%d]\n",__func__,ret);
	return ret;
}



static int apds990x_set_enable(struct i2c_client *client, int enable)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s vreg_on[%d] enable[%d]\n",__func__,data->vreg_on,enable);
	if(!data->vreg_on) {
		apds_vpro_voltage_set(1);
		data->vreg_on = 1;
	}

	mutex_lock(&data->update_lock);

	ret = apds990x_reg_write(client, TYPE_BYTE_WRITE, APDS990x_ENABLE_REG, enable);
	APDS_DEBUG_LOG("%s ps_enable[%d] als_enable[%d]\n",__func__,
		data->enable_ps_sensor,data->enable_als_sensor);
	if(data->enable_ps_sensor <=0 && data->enable_als_sensor <= 0) {
		apds_vpro_voltage_set(0);
		data->vreg_on = 0;
	}

	mutex_unlock(&data->update_lock);

	data->enable = enable;

	APDS_DEBUG_LOG("[OUT]%s ret=%d\n",__func__,ret);
	return ret;
}

static int apds990x_set_atime(struct i2c_client *client, int atime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s atime[%d]\n",__func__,atime);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);

	data->atime = atime;

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return ret;
}

static int apds990x_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s ptime[%d]\n",__func__,ptime);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return ret;
}

static int apds990x_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s wtime[%d]\n",__func__,wtime);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return ret;
}

static int apds990x_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s threshold[%d]\n",__func__,threshold);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_AILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->ailt = threshold;

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return ret;
}

static int apds990x_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s threshold[%d]\n",__func__,threshold);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_AIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->aiht = threshold;

	return ret;
}

static int apds990x_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s threshold[%d]\n",__func__,threshold);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_PILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->pilt = threshold;

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return ret;
}

static int apds990x_set_piht(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s threshold[%d]\n",__func__,threshold);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_PIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->piht = threshold;

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return ret;
}

static int apds990x_set_pers(struct i2c_client *client, int pers)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s pers[%d]\n",__func__,pers);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	data->pers = pers;

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return ret;
}

static int apds990x_set_config(struct i2c_client *client, int config)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s config[%d]\n",__func__,config);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_CONFIG_REG, config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds990x_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s ppcount[%d]\n",__func__,ppcount);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_PPCOUNT_REG, ppcount);
	mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return ret;
}

static int apds990x_set_control(struct i2c_client *client, int control)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s control[%d]\n",__func__,control);
	mutex_lock(&data->update_lock);
	ret = apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_CONTROL_REG, control);
	mutex_unlock(&data->update_lock);

	data->control = control;

	if ((data->control&0x03) == 0x00)
		data->als_gain = APDS990X_ALS_GAIN_1X;
	else if ((data->control&0x03) == 0x01)
		data->als_gain = APDS990X_ALS_GAIN_8X;
	else if ((data->control&0x03) == 0x02)
		data->als_gain = APDS990X_ALS_GAIN_16X;
	else
		data->als_gain = APDS990X_ALS_GAIN_120X;

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return ret;
}

static int LuxCalculation(struct i2c_client *client, int cdata, int irdata)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int luxValue=0;

	int IAC1=0;
	int IAC2=0;
	int IAC=0;
	int colvar = guc_nv_prox_photo_colvar[0];
	int GA=guc_nv_photo_sensor_ga[colvar];
	int COE_B=guc_nv_photo_sensor_b[colvar];
	int COE_C=guc_nv_photo_sensor_c[colvar];
	int COE_D=guc_nv_photo_sensor_d[colvar];
	int DF=52;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s cdata[%d] irdata[%d]\n",__func__,cdata,irdata);

	IAC1 = ((cdata*1000 - (COE_B*irdata))/1000);
	IAC2 = (((COE_C*cdata) - (COE_D*irdata))/1000);

	if (IAC1 > IAC2)
		IAC = IAC1;
	else if (IAC1 <= IAC2)
		IAC = IAC2;
	else
		IAC = 0;

	luxValue = (IAC*GA)/(2720/DF*(256-data->atime)*data->als_gain);

	APDS_DEBUG_LOG("[OUT]%s luxValue=%d\n",__func__,luxValue);
	return luxValue;
}

static void apds990x_reschedule_work(struct apds990x_data *data,
                      unsigned long delay)
{
	unsigned long flags;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s delay[%ld]\n",__func__,delay);

	spin_lock_irqsave(&data->update_lock.wait_lock, flags);

	__cancel_delayed_work(&data->dwork);
	queue_delayed_work(ps_polling_wq,&data->dwork, delay);

	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static void apds990x_change_als_gain(struct i2c_client *client, uint32_t als_gain)
{
	int32_t enable;

	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	APDS_DEBUG_LOG("%s: als_gain = %d\n", __func__, als_gain);
	if(als_gain == APDS990X_ALS_GAIN_1X)
	{
		apds990x_set_control(client, 0x20);
	}
	else if(als_gain == APDS990X_ALS_GAIN_8X)
	{
		apds990x_set_control(client, 0x21);
	}
	else if(als_gain == APDS990X_ALS_GAIN_16X)
	{
		apds990x_set_control(client, 0x22);
	}
	else
	{
		apds990x_set_control(client, 0x23);
	}

	enable = apds990x_reg_read(client, TYPE_BYTE_READ,APDS990x_ENABLE_REG);
	apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ENABLE_REG, enable&0x000000FE);
	mdelay(1);
	apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ENABLE_REG, enable);
	msleep(260);
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static void apds990x_put_luxValue( struct apds990x_data *data, uint32_t luxValue )
{
	uint32_t cnt = 1;
	uint32_t mean_temp = 0;
	uint32_t mean_times = 0;
	
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	if( data == NULL )
	{
		APDS_DEBUG_LOG("[OUT]%s\n",__func__);
		return;
	}
	
	if( data->als_polling_cnt_reset )
	{
		APDS_DEBUG_LOG("%s: als_polling_cnt_reset\n", __func__);
		data->als_polling_cnt_reset = ALS_POLLING_CNT_RESET_NONE;
		data->als_lux_ave = luxValue;
		data->als_polling_cnt = 0;
		
		input_report_abs(data->input_dev_als, ABS_MISC, data->als_lux_ave);
		input_sync(data->input_dev_als);
	}
	else
	{
		data->luxValue_table[data->als_polling_cnt] = luxValue;
		data->als_polling_cnt++;
		APDS_DEBUG_LOG("%s: als_polling_cnt = %d\n", __func__, data->als_polling_cnt);
		
		if( data->als_polling_cnt >= data->als_mean_times )
		{
			for( cnt = 0; cnt < data->als_mean_times; cnt++ )
			{
				APDS_DEBUG_LOG("%s: luxValue_table[%d]=%d\n", __func__, cnt, data->luxValue_table[cnt]);
				mean_temp += data->luxValue_table[cnt] * ( cnt + 1 );
				mean_times += cnt + 1;
			}
			data->als_lux_ave = mean_temp / mean_times;
			data->als_polling_cnt = 0;
			input_report_abs(data->input_dev_als, ABS_MISC, data->als_lux_ave);
			input_sync(data->input_dev_als);
		}
	}
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static void apds990x_als_polling_work_handler(struct work_struct *work)
{
	struct apds990x_data *data = container_of(work, struct apds990x_data, als_dwork.work);
	struct i2c_client *client=data->client;
	int luxValue=0;

	if(data->enable_als_sensor <= 0){
		return;
	}
	
	data->cdata  = apds990x_reg_read(client, TYPE_WORD_READ,APDS990x_CDATAL_REG);
	data->irdata = apds990x_reg_read(client, TYPE_WORD_READ,APDS990x_IRDATAL_REG);
/*	data->pdata  = apds990x_reg_read(client, TYPE_WORD_READ,APDS990x_PDATAL_REG); */
	
	luxValue = LuxCalculation(client, data->cdata, data->irdata);
	
	luxValue = luxValue>0 ? luxValue : 0;
	luxValue = luxValue<APDS990X_LUXVALUE_MAX ? luxValue : APDS990X_LUXVALUE_MAX;
	
	if(data->enable_ps_sensor>0)
	{
		if( (data->cdata >= 0xFFFF) || (data->irdata >= 0xFFFF) )
		{
			luxValue = APDS990X_LUXVALUE_MAX;
			data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_MAX_DATA;
		}
	}
	else if(data->als_gain == APDS990X_ALS_GAIN_1X)
	{
		if( (data->cdata >= 0xFFFF) || (data->irdata >= 0xFFFF) )
		{
			luxValue = APDS990X_LUXVALUE_MAX;
			data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_MAX_DATA;
		}
		else if( (data->cdata < 0x0900) && (data->irdata < 0x0900) )
		{
			apds990x_change_als_gain(client, APDS990X_ALS_GAIN_8X);
		}
		else
		{
			/* nop */
		}
	}
	else
	{
		if( (data->cdata >= 0x5D00) || (data->irdata >= 0x5D00) )
		{
			apds990x_change_als_gain(client, APDS990X_ALS_GAIN_1X);
			
			if( (data->cdata >= 0xFFFF) || (data->irdata >= 0xFFFF) )
			{
				queue_delayed_work(als_polling_wq,&data->als_dwork,msecs_to_jiffies(data->als_poll_delay));
				return;
			}
		}
	}
	
	apds990x_put_luxValue( data, (uint32_t)luxValue );
	
	APDS_DEBUG_LOG("%s: lux = %d lux_mean = %d cdata = %x  irdata = %x pdata = %x \n", __func__, luxValue, data->als_lux_ave, data->cdata, data->irdata, data->pdata);
	
	queue_delayed_work(als_polling_wq,&data->als_dwork,msecs_to_jiffies(data->als_poll_delay));
}

static void apds990x_work_handler(struct work_struct *work)
{
	struct apds990x_data *data = container_of(work, struct apds990x_data, dwork.work);
	struct i2c_client *client=data->client;
	int enable;
	int cdata;
	uint32_t dev_status_tmp = 0;

	enable = apds990x_reg_read(client, TYPE_BYTE_READ,APDS990x_ENABLE_REG);

	apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ENABLE_REG, enable&0x0000000F);
	apds990x_reg_write(client, TYPE_SP_WRITE,0x07,0);

	cdata = apds990x_reg_read(client, TYPE_WORD_READ,APDS990x_CDATAL_REG);
	data->ps_data = apds990x_reg_read(client, TYPE_WORD_READ,APDS990x_PDATAL_REG);

	APDS_DEBUG_LOG("%s: enable = %x cdata = %x ps_data = %x\n", __func__, enable, cdata, data->ps_data);

	if( data->ps_detection )
	{
		if( ((data->ps_data < data->pilt) && (data->ps_data < data->piht)) ||
			( cdata >= guc_nv_photo_sensor_beamish[0] ) )
		{
			data->ps_detection = 0;
			input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
			input_sync(data->input_dev_ps);

			wake_lock_timeout( &apds_wake_lock_input, APDS_WAKE_LOCK_INPUT_TIME );

			apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_AILTL_REG, guc_nv_photo_sensor_dark[0]);
			apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_AIHTL_REG, 0xFFFF);
			apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_PILTL_REG, 0);
			apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_PIHTL_REG, data->ps_threshold);

			data->pilt = 0;
			data->piht = data->ps_threshold;

			APDS_DEBUG_LOG("near-to-far detected\n");
			apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ENABLE_REG, 0x2F);
		}
		else
		{
			APDS_DEBUG_LOG("near-keep\n");
			apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ENABLE_REG, 0x3F);
		}
	}
	else
	{
		if( ( cdata >= guc_nv_photo_sensor_beamish[0] ) )
		{
			APDS_DEBUG_LOG("far-keep Ch0 saturation\n");
			apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ENABLE_REG, 0x1F);
		}
		else
		{
			if( (data->ps_data > data->pilt) && (data->ps_data > data->piht) )
			{
				data->ps_detection = 1;

				input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);
				input_sync(data->input_dev_ps);

				apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_AILTL_REG, 0x0000);
				apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_AIHTL_REG, guc_nv_photo_sensor_beamish[0]);
				apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_PILTL_REG, data->ps_hysteresis_threshold);
				apds990x_reg_write(client, TYPE_WORD_WRITE,APDS990x_PIHTL_REG, 1023);

				data->pilt = data->ps_hysteresis_threshold;
				data->piht = 1023;

				APDS_DEBUG_LOG("far-to-near detected\n");

				apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ENABLE_REG, 0x3F);
			}
			else
			{
				APDS_DEBUG_LOG("far-keep\n");
				apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ENABLE_REG, 0x2F);
			}
		}
	}
	

	dev_status_tmp = atomic_read(&g_dev_status);

	if(!(dev_status_tmp & APDS990X_DEV_STATUS_SUSPEND_INT) &&
		(dev_status_tmp & APDS990X_DEV_STATUS_RESUME)){

	}else{
		enable_irq(data->ps_irq);
	}

	dev_status_tmp &= ~(APDS990X_DEV_STATUS_RESUME | APDS990X_DEV_STATUS_SUSPEND_INT);
	atomic_set(&g_dev_status, dev_status_tmp );

}

static irqreturn_t apds990x_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct apds990x_data *data = i2c_get_clientdata(client);
	uint32_t dev_status = 0;

	APDS_DEBUG_LOG("==> apds990x_interrupt\n");

	disable_irq_nosync(data->ps_irq);
	dev_status = atomic_read(&g_dev_status);
	if( dev_status & APDS990X_DEV_STATUS_SUSPEND )
	{
		APDS_DEBUG_LOG("%s dev_status=%d\n",__func__,dev_status);
		APDS_DEBUG_LOG("%s set_status=%d\n",__func__,APDS990X_DEV_STATUS_SUSPEND_INT);
		atomic_set(&g_dev_status, dev_status|APDS990X_DEV_STATUS_SUSPEND_INT);
        wake_lock_timeout( &apds_wake_lock, APDS_WAKE_LOCK_TIME );
	} else 
	{
		apds990x_reschedule_work(data, 0);
	}

	APDS_DEBUG_LOG("[OUT]%s \n",__func__);
	return IRQ_HANDLED;
}

static int32_t apds990x_ps_irq_cnt = 0;
static void apds990x_enable_ps_irq( struct apds990x_data *data )
{
	APDS_DEBUG_LOG("[IN]%s apds990x_ps_irq_cnt=%d\n",__func__,apds990x_ps_irq_cnt);
	if( apds990x_ps_irq_cnt <= 0 )
	{
		APDS_DEBUG_LOG("enable_irq\n");
		enable_irq(data->ps_irq);
		apds990x_ps_irq_cnt++;
	}
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}
static void apds990x_disable_ps_irq( struct apds990x_data *data )
{
	APDS_DEBUG_LOG("[IN]%s apds990x_ps_irq_cnt=%d\n",__func__,apds990x_ps_irq_cnt);
	if( apds990x_ps_irq_cnt > 0 )
	{
		apds990x_ps_irq_cnt--;
		disable_irq(data->ps_irq);
		APDS_DEBUG_LOG("disable_irq\n");
	}
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}


static ssize_t apds990x_show_enable_ps_sensor(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	APDS_DEBUG_LOG("[IN]%s \n",__func__);
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

void apds990x_enable_ps_sensor(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
#if APDS_DEBUG
	int32_t cnt;
#endif
	int colvar = guc_nv_prox_photo_colvar[0];

	APDS_DEBUG_LOG("[IN]%s\n",__func__);

	if (data->enable_als_sensor<=0) {
		apds990x_set_enable(client,0);
		apds990x_set_atime(client, 0xC0);
		apds990x_set_ptime(client, 0xff);
		apds990x_set_ppcount(client, 0x08);
		apds990x_set_pers(client, 0x12);
		apds990x_set_config(client, 0x00);
	}

	apds990x_set_wtime(client, 0xE8);
	apds990x_set_ailt(client, guc_nv_photo_sensor_dark[0]);
	apds990x_set_aiht(client, 0xffff);
	apds990x_set_pilt(client, 0);
	apds990x_set_piht(client, guc_nv_proximity_sensor_near[colvar]);

	if( data->als_gain == APDS990X_ALS_GAIN_1X )
	{
		apds990x_set_control(client, 0x20);
	}
	else
	{
		apds990x_set_control(client, 0x20);
		msleep(250);
	}

	data->ps_threshold = guc_nv_proximity_sensor_near[colvar];
	data->ps_hysteresis_threshold = guc_nv_proximity_sensor_far[colvar];

	data->ps_detection = 0;

	apds990x_set_enable(client, 0x2F);

#if APDS_DEBUG
	for(cnt=APDS990x_ENABLE_REG;cnt<=APDS990x_PDATAH_REG;cnt++)
		APDS_DEBUG_LOG("[REG]addr=0x%x val=0x%x\n",cnt,
			apds990x_reg_read(client, TYPE_BYTE_READ,cnt));
#endif

	apds990x_enable_ps_irq(data);

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static ssize_t apds990x_store_enable_ps_sensor(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	unsigned long flags;
	
	APDS_DEBUG_LOG("%s: enable ps senosr ( %ld)\n", __func__, val);
	
	if ((val != 0) && (val != 1)) {
		APDS_DEBUG_LOG("%s:store unvalid value=%ld\n", __func__, val);
		return count;
	}
	
	if(val == 1) {
		if (data->enable_ps_sensor<=0) {

			data->enable_ps_sensor=1;

			apds990x_enable_ps_sensor(client);
		} else
		{
			data->enable_ps_sensor++;
		}
	} 
	else {
		data->enable_ps_sensor--;
		
		if (data->enable_ps_sensor>0) {
			;
		} else if(data->enable_als_sensor) {
			
			apds990x_disable_ps_irq(data);
			apds990x_set_enable(client, 0x0A);
			apds990x_reg_write(client, TYPE_SP_WRITE,0x07,0);
			apds990x_set_wtime(client, 0xE6);
			apds990x_set_enable(client, 0xB);
			
			if( data->ps_detection != 0 )
			{
				data->ps_detection = 0;
				input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
				input_sync(data->input_dev_ps);
			}

			spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
			__cancel_delayed_work(&data->als_dwork);
			queue_delayed_work(als_polling_wq,&data->als_dwork,msecs_to_jiffies(data->als_poll_delay));
			
			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);	
		}
		else {
			apds990x_disable_ps_irq(data);
			apds990x_reg_write(client, TYPE_BYTE_WRITE,APDS990x_ENABLE_REG, 0x08);
			apds990x_reg_write(client, TYPE_SP_WRITE,0x07,0);
			apds990x_set_wtime(client, 0xE6);
			apds990x_set_enable(client, 0);

			if( data->ps_detection != 0 )
			{
				data->ps_detection = 0;
				input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
				input_sync(data->input_dev_ps);
			}
		}
	}
	
	
	return count;
}

static DEVICE_ATTR(enable_ps_sensor, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
                   apds990x_show_enable_ps_sensor, apds990x_store_enable_ps_sensor);

static ssize_t apds990x_show_enable_als_sensor(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	APDS_DEBUG_LOG("[IN]%s \n",__func__);
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	
	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

void apds990x_enable_als_sensor(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long flags;
#if APDS_DEBUG
	int32_t cnt;
#endif

	APDS_DEBUG_LOG("[IN]%s\n",__func__);

	if (data->enable_ps_sensor<=0) {
		apds990x_set_enable(client,0);
		apds990x_set_atime(client, 0xC0);
		apds990x_set_ptime(client, 0xff);
		apds990x_set_wtime(client, 0xE6);
		apds990x_set_ppcount(client, 0x08);
		apds990x_set_control(client, 0x20);
		apds990x_set_pers(client, 0x12);
		apds990x_set_config(client, 0x00);
		apds990x_set_enable(client, 0x0B);
	}
#if APDS_DEBUG
	for(cnt=APDS990x_ENABLE_REG;cnt<=APDS990x_PDATAH_REG;cnt++)
		APDS_DEBUG_LOG("[REG]addr=0x%x val=0x%x\n",cnt,
			apds990x_reg_read(client, TYPE_BYTE_READ,cnt));
#endif

	input_report_abs(data->input_dev_als, ABS_MISC, ABS_MISC_DUMMY_VAL);

	spin_lock_irqsave(&data->update_lock.wait_lock, flags); 

	__cancel_delayed_work(&data->als_dwork);
	queue_delayed_work(als_polling_wq,&data->als_dwork,msecs_to_jiffies(data->als_poll_delay+150));

	APDS_DEBUG_LOG("delay[ms]=%d, jiffies=%ld\n",data->als_poll_delay, msecs_to_jiffies(data->als_poll_delay));

	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static ssize_t apds990x_store_enable_als_sensor(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	unsigned long flags;
	
	APDS_DEBUG_LOG("%s: enable als sensor ( %ld)\n", __func__, val);
	
	if ((val != 0) && (val != 1))
	{
		APDS_DEBUG_LOG("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}
	
	if(val == 1) {
		if (data->enable_als_sensor<=0) {

			data->enable_als_sensor = 1;
			apds990x_enable_als_sensor(client);
		}
		else
			data->enable_als_sensor++;
	}
	else {
		data->enable_als_sensor--;
		if( data->enable_als_sensor > 0)
			;
		else {
			if (data->enable_ps_sensor<=0) {
				apds990x_set_enable(client, 0);
			}
		
			spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
			
			__cancel_delayed_work(&data->als_dwork);
			
			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags); 
			
			data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_DISABLE;
		}
	}
	
	return count;
}

static DEVICE_ATTR(enable_als_sensor, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
                   apds990x_show_enable_als_sensor, apds990x_store_enable_als_sensor);

static ssize_t apds990x_show_als_poll_delay(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->als_poll_delay*1000);
}

static ssize_t apds990x_store_als_poll_delay(struct device *dev,
                    struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	unsigned long flags;
	
	if( val < (1000 / APDS990X_LUXVALUE_TABLE_MAX) * 1000 )
		val = (1000 / APDS990X_LUXVALUE_TABLE_MAX) * 1000;
	
	data->als_poll_delay = val/1000;
	
	data->als_mean_times = 1000 / data->als_poll_delay;
	data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_STORE_POLL;

	spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
		
	__cancel_delayed_work(&data->als_dwork);
	queue_delayed_work(als_polling_wq,&data->als_dwork,msecs_to_jiffies(data->als_poll_delay));

	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);	
	
	return count;
}

static DEVICE_ATTR(als_poll_delay, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
                   apds990x_show_als_poll_delay, apds990x_store_als_poll_delay);

static struct attribute *apds990x_attributes[] = {
    &dev_attr_enable_ps_sensor.attr,
    &dev_attr_enable_als_sensor.attr,
    &dev_attr_als_poll_delay.attr,
    NULL
};


static const struct attribute_group apds990x_attr_group = {
    .attrs = apds990x_attributes,
};

static void apds990x_store_als_mean_times( struct apds990x_data *data, uint32_t mean_times )
{
	unsigned long flags;

	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	if( mean_times == 0 )
	{
		APDS_DEBUG_LOG("%s: bad param\n", __func__);
		return;
	}
	else if( mean_times > APDS990X_LUXVALUE_TABLE_MAX )
	{
		data->als_mean_times = APDS990X_LUXVALUE_TABLE_MAX;
	}
	else
	{
		data->als_mean_times = mean_times;
	}
	data->als_poll_delay = 1000 / data->als_mean_times;
	data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_STORE_TIME;
	
	spin_lock_irqsave(&data->update_lock.wait_lock, flags); 

	__cancel_delayed_work(&data->als_dwork);
	queue_delayed_work(als_polling_wq,&data->als_dwork,msecs_to_jiffies(data->als_poll_delay));
	
	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);	
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static void apds990x_set_sensor_nv( unsigned long ulArg )
{
	T_APDS990X_IOCTL_NV* nv_data_type 
							= (T_APDS990X_IOCTL_NV*)ulArg;
	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	switch(nv_data_type->ulItem)
	{
	case en_NV_PROXIMITY_SENSOR_NEAR_I:
		APDS_DEBUG_LOG("%s en_NV_PROXIMITY_SENSOR_NEAR_I\n",__func__);
		gun_nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_NEAR_I);
		memcpy(guc_nv_proximity_sensor_near,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		break;
	case en_NV_PROXIMITY_SENSOR_FAR_I:
		APDS_DEBUG_LOG("%s en_NV_PROXIMITY_SENSOR_FAR_I\n",__func__);
		gun_nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_FAR_I);
		memcpy(guc_nv_proximity_sensor_far,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		break;
	case en_NV_PHOTO_SENSOR_BEAMISH_I:
		APDS_DEBUG_LOG("%s en_NV_PHOTO_SENSOR_BEAMISH_I\n",__func__);
		gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_BEAMISH_I);
		memcpy(guc_nv_photo_sensor_beamish,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		break;
	case en_NV_PHOTO_SENSOR_DARK_I:
		APDS_DEBUG_LOG("%s en_NV_PHOTO_SENSOR_DARK_I\n",__func__);
		gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_DARK_I);
		memcpy(guc_nv_photo_sensor_dark,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		break;
	case en_NV_PHOTO_SENSOR_B_I:
		APDS_DEBUG_LOG("%s en_NV_PHOTO_SENSOR_B_I\n",__func__);
		gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_I);
		memcpy(guc_nv_photo_sensor_b,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		break;
	case en_NV_PHOTO_SENSOR_C_I:
		APDS_DEBUG_LOG("%s en_NV_PHOTO_SENSOR_C_I\n",__func__);
		gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_C_I);
		memcpy(guc_nv_photo_sensor_c,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		break;
	case en_NV_PHOTO_SENSOR_D_I:
		APDS_DEBUG_LOG("%s en_NV_PHOTO_SENSOR_D_I\n",__func__);
		gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_D_I);
		memcpy(guc_nv_photo_sensor_d,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		break;
	case en_NV_PHOTO_SENSOR_GA_I:
		APDS_DEBUG_LOG("%s en_NV_PHOTO_SENSOR_GA_I\n",__func__);
		gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_GA_I);
		memcpy(guc_nv_photo_sensor_ga,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		break;
	case en_NV_PROX_PHOTO_COLVAR_I:
		APDS_DEBUG_LOG("%s en_NV_PROX_PHOTO_COLVAR_I\n",__func__);
		gun_nv_status |= (0x01<<en_NV_PROX_PHOTO_COLVAR_I);
		memcpy(guc_nv_prox_photo_colvar,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		if(guc_nv_prox_photo_colvar[0] >= APDS990X_COLOR_VARI)
		{
			guc_nv_prox_photo_colvar[0] = 0;
		}
		break;
	default :
		APDS_DEBUG_LOG(KERN_ERR "set_sensor_nv: Can't set nv data\n");
		break;
	}
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static int apds990x_open(struct inode *inode_type, struct file *file)
{
	APDS_DEBUG_LOG("[IN]apds990x_open\n");
	APDS_DEBUG_LOG("[OUT]apds990x_open\n");
	return 0;
}

static int apds990x_release(struct inode *inode_type, struct file *file)
{
	APDS_DEBUG_LOG("[IN]apds990x_release\n");
	APDS_DEBUG_LOG("[OUT]apds990x_release\n");
	return 0;
}

static long apds990x_ioctl(struct file *file_type, 
                          unsigned int unCmd, unsigned long ulArg)
{
	int32_t nRet = -EINVAL;
	T_APDS990X_IOCTL_PS_DETECTION ps_detection_type;
	T_APDS990X_IOCTL_ALS_MEAN_TIMES als_mean_times_type;
	T_APDS990X_IOCTL_ALS_LUX_AVE als_lux_ave_type;
	struct apds990x_data *data = i2c_get_clientdata(client_apds);

	APDS_DEBUG_LOG("[IN]apds990x_ioctl\n");
	
	memset((void*)&ps_detection_type, 0,
						sizeof(T_APDS990X_IOCTL_PS_DETECTION) );
	memset((void*)&als_mean_times_type, 0,
						sizeof(T_APDS990X_IOCTL_ALS_MEAN_TIMES) );
	memset((void*)&als_lux_ave_type, 0,
						sizeof(T_APDS990X_IOCTL_ALS_LUX_AVE) );
	switch( unCmd )
	{
		case IOCTL_PS_DETECTION_GET:
			APDS_DEBUG_LOG("IOCTL_PS_DETECTION_GET START\n" );
			nRet = copy_from_user(&ps_detection_type, 
					(void __user *)ulArg, sizeof(T_APDS990X_IOCTL_PS_DETECTION) );
			if (nRet) {
				APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_PS_DETECTION_GET)\n" );
				return -EFAULT;
			}
			ps_detection_type.ulps_detection = data->ps_detection;
			nRet = copy_to_user((void *)(ulArg),
					 &ps_detection_type, sizeof(T_APDS990X_IOCTL_PS_DETECTION) );
			if (nRet) {
				APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_PS_DETECTION_GET)\n" );
				return -EFAULT;
			}
			APDS_DEBUG_LOG("IOCTL_PS_DETECTION_GET END\n" );
			break;
		case IOCTL_ALS_MEAN_TIMES_SET:
			APDS_DEBUG_LOG("IOCTL_ALS_MEAN_TIMES_SET START\n" );
			nRet = copy_from_user(&als_mean_times_type, 
					(void __user *)ulArg, sizeof(T_APDS990X_IOCTL_ALS_MEAN_TIMES) );
			if (nRet) {
				APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_ALS_MEAN_TIMES_SET)\n" );
				return -EFAULT;
			}
			apds990x_store_als_mean_times( data, als_mean_times_type.ulals_mean_times );
			nRet = copy_to_user((void *)(ulArg),
					 &als_mean_times_type, sizeof(T_APDS990X_IOCTL_ALS_MEAN_TIMES) );
			if (nRet) {
				APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_ALS_MEAN_TIMES_SET)\n" );
				return -EFAULT;
			}
			break;
		case IOCTL_ALS_LUX_AVE_GET:
			APDS_DEBUG_LOG("IOCTL_ALS_LUX_AVE_GET START\n" );
			nRet = copy_from_user(&als_lux_ave_type, 
					(void __user *)ulArg, sizeof(T_APDS990X_IOCTL_ALS_LUX_AVE) );
			if (nRet) {
				APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_ALS_LUX_AVE_GET)\n" );
				return -EFAULT;
			}
			als_lux_ave_type.ulals_lux_ave = data->als_lux_ave;
			als_lux_ave_type.lcdata = data->cdata;
			als_lux_ave_type.lirdata = data->irdata;
			nRet = copy_to_user((void *)(ulArg),
					 &als_lux_ave_type, sizeof(T_APDS990X_IOCTL_ALS_LUX_AVE) );
			if (nRet) {
				APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_ALS_LUX_AVE_GET)\n" );
				return -EFAULT;
			}
			break;
		case IOCTL_APDS990X_NV_DATA_SET:
			{
				T_APDS990X_IOCTL_NV sensor_nv_type;
				APDS_DEBUG_LOG("IOCTL_APDS990X_NV_DATA_SET START\n" );
				memset((void*)&sensor_nv_type, 0,
							sizeof(T_APDS990X_IOCTL_NV) );
				nRet = copy_from_user(&sensor_nv_type, 
						(void __user *)ulArg, sizeof(T_APDS990X_IOCTL_NV) );
				if(!nRet)
				{
					apds990x_set_sensor_nv((unsigned long)&sensor_nv_type);
					nRet = 0;
				}
			}
			break;
		default:
			APDS_DEBUG_LOG("default err\n" );
			break;
	}
	APDS_DEBUG_LOG("[OUT]apds990x_ioctl nRet=%d\n",nRet);
	return nRet;
}

static struct file_operations apds990x_fops = {
    .owner = THIS_MODULE,
    .open = apds990x_open,
    .release = apds990x_release,
    .unlocked_ioctl = apds990x_ioctl,
};

static struct miscdevice apds990x_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = APDS990x_DRV_NAME,
    .fops = &apds990x_fops,
};
static int apds990x_init_client2(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int err;
	int id;
	int colvar = guc_nv_prox_photo_colvar[0];

	APDS_DEBUG_LOG("[IN]%s\n",__func__);

	err = apds990x_set_enable(client, 0);

	if (err < 0)
	{
		APDS_DEBUG_LOG("Failed apds990x_set_enable\n");
		return err;
	}
	
	id = apds990x_reg_read(client, TYPE_BYTE_READ,APDS990x_ID_REG);
	if (id == 0x20) {
		APDS_DEBUG_LOG("APDS-9901\n");
	}
	else if (id == 0x29) {
		APDS_DEBUG_LOG("APDS-990x\n");
	}
	else {
		APDS_DEBUG_LOG("Neither APDS-9901 nor APDS-990x id=0x%x\n",id);
		return -EIO;
	}

	apds990x_set_atime(client, 0xC0);
	apds990x_set_ptime(client, 0xFF);
	apds990x_set_wtime(client, 0xE6);

	apds990x_set_ppcount(client, 0x08);
	apds990x_set_config(client, 0);
	apds990x_set_control(client, 0x20);

	apds990x_set_pilt(client, 0);
	apds990x_set_piht(client, guc_nv_proximity_sensor_near[colvar]);

	data->ps_threshold = guc_nv_proximity_sensor_near[colvar];
	data->ps_hysteresis_threshold = guc_nv_proximity_sensor_far[colvar];

	apds990x_set_ailt(client, 0);
	apds990x_set_aiht(client, 0xFFFF);

	apds990x_set_pers(client, 0x12);

	err = apds990x_set_enable(client, 0);

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);

	return 0;
}

static int apds990x_init_client(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int err;
	int id;
	int colvar = guc_nv_prox_photo_colvar[0];

	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	data->enable_ps_sensor = 1;
	data->enable_als_sensor = 1;
	err = apds990x_set_enable(client, 0);

	if (err < 0)
	{
		APDS_DEBUG_LOG("Failed apds990x_set_enable\n");
		return err;
	}
	
	id = apds990x_reg_read(client, TYPE_BYTE_READ,APDS990x_ID_REG);
	if (id == 0x20) {
		APDS_DEBUG_LOG("APDS-9901\n");
	}
	else if (id == 0x29) {
		APDS_DEBUG_LOG("APDS-990x\n");
	}
	else {
		APDS_DEBUG_LOG("Neither APDS-9901 nor APDS-990x id=0x%x\n",id);
		return -EIO;
	}

	apds990x_set_atime(client, 0xC0);
	apds990x_set_ptime(client, 0xFF);
	apds990x_set_wtime(client, 0xE6);

	apds990x_set_ppcount(client, 0x08);
	apds990x_set_config(client, 0);
	apds990x_set_control(client, 0x20);

	apds990x_set_pilt(client, 0);
	apds990x_set_piht(client, guc_nv_proximity_sensor_near[colvar]);

	data->ps_threshold = guc_nv_proximity_sensor_near[colvar];
	data->ps_hysteresis_threshold = guc_nv_proximity_sensor_far[colvar];

	apds990x_set_ailt(client, 0);
	apds990x_set_aiht(client, 0xFFFF);

	apds990x_set_pers(client, 0x12);

	data->enable_ps_sensor = 0;
	data->enable_als_sensor = 0;
	err = apds990x_set_enable(client, 0);

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);

	return 0;
}


static struct i2c_driver apds990x_driver;
static int __devinit apds990x_probe(struct i2c_client *client,
                   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds990x_data *data;
	int err = 0;

	APDS_DEBUG_LOG("[IN]%s\n",__func__);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds990x_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

	data->enable = 0;
	data->ps_threshold = 0;
	data->ps_hysteresis_threshold = 0;
	data->ps_detection = 0;
	data->enable_als_sensor = 0;
	data->enable_ps_sensor = 0;
	data->als_poll_delay = 250;
	data->als_atime = 0xC0;
	
	memset( data->luxValue_table, 0, sizeof(data->luxValue_table) );
	data->als_lux_ave = 0;
	data->als_polling_cnt = 0;
	data->als_mean_times = 4;
	data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_INIT;
	
	apds_power_cb.power_on = apds990x_power_on;
	apds_power_cb.power_off = apds990x_power_off;
	
	APDS_DEBUG_LOG("enable = %x\n", data->enable);

	mutex_init(&data->update_lock);
    wake_lock_init( &apds_wake_lock, WAKE_LOCK_SUSPEND, "apds990x_ps" );
	wake_lock_init( &apds_wake_lock_input, WAKE_LOCK_SUSPEND, "apds990x_ps_input" );

	err = gpio_request(APDS_PROXIMITY_SENSOR_GPIO,
						APDS990x_DRV_NAME);
	APDS_DEBUG_LOG("gpio_request err=%d\n",err);
	if (err < 0) {
		APDS_DEBUG_LOG("[%s] failed to request GPIO=%d, ret=%d\n",
			   __FUNCTION__,
			   APDS_PROXIMITY_SENSOR_GPIO,
			   err);
		goto exit_kfree;
	}
	err = gpio_direction_input(APDS_PROXIMITY_SENSOR_GPIO);
	APDS_DEBUG_LOG("gpio_direction_input err=%d\n",err);
	if (err < 0) {
		APDS_DEBUG_LOG("[%s] failed to configure direction for GPIO=%d, ret=%d\n",
			   __FUNCTION__,
			   APDS_PROXIMITY_SENSOR_GPIO,
			   err);
		goto exit_kfree;
	}
	data->ps_irq = gpio_to_irq(APDS_PROXIMITY_SENSOR_GPIO);

	err = request_any_context_irq(data->ps_irq, apds990x_interrupt, IRQ_TYPE_EDGE_FALLING,
		APDS990x_DRV_NAME, (void *)client);
	APDS_DEBUG_LOG("request_any_context_irq err=%d\n",err);
	if(err < 0) {
		APDS_DEBUG_LOG("%s Could not allocate APDS990x_INT(%d) ! err=%d\n",
				 __func__,APDS_PROXIMITY_SENSOR_GPIO,err);
	
		goto exit_kfree;
	}
	apds990x_ps_irq_cnt++;
	apds990x_disable_ps_irq(data);

	INIT_DELAYED_WORK(&data->dwork, apds990x_work_handler);
	INIT_DELAYED_WORK(&data->als_dwork, apds990x_als_polling_work_handler); 

	APDS_DEBUG_LOG("%s interrupt is hooked\n", __func__);

	err = apds990x_init_client(client);
	if (err)
	{
		APDS_DEBUG_LOG("Failed apds990x_init_client\n");
		goto exit_kfree;
	}

	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		APDS_DEBUG_LOG("Failed to allocate input device als\n");
		goto exit_free_irq;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		APDS_DEBUG_LOG("Failed to allocate input device ps\n");
		goto exit_free_dev_als;
	}
	
	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, APDS990X_LUXVALUE_MAX, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_als->name = "Avago light sensor";
	data->input_dev_ps->name = "Avago proximity sensor";

	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		APDS_DEBUG_LOG("Unable to register input device als: %s\n",
			   data->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		APDS_DEBUG_LOG("Unable to register input device ps: %s\n",
			   data->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	err = sysfs_create_group(&client->dev.kobj, &apds990x_attr_group);
	if (err)
	{
		APDS_DEBUG_LOG("Failed sysfs_create_group\n");
		goto exit_unregister_dev_ps;
	}

	device_init_wakeup(&client->dev, 1);
	atomic_set(&g_dev_status, APDS990X_DEV_STATUS_INIT);
	
	err = misc_register(&apds990x_device);
	if (err)
	{
		APDS_DEBUG_LOG(KERN_ERR
			   "apds990x_probe: apds990x register failed\n");
		goto exit_sysfs_remove;
	}
	APDS_DEBUG_LOG("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);
	client_apds = client;
	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return 0;

exit_sysfs_remove:
	sysfs_remove_group(&client->dev.kobj, &apds990x_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);	
exit_unregister_dev_als:
	input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_free_dev_als:
	input_free_device(data->input_dev_als);
exit_free_irq:
	free_irq(data->ps_irq, client); 
exit_kfree:
	kfree(data);
exit:
	APDS_DEBUG_LOG("[OUT]%s err=%d\n",__func__,err);
	return err;
}

static int __devexit apds990x_remove(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	
	input_unregister_device(data->input_dev_als);
	input_unregister_device(data->input_dev_ps);
	
	input_free_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);

    wake_lock_destroy( &apds_wake_lock );
    wake_lock_destroy( &apds_wake_lock_input );

	free_irq(data->ps_irq, client);

	sysfs_remove_group(&client->dev.kobj, &apds990x_attr_group);
	misc_deregister(&apds990x_device);

	data->enable_ps_sensor = 0;
	data->enable_als_sensor = 0;
	
	apds990x_set_enable(client, 0);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM

static int apds990x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	APDS_DEBUG_LOG("[IN]%s\n", __func__);

	if(device_may_wakeup(&client->dev))
	{
		enable_irq_wake(data->ps_irq);
	} else
	{
		printk("%s:failed device_may_wakeup\n",__func__);
	}
	atomic_set(&g_dev_status, APDS990X_DEV_STATUS_SUSPEND);
	APDS_DEBUG_LOG("[OUT]%s\n", __func__);
	return 0;
}

static int apds990x_resume(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	uint32_t dev_status_tmp = 0;
	APDS_DEBUG_LOG("[IN]%s\n", __func__);

	if(device_may_wakeup(&client->dev))
	{
		disable_irq_wake(data->ps_irq);
	} else
	{
		printk("%s:failed device_may_wakeup\n",__func__);
	}
	data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_RESUME;
	
#if 1
	if (data->enable_ps_sensor > 0){
		dev_status_tmp = (atomic_read(&g_dev_status) & 
								  APDS990X_DEV_STATUS_SUSPEND_INT);
		atomic_set(&g_dev_status, APDS990X_DEV_STATUS_INIT | 
								  APDS990X_DEV_STATUS_RESUME | dev_status_tmp);

		apds990x_reschedule_work(data, 0);
	}else{
		atomic_set(&g_dev_status, APDS990X_DEV_STATUS_INIT);	
	}
#else
	if(atomic_read(&g_dev_status) & APDS990X_DEV_STATUS_SUSPEND_INT)
	{
		apds990x_reschedule_work(data, 0);
	}
#endif

/*	  dev_status_tmp = (atomic_read(&g_dev_status) & APDS990X_DEV_STATUS_SUSPEND_INT); */
/*	  atomic_set(&g_dev_status, APDS990X_DEV_STATUS_INIT | APDS990X_DEV_STATUS_RESUME | dev_status_tmp ); */
	APDS_DEBUG_LOG("[OUT]%s\n", __func__);
	return 0;
}

#else

#define apds990x_suspend    NULL
#define apds990x_resume     NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds990x_id[] = {
    { APDS990x_DRV_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, apds990x_id);

static struct i2c_driver apds990x_driver = {
    .driver = {
        .name   = APDS990x_DRV_NAME,
        .owner  = THIS_MODULE,
    },
    .suspend = apds990x_suspend,
    .resume = apds990x_resume,
    .probe  = apds990x_probe,
    .remove = __devexit_p(apds990x_remove),
    .id_table = apds990x_id,
};

static int __init apds990x_init(void)
{
	int32_t rc;
	
	APDS_DEBUG_LOG("[IN]%s\n",__func__);

    als_polling_wq = create_singlethread_workqueue("als_polling_wq");
    if(!als_polling_wq)
    {
		APDS_DEBUG_LOG("can't create queue : als_polling_wq \n");
		goto REGIST_ERR1;
    }

    ps_polling_wq = create_singlethread_workqueue("ps_polling_wq");
    if(!ps_polling_wq)
    {
		APDS_DEBUG_LOG("can't create queue : ps_polling_wq \n");
		goto REGIST_ERR2;
    }

	rc = i2c_add_driver(&apds990x_driver);
	if (rc != 0) {
		APDS_DEBUG_LOG("can't add i2c driver\n");
		goto REGIST_ERR3;
	}

	APDS_DEBUG_LOG("[OUT]%s\n",__func__);
	return rc;

REGIST_ERR3:
    if(ps_polling_wq != NULL){
        flush_workqueue(ps_polling_wq);
        destroy_workqueue(ps_polling_wq);
        ps_polling_wq = NULL;
    }

REGIST_ERR2:
    if(als_polling_wq != NULL){
        flush_workqueue(als_polling_wq);
        destroy_workqueue(als_polling_wq);
        als_polling_wq = NULL;
    }

REGIST_ERR1:
    return -ENOTSUPP;
}

static void __exit apds990x_exit(void)
{
    if(als_polling_wq != NULL){
        flush_workqueue(als_polling_wq);
        destroy_workqueue(als_polling_wq);
        als_polling_wq = NULL;
    }

    if(ps_polling_wq != NULL){
        flush_workqueue(ps_polling_wq);
        destroy_workqueue(ps_polling_wq);
        ps_polling_wq = NULL;
    }

	i2c_del_driver(&apds990x_driver);
	
	i2c_unregister_device(client_apds);
	client_apds = NULL;
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS990x ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds990x_init);
module_exit(apds990x_exit);

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
#include <linux/module.h>
//#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <mach/hs_io_ctl_a.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/sensor_power.h>
#undef SENSOR_LOG_ON

#ifdef SENSOR_LOG_ON
#define SENSOR_LOG_I(arg...) printk(KERN_INFO "[SP]"arg)
#define SENSOR_LOG_D(arg...) printk("[SP]"arg)
#else
#define SENSOR_LOG_I(arg...)
#define SENSOR_LOG_D(arg...)
#endif


#define VSENSOR_GPIO				(56)
#define GPIO_ON						(1)
#define GPIO_OFF					(0)
#define SENSOR_INIT_VSENSOR			0x00000001
#define SENSOR_INIT_DONE			0x00000002
#define SENSOR_INIT_ACC_ON			0x00000100
#define SENSOR_INIT_MAG_ON			0x00000200
#define SENSOR_INIT_GYRO_ON			0x00000400
#define SENSOR_INIT_ORI_ON			0x00000800
#define SENSOR_INIT_APDS			0x00001000
#define SENSOR_INIT_DEVICE			0x0000FF00


static spinlock_t sensor_power_spin_lock;
static struct mutex sensor_power_mutex_lock;
static uint32_t sensor_init_status=0;

struct sensor_power_callback* sensor_power_cb_tbl[SENSOR_INDEX_DEVICE_MAX];

#define SENSOR_INIT_STATUS_SET(status,val) \
	if(val) {\
		sensor_init_status |= status;\
	} else {\
		sensor_init_status &= ~(status);\
	}

#define SENSOR_INIT_STATUS_GET(status) \
	(sensor_init_status&(status))


static void kc_sensor_power_on_cbfunc(void);
static void kc_sensor_power_off_cbfunc(void);

/* -------------------------------------------------------------------------- */
/*  Local functions                                                           */
/* -------------------------------------------------------------------------- */
void kc_sensor_power_off( void )
{
	int32_t rc;
//	int32_t gpio_ret;
	struct regulator *vpro_vreg;
	
	SENSOR_LOG_I("%s(): start\n",__func__);
	kc_sensor_power_off_cbfunc();
	vpro_vreg = regulator_get(NULL, "8921_lvs6");
	if (IS_ERR(vpro_vreg)) {
		pr_err("%s: regulator_get(%s) failed (%ld)\n",
			__func__, "gp11", PTR_ERR(vpro_vreg));
	} else {
		regulator_set_voltage(vpro_vreg, 1800000,1800000);
		rc = regulator_disable(vpro_vreg);
//		msleep(5);
		msleep(10);
//		gpio_set_value_cansleep(VSENSOR_GPIO, GPIO_OFF);
//		gpio_ret = gpio_get_value(VSENSOR_GPIO);
//		SENSOR_LOG_D("%s(): regulator_disable rc=%d , VSENSOR_GPIO=%d\n",__func__,rc,gpio_ret);
		SENSOR_LOG_D("%s(): regulator_disable rc=%d \n",__func__,rc);
//		msleep(20);
		if (!rc) {
			SENSOR_LOG_D("%s():i2c reset done\n" ,__func__);
			SENSOR_INIT_STATUS_SET(SENSOR_INIT_DONE,0);
		}
		SENSOR_INIT_STATUS_SET(SENSOR_INIT_VSENSOR,0);
	}
	SENSOR_LOG_I("%s(): end\n",__func__);
}

void sensor_power_on(enum sensor_index id)
{
	int32_t rc;
//	int32_t gpio_ret;
	struct regulator *vpro_vreg;

	SENSOR_LOG_I("%s(): start\n",__func__);
	mutex_lock( &sensor_power_mutex_lock );

	SENSOR_INIT_STATUS_SET((1<<id)<<8, 1);
	
	if(!(SENSOR_INIT_STATUS_GET(SENSOR_INIT_VSENSOR))){ 
		vpro_vreg = regulator_get(NULL, "8921_lvs6");
		if (IS_ERR(vpro_vreg)) {
			pr_err("%s: regulator_get(%s) failed (%ld)\n",
			__func__, "gp11", PTR_ERR(vpro_vreg));

		} else	{
			regulator_set_voltage(vpro_vreg, 1800000,1800000);
//			gpio_set_value_cansleep(VSENSOR_GPIO, GPIO_ON);
//			gpio_ret = gpio_get_value(VSENSOR_GPIO);
			rc = regulator_enable(vpro_vreg);
//			SENSOR_LOG_D("%s(): regulator_enable rc=%d , VSENSOR_GPIO=%d\n",__func__,rc,gpio_ret);
			SENSOR_LOG_D("%s(): regulator_enable rc=%d\n",__func__,rc);
			msleep(50);
			if (!rc) {
				SENSOR_LOG_D("%s():i2c reset done\n" ,__func__);
				SENSOR_INIT_STATUS_SET(SENSOR_INIT_DONE,1);
			}
			SENSOR_INIT_STATUS_SET(SENSOR_INIT_VSENSOR,1);
			kc_sensor_power_on_cbfunc();
		}
	}
	mutex_unlock( &sensor_power_mutex_lock );
	SENSOR_LOG_I("%s(): status=0x%x end\n",__func__,sensor_init_status);
}
EXPORT_SYMBOL(sensor_power_on);

void sensor_power_off(enum sensor_index id)
{
	uint32_t devices, init_vsensor;

	SENSOR_LOG_I("%s(): start\n",__func__);
	mutex_lock( &sensor_power_mutex_lock );
	
	SENSOR_INIT_STATUS_SET((1<<id)<<8, 0);
	devices = SENSOR_INIT_STATUS_GET(SENSOR_INIT_DEVICE);
	init_vsensor = SENSOR_INIT_STATUS_GET(SENSOR_INIT_VSENSOR);
	
//	if(!devices && init_vsensor)
//		kc_sensor_power_off();
	mutex_unlock( &sensor_power_mutex_lock );
	SENSOR_LOG_I("%s(): status=0x%x end\n",__func__,sensor_init_status);
}
EXPORT_SYMBOL(sensor_power_off);


void sensor_power_reset(enum sensor_index id)
{
	SENSOR_LOG_I("%s(): start id=0x%x\n",__func__,id);
	mutex_lock( &sensor_power_mutex_lock );
	kc_sensor_power_off();
	mutex_unlock( &sensor_power_mutex_lock );
	sensor_power_on(id);
	SENSOR_LOG_I("%s(): end id=0x%x\n",__func__,id);
}
EXPORT_SYMBOL(sensor_power_reset);

int32_t sensor_power_reg_cbfunc(struct sensor_power_callback* cb)
{
	int32_t i;
	int32_t ret = -1;
	SENSOR_LOG_I("%s(): start\n",__func__);
	spin_lock(&sensor_power_spin_lock);
	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (sensor_power_cb_tbl[i] == (struct sensor_power_callback*)NULL ||
			sensor_power_cb_tbl[i] == cb) {
			sensor_power_cb_tbl[i] = cb;
			ret = 0;
			break;
		}
	}
	spin_unlock(&sensor_power_spin_lock);
	SENSOR_LOG_I("%s(): end\n",__func__);
	return ret;
}
EXPORT_SYMBOL(sensor_power_reg_cbfunc);

int32_t sensor_power_unreg_cbfunc(struct sensor_power_callback* cb)
{
	int32_t i;
	int32_t ret = -1;
	SENSOR_LOG_I("%s(): start\n",__func__);
	spin_lock(&sensor_power_spin_lock);
	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (sensor_power_cb_tbl[i] == cb) {
			sensor_power_cb_tbl[i] = NULL;
			ret = 0;
			break;
		}
		
	}
	spin_unlock(&sensor_power_spin_lock);
	SENSOR_LOG_I("%s(): end\n",__func__);
	return ret;
}
EXPORT_SYMBOL(sensor_power_unreg_cbfunc);

void kc_sensor_power_on_cbfunc(void)
{
	int32_t i;
	SENSOR_LOG_I("%s(): start\n",__func__);
	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (sensor_power_cb_tbl[i] != (struct sensor_power_callback*)NULL) {
			SENSOR_LOG_D("%s(): sensor_power_cb_tbl[%d]->power_on()\n",__func__,i);
			sensor_power_cb_tbl[i]->power_on();
		}
	}
	SENSOR_LOG_I("%s(): end\n",__func__);
	return ;
}

static void kc_sensor_power_off_cbfunc(void)
{
	int32_t i;
	SENSOR_LOG_I("%s(): start\n",__func__);
	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (sensor_power_cb_tbl[i] != (struct sensor_power_callback*)NULL) {
			SENSOR_LOG_D("%s(): sensor_power_cb_tbl[%d]->power_off()\n",__func__,i);
			sensor_power_cb_tbl[i]->power_off();
		}
	}
	SENSOR_LOG_I("%s(): end\n",__func__);
	return ;
}

static int32_t kc_sensor_power_probe(struct platform_device *dev)
{
	SENSOR_LOG_I("%s: \n",__func__);
	return 0;
}

static int32_t kc_sensor_power_remove(struct platform_device *dev)
{
	SENSOR_LOG_I("%s: \n",__func__);
	return 0;
}

static struct platform_driver sensor_power_driver = {
	.driver = {
		.name = "sensor_power",
		.owner = THIS_MODULE,
	},
	.probe = kc_sensor_power_probe,
	.remove = kc_sensor_power_remove,
};

static int32_t __init kc_sensor_power_init(void)
{
	int32_t ret = 0;

	ret = platform_driver_register(&sensor_power_driver);
	if (ret)
		goto out_region;
	SENSOR_LOG_I("%s: platform_driver_register\n",__func__);

	spin_lock_init( &sensor_power_spin_lock );
	mutex_init( &sensor_power_mutex_lock );
	memset((void*)sensor_power_cb_tbl,(int)NULL,
			sizeof(struct sensor_power_callback*)*SENSOR_INDEX_DEVICE_MAX);

	sensor_power_on(SENSOR_INDEX_ACC);
	SENSOR_INIT_STATUS_SET(SENSOR_INIT_ACC_ON|SENSOR_INIT_GYRO_ON|SENSOR_INIT_APDS,1);

	return 0;

out_region:
	return ret;
}

static void __exit kc_sensor_power_exit(void)
{
	platform_driver_unregister(&sensor_power_driver);
	SENSOR_LOG_I("%s: platform_driver_unregister\n",__func__);
}

module_init(kc_sensor_power_init);
module_exit(kc_sensor_power_exit);

MODULE_DESCRIPTION("Sensor power driver");
MODULE_AUTHOR("KYOCERA Corporation");
MODULE_LICENSE("GPL");

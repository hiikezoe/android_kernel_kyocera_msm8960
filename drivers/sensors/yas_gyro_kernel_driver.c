/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/*
 * Copyright (c) 2011 Yamaha Corporation
 *
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /* i2c */
#include <linux/i2c.h>
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
#include <linux/device.h>
#include <linux/spi/spi.h>
#endif
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>
#endif

#include <linux/yas.h>
#include <linux/sensor_power.h>

#if YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_BMI055
#include "yas_gyro_driver-bmi055.c"
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_EWTZMU
#include "yas_gyro_driver-ewtzmu.c"
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_ITG3500
#include "yas_gyro_driver-itg3500.c"
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G3200D
#include "yas_gyro_driver-l3g3200d.c"
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G4200D
#include "yas_gyro_driver-l3g4200d.c"
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_LSM330DLC
#include "yas_gyro_driver-lsm330dlc.c"
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_MPU3050
#include "yas_gyro_driver-mpu3050.c"
#else
#include "yas_gyro_driver-none.c"
#endif

#define YAS_GYRO_KERNEL_VERSION                                      "4.17.909"
#define YAS_GYRO_KERNEL_NAME                                         "ITG3500"

/* ABS axes parameter range [um/s^2] (for input event) */
#define ABSMAX_DPS                                                     (2000000)
#define ABSMIN_DPS                                                    (-2000000)

#define INTERRUPT_THRESHOLD                                                 (10)
#define MIN_DELAY                                                            (0)
#define MAX_DELAY                                                          (200)

#define delay_to_jiffies(d)                      ((d) ? msecs_to_jiffies(d) : 1)

/* -------------------------------------------------------------------------- *
 *  Function prototype declaration
 * -------------------------------------------------------------------------- */
static struct yas_gyro_private_data *yas_gyro_get_data(void);
static void yas_gyro_set_data(struct yas_gyro_private_data *);
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
static void yas_gyro_current_time(int32_t *sec, int32_t *msec);
#endif
static int yas_gyro_lock(void);
static int yas_gyro_unlock(void);
static int yas_gyro_device_open(void);
static int yas_gyro_device_close(void);
static int yas_gyro_device_write(uint8_t, const uint8_t *, int);
static int yas_gyro_device_read(uint8_t, uint8_t *, int);
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
static int yas_gyro_interrupt_enable(void);
static int yas_gyro_interrupt_disable(void);
static void yas_gyro_interrupt_notify(int);
static irqreturn_t yas_gyro_interrupt_handler(int, void *);
#endif /* defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT) */
static void yas_gyro_msleep(int);
static int32_t kc_gyro_set_enable(int32_t);

static int yas_gyro_core_driver_init(struct yas_gyro_private_data *);
static void yas_gyro_core_driver_fini(struct yas_gyro_private_data *);
static int yas_gyro_get_enable(struct yas_gyro_driver *);
static int yas_gyro_set_enable(struct yas_gyro_driver *, int);
static int yas_gyro_get_delay(struct yas_gyro_driver *);
static int yas_gyro_set_delay(struct yas_gyro_driver *, int);
static int yas_gyro_get_position(struct yas_gyro_driver *);
static int yas_gyro_set_position(struct yas_gyro_driver *, int);
static int yas_gyro_get_offset(struct yas_gyro_driver *, struct yas_vector *);
static int yas_gyro_set_offset(struct yas_gyro_driver *, struct yas_vector *);
static int yas_gyro_get_threshold(struct yas_gyro_driver *);
static int yas_gyro_set_threshold(struct yas_gyro_driver *, int);
static int yas_gyro_get_filter_enable(struct yas_gyro_driver *);
static int yas_gyro_set_filter_enable(struct yas_gyro_driver *, int);

static void kc_gyro_set_vsensor(struct yas_gyro_driver *,int32_t);
static int32_t kc_gyro_get_fs(struct yas_gyro_driver *);
static int32_t kc_gyro_set_fs(struct yas_gyro_driver *, int32_t);
static int32_t kc_gyro_get_calbretion_mode(struct yas_gyro_driver *);
static int32_t kc_gyro_set_calbretion_mode(struct yas_gyro_driver *, int);
static int32_t kc_gyro_get_cal_sample_num(struct yas_gyro_driver *);
static int32_t kc_gyro_set_cal_sample_num(struct yas_gyro_driver *, int);
static int32_t kc_gyro_get_cal_acc_sample_num(struct yas_gyro_driver *);
static int32_t kc_gyro_set_cal_acc_sample_num(struct yas_gyro_driver *, int);
static int32_t kc_gyro_get_cal_chk_mode(struct yas_gyro_driver *);
static int32_t kc_gyro_set_cal_chk_mode(struct yas_gyro_driver *, int);
static int32_t kc_gyro_get_acc_calbretion(struct yas_gyro_driver *);
static int32_t kc_gyro_set_acc_calbretion(struct yas_gyro_driver *, int);
static int32_t kc_gyro_get_acc_cal_thershold(struct yas_gyro_driver *);
static int32_t kc_gyro_set_acc_cal_thershold(struct yas_gyro_driver *, int);
static int32_t kc_gyro_get_gyro_cal_thershold(struct yas_gyro_driver *);
static int32_t kc_gyro_set_gyro_cal_thershold(struct yas_gyro_driver *, int);
static int32_t kc_gyro_set_iir_filter(struct yas_gyro_driver *, int);
static int32_t kc_gyro_get_iir_filter(struct yas_gyro_driver *);
static int32_t kc_gyro_set_iir_filter(struct yas_gyro_driver *, int);
static double kc_gyro_get_filter_b0(struct yas_gyro_driver *);
static double kc_gyro_get_filter_b1(struct yas_gyro_driver *);
static double kc_gyro_get_filter_b2(struct yas_gyro_driver *);
static double kc_gyro_get_filter_a1(struct yas_gyro_driver *);
static double kc_gyro_get_filter_a2(struct yas_gyro_driver *);
static int32_t kc_gyro_set_filter_b0(struct yas_gyro_driver *, double);
static int32_t kc_gyro_set_filter_b1(struct yas_gyro_driver *, double);
static int32_t kc_gyro_set_filter_b2(struct yas_gyro_driver *, double);
static int32_t kc_gyro_set_filter_a1(struct yas_gyro_driver *, double);
static int32_t kc_gyro_set_filter_a2(struct yas_gyro_driver *, double);

static int yas_gyro_measure(struct yas_gyro_driver *, struct yas_gyro_data *,
			    int);
static int yas_gyro_input_init(struct yas_gyro_private_data *);
static void yas_gyro_input_fini(struct yas_gyro_private_data *);

static ssize_t yas_gyro_enable_show(struct device *, struct device_attribute *,
				    char *);
static ssize_t yas_gyro_enable_store(struct device *, struct device_attribute *,
				     const char *, size_t);
static ssize_t yas_gyro_delay_show(struct device *, struct device_attribute *,
				   char *);
static ssize_t yas_gyro_delay_store(struct device *, struct device_attribute *,
				    const char *, size_t);
static ssize_t yas_gyro_position_show(struct device *,
				      struct device_attribute *, char *);
static ssize_t yas_gyro_position_store(struct device *,
				       struct device_attribute *,
				       const char *, size_t);
static ssize_t yas_gyro_offset_show(struct device *, struct device_attribute *,
				    char *);
static ssize_t yas_gyro_offset_store(struct device *, struct device_attribute *,
				     const char *, size_t);
static ssize_t yas_gyro_threshold_show(struct device *,
				       struct device_attribute *, char *);
static ssize_t yas_gyro_threshold_store(struct device *,
					struct device_attribute *,
					const char *, size_t);
static ssize_t yas_gyro_filter_enable_show(struct device *,
					   struct device_attribute *, char *);
static ssize_t yas_gyro_filter_enable_store(struct device *,
					    struct device_attribute *,
					    const char *, size_t);
static ssize_t yas_gyro_wake_store(struct device *, struct device_attribute *,
				   const char *, size_t);
static ssize_t yas_gyro_private_data_show(struct device *,
					  struct device_attribute *, char *);
#if DEBUG
static ssize_t yas_gyro_debug_reg_show(struct device *,
				       struct device_attribute *, char *);
static ssize_t yas_gyro_debug_reg_store(struct device *,
					struct device_attribute *,
					const char *, size_t);
#endif

static ssize_t kc_gyro_fs_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_fs_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_calbretion_mode_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_calbretion_mode_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_calbretion_chk_mode_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_calbretion_chk_mode_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_cal_sample_num_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_cal_sample_num_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_cal_acc_sample_num_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_cal_acc_sample_num_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_acc_calbretion_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_acc_calbretion_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_acc_cal_thershold_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_acc_cal_thershold_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_gyro_cal_thershold_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_gyro_cal_thershold_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_iir_filter_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_iir_filter_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_filter_b0_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_filter_b0_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_filter_b1_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_filter_b1_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_filter_b2_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_filter_b2_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_filter_a1_show(struct device *,
					struct device_attribute *, char *);
static ssize_t kc_gyro_filter_a1_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_filter_a2_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);
static ssize_t kc_gyro_filter_a2_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_ope_device_store(struct device *,
					struct device_attribute *,
					const char *, size_t);

static ssize_t kc_gyro_cal_val_fil_val_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /*i2c */
static int yas_gyro_suspend(struct i2c_client *, pm_message_t);
static int yas_gyro_resume(struct i2c_client *);
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
static int yas_gyro_suspend(struct spi_device *, pm_message_t);
static int yas_gyro_resume(struct spi_device *);
#endif

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
static void yas_gyro_work_func(struct work_struct *);
#endif
static void yas_gyro_delayed_work_func(struct work_struct *);

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /* i2c */
static int yas_gyro_probe(struct i2c_client *, const struct i2c_device_id *);
static int yas_gyro_remove(struct i2c_client *);
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
static int yas_gyro_probe(struct spi_device *);
static int yas_gyro_remove(struct spi_device *);
#endif

/* -------------------------------------------------------------------------- *
 *  Driver private data
 * -------------------------------------------------------------------------- */
struct yas_gyro_private_data {
	struct mutex driver_mutex;
	struct mutex data_mutex;
	struct mutex enable_mutex;
#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /* i2c */
	struct i2c_client *client;
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
	struct spi_device *client;
#endif
	struct input_dev *input;
	struct yas_gyro_driver *driver;
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	struct work_struct work;
#endif /* defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT) */
	struct delayed_work delayed_work;

	struct yas_gyro_data last;
	int suspend_enable;
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	int interrupt_enable;
#endif /* defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT) */
	int p_time;
#if DEBUG
	struct mutex suspend_mutex;
	int suspend;
#endif
};
static struct workqueue_struct *gyro_polling_wq;

static struct yas_gyro_private_data *yas_gyro_private_data;
static struct yas_gyro_private_data *yas_gyro_get_data(void)
{
	return yas_gyro_private_data;
}
static void yas_gyro_set_data(struct yas_gyro_private_data *data)
{
	yas_gyro_private_data = data;
}
static struct yas_gyro_data gyro_data[YAS_GYRO_FIFO_MAX];

/* -------------------------------------------------------------------------- *
 *  Local function
 * -------------------------------------------------------------------------- */
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
static void yas_gyro_current_time(int32_t *sec, int32_t *msec)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	*sec = tv.tv_sec;
	*msec = (tv.tv_usec % 1000 == 0) ?
		tv.tv_usec / 1000 : tv.tv_usec / 1000 + 1;
}
#endif

/* -------------------------------------------------------------------------- *
 *  Gyroelerlomete core driver callback function
 * -------------------------------------------------------------------------- */
static int yas_gyro_lock(void)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();

	mutex_lock(&data->driver_mutex);

	return 0;
}

static int yas_gyro_unlock(void)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();

	mutex_unlock(&data->driver_mutex);

	return 0;
}

static int yas_gyro_device_open(void)
{
	return 0;
}

static int yas_gyro_device_close(void)
{
	return 0;
}

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0    /* i2c */
static int yas_gyro_device_write(uint8_t adr, const uint8_t *buf, int len)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();
	uint8_t buffer[16];

	if (len <= 0 || len > sizeof(buffer)-1 || buf == NULL)
		return -1;

	buffer[0] = adr;
	memcpy(&buffer[1], buf, len);

#if DEBUG
	YLOGD(("[W] addr[%02x] [%02x]\n", adr, buf[0]));
#endif
	if (i2c_master_send(data->client, buffer, len+1) < 0)
		return -1;

	return 0;
}
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1  /* spi */
static int yas_gyro_device_write(uint8_t adr, const uint8_t *buf, int len)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();
	uint8_t buffer[16];
	int ret;

	if (len <= 0 || len > sizeof(buffer)-1 ||  buf == NULL)
		return -1;

#if YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G3200D
	adr |= (len > 1 ? 0x40 : 0x00);
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G4200D
	adr |= (len > 1 ? 0x40 : 0x00);
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_LSM330DLC
	adr |= (len > 1 ? 0x40 : 0x00);
#endif
	buffer[0] = adr;
	memcpy(&buffer[1], buf, len);

	ret = spi_write(data->client, buffer, len+1);
	if (ret != 0)
		return -1;

	return 0;
}
#endif

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0    /* i2c */
static int yas_gyro_device_read(uint8_t adr, uint8_t *buf, int len)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();
	struct i2c_msg msg[2];
	uint8_t reg;
	int err;

	reg = adr;
	msg[0].addr = data->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&data->client->dev,
			"i2c_transfer() read error:"
			"slave_addr=%02x, reg_addr=%02x, err=%d\n",
			data->client->addr, adr, err);
		return err;
	}

#if DEBUG
	if (len == 1) {
		YLOGD(("[R] addr[%02x] [%02x]\n", adr, buf[0]));
	} else if (len == 6) {
		YLOGD(("[R] addr[%02x] "
		"[%02x%02x%02x%02x%02x%02x]\n",
		adr, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]));
	} else if (len == 8) {
		YLOGD(("[R] addr[%02x] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x]\n",
		adr, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
		buf[7]));
	} else if (len == 9) {
		YLOGD(("[R] addr[%02x] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x%02x]\n",
		adr, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
		buf[7], buf[8]));
	} else if (len == 16) {
		YLOGD(("[R] addr[%02x] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x]\n",
		adr,
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
		buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14],
		buf[15]));
	}
#endif
	return 0;

}
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1
static int yas_gyro_device_read(uint8_t adr, uint8_t *buf, int len)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();
	uint8_t rx_buf[16];
	uint8_t tx_buf[sizeof(rx_buf)];
	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = len + 1
	};
	struct spi_message msg;

	if (len <= 0 || len > sizeof(rx_buf)-1 || buf == NULL)
		return -1;

#if YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G3200D
	adr = (adr | 0x80) | (len > 1 ? 0x40 : 0x00);
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G4200D
	adr = (adr | 0x80) | (len > 1 ? 0x40 : 0x00);
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_EWTZMU
	adr |= 0x80;
#endif
	tx_buf[0] = adr;
	memset(&tx_buf[1], 0xff, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	spi_sync(data->client, &msg);

	memcpy(buf, &rx_buf[1], len);

	return 0;
}
#endif

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
static int yas_gyro_interrupt_enable(void)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();

	if (data->interrupt_enable == 0) {
		enable_irq(data->client->irq);
		data->interrupt_enable = 1;
	}

	return 0;
}

static int yas_gyro_interrupt_disable(void)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();

	if (data->interrupt_enable == 1) {
		disable_irq_nosync(data->client->irq);
		data->interrupt_enable = 0;
	}

	return 0;
}

static void yas_gyro_interrupt_notify(int num)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();

	schedule_work(&data->work);
}

static irqreturn_t yas_gyro_interrupt_handler(int irq, void *dev)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();
	struct yas_gyro_driver *driver = data->driver;

	if (driver->interrupt_handler)
		driver->interrupt_handler();

	return IRQ_HANDLED;
}

#endif /* defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT) */

static void yas_gyro_msleep(int msec)
{
	msleep(msec);
}

static int32_t kc_gyro_set_enable(int32_t enable)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();
	struct yas_gyro_driver *driver = data->driver;
	int32_t ret;

	ret = yas_gyro_set_enable(driver, enable);
	YLOGI(("%s(): ret=%d\n",__func__, ret));
	return ret;
}

static int32_t kc_gyro_set_reset(int32_t reset)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();
	struct yas_gyro_driver *driver = data->driver;
	int32_t delay = 0;
	int32_t ret = 0;

	YLOGI(("%s(): [IN] reset=%d\n",__func__,reset));

	if (reset) {
		if (yas_gyro_get_enable(data->driver)) {
			driver->set_reset(1);
			delay = driver->get_delay();
			ret = queue_delayed_work(gyro_polling_wq, &data->delayed_work, delay_to_jiffies(delay) + 1);
			YLOGD(("%s(): queue_delayed_work ret=%d\n",__func__,ret));
		}
	} else {
		ret = cancel_delayed_work(&data->delayed_work);
		YLOGD(("%s(): cancel_delayed_work ret=%d\n",__func__,ret));
		driver->set_reset(0);
	}
	YLOGI(("%s(): [OUT]\n",__func__));

	return 0;
}
/* -------------------------------------------------------------------------- *
 *  Gyroscope core driver access function
 * -------------------------------------------------------------------------- */
static int yas_gyro_core_driver_init(struct yas_gyro_private_data *data)
{
	struct yas_gyro_driver_callback *cbk;
	struct yas_gyro_driver *driver;
	int err;

	YLOGI(("%s(): start\n",__func__));
	if(!data->driver) {
		driver = kzalloc(sizeof(struct yas_gyro_driver), GFP_KERNEL);
		data->driver = driver;
		if (!driver) {
			err = -ENOMEM;
			YLOGI(("%s(): err kzalloc\n",__func__));
			return err;
		}
	} else {
		driver = data->driver;
	}

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	data->interrupt_enable = 1;
#endif
	cbk = &driver->callback;
	cbk->lock = yas_gyro_lock;
	cbk->unlock = yas_gyro_unlock;
	cbk->device_open = yas_gyro_device_open;
	cbk->device_close = yas_gyro_device_close;
	cbk->device_write = yas_gyro_device_write;
	cbk->device_read = yas_gyro_device_read;
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	cbk->interrupt_enable = yas_gyro_interrupt_enable;
	cbk->interrupt_disable = yas_gyro_interrupt_disable;
	cbk->interrupt_notify = yas_gyro_interrupt_notify;
#else
	cbk->interrupt_enable = NULL;
	cbk->interrupt_disable = NULL;
	cbk->interrupt_notify = NULL;
#endif
	cbk->msleep = yas_gyro_msleep;
	cbk->device_set_enable = kc_gyro_set_enable;
	cbk->device_set_reset = kc_gyro_set_reset;

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	err = yas_gyro_driver_init(driver, 1);
#else
	err = yas_gyro_driver_init(driver, 0);
#endif
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		YLOGE(("%s(): err yas_gyro_driver_init\n",__func__));
		return err;
	}

	err = driver->init();
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		YLOGE(("%s(): err driver->init\n",__func__));
		return err;
	}

	err = driver->set_position(CONFIG_INPUT_YAS_GYROSCOPE_POSITION);
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		YLOGE(("%s(): err driver->set_position\n",__func__));
		return err;
	}

	YLOGI(("%s(): end\n",__func__));
	return 0;
}

static void yas_gyro_core_driver_fini(struct yas_gyro_private_data *data)
{
	struct yas_gyro_driver *driver = data->driver;

	driver->term();
	kfree(driver);
}

static int yas_gyro_get_enable(struct yas_gyro_driver *driver)
{
	int enable;

	enable = driver->get_enable();
	enable = (enable > 0)?1:0;

	return enable;
}

static int yas_gyro_set_enable(struct yas_gyro_driver *driver, int enable)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();
	int delay = driver->get_delay();
	int32_t curr_enable = yas_gyro_get_enable(data->driver);

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	int current_interrupt = driver->get_interrupt();
#endif
	YLOGI(("%s(): [IN]\n",__func__));
	driver->set_enable(enable);
	if (yas_gyro_get_enable(data->driver) != curr_enable) {
		YLOGD(("%s(): enable = %d\n",__func__,enable));
		if (enable) {
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
			if (delay >= INTERRUPT_THRESHOLD && current_interrupt)
				driver->set_interrupt(0);
			else if (delay < INTERRUPT_THRESHOLD &&
				 !current_interrupt)
				driver->set_interrupt(1);
			if (delay >= INTERRUPT_THRESHOLD)
				schedule_delayed_work(&data->delayed_work,
						      delay_to_jiffies(delay)
						      + 1);
#else
			queue_delayed_work(gyro_polling_wq,&data->delayed_work,
						 delay_to_jiffies(delay) + 1);
#endif
		} else {
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
			if (delay >= INTERRUPT_THRESHOLD) {
				cancel_delayed_work_sync(&data->delayed_work);
			} else {
				cancel_work_sync(&data->work);
			}
#else
			cancel_delayed_work_sync(&data->delayed_work);
#endif
		}
	}

	YLOGI(("%s(): [OUT]\n",__func__));
	return 0;
}

static int yas_gyro_get_delay(struct yas_gyro_driver *driver)
{
	return driver->get_delay();
}

static int yas_gyro_set_delay(struct yas_gyro_driver *driver, int delay)
{
	struct yas_gyro_private_data *data = yas_gyro_get_data();
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	int interrupt = driver->get_interrupt();
#endif

	if (delay < MIN_DELAY || MAX_DELAY < delay)
		return YAS_ERROR_ARG;

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	if (delay < INTERRUPT_THRESHOLD && !interrupt)
		driver->set_interrupt(1);
	else if (delay >= INTERRUPT_THRESHOLD && interrupt)
		driver->set_interrupt(0);
#endif /* defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT) */

	mutex_lock(&data->enable_mutex);

	if (yas_gyro_get_enable(data->driver)) {
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
		if (!interrupt)
			cancel_delayed_work_sync(&data->delayed_work);
#else
		cancel_delayed_work_sync(&data->delayed_work);
#endif
		driver->set_delay(delay);
#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
		if ((delay < INTERRUPT_THRESHOLD && !interrupt) ||
		    (delay >= INTERRUPT_THRESHOLD && interrupt)) {
			driver->set_enable(0);
			driver->set_enable(1);
		}

		if (delay >= INTERRUPT_THRESHOLD)
			schedule_delayed_work(&data->delayed_work,
					      delay_to_jiffies(delay) + 1);
#else
		queue_delayed_work(gyro_polling_wq, &data->delayed_work,
				 delay_to_jiffies(delay) + 1);
#endif
	} else {
		driver->set_delay(delay);
	}

	mutex_unlock(&data->enable_mutex);

	return 0;
}

static int yas_gyro_get_position(struct yas_gyro_driver *driver)
{
	return driver->get_position();
}

static int yas_gyro_set_position(struct yas_gyro_driver *driver, int position)
{
	return driver->set_position(position);
}

static int yas_gyro_get_offset(struct yas_gyro_driver *driver,
			       struct yas_vector *offset)
{
	return driver->get_offset(offset);
}

static int yas_gyro_set_offset(struct yas_gyro_driver *driver,
			       struct yas_vector *offset)
{
	return driver->set_offset(offset);
}

static int yas_gyro_get_threshold(struct yas_gyro_driver *driver)
{
	struct yas_gyro_filter filter;

	driver->get_filter(&filter);

	return filter.threshold;
}

static int yas_gyro_set_threshold(struct yas_gyro_driver *driver,
				  int threshold)
{
	struct yas_gyro_filter filter;

	filter.threshold = threshold;

	return driver->set_filter(&filter);
}

static int yas_gyro_get_filter_enable(struct yas_gyro_driver *driver)
{
	return driver->get_filter_enable();
}

static int yas_gyro_set_filter_enable(struct yas_gyro_driver *driver,
				      int enable)
{
	return driver->set_filter_enable(enable);
}

static void kc_gyro_set_vsensor(struct yas_gyro_driver *driver, int32_t enable)
{
	driver->set_vsensor(enable);
}
static int32_t kc_gyro_set_enable_cnt(struct yas_gyro_driver *driver, int32_t enable)
{
	return driver->set_enable_cnt(enable);
}
static int32_t kc_gyro_get_fs(struct yas_gyro_driver *driver)
{
	return driver->get_gyro_fs();
}
static int32_t kc_gyro_set_fs(struct yas_gyro_driver *driver, int32_t fs)
{
	return driver->set_gyro_fs(fs);
}
static int32_t kc_gyro_get_calbretion_mode(struct yas_gyro_driver *driver)
{
	return driver->get_calbretion_mode();
}
static int32_t kc_gyro_set_calbretion_mode(struct yas_gyro_driver *driver, int mode)
{
	return driver->set_calbretion_mode(mode);
}
static int32_t kc_gyro_get_cal_chk_mode(struct yas_gyro_driver *driver)
{
	return driver->get_cal_chk_mode();
}
static int32_t kc_gyro_set_cal_chk_mode(struct yas_gyro_driver *driver, int mode)
{
	return driver->set_cal_chk_mode(mode);
}
static int32_t kc_gyro_get_cal_sample_num(struct yas_gyro_driver *driver)
{
	return driver->get_cal_sample_num();
}
static int32_t kc_gyro_set_cal_sample_num(struct yas_gyro_driver *driver, int num)
{
	return driver->set_cal_sample_num(num);
}
static int32_t kc_gyro_get_cal_acc_sample_num(struct yas_gyro_driver *driver)
{
	return driver->get_cal_acc_sample_num();
}
static int32_t kc_gyro_set_cal_acc_sample_num(struct yas_gyro_driver *driver, int num)
{
	return driver->set_cal_acc_sample_num(num);
}
static int32_t kc_gyro_get_acc_calbretion(struct yas_gyro_driver *driver)
{
	return driver->get_acc_calbretion();
}
static int32_t kc_gyro_set_acc_calbretion(struct yas_gyro_driver *driver, int acc_cal)
{
	return driver->set_acc_calbretion(acc_cal);
}
static int32_t kc_gyro_get_acc_cal_thershold(struct yas_gyro_driver *driver)
{
	return driver->get_acc_cal_thershold();
}
static int32_t kc_gyro_set_acc_cal_thershold(struct yas_gyro_driver *driver, int threshold)
{
	return driver->set_acc_cal_thershold(threshold);
}
static int32_t kc_gyro_get_gyro_cal_thershold(struct yas_gyro_driver *driver)
{
	return driver->get_gyro_cal_thershold();
}
static int32_t kc_gyro_set_gyro_cal_thershold(struct yas_gyro_driver *driver, int threshold)
{
	return driver->set_gyro_cal_thershold(threshold);
}
static int32_t kc_gyro_get_iir_filter(struct yas_gyro_driver *driver)
{
	return driver->get_iir_filter();
}
static int32_t kc_gyro_set_iir_filter(struct yas_gyro_driver *driver, int iir_filter)
{
	return driver->set_iir_filter(iir_filter);
}
/* static int32_t kc_gyro_iir_filter(struct yas_gyro_driver *driver, struct yas_gyro_data *data) */
/* { */
/*	return driver->iir_filter(data); */
/* } */
static double kc_gyro_get_filter_b0(struct yas_gyro_driver *driver)
{
	return driver->get_filter_b0();
}
static int32_t kc_gyro_set_filter_b0(struct yas_gyro_driver *driver, double filter)
{
	return driver->set_filter_b0(filter);
}
static double kc_gyro_get_filter_b1(struct yas_gyro_driver *driver)
{
	return driver->get_filter_b1();
}
static int32_t kc_gyro_set_filter_b1(struct yas_gyro_driver *driver, double filter)
{
	return driver->set_filter_b1(filter);
}
static double kc_gyro_get_filter_b2(struct yas_gyro_driver *driver)
{
	return driver->get_filter_b2();
}
static int32_t kc_gyro_set_filter_b2(struct yas_gyro_driver *driver, double filter)
{
	return driver->set_filter_b2(filter);
}
static double kc_gyro_get_filter_a1(struct yas_gyro_driver *driver)
{
	return driver->get_filter_a1();
}
static int32_t kc_gyro_set_filter_a1(struct yas_gyro_driver *driver, double filter)
{
	return driver->set_filter_a1(filter);
}
static double kc_gyro_get_filter_a2(struct yas_gyro_driver *driver)
{
	return driver->get_filter_a2();
}
static int32_t kc_gyro_set_filter_a2(struct yas_gyro_driver *driver, double filter)
{
	return driver->set_filter_a2(filter);
}

static int yas_gyro_measure(struct yas_gyro_driver *driver,
			    struct yas_gyro_data *gyro, int num)
{
	int actual_num;

	actual_num = driver->measure(gyro, num);

	return actual_num;
}

/* -------------------------------------------------------------------------- *
 *  Input device interface
 * -------------------------------------------------------------------------- */
static int yas_gyro_input_init(struct yas_gyro_private_data *data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = "gyroscope";
#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0
	dev->id.bustype = BUS_I2C;
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1
	dev->id.bustype = BUS_HOST;
#endif
	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_capability(dev, EV_ABS, ABS_RUDDER);
	input_set_capability(dev, EV_ABS, ABS_WHEEL);
	input_set_abs_params(dev, ABS_X, ABSMIN_DPS, ABSMAX_DPS, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_DPS, ABSMAX_DPS, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_DPS, ABSMAX_DPS, 0, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	data->input = dev;

	return 0;
}

static void yas_gyro_input_fini(struct yas_gyro_private_data *data)
{
	struct input_dev *dev = data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static ssize_t yas_gyro_enable_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	int enable = 0;

	mutex_lock(&data->enable_mutex);

	enable = yas_gyro_get_enable(data->driver);

	mutex_unlock(&data->enable_mutex);

	return sprintf(buf, "%d\n", enable);
}

static ssize_t yas_gyro_enable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	unsigned long enable;
	int ret;
	static int cnt = 1;	
	YLOGI(("%s(): [IN]\n",__func__));

	ret = strict_strtoul(buf, 10, &enable);
	if (ret < 0)
		return count;

	mutex_lock(&data->enable_mutex);

	yas_gyro_set_enable(data->driver, enable);
	input_report_abs(input, ABS_WHEEL, cnt++);
	input_sync(input);

	mutex_unlock(&data->enable_mutex);

	return count;
}

static ssize_t yas_gyro_delay_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_gyro_get_delay(data->driver));
}

static ssize_t yas_gyro_delay_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	unsigned long delay;
	int ret;

	ret = strict_strtoul(buf, 10, &delay);
	if (ret < 0)
		return count;

	yas_gyro_set_delay(data->driver, delay);

	return count;
}

static ssize_t yas_gyro_position_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_gyro_get_position(data->driver));
}

static ssize_t yas_gyro_position_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	unsigned long position;
	int ret;

	ret = strict_strtoul(buf, 10, &position);
	if (ret < 0)
		return count;

	yas_gyro_set_position(data->driver, position);

	return count;
}

static ssize_t yas_gyro_offset_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	struct yas_vector offset;

	yas_gyro_get_offset(data->driver, &offset);

	return sprintf(buf, "%d %d %d\n",
		       offset.v[0], offset.v[1], offset.v[2]);
}

static ssize_t yas_gyro_offset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	struct yas_vector offset;

	sscanf(buf, "%d %d %d", &offset.v[0], &offset.v[1], &offset.v[2]);
	yas_gyro_set_offset(data->driver, &offset);

	return count;
}

static ssize_t yas_gyro_threshold_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_gyro_get_threshold(data->driver));
}

static ssize_t yas_gyro_threshold_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	unsigned long threshold;
	int ret;

	ret = strict_strtoul(buf, 10, &threshold);
	if (ret < 0)
		return count;

	yas_gyro_set_threshold(data->driver, threshold);

	return count;
}

static ssize_t yas_gyro_filter_enable_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_gyro_get_filter_enable(data->driver));
}

static ssize_t yas_gyro_filter_enable_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf,
					    size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	unsigned long enable;
	int ret;

	ret = strict_strtoul(buf, 10, &enable);
	if (ret < 0)
		return count;

	yas_gyro_set_filter_enable(data->driver, enable);

	return count;
}

static ssize_t yas_gyro_wake_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);

	input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));
	input_sync(input);

	return count;
}

static ssize_t yas_gyro_private_data_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	struct yas_gyro_data gyro;

	YLOGI(("%s(): [IN]\n",__func__));
	mutex_lock(&data->data_mutex);
	gyro = data->last;
	mutex_unlock(&data->data_mutex);

	YLOGI(("%s(): [OUT]\n",__func__));
	return sprintf(buf, "%d %d %d\n",
		       gyro.xyz.v[0], gyro.xyz.v[1], gyro.xyz.v[2]);
}

#if DEBUG
#if YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_BMI055
#define ADR_MAX (0x39)
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_EWTZMU
#define ADR_MAX (0x3f)
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_ITG3500
#define ADR_MAX (0x76)
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G3200D
#define ADR_MAX (0x39)
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G4200D
#define ADR_MAX (0x39)
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_LSM330DLC
#define ADR_MAX (0x39)
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_MPU3050
#define ADR_MAX (0x3f)
#else
#define ADR_MAX (0x16)
#endif
static uint8_t reg[ADR_MAX];

static uint8_t hcharton(uint8_t c)
{
	if ('0' <= c && '9' >= c)
		return c - 0x30;
	if ('A' <= c && 'F' >= c)
		return c + 0x0A - 0x41;
	if ('a' <= c && 'f' >= c)
		return c + 0x0A - 0x61;
	return 0;
}

static uint8_t hstrton(uint8_t *str)
{
	uint8_t i, x;

	for (i = 0, x = 0; i < strlen(str); i++, str) {
		x <<= (4*i);
		x += hcharton(*(str+i));
	}

	return x;
}

static ssize_t yas_gyro_debug_reg_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	ssize_t count = 0;
	int ret;
	int i;

	memset(reg, -1, ADR_MAX);
	for (i = 0; i < ADR_MAX; i++) {
		ret = data->driver->get_register(i, &reg[i]);
		if (ret != 0)
			dev_err(dev, "get_register() erorr %d (%d)\n", ret, i);
		else
			count += sprintf(&buf[count], "%02x: %d\n", i, reg[i]);
	}

	return count;
}

static ssize_t yas_gyro_debug_reg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	uint8_t buf1[4], buf2[4], buf3[6];
	uint8_t adr, val;
	int ret;

	memset(buf1, 0, 4);
	memset(buf2, 0, 4);
	memcpy(buf3, buf, 5);
	buf3[5] = 0;

	sscanf(buf3, "%s %s", buf1, buf2);
	adr = hstrton(buf1);
	val = hstrton(buf2);

	ret = data->driver->set_register(adr, val);
	if (ret != 0)
		dev_err(dev, "set_register() erorr %d\n", ret);

	YLOGI((KERN_INFO "set register (adr=0x%02x val=0x%02x)\n", adr, val));

	return count;
}

static ssize_t yas_gyro_debug_suspend_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	int suspend = 0;

	mutex_lock(&data->suspend_mutex);

	suspend = sprintf(buf, "%d\n", data->suspend);

	mutex_unlock(&data->suspend_mutex);

	return suspend;
}

static ssize_t yas_gyro_debug_suspend_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /* i2c */
	struct i2c_client *client = data->client;
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1
	struct spi_device *client = data->client;
#endif
	unsigned long suspend;
	pm_message_t msg;
	int ret;

	ret = strict_strtoul(buf, 10, &suspend);
	if (ret < 0)
		return count;

	memset(&msg, 0, sizeof(pm_message_t));

	mutex_lock(&data->suspend_mutex);

	if (suspend) {
		yas_gyro_suspend(client, msg);
		data->suspend = 1;
	} else {
		yas_gyro_resume(client);
		data->suspend = 0;
	}

	mutex_unlock(&data->suspend_mutex);

	return count;
}
#endif /* DEBUG */

static ssize_t kc_gyro_fs_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	YLOGI(("%s(): [IN]\n",__func__));

	return sprintf(buf, "%d\n", kc_gyro_get_fs(data->driver));
}

static ssize_t kc_gyro_fs_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int32_t fs;

	YLOGI(("%s(): [IN]\n",__func__));
	sscanf(buf, "%d", &fs);
	YLOGD(("%s(): fs = %d\n",__func__,fs));

	kc_gyro_set_fs(data->driver, fs);
	
	return count;
}
static ssize_t kc_gyro_calbretion_mode_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", kc_gyro_get_calbretion_mode(data->driver));

}
static ssize_t kc_gyro_calbretion_mode_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int calbretion_mode;
	YLOGI(("%s(): start\n",__func__));

	sscanf(buf, "%d", &calbretion_mode);

	YLOGD(("%s(): calbretion_mode = %d \n",__func__,calbretion_mode));

	kc_gyro_set_calbretion_mode(data->driver, calbretion_mode);
	
	return count;
}
static ssize_t kc_gyro_calbretion_chk_mode_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", kc_gyro_get_cal_chk_mode(data->driver));

}
static ssize_t kc_gyro_calbretion_chk_mode_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int calbretion_mode;

	sscanf(buf, "%d", &calbretion_mode);
	YLOGD(("%s(): calbretion_mode = %d \n",__func__,calbretion_mode));

	kc_gyro_set_cal_chk_mode(data->driver, calbretion_mode);
	
	return count;
}
static ssize_t kc_gyro_cal_sample_num_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", kc_gyro_get_cal_sample_num(data->driver));
}
static ssize_t kc_gyro_cal_sample_num_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int32_t num;

	sscanf(buf, "%d", &num);
	YLOGD(("%s(): num = %d \n",__func__,num));

	kc_gyro_set_cal_sample_num(data->driver, num);
	
	return count;
}
static ssize_t kc_gyro_cal_acc_sample_num_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", kc_gyro_get_cal_acc_sample_num(data->driver));
}
static ssize_t kc_gyro_cal_acc_sample_num_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int32_t num;

	sscanf(buf, "%d", &num);
	YLOGD(("%s(): num = %d \n",__func__,num));

	kc_gyro_set_cal_acc_sample_num(data->driver, num);
	
	return count;
}
static ssize_t kc_gyro_acc_calbretion_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", kc_gyro_get_acc_calbretion(data->driver));
}
static ssize_t kc_gyro_acc_calbretion_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int acc_cal;

	sscanf(buf, "%d", &acc_cal);
	YLOGI(("%s(): start\n",__func__));
	YLOGD(("%s(): acc_cal = %d \n",__func__,acc_cal));

	kc_gyro_set_acc_calbretion(data->driver, acc_cal);
	
	return count;
}
static ssize_t kc_gyro_acc_cal_thershold_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", kc_gyro_get_acc_cal_thershold(data->driver));
}
static ssize_t kc_gyro_acc_cal_thershold_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int threshold;

	sscanf(buf, "%d", &threshold);

	kc_gyro_set_acc_cal_thershold(data->driver, threshold);
	
	return count;
}
static ssize_t kc_gyro_gyro_cal_thershold_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", kc_gyro_get_gyro_cal_thershold(data->driver));
}
static ssize_t kc_gyro_gyro_cal_thershold_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int threshold;

	sscanf(buf, "%d", &threshold);

	kc_gyro_set_gyro_cal_thershold(data->driver, threshold);
	
	return count;
}

static ssize_t kc_gyro_iir_filter_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", kc_gyro_get_iir_filter(data->driver));
}
static ssize_t kc_gyro_iir_filter_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int iir;

	sscanf(buf, "%d", &iir);
	YLOGI(("%s(): start\n",__func__));
	YLOGD(("%s(): iir = %d \n",__func__,iir));

	kc_gyro_set_iir_filter(data->driver, iir);
	
	return count;
}
static ssize_t kc_gyro_filter_b0_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	double filter;
	
	filter = kc_gyro_get_filter_b0(data->driver);

	return sprintf(buf, "%d\n", (int)(filter*10000));
}
static ssize_t kc_gyro_filter_b0_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int filter;

	YLOGI(("%s(): start\n",__func__));
	sscanf(buf, "%d", &filter);
	YLOGD(("%s(): filter_b0 = %d \n",__func__,filter));

	kc_gyro_set_filter_b0(data->driver, (double)filter);
	
	return count;
}
static ssize_t kc_gyro_filter_b1_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	double filter;
	
	filter = kc_gyro_get_filter_b1(data->driver);

	return sprintf(buf, "%d\n", (int)(filter*10000));

}
static ssize_t kc_gyro_filter_b1_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int filter;

	YLOGI(("%s(): start\n",__func__));
	sscanf(buf, "%d", &filter);
	YLOGD(("%s(): filter_b1 = %d \n",__func__,filter));

	kc_gyro_set_filter_b1(data->driver, (double)filter);
	
	return count;
}
static ssize_t kc_gyro_filter_b2_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	double filter;
	
	filter = kc_gyro_get_filter_b2(data->driver);

	return sprintf(buf, "%d\n", (int)(filter*10000));
}
static ssize_t kc_gyro_filter_b2_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int filter;

	YLOGI(("%s(): start\n",__func__));
	sscanf(buf, "%d", &filter);
	YLOGD(("%s(): filter_b2 = %d \n",__func__,filter));

	kc_gyro_set_filter_b2(data->driver, (double)filter);
	
	return count;
}
static ssize_t kc_gyro_filter_a1_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	double filter;
	
	filter = kc_gyro_get_filter_a1(data->driver);

	return sprintf(buf, "%d\n", (int)(filter*10000));

}
static ssize_t kc_gyro_filter_a1_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int filter;

	YLOGI(("%s(): start\n",__func__));
	sscanf(buf, "%d", &filter);
	YLOGD(("%s(): filter_a1 = %d \n",__func__,filter));

	kc_gyro_set_filter_a1(data->driver, (double)filter);
	
	return count;

}
static ssize_t kc_gyro_filter_a2_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	double filter;
	
	filter = kc_gyro_get_filter_a2(data->driver);

	return sprintf(buf, "%d\n", (int)(filter*10000));

}
static ssize_t kc_gyro_filter_a2_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int filter;

	YLOGI(("%s(): start\n",__func__));
	sscanf(buf, "%d", &filter);
	YLOGD(("%s(): filter_a2 = %d \n",__func__,filter));

	kc_gyro_set_filter_a2(data->driver, (double)filter);
	
	return count;

}

static ssize_t kc_gyro_ope_device_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);

	int number;
	int32_t curr_enable;
	YLOGI(("%s(): [IN]\n",__func__));

	sscanf(buf, "%d", &number);

	YLOGD(("%s(): number = %d\n",__func__,number));
	switch (number)
	{
	case KC_SENSOR_COMMON_POWER_ON:
		kc_gyro_set_vsensor(data->driver,1);
		break;
	case KC_SENSOR_COMMON_POWER_OFF:
		kc_gyro_set_vsensor(data->driver,0);
		break;
	case KC_SENSOR_COMMON_INIT:
		curr_enable = yas_gyro_get_enable(data->driver);
		YLOGD(("%s(): curr_enable = %d\n",__func__,curr_enable));
		if (curr_enable > 0) {
			kc_gyro_set_enable_cnt(data->driver, 1);
			yas_gyro_set_enable(data->driver, 0);
		}
		yas_gyro_core_driver_init(yas_gyro_get_data());
		break;
	default:
		break;
	}
	YLOGI(("%s(): [OUT]\n",__func__));
	return count;
}

static ssize_t kc_gyro_cal_val_fil_val_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	struct yas_gyro_data gyro;
	
	YLOGI(("%s(): [IN]\n",__func__));
	mutex_lock(&data->data_mutex);
	gyro = data->last;
	mutex_unlock(&data->data_mutex);

	YLOGI(("%s(): [OUT] %d %d %d\n",__func__,gyro.xyz.v[0], gyro.xyz.v[1], gyro.xyz.v[2]));
	return sprintf(buf, "%d %d %d\n",
		       gyro.xyz.v[0], gyro.xyz.v[1], gyro.xyz.v[2]);

}

static ssize_t kc_gyro_raw_data_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_gyro_private_data *data = input_get_drvdata(input);
	struct yas_gyro_data gyro;

	YLOGI(("%s(): [IN]\n",__func__));
	mutex_lock(&data->data_mutex);
	gyro = data->last;
	mutex_unlock(&data->data_mutex);
	YLOGI(("%s(): [OUT] %d %d %d\n",__func__,gyro.raw.v[0], gyro.raw.v[1], gyro.raw.v[2]));
	return sprintf(buf, "%d %d %d\n",
		       gyro.raw.v[0], gyro.raw.v[1], gyro.raw.v[2]);


}

static DEVICE_ATTR(enable,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   yas_gyro_enable_show,
		   yas_gyro_enable_store
		   );
static DEVICE_ATTR(delay,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   yas_gyro_delay_show,
		   yas_gyro_delay_store
		   );
static DEVICE_ATTR(threshold,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   yas_gyro_threshold_show,
		   yas_gyro_threshold_store
		   );
static DEVICE_ATTR(filter_enable,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   yas_gyro_filter_enable_show,
		   yas_gyro_filter_enable_store
		   );
static DEVICE_ATTR(position,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   yas_gyro_position_show,
		   yas_gyro_position_store
		   );
static DEVICE_ATTR(offset,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   yas_gyro_offset_show,
		   yas_gyro_offset_store
		   );
static DEVICE_ATTR(wake,
		   S_IWUSR|S_IWGRP,
		   NULL,
		   yas_gyro_wake_store);
static DEVICE_ATTR(data,
		   S_IRUSR|S_IRGRP,
		   yas_gyro_private_data_show,
		   NULL);
#if DEBUG
static DEVICE_ATTR(debug_reg,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   yas_gyro_debug_reg_show,
		   yas_gyro_debug_reg_store
		   );
static DEVICE_ATTR(debug_suspend,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   yas_gyro_debug_suspend_show,
		   yas_gyro_debug_suspend_store
		   );
#endif /* DEBUG */

static DEVICE_ATTR(fs,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_fs_show,
		   kc_gyro_fs_store);

static DEVICE_ATTR(calbretion_mode,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_calbretion_mode_show,
		   kc_gyro_calbretion_mode_store);

static DEVICE_ATTR(calbretion_chk_mode,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_calbretion_chk_mode_show,
		   kc_gyro_calbretion_chk_mode_store);

static DEVICE_ATTR(cal_sample_num,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_cal_sample_num_show,
		   kc_gyro_cal_sample_num_store);

static DEVICE_ATTR(cal_acc_sample_num,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_cal_acc_sample_num_show,
		   kc_gyro_cal_acc_sample_num_store);

static DEVICE_ATTR(acc_calbretion,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_acc_calbretion_show,
		   kc_gyro_acc_calbretion_store);

static DEVICE_ATTR(acc_cal_thershold,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_acc_cal_thershold_show,
		   kc_gyro_acc_cal_thershold_store);

static DEVICE_ATTR(gyro_cal_thershold,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_gyro_cal_thershold_show,
		   kc_gyro_gyro_cal_thershold_store);
		   
static DEVICE_ATTR(iir_filter,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_iir_filter_show,
		   kc_gyro_iir_filter_store);

static DEVICE_ATTR(filter_b0,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_filter_b0_show,
		   kc_gyro_filter_b0_store);

static DEVICE_ATTR(filter_b1,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_filter_b1_show,
		   kc_gyro_filter_b1_store);

static DEVICE_ATTR(filter_b2,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_filter_b2_show,
		   kc_gyro_filter_b2_store);

static DEVICE_ATTR(filter_a1,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_filter_a1_show,
		   kc_gyro_filter_a1_store);

static DEVICE_ATTR(filter_a2,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   kc_gyro_filter_a2_show,
		   kc_gyro_filter_a2_store);

static DEVICE_ATTR(ope_device,
		   S_IWUSR|S_IWGRP,
		   NULL,
		   kc_gyro_ope_device_store);

static DEVICE_ATTR(cal_val_fil_val,
		   S_IRUSR|S_IRGRP,
		   kc_gyro_cal_val_fil_val_show,
		   NULL);

static DEVICE_ATTR(raw_data,
		   S_IRUSR|S_IRGRP,
		   kc_gyro_raw_data_show,
		   NULL);

static struct attribute *yas_gyro_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_position.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_offset.attr,
	&dev_attr_threshold.attr,
	&dev_attr_filter_enable.attr,
#if DEBUG
	&dev_attr_debug_reg.attr,
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
	&dev_attr_fs.attr,
	&dev_attr_calbretion_mode.attr,
	&dev_attr_calbretion_chk_mode.attr,
	&dev_attr_cal_sample_num.attr,
	&dev_attr_cal_acc_sample_num.attr,
	&dev_attr_acc_calbretion.attr,
	&dev_attr_acc_cal_thershold.attr,
	&dev_attr_gyro_cal_thershold.attr,
	&dev_attr_iir_filter.attr,
	&dev_attr_filter_b0.attr,
	&dev_attr_filter_b1.attr,
	&dev_attr_filter_b2.attr,
	&dev_attr_filter_a1.attr,
	&dev_attr_filter_a2.attr,
	&dev_attr_ope_device.attr,
	&dev_attr_cal_val_fil_val.attr,
	&dev_attr_raw_data.attr,
	NULL
};

static struct attribute_group yas_gyro_attribute_group = {
	.attrs = yas_gyro_attributes
};

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
static void yas_gyro_work_func(struct work_struct *work)
{
	struct yas_gyro_private_data *data =
		container_of((struct work_struct *)work,
			     struct yas_gyro_private_data, work);
	struct yas_gyro_data last;
	int filter_enable = yas_gyro_get_filter_enable(data->driver);
	int threshold = yas_gyro_get_threshold(data->driver);
	int32_t sec, msec, c_time, x_time;
	unsigned long delay = yas_gyro_get_delay(data->driver);
	int d_x, d_y, d_z;
	static int cnt;
	int num;
	int i;

	x_time = 0;
	yas_gyro_current_time(&sec, &msec);
	if (msec == 1000) {
		sec++;
		msec = 0;
	}
	c_time = sec * 1000 + msec;
	x_time = c_time - data->p_time;
	data->p_time = c_time;

	memset(gyro_data, 0, NELEMS(gyro_data));
	num = yas_gyro_measure(data->driver, gyro_data, YAS_GYRO_FIFO_MAX);
	if (num < 1 || x_time < delay)
		return;

	for (i = 0; i < num; i++) {
		/* filter */
		if (filter_enable) {
			mutex_lock(&data->data_mutex);
			d_x = ABS(data->last.xyz.v[0] - gyro_data[i].xyz.v[0]);
			d_y = ABS(data->last.xyz.v[1] - gyro_data[i].xyz.v[1]);
			d_z = ABS(data->last.xyz.v[2] - gyro_data[i].xyz.v[2]);
			if (d_x > threshold ||
			    d_y > threshold ||
			    d_z > threshold)
				;
			else
				gyro_data[i] = data->last;
			mutex_unlock(&data->data_mutex);
		}

		mutex_lock(&data->data_mutex);
		last = data->last;
		mutex_unlock(&data->data_mutex);

		input_report_abs(data->input, ABS_X, gyro_data[i].xyz.v[0]);
		input_report_abs(data->input, ABS_Y, gyro_data[i].xyz.v[1]);
		input_report_abs(data->input, ABS_Z, gyro_data[i].xyz.v[2]);
		if (last.xyz.v[0] == gyro_data[0].xyz.v[0] &&
		    last.xyz.v[1] == gyro_data[0].xyz.v[1] &&
		    last.xyz.v[2] == gyro_data[0].xyz.v[2])
			input_report_abs(data->input, ABS_RUDDER, cnt++);
		input_sync(data->input);

		mutex_lock(&data->data_mutex);
		data->last = gyro_data[i];
		mutex_unlock(&data->data_mutex);
	}
}
#endif /* defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT) */

static void yas_gyro_delayed_work_func(struct work_struct *delayed_work)
{
	struct yas_gyro_private_data *data =
		container_of((struct delayed_work *)delayed_work,
			     struct yas_gyro_private_data, delayed_work);
	struct yas_gyro_data last;
	int32_t delay = 0;
	static int cnt;

/*	int32_t iir_filter = kc_gyro_get_iir_filter(data->driver); */
	
	memset(gyro_data, 0, NELEMS(gyro_data));
	yas_gyro_measure(data->driver, gyro_data, 1);

/*	if(iir_filter) */
/*		kc_gyro_iir_filter(data->driver, gyro_data); */

	mutex_lock(&data->data_mutex);
	last = data->last;
	mutex_unlock(&data->data_mutex);

	input_report_abs(data->input, ABS_X, gyro_data[0].xyz.v[0]);
	input_report_abs(data->input, ABS_Y, gyro_data[0].xyz.v[1]);
	input_report_abs(data->input, ABS_Z, gyro_data[0].xyz.v[2]);
	if (last.xyz.v[0] == gyro_data[0].xyz.v[0] &&
	    last.xyz.v[1] == gyro_data[0].xyz.v[1] &&
	    last.xyz.v[2] == gyro_data[0].xyz.v[2])
		input_report_abs(data->input, ABS_RUDDER, cnt++);
	input_sync(data->input);

	mutex_lock(&data->data_mutex);
	data->last = gyro_data[0];
	mutex_unlock(&data->data_mutex);

	delay = yas_gyro_get_delay(data->driver);
	if (delay > 0)
		queue_delayed_work(gyro_polling_wq, &data->delayed_work, (unsigned long)delay_to_jiffies(delay));
	else
		YLOGD(("%s(): delay=%d\n",__func__,delay));
}

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /* i2c */
static int yas_gyro_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
static int yas_gyro_probe(struct spi_device *client)
#endif
{
	struct yas_gyro_private_data *data;
	int err;

	YLOGI(("%s(): start\n",__func__));

	/* Setup private data */
	data = kzalloc(sizeof(struct yas_gyro_private_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		YLOGE(("%s(): err kzallc\n",__func__));
		goto ERR1;
	}
	yas_gyro_set_data(data);

	mutex_init(&data->driver_mutex);
	mutex_init(&data->data_mutex);
	mutex_init(&data->enable_mutex);
#if DEBUG
	mutex_init(&data->suspend_mutex);
#endif

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /* i2c */
	/* Setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		YLOGE(("%s(): err i2c_check_functionality\n",__func__));
		goto ERR2;
	}
	i2c_set_clientdata(client, data);
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
	/* Setup spi device */
	spi_set_drvdata(client, data);
#endif
	data->client = client;

	/* Setup gyroscope core driver */
	err = yas_gyro_core_driver_init(data);
	if (err < 0)
	{
		YLOGE(("%s(): err yas_gyro_core_driver_init\n",__func__));
		goto ERR2;
	}

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	/* Setup driver interface */
	INIT_WORK(&data->work, yas_gyro_work_func);
	INIT_DELAYED_WORK(&data->delayed_work, yas_gyro_delayed_work_func);
#else
	/* Setup driver interface */
	INIT_DELAYED_WORK(&data->delayed_work, yas_gyro_delayed_work_func);
#endif /* defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT) */

	/* Setup input device interface */
	err = yas_gyro_input_init(data);
	if (err < 0)
	{
		YLOGE(("%s(): err yas_gyro_input_init\n",__func__));
		goto ERR3;
	}

	/* Setup sysfs */
	err = sysfs_create_group(&data->input->dev.kobj,
				 &yas_gyro_attribute_group);
	if (err < 0)
	{
		YLOGE(("%s(): err sysfs_create_group\n",__func__));
		goto ERR4;
	}

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	/* Setup interrupt */
	err = request_irq(data->client->irq, yas_gyro_interrupt_handler, 0,
			  "Ext-Bus", NULL);
	if (err != 0)
	{
		YLOGE(("%s(): err request_irq\n",__func__));
		goto ERR5;
	}
#endif /* defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT) */

	YLOGI(("%s(): end\n",__func__));
	return 0;

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
ERR5:
	sysfs_remove_group(&data->input->dev.kobj, &yas_gyro_attribute_group);
#endif
ERR4:
	YLOGI(("%s(): ERR4\n",__func__));
	yas_gyro_input_fini(data);
ERR3:
	YLOGI(("%s(): ERR3\n",__func__));
	yas_gyro_core_driver_fini(data);
ERR2:
	YLOGI(("%s(): ERR2\n",__func__));
	kfree(data);
ERR1:
	YLOGI(("%s(): end err=%d\n",__func__,err));
	return err;
}

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /*i2c */
static int yas_gyro_remove(struct i2c_client *client)
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
static int yas_gyro_remove(struct spi_device *client)
#endif
{
#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /*i2c */
	struct yas_gyro_private_data *data = i2c_get_clientdata(client);
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
	struct yas_gyro_private_data *data =
		dev_get_drvdata((struct device *)client);
#endif
	struct yas_gyro_driver *driver = data->driver;

#if defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT)
	free_irq(data->client->irq, NULL);
#endif /* defined(CONFIG_INPUT_YAS_GYROSCOPE_INTERRUPT) */
	yas_gyro_set_enable(driver, 0);
	sysfs_remove_group(&data->input->dev.kobj, &yas_gyro_attribute_group);
	yas_gyro_input_fini(data);
	yas_gyro_core_driver_fini(data);
	kfree(data);

	return 0;
}

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /*i2c */
static int yas_gyro_suspend(struct i2c_client *client, pm_message_t mesg)
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
static int yas_gyro_suspend(struct spi_device *client, pm_message_t mesg)
#endif
{
#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /*i2c */
	struct yas_gyro_private_data *data =
		i2c_get_clientdata((struct i2c_client *)client);
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
	struct yas_gyro_private_data *data =
		dev_get_drvdata((struct device *)client);
#endif
	struct yas_gyro_driver *driver = data->driver;

	(void)mesg;

	mutex_lock(&data->enable_mutex);

	data->suspend_enable = driver->get_enable();
	if (data->suspend_enable > 0) {
		kc_gyro_set_enable_cnt(driver, 1);
		yas_gyro_set_enable(driver, 0);
	}

	mutex_unlock(&data->enable_mutex);

	return 0;
}

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /*i2c */
static int yas_gyro_resume(struct i2c_client *client)
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
static int yas_gyro_resume(struct spi_device *client)
#endif
{
#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0   /*i2c */
	struct yas_gyro_private_data *data = i2c_get_clientdata(client);
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
	struct yas_gyro_private_data *data =
		dev_get_drvdata((struct device *)client);
#endif
	struct yas_gyro_driver *driver = data->driver;

	mutex_lock(&data->enable_mutex);

	if (data->suspend_enable > 0) {
		yas_gyro_set_enable(driver, 1);
		kc_gyro_set_enable_cnt(driver, data->suspend_enable);
	}

	mutex_unlock(&data->enable_mutex);

	return 0;
}

#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0     /* i2c */
static const struct i2c_device_id yas_gyro_id[] = {
	{YAS_GYRO_KERNEL_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, yas_gyro_id);
static struct i2c_driver yas_gyro_driver = {
	.driver = {
		.name = YAS_GYRO_KERNEL_NAME,
		.owner = THIS_MODULE,
	},
	.probe = yas_gyro_probe,
	.remove = yas_gyro_remove,
	.suspend = yas_gyro_suspend,
	.resume = yas_gyro_resume,
	.id_table = yas_gyro_id,
};
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1 /* spi */
static struct spi_driver yas_gyro_driver = {
	.driver = {
		.name = "gyroscope",
		.owner = THIS_MODULE,
	},
	.probe = yas_gyro_probe,
	.remove = yas_gyro_remove,
	.suspend = yas_gyro_suspend,
	.resume = yas_gyro_resume,
};
#endif

/* -------------------------------------------------------------------------- *
 *  Module init and exit
 * -------------------------------------------------------------------------- */
static int __init yas_gyro_init(void)
{
#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0
    int32_t rc;

    gyro_polling_wq = create_singlethread_workqueue("gyro_polling_wq");
    if(!gyro_polling_wq)
    {
		pr_err("can't create queue : gyro_polling_wq \n");
		rc = -ENOTSUPP;
    }

    rc = i2c_add_driver(&yas_gyro_driver);
    if (rc != 0) {
        pr_err("can't add i2c driver\n");
		if(gyro_polling_wq != NULL){
			flush_workqueue(gyro_polling_wq);
			destroy_workqueue(gyro_polling_wq);
			gyro_polling_wq = NULL;
		}
        rc = -ENOTSUPP;
    }
    return rc;
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1
	return spi_register_driver(&yas_gyro_driver);
#endif
}
module_init(yas_gyro_init);

static void __exit yas_gyro_exit(void)
{
#if CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 0
	if(gyro_polling_wq != NULL){
        flush_workqueue(gyro_polling_wq);
        destroy_workqueue(gyro_polling_wq);
        gyro_polling_wq = NULL;
    }

	i2c_del_driver(&yas_gyro_driver);
#elif CONFIG_INPUT_YAS_GYROSCOPE_INTERFACE == 1
	spi_unregister_driver(&yas_gyro_driver);
#endif
}
module_exit(yas_gyro_exit);

MODULE_DESCRIPTION("gyro kernel driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(YAS_GYRO_KERNEL_VERSION);

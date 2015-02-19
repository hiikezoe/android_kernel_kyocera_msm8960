/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/input/touchscreen/pixart.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/time.h>
#include <asm/irq.h>

#include <linux/i2c/pixart.h>
#include "pixart-fw.h"
#include "ts_ctrl.h"

#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/regulator/consumer.h>

#define PIXART_DRIVER_MOD_TIME "2012/10/30"
#define PIXART_DRIVER_VERSION "Pixart AMRI/PAP1100 v2.10"

#define MODULE_NAME "pixart: "

#define PIX_DEV_DBG(fmt, arg...)	if(ts_log_level & 0x01) printk(fmt, ## arg)
#define PIX_DEV_INFO(fmt, arg...)	if(ts_log_level & 0x02) printk(fmt, ## arg)
#define PIX_DEV_TOUCH(fmt, arg...)	if(ts_log_level & 0x04) printk(fmt, ## arg)
#define PIX_DEV_I2C(fmt, arg...)	if(ts_log_level & 0x08) printk(fmt, ## arg)

#define TMP_FACTORY_TEST

/**
 *	Variables
 */
#ifdef REGULATOR_ON
static struct regulator *reg_touch;
#endif
static uint8_t pid = 0;
static struct workqueue_struct *pixart_wq;
static struct sysfs_data_ {
	uint8_t command;
	uint8_t reg[3];
	uint8_t data;
	uint8_t number;
	uint8_t val0;
	uint8_t val1;
} sdata;

struct pixart_data {
    struct i2c_client *client;
    struct input_dev *input;
    unsigned int irq;
    struct delayed_work esdwork;
    uint8_t touch_number;
    struct early_suspend early_suspend;
    struct mutex lock;
    struct cdev device_cdev;
    struct completion init_done;
    struct pixart_touch_data_gen touch_data;
    enum ts_config_type config_status;
    enum ts_config_type last_config_status;
    struct ts_config_nv config_nv[TS_NV_MAX];
    bool is_enable;
    bool is_set;
    u8 *motion_data;
    int Touch_Status;
    u8 err_cnt;
    u8 probe_status;
};

/**
 * Function prototypes
 */
static int pixart_init_panel(struct pixart_data *ts);
static void pixart_error_check_process(u8 val, struct pixart_data *ts);
static long pixart_set_nv(struct pixart_data *ts);
static int pixart_flash_save(struct pixart_data *ts, u8 dst);
static void pixart_report_clear(struct pixart_data *ts);
static long pixart_switch_config(struct device *dev);
static long pixart_pixel_dump(struct pixart_data *ts, u8 *p_pixel_dump);

static int pixart_r_reg_byte(struct i2c_client *client, uint8_t reg, uint8_t *val)
{
	int i;
	unsigned char data[1];

	if (!client->adapter)
		return -ENODEV;

	for(i = 0; i < PIXART_RW_RETRY; i++){
		if(i2c_smbus_read_i2c_block_data(client, reg, 1, (uint8_t *) data) == 1){
			PIX_DEV_I2C("%s: add: %02x, val: %02x\n",__func__, reg , data[0]);
			*val = data[0];
			return 0;
		}
		msleep(PIXART_I2C_WAIT);
	}
	pr_err("%s:Err add = 0x%02x, val = 0x%02x\n",__func__ , reg, data[0]);
	return -EIO;
}

static int pixart_r_reg_nbytes(struct i2c_client *client,
		uint8_t readAddress, uint8_t *pdata, uint8_t len)
{
	uint8_t i;

	if (!client->adapter)
		return -ENODEV;

	for(i = 0; i < PIXART_RW_RETRY; i++){
		if(i2c_smbus_read_i2c_block_data(client, readAddress, len, pdata)){
			if(ts_log_level & 0x08){
				for(i=0; i<len; i++){
					printk("%s: add: %02x, val: %02x\n",__func__, readAddress, *pdata);
					pdata++;
				}
			}
			return 0;
		}
		msleep(PIXART_I2C_WAIT);
	}
	pr_err("%s:Err add = 0x%02x\n",__func__ , readAddress);
	return -EIO;

}

static int pixart_w_reg_byte(struct i2c_client *client,
		uint8_t reg, uint8_t val)
{
	int i;

	if (!client->adapter)
		return -ENODEV;

	for(i = 0; i < PIXART_RW_RETRY; i++){
		if(i2c_smbus_write_i2c_block_data(client, reg, 1, &val) == 0){
			PIX_DEV_I2C("%s: add: %02x, val: %02x\n",__func__, reg , val);
			return 0;
		}
		msleep(PIXART_I2C_WAIT);
	}
	pr_err("%s:Err add = 0x%02x, val = 0x%02x\n",__func__ , reg, val);
	return -EIO;
}

static int pixart_w_na_reg(struct i2c_client *client,
		uint8_t index, uint8_t val)
{
	int error;

	if (!client->adapter)
		return -ENODEV;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_REG_GEN2,
				  PIXART_EX_NA_CHOOSE_ACTION_INDEX);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_VAL_GEN2,
				  index);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_REG_GEN2,
				  PIXART_EX_NA_CHOOSE_NA);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_VAL_GEN2,
				  val);

	return error;
}

static int pixart_r_na_reg(struct i2c_client *client,
		uint8_t index, uint8_t *val)
{
	int error;

	if (!client->adapter)
		return -ENODEV;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_REG_GEN2,
				  PIXART_EX_NA_CHOOSE_ACTION_INDEX);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_VAL_GEN2,
				  index);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_REG_GEN2,
				  PIXART_EX_NA_CHOOSE_NA);
	if (error)
		return error;

	error = pixart_r_reg_byte(client, PIXART_REG_EXTENDED_VAL_GEN2, val);
	if (error)
		return error;

	return error;
}

static int pixart_enable_irq(struct pixart_data *ts)
{
	PIX_DEV_DBG("%s() is called.\n",__func__);
	mutex_lock(&ts->lock);
	if (ts->is_enable)
		goto done;

	ts->is_enable = true;
	enable_irq(ts->client->irq);
done:
	mutex_unlock(&ts->lock);
	return 0;
}

static void pixart_disable_irq(struct pixart_data *ts)
{
	PIX_DEV_DBG("%s() is called.\n",__func__);
	mutex_lock(&ts->lock);
	if (!ts->is_enable)
		goto done;

	disable_irq_nosync(ts->client->irq);
	ts->is_enable = false;
done:
	mutex_unlock(&ts->lock);
}

static int pixart_set_config(struct pixart_data *ts)
{
	const struct pixart_platform_data *pdata = ts->client->dev.platform_data;
	int ret, i;
	uint8_t val;
	u8 flash = 0;

	for(i=0; i<pdata->config_length; ){
		ret = pixart_r_reg_byte(ts->client, pdata->config[i], &val);
		if(ret){
			pr_err("%s: Error Read pdata->config[%d] = %x\n",
					__func__, i, pdata->config[i]);
			return ret;
		}
		if(val != pdata->config[i+2]){
			printk("%s: Add->%02x, Value->%02x\n",__func__,pdata->config[i], pdata->config[i+2]);
			ret = pixart_w_reg_byte(ts->client, pdata->config[i], pdata->config[i+2]);
			if(ret){
				pr_err("%s: Error Write register\n",__func__);
				return ret;
			}
			flash = 1;
		}
		i += 3;
	}
	ret = pixart_r_reg_byte(ts->client, PIXART_REG_ORIENTATION_GEN2, &val);
	if(ret){
		pr_err("%s: Error Read ORIENTATION\n",__func__);
		return ret;
	}
	val = val & 0xF8;
	ret = pixart_w_reg_byte(ts->client, PIXART_REG_ORIENTATION_GEN2, 0x03 | val);
	if(ret){
		pr_err("%s: Error Write ORIENTATION\n",__func__);
		return ret;
	}

	ret = pixart_w_reg_byte(ts->client, PIXART_REG_REPORT_RATE_GEN2, 0x78);
	if(ret){
		pr_err("%s: Error Write Report_Rate\n",__func__);
		return ret;
	}

	for(i=0; i<pdata->na_config_length; ){
		ret = pixart_r_na_reg(ts->client, pdata->na_config[i], (uint8_t *)&val);
		if(ret){
			pr_err("%s: Error Read NA register\n",__func__);
			return ret;
		}
		if(val != pdata->na_config[i+1]){
			printk("%s: NA Add->%02x, Val->%02x\n",__func__, pdata->na_config[i], pdata->na_config[i+1]);
			ret = pixart_w_na_reg(ts->client, pdata->na_config[i], pdata->na_config[i+1]);
			if(ret){
				pr_err("%s: Error Write NA register\n",__func__);
				return ret;
			}
			flash = 1;
		}
		i += 2;
	}

	ts->is_set = false;
	if(flash){
		ret = pixart_flash_save(ts, PIXART_REG_FLASH_SAVE_CUST_REGS);
		if(ret == 0){
			pr_info("%s: Success Backup\n",__func__);
			ts->is_set = true;
		}
		else{
			pr_err("%s: Fail Backup\n",__func__);
			ts->is_set = false;
			return ret;
		}
	}

	return ret;
}

static int pixart_reset_and_wait(struct pixart_data *ts)
{
	const struct pixart_platform_data *pdata = ts->client->dev.platform_data;
	int retry, err;
	u8 val = 0;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	if (pdata->reset_hw)
		pdata->reset_hw();

	for(retry = 0; retry < 11; retry++){
		msleep(10);
		err = pixart_r_reg_byte(ts->client, PIXART_REG_PID, &pid);
		if(err){
			pr_err("%s: Error Read PRODUCT_ID\n",__func__);
			return err;
		}
		if(pid == PIXART_5305_PAP1100QN_PID){
			break;
		}
		if(retry == 10) {
			pr_err("%s: No hardware detected.\n",__func__);
			return 1;
		}
	}

	retry = 10;
	while (retry > 0) {
		if (pid == PIXART_5305_PAP1100QN_PID){
			err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
			if(err){
				pr_err("%s: Error Read BOOT Status. PID = %x\n",__func__, pid);
				return err;
			}
		}

		if (val & 0x01)
			break;

		msleep(10);
		retry--;
	}

	if (retry == 0) {
		pr_err("%s: Reset failed\n",__func__);
		return 1;
	}

	msleep(30);

	return 0;

}

static int pixart_reset_status(struct pixart_data *ts)
{
	int ret = 0;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	pixart_report_clear(ts);
	if( ts->config_nv[TS_INITIAL_VALUE_NV].data &&
		ts->config_nv[TS_EXTENDED_NV].data &&
		ts->config_nv[TS_CHARGE_C_NV].data )
		ret = (int)pixart_set_nv(ts);
	else
		ret = pixart_set_config(ts);
	if(!ret){
		PIX_DEV_DBG("%s: Success\n",__func__);
	}else{
		pr_err("%s: error\n",__func__);
	}
	return ret;
}

static ssize_t pixart_mode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{

	struct pixart_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	const struct pixart_platform_data *pdata = ts->client->dev.platform_data;
	ssize_t ret = 0;
	uint8_t i, len;
	int j, err;
	static uint8_t data[128];
	u8 *p_pixel_dump, *p_cat;
	u8 val, crc_hi, crc_lo;

	len = sdata.number;
	switch (sdata.command) {
	case SYSFS_READ:
		if(sdata.reg[0] + len > 0x80){
			strcat(buf, "Error: invalid Register\n");
			ret = strlen(buf) + 1;
			return ret;
		}

		sprintf(buf, "    \t");
		for (i = 0; i < 16; i++) {
			sprintf(data, "%2x ", i);
			strcat(buf, data);
		}

		i = sdata.reg[0] & 0xf0;
		for (; i < ((sdata.reg[0] + len + 15) & 0xf0); i++) {
			/* start at 10 20 ... n0, \t = 4 chars */
			if (!(i & 0xf)) {
				sprintf(data, "\n%02x :\t", i);
				strcat(buf, data);
			}
			if (i >= sdata.reg[0] && i < sdata.reg[0] + len) {
				err = pixart_r_reg_byte(ts->client, i, &val);
				if(err){
					strcat(buf, "Error Read register\n");
					ret = strlen(buf) + 1;
					return ret;
				}
				sprintf(data, "%02x ", val);
				strcat(buf, data);
			}
			else {
				sprintf(data, "   ");
				strcat(buf, data);
			}
		}

		strcat(buf, "\n");
		ret = strlen(buf) + 1;
		break;

	case SYSFS_NA_READ:
		pixart_r_na_reg(ts->client, 0x00, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x00 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x04, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x04 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x0C, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x0C , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x57, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x57 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x58, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x58 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x59, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x59 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x5A, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x5A , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x5B, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x5B , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x5C, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x5C , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x5D, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x5D , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x5E, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x5E , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x5F, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x5F , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x60, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x60 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x61, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x61 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x62, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x62 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x64, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x64 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x65, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x65 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x66, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x66 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x6A, &val);
		ret += scnprintf(buf + ret, 0x6A - ret,
							"NA_READ Addr:0x00 , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		pixart_r_na_reg(ts->client, 0x6B, &val);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"NA_READ Addr:0x6B , Val:0x%02x\n",val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		break;

	case SYSFS_READ3N:
		for (i = 0; i < len; i++) {
			err = pixart_r_reg_nbytes(ts->client, sdata.reg[0], data, 3);
			if(err){
				strcat(buf, "Error Read register\n");
				ret = strlen(buf) + 1;
				return ret;
			}
			sprintf(&data[3], " %02x %02x %02x", data[0],data[1],data[2]);
			strcat(buf, &data[3]);
			if (ts_log_level & 0x10)
				pr_debug(MODULE_NAME "triple read: %02x %02x %02x\n",
					data[0],data[1],data[2]);
		}

		ret = strlen(buf) + 1;
		break;

	case SYSFS_READ2N:		
		if(sdata.reg[0] > 0x7E || sdata.reg[1] > 0x7E){
			strcat(buf, "Error: invalid Register\n");
			ret = strlen(buf) + 1;
			return ret;
		}

		for (i = 0; i < len; i++) {
			err = pixart_r_reg_byte(ts->client, sdata.reg[0], &data[0]);
			if(err){
				sprintf(data, "Error Read register. Add = 0x%x\n", sdata.reg[0]);
				strcat(buf, data);
				ret = strlen(buf) + 1;
				return ret;
			}
			err = pixart_r_reg_byte(ts->client, sdata.reg[1], &data[1]);
			if(err){
				sprintf(data, "Error Read register. Add = 0x%x\n", sdata.reg[1]);
				strcat(buf, data);
				ret = strlen(buf) + 1;
				return ret;
			}

			sprintf(&data[2], "%02x %02x\n", data[0], data[1]);
			strcat(buf, &data[2]);
			if (ts_log_level & 0x10)
				pr_debug(MODULE_NAME "double read: %02x %02x %02x\n",
					data[0], data[1], data[2]);
		}

		ret = strlen(buf) + 1;
		break;

	case SYSFS_WRITE_READ:
		err = pixart_w_reg_byte(ts->client, sdata.reg[0], sdata.val0);
		if(err){
			sprintf(data, "Error Write Add: %x, Val: %x\n", sdata.reg[0], sdata.val0);
			strcat(buf, data);
			ret = strlen(buf) + 1;
			return ret;
		}
		err = pixart_w_reg_byte(ts->client, sdata.reg[1], sdata.val1);
		if(err){
			sprintf(data, "Error Write Add: %x, Val: %x\n", sdata.reg[1], sdata.val1);
			strcat(buf, data);
			ret = strlen(buf) + 1;
			return ret;
		}
		sprintf(buf, " ");
		for (i = 0; i < len; i++) {
			err = pixart_r_reg_byte(ts->client, sdata.reg[2], &val);
			if(err){
				sprintf(data, "Error Read register. Add = 0x%x\n", sdata.reg[2]);
				strcat(buf, data);
				ret = strlen(buf) + 1;
				return ret;
			}
			sprintf(data, "%02x ", val);
			strcat(buf, data);
		}
		ret = strlen(buf) + 1;
		break;

	case SYSFS_VERSION:
		sprintf(buf, " ");
		sprintf(data, "%s", PIXART_DRIVER_VERSION);
		strcat(buf, data);
		strcat(buf, "\n ");
		break;

	case SYSFS_FIRMWARE:
		sprintf(buf, " ");
		sprintf(data, "%02x", sdata.val0);
		strcat(buf, data);
		strcat(buf, "\n ");
		break;

	case SYSFS_KDBGLEVEL:
		sprintf(buf, " ");
		sprintf(data, "%02x", ts_log_level);
		strcat(buf, data);
		strcat(buf, "\n ");
		break;

	case SYSFS_PIXEL_DUMP:
		mutex_lock(&ts->lock);
		p_pixel_dump = kcalloc(PIXEL_DUMP, sizeof(u8), GFP_KERNEL);
		if (!p_pixel_dump) {
			pr_err("%s: Failed to allocate memory!\n",__func__);
			sprintf(buf, "ENOMEM\n");
			ret = strlen(buf) + 1;
			mutex_unlock(&ts->lock);
			break;
		}
		err = (int)pixart_pixel_dump(ts, p_pixel_dump);
		if(err){
			pr_err("%s: Failed Pixel Dump\n", __func__);
			kfree(p_pixel_dump);
			p_pixel_dump = NULL;
			mutex_unlock(&ts->lock);
			sprintf(buf, "err=%x\n",err);
			ret = strlen(buf) + 1;
			break;
		}
		p_cat = p_pixel_dump;
		mutex_unlock(&ts->lock);
		for(j = 0; j < PIXEL_DUMP; j++){
			sprintf(data, "%02x", *p_cat);
			strcat(buf, data);
			p_cat++;
		}
		kfree(p_pixel_dump);
		p_pixel_dump = NULL;
		strcat(buf, "\n");
		ret = strlen(buf) + 1;
		break;

	case SYSFS_STATUS:
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"ts->init_done.done = %x\n", ts->init_done.done);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"ts->cilent->irq = %d\n", ts->client->irq);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"ts->is_enable = %d\n", ts->is_enable);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"ts->Touch_Status = %d\n", ts->Touch_Status);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"ts->config_status = %d\n", ts->config_status);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"ts->last_config_status = %d\n", ts->last_config_status);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"ts->lock.count.counter = %d\n", ts->lock.count.counter);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		val = gpio_get_value(pdata->irq_gpio);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"Int Signal = %d\n", val);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"Err Count = %d\n", ts->err_cnt);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"ts->is_set = %d\n", ts->is_set);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"ts->probe_status = %d\n", ts->probe_status);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		err = pixart_r_reg_byte(ts->client, PIXART_REG_FW_REVID_GEN2, &val);
		if(err){
			strcat(buf, "Error Read FW register\n");
			ret = strlen(buf) + 1;
			return ret;
		}
		err = pixart_w_reg_byte(ts->client, PIXART_REG_CHECHK_FW_CRC, 0x02);
		if(err){
			strcat(buf, "Error Write FW-CRC check\n");
			ret = strlen(buf) + 1;
			return ret;
		}
		msleep(500);
		err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_HI_GEN2, &crc_hi);
		if(err){
			strcat(buf, "Error Read FW-CRC_HI register\n");
			ret = strlen(buf) + 1;
			return ret;
		}
		err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_LO_GEN2, &crc_lo);
		if(err){
			strcat(buf, "Error Read FW-CRC_LO register\n");
			ret = strlen(buf) + 1;
			return ret;
		}

		err = pixart_w_reg_byte(ts->client, PIXART_REG_CHECHK_FW_CRC, 0x00);
		if(err){
			strcat(buf, "Error Write FW-CRC Init\n");
			ret = strlen(buf) + 1;
			return ret;
		}

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"FW%02x, CRC = %02x%02x\n", val, crc_hi, crc_lo);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;

		err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, 0x04);
		if(err){
			strcat(buf, "Error Write DS-CRC check\n");
			ret = strlen(buf) + 1;
			return ret;
		}
		msleep(500);
		err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_HI_GEN2, &crc_hi);
		if(err){
			strcat(buf, "Error Read DS-CRC_HI register\n");
			ret = strlen(buf) + 1;
			return ret;
		}
		err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_LO_GEN2, &crc_lo);
		if(err){
			strcat(buf, "Error Read DS-CRC_LO register\n");
			ret = strlen(buf) + 1;
			return ret;
		}

		err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, 0x00);
		if(err){
			strcat(buf, "Error Write FW-CRC Init\n");
			ret = strlen(buf) + 1;
			return ret;
		}

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"Drive-Sense CRC = %02x%02x\n", crc_hi, crc_lo);
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		break;

	case SYSFS_TOUCH_DATA:
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"Touch status = %x\n", ts->touch_data.status);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"Touch total_touch = %x\n", ts->touch_data.total_touch);
		for(i = 0; i < PIXART_MAX_FINGER; i++){
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"Touch slot_id[%d] = %x, TOOL_FINGER = %d\n",
							i, ts->touch_data.slot[i].id, ts->touch_data.slot[i].tool_finger);
		}
		for(i = 0; i < PIXART_MAX_FINGER; i++){
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"Touch data_id[%d] = %x\n", i, ts->touch_data.point_data[i].id);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"	x, y = %x, %x\n",
							ts->touch_data.point_data[i].x, ts->touch_data.point_data[i].y);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
							"	force = %x, area = %x\n",
							ts->touch_data.point_data[i].force, ts->touch_data.point_data[i].area);
		}
		if(ret >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		break;

	default:
		break;
	}

	return ret;
}

static ssize_t pixart_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct pixart_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	static char data[128];
	const char *p;
	char temp;
	int n;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	switch (buf[0]) {
	case 'r':
		sdata.command = SYSFS_READ;
		sscanf(buf, "%c %x %x", data, (unsigned int *) &sdata.reg[0],
				(unsigned int *) &sdata.number);
		PIX_DEV_DBG("%s: read mode: %02x %02x \n",__func__, sdata.reg[0],
				sdata.number);
		break;

	case 'n':
		sdata.command = SYSFS_NA_READ;
		PIX_DEV_DBG("%s: NA READ\n",__func__);
		break;

	case 'w':
		sdata.command = SYSFS_WRITE;
		p = buf + 1;
		while (sscanf(p, "%x %x%n", (unsigned int *) &sdata.reg[0],
				(unsigned int *) &sdata.data, &n) == 2) {
			p += n;
			if(sdata.reg[0] >0x7E){
				pr_err("%s:Error: invalid Register\n",__func__);
				break;
			}
			if(pixart_w_reg_byte(ts->client, sdata.reg[0], sdata.data)){
				pr_err("%s: Error Write Add: %x, Val: %x\n",
						__func__, sdata.reg[0], sdata.data);
			}
			PIX_DEV_DBG("%s: write mode: %02x %02x \n",__func__,
				sdata.reg[0], sdata.data);
		}
		break;

	case 'b':
		p = buf + 1;
		sdata.command = SYSFS_BURST;
		/* find out what address to write to */
		/* %n how many chars consumed this time */
		sscanf(p, "%x%n", (unsigned int *) &sdata.reg[0], &n);
		p += n;
		/* write the data bytes which follow */
		while (sscanf(p, "%x%n", (unsigned int *) &temp, &n) == 1) {
			p += n;
			pixart_w_reg_byte(ts->client, sdata.reg[0], temp);
			PIX_DEV_DBG("%s: burst write %02x %02x \n",__func__, sdata.reg[0], temp);
		}
		break;

	case 't':
		sdata.command = SYSFS_READ3N;
		sscanf(buf, "%c %x %x %x %x", data, (unsigned int *) &sdata.reg[0],
			   (unsigned int *) &sdata.reg[1],
			   (unsigned int *) &sdata.reg[2],
			   (unsigned int *) &sdata.number);
		PIX_DEV_DBG("%s: triple read mode: 0x%02x 0x%02x 0x%02x x 0x%02x times\n"
				,__func__ , sdata.reg[0], sdata.reg[1], sdata.reg[2], sdata.number);
		break;

	case 'm':
		sdata.command = SYSFS_WRITE_READ;
		sscanf(buf, "%c %x %x %x %x %x %x", data, (unsigned int *) &sdata.reg[0],
			   (unsigned int *) &sdata.val0,
			   (unsigned int *) &sdata.reg[1],
			   (unsigned int *) &sdata.val1,
			   (unsigned int *) &sdata.reg[2],
			   (unsigned int *) &sdata.number);
		PIX_DEV_DBG("%s: write 2, read 1 values set\n",__func__);
		break;

	case 'v':
		sdata.command = SYSFS_VERSION;
		pr_info(MODULE_NAME "Ver: %s\n", PIXART_DRIVER_VERSION);
		break;

	case 'f':
		sdata.command = SYSFS_FIRMWARE;
		sdata.val0 = (uint8_t) pixart_init_panel(ts);
		break;

	case 'l':
		sdata.command = SYSFS_KDBGLEVEL;
		sscanf(buf, "%c %x ", data, (unsigned int *)&ts_log_level);
		PIX_DEV_DBG("%s: touch debug level changed : 0x%02x\n",__func__,ts_log_level);
		break;

	case 'p':
		pr_info(MODULE_NAME "trigger kernel panic\n");
		BUG_ON(1);
		break;

	case 'e':
		p = buf + 1;
		sscanf(p, "%x", (int *)&ts_esd_recovery);
		PIX_DEV_DBG("%s: ts_esd_recovery = %x\n",__func__, (int)ts_esd_recovery);
		break;

	case 'u':
		sdata.command = SYSFS_READ2N;
		sscanf(buf, "%c %x %x %x", data, (unsigned int *) &sdata.reg[0],
			   (unsigned int *) &sdata.reg[1],
			   (unsigned int *) &sdata.number);
		if (ts_log_level & 0x10)
			pr_debug(MODULE_NAME "double read mode: 0x%02x 0x%02x x "
				"0x%02x times\n", sdata.reg[0], sdata.reg[1], sdata.number);
		break;

	case 'g':
		sdata.command = SYSFS_PIXEL_DUMP;
		PIX_DEV_DBG("%s: pixel-dump in store\n", __func__);
		break;

	case 's':
		sdata.command = SYSFS_STATUS;
		PIX_DEV_DBG("%s: Get Status\n", __func__);
		break;

	case 'c':
		sdata.command = SYSFS_TOUCH_DATA;
		PIX_DEV_DBG("%s: Get Touch Data\n", __func__);
		break;

	default:
		break;
	}

	return count;
}

static DEVICE_ATTR(ctrl, S_IRUGO|S_IWUSR, pixart_mode_show, pixart_mode_store);

static struct attribute *tp_attributes[] = {
	&dev_attr_ctrl.attr,
	NULL
};

static const struct attribute_group tp_attr_group = {
	.attrs = tp_attributes,
};

static int pixart_sysfs_init(struct pixart_data *ts)
{
	int ret = 0;

	ret = sysfs_create_group(&ts->client->dev.kobj, &tp_attr_group);
	if (ret) {
		pr_err( "%s: subsystem_register failed\n",__func__);
		ret = -ENOMEM;
	}

	return ret;
}

static int pixart_flash_save(struct pixart_data *ts, u8 dst)
{
	u8 val;
	int i, err;

	PIX_DEV_DBG("%s() is called.\n",__func__);

	err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_ENABLE, PIXART_FLASH_ENABLE);
	if(err){
		pr_err("%s: Error Write FLASH_ENABLE\n",__func__);
		return err;
	}
	msleep(1);
	err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, dst);
	if(err){
		pr_err("%s: Error Write FLASH_CTL\n",__func__);
		return err;
	}
	msleep(300);

	i = 0;
	err = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, &val);
	if(err){
		pr_err("%s: Error Read FLASH\n", __func__);
		return err;
	}
	while((val & 0x01) && (i*10 < PIXART_TIMEOUT_MS_AFTER_FLASH_ENABLE_GEN2)){
		i++;
		mdelay(10);
		err = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, &val);
		if(err){
			pr_err("%s: Error Read FLASH\n", __func__);
			return err;
		}
	}

	if(i == 50)
		return 1;

	return 0;
}

static int pixart_check_and_load_fw(struct pixart_data *ts)
{
	int i, len, retry, err;
	uint8_t val;
	uint8_t *pfw;

	err = pixart_r_reg_byte(ts->client, PIXART_REG_FW_REVID_GEN2, &val);
	if(err){
		pr_err("%s: Error Read FW_REVID\n", __func__);
		return err;
	}
	if(val >= PIXART_LATEST_FW){
		pr_info("%s:This FW is latest 0x%02x\n",__func__, val);
		retry = 0;
		while(1){
			err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
			if(err){
				pr_err("%s:Error Read BOOT Stat\n", __func__);
				return err;
			}
			if((val & 0x01) && (val & 0x80))
				break;
			msleep(10);
			retry++;
			if(retry > 50){
				pr_err("%s:Time Out\n",__func__);
				return -1;
			}
		}
		err = pixart_r_reg_byte(ts->client, PIXART_REG_ERROR_ID_GEN2, &val);
		if(err){
			pr_err("%s:ReadError PIXART_REG_ERROR_ID_GEN2\n", __func__);
			return err;
		}
		if(val == 0x50){
			err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_ENABLE, 0xFE);
			if(err){
				pr_err("%s: Error Write Flash\n",__func__);
				return err;
			}
			msleep(1);
			err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, 0xC0);
			if(err){
				pr_err("%s: Error Write Flash\n",__func__);
				return err;
			}
			msleep(100);
			retry = 0;
			while(1){
				err = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, &val);
				if(err){
					pr_err("%s:ReadError PIXART_REG_FLASH_CTL_GEN2\n", __func__);
					return err;
				}
				if(val == 0xc0){
					pr_info("%s: Success\n",__func__);
					break;
				}
				msleep(10);
				retry++;
				if(retry > 20){
					pr_err("%s: Time Out\n",__func__);
					return -1;
				}
			}
		}
		return 0;
	}

	len = sizeof (firmware0C);
	pfw = firmware0C;

	retry = 0;

	if(val == 0){
		err = pixart_w_reg_byte(ts->client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);
		if(err){
			pr_err("%s: Error Write SHUTDOWN\n",__func__);
			return err;
		}
		msleep(1);
		err = pixart_w_reg_byte(ts->client, PIXART_REG_SHUTDOWN, PIXART_RESET);
		if(err){
			pr_err("%s: Error Write RESET\n",__func__);
			return err;
		}
		while(1){
			msleep(10);
			err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
			if(err){
				pr_err("%s:Error Read BOOT Stat\n", __func__);
				return err;
			}
			if(val & 0x01)
				break;
			retry++;
			if(retry > 100){
				pr_err("%s:Failed Reset_1\n",__func__);
				return -1;
			}
		}
		msleep(30);
		err = pixart_r_reg_byte(ts->client, PIXART_REG_FW_REVID_GEN2, &val);
		if(err){
			pr_err("%s:Error Read FW_REVID\n", __func__);
			return err;
		}
		if(val == 0){
			pr_info("%s:FW_REV_ID = 0x00\n",__func__);
			goto boot_status;
		}
	}
	err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_ENABLE, PIXART_FLASH_ENABLE);
	if(err){
		pr_err("%s: Error Write FLASH_ENABLE\n",__func__);
		return err;
	}
	msleep(1);
	err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, PIXART_REG_FLASH_NO_SAVE_CUST_REGS);
	if(err){
		pr_err("%s: Error Write FLASH_CTL\n",__func__);
		return err;
	}
	msleep(300);
	retry = 0;
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, &val);
		if(err){
			pr_err("%s:Error Read FLASH\n", __func__);
			return err;
		}
		if((val & 0x01) == 0)
			break;
		msleep(10);
		retry++;
		if(retry > 20){
			pr_err("%s:Failed Flash CTL\n",__func__);
			return -1;
		}
	}

boot_status:
	retry = 0;
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
		if(err){
			pr_err("%s:Error Read BOOT_Stat\n", __func__);
			return err;
		}
		if(val & 0x01)
			break;
		msleep(10);
		retry++;
		if(retry > 100){
			pr_err("%s:Failed Reset_2\n",__func__);
			return -1;
		}
	}
	err = pixart_w_reg_byte(ts->client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);
	if(err){
		pr_err("%s: Error Write SHUTDOWN\n",__func__);
		return err;
	}
	msleep(1);
	err = pixart_w_reg_byte(ts->client, PIXART_REG_SHUTDOWN, PIXART_RESET_TO_ROM_GEN2);
	if(err){
		pr_err("%s: Error Write RESET\n",__func__);
		return err;
	}
	msleep(10);

	retry = 0;
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
		if(err){
			pr_err("%s:Error Read BOOT_Stat\n", __func__);
			return err;
		}
		if(val & 0x01)
			break;
		msleep(10);
		retry++;
		if(retry > 100){
			pr_err("%s:Failed Reset_3\n",__func__);
			return -1;
		}
	}

	mdelay(30);
	retry = 0;
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_FW_REVID_GEN2, &val);
		if(err){
			pr_err("%s:Error Read FW_REVID\n", __func__);
			return err;
		}
		if(val == 0x00)
			break;
		msleep(10);
		retry++;
		if(retry > 50){
			pr_err("%s:FW_REV_ID is not 0x00\n",__func__);
			return -1;
		}
	}

	err = pixart_w_reg_byte(ts->client, PIXART_REG_WD_DISABLE, PIXART_WD_DISABLE);
	if(err){
		pr_err("%s: Error Write DISABLE\n",__func__);
		return err;
	}
	err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_ENABLE, PIXART_FLASH_ENABLE);
	if(err){
		pr_err("%s: Error Write ENABLE\n",__func__);
		return err;
	}
	msleep(1);
	err = pixart_w_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, 0x00);
	if(err){
		pr_err("%s: Error Write BOOT_Stat\n",__func__);
		return err;
	}
	err = pixart_w_reg_byte(ts->client, PIXART_REG_IODL_CTL_GEN2, PIXART_ENABLE_DL_GEN2);
	if(err){
		pr_err("%s: Error Write IODL_CTL\n",__func__);
		return err;
	}
	msleep(10);

	retry = 0;
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_IODL_CTL_GEN2, &val);
		if(err){
			pr_err("%s:Error Read ICOTL_CTL\n", __func__);
			return err;
		}
		if((val & 0x80) == 0)
			break;
		msleep(10);
		retry++;
		if(retry > 200){
			pr_err("%s:Failed DownLoad CTL 0x%x\n",__func__,val);
			return -1;
		}
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_ERROR_ID_GEN2, &val);
	if(err){
		pr_err("%s:Error Read ERROR_ID\n", __func__);
		return err;
	}
	if(val){
		pr_err("%s:IC ERROR. ID = %x\n", __func__, val);
		return -1;
	}

	for(i = 0; i < len; i++){
		err = pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, *(pfw + i));
		if(err){
			pr_err("%s: Error Write IODL_DATA\n",__func__);
			return err;
		}
	}

	retry = 0;
	while(1){
		msleep(10);
		err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
		if(err){
			pr_err("%s:Error Read BOOT_Stat\n", __func__);
			return err;
		}
		if((val & 0x01) && (val & 0x02))
			break;
		retry++;
		if(retry > 100){
			pr_err("%s:Error Boot Status = 0x%x\n",__func__, val);
			return -1;
		}
	}

	msleep(70);
	err = pixart_w_reg_byte(ts->client, PIXART_REG_WD_DISABLE, PIXART_WD_ENABLE);
	if(err){
		pr_err("%s: Error Write DISABLE\n",__func__);
		return err;
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_FW_REVID_GEN2, &val);
	if(err){
		pr_err("%s:Error Read FW_REVID\n", __func__);
		return err;
	}
	if(val != PIXART_LATEST_FW){
		pr_err("%s:Not latest FW. This FW is 0x%x\n",__func__, val);
		return -1;
	}

	retry = 0;
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
		if(err){
			pr_err("%s:Error Read BOOT_Stat\n", __func__);
			return err;
		}
		if(val & 0x80)
			break;
		msleep(10);
		retry++;
		if(retry > 100){
			pr_err("%s:Error Boot Status = 0x%x\n",__func__, val);
			return -1;
		}
	}

	err = pixart_w_reg_byte(ts->client, PIXART_REG_CHECHK_FW_CRC, 0x02);
	if(err){
		pr_err("%s:Error Write FW-CRC check\n",__func__);
		return err;
	}
	retry = 0;
	while(1){
		msleep(10);
		err = pixart_r_reg_byte(ts->client, PIXART_REG_CHECHK_FW_CRC, &val);
		if(err){
			pr_err("%s:Error Read FLASH_CTL\n", __func__);
			return err;
		}
		if(val & 0x80)
			break;
		retry++;
		if(retry > 50){
			pr_err("%s:CRC Mode Error Val = 0x%02x\n",__func__, val);
			return -1;
		}
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_HI_GEN2, &val);
	if(err){
		pr_err("%s:Error Read FW-CRC_HI\n",__func__);
		return err;
	}
	if(val != PIXART_FW_CRC_HIGH){
		pr_err("%s:Error FW-CRC_HI, \n",__func__);
		return -1;
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_LO_GEN2, &val);
	if(err){
		pr_err("%s:Error Read FW-CRC_LO\n",__func__);
		return err;
	}
	if(val != PIXART_FW_CRC_LOW){
		pr_err("%s:Error FW-CRC_LO\n",__func__);
		return -1;
	}

	err = pixart_w_reg_byte(ts->client, PIXART_REG_CHECHK_FW_CRC, 0x00);
	if(err){
		pr_err("%s:Error Write FW-CRC Init\n",__func__);
		return err;
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_ERROR_ID_GEN2, &val);
	if(err){
		pr_err("%s:ReadError PIXART_REG_ERROR_ID_GEN2\n", __func__);
		return err;
	}
	if(val == 0x50){
		err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_ENABLE, 0xFE);
		if(err){
			pr_err("%s: Error Write Flash\n",__func__);
			return err;
		}
		msleep(1);
		err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, 0xC0);
		if(err){
			pr_err("%s: Error Write Flash\n",__func__);
			return err;
		}
		msleep(100);
		retry = 0;
		while(1){
			err = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, &val);
			if(err){
				pr_err("%s:ReadError PIXART_REG_FLASH_CTL_GEN2\n", __func__);
				return err;
			}
			if(val == 0xc0){
				pr_info("%s: Success\n",__func__);
				break;
			}
			msleep(10);
			retry++;
			if(retry > 20){
				pr_err("%s: Time Out\n",__func__);
				return -1;
			}
		}
	}

	pr_info("%s:FW Update is complete\n",__func__);
	return 0;
}

static int pixart_check_and_load_ds_map(struct pixart_data *ts)
{
	int i,retry, len, err;
	uint8_t *pmap, val, val2;

	err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, 0x04);
	if(err){
		pr_err("%s: Error Write CRC_CTL\n",__func__);
		return err;
	}
	msleep(1);
	retry = 0;
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, &val);
		if(err){
			pr_err("%s:Error Read FLASH_CTL\n", __func__);
			return err;
		}
		if((val & 0x01) == 0)
			break;
		msleep(10);
		retry++;
		if(retry > 50){
			pr_err("%s:CRC Mode Error Val = 0x%02x\n",__func__, val);
			return -1;
		}
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_HI_GEN2, &val);
	if(err){
		pr_err("%s:Error Read CRC_HI\n", __func__);
		return err;
	}
	err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_LO_GEN2, &val2);
	if(err){
		pr_err("%s:Error Read CRC_LO\n", __func__);
		return err;
	}

	err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, 0x00);
	if(err){
		pr_err("%s:Error Write FW-CRC Init\n",__func__);
		return err;
	}

	if(val == PIXART_CRC_HIGH && val2 == PIXART_CRC_LOW){
		pr_info("%s:CRC info is normal\n",__func__);
		return 0;
	} else{
		pr_err("%s:CRC info is abnormal. CRC = 0x%2x%2x\n",__func__, val, val2);
		
		err = pixart_w_reg_byte(ts->client, PIXART_REG_WD_DISABLE, PIXART_WD_DISABLE);
		if(err){
			pr_err("%s: Error Write DISABLE\n",__func__);
			return err;
		}
		err = pixart_w_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, 0x00);
		if(err){
			pr_err("%s: Error Write BOOT_Stat\n",__func__);
			return err;
		}
		err = pixart_w_reg_byte(ts->client, PIXART_REG_IODL_CTL_GEN2, PIXART_VALUE_DS_MAP_DL);
		if(err){
			pr_err("%s: Error Write IODL_CTL\n",__func__);
			return err;
		}
		msleep(1);

		err = pixart_r_reg_byte(ts->client, PIXART_REG_ERROR_ID_GEN2, &val);
		if(err){
			pr_err("%s:Error Read ERROR_ID\n", __func__);
			return err;
		}
		if(val){
			pr_err("%s:IC ERROR. ID = %x\n", __func__, val);
			return -1;
		}

		err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
		if(err){
			pr_err("%s:Error Read BOOT_Stat\n", __func__);
			return err;
		}
		err = pixart_w_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, (val & 0x7F));
		if(err){
			pr_err("%s: Error Write BOOT_Stat\n",__func__);
			return err;
		}

		len = sizeof (ds_map);
		pmap = ds_map;
		for(i = 0; i < len; i++){
			err = pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, *(pmap + i));
			if(err){
				pr_err("%s: Error Write IODL_DATA\n",__func__);
				return err;
			}
		}

		retry = 0;
		while(1){
			err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
			if(err){
				pr_err("%s:Error Read BOOT_Stat\n", __func__);
				return err;
			}
			if((val & 0x40) && ((val & 0x20) == 0))
				break;
			msleep(10);
			retry++;
			if(retry > 50){
				pr_err("%s:Error Boot01 Status = 0x%x\n",__func__, val);
				return -1;
			}
		}

		err = pixart_w_reg_byte(ts->client, PIXART_REG_WD_DISABLE, PIXART_WD_ENABLE);
		if(err){
			pr_err("%s: Error Write DISABLE\n",__func__);
			return err;
		}
		msleep(2);

		retry = 0;
		while(1){
			err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
			if(err){
				pr_err("%s:Error Read BOOT_Stat\n", __func__);
				return err;
			}
			if(val & 0x80)
				break;
			msleep(10);
			retry++;
			if(retry > 150){
				pr_err("%s:Error Boot02 Status = 0x%x\n",__func__, val);
				return -1;
			}
		}

		if(pixart_flash_save(ts, PIXART_REG_FLASH_SAVE_DS_MAP)){
			pr_err("%s:Fail FLASH-SAVE\n",__func__);
			return -1;
		}
	}

	err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, 0x04);
	if(err){
		pr_err("%s: Error Write CRC_CTL\n",__func__);
		return err;
	}
	retry = 0;
	while(1){
		msleep(10);
		err = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, &val);
		if(err){
			pr_err("%s:Error Read FLASH_CTL\n", __func__);
			return err;
		}
		if((val & 0x01) == 0)
			break;
		retry++;
		if(retry > 50){
			pr_err("%s:CRC Mode Error Val = 0x%02x\n",__func__, val);
			return -1;
		}
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_HI_GEN2, &val);
	if(err){
		pr_err("%s:Error Read CRC_HI\n", __func__);
		return err;
	}
	err = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_LO_GEN2, &val2);
	if(err){
		pr_err("%s:Error Read CRC_LO\n", __func__);
		return err;
	}

	err = pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, 0x00);
	if(err){
		pr_err("%s:Error Write FW-CRC Init\n",__func__);
		return err;
	}

	if(val != PIXART_CRC_HIGH || val2 != PIXART_CRC_LOW){
		pr_err("%s is DL_err\n", __func__);
		return -1;
	}

	pr_info("%s is complete\n",__func__);
	return 0;
}

static int pixart_init_panel(struct pixart_data *ts)
{
	if (pixart_check_and_load_fw(ts)){
		pr_err("%s: Fail Load FW\n",__func__);
		return -1;
	}

	if (pixart_check_and_load_ds_map(ts)){
		pr_err("%s: Fail Drive_Sense Map DL\n",__func__);
		return -1;
	}

	if (pixart_set_config(ts)){
		pr_err("%s: Fail Set config\n",__func__);
		return -1;
	}

	return 0;
}

static int pixart_diag_data_start(struct pixart_data *ts)
{
	int ret = 0;
	
	PIX_DEV_DBG("%s is called.\n",__func__);
	if(diag_data == NULL){
		diag_data = kzalloc(sizeof(struct ts_diag_type), GFP_KERNEL);
		if (!diag_data) {
			pr_err("%s: Failed to allocate memory!\n",__func__);
			return -ENOMEM;
		}
		memset(diag_data, 0, sizeof(struct ts_diag_type));
	}

	if (ts->client->irq != -1)
		ret = pixart_enable_irq(ts);
	return ret;
}

static void pixart_diag_store(struct pixart_data *ts)
{
	struct pixart_touch_data_gen *touch_data = (struct pixart_touch_data_gen *)&ts->touch_data;
	int i;
	int cnt = 0;

	mutex_lock(&ts->lock);
	memset(diag_data, 0, sizeof(struct ts_diag_type));
	for (i = 0; i < PIXART_MAX_FINGER; i++) {
		if (touch_data->slot[i].id == 0)
			continue;
		diag_data->ts[i].x = touch_data->point_data[i].x;
		diag_data->ts[i].y = touch_data->point_data[i].y;
		diag_data->ts[i].width = touch_data->point_data[i].area;
		PIX_DEV_DBG("%s: touch[%d] x, y, width = %d, %d, %d\n", __func__, i,
							touch_data->point_data[i].x,
							touch_data->point_data[i].y,
							touch_data->point_data[i].area);
		cnt++;
	}
	diag_data->diag_count = cnt;
	PIX_DEV_DBG("%s: diag_count = %d\n", __func__, diag_data->diag_count);
	mutex_unlock(&ts->lock);
}

static int pixart_diag_data_end(struct pixart_data *ts)
{
	PIX_DEV_DBG("%s is called.\n", __func__);
	if (diag_data != NULL) {
		kfree(diag_data);
		diag_data = NULL;
	}
	return 0;
}

static int pixart_reset_process(struct pixart_data *ts)
{
	int err = 0;

	pr_err("%s: IC Reset\n",__func__);
	if (ts->is_enable) {
		disable_irq_nosync(ts->irq);
		ts->is_enable = false;
	}

	err = pixart_reset_and_wait(ts);
	if (err){
		pr_err("%s: Failed to restart!\n",__func__);
		goto done;
	}
	err = pixart_check_and_load_fw(ts);
	if (err){
		pr_err("%s: Fail Load FW\n",__func__);
		goto done;
	}
	err = pixart_check_and_load_ds_map(ts);
	if (err){
		pr_err("%s: Fail Drive_Sense Map DL\n",__func__);
		goto done;
	}
	err = pixart_reset_status(ts);
	if (err){
		pr_err("%s: Failed to reset status!\n",__func__);
		goto done;
	}

done:
	if (!ts->is_enable) {
		enable_irq(ts->irq);
		ts->is_enable = true;
	}
	return err;
}

static void pixart_error_check_process(u8 val, struct pixart_data *ts)
{
	switch (val) {
	case 0x01:
	case 0x02:
	case 0x06:
	case 0x07:
	case 0x08:
	case 0x09:
	case 0x0A:
	case 0x0B:
	case 0x0C:
	case 0x0D:
	case 0x0E:
	case 0x0F:
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x13:
	case 0x14:
	case 0x15:
	case 0x23:
		pr_err("%s: IC Error. ID = 0x%x\n",__func__, val);
		ts->err_cnt++;
		pixart_reset_process(ts);
		break;
	default:
		pr_err("%s: IC Error. ID = 0x%x\n",__func__, val);
		ts->err_cnt++;
		break;
	}
}

static irqreturn_t pixart_interrupt(int irq, void *dev_id)
{
	int i, j, err;
	struct pixart_data *ts = dev_id;
	struct pixart_touch_data_gen *touch_data = (struct pixart_touch_data_gen *)&ts->touch_data;
	uint8_t *pdata, status, val;
	uint8_t move = 0;

	mutex_lock(&ts->lock);
	ts->touch_number = 0;

	if (ts->Touch_Status >= TOUCH_POWEROFF){
		pr_err("%s: Abnormal Status\n", __func__);
		goto end_of_interrupt;
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_STATUS_GEN2, &status);
	if(err){
		pr_err("%s:Error Read Status\n", __func__);
		goto end_of_interrupt;
	}
	if (status & PIXART_REG_STATUS_GEN2_ERROR){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_ERROR_ID_GEN2, &val);
		if(err){
			pr_err("%s:Error Read ERROR_ID\n", __func__);
			goto end_of_interrupt;
		}
		pr_err("%s: Error Status. ID = 0x%02x\n",__func__, val);
		pixart_error_check_process(val, ts);
		if(val != 0x03)
			goto end_of_interrupt;
	}
	if (!(status & PIXART_REG_STATUS_GEN2_DATA_READY)){
		pr_err("%s: Error End\n",__func__);
		goto end_of_interrupt;
	}
	if (!(status & PIXART_REG_STATUS_GEN2_TOUCH))
		PIX_DEV_TOUCH("%s: decision No Touch\n",__func__);

	err = pixart_w_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2,
		PIXART_MOTION_MARK_READ_GEN2 | PIXART_MOTION_DISABLE_HOVER_GEN2);
	if(err){
		pr_err("%s: Error Write MOTION_REPORT_CTL 01\n",__func__);
		goto end_of_interrupt;
	}

	pdata = (uint8_t *)touch_data;
	err = pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, pdata, 2);
	if(err){
		pr_err("%s: Error READ MOTION_REPORT_DATA 01\n",__func__);
		goto end_of_interrupt;
	}	
	if (*pdata == 0xff) {
		pr_err("%s: ignoring report w/ status = 0xff.\n",__func__);
		err = pixart_w_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2,0x04);
		if(err){
			pr_err("%s: Error Write MOTION_REPORT_CTL 02\n",__func__);
			goto end_of_interrupt;
		}
		goto end_of_interrupt;
	}
	pdata += 2;
	err = pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, pdata, 9);
	if(err){
		pr_err("%s: Error READ MOTION_REPORT_DATA 02\n",__func__);
		goto end_of_interrupt;
	}
	pdata += 9;
	if (touch_data->total_touch > PIXART_MAX_FINGER)
		touch_data->total_touch = PIXART_MAX_FINGER;

	for (i = 1; i < touch_data->total_touch; i++) {
		err = pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, pdata, 9);
		if(err){
			pr_err("%s: Error READ MOTION_REPORT_DATA 03\n",__func__);
			goto end_of_interrupt;
		}
		pdata += 9;
	}

	err = pixart_w_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2,0x04);
	if(err){
		pr_err("%s: Error Write MOTION_REPORT_CTL 03\n",__func__);
		goto end_of_interrupt;
	}

	if(ts_event_control)
		goto end_of_interrupt;

	for(j = 0; j < PIXART_MAX_FINGER; j++){
		if(touch_data->slot[j].id == 0)
			continue;
		val = j;
		for(i = 0; i < touch_data->total_touch; i++){
			if(touch_data->slot[j].id == touch_data->point_data[i].id){
				val = 255;
				break;
			}
		}
		if(val != 255){
			PIX_DEV_TOUCH("%s RELEASE j=%d\n",__func__,j);
			input_mt_slot(ts->input, j);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 0);
			touch_data->slot[j].id = 0;
			touch_data->slot[j].tool_finger = 0;
		}
	}

	for (i = 0; i < touch_data->total_touch; i++) {
		if(touch_data->point_data[i].id < PIXART_TOUCH_ID_MIN ||
		   touch_data->point_data[i].id > PIXART_TOUCH_ID_MAX){
			pr_err("%s: Error id = %d\n",__func__, touch_data->point_data[i].id);
			continue;
		}
		for(j = 0; j < PIXART_MAX_FINGER; j++){
			if(touch_data->slot[j].id == touch_data->point_data[i].id){
				PIX_DEV_TOUCH("%s: MOVE j=%d , touch_id = %d\n",
							__func__,j,touch_data->point_data[i].id);
				move = 1;
				break;
			}
		}
		if(!move){
			for(j = 0; j < PIXART_MAX_FINGER; j++){
				if(touch_data->slot[j].id == 0){
					touch_data->slot[j].id = touch_data->point_data[i].id;
					PIX_DEV_TOUCH("%s: PUSH j=%d , touch_id = %d\n",
							__func__,j,touch_data->point_data[i].id);
					break;
				}
			}
		}
		move = 0;

		input_mt_slot(ts->input, j);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, touch_data->point_data[i].id != 0);
		touch_data->slot[j].tool_finger = (touch_data->point_data[i].id != 0);

		ts->touch_number++;
		if(touch_data->point_data[i].force != 1){
			input_report_abs(ts->input, ABS_MT_POSITION_X,	touch_data->point_data[i].x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,	touch_data->point_data[i].y);
			input_report_abs(ts->input, ABS_MT_PRESSURE,	touch_data->point_data[i].force);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,	touch_data->point_data[i].area);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,	AMRI5K_AREA_MAX);
			PIX_DEV_TOUCH("%s:[%02d] id:%02d, (x,y) = (%03d, %04d) area=%d, force=%d\n",__func__,j,
				touch_data->point_data[i].id,touch_data->point_data[i].x,touch_data->point_data[i].y,
				touch_data->point_data[i].area, touch_data->point_data[i].force);
		}
	}

	input_report_key(ts->input, BTN_TOUCH, ts->touch_number > 0);
	input_sync(ts->input);

	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;

end_of_interrupt:
	mutex_unlock(&ts->lock);
	PIX_DEV_TOUCH("%s: end of work func\n",__func__);
	return IRQ_HANDLED;
}

static int pixart_ts_open(struct inode *inode, struct file *file)
{
	struct pixart_data *ts = container_of(inode->i_cdev, struct pixart_data, device_cdev);

	file->private_data = ts;
	return 0;
};

static int pixart_ts_release(struct inode *inode, struct file *file)
{
	return 0;
};

static int pixart_atoi(struct device *dev, const char *src, u8 *dst)
{
	u8 val = 0;
	int cnt = 0;

	for (;; src++) {
		switch (*src) {
		case '0' ... '9':
			val = 16 * val + (*src - '0');
			break;
		case 'A' ... 'F':
			val = 16 * val + (*src - '7');
			break;
		case 'a' ... 'f':
			val = 16 * val + (*src - 'W');
			break;
		default:
			return 0;
		}
		if ((cnt % 2) == 1) {
			*dst = val;
			dst++;
			val = 0;
		}
		cnt++;
	}
	return -1;
}

static long pixart_get_property(struct device *dev, unsigned long arg)
{
	struct pixart_data *ts = dev_get_drvdata(dev);
	struct ts_nv_data *dp = (struct ts_nv_data *)arg;
	struct ts_config_nv *config_nv;
	enum ts_nv_type nv_type;
	long err = 0;
	size_t size;
	char *str;
	char ver[5];
	char *p;

	err = copy_from_user(&nv_type, (void __user *)&dp->nv_type, sizeof(dp->nv_type));
	if (err){
		err = -EFAULT;
		pr_err("%s: copy_from_user error :nv_type\n", __func__);
		goto done;
	}
	config_nv = &ts->config_nv[nv_type];

	err = copy_from_user(&size, (void __user *)&dp->size, sizeof(size_t));
	if (err){
		err = -EFAULT;
		pr_err("%s: copy_from_user error\n", __func__);
		goto done;
	}
	str = kcalloc(size, sizeof(char), GFP_KERNEL);
	if (!str) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		err = -ENOMEM;
		goto done;
	}
	err = copy_from_user(str, (void __user *)dp->data, size);
	if (err){
		err = -EFAULT;
		pr_err("%s: copy_from_user error\n", __func__);
		goto done;
	}

	mutex_lock(&ts->lock);
	config_nv->size = size / 2 - 2;

	if (!config_nv->data) {
		config_nv->data = kcalloc(config_nv->size, sizeof(char), GFP_KERNEL);
		if (!config_nv->data) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			err = -ENOMEM;
			goto err_free_str;
		}
	} else
		pr_err("%s: config_nv->data has been allocated.\n", __func__);

	memset(ver, '\0', sizeof(ver));
	memcpy(ver, str, 4);
	pixart_atoi(dev, ver, (u8 *)&config_nv->ver);

	p = str + 4;
	pixart_atoi(dev, p, config_nv->data);
	PIX_DEV_DBG("%s: type = %d, size = %d\n", __func__, nv_type, size);

err_free_str:
	mutex_unlock(&ts->lock);
	kfree(str);

done:
	return err;
}


static int pixart_set_initial_nv(struct pixart_data *ts, u8 *pf)
{
	struct ts_config_nv *config_nv;
	u8 val;
	int i, err = 0;

	PIX_DEV_DBG("%s() is called.\n",__func__);

	if (!ts->config_nv[TS_INITIAL_VALUE_NV].data) {
		pr_err("%s: No nv data. Skipping set nv.\n",__func__);
		return err;
	}
	config_nv = &ts->config_nv[TS_INITIAL_VALUE_NV];
	
	for(i = 0; i < config_nv->size; ){
		if(config_nv->data[i+1] == 0){
			PIX_DEV_DBG("%s: config is over\n",__func__);
			break;
		}
		err = pixart_r_reg_byte(ts->client, config_nv->data[i], &val);
		if(err){
			pr_err("%s:Error Read Add = 0x%x\n", __func__, config_nv->data[i]);
			return err;
		}
		if(val != config_nv->data[i+2]){
			printk("%s: val=%02x, Write 0x%02x -> 0x%02x\n",__func__,val,config_nv->data[i], config_nv->data[i+2]);
			err = pixart_w_reg_byte(ts->client, config_nv->data[i], config_nv->data[i+2]);
			if(err){
				pr_err("%s: Failed Write Register\n",__func__);
				break;
			}
			*pf = 1;
		}
		i += 3;
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_ORIENTATION_GEN2, &val);
	if(err){
		pr_err("%s: Error Read ORIENTATION\n",__func__);
		return err;
	}
	if((val & 0x07) != 0x03){
		pr_info("%s: ORIENTATION: Flash-Save\n", __func__);
		*pf = 1;
	}
	val = val & 0xF8;
	err = pixart_w_reg_byte(ts->client, PIXART_REG_ORIENTATION_GEN2, 0x03 | val);
	if(err){
		pr_err("%s: Error Write ORIENTATION\n",__func__);
		return err;
	}

	err = pixart_r_reg_byte(ts->client, PIXART_REG_REPORT_RATE_GEN2, &val);
	if(err){
		pr_err("%s: Error Read REPORT_RATE\n",__func__);
		return err;
	}
	if(val != 0x78){
		pr_info("%s: Report Rate: Flash-Save\n", __func__);
		*pf = 1;
	}
	err = pixart_w_reg_byte(ts->client, PIXART_REG_REPORT_RATE_GEN2, 0x78);
	if(err){
		pr_err("%s: Error Write Report_Rate\n",__func__);
		return err;
	}

	return err;
}

static int pixart_set_extended_nv(struct pixart_data *ts, u8 *pf)
{
	struct ts_config_nv *config_nv;
	uint8_t val = 0;
	int i, err = 0;

	PIX_DEV_DBG("%s() is called.\n",__func__);

	if (!ts->config_nv[TS_EXTENDED_NV].data) {
		pr_err("%s: No nv data. Skipping set nv.\n",__func__);
		return err;
	}
	config_nv = &ts->config_nv[TS_EXTENDED_NV];

	for(i = 0; i < config_nv->size; ){
		if(config_nv->data[i+1] == 0){
			PIX_DEV_DBG("%s: config is over\n",__func__);
			break;
		}
		err = pixart_r_na_reg(ts->client, config_nv->data[i+5], (uint8_t *)&val);
		if(val != config_nv->data[i+11]){
			printk("%s: val=%02x, Write 0x%02x -> 0x%02x\n",__func__,val,config_nv->data[i+5], config_nv->data[i+11]);
			err = pixart_w_na_reg(ts->client, config_nv->data[i+5], config_nv->data[i+11]);
			if(err){
				pr_err("%s: Failed Write Register\n",__func__);
				break;
			}
			*pf = 1;
		}
		i += 12;
	}

	return err;
}

static long pixart_set_nv(struct pixart_data *ts)
{
	long err = 0;
	u8 flash = 0;
	u8 *pflash = &flash;
	struct device *dev = &ts->client->dev;

	PIX_DEV_DBG("%s() is called.\n",__func__);

	err = wait_for_completion_interruptible_timeout(&ts->init_done,
			msecs_to_jiffies(5 * MSEC_PER_SEC));

	if (err < 0) {
		pr_err("%s: Error waiting device init (%d)!\n", __func__, (int)err);
		return -ENXIO;
	} else if (err == 0) {
		pr_err("%s: Timedout while waiting for device init!\n",__func__);
		return -ENXIO;
	}

	if (ts->is_enable) {
		disable_irq_nosync(ts->client->irq);
		ts->is_enable = false;
	}

	err = (long)pixart_set_initial_nv(ts, pflash);
	err |= (long)pixart_set_extended_nv(ts, pflash);
	ts->last_config_status = TS_INITIAL;
	err |= pixart_switch_config(dev);
	if(err){
		pr_err("%s: Failed set nv\n",__func__);
		return err;
	}

	if(*pflash){
		err = (long)pixart_flash_save(ts, PIXART_REG_FLASH_SAVE_CUST_REGS);
		if(err == 0){
			pr_info("%s: Success Backup\n", __func__);
			ts->is_set = true;
		} else{
			pr_err("%s: Fail Backup\n",__func__);
			ts->is_set = false;
		}
	}

	if (!ts->is_enable) {
		enable_irq(ts->client->irq);
		ts->is_enable = true;
	}

	return err;
}

static long pixart_switch_config(struct device *dev)
{
	struct pixart_data *ts = dev_get_drvdata(dev);
	struct ts_config_nv *config_nv;
	enum ts_nv_type nv_type;
	int i, err;

	nv_type = ts->config_status;

	config_nv = &ts->config_nv[nv_type];
	PIX_DEV_DBG("%s: config status is %d\n",__func__ ,ts->config_status);

	if(ts_log_level & 0x01){
		printk("%s: config is = ",__func__);
		for(i = 0; i < config_nv->size; i++){
			printk("%02x",config_nv->data[i]);
		}
		printk("\n");
	}

	if(ts->last_config_status == ts->config_status){
		PIX_DEV_DBG("%s: Skip. Same status.\n",__func__);
		return 0;
	}

	if(!config_nv->data){
		pr_err("%s: No nv data.\n",__func__);
		return 0;
	}
	for(i = 0; i < config_nv->size; ){
		if(config_nv->data[i+1] == 0){
			PIX_DEV_DBG("%s: config is over\n",__func__);
			break;
		}
		err = pixart_w_reg_byte(ts->client, config_nv->data[i], config_nv->data[i+2]);
		i += 3;
		if(err){
			pr_err("%s: Error Write register\n",__func__);
			return err;
		}
	}

	ts->last_config_status = ts->config_status;
	return 0;
}

static int pixart_read_summary_report(struct pixart_data *ts)
{
	const struct pixart_platform_data *pdata = ts->client->dev.platform_data;
	int count = 0;
	int retry = 0, err;
	u8 val;
	u8 *p_motion;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	if (!ts->motion_data)
		ts->motion_data = kcalloc(MOTION_DATA_SIZE * MOTION_DATA_PAGE_MAX,
												 sizeof(u8), GFP_KERNEL);
	else
		PIX_DEV_DBG("%s: motion_data has been allocated.\n",__func__);

	if (!ts->motion_data)  {
		pr_err("%s: Failed to allocate memory!\n",__func__);
		return -ENOMEM;
	}
	p_motion = ts->motion_data;

	while(count < MOTION_DATA_PAGE_MAX && retry < 20){
		val = gpio_get_value(pdata->irq_gpio);
		if(val != 1){
			retry++;
			msleep(30);
			continue;
		}

		err = pixart_r_reg_byte(ts->client, PIXART_REG_STATUS_STATUS_CHANGE, &val);
		if(err){
			pr_err("%s:Error Read CHANGE_STATUS\n", __func__);
			return err;
		}
		if(val == 0x00){
			pr_err("%s:Retry count = %d\n",__func__, retry);
			retry++;
			msleep(10);
			continue;
		}

		if(val & 0x01){
			pr_err("%s:Read error val = %02x\n",__func__, val);
			err = pixart_r_reg_byte(ts->client, PIXART_REG_ERROR_ID_GEN2, &val);
			if(err){
				pr_err("%s:Error Read ERROR_ID\n", __func__);
				return err;
			}
			pr_err("%s:Error Status. ID = 0x%02x\n",__func__, val);
			if(val != 0x03){
				pixart_error_check_process(val, ts);
			}
			retry++;
			continue;
		}

		if((val & 0x10) != 0x10){
			pr_err("%s:Error status Bit4 = 0x%02x\n",__func__,val);
			retry++;
			continue;
		}

		if((val & 0x02) != 0x02){
			pr_err("%s:Error status Bit1 = 0x%02x\n",__func__,val);
			retry++;
			continue;
		}

		err = pixart_r_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2, &val);
		if(err){
			pr_err("%s:Error Read MOTION_REPORT\n", __func__);
			return err;
		}
		val |= 0x80;
		err = pixart_w_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2, val);
		if(err){
			pr_err("%s: Error Write MOTION_REPORT_CTL 01\n",__func__);
			return err;
		}

		if(pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9)){
			pr_err("%s:Error Read MOTION_REPORT_DATA\n", __func__);
			return -1;
		}
		p_motion += 9;
		if(pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9)){
			pr_err("%s:Error Read MOTION_REPORT_DATA\n", __func__);
			return -1;
		}
		p_motion += 9;
		if(pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9)){
			pr_err("%s:Error Read MOTION_REPORT_DATA\n", __func__);
			return -1;
		}
		p_motion += 9;
		if(pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9)){
			pr_err("%s:Error Read MOTION_REPORT_DATA\n", __func__);
			return -1;
		}
		p_motion += 9;
		if(pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9)){
			pr_err("%s:Error Read MOTION_REPORT_DATA\n", __func__);
			return -1;
		}
		p_motion += 9;
		if(pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9)){
			pr_err("%s:Error Read MOTION_REPORT_DATA\n", __func__);
			return -1;
		}
		p_motion += 45;
		if(pixart_r_reg_nbytes(ts->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 2)){
			pr_err("%s:Error Read MOTION_REPORT_DATA\n", __func__);
			return -1;
		}
		p_motion += 2;

		val &= 0x7F;
		err = pixart_w_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2, val);
		if(err){
			pr_err("%s: Error Write MOTION_REPORT_CTL 02\n",__func__);
			return err;
		}

		retry = 0;
		count++;
		PIX_DEV_DBG("%s: count %d\n",__func__ ,count);
	}
	if(retry == 20){
		pr_err("%s:Error Retry = %d\n",__func__, retry);
		return -1;
	}
	return 0;

}

static int pixart_open_short(struct pixart_data *ts, u8 *p_motin_fail)
{
	u8 val;
	int err;
	int retry = 0;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	err = pixart_w_reg_byte(ts->client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);
	if(err){
		pr_err("%s: Error Write SHUTDOWN\n",__func__);
		return err;
	}
	msleep(1);
	err = pixart_w_reg_byte(ts->client, PIXART_REG_SHUTDOWN, PIXART_RESET);
	if(err){
		pr_err("%s: Error Write RESET\n",__func__);
		return err;
	}
	msleep(10);

	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, &val);
		if(err){
			pr_err("%s:Error Read BOOT_Stat\n", __func__);
			return err;
		}
		if(val & 0x01)
			break;
		retry++;
		msleep(10);
		if(retry > 50){
			pr_err("%s:Error01 Retry count = %d\n",__func__, retry);
			return -1;
		}
	}
	retry = 0;

	err = pixart_w_reg_byte(ts->client, 0x31, 0xFC);
	if(err){
		pr_err("%s: Error Write TEST_CTL\n",__func__);
		return err;
	}
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_STATUS, &val);
		if(err){
			pr_err("%s:Error Read STATUS\n", __func__);
			return err;
		}
		if(val == 0xFC)
			break;
		retry++;
		msleep(10);
		if(retry > 10){
			pr_err("%s:Error02 Retry count = %d\n",__func__, retry);
			return -1;
		}
	}
	retry = 0;

	err = pixart_w_reg_byte(ts->client, PIXART_REG_REPORT_POINTS_GEN2, 0x06);
	if(err){
		pr_err("%s: Error Write REPORT_POINT\n",__func__);
		return err;
	}
	err = pixart_w_reg_byte(ts->client, PIXART_REG_OPEN_SHORT_SELECT, 0x10);
	if(err){
		pr_err("%s: Error Write OPEN_SHORT_SELECT\n",__func__);
		return err;
	}
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_OPEN_SHORT_SELECT, &val);
		if(err){
			pr_err("%s:Error Read OPEN_SHORT_SELECT 01\n", __func__);
			return err;
		}
		if(val == 0x00)
			break;
		retry++;
		msleep(10);
		if(retry > 10){
			pr_err("%s:Error03 Retry count = %d\n",__func__, retry);
			return -1;
		}
	}
	retry = 0;

	err = pixart_w_reg_byte(ts->client, PIXART_REG_REPORT_RATE_GEN2, 0x5A);
	if(err){
		pr_err("%s: Error Write REPORT_RATE\n",__func__);
		return err;
	}
	err = pixart_w_reg_byte(ts->client, PIXART_REG_OPEN_SHORT_SELECT, 0x65);
	if(err){
		pr_err("%s: Error Write OPEN_SHORT_SELECT 02\n",__func__);
		return err;
	}
	msleep(1000);
	while(1){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_OPEN_SHORT_RESULT, &val);
		if(err){
			pr_err("%s:Error Read OPEN_SHORT_RESULT\n", __func__);
			return err;
		}
		if(val & 0x01)
			break;
		retry++;
		msleep(100);
		if(retry > 40){
			pr_err("%s:Error04 Retry count = %d\n",__func__, retry);
			return -1;
		}
	}
	retry = 0;

	err = pixart_read_summary_report(ts);
	if(err)
		goto done;

	err = pixart_r_reg_byte(ts->client, PIXART_REG_OPEN_SHORT_RESULT, &val);
	if(err){
		pr_err("%s:Error Read OPEN_SHORT_RESULT\n", __func__);
		return err;
	}
	if((val & 0x02) != 0x02){
		PIX_DEV_DBG("%s: Test PASS\n",__func__);
		goto done;
	}

	err = pixart_r_reg_nbytes(ts->client, 0x92, p_motin_fail, 11);
	if(err){
		pr_err("%s: Error READ Motion_fail 03\n",__func__);
		return err;;
	}
	if(pixart_w_reg_byte(ts->client, 0x31, 0x00))
		pr_err("%s: Error Write TEST_CTL\n",__func__);
	if(pixart_reset_process(ts))
		pr_err("%s: Error Reset Process\n",__func__);
	return -1;

done:
	err = pixart_w_reg_byte(ts->client, 0x31, 0x00);
	if(err)
		pr_err("%s: Error Write TEST_CTL\n",__func__);
	err = pixart_reset_process(ts);
	if(err)
		pr_err("%s: Error Reset Process\n",__func__);
	return err;

}

static long pixart_pixel_dump(struct pixart_data *ts, u8 *p_pixel_dump)
{
	int i = 0;
	long err = 0;

	PIX_DEV_DBG("%s() is called.\n",__func__);

	if (ts->Touch_Status >= TOUCH_POWEROFF){
		pr_err("%s: Failed. IC is Suspend.\n",__func__);
		return -1;
	}

	err = pixart_w_reg_byte(ts->client, PIXART_REG_PIXEL_CONFIG, 0x00);
	err |= pixart_w_reg_byte(ts->client, PIXART_REG_PIXEL_CONFIG, 0x01);
	if(err){
		pr_err("%s: Failed Write Register\n",__func__);
		return err;
	}

	msleep(500);

	for(i = 0; i < PIXEL_DUMP/2; i++){
		err = pixart_r_reg_byte(ts->client, PIXART_REG_PIXEL_DATA_HI, p_pixel_dump);
		if(err){
			pr_err("%s:Error Read PIXEL_DATA_HI\n", __func__);
			return err;
		}
		p_pixel_dump++;
		err = pixart_r_reg_byte(ts->client, PIXART_REG_PIXEL_DATA_LO, p_pixel_dump);
		if(err){
			pr_err("%s:Error Read PIXEL_DATA_LO\n", __func__);
			return err;
		}
		p_pixel_dump++;
	}

	err = pixart_w_reg_byte(ts->client, PIXART_REG_PIXEL_CONFIG, 0x00);
	if(err)
		pr_err("%s:Error Write PIXEL_CONFIG\n",__func__);
	return err;

}

static long pixart_ts_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pixart_data *ts = (struct pixart_data *)file->private_data;
	const struct pixart_platform_data *pdata = ts->client->dev.platform_data;
	struct device *dev = &ts->client->dev;
	long err = 0;
	char count;
	u8 *p_motion;
	u8 motion_fail_data[11];

	switch (cmd) {
	case IOCTL_SET_CONF_STAT:
		if (ts->client->irq == -1) {
			if(ts->probe_status & 0x10){
				pr_err("%s: driver is abnormal status.\n",__func__);
				ts->probe_status += 0x10;
			}
			return -1;
		}
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&ts->lock);
		if (copy_from_user(&ts->config_status, (void __user *)arg, sizeof(ts->config_status))) {
			err = -EFAULT;
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			mutex_unlock(&ts->lock);
			goto done;
		}

		err = pixart_switch_config(dev);
		mutex_unlock(&ts->lock);
		break;
	case IOCTL_SET_LOG:
		return 0;
	case IOCTL_DIAG_START:
		PIX_DEV_DBG("%s: DIAG_START\n",__func__);
		if (ts->client->irq == -1) {
			pr_err("%s: driver is abnormal status.\n",__func__);
			return -1;
		}
		err = pixart_diag_data_start(ts);
		break;
	case IOCTL_MULTI_GET:
	case IOCTL_COODINATE_GET:
		PIX_DEV_DBG("%s: IOCTL_MULTI_GET\n", __func__);
		PIX_DEV_DBG("%s: IOCTL_COODINATE_GET\n", __func__);
		if (ts->client->irq == -1) {
			dev_err(dev, "driver is abnormal status.\n");
			return -1;
		}
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			pr_err("%s: invalid access\n", __func__);
			goto done;
		}
 		if (diag_data != NULL) {
			pixart_diag_store(ts);
			err = copy_to_user((void __user *)arg, diag_data,
						sizeof(struct ts_diag_type));
		} else
			pr_info("%s: Touchscreen Diag not active!\n",__func__);

		if (err) {
			pr_err("%s: copy_to_user error\n", __func__);
			return -EFAULT;
		}
		break;
	case IOCTL_DIAG_END:
		PIX_DEV_DBG("%s: DIAG_END\n",__func__);
		if (ts->client->irq == -1) {
			pr_err("%s: driver is abnormal status.\n",__func__);
			return -1;
		}
		err = pixart_diag_data_end(ts);
		break;
	case IOCTL_DIAG_EVENT_CTRL:
		if (ts->client->irq == -1) {
			pr_err("%s: driver is abnormal status.\n",__func__);
			return -1;
		}
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		err = copy_from_user(&ts_event_control, (void __user *)arg,
						sizeof(unsigned char));
		if (err){
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			return -EFAULT;
		}
		return 0;
	case IOCTL_LOAD_NV:
		PIX_DEV_DBG("%s: IOCTL_LOAD_NV\n",__func__);
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		err = pixart_get_property(dev, arg);
		break;
	case IOCTL_SET_NV:
		PIX_DEV_DBG("%s: IOCTL_SET_NV\n", __func__);
		if (ts->client->irq == -1) {
			pr_err("%s: driver is abnormal status.\n",__func__);
			return -1;
		}
		mutex_lock(&ts->lock);
		err = pixart_set_nv(ts);
		mutex_unlock(&ts->lock);
		break;
	case IOCTL_DIAG_RESET_HW:
		PIX_DEV_DBG("%s: IOCTL_DIAG_RESET_HW\n",__func__);
		if (ts->client->irq == -1) {
			pr_err("%s: driver is abnormal status.\n",__func__);
			return -1;
		}
		mutex_lock(&ts->lock);
		if(pdata->reset_hw)
			pdata->reset_hw();
		mutex_unlock(&ts->lock);
		return 0;
	case IOCTL_DIAG_LOG_LEVEL:
		PIX_DEV_DBG("%s: Change Log Level\n",__func__);
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		if (copy_from_user(&ts_log_level, (void __user *)arg, sizeof(unsigned char))) {
			err = -EFAULT;
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			goto done;
		}
		break;
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT_KC
	case IOCTL_GET_GOLDEN_REFERENCE:
		return 0;
#endif
	case IOCTL_DIAG_GET_C_REFERENCE:
		PIX_DEV_DBG("%s: Open Short Test\n",__func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			pr_err("%s: invalid access\n", __func__);
			goto done;
		}
		err = copy_from_user(&count, (void __user *)arg, sizeof(unsigned char));
		if (err){
			pr_err("%s: copy_from_user error\n", __func__);
			return -EFAULT;
		}
		switch(count){
		case 0:
			mutex_lock(&ts->lock);
			memset(motion_fail_data, 0, sizeof(motion_fail_data));

			err = pixart_open_short(ts, motion_fail_data);
			if(err){
				pr_err("%s: Failed Open Short Test\n", __func__);
				if(motion_fail_data[0]){
					pr_err("%s: Touch Panel NG\n", __func__);
					err = copy_to_user((void __user *)arg, motion_fail_data, 12);
					if (err) {
						pr_err("%s: copy_to_user error\n", __func__);
					}
				}
			}
			mutex_unlock(&ts->lock);
			break;
		case 1:
		case 2:
		case 3:
		case 4:
			if(!ts->motion_data){
				pr_err("%s: motion data is NULL\n",__func__);
				return -ENOMEM;
			}
			p_motion = ts->motion_data + (count - 1) * MOTION_DATA_SIZE * MOTION_DATA_1BLOCK;
			err = copy_to_user((void __user *)arg, p_motion,
								 MOTION_DATA_SIZE * MOTION_DATA_1BLOCK);
			if (err) {
				pr_err("%s: copy_to_user error\n", __func__);
				goto done;
			}
			break;
		case 5:
			if(ts->motion_data){
				PIX_DEV_DBG("%s: motion data release\n",__func__);
				kfree(ts->motion_data);
				ts->motion_data = NULL;
			}
			break;
		default:
			pr_err("%s: wrong value\n",__func__);
			break;
		}

		PIX_DEV_DBG("%s: Open Short Test is completed\n",__func__);
		break;
	case IOCTL_DIAG_GET_DELTA:
		PIX_DEV_DBG("%s: Pixel Dump\n",__func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			pr_err("%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&ts->lock);
		p_motion = kcalloc(PIXEL_DUMP, sizeof(u8), GFP_KERNEL);
		if (!p_motion) {
			pr_err("%s: Failed to allocate memory!\n",__func__);
			mutex_unlock(&ts->lock);
			return -ENOMEM;
		}

		err = pixart_pixel_dump(ts, p_motion);
		if(err){
			pr_err("%s: Failed Pixel Dump\n", __func__);
			kfree(p_motion);
			p_motion = NULL;
			mutex_unlock(&ts->lock);
			goto done;
		}
		mutex_unlock(&ts->lock);
		err = copy_to_user((void __user *)arg, p_motion, PIXEL_DUMP);

		kfree(p_motion);
		p_motion = NULL;

		if (err) {
			pr_err("%s: copy_to_user error\n", __func__);
			goto done;
		}

		PIX_DEV_DBG("%s: Pixel Dump completed\n",__func__);
		break;
	default:
		return -EINVAL;
		break;
	}
done:
	return err;
}

const struct file_operations pixart_ts_fops = {
	.owner = THIS_MODULE,
	.open = pixart_ts_open,
	.unlocked_ioctl = pixart_ts_ioctl,
	.release = pixart_ts_release,
};

static int pixart_input_open(struct input_dev *dev)
{
	struct pixart_data *ts = input_get_drvdata(dev);
	int error;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	error = wait_for_completion_interruptible_timeout(&ts->init_done,
			msecs_to_jiffies(5 * MSEC_PER_SEC));

	if (error > 0) {
		if (ts->client->irq != -1){
			error = pixart_enable_irq(ts);
		}
		else {
			pr_err("%s: Can't enable irq.\n",__func__);
#ifdef	TMP_FACTORY_TEST
			error = 0;
#else
			error = -ENXIO;
#endif
		}
	} else if (error < 0) {
		pr_err("%s: Error while waiting for device init (%d)!\n",__func__, error);
		error = -ENXIO;
	} else if (error == 0) {
		pr_err("%s: Timedout while waiting for device init!\n",__func__);
		error = -ENXIO;
	}
#ifdef REGULATOR_ON
	reg_touch = regulator_get(NULL, "8921_l17");
	if(IS_ERR(reg_touch)){
		pr_err("%s regulator is not get\n",__func__);
	}
#endif

	return error;
}

static void pixart_input_close(struct input_dev *dev)
{
	struct pixart_data *ts = input_get_drvdata(dev);

	PIX_DEV_DBG("%s() is called.\n",__func__);
	if (ts->client->irq != -1){
		pixart_disable_irq(ts);
	}
}


static int pixart_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pixart_data *ts = i2c_get_clientdata(client);
#ifdef REGULATOR_ON
	int rc = 0;
#endif

	PIX_DEV_DBG("%s() is called.\n",__func__);

	if(pixart_wq){
		cancel_delayed_work_sync(&ts->esdwork);
		flush_workqueue(pixart_wq);
	}

	mutex_lock(&ts->lock);

	disable_irq_nosync(client->irq);
	ts->Touch_Status = TOUCH_POWEROFF;

	if(pixart_w_reg_byte(client, 0x7a, 0xaa)){
		pr_err("%s:Error Write Suspend\n",__func__);
		mutex_unlock(&ts->lock);
		return -1;
	}

	mutex_unlock(&ts->lock);
#ifdef REGULATOR_ON
	if(reg_touch)
		rc = regulator_set_optimum_mode(reg_touch, 100);

	if(rc < 0)
		pr_err("%s regulator set error\n",__func__);
	else
		PIX_DEV_DBG("%s regulator set 0x%02x\n",__func__,rc);
#endif

	return 0;
}

static void pixart_report_clear(struct pixart_data *ts)
{
	int i;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	for(i = 0; i < 10; i++){
		if(ts->touch_data.slot[i].id == 0)
			continue;

		PIX_DEV_DBG("%s: [%d] released\n",__func__, i);
		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 0);
		ts->touch_data.slot[i].id = 0;
	}

	input_report_key(ts->input, BTN_TOUCH, 0);
	input_sync(ts->input);
}

static int pixart_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pixart_data *ts = i2c_get_clientdata(client);
	int i, err = 0;
	u8 val;
#ifdef REGULATOR_ON
	int rc = 0;
#endif

	PIX_DEV_DBG("%s() is called.\n",__func__);

#ifdef REGULATOR_ON
	if(reg_touch)
		rc = regulator_set_optimum_mode(reg_touch, 30000);

	if(rc < 0)
		pr_err("%s regulator set error\n",__func__);
	else
		PIX_DEV_DBG("%s regulator set 0x%02x\n",__func__,rc);
#endif

	mutex_lock(&ts->lock);
	pixart_report_clear(ts);

	err = pixart_w_reg_byte(client, 0x7a, 0xdd);
	if(err){
		pr_err("%s:Error Write Resume\n",__func__);
		goto done;
	}

	for (i = 0; i < 75; i++) {
		err = pixart_r_reg_byte(client, 0x03, &val);
		if(err){
			pr_err("%s:Error Read resume\n", __func__);
			goto done;
		}
		if ((val & 0x91) == 0x91){
			PIX_DEV_DBG("%s: Success. Read retry = %d\n",__func__, i);
			break;
		}
		msleep(10);
	}
	if(i >= 75){
		pr_err("%s: Error Status\n",__func__);
		pixart_reset_process(ts);
	}

done:
	enable_irq(client->irq);
	ts->Touch_Status = TOUCH_POWERON;

	if(pixart_wq){
		queue_delayed_work(pixart_wq, &ts->esdwork,
				   msecs_to_jiffies(5000));
	}

	mutex_unlock(&ts->lock);
	return err;
}

static void pixart_early_suspend(struct early_suspend *h)
{
	struct pixart_data *ts = container_of(h, struct pixart_data, early_suspend);
	PIX_DEV_DBG("%s() is called.\n",__func__);
	pixart_suspend(&ts->client->dev);
	PIX_DEV_DBG("%s() is completed.\n",__func__);
}

static void pixart_late_resume(struct early_suspend *h)
{
	struct pixart_data *ts = container_of(h, struct pixart_data, early_suspend);
	PIX_DEV_DBG("%s() is called.\n",__func__);
	pixart_resume(&ts->client->dev);
	PIX_DEV_DBG("%s() is completed.\n",__func__);
}

static void pixart_esd_work(struct work_struct *work)
{
	struct pixart_data *ts = container_of(work, struct pixart_data, esdwork.work);
	int err;
	u8 val;

	if(ts_esd_recovery != 0){
		if(mutex_trylock(&ts->lock)) {
			err = pixart_r_reg_byte(ts->client, PIXART_REG_PID, &val);
			if(err){
				pr_err("%s: Error Read PRODUCT_ID\n", __func__);
			}
			if(pid != val){
				pr_err("%s: error\n",__func__);
				pixart_reset_process(ts);
			}else
				PIX_DEV_DBG("%s: Correct PROD_ID [0x%02x]\n",__func__, val);
			mutex_unlock(&ts->lock);
		}
	}
	if(pixart_wq){
		queue_delayed_work(pixart_wq, &ts->esdwork,
					msecs_to_jiffies(5000));
	}
}

static int __devinit pixart_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct pixart_platform_data *pdata = client->dev.platform_data;
	struct pixart_data *ts;
	struct input_dev *input_dev;
	int i, retry = 0, err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n",__func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof (struct pixart_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		pr_err("%s: Failed to allocate memory\n",__func__);
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->input = input_dev;
	ts->touch_number = 0;
	ts->irq = client->irq;

	for(i = 0; i < 10; i++){
		ts->touch_data.slot[i].id = 0;
		ts->touch_data.slot[i].tool_finger = 0;
		ts->touch_data.point_data[i].id = 0;
		ts->touch_data.point_data[i].x = 0;
		ts->touch_data.point_data[i].y = 0;
		ts->touch_data.point_data[i].force = 0;
		ts->touch_data.point_data[i].area = 0;
	}

	input_dev->name = PIXART_I2C_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = pixart_input_open;
	input_dev->close = pixart_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, PIXART_MAX_FINGER);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,  0, pdata->y_size, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,  0, pdata->x_size, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, AMRI5K_AREA_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, AMRI5K_AREA_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,    0, AMRI5K_FORCE_MAX, 0, 0);

	ts->config_status = TS_INITIAL;
	ts->last_config_status = TS_INITIAL;
	ts->Touch_Status = TOUCH_POWEROFF;
	ts->err_cnt = 0;
	ts->probe_status = 0;

	mutex_init(&ts->lock);
	init_completion(&ts->init_done);

	input_set_drvdata(input_dev, ts);
	i2c_set_clientdata(client, ts);

	if (pdata->init_hw)
		err = pdata->init_hw();

	if(err){
		ts->probe_status = 0x01;
		pr_err("%s: Failed to initialize hardware\n",__func__);
#ifdef	TMP_FACTORY_TEST
		goto err_dev_access;
#else
		goto err_free_mem;
#endif
	}
	ts->probe_status = 0x02;

	while(retry < 2){
		err = pixart_reset_and_wait(ts);
		if (err) {
			ts->probe_status = 0x03;
			pr_err("%s: Reset and wait failed\n",__func__);
#ifdef	TMP_FACTORY_TEST
			goto err_dev_access;
#else
			goto err_free_mem;
#endif
		}
		ts->probe_status = 0x04;

		if (pid < PIXART_5000_PID) {
			pr_err("%s: Failed. PID reported was 0x%x\n",__func__, pid);
			goto err_free_mem;
		}

		err = pixart_init_panel(ts);
		if (err == 0)
			break;
		pr_err("%s: init_panel failed.\n",__func__);
		retry++;
		ts->probe_status = 0x05;
	}
	if(retry >= 2)
		goto err_free_mem;

	ts->config_status = TS_CHARGE_CABLE;
	ts->Touch_Status = TOUCH_POWERON;
	ts->probe_status = 0x06;

	if (request_threaded_irq(client->irq, NULL, pixart_interrupt,
				pdata->irqflags, client->dev.driver->name, ts)) {
		pr_err("%s: Failed to register interrupt\n",__func__);
		goto err_free_mem;
	}

	disable_irq(client->irq);
	complete_all(&ts->init_done);
	ts->probe_status = 0x07;

	if(pixart_wq) {
		INIT_DELAYED_WORK(&ts->esdwork, pixart_esd_work);
		queue_delayed_work(pixart_wq, &ts->esdwork,
				   msecs_to_jiffies(10000));
	}

	err = input_register_device(input_dev);
	if (err) {
		pr_err("%s: input_register_device\n",__func__);
		ts->probe_status = 0x08;
		goto err_free_irq;
	}

	ts->probe_status = 0x09;
	err = pixart_sysfs_init(ts);
	if(err)
		goto err_unregister_device;

	ts_ctrl_init(&(ts->device_cdev), &pixart_ts_fops);
	ts->probe_status = 0x0A;

	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = pixart_early_suspend;
	ts->early_suspend.resume = pixart_late_resume;
	register_early_suspend(&ts->early_suspend);

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, ts);
err_free_mem:
	input_free_device(input_dev);
	mutex_destroy(&ts->lock);
	kfree(ts);
err_check_functionality_failed:
	return err;

#ifdef TMP_FACTORY_TEST
err_dev_access:
	client->irq = -1;
	complete_all(&ts->init_done);
	err = input_register_device(input_dev);
	if (err)
		goto err_free_mem;
	err = pixart_sysfs_init(ts);
	if (err)
		goto err_free_mem;
	ts_ctrl_init(&(ts->device_cdev), &pixart_ts_fops);
	return err;
#endif
}

static int pixart_ts_remove(struct i2c_client *client)
{
	struct pixart_data *ts = i2c_get_clientdata(client);

	PIX_DEV_DBG("%s() is called.\n",__func__);
	sysfs_remove_group(&client->dev.kobj, &tp_attr_group);
	free_irq(ts->client->irq, ts);
	input_unregister_device(ts->input);
	unregister_early_suspend(&ts->early_suspend);
	ts_ctrl_exit(&(ts->device_cdev));

	kfree(ts);
	pr_info("unregistered touchscreen\n");
	return 0;
}

static void pixart_ts_shutdown(struct i2c_client *client)
{
	const struct pixart_platform_data *pdata = client->dev.platform_data;
	struct pixart_data *ts = i2c_get_clientdata(client);

	PIX_DEV_DBG("%s() is called.\n",__func__);
	if(pixart_wq){
		cancel_delayed_work_sync(&ts->esdwork);
		flush_workqueue(pixart_wq);
	}
	mutex_lock(&ts->lock);
	if (pdata->shutdown)
		pdata->shutdown();
	mutex_unlock(&ts->lock);
}

static const struct i2c_device_id pixart_ts_id[] = {
	{ PIXART_I2C_NAME, 0},
	{}
};

static struct i2c_driver pixart_ts_driver = {
	.probe		= pixart_ts_probe,
	.remove		= pixart_ts_remove,
	.shutdown	= pixart_ts_shutdown,
	.id_table	= pixart_ts_id,
	.driver = {
		.name	= PIXART_I2C_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init pixart_init(void)
{
	pr_info(MODULE_NAME "%s\n", __func__);

	pixart_wq = alloc_workqueue("pixart_wq", WQ_MEM_RECLAIM, 1);
	if (!pixart_wq){
		pr_info(MODULE_NAME "%s workqueue Error\n", __func__);
		return -ENOMEM;
	}

	return i2c_add_driver(&pixart_ts_driver);
}

static void __exit pixart_exit(void)
{
	PIX_DEV_DBG("%s() is called.\n",__func__);
	if (pixart_wq)
		destroy_workqueue(pixart_wq);

	i2c_del_driver(&pixart_ts_driver);
}
module_init(pixart_init);
module_exit(pixart_exit);

MODULE_AUTHOR("bert_lin@pixart.com.tw");
MODULE_DESCRIPTION("Pixart Touchscreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("pixart");

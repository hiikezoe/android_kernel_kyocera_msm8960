/* Copyright (c) 2010-2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/vibrator.h>

#include "timed_output.h"

//#define DEBUG_VIB_PM8XXX
#define VIB_TEST

#define VIB_DRV			0x4A

#define VIB_DRV_SEL_MASK	0xf8
#define VIB_DRV_SEL_SHIFT	0x03
#define VIB_DRV_EN_MANUAL_MASK	0xfc
#define VIB_DRV_LOGIC_SHIFT	0x2

#define VIB_MAX_LEVEL_mV	3100
#define VIB_MIN_LEVEL_mV	1200

#define VIB_WORK_NUM        (5)

/* log */
#define VIB_LOG(md, fmt, ...) \
printk(md "[VIB]%s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#ifdef DEBUG_VIB_PM8XXX
#define VIB_DEBUG_LOG(md, fmt, ...) \
printk(md "[VIB]%s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#else
#define VIB_DEBUG_LOG(md, fmt, ...)
#endif /* DEBUG_VIB_PM8XXX */

struct vib_on_work_struct
{
    struct work_struct work_vib_on;
    int vib_time;
};

struct pm8xxx_vib {
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	spinlock_t lock;
    struct vib_on_work_struct vib_on_work_data[VIB_WORK_NUM];
    struct work_struct work_vib_off[VIB_WORK_NUM];
    int work_vib_on_pos;
    int work_vib_off_pos;
	struct device *dev;
	const struct pm8xxx_vibrator_platform_data *pdata;
	int level;
	u8  reg_vib_drv;
    int add_time_flag;
    struct mutex vib_mutex;
};

static struct pm8xxx_vib *vib_dev;

#ifdef VIB_TEST
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>

#define VIB_TEST_IOC_MAGIC 'v'
#define IOCTL_VIB_TEST_CTRL _IOWR(VIB_TEST_IOC_MAGIC, 1, vib_test_param)

#define VIB_TEST_SET_VOLTAGE 0x0001

#define VIB_TEST_STATUS_SUCCESS (0)
#define VIB_TEST_STATUS_FAIL    (-1)

typedef struct {
    u16 req_code; 
	u8 data[4];
} vib_test_param;

typedef struct {
    u16 voltage;
    u8 reserved[2];
} vib_test_set_voltage_req_data;

typedef struct {
    u16 status;
    u8 reserved[2];
} vib_test_rsp_data;
#endif /* VIB_TEST */

int pm8xxx_vibrator_config(struct pm8xxx_vib_config *vib_config)
{
	u8 reg = 0;
	int rc;

    mutex_lock(&vib_dev->vib_mutex);
    VIB_DEBUG_LOG(KERN_INFO, "called.drive_mV=%d,active_low=%d,enable_mode=%d\n"
        ,vib_config->drive_mV, vib_config->active_low, vib_config->enable_mode);

	if (vib_dev == NULL) {
        VIB_LOG(KERN_ERR, "vib_dev is NULL\n");
        mutex_unlock(&vib_dev->vib_mutex);
		return -EINVAL;
	}

	if (vib_config->drive_mV) {
		if ((vib_config->drive_mV < VIB_MIN_LEVEL_mV) ||
			(vib_config->drive_mV > VIB_MAX_LEVEL_mV)) {
            VIB_LOG(KERN_ERR, "Invalid vibrator drive strength\n");
            mutex_unlock(&vib_dev->vib_mutex);
			return -EINVAL;
		}
	}

	reg = (vib_config->drive_mV / 100) << VIB_DRV_SEL_SHIFT;

	reg |= (!!vib_config->active_low) << VIB_DRV_LOGIC_SHIFT;

	reg |= vib_config->enable_mode;

	rc = pm8xxx_writeb(vib_dev->dev->parent, VIB_DRV, reg);
    VIB_DEBUG_LOG(KERN_INFO, 
    "pm8xxx_writeb() called. rc=%d,reg=0x%02X,data=0x%02X\n", rc, VIB_DRV, reg);
	if (rc)
        VIB_LOG(KERN_ERR, "pm8xxx write failed: rc=%d\n",rc);

    mutex_unlock(&vib_dev->vib_mutex);
    VIB_DEBUG_LOG(KERN_INFO, "end.rc=%d\n", rc);
	return rc;
}
EXPORT_SYMBOL(pm8xxx_vibrator_config);

#ifdef DEBUG_VIB_PM8XXX
/* REVISIT: just for debugging, will be removed in final working version */
static void __dump_vib_regs(struct pm8xxx_vib *vib, char *msg)
{
	u8 temp;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    VIB_DEBUG_LOG(KERN_INFO, "%s\n", msg);

	pm8xxx_readb(vib->dev->parent, VIB_DRV, &temp);
    VIB_DEBUG_LOG(KERN_INFO, "VIB_DRV - %X\n", temp);
}
#endif /* DEBUG_VIB_PM8XXX */

static int pm8xxx_vib_read_u8(struct pm8xxx_vib *vib, u8 *data, u16 reg)
{
	int rc;

    VIB_DEBUG_LOG(KERN_INFO, "called. reg=0x%02X\n", reg);
	rc = pm8xxx_readb(vib->dev->parent, reg, data);
    VIB_DEBUG_LOG(KERN_INFO,
          "pm8xxx_readb() called. rc=%d,reg=0x%02X,data=0x%02X\n",
                                                  rc, reg, *data);
	if (rc < 0)
        VIB_LOG(KERN_ERR, "Error reading pm8xxx: %X - ret %X\n",
				reg, rc);

    VIB_DEBUG_LOG(KERN_INFO, "end.rc=%d\n", rc);
	return rc;
}

static int pm8xxx_vib_write_u8(struct pm8xxx_vib *vib, u8 data, u16 reg)
{
	int rc;

    VIB_DEBUG_LOG(KERN_INFO, "called. data=0x%02X,reg=0x%02X\n", data, reg);
	rc = pm8xxx_writeb(vib->dev->parent, reg, data);
    VIB_DEBUG_LOG(KERN_INFO,
        "pm8xxx_writeb() called. rc=%d,reg=0x%02X,data=0x%02X\n", rc, reg, data);
	if (rc < 0)
        VIB_LOG(KERN_ERR, "Error writing pm8xxx: %X - ret %X\n",
				reg, rc);
    VIB_DEBUG_LOG(KERN_INFO, "end.rc=%d\n", rc);
	return rc;
}

static int pm8xxx_vib_set(struct pm8xxx_vib *vib, int on, int time)
{
	int rc;
	u8 val;

    mutex_lock(&vib->vib_mutex);
    VIB_DEBUG_LOG(KERN_INFO, "called. on=%d,time=%d\n", on, time);
	if (on) {
        VIB_DEBUG_LOG(KERN_INFO, "VIB ON.reg_vib_drv=0x%02X,level=0x%02X\n",
                                              vib->reg_vib_drv, vib->level);
		val = vib->reg_vib_drv;
		val |= ((vib->level << VIB_DRV_SEL_SHIFT) & VIB_DRV_SEL_MASK);
		rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
        VIB_DEBUG_LOG(KERN_INFO,
                      "pm8xxx_vib_write_u8 called.rc=%d,val=0x%02X\n", rc, val);
        if (rc < 0) {
            VIB_DEBUG_LOG(KERN_INFO, "pm8xxx_vib_write_u8() error. rc=%d\n",
                                                                        rc);
            mutex_unlock(&vib->vib_mutex);
			return rc;
        }
		vib->reg_vib_drv = val;
        hrtimer_start(&vib->vib_timer,
                  ktime_set(time / 1000, (time % 1000) * 1000000),
                  HRTIMER_MODE_REL);
	} else {
        VIB_DEBUG_LOG(KERN_INFO, "VIB OFF.reg_vib_drv=0x%02X,level=0x%02X\n",
                                               vib->reg_vib_drv, vib->level);
		val = vib->reg_vib_drv;
		val &= ~VIB_DRV_SEL_MASK;
		rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
        VIB_DEBUG_LOG(KERN_INFO,
                      "pm8xxx_vib_write_u8 called.rc=%d,val=0x%02X\n", rc, val);
        if (rc < 0) {
            VIB_DEBUG_LOG(KERN_INFO, "pm8xxx_vib_write_u8() error. rc=%d\n"
                                                                     , rc);
            mutex_unlock(&vib->vib_mutex);
			return rc;
        }
		vib->reg_vib_drv = val;
	}
#ifdef DEBUG_VIB_PM8XXX
	__dump_vib_regs(vib, "vib_set_end");
#endif /* DEBUG_VIB_PM8XXX */

    mutex_unlock(&vib->vib_mutex);
    VIB_DEBUG_LOG(KERN_INFO, "end.rc=%d,reg_vib_drv=0x%02X\n", rc,
                                                vib->reg_vib_drv);
	return rc;
}

static void pm8xxx_vibrator_on(struct work_struct *work)
{
    struct vib_on_work_struct *work_data = container_of
                                 (work, struct vib_on_work_struct, work_vib_on);
    struct pm8xxx_vib *vib = vib_dev;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    pm8xxx_vib_set(vib, 1, work_data->vib_time);
}

static void pm8xxx_vibrator_off(struct work_struct *work)
{
    struct pm8xxx_vib *vib = vib_dev;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    pm8xxx_vib_set(vib, 0, 0);
	}

static void pm8xxx_timed_vibrator_on(struct pm8xxx_vib *vib, int time)
{
    int ret = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called.time=%d\n", time);
    vib->vib_on_work_data[vib->work_vib_on_pos].vib_time = time;
    VIB_DEBUG_LOG(KERN_INFO, "vib_on_work_data[%d].time=%d\n",
                                         vib->work_vib_on_pos,
        vib->vib_on_work_data[vib->work_vib_on_pos].vib_time);

    ret = schedule_work
                  (&(vib->vib_on_work_data[vib->work_vib_on_pos].work_vib_on));
    VIB_DEBUG_LOG(KERN_INFO, "schedule_work(). ret=%d", ret);
    if (ret != 0) {
        vib->work_vib_on_pos++;
        if (vib->work_vib_on_pos >= VIB_WORK_NUM)
            vib->work_vib_on_pos = 0;
        VIB_DEBUG_LOG(KERN_INFO, "work_vib_on_pos=%d\n",
                                  vib->work_vib_on_pos);
    }
	else {
        VIB_LOG(KERN_ERR, "schedule_work() error.ret=%d\n", ret);
	}

    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
}

static void pm8xxx_timed_vibrator_off(struct pm8xxx_vib *vib)
{
    int ret = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    ret = schedule_work(&vib->work_vib_off[vib->work_vib_off_pos]);
    VIB_DEBUG_LOG(KERN_INFO, "schedule_work(). ret=%d", ret);
    if (ret != 0) {
        vib->work_vib_off_pos++;
        if (vib->work_vib_off_pos >= VIB_WORK_NUM)
            vib->work_vib_off_pos = 0;
        VIB_DEBUG_LOG(KERN_INFO, "work_vib_off_pos=%d\n",
                                  vib->work_vib_off_pos);
    }
    else {
        VIB_LOG(KERN_ERR, "schedule_work() error.ret=%d\n", ret);
}

    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
}

static void pm8xxx_vib_enable(struct timed_output_dev *dev, int value)
{
	struct pm8xxx_vib *vib = container_of(dev, struct pm8xxx_vib,
							 timed_dev);

    VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%X,value=%d,add_time_flag=%d\n",
                                       (u32)dev, value, vib->add_time_flag);
    if ((value <= 0) && (vib->add_time_flag == 1)) {
        VIB_DEBUG_LOG(KERN_INFO, "vib off skip.");
        return;
    }

    hrtimer_cancel(&vib->vib_timer);

    if (value <= 0) {
        pm8xxx_timed_vibrator_off(vib);
    } else {
        if(value < 50) {
            value = 55;
            vib->add_time_flag = 1;
        } else if(value < 55) {
            value += 5;
            vib->add_time_flag = 1;
        } else if(value < 60) {
            value += 3;
            vib->add_time_flag = 1;
        } else {
            vib->add_time_flag = 0;
        }

        VIB_DEBUG_LOG(KERN_INFO, "fixed value=%d, add_time_flag=%d.", value,
                                                        vib->add_time_flag);
        pm8xxx_timed_vibrator_on(vib, value);
    }
    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
}

static int pm8xxx_vib_get_time(struct timed_output_dev *dev)
{
    struct pm8xxx_vib *vib = container_of(dev, struct pm8xxx_vib, timed_dev);

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
        VIB_DEBUG_LOG(KERN_INFO, "remaining=%d.\n", (int)ktime_to_us(r));
        return (int)ktime_to_ms(r);
    } else {
        VIB_DEBUG_LOG(KERN_INFO, "hrtimer not active.\n");
		return 0;
}
}

static enum hrtimer_restart pm8xxx_vib_timer_func(struct hrtimer *timer)
{
	struct pm8xxx_vib *vib = container_of(timer, struct pm8xxx_vib,
							 vib_timer);

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    vib->add_time_flag = 0;
    VIB_DEBUG_LOG(KERN_INFO, "add_time_flag=%d\n", vib->add_time_flag);
    pm8xxx_timed_vibrator_off(vib);

    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
	return HRTIMER_NORESTART;
}

#ifdef VIB_TEST
static int vibrator_test_open(struct inode *ip, struct file *fp)
{
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
    return 0;
}

static int vibrator_test_release(struct inode *ip, struct file *fp)
{
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
    return 0;
}

static int vibrator_test_set_voltage(u8 *data)
{
    int ret = 0;
    vib_test_set_voltage_req_data *req_data =
       (vib_test_set_voltage_req_data *)data;
    vib_test_rsp_data *rsp_data = (vib_test_rsp_data *)data;
    struct pm8xxx_vib_config config;
    s16 status = VIB_TEST_STATUS_SUCCESS;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");

    config.drive_mV = req_data->voltage;
    config.active_low = 0;
    config.enable_mode = PM8XXX_VIB_MANUAL;
    ret = pm8xxx_vibrator_config(&config);
    if (ret < 0) {
        VIB_LOG(KERN_ERR, "pm8xxx_vibrator_config error.ret=%d\n", ret);
        status = VIB_TEST_STATUS_FAIL;
    }

    memset(rsp_data, 0x00, sizeof(vib_test_rsp_data));
    rsp_data->status = (u32)status;

    VIB_DEBUG_LOG(KERN_INFO, "end. ret=%d\n", ret);
    return ret;
}

static long vibrator_test_ioctl
                        (struct file *file, unsigned int cmd, unsigned long arg)
{
    int rc = 0;
    int ret = 0;
    u64 ret2 = 0;
    vib_test_param test_param;
    VIB_DEBUG_LOG(KERN_INFO, "called. cmd=0x%08X\n", cmd);
    
    switch (cmd) {
    case IOCTL_VIB_TEST_CTRL:
        VIB_DEBUG_LOG(KERN_INFO, "cmd=IOCTL_VIB_TEST_CTRL\n");
        ret2 = copy_from_user(&test_param, (void *)arg, sizeof(test_param));
        VIB_DEBUG_LOG(KERN_INFO, "copy_from_user() called. ret2=%lu\n",
                                              (long unsigned int)ret2);
        VIB_DEBUG_LOG(KERN_INFO, 
               "copy_from_user() req_code=0x%04X,data=0x%02X%02X%02X%02X\n",
                test_param.req_code, test_param.data[0], test_param.data[1],
                                    test_param.data[2], test_param.data[3]);
        if (ret2) {
            VIB_LOG(KERN_ERR, "copy_from_user() error. ret2=%lu\n",
                                          (long unsigned int)ret2);
            rc = -EINVAL;
            break;
        }
        VIB_DEBUG_LOG(KERN_INFO, "req_code=0x%04X\n", test_param.req_code);
        switch (test_param.req_code) {
        case VIB_TEST_SET_VOLTAGE:
            ret = vibrator_test_set_voltage(&test_param.data[0]);
            if (ret < 0) VIB_LOG(KERN_ERR,
                             "vibrator_test_set_voltage() error. ret=%d\n", ret);
            ret2 = copy_to_user((void *)arg, &test_param, sizeof(vib_test_param));
            VIB_DEBUG_LOG(KERN_INFO, "copy_to_user() called. ret2=%lu\n",
                                                (long unsigned int)ret2);
            VIB_DEBUG_LOG(KERN_INFO, 
                     "copy_to_user() req_code=0x%04X,data=0x%02X%02X%02X%02X\n",
                    test_param.req_code, test_param.data[0], test_param.data[1],
                                        test_param.data[2], test_param.data[3]);
            if (ret2) {
                VIB_LOG(KERN_ERR, "copy_to_user() error. ret2=%lu\n",
                                            (long unsigned int)ret2);
                rc = -EINVAL;
            }
            break;
        default:
            VIB_LOG(KERN_ERR, "req_code error. req_code=0x%04X\n",
                                             test_param.req_code);
            rc = -EINVAL;
            break;
        }
        break;
    default:
        VIB_LOG(KERN_ERR, "cmd error. cmd=0x%08X\n", cmd);
        rc = -EINVAL;
        break;
    }

    VIB_DEBUG_LOG(KERN_INFO, "end. rc=%d\n", rc);
    return rc;
}

static const struct file_operations vibrator_test_fops = {
    .owner          = THIS_MODULE,
    .open           = vibrator_test_open,
    .release        = vibrator_test_release,
    .unlocked_ioctl = vibrator_test_ioctl,
};

static struct miscdevice vibrator_test_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "kc_vibrator_test",
    .fops = &vibrator_test_fops,
};

void vibrator_test_init(void)
{
    misc_register(&vibrator_test_dev);
}
#endif /* VIB_TEST */

#ifdef CONFIG_PM
static int pm8xxx_vib_suspend(struct device *dev)
{
	struct pm8xxx_vib *vib = dev_get_drvdata(dev);
    int count = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	hrtimer_cancel(&vib->vib_timer);
    for (count = 0; count < VIB_WORK_NUM; count++) {
        VIB_DEBUG_LOG(KERN_INFO, "count=%d.\n", count);
        cancel_work_sync(&(vib->vib_on_work_data[count].work_vib_on));
        cancel_work_sync(&vib->work_vib_off[count]);
        vib->vib_on_work_data[count].vib_time = 0;
    }
	/* turn-off vibrator */
    pm8xxx_vib_set(vib, 0, 0);

    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
	return 0;
}

static const struct dev_pm_ops pm8xxx_vib_pm_ops = {
	.suspend = pm8xxx_vib_suspend,
};
#endif

static int __devinit pm8xxx_vib_probe(struct platform_device *pdev)

{
	const struct pm8xxx_vibrator_platform_data *pdata =
						pdev->dev.platform_data;
	struct pm8xxx_vib *vib;
	u8 val;
	int rc;
    int count = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    if (!pdata) {
        VIB_LOG(KERN_ERR, "pdata is NULL\n");
		return -EINVAL;
    }

	if (pdata->level_mV < VIB_MIN_LEVEL_mV ||
             pdata->level_mV > VIB_MAX_LEVEL_mV) {
        VIB_LOG(KERN_ERR, "level_mV error. level_mV=%d\n",
                                         pdata->level_mV);
		return -EINVAL;
    }

	vib = kzalloc(sizeof(*vib), GFP_KERNEL);
    if (!vib) {
        VIB_LOG(KERN_ERR, "kzalloc error.\n");
		return -ENOMEM;
    }

	vib->pdata	= pdata;
	vib->level	= pdata->level_mV / 100;
	vib->dev	= &pdev->dev;

    mutex_init(&vib->vib_mutex);
    for (count = 0; count < VIB_WORK_NUM; count++) {
        INIT_WORK(&(vib->vib_on_work_data[count].work_vib_on),
                                          pm8xxx_vibrator_on);
        INIT_WORK(&vib->work_vib_off[count], pm8xxx_vibrator_off);
        vib->vib_on_work_data[count].vib_time = 0;
        VIB_DEBUG_LOG(KERN_INFO, "vib_on_work_data[%d].vib_time=%d\n", count,
                                      vib->vib_on_work_data[count].vib_time);
    }

    vib->work_vib_on_pos = 0;
    vib->work_vib_off_pos = 0;

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = pm8xxx_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = pm8xxx_vib_get_time;
	vib->timed_dev.enable = pm8xxx_vib_enable;

#ifdef DEBUG_VIB_PM8XXX
	__dump_vib_regs(vib, "boot_vib_default");
#endif /* DEBUG_VIB_PM8XXX */

	/*
	 * Configure the vibrator, it operates in manual mode
	 * for timed_output framework.
	 */
	rc = pm8xxx_vib_read_u8(vib, &val, VIB_DRV);
    if (rc < 0) {
        VIB_LOG(KERN_ERR, "pm8xxx_vib_read_u8 error. rc=%d\n", rc);
		goto err_read_vib;
    }
	val &= ~VIB_DRV_EN_MANUAL_MASK;
	rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
    if (rc < 0) {
        VIB_LOG(KERN_ERR, "pm8xxx_vib_write_u8 error. rc=%d\n", rc);
		goto err_read_vib;
    }

	vib->reg_vib_drv = val;

	rc = timed_output_dev_register(&vib->timed_dev);
    if (rc < 0) {
        VIB_LOG(KERN_ERR, "timed_output_dev_register error. rc=%d\n", rc);
		goto err_read_vib;
    }

	platform_set_drvdata(pdev, vib);

	vib_dev = vib;

    VIB_DEBUG_LOG(KERN_INFO, "end.rc=%d\n", 0);
	return 0;

err_read_vib:
    VIB_DEBUG_LOG(KERN_INFO, "err_read_vib.\n");
	kfree(vib);
    VIB_DEBUG_LOG(KERN_INFO, "end.rc=%d\n", rc);
	return rc;
}

static int __devexit pm8xxx_vib_remove(struct platform_device *pdev)
{
	struct pm8xxx_vib *vib = platform_get_drvdata(pdev);
    int count = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    for (count = 0; count < VIB_WORK_NUM; count++) {
        VIB_DEBUG_LOG(KERN_INFO, "count=%d.\n", count);
        cancel_work_sync(&(vib->vib_on_work_data[count].work_vib_on));
        cancel_work_sync(&vib->work_vib_off[count]);
        vib->vib_on_work_data[count].vib_time = 0;
    }
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);
	platform_set_drvdata(pdev, NULL);
	kfree(vib);

    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
	return 0;
}

static struct platform_driver pm8xxx_vib_driver = {
	.probe		= pm8xxx_vib_probe,
	.remove		= __devexit_p(pm8xxx_vib_remove),
	.driver		= {
		.name	= PM8XXX_VIBRATOR_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm8xxx_vib_pm_ops,
#endif
	},
};

static int __init pm8xxx_vib_init(void)
{
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
#ifdef VIB_TEST
    vibrator_test_init();
#endif /* VIB_TEST */
	return platform_driver_register(&pm8xxx_vib_driver);
}
module_init(pm8xxx_vib_init);

static void __exit pm8xxx_vib_exit(void)
{
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	platform_driver_unregister(&pm8xxx_vib_driver);
}
module_exit(pm8xxx_vib_exit);

MODULE_ALIAS("platform:" PM8XXX_VIBRATOR_DEV_NAME);
MODULE_DESCRIPTION("pm8xxx vibrator driver");
MODULE_LICENSE("GPL v2");

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
drivers/felica/felica.c  (FeliCa driver)

This software is contributed or developed by KYOCERA Corporation.
(C) 2011 KYOCERA Corporation
(C) 2012 KYOCERA Corporation
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/* drivers/sharp/mfc/felica.c  (FeliCa driver)
 *
 * Copyright (C) 2010 SHARP CORPORATION
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


/***************header***************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <asm/current.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/unistd.h>

#include <felica/felica_cen_diag.h>
#include <felica/felica_cen.h>
#include <felica/felica_pon.h>
#include <felica/felica_poll.h>

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/major.h>

#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/device.h>

#include <linux/slab.h>
#include <linux/i2c-dev.h>
#include <linux/platform_device.h>
#include <asm/mach/mmc.h>
#include <linux/irq.h>
#include <linux/mfd/pmic8058.h>
#include <linux/serial_core.h>

#include <mach/irqs-8960.h>


static struct class *felica_class = NULL;
static struct cdev rfs_cdev;
static struct cdev pon_cdev;
static struct cdev cen_cdev;
static struct cdev int_poll_cdev;

static char g_readCmd = 0xFF;

static unsigned int	g_unrate = 0;
static unsigned int	g_unbaud = 0;
static struct cdev rate_cdev;
static struct cdev rws_cdev;
static char g_rws_data = -1;
/*
 * Definition
*/

/* FC PON IO	*/
#define MFD_PON_GPIO_NUM 	9
#define I2C_BUS_NUMBER				1
#define I2C_MFD_SLAVE_ADDR	(0x80 >> 1)
#define MFD_INT_GPIO_NUM	106
#define MFD_RFS_GPIO_NUM	107
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)

/* rate */
#define RATE_MAGIC	0xFC
#define RATE_REQ_RATE_115					_IO(RATE_MAGIC, 1)			/* 115.2					*/
#define RATE_REQ_RATE_230					_IO(RATE_MAGIC, 2)			/* 230.4					*/
#define RATE_REQ_RATE_460					_IO(RATE_MAGIC, 3)			/* 460.8					*/

#define RATE_DEVS		(1)
#define RATE_DEV_NAME	("felica_rate")

/* RWS */
#define RWS_DEVS		(1)
#define RWS_DEV_NAME	("felica_rws")

/* RFS */
#define D_RFS_DEVS		(1)
#define D_RFS_GPIO_NO	(21)
#define D_RFS_DEV_NAME	("felica_rfs")

/* INT */
#define D_INT_DEVS		(1)
#define D_INT_GPIO_NO	MFD_INT_GPIO_NUM
#define D_INT_DEV_NAME	("felica_int")

/* PON */
#define D_PON_DEVS		(1)
#define D_PON_GPIO_NO	(22)
#define D_PON_DEV_NAME	("felica_pon")
#define D_PON_DEV_LOW		(0)
#define D_PON_DEV_HIGH		(1)
#define D_PON_HIGH_ENABLE	(1)
#define D_PON_HIGH_DISABLE	(0)

/* CEN */
#define D_CEN_DEVS		(1)
#define D_CEN_DEV_NAME	("felica_cen")

/* RFS_POLL */
#define D_RFS_POLL_DEVS		D_RFS_DEVS
#define D_RFS_POLL_GPIO_NO	D_RFS_GPIO_NO
#define D_RFS_POLL_DEV_NAME	("rfs_poll")

#define D_RFS_POLL_SLEEP_NUM	(3)
#define D_RFS_POLL_SLEEP_MSEC	(1)
#define D_RFS_POLL_DELAY_MSEC	(3)
#define D_RFS_DEV_LOW			(0)
#define D_RFS_DEV_HIGH			(1)

/* INT_POLL */
#define D_INT_POLL_DEVS		D_INT_DEVS
#define D_INT_POLL_GPIO_NO	MFD_INT_GPIO_NUM
#define D_INT_POLL_DEV_NAME	("int_poll")

#define D_INT_POLL_SLEEP_NUM	(3)
#define D_INT_POLL_SLEEP_MSEC	(1)
#define D_INT_POLL_DELAY_MSEC	(3)
#define D_INT_DEV_LOW			(0)
#define D_INT_DEV_HIGH			(1)

/* CEN */
static struct i2c_client *this_client;

struct cen_data{
	struct input_dev *input_dev;
};


/* DEBUG_LOG */
#if 1
#define DEBUG_MFC_DRV
#endif

#ifdef DEBUG_MFC_DRV
#define MFC_DRV_DBG_LOG(fmt, args...) printk(KERN_INFO "[FeliCa][%s]" fmt"\n", __func__, ## args)
#else
#define MFC_DRV_DBG_LOG(fmt, args...)
#endif

/* ERROR_LOG */
#define MFC_DRV_ERR_LOG(fmt, args...) printk(KERN_ERR "[FeliCa][%s]ERR " fmt"\n", __func__, ## args)


/*
 * prototype
*/
static __init int felica_init(void);
static void __exit felica_exit(void);

static int cen_probe(struct i2c_client *client, const struct i2c_device_id * devid);

/* rws device	*/
ssize_t rws_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	char rws_val;
	
	MFC_DRV_DBG_LOG("START");

	if (buf == NULL) {
		MFC_DRV_ERR_LOG("rws_read param err");
		return -EIO;
	}

	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}

	rws_val = g_rws_data;

	if (copy_to_user(buf, &rws_val, 1)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}

	MFC_DRV_DBG_LOG("END rws_val = %d, len = %d", rws_val, len);

	return 1;
}

ssize_t rws_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char rws_val;

	MFC_DRV_DBG_LOG("START");

	if (data == NULL) {
		MFC_DRV_ERR_LOG("rws_write param err");
		return -EIO;
	}

	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	if (copy_from_user(&rws_val, data, 1)) {
		MFC_DRV_ERR_LOG("copy_from_user");
		return -EFAULT;
	}

	g_rws_data = rws_val;

	MFC_DRV_DBG_LOG("END rws_val = %d, g_rws_data = %d", rws_val, g_rws_data);

	return 1;
}
static int rws_open(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("START");
	MFC_DRV_DBG_LOG("END");
	return 0;
}

static int rws_release(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("START");
	MFC_DRV_DBG_LOG("END");
	return 0;
}
static const struct file_operations rws_fileops = {
	.owner   = THIS_MODULE,
	.read    = rws_read,
	.write   = rws_write,
	.open    = rws_open,
	.release = rws_release,
};
static int rws_init(void)
{
	int           sdResult      = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , RWS_DEVS, RWS_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rws_cdev, &rws_fileops);
	rws_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rws_cdev, dev, RWS_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, RWS_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}

	class_dev = device_create(felica_class, NULL, dev, NULL, RWS_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rws_cdev);
		unregister_chrdev_region(dev, RWS_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	g_rws_data = 0;
	MFC_DRV_DBG_LOG("END");

	return sdResult;
}

static void rws_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");

	device_destroy(felica_class, dev);

	cdev_del(&rws_cdev);
	unregister_chrdev_region(dev, RWS_DEVS);

	MFC_DRV_DBG_LOG("END");
}

ssize_t rate_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	int rate_val;
	
	MFC_DRV_DBG_LOG("START");
	
	if ( len < 4 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}

	rate_val = g_unrate;
	if (copy_to_user(buf, &rate_val, 4)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}

	MFC_DRV_DBG_LOG("END rate_val = %d, len = %d", rate_val, len);

	return 4;
}
static long rate_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int				nRet	= 0;

	MFC_DRV_DBG_LOG("START");
	MFC_DRV_DBG_LOG("cmd = %x\n",cmd);
	switch( cmd ){
		case RATE_REQ_RATE_115:
			g_unrate = 115200;
			g_unbaud = B115200;
			break;
		case RATE_REQ_RATE_230:
			g_unrate = 230400;
			g_unbaud = B230400;
			break;
		case RATE_REQ_RATE_460:
			g_unrate = 460800;
			g_unbaud = B460800;
			break;
		default:
			nRet = -1;
			break;
	}
	MFC_DRV_DBG_LOG("END");
	return nRet;
}

static int rate_open(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("START");
	MFC_DRV_DBG_LOG("END");
	return 0;
}

static int rate_release(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("START");
	MFC_DRV_DBG_LOG("END");
	return 0;
}
static const struct file_operations rate_fileops = {
	.owner   = THIS_MODULE,
	.read    = rate_read,
	.unlocked_ioctl   = rate_ioctl,
	.open    = rate_open,
	.release = rate_release,
};
static int rate_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , RATE_DEVS, RATE_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rate_cdev, &rate_fileops);
	rate_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rate_cdev, dev, RATE_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, RATE_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(felica_class, NULL, dev, NULL, RATE_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rate_cdev);
		unregister_chrdev_region(dev, RATE_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
}

static void rate_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");

	device_destroy(felica_class, dev);

	cdev_del(&rate_cdev);
	unregister_chrdev_region(dev, RATE_DEVS);

	MFC_DRV_DBG_LOG("END");
}

/*
 * function_rfs
 */
ssize_t rfs_read(struct file *file, char __user * buf,
		      size_t len, loff_t * ppos)
{
	int ret;
	char on[2];
	
	MFC_DRV_DBG_LOG("START");
	
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	ret = gpio_get_value_cansleep( MFD_RFS_GPIO_NUM );

	if (ret < 0) {
		MFC_DRV_ERR_LOG("gpio_get_value ret = %d", ret);
		return ret;
	}
	
	if (ret == D_RFS_DEV_HIGH)
		on[0] = SHMFD_RFS_STATUS_HIGH;
	else
		on[0] = SHMFD_RFS_STATUS_LOW;
	
	on[1] = 0x00;
	
	if (len > 2)
		len = 2;
	
	if (copy_to_user(buf, on, len)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}
	
	MFC_DRV_DBG_LOG("END on[0] = %d, len = %d", on[0], len);
	
	return len;
}

static int rfs_open(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

static int rfs_release(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

static const struct file_operations rfs_fileops = {
	.owner   = THIS_MODULE,
	.read    = rfs_read,
	.open    = rfs_open,
	.release = rfs_release,
};

static int rfs_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , D_RFS_DEVS, D_RFS_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rfs_cdev, &rfs_fileops);
	rfs_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rfs_cdev, dev, D_RFS_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_RFS_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(felica_class, NULL, dev, NULL, D_RFS_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rfs_cdev);
		unregister_chrdev_region(dev, D_RFS_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
}

static void rfs_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	cdev_del(&rfs_cdev);
	unregister_chrdev_region(dev, D_RFS_DEVS);
	
	MFC_DRV_DBG_LOG("END");
}


/*
 * function_pon
 */
ssize_t pon_write(struct file *file, const char __user *data,
		       size_t len, loff_t *ppos)
{
	char on;
	int seton;
	
	MFC_DRV_DBG_LOG("START");
	
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	if (copy_from_user(&on, data, 1)) {
		MFC_DRV_ERR_LOG("copy_from_user");
		return -EFAULT;
	}

	if (on == D_PON_DEV_HIGH){
		seton = D_PON_DEV_HIGH;
		MFC_DRV_DBG_LOG("pon high.\n");
	}else if (on == D_PON_DEV_LOW){
		MFC_DRV_DBG_LOG("pon low.\n");
		seton = D_PON_DEV_LOW;
	}else {
		MFC_DRV_DBG_LOG("pon err value = %x \n",on );
		return -EFAULT;
	}

	gpio_set_value(MFD_PON_GPIO_NUM , seton );

	MFC_DRV_DBG_LOG("END on = %d, seton = %d", on, seton);
	
	return len;
}

static long pon_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	MFC_DRV_DBG_LOG("START");

	MFC_DRV_DBG_LOG("END");
	return 0;
}

static int pon_open(struct inode *inode, struct file *file)
{
	int				nRet = -1;

	MFC_DRV_DBG_LOG("START");
	MFC_DRV_DBG_LOG("pon rate = %d\n",g_unrate);
	MFC_DRV_DBG_LOG("pon baud = %o\n",g_unbaud);
	nRet = 0;
	MFC_DRV_DBG_LOG("END");
	return nRet;
}

static int pon_release(struct inode *inode, struct file *file)
{
	MFC_DRV_DBG_LOG("START");

	gpio_set_value(MFD_PON_GPIO_NUM , D_PON_DEV_LOW);

	MFC_DRV_DBG_LOG("END");

	return 0;
}

static const struct file_operations pon_fileops = {
	.owner   = THIS_MODULE,
	.write   = pon_write,
	.unlocked_ioctl   = pon_ioctl,
	.open    = pon_open,
	.release = pon_release,
};

static int pon_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , D_PON_DEVS, D_PON_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&pon_cdev, &pon_fileops);
	pon_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&pon_cdev, dev, D_PON_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_PON_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(felica_class, NULL, dev, NULL, D_PON_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&pon_cdev);
		unregister_chrdev_region(dev, D_PON_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
}

static void pon_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");

	device_destroy(felica_class, dev);

	cdev_del(&pon_cdev);
	unregister_chrdev_region(dev, D_PON_DEVS);

	MFC_DRV_DBG_LOG("END");
}

struct poll_data {
	wait_queue_head_t read_wait;
	int irq_handler_done;
	struct delayed_work work;
	int device_status;
	int read_error;
	int open_flag;
};

static struct poll_data g_int_data;
static struct poll_data *g_int_d = &g_int_data;

/*
 * function_int_poll
 */
static irqreturn_t int_poll_irq_handler(int irq, void *dev_id);
void int_poll_work_func(struct work_struct *work)
{
	struct poll_data *int_d = g_int_d;
	int read_value = 0, old_value = 0, i = 0;
	unsigned long irqflag = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	old_value = int_d->device_status;
	for (i = 0; i < D_INT_POLL_SLEEP_NUM; i++) {
		read_value = gpio_get_value(D_INT_POLL_GPIO_NO);

		if (read_value < 0 || read_value == old_value)
			break;
		
		msleep(D_INT_POLL_SLEEP_MSEC);
	}
	
	MFC_DRV_DBG_LOG("read_value = %d old_value = %d", read_value, old_value);
	
	if (read_value < 0) {
		int_d->read_error = read_value;
	} else if (read_value != old_value) {
		int_d->device_status = read_value;
		int_d->read_error = 0;
		
		if (int_d->device_status == D_INT_DEV_LOW)
			irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		else
			irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;

		if (irq_set_irq_type(gpio_to_irq(D_INT_POLL_GPIO_NO), irqflag))
			MFC_DRV_ERR_LOG("set_irq_type irqflag = %ld", irqflag);
	}
	
	enable_irq(gpio_to_irq(D_INT_POLL_GPIO_NO));
	
	if (read_value != old_value || int_d->read_error) {
		int_d->irq_handler_done = 1;
		wake_up_interruptible(&int_d->read_wait);
	}
	
	MFC_DRV_DBG_LOG("END read_value = %d, old_value = %d, int_d->read_error = %d"
					, read_value, old_value, int_d->read_error);
}

static irqreturn_t int_poll_irq_handler(int irq, void *dev_id)
{
	struct poll_data *int_d = g_int_d;
	
	MFC_DRV_DBG_LOG("START irq = %d", irq);
	
	disable_irq_nosync(gpio_to_irq(D_INT_POLL_GPIO_NO));

	schedule_delayed_work(&int_d->work, msecs_to_jiffies(D_INT_POLL_DELAY_MSEC));
	
	MFC_DRV_DBG_LOG("END");
	
	return IRQ_HANDLED;
}

unsigned int int_poll_poll(struct file *file, poll_table *wait)
{
	struct poll_data *int_d = g_int_d;
	unsigned int mask = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	poll_wait(file, &int_d->read_wait, wait);
	if (int_d->irq_handler_done)
		mask = POLLIN | POLLRDNORM;
	
	MFC_DRV_DBG_LOG("END mask = %d", mask);
	
	return (mask);
}

ssize_t int_poll_read(struct file *file, char __user * buf,
		      size_t len, loff_t * ppos)
{
	struct poll_data *int_d = g_int_d;
	int ret;
	char on[2];
	
	MFC_DRV_DBG_LOG("START");
	
	if ( len < 1 ) {
		MFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}
	
	if (!int_d->irq_handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			MFC_DRV_ERR_LOG("NONBLOCK");
			return -EAGAIN;
		}
		MFC_DRV_DBG_LOG("FeliCa int_poll wait irq");
		ret = wait_event_interruptible(int_d->read_wait,
		                               int_d->irq_handler_done == 1);
		if (-ERESTARTSYS == ret) {
			MFC_DRV_DBG_LOG("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}
	
	if (int_d->read_error) {
		int_d->irq_handler_done = 0;
		int_d->read_error = 0;
		MFC_DRV_ERR_LOG("int_d->read_error = %d", int_d->read_error);
		return -EIO;
	}
	
	if (int_d->device_status == D_INT_DEV_HIGH)
		on[0] = SHMFD_INT_STATUS_HIGH;
	else
		on[0] = SHMFD_INT_STATUS_LOW;
	
	on[1] = 0x00;
	
	if (len > 2)
		len = 2;
	if (copy_to_user(buf, on, len)) {
		MFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}
	int_d->irq_handler_done = 0;
	
	MFC_DRV_DBG_LOG("END len = %d, on[0] = %d", len, on[0]);
	
	return len;
}

static int int_poll_open(struct inode *inode, struct file *file)
{
	struct poll_data *int_d = g_int_d;
	unsigned long irqflag = 0;
	int ret = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	if (int_d->open_flag) {
		MFC_DRV_ERR_LOG("only one time");
		return -EBUSY;
	}
	int_d->open_flag = 1;
	
	ret = gpio_get_value(D_INT_POLL_GPIO_NO);
	if (ret < 0) {
		int_d->open_flag = 0;
		MFC_DRV_ERR_LOG("gpio_get_value ret = %d", ret);
		return -EIO;
	}
	int_d->device_status = ret;
	
	if (int_d->device_status == D_INT_DEV_LOW)
		irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
	else
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
	
	if (request_irq(gpio_to_irq(D_INT_POLL_GPIO_NO),
	                int_poll_irq_handler,
	                irqflag,
	                D_INT_POLL_DEV_NAME,
	                (void*)int_d)) {
		
		int_d->open_flag = 0;
		
		MFC_DRV_ERR_LOG("request_irq irqflag = %ld", irqflag);
		
		return -EIO;
	}
	
	if (enable_irq_wake(gpio_to_irq(D_INT_POLL_GPIO_NO))){
		
		MFC_DRV_ERR_LOG("enable_irq_wake");
		
		free_irq(gpio_to_irq(D_INT_POLL_GPIO_NO), (void *)int_d);
		
		return -EIO;
	}
	
	int_d->irq_handler_done = 0;
	
	MFC_DRV_DBG_LOG("END");
	
	return 0;
}

static int int_poll_release(struct inode *inode, struct file *file)
{
	struct poll_data *int_d = g_int_d;
	
	MFC_DRV_DBG_LOG("START");
	
	cancel_delayed_work(&int_d->work);
	
	if (disable_irq_wake(gpio_to_irq(D_INT_POLL_GPIO_NO)))
		MFC_DRV_ERR_LOG("disable_irq_wake");
	
	free_irq(gpio_to_irq(D_INT_POLL_GPIO_NO), (void *)int_d);
	
	int_d->open_flag = 0;
	
	MFC_DRV_DBG_LOG("END");
	
	return 0;
}

static const struct file_operations int_poll_fileops = {
	.owner   = THIS_MODULE,
	.read    = int_poll_read,
	.open    = int_poll_open,
	.release = int_poll_release,
	.poll    = int_poll_poll,
};

static int int_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , D_INT_POLL_DEVS, D_INT_POLL_DEV_NAME);
	if (sdResult) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&int_poll_cdev, &int_poll_fileops);
	int_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&int_poll_cdev, dev, D_INT_POLL_DEVS);
	if (sdResult) {
		unregister_chrdev_region(dev, D_INT_POLL_DEVS);
		MFC_DRV_ERR_LOG("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(felica_class, NULL, dev, NULL, D_INT_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&int_poll_cdev);
		unregister_chrdev_region(dev, D_INT_POLL_DEVS);
		sdResult = PTR_ERR(class_dev);
		MFC_DRV_ERR_LOG("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	memset(g_int_d, 0x00, sizeof(struct poll_data));

	INIT_DELAYED_WORK(&g_int_d->work, int_poll_work_func);

	init_waitqueue_head(&g_int_d->read_wait);
	
	g_int_d->open_flag = 0;
	
	MFC_DRV_DBG_LOG("END");
	
	return sdResult;
}

static void int_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	MFC_DRV_DBG_LOG("START");
	
	cdev_del(&int_poll_cdev);
	unregister_chrdev_region(dev, D_INT_POLL_DEVS);
	MFC_DRV_DBG_LOG("END");
}


static int cen_open( struct inode *inode, struct file *filp )
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

static int cen_release( struct inode *inode, struct file *filp )
{
	MFC_DRV_DBG_LOG("");
	return 0;
}

ssize_t cen_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	int				i2c_ret				= -1;
	unsigned char	cmd					= g_readCmd;
	unsigned char	read_buff			= 0;
	struct i2c_msg read_msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &cmd,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,	
			.len	= 1,
			.buf	= &read_buff,	
		},
	};

	MFC_DRV_DBG_LOG("cen_read start\n");
	MFC_DRV_DBG_LOG("cen_read cmd = %x\n", cmd);

	if (buf == NULL) {
		MFC_DRV_DBG_LOG("cen_read param err\n");
		return -EFAULT;
	}
	
	i2c_ret  =i2c_transfer(this_client->adapter, read_msgs, 2);
	MFC_DRV_DBG_LOG("cen_read i2c_ret = %d \n",i2c_ret);
	if (I2C_FAILURE >= i2c_ret) {
		return -EIO;
	}

	read_buff &= FELICA_CONTROL_LOCK_MASK;

	if (copy_to_user(buf, &read_buff, sizeof(read_buff))) {
		MFC_DRV_DBG_LOG("cen_read copy_to_user err \n");
		return -EFAULT;
	}
	MFC_DRV_DBG_LOG("cen_read end\n");
	return (sizeof(read_buff));

}

static long cen_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned char	lock_status[2];	
	int				i2c_ret		= -1;
	int				retry_cnt	= 0;
	unsigned char	read_cmd	= 0;
	struct i2c_msg write_msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 2,
		},
	};
	MFC_DRV_DBG_LOG("START");
	
	switch( cmd ){
		case FELICA_PPC_SET_COMMAND:
			MFC_DRV_DBG_LOG("START FELICA_PPC_SET_COMMAND");
			if (copy_from_user(&read_cmd, (void __user *)arg, sizeof(read_cmd))) {
				MFC_DRV_ERR_LOG("copy_from_user set_COMMAND = %x", read_cmd);
				return -EFAULT;
			}
			MFC_DRV_DBG_LOG("copy_from_user set_COMMAND = %x", read_cmd);
			if( 0xFF == g_readCmd ){
				g_readCmd = read_cmd;
				return 0;
			}else{
				return -1;
			}
			break;
		case FELICA_PPC_WRITE_E2PROM:
			MFC_DRV_DBG_LOG("START FELICA_PPC_WRITE_E2PROM");
			if (copy_from_user(&lock_status, (void __user *)arg, 2)) {
				MFC_DRV_ERR_LOG("copy_from_user write_E2PROM");
				return -EFAULT;
			}

			write_msg[0].buf = lock_status;

			for( retry_cnt = 0; retry_cnt<2; retry_cnt++ ){
				i2c_ret = i2c_transfer(this_client->adapter, write_msg, 1);
				MFC_DRV_DBG_LOG("write_felica_cen ret = %d \n", i2c_ret);
				if( I2C_FAILURE < i2c_ret ){
					break;
				}
				usleep( 10000 );
			}

			if (I2C_FAILURE >= i2c_ret) {
				return -EIO;
			}else{
				MFC_DRV_DBG_LOG("write_felica_cen SUCCESS \n");
				return 1;
			}
		default:
			MFC_DRV_ERR_LOG("cmd = %d", cmd);
			return -EINVAL;
	}
	return 0;
}


static struct file_operations cen_fileops = {
	.owner	 = THIS_MODULE,
	.open	 = cen_open,
	.release = cen_release,
	.read	 = cen_read,
	.unlocked_ioctl	 = cen_ioctl, 
};

static int cen_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
	struct cen_data *cen;
	int alloc_ret = 0;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	MFC_DRV_DBG_LOG("START");
	
	cen = kzalloc(sizeof(struct cen_data), GFP_KERNEL);
	if (!cen) {
		MFC_DRV_ERR_LOG("kzalloc");
		return -ENOMEM;	
	}
	
	i2c_set_clientdata(client, cen);

	cen->input_dev = input_allocate_device();
	
	alloc_ret = alloc_chrdev_region(&dev , 0 , D_CEN_DEVS, D_CEN_DEV_NAME);
	if (alloc_ret) {
		MFC_DRV_ERR_LOG("alloc_chrdev_region");
		return alloc_ret;
	}
	
	cdev_init(&cen_cdev, &cen_fileops);
	cen_cdev.owner = THIS_MODULE;
	
	cdev_add(&cen_cdev, dev, D_CEN_DEVS);
	
	device_create(felica_class, NULL, dev, NULL, D_CEN_DEV_NAME);

	g_readCmd = 0xFF;

	this_client = client;

	MFC_DRV_DBG_LOG("END");
	
	return (0);
}

static int cen_remove(struct i2c_client *client)
{

	dev_t dev;

	struct cen_data *cen = i2c_get_clientdata(client);
	
	MFC_DRV_DBG_LOG("START");

	dev  = MKDEV(MISC_MAJOR, 0);

	device_destroy(felica_class, dev);

	cdev_del(&cen_cdev);

	unregister_chrdev_region(dev, 1);

	input_unregister_device(cen->input_dev);
	
	kfree(cen);
	
	MFC_DRV_DBG_LOG("END");
	
	return (0);
}

static const struct i2c_device_id cen_id[] = {
	{ CEN_IC_NAME, 0 },
	{ }
};

static struct i2c_driver s7780a_driver =
{
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
	},
	.probe	 = cen_probe,
	.id_table = cen_id,
	.remove	 = cen_remove,
};

/*
 * felica_init
 */
static __init int felica_init(void)
{
	int ret;

	felica_class = class_create(THIS_MODULE, "felica");
	
	if (IS_ERR(felica_class)) {
		return PTR_ERR(felica_class);
	}

	ret = rate_init();
	if( ret < 0 ){
		MFC_DRV_ERR_LOG("rate init err\n");
		return ret;
	}

	ret = pon_init();
	if( ret < 0 ){
		MFC_DRV_ERR_LOG("pon init err\n");
		return ret;
	}

	ret = rfs_init();
	if( ret < 0 )
		return ret;

	ret = i2c_add_driver(&s7780a_driver);

	if( ret < 0 ){
		MFC_DRV_ERR_LOG("i2c add driver err\n");
		return ret;
	}

	ret = int_poll_init();
	if( ret < 0 )
		return ret;

	ret = rws_init();
	if( ret < 0 ){
		return ret;
	}
	return 0;
}

/*
 * felica_exit
 */
static void __exit felica_exit(void)
{

	class_destroy( felica_class );

	rate_exit();

	pon_exit();

	rfs_exit();

	i2c_del_driver(&s7780a_driver);

	int_poll_exit();

	rws_exit();

	return;
}


MODULE_LICENSE("GPL v2");

module_init(felica_init);
module_exit(felica_exit);


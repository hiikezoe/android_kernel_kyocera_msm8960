/*****************************************************************************
*
* Filename:      irda_eg.c
* Version:       0.1
* Description:   IrDA driver
* Status:        Experimental
* Author:        KYOCERA Corporation
*
* This software is contributed or developed by KYOCERA Corporation.
* (C) 2011 KYOCERA Corporation
* (C) 2012 KYOCERA Corporation
*
*   This program is free software; you can redistribute it and/or
*   modify it under the terms of the GNU General Public License
*   as published by the Free Software Foundation; only version 2.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/gpio.h>
#include <linux/major.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "irda_eg.h"

#include <mach/msm_serial_hs.h>

extern int irda_eg_set_pm_port(void);
extern int irda_eg_unset_pm_port(void);

/* for MSM8960 UARTDM2 ( GSBI4 IRDA )  register setting */
#define	GSBI4_DM_BASE_REG_ADR 	0x16340000
#define	GSBI4_DM_BASE_REG_LEN 	0x100
#define GSBI4_DM_IRDA_ADDR 	0x38
#define	GSBI4_DM_MODE_NORMAL 	0x00
#define	GSBI4_DM_MODE_IRDA 	0x03
static char		*gsbi4_dm ;

static struct irda_eg_info *irdaeg_info;

static int irdaeg_fop_open(struct inode *inode, struct file *file) ;
static int irdaeg_fop_release(struct inode *inode, struct file *file) ;



/* ------------------------------------------------------------------------------------
 *		irdaeg file operation
 * ------------------------------------------------------------------------------------ */
static int irdaeg_fop_open(struct inode *inode, struct file *file)
{

	printk( "%s:%d\n", __func__, __LINE__ );
	printk( "%s: major %d minor %d (pid %d)\n", __func__, imajor(inode), iminor(inode), current->pid);

	file->private_data = irdaeg_info ;

/* Set IrDA mode at the UART_PM2 register at MSM86550 */
	gsbi4_dm = ioremap_nocache( GSBI4_DM_BASE_REG_ADR, GSBI4_DM_BASE_REG_LEN );
	iowrite32( GSBI4_DM_MODE_IRDA , gsbi4_dm+GSBI4_DM_IRDA_ADDR ) ;

	printk( "%s:%d\n", __func__, __LINE__ );
	return 0;
}

static int irdaeg_fop_release(struct inode *inode, struct file *file)
{
	printk( "%s: major %d minor %d (pid %d)\n", __func__, imajor(inode), iminor(inode), current->pid);

/* Set Normal mode at the UART_PM2 register at MSM86550 */
	iowrite32( GSBI4_DM_MODE_NORMAL , gsbi4_dm+GSBI4_DM_IRDA_ADDR ) ;
	iounmap( gsbi4_dm );


	printk( "%s:%d\n", __func__, __LINE__ );
	return 0;
}


static struct class *irdaeg_class;

struct file_operations irdaeg_fops =
{
	.owner		= THIS_MODULE,
	.open		= irdaeg_fop_open,
	.release	= irdaeg_fop_release,
};

static char *irdaeg_devnode(struct device *dev, mode_t *mode)
{
	printk("irdaeg_devnode\n");

	if (mode)
		*mode = 0666;
	return kasprintf(GFP_KERNEL,"%s", dev_name(dev));
}

static int init_irdaeg( void )
{
	int ret = 0;
	
	printk( KERN_NOTICE"IRDAEG module is beeing initialized.\n" ) ;
	irdaeg_info = kzalloc(sizeof(*irdaeg_info), GFP_KERNEL);
	if (irdaeg_info == NULL) {
		pr_err(MODULE_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}

	irdaeg_class = class_create(THIS_MODULE, MODULE_NAME);

	ret = alloc_chrdev_region(&irdaeg_info->dev_num, 0, 1, MODULE_NAME);
	if (ret) {
		printk(MODULE_NAME "alloc_chrdev_region err.\n");
		return -ENODEV;
	}
	irdaeg_class->devnode = irdaeg_devnode;
	irdaeg_info->dev = device_create(irdaeg_class, NULL, irdaeg_info->dev_num,
				      irdaeg_info, MODULE_NAME);
	if (IS_ERR(irdaeg_info->dev)) {
		printk(MODULE_NAME ":device_create err.\n");
		return -ENODEV;
	}

	irdaeg_info->cdev = cdev_alloc();
	if (irdaeg_info->cdev == NULL) {
		printk(MODULE_NAME ":cdev_alloc err.\n");
		return -ENODEV;
	}
	cdev_init(irdaeg_info->cdev, &irdaeg_fops);
	irdaeg_info->cdev->owner = THIS_MODULE;

	ret = cdev_add(irdaeg_info->cdev, irdaeg_info->dev_num, 1);
	if (ret)
		printk(MODULE_NAME ":cdev_add err=%d\n", -ret);
	else
		printk(MODULE_NAME ":irdaeg init OK..\n");

	printk( " %s driver installed.\n", MODULE_NAME );

	return ret;

}

static void exit_irdaeg( void )
{
	cdev_del(irdaeg_info->cdev);
	device_destroy(irdaeg_class, irdaeg_info->dev_num);
	unregister_chrdev_region(irdaeg_info->dev_num, 1);

	kfree(irdaeg_info);
	printk( "IRDAEG module is removed.\n" ) ;
}

module_init( init_irdaeg ) ;
module_exit( exit_irdaeg ) ;


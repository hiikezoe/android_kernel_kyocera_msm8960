/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 *
 * drivers/input/touchscreen/atmel_mxt_kc.c
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

/* H35 WS1 20120328 A S */
#undef USE_WORKQUEUE
#undef MT_PROTOCOL_TYPE_A
#define MXT_DUMP_OBJECT
/* H35 WS1 20120328 A E */
/* H35 forFact(temporary) 20120328 A S */
#undef FEATURE_TEMP_FACT
/* H35 forFact(temporary) 20120328 A E */
/* H35 ESD 20120328 A S */
#define ESD_RECOVERY
/* H35 ESD 20120328 A E */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
/* H35 WS1 20120328 M S */
/* #include <linux/i2c/atmel_mxt_ts.h> */
#include <linux/i2c/atmel_mxt_kc.h>
#ifdef MT_PROTOCOL_TYPE_A
#include <linux/input.h>
#else /* MT_PROTOCOL_TYPE_A */
#include <linux/input/mt.h>
#endif /* MT_PROTOCOL_TYPE_A */
/* H35 WS1 20120328 M E */
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
/* H35 WS1 20120328 D S */
/* #include <linux/regulator/consumer.h> */
/* H35 WS1 20120328 D E */

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define MXT_SUSPEND_LEVEL 1
#endif
/* H35 LOG 20120329 A S */
#include <linux/fs.h>
#include <linux/namei.h>
#include <asm/uaccess.h>
/* H35 LOG 20120329 A E */
/* H35 CDEV 20120329 A S */
#include <linux/cdev.h>
/* H35 CDEV 20120329 A E */

/* Family ID */
#define MXT224_ID	0x80
#define MXT224E_ID	0x81
#define MXT1386_ID	0xA0

/* Version */
#define MXT_VER_20		20
#define MXT_VER_21		21
#define MXT_VER_22		22

/* Slave addresses */
#define MXT_APP_LOW		0x4a
#define MXT_APP_HIGH		0x4b
#define MXT_BOOT_LOW		0x24
#define MXT_BOOT_HIGH		0x25

/* Firmware */
#define MXT_FW_NAME		"maxtouch.fw"

/* Registers */
#define MXT_FAMILY_ID		0x00
#define MXT_VARIANT_ID		0x01
#define MXT_VERSION		0x02
#define MXT_BUILD		0x03
#define MXT_MATRIX_X_SIZE	0x04
#define MXT_MATRIX_Y_SIZE	0x05
#define MXT_OBJECT_NUM		0x06
#define MXT_OBJECT_START	0x07

#define MXT_OBJECT_SIZE		6

/* Object types */
/* H35 WS1 20120328 M S */
#if 0
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#else /* 0 */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCH_T42		42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCI_ADAPTIVE_T55		55
#define MXT_PROCI_SHIELDLESS_T56	56
#define MXT_PROCI_EXTRA_T57		57
#define MXT_PROCG_NOISE_T62		62
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_SPT_TIMER_T61		61
#endif /* 0 */
/* H35 WS1 20120328 M E */

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* MXT_GEN_POWER_T7 field */
#define MXT_POWER_IDLEACQINT	0
#define MXT_POWER_ACTVACQINT	1
#define MXT_POWER_ACTV2IDLETO	2

/* MXT_GEN_ACQUIRE_T8 field */
#define MXT_ACQUIRE_CHRGTIME	0
#define MXT_ACQUIRE_TCHDRIFT	2
#define MXT_ACQUIRE_DRIFTST	3
#define MXT_ACQUIRE_TCHAUTOCAL	4
#define MXT_ACQUIRE_SYNC	5
#define MXT_ACQUIRE_ATCHCALST	6
#define MXT_ACQUIRE_ATCHCALSTHR	7

/* MXT_TOUCH_MULT_T9 field */
#define MXT_TOUCH_CTRL		0
#define MXT_TOUCH_XORIGIN	1
#define MXT_TOUCH_YORIGIN	2
#define MXT_TOUCH_XSIZE		3
#define MXT_TOUCH_YSIZE		4
#define MXT_TOUCH_BLEN		6
#define MXT_TOUCH_TCHTHR	7
#define MXT_TOUCH_TCHDI		8
#define MXT_TOUCH_ORIENT	9
#define MXT_TOUCH_MOVHYSTI	11
#define MXT_TOUCH_MOVHYSTN	12
#define MXT_TOUCH_NUMTOUCH	14
#define MXT_TOUCH_MRGHYST	15
#define MXT_TOUCH_MRGTHR	16
#define MXT_TOUCH_AMPHYST	17
#define MXT_TOUCH_XRANGE_LSB	18
#define MXT_TOUCH_XRANGE_MSB	19
#define MXT_TOUCH_YRANGE_LSB	20
#define MXT_TOUCH_YRANGE_MSB	21
#define MXT_TOUCH_XLOCLIP	22
#define MXT_TOUCH_XHICLIP	23
#define MXT_TOUCH_YLOCLIP	24
#define MXT_TOUCH_YHICLIP	25
#define MXT_TOUCH_XEDGECTRL	26
#define MXT_TOUCH_XEDGEDIST	27
#define MXT_TOUCH_YEDGECTRL	28
#define MXT_TOUCH_YEDGEDIST	29
#define MXT_TOUCH_JUMPLIMIT	30

/* MXT_PROCI_GRIPFACE_T20 field */
#define MXT_GRIPFACE_CTRL	0
#define MXT_GRIPFACE_XLOGRIP	1
#define MXT_GRIPFACE_XHIGRIP	2
#define MXT_GRIPFACE_YLOGRIP	3
#define MXT_GRIPFACE_YHIGRIP	4
#define MXT_GRIPFACE_MAXTCHS	5
#define MXT_GRIPFACE_SZTHR1	7
#define MXT_GRIPFACE_SZTHR2	8
#define MXT_GRIPFACE_SHPTHR1	9
#define MXT_GRIPFACE_SHPTHR2	10
#define MXT_GRIPFACE_SUPEXTTO	11

/* MXT_PROCI_NOISE field */
#define MXT_NOISE_CTRL		0
#define MXT_NOISE_OUTFLEN	1
#define MXT_NOISE_GCAFUL_LSB	3
#define MXT_NOISE_GCAFUL_MSB	4
#define MXT_NOISE_GCAFLL_LSB	5
#define MXT_NOISE_GCAFLL_MSB	6
#define MXT_NOISE_ACTVGCAFVALID	7
#define MXT_NOISE_NOISETHR	8
#define MXT_NOISE_FREQHOPSCALE	10
#define MXT_NOISE_FREQ0		11
#define MXT_NOISE_FREQ1		12
#define MXT_NOISE_FREQ2		13
#define MXT_NOISE_FREQ3		14
#define MXT_NOISE_FREQ4		15
#define MXT_NOISE_IDLEGCAFVALID	16

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1

/* MXT_SPT_CTECONFIG_T28 field */
#define MXT_CTE_CTRL		0
#define MXT_CTE_CMD		1
#define MXT_CTE_MODE		2
#define MXT_CTE_IDLEGCAFDEPTH	3
#define MXT_CTE_ACTVGCAFDEPTH	4
#define MXT_CTE_VOLTAGE		5

#define MXT_VOLTAGE_DEFAULT	2700000
#define MXT_VOLTAGE_STEP	10000

/* Analog voltage @2.7 V */
#define MXT_VTG_MIN_UV		2700000
#define MXT_VTG_MAX_UV		3300000
#define MXT_ACTIVE_LOAD_UA	15000
#define MXT_LPM_LOAD_UA		10
/* Digital voltage @1.8 V */
#define MXT_VTG_DIG_MIN_UV	1800000
#define MXT_VTG_DIG_MAX_UV	1800000
#define MXT_ACTIVE_LOAD_DIG_UA	10000
#define MXT_LPM_LOAD_DIG_UA	10

#define MXT_I2C_VTG_MIN_UV	1800000
#define MXT_I2C_VTG_MAX_UV	1800000
#define MXT_I2C_LOAD_UA		10000
#define MXT_I2C_LPM_LOAD_UA	10

/* Define for MXT_GEN_COMMAND_T6 */
/* H35 WS1 20120328 A S */
#define MXT_RESET_ORDER		0x01
#define MXT_CALIBRATE_ORDER	0x01
/* H35 WS1 20120328 A E */
#define MXT_BOOT_VALUE		0xa5
#define MXT_BACKUP_VALUE	0x55
#define MXT_BACKUP_TIME		25	/* msec */
#define MXT224_RESET_TIME		65	/* msec */
#define MXT224E_RESET_TIME		22	/* msec */
#define MXT1386_RESET_TIME		250	/* msec */
#define MXT_RESET_TIME		250	/* msec */
/* H35 WS1 20120328 A S */
#define MXT_MAX_RESET_TIME	160	/* msec */
/* H35 WS1 20120328 A E */
#define MXT_RESET_NOCHGREAD		400	/* msec */

#define MXT_FWRESET_TIME	175	/* msec */

#define MXT_WAKE_TIME		25

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f

/* H35 WS1 20120328 A S */
#define MXT_CTRL		0
#define MXT_RPTEN		(1 << 1)

/* Current status */
#define MXT_COMSERR		(1 << 2)
#define MXT_CFGERR		(1 << 3)
#define MXT_CAL			(1 << 4)
#define MXT_SIGERR		(1 << 5)
#define MXT_OFL			(1 << 6)
#define MXT_RESET		(1 << 7)
/* H35 WS1 20120328 A E */

/* Touch status */
#define MXT_SUPPRESS		(1 << 1)
#define MXT_AMP			(1 << 2)
#define MXT_VECTOR		(1 << 3)
#define MXT_MOVE		(1 << 4)
#define MXT_RELEASE		(1 << 5)
#define MXT_PRESS		(1 << 6)
#define MXT_DETECT		(1 << 7)

/* Touch orient bits */
#define MXT_XY_SWITCH		(1 << 0)
#define MXT_X_INVERT		(1 << 1)
#define MXT_Y_INVERT		(1 << 2)

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_MAX_FINGER		10

#define T7_DATA_SIZE		3
#define MXT_MAX_RW_TRIES	3
#define MXT_BLOCK_SIZE		256
/* H35 WS1 20120328 A S */
#define MXT_MAX_RST_TRIES	2
#define T7_BACKUP_SIZE		2
/* H35 WS1 20120328 A E */
/* H35 ESD 20120328 A S */
#define ESD_POLLING_TIME	5000	/* 5 seconds */
/* H35 ESD 20120328 A E */

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
/* H35 WS1 20120328 A S */
	bool report_enable;
/* H35 WS1 20120328 A E */
};

struct mxt_message {
	u8 reportid;
	u8 message[7];
/* H35 WS1 20120328 D S */
/* 	u8 checksum; */
/* H35 WS1 20120328 D E */
};

struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
	int pressure;
};
/* H35 NV 20120329 A S */
#define MXT_N_CHARGE_NV_NAME	"mXT224S_non_charge_nv.bin"
#define MXT_CHARGE_C_NV_NAME	"mXT224S_charge_c_nv.bin"
#define MXT_CHARGE_A_NV_NAME	"mXT224S_charge_a_nv.bin"

struct mxt_config_nv {
	size_t size;
	u16 ver;
	u8 *data;
};
/* H35 NV 20120329 A E */

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info info;
	struct mxt_finger finger[MXT_MAX_FINGER];
	unsigned int irq;
/* H35 WS1 20120328 D S */
#if 0
	struct regulator *vcc_ana;
	struct regulator *vcc_dig;
	struct regulator *vcc_i2c;
#endif /* 0 */
/* H35 WS1 20120328 D E */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

/* H35 WS1 20120328 M S */
#if 0
	u8 t7_data[T7_DATA_SIZE];
#else /* 0 */
	u8 t7_data[T7_BACKUP_SIZE];
#endif /* 0 */
/* H35 WS1 20120328 M E */
	u16 t7_start_addr;
/* H35 WS1 20120328 D S */
#if 0
	u8 t9_ctrl;
#endif /* 0 */
/* H35 WS1 20120328 D E */
/* H35 WS1 20120328 A S */
	unsigned int max_x;
	unsigned int max_y;
	struct mutex lock;
#ifdef USE_WORKQUEUE
	struct work_struct work;
#endif /* USE_WORKQUEUE */
	bool is_suspended;
/* H35 WS1 20120328 A E */
/* H35 SYSFS 20120329 A S */
	int max_o_size;
/* H35 SYSFS 20120329 A E */
/* H35 ESD 20120328 A S */
#ifdef ESD_RECOVERY
	struct delayed_work esdwork;
#endif /* ESD_RECOVERY */
/* H35 ESD 20120328 A E */
/* H35 CDEV 20120329 A S */
	struct cdev device_cdev;
	int device_major;
	struct class *device_class;
/* H35 CDEV 20120329 A E */
/* H35 CONF 20120329 A S */
	int config_status;
/* H35 CONF 20120329 A E */
/* H35 NV 20120329 A S */
	struct mxt_config_nv config_nv[TS_NUM_OF_NV];
/* H35 NV 20120329 A E */
};

/* H35 SYSFS 20120329 A S */
struct sysfs_data_{
	char command;
	u16 start_addr;
	u16 size;
} sdata;

/* sysfs ctrl command list */
#define MXT_SYSFS_LOG_FS		'l'
#define MXT_SYSFS_POLLING		'p'
#define MXT_SYSFS_INT_STATUS		'g'
#define MXT_SYSFS_WRITE			'w'
#define MXT_SYSFS_READ			'r'
#define MXT_SYSFS_CONTIGUOUS_READ	't'

/* H35 SYSFS 20120329 A E */
/* H35 CONF 20120329 A S */
typedef enum {
	MXT_CHARGE_CABLE = 0,
	MXT_CHARGE_ADAPTER,
	MXT_DISCHARGE,
	MXT_STATUS_MAX
}MXT_CHARGE_STATUS;
/* H35 CONF 20120329 A E */

/* H35 LOG 20120329 A S */
enum mxt_log_type {
	MXT_LOG_MSG,
	MXT_LOG_CONF_STAT,
/* H35 20120316 for debug Fujiki A S */
	MXT_LOG_DEAMON,
/* H35 20120316 for debug Fujiki A E */
	MXT_LOG_MAX,
};
static int log_enabled = 1;
/* H35 LOG 20120329 A E */

/* H35 ESD 20120328 A S */
#ifdef ESD_RECOVERY
static int esd_recovery = 1;
#endif /* ESD_RECOVERY */
/* H35 ESD 20120328 A E */
/* H35 20120313 TEMP Fujiki A S */
static int error_enable = 0;
/* H35 20120313 TEMP Fujiki A E */

/* H35 WS1 20120328 A S */
/* H35 ESD 20120328 M S */
/* #ifdef USE_WORKQUEUE */
#if defined(USE_WORKQUEUE) || defined(ESD_RECOVERY)
static struct workqueue_struct *mxt_wq;
#endif /* USE_WORKQUEUE || ESD_RECOVERY */
/* #endif */ /* USE_WORKQUEUE */
/* H35 ESD 20120328 M E */
/* H35 Diag 20120402 A S */
#define MXT_EVENT_ENABLE	0

static int mxt_diag_start_flag = 0;

static struct mxt_diag_type *diag_data;

static struct mutex diag_lock;
static char mxt_event_control = MXT_EVENT_ENABLE;
/* H35 Diag 20120402 A E */

static int mxt_make_highchg(struct mxt_data *data);
static int mxt_stop(struct mxt_data *data);
/* H35 WS1 20120328 A E */

/* H35 LOG 20120329 A S */
static void mxt_write_log(struct mxt_data *data, enum mxt_log_type type,
						void *arg)
{
	struct device *dev = &data->client->dev;
	struct file *fp;
	mm_segment_t old_fs;
	char *filename = "/data/ts_log";
	char buf[100];
	int len;
	int error;
	struct path path;
	struct mxt_message *message;
	int *flag;

	if (!log_enabled)
		return;

	/* change to KERNEL_DS address limit */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	error = kern_path(filename, LOOKUP_FOLLOW, &path);
	if(error) {
		fp = filp_open(filename, O_CREAT, S_IRUGO|S_IWUSR);
		dev_dbg(dev, "created /data/ts_log.\n");
		len = sprintf(buf, "Report ID,Data01,Data02,Data03,Data04,"
						"Data05,Data06,Data07\n");
		fp->f_op->write(fp, buf, len, &(fp->f_pos));
		memset(buf, '\0', sizeof(buf));
	} else
		fp = filp_open(filename, O_WRONLY | O_APPEND, 0);

	switch (type) {
	case MXT_LOG_MSG:
		message = arg;
		len = sprintf(buf, "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
							message->reportid,
							message->message[0],
							message->message[1],
							message->message[2],
							message->message[3],
							message->message[4],
							message->message[5],
							message->message[6]);
		fp->f_op->write(fp, buf, len, &(fp->f_pos));
		break;
	case MXT_LOG_CONF_STAT:
		switch (data->config_status) {
		case MXT_CHARGE_CABLE:
			len = sprintf(buf, "Change config_status to "
							"MXT_CHARGE_CABLE.\n");
			fp->f_op->write(fp, buf, len, &(fp->f_pos));
			break;
		case MXT_CHARGE_ADAPTER:
			len = sprintf(buf, "Change config_status to "
							"MXT_CHARGE_ADAPTER.\n");
			fp->f_op->write(fp, buf, len, &(fp->f_pos));
			break;
		case MXT_DISCHARGE:
			len = sprintf(buf, "Change config_status to "
							"MXT_DISCHARGE.\n");
			fp->f_op->write(fp, buf, len, &(fp->f_pos));
			break;
		default:
			break;
		}
		break;
	case MXT_LOG_DEAMON:
		flag = arg;
		len = sprintf(buf, "ts_deamon log[%d]\n", *flag);
		fp->f_op->write(fp, buf, len, &(fp->f_pos));
		break;
	default:
		break;
	}

	/* close file before return */
	if (fp)
		filp_close(fp, current->files);
	/* restore previous address limit */
	set_fs(old_fs);
}
/* H35 LOG 20120329 A E */

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
/* H35 WS1 20120328 M S */
#if 0
	case MXT_GEN_MESSAGE_T5:
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
#else /* 0 */
	case MXT_DEBUG_DIAGNOSTIC_T37:
	case MXT_GEN_MESSAGE_T5:
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCH_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCI_ADAPTIVE_T55:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_PROCI_EXTRA_T57:
	case MXT_PROCG_NOISE_T62:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_MESSAGECOUNT_T44:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_SPT_TIMER_T61:
#endif /* 0 */
/* H35 WS1 20120328 M E */
		return true;
	default:
		return false;
	}
}

static bool mxt_object_writable(unsigned int type)
{
	switch (type) {
/* H35 WS1 20120328 M S */
#if 0
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
#else /* 0 */
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_TOUCH_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCI_ADAPTIVE_T55:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_PROCI_EXTRA_T57:
	case MXT_PROCG_NOISE_T62:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_SPT_TIMER_T61:
#endif /* 0 */
/* H35 WS1 20120328 M E */
		return true;
	default:
		return false;
	}
}

/* H35 WS1 20120328 A S */
static bool mxt_object_exist_rpten(unsigned int type)
{
	switch (type) {
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_PROCI_TOUCH_T42:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_PROCI_EXTRA_T57:
	case MXT_PROCG_NOISE_T62:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_TIMER_T61:
		return true;
	default:
		return false;
	}
}
/* H35 WS1 20120328 A E */

static void mxt_dump_message(struct device *dev,
				  struct mxt_message *message)
{
/* H35 20120228 forDebug Fujiki C S */
#if 0
	dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
	dev_dbg(dev, "message1:\t0x%x\n", message->message[0]);
	dev_dbg(dev, "message2:\t0x%x\n", message->message[1]);
	dev_dbg(dev, "message3:\t0x%x\n", message->message[2]);
	dev_dbg(dev, "message4:\t0x%x\n", message->message[3]);
	dev_dbg(dev, "message5:\t0x%x\n", message->message[4]);
	dev_dbg(dev, "message6:\t0x%x\n", message->message[5]);
	dev_dbg(dev, "message7:\t0x%x\n", message->message[6]);
	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum);
#else /* 0 */
//	if (message->reportid != 0xFF) {
		dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
		dev_dbg(dev, "message1:\t0x%x\n", message->message[0]);
//	}
#endif /* 0 */
/* H35 20120228 forDebug Fujiki C E */
/* H35 WS1 20120328 D S */
/* 	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum); */
/* H35 WS1 20120328 D E */
}

static int mxt_check_bootloader(struct i2c_client *client,
				     unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int i = 0;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	do {
		if (i2c_transfer(client->adapter, xfer, 2) == 2)
			return 0;
		msleep(MXT_WAKE_TIME);
	} while (++i < MXT_MAX_RW_TRIES);

	dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
	return -EIO;
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __mxt_read_reg(client, reg, 1, val);
}

static int __mxt_write_reg(struct i2c_client *client,
		    u16 addr, u16 length, u8 *value)
{
	u8 buf[MXT_BLOCK_SIZE + 2];
	int i, tries = 0;

	if (length > MXT_BLOCK_SIZE)
		return -EINVAL;

	buf[0] = addr & 0xff;
	buf[1] = (addr >> 8) & 0xff;
	for (i = 0; i < length; i++)
		buf[i + 2] = *value++;

	do {
		if (i2c_master_send(client, buf, length + 2) == (length + 2))
			return 0;
		msleep(MXT_WAKE_TIME);
	} while (++tries < MXT_MAX_RW_TRIES);

	dev_err(&client->dev, "%s: i2c send failed\n", __func__);
	return -EIO;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static int mxt_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __mxt_read_reg(client, reg, MXT_OBJECT_SIZE,
				   object_buf);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

static int mxt_read_message(struct mxt_data *data,
				 struct mxt_message *message)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg,
			sizeof(struct mxt_message), message);
}

static int mxt_read_object(struct mxt_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg + offset, 1, val);
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

static void mxt_input_report(struct mxt_data *data, int single_id)
{
	struct mxt_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int status = finger[single_id].status;
	int finger_num = 0;
	int id;

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;

/* H35 WS1 20120328 M S */
#ifdef MT_PROTOCOL_TYPE_A
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
				finger[id].status != MXT_RELEASE ?
				finger[id].area : 0);
		input_report_abs(input_dev, ABS_MT_POSITION_X,
				finger[id].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y,
				finger[id].y);
		input_report_abs(input_dev, ABS_MT_PRESSURE,
				finger[id].status != MXT_RELEASE ?
				finger[id].pressure : 0);
		input_mt_sync(input_dev);

		if (finger[id].status == MXT_RELEASE)
			finger[id].status = 0;
		else
			finger_num++;
#else /* MT_PROTOCOL_TYPE_A */
		input_mt_slot(input_dev, id);
		dev_dbg(&data->client->dev, "input event : slot[%02x]\n", id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
				finger[id].status != MXT_RELEASE);

		if (finger[id].status != MXT_RELEASE) {
			finger_num++;
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
					finger[id].area);
			input_report_abs(input_dev, ABS_MT_POSITION_X,
					finger[id].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
					finger[id].y);
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					finger[id].pressure);
			dev_dbg(&data->client->dev, "input event : "
			"touch_major[%d], pos_x[%d], pos_y[%d], pressure[%d]\n"
			, finger[id].area
			, finger[id].x
			, finger[id].y
			, finger[id].pressure);
		} else {
			finger[id].status = 0;
		}
#endif /* MT_PROTOCOL_TYPE_A */
/* H35 WS1 20120328 M E */
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

	if (status != MXT_RELEASE) {
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
		input_report_abs(input_dev,
			ABS_PRESSURE, finger[single_id].pressure);
	}

	input_sync(input_dev);
}

/* H35 WS1 20120328 A S */
static void mxt_input_report_clear(struct mxt_data *data)
{
	struct mxt_finger *finger = data->finger;
	int id;

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;
		finger[id].status = MXT_RELEASE;
		dev_dbg(&data->client->dev, "%s:[%d] released\n", __func__, id);
		mxt_input_report(data, id);
	}
}
/* H35 WS1 20120328 A E */

static void mxt_input_touchevent(struct mxt_data *data,
				      struct mxt_message *message, int id)
{
	struct mxt_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;
	int pressure;

	/* Check the touch is present on the screen */
	if (!(status & MXT_DETECT)) {
		if (status & MXT_RELEASE) {
			dev_dbg(dev, "[%d] released\n", id);

			finger[id].status = MXT_RELEASE;
			mxt_input_report(data, id);
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (MXT_PRESS | MXT_MOVE)))
		return;

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
	y = (message->message[2] << 4) | ((message->message[3] & 0xf));
/* H35 WS1 20120328 M S */
#if 0
	if (data->pdata->x_size < 1024)
		x = x >> 2;
	if (data->pdata->y_size < 1024)
		y = y >> 2;
#else /* 0 */
	if (data->max_x < 1024)
		x = x >> 2;
	if (data->max_y < 1024)
		y = y >> 2;
#endif /* 0 */
/* H35 WS1 20120328 M E */

	area = message->message[4];
	pressure = message->message[5];

	dev_dbg(dev, "[%d] %s x: %d, y: %d, area: %d\n", id,
		status & MXT_MOVE ? "moved" : "pressed",
		x, y, area);

	finger[id].status = status & MXT_MOVE ?
				MXT_MOVE : MXT_PRESS;
	finger[id].x = x;
	finger[id].y = y;
	finger[id].area = area;
	finger[id].pressure = pressure;

	mxt_input_report(data, id);
}

/* H35 WS1 20120328 A S */
static int mxt_check_reset_report(struct mxt_data *data)
{
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int ret = 0;
	u8 t6_reportid;

	object = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	if (!object) {
		dev_err(dev, "Failed to get T6 object!\n");
		ret = -EIO;
		goto error;
	}
	t6_reportid = object->max_reportid;

	do {
		ret = mxt_read_message(data, &message);
		if (ret) {
			dev_err(dev, "Failed to read message!\n");
			goto error;
		}
/* H35 20120307 forDebug Fujiki A S */
		dev_dbg(dev, "%s: read report id is [%02x]\n", __func__,
							message.reportid);
		dev_dbg(dev, "%s: read status is [%02x]\n", __func__,
							message.message[0]);
/* H35 20120307 forDebug Fujiki A E */

		if ((t6_reportid == message.reportid) &&
		    (message.message[0] & MXT_RESET))
			return 0;
		else
			mxt_dump_message(dev, &message);
	} while (message.reportid != 0xff);

error:
	dev_err(dev, "%s:Failed to get T6 Reset Report!\n", __func__);
	return ret;
};

static int mxt_wait_interrupt(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	int i;
	int val;

	for (i = 0; i < (MXT_MAX_RESET_TIME / 10); i++) {
		val = gpio_get_value(pdata->irq_gpio);
		if (val == 0) {
			dev_dbg(dev, "%s: %d0 ms wait.\n", __func__, i);
			return 0;
		}
		msleep(10);
	}

	return -ETIME;
}

static int mxt_reset_and_delay(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	int i;
	int error = 0;

	for (i = 0; i < MXT_MAX_RST_TRIES; i++) {
		if (pdata->reset_hw)
			error = pdata->reset_hw();
		if (error) {
			dev_err(dev, "Failed to reset hardware!\n");
			goto done;
		}
		error = mxt_wait_interrupt(data);
		if (!error)
			goto done;
		dev_err(dev, "%s: Reset Retry %d times.\n", __func__, i + 1);
	}
	dev_err(dev, "Interrupt wait time out!\n");
done:
	return error;
}

static int mxt_restart(struct mxt_data *data)
{
	int error = 0;

	dev_dbg(&data->client->dev, "%s: start\n", __func__);
	/* Reset, then wait interrupt */
	error = mxt_reset_and_delay(data);
	if (error)
		return error;

	/* Check T6 reset report  */
	error = mxt_check_reset_report(data);
	if (error)
		return error;

	/* Send release event as needed */
	mxt_input_report_clear(data);

	dev_dbg(&data->client->dev, "%s: end\n", __func__);
	return 0;
}

static int mxt_error_process(struct mxt_data *data, struct mxt_message *message)
{

	struct device *dev = &data->client->dev;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;
	struct mxt_object *object;
	int error = 0;
	u8 status = message->message[0];
	int i;

	dev_dbg(dev, "Run %s!\n", __func__);

	reportid = message->reportid;

	/* Check if received T6 error message */
	object = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	if (!object) {
		dev_err(dev, "Failed to get T6 object!\n");
		error = -EIO;
		goto done;
	}
	max_reportid = object->max_reportid;
	min_reportid = max_reportid - object->num_report_ids + 1;
	if ((reportid >= min_reportid) && (reportid <= max_reportid)) {
		if ((status & MXT_OFL) || (status & MXT_CFGERR)) {
			dev_err(dev, "%s:Received GEN_COMMANDPROCESSOR_T6"
						"[%02X]!\n", __func__, status);
			goto error;
		}
		goto done;
	}

	/* Check if received invalid messages */
	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_exist_rpten(object->type))
			continue;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		if ((reportid >= min_reportid) &&
		    (reportid <= max_reportid) &&
		    (!object->report_enable)) {
			dev_dbg(dev, "%s:Received T[%d] message even though"
							"RPTEN is not set!\n"
							, __func__
							, object->type);
			goto error;
		}
	}
	goto done;
error:
/* H35 20120313 TEMP Fujiki A S */
	error_enable++;
	if (error_enable > 0) {
		dev_err(dev, "Skip error process!\n");
		goto done;
	}
/* H35 20120313 TEMP Fujiki A E */
	/* Maybe abnormal state, then restart */
	error = mxt_restart(data);
	if (error)
		dev_err(dev, "Failed to restart!\n");
done:
	mxt_dump_message(dev, message);
	return error;
}
/* H35 WS1 20120328 A E */

/* H35 WS1 20120328 A S */
#ifndef USE_WORKQUEUE
/* H35 WS1 20120328 A E */
static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;

/* H35 ESD 20120328 A S */
#ifdef ESD_RECOVERY
	mutex_lock(&data->lock);
#endif /* ESD_RECOVERY */
/* H35 ESD 20120328 A E */
/* H35 WS1 20120328 A S */
	if (!data->is_suspended) {
/* H35 WS1 20120328 A E */
		do {
			if (mxt_read_message(data, &message)) {
				dev_err(dev, "Failed to read message\n");
				goto end;
			}
			reportid = message.reportid;

			/* whether reportid is thing of MXT_TOUCH_MULTI_T9 */
			object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
			if (!object)
				goto end;
			max_reportid = object->max_reportid;
			min_reportid = max_reportid - object->num_report_ids + 1;
			id = reportid - min_reportid;

/* H35 WS1 20120328 M S */
/* 			if (reportid >= min_reportid && reportid <= max_reportid) */
			if ((reportid >= min_reportid) &&
			    (reportid <= max_reportid) &&
			    (object->report_enable))
/* H35 WS1 20120328 M E */
				mxt_input_touchevent(data, &message, id);
/* H35 WS1 20120328 A S */
			else if (reportid != 0xff)
#if 0
				mxt_dump_message(dev, &message);
#else /* 0 */
				mxt_error_process(data, &message);
#endif /* 0 */
/* H35 WS1 20120328 A E */
			else
				mxt_dump_message(dev, &message);
/* H35 LOG 20120329 A S */
			mxt_write_log(data, MXT_LOG_MSG, &message);
/* H35 LOG 20120329 A E */
		} while (reportid != 0xff);
/* H35 WS1 20120328 A S */
	} else {
		/* Maybe reset occur.
		 * Read dummy message to make high CHG pin,
		 * then configure deep sleep mode.
		 */
		dev_info(dev, "Device should be suspended. "
						"Configure deep sleep mode.\n");
		if (mxt_stop(data))
			dev_err(dev, "mxt_stop failed in mxt_interrupt\n");
		mxt_dump_message(dev, &message);
	}
/* H35 WS1 20120328 A E */

end:
/* H35 ESD 20120328 A S */
#ifdef ESD_RECOVERY
	mutex_unlock(&data->lock);
#endif /* ESD_RECOVERY */
/* H35 ESD 20120328 A E */
/* H35 20120228 forDebug Fujiki A S */
	dev_dbg(dev, "CPU[%d]%s() is finished.\n"
						, smp_processor_id()
						, __func__);
/* H35 20120228 forDebug Fujiki A E */
	return IRQ_HANDLED;
}
/* H35 WS1 20120328 A S */
#endif /* USE_WORKQUEUE */
/* H35 WS1 20120328 A E */

/* H35 WS1 20120328 A S */
static void mxt_check_rpten(struct mxt_data *data,
				struct mxt_object *object, int index)
{
	const struct mxt_platform_data *pdata = data->pdata;

	if (mxt_object_exist_rpten(object->type)) {
		object->report_enable = pdata->config[index] & MXT_RPTEN;
		dev_dbg(&data->client->dev, "%s:t[%d] RPTEN is [%d])\n"
					, __func__
					, object->type
					, object->report_enable);
	}
}
/* H35 WS1 20120328 A E */

static int mxt_check_reg_init(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int index = 0;
	int i, j, config_offset;

	if (!pdata->config) {
		dev_dbg(dev, "No cfg data defined, skipping reg init\n");
		return 0;
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_writable(object->type))
			continue;

		for (j = 0; j < object->size + 1; j++) {
			config_offset = index + j;
			if (config_offset > pdata->config_length) {
				dev_err(dev, "Not enough config data!\n");
				return -EINVAL;
			}
			mxt_write_object(data, object->type, j,
					 pdata->config[config_offset]);
		}
/* H35 WS1 20120328 A S */
		mxt_check_rpten(data, object, index);
/* H35 WS1 20120328 A E */
		index += object->size + 1;
	}

	return 0;
}

static int mxt_make_highchg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_message message;
/* H35 WS1 20120328 M S */
/* 	int count = 10; */
	int count = 23;
/* H35 WS1 20120328 M E */
	int error;
/* H35 WS1 20120328 A S */
	int tmp;
	struct mxt_object *object;

	object = data->object_table + (data->info.object_num - 1);
	tmp = object->max_reportid + 1;
	dev_dbg(dev, "%s:tmp = [%d]\n", __func__, tmp);
	if (tmp)
		count = tmp;
	dev_dbg(dev, "%s:count = [%d]\n", __func__, count);
/* H35 WS1 20120328 A E */

	/* Read dummy message to make high CHG pin */
	do {
		error = mxt_read_message(data, &message);
		if (error)
			return error;
	} while (message.reportid != 0xff && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared\n");
		return -EBUSY;
	}

	return 0;
}

static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;

	error = mxt_read_reg(client, MXT_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = mxt_read_reg(client, MXT_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = mxt_read_reg(client, MXT_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = mxt_read_reg(client, MXT_BUILD, &val);
	if (error)
		return error;
	info->build = val;

/* H35 WS1 20120328 A S */
	/* Update matrix size at info struct */
	error = mxt_read_reg(client, MXT_MATRIX_X_SIZE, &val);
	if (error)
		return error;
	info->matrix_xsize = val;

	error = mxt_read_reg(client, MXT_MATRIX_Y_SIZE, &val);
	if (error)
		return error;
	info->matrix_ysize = val;
/* H35 WS1 20120328 A E */

	error = mxt_read_reg(client, MXT_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct mxt_object *object = data->object_table + i;

		reg = MXT_OBJECT_START + MXT_OBJECT_SIZE * i;
		error = mxt_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
					(object->instances + 1);
			object->max_reportid = reportid;
		}
/* H35 SYSFS 20120329 A S */
		if (data->max_o_size < (object->size + 1))
			data->max_o_size = object->size + 1;
/* H35 SYSFS 20120329 A E */
	}

	return 0;
}

static void mxt_reset_delay(struct mxt_data *data)
{
	struct mxt_info *info = &data->info;

	switch (info->family_id) {
	case MXT224_ID:
		msleep(MXT224_RESET_TIME);
		break;
	case MXT224E_ID:
		msleep(MXT224E_RESET_TIME);
		break;
	case MXT1386_ID:
		msleep(MXT1386_RESET_TIME);
		break;
	default:
		msleep(MXT_RESET_TIME);
	}
}

/* H35 WS1 20120328 A S */
#ifdef MXT_DUMP_OBJECT
static int mxt_dump_object(struct mxt_data *data)
{
	struct mxt_object *object;
	int i, j;
	int error = 0;
	u8 val;
	char buf[700];
	char str[80];

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		sprintf(str, "\nObject table Element[%d] (Type : %d)",
						i + 1, object->type);
		strcat(buf, str);
		if (!mxt_object_readable(object->type)) {
			strcat(buf, "\n");
			continue;
		}

		for (j = 0; j < object->size + 1; j++) {
			if (!(j % 10)) {
				strcat(buf, "\n");
			}
			error = mxt_read_object(data,
						object->type, j, &val);
			if (error) {
				dev_dbg(&data->client->dev,
					"t%d[%02d] read error!\n",
					object->type, j);
				goto done;
			}
			sprintf(str, "0x%02x ", val);
			strcat(buf, str);
		}
		dev_dbg(&data->client->dev, "buf length %d", strlen(buf));
		dev_info(&data->client->dev, "%s",buf);
		memset(buf, '\0', sizeof(buf));
	}
done:
	return error;
};
#endif /* MXT_DUMP_OBJECT */
/* H35 WS1 20120328 A E */

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
/* H35 TEMP 20120403 D S */
#if 0
	int timeout_counter = 0;
/* H35 WS1 20120328 D S */
/* 	u8 val; */
/* H35 WS1 20120328 D E */
	u8 command_register;
#endif
/* H35 TEMP 20120403 D E */
	struct mxt_object *t7_object;

	error = mxt_get_info(data);
	if (error)
		return error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		goto free_object_table;

/* H35 WS1 20120328 A S */
	/* Check T6 reset report */
	error = mxt_check_reset_report(data);
	if (error)
		goto free_object_table;
/* H35 WS1 20120328 A E */

	/* Check register init values */
	error = mxt_check_reg_init(data);
	if (error)
		goto free_object_table;

/* H35 CONF 20120329 A S */
	data->config_status = MXT_CHARGE_CABLE;
/* H35 CONF 20120329 A E */

/* H35 WS1 20120328 A S */
#ifdef MXT_DUMP_OBJECT
	dev_info(&client->dev, "mXT224E Object dump after config\n");
	mxt_dump_object(data);
#endif /* MXT_DUMP_OBJECT */
/* H35 WS1 20120328 A E */

	/* Store T7 and T9 locally, used in suspend/resume operations */
	t7_object = mxt_get_object(data, MXT_GEN_POWER_T7);
	if (!t7_object) {
		dev_err(&client->dev, "Failed to get T7 object\n");
		error = -EINVAL;
		goto free_object_table;
	}

	data->t7_start_addr = t7_object->start_address;
	error = __mxt_read_reg(client, data->t7_start_addr,
/* H35 WS1 20120328 M S */
#if 0
				T7_DATA_SIZE, data->t7_data);
#else /* 0 */
				T7_BACKUP_SIZE, data->t7_data);
#endif /* 0 */
/* H35 WS1 20120328 M E */
	if (error < 0) {
		dev_err(&client->dev,
			"Failed to save current power state\n");
		goto free_object_table;
	}
/* H35 WS1 20120328 D S */
#if 0
	error = mxt_read_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL,
			&data->t9_ctrl);
	if (error < 0) {
		dev_err(&client->dev, "Failed to save current touch object\n");
		goto free_object_table;
	}
#endif /* 0 */
/* H35 WS1 20120328 D E */

/* H35 TEMP 20120403 D S */
#if 0
	/* Backup to memory */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_BACKUPNV,
			MXT_BACKUP_VALUE);
	msleep(MXT_BACKUP_TIME);
	do {
		error =  mxt_read_object(data, MXT_GEN_COMMAND_T6,
					MXT_COMMAND_BACKUPNV,
					&command_register);
		if (error)
			goto free_object_table;
		usleep_range(1000, 2000);
	} while ((command_register != 0) && (++timeout_counter <= 100));
	if (timeout_counter > 100) {
		dev_err(&client->dev, "No response after backup!\n");
		error = -EIO;
		goto free_object_table;
	}
#endif
/* H35 TEMP 20120403 D E */


/* H35 WS1 20120328 D S */
#if 0
	/* Soft reset */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_RESET, 1);

	mxt_reset_delay(data);

	/* Update matrix size at info struct */
	error = mxt_read_reg(client, MXT_MATRIX_X_SIZE, &val);
	if (error)
		goto free_object_table;
	info->matrix_xsize = val;

	error = mxt_read_reg(client, MXT_MATRIX_Y_SIZE, &val);
	if (error)
		goto free_object_table;
	info->matrix_ysize = val;
#endif /* 0 */
/* H35 WS1 20120328 D E */

	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

	return 0;

free_object_table:
	kfree(data->object_table);
	return error;
}

/* H35 WS1 20120328 A S */
static void mxt_calc_resolution(struct mxt_data *data)
{
	unsigned int max_x = data->pdata->x_size - 1;
	unsigned int max_y = data->pdata->y_size - 1;

	if (data->pdata->orient & MXT_XY_SWITCH) {
		data->max_x = max_y;
		data->max_y = max_x;
	} else {
		data->max_x = max_x;
		data->max_y = max_y;
	}
/* H35 20120228 forDebug Fujiki A S */
	dev_dbg(&data->client->dev, "CPU[%d]%s() max_x[%d], max_y[%d].\n"
						, smp_processor_id()
						, __func__
						, data->max_x
						, data->max_y);
/* H35 20120228 forDebug Fujiki A E */
}
/* H35 WS1 20120328 A E */

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;
/* H35 WS1 20120328 A S */
	int num;
/* H35 WS1 20120328 A E */

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

/* H35 WS1 20120328 M S */
/* 		count += snprintf(buf + count, PAGE_SIZE - count, */
/* 				"Object[%d] (Type %d)\n", */
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"Object table Element[%d] (Type : %d)",
/* H35 WS1 20120328 M E */
				i + 1, object->type);
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;

		if (!mxt_object_readable(object->type)) {
/* H35 WS1 20120328 M S */
/* 			count += snprintf(buf + count, PAGE_SIZE - count, */
			count += scnprintf(buf + count, PAGE_SIZE - count,
/* H35 WS1 20120328 M E */
					"\n");
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
			continue;
		}

/* H35 WS1 20120328 M S */
/* 		for (j = 0; j < object->size + 1; j++) { */
		num = (object->size + 1) * (object->instances + 1);
		for (j = 0; j < num; j++) {
/* H35 WS1 20120328 M E */
/* H35 WS1 20120328 A S */
			if (!(j % 10))
				count += scnprintf(buf + count
						, PAGE_SIZE - count
						, "\n");
/* H35 WS1 20120328 A E */
			error = mxt_read_object(data,
						object->type, j, &val);
			if (error)
				return error;

/* H35 WS1 20120328 M S */
/* 			count += snprintf(buf + count, PAGE_SIZE - count, */
/* 					"\t[%2d]: %02x (%d)\n", j, val, val); */
			count += scnprintf(buf + count, PAGE_SIZE - count,
							"0x%02x ", val);
/* H35 WS1 20120328 M E */
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
		}

/* H35 WS1 20120328 M S */
/* 		count += snprintf(buf + count, PAGE_SIZE - count, "\n"); */
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
/* H35 WS1 20120328 M E */
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;
	}

	return count;
}

/* H35 SYSFS 20120329 A S */
static ssize_t mxt_object_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	u8 *odata;
	unsigned int type;
	const char *p;
	int n = 0;
	int i;
	int offset = 0;
	unsigned int tmp;

	odata = kcalloc(data->max_o_size, sizeof(u8), GFP_KERNEL);
	if (!odata) {
		dev_err(&data->client->dev, "Failed to allocate memory!\n");
		count = -ENOMEM;
		goto done;
	}

	sscanf(buf,"%u ", &type);
	tmp = type;
	while (tmp > 0) {
		offset++;
		tmp /= 10;
	}

	object = mxt_get_object(data, (u8)type);
	if (!object) {
		dev_err(&data->client->dev, "Invalid Object Type!\n");
		count = -EINVAL;
		goto done;
	}

	p = buf + offset;
	while (sscanf(p, "%x %n", &tmp, &offset) == 1) {
		odata[n] = (u8)tmp;
		n++;
		p += offset;
	}

	if (n < (object->size + 1)) {
		dev_err(&data->client->dev, "Too short Parameters!\n");
		count = -EINVAL;
		goto done;
	} else if (n > (object->size + 1)) {
		dev_err(&data->client->dev, "Too long Parameters!\n");
		count = -EINVAL;
		goto done;
	}

	for (i = 0; i < object->size + 1; i++) {
		if (mxt_write_object(data, object->type, i, odata[i])) {
			dev_err(&data->client->dev, "failed to write object!\n");
			count = -EIO;
			goto done;
		}
	}
done:
	kfree(odata);
	return count;
}
/* H35 SYSFS 20120329 A E */

/* H35 NV 20120404 A S */
static int mxt_get_nv(struct device *dev, unsigned long arg,
					enum mxt_nv_type nv_type)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;
	void __user *p = (void __user *)arg;
	size_t size;
/* H35 forDebug 20120404 A S */
	int i;
/* H35 forDebug 20120404 A E */

	ret = copy_from_user(&size, p, sizeof(size_t));
	if (ret){
		dev_err(dev, "%s: copy_from_user error\n", __func__);
		goto done;
	}

	p += sizeof(size_t);
	ret = copy_from_user(&data->config_nv[nv_type].ver, p, sizeof(u16));
	if (ret){
		dev_err(dev, "%s: copy_from_user error\n", __func__);
		goto done;
	}

	data->config_nv[nv_type].size = size - sizeof(size_t) - sizeof(u16);
	data->config_nv[nv_type].data = kcalloc(data->config_nv[nv_type].size,
						sizeof(u8), GFP_KERNEL);
	if (!data->config_nv[nv_type].data) {
		dev_err(dev, "Failed to allocate memory!\n");
		ret = -ENOMEM;
		goto done;
	}

	p += sizeof(u16);
	ret = copy_from_user(data->config_nv[nv_type].data, p,
					data->config_nv[nv_type].size);
	if (ret){
		dev_err(dev, "%s: copy_from_user error\n", __func__);
		goto done;
	}
/* H35 forDebug 20120404 A S */
	dev_dbg(dev, "%s: nv file type[%d] is opened.\n", __func__, nv_type);
	dev_dbg(dev, "ver. %04X", data->config_nv[nv_type].ver);
	for (i = 0; i < data->config_nv[nv_type].size; i++) {
		dev_dbg(dev, "val[%d]: %02X\n", i, data->config_nv[nv_type].data[i]);
	}
/* H35 forDebug 20120404 A E */
done:
	return ret;
}
/* H35 NV 20120404 A E */

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_RESET, MXT_BOOT_VALUE);

	mxt_reset_delay(data);

	/* Change to slave address of bootloader */
	if (client->addr == MXT_APP_LOW)
		client->addr = MXT_BOOT_LOW;
	else
		client->addr = MXT_BOOT_HIGH;

	ret = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	mxt_unlock_bootloader(client);

	while (pos < fw->size) {
		ret = mxt_check_bootloader(client,
						MXT_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* Write one frame to device */
		mxt_fw_write(client, fw->data + pos, frame_size);

		ret = mxt_check_bootloader(client,
						MXT_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);

	/* Change to slave address of application */
	if (client->addr == MXT_BOOT_LOW)
		client->addr = MXT_APP_LOW;
	else
		client->addr = MXT_APP_HIGH;

	return ret;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	disable_irq(data->irq);

	error = mxt_load_fw(dev, MXT_FW_NAME);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_dbg(dev, "The firmware update succeeded\n");

		/* Wait for reset */
		msleep(MXT_FWRESET_TIME);

		kfree(data->object_table);
		data->object_table = NULL;

		mxt_initialize(data);
	}

	enable_irq(data->irq);

	error = mxt_make_highchg(data);
	if (error)
		return error;

	return count;
}

/* H35 SYSFS 20120329 A S */
static ssize_t mxt_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const char *p;
	unsigned int val;
	int offset = 0;
	u16 reg_addr;

	switch (buf[0]) {
	case MXT_SYSFS_LOG_FS:
		sscanf(buf, "%c %x", &sdata.command, (unsigned int *) &log_enabled);
		dev_dbg(dev, "log_enabled is set to %d\n", log_enabled);
		break;
#ifdef ESD_RECOVERY
	case MXT_SYSFS_POLLING:
		sscanf(buf, "%c %x", &sdata.command, (unsigned int *) &esd_recovery);
		dev_dbg(dev, "esd_recovery is set to %d\n", esd_recovery);
		break;
#endif /* ESD_RECOVERY */
	case MXT_SYSFS_INT_STATUS:
		sscanf(buf, "%c", &sdata.command);
		break;
	case MXT_SYSFS_WRITE:
		sscanf(buf, "%c", &sdata.command);
		p = buf + 1;
		if (sscanf(p, " %x%n", (unsigned int *)&sdata.start_addr,
								&offset) != 1) {
			dev_err(&data->client->dev, "Too short Parameters!\n");
			break;
		}
		p += offset;
		sdata.size = 0;
		dev_dbg(dev, "MXT_SYSFS_WRITE: start_address is [%04X]. "
					"offset is [%d]\n", sdata.start_addr, offset);
		reg_addr = sdata.start_addr;
		while (sscanf(p, " %x%n", &val, &offset) == 1) {
			mxt_write_reg(data->client,
					reg_addr, (u8)val);
			p += offset;
			reg_addr++;
			sdata.size++;
		}
		dev_dbg(dev, "MXT_SYSFS_WRITE result: size is [%d].\n",
								sdata.size);
		break;
	case MXT_SYSFS_READ:
		sscanf(buf, "%c %x %x", &sdata.command,
			(unsigned int *)&sdata.start_addr,
			(unsigned int *)&sdata.size);
		dev_dbg(dev, "MXT_SYSFS_READ result: start_addr is [%04X]. "
				"size is [%d].\n", sdata.start_addr, sdata.size);
		break;
/* H35 otameshi 20120409 A S */
	case MXT_SYSFS_CONTIGUOUS_READ:
		sscanf(buf, "%c %x %x", &sdata.command,
			(unsigned int *)&sdata.start_addr,
			(unsigned int *)&sdata.size);
		dev_dbg(dev, "MXT_SYSFS_CONTIGUOUS_READ result: start_addr"
					" is [%04X]. size is [%d].\n",
					sdata.start_addr,
					sdata.size);
		break;
/* H35 otameshi 20120409 A E */
	default:
		break;
	}

	return count;
}

/* H35 Diag 20120403 A S */
static int mxt_diag_data_start(struct device *dev)
{

	if (diag_data == NULL) {
		diag_data = kzalloc(sizeof(struct mxt_diag_type), GFP_KERNEL);
		if (!diag_data) {
			dev_err(dev, "Failed to allocate memory!\n");
			return -ENOMEM;
		}

		mutex_init(&diag_lock);
		memset(diag_data, 0, sizeof(struct mxt_diag_type));
		mxt_diag_start_flag = 1;
	}
	dev_dbg(dev, "%s is completed.\n", __func__);
	return 0;
}

static int mxt_diag_data_end(struct device *dev)
{
	mxt_diag_start_flag = 0;

	if (diag_data != NULL) {
		mutex_lock(&diag_lock);
		kfree(diag_data);
		mutex_unlock(&diag_lock);

		mutex_destroy(&diag_lock);
	}
	dev_dbg(dev, "%s is completed.\n", __func__);
	return 0;
}
/* H35 Diag 20120403 A E */

static ssize_t mxt_ctrl_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int val;
	int last_address;
	int i;
	u16 offset;
	u16 p;
/* H35 otameshi 20120409 A S */
	u8 *r_data;
	u8 *dp;
/* H35 otameshi 20120409 A E */

	switch (sdata.command) {
	case MXT_SYSFS_LOG_FS:
		count += scnprintf(buf, PAGE_SIZE - count, "log_enabled is "
							"[%d]\n", log_enabled);
		break;
#ifdef ESD_RECOVERY
	case MXT_SYSFS_POLLING:
		count += scnprintf(buf, PAGE_SIZE - count, "esd_recovery is "
							"[%d]\n", esd_recovery);
		break;
#endif /* ESD_RECOVERY */
	case MXT_SYSFS_INT_STATUS:
		val = gpio_get_value(data->pdata->irq_gpio);
		count += scnprintf(buf, PAGE_SIZE - count, "CHG signal is "
							"[%d]\n", val);
		break;

	case MXT_SYSFS_WRITE:
	case MXT_SYSFS_READ:
		object = data->object_table + (data->info.object_num - 1);
		last_address = object->start_address +
				((object->size + 1) *
				 (object->instances + 1)) - 1;
		dev_dbg(dev, "%s:t[%d] last_address is [%04X].\n",
					__func__, object->type, last_address);
		if (sdata.start_addr > last_address) {
			dev_err(dev, "%s:Invalid start address[%04X]!\n",
						__func__, sdata.start_addr);
		}
		if (!sdata.size) {
			sdata.size = last_address - sdata.start_addr + 1;
			dev_dbg(dev, "%s:size is set to[%d].\n",
							__func__, sdata.size);
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "        ");
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		for (i = 0; i < 16; i++) {
			count += scnprintf(buf + count, PAGE_SIZE - count,
								"%2x ", i);
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
		}
		offset = sdata.start_addr & 0xfff0;
		p = sdata.start_addr;
		for (i = 0; i < (sdata.size + (sdata.start_addr & 0xf)) ; i++) {
			if (!(i  & 0xf)) {
				count += scnprintf(buf + count,
						   PAGE_SIZE - count,
						   "\n%04x :\t",
						   i + offset);
				if (count >= PAGE_SIZE)
					return PAGE_SIZE - 1;
			}
			if (i < (sdata.start_addr & 0xf)) {
				count += scnprintf(buf + count,
						   PAGE_SIZE - count,
						   "   ");
				if (count >= PAGE_SIZE)
					return PAGE_SIZE - 1;
			} else {
				mxt_read_reg(data->client, p++, (u8 *)&val);
				count += scnprintf(buf + count,
						   PAGE_SIZE - count,
						   "%02x ", (u8)val);
				if (count >= PAGE_SIZE)
					return PAGE_SIZE - 1;
			}
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;
		break;
/* H35 otameshi 20120409 A S */
	case MXT_SYSFS_CONTIGUOUS_READ:
		object = data->object_table + (data->info.object_num - 1);
		last_address = object->start_address +
				((object->size + 1) *
				 (object->instances + 1)) - 1;
		dev_dbg(dev, "%s:t[%d] last_address is [%04X].\n",
					__func__, object->type, last_address);
		if (sdata.start_addr > last_address) {
			dev_err(dev, "%s:Invalid start address[%04X]!\n",
						__func__, sdata.start_addr);
		}
		if (!sdata.size) {
			sdata.size = last_address - sdata.start_addr + 1;
			dev_dbg(dev, "%s:size is set to[%d].\n",
							__func__, sdata.size);
		}
		if (sdata.size > 127) {
			sdata.size = 127;
		}
		r_data = kcalloc(sdata.size, sizeof(u8), GFP_KERNEL);
		__mxt_read_reg(data->client, sdata.start_addr, sdata.size, r_data);

		count += scnprintf(buf + count, PAGE_SIZE - count, "        ");
		if (count >= PAGE_SIZE) {
			kfree(r_data);
			return PAGE_SIZE - 1;
		}
		for (i = 0; i < 16; i++) {
			count += scnprintf(buf + count, PAGE_SIZE - count,
								"%2x ", i);
			if (count >= PAGE_SIZE) {
				kfree(r_data);
				return PAGE_SIZE - 1;
			}
		}
		offset = sdata.start_addr & 0xfff0;
		dp = r_data;
		for (i = 0; i < (sdata.size + (sdata.start_addr & 0xf)) ; i++) {
			if (!(i  & 0xf)) {
				count += scnprintf(buf + count,
						   PAGE_SIZE - count,
						   "\n%04x :\t",
						   i + offset);
				if (count >= PAGE_SIZE) {
					kfree(r_data);
					return PAGE_SIZE - 1;
				}
			}
			if (i < (sdata.start_addr & 0xf)) {
				count += scnprintf(buf + count,
						   PAGE_SIZE - count,
						   "   ");
				if (count >= PAGE_SIZE) {
					kfree(r_data);
					return PAGE_SIZE - 1;
				}
			} else {
				count += scnprintf(buf + count,
						   PAGE_SIZE - count,
						   "%02x ", dp[i]);
				if (count >= PAGE_SIZE) {
					kfree(r_data);
					return PAGE_SIZE - 1;
				}
			}
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

		if (count >= PAGE_SIZE) {
			kfree(r_data);
			return PAGE_SIZE - 1;
		}
		kfree(r_data);
		break;
/* H35 otameshi 20120409 A E */
	default:
		break;
	}

	return count;
}
/* H35 SYSFS 20120329 A E */

/* H35 CDEV 20120329 A S */
static int mxt_ts_open(struct inode *inode, struct file *file)
{
	struct mxt_data *data =
		container_of(inode->i_cdev, struct mxt_data, device_cdev);
	dev_dbg(&data->client->dev, "%s() is called.\n", __func__);

	file->private_data = data;
	return 0;
};

static int mxt_ts_release(struct inode *inode, struct file *file)
{
	return 0;
};

static long mxt_ts_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	struct mxt_data *data = (struct mxt_data *)file->private_data;
	struct device *dev = &data->client->dev;
	long err = 0;
	int flag;

	dev_dbg(dev, "%s() is called.\n", __func__);
	switch (cmd) {
/* H35 CONF 20120329 A S */
	case IOCTL_SET_CONF_STAT:
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}

		if (copy_from_user(&data->config_status,
				   (void __user *)arg,
				   sizeof(data->config_status))) {
			err = -EFAULT;
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			goto done;
		}

		dev_dbg(dev, "mxt_write_log(data, MXT_LOG_CONF_STAT, NULL)\n");
		mxt_write_log(data, MXT_LOG_CONF_STAT, NULL);
		break;
	case IOCTL_GET_CONF_STAT:
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}

		if (copy_to_user((void __user *)arg,
				 &data->config_status,
				 sizeof(data->config_status))) {
			err = -EFAULT;
			dev_err(dev, "%s: copy_to_user error\n", __func__);
			goto done;
		}
		dev_dbg(dev, "Get config_status is [%d]\n", data->config_status);
		break;
/* H35 20120316 for debug Fujiki A S */
	case IOCTL_SET_LOG:
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}

		if (copy_from_user(&flag, (void __user *)arg, sizeof(flag))) {
			err = -EFAULT;
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			goto done;
		}

		dev_dbg(dev, "mxt_write_log(data, MXT_LOG_DEAMON, &flag)");
		mxt_write_log(data, MXT_LOG_DEAMON, &flag);
		break;
/* H35 20120316 for debug Fujiki A E */
/* H35 CONF 20120329 A E */
/* H35 Diag 20120403 A S */
	case IOCTL_DIAG_START:
		err = mxt_diag_data_start(dev);
		break;

	case IOCTL_MULTI_GET:
	case IOCTL_COODINATE_GET:
 		if (diag_data != NULL) {
			if (!access_ok(VERIFY_WRITE, (void __user *)arg,
							_IOC_SIZE(cmd))) {
				err = -EFAULT;
				dev_err(dev, "%s: invalid access\n", __func__);
				goto done;
			}
			err = copy_to_user((void __user *)arg, &diag_data,
							sizeof(diag_data));
		} else
			dev_dbg(dev, "Touchscreen Diag inactive!\n");

		if (err) {
			dev_err(dev, "%s: copy_to_user error\n", __func__);
			return -EFAULT;
		}
		break;

	case IOCTL_DIAG_END:
		err = mxt_diag_data_end(dev);
		break;

	case IOCTL_DIAG_EVENT_CTRL:
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		err = copy_from_user(&mxt_event_control, (void __user *)arg,
						sizeof(unsigned char));
		if (err){
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			return -EFAULT;
		}
		dev_dbg(dev, "%s: mxt_event_control is [%d]\n", __func__
							, mxt_event_control);
		break;
/* H35 Diag 20120403 A E */
/* H35 NV 20120404 A S */
	case IOCTL_LOAD_CHRG_C_NV:
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mxt_get_nv(dev, arg, MXT_CHARGE_C_NV);
		break;
	case IOCTL_LOAD_CHRG_A_NV:
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mxt_get_nv(dev, arg, MXT_CHARGE_A_NV);
		break;
	case IOCTL_LOAD_N_CHRG_NV:
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mxt_get_nv(dev, arg, MXT_N_CHARGE_NV);
		break;
	case IOCTL_SET_NV:
		printk(KERN_ERR "%s: IOCTL_SET_NV\n", __func__);
		break;
/* H35 NV 20120404 A E */
	default:
		printk(KERN_ERR "%s: ioctl, cmd error\n", __func__);
		return -EINVAL;
		break;
	}
done:
	return err;
}

static const struct file_operations mxt_ts_fops = {
	.owner = THIS_MODULE,
	.open = mxt_ts_open,
	.unlocked_ioctl = mxt_ts_ioctl,
	.release = mxt_ts_release,
};
/* H35 CDEV 20120329 A E */

/* H35 SYSFS 20120329 M S */
#if 0
static DEVICE_ATTR(object, 0444, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, 0664, NULL, mxt_update_fw_store);
#else /* 0 */
static DEVICE_ATTR(object, S_IRUGO|S_IWUSR, mxt_object_show, mxt_object_store);
static DEVICE_ATTR(update_fw, S_IRUGO|S_IWUSR, NULL, mxt_update_fw_store);
#endif /* 0 */
/* H35 SYSFS 20120329 M E */
/* H35 SYSFS 20120329 A S */
static DEVICE_ATTR(ctrl, S_IRUGO|S_IWUSR, mxt_ctrl_show, mxt_ctrl_store);
/* H35 SYSFS 20120329 A E */

static struct attribute *mxt_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
/* H35 SYSFS 20120329 A S */
	&dev_attr_ctrl.attr,
/* H35 SYSFS 20120329 A E */
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static int mxt_start(struct mxt_data *data)
{
	int error;

	/* restore the old power state values and reenable touch */
	error = __mxt_write_reg(data->client, data->t7_start_addr,
/* H35 WS1 20120328 M S */
#if 0
				T7_DATA_SIZE, data->t7_data);
#else /* 0 */
				T7_BACKUP_SIZE, data->t7_data);
#endif /* 0 */
/* H35 WS1 20120328 M E */
	if (error < 0) {
		dev_err(&data->client->dev,
			"failed to restore old power state\n");
		return error;
	}

/* H35 WS1 20120328 D S */
#if 0
	error = mxt_write_object(data,
			MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, data->t9_ctrl);
	if (error < 0) {
		dev_err(&data->client->dev, "failed to restore touch\n");
		return error;
	}
#endif /* 0 */
/* H35 WS1 20120328 D E */

	return 0;
}

static int mxt_stop(struct mxt_data *data)
{
	int error;
/* H35 WS1 20120328 M S */
#if 0
	u8 t7_data[T7_DATA_SIZE] = {0};
#else /* 0 */
	u8 t7_data[T7_BACKUP_SIZE] = {0};
#endif /* 0 */
/* H35 WS1 20120328 M E */

/* H35 WS1 20120328 D S */
#if 0
	/* disable touch and configure deep sleep mode */
	error = mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0);
	if (error < 0) {
		dev_err(&data->client->dev, "failed to disable touch\n");
		return error;
	}
#endif /* 0 */
/* H35 WS1 20120328 D E */

	error = __mxt_write_reg(data->client, data->t7_start_addr,
/* H35 WS1 20120328 M S */
#if 0
				T7_DATA_SIZE, t7_data);
#else /* 0 */
				T7_BACKUP_SIZE, t7_data);
#endif /* 0 */
/* H35 WS1 20120328 M E */
	if (error < 0) {
		dev_err(&data->client->dev,
			"failed to configure deep sleep mode\n");
		return error;
	}
/* H35 WS1 20120328 A S */
	/* Read dummy to avoid inconsistent */
	mxt_make_highchg(data);
/* H35 WS1 20120328 A E */

	return 0;
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int error;

	error = mxt_start(data);
	if (error < 0) {
		dev_err(&data->client->dev, "mxt_start failed in input_open\n");
		return error;
	}

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int error;

	error = mxt_stop(data);
	if (error < 0)
		dev_err(&data->client->dev, "mxt_stop failed in input_close\n");

}

/* H35 WS1 20120328 D S */
#if 0
static int mxt_power_on(struct mxt_data *data, bool on)
{
	int rc;

	if (on == false)
		goto power_off;

	rc = regulator_set_optimum_mode(data->vcc_ana, MXT_ACTIVE_LOAD_UA);
	if (rc < 0) {
		dev_err(&data->client->dev,
			"Regulator vcc_ana set_opt failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_ana);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_ana enable failed rc=%d\n", rc);
		goto error_reg_en_vcc_ana;
	}

	if (data->pdata->digital_pwr_regulator) {
		rc = regulator_set_optimum_mode(data->vcc_dig,
						MXT_ACTIVE_LOAD_DIG_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator vcc_dig set_opt failed rc=%d\n",
				rc);
			goto error_reg_opt_vcc_dig;
		}

		rc = regulator_enable(data->vcc_dig);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vcc_dig enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_dig;
		}
	}

	if (data->pdata->i2c_pull_up) {
		rc = regulator_set_optimum_mode(data->vcc_i2c, MXT_I2C_LOAD_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto error_reg_opt_i2c;
		}

		rc = regulator_enable(data->vcc_i2c);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_i2c;
		}
	}

	msleep(130);

	return 0;

error_reg_en_vcc_i2c:
	if (data->pdata->i2c_pull_up)
		regulator_set_optimum_mode(data->vcc_i2c, 0);
error_reg_opt_i2c:
	if (data->pdata->digital_pwr_regulator)
		regulator_disable(data->vcc_dig);
error_reg_en_vcc_dig:
	if (data->pdata->digital_pwr_regulator)
		regulator_set_optimum_mode(data->vcc_dig, 0);
error_reg_opt_vcc_dig:
	regulator_disable(data->vcc_ana);
error_reg_en_vcc_ana:
	regulator_set_optimum_mode(data->vcc_ana, 0);
	return rc;

power_off:
	regulator_set_optimum_mode(data->vcc_ana, 0);
	regulator_disable(data->vcc_ana);
	if (data->pdata->digital_pwr_regulator) {
		regulator_set_optimum_mode(data->vcc_dig, 0);
		regulator_disable(data->vcc_dig);
	}
	if (data->pdata->i2c_pull_up) {
		regulator_set_optimum_mode(data->vcc_i2c, 0);
		regulator_disable(data->vcc_i2c);
	}
	msleep(50);
	return 0;
}

static int mxt_regulator_configure(struct mxt_data *data, bool on)
{
	int rc;

	if (on == false)
		goto hw_shutdown;

	data->vcc_ana = regulator_get(&data->client->dev, "vdd_ana");
	if (IS_ERR(data->vcc_ana)) {
		rc = PTR_ERR(data->vcc_ana);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vcc_ana) > 0) {
		rc = regulator_set_voltage(data->vcc_ana, MXT_VTG_MIN_UV,
							MXT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
	if (data->pdata->digital_pwr_regulator) {
		data->vcc_dig = regulator_get(&data->client->dev, "vdd_dig");
		if (IS_ERR(data->vcc_dig)) {
			rc = PTR_ERR(data->vcc_dig);
			dev_err(&data->client->dev,
				"Regulator get dig failed rc=%d\n", rc);
			goto error_get_vtg_vcc_dig;
		}

		if (regulator_count_voltages(data->vcc_dig) > 0) {
			rc = regulator_set_voltage(data->vcc_dig,
				MXT_VTG_DIG_MIN_UV, MXT_VTG_DIG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_vcc_dig;
			}
		}
	}
	if (data->pdata->i2c_pull_up) {
		data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
		if (IS_ERR(data->vcc_i2c)) {
			rc = PTR_ERR(data->vcc_i2c);
			dev_err(&data->client->dev,
				"Regulator get failed rc=%d\n",	rc);
			goto error_get_vtg_i2c;
		}
		if (regulator_count_voltages(data->vcc_i2c) > 0) {
			rc = regulator_set_voltage(data->vcc_i2c,
				MXT_I2C_VTG_MIN_UV, MXT_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_i2c;
			}
		}
	}

	return 0;

error_set_vtg_i2c:
	regulator_put(data->vcc_i2c);
error_get_vtg_i2c:
	if (data->pdata->digital_pwr_regulator)
		if (regulator_count_voltages(data->vcc_dig) > 0)
			regulator_set_voltage(data->vcc_dig, 0,
				MXT_VTG_DIG_MAX_UV);
error_set_vtg_vcc_dig:
	if (data->pdata->digital_pwr_regulator)
		regulator_put(data->vcc_dig);
error_get_vtg_vcc_dig:
	if (regulator_count_voltages(data->vcc_ana) > 0)
		regulator_set_voltage(data->vcc_ana, 0, MXT_VTG_MAX_UV);
error_set_vtg_vcc_ana:
	regulator_put(data->vcc_ana);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(data->vcc_ana) > 0)
		regulator_set_voltage(data->vcc_ana, 0, MXT_VTG_MAX_UV);
	regulator_put(data->vcc_ana);
	if (data->pdata->digital_pwr_regulator) {
		if (regulator_count_voltages(data->vcc_dig) > 0)
			regulator_set_voltage(data->vcc_dig, 0,
						MXT_VTG_DIG_MAX_UV);
		regulator_put(data->vcc_dig);
	}
	if (data->pdata->i2c_pull_up) {
		if (regulator_count_voltages(data->vcc_i2c) > 0)
			regulator_set_voltage(data->vcc_i2c, 0,
						MXT_I2C_VTG_MAX_UV);
		regulator_put(data->vcc_i2c);
	}
	return 0;
}
#endif /* 0 */
/* H35 WS1 20120328 D E */

#ifdef CONFIG_PM
/* H35 WS1 20120328 D S */
#if 0
static int mxt_regulator_lpm(struct mxt_data *data, bool on)
{

	int rc;

	if (on == false)
		goto regulator_hpm;

	rc = regulator_set_optimum_mode(data->vcc_ana, MXT_LPM_LOAD_UA);
	if (rc < 0) {
		dev_err(&data->client->dev,
			"Regulator vcc_ana set_opt failed rc=%d\n", rc);
		goto fail_regulator_lpm;
	}

	if (data->pdata->digital_pwr_regulator) {
		rc = regulator_set_optimum_mode(data->vcc_dig,
						MXT_LPM_LOAD_DIG_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator vcc_dig set_opt failed rc=%d\n", rc);
			goto fail_regulator_lpm;
		}
	}

	if (data->pdata->i2c_pull_up) {
		rc = regulator_set_optimum_mode(data->vcc_i2c,
						MXT_I2C_LPM_LOAD_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto fail_regulator_lpm;
		}
	}

	return 0;

regulator_hpm:

	rc = regulator_set_optimum_mode(data->vcc_ana, MXT_ACTIVE_LOAD_UA);
	if (rc < 0) {
		dev_err(&data->client->dev,
			"Regulator vcc_ana set_opt failed rc=%d\n", rc);
		goto fail_regulator_hpm;
	}

	if (data->pdata->digital_pwr_regulator) {
		rc = regulator_set_optimum_mode(data->vcc_dig,
						 MXT_ACTIVE_LOAD_DIG_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator vcc_dig set_opt failed rc=%d\n", rc);
			goto fail_regulator_hpm;
		}
	}

	if (data->pdata->i2c_pull_up) {
		rc = regulator_set_optimum_mode(data->vcc_i2c, MXT_I2C_LOAD_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto fail_regulator_hpm;
		}
	}

	return 0;

fail_regulator_lpm:
	regulator_set_optimum_mode(data->vcc_ana, MXT_ACTIVE_LOAD_UA);
	if (data->pdata->digital_pwr_regulator)
		regulator_set_optimum_mode(data->vcc_dig,
						MXT_ACTIVE_LOAD_DIG_UA);
	if (data->pdata->i2c_pull_up)
		regulator_set_optimum_mode(data->vcc_i2c, MXT_I2C_LOAD_UA);

	return rc;

fail_regulator_hpm:
	regulator_set_optimum_mode(data->vcc_ana, MXT_LPM_LOAD_UA);
	if (data->pdata->digital_pwr_regulator)
		regulator_set_optimum_mode(data->vcc_dig, MXT_LPM_LOAD_DIG_UA);
	if (data->pdata->i2c_pull_up)
		regulator_set_optimum_mode(data->vcc_i2c, MXT_I2C_LPM_LOAD_UA);

	return rc;
}
#endif /* 0 */
/* H35 WS1 20120328 A E */

static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
/* H35 WS1 20120328 M S */
#if 0
	int error;
#else /* 0 */
	int error = 0;
#endif /* 0 */

/* H35 ESD 20120328 M S */
#ifdef ESD_RECOVERY
	mutex_lock(&data->lock);

	cancel_delayed_work_sync(&data->esdwork); /* no "new ones" */
	flush_workqueue(mxt_wq); /* wait till all "old ones" finished */
#else /* ESD_RECOVERY */
	mutex_lock(&input_dev->mutex);
#endif /* ESD_RECOVERY */
/* H35 ESD 20120328 M E */

	if (input_dev->users) {
		if (data->is_suspended)
			goto done;
		error = mxt_stop(data);
		if (error < 0) {
			dev_err(dev, "mxt_stop failed in suspend\n");
#if 0
			mutex_unlock(&input_dev->mutex);
			return error;
#else /* 0 */
			goto done;
#endif /* 0 */
		}
		data->is_suspended = true;
		mxt_input_report_clear(data);
		dev_dbg(dev, "%s done.\n", __func__);
	}
#if 0
	mutex_unlock(&input_dev->mutex);

	/* put regulators in low power mode */
	error = mxt_regulator_lpm(data, true);
	if (error < 0) {
		dev_err(dev, "failed to enter low power mode\n");
		return error;
	}

	return 0;
#else /* 0 */
done:
/* H35 ESD 20120328 M S */
#ifdef ESD_RECOVERY
	mutex_unlock(&data->lock);
#else /* ESD_RECOVERY */
	mutex_unlock(&input_dev->mutex);
#endif /* ESD_RECOVERY */
/* H35 ESD 20120328 M E */
	return error;
#endif /* 0 */
/* H35 WS1 20120328 M E */
}

/* H35 WS1 20120328 A S */
static int mxt_calibrate(struct mxt_data *data)
{
	int ret = 0;

	ret = mxt_write_object(data, MXT_GEN_COMMAND_T6,
				MXT_COMMAND_CALIBRATE, MXT_CALIBRATE_ORDER);
	return ret;
}
/* H35 WS1 20120328 A E */

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
/* H35 WS1 20120328 M S */
#if 0
	int error;
#else /* 0 */
	int error = 0;
#endif /* 0 */
#if 0
	/* put regulators in high power mode */
	error = mxt_regulator_lpm(data, false);
	if (error < 0) {
		dev_err(dev, "failed to enter high power mode\n");
		return error;
	}
#endif /* 0 */

/* H35 ESD 20120328 M S */
#ifdef ESD_RECOVERY
	mutex_lock(&data->lock);
#else /* ESD_RECOVERY */
	mutex_lock(&input_dev->mutex);
#endif /* ESD_RECOVERY */
/* H35 ESD 20120328 M E */

	if (input_dev->users) {
		if (!data->is_suspended)
			goto done;
		error = mxt_start(data);
		if (error < 0) {
			dev_err(dev, "mxt_start failed in resume\n");
#if 0
			mutex_unlock(&input_dev->mutex);
			return error;
#else /* 0 */
			goto done;
#endif /* 0 */
		}
		data->is_suspended = false;
		error = mxt_calibrate(data);
		if (error < 0)
			dev_err(dev, "mxt_calibrate failed in resume\n");
		else
			dev_dbg(dev, "%s: mxt_calibrate done.\n", __func__);

		dev_dbg(dev, "%s done.\n", __func__);
	}
done:
#ifdef ESD_RECOVERY
	mutex_unlock(&data->lock);
	queue_delayed_work(mxt_wq, &data->esdwork,
			   msecs_to_jiffies(ESD_POLLING_TIME));
#else /* ESD_RECOVERY */
	mutex_unlock(&input_dev->mutex);
#endif /* ESD_RECOVERY */
#if 0
	return 0;
#else /* 0 */
	return error;
#endif /* 0 */
/* H35 WS1 20120328 M E */
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);

	mxt_suspend(&data->client->dev);
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);

	mxt_resume(&data->client->dev);
}
#endif

static const struct dev_pm_ops mxt_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,
#endif
};
#endif

/* H35 WS1 20120328 A S */
#ifdef USE_WORKQUEUE
static void mxt_work_func(struct work_struct *work)
{
	struct mxt_data *data = container_of(work, struct mxt_data, work);
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;

	do {
		if (mxt_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;

		/* whether reportid is thing of MXT_TOUCH_MULTI */
		object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
		if (!object)
			goto end;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		id = reportid - min_reportid;

		if (reportid >= min_reportid && reportid <= max_reportid)
			mxt_input_touchevent(data, &message, id);
		else
			mxt_dump_message(dev, &message);
	} while (reportid != 0xff);

end:
	enable_irq(data->client->irq);
/* H35 20120228 forDebug Fujiki A S */
	dev_dbg(dev, "CPU[%d]%s() is finished.\n"
						, smp_processor_id()
						, __func__);
/* H35 20120228 forDebug Fujiki A E */
}

static irqreturn_t mxt_irq(int irq, void *dev_id)
{
    struct mxt_data *data = dev_id;

    dev_dbg(&data->client->dev, "%s() is called.", __func__);

    disable_irq_nosync(data->client->irq);
    queue_work(mxt_wq, &data->work);

    return IRQ_HANDLED;
}
#endif /* USE_WORKQUEUE */
/* H35 WS1 20120328 A E */

/* H35 ESD 20120328 A S */
#ifdef ESD_RECOVERY
static void mxt_esd_recovery_work(struct work_struct *work)
{
	struct mxt_data *data = container_of(work, struct mxt_data, esdwork.work);
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	u8 val;

/* 20120309 deadlock test s */
	dev_dbg(&client->dev, "CPU[%d]%s() is called.\n"
				, smp_processor_id() , __func__);
/* 20120309 deadlock test e */
	if ((esd_recovery != 0) && mutex_trylock(&data->lock)) {
/* 20120309 deadlock test s */
		dev_dbg(&client->dev, "%s:lock mutex.\n", __func__);
/* 20120309 deadlock test e */
		/* Check Family ID */
		mxt_read_reg(client, MXT_FAMILY_ID, &val);
		if (val != info->family_id) {
			/*
			 * Can't read family id correctly.
			 * Recovery process start.
			 */
			dev_err(&client->dev, "%s: Recovery start! "
				"Uncorrect family id [%02X]\n", __func__, val);
			disable_irq(data->irq);
			if (mxt_restart(data))
				dev_err(&client->dev, "Failed to restart!\n");
			enable_irq(data->irq);
			/* Read dummy to avoid inconsistent */
			mxt_make_highchg(data);
		} else
			dev_dbg(&client->dev, "%s: Corrct family id [%02X]\n"
							, __func__, val);
/* 20120309 deadlock test s */
		dev_dbg(&client->dev, "%s:unlock mutex.\n", __func__);
/* 20120309 deadlock test e */
		mutex_unlock(&data->lock);
	}
	queue_delayed_work(mxt_wq, &data->esdwork,
					msecs_to_jiffies(ESD_POLLING_TIME));
/* 20120309 deadlock test s */
		dev_dbg(&client->dev, "%s() is completed.\n", __func__);
/* 20120309 deadlock test e */
}
#endif /* ESD_RECOVERY */
/* H35 ESD 20120328 A E */

static int __devinit mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
	struct input_dev *input_dev;
	int error;
/* H35 CDEV 20120329 A S */
	dev_t device_t = MKDEV(0, 0);
	struct device *class_dev_t = NULL;
/* H35 CDEV 20120329 A E */

	if (!pdata)
		return -EINVAL;

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "Atmel maXTouch Touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	data->client = client;
	data->input_dev = input_dev;
	data->pdata = pdata;
	data->irq = client->irq;
/* H35 WS1 20120328 A S */
	mutex_init(&data->lock);

	mxt_calc_resolution(data);
/* H35 WS1 20120328 A E */

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
/* H35 WS1 20120328 A S */
	__set_bit(EV_SYN, input_dev->evbit);
/* H35 WS1 20120328 A E */

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->pdata->x_size, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->pdata->y_size, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);

	/* For multi touch */
/* H35 WS1 20120328 A S */
#ifndef MT_PROTOCOL_TYPE_A
	input_mt_init_slots(input_dev, MXT_MAX_FINGER);
#endif /* MT_PROTOCOL_TYPE_A */
/* H35 WS1 20120328 A E */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
/* H35 WS1 20120328 M S */
/* 			     0, data->pdata->x_size, 0, 0); */
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
/* 			     0, data->pdata->y_size, 0, 0); */
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	if (pdata->init_hw)
#if 0
		error = pdata->init_hw(true);
#else /* 0 */
		error = pdata->init_hw();
	else
		error = 0;
#endif /* 0 */
/* H35 WS1 20120328 M E */
/* H35 WS1 20120328 D S */
#if 0
	else
		error = mxt_regulator_configure(data, true);
#endif /* 0 */
/* H35 WS1 20120328 D E */
	if (error) {
		dev_err(&client->dev, "Failed to intialize hardware\n");
/* H35 forFact(temporary) 20120328 M S */
#ifdef FEATURE_TEMP_FACT
		goto err_dev_access;
#else /* FEATURE_TEMP_FACT */
		goto err_free_mem;
#endif /* FEATURE_TEMP_FACT */
/* H35 forFact(temporary) 20120328 M E */
	}

/* H35 WS1 20120328 D S */
#if 0
	if (pdata->power_on)
		error = pdata->power_on(true);
	else
		error = mxt_power_on(data, true);
	if (error) {
		dev_err(&client->dev, "Failed to power on hardware\n");
		goto err_regulator_on;
	}
#endif /* 0 */
/* H35 WS1 20120328 D E */
/* H35 WS1 20120328 A S */
	error = mxt_reset_and_delay(data);
	if (error)
/* H35 forFact(temporary) 20120328 M S */
#ifdef FEATURE_TEMP_FACT
		goto err_dev_access;
#else /* FEATURE_TEMP_FACT */
		goto err_free_mem;
#endif /* FEATURE_TEMP_FACT */
/* H35 forFact(temporary) 20120328 M E */
/* H35 WS1 20120328 A E */

/* H35 WS1 20120328 D S */
#if 0
	if (gpio_is_valid(pdata->irq_gpio)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(pdata->irq_gpio,
							"mxt_irq_gpio");
		if (error) {
			pr_err("%s: unable to request gpio [%d]\n", __func__,
						pdata->irq_gpio);
			goto err_power_on;
		}
		error = gpio_direction_input(pdata->irq_gpio);
		if (error) {
			pr_err("%s: unable to set_direction for gpio [%d]\n",
					__func__, pdata->irq_gpio);
			goto err_irq_gpio_req;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		/* configure touchscreen reset out gpio */
		error = gpio_request(pdata->reset_gpio,
						"mxt_reset_gpio");
		if (error) {
			pr_err("%s: unable to request reset gpio %d\n",
				__func__, pdata->reset_gpio);
			goto err_irq_gpio_req;
		}

		error = gpio_direction_output(
					pdata->reset_gpio, 1);
		if (error) {
			pr_err("%s: unable to set direction for gpio %d\n",
				__func__, pdata->reset_gpio);
			goto err_reset_gpio_req;
		}
	}

	mxt_reset_delay(data);
#endif
/* H35 WS1 20120328 D E */

	error = mxt_initialize(data);
	if (error)
/* H35 WS1 20120328 M S */
#if 0
		goto err_reset_gpio_req;
#else /* 0 */
/* H35 forFact(temporary) 20120328 M S */
#ifdef FEATURE_TEMP_FACT
		goto err_dev_access;
#else /* FEATURE_TEMP_FACT */
		goto err_free_mem;
#endif /* FEATURE_TEMP_FACT */
/* H35 forFact(temporary) 20120328 M E */
#endif /* 0 */
/* H35 WS1 20120328 M E */

/* H35 WS1 20120328 A S */
	data->is_suspended = false;
/* H35 WS1 20120328 A E */

/* H35 WS1 20120328 M S */
#ifdef USE_WORKQUEUE
	INIT_WORK(&data->work, mxt_work_func);
	error = request_irq(client->irq
			, mxt_irq
			, pdata->irqflags
			, client->dev.driver->name
			, data);
#else /* USE_WORKQUEUE */
/* H35 20120315 forDebug Fujiki A S */
	dev_dbg(&client->dev, "%s:request_threaded_irq() is called.\n", __func__);
/* H35 20120315 forDebug Fujiki A E */
	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
			pdata->irqflags, client->dev.driver->name, data);
#endif /* USE_WORKQUEUE */
/* H35 WS1 20120328 M E */
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}
/* H35 ESD 20120328 A S */
#ifdef ESD_RECOVERY
	INIT_DELAYED_WORK(&data->esdwork, mxt_esd_recovery_work);
	/* Check start after minimum 5 secs */
/* H35 20120315 forDebug Fujiki A S */
	dev_dbg(&client->dev, "%s:queue_delayed_work() is called.\n", __func__);
/* H35 20120315 forDebug Fujiki A E */
	queue_delayed_work(mxt_wq, &data->esdwork,
			   msecs_to_jiffies(ESD_POLLING_TIME + 5000));
#endif /* ESD_RECOVERY */
/* H35 ESD 20120328 A E */

/* H35 WS1 20120328 A S */
	mutex_lock(&data->lock);
/* H35 20120315 forDebug Fujiki A S */
	dev_dbg(&client->dev, "%s:mxt_make_highchg() is called.\n", __func__);
/* H35 20120315 forDebug Fujiki A E */
/* H35 WS1 20120328 A E */
	error = mxt_make_highchg(data);
/* H35 WS1 20120328 A S */
/* H35 20120315 forDebug Fujiki A S */
	dev_dbg(&client->dev, "%s:mxt_make_highchg() is end[%d].\n", __func__, error);
/* H35 20120315 forDebug Fujiki A E */
	mutex_unlock(&data->lock);
/* H35 WS1 20120328 A E */
	if (error)
/* H35 forFact(temporary) 20120328 M S */
#ifdef FEATURE_TEMP_FACT
		goto err_dev_free_irq;
#else /* FEATURE_TEMP_FACT */
		goto err_free_irq;
#endif /* FEATURE_TEMP_FACT */
/* H35 forFact(temporary) 20120328 M E */

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error)
		goto err_unregister_device;

/* H35 CDEV 20120329 A S */
	error = alloc_chrdev_region(&device_t, 0, 1, "mxt_ts");
	if (error)
		goto err_remove_sysfs;

	data->device_major = MAJOR(device_t);

	cdev_init(&(data->device_cdev), &mxt_ts_fops);
	data->device_cdev.owner = THIS_MODULE;
	data->device_cdev.ops = &mxt_ts_fops;
	error = cdev_add(&(data->device_cdev), MKDEV(data->device_major, 0), 1);
	if (error)
		goto err_unregister_chrdev;

	data->device_class = class_create(THIS_MODULE, "mxt_ts");
	if (IS_ERR(data->device_class)) {
		error = -1;
		goto err_cleanup_cdev;
	};

	class_dev_t = device_create(data->device_class, NULL,
		MKDEV(data->device_major, 0), NULL, "mxt_ts");
	if (IS_ERR(class_dev_t)) {
		error = -1;
		goto err_destroy_class;
	}
/* H35 CDEV 20120329 A E */

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						MXT_SUSPEND_LEVEL;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	return 0;

/* H35 CDEV 20120329 A S */
err_destroy_class:
	class_destroy(data->device_class);
err_cleanup_cdev:
	cdev_del(&(data->device_cdev));
err_unregister_chrdev:
	unregister_chrdev_region(device_t, 1);
err_remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
/* H35 CDEV 20120329 A E */
err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
/* H35 WS1 20120328 D S */
#if 0
err_reset_gpio_req:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
err_irq_gpio_req:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
err_power_on:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		mxt_power_on(data, false);
err_regulator_on:
	if (pdata->init_hw)
		pdata->init_hw(false);
	else
		mxt_regulator_configure(data, false);
#endif
/* H35 WS1 20120328 D E */
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;

/* H35 forFact(temporary) 20120328 A S */
#ifdef FEATURE_TEMP_FACT
err_dev_free_irq:
	free_irq(client->irq, data);
	kfree(data->object_table);
err_dev_access:
	mutex_destroy(&data->lock);
	error = input_register_device(input_dev);
	if (error)
		goto err_free_mem;
	return error;
#endif /* FEATURE_TEMP_FACT */
/* H35 forFact(temporary) 20120328 A E */
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);
/* H35 CDEV 20120329 A S */
	dev_t device_t = MKDEV(data->device_major, 0);
/* H35 CDEV 20120329 A E */

	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
/* H35 CDEV 20120329 A S */
	if (data->device_class) {
		device_destroy(data->device_class,
			MKDEV(data->device_major, 0));
		class_destroy(data->device_class);
	}
	if (&data->device_cdev) {
		cdev_del(&(data->device_cdev));
		unregister_chrdev_region(device_t, 1);
	}
/* H35 CDEV 20120329 A E */

/* H35 WS1 20120328 D S */
#if 0
	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		mxt_power_on(data, false);

	if (data->pdata->init_hw)
		data->pdata->init_hw(false);
	else
		mxt_regulator_configure(data, false);
#endif /* 0 */
/* H35 WS1 20120328 D E */

	kfree(data->object_table);
	kfree(data);

	return 0;
}

static const struct i2c_device_id mxt_id[] = {
/* H35 WS1 20120328 M S */
#if 0
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "mXT224", 0 },
#else /* 0 */
	{ "mXT224S", 0 },
#endif /* 0 */
/* H35 WS1 20120328 M E */
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &mxt_pm_ops,
#endif
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.id_table	= mxt_id,
};

static int __init mxt_init(void)
{
/* H35 WS1 20120328 A S */
/* H35 ESD 20120328 M S */
/* #ifdef USE_WORKQUEUE */
#if defined(USE_WORKQUEUE) || defined(ESD_RECOVERY)
	mxt_wq = alloc_workqueue("mxt_wq", WQ_MEM_RECLAIM, 1);
	if (!mxt_wq)
	{
		return -ENOMEM;
	}
/* #endif */ /* USE_WORKQUEUE */
#endif /* USE_WORKQUEUE || ESD_RECOVERY */
/* H35 ESD 20120328 M E */
/* H35 WS1 20120328 A E */
	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
/* H35 WS1 20120328 A S */
/* H35 ESD 20120328 M S */
/* #ifdef USE_WORKQUEUE */
#if defined(USE_WORKQUEUE) || defined(ESD_RECOVERY)
	if (mxt_wq)
	{
		destroy_workqueue(mxt_wq);
	}
/* #endif */ /* USE_WORKQUEUE */
#endif /* USE_WORKQUEUE || ESD_RECOVERY */
/* H35 ESD 20120328 M E */
/* H35 WS1 20120328 A E */
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");

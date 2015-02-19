/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/*
 *  ml610q793.c - Linux kernel modules for acceleration sensor
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


#define CONFIG_ML610Q793_DEBUG

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <asm/gpio.h>
#include <linux/types.h>
#include <linux/sensors/ml610q793.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/earlysuspend.h>
#include <mach/kc_board.h>
#include <linux/miscdevice.h>
#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
#include <linux/i2c.h>
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
#include <linux/spi/spi.h>
#endif
#include <linux/sensor_power.h>

#ifdef CONFIG_ML610Q793_DEBUG
#define DBG_LV_IO      0x01
#define DBG_LV_INT     0x02
#define DBG_LV_SPI     0x04
#define DBG_LV_INFO    0x10
#define DBG_LV_DATA    0x20
#define DBG_LV_ERROR   0x40
#define DBG_LV_LOW     0x80
#define DBG_LV_ALL     (DBG_LV_ERROR)
static int32_t dbg_level = DBG_LV_ALL;
#define DBG(lv, msg, ...) {                       \
    if( lv & dbg_level ){                         \
        printk(KERN_ERR  msg, ##__VA_ARGS__);     \
    }                                             \
}
#define DBG_PRINT_IO(io,ret) {                                      \
    if( io == 0 ){                                                  \
        DBG(DBG_LV_IO, "[ACC]<%s>:Start\n",__FUNCTION__)            \
    }else{                                                          \
        DBG(DBG_LV_IO, "[ACC]<%s>:End(%x) ret=%x\n",__FUNCTION__ ,io, ret)    \
    }                                                               \
}
#else
#define DBG(lv, msg, ...)
#define DBG_PRINT_IO(io, ret)
#endif

#define ACC_FW_VERSION_NONE          (0x00000000)
#define ACC_FW_VERSION_CYCLE         (0x03000000)
#define ACC_FW_VERSION_RECOVER_1     (0x07000090)
#define ACC_FW_VERSION_RECOVER_2     (0x08000000)
#define ACC_FW_VERSION_FIFOUPDATE    (0x02000000)

#define ACC_FW_VERSION_DATA          (0xF2000015)

#define HOST_VER_PROG_MASK           (0x00000000)

#define CFG                          (0x00u)
#define INTMASK0                     (0x01u)
#define INTMASK1                     (0x02u)
#define STATUS                       (0x08u)
#define INTREQ0                      (0x09u)
#define ERROR0                       (0x0bu)
#define CMD0                         (0x10u)
#define PRM0                         (0x12u)
#define RSLT00                       (0x20u)
#define RSLT0E                       (0x2eu)
#define RSLT14                       (0x34u)
#define RSLT20                       (0x40u)

#define INTREQ_NONE                  (0x0000u)
#define INTREQ_HOST_CMD              (0x0001u)
#define INTREQ_ACC                   (0x0002u)
#define INTREQ_ERROR                 (0x8000u)

#define ID_NONE                      (0x00u)
#define ID_ACC_PEDO_CNT              (0x02u)
#define ID_ACC_PEDO_MOVE             (0x04u)
#define ID_ACC_PEDO_STOP             (0x08u)
#define ID_ACC_PEDO_WALK_RUN         (0x10u)
#define ID_ACC_PEDO_PEDO_TRANS       (0x20u)
#define ID_ACC_MOVE_DETECT           (0x10u)
#define ID_ACC_XY_DETECT             (0x40u)
#define ID_ACC_PEDO_TIMER            (0x40u)

#define HC_MCU_GET_VERSION           (0x0001u)
#define HC_MCU_SOFT_RESET            (0x0002u)
#define HC_MCU_GET_EX_SENSOR         (0x0003u)
#define HC_MCU_SET_PCON              (0x0004u)
#define HC_MCU_GET_PCON              (0x0005u)
#define HC_MCU_GET_INT_DETAIL        (0x0006u)
#define HC_MCU_SET_PORT_OUT          (0x0007u)
#define HC_MCU_GET_PORT_OUT          (0x0008u)
#define HC_MCU_SELF_CHK_FW           (0x000Au)
#define HC_MCU_SENSOR_INIT           (0x000Bu)
#define HC_MCU_I2C_IO                (0x000Cu)
#define HC_MCU_SET_PERI              (0x000Du)
#define HC_MCU_FUP_START             (0x0101u)
#define HC_MCU_FUP_ERASE             (0x0102u)
#define HC_MCU_FUP_WRITE             (0x0103u)
#define HC_MCU_FUP_END               (0x0104u)
#define HC_MCU_FUP_WRITE_FIFO        (0x0105u)
#define HC_DST_GET_TRANS1            (0x1088u)
#define HC_DST_GET_TRANS2            (0x1089u)
#define HC_DST_GET_DAILYS            (0x1080u)
#define HC_DST_SET_DAILYS            (0x1081u)
#define HC_DST_CLR_DAILYS            (0x1082u)
#define HC_DST_GET_INT_DETAIL        (0x1083u)
#define HC_DST_CLR_INT_DETAIL        (0x1084u)
#define HC_DST_GET_PEDO1             (0x1085u)
#define HC_DST_GET_PEDO2             (0x1086u)
#define HC_DST_GET_RUN1              (0x1087u)
#define HC_DST_GET_INTELLI_WIFI      (0x108Au)
#define HC_DST_SET_IWIFI_INFO        (0x108Bu)
#define HC_DST_GET_IWIFI_INFO        (0x108Cu)
#define HC_DST_GET_PEDO3             (0x108Du)
#define HC_DST_GET_PEDO4             (0x108Eu)
#define HC_DST_GET_TRANS3            (0x108Fu)
#define HC_DST_GET_TRANS4            (0x1090u)
#define HC_DST_GET_TRANS5            (0x1091u)
#define HC_DST_GET_RUN2              (0x1092u)
#define HC_ACC_MEASURE               (0x1001u)
#define HC_ACC_SET_AUTO_MEASURE      (0x1005u)
#define HC_ACC_GET_AUTO_MEASURE      (0x1006u)
#define HC_ACC_SET_CALIB             (0x1009u)
#define HC_ACC_GET_CALIB             (0x100au)
#define HC_ACC_SET_MOVE_DETECT       (0x1010u)
#define HC_ACC_SET_PEDO              (0x101bu)
#define HC_ACC_GET_PEDO              (0x101cu)
#define HC_ACC_PEDO_CNT              (0x101fu)
#define HC_ACC_SET_DIST_STOP         (0x1020u)
#define HC_ACC_SET_WALK_RUN          (0x1025u)
#define HC_ACC_SET_TRANS             (0x1028u)
#define HC_ACC_GET_TRANS             (0x1029u)
#define HC_ACC_TRANS_INFO            (0x102cu)
#define HC_ACC_PEDO_CLEAR            (0x1049u)
#define HC_ACC_TRANS_CLEAR           (0x104cu)
#define HC_ACC_SET_TRANS_BYC         (0x1050u)
#define HC_ACC_SET_U2DH              (0x105cu)
#define HC_ACC_GET_U2DH              (0x105du)
#define HC_ACC_SET_CONV_AXIS         (0x105eu)
#define HC_ACC_GET_CONV_AXIS         (0x105fu)
#define HC_ACC_SET_PEDO_TIMER        (0x1056u)
#define HC_PRE_SET_MEASURE           (0x6001u)
#define HC_PRE_GET_MEASURE           (0x6002u)
#define HC_PRE_SET_OFFSET_HEIGHT     (0x6005u)
#define HC_PRE_GET_OFFSET_HEIGHT     (0x6006u)
#define HC_PRE_SET_PARAM_BMP280      (0x6007u)
#define HC_PRE_GET_PARAM_BMP280      (0x6008u)
#define HC_PRE_SET_BASE_VAL          (0x6080u)
#define HC_PRE_GET_BASE_VAL          (0x6081u)
#define HC_PRE_GET_HEIGHT1           (0x6082u)
#define HC_PRE_GET_HEIGHT2           (0x6083u)



#define HC_MUL_MEASURE               (0xf001u)
#define HC_MUL_SET_ANDROID           (0xf00bu)
#define HC_MUL_GET_ANDROID           (0xf00cu)
#define HC_MUL_SET_ANDROID_PERIOD    (0xf00du)
#define HC_MUL_SET_T5400             (0xf013u)
#define HC_MUL_GET_T5400             (0xf014u)

#define MT_AUTO_START                (1u)
#define MT_AUTO_STOP                 (2u)
#define MT_ANDROID_START             (2u)
#define MT_ANDROID_STOP              (3u)

#define RT_NORMAL                    (0u)
#define RT_ANDROID                   (1u)

#define HC_INVALID                   (0u)
#define HC_VALID                     (1u)

#define HC_ACC_PEDO_STABLE           (0u)
#define HC_ACC_PEDO_ADD              (2u)
#define HC_ACC_PEDO_CYCLE            (5u)

#define EXE_HOST_WAIT                (1)
#define EXE_HOST_RES                 (2)
#define EXE_HOST_ERR                 (4)
#define EXE_HOST_ALL                 (EXE_HOST_WAIT|EXE_HOST_RES|EXE_HOST_ERR)
#define EXE_HOST_EX_NO_RECOVER       (16)

#define SSIO_MASK_WRITE              (0x7f)
#define SSIO_MASK_READ               (0x80)

#define FUP_MAX_RAMSIZE              (8192)

#define ERROR_FUP_MAXSIZE            (0x0011u)
#define ERROR_FUP_VERIFY             (0x0012u)
#define ERROR_FUP_CERTIFICATION      (0x0013u)
#define ERROR_FUP_ODDSIZE            (0x0014u)

#define ACC_DRIVER_NAME              "accsns"
#define ACC_GPIO_INT_NAME            "accsns_irq"
#define ACC_GPIO_RESET_NAME          "accsns_reset"
#define ACC_GPIO_TEST0_NAME          "accsns_test0"
#define ML610Q793_ACC_KERNEL_NAME    "ML610Q793"

#define ACCSNS_GPIO_INT              (67)
#define ACCSNS_GPIO_TEST0            (64)
#define ACCSNS_GPIO_RST              (66)

#define ONESEC_MS                    (1000)
#define SETTING_0G                   (0)
#define WEIGHT_1G                    (1000)

#define ACCSNS_RC_OK                 (0)
#define ACCSNS_RC_OK_NACK            (1)
#define ACCSNS_RC_ERR                (-1)
#define ACCSNS_RC_ERR_TIMEOUT        (-2)
#define ACCSNS_RC_ERR_RAMWRITE       (-3)

#define WAITEVENT_TIMEOUT            (3000)

#define DEFAULT_FREQ                 (50)
#define OFFSET_SUMPLE_NUM            (100)
#define OFFSET_AVE_NUM               (1)
#define DEFAULT_WEIGHT               (650)
#define MAX_WEIGHT                  (3000)
#define DEFAULT_PEDOMETER            (76)
#define DEFAULT_VEHITYPE             (2)
#define PEDO_TIMER_COUNT_COEFF       (30)
#define DEFAULT_ACC_DATA_DELAY       (200)
#define DEFAULT_PRE_DATA_DELAY       (45)
#define OFFSET_PRE_SUMPLE_NUM        (200)
#define OFFSET_ACC_HIGH_TH           (300)
#define OFFSET_ACC_LOW_TH           (-300)
#define OFFSET_PRE_TH                (200)

#define    POWER_DISABLE       false
#define    POWER_ENABLE        true

#define    ACTIVE_OFF          0x00
#define    ACTIVE_ON           0x01
#define    ACTIVE_PRE_ON       0x100

#define ACC_SPI_RETRY_NUM    5

#define ACC_WORK_QUEUE_NUM   15

#define GRAVITY_EARTH        9806550
#define ABSMAX_2G            (GRAVITY_EARTH * 2)
#define ABSMIN_2G            (-GRAVITY_EARTH * 2)
#define U2DH_RESOLUTION      1000
#define U2DH_GRAVITY_EARTH   9806550
#define ABSMAX_PA            (110000)
#define ABSMIN_PA            (30000)
#define ABSMAX_CAL_PA        (200000)
#define ABSMIN_CAL_PA        (-200000)
#define ABS_WAKE			(ABS_MISC)
#define ABS_X_DUMMY_VAL      (-1)

#define MEASURE_DATA_SIZE    (18)

#define FW_VERSION_CHK_OK    (0x00)
#define FW_VERSION_CHK_NG    (0xFF)

#define STATUS_READ_RETRY_NUM  75
#define MICON_I2C_ENABLE_RETRY_NUM  5

enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_XYZ_MAX
} AXIS;

enum {
    MODE_0 = 0,
    MODE_1,
    MODE_2,
    MODE_3,
    MODE_4,
    MODE_MAX
}MODE;

enum {
    DETECT_NONE = 0,
    DETECT_STOP,
    DETECT_MOVE,
    DETECT_MAX,
}MOVE_DETECT;

//acc_tmp
enum OPERATION_CODE
{
	KC_SENSOR_COMMON_POWER_ON = 0x00000000,
	KC_SENSOR_COMMON_POWER_OFF,
	KC_SENSOR_COMMON_INIT,
	KC_SENSOR_COMMON_IMITATION_ON,
	KC_SENSOR_COMMON_IMITATION_OFF,
	KC_SENSOR_COMMON_MAX
};

typedef union {
    uint16_t   udata16;
    int16_t    sdata16;
    uint8_t    udata8[2];
    int8_t     sdata8[2];
} Word;

typedef union {
    uint8_t    ub_prm[13];
    int8_t     sb_prm[13];
    uint16_t   uw_prm[6];
    int16_t    sw_prm[6];
    uint32_t   ud_prm[3];
    int32_t    sd_prm[3];
} HCParam;

typedef union {
    uint8_t    ub_res[32];
    int8_t     sb_res[32];
    uint16_t   uw_res[16];
    int16_t    sw_res[16];
    uint32_t   ud_res[8];
    int32_t    sd_res[8];
} HCRes;

typedef struct {
    Word       cmd;
    HCParam    prm;
} HostCmd;

typedef struct {
    HCRes      res;
    Word       err;
} HostCmdRes;

struct acceleration {
    int32_t nX;
    int32_t nY;
    int32_t nZ;
	int32_t outX;
	int32_t outY;
	int32_t outZ;
};

struct pedometer {
    int32_t usStepCnt;
    int32_t usWalkTime;
    int32_t usCal;
    int32_t usBodyFat;
    int32_t usExercise;
    int32_t usMets;
    int32_t usSpeed;
    int32_t usRunStatus;
    int32_t usRunStepCnt;
    int32_t usRunTime;
    int32_t usStExercise;
    int32_t usStCal;
    int32_t usStBodyFat;
    int32_t usSportExercise;
    int32_t usRunCal;
    int32_t usRunExercise;
    int32_t usPS_InitBaseHeight;
    int32_t usPS_ActHeight;
    int32_t usPS_ActHeightMin;
    int32_t usPS_ActHeightAve;
    int32_t usPS_ActHeightMax;
};

struct vehicle {
    int32_t usVehiStatus;
    int32_t usVehiKind;
    int32_t usVehiDetectTime;
    int32_t usVehiRideTime;
    int32_t usVehiRideCal;
    int32_t usVehiBodyFat;
    int32_t usVehiExercise;
    int32_t usVehiMets;
    int32_t usVehiStExercise;
    int32_t usVehiStRideCal;
    int32_t usVehiStBodyFat;
    int32_t usVehiBiExercise;
    int32_t usVehiBiRideCal;
    int32_t usVehiBiBodyFat;
    int32_t usVehiSportExercise;
};

struct pressure {
    int32_t usHeight;
    int32_t usTemp;
    int32_t usPressure;
    int32_t usOutPressure;
    int32_t usAccuracy;
};

struct iwifi{
    int32_t usPedoStatus;
    int32_t usVehiStatus;
};

typedef struct tMovFilterWork {
    int32_t* m_pSamplWork[AXIS_XYZ_MAX];
    int32_t  m_unSum[AXIS_XYZ_MAX];
    int32_t  m_unCnt[AXIS_XYZ_MAX];
    uint8_t  m_ucAveN;
} MovFilterWork;

typedef struct tCalibrationCtrl {
    bool     m_bFilterEnable;
    bool     m_bWaitSetting;
    bool     m_bComplete;
    bool     m_bRengeChkErr;
    uint16_t m_unSmpN;
    MovFilterWork m_tFilterWork;
    int32_t  m_nCalX;
    int32_t  m_nCalY;
    int32_t  m_nCalZ;
    int32_t  m_nCurrentSampleNum;
    int32_t  m_nSummationX;
    int32_t  m_nSummationY;
    int32_t  m_nSummationZ;
    int32_t  m_nMode;
    int32_t  m_nHighTH;
    int32_t  m_nLowTH;
    struct mutex m_tCalibMutex;
} CalibrationCtrl;

typedef struct t_ACC_WorkQueue {
    struct work_struct  work;
    bool                status;
} ACC_WorkQueue;

typedef struct tPreCalibrationCtrl {
    bool     m_bWaitSetting;
    bool     m_bComplete;
	bool     m_bRengeChkErr;
    uint16_t m_unSmpN;
    int32_t  m_nP0;
    int32_t  m_nRefP;
    int32_t  m_nCalP;
    int32_t  m_nCurrentSampleNum;
    int32_t  m_nSummationSample;
    int32_t  m_nSummationPa;
    int32_t  m_nSummationPb;
    int32_t  m_nSummationPa_Sample;
    int32_t  m_nSummationPb_Sample;
    int32_t  m_nSummationSample_squ;
    int32_t  m_nPressureBase;
	int32_t  m_nSmpMaxP;
	int32_t  m_nSmpMinP;
	int32_t  m_nCalTH;
    int32_t  m_nMode;
    struct mutex m_tCalibMutex;
} PreCalibrationCtrl;

static ACC_WorkQueue  s_tAccWork[ACC_WORK_QUEUE_NUM];

struct semaphore s_tAccSnsrSema;
static int32_t g_nAccFWVersion;
static int32_t g_nIntIrqFlg;
static int32_t g_nIntIrqNo;
static int32_t s_nAccWorkCnt;
static wait_queue_head_t s_tWaitInt;
static wait_queue_head_t s_tPollWaitPedom;
static wait_queue_head_t s_tPollWaitVehicle;
static wait_queue_head_t s_tPollWaitIWifi;
static CalibrationCtrl s_tCalibCtrl;
static IoCtlAccSetAccsnsNVParams g_acc_nv_param;
static bool g_bDevIF_Error = false;
static int work_ped_pos = 0;
static bool dummy_on_flg = false;
#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static struct i2c_driver accsns_driver;
static struct i2c_client *client_accsns;
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static struct spi_driver accsns_driver;
static struct spi_device *client_accsns;
#endif
static struct workqueue_struct *accsns_wq_int;
static struct workqueue_struct *accsns_wq;
static struct workqueue_struct *fw_update_wq;
static struct workqueue_struct *accsns_cal_wq;
static struct workqueue_struct *accsns_data_wq;
static struct workqueue_struct *presns_cal_wq;
static struct workqueue_struct *presns_data_wq;
static struct delayed_work s_tDelayWork_Acc;
static struct mutex s_tDataMutex;
static struct acceleration s_tLatestAccData;
static struct pedometer    s_tLatestPedoData;
static struct vehicle      s_tLatestVehiData;
static struct pressure     s_tLatestPresData;
static struct iwifi        s_tLatestIWifiData;
static DEFINE_SPINLOCK(acc_lock);
static struct mutex s_tEnableMutex;
static struct input_dev *acc_idev;
static struct input_dev *pre_idev;
static struct delayed_work s_tDelayWork_Acc_Data;
static struct delayed_work s_tDelayWork_Pre_Data;
static struct delayed_work s_tDelayWork_Pre;
static HostCmdRes diag_res;
static PreCalibrationCtrl s_tPreCalibCtrl;

static atomic_t g_CurrentSensorEnable;
static atomic_t g_bIsIntIrqEnable;
static atomic_t g_flgEna;
static atomic_t g_nCalX;
static atomic_t g_nCalY;
static atomic_t g_nCalZ;
static atomic_t g_nWeight;
static atomic_t g_nStepWide;
static atomic_t g_nVehiType;
static atomic_t g_WakeupSensor;
static atomic_t g_FWUpdateStatus;
static atomic_t s_nDataDelayTime;
static atomic_t s_nPreData_DelayTime;
static atomic_t g_nCalP;
static atomic_t g_nOfs_en;
static atomic_t g_nRefP;
static atomic_t g_Dailys_IntType;
static atomic_t g_ResetStatus;
static int u2dh_position = CONFIG_INPUT_ML610Q793_ACCELEROMETER_POSITION;
static const int u2dh_position_map[][3][3] = {
	{ { 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} }, /* top/upper-left */
	{ {-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} }, /* top/upper-right */
	{ { 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} }, /* top/lower-right */
	{ { 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} }, /* top/lower-left */
	{ { 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} }, /* bottom/upper-left */
	{ { 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} }, /* bottom/upper-right */
	{ { 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} }, /* bottom/lower-right */
	{ {-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} }, /* bottom/lower-right */
};
static int32_t flgA = 0,flgP = 0;
static bool g_bIsAlreadyExistAccImitationXYZData       = false;
static int32_t  g_naAccImitationData[3];
static struct sensor_power_callback acc_pre_power_cb;
static int32_t g_CurrentSensorEnable_backup;

static int32_t accsns_mov_acc_avg (MovFilterWork* pCtrl, int32_t sample,int32_t axis);
static void accsns_calibration_periodic(const struct acceleration* accData);
static int32_t accsns_activateEx(int32_t arg_iUpdate, int32_t arg_iSensType, int32_t arg_iEnable);
static int32_t accsns_power_onoff(bool arg_iEnable);
static int32_t accsns_acc_activate(bool arg_iEnable);
static int32_t accsns_acc_getdata(struct acceleration *arg_Acc);
static void accsns_acc_work_func(struct work_struct *work);
static int32_t accsns_pedom_activate(bool arg_iEnable);
static int32_t accsns_pedom_getdata(struct pedometer *arg_Pedo);
static int32_t accsns_vehicle_activate(bool arg_iEnable);
static int32_t accsns_vehicle_getdata(struct vehicle *arg_Vehi);
static int32_t accsns_measure(int32_t arg_iSensType);
static int32_t accsns_check_accsensor(void);
static int32_t accsns_hostcmd(const HostCmd *prm, HostCmdRes *res, uint8_t mode);
static int32_t accsns_waitcmd(uint16_t intBit);
static irqreturn_t accsns_irq_handler(int32_t irq, void *dev_id);
static void accsns_int_work_func(struct work_struct *work);
static void accsns_int_acc_work_func(struct work_struct *work);
static int32_t accsns_gpio_init(void);
#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static void accsns_shutdown( struct i2c_client *client );
static int32_t accsns_remove( struct i2c_client *client );
static int32_t accsns_probe( struct i2c_client *client,const struct i2c_device_id *id);
static int32_t accsns_i2c_read_proc(uint8_t adr, uint8_t *data, uint16_t size);
static int32_t accsns_i2c_write_proc(uint8_t adr, const uint8_t *data, uint8_t size);
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static void accsns_shutdown( struct spi_device *client );
static int32_t accsns_remove( struct spi_device *client );
static int32_t accsns_probe( struct spi_device *client );
static int32_t accsns_spi_read_proc(uint8_t adr, uint8_t *data, uint16_t size);
static int32_t accsns_spi_write_proc(uint8_t adr, const uint8_t *data, uint8_t size);
#endif
static int32_t accsns_workqueue_create( struct workqueue_struct *queue, void (*func)(struct work_struct *) );
static void accsns_timeout_dump(uint8_t *reg);
static void accsns_workqueue_init(void);
static void accsns_workqueue_delete(struct work_struct *work);
static int32_t accsns_get_delay(void);
static void accsns_set_delay(int32_t delay);
static int32_t presns_power_onoff(bool arg_iEnable);
static int32_t presns_pre_activate(bool arg_iEnable);
static int32_t presns_pre_getdata(struct pressure *arg_Pre);
static void accsns_calibration_range_chk(const struct acceleration* accData);
static void presns_calibration_range_chk(const struct pressure* preData);
static int32_t accsns_get_iwifi_data( int32_t* arg_ipIWifi );
static int32_t accsns_iwifi_getdata(struct iwifi *arg_IWifi);

static int32_t accsns_get_acceleration_data( int32_t* arg_ipXYZ );
static int32_t accsns_get_pedometer_data( int32_t* arg_ipPedom );
static int32_t accsns_get_vehicle_data( int32_t* arg_ipVehicle );
static int32_t accsns_activate_pedom(int32_t arg_iUpdate, int32_t arg_iEnable);
static int32_t accsns_calibration_mode(void);
static int32_t accsns_calibration_start(int32_t argMode);
static int32_t accsns_calibration_is_wait(void);
static void accsns_set_offset(int32_t* offsets);
static bool accsns_devif_error_check(void);
static int32_t accsns_update_fw(bool boot, uint8_t *arg_iData, uint32_t arg_iLen);
static int32_t accsns_update_fw_seq(uint8_t *arg_iData, uint32_t arg_iLen);
static int32_t accsns_get_fw_version(uint8_t *arg_iData);
static int32_t accsns_pedom_set_info(uint32_t arg_iWeight, uint32_t arg_iStepWide, uint32_t arg_iDelay);
static int32_t accsns_pedom_clear(int32_t clear_req);
static void accsns_debug_level_chg(int32_t lv);
static int32_t accsns_initialize( void );
#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static int32_t accsns_suspend( struct i2c_client *client, pm_message_t mesg );
static int32_t accsns_resume( struct i2c_client *client );
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static int32_t accsns_suspend( struct spi_device *client, pm_message_t mesg );
static int32_t accsns_resume( struct spi_device *client );
#endif
static int32_t accsns_io_poll_pedom(struct file *fp, poll_table *wait);
static int32_t accsns_io_poll_vehicle(struct file *fp, poll_table *wait);
static int32_t accsns_micon_i2c_enable(bool arg_iEnable);
static int32_t accsns_micon_initcmd(void);
static int32_t accsns_set_dev_param(void);

#define WAIT_FOR_EVENT_MS(time){                                             \
    int32_t  flg = 0;                                                        \
    long timeout;                                                            \
    timeout = msecs_to_jiffies(time);                                        \
    while(1){                                                                \
        if( wait_event_interruptible_timeout(s_tWaitInt, flg, timeout) == 0){\
            break;                                                           \
        }                                                                    \
    }                                                                        \
}

#define ENABLE_IRQ {                                                         \
    if((g_nIntIrqNo != -1) && (atomic_read(&g_bIsIntIrqEnable) == false)){   \
        atomic_set(&g_bIsIntIrqEnable,true);                                 \
        enable_irq(g_nIntIrqNo);                                             \
    }                                                                        \
}
#define DISABLE_IRQ {                                                        \
    if((g_nIntIrqNo != -1) && (atomic_read(&g_bIsIntIrqEnable) == true)){    \
        disable_irq_nosync(g_nIntIrqNo);                                     \
        atomic_set(&g_bIsIntIrqEnable,false);                                \
    }                                                                        \
}

#define ERR_WAKEUP {                                                                \
            int32_t iWakeupSensor;                                                  \
            int32_t iCurrentSensorEnable;                                           \
            iWakeupSensor = atomic_read(&g_WakeupSensor);                           \
            iCurrentSensorEnable = atomic_read(&g_CurrentSensorEnable);             \
            atomic_set(&g_WakeupSensor, iWakeupSensor | ((iCurrentSensorEnable >> 4) & ACCSNS_ACTIVE_ERROR));  \
            wake_up_interruptible(&s_tPollWaitPedom);                               \
            wake_up_interruptible(&s_tPollWaitVehicle);                             \
            wake_up_interruptible(&s_tPollWaitIWifi);                               \
}

#define ACC_FW_VERSION_GET_32BIT(data)         ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]))

#define ACCDATA_SIGN_COMVERT_12_32BIT(data)    ((data) & 0x00000800 ? (((~(data) & 0x00000FFF) + 1) * (-1)): (data))

static int32_t accsns_err_check(void)
{
  int ret_val = 0;

  if ((accsns_devif_error_check() != false) && (atomic_read(&g_ResetStatus) == false)) {
	sensor_power_reset(SENSOR_INDEX_ACC);
    ret_val = -ECOMM;
  } else {
    ret_val = -EFAULT;
  }
  
  return ret_val;
}

static void acc_pre_power_on(void)
{
	int32_t ret = ACCSNS_RC_OK;
	int32_t cnt = 0;
DBG_PRINT_IO(0, 0);

	if(accsns_devif_error_check() == false){
DBG(DBG_LV_INFO, "%s:Other Sensor Error \n", __FUNCTION__);
		while(1){
			ret = accsns_micon_i2c_enable(true);
			cnt++;
			if(ret == ACCSNS_RC_OK || cnt >= MICON_I2C_ENABLE_RETRY_NUM){
				DBG(DBG_LV_INFO, "%s: i2c enable cnt = %d \n", __FUNCTION__,cnt);
				break;
			}
			msleep(10);
		}
		ret |= accsns_micon_initcmd();
		ret |= accsns_set_dev_param();
	} else {
DBG(DBG_LV_INFO, "%s:Micon SPI Error \n", __FUNCTION__);
		ret |= accsns_initialize();
		if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_INFO, "%s:Micon SPI Error accsns_initialize ret = %d\n", __FUNCTION__,ret);
			g_CurrentSensorEnable_backup = ACTIVE_OFF;
			atomic_set(&g_ResetStatus,false);
			return;
		}
		ret |= accsns_set_dev_param();
	}

	if((g_CurrentSensorEnable_backup != ACTIVE_OFF) && (ret == ACCSNS_RC_OK)){
		if((g_CurrentSensorEnable_backup & ACCSNS_ACTIVE_ACC) == ACCSNS_ACTIVE_ACC){
			accsns_activateEx(0,ACCSNS_ACTIVE_ACC,POWER_ENABLE);
		}
		if((g_CurrentSensorEnable_backup & PRESNS_ACTIVE_PRE) == PRESNS_ACTIVE_PRE){
			accsns_activateEx(0,PRESNS_ACTIVE_PRE,POWER_ENABLE);
		}
		if((g_CurrentSensorEnable_backup & ACCSNS_ACTIVE_DAILYS) == ACCSNS_ACTIVE_DAILYS){
			accsns_activateEx(0,ACCSNS_ACTIVE_DAILYS,POWER_ENABLE);
		}
	}
	g_CurrentSensorEnable_backup = ACTIVE_OFF;
	atomic_set(&g_ResetStatus,false);

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
}

static void acc_pre_power_off(void)
{
	int32_t ret = ACCSNS_RC_OK;

DBG_PRINT_IO(0, 0);

	atomic_set(&g_ResetStatus,true);
	g_CurrentSensorEnable_backup = atomic_read(&g_CurrentSensorEnable);
	if(accsns_devif_error_check() == false){
DBG(DBG_LV_INFO, "%s:Other Sensor Error \n", __FUNCTION__);
		if(g_CurrentSensorEnable_backup != ACTIVE_OFF){
			ret = accsns_activateEx(0,g_CurrentSensorEnable_backup,POWER_DISABLE);
		}
		accsns_micon_i2c_enable(false);
	} else {
DBG(DBG_LV_INFO, "%s:Micon SPI Error \n", __FUNCTION__);
		if((g_CurrentSensorEnable_backup & ACCSNS_ACTIVE_ACC) == ACCSNS_ACTIVE_ACC){
			ret |= accsns_acc_activate(POWER_DISABLE);
		}
		if((g_CurrentSensorEnable_backup & PRESNS_ACTIVE_PRE) == PRESNS_ACTIVE_PRE){
			ret |= presns_pre_activate(POWER_DISABLE);
		}
	}

DBG_PRINT_IO(0xFF, ret);
}

static int32_t accsns_micon_i2c_enable(bool arg_iEnable)
{
	int32_t ret;
	HostCmd cmd;
	HostCmdRes res;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);

	cmd.cmd.udata16 = HC_MCU_SET_PERI;
	cmd.prm.ub_prm[0] = 0x01;
	cmd.prm.ub_prm[1] = (arg_iEnable == false) ? 0x01:0x00;
	cmd.prm.ub_prm[2] = 0x00;
	cmd.prm.ub_prm[3] = 0x00;
	cmd.prm.ub_prm[4] = 0x00;
	ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
	if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "I2C Valid/Invalid Error HC_MCU_SET_PERI(-) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
	}
	if((arg_iEnable == true) && res.res.ub_res[1] == 0x01)
	{
DBG(DBG_LV_ERROR, "I2C Valid Error RST01 %x\n", res.res.ub_res[1]);
		return ACCSNS_RC_ERR;
	}

	DBG_PRINT_IO(0xFF, ret);
	return ret;
}

static int32_t accsns_micon_initcmd(void)
{
	int32_t ret;
	HostCmd cmd;
	HostCmdRes res;
DBG_PRINT_IO(0, 0);

	cmd.cmd.udata16 = HC_MCU_SENSOR_INIT;
	cmd.prm.ub_prm[0] = 0x21;
	ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
	if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Sensor Init Error HC_MCU_SENSOR_INIT(-) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
	}

	if((res.res.ub_res[0] & 0x01) != 0)
	{
DBG(DBG_LV_ERROR, "[ACC] accsns_micon_initcmd:Acc Sensor Init Error[%x].\n", res.res.uw_res[0]);
		ret = ACCSNS_RC_ERR;
	}
	if((res.res.ub_res[0] & 0x20) != 0)
	{
DBG(DBG_LV_ERROR, "[ACC] accsns_micon_initcmd:Pre Sensor Init Error[%x].\n", res.res.uw_res[0]);
		ret = ACCSNS_RC_ERR;
	}

	DBG_PRINT_IO(0xFF, ret);
	return ret;
}

static int32_t accsns_get_acceleration_data( int32_t* arg_ipXYZ )
{
    int32_t ret = ACCSNS_RC_OK;
    int32_t nCalX,nCalY,nCalZ;
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        memset(arg_ipXYZ, 0x00, (sizeof(int32_t)*3));
        return 0;
    }

    nCalX = atomic_read(&g_nCalX) * (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    nCalY = atomic_read(&g_nCalY) * (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    nCalZ = atomic_read(&g_nCalZ) * (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);

    if(delayed_work_pending(&s_tDelayWork_Acc)) {
DBG(DBG_LV_LOW, "pending...\n");
        
    } else {
        ret = accsns_measure(ACCSNS_ACTIVE_ACC);
    }
    
    if(ret == ACCSNS_RC_OK){
        mutex_lock(&s_tDataMutex);
		if(atomic_read(&g_nOfs_en) == 1){
        s_tLatestAccData.outX -= (int32_t)nCalX;
        s_tLatestAccData.outY -= (int32_t)nCalY;
        s_tLatestAccData.outZ -= (int32_t)nCalZ;
		}
        arg_ipXYZ[0] = s_tLatestAccData.outX;
        arg_ipXYZ[1] = s_tLatestAccData.outY;
        arg_ipXYZ[2] = s_tLatestAccData.outZ;
        mutex_unlock(&s_tDataMutex);
    }
  
DBG(DBG_LV_LOW, "Acc, s_tLatestAccData x:%d, y:%d, z:%d\n", s_tLatestAccData.nX, s_tLatestAccData.nY, s_tLatestAccData.nZ );
DBG(DBG_LV_LOW, "Acc, nCal             x:%d, y:%d, z:%d\n", nCalX, nCalY, nCalZ );
DBG(DBG_LV_LOW, "Acc, ret=%02d         x:%d, y:%d, z:%d\n", ret, arg_ipXYZ[0], arg_ipXYZ[1], arg_ipXYZ[2] );
    return ret;
}

static int32_t accsns_get_pedometer_data( int32_t* arg_ipPedom )
{
    int32_t ret = 0;
DBG_PRINT_IO(0, 0);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        memset(arg_ipPedom, 0x00, (sizeof(int32_t)*21));
        return 0;
    }
	if(atomic_read(&g_ResetStatus) == true){
DBG(DBG_LV_ERROR, "[ACC] SensorReset Now:%s\n", __FUNCTION__);
	} else {
		ret = accsns_measure(ACCSNS_ACTIVE_PEDOM);
	}
    
    if(ret == ACCSNS_RC_OK){
        mutex_lock(&s_tDataMutex);
        arg_ipPedom[0] = s_tLatestPedoData.usStepCnt;
        arg_ipPedom[1] = s_tLatestPedoData.usWalkTime;
        arg_ipPedom[2] = s_tLatestPedoData.usCal;
        arg_ipPedom[3] = s_tLatestPedoData.usBodyFat;
        arg_ipPedom[4] = s_tLatestPedoData.usExercise;
        arg_ipPedom[5] = s_tLatestPedoData.usMets;
        arg_ipPedom[6] = s_tLatestPedoData.usSpeed;
        arg_ipPedom[7] = s_tLatestPedoData.usRunStatus;
        arg_ipPedom[8] = s_tLatestPedoData.usRunStepCnt;
        arg_ipPedom[9] = s_tLatestPedoData.usRunTime;
        arg_ipPedom[10] = s_tLatestPedoData.usStExercise;
        arg_ipPedom[11] = s_tLatestPedoData.usStCal;
        arg_ipPedom[12] = s_tLatestPedoData.usStBodyFat;
        arg_ipPedom[13] = s_tLatestPedoData.usSportExercise;
        arg_ipPedom[14] = s_tLatestPedoData.usRunCal;
        arg_ipPedom[15] = s_tLatestPedoData.usRunExercise;
        arg_ipPedom[16] = s_tLatestPedoData.usPS_InitBaseHeight;
        arg_ipPedom[17] = s_tLatestPedoData.usPS_ActHeight;
        arg_ipPedom[18] = s_tLatestPedoData.usPS_ActHeightMin;
        arg_ipPedom[19] = s_tLatestPedoData.usPS_ActHeightAve;
        arg_ipPedom[20] = s_tLatestPedoData.usPS_ActHeightMax;
        mutex_unlock(&s_tDataMutex);
    }

DBG(DBG_LV_DATA, "Pedom, ret=%d usStepCnt:%d, usWalkTime:%d, usCal:%d, usBodyFat:%d, usExercise:%d\n",ret,
				arg_ipPedom[0], arg_ipPedom[1], arg_ipPedom[2], arg_ipPedom[3], arg_ipPedom[4]);
DBG(DBG_LV_DATA, "Pedom, usMets:%d, usSpeed:%d usRunStatus:%d usRunStepCnt:%d usRunTime:%d\n",
				arg_ipPedom[5], arg_ipPedom[6], arg_ipPedom[7],arg_ipPedom[8],arg_ipPedom[9]);
DBG(DBG_LV_DATA, "Pedom, usStExercise:%d, usStCal:%d usStBodyFat:%d usSportExercise:%d\n",
				arg_ipPedom[10], arg_ipPedom[11],arg_ipPedom[12],arg_ipPedom[13]);
DBG(DBG_LV_DATA, "Pedom, usRunCal:%d, usRunExercise:%d usPS_InitBaseHeight:%d usPS_ActHeight:%d\n",
				arg_ipPedom[14], arg_ipPedom[15],arg_ipPedom[16],arg_ipPedom[17]);
DBG(DBG_LV_DATA, "Pedom, usPS_ActHeightMin:%d, usPS_ActHeightAve:%d usPS_ActHeightMax:%d\n",
				arg_ipPedom[18], arg_ipPedom[19],arg_ipPedom[20]);

DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_get_vehicle_data( int32_t* arg_ipVehicle )
{
    int32_t ret = 0;
DBG_PRINT_IO(0, 0);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
		memset(arg_ipVehicle, 0x00, (sizeof(int32_t)*15));
        return 0;
    }

	if(atomic_read(&g_ResetStatus) == true){
DBG(DBG_LV_ERROR, "[ACC] SensorReset Now:%s\n", __FUNCTION__);
	} else {
		ret = accsns_measure(ACCSNS_ACTIVE_VEHICLE);
	}
    
    if(ret == ACCSNS_RC_OK){
        mutex_lock(&s_tDataMutex);
        arg_ipVehicle[0] = s_tLatestVehiData.usVehiStatus;
        arg_ipVehicle[1] = s_tLatestVehiData.usVehiKind;
		arg_ipVehicle[2] = s_tLatestVehiData.usVehiDetectTime;
        arg_ipVehicle[3] = s_tLatestVehiData.usVehiRideTime;
        arg_ipVehicle[4] = s_tLatestVehiData.usVehiRideCal;
        arg_ipVehicle[5] = s_tLatestVehiData.usVehiBodyFat;
        arg_ipVehicle[6] = s_tLatestVehiData.usVehiExercise;
        arg_ipVehicle[7] = s_tLatestVehiData.usVehiMets;
        arg_ipVehicle[8] = s_tLatestVehiData.usVehiStExercise;
        arg_ipVehicle[9] = s_tLatestVehiData.usVehiStRideCal;
        arg_ipVehicle[10] = s_tLatestVehiData.usVehiStBodyFat;
        arg_ipVehicle[11] = s_tLatestVehiData.usVehiBiExercise;
        arg_ipVehicle[12] = s_tLatestVehiData.usVehiBiRideCal;
        arg_ipVehicle[13] = s_tLatestVehiData.usVehiBiBodyFat;
        arg_ipVehicle[14] = s_tLatestVehiData.usVehiSportExercise;
        mutex_unlock(&s_tDataMutex);
    }
  
DBG(DBG_LV_DATA, "Vehicle, ret=%d usVehiStatus:%d, usVehiKind:%d, usVehiDetectTime:%d, usVehiRideTime:%d\n",
					 ret, arg_ipVehicle[0], arg_ipVehicle[1], arg_ipVehicle[2], arg_ipVehicle[3]);
DBG(DBG_LV_DATA, "Vehicle, usVehiRideCal:%d, usVehiBodyFat:%d, usVehiExercise:%d, usVehiMets:%d\n",
					 arg_ipVehicle[4], arg_ipVehicle[5], arg_ipVehicle[6], arg_ipVehicle[7]);
DBG(DBG_LV_DATA, "Vehicle, usVehiStExercise:%d, usVehiStRideCal:%d, usVehiStBodyFat:%d\n",
					 arg_ipVehicle[8], arg_ipVehicle[9], arg_ipVehicle[10]);
DBG(DBG_LV_DATA, "Vehicle, usVehiBiExercise:%d, usVehiBiRideCal:%d, usVehiBiBodyFat:%d, usVehiSportExercise:%d\n",
					 arg_ipVehicle[11], arg_ipVehicle[12], arg_ipVehicle[13], arg_ipVehicle[14]);
  
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_get_iwifi_data( int32_t* arg_ipIWifi )
{
    int32_t ret = 0;
DBG_PRINT_IO(0, 0);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
		memset(arg_ipIWifi, 0x00, (sizeof(int32_t)*2));
        return 0;
    }

	if(atomic_read(&g_ResetStatus) == true){
DBG(DBG_LV_ERROR, "[ACC] SensorReset Now:%s\n", __FUNCTION__);
	} else {
		ret = accsns_measure(ACCSNS_ACTIVE_IWIFI);
	}

    if(ret == ACCSNS_RC_OK){
        mutex_lock(&s_tDataMutex);
        arg_ipIWifi[0] = s_tLatestIWifiData.usPedoStatus;
        arg_ipIWifi[1] = s_tLatestIWifiData.usVehiStatus;
        mutex_unlock(&s_tDataMutex);
    }
  
DBG(DBG_LV_DATA, "Intelli WiFi, ret=%d usPedoStatus:%d, usVehiStatus:%d\n",ret, arg_ipIWifi[0], arg_ipIWifi[1]);
  
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_pedom_set_info(uint32_t arg_iWeight, uint32_t arg_iStepWide, uint32_t arg_iVehiType)
{
	int32_t iflgEna;
	int32_t ret = 0;
DBG_PRINT_IO(0, 0);

DBG(DBG_LV_INFO, "accsns_pedom_set_info\n");
    if(arg_iWeight > MAX_WEIGHT){
        arg_iWeight = DEFAULT_WEIGHT;
    }
    atomic_set(&g_nWeight, arg_iWeight);
    
    if(arg_iStepWide > 0xFF){
        arg_iStepWide = DEFAULT_PEDOMETER;
    }
    atomic_set(&g_nStepWide, arg_iStepWide);
    
	if(arg_iVehiType > 0x03){
        arg_iVehiType = DEFAULT_VEHITYPE;
    }
    atomic_set(&g_nVehiType, arg_iVehiType);

	iflgEna = atomic_read(&g_flgEna);
DBG(DBG_LV_INFO, "arg_iWeight=%d, arg_iStepWide=%d arg_iVehiType=%d iflgEna=%x\n",
						arg_iWeight, arg_iStepWide, arg_iVehiType,iflgEna);

	if ((iflgEna & ACCSNS_ACTIVE_DAILYS) == ACCSNS_ACTIVE_DAILYS) {
		ret = accsns_pedom_activate(POWER_ENABLE);
		DBG(DBG_LV_INFO, "%s(): Set Info Immediately!! ret = %d\n",__func__,ret);
	}

DBG_PRINT_IO(0xFF, 0);
    return ret;
}

static int32_t accsns_get_acc_active(void)
{
	int32_t iCurrentEnable;
DBG_PRINT_IO(0, 0);
    
	iCurrentEnable = atomic_read(&g_CurrentSensorEnable);

DBG_PRINT_IO(0xFF, 0);
    return (iCurrentEnable & ACCSNS_ACTIVE_ACC ? 1:0);
}

static int32_t accsns_activate_pedom(int32_t arg_iUpdate, int32_t arg_iEnable)
{
DBG(DBG_LV_INFO, "accsns_activate_pedom \n");

    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return 0;
    }

    return accsns_activateEx(arg_iUpdate, ACCSNS_ACTIVE_DAILYS, arg_iEnable);
}

int32_t accsns_activate_vehicle(int32_t arg_iUpdate, int32_t arg_iEnable)
{
DBG(DBG_LV_INFO, "accsns_activate_vehicle \n");

    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return 0;
    }

    return accsns_activateEx(arg_iUpdate, ACCSNS_ACTIVE_VEHICLE, arg_iEnable);
}

int32_t accsns_calibration_mode(void)
{
    int32_t i = 0;

DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));

    s_tCalibCtrl.m_bFilterEnable        = false;
    s_tCalibCtrl.m_bWaitSetting         = true;
    s_tCalibCtrl.m_bComplete            = false;
	s_tCalibCtrl.m_bRengeChkErr         = false;

    s_tCalibCtrl.m_tFilterWork.m_ucAveN = OFFSET_AVE_NUM;   
    s_tCalibCtrl.m_nCalX                = 0;
    s_tCalibCtrl.m_nCalY                = 0;
    s_tCalibCtrl.m_nCalZ                = 0;

    s_tCalibCtrl.m_nCurrentSampleNum    = 0;
    s_tCalibCtrl.m_nSummationX          = 0;
    s_tCalibCtrl.m_nSummationY          = 0;
    s_tCalibCtrl.m_nSummationZ          = 0;
    s_tCalibCtrl.m_nMode                = -1; 
    
    for (i = 0; i < AXIS_XYZ_MAX; i++) {
        if(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] != NULL) {
DBG(DBG_LV_ERROR, "ACC Calib:err occurd \n");
            
        } else {
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] = NULL;
        }
    }
    memset(s_tCalibCtrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    memset(s_tCalibCtrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);

DBG(DBG_LV_INFO, "Enter AccCalibration Mode\n");
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_calibration_start(int32_t argMode)
{
    int32_t delay = 0;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
    s_tCalibCtrl.m_bWaitSetting = false;
    s_tCalibCtrl.m_nMode = argMode;

	if(s_tCalibCtrl.m_nMode == MODE_4){
		s_tCalibCtrl.m_bComplete  = false;
	}

    s_tCalibCtrl.m_nCurrentSampleNum    = 0;
    s_tCalibCtrl.m_nSummationX          = 0;
    s_tCalibCtrl.m_nSummationY          = 0;
    s_tCalibCtrl.m_nSummationZ          = 0;

    memset(s_tCalibCtrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    memset(s_tCalibCtrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);

DBG(DBG_LV_INFO, "Start AccCalibration Mode %d[%d]\n", s_tCalibCtrl.m_nMode,delay);
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

    delay = atomic_read(&s_nDataDelayTime);
	queue_delayed_work(accsns_cal_wq, &s_tDelayWork_Acc, msecs_to_jiffies(delay));

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_calibration_is_wait(void)
{
    int32_t wait;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
    wait = s_tCalibCtrl.m_bWaitSetting;
	if(wait == true && s_tCalibCtrl.m_nMode == MODE_4){
		wait = 2;
	}
	if(s_tCalibCtrl.m_bRengeChkErr == true){
		wait = 3;
	}
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

DBG(DBG_LV_INFO, "calib Wait %d\n", wait);
DBG_PRINT_IO(0xFF, wait);
    return wait;
}

static void accsns_set_offset(int32_t* offsets)
{
    int32_t temp;
DBG_PRINT_IO(0, 0);
    
    temp = ACCDATA_SIGN_COMVERT_12_32BIT(offsets[AXIS_X]);
    atomic_set(&g_nCalX, temp);
    
    temp = ACCDATA_SIGN_COMVERT_12_32BIT(offsets[AXIS_Y]);
    atomic_set(&g_nCalY, temp);
    
    temp = ACCDATA_SIGN_COMVERT_12_32BIT(offsets[AXIS_Z]);
    atomic_set(&g_nCalZ, temp);

DBG(DBG_LV_INFO, "set offset X %x -> %d\n", offsets[AXIS_X], temp);
DBG(DBG_LV_INFO, "set offset Y %x -> %d\n", offsets[AXIS_Y], temp);
DBG(DBG_LV_INFO, "set offset Z %x -> %d\n", offsets[AXIS_Z], temp);

DBG_PRINT_IO(0xFF, 0);
}

static int32_t accsns_get_delay(void)
{
	return atomic_read(&s_nDataDelayTime);
};

static void accsns_set_delay(int32_t delay)
{

	mutex_lock(&s_tEnableMutex);

	if (accsns_get_acc_active()) {
		cancel_delayed_work_sync(&s_tDelayWork_Acc_Data);
		atomic_set(&s_nDataDelayTime, delay);
		queue_delayed_work(accsns_data_wq, &s_tDelayWork_Acc_Data, msecs_to_jiffies(delay));
	} else {
		atomic_set(&s_nDataDelayTime, delay);
	}

	mutex_unlock(&s_tEnableMutex);

};

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static int32_t accsns_i2c_write_proc(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t ret = 0;
    uint8_t send_data[100];
DBG(DBG_LV_SPI, "[ACC]<%s>:Start adr=%x size=%x\n",__FUNCTION__, adr, size);

    if((data == NULL) || (size == 0)){
DBG(DBG_LV_ERROR, "I2C write input error(data %p, size %x)\n", data, size);
        return ACCSNS_RC_ERR;
    }

    adr &= SSIO_MASK_WRITE;

    send_data[0] = adr;
    memcpy(&send_data[1], data, size);

	DBG(DBG_LV_SPI, "[W] addr[%02x] [%02x]\n", adr, send_data[0]);

	ret = i2c_master_send(client_accsns, send_data, size+1);

	if (ret < 0){
DBG(DBG_LV_ERROR, "I2C write error(ret %x, adr %x, size %x)", ret, adr, size);
		return ret;
	} else {
		ret = 0;
	}

DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}

static int32_t accsns_i2c_read_proc(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t ret = 0;
	struct i2c_msg msg[2];
	uint8_t reg;

DBG(DBG_LV_SPI, "[ACC]<%s>:Start adr=%x size=%x\n",__FUNCTION__, adr, size);
    
    if( (data == NULL) || (size == 0)){
        DBG(DBG_LV_ERROR, "I2C read input error(data %p, size %x)", data, size);
        return ACCSNS_RC_ERR;
    }

    adr |= SSIO_MASK_READ;

	reg = adr;
	msg[0].addr = client_accsns->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = client_accsns->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size;
	msg[1].buf = data;

	ret = i2c_transfer(client_accsns->adapter, msg, 2);
	if (ret != 2) {
DBG(DBG_LV_ERROR, "I2C read error(ret %x, slave_addr=%02x, adr %x, size %x)",
			 ret, client_accsns->addr, adr, size);
		return ret;
	} else {
		ret = 0;
	}
    
DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}

static int32_t accsns_i2c_ram_write_proc(uint8_t adr, const uint8_t *data, int32_t size)
{
    int32_t ret = 0;
    uint8_t *send_data=  NULL;
DBG(DBG_LV_SPI, "[ACC]<%s>:Start adr=%x size=%x\n",__FUNCTION__, adr, size);

    if((data == NULL) || (size == 0)){
DBG(DBG_LV_ERROR, "I2C write input error(data %p, size %x)\n", data, size);
        return ACCSNS_RC_ERR;
    }

	send_data = (uint8_t *)kmalloc( size + 1, GFP_KERNEL );
	if(send_data == NULL){
		DBG(DBG_LV_ERROR, "error(kmalloc) : accsns_i2c_ram_write_proc\n" );
		return -ENOMEM;
	}

    adr &= SSIO_MASK_WRITE;

    send_data[0] = adr;
    memcpy(&send_data[1], data, size);

	DBG(DBG_LV_SPI, "[W] addr[%02x] [%02x]\n", adr, send_data[0]);

	ret = i2c_master_send(client_accsns, send_data, size+1);

	if (ret < 0){
DBG(DBG_LV_ERROR, "I2C write error(ret %x, adr %x, size %x)", ret, adr, size);
		kfree(send_data);
		return ret;
	} else {
		ret = 0;
	}

	kfree(send_data);
DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}

#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static int32_t accsns_spi_write_proc(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t ret = 0;
    struct spi_message  message;
    struct spi_transfer transfer;
    uint8_t send_data[100];
DBG(DBG_LV_SPI, "[ACC]<%s>:Start adr=%x size=%x\n",__FUNCTION__, adr, size);

    if((data == NULL) || (size == 0)){
DBG(DBG_LV_ERROR, "SPI write input error(data %p, size %x)\n", data, size);
        return ACCSNS_RC_ERR;
    }
    
    adr &= SSIO_MASK_WRITE;

    send_data[0] = adr;
    memcpy(&send_data[1], data, size);
    memset(&transfer, 0, sizeof(transfer));

    ret = spi_setup(client_accsns);
    if(ret < 0) {
DBG(DBG_LV_ERROR, "init SPI failed. ret=%x\n", ret);
        return ret;
    }
    spi_message_init(&message);
    
    transfer.tx_buf = send_data;
    transfer.rx_buf = NULL;
    transfer.len    = 1 + size;
    spi_message_add_tail(&transfer, &message);
    
    ret = spi_sync(client_accsns, &message);
    if(ret < 0){
DBG(DBG_LV_ERROR, "SPI write error(ret %x, adr %x, size %x)", ret, adr, size);
    }   
    
DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}

static int32_t accsns_spi_read_proc(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t ret = 0;
    struct spi_message  message;
    struct spi_transfer transfer[2];
    
DBG(DBG_LV_SPI, "[ACC]<%s>:Start adr=%x size=%x\n",__FUNCTION__, adr, size);

    
    if( (data == NULL) || (size == 0)){
        DBG(DBG_LV_ERROR, "SPI read input error(data %p, size %x)", data, size);
        return ACCSNS_RC_ERR;
    }
    
    memset(&transfer, 0, sizeof(transfer));
    
    adr |= SSIO_MASK_READ;
    
    ret = spi_setup(client_accsns);
    if(ret < 0){
DBG(DBG_LV_ERROR, "init SPI failed. ret=%x\n", ret);
        return ret;
    }
    spi_message_init(&message);

    transfer[0].tx_buf = &adr;
    transfer[0].rx_buf = NULL;
    transfer[0].len    = 1;
    spi_message_add_tail(&transfer[0], &message);

    transfer[1].tx_buf = NULL;
    transfer[1].rx_buf = (void *)data;
    transfer[1].len    = size;
    spi_message_add_tail(&transfer[1], &message);

    ret = spi_sync(client_accsns, &message);
    if(ret < 0){
DBG(DBG_LV_ERROR, "SPI read error(ret %x, adr %x, size %x)", ret, adr, size);
    }

DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}

static int32_t accsns_spi_ram_write_proc(uint8_t adr, const uint8_t *data, int32_t size)
{
    int32_t ret = 0;
    struct spi_message  message;
    struct spi_transfer transfer;
    uint8_t *send_data=  NULL;
DBG(DBG_LV_SPI, "[ACC]<%s>:Start adr=%x size=%x\n",__FUNCTION__, adr, size);

    if((data == NULL) || (size == 0)){
DBG(DBG_LV_ERROR, "SPI write input error(data %p, size %x)\n", data, size);
        return ACCSNS_RC_ERR;
    }

	send_data = (uint8_t *)kmalloc( size + 1, GFP_KERNEL );
	if(send_data == NULL){
		DBG(DBG_LV_ERROR, "error(kmalloc) : accsns_spi_ram_write_proc\n" );
		return -ENOMEM;
	}

    adr &= SSIO_MASK_WRITE;

    send_data[0] = adr;
    memcpy(&send_data[1], data, size);
    memset(&transfer, 0, sizeof(transfer));

    ret = spi_setup(client_accsns);
    if(ret < 0) {
DBG(DBG_LV_ERROR, "init SPI failed. ret=%x\n", ret);
		kfree(send_data);
        return ret;
    }
    spi_message_init(&message);
    
    transfer.tx_buf = send_data;
    transfer.rx_buf = NULL;
    transfer.len    = 1 + size;
    spi_message_add_tail(&transfer, &message);
    
    ret = spi_sync(client_accsns, &message);
	kfree(send_data);

    if(ret < 0){
DBG(DBG_LV_ERROR, "SPI write error(ret %x, adr %x, size %x)", ret, adr, size);
    }
    
DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}
#endif

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
int32_t accsns_device_write(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t i;
    int32_t ret;
    
    for(i=0; i<ACC_SPI_RETRY_NUM; i++)
    {
        ret = accsns_i2c_write_proc(adr, data, size);
        if(ret == 0){
            return 0;
            
        }else if(ret == -EBUSY){
DBG(DBG_LV_ERROR, "I2C write EBUSY error(Retry:%d)\n", i);
            msleep(100);
            
        }else{
            g_bDevIF_Error = true;
DBG(DBG_LV_ERROR, "I2C write Other error (H/W Reset ON) \n");
            break;
        }
    }
    
    return ret;
}

int32_t accsns_device_read(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t i;
    int32_t ret;
    
    for(i=0; i<ACC_SPI_RETRY_NUM; i++)
    {
        ret = accsns_i2c_read_proc(adr, data, size);
        if(ret == 0){
            return 0;
            
        }else if(ret == -EBUSY){
DBG(DBG_LV_ERROR, "I2C read EBUSY error(Retry:%d)\n", i);
            msleep(100);
            
        }else{
            g_bDevIF_Error = true;
DBG(DBG_LV_ERROR, "I2C read Other error (H/W Reset ON) \n");
            break;
        }
    }
    
    return ret;
}
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
int32_t accsns_device_write(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t i;
    int32_t ret;
    
    for(i=0; i<ACC_SPI_RETRY_NUM; i++)
    {
        ret = accsns_spi_write_proc(adr, data, size);
        if(ret == 0){
            return 0;
            
        }else if((ret == -EBUSY) || ((ret == -EIO) && (i < ACC_SPI_RETRY_NUM - 1))){
DBG(DBG_LV_ERROR, "SPI write EBUSY error(Retry:%d)\n", i);
            msleep(100);
            
        }else{
            g_bDevIF_Error = true;
DBG(DBG_LV_ERROR, "SPI write Other error (H/W Reset ON) \n");
            break;
        }
    }
    
    return ret;
}

int32_t accsns_device_read(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t i;
    int32_t ret;
    
    for(i=0; i<ACC_SPI_RETRY_NUM; i++)
    {
        ret = accsns_spi_read_proc(adr, data, size);
        if(ret == 0){
            return 0;
            
        }else if((ret == -EBUSY) || ((ret == -EIO) && (i < ACC_SPI_RETRY_NUM - 1))){
DBG(DBG_LV_ERROR, "SPI read EBUSY error(Retry:%d)\n", i);
            msleep(100);
            
        }else{
            g_bDevIF_Error = true;
DBG(DBG_LV_ERROR, "SPI read Other error (H/W Reset ON) \n");
            break;
        }
    }
    
    return ret;
}
#endif

static bool accsns_devif_error_check(void)
{
DBG(DBG_LV_SPI, "accsns_devif_error_check\n");
    return g_bDevIF_Error;
}

static int32_t accsns_io_poll_pedom(struct file *fp, poll_table *wait)
{
    int32_t iWakeupSensor;
    int32_t ret = 0;

    poll_wait(fp, &s_tPollWaitPedom, wait);

    iWakeupSensor = atomic_read(&g_WakeupSensor);
    if ((iWakeupSensor & ACCSNS_ACTIVE_PEDOM_ERROR) == ACCSNS_ACTIVE_PEDOM_ERROR) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_PEDOM_ERROR));
        ret |= ACCSNS_ACTIVE_PEDOM_ERROR;
    } else if ((iWakeupSensor & ACCSNS_ACTIVE_PEDOM) == ACCSNS_ACTIVE_PEDOM) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_PEDOM));
        ret |= ACCSNS_ACTIVE_PEDOM;
    }
    
    return ret;
}

static int32_t accsns_io_poll_vehicle(struct file *fp, poll_table *wait)
{
    int32_t iWakeupSensor;
    int32_t ret = 0;

    poll_wait(fp, &s_tPollWaitVehicle, wait);

    iWakeupSensor = atomic_read(&g_WakeupSensor);
    if ((iWakeupSensor & ACCSNS_ACTIVE_VEHICLE_ERROR) == ACCSNS_ACTIVE_VEHICLE_ERROR) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_VEHICLE_ERROR));
        ret |= ACCSNS_ACTIVE_VEHICLE_ERROR;
    } else if ((iWakeupSensor & ACCSNS_ACTIVE_VEHICLE) == ACCSNS_ACTIVE_VEHICLE) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_VEHICLE));
        ret |= ACCSNS_ACTIVE_VEHICLE;
    }
    
    return ret;
}

int32_t accsns_io_poll_iwifi(struct file *fp, poll_table *wait)
{
    int32_t iWakeupSensor;
    int32_t ret = 0;

    poll_wait(fp, &s_tPollWaitIWifi, wait);

    iWakeupSensor = atomic_read(&g_WakeupSensor);
    if ((iWakeupSensor & ACCSNS_ACTIVE_IWIFI_ERROR) == ACCSNS_ACTIVE_IWIFI_ERROR) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_IWIFI_ERROR));
        ret |= ACCSNS_ACTIVE_IWIFI_ERROR;
    } else if ((iWakeupSensor & ACCSNS_ACTIVE_IWIFI) == ACCSNS_ACTIVE_IWIFI) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_IWIFI));
        ret |= ACCSNS_ACTIVE_IWIFI;
    }
    
    return ret;
}

static int32_t accsns_mov_acc_avg (MovFilterWork* pCtrl, int32_t sample,int32_t axis)
{
DBG_PRINT_IO(0, 0);
    
    if(pCtrl->m_pSamplWork[axis] == NULL) {
        pCtrl->m_pSamplWork[axis] = kzalloc(sizeof(int32_t) * pCtrl->m_ucAveN, GFP_KERNEL);
        
        if(pCtrl->m_pSamplWork[axis] == NULL) {
DBG(DBG_LV_ERROR, "ACC-Calib:Memory crisis\n");
            return ACCSNS_RC_ERR;
        }
        memset(pCtrl->m_pSamplWork[axis], 0x00, sizeof(int32_t) * pCtrl->m_ucAveN);
    }

    if(pCtrl->m_unCnt[axis]++ < pCtrl->m_ucAveN) {
        pCtrl->m_pSamplWork[axis][(pCtrl->m_unCnt[axis]-1) % pCtrl->m_ucAveN] = sample;
        pCtrl->m_unSum[axis] += sample;
        
#ifdef CONFIG_ML610Q793_DEBUG
{
    int32_t i = 0;
    if( DBG_LV_DATA & dbg_level ){
        printk("ACC-Calib:List %d:",pCtrl->m_unCnt[axis]);
        for (i = 0; i < pCtrl->m_ucAveN; i++) {
            printk("%d,",pCtrl->m_pSamplWork[axis][i]);
        }
        printk("\n");
    }
}
#endif
        return ACCSNS_RC_ERR;

    } else {
        pCtrl->m_unSum[axis] -= pCtrl->m_pSamplWork[axis][(pCtrl->m_unCnt[axis]-1) % pCtrl->m_ucAveN];
        pCtrl->m_unSum[axis] += sample;
        pCtrl->m_pSamplWork[axis][(pCtrl->m_unCnt[axis]-1) % pCtrl->m_ucAveN] = sample;
    }

#ifdef CONFIG_ML610Q793_DEBUG
{
    int32_t i = 0;
    printk("List %d:",pCtrl->m_unCnt[axis]);
    for (i = 0; i < pCtrl->m_ucAveN; i++) {
        printk("%d,",pCtrl->m_pSamplWork[axis][i]);
    }
    printk("\n");
}
#endif
  
DBG_PRINT_IO(0xFF, (pCtrl->m_unSum[axis] / pCtrl->m_ucAveN));
    return pCtrl->m_unSum[axis] / pCtrl->m_ucAveN;
}

static void accsns_calibration_periodic(const struct acceleration* accData)
{
    int32_t ret = 0;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
DBG(DBG_LV_INFO, "calibPeriodic:%d\n",s_tCalibCtrl.m_nCurrentSampleNum);
    
    if(s_tCalibCtrl.m_nCurrentSampleNum == s_tCalibCtrl.m_unSmpN) {
        s_tCalibCtrl.m_bComplete    = true;
        s_tCalibCtrl.m_bWaitSetting = true;
        
    } else {
		accsns_calibration_range_chk(accData);

        if(s_tCalibCtrl.m_bFilterEnable == true) {
            ret = accsns_mov_acc_avg(&(s_tCalibCtrl.m_tFilterWork), accData->nX, AXIS_X);
            if(ret != ACCSNS_RC_ERR) {
                s_tCalibCtrl.m_nCurrentSampleNum++;
                s_tCalibCtrl.m_nSummationX += ret;
            }
            
            ret = accsns_mov_acc_avg(&(s_tCalibCtrl.m_tFilterWork), accData->nY, AXIS_Y);
            if(ret != ACCSNS_RC_ERR) {
                s_tCalibCtrl.m_nSummationY += ret;
            }
            
            ret = accsns_mov_acc_avg(&(s_tCalibCtrl.m_tFilterWork), accData->nZ, AXIS_Z);
            if(ret != ACCSNS_RC_ERR) {
                s_tCalibCtrl.m_nSummationZ += ret;
            }
            
        } else {
            s_tCalibCtrl.m_nSummationX += accData->nX;
            s_tCalibCtrl.m_nSummationY += accData->nY;
            s_tCalibCtrl.m_nSummationZ += accData->nZ;
            
            s_tCalibCtrl.m_nCurrentSampleNum++;
        }
        
        if(s_tCalibCtrl.m_nCurrentSampleNum == s_tCalibCtrl.m_unSmpN) {
            switch (s_tCalibCtrl.m_nMode) {
                case MODE_0:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalZ = 0;
                    break;
                
                case MODE_1:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalZ = (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN) - WEIGHT_1G;
                    break;
                
                case MODE_2:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalZ = (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN) + WEIGHT_1G;
                    break;

                case MODE_3:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN);
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN);
                    s_tCalibCtrl.m_nCalZ = (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN);
                    break;
                
                case MODE_4:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nCalX + (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN)) / 2;
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nCalY + (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN)) / 2;
                    s_tCalibCtrl.m_nCalZ = (s_tCalibCtrl.m_nCalZ + (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN)) / 2; 
                    break;
                
                default:
                    ret = ACCSNS_RC_ERR;
DBG(DBG_LV_ERROR, "ACC-Calib: Mode Err!!\n");
                    break;
            }

            if(ret != ACCSNS_RC_ERR && s_tCalibCtrl.m_bRengeChkErr == false) {
                atomic_set(&g_nCalX, s_tCalibCtrl.m_nCalX);
                atomic_set(&g_nCalY, s_tCalibCtrl.m_nCalY);
                atomic_set(&g_nCalZ, s_tCalibCtrl.m_nCalZ);
				input_report_abs(acc_idev, ABS_RX, s_tCalibCtrl.m_nCalX);
				input_report_abs(acc_idev, ABS_RY, s_tCalibCtrl.m_nCalY);
				input_report_abs(acc_idev, ABS_RZ, s_tCalibCtrl.m_nCalZ);
DBG(DBG_LV_DATA, "AccCalib:mode[%d] complete calX = %d sumX = %u smp=%u\n",s_tCalibCtrl.m_nMode,  s_tCalibCtrl.m_nCalX, s_tCalibCtrl.m_nSummationX, s_tCalibCtrl.m_unSmpN);
DBG(DBG_LV_DATA, "AccCalib:mode[%d] complete calY = %d sumY = %u smp=%u\n",s_tCalibCtrl.m_nMode, s_tCalibCtrl.m_nCalY, s_tCalibCtrl.m_nSummationY, s_tCalibCtrl.m_unSmpN);
DBG(DBG_LV_DATA, "AccCalib:mode[%d] complete calZ = %d sumZ = %u smp=%u\n",s_tCalibCtrl.m_nMode, s_tCalibCtrl.m_nCalZ, s_tCalibCtrl.m_nSummationZ, s_tCalibCtrl.m_unSmpN);
            }

            kfree(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_X]);
            kfree(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Y]);
            kfree(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Z]);
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_X] = NULL;
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Y] = NULL;
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Z] = NULL;
        }
    }
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
    
DBG_PRINT_IO(0xFF, 0);
}

static int32_t accsns_activateEx(int32_t arg_iUpdate, int32_t arg_iSensType, int32_t arg_iEnable)
{
    bool bIsChanged = false;
    int32_t ret  = ACCSNS_RC_OK; 
    int32_t iCurrentEnable,tmp_iCurrentEnable;
    int32_t iflgEna;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iUpdate %x, arg_iSensType %x, arg_iEnable %x\n",arg_iUpdate, arg_iSensType, arg_iEnable);

    if(arg_iSensType == ACTIVE_OFF){
        return ret;
    }
    
    if(arg_iEnable != 0) {
        arg_iEnable = POWER_ENABLE;
    }
    
    iCurrentEnable = atomic_read(&g_CurrentSensorEnable);
    iflgEna = iCurrentEnable;
    
    if(arg_iEnable) {
		if(!((iCurrentEnable | ACC_ENABLE_MASK) & arg_iSensType)){
            if(!(iCurrentEnable & ACTIVE_ON)){
                ret = accsns_power_onoff(POWER_ENABLE);
                if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Error ret %x\n", ret);
                    return ret;
                }
                iCurrentEnable |= ACTIVE_ON;
            }
            
            if(arg_iSensType & ACCSNS_ACTIVE_ACC){
                ret |= accsns_acc_activate(POWER_ENABLE);
            }
		}

		if(!((iCurrentEnable | PRE_ENABLE_MASK) & arg_iSensType)){
            if(!(iCurrentEnable & ACTIVE_PRE_ON)){
                ret = presns_power_onoff(POWER_ENABLE);
                if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Error ret %x\n", ret);
                    return ret;
                }
                iCurrentEnable |= ACTIVE_PRE_ON;
            }
            if(arg_iSensType & PRESNS_ACTIVE_PRE){
                ret |= presns_pre_activate(POWER_ENABLE);
            }
            if((arg_iSensType & ACCSNS_ACTIVE_DAILYS) == ACCSNS_ACTIVE_DAILYS){
                ret |= accsns_pedom_activate(POWER_ENABLE);
            }
            if(arg_iSensType & ACCSNS_ACTIVE_VEHICLE){
                ret |= accsns_vehicle_activate(POWER_ENABLE);
            }
        }

        iCurrentEnable |= arg_iSensType;
        iflgEna         = iCurrentEnable;

        bIsChanged = true;
DBG(DBG_LV_INFO, "[Enable] All OK[%d]\n", ret);
        
    }else{
        if(iCurrentEnable & (arg_iSensType & ~PRE_ENABLE_MASK)){
            if(arg_iSensType & PRESNS_ACTIVE_PRE){
                ret |= presns_pre_activate(POWER_DISABLE);
            }
            if((arg_iSensType & ACCSNS_ACTIVE_DAILYS) == ACCSNS_ACTIVE_DAILYS){
                ret |= accsns_pedom_activate(POWER_DISABLE);
            }
            if(arg_iSensType & ACCSNS_ACTIVE_VEHICLE){
                ret |= accsns_vehicle_activate(POWER_DISABLE);
            }
			tmp_iCurrentEnable = (iCurrentEnable & (~arg_iSensType));

            if( (iCurrentEnable & ACTIVE_PRE_ON) && (!(tmp_iCurrentEnable & ACTIVE_FUNC_PRE_MASK)) ){
                 ret |= presns_power_onoff(POWER_DISABLE);
                 iCurrentEnable &= ~ACTIVE_PRE_ON;
            }
		}

        if(iCurrentEnable & (arg_iSensType & ~ACC_ENABLE_MASK)){

            if(arg_iSensType & ACCSNS_ACTIVE_ACC){
                ret |= accsns_acc_activate(POWER_DISABLE);
            }
            
			tmp_iCurrentEnable = (iCurrentEnable & (~arg_iSensType));
            
            if(dummy_on_flg == false) {
                if( (iCurrentEnable & ACTIVE_ON) && (!(tmp_iCurrentEnable & ACTIVE_FUNC_MASK)) ){
                    ret |= accsns_power_onoff(POWER_DISABLE);
                    
                    iCurrentEnable &= ~ACTIVE_ON;
                    
                }
            }
        }
		iCurrentEnable &= ~arg_iSensType;
        iflgEna = iCurrentEnable;

        bIsChanged = true;
DBG(DBG_LV_INFO, "[Disable] All OK[%d]\n", ret);
    }
    
    if(bIsChanged) {
        atomic_set(&g_CurrentSensorEnable,iCurrentEnable);
DBG(DBG_LV_INFO, "CurrentEnable update.(%x)\n",iCurrentEnable);
    }
    
    if(arg_iUpdate) {
        atomic_set(&g_flgEna, iflgEna);
DBG(DBG_LV_INFO, "FlgEnable update.(%x)\n",iflgEna);
    }
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_power_onoff(bool arg_iEnable)
{
    int32_t   ret;
	uint8_t   sns_enable;
    HostCmd cmd;
    HostCmdRes res;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    if(arg_iEnable == true){
        cmd.cmd.udata16 = HC_ACC_SET_AUTO_MEASURE;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x1E;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x00;
        cmd.prm.ub_prm[5] = 0x04;
        cmd.prm.ub_prm[6] = 0x00;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power On Error HC_ACC_SET_AUTO_MEASURE(-) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
        }

		cmd.cmd.udata16 = HC_MUL_GET_ANDROID;
		ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
		if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power On Error HC_MUL_GET_ANDROID(-) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
		}
		sns_enable = res.res.ub_res[0];

        cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
        cmd.prm.ub_prm[0] = HC_VALID | res.res.ub_res[0];
		cmd.prm.ub_prm[1] = res.res.ub_res[1];
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power On Error HC_MUL_SET_ANDROID(HC_VALID) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
        }

		if(0 == sns_enable){
			cmd.cmd.udata16 = HC_MUL_MEASURE;
			cmd.prm.ub_prm[0] = MT_ANDROID_START;
			ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
			if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power On Error HC_MUL_MEASURE(MT_ANDROID_START) err %x\n", res.err.udata16);
				return ACCSNS_RC_ERR;
			}
		}
    }else{
		cmd.cmd.udata16 = HC_MUL_GET_ANDROID;
		ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
		if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power Off Error HC_MUL_GET_ANDROID(-) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
		}
		sns_enable = res.res.ub_res[0];

        cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
        cmd.prm.ub_prm[0] = res.res.ub_res[0] & 0xFE;
		cmd.prm.ub_prm[1] = res.res.ub_res[1];
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power Off Error HC_MUL_SET_ANDROID(%x) err %x\n", res.err.udata16,(res.res.ub_res[0] & 0xFE));
            return ACCSNS_RC_ERR;
        }

		if(0x01 == sns_enable){
			cmd.cmd.udata16 = HC_MUL_MEASURE;
			cmd.prm.ub_prm[0] = MT_ANDROID_STOP;
			ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
			if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power Off Error HC_MUL_MEASURE(MT_ANDROID_STOP) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
			}
		}
	}

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_set_dev_param(void)
{
    int32_t   ret;
    HostCmd cmd;
    HostCmdRes res;
DBG_PRINT_IO(0, 0);

    cmd.cmd.udata16 = HC_ACC_SET_CONV_AXIS;
	cmd.prm.ub_prm[0] = 0x01;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Set Acc AXIS HC_ACC_SET_CONV_AXIS(-) err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID_PERIOD;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x03;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Set Andorid Period HC_MUL_SET_ANDROID_PERIOD(-) err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

	cmd.cmd.udata16 = HC_ACC_SET_U2DH;
	cmd.prm.ub_prm[0] = 0x02;
	cmd.prm.ub_prm[1] = 0x05;
	cmd.prm.ub_prm[2] = 0x01;
	ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
	if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Set U2DH Param HC_ACC_SET_U2DH(-) err %x\n", res.err.udata16);
		return ACCSNS_RC_ERR;
	}

	cmd.cmd.udata16 = HC_MCU_I2C_IO;
	cmd.prm.ub_prm[0] = 0x77;
	cmd.prm.ub_prm[1] = 0x00;
	cmd.prm.ub_prm[2] = 0x88;
	cmd.prm.ub_prm[3] = 0x01;
	ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
	if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "T5400 I2C Read Error HC_MCU_I2C_IO(-) err %x\n", res.err.udata16);
		return ACCSNS_RC_ERR;
	}

	if((res.res.ub_res[0] == 0) && ((res.res.ub_res[1] == 0x77) || (res.res.ub_res[1] == 0xF7))){
DBG(DBG_LV_INFO, "%s: Detect Pressure Sensor T5400\n",__func__);
    	cmd.cmd.udata16 = HC_MUL_SET_T5400;
		cmd.prm.ub_prm[0] = 0x03;
		cmd.prm.ub_prm[1] = 0x02;
    	ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    	if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Set T5400 Param HC_MUL_SET_T5400(-) err %x\n", res.err.udata16);
        	return ACCSNS_RC_ERR;
		}
	} else {
		cmd.cmd.udata16 = HC_MCU_I2C_IO;
		cmd.prm.ub_prm[0] = 0x76;
		cmd.prm.ub_prm[1] = 0x00;
		cmd.prm.ub_prm[2] = 0xD0;
		cmd.prm.ub_prm[3] = 0x01;
		ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
		if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "BMP280 I2C Read Error HC_MCU_I2C_IO(-) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
		}
		if((res.res.ub_res[0] == 0) && (res.res.ub_res[1] == 0x56)){
DBG(DBG_LV_INFO, "%s: Detect Pressure Sensor BMP280\n",__func__);
			cmd.cmd.udata16 = HC_PRE_SET_PARAM_BMP280;
			cmd.prm.ub_prm[0] = 0x00;
			cmd.prm.ub_prm[1] = 0x01;
			ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
			if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Set BMP280 Param HC_PRE_SET_PARAM_BMP280(-) err %x\n", res.err.udata16);
				return ACCSNS_RC_ERR;
			}
		}
	}

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_acc_activate(bool arg_iEnable)
{
	int delay;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
	if(arg_iEnable == true)
	{
		delay = accsns_get_delay();
		queue_delayed_work(accsns_data_wq, &s_tDelayWork_Acc_Data, msecs_to_jiffies(delay));
		
	} else {
		cancel_delayed_work_sync(&s_tDelayWork_Acc_Data);
	}

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
  return ACCSNS_RC_OK;
}

static int32_t accsns_acc_getdata(struct acceleration *arg_Acc)
{
    uint8_t ucBuff[MEASURE_DATA_SIZE];
    int32_t ret;
    int32_t temp;
	int32_t raw[3]={0};
	int32_t pos[3]={0};
	int32_t xyz[3];
	int i,j;
    
    ret = accsns_device_read(RSLT0E, ucBuff, sizeof(ucBuff));
    if(ACCSNS_RC_OK == ret){
    
        temp = (int32_t)ucBuff[0] | (((int32_t)ucBuff[1] & 0x0F) << 8);
		raw[0] = ACCDATA_SIGN_COMVERT_12_32BIT(temp);
        
        temp = (int32_t)ucBuff[2] | (((int32_t)ucBuff[3] & 0x0F) << 8);
		raw[1] = ACCDATA_SIGN_COMVERT_12_32BIT(temp);
        
        temp = (int32_t)ucBuff[4] | (((int32_t)ucBuff[5] & 0x0F) << 8);
		raw[2] = ACCDATA_SIGN_COMVERT_12_32BIT(temp);

DBG(DBG_LV_LOW, "reg     x:%02x %02x, y:%02x %02x, z:%02x %02x\n",ucBuff[1],ucBuff[0],ucBuff[3],ucBuff[2],ucBuff[5],ucBuff[4]);
DBG(DBG_LV_LOW, "raw[0]:%04x(%d), raw[1]:%04x(%d), raw[2]:%04x(%d)\n", raw[0], raw[0], raw[1], raw[1], raw[2], raw[2]);
    }

	for (i = 0; i < 3; i++) {
		xyz[i] = 0;
		for (j = 0; j < 3; j++){
			xyz[i] += raw[j] * u2dh_position_map[u2dh_position][i][j];
		}
		pos[i] = xyz[i];
		xyz[i] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
	}

	arg_Acc->nX   = pos[0];
	arg_Acc->nY   = pos[1];
	arg_Acc->nZ   = pos[2];
	arg_Acc->outX = xyz[0];
	arg_Acc->outY = xyz[1];
	arg_Acc->outZ = xyz[2];
DBG(DBG_LV_LOW, "arg_Acc x:%04x(%d), y:%04x(%d), z:%04x(%d)\n", arg_Acc->nX, arg_Acc->nX, arg_Acc->nY, arg_Acc->nY, arg_Acc->nZ, arg_Acc->nZ);
    
    return ret;
}

static void accsns_acc_work_func(struct work_struct *work)
{
    int32_t ret = ACCSNS_RC_ERR;
    int32_t delay = 0;
    bool bCalibIdle = false;
    bool bCalibComp = false;
DBG_PRINT_IO(0, 0);
    
    ret = accsns_measure(ACCSNS_ACTIVE_ACC);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_measure Error err %x\n", ret);
        return;
    }
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
    bCalibIdle = s_tCalibCtrl.m_bWaitSetting;
    bCalibComp = s_tCalibCtrl.m_bComplete;
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
    
    if((bCalibIdle == false) && (bCalibComp == false)){
    
        mutex_lock(&s_tDataMutex);
        accsns_calibration_periodic(&s_tLatestAccData);
        mutex_unlock(&s_tDataMutex);
        
        mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
        bCalibComp = s_tCalibCtrl.m_bComplete;
        mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
        if(bCalibComp == false){
            delay = atomic_read(&s_nDataDelayTime);
			queue_delayed_work(accsns_cal_wq, &s_tDelayWork_Acc, msecs_to_jiffies(delay));
        }
    }
    
DBG(DBG_LV_INFO, "accsns_acc_work_func:Compleate ret=%x, bCalibIdle=%x, delay=%x\n", ret, bCalibIdle, delay);
DBG_PRINT_IO(0xFF, 0);
}

static void accsns_data_work_func(struct work_struct *work)
{
    int32_t ret = ACCSNS_RC_ERR;
    int32_t delay = 0;
	int32_t xyz[3] = {0};
	struct acceleration last;
	static int cnt;
DBG_PRINT_IO(0, 0);
	memcpy(&last,&s_tLatestAccData,sizeof(last));

	ret = accsns_get_acceleration_data(xyz);

	if(g_bIsAlreadyExistAccImitationXYZData == true){
		xyz[0] = g_naAccImitationData[0];
		xyz[1] = g_naAccImitationData[1];
		xyz[2] = g_naAccImitationData[2];
	}

	if(ret == ACCSNS_RC_OK){
		input_report_abs(acc_idev, ABS_X, xyz[0]);
		input_report_abs(acc_idev, ABS_Y, xyz[1]);
		input_report_abs(acc_idev, ABS_Z, xyz[2]);
		if (last.outX == xyz[0] &&
	    	last.outY == xyz[1] &&
	    	last.outZ == xyz[2])
			input_report_abs(acc_idev, ABS_RUDDER, cnt++);
		input_sync(acc_idev);
	}

	delay = accsns_get_delay();
	if (delay > 0)
		queue_delayed_work(accsns_data_wq, &s_tDelayWork_Acc_Data, msecs_to_jiffies(delay));
	else
		DBG(DBG_LV_INFO, "%s(): delay=%d\n",__func__,delay);

DBG(DBG_LV_INFO, "accsns_data_work_func:Compleate ret=%x, delay=%x\n", ret, delay);
DBG_PRINT_IO(0xFF, 0);
}

static int32_t accsns_pedom_activate(bool arg_iEnable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
    int32_t type = atomic_read(&g_Dailys_IntType);
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    cmd.cmd.udata16 = HC_DST_SET_DAILYS;
    
    if(arg_iEnable == true){
        cmd.prm.ub_prm[1] = HC_VALID;
    }else{
		if(type == 0){
        cmd.prm.ub_prm[1] = HC_INVALID;
		} else {
			cmd.prm.ub_prm[1] = HC_VALID;
		}
    }
    
	cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[2] = atomic_read(&g_nStepWide);
    cmd.prm.ub_prm[3] = (atomic_read(&g_nWeight) & 0xFF);
    cmd.prm.ub_prm[4] = ((atomic_read(&g_nWeight) >> 8) & 0xFF);
    cmd.prm.ub_prm[5] = atomic_read(&g_nVehiType);
    cmd.prm.ub_prm[6] = 0x00;
    cmd.prm.ub_prm[7] = (type & DAILYS_INT_DST) ? 0x01:0x00;
    cmd.prm.ub_prm[8] = 0x00;
    cmd.prm.ub_prm[9] = (type & DAILYS_INT_WIFI) ? 0x01:0x00;
	cmd.prm.ub_prm[10] = 0x00;
    
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Pedometer Activate Ctrl Error HC_DST_SET_DAILYS(%d) err %x\n",cmd.prm.ub_prm[0], res.err.udata16);
        return ACCSNS_RC_ERR;
    }

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_pedom_getdata(struct pedometer *arg_Pedo)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_DST_GET_PEDO1;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usStepCnt  = res.res.ud_res[0];
        arg_Pedo->usWalkTime = res.res.ud_res[1];
        arg_Pedo->usCal      = res.res.uw_res[4];

    }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_DST_GET_PEDO1 err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
    cmd.cmd.udata16 = HC_DST_GET_PEDO2;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usBodyFat  = res.res.ud_res[0];
        arg_Pedo->usExercise = res.res.ud_res[1];
        arg_Pedo->usMets     = res.res.ub_res[8];
		arg_Pedo->usSpeed    = ((res.res.ub_res[10] << 8 ) | res.res.ub_res[9]);

    }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_DST_GET_PEDO2 err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_RUN1;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usRunStatus  = res.res.ub_res[0];
		arg_Pedo->usRunStepCnt = ((res.res.ub_res[4] << 24) | (res.res.uw_res[1] << 8) | res.res.ub_res[1]);
        arg_Pedo->usRunTime    = ((res.res.ub_res[8] << 24) | (res.res.uw_res[3] << 8) | res.res.ub_res[5]);
        }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_DST_GET_RUN1 err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
        }

    cmd.cmd.udata16 = HC_DST_GET_PEDO3;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usStExercise = res.res.ud_res[0];
        arg_Pedo->usStCal      = res.res.ud_res[1];
        arg_Pedo->usStBodyFat  = res.res.ud_res[2];
    }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_DST_GET_PEDO3 err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_PEDO4;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usSportExercise = res.res.ud_res[0];
    }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_DST_GET_PEDO4 err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_RUN2;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usRunCal      = res.res.ud_res[0];
        arg_Pedo->usRunExercise = res.res.ud_res[1];
    }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_DST_GET_RUN2 err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_PRE_GET_HEIGHT1;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usPS_InitBaseHeight = res.res.sd_res[0];
        arg_Pedo->usPS_ActHeight      = res.res.sd_res[1];
    }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_PRE_GET_HEIGHT1 err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_PRE_GET_HEIGHT2;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usPS_ActHeightMin = res.res.sd_res[0];
        arg_Pedo->usPS_ActHeightAve = res.res.sd_res[1];
        arg_Pedo->usPS_ActHeightMax = res.res.sd_res[2];
    }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_PRE_GET_HEIGHT2 err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

DBG(DBG_LV_INFO, "[Stability Step information]/[Additive information] ret %x\n", ret);
DBG(DBG_LV_DATA, " Step Count         : %u\n", arg_Pedo->usStepCnt );
DBG(DBG_LV_DATA, " Walking Time(10ms) : %u\n", arg_Pedo->usWalkTime );
DBG(DBG_LV_DATA, " Calorie(kcal)      : %u\n", arg_Pedo->usCal );
DBG(DBG_LV_DATA, " Body Fat(100mg)    : %u\n", arg_Pedo->usBodyFat );
DBG(DBG_LV_DATA, " Exercise(0.1Ex)       : %u\n", arg_Pedo->usExercise );
DBG(DBG_LV_DATA, " Mets               : %d\n", arg_Pedo->usMets );
DBG(DBG_LV_DATA, " Speedometer(cm/sec): %u\n", arg_Pedo->usSpeed );
DBG(DBG_LV_DATA, " Run Status         : %u\n", arg_Pedo->usRunStatus );
DBG(DBG_LV_DATA, " Run StepCnt        : %u\n", arg_Pedo->usRunStepCnt );
DBG(DBG_LV_DATA, " Run Time(10ms)     : %u\n", arg_Pedo->usRunTime );
DBG(DBG_LV_DATA, " Static Exercise(0.1Ex)   : %u\n", arg_Pedo->usStExercise );
DBG(DBG_LV_DATA, " Static Calorie(kcal)  : %u\n", arg_Pedo->usStCal );
DBG(DBG_LV_DATA, " Static Body Fat(100mg): %u\n", arg_Pedo->usStBodyFat );
DBG(DBG_LV_DATA, " Sport Exercise(0.1Ex)    : %u\n", arg_Pedo->usSportExercise );
DBG(DBG_LV_DATA, " Run Calorie(kcal)      : %u\n", arg_Pedo->usRunCal );
DBG(DBG_LV_DATA, " Run Exercise(0.1Ex)    : %u\n", arg_Pedo->usRunExercise );
DBG(DBG_LV_DATA, " PS InitBaseHeight(cm)  : %d\n", arg_Pedo->usPS_InitBaseHeight );
DBG(DBG_LV_DATA, " PS ActHeight(cm)       : %d\n", arg_Pedo->usPS_ActHeight );
DBG(DBG_LV_DATA, " PS ActHeightMin(cm)    : %d\n", arg_Pedo->usPS_ActHeightMin );
DBG(DBG_LV_DATA, " PS ActHeightAve(cm)    : %d\n", arg_Pedo->usPS_ActHeightAve );
DBG(DBG_LV_DATA, " PS ActHeightMax(cm)    : %d\n", arg_Pedo->usPS_ActHeightMax );

    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_pedom_clear(int32_t clear_req)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_DST_CLR_DAILYS;
	cmd.prm.ub_prm[0] = 0x01;
	cmd.prm.ub_prm[1] = (clear_req & DAILYS_CLEAR_PEDO_DATA)  ? 0x01:0x00;
	cmd.prm.ub_prm[2] = (clear_req & DAILYS_CLEAR_PEDO_STATE) ? 0x01:0x00;
	cmd.prm.ub_prm[3] = (clear_req & DAILYS_CLEAR_VEHI_DATA)  ? 0x01:0x00;
	cmd.prm.ub_prm[4] = (clear_req & DAILYS_CLEAR_VEHI_STATE) ? 0x01:0x00;
	cmd.prm.ub_prm[5] = (clear_req & DAILYS_CLEAR_WIFI_STATE) ? 0x01:0x00;
	cmd.prm.ub_prm[6] = (clear_req & DAILYS_CLEAR_HEIGHT_STATE) ? 0x01:0x00;
	cmd.prm.ub_prm[7] = (clear_req & DAILYS_CLEAR_HEIGHT_ALL) ? 0x01:0x00;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Pedometer Data Clear Error %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_vehicle_activate(bool arg_iEnable)
{
#if 0
acc_tmp
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_ERR;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    cmd.cmd.udata16 = HC_ACC_SET_TRANS;
    
    if(arg_iEnable == true){
        cmd.prm.ub_prm[0] = HC_VALID;
    }else{
        cmd.prm.ub_prm[0] = HC_INVALID;
    }
    
    cmd.prm.ub_prm[1] = 20;
    cmd.prm.ub_prm[2] = (g_acc_nv_param.trans_p.judge_time & 0xFF);
    cmd.prm.ub_prm[3] = ((g_acc_nv_param.trans_p.judge_time >> 8) & 0xFF);
    cmd.prm.ub_prm[4] = g_acc_nv_param.trans_p.calc_time;
    cmd.prm.ub_prm[5] = 0x14;
    cmd.prm.ub_prm[6] = 0x00;
    cmd.prm.ub_prm[7] = 0x00;
    cmd.prm.ub_prm[8] = 0x01;
    
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "vehicle Activate Error HC_ACC_SET_PEDO(-) err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = 0x105b;
    cmd.prm.ub_prm[0] = 0x00;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "vehicle getdata Error 0x105b(0x00) err %x\n", res.err.udata16);
    }
    cmd.prm.ub_prm[0] = 0x02;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "vehicle getdata Error 0x105b(0x02) err %x\n", res.err.udata16);
    }
    cmd.prm.ub_prm[0] = 0x04;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "vehicle getdata Error 0x105b(0x04) err %x\n", res.err.udata16);
    }

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
#endif
    return ACCSNS_RC_OK;
}

static int32_t accsns_vehicle_getdata(struct vehicle *arg_Vehi)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_DST_GET_TRANS1;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
		arg_Vehi->usVehiStatus     = res.res.ub_res[0];
            arg_Vehi->usVehiKind = res.res.ub_res[1];
		arg_Vehi->usVehiDetectTime = ((res.res.uw_res[2] << 16) | res.res.uw_res[1]);
		arg_Vehi->usVehiRideTime   = ((res.res.uw_res[4] << 16) | res.res.uw_res[3]);
    }else{
DBG(DBG_LV_ERROR, "Vehicle getdata Error HC_DST_GET_TRANS1 err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
    cmd.cmd.udata16 = HC_DST_GET_TRANS2;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
		arg_Vehi->usVehiRideCal  = res.res.ud_res[0];
		arg_Vehi->usVehiBodyFat  = res.res.ud_res[1];
		arg_Vehi->usVehiExercise = res.res.ud_res[2];
		arg_Vehi->usVehiMets     = res.res.ub_res[12];
        }else{
DBG(DBG_LV_ERROR, "Vehicle getdata Error HC_DST_GET_TRANS2 err %x\n", res.err.udata16);
		return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_TRANS3;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
		arg_Vehi->usVehiStExercise  = res.res.ud_res[0];
		arg_Vehi->usVehiStRideCal   = res.res.ud_res[1];
		arg_Vehi->usVehiStBodyFat   = res.res.ud_res[2];
    }else{
DBG(DBG_LV_ERROR, "Vehicle getdata Error HC_DST_GET_TRANS3 err %x\n", res.err.udata16);
		return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_TRANS4;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
		arg_Vehi->usVehiBiExercise  = res.res.ud_res[0];
		arg_Vehi->usVehiBiRideCal   = res.res.ud_res[1];
		arg_Vehi->usVehiBiBodyFat   = res.res.ud_res[2];
    }else{
DBG(DBG_LV_ERROR, "Vehicle getdata Error HC_DST_GET_TRANS4 err %x\n", res.err.udata16);
		return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_TRANS5;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
		arg_Vehi->usVehiSportExercise  = res.res.ud_res[0];
    }else{
DBG(DBG_LV_ERROR, "Vehicle getdata Error HC_DST_GET_TRANS5 err %x\n", res.err.udata16);
		return ACCSNS_RC_ERR;
    }

DBG(DBG_LV_INFO, "[Vehicle information]/[Additive information] ret %x\n", ret);
DBG(DBG_LV_DATA, " Vehi Status           : %u\n", arg_Vehi->usVehiStatus );
DBG(DBG_LV_DATA, " Vehi Kind             : %u\n", arg_Vehi->usVehiKind );
DBG(DBG_LV_DATA, " Vehi DetectTime(10ms) : %u\n", arg_Vehi->usVehiDetectTime );
DBG(DBG_LV_DATA, " Vehi RideTime(10ms)   : %u\n", arg_Vehi->usVehiRideTime );
DBG(DBG_LV_DATA, " Vehi RideCal(kcal)    : %u\n", arg_Vehi->usVehiRideCal );
DBG(DBG_LV_DATA, " Vehi BodyFat(100mg)   : %d\n", arg_Vehi->usVehiBodyFat );
DBG(DBG_LV_DATA, " Vehi Exercise(Ex)     : %u\n", arg_Vehi->usVehiExercise );
DBG(DBG_LV_DATA, " Vehi Mets             : %u\n", arg_Vehi->usVehiMets );
DBG(DBG_LV_DATA, " Vehi Static Exercise(Ex)     : %u\n", arg_Vehi->usVehiStExercise );
DBG(DBG_LV_DATA, " Vehi Static RideCal(kcal)    : %u\n", arg_Vehi->usVehiStRideCal );
DBG(DBG_LV_DATA, " Vehi Static BodyFat(100mg)   : %d\n", arg_Vehi->usVehiStBodyFat );
DBG(DBG_LV_DATA, " Vehi Bicycle Exercise(Ex)     : %u\n", arg_Vehi->usVehiBiExercise );
DBG(DBG_LV_DATA, " Vehi Bicycle RideCal(kcal)    : %u\n", arg_Vehi->usVehiBiRideCal );
DBG(DBG_LV_DATA, " Vehi Bicycle BodyFat(100mg)   : %d\n", arg_Vehi->usVehiBiBodyFat );
DBG(DBG_LV_DATA, " Vehi Sport Exercise(Ex)     : %u\n", arg_Vehi->usVehiSportExercise );
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_iwifi_getdata(struct iwifi *arg_IWifi)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_DST_GET_INTELLI_WIFI;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
		arg_IWifi->usPedoStatus = res.res.ub_res[0];
        arg_IWifi->usVehiStatus = res.res.ub_res[1];
    }else{
DBG(DBG_LV_ERROR, "IWifi getdata Error HC_DST_GET_INTELLI_WIFI err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
DBG(DBG_LV_INFO, "[Intelle WiFi information]/[Additive information] ret %x\n", ret);
DBG(DBG_LV_DATA, " Pedo Status           : %u\n", arg_IWifi->usPedoStatus );
DBG(DBG_LV_DATA, " Vehi Status           : %u\n", arg_IWifi->usVehiStatus );
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_measure(int32_t arg_iSensType)
{
    int32_t  ret = ACCSNS_RC_OK;
    struct pedometer     arg_Pedo;
    struct acceleration  arg_Acc;
    struct vehicle       arg_Vehi;
	struct pressure      arg_Pre;
	struct iwifi         arg_IWifi;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_LOW, "     Enable = %04x, IrqFlg = %04x GPIO_INT = %04x \n", (int)atomic_read(&g_bIsIntIrqEnable), g_nIntIrqFlg, gpio_get_value(ACCSNS_GPIO_INT));
DBG(DBG_LV_INFO, "arg_iSensType %x\n", arg_iSensType);
    
    memset(&arg_Acc,  0x00, sizeof(arg_Acc));
    memset(&arg_Pedo, 0x00, sizeof(arg_Pedo));
    memset(&arg_Vehi, 0x00, sizeof(arg_Vehi));
	memset(&arg_Pre,  0x00, sizeof(arg_Pre));
	memset(&arg_IWifi,  0x00, sizeof(arg_IWifi));
    
    if(arg_iSensType & ACCSNS_ACTIVE_ACC){

        ret = accsns_acc_getdata(&arg_Acc);
        mutex_lock(&s_tDataMutex);
        if(ret == ACCSNS_RC_OK){
            s_tLatestAccData.nX = arg_Acc.nX;
            s_tLatestAccData.nY = arg_Acc.nY;
            s_tLatestAccData.nZ = arg_Acc.nZ;
            s_tLatestAccData.outX = arg_Acc.outX;
            s_tLatestAccData.outY = arg_Acc.outY;
            s_tLatestAccData.outZ = arg_Acc.outZ;
DBG(DBG_LV_DATA, "Acc Data get . outX %x, outY %x, outZ %x\n",s_tLatestAccData.outX, s_tLatestAccData.outY, s_tLatestAccData.outZ);
        }else{
            memset(&s_tLatestAccData, 0x00, sizeof(s_tLatestAccData));
        }
        mutex_unlock(&s_tDataMutex);
    }

    if(arg_iSensType & ACCSNS_ACTIVE_PEDOM){
    
        ret = accsns_pedom_getdata(&arg_Pedo);
        mutex_lock(&s_tDataMutex);
        if(ret == ACCSNS_RC_OK){
            s_tLatestPedoData.usStepCnt  = arg_Pedo.usStepCnt;
            s_tLatestPedoData.usWalkTime = arg_Pedo.usWalkTime;
            s_tLatestPedoData.usCal      = arg_Pedo.usCal;
            s_tLatestPedoData.usBodyFat  = arg_Pedo.usBodyFat;
            s_tLatestPedoData.usExercise   = arg_Pedo.usExercise;
            s_tLatestPedoData.usMets       = arg_Pedo.usMets;
            s_tLatestPedoData.usSpeed    = arg_Pedo.usSpeed;
            s_tLatestPedoData.usRunStatus  = arg_Pedo.usRunStatus;
            s_tLatestPedoData.usRunStepCnt = arg_Pedo.usRunStepCnt;
			s_tLatestPedoData.usRunTime    = arg_Pedo.usRunTime;
			s_tLatestPedoData.usStExercise = arg_Pedo.usStExercise;
			s_tLatestPedoData.usStCal      = arg_Pedo.usStCal;
			s_tLatestPedoData.usStBodyFat  = arg_Pedo.usStBodyFat;
			s_tLatestPedoData.usSportExercise = arg_Pedo.usSportExercise;
			s_tLatestPedoData.usRunCal      = arg_Pedo.usRunCal;
			s_tLatestPedoData.usRunExercise = arg_Pedo.usRunExercise;
			s_tLatestPedoData.usPS_InitBaseHeight = arg_Pedo.usPS_InitBaseHeight;
			s_tLatestPedoData.usPS_ActHeight    = arg_Pedo.usPS_ActHeight;
			s_tLatestPedoData.usPS_ActHeightMin = arg_Pedo.usPS_ActHeightMin;
			s_tLatestPedoData.usPS_ActHeightAve = arg_Pedo.usPS_ActHeightAve;
			s_tLatestPedoData.usPS_ActHeightMax = arg_Pedo.usPS_ActHeightMax;

DBG(DBG_LV_DATA, "Pedo Data get . usStepCnt %x, usWalkTime %x, usCal %x, usBodyFat %x, usExercise %x\n"
                ,s_tLatestPedoData.usStepCnt, s_tLatestPedoData.usWalkTime, s_tLatestPedoData.usCal, 
				s_tLatestPedoData.usBodyFat, s_tLatestPedoData.usExercise);
DBG(DBG_LV_DATA, "Pedo Data get . usMets %x, usSpeed %x, usRunStatus %x, usRunStepCnt %x, usRunTime %x\n"
                ,s_tLatestPedoData.usMets, s_tLatestPedoData.usSpeed, s_tLatestPedoData.usRunStatus,
				 s_tLatestPedoData.usRunStepCnt, s_tLatestPedoData.usRunTime);
DBG(DBG_LV_DATA, "Pedo Data get . usStExercise %x, usStCal %x, usStBodyFat %x, usSportExercise %x\n"
                ,s_tLatestPedoData.usStExercise, s_tLatestPedoData.usStCal, s_tLatestPedoData.usStBodyFat,
					s_tLatestPedoData.usSportExercise);
DBG(DBG_LV_DATA, "Pedo Data get . usRunCal %x, usRunExercise %x, usPS_InitBaseHeight %x, usPS_ActHeight %x\n"
                ,s_tLatestPedoData.usRunCal, s_tLatestPedoData.usRunExercise, s_tLatestPedoData.usPS_InitBaseHeight,
					s_tLatestPedoData.usPS_ActHeight);
DBG(DBG_LV_DATA, "Pedo Data get . usPS_ActHeightMin %x, usPS_ActHeightAve %x, usPS_ActHeightMax %x\n"
                ,s_tLatestPedoData.usPS_ActHeightMin, s_tLatestPedoData.usPS_ActHeightAve, s_tLatestPedoData.usPS_ActHeightMax);

        }else{
            memset(&s_tLatestPedoData, 0x00, sizeof(s_tLatestPedoData));
        }
        mutex_unlock(&s_tDataMutex);
    }

    if(arg_iSensType & ACCSNS_ACTIVE_VEHICLE){
    
        ret = accsns_vehicle_getdata(&arg_Vehi);
        mutex_lock(&s_tDataMutex);
        if(ret == ACCSNS_RC_OK){
            s_tLatestVehiData.usVehiStatus   = arg_Vehi.usVehiStatus;
            s_tLatestVehiData.usVehiKind     = arg_Vehi.usVehiKind;
			s_tLatestVehiData.usVehiDetectTime = arg_Vehi.usVehiDetectTime;
            s_tLatestVehiData.usVehiRideTime = arg_Vehi.usVehiRideTime;
			s_tLatestVehiData.usVehiRideCal    = arg_Vehi.usVehiRideCal;
			s_tLatestVehiData.usVehiBodyFat    = arg_Vehi.usVehiBodyFat;
			s_tLatestVehiData.usVehiExercise   = arg_Vehi.usVehiExercise;
			s_tLatestVehiData.usVehiMets       = arg_Vehi.usVehiMets;
			s_tLatestVehiData.usVehiStExercise  = arg_Vehi.usVehiStExercise;
			s_tLatestVehiData.usVehiStRideCal   = arg_Vehi.usVehiStRideCal;
			s_tLatestVehiData.usVehiStBodyFat   = arg_Vehi.usVehiStBodyFat;
			s_tLatestVehiData.usVehiBiExercise  = arg_Vehi.usVehiBiExercise;
			s_tLatestVehiData.usVehiBiRideCal   = arg_Vehi.usVehiBiRideCal;
			s_tLatestVehiData.usVehiBiBodyFat   = arg_Vehi.usVehiBiBodyFat;
			s_tLatestVehiData.usVehiSportExercise = arg_Vehi.usVehiSportExercise;
DBG(DBG_LV_DATA, "vehicle Data get . usVehiStatus %x usVehiKind %x usVehiDetectTime %x usVehiRideTime %x\n",
					s_tLatestVehiData.usVehiStatus, s_tLatestVehiData.usVehiKind, 
					s_tLatestVehiData.usVehiDetectTime, s_tLatestVehiData.usVehiRideTime);
DBG(DBG_LV_DATA, "vehicle Data get . usVehiRideCal %x usVehiBodyFat %x usVehiExercise %x usVehiMets %x\n",
					s_tLatestVehiData.usVehiRideCal, s_tLatestVehiData.usVehiBodyFat, 
					s_tLatestVehiData.usVehiExercise, s_tLatestVehiData.usVehiMets);
DBG(DBG_LV_DATA, "vehicle Data get . usVehiStExercise %x usVehiStRideCal %x usVehiStBodyFat %x\n",
					s_tLatestVehiData.usVehiStExercise, s_tLatestVehiData.usVehiStRideCal, 
					s_tLatestVehiData.usVehiStBodyFat);
DBG(DBG_LV_DATA, "vehicle Data get . usVehiBiExercise %x usVehiBiRideCal %x usVehiBiBodyFat %x usVehiSportExercise %x\n",
					s_tLatestVehiData.usVehiBiExercise, s_tLatestVehiData.usVehiBiRideCal, 
					s_tLatestVehiData.usVehiBiBodyFat, s_tLatestVehiData.usVehiSportExercise);

        }else{
            memset(&s_tLatestVehiData, 0x00, sizeof(s_tLatestVehiData));
        }
        mutex_unlock(&s_tDataMutex);
    }

    if(arg_iSensType & PRESNS_ACTIVE_PRE){

        ret = presns_pre_getdata(&arg_Pre);
        mutex_lock(&s_tDataMutex);
        if(ret == ACCSNS_RC_OK){
            s_tLatestPresData.usHeight   = arg_Pre.usHeight;
            s_tLatestPresData.usTemp     = arg_Pre.usTemp;
            s_tLatestPresData.usPressure = arg_Pre.usPressure;
			s_tLatestPresData.usOutPressure = arg_Pre.usPressure;
			s_tLatestPresData.usAccuracy = arg_Pre.usAccuracy;
DBG(DBG_LV_DATA, "Pre Data get . Height %x, Temp %x, Pressure %x OutPressure %x Accuracy%x\n",
					s_tLatestPresData.usHeight, s_tLatestPresData.usTemp, s_tLatestPresData.usPressure,
					s_tLatestPresData.usOutPressure,s_tLatestPresData.usAccuracy);
        }else{
            memset(&s_tLatestPresData, 0x00, sizeof(s_tLatestPresData));
        }
        mutex_unlock(&s_tDataMutex);
    }

    if(arg_iSensType & ACCSNS_ACTIVE_IWIFI){

        ret = accsns_iwifi_getdata(&arg_IWifi);
        mutex_lock(&s_tDataMutex);
        if(ret == ACCSNS_RC_OK){
			s_tLatestIWifiData.usPedoStatus = arg_IWifi.usPedoStatus;
			s_tLatestIWifiData.usVehiStatus = arg_IWifi.usVehiStatus;
DBG(DBG_LV_DATA, "Intelli Wifi get . PedoStatus %d usVehiStatus %d\n",
				s_tLatestIWifiData.usPedoStatus,s_tLatestIWifiData.usVehiStatus);
        }else{
            memset(&s_tLatestIWifiData, 0x00, sizeof(s_tLatestIWifiData));
        }
        mutex_unlock(&s_tDataMutex);
    }

DBG_PRINT_IO(0xFF, ret);
DBG(DBG_LV_LOW, "     Enable = %04x, IrqFlg = %04x GPIO_INT = %04x \n", (int)atomic_read(&g_bIsIntIrqEnable), g_nIntIrqFlg, gpio_get_value(ACCSNS_GPIO_INT));
  return ret;
}

static int32_t accsns_initialize( void )
{
    uint8_t fw_ver[4];
    uint8_t reg = 0xFF;
    int32_t cnt;
    int32_t ret = ACCSNS_RC_OK;
    int32_t iWakeupSensor;
    HostCmd cmd;
    HostCmdRes res;
    Word sreg;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "accsns_initialize:register_init\n");

    DISABLE_IRQ;
    
    accsns_workqueue_init();

    atomic_set(&g_flgEna, ACTIVE_OFF);
    atomic_set(&g_CurrentSensorEnable,ACTIVE_OFF);
    iWakeupSensor = atomic_read(&g_WakeupSensor);
    atomic_set(&g_WakeupSensor,iWakeupSensor & ~ACTIVE_FUNC_MASK);
    g_bDevIF_Error = false;
    mutex_lock(&s_tDataMutex);
    
    memset(&s_tLatestAccData, 0x00, sizeof(s_tLatestAccData));
    memset(&s_tLatestPedoData, 0x00, sizeof(s_tLatestPedoData));
    memset(&s_tLatestVehiData, 0x00, sizeof(s_tLatestVehiData));
	memset(&s_tLatestPresData, 0x00, sizeof(s_tLatestPresData));
	memset(&s_tLatestIWifiData, 0x00, sizeof(s_tLatestIWifiData));
    mutex_unlock(&s_tDataMutex);
    
    gpio_set_value(ACCSNS_GPIO_RST, 0);
    udelay(300);
    gpio_set_value(ACCSNS_GPIO_RST, 1);

    msleep(260);
    
    cnt = 0;
    while(1) {
        accsns_device_read(STATUS, &reg, sizeof(reg));
        if(reg == 0x00) {
DBG(DBG_LV_INFO, "STATUS OK!!\n");
            break;
        }
        
        msleep(10);
        ++cnt;
        if(cnt > STATUS_READ_RETRY_NUM) {
DBG(DBG_LV_ERROR, "[ACC] accsns_initialize:STATUS read TimeOut. reg=%x \n", reg);
            return ACCSNS_RC_ERR_TIMEOUT;
        }
    }
    
    reg = 0x04;
    accsns_device_write(CFG, &reg, sizeof(reg));

    accsns_device_read(INTREQ0, sreg.udata8, 2);

    reg = 0x00;
    accsns_device_write(INTMASK0, &reg, sizeof(reg));
    accsns_device_write(INTMASK1, &reg, sizeof(reg));
    
    ENABLE_IRQ;
    
    cmd.cmd.udata16 = HC_MCU_SET_PCON;
    cmd.prm.ub_prm[0] = 0x00;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x01;
    cmd.prm.ub_prm[5] = 0x01;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "accsns_initialize:CMD Error <HC_MCU_SET_PCON> err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
    ret = accsns_check_accsensor();
    if(ret != 0) {
DBG(DBG_LV_ERROR, "error(accsns_check_accsensor) : accsns_initialize\n" );
    }
    
    ret = accsns_get_fw_version(fw_ver);
    g_nAccFWVersion = ACC_FW_VERSION_GET_32BIT(fw_ver);
    if(ret != ACCSNS_RC_OK ){
        g_nAccFWVersion = ACC_FW_VERSION_NONE;
DBG(DBG_LV_ERROR, "[ACC] accsns_get_fw_version:Version not get.\n");
    }
DBG(ACCSNS_RC_ERR, "[ACC] Sensor FW Version.%08x \n", g_nAccFWVersion);

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_hostcmd(const HostCmd *prm, HostCmdRes *res, uint8_t mode)
{
    int32_t ret;
    uint8_t reg[16];
    
DBG(DBG_LV_LOW, "AccSem Get\n");
    down(&s_tAccSnsrSema);
    
    reg[0]  = prm->cmd.udata8[0];
    reg[1]  = prm->cmd.udata8[1];
    memcpy(&reg[2], &prm->prm.ub_prm[0], sizeof(prm->prm.ub_prm));
    reg[15] = 1;

DBG(DBG_LV_INFO, "[ACC]<%s>:Start HostCmd :reg[%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x] mode %x\n",__FUNCTION__
    ,reg[0] ,reg[1] ,reg[2] ,reg[3] ,reg[4] ,reg[5] ,reg[6] ,reg[7] ,reg[8] ,reg[9] ,reg[10] 
    ,reg[11] ,reg[12] ,reg[13] ,reg[14] ,reg[15] 
    , mode);
    
    mutex_lock(&s_tDataMutex);
    g_nIntIrqFlg = 0;
    mutex_unlock(&s_tDataMutex);

    ret = accsns_device_write(CMD0, reg, sizeof(reg));
    if(ret == ACCSNS_RC_OK){
        
        if((mode & EXE_HOST_WAIT) == EXE_HOST_WAIT){
            ret |= accsns_waitcmd(INTREQ_HOST_CMD);
            if(ret != ACCSNS_RC_OK) {
                if( ((mode & EXE_HOST_EX_NO_RECOVER) == EXE_HOST_EX_NO_RECOVER) && 
                    (ret == ACCSNS_RC_ERR_TIMEOUT) ){
DBG(DBG_LV_ERROR, "SPI HostCmd error(accsns_waitcmd):F/W Self Checking...(No Recovery)\n");
                    up(&s_tAccSnsrSema);
DBG(DBG_LV_LOW, "AccSem Post\n");
                    return ACCSNS_RC_ERR_TIMEOUT;
                }
                accsns_timeout_dump(reg);
               
                up(&s_tAccSnsrSema);
DBG(DBG_LV_LOW, "DBG_LV_INFO Post\n");
DBG(DBG_LV_ERROR, "SPI HostCmd error(accsns_waitcmd)\n");
                g_bDevIF_Error = true;
                /* acc_tmp ERR_WAKEUP; */
                
                return ret;
            }
        }

        if((mode & EXE_HOST_RES) == EXE_HOST_RES){
            ret |= accsns_device_read(RSLT00, res->res.ub_res, 31);
            if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "SPI HostCmd error(RSLT00)\n");
            }
        }

        if((mode & EXE_HOST_ERR) == EXE_HOST_ERR){
            ret |= accsns_device_read(ERROR0, res->err.udata8, 2);
            if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "SPI HostCmd error(ERROR0)\n");
            }
        }
    }else{
DBG(DBG_LV_ERROR, "SPI HostCmd error(ret %x)", ret);
        g_bDevIF_Error = true;
    }
    
DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    
    up(&s_tAccSnsrSema);
DBG(DBG_LV_LOW, "AccSem Post\n");
    
    return ret;
}

static int32_t accsns_waitcmd(uint16_t intBit)
{
    int32_t ret = ACCSNS_RC_ERR_TIMEOUT;
    int32_t result = 0;
    long timeout;
    int32_t retry = 300;
DBG_PRINT_IO(0, 0);
    
    timeout = msecs_to_jiffies(WAITEVENT_TIMEOUT);

    while(retry){
    
        result = wait_event_interruptible_timeout(s_tWaitInt, (g_nIntIrqFlg & (INTREQ_HOST_CMD | INTREQ_ERROR)), timeout);
        if( g_nIntIrqFlg & INTREQ_ERROR ){
            mutex_lock(&s_tDataMutex);
            g_nIntIrqFlg &= ~INTREQ_ERROR;
            mutex_unlock(&s_tDataMutex);
            
DBG(DBG_LV_INT, "INTREQ0/1 -Error- \n");
            ret = ACCSNS_RC_ERR;
            break;

        }else if( g_nIntIrqFlg & INTREQ_HOST_CMD ){
            mutex_lock(&s_tDataMutex);
            g_nIntIrqFlg &= ~INTREQ_HOST_CMD;
            mutex_unlock(&s_tDataMutex);
            
            ret = ACCSNS_RC_OK;
DBG(DBG_LV_INFO, "Wakeup Event... \n");
            break;
        }
        if( result == -ERESTARTSYS ) {
DBG(DBG_LV_INFO, "wait event signal received. retry = %d, g_nIntIrqFlg = %x \n", retry, g_nIntIrqFlg);
			msleep(10);
        }

        if( result == 0 ){
            ret = ACCSNS_RC_ERR_TIMEOUT;
DBG(DBG_LV_ERROR, "wait event timeout... %x \n", g_nIntIrqFlg);
            break;
        }
        retry--;
    }

DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}

static irqreturn_t accsns_irq_handler(int32_t irq, void *dev_id)
{
DBG(DBG_LV_LOW, "### accsns_irq_handler In \n");

    if( irq != g_nIntIrqNo ){
        return IRQ_NONE;
    }

    DISABLE_IRQ;
    if( accsns_workqueue_create(accsns_wq_int, accsns_int_work_func) != ACCSNS_RC_OK){
        ENABLE_IRQ;
        
    }else{
DBG(DBG_LV_INT, "### --> s_tWork_Int \n");
    }

    return IRQ_HANDLED;
}

static void accsns_int_work_func(struct work_struct *work)
{
    Word sreg;
    int32_t iCurrentEnable;
    
    memset( &sreg, 0x00, sizeof(sreg));
    
    iCurrentEnable = atomic_read(&g_CurrentSensorEnable);

    accsns_device_read(INTREQ0, sreg.udata8, 2);

    if(sreg.udata16 == 0){
        accsns_workqueue_delete(work) ;
        ENABLE_IRQ;
        return;
    }
DBG(DBG_LV_INT, "### INTREQ0/1=%x iCurrentEnable=%x \n", sreg.udata16, iCurrentEnable);
    
    if(sreg.udata16 & INTREQ_ERROR){
DBG(DBG_LV_ERROR, "### accsns_int_work_func Error %x\n", sreg.udata16);
        mutex_lock(&s_tDataMutex);
        g_nIntIrqFlg |= INTREQ_ERROR;
        mutex_unlock(&s_tDataMutex);
        
        wake_up_interruptible(&s_tWaitInt);
        
        accsns_workqueue_delete(work) ;
        ENABLE_IRQ;
        return;
    }
    
    if(sreg.udata16 & INTREQ_HOST_CMD){
        if(!(g_nIntIrqFlg & INTREQ_HOST_CMD)){
            mutex_lock(&s_tDataMutex);
            g_nIntIrqFlg |= INTREQ_HOST_CMD;
            mutex_unlock(&s_tDataMutex);
            wake_up_interruptible(&s_tWaitInt);
        }
DBG(DBG_LV_INT, "### accsns_int_work_func INTREQ_HOST_CMD g_nIntIrqFlg:%x \n", g_nIntIrqFlg);
    }
    
    if(sreg.udata16 & INTREQ_ACC){
DBG(DBG_LV_INT, "### accsns_int_work_func INTREQ_ACC iCurrentEnable:%x \n", iCurrentEnable);

        if( (iCurrentEnable & ACCSNS_ACTIVE_VEHICLE) ||
            (iCurrentEnable & ACCSNS_ACTIVE_PEDOM) ){
            accsns_workqueue_create(accsns_wq, accsns_int_acc_work_func);
        }
    }
    
    accsns_workqueue_delete(work) ;
    ENABLE_IRQ;
    return;
}

static void accsns_int_acc_work_func(struct work_struct *work)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t iCurrentEnable;
    int32_t iWakeupSensor;
    int32_t ret = ACCSNS_RC_OK;
    
DBG(DBG_LV_INT, "### accsns_int_acc_work_func In \n");
    
    cmd.cmd.udata16 = HC_DST_GET_INT_DETAIL;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "### accsns_int_acc_work_func Error HC_DST_GET_INT_DETAIL\n");
        accsns_workqueue_delete(work) ;
        return;
    }

    iCurrentEnable = atomic_read(&g_CurrentSensorEnable);
    
DBG(DBG_LV_INT, "### accsns_int_acc_work_func HC_DST_GET_INT_DETAIL iCurrentEnable:%x ret:%x res[0]:%x res[1]:%x res[2]:%x res[3]:%x res[4]:%x\n"
    ,iCurrentEnable, ret, res.res.ub_res[0], res.res.ub_res[1], res.res.ub_res[2], res.res.ub_res[3], res.res.ub_res[4]);

    if(res.res.ub_res[1] == 0x01){
        if(iCurrentEnable & ACCSNS_ACTIVE_VEHICLE){
DBG(DBG_LV_INT, "### *** Vehicle Detected !! \n");
            iWakeupSensor = atomic_read(&g_WakeupSensor);
            atomic_set(&g_WakeupSensor,iWakeupSensor | ACCSNS_ACTIVE_VEHICLE);
            wake_up_interruptible(&s_tPollWaitVehicle);

        }
    }

    if(res.res.ub_res[3] == 0x01){
        if(iCurrentEnable & ACCSNS_ACTIVE_DAILYS){
DBG(DBG_LV_INT, "### *** Intelli Wifi Detected !! \n");
            iWakeupSensor = atomic_read(&g_WakeupSensor);
            atomic_set(&g_WakeupSensor,iWakeupSensor | ACCSNS_ACTIVE_IWIFI);
            wake_up_interruptible(&s_tPollWaitIWifi);
        }
    }

    cmd.cmd.udata16 = HC_DST_CLR_INT_DETAIL;
	cmd.prm.ub_prm[0] = 0x01;
	cmd.prm.ub_prm[1] = 0x00;
	cmd.prm.ub_prm[2] = 0x01;
	cmd.prm.ub_prm[3] = 0x00;
	cmd.prm.ub_prm[4] = 0x01;
	cmd.prm.ub_prm[5] = 0x00;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "### accsns_int_acc_work_func Error HC_DST_CLR_INT_DETAIL\n");
        accsns_workqueue_delete(work) ;
        return;
    }

    accsns_workqueue_delete(work) ;
    return;
}

static int32_t accsns_gpio_init(void)
{
    int32_t ret;
DBG_PRINT_IO(0, 0);

    ret = gpio_request(ACCSNS_GPIO_RST, ACC_GPIO_RESET_NAME);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        return ret;
    }
    
    ret = gpio_direction_output(ACCSNS_GPIO_RST, 1);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        goto ERROR;
    }
    
    ret = gpio_request(ACCSNS_GPIO_TEST0, ACC_GPIO_TEST0_NAME);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        goto ERROR;
    }
    
    ret = gpio_direction_output(ACCSNS_GPIO_TEST0, 0);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        goto ERROR;
    }
    
    g_nIntIrqNo = MSM_GPIO_TO_INT(ACCSNS_GPIO_INT);
    atomic_set(&g_bIsIntIrqEnable, true);
    ret = gpio_request(ACCSNS_GPIO_INT, ACC_GPIO_INT_NAME);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        goto ERROR;
    }
    
    ret = gpio_direction_input(ACCSNS_GPIO_INT);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request1 ret=%d\n",ret);
        goto ERROR;
    }

    ret = request_any_context_irq(g_nIntIrqNo, accsns_irq_handler, IRQF_TRIGGER_LOW, ACC_GPIO_INT_NAME, NULL);
    if(ret < 0) {
DBG(DBG_LV_ERROR, "Failed gpio_request. ret=%x\n", ret);
        goto ERROR;
    }
DBG(DBG_LV_INT, "g_nIntIrqNo=%d\n",g_nIntIrqNo);
    
DBG_PRINT_IO(0xFF, ret);
    return ACCSNS_RC_OK;
    
ERROR:
    gpio_free(ACCSNS_GPIO_INT);
    gpio_free(ACCSNS_GPIO_RST);
    gpio_free(ACCSNS_GPIO_TEST0);
    return -ENODEV;
}
    
static int32_t accsns_check_accsensor(void)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_MCU_GET_EX_SENSOR;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "[ACC] accsns_check_accsensor:CMD Error <HC_MCU_GET_EX_SENSOR> err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    if( !(res.res.uw_res[0] & 0x01) ){
DBG(DBG_LV_ERROR, "[ACC] accsns_check_accsensor:Acc Sensor Not found[%x].\n", res.res.uw_res[0]);
        return ACCSNS_RC_ERR;
    }
    if( !(res.res.uw_res[0] & 0x20) ){
DBG(DBG_LV_ERROR, "[ACC] accsns_check_accsensor:Pre Sensor Not found[%x].\n", res.res.uw_res[0]);
        return ACCSNS_RC_ERR;
    }
DBG(DBG_LV_INFO, "Sensor Existed.%x \n", res.res.uw_res[0]);
    
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}
 


static int32_t accsns_get_fw_version(uint8_t *arg_iData)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;

    ret = accsns_hostcmd(&cmd, &res, (EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER));
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "FW Version Get Error HC_MCU_GET_VERSION(%d) err %x\n",cmd.prm.ub_prm[0], res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
    arg_iData[0] = res.res.ub_res[0];
    arg_iData[1] = res.res.ub_res[1];
    arg_iData[2] = res.res.ub_res[2];
    arg_iData[3] = res.res.ub_res[3];
    
DBG(DBG_LV_INFO, "FW Version=%02x %02x %02x %02x\n", arg_iData[0], arg_iData[1], arg_iData[2], arg_iData[3]);
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static void accsns_FW_TEST0_ctrl(void)
{
DBG(DBG_LV_ERROR, "accsns_update_fw_seq: TEST0 Ctro Start... \n");

    gpio_set_value(ACCSNS_GPIO_RST, 0);
    
    gpio_set_value(ACCSNS_GPIO_TEST0, 1);
    
    udelay(55);
    
    gpio_set_value(ACCSNS_GPIO_RST, 1);
    
    msleep(560);
    
    gpio_set_value(ACCSNS_GPIO_TEST0, 0);
}

static int32_t accsns_update_fw_seq(uint8_t *arg_iData, uint32_t arg_iLen)
{
    int32_t    ret = ACCSNS_RC_OK;
    uint8_t    fw_ver[4];
    HostCmd    cmd;
    HostCmdRes res;
	int32_t    i;
DBG_PRINT_IO(0, 0);
    
    atomic_set(&g_FWUpdateStatus,true);
    
    ret = accsns_get_fw_version(fw_ver);
    g_nAccFWVersion = ACC_FW_VERSION_GET_32BIT(fw_ver);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_update_fw_seq:Get Version Error!!\n");
        g_nAccFWVersion = ACC_FW_VERSION_NONE;
    }
    
DBG(DBG_LV_ERROR, "accsns_update_fw_seq:Now=%x, Base:%x \n",g_nAccFWVersion, ACC_FW_VERSION_DATA);
    
    if(g_nAccFWVersion != ACC_FW_VERSION_DATA){
DBG(DBG_LV_ERROR, "[ACC] accsns_update_fw_seq:Need to update F/W Version\n");
        
        cmd.cmd.udata16 = HC_MCU_SELF_CHK_FW;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER);
DBG(DBG_LV_ERROR, "accsns_update_fw_seq: HC_MCU_SELF_CHK_FW(%d) err res=%x err=%x\n", ret, res.res.ub_res[0], res.err.udata16);
        if(1) {
DBG(DBG_LV_ERROR, "[ACC] accsns_update_fw_seq: F/W Update Start... \n");
			DISABLE_IRQ;
			for(i = 0; i < ACC_SPI_RETRY_NUM; i++){
			accsns_FW_TEST0_ctrl();
			ret = accsns_update_fw(true, arg_iData, arg_iLen);
				if(ret != ACCSNS_RC_ERR_RAMWRITE)
					break;
			}
        }else if(0x00 == res.res.ub_res[0]){
DBG(DBG_LV_ERROR, "accsns_update_fw_seq: HC_MCU_SELF_CHK_FW(-) OK\n");
        }
        
    }else{
DBG(DBG_LV_INFO, "accsns_update_fw_seq:None update F/W Version\n");
    }
DBG(DBG_LV_INFO, "accsns_update_fw_seq: F/W Update Check Completed... \n");
    
DBG(DBG_LV_INFO, "accsns_update_fw_seq: F/W Initialize Start... \n");
    ret |= accsns_initialize();
        
	ret |= gpio_direction_input(ACCSNS_GPIO_TEST0);

    atomic_set(&g_FWUpdateStatus,false);
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_update_fw(bool boot, uint8_t *arg_iData, uint32_t arg_iLen)
{
    uint8_t reg = 0xFF;
    Word    sreg;
    int32_t i;
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;
	int32_t size;
	int32_t send_num = 0;
    uint8_t    fw_ver[4];
	int32_t    FWSoftVersion;
    
DBG_PRINT_IO(0, 0);
    

    memset(&res, 0x00, sizeof(HostCmdRes));

DBG(DBG_LV_INFO, "### Start Firmware Update ### boot=%d, Data=%x Len=%d \r\n", boot, (int)arg_iData, arg_iLen);
    
    if((arg_iData == NULL) || (arg_iLen == 0)){
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    
    if(!boot){
        ENABLE_IRQ;
        
        cmd.cmd.udata16 = HC_MCU_FUP_START;
        cmd.prm.ub_prm[0] = 0x55;
        cmd.prm.ub_prm[1] = 0xAA;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_WAIT|EXE_HOST_ERR);
        if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }
        if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
DBG(DBG_LV_ERROR, "Certification Error!\r\n");
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }

    }else{
        reg = 0x04;
        accsns_device_write(CFG, &reg, sizeof(reg));

        accsns_device_read(INTREQ0, sreg.udata8, 2);

        reg = 0x00;
        accsns_device_write(INTMASK0, &reg, sizeof(reg));
        accsns_device_write(INTMASK1, &reg, sizeof(reg));
        
        ENABLE_IRQ;
        
        cmd.cmd.udata16 = HC_MCU_SET_PCON;
        cmd.prm.ub_prm[0] = 0x00;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x00;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x01;
        cmd.prm.ub_prm[5] = 0x01;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "[ACC] PortSettint err %x\n", res.err.udata16);
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }
    }

DBG(DBG_LV_INFO, "Check Firmware Mode.\r\n");
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    if(res.res.ub_res[2] != 0x01){
DBG(DBG_LV_ERROR, "Version check Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }

DBG(DBG_LV_INFO, "Flash Clear.\r\n");
    cmd.cmd.udata16 = HC_MCU_FUP_ERASE;
    cmd.prm.ub_prm[0] = 0xAA;
    cmd.prm.ub_prm[1] = 0x55;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_WAIT|EXE_HOST_ERR);
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
DBG(DBG_LV_ERROR, "Certification Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }

    ret = accsns_get_fw_version(fw_ver);
    FWSoftVersion = ACC_FW_VERSION_GET_32BIT(fw_ver);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_update_fw:Get Version Error!!\n");
        FWSoftVersion = ACC_FW_VERSION_NONE;
    }
DBG(DBG_LV_INFO, "accsns_update_fw:FWSoftVersion=%x\n",FWSoftVersion);

DBG(DBG_LV_INFO, "Flash Write.\r\n");
	if(FWSoftVersion >= ACC_FW_VERSION_FIFOUPDATE){
DBG(DBG_LV_INFO, "FIFO UPDATE.\r\n");
		send_num = arg_iLen / FUP_MAX_RAMSIZE;
		send_num = (arg_iLen % FUP_MAX_RAMSIZE) ? send_num+1 : send_num;

		for(i=0; i < send_num; i++){
			if((arg_iLen - (FUP_MAX_RAMSIZE * i)) >= FUP_MAX_RAMSIZE){
				size = FUP_MAX_RAMSIZE;
			} else {
				size = arg_iLen % FUP_MAX_RAMSIZE;
			}
#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
			ret = accsns_i2c_ram_write_proc(RSLT20, &arg_iData[i * FUP_MAX_RAMSIZE], size);
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
			ret = accsns_spi_ram_write_proc(RSLT20, &arg_iData[i * FUP_MAX_RAMSIZE], size);
#endif
			if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "RAM Write Error!\r\n");
				DISABLE_IRQ;
				return ACCSNS_RC_ERR_RAMWRITE;
			}
			cmd.cmd.udata16 = HC_MCU_FUP_WRITE_FIFO;
			ret = accsns_hostcmd(&cmd, &res, EXE_HOST_WAIT|EXE_HOST_ERR);
			if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
				DISABLE_IRQ;
				return ACCSNS_RC_ERR;
			}
			if(res.err.udata16 == ERROR_FUP_MAXSIZE) {
DBG(DBG_LV_ERROR, "Size max Error! :: (%d/%dbyte)\r\n",i,(60*1024));
				DISABLE_IRQ;
				return ACCSNS_RC_ERR;
			}else if(res.err.udata16 == ERROR_FUP_VERIFY) {
DBG(DBG_LV_ERROR, "Verify Error! :: (%d/%dbyte)\r\n",i,(60*1024));
				DISABLE_IRQ;
				return ACCSNS_RC_ERR;
			}else if(res.err.udata16 == ERROR_FUP_ODDSIZE) {
DBG(DBG_LV_ERROR, "Odd size data Error! :: (%d/%dbyte)\r\n",i,(60*1024));
				DISABLE_IRQ;
				return ACCSNS_RC_ERR;
			}
		}
	} else {
DBG(DBG_LV_INFO, "CMD UPDATE.\r\n");
    for(i=0; i < arg_iLen; i+=8){
        cmd.prm.ub_prm[0] = 3;
        cmd.prm.ub_prm[1] = arg_iData[i + 0];
        cmd.prm.ub_prm[2] = arg_iData[i + 1];
        cmd.prm.ub_prm[3] = arg_iData[i + 2];
        cmd.prm.ub_prm[4] = arg_iData[i + 3];
        cmd.prm.ub_prm[5] = arg_iData[i + 4];
        cmd.prm.ub_prm[6] = arg_iData[i + 5];
        cmd.prm.ub_prm[7] = arg_iData[i + 6];
        cmd.prm.ub_prm[8] = arg_iData[i + 7];
        cmd.cmd.udata16 = HC_MCU_FUP_WRITE;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_WAIT|EXE_HOST_ERR);
        if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }
        if(res.err.udata16 == ERROR_FUP_MAXSIZE) {
DBG(DBG_LV_ERROR, "Size max Error! :: (%d/%dbyte)\r\n",i,(60*1024));
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }else if(res.err.udata16 == ERROR_FUP_VERIFY) {
DBG(DBG_LV_ERROR, "Verify Error! :: (%d/%dbyte)\r\n",i,(60*1024));
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }
    }
	}

DBG(DBG_LV_INFO, "Self Check.\r\n");
	cmd.cmd.udata16 = HC_MCU_SELF_CHK_FW;
	ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER);
DBG(DBG_LV_ERROR, "accsns_update_fw: HC_MCU_SELF_CHK_FW(%d) err res=%x err=%x\n", ret, res.res.ub_res[0], res.err.udata16);

DBG(DBG_LV_INFO, "End Firmware Update.\r\n");
    cmd.cmd.udata16 = HC_MCU_FUP_END;
    accsns_hostcmd(&cmd, &res, 0);
    
    msleep(300);

DBG(DBG_LV_INFO, "Check User program mode.\r\n");
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    if(res.res.ub_res[2] != 0x00){
DBG(DBG_LV_ERROR, "Version check Error!");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    
    DISABLE_IRQ;
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);

    return ACCSNS_RC_OK;
}

static void accsns_workqueue_init(void)
{
    int32_t i;
DBG_PRINT_IO(0, 0);
    
    for(i=0; i<ACC_WORK_QUEUE_NUM; i++){
        cancel_work_sync(&s_tAccWork[i].work);
        s_tAccWork[i].status = false;
    }
    s_nAccWorkCnt = 0;
}
    
static int32_t accsns_workqueue_create( struct workqueue_struct *queue, void (*func)(struct work_struct *) )
{
    int32_t ret = ACCSNS_RC_ERR;
    unsigned long flags;
    
    if((queue == NULL) || (func == NULL)){
        return ACCSNS_RC_ERR;
    }
    
    spin_lock_irqsave(&acc_lock, flags);
    
DBG(DBG_LV_LOW, "  [%d]  status:0x%x, \n",s_nAccWorkCnt, s_tAccWork[s_nAccWorkCnt].status);
        
    if(s_tAccWork[s_nAccWorkCnt].status == false){
        
        INIT_WORK( &s_tAccWork[s_nAccWorkCnt].work, func );
        
        ret = queue_work( queue, &s_tAccWork[s_nAccWorkCnt].work );
        
        if (ret == 1) {
            s_tAccWork[s_nAccWorkCnt].status = true;
            
            if(++s_nAccWorkCnt >= ACC_WORK_QUEUE_NUM){
                s_nAccWorkCnt = 0;
            }
            ret = ACCSNS_RC_OK;
            
        }else{
DBG(DBG_LV_ERROR, "ACC %s[%d] queue_work Non Create(%d) \n",__FUNCTION__, __LINE__, ret);
        }
        
    }else{
DBG(DBG_LV_ERROR, "ACC queue_work[%d] used!! \n", s_nAccWorkCnt);
    }
    
    spin_unlock_irqrestore(&acc_lock, flags);

    return ret;
}
    
static void accsns_workqueue_delete(struct work_struct *work)
{
    int32_t i;
    unsigned long flags;
    
    spin_lock_irqsave(&acc_lock, flags);
    
    for(i=0; i<ACC_WORK_QUEUE_NUM; i++){
        
        if(&s_tAccWork[i].work == work){
            s_tAccWork[i].status = false;
            
DBG(DBG_LV_LOW, "  hit delete queue[%d]! work:0x%x 0x%x \n",i, (int)&s_tAccWork[i].work, (int)work);
            break;
        }
    }
    
    spin_unlock_irqrestore(&acc_lock, flags);
    return ;
}

static void accsns_timeout_dump(uint8_t *reg)
{
    unsigned char data[0x1f];
    int i;
    
    DBG(DBG_LV_ERROR, "##### ACC TimeOut Error Log #####\n");
    DBG(DBG_LV_ERROR, "  Gloval Value :\n");
    DBG(DBG_LV_ERROR, "     Send Cmd :%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \n"
                             ,reg[0] ,reg[1] ,reg[2] ,reg[3] ,reg[4] ,reg[5] ,reg[6] ,reg[7] ,reg[8] ,reg[9] ,reg[10] 
                             ,reg[11] ,reg[12] ,reg[13] ,reg[14] ,reg[15] );
    DBG(DBG_LV_ERROR, "     g_bIsIntIrqEnable = %04x, g_nIntIrqFlg = %04x, ACCSNS_GPIO_INT= %04x \n"
                        , (int)atomic_read(&g_bIsIntIrqEnable)
                        , g_nIntIrqFlg
                        , gpio_get_value(ACCSNS_GPIO_INT));
    
    accsns_device_read(0x00, data, sizeof(data));
    
    DBG(DBG_LV_ERROR, "  H/W Register Dump :");
    for(i=0; i<sizeof(data); i++){
        if((i % 16) == 0){
            DBG(DBG_LV_ERROR, "\n     %02XH : ", i);
        }
        DBG(DBG_LV_ERROR, "0x%02X ",data[i]);
    }
    DBG(DBG_LV_ERROR, "\n");
    
    dbg_level = 0;
}

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static void accsns_shutdown( struct i2c_client *client )
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static void accsns_shutdown( struct spi_device *client )
#endif
{
DBG(DBG_LV_IO, "shutdown\n");
    accsns_activateEx(0, atomic_read(&g_flgEna), POWER_DISABLE);
}

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static int32_t accsns_suspend( struct i2c_client *client, pm_message_t mesg )
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static int32_t accsns_suspend( struct spi_device *client, pm_message_t mesg )
#endif
{
	int32_t iflgEna;
DBG(DBG_LV_IO, "ACC suspend\n");

	iflgEna = atomic_read(&g_flgEna);

DBG(DBG_LV_INFO, "%s: SUSPEND Before iflgEna = %x CurrentSensorEnable = %x\n",
			__FUNCTION__,iflgEna,atomic_read(&g_CurrentSensorEnable));
	if((iflgEna & ACCSNS_ACTIVE_ACC) == ACCSNS_ACTIVE_ACC){
		accsns_activateEx(0,ACCSNS_ACTIVE_ACC,POWER_DISABLE);
	}
	if((iflgEna & PRESNS_ACTIVE_PRE) == PRESNS_ACTIVE_PRE){
		accsns_activateEx(0,PRESNS_ACTIVE_PRE,POWER_DISABLE);
	}
	if ((iflgEna & ACCSNS_ACTIVE_DAILYS) == ACCSNS_ACTIVE_DAILYS) {
        enable_irq_wake(g_nIntIrqNo);
	}

DBG(DBG_LV_INFO, "%s: SUSPEND After iflgEna = %x CurrentSensorEnable = %x\n",
			__FUNCTION__,atomic_read(&g_flgEna),atomic_read(&g_CurrentSensorEnable));

    return ACCSNS_RC_OK;
}

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static int32_t accsns_resume( struct i2c_client *client )
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static int32_t accsns_resume( struct spi_device *client )
#endif
{
	int32_t iflgEna;
DBG(DBG_LV_IO, "ACC resume\n");

	iflgEna = atomic_read(&g_flgEna);

DBG(DBG_LV_INFO, "%s: RESUME Before iflgEna = %x CurrentSensorEnable = %x\n",
			__FUNCTION__,iflgEna,atomic_read(&g_CurrentSensorEnable));
	if((iflgEna & ACCSNS_ACTIVE_ACC) == ACCSNS_ACTIVE_ACC){
		accsns_activateEx(0,ACCSNS_ACTIVE_ACC,POWER_ENABLE);
	}
	if((iflgEna & PRESNS_ACTIVE_PRE) == PRESNS_ACTIVE_PRE){
		accsns_activateEx(0,PRESNS_ACTIVE_PRE,POWER_ENABLE);
	}
	if ((iflgEna & ACCSNS_ACTIVE_DAILYS) == ACCSNS_ACTIVE_DAILYS) {
        disable_irq_wake(g_nIntIrqNo);
	}

DBG(DBG_LV_INFO, "%s: RESUME After iflgEna = %x CurrentSensorEnable = %x\n",
			__FUNCTION__,atomic_read(&g_flgEna),atomic_read(&g_CurrentSensorEnable));

    return ACCSNS_RC_OK;
}

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static int32_t accsns_remove( struct i2c_client *client )
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static int32_t accsns_remove( struct spi_device *client )
#endif
{
DBG(DBG_LV_IO, "remove\n");
    accsns_activateEx(0, atomic_read(&g_flgEna), POWER_DISABLE);
    
    return ACCSNS_RC_OK;
}

static void accsns_early_suspend(struct early_suspend *h)
{
DBG(DBG_LV_IO, "ACC early_suspend\n");
}

static void accsns_late_resume(struct early_suspend *h)
{
DBG(DBG_LV_IO, "ACC late_resume");
}

static int32_t accsns_get_dailys_active(void)
{
	int32_t iCurrentEnable;
DBG_PRINT_IO(0, 0);
    
	iCurrentEnable = atomic_read(&g_CurrentSensorEnable);

DBG_PRINT_IO(0xFF, 0);
    return (((iCurrentEnable & ACCSNS_ACTIVE_DAILYS) == ACCSNS_ACTIVE_DAILYS) ? 1:0);
}

static int accsns_input_init(void)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "accelerometer";

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
	dev->id.bustype = BUS_I2C;
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
	dev->id.bustype = BUS_SPI;
#endif

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_capability(dev, EV_ABS, ABS_RUDDER);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_RX, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_RY, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_RZ, ABSMIN_2G, ABSMAX_2G, 0, 0);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	acc_idev = dev;

	return 0;
}

void accsns_set_ofs_en(int32_t ofs_en)
{
DBG_PRINT_IO(0, 0);

    atomic_set(&g_nOfs_en, ofs_en);

DBG(DBG_LV_INFO, "set ofs_en:%d\n", ofs_en);

DBG_PRINT_IO(0xFF, 0);
}

static void accsns_calibration_range_chk(const struct acceleration* accData)
{
	int32_t ret = 0;

	DBG(DBG_LV_INFO, "range_chk X:%d Y:%d Z:%d\n",accData->nX,accData->nY,accData->nZ);

	switch (s_tCalibCtrl.m_nMode) {
		case MODE_0:
		case MODE_1:
		case MODE_3:
			if(s_tCalibCtrl.m_nLowTH > accData->nX || s_tCalibCtrl.m_nHighTH < accData->nX ||
				s_tCalibCtrl.m_nLowTH > accData->nY || s_tCalibCtrl.m_nHighTH < accData->nY ||
				s_tCalibCtrl.m_nLowTH > (accData->nZ - WEIGHT_1G) ||
				s_tCalibCtrl.m_nHighTH < (accData->nZ - WEIGHT_1G)){
				ret = ACCSNS_RC_ERR;
			}
		break;
		case MODE_2:
		case MODE_4:
			if(s_tCalibCtrl.m_nLowTH > accData->nX || s_tCalibCtrl.m_nHighTH < accData->nX ||
				s_tCalibCtrl.m_nLowTH > accData->nY || s_tCalibCtrl.m_nHighTH < accData->nY ||
				s_tCalibCtrl.m_nLowTH > (accData->nZ + WEIGHT_1G) ||
				s_tCalibCtrl.m_nHighTH < (accData->nZ + WEIGHT_1G)){
				ret = ACCSNS_RC_ERR;
			}
			break;
		default:
DBG(DBG_LV_ERROR, "ACC-Calib: Mode Err!!\n");
			break;
	}

	if(ret < 0){
		s_tCalibCtrl.m_bRengeChkErr = true;
DBG(DBG_LV_ERROR, "ACC-Calib: Range Chk Err m_bRengeChkErr :%d\n",s_tCalibCtrl.m_bRengeChkErr);
	}
}

static int32_t accsns_iwifi_set_info(bool req, DailysSetIWifiParam *DailysIWifiParam)
{
    int32_t   ret;
    HostCmd cmd;
    HostCmdRes res;
DBG_PRINT_IO(0, 0);

	if(req == true){
		cmd.cmd.udata16 = HC_DST_SET_IWIFI_INFO;
		cmd.prm.ub_prm[0] = 0x01;
		cmd.prm.ub_prm[1] = (DailysIWifiParam->m_nPedoStartStep & 0xFF);
		cmd.prm.ub_prm[2] = (DailysIWifiParam->m_nPedoEndTime & 0xFF);
		cmd.prm.ub_prm[3] = ((DailysIWifiParam->m_nPedoEndTime >> 8) & 0xFF);
		cmd.prm.ub_prm[4] = (DailysIWifiParam->m_nVehiStartTime & 0xFF);
		cmd.prm.ub_prm[5] = ((DailysIWifiParam->m_nVehiStartTime >> 8) & 0xFF);
		cmd.prm.ub_prm[6] = (DailysIWifiParam->m_nVehiEndTime & 0xFF);
		cmd.prm.ub_prm[7] = ((DailysIWifiParam->m_nVehiEndTime >> 8) & 0xFF);

	    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
		if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Set IWifi Value HC_DST_SET_IWIFI_INFO(-) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
		}
	} else {
		cmd.cmd.udata16 = HC_DST_GET_IWIFI_INFO;

		ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
		if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Get IWifi Value HC_DST_GET_IWIFI_INFO(-) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
		}
		DailysIWifiParam->m_nPedoStartStep = (uint32_t)res.res.ub_res[1];
		DailysIWifiParam->m_nPedoEndTime   = (uint32_t)res.res.uw_res[1];
		DailysIWifiParam->m_nVehiStartTime = (uint32_t)res.res.uw_res[2];
		DailysIWifiParam->m_nVehiEndTime   = (uint32_t)res.res.uw_res[3];

		DBG(DBG_LV_LOW, "HC_DST_GET_IWIFI_INFO get, m_nPedoStartStep:%d, m_nPedoEndTime:%d m_nVehiStartTime:%d m_nVehiEndTime:%d\n",
						DailysIWifiParam->m_nPedoStartStep,DailysIWifiParam->m_nPedoEndTime,
						DailysIWifiParam->m_nVehiStartTime,DailysIWifiParam->m_nVehiEndTime);
	}

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static ssize_t accsns_enable_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int enable;
	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	mutex_lock(&s_tEnableMutex);
	enable = accsns_get_acc_active();
	mutex_unlock(&s_tEnableMutex);

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t accsns_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	unsigned long arg_iEnable;
	int32_t ret;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }

	ret = strict_strtoul(buf, 10, &arg_iEnable);
	if (ret < 0)
		return count;

	mutex_lock(&s_tEnableMutex);
	flgA += (arg_iEnable != 0)? 1: -1;
	if(flgA < 0)
		flgA = 0;
    arg_iEnable = (flgA == 0)? POWER_DISABLE : POWER_ENABLE;
	ret = accsns_activateEx(1, ACCSNS_ACTIVE_ACC, arg_iEnable);
	mutex_unlock(&s_tEnableMutex);

	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR ACCSNS_ACTIVE_ACC:%s\n", __FUNCTION__);
		accsns_err_check();
	}

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t accsns_delay_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	DBG(DBG_LV_INFO, "%s(): \n",__func__);
	return sprintf(buf, "%d\n", accsns_get_delay());
}

static ssize_t accsns_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	unsigned long delay;
	int ret;
	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	ret = strict_strtoul(buf, 10, &delay);
	if (ret < 0)
		return count;
	DBG(DBG_LV_INFO, "%s(): delay = %ld\n",__func__,delay);

	accsns_set_delay(delay);

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t accsns_data_raw_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct acceleration accel;
	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	mutex_lock(&s_tDataMutex);
	accel = s_tLatestAccData;
	mutex_unlock(&s_tDataMutex);

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d %d %d\n", accel.nX,accel.nY,accel.nZ);
}

static ssize_t accsns_data_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct acceleration accel;
	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	mutex_lock(&s_tDataMutex);
	accel = s_tLatestAccData;
	mutex_unlock(&s_tDataMutex);

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d %d %d\n", accel.outX,accel.outY,accel.outZ);
}

static ssize_t accsns_fw_version_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	uint8_t  fw_ver[4];
	uint32_t ret;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

    ret = accsns_get_fw_version(fw_ver);
    g_nAccFWVersion = ACC_FW_VERSION_GET_32BIT(fw_ver);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_get_fw_version:Get Version Error!!\n");
        g_nAccFWVersion = ACC_FW_VERSION_NONE;
    }

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%08x\n", g_nAccFWVersion);
}

static ssize_t accsns_fw_update_sq_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	uint8_t *fw_data = NULL;
	uint8_t *data_addr = NULL;
	uint32_t len;
	uint32_t data[2];
	uint32_t ret;

DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }

	sscanf(buf, "%x %d", &data[0],&data[1]);
	data_addr = (uint8_t*)data[0];
	len = data[1];
DBG(DBG_LV_INFO, "[ACC] FW Update data:%x len:%d data_addr:%x\n", data[0], len, (uint32_t)data_addr);


	if((data_addr == NULL) || (len == 0)){
		DBG(DBG_LV_ERROR, "error(Input param) : fw_update_sq\n" );
		return count;
		}

	if((len % 8) != 0){
		DBG(DBG_LV_ERROR, "error(Firmdata size error) : fw_update_sq\n" );
		return count;
	}

	fw_data = (uint8_t *)kmalloc( len, GFP_KERNEL );
	if(fw_data == NULL){
		DBG(DBG_LV_ERROR, "error(kmalloc) : fw_update_sq\n" );
		return -ENOMEM;
	}

	ret = copy_from_user( fw_data, data_addr, len );
	if( ret != 0 )
	{
		DBG(DBG_LV_ERROR, "error(copy_from_user(data)) : fw_update_sq\n" );
		kfree( fw_data );
		return count;
	}

	ret = accsns_update_fw_seq(fw_data, len);
	kfree( fw_data );
  
	if(ret != 0){
		DBG(DBG_LV_ERROR, "error(accsns_update_fw_seq) : fw_update_sq\n");
		return count;
	}

	ret = accsns_set_dev_param();
	if(ret != 0){
		DBG(DBG_LV_ERROR, "error(accsns_set_dev_param) : fw_update_sq\n");
		return count;
    }

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t accsns_fw_update_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	uint8_t *fw_data = NULL;
	uint8_t *data_addr = NULL;
	uint32_t len,i;
	uint32_t data[2];
	uint32_t ret;

DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }

	sscanf(buf, "%x %d", &data[0],&data[1]);
	data_addr = (uint8_t*)data[0];
	len = data[1];
DBG(DBG_LV_INFO, "[ACC] FW Update data:%x len:%d data_addr:%x\n", data[0], len, (uint32_t)data_addr);


	if((data_addr == NULL) || (len == 0)){
		DBG(DBG_LV_ERROR, "error(Input param) : fw_update\n" );
		return count;
		}

	if((len % 8) != 0){
		DBG(DBG_LV_ERROR, "error(Firmdata size error) : fw_update\n" );
		return count;
	}

	fw_data = (uint8_t *)kmalloc( len, GFP_KERNEL );
	if(fw_data == NULL){
		DBG(DBG_LV_ERROR, "error(kmalloc) : fw_update\n" );
		return count;
	}

	ret = copy_from_user( fw_data, data_addr, len );
	if( ret != 0 )
	{
		DBG(DBG_LV_ERROR, "error(copy_from_user(data)) : fw_update\n" );
		kfree( fw_data );
		return count;
	}
    atomic_set(&g_FWUpdateStatus,true);
	ret = accsns_update_fw(false,fw_data, len);
	if(ret == ACCSNS_RC_ERR_RAMWRITE){
		ret = gpio_direction_output(ACCSNS_GPIO_TEST0, 0);
		if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request(output) ret=%d\n",ret);
			kfree( fw_data );
			atomic_set(&g_FWUpdateStatus,false);
			return count;
		}
		DISABLE_IRQ;
		for(i = 0; i < ACC_SPI_RETRY_NUM; i++){
			accsns_FW_TEST0_ctrl();
			ret = accsns_update_fw(true, fw_data, len);
			if(ret != ACCSNS_RC_ERR_RAMWRITE)
				break;
		}
		ret |= gpio_direction_input(ACCSNS_GPIO_TEST0);
	}
	atomic_set(&g_FWUpdateStatus,false);

	kfree( fw_data );

	if(ret != 0){
		DBG(DBG_LV_ERROR, "error(accsns_update_fw) : fw_update\n");
		return count;
	}

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t accsns_fw_version_chk_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	uint8_t  fw_ver[4];
	uint32_t ret,FWVersion;
	uint32_t fw_chk = FW_VERSION_CHK_NG;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return sprintf(buf, "%x\n", fw_chk);
    }

    ret = accsns_get_fw_version(fw_ver);
    FWVersion = ACC_FW_VERSION_GET_32BIT(fw_ver);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_get_fw_version:Get Version Error!!\n");
        FWVersion = ACC_FW_VERSION_NONE;
    }

	if(FWVersion == ACC_FW_VERSION_DATA){
		fw_chk = FW_VERSION_CHK_OK;
	} else {
		fw_chk = FW_VERSION_CHK_NG;
	}

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%x\n", fw_chk);
}

static ssize_t accsns_debug_level_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	DBG(DBG_LV_INFO, "%s(): \n",__func__);
	return sprintf(buf, "%x\n", dbg_level);
}

static ssize_t accsns_debug_level_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	uint32_t level;
	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	sscanf(buf, "%x", &level);
	DBG(DBG_LV_INFO, "%s(): level = %x\n",__func__,level);

	accsns_debug_level_chg(level);

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t accsns_test0_ctl_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	unsigned long direction;
	int32_t ret;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }

	ret = strict_strtoul(buf, 10, &direction);
	if (ret < 0)
		return count;

DBG(DBG_LV_INFO, "%s(): direction:%ld\n",__func__,direction);

	if(direction == 1){
		ret = gpio_direction_input(ACCSNS_GPIO_TEST0);
	}

	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR TEST0 CTL:%s\n", __FUNCTION__);
	}

	ret = accsns_set_dev_param();
	if(ret != 0){
		DBG(DBG_LV_ERROR, "error(accsns_set_dev_param) : test0_ctl\n");
		return count;
    }

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t accsns_host_cmd_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", 
					diag_res.res.ub_res[0],diag_res.res.ub_res[1],
					diag_res.res.ub_res[2],diag_res.res.ub_res[3],
					diag_res.res.ub_res[4],diag_res.res.ub_res[5],
					diag_res.res.ub_res[6],diag_res.res.ub_res[7],
					diag_res.res.ub_res[8],diag_res.res.ub_res[9],
					diag_res.res.ub_res[10],diag_res.res.ub_res[11],
					diag_res.res.ub_res[12],diag_res.res.ub_res[13],
					diag_res.res.ub_res[14],diag_res.res.ub_res[15],
					diag_res.res.ub_res[16],diag_res.res.ub_res[17],
					diag_res.res.ub_res[18],diag_res.res.ub_res[19],
					diag_res.res.ub_res[20],diag_res.res.ub_res[21],
					diag_res.res.ub_res[22],diag_res.res.ub_res[23],
					diag_res.res.ub_res[24],diag_res.res.ub_res[25],
					diag_res.res.ub_res[26],diag_res.res.ub_res[27],
					diag_res.res.ub_res[28],diag_res.res.ub_res[29],
					diag_res.res.ub_res[30]);
}

static ssize_t accsns_host_cmd_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
    HostCmd cmd;
    HostCmdRes res;
	uint32_t ret,i;
	uint32_t req_cmd[2];
	uint32_t req_param[13];

DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	memset(&res, 0x00, sizeof(HostCmdRes));
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }

	sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",&req_cmd[0],&req_cmd[1],
				&req_param[0],&req_param[1],&req_param[2],&req_param[3],&req_param[4],
				&req_param[5],&req_param[6],&req_param[7],&req_param[8],&req_param[9],
				&req_param[10],&req_param[11],&req_param[12]);
	cmd.cmd.udata16 = (uint16_t)((req_cmd[0] << 8) | req_cmd[1]);
	for(i=0;i<13;i++){
		cmd.prm.ub_prm[i] = (uint8_t)req_param[i];
	}

	ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
	if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "accsns_host_cmd_store(0x%x) err %x\n",cmd.cmd.udata16,res.err.udata16);
	}
	memcpy(&diag_res, &res, sizeof(HostCmdRes));

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t acc_ope_device_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	DBG(DBG_LV_INFO,"%s(): start\n",__func__);
	return sprintf(buf, "not supported\n" );
}

static ssize_t acc_ope_device_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret = 0;
	uint32_t ope = 0;

	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	sscanf(buf, "%d", &ope);
	DBG(DBG_LV_INFO,"%s(): ope = %d\n",__func__,ope);

	switch(ope)
	{
	case KC_SENSOR_COMMON_INIT:
		ret = accsns_initialize();
		ret |= accsns_set_dev_param();
		break;
	case KC_SENSOR_COMMON_IMITATION_ON:
		g_bIsAlreadyExistAccImitationXYZData       = true;
		break;
	case KC_SENSOR_COMMON_IMITATION_OFF:
		g_bIsAlreadyExistAccImitationXYZData       = false;
		break;
	default:
		break;
	}
	DBG(DBG_LV_INFO,"%s(): [OUT] ret = %d\n",__func__,ret);
	return count;
}

static ssize_t acc_cal_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret = 0;
	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);

	ret = accsns_calibration_mode();
	if(ret != 0){
		DBG(DBG_LV_ERROR, "error(accsns_calibration_mode) : cal_mode\n");
		return count;
	}

	DBG(DBG_LV_INFO,"%s(): [OUT] ret = %d\n",__func__,ret);
	return count;
}

static ssize_t acc_start_cal_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret = 0;
	uint32_t mode;

	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	sscanf(buf, "%d", &mode);
	DBG(DBG_LV_INFO,"%s(): mode = %d\n",__func__,mode);

	ret = accsns_calibration_start(mode);
	if(ret != 0){
		DBG(DBG_LV_ERROR, "error(accsns_calibration_start) : start_cal\n");
		return count;
	}

	DBG(DBG_LV_INFO,"%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t acc_cal_wait_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	DBG(DBG_LV_INFO,"%s():\n",__func__);

	return sprintf(buf, "%d\n", accsns_calibration_is_wait());
}

static ssize_t acc_offset_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{

	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	return sprintf(buf, "%d %d %d\n"
		       , atomic_read(&g_nCalX), atomic_read(&g_nCalY), atomic_read(&g_nCalZ));
}

static ssize_t acc_offset_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t offset[3];

	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	sscanf(buf, "%d %d %d", &offset[0], &offset[1], &offset[2]);

	DBG(DBG_LV_INFO,"%s(): offset = %d %d %d\n",__func__,offset[0], offset[1], offset[2]);
	accsns_set_offset(offset);

	DBG(DBG_LV_INFO,"%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t acc_ofs_en_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{

	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	return sprintf(buf, "%d\n", atomic_read(&g_nOfs_en));
}

static ssize_t acc_ofs_en_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	uint32_t ofs_en = 0;

	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	sscanf(buf, "%d", &ofs_en);
	DBG(DBG_LV_INFO,"%s(): flt_en = %d \n",__func__,ofs_en);

	accsns_set_ofs_en(ofs_en);

	DBG(DBG_LV_INFO,"%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t accsns_cal_smp_n_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int cal_smp_n;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
	cal_smp_n = s_tCalibCtrl.m_unSmpN;
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);

	return sprintf(buf, "%d\n", cal_smp_n);
}

static ssize_t accsns_cal_smp_n_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret = 0;
	unsigned long arg_iCal_smp_n;

DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	ret = strict_strtoul(buf, 10, &arg_iCal_smp_n);
	if (ret < 0)
		return count;
DBG(DBG_LV_INFO, "%s(): Cal_smp_n = %ld\n",__func__,arg_iCal_smp_n);

	mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
	s_tCalibCtrl.m_unSmpN = arg_iCal_smp_n;
	mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}


static ssize_t accsns_cal_th_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int cal_th;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
	cal_th = s_tCalibCtrl.m_nHighTH;
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);

	return sprintf(buf, "%d\n", cal_th);
}

static ssize_t accsns_cal_th_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret = 0;
	unsigned long arg_iCal_th;

DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	ret = strict_strtoul(buf, 10, &arg_iCal_th);
	if (ret < 0)
		return count;
DBG(DBG_LV_INFO, "%s(): Cal_th = %ld\n",__func__,arg_iCal_th);

	mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
	s_tCalibCtrl.m_nHighTH = arg_iCal_th;
	s_tCalibCtrl.m_nLowTH  = -(arg_iCal_th);
	mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

DBG(DBG_LV_INFO, "%s(): HighTH = %d LowTH = %d\n",__func__,s_tCalibCtrl.m_nHighTH,s_tCalibCtrl.m_nLowTH);

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t acc_imit_data_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);

	DBG(DBG_LV_INFO,"%s(): g_naAccImitationData [0] = %d [1] = %d [2] = %d\n",
					__func__,g_naAccImitationData[0],
					g_naAccImitationData[1],g_naAccImitationData[2]);

	DBG(DBG_LV_INFO,"%s(): [OUT]\n",__func__);

	return sprintf(buf, "%d %d %d\n",
				g_naAccImitationData[0], 
				g_naAccImitationData[1], 
				g_naAccImitationData[2]);
}

static ssize_t acc_imit_data_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	sscanf(buf, "%d %d %d", &g_naAccImitationData[0],
					&g_naAccImitationData[1],&g_naAccImitationData[2]);

	DBG(DBG_LV_INFO,"%s(): g_naAccImitationData [0] = %d [1] = %d [2] = %d\n"
					,__func__,g_naAccImitationData[0],
					g_naAccImitationData[1],g_naAccImitationData[2]);

	DBG(DBG_LV_INFO,"%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t dailys_enable_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int enable;
	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	mutex_lock(&s_tEnableMutex);
	enable = accsns_get_dailys_active();
	mutex_unlock(&s_tEnableMutex);

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%x %x\n",atomic_read(&g_Dailys_IntType), enable);
}

static ssize_t dailys_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t arg_iType = 0,arg_iEnable = 0;
	int32_t ret = 0;
	int32_t old_type,new_type;
	int32_t vehi_param[15];
	int32_t iwifi_param[2];
    int32_t iWakeupSensor;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }

	sscanf(buf, "%x %x", &arg_iType,&arg_iEnable);
DBG(DBG_LV_INFO, "%s(): Type = %x Enable = %x\n",__func__,arg_iType,arg_iEnable);

	old_type = atomic_read(&g_Dailys_IntType);

	if(old_type == 0){
		ret = accsns_pedom_clear(DAILYS_CLEAR_ALL);
	}

	if(arg_iEnable == 0){
		new_type = old_type & (~arg_iType);
	} else {
		new_type = old_type | arg_iType;
	}
	atomic_set(&g_Dailys_IntType,new_type);

DBG(DBG_LV_INFO, "%s(): OldType = %x NewType = %x\n",__func__,old_type,new_type);

	mutex_lock(&s_tEnableMutex);
	if(old_type != 0 && new_type != 0){
		ret |= accsns_pedom_activate(POWER_ENABLE);
	} else {
		ret |= accsns_activate_pedom(1, arg_iEnable);
	}
	mutex_unlock(&s_tEnableMutex);

	if(((old_type & ~DAILYS_INT_PEDO) != 0) && (arg_iEnable != 0)){
		if((arg_iType & DAILYS_INT_VEHI) == DAILYS_INT_VEHI){
			ret |= accsns_get_vehicle_data(vehi_param);
			if((ret == ACCSNS_RC_OK) && (vehi_param[0] == 1)){
DBG(DBG_LV_INT, "### *** Vehicle Detected Immediately !! \n");
				iWakeupSensor = atomic_read(&g_WakeupSensor);
				atomic_set(&g_WakeupSensor,iWakeupSensor | ACCSNS_ACTIVE_VEHICLE);
				wake_up_interruptible(&s_tPollWaitVehicle);
			}
		}
		if((arg_iType & DAILYS_INT_WIFI) == DAILYS_INT_WIFI){
			ret |= accsns_get_iwifi_data(iwifi_param);
			if((ret == ACCSNS_RC_OK) && ((iwifi_param[0] | iwifi_param[1]) != 0)){
DBG(DBG_LV_INT, "### *** Intelli Wifi Detected Immediately !! \n");
				iWakeupSensor = atomic_read(&g_WakeupSensor);
				atomic_set(&g_WakeupSensor,iWakeupSensor | ACCSNS_ACTIVE_IWIFI);
				wake_up_interruptible(&s_tPollWaitIWifi);
			}
		}
	}


	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR ACCSNS_ACTIVE_DAILYS:%s\n", __FUNCTION__);
		accsns_err_check();
	}

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t dailys_param_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	DailysSetParam DailysParam;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	DailysParam.m_nStepWide = atomic_read(&g_nStepWide);
	DailysParam.m_nWeight   = atomic_read(&g_nWeight);
	DailysParam.m_VehiType  = atomic_read(&g_nVehiType);

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d %d %d\n", DailysParam.m_nStepWide,
					DailysParam.m_nWeight,DailysParam.m_VehiType);
}

static ssize_t dailys_param_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret;
	DailysSetParam DailysParam;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }
	sscanf(buf, "%d %d %d", &DailysParam.m_nStepWide,&DailysParam.m_nWeight,
							&DailysParam.m_VehiType);

	mutex_lock(&s_tEnableMutex);
	ret = accsns_pedom_set_info(DailysParam.m_nWeight,DailysParam.m_nStepWide,
								DailysParam.m_VehiType);
	mutex_unlock(&s_tEnableMutex);

	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR accsns_pedom_set_info:%s\n", __FUNCTION__);
	}

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t dailys_pedo_data_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int32_t ret;
	int32_t param[21];
	DailysGetPedomInfo DailysPedomInfo;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	memset(&DailysPedomInfo, 0, sizeof(DailysPedomInfo));

    ret = accsns_get_pedometer_data(param);
    if(ret == 0){
		DailysPedomInfo.m_iStep       = param[0];
		DailysPedomInfo.m_iTime       = param[1];
		DailysPedomInfo.m_iCalorie    = param[2];
		DailysPedomInfo.m_iFat        = param[3];
		DailysPedomInfo.m_iExercise   = param[4];
		DailysPedomInfo.m_iMets       = param[5];
		DailysPedomInfo.m_iSpeedmeter = param[6];
		DailysPedomInfo.m_iRunStatus  = param[7];
		DailysPedomInfo.m_iRunStepCnt = param[8];
		DailysPedomInfo.m_iRunTime    = param[9];
		DailysPedomInfo.m_iStExercise = param[10];
		DailysPedomInfo.m_iStCal      = param[11];
		DailysPedomInfo.m_iStBodyFat  = param[12];
		DailysPedomInfo.m_iSportExercise = param[13];
		DailysPedomInfo.m_iRunCal      = param[14];
		DailysPedomInfo.m_iRunExercise = param[15];
		DailysPedomInfo.m_iPS_InitBaseHeight = param[16];
		DailysPedomInfo.m_iPS_ActHeight      = param[17];
		DailysPedomInfo.m_iPS_ActHeightMin   = param[18];
		DailysPedomInfo.m_iPS_ActHeightAve   = param[19];
		DailysPedomInfo.m_iPS_ActHeightMax   = param[20];
	} else {
DBG(DBG_LV_ERROR, "error :accsns_get_pedometer_data\n" );
		accsns_err_check();
    }

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",DailysPedomInfo.m_iStep,
					DailysPedomInfo.m_iTime,DailysPedomInfo.m_iCalorie,
					DailysPedomInfo.m_iFat,DailysPedomInfo.m_iExercise,
					DailysPedomInfo.m_iMets,DailysPedomInfo.m_iSpeedmeter,
					DailysPedomInfo.m_iRunStatus,DailysPedomInfo.m_iRunStepCnt,
					DailysPedomInfo.m_iRunTime,DailysPedomInfo.m_iStExercise,
					DailysPedomInfo.m_iStCal,DailysPedomInfo.m_iStBodyFat,
					DailysPedomInfo.m_iSportExercise,DailysPedomInfo.m_iRunCal,
					DailysPedomInfo.m_iRunExercise,DailysPedomInfo.m_iPS_InitBaseHeight,
					DailysPedomInfo.m_iPS_ActHeight,DailysPedomInfo.m_iPS_ActHeightMin,
					DailysPedomInfo.m_iPS_ActHeightAve,DailysPedomInfo.m_iPS_ActHeightMax);
}

static ssize_t dailys_vehi_data_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int32_t ret;
	int32_t param[15];
	DailysGetVehicleInfo DailysVehicleInfo;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	memset(&DailysVehicleInfo, 0, sizeof(DailysVehicleInfo));

    ret = accsns_get_vehicle_data(param);
    if(ret == 0){
		DailysVehicleInfo.m_iStatus       = param[0];
		DailysVehicleInfo.m_iKind         = param[1];
		DailysVehicleInfo.m_iDetectTime   = param[2];
		DailysVehicleInfo.m_iRideTime     = param[3];
		DailysVehicleInfo.m_iRideCal      = param[4];
		DailysVehicleInfo.m_iVehiBodyFat  = param[5];
		DailysVehicleInfo.m_iVehiExercise = param[6];
		DailysVehicleInfo.m_iVehiMets     = param[7];
		DailysVehicleInfo.m_iVehiStExercise = param[8];
		DailysVehicleInfo.m_iVehiStRideCal  = param[9];
		DailysVehicleInfo.m_iVehiStBodyFat  = param[10];
		DailysVehicleInfo.m_iVehiBiExercise    = param[11];
		DailysVehicleInfo.m_iVehiBiRideCal     = param[12];
		DailysVehicleInfo.m_iVehiBiBodyFat     = param[13];
		DailysVehicleInfo.m_iVehiSportExercise = param[14];
	} else {
DBG(DBG_LV_ERROR, "error :accsns_get_vehicle_data\n" );
		accsns_err_check();
    }

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",DailysVehicleInfo.m_iStatus,
					DailysVehicleInfo.m_iKind,DailysVehicleInfo.m_iDetectTime,
					DailysVehicleInfo.m_iRideTime,DailysVehicleInfo.m_iRideCal,
					DailysVehicleInfo.m_iVehiBodyFat,DailysVehicleInfo.m_iVehiExercise,
					DailysVehicleInfo.m_iVehiMets,DailysVehicleInfo.m_iVehiStExercise,
					DailysVehicleInfo.m_iVehiStRideCal,DailysVehicleInfo.m_iVehiStBodyFat,
					DailysVehicleInfo.m_iVehiBiExercise,DailysVehicleInfo.m_iVehiBiRideCal,
					DailysVehicleInfo.m_iVehiBiBodyFat,DailysVehicleInfo.m_iVehiSportExercise);
}

static ssize_t dailys_clear_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	uint32_t clear_req;
	int32_t ret;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }

	sscanf(buf, "%x", &clear_req);
	DBG(DBG_LV_INFO, "%s(): clear_req = %x\n",__func__,clear_req);

	ret = accsns_pedom_clear(clear_req);

	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR CLEAR DATA:%s\n", __FUNCTION__);
		accsns_err_check();
	}

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t dailys_iwifi_data_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int32_t ret;
	int32_t param[2];
	DailysGetIntelliWifiInfo DailysIntelliWifiInfo;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	memset(&DailysIntelliWifiInfo, 0, sizeof(DailysIntelliWifiInfo));

    ret = accsns_get_iwifi_data(param);
    if(ret == 0){
		DailysIntelliWifiInfo.m_iPedoStatus = param[0];
		DailysIntelliWifiInfo.m_iVehiStatus = param[1];
	} else {
DBG(DBG_LV_ERROR, "error :accsns_get_iwifi_data\n" );
		accsns_err_check();
    }

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d %d\n",DailysIntelliWifiInfo.m_iPedoStatus,DailysIntelliWifiInfo.m_iVehiStatus);
}

static ssize_t dailys_iwifi_param_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int32_t ret;
	DailysSetIWifiParam DailysIWifiParam;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	memset(&DailysIWifiParam, 0x00, sizeof(DailysIWifiParam));

	mutex_lock(&s_tEnableMutex);
	ret = accsns_iwifi_set_info(false, &DailysIWifiParam);
	mutex_unlock(&s_tEnableMutex);

	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR accsns_iwifi_set_info:%s\n", __FUNCTION__);
	}

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d %d %d %d\n", DailysIWifiParam.m_nPedoStartStep,DailysIWifiParam.m_nPedoEndTime,
										DailysIWifiParam.m_nVehiStartTime,DailysIWifiParam.m_nVehiEndTime);
}

static ssize_t dailys_iwifi_param_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret;
	DailysSetIWifiParam DailysIWifiParam;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }
	sscanf(buf, "%d %d %d %d", &DailysIWifiParam.m_nPedoStartStep,&DailysIWifiParam.m_nPedoEndTime,
							&DailysIWifiParam.m_nVehiStartTime,&DailysIWifiParam.m_nVehiEndTime);

	mutex_lock(&s_tEnableMutex);
	ret = accsns_iwifi_set_info(true, &DailysIWifiParam);
	mutex_unlock(&s_tEnableMutex);

	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR accsns_iwifi_set_info:%s\n", __FUNCTION__);
	}

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static int32_t dailys_io_open( struct inode* inode, struct file* filp )
{
  return 0;
}

static int32_t dailys_io_release( struct inode* inode, struct file* filp )
{
  return 0;
}

static unsigned int dailys_io_poll_pedom(struct file *fp, poll_table *wait)
{
    unsigned int wake_func = 0;
    unsigned int ret = 0;

    wake_func = accsns_io_poll_pedom(fp, wait);

    if ((wake_func & ACCSNS_ACTIVE_PEDOM_ERROR) == ACCSNS_ACTIVE_PEDOM_ERROR) {
        printk( "error(accsns_io_poll_pedom) : dailys_io_poll_pedom \n" );
        accsns_err_check();
        ret = POLLERR;
    } else if ((wake_func & ACCSNS_ACTIVE_PEDOM) == ACCSNS_ACTIVE_PEDOM) {
        ret = POLLIN | POLLPRI;
    }
    
    return ret;
}

static unsigned int dailys_io_poll_vehicle(struct file *fp, poll_table *wait)
{
    unsigned int wake_func = 0;
    unsigned int ret = 0;

    wake_func = accsns_io_poll_vehicle(fp, wait);

    if ((wake_func & ACCSNS_ACTIVE_VEHICLE_ERROR) == ACCSNS_ACTIVE_VEHICLE_ERROR) {
        printk( "error(accsns_io_poll_vehicle) : dailys_io_poll_vehicle \n" );
        accsns_err_check();
        ret = POLLERR;
    } else if ((wake_func & ACCSNS_ACTIVE_VEHICLE) == ACCSNS_ACTIVE_VEHICLE) {
        ret = POLLIN | POLLPRI;
    }
    
    return ret;
}

static unsigned int dailys_io_poll_iwifi(struct file *fp, poll_table *wait)
{
    unsigned int wake_func = 0;
    unsigned int ret = 0;

    wake_func = accsns_io_poll_iwifi(fp, wait);

    if ((wake_func & ACCSNS_ACTIVE_IWIFI_ERROR) == ACCSNS_ACTIVE_IWIFI_ERROR) {
        printk( "error(accsns_io_poll_iwifi) : dailys_io_poll_iwifi \n" );
        accsns_err_check();
        ret = POLLERR;
    } else if ((wake_func & ACCSNS_ACTIVE_IWIFI) == ACCSNS_ACTIVE_IWIFI) {
        ret = POLLIN | POLLPRI;
    }
    
    return ret;
}


static struct file_operations dailys_fops_pedom = {
  .owner   = THIS_MODULE,
  .open    = dailys_io_open,
  .release = dailys_io_release,
  .poll    = dailys_io_poll_pedom,
};

static struct file_operations dailys_fops_vehicle = {
  .owner   = THIS_MODULE,
  .open    = dailys_io_open,
  .release = dailys_io_release,
  .poll    = dailys_io_poll_vehicle,
};

static struct file_operations dailys_fops_iwifi = {
  .owner   = THIS_MODULE,
  .open    = dailys_io_open,
  .release = dailys_io_release,
  .poll    = dailys_io_poll_iwifi,
};

static struct miscdevice dailys_device_pedom = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "dailys_io_pedom",
  .fops  = &dailys_fops_pedom,
};
static struct miscdevice dailys_device_vehicle = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "dailys_io_vehicle",
  .fops  = &dailys_fops_vehicle,
};
static struct miscdevice dailys_device_iwifi = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "dailys_io_iwifi",
  .fops  = &dailys_fops_iwifi,
};

static DEVICE_ATTR(enable,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_enable_show,
		   accsns_enable_store
		   );
static DEVICE_ATTR(delay,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_delay_show,
		   accsns_delay_store
		   );
static DEVICE_ATTR(data_raw,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_data_raw_show,
		   NULL
		   );
static DEVICE_ATTR(data,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_data_show,
		   NULL
		   );
static DEVICE_ATTR(fw_version,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_fw_version_show,
		   NULL
		   );
static DEVICE_ATTR(fw_update_sq,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   NULL,
		   accsns_fw_update_sq_store
		   );
static DEVICE_ATTR(fw_update,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   NULL,
		   accsns_fw_update_store
		   );
static DEVICE_ATTR(fw_version_chk,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_fw_version_chk_show,
		   NULL
		   );
static DEVICE_ATTR(debug_level,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_debug_level_show,
		   accsns_debug_level_store
		   );
static DEVICE_ATTR(test0_ctl,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   NULL,
		   accsns_test0_ctl_store
		   );
static DEVICE_ATTR(host_cmd,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_host_cmd_show,
		   accsns_host_cmd_store
		   );
static DEVICE_ATTR(ope_device,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   acc_ope_device_show,
		   acc_ope_device_store
		   );
static DEVICE_ATTR(cal_mode,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   NULL,
		   acc_cal_mode_store
		   );
static DEVICE_ATTR(start_cal,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   NULL,
		   acc_start_cal_store
		   );
static DEVICE_ATTR(cal_wait,
		   S_IRUSR|S_IRGRP,
		   acc_cal_wait_show,
		   NULL
		   );
static DEVICE_ATTR(offset,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   acc_offset_show,
		   acc_offset_store
		   );
static DEVICE_ATTR(ofs_en,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   acc_ofs_en_show,
		   acc_ofs_en_store
		   );
static DEVICE_ATTR(cal_smp_n,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_cal_smp_n_show,
		   accsns_cal_smp_n_store
		   );
static DEVICE_ATTR(cal_th,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   accsns_cal_th_show,
		   accsns_cal_th_store
		   );
static DEVICE_ATTR(imit_data,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   acc_imit_data_show,
		   acc_imit_data_store
		   );
static DEVICE_ATTR(dailys_enable,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   dailys_enable_show,
		   dailys_enable_store
		   );
static DEVICE_ATTR(dailys_param,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   dailys_param_show,
		   dailys_param_store
		   );
static DEVICE_ATTR(dailys_pedo_data,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   dailys_pedo_data_show,
		   NULL
		   );
static DEVICE_ATTR(dailys_vehi_data,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   dailys_vehi_data_show,
		   NULL
		   );
static DEVICE_ATTR(dailys_clear,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   NULL,
		   dailys_clear_store
		   );
static DEVICE_ATTR(dailys_iwifi_data,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   dailys_iwifi_data_show,
		   NULL
		   );
static DEVICE_ATTR(dailys_iwifi_param,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   dailys_iwifi_param_show,
		   dailys_iwifi_param_store
		   );

static struct attribute *accsns_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_data_raw.attr,
	&dev_attr_data.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_fw_update_sq.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_fw_version_chk.attr,
	&dev_attr_debug_level.attr,
	&dev_attr_test0_ctl.attr,
	&dev_attr_host_cmd.attr,
	&dev_attr_ope_device.attr,
	&dev_attr_cal_mode.attr,
	&dev_attr_start_cal.attr,
	&dev_attr_cal_wait.attr,
	&dev_attr_offset.attr,
	&dev_attr_ofs_en.attr,
	&dev_attr_cal_smp_n.attr,
	&dev_attr_cal_th.attr,
	&dev_attr_imit_data.attr,
	&dev_attr_dailys_enable.attr,
	&dev_attr_dailys_param.attr,
	&dev_attr_dailys_pedo_data.attr,
	&dev_attr_dailys_vehi_data.attr,
	&dev_attr_dailys_clear.attr,
	&dev_attr_dailys_iwifi_data.attr,
	&dev_attr_dailys_iwifi_param.attr,
	NULL
};

static struct attribute_group accsns_attribute_group = {
	.attrs = accsns_attributes
};

static int presns_input_init(void)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "pressure";
#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
	dev->id.bustype = BUS_I2C;
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
	dev->id.bustype = BUS_SPI;
#endif

	set_bit(EV_ABS, dev->evbit);
	input_set_abs_params(dev, ABS_X, ABSMIN_PA, ABSMAX_PA, 0, 0);
	input_set_abs_params(dev, ABS_RX, ABSMIN_CAL_PA, ABSMAX_CAL_PA, 0, 0);
	input_set_abs_params(dev, ABS_WAKE, INT_MIN, INT_MAX, 0, 0);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	pre_idev = dev;

	return 0;
}
static int32_t presns_get_delay(void)
{
	return atomic_read(&s_nPreData_DelayTime);
};

static int32_t presns_get_pre_active(void)
{
	int32_t iCurrentEnable;
    
	iCurrentEnable = atomic_read(&g_CurrentSensorEnable);
    
    return (iCurrentEnable & PRESNS_ACTIVE_PRE ? 1:0);
}

static void presns_set_delay(int32_t delay)
{

	mutex_lock(&s_tEnableMutex);

	if (presns_get_pre_active()) {
		cancel_delayed_work_sync(&s_tDelayWork_Pre_Data);
		atomic_set(&s_nPreData_DelayTime, delay);
		queue_delayed_work(presns_data_wq, &s_tDelayWork_Pre_Data, msecs_to_jiffies(delay));
	} else {
		atomic_set(&s_nPreData_DelayTime, delay);
	}

	mutex_unlock(&s_tEnableMutex);

};

static int32_t presns_pre_activate(bool arg_iEnable)
{
	int delay;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
	if(arg_iEnable == true)
	{
		delay = presns_get_delay();
		queue_delayed_work(presns_data_wq, &s_tDelayWork_Pre_Data, msecs_to_jiffies(delay));
		
	} else {
		cancel_delayed_work_sync(&s_tDelayWork_Pre_Data);
	}

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
  return ACCSNS_RC_OK;
}

static int32_t presns_power_onoff(bool arg_iEnable)
{
    int32_t   ret;
	uint8_t   sns_enable;
    HostCmd cmd;
    HostCmdRes res;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    if(arg_iEnable == true){
		cmd.cmd.udata16 = HC_MUL_GET_ANDROID;
		ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
		if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power On Error HC_MUL_GET_ANDROID(-) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
		}
		sns_enable = res.res.ub_res[0];

        cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
        cmd.prm.ub_prm[0] = 0x08 | res.res.ub_res[0];
		cmd.prm.ub_prm[1] = res.res.ub_res[1];
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power On Error HC_MUL_SET_ANDROID(HC_VALID) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
        }

		if(0 == sns_enable){
			cmd.cmd.udata16 = HC_MUL_MEASURE;
			cmd.prm.ub_prm[0] = MT_ANDROID_START;
			ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
			if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
				DBG(DBG_LV_ERROR, "Power On Error HC_MUL_MEASURE(MT_ANDROID_START) err %x\n", res.err.udata16);
				return ACCSNS_RC_ERR;
			}
		}
    }else{
		cmd.cmd.udata16 = HC_MUL_GET_ANDROID;
		ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
		if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power Off Error HC_MUL_GET_ANDROID(-) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
		}
		sns_enable = res.res.ub_res[0];

        cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
        cmd.prm.ub_prm[0] = res.res.ub_res[0] & 0xF7;
		cmd.prm.ub_prm[1] = res.res.ub_res[1];
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power Off Error HC_MUL_SET_ANDROID(%x) err %x\n", res.err.udata16,(res.res.ub_res[0] & 0xF7));
            return ACCSNS_RC_ERR;
        }

		if(0x08 == sns_enable){
			cmd.cmd.udata16 = HC_MUL_MEASURE;
			cmd.prm.ub_prm[0] = MT_ANDROID_STOP;
			ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
			if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power Off Error HC_MUL_MEASURE(MT_ANDROID_STOP) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
			}
		}
	}
    
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t presns_pre_getdata(struct pressure *arg_Pre)
{
    uint8_t ucBuff[MEASURE_DATA_SIZE];
    int32_t ret;
    
    ret = accsns_device_read(RSLT0E, ucBuff, sizeof(ucBuff));
    if(ACCSNS_RC_OK == ret){
    
        arg_Pre->usHeight = (int16_t)(ucBuff[6] | (ucBuff[7] << 8));
        
        arg_Pre->usTemp = (int16_t)(ucBuff[8] | (ucBuff[9] << 8));
        
        arg_Pre->usPressure = (int32_t)((ucBuff[10] << 9) | (ucBuff[11] << 1) | ((ucBuff[12] & 0x80) >> 7));

        arg_Pre->usAccuracy = (int32_t)ucBuff[13];

DBG(DBG_LV_LOW, "reg     Height:%02x %02x, usTemp:%02x %02x, usPressure:%02x %02x %02x usAccuracy:%02x\n",
					ucBuff[7],ucBuff[6],ucBuff[9],ucBuff[8],ucBuff[10],ucBuff[11],ucBuff[12],ucBuff[13]);
DBG(DBG_LV_LOW, "arg_Pre usHeight:%04x(%d), usTemp:%04x(%d), usPressure:%04x(%d),usAccuracy:%04x(%d)\n", 
					arg_Pre->usHeight, arg_Pre->usHeight, arg_Pre->usTemp, arg_Pre->usTemp, 
					arg_Pre->usPressure, arg_Pre->usPressure,arg_Pre->usAccuracy,arg_Pre->usAccuracy);
    }
    
    return ret;
}

int32_t presns_get_pressure_data( int32_t* arg_data )
{
    int32_t ret = ACCSNS_RC_OK;
	int32_t nCalP;
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        memset(arg_data, 0x00, (sizeof(int32_t)));
        return 0;
    }

    nCalP = atomic_read(&g_nCalP);

    if(delayed_work_pending(&s_tDelayWork_Pre)) {
DBG(DBG_LV_LOW, "pending...\n");
        
    } else {
	ret = accsns_measure(PRESNS_ACTIVE_PRE);
	}

    if(ret == ACCSNS_RC_OK){
        mutex_lock(&s_tDataMutex);
        s_tLatestPresData.usOutPressure = s_tLatestPresData.usPressure + nCalP;
		*arg_data = s_tLatestPresData.usOutPressure;
        mutex_unlock(&s_tDataMutex);
    }
  
DBG(DBG_LV_LOW, "Pre, s_tLatestPresData Pressure:%d OutPressure:%d\n",
						 s_tLatestPresData.usPressure,s_tLatestPresData.usOutPressure);
DBG(DBG_LV_LOW, "Pre, ret=%02d          Pressure:%d\n", ret, *arg_data);
    return ret;
}

static void presns_data_work_func(struct work_struct *work)
{
    int32_t ret = ACCSNS_RC_ERR;
    int32_t delay = 0;
	int32_t pre_data = 0;

DBG_PRINT_IO(0, 0);

	ret = presns_get_pressure_data(&pre_data);

	if(ret == ACCSNS_RC_OK){
		input_report_abs(pre_idev, ABS_X, ABS_X_DUMMY_VAL);
		input_report_abs(pre_idev, ABS_X, pre_data);
		input_sync(pre_idev);
	}

	delay = presns_get_delay();
	if (delay > 0)
		queue_delayed_work(presns_data_wq, &s_tDelayWork_Pre_Data, msecs_to_jiffies(delay));
	else
		DBG(DBG_LV_INFO, "%s(): delay=%d\n",__func__,delay);

DBG(DBG_LV_INFO, "presns_data_work_func:Compleate ret=%x, delay=%x\n", ret, delay);
DBG_PRINT_IO(0xFF, 0);
}

static int32_t presns_base_val(bool req, DailysSetBaseParam *DailysBaseParam)
{
    int32_t   ret;
    HostCmd cmd;
    HostCmdRes res;
DBG_PRINT_IO(0, 0);

	if(req == true){
		cmd.cmd.udata16 = HC_PRE_SET_BASE_VAL;
		cmd.prm.ub_prm[0] = 0x01;
		cmd.prm.ub_prm[1] = DailysBaseParam->m_SetMode;
		cmd.prm.ub_prm[2] = (DailysBaseParam->m_BasePress & 0xFF);
		cmd.prm.ub_prm[3] = ((DailysBaseParam->m_BasePress >> 8) & 0xFF);
		cmd.prm.ub_prm[4] = ((DailysBaseParam->m_BasePress >> 16) & 0xFF);
		cmd.prm.ub_prm[5] = (DailysBaseParam->m_BaseHeight & 0xFF);
		cmd.prm.ub_prm[6] = ((DailysBaseParam->m_BaseHeight >> 8) & 0xFF);
		cmd.prm.ub_prm[7] = ((DailysBaseParam->m_BaseHeight >> 16) & 0xFF);
		cmd.prm.ub_prm[8] = ((DailysBaseParam->m_BaseHeight >> 24) & 0xFF);

	    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
		if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Set Base Value HC_PRE_SET_BASE_VAL(-) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
		}
	} else {
		cmd.cmd.udata16 = HC_PRE_GET_BASE_VAL;

		ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
		if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Set Base Value HC_PRE_GET_BASE_VAL(-) err %x\n", res.err.udata16);
			return ACCSNS_RC_ERR;
		}
		DailysBaseParam->m_BasePress =  (uint32_t)((res.res.ub_res[4] << 16) | res.res.uw_res[1]);
		DailysBaseParam->m_BaseHeight = (uint32_t)((res.res.ub_res[8] << 24) | (res.res.uw_res[3] << 8) | res.res.ub_res[5]);

		DBG(DBG_LV_LOW, "HC_PRE_GET_BASE_VAL get, m_BasePress:%d, m_BaseHeight:%d\n",DailysBaseParam->m_BasePress,DailysBaseParam->m_BaseHeight);
	}

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

int32_t presns_calibration_mode(void)
{
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));

    s_tPreCalibCtrl.m_bWaitSetting         = true;
    s_tPreCalibCtrl.m_bComplete            = false;
	s_tPreCalibCtrl.m_bRengeChkErr         = false;

	s_tPreCalibCtrl.m_nP0                  = 0;
	s_tPreCalibCtrl.m_nRefP                = 0;
    s_tPreCalibCtrl.m_nCalP                = 0;

    s_tPreCalibCtrl.m_nCurrentSampleNum    = 0;
	s_tPreCalibCtrl.m_nSummationSample     = 0;
    s_tPreCalibCtrl.m_nSummationPa         = 0;
    s_tPreCalibCtrl.m_nSummationPb         = 0;
	s_tPreCalibCtrl.m_nSummationPa_Sample  = 0;
	s_tPreCalibCtrl.m_nSummationPb_Sample  = 0;
	s_tPreCalibCtrl.m_nSummationSample_squ = 0;
	s_tPreCalibCtrl.m_nMode = -1;

DBG(DBG_LV_INFO, "Enter PreCalibration Mode\n");
    mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

int32_t presns_calibration_start(int32_t pres_mode, int32_t PressureBase)
{
    int32_t delay = 0;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
    s_tPreCalibCtrl.m_bWaitSetting = false;
	s_tPreCalibCtrl.m_nMode = pres_mode;
	s_tPreCalibCtrl.m_nPressureBase = PressureBase;

	if(s_tPreCalibCtrl.m_nMode == MODE_2){
		s_tPreCalibCtrl.m_bComplete = false;
	}

	s_tPreCalibCtrl.m_nP0                  = 0;
    s_tPreCalibCtrl.m_nCurrentSampleNum    = 0;
	s_tPreCalibCtrl.m_nSummationSample     = 0;
    s_tPreCalibCtrl.m_nSummationPa         = 0;
    s_tPreCalibCtrl.m_nSummationPb         = 0;
	s_tPreCalibCtrl.m_nSummationPa_Sample  = 0;
	s_tPreCalibCtrl.m_nSummationPb_Sample  = 0;
	s_tPreCalibCtrl.m_nSummationSample_squ = 0;
	s_tPreCalibCtrl.m_nSmpMaxP             = 0;
	s_tPreCalibCtrl.m_nSmpMinP             = 0;

DBG(DBG_LV_INFO, "Start PreCalibration delay[%d]\n",delay);
    mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));

    delay = atomic_read(&s_nPreData_DelayTime);
	queue_delayed_work(presns_cal_wq, &s_tDelayWork_Pre, msecs_to_jiffies(delay));

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

int32_t presns_calibration_is_wait(void)
{
    int32_t wait;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
    wait = s_tPreCalibCtrl.m_bWaitSetting;
	if(wait == true && s_tPreCalibCtrl.m_nMode == MODE_2){
		wait = 2;
	}
	if(s_tPreCalibCtrl.m_bRengeChkErr == true){
		wait = 3;
	}
    mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));

DBG(DBG_LV_INFO, "calib Wait %d\n", wait);
DBG_PRINT_IO(0xFF, wait);
    return wait;
}

int32_t presns_calibration_is_comp(int32_t* argCal)
{
    int32_t comp;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
    if(s_tPreCalibCtrl.m_bComplete == true) {
        *argCal = s_tPreCalibCtrl.m_nCalP;
    }
    
    comp = s_tPreCalibCtrl.m_bComplete;
    mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));

DBG(DBG_LV_INFO, "calib Wait %d\n", comp);
DBG_PRINT_IO(0xFF, comp);
    return comp;
}

static void presns_calibration_periodic(const struct pressure* preData)
{
	int32_t a_data,b_data;
	int32_t sample_num,summ_samp,summ_pa,summ_pa_samp,summ_pb,summ_pb_samp,summ_squ;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
DBG(DBG_LV_INFO, "calibPeriodic:%d\n",s_tPreCalibCtrl.m_nCurrentSampleNum);
    
    if(s_tPreCalibCtrl.m_nCurrentSampleNum == s_tPreCalibCtrl.m_unSmpN) {
        s_tPreCalibCtrl.m_bComplete    = true;
        s_tPreCalibCtrl.m_bWaitSetting = true;
        
    } else {
		presns_calibration_range_chk(preData);

		if(s_tPreCalibCtrl.m_nCurrentSampleNum == 0){
			s_tPreCalibCtrl.m_nP0 = preData->usPressure;
		}

		s_tPreCalibCtrl.m_nSummationSample += (s_tPreCalibCtrl.m_nCurrentSampleNum + 1);
        s_tPreCalibCtrl.m_nSummationPa += (preData->usPressure - s_tPreCalibCtrl.m_nP0) * 100;
		s_tPreCalibCtrl.m_nSummationPb += (preData->usPressure - s_tPreCalibCtrl.m_nP0);
		s_tPreCalibCtrl.m_nSummationPa_Sample += ((s_tPreCalibCtrl.m_nCurrentSampleNum + 1) * ((preData->usPressure - s_tPreCalibCtrl.m_nP0) * 100));
		s_tPreCalibCtrl.m_nSummationPb_Sample += ((s_tPreCalibCtrl.m_nCurrentSampleNum + 1) * (preData->usPressure - s_tPreCalibCtrl.m_nP0));
		s_tPreCalibCtrl.m_nSummationSample_squ += ((s_tPreCalibCtrl.m_nCurrentSampleNum + 1) * (s_tPreCalibCtrl.m_nCurrentSampleNum + 1));
        s_tPreCalibCtrl.m_nCurrentSampleNum++;

        if(s_tPreCalibCtrl.m_nCurrentSampleNum == s_tPreCalibCtrl.m_unSmpN && 
			s_tPreCalibCtrl.m_bRengeChkErr == false) {

			sample_num = s_tPreCalibCtrl.m_nCurrentSampleNum;
			summ_samp  = s_tPreCalibCtrl.m_nSummationSample;
			summ_pa     = s_tPreCalibCtrl.m_nSummationPa;
			summ_pb     = s_tPreCalibCtrl.m_nSummationPb;
			summ_pa_samp = s_tPreCalibCtrl.m_nSummationPa_Sample;
			summ_pb_samp = s_tPreCalibCtrl.m_nSummationPb_Sample;
			summ_squ   = s_tPreCalibCtrl.m_nSummationSample_squ;

			if(s_tPreCalibCtrl.m_nMode == MODE_0){
				a_data = ((summ_pa_samp - (summ_pa / sample_num * summ_samp))) / (summ_squ - (summ_samp / sample_num * summ_samp));
				b_data = ((summ_squ / sample_num * summ_pb) - (summ_pb_samp / sample_num * summ_samp)) / (summ_squ - (summ_samp / sample_num * summ_samp));
DBG(DBG_LV_DATA, "!!!!! PreCalib: a_data = %d b_data = %d\n",a_data,b_data);
DBG(DBG_LV_DATA, "!!!!! PreCalib:A (summ_pa / sample_num * summ_samp) = %d (summ_squ - (summ_samp / sample_num * summ_samp)) = %d\n",
						(summ_pa / sample_num * summ_samp),(summ_squ - (summ_samp / sample_num * summ_samp)));

				s_tPreCalibCtrl.m_nRefP = (a_data * sample_num / 100) + b_data + s_tPreCalibCtrl.m_nP0;
				atomic_set(&g_nRefP, s_tPreCalibCtrl.m_nRefP);
DBG(DBG_LV_DATA, "PreCalib:Reference  m_nRefP = %d m_nP0 = %d sample_num = %d summ_samp=%d summ_pa=%d summ_pa_samp=%d summ_pb=%d summ_pb_samp=%d summ_squ=%d \n",
				s_tPreCalibCtrl.m_nRefP,s_tPreCalibCtrl.m_nP0, sample_num, summ_samp, summ_pa, summ_pa_samp, summ_pb, summ_pb_samp, summ_squ);
			} else if(s_tPreCalibCtrl.m_nMode == MODE_2){
				a_data = ((summ_pa_samp - (summ_pa / sample_num * summ_samp))) / (summ_squ - (summ_samp / sample_num * summ_samp));
				b_data = ((summ_squ / sample_num * summ_pb) - (summ_pb_samp / sample_num * summ_samp)) / (summ_squ - (summ_samp / sample_num * summ_samp));
DBG(DBG_LV_DATA, "!!!!! PreCalib: a_data = %d b_data = %d\n",a_data,b_data);
DBG(DBG_LV_DATA, "!!!!! PreCalib:A (summ_pa / sample_num * summ_samp) = %d (summ_squ - (summ_samp / sample_num * summ_samp)) = %d\n",
						(summ_pa / sample_num * summ_samp),(summ_squ - (summ_samp / sample_num * summ_samp)));

				s_tPreCalibCtrl.m_nCalP = s_tPreCalibCtrl.m_nPressureBase - ((a_data / 100) + b_data + s_tPreCalibCtrl.m_nP0);
            atomic_set(&g_nCalP, s_tPreCalibCtrl.m_nCalP);
			input_report_abs(pre_idev, ABS_RX, s_tPreCalibCtrl.m_nCalP);
DBG(DBG_LV_DATA, "PreCalib:complete calp = %d m_nP0 = %d sample_num = %d summ_samp=%d summ_pa=%d summ_pa_samp=%d summ_pb=%d summ_pb_samp=%d summ_squ=%d \n",
					s_tPreCalibCtrl.m_nCalP,s_tPreCalibCtrl.m_nP0, sample_num, summ_samp, summ_pa, summ_pa_samp, summ_pb, summ_pb_samp, summ_squ);
			}
        }
    }
    mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));
    
DBG_PRINT_IO(0xFF, 0);
}

static void presns_pre_work_func(struct work_struct *work)
{
    int32_t ret = ACCSNS_RC_ERR;
    int32_t delay = 0;
    bool bCalibIdle = false;
    bool bCalibComp = false;
DBG_PRINT_IO(0, 0);
    
    ret = accsns_measure(PRESNS_ACTIVE_PRE);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_measure Error err %x\n", ret);
        return;
    }
    
    mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
    bCalibIdle = s_tPreCalibCtrl.m_bWaitSetting;
    bCalibComp = s_tPreCalibCtrl.m_bComplete;
    mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));
    
    if((bCalibIdle == false) && (bCalibComp == false)){
    
        mutex_lock(&s_tDataMutex);
        presns_calibration_periodic(&s_tLatestPresData);
        mutex_unlock(&s_tDataMutex);
        
        mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
        bCalibComp = s_tPreCalibCtrl.m_bComplete;
        mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));
        if(bCalibComp == false){
            delay = atomic_read(&s_nPreData_DelayTime);
			queue_delayed_work(presns_cal_wq, &s_tDelayWork_Pre, msecs_to_jiffies(delay));
        }
    }
    
DBG(DBG_LV_INFO, "presns_pre_work_func:Compleate ret=%x, bCalibIdle=%x, delay=%x\n", ret, bCalibIdle, delay);
DBG_PRINT_IO(0xFF, 0);
}
void presns_set_offset(int32_t offset)
{
DBG_PRINT_IO(0, 0);
    
    atomic_set(&g_nCalP, (int32_t)offset);

DBG(DBG_LV_INFO, "set offset P %d\n", offset);

DBG_PRINT_IO(0xFF, 0);
}

static void presns_calibration_range_chk(const struct pressure* preData)
{
	int32_t presns_threshold = 0;

	DBG(DBG_LV_INFO, "range_chk P:%d\n",preData->usPressure);

	if(s_tPreCalibCtrl.m_nCurrentSampleNum == 0){
		s_tPreCalibCtrl.m_nSmpMaxP = preData->usPressure;
		s_tPreCalibCtrl.m_nSmpMinP = preData->usPressure;
	}

	if(s_tPreCalibCtrl.m_nSmpMaxP < preData->usPressure){
		s_tPreCalibCtrl.m_nSmpMaxP = preData->usPressure;
	} else if(s_tPreCalibCtrl.m_nSmpMinP > preData->usPressure){
		s_tPreCalibCtrl.m_nSmpMinP = preData->usPressure;
	}

	presns_threshold = s_tPreCalibCtrl.m_nSmpMaxP - s_tPreCalibCtrl.m_nSmpMinP;

	if(presns_threshold >= s_tPreCalibCtrl.m_nCalTH){
		s_tPreCalibCtrl.m_bRengeChkErr = true;
DBG(DBG_LV_ERROR, "PRE-Calib: Range Chk Err m_bRengeChkErr :%d\n",s_tPreCalibCtrl.m_bRengeChkErr);
	}
}

static ssize_t presns_enable_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int enable;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	mutex_lock(&s_tEnableMutex);
	enable = presns_get_pre_active();
	mutex_unlock(&s_tEnableMutex);

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t presns_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	unsigned long arg_iEnable;
	int32_t ret;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[PRE] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }

	ret = strict_strtoul(buf, 10, &arg_iEnable);
	if (ret < 0)
		return count;

	mutex_lock(&s_tEnableMutex);
	flgP += (arg_iEnable != 0)? 1: -1;
	if(flgP < 0)
		flgP = 0;
    arg_iEnable = (flgP == 0)? POWER_DISABLE : POWER_ENABLE;
	accsns_activateEx(1,PRESNS_ACTIVE_PRE,arg_iEnable);
	mutex_unlock(&s_tEnableMutex);

	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR PRESNS_ACTIVE_PRE:%s\n", __FUNCTION__);
		accsns_err_check();
	}

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t presns_delay_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	DBG(DBG_LV_INFO, "%s():\n",__func__);
	return sprintf(buf, "%d\n", presns_get_delay());
}

static ssize_t presns_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	unsigned long delay;
	int ret;
	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	ret = strict_strtoul(buf, 10, &delay);
	if (ret < 0)
		return count;
	DBG(DBG_LV_INFO, "%s(): delay = %ld\n",__func__,delay);

	presns_set_delay(delay);

	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t presns_data_raw_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct pressure pres;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	mutex_lock(&s_tDataMutex);
	pres = s_tLatestPresData;
	mutex_unlock(&s_tDataMutex);

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d %d %d %d\n", pres.usHeight,pres.usTemp,pres.usPressure,pres.usAccuracy);
}

static ssize_t presns_data_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct pressure pres;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	mutex_lock(&s_tDataMutex);
	pres = s_tLatestPresData;
	mutex_unlock(&s_tDataMutex);

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d\n",pres.usOutPressure);
}

static ssize_t presns_wake_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	static int cnt = 1;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	input_report_abs(pre_idev, ABS_WAKE, cnt++);
	input_sync(pre_idev);

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t presns_param_base_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int32_t ret;
	DailysSetBaseParam DailysBaseParam;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	memset(&DailysBaseParam, 0x00, sizeof(DailysBaseParam));

	mutex_lock(&s_tEnableMutex);
	ret = presns_base_val(false, &DailysBaseParam);
	mutex_unlock(&s_tEnableMutex);

	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR presns_base_val:%s\n", __FUNCTION__);
	}

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d %d\n", DailysBaseParam.m_BasePress,DailysBaseParam.m_BaseHeight);
}

static ssize_t presns_param_base_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret;
	DailysSetBaseParam DailysBaseParam;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return count;
    }
	sscanf(buf, "%d %d %d", &DailysBaseParam.m_SetMode,&DailysBaseParam.m_BasePress,
							&DailysBaseParam.m_BaseHeight);

	mutex_lock(&s_tEnableMutex);
	ret = presns_base_val(true, &DailysBaseParam);
	mutex_unlock(&s_tEnableMutex);
	if(ret < 0 ){
DBG(DBG_LV_ERROR, "[ACC] ERROR presns_base_val:%s\n", __FUNCTION__);
	}

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static ssize_t presns_cal_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret = 0;
	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);

	ret = presns_calibration_mode();
	if(ret != 0){
		DBG(DBG_LV_ERROR, "error(presns_calibration_mode) : cal_mode\n");
		return count;
	}

	DBG(DBG_LV_INFO,"%s(): [OUT] ret = %d\n",__func__,ret);
	return count;
}

static ssize_t presns_start_cal_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int32_t nRefP;
DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	nRefP = atomic_read(&g_nRefP);

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
	return sprintf(buf, "%d\n", nRefP);
}

static ssize_t presns_start_cal_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret = 0;
	uint32_t pres_mode,pres_base;

	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	sscanf(buf, "%d %d", &pres_mode, &pres_base);
	DBG(DBG_LV_INFO,"%s(): pres_mode = %d pres_base = %d\n",__func__,pres_mode,pres_base);

	ret = presns_calibration_start(pres_mode,pres_base);
	if(ret != 0){
		DBG(DBG_LV_ERROR, "error(presns_calibration_start) : start_cal\n");
		return count;
	}

	DBG(DBG_LV_INFO,"%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t presns_cal_wait_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	DBG(DBG_LV_INFO,"%s():\n",__func__);

	return sprintf(buf, "%d\n", presns_calibration_is_wait());
}

static ssize_t presns_offset_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{

	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	return sprintf(buf, "%d\n", atomic_read(&g_nCalP));
}

static ssize_t presns_offset_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t offset;

	DBG(DBG_LV_INFO,"%s(): [IN]\n",__func__);
	sscanf(buf, "%d", &offset);

	DBG(DBG_LV_INFO,"%s(): offset = %d\n",__func__,offset);
	presns_set_offset(offset);

	DBG(DBG_LV_INFO,"%s(): [OUT]\n",__func__);
	return count;
}

static ssize_t presns_cal_smp_n_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int cal_smp_n;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
	cal_smp_n = s_tPreCalibCtrl.m_unSmpN;
    mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));
	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);

	return sprintf(buf, "%d\n", cal_smp_n);
}

static ssize_t presns_cal_smp_n_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret = 0;
	unsigned long arg_iCal_smp_n;

DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	ret = strict_strtoul(buf, 10, &arg_iCal_smp_n);
	if (ret < 0)
		return count;
DBG(DBG_LV_INFO, "%s(): Cal_smp_n = %ld\n",__func__,arg_iCal_smp_n);

	mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
	s_tPreCalibCtrl.m_unSmpN = arg_iCal_smp_n;
	mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}


static ssize_t presns_cal_th_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int cal_th;

	DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);
    mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
	cal_th = s_tPreCalibCtrl.m_nCalTH;
    mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));
	DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);

	return sprintf(buf, "%d\n", cal_th);
}

static ssize_t presns_cal_th_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int32_t ret = 0;
	unsigned long arg_iCal_th;

DBG(DBG_LV_INFO, "%s(): [IN]\n",__func__);

	ret = strict_strtoul(buf, 10, &arg_iCal_th);
	if (ret < 0)
		return count;
DBG(DBG_LV_INFO, "%s(): Cal_th = %ld\n",__func__,arg_iCal_th);

	mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));
	s_tPreCalibCtrl.m_nCalTH = arg_iCal_th;
	mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));

DBG(DBG_LV_INFO, "%s(): m_nCalTH = %d \n",__func__,s_tPreCalibCtrl.m_nCalTH);

DBG(DBG_LV_INFO, "%s(): [OUT]\n",__func__);
    return count;
}

static DEVICE_ATTR(presns_enable,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   presns_enable_show,
		   presns_enable_store
		   );
static DEVICE_ATTR(presns_delay,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   presns_delay_show,
		   presns_delay_store
		   );
static DEVICE_ATTR(presns_data_raw,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   presns_data_raw_show,
		   NULL
		   );
static DEVICE_ATTR(presns_data,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   presns_data_show,
		   NULL
		   );
static DEVICE_ATTR(presns_wake,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   NULL,
		   presns_wake_store
		   );
static DEVICE_ATTR(presns_param_base,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   presns_param_base_show,
		   presns_param_base_store
		   );
static DEVICE_ATTR(presns_cal_mode,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   NULL,
		   presns_cal_mode_store
		   );
static DEVICE_ATTR(presns_start_cal,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   presns_start_cal_show,
		   presns_start_cal_store
		   );
static DEVICE_ATTR(presns_cal_wait,
		   S_IRUSR|S_IRGRP,
		   presns_cal_wait_show,
		   NULL
		   );
static DEVICE_ATTR(presns_offset,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   presns_offset_show,
		   presns_offset_store
		   );
static DEVICE_ATTR(presns_cal_smp_n,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   presns_cal_smp_n_show,
		   presns_cal_smp_n_store
		   );
static DEVICE_ATTR(presns_cal_th,
		   S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		   presns_cal_th_show,
		   presns_cal_th_store
		   );

static struct attribute *presns_attributes[] = {
	&dev_attr_presns_enable.attr,
	&dev_attr_presns_delay.attr,
	&dev_attr_presns_data_raw.attr,
	&dev_attr_presns_data.attr,
	&dev_attr_presns_wake.attr,
	&dev_attr_presns_param_base.attr,
	&dev_attr_presns_cal_mode.attr,
	&dev_attr_presns_start_cal.attr,
	&dev_attr_presns_cal_wait.attr,
	&dev_attr_presns_offset.attr,
	&dev_attr_presns_cal_smp_n.attr,
	&dev_attr_presns_cal_th.attr,
	NULL
};

static struct attribute_group presns_attribute_group = {
	.attrs = presns_attributes
};

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static int32_t accsns_probe( struct i2c_client *client,const struct i2c_device_id *id)
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static int32_t accsns_probe( struct spi_device *client )
#endif
{
    int32_t i;
    int32_t ret;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "[ACC] accsns_probe\n");

    g_acc_nv_param.pedo_p.param_on             = 0;
    g_acc_nv_param.pedo_p.on                   = 0;
    g_acc_nv_param.pedo_p.stepwide             = 40;
    g_acc_nv_param.pedo_p.weight               = 60;
    g_acc_nv_param.pedo_p.notify               = 0;
    g_acc_nv_param.pedo_p.speed_ave_time       = 3;
    g_acc_nv_param.pedo_p.mets_stop_time       = 10;
    g_acc_nv_param.pedo_p.bodyfat_on           = 0;
    g_acc_nv_param.pedo_p.bodyfat_cal          = 72;
    g_acc_nv_param.dist_stop_p.stop_notify_on  = 0;
    g_acc_nv_param.dist_stop_p.stop_notify_time= 60;
    g_acc_nv_param.walk_run_p.walk_judge_on    = 0;
    g_acc_nv_param.walk_run_p.consecutive_num  = 1;
    g_acc_nv_param.walk_run_p.speed_th         = 167;
    g_acc_nv_param.trans_p.on                  = 1;
    g_acc_nv_param.trans_p.judge_step          = 50;
    g_acc_nv_param.trans_p.judge_time          = 300;
    g_acc_nv_param.trans_p.calc_time           = 2;
    g_acc_nv_param.trans_p.consecutive_num     = 30;
    g_acc_nv_param.trans_byc_p.consecutive_num = 3;
    g_acc_nv_param.trans_byc_p.calory_mets     = 50;
    g_acc_nv_param.timer_p.on                  = 1;
    g_acc_nv_param.timer_p.num                 = 120000;
    g_acc_nv_param.move_p.on                   = 1;
    g_acc_nv_param.move_p.axis                 = 0x07;
    g_acc_nv_param.move_p.acc_th               = 154;
    g_acc_nv_param.move_p.judge_num            = 5;
    g_acc_nv_param.move_p.judge_th             = 3;
    g_acc_nv_param.move_p.judge                = 1;

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
	/* Setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
DBG(DBG_LV_ERROR, "%s(): err i2c_check_functionality\n",__func__);
		return -ENODEV;
	}
#endif
    client_accsns = client;

#if CONFIG_INPUT_ML610Q793_INTERFACE == 1
    client_accsns->bits_per_word = 8;
#endif

    g_nIntIrqNo = -1;
    g_nIntIrqFlg = 0;
    atomic_set(&g_flgEna, ACTIVE_OFF);
    atomic_set(&g_CurrentSensorEnable,ACTIVE_OFF);
    atomic_set(&g_WakeupSensor,ACTIVE_OFF);
    atomic_set(&g_FWUpdateStatus,false);
    g_bDevIF_Error = false;
    work_ped_pos = 0;
    dummy_on_flg = false;
	g_CurrentSensorEnable_backup = ACTIVE_OFF;
	atomic_set(&g_ResetStatus,false);
	atomic_set(&g_Dailys_IntType,0);

    atomic_set(&g_nCalX,0);
    atomic_set(&g_nCalY,0);
    atomic_set(&g_nCalZ,0);
	atomic_set(&g_nCalP,0);
	atomic_set(&g_nOfs_en,0);
	atomic_set(&g_nRefP,0);
    
    atomic_set(&g_nWeight, DEFAULT_WEIGHT);
    atomic_set(&g_nStepWide, DEFAULT_PEDOMETER);
    atomic_set(&g_nVehiType, DEFAULT_VEHITYPE);
	atomic_set(&s_nDataDelayTime, DEFAULT_ACC_DATA_DELAY);
	atomic_set(&s_nPreData_DelayTime, DEFAULT_PRE_DATA_DELAY);

    memset(&s_tLatestAccData, 0x00, sizeof(s_tLatestAccData));
    memset(&s_tLatestPedoData, 0x00, sizeof(s_tLatestPedoData));
    memset(&s_tLatestVehiData, 0x00, sizeof(s_tLatestVehiData));
	memset(&s_tLatestPresData, 0x00, sizeof(s_tLatestPresData));
	memset(&s_tLatestIWifiData, 0x00, sizeof(s_tLatestIWifiData));

    init_waitqueue_head(&s_tWaitInt);
    INIT_DELAYED_WORK(&s_tDelayWork_Acc, accsns_acc_work_func);
	INIT_DELAYED_WORK(&s_tDelayWork_Acc_Data, accsns_data_work_func);
	INIT_DELAYED_WORK(&s_tDelayWork_Pre, presns_pre_work_func);
	INIT_DELAYED_WORK(&s_tDelayWork_Pre_Data, presns_data_work_func);
    accsns_workqueue_init();
    
    mutex_init(&s_tDataMutex);
    mutex_init(&(s_tCalibCtrl.m_tCalibMutex));
    mutex_init(&(s_tPreCalibCtrl.m_tCalibMutex));
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
	mutex_init(&s_tEnableMutex);
    
    s_tCalibCtrl.m_bFilterEnable        = false;
    s_tCalibCtrl.m_bWaitSetting         = true;
    s_tCalibCtrl.m_bComplete            = true;
	s_tCalibCtrl.m_bRengeChkErr         = false;
    s_tCalibCtrl.m_unSmpN               = OFFSET_SUMPLE_NUM; 

    s_tCalibCtrl.m_tFilterWork.m_ucAveN = OFFSET_AVE_NUM;   
    s_tCalibCtrl.m_nCalX                = 0;
    s_tCalibCtrl.m_nCalY                = 0;
    s_tCalibCtrl.m_nCalZ                = 0;
    
    s_tCalibCtrl.m_nCurrentSampleNum    = 0;
    s_tCalibCtrl.m_nSummationX          = 0;
    s_tCalibCtrl.m_nSummationY          = 0;
    s_tCalibCtrl.m_nSummationZ          = 0;
    s_tCalibCtrl.m_nHighTH              = OFFSET_ACC_HIGH_TH;
    s_tCalibCtrl.m_nLowTH               = OFFSET_ACC_LOW_TH;

    s_tCalibCtrl.m_nMode                = -1; 
    for (i = 0; i < AXIS_XYZ_MAX; i++) {
        if(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] != NULL) {
DBG(DBG_LV_INFO, "accsns_probe:Calib err occurd \n");
        } else {
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] = NULL;
        }
    }
    memset(s_tCalibCtrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    memset(s_tCalibCtrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
    
    mutex_lock(&(s_tPreCalibCtrl.m_tCalibMutex));

	s_tPreCalibCtrl.m_bWaitSetting = true;
	s_tPreCalibCtrl.m_bComplete    = true;
	s_tPreCalibCtrl.m_bRengeChkErr = false;

	s_tPreCalibCtrl.m_unSmpN            = OFFSET_PRE_SUMPLE_NUM;
	s_tPreCalibCtrl.m_nP0               = 0;
	s_tPreCalibCtrl.m_nRefP             = 0;
	s_tPreCalibCtrl.m_nCalP             = 0;
	s_tPreCalibCtrl.m_nCurrentSampleNum = 0;
	s_tPreCalibCtrl.m_nSummationSample  = 0;
	s_tPreCalibCtrl.m_nSummationPa      = 0;
	s_tPreCalibCtrl.m_nSummationPa_Sample= 0;
	s_tPreCalibCtrl.m_nSummationPb      = 0;
	s_tPreCalibCtrl.m_nSummationPb_Sample= 0;
	s_tPreCalibCtrl.m_nSummationSample_squ = 0;
	s_tPreCalibCtrl.m_nPressureBase     = 0;
	s_tPreCalibCtrl.m_nSmpMaxP          = 0;
	s_tPreCalibCtrl.m_nSmpMinP          = 0;
	s_tPreCalibCtrl.m_nCalTH            = OFFSET_PRE_TH;
	s_tPreCalibCtrl.m_nMode = -1;

    mutex_unlock(&(s_tPreCalibCtrl.m_tCalibMutex));

	g_bIsAlreadyExistAccImitationXYZData = false;
	for(i=0;i<3;i++)
		g_naAccImitationData[i] = 0;

	ret = misc_register(&dailys_device_pedom);
	if (ret) {
		printk("accsns_probe: dailys_io_pedom(pedom) register failed\n");
		return ret;
	}

	ret = misc_register(&dailys_device_vehicle);
	if (ret) {
		printk("accsns_probe: dailys_io_vehicle(vehicle) register failed\n");
		return ret;
	}

	ret = misc_register(&dailys_device_iwifi);
	if (ret) {
		printk("accsns_probe: dailys_io_iwifi(iwifi) register failed\n");
		return ret;
	}

    if(g_nIntIrqNo == -1){
        ret = accsns_gpio_init();
        if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Failed accsns_gpio_init. ret=%x\n", ret);
            return ret;
        }
        DISABLE_IRQ;
    }

	ret = accsns_input_init();
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Failed accsns_input_init. ret=%x\n", ret);
        return -ENODEV;
    }

	ret = sysfs_create_group(&acc_idev->dev.kobj,
				 &accsns_attribute_group);
	if (ret < 0)
	{
DBG(DBG_LV_ERROR, "Failed sysfs_create_group. accsns ret=%x\n", ret);
        return -ENODEV;
	}

	ret = presns_input_init();
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Failed presns_input_init. ret=%x\n", ret);
        return -ENODEV;
    }

	ret = sysfs_create_group(&pre_idev->dev.kobj,
				 &presns_attribute_group);
	if (ret < 0)
	{
DBG(DBG_LV_ERROR, "Failed sysfs_create_group. presns ret=%x\n", ret);
        return -ENODEV;
	}

    ret = accsns_initialize();
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Failed accsns_initialize. ret=%x\n", ret);
        return -ENODEV;
    }

	acc_pre_power_cb.power_on  = acc_pre_power_on;
	acc_pre_power_cb.power_off = acc_pre_power_off;
	sensor_power_reg_cbfunc(&acc_pre_power_cb);

DBG(DBG_LV_INFO, "Init Complete!!!\n");
DBG_PRINT_IO(0xFF, 0);
    return 0;
}
#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
static const struct i2c_device_id accsns_id[] = {
	{ML610Q793_ACC_KERNEL_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, accsns_id);
static struct i2c_driver accsns_driver = {
	.driver = {
		.name = ML610Q793_ACC_KERNEL_NAME,
		.owner = THIS_MODULE,
	},
	.probe = accsns_probe,
	.remove = accsns_remove,
	.suspend = accsns_suspend,
	.resume = accsns_resume,
    .shutdown = accsns_shutdown,
	.id_table = accsns_id,
};
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
static struct spi_driver accsns_driver = {
    .probe       = accsns_probe,
    .driver = {
        .name    = ACC_DRIVER_NAME,
        .bus     = &spi_bus_type,
        .owner   = THIS_MODULE,
    },
    .remove      = accsns_remove,
    .suspend     = accsns_suspend,
    .resume      = accsns_resume,
    .shutdown    = accsns_shutdown,
};
#endif

static struct early_suspend accsns_early_suspend_handler = {
  .level	= EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
  .suspend	= accsns_early_suspend,
  .resume 	= accsns_late_resume,
}; 

static int32_t __init accsns_init( void )
{
    int32_t ret;
DBG_PRINT_IO(0, 0);
    
    sema_init(&s_tAccSnsrSema, 1);
    init_waitqueue_head(&s_tPollWaitPedom);
    init_waitqueue_head(&s_tPollWaitVehicle);
    init_waitqueue_head(&s_tPollWaitIWifi);

    accsns_wq_int = create_singlethread_workqueue("accsns_wq_int");
    if(!accsns_wq_int)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : accsns_wq_int \n");
        return -ENODEV;
    }
    accsns_wq = create_singlethread_workqueue("accsns_wq");
    if(!accsns_wq)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : accsns_wq \n");
        goto REGIST_ERR;
    }
    fw_update_wq = create_singlethread_workqueue("fw_update_wq");
    if(!fw_update_wq)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : fw_update_wq \n");
        goto REGIST_ERR;
    }

    accsns_cal_wq = create_singlethread_workqueue("accsns_cal_wq");
    if(!accsns_cal_wq)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : accsns_cal_wq \n");
        goto REGIST_ERR;
    }

    accsns_data_wq = create_singlethread_workqueue("accsns_data_wq");
    if(!accsns_data_wq)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : accsns_data_wq \n");
        goto REGIST_ERR;
    }

    presns_cal_wq = create_singlethread_workqueue("presns_cal_wq");
    if(!presns_cal_wq)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : presns_cal_wq \n");
        goto REGIST_ERR;
    }

    presns_data_wq = create_singlethread_workqueue("presns_data_wq");
    if(!presns_data_wq)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : presns_data_wq \n");
        goto REGIST_ERR;
    }

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
    ret = i2c_add_driver(&accsns_driver);
    if(ret != 0){
DBG(DBG_LV_ERROR, "can't regist i2c driver \n");
        goto REGIST_ERR;
    }

#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
    ret = spi_register_driver(&accsns_driver);
    if(ret != 0){
DBG(DBG_LV_ERROR, "can't regist spi driver \n");
        goto REGIST_ERR;
    }
#endif

    register_early_suspend(&accsns_early_suspend_handler);

    return 0;
    
REGIST_ERR:
    if(accsns_wq_int != NULL){
        flush_workqueue(accsns_wq_int);
        destroy_workqueue(accsns_wq_int);
        accsns_wq_int = NULL;
    }
    if(accsns_wq != NULL){
        flush_workqueue(accsns_wq);
        destroy_workqueue(accsns_wq);
        accsns_wq = NULL;
    }

    if(fw_update_wq != NULL){
        flush_workqueue(fw_update_wq);
        destroy_workqueue(fw_update_wq);
        fw_update_wq = NULL;
    }

    if(accsns_cal_wq != NULL){
        flush_workqueue(accsns_cal_wq);
        destroy_workqueue(accsns_cal_wq);
        accsns_cal_wq = NULL;
    }

    if(accsns_data_wq != NULL){
        flush_workqueue(accsns_data_wq);
        destroy_workqueue(accsns_data_wq);
        accsns_data_wq = NULL;
    }

    if(presns_cal_wq != NULL){
        flush_workqueue(presns_cal_wq);
        destroy_workqueue(presns_cal_wq);
        presns_cal_wq = NULL;
    }

    if(presns_data_wq != NULL){
        flush_workqueue(presns_data_wq);
        destroy_workqueue(presns_data_wq);
        presns_data_wq = NULL;
    }

    return -ENODEV;
}

static void __exit accsns_exit( void )
{
    unregister_early_suspend(&accsns_early_suspend_handler);
    
    DISABLE_IRQ;
    accsns_workqueue_init();
    
    gpio_free(ACCSNS_GPIO_INT);
    gpio_free(ACCSNS_GPIO_RST);

    if(accsns_wq_int != NULL){
        flush_workqueue(accsns_wq_int);
        destroy_workqueue(accsns_wq_int);
        accsns_wq_int = NULL;
    }
    if(accsns_wq != NULL){
        flush_workqueue(accsns_wq);
        destroy_workqueue(accsns_wq);
        accsns_wq = NULL;
    }
    if(fw_update_wq != NULL){
        flush_workqueue(fw_update_wq);
        destroy_workqueue(fw_update_wq);
        fw_update_wq = NULL;
    }

    if(accsns_cal_wq != NULL){
        flush_workqueue(accsns_cal_wq);
        destroy_workqueue(accsns_cal_wq);
        accsns_cal_wq = NULL;
    }

    if(accsns_data_wq != NULL){
        flush_workqueue(accsns_data_wq);
        destroy_workqueue(accsns_data_wq);
        accsns_data_wq = NULL;
    }

    if(presns_cal_wq != NULL){
        flush_workqueue(presns_cal_wq);
        destroy_workqueue(presns_cal_wq);
        presns_cal_wq = NULL;
    }

    if(presns_data_wq != NULL){
        flush_workqueue(presns_data_wq);
        destroy_workqueue(presns_data_wq);
        presns_data_wq = NULL;
    }

#if CONFIG_INPUT_ML610Q793_INTERFACE == 0
	i2c_del_driver(&accsns_driver);
#elif CONFIG_INPUT_ML610Q793_INTERFACE == 1
    spi_unregister_driver(&accsns_driver);
#endif

    client_accsns = NULL;
}
    
    
static void accsns_debug_level_chg(int32_t lv)
{
#ifdef CONFIG_ML610Q793_DEBUG
    dbg_level = lv;
    printk("accsns_debug_level_chg level:%x\n", lv);
#endif
}

module_init(accsns_init);
module_exit(accsns_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("AccSensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ML610Q793.c");


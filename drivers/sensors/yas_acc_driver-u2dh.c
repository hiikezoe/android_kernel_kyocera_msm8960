/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/*
 * Copyright (c) 2010 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include <linux/yas.h>

#define YAS_U2DH_RESOLUTION                                                 1000
#define YAS_U2DH_BOOT_TIME                                                   (5)

/* Axes data range  [um/s^2] */
#define YAS_U2DH_GRAVITY_EARTH                                           9806550
#define YAS_U2DH_ABSMIN_2G                         (-YAS_U2DH_GRAVITY_EARTH * 2)
#define YAS_U2DH_ABSMAX_2G                          (YAS_U2DH_GRAVITY_EARTH * 2)

#define KC_U2DH_MIN_2G                                                   (-2048)
#define KC_U2DH_MAX_2G                                                    (2047)

/* Default parameters */
#define YAS_U2DH_DEFAULT_DELAY                                               100
#define YAS_U2DH_DEFAULT_POSITION                                              0

#define YAS_U2DH_MAX_DELAY                                                   200
#define YAS_U2DH_MIN_DELAY                                                    10

/* Registers */
#define YAS_U2DH_WHO_AM_I_REG                                               0x0f
#define YAS_U2DH_WHO_AM_I                                                   0x33

#define YAS_U2DH_CTRL_REG1                                                  0x20
#define YAS_U2DH_CTRL_REG2                                                  0x21
#define YAS_U2DH_CTRL_REG3                                                  0x22
#define YAS_U2DH_CTRL_REG4                                                  0x23
#define YAS_U2DH_CTRL_REG5                                                  0x24
#define YAS_U2DH_CTRL_REG6                                                  0x25

#define YAS_U2DH_X_ENABLE                                                   0x01
#define YAS_U2DH_Y_ENABLE                                                   0x02
#define YAS_U2DH_Z_ENABLE                                                   0x04
#define YAS_U2DH_XYZ_ENABLE                                                 0x07

#define YAS_U2DH_FS_2G                                                      0x00
#define YAS_U2DH_FS_4G                                                      0x10
#define YAS_U2DH_FS_8G                                                      0x20
#define YAS_U2DH_FS_16                                                      0x30

#define YAS_U2DH_HR_ENABLE                                                  0x08

#define YAS_U2DH_ODR_1HZ                                                    0x10
#define YAS_U2DH_ODR_10HZ                                                   0x20
#define YAS_U2DH_ODR_25HZ                                                   0x30
#define YAS_U2DH_ODR_50HZ                                                   0x40
#define YAS_U2DH_ODR_100HZ                                                  0x50
#define YAS_U2DH_ODR_200HZ                                                  0x60
#define YAS_U2DH_ODR_400HZ                                                  0x70

#define YAS_U2DH_ACC_REG                                                    0x28

#define KC_U2DH_REG5_BOOT                                              (0x01<<7)
#define KC_U2DH_REG4_BDU                                               (0x01<<7)

enum KC_U2DH_CAL_STATE
{
	KC_U2DH_CAL_STATE_ONE_DIRECTION = 0x00000000,
	KC_U2DH_CAL_STATE_FRONT_ONE_DIRECTION,
	KC_U2DH_CAL_STATE_BACK_ONE_DIRECTION,
	KC_U2DH_CAL_STATE_TWO_DIRECTION_1,
	KC_U2DH_CAL_STATE_TWO_DIRECTION_2,
	KC_U2DH_CAL_STATE_NONE = 0xffffffff
};

/* -------------------------------------------------------------------------- */
/*  Structure definition                                                      */
/* -------------------------------------------------------------------------- */
/* Output data rate */
struct yas_u2dh_odr {
	unsigned long delay;          /* min delay (msec) in the range of ODR */
	unsigned char odr;            /* bandwidth register value             */
	int32_t fs;
};

/* Axes data */
struct yas_u2dh_acceleration {
	int x;
	int y;
	int z;
	int x_raw;
	int y_raw;
	int z_raw;
};

struct kc_u2dh_cal_data {
	int32_t mode;
	int32_t state;
	int32_t wait;
	int32_t smp_n;
	int32_t ave_n;
	int32_t flt_en;
};

struct kc_u2dh_cal_buf {
	int32_t x[KC_ACC_CAL_FIT_CNT_MAX];
	int32_t y[KC_ACC_CAL_FIT_CNT_MAX];
	int32_t z[KC_ACC_CAL_FIT_CNT_MAX];
};

struct kc_u2dh_acceleration_cal_data {
	int32_t x_cal;
	int32_t y_cal;
	int32_t z_cal;
};

/* Driver private data */
struct yas_u2dh_data {
	int initialize;
	int i2c_open;
	int enable;
	int delay;
	int position;
	int threshold;
	int filter_enable;
	uint8_t odr;
	struct yas_vector offset;
	struct yas_u2dh_acceleration last;
	int32_t fs;
	struct kc_u2dh_cal_data cal;
	struct kc_u2dh_acceleration_cal_data acc_cal_data;
	struct kc_u2dh_cal_buf cal_buf;
	struct kc_u2dh_acceleration_cal_data add_cal;
	struct kc_u2dh_acceleration_cal_data first_cal;
	struct kc_u2dh_acceleration_cal_data second_cal;
	unsigned long cal_count;
	unsigned long cal_flt_count;
	int32_t offset_enable;
	uint32_t input_write;
};

/* -------------------------------------------------------------------------- */
/*  Data                                                                      */
/* -------------------------------------------------------------------------- */
/* Control block */
static struct yas_acc_driver   cb;
static struct yas_acc_driver  *pcb;
static struct yas_u2dh_data  acc_data;
static struct sensor_power_callback acc_power_cb;
atomic_t at_acc_device_reset = ATOMIC_INIT(0);

/* Output data rate */
static const struct yas_u2dh_odr yas_u2dh_odr_tbl[] = {
	{3,    YAS_U2DH_ODR_400HZ,    400},
	{5,    YAS_U2DH_ODR_200HZ,    200},
	{10,   YAS_U2DH_ODR_100HZ,    100},
	{20,   YAS_U2DH_ODR_50HZ,      50},
	{40,   YAS_U2DH_ODR_25HZ,      25},
	{100,  YAS_U2DH_ODR_10HZ,      10},
	{1000, YAS_U2DH_ODR_1HZ,        1},
};

/* Transformation matrix for chip mounting position 3 */
static const int yas_u2dh_position_map[][3][3] = {
	{ { 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} }, /* top/upper-left */
	{ {-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} }, /* top/upper-right */
	{ { 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} }, /* top/lower-right */
	{ { 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} }, /* top/lower-left */
	{ { 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} }, /* bottom/upper-left */
	{ { 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} }, /* bottom/upper-right */
	{ { 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} }, /* bottom/lower-right */
	{ {-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} }, /* bottom/lower-right */
};

/* -------------------------------------------------------------------------- */
/*  Prototype declaration                                                     */
/* -------------------------------------------------------------------------- */
static void kc_u2dh_reset_init_data(void);
static void yas_u2dh_init_data(void);
static int yas_u2dh_ischg_enable(int);
static int yas_u2dh_read_reg(unsigned char, unsigned char *, int);
static int yas_u2dh_write_reg(unsigned char, unsigned char *, int);
static unsigned char yas_u2dh_read_reg_byte(unsigned char);
static int yas_u2dh_write_reg_byte(unsigned char, unsigned char);
static int yas_u2dh_lock(void);
static int yas_u2dh_unlock(void);
static int yas_u2dh_i2c_open(void);
static int yas_u2dh_i2c_close(void);
static int yas_u2dh_msleep(int);

static int yas_u2dh_power_up(void);
static int yas_u2dh_power_down(void);
static int32_t kc_u2dh_reset_init(void);
static int32_t kc_u2dh_reset_term(void);
static int yas_u2dh_init(void);
static int yas_u2dh_term(void);
static int yas_u2dh_get_delay(void);
static int yas_u2dh_set_delay(int);
static int yas_u2dh_get_offset(struct yas_vector *);
static int yas_u2dh_set_offset(struct yas_vector *);
static int yas_u2dh_get_enable(void);
static int32_t kc_u2dh_set_reset(int32_t);
static int yas_u2dh_set_enable(int);
static int yas_u2dh_get_filter(struct yas_acc_filter *);
static int yas_u2dh_set_filter(struct yas_acc_filter *);
static int yas_u2dh_get_filter_enable(void);
static int yas_u2dh_set_filter_enable(int);
static int yas_u2dh_get_position(void);
static int yas_u2dh_set_position(int);
static int yas_u2dh_measure(int *, int *);
static void kc_u2dh_set_enable_cnt(int32_t enable);
static int32_t kc_u2dh_one_direction_calibration(struct yas_acc_data *data);
static int32_t kc_u2dh_front_one_direction_calibration(struct yas_acc_data *data);
static int32_t kc_u2dh_back_one_direction_calibration(struct yas_acc_data *data);
static int32_t kc_u2dh_two_direction_calibration_one(struct yas_acc_data *data);
static int32_t kc_u2dh_two_direction_calibration_two(struct yas_acc_data *data);
static int32_t kc_u2dh_calibration(struct yas_acc_data *data);
static int32_t kc_u2dh_get_fs(void);
static int32_t kc_u2dh_set_fs(int32_t);
static int32_t kc_u2dh_get_cal_mode(void);
static int32_t kc_u2dh_set_cal_mode(int32_t);
static int32_t kc_u2dh_get_cal_state(void);
static int32_t kc_u2dh_set_cal_state(int32_t);
static int32_t kc_u2dh_get_cal_wait(void);
static int32_t kc_u2dh_set_cal_wait(int32_t);
static int32_t kc_u2dh_get_cal_smp_n(void);
static int32_t kc_u2dh_set_cal_smp_n(int32_t);
static int32_t kc_u2dh_get_cal_ave_n(void);
static int32_t kc_u2dh_set_cal_ave_n(int32_t);
static int32_t kc_u2dh_get_cal_flt_en(void);
static int32_t kc_u2dh_set_cal_flt_en(int32_t);
static int32_t kc_u2dh_set_ofs_en(int32_t);
static int yas_init(void);
static int yas_term(void);

/* -------------------------------------------------------------------------- */
/*  Local functions                                                           */
/* -------------------------------------------------------------------------- */

static void kc_u2dh_power_on(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;

	YLOGI(("%s(): [IN]\n",__func__));
	kc_u2dh_reset_init();
		cbk->device_set_reset(1);
	YLOGI(("%s(): [OUT]\n",__func__));
}
static void kc_u2dh_power_off(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;

	YLOGI(("%s(): [IN]\n",__func__));
		cbk->device_set_reset(0);
	kc_u2dh_reset_term();
	YLOGI(("%s(): [OUT]\n",__func__));
}

static int32_t kc_u2dh_reset( void )
{
	int32_t reset = atomic_read( &at_acc_device_reset );
	if(reset == 1) {
		YLOGD(("%s(): reset start\n",__func__));
		atomic_set( &at_acc_device_reset, 2 );
		sensor_power_reset(SENSOR_INDEX_ACC);
		reset = atomic_read( &at_acc_device_reset );
		if (reset != 3)
			atomic_set( &at_acc_device_reset, 0 );
		YLOGD(("%s(): reset end reset=%d\n",__func__,reset));
	}
	return YAS_NO_ERROR;
}

static void kc_u2dh_set_vsensor(int32_t enable)
{
	if(enable) {
		sensor_power_on(SENSOR_INDEX_ACC);
	    sensor_power_reg_cbfunc(&acc_power_cb);
	} else {
	    sensor_power_unreg_cbfunc(&acc_power_cb);
		sensor_power_off(SENSOR_INDEX_ACC);
	}
}

static void kc_u2dh_reset_init_data(void)
{
	acc_data.initialize = 0;
	acc_data.last.x = 0;
	acc_data.last.y = 0;
	acc_data.last.z = 0;
	acc_data.last.x_raw = 0;
	acc_data.last.y_raw = 0;
	acc_data.last.z_raw = 0;
	acc_data.cal.mode = 0;
	acc_data.cal.state = KC_U2DH_CAL_STATE_NONE;
	acc_data.cal.wait = 0;
	acc_data.acc_cal_data.x_cal = 0;
	acc_data.acc_cal_data.y_cal = 0;
	acc_data.acc_cal_data.z_cal = 0;
	acc_data.add_cal.x_cal = 0;
	acc_data.add_cal.y_cal = 0;
	acc_data.add_cal.z_cal = 0;
	acc_data.first_cal.x_cal = 0;
	acc_data.first_cal.y_cal = 0;
	acc_data.first_cal.z_cal = 0;
	acc_data.second_cal.x_cal = 0;
	acc_data.second_cal.y_cal = 0;
	acc_data.second_cal.z_cal = 0;
	acc_data.cal_count = 0;
	acc_data.cal_flt_count = 0;
	
}

static void yas_u2dh_init_data(void)
{
	acc_data.initialize = 0;
	acc_data.enable = 0;
	acc_data.delay = YAS_U2DH_DEFAULT_DELAY;
	acc_data.offset.v[0] = 0;
	acc_data.offset.v[1] = 0;
	acc_data.offset.v[2] = 0;
	acc_data.position = YAS_U2DH_DEFAULT_POSITION;
	acc_data.threshold = YAS_ACC_DEFAULT_FILTER_THRESH;
	acc_data.filter_enable = 0;
	acc_data.odr = 0;
	acc_data.last.x = 0;
	acc_data.last.y = 0;
	acc_data.last.z = 0;
	acc_data.last.x_raw = 0;
	acc_data.last.y_raw = 0;
	acc_data.last.z_raw = 0;
	acc_data.fs = 50;
	acc_data.cal.mode = 0;
	acc_data.cal.state = KC_U2DH_CAL_STATE_NONE;
	acc_data.cal.wait = 0;
	acc_data.cal.smp_n = 100;
	acc_data.cal.ave_n = 1;
	acc_data.cal.flt_en = 0;
	acc_data.acc_cal_data.x_cal = 0;
	acc_data.acc_cal_data.y_cal = 0;
	acc_data.acc_cal_data.z_cal = 0;
	acc_data.add_cal.x_cal = 0;
	acc_data.add_cal.y_cal = 0;
	acc_data.add_cal.z_cal = 0;
	acc_data.first_cal.x_cal = 0;
	acc_data.first_cal.y_cal = 0;
	acc_data.first_cal.z_cal = 0;
	acc_data.second_cal.x_cal = 0;
	acc_data.second_cal.y_cal = 0;
	acc_data.second_cal.z_cal = 0;
	acc_data.cal_count = 0;
	acc_data.cal_flt_count = 0;
	
	acc_data.offset_enable = 0;
	acc_data.input_write = 0;
}

static int yas_u2dh_ischg_enable(int enable)
{
	if(enable) {
		if (acc_data.enable > 0)
			return 0;
	} else
		if (acc_data.enable != 1)
			return 0;

	return 1;
}

/* register access functions */
static int yas_u2dh_read_reg(unsigned char adr, unsigned char *buf, int len)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err = YAS_ERROR_DEVICE_COMMUNICATION;
	int32_t err_cnt = 0;
	int32_t reset = 0;

	if (acc_data.i2c_open) {
		do {
			err = cbk->device_read(adr, buf, len);
			if(err != 0 && 
				++err_cnt >= KC_SENSOR_DEVICE_ERR_MAX){
				reset = atomic_read( &at_acc_device_reset );
				YLOGE(("%s(): [ERR device_read] reset=%d\n",__func__,reset));
				if(reset == 0)
					atomic_set( &at_acc_device_reset, 1 );
				if(reset == 2)
					atomic_set( &at_acc_device_reset, 3 );
			}
		} while(err != 0 && 
			err_cnt < KC_SENSOR_DEVICE_ERR_MAX);
	}

	return err;
}

static int yas_u2dh_write_reg(unsigned char adr, unsigned char *buf, int len)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err = YAS_ERROR_DEVICE_COMMUNICATION;
	int32_t err_cnt = 0;
	int32_t reset = 0;

	if (acc_data.i2c_open) {
		do {
			err = cbk->device_write(adr, buf, len);
			if(err != 0 && 
				++err_cnt >= KC_SENSOR_DEVICE_ERR_MAX){
				reset = atomic_read( &at_acc_device_reset );
				YLOGE(("%s(): [ERR device_read] reset=%d\n",__func__,reset));
				if(reset == 0)
					atomic_set( &at_acc_device_reset, 1 );
				if(reset == 2)
					atomic_set( &at_acc_device_reset, 3 );
			}
		} while(err != 0 && 
			err_cnt < KC_SENSOR_DEVICE_ERR_MAX);
	}

	return err;
}

static unsigned char yas_u2dh_read_reg_byte(unsigned char adr)
{
	unsigned char buf;
	int err;

	err = yas_u2dh_read_reg(adr, &buf, 1);
	if (err == 0)
		return buf;

	return 0;
}

static int yas_u2dh_write_reg_byte(unsigned char adr, unsigned char val)
{
	return yas_u2dh_write_reg(adr, &val, 1);
}

#define yas_u2dh_read_bits(r) \
	((yas_u2dh_read_reg_byte(r##_REG) & r##_MASK) >> r##_SHIFT)

#define yas_u2dh_update_bits(r, v) \
	yas_u2dh_write_reg_byte(r##_REG, \
				((yas_u2dh_read_reg_byte(r##_REG) & \
				  ~r##_MASK) | ((v) << r##_SHIFT)))

static int yas_u2dh_lock(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	if (cbk->lock != NULL && cbk->unlock != NULL)
		err = cbk->lock();
	else
		err = YAS_NO_ERROR;

	return err;
}

static int yas_u2dh_unlock(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	if (cbk->lock != NULL && cbk->unlock != NULL)
		err = cbk->unlock();
	else
		err = YAS_NO_ERROR;

	kc_u2dh_reset();

	return err;
}

static int yas_u2dh_i2c_open(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	YLOGI(("%s(): start\n",__func__));
	if (acc_data.i2c_open == 0) {
		kc_u2dh_set_vsensor(1);
		err = cbk->device_open();
		if (err != YAS_NO_ERROR)
		{
			YLOGE(("%s(): err device_open\n",__func__));
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
		acc_data.i2c_open = 1;
	}

	YLOGI(("%s(): end\n",__func__));
	return YAS_NO_ERROR;
}

static int yas_u2dh_i2c_close(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	if (acc_data.i2c_open != 0) {
		err = cbk->device_close();
		if (err != YAS_NO_ERROR)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		acc_data.i2c_open = 0;
		kc_u2dh_set_vsensor(0);
	}
	return YAS_NO_ERROR;
}

static int yas_u2dh_msleep(int msec)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;

	if (msec <= 0)
		return YAS_ERROR_ARG;

	cbk->msleep(msec);

	return YAS_NO_ERROR;
}

static int yas_u2dh_power_up(void)
{
	yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG1, acc_data.odr |
				YAS_U2DH_XYZ_ENABLE);

	return YAS_NO_ERROR;
}

static int yas_u2dh_power_down(void)
{
	yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG1, YAS_U2DH_XYZ_ENABLE);

	return YAS_NO_ERROR;
}

static int32_t kc_u2dh_reset_init(void)
{
	YLOGI(("%s(): start\n",__func__));
	/* Check intialize */
	if (acc_data.initialize == 1)
	{
		YLOGE(("%s(): err acc_data.initialize\n",__func__));
		return YAS_ERROR_NOT_INITIALIZED;
	}

	/* Wait device boot time */
	yas_u2dh_msleep(YAS_U2DH_BOOT_TIME);

	/* Init data */
	kc_u2dh_reset_init_data();

	yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG5, KC_U2DH_REG5_BOOT);
	msleep(10);
	yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG1, YAS_U2DH_XYZ_ENABLE);
	yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG4, 
				(YAS_U2DH_HR_ENABLE | KC_U2DH_REG4_BDU));

	acc_data.initialize = 1;

	YLOGI(("%s(): end\n",__func__));
	return YAS_NO_ERROR;
}

static int32_t kc_u2dh_reset_term(void)
{
	YLOGI(("%s(): [IN]\n",__func__));
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	acc_data.initialize = 0;

	YLOGI(("%s(): [OUT]\n",__func__));
	return YAS_NO_ERROR;
}

static int yas_u2dh_init(void)
{
	struct yas_acc_filter filter;
	unsigned char id;
	int err;

	YLOGI(("%s(): start\n",__func__));
	/* Check intialize */
	if (acc_data.initialize == 1)
	{
		YLOGE(("%s(): err acc_data.initialize\n",__func__));
		return YAS_ERROR_NOT_INITIALIZED;
	}

	/* Wait device boot time */
	yas_u2dh_msleep(YAS_U2DH_BOOT_TIME);

	/* Init data */
	yas_u2dh_init_data();
	acc_power_cb.power_on = kc_u2dh_power_on;
	acc_power_cb.power_off = kc_u2dh_power_off;

	/* Open i2c */
	err = yas_u2dh_i2c_open();
	if (err != YAS_NO_ERROR)
	{
		YLOGE(("%s(): err yas_u2dh_i2c_open\n",__func__));
		return err;
	}

	/* Check id */
	id = yas_u2dh_read_reg_byte(YAS_U2DH_WHO_AM_I_REG);
	if (id != YAS_U2DH_WHO_AM_I) {
		yas_u2dh_i2c_close();
		YLOGE(("%s(): err yas_u2dh_read_reg_byte\n",__func__));
		return YAS_ERROR_CHIP_ID;
	}

	yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG5, KC_U2DH_REG5_BOOT);
	msleep(10);
	yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG1, YAS_U2DH_XYZ_ENABLE);
	yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG4, 
				(YAS_U2DH_HR_ENABLE | KC_U2DH_REG4_BDU));

	acc_data.initialize = 1;

	yas_u2dh_set_delay(YAS_U2DH_DEFAULT_DELAY);
	yas_u2dh_set_position(YAS_U2DH_DEFAULT_POSITION);
	filter.threshold = YAS_ACC_DEFAULT_FILTER_THRESH;
	yas_u2dh_set_filter(&filter);

	yas_u2dh_i2c_close();
	YLOGI(("%s(): end\n",__func__));
	return YAS_NO_ERROR;
}

static int yas_u2dh_term(void)
{
	YLOGI(("%s(): [IN]\n",__func__));
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_set_enable(0);

	/* Close I2C */
	yas_u2dh_i2c_close();

	acc_data.initialize = 0;

	YLOGI(("%s(): [OUT]\n",__func__));
	return YAS_NO_ERROR;
}

static int yas_u2dh_get_delay(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return acc_data.delay;
}

static int yas_u2dh_set_delay(int delay)
{
	int i;
	int32_t err = YAS_NO_ERROR;

	YLOGI(("%s(): [IN]\n",__func__));
	YLOGD(("%s(): delay = %d\n",__func__,delay));
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Determine optimum odr */
	for (i = 1; i < (int)(sizeof(yas_u2dh_odr_tbl) /
			      sizeof(struct yas_u2dh_odr)) &&
		     delay >= (int)yas_u2dh_odr_tbl[i].delay; i++)
		;

	acc_data.odr = yas_u2dh_odr_tbl[i-1].odr;
	acc_data.delay = delay;
	acc_data.fs = yas_u2dh_odr_tbl[i-1].fs;

	if (yas_u2dh_get_enable())
		yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG1, acc_data.odr|
					YAS_U2DH_XYZ_ENABLE);
	else {
		err = yas_u2dh_i2c_open();
		if (err != YAS_NO_ERROR)
				return err;
		yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG1,
					YAS_U2DH_XYZ_ENABLE);
		yas_u2dh_i2c_close();
	}

	YLOGI(("%s(): [OUT]\n",__func__));
	return YAS_NO_ERROR;
}

static int yas_u2dh_get_offset(struct yas_vector *offset)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	*offset = acc_data.offset;

	return YAS_NO_ERROR;
}

static int yas_u2dh_set_offset(struct yas_vector *offset)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;

	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	acc_data.offset = *offset;
	if(!acc_data.input_write)
		cbk->device_input_offset(&acc_data.offset);
	YLOGI(("%s(): [OUT]\n",__func__));

	return YAS_NO_ERROR;
}

static int yas_u2dh_get_enable(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return acc_data.enable;
}

static int32_t kc_u2dh_set_reset(int32_t reset)
{
	YLOGI(("%s(): [IN] reset=%d\n",__func__,reset));

	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	if (reset) {
		yas_u2dh_power_up();
	} else {
		yas_u2dh_power_down();
	}

	YLOGI(("%s(): [OUT]\n",__func__));
	return YAS_NO_ERROR;
}

static int yas_u2dh_set_enable(int enable)
{
	int err;

	YLOGI(("%s(): [IN]\n",__func__));
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGD(("%s(): [S] acc_data.enable=%d enable=%d\n",__func__,acc_data.enable,enable));
	if (yas_u2dh_ischg_enable(enable)) {
		if (enable) {
			/* Open i2c */
			err = yas_u2dh_i2c_open();
			if (err != YAS_NO_ERROR)
				return err;

			yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG5, KC_U2DH_REG5_BOOT);
			msleep(10);
			yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG1, YAS_U2DH_XYZ_ENABLE);
			yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG4, 
						(YAS_U2DH_HR_ENABLE | KC_U2DH_REG4_BDU));

			yas_u2dh_power_up();
			acc_data.enable = 1;
			YLOGD(("%s(): DISABLE -> ENABLE acc_data.enable=%d\n",__func__,acc_data.enable));
#if DEBUG
			{
				int32_t reg;
				uint8_t i;
				for (i=0x1f;i<0x3f;i++) {
					reg = yas_u2dh_read_reg_byte(i);
					YLOGD(("reg[0x%02x]=0x%02x\n",i,reg));
				}
			}
#endif
		} else {
			yas_u2dh_power_down();
			acc_data.enable = 0;
			YLOGD(("%s(): ENABLE -> DISABLE acc_data.enable=%d\n",__func__,acc_data.enable));
			err = yas_u2dh_i2c_close();
			if (err != YAS_NO_ERROR)
				return err;
		}
	} else {
		if(enable)
			acc_data.enable++;
		else
			if(--acc_data.enable < 0)
				acc_data.enable = 0;

	}
	YLOGD(("%s(): [E] acc_data.enable=%d enable=%d\n",__func__,acc_data.enable,enable));
	YLOGI(("%s(): [OUT]\n",__func__));
	return YAS_NO_ERROR;
}

static int yas_u2dh_get_filter(struct yas_acc_filter *filter)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	filter->threshold = acc_data.threshold;

	return YAS_NO_ERROR;
}

static int yas_u2dh_set_filter(struct yas_acc_filter *filter)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	acc_data.threshold = filter->threshold;

	YLOGD(("%s(): threshold = %d\n",__func__,filter->threshold));
	YLOGI(("%s(): [OUT]\n",__func__));
	return YAS_NO_ERROR;
}

static int yas_u2dh_get_filter_enable(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	return acc_data.filter_enable;
}

static int yas_u2dh_set_filter_enable(int enable)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	acc_data.filter_enable = enable;
	YLOGD(("%s(): filter_enable = %d\n",__func__,enable));
	YLOGI(("%s(): [OUT]\n",__func__));

	return YAS_NO_ERROR;
}

static int yas_u2dh_get_position(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return acc_data.position;
}

static int yas_u2dh_set_position(int position)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	acc_data.position = position;
	YLOGD(("%s(): acc_data.position = %d\n",__func__,position));
	YLOGI(("%s(): [OUT]\n",__func__));

	return YAS_NO_ERROR;
}

static int yas_u2dh_data_filter(int data[], int raw[],
				struct yas_u2dh_acceleration *accel)
{
	int filter_enable = acc_data.filter_enable;
	int threshold = acc_data.threshold;

	if (filter_enable) {
		if ((ABS(acc_data.last.x - data[0]) > threshold) ||
		    (ABS(acc_data.last.y - data[1]) > threshold) ||
		    (ABS(acc_data.last.z - data[2]) > threshold)) {
			accel->x = data[0];
			accel->y = data[1];
			accel->z = data[2];
			accel->x_raw = raw[0];
			accel->y_raw = raw[1];
			accel->z_raw = raw[2];
		} else {
			*accel = acc_data.last;
		}
	} else {
		accel->x = data[0];
		accel->y = data[1];
		accel->z = data[2];
		accel->x_raw = raw[0];
		accel->y_raw = raw[1];
		accel->z_raw = raw[2];
	}

	return YAS_NO_ERROR;
}

static int yas_u2dh_measure(int *out_data, int *out_raw)
{
	struct yas_u2dh_acceleration accel;
	unsigned char buf[6];
	int32_t raw[3], data[3], offset[3] = {0};
	int pos = acc_data.position;
	int i, j;

	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Check device */
	if (acc_data.i2c_open == 0) {
		out_data[0] = acc_data.last.x;
		out_data[1] = acc_data.last.y;
		out_data[2] = acc_data.last.z;
		out_raw[0] = acc_data.last.x_raw;
		out_raw[1] = acc_data.last.y_raw;
		out_raw[2] = acc_data.last.z_raw;
		return YAS_NO_ERROR;
	}

	/* Read acceleration data */
	if (yas_u2dh_read_reg(YAS_U2DH_ACC_REG | 0x80, buf, 6) != 0)
		for (i = 0; i < 3; i++)
			raw[i] = 0;
	else
		for (i = 0; i < 3; i++)
			raw[i] = (int16_t)((buf[i*2+1] << 8) | buf[i*2]) >> 4;

	/* for X, Y, Z axis */
	for (i = 0; i < 3; i++) {
		/* coordinate transformation */
		data[i] = 0;
		for (j = 0; j < 3; j++)
			data[i] += raw[j] * yas_u2dh_position_map[pos][i][j];
		/* normalization */
		data[i] *= (YAS_U2DH_GRAVITY_EARTH / YAS_U2DH_RESOLUTION);
	}

	yas_u2dh_data_filter(data, raw, &accel);

	if(acc_data.offset_enable) {
		offset[0] = acc_data.offset.v[0] * (YAS_U2DH_GRAVITY_EARTH / YAS_U2DH_RESOLUTION);
		offset[1] = acc_data.offset.v[1] * (YAS_U2DH_GRAVITY_EARTH / YAS_U2DH_RESOLUTION);
		offset[2] = acc_data.offset.v[2] * (YAS_U2DH_GRAVITY_EARTH / YAS_U2DH_RESOLUTION);
	}
	
	out_data[0] = accel.x - offset[0];
	out_data[1] = accel.y - offset[1];
	out_data[2] = accel.z - offset[2];
	out_raw[0] = accel.x_raw;
	out_raw[1] = accel.y_raw;
	out_raw[2] = accel.z_raw;
	acc_data.last = accel;

	return YAS_NO_ERROR;
}

static void kc_u2dh_set_enable_cnt(int32_t enable)
{
	acc_data.enable = enable;
}
static int32_t kc_u2dh_calibration_data_keep(struct yas_acc_data *data)
{
	int32_t ret = YAS_NO_ERROR;
	int32_t i = 0;
	struct kc_u2dh_acceleration_cal_data flt_data;
	YLOGI(("%s(): start\n",__func__));
	
	if (acc_data.cal.smp_n <= 0)
		return YAS_ERROR_ERROR;
	
	if(acc_data.cal.flt_en) {
		if (acc_data.cal.ave_n <= 0)
			return YAS_ERROR_ERROR;

		acc_data.cal_buf.x[acc_data.cal_flt_count%KC_ACC_CAL_FIT_CNT_MAX] = data->raw.v[0];
		acc_data.cal_buf.y[acc_data.cal_flt_count%KC_ACC_CAL_FIT_CNT_MAX] = data->raw.v[1];
		acc_data.cal_buf.z[acc_data.cal_flt_count%KC_ACC_CAL_FIT_CNT_MAX] = data->raw.v[2];
		acc_data.cal_flt_count++;
		YLOGD(("%s(): cal_flt_count = %ld\n",__func__,acc_data.cal_flt_count));
		if(acc_data.cal.ave_n <= acc_data.cal_flt_count) {
			memset(&flt_data,0,sizeof(struct kc_u2dh_acceleration_cal_data));
			for(i=0;i<acc_data.cal.ave_n;i++) {
				flt_data.x_cal += acc_data.cal_buf.x[(acc_data.cal_flt_count-i)%KC_ACC_CAL_FIT_CNT_MAX];
				flt_data.y_cal += acc_data.cal_buf.y[(acc_data.cal_flt_count-i)%KC_ACC_CAL_FIT_CNT_MAX];
				flt_data.z_cal += acc_data.cal_buf.z[(acc_data.cal_flt_count-i)%KC_ACC_CAL_FIT_CNT_MAX];
			}
			flt_data.x_cal /= acc_data.cal.ave_n;
			flt_data.y_cal /= acc_data.cal.ave_n;
			flt_data.z_cal /= acc_data.cal.ave_n;
			acc_data.add_cal.x_cal += flt_data.x_cal;
			acc_data.add_cal.y_cal += flt_data.y_cal;
			acc_data.add_cal.z_cal += flt_data.z_cal;
			acc_data.cal_count++;
			YLOGD(("flt_data %d %d %d \n",flt_data.x_cal,flt_data.y_cal,flt_data.z_cal));
			YLOGD(("add_cal %d %d %d \n",acc_data.add_cal.x_cal,acc_data.add_cal.y_cal,acc_data.add_cal.z_cal));
			YLOGD(("%s(): cal_count = %ld\n",__func__,acc_data.cal_count));
		}
	} else {
		acc_data.add_cal.x_cal += data->raw.v[0];
		acc_data.add_cal.y_cal += data->raw.v[1];
		acc_data.add_cal.z_cal += data->raw.v[2];
		acc_data.cal_count++;
		YLOGD(("%s(): cal_count = %ld\n",__func__,acc_data.cal_count));
	}

	ret = (acc_data.cal.smp_n > acc_data.cal_count) ? YAS_NO_ERROR : 1;

	YLOGI(("%s(): end ret = %d\n",__func__,ret));
	return ret;
}

static int32_t kc_u2dh_one_direction_calibration(struct yas_acc_data *data)
{
	int32_t ret = YAS_ERROR_ERROR;
	YLOGI(("%s(): start\n",__func__));

	ret = kc_u2dh_calibration_data_keep(data);
	if (ret > YAS_NO_ERROR) {
		acc_data.acc_cal_data.x_cal = acc_data.add_cal.x_cal/acc_data.cal.smp_n;
		acc_data.acc_cal_data.y_cal = acc_data.add_cal.y_cal/acc_data.cal.smp_n;
		acc_data.acc_cal_data.z_cal = 0;
		yas_u2dh_set_offset((struct yas_vector*)&acc_data.acc_cal_data);
		kc_u2dh_set_cal_state(KC_U2DH_CAL_STATE_NONE);
		kc_u2dh_set_cal_mode(0);
		kc_u2dh_set_cal_wait(1);
		ret = YAS_NO_ERROR;
	} else if (ret == YAS_NO_ERROR) {
		kc_u2dh_set_cal_wait(0);
		ret = YAS_NO_ERROR;
	} else {
		YLOGE(("%s(): err kc_u2dh_calibration_data_keep\n",__func__));
		kc_u2dh_set_cal_mode(0);
	}

	YLOGI(("%s(): end ret = %d\n",__func__,ret));
	return ret;
}

static int32_t kc_u2dh_front_one_direction_calibration(struct yas_acc_data *data)
{
	int32_t ret = YAS_ERROR_ERROR;
	YLOGI(("%s(): start\n",__func__));

	ret = kc_u2dh_calibration_data_keep(data);
	if (ret > YAS_NO_ERROR) {
		acc_data.acc_cal_data.x_cal = acc_data.add_cal.x_cal/acc_data.cal.smp_n;
		acc_data.acc_cal_data.y_cal = acc_data.add_cal.y_cal/acc_data.cal.smp_n;
		acc_data.acc_cal_data.z_cal = (acc_data.add_cal.z_cal/acc_data.cal.smp_n) - YAS_U2DH_RESOLUTION;
		yas_u2dh_set_offset((struct yas_vector*)&acc_data.acc_cal_data);
		kc_u2dh_set_cal_mode(0);
		kc_u2dh_set_cal_wait(1);
		ret = YAS_NO_ERROR;
	} else if (ret == YAS_NO_ERROR) {
		kc_u2dh_set_cal_wait(0);
		ret = YAS_NO_ERROR;
	} else {
		YLOGE(("%s(): err kc_u2dh_calibration_data_keep\n",__func__));
		kc_u2dh_set_cal_mode(0);
	}

	YLOGI(("%s(): end ret = %d\n",__func__,ret));
	return ret;
}

static int32_t kc_u2dh_back_one_direction_calibration(struct yas_acc_data *data)
{
	int32_t ret = YAS_ERROR_ERROR;
	YLOGI(("%s(): start\n",__func__));

	ret = kc_u2dh_calibration_data_keep(data);
	if (ret > YAS_NO_ERROR) {
		acc_data.acc_cal_data.x_cal = acc_data.add_cal.x_cal/acc_data.cal.smp_n;
		acc_data.acc_cal_data.y_cal = acc_data.add_cal.y_cal/acc_data.cal.smp_n;
		acc_data.acc_cal_data.z_cal = (acc_data.add_cal.z_cal/acc_data.cal.smp_n) + YAS_U2DH_RESOLUTION;
		yas_u2dh_set_offset((struct yas_vector*)&acc_data.acc_cal_data);
		kc_u2dh_set_cal_mode(0);
		kc_u2dh_set_cal_wait(1);
		ret = YAS_NO_ERROR;
	} else if (ret == YAS_NO_ERROR) {
		kc_u2dh_set_cal_wait(0);
		ret = YAS_NO_ERROR;
	} else {
		YLOGE(("%s(): err kc_u2dh_calibration_data_keep\n",__func__));
		kc_u2dh_set_cal_mode(0);
	}

	YLOGI(("%s(): end ret = %d\n",__func__,ret));
	return ret;
}

static int32_t kc_u2dh_two_direction_calibration_one(struct yas_acc_data *data)
{
	int32_t ret = YAS_ERROR_ERROR;
	YLOGI(("%s(): start\n",__func__));

	ret = kc_u2dh_calibration_data_keep(data);
	if (ret > YAS_NO_ERROR) {
		acc_data.first_cal.x_cal = acc_data.add_cal.x_cal/acc_data.cal.smp_n;
		acc_data.first_cal.y_cal = acc_data.add_cal.y_cal/acc_data.cal.smp_n;
		acc_data.first_cal.z_cal = acc_data.add_cal.z_cal/acc_data.cal.smp_n;
		kc_u2dh_set_cal_state(KC_U2DH_CAL_STATE_NONE);
		kc_u2dh_set_cal_wait(1);
		ret = YAS_NO_ERROR;
	} else if (ret == YAS_NO_ERROR) {
		kc_u2dh_set_cal_wait(0);
		ret = YAS_NO_ERROR;
	} else {
		YLOGE(("%s(): err kc_u2dh_calibration_data_keep\n",__func__));
		kc_u2dh_set_cal_mode(0);
	}

	YLOGI(("%s(): end ret = %d\n",__func__,ret));
	return ret;
}

static int32_t kc_u2dh_two_direction_calibration_two(struct yas_acc_data *data)
{
	int32_t ret = YAS_ERROR_ERROR;
	YLOGI(("%s(): start\n",__func__));

	ret = kc_u2dh_calibration_data_keep(data);
	if (ret > YAS_NO_ERROR) {
		acc_data.second_cal.x_cal = acc_data.add_cal.x_cal/acc_data.cal.smp_n;
		acc_data.second_cal.y_cal = acc_data.add_cal.y_cal/acc_data.cal.smp_n;
		acc_data.second_cal.z_cal = acc_data.add_cal.z_cal/acc_data.cal.smp_n;
		acc_data.acc_cal_data.x_cal = (acc_data.first_cal.x_cal + acc_data.second_cal.x_cal)/2;
		acc_data.acc_cal_data.y_cal = (acc_data.first_cal.y_cal + acc_data.second_cal.y_cal)/2;
		acc_data.acc_cal_data.z_cal = (acc_data.first_cal.z_cal + acc_data.second_cal.z_cal)/2;
		yas_u2dh_set_offset((struct yas_vector*)&acc_data.acc_cal_data);
		kc_u2dh_set_cal_mode(0);
		kc_u2dh_set_cal_wait(1);
		ret = YAS_NO_ERROR;
	} else if (ret == YAS_NO_ERROR) {
		kc_u2dh_set_cal_wait(0);
		ret = YAS_NO_ERROR;
	} else {
		YLOGE(("%s(): err kc_u2dh_calibration_data_keep\n",__func__));
		kc_u2dh_set_cal_mode(0);
	}

	YLOGI(("%s(): end ret = %d\n",__func__,ret));
	return ret;
}

static int32_t kc_u2dh_calibration(struct yas_acc_data *data)
{
	int32_t ret = YAS_NO_ERROR;
	YLOGI(("%s(): start\n",__func__));
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;
	YLOGD(("%s(): state = %d\n",__func__,acc_data.cal.state));

	switch (acc_data.cal.state)
	{
	case KC_U2DH_CAL_STATE_ONE_DIRECTION:
		ret = kc_u2dh_one_direction_calibration(data);
		break;
	case KC_U2DH_CAL_STATE_FRONT_ONE_DIRECTION:
		ret = kc_u2dh_front_one_direction_calibration(data);
		break;
	case KC_U2DH_CAL_STATE_BACK_ONE_DIRECTION:
		ret = kc_u2dh_back_one_direction_calibration(data);
		break;
	case KC_U2DH_CAL_STATE_TWO_DIRECTION_1:
		ret = kc_u2dh_two_direction_calibration_one(data);
		break;
	case KC_U2DH_CAL_STATE_TWO_DIRECTION_2:
		ret = kc_u2dh_two_direction_calibration_two(data);
		break;
	case KC_U2DH_CAL_STATE_NONE:
		break;
	default :
		YLOGE(("%s(): err acc_data.cal.state=%x\n",__func__,acc_data.cal.state));
		ret = YAS_ERROR_ERROR;
		break;
	}
	
	YLOGI(("%s(): end\n",__func__));
	return ret;
}

static int32_t kc_u2dh_get_fs(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;
	YLOGD(("%s(): fs = %d\n",__func__,acc_data.fs));

	return acc_data.fs;
}

static int32_t kc_u2dh_set_fs(int32_t fs)
{
	int32_t i;
	int32_t array_size;
	int32_t err = YAS_NO_ERROR;

	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Determine optimum odr */
	array_size = (int32_t)(sizeof(yas_u2dh_odr_tbl) / sizeof(struct yas_u2dh_odr));
	for (i = 1; i < array_size; i++)
	{
		if(fs > yas_u2dh_odr_tbl[i].fs)
			break;
	}

	acc_data.odr = yas_u2dh_odr_tbl[i-1].odr;
	acc_data.delay = yas_u2dh_odr_tbl[i-1].delay;
	acc_data.fs = yas_u2dh_odr_tbl[i-1].fs;

	if (yas_u2dh_get_enable())
		yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG1, acc_data.odr|
					  YAS_U2DH_XYZ_ENABLE);
	else {
		err = yas_u2dh_i2c_open();
		if (err != YAS_NO_ERROR)
				return err;
		yas_u2dh_write_reg_byte(YAS_U2DH_CTRL_REG1,
					  YAS_U2DH_XYZ_ENABLE);
		yas_u2dh_i2c_close();
	}

	YLOGD(("%s(): odr=%d delay=%d fs=%d\n",__func__,acc_data.odr,acc_data.delay,acc_data.fs));
	return YAS_NO_ERROR;
}

static int32_t kc_u2dh_get_cal_mode(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	if (acc_data.cal.mode != 0)
		YLOGD(("%s(): mode = %d \n",__func__,acc_data.cal.mode));
	return acc_data.cal.mode;
}

static int32_t kc_u2dh_set_cal_mode(int32_t mode)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	kc_u2dh_set_cal_state(KC_U2DH_CAL_STATE_NONE);

	memset(&acc_data.first_cal, 0, sizeof(struct kc_u2dh_acceleration_cal_data));
	memset(&acc_data.second_cal, 0, sizeof(struct kc_u2dh_acceleration_cal_data));
	acc_data.cal.mode = mode;

	YLOGD(("%s(): mode = %d \n",__func__,mode));
	return YAS_NO_ERROR;
}

static int32_t kc_u2dh_get_cal_state(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGD(("%s(): state = %d \n",__func__,acc_data.cal.state));
	return acc_data.cal.state;
}

static int32_t kc_u2dh_set_cal_state(int32_t state)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	acc_data.cal_count = 0;
	acc_data.cal_flt_count = 0;
	memset(&acc_data.add_cal, 0, sizeof(struct kc_u2dh_acceleration_cal_data));
	memset(&acc_data.cal_buf, 0, sizeof(struct kc_u2dh_cal_buf));
	acc_data.cal.state = state;

	kc_u2dh_set_cal_wait(0);

	YLOGD(("%s(): state = %d \n",__func__,state));
	return YAS_NO_ERROR;
}

static int32_t kc_u2dh_get_cal_wait(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGD(("%s(): wait = %d \n",__func__,acc_data.cal.wait));
	return acc_data.cal.wait;
}

static int32_t kc_u2dh_set_cal_wait(int32_t wait)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	if (acc_data.cal.wait != wait)
	{
		acc_data.cal.wait = wait;
		YLOGD(("%s(): wait = %d \n",__func__,acc_data.cal.wait));
	}

	return YAS_NO_ERROR;
}

static int32_t kc_u2dh_get_cal_smp_n(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;
	YLOGD(("%s(): smp_n = %d \n",__func__,acc_data.cal.smp_n));

	return acc_data.cal.smp_n;
}

static int32_t kc_u2dh_set_cal_smp_n(int32_t smp_n)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	acc_data.cal.smp_n = smp_n;
	YLOGD(("%s(): smp_n = %d \n",__func__,smp_n));

	return YAS_NO_ERROR;
}

static int32_t kc_u2dh_get_cal_ave_n(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGD(("%s(): ave_n = %d \n",__func__,acc_data.cal.ave_n));
	return acc_data.cal.ave_n;
}

static int32_t kc_u2dh_set_cal_ave_n(int32_t ave_n)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	acc_data.cal.ave_n = ave_n;

	YLOGD(("%s(): ave_n = %d \n",__func__,ave_n));
	return YAS_NO_ERROR;
}

static int32_t kc_u2dh_get_cal_flt_en(void)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGD(("%s(): flt_en = %d \n",__func__,acc_data.cal.flt_en));
	return acc_data.cal.flt_en;
}

static int32_t kc_u2dh_set_cal_flt_en(int32_t flt_en)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	acc_data.cal.flt_en = flt_en;
	YLOGD(("%s(): flt_en = %d \n",__func__,flt_en));

	return YAS_NO_ERROR;
}

static int32_t kc_u2dh_set_ofs_en(int32_t ofs_en)
{
	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	acc_data.offset_enable = ofs_en;
	YLOGD(("%s(): ofs_en = %d \n",__func__,ofs_en));

	return YAS_NO_ERROR;
}

/* -------------------------------------------------------------------------- */
static int yas_init(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = yas_u2dh_init();
	yas_u2dh_unlock();

	return err;
}

static int yas_term(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = yas_u2dh_term();
	yas_u2dh_unlock();

	return err;
}

static int yas_get_delay(void)
{
	int ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	ret = yas_u2dh_get_delay();
	yas_u2dh_unlock();

	return ret;
}

static int yas_set_delay(int delay)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (delay < 0 || delay > YAS_U2DH_MAX_DELAY)
		return YAS_ERROR_ARG;
	else if (delay < YAS_U2DH_MIN_DELAY)
		delay = YAS_U2DH_MIN_DELAY;

	yas_u2dh_lock();
	err = yas_u2dh_set_delay(delay);
	yas_u2dh_unlock();

	return err;
}

static int yas_get_offset(struct yas_vector *offset)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (offset == NULL)
		return YAS_ERROR_ARG;

	yas_u2dh_lock();
	err = yas_u2dh_get_offset(offset);
	yas_u2dh_unlock();

	return err;
}

static int yas_set_offset(struct yas_vector *offset)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (offset == NULL ||
	    offset->v[0] < KC_U2DH_MIN_2G ||
	    KC_U2DH_MAX_2G < offset->v[0] ||
	    offset->v[1] < KC_U2DH_MIN_2G ||
	    KC_U2DH_MAX_2G < offset->v[1] ||
	    offset->v[2] < KC_U2DH_MIN_2G ||
	    KC_U2DH_MAX_2G < offset->v[2])
		return YAS_ERROR_ARG;

	yas_u2dh_lock();
	acc_data.input_write = 1;
	err = yas_u2dh_set_offset(offset);
	acc_data.input_write = 0;
	yas_u2dh_unlock();

	return err;
}

static int yas_get_enable(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = yas_u2dh_get_enable();
	yas_u2dh_unlock();

	return err;
}

static int32_t kc_set_reset(int32_t reset)
{
	int32_t err;

	YLOGI(("%s(): start reset=%d\n",__func__,reset));
	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (reset != 0)
		reset = 1;

	yas_u2dh_lock();
	err = kc_u2dh_set_reset(reset);
	yas_u2dh_unlock();

	YLOGI(("%s(): end err=%d\n",__func__,err));
	return err;
}

static int yas_set_enable(int enable)
{
	int err;

	YLOGI(("%s(): start enable=%d\n",__func__,enable));
	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (enable != 0)
		enable = 1;

	yas_u2dh_lock();
	err = yas_u2dh_set_enable(enable);
	yas_u2dh_unlock();

	YLOGI(("%s(): end err=%d\n",__func__,err));
	return err;
}

static int yas_get_filter(struct yas_acc_filter *filter)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (filter == NULL)
		return YAS_ERROR_ARG;

	yas_u2dh_lock();
	err = yas_u2dh_get_filter(filter);
	yas_u2dh_unlock();

	return err;
}

static int yas_set_filter(struct yas_acc_filter *filter)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (filter == NULL || filter->threshold < 0 ||
	    filter->threshold > YAS_U2DH_ABSMAX_2G)
		return YAS_ERROR_ARG;

	yas_u2dh_lock();
	err = yas_u2dh_set_filter(filter);
	yas_u2dh_unlock();

	return err;
}

static int yas_get_filter_enable(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = yas_u2dh_get_filter_enable();
	yas_u2dh_unlock();

	return err;
}

static int yas_set_filter_enable(int enable)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (enable != 0)
		enable = 1;

	yas_u2dh_lock();
	err = yas_u2dh_set_filter_enable(enable);
	yas_u2dh_unlock();

	return err;
}

static int yas_get_position(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = yas_u2dh_get_position();
	yas_u2dh_unlock();

	return err;
}

static int yas_set_position(int position)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (!((position >= 0) && (position <= 7)))
		return YAS_ERROR_ARG;

	yas_u2dh_lock();
	err = yas_u2dh_set_position(position);
	yas_u2dh_unlock();

	return err;
}

static int yas_measure(struct yas_acc_data *data)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (data == NULL)
		return YAS_ERROR_ARG;

	yas_u2dh_lock();
	err = yas_u2dh_measure(data->xyz.v, data->raw.v);
	yas_u2dh_unlock();

	return err;
}

static int32_t kc_set_enable_cnt(int32_t enable)
{
	int32_t ret = YAS_NO_ERROR;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	kc_u2dh_set_enable_cnt(enable);
	yas_u2dh_unlock();

	return ret;
}

static int32_t kc_calibration(struct yas_acc_data *data)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (data == NULL)
		return YAS_ERROR_ARG;

	yas_u2dh_lock();
	err = kc_u2dh_calibration(data);
	yas_u2dh_unlock();

	return err;
}

static int32_t yas_get_register(uint8_t adr, uint8_t *val)
{
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	*val = yas_u2dh_read_reg_byte(adr);

	return YAS_NO_ERROR;
}

static int32_t kc_set_register(uint8_t adr, uint8_t val)
{
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Check initialize */
	if (acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_write_reg_byte(adr, val);

	return YAS_NO_ERROR;
}

static int32_t kc_get_fs(void)
{
	int32_t ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	ret = kc_u2dh_get_fs();
	yas_u2dh_unlock();

	return ret;
}

static int32_t kc_set_fs(int32_t fs)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = kc_u2dh_set_fs(fs);
	yas_u2dh_unlock();

	return err;
}

static int32_t kc_get_cal_mode(void)
{
	int32_t ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	ret = kc_u2dh_get_cal_mode();
	yas_u2dh_unlock();

	return ret;
}

static int32_t kc_set_cal_mode(int32_t mode)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = kc_u2dh_set_cal_mode(mode);
	yas_u2dh_unlock();

	return err;
}

static int32_t kc_get_cal_state(void)
{
	int32_t ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	ret = kc_u2dh_get_cal_state();
	yas_u2dh_unlock();

	return ret;
}

static int32_t kc_set_cal_state(int32_t state)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = kc_u2dh_set_cal_state(state);
	yas_u2dh_unlock();

	return err;
}

static int32_t kc_get_cal_wait(void)
{
	int32_t ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	ret = kc_u2dh_get_cal_wait();
	yas_u2dh_unlock();

	return ret;
}

static int32_t kc_get_cal_smp_n(void)
{
	int32_t ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	ret = kc_u2dh_get_cal_smp_n();
	yas_u2dh_unlock();

	return ret;
}

static int32_t kc_set_cal_smp_n(int32_t smp_n)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = kc_u2dh_set_cal_smp_n(smp_n);
	yas_u2dh_unlock();

	return err;
}

static int32_t kc_get_cal_ave_n(void)
{
	int32_t ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	ret = kc_u2dh_get_cal_ave_n();
	yas_u2dh_unlock();

	return ret;
}

static int32_t kc_set_cal_ave_n(int32_t ave_n)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = kc_u2dh_set_cal_ave_n(ave_n);
	yas_u2dh_unlock();

	return err;
}

static int32_t kc_get_cal_flt_en(void)
{
	int32_t ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	ret = kc_u2dh_get_cal_flt_en();
	yas_u2dh_unlock();

	return ret;
}

static int32_t kc_set_cal_flt_en(int32_t flt_en)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = kc_u2dh_set_cal_flt_en(flt_en);
	yas_u2dh_unlock();

	return err;
}

static int32_t kc_set_ofs_en(int32_t ofs_en)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_u2dh_lock();
	err = kc_u2dh_set_ofs_en(ofs_en);
	yas_u2dh_unlock();

	return err;
}

/* -------------------------------------------------------------------------- */
/*  Global function                                                           */
/* -------------------------------------------------------------------------- */
int yas_acc_driver_init(struct yas_acc_driver *f)
{
	struct yas_acc_driver_callback *cbk;

	/* Check parameter */
	if (f == NULL)
		return YAS_ERROR_ARG;

	cbk = &f->callback;
	if (cbk->device_open == NULL ||
	    cbk->device_close == NULL ||
	    cbk->device_write == NULL ||
	    cbk->device_read == NULL ||
	    cbk->msleep == NULL ||
	    cbk->device_set_reset == NULL||
	    cbk->device_set_enable == NULL||
	    cbk->device_input_offset == NULL)
		return YAS_ERROR_ARG;

	/* Clear intialize */
	yas_u2dh_term();

	/* Set callback interface */
	cb.callback = *cbk;

	/* Set driver interface */
	f->init = yas_init;
	f->term = yas_term;
	f->get_delay = yas_get_delay;
	f->set_delay = yas_set_delay;
	f->get_offset = yas_get_offset;
	f->set_offset = yas_set_offset;
	f->get_enable = yas_get_enable;
	f->set_enable = yas_set_enable;
	f->get_filter = yas_get_filter;
	f->set_filter = yas_set_filter;
	f->get_filter_enable = yas_get_filter_enable;
	f->set_filter_enable = yas_set_filter_enable;
	f->get_position = yas_get_position;
	f->set_position = yas_set_position;
	f->measure = yas_measure;
	f->get_register = yas_get_register;
	f->set_register = kc_set_register;
	f->set_vsensor = kc_u2dh_set_vsensor;
	f->set_enable_cnt = kc_set_enable_cnt;
	f->get_acc_fs = kc_get_fs;
	f->set_acc_fs = kc_set_fs;
	f->get_cal_mode = kc_get_cal_mode;
	f->set_cal_mode = kc_set_cal_mode;
	f->get_cal_state = kc_get_cal_state;
	f->set_cal_state = kc_set_cal_state;
	f->get_cal_wait = kc_get_cal_wait;
	f->get_cal_smp_n = kc_get_cal_smp_n;
	f->set_cal_smp_n = kc_set_cal_smp_n;
	f->get_cal_ave_n = kc_get_cal_ave_n;
	f->set_cal_ave_n = kc_set_cal_ave_n;
	f->get_cal_flt_en = kc_get_cal_flt_en;
	f->set_cal_flt_en = kc_set_cal_flt_en;
	f->calibration = kc_calibration;
	f->set_ofs_en = kc_set_ofs_en;
	f->set_reset = kc_set_reset;
	pcb = &cb;

	return YAS_NO_ERROR;
}

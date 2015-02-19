/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/*
 * Copyright (c) 2011 Yamaha Corporation
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

#define USE_GYRO_IIR_FILTER                                                  (0)

/* 0:Active-Low 1:Active-High */
#define YAS_L3G3200D_INT_ACTIVE                                              (1)

/* Default parameters */
#define YAS_L3G3200D_DEFAULT_DELAY                                          (10)
#define YAS_L3G3200D_DEFAULT_POSITION      (CONFIG_INPUT_YAS_GYROSCOPE_POSITION)

#define YAS_L3G3200D_MAX_DELAY                                             (200)
#define YAS_L3G3200D_MIN_DELAY_INT                                           (0)
#define YAS_L3G3200D_MIN_DELAY_POL                                          (10)
#define YAS_L3G3200D_MAX_POSITION                                            (7)
#define YAS_L3G3200D_MIN_POSITION                                            (0)
#define YAS_L3G3200D_MAX_DATA                                          (2000000)
#define YAS_L3G3200D_MIN_DATA                                         (-2000000)

#define YAS_L3G3200D_POWERUP_TIME                                          (400)

/* Register */
#define YAS_L3G3200D_WHOAMI_REG                                           (0x0F)
#define YAS_L3G3200D_WHOAMI_ID                                            (0xD4)

#define YAS_L3G3200D_FS_REG                                               (0x23)
#define YAS_L3G3200D_FS_MASK                                              (0x30)
#define YAS_L3G3200D_FS_SHIFT                                                (4)
#define YAS_L3G3200D_FS_2000                                                 (2)
#define YAS_L3G3200D_FS_500                                                  (1)
#define YAS_L3G3200D_FS_250                                                  (0)

#define YAS_L3G3200D_BDU_REG                                              (0x23)
#define YAS_L3G3200D_BDU_MASK                                             (0x80)
#define YAS_L3G3200D_BDU_SHIFT                                               (7)

#define YAS_L3G3200D_ODR_REG                                              (0x20)
#define YAS_L3G3200D_ODR_MASK                                             (0xF0)
#define YAS_L3G3200D_ODR_SHIFT                                               (4)
#define YAS_L3G3200D_ODR_760HZ                                              (12)
#define YAS_L3G3200D_ODR_380HZ                                               (8)
#define YAS_L3G3200D_ODR_190HZ                                               (4)
#define YAS_L3G3200D_ODR_95HZ                                                (0)

#define KC_L3G3200D_BW                                                       (3)
#define KC_L3G3200D_ODR_760HZ          (YAS_L3G3200D_ODR_760HZ + KC_L3G3200D_BW)
#define KC_L3G3200D_ODR_380HZ          (YAS_L3G3200D_ODR_380HZ + KC_L3G3200D_BW)
#define KC_L3G3200D_ODR_190HZ          (YAS_L3G3200D_ODR_190HZ + KC_L3G3200D_BW)
#define KC_L3G3200D_ODR_95HZ           (YAS_L3G3200D_ODR_95HZ  + KC_L3G3200D_BW)

#define YAS_L3G3200D_AXIS_REG                                             (0x20)
#define YAS_L3G3200D_AXIS_MASK                                            (0x07)
#define YAS_L3G3200D_AXIS_SHIFT                                              (0)
#define YAS_L3G3200D_AXIS_ENABLE                                          (0x07)
#define YAS_L3G3200D_AXIS_DISABLE                                         (0x00)

#define YAS_L3G3200D_INTERRUPT_REG                                        (0x22)
#define YAS_L3G3200D_INTERRUPT_MASK                                       (0x06)
#define YAS_L3G3200D_INTERRUPT_SHIFT                                         (1)
#define YAS_L3G3200D_INTERRUPT_ENABLE                                        (3)
#define YAS_L3G3200D_INTERRUPT_DISABLE                                       (0)
#if YAS_L3G3200D_INT_ACTIVE == 1
#define YAS_L3G3200D_INTERRUPT_CFG                                        (0x00)
#else
#define YAS_L3G3200D_INTERRUPT_CFG                                        (0x20)
#endif
#define YAS_L3G3200D_POWER_REG                                            (0x20)
#define YAS_L3G3200D_POWER_MASK                                           (0x08)
#define YAS_L3G3200D_POWER_SHIFT                                             (3)

#define YAS_L3G3200D_FIFO_REG                                             (0x2E)
#define YAS_L3G3200D_FIFO_BYPASS                                          (0x00)
#define YAS_L3G3200D_FIFO_FIFO                                            (0x20)
#define YAS_L3G3200D_FIFO_STREAM                                          (0x40)
#define YAS_L3G3200D_FIFO_STREAM_FIFO                                     (0x60)
#define YAS_L3G3200D_FIFO_BYPASS_STREAM                                   (0x80)
#define YAS_L3G3200D_FIFO_MAX                                  YAS_GYRO_FIFO_MAX

#define YAS_L3G3200D_FIFO_ENABLE_REG                                      (0x24)
#define YAS_L3G3200D_FIFO_ENABLE_MASK                                     (0x40)
#define YAS_L3G3200D_FIFO_ENABLE_SHIFT                                       (6)
#define YAS_L3G3200D_FIFO_ENABLE_ENABLE                                   (0x5c)
#define YAS_L3G3200D_FIFO_ENABLE_DISABLE                                  (0x1c)

#define YAS_L3G3200D_FIFO_STATUS_REG                                      (0x2F)

#define YAS_L3G3200D_STATUS_REG                                           (0x27)
#define YAS_L3G3200D_DATA_REG                                             (0x28)

#define YAS_L3G3200D_RESOLUTION                                             (70)
#define YAS_L3G3200D_INITIAL_DATA_TIME                                     (320)

#define KC_L3G3200D_20_REG                                                (0x20)
#define KC_L3G3200D_21_REG                                                (0x21)
#define KC_L3G3200D_22_REG                                                (0x22)
#define KC_L3G3200D_23_REG                                                (0x23)
#define KC_L3G3200D_24_REG                                                (0x24)
#define KC_L3G3200D_20_CFG                                                (0x77)
#define KC_L3G3200D_21_CFG                                                   (0)
#define KC_L3G3200D_22_CFG                                                   (0)
#define KC_L3G3200D_23_CFG                                                (0xB0)
#define KC_L3G3200D_24_CFG                                                (0x80)

/* -------------------------------------------------------------------------- */
/*  Structure definition                                                      */
/* -------------------------------------------------------------------------- */
/* Output data rate */
struct yas_l3g3200d_odr {
	int delay;                    /* min delay (msec) in the range of ODR */
	unsigned char odr;            /* bandwidth register value             */
	int32_t fs;
};

/* Driver private data */
struct yas_l3g3200d_data {
	int initialize;
	int device_open;
	int enable;
	int delay;
	int actual_delay;
	int position;
	int threshold;
	int filter_enable;
	int interrupt;
	int overrun;
	int initial_data;
	int wtm;
	struct yas_gyro_data last; /* mdps */
	struct yas_vector offset;
	int32_t fs;
	int32_t calbretion_mode;
	int32_t cal_sample_num;
	int32_t cal_acc_sample_num;
	uint32_t cal_chk_mode;
	int32_t acc_calbretion;
	int32_t acc_cal_thershold;
	int32_t gyro_cal_thershold;
	int32_t iir_filter;
	unsigned long iir_filter_cnt;
	double filter_b0;
	double filter_b1;
	double filter_b2;
	double filter_a1;
	double filter_a2;
	double iir_w[3][3];
	struct yas_vector add_val;
	struct yas_vector average;
};

/* -------------------------------------------------------------------------- */
/*  Data                                                                      */
/* -------------------------------------------------------------------------- */
/* Control block */
static struct yas_gyro_driver    cb;
static struct yas_gyro_driver   *pcb;
static struct yas_l3g3200d_data  l3g3200d_data;
static uint8_t                   l3g3200d_buf[YAS_L3G3200D_FIFO_MAX * 6];

static struct sensor_power_callback gyro_power_cb;
atomic_t at_device_reset = ATOMIC_INIT(0);

/* Output data rate */
static const struct yas_l3g3200d_odr yas_l3g3200d_odr_tbl[] = {
	{2,   KC_L3G3200D_ODR_760HZ,   760},
	{3,   KC_L3G3200D_ODR_380HZ,   380},
	{10,  KC_L3G3200D_ODR_190HZ,   190},
	{11,  KC_L3G3200D_ODR_95HZ ,    95},
};

/* Transformation matrix for chip mounting position */
static const int yas_l3g3200d_position_map[][3][3] = {
	{ { 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} }, /* top/upper-left */
	{ { 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} }, /* top/upper-right */
	{ {-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} }, /* top/lower-right */
	{ { 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} }, /* top/lower-left */
	{ {-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} }, /* bottom/upper-left */
	{ { 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} }, /* bottom/upper-right */
	{ { 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} }, /* bottom/lower-right */
	{ { 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} }, /* bottom/lower-left */
};

/* -------------------------------------------------------------------------- */
/*  Prototype declaration                                                     */
/* -------------------------------------------------------------------------- */
static int  yas_l3g3200d_read_reg(unsigned char, unsigned char *, int);
static int  yas_l3g3200d_write_reg(unsigned char, unsigned char *, int);
static unsigned char  yas_l3g3200d_read_reg_byte(unsigned char);
static int  yas_l3g3200d_write_reg_byte(unsigned char, unsigned char);
static void yas_l3g3200d_init_offset(void);
static void yas_l3g3200d_init_lastvalue(void);
static void yas_l3g3200d_init_gyro_data(struct yas_gyro_data *);
static void yas_l3g3200d_init_data(void);
static void yas_l3g3200d_reset_fifo(void);
static void yas_l3g3200d_configure(void);
static int  yas_l3g3200d_ischg_enable(int);
static int  yas_l3g3200d_set_wtm(void);

static int  yas_l3g3200d_lock(void);
static int  yas_l3g3200d_unlock(void);
static int  yas_l3g3200d_device_open(void);
static int  yas_l3g3200d_device_close(void);
static int  yas_l3g3200d_interrupt_enable(void);
static int  yas_l3g3200d_interrupt_disable(void);
static void yas_l3g3200d_interrupt_notify(int);
#if 0
static int  yas_l3g3200d_msleep(int);
#endif

static int32_t  yas_l3g3200d_reset_init(void);
static int32_t  yas_l3g3200d_reset_term(void);
static int  yas_l3g3200d_init(void);
static int  yas_l3g3200d_term(void);
static int  yas_l3g3200d_get_delay(void);
static int  yas_l3g3200d_set_delay(int);
static int  yas_l3g3200d_get_enable(void);
static int  yas_l3g3200d_set_enable(int);
static int  yas_l3g3200d_get_position(void);
static int  yas_l3g3200d_set_position(int);
static int  yas_l3g3200d_get_offset(struct yas_vector *);
static int  yas_l3g3200d_set_offset(struct yas_vector *);
static int  yas_l3g3200d_get_filter(struct yas_gyro_filter *);
static int  yas_l3g3200d_set_filter(struct yas_gyro_filter *);
static int  yas_l3g3200d_get_filter_enable(void);
static int  yas_l3g3200d_set_filter_enable(int);
static int  yas_l3g3200d_get_interrupt(void);
static int  yas_l3g3200d_set_interrupt(int);

static int  yas_l3g3200d_measure_fifo(struct yas_gyro_data *, int);
static int  yas_l3g3200d_measure_direct(struct yas_gyro_data *);
static int  yas_l3g3200d_measure(struct yas_gyro_data *, int);

static void kc_l3g3200d_set_enable_cnt(int enable);
static int32_t kc_l3g3200d_get_fs( void );
static int32_t kc_l3g3200d_set_fs( int32_t );
static int32_t kc_l3g3200d_get_iir_filter( void );
static int32_t kc_l3g3200d_set_iir_filter( int32_t );
static double  kc_l3g3200d_get_filter_b0( void );
static double  kc_l3g3200d_get_filter_b1( void );
static double  kc_l3g3200d_get_filter_b2( void );
static double  kc_l3g3200d_get_filter_a1( void );
static double  kc_l3g3200d_get_filter_a2( void );
static int32_t kc_l3g3200d_set_filter_b0( double );
static int32_t kc_l3g3200d_set_filter_b1( double );
static int32_t kc_l3g3200d_set_filter_b2( double );
static int32_t kc_l3g3200d_set_filter_a1( double );
static int32_t kc_l3g3200d_set_filter_a2(  double );
static int32_t kc_l3g3200d_iir_filter( struct yas_gyro_data * );
static int yas_init(void);
static int yas_term(void);
/* -------------------------------------------------------------------------- */
/*  Local functions                                                           */
/* -------------------------------------------------------------------------- */
static void kc_l3g3200d_power_on(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	YLOGI(("%s(): [IN]\n",__func__));

	yas_l3g3200d_reset_init();
	cbk->device_set_reset(1);

	YLOGI(("%s(): [OUT]\n",__func__));
}
static void kc_l3g3200d_power_off(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	YLOGI(("%s(): [IN]\n",__func__));

	cbk->device_set_reset(0);
	yas_l3g3200d_reset_term();

	YLOGI(("%s(): [OUT]\n",__func__));
}


static void kc_l3g3200d_set_vsensor(int32_t enable)
{
	if (enable) {
		sensor_power_on(SENSOR_INDEX_GYRO);
		sensor_power_reg_cbfunc(&gyro_power_cb);
	}
	else {
		sensor_power_unreg_cbfunc(&gyro_power_cb);
		sensor_power_off(SENSOR_INDEX_GYRO);
	}
}

static int32_t kc_l3g3200d_reset( void )
{
	int32_t reset = atomic_read( &at_device_reset );

	if(reset == 1) {
		YLOGD(("%s(): reset start\n",__func__));
		atomic_set( &at_device_reset, 2 );
		sensor_power_reset(SENSOR_INDEX_GYRO);
		reset = atomic_read( &at_device_reset );
		if (reset != 3)
			atomic_set( &at_device_reset, 0 );
		YLOGD(("%s(): reset end reset=%d\n",__func__,reset));
	}

	return YAS_NO_ERROR;
}

/* register access functions */
static int yas_l3g3200d_read_reg(unsigned char adr, unsigned char *buf, int len)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int err = YAS_NO_ERROR;
	int32_t err_cnt = 0;
	int32_t reset = 0;

	if (p->device_open) {
		do {
			err = cbk->device_read(adr, buf, len);
			if (err != 0 && (++err_cnt >= KC_SENSOR_DEVICE_ERR_MAX)) {
				reset = atomic_read(&at_device_reset);
				YLOGE(("%s(): [ERR device_read] reset=%d\n",__func__,reset));
				if(reset == 0)
					atomic_set( &at_device_reset, 1 );
				if(reset == 2)
					atomic_set( &at_device_reset, 3 );
			}
		} while(err != 0 && (err_cnt < KC_SENSOR_DEVICE_ERR_MAX));
	}

	return err;
}

static int yas_l3g3200d_write_reg(unsigned char adr, unsigned char *buf,
				  int len)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int err = YAS_NO_ERROR;
	int32_t err_cnt = 0;
	int32_t reset = 0;

	if (p->device_open) {
		do {
			err = cbk->device_write(adr, buf, len);
			if (err != 0 && (++err_cnt >= KC_SENSOR_DEVICE_ERR_MAX)) {
				reset = atomic_read(&at_device_reset);
				YLOGE(("%s(): [ERR device_write] reset=%d\n",__func__,reset));
				if(reset == 0)
					atomic_set( &at_device_reset, 1 );
				if(reset == 2)
					atomic_set( &at_device_reset, 3 );
			}
		} while(err != 0 && (err_cnt < KC_SENSOR_DEVICE_ERR_MAX));
	}

	return err;
}

static unsigned char yas_l3g3200d_read_reg_byte(unsigned char adr)
{
	unsigned char buf;
	int err;

	err = yas_l3g3200d_read_reg(adr, &buf, 1);
	if (err == 0)
		return buf;

	return 0;
}

static int yas_l3g3200d_write_reg_byte(unsigned char adr, unsigned char val)
{
	return yas_l3g3200d_write_reg(adr, &val, 1);
}

#define yas_l3g3200d_read_bits(r) \
	((yas_l3g3200d_read_reg_byte(r##_REG) & r##_MASK) >> r##_SHIFT)

#define yas_l3g3200d_update_bits(r, v) \
	yas_l3g3200d_write_reg_byte(r##_REG, \
				    ((yas_l3g3200d_read_reg_byte(r##_REG) & \
				      ~r##_MASK) | ((v) << r##_SHIFT)))

static void yas_l3g3200d_init_offset(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int i;

	for (i = 0; i < 3; ++i)
		p->offset.v[i] = 0;
}

static void yas_l3g3200d_init_lastvalue(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int i;

	for (i = 0; i < 3; ++i) {
		p->last.xyz.v[i] = 0;
		p->last.raw.v[i] = 0;
	}
}

static void yas_l3g3200d_init_gyro_data(struct yas_gyro_data *data)
{
	int i;

	for (i = 0; i < 3; ++i) {
		data->xyz.v[i] = 0;
		data->raw.v[i] = 0;
	}

	data->num = 0;
	data->overrun = 0;
}

static void yas_l3g3200d_reset_init_data(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	p->initialize = 0;
	p->interrupt = 0;
	p->overrun = 0;
	p->wtm = 0;
	memset(p->iir_w,0,sizeof(p->iir_w));
	memset(&p->add_val,0,sizeof(struct yas_vector));
	memset(&p->average,0,sizeof(struct yas_vector));
}

static void yas_l3g3200d_init_data(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	p->initialize = 0;
	p->device_open = 0;
	p->enable = 0;
	p->delay = YAS_L3G3200D_DEFAULT_DELAY;
	p->position = YAS_L3G3200D_DEFAULT_POSITION;
	p->threshold = YAS_GYRO_DEFAULT_FILTER_THRESH;
	p->filter_enable = 0;
	p->interrupt = 0;
	p->overrun = 0;
	p->wtm = 0;
	yas_l3g3200d_init_offset();
	yas_l3g3200d_init_lastvalue();
	p->fs = 100;
	p->calbretion_mode = 0;
	p->cal_sample_num = 128;
	p->cal_acc_sample_num = 50;
	p->cal_chk_mode = 0;
	p->acc_calbretion = 0;
	p->acc_cal_thershold = 20;
	p->gyro_cal_thershold = 10;
#if USE_GYRO_IIR_FILTER
	p->iir_filter = 1;
	p->iir_filter_cnt = 0;
	p->filter_b0 = 1;
	p->filter_b1 = 0;
	p->filter_b2 = 0;
	p->filter_a1 = 0;
	p->filter_a2 = 0;
#else
	p->iir_filter = 0;
	p->iir_filter_cnt = 0;
#endif /* USE_GYRO_IIR_FILTER */
	memset(p->iir_w,0,sizeof(p->iir_w));
	memset(&p->add_val,0,sizeof(struct yas_vector));
	memset(&p->average,0,sizeof(struct yas_vector));
}

static void yas_l3g3200d_reset_fifo(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	yas_l3g3200d_write_reg_byte(YAS_L3G3200D_FIFO_REG, 0);
	yas_l3g3200d_write_reg_byte(YAS_L3G3200D_FIFO_REG,
				    YAS_L3G3200D_FIFO_FIFO | p->wtm);
}

static void yas_l3g3200d_configure(void)
{
	yas_l3g3200d_write_reg_byte(KC_L3G3200D_24_REG,
				    KC_L3G3200D_24_CFG);
	msleep(10);

	yas_l3g3200d_write_reg_byte(KC_L3G3200D_20_REG,
				    KC_L3G3200D_20_CFG);

	yas_l3g3200d_write_reg_byte(KC_L3G3200D_23_REG,
				    KC_L3G3200D_23_CFG);

	/* Powerup */
	yas_l3g3200d_update_bits(YAS_L3G3200D_POWER, 1);
	msleep(YAS_L3G3200D_POWERUP_TIME);

}

static int yas_l3g3200d_ischg_enable(int enable)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	if(enable) {
		if (p->enable > 0)
			return 0;
	} else
		if (p->enable != 1)
			return 0;

	return 1;
}

static int  yas_l3g3200d_set_wtm(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int wtm;

	wtm = p->delay / p->actual_delay;
	if (wtm > YAS_L3G3200D_FIFO_MAX) {
		wtm = YAS_L3G3200D_FIFO_MAX;
		return 0;
	}

	p->wtm = (wtm > 2) ? wtm : 2;

	return 0;
}

static int yas_l3g3200d_lock(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	int err;

	if (cbk->lock != NULL && cbk->unlock != NULL)
		err = cbk->lock();
	else
		err = YAS_NO_ERROR;

	return err;
}

static int yas_l3g3200d_unlock(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	int err;

	if (cbk->lock != NULL && cbk->unlock != NULL)
		err = cbk->unlock();
	else
		err = YAS_NO_ERROR;

	kc_l3g3200d_reset();
	return err;
}

static int yas_l3g3200d_device_open(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int err;

	if (p->device_open == 0) {
		kc_l3g3200d_set_vsensor(1);
		err = cbk->device_open();
		if (err != YAS_NO_ERROR)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		p->device_open = 1;
	}

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_device_close(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int err;

	if (p->device_open != 0) {
		err = cbk->device_close();
		if (err != YAS_NO_ERROR)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		p->device_open = 0;
		kc_l3g3200d_set_vsensor(0);
	}

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_interrupt_enable(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int err;

	if (p->interrupt == 0)
		return YAS_NO_ERROR;

	err = cbk->interrupt_enable();
	if (err != YAS_NO_ERROR)
		return YAS_ERROR_INTERRUPT;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_interrupt_disable(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int err;

	if (p->interrupt == 0)
		return YAS_NO_ERROR;

	err = cbk->interrupt_disable();
	if (err != YAS_NO_ERROR)
		return YAS_ERROR_INTERRUPT;

	return YAS_NO_ERROR;
}

static void yas_l3g3200d_interrupt_notify(int num)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	if (p->interrupt == 0)
		return;

	cbk->interrupt_notify(num);
}

#if 0
static int yas_l3g3200d_msleep(int msec)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;

	if (msec <= 0)
		return YAS_ERROR_ARG;

	cbk->msleep(msec);

	return YAS_NO_ERROR;
}
#endif

static int yas_l3g3200d_reset_init(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check intialize */
	if (p->initialize == 1)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_write_reg_byte(KC_L3G3200D_24_REG,
				    KC_L3G3200D_24_CFG);
	msleep(10);

	yas_l3g3200d_write_reg_byte(KC_L3G3200D_20_REG,
				    KC_L3G3200D_20_CFG);

	yas_l3g3200d_write_reg_byte(KC_L3G3200D_23_REG,
				    KC_L3G3200D_23_CFG);

	/* Init data */
	yas_l3g3200d_reset_init_data();

	p->initialize = 1;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_reset_term(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Close i2c/spi */
	p->initialize = 0;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_init(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	unsigned char id;
	int err;

	/* Check intialize */
	if (p->initialize == 1)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Init data */
	yas_l3g3200d_init_data();

	/* Open i2c/spi */
	err = yas_l3g3200d_device_open();
	if (err != YAS_NO_ERROR)
		return err;

	/* Check id */
	id = yas_l3g3200d_read_reg_byte(YAS_L3G3200D_WHOAMI_REG);
	if (id != YAS_L3G3200D_WHOAMI_ID) {
		yas_l3g3200d_device_close();
		return YAS_ERROR_CHIP_ID;
	}

	yas_l3g3200d_write_reg_byte(KC_L3G3200D_24_REG,
				    KC_L3G3200D_24_CFG);
	msleep(10);

	yas_l3g3200d_write_reg_byte(KC_L3G3200D_20_REG,
				    KC_L3G3200D_20_CFG);

	yas_l3g3200d_write_reg_byte(KC_L3G3200D_23_REG,
				    KC_L3G3200D_23_CFG);

	yas_l3g3200d_device_close();

	p->initialize = 1;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_term(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_set_enable(0);

	/* Close i2c/spi */
	yas_l3g3200d_device_close();

	p->initialize = 0;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_get_delay(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->delay;
}

static int yas_l3g3200d_set_delay(int delay)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	unsigned char odr;
	int i;

	YLOGD(("%s(): [S] p->delay=%d delay=%d\n",__func__,p->delay,delay));
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Determine optimum odr */
	for (i = 1; i < (int)(sizeof(yas_l3g3200d_odr_tbl) /
			      sizeof(struct yas_l3g3200d_odr)) &&
		     delay >= (int)yas_l3g3200d_odr_tbl[i].delay; i++)
		;

	odr = yas_l3g3200d_odr_tbl[i-1].odr;
	p->delay = delay;
	p->actual_delay = yas_l3g3200d_odr_tbl[i-1].delay;
	p->fs = yas_l3g3200d_odr_tbl[i-1].fs;
	YLOGD(("%s(): p->delay=%d p->actual_delay=%d p->fs=%d\n",__func__,p->delay,p->actual_delay,p->fs));

	if (p->enable) {
		if (p->interrupt)
			yas_l3g3200d_set_wtm();
		yas_l3g3200d_update_bits(YAS_L3G3200D_ODR, odr);
		if (p->interrupt) {
			/* yas_l3g3200d_msleep(10); */
			msleep(10);
			yas_l3g3200d_reset_fifo();
		}
	}

	YLOGD(("%s(): [E] p->delay=%d delay=%d\n",__func__,p->delay,delay));
	return YAS_NO_ERROR;
}

static int yas_l3g3200d_get_enable(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->enable;
}

static int yas_l3g3200d_set_enable(int enable)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int err;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGD(("%s(): [S] p->enable=%d enable=%d\n",__func__,p->enable,enable));
	if (enable) {
		if (yas_l3g3200d_ischg_enable(enable)) {
			if (p->enable >= 1)
				return YAS_NO_ERROR;

			p->iir_filter_cnt = 0;
			memset(p->iir_w,0,sizeof(p->iir_w));
			memset(&p->add_val,0,sizeof(struct yas_vector));
			memset(&p->average,0,sizeof(struct yas_vector));

			p->overrun = 0;

			/* Open transfer i2c/spi */
			err = yas_l3g3200d_device_open();
			if (err != YAS_NO_ERROR)
				return err;

			/* init data */
			yas_l3g3200d_init_lastvalue();
			p->initial_data = 0;
			p->enable = 1;
			YLOGD(("%s(): DISABLE -> ENABLE\n",__func__));

			/* Set DLPF */
			yas_l3g3200d_set_delay(p->delay);

			/* Set configuration */
			yas_l3g3200d_configure();
#if DEBUG
			{
				int32_t reg;
				uint8_t i;
				for (i=0x20;i<0x38;i++) {
					reg = yas_l3g3200d_read_reg_byte(i);
					YLOGD(("reg[0x%02x]=0x%02x\n",i,reg));
				}
			}
#endif
		} else
			p->enable++;
	} else {
		if (yas_l3g3200d_ischg_enable(enable)) {
			if (p->enable <= 0)
				return YAS_NO_ERROR;

			/* Powerdown */
			yas_l3g3200d_update_bits(YAS_L3G3200D_POWER, 0);

			/* Close transfer i2c/spi */
			err = yas_l3g3200d_device_close();
			if (err != YAS_NO_ERROR) {
				/* Set configuration */
				yas_l3g3200d_configure();
				return err;
			}

			p->enable = 0;
			YLOGD(("%s(): ENABLE -> DISABLE\n",__func__));
		} else 
			if(--p->enable < 0)
				p->enable = 0;
	}
	YLOGD(("%s(): [E] p->enable=%d enable=%d\n",__func__,p->enable,enable));

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_get_position(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->position;
}

static int yas_l3g3200d_set_position(int position)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	p->position = position;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_get_offset(struct yas_vector *offset)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	*offset = p->offset;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_set_offset(struct yas_vector *offset)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	p->offset = *offset;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_get_filter(struct yas_gyro_filter *filter)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	filter->threshold = p->threshold;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_set_filter(struct yas_gyro_filter *filter)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	p->threshold = filter->threshold;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_get_filter_enable(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->filter_enable;
}

static int yas_l3g3200d_set_filter_enable(int enable)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	p->filter_enable = enable;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_get_interrupt(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->interrupt;
}

static int yas_l3g3200d_set_interrupt(int interrupt)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	if (interrupt == 0)
		yas_l3g3200d_interrupt_disable();

	p->interrupt = interrupt;

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_data_filter(struct yas_gyro_data *out_data)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int d_x, d_y, d_z;

	if (p->filter_enable) {
		d_x = ABS(p->last.xyz.v[0] - out_data->xyz.v[0]);
		d_y = ABS(p->last.xyz.v[1] - out_data->xyz.v[1]);
		d_z = ABS(p->last.xyz.v[2] - out_data->xyz.v[2]);
		if (d_x > p->threshold ||
		    d_y > p->threshold ||
		    d_z > p->threshold)
			;
		else
			*out_data = p->last;
	}

	return YAS_NO_ERROR;
}

static int yas_l3g3200d_measure_fifo(struct yas_gyro_data *out_data, int num)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	unsigned char reg;
	int32_t raw[3];
	int data_num;
	int pos = p->position;
	int i, j, k;

	data_num = 0;

	/* Check devie */
	if (p->device_open == 0) {
		out_data[0] = p->last;
		out_data[0].num = 1;
		out_data[0].overrun = p->overrun;
		yas_l3g3200d_interrupt_enable();
		return 1;
	}

	/* Read FIFO status */
	reg = yas_l3g3200d_read_reg_byte(YAS_L3G3200D_FIFO_STATUS_REG);

	/* Check FIFO stored data level */
	data_num = (reg & 0x1f) + 1;
	if (num < data_num)
		data_num = num;

	/* Disable interrupt */
	yas_l3g3200d_update_bits(YAS_L3G3200D_INTERRUPT,
				 YAS_L3G3200D_INTERRUPT_DISABLE);

	/* Read angle rate data */
	for (i = 0; i < data_num; i++) {
		if (yas_l3g3200d_read_reg(YAS_L3G3200D_DATA_REG | 0x80,
					  &l3g3200d_buf[i*6], 6) != 0) {
			out_data[0] = p->last;
			out_data[0].num = 1;
			out_data[0].overrun = p->overrun;
			yas_l3g3200d_reset_fifo();
			yas_l3g3200d_update_bits(YAS_L3G3200D_INTERRUPT,
						 YAS_L3G3200D_INTERRUPT_ENABLE);
			yas_l3g3200d_interrupt_enable();
			return 1;
		}
	}

	yas_l3g3200d_reset_fifo();

	/* Enable interrupt */
	yas_l3g3200d_update_bits(YAS_L3G3200D_INTERRUPT,
				 YAS_L3G3200D_INTERRUPT_ENABLE);

	for (i = 0; i < data_num; ++i) {
		for (j = 0; j < 3; ++j)
			raw[j] = ((int16_t)((l3g3200d_buf[i*6+j*2+1] << 8)) |
				  l3g3200d_buf[i*6+j*2]);

		for (j = 0; j < 3; ++j) {
			/* coordinate transformation */
			out_data[i].raw.v[j] = 0;
			for (k = 0; k < 3; ++k)
				out_data[i].raw.v[j] += raw[k] *
					yas_l3g3200d_position_map[pos][j][k];
			out_data[i].xyz.v[j] = out_data[i].raw.v[j] *
				YAS_L3G3200D_RESOLUTION;
		}
		yas_l3g3200d_data_filter(&out_data[i]);
		p->last = out_data[i];
		out_data[i].xyz.v[0] -= p->offset.v[0];
		out_data[i].xyz.v[1] -= p->offset.v[1];
		out_data[i].xyz.v[2] -= p->offset.v[2];
		out_data[i].overrun = p->overrun;
		out_data[i].num = data_num;
	}

	/* Enable interrupt */
	yas_l3g3200d_interrupt_enable();

	return data_num;
}

static int yas_l3g3200d_measure_direct(struct yas_gyro_data *out_data)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int32_t raw[3];
	int pos = p->position;
	int i, j;

	/* Check device */
	if (p->device_open == 0) {
		out_data[0] = p->last;
		out_data[0].num = 1;
		out_data[0].overrun = p->overrun;
		return 1;
	}

	/* Dummy read angle rate data */
	if (yas_l3g3200d_read_reg(YAS_L3G3200D_DATA_REG | 0x80,
				  &l3g3200d_buf[0], 6) != 0) {
		out_data[0] = p->last;
		return 1;
	}

	/* Read angle rate data */
	if (yas_l3g3200d_read_reg(YAS_L3G3200D_DATA_REG | 0x80,
				  &l3g3200d_buf[0], 6) != 0) {
		out_data[0] = p->last;
		return 1;
	}

	for (i = 0; i < 3; ++i)
		raw[i] = ((int16_t)((l3g3200d_buf[i*2+1] << 8)) |
			  l3g3200d_buf[i*2]);

	for (i = 0; i < 3; ++i) {
		/* coordinate transformation */
		out_data->raw.v[i] = 0;
		for (j = 0; j < 3; ++j)
			out_data->raw.v[i] += raw[j] *
				yas_l3g3200d_position_map[pos][i][j];
		out_data->xyz.v[i] = out_data->raw.v[i] *
			YAS_L3G3200D_RESOLUTION;
	}
	yas_l3g3200d_data_filter(out_data);
	p->last = out_data[0];
	out_data->xyz.v[0] -= p->offset.v[0];
	out_data->xyz.v[1] -= p->offset.v[1];
	out_data->xyz.v[2] -= p->offset.v[2];
	out_data->num = 1;
	out_data->overrun = p->overrun;

	return 1;
}

static int yas_l3g3200d_measure(struct yas_gyro_data *out_data, int num)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int actual_num;
	int i;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	for (i = 0; i < num; i++)
		yas_l3g3200d_init_gyro_data(&out_data[i]);

	if (p->interrupt)
		actual_num = yas_l3g3200d_measure_fifo(out_data, num);
	else
		actual_num = yas_l3g3200d_measure_direct(out_data);

	return actual_num;
}

static void kc_l3g3200d_set_enable_cnt(int enable)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	p->enable = enable;
}

static int32_t kc_l3g3200d_get_fs( void )
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;
	YLOGI(("%s(): [IN]\n",__func__));
	YLOGD(("%s(): fs = %d\n",__func__,p->fs));
	return p->fs;
}

static int32_t kc_l3g3200d_set_fs( int32_t fs )
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	uint8_t odr;
	int32_t i;
	int32_t array_size;
	YLOGI(("%s(): [IN]\n",__func__));

	/* Check initialize */
	if (p->initialize == 0){
		YLOGE(("%s(): [ERR initialize]\n",__func__));
		return YAS_ERROR_NOT_INITIALIZED;
	}

	/* Determine optimum odr */
	array_size = (int32_t)(sizeof(yas_l3g3200d_odr_tbl) / sizeof(struct yas_l3g3200d_odr));
	for (i = 1; i < array_size; i++)
	{
		if(fs > yas_l3g3200d_odr_tbl[i].fs)
			break;
	}

	odr = yas_l3g3200d_odr_tbl[i-1].odr;
	p->delay = yas_l3g3200d_odr_tbl[i-1].delay;
	p->actual_delay = yas_l3g3200d_odr_tbl[i-1].delay;
	p->fs = yas_l3g3200d_odr_tbl[i-1].fs;
	
	if (p->enable) {
		if (p->interrupt)
			yas_l3g3200d_set_wtm();
		yas_l3g3200d_update_bits(YAS_L3G3200D_ODR, odr);
	}
	YLOGI(("%s(): odr=%d delay=%d fs=%d\n",__func__,odr,p->delay,p->fs));

	return YAS_NO_ERROR;

}

static int32_t kc_l3g3200d_get_iir_filter( void )
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->iir_filter;

}
static int32_t kc_l3g3200d_set_iir_filter( int32_t iir )
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	if(p->iir_filter != iir) {
		p->iir_filter_cnt = 0;
		memset(p->iir_w,0,sizeof(p->iir_w));
	}
	p->iir_filter = iir;

	return YAS_NO_ERROR;
}
static double kc_l3g3200d_get_filter_b0()
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;
	YLOGI(("%s(): [IN]\n",__func__));

	return p->filter_b0;

}
static int32_t kc_l3g3200d_set_filter_b0( double filter )
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	YLOGD(("%s(): filter_b0 = %d\n",__func__,(int32_t)filter));

	p->filter_b0 = filter/10000;

	return YAS_NO_ERROR;

}
static double kc_l3g3200d_get_filter_b1()
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	return p->filter_b1;
}
static int32_t kc_l3g3200d_set_filter_b1( double filter )
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	YLOGD(("%s(): filter_b1 = %d\n",__func__,(int32_t)filter));
	p->filter_b1 = filter/10000;

	return YAS_NO_ERROR;
}
static double kc_l3g3200d_get_filter_b2()
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;
	YLOGI(("%s(): [IN]\n",__func__));

	return p->filter_b2;
}
static int32_t kc_l3g3200d_set_filter_b2( double filter )
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	YLOGD(("%s(): filter_b2 = %d\n",__func__,(int32_t)filter));
	p->filter_b2 = filter/10000;

	return YAS_NO_ERROR;
}
static double kc_l3g3200d_get_filter_a1()
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));

	return p->filter_a1;
}
static int32_t kc_l3g3200d_set_filter_a1( double filter )
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	YLOGD(("%s(): filter_a1 = %d\n",__func__,(int32_t)filter));

	p->filter_a1 = filter/10000;

	return YAS_NO_ERROR;
}
static double kc_l3g3200d_get_filter_a2()
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));

	return p->filter_a2;
}
static int32_t kc_l3g3200d_set_filter_a2( double filter )
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN]\n",__func__));
	YLOGD(("%s(): filter_a2 = %d\n",__func__,(int32_t)filter));

	p->filter_a2 = filter/10000;

	return YAS_NO_ERROR;
}

static int32_t kc_l3g3200d_iir_filter(struct yas_gyro_data *data)
{
#if USE_GYRO_IIR_FILTER
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int32_t ret = YAS_NO_ERROR;
	int32_t axis;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	for(axis=0;axis<3;axis++)
		switch(p->iir_filter_cnt) {
		case 0:
			p->iir_w[p->iir_filter_cnt][axis] = data->xyz.v[axis];
			data->xyz.v[axis] = p->iir_w[p->iir_filter_cnt][axis] * p->filter_b0;
			break;
		case 1:
			p->iir_w[p->iir_filter_cnt][axis] = p->iir_w[p->iir_filter_cnt-1][axis] * p->filter_a1 +
				data->xyz.v[axis];
			data->xyz.v[axis] = p->iir_w[p->iir_filter_cnt][axis] * p->filter_b0 +
				p->iir_w[p->iir_filter_cnt-1][axis] * p->filter_b1;
			break;
		default:
			p->iir_w[(p->iir_filter_cnt)%3][axis] = p->iir_w[(p->iir_filter_cnt-1)%3][axis] * p->filter_a1 + 
				p->iir_w[(p->iir_filter_cnt-2)%3][axis] * p->filter_a2 +
				data->xyz.v[axis];
			data->xyz.v[axis] = p->iir_w[(p->iir_filter_cnt)%3][axis] * p->filter_b0 +
				p->iir_w[(p->iir_filter_cnt-1)%3][axis] * p->filter_b1 +
				p->iir_w[(p->iir_filter_cnt-2)%3][axis] * p->filter_b2;
			break;
		}
	p->iir_filter_cnt++;
	
	return ret;
#else
	return YAS_NO_ERROR;
#endif /* USE_GYRO_IIR_FILTER */
}

static int32_t kc_l3g3200d_set_reset(int32_t reset)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN] reset=%d\n",__func__,reset));
	if (reset) {
		/* Powerup */
		yas_l3g3200d_update_bits(YAS_L3G3200D_POWER, 1);
		msleep(YAS_L3G3200D_POWERUP_TIME);
	} else {
		/* Powerdown */
		yas_l3g3200d_update_bits(YAS_L3G3200D_POWER, 0);
	}
	YLOGI(("%s(): [OUT]\n",__func__));

	return YAS_NO_ERROR;
}

/* -------------------------------------------------------------------------- */
static int yas_init(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_init();
	yas_l3g3200d_unlock();

	return err;
}

static int yas_term(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_term();
	yas_l3g3200d_unlock();

	return err;
}

static int yas_get_delay(void)
{
	int ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	ret = yas_l3g3200d_get_delay();
	yas_l3g3200d_unlock();

	return ret;
}

static int yas_set_delay(int delay)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (delay < YAS_L3G3200D_MIN_DELAY_INT ||
	    delay > YAS_L3G3200D_MAX_DELAY)
		return YAS_ERROR_ARG;

	if (delay == YAS_L3G3200D_MIN_DELAY_INT)
		delay = 1;

	if (p->interrupt == 0) {
		if (delay < YAS_L3G3200D_MIN_DELAY_POL)
			delay = YAS_L3G3200D_MIN_DELAY_POL;
	}

	yas_l3g3200d_lock();
	err = yas_l3g3200d_set_delay(delay);
	yas_l3g3200d_unlock();

	return err;
}

static int yas_get_enable(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_get_enable();
	yas_l3g3200d_unlock();

	return err;
}

static int yas_set_enable(int enable)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (enable != 0)
		enable = 1;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_set_enable(enable);
	yas_l3g3200d_unlock();

	return err;
}

static int yas_get_position(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_get_position();
	yas_l3g3200d_unlock();

		return err;
}

static int yas_set_position(int position)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (position < YAS_L3G3200D_MIN_POSITION ||
	    position > YAS_L3G3200D_MAX_POSITION)
		return YAS_ERROR_ARG;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_set_position(position);
	yas_l3g3200d_unlock();

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

	yas_l3g3200d_lock();
	err = yas_l3g3200d_get_offset(offset);
	yas_l3g3200d_unlock();

	return err;
}

static int yas_set_offset(struct yas_vector *offset)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (offset == NULL)
		return YAS_ERROR_ARG;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_set_offset(offset);
	yas_l3g3200d_unlock();

	return err;
}

static int yas_get_filter(struct yas_gyro_filter *filter)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (filter == NULL)
		return YAS_ERROR_ARG;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_get_filter(filter);
	yas_l3g3200d_unlock();

	return err;
}

static int yas_set_filter(struct yas_gyro_filter *filter)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (filter == NULL || filter->threshold < 0 ||
	    filter->threshold > YAS_L3G3200D_MAX_DATA)
		return YAS_ERROR_ARG;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_set_filter(filter);
	yas_l3g3200d_unlock();

	return err;
}

static int yas_get_filter_enable(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_get_filter_enable();
	yas_l3g3200d_unlock();

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

	yas_l3g3200d_lock();
	err = yas_l3g3200d_set_filter_enable(enable);
	yas_l3g3200d_unlock();

	return err;
}

static int yas_get_interrupt(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_get_interrupt();
	yas_l3g3200d_unlock();

	return err;
}

static int yas_set_interrupt(int interrupt)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (interrupt != 0)
		interrupt = 1;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_set_interrupt(interrupt);
	yas_l3g3200d_unlock();

	return err;
}

static void yas_interrupt_handler(void)
{
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check initialize */
	if (p->initialize == 0)
		return;

	yas_l3g3200d_interrupt_disable();
	yas_l3g3200d_interrupt_notify(p->wtm);
}

static int yas_measure(struct yas_gyro_data *data, int num)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (data == NULL || num <= 0)
		return YAS_ERROR_ARG;

	yas_l3g3200d_lock();
	err = yas_l3g3200d_measure(data, num);
	yas_l3g3200d_unlock();

	return err;
}

#if DEBUG
static int yas_get_register(uint8_t adr, uint8_t *val)
{
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Check initialize */
	if (l3g3200d_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	*val = yas_l3g3200d_read_reg_byte(adr);

	return YAS_NO_ERROR;
}

static int yas_set_register(uint8_t adr, uint8_t val)
{
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Check initialize */
	if (l3g3200d_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_write_reg_byte(adr, val);

	return YAS_NO_ERROR;
}
#endif

static int32_t kc_set_enable_cnt(int enable)
{
	int ret = YAS_NO_ERROR;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	kc_l3g3200d_set_enable_cnt(enable);
	yas_l3g3200d_unlock();

	return ret;
}
static int32_t kc_get_fs(void)
{
	int32_t ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	ret = kc_l3g3200d_get_fs();
	yas_l3g3200d_unlock();

	return ret;
}
static int32_t kc_set_fs(int32_t fs)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = kc_l3g3200d_set_fs(fs);
	yas_l3g3200d_unlock();

	return err;
}

static int32_t kc_get_calbretion_mode(void)
{
	return 0;
}
static int32_t kc_set_calbretion_mode(int32_t mode)
{
	return 0;
}
static int32_t kc_get_cal_chk_mode(void)
{
	return 0;
}
static int32_t kc_set_cal_chk_mode(int mode)
{
	return 0;
}

static int32_t kc_get_cal_sample_num(void)
{
	return 0;
}
static int32_t kc_set_cal_sample_num(int32_t num)
{
	return 0;
}
static int32_t kc_get_cal_acc_sample_num(void)
{
	return 0;
}
static int32_t kc_set_cal_acc_sample_num(int32_t num)
{
	return 0;
}
static int32_t kc_get_acc_calbretion(void)
{
	return 0;
}
static int32_t kc_set_acc_calbretion(int32_t acc_calbretion)
{
	return 0;
}
static int32_t kc_get_acc_cal_thershold(void)
{
	return 0;
}
static int32_t kc_set_acc_cal_thershold(int32_t acc_thershold)
{
	return 0;
}
static int32_t kc_get_gyro_cal_thershold(void)
{
	return 0;
}
static int32_t kc_set_gyro_cal_thershold(int32_t gyro_thershold)
{
	return 0;
}

static int32_t kc_get_iir_filter(void)
{
	int32_t ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	ret = kc_l3g3200d_get_iir_filter();
	yas_l3g3200d_unlock();

	return ret;
}

static int32_t kc_set_iir_filter(int32_t iir)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = kc_l3g3200d_set_iir_filter(iir);
	yas_l3g3200d_unlock();

	return err;
}

static int32_t kc_iir_filter(struct yas_gyro_data *data)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = kc_l3g3200d_iir_filter(data);
	yas_l3g3200d_unlock();

	return err;
}

static double kc_get_filter_b0(void)
{
	double ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	ret = kc_l3g3200d_get_filter_b0();
	yas_l3g3200d_unlock();

	return ret;
}
static int32_t kc_set_filter_b0( double filter)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = kc_l3g3200d_set_filter_b0(filter);
	yas_l3g3200d_unlock();

	return err;
}
static double kc_get_filter_b1(void)
{
	double ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	ret = kc_l3g3200d_get_filter_b1();
	yas_l3g3200d_unlock();

	return ret;
}
static int32_t kc_set_filter_b1( double filter)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = kc_l3g3200d_set_filter_b1(filter);
	yas_l3g3200d_unlock();

	return err;
}
static double kc_get_filter_b2(void)
{
	double ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	ret = kc_l3g3200d_get_filter_b2();
	yas_l3g3200d_unlock();

	return ret;
}
static int32_t kc_set_filter_b2(double filter)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = kc_l3g3200d_set_filter_b2(filter);
	yas_l3g3200d_unlock();

	return err;
}
static double kc_get_filter_a1(void)
{
	double ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	ret = kc_l3g3200d_get_filter_a1();
	yas_l3g3200d_unlock();

	return ret;
}
static int32_t kc_set_filter_a1(double filter)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = kc_l3g3200d_set_filter_a1(filter);
	yas_l3g3200d_unlock();

	return err;
}
static double kc_get_filter_a2(void)
{
	double ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	ret = kc_l3g3200d_get_filter_a2();
	yas_l3g3200d_unlock();

	return ret;
}
static int32_t kc_set_filter_a2(double filter)
{
	int32_t err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_l3g3200d_lock();
	err = kc_l3g3200d_set_filter_a2(filter);
	yas_l3g3200d_unlock();

	return err;
}

static int32_t kc_calibration(struct yas_gyro_data *data)
{
	return 0;
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

	yas_l3g3200d_lock();
	err = kc_l3g3200d_set_reset(reset);
	yas_l3g3200d_unlock();

	YLOGI(("%s(): end err=%d\n",__func__,err));
	return err;
}

/* -------------------------------------------------------------------------- */
/*  Global function                                                           */
/* -------------------------------------------------------------------------- */
int yas_gyro_driver_init(struct yas_gyro_driver *f, int interrupt)
{
	struct yas_gyro_driver_callback *cbk;
	struct yas_l3g3200d_data *p = &l3g3200d_data;

	/* Check parameter */
	if (f == NULL)
		return YAS_ERROR_ARG;

	/* Clear driver interface */
	f->init = NULL;
	f->term = NULL;
	f->get_delay = NULL;
	f->set_delay = NULL;
	f->get_enable = NULL;
	f->set_enable = NULL;
	f->get_position = NULL;
	f->set_position = NULL;
	f->get_offset = NULL;
	f->set_offset = NULL;
	f->get_filter = NULL;
	f->set_filter = NULL;
	f->get_filter_enable = NULL;
	f->set_filter_enable = NULL;
	f->get_interrupt = NULL;
	f->set_interrupt = NULL;
	f->measure = NULL;
#if DEBUG
	f->get_register = NULL;
	f->set_register = NULL;
#endif
	f->get_gyro_fs = NULL;
	f->set_gyro_fs = NULL;
	f->get_calbretion_mode = NULL;
	f->set_calbretion_mode = NULL;
	f->get_cal_sample_num = NULL;
	f->set_cal_sample_num = NULL;
	f->get_acc_calbretion = NULL;
	f->set_acc_calbretion = NULL;
	f->get_acc_cal_thershold = NULL;
	f->set_acc_cal_thershold = NULL;
	f->get_gyro_cal_thershold = NULL;
	f->set_gyro_cal_thershold = NULL;
	f->get_iir_filter = NULL;
	f->set_iir_filter = NULL;
	f->iir_filter = NULL;
	f->get_filter_b0 = NULL;
	f->set_filter_b0 = NULL;
	f->get_filter_b1 = NULL;
	f->set_filter_b1 = NULL;
	f->get_filter_b2 = NULL;
	f->set_filter_b2 = NULL;
	f->get_filter_a1 = NULL;
	f->set_filter_a1 = NULL;
	f->get_filter_a2 = NULL;
	f->set_filter_a2 = NULL;
	f->calibration = NULL;
	f->interrupt_handler = NULL;

	cbk = &f->callback;
	if (cbk->device_open == NULL ||
	    cbk->device_close == NULL ||
	    cbk->device_write == NULL ||
	    cbk->device_read == NULL ||
	    cbk->msleep == NULL ||
	    cbk->device_set_reset == NULL||
	    cbk->device_set_enable == NULL) {
		return YAS_ERROR_ARG;
	}
	if (interrupt) {
		interrupt = 1;
		if (cbk->interrupt_enable == NULL ||
		    cbk->interrupt_disable == NULL ||
		    cbk->interrupt_notify == NULL) {
			return YAS_ERROR_ARG;
		}
	}

	/* Clear intialize */
	yas_l3g3200d_term();

	/* Set callback interface */
	cb.callback = *cbk;

	/* Set driver interface */
	f->init = yas_init;
	f->term = yas_term;
	f->get_delay = yas_get_delay;
	f->set_delay = yas_set_delay;
	f->get_enable = yas_get_enable;
	f->set_enable = yas_set_enable;
	f->get_position = yas_get_position;
	f->set_position = yas_set_position;
	f->get_offset = yas_get_offset;
	f->set_offset = yas_set_offset;
	f->get_filter = yas_get_filter;
	f->set_filter = yas_set_filter;
	f->get_filter_enable = yas_get_filter_enable;
	f->set_filter_enable = yas_set_filter_enable;
	f->measure = yas_measure;
#if DEBUG
	f->get_register = yas_get_register;
	f->set_register = yas_set_register;
#endif
	if (interrupt) {
		f->get_interrupt = yas_get_interrupt;
		f->set_interrupt = yas_set_interrupt;
		f->interrupt_handler = yas_interrupt_handler;
	}

	f->set_vsensor = kc_l3g3200d_set_vsensor;
	f->set_enable_cnt = kc_set_enable_cnt;
	f->get_gyro_fs = kc_get_fs;
	f->set_gyro_fs = kc_set_fs;
	f->get_calbretion_mode = kc_get_calbretion_mode;
	f->set_calbretion_mode = kc_set_calbretion_mode;
	f->get_cal_chk_mode = kc_get_cal_chk_mode;
	f->set_cal_chk_mode = kc_set_cal_chk_mode;
	f->get_cal_sample_num = kc_get_cal_sample_num;
	f->set_cal_sample_num = kc_set_cal_sample_num;
	f->get_cal_acc_sample_num = kc_get_cal_acc_sample_num;
	f->set_cal_acc_sample_num = kc_set_cal_acc_sample_num;
	f->get_acc_calbretion = kc_get_acc_calbretion;
	f->set_acc_calbretion = kc_set_acc_calbretion;
	f->get_acc_cal_thershold = kc_get_acc_cal_thershold;
	f->set_acc_cal_thershold = kc_set_acc_cal_thershold;
	f->get_gyro_cal_thershold = kc_get_gyro_cal_thershold;
	f->set_gyro_cal_thershold = kc_set_gyro_cal_thershold;
	f->get_iir_filter = kc_get_iir_filter;
	f->set_iir_filter = kc_set_iir_filter;
	f->iir_filter = kc_iir_filter;
	f->get_filter_b0 = kc_get_filter_b0;
	f->set_filter_b0 = kc_set_filter_b0;
	f->get_filter_b1 = kc_get_filter_b1;
	f->set_filter_b1 = kc_set_filter_b1;
	f->get_filter_b2 = kc_get_filter_b2;
	f->set_filter_b2 = kc_set_filter_b2;
	f->get_filter_a1 = kc_get_filter_a1;
	f->set_filter_a1 = kc_set_filter_a1;
	f->get_filter_a2 = kc_get_filter_a2;
	f->set_filter_a2 = kc_set_filter_a2;
	f->calibration = kc_calibration;
	f->set_reset = kc_set_reset;

	p->interrupt = interrupt;
	pcb = &cb;

	gyro_power_cb.power_on = kc_l3g3200d_power_on;
	gyro_power_cb.power_off = kc_l3g3200d_power_off;

	return YAS_NO_ERROR;
}

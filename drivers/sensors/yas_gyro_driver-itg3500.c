/*
 * This software is contributed or developed by KYOCERA Corporation.
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

/* Select int pin */
#define YAS_ITG3500_INT_LEVEL_ACTIVE_H                                    (0x00)
#define YAS_ITG3500_INT_LEVEL_ACTIVE_L                                    (0x80)
#define YAS_ITG3500_INT_LEVEL                   (YAS_ITG3500_INT_LEVEL_ACTIVE_H)

#define YAS_ITG3500_INT_OPEN_PUSHPULL                                     (0x00)
#define YAS_ITG3500_INT_OPEN_OPENDRAIN                                    (0x40)
#define YAS_ITG3500_INT_OPEN                    (YAS_ITG3500_INT_OPEN_OPENDRAIN)

#define YAS_ITG3500_INT_EN_50US                                           (0x00)
#define YAS_ITG3500_INT_EN_HOLD                                           (0x20)
#define YAS_ITG3500_INT_EN                             (YAS_ITG3500_INT_EN_HOLD)

#define YAS_ITG3500_BOOT_TIME                                              (100)
#define YAS_ITG3500_DELAY_TIME                                             (100)
#define YAS_ITG3500_STARTUP_TIME                                            (50)
#define KC_ITG3500_SLEEPOUT_TIME                                            (60)

/* Default parameters */
#define YAS_ITG3500_DEFAULT_DELAY                                           (10)
#define YAS_ITG3500_DEFAULT_POSITION       (CONFIG_INPUT_YAS_GYROSCOPE_POSITION)

#define YAS_ITG3500_MAX_DELAY                                              (200)
#define YAS_ITG3500_MIN_DELAY_INT                                            (0)
#define YAS_ITG3500_MIN_DELAY_POL                                           (10)
#define YAS_ITG3500_MAX_POSITION                                             (7)
#define YAS_ITG3500_MIN_POSITION                                             (0)
#define YAS_ITG3500_MAX_DATA                                           (2000000)
#define YAS_ITG3500_MIN_DATA                                          (-2000000)

/* Register */
#define YAS_ITG3500_SMPLRT_DIV_REG                                        (0x19)
#define YAS_ITG3500_DLPF_REG                                              (0x1A)
#define YAS_ITG3500_DLPF_MASK                                             (0x07)
#define YAS_ITG3500_DLPF_SHIFT                                               (0)
#define YAS_ITG3500_DLPF_250HZ                                               (0)
#define YAS_ITG3500_DLPF_184HZ                                               (1)
#define YAS_ITG3500_DLPF_92HZ                                                (2)
#define YAS_ITG3500_DLPF_41HZ                                                (3)
#define YAS_ITG3500_DLPF_20HZ                                                (4)
#define YAS_ITG3500_DLPF_10HZ                                                (5)
#define YAS_ITG3500_DLPF_5HZ                                                 (6)

#define YAS_ITG3500_FS_REG                                                (0x1B)
#define YAS_ITG3500_FS_MASK                                               (0x18)
#define YAS_ITG3500_FS_SHIFT                                                 (3)
#define YAS_ITG3500_FS_2000                                                  (3)
#define YAS_ITG3500_FS_1000                                                  (2)
#define YAS_ITG3500_FS_500                                                   (1)
#define YAS_ITG3500_FS_250                                                   (0)

#define YAS_ITG3500_FIFO_CFG_REG                                          (0x23)
#define YAS_ITG3500_FIFO_CFG                                              (0x70)

#define YAS_ITG3500_INT_CFG_REG                                           (0x37)
#define YAS_ITG3500_INT_CFG                            (YAS_ITG3500_INT_LEVEL |\
							YAS_ITG3500_INT_OPEN  |\
							 YAS_ITG3500_INT_EN)

#define YAS_ITG3500_INT_ENABLE_REG                                        (0x38)
#define YAS_ITG3500_INT_ENABLE                                            (0x01)
#define YAS_ITG3500_INT_DISABLE                                           (0x00)

#define YAS_ITG3500_INT_STATUS_REG                                        (0x3A)
#define YAS_ITG3500_INT_STATUS_RDY                                        (0x01)

#define YAS_ITG3500_DATA_REG                                              (0x43)
#define YAS_ITG3500_RESOLUTION                                              (61)

#define YAS_ITG3500_FIFO_RESET_REG                                        (0x6A)
#define YAS_ITG3500_FIFO_RESET                                            (0x44)

#define YAS_ITG3500_RESET_REG                                             (0x6B)
#define YAS_ITG3500_RESET                                                 (0x80)

#define YAS_ITG3500_POWER_REG                                             (0x6B)
#define YAS_ITG3500_POWER_ON                                              (0x09)
#define YAS_ITG3500_POWER_OFF                                             (0x48)

#define YAS_ITG3500_FIFO_COUNT_REG                                        (0x72)

#define YAS_ITG3500_FIFO_DATA_REG                                         (0x74)

#define YAS_ITG3500_WHOAMI_REG                                            (0x75)
#define YAS_ITG3500_WHOAMI_MASK                                           (0x7e)
#define YAS_ITG3500_I_AM_ITG3500                                          (0x68)

#define KC_ITG3500_19_REG                                                 (0x19)
#define KC_ITG3500_1A_REG                                                 (0x1A)
#define KC_ITG3500_1B_REG                                                 (0x1B)
#define KC_ITG3500_6B_REG                                                 (0x6B)

#define KC_ITG3500_19_CFG                                                 (0x09)
#define KC_ITG3500_1A_CFG                                                 (0x01)
#define KC_ITG3500_1B_CFG                                                 (0x18)
#define KC_ITG3500_6B_SLEEP_IN                                            (0x41)
#define KC_ITG3500_6B_SLEEP_OUT                                           (0x01)

/* -------------------------------------------------------------------------- */
/*  Structure definition                                                      */
/* -------------------------------------------------------------------------- */
/* Digital Low Pass Filetr */
struct yas_itg3500_dlpf {
	int delay;                    /* min delay (msec) in the range of ODR */
	unsigned char dlpf;           /* output data rate register value      */
};

/* Driver private data */
struct yas_itg3500_data {
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
	int fifo_count;

	struct yas_gyro_data last; /* mdps */
	struct yas_vector offset;
};

/* -------------------------------------------------------------------------- */
/*  Data                                                                      */
/* -------------------------------------------------------------------------- */
/* Control block */
static struct yas_gyro_driver   cb;
static struct yas_gyro_driver  *pcb;
static struct yas_itg3500_data  itg3500_data;
static unsigned char            itg3500_buf[YAS_GYRO_FIFO_MAX * 6];

static struct sensor_power_callback gyro_power_cb;
atomic_t at_device_reset = ATOMIC_INIT(0);

/* Digital Low Pass Filetr */
static const struct yas_itg3500_dlpf yas_itg3500_dlpf_tbl[] = {
	{4,   YAS_ITG3500_DLPF_250HZ},
	{6,   YAS_ITG3500_DLPF_184HZ},
	{12,  YAS_ITG3500_DLPF_92HZ},
	{25,  YAS_ITG3500_DLPF_41HZ},
	{50,  YAS_ITG3500_DLPF_20HZ},
	{100, YAS_ITG3500_DLPF_10HZ},
	{200, YAS_ITG3500_DLPF_5HZ},
};

/* Transformation matrix for chip mounting position (0) */
static const int yas_itg3500_position_map[][3][3] = {
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
static int  yas_itg3500_read_reg(unsigned char, unsigned char *, int);
static int  yas_itg3500_write_reg(unsigned char, unsigned char *, int);
static unsigned char yas_itg3500_read_reg_byte(unsigned char);
static int yas_itg3500_write_reg_byte(unsigned char, unsigned char);
static void yas_itg3500_init_offset(void);
static void yas_itg3500_init_lastvalue(void);
static void yas_itg3500_init_gyro_data(struct yas_gyro_data *);
static void yas_itg3500_init_data(void);
static int  yas_itg3500_ischg_enable(int);
static unsigned char yas_itg3500_smplrt_div(int, unsigned char);
/* static void yas_itg3500_reset_device(void);*/
static void yas_itg3500_power_on(void);
static void yas_itg3500_power_off(void);
static void yas_itg3500_reset_fifo(void);
static int  yas_itg3500_measure_fifo(struct yas_gyro_data *, int);
static int  yas_itg3500_measure_direct(struct yas_gyro_data *);

static int  yas_itg3500_lock(void);
static int  yas_itg3500_unlock(void);
static int  yas_itg3500_device_open(void);
static int  yas_itg3500_device_close(void);
static int  yas_itg3500_interrupt_enable(void);
static int  yas_itg3500_interrupt_disable(void);
static void yas_itg3500_interrupt_notify(int);
static int  yas_itg3500_msleep(int);

static int  yas_itg3500_init(void);
static int  yas_itg3500_term(void);
static int  yas_itg3500_get_delay(void);
static int  yas_itg3500_set_delay(int);
static int  yas_itg3500_get_enable(void);
static int  yas_itg3500_set_enable(int);
static int  yas_itg3500_get_position(void);
static int  yas_itg3500_set_position(int);
static int  yas_itg3500_get_offset(struct yas_vector *);
static int  yas_itg3500_set_offset(struct yas_vector *);
static int  yas_itg3500_get_filter(struct yas_gyro_filter *);
static int  yas_itg3500_set_filter(struct yas_gyro_filter *);
static int  yas_itg3500_get_filter_enable(void);
static int  yas_itg3500_set_filter_enable(int);
static int  yas_itg3500_get_interrupt(void);
static int  yas_itg3500_set_interrupt(int);
static int  yas_itg3500_measure(struct yas_gyro_data *, int);
#if DEBUG
static int  yas_itg3500_get_register(unsigned char, unsigned char *);
static int  yas_itg3500_set_register(unsigned char, unsigned char);
#endif /* DEBUG */
static int yas_itg3500_reset_init(void);
static int yas_itg3500_reset_term(void);

/* -------------------------------------------------------------------------- */
/*  Local functions                                                           */
/* -------------------------------------------------------------------------- */
static void kc_itg3500_power_on(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	YLOGI(("%s(): [IN]\n",__func__));

	yas_itg3500_reset_init();
	cbk->device_set_reset(1);

	YLOGI(("%s(): [OUT]\n",__func__));
}
static void kc_itg3500_power_off(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	YLOGI(("%s(): [IN]\n",__func__));

	cbk->device_set_reset(0);
	yas_itg3500_reset_term();

	YLOGI(("%s(): [OUT]\n",__func__));
}

static void kc_itg3500_set_vsensor(int32_t enable)
{
	if (enable) {
		sensor_power_reg_cbfunc(&gyro_power_cb);
	}
	else {
		sensor_power_unreg_cbfunc(&gyro_power_cb);
	}
}

static int32_t kc_itg3500_reset( void )
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
static int yas_itg3500_read_reg(unsigned char adr, unsigned char *buf, int len)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_itg3500_data *p = &itg3500_data;
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

static int yas_itg3500_write_reg(unsigned char adr, unsigned char *buf, int len)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_itg3500_data *p = &itg3500_data;
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

static unsigned char yas_itg3500_read_reg_byte(unsigned char adr)
{
	unsigned char buf;
	int err;

	err = yas_itg3500_read_reg(adr, &buf, 1);
	if (err == 0)
		return buf;

	return 0;
}

static int yas_itg3500_write_reg_byte(unsigned char adr, unsigned char val)
{
	return yas_itg3500_write_reg(adr, &val, 1);
}

#define yas_itg3500_read_bits(r) \
	((yas_itg3500_read_reg_byte(r##_REG) & r##_MASK) >> r##_SHIFT)

#define yas_itg3500_update_bits(r, v) \
	yas_itg3500_write_reg_byte(r##_REG, \
				   ((yas_itg3500_read_reg_byte(r##_REG) & \
				     ~r##_MASK) | ((v) << r##_SHIFT)))

static void yas_itg3500_init_offset(void)
{
	struct yas_itg3500_data *p = &itg3500_data;
	int i;

	for (i = 0; i < 3; ++i)
		p->offset.v[i] = 0;
}

static void yas_itg3500_init_lastvalue(void)
{
	struct yas_itg3500_data *p = &itg3500_data;
	int i;

	for (i = 0; i < 3; ++i) {
		p->last.xyz.v[i] = 0;
		p->last.raw.v[i] = 0;
	}
}

static void yas_itg3500_init_gyro_data(struct yas_gyro_data *data)
{
	int i;

	for (i = 0; i < 3; ++i) {
		data->xyz.v[i] = 0;
		data->raw.v[i] = 0;
	}

	data->num = 0;
}

static void yas_itg3500_reset_init_data(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	p->initialize = 0;
	p->interrupt = 0;
	p->overrun = 0;
}

static void yas_itg3500_init_data(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	p->initialize = 0;
	p->device_open = 0;
	p->enable = 0;
	p->delay = YAS_ITG3500_DEFAULT_DELAY;
	p->position = YAS_ITG3500_DEFAULT_POSITION;
	p->threshold = YAS_GYRO_DEFAULT_FILTER_THRESH;
	p->filter_enable = 0;
	p->interrupt = 0;
	p->overrun = 0;
	p->fifo_count = 0;
	yas_itg3500_init_offset();
	yas_itg3500_init_lastvalue();
}

static int yas_itg3500_ischg_enable(int enable)
{
	struct yas_itg3500_data *p = &itg3500_data;

	if (p->enable == enable)
		return 0;

	return 1;
}

static unsigned char yas_itg3500_smplrt_div(int delay, unsigned char dlpf)
{
	if (delay <= 0)
		return 0;

	if (dlpf == YAS_ITG3500_DLPF_184HZ ||
	    dlpf == YAS_ITG3500_DLPF_92HZ ||
	    dlpf == YAS_ITG3500_DLPF_41HZ ||
	    dlpf == YAS_ITG3500_DLPF_20HZ ||
	    dlpf == YAS_ITG3500_DLPF_10HZ ||
	    dlpf == YAS_ITG3500_DLPF_5HZ)
		return (unsigned char)(delay - 1);

	return (unsigned char)(8 * delay - 1);
}

#if 0
static void yas_itg3500_reset_device(void)
{
	unsigned char reg;

	/* Reset device */
	yas_itg3500_write_reg_byte(YAS_ITG3500_RESET_REG, YAS_ITG3500_RESET);
	yas_itg3500_msleep(YAS_ITG3500_STARTUP_TIME);
	while (1) {
		reg = yas_itg3500_read_reg_byte(YAS_ITG3500_RESET_REG);
		if ((reg & YAS_ITG3500_RESET) == 0)
			break;
	}
}
#endif

static void yas_itg3500_power_on(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Reset device (all register default) */
	/* yas_itg3500_reset_device(); */

	yas_itg3500_write_reg_byte(KC_ITG3500_19_REG, KC_ITG3500_19_CFG);
	yas_itg3500_write_reg_byte(KC_ITG3500_1A_REG, KC_ITG3500_1A_CFG);
	yas_itg3500_write_reg_byte(KC_ITG3500_1B_REG, KC_ITG3500_1B_CFG);

	/* Set sleep bit */
	/* yas_itg3500_write_reg_byte(YAS_ITG3500_POWER_REG, YAS_ITG3500_POWER_ON); */
	yas_itg3500_write_reg_byte(KC_ITG3500_6B_REG, KC_ITG3500_6B_SLEEP_OUT);
	yas_itg3500_msleep(KC_ITG3500_SLEEPOUT_TIME);

	/* Set range full scale */
	/* yas_itg3500_update_bits(YAS_ITG3500_FS, YAS_ITG3500_FS_2000); */

	if (p->interrupt) {
		/* Set fifo config */
		yas_itg3500_write_reg_byte(YAS_ITG3500_FIFO_CFG_REG,
					   YAS_ITG3500_FIFO_CFG);

		/* Reset fifo */
		yas_itg3500_reset_fifo();

		/* Set interrupt config */
		yas_itg3500_write_reg_byte(YAS_ITG3500_INT_CFG_REG,
					   YAS_ITG3500_INT_CFG);

		/* Enable interrupt */
		yas_itg3500_write_reg_byte(YAS_ITG3500_INT_ENABLE_REG,
					   YAS_ITG3500_INT_ENABLE);
	}
}

static void yas_itg3500_power_off(void)
{
	/* Clear sleep bit */
	/* yas_itg3500_write_reg_byte(YAS_ITG3500_POWER_REG, */
	/*			   YAS_ITG3500_POWER_OFF); */
	yas_itg3500_write_reg_byte(KC_ITG3500_6B_REG, KC_ITG3500_6B_SLEEP_IN);

}

static void yas_itg3500_reset_fifo(void)
{
	struct yas_itg3500_data *p = &itg3500_data;
	unsigned char reg;

	yas_itg3500_write_reg_byte(YAS_ITG3500_FIFO_RESET_REG,
				   YAS_ITG3500_FIFO_RESET);
	while (1) {
		reg = yas_itg3500_read_reg_byte(YAS_ITG3500_FIFO_RESET_REG);
		if ((reg & YAS_ITG3500_FIFO_RESET) == 0)
			break;
	}

	p->fifo_count = 0;
}

static int yas_itg3500_data_filter(struct yas_gyro_data *out_data)
{
	struct yas_itg3500_data *p = &itg3500_data;
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

static int yas_itg3500_measure_fifo(struct yas_gyro_data *out_data, int num)
{
	struct yas_itg3500_data *p = &itg3500_data;
	int raw[3];
	int data_num;
	int pos = p->position;
	int i, j, k;

	data_num = 0;

	/* Check devie */
	if (p->device_open == 0) {
		out_data[0] = p->last;
		out_data[0].num = 1;
			out_data[0].overrun = 0;
		yas_itg3500_interrupt_enable();
		return 1;
	}

	/* Read fifo count (byte) */
	if (yas_itg3500_read_reg(YAS_ITG3500_FIFO_COUNT_REG | 0x08,
				 &itg3500_buf[0], 2 != 0)) {
		out_data[0] = p->last;
		out_data[0].num = 1;
		out_data[0].overrun = 0;
		yas_itg3500_reset_fifo();
		yas_itg3500_interrupt_enable();
		return 1;
	}

	/* Check FIFO stored data level */
	data_num = ((short) ((itg3500_buf[0] & 0x03) << 8) | itg3500_buf[1]);
	data_num /= 6;
	if (YAS_GYRO_FIFO_MAX < data_num)
		data_num = YAS_GYRO_FIFO_MAX;
	if (num < data_num)
		data_num = num;

	/* Read angle rate data */
	if (yas_itg3500_read_reg(YAS_ITG3500_FIFO_DATA_REG | 0x80,
				 &itg3500_buf[0], data_num * 6) != 0) {
		out_data[0] = p->last;
		out_data[0].num = 1;
		out_data[0].overrun = 0;
		yas_itg3500_reset_fifo();
		yas_itg3500_interrupt_enable();
		return 1;
	}

	/* yas_itg3500_reset_fifo(); ToDo */

	/* Clear interrupt status */
	yas_itg3500_read_reg_byte(YAS_ITG3500_INT_STATUS_REG);

	for (i = 0; i < data_num; ++i) {
		for (j = 0; j < 3; ++j)
			raw[j] = ((short) (itg3500_buf[i*6+j*2] << 8) |
				  itg3500_buf[i*6+j*2+1]);

		for (j = 0; j < 3; ++j) {
			/* coordinate transformation */
			out_data[i].raw.v[j] = 0;
			for (k = 0; k < 3; ++k)
				out_data[i].raw.v[j] += raw[k] *
					yas_itg3500_position_map[pos][j][k];
			out_data[i].xyz.v[j] =
				out_data[i].raw.v[j] * YAS_ITG3500_RESOLUTION;
		}
		yas_itg3500_data_filter(&out_data[i]);
		p->last = out_data[i];
		out_data[i].xyz.v[0] -= p->offset.v[0];
		out_data[i].xyz.v[1] -= p->offset.v[1];
		out_data[i].xyz.v[2] -= p->offset.v[2];
		out_data[i].num = data_num;
		out_data[i].overrun = 0;
	}

	/* Enable interrupt */
	yas_itg3500_interrupt_enable();

	return data_num;
}

static int yas_itg3500_measure_direct(struct yas_gyro_data *out_data)
{
	struct yas_itg3500_data *p = &itg3500_data;
	int raw[3];
	int pos = p->position;
	int i, j;

	/* Check device */
	if (p->device_open == 0) {
		out_data[0] = p->last;
		out_data[0].num = 1;
		out_data[0].overrun = 0;
		return 1;
	}

	/* Read angle rate data */
	if (yas_itg3500_read_reg(YAS_ITG3500_DATA_REG | 0x80,
				 &itg3500_buf[0], 6) != 0) {
		out_data[0] = p->last;
		out_data[0].num = 1;
		out_data[0].overrun = 0;
		return 1;
	}

	for (i = 0; i < 3; ++i)
		raw[i] = ((short)((itg3500_buf[i*2] << 8)) |
			  itg3500_buf[i*2+1]);

	for (i = 0; i < 3; ++i) {
		/* coordinate transformation */
		out_data->raw.v[i] = 0;
		for (j = 0; j < 3; ++j)
			out_data->raw.v[i] += raw[j] *
				yas_itg3500_position_map[pos][i][j];
		out_data->xyz.v[i] =
			out_data->raw.v[i] * YAS_ITG3500_RESOLUTION;
	}
	yas_itg3500_data_filter(out_data);
	p->last = out_data[0];
	out_data->xyz.v[0] -= p->offset.v[0];
	out_data->xyz.v[1] -= p->offset.v[1];
	out_data->xyz.v[2] -= p->offset.v[2];
	out_data->num = 1;
	out_data->overrun = 0;

	return 1;
}

static int yas_itg3500_lock(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	int err;

	if (cbk->lock != NULL && cbk->unlock != NULL)
		err = cbk->lock();
	else
		err = YAS_NO_ERROR;

	return err;
}

static int yas_itg3500_unlock(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	int err;

	if (cbk->lock != NULL && cbk->unlock != NULL)
		err = cbk->unlock();
	else
		err = YAS_NO_ERROR;

	kc_itg3500_reset();

	return err;
}

static int yas_itg3500_device_open(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_itg3500_data *p = &itg3500_data;
	int err;

	if (p->device_open == 0) {
		kc_itg3500_set_vsensor(1);
		err = cbk->device_open();
		if (err != YAS_NO_ERROR)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		p->device_open = 1;
	}

	return YAS_NO_ERROR;
}

static int yas_itg3500_device_close(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_itg3500_data *p = &itg3500_data;
	int err;

	if (p->device_open != 0) {
		err = cbk->device_close();
		if (err != YAS_NO_ERROR)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		p->device_open = 0;
		kc_itg3500_set_vsensor(0);
	}

	return YAS_NO_ERROR;
}

static int yas_itg3500_interrupt_enable(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_itg3500_data *p = &itg3500_data;
	int err;

	if (p->interrupt == 0)
		return YAS_NO_ERROR;

	err = cbk->interrupt_enable();
	if (err != YAS_NO_ERROR)
		return YAS_ERROR_INTERRUPT;

	return YAS_NO_ERROR;
}

static int yas_itg3500_interrupt_disable(void)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_itg3500_data *p = &itg3500_data;
	int err;

	if (p->interrupt == 0)
		return YAS_NO_ERROR;

	err = cbk->interrupt_disable();
	if (err != YAS_NO_ERROR)
		return YAS_ERROR_INTERRUPT;

	return YAS_NO_ERROR;
}

static void yas_itg3500_interrupt_notify(int num)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;
	struct yas_itg3500_data *p = &itg3500_data;

	if (p->interrupt == 0)
		return;

	cbk->interrupt_notify(num);
}

static int yas_itg3500_msleep(int msec)
{
	struct yas_gyro_driver_callback *cbk = &pcb->callback;

	if (msec <= 0)
		return YAS_ERROR_ARG;

	cbk->msleep(msec);

	return YAS_NO_ERROR;
}

static int yas_itg3500_reset_init(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	if (p->initialize == 1)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_reset_init_data();

	yas_itg3500_write_reg_byte(YAS_ITG3500_RESET_REG, YAS_ITG3500_RESET);
	msleep(YAS_ITG3500_STARTUP_TIME);

	yas_itg3500_write_reg_byte(KC_ITG3500_19_REG, KC_ITG3500_19_CFG);
	yas_itg3500_write_reg_byte(KC_ITG3500_1A_REG, KC_ITG3500_1A_CFG);
	yas_itg3500_write_reg_byte(KC_ITG3500_1B_REG, KC_ITG3500_1B_CFG);
	yas_itg3500_write_reg_byte(KC_ITG3500_6B_REG, KC_ITG3500_6B_SLEEP_IN);

	p->initialize = 1;

	return YAS_NO_ERROR;
}

static int yas_itg3500_reset_term(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Close i2c/spi */
	p->initialize = 0;

	return YAS_NO_ERROR;
}

static int yas_itg3500_init(void)
{
	struct yas_itg3500_data *p = &itg3500_data;
	unsigned char id;
	int err;

	/* Check intialize */
	if (p->initialize == 1)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Wait device boot time */
	/* yas_itg3500_msleep(YAS_ITG3500_BOOT_TIME); */

	/* Init data */
	yas_itg3500_init_data();

	/* Open i2c/spi */
	err = yas_itg3500_device_open();
	if (err != YAS_NO_ERROR)
		return err;

	/* Check id */
	id = yas_itg3500_read_reg_byte(YAS_ITG3500_WHOAMI_REG);
	if ((id & YAS_ITG3500_WHOAMI_MASK) != YAS_ITG3500_I_AM_ITG3500) {
		yas_itg3500_device_close();
		return YAS_ERROR_CHIP_ID;
	}

	/* Reset device (all register default) */
	/* yas_itg3500_reset_device(); */
	yas_itg3500_write_reg_byte(YAS_ITG3500_RESET_REG, YAS_ITG3500_RESET);
	msleep(YAS_ITG3500_STARTUP_TIME);

	/* Power off */
	/* yas_itg3500_power_off(); */

	yas_itg3500_write_reg_byte(KC_ITG3500_19_REG, KC_ITG3500_19_CFG);
	yas_itg3500_write_reg_byte(KC_ITG3500_1A_REG, KC_ITG3500_1A_CFG);
	yas_itg3500_write_reg_byte(KC_ITG3500_1B_REG, KC_ITG3500_1B_CFG);
	yas_itg3500_write_reg_byte(KC_ITG3500_6B_REG, KC_ITG3500_6B_SLEEP_IN);

	/* Close i2c */
	yas_itg3500_device_close();

	p->initialize = 1;

	return YAS_NO_ERROR;
}

static int yas_itg3500_term(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_set_enable(0);

	/* Close i2c/spi */
	yas_itg3500_device_close();

	p->initialize = 0;

	return YAS_NO_ERROR;
}

static int yas_itg3500_get_delay(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->delay;
}

static int yas_itg3500_set_delay(int delay)
{
	struct yas_itg3500_data *p = &itg3500_data;
	unsigned char smplrt_div;
	unsigned char dlpf;
	int i;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Determine optimum dlpf */
	for (i = 1; i < (int)(sizeof(yas_itg3500_dlpf_tbl) /
			      sizeof(struct yas_itg3500_dlpf)) &&
		     delay >= (int)yas_itg3500_dlpf_tbl[i].delay; i++)
		;

	dlpf = yas_itg3500_dlpf_tbl[i-1].dlpf;
	smplrt_div = yas_itg3500_smplrt_div(delay, dlpf);
	p->delay = delay;
	p->actual_delay = yas_itg3500_dlpf_tbl[i-1].delay;

	if (p->enable) {
		yas_itg3500_update_bits(YAS_ITG3500_DLPF, dlpf);
		yas_itg3500_write_reg_byte(YAS_ITG3500_SMPLRT_DIV_REG,
					   smplrt_div);
		if (p->interrupt)
			yas_itg3500_reset_fifo();
	}

	return YAS_NO_ERROR;
}

static int yas_itg3500_get_enable(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->enable;
}

static int yas_itg3500_set_enable(int enable)
{
	struct yas_itg3500_data *p = &itg3500_data;
	int err;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	if (enable) {
		if (yas_itg3500_ischg_enable(enable)) {
			/* Open transfer i2c/spi */
			err = yas_itg3500_device_open();
			if (err != YAS_NO_ERROR)
				return err;

			/* Init data */
			yas_itg3500_init_lastvalue();

			/* Power on */
			yas_itg3500_power_on();
			p->enable = 1;

			/* Set DLPF */
			yas_itg3500_set_delay(p->delay);
			yas_itg3500_msleep(p->delay * 2);

			/* Interrupt enable */
			if (p->interrupt)
				yas_itg3500_interrupt_enable();
		}
	} else {
		if (yas_itg3500_ischg_enable(enable)) {
			/* Interrupt disable */
			if (p->interrupt)
				yas_itg3500_interrupt_disable();

			/* Power off */
			yas_itg3500_power_off();

			/* Close transfer i2c/spi */
			err = yas_itg3500_device_close();
			if (err != YAS_NO_ERROR) {
				/* Power on */
				yas_itg3500_power_on();
				return err;
			}

			p->enable = 0;
		}
	}

	return YAS_NO_ERROR;
}

static int yas_itg3500_get_position(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->position;
}

static int yas_itg3500_set_position(int position)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	p->position = position;

	return YAS_NO_ERROR;
}

static int yas_itg3500_get_offset(struct yas_vector *offset)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	*offset = p->offset;

	return YAS_NO_ERROR;
}

static int yas_itg3500_set_offset(struct yas_vector *offset)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	p->offset = *offset;

	return YAS_NO_ERROR;
}

static int yas_itg3500_get_filter(struct yas_gyro_filter *filter)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	filter->threshold = p->threshold;

	return YAS_NO_ERROR;
}

static int yas_itg3500_set_filter(struct yas_gyro_filter *filter)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	p->threshold = filter->threshold;

	return YAS_NO_ERROR;
}

static int yas_itg3500_get_filter_enable(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->filter_enable;
}

static int yas_itg3500_set_filter_enable(int enable)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	p->filter_enable = enable;

	return YAS_NO_ERROR;
}

static int yas_itg3500_get_interrupt(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return p->interrupt;
}

static int yas_itg3500_set_interrupt(int interrupt)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	if (interrupt == 0)
		yas_itg3500_interrupt_disable();

	p->interrupt = interrupt;

	return YAS_NO_ERROR;
}

static int yas_itg3500_measure(struct yas_gyro_data *out_data, int num)
{
	struct yas_itg3500_data *p = &itg3500_data;
	int actual_num;
	int i;

	/* Check initialize */
	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	for (i = 0; i < num; i++)
		yas_itg3500_init_gyro_data(&out_data[i]);

	if (p->interrupt)
		actual_num = yas_itg3500_measure_fifo(out_data, num);
	else
		actual_num = yas_itg3500_measure_direct(out_data);

	return actual_num;
}

static void kc_itg3500_set_enable_cnt(int enable)
{
	struct yas_itg3500_data *p = &itg3500_data;
	p->enable = enable;
}

#if DEBUG
static int yas_itg3500_get_register(unsigned char adr, unsigned char *val)
{
	struct yas_itg3500_data *p = &itg3500_data;
	int open;

	open = p->device_open;

	if (open == 0)
		yas_itg3500_device_open();

	*val = yas_itg3500_read_reg_byte(adr);

	if (open == 0)
		yas_itg3500_device_close();

	return YAS_NO_ERROR;
}

static int yas_itg3500_set_register(unsigned char adr, unsigned char val)
{
	struct yas_itg3500_data *p = &itg3500_data;
	int open;

	open = p->device_open;

	if (open == 0)
		yas_itg3500_device_open();

	yas_itg3500_write_reg_byte(adr, val);

	if (open == 0)
		yas_itg3500_device_close();

	return YAS_NO_ERROR;
}
#endif /* DEBUG */

static int32_t kc_itg3500_set_reset(int32_t reset)
{
	struct yas_itg3500_data *p = &itg3500_data;

	if (p->initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	YLOGI(("%s(): [IN] reset=%d\n",__func__,reset));
	if (reset) {
		yas_itg3500_write_reg_byte(KC_ITG3500_6B_REG, KC_ITG3500_6B_SLEEP_OUT);
		yas_itg3500_msleep(KC_ITG3500_SLEEPOUT_TIME);
	} else {
		yas_itg3500_write_reg_byte(KC_ITG3500_6B_REG, KC_ITG3500_6B_SLEEP_IN);
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

	yas_itg3500_lock();
	err = yas_itg3500_init();
	yas_itg3500_unlock();

	return err;
}

static int yas_term(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_lock();
	err = yas_itg3500_term();
	yas_itg3500_unlock();

	return err;
}

static int yas_get_delay(void)
{
	int ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_lock();
	ret = yas_itg3500_get_delay();
	yas_itg3500_unlock();

	return ret;
}

static int yas_set_delay(int delay)
{
	struct yas_itg3500_data *p = &itg3500_data;
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (delay < YAS_ITG3500_MIN_DELAY_INT ||
	    delay > YAS_ITG3500_MAX_DELAY)
		return YAS_ERROR_ARG;

	if (delay == YAS_ITG3500_MIN_DELAY_INT)
		delay = 1;
	if (p->interrupt == 0) {
		if (delay < YAS_ITG3500_MIN_DELAY_POL)
			delay = YAS_ITG3500_MIN_DELAY_POL;
	}

	yas_itg3500_lock();
	err = yas_itg3500_set_delay(delay);
	yas_itg3500_unlock();

	return err;
}

static int yas_get_enable(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_lock();
	err = yas_itg3500_get_enable();
	yas_itg3500_unlock();

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

	yas_itg3500_lock();
	err = yas_itg3500_set_enable(enable);
	yas_itg3500_unlock();

	return err;
}

static int yas_get_position(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_lock();
	err = yas_itg3500_get_position();
	yas_itg3500_unlock();

	return err;
}

static int yas_set_position(int position)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (position < YAS_ITG3500_MIN_POSITION ||
	    position > YAS_ITG3500_MAX_POSITION)
		return YAS_ERROR_ARG;

	yas_itg3500_lock();
	err = yas_itg3500_set_position(position);
	yas_itg3500_unlock();

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

	yas_itg3500_lock();
	err = yas_itg3500_get_offset(offset);
	yas_itg3500_unlock();

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

	yas_itg3500_lock();
	err = yas_itg3500_set_offset(offset);
	yas_itg3500_unlock();

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

	yas_itg3500_lock();
	err = yas_itg3500_get_filter(filter);
	yas_itg3500_unlock();

	return err;
}

static int yas_set_filter(struct yas_gyro_filter *filter)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (filter == NULL || filter->threshold < 0 ||
	    filter->threshold > YAS_ITG3500_MAX_DATA)
		return YAS_ERROR_ARG;

	yas_itg3500_lock();
	err = yas_itg3500_set_filter(filter);
	yas_itg3500_unlock();

	return err;
}

static int yas_get_filter_enable(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_lock();
	err = yas_itg3500_get_filter_enable();
	yas_itg3500_unlock();

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

	yas_itg3500_lock();
	err = yas_itg3500_set_filter_enable(enable);
	yas_itg3500_unlock();

	return err;
}

static int yas_get_interrupt(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_lock();
	err = yas_itg3500_get_interrupt();
	yas_itg3500_unlock();

	return err;
}

static int yas_set_interrupt(int interrupt)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;
#if 0
	if (interrupt != 0)
		interrupt = 1;
#else /* Interrupt not support */
	interrupt = 0;
#endif
	yas_itg3500_lock();
	err = yas_itg3500_set_interrupt(interrupt);
	yas_itg3500_unlock();

	return err;
}

static void yas_interrupt_handler(void)
{
	struct yas_itg3500_data *p = &itg3500_data;

	/* Check initialize */
	if (p->initialize == 0)
		return;

	yas_itg3500_interrupt_disable();
	yas_itg3500_interrupt_notify(p->fifo_count); /* ToDo fifo_count */
}

static int yas_measure(struct yas_gyro_data *data, int num)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (data == NULL || num <= 0)
		return YAS_ERROR_ARG;

	yas_itg3500_lock();
	err = yas_itg3500_measure(data, num);
	yas_itg3500_unlock();

	return err;
}

#if DEBUG
static int yas_get_register(unsigned char adr, unsigned char *val)
{
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Check initialize */
	if (itg3500_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_lock();
	yas_itg3500_get_register(adr, val);
	yas_itg3500_unlock();

	return YAS_NO_ERROR;
}

static int yas_set_register(unsigned char adr, unsigned char val)
{
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Check initialize */
	if (itg3500_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_lock();
	yas_itg3500_set_register(adr, val);
	yas_itg3500_unlock();

	return YAS_NO_ERROR;
}
#endif

static int32_t kc_set_enable_cnt(int enable)
{
	int ret = YAS_NO_ERROR;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_itg3500_lock();
	kc_itg3500_set_enable_cnt(enable);
	yas_itg3500_unlock();

	return ret;
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

	yas_itg3500_lock();
	err = kc_itg3500_set_reset(reset);
	yas_itg3500_unlock();

	YLOGI(("%s(): end err=%d\n",__func__,err));
	return err;
}

/* -------------------------------------------------------------------------- */
/*  Global function                                                           */
/* -------------------------------------------------------------------------- */
int yas_gyro_driver_init(struct yas_gyro_driver *f, int interrupt)
{
	struct yas_gyro_driver_callback *cbk;
	struct yas_itg3500_data *p = &itg3500_data;

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
	f->interrupt_handler = NULL;

	cbk = &f->callback;
	if (cbk->device_open == NULL ||
	    cbk->device_close == NULL ||
	    cbk->device_write == NULL ||
	    cbk->device_read == NULL ||
	    cbk->msleep == NULL) {
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
	yas_itg3500_term();

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
	f->set_reset = kc_set_reset;
	f->set_enable_cnt = kc_set_enable_cnt;

	p->interrupt = interrupt;
	pcb = &cb;

	gyro_power_cb.power_on = kc_itg3500_power_on;
	gyro_power_cb.power_off = kc_itg3500_power_off;

	return YAS_NO_ERROR;
}

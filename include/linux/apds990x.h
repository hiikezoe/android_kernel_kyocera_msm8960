/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */
#ifndef APDS990X_H
#define APDS990X_H

#include <linux/ioctl.h>

#define D_NV_PROXIMITY_SENSOR_NEAR_SIZE         (0x0a)
#define D_NV_PROXIMITY_SENSOR_FAR_SIZE          (0x0a)
#define D_NV_PHOTO_SENSOR_BEAMISH_SIZE          (0x04)
#define D_NV_PHOTO_SENSOR_DARK_SIZE             (0x04)
#define D_NV_PHOTO_SENSOR_B_SIZE                (0x0a)
#define D_NV_PHOTO_SENSOR_C_SIZE                (0x0a)
#define D_NV_PHOTO_SENSOR_D_SIZE                (0x0a)
#define D_NV_PHOTO_SENSOR_GA_SIZE               (0x0a)
#define D_NV_PROX_PHOTO_COLVAR_SIZE             (0x01)
#define D_NV_DATA_MAX                           (0x0c)

typedef struct _t_apds990x_ioctl_ps_detection
{
    unsigned long ulps_detection;
}T_APDS990X_IOCTL_PS_DETECTION;

typedef struct _t_apds990x_ioctl_als_mean_times
{
    unsigned long ulals_mean_times;
}T_APDS990X_IOCTL_ALS_MEAN_TIMES;

typedef struct _t_apds990x_ioctl_als_lux_ave
{
    unsigned long ulals_lux_ave;
    long lcdata;
    long lirdata;
}T_APDS990X_IOCTL_ALS_LUX_AVE;

typedef struct _t_apds990x_ioctl_nv
{
    unsigned long ulLength;
    unsigned char ucData[D_NV_DATA_MAX];
    unsigned long ulItem;
}T_APDS990X_IOCTL_NV;

#define APDS990X_IO             'A'

#define IOCTL_PS_DETECTION_GET        _IOR(APDS990X_IO, 0x01, T_APDS990X_IOCTL_PS_DETECTION)
#define IOCTL_ALS_MEAN_TIMES_SET      _IOW(APDS990X_IO, 0x02, T_APDS990X_IOCTL_ALS_MEAN_TIMES)
#define IOCTL_ALS_LUX_AVE_GET         _IOR(APDS990X_IO, 0x03, T_APDS990X_IOCTL_ALS_LUX_AVE)
#define IOCTL_APDS990X_NV_DATA_SET    _IOW(APDS990X_IO, 0x04, T_APDS990X_IOCTL_NV)
#define IOCTL_APDS990X_NV_DATA_GET    _IOR(APDS990X_IO, 0x05, T_APDS990X_IOCTL_NV)

enum {
    en_NV_PROXIMITY_SENSOR_NEAR_I = 0,
    en_NV_PROXIMITY_SENSOR_FAR_I,
    en_NV_PHOTO_SENSOR_BEAMISH_I,
    en_NV_PHOTO_SENSOR_DARK_I,
    en_NV_PHOTO_SENSOR_B_I,
    en_NV_PHOTO_SENSOR_C_I,
    en_NV_PHOTO_SENSOR_D_I,
    en_NV_PHOTO_SENSOR_GA_I,
    en_NV_PROX_PHOTO_COLVAR_I,
};

#endif /* APDS990X_H */

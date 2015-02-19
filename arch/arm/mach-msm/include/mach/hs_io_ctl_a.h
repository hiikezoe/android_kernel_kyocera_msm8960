#ifndef HS_IO_CTL_A_H
#define HS_IO_CTL_A_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/* 
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

/*===========================================================================

                       IO CONTROL MACRO HEADER FILE

===========================================================================*/
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/irqs.h>
#include <mach/rpm-regulator.h>

#define GPIO_LO (0)
#define GPIO_HI (1)

typedef enum {
  GPIO_0,
  GPIO_1,
  GPIO_2,
  GPIO_3,
  GPIO_4,
  GPIO_5,
  GPIO_6,
  GPIO_7,
  GPIO_8,
  GPIO_9,

  GPIO_10,
  GPIO_11,
  GPIO_12,
  GPIO_13,
  GPIO_14,
  GPIO_15,
  GPIO_16,
  GPIO_17,
  GPIO_18,
  GPIO_19,

  GPIO_20,
  GPIO_21,
  GPIO_22,
  GPIO_23,
  GPIO_24,
  GPIO_25,
  GPIO_26,
  GPIO_27,
  GPIO_28,
  GPIO_29,

  GPIO_30,
  GPIO_31,
  GPIO_32,
  GPIO_33,
  GPIO_34,
  GPIO_35,
  GPIO_36,
  GPIO_37,
  GPIO_38,
  GPIO_39,

  GPIO_40,
  GPIO_41,
  GPIO_42,
  GPIO_43,
  GPIO_44,
  GPIO_45,
  GPIO_46,
  GPIO_47,
  GPIO_48,
  GPIO_49,

  GPIO_50,
  GPIO_51,
  GPIO_52,
  GPIO_53,
  GPIO_54,
  GPIO_55,
  GPIO_56,
  GPIO_57,
  GPIO_58,
  GPIO_59,

  GPIO_60,
  GPIO_61,
  GPIO_62,
  GPIO_63,
  GPIO_64,
  GPIO_65,
  GPIO_66,
  GPIO_67,
  GPIO_68,
  GPIO_69,

  GPIO_70,
  GPIO_71,
  GPIO_72,
  GPIO_73,
  GPIO_74,
  GPIO_75,
  GPIO_76,
  GPIO_77,
  GPIO_78,
  GPIO_79,

  GPIO_80,
  GPIO_81,
  GPIO_82,
  GPIO_83,
  GPIO_84,
  GPIO_85,
  GPIO_86,
  GPIO_87,
  GPIO_88,
  GPIO_89,

  GPIO_90,
  GPIO_91,
  GPIO_92,
  GPIO_93,
  GPIO_94,
  GPIO_95,
  GPIO_96,
  GPIO_97,
  GPIO_98,
  GPIO_99,

  GPIO_100,
  GPIO_101,
  GPIO_102,
  GPIO_103,
  GPIO_104,
  GPIO_105,
  GPIO_106,
  GPIO_107,
  GPIO_108,
  GPIO_109,

  GPIO_110,
  GPIO_111,
  GPIO_112,
  GPIO_113,
  GPIO_114,
  GPIO_115,
  GPIO_116,
  GPIO_117,
  GPIO_118,
  GPIO_119,

  GPIO_120,
  GPIO_121,
  GPIO_122,
  GPIO_123,
  GPIO_124,
  GPIO_125,
  GPIO_126,
  GPIO_127,
  GPIO_128,
  GPIO_129,

  GPIO_130,
  GPIO_131,
  GPIO_132,
  GPIO_133,
  GPIO_134,
  GPIO_135,
  GPIO_136,
  GPIO_137,
  GPIO_138,
  GPIO_139,

  GPIO_140,
  GPIO_141,
  GPIO_142,
  GPIO_143,
  GPIO_144,
  GPIO_145,
  GPIO_146,
  GPIO_147,
  GPIO_148,
  GPIO_149,

  GPIO_150,
  GPIO_151,

  PM_GPIO_1, // PM8921_GPIO_BASE,
  PM_GPIO_2,
  PM_GPIO_3,
  PM_GPIO_4,
  PM_GPIO_5,
  PM_GPIO_6,
  PM_GPIO_7,
  PM_GPIO_8,
  PM_GPIO_9,

  PM_GPIO_10,
  PM_GPIO_11,
  PM_GPIO_12,
  PM_GPIO_13,
  PM_GPIO_14,
  PM_GPIO_15,
  PM_GPIO_16,
  PM_GPIO_17,
  PM_GPIO_18,
  PM_GPIO_19,

  PM_GPIO_20,
  PM_GPIO_21,
  PM_GPIO_22,
  PM_GPIO_23,
  PM_GPIO_24,
  PM_GPIO_25,
  PM_GPIO_26,
  PM_GPIO_27,
  PM_GPIO_28,
  PM_GPIO_29,

  PM_GPIO_30,
  PM_GPIO_31,
  PM_GPIO_32,
  PM_GPIO_33,
  PM_GPIO_34,
  PM_GPIO_35,
  PM_GPIO_36,
  PM_GPIO_37,
  PM_GPIO_38,
  PM_GPIO_39,

  PM_GPIO_40,
  PM_GPIO_41,
  PM_GPIO_42,
  PM_GPIO_43,
  PM_GPIO_44

} gpio_enum_type;

extern void hs_vreg_ctl(const char* reg_name, bool onoff, int min_uV, int max_uV);

#define GPIO_I2C_PRIM_SDA       12
#define GPIO_I2C_PRIM_SCL       13
#define GPIO_I2C_CAM_SDA        16
#define GPIO_I2C_CAM_SCL        17
#define GPIO_I2C_MAX_SDA        36
#define GPIO_I2C_MAX_SCL        37
#define GPIO_I2C_TP_SDA         40
#define GPIO_I2C_TO_SCL         41
#define GPIO_I2C_SECO_SDA       44
#define GPIO_I2C_SECO_SCL       45

#define GPIO_S7708A_I2C_SDA     GPIO_I2C_PRIM_SDA
#define GPIO_S7708A_I2C_SCL     GPIO_I2C_PRIM_SCL

#define GPIO_I2C_CAM_SCL_LABEL  "i2c_cam_scl"
#define GPIO_I2C_CAM_SDA_LABEL  "i2c_cam_sda"
#define GPIO_I2C_PRIM_SCL_LABEL "i2c_prim_scl"
#define GPIO_I2C_PRIM_SDA_LABEL "i2c_prim_sda"
#define GPIO_I2C_MAX_SCL_LABEL  "i2c_max_scl"
#define GPIO_I2C_MAX_SDA_LABEL  "i2c_max_sda"
#define GPIO_I2C_TP_SCL_LABEL   "i2c_tp_scl"
#define GPIO_I2C_TP_SDA_LABEL   "i2c_tp_sda"
#define GPIO_I2C_SECO_SCL_LABEL "i2c_seco_scl"
#define GPIO_I2C_SECO_SDA_LABEL "i2c_seco_sda"

#define MSM_8960_GSBI2_QUP_I2C_BUS_ID    2
#define MSM_8960_GSBI3_QUP_I2C_BUS_ID    3
#define MSM_8960_GSBI11_QUP_I2C_BUS_ID  11
#define MSM_8960_GSBI12_QUP_I2C_BUS_ID  12

#endif /* HS_IO_CTL_A_H */


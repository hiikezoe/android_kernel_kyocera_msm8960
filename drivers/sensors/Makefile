#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2012 KYOCERA Corporation
# (C) 2013 KYOCERA Corporation
#
#
# Copyright (c) 2010 Yamaha Corporation
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA.
#
ccflags-y := -mfloat-abi=softfp -mfpu=vfp

obj-$(CONFIG_SENSOR_POWER) += kc_sensor_power.o

obj-$(CONFIG_INPUT_ML610Q793_ACCELEROMETER) += ml610q793.o
obj-$(CONFIG_INPUT_YAS_MAGNETOMETER) += yas_mag_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_GYROSCOPE) += yas_gyro_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_ORIENTATION) += yas_ori_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_AMBIENT_TEMPERATURE) += yas_ambient_temperature_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_GRAVITY) +=  yas_gravity_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_LINEAR_ACCELERATION) += yas_linear_acceleration_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_PRESSURE) +=  yas_pres_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_RELATIVE_HUMIDITY) += yas_relative_humidity_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_ROTATION_VECTOR) += yas_rotation_vector_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_TEMPERATURE) += yas_temp_kernel_driver.o
obj-$(CONFIG_INPUT_YAS_SOFTWARE_GYROSCOPE) += yas_sw_gyroscope_kernel_driver.o

obj-$(CONFIG_INPUT_SENSOR_APDS990X) += apds990x.o
obj-$(CONFIG_INPUT_SENSOR_GP2AP020) += gp2ap020a.o


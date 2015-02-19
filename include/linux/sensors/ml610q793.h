/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/*
 *  ml610q793.h - Linux kernel modules for acceleration sensor
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


#ifndef _ML610D793_H_
#define _ML610D793_H_

#define    ACCSNS_ACTIVE_ACC          0x10
#define    ACCSNS_ACTIVE_PEDOM        0x20
#define    ACCSNS_ACTIVE_VEHICLE      0x40
#define    ACCSNS_ACTIVE_MOVE         0x80
#define    ACCSNS_ACTIVE_IWIFI      0x2000
#define    ACCSNS_ACTIVE_DAILYS       (ACCSNS_ACTIVE_PEDOM | ACCSNS_ACTIVE_VEHICLE)
#define    ACCSNS_ACTIVE_PEDOM_ERROR    (ACCSNS_ACTIVE_PEDOM >> 4)
#define    ACCSNS_ACTIVE_VEHICLE_ERROR  (ACCSNS_ACTIVE_VEHICLE >> 4)
#define    ACCSNS_ACTIVE_IWIFI_ERROR  (ACCSNS_ACTIVE_IWIFI >> 4)
#define    ACTIVE_FUNC_MASK    (ACCSNS_ACTIVE_ACC | ACCSNS_ACTIVE_PEDOM | ACCSNS_ACTIVE_VEHICLE | ACCSNS_ACTIVE_IWIFI)
#define    ACCSNS_ACTIVE_ERROR (ACCSNS_ACTIVE_PEDOM_ERROR | ACCSNS_ACTIVE_VEHICLE_ERROR | ACCSNS_ACTIVE_IWIFI_ERROR)

#define    PRESNS_ACTIVE_PRE          0x1000
#define    ACTIVE_FUNC_PRE_MASK  (PRESNS_ACTIVE_PRE | ACCSNS_ACTIVE_PEDOM | ACCSNS_ACTIVE_VEHICLE)

#define    ACC_ENABLE_MASK (PRESNS_ACTIVE_PRE)
#define    PRE_ENABLE_MASK (ACCSNS_ACTIVE_ACC | ACCSNS_ACTIVE_MOVE)

#define    DAILYS_CLEAR_PEDO_DATA     0x01
#define    DAILYS_CLEAR_PEDO_STATE    0x02
#define    DAILYS_CLEAR_VEHI_DATA     0x04
#define    DAILYS_CLEAR_VEHI_STATE    0x08
#define    DAILYS_CLEAR_WIFI_STATE    0x10
#define    DAILYS_CLEAR_HEIGHT_STATE  0x20
#define    DAILYS_CLEAR_HEIGHT_ALL    0x40
#define    DAILYS_CLEAR_ALL (DAILYS_CLEAR_PEDO_DATA | DAILYS_CLEAR_PEDO_STATE | DAILYS_CLEAR_VEHI_DATA | DAILYS_CLEAR_VEHI_STATE | DAILYS_CLEAR_WIFI_STATE | DAILYS_CLEAR_HEIGHT_ALL)

#define    DAILYS_INT_PEDO     0x01
#define    DAILYS_INT_VEHI     0x02
#define    DAILYS_INT_WIFI     0x10

#define    DAILYS_INT_DST (DAILYS_INT_PEDO | DAILYS_INT_VEHI)

//acc_tmp alps_io
typedef struct tACC_Set_Pedo_Params_NV {
    uint8_t param_on;
    uint8_t on;
    uint8_t stepwide;
    uint8_t weight;
    uint8_t notify;
    uint8_t speed_ave_time;
    uint8_t mets_stop_time;
    uint8_t bodyfat_on;
    uint8_t bodyfat_cal;
} ACC_SET_PEDO_PARAMS_NV;
    
typedef struct tACC_Set_Dist_Stop_Params_NV {
    uint8_t  stop_notify_on;
    uint16_t stop_notify_time;
} ACC_SET_DIST_STOP_PARAMS_NV;
    
typedef struct tACC_Set_Walk_Run_Params_NV {
    uint8_t  walk_judge_on;
    uint8_t  consecutive_num;
    uint16_t speed_th;
} ACC_SET_WALK_RUN_PARAMS_NV;

typedef struct tACC_Set_Trans_Params_NV {
    uint8_t  on;
    uint8_t  judge_step;
    uint16_t judge_time;
    uint8_t  calc_time;
    uint8_t  consecutive_num;
} ACC_SET_TRANS_PARAMS_NV;
    
typedef struct tACC_Set_Trans_Byc_Params_NV {
    uint8_t consecutive_num;
    uint8_t calory_mets;
} ACC_SET_TRANS_BYC_PARAMS_NV;

typedef struct tACC_Set_Timer_Params_NV {
    uint8_t  on;
    uint32_t num;
} ACC_SET_TIMER_PARAMS_NV;

typedef struct tACC_Set_MoveDetect_Params_NV {
    uint8_t  on;
    uint8_t  axis;
    uint16_t acc_th;
    uint8_t  judge_num;
    uint8_t  judge_th;
    uint8_t  judge;
} ACC_SET_MOVE_DETECT_PARAMS_NV;
    
typedef struct {
    ACC_SET_PEDO_PARAMS_NV        pedo_p;
    ACC_SET_DIST_STOP_PARAMS_NV   dist_stop_p;
    ACC_SET_WALK_RUN_PARAMS_NV    walk_run_p;
    ACC_SET_TRANS_PARAMS_NV       trans_p;
    ACC_SET_TRANS_BYC_PARAMS_NV   trans_byc_p;
    ACC_SET_TIMER_PARAMS_NV       timer_p;
    ACC_SET_MOVE_DETECT_PARAMS_NV move_p;
}IoCtlAccSetAccsnsNVParams;
//acc_tmp alps_io

typedef struct {
  uint32_t m_nStepWide;
  uint32_t m_nWeight;
  uint32_t m_VehiType;
}DailysSetParam;

typedef struct {
  uint32_t m_nPedoStartStep;
  uint32_t m_nPedoEndTime;
  uint32_t m_nVehiStartTime;
  uint32_t m_nVehiEndTime;
}DailysSetIWifiParam;

typedef struct {
  uint32_t m_SetMode;
  uint32_t m_BasePress;
  uint32_t m_BaseHeight;
}DailysSetBaseParam;

typedef struct {
  uint32_t m_iStep;
  uint32_t m_iTime;
  uint32_t m_iCalorie;
  uint32_t m_iFat;
  uint32_t m_iExercise;
  uint32_t m_iMets;
  uint32_t m_iSpeedmeter;
  uint32_t m_iRunStatus;
  uint32_t m_iRunStepCnt;
  uint32_t m_iRunTime;
  uint32_t m_iStExercise;
  uint32_t m_iStCal;
  uint32_t m_iStBodyFat;
  uint32_t m_iSportExercise;
  uint32_t m_iRunCal;
  uint32_t m_iRunExercise;
  int32_t  m_iPS_InitBaseHeight;
  int32_t  m_iPS_ActHeight;
  int32_t  m_iPS_ActHeightMin;
  int32_t  m_iPS_ActHeightAve;
  int32_t  m_iPS_ActHeightMax;
}DailysGetPedomInfo;

typedef struct {
  uint32_t m_iStatus;
  uint32_t m_iKind;
  uint32_t m_iDetectTime;
  uint32_t m_iRideTime;
  uint32_t m_iRideCal;
  uint32_t m_iVehiBodyFat;
  uint32_t m_iVehiExercise;
  uint32_t m_iVehiMets;
  uint32_t m_iVehiStExercise;
  uint32_t m_iVehiStRideCal;
  uint32_t m_iVehiStBodyFat;
  uint32_t m_iVehiBiExercise;
  uint32_t m_iVehiBiRideCal;
  uint32_t m_iVehiBiBodyFat;
  uint32_t m_iVehiSportExercise;
}DailysGetVehicleInfo;

typedef struct {
  uint32_t m_iPedoStatus;
  uint32_t m_iVehiStatus;
}DailysGetIntelliWifiInfo;

#endif

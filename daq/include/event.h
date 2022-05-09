#pragma once

#define M_SL_MSGID_TP_LOCATION_REQ      0x0200

#define M_SL_MSGID_PT_LOCATION_REQ      0x8201
#define M_SL_MSGID_TP_LOCATION_RSP      0x0201

#define M_SL_MSGID_TP_FAULT_REQ         0x0F3A
#define M_SL_MSGID_TP_BUCKET_REQ        0x0F3B
#define M_SL_MSGID_TP_STATISTICS_REQ    0x0F3C
#define M_SL_MSGID_TP_BEHAVIOR_REQ      0x0F3D

#define M_SL_MSGID_UPDATE_CFG_REQ       0x9527
#define M_SL_MSGID_UPDATE_CFG_RSP       0x1527

#define M_SL_MSGID_SYSTIME_SYNC         0x9528
#define M_SL_MSGID_SYSTIME_READY        0x9529

#define M_SL_MSGID_TEL_NTF              0x952B
#define M_SL_MSGID_DRIVE_EVENT          0x952C
#define M_SL_MSGID_MCU_NTF              0x952D
#define M_SL_MSGID_SYSPOWER_NTF         0x952E
#define M_SL_MSGID_LOCATOIN_NTF         0x952F

#define M_SL_MSGID_DAY_CHANGED          0x9600

#define M_SL_MSGID_UPDATE_VAL_REQ       0xA527
#define M_SL_MSGID_UPDATE_VAL_RSP       0x2527

#define M_SL_MSGID_COMPRESS_REQ         0xF527
#define M_SL_MSGID_COMPRESS_RSP         0x7527

#define M_SL_MSGID_TIMER_NOTIFY         0xFF01
#define M_SL_MSGID_DATA_CHANGED         0xFF0A
#define M_SL_MSGID_DATA_TIRE_NTF        0xFF0B
#define M_SL_MSGID_DATA_DM1_NTF         0xFF0C
#define M_SL_MSGID_DATA_DM1_TIMER       0xFF0D

#define M_SL_EVID_SHARP_SLOW_DOWN       0x0A
#define M_SL_EVID_SHARP_SPEED_UP        0x0C
#define M_SL_EVID_SHARP_TURN            0x0E
#define M_SL_EVID_SPEEDING              0x12
#define M_SL_EVID_LONG_LONG_PARKING     0x13
#define M_SL_EVID_ACCIDENT              0x14
#define M_SL_EVID_TPMS_TIRE_WARN        0x1F
#define M_SL_EVID_TIREDNESS_DRIVING     0x22
#define M_SL_EVID_DISMANTLE_TBOX        0x24
#define M_SL_EVID_SOC_LOW               0x28
#define M_SL_EVID_DMS_WARN              0x2C
#define M_SL_EVID_FUEL_TANK_WARN        0x2D
#define M_SL_EVID_AGITATOR_FAULT        0x2E
#define M_SL_EVID_GB_CHECK_VIN_ERROR    0x2F
#define M_SL_EVID_REFRIGERATOR_FAILURE  0x30

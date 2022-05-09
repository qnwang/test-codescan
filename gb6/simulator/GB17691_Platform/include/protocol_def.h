//
// Created by t on 4/6/22.
//

#ifndef GB17691_PLATFORM_PROTOCOL_DEF_H
#define GB17691_PLATFORM_PROTOCOL_DEF_H

#pragma once

#include <smlk_typedefs.h>

#define M_GB_17691_SZ_FLAGS                 2
#define M_GB_17691_SZ_VIN                   17
#define M_GB_17691_SZ_SCIN                  18
#define M_GB_17691_SZ_CVN                   18
#define M_GB_17691_SZ_IUPR                  36
#define M_GB_17691_SZ_CHIP_ID               16
#define M_GB_17691_SZ_PUB_KEY               64
#define M_GB_17691_SZ_TIMESTAMP             6
#define M_GB_17691_SZ_ICCID                 20
#define M_GB_17691_SZ_DATA_MAX              65531
#define M_GB_17691_SZ_HEAD_INFO             25              // 2 x '#' + 1 x ${cmd} + 17 x ${vin} + 1 x ${reversion} + 1 x ${encryption} + 2 x ${length} + 1 x ${BCC}

#define M_GB_17691_OBD_DELAY_MAX            20              // OBD information max delay seconds after engine started
#define M_GB_17691_OBD_DELAY_DEF            10              // OBD information default delay seconds after engine started
#define M_GB_17691_ENGINE_DELAY_MAX         60              // Engine stream information report max delay seconds after engine started
#define M_GB_17691_ENGINE_DELAY_DEF         30              // Engine stream information report default delay seconds after engine started
#define M_GB_17691_REPORT_PERIOD_MAX        10              // report period max
#define M_GB_17691_REPORT_PERIOD_DEF        10              // report period defualt
#define M_GB_17691_RECONNECT_PERIOD_MAX     180             // reconnect period max
#define M_GB_17691_RECONNECT_PERIOD_MIN     3
#define M_GB_17591_RECONNECT_PERIOD_DEF     30              // reconnect period default

#define M_GB_17691_MSG_FLAG                 0x23            // '#'

#define M_GB_17691_PROTOCOL_REVERSION       0x00            // protocol reversion

// GB17691 command define
#define M_GB_17691_CMD_VEHICLE_LOGIN        0x01
#define M_GB_17691_CMD_REALTIME_REPORT      0x02
#define M_GB_17691_CMD_REISSUE_REPORT       0x03
#define M_GB_17691_CMD_VEHICLE_LOGOUT       0x04
#define M_GB_17691_CMD_TERMIAL_TIMING       0x05

// GB17691 encryption method define
#define M_GB_17691_ENCRYPTION_NONE          0x01
#define M_GB_17691_ENCRYPTION_RSA           0x02
#define M_GB_17691_ENCRYPTION_SM2           0x03
#define M_GB_17691_ENCRYPTION_ABNORMAL      0xFE
#define M_GB_17691_ENCRYPTION_INVALID       0xFF

// GB17691 message type define
#define M_GB_17691_MESSAGE_OBD              0x01
#define M_GB_17691_MESSAGE_STREAM           0x02

// GB17691 OBD protocol define
#define M_GB_17691_OBD_PROTOCOL_ISO15765    0x00
#define M_GB_17691_OBD_PROTOCOL_ISO27154    0x01
#define M_GB_17691_OBD_PROTOCOL_SAEJ1939    0x02
#define M_GB_17691_OBD_PROTOCOL_INVALID     0xFE

// GB17691 MIL status define
#define M_GB_17691_MIL_OFF                  0x00
#define M_GB_17691_MIL_ON                   0x01
#define M_GB_17691_MIL_INVALID              0xFE

// GB17691 diagnostic bit mask define
#define M_GB_17691_MASK_DIAG_CATALYST                   0x0001
#define M_GB_17691_MASK_DIAG_HEATED_CATALYST            0x0002
#define M_GB_17691_MASK_DIAG_EVAPORATIVE_SYS            0x0004
#define M_GB_17691_MASK_DIAG_SECONDARY_AIR_SYS          0x0008
#define M_GB_17691_MASK_DIAG_AC_SYS_REFRIGERANT         0x0010
#define M_GB_17691_MASK_DIAG_EXHAUST_GAS_SENSOR         0x0020
#define M_GB_17691_MASK_DIAG_EXHAUST_GAS_SENSOR_HEATER  0x0040
#define M_GB_17691_MASK_DIAG_EGR_VVT_SYS                0x0080
#define M_GB_17691_MASK_DIAG_COLD_START_AID_SYS         0x0100
#define M_GB_17691_MASK_DIAG_BOOST_PRESSURE_CONTROL_SYS 0x0200
#define M_GB_17691_MASK_DIAG_DIESEL_PARTICULATE_FILTER  0x0400
#define M_GB_17691_MASK_DIAG_SCR_NOX_ADSORBER           0x0800
#define M_GB_17691_MASK_DIAG_NMHC_CONVERTING_CATALYST   0x1000
#define M_GB_17691_MASK_DIAG_MISFIRE                    0x2000
#define M_GB_17691_MASK_DIAG_FUEL_SYS                   0x4000
#define M_GB_17691_MASK_DIAG_COMPREHENSIVE_COMPONENT    0x8000

// GB17691 message head define
#pragma pack(1)
typedef struct {
    BYTE    flags[M_GB_17691_SZ_FLAGS];
    BYTE    cmd;
    BYTE    vin[M_GB_17691_SZ_VIN];
    BYTE    ver;
    BYTE    encryption;
    WORD    length;
} GB_17691_MsgHead;

typedef struct {
    BCD     timestamp[M_GB_17691_SZ_TIMESTAMP];
    WORD    sequence;
    BYTE    iccid[M_GB_17691_SZ_ICCID];             // string type, I'm not sure if it is a specified length.
} GB_17691_VehicleLogin;

typedef struct {
    BCD     timestamp[M_GB_17691_SZ_TIMESTAMP];
    WORD    sequence;
} GB_17691_VehicleLogout;

typedef struct {
    BYTE    protocol;                               // 0x00: IOS15765, 0x01: IOS27145, 0x02: SAEJ1939, 0xFE: invalid
    BYTE    mil_status;                             // 0x00: OFF, 0x01: ON, 0xFE: invalid
    WORD    diag_support_status;                    // seed M_GB_17691_MASK_DIAG_... defines
    WORD    diag_ready_status;                      // seed M_GB_17691_MASK_DIAG_... defines
    BYTE    vin[M_GB_17691_SZ_VIN];                 // VIN
    BYTE    scvn[M_GB_17691_SZ_SCIN];               // SCVN software calibration id number
    BYTE    cvn[M_GB_17691_SZ_CVN];                 // CVN  calibration verify number
    BYTE    iupr[M_GB_17691_SZ_IUPR];               // IUPR DSTRING
} GB_17691_OBD_InformationHead;

typedef struct {
    WORD    vehicle_speed;                          // vehicle speed
    BYTE    barometric_pressure;                    // barometric pressure
    BYTE    output_torque;                          // engine output torque
    BYTE    nominal_friction_percent_torque;        // engine nominal
    WORD    speed;                                  // engine rotate speed
    WORD    fuel_flow;                              // engine fuel flow
    WORD    aftertreatment_intake_nox;              // aftertreatment intake NOx
    WORD    aftertreatment_outlet_nox;              // aftertreatment outlet NOx
    BYTE    reaction_dhi_margin;                    // reachtion dhi margin
    WORD    air_inflow;                             // engine air inflow
    WORD    scr_intake_temperature;                 // SCR intake temperature
    WORD    scr_outlet_temperature;                 // SCR outlet temperature
    WORD    dpf_differential_pressure;              // DPF differential pressure
    BYTE    coolant_temperature;                    // engine collant temperature
    BYTE    fuel_level;                             // fuel level
    BYTE    located_status;                         // located status       bit 0: valid flat, bit 1: north or sourth, bit 2: east or west
    DWORD   longitude;                              // longitude
    DWORD   latitude;                               // latitude
    DWORD   mileage;                                // mileage
} GB_17691_EngineDataStream;
#pragma pack()

#endif //GB17691_PLATFORM_PROTOCOL_DEF_H

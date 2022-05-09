/******************************************************************************
*
*  Copyright (C) 2020 SmartLink
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/
#ifndef __GB_17691_H__
#define __GB_17691_H__
#include <smlk_types.h>

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

#define M_GB_17691_RECONNECT_PERIOD_MAX     180             // reconnect period max
#define M_GB_17691_RECONNECT_PERIOD_MIN     3
#define M_GB_17591_RECONNECT_PERIOD_DEF     30              // reconnect period default

#define M_GB_LOCATION_CHANGCHUN             0
#define M_GB_LOCATION_QINGDAO               1

#define M_HJ_1239_SZ_DATA_MAX               2048

#define M_GB_17691_MSG_FLAG                 0x23            // '#'

#define M_HJ_1239_MSG_FLAG                  0x7e            // '~'
#define M_HJ_1239_PROTOCOL_REVERSION        0x06            // protocol reversion

// GB17691 command define
#define M_GB_17691_CMD_VEHICLE_LOGIN        0x01
#define M_GB_17691_CMD_REALTIME_REPORT      0x02
#define M_GB_17691_CMD_REISSUE_REPORT       0x03
#define M_GB_17691_CMD_VEHICLE_LOGOUT       0x04
#define M_GB_17691_CMD_TERMIAL_TIMING       0x05

// HJ1239 command define
#define M_HJ_1239_CMD_VEHICLE_LOGIN         0x01
#define M_HJ_1239_CMD_REALTIME_REPORT       0x02
#define M_HJ_1239_CMD_REISSUE_REPORT        0x03
#define M_HJ_1239_CMD_VEHICLE_LOGOUT        0x04
#define M_HJ_1239_CMD_TERMIAL_TIMING        0x05
#define M_HJ_1239_CMD_TERMIAL_ALARM         0x06        // 车辆拆除报警信息     上行 
#define M_HJ_1239_CMD_VEHICLE_ACT_REQ       0x07        // 激活信息             上行 
#define M_HJ_1239_CMD_VEHICLE_ACT_RSP       0x08        // 激活信息应答         下行

// HJ1239 activated status
#define M_HJ_1239_ACTIVATED_RESERVE         0x00        // 未收到激活命令
#define M_HJ_1239_ACTIVATED_SUCCESS         0x01        // 激活成功
#define M_HJ_1239_ACTIVATED_CHIP_ERROR      0x02        // 激活失败 芯片已激活
#define M_HJ_1239_ACTIVATED_VIN_ERROR       0x03        // 激活失败 VIN 错误
#define M_HJ_1239_ACTIVATED_OTHER_ERROR     0x04        // 激活失败 超时/验签失败 其他失败

#define M_HJ_1239_ACTIVATED_ING             0x05        // 激活中
#define M_HJ_1239_ACTIVATED_DONE            0x06        // 激活完成

// GB17691 encryption method define
#define M_GB_17691_ENCRYPTION_NONE          0x01
#define M_GB_17691_ENCRYPTION_RSA           0x02
#define M_GB_17691_ENCRYPTION_SM2           0x03
#define M_GB_17691_ENCRYPTION_ABNORMAL      0xFE
#define M_GB_17691_ENCRYPTION_INVALID       0xFF

// GB17691 message type define
#define M_GB_17691_MESSAGE_OBD              0x01
#define M_GB_17691_MESSAGE_STREAM           0x02

#define M_HJ_1239_MESSAGE_STREAM_DPF_SCR    0x02
#define M_HJ_1239_MESSAGE_STREAM_TWC        0x03
#define M_HJ_1239_MESSAGE_STREAM_HEV        0x04
#define M_HJ_1239_MESSAGE_STREAM_TWC_NOX    0x05

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

// GB17691 IGN status define
#define M_GB_17691_IGN_OFF                  0x00
#define M_GB_17691_IGN_ON                   0x01

// Platform
#define M_PLATFORM_BUSINESS                 0x01
#define M_PLATFORM_NATIONAL                 0x02

// Platform connection status to OBD
#define M_ENT_PLATFORM_CONNECT_NORMAL       0x01
#define M_ENT_PLATFORM_CONNECT_FAILED       0x02
#define M_LOC_PLATFORM_CONNECT_NORMAL       0x03
#define M_LOC_PLATFORM_CONNECT_FAILED       0x04


#define M_SMARTLINK_DIR             "/userdata/smartlink"
#define M_SMARTLINK_RECORD_DIR      M_SMARTLINK_DIR "/record"
#define M_SMARTLINK_DATABASE_DIR    M_SMARTLINK_RECORD_DIR "/database"
#define M_GB6_RECORD_DATABASE_DIR   M_SMARTLINK_RECORD_DIR  "/gb6"

#define M_GB6_CONFIG_DIR            "/userdata/smartlink/config/gb6/"
#define M_GB6_CONFIG_FILE_DIR       "/userdata/smartlink/config/gb6/gb6_config.ini"
#define M_GB6_RAW_DATA_SAVE_DIR     "/userdata/smartlink/record/gb6/record_gb17691.log"

#define SYS_PRO_NAME_HJ_ACTIVATED_STATUS                "hj_act_status"

// GB17691 message head define
#pragma pack(1)
typedef struct {
    SMLK_UINT8  flags[M_GB_17691_SZ_FLAGS];
    SMLK_UINT8  cmd;
    SMLK_UINT8  vin[M_GB_17691_SZ_VIN];
    SMLK_UINT8  ver;
    SMLK_UINT8  encryption;
    SMLK_UINT16 length;
} GB_17691_MsgHead;

typedef struct {
    SMLK_BCD    timestamp[M_GB_17691_SZ_TIMESTAMP];
    SMLK_UINT16 sequence;
    SMLK_UINT8  iccid[M_GB_17691_SZ_ICCID];             // string type, I'm not sure if it is a specified length.
} GB_17691_VehicleLogin;

typedef struct {
    SMLK_BCD    timestamp[M_GB_17691_SZ_TIMESTAMP];
    SMLK_UINT16 sequence;
} GB_17691_VehicleLogout;

typedef struct {
    SMLK_UINT8  protocol;                               // 0x00: IOS15765, 0x01: IOS27145, 0x02: SAEJ1939, 0xFE: invalid
    SMLK_UINT8  mil_status;                             // 0x00: OFF, 0x01: ON, 0xFE: invalid
    SMLK_UINT16 diag_support_status;                    // seed M_GB_17691_MASK_DIAG_... defines
    SMLK_UINT16 diag_ready_status;                      // seed M_GB_17691_MASK_DIAG_... defines
    SMLK_UINT8  vin[M_GB_17691_SZ_VIN];                 // VIN
    SMLK_UINT8  scvn[M_GB_17691_SZ_SCIN];               // SCVN software calibration id number
    SMLK_UINT8  cvn[M_GB_17691_SZ_CVN];                 // CVN  calibration verify number
    SMLK_UINT8  iupr[M_GB_17691_SZ_IUPR];               // IUPR DSTRING
} GB_17691_OBD_InformationHead;

typedef struct {
    SMLK_UINT16 vehicle_speed;                          // vehicle speed
    SMLK_UINT8  barometric_pressure;                    // barometric pressure
    SMLK_UINT8  output_torque;                          // engine output torque
    SMLK_UINT8  nominal_friction_percent_torque;        // engine nominal
    SMLK_UINT16 speed;                                  // engine rotate speed
    SMLK_UINT16 fuel_flow;                              // engine fuel flow
    SMLK_UINT16 aftertreatment_intake_NOx;              // aftertreatment intake NOx
    SMLK_UINT16 aftertreatment_outlet_NOx;              // aftertreatment outlet NOx
    SMLK_UINT8  reaction_dhi_margin;                    // reachtion dhi margin
    SMLK_UINT16 air_inflow;                             // engine air inflow
    SMLK_UINT16 scr_intake_temperature;                 // SCR intake temperature
    SMLK_UINT16 scr_outlet_temperature;                 // SCR outlet temperature
    SMLK_UINT16 dpf_differential_pressure;              // DPF differential pressure
    SMLK_UINT8  coolant_temperature;                    // engine collant temperature
    SMLK_UINT8  fuel_level;                             // fuel level
    SMLK_UINT8  located_status;                         // located status       bit 0: valid flat, bit 1: north or sourth, bit 2: east or west
    SMLK_UINT32 longitude;                              // longitude
    SMLK_UINT32 latitude;                               // latitude
    SMLK_UINT32 mileage;                                // mileage
} GB_17691_EngineDataStream;


// HJ
typedef struct
{
    GB_17691_MsgHead msghead;
    SMLK_UINT8      *dataBody;
    SMLK_UINT8      checkCode;
} HJ_1239_MsgEntity;

typedef struct {
    SMLK_UINT8  statuscode;
    SMLK_UINT8  infor;
} HJ_1239_VehicleActiveResp;

typedef struct {
    SMLK_UINT8  flags[M_GB_17691_SZ_FLAGS];
    SMLK_UINT8  cmd;
    SMLK_UINT8  vin[M_GB_17691_SZ_VIN];
    SMLK_UINT8  ver;
    SMLK_UINT8  encryption;
    SMLK_UINT16 length;
} GB_HJ_MsgHead;

typedef struct {
    SMLK_BCD    timestamp[M_GB_17691_SZ_TIMESTAMP];
    SMLK_UINT16 sequence;
    SMLK_UINT8  iccid[M_GB_17691_SZ_ICCID];             // string type, I'm not sure if it is a specified length.
} GB_HJ_VehicleLogin;

typedef struct {
    SMLK_BCD    timestamp[M_GB_17691_SZ_TIMESTAMP];
    SMLK_UINT16 sequence;
} GB_HJ_VehicleLogout;

typedef struct {
    SMLK_UINT8  protocol;                               // 0x00: IOS15765, 0x01: IOS27145, 0x02: SAEJ1939, 0xFE: invalid
    SMLK_UINT8  mil_status;                             // 0x00: OFF, 0x01: ON, 0xFE: invalid
    SMLK_UINT16 diag_support_status;                    // seed M_GB_17691_MASK_DIAG_... defines
    SMLK_UINT16 diag_ready_status;                      // seed M_GB_17691_MASK_DIAG_... defines
    SMLK_UINT8  vin[M_GB_17691_SZ_VIN];                 // VIN
    SMLK_UINT8  scvn[M_GB_17691_SZ_SCIN];               // SCVN software calibration id number
    SMLK_UINT8  cvn[M_GB_17691_SZ_CVN];                 // CVN  calibration verify number
    SMLK_UINT8  iupr[M_GB_17691_SZ_IUPR];               // IUPR DSTRING
} GB_HJ_OBD_InformationHead;

typedef struct {
    SMLK_UINT16 vehicle_speed;                          // vehicle speed
    SMLK_UINT8  barometric_pressure;                    // barometric pressure
    SMLK_UINT8  output_torque;                          // engine output torque
    SMLK_UINT8  nominal_friction_percent_torque;        // engine nominal
    SMLK_UINT16 speed;                                  // engine rotate speed
    SMLK_UINT16 fuel_flow;                              // engine fuel flow
    SMLK_UINT16 aftertreatment_intake_NOx;              // aftertreatment intake NOx
    SMLK_UINT16 aftertreatment_outlet_NOx;              // aftertreatment outlet NOx
    SMLK_UINT8  reaction_dhi_margin;                    // reachtion dhi margin
    SMLK_UINT16 air_inflow;                             // engine air inflow
    SMLK_UINT16 scr_intake_temperature;                 // SCR intake temperature
    SMLK_UINT16 scr_outlet_temperature;                 // SCR outlet temperature
    SMLK_UINT16 dpf_differential_pressure;              // DPF differential pressure
    SMLK_UINT8  coolant_temperature;                    // engine collant temperature
    SMLK_UINT8  fuel_level;                             // fuel level
    SMLK_UINT8  located_status;                         // located status       bit 0: valid flat, bit 1: north or sourth, bit 2: east or west
    SMLK_UINT32 longitude;                              // longitude
    SMLK_UINT32 latitude;                               // latitude
    SMLK_UINT32 mileage;                                // mileage
} GB_HJ_EngineDataStream;

// 2020.4 HJ
typedef struct {
    SMLK_UINT16 vehicle_speed;                          // vehicle speed
    SMLK_UINT8  barometric_pressure;                    // barometric pressure
    SMLK_UINT8  output_torque;                          // engine output torque
    SMLK_UINT8  nominal_friction_percent_torque;        // engine nominal
    SMLK_UINT16 speed;                                  // engine rotate speed
    SMLK_UINT16 fuel_flow;                              // engine fuel flow
    SMLK_UINT16 twc_aftertreatment_outlet_NOx;          // aftertreatment outlet NOx
    SMLK_UINT16 twc_aftertreatment_intake_NOx;          // aftertreatment intake NOx
    SMLK_UINT16 twc_aftertreatment_intake_O2;           // aftertreatment intake O2
    SMLK_UINT8  coolant_temperature;                    // engine collant temperature
    SMLK_UINT8  fuel_level;                             // fuel level
    SMLK_UINT8  located_status;                         // located status       bit 0: valid flat, bit 1: north or sourth, bit 2: east or west
    SMLK_UINT32 longitude;                              // longitude
    SMLK_UINT32 latitude;                               // latitude
    SMLK_UINT32 mileage;                                // mileage
} GB_HJ_EngineDataStream_TWC;

// 2021.1 HJ
typedef struct {
    SMLK_UINT16 motor_speed;
    SMLK_UINT8  motor_load_percent;
    SMLK_UINT16 battery_voltage;
    SMLK_UINT16 battery_current;
    SMLK_UINT8  soc_percent;
} GB_HJ_EngineDataStream_HEV;

namespace smartlink {

namespace ProtocolReporter {

namespace GNSS {
namespace Flags {
    static const SMLK_UINT32    SL_VALID                = 0x00000000;
    static const SMLK_UINT32    SL_INVALID              = 0x00000001;
    static const SMLK_UINT32    SL_FIXED_GPS            = 0x00000002;
    static const SMLK_UINT32    SL_FIXED_GLONASS        = 0x00000004;
    static const SMLK_UINT32    SL_FIXED_BDS            = 0x00000008;
    static const SMLK_UINT32    SL_FIXED_GSNS           = 0x00000010;

    static const SMLK_UINT32    SL_ANTENNA_PRESENT      = 0x00000100;
    static const SMLK_UINT32    SL_ANTENNA_SHORT        = 0x00000200;

    static const SMLK_UINT32    SL_MODULE_FAULT         = 0x10000000;
};  // namespace Flags

struct Info {
    SMLK_UINT32     flags;
    SMLK_FLOAT      speed;
    SMLK_FLOAT      heading;
    SMLK_FLOAT      altitude;
    SMLK_DOUBLE     latitude;
    SMLK_DOUBLE     longitude;
    SMLK_UINT64     timestamp;
};
};  // namespace GNSS

namespace telephony {
namespace channel {
    static const SMLK_UINT8     SL_PUBLIC           = 0x00;
    static const SMLK_UINT8     SL_PRIVATE          = 0x01;
    static const SMLK_UINT8     SL_INVALID          = 0xFF;
};  // namespace channel

namespace status {
    static const SMLK_UINT8     SL_DISCONNECTED     = 0x00;
    static const SMLK_UINT8     SL_CONNECTED        = 0x01;
    static const SMLK_UINT8     SL_INVALID          = 0xFF;
};  // namespace status

struct Info {
    SMLK_UINT8      channel;
    SMLK_UINT8      type;
    SMLK_UINT8      status;
    std::string     interface;
};  // struct Info

};  // namespace telephony

};  // namespace ProtocolReporter

};

#pragma pack()

#endif  // #ifndef __GB_17691_H__
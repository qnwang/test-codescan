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
#include <set>
#include <map>
#include <array>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cstring>
#include <ctime>
#include <zlib.h>
#include <json/json.h>
#include <smlk_spn.h>
#include <smlk_log.h>
#include <smlk_timer_new.h>
#include <event.h>
#include <errors_def.h>
#include <message_808.h>
#include <tsp_service_api.h>
#include <vehicle_data_api.h>
#include <smartlink_sdk_tel.h>
#include <smartlink_sdk_mcu.h>
#include <smartlink_sdk_location.h>
#include <smartlink_sdk_tel.h>
#include <collection_service.h>
#include <smartlink_sdk_sys_power.h>
#include <location_manager.h>
#include "smartlink_sdk_sys_property_def.h"
#include "smartlink_sdk_sys_property.h"
#include "smartlink_sdk_sys_time.h"
#ifdef SL_SIMULATOR
#include <cstdio>
#include <random>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <sys/select.h>
#endif

#define M_SL_SMARTLINK_DIR          "/userdata/smartlink"
#define M_SL_SMARTLINK_CONFIG_DIR   M_SL_SMARTLINK_DIR "/config"
#define M_SL_SMARTLINK_DAQ_CFG      M_SL_SMARTLINK_CONFIG_DIR "/smlk_daq.json"
#define M_SL_SMARTLINK_DAQ_TSP_CFG  "/userdata/smartlink/config/tsp_smlk_daq.json"
#define CYCLE_TIMEOUT_COUNT         10
#define LOW_POWER_VOLTAGE_LIMIT     21
#define VOLTAGE_V_TO_MV             1000
#define DM1_NEED_EXTRA_DATA         1
#define DM1_SIGNAL_CYCLE            1    // second
#define KM_PER_HOUR_TO_M_PER_S(speed) ((speed) * 1000.0 / 60.0 / 60.0)
#define INVALID_TIRE_LOCATION_VALUE (0xFF)  // define by can martix
#define M_PER_S_TO_KM_PER_HOUR      (3.6)
#define RTC_TIME_SOURCE             (5) // TimeSource::E_TIME_SOURCE_RTC
#define DIRECT_TYPE              (1)
#define INDIRECT_TYPE            (0)
#define INVALID_TYPE             (3)
#pragma pack(1)
typedef struct
{
    SMLK_UINT8  src;
    SMLK_UINT8  fmi;
    SMLK_UINT32 spn;
} FaultCodeT;

typedef struct {
    SMLK_UINT8  count;
    FaultCodeT  codes[0];
} FaultCodesT;

typedef struct {
    SMLK_UINT32 mask:8;
    SMLK_UINT32 flag:8;
    SMLK_UINT32 :8;
    SMLK_UINT32 :8;
} ExtraFault;

struct RawDTC {
    SMLK_UINT8  Byte0;
    SMLK_UINT8  Byte1;
    SMLK_UINT8  Byte2;
    SMLK_UINT8  Byte3;
};

struct RawDM1 {
    SMLK_UINT8  can_channel;
    SMLK_UINT8  can_src;

    SMLK_UINT8  lamp_status_protect:2;
    SMLK_UINT8  lamp_status_amber:2;
    SMLK_UINT8  lamp_status_red_stop:2;
    SMLK_UINT8  lamp_status_malfunction_indicator:2;
    SMLK_UINT8  lamp_status_reserved;
    RawDTC      dtcs[0];
};

#pragma pack()

using namespace smartlink;

namespace Protocol {
    static const SMLK_UINT8     SL_808  = static_cast<SMLK_UINT8>(ProtoclID::E_PROT_JTT808);
}

// internal flags
namespace InternalFlags {
    static const SMLK_UINT32    SL_FLAG_SYSTIME_SER        = 0x00000001;
    static const SMLK_UINT32    SL_FLAG_SYSTIME_SYN        = 0x00000002;
    static const SMLK_UINT32    SL_FLAG_COMPRESS           = 0x00000004;
    static const SMLK_UINT32    SL_FLAG_IGNON              = 0x00000008;
    static const SMLK_UINT32    SL_FLAG_ETC2_RECEIVED      = 0x00000010;
    static const SMLK_UINT32    SL_FLAG_SET_SIGNAL_INVALID = 0x00000020;

    static const SMLK_UINT32    SL_FLAG_PERIOD_WAKEUP      = 0x00000100;
    static const SMLK_UINT32    SL_FLAG_TEL_READY          = 0x00001000;
    static const SMLK_UINT32    SL_FLAG_CAN_NORMAL         = 0x00010000;
    static const SMLK_UINT32    SL_FLAG_DIRECT_CONTROL     = 0x00100000;
    static const SMLK_UINT32    SL_FLAG_DAY_CHANGE         = 0x01000000;
    static const SMLK_UINT32    SL_FLAG_TRIP_START         = 0x10000000;
}

#ifdef VEHICLE_VKA
namespace VKaBackupFlags {
    static const SMLK_UINT32    SL_FLAG_USE_VEHICLE_SPEED = 0x00000001;
}
#endif
namespace TimeSyncSourceFlag {
    static const SMLK_UINT32    SL_FLAG_SYSTEM_START_SYNC               = 0x00000001;
    static const SMLK_UINT32    SL_FLAG_SYSTEM_WAKEUP_SYNC              = 0x00000002;
    static const SMLK_UINT32    SL_FLAG_SYSTEM_GNSS_ALREADY_SYNC        = 0x00000004;
}

namespace TimerFlags {
    static const SMLK_UINT32    SL_FLAG_0F3C_TIMER_START     = 0x00000001;
    static const SMLK_UINT32    SL_FLAG_0F3B_TIMER_START     = 0x00000010;
    static const SMLK_UINT32    SL_FLAG_0200_TIMER_START     = 0x00000100;
    static const SMLK_UINT32    SL_FLAG_0F3B_30S_TIMER_START = 0x00001000;
    static const SMLK_UINT32    SL_FLAG_CACHE_TIMER_START    = 0x00010000;
}

namespace LastFrameCollectFlags{
    static const SMLK_UINT32    SL_0F3B_1S_COLLECT   = 0x00000001;
    static const SMLK_UINT32    SL_0F3B_30S_COLLECT  = 0x00000010;
    static const SMLK_UINT32    SL_0200_COLLECT      = 0x00000100;
    static const SMLK_UINT32    SL_0F3C_COLLECT      = 0x00001000;
}


/*
vehicle::SignalID::ClutchSwitch 离合
                BrakeSwitch  制动脚踏开关
                TransmissionCurrentGear 变速器空挡
                ParkingBrakeSwitch 驻车制动
                TransmissionMode EP模式
*/
namespace VehicleSwitchBitMask {
    static const SMLK_UINT8    SL_PARK_BRAKE_SWITCH           = 0;  // 驻车制动开关 关：0 开：1 Parking Brake Switch  18FEF100
    static const SMLK_UINT8    SL_CLUTCH_SWITCH               = 1; // 离合开关 踩：1 离合开关状态  Clutch Switch  18FEF100
    static const SMLK_UINT8    SL_BRAKE_SWITCH                = 2; // 制动脚踏开关 踩：1 离合开关状态  制动开关  Brake switch  18FEF100
    static const SMLK_UINT8    SL_ACC_SWITCH                  = 3; // 1:点火开关位置 ACC ON AD采样  点火开关  IGON
    static const SMLK_UINT8    SL_ABS_SWITCH                  = 4; // ABS介入 介入:1 BS 介入信号  ASR brake control active  18F0010B
    static const SMLK_UINT8    SL_TCS_SWITCH                  = 5; // TCS介入 介入:1    TCS 介入信号  ASR engine control active  18F0010B
    static const SMLK_UINT8    SL_CURRENT_GEAR_SWITCH         = 6; // 变速器空挡： 开:1 变速器空挡  Current gear/空档开关  18F00503
    static const SMLK_UINT8    SL_TRILER_CONNECT_SWITCH       = 7; // 挂车连接： 有效:1 挂车连接上信号 Trailer connected 18FED921
    static const SMLK_UINT8    SL_DOOR_OPEN_SWITCH            = 8; // 车门关信号 有效:1 车门关信号 Open Status of Door 18FDA5EC
    static const SMLK_UINT8    SL_FUEL_LEAK_SWITCH_BIT0       = 9; // 燃油泄露 no warning:00 waring :01 燃油泄漏报警  Fuel Leakage Alert  18FE0217
    static const SMLK_UINT8    SL_FUEL_LEAK_SWITCH_BIT1       = 10; // 燃油泄露 reserved:10 not care:11
    static const SMLK_UINT8    SL_EP_MODE_SWITCH_BIT0         = 11; // EP模式表 eco:00 power:01         EP 模式表  Transmission Mode 1  0C010027
    static const SMLK_UINT8    SL_EP_MODE_SWITCH_BIT1         = 12; // EP模式表 reserve:10 invalid :11
    static const SMLK_UINT8    SL_CRUISE_ACTIVE_SWITCH        = 13; // 巡航激活 有效:1 未激活:0          巡航激活状态  Cruise Control Active  18FEF100
    static const SMLK_UINT8    SL_TRAILER_PARKE_BRAKE_SWITCH  = 14; // 挂车驻车 有效:1 关闭:0          挂车驻车状态 Trailer Parking BrakeSwitch 18FED921
    static const SMLK_UINT8    SL_REVERSE_GEAR_SWITCH         = 15; // 倒挡开关状态 有效:1 关闭: 0     倒挡开关状态  Reverse Gear Switch  18FF0221
}

// DismateTbox events bit mask
namespace DismateTboxMask {
    static const SMLK_UINT8    SL_MAIN_POWER_ABSENT        = 0;
    static const SMLK_UINT8    SL_GNSS_ANTENNA_ABSENT      = 1;
    static const SMLK_UINT8    SL_TEL_MAIN_ANTENNA_ABSENT  = 2;
    static const SMLK_UINT8    SL_TEL_SLAVE_ANTENNA_ABSENT = 3;
}

// message types define
namespace MessageType {
    static const SMLK_UINT8     SL_LOCATION             = 0x01;
    static const SMLK_UINT8     SL_BUCKET               = 0x02;
    static const SMLK_UINT8     SL_STATISTICS           = 0x03;
    static const SMLK_UINT8     SL_EVENT                = 0x04;
    static const SMLK_UINT8     SL_BUCKET_30S           = 0x05;
    static const SMLK_UINT8     SL_FAULT                = 0x06;
    static const SMLK_UINT8     SL_LOCATION_RSP         = 0xA1;
    static const SMLK_UINT8     SL_COMPRESSED_BUCKET    = 0xA2;
    static const SMLK_UINT8     SL_FAULT_DM1            = 0xA3;
}

// report frequency define
namespace Frequency {
    static const SMLK_UINT8     SL_100MS                = 0x01;
    static const SMLK_UINT8     SL_500MS                = 0x02;
    static const SMLK_UINT8     SL_1S                   = 0x03;
    static const SMLK_UINT8     SL_5S                   = 0x04;
    static const SMLK_UINT8     SL_10S                  = 0x05;
    static const SMLK_UINT8     SL_30S                  = 0x06;
    static const SMLK_UINT8     SL_1M                   = 0x07;
    static const SMLK_UINT8     SL_2M                   = 0x08;
}

// compress result define
namespace Compress {
    static const SMLK_UINT8     SL_RC_SUCCESS           = 0x00;
    static const SMLK_UINT8     SL_RC_EFAILED           = 0xFF;
    static const SMLK_UINT8     SL_RC_EPARAM            = 0x01;
}

namespace DriveEventFlag {
    static const SMLK_UINT8     SL_EVENT_START          = 0x00;
    static const SMLK_UINT8     SL_EVENT_STOP           = 0xFF;
}

namespace FaultEventMask {
    static const SMLK_UINT8     SL_EVENT_DM1            = 0x01;
    static const SMLK_UINT8     SL_EVENT_ENG            = 0x02;
    static const SMLK_UINT8     SL_EVENT_UDS            = 0x04;
    static const SMLK_UINT8     SL_EVENT_ABN            = 0x08;
}

namespace FaultEventFalg {
    static const SMLK_UINT8     SL_EVENT_START          = 0x00;
    static const SMLK_UINT8     SL_EVENT_STOP           = 0xFF;
}

namespace AlarmMask {
    static const SMLK_UINT32    SL_URGENT               = 0x00000001;
    static const SMLK_UINT32    SL_SPEEDING             = 0x00000002;
    static const SMLK_UINT32    SL_FATIGUE_DRIVING      = 0x00000004;
    static const SMLK_UINT32    SL_WARNNING             = 0x00000008;
    static const SMLK_UINT32    SL_GNSS_FAULT           = 0x00000010;
    static const SMLK_UINT32    SL_GNSS_ANTENNA         = 0x00000020;
    static const SMLK_UINT32    SL_GNSS_SHORT           = 0x00000040;
    static const SMLK_UINT32    SL_POWER_UNDER_VOLTAGE  = 0x00000080;

    static const SMLK_UINT32    SL_OPEN_DOOR_ILLEGAL    = 0x80000000;           // how to
}

namespace TireMask {
    static const SMLK_UINT8     SL_TIRE_EXT             = 0x01;
}

namespace TirePressureThreshold {
namespace Original {
    static const SMLK_UINT8     SL_PRESSURE_OVER        = 0x01;
    static const SMLK_UINT8     SL_PRESSURE_NORMAL      = 0x02;
    static const SMLK_UINT8     SL_PRESSURE_UNDER       = 0x03;
}

namespace Report {
    static const SMLK_UINT8     SL_PRESSURE_NORMAL      = 0x00;
    static const SMLK_UINT8     SL_PRESSURE_OVER        = 0x01;
    static const SMLK_UINT8     SL_PRESSURE_UNDER       = 0x02;
    static const SMLK_UINT8     SL_PRESSURE_INVALID     = 0x03;
}
}

namespace EngineWait2StartLamp {
    static const SMLK_UINT8     SL_OFF                  = 0x00;
    static const SMLK_UINT8     SL_ON                   = 0x01;
    static const SMLK_UINT8     SL_ERROR                = 0x02;
    static const SMLK_UINT8     SL_INVALID              = 0x03;
}

namespace TrailerConnectedSignal {
    static const SMLK_UINT8     SL_TRAILER_NOT_CONNECTED = 0x00;
    static const SMLK_UINT8     SL_TRAILER_CONNECTED     = 0x01;
}

namespace DoorOpenSignal {
    static const SMLK_UINT8     SL_DOOR_CLOSED       = 0x00;
    static const SMLK_UINT8     SL_DOOR_OPEN         = 0x01;
}

namespace StatisticsMask {
    static const SMLK_UINT8     SL_TRIP_FINISHED        = 0x40;
    static const SMLK_UINT8     SL_ACROSS_DAY           = 0x80;
}

namespace Systime {
    static const SMLK_UINT32    SL_SYSTIME_SER_OK       = 0x00000001;
    static const SMLK_UINT32    SL_SYSTIME_SYN_OK       = 0x00000002;
}

namespace Location {
    static const SMLK_UINT32    SL_SERVICE_READY        = 0x00000001;
    static const SMLK_UINT32    SL_SERVICE_EXIT         = 0x00000002;
    static const SMLK_UINT32    SL_LOCATION_CHANGED     = 0x00000004;
    static const SMLK_UINT32    SL_SATELLITE_CHANGED    = 0x00000008;
    static const SMLK_UINT32    SL_ANTENNA_CHANGED      = 0x00000010;
}

namespace PowerState {
    static const SMLK_UINT32    SL_WAKEUP               = 0x00000001;
    static const SMLK_UINT32    SL_SLEEPING             = 0x00000002;
    static const SMLK_UINT32    SL_SLEEPING_CANCEL      = 0x00000003;
    static const SMLK_UINT32    SL_STATE_CHANGED        = 0x00010000;
    static const SMLK_UINT32    SL_SOURCE_CHANGED       = 0x00100000;

    static const SMLK_UINT32    SL_INVALID              = 0xFFFFFFFF;
}

namespace PowerLevel {
    static const SMLK_UINT8     SL_OVER_VOLTAGE         = 0x00;
    static const SMLK_UINT8     SL_UNDER_VOLTAGE        = 0x01;
    static const SMLK_UINT8     SL_NORMAL               = 0x02;
}

namespace MCU {
    static const SMLK_UINT32    SL_SERVICE_READY        = 0x00000001;
    static const SMLK_UINT32    SL_IGN_NOTIFY           = 0x00000002;
    static const SMLK_UINT32    SL_CAN_STATUS_NOTIFY    = 0x00000003;
}

namespace Tel {
namespace Antenna {
    static const SMLK_UINT8     SL_INVALID              = 0xFF;
    static const SMLK_UINT8     SL_ABSENT               = 0x01;
    static const SMLK_UINT8     SL_PRESENT              = 0x02;
    static const SMLK_UINT8     SL_SHORT                = 0x03;
}
    static const SMLK_UINT32    SL_ANTENNA_CHANGED      = 0x00000001;
    static const SMLK_UINT32    SL_SIGNAL_CHANGED       = 0x00000002;
}

namespace IgnState {
    static const SMLK_UINT32    SL_OFF                  = 0x000000000;
    static const SMLK_UINT32    SL_ON                   = 0x000000001;
}

namespace CanBusState {
    static const SMLK_UINT32    CAN_NORMAL                  = 0x000000000;
    static const SMLK_UINT32    CAN_SLEEP                   = 0x000000001;
    static const SMLK_UINT32    CAN_AUTHENTICATING          = 0x000000002;
    static const SMLK_UINT32    CAN_FAILED                  = 0x000000003;
}

namespace DismantleReport {
    static const SMLK_UINT32    EVENT_NOT_REPORT               = 0;
    static const SMLK_UINT32    EVENT_REPORT                   = 1;
}

namespace DoorSignalStatus {
    static const SMLK_UINT8     SL_DOOR_SIGNAL_NOT_RECEIVED       = 0;               // value
    static const SMLK_UINT8     SL_DRIVER_DOOR_SIGNAL_RECIEVED    = 1;               // bitmask
    static const SMLK_UINT8     SL_PAS_DOOR_SIGNAL_RECEIVED       = 2;               // bitmask
    static const SMLK_UINT8     SL_ALL_DOOR_SIGNAL_RECEIVED       = 3;               // value
};

namespace GpsAndTelStatusCheck {
    static const SMLK_UINT8    GPG_NEED_CHECK      = 0x01;
    static const SMLK_UINT8    TEL_NEED_CHECK      = 0x02;
    static const SMLK_UINT8    GPS_AND_TEL_NEED_CHECK      = 0x03;
}

namespace InternalStruct {

    namespace LocatingFlags {
        static const SMLK_UINT32    SL_FIXED                = 0x00000001;
        static const SMLK_UINT32    SL_FIXED_GPS            = 0x00000002;
        static const SMLK_UINT32    SL_FIXED_GLONASS        = 0x00000004;
        static const SMLK_UINT32    SL_FIXED_BDS            = 0x00000008;
        static const SMLK_UINT32    SL_FIXED_GSNS           = 0x00000010;

        static const SMLK_UINT32    SL_ANTENNA_PRESENT      = 0x00000100;
        static const SMLK_UINT32    SL_ANTENNA_SHORT        = 0x00000200;
    }

    struct LocatingInf {
        SMLK_UINT32 flags;                                                      // fix with the locating flags
        SMLK_FLOAT  speed;
        SMLK_FLOAT  heading;
        SMLK_FLOAT  altitude;
        SMLK_DOUBLE latitude;
        SMLK_DOUBLE longitude;
        SMLK_UINT64 utc;
    };

    class DriveEvent : public IMessage {
    public:
        DriveEvent()
            : m_status(0)
            , m_actual_minimum_soc(vehicle::g_vehicle_signals.find(vehicle::SignalID::ActualMinimumSOC)->second)
        {}
        virtual ~DriveEvent()
        {}

        virtual SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &) { return SL_SUCCESS; }
        virtual SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &) { return SL_SUCCESS; }
    public:
        SMLK_UINT32                 m_status;
        SMLK_UINT32                 m_type; // for accident type
        vehicle::SignalVariant      m_actual_minimum_soc;
        TireInfo                    m_tire;
        smartlink::LocationInfo     m_location;
    };

    class TireInfo : public IMessage {
    public:
        TireInfo()
            : m_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::TireTemperature)->second)
            , m_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::TirePressure)->second)
            , m_extend_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::TirePressureExtendedRange)->second)
        {}
        virtual ~TireInfo()
        {}

        virtual SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &) { return SL_SUCCESS; }
        virtual SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &) { return SL_SUCCESS; }
    public:
        vehicle::SignalVariant                  m_temperature;
        vehicle::SignalVariant                  m_pressure;
        vehicle::SignalVariant                  m_extend_pressure;
        std::chrono::steady_clock::time_point   m_expired_at;
    };
}

namespace {
    static const SMLK_UINT32    SL_TIMER_1S                     = 0x00000001;           // 1s bucket report timer index
    static const SMLK_UINT32    SL_TIMER_30S                    = 0x00000002;           // 30s bucket report timer index
    static const SMLK_UINT32    SL_TIMER_REPORT                 = 0x00000003;           // location report timer index
    static const SMLK_UINT32    SL_TIMER_STATISTICS             = 0x00000004;           // statistics information timer index
    static const SMLK_UINT32    SL_TIMER_100MS                  = 0x00000005;           // 100ms chached data report
    static const SMLK_UINT32    SL_TIMER_1S_CACHE               = 0x00000006;           // 1s collect all data to be cached
    static const SMLK_UINT32    SL_TIMER_STATISTICS_STORE       = 0x00000007;           // statistics information backup to sql db timer index
    static const SMLK_UINT32    SL_TIMER_DM1                    = 0x00000008;           // DM1 timerout timer index


    static const int            SL_QUEUE_GET_TIMEOUT_MS         = 200;                  // timeout millisecond to get item from queue

    static const SMLK_UINT32    SL_STATISTICS_REPORT_PERIOD_DEF = 600;          // statistics report default period
    static const SMLK_UINT32    SL_STATISTICS_STORE_PERIOD_DEF  = 300;          // statistics data store default period
    static const SMLK_UINT32    SL_LOCATION_REPORT_PERIOD_DEF   = 30;           // location report default period
    static const SMLK_UINT32    SL_LOCATION_REPORT_REPORT_EMG   = 10;           // location report emergency period
    static const SMLK_UINT32    SL_TRIGGER_PERIOD_30S_DEF       = 30;          // bucket 30s trigger period default 30s
    static const SMLK_UINT32    SL_TRIGGER_PERIOD_1S_DEF        = 1000;           // bucket 1s trigger period default 1s
    static const SMLK_UINT32    SL_WAKEUP_REPORT_COUNT_DEF      = 4;            // wakeup report count while located status is not valid
    static const SMLK_UINT32    SL_WAKEUP_REPORT_PERIOD_DEF     = 7200;         // wakeup report period default 7200s
    static const SMLK_UINT32    SL_PERIOD_TRIGGER_POINT_MS      = 200;          // period trigger ms point (0 ~ 999)
    static const SMLK_UINT32    SL_PERIOD_TIRE_VALID            = 30000;        // period of tire information valid
    static const SMLK_UINT32    SL_CYCLES_SIGNAL_VAL_VALID      = 10;           // cycles of signal value still valid if can not receive signal
    static const SMLK_UINT8     VOL_LEVEL_FLAG_DEFAULT_VALUE    = 2;
    static const SMLK_FLOAT     VOL_LOW_LEVEL_LIMIT_MV          = 21*1000 ;
    static const SMLK_UINT16    MAX_ETHERNET_SEND_SIZE          = 1500;

#ifdef PLATFORM_ARM
    static const std::size_t    SL_PLATFORM_MAX_BYTES           = 32;
    static const std::size_t    SL_PLATFORM_MAX_CHARS           = 160;
#endif

    static const std::size_t    SL_COMPRESS_BUFFER_SZ   = 16384;                // compress block buffer size

    static const SMLK_UINT8     SL_DRIVE_EVENT_REV              = 0x00;

    static const SMLK_UINT32    SL_TIMEZONE_OFFSET              = 8 * 3600;     // GMT +8

    static const std::string    SL_PROCESS_NAME = "DAQ";

    static const std::string    DIRECT_NETWORK_1("7925100-88P-C00");
    static const std::string    DIRECT_NETWORK_2("7925100C88P-C00");
    static const std::string    DIRECT_NETWORK_3("7925100D88P-C00");
    static const std::string    INDIRECT_NETWORK("7925100B88P-C00");

    static const std::set<SMLK_UINT8>   g_chache_pos_ids = {
        M_SL_EVID_SHARP_SLOW_DOWN,
        M_SL_EVID_SHARP_SPEED_UP,
        M_SL_EVID_SHARP_TURN,
    };

    static const std::set<SMLK_UINT8>   g_upload_immediately_ids = {
        M_SL_EVID_LONG_LONG_PARKING,
        M_SL_EVID_TIREDNESS_DRIVING
    };

    static const std::set<SMLK_UINT8>   g_upload_immediately_and_delay_ids = {
        M_SL_EVID_ACCIDENT
    };
}

#ifdef SL_SIMULATOR
namespace SimulatorEvent {
    static const SMLK_UINT16    SL_SM_EVENT_START       = 0xFE01;               // drive event start
    static const SMLK_UINT16    SL_SM_EVENT_STOP        = 0xFE02;               // drive event stop
    static const SMLK_UINT16    SL_SM_FAULT_OCCUR       = 0xFE11;               // fault occur
    static const SMLK_UINT16    SL_SM_FAULT_CLEAR       = 0xFE12;               // fault clear
};
#endif

CollectionService::CollectionService()
    : m_flags(IService::Uninitialized)
    , m_internal_flags(0)
    , m_dismantle_tbox_status(0)
    , m_report_period_default(SL_LOCATION_REPORT_PERIOD_DEF)
    , m_report_period_emergency(SL_LOCATION_REPORT_REPORT_EMG)
    , m_report_period(m_report_period_default)
    , m_seq(0)
    , m_bucket_num_1s(10)
    , m_bucket_num_30s(10)
    , m_wakeup_report_cnt(0)
    , m_wakeup_report_period(SL_WAKEUP_REPORT_PERIOD_DEF)
    , m_trigger_period_30s(SL_TRIGGER_PERIOD_30S_DEF)
    , m_trigger_period_1s_ms(1000)
    , m_time_source(RTC_TIME_SOURCE)
    , m_time_sync_flags(0)
    , m_timeSyncGap_ms(0)
    , m_upload_period_1s_ms(10000)
    , m_ig_on_to_off(false)
    , m_ig_sample_count(0)
    , m_statistics_report_period(SL_STATISTICS_REPORT_PERIOD_DEF)
    , m_statistics_store_period(SL_STATISTICS_STORE_PERIOD_DEF)
    , m_collect_last_frame_flag(0)
    , m_door_signal_status(DoorSignalStatus::SL_DOOR_SIGNAL_NOT_RECEIVED)
#ifdef VEHICLE_VKA
    , m_vka_flags(0)
#endif
{
    m_frequency_value.emplace(Frequency::SL_1S, m_trigger_period_1s_ms);
    m_frequency_value.emplace(Frequency::SL_30S, m_trigger_period_30s);
}

CollectionService::~CollectionService()
{
    Stop();
}

/******************************************************************************
 * NAME: Init
 *
 * DESCRIPTION: Init function for the collection service, you may finish all
 *              the initialization work here
 *
 * PARAMETERS:
 *
 * RETURN:
 *      SL_SUCCESS on success, and for now no error will return
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 CollectionService::Init()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Initialized ) {
        SMLK_LOGI("service already initialized, do nothing");
        return SL_SUCCESS;
    }

    // this should be configurable parameter, defaulty set to compress
    m_internal_flags |= InternalFlags::SL_FLAG_COMPRESS;

    std::ifstream   ifs(M_SL_SMARTLINK_DAQ_CFG);
    if ( ifs.is_open() ) {
        Json::Value root;
        Json::CharReaderBuilder builder;
        JSONCPP_STRING errs;

        if ( !parseFromStream(builder, ifs, &root, &errs) ) {
            SMLK_LOGE("fail to parse config file!!! %s", std::string(errs).c_str());
        } else {
            // those configurations in 'debug' field is just for test, for the product version we should not have those configurations
            auto debug = root.get("debug", Json::Value::null);
            if ( !debug.isNull() ) {
                SMLK_UINT32     wakeup_report_period_s  = debug.get("wakeup_report_period", SL_WAKEUP_REPORT_PERIOD_DEF).asUInt();
                SMLK_UINT32     trigger_period_30s_s    = debug.get("trigger_30s_period",   SL_TRIGGER_PERIOD_30S_DEF).asUInt();
                SMLK_UINT32     trigger_period_statistics_s    = debug.get("trigger_statistics_period",   SL_STATISTICS_REPORT_PERIOD_DEF).asUInt();

                SMLK_LOGI("[CFG] [locationtest]wakeup_report_period_s:    %u", wakeup_report_period_s);
                SMLK_LOGI("[CFG] trigger_period_30s_s:      %u", trigger_period_30s_s);
                SMLK_LOGI("[CFG] trigger_period_statistics_s:   %u", trigger_period_statistics_s);

                if ( 30 <= wakeup_report_period_s && wakeup_report_period_s <= 36000 ) {
                    SMLK_LOGI("[locationtest] wakeup_report_period set to %u", wakeup_report_period_s);
                    m_wakeup_report_period = wakeup_report_period_s;
                }

                if ( 3 <= trigger_period_30s_s && trigger_period_30s_s <= 300 ) {
                    SMLK_LOGI("trigger_period_30s set to %u", trigger_period_30s_s);
                    m_trigger_period_30s = trigger_period_30s_s;
                }

                if ( 60 <= trigger_period_statistics_s && trigger_period_statistics_s <= 600 ) {
                    SMLK_LOGI("trigger_period_statistics_s set to %u", trigger_period_statistics_s);
                    m_statistics_report_period = trigger_period_statistics_s;
                }
            }
        }
    }
    if ( SMLK_RC::RC_OK !=  m_sqldb.Init() ) {
        SMLK_LOGE("sqldebug m_sqldb.Init() failed, return.\n");
        return SL_EFAILED;
    }
    SMLK_LOGD("sqldebug Init() OK.");

    SMLK_LOGD("create timer interface!!!");
#if 0
    m_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 1000>>(
            [this](SMLK_UINT32 index){

                MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_1S, 0, 0, 0, index);
                Post(head, nullptr, 0);

                if ( 0 == index % m_trigger_period_30s ) {
                    MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_30S, 0, 0, 0, 0);
                    Post(head, nullptr, 0);
                }

                if ( 0 == index % m_report_period ) {
                    MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_REPORT, 0, 0, 0, 0);
                    Post(head, nullptr, 0);
                }

                if ( 0 == index % m_statistics_report_period ) {
                    MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_STATISTICS, 0, 0, 0, 0);
                    Post(head, nullptr, 0);
                }

                if ( 0 == index % m_statistics_store_period ) {
                    MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_STATISTICS_STORE, 0, 0, 0, 0);
                    Post(head, nullptr, 0);
                }
            }
        );
#endif

    SMLK_LOGD("create timer !!!");

    m_dm1_timeout_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 1000>>(
            [this](SMLK_UINT32 index) {
                PostDM1(MsgHead(M_SL_MSGID_DATA_DM1_TIMER, 0, 0, 0, 0, 0), nullptr, 0);
            }
        );

    // dueto 8130 config change 0f3b should support 100ms resolution
    m_0f3b_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 100>>(
            [this](SMLK_UINT32 index){
                if ( 0 == (index*100) % (m_trigger_period_1s_ms) ) {
                    if (true == m_ig_on_to_off) {
                        SMLK_LOGI("1Scall direct sample!!!");
                        m_ig_sample_count++;
                        if ( Check1sReportContidion() ) {
                            SamplingPeriod1S();
                        }
                        if (m_ig_sample_count >= 2) {
                            m_ig_on_to_off = false;
                            m_ig_sample_count = 0;
                        }
                    } else {
                        SMLK_LOGI("1Scall post !!!");
                        MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_1S, 0, 0, 0, index);
                        Post(head, nullptr, 0);
                    }
                }
            }
        );
    m_0f3b_30s_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 100>>(
            [this](SMLK_UINT32 index){
                if ( 0 == index % (m_trigger_period_30s * 10)) {
                    MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_30S, 0, 0, 0, 0);
                    Post(head, nullptr, 0);
                }
            }
        );
    m_0f3c_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 1000>>(
            [this](SMLK_UINT32 index){
                if ( 0 == index % m_statistics_report_period ) {

                    MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_STATISTICS, 0, 0, 0, 0);
                    Post(head, nullptr, 0);
                }

                if ( 0 == index % m_statistics_store_period ) {
                    MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_STATISTICS_STORE, 0, 0, 0, 0);
                    Post(head, nullptr, 0);
                }
            }
        );
    m_0200_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 100>>(
            [this](SMLK_UINT32 index){
                if ( 0 == index % (m_report_period *10)) {
                    MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_REPORT, 0, 0, 0, 0);
                    Post(head, nullptr, 0);
                }
            }
        );
    SMLK_LOGD("create timer end!!!");

    // for now, only protocol 808 is supported
    m_protocols.push_back(Protocol::SL_808);

    m_protocol_messages[Protocol::SL_808][MessageType::SL_LOCATION]             = std::make_shared<MessageLocation808>();
    m_protocol_messages[Protocol::SL_808][MessageType::SL_BUCKET]               = std::make_shared<MessageBucket808>();
    m_protocol_messages[Protocol::SL_808][MessageType::SL_STATISTICS]           = std::make_shared<MessageStatistics808>();
    m_protocol_messages[Protocol::SL_808][MessageType::SL_EVENT]                = std::make_shared<MessageEvent808>();
    m_protocol_messages[Protocol::SL_808][MessageType::SL_BUCKET_30S]           = std::make_shared<MessageBucket30S808>();
    m_protocol_messages[Protocol::SL_808][MessageType::SL_FAULT]                = std::make_shared<MessageFault808>();
    m_protocol_messages[Protocol::SL_808][MessageType::SL_LOCATION_RSP]         = std::make_shared<MessageLocationRsp808>();
    m_protocol_messages[Protocol::SL_808][MessageType::SL_COMPRESSED_BUCKET]    = std::make_shared<MessageCompressedBucket808>();
    m_protocol_messages[Protocol::SL_808][MessageType::SL_FAULT_DM1]            = std::make_shared<MessageDM1808>();

    static const std::set<vehicle::SignalID>    tire_signals = {
        vehicle::SignalID::TirePressure,
        vehicle::SignalID::TireTemperature,
        vehicle::SignalID::TireStatus,
        vehicle::SignalID::TireExtrentPressureSupport,
        vehicle::SignalID::TireAirLeakageRate,
        vehicle::SignalID::TirePressureThresholdDetectionOriginal,
    };

    static const std::set<vehicle::SignalID>  tire_ext_signals = {
        vehicle::SignalID::TirePressureExtendedRange,
        vehicle::SignalID::TirePressureRequired,
    };

    for ( auto const &pair : vehicle::g_vehicle_signals ) {
        m_cached_signals.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(pair.first),
            std::forward_as_tuple(pair.second)
        );

        auto const it = vehicle::g_vehicle_signal_index.find(pair.first);
        if ( vehicle::g_vehicle_signal_index.cend() == it ) {
            continue;
        }

        if ( tire_signals.end() != tire_signals.find(pair.first) ) {
            m_cached_tires_signals.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(it->second),
                std::forward_as_tuple(std::ref(m_cached_signals.find(pair.first)->second))
            );
        } else if ( tire_ext_signals.end() != tire_ext_signals.find(pair.first) ) {
            m_cached_tires_ext_signals.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(it->second),
                std::forward_as_tuple(std::ref(m_cached_signals.find(pair.first)->second))
            );
        } else {
            m_cached_index_signals.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(it->second),
                std::forward_as_tuple(std::ref(m_cached_signals.find(pair.first)->second))
            );
        }
#ifdef SL_DEBUG
        SMLK_LOGD("SPN[%u] --> INDEX[%u]", m_cached_signals.find(pair.first)->second.ID(), it->second);
#endif
    }

    m_cached_signals.find(vehicle::AlarmFlags)->second = 0; // 报警标志

    //TODO: init Systime service and test time sync

    SMLK_LOGD("init TSP service");
    auto rc = TspServiceApi::getInstance()->Init(ModuleID::E_Module_daq_service);
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to init TSP service API interface!!!");
        return SL_EFAILED;
    }

    std::vector<TspEventId> tsp_events = {
        TspEventId::E_TSP_EVENT_CONNECT_STATE,
        TspEventId::E_TSP_EVENT_LOGIN_STATE,
    };

    SMLK_LOGD("register TSP event callback");
    rc = TspServiceApi::getInstance()->RegEventCB(
        tsp_events,
        [this](TspEventId id, void *data, int len){
             SMLK_LOGI("revceive tsp event %d",id);
            switch(id)
            {
                case TspEventId::E_TSP_EVENT_CONNECT_STATE:
                {
                    TspConnectState* state =  (TspConnectState*)data;
                    SMLK_LOGI("revceive tsp connected state %d",*state);
                    break;
                }
                case TspEventId::E_TSP_EVENT_LOGIN_STATE:
                {
                    TspLoginState* state =  (TspLoginState*)data;
                    SMLK_LOGI("revceive tsp login state %d",*state);
                    break;
                }
                default:
                    break;
            }
        }
    );

    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register TSP service event callback");
        return SL_EFAILED;
    }

    SMLK_LOGD("register TSP message callback");
    rc = TspServiceApi::getInstance()->RegisterMsgCB(
        ProtoclID::E_PROT_JTT808,
        {M_SL_MSGID_PT_LOCATION_REQ},
        [this](IN IpcTspHead &ipc, IN void *data, std::size_t sz){
#ifdef SL_DEBUG
            std::stringstream   ss;
            for ( decltype(ipc.length) index = 0; index < sz; index++ ) {
                ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << ((char *)data)[index];
            }
            SMLK_LOGD("IPC:%s", ss.str().c_str());
#endif
            Post(
                MsgHead(ipc.msg_id, ipc.seq_id, (SMLK_UINT8)ipc.protocol, (SMLK_UINT8)ipc.compress, ipc.length, 0),
                (const SMLK_UINT8 *)data,
                sz
            );
        }
    );
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register TSP service event callback");
        return SL_EFAILED;
    }

    SMLK_LOGD("TSP service registered SUCCESS");


    SMLK_LOGD("[timesyncDebug] register SysTime");
    if (!smartlink_sdk::SysTime::GetInstance()->Init(ModuleID::E_Module_daq_service)){
        SMLK_LOGF("E_Module_daq_service fail to init SysTime service API interface.\n");
        return SL_EFAILED;
    } else {
        m_internal_flags |= InternalFlags::SL_FLAG_SYSTIME_SER;
        std::vector<smartlink_sdk::SysTimeEventId> timer_event_vec;
        timer_event_vec.push_back(smartlink_sdk::SysTimeEventId::E_TIME_EVENT_TIME_CHANGED);
        smartlink_sdk::SysTime::GetInstance()->RegEventCB(timer_event_vec,
            [this](smartlink_sdk::SysTimeEventId event_id, void *data, int len) -> SMLK_UINT8 {
                SMLK_LOGD("***** [timesyncDebug] SysTime event_id = %d, .\n", event_id);
                switch ( event_id )
                {
                    case smartlink_sdk::SysTimeEventId::E_TIME_EVENT_TIME_CHANGED:
                    {
                        smartlink_sdk::SysTimeEventInfo * info = (smartlink_sdk::SysTimeEventInfo*)data;
                        SMLK_LOGI("[timesyncDebug] time source 1:GNSS 2:NTP 3:NITZ 4:TSP 5:RTC");
                        SMLK_LOGI("[timesyncDebug]***** timesync = [%d]", info->timesync);
                        SMLK_LOGI("[timesyncDebug] TimeSource = [%d] ", info->source);
                        SMLK_LOGI("[timesyncDebug] gap us is = [%ld] ", info->us);
                        SMLK_UINT32 new_time_source = (SMLK_UINT32)info->source;
                        SMLK_INT32 change_ms = 0;
                        if (0 != info->us) {
                            change_ms = SMLK_INT32(info->us/1000);
                            SMLK_LOGI("[timesyncDebug] change_ms = [%ld] ", change_ms);
                        }
                        // 恒润: 同优先级的时间源 判断<5s 是可信的; 高优先级校时没有此判断
                        if (info->timesync == 1 )
                        {
                            // 业务数据中的时间, 以TBOX拿到的第一个可信任的授时源为准, 最低标准为NTP
                            if ( smartlink_sdk::TimeSource::E_TIME_SOURCE_NTP  == new_time_source
                                || smartlink_sdk::TimeSource::E_TIME_SOURCE_GNSS  == new_time_source) {
                                SMLK_LOGI("[timesyncDebug] post change_ms = %ld", change_ms);
                                m_time_source = new_time_source;
                                Post(MsgHead(M_SL_MSGID_SYSTIME_SYNC, Systime::SL_SYSTIME_SYN_OK, 0, 0, 0, 0), nullptr, 0);
                                // 从未授时时,先缓存数据,然后在授时后修改缓存的GNSSTimeStamp字段,从缓存中取出所有数据发送
                                // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
                                if ( CheckWhetherTimeSync() ) {
                                    // 负值: 授时后系统时间 < 授时前时间 正值：授时后系统时间 > 授时前系统时间
                                    // 为保持和旧系统获取的时戳 连续 newtp = newSysTime - (m_timeSyncGap_ms)
                                    // 已经授时后 除了 系统唤醒后 和 未gnss 授时, 不再叠加授时差
                                    if ( (0 == (m_time_sync_flags&TimeSyncSourceFlag::SL_FLAG_SYSTEM_WAKEUP_SYNC))
                                            || (0 == m_time_sync_flags&TimeSyncSourceFlag::SL_FLAG_SYSTEM_GNSS_ALREADY_SYNC) ) {
                                        SMLK_LOGI("[timesyncDebug] last m_timeSyncGap_ms [%lld] !!!", m_timeSyncGap_ms);
                                        SMLK_LOGI("[timesyncDebug] change_ms = [%ld] ", change_ms);
                                        m_timeSyncGap_ms += change_ms;
                                        SMLK_LOGI("[timesyncDebug] new m_timeSyncGap_ms  %lld !!!", m_timeSyncGap_ms);
                                        if (0 == (m_time_sync_flags&TimeSyncSourceFlag::SL_FLAG_SYSTEM_WAKEUP_SYNC) ) {
                                            m_time_sync_flags |= TimeSyncSourceFlag::SL_FLAG_SYSTEM_WAKEUP_SYNC;
                                        }
                                    }
                                }
                                if (smartlink_sdk::TimeSource::E_TIME_SOURCE_GNSS  == new_time_source) {
                                    m_time_sync_flags |= TimeSyncSourceFlag::SL_FLAG_SYSTEM_GNSS_ALREADY_SYNC;
                                }
                            }
                        } else {
                            // 同优先级的时间源 判断>5s
                            // 此种授时适用debug手动调同级时间
                            SMLK_LOGI("[timesyncDebug]***** time sync false");
#if 0
                            m_timeSyncGap_ms += change_ms;
                            Post(MsgHead(M_SL_MSGID_SYSTIME_SYNC, Systime::SL_SYSTIME_SYN_OK, 0, 0, 0, change_ms), nullptr, 0);
                            // 如果是ntp授时了, 也视作授时
                            // TODO:add check
                            if (new_time_source > m_time_source) {
                                m_timeSyncGap_ms += change_ms;
                                Post(MsgHead(M_SL_MSGID_SYSTIME_SYNC, Systime::SL_SYSTIME_SYN_OK, 0, 0, 0, change_ms), nullptr, 0);
                            }
#endif
                        }
                    }
                    break;
                    default:
                        SMLK_LOGE("[timesyncDebug] unsupported event id: %d", (SMLK_UINT8)event_id);
                        return 0;
                }
                return 0;
        });
        SMLK_LOGI("[timesyncDebug] SysTime::GetInstance()->Init() OK.\n");
    }
    SMLK_LOGD("[timesyncDebug] register SysTime end ");

    std::vector<SMLK_UINT32>    indexs;
    indexs.reserve(m_cached_index_signals.size());
    for ( auto const &pair : m_cached_index_signals ) {
        indexs.push_back(pair.first);
    }

    SMLK_LOGD("init vehicle data service");
    rc = VehicleDataApi::GetInstance()->Init(ModuleID::E_Module_daq_service);
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to init vehicle data service API interface!!!");
        return SL_EFAILED;
    }

    SMLK_LOGD("register vehicle data changed callback");
    rc = VehicleDataApi::GetInstance()->RegistVehicleDataChangedCB(
        indexs,
        [this](VehicleData data){
//#ifdef SL_DEBUG
            // SMLK_LOGD("[VehicleDatqa] data changed notification: index[%u] valid[%s] value[%f]", data.index, (data.valid ? "YES" : "NO"), data.value);
//#endif
            Post(
                // here we reuse the seq and protocol of head to store spn and valid
                MsgHead(M_SL_MSGID_DATA_CHANGED, data.index, data.valid ? 0x01 : 0x00, 0, sizeof(data.value), 0),
                reinterpret_cast<const SMLK_UINT8 *>(&data.value),
                sizeof(data.value)
            );
        }
    );
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register vehicle data changed callback");
        return SL_EFAILED;
    }
    indexs.clear();
    for ( auto const &pair : m_cached_tires_signals ) {
        indexs.push_back(pair.first);
    }
    SMLK_LOGD("register vehicle data (tire signals) changed callback");
    rc = VehicleDataApi::GetInstance()->RegistVehicleDataChangedCB(
        indexs,
        [this](VehicleData data) {
#ifdef SL_DEBUG
            SMLK_LOGD("[tiredebug] vehicle trigger tires signal callback");
#endif
            Post(
                // here we reuse the seq and protocol of head to store spn and valid
                MsgHead(M_SL_MSGID_DATA_TIRE_NTF, data.index, data.valid ? 0x01 : 0x00, 0, sizeof(data.value), 0),
                reinterpret_cast<const SMLK_UINT8 *>(&data.value),
                sizeof(data.value)
            );
        }
    );
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register tire changed callback");
        return SL_EFAILED;
    }

    indexs.clear();
    for ( auto const &pair : m_cached_tires_ext_signals ) {
        indexs.push_back(pair.first);
    }
    SMLK_LOGD("register vehicle data (tire extended signals) changed callback");
    rc = VehicleDataApi::GetInstance()->RegistVehicleDataChangedCB(
        indexs,
        [this](VehicleData data) {
#ifdef SL_DEBUG
            SMLK_LOGD("[tiredebug] vehicle trigger m_cached_tires_ext_signals callback");
#endif
            Post(
                // here we reuse the seq and protocol of head to store spn and valid
                MsgHead(M_SL_MSGID_DATA_TIRE_NTF, data.index, data.valid ? 0x01 : 0x00, 0, sizeof(data.value), TireMask::SL_TIRE_EXT),
                reinterpret_cast<const SMLK_UINT8 *>(&data.value),
                sizeof(data.value)
            );
        }
    );
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register tire ext changed callback");
        return SL_EFAILED;
    }

    std::vector<DriveEvent> drive_events = {
        DriveEvent::E_DRIVE_EVENT_URGENT_SPEED_DOWN,
        DriveEvent::E_DRIVE_EVENT_OVER_SPEED,
        DriveEvent::E_DRIVE_EVENT_FATIGUE_DRIVING,
        DriveEvent::E_DRIVE_EVENT_MAJOR_ACCIDENT,
        DriveEvent::E_DRIVE_EVENT_TPMS_WARNING,
        DriveEvent::E_DRIVE_EVENT_SOC_LOW_POWER,
        DriveEvent::E_DRIVE_EVENT_DMS_WARNING,
        DriveEvent::E_DRIVE_EVENT_FUEL_TANK_SENSOR_WARNING,
        DriveEvent::E_DRIVE_EVENT_MIX_TANK_ERROR,
        DriveEvent::E_DRIVE_EVENT_REFRIGERATOR_FAILURE
    };
    SMLK_LOGD("register vehicle data drive event callback");
    rc = VehicleDataApi::GetInstance()->RegistDriveEvent(
        drive_events,
        [this](const DriveEvent &id, const std::vector<SMLK_UINT8> &header, const std::vector<SMLK_UINT8> &body) {
            static const std::map<DriveEvent, SMLK_UINT8> events = {
                { DriveEvent::E_DRIVE_EVENT_URGENT_SPEED_DOWN,        M_SL_EVID_SHARP_SLOW_DOWN       },
                { DriveEvent::E_DRIVE_EVENT_OVER_SPEED,               M_SL_EVID_SPEEDING              },
                { DriveEvent::E_DRIVE_EVENT_FATIGUE_DRIVING,          M_SL_EVID_TIREDNESS_DRIVING     },
                { DriveEvent::E_DRIVE_EVENT_MAJOR_ACCIDENT,           M_SL_EVID_ACCIDENT              },
                { DriveEvent::E_DRIVE_EVENT_TPMS_WARNING,             M_SL_EVID_TPMS_TIRE_WARN        },
                { DriveEvent::E_DRIVE_EVENT_SOC_LOW_POWER,            M_SL_EVID_SOC_LOW               },
                { DriveEvent::E_DRIVE_EVENT_DMS_WARNING,              M_SL_EVID_DMS_WARN              },
                { DriveEvent::E_DRIVE_EVENT_FUEL_TANK_SENSOR_WARNING, M_SL_EVID_FUEL_TANK_WARN        },
                { DriveEvent::E_DRIVE_EVENT_MIX_TANK_ERROR,           M_SL_EVID_AGITATOR_FAULT        },
                { DriveEvent::E_DRIVE_EVENT_REFRIGERATOR_FAILURE,     M_SL_EVID_REFRIGERATOR_FAILURE  }
            };

            auto it = events.find(id);
            if ( events.end() == it ) {
                SMLK_LOGE("unsupported event id: %u", static_cast<SMLK_UINT8>(id));
                return;
            }

#ifdef SMLK_SIM_LOCATION
            if ( header.size() < sizeof(DriveEventInfo) ) {
                SMLK_LOGE("header size is too small, at least %u", sizeof(DriveEventInfo));
                return;
            }

            const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(header.data());
            if ( inf->location_num < 2 ) {
                SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
                return;
            }

            auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
            if ( header.size() != expected ) {
                SMLK_LOGE("body size %u is not match with expected: %u", header.size(), expected);
                return;
            }

            std::vector<SMLK_UINT8> all;
            all.reserve(header.size() + body.size());
            all.insert(all.end(), header.cbegin(), header.cend());
            all.insert(all.end(), body.cbegin(), body.cend());

            DriveEventInfo  *mod = reinterpret_cast<DriveEventInfo *>(all.data());

            mod->time   = mod->location_num * 1000;
            for ( decltype(mod->location_num) index = 0; index < mod->location_num; index++ ) {
                mod->location_info[index].latitude  = 118.816758;
                mod->location_info[index].longitude = 31.919299;
                mod->location_info[index].altitude  = 18.3456789;
                mod->location_info[index].flag      = true;
                mod->location_info[index].speed     = 123.456;
                mod->location_info[index].direction = 30.456;
                mod->location_info[index].utc       = (std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) + index - inf->location_num) * 1000;

                auto now = std::chrono::system_clock::now();
                auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

                mod->location_info[index].utc = ms;
            }

            Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, it->second, 0, 0, 0, 0), std::move(all));
#else
            if ( 0 == body.size() ) {
                Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, it->second, 0, 0, 0, 0), header);
            } else {
                std::vector<SMLK_UINT8> all;
                all.reserve(header.size() + body.size());
                all.insert(all.end(), header.cbegin(), header.cend());
                all.insert(all.end(), body.cbegin(), body.cend());
                Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, it->second, 0, 0, 0, 0), std::move(all));
            }
#endif
        }
    );
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register drive event callback");
        return SL_EFAILED;
    }

    SMLK_LOGD("register vehilce data DM1 raw CAN data callback");
    rc = VehicleDataApi::GetInstance()->RegistDM1RawData(
        [this](CAN_ID id, std::vector<SMLK_UINT8> &data) {
#ifdef SL_DEBUG
            // SMLK_LOGD("[dm1test] receive vehicle CAN_ID id  %d ", id);
#endif
            PostDM1(MsgHead(M_SL_MSGID_DATA_DM1_NTF, id, 0, 0, 0, 0), data);
        }
    );
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register drive event callback");
        return SL_EFAILED;
    }

    SMLK_LOGD("init location service");
    // register location service related data
    if ( !smartlink_sdk::Location::GetInstance()->Init(ModuleID::E_Module_daq_service) ) {
        SMLK_LOGE("fail to init location service API interface!!!");
        return SL_EFAILED;
    }

    std::vector<smartlink_sdk::LocationEventId> loc_events = {
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_ISREADY,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_EXIT,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_GNSS_ANTENNA_CHANGED,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_SATELLITE_INFO_CHANGED,
    };

    SMLK_LOGD("register location callback");
    auto return_code = smartlink_sdk::Location::GetInstance()->RegEventCallback(
        loc_events,
        [this](smartlink_sdk::LocationEventId id, void *data, int len){
            MsgHead head(M_SL_MSGID_LOCATOIN_NTF, 0, 0, 0, 0, 0);
            std::vector<SMLK_UINT8> bytes;

            switch ( id ) {
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_ISREADY:
                    head.m_seq |= Location::SL_SERVICE_READY;
                    break;
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_EXIT:
                    head.m_seq |= Location::SL_SERVICE_EXIT;
                    break;
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_GNSS_ANTENNA_CHANGED:
                    {
                        head.m_seq |= Location::SL_ANTENNA_CHANGED;
                        SMLK_LOGD("[GNSS] antenna state changed, len = %d", len);
                        SMLK_UINT8 *id = reinterpret_cast<SMLK_UINT8*>(data);
                        smartlink_sdk::GnssAntennaState state = (smartlink_sdk::GnssAntennaState)(*id);
                        SMLK_LOGD("[GNSS] notify antenna state changed state is = %d",  state);
                        head.m_protocol = (SMLK_UINT32)state;
                    }

                    break;
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED:
                    {
                        if ( sizeof(smartlink_sdk::LocationInfo) != (std::size_t)len ) {
                            SMLK_LOGE("[GNSS] unexpected body length for FIX INFO CHNAGED notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::LocationInfo));
                            return;
                        }
                        smartlink_sdk::LocationInfo     *inf = (smartlink_sdk::LocationInfo *)data;
                        InternalStruct::LocatingInf     loc = {};
                        // if remove atnnea  will report invalid twice and not report any more
                        if ( inf->fix_valid ) {
                            loc.flags   |= InternalStruct::LocatingFlags::SL_FIXED;
                        } else {
                            loc.flags   &= ~InternalStruct::LocatingFlags::SL_FIXED;
                        }
#ifdef SL_DEBUG
                        SMLK_LOGD("[GNSS] loc.flags is %d fied:1 unfixed:0", loc.flags);
#endif
                        loc.speed       = inf->speed;
                        loc.heading     = inf->heading;
                        loc.altitude    = inf->altitude;
                        loc.latitude    = inf->latitude;
                        loc.longitude   = inf->longitude;
                        loc.utc         = inf->utc;

#ifdef SL_DEBUG
#if 0
                        SMLK_LOGD("\tinf->speed:        %f", inf->speed);
                        SMLK_LOGD("\tinf->heading:      %f", inf->heading);
                        SMLK_LOGD("\tinf->altitude:     %f", inf->altitude);
                        SMLK_LOGD("\tinf->latitude:     %f", inf->latitude);
                        SMLK_LOGD("\tinf->longitude:    %f", inf->longitude);
                        SMLK_LOGD("\tinf->utc:          %lu", inf->utc);
#endif
#endif
                        bytes.insert(bytes.end(), (SMLK_UINT8 *)&loc, (SMLK_UINT8 *)(&loc + 1));

                        head.m_length   = sizeof(smartlink_sdk::LocationInfo);
                        head.m_seq |= Location::SL_LOCATION_CHANGED;
                    }
                    break;
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_SATELLITE_INFO_CHANGED:
                    {
#if 0
                        if ( sizeof(smartlink_sdk::GnssSatelliteInfo) > (std::size_t)len ) {
                            SMLK_LOGE("[GNSS] body length too short for SATELLITE INFO CHANGED notification: %d, at least: %d", len, sizeof(smartlink_sdk::GnssSatelliteInfo));
                        }
#endif
                        smartlink_sdk::GnssSatelliteInfo    *inf    = (smartlink_sdk::GnssSatelliteInfo *)data;

                        bytes.push_back(inf->tracked_num);
                        bytes.push_back(inf->visible_num);

                        head.m_seq |= Location::SL_SATELLITE_CHANGED;
                    }
                    break;
                default:
                    SMLK_LOGE("unsupported event id: %u", static_cast<SMLK_UINT32>(id));
                    return;
            }

            Post(head, std::move(bytes));
        }
    );

    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to init register event callback to location service!!! return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    SMLK_LOGD("init sys-power service");
    if ( !smartlink_sdk::SysPower::GetInstance()->Init(ModuleID::E_Module_daq_service) ) {
        SMLK_LOGE("fail to init power service API interface!!!");
        return SL_EFAILED;
    }

    std::vector<smartlink_sdk::SysPowerEventID> power_events = {
        smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_SOURCE_CHANGED,
        smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_STATE_CHANGED,
        smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP,
        smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP,
    };

    SMLK_LOGD("register sys-power event callback");
    return_code = smartlink_sdk::SysPower::GetInstance()->RegEventCB(
        power_events,
        [this](smartlink_sdk::SysPowerEventID id, void *data, int len) {
            MsgHead head(M_SL_MSGID_SYSPOWER_NTF, 0, 0, 0, 0, 0);
            // SMLK_LOGI("[POWER] received sys-power event notification, id = 0x%02X", static_cast<SMLK_UINT8>(id));
            std::vector<SMLK_UINT8> bytes;
            switch ( id )
            {
                case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_STATE_CHANGED:
                    {
                        // it is cycle report 1s
                        // SMLK_LOGI("[POWER] power state changed event received");
                        if ( sizeof(smartlink_sdk::SysPowerStateInfo) != (std::size_t)len ) {
                            SMLK_LOGE("[POWER] body size %d not match expected: %d", len, (int)sizeof(smartlink_sdk::SysPowerStateInfo));
                            return;
                        }

                        auto inf = reinterpret_cast<const smartlink_sdk::SysPowerStateInfo *>(data);

                        head.m_seq          = PowerState::SL_STATE_CHANGED;

                        // this case only use for 0200 voltage low level flag < 21 v
                        SMLK_FLOAT vol = inf->voltage;
                        static SMLK_UINT8 vol_level_flag = VOL_LEVEL_FLAG_DEFAULT_VALUE;
                        // SMLK_LOGI("[POWER] vol_low_limit_mv is %f", VOL_LOW_LEVEL_LIMIT_MV);
                        m_cached_signals.find(vehicle::SignalID::TboxVlotage)->second = vol/1000;

                        if (VOL_LOW_LEVEL_LIMIT_MV > vol && (PowerLevel::SL_UNDER_VOLTAGE != vol_level_flag)) {
                            SMLK_LOGI("[POWER] inf->voltage is %f", vol);
                            vol_level_flag = PowerLevel::SL_UNDER_VOLTAGE;
                            head.m_extra    = PowerLevel::SL_UNDER_VOLTAGE;
                            SMLK_LOGD("[POWER] under voltage ");
                        } else if (VOL_LOW_LEVEL_LIMIT_MV < vol && (PowerLevel::SL_NORMAL != vol_level_flag)) {
                            // normal
                            SMLK_LOGI("[POWER] inf->voltage is %f", vol);
                            SMLK_LOGD("[POWER] normal voltage ");
                            vol_level_flag = PowerLevel::SL_NORMAL;
                            head.m_extra    = PowerLevel::SL_NORMAL;
                        }
#if 0
                        if ( (SMLK_UINT8)smartlink_sdk::SysPowerVoltageLevel::E_SYS_VOLTAGE_LEVEL_NORMAL == inf->level ) {
                            head.m_extra    = PowerLevel::SL_NORMAL;
                        } else if ( (SMLK_UINT8)smartlink_sdk::SysPowerVoltageLevel::E_SYS_VOLTAGE_LEVEL_ULTRA_LOW == inf->level ) {
                            head.m_extra    = PowerLevel::SL_UNDER_VOLTAGE;
                        } else if ( (SMLK_UINT8)smartlink_sdk::SysPowerVoltageLevel::E_SYS_VOLTAGE_LEVEL_LOW == inf->level ) {
                            head.m_extra    = PowerLevel::SL_UNDER_VOLTAGE;
                        } else if ( (SMLK_UINT8)smartlink_sdk::SysPowerVoltageLevel::E_SYS_VOLTAGE_LEVEL_HIGH == inf->level ) {
                            head.m_extra    = PowerLevel::SL_OVER_VOLTAGE;
                        } else if ( (SMLK_UINT8)smartlink_sdk::SysPowerVoltageLevel::E_SYS_VOLTAGE_LEVEL_ULTRA_HIGH == inf->level ) {
                            head.m_extra    = PowerLevel::SL_OVER_VOLTAGE;
                        } else {
                            SMLK_LOGE("[POWER] unsupported power state: 0x%02X", static_cast<SMLK_UINT8>(inf->level));
                            return;
                        }
#endif
                    }

                    break;
                case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP:
                    {
#if 0
                        if ( sizeof(smartlink_sdk::SysPowerWakeupInfo) != (std::size_t)len ) {
                            SMLK_LOGE("[POWER] unexpected body length for SYS POWER STATE CHANGED: %d, expected: %d", len, (int)sizeof(smartlink_sdk::SysPowerWakeupInfo));
                            return;
                        }
#endif

#ifdef SL_DEBUG
                        SMLK_LOGD("[POWER] system power event wakeup received!!!");
#endif

                        auto inf = reinterpret_cast<smartlink_sdk::SysPowerWakeupInfo *>(data);

                        SMLK_LOGD("[POWER] inf->clock_id:   %s", inf->clock_id.c_str());
                        SMLK_LOGD("[POWER] inf->source:     0x%02X", static_cast<SMLK_UINT8>(inf->source));

                        head.m_seq      = PowerState::SL_WAKEUP;
                        head.m_protocol = inf->source == smartlink_sdk::SysWakeupSource::E_SYS_POWER_WAKEUP_SOURCE_MPU_RTC ? 1 : 0;

                        bytes.insert(bytes.end(), inf->clock_id.cbegin(), inf->clock_id.cend());
                    }
                    break;
                case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP:
                    {
#if 0
                        if ( sizeof(smartlink_sdk::SysPowerWakeupInfo) != (std::size_t)len ) {
                            SMLK_LOGE("[POWER] unexpected body length for SYS POWER STATE CHANGED: %d, expected: %d", len, (int)sizeof(smartlink_sdk::SysPowerWakeupInfo));
                            return;
                        }
#endif

#ifdef SL_DEBUG
                        SMLK_LOGD("[POWER] system power event to sleep received!!!");
#endif
                        head.m_seq      = PowerState::SL_SLEEPING;
                    }
                    break;
                case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_SOURCE_CHANGED:
                    {
                        static SMLK_UINT8 dismantle_main_power_flag = 0;
#ifdef SL_DEBUG
                        SMLK_LOGD("[POWER] system power source change !!!");
#endif
                        if (sizeof(smartlink_sdk::SysPowerSource) != (std::size_t)len ) {
                            SMLK_LOGE("[POWER] unexpected body length for SYS POWER source CHANGED: %d, expected: %d", len, (SMLK_UINT8)sizeof(smartlink_sdk::SysPowerSource));
                            return;
                        }
                        auto inf = reinterpret_cast<const smartlink_sdk::SysPowerSource *>(data);
                        head.m_seq          = PowerState::SL_SOURCE_CHANGED;
                        if ((SMLK_UINT8)smartlink_sdk::SysPowerSource::E_SYS_POWER_SOURCE_MAIN == (SMLK_UINT8)*inf ) {
                            SMLK_LOGD("[POWER] system power source change to main power!!!");
                            if ( 0 == dismantle_main_power_flag) {
                                SMLK_LOGD("[POWER] alerady change to main power!!!");
                                return;
                            }
                            CLRBIT(m_dismantle_tbox_status, DismateTboxMask::SL_MAIN_POWER_ABSENT);
                            dismantle_main_power_flag = 0;
                            Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, M_SL_EVID_DISMANTLE_TBOX, GpsAndTelStatusCheck::GPS_AND_TEL_NEED_CHECK, 0, 0, DismantleReport::EVENT_NOT_REPORT), nullptr, 0);
                        } else if ((SMLK_UINT8)smartlink_sdk::SysPowerSource::E_SYS_POWER_SOURCE_STANDBY == (SMLK_UINT8)*inf) {
                            SMLK_LOGD("[POWER] system power source change to standy power!!!");
                            if ( 1 == dismantle_main_power_flag) {
                                SMLK_LOGD("[POWER] alerady change to backup power!!!");
                                return;
                            }
                            SETBIT(m_dismantle_tbox_status, DismateTboxMask::SL_MAIN_POWER_ABSENT);
                            dismantle_main_power_flag = 1;
                            smartlink_sdk::VehicleMode current_vehicle_mode;
                            auto return_code = smartlink_sdk::MCU::GetInstance()->GetVehicleMode(current_vehicle_mode);
                            if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
                                SMLK_LOGE("fail to GetVehicleMode !!! return code: %d", static_cast<std::int32_t>(return_code));
                                return;
                            }
                            if (smartlink_sdk::VehicleMode::E_VEHICLE_MODE_NORMAL == current_vehicle_mode) {
                                SMLK_LOGD("[POWER] Post M_SL_EVID_DISMANTLE_TBOX dueto powerchange!!!");
                                Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, M_SL_EVID_DISMANTLE_TBOX, GpsAndTelStatusCheck::GPS_AND_TEL_NEED_CHECK, 0, 0, DismantleReport::EVENT_REPORT), nullptr, 0);
                            }
                        } else {
                            SMLK_LOGD("[POWER] system power source change to invalid data%d!!!", (SMLK_UINT8)*inf);
                        }
                    }
                    break;
                default:
                    SMLK_LOGE("[POWER] unsupported event id: %u", static_cast<SMLK_UINT8>(id));
                    return;
            }

            Post(head, std::move(bytes));
        }
    );
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to init register event callback to power service!!! return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    SMLK_LOGD("init MCU service");
    return_code = smartlink_sdk::MCU::GetInstance()->Init(ModuleID::E_Module_daq_service);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to init MCU service API instance!!! return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    std::vector<smartlink_sdk::McuEventId>  mcu_events = {
        smartlink_sdk::McuEventId::E_MCU_EVENT_ISREADY,
        smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE,
        smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_STATE,
        smartlink_sdk::McuEventId::E_MCU_EVENT_GEAR_STATE_CHANGED,
    };
    SMLK_LOGD("register MCU event callback");
    return_code = smartlink_sdk::MCU::GetInstance()->RegEventCB(
        mcu_events,
        [this](smartlink_sdk::McuEventId id, void *data, int len) {
            MsgHead head(M_SL_MSGID_MCU_NTF, 0, 0, 0, 0, 0);
            std::vector<SMLK_UINT8> bytes;

            // SMLK_LOGD("[MCU] MCU event received: 0x%08X", static_cast<SMLK_UINT32>(id));

            switch ( id ) {
                case smartlink_sdk::McuEventId::E_MCU_EVENT_ISREADY:
                    head.m_seq      = MCU::SL_SERVICE_READY;
                    break;
                case smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE:
                    {
                        head.m_seq  = MCU::SL_IGN_NOTIFY;
                        if ( sizeof(smartlink_sdk::IGNState) != (std::size_t)len ) {
                            SMLK_LOGE("[MCU] unexpected body length for MCU IGN state changed notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::IGNState));
                            return;
                        }

                        // modfiy by lsc 2021/6/16  adaptive new IGN struct
                        smartlink_sdk::IGNState *ign = reinterpret_cast<smartlink_sdk::IGNState *>(data);

                        if (ign->on)
                        {
                            head.m_extra = IgnState::SL_ON;
                            auto &signal = m_cached_signals.find(vehicle::SignalID::ACCStatus)->second;
                            signal =  1;
                            m_ig_on_to_off = false;
                        }
                        else
                        {
                            if (1 == m_cached_signals.find(vehicle::SignalID::ACCStatus)->second.Val()) {
                                m_ig_on_to_off = true;
                            }
                            head.m_extra = IgnState::SL_OFF;
                            auto &signal = m_cached_signals.find(vehicle::SignalID::ACCStatus)->second;
                            signal =  0;
                        }
                        if (ign->acc)
                        {

                        }
                        else
                        {

                        }
                        //modify end
                    }
                    break;
                case smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_STATE:
                    {
                        if ( smartlink_sdk::CANState::E_CAN_BUS_STATE_NORMAL == *(reinterpret_cast<smartlink_sdk::CANState *>(data)) ) {
                            head.m_extra = CanBusState::CAN_NORMAL;
                        } else if (smartlink_sdk::CANState::E_CAN_BUS_STATE_SLEEP == *(reinterpret_cast<smartlink_sdk::CANState *>(data))) {
                            head.m_extra = CanBusState::CAN_SLEEP;
                        }
                        head.m_seq  = MCU::SL_CAN_STATUS_NOTIFY;
                    }
                    break;
                case smartlink_sdk::McuEventId::E_MCU_EVENT_GEAR_STATE_CHANGED:
                    {
                        SMLK_LOGD("[GearDebug] E_MCU_EVENT_GEAR_STATE_CHANGED");
                        if (m_internal_flags & InternalFlags::SL_FLAG_ETC2_RECEIVED) {
                            return;
                        }
                        auto gear_value = *(reinterpret_cast<smartlink_sdk::GearState *>(data));
                        if ( smartlink_sdk::GearState::E_GEAR_STATE_NEUTRAL == gear_value ) {
                            SMLK_LOGD("[GearDebug] E_GEAR_STATE_NEUTRAL ");
                            m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second = 0;
                        } else if (smartlink_sdk::GearState::E_GEAR_STATE_OTHER == gear_value ) {
                            SMLK_LOGD("[GearDebug] E_GEAR_STATE_OTHER ");
                            m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second = 48;
                        } else if (smartlink_sdk::GearState::E_GEAR_STATE_INVALID == gear_value) {
                            SMLK_LOGE("[GearDebug] E_GEAR_STATE_INVALID !!! set neutral");
                            m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second = 0;

                        }
                    }
                    break;
                default:
                    SMLK_LOGE("[MCU] unsupported event id: %u", static_cast<SMLK_UINT32>(id));
                    return;
            }

            Post(head, std::move(bytes));
        }
    );
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to init register event callback to MCU service!!! return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    SMLK_LOGD("init telephony service");
    if ( !smartlink_sdk::Telephony::GetInstance()->Init(ModuleID::E_Module_daq_service) ) {
        SMLK_LOGE("fail to init Telephony service API instance!!!");
        return SL_EFAILED;
    }

    std::vector<smartlink_sdk::TelEventId>  tel_events = {
        smartlink_sdk::TelEventId::E_TEL_EVENT_RADIO_REG_CHANGED,
        smartlink_sdk::TelEventId::E_TEL_EVENT_SIGNAL_STRENGTH_CHANGED,
        smartlink_sdk::TelEventId::E_TEL_EVENT_ANTENNA_CHANGED,
        smartlink_sdk::TelEventId::E_TEL_EVENT_SIM_CHANGED,
        smartlink_sdk::TelEventId::E_TEL_EVENT_MODEM_STATE,
        smartlink_sdk::TelEventId::E_TEL_EVENT_DATA_CHANNEL_CHANGED
    };
    return_code = smartlink_sdk::Telephony::GetInstance()->RegEventCB(
        tel_events,
        [this](smartlink_sdk::TelEventId id, void *data, int len) {
            MsgHead head(M_SL_MSGID_TEL_NTF, 0, 0, 0, 0, 0);
            std::vector<SMLK_UINT8> bytes;

            SMLK_LOGD("[TEL] event received: 0x%08X", static_cast<SMLK_UINT32>(id));
            if (data == nullptr)
            {
                SMLK_LOGE("data is nullptr!!!");
                return;
            }
            switch ( id ) {
                case smartlink_sdk::TelEventId::E_TEL_EVENT_SIGNAL_STRENGTH_CHANGED:
                    {

                        head.m_seq  = Tel::SL_SIGNAL_CHANGED;
                        if ( sizeof(smartlink_sdk::TelRadioSignalInfo) != (std::size_t)len ) {
                            SMLK_LOGE("[TEL] unexpected body length for signal strength changed notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::TelRadioSignalInfo));
                            return;
                        }

                        auto inf = reinterpret_cast<const smartlink_sdk::TelRadioSignalInfo *>(data);
                        head.m_extra = inf->signal;
                        SMLK_LOGD("[TEL] signal change %d", inf->signal);
                    }
                    break;
                case smartlink_sdk::TelEventId::E_TEL_EVENT_ANTENNA_CHANGED:
                    {
                        head.m_seq      = Tel::SL_ANTENNA_CHANGED;
                        if ( sizeof(smartlink_sdk::ModemAntennaInfo) != (std::size_t)len ) {
                            SMLK_LOGE("[TEL] unexpected body length for Telephony antenna state changed notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::ModemAntennaInfo));
                            return;
                        }

                        auto inf = reinterpret_cast<const smartlink_sdk::ModemAntennaInfo *>(data);
                        if ( smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_ABSENT == inf->state ) {
                            head.m_extra    = Tel::Antenna::SL_ABSENT;
                        } else if ( smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_PRESENT == inf->state ) {
                            head.m_extra    = Tel::Antenna::SL_PRESENT;
                        } else if ( smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_SHORT_CIRCUIT == inf->state ) {
                            head.m_extra    = Tel::Antenna::SL_SHORT;
                        } else {
                            SMLK_LOGE("[TEL] unsupporte antenna state:  0x%02X", static_cast<SMLK_UINT8>(inf->state) );
                            return;
                        }
                    }
                    break;
                default:
                    SMLK_LOGE("[TEL] unsupported event id: %u", static_cast<SMLK_UINT32>(id));
                    return;
            }

            Post(head, std::move(bytes));
        }
    );
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to init register event callback to Telephony service!!! return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }
    SMLK_LOGD("##############init property service###############");
    if ( !smartlink_sdk::SysProperty::GetInstance()->Init(ModuleID::E_Module_daq_service) ) {
        SMLK_LOGE("fail to init property service API interface!!!");
        return SL_EFAILED;
    }

    // first read property once to avoid daq work later than gb6 check vin notify
    HandleGbCheckVinResult();

    // default is direct control
    m_internal_flags |= InternalFlags::SL_FLAG_DIRECT_CONTROL;
    ReadNetWorkTypeConfig();

    ReadCarTypeConfig();

    SMLK_LOGI("[configDebug] daq init");
    HandleFinaceLockStatus();
    ReadEvEmissionConfig();
    SMLK_LOGD("register sys-power event callback");
    const std::vector<std::string> propertys_key = {
        std::string(SYS_PRO_NAME_GB_CHECK_VIN_FOR_DAQ),
        std::string(SYS_PRO_NAME_DID_OEM_PARTNUMBER), // 判断网络管理类型
        std::string(SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS),
        std::string(SYS_PRO_NAME_DID_VEHICLE_TYPE),
        std::string(SYS_PRO_NAME_REALTIME_DATA_UPLOAD_CYCLE),/*实时数据上传周期（单位：s）,范围:(0~60）*/
        std::string(SYS_PRO_NAME_REALTIME_DATA_GATHER_CYCLE),/*实时数据采集周期（单位：ms） ；没有收到该配置则默认：1000ms*/
        std::string(SYS_PRO_NAME_STATIS_INFO_UPLOAD_CYCLE),
        std::string(SYS_PRO_NAME_SLEEP_REPORT_INTERVAL),//SYS_PRO_NAME_LCKCAR_VALUE_JTT808
        std::string(SYS_PRO_NAME_LCKCAR_VALUE_JTT808),
        std::string(SYS_PRO_NAME_TSC1_VALID),
        std::string(SYS_PRO_NAME_DID_NEW_ENERGY_EMISSION_REMOTE_MONITOR_CONFIG) // 102c
    };

    smartlink_sdk::SysProperty::GetInstance()->RegEventCB(propertys_key,
        [this](smartlink_sdk::SysProperty::PropertyEventId id, smartlink_sdk::PropertyInfo* info) {

            if (info == nullptr) {
                SMLK_LOGE("[property] PropertyInfo ptr is nullptr!!!");
                return;
            }
            switch (id)
            {
                case smartlink_sdk::SysProperty::PropertyEventId::E_SYS_PROPERTY_EVENT_VALUE_CHANGED:
                {
                    if (info->name == std::string(SYS_PRO_NAME_GB_CHECK_VIN_FOR_DAQ))
                    {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[VinDebug] [%s], value is [%s]", info->name.c_str(), info->value.c_str());
                            HandleGbCheckVinResult();
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_DID_OEM_PARTNUMBER)) {
                        // property change
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[property] is [%s], value is [%s]", info->name.c_str(), info->value.c_str());
                            ReadNetWorkTypeConfig();
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS)) {
                        if (!info->value.empty())
                        {
                            i_lckcar_sts = atoi(info->value.c_str());
                            SMLK_LOGI("[configDebug]FINACAIL_LCK_STATUS change [%d]", i_lckcar_sts );
                            HandleFinaceLockStatus();
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_DID_VEHICLE_TYPE)) {
                        if (!info->value.empty())
                        {
                            ReadCarTypeConfig();
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_REALTIME_DATA_GATHER_CYCLE)) {
                        if (!info->value.empty()) {
                            SMLK_LOGI("[configDebug][property] is [%s], value is [%s]", info->name.c_str(), info->value.c_str());
                            HandleSampling1SCollectConfig();
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_REALTIME_DATA_UPLOAD_CYCLE)) {
                        if (!info->value.empty()) {
                            SMLK_LOGI("[configDebug][property] is [%s], value is [%s]", info->name.c_str(), info->value.c_str());
                            HandleSampling1SUploadConfig();
                        }
                    }  else if (info->name == std::string(SYS_PRO_NAME_STATIS_INFO_UPLOAD_CYCLE)) {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[configDebug]Set statistics report value period  %s min", info->value.c_str());
                            HandleStatisticsConfig();
                            SMLK_LOGI("Set statistics report value period  %d second", m_statistics_report_period);
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_SLEEP_REPORT_INTERVAL)) {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[configDebug][WakeupDebug]Set wakeup report value period  %s s", info->value.c_str());
                            // handle wakeup interval
                            HandleWakeupInterval();
                        }
                    } else if(info->name == std::string(SYS_PRO_NAME_LCKCAR_TYPE_JTT808)) {
                        if (!info->value.empty())
                        {
                            std::stringstream ss;
                            ss << info->value;
                            SMLK_UINT8 lock_type = 0;
                            ss >> lock_type;
                            SMLK_LOGI("[configDebug] lock type change  [%d]", lock_type);
                            HandleFinaceLockStatus();
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_LCKCAR_VALUE_JTT808)) {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[configDebug] tsc1 lck car value change");
                            // not call HandleFinaceLockStatus here because vec will set SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS again here
                            HandleFinaceLockStatus();
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_TSC1_VALID)) {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[configDebug]tsc1 valid change");
                            HandleFinaceLockStatus();
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_DID_NEW_ENERGY_EMISSION_REMOTE_MONITOR_CONFIG)) {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[configDebug]tsc1 valid change");
                            ReadEvEmissionConfig();
                        }
                    }
                    break;
                }
                default:
                    SMLK_LOGW("[property] receive unregist event");
                    break;
            }
        } // end lambda
    ); // end reg callback

    SMLK_LOGD("mark service as initialized");
    m_flags |= IService::Initialized;

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: OnUpdateJsonConfig
 *
 * DESCRIPTION: OnUpdateJsonConfig
 *
 * PARAMETERS:
 *
 * RETURN:
 * NOTE:
 *****************************************************************************/
void CollectionService::OnUpdateJsonConfig()
{
    SMLK_LOGI("[configDebug] OnUpdateJsonConfig");
    std::ofstream o(M_SL_SMARTLINK_DAQ_TSP_CFG, std::ios::trunc);
    if (!o.is_open())
    {
        SMLK_LOGW("[configDebug]open file failed");
        return;
    }
    Json::StyledWriter sw;
    Json::Value data;
    Json::Value root;
    SMLK_UINT32 collect_interval_ms;
    SMLK_UINT32 upload_interval_ms;
    SMLK_UINT32 statistics_interval_s;
    SMLK_UINT32 wakeup_interval_s;
    collect_interval_ms = m_trigger_period_1s_ms;
    upload_interval_ms  = m_upload_period_1s_ms;
    statistics_interval_s = m_statistics_report_period;
    wakeup_interval_s = m_wakeup_report_period;

    SMLK_LOGD("[configDebug]update json m_trigger_period_1s_ms is [%d]", m_trigger_period_1s_ms);
    SMLK_LOGD("[configDebug]update json m_upload_period_1s_ms is [%d]", m_upload_period_1s_ms);
    SMLK_LOGD("[configDebug][wakeupdebug]update json m_wakeup_report_period is [%d]", m_wakeup_report_period);

    root["1s_collect_interval_ms"] = Json::Value(collect_interval_ms);
    root["1s_upload_interval_ms"] = Json::Value(upload_interval_ms);
    root["statistic_interval_s"] = Json::Value(statistics_interval_s);
    root["wakeup_interval_s"] = Json::Value(wakeup_interval_s);

    o << sw.write(root);
    o.close();

}

/******************************************************************************
 * NAME: OnReadJsonConfig
 *
 * DESCRIPTION: OnReadJsonConfig
 *
 * PARAMETERS:
 *
 * RETURN:
 * NOTE:
 *****************************************************************************/
void CollectionService::OnReadJsonConfig()
{
    SMLK_LOGI("[configDebug]OnReadJsonConfig");

    SMLK_UINT32 index = 0;
    SMLK_DOUBLE value = 0;
    SMLK_BOOL   flag  = false;

    std::ifstream   ifs(M_SL_SMARTLINK_DAQ_TSP_CFG);
    if ( ifs.is_open() ) {
        Json::Value root;
        Json::CharReaderBuilder builder;
        JSONCPP_STRING errs;

        if ( !parseFromStream(builder, ifs, &root, &errs) ) {
            SMLK_LOGE("fail to parse config file!!! %s", std::string(errs).c_str());
        } else {
                // those configurations in 'debug' field is just for test, for the product version we should not have those configurations
                SMLK_UINT32     tmp_trigger_period_1s_ms  = root.get("1s_collect_interval_ms", 1000).asUInt();
                SMLK_UINT32     tmp_upload_period_1s_ms    = root.get("1s_upload_interval_ms",   10000).asUInt();
                SMLK_UINT32     tmp_trigger_period_statistics_s    = root.get("statistic_interval_s", 600).asUInt();
                SMLK_UINT32     tmp_wakeup_period_s    = root.get("wakeup_interval_s", SL_WAKEUP_REPORT_PERIOD_DEF).asUInt();

                SMLK_LOGI("[configDebug]json tmp_trigger_period_1s_ms is [%d]", tmp_trigger_period_1s_ms);
                SMLK_LOGI("[configDebug]json data tmp_upload_period_1s_ms is [%d]", tmp_upload_period_1s_ms);
                SMLK_LOGI("[configDebug]json data tmp_trigger_period_statistics_s is [%d]", tmp_trigger_period_statistics_s);
                SMLK_LOGI("[configDebug]json data tmp_wakeup_period_s is [%d]", tmp_wakeup_period_s);

                m_trigger_period_1s_ms = tmp_trigger_period_1s_ms;
                m_upload_period_1s_ms = tmp_upload_period_1s_ms;
                m_statistics_report_period = tmp_trigger_period_statistics_s;
                m_bucket_num_1s = m_upload_period_1s_ms / m_trigger_period_1s_ms;
                m_wakeup_report_period = tmp_wakeup_period_s;
                SMLK_LOGI("[configDebug]now m_trigger_period_1s_ms is [%d]", m_trigger_period_1s_ms);
                SMLK_LOGI("[configDebug]now m_upload_period_1s_ms is [%d]", m_upload_period_1s_ms);
                SMLK_LOGI("[configDebug]now m_bucket_num_1s is [%d]", m_bucket_num_1s);
                SMLK_LOGI("[configDebug]now m_statistics_report_period is [%d]", m_statistics_report_period);
                SMLK_LOGI("[configDebug][wakeupdebug]now m_wakeup_report_period is [%d]", m_wakeup_report_period);

        }
        ifs.close();
    } else {
        SMLK_LOGI("[configDebug]recover file is not exist!!!");
        return;
    }

}

/******************************************************************************
 * NAME: Start
 *
 * DESCRIPTION: start the service
 *
 * PARAMETERS:
 *
 * RETURN:
 *      SL_SUCCESS on success
 *      SL_EFAILED on failed
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 CollectionService::Start()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Running ) {
        SMLK_LOGI("service already started, do nothing");
        return SL_SUCCESS;
    }

    if ( !(m_flags & IService::Initialized) ) {
        SMLK_LOGE("service not initialized");
        return SL_EFAILED;
    }

    // TODO: read sql and check whether need use it

    TspConnectState m_tsp_connect_state;
    TspServiceApi::getInstance()->GetConnectState(m_tsp_connect_state);
    SMLK_LOGI("m_tsp_connect_state = %d.\n",m_tsp_connect_state);

    TspLoginState m_tsp_login_state;
    TspServiceApi::getInstance()->GetLoginState(m_tsp_login_state);
    SMLK_LOGI("m_tsp_login_state = %d.\n",m_tsp_login_state);

    for ( auto &pair : m_cached_index_signals ) {
        VehicleData data(pair.first, 0.0);

        auto rc = VehicleDataApi::GetInstance()->GetVehicleData(data);
        if ( SMLK_RC::RC_OK != rc ) {
            SMLK_LOGD("fail to get vehicle data for index:  %u", pair.first);
            continue;
        }

        pair.second = data.value;

        if ( !data.valid ) {
            SMLK_LOGD("vehicle data not valid for index:    %u, val: %f, min[%f], max[%f]", pair.first, data.value, pair.second.Min(), pair.second.Max());
            pair.second.Invalid();
            continue;
#ifdef SL_DEBUG
        } else {
            SMLK_LOGD("vehicle data valid for index:    %u, %f, SPN[%u], MIN[%f], MAX[%f]", pair.first, pair.second.Val(), pair.second.ID(), pair.second.Min(), pair.second.Max());
#endif
        }
    }

    SMLK_LOGD("check GNSS status");
    if ( smartlink_sdk::Location::GetInstance()->IsReady() ) {
        SMLK_LOGD("GNSS service is ready, get GNSS data");
        GetGNSS();
        SMLK_LOGD("GNSS get GNSS finished!!!");

        SMLK_LOGD("get GNSS antenna state");
        smartlink_sdk::GnssAntennaState state;
        auto return_code = smartlink_sdk::Location::GetInstance()->GetGnssAntennaState(state);
        if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
            SMLK_LOGD("fail to get GNSS antenna state!!! return_code = %d", static_cast<std::int32_t>(return_code));
        } else {
            SMLK_LOGD("get GNSS antenna state success");
            auto &signal = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
            if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_ABSENT == state ) {
                SMLK_LOGW("GNSS antenna not detached");
                signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_GNSS_ANTENNA);
            } else if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_SHORT_CIRCUIT == state ) {
                SMLK_LOGW("GNSS antenna short detached");
                signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_GNSS_SHORT);
            } else if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_INVALID == state ) {
                SMLK_LOGW("GNSS module fault detached");
                signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_GNSS_FAULT);
            }
        }
    }
    std::string name_lckcar = SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS;
    std::string lckcar_sts;
    auto property_rc = smartlink_sdk::SysProperty::GetInstance()->GetValue(name_lckcar, lckcar_sts);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != property_rc ){
        SMLK_LOGE("SysProperty::GetInstance()->GetValue(name is %s) failed,errcode=%ld, set to off.\n",name_lckcar.c_str(),static_cast<std::int32_t>(property_rc));
        i_lckcar_sts = STATUS_NOT_ENABLE;
    } else {
        i_lckcar_sts = atoi(lckcar_sts.c_str());
    }

    SMLK_LOGD("check TEL status");
    if ( smartlink_sdk::Telephony::GetInstance()->IsReady() )
    {
        smartlink_sdk::TelRadioSignalInfo rssi;
        smartlink_sdk::Telephony::GetInstance()->GetSignalStrength(rssi);
        SMLK_LOGD("get telephony rssi %d ",rssi.signal);
        auto &signal = m_cached_signals.find(vehicle::SignalID::RSSI)->second;
        signal =  rssi.signal;
        m_internal_flags |= InternalFlags::SL_FLAG_TEL_READY;
    } else {
        SMLK_LOGD("TEL service not  ready");
    }

    if ( smartlink_sdk::MCU::GetInstance()->IsReady()) {
        SMLK_LOGI("[GearDebug] get gear hardwire signal first");
        smartlink_sdk::GearState current_gear;
        auto gear_result = smartlink_sdk::MCU::GetInstance()->GetGearState(current_gear);
        if ( smartlink_sdk::RtnCode::E_SUCCESS == gear_result ) {
            SMLK_LOGI("[GearDebug] current_gear from hardware = %d ", current_gear);
            if (m_internal_flags&InternalFlags::SL_FLAG_ETC2_RECEIVED) {
                SMLK_LOGI("[GearDebug] already receive etc can signal valid");
            } else {
                if ( smartlink_sdk::GearState::E_GEAR_STATE_NEUTRAL == current_gear ) {
                    SMLK_LOGD("[GearDebug] E_GEAR_STATE_NEUTRAL ");
                    m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second = 0;
                } else if (smartlink_sdk::GearState::E_GEAR_STATE_OTHER == current_gear ) {
                    SMLK_LOGD("[GearDebug] E_GEAR_STATE_OTHER ");
                    m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second = 48;
                } else if (smartlink_sdk::GearState::E_GEAR_STATE_INVALID == current_gear) {
                    SMLK_LOGE("[GearDebug] E_GEAR_STATE_INVALID !!!! set neutral");
                    m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second = 0;
                }
            }
        } else {
            SMLK_LOGI("[GearDebug] get current_gear from hardware fail ErrorCode[%d]", gear_result);
        }
    } else {
        SMLK_LOGD("mcu service not ready");
    }


    SMLK_LOGD("mark service as running");
    m_flags |= IService::Running;

    SMLK_LOGD("try to start main work thread");
    m_thread = std::thread(
        [this](){
            SMLK_LOGI("main work thread started");
            while ( !(m_flags & IService::Stopping) ) {
                m_events.get(SL_QUEUE_GET_TIMEOUT_MS)();
            }

            SMLK_LOGI("main work thread greacefully exit!!!");
        }
    );

    // dm1 callback freq maybe very high use specific thread avoid block main signal queue thread
    SMLK_LOGD("try to start m_dm1_events work thread");
    m_dm1_thread = std::thread(
        [this](){
            SMLK_LOGD("m_dm1_events work thread started");
            while ( !(m_flags & IService::Stopping) ) {
                m_dm1_events.get(SL_QUEUE_GET_TIMEOUT_MS)();
            }
            SMLK_LOGE("m_dm1_events work thread greacefully exit!!!");
        }
    );

    SMLK_LOGD("try to start compress thread");
    m_compress_thread = std::thread(
        [this](){
            SMLK_LOGI("compress thread started");
            while ( !(m_flags & IService::Stopping) ) {
                m_compress_events.get(SL_QUEUE_GET_TIMEOUT_MS)();
            }
            SMLK_LOGI("compress thread greacefully exit!!!");
        }
    );

#ifdef SL_SIMULATOR
    m_simulator_thread = std::thread(
        [this](){
            SMLK_LOGI("simulator thread started");
            SimulatorWorker();
        }
        SMLK_LOGI("simulator thread greacefully exit!!!");
    );
#endif

    // since system time service is not ready now
    // m_internal_flags |= InternalFlags::SL_FLAG_SYSTIME_SER | InternalFlags::SL_FLAG_SYSTIME_SYN;
    // m_internal_flags |= InternalFlags::SL_FLAG_CAN_NORMAL;

    // since sys power module is not ready, mark IGN ON always
    // m_internal_flags |= InternalFlags::SL_FLAG_IGNON;

    OnReadJsonConfig();

    // timesync check
    if ( smartlink_sdk::SysTime::GetInstance()->IsReady()) {
        ReadTimeSource();
    } else {
        SMLK_LOGE("[timesyncDebug] SysTime not ready");
    }

    m_dm1_timeout_timer->Start();

    if ( !CheckWhetherTimeSync() ) {
        m_timer_shared  = std::make_shared<LoopTimer<std::chrono::milliseconds, 1000>>(
            [this](SMLK_UINT32 index){
                MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_1S_CACHE, 0, 0, 0, index);
                Post(head, nullptr, 0);
            }
        );

        SMLK_LOGI("start cache timer");
        m_timer_shared->Start();
        m_timer_flags |= TimerFlags::SL_FLAG_CACHE_TIMER_START;
    } else {
        auto now = std::chrono::system_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

        // make sure we almost always trigger the 1s timer at the 500ms of each seconds
        if ( ms <= SL_PERIOD_TRIGGER_POINT_MS ) {
            std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(SL_PERIOD_TRIGGER_POINT_MS - ms));
        } else {
            // here since we may sleep up to 1s, so if we just got a term signal during this time, we may stop until 1 second later
            std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 + SL_PERIOD_TRIGGER_POINT_MS - ms));
        }

        SMLK_LOGD("[timesyncDebug]start normal timer");

        // m_timer->Start();
        if (0 != m_upload_period_1s_ms) {
            m_0f3b_timer->Start();
            m_timer_flags |= TimerFlags::SL_FLAG_0F3B_TIMER_START;
        } else {
            SMLK_LOGD("[timesyncDebug] upload interval is 0 not report-> start timer and reset");
            m_0f3b_timer->Start();
            m_timer_flags |= TimerFlags::SL_FLAG_0F3B_TIMER_START;
            m_0f3b_timer->Reset();
        }

        m_0f3b_30s_timer->Start();
        m_timer_flags |= TimerFlags::SL_FLAG_0F3B_30S_TIMER_START;

        m_0200_timer->Start();
        m_timer_flags |= TimerFlags::SL_FLAG_0200_TIMER_START;

    }
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Stop
 *
 * DESCRIPTION: stop the service
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::Stop()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Running ) {
        m_flags |= IService::Stopping;

        m_dm1_timeout_timer->Stop();

        // m_timer->Stop();
        m_0f3b_timer->Stop();
        m_timer_flags &= ~TimerFlags::SL_FLAG_0F3B_TIMER_START;

        m_0f3b_30s_timer->Stop();
        m_timer_flags &= ~TimerFlags::SL_FLAG_0F3B_30S_TIMER_START;

        m_0f3c_timer->Stop();
        m_timer_flags &= ~TimerFlags::SL_FLAG_0F3C_TIMER_START;

        m_0200_timer->Stop();
        m_timer_flags &= ~TimerFlags::SL_FLAG_0200_TIMER_START;

        if ( m_timer_shared ) {
            SMLK_LOGD("stop cache timer");
            m_timer_shared->Stop();
            m_timer_flags &= ~TimerFlags::SL_FLAG_CACHE_TIMER_START;
        }

#ifdef SL_SIMULATOR
        if ( m_simulator_thread.joinable() ) {
            m_simulator_thread.join();
        }
#endif

        if ( m_thread.joinable() ) {
            m_thread.join();
        }

        if ( m_compress_thread.joinable() ) {
            m_compress_thread.join();
        }

        m_flags &= ~(IService::Running | IService::Stopping);
    }
}

/******************************************************************************
 * NAME: IsRunning
 *
 * DESCRIPTION: check the service running status
 *
 * PARAMETERS:
 *
 * RETURN:      true if the service already running, otherwise false
 *
 * NOTE:
 *****************************************************************************/
bool CollectionService::IsRunning() const
{
    return  m_flags & IService::Running;
}



/******************************************************************************
 * NAME: Post
 *
 * DESCRIPTION: post message the service main queue, the function can be called
 *              by the service its self or other module
 *
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *        sz:   data length
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::Post(IN MsgHead &head, IN SMLK_UINT8 *data, IN std::size_t sz)
{
    m_events.emplace(
        head,
        data,
        sz,
        std::bind(&CollectionService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: Post
 *
 * DESCRIPTION: post message the service main queue, the function can be called
 *              by the service its self or other module
 *
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_events.emplace(
        head,
        data,
        std::bind(&CollectionService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: Post
 *
 * DESCRIPTION: post message the service main queue, the function can be called
 *              by the service its self or other module
 *
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *
 * RETURN:
 *
 * NOTE:        this is a move operation, so after this call, the function
 *              caller should never use the data again
 *****************************************************************************/
void CollectionService::Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data)
{
    m_events.emplace(
        head,
        std::move(data),
        std::bind(&CollectionService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: HandleEvent
 *
 * DESCRIPTION: the main entry for all the events includeing the IPC message,
 *              timer notify message, notify message between threads etc.
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:        this is the main event entry for the main thread, any cost time
 *              operation may block other event, so do not do any const time
 *              operation in the main thread
 *****************************************************************************/
void CollectionService::HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    // SMLK_LOGD("HandleEvent m_id %d", head.m_id);

    switch ( head.m_id ) {
        case M_SL_MSGID_PT_LOCATION_REQ:
            OnRequest(head, data);
            break;
        case M_SL_MSGID_UPDATE_CFG_REQ:
        case M_SL_MSGID_UPDATE_VAL_REQ:
            OnMessage(head, data);
            break;
        case M_SL_MSGID_COMPRESS_RSP:
            OnNotify(head, data);
            break;
        case M_SL_MSGID_TIMER_NOTIFY:
            OnTimer(head.m_seq, head.m_extra);
            break;
        case M_SL_MSGID_DATA_CHANGED:
            OnDataChanged(head, data);
            break;
        case M_SL_MSGID_DATA_TIRE_NTF:
            OnTireNotify(head, data);
            break;
        case M_SL_MSGID_SYSTIME_SYNC:
            OnSystimeSync(head, data);
            break;
        case M_SL_MSGID_SYSTIME_READY:
            OnSystimeReady(head, data);
            break;
        case M_SL_MSGID_MCU_NTF:
            OnMcuNotify(head, data);
            break;
        case M_SL_MSGID_LOCATOIN_NTF:
            OnLocation(head, data);
            break;
        case M_SL_MSGID_SYSPOWER_NTF:
            OnSysPower(head, data);
            break;
        case M_SL_MSGID_TEL_NTF:
            OnTelNotify(head, data);
            break;
        case M_SL_MSGID_DRIVE_EVENT:
            OnDriveEvent(head, data);
            break;
        case M_SL_MSGID_DAY_CHANGED:
            OnDayChanged();
            break;
        default:
            // TODO
            break;
    }
}

void CollectionService::PostDM1(IN MsgHead &head, IN SMLK_UINT8 *data, IN std::size_t sz)
{
    m_dm1_events.emplace(
        head,
        data,
        sz,
        std::bind(&CollectionService::DM1HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: PostDM1
 *
 * DESCRIPTION: post message the service dm1 queue, the function can be called
 *              by the service its self or other module
 *              only for dm1 process for its freqence
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::PostDM1(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_dm1_events.emplace(
        head,
        data,
        std::bind(&CollectionService::DM1HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void CollectionService::PostDM1(IN MsgHead &head, IN std::vector<SMLK_UINT8> &&data)
{
    m_dm1_events.emplace(
        head,
        std::move(data),
        std::bind(&CollectionService::DM1HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void CollectionService::DM1HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_id ) {
        case M_SL_MSGID_DATA_DM1_NTF:
            OnDM1Notify(head, data);
        break;
        case M_SL_MSGID_DATA_DM1_TIMER:
            OnCheckDataValidTimeout();
        break;
        default:
            // TODO
            break;
    }
}

/******************************************************************************
 * NAME: OnMessage
 *
 * DESCRIPTION: message entry to hanlde the messages between processes switched
 *              via IPC.
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/

void CollectionService::OnMessage(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    (void)head;
    (void)data;
}

/******************************************************************************
 * NAME: OnRequest
 *
 * DESCRIPTION: request entry to hanlde the request, for now only location query
 *              reqeust will come here
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *       data:  message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnRequest(IN MsgHead &head, IN std::vector<SMLK_UINT8> &)
{
    auto protocol_it = m_protocol_messages.find(head.m_protocol);
    if ( m_protocol_messages.end() == protocol_it ) {
        SMLK_LOGE("unsupported protocol!!!");
        return;
    }

    SamplingLocation();

    std::shared_ptr<MessageLocation>    loation = std::static_pointer_cast<MessageLocation>(protocol_it->second[MessageType::SL_LOCATION]);
    std::shared_ptr<MessageLocationRsp> resp    = std::static_pointer_cast<MessageLocationRsp>(protocol_it->second[MessageType::SL_LOCATION_RSP]);

    resp->m_seq                 = (SMLK_UINT16)head.m_seq;
    resp->m_message_location    = *loation;

    std::vector<SMLK_UINT8> encoded;
    encoded.reserve(MAX_ETHERNET_SEND_SIZE);

    if ( SL_SUCCESS != resp->Encode(encoded) ) {
        SMLK_LOGE("fail to encode location information!!!");
        return;
    }

    IpcTspHead      resp_head;
    std::memset(&resp_head, 0x00, sizeof(resp_head));

    resp_head.msg_id    = M_SL_MSGID_TP_LOCATION_RSP;
    resp_head.protocol  = head.m_protocol;
    resp_head.qos       = QOS_SEND_ALWAYS;

    TspServiceApi::getInstance()->SendMsg(resp_head, encoded.data(), encoded.size());
}

/******************************************************************************
 * NAME: OnNotify
 *
 * DESCRIPTION: notify entry to hanlde the notify, for now
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *       data:  message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnNotify");

#ifdef SL_DEBUG
        std::stringstream   ss;
#ifdef PLATFORM_ARM
        std::size_t         index   = 0;
        std::size_t         seq     = 0;
        for ( auto ch : data ) {
            ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
            if ( ++index >=  SL_PLATFORM_MAX_BYTES ) {
                // SMLK_LOGD("compressed data: [%03u]%s", seq, ss.str().c_str());
                ss.str("");
                ss.clear();
                seq++;
                index = 0;
            }
        }
       // SMLK_LOGD("compressed data: [%03u]%s", seq, ss.str().c_str());
#else
        for ( auto ch : data ) {
            ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
        }

        SMLK_LOGD("compressed data: \"%s\"", ss.str().c_str());
#endif  // #ifdef PLATFORM_ARM
#endif  // #ifdef SL_DEBUG

#ifdef SL_DEBUG
    {
        std::uint8_t            buffer[SL_COMPRESS_BUFFER_SZ];
        std::vector<SMLK_UINT8> depressed;

        z_stream    stream;

        stream.zalloc   = Z_NULL;
        stream.zfree    = Z_NULL;
        stream.opaque   = Z_NULL;
        stream.avail_in = 0;
        stream.next_in  = Z_NULL;

        int ret = inflateInit(&stream);
        if ( Z_OK != ret ) {
            SMLK_LOGE("fail to init inflate, ret: %d", ret);
            return;
        }

        stream.avail_in = data.size();
        stream.next_in  = (unsigned char *)data.data();

        do {
            stream.avail_out    = sizeof(buffer);
            stream.next_out     = buffer;

            ret = inflate(&stream, Z_NO_FLUSH);
            switch ( ret ) {
                case Z_NEED_DICT:
                case Z_DATA_ERROR:
                case Z_MEM_ERROR:
                    {
                        SMLK_LOGE("inflate return failed. ret = %d", ret);
                        inflateEnd(&stream);
                    }
                    return;
            }

            depressed.insert(depressed.end(), buffer, buffer + (sizeof(buffer) - stream.avail_out));
        } while ( 0 == stream.avail_out );

        inflateEnd(&stream);

        std::string     records((const char *)&depressed[22], depressed.size() - 22);
#ifdef PLATFORM_ARM
        static constexpr auto split = [](const std::string &str, const std::string &sub)->std::vector<std::string> {
            std::vector<std::string>    vs;
            std::string                 s(str);
            auto pos    = s.find(sub);
            while ( std::string::npos != pos ) {
                vs.push_back(s.substr(0, pos));
                s = s.substr(pos + sub.length());
                pos = s.find(sub);
            }
            vs.push_back(s);

            return vs;
        };

        auto vs = split(records, "\r\n");
        for ( auto const &s : vs ) {
            auto    *pcur = reinterpret_cast<const char *>(s.c_str());
            auto    sz = s.size();

            std::size_t seq = 0;
            while ( sz > SL_PLATFORM_MAX_CHARS ) {
                // SMLK_LOGD("[%03u] %s", seq, std::string(pcur, pcur + SL_PLATFORM_MAX_CHARS).c_str());
                seq++;
                pcur    += SL_PLATFORM_MAX_CHARS;
                sz      -= SL_PLATFORM_MAX_CHARS;
            }
            // SMLK_LOGD("[%03u] %s", seq, std::string(pcur, pcur + sz).c_str());
        }
#else
        SMLK_LOGD("%s", records.c_str());
#endif  // #ifdef PLATFORM_ARM
    }
#endif

    /*
     * Here since we do not need the length attr for the internal message, so
     * we ruse the length parameter to pass the frequency and compress result.
     * frequency will be located at the high SMLK_UINT8 of length, and the result will
     * be located at the low SMLK_UINT8 of length.
    */

    SMLK_UINT8  freq    = (SMLK_UINT8)((head.m_length >> 8) & 0xFF);
    SMLK_UINT8  status  = (SMLK_UINT8)(head.m_length & 0xFF);

    if ( Compress::SL_RC_SUCCESS != status ) {
        SMLK_LOGE("fail to compress data!!!");
        return;
    }

    if ( Frequency::SL_1S != freq && Frequency::SL_30S != freq){
        SMLK_LOGE("invalid frequency type, type: %d", freq);
        return;
    }

    auto protocol_it = m_protocol_messages.find(head.m_protocol);
    if ( m_protocol_messages.end() == protocol_it ) {
        SMLK_LOGE("invalid protocol information, protocol: %d", head.m_protocol);
        return;
    }

    std::shared_ptr<MessageCompressedBucket>    ptr = std::static_pointer_cast<MessageCompressedBucket>(protocol_it->second[MessageType::SL_COMPRESSED_BUCKET]);

    // ptr->m_sampling_frequency   = frequencies[freq];
    auto it = m_frequency_value.find(freq);
    if (m_frequency_value.end() != it)
    {
        if(Frequency::SL_1S == freq)
        {
            ptr->m_sampling_frequency = it->second / 10;
        }
        else if(Frequency::SL_30S == freq)
        {
            ptr->m_sampling_frequency = it->second * 100;
        }

    }
    else{
        ptr->m_sampling_frequency = 100;
    }

    ptr->m_crompression         = 0x01 | (freq << 1);
    ptr->m_compressed           = data;

    std::vector<SMLK_UINT8> encoded;
    encoded.reserve(MAX_ETHERNET_SEND_SIZE);

    if ( SL_SUCCESS != ptr->Encode(encoded) ) {
        SMLK_LOGE("fail to encode compressed bucket message");
        return;
    }

    IpcTspHead  resp_head;
    std::memset(&resp_head, 0x00, sizeof(resp_head));

    resp_head.msg_id    = M_SL_MSGID_TP_BUCKET_REQ;
    resp_head.protocol  = head.m_protocol;
    resp_head.qos       = QOS_SEND_ALWAYS;

    TspServiceApi::getInstance()->SendMsg(resp_head, encoded.data(), encoded.size());
}

/******************************************************************************
 * NAME: OnCompress
 *
 * DESCRIPTION: data compress function
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *       data:  message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnCompress(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    /*
     * Here since we use the vector to pass the data, so we do not need the length
     * of the message head, so we resue the length of the head to store the frequency
     * and compress level, which frequency will located at the high SMLK_UINT8 of length
     * while compress result located at the low SMLK_UINT8.
    */
    int             ret;
    z_stream        stream;
    std::uint8_t    buffer[SL_COMPRESS_BUFFER_SZ];

    // reuse the length of the head to mark the compress result
    MsgHead resp_head(M_SL_MSGID_COMPRESS_RSP, head.m_seq, head.m_protocol, 0x01, (head.m_length & 0xFF00) | Compress::SL_RC_SUCCESS, 0x00000000);
    if ( 0 == data.size() ) {
        Post(resp_head, nullptr, 0);
        return;
    }

    stream.zalloc   = Z_NULL;
    stream.zfree    = Z_NULL;
    stream.opaque   = Z_NULL;

    ret = deflateInit(&stream, Z_DEFAULT_COMPRESSION);
    if ( Z_OK != ret ) {
        SMLK_LOGE("fail to init deflate");
        resp_head.m_length |= Compress::SL_RC_EPARAM;
        Post(resp_head, nullptr, 0);
        return;
    }

    auto sz     = data.size();
    auto ptr    = data.data();

    std::vector<SMLK_UINT8> compressed;
    while ( sz > 0 ) {
        stream.next_in      = (unsigned char *)ptr;
        stream.avail_in     = (sz >= sizeof(buffer)) ? sizeof(buffer) : sz;

        ptr += stream.avail_in;
        sz  -= stream.avail_in;

        int flash = sz > 0 ? Z_NO_FLUSH : Z_FINISH;

        do {
            stream.avail_out    = sizeof(buffer);
            stream.next_out     = buffer;

            ret = deflate(&stream, flash);
            if ( Z_STREAM_ERROR == ret ) {
                (void)deflateEnd(&stream);
                std::vector<SMLK_UINT8>().swap(compressed);
                resp_head.m_length |= Compress::SL_RC_EFAILED;
                SMLK_LOGE("deflate return failed, ret = %d", ret);
                Post(resp_head, nullptr, 0);
                return;
            }

            compressed.insert(compressed.end(), buffer, buffer + (sizeof(buffer) - stream.avail_out));
        } while ( 0 == stream.avail_out );
    }

    (void)deflateEnd(&stream);

    Post(resp_head, std::move(compressed));
}

/******************************************************************************
 * NAME: OnDataChanged
 *
 * DESCRIPTION: data changed event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *       data:  message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnDataChanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    const double    *ptr    = reinterpret_cast<const double *>(data.data());
    SMLK_UINT32     index   = head.m_seq;
    bool            valid   = 0 != head.m_protocol;
        if (false == m_ig_on_to_off) {
            auto it = m_cached_index_signals.find(index);
            if ( m_cached_index_signals.cend() != it ) {
                if ( it->second.Valid() && !valid ) {
                    if (m_internal_flags&InternalFlags::SL_FLAG_IGNON) {
                        SMLK_LOGD("signal changed:  VALID ---> INVALID, signal index: %u", index);
                    }
                } else if ( !it->second.Valid() && valid ) {
                    SMLK_LOGI("signal changed:  INVALID ---> VALID, signal index: %u val = %lf id is [%d]", index, *ptr,it->second.ID() );
                }

                if ( it->second.ID() == vehicle::SPN::AcceleratorPedalPosition1 ) {
                    m_cached_ap_positions.emplace(std::chrono::steady_clock::now(), (SMLK_UINT32)(it->second.Val() * 10));
                }
#ifdef VEHICLE_VKA
                if (0 == (m_vka_flags&VKaBackupFlags::SL_FLAG_USE_VEHICLE_SPEED)) {
                    if ( it->second.ID() == vehicle::SPN::TachographVehicleSpeed ) {
                        SMLK_LOGI("[backupSignal] not use TachographVehicleSpeed_backup any more" );
                        m_vka_flags |= VKaBackupFlags::SL_FLAG_USE_VEHICLE_SPEED;
                    }
                }
#endif
                // check whether SL_FLAG_ETC2_RECEIVED
                if (0 == (m_internal_flags&InternalFlags::SL_FLAG_ETC2_RECEIVED)) {
                    if ( it->second.ID() == vehicle::SPN::TransmissionCurrentGear ) {
                        if (it->second.Valid()) {
                            // not use HardWired signal anymore
                            m_internal_flags |= InternalFlags::SL_FLAG_ETC2_RECEIVED;
                        }
                        //未收到有效的TransmissionCurrentGear信号前， 不存放TransmissionCurrentGear信号相关信息。
                        else if (0 == (m_internal_flags & InternalFlags::SL_FLAG_ETC2_RECEIVED))
                        {
                            SMLK_LOGI("TransmissionCurrentGear no valided, so no save.");
                            return;
                        }
                    }
                }

                it->second = *ptr;
                if ( !valid ) {
                    it->second.Invalid();
                }

                //存在授时时，对行程开始时间进行授时时间补偿
                if (index == VEHICLE_DATA_STA_TRIP_START_TIME )
                {
                    if (CheckWhetherTimeSync())
                    {
                        SMLK_LOGI("startTimeValue m_timeSyncGap_ms: %lld", m_timeSyncGap_ms);
                        double startTimeValue = *ptr;
                        startTimeValue = startTimeValue - (m_timeSyncGap_ms/1000);
                        it->second = startTimeValue;
                    }
                }

                // add door received check
                if (DoorSignalStatus::SL_ALL_DOOR_SIGNAL_RECEIVED != m_door_signal_status) {
                    if ( it->second.ID() == vehicle::SPN::SMLK_OpenStatusDoorDrv) {
                        m_door_signal_status |= DoorSignalStatus::SL_DRIVER_DOOR_SIGNAL_RECIEVED;
                        //未收到有效的车门信号前 开关信号中 车门关有效位 都为无效
                    }
                    if ( it->second.ID() == vehicle::SPN::SMLK_OpenStatusDoorPas) {
                        m_door_signal_status |= DoorSignalStatus::SL_PAS_DOOR_SIGNAL_RECEIVED;
                    }
                }

                // define whether trip start to report staticsis data:only define start
                if ( it->second.ID() == vehicle::SPN::EngineSpeed ) {
                    // avoid when ig off and vec still notify siganl valid&&changing from ipc cached data
                    if ( (it->second.Val() > 500) && (m_internal_flags&InternalFlags::SL_FLAG_IGNON) )
                    {
                        if (0 == (m_timer_flags&TimerFlags::SL_FLAG_0F3C_TIMER_START))
                        {
                            SMLK_LOGI(" m_0f3c_timer start!");
                            m_0f3c_timer->Start();
                            m_timer_flags |= TimerFlags::SL_FLAG_0F3C_TIMER_START;
                        }
                        m_internal_flags |= InternalFlags::SL_FLAG_TRIP_START;
                    }
                }
            } else {
                SMLK_LOGE("unregisted data change notification for index: %u", index);
            }
        } else {
            std::lock_guard<std::mutex> lk(m_cache_map_mutex);
            auto it = m_cached_index_signals.find(index);
            if ( m_cached_index_signals.cend() != it ) {
                if ( it->second.Valid() && !valid ) {
                    SMLK_LOGI("signal changed:  VALID ---> INVALID, signal index: %u", index);
                } else if ( !it->second.Valid() && valid ) {
                    SMLK_LOGI("signal changed:  INVALID ---> VALID, signal index: %u val = %lf", index, *ptr);
                }

                if ( it->second.ID() == vehicle::SPN::AcceleratorPedalPosition1 ) {
                    m_cached_ap_positions.emplace(std::chrono::steady_clock::now(), (SMLK_UINT32)(it->second.Val() * 10));
                }

                // check whether SL_FLAG_ETC2_RECEIVED
                if (0 == (m_internal_flags&InternalFlags::SL_FLAG_ETC2_RECEIVED)) {
                    if ( it->second.ID() == vehicle::SPN::TransmissionCurrentGear ) {
                        if (it->second.Valid()) {
                            // not use HardWired signal anymore
                            m_internal_flags |= InternalFlags::SL_FLAG_ETC2_RECEIVED;
                        }
                        //未收到有效的TransmissionCurrentGear信号前， 不存放TransmissionCurrentGear信号相关信息。
                        else if (0 == (m_internal_flags & InternalFlags::SL_FLAG_ETC2_RECEIVED))
                        {
                            SMLK_LOGI("TransmissionCurrentGear no valided, so no save.");
                            return;
                        }
                    }
                }
#ifdef VEHICLE_VKA
                if (0 == (m_vka_flags&VKaBackupFlags::SL_FLAG_USE_VEHICLE_SPEED)) {
                    if ( it->second.ID() == vehicle::SPN::TachographVehicleSpeed ) {
                        SMLK_LOGI(" [backupSignal] not use TachographVehicleSpeed_backup now" );
                        m_vka_flags |= VKaBackupFlags::SL_FLAG_USE_VEHICLE_SPEED;
                    }
                }
#endif
                it->second = *ptr;
                if ( !valid ) {
                    it->second.Invalid();
                }
                //存在授时时，对行程开始时间进行授时时间补偿
                if (index == VEHICLE_DATA_STA_TRIP_START_TIME )
                {
                    if (CheckWhetherTimeSync())
                    {
                        SMLK_LOGI("startTimeValue m_timeSyncGap_ms: %lld", m_timeSyncGap_ms);
                        double startTimeValue = *ptr;
                        startTimeValue = startTimeValue - (m_timeSyncGap_ms/1000);
                        it->second = startTimeValue;
                    }
                }

                // add door received check
                if (DoorSignalStatus::SL_ALL_DOOR_SIGNAL_RECEIVED != m_door_signal_status) {
                    if ( it->second.ID() == vehicle::SPN::SMLK_OpenStatusDoorDrv) {
                        m_door_signal_status |= DoorSignalStatus::SL_DRIVER_DOOR_SIGNAL_RECIEVED;
                        //未收到有效的车门信号前 开关信号中 车门关有效位 都为无效
                    }
                    if ( it->second.ID() == vehicle::SPN::SMLK_OpenStatusDoorPas) {
                        m_door_signal_status |= DoorSignalStatus::SL_PAS_DOOR_SIGNAL_RECEIVED;
                    }
                }

                // define whether trip start to report staticsis data:only define start
                if ( it->second.ID() == vehicle::SPN::EngineSpeed ) {
                    // avoid when ig off and vec still notify siganl valid&&changing from ipc cached data
                    if ( (it->second.Val() > 500) && (m_internal_flags&InternalFlags::SL_FLAG_IGNON) )
                    {
                        if (0 == (m_timer_flags&TimerFlags::SL_FLAG_0F3C_TIMER_START))
                        {
                            SMLK_LOGI(" m_0f3c_timer start!");
                            m_0f3c_timer->Start();
                            m_timer_flags |= TimerFlags::SL_FLAG_0F3C_TIMER_START;
                        }
                        m_internal_flags |= InternalFlags::SL_FLAG_TRIP_START;
                    }
                }
            } else {
                SMLK_LOGE("unregisted data change notification for index: %u", index);
            }
        }
#ifdef VEHICLE_VKA
        auto iter = m_cached_index_signals.find(index);
        if (iter->second.ID() == vehicle::SPN::TachographVehicleSpeed_backup) {
            if (0 == (m_vka_flags&VKaBackupFlags::SL_FLAG_USE_VEHICLE_SPEED)) {
                auto& signal = m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed_backup)->second;
                if (!signal.Valid() ) {
                    m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second = signal.Val();
                    m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second.Invalid();
                } else {
                    m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second = signal.Val();
                }
            }
        }
#endif
}

/******************************************************************************
 * NAME: OnTireNotify
 *
 * DESCRIPTION: tire information notification handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *       data:  message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnTireNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_UINT32     index   = head.m_seq;
    bool            valid   = 0 != head.m_protocol;
    bool            ext     = 0 != (TireMask::SL_TIRE_EXT & head.m_extra);
#ifdef SL_DEBUG
    SMLK_LOGD("[tiredebug] ext = %d", ext );
    SMLK_LOGD("[tiredebug] index is = %d", index );
#endif

    if ( ext ) {
        auto it = m_cached_tires_ext_signals.find(index);

        if ( m_cached_tires_ext_signals.end() == it ) {
            SMLK_LOGE("[tiredebug] unregister tire ext notification for index: %u", index);
            return;
        }

        if ( sizeof(SMLK_UINT64) != data.size() ) {
            SMLK_LOGE("[tiredebug]unexpected body size: %u", data.size());
            return;
        }
        // 4byte + 4 byte = location + value
        SMLK_UINT64     ll = 0;
        for ( auto v : data ) {
            ll <<= 8;
            ll |= v;
        }
        SMLK_UINT8  location = (SMLK_UINT8)((ll >> 32) & 0xFF);

        // SMLK_UINT16  ext_pressue = (SMLK_UINT16)(ll & 0xFF);
        if (INVALID_TIRE_LOCATION_VALUE == location) {
            SMLK_LOGD("[tiredebug] receive invalid tire location [%02x] in can martix", location);
            return;
        }

        it->second.AssignEncoded(ll & 0xFFFFFFFF);

        m_cached_signals.find(vehicle::SignalID::TireLocation2)->second.AssignEncoded(location);
        // m_cached_signals.find(vehicle::SignalID::TirePressureExtendedRange)->second.AssignEncoded(ext_pressue);

        auto it_tire = m_cached_tires.find(location);
        if (it_tire != m_cached_tires.end())
        {
            std::shared_ptr<InternalStruct::TireInfo>   info = std::static_pointer_cast<InternalStruct::TireInfo>(it_tire->second);
            info->m_expired_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(SL_PERIOD_TIRE_VALID);

            if (valid)
            {
                if ( it->second.ID() == info->m_extend_pressure.ID())
                {
                    info->m_extend_pressure.AssignEncoded(ll & 0xFFFFFFFF);
                }
            }
        }
        else
        {
            if (valid)
            {
                std::shared_ptr<InternalStruct::TireInfo>   info = std::make_shared<InternalStruct::TireInfo>();
                info->m_temperature.AssignEncoded(0xFFFFFFFF);
                info->m_pressure.AssignEncoded(0xFFFFFFFF);
                info->m_extend_pressure.AssignEncoded(0xFFFFFFFF);

                info->m_expired_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(SL_PERIOD_TIRE_VALID);
                if ( it->second.ID() == info->m_extend_pressure.ID() ) {
                    info->m_extend_pressure.AssignEncoded(ll & 0xFFFFFFFF);
                }
                m_cached_tires[location]    = info;
            }
        }

        if ( !valid ) {
            SMLK_LOGE("[tiredebug] check invalid");
            SMLK_LOGD("[tiredebug] 18FC4233 signal is invalid");
            it->second.Invalid();
            m_cached_signals.find(vehicle::SignalID::TireLocation2)->second.Invalid();
        }
    } else {
        auto it = m_cached_tires_signals.find(index);
        if (m_cached_tires_signals.end() == it ) {
            SMLK_LOGE("[tiredebug] unregister tire notification for index: %u", index);
            return;
        }

        if ( sizeof(SMLK_UINT64) != data.size() ) {
            SMLK_LOGE("[tiredebug] unexpected body size: %u", data.size());
            return;
        }

        SMLK_UINT64     ll = 0;
        for ( auto v : data ) {
            ll <<= 8;
            ll |= v;
        }
        SMLK_UINT8  location = (SMLK_UINT8)((ll >> 32) & 0xFF);

        if (INVALID_TIRE_LOCATION_VALUE == location) {
            SMLK_LOGD("[tiredebug] receive invalid tire location [%02x] in can martix", location);
            return;
        }

        it->second.AssignEncoded(ll & 0xFFFFFFFF);

        m_cached_signals.find(vehicle::SignalID::TireLocation1)->second.AssignEncoded(location);

        auto it_tire = m_cached_tires.find(location);
        if (it_tire != m_cached_tires.end())
        {
            std::shared_ptr<InternalStruct::TireInfo>   info = std::static_pointer_cast<InternalStruct::TireInfo>(it_tire->second);
            info->m_expired_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(SL_PERIOD_TIRE_VALID);

            if (valid)
            {
                if (it->second.ID() == info->m_pressure.ID())
                {
                    info->m_pressure.AssignEncoded(ll & 0xFFFFFFFF);
                }
                else if ( it->second.ID() == info->m_temperature.ID())
                {
                    info->m_temperature.AssignEncoded(ll & 0xFFFFFFFF);
                }
                else if ( it->second.ID() == info->m_extend_pressure.ID())
                {
                    info->m_extend_pressure.AssignEncoded(ll & 0xFFFFFFFF);
                }
            }
        }
        else
        {
            if (valid)
            {
                std::shared_ptr<InternalStruct::TireInfo>   info = std::make_shared<InternalStruct::TireInfo>();
                info->m_expired_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(SL_PERIOD_TIRE_VALID);
                info->m_temperature.AssignEncoded(0xFFFFFFFF);
                info->m_pressure.AssignEncoded(0xFFFFFFFF);
                info->m_extend_pressure.AssignEncoded(0xFFFFFFFF);

                if (it->second.ID() == info->m_pressure.ID())
                {
                    info->m_pressure.AssignEncoded(ll & 0xFFFFFFFF);
                }
                else if ( it->second.ID() == info->m_temperature.ID())
                {
                    info->m_temperature.AssignEncoded(ll & 0xFFFFFFFF);
                }
                else if ( it->second.ID() == info->m_extend_pressure.ID())
                {
                    info->m_extend_pressure.AssignEncoded(ll & 0xFFFFFFFF);
                }
                m_cached_tires[location]    = info;
            }
        }

        if ( !valid) {
            it->second.Invalid();
                // tire1 is invalid
                SMLK_LOGD("[tiredebug] 18FEF433 signal is invalid");
                m_cached_signals.find(vehicle::SignalID::TirePressureThresholdDetection)->second.Invalid();
                m_cached_signals.find(vehicle::SignalID::TirePressureThresholdDetectionOriginal)->second.Invalid();
                m_cached_signals.find(vehicle::SignalID::TireLocation1)->second.Invalid();
        }
    }
}

void CollectionService::ReadTimeSource()
{
    SMLK_LOGI("[timesyncDebug] on ReadTimeSource");
    smartlink_sdk::TimeInfo current_info;
    memset(&current_info, 0, sizeof(current_info));
    auto result = smartlink_sdk::SysTime::GetInstance()->GetClockInfo(current_info);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != result ) {
        SMLK_LOGE("[timesyncDebug] GetClockInfo fail");
    } else {
        bool timeSourceSync = current_info.timesync;
        SMLK_UINT32 currentTimeSource = (SMLK_UINT32)current_info.source;
        SMLK_LOGI("[timesyncDebug] time source 1:GNSS 2:NTP 3:NITZ 4:TSP 5:RTC");
        SMLK_LOGI("[timesyncDebug]***** timesync = [%d]", current_info.timesync);
        SMLK_LOGI("[timesyncDebug] TimeSource = [%d] ", current_info.source);
        SMLK_LOGI("[timesyncDebug] current utc s is = [%llu] ", current_info.timestamp/1000000);
        if (timeSourceSync) {
            SMLK_LOGI("[timesyncDebug] timesync when read");
            m_internal_flags |= InternalFlags::SL_FLAG_SYSTIME_SER;
            if ( smartlink_sdk::TimeSource::E_TIME_SOURCE_NTP  == currentTimeSource
                || smartlink_sdk::TimeSource::E_TIME_SOURCE_GNSS  == currentTimeSource) {
                SMLK_LOGI("[timesyncDebug] read useful time source can upload data");
                m_internal_flags |= InternalFlags::SL_FLAG_SYSTIME_SYN;
            }
        } else {
            if ( smartlink_sdk::TimeSource::E_TIME_SOURCE_NTP  == currentTimeSource
                || smartlink_sdk::TimeSource::E_TIME_SOURCE_GNSS  == currentTimeSource) {
                SMLK_LOGE("[timesyncDebug] read useful time source but not sync");
                // m_internal_flags |= InternalFlags::SL_FLAG_SYSTIME_SER | InternalFlags::SL_FLAG_SYSTIME_SYN;
            }
        }
    }
    SMLK_LOGI("[timesyncDebug] m_timeSyncGap_ms is %lld", m_timeSyncGap_ms);
}

bool CollectionService::CheckWhetherTimeSync()
{
    bool ret = false;
    if (m_internal_flags&InternalFlags::SL_FLAG_SYSTIME_SYN) {
        ret = true;
    } else {
        ret = false;
    }
    return ret;
}

bool CollectionService::CheckWhetherInPeriodWakeup()
{
    bool ret = false;
    if (m_internal_flags&InternalFlags::SL_FLAG_PERIOD_WAKEUP) {
        ret = true;
    } else {
        ret = false;
    }
    return ret;
}

/******************************************************************************
 * NAME: Check30sReportContidion
 *
 * DESCRIPTION: Check 0f3b1s Sampling Condition
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 * 0f3b 30s need report last frame when igoff by 30s period
 *****************************************************************************/
bool CollectionService::Check30sReportContidion()
{
    SMLK_LOGD("[newdebug] Check30sReportContidion");
    bool ret = false;
    if (m_internal_flags&InternalFlags::SL_FLAG_DIRECT_CONTROL) {
        if (m_internal_flags&InternalFlags::SL_FLAG_CAN_NORMAL) {
            ret = true;
            SMLK_LOGD("[newdebug] direct network SL_FLAG_CAN_NORMAL");
        } else {
            if ( CheckWhetherInPeriodWakeup() ) {
                ret = false;
                SMLK_LOGD("[newdebug] direct network InPeriodWakeup");
            } else {
                SMLK_LOGD("[newdebug] direct network collect last frame m_0f3b_30s_timer reset");
                if (m_collect_last_frame_flag&LastFrameCollectFlags::SL_0F3B_30S_COLLECT) {
                    ret = true;
                    m_collect_last_frame_flag &= ~LastFrameCollectFlags::SL_0F3B_30S_COLLECT;
                    SMLK_LOGI("[newdebug] direct network collect last frame m_0f3b_30s_timer reset");
                    m_0f3b_30s_timer->Reset();
                }
            }
        }
    } else {
        if (m_internal_flags&InternalFlags::SL_FLAG_IGNON) {
            SMLK_LOGD("[newdebug] indirect network SL_FLAG_IGNON");
            ret = true;
        } else {
        SMLK_LOGD("[newdebug] indirect network SL_FLAG_IGNOFF");
        if ( CheckWhetherInPeriodWakeup() ) {
            SMLK_LOGD("[newdebug] indirect network InPeriodWakeup");
            ret = false;
        } else {
            SMLK_LOGD("[newdebug] not in wakeup");
            if (m_collect_last_frame_flag&LastFrameCollectFlags::SL_0F3B_30S_COLLECT) {
                ret = true;
                m_collect_last_frame_flag &= ~LastFrameCollectFlags::SL_0F3B_30S_COLLECT;
                SMLK_LOGI("[newdebug] indirect network collect last frame m_0f3b_30s_timer reset");
                m_0f3b_30s_timer->Reset();
            }
        }
        }
    }
    return ret;
}

/******************************************************************************
 * NAME: Check1sReportContidion
 *
 * DESCRIPTION: Check 0f3b1s Sampling Condition
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 * 0f3b 1s need report last frame when igoff by 30s period
 *****************************************************************************/
bool CollectionService::Check1sReportContidion()
{
    bool ret = false;
    if (m_internal_flags&InternalFlags::SL_FLAG_DIRECT_CONTROL) {
        if (m_internal_flags&InternalFlags::SL_FLAG_CAN_NORMAL) {
            ret = true;
        } else {
            if ( CheckWhetherInPeriodWakeup() ) {
                ret = false;
                SMLK_LOGD("[newdebug] direct network InPeriodWakeup");
            } else {
                if (m_collect_last_frame_flag&LastFrameCollectFlags::SL_0F3B_1S_COLLECT) {
                    ret = true;
                    m_collect_last_frame_flag &= ~LastFrameCollectFlags::SL_0F3B_1S_COLLECT;
                    SMLK_LOGI("[newdebug] direct network collected last frame noe reset m_0f3b_timer");
                    m_0f3b_timer->Reset();
                }
            }
        }
    } else {
        if (m_internal_flags&InternalFlags::SL_FLAG_IGNON) {
            ret = true;
        } else {
            if ( CheckWhetherInPeriodWakeup() ) {
                SMLK_LOGD("[newdebug] indirect network InPeriodWakeup");
                ret = false;
            } else {
                if (m_collect_last_frame_flag&LastFrameCollectFlags::SL_0F3B_1S_COLLECT) {
                    ret = true;
                    m_collect_last_frame_flag &= ~LastFrameCollectFlags::SL_0F3B_1S_COLLECT;
                    SMLK_LOGI("[newdebug] indirect network collected last frame now reset m_0f3b_timer ");
                    m_0f3b_timer->Reset();
                }
            }
        }
    }
    return ret;
}

/******************************************************************************
 * NAME: CheckLocationReportContidion
 *
 * DESCRIPTION: Check 0200 Sampling Condition
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 * 0200 need report last frame when igoff by 30s period
 *****************************************************************************/
bool CollectionService::CheckLocationReportContidion()
{
    SMLK_LOGD("[newdebug] CheckLocationReportContidion");
    bool ret = false;
    if (m_internal_flags&InternalFlags::SL_FLAG_DIRECT_CONTROL) {
        if (m_internal_flags&InternalFlags::SL_FLAG_CAN_NORMAL) {
            ret = true;
            SMLK_LOGD("[newdebug] direct network can normal");
        } else {
            if ( CheckWhetherInPeriodWakeup() ) {
                ret = true;
                SMLK_LOGD("[newdebug] direct network InPeriodWakeup");
            } else {
                SMLK_LOGD("[newdebug] direct network collect last frame and not in wakeup");
                if (m_collect_last_frame_flag&LastFrameCollectFlags::SL_0200_COLLECT) {
                    ret = true;
                    m_collect_last_frame_flag &= ~LastFrameCollectFlags::SL_0200_COLLECT;
                    SMLK_LOGI("[newdebug] direct network collected last frame now reset m_0200_timer");
                    m_0200_timer->Reset();
                }
            }
        }
    } else {
        if (m_internal_flags&InternalFlags::SL_FLAG_IGNON) {
            SMLK_LOGD("[newdebug] indirect network SL_FLAG_IGNON");
            ret = true;
        } else {
            SMLK_LOGD("[newdebug] indirect network SL_FLAG_IGNOFF");
            if ( CheckWhetherInPeriodWakeup() ) {
                SMLK_LOGD("[newdebug] indirect network InPeriodWakeup");
                ret = true;
            } else {
                SMLK_LOGD("[newdebug] not in wakeup");
                if (m_collect_last_frame_flag&LastFrameCollectFlags::SL_0200_COLLECT) {
                    ret = true;
                    m_collect_last_frame_flag &= ~LastFrameCollectFlags::SL_0200_COLLECT;
                    SMLK_LOGI("[newdebug] indirect network have collect last frame now reset m_0200_timer reset");
                    m_0200_timer->Reset();
                }
            }
        }
    }
    return ret;
}

bool CollectionService::CheckSamplingCondition()
{
    bool ret = false;

    if ( 0 != (m_internal_flags & InternalFlags::SL_FLAG_DIRECT_CONTROL)) {
        if( m_internal_flags & InternalFlags::SL_FLAG_PERIOD_WAKEUP)
        {
            ret = true;
        }
        else
        {
            if (0 != (m_internal_flags & InternalFlags::SL_FLAG_CAN_NORMAL)) {
                ret = true;
            }
        }

    } else {
        // indirect and igon
        if( m_internal_flags & InternalFlags::SL_FLAG_PERIOD_WAKEUP)
        {
            ret = true;
        }
        else
        {
            if (0 != (m_internal_flags & InternalFlags::SL_FLAG_IGNON)) {
                ret = true;
            }
        }
    }
    return ret;
}


/******************************************************************************
 * NAME: OnTimer
 *
 * DESCRIPTION: timer event handler
 *
 * PARAMETERS:
 *         id:  timer ID
 *      param:  exra timer parameter, usually be the timer tirgger index
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnTimer(IN SMLK_UINT32 id, IN SMLK_UINT32 /*param*/)
{
    switch ( id ) {
        case SL_TIMER_1S:
#ifdef SL_SIMULATOR
            GenerateRandomValues();
#endif
            if ( Check1sReportContidion() ) {

                SamplingPeriod1S();

                auto now    = std::chrono::system_clock::now();
                auto ms     = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

                if ( m_internal_flags & InternalFlags::SL_FLAG_SYSTIME_SYN ) {
                    struct tm       tm;
                    std::time_t tt = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
                    localtime_r(&tt, &tm);
                    if ( tm.tm_hour == 23 && tm.tm_min >= 58 && !m_timer_shared ) {
                        decltype(ms)    ms_offset = 60 - tm.tm_min;
                        ms_offset *= 60;
                        ms_offset -= tm.tm_sec;
                        ms_offset *= 1000;
                        ms_offset -= ms;
                        ms_offset += 400;   //补偿400ms


                        m_timer_shared = std::make_shared<Timer<std::chrono::milliseconds>>(
                            0,
                            (int)ms_offset,
                            [this](IN std::uint32_t){
                                Post(MsgHead(M_SL_MSGID_DAY_CHANGED, 0, 0, 0, 0, 0), nullptr, 0);
                            }
                        );
                        m_timer_shared->Start();
                    }
                }
            }

            break;
        case SL_TIMER_30S:
            if ( Check30sReportContidion() ) {
                SamplingPeriod30S();
            }
            break;
        case SL_TIMER_REPORT:
#if 0
            if ((0 == (m_internal_flags & InternalFlags::SL_FLAG_PERIOD_WAKEUP))
                && (0 == (m_internal_flags & InternalFlags::SL_FLAG_IGNON)) ) {
                    // case: tester set sleep period and keep usb connected
                    SMLK_LOGD(" [locationtest] ig off and is not rtc wakeup, should not sample and report location now");
            } else
#endif
             {

#ifdef SL_DEBUG
                SMLK_LOGD(" [locationtest] SL_TIMER_REPORT call report!! m_internal_flags is %d", m_internal_flags);
#endif
                if ( CheckLocationReportContidion() ) {
                    ReportLocation();
                }
            }
            break;
        case SL_TIMER_STATISTICS:
            if ( CheckSamplingCondition() ) {
                if (m_internal_flags & InternalFlags::SL_FLAG_TRIP_START
                && m_internal_flags & InternalFlags::SL_FLAG_IGNON) {
                    ReportStatistics();
                } else {
                    SMLK_LOGI("not report statistic now for ig and speed ");
                }
            }
            break;
        case SL_TIMER_STATISTICS_STORE:
            if ( CheckSamplingCondition() ) {
                StoreStatistics();
            }
            break;
        case SL_TIMER_1S_CACHE:
            if ( CheckSamplingCondition() ) {
                CacheAllSignals();
            }
            break;
        case SL_TIMER_100MS:
            break;
        default:
            break;
    }
}

/******************************************************************************
 * NAME: OnMcuNotify
 *
 * DESCRIPTION: MCU service event handler
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnMcuNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_seq ) {
        case MCU::SL_CAN_STATUS_NOTIFY:
            {
              if (CanBusState::CAN_NORMAL == head.m_extra) {
                if ( 0 == (m_internal_flags & InternalFlags::SL_FLAG_CAN_NORMAL) ) {
                    m_internal_flags |= InternalFlags::SL_FLAG_CAN_NORMAL;
                    OnCanBusWakeup(head, data);
                    SMLK_LOGD("dm1debug CanBusState::CAN_NORMAL");
                }
              } else if (CanBusState::CAN_SLEEP == head.m_extra) {
                if ( 0 != (m_internal_flags & InternalFlags::SL_FLAG_CAN_NORMAL) ) {
                    m_internal_flags &= ~InternalFlags::SL_FLAG_CAN_NORMAL;
                    OnCanBusSleep(head, data);
                    SMLK_LOGD("dm1debug CanBusState::CAN_SLEEP");
                }
              }
            }
        break;
        case MCU::SL_SERVICE_READY:
            // MCU service will always broadcast current IGN status, so do not need to get the state
            break;
        case MCU::SL_IGN_NOTIFY:
            {
                if ( IgnState::SL_ON == head.m_extra ) {
                    if ( 0 == (m_internal_flags & InternalFlags::SL_FLAG_IGNON) ) {
                        m_internal_flags |= InternalFlags::SL_FLAG_IGNON;
                        m_internal_flags &= ~InternalFlags::SL_FLAG_SET_SIGNAL_INVALID;
                        OnIgnOn(head, data);
                    }
                } else {
                    if ( 0 != (m_internal_flags & InternalFlags::SL_FLAG_IGNON) ) {
                        m_internal_flags &= ~InternalFlags::SL_FLAG_IGNON;
                        OnIgnOff(head, data);
                    }
                }
            }
            break;

        default:
            SMLK_LOGE("[MCU] unsupported MCU event notify: 0x%08x", head.m_seq);
            break;
    }
}

/******************************************************************************
 * NAME: OnLocation
 *
 * DESCRIPTION: location service event handler
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnLocation(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_seq ) {
        case Location::SL_SERVICE_READY:
            GetGNSS();
            break;
        case Location::SL_SERVICE_EXIT:
            SMLK_LOGI("[GNSS]antenna SL_SERVICE_EXIT!!!");
            m_cached_signals.find(vehicle::GNSSLocatedStatus)->second = 1;
            break;
        case Location::SL_ANTENNA_CHANGED:
            {
                static SMLK_UINT8 dismantle_gnss_antenna_flag = 0;
                smartlink_sdk::GnssAntennaState state;
                state = (smartlink_sdk::GnssAntennaState)head.m_protocol;
                SMLK_LOGI("[GNSS]get state is %d", state);
                auto &signal    = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
                CLRBIT(m_dismantle_tbox_status, DismateTboxMask::SL_GNSS_ANTENNA_ABSENT);
                signal.AssignEncoded(signal.Encoded() & ~(AlarmMask::SL_GNSS_ANTENNA | AlarmMask::SL_GNSS_SHORT | AlarmMask::SL_GNSS_FAULT));
                if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_PRESENT == state ) {
                    if (0 == dismantle_gnss_antenna_flag) {
                        SMLK_LOGD("[GNSS] antenna state already present");
                        return;
                    }
                    dismantle_gnss_antenna_flag = 0;
                    SMLK_LOGI("[GNSS]antenna state changed to normal");
                    Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, M_SL_EVID_DISMANTLE_TBOX, GpsAndTelStatusCheck::TEL_NEED_CHECK, 0, 0, DismantleReport::EVENT_NOT_REPORT), nullptr, 0);

                } else if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_ABSENT == state ) {
                    SMLK_LOGE("[GNSS] antenna state changed to absent");
                    m_cached_signals.find(vehicle::GNSSLocatedStatus)->second = 1;
                    SETBIT(m_dismantle_tbox_status, DismateTboxMask::SL_GNSS_ANTENNA_ABSENT);
                    signal.AssignEncoded(signal.Encoded() & ~(AlarmMask::SL_GNSS_ANTENNA | AlarmMask::SL_GNSS_SHORT | AlarmMask::SL_GNSS_FAULT));
                    signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_GNSS_ANTENNA);
                    // auto set located status
                    if (1 == dismantle_gnss_antenna_flag) {
                        SMLK_LOGD("[GNSS] antenna state already absent");
                        return;
                    }
                    dismantle_gnss_antenna_flag = 1;
                    SMLK_LOGD("post M_SL_EVID_DISMANTLE_TBOX dueto gnss break");
                    Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, M_SL_EVID_DISMANTLE_TBOX, GpsAndTelStatusCheck::TEL_NEED_CHECK, 0, 0, DismantleReport::EVENT_REPORT), nullptr, 0);
                } else if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_SHORT_CIRCUIT == state ) {
                    SMLK_LOGE("[GNSS]antenna state changed to short");
                    dismantle_gnss_antenna_flag = 2;
                    signal.AssignEncoded(signal.Encoded() & ~(AlarmMask::SL_GNSS_ANTENNA | AlarmMask::SL_GNSS_SHORT | AlarmMask::SL_GNSS_FAULT));
                    signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_GNSS_SHORT);
                } else if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_INVALID == state ) {
                    SMLK_LOGE("[GNSS]antenna state changed to invalid");
                    dismantle_gnss_antenna_flag = 2;
                    signal.AssignEncoded(signal.Encoded() & ~(AlarmMask::SL_GNSS_ANTENNA | AlarmMask::SL_GNSS_SHORT | AlarmMask::SL_GNSS_FAULT));
                    signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_GNSS_FAULT);
                } else {
                    SMLK_LOGE("[GNSS]unsupported antenna state: 0x%02X", static_cast<SMLK_UINT8>(state));
                }
            }
            break;
        case Location::SL_SATELLITE_CHANGED:
            {
                if ( sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8) != data.size() ) {
                    // we should never get here
                    SMLK_LOGE("[GNSS] unexpected data length for satellite changed notification: %d", (SMLK_UINT8)data.size());
                    return;
                }
                // 20210927:modify by latest requirement use visiable num
                m_cached_signals.find(vehicle::GNSSSatelliteUsed)->second = (int)data[1];
            }
            break;
        case Location::SL_LOCATION_CHANGED:
            {
                if ( sizeof(InternalStruct::LocatingInf) != data.size() ) {
                    // we should never get here
                    SMLK_LOGE("[GNSS] unexpected data length for location changed notification: %d", (SMLK_UINT8)data.size());
                    return;
                }

                InternalStruct::LocatingInf *inf = (InternalStruct::LocatingInf *)data.data();

                // if no fixed, no update
                if (inf->flags & InternalStruct::LocatingFlags::SL_FIXED)
                {
                    m_cached_signals.find(vehicle::GNSSSpeed)->second           = inf->speed;
                    m_cached_signals.find(vehicle::GNSSHeading)->second         = inf->heading;
                    m_cached_signals.find(vehicle::GNSSAltitude)->second        = inf->altitude;
                    m_cached_signals.find(vehicle::GNSSLatitude)->second        = inf->latitude;
                    m_cached_signals.find(vehicle::GNSSLongitude)->second       = inf->longitude;
                    m_cached_signals.find(vehicle::GNSSLocatedStatus)->second   = (inf->flags & InternalStruct::LocatingFlags::SL_FIXED) ? 0 : 1;
                    m_cached_signals.find(vehicle::GNSSTimeStamp)->second       = inf->utc * 0.001;         // ms --> s
                } else {
                    m_cached_signals.find(vehicle::GNSSLocatedStatus)->second = 1;
                }
            }
            break;
        default:
            SMLK_LOGE("[GNSS] unsupported location event: 0x%08X", head.m_seq);
            break;
    }
}

/******************************************************************************
 * NAME: OnSysPower
 *
 * DESCRIPTION: system power event handler
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnSysPower(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_seq ) {
        case PowerState::SL_SLEEPING:
            {
                SMLK_LOGI("[WakeupDebug] system sleeping notification");
                m_internal_flags &= ~InternalFlags::SL_FLAG_PERIOD_WAKEUP;
                std::string clock_id;
                std::uint64_t   utc = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                SMLK_LOGI("[WakeupDebug] NOW UTC: %u", (SMLK_UINT32)utc);
                SMLK_LOGI("[WakeupDebug] NOW m_wakeup_report_period: [%d]s", m_wakeup_report_period);
                utc += m_wakeup_report_period;
                SMLK_LOGI("[WakeupDebug] looking forward wakeup UTC: %u", (SMLK_UINT32)utc);
                // daq not set wakeup clock itself
                // use hirain 2 hours clock and do 0200 wakeup logic when receive rtc wakeup
#if 0
                auto return_code = smartlink_sdk::SysPower::GetInstance()->SetWakeupTime(clock_id, utc);
                if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
                    SMLK_LOGE("[WakeupDebug] fail to set next wakeup time to: %u", (SMLK_UINT32)utc);
                } else {
                    SMLK_LOGI("[WakeupDebug] success set next wakeup time to: %u", (SMLK_UINT32)utc);
                    SMLK_LOGI("[WakeupDebug] clock_id:                        %s", clock_id.c_str());
                    m_clock_id = clock_id;
                }
#endif
            }
            break;
        case PowerState::SL_WAKEUP:
            {
                std::string     clock_id(data.cbegin(), data.cend());
                SMLK_LOGI("[WakeupDebug] system wakeup notification");
                SMLK_LOGI("[WakeupDebug] head.m_protocol: 0x%02X", head.m_protocol);
                SMLK_LOGI("[WakeupDebug] clock_id:        %s", clock_id.c_str());
                SMLK_LOGI("[WakeupDebug] m_clock_id:      %s", m_clock_id.c_str());

                // if system wakeup accept timesync change m_timeSyncGap_ms once
                m_time_sync_flags &= ~TimeSyncSourceFlag::SL_FLAG_SYSTEM_WAKEUP_SYNC ;
                SMLK_LOGD("[timesyncDebug] m_timeSyncGap_ms is %lld", m_timeSyncGap_ms);
                SMLK_LOGD("[timesyncDebug] receive system wake up and reset m_timeSyncGap_ms to 0");
                m_timeSyncGap_ms = 0;

                // check if it is MPU RTC wake up
                // not check clock id :wakeup if rtc wake, no matter who's clock id
                if ( 0 != head.m_protocol ) {
                    m_clock_id.clear();
                    m_internal_flags    |= InternalFlags::SL_FLAG_PERIOD_WAKEUP;

                    GetGNSS();

                    if ( 0 == m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second.Encoded() ) {
                        SMLK_LOGI("[WakeupDebug] system wakeup notification and GNSS located valid, directly report!!!");
                        ReportLocation();
                        m_internal_flags &= ~InternalFlags::SL_FLAG_PERIOD_WAKEUP;
                        return;
                    }

                    SMLK_LOGI("[WakeupDebug] system wakeup notification and GNSS located not valid, lock wakeup!");
                    smartlink_sdk::SysPower::GetInstance()->LockWake(std::string(SL_PROCESS_NAME)); // 使用唤醒锁保持唤醒
                    m_wakeup_report_cnt = SL_WAKEUP_REPORT_COUNT_DEF;

                    if ( m_timer_flags&TimerFlags::SL_FLAG_0200_TIMER_START ) {
                        SMLK_LOGI("[WakeupDebug]m_0200_timer reset");
                        m_0200_timer->Reset();
                        m_0200_timer->ReStart();
                    } else {
                        if ( !CheckWhetherTimeSync() ) {
                            SMLK_LOGD("[WakeupDebug] time not sync but in wakeup start  0200report use systime");
                            m_0200_timer->Start();
                            m_timer_flags |= TimerFlags::SL_FLAG_0200_TIMER_START;
                        } else {
                            SMLK_LOGE("[WakeupDebug] !!!time not sync and not start something wrong !!!!");
                        }
                    }
                }
            }
            break;
        case PowerState::SL_STATE_CHANGED:
            {
                auto &signal = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
                if ( PowerLevel::SL_NORMAL == head.m_extra ) {
                    signal.AssignEncoded(signal.Encoded() & ~AlarmMask::SL_POWER_UNDER_VOLTAGE);
                } else if ( PowerLevel::SL_UNDER_VOLTAGE == head.m_extra ) {
                    signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_POWER_UNDER_VOLTAGE);
                }
            }
            break;
        default:
            SMLK_LOGE("[POWER] unsupported power event: 0x%08X", head.m_seq);
            break;
    }
}

/******************************************************************************
 * NAME: OnTelNotify
 *
 * DESCRIPTION: telephony event handler
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnTelNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_seq ) {
        case Tel::SL_ANTENNA_CHANGED:
            {
                SMLK_LOGD("[Tel] get 4G SL_ANTENNA_CHANGED");
// unsupport check slave antenna status now dueto hardware
#if 0   // enable it when hardware support slave antenna
                smartlink_sdk::ModemAntennaInfo slave_info;
                slave_info.type = smartlink_sdk::ModemAntennaType::E_MODEM_ANTENNA_TYPE_SLAVE;
                auto slave_return_code  = smartlink_sdk::Telephony::GetInstance()->GetModemAntennaInfo(slave_info);
#endif
                static SMLK_UINT8 dismantle_main_antenna_flag = 0;
                smartlink_sdk::ModemAntennaInfo master_info;
                master_info.type = smartlink_sdk::ModemAntennaType::E_MODEM_ANTENNA_TYPE_MASTER;
                auto master_return_code  = smartlink_sdk::Telephony::GetInstance()->GetModemAntennaInfo(master_info);
                if (smartlink_sdk::RtnCode::E_SUCCESS != master_return_code) {
                    SMLK_LOGE("[Tel] fail to get 4G antenna state, master_return_code = %d ", static_cast<std::int32_t>(master_return_code));
                } else {
                    if (smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_ABSENT == master_info.state) {
                        SMLK_LOGD("[Tel] get 4G master antenna break");
                        if ( 1 == dismantle_main_antenna_flag) {
                            SMLK_LOGD("[Tel] 4G master antenna alerady absent!!!");
                            return;
                        }
                        dismantle_main_antenna_flag = 1;
                        SETBIT(m_dismantle_tbox_status, DismateTboxMask::SL_TEL_MAIN_ANTENNA_ABSENT);
                        SMLK_LOGD(" post M_SL_EVID_DISMANTLE_TBOX dueto 4g break");
                        Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, M_SL_EVID_DISMANTLE_TBOX, GpsAndTelStatusCheck::GPG_NEED_CHECK, 0, 0, DismantleReport::EVENT_REPORT), nullptr, 0);
                    } else if (smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_PRESENT == master_info.state) {
                        SMLK_LOGD("[Tel] get 4G master antenna present");
                        if ( 0 == dismantle_main_antenna_flag) {
                            SMLK_LOGD("[Tel] 4G master antenna alerady present!!!");
                            return;
                        }
                        dismantle_main_antenna_flag = 0;
                        CLRBIT(m_dismantle_tbox_status, DismateTboxMask::SL_TEL_MAIN_ANTENNA_ABSENT);
                        Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, M_SL_EVID_DISMANTLE_TBOX, GpsAndTelStatusCheck::GPG_NEED_CHECK, 0, 0, DismantleReport::EVENT_NOT_REPORT), nullptr, 0);
                    }
#if 0   // enable it when hardware support slave antenna
                    if (smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_ABSENT == slave_info.state) {
                            SMLK_LOGD("[Tel] get 4G slave antenna break");
                            SETBIT(m_dismantle_tbox_status, DismateTboxMask::SL_TEL_SLAVE_ANTENNA_ABSENT);
                    } else if (smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_PRESENT == slave_info.state) {
                            SMLK_LOGD("[Tel]get 4G slave antenna present");
                            CLRBIT(m_dismantle_tbox_status, DismateTboxMask::SL_TEL_SLAVE_ANTENNA_ABSENT);
                    }
#endif
                    smartlink_sdk::TelRadioSignalInfo signal_info;
                    auto signal_return_code  = smartlink_sdk::Telephony::GetInstance()->GetSignalStrength(signal_info);
                    if (smartlink_sdk::RtnCode::E_SUCCESS != signal_return_code) {
                        SMLK_LOGE("[Tel] fail to get Signal Strength, signal_return_code = %d ", static_cast<std::int32_t>(signal_return_code));
                    } else {
                        SMLK_LOGD("[Tel] auto get rssi value is [%d]", signal_info.signal);
                        auto &signal = m_cached_signals.find(vehicle::SignalID::RSSI)->second;
                        signal =  signal_info.signal;
                    }
                }
            }
            break;
        case Tel::SL_SIGNAL_CHANGED:
            {
                auto &signal = m_cached_signals.find(vehicle::SignalID::RSSI)->second;
                signal = head.m_extra;
            }
            break;
        default:
            SMLK_LOGE("[TEL] unsupported telephony event:   0x%08X", head.m_seq);
            break;
    }
}

/******************************************************************************
 * NAME: OnSystimeSync
 *
 * DESCRIPTION: system time sync notification
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnSystimeSync(IN MsgHead &head, IN std::vector<SMLK_UINT8> &/*data*/)
{
    SMLK_LOGI("[timesyncDebug] OnSystimeSync");

    if ( head.m_seq & Systime::SL_SYSTIME_SYN_OK ) {
        SMLK_LOGI("[timesyncDebug] systime service sync status SUCCESS");

        if ( CheckWhetherTimeSync() ) {
            SMLK_LOGI("[timesyncDebug] systime time has already synchronized!!!");
            return;
        }

        auto stready_now    = std::chrono::steady_clock::now();
        auto system_now     = std::chrono::system_clock::now();

        // cache the last frame
        CacheAllSignals();

        // we do not care the drive event during the time system time not ready
        // for now we just handle the data for 1s bucket report and 30s bucket report and location report
        auto index = 0;
        while ( !m_cached_signals_pool.empty() ) {
            auto const time_span    = std::chrono::duration_cast<std::chrono::duration<double>>(stready_now - m_cached_signals_pool.front().first);
            auto const system_tp    = system_now - std::chrono::milliseconds((long long)(1000 * time_span.count()));

            for ( auto const &pair : m_cached_signals_pool.front().second ) {
                auto it = m_cached_signals.find(pair.first);
                if ( it != m_cached_signals.end() ) {
                    it->second = pair.second;
                }
            }
            m_cached_signals_pool.pop();

            // fix timestamp
            auto it = m_cached_signals.find(vehicle::SignalID::GNSSTimeStamp);
            if ( it != m_cached_signals.end() ) {
                it->second = std::chrono::duration_cast<std::chrono::seconds>(system_tp.time_since_epoch()).count();
            }

            // sampling 1s bucket signals
            SamplingPeriod1S();

            // sampling 30s bucket signals
            if ( 0 == index % m_trigger_period_30s ) {
                SamplingPeriod30S();
            }

            // report location
            // 不上报第一帧
            if ( (0 != index) && (0 == index % m_report_period) ) {
                ReportLocation();
            }
            // not process statisis data cause it has trip start check
            // TODO later
            index++;
        }
        m_internal_flags |= InternalFlags::SL_FLAG_SYSTIME_SER | InternalFlags::SL_FLAG_SYSTIME_SYN;

        if ( m_timer_shared ) {
            // m_timer_shared->Stop();
            m_timer_shared->Reset();
            SMLK_LOGI("[timesyncDebug] stop and reset m_timer_shared");
        }

        // m_timer->Stop();

        auto now    = std::chrono::system_clock::now();
        auto ms     = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

        // make sure we almost always trigger the 1s timer at the 300ms of each seconds
        if ( ms <= SL_PERIOD_TRIGGER_POINT_MS ) {
            std::this_thread::sleep_for(std::chrono::milliseconds(SL_PERIOD_TRIGGER_POINT_MS - ms));
        } else {
            // here since we may sleep up to 1s, so if we just got a term signal during this time, we may stop until 1 second later
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 + SL_PERIOD_TRIGGER_POINT_MS - ms));
        }

        SMLK_LOGD("[timesyncDebug] start normal timer");
        if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_0F3B_TIMER_START) ) {
            if (0 != m_upload_period_1s_ms) {
                SMLK_LOGD(" m_0f3b_timer start!");
                m_0f3b_timer->Start();
                m_timer_flags |= TimerFlags::SL_FLAG_0F3B_TIMER_START;
            }
        } else {
            SMLK_LOGD(" [timesyncDebug] m_0f3b_timer restart!");
            m_0f3b_timer->Reset();
            m_0f3b_timer->ReStart();
            m_timer_flags |= TimerFlags::SL_FLAG_0F3B_TIMER_START;
        }

        if (m_timer_flags & TimerFlags::SL_FLAG_0F3C_TIMER_START) {
            SMLK_LOGD(" [timesyncDebug] m_0f3c_timer restart!");
            m_0f3c_timer->Reset();
            m_0f3c_timer->ReStart();
        }

        int diffTime = ( 30 - (index % 30));
        SMLK_LOGD("[timesyncDebug] 30s diff [%d]", diffTime);
        std::thread diffThread = std::thread([this, diffTime]{
            //if diffTime = 30 , last period is 0f3b-30 & 0200, so direact run timer.
            if (diffTime != 30)
            {
                sleep(diffTime);
                SMLK_LOGD("[timesyncDebug] diff thread sleep over");
                MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_30S, 0, 0, 0, 0);
                Post(head, nullptr, 0);

                MsgHead head1(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_REPORT, 0, 0, 0, 0);
                Post(head1, nullptr, 0);
            }

            if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_0200_TIMER_START) ) {
                    SMLK_LOGD("[timesyncDebug] m_0200_timer start!");
                    m_0200_timer->Start();
                    m_timer_flags |= TimerFlags::SL_FLAG_0200_TIMER_START;
                } else {
                    SMLK_LOGD("[timesyncDebug] m_0200_timer restart!");
                    m_0200_timer->Reset();
                    m_0200_timer->ReStart();
                    m_timer_flags |= TimerFlags::SL_FLAG_0200_TIMER_START;
                }

                if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_0F3B_30S_TIMER_START) ) {
                    SMLK_LOGD(" [timesyncDebug] m_0f3b_30s_timer start!");
                    m_0f3b_30s_timer->Start();
                    m_timer_flags |= TimerFlags::SL_FLAG_0F3B_30S_TIMER_START;
                } else {
                    SMLK_LOGD(" [timesyncDebug] m_0f3b_30s_timer restart!");
                    m_0f3b_30s_timer->Reset();
                    m_0f3b_30s_timer->ReStart();
                    m_timer_flags |= TimerFlags::SL_FLAG_0F3B_30S_TIMER_START;
                }
        });
        diffThread.detach();
    } else {
        SMLK_LOGE("systime service sync status FAIL");
    }
}

/******************************************************************************
 * NAME: OnSystimeReady
 *
 * DESCRIPTION: system time service ready notification
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnSystimeReady(IN MsgHead &head, IN std::vector<SMLK_UINT8> &)
{
    if ( head.m_seq & Systime::SL_SYSTIME_SER_OK ) {
        SMLK_LOGI("systime service online");
    } else {
        SMLK_LOGE("systime service offline");
    }
}

/******************************************************************************
 * NAME: OnCanBusWakeup
 *
 * DESCRIPTION: direct can control : sampling msg timing after can wakeup
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnCanBusWakeup(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &/*data*/)
{
    SMLK_LOGI(" [networkDebug] OnCanBusWakeup !");

    // can唤醒后 hirain中间件GetClockInfo获得的timesync 不可靠 所以应用先缓存数据 等待被动授时通知
    m_time_sync_flags &= ~TimeSyncSourceFlag::SL_FLAG_SYSTEM_WAKEUP_SYNC ;
    // m_internal_flags &= ~InternalFlags::SL_FLAG_SYSTIME_SYN;
    m_timeSyncGap_ms = 0;

    if ( m_internal_flags & InternalFlags::SL_FLAG_DIRECT_CONTROL) {
        m_collect_last_frame_flag |= LastFrameCollectFlags::SL_0200_COLLECT;
        m_collect_last_frame_flag |= LastFrameCollectFlags::SL_0F3B_1S_COLLECT;
        m_collect_last_frame_flag |= LastFrameCollectFlags::SL_0F3B_30S_COLLECT;

        if (m_internal_flags&InternalFlags::SL_FLAG_PERIOD_WAKEUP) {
            if ( m_timer_flags&TimerFlags::SL_FLAG_0200_TIMER_START ) {
                SMLK_LOGI("[wakeupdebug] in wakeup and receive can bus wake reset 0200 timer  ");
                m_0200_timer->Reset();
            }
            m_internal_flags &= ~InternalFlags::SL_FLAG_PERIOD_WAKEUP;
            smartlink_sdk::SysPower::GetInstance()->UnLockWake(std::string(SL_PROCESS_NAME));
        }

        if ( CheckWhetherTimeSync() ) {
            SMLK_LOGI("time has sync OnCanBusWakeup !");
            auto now    = std::chrono::system_clock::now();
            auto ms     = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
            // make sure we almost always trigger the 1s timer at the 300ms of each seconds
            if ( ms <= SL_PERIOD_TRIGGER_POINT_MS ) {
                std::this_thread::sleep_for(std::chrono::milliseconds(SL_PERIOD_TRIGGER_POINT_MS - ms));
            } else {
                // here since we may sleep up to 1s, so if we just got a term signal during this time, we may stop until 1 second later
                std::this_thread::sleep_for(std::chrono::milliseconds(1000 + SL_PERIOD_TRIGGER_POINT_MS - ms));
            }

            if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_0F3B_TIMER_START) ) {
                SMLK_LOGI(" m_0f3b_timer start!");
                m_0f3b_timer->Start();
                m_timer_flags |= TimerFlags::SL_FLAG_0F3B_TIMER_START;
            } else {
                SMLK_LOGI(" m_0f3b_timer restart!");
                m_0f3b_timer->Reset();
                m_0f3b_timer->ReStart();
                m_timer_flags |= TimerFlags::SL_FLAG_0F3B_TIMER_START;
            }

            if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_0F3B_30S_TIMER_START) ) {
                SMLK_LOGI(" m_0f3b_30s_timer start!");
                m_0f3b_30s_timer->Start();
                m_timer_flags |= TimerFlags::SL_FLAG_0F3B_30S_TIMER_START;
            } else {
                SMLK_LOGI(" m_0f3b_30s_timer restart!");
                m_0f3b_30s_timer->Reset();
                m_0f3b_30s_timer->ReStart();
                m_timer_flags |= TimerFlags::SL_FLAG_0F3B_30S_TIMER_START;
            }

            if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_0200_TIMER_START) ) {
                SMLK_LOGI("m_0200_timer start!");
                m_0200_timer->Start();
                m_timer_flags |= TimerFlags::SL_FLAG_0200_TIMER_START;
            } else {
                SMLK_LOGI("m_0200_timer restart!");
                m_0200_timer->Reset();
                m_0200_timer->ReStart();
                m_timer_flags |= TimerFlags::SL_FLAG_0200_TIMER_START;
            }
        } else {
            SMLK_LOGI("[timesyncDebug] canbus wakeup and not ever time sync");
            // shared timer start
            if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_CACHE_TIMER_START) ) {
                SMLK_LOGI(" [timesyncDebug] m_timer_shared start!");
                m_timer_shared->Start();
                m_timer_flags |= TimerFlags::SL_FLAG_CACHE_TIMER_START;
            } else {
                SMLK_LOGI(" [timesyncDebug] m_timer_shared restart!");
                m_timer_shared->Reset();
                m_timer_shared->ReStart();
                m_timer_flags |= TimerFlags::SL_FLAG_CACHE_TIMER_START;
            }
        }
    }
}

/******************************************************************************
 * NAME: OnCanBusSleep
 *
 * DESCRIPTION: direct can control : stop sampling msg timing after can sleep
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnCanBusSleep(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &/*data*/)
{
    SMLK_LOGI("[networkDebug] CollectionService::OnCanBusSleep");
    if (m_internal_flags & InternalFlags::SL_FLAG_DIRECT_CONTROL) {
        if ( !CheckWhetherTimeSync() ) {
            SMLK_LOGI(" [timesyncDebug] time not sync now ");
        }
    }
}


/******************************************************************************
 * NAME: OnIgnOn
 *
 * DESCRIPTION: IGN ON event handler
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnIgnOn(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &/*data*/)
{
    SMLK_LOGI("[igDebug]======CollectionService::OnIgnOn=====");
    // if indirect network start sampling 0f3b and 0200
    if ( 0 == (m_internal_flags & InternalFlags::SL_FLAG_DIRECT_CONTROL) ) {
        m_collect_last_frame_flag |= LastFrameCollectFlags::SL_0200_COLLECT;
        m_collect_last_frame_flag |= LastFrameCollectFlags::SL_0F3B_1S_COLLECT;
        m_collect_last_frame_flag |= LastFrameCollectFlags::SL_0F3B_30S_COLLECT;

        if (m_internal_flags&InternalFlags::SL_FLAG_PERIOD_WAKEUP) {
            if ( m_timer_flags&TimerFlags::SL_FLAG_0200_TIMER_START ) {
                SMLK_LOGI("[wakeupdebug] in wakeup and receive ign on reset 0200 timer ");
                m_0200_timer->Reset();
            }
            m_internal_flags &= ~InternalFlags::SL_FLAG_PERIOD_WAKEUP;
            smartlink_sdk::SysPower::GetInstance()->UnLockWake(std::string(SL_PROCESS_NAME));
        }

        if ( CheckWhetherTimeSync() ) {
            SMLK_LOGI("[newdebug] igon start timer");
            auto now_systime    = std::chrono::system_clock::now();
            auto ms_abs     = std::chrono::duration_cast<std::chrono::milliseconds>(now_systime.time_since_epoch()).count() % 1000;
            // make sure we almost always trigger the 1s timer at the 300ms of each seconds
            if ( ms_abs <= SL_PERIOD_TRIGGER_POINT_MS ) {
                std::this_thread::sleep_for(std::chrono::milliseconds(SL_PERIOD_TRIGGER_POINT_MS - ms_abs));
            } else {
                // here since we may sleep up to 1s, so if we just got a term signal during this time, we may stop until 1 second later
                std::this_thread::sleep_for(std::chrono::milliseconds(1000 + SL_PERIOD_TRIGGER_POINT_MS - ms_abs));
            }

            if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_0F3B_TIMER_START) ) {
                SMLK_LOGI("m_0f3b_timer start");
                m_0f3b_timer->Start();
                m_timer_flags |= TimerFlags::SL_FLAG_0F3B_TIMER_START;
            } else {
                SMLK_LOGI("m_0f3b_timer restart");
                m_0f3b_timer->Reset();
                m_0f3b_timer->ReStart();
                m_timer_flags |= TimerFlags::SL_FLAG_0F3B_TIMER_START;
            }

            if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_0F3B_30S_TIMER_START) ) {
                SMLK_LOGI(" m_0f3b_30s_timer start!");
                m_0f3b_30s_timer->Start();
                m_timer_flags |= TimerFlags::SL_FLAG_0F3B_30S_TIMER_START;
            } else {
                SMLK_LOGI(" m_0f3b_30s_timer restart!");
                m_0f3b_30s_timer->Reset();
                m_0f3b_30s_timer->ReStart();
                m_timer_flags |= TimerFlags::SL_FLAG_0F3B_30S_TIMER_START;
            }

            if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_0200_TIMER_START) ) {
                SMLK_LOGI("m_0200_timer start");
                m_0200_timer->Start();
                m_timer_flags |= TimerFlags::SL_FLAG_0200_TIMER_START;
            } else {
                SMLK_LOGI("timerDebug m_0200_timer restart");
                m_0200_timer->Reset();
                m_0200_timer->ReStart();
                m_timer_flags |= TimerFlags::SL_FLAG_0200_TIMER_START;
            }

            auto now    = std::chrono::system_clock::now();
            auto ms     = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
            std::time_t tt = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
            struct tm       tm;
            localtime_r(&tt, &tm);

            if ( tm.tm_hour == 23 && tm.tm_min >= 58 && !m_timer_shared ) {
                decltype(ms)    ms_offset = 60 - tm.tm_min;
                ms_offset *= 60;
                ms_offset -= tm.tm_sec;
                ms_offset *= 1000;
                ms_offset -= ms - 1;

                m_timer_shared = std::make_shared<Timer<std::chrono::milliseconds>>(
                    0,
                    (int)ms_offset,
                    [this](IN std::uint32_t){
                        Post(MsgHead(M_SL_MSGID_DAY_CHANGED, 0, 0, 0, 0, 0), nullptr, 0);
                    }
                );
                m_timer_shared->Start();
            }

        } else {
            SMLK_LOGI("[timesyncDebug] time not sync, restart cache timer");
            // shared timer
            if ( 0 == (m_timer_flags&TimerFlags::SL_FLAG_CACHE_TIMER_START) ) {
                SMLK_LOGI(" m_timer_shared start");
                m_timer_shared->Start();
                m_timer_flags |= TimerFlags::SL_FLAG_CACHE_TIMER_START;
            } else {
                SMLK_LOGI(" m_timer_shared restart");
                m_timer_shared->Reset();
                m_timer_shared->ReStart();
                m_timer_flags |= TimerFlags::SL_FLAG_CACHE_TIMER_START;
            }
        }
    }
    m_trip_uploaded_spn.clear();
    m_actived_dm1.clear();
    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageDM1>     p_dm1   = std::static_pointer_cast<MessageDM1>(m_protocol_messages[protocol][MessageType::SL_FAULT_DM1]);
        // clear all the reported faults during IGN ON
        decltype(p_dm1->m_reported_faults)  empty;
        p_dm1->m_reported_faults.swap(empty);
    }

}

/******************************************************************************
 * NAME: OnIgnOff
 *
 * DESCRIPTION: IGN OFF event handler
 *
 * PARAMETERS:
 *
 * RETURN:
 * NOTE:
 *****************************************************************************/
void CollectionService::OnIgnOff(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &/*data*/)
{
    SMLK_LOGI("[igDebug] ======CollectionService::OnIgnOff============");
    m_cached_signals.find(vehicle::SignalID::EvVehicleStatus)->second = 0;
    // indirect network control : system ign off, directly report once and do not report anymore
    if (0 == (m_internal_flags & InternalFlags::SL_FLAG_DIRECT_CONTROL)) {
        SMLK_LOGI("can indirect control collect last frame and stop timer");
    }

    if (m_internal_flags & InternalFlags::SL_FLAG_TRIP_START) {
        ReportStatistics();
        m_internal_flags &= ~InternalFlags::SL_FLAG_TRIP_START;
        if (m_timer_flags & TimerFlags::SL_FLAG_0F3C_TIMER_START)
        {
            SMLK_LOGI("m_0f3c_timer stop");
            m_0f3c_timer->Reset();
            m_0f3c_timer->Stop();
            m_timer_flags &= ~TimerFlags::SL_FLAG_0F3C_TIMER_START;
        }

        // vec and daq cannot inform each other ignoff event
        // so daq sholud also clear itself cached signal which should be cleard when trip end
        m_cached_signals.find(vehicle::SignalID::StatisticsTachographVehicleSpeedDistance)->second = 0;
        m_cached_signals.find(vehicle::SignalID::StatisticsTripFuelUsed)->second = 0;

    } else {
        SMLK_LOGI("engine not ever over 0, should not report Statistics when ig off");
    }

    // dm1 related clear data
    auto &signal = m_cached_signals.find(vehicle::SignalID::MalfunctionIndicatorLampStatus)->second;
    // TAPD 1028272 FAW requirement
    signal =  0xFF;

    if (m_actived_dm1.size() > 0) {
        SMLK_LOGI("[dm1debug] has left uncleared error");
        // IGN OFF notify, we need finally send clear all left faults message
        static const std::vector<SMLK_UINT8> data = {0};
        MsgHead     head;
        head.m_extra = ((SMLK_UINT32)FaultEventMask::SL_EVENT_DM1 << 24) | ((SMLK_UINT32)FaultEventFalg::SL_EVENT_STOP << 16);
        head.m_protocol = 0;
        OnFaultEvent(head, data);
        m_actived_dm1.clear();
    }
    m_dm1_src_all_spn_map.clear();
    g_dm1_data_pool.clear();
    // std::lock_guard<std::mutex> lock(g_dm1_trip_uploaded_mutex);
    m_trip_uploaded_spn.clear();

    m_cached_tires.clear();

    // daq not do auto report logic for igoff... only report by vehicle trigger
#if 0
    SMLK_UINT32 now_second = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    for ( auto const &pair : m_pending_events ) {
        for ( auto const &event : pair.second ) {
            const std::shared_ptr<InternalStruct::DriveEvent>   evp = std::static_pointer_cast<InternalStruct::DriveEvent>(event.second);
            std::vector<SMLK_UINT8> encoded;

            for ( auto protocol : m_protocols ) {
                std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

                ptr->m_id                   = pair.first;
                ptr->m_reversion            = SL_DRIVE_EVENT_REV;
                ptr->m_status               = evp->m_status;
                ptr->m_tire_info            = evp->m_tire;
                ptr->m_actual_minimum_soc   = evp->m_actual_minimum_soc;

                SMLK_UINT32 secs = now_second > (SMLK_UINT32)evp->m_location.m_timestamp.Val() ? now_second - (SMLK_UINT32)evp->m_location.m_timestamp.Val() : 0;

                if ( M_SL_EVID_TIREDNESS_DRIVING ) {
                    ptr->m_duration = secs / 60;
                } else {
                    ptr->m_duration = secs;
                }

                ptr->m_locations.clear();

                ptr->m_locations.push_back(evp->m_location);

                LocationInfo     loc;
                loc.m_status                = m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second;
                loc.m_latitude              = m_cached_signals.find(vehicle::SignalID::GNSSLatitude)->second;
                loc.m_longitude             = m_cached_signals.find(vehicle::SignalID::GNSSLongitude)->second;
                loc.m_altitude              = m_cached_signals.find(vehicle::SignalID::GNSSAltitude)->second;
                loc.m_heading               = m_cached_signals.find(vehicle::SignalID::GNSSHeading)->second;
                loc.m_speed                 = m_cached_signals.find(vehicle::SignalID::GNSSSpeed)->second.Val() * M_PER_S_TO_KM_PER_HOUR;
                loc.m_timestamp             = now_second;

                ptr->m_locations.push_back(loc);

                encoded.clear();
                ptr->Encode(encoded);

                IpcTspHead  head;
                std::memset(&head, 0x00, sizeof(head));

                head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
                head.protocol   = protocol;
                head.qos        = QOS_SEND_ALWAYS;

                TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
            }
        }
    }
    m_pending_events.clear();
#endif
}

/******************************************************************************
 * NAME: OnDriveEvent
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGI("OnDriveEvent event id: %u", head.m_seq);

    switch ( head.m_seq ) {
        case M_SL_EVID_SHARP_SLOW_DOWN:
            OnDriveEventSharpSlowdown(head, data);
            break;
        case M_SL_EVID_SPEEDING:
            OnDriveEventSpeeding(head, data);
            break;
        case M_SL_EVID_ACCIDENT:
            OnDriveEventAccident(head, data);
            break;
        case M_SL_EVID_TPMS_TIRE_WARN:
            OnDriveEventTireWarn(head, data);
            break;
        case M_SL_EVID_TIREDNESS_DRIVING:
            OnDriveEventTirednessDriving(head, data);
            break;
        case M_SL_EVID_DISMANTLE_TBOX:
            OnDriveEventDisMantleTbox(head);
            break;
        case M_SL_EVID_SOC_LOW:
            OnDriveEventLowSOC(head, data);
            break;
        case M_SL_EVID_DMS_WARN:
            OnDriveEventDMSWarn(head, data);
            break;
        case M_SL_EVID_FUEL_TANK_WARN:
            OnDriveEventFuelTankWarn(head, data);
            break;
        case M_SL_EVID_AGITATOR_FAULT:
            OnDriveEventAgitatorFault(head, data);
            break;
        case M_SL_EVID_GB_CHECK_VIN_ERROR:
            OnDriveEventGbCheckVinError(head);
            break;
        case M_SL_EVID_REFRIGERATOR_FAILURE:
            OnDriveEventRefrigeratorFailure(head,data);
            break;
        default:
            SMLK_LOGE("unsupported event id: %u", head.m_seq);
            break;
    }
}

void CollectionService::OnCheckDataValidTimeout()
{
#ifdef SL_DEBUG
    SMLK_LOGD("[dm1debug]On CheckDataValidTimeout");
    // SMLK_LOGD("[dm1debug] g_dm1_data_pool siez is %d", g_dm1_data_pool.size());
#endif
    std::set<SMLK_UINT32>   timeout_dm1;

    for (auto &pair : g_dm1_data_pool)
    {
        SMLK_LOGD("[dm1debug] spn [%d] time diff is [%u] ", pair.first ,
        (SMLK_UINT32)std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - pair.second.tp).count());

        // since last revice time is longer than 10 period cyle, mark valid to false
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - pair.second.tp).count() >=  CYCLE_TIMEOUT_COUNT * pair.second.cycle)
        {
            // spn timeout check
            if (true == pair.second.data_valid)
            {
                SMLK_LOGD("[dm1debug] spn index[%d] is time out!!!!", pair.first);
                timeout_dm1.insert(pair.first);
                pair.second.data_valid = false;
                // 排放相关src长断 mil  lamp 置为无效
                if (0 == pair.second.can_src) {
                    auto &signal = m_cached_signals.find(vehicle::SignalID::MalfunctionIndicatorLampStatus)->second;
                    SMLK_LOGD("[dm1debug] set mil lamp invalid");
                    signal.Invalid();
                }
            }
        }
    }

    // for can fd, it will have at most 64 bytes data
    static const std::size_t        max_bytes = 64;

    SMLK_UINT8  buffer[max_bytes * sizeof(FaultCodeT)];
    auto faults = reinterpret_cast<FaultCodesT *>(buffer);
    faults->count = 0;
    if (timeout_dm1.size() > 0 ) {
        for ( auto spn : timeout_dm1 )
        {
            faults->codes[faults->count].spn = spn;
            auto iter = g_dm1_data_pool.find(spn);
            if (g_dm1_data_pool.end() != iter) {
                faults->codes[faults->count].fmi    = iter->second.dm1_fmi;
                faults->codes[faults->count].src    = iter->second.can_src;

                // m_actived_dm1  clear timeout error
                InternalDTC temp_data;
                temp_data.dtc.FMI = iter->second.dm1_fmi;
                temp_data.dtc.SPN = spn;
                temp_data.dtc.SRC = iter->second.can_src;
                if ( m_actived_dm1.end() != m_actived_dm1.find(temp_data.id) ) {
                    SMLK_LOGD("[dm1debug] m_actived_dm1 erase [%d] !!!", temp_data.dtc.SPN);
                    m_actived_dm1.erase(temp_data.id);
                }

                auto iter_src = m_dm1_src_all_spn_map.find(iter->second.can_src);
                if ( m_dm1_src_all_spn_map.end() != iter_src) {
                    m_dm1_src_all_spn_map.erase(iter_src);
                }

                g_dm1_data_pool.erase(iter);
            } else {
                SMLK_LOGI("[dm1debug] timeout check g_dm1_data_pool not find spn info [%u] !!!", spn);
            }
            ++faults->count;
        }
    }

    MsgHead     msg_head;
    if ( faults->count > 0 ) {
        msg_head.m_extra = (((SMLK_UINT32)FaultEventMask::SL_EVENT_DM1 << 24) | ((SMLK_UINT32)FaultEventFalg::SL_EVENT_STOP << 16));
        msg_head.m_protocol = DM1_NEED_EXTRA_DATA;
        std::vector<SMLK_UINT8> bytes;
        bytes.insert(bytes.end(), &buffer[0], &buffer[sizeof(FaultCodesT) + std::size_t(faults->count) * sizeof(faults->codes[0])]);
        SMLK_LOGI("[dm1debug] timeout send timeout dm1 data!");
        OnFaultEventDM1(msg_head, bytes);
        timeout_dm1.clear();
    }
    SMLK_LOGD("[dm1debug] m_trip_uploaded_spn size is%d", m_trip_uploaded_spn.size());
    SMLK_LOGD("[dm1debug] m_dm1_src_all_spn_map size [%d]", m_dm1_src_all_spn_map.size());
#ifdef SL_DEBUG
    SMLK_LOGD("[dm1debug] g_dm1_data_pool siez is %d", g_dm1_data_pool.size());
#endif
}

/******************************************************************************
 * NAME: OnDM1Notify
 *
 * DESCRIPTION: DM1 raw CAN data notify
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDM1Notify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    if (!(m_internal_flags & InternalFlags::SL_FLAG_IGNON)) {
        SMLK_LOGD("[dm1debug] now is ign off not process dm1 data!!!");
        return;
    }
    if ( data.size() < sizeof(RawDM1) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(RawDM1));
        return;
    }

    auto dm1 = reinterpret_cast<const RawDM1 *>(data.data());

    // SMLK_LOGD("[dm1debug] dm1->can_src is %d", dm1->can_src);

    // mil 只用于传达排放相关的故障代码信息 当有一个排放相关的00 src 故障代码处于激活状态时才被点亮
    if ( 0 == dm1->can_src) {
        auto &signal = m_cached_signals.find(vehicle::SignalID::MalfunctionIndicatorLampStatus)->second;
        SMLK_LOGD("dm1->lamp_status_malfunction_indicator value is %d ", dm1->lamp_status_malfunction_indicator);
        signal =  dm1->lamp_status_malfunction_indicator;
        signal.SetValid();
        SMLK_LOGD("MalfunctionIndicatorLampStatus value is %f ", signal.Val());
    }

    std::size_t left    = data.size() - sizeof(RawDM1);
    // one single frame sample: DM[1] 02 41 00 FF BE 00 0C 01 FF FF
#if 0
    if ( 0 != left % sizeof(dm1->dtcs[0]) ) {
        SMLK_LOGE("invalid body size:   %u", data.size());
        return;
    }
#endif

    // for can fd, it will have at most 64 bytes data
    static const std::size_t        max_bytes = 64;

    if ( data.size() > max_bytes ) {
        SMLK_LOGE("invalid data received, data.size() = %u", data.size());
        return;
    }

    // for the DM1 can raw data, it will have 2 bytes lamp status, so the fault code number will be max_bytes - 2 at most
    // and those two bytes here we will used to save the fault count
    SMLK_UINT8  buffer[max_bytes * sizeof(FaultCodeT)];

    auto faults = reinterpret_cast<FaultCodesT *>(buffer);
    faults->count  = 0;

    // the dtc src in the second byte of mcu raw data
    SMLK_UINT8  src = dm1->can_src;
    SMLK_UINT32 current_spn = 0;
    std::set<SMLK_UINT32>   actived;
    auto dtc = &dm1->dtcs[0];

    std::vector<DM1_SPN> spn_vector;
    while ( left >= sizeof(dm1->dtcs[0]) ) {
        /**********************************************************************
         * For now, we only have the SPN conversion method 0 which will be looks
         * like as following (SAE J1939 version 4):
         * _____________________________________________________________________
         * | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 |
         * |87654321|87654321|87654321|87654321|87654321|87654321|
         * |        |        | SPN-L  | SPN-M  | SPN-H  |        |
         *
         * Byte 0: lamp status
         * Byte 1: reserved lamp status
         * Byte 2 ~ Byte 5: the first DTC
         * Byte 2 ~ 5 无故障填充00 00 00 00
         * SPN = Byte 4 >> 5;
         * SPN <<= 8;
         * SPN |= Byte 3
         * SPN << = 8;
         * SPN |= Byte 2
         *
         * CM = Byte 5 & 0x80 >> 7;
         * OC = Byte 5 & 0x7F;
        ***********************************************************************/
        InternalDTC internal;

        internal.dtc.SRC    = src;
        internal.dtc.FMI    = dtc->Byte2 & 0x1F;
        internal.dtc.SPN    = dtc->Byte2 >> 5;
        internal.dtc.SPN    <<= 8;
        internal.dtc.SPN    |= dtc->Byte1;
        internal.dtc.SPN    <<= 8;
        internal.dtc.SPN    |= dtc->Byte0;
        current_spn = internal.dtc.SPN;
        // SMLK_LOGD("[dm1debug]current  spn[%d]", current_spn);
        // SMLK_LOGD("[dm1debug]internal.dtc.SPN = [%d]", internal.dtc.SPN);

        actived.insert(internal.id);

        // update or insert new spn
        SMLK_UINT32 spn = internal.dtc.SPN;
        DM1DataInfo dm1_info;
        dm1_info.tp =  std::chrono::steady_clock::now();
        dm1_info.cycle = DM1_SIGNAL_CYCLE;
        dm1_info.data_valid = true;
        dm1_info.can_src = src;
        dm1_info.dm1_fmi = internal.dtc.FMI;

        // 是否是持续存在的spn
        if (0 != current_spn) {
            auto iter_dm1 = g_dm1_data_pool.find(current_spn);
            if ( (g_dm1_data_pool.end() != iter_dm1) ) {
                // 更新时间戳
                iter_dm1->second.can_src = dm1_info.can_src;
                iter_dm1->second.tp = dm1_info.tp;
                iter_dm1->second.data_valid = true;
                iter_dm1->second.dm1_fmi = dm1_info.dm1_fmi;
            } else {
                // 插入做超时计算
                auto iter_trip = m_trip_uploaded_spn.find(current_spn);
                if ( m_trip_uploaded_spn.end() == iter_trip ) {
                    SMLK_LOGD("[dm1debug] insert Spn new spn[%d]", current_spn);
                    g_dm1_data_pool.emplace(current_spn, dm1_info);
                }
            }
        }

        // 已上传的spn不处理
        if ( 0 != current_spn) {
            auto iter_trip = m_trip_uploaded_spn.find(current_spn);
            if ( m_trip_uploaded_spn.end() != iter_trip) {
                // SMLK_LOGD("[dm1debug] uploaded spn, not process");
                left -= sizeof(dm1->dtcs[0]);
                ++dtc;
                continue;
            } else {
                m_trip_uploaded_spn.emplace(current_spn);;
            }
        }

        // 处理新spn  spn != 0
        if ( m_actived_dm1.end() == m_actived_dm1.find(internal.id) && 0 != internal.dtc.SPN) {
            SMLK_LOGI("[dm1debug] DTC is a new DTC");

            SMLK_LOGD("[dm1debug] internal.dtc.SPN:    %u", internal.dtc.SPN);
            SMLK_LOGD("[dm1debug] internal.dtc.FMI:    %u", internal.dtc.FMI);
            SMLK_LOGD("[dm1debug] internal.dtc.SRC:    %u", internal.dtc.SRC);
            SMLK_LOGD("[dm1debug] internal.id:     0x%08X", internal.id);
            m_actived_dm1.insert(internal.id);

            faults->codes[faults->count].spn    = internal.dtc.SPN;
            faults->codes[faults->count].fmi    = internal.dtc.FMI;
            faults->codes[faults->count].src    = internal.dtc.SRC;
            ++faults->count;

            spn_vector.push_back(internal.dtc.SPN);
        }
        left -= sizeof(dm1->dtcs[0]);
        ++dtc;
    }
#if 0 // not check in case one frame
    if ( left != 0 ) {
        SMLK_LOGE("invalid body size: %u", data.size());
        return;
    }
#endif

    MsgHead     msg_head;
    if ( faults->count > 0 ) {
        msg_head.m_extra    = ((SMLK_UINT32)FaultEventMask::SL_EVENT_DM1 << 24) | ((SMLK_UINT32)FaultEventFalg::SL_EVENT_START << 16);
        msg_head.m_protocol = DM1_NEED_EXTRA_DATA;
        std::vector<SMLK_UINT8> bytes;
        bytes.insert(bytes.end(), &buffer[0], &buffer[sizeof(FaultCodesT) + std::size_t(faults->count) * sizeof(faults->codes[0])]);
        SMLK_LOGD("[dm1debug] call dm1 report bytes size is %d", bytes.size());
        OnFaultEvent(msg_head, bytes);
    }

        // 按照源地址归类将错误信息存储
        auto iter_src = m_dm1_src_all_spn_map.find(src);
        if (m_dm1_src_all_spn_map.end() != iter_src) {
            for ( auto spn : spn_vector ) {
                SMLK_LOGD("[dm1debug] check spn [%d]", spn);
                auto it_end = iter_src->second.end();
                if ( std::find(iter_src->second.begin(), iter_src->second.end(), spn) == it_end ) {
                    iter_src->second.push_back(spn);
                    SMLK_LOGD("[dm1debug] add spn [%d] in src [%d]", spn, src);
                }
            }
        } else {
            // check whether alerady have this src
            m_dm1_src_all_spn_map.emplace(std::make_pair(src, spn_vector));
            SMLK_LOGD("[dm1debug] m_dm1_src_all_spn_map [%d]", m_dm1_src_all_spn_map.size() );
        }

    faults->count = 0;
    if (0 == current_spn) {
        SMLK_LOGD("[dm1debug] src [%d] need clear", src);
        // 处理总线上主动发送取消的src的所有spn
        auto iter = m_dm1_src_all_spn_map.find(src);
        if (m_dm1_src_all_spn_map.end() != iter) {
            for (auto spn : iter->second ) {
                auto iter_spn = g_dm1_data_pool.find(spn);
                if (g_dm1_data_pool.end() != iter_spn) {
                    faults->codes[faults->count].src    = iter_spn->second.can_src;
                    faults->codes[faults->count].spn    = spn;
                    faults->codes[faults->count].fmi    = iter_spn->second.dm1_fmi;
                    ++faults->count;
                    // 清除需要取消的spn
                    InternalDTC internal;
                    internal.dtc.SRC = iter_spn->second.can_src;
                    internal.dtc.SPN = spn;
                    internal.dtc.FMI = iter_spn->second.dm1_fmi;
                    m_actived_dm1.erase(internal.id);

                    auto it = g_dm1_data_pool.find(spn);
                    if ( g_dm1_data_pool.end() != it) {
                        g_dm1_data_pool.erase(it);
                    }
                }
            }
            SMLK_LOGD("[dm1debug] erase src [%d]", src);
            m_dm1_src_all_spn_map.erase(iter);
        }
    }

    if ( faults->count > 0 ) {
        msg_head.m_extra    = ((SMLK_UINT32)FaultEventMask::SL_EVENT_DM1 << 24) | ((SMLK_UINT32)FaultEventFalg::SL_EVENT_STOP << 16);
        msg_head.m_protocol = DM1_NEED_EXTRA_DATA;
        std::vector<SMLK_UINT8> bytes;
        bytes.insert(bytes.end(), &buffer[0], &buffer[sizeof(FaultCodesT) + std::size_t(faults->count) * sizeof(faults->codes[0])]);
        SMLK_LOGI("[dm1debug] call dm1 clear bytes size is %d", bytes.size());
        OnFaultEventDM1(msg_head, bytes);
    }

#if 0
    faults->count = 0;
    for ( auto id : m_actived_dm1 ) {
        InternalDTC     internal;
        internal.id = id;
        if ( internal.dtc.SRC == src && 0 == current_spn) {
            // 新消息的src 与已经激活发送过的src重复 并且 spn = 0(Byte3-6) 视为此src下的spn都无效

            faults->codes[faults->count].spn    = internal.dtc.SPN;
            faults->codes[faults->count].fmi    = internal.dtc.FMI;
            faults->codes[faults->count].src    = internal.dtc.SRC;
            ++faults->count;

            SMLK_LOGI("dm1debug clear DTC");
            SMLK_LOGD("dm1debug internal.dtc.SPN:    %u", internal.dtc.SPN);
            SMLK_LOGD("dm1debug internal.dtc.FMI:    %u", internal.dtc.FMI);
            SMLK_LOGD("dm1debug internal.dtc.SRC:    %u", internal.dtc.SRC);
            SMLK_LOGD("dm1debug internal.id:         0x%08X", internal.id);
            m_actived_dm1.erase(id);
            continue;
        }
    }
#endif

}

/******************************************************************************
 * NAME: OnDriveEventSharpSlowdown
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventSharpSlowdown(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnDriveEventSharpSlowdown");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match with expected: %u", data.size(), expected);
        return;
    }

    if ( inf->location_num < 1 ) {
        SMLK_LOGE("no any position information provided!!!");
        return;
    }
    SMLK_LOGD("receive inf->location_num is %d", inf->location_num);

    std::vector<SMLK_UINT8> encoded;

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_SHARP_SLOW_DOWN;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        ptr->m_speed_start          = inf->location_info[0].speed;
        ptr->m_speed_stop           = inf->location_info[inf->location_num - 1].speed;
        ptr->m_duration             = inf->time / 1000;
        ptr->m_distance             = inf->odo;
        ptr->m_fuel_used            = inf->fuel;

        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            info.m_timestamp    = (inf->location_info[index].utc / 1000) - (m_timeSyncGap_ms/1000) ;

            ptr->m_locations.push_back(info);
        }

        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
}

/******************************************************************************
 * NAME: OnDriveEventSpeeding
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventSpeeding(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    // TODO: 报警标志处理
    SMLK_LOGD("OnDriveEventSpeeding");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match with expected: %u", data.size(), expected);
        return;
    }

    decltype(inf->location_num) index = inf->location_num - 1;
    if( 0x00 == inf->location_info[index].utc)
    {
        SMLK_LOGD("+++++++++++++++++++++++++++ 0x%08x", inf->location_info[index].utc);
        auto &signal    = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
        signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_SPEEDING);
        SMLK_LOGD("+++++++++++++++++++++++++++m_alarm_flags 0x%08x", signal.Encoded());
        return;
    }
    else
    {
        SMLK_LOGD("=========================== %ld", inf->location_info[index].utc);
        auto &signal    = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
        signal.AssignEncoded(signal.Encoded() & ~AlarmMask::SL_SPEEDING);
        SMLK_LOGD("===========================m_alarm_flags 0x%08x", signal.Encoded());
    }

    std::vector<SMLK_UINT8> encoded;

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_SPEEDING;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        ptr->m_speed_start          = inf->location_info[0].speed;
        ptr->m_speed_stop           = inf->location_info[inf->location_num - 1].speed;
        ptr->m_duration             = inf->time / 1000;
        ptr->m_distance             = inf->odo;
        ptr->m_fuel_used            = inf->fuel;

        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            info.m_timestamp    = (inf->location_info[index].utc / 1000) - (m_timeSyncGap_ms/1000);

            ptr->m_locations.push_back(info);
            // if ((index == inf->location_num - 1) && (info.m_timestamp.Encoded() == 0x00))
            // {
            //     SMLK_LOGD("+++++++++++++++++++++++++++ 0x%08x", info.m_timestamp.Encoded());
            //     auto &signal    = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
            //     signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_SPEEDING);
            //     SMLK_LOGD("+++++++++++++++++++++++++++m_alarm_flags 0x%08x", signal.Encoded());
            // }

            // if ((index == inf->location_num - 1) && (info.m_timestamp.Encoded() != 0x00))
            // {
            //     SMLK_LOGD("=========================== %ld", info.m_timestamp.Encoded());
            //     auto &signal    = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
            //     signal.AssignEncoded(signal.Encoded() & ~AlarmMask::SL_SPEEDING);
            //     SMLK_LOGD("===========================m_alarm_flags 0x%08x", signal.Encoded());
            // }
        }
        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
}

/******************************************************************************
 * NAME: OnDriveEventAccident
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventAccident(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnDriveEventAccident");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    const MajorAccident *acc = reinterpret_cast<const MajorAccident *>(&inf->location_info[inf->location_num]);
    expected += sizeof(MajorAccident) + ((std::size_t)acc->freeze_frame_num) * sizeof(acc->freeze_frame[0]);

    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match as expected: %u", data.size(), expected);
        return;
    }

    if ( inf->location_num < 1 ) {
        SMLK_LOGE("no any position information provided!!!");
        return;
    }

    std::vector<SMLK_UINT8> encoded;

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_ACCIDENT;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        ptr->m_speed_start          = inf->location_info[0].speed;
        ptr->m_speed_stop           = inf->location_info[inf->location_num - 1].speed;
        ptr->m_duration             = inf->time / 1000;
        ptr->m_distance             = inf->odo;
        ptr->m_fuel_used            = inf->fuel;

        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            // accident not report end timestamp
            info.m_timestamp    = inf->location_info[index].utc / 1000;

            ptr->m_locations.push_back(info);
        }

        std::queue<AccidentFrame> empty;
        ptr->m_frames.swap(empty);

        for ( decltype(acc->freeze_frame_num) index = 0; index < acc->freeze_frame_num; index++ ) {
            AccidentFrame   frame(
                acc->freeze_frame[index].braking_sys_state,
                acc->freeze_frame[index].relevant_obj_id,
                acc->freeze_frame[index].obj_relative_speed,
                acc->freeze_frame[index].obj_speed,
                acc->freeze_frame[index].obj_distance
            );
            ptr->m_frames.push(frame);
        }
        ptr->m_type = (SMLK_UINT32)acc->type;
        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
}

/******************************************************************************
 * NAME: OnDriveEventTireWarn
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventTireWarn(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnDriveEventTireWarn");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    expected += sizeof(TPMSwarning);
    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match as expected: %u", data.size(), expected);
        return;
    }

    if ( inf->location_num < 1 ) {
        SMLK_LOGE("no any position information provided!!!");
        return;
    }

    const TPMSwarning   *tpms = reinterpret_cast<const TPMSwarning *>(&inf->location_info[inf->location_num]);

    static const std::unordered_map<TirePressureStatus, SMLK_UINT8> valid_pressure_status = {
        {TirePressureStatus::E_TIRE_PRESSURE_NORMAL,    MessageEvent::TIRE_PRESSURE_NORMAL  },
        {TirePressureStatus::E_TIRE_PRESSURE_HIGH,      MessageEvent::TIRE_PRESSURE_HIGH    },
        {TirePressureStatus::E_TIRE_PRESSURE_LOW,       MessageEvent::TIRE_PRESSURE_LOW     },
        {TirePressureStatus::E_TIRE_PRESSURE_INVALID,   MessageEvent::TIRE_PRESSURE_INVALID },
    };

    static const std::unordered_map<TireAirLeakWarning, SMLK_UINT8> valid_air_leakage_warning_status = {
        { TireAirLeakWarning::E_TIRE_AIR_NORMAL,        MessageEvent::TIRE_AIR_LEAKAGE_UNDETACT },
        { TireAirLeakWarning::E_TIRE_AIR_LEAK_WARNING,  MessageEvent::TIRE_AIR_LEAKAGE_DETACT   },
        { TireAirLeakWarning::E_TIRE_AIR_INVALID,       MessageEvent::TIRE_AIR_LEAKAGE_INVALID  },
    };

    auto it_pressure    = valid_pressure_status.find(tpms->tp_status);
    auto it_air_leakage = valid_air_leakage_warning_status.find(tpms->leak_warning);

    if ( valid_pressure_status.end() == it_pressure ) {
        SMLK_LOGE("unsupported tire pressure status:  %02X", static_cast<SMLK_UINT8>(tpms->tp_status));
        return;
    }

    if ( valid_air_leakage_warning_status.end() == it_air_leakage ) {
        SMLK_LOGE("unsupported tire air leakage status:  %02X", static_cast<SMLK_UINT8>(tpms->leak_warning));
        return;
    }

    std::vector<SMLK_UINT8> encoded;
    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                           = M_SL_EVID_TPMS_TIRE_WARN;
        ptr->m_reversion                    = SL_DRIVE_EVENT_REV;
        ptr->m_speed_start                  = inf->location_info[0].speed;
        ptr->m_speed_stop                   = inf->location_info[inf->location_num - 1].speed;
        ptr->m_status                       = (SMLK_UINT32)it_pressure->second;
        ptr->m_status                      |= (SMLK_UINT32)it_air_leakage->second << 8;
        ptr->m_tire_info.m_id               = tpms->position;
        ptr->m_tire_info.m_temperature      = tpms->tire_temp;
        ptr->m_tire_info.m_pressure         = tpms->tire_pressure;
        ptr->m_tire_info.m_extend_pressure  = tpms->extended_tp;

        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            info.m_timestamp    = inf->location_info[index].utc / 1000;

            ptr->m_locations.push_back(info);
        }

        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

        SMLK_LOGD("OnDriveEventTireWarn send data szie[%d] to tsp", encoded.size());
        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
#if 0
    SMLK_UINT32 status = ((SMLK_UINT32)it_air_leakage->second << 8) | (SMLK_UINT32)it_pressure->second;
    if ( (inf->location_num > 1) && (inf->location_info[inf->location_num-1].utc > inf->location_info[0].utc) ) {
        auto iterator   = m_pending_events.find(M_SL_EVID_TPMS_TIRE_WARN);
        if ( m_pending_events.end() != iterator ) {
            iterator->second.erase(status);
            if ( 0 == iterator->second.size() ) {
                m_pending_events.erase(M_SL_EVID_TPMS_TIRE_WARN);
            }
        }
    } else {
        std::shared_ptr<InternalStruct::DriveEvent> ptr = std::make_shared<InternalStruct::DriveEvent>();
        ptr->m_status                   = status;
        ptr->m_tire.m_id                = tpms->position;
        ptr->m_tire.m_temperature       = tpms->tire_temp;
        ptr->m_tire.m_pressure          = tpms->tire_pressure;
        ptr->m_tire.m_extend_pressure   = tpms->extended_tp;
        ptr->m_location.m_status        = inf->location_info[0].flag ? 0 : 1;
        ptr->m_location.m_latitude      = inf->location_info[0].latitude;
        ptr->m_location.m_longitude     = inf->location_info[0].longitude;
        ptr->m_location.m_altitude      = inf->location_info[0].altitude;
        ptr->m_location.m_heading       = inf->location_info[0].direction;
        ptr->m_location.m_speed         = inf->location_info[0].speed;
        ptr->m_location.m_timestamp     = inf->location_info[0].utc / 1000;

        auto iterator   = m_pending_events.find(M_SL_EVID_TPMS_TIRE_WARN);
        if ( m_pending_events.end() == iterator ) {
            m_pending_events[M_SL_EVID_TPMS_TIRE_WARN] = {};
            iterator   = m_pending_events.find(M_SL_EVID_TPMS_TIRE_WARN);
        }

        iterator->second[status]        = ptr;
    }
#endif
}

/******************************************************************************
 * NAME: OnDriveEventTirednessDriving
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventTirednessDriving(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnDriveEventTirednessDriving");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match with expected: %u", data.size(), expected);
        return;
    }

    if ( inf->location_num < 1 ) {
        SMLK_LOGE("no any position information provided!!!");
        return;
    }

    std::vector<SMLK_UINT8> encoded;

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_TIREDNESS_DRIVING;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        ptr->m_duration             = inf->time / 60000;

        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            info.m_timestamp    = (inf->location_info[index].utc / 1000) - (m_timeSyncGap_ms/1000);

            ptr->m_locations.push_back(info);
            if ((index == inf->location_num - 1) && (info.m_timestamp.Encoded() == 0x00)) {
                auto &signal    = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
                signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_FATIGUE_DRIVING);
            } else if ((index == inf->location_num - 1) && (info.m_timestamp.Encoded() != 0x00)) {
                auto &signal    = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
                signal.AssignEncoded(signal.Encoded() & ~ AlarmMask::SL_FATIGUE_DRIVING);
            }
        }

        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
#if 0
    if ( (inf->location_num > 1) && (inf->location_info[inf->location_num-1].utc > inf->location_info[0].utc) ) {
        m_pending_events.erase(M_SL_EVID_SOC_LOW);
    } else {
        std::shared_ptr<InternalStruct::DriveEvent> ptr = std::make_shared<InternalStruct::DriveEvent>();
        ptr->m_location.m_status    = inf->location_info[0].flag ? 0 : 1;
        ptr->m_location.m_latitude  = inf->location_info[0].latitude;
        ptr->m_location.m_longitude = inf->location_info[0].longitude;
        ptr->m_location.m_altitude  = inf->location_info[0].altitude;
        ptr->m_location.m_heading   = inf->location_info[0].direction;
        ptr->m_location.m_speed     = inf->location_info[0].speed;
        ptr->m_location.m_timestamp = inf->location_info[0].utc / 1000;

        m_pending_events[M_SL_EVID_SOC_LOW] = {{0, ptr}};
    }
#endif
}

/******************************************************************************
 * NAME: OnDriveEventLowSOC
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventLowSOC(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnDriveEventLowSOC");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    expected += sizeof(SOC_LP);
    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match with expected: %u", data.size(), expected);
        return;
    }

    if ( inf->location_num < 1 ) {
        SMLK_LOGE("no any position information provided!!!");
        return;
    }

    const SOC_LP    *soc    = reinterpret_cast<const SOC_LP *>(&inf->location_info[inf->location_num]);

    static const std::unordered_map<SOCPowerOnStatus, SMLK_UINT8>   valid = {
        { SOCPowerOnStatus::E_SOC_STATUS_INVALID,           MessageEvent::SOC_STATUS_INVALID        },
        { SOCPowerOnStatus::E_SOC_STATUS_LOW,               MessageEvent::SOC_STATUS_LOW_WARNING    },
        { SOCPowerOnStatus::E_SOC_STATUS_LOW_WARNING,       MessageEvent::SOC_STATUS_LOW_ALARM      },
        { SOCPowerOnStatus::E_SOC_STATUS_BATTERY_OVERSHOOT, MessageEvent::SOC_STATUS_OVERCHARGE     },
    };

    auto it = valid.find(soc->status);
    if ( valid.end() == it ) {
        SMLK_LOGE("unsupported soc status:  %02X", static_cast<SMLK_UINT8>(soc->status));
        return;
    }

    std::vector<SMLK_UINT8> encoded;
    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_SOC_LOW;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        ptr->m_status               = it->second;
        ptr->m_actual_minimum_soc   = soc->actual_min_soc;

        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            info.m_timestamp    = inf->location_info[index].utc / 1000;

            ptr->m_locations.push_back(info);
        }

        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
#if 0
    if ( (inf->location_num > 1) && (inf->location_info[inf->location_num-1].utc > inf->location_info[0].utc) ) {
        auto iterator   = m_pending_events.find(M_SL_EVID_SOC_LOW);
        if ( m_pending_events.end() != iterator ) {
            iterator->second.erase(it->second);
            if ( 0 == iterator->second.size() ) {
                m_pending_events.erase(M_SL_EVID_SOC_LOW);
            }
        }
    } else {
        std::shared_ptr<InternalStruct::DriveEvent> ptr = std::make_shared<InternalStruct::DriveEvent>();
        ptr->m_status               = it->second;
        ptr->m_actual_minimum_soc   = soc->actual_min_soc;
        ptr->m_location.m_status    = inf->location_info[0].flag ? 0 : 1;
        ptr->m_location.m_latitude  = inf->location_info[0].latitude;
        ptr->m_location.m_longitude = inf->location_info[0].longitude;
        ptr->m_location.m_altitude  = inf->location_info[0].altitude;
        ptr->m_location.m_heading   = inf->location_info[0].direction;
        ptr->m_location.m_speed     = inf->location_info[0].speed;
        ptr->m_location.m_timestamp = inf->location_info[0].utc / 1000;

        auto iterator   = m_pending_events.find(M_SL_EVID_SOC_LOW);
        if ( m_pending_events.end() == iterator ) {
            m_pending_events[M_SL_EVID_SOC_LOW] = {};
            iterator   = m_pending_events.find(M_SL_EVID_SOC_LOW);
        }

        iterator->second[it->second]    = ptr;
    }
#endif
}

/******************************************************************************
 * NAME: OnDriveEventDMSWarn
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventDMSWarn(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnDriveEventDMSWarn");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    expected += sizeof(DMSWarning);
    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match with expected: %u", data.size(), expected);
        return;
    }

    if ( inf->location_num < 1 ) {
        SMLK_LOGE("no any position information provided!!!");
        return;
    }

    const DMSWarning    *status = reinterpret_cast<const DMSWarning *>(&inf->location_info[inf->location_num]);

    static const std::unordered_map<DMSWarning, SMLK_UINT8> valid = {
        { DMSWarning::E_WARNING_EYES_CLOSED,                MessageEvent::DMS_WAR_EYES_CLOSED           },
        { DMSWarning::E_WARNING_DOZE,                       MessageEvent::DMS_WAR_DOZING                },
        { DMSWarning::E_WARNING_SMOKING,                    MessageEvent::DMS_WAR_SMOKING               },
        { DMSWarning::E_WARNING_CALLING,                    MessageEvent::DMS_WAR_CALL                  },
        { DMSWarning::E_WARNING_INFRARED_BLOCK_SUNGLASS,    MessageEvent::DMS_WAR_INFRARED_INTERDICT    },
        { DMSWarning::E_WARNING_CAMERA_OCCLUSION,           MessageEvent::DMS_WAR_MONITOR_INTERDICT     },
        { DMSWarning::E_WARNING_NO_DRIVER_DETECTED,         MessageEvent::DMS_WAR_DRIVER_UNDETECTED     },
        { DMSWarning::E_WARNING_MILD_FATIGUE,               MessageEvent::DMS_WAR_MILD_FATIGUE          },
        { DMSWarning::E_WARNING_SERIOUS_FATIGUE,            MessageEvent::DMS_WAR_SEVERE_FATIGUE        },
        { DMSWarning::E_WARNING_MILD_DISTRACTION,           MessageEvent::DMS_WAR_MILD_INATTENTIVE      },
        { DMSWarning::E_WARNING_SERIOUS_DISTRACTION,        MessageEvent::DMS_WAR_SEVERE_INATTENTIVE    },
    };

    auto it = valid.find(*status);
    if ( valid.end() == it ) {
        SMLK_LOGE("unsupported dms status:  %02X", static_cast<SMLK_UINT8>(*status));
        return;
    }

    std::vector<SMLK_UINT8> encoded;

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_DMS_WARN;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        ptr->m_status               = it->second;

        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            info.m_timestamp    = (inf->location_info[index].utc / 1000) - (m_timeSyncGap_ms/1000);

            ptr->m_locations.push_back(info);
        }

        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
#if 0
    if ( (inf->location_num > 1) && (inf->location_info[inf->location_num-1].utc > inf->location_info[0].utc) ) {
        auto iterator   = m_pending_events.find(M_SL_EVID_DMS_WARN);
        if ( m_pending_events.end() != iterator ) {
            iterator->second.erase(it->second);
            if ( 0 == iterator->second.size() ) {
                m_pending_events.erase(M_SL_EVID_DMS_WARN);
            }
        }
    } else {
        std::shared_ptr<InternalStruct::DriveEvent> ptr = std::make_shared<InternalStruct::DriveEvent>();
        ptr->m_status               = it->second;
        ptr->m_location.m_status    = inf->location_info[0].flag ? 0 : 1;
        ptr->m_location.m_latitude  = inf->location_info[0].latitude;
        ptr->m_location.m_longitude = inf->location_info[0].longitude;
        ptr->m_location.m_altitude  = inf->location_info[0].altitude;
        ptr->m_location.m_heading   = inf->location_info[0].direction;
        ptr->m_location.m_speed     = inf->location_info[0].speed;
        ptr->m_location.m_timestamp = inf->location_info[0].utc / 1000;

        auto iterator   = m_pending_events.find(M_SL_EVID_DMS_WARN);
        if ( m_pending_events.end() == iterator ) {
            m_pending_events[M_SL_EVID_DMS_WARN] = {};
            iterator   = m_pending_events.find(M_SL_EVID_DMS_WARN);
        }

        iterator->second[it->second]    = ptr;
    }
#endif
}

/******************************************************************************
 * NAME: OnDriveEventFuelTankWarn
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventFuelTankWarn(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnDriveEventFuelTankWarn");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match with expected: %u", data.size(), expected);
        return;
    }

    if ( inf->location_num < 1 ) {
        SMLK_LOGE("no any position information provided!!!");
        return;
    }

    std::vector<SMLK_UINT8> encoded;

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_FUEL_TANK_WARN;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;

        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            info.m_timestamp    = (inf->location_info[index].utc / 1000) - (m_timeSyncGap_ms/1000);

            ptr->m_locations.push_back(info);
        }

        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
#if 0
    if ( (inf->location_num > 1) && (inf->location_info[inf->location_num-1].utc > inf->location_info[0].utc) ) {
        m_pending_events.erase(M_SL_EVID_FUEL_TANK_WARN);
    } else {
        std::shared_ptr<InternalStruct::DriveEvent> ptr = std::make_shared<InternalStruct::DriveEvent>();
        ptr->m_location.m_status    = inf->location_info[0].flag ? 0 : 1;
        ptr->m_location.m_latitude  = inf->location_info[0].latitude;
        ptr->m_location.m_longitude = inf->location_info[0].longitude;
        ptr->m_location.m_altitude  = inf->location_info[0].altitude;
        ptr->m_location.m_heading   = inf->location_info[0].direction;
        ptr->m_location.m_speed     = inf->location_info[0].speed;
        ptr->m_location.m_timestamp = inf->location_info[0].utc / 1000;

        m_pending_events[M_SL_EVID_FUEL_TANK_WARN] = {{0,   ptr}};
    }
#endif
}

/******************************************************************************
 * NAME: OnDriveEventAgitatorFault
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventAgitatorFault(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnDriveEventAgitatorFaults");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    expected += sizeof(FixTankError);
    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match with expected: %u", data.size(), expected);
        return;
    }

    if ( inf->location_num < 1 ) {
        SMLK_LOGE("no any position information provided!!!");
        return;
    }

    const FixTankError  *status = reinterpret_cast<const FixTankError *>(&inf->location_info[inf->location_num]);

    static const std::unordered_map<FixTankError, SMLK_UINT8>   valid = {
        { FixTankError::E_ERROR_SIGNAL1_OPEN_CIRCUIT,               MessageEvent::AGITATOR_WAR_SIG1_OPEN                },
        { FixTankError::E_ERROR_SIGNAL1_SCP,                        MessageEvent::AGITATOR_WAR_SIG1_SHORT_POWER         },
        { FixTankError::E_ERROR_SIGNAL1_SCG,                        MessageEvent::AGITATOR_WAR_SIG1_SHORT_GROUND        },
        { FixTankError::E_ERROR_SIGNAL2_OPEN_CIRCUIT,               MessageEvent::AGITATOR_WAR_SIG2_OPEN                },
        { FixTankError::E_ERROR_SIGNAL2_SCP,                        MessageEvent::AGITATOR_WAR_SIG2_SHORT_POWER         },
        { FixTankError::E_ERROR_SIGNAL2_SCG,                        MessageEvent::AGITATOR_WAR_SIG2_SHORT_GROUND        },
        { FixTankError::E_ERROR_SIGNAL1_AND_SIGNAL2_SHORT_CIRCUIT,  MessageEvent::AGITATOR_WAR_SIG1_SHORT_SIG2          },
        { FixTankError::E_ERROR_SIGNAL1_AND_SIGNAL2_TIME_SERIES,    MessageEvent::AGITATOR_WAR_SIG1_SIG2_SEQ_REVERSED   },
        { FixTankError::E_ERROR_SENSOR_INTERVAL_OVERSIZE,           MessageEvent::AGITATOR_WAR_SENSOR_GAP_BIG           },
        { FixTankError::E_ERROR_SENSOR_INTERVAL_UNDERSIZE,          MessageEvent::AGITATOR_WAR_SENSOR_GAP_SMALL         },
        { FixTankError::E_ERROR_SENSOR_NO_SIGNAL,                   MessageEvent::AGITATOR_WAR_SENSOR_INACTIVE          },
    };

    auto it = valid.find(*status);
    if ( valid.end() == it ) {
        SMLK_LOGE("unsupported agitator status:  %02X", static_cast<SMLK_UINT8>(*status));
        return;
    }

    std::vector<SMLK_UINT8> encoded;

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_AGITATOR_FAULT;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        ptr->m_status               = it->second;

        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            info.m_timestamp    = (inf->location_info[index].utc / 1000) - (m_timeSyncGap_ms/1000);

            ptr->m_locations.push_back(info);
        }

        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
#if 0
    if ( (inf->location_num > 1) && (inf->location_info[inf->location_num-1].utc > inf->location_info[0].utc) ) {
        auto iterator   = m_pending_events.find(M_SL_EVID_AGITATOR_FAULT);
        if ( m_pending_events.end() != iterator ) {
            iterator->second.erase(it->second);
            if ( 0 == iterator->second.size() ) {
                m_pending_events.erase(M_SL_EVID_AGITATOR_FAULT);
            }
        }
    } else {
        std::shared_ptr<InternalStruct::DriveEvent> ptr = std::make_shared<InternalStruct::DriveEvent>();
        ptr->m_status               = it->second;
        ptr->m_location.m_status    = inf->location_info[0].flag ? 0 : 1;
        ptr->m_location.m_latitude  = inf->location_info[0].latitude;
        ptr->m_location.m_longitude = inf->location_info[0].longitude;
        ptr->m_location.m_altitude  = inf->location_info[0].altitude;
        ptr->m_location.m_heading   = inf->location_info[0].direction;
        ptr->m_location.m_speed     = inf->location_info[0].speed;
        ptr->m_location.m_timestamp = inf->location_info[0].utc / 1000;

        auto iterator   = m_pending_events.find(M_SL_EVID_AGITATOR_FAULT);
        if ( m_pending_events.end() == iterator ) {
            m_pending_events[M_SL_EVID_AGITATOR_FAULT] = {};
            iterator   = m_pending_events.find(M_SL_EVID_AGITATOR_FAULT);
        }

        iterator->second[it->second]    = ptr;
    }
#endif
}

/******************************************************************************
 * NAME: OnDriveEventDisMantleTbox
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventDisMantleTbox(IN MsgHead &head)
{
    SMLK_LOGE("on OnDriveEventDisMantleTbox");
    std::vector<SMLK_UINT8> encoded;
    smartlink_sdk::LocationInfo start_loc;
    std::memset(&start_loc, 0x0, sizeof(smartlink_sdk::LocationInfo));

    auto rc = smartlink_sdk::Location::GetInstance()->GetLocation(start_loc);
    if (smartlink_sdk::RtnCode::E_SUCCESS != rc) {
        SMLK_LOGE("[GNSS]fail to get location information, rc: %d", static_cast<std::int32_t>(rc));
    }

    //针对不同类型做不同检查,电源改变，检查4G和GPS， 4G改变，检查GPS, gps状态改变，检查4G。
    CheckWireConnectStatus(head.m_protocol);

    static SMLK_UINT8 last_dismantle_status = 0;
    if (last_dismantle_status != m_dismantle_tbox_status) {
        last_dismantle_status = m_dismantle_tbox_status;
        if ( head.m_extra == DismantleReport::EVENT_NOT_REPORT) {
            // 待需求规范更新 防拆恢复也需要上报
            SMLK_LOGD("dismantle recover should not report 0f3d");
            return;
        }
    } else {
        SMLK_LOGD("repeat dismantle status, not report 0f3d!!!");
        SMLK_LOGD("last_dismantle_status is[%d] current m_dismantle_tbox_status is [%d]", last_dismantle_status, m_dismantle_tbox_status);
        return;
    }

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_DISMANTLE_TBOX;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        ptr->m_status               = m_dismantle_tbox_status;
        ptr->m_locations.clear();
        LocationInfo    info_start;
        LocationInfo    info_end;

        info_start.m_status       = m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second;

        info_start.m_heading      = start_loc.heading; // debug
        info_start.m_latitude     = start_loc.latitude;
        info_start.m_longitude    = start_loc.longitude;
        info_start.m_altitude     = start_loc.altitude;
        info_start.m_speed        = start_loc.speed;
        // info_start.m_timestamp    = start_loc.utc / 1000;
        if ( !CheckWhetherTimeSync() ) {
            // 未授时 先使用系统时间
            info_start.m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        } else {
            // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
            info_start.m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_timeSyncGap_ms/1000);
        }

        ptr->m_locations.push_back(info_start);
        // 单点事件只有起始点，结束点填充0
        info_end.m_status       = m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second;
        info_end.m_heading      = 0;
        info_end.m_latitude     = 0;
        info_end.m_longitude    = 0;
        info_end.m_altitude     = 0;
        info_end.m_speed        = 0;
        info_end.m_timestamp    = 0;
        ptr->m_locations.push_back(info_end);

        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;
        SMLK_LOGD("send encode data to tsp size =%d", (SMLK_UINT8)encoded.size());

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
}

/******************************************************************************
 * NAME: OnDriveEventGbVinError
 *
 * DESCRIPTION: drive event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *
 * RETURN:
 *****************************************************************************/
void CollectionService::OnDriveEventGbCheckVinError(IN MsgHead &/*head*/)
{
    SMLK_LOGD("[VinDebug] on OnDriveEventGbVinError");
    std::vector<SMLK_UINT8> encoded;
    smartlink_sdk::LocationInfo start_loc;
    std::memset(&start_loc, 0x0, sizeof(smartlink_sdk::LocationInfo));

    auto rc = smartlink_sdk::Location::GetInstance()->GetLocation(start_loc);
    if (smartlink_sdk::RtnCode::E_SUCCESS != rc) {
        SMLK_LOGE("[VinDebug] fail to get location information, rc: %d", static_cast<std::int32_t>(rc));
    }

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);

        ptr->m_id                   = M_SL_EVID_GB_CHECK_VIN_ERROR;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        ptr->m_locations.clear();
        LocationInfo    info_start;
        LocationInfo    info_end;
        info_start.m_status       = start_loc.fix_valid ? 0 : 1;
        info_start.m_heading      = start_loc.heading;
        info_start.m_latitude     = start_loc.latitude;
        info_start.m_longitude    = start_loc.longitude;
        info_start.m_altitude     = start_loc.altitude;
        info_start.m_speed        = start_loc.speed;

        // use local time to avoid gps invalid
        if ( !CheckWhetherTimeSync() ) {
            // 未授时,先使用系统时间
            info_start.m_timestamp = (SMLK_UINT64)std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        } else {
            SMLK_LOGD("[timesyncDebug] m_timeSyncGap_ms is %lld", m_timeSyncGap_ms);
            SMLK_LOGD("[timesyncDebug] localtime is %lld", (SMLK_UINT64)std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
            // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
            info_start.m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_timeSyncGap_ms/1000);
        }
        ptr->m_locations.push_back(info_start);
        // 单点事件只有起始点，结束点填充 0
        info_end.m_status       = start_loc.fix_valid ? 0 : 1;
        info_end.m_heading      = 0;
        info_end.m_latitude     = 0;
        info_end.m_longitude    = 0;
        info_end.m_altitude     = 0;
        info_end.m_speed        = 0;
        info_end.m_timestamp    = 0;
        ptr->m_locations.push_back(info_end);
        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;
        SMLK_LOGD("[VinDebug] send encode data to tsp size =%d", (SMLK_UINT8)encoded.size());

        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
}

/******************************************************************************
* NAME: OnDriveEventRefrigeratorFailure
*
* DESCRIPTION: regrigerator failure event handle
*
* PARAMETERS:
*       head:  message head which has the basic information about this message
        data:  message data
* RETURN:
*
* NOTE:
*****************************************************************************/
void CollectionService::OnDriveEventRefrigeratorFailure(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnDriveEventRefrigeratorFailure in...");
    if ( data.size() < sizeof(DriveEventInfo) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(DriveEventInfo));
        return;
    }

    const DriveEventInfo    *inf = reinterpret_cast<const DriveEventInfo *>(data.data());
    if ( inf->location_num < 2 ) {
        SMLK_LOGE("invalid data, we need at least 2 location information, one for event start and the other for event stop!!!");
        return;
    }

    auto expected   = sizeof(DriveEventInfo) + ((std::size_t)inf->location_num) * sizeof(inf->location_info[0]);
    expected += sizeof(DMSWarning);
    if ( data.size() != expected ) {
        SMLK_LOGE("body size %u is not match with expected: %u", data.size(), expected);
        return;
    }

    if ( inf->location_num < 1 ) {
        SMLK_LOGE("no any position information provided!!!");
        return;
    }

    const RefrigeratorFailure status = *reinterpret_cast<const RefrigeratorFailure *>(&inf->location_info[inf->location_num]);
    std::vector<SMLK_UINT8> encoded;

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageEvent> ptr = std::static_pointer_cast<MessageEvent>(m_protocol_messages[protocol][MessageType::SL_EVENT]);
        ptr->m_id                   = M_SL_EVID_REFRIGERATOR_FAILURE;
        ptr->m_reversion            = SL_DRIVE_EVENT_REV;
        // ptr->m_status               = status;
        ptr->m_status               = (SMLK_UINT32)status;
        ptr->m_locations.clear();

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            LocationInfo    info;

            info.m_status       = inf->location_info[index].flag ? 0 : 1;
            info.m_heading      = inf->location_info[index].direction;
            info.m_latitude     = inf->location_info[index].latitude;
            info.m_longitude    = inf->location_info[index].longitude;
            info.m_altitude     = inf->location_info[index].altitude;
            info.m_speed        = inf->location_info[index].speed;
            info.m_timestamp    = inf->location_info[index].utc / 1000;

            ptr->m_locations.push_back(info);
        }
        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;
        std::memset(&head, 0x00, sizeof(head));
        head.msg_id     = M_SL_MSGID_TP_BEHAVIOR_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;
        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
    }
}

/******************************************************************************
 * NAME: OnFaultEvent
 *
 * DESCRIPTION: fault event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *       data:  message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnFaultEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnFaultEvent");
    switch( ((head.m_extra >> 24) & 0xFF) ) {
        case FaultEventMask::SL_EVENT_DM1:
            OnFaultEventDM1(head, data);
            break;
        default:
            SMLK_LOGE("unsupported fault event type: 0x%02X", (head.m_extra >> 24) & 0xFF);
            break;
    }
}

/******************************************************************************
 * NAME: OnFaultEventDM1
 *
 * DESCRIPTION: DM1 fault event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *       data:  message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::OnFaultEventDM1(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("DM1debug OnFaultEventDM1");
    if ( data.size() < sizeof(FaultCodesT) ) {
        SMLK_LOGE("dm1debug invalid data size");
        return;
    }

    SMLK_UINT32 DM1_status_flag = head.m_extra & 0x00F00000;

    const ExtraFault *  extra   = (const ExtraFault *)&head.m_extra;
    const FaultCodesT * faults  = (const FaultCodesT *)data.data();

    decltype(data.size())   expected = ((decltype(data.size()))faults->count) * sizeof(FaultCodeT) + sizeof(FaultCodesT);
    if ( expected != data.size() ) {
        SMLK_LOGE("[dm1debug] mismatched data size");
        return;
    }

    auto target = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto &signal_ap     = m_cached_signals.find(vehicle::SignalID::AccPedalTravel)->second;
    auto &signal_ap_cr  = m_cached_signals.find(vehicle::SignalID::AcceleratorChangeRate)->second;

    signal_ap_cr.Invalid();
    if ( m_cached_ap_positions.size() > 0 ) {
        // calculate the accelerator change rate
        SMLK_UINT16 ap_ps = m_cached_ap_positions.front().second;

        // get a copy of cached AP positions
        auto ap_positions = m_cached_ap_positions;

        ap_positions.pop();
        while ( ap_positions.size() ) {
            if ( ap_positions.front().first > target ) {
                break;      // this is frame in the latest 1 seconds
            }

            ap_ps = ap_positions.front().second;
            ap_positions.pop();
        }

        signal_ap_cr = std::abs(signal_ap.Val() - (double)ap_ps * 0.1);
    }

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageFault>   p_fault = std::static_pointer_cast<MessageFault>(m_protocol_messages[protocol][MessageType::SL_FAULT]);
        std::shared_ptr<MessageDM1>     p_dm1   = std::static_pointer_cast<MessageDM1>(m_protocol_messages[protocol][MessageType::SL_FAULT_DM1]);

        p_fault->m_location.m_status        = m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second;
        p_fault->m_location.m_heading       = m_cached_signals.find(vehicle::SignalID::GNSSHeading)->second;
        p_fault->m_location.m_latitude      = m_cached_signals.find(vehicle::SignalID::GNSSLatitude)->second;
        p_fault->m_location.m_longitude     = m_cached_signals.find(vehicle::SignalID::GNSSLongitude)->second;
        p_fault->m_location.m_altitude      = m_cached_signals.find(vehicle::SignalID::GNSSAltitude)->second;
        p_fault->m_location.m_speed         = m_cached_signals.find(vehicle::SignalID::GNSSSpeed)->second.Val() * M_PER_S_TO_KM_PER_HOUR;
        // p_fault->m_location.m_timestamp     = m_cached_signals.find(vehicle::SignalID::GNSSTimeStamp)->second;

        // use local time to avoid gps invalid
        if ( !CheckWhetherTimeSync() ) {
            // 未授时,先使用系统时间
            p_fault->m_location.m_timestamp = (SMLK_UINT64)std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        } else {
            // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
            p_fault->m_location.m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_timeSyncGap_ms/1000);
        }

        p_dm1->m_faults.clear();
        // Flag
        p_dm1->m_type   = ((FaultEventFalg::SL_EVENT_START == DM1_status_flag) ? MessageDM1::FAULT_OCCURS : MessageDM1::FAULT_CLEAR);
        p_dm1->m_add_extra_info_flag        = head.m_protocol;
        SMLK_LOGD("[dm1debug] dm1 upload type is %d", p_dm1->m_type);

        p_dm1->m_tachograph_speed           = m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second;
        p_dm1->m_acc_pedal_travel           = signal_ap;
        p_dm1->m_brake_pedal_state          = m_cached_signals.find(vehicle::SignalID::BrakeSwitch)->second;
        p_dm1->m_engine_speed               = m_cached_signals.find(vehicle::SignalID::EngineSpeed)->second;
        //p_dm1->m_tubocharing_pressue        = m_cached_signals.find(vehicle::SignalID::BrakeSwitch)->second;
        p_dm1->m_intake_manifold_pressure   = m_cached_signals.find(vehicle::SignalID::EngineIntakeManifoldPressure)->second;
        p_dm1->m_exhaust_gas_temperature    = m_cached_signals.find(vehicle::SignalID::EngineExhaustGasTemperature)->second;
        p_dm1->m_coolant_temperature        = m_cached_signals.find(vehicle::SignalID::EngineCoolantTemperature)->second;
        p_dm1->m_acc_change_rate            = signal_ap_cr;
        p_dm1->m_gear                       = m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second;
        p_dm1->m_torque_percent             = m_cached_signals.find(vehicle::SignalID::ActualEnginePercentTorque)->second;
        p_dm1->m_vehicle_load               = m_cached_signals.find(vehicle::SignalID::TrailerWeight)->second;
        p_dm1->m_engine_load                = m_cached_signals.find(vehicle::SignalID::EngineLoadOnCurrentSpeed)->second;
        // add dm1 acc and acc flag
        auto& acc_signal = m_cached_signals.find(vehicle::SignalID::TBoxAcceleration)->second;
        p_dm1->m_acceleration               = acc_signal.Val() < 0 ? 0.0 : acc_signal;
        p_dm1->m_deceleration               = acc_signal.Val() < 0 ? (-acc_signal.Val()) : 0.0;

        for ( decltype(faults->count) index = 0; index < faults->count; index++ ) {
            InternalDTC internal;
            internal.dtc.SPN    = faults->codes[index].spn;
            internal.dtc.FMI    = faults->codes[index].fmi;
            internal.dtc.SRC    = faults->codes[index].src;
            SMLK_LOGD("onfaultevent internal.dtc.SPN:    %u", internal.dtc.SPN);
            SMLK_LOGD("onfaultevent internal.dtc.FMI:    %u", internal.dtc.FMI);
            SMLK_LOGD("onfaultevent internal.dtc.SRC:    %u", internal.dtc.SRC);
            SMLK_LOGD("onfaultevent internal.id:     0x%08X", internal.id);

            if ( p_dm1->m_reported_faults.end() != p_dm1->m_reported_faults.find(internal.id) ) {
                SMLK_LOGD("m_reported_faults not find id");
                // continue;  // not break this time for() because compare uint32 TODO:change compare src
            }
            p_dm1->m_faults.emplace_back(faults->codes[index].src, faults->codes[index].fmi, faults->codes[index].spn);

            p_dm1->m_reported_faults.insert(internal.id);
            SMLK_LOGD("p_dm1->m_reported_faults size is %d", p_dm1->m_reported_faults.size());

        }

        if ( (MessageDM1::FAULT_OCCURS == p_dm1->m_type) && (0 == p_dm1->m_faults.size()) ) {
            SMLK_LOGD("fault occur but p_dm1->m_faults is 0!!!! no need to be reported");

            // fault occur but no fault need to be reported, may be it has been reported sometime.
            continue;
        }

        p_fault->ResetMessage(MessageFault::MSG_DM1, p_dm1);

        std::vector<SMLK_UINT8> encoded;
        encoded.reserve(512);

        if ( SL_SUCCESS != p_fault->Encode(encoded) ) {
            SMLK_LOGE("fail to encode fault event message");
            continue;
        }

        IpcTspHead  resp_head;

        std::memset(&resp_head, 0x00, sizeof(resp_head));

        resp_head.msg_id    = M_SL_MSGID_TP_FAULT_REQ;
        resp_head.protocol  = protocol;
        resp_head.qos       = QOS_SEND_ALWAYS;
        SMLK_LOGD("[DM1debug] send DM1 msg to tsp and ebcode size is %d", encoded.size());

        TspServiceApi::getInstance()->SendMsg(resp_head, encoded.data(), encoded.size());
    }
}

/******************************************************************************
 * NAME: SamplingLocation
 *
 * DESCRIPTION: sampling location information
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::SamplingLocation()
{
    SMLK_LOGI("SamplingLocation");
    if ( 0 == (m_internal_flags & InternalFlags::SL_FLAG_TEL_READY) ) {
        if (smartlink_sdk::Telephony::GetInstance()->IsReady()) {
            smartlink_sdk::TelRadioSignalInfo rssi;
            smartlink_sdk::Telephony::GetInstance()->GetSignalStrength(rssi);
            SMLK_LOGI("get telephony  rssi %d ", rssi.signal);
            auto &signal = m_cached_signals.find(vehicle::SignalID::RSSI)->second;
            signal =  rssi.signal;
        } else {
            SMLK_LOGD("TEL service not ready");
        }
    }

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageLocation> ptr = std::static_pointer_cast<MessageLocation>(m_protocol_messages[protocol][MessageType::SL_LOCATION]);

        if ( !CheckWhetherTimeSync() ) {
            // 从未授时时,先缓存数据,然后在授时后修改缓存的GNSSTimeStamp字段,从缓存中取出所有数据发送
            ptr->m_location.m_timestamp = m_cached_signals.find(vehicle::SignalID::GNSSTimeStamp)->second;
        } else {
            SMLK_LOGI("[timesyncDebug] m_timeSyncGap_ms is %lld", m_timeSyncGap_ms);
            SMLK_LOGI("[timesyncDebug] localtime is %lld",(SMLK_UINT64)std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
           // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
            ptr->m_location.m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_timeSyncGap_ms/1000);
        }

        ptr->m_alarm_flags                              = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
        ptr->m_tachograph_vehicle_speed                 = m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second;
        ptr->m_engine_torque_mode                       = m_cached_signals.find(vehicle::SignalID::EngineTorqueMode)->second;
        ptr->m_torque_percent_cmd                       = m_cached_signals.find(vehicle::SignalID::DriverDemandEnginePercentTorque)->second;
        ptr->m_torque_percent                           = m_cached_signals.find(vehicle::SignalID::ActualEnginePercentTorque)->second;
        ptr->m_engine_speed                             = m_cached_signals.find(vehicle::SignalID::EngineSpeed)->second;
        ptr->m_agtator_total_rotations                  = m_cached_signals.find(vehicle::SignalID::AgtatorTotalRotations)->second;
        ptr->m_coolant_temperature                      = m_cached_signals.find(vehicle::SignalID::EngineCoolantTemperature)->second;
        ptr->m_oil_pressure                             = m_cached_signals.find(vehicle::SignalID::EngineOilPressure)->second;
        ptr->m_engine_total_fuel_used                   = m_cached_signals.find(vehicle::SignalID::EngineTotalFuelUsed)->second;
        ptr->m_acc_pedal_travel                         = m_cached_signals.find(vehicle::SignalID::AccPedalTravel)->second;
        ptr->m_parking                                  = m_cached_signals.find(vehicle::SignalID::ParkingBrakeSwitch)->second;
        ptr->m_wheel_speed                              = m_cached_signals.find(vehicle::SignalID::WheelSpeed)->second;
        ptr->m_clutch                                   = m_cached_signals.find(vehicle::SignalID::ClutchSwitch)->second;
        ptr->m_brake_pedal_state                        = m_cached_signals.find(vehicle::SignalID::BrakeSwitch)->second;
        ptr->m_urea_tank_level                          = m_cached_signals.find(vehicle::SignalID::CatalystTankLevel)->second;
        ptr->m_engine_total_time                        = m_cached_signals.find(vehicle::SignalID::EngineTotalHoursOfOperation)->second;

        // workaround for TAPD：1018895+1021096 total_cycles should always repor valid data
        if ( (!m_cached_signals.find(vehicle::SignalID::EngineTotalRevolutions)->second.Valid())
            && SMLK_DOUBLE(0xFFFFFFFF) == m_cached_signals.find(vehicle::SignalID::EngineTotalRevolutions)->second.Val()) {
            ptr->m_engine_total_cycles = 4294967295000; // 0xFFFFFFFF*1000
            ptr->m_engine_total_cycles.Invalid();
        } else {
            ptr->m_engine_total_cycles                      = m_cached_signals.find(vehicle::SignalID::EngineTotalRevolutions)->second;
        }
        ptr->m_engine_fuel_consumption_rate             = m_cached_signals.find(vehicle::SignalID::EngineFuelRate)->second;
        ptr->m_engine_instantaneous_fuel_consumption    = m_cached_signals.find(vehicle::SignalID::EngineInstantaneousFuelEconomy)->second;
        ptr->m_engine_average_fuel_consumption          = m_cached_signals.find(vehicle::SignalID::EngineAverageFuelEconomy)->second;
        ptr->m_dpf_pressure_differential                = m_cached_signals.find(vehicle::SignalID::AftertreatmentDieselParticulateFilterDifferentialPressure)->second;
        ptr->m_intake_manifold_pressure                 = m_cached_signals.find(vehicle::SignalID::EngineIntakeManifoldPressure)->second;
        ptr->m_intake_manifole_temprerature             = m_cached_signals.find(vehicle::SignalID::EngineIntakeManifoldTemperature)->second;
        ptr->m_engine_intake_pressure                   = m_cached_signals.find(vehicle::SignalID::EngineAirIntakePressure)->second;
        ptr->m_barometric_pressure                      = m_cached_signals.find(vehicle::SignalID::BarometricPressure)->second;

        {
            // here is a gap between CAN matrix and TSP protocol about the Cab Temperature, so here we use value assign instead of signal assign
            auto const &signal = m_cached_signals.find(vehicle::SignalID::CabTemperature)->second;
            ptr->m_cab_interior_temperature             = signal.Val();
            if ( !signal.Valid() ) {
                ptr->m_cab_interior_temperature.Invalid();
            }
        }

        // ptr->m_cab_interior_temperature                 = m_cached_signals.find(vehicle::SignalID::CabTemperature)->second.Val();

        ptr->m_ambient_air_temperature                  = m_cached_signals.find(vehicle::SignalID::AmbientAirTemperature)->second;
        ptr->m_engine_intake_temperature                = m_cached_signals.find(vehicle::SignalID::EngineAirIntakeTemperature)->second;
        // 2021-0826 update by latest requirement
         ptr->m_cold_start_light                         = m_cached_signals.find(vehicle::SignalID::EngineWaitToStartLamp)->second;

        ptr->m_water_in_fuel_flag                       = m_cached_signals.find(vehicle::SignalID::WaterInFuelIndicator)->second;
        ptr->m_target_gear                              = m_cached_signals.find(vehicle::SignalID::TransmissionSelectedGear)->second;
        ptr->m_gear_ratio                               = m_cached_signals.find(vehicle::SignalID::TransmissionActualGearRatio)->second;
        ptr->m_gear                                     = m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second;
        ptr->m_instrument_fuel_level                    = m_cached_signals.find(vehicle::SignalID::FuelLevel)->second;
        ptr->m_instrument_subtotal_mileage              = m_cached_signals.find(vehicle::SignalID::TripDistance)->second;
        ptr->m_instrument_total_mileage                 = m_cached_signals.find(vehicle::SignalID::TotalVehicleDistance)->second;
        ptr->m_instrument_total_mileage_head_ext1       = m_cached_signals.find(vehicle::SignalID::TotalVehicleDistance)->second;
        ptr->m_calculus_distance                        = m_cached_signals.find(vehicle::SignalID::CalculusDistance)->second;
        ptr->m_calculus_fuel_used                       = m_cached_signals.find(vehicle::SignalID::CalculusFuelConsumption)->second;
        ptr->m_actural_catalyst_spewed                  = m_cached_signals.find(vehicle::SignalID::ActuralCatalystSpewed)->second;
        ptr->m_total_catalyst_spewed                    = m_cached_signals.find(vehicle::SignalID::TotalCatalystSpewed)->second;
        ptr->m_acc_status                               = m_cached_signals.find(vehicle::SignalID::ACCStatus)->second;
        ptr->m_rssi                                     = m_cached_signals.find(vehicle::SignalID::RSSI)->second;
        ptr->m_satellites_used                          = m_cached_signals.find(vehicle::SignalID::GNSSSatelliteUsed)->second;
        ptr->m_battery_1_temperature                    = m_cached_signals.find(vehicle::SignalID::Battery1Temperature)->second;
        ptr->m_battery_1_vol                            = m_cached_signals.find(vehicle::SignalID::Battery1Voltage)->second;
        ptr->m_battery_1_soc                            = m_cached_signals.find(vehicle::SignalID::Battery1SOC)->second;
        ptr->m_battery_1_soh                            = m_cached_signals.find(vehicle::SignalID::Battery1SOH)->second;
        ptr->m_battery_2_temperature                    = m_cached_signals.find(vehicle::SignalID::Battery2Temperature)->second;
        ptr->m_battery_2_vol                            = m_cached_signals.find(vehicle::SignalID::Battery2Voltage)->second;
        ptr->m_battery_2_soc                            = m_cached_signals.find(vehicle::SignalID::Battery2SOC)->second;
        ptr->m_battery_2_soh                            = m_cached_signals.find(vehicle::SignalID::Battery2SOH)->second;
        ptr->m_actual_minimum_soc                       = m_cached_signals.find(vehicle::SignalID::ActualMinimumSOC)->second;
        ptr->m_actual_minimum_soh                       = m_cached_signals.find(vehicle::SignalID::ActualMinimumSOH)->second;
        ptr->m_agtator_state                            = m_cached_signals.find(vehicle::SignalID::AgtatorState)->second;
        ptr->m_agtator_speed                            = m_cached_signals.find(vehicle::SignalID::AgtatorSpeed)->second;

        ptr->m_catalyst_tank_temperature                = m_cached_signals.find(vehicle::SignalID::CatalystTankTemperature)->second;
        ptr->m_engine_power_input1                      = m_cached_signals.find(vehicle::SignalID::EnginePowerInput1)->second;
        // 发动机排气温度
        ptr->m_exhaust_gas_temperature                  = m_cached_signals.find(vehicle::SignalID::EngineExhaustGasTemperature)->second;

        ptr->m_keyswitch_battery_voltage                = m_cached_signals.find(vehicle::SignalID::keyswitchBatteryVoltage)->second;
        ptr->m_sensor1_humidity                         = m_cached_signals.find(vehicle::SignalID::Sensor1Humidity)->second;
        ptr->m_sensor2_humidity                         = m_cached_signals.find(vehicle::SignalID::Sensor2Humidity)->second;
        ptr->m_sensor3_humidity                         = m_cached_signals.find(vehicle::SignalID::Sensor3Humidity)->second;
        ptr->m_sensor4_humidity                         = m_cached_signals.find(vehicle::SignalID::Sensor4Humidity)->second;
        ptr->m_sensor5_humidity                         = m_cached_signals.find(vehicle::SignalID::Sensor5Humidity)->second;
        ptr->m_sensor6_humidity                         = m_cached_signals.find(vehicle::SignalID::Sensor6Humidity)->second;
        ptr->m_sensor7_humidity                         = m_cached_signals.find(vehicle::SignalID::Sensor7Humidity)->second;
        ptr->m_sensor8_humidity                         = m_cached_signals.find(vehicle::SignalID::Sensor8Humidity)->second;
        ptr->m_sensor1_temperature                      = m_cached_signals.find(vehicle::SignalID::Sensor1Temperature)->second;
        ptr->m_sensor2_temperature                      = m_cached_signals.find(vehicle::SignalID::Sensor2Temperature)->second;
        ptr->m_sensor3_temperature                      = m_cached_signals.find(vehicle::SignalID::Sensor3Temperature)->second;
        ptr->m_sensor4_temperature                      = m_cached_signals.find(vehicle::SignalID::Sensor4Temperature)->second;
        ptr->m_sensor5_temperature                      = m_cached_signals.find(vehicle::SignalID::Sensor5Temperature)->second;
        ptr->m_sensor6_temperature                      = m_cached_signals.find(vehicle::SignalID::Sensor6Temperature)->second;
        ptr->m_sensor7_temperature                      = m_cached_signals.find(vehicle::SignalID::Sensor7Temperature)->second;
        ptr->m_sensor8_temperature                      = m_cached_signals.find(vehicle::SignalID::Sensor8Temperature)->second;
        ptr->m_pefrigeration_switch_status              = m_cached_signals.find(vehicle::SignalID::PefrigerationSwitchStatus)->second;
        ptr->m_pefrigeration_work_mode                  = m_cached_signals.find(vehicle::SignalID::PefrigerationWorkMode)->second;
        ptr->m_pefrigeration_cpmpartment_temperature    = m_cached_signals.find(vehicle::SignalID::PefrigerationCpmpartmentTemperature)->second;
        ptr->m_defrost_temperature                      = m_cached_signals.find(vehicle::SignalID::DefrostTemperature)->second;
        ptr->m_supply_voltage                           = m_cached_signals.find(vehicle::SignalID::SupplyVoltage)->second;
        // update 2022-02-15 0200-e7
        ptr->m_dc_ac_power_consumption                  = m_cached_signals.find(vehicle::SignalID::DcAcPowerConsumption)->second;
        ptr->m_steerwheel_heat_power_consumption        = m_cached_signals.find(vehicle::SignalID::SteerWheelHeatPowerConsumption)->second;
        ptr->m_cigar_light_power_consumption            = m_cached_signals.find(vehicle::SignalID::CigarLighterPowerConsumption)->second;
        ptr->m_24v_socket1_power_consumption            = m_cached_signals.find(vehicle::SignalID::Socket1PowerConsumption)->second;
        ptr->m_24v_socket2_power_consumption            = m_cached_signals.find(vehicle::SignalID::Socket2PowerConsumption)->second;
        ptr->m_left_seat_power_consumption              = m_cached_signals.find(vehicle::SignalID::LeftSeatHeatPowerConsumption)->second;
        ptr->m_right_seat_power_consumption             = m_cached_signals.find(vehicle::SignalID::RightSeatHeatPowerConsumption)->second;
        ptr->m_lower_bretht_heat_power_consumption      = m_cached_signals.find(vehicle::SignalID::LowerBerthtHeatPowerConsumption)->second;
        ptr->m_sleep_read_lamp_power_consumption        = m_cached_signals.find(vehicle::SignalID::SleepReadLampPowerConsumption)->second;
        ptr->m_elevated_box_power_consumption           = m_cached_signals.find(vehicle::SignalID::ElevatedBoxPowerConsumption)->second;
        ptr->m_humidity_in_box1                         = m_cached_signals.find(vehicle::SignalID::HumidityInBox1)->second;
        ptr->m_humidity_in_box2                         = m_cached_signals.find(vehicle::SignalID::HumidityInBox2)->second;
        ptr->m_temperature_in_box1                      = m_cached_signals.find(vehicle::SignalID::TemperatureInBox1)->second;
        ptr->m_temperature_in_box2                      = m_cached_signals.find(vehicle::SignalID::TemperatureInBox2)->second;
        // add 0200-EC 2022-03-08
        ptr->m_tbox_voltage                             = m_cached_signals.find(vehicle::SignalID::TboxVlotage)->second;
        ptr->m_fuel_save_switch                         = m_cached_signals.find(vehicle::SignalID::FuelSaveSwitch)->second;
        ptr->m_hydraulic_motor_oil_return_temperature   = m_cached_signals.find(vehicle::SignalID::HydraulicMotorOilReturnTemperature)->second;

        ptr->m_ev_vehicle_status                        = m_cached_signals.find(vehicle::SignalID::EvVehicleStatus)->second;
        ptr->m_ev_charging_status                       = m_cached_signals.find(vehicle::SignalID::EvChargingStatus)->second;
        ptr->m_ev_working_status                        = m_cached_signals.find(vehicle::SignalID::EvWorkingStatus)->second;
        ptr->m_ev_vehicle_speed                         = m_cached_signals.find(vehicle::SignalID::EvVehicleSpeed)->second;
        ptr->m_ev_total_voltage                         = m_cached_signals.find(vehicle::SignalID::EvTotalVoltage)->second;
        ptr->m_ev_total_current                         = m_cached_signals.find(vehicle::SignalID::EvTotalCurrent)->second;
        ptr->m_ev_battery_soc                           = m_cached_signals.find(vehicle::SignalID::EvBatterySoc)->second;
        ptr->m_ev_dc_working_status                     = m_cached_signals.find(vehicle::SignalID::EvDcDcWoringStatus)->second;

        ptr->m_ev_insulation_resistance                 = m_cached_signals.find(vehicle::SignalID::EvInsulationResistance)->second;

        ptr->m_location.m_status                        = m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second;
        ptr->m_location.m_heading                       = m_cached_signals.find(vehicle::SignalID::GNSSHeading)->second;
        SMLK_LOGI("gnss heading val is %f", m_cached_signals.find(vehicle::SignalID::GNSSHeading)->second.Val());
        ptr->m_location.m_latitude                      = m_cached_signals.find(vehicle::SignalID::GNSSLatitude)->second;
        ptr->m_location.m_longitude                     = m_cached_signals.find(vehicle::SignalID::GNSSLongitude)->second;
        ptr->m_location.m_altitude                      = m_cached_signals.find(vehicle::SignalID::GNSSAltitude)->second;
        ptr->m_location.m_speed                         = m_cached_signals.find(vehicle::SignalID::GNSSSpeed)->second.Val() * M_PER_S_TO_KM_PER_HOUR;
        // here timestamp should mean the sampling timestamp, so it should be the system time

        ptr->m_tires.clear();

        static constexpr auto   ms_invalid = std::chrono::milliseconds((SL_CYCLES_SIGNAL_VAL_VALID - 1) * SL_PERIOD_TIRE_VALID);

        std::vector<SMLK_UINT8> expired;

        auto expired_at = std::chrono::steady_clock::now() + ms_invalid;
        for ( auto const &tire : m_cached_tires ) {
            std::shared_ptr<InternalStruct::TireInfo>   sp = std::static_pointer_cast<InternalStruct::TireInfo>(tire.second);
            if ( sp->m_expired_at > expired_at ) {
                expired.push_back(tire.first);
                continue;
            }

            TireInfo    info;
            info.m_id               = tire.first;
            info.m_temperature      = sp->m_temperature.Val();
            info.m_pressure         = sp->m_pressure.Val();
            info.m_extend_pressure  = sp->m_extend_pressure.Val();

            ptr->m_tires.push_back(info);
        }

        for ( auto location : expired ) {
            m_cached_tires.erase(location);
        }

        SMLK_UINT8 ev_gear = 0;
        // Bit7 Bit6：预留，预留位用0表示
        // Bit5：1：有驱动力：加速踏板开度 ＞ 0; 0：无驱动力：加速踏板开度 = 0

        if (ptr->m_acc_pedal_travel.Valid()) {
            if (ptr->m_acc_pedal_travel.Val() > 0.0) {
                ev_gear |= 0x20;
            }
        }

        // Bit4：1：有制动力: Brake switch = 01; 0：无制动力: Brake switch = 00
        if (ptr->m_parking.Valid()) {
            if (ptr->m_parking.Encoded() == 1) {
                ev_gear |= 0x10;
            }
        }

        // Bit3-0：挡位：=0000 空挡; =0001 1挡; =0010 2挡; =0011 3挡; =0100 4挡; =0101 5挡; =0110 6挡; =1101 倒挡; =1110 自动D挡; =1111 停车P挡;
        if (ptr->m_gear.Valid()) {
            if ((ptr->m_gear.Encoded() > 0x7D)) {
                // =1110 自动D挡: Current Gear > 0x7D
                ev_gear |= 0x0E;
            } else if (ptr->m_gear.Encoded() < 0x7D) {
                // =1101倒挡: Current Gear < 0x7D
                ev_gear|= 0x0D;
            } else if (ptr->m_gear.Encoded() == 0x7D) {
                // =0000 空挡: Current Gear = 0x7D
                ev_gear |= 0x00;
            }
        }
        m_cached_signals.find(vehicle::SignalID::EvGearStatus)->second = ev_gear;
        ptr->m_ev_gear_status                           = m_cached_signals.find(vehicle::SignalID::EvGearStatus)->second;
    }
}

/******************************************************************************
 * NAME: SamplingPeriod1S
 *
 * DESCRIPTION: sampling 1s bucket information
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::SamplingPeriod1S()
{
    SMLK_LOGI("SamplingPeriod1S");

    static const std::array<vehicle::SignalID, 13>  air_suspension_control_1_sub_signals = {
        vehicle::SignalID::NominalLevelFrontAxle,
        vehicle::SignalID::NominalLevelRearAxle,
        vehicle::SignalID::BelowNominalLevelFrontAxle,
        vehicle::SignalID::BelowNominalLevelRearAxle,
        vehicle::SignalID::AboveNominalLevelFrontAxle,
        vehicle::SignalID::AboveNominalLevelRearAxle,
        vehicle::SignalID::LoweringControlModeFrontAxle,
        vehicle::SignalID::LoweringControlModeRearAxle,
        vehicle::SignalID::LiftingControlModeFrontAxle,
        vehicle::SignalID::LiftingControlModeRearAxle,
        vehicle::SignalID::LevelControlMode,
        vehicle::SignalID::LiftAxle1Position,
        vehicle::SignalID::FrontAxleInBumperRange,
    };

    static const std::array<vehicle::SignalID, 4>   air_suspension_control_2_sub_signals = {
        vehicle::SignalID::RearAxleInBumperRange,
        vehicle::SignalID::LiftAxle2Position,
        vehicle::SignalID::SuspensionRemoteControl1,
        vehicle::SignalID::SuspensionControlRefusalInformation,
    };

    static const std::array<vehicle::SignalID, 4>   air_suspension_control_3_sub_signals = {
        vehicle::SignalID::RelativeLevelFrontAxleLeft,
        vehicle::SignalID::RelativeLevelFrontAxleRight,
        vehicle::SignalID::RelativeLevelRearAxleLeft,
        vehicle::SignalID::RelativeLevelRearAxleRight,
    };

    static const std::array<vehicle::SignalID, 4>   air_suspension_control_4_sub_signals = {
        vehicle::SignalID::BellowPressureFrontAxleLeft,
        vehicle::SignalID::BellowPressureFrontAxleRight,
        vehicle::SignalID::BellowPressureRearAxleLeft,
        vehicle::SignalID::BellowPressureRearAxleRight,
    };

    static const std::array<vehicle::SignalID, 2>   gnss_sub_signals = {
        vehicle::SignalID::GNSSLocatedStatus,
        vehicle::SignalID::GNSSHeading,
    };

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageBucket> ptr = std::static_pointer_cast<MessageBucket>(m_protocol_messages[protocol][MessageType::SL_BUCKET]);

        ptr->m_signals.emplace_back();

        for ( auto const signal : air_suspension_control_1_sub_signals ) {
            ptr->m_signals.back().emplace(
                std::piecewise_construct,
                std::forward_as_tuple(signal),
                std::forward_as_tuple(m_cached_signals.find(signal)->second)
            );
        }

        for ( auto const signal : air_suspension_control_2_sub_signals ) {
            ptr->m_signals.back().emplace(
                std::piecewise_construct,
                std::forward_as_tuple(signal),
                std::forward_as_tuple(m_cached_signals.find(signal)->second)
            );
        }

        for ( auto const signal : air_suspension_control_3_sub_signals ) {
            ptr->m_signals.back().emplace(
                std::piecewise_construct,
                std::forward_as_tuple(signal),
                std::forward_as_tuple(m_cached_signals.find(signal)->second)
            );
        }

        for ( auto const signal : air_suspension_control_4_sub_signals ) {
            ptr->m_signals.back().emplace(
                std::piecewise_construct,
                std::forward_as_tuple(signal),
                std::forward_as_tuple(m_cached_signals.find(signal)->second)
            );
        }

        for ( auto const signal : gnss_sub_signals ) {
            ptr->m_signals.back().emplace(
                std::piecewise_construct,
                std::forward_as_tuple(signal),
                std::forward_as_tuple(m_cached_signals.find(signal)->second)
            );
        }

        for ( auto const signal : ptr->SignalIDs() ) {
            if (vehicle::SignalID::TBoxAcceleration == signal)
            {
                if (false == m_ig_on_to_off) {
                    auto& signal_speed = m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second;
                    static SMLK_DOUBLE last_speed = 0;
                    SMLK_DOUBLE current_speed_m_per_s = KM_PER_HOUR_TO_M_PER_S(signal_speed.Val());
                    m_cached_signals.find(vehicle::SignalID::TBoxAcceleration)->second = current_speed_m_per_s - last_speed;
    #ifdef SL_DEBUG
                    SMLK_LOGD("TBoxAcceleration current_speed_km_per_h === [%lf]", signal_speed.Val());
                    SMLK_LOGD("TBoxAcceleration last_speed_m_per_s === [%lf]", last_speed);
                    SMLK_LOGD("TBoxAcceleration acce speed === [%lf]", m_cached_signals.find(signal)->second.Val());
    #endif
                    last_speed = current_speed_m_per_s;
                    if (!m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second.Valid()) {
                        // TAPD:1018345
                        m_cached_signals.find(signal)->second.Invalid();
                    }
                } else {
                        std::lock_guard<std::mutex> lk(m_cache_map_mutex);
                        auto& signal_speed = m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second;
                        static SMLK_DOUBLE last_speed = 0;
                        SMLK_DOUBLE current_speed_m_per_s = KM_PER_HOUR_TO_M_PER_S(signal_speed.Val());
                        m_cached_signals.find(vehicle::SignalID::TBoxAcceleration)->second = current_speed_m_per_s - last_speed;
                        last_speed = current_speed_m_per_s;
                        if (!m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second.Valid()) {
                            // TAPD:1018345
                            m_cached_signals.find(signal)->second.Invalid();
                        }
                }
            } else if (vehicle::SignalID::VehicleSwitchStatus == signal) {
                if (false == m_ig_on_to_off) {
                    auto& signal = m_cached_signals.find(vehicle::SignalID::VehicleSwitchStatus)->second;
                    if ( !signal.Valid() )
                    {
                        signal.SetValid();
                    }
                    if (m_cached_signals.find(vehicle::SignalID::ParkingBrakeSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_PARK_BRAKE_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_PARK_BRAKE_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::ClutchSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CLUTCH_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CLUTCH_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::BrakeSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_BRAKE_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_BRAKE_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::ACCStatus)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_ACC_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_ACC_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_AsrBrakeControl)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_ABS_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_ABS_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_AsrEngineControl)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TCS_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TCS_SWITCH));
                    }

                    if (m_internal_flags&InternalFlags::SL_FLAG_ETC2_RECEIVED) {
                        if (m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second.Val() == 0)
                        {
                            signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CURRENT_GEAR_SWITCH));
                        }
                        else
                        {
                            signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CURRENT_GEAR_SWITCH));
                        }
                    } else {
                        if (m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second.Val() == 48)
                        {
                            signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CURRENT_GEAR_SWITCH));
                        }
                        else
                        {
                            signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CURRENT_GEAR_SWITCH));
                        }
                    }
                    // TrailerConnected 1:connected  0:except connected
                    if ( TrailerConnectedSignal::SL_TRAILER_CONNECTED == m_cached_signals.find(vehicle::SignalID::SMLK_TrailerConnected)->second.Val())
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TRILER_CONNECT_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TRILER_CONNECT_SWITCH));
                    }
                    // OpenStatusDoor bitValue 1:door closed  0:except closed
                    // TAPD 1017017 -> 1028615: complete by requiremrnt not care personal explanation
                    if (DoorSignalStatus::SL_ALL_DOOR_SIGNAL_RECEIVED != m_door_signal_status) {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_DOOR_OPEN_SWITCH));
                    } else {
                        if ( (DoorOpenSignal::SL_DOOR_CLOSED == m_cached_signals.find(vehicle::SignalID::SMLK_OpenStatusDoorDrv)->second.Val())
                            && (DoorOpenSignal::SL_DOOR_CLOSED == m_cached_signals.find(vehicle::SignalID::SMLK_OpenStatusDoorPas)->second.Val()) )
                        {
                            signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_DOOR_OPEN_SWITCH));
                        }
                        else
                        {
                            signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_DOOR_OPEN_SWITCH));
                        }
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_FuelLeakageAlert)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT0));
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT1)); // 燃油泄露 reserved:10 not care:11
                    } else if (m_cached_signals.find(vehicle::SignalID::SMLK_FuelLeakageAlert)->second.Val() == 2) {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT0));
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT1)); // 燃油泄露 reserved:10 not care:11
                    } else if (m_cached_signals.find(vehicle::SignalID::SMLK_FuelLeakageAlert)->second.Val() == 3) {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT0));
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT1)); // 燃油泄露 reserved:10 not care:11
                    } else {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT0));
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT1)); // 燃油泄露 reserved:10 not care:11
                    }

                    if (m_cached_signals.find(vehicle::SignalID::TransmissionMode1)->second.Val() == 0) {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT0));
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT1)); // EP模式表 reserve:10 invalid :11
                    } else if (m_cached_signals.find(vehicle::SignalID::TransmissionMode1)->second.Val() == 1) {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT0));
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT1)); // EP模式表 reserve:10 invalid :11
                    } else if (m_cached_signals.find(vehicle::SignalID::TransmissionMode1)->second.Val() == 2) {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT0));
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT1)); // EP模式表 reserve:10 invalid :11
                    } else if (m_cached_signals.find(vehicle::SignalID::TransmissionMode1)->second.Val() == 3) {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT0));
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT1)); // EP模式表 reserve:10 invalid :11
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_CruiseControl)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CRUISE_ACTIVE_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CRUISE_ACTIVE_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_TrailerParkBrakeSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TRAILER_PARKE_BRAKE_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TRAILER_PARKE_BRAKE_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_ReverseGearSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_REVERSE_GEAR_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_REVERSE_GEAR_SWITCH));
                    }
                } else {
                    std::lock_guard<std::mutex> lk(m_cache_map_mutex);
                    auto& signal = m_cached_signals.find(vehicle::SignalID::VehicleSwitchStatus)->second;
                    if ( !signal.Valid() )
                    {
                        signal.SetValid();
                    }
                    if (m_cached_signals.find(vehicle::SignalID::ParkingBrakeSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_PARK_BRAKE_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_PARK_BRAKE_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::ClutchSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CLUTCH_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CLUTCH_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::BrakeSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_BRAKE_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_BRAKE_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::ACCStatus)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_ACC_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_ACC_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_AsrBrakeControl)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_ABS_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_ABS_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_AsrEngineControl)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TCS_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TCS_SWITCH));
                    }

                    if (m_internal_flags&InternalFlags::SL_FLAG_ETC2_RECEIVED) {
                        if (m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second.Val() == 0)
                        {
                            signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CURRENT_GEAR_SWITCH));
                        }
                        else
                        {
                            signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CURRENT_GEAR_SWITCH));
                        }
                    } else {
                        if (m_cached_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second.Val() == 48)
                        {
                            signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CURRENT_GEAR_SWITCH));
                        }
                        else
                        {
                            signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CURRENT_GEAR_SWITCH));
                        }
                    }
                    // TrailerConnected 1:connected  0:except connected
                    if ( TrailerConnectedSignal::SL_TRAILER_CONNECTED == m_cached_signals.find(vehicle::SignalID::SMLK_TrailerConnected)->second.Val())
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TRILER_CONNECT_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TRILER_CONNECT_SWITCH));
                    }
                    // OpenStatusDoor bitValue 1:door closed  0:except closed
                    // TAPD 1017017 -> 1028615: complete by requiremrnt not care personal explanation
                    if (DoorSignalStatus::SL_ALL_DOOR_SIGNAL_RECEIVED != m_door_signal_status) {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_DOOR_OPEN_SWITCH));
                    } else {
                        if ( (DoorOpenSignal::SL_DOOR_CLOSED == m_cached_signals.find(vehicle::SignalID::SMLK_OpenStatusDoorDrv)->second.Val())
                            && (DoorOpenSignal::SL_DOOR_CLOSED == m_cached_signals.find(vehicle::SignalID::SMLK_OpenStatusDoorPas)->second.Val()) )
                        {
                            signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_DOOR_OPEN_SWITCH));
                        }
                        else
                        {
                            signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_DOOR_OPEN_SWITCH));
                        }
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_FuelLeakageAlert)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT0));
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT1)); // 燃油泄露 reserved:10 not care:11
                    } else if (m_cached_signals.find(vehicle::SignalID::SMLK_FuelLeakageAlert)->second.Val() == 2) {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT0));
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT1)); // 燃油泄露 reserved:10 not care:11
                    } else if (m_cached_signals.find(vehicle::SignalID::SMLK_FuelLeakageAlert)->second.Val() == 3) {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT0));
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT1)); // 燃油泄露 reserved:10 not care:11
                    } else {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT0));
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_FUEL_LEAK_SWITCH_BIT1)); // 燃油泄露 reserved:10 not care:11
                    }

                    if (m_cached_signals.find(vehicle::SignalID::TransmissionMode1)->second.Val() == 0) {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT0));
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT1)); // EP模式表 reserve:10 invalid :11
                    } else if (m_cached_signals.find(vehicle::SignalID::TransmissionMode1)->second.Val() == 1) {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT0));
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT1)); // EP模式表 reserve:10 invalid :11
                    } else if (m_cached_signals.find(vehicle::SignalID::TransmissionMode1)->second.Val() == 2) {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT0));
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT1)); // EP模式表 reserve:10 invalid :11
                    } else if (m_cached_signals.find(vehicle::SignalID::TransmissionMode1)->second.Val() == 3) {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT0));
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_EP_MODE_SWITCH_BIT1)); // EP模式表 reserve:10 invalid :11
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_CruiseControl)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CRUISE_ACTIVE_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_CRUISE_ACTIVE_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_TrailerParkBrakeSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TRAILER_PARKE_BRAKE_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_TRAILER_PARKE_BRAKE_SWITCH));
                    }

                    if (m_cached_signals.find(vehicle::SignalID::SMLK_ReverseGearSwitch)->second.Val() == 1)
                    {
                        signal.AssignEncoded(SETBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_REVERSE_GEAR_SWITCH));
                    }
                    else
                    {
                        signal.AssignEncoded(CLRBIT_PARAM(signal.Encoded(), VehicleSwitchBitMask::SL_REVERSE_GEAR_SWITCH));
                    }
                }
            }
            // TotalCatalystSpewed should always report valid value
            if (vehicle::SignalID::TotalCatalystSpewed == signal) {
                if (false == m_ig_on_to_off) {
                    auto& signal_totalCataly = m_cached_signals.find(vehicle::SignalID::TotalCatalystSpewed)->second;
                    signal_totalCataly.SetValid();
                } else {
                    std::lock_guard<std::mutex> lk(m_cache_map_mutex);
                    auto& signal_totalCataly = m_cached_signals.find(vehicle::SignalID::TotalCatalystSpewed)->second;
                    signal_totalCataly.SetValid();
                }
            }

            ptr->m_signals.back().emplace(
                std::piecewise_construct,
                std::forward_as_tuple(signal),
                std::forward_as_tuple(m_cached_signals.find(signal)->second)
            );
        }

        ptr->i_lckcar_sts = i_lckcar_sts;

        auto it = ptr->m_signals.back().find(vehicle::SignalID::GNSSTimeStamp);
        if ( !CheckWhetherTimeSync() ) {
            // 从未授时时,先缓存数据,然后在授时后修改缓存的GNSSTimeStamp字段,从缓存中取出所有数据发送
            if ( ptr->m_signals.back().end() != it ) {
                it->second = m_cached_signals.find(vehicle::SignalID::GNSSTimeStamp)->second;
            }
        } else {
           // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
            it->second = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_timeSyncGap_ms/1000);
        }

        auto iter_speed = ptr->m_signals.back().find(vehicle::SignalID::GNSSSpeed);
        if ( ptr->m_signals.back().end() != iter_speed ) {
            iter_speed->second = m_cached_signals.find(vehicle::SignalID::GNSSSpeed)->second.Val() * M_PER_S_TO_KM_PER_HOUR;
        }

        // be careful, here is a mismatch between the CAN signal and report
        static const std::array<SMLK_UINT8, 8>  tire_pressure_threshold_detection = {
            TirePressureThreshold::Report::SL_PRESSURE_INVALID,
            TirePressureThreshold::Report::SL_PRESSURE_OVER,
            TirePressureThreshold::Report::SL_PRESSURE_NORMAL,
            TirePressureThreshold::Report::SL_PRESSURE_UNDER,
            TirePressureThreshold::Report::SL_PRESSURE_INVALID,
            TirePressureThreshold::Report::SL_PRESSURE_INVALID,
            TirePressureThreshold::Report::SL_PRESSURE_INVALID,
            TirePressureThreshold::Report::SL_PRESSURE_INVALID,
        };
        it = ptr->m_signals.back().find(vehicle::SignalID::TirePressureThresholdDetection);
        if ( ptr->m_signals.back().end() != it ) {
            auto signal(m_cached_signals.find(vehicle::SignalID::TirePressureThresholdDetectionOriginal)->second);

            if ( (std::size_t)signal.Val() >= tire_pressure_threshold_detection.size() ) {
                it->second = TirePressureThreshold::Report::SL_PRESSURE_INVALID;
            } else {
                it->second = tire_pressure_threshold_detection[(std::size_t)signal.Val()];
            }
            if (!signal.Valid()) {
                it->second.Invalid();
            }
        }

        {
            // the unit of calculus day distance is meter instead of km
            auto const &signal = m_cached_signals.find(vehicle::SignalID::CalculusDayDistance)->second;

            ptr->m_signal_day_vehicle_distance  = signal.Val();
            if ( !signal.Valid() ) {
                ptr->m_signal_day_vehicle_distance.Invalid();
            }
        }

        ptr->m_signal_total_vehicle_distance    = m_cached_signals.find(vehicle::SignalID::TotalVehicleDistance)->second;
        ptr->m_signal_ecu_total_distacne        = m_cached_signals.find(vehicle::SignalID::TotalECUDistance)->second;
        {
            // vec send L and convert to tsp 0.1L
            auto const &signal = m_cached_signals.find(vehicle::SignalID::CalculusDayFuelConsumption)->second;
            ptr->m_signal_day_fuel_used         = signal.Val();
            if ( !signal.Valid() ) {
                ptr->m_signal_day_fuel_used.Invalid();
            }
        }

        ptr->m_signal_total_fuel_used           = m_cached_signals.find(vehicle::SignalID::CalculusFuelConsumption)->second;
        ptr->m_signal_ecu_total_fuel_used       = m_cached_signals.find(vehicle::SignalID::EngineTotalFuelUsed)->second;

        if ( ptr->m_signals.size() >= m_bucket_num_1s ) {
            auto it = m_frequency_value.find(Frequency::SL_1S);
            if (m_frequency_value.end() != it)
            {
                ptr->m_sampling_frequency = it->second / 10;
            }
            else{
                ptr->m_sampling_frequency = 100;
            }

            std::vector<SMLK_UINT8> encoded;
            encoded.reserve(MAX_ETHERNET_SEND_SIZE);

            ptr->Encode(encoded);
            ptr->m_signals.clear();

            if ( m_internal_flags & InternalFlags::SL_FLAG_COMPRESS ) {
                encoded.erase(encoded.begin(), encoded.begin() + ptr->CompressOffset());
                MsgHead head(M_SL_MSGID_COMPRESS_REQ, 0, protocol, 0, (SMLK_UINT16)(Frequency::SL_1S) << 8, 0);
                m_compress_events.emplace(
                    head,
                    std::move(encoded),
                    std::bind(&CollectionService::OnCompress, this, std::placeholders::_1, std::placeholders::_2)
                );
            } else {
                IpcTspHead  resp_head;

                std::memset(&resp_head, 0x00, sizeof(resp_head));

                resp_head.msg_id    = M_SL_MSGID_TP_BUCKET_REQ;
                resp_head.protocol  = protocol;
                resp_head.qos       = QOS_SEND_ALWAYS;

                TspServiceApi::getInstance()->SendMsg(resp_head, encoded.data(), encoded.size());
            }
        }
    }
}

/******************************************************************************
 * NAME: SamplingPeriod30S
 *
 * DESCRIPTION: sampling 30s bucket information
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::SamplingPeriod30S()
{
    SMLK_LOGI("SamplingPeriod30S");
#if 0
    {
        SMLK_UINT8  buffer[16384];
        auto inf    = reinterpret_cast<DriveEventInfo *>(buffer);


        inf->odo            = 3456.789;
        inf->fuel           = 6789.124;
        inf->time           = 5678.345;
        inf->location_num   = 30;

        auto total = sizeof(DriveEventInfo) + (std::size_t)inf->location_num * sizeof(inf->location_info[0]);

        for ( decltype(inf->location_num) index = 0; index < inf->location_num; index++ ) {
            inf->location_info[index].latitude  = 118.816758;
            inf->location_info[index].longitude = 31.919299;
            inf->location_info[index].altitude  = 18.3456789;
            inf->location_info[index].flag      = true;
            inf->location_info[index].speed     = 123.456;
            inf->location_info[index].direction = 30.456;
            inf->location_info[index].utc       = (std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) + index - inf->location_num) * 1000;
        }

        auto acc = reinterpret_cast<MajorAccident *>(&inf->location_info[inf->location_num]);
        acc->freeze_frame_num   = 30;

        total += sizeof(MajorAccident);
        total += std::size_t(acc->freeze_frame_num) * sizeof(acc->freeze_frame[0]);

        for ( decltype(acc->freeze_frame_num) index = 0; index < acc->freeze_frame_num; index++ ) {
            acc->type = MajorAccidentEventType::E_ACCIDENT_G_SENSOR;
            acc->freeze_frame[index].braking_sys_state  = 5;
            acc->freeze_frame[index].relevant_obj_id    = 4;
            acc->freeze_frame[index].obj_relative_speed = 60;
            acc->freeze_frame[index].obj_speed          = 16;
            acc->freeze_frame[index].obj_distance       = 70;
        }

        Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, M_SL_EVID_ACCIDENT, 0, 0, 0, 0), buffer, total);
        return;
    }
#endif

    static const std::array<vehicle::SignalID, 4>   transmission_mode = {
        vehicle::SignalID::TransmissionMode1,
        vehicle::SignalID::TransmissionMode2,
        vehicle::SignalID::TransmissionMode3,
        vehicle::SignalID::TransmissionMode4
    };

    static const std::array<vehicle::SignalID, 2>   gnss_sub_signals = {
        vehicle::SignalID::GNSSLocatedStatus,
        vehicle::SignalID::GNSSHeading,
    };

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageBucket30S> ptr = std::static_pointer_cast<MessageBucket30S>(m_protocol_messages[protocol][MessageType::SL_BUCKET_30S]);

        ptr->m_signals.emplace_back();

        for ( auto const signal : transmission_mode ) {
            ptr->m_signals.back().emplace(
                std::piecewise_construct,
                std::forward_as_tuple(signal),
                std::forward_as_tuple(m_cached_signals.find(signal)->second)
            );
        }

        for ( auto const signal : gnss_sub_signals ) {
            ptr->m_signals.back().emplace(
                std::piecewise_construct,
                std::forward_as_tuple(signal),
                std::forward_as_tuple(m_cached_signals.find(signal)->second)
            );
        }

        for ( auto const signal : ptr->SignalIDs() ) {
            ptr->m_signals.back().emplace(
                std::piecewise_construct,
                std::forward_as_tuple(signal),
                std::forward_as_tuple(m_cached_signals.find(signal)->second)
            );
        }

        auto it = ptr->m_signals.back().find(vehicle::SignalID::GNSSTimeStamp);
        if ( !CheckWhetherTimeSync() ) {
            // 从未授时时,先缓存数据,然后在授时后修改缓存的GNSSTimeStamp字段,从缓存中取出所有数据发送
            if ( ptr->m_signals.back().end() != it ) {
                it->second = m_cached_signals.find(vehicle::SignalID::GNSSTimeStamp)->second;
            }
        } else {
           // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
            it->second = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_timeSyncGap_ms/1000);
        }

        auto iter_speed = ptr->m_signals.back().find(vehicle::SignalID::GNSSSpeed);
        if ( ptr->m_signals.back().end() != iter_speed ) {
            iter_speed->second = m_cached_signals.find(vehicle::SignalID::GNSSSpeed)->second.Val() * M_PER_S_TO_KM_PER_HOUR;
        }

        {
            // the unit of calculus day distance is meter instead of km
            auto const &signal = m_cached_signals.find(vehicle::SignalID::CalculusDayDistance)->second;
            ptr->m_signal_day_vehicle_distance  = signal.Val();
            if ( !signal.Valid() ) {
                ptr->m_signal_day_vehicle_distance.Invalid();
            }
        }

        ptr->m_signal_total_vehicle_distance    = m_cached_signals.find(vehicle::SignalID::TotalVehicleDistance)->second;
        ptr->m_signal_ecu_total_distacne        = m_cached_signals.find(vehicle::SignalID::TotalECUDistance)->second;

        {
            // vec send L and convert to tsp 0.1L
            auto const &signal = m_cached_signals.find(vehicle::SignalID::CalculusDayFuelConsumption)->second;
            ptr->m_signal_day_fuel_used         = signal.Val();
            if ( !signal.Valid() ) {
                ptr->m_signal_day_fuel_used.Invalid();
            }
        }

        ptr->m_signal_total_fuel_used           = m_cached_signals.find(vehicle::SignalID::CalculusFuelConsumption)->second;
        ptr->m_signal_ecu_total_fuel_used       = m_cached_signals.find(vehicle::SignalID::EngineTotalFuelUsed)->second;

        if ( ptr->m_signals.size() >= m_bucket_num_30s ) {
            auto it = m_frequency_value.find(Frequency::SL_30S);
            if (m_frequency_value.end() != it)
            {
                ptr->m_sampling_frequency = it->second * 100;
            }
            else{
                ptr->m_sampling_frequency = 100;
            }

            std::vector<SMLK_UINT8> encoded;
            encoded.reserve(MAX_ETHERNET_SEND_SIZE);

            ptr->Encode(encoded);
            ptr->m_signals.clear();

            if ( m_internal_flags & InternalFlags::SL_FLAG_COMPRESS ) {
                encoded.erase(encoded.begin(), encoded.begin() + ptr->CompressOffset());

                MsgHead head(M_SL_MSGID_COMPRESS_REQ, 0, protocol, 0, ((SMLK_UINT16)Frequency::SL_30S) << 8, 0);

                m_compress_events.emplace(
                    head,
                    std::move(encoded),
                    std::bind(&CollectionService::OnCompress, this, std::placeholders::_1, std::placeholders::_2)
                );
            } else {
                IpcTspHead  resp_head;

                std::memset(&resp_head, 0x00, sizeof(resp_head));

                resp_head.msg_id    = M_SL_MSGID_TP_BUCKET_REQ;
                resp_head.protocol  = protocol;
                resp_head.qos       = QOS_SEND_ALWAYS;

                TspServiceApi::getInstance()->SendMsg(resp_head, encoded.data(), encoded.size());
            }
        }
    }
}

/******************************************************************************
 * NAME: CacheAllSignals
 *
 * DESCRIPTION: cache all signals
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::CacheAllSignals(void) {
    SMLK_LOGI("[timesync]system time not sync, cache all signals");
    m_cached_signals_pool.emplace(
        std::chrono::steady_clock::now(),
        cached_item_hash()
    );

    for ( auto const &pair : m_cached_signals ) {
        m_cached_signals_pool.back().second.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(pair.first),
            std::forward_as_tuple(pair.second)
        );
    }
}

/******************************************************************************
 * NAME: GetGNSS
 *
 * DESCRIPTION: get GNSS information
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::GetGNSS()
{
    smartlink_sdk::LocationInfo location;
    auto rc = smartlink_sdk::Location::GetInstance()->GetLocation(location);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != rc ) {
        SMLK_LOGE("[GNSS] fail to get location information, rc: %d", static_cast<std::int32_t>(rc));
    } else {
        if ( m_internal_flags&InternalFlags::SL_FLAG_PERIOD_WAKEUP ) {
        // if not located no update
            if (location.fix_valid) {
                m_cached_signals.find(vehicle::SignalID::GNSSSpeed)->second             = location.speed;
                m_cached_signals.find(vehicle::SignalID::GNSSHeading)->second           = location.heading;
                m_cached_signals.find(vehicle::SignalID::GNSSAltitude)->second          = location.altitude;
                m_cached_signals.find(vehicle::SignalID::GNSSLatitude)->second          = location.latitude;
                m_cached_signals.find(vehicle::SignalID::GNSSLongitude)->second         = location.longitude;
                m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second     = location.fix_valid ? 0 : 1;
                m_cached_signals.find(vehicle::SignalID::GNSSTimeStamp)->second         = location.utc * 0.001 ; // ms->s
            } else {
                m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second     = 1;
            }
        } else {
            m_cached_signals.find(vehicle::SignalID::GNSSSpeed)->second             = location.speed;
            m_cached_signals.find(vehicle::SignalID::GNSSHeading)->second           = location.heading;
            m_cached_signals.find(vehicle::SignalID::GNSSAltitude)->second          = location.altitude;
            m_cached_signals.find(vehicle::SignalID::GNSSLatitude)->second          = location.latitude;
            m_cached_signals.find(vehicle::SignalID::GNSSLongitude)->second         = location.longitude;
            m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second     = location.fix_valid ? 0 : 1;
            m_cached_signals.find(vehicle::SignalID::GNSSTimeStamp)->second         = location.utc * 0.001 ; // ms->s
        }
    }
    smartlink_sdk::GnssSatelliteInfo    inf;
    rc = smartlink_sdk::Location::GetInstance()->GetSatelliteInfo(inf);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != rc ) {
        SMLK_LOGE("[GNSS] fail to get satellite information, rc: %d", static_cast<std::int32_t>(rc));
    } else {
#ifdef SL_DEBUG
        SMLK_LOGD("[GNSS] inf.tracked_num:    %u", inf.tracked_num);
#endif
        m_cached_signals.find(vehicle::SignalID::GNSSSatelliteUsed)->second = inf.tracked_num;
    }
}

/******************************************************************************
 * NAME: ReportLocation
 *
 * DESCRIPTION: cyclic reporting location information
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::ReportLocation(void) {
#ifdef SL_DEBUG
    SMLK_LOGD(" on ReportLocation");
#endif
    SamplingLocation();

    std::vector<SMLK_UINT8> encoded;
    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageLocation>    ptr = std::static_pointer_cast<MessageLocation>(m_protocol_messages[protocol][MessageType::SL_LOCATION]);

        ptr->m_report_location_only = 0 != (m_internal_flags & InternalFlags::SL_FLAG_PERIOD_WAKEUP);

        encoded.clear();
        ptr->Encode(encoded);
#ifdef SL_DEBUG
        std::stringstream   ss;
#ifdef PLATFORM_ARM
        std::size_t         index   = 0;
        std::size_t         seq     = 0;
        for ( auto ch : encoded ) {
            ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
            if ( ++index >=  SL_PLATFORM_MAX_BYTES ) {
                // SMLK_LOGD("ReportLocation data [%03u]%s", seq, ss.str().c_str());
                ss.str("");
                ss.clear();
                seq++;
                index = 0;
            }
        }
        // SMLK_LOGD("data: [%03u]%s", seq, ss.str().c_str());
#else
        for ( auto ch : encoded ) {
            ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
        }

        // SMLK_LOGD("%s", ss.str().c_str());
#endif  // #ifdef PLATFORM_ARM
#endif  // #ifdef SL_DEBUG

        IpcTspHead  head;
        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_LOCATION_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;

       if ( m_internal_flags&InternalFlags::SL_FLAG_PERIOD_WAKEUP) {
            if ( 0 != m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second.Encoded()) {
                SMLK_LOGI("[WakeupDebug] located is invalid, also need report now");
            }
            TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
       } else {
            SMLK_LOGD("send data to tsp and size is == %d", (SMLK_UINT8)encoded.size());
            TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
       }
    }
    if ( m_internal_flags & InternalFlags::SL_FLAG_PERIOD_WAKEUP ) {
        SMLK_LOGI("[WakeupDebug] m_internal_flags is %d and m_wakeup_report_cnt is %d", m_internal_flags, m_wakeup_report_cnt);
        SMLK_LOGI("[WakeupDebug] GNSSLocatedStatus is %d", m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second.Encoded() );
        if ( (0 == m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second.Encoded())
          || (0 == --m_wakeup_report_cnt) ) {
            m_wakeup_report_cnt = 0;
            m_internal_flags &= ~InternalFlags::SL_FLAG_PERIOD_WAKEUP;
            // make sure not in sampling condition , example : when rtc wakeup, can bus wakeup came
            if ( !CheckSamplingCondition() ) {
                if ( m_timer_flags&TimerFlags::SL_FLAG_0200_TIMER_START ) {
                    SMLK_LOGI("[timerDebug] 0200 timer reset ");
                    m_0200_timer->Reset();
                }
            }
            SMLK_LOGI("[WakeupDebug] gnss located or m_wakeup_report_cnt=0, unlock wake");
            smartlink_sdk::SysPower::GetInstance()->UnLockWake(std::string(SL_PROCESS_NAME));
        }
    }
}

/******************************************************************************
 * NAME: ReportStatistics
 *
 * DESCRIPTION: cyclic reporting statistics information
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::ReportStatistics()
{
    SMLK_LOGI("ReportStatistics");

    std::vector<SMLK_UINT8> encoded;
    encoded.reserve(MAX_ETHERNET_SEND_SIZE);

    SMLK_LOGI("TripStartTime valid is %d", m_cached_signals.find(vehicle::SignalID::StatisticsTripStartTime)->second.Valid());
    SMLK_LOGI("TripEndTime valid is %d", m_cached_signals.find(vehicle::SignalID::StatisticsTripEndTime)->second.Valid());

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageStatistics>  ptr = std::static_pointer_cast<MessageStatistics>(m_protocol_messages[protocol][MessageType::SL_STATISTICS]);
        if (ptr->m_trip_start.Valid() && ptr->m_trip_stop.Valid()) {
            if ((ptr->m_trip_stop.Val() > ptr->m_trip_start.Val()) || (0 == (m_internal_flags & InternalFlags::SL_FLAG_IGNON))) {
                ptr->m_flags |= StatisticsMask::SL_TRIP_FINISHED;
            } else {
                ptr->m_flags &= ~StatisticsMask::SL_TRIP_FINISHED;
            }
            if (m_internal_flags & InternalFlags::SL_FLAG_DAY_CHANGE) {
                ptr->m_flags |= StatisticsMask::SL_ACROSS_DAY;
            } else {
                ptr->m_flags &= ~StatisticsMask::SL_ACROSS_DAY;
            }
        }

        ptr->m_location.m_status                            = m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second;
        ptr->m_location.m_latitude                          = m_cached_signals.find(vehicle::SignalID::GNSSLatitude)->second;
        ptr->m_location.m_longitude                         = m_cached_signals.find(vehicle::SignalID::GNSSLongitude)->second;
        ptr->m_location.m_altitude                          = m_cached_signals.find(vehicle::SignalID::GNSSAltitude)->second;
        ptr->m_location.m_heading                           = m_cached_signals.find(vehicle::SignalID::GNSSHeading)->second;
        ptr->m_location.m_speed                             = m_cached_signals.find(vehicle::SignalID::GNSSSpeed)->second.Val() * M_PER_S_TO_KM_PER_HOUR;

        if ( !CheckWhetherTimeSync() ) {
            // 从未授时时,先缓存数据,然后在授时后修改缓存的GNSSTimeStamp字段,从缓存中取出所有数据发送
            ptr->m_location.m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        } else {
           // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
            if (m_internal_flags&InternalFlags::SL_FLAG_DAY_CHANGE) {
                // 跨天包需要保证为0:00:00
                ptr->m_location.m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            } else {
                ptr->m_location.m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_timeSyncGap_ms/1000);
            }
        }

        //行程开始时间补偿在信号接收处 OnDataChanged()。
        ptr->m_trip_start                                   = m_cached_signals.find(vehicle::SignalID::StatisticsTripStartTime)->second;

        if (0 == (m_internal_flags & InternalFlags::SL_FLAG_IGNON))
        {
            if ( !CheckWhetherTimeSync() ) {
                // TDDO: 从未授时时,先缓存数据,然后在授时后修改缓存的GNSSTimeStamp字段,从缓存中取出所有数据发送
                // ptr->m_location.m_timestamp = m_cached_signals.find(vehicle::SignalID::GNSSTimeStamp)->second;
                // 暂时使用系统时间
                ptr->m_trip_stop = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            } else {
                // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
                ptr->m_trip_stop = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_timeSyncGap_ms/1000);
            }
        }
        else
        {
            ptr->m_trip_stop                                = m_cached_signals.find(vehicle::SignalID::StatisticsTripEndTime)->second;
        }

        ptr->m_gnss_statistics_distance                     = m_cached_signals.find(vehicle::SignalID::StatisticsGnssSpeedDistance)->second;

        ptr->m_tachograph_vehicle_speed_statistics_distance = m_cached_signals.find(vehicle::SignalID::StatisticsTachographVehicleSpeedDistance)->second;
        // ptr->m_tachograph_vehicle_speed_statistics_distance = m_cached_signals.find(vehicle::SignalID::CalculusDistance)->second;               // confirmed to use the calculus distance
        ptr->m_vehicle_speed_statistics_life_total_distance = m_cached_signals.find(vehicle::SignalID::StatisticsVehicleSpeedLifeTotalDistance)->second;
        ptr->m_vehicle_total_distance                       = m_cached_signals.find(vehicle::SignalID::TotalVehicleDistance)->second.Val();
        ptr->m_vehicle_estimated_load                       = m_cached_signals.find(vehicle::SignalID::TrailerWeight)->second;
        ptr->m_trip_statistics_fuel_used                    = m_cached_signals.find(vehicle::SignalID::StatisticsTripFuelUsed)->second;
        ptr->m_total_statistics_fuel_used                   = m_cached_signals.find(vehicle::SignalID::CalculusFuelConsumption)->second.Val();        // the source CAN signal is the same with calculus
        ptr->m_engine_total_fuel_used                       = m_cached_signals.find(vehicle::SignalID::EngineTotalFuelUsed)->second;
        ptr->m_engine_average_fuel_economy                  = m_cached_signals.find(vehicle::SignalID::StatisticsAverageFuelEconomy)->second;
        ptr->m_speeding_times                               = m_cached_signals.find(vehicle::SignalID::StatisticsSpeedingTimes)->second;
        ptr->m_speeding_distance                            = m_cached_signals.find(vehicle::SignalID::StatisticsSpeedingDistance)->second;
        ptr->m_speeding_fuel_used                           = m_cached_signals.find(vehicle::SignalID::StatisticsSpeedingFuelUsed)->second;
        ptr->m_brake_pedal_times                            = m_cached_signals.find(vehicle::SignalID::StatisticsBrakePedalBrakeTimes)->second;
        ptr->m_brake_pedal_brake_distance                   = m_cached_signals.find(vehicle::SignalID::StatisticsBrakePedalBrakeDistance)->second;
        ptr->m_brake_pedal_brake_duration                   = m_cached_signals.find(vehicle::SignalID::StatisticsBrakePedalBrakeDuration)->second;
        ptr->m_sharp_slowdown_times                         = m_cached_signals.find(vehicle::SignalID::StatisticsSharpShowDownTimes)->second;
        ptr->m_sharp_slowdown_duration                      = m_cached_signals.find(vehicle::SignalID::StatisticsSharpShowDownDuration)->second;
        ptr->m_sharp_slowdown_distance                      = m_cached_signals.find(vehicle::SignalID::StatisticsSharpShowDownDistance)->second;
        ptr->m_sharp_speedup_times                          = m_cached_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpTimes)->second;
        ptr->m_sharp_speedup_duration                       = m_cached_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpDuration)->second;
        ptr->m_sharp_speedup_distance                       = m_cached_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpDistance)->second;
        ptr->m_sharp_speedup_fuel_used                      = m_cached_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpFuelUsed)->second;
        ptr->m_sharp_turnning_times                         = m_cached_signals.find(vehicle::SignalID::StatisticsSharpTrunningTimes)->second;
        ptr->m_sharp_turnning_distance                      = m_cached_signals.find(vehicle::SignalID::StatisticsSharpTrunningDistance)->second;
        ptr->m_sleepy_drive_times                           = m_cached_signals.find(vehicle::SignalID::StatisticsSleepyDriveTimes)->second;

        encoded.clear();
        ptr->Encode(encoded);

        IpcTspHead  head;

        std::memset(&head, 0x00, sizeof(head));

        head.msg_id     = M_SL_MSGID_TP_STATISTICS_REQ;
        head.protocol   = protocol;
        head.qos        = QOS_SEND_ALWAYS;
        SMLK_LOGI("ReportStatistics send msg to tsp size = [%d]", encoded.size());
        TspServiceApi::getInstance()->SendMsg(head, encoded.data(), encoded.size());
        // TODO:mark and store report info , to check and use sql stored message
    }
}

/******************************************************************************
 * NAME: StoreStatistics
 *
 * DESCRIPTION: cyclic sotre statistics information
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void CollectionService::StoreStatistics()
{
    SMLK_LOGD("Store Statistics every 5 minutes");

    std::vector<SMLK_UINT8> encoded;
    encoded.reserve(MAX_ETHERNET_SEND_SIZE);

    for ( auto protocol : m_protocols ) {
        std::shared_ptr<MessageStatistics>  ptr = std::static_pointer_cast<MessageStatistics>(m_protocol_messages[protocol][MessageType::SL_STATISTICS]);

        if ( ptr->m_trip_start.Valid() && ptr->m_trip_stop.Valid() && (ptr->m_trip_stop.Val() > ptr->m_trip_start.Val()) ) {
            ptr->m_flags |= StatisticsMask::SL_TRIP_FINISHED;
        } else {
            ptr->m_flags &= ~StatisticsMask::SL_TRIP_FINISHED;
        }

        ptr->m_location.m_status                            = m_cached_signals.find(vehicle::SignalID::GNSSLocatedStatus)->second;
        ptr->m_location.m_latitude                          = m_cached_signals.find(vehicle::SignalID::GNSSLatitude)->second;
        ptr->m_location.m_longitude                         = m_cached_signals.find(vehicle::SignalID::GNSSLongitude)->second;
        ptr->m_location.m_altitude                          = m_cached_signals.find(vehicle::SignalID::GNSSAltitude)->second;
        ptr->m_location.m_heading                           = m_cached_signals.find(vehicle::SignalID::GNSSHeading)->second;
        ptr->m_location.m_speed                             = m_cached_signals.find(vehicle::SignalID::GNSSSpeed)->second.Val() * M_PER_S_TO_KM_PER_HOUR;
        ptr->m_location.m_timestamp                         = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        ptr->m_trip_start                                   = m_cached_signals.find(vehicle::SignalID::StatisticsTripStartTime)->second;
        ptr->m_trip_stop                                    = m_cached_signals.find(vehicle::SignalID::StatisticsTripEndTime)->second;
        ptr->m_gnss_statistics_distance                     = m_cached_signals.find(vehicle::SignalID::StatisticsGnssSpeedDistance)->second;
        ptr->m_tachograph_vehicle_speed_statistics_distance = m_cached_signals.find(vehicle::SignalID::StatisticsTachographVehicleSpeedDistance)->second;
        //ptr->m_tachograph_vehicle_speed_statistics_distance = m_cached_signals.find(vehicle::SignalID::CalculusDistance)->second;               // confirmed to use the calculus distance
        ptr->m_vehicle_speed_statistics_life_total_distance = m_cached_signals.find(vehicle::SignalID::StatisticsVehicleSpeedLifeTotalDistance)->second;
        ptr->m_vehicle_total_distance                       = m_cached_signals.find(vehicle::SignalID::TotalVehicleDistance)->second;
        ptr->m_vehicle_estimated_load                       = m_cached_signals.find(vehicle::SignalID::TrailerWeight)->second;
        ptr->m_trip_statistics_fuel_used                    = m_cached_signals.find(vehicle::SignalID::StatisticsTripFuelUsed)->second;
        ptr->m_total_statistics_fuel_used                   = m_cached_signals.find(vehicle::SignalID::CalculusFuelConsumption)->second;        // the source CAN signal is the same with calculus
        ptr->m_engine_total_fuel_used                       = m_cached_signals.find(vehicle::SignalID::EngineTotalFuelUsed)->second;
        ptr->m_engine_average_fuel_economy                  = m_cached_signals.find(vehicle::SignalID::StatisticsAverageFuelEconomy)->second;
        ptr->m_speeding_times                               = m_cached_signals.find(vehicle::SignalID::StatisticsSpeedingTimes)->second;
        ptr->m_speeding_distance                            = m_cached_signals.find(vehicle::SignalID::StatisticsSpeedingDistance)->second;
        ptr->m_speeding_fuel_used                           = m_cached_signals.find(vehicle::SignalID::StatisticsSpeedingFuelUsed)->second;
        ptr->m_brake_pedal_times                            = m_cached_signals.find(vehicle::SignalID::StatisticsBrakePedalBrakeTimes)->second;
        ptr->m_brake_pedal_brake_distance                   = m_cached_signals.find(vehicle::SignalID::StatisticsBrakePedalBrakeDistance)->second;
        ptr->m_brake_pedal_brake_duration                   = m_cached_signals.find(vehicle::SignalID::StatisticsBrakePedalBrakeDuration)->second;
        ptr->m_sharp_slowdown_times                         = m_cached_signals.find(vehicle::SignalID::StatisticsSharpShowDownTimes)->second;
        ptr->m_sharp_slowdown_duration                      = m_cached_signals.find(vehicle::SignalID::StatisticsSharpShowDownDuration)->second;
        ptr->m_sharp_slowdown_distance                      = m_cached_signals.find(vehicle::SignalID::StatisticsSharpShowDownDistance)->second;
        ptr->m_sharp_speedup_times                          = m_cached_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpTimes)->second;
        ptr->m_sharp_speedup_duration                       = m_cached_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpDuration)->second;
        ptr->m_sharp_speedup_distance                       = m_cached_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpDistance)->second;
        ptr->m_sharp_speedup_fuel_used                      = m_cached_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpFuelUsed)->second;
        ptr->m_sharp_turnning_times                         = m_cached_signals.find(vehicle::SignalID::StatisticsSharpTrunningTimes)->second;
        ptr->m_sharp_turnning_distance                      = m_cached_signals.find(vehicle::SignalID::StatisticsSharpTrunningDistance)->second;
        ptr->m_sleepy_drive_times                           = m_cached_signals.find(vehicle::SignalID::StatisticsSleepyDriveTimes)->second;

        encoded.clear();
        ptr->Encode(encoded);

        std::lock_guard<std::mutex> lk(m_sql_mutex);
        std::vector<std::tuple<SMLK_UINT32, std::vector<SMLK_UINT8>>> records;
        SMLK_RC rc = m_sqldb.QueryAllInfo(records);
        if (SMLK_RC::RC_OK != rc ) {
            SMLK_LOGE("sqldebug fail to query suppliment record, error=%s.\n", m_sqldb.LastError().c_str());
            return;
        } else {
            if (0 == (SMLK_UINT16)records.size()) {
                SMLK_LOGE("sqldebug sql size = 0");
            } else {
                SMLK_LOGD(" sqldebug read message size is %d", records.size());
                for (SMLK_UINT8 k = 0; k < records.size(); k++) {
                    SMLK_UINT32 time_id = std::get<0>(records[k]);
                    auto delete_rc = m_sqldb.Delete(std::get<0>(records[k]));
                    if ( SMLK_RC::RC_OK != delete_rc ) {
                        SMLK_LOGE(" sqldebug delete fail!!");
                        return;
                    }
                }
            }
        }
    // now only keep one sql data
    auto insert_rc = m_sqldb.Insert(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count(), encoded.data(), encoded.size());
    if ( SMLK_RC::RC_OK != insert_rc ) {
        SMLK_LOGE(" sqldebug insert fail!!");
        return;
    }

#ifdef SL_DEBUG
        SMLK_LOGD(" sqldebug insert time_id %d", std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        SMLK_LOGD(" sqldebug insert encode size is %d", encoded.size());
        rc = m_sqldb.QueryAllInfo(records);
        if (SMLK_RC::RC_OK != rc ) {
            SMLK_LOGE("sqldebug fail to query suppliment record, error=%s.\n", m_sqldb.LastError().c_str());
            return;
        } else {
            SMLK_LOGD(" sqldebug after insert size is %d", records.size());
        }
#endif
    }
}


/******************************************************************************
 * NAME: OnDayChanged
 *
 * DESCRIPTION: day changed event handler
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *     we should report the statistics data while ING ON at 00:00:00 of each day
 *****************************************************************************/
void CollectionService::OnDayChanged()
{
    SMLK_LOGI("OnDayChanged!!!");

    // reset the shared timer
    if (m_timer_shared) {
        m_timer_shared->Reset();
    }

    if ( !CheckWhetherTimeSync() ) {
        SMLK_LOGE("I'm not sure why I get here, there must be something wrong!!!");
        return;
    }

#ifdef SL_DEBUG
    struct tm   tm;
    auto now        = std::chrono::system_clock::now();
    auto ms         = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::time_t tt  = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

    localtime_r(&tt, &tm);

    SMLK_LOGD("%04d-%02d-%02d %02d:%02d:%02d [%03d]",
        tm.tm_year + 1900,
        tm.tm_mon + 1,
        tm.tm_mday,
        tm.tm_hour,
        tm.tm_min,
        tm.tm_sec,
        ms
    );
#endif

    // we only report the statistics data while ING on at 00:00:00 at each day
    if ( m_internal_flags & InternalFlags::SL_FLAG_IGNON ) {
        m_internal_flags |= InternalFlags::SL_FLAG_DAY_CHANGE;
        ReportStatistics();
        m_internal_flags &= ~InternalFlags::SL_FLAG_DAY_CHANGE;
    }
}

/******************************************************************************
 * NAME: HandleSampling1SCollectConfig
 *
 * DESCRIPTION: handle 1s config
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *
 *****************************************************************************/
void CollectionService::HandleSampling1SCollectConfig()
{
    SMLK_LOGI("[configDebug] HandleSampling1SCollectConfig");
    std::string collect_config_property(SYS_PRO_NAME_REALTIME_DATA_GATHER_CYCLE);
    std::string read_config_result_str;

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(collect_config_property, read_config_result_str);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGD("[configDebug]fail to get system property \"%s\", return code: %d", collect_config_property, static_cast<std::int32_t>(return_code));
        return;
    } else {
        SMLK_LOGD("[configDebug]collect_config_property read_config_result_str is [%s]!", read_config_result_str.c_str());
        if (read_config_result_str.size() < 1) {
            SMLK_LOGD(" read_config_result_str error!");
            return;
        } else {
            std::stringstream ss;
            SMLK_UINT32 collect_interval_ms;
            ss << read_config_result_str;
            ss >> collect_interval_ms;
            if ( (0 == collect_interval_ms % 100) && ( collect_interval_ms >= 100 && collect_interval_ms <= 60000) ) {
                if ( m_trigger_period_1s_ms != collect_interval_ms) {
                    m_trigger_period_1s_ms = (collect_interval_ms / 100) * 100;
                    m_0f3b_timer->Reset();
                    for ( auto protocol : m_protocols ) {
                        std::shared_ptr<MessageBucket> ptr = std::static_pointer_cast<MessageBucket>(m_protocol_messages[protocol][MessageType::SL_BUCKET]);
                        ptr->m_signals.clear();
                    }
                    m_bucket_num_1s = m_upload_period_1s_ms / m_trigger_period_1s_ms;
                    SMLK_LOGD("[configDebug] m_bucket_num_1s is [%d]", m_bucket_num_1s);
                    OnUpdateJsonConfig();
                    SMLK_LOGD("[configDebug]m_bucket_num_1s is [%d] and reset timer ", m_bucket_num_1s);
                    SMLK_LOGD("[configDebug]m_upload_period_1s_ms is [%d] and reset timer ", m_upload_period_1s_ms);

                    m_0f3b_timer->ReStart();
                } else {
                    SMLK_LOGD("[configDebug]m_trigger_period_1s_ms config is not change!");
                }
            } else {
                SMLK_LOGD("[configDebug]m_trigger_period_1s_ms config is invalid!");
            }

            ss.str("");
            ss.clear();
        }
    }
}

/******************************************************************************
 * NAME: HandleFinaceLockStatus
 *
 * DESCRIPTION: handle wakeup config
 * SYS_PRO_NAME_LCKCAR_TYPE_JTT808 -- 表示限制的内容(转速:1/扭矩:2/喷油量限制:3)
 * SYS_PRO_NAME_LCKCAR_VALUE_JTT808 -- 表示具体限制的值
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 * TSC1报文为mcu外发 mpu侧无法获取到 故从远控模块获取
 *****************************************************************************/
void CollectionService::HandleFinaceLockStatus()
{
    SMLK_LOGI("[configDebug]HandleFinaceLockStatus");
    std::string requested_torque_property(SYS_PRO_NAME_LCKCAR_VALUE_JTT808);
    std::string finace_type_property(SYS_PRO_NAME_LCKCAR_TYPE_JTT808);
    std::string finace_lock_state_property(SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS);// 4 检查扭矩 3 不检查
    std::string tsc1_valid_property(SYS_PRO_NAME_TSC1_VALID); // tsc vaild 1   invaild 0

    std::string read_config_result_str;
    SMLK_UINT32 requested_value = 0;
    SMLK_UINT32 finace_type_value = 0;
    SMLK_UINT32 tsc1_valid_value = 0;

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(tsc1_valid_property, read_config_result_str);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGD("[configDebug]fail to get system property \"%s\", return code: %d", tsc1_valid_property, static_cast<std::int32_t>(return_code));
        return;
    } else {
        SMLK_LOGD("[configDebug]tsc1_valid_property read_config_result_str is [%s]!", read_config_result_str.c_str());
        if (read_config_result_str.size() < 1) {
            SMLK_LOGD("[configDebug] read_config_result_str error!");
            return;
        } else {
            std::stringstream ss;
            ss << read_config_result_str;
            ss >> tsc1_valid_value;
            SMLK_LOGD("[configDebug] tsc1_valid_value is [%d] and return !", tsc1_valid_value);
            if (0 == tsc1_valid_value) {
                m_cached_signals.find(vehicle::SignalID::RequestTorqueLimit)->second.Invalid();
                m_cached_signals.find(vehicle::SignalID::OverrideControlMode)->second.Invalid();
                m_cached_signals.find(vehicle::SignalID::RequestTorqueHighResolution)->second.Invalid();
                m_cached_signals.find(vehicle::SignalID::RequestSpeedLimit)->second.Invalid();
                return;
            }
        }
    }

    auto property_rc = smartlink_sdk::SysProperty::GetInstance()->GetValue(finace_lock_state_property, read_config_result_str);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != property_rc ){
        SMLK_LOGE("SysProperty::GetInstance()->GetValue(name is %s) failed,errcode=%ld, set to off.\n", finace_lock_state_property.c_str(), static_cast<std::int32_t>(property_rc));
        i_lckcar_sts = STATUS_NOT_ENABLE;
    } else {
        std::stringstream ss;
        SMLK_UINT32 lock_state;
        ss << read_config_result_str;
        ss >> lock_state;
        SMLK_LOGD("[configDebug] lock_state is [%d]!", lock_state);
        i_lckcar_sts = lock_state;

    }

    if (i_lckcar_sts <= STATUS_NOT_ACTIVE) {
        SMLK_LOGD("[configDebug] loan is NOT_ACTIVE report all ff and return !");
        m_cached_signals.find(vehicle::SignalID::RequestTorqueLimit)->second.Invalid();
        m_cached_signals.find(vehicle::SignalID::OverrideControlMode)->second.Invalid();
        m_cached_signals.find(vehicle::SignalID::RequestTorqueHighResolution)->second.Invalid();
        m_cached_signals.find(vehicle::SignalID::RequestSpeedLimit)->second.Invalid();
        return;
    } else {
        if (STATUS_IS_ACTIVE == i_lckcar_sts) {
            // 报文最大值 限扭填充100
            SMLK_LOGD("[configDebug] tsc1 can signal max value!");
            m_cached_signals.find(vehicle::SignalID::OverrideControlMode)->second = 0;
            m_cached_signals.find(vehicle::SignalID::RequestTorqueLimit)->second = 100;
            m_cached_signals.find(vehicle::SignalID::RequestTorqueHighResolution)->second = 0; // 0xF0末四个bit
            m_cached_signals.find(vehicle::SignalID::RequestSpeedLimit)->second = 0;
        } else if (STATUS_IS_LOCKED == i_lckcar_sts) {
            // 区分锁扭矩还是锁转速
            return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(finace_type_property, read_config_result_str);
            if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
                SMLK_LOGD("[configDebug]fail to get system property \"%s\", return code: %d", finace_type_property, static_cast<std::int32_t>(return_code));
                return;
            } else {
                SMLK_LOGD("[configDebug]finace_type_property read_config_result_str is [%s]!", read_config_result_str.c_str());
                if (read_config_result_str.size() < 1) {
                    SMLK_LOGD("[configDebug] read_config_result_str error!");
                    return;
                } else {
                    std::stringstream ss;
                    ss << read_config_result_str;
                    ss >> finace_type_value;
                    SMLK_LOGD("[configDebug] finace_type_property is [%d]!", finace_type_value);
                }
            }
            // 需要上报平台设置的实际值 不需要填充实际销贷报文值 TAPD:1040166
            return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(requested_torque_property, read_config_result_str);
            if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
                SMLK_LOGD("[configDebug]fail to get system property \"%s\", return code: %d", requested_torque_property, static_cast<std::int32_t>(return_code));
                return;
            } else {
                SMLK_LOGD("[configDebug]requested_torque_property read_config_result_str is [%s]!", read_config_result_str.c_str());
                if (read_config_result_str.size() < 1) {
                    SMLK_LOGD("[configDebug] read_config_result_str error!");
                    return;
                } else {
                    std::stringstream ss;
                    ss << read_config_result_str;
                    ss >> requested_value;
                    SMLK_LOGD("[configDebug] requested_value is [%d]!", requested_value);
                    SMLK_LOGD("[configDebug] i_lckcar_sts is [%d]!", i_lckcar_sts);
                }
            }

            SMLK_LOGD("[configDebug] tsc1 should report actual value!");
            m_cached_signals.find(vehicle::SignalID::RequestTorqueHighResolution)->second = 0;// 0xF0末四个bit
            m_cached_signals.find(vehicle::SignalID::OverrideControlMode)->second = 3;
            if (100 == requested_value) {
                if ( JTT808_LCKCAR_TYPE_ROTATE == finace_type_value) {
                    m_cached_signals.find(vehicle::SignalID::RequestSpeedLimit)->second = 100;
                } else if (JTT808_LCKCAR_TYPE_TORQUE == finace_type_value) {
                    m_cached_signals.find(vehicle::SignalID::RequestTorqueLimit)->second = 100;
                }
            } else {
                if (requested_value < 100) {
                    if ( JTT808_LCKCAR_TYPE_ROTATE == finace_type_value) {
                        m_cached_signals.find(vehicle::SignalID::RequestTorqueLimit)->second = 0;
                        m_cached_signals.find(vehicle::SignalID::RequestSpeedLimit)->second = requested_value;
                    } else if (JTT808_LCKCAR_TYPE_TORQUE == finace_type_value) {
                        m_cached_signals.find(vehicle::SignalID::RequestTorqueLimit)->second = requested_value;
                        m_cached_signals.find(vehicle::SignalID::RequestSpeedLimit)->second = 0;
                    }
                } else if (requested_value > 100) {
                    m_cached_signals.find(vehicle::SignalID::RequestTorqueLimit)->second.Invalid();
                    m_cached_signals.find(vehicle::SignalID::OverrideControlMode)->second.Invalid();
                    m_cached_signals.find(vehicle::SignalID::RequestTorqueHighResolution)->second.Invalid();
                    m_cached_signals.find(vehicle::SignalID::RequestSpeedLimit)->second.Invalid();
                }
            }
        }
    }
}

/******************************************************************************
 * NAME: HandleWakeupInterval
 *
 * DESCRIPTION: handle wakeup config
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *
 *****************************************************************************/
void CollectionService::HandleWakeupInterval()
{
    SMLK_LOGI("[configDebug]HandleStatisticsConfig");
    std::string wakeup_config_property(SYS_PRO_NAME_SLEEP_REPORT_INTERVAL);
    std::string read_config_result_str;

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(wakeup_config_property, read_config_result_str);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGD("[configDebug]fail to get system property \"%s\", return code: %d", wakeup_config_property, static_cast<std::int32_t>(return_code));
        return;
    } else {
        SMLK_LOGD("[configDebug]collect_config_property read_config_result_str is [%s]!", read_config_result_str.c_str());
        if (read_config_result_str.size() < 1) {
            SMLK_LOGD("[configDebug] read_config_result_str error!");
            return;
        } else {
            std::stringstream ss;
            SMLK_UINT32 wakeup_interval_s;
            ss << read_config_result_str;
            ss >> wakeup_interval_s;
            SMLK_LOGD("[configDebug] wakeup_interval_s is [%d]!", wakeup_interval_s);

            if ( wakeup_interval_s >= 1800 && wakeup_interval_s <= 86400) {
                if ( m_wakeup_report_period != wakeup_interval_s ) {
                    SMLK_LOGD("[configDebug][wakeupdebug] last m_wakeup_report_period is [%d]s!", m_wakeup_report_period);
                    m_wakeup_report_period = wakeup_interval_s;
                    SMLK_LOGD("[configDebug][wakeupdebug] update m_wakeup_report_period is [%d]s!", m_wakeup_report_period);

                    OnUpdateJsonConfig();
                    SMLK_LOGD("[configDebug]wakeup_interval_s is [%d]", wakeup_interval_s);
                } else {
                    SMLK_LOGD("[configDebug]wakeup_interval config is not change!");
                }
            } else {
                SMLK_LOGD("[configDebug] wakeup_interval_s invalid config!");
            }
        }
    }
}

/******************************************************************************
 * NAME: HandleStatisticsConfig
 *
 * DESCRIPTION: handle Statistics config
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *
 *****************************************************************************/
void CollectionService::HandleStatisticsConfig()
{
    SMLK_LOGI("[configDebug]HandleStatisticsConfig");
    std::string statis_config_property(SYS_PRO_NAME_STATIS_INFO_UPLOAD_CYCLE);
    std::string read_config_result_str;

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(statis_config_property, read_config_result_str);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGD("[configDebug]fail to get system property \"%s\", return code: %d", statis_config_property, static_cast<std::int32_t>(return_code));
        return;
    } else {
        SMLK_LOGD("[configDebug]collect_config_property read_config_result_str is [%s]!", read_config_result_str.c_str());
        if (read_config_result_str.size() < 1) {
            SMLK_LOGD("[configDebug] read_config_result_str error!");
            return;
        } else {
            std::stringstream ss;
            SMLK_UINT16 statis_interval_min;
            ss << read_config_result_str;
            ss >> statis_interval_min;

            if ( statis_interval_min >= 5 && statis_interval_min <= 60) {

                if ( m_statistics_report_period != (statis_interval_min*60) ) {

                    m_statistics_report_period = (statis_interval_min*60);
                    OnUpdateJsonConfig();
                    SMLK_LOGD("[configDebug]m_statistics_report_period is [%d] and reset timer ", m_statistics_report_period);

                    if (m_timer_flags & TimerFlags::SL_FLAG_0F3C_TIMER_START) {
                        SMLK_LOGD(" [m_0f3c_timer] m_0f3c_timer restart!");
                        m_0f3c_timer->Reset();
                        m_0f3c_timer->ReStart();
                    }
                } else {
                    SMLK_LOGD("[configDebug]statis config is not change!");
                }
            } else {
                SMLK_LOGD("[configDebug] read_config_result_str invalid config!");
            }
        }
    }
}

/******************************************************************************
 * NAME: HandleSampling1SUploadConfig
 *
 * DESCRIPTION: handle 1s config
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *
 *****************************************************************************/
void CollectionService::HandleSampling1SUploadConfig()
{
    SMLK_LOGI("[configDebug]HandleSampling1SUploadConfig");
    // check property vehicle config
    std::string upload_config_property(SYS_PRO_NAME_REALTIME_DATA_UPLOAD_CYCLE);
    std::string read_config_result_str;

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(upload_config_property, read_config_result_str);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGD("[configDebug]fail to get system property \"%s\", return code: %d", upload_config_property, static_cast<std::int32_t>(return_code));
        return;
    } else {
        SMLK_LOGD("[configDebug]collect_config_property read_config_result_str is [%s]!", read_config_result_str.c_str());
        if (read_config_result_str.size() < 1) {
            SMLK_LOGD("[configDebug] read_config_result_str error!");
            return;
        } else {
            std::stringstream ss;
            SMLK_UINT32 upload_interval_s;
            ss << read_config_result_str;
            ss >> upload_interval_s;

            SMLK_LOGD("[configDebug] m_upload_period_1s_ms is [%d] upload_interval_s is [%d]", m_upload_period_1s_ms , upload_interval_s);

            if ( upload_interval_s >= 0 && upload_interval_s <= 60) {
                if (0 == upload_interval_s) {
                    //  0 means not upload
                    m_upload_period_1s_ms = 0;
                    m_0f3b_timer->Reset();
                    OnUpdateJsonConfig();
                    for ( auto protocol : m_protocols ) {
                        std::shared_ptr<MessageBucket> ptr = std::static_pointer_cast<MessageBucket>(m_protocol_messages[protocol][MessageType::SL_BUCKET]);
                        ptr->m_signals.clear();
                    }
                    SMLK_LOGD("[configDebug]property is 0!! Reset timer and not report");
                } else {
                    if ( m_upload_period_1s_ms != (upload_interval_s*1000) ) {
                        m_upload_period_1s_ms = upload_interval_s*1000;
                        SMLK_LOGD("[configDebug]m_upload_period_1s_ms is [%d] and reset timer ", m_upload_period_1s_ms);
                        m_0f3b_timer->Reset();
                        for ( auto protocol : m_protocols ) {
                            std::shared_ptr<MessageBucket> ptr = std::static_pointer_cast<MessageBucket>(m_protocol_messages[protocol][MessageType::SL_BUCKET]);
                            ptr->m_signals.clear();
                        }
                        m_bucket_num_1s = m_upload_period_1s_ms / m_trigger_period_1s_ms;
                        OnUpdateJsonConfig();

                        m_0f3b_timer->ReStart();
                    } else {
                        SMLK_LOGD("[configDebug]m_upload_period_1s_ms config not change!");
                    }
                }
            } else {
                SMLK_LOGE("[configDebug]m_upload_period_1s_ms change invalid!");
            }
            ss.str("");
            ss.clear();
        }
    }
}


/******************************************************************************
 * NAME: ReadEvEmissionConfig
 *
 * DESCRIPTION: read vehicle type for specific signal requirement
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *
 *****************************************************************************/
void CollectionService::ReadEvEmissionConfig()
{
    SMLK_LOGI("ReadEvEmissionConfig");
    // check property vehicle config
    std::string ev_type_config_property(SYS_PRO_NAME_DID_NEW_ENERGY_EMISSION_REMOTE_MONITOR_CONFIG);
    std::string read_config_result_str;

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(ev_type_config_property, read_config_result_str);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGD("fail to get system property \"%s\", return code: %d", ev_type_config_property.c_str(), static_cast<std::int32_t>(return_code));
        return;
    } else {
        SMLK_LOGD("ev_type_config_property read_config_result_str is [%s]!", read_config_result_str.c_str());
        if (read_config_result_str.size() < 1) {
            SMLK_LOGD(" read_config_result_str error!");
            return;
        }

        SMLK_UINT8 type = 0;
        type = std::stoi(read_config_result_str);

        m_cached_signals.find(vehicle::SignalID::EvWorkingStatus)->second = type;
    }
}

/******************************************************************************
 * NAME: ReadCarTypeConfig
 *
 * DESCRIPTION: read vehicle type for specific signal requirement
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *
 *****************************************************************************/
void CollectionService::ReadCarTypeConfig()
{
    SMLK_LOGI("ReadCarTypeConfig");
    // check property vehicle config
    std::string cartype_config_property(SYS_PRO_NAME_DID_VEHICLE_TYPE);
    std::string read_config_result_str;

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(cartype_config_property, read_config_result_str);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGD("fail to get system property \"%s\", return code: %d", cartype_config_property, static_cast<std::int32_t>(return_code));
        return;
    } else {
        SMLK_LOGD("cartypedebug read_config_result_str is [%s]!", read_config_result_str.c_str());
        if (read_config_result_str.size() < 1) {
            SMLK_LOGD(" read_config_result_str error!");
            return;
        }
    }
}

/******************************************************************************
 * NAME: ReadNetWorkTypeConfig
 *
 * DESCRIPTION: read vehicle type for check direct/indirect network control
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *
 *****************************************************************************/
void CollectionService::ReadNetWorkTypeConfig()
{
    SMLK_LOGI("[networkdebug] ReadNetWorkTypeConfig!");
    // check property vehicle config
    std::string network_config_property(SYS_PRO_NAME_DID_OEM_PARTNUMBER);
    std::string read_config_result_str;

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(network_config_property, read_config_result_str);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGD(" [networkdebug] fail to get system property \"%s\", return code: %d", network_config_property.c_str(), static_cast<std::int32_t>(return_code));
        return;
    } else {
        SMLK_LOGD("networkdebug property is %s read_config_result_str is [%s]!", network_config_property.c_str() , read_config_result_str.c_str());
        if (read_config_result_str.size() < 1) {
            SMLK_LOGD("[networkdebug] read_config_result_str error!");
            return;
        }
        if (INDIRECT_NETWORK == read_config_result_str) {
            if (0 != (m_internal_flags & InternalFlags::SL_FLAG_DIRECT_CONTROL) ) {
                m_internal_flags &= ~InternalFlags::SL_FLAG_DIRECT_CONTROL;
                SMLK_LOGD("[networkdebug] set indirect control");
            }
        } else {
            if (DIRECT_NETWORK_1 == read_config_result_str
                || DIRECT_NETWORK_2 == read_config_result_str
                || DIRECT_NETWORK_3 == read_config_result_str) {
                SMLK_LOGD("[networkdebug] set direct control");
                m_internal_flags |= InternalFlags::SL_FLAG_DIRECT_CONTROL;
            } else {
                SMLK_LOGE("[networkdebug] RECEIVE INVALID SN!!!!!!!!!!!");
                m_internal_flags |= InternalFlags::SL_FLAG_DIRECT_CONTROL;
            }
        }
    }
}

/******************************************************************************
 * NAME: HandleGbCheckVinResult
 *
 * DESCRIPTION: Handle Gb CheckVin Result from gb6 set property
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *     gb6 will notify daq by set property
 *****************************************************************************/
void CollectionService::HandleGbCheckVinResult()
{
    SMLK_LOGI(" [VinDebug] HandleGbCheckVinResult!");
    try {
        SMLK_UINT8   default_result = 0;
        SMLK_UINT8   current_check_vin_result = 0;
        std::string   gb_check_vin_result;
        std::string   str_default("0");
        std::string   property(SYS_PRO_NAME_GB_CHECK_VIN_FOR_DAQ);
#if 0
        // direct set value to default when init and wait gb6 change 0->2 notify
        if (!(m_flags&IService::Running)) {
            auto return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, str_default);
            if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
                SMLK_LOGE("[VinDebug]fail to set system property \"%s\", return code: %d", property, static_cast<std::int32_t>(return_code));
                return;
            } else {
                SMLK_LOGI("set system property default value [%s]", std::to_string(default_result).c_str());
            }
        }
#endif
        auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, gb_check_vin_result);
        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
            SMLK_LOGE("[VinDebug]fail to get system property \"%s\", return code: %d", property, static_cast<std::int32_t>(return_code));
            return;
        } else {
            current_check_vin_result = std::stoi(gb_check_vin_result);
        }
        if (current_check_vin_result != default_result) {
            SMLK_LOGD("[VinDebug]current_check_vin_result %d ", current_check_vin_result);
            if (GB_READ_VIN_FAIL == current_check_vin_result || GB_CHECK_VIN_INVALID == current_check_vin_result) {
                Post(MsgHead(M_SL_MSGID_DRIVE_EVENT, M_SL_EVID_GB_CHECK_VIN_ERROR, 0, 0, 0, 0), nullptr, 0);
                auto set_result = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, str_default);
                if (smartlink_sdk::RtnCode::E_SUCCESS != set_result ) {
                    SMLK_LOGE("[VinDebug]fail to get system property \"%s\", set_result code: %d", property, static_cast<std::int32_t>(set_result));
                } else {
                    SMLK_LOGD("[VinDebug]Set property to 0 after receive vin error");
                }
            } else {
                SMLK_LOGI("[VinDebug]current gb check vin result is [%d]", current_check_vin_result);
            }
        } else {
            SMLK_LOGI("[VinDebug]current_check_vin_result is default_result!");
        }
    } catch(const std::exception &e) {
        std::cerr << e.what() <<std::endl;
        return;
    }
}

/******************************************************************************
 * NAME: CheckWireConnectStatus
 *
 * DESCRIPTION: check 4G and gnss wire connectd status to avoid mcu not nptify
 *              case: unplug wire while power off and then power on
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
*
 *****************************************************************************/
void CollectionService::CheckWireConnectStatus(SMLK_UINT8 nCheckItem)
{
    SMLK_LOGD("[GNSS]ON CheckWireConnectStatus , checkItem:%d",nCheckItem);

    // check 4G
    if (nCheckItem & GpsAndTelStatusCheck::TEL_NEED_CHECK)
    {
        smartlink_sdk::ModemAntennaInfo master_info;
        master_info.type = smartlink_sdk::ModemAntennaType::E_MODEM_ANTENNA_TYPE_MASTER;
        auto master_return_code  = smartlink_sdk::Telephony::GetInstance()->GetModemAntennaInfo(master_info);
        if (smartlink_sdk::RtnCode::E_SUCCESS != master_return_code) {
            SMLK_LOGE("[Tel] fail to get 4G antenna state, master_return_code = %d ", static_cast<std::int32_t>(master_return_code));
        } else {
            if (smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_ABSENT == master_info.state) {
                    SMLK_LOGD("[Tel] get 4G master antenna break");
                    SETBIT(m_dismantle_tbox_status, DismateTboxMask::SL_TEL_MAIN_ANTENNA_ABSENT);
            } else if (smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_PRESENT == master_info.state) {
                    SMLK_LOGD("[Tel] get 4G master antenna present");
                    CLRBIT(m_dismantle_tbox_status, DismateTboxMask::SL_TEL_MAIN_ANTENNA_ABSENT);
            }

        #if 0   // enable it when hardware support slave antenna
            if (smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_ABSENT == slave_info.state) {
                    SMLK_LOGD("[Tel] get 4G slave antenna break");
                    SETBIT(m_dismantle_tbox_status, DismateTboxMask::SL_TEL_SLAVE_ANTENNA_ABSENT);
            } else if (smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_PRESENT == slave_info.state) {
                    SMLK_LOGD("[Tel]get 4G slave antenna present");
                    CLRBIT(m_dismantle_tbox_status, DismateTboxMask::SL_TEL_SLAVE_ANTENNA_ABSENT);
            }
        #endif
        }
    }

    // check gnss
    if (nCheckItem & GpsAndTelStatusCheck::GPG_NEED_CHECK)
    {
        smartlink_sdk::GnssAntennaState state;
        auto return_code = smartlink_sdk::Location::GetInstance()->GetGnssAntennaState(state);
        if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
            SMLK_LOGE("[GNSS] fail to get GNSS antenna state, return_code = %d", static_cast<std::int32_t>(return_code));
        } else {
            auto &signal    = m_cached_signals.find(vehicle::SignalID::AlarmFlags)->second;
            if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_PRESENT == state ) {
                SMLK_LOGI("[GNSS]antenna state changed to normal");
                CLRBIT(m_dismantle_tbox_status, DismateTboxMask::SL_GNSS_ANTENNA_ABSENT);
                signal.AssignEncoded(signal.Encoded() & ~(AlarmMask::SL_GNSS_ANTENNA | AlarmMask::SL_GNSS_SHORT | AlarmMask::SL_GNSS_FAULT));
            } else if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_ABSENT == state ) {
                SMLK_LOGE("[GNSS] antenna state changed to absent");
                SETBIT(m_dismantle_tbox_status, DismateTboxMask::SL_GNSS_ANTENNA_ABSENT);
                signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_GNSS_ANTENNA);
            } else if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_SHORT_CIRCUIT == state ) {
                SMLK_LOGE("[GNSS]antenna state changed to short");
                signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_GNSS_SHORT);
            } else if ( smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_INVALID == state ) {
                SMLK_LOGE("[GNSS]antenna state changed to invalid");
                signal.AssignEncoded(signal.Encoded() | AlarmMask::SL_GNSS_FAULT);
            } else {
                SMLK_LOGE("[GNSS]unsupported antenna state: 0x%02X", static_cast<SMLK_UINT8>(state));
            }
        }
    }
}

SMLK_UINT8 CollectionService::ClearCanDataType()
{
    SMLK_UINT8 ret = INVALID_TYPE;
    if (m_internal_flags&InternalFlags::SL_FLAG_DIRECT_CONTROL
        && (0 == m_internal_flags&InternalFlags::SL_FLAG_IGNON)) {
        ret = DIRECT_TYPE;
    } else {
        if (0 == (m_internal_flags&InternalFlags::SL_FLAG_IGNON) ) {
            if (0 == (m_internal_flags&InternalFlags::SL_FLAG_SET_SIGNAL_INVALID) ) {
                ret = INDIRECT_TYPE;
            }
        }
    }
    return ret;
}


#ifdef SL_SIMULATOR
void CollectionService::GenerateRandomValues()
{
    static const std::unordered_map<vehicle::SignalID, double> reasonable = {
        { vehicle::SignalID::GNSSLatitude,          36.12345678  },
        { vehicle::SignalID::GNSSLongitude,         140.987654321 },
        { vehicle::SignalID::GNSSAltitude,          20.235678  },
        { vehicle::SignalID::GNSSSpeed,             100.234 },
        { vehicle::SignalID::RSSI,                  85.6},
        { vehicle::SignalID::GNSSSatelliteUsed,     8},
        { vehicle::SignalID::GNSSSatelliteVisible,  12},

        // E6
        { vehicle::SignalID::TachographVehicleSpeed,            85.6},
        { vehicle::SignalID::DriverDemandEnginePercentTorque,   1125},
        { vehicle::SignalID::ActualEnginePercentTorque,         30},
        { vehicle::SignalID::EngineTorqueMode,                  4},
        { vehicle::SignalID::EngineOilPressure,                 1000},
        { vehicle::SignalID::EngineSpeed,                       1500},
        { vehicle::SignalID::EngineCoolantTemperature,          110},
        { vehicle::SignalID::CabInteriorTemperature,            210},
        { vehicle::SignalID::AccPedalTravel,                    20.3},
        { vehicle::SignalID::EngineTotalFuelUsed,               1314520.234},
        { vehicle::SignalID::ParkingBrakeSwitch,                0x01},
        { vehicle::SignalID::WheelSpeed,                        101 },
        { vehicle::SignalID::ClutchSwitch,                      0x00},
        { vehicle::SignalID::BrakeSwitch,                       0x02},
        { vehicle::SignalID::CatalystTankLevel,                 1010},
        { vehicle::SignalID::EngineTotalHoursOfOperation,       76542.234},
        { vehicle::SignalID::EngineTotalRevolutions,            98767668.45},
        { vehicle::SignalID::EngineFuelRate,                    40.7},
        { vehicle::SignalID::EngineInstantaneousFuelEconomy,    37.6},
        { vehicle::SignalID::EngineAverageFuelEconomy,          36.3},
        { vehicle::SignalID::AftertreatmentDieselParticulateFilterDifferentialPressure, 64255},
        { vehicle::SignalID::EngineIntakeManifoldPressure,      600},
        { vehicle::SignalID::EngineIntakeManifoldTemperature,   97},
        { vehicle::SignalID::EngineAirIntakePressure,           510},
        { vehicle::SignalID::BarometricPressure,                1.3},
        { vehicle::SignalID::AmbientAirTemperature,             15.2},
        { vehicle::SignalID::EngineAirIntakeTemperature,        2101},
        { vehicle::SignalID::EngineWaitToStartLamp,             0x00},
        { vehicle::SignalID::WaterInFuelIndicator,              0x00},
        { vehicle::SignalID::TransmissionSelectedGear,          75},
        { vehicle::SignalID::TransmissionActualGearRatio,       35.123},
        { vehicle::SignalID::TransmissionCurrentGear,           65},
        { vehicle::SignalID::FuelLevel,                         1001},
        { vehicle::SignalID::TripDistance,                      367.3},
        { vehicle::SignalID::TotalVehicleDistance,              57391.2468},
        { vehicle::SignalID::CalculusDistance,                  57388.987},
        { vehicle::SignalID::CalculusFuelConsumption,           3579128.321},
        { vehicle::SignalID::ActuralCatalystSpewed,             22110812.15},
        { vehicle::SignalID::TotalCatalystSpewed,               4111081215},
        { vehicle::SignalID::ACCStatus,                         1},


        { vehicle::SignalID::TireAirLeakageRate,                0},
        { vehicle::SignalID::TireLocation2,                     0},
        { vehicle::SignalID::TireExtendedPressureRange,         0},
        { vehicle::SignalID::TirePressureRequired,              0},
        { vehicle::SignalID::AirConditionerModeStatus,          0},
        { vehicle::SignalID::CabTemperature,                    210},

        { vehicle::SignalID::RelativeLevelFrontAxleLeft,        0x1357},
        { vehicle::SignalID::RelativeLevelFrontAxleRight,       0x2468},
        { vehicle::SignalID::RelativeLevelRearAxleLeft,         0x9BDF},
        { vehicle::SignalID::RelativeLevelRearAxleRight,        0xACE0},

        { vehicle::SignalID::BellowPressureFrontAxleLeft,       0x1234},
        { vehicle::SignalID::BellowPressureFrontAxleRight,      0x5678},
        { vehicle::SignalID::BellowPressureRearAxleLeft,        0x9ABC},
        { vehicle::SignalID::BellowPressureRearAxleRight,       0xDEF0},
        { vehicle::SignalID::NominalLevelRearAxle,                  0xFFFF},
        { vehicle::SignalID::NominalLevelFrontAxle,                 0xFFFF},
        { vehicle::SignalID::AboveNominalLevelRearAxle,             0xFF},
        { vehicle::SignalID::AboveNominalLevelFrontAxle,            0xFF},
        { vehicle::SignalID::BelowNominalLevelFrontAxle,            0xFF},
        { vehicle::SignalID::LiftingControlModeFrontAxle,           0xFF},
        { vehicle::SignalID::LoweringControlModeFrontAxle,          0xFF},
        { vehicle::SignalID::LevelControlMode,                      0xFF},
        { vehicle::SignalID::LiftAxle1Position,                     0xFF},
        { vehicle::SignalID::BelowNominalLevelRearAxle,             0xFF},
        { vehicle::SignalID::LoweringControlModeRearAxle,           0xFF},
        { vehicle::SignalID::LiftingControlModeRearAxle,            0xFF},
        { vehicle::SignalID::LiftAxle2Position,                     0xFF},
        { vehicle::SignalID::RearAxleInBumperRange,                 0xFF},
        { vehicle::SignalID::FrontAxleInBumperRange,                0xFF},
        { vehicle::SignalID::SuspensionRemoteControl1,              0xFF},
        { vehicle::SignalID::SuspensionControlRefusalInformation,   0xFF},
    };

    std::random_device rd;
    std::mt19937 gen(rd());

    for ( auto &pair : m_cached_signals ) {
        std::uniform_real_distribution<> dis(pair.second.Min(), pair.second.Max());
        if ( pair.second.IsInteger() ) {
            pair.second = (std::int64_t)dis(gen);
        } else {
            pair.second = dis(gen);
        }
    }

    for ( auto &pair : reasonable ) {
        auto it = m_cached_signals.find(pair.first);
        if ( m_cached_signals.end() != it ) {
            it->second = pair.second;
        }
    }

    auto it_ts_ex = m_cached_signals.find(vehicle::SignalID::GNSSTimeStamp);
    if ( m_cached_signals.end() != it_ts_ex ) {
        it_ts_ex->second = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }
}

void CollectionService::SimulatorWorker()
{
    static const std::string    sun_path    = "/run/collection_service.sock";

    struct sockaddr_un  server_addr;

    std::memset(&server_addr, 0x00, sizeof(server_addr));

    server_addr.sun_family  = AF_UNIX;
    strncpy(server_addr.sun_path, sun_path.c_str(), sizeof(server_addr.sun_path) - 1);
    std::remove(sun_path.c_str());

redo:
    auto lfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if ( lfd < 0 ) {
        SMLK_LOGE("fail to create unix domain socket: %s", std::strerror(errno));
        return;
    }

    if ( bind(lfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0 ) {
        close(lfd);
        SMLK_LOGE("fail to bind unix domain socket: %s", std::strerror(errno));
        return;
    }

    if ( listen(lfd, 3) < 0 ) {
        close(lfd);
        SMLK_LOGE("fail to listen on unix domain socket: std::strerror(errno)");
        return;
    }

    static const struct timespec tv = {0, 200000000};

    auto maxfd = lfd + 1;

    fd_set                      rsets;
    std::set<int>               clients;
    std::vector<int>            fds;
    std::uint8_t                buffer[1024];

    while ( !(m_flags & IService::Stopping) ) {
        FD_ZERO(&rsets);
        FD_SET(lfd, &rsets);

        for ( auto fd : clients ) {
            FD_SET(fd, &rsets);
        }

        auto nready = pselect(maxfd, &rsets, nullptr, nullptr, &tv, nullptr);
        if ( 0 == nready ) {
            continue;
        } else if ( 0 > nready ) {
            SMLK_LOGE("select return error: %s", std::strerror(errno));
            break;
        }

        if ( FD_ISSET(lfd, &rsets) ) {
            auto cfd = accept(lfd, nullptr, nullptr);
            if ( cfd < 0 ) {
                SMLK_LOGE("accept return error: %s", std::strerror(errno));
                break;
            }

            clients.insert(cfd);
            maxfd = std::max(maxfd, cfd + 1);
            continue;
        }

        fds.clear();
        for ( auto fd : clients ) {
            if ( FD_ISSET(fd, &rsets) ) {
                auto nbytes = recv(fd, buffer, sizeof(buffer), 0);
                if ( 0 >= nbytes ) {
                    fds.push_back(fd);
                } else {
                    OnSimulatorRecv(buffer, (std::size_t)nbytes);
                }
            }
        }

        for ( auto fd : fds ) {
            close(fd);
            clients.erase(fd);
        }

        if ( fds.size() ) {
            maxfd = lfd;
            for ( auto fd : clients ) {
                maxfd = std::max(maxfd, fd);
            }
            maxfd += 1;
        }
    }

    close(lfd);
    for ( auto fd : clients ) {
        close(fd);
    }
    std::remove(sun_path.c_str());

    if ( !(m_flags & IService::Stopping) ) {
        goto redo;
    }
    SMLK_LOGI("simulator worker exit");
}

void CollectionService::OnSimulatorRecv(const SMLK_UINT8 *data, std::size_t sz)
{
    static const std::set<SMLK_UINT8>   valid_ids = {
        M_SL_EVID_SHARP_SLOW_DOWN,
        M_SL_EVID_SHARP_SPEED_UP,
        M_SL_EVID_SHARP_TURN,
        M_SL_EVID_SPEEDING,
        M_SL_EVID_LONG_LONG_PARKING,
        M_SL_EVID_ACCIDENT,
        M_SL_EVID_TIREDNESS_DRIVING,
    };

    if ( sz < sizeof(IpcTspHead) + 2 ) {
        SMLK_LOGE("invalid data format, message too short!");
        return;
    }

    if ( '|' != data[0] || '#' != data[1] ) {
        SMLK_LOGE("invalid data format, message start flag incorrect");
        return;
    }

    const IpcTspHead *  p_head  = (const IpcTspHead *)&data[2];
    const SMLK_UINT8 *  p_cur   = (const SMLK_UINT8 *)(p_head+1);

    IpcTspHead  ipc = {};
    ipc.msg_id      = be16toh(p_head->msg_id);
    ipc.seq_id      = be16toh(p_head->seq_id);
    ipc.length      = be16toh(p_head->length);
    ipc.reserved_dw = be32toh(p_head->reserved_dw);

    sz -= 2 + sizeof(IpcTspHead);           // '|' && '#' && IpcTspHead
    if ( sz != (std::size_t)ipc.length ) {
        SMLK_LOGE("mismatched message length");
        return;
    }

    switch ( ipc.msg_id ) {
        case SimulatorEvent::SL_SM_EVENT_START:
            {
                SMLK_LOGI("SimulatorEvent::SL_SM_EVENT_START");
                if ( sz < sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8) ) {
                    SMLK_LOGE("invalid body size!");
                    return;
                }

                DriveEvent  event = {
                    .id     = p_cur[0],
                    .type   = p_cur[1],
                    .flag   = DriveEventFlag::SL_EVENT_START
                };

                if ( valid_ids.end() == valid_ids.find(event.id) ) {
                    SMLK_LOGE("invalid event id:    0x%02X", event.id);
                    return;
                }

                std::vector<SMLK_UINT8> data(p_cur, p_cur+sz);

                OnDriveEvent(event, data);
                break;
            }
        case SimulatorEvent::SL_SM_EVENT_STOP:
            {
                SMLK_LOGI("SimulatorEvent::SL_SM_EVENT_STOP");
                if ( sz < sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8) ) {
                    SMLK_LOGE("invalid body size!");
                    return;
                }

                DriveEvent  event = {
                    .id     = p_cur[0],
                    .type   = p_cur[1],
                    .flag   = DriveEventFlag::SL_EVENT_STOP
                };

                if ( valid_ids.end() == valid_ids.find(event.id) ) {
                    SMLK_LOGE("invalid event id:    0x%02X", event.id);
                    return;
                }

                std::vector<SMLK_UINT8> empty;

                OnDriveEvent(event, empty);
                break;
            }
        case SimulatorEvent::SL_SM_FAULT_OCCUR:
            {
                SMLK_LOGI("SimulatorEvent::SL_SM_FAULT_OCCUR");
                if ( sz < sizeof(FaultCodesT) ) {
                    SMLK_LOGE("invalid body size! sz = %lu", sz);
                    return;
                }

                const FaultCodesT   *p_faults   = (const FaultCodesT *)p_cur;
                decltype(sz)        expected    = ((decltype(sz))p_faults->count) * sizeof(p_faults->codes[0]) + sizeof(FaultCodesT);
                if ( sz != expected ) {
                    SMLK_LOGE("mismatched message body size, sz = %lu, expected = %lu", sz, expected);
                    return;
                }

                MsgHead     head = {};

                head.m_extra        = (ipc.reserved_dw & 0xFF000000) | ((SMLK_UINT32)FaultEventFalg::SL_EVENT_START << 16);

                OnFaultEvent(head, std::vector<SMLK_UINT8>(p_cur, p_cur + sz));

                break;
            }
        case SimulatorEvent::SL_SM_FAULT_CLEAR:
            {
                SMLK_LOGI("SimulatorEvent::SL_SM_FAULT_CLEAR");
                if ( sz < sizeof(FaultCodesT) ) {
                    SMLK_LOGE("invalid body size! sz = %lu", sz);
                    return;
                }

                const FaultCodesT   *p_faults   = (const FaultCodesT *)p_cur;
                decltype(sz)        expected    = ((decltype(sz))p_faults->count) * sizeof(p_faults->codes[0]) + sizeof(FaultCodesT);;
                if ( sz != expected ) {
                    SMLK_LOGE("mismatched message body size, sz = %lu, expected: %lu", sz, expected);
                    return;
                }

                MsgHead     head = {};

                head.m_extra        = (ipc.reserved_dw & 0xFF000000) | ((SMLK_UINT32)FaultEventFalg::SL_EVENT_STOP << 16);

                OnFaultEvent(head, std::vector<SMLK_UINT8>(p_cur, p_cur + sz));
                break;
            }
        default:
            SMLK_LOGE("unsupported simulator event: 0x%08X", ipc.msg_id);
            break;
    }
}
#endif

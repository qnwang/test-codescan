#pragma once
#include <tuple>
#include <unordered_map>
#include <smlk_types.h>
#include <smlk_signal.h>
#include "SignalFormat.h"
#include <vehicle_data_index_def.h>

#define SPN_GNSS_TimeStamp_S              (516129)             // 时间
#define SPN_GNSS_LATITUDE                 (516130)             // 纬度
#define SPN_GNSS_LONGITUDE                (516131)             // 经度
#define SPN_GNSS_SPEED                    (516132)             // 速度
#define SPN_GNSS_LOCATEDSTATUS            (516133)             // 定位状态
#define SPN_GNSS_ALTITUDE                 (516134)             // 高度
#define SPN_GNSSTimeStamp_ABS_MS          (516200)             // 毫秒数
namespace smartlink {
namespace vehicle {
enum HeaderDataID : SMLK_UINT32 {
                StartHere,
                GNSSTimeStamp = SPN_GNSS_TimeStamp_S,
                GNSSLatitude = SPN_GNSS_LATITUDE,
                GNSSLongitude = SPN_GNSS_LONGITUDE,
                GNSSSpeed = SPN_GNSS_SPEED,
                GNSSLocatedStatus = SPN_GNSS_LOCATEDSTATUS,
                GNSSAltitude = SPN_GNSS_ALTITUDE,
                GNSSHeading,
                GNSSSatelliteUsed,
                GNSSSatelliteVisible,
                GNSSLocatedStatusEx,
                GNSSTimeStampMs = SPN_GNSSTimeStamp_ABS_MS,
                StaDayTrip = VEHILCE_DATA_STA_DAY_TRIP_DISTANCE,
                StaTrip = VEHICLE_DATA_TOTAL_VEHICLE_DISTANCE,
                StaTotalECU = VEHICLE_DATA_TOTAL_ECU_DISTANCE,
                StaDayFuel = VEHILCE_DATA_STA_DAY_FUEL,
                StaTripFuel = VEHICLE_DATA_STA_CALC_TOTAL_FUEL,
                StaTotalFuel = VEHICLE_DATA_TOTAL_FUEL_USED,
};

/*
StaDayTrip    当日里程    0.1km
StaTrip       仪表里程    0.1km
StaTotalECU   ECU里程     0.1km
StaDayFuel    当日油耗    0.1L
StaTripFuel   仪表油耗    L
StaTotalFuel  ECU油耗     L
*/


static const std::unordered_map<vehicle::HeaderDataID, vehicle::SignalVariant>    g_vec_signals = {
    { StaDayTrip,                                                   SignalVariant(0,            16,  0.1,   0,   0,   65535)},
    { StaTrip,                                                      SignalVariant(245,          32,  0.1)    },
    { StaTotalECU,                                                  SignalVariant(1032,         32,  0.1)    },
    { StaDayFuel,                                                   SignalVariant(0,            16,  0.1)    },
    { StaTripFuel,                                                  SignalVariant(0,            32)          },
    { StaTotalFuel,                                                 SignalVariant(250,          32)          }
};

static const std::unordered_map<vehicle::HeaderDataID, vehicle::SignalVariant>    g_gnss_signals = {
    { GNSSTimeStamp,                                                SignalVariant(516129,       32)                                                     },
    { GNSSLatitude,                                                 SignalVariant(516130,       32, 0.000001,   0,          0,          90)             },
    { GNSSLongitude,                                                SignalVariant(516131,       32, 0.000001,   0,          0,          180)            },
    { GNSSSpeed,                                                    SignalVariant(516132,       16, 0.1,        0,          0,          150)            },
    { GNSSLocatedStatus,                                            SignalVariant(516133,       3)                                                      },
    { GNSSAltitude,                                                 SignalVariant(516134,       32, 1.0/256.0)                                          },
    { GNSSHeading,                                                  SignalVariant(0,            16, 1,          0,          0,          359)            },
    { GNSSSatelliteUsed,                                            SignalVariant(0,            8)                                                      },
    { GNSSSatelliteVisible,                                         SignalVariant(0,            8)                                                      },
    { GNSSLocatedStatusEx,                                          SignalVariant(516133,       16)                                                     }
};

static const std::unordered_map<SMLK_UINT32, SignalFormat>    g_header_signals = {
    { SPN_GNSS_TimeStamp_S,                                                   SignalFormat(516129, 0, 32, 0, 4294967296, 0,         1, ByteOrderType::BYTE_ORDER_BIG_ENDIAN, 4, 0) },
    { SPN_GNSS_LATITUDE,                                                      SignalFormat(516130, 0, 32, 0,         90, 0,  0.000001, ByteOrderType::BYTE_ORDER_BIG_ENDIAN, 4, 0) },
    { SPN_GNSS_LONGITUDE,                                                     SignalFormat(516131, 0, 32, 0,        180, 0,  0.000001, ByteOrderType::BYTE_ORDER_BIG_ENDIAN, 4, 0) },
    { SPN_GNSS_SPEED,                                                         SignalFormat(516132, 0, 32, 0,        150, 0,       0.1, ByteOrderType::BYTE_ORDER_BIG_ENDIAN, 4, 0) },
    { SPN_GNSS_LOCATEDSTATUS,                                                 SignalFormat(516133, 0,  3, 0,          8, 0,         1, ByteOrderType::BYTE_ORDER_BIG_ENDIAN, 4, 0) },
    { SPN_GNSS_ALTITUDE,                                                      SignalFormat(516134, 0, 32, 0,      65535, 0, 1.0/256.0, ByteOrderType::BYTE_ORDER_BIG_ENDIAN, 4, 0) },
    { SPN_GNSSTimeStamp_ABS_MS,                                               SignalFormat(516200, 0, 16, 0,       1000, 0,         1, ByteOrderType::BYTE_ORDER_BIG_ENDIAN, 2, 0) }
};

static const std::vector<SMLK_UINT32> g_header_signal_spn_vec = {
       SPN_GNSSTimeStamp_ABS_MS,
       SPN_GNSS_ALTITUDE,
       SPN_GNSS_LOCATEDSTATUS,
       SPN_GNSS_SPEED,
       SPN_GNSS_LONGITUDE,
       SPN_GNSS_LATITUDE,
       SPN_GNSS_TimeStamp_S
};

}

}

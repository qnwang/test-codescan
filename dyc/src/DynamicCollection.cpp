#include "DynamicCollection.h"
#include <tsp_service_api.h>
#include "dyc_event.h"
#include "callable_event.h"
#include "Poco/Net/HTTPClientSession.h"
#include "Poco/Net/HTTPRequest.h"
#include "Poco/Net/HTTPResponse.h"
#include "Poco/StreamCopier.h"
#include <istream>
#include <ostream>
#include <sstream>
#include "ConfigLoader.h"
#include "Poco/Net/HTTPStreamFactory.h"
#include "Poco/Net/FTPStreamFactory.h"
#include <memory>
#include "Poco/URIStreamOpener.h"
#include "DbcConfigFormat.h"
#include <stdio.h>
#include <curl/curl.h>
#include <zlib.h>
#include "ipc_head.h"
#include <algorithm>
#include <vehicle_data_api.h>
#include <smartlink_sdk_mcu.h>
#include <smartlink_sdk_location.h>
#include <location_manager.h>
#include "smlk_timer_new.h"
#include <sys/syscall.h>
#include "vec_signals.h"

#define  EXTENDED_TIRE_INFO_CAN_ID                                  0x18FC4233
#define  MASK_CAN_ID_HIGH_BIT                                       0x7FFFFFFF
#define  DM1_DATA                                                   1
#define CYCLE_TIMEOUT_COUNT          (10)
#define CYCLE_MAX_VALUE              (0xFFFFFF)
#define CAN_MESSAGE_NORMAL_LENTH     (8)  // byte lenth
#define MAX_FRAME_LIMIT              (1000) // avoid use too much memory size
#define DEFAULT_MSG_CYCLE            (200)
#define MD5_BYTE_LENGTH              (16)
#define MAX_COLLECT_INTERVAL_MS      (65530)
static  SMLK_UINT8  g_curlResponse;

#define PUSH_U4(v)  \
    data.push_back((SMLK_UINT8)(((v) >> 24) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v) >> 16) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v) >>  8) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v)      ) & 0xFF))

#define PUSH_U2(v)  \
    data.push_back((SMLK_UINT8)(((v) >>  8) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v)      ) & 0xFF))

#define PUSH_U1(v)   \
    data.push_back(v)

#define PUSH_AR(v)  \
    data.insert(data.end(), (v), (v) + sizeof(v))

#define FRAME_LIMIT 200
using namespace std;
using namespace smartlink;
using namespace std::chrono;

using Poco::Net::HTTPClientSession;
using Poco::Net::HTTPRequest;
using Poco::Net::HTTPResponse;
using Poco::Net::HTTPMessage;
using Poco::StreamCopier;
using Poco::Path;
using Poco::URI;
using Poco::Exception;
using Poco::Net::HTTPStreamFactory;
using Poco::Net::FTPStreamFactory;
using Poco::URIStreamOpener;

using namespace smartlink;
    static const std::size_t    SL_COMPRESS_BUFFER_SZ    = 16384;                // compress block buffer size

    static const int            SL_QUEUE_GET_TIMEOUT_MS         = 200;                  // timeout millisecond to get item from queue
    static const SMLK_UINT32    SL_PERIOD_TRIGGER_POINT_MS      = 300;                  // period trigger ms point (0 ~ 999)
    static const SMLK_UINT32    SL_TIMER_1S                     = 0x00000001;           // 1s bucket report timer index
    static const SMLK_UINT8     DATATYPE_PHY_DATA               = 1;              // PHYCISICAL DATA
    static const SMLK_UINT8     DATATYPE_RAW_DATA               = 2;              // CAN RAW DATA

namespace TspState {
    static const SMLK_UINT8    SL_TSP_DISCONNECTED   = 0;
    static const SMLK_UINT8    SL_TSP_CONNECTED      = 1;
}

namespace InternalFlags {
    static const SMLK_UINT32    SL_FLAG_SYSTIME_SER      = 0x00000001;
    static const SMLK_UINT32    SL_FLAG_SYSTIME_SYN      = 0x00000002;
    static const SMLK_UINT32    SL_FLAG_COMPRESS         = 0x00000004;
    static const SMLK_UINT32    SL_FLAG_IGNON            = 0x00000008;
    static const SMLK_UINT32    SL_FLAG_PERIOD_WAKEUP    = 0x00000100;
    static const SMLK_UINT32    SL_FLAG_TEL_READY        = 0x00001000;
    static const SMLK_UINT32    SL_FLAG_CAN_NORMAL       = 0x00010000;
    static const SMLK_UINT32    SL_FLAG_DIRECT_CONTROL   = 0x00100000;
}

namespace Location {
    static const SMLK_UINT32    SL_SERVICE_READY        = 0x00000001;
    static const SMLK_UINT32    SL_SERVICE_EXIT         = 0x00000002;
    static const SMLK_UINT32    SL_LOCATION_CHANGED     = 0x00000004;
    static const SMLK_UINT32    SL_SATELLITE_CHANGED    = 0x00000008;
    static const SMLK_UINT32    SL_ANTENNA_CHANGED      = 0x00000010;
}

namespace MCU {
    static const SMLK_UINT32    SL_SERVICE_READY        = 0x00000001;
    static const SMLK_UINT32    SL_IGN_NOTIFY           = 0x00000002;
    static const SMLK_UINT32    SL_CAN_STATUS_NOTIFY    = 0x00000003;
}

namespace CanBusState {
    static const SMLK_UINT32    CAN_NORMAL                  = 0x000000000;
    static const SMLK_UINT32    CAN_SLEEP                   = 0x000000001;
    static const SMLK_UINT32    CAN_AUTHENTICATING          = 0x000000002;
    static const SMLK_UINT32    CAN_FAILED                  = 0x000000003;
}

namespace IgnState {
    static const SMLK_UINT32    SL_OFF                  = 0x000000000;
    static const SMLK_UINT32    SL_ON                   = 0x000000001;
}

namespace Protocol {
    static const SMLK_UINT8     SL_808  = static_cast<SMLK_UINT8>(ProtoclID::E_PROT_JTT808);
}

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

namespace Frequency {
    static const SMLK_UINT8     SL_DBC_A                = 0x01;
    static const SMLK_UINT8     SL_DBC_B                = 0x02;
    static const SMLK_UINT8     SL_DBC_C                = 0x03;
    static const SMLK_UINT8     SL_DBC_D                = 0x04;
    static const SMLK_UINT8     SL_DBC_E                = 0x05;
}

namespace Compress {
    static const SMLK_UINT8     SL_RC_SUCCESS           = 0x00;
    static const SMLK_UINT8     SL_RC_EFAILED           = 0xFF;
    static const SMLK_UINT8     SL_RC_EPARAM            = 0x01;
}

namespace InternalEvents {
    static const SMLK_UINT32 can_raw_data               = 0x00000001;
    static const SMLK_UINT32 dm1_raw_data               = 0x00000002;
    static const SMLK_UINT32 config_changed             = 0x00000003;
    static const SMLK_UINT32 ign_state_changed          = 0x00000004;
    static const SMLK_UINT32 timer_1s                   = 0x00000005;
    static const SMLK_UINT32 day_changed                = 0x00000006;
    static const SMLK_UINT32 correct_sta_data           = 0x00000007;
    static const SMLK_UINT32 drive_event_state_changed  = 0x00000008;
    static const SMLK_UINT32 dirve_event_time_out       = 0x00000009;
};


pid_t gettid() {
  return syscall(__NR_gettid);
}


DynamicCollectionService::DynamicCollectionService()
    : m_flags(IService::Uninitialized)
    , m_internal_flags(InternalFlags::SL_FLAG_IGNON)
    , m_sampling_frequency(0)
    , m_crompression(1)
    , m_current_frameLimit(10)
    , m_upload_data_type(1)
    , m_tsp_connect_state(0)
    , m_signal_day_vehicle_distance(0,            16,  0.1,   0,   0,   65535)
    , m_signal_total_vehicle_distance(245,          32,  0.1)
    , m_signal_ecu_total_distacne(1032,         32,  0.1)
    , m_signal_day_fuel_used(0,            16,  0.1)
    , m_signal_total_fuel_used(0,     32)
    , m_signal_ecu_total_fuel_used(0,     32)
{}

DynamicCollectionService::~DynamicCollectionService()
{
    SMLK_LOGI("service already initialized, do nothing");
    Stop();
}

SMLK_UINT32 DynamicCollectionService::Init()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Initialized ) {
        SMLK_LOGI("service already initialized, do nothing");
        return SL_SUCCESS;
    }

    m_protocols.push_back(Protocol::SL_808);

    m_internal_flags |= InternalFlags::SL_FLAG_COMPRESS; // 所有信号都需要压缩上传
    m_internal_flags |= InternalFlags::SL_FLAG_IGNON;

    {
        SMLK_LOGD("init TSP service");
        auto rc = TspServiceApi::getInstance()->Init(ModuleID::E_MODULE_dyc_service);
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

                        if (TspConnectState::E_CONNECTED == (TspConnectState)*state) {
                            m_tsp_connect_state = TspState::SL_TSP_CONNECTED;
                        } else if (TspConnectState::E_DISCONNECTED == (TspConnectState)*state) {
                            m_tsp_connect_state = TspState::SL_TSP_DISCONNECTED;
                        }
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
    }

    m_thread = std::thread(
        [this](){
            SMLK_LOGI("main work thread started");
            while ( !(m_flags & IService::Stopping) ) {
                m_events.get(SL_QUEUE_GET_TIMEOUT_MS)();
            }
            SMLK_LOGI("main work thread greacefully exit!!!");
        }
    );

    SMLK_LOGD("TSP service registered SUCCESS");
    auto rc = TspServiceApi::getInstance()->RegisterMsgCB(
        ProtoclID::E_PROT_JTT808,
        {M_SL_MSGID_TSP_DYC_REQ},
        [this](IN IpcTspHead &ipc, IN void *data, std::size_t sz) {
#ifdef SL_DEBUG
            // std::stringstream   ss;
            // for ( decltype(ipc.length) index = 0; index < sz; index++ ) {
            //     ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << ((char *)data)[index];
            // }
#endif
            SMLK_LOGD("#########IPC size :%d", (SMLK_UINT32)sz);
            Post(
                MsgHead(M_SL_MSGID_TSP_DYC_REQ, ipc.seq_id , 0, 0, ipc.length, 0),
                (const SMLK_UINT8 *)data,
                sz
            );
        }
    );
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register TSP service event callback");
        return SL_EFAILED;
    }

    for ( auto const &pair : vehicle::g_gnss_signals ) {
        m_gnss_signals.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(pair.first),
            std::forward_as_tuple(pair.second)
        );
    }

    {
        for ( auto const &pair : vehicle::g_vec_signals ) {
            m_vec_signals.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(pair.first),
                std::forward_as_tuple(pair.second)
            );
        }
        // register statisis data
        std::vector<SMLK_UINT32>    indexs = {
            VEHILCE_DATA_STA_DAY_TRIP_DISTANCE,  // 1029
            VEHICLE_DATA_TOTAL_VEHICLE_DISTANCE,
            VEHICLE_DATA_TOTAL_ECU_DISTANCE,
            VEHILCE_DATA_STA_DAY_FUEL,
            VEHICLE_DATA_STA_CALC_TOTAL_FUEL,
            VEHICLE_DATA_TOTAL_FUEL_USED
        };

        SMLK_LOGD("init vehicle data service");
        rc = VehicleDataApi::GetInstance()->Init(ModuleID::E_MODULE_dyc_service);
        if ( SMLK_RC::RC_OK != rc ) {
            SMLK_LOGE("fail to init vehicle data service API interface!!!");
            return SL_EFAILED;
        }

        SMLK_LOGD("register vehicle data changed callback");
        rc = VehicleDataApi::GetInstance()->RegistVehicleDataChangedCB(
            indexs,
            [this](VehicleData data){
    #ifdef SL_DEBUG
                // SMLK_LOGD("[VehicleDatqa] data changed notification: index[%u] valid[%s] value[%f]", data.index, (data.valid ? "YES" : "NO"), data.value);
    #endif
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
    }
    SMLK_LOGD("init MCU service");
    {
        auto return_code = smartlink_sdk::MCU::GetInstance()->Init(ModuleID::E_MODULE_dyc_service);
        if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
            SMLK_LOGE("fail to init MCU service API instance!!! return code: %d", static_cast<std::int32_t>(return_code));
            return SL_EFAILED;
        }

        // init mcu api
        std::vector<smartlink_sdk::McuEventId> events = {
            smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_DATA,
            smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE,
            smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_STATE
        };
        return_code = smartlink_sdk::MCU::GetInstance()->RegEventCB(events,
            [this](smartlink_sdk::McuEventId id, void* data, int len)
            {
                if (data == nullptr)
                {
                    SMLK_LOGW("Mcu E_MCU_EVENT_CAN_DATA can data ptr is null!!!");
                    return;
                }
                MsgHead head(M_SL_MSGID_MCU_STATE_NOTIFY, 0, 0, 0, 0, 0);

                switch(id)
                {
                    case smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_DATA:
                    {
                        // SMLK_LOGW("smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_DATA!!!!!");

                        smartlink_sdk::CanDataInfo *info = (smartlink_sdk::CanDataInfo *)data;
                        SMLK_UINT32 length = sizeof(CanFormatData::CanFrame)+info->can_dlc;
                        CanFormatData::CanFrame *frame = (CanFormatData::CanFrame *)malloc(length);

                        if (frame == nullptr)
                        {
                            SMLK_LOGE("**********malloc failed");
                            return;
                        }

                        frame->can_id = info->can_id & MASK_CAN_ID_HIGH_BIT;

                        frame->can_dlc = info->can_dlc;
                        // SMLK_LOGD("source CAN_ID[%x], mask can id[%x], lenth[%d]", info->can_id, frame->can_id, length);

                        memcpy(frame->data, info->data, info->can_dlc);

                        static std::uint32_t cnt = 0;
                        std::cout << "data lenth : " << frame->can_dlc << std::endl;
                        std::cout << "receive can data times : " << ++cnt << std::endl;
                        // dump can data
                        // Post(MsgHead(M_SL_MSGID_CAN_EVENT, 0, 0, 0, 0, 0), (SMLK_UINT8*)frame, length);

                        CanFrameAceept(frame);
                        free(frame);
                        break;
                    }
                    case smartlink_sdk::McuEventId::E_MCU_EVENT_J1939_ACK:
                    {
                        break;
                    }
                    case smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE:
                    {
                        head.m_seq  = MCU::SL_IGN_NOTIFY;
                        if ( sizeof(smartlink_sdk::IGNState) != (std::size_t)len ) {
                            SMLK_LOGE("[MCU] unexpected body length for MCU IGN state changed notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::IGNState));
                            return;
                        }

                        smartlink_sdk::IGNState *ign = reinterpret_cast<smartlink_sdk::IGNState *>(data);

                        if (ign->on)
                        {
                            head.m_extra = IgnState::SL_ON;
                        }
                        else
                        {
                            head.m_extra = IgnState::SL_OFF;
                        }
                        Post(head, nullptr, 0);

#if 0
                        if (ign->acc)
                        {
                            auto &signal = m_cached_signals.find(vehicle::SignalID::ACCStatus)->second;
                            signal =  1;
                        }
                        else
                        {
                            auto &signal = m_cached_signals.find(vehicle::SignalID::ACCStatus)->second;
                            signal =  0;
                        }
#endif
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
                        Post(head, nullptr, 0);
                    }
                        break;
                    default:
                        break;
                } // end for switch
            } // end for lambda
        ); // end for regist mcu callback

        if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
            SMLK_LOGE("fail to init register event callback to MCU service!!! return code: %d", static_cast<std::int32_t>(return_code));
            return SL_EFAILED;
        }
    }

    {
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
                            // SMLK_LOGD("[GNSS] antenna state changed, len = %d", len);
                        }
                        break;
                    case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED:
                        {
                            if ( sizeof(smartlink_sdk::LocationInfo) != (std::size_t)len ) {
                                SMLK_LOGE("[GNSS] unexpected body length for FIX INFO CHNAGED notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::LocationInfo));
                                return;
                            }

                            smartlink_sdk::LocationInfo     *inf = (smartlink_sdk::LocationInfo *)data;
                            LocatingInf     loc = {};

                            if ( inf->fix_valid ) {
                                loc.flags   |= LocatingFlags::SL_FIXED;
                            } else {
                                loc.flags   &= ~LocatingFlags::SL_FIXED;
                            }
                            loc.speed       = inf->speed;
                            loc.heading     = inf->heading;
                            loc.altitude    = inf->altitude;
                            loc.latitude    = inf->latitude;
                            loc.longitude   = inf->longitude;
                            loc.utc         = inf->utc;
#ifdef SL_DEBUG
#if 0
                        SMLK_LOGD("[GNSS] fix mode changed notification received");
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
    }

    // DBC init
    if (!FileTool::fileExist(DBC_FILE_PATH)) {
        std::string dyc_folder = DBC_FILE_PATH;
        if (!FileTool::createMutiDirectory(dyc_folder)) {
            SMLK_LOGE("no dbc path ");
        }
    }
    if (!FileTool::fileExist(DBC_FILE_CONFIG_PATH)) {
        std::string dyc_config_folder = DBC_FILE_CONFIG_PATH;
        if (!FileTool::createMutiDirectory(dyc_config_folder)) {
            SMLK_LOGE("no dbc config path ");
        }
    }

    // 初始化配置名称 可用配置自测试
    for (const auto &pair:g_dbc_default) {
        m_dbc_config.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(pair.first),
            std::forward_as_tuple(pair.second)
        );
    }

    // 查找当前所有DBC文件
    FileTool::getAllfiles(DBC_FILE_PATH, m_dbc_name_vec);
    SMLK_LOGD("m_dbc_name_vec size is %d!!", m_dbc_name_vec.size());
    if (m_dbc_name_vec.size() > 0) {
        std::vector<std::string>::iterator it;
        sort(m_dbc_name_vec.begin(), m_dbc_name_vec.end());
        for (auto &str: m_dbc_name_vec) {
            SMLK_LOGD("m_dbc_name_vec name is %s !!!", str.c_str());
            for (auto &pair:g_name_index_map) {
                if (pair.first == str) {
                    DBC_INDEX  dbc_index = pair.second;
                    LoadDbcConfigByIndex(dbc_index);
                    SMLK_LOGD("name is %s index is %d !!!", str.c_str() , dbc_index);

                    ConfigLoader::NewLoadConfigFromDBCFile(dbc_index ,\
                                                    &m_dbc_sample_signals_map, \
                                                    &g_message_decode_map);
                }
            }
        }
        SMLK_LOGD(" m_dbc_sample_signals_map size is %d", m_dbc_sample_signals_map.size());
    } else {
        SMLK_LOGI("no stored dbc file!!");
    }

    auto it = g_message_decode_map.begin();
    for ( ; it != g_message_decode_map.end(); it++) {
        SMLK_LOGE("g_message_decode_map every can_id current size is %d", it->second.second.size());
    }
    if (!g_message_decode_map.empty()) {
        InitVehicleDataPool();
        // init m_cached_signals for sample data
        SMLK_LOGD(" init cached signals !!");
#if 0
        for (const auto &pair : g_message_decode_map) {
            // 初始化需要采集的can数据到缓存 m_cached_signals
            auto vec = pair.second.second;
            if (!vec.empty()) {
                for (const auto &it: vec) {
                    SMLK_LOGD("cache sig[%d] ", it.GetIndex());
                    m_cached_signals.emplace(
                        std::piecewise_construct,
                        std::forward_as_tuple(it.GetIndex()),
                        std::forward_as_tuple(it)
                    );
                }
            }
        }
#endif
    } else {
        SMLK_LOGE("no sample signal to init data pool");
    }

    SMLK_LOGE("m_cached_signals size is %d", m_cached_signals.size());
    SMLK_LOGD("create timer interface!!!");
    m_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 100>>(
            [this](SMLK_UINT32 index) {
                MsgHead head(M_SL_MSGID_TIMER_NOTIFY, SL_TIMER_1S, 0, 0, 0, index);

                // SMLK_LOGD("LoopTimer tid is %d", (SMLK_UINT32)gettid());
                // SMLK_LOGD("call post every 100ms");
                Post(head, nullptr, 0);

                // SMLK_LOGD("current timer index %d", index);
            }
    );
    // Timer

    m_flags |= IService::Initialized;
    return SL_SUCCESS;

}

/******************************************************************************
 * NAME: UpdateDbcConfigByIndex
 *
 * DESCRIPTION: 根据dbc索引查找对应的周期配置文件，如果有文件，读取信息到缓存配置；如果没有，创建一个默认文件
 *
 * PARAMETERS:
 *
 * RETURN: success: 1 fail: 0
 *
 * NOTE: TODO security
 *****************************************************************************/
void DynamicCollectionService::LoadDbcConfigByIndex(IN DBC_INDEX &index)
{
    SMLK_LOGD("LoadDbcConfigByIndex");

    std::string config_name(DBC_FILE_CONFIG_PATH);
    dbc_config_struct dbcConfig;
    std::string  dbc_config_name;
    std::string dbc_name;
    SMLK_UINT8 data_type;
    auto it = g_txt_name_map.find(index);
    if ( g_txt_name_map.end() != it) {
        config_name.append(it->second);
        dbc_config_name = it->second;
    }
    auto it_name = g_index_name_map.find(index);
    if ( g_index_name_map.end() != it_name) {
        dbc_name = it_name->second;
    }
    SMLK_LOGD("[md5debug]dbc_name is %s", dbc_name.c_str());

    SMLK_LOGD("config path is %s", config_name.c_str());

    if (FileTool::fileExist(config_name)) {
        // firstly, we read all data to cache
        SMLK_LOGD("[md5debug]config path === %s ", config_name.c_str());

        // check md5sum
        std::string fileMd5("");
        std::vector<SMLK_UINT8> vec_fileMd5;
        std::string dbc_path(DBC_FILE_PATH);
        dbc_path.append(dbc_name);
        SMLK_LOGD("[md5debug] calculate dbc_path === %s ", dbc_path.c_str());

        FileTool::CalculateFileMd5Sum(dbc_path, vec_fileMd5);
        fileMd5.insert(fileMd5.begin(), vec_fileMd5.begin(), vec_fileMd5.end());
        vec_fileMd5.clear();
        SMLK_LOGD("[md5debug] fileMd5 === %s ", fileMd5.c_str());

        FILE * fp = NULL;
        fp = FileTool::fopenFile(config_name, "r+");
        if (fp) {
            FileTool::rewindFile(fp);
            FileTool::freadFromFile(&dbcConfig, sizeof(dbcConfig), 1, fp);
            SMLK_LOGD("read config path %s get******** uploadid is %d *****collect interval %d",
                        config_name.c_str(), dbcConfig.upload_id,  dbcConfig.collect_interval);
            {
                // std::lock_guard<std::mutex> config_lock(m_dbc_config_mutex);
                auto it  = m_dbc_config.find(index);
                if (m_dbc_config.end() != it) {
                    SMLK_LOGD("###init read config ##dbc name %s upload interval is %d ", dbc_config_name.c_str() , dbcConfig.upload_id);
                    SMLK_LOGD("###init read config ##dbc index %s collect interval is %d ", dbc_config_name.c_str() , dbcConfig.collect_interval);
                    SMLK_LOGD("###init read config ##dbc index %s uploadid  is %d ", dbc_config_name.c_str() , dbcConfig.upload_id);
                    SMLK_LOGD("###init read config ##dbc index %s data_type  is %d ", dbc_config_name.c_str() , dbcConfig.data_type);

                    it->second.setDbcName(dbc_name);
                    it->second.setUploadId(dbcConfig.upload_id);
                    it->second.setCollectionTnterval(dbcConfig.collect_interval);
                    it->second.setUploadInterval(dbcConfig.upload_interval);
                    // set md5
                    it->second.setFileMd5(fileMd5);
                    it->second.setUploadDataType(dbcConfig.data_type);
                }
            }
        }
        if (fp) {
            FileTool::fcloseFile(fp);
            fp = NULL;
        }
    } else {
        dbcConfig.collect_interval = 0;
        dbcConfig.upload_interval = 0;
        dbcConfig.upload_id = 0xE26F;
        dbcConfig.data_type = DATATYPE_PHY_DATA;
        // 有DBC文件但是没有周期配置文件 先创建文件
        FILE *fp = FileTool::fopenFile(config_name, "w+");
        if (NULL != fp) {
            FileTool::rewindFile(fp);
            FileTool::fwriteToFile(&dbcConfig, sizeof(dbc_config_struct), 1, fp);
            FileTool::fcloseFile(fp);
            fp = NULL;
            // 将此dbc文件缓存配置置为默认信息不上传 只配置id 待后台读取设置
            auto it = m_dbc_config.find(index);
            if (m_dbc_config.end() != it) {
                SMLK_LOGD("###init no config ##dbc name %s set upload interval is %d ", dbc_config_name.c_str() , dbcConfig.upload_id);
                SMLK_LOGD("###init no config ##dbc index %s set default collect interval is %d ", dbc_config_name.c_str() , dbcConfig.collect_interval);
                SMLK_LOGD("###init no config ##dbc index %s set default uploadid  is %d ", dbc_config_name.c_str() , dbcConfig.upload_id);
                SMLK_LOGD("###init no config ##dbc index %s set default data_type  is %d ", dbc_config_name.c_str() , dbcConfig.data_type);

                it->second.setDbcName(dbc_name);
                it->second.setUploadId(dbcConfig.upload_id);
                it->second.setCollectionTnterval(dbcConfig.collect_interval);
                it->second.setUploadInterval(dbcConfig.upload_interval);
                it->second.setUploadDataType(dbcConfig.data_type);
            }
        }
    }
    return;
}

/******************************************************************************
 * NAME: UpdateDbcConfigByIndex
 *
 * DESCRIPTION: 接收到新dbc配置 更新配置到文件
 *
 * PARAMETERS:
 *
 * RETURN: success: 1 fail: 0
 *
 * NOTE: TODO security
 *****************************************************************************/
void DynamicCollectionService::UpdateDbcConfigByIndex(IN DBC_INDEX &index, IN dbc_config_struct &info )
{
    SMLK_LOGD("UpdateDbcConfigByIndex get index %d", index);
    {
        auto it_dbc = m_dbc_config.find(index);
        if (m_dbc_config.end() != it_dbc) {
            it_dbc->second.setUploadId(info.upload_id);
            it_dbc->second.setCollectionTnterval(info.collect_interval);
            it_dbc->second.setUploadInterval(info.upload_interval);
            it_dbc->second.setUploadDataType(info.data_type);
            if (0 == info.collect_interval) {
                it_dbc->second.setDbcName("");
            }
        }
    }

    std::string config_name(DBC_FILE_CONFIG_PATH);
    // dbc_config_struct dbcConfig;
    auto it = g_txt_name_map.find(index);
    if ( g_txt_name_map.end() != it) {
        SMLK_LOGD("finded name is %s" , it->second.c_str());
        config_name.append(it->second);
    }
    SMLK_LOGD("config path is %s", config_name.c_str());
    if (!FileTool::fileExist(DBC_FILE_CONFIG_PATH)) {
        SMLK_LOGE("not find file, try to creat");
        std::string path = DBC_FILE_CONFIG_PATH;
        if ( !FileTool::createMutiDirectory(path)) {
            SMLK_LOGE("creat fail");
        }
    }
    FILE *fp = FileTool::fopenFile(config_name, "w+");
    if (NULL != fp) {
        FileTool::rewindFile(fp);
        FileTool::fwriteToFile(&info, sizeof(dbc_config_struct), 1, fp);
        FileTool::fcloseFile(fp);
        fp = NULL;
    } else {
        SMLK_LOGD("update config fail");
    }
    SMLK_LOGD("UpdateDbcConfigByIndex end");

    return;
}

/******************************************************************************
 * NAME: write_data
 *
 * DESCRIPTION: http回调返回内容 需要检查长度
 *
 * PARAMETERS:
 *
 * RETURN: success: 1 fail: 0
 *
 * NOTE: TODO 检查内容
 *****************************************************************************/
size_t DynamicCollectionService::write_data(void *ptr, size_t size, size_t nmemb, void *stream)
{
    SMLK_LOGD("write_data size is %ld" , SMLK_UINT64(size));
    // TODO:check size and data because may return error remind txt
    if ( 0 == SMLK_INT64(size)) {
        SMLK_LOGD("write back ");
        return size;
    }

    size_t written = fwrite(ptr, size, nmemb, (FILE *)stream);
    g_curlResponse = 1;
    return written;

}

/******************************************************************************
 * NAME: HttpGet
 *
 * DESCRIPTION: 根据下载地址和实际需要生成的名称 下载文件并在tbox路径下生成dbc文件
 *
 * PARAMETERS:
 *
 * RETURN: success: 1 fail: 0
 *
 * NOTE: cmd 0x02
 *****************************************************************************/
int DynamicCollectionService::HttpGet(std::string &address, std::string &true_name)
{
    SMLK_LOGD("HttpGet start download!");

    int ret = 0;
    g_curlResponse = false;
    std::string path(DBC_FILE_PATH);

    std::string file_name = path.append(true_name);
    CURL *curl_handle;
    SMLK_LOGD("file_name.c_str() is %s" , file_name.c_str());

    const char *pagefilename = file_name.c_str();
    FILE *pagefile;

    CURLcode res;

    /* init the curl session */
    curl_handle = curl_easy_init();

    /* set URL to get here */
    curl_easy_setopt(curl_handle, CURLOPT_URL, address.c_str() );
    // curl set ssl
    curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 20L);
    /* Switch on full protocol/debug output while testing */
    curl_easy_setopt(curl_handle, CURLOPT_VERBOSE, 1L);
    // curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 0L);
    /* disable progress meter, set to 0L to enable it */
    curl_easy_setopt(curl_handle, CURLOPT_NOPROGRESS, 1L);

    /* send all data to this function  */
    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, write_data);
#if 0
    int retcode = 0;
    curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE , &retcode);
    // 如果HTTP反应代码为200，表示网址有效
    SMLK_LOGD("curl_easy_getinfo retcode is %d" , retcode);
#endif
    /* open the file */
    pagefile = fopen(pagefilename, "wb+");
    if (pagefile) {
        /* write the page body to this file handle */
        curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, pagefile);

        /* get it! */
        res = curl_easy_perform(curl_handle);

        SMLK_LOGD("ret_Code: %d, errorMsg: %s", res, curl_easy_strerror(res));

        if (res != CURLE_OK)
        SMLK_LOGE("curl_easy_perform() failed: %s\n",
                curl_easy_strerror(res));
        else
            SMLK_LOGD("curl ok");

        /* close the header file */
        fflush(pagefile);
        fclose(pagefile);
    } else {
        SMLK_LOGE("open fail");
    }
    /* cleanup curl stuff */

    curl_easy_cleanup(curl_handle);
    if (g_curlResponse == 1) {
        ret = 1;
        g_curlResponse = 0;
    } else {
        ret = 0;
    }
    return ret;
}


SMLK_UINT32 DynamicCollectionService::Start()
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
    // 部分来源于vec需要获取一次
    for ( auto &pair :  m_vec_signals) {
        VehicleData data(pair.first, 0.0);

        auto rc = VehicleDataApi::GetInstance()->GetVehicleData(data);
        if ( SMLK_RC::RC_OK != rc ) {
            SMLK_LOGD("fail to get vehicle data for index:  %u", pair.first);
            continue;
        }

        pair.second = data.value;

        if ( !data.valid ) {
            SMLK_LOGD("init vehicle data not valid for index: %u, val: %f, min[%f], max[%f]", pair.first, data.value, pair.second.Min(), pair.second.Max());
            pair.second.Invalid();
            continue;
#ifdef SL_DEBUG
        } else {
            SMLK_LOGD("vehicle data valid for index:    %u, %f, SPN[%u], MIN[%f], MAX[%f]", pair.first, pair.second.Val(), pair.second.ID(), pair.second.Min(), pair.second.Max());
#endif
        }
    }
    // 当日里程 断电存储
    auto const &dayTrip = m_vec_signals.find(vehicle::HeaderDataID::StaDayTrip)->second;
    m_signal_day_vehicle_distance  = dayTrip.Val();
    if ( !m_signal_day_vehicle_distance.Valid() ) {
        m_signal_day_vehicle_distance.Invalid();
    }

    // 仪表里程 断电存储
    auto const &ecuDistance = m_vec_signals.find(vehicle::HeaderDataID::StaTotalECU)->second;
    m_signal_ecu_total_distacne  = ecuDistance.Val();
    if ( !m_signal_ecu_total_distacne.Valid() ) {
        m_signal_ecu_total_distacne.Invalid();
    }
    m_signal_ecu_total_distacne = m_vec_signals.find(vehicle::HeaderDataID::StaTotalECU)->second;

    // 20210926 : mcu do check can_id signal timeout
#if 0
    m_timer_long_miss = std::make_shared<LoopTimer<std::chrono::milliseconds, 1000>>(
        [this](SMLK_UINT32 index) {
            std::lock_guard<std::mutex> lk(g_vehicle_data_pool_mutex);
            for (auto &pair : g_vehicle_data_pool)
            {
                // since last revice time is longer than 10 period cyle, mark valid to false;
                if (duration_cast<std::chrono::milliseconds>(steady_clock::now() - pair.second.tp).count() >=  CYCLE_TIMEOUT_COUNT * 100)
                {
                    if (pair.second.data_valid)
                    {
                        SMLK_LOGW("index[%d] is time out!!!!", pair.first);
                        pair.second.data_valid = false;
                        CanFormatData::VehicleData data(pair.first, pair.second.physical_data);
                        data.valid = false;
                        // SMLK_LOGD("m_timer_long_miss tid is %d", (SMLK_UINT32)gettid());
                        Post(
                        MsgHead(M_SL_MSGID_SIGNAL_DATA_CHANGE, data.index, data.valid ? 0x01 : 0x00, 0, sizeof(data.value), 0),
                        reinterpret_cast<const SMLK_UINT8 *>(&data.value),
                        sizeof(data.value)
                        );
                    }
                }
            }// end for
        }// end lambda
    );// end make shared

    m_timer_long_miss->Start();
#endif

    auto now = std::chrono::system_clock::now();
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    // make sure we almost always trigger the 1s timer at the 500ms of each seconds
    if ( ms <= SL_PERIOD_TRIGGER_POINT_MS ) {
        std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(SL_PERIOD_TRIGGER_POINT_MS - ms));
    } else {
        // here since we may sleep up to 1s, so if we just got a term signal during this time, we may stop until 1 second later
        std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 + SL_PERIOD_TRIGGER_POINT_MS - ms));
    }

    SMLK_LOGD("start timer");
    m_timer->Start();

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

    m_flags |= IService::Running;
    SMLK_LOGD("mark service as running");
    return SL_SUCCESS;

}

bool DynamicCollectionService::IsRunning() const
{
    return  m_flags & IService::Running;
}


void DynamicCollectionService::Stop()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Running ) {

        if (m_timer_long_miss)
        {
            m_timer_long_miss->Stop();
            m_timer_long_miss.reset();
        }

        if ( m_thread.joinable() ) {
            m_thread.join();
        }

        if ( m_compress_thread.joinable() ) {
            m_compress_thread.join();
        }

        m_flags |= IService::Stopping;
        SMLK_LOGD("mark service as stoping");

        // TODO：close and stop thread/timer

        m_flags &= ~(IService::Running | IService::Stopping);

    }
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
 *  *****************************************************************************/

void DynamicCollectionService::Post(IN MsgHead &head, IN SMLK_UINT8 *data, IN std::size_t sz)
{
    // SMLK_LOGD("call post fun and now m_events size is %d" , m_events.size());
    m_events.emplace(
        head,
        data,
        sz,
        std::bind(&DynamicCollectionService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
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
void DynamicCollectionService::Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    // SMLK_LOGD("call post fun and now m_events size is %d" , m_events.size());
    m_events.emplace(
        head,
        data,
        std::bind(&DynamicCollectionService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
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
void DynamicCollectionService::Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data)
{
    m_events.emplace(
        head,
        std::move(data),
        std::bind(&DynamicCollectionService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: ResponseTspClearDycConfig
 *
 * DESCRIPTION: 回复平台清除成功
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE: cmd 0x05
 *****************************************************************************/
void DynamicCollectionService::ResponseTspClearDycConfig(IN MsgHead &head, IN std::vector<SMLK_UINT8> &tsp_data)
{
    SMLK_LOGD("ResponseTspReadDycInterval");
    std::vector<SMLK_UINT8> data;

    PUSH_U2(head.m_seq); // htobe16
    PUSH_U1(0x00); // version
    PUSH_U1(TSP_CLEAR_DYC_CONFIG);

    SMLK_LOGD("OnTspClearDycConfig");
    if (tsp_data.size() < 2) {
        return ;
    }

    SMLK_UINT8 file_num = tsp_data[2];
    SMLK_LOGD("file_num = %d", file_num);

    auto  tsp_clear_playload  = reinterpret_cast<const TspClearIntervalPayload *>(tsp_data.data());
    std::vector<std::string> str_vec;
    SMLK_LOGD("tsp_clear_playload->num = %d", tsp_clear_playload->num);

    if (tsp_clear_playload->num == 0) {
        if (data.size() > 2) {
            SMLK_LOGD("data size uncorrespond with num = 0");
            return;
        }
    } else if (tsp_clear_playload->num == TSP_CLEAR_ALL_CONFIG) {
        // direct response
        file_num = 250;
        PUSH_U1(file_num); // file num

        IpcTspHead      resp_head;
        std::memset(&resp_head, 0x00, sizeof(resp_head));
        resp_head.protocol  = Protocol::SL_808;
        resp_head.msg_id    = M_SL_MSGID_TSP_DYC_REP;
        resp_head.seq_id    = htobe16(head.m_seq);
        resp_head.qos       = QOS_SEND_ALWAYS;

        SMLK_LOGE("send data to tsp size is %d", data.size());

        TspServiceApi::getInstance()->SendMsg(resp_head, data.data(), data.size());
        return;
    } else if (tsp_clear_playload->num > TSP_CLEAR_CONFIG_MAX_NUM) {
        SMLK_LOGE("error num size is %d", tsp_clear_playload->num);
        return ;
    } else {
        PUSH_U1(file_num); // file num

        SMLK_LOGD("TspClear DycConfig num is %d", (SMLK_UINT32)(tsp_clear_playload->num) );
        SMLK_UINT32 current_byte_index = sizeof(PayloadHead);
        SMLK_UINT32 current_name_len = 0;

        std::size_t left_size    = tsp_data.size() - sizeof(PayloadHead);
        // get name_len and name
        while (left_size > 4 ) { // left > sizeof(TspClearIntervalPayload.TspClearInterval)
            SMLK_LOGD("current_byte_index = %d", current_byte_index);
            current_name_len = tsp_data[current_byte_index];

            PUSH_U1(current_name_len); // file num

            SMLK_LOGE("current_name_len = %d", current_name_len);
            // name
            ++current_byte_index;
            std::stringstream ss;
            for ( int index = current_byte_index; index < current_byte_index + current_name_len; index++ ) {
                // SMLK_LOGE("ss i %d = %02x", index , data[index]);
                PUSH_U1(tsp_data[index]);    // add to data
                ss << tsp_data[index];
            }
            current_byte_index += current_name_len;

            std::string tmp_dbc_name_str;
            ss >> tmp_dbc_name_str;
            ss.clear();
            ss.str(std::string());

            SMLK_LOGD("current receive name = %s", tmp_dbc_name_str.c_str());
            SMLK_UINT8 check_dbc_valid = 0; // default set fail
            for (auto &pair:m_dbc_config) {
                std::string cache_dbc_name = pair.second.getDbcName();
                SMLK_LOGD("cache_dbc_name name = %s", cache_dbc_name.c_str());
                if ( tmp_dbc_name_str == cache_dbc_name) {
                    SMLK_LOGD("find name [%s] success name", tmp_dbc_name_str.c_str());
                    check_dbc_valid = 1;
                    break;
                } else {
                    check_dbc_valid = 0;
                }
            }
            if (1 == check_dbc_valid) {
                SMLK_LOGD("push 0x01");
                PUSH_U1(0x01);    // have this dbc will clear success
            } else {
                SMLK_LOGD("push 0x02");
                PUSH_U1(0x02);    // dbc not exist
            }

            left_size = tsp_data.size() - current_byte_index;
        }
        // do execute clear dbc config
    }

    IpcTspHead      resp_head;
    std::memset(&resp_head, 0x00, sizeof(resp_head));
    resp_head.protocol  = Protocol::SL_808;
    resp_head.msg_id    = M_SL_MSGID_TSP_DYC_REP;
    // resp_head.seq_id    = htobe16(head.m_seq);
    resp_head.qos       = QOS_SEND_ALWAYS;

    SMLK_LOGE("send data to tsp size is %d", data.size());

    TspServiceApi::getInstance()->SendMsg(resp_head, data.data(), data.size());

}

/******************************************************************************
 * NAME: OnTspClearDycConfig
 *
 * DESCRIPTION: 执行平台清除指令
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE: cmd 0x05
 *****************************************************************************/
bool DynamicCollectionService::OnTspClearDycConfig(IN std::vector<SMLK_UINT8> &data)
{
    SMLK_BOOL ret = false;
    SMLK_LOGD("OnTspClearDycConfig");
    if (data.size() < 2) {
        return false;
    }
    auto  tsp_clear_playload  = reinterpret_cast<const TspClearIntervalPayload *>(data.data());
    std::vector<std::string> str_vec;

    if (tsp_clear_playload->num == 0) {
        if (data.size() > 2) {
            SMLK_LOGD("data size uncorrespond with num = 0");
            return false;
        }
    } else if (tsp_clear_playload->num == TSP_CLEAR_ALL_CONFIG) {
        std::vector<std::string> all_name_vec = { "a.dbc", "b.dbc", "c.dbc", "d.dbc", "e.dbc"};
        DoExecuteClearDbcConfig(all_name_vec);
        return true;
    } else if (tsp_clear_playload->num > TSP_CLEAR_CONFIG_MAX_NUM) {
        SMLK_LOGD("error num is %d", tsp_clear_playload->num);
        return false;
    } else {
        // 根据文件名找到index 删除缓存和文件
        SMLK_LOGD("TspClear DycConfig num is %d", (SMLK_UINT32)(tsp_clear_playload->num) );
        SMLK_UINT32 current_byte_index = sizeof(PayloadHead);
        SMLK_UINT32 current_name_len = 0;

        std::size_t left_size    = data.size() - sizeof(PayloadHead);
        // get name_len and name
        while (left_size > 4 ) { // left > sizeof(TspClearIntervalPayload.TspClearInterval)
            SMLK_LOGD("current_byte_index = %d", current_byte_index);
            current_name_len = data[current_byte_index];

            SMLK_LOGE("current_name_len = %d", current_name_len);
            // name
            ++current_byte_index;

            std::stringstream   ss;
            for ( int index = current_byte_index; index < current_byte_index + current_name_len; index++ ) {
                // SMLK_LOGE("ss i %d = %02x", index , data[index]);
                ss << (char)data[index];
            }
            current_byte_index += current_name_len;
            std::string temp_dbc_name;
            ss >> temp_dbc_name;
            ss.clear();
            ss.str(std::string());

            SMLK_LOGI("current_name is = %s", temp_dbc_name.c_str());

            std::string name_example("x.dbc");
            if (name_example.size() == temp_dbc_name.size()) {
                str_vec.emplace_back(temp_dbc_name);
            } else {
                SMLK_LOGE("invalid dbc name %s", temp_dbc_name.c_str());
            }

            left_size = data.size() - current_byte_index;
        }
        // do execute clear dbc config
        DoExecuteClearDbcConfig(str_vec);
        ret = true;
    }
    return ret;
}

/******************************************************************************
 * NAME: DoExecuteClearDbcConfig
 *
 * DESCRIPTION: 执行平台清除动作
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE: 清理缓存配置 删除dbc文件 删除配置文件
 *****************************************************************************/
bool DynamicCollectionService::DoExecuteClearDbcConfig(IN std::vector<string> &str_vec)
{
    SMLK_LOGI("DoExecuteClearDbcConfig");
    dbc_config_struct dbcInfo;
    std::string  dbc_name;
    auto iter_name_str = str_vec.cbegin();
    for ( ; str_vec.cend() != iter_name_str; ++iter_name_str) {
        for (auto const &pair:g_name_index_map) {
            if (pair.first == *iter_name_str) {
                // 清理 cache signal

                // 清理缓存配置
                SMLK_UINT32 dbc_index =  pair.second;
                dbcInfo.collect_interval = 0;
                dbcInfo.upload_id = 0;
                dbcInfo.upload_interval = 0;
                UpdateDbcConfigByIndex(dbc_index, dbcInfo);
                // 删除dbc文件
                std::string config_path(DBC_FILE_PATH);
                config_path.append(*iter_name_str);
                SMLK_LOGI("delete dbc path is %s", config_path.c_str());
                FileTool::deleteFile(config_path);
                // 删除配置文件
                auto it_config = g_txt_name_map.find(dbc_index);
                if ( g_txt_name_map.end() != it_config) {
                    std::string config_path(DBC_FILE_CONFIG_PATH);
                    config_path.append(it_config->second);
                    SMLK_LOGI("delete config path is %s", config_path.c_str());
                    FileTool::deleteFile(config_path);
                }
            }
        }
    }
    sync();
    return true;
}

/******************************************************************************
 * NAME: OnTspSetDycInterval
 *
 * DESCRIPTION: 设置上报间隔、周期和id
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE: cmd 0x01
 *****************************************************************************/
void DynamicCollectionService::OnTspSetDycInterval(IN MsgHead &head, IN std::vector<SMLK_UINT8> &tsp_data)
{
    SMLK_LOGD("OnTspSetDycInterval, data.size() = %d", tsp_data.size());
    // 判断 长度 3+data[2]*7
    // id(byte)+ num(byte) + e_id(word) + e_imterval(word)
    if(tsp_data.size() < 3) {
        SMLK_LOGE("body size too short");
        return;
    }
    SMLK_UINT8 file_num = tsp_data[2];
    SMLK_LOGD("file_num is%d",file_num);

    if ( (file_num * 7 + 3)!= tsp_data.size() ) {
        SMLK_LOGD("body size invalid");
    }

    std::vector<SMLK_UINT8> data;
    PUSH_U2(head.m_seq);
    PUSH_U1(TSP_SET_DYC_INTERVAL);
    PUSH_U1(file_num); // file num
    dbc_config_struct dbcInfo;

    std::size_t left_size = tsp_data.size() - sizeof(TspSetIntervalHead);  // size = 3
    SMLK_UINT32 byte_index = (SMLK_UINT32)sizeof(TspSetIntervalHead);
    while ( (SMLK_UINT16)left_size >=  7) { // hard code modify
        SMLK_UINT16 upload_id = (SMLK_UINT16)tsp_data[byte_index];
        upload_id<<= 8;
        ++byte_index;
        upload_id |= (SMLK_UINT16)tsp_data[byte_index];
        ++byte_index;

        SMLK_UINT16 collect_interval = (SMLK_UINT16)tsp_data[byte_index];
        collect_interval<<= 8;
        ++byte_index;
        collect_interval |= (SMLK_UINT16)tsp_data[byte_index];
        ++byte_index;
        if ( collect_interval > MAX_COLLECT_INTERVAL_MS) {
            collect_interval = MAX_COLLECT_INTERVAL_MS;
        }

        SMLK_UINT16 upload_interval = (SMLK_UINT16)tsp_data[byte_index];
        upload_interval<<= 8;
        ++byte_index;
        upload_interval |= (SMLK_UINT16)tsp_data[byte_index];
        ++byte_index;

        SMLK_UINT8 dataType = tsp_data[byte_index];
        SMLK_LOGD("============dataType   is %d", dataType);


        ++byte_index;
        left_size = tsp_data.size() - byte_index;

        dbcInfo.upload_id = upload_id;
        dbcInfo.collect_interval = collect_interval;
        dbcInfo.upload_interval = upload_interval;
        dbcInfo.data_type = dataType;
        SMLK_UINT8 update_result = 0x02; // fail
        for (auto &pair:m_dbc_config) {
            // 检查缓存中有没有判断为有效的指令id
            if (pair.second.getUploadId() == dbcInfo.upload_id ) {
               SMLK_LOGD( "update index upload is %d", pair.first);
                UpdateDbcConfigByIndex(pair.first, dbcInfo);
                update_result = 0x01;
            } else {
                // 是无效的指令id 回复下发失败
                update_result = 0x02;
            }
        }

        PUSH_U2(upload_id); // upload id
        PUSH_U2(update_result); // result success
        SMLK_LOGE(" byte_index = %d", byte_index);
        SMLK_LOGD( "tsp_set_interval_playload collect interval %d", collect_interval);
        SMLK_LOGD( "tsp_set_interval_playload upload_id %d", upload_id );
        SMLK_LOGD( "tsp_set_interval_playload upload_interval %d", upload_interval );
        --file_num;
        if (file_num == 0) {
            break;
        }
    }

#if 0
    auto tsp_set_interval_playload  = reinterpret_cast<const TspSetIntervalPayload *>(tsp_data.data());
    auto sub_load = &tsp_set_interval_playload->set_info[0];
    // 根据参数长度循环取指令包到结尾
    // 对每条参数组专有应答包
    while ( left >= sizeof(TspSetInterval) ) {
        SMLK_LOGE("left size: %u", (SMLK_UINT8)left);
        dbcInfo.upload_id = sub_load->upload_cmd_id;
        dbcInfo.collect_interval = sub_load->collect_interval;
        dbcInfo.upload_interval = sub_load->upload_interval;
        left -= sizeof(TspSetInterval);
        ++sub_load;
    }
#endif
    IpcTspHead      resp_head;
    std::memset(&resp_head, 0x00, sizeof(resp_head));
    resp_head.protocol  = Protocol::SL_808;
    resp_head.msg_id    = M_SL_MSGID_TSP_DYC_REP;
    // resp_head.seq_id    = htobe16(head.m_seq);
    resp_head.qos       = QOS_SEND_ALWAYS;
    SMLK_LOGD("TspSetInterval send size: %u to tsp ", (SMLK_UINT8)(data.size()));
    // new version no need response
    // TspServiceApi::getInstance()->SendMsg(resp_head, data.data(), data.size());
    return;
}


/******************************************************************************
 * NAME: ResponseTspReadDycInterval
 *
 * DESCRIPTION: 读取当前缓存中的有效配置上报tsp
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE: cmd 0x02
 *****************************************************************************/
void DynamicCollectionService::ResponseTspReadDycInterval(IN MsgHead &head)
{
    SMLK_LOGD("ResponseTspReadDycInterval");
    std::vector<SMLK_UINT8> data;

    PUSH_U2(head.m_seq);
    PUSH_U1(0x00); // version
    PUSH_U1(TSP_READ_DYC_INTERVAL);
    std::size_t num_index = data.size();
    PUSH_U1(0x00); // file num

    SMLK_UINT8 useful_num = 0;
    int i = 0;
    // std::lock_guard<std::mutex> config_lock(m_dbc_config_mutex);
    for (auto &pair: m_dbc_config) {
        SMLK_UINT32 collect_interval = pair.second.getCollectionTnterval();
        SMLK_UINT32 upload_interval = pair.second.getUploadInterval();
        SMLK_UINT32 upload_id = pair.second.getUploadId();
        SMLK_UINT8 data_type = pair.second.getUploadDataType();

        if ( 0 != collect_interval && 0 != upload_interval) {
            SMLK_LOGD("response useful id %x" , pair.second.getUploadId());
            PUSH_U2(upload_id);
            PUSH_U2(collect_interval);
            PUSH_U2(upload_interval);
            PUSH_U1(data_type);
            ++useful_num;
        }
    }
    // try to fix the num
    data[num_index]      = (SMLK_UINT8)(useful_num);
    SMLK_LOGD("size of DycReadIntervalRespBody is %d", sizeof(DycReadIntervalRespBody));

    IpcTspHead      resp_head;
    std::memset(&resp_head, 0x00, sizeof(resp_head));
    resp_head.protocol  = Protocol::SL_808;
    resp_head.msg_id    = M_SL_MSGID_TSP_DYC_REP;
    // resp_head.seq_id    = htobe16(head.m_seq);
    resp_head.qos       = QOS_SEND_ALWAYS;
    SMLK_LOGD("ResponseTspReadDycInterval  send tsp data size is %d", sizeof(data));

    TspServiceApi::getInstance()->SendMsg(resp_head, data.data() , data.size());
}

/******************************************************************************
 * NAME: OnTspSendDycDbc
 *
 * DESCRIPTION: 发送新的dbc文件配置
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 * 不能下载http返回size 0
 * TODO:可以下载不能解析，此时不能覆盖原有文件
 *****************************************************************************/
void DynamicCollectionService::OnTspSendDycDbc(IN MsgHead &head, IN std::vector<SMLK_UINT8> &tsp_data)
{
        SMLK_LOGE("OnTspSendDycDbc, tsp_data.size() = %d", tsp_data.size());
        SMLK_UINT32 dbc_index;
        SMLK_UINT8 version = 0;
        SMLK_UINT8 index = 0;
        SMLK_UINT8 ID = tsp_data[1];
        SMLK_UINT8 reserve = tsp_data[2];
        SMLK_UINT8 current_name_len = 0;
        // TODO: check data valid
        SMLK_UINT8 http_url_len = 0;

        SMLK_UINT8 config_count = tsp_data[3];
        SMLK_UINT32 byte_index = (SMLK_UINT32)sizeof(SendDbcRawDataHead);

        std::size_t left_size = tsp_data.size() - sizeof(SendDbcRawDataHead);
        std::string dbc_name;
        std::string http_url;
        std::vector<SMLK_UINT8> name;
        DbcConfigInfo DbcConfig;
        // response data

        std::vector<SMLK_UINT8> data;
        PUSH_U2(head.m_seq);
        PUSH_U1(TSP_VERSION);
        PUSH_U1(TSP_SEND_DYC_DBC);
        std::size_t num_index = data.size();
        PUSH_U1(config_count); // file num

        while ( left_size >  25) { // hard code modify
            SMLK_LOGE(" config_count = %d", config_count);
            SMLK_LOGE(" left_size = %d", left_size);
            name.clear();

            SMLK_LOGE(" start parse byte_index = %d", byte_index);
            current_name_len = tsp_data[byte_index];

            PUSH_U1(current_name_len); // response add namelen

            SMLK_LOGE("current_name_len = %d", current_name_len); // len = 5
            // name
            ++byte_index;
            SMLK_LOGE(" byte_index = %d", byte_index);

            std::stringstream   ss;
            for ( int index = byte_index; index < byte_index + current_name_len; index++ ) {
                // SMLK_LOGE("ss i %d = %02x", index , tsp_data[index]);
                ss << (char)tsp_data[index];
                PUSH_U1(tsp_data[index]); // response add name
            }
            byte_index += current_name_len;

            SMLK_LOGE(" byte_index = %d", byte_index);
            ss >> dbc_name;
            ss.clear();
            ss.str(std::string());

            SMLK_LOGE("dbc_name = %s", dbc_name.c_str());
            //  word: uploadid
            DbcConfig.upload_id = (SMLK_UINT32)tsp_data[byte_index];
            DbcConfig.upload_id<<= 8;
            ++byte_index;

            DbcConfig.upload_id |= (SMLK_UINT32)tsp_data[byte_index];
            ++byte_index;
            SMLK_LOGE(" DbcConfig.upload_id = %d", DbcConfig.upload_id);

            // word: collect interval
            DbcConfig.collection_interval = (SMLK_UINT32)tsp_data[byte_index];
            DbcConfig.collection_interval<<= 8;
            ++byte_index;

            DbcConfig.collection_interval |= (SMLK_UINT32)tsp_data[byte_index];
            if ( DbcConfig.collection_interval > MAX_COLLECT_INTERVAL_MS) {
                DbcConfig.collection_interval = MAX_COLLECT_INTERVAL_MS;
            }
            ++byte_index;
            SMLK_LOGE("DbcConfig.collection_interval = %d", DbcConfig.collection_interval);
            // word: upload interval

            DbcConfig.upload_interval = (SMLK_UINT32)tsp_data[byte_index];
            DbcConfig.upload_interval<<= 8;
            ++byte_index;

            DbcConfig.upload_interval |= (SMLK_UINT32)tsp_data[byte_index];
            ++byte_index;
            SMLK_LOGE("DbcConfig.upload_interval = %d", DbcConfig.upload_interval);

            // word : http url len  http_url_len
            SMLK_LOGE("http_url_len byte_index = %d", byte_index);

            http_url_len = (SMLK_UINT8)tsp_data[byte_index];
            SMLK_LOGE("http_url_len = %d", http_url_len);
            ++byte_index;

            // url
            std::stringstream   sss;
            for ( int index = byte_index; index < byte_index + http_url_len; index++ ) {
                // SMLK_LOGE("sss i %d = %02x", index , tsp_data[index]);
                sss << (char)tsp_data[index];
            }
            byte_index += http_url_len;
            SMLK_LOGD(" byte_index = %d", byte_index);
            sss >> http_url;
            SMLK_LOGD("http_url = %s", http_url.c_str());
            sss.clear();
            sss.str(std::string());

            // md5
            std::string md5_str;
            for ( int index = byte_index; index < byte_index + 16; index++ ) { // hard code
                sss << (char)tsp_data[index];
            }
            sss >> md5_str;
            sss.clear();
            sss.str(std::string());
            SMLK_LOGD("[md5debug]md5_str = %s", md5_str.c_str());

            byte_index += 16;
            SMLK_LOGD(" byte_index = %d", byte_index);
            // data type
            // ++byte_index;
            // m_upload_data_type
            SMLK_UINT8 datatype = tsp_data[byte_index];

            SMLK_LOGE("datatype  = %d", tsp_data[byte_index]);

            left_size = (tsp_data.size() - byte_index);
            SMLK_LOGE("left_size = %d", left_size);

            --config_count;
            SMLK_LOGE(" config_count = %d", config_count);
            SMLK_LOGE(" update m_dbc_config m_dbc_config size %d ", m_dbc_config.size());
            /*将对应信息配置到对应 m_dbc_config map*/
            SMLK_UINT32 index = 0 ;
            auto iter = g_name_index_map.find(dbc_name);
            if ( g_name_index_map.end() != iter) {
                index = iter->second;
            }
            if (config_count > 5) {
                PUSH_U1(0x05); // out of limit
                break;
            } else {
                auto iter_local_name = g_name_index_map.find(dbc_name);
                if ( g_name_index_map.end() == iter) {
                    PUSH_U1(0x05); // name out of limit
                    break;
                }

                SMLK_LOGI("http_url = %s", http_url.c_str());
                // check whether need update and download
                // std::lock_guard<std::mutex> config_lock(m_dbc_config_mutex);
                for (auto &pair: m_dbc_config) {
                    std::string tmp_md5_str = pair.second.getFileMd5();
                    if (md5_str == tmp_md5_str) {
                        // response add result parse success if repeat .also need update config
                        // but this case only diff is extra download once
                    }
                    std::string tmp_str =  pair.second.getDbcName();
                    SMLK_LOGD(" pair.second.getDbcName() = %s", tmp_str.c_str());
                    auto iter = g_name_index_map.find(dbc_name);
                    if ( g_name_index_map.end() != iter) {
                        // check whether already use name
                        dbc_config_struct config_info;
                        config_info.upload_id = DbcConfig.upload_id;
                        config_info.collect_interval = DbcConfig.collection_interval;
                        config_info.upload_interval = DbcConfig.upload_interval;
                        config_info.data_type = datatype;
                        dbc_index = iter->second;
                        if (dbc_name == pair.second.getDbcName()) {
                            SMLK_LOGD(" receive dbc index is = %d", pair.first);
                            pair.second.setUploadId(DbcConfig.upload_id);
                            pair.second.setHttpUrl(http_url);
                            pair.second.setCollectionTnterval(DbcConfig.collection_interval);
                            pair.second.setUploadInterval(DbcConfig.upload_interval);
                            pair.second.setFileMd5(md5_str);
                            pair.second.setUploadDataType(datatype);
                            // reset
                            // use new timer
                            // m_timer->Reset();
                            UpdateDbcConfigByIndex(dbc_index , config_info);
                        } else {
                            // m_timer->Reset();
                            SMLK_LOGD("new dbc = %s", dbc_name.c_str());
                            {
                                auto it_dbc = m_dbc_config.find(index);
                                if (m_dbc_config.end() != it_dbc) {
                                    it_dbc->second.setDbcName(dbc_name);
                                    it_dbc->second.setUploadId(DbcConfig.upload_id);
                                    it_dbc->second.setHttpUrl(http_url);
                                    it_dbc->second.setCollectionTnterval(DbcConfig.collection_interval);
                                    it_dbc->second.setUploadInterval(DbcConfig.upload_interval);
                                    it_dbc->second.setFileMd5(md5_str);
                                    it_dbc->second.setUploadDataType(datatype);
                                }
                            }
                            UpdateDbcConfigByIndex(dbc_index , config_info);
                        }
                        break;
                    }
                }
                // 更新完dbc配置之后
                auto result = HttpGet(http_url , dbc_name);
                // 下载完成
                if ( result == 1 ) {
                    SMLK_LOGD(" download = %s sucess", dbc_name);
                    // todo:diff handle num > 1
                    if (!ConfigLoader::CheckDbcValid(dbc_index) ) {
                        SMLK_LOGI(" xukedebug  dbc is invalid!!!!!!!!!!!" );
                        PUSH_U1(0x04); // response add result parse fail
                        std::vector<std::string> str_vec;
                        str_vec.emplace_back(dbc_name);
                        // do execute clear dbc config
                        DoExecuteClearDbcConfig(str_vec);
                    } else {
                        PUSH_U1(0x03); // response add result parse success
                        // UPDATE CONFIG
                        MsgHead head(M_SL_MSGID_DBC_FILE_UPDATE_NOTIFY, 0, 0, 0, 0, dbc_index);
                        Post(head, nullptr, 0);
                    }
                } else {
                    PUSH_U1(0x02); // response add result download fail
                }
            }
            if ( 0 == config_count) {
                break;
            }
            ++byte_index;
        }
        // out of can dbc process
        SMLK_LOGD("STATR TIMER!!!!!!!!!!!!!!!!!!!");

        m_timer->Start();

    IpcTspHead      resp_head;
    std::memset(&resp_head, 0x00, sizeof(resp_head));
    resp_head.protocol  = Protocol::SL_808;
    resp_head.msg_id    = M_SL_MSGID_TSP_DYC_REP;
    // resp_head.seq_id    = htobe16(head.m_seq);
    resp_head.qos       = QOS_SEND_ALWAYS;

    SMLK_LOGD("Tspset DycConfig send to tsp size %d", data.size());

    TspServiceApi::getInstance()->SendMsg(resp_head, data.data() , data.size());
}

/******************************************************************************
 * NAME: SendTspCommonResponse
 *
 * DESCRIPTION: 所有tsp消息先回复通用应答
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::SendTspCommonResponse(IN MsgHead &head, IN SMLK_UINT8 &result)
{
    SMLK_LOGD(" ON SendTspCommonResponse");

    J808CommResp response_body;

    response_body.msg_id = htobe16(head.m_id);
    response_body.seq_id = htobe16(head.m_seq);
    response_body.result = result;

    IpcTspHead      resp_head;
    std::memset(&resp_head, 0x00, sizeof(resp_head));
    resp_head.protocol  = Protocol::SL_808;
    resp_head.msg_id    = M_SL_MSGID_TBOX_COMMOM_REP;

    // resp_head.qos       = QOS_SEND_ALWAYS;

    SMLK_LOGD(" Common Response size[%d] to tsp ", (SMLK_UINT32)sizeof(response_body));

    TspServiceApi::getInstance()->SendMsg(resp_head, (SMLK_UINT8 *)&response_body , sizeof(response_body));
}

/******************************************************************************
 * NAME: ResponseTspSendDycDbc
 *
 * DESCRIPTION: tsp下发DBC回复
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::ResponseTspSendDycDbc(IN MsgHead &head)
{
    SMLK_LOGD(" ResponseTspSendDycDbc ");
}


void DynamicCollectionService::OnTspReadDycConfig(IN MsgHead &head)
{
    SMLK_LOGD(" OnTspReadDycConfig ");

    std::vector<SMLK_UINT8> data;
    PUSH_U2(head.m_seq);
    PUSH_U1(TSP_VERSION);
    PUSH_U1(TSP_READ_DYC_CONFIG);
    std::size_t num_index = data.size();
    PUSH_U1(0x00); // file num
    SMLK_UINT8 valid_num = 0;
    int i = 0;
    // std::lock_guard<std::mutex> lk(m_dbc_config_mutex);
    for (auto &pair: m_dbc_config) {
        SMLK_UINT32 upload_interval =  pair.second.getUploadInterval();
        SMLK_UINT32 collect_interval =  pair.second.getCollectionTnterval();
        if ( 0 != upload_interval && 0 != collect_interval) {
            std::string dbc_name = pair.second.getDbcName();
            SMLK_UINT8 name_len = dbc_name.length();
            PUSH_U1(name_len);
            SMLK_LOGD("read dbc name is %s", dbc_name.c_str());

            for (int i = 0; i < dbc_name.length(); i++)
            {
                SMLK_LOGD("read dbc name is %x", dbc_name[i]);
                PUSH_U1(dbc_name[i]);
            }

            std::string fileMd5("");
            std::vector<SMLK_UINT8> vec_fileMd5;
            std::string dbc_path(DBC_FILE_PATH);
            dbc_path.append(dbc_name);
            SMLK_LOGD("[md5debug] calculate dbc_path === %s ", dbc_path.c_str());
            // add file exit check
            FileTool::CalculateFileMd5Sum(dbc_path, vec_fileMd5);
            if (MD5_BYTE_LENGTH <= vec_fileMd5.size()) {
                for (SMLK_UINT16 i = 0; i < MD5_BYTE_LENGTH; ++i) {
                    SMLK_LOGD("[md5debug] data [%d] is [%02x] ", i, vec_fileMd5[i]);
                    PUSH_U1(vec_fileMd5[i]);
                }
                vec_fileMd5.clear();
            } else {
                SMLK_LOGE("[md5debug] INVALID MD5 ");
                PUSH_U4(0x00);
                PUSH_U4(0x00);
                PUSH_U4(0x00);
                PUSH_U4(0x00);
            }

            SMLK_UINT8 data_type = pair.second.getUploadDataType();
            PUSH_U1(data_type); // 物理值/实际值
            ++valid_num;
            ++i;
        }
    }
    // try to fix the num
    data[num_index]      = (SMLK_UINT8)(valid_num);
    SMLK_LOGD("size of DycReadIntervalRespBody is %d", sizeof(DycReadIntervalRespBody));


    IpcTspHead      resp_head;
    std::memset(&resp_head, 0x00, sizeof(resp_head));
    resp_head.protocol  = Protocol::SL_808;
    resp_head.msg_id    = M_SL_MSGID_TSP_DYC_REP;
    // resp_head.seq_id    = head.m_seq;
    resp_head.qos       = QOS_SEND_ALWAYS;

    SMLK_LOGD("TspReadDycConfig send to tsp size %d", data.size());

    TspServiceApi::getInstance()->SendMsg(resp_head, data.data() , data.size());
}


/******************************************************************************
 * NAME: OnDycConfig
 *
 * DESCRIPTION: 所有tsp消息先回复通用应答
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::OnDycConfig(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("  OnDycConfig  cmd");
    // 回复通用应答
    // TODO :check every cmd length
    SMLK_UINT8 result = 0; // success
    SendTspCommonResponse(head, result);

    auto  dyc_raw_playload  = reinterpret_cast<const RawDycPayload *>(data.data());
    SMLK_LOGD("sub cmd_id = %d ", dyc_raw_playload->cmd_id);

    switch (dyc_raw_playload->cmd_id)
    {
        case TSP_SET_DYC_INTERVAL:
            {
                OnTspSetDycInterval(head, data);
            }
            break;
        case TSP_READ_DYC_INTERVAL:
            {
                ResponseTspReadDycInterval(head);
            }
            break;
        case TSP_SEND_DYC_DBC:
            {
                m_timer->Stop();
                OnTspSendDycDbc(head, data);
            }
            break;
        case TSP_READ_DYC_CONFIG:
            {
                OnTspReadDycConfig(head);
            }
            break;
        case TSP_CLEAR_DYC_CONFIG:
            {
                ResponseTspClearDycConfig(head, data);
                OnTspClearDycConfig(data);
            }
            break;
        default:
            break;
    }
}

/******************************************************************************
 * NAME: InitVehicleDataPool
 *
 * DESCRIPTION: 从全量数据得到解析规则
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::InitVehicleDataPool()
{
    SMLK_LOGD("InitVehicleDataPool");

    CanFormatData::CanPhysicalDataInfo info;
    info.decoded_can_data = 0;
    info.physical_data = 0;
    info.data_valid = false;

    // g_vehicle_data_pool.clear();
    // dbc更新时再次更新
    std::lock_guard<std::mutex> lk(m_message_decode_map_mutex);
    for (const auto& message_map : g_message_decode_map)
    {
        for (const auto& signal : message_map.second.second)
        {
            // 获取can信号timeout检测定时器最小执行周期
            m_check_period = 100; // hard code modify
            // GetGcd(m_check_period, message_map.second.first.cycle);  // siganal 对应的can信号采集周期
            info.cycle = message_map.second.first.cycle;
            std::lock_guard<std::mutex> lock(g_vehicle_data_pool_mutex); // dbc更新时再次更新
            if (g_vehicle_data_pool.find(signal.GetIndex()) != g_vehicle_data_pool.cend())
            {
                SMLK_LOGD("index[%d] is aleardy exsit!!!", signal.GetIndex());
            }
            g_vehicle_data_pool[signal.GetIndex()] = info;     // 初始化spn对应的信号物理值
        }
    }


    // 初始化需要采集的can数据到缓存 m_cached_signals

    for (const auto &pair : vehicle::g_header_signals) {
        auto vec_header = pair.second;
        SMLK_UINT32 index = pair.second.GetIndex();
        auto iter = m_cached_signals.find(index);
        if (m_cached_signals.end() == iter) {
            SMLK_LOGD("m_cached_signals add sig [%d] ", index);
            m_cached_signals.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(pair.second.GetIndex()),
                std::forward_as_tuple(pair.second)
            );
        }
    }

    for (const auto &pair : g_message_decode_map) {
        auto vec = pair.second.second;
        if (!vec.empty()) {
            for (auto &it: vec) {
                SMLK_UINT32 index = it.GetIndex();
                auto iter = m_cached_signals.find(index);
                if (m_cached_signals.end() == iter) {
                    SMLK_LOGD("m_cached_signals add  sig[%d] ", index);
                    m_cached_signals.emplace(
                        std::piecewise_construct,
                        std::forward_as_tuple(it.GetIndex()),
                        std::forward_as_tuple(it)
                    );
                }
            }
        }
    }
    SMLK_LOGD("init g_vehicle_data_pool size[%d], m_check_period[%d]", g_vehicle_data_pool.size(), m_check_period);
}

/******************************************************************************
 * NAME: ProcessPhysicalData
 *
 * DESCRIPTION: 更新物理值
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::ProcessPhysicalData(Index index, CanFormatData::CanPhysicalDataInfo& info)
{
    // SMLK_LOGD("ProcessPhysicalData tid is %d", (SMLK_UINT32)gettid());

    // SMLK_LOGD(" current physical data = %lf ", info.physical_data);
    // SMLK_LOGD(" data_valid is = %d ", info.data_valid);
    {
        std::lock_guard<std::mutex> mtx(g_vehicle_data_pool_mutex);
        auto vehicle_data_iter = g_vehicle_data_pool.find(index);
        if (vehicle_data_iter != g_vehicle_data_pool.end())
        {
            // SMLK_LOGD("old data = %lf, new data = %lf", vehicle_data_iter->second.physical_data, info.physical_data);

            // data have changed
            if (vehicle_data_iter->second.physical_data !=  info.physical_data
                || vehicle_data_iter->second.data_valid != info.data_valid)
            {
                // SMLK_LOGD("valuetest index data have change !!!!!!!!!!!!!");
                vehicle_data_iter->second.decoded_can_data = info.decoded_can_data;
                vehicle_data_iter->second.physical_data    = info.physical_data;
                vehicle_data_iter->second.tp               = info.tp;
                vehicle_data_iter->second.data_valid       = info.data_valid;   // 更新最新物理值到对应spn的sigFormat

                // data change post physic data  M_SL_MSGID_SIGNAL_DATA_CHANGE
                // Post(MsgHead(M_SL_MSGID_SIGNAL_DATA_CHANGE, 0, 0, 0, 0, 0), (SMLK_UINT8*)info.physical_data);
                CanFormatData::VehicleData data;
                std::memset(&data, 0, sizeof(CanFormatData::VehicleData));
                data.index = index;
                data.value = info.physical_data;
                data.valid = info.data_valid;

                Post(
                MsgHead(M_SL_MSGID_SIGNAL_DATA_CHANGE, data.index, data.valid ? 0x01 : 0x00, 0, sizeof(data.value), 0),
                reinterpret_cast<const SMLK_UINT8 *>(&data.value),
                sizeof(data.value)
                );
            }
            // data have not changed, do not send data
            else
            {
                // SMLK_LOGD("data have not changed, do not send data, function return");
                vehicle_data_iter->second.tp  = info.tp;
                return;
            }
        }
        else
        {
            // error log, index is not exsit
            SMLK_LOGW("index is not in g_vehicle_data_pool!!!!!, function return");
            return;
        }
    }
    // 也更新到备份的数据源   m_cached_signals
}

/******************************************************************************
 * NAME: ProcessNormalCanFrame
 *
 * DESCRIPTION: 处理can_id对应的信号
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::ProcessNormalCanFrame(const CanFormatData::CanFrame* can_frame, const std::vector<SignalFormat> &vec_signals)
{
    // SMLK_LOGD(" on ProcessCanFrame");
    PhysicalData physical_data = 0;
    CAN_DATA     can_data = 0;
    SMLK_BOOL    decode_ret = true;

    for (const auto &signal : vec_signals)
    {
        CanFormatData::CanPhysicalDataInfo info;
        info.tp = steady_clock::now();
        // SMLK_LOGW("ProcessNormalCanFrame signal [%d] ", signal.GetIndex());
        // SMLK_LOGW("ProcessNormalCanFrame signal Min[%f] Max[%f] ", signal.Min() , signal.Max());
        if (can_frame->can_dlc == 0) {
            info.decoded_can_data = can_data;
            info.physical_data = physical_data;
            info.data_valid = false;
            info.cycle = DEFAULT_MSG_CYCLE; // To read cycle and can channel from dbc
            info.signal_timeout_out = can_frame->can_dlc == 0 ? true : false;

            // 变化更新缓存数据
            ProcessPhysicalData(signal.GetIndex(), info);
            continue;
        }

        // get physical data and decoded can data
        decode_ret = signal.Decode(can_frame, physical_data, can_data);
        // SMLK_LOGD("Decode result : Index %d , physical data = %lf, decoded can_data = %llu", signal.GetIndex(), physical_data, can_data);
        if (!decode_ret)
        {
            return;
        }

        info.decoded_can_data = can_data;
        info.physical_data = physical_data;
        info.data_valid = decode_ret;
        info.cycle = DEFAULT_MSG_CYCLE; // To read cycle and can channel from dbc
        info.signal_timeout_out = can_frame->can_dlc == 0 ? true : false;

        // 变化更新缓存数据
        ProcessPhysicalData(signal.GetIndex(), info);
    }
}

/******************************************************************************
 * NAME: GetGNSS
 *
 * DESCRIPTION: 获取GNSS
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::GetGNSS()
{
    smartlink_sdk::LocationInfo location;
    auto rc = smartlink_sdk::Location::GetInstance()->GetLocation(location);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != rc ) {
        SMLK_LOGE("[GNSS] fail to get location information, rc: %d", static_cast<std::int32_t>(rc));
    } else {
        m_gnss_signals.find(vehicle::HeaderDataID::GNSSSpeed)->second             = location.speed;
        m_gnss_signals.find(vehicle::HeaderDataID::GNSSHeading)->second           = location.heading;
        m_gnss_signals.find(vehicle::HeaderDataID::GNSSAltitude)->second          = location.altitude;
        m_gnss_signals.find(vehicle::HeaderDataID::GNSSLatitude)->second          = location.latitude;
        m_gnss_signals.find(vehicle::HeaderDataID::GNSSLongitude)->second         = location.longitude;
        m_gnss_signals.find(vehicle::HeaderDataID::GNSSLocatedStatus)->second     = location.fix_valid ? 0 : 1;
        m_gnss_signals.find(vehicle::HeaderDataID::GNSSTimeStamp)->second         = location.utc;
#ifdef SL_DEBUG
        SMLK_LOGD("[GNSS]   location.speed:     %f", location.speed);
        SMLK_LOGD("[GNSS]   location.heading:   %f", location.heading);
        SMLK_LOGD("[GNSS]   location.altitude:  %f", location.altitude);
        SMLK_LOGD("[GNSS]   location.latitude:  %f", location.latitude);
        SMLK_LOGD("[GNSS]   location.longitude: %f", location.longitude);
        SMLK_LOGD("[GNSS]   location.fix_valid: %s", (location.fix_valid) ? "YES" : "NO");
        SMLK_LOGD("[GNSS]   location.utc:       %u", location.utc);
#endif

    }

    smartlink_sdk::GnssSatelliteInfo    inf;
    rc = smartlink_sdk::Location::GetInstance()->GetSatelliteInfo(inf);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != rc ) {
        SMLK_LOGE("[GNSS] fail to get satellite information, rc: %d", static_cast<std::int32_t>(rc));
    } else {
#ifdef SL_DEBUG
        SMLK_LOGD("[GNSS] inf.tracked_num:    %u", inf.tracked_num);
#endif
        m_gnss_signals.find(vehicle::HeaderDataID::GNSSSatelliteUsed)->second = inf.tracked_num;
    }
}

/******************************************************************************
 * NAME: OnLocation
 *
 * DESCRIPTION: 位置信息
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::OnLocation(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{

    switch ( head.m_seq ) {
        case Location::SL_SERVICE_READY:
        SMLK_LOGD("[GNSS] SL_SERVICE_READY  ");
            GetGNSS();
            break;
        case Location::SL_SERVICE_EXIT:
            SMLK_LOGD("[GNSS] SL_SERVICE_EXIT  ");
            m_vec_signals.find(vehicle::HeaderDataID::GNSSLocatedStatus)->second = 1;
            break;
        case Location::SL_ANTENNA_CHANGED:
            {
            }
            break;
        case Location::SL_SATELLITE_CHANGED:
            {
                if ( sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8) != data.size() ) {
                    // we should never get here
                    SMLK_LOGE("[GNSS] unexpected data length for satellite changed notification: %d", (SMLK_UINT8)data.size());
                    return;
                }
                //m_gnss_signals.find(vehicle::HeaderDataID::GNSSSatelliteUsed)->second = (int)data[0];
            }
            break;
        case Location::SL_LOCATION_CHANGED:
            {

                if ( sizeof(LocatingInf) != data.size() ) {
                    // we should never get here
                    SMLK_LOGE("[GNSS] unexpected data length for location changed notification: %d", (SMLK_UINT8)data.size());
                    return;
                }

                LocatingInf *inf = (LocatingInf *)data.data();
                // if no fixed, no update

                if (inf->flags & LocatingFlags::SL_FIXED)
                {
                    m_gnss_signals.find(vehicle::HeaderDataID::GNSSSpeed)->second           = inf->speed;
                    m_gnss_signals.find(vehicle::HeaderDataID::GNSSHeading)->second         = inf->heading;
                    m_gnss_signals.find(vehicle::HeaderDataID::GNSSAltitude)->second        = inf->altitude;
                    m_gnss_signals.find(vehicle::HeaderDataID::GNSSLatitude)->second        = inf->latitude;
                    m_gnss_signals.find(vehicle::HeaderDataID::GNSSLongitude)->second       = inf->longitude;
                    m_gnss_signals.find(vehicle::HeaderDataID::GNSSLocatedStatus)->second   = (inf->flags & LocatingFlags::SL_FIXED) ? 0 : 1;
                    m_gnss_signals.find(vehicle::HeaderDataID::GNSSTimeStamp)->second       = inf->utc * 0.001;         // ms --> s

                } else {
                    SMLK_LOGD("[GNSS] SL_LOCATION_CHANGED  ");
                    m_gnss_signals.find(vehicle::GNSSLocatedStatus)->second = 1;
                }
            }
            break;
        default:
            SMLK_LOGE("[GNSS] unsupported location event: 0x%08X", head.m_seq);
            break;
    }
}


/******************************************************************************
 * NAME: CanFrameAceept
 *
 * DESCRIPTION: 从mcu接收到can原始数据
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::CanFrameAceept(const CanFormatData::CanFrame* can_frame)
{
    // SMLK_LOGD("VehicleService::CanFrameAceept Can data id[0x%x]", can_frame->can_id);
    // m_message_decode_regulation do not need lock,it run in main thread
    // SMLK_LOGD("canframe tid is %d", (SMLK_UINT32)gettid());
    std::lock_guard<std::mutex> lk(m_message_decode_map_mutex);
    auto strategy_iter = g_message_decode_map.find(can_frame->can_id);
    if (strategy_iter != g_message_decode_map.end())
    {
        ProcessNormalCanFrame(can_frame, strategy_iter->second.second);
    }
    else
    {
#ifdef SL_DEBUG
        // SMLK_LOGW("Unprocess Can data id[0x%x]", can_frame->can_id);
#endif
    }
}

std::size_t DynamicCollectionService::CompressOffset() const {
    //     frequency             compress flag        reserved              length
    return sizeof(SMLK_UINT16) + sizeof(SMLK_UINT8) + sizeof(SMLK_UINT32) + sizeof(SMLK_UINT16);
}

/******************************************************************************
 * NAME: SampleSignalsByDbcIndex
 *
 * DESCRIPTION: 根据dbc_index从配置文件中获取数据映射 从缓存数据中获取数据
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::SampleSignalsByDbcIndex(IN MsgHead &head, IN DBC_INDEX &index)
{
        SMLK_LOGD("on SampleSignalsByDbcIndex index is[%d]", index);
        // SMLK_LOGD("SampleSignalsByDbcIndex tid is %d", (SMLK_UINT32)gettid());

        SMLK_UINT32 uploadid = head.m_extra;
        // 从cached signals 将dbc对应spn 采集
        std::lock_guard<std::mutex> lk(m_dbc_sample_siganls_mutex);
        SMLK_LOGD("on m_dbc_sample_signals_map size is %d ", m_dbc_sample_signals_map.size());
        if ( m_dbc_sample_signals_map.empty() ) {
            SMLK_LOGE("m_dbc_sample_signals_map is empty");
        }
        auto iter_dbc_signal_map = m_dbc_sample_signals_map.find(index);
        if (m_dbc_sample_signals_map.end() != iter_dbc_signal_map ) {
            SMLK_LOGD("on find index[%d] success", index);

            if (!m_sampled_frame_signals.empty()) {
                auto it_sampled = m_sampled_frame_signals.find(index);
                if ( m_sampled_frame_signals.end() == it_sampled) {
                    // it_sampled->second.insert(index, );
                    SMLK_LOGD("*******not find index in m_sampled_frame_signals********");
                } else {
                    it_sampled->second.emplace_back(); // 此dbc对应采集数据的vector里增加一帧
                    // SMLK_LOGD("********add one frame********");
                    SMLK_LOGD("********it_sampled->second size now is %d********", it_sampled->second.size());
                }
            } else {
                SMLK_LOGE("m_sampled_frame_signals empty!");
            }
        SMLK_LOGD("on this dbc contains spn size is %d", iter_dbc_signal_map->second.size());

        auto it = m_cached_signals.find(vehicle::HeaderDataID::GNSSTimeStamp);
        if (m_cached_signals.end() != it) {
            // TODO 授时
            it->second = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }

        it = m_cached_signals.find(vehicle::HeaderDataID::GNSSTimeStampMs);
        if (m_cached_signals.end() != it) {
            auto now_systime    = std::chrono::system_clock::now();
            it->second = std::chrono::duration_cast<std::chrono::milliseconds>(now_systime.time_since_epoch()).count() % 1000;
        }

        it = m_cached_signals.find(vehicle::HeaderDataID::GNSSSpeed);
        if (m_cached_signals.end() != it) {
            it->second = m_gnss_signals.find(vehicle::HeaderDataID::GNSSSpeed)->second.Val();
            if (0 != m_gnss_signals.find(vehicle::HeaderDataID::GNSSLocatedStatus)->second.Encoded() ) {
                it->second.Invalid();
            }
        }

        it = m_cached_signals.find(vehicle::HeaderDataID::GNSSAltitude);
        if (m_cached_signals.end() != it) {
            it->second = m_gnss_signals.find(vehicle::HeaderDataID::GNSSAltitude)->second.Val();
            if (0 != m_gnss_signals.find(vehicle::HeaderDataID::GNSSLocatedStatus)->second.Encoded() ) {
                it->second.Invalid();
            }
        }

        it = m_cached_signals.find(vehicle::HeaderDataID::GNSSLatitude);
        if (m_cached_signals.end() != it) {
            it->second = m_gnss_signals.find(vehicle::HeaderDataID::GNSSLatitude)->second.Val();
        }

        it = m_cached_signals.find(vehicle::HeaderDataID::GNSSLongitude);
        if (m_cached_signals.end() != it) {
            it->second = m_gnss_signals.find(vehicle::HeaderDataID::GNSSLongitude)->second.Val();
        }
        it = m_cached_signals.find(vehicle::HeaderDataID::GNSSLocatedStatus);
        it->second = m_gnss_signals.find(vehicle::HeaderDataID::GNSSLocatedStatus)->second;
        if (m_cached_signals.end() != it) {
            SMLK_UINT16 status = 0;
            if (0 != it->second.Encoded()) {
                it->second.Invalid();
            } else {
                    auto it_tmp = m_gnss_signals.find(vehicle::HeaderDataID::GNSSHeading);
                    if (!it_tmp->second.Valid()) {
                        // hirain range: 0 < data < 360
                        // tsp range: 0-359
                        if (359 <= it_tmp->second.Val() ) {
                            status |= 359 & 0x1FF;
                        } else {
                            status |= 0 & 0x1FF;
                        }
                    } else {
                        status |= it_tmp->second.Encoded() & 0x1FF;
                    }

                it_tmp = m_gnss_signals.find(vehicle::HeaderDataID::GNSSLatitude);
                if ( it_tmp->second.Val() < 0  ) {
                    status |= ((SMLK_UINT32)1) << 14;
                }

                it_tmp = m_gnss_signals.find(vehicle::HeaderDataID::GNSSLongitude);
                if ( it_tmp->second.Val() < 0 ) {
                    status |= ((SMLK_UINT32)1) << 15;
                }
                // it->second = status;
            }
        } else {
            SMLK_LOGD("iter not find  find GNSSLocatedStatus ");
        }

            for (auto const &spn: iter_dbc_signal_map->second) {
                // 需要采集的所有spn 插入此帧
                // SMLK_LOGD("current insert spn is %d", spn);
                auto iter_cached_signal = m_cached_signals.find(spn);
                if ( m_cached_signals.end() == iter_cached_signal) {
                    SMLK_LOGE("m_cached_signals not find should collect spn[%d]", spn);
                } else {
                    auto it_frames = m_sampled_frame_signals.find(index);
                    if (m_sampled_frame_signals.end() != it_frames
                        && (!m_sampled_frame_signals.empty())) {
                            it_frames->second.back().emplace(
                                    std::piecewise_construct,
                                    std::forward_as_tuple(iter_cached_signal->first),
                                    std::forward_as_tuple(iter_cached_signal->second)
                            );
                    } else {
                        SMLK_LOGD("m_sampled_frame_signals not find dnc index or empty!!");
                        std::vector<std::unordered_map<SPN_INDEX, SignalFormat>> vec;
                        std::unordered_map<SPN_INDEX, SignalFormat> map;
                        if ( m_cached_signals.end() != iter_cached_signal) {
                            SMLK_LOGE("m_cached_signals not find index spn!!!!");
                            map.insert(std::make_pair(iter_cached_signal->first, iter_cached_signal->second));
                            vec.push_back(map);
                            SMLK_LOGD("m_sampled_frame_signals insert index =  %d", index);
                            m_sampled_frame_signals.insert(std::make_pair(index, vec) );
                        }
                    }
                }
            }
            if (m_sampled_frame_signals.empty()) {
                SMLK_LOGD("m_sampled_frame_signals is empty");
                return;
            }
            auto it_index_frames_vec = m_sampled_frame_signals.find(index);
            SMLK_LOGD("it_index_frames_vec size is  %d", it_index_frames_vec->second.size());

            if ( it_index_frames_vec->second.size() >= m_current_frameLimit
                || it_index_frames_vec->second.size() >= FRAME_LIMIT) {
                std::vector<SMLK_UINT8> encoded;
                Encode(encoded, index);
                it_index_frames_vec->second.clear();
                if ( m_internal_flags & InternalFlags::SL_FLAG_COMPRESS ) {
                    IpcTspHead  resp_head;
                    std::memset(&resp_head, 0x00, sizeof(resp_head));
                    resp_head.msg_id    = M_SL_MSGID_TP_DYC;
                    resp_head.protocol  = Protocol::SL_808;
                    resp_head.qos       = QOS_SEND_ALWAYS;

                    encoded.erase(encoded.begin(), encoded.begin() + CompressOffset());
                    MsgHead head(M_SL_MSGID_COMPRESS_REQ, 0, Protocol::SL_808, 0, ((SMLK_UINT16)Frequency::SL_DBC_A) << 8, uploadid);
                    m_compress_events.emplace(
                        head,
                        std::move(encoded),
                        std::bind(&DynamicCollectionService::OnCompress, this, std::placeholders::_1, std::placeholders::_2)
                    );
                } else {
                    SMLK_LOGD("not set compress flag");
                }
            }
        }
}

/******************************************************************************
 * NAME: CheckSampleCondition
 *
 * DESCRIPTION: 除ign on/off 状态检查使用
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::CheckSampleCondition(IN MsgHead &head)
{
    SMLK_LOGD("CheckSampleCondition ");
}

/******************************************************************************
 * NAME: OnTimer
 *
 * DESCRIPTION: 一秒触发一次 根据dbc index获取缓存规则中的采集和上报间隔
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::OnTimer(IN MsgHead &head)
{

    SMLK_UINT32 index = head.m_extra;
    for (auto config : m_dbc_config) {
        SMLK_INT32 dbc_index = config.first;
        SMLK_UINT32 collect_interval = config.second.getCollectionTnterval();
        SMLK_UINT32 upload_interval = config.second.getUploadInterval();
        SMLK_UINT32 upload_id = config.second.getUploadId();
        // collect is ms upload is s
        if (collect_interval != 0 && upload_interval != 0) {
            if (0 == (index*100) % collect_interval) {
                SMLK_LOGD("!!!dbc_index == %d  going to sample data!!!", dbc_index);
                SMLK_LOGD("timer call dbc index  %d upload interval is %d ", dbc_index , config.second.getUploadInterval());
                SMLK_LOGD("timer call dbc index %d collect interval is %d ", dbc_index , config.second.getCollectionTnterval());

                m_current_frameLimit = (SMLK_UINT32) (upload_interval * 1000 / collect_interval);
                SMLK_LOGD("!!!m_current_frameLimit %d ", m_current_frameLimit);
                MsgHead head(0, 0, 0, 0, 0, upload_id);
                SMLK_LOGD("resp_head.msg_id use of uploadid is = %0x", head.m_extra);

                SampleSignalsByDbcIndex(head, dbc_index);
                m_sampling_frequency = config.second.getCollectionTnterval();
            }
        }
    }
}

/******************************************************************************
 * NAME: EncodeZip
 *
 * DESCRIPTION: zip之后上报添加ipc数据头
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
std::uint32_t DynamicCollectionService::EncodeZip(std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("EncodeZip");

    data.clear();

    SMLK_UINT16    length = (SMLK_UINT16)m_compressed.size();

    // 协议控制字
    m_crompression |= 0x01;         // bit0 : 是否压缩

    if (TspState::SL_TSP_DISCONNECTED == m_tsp_connect_state) {
        m_crompression |= 0x02;     // bit1 : 是否重传
    }

    PUSH_U2(m_sampling_frequency);
    PUSH_U1(m_crompression); // m_crompression
    PUSH_U1(0x00); // protocal
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U2(length);

    data.insert(data.end(), m_compressed.begin(), m_compressed.end());

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: EncodeZip
 *
 * DESCRIPTION: zip成功之后上报添加ipc数据头后发送
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::OnZipNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{

#ifdef SL_DEBUG
        std::stringstream   ss;
        std::size_t         index   = 0;
        std::size_t         seq     = 0;
        for ( auto ch : data ) {
            ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
            if ( ++index >=  SL_PLATFORM_MAX_BYTES ) {
                 SMLK_LOGD("compressed data: [%03u]%s", seq, ss.str().c_str());
                ss.str("");
                ss.clear();
                seq++;
                index = 0;
            }
        }
       SMLK_LOGD("compressed data: [%03u]%s  end", seq, ss.str().c_str());
        for ( auto ch : data ) {
            ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
        }

        SMLK_LOGD("compressed data: \"%s\"  end", ss.str().c_str());
#endif  // #ifdef SL_DEBUG


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

    m_crompression         = 0x01 ;
    m_compressed           = data;

    std::vector<SMLK_UINT8> encoded;

    if ( SL_SUCCESS != EncodeZip(encoded) ) {
        SMLK_LOGE("fail to encode compressed bucket message");
        return;
    }
    SMLK_LOGD("encoded.size() = %d", encoded.size());

#ifdef SL_DEBUG
        // for(int i = 0; i < data.size() ;i++)
        // {
        //     SMLK_LOGE("encoded data[%d] = %02x", i, encoded[i]);
        // }
#endif

    IpcTspHead  resp_head;
    std::memset(&resp_head, 0x00, sizeof(resp_head));
    SMLK_LOGD("resp_head.msg_id  is = %0x", head.m_extra);
    resp_head.protocol  = Protocol::SL_808;
    resp_head.msg_id    = (SMLK_UINT16)head.m_extra; // it should be 260-26f
    resp_head.protocol  = head.m_protocol;
    resp_head.qos       = QOS_SEND_ALWAYS;
    SMLK_LOGD("dyc send encoded data size is %d ", SMLK_UINT32(encoded.size()) );

    TspServiceApi::getInstance()->SendMsg(resp_head, encoded.data(), encoded.size());
}


/******************************************************************************
 * NAME: OnCanDatachanged
 *
 * DESCRIPTION: can信号变化后触发 更新缓存m_cached_signals的value
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::OnCanDatachanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("valuetest OnCanDatachanged  spn index is %d ", SMLK_UINT32(head.m_seq) );
    const double    *ptr    = reinterpret_cast<const double *>(data.data());

    SMLK_UINT32     index   = head.m_seq;
    bool            valid   = (0 != head.m_protocol);

    auto it = m_cached_signals.find(index); // find spn index
    if ( m_cached_signals.cend() != it ) {
        if ( it->second.Valid() && !valid ) {
            SMLK_LOGD("valuetest signal changed:  VALID ---> INVALID, signal index:  %u  and value is %f", index , *ptr );
        } else if ( !it->second.Valid() && valid ) {
            SMLK_LOGD("valuetest signal changed:  INVALID ---> VALID, signal index: %u  and value is %f", index , *ptr );
        }
        SMLK_LOGD("valuetest OnCanDatachanged  spn get value is %lf ", *ptr );

        it->second = *ptr; // SignalFormat operator = is update m_val
        if ( !valid ) {
            SMLK_LOGW("now is invalid");
            it->second.Invalid();
        }
    } else {
        SMLK_LOGE("unregisted data change notification for index: %u", index);
    }
}

/******************************************************************************
 * NAME: onNewDbcSet
 *
 * DESCRIPTION: 新的dbc下发后 需要重新解析加载规则
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::onNewDbcSet(IN MsgHead &head)
{
    SMLK_UINT32 dbc_index = head.m_extra;
    SMLK_LOGD("onNewDbcSet dbc_index is %d", dbc_index);

    // load new dbc config
    // m_timer->ReStart();
    {
        SMLK_LOGD("TRY GET MUUTEX m_message_decode_map_mutex");
        std::lock_guard<std::mutex> lk_message(m_message_decode_map_mutex);
        SMLK_LOGD("GET MUUTEX m_message_decode_map_mutex !!");

        std::lock_guard<std::mutex> lock_siganl(m_dbc_sample_siganls_mutex);
        ConfigLoader::NewLoadConfigFromDBCFile(dbc_index ,\
                                        &m_dbc_sample_signals_map, \
                                        &g_message_decode_map);
    }
    InitVehicleDataPool();
}

/******************************************************************************
 * NAME: OnDataChanged
 *
 * DESCRIPTION: 从vehicle获取的统计数据
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::OnDataChanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    const double    *ptr    = reinterpret_cast<const double *>(data.data());
    SMLK_UINT32     index   = head.m_seq;
    bool            valid   = 0 != head.m_protocol;

    auto it = m_vec_signals.find(index);
    if ( m_vec_signals.cend() != it ) {
        if ( it->second.Valid() && !valid ) {
            SMLK_LOGD("[dycDebug] data changed:  VALID ---> INVALID, signal index: %u", index);
        } else if ( !it->second.Valid() && valid ) {
            SMLK_LOGD("[dycDebug] data changed:  INVALID ---> VALID, signal index: %u", index);
        }
        it->second = *ptr;
        if ( !valid ) {
            it->second.Invalid();
        }
    } else {
        SMLK_LOGE("unregisted data change notification for index: %u", index);
    }
    m_signal_day_vehicle_distance = m_vec_signals.find(vehicle::HeaderDataID::StaDayTrip)->second;
    m_signal_total_vehicle_distance = m_vec_signals.find(vehicle::HeaderDataID::StaTrip)->second;
    m_signal_ecu_total_distacne = m_vec_signals.find(vehicle::HeaderDataID::StaTotalECU)->second;
    m_signal_day_fuel_used = m_vec_signals.find(vehicle::HeaderDataID::StaDayFuel)->second;
    m_signal_total_fuel_used = m_vec_signals.find(vehicle::HeaderDataID::StaTripFuel)->second;
    m_signal_ecu_total_fuel_used = m_vec_signals.find(vehicle::HeaderDataID::StaTotalFuel)->second;
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
void DynamicCollectionService::OnCanBusWakeup(IN MsgHead &/*head*/)
{
    if (m_internal_flags & InternalFlags::SL_FLAG_DIRECT_CONTROL) {
        SMLK_LOGD(" OnCanBusWakeup and direct control!");
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
void DynamicCollectionService::OnCanBusSleep(IN MsgHead &/*head*/)
{
    SMLK_LOGI("CollectionService::OnCanBusSleep");

    if (m_internal_flags & InternalFlags::SL_FLAG_DIRECT_CONTROL) {
        SMLK_LOGI(" can sleep and direct manage , report data now");
        // check direct can network
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
 *
 * NOTE:
 *****************************************************************************/
void DynamicCollectionService::OnIgnOff(IN MsgHead &/*head*/)
{
    SMLK_LOGI(" =========OnIgnOff===========");
    m_timer->Stop();
    // indirect network control : system ign off, directly report once and do not report anymore
    if (0 == (m_internal_flags & InternalFlags::SL_FLAG_DIRECT_CONTROL)) {
    }
    // m_timer->Stop();

    SMLK_UINT32 now_second = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

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
void DynamicCollectionService::OnIgnOn(IN MsgHead &/*head*/)
{
    SMLK_LOGI(" =========OnIgnOn===========");
    m_timer->Start();

#if 0
    auto now    = std::chrono::system_clock::now();
    auto ms     = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    if ( m_internal_flags & InternalFlags::SL_FLAG_SYSTIME_SYN ) {
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
    }
    // make sure we almost always trigger the 1s timer at the 500ms of each seconds
    if ( ms <= SL_PERIOD_TRIGGER_POINT_MS ) {
        std::this_thread::sleep_for(std::chrono::milliseconds(SL_PERIOD_TRIGGER_POINT_MS-ms));
    } else {
        // here since we may sleep up to 1s, so if we just got a term signal during this time, we may stop until 1 second later
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 + SL_PERIOD_TRIGGER_POINT_MS -ms));
    }

    // m_timer->Start();
#endif

}

void DynamicCollectionService::OnMcuStateNotify(IN MsgHead &head)
{
    switch ( head.m_seq ) {
        case MCU::SL_CAN_STATUS_NOTIFY:
            {
              if (CanBusState::CAN_NORMAL == head.m_extra) {
                if ( 0 == (m_internal_flags & InternalFlags::SL_FLAG_CAN_NORMAL) ) {
                    m_internal_flags |= InternalFlags::SL_FLAG_CAN_NORMAL;

                }
              } else if (CanBusState::CAN_SLEEP == head.m_extra) {
                if ( 0 != (m_internal_flags & InternalFlags::SL_FLAG_CAN_NORMAL) ) {
                    m_internal_flags &= ~InternalFlags::SL_FLAG_CAN_NORMAL;
                    OnCanBusSleep(head);
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
                        OnIgnOn(head);
                    }
                } else {
                    if ( 0 != (m_internal_flags & InternalFlags::SL_FLAG_IGNON) ) {
                        m_internal_flags &= ~InternalFlags::SL_FLAG_IGNON;
                        OnIgnOff(head);
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
void DynamicCollectionService::HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    // SMLK_LOGW("HandleEvent id is head.m_id %x", head.m_id);

    switch ( head.m_id )
    {
        case M_SL_MSGID_TSP_DYC_REQ:
            OnDycConfig(head, data);
            break;
        case M_SL_MSGID_CAN_EVENT:
        {
            // CanFormatData::CanFrame *frame = (CanFormatData::CanFrame *)data.data();
            // CanFrameAceept(frame);
            break;
        }
        case M_SL_MSGID_TIMER_NOTIFY:
        {
            OnTimer(head);
            break;
        }
        case M_SL_MSGID_COMPRESS_RSP:
            OnZipNotify(head, data);
            break;
        case M_SL_MSGID_SIGNAL_DATA_CHANGE:
            OnCanDatachanged(head, data);
        case M_SL_MSGID_DATA_CHANGED:
            OnDataChanged(head, data);
            break;
        case M_SL_MSGID_LOCATOIN_NTF:
            OnLocation(head, data);
            break;
        case M_SL_MSGID_DBC_FILE_UPDATE_NOTIFY:
            onNewDbcSet(head);
            // m_timer->ReStart();
            break;
        case M_SL_MSGID_MCU_STATE_NOTIFY:
            OnMcuStateNotify(head);
        default:
            // TODO
            break;
    }
}

void DynamicCollectionService::OnCompress(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnCompress");
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
    SMLK_UINT32 uploadid = head.m_extra;
    MsgHead resp_head(M_SL_MSGID_COMPRESS_RSP, head.m_seq, head.m_protocol, 0x01, (head.m_length & 0xFF00) | Compress::SL_RC_SUCCESS, uploadid);
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
    SMLK_LOGD("compress data size is[%d]", sz);

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
    SMLK_LOGD("OnCompress end");

    Post(resp_head, std::move(compressed));
}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION: 根据tsp协议处理数据格式
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:根据每个dbc的采集频率和采集周期和上报间隔决定是否采集和上传
 *****************************************************************************/
std::uint32_t DynamicCollectionService::Encode(std::vector<SMLK_UINT8> &data, IN DBC_INDEX &index)
{
    SMLK_LOGD("ON Encode");

    data.clear();
    PUSH_U2(m_sampling_frequency); // 采集频率

    m_crompression = 0x01;         //是否压缩
    PUSH_U1(m_crompression);

    PUSH_U1(0x00);                 // 保留4字节
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U1(0x00);

    std::size_t length_index = data.size();

    // content length word
    PUSH_U1(0x00);
    PUSH_U1(0x00);

    m_signal_total_vehicle_distance.SetValid();
    m_signal_ecu_total_fuel_used.SetValid();
    PUSH_U2(m_signal_day_vehicle_distance.Encoded());

    PUSH_U4(m_signal_total_vehicle_distance.Encoded());
    PUSH_U4(m_signal_ecu_total_distacne.Encoded());
    PUSH_U2(m_signal_day_fuel_used.Encoded());
    PUSH_U4(m_signal_total_fuel_used.Encoded());
    PUSH_U4(m_signal_ecu_total_fuel_used.Encoded());
    auto it_config  = m_dbc_config.find(index);

    SMLK_UINT8 data_type = DATATYPE_PHY_DATA; // hard code modify
    if (m_dbc_config.end() != it_config) {
        data_type = it_config->second.getUploadDataType();
    }
    PUSH_U1(data_type); // 数据类型 物理值

    SMLK_UINT8  num_index = data.size(); // 内容行数
    PUSH_U2(0x00);

    std::stringstream   ss;

    auto it_frames = m_sampled_frame_signals.find(index);
    if (m_sampled_frame_signals.end() == it_frames) {
        SMLK_LOGD("m_dbc_sample_signals_map not find index");
        return SL_EFAILED;
    }
    // std::unordered_map<DBC_INDEX, std::vector<std::unordered_map<SPN_INDEX, SignalFormat>>>    m_sampled_frame_signals;

    auto map = it_frames->second.front();

    SMLK_LOGD("sample size is %d", it_frames->second.front().size());
    if ( 0 == it_frames->second.front().size() ) {
        return SL_EFAILED;
    }

    // 填充采集信号名称
    auto dbc_signal_index = m_dbc_sample_signals_map.find(index);
    if (m_dbc_sample_signals_map.end() == dbc_signal_index) {
        SMLK_LOGD("m_dbc_sample_signals_map not find index");
        return SL_EFAILED;
    }
    for ( auto const &pair: dbc_signal_index->second ) {
        ss << pair << ",";
    }

    std::string  head(ss.str());
    // 头  time + 纬度 + 经度 + 速度 + 定位状态 + 高度 + 信号别名SPN
    // 内容信号值最多保留三位
    // 行与行之间回车换行，每行之间元素,分割
    // 去除尾部","
    head.pop_back();

    data.insert(data.end(), head.begin(), head.end());

    // PUSH_U1('\r');
    // PUSH_U1('\n');
    ss.clear();
    ss.str(std::string());

    // SMLK_LOGD("!!!!!!start add signal value !!!!!");
    if( m_sampled_frame_signals.empty()) {
        SMLK_LOGD("m_sampled_frame_signals is empty!!! ");
        return SL_EFAILED;
    }

    auto it_index_frames = m_sampled_frame_signals.find(index);
    // std::vector<std::unordered_map<SPN_INDEX, vehicle::SignalVariant>> = it_index_frames->second
    auto iter = it_index_frames->second.cbegin();
    // 遍历每一帧
    // fix frame num
    SMLK_UINT16 frame_size = it_index_frames->second.size() + 1;
    data[num_index]      = (SMLK_UINT8)((frame_size >> 8) & 0xFF);
    data[num_index+1]    = (SMLK_UINT8)(frame_size & 0xFF);

    for ( ; it_index_frames->second.cend() != iter ; ++iter) {
        // 填充每行内容
        // 头
        auto map = *iter;
        for ( auto const &tmp_index: dbc_signal_index->second ) {
            auto pair = map.find(tmp_index);
            if ( map.end() != pair) {
                // SMLK_LOGD("valuetest add one sign %d", pair->second.GetIndex());
                // SMLK_LOGD("valuetest this signal vaild is %d", pair.second.Valid());
                SMLK_UINT32 id =  pair->second.GetIndex();
                if ( vehicle::HeaderDataID::GNSSTimeStamp == id ) {
                    std::time_t timestamp = pair->second.Val();
                    //struct tm tt = *std::localtime(&timestamp);
                    struct tm tt;
                    localtime_r(&timestamp, &tt);
                    ss << std::setw(2) << std::setfill('0') << std::dec << (tt.tm_year - 100)%100;
                    ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_mon + 1;
                    ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_mday;
                    ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_hour;
                    ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_min;
                    ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_sec;
                } else if ( vehicle::HeaderDataID::GNSSLocatedStatus == id ) {
                    if ( pair->second.Valid() ) {

                        SMLK_UINT16 status = 0;
                        auto it_tmp = m_gnss_signals.find(vehicle::HeaderDataID::GNSSHeading);
                        if (!it_tmp->second.Valid()) {
                            // hirain range: 0 < data < 360
                            // tsp range: 0-359
                            if (359 <= it_tmp->second.Val() ) {
                                status |= 359 & 0x1FF;
                            } else {
                                status |= 0 & 0x1FF;
                            }
                        } else {
                            status |= it_tmp->second.Encoded() & 0x1FF;
                        }

                        it_tmp = m_gnss_signals.find(vehicle::HeaderDataID::GNSSLatitude);
                        if ( it_tmp->second.Val() < 0  ) {
                            status |= ((SMLK_UINT32)1) << 14;
                        }

                        it_tmp = m_gnss_signals.find(vehicle::HeaderDataID::GNSSLongitude);
                        if ( it_tmp->second.Val() < 0 ) {
                            status |= ((SMLK_UINT32)1) << 15;
                        }
                            ss << std::dec << (int)status;
                    } else {
                            ss << "FFFF";
                    }

                } else if ( vehicle::HeaderDataID::GNSSLatitude == id ) {
                    ss << std::fixed << std::setprecision(6) << ((pair->second.Val() < 0) ? - pair->second.Val() : pair->second.Val());
                } else if ( vehicle::HeaderDataID::GNSSLongitude == id ) {
                    ss << std::fixed << std::setprecision(6) << ((pair->second.Val() < 0) ? - pair->second.Val() : pair->second.Val());
                } else if (vehicle::HeaderDataID::GNSSAltitude == id) {
                    if ( pair->second.Valid() ) {
                        ss << std::fixed << std::setprecision(2) << pair->second.Val();
                    } else {
                        ss << "FFFF";
                    }
                } else if (vehicle::HeaderDataID::GNSSSpeed == id) {
                    if ( pair->second.Valid() ) {
                        if ( pair->second.Val() > pair->second.Max() || pair->second.Val() < pair->second.Min() ) {
                            ss << "FF";
                        } else {
                            ss << std::fixed << std::setprecision(2) << pair->second.Val();
                        }
                    } else {
                        ss << "FF";
                    }
                } else {
                    if ( pair->second.Valid() ) {
                        if (DATATYPE_PHY_DATA == data_type) {
                            if ( pair->second.IsInteger() ) {
                                // make sure the formated string is a integer like string
                                ss << std::dec << (std::int64_t)pair->second.Val();
                                // SMLK_LOGD("data % ld", (std::int64_t)pair.second.Val());
                            } else {
                                // ss << std::fixed << std::setprecision(2) << signal.Val();
                                auto val = ((double)((std::int64_t)(pair->second.Val() * 100.0))) * 0.01 + 0.001;
                                ss << std::fixed << std::setprecision(2) << val;
                                // SMLK_LOGD("data %ld ", (std::int64_t)pair.second.Val());
                            }
                        } else { // raw data
                            auto val = ((double)((std::int64_t)(pair->second.CalculateRawData() * 100.0))) * 0.01 + 0.001;
                            ss << std::fixed << std::setprecision(2) << val;
                        }
                    } else {
                        ss << "FF";         // out of range
                    }
                }
                ss << ",";
            }
        }

        std::string row(ss.str());
        // remove the last ','
        row.pop_back();
        // SMLK_LOGD("push current string is %s", row.c_str());

        PUSH_U1('\r');
        PUSH_U1('\n');

        data.insert(data.end(), row.begin(), row.end());
        ss.clear();
        ss.str(std::string());
    }
    // try to fix the length parameter
    SMLK_UINT16 length = data.size() - length_index - sizeof(SMLK_UINT16);

    data[length_index]      = (SMLK_UINT8)((length >> 8) & 0xFF);
    data[length_index+1]    = (SMLK_UINT8)(length & 0xFF);

    return SL_SUCCESS;

}
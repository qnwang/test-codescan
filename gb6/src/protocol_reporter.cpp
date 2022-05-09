/******************************************************************************
*
*  Copyright (C) 2021 SmartLink
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
#include <sstream>
#include <iomanip>
#include <ctime>
#include <sys/time.h>
#include <unistd.h>
#include <report_signals.h>
#include <smlk_log.h>
#include <smlk_timer_new.h>
#include <protocol_reporter.h>
#include <message_body_1239.h>
#include <message_body_17691.h>
#include <vehicle_data_index_def.h>

#include "smlk_hsm_api.h"

using namespace smartlink;

std::mutex Reporter::m_sendMtx;

namespace InternalFlag {
    static const SMLK_UINT32    SL_MASK_IGNON           = 0x00000001;           // IGN ON/OFF mask
    static const SMLK_UINT32    SL_MASK_CONNECTED       = 0x00000002;           // async connection ready mask
    static const SMLK_UINT32    SL_MASK_LOGIN           = 0x00000004;           // login mask
    static const SMLK_UINT32    SL_MASK_TIME_SYNC       = 0x00000008;           // system time sync mask
};

namespace TimerFlag {
    static const SMLK_UINT32    SL_MASK_TIMER_RT        = 0x00000001;           
    static const SMLK_UINT32    SL_MASK_TIMER_RT_RUN    = 0x00000002;           
    static const SMLK_UINT32    SL_MASK_TIMER_SUP       = 0x00000004;           
    static const SMLK_UINT32    SL_MASK_TIMER_SUP_RUN   = 0x00000008;           
};

namespace SystemTime {
    static const SMLK_UINT32    SL_MASK_SYNC                = 0x00000001;
};

namespace UDS {
namespace DID {
    static const SMLK_UINT32    SL_DID_VIN                  = 0x00000001;
    static const SMLK_UINT32    SL_DID_CVN                  = 0x00000002;
    static const SMLK_UINT32    SL_DID_SCIN                 = 0x00000003;
    static const SMLK_UINT32    SL_DID_IUPR                 = 0x00000004;
    static const SMLK_UINT32    SL_DID_OBD                  = 0x00000008;
    static const SMLK_UINT32    SL_DID_DP                   = 0x00000010;
}
}

// message types define
namespace MessageType {
    static const SMLK_UINT8     SL_VEHICLE_LOGIN            = 0x01;             // vehicle login
    static const SMLK_UINT8     SL_VEHICLE_LOGOUT           = 0x02;             // vehicle logout
    static const SMLK_UINT8     SL_REPORT                   = 0x03;             // report
    static const SMLK_UINT8     SL_ENGINE_INFO              = 0xF1;             // engine info
    static const SMLK_UINT8     SL_OBD_INFO                 = 0xF2;             // OBD info
    static const SMLK_UINT8     SL_PROTOCOL_MSG_LOGIN       = 0xF3;             // protocol vehicle login
    static const SMLK_UINT8     SL_PROTOCOL_MSG_LOGOUT      = 0xF4;             // protocol vehicle logout
    static const SMLK_UINT8     SL_PROTOCOL_MSG_REPORT      = 0xF5;             // protocol message for report
    static const SMLK_UINT8     SL_PROTOCOL_MSG_SUPPLEMENT  = 0xF6;             // protocol message for suppment
};

namespace {
    static const int            SL_QUEUE_GET_TIMEOUT_MS     = 200;              // timeout millisecond to get item from queue
    // static const int            SL_SESSION_EXPIRED_MS       = 3000;             // session expired

    static const SMLK_UINT32    SL_EVENT_TIMER_NOTIFY   = 0xFF000001;           // event timer notify
    static const SMLK_UINT32    SL_EVENT_DAY_CHANGE_NOTIFY  = 0xFF000002;
    static const SMLK_UINT32    SL_EVENT_LOCAT_NOTIFY   = 0xFF000004;           // event location notify
    static const SMLK_UINT32    SL_EVENT_DATAC_NOTIFY   = 0xFF000005;           // event data changed notify
    static const SMLK_UINT32    SL_SYSTEM_TIME_NOTIFY   = 0xFF000007;           // event system time sync notify
    static const SMLK_UINT32    SL_EVENT_DID_UPDATE     = 0xFF000008;           // event DID value update
    static const SMLK_UINT32    SL_EVENT_DTC_UPDATE     = 0xFF000009;           // event DTC value update

    static const SMLK_UINT32    SL_TIMER_GENRIC_1S      = 0x00000001;
    static const SMLK_UINT32    SL_TIMER_SUPPLEMENT     = 0x00000002;           // supplement data send timer id, this should not trigger before OBD query finished since they use the same shared timer
    static const SMLK_UINT32    SL_TIMER_ACTIVATION     = 0x00000003;           

    static const SMLK_UINT32    SL_PERIOD_TRIGGER_POINT_MS  = 100;              // period trigger ms point (0 ~ 999)
    
    static const SMLK_UINT32    SL_REPORT_PERIOD_DEF        = 10;               // default report report period 10 s
    static const SMLK_UINT32    SL_REPORT_START_INDEX_DEF   = 12;               // default report engine information index
    
    static const SMLK_UINT32    SL_REPORT_FIRST_LOGIN_DEF   = 120;              // default do first login after 12000 ms (100ms as unit)
    static const SMLK_UINT32    SL_SAMPLING_PERIOD_DEF      = 10;               // default sampling report period 1000 ms (100ms as unit)
    static const SMLK_UINT32    SL_SUPPLEMENT_PERIOD_DEF    = 10;               // default supplement report period 1000 ms (100ms as unit)
    static const SMLK_UINT32    SL_RECONNECT_PERIOD_DEF     = 100;              // default reconnect period 10 s (100ms as unit)
    static const SMLK_UINT32    SL_EXPIRED_RECORD_PERIOD    = 36000;            // default expired record clear period 3600s (100ms as unit)

    static const SMLK_UINT32    SL_RECORD_VALID_DURATION    = 691200;           // deafult record valid duration 8 days 691200s
    static const SMLK_INT32     SL_TIMEZONE_OFFSET          = 28800;            // default timezone offset seconds (+8:00   8 * 3600)
};

static void get_bcd_timestamp(SMLK_BCD *buffer) {
    struct tm   tt{};
    time_t now = time(0);
    localtime_r(&now, &tt);

    tt.tm_year  -= 100;
    tt.tm_mon   += 1;

    buffer[0]   = (SMLK_UINT8)tt.tm_year;
    buffer[1]   = (SMLK_UINT8)tt.tm_mon;
    buffer[2]   = (SMLK_UINT8)tt.tm_mday;
    buffer[3]   = (SMLK_UINT8)tt.tm_hour;
    buffer[4]   = (SMLK_UINT8)tt.tm_min;
    buffer[5]   = (SMLK_UINT8)tt.tm_sec;

}

static SMLK_UINT32 bcd_timestamp_2_id(const SMLK_BCD *timestamp) {
    /*
     * bit  0 ~  5  second
     * bit  6 ~ 11  minute
     * bit 12 ~ 16  hour
     * bit 17 ~ 21  day
     * bit 22 ~ 25  month
     * bit 26 ~ 31  year
     */
    return (SMLK_UINT32)(((timestamp[0] >> 4) & 0xF) * 10 + (timestamp[0] & 0xF)) << 26             /* year since 2000 */
         | (SMLK_UINT32)(((timestamp[1] >> 4) & 0xF) * 10 + (timestamp[1] & 0xF)) << 22             /* month */
         | (SMLK_UINT32)(((timestamp[2] >> 4) & 0xF) * 10 + (timestamp[2] & 0xF)) << 17             /* day */
         | (SMLK_UINT32)(((timestamp[3] >> 4) & 0xF) * 10 + (timestamp[3] & 0xF)) << 12             /* hour */
         | (SMLK_UINT32)(((timestamp[4] >> 4) & 0xF) * 10 + (timestamp[4] & 0xF)) << 6              /* minute */
         | (SMLK_UINT32)(((timestamp[5] >> 4) & 0xF) * 10 + (timestamp[5] & 0xF));                  /* second */
}


static void timestamp_2_bcd(IN std::time_t tt, INOUT SMLK_BCD *BCD, IN std::size_t sz) {
    if ( (nullptr == BCD) || (sz != M_GB_17691_SZ_TIMESTAMP) ) {
        SMLK_LOGE("TimeStamp2BCD FAIL!!!");
        return;
    }
    std::tm tm;
    localtime_r(&tt, &tm);
    // check whether src time is invalid
    std::time_t local_time;
    if (tt == 0) {
        SMLK_LOGD("timestamp src = 0");
        std::memset(BCD, 0x0, sz);
        return;
    } else {
        localtime_r(&tt, &tm);
    }

    tm.tm_year -= 100;      // years since 2000
    tm.tm_mon += 1;

    BCD[0]   = (SMLK_UINT8)tm.tm_year;
    BCD[1]   = (SMLK_UINT8)tm.tm_mon;
    BCD[2]   = (SMLK_UINT8)tm.tm_mday;
    BCD[3]   = (SMLK_UINT8)tm.tm_hour;
    BCD[4]   = (SMLK_UINT8)tm.tm_min;
    BCD[5]   = (SMLK_UINT8)tm.tm_sec;
}


/**
 * @brief Construct a new Reporter:: Reporter object
 * 
 * @param host 
 * @param port 
 * @param protocol 协议类型
 * @param platform 企业/地方
 */
Reporter::Reporter(IN std::string &host, IN SMLK_UINT16 port, IN SMLK_UINT8 protocol, IN SMLK_UINT8 platform)
    : m_service_flags(0)
    , m_internal_flags(0)
    , m_timer_flags(0)
    , m_vin_flags(0)
    , m_seq(0)
    , m_time_sync_gap(0)
    , m_protocol_version(protocol)
    , m_platform_type(platform)
    , m_reconnect_period(SL_RECONNECT_PERIOD_DEF)
    , m_reprot_first_login(SL_REPORT_FIRST_LOGIN_DEF)
    , m_sampling_period(SL_SAMPLING_PERIOD_DEF)
    , m_supplement_period(SL_SUPPLEMENT_PERIOD_DEF)
    , m_report_period(SL_REPORT_PERIOD_DEF)
    , m_expired_record_clear_period(SL_EXPIRED_RECORD_PERIOD)
    , m_report_index(0)
    , m_reprot_index_engine(SL_REPORT_START_INDEX_DEF)
    , m_port(port)
    , m_host(host)
    , m_vin("0")
    , m_chipid("")
    , m_db_dir(".")
    , m_todisk(false)
    , m_entnet_state(2)
    , m_locnet_state(4)
    , m_actstate(M_HJ_1239_ACTIVATED_RESERVE)
    , is_ng(false)
    , is_hev(false)
{
    if (m_protocol_version == 0x01) {
        m_taskName = "BJ";
        m_report_period = 10;
    } else if (m_protocol_version == 0x04) {
        m_taskName = "TS";
        m_report_period = 1;
    } else if (m_protocol_version == 0x06) {
        m_taskName = "HJ";
        m_report_period = 10;
    }
    SMLK_LOGI("[%s] Reporter::Reporter", m_taskName.c_str());
}


/**
 * @brief Destroy the Reporter:: Reporter object
 * 
 */
Reporter::~Reporter()
{
    Stop();
}


/**
 * @brief Init the service
 * 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::Init()
{
    SMLK_LOGI("[%s] Init enter", m_taskName.c_str());
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_service_flags & IService::Initialized ) {
        SMLK_LOGD("[%s] Already initialized, do nothing", m_taskName.c_str());
        return SL_SUCCESS;
    }

    // init all the cached signal
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::TachographVehicleSpeed),                 std::forward_as_tuple(std::ref(signal::TachographVehicleSpeed)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::BarometricPressure),                     std::forward_as_tuple(std::ref(signal::BarometricPressure)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::ActualEnginePercentTorque),              std::forward_as_tuple(std::ref(signal::ActualEnginePercentTorque)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::NominalFrictionPercentTorque),           std::forward_as_tuple(std::ref(signal::NominalFrictionPercentTorque)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::EngineSpeed),                            std::forward_as_tuple(std::ref(signal::EngineSpeed)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::EngineFuelRate),                         std::forward_as_tuple(std::ref(signal::EngineFuelRate)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::Aftertreatment1IntakeNOx),               std::forward_as_tuple(std::ref(signal::Aftertreatment1IntakeNOx)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::Aftertreatment1OutletNOx),               std::forward_as_tuple(std::ref(signal::Aftertreatment1OutletNOx)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::CatalystTankLevel),                      std::forward_as_tuple(std::ref(signal::CatalystTankLevel)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::EngineInletAirMassFlowRate),             std::forward_as_tuple(std::ref(signal::EngineInletAirMassFlowRate)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::Aftertreatment1SCRIntakeTemperature),    std::forward_as_tuple(std::ref(signal::Aftertreatment1SCRIntakeTemperature)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::Aftertreatment1SCROutletTemperature),    std::forward_as_tuple(std::ref(signal::Aftertreatment1SCROutletTemperature)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::Aftertreatment1DPFDifferentialPressure), std::forward_as_tuple(std::ref(signal::Aftertreatment1DPFDifferentialPressure)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::EngineCoolantTemperature),               std::forward_as_tuple(std::ref(signal::EngineCoolantTemperature)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::FuelLevel),                              std::forward_as_tuple(std::ref(signal::FuelLevel)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::TotalVehicleDistance),                   std::forward_as_tuple(std::ref(signal::TotalVehicleDistance)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::LocatedStatus),                          std::forward_as_tuple(std::ref(signal::LocatedStatus)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::Longitude),                              std::forward_as_tuple(std::ref(signal::Longitude)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::Latitude),                               std::forward_as_tuple(std::ref(signal::Latitude)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::TimeStamp),                              std::forward_as_tuple(std::ref(signal::TimeStamp)));

    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::TWCUpstreamO2Sensor),                    std::forward_as_tuple(std::ref(signal::TWCUpstreamO2Sensor)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::TWCDownstreamO2Sensor),                  std::forward_as_tuple(std::ref(signal::TWCDownstreamO2Sensor)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::TWCDownstreamNOxSensor),                 std::forward_as_tuple(std::ref(signal::TWCDownstreamNOxSensor)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::TWCTempSensor),                          std::forward_as_tuple(std::ref(signal::TWCTempSensor)));
    
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::MotorSpeed),                             std::forward_as_tuple(std::ref(signal::MotorSpeed)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::DriveInsPowerPer),                       std::forward_as_tuple(std::ref(signal::DriveInsPowerPer)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::PowerBatVolt),                           std::forward_as_tuple(std::ref(signal::PowerBatVolt)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::PowerBatCurr),                           std::forward_as_tuple(std::ref(signal::PowerBatCurr)));
    m_cached_signals.emplace(std::piecewise_construct, std::forward_as_tuple(vehicle::SignalID::PowerBatSOC),                            std::forward_as_tuple(std::ref(signal::PowerBatSOC)));

    for ( auto &pair : m_cached_signals ) {
        auto const it = vehicle::g_vehicle_signal_index.find(pair.first);
        if ( vehicle::g_vehicle_signal_index.cend() == it ) {
            continue;
        }
        m_cached_index_signals.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(it->second),
            std::forward_as_tuple(std::ref(pair.second))
        );
    }


    if (0x01 == m_protocol_version || 0x04 == m_protocol_version)
    {
        auto spVehicleLogin = std::make_shared<MessageVehicleLogin17691>();
        auto spOBD = std::make_shared<MessageOBDInfo17691>();

        m_vehicle_login = spVehicleLogin;
        m_vehicle_logout = std::make_shared<MessageVehicleLogout17691>();
        m_report = std::make_shared<MessageReport17691>();
        m_engine_info = std::make_shared<MessageEngineInfo17691>();
        m_obd_info = spOBD;
    }
    else if (0x06 == m_protocol_version)
    {
        auto spVehicleLogin = std::make_shared<MessageVehicleLogin1239>();
        auto spOBD = std::make_shared<MessageOBDInfo1239>();

        m_vehicle_login = spVehicleLogin;
        m_vehicle_logout = std::make_shared<MessageVehicleLogout1239>();
        m_report = std::make_shared<MessageReport1239>();
        m_obd_info = spOBD;

        if (is_ng)
        {
            m_engine_info = std::make_shared<MessageEngineTwcInfo1239>();
        }
        else
        {
            m_engine_info = std::make_shared<MessageEngineDpfScrInfo1239>();
        }
        if (is_hev)
        {
            m_engine_hev_info = std::make_shared<MessageEngineHevInfo1239>();
        }
    }

    auto timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 100>>(
        [this](SMLK_UINT32 index)
        {
            if (0 == index % SL_SAMPLING_PERIOD_DEF)
            {
                // SMLK_LOGD("[%s] [G6TimerDebug] send SL_TIMER_GENRIC_1S !", m_taskName.c_str());
                MsgHead head(SL_EVENT_TIMER_NOTIFY, index, 0, 0, 0, SL_TIMER_GENRIC_1S);
                Post(head, nullptr, 0);
            }
        });

    auto timer_sup = std::make_shared<LoopTimer<std::chrono::milliseconds, 100>>(
        [this](SMLK_UINT32 index)
        {
            if (0 == index % SL_SUPPLEMENT_PERIOD_DEF)
            {
                // SMLK_LOGD("[%s] [G6TimerDebug] send SL_TIMER_SUPPLEMENT !", m_taskName.c_str());
                MsgHead head(SL_EVENT_TIMER_NOTIFY, index, 0, 0, 0, SL_TIMER_SUPPLEMENT);
                Post(head, nullptr, 0);
            }
        });

    std::stringstream   ss;
    ss << "record_gb17691_v";
    ss << std::uppercase << std::setw(2) << std::setfill('0') << m_taskName;
    ss << ".sqlite3";

    SMLK_LOGD("[%s] init reocrder %s", m_taskName.c_str(), ss.str().c_str());

    auto recorder = std::make_shared<Recorder>(m_db_dir, ss.str());
    if ( SL_SUCCESS != recorder->Init() ) {
        SMLK_LOGE("[%s] fail to init recorder instance", m_taskName.c_str());
        return SL_EFAILED;
    }

    std::stringstream   ss_sup;
    ss_sup << "record_gb17691_sup_v";
    ss_sup << std::uppercase << std::setw(2) << std::setfill('0') << m_taskName;
    ss_sup << ".sqlite3";

    SMLK_LOGD("[%s] init reocrder %s", m_taskName.c_str(), ss_sup.str().c_str());

    auto recorder_sup = std::make_shared<Recorder>(m_db_dir, ss_sup.str());
    if ( SL_SUCCESS != recorder_sup->Init() ) {
        SMLK_LOGE("[%s] fail to init recorder_sup instance", m_taskName.c_str());
        return SL_EFAILED;
    }

    m_comm17691 = new common17691(m_taskName);

    // Socket 初始化
    m_TcpClient17691 = new TCPClient(m_taskName);
    // 注册SOCKET状态的回调
    m_TcpClient17691->RegisterSocketStateCB(std::bind(&Reporter::HandleSocketConnState, this, std::placeholders::_1, std::placeholders::_2));
    // 首次连接一定要连接成功程序才可以继续执行
    while (m_TcpClient17691->m_connectState != normal)
    {
        SMLK_LOGE("[%s] m_connectState=%d", m_taskName.c_str(), m_TcpClient17691->m_connectState);

        m_TcpClient17691->initParam(m_host.c_str(), m_port);
        int ret = m_TcpClient17691->connect();
        SMLK_LOGI("[%s] Reporter::InitLogic() ret=%d", m_taskName.c_str(), ret);

        if (ret != 0)
        {
            sleep(3);
        }
    }

    m_timer_rt      = timer;
    m_timer_sup     = timer_sup;

    m_recorder      = recorder;
    m_recorder_sup  = recorder_sup;

    m_service_flags |= IService::Initialized;

    SMLK_LOGI("[%s] Init finish", m_taskName.c_str());
    return SL_SUCCESS;
}


/**
 * @brief Start the service
 * 
 * @return SMLK_UINT32 SL_SUCCESS/SL_EFAILED
 */
SMLK_UINT32 Reporter::Start()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_service_flags & IService::Running ) {
        SMLK_LOGI("[%s] service already started, do nothing", m_taskName.c_str());
        return SL_SUCCESS;
    }

    if ( !(m_service_flags & IService::Initialized) ) {
        SMLK_LOGE("[%s] service not initialized", m_taskName.c_str());
        return SL_EFAILED;
    }

    if ( SL_SUCCESS != m_recorder->Open() ) {
        SMLK_LOGE("[%s] fail to open recorder DB", m_taskName.c_str());
        return SL_EFAILED;
    }

    if ( SL_SUCCESS != m_recorder_sup->Open() ) {
        SMLK_LOGE("[%s] fail to open recorder_sup DB", m_taskName.c_str());
        return SL_EFAILED;
    }

    m_timer_rt->Start();
    m_timer_rt->Reset();
    m_timer_flags |= TimerFlag::SL_MASK_TIMER_RT;
    SMLK_LOGI("[%s] [G6TimerDebug] start and reset rt timer", m_taskName.c_str());

    m_timer_sup->Start();
    m_timer_sup->Reset();
    m_timer_flags |= TimerFlag::SL_MASK_TIMER_SUP;
    SMLK_LOGI("[%s] [G6TimerDebug] start and reset sup timer", m_taskName.c_str());

    m_service_flags |= IService::Running;

    m_thread = std::thread(
        [this]() {
            SMLK_LOGI("[%s] main work thread started", m_taskName.c_str());
            while ( 0 == (m_service_flags & IService::Stopping) ) {
                m_events.get(SL_QUEUE_GET_TIMEOUT_MS)();
            }
            SMLK_LOGI("[%s] main work thread exit!!!", m_taskName.c_str());
        }
    );

    return SL_SUCCESS;
}


/**
 * @brief Stop the service
 * 
 */
void Reporter::Stop()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_service_flags & IService::Running ) {
        SMLK_LOGI("[%s] told to stop, mark as stopping", m_taskName.c_str());
        m_service_flags |= IService::Stopping;

        if ( m_thread.joinable() ) {
            m_thread.join();
        }

        SMLK_LOGI("[%s] [G6TimerDebug] stop rt timer", m_taskName.c_str());
        m_timer_flags &= ~TimerFlag::SL_MASK_TIMER_RT;
        m_timer_rt->Stop();

        SMLK_LOGI("[%s] [G6TimerDebug] stop sup timer", m_taskName.c_str());
        m_timer_flags &= ~TimerFlag::SL_MASK_TIMER_SUP;
        m_timer_sup->Stop();

        SMLK_LOGI("[%s] stop tcp client", m_taskName.c_str());
        if (m_TcpClient17691 != nullptr) {
            m_TcpClient17691->close();
        }
        SMLK_LOGI("[%s] close recorder", m_taskName.c_str());
        m_recorder->Close();
        SMLK_LOGI("[%s] close recorder_sup", m_taskName.c_str());
        m_recorder_sup->Close();

        if ( m_internal_flags & InternalFlag::SL_MASK_LOGIN ) {
            m_internal_flags &= ~InternalFlag::SL_MASK_LOGIN;
        }

        m_service_flags |= ~(IService::Running | IService::Stopping);

        SMLK_LOGI("[%s] stopped", m_taskName.c_str());
    }
}


/**
 * @brief check the service running status
 * 
 * @return true if the service already running
 * @return false 
 */
bool Reporter::IsRunning() const
{
    return  m_service_flags & IService::Running;
}


/**
 * @brief 设置产地信息
 * 
 * @param loc : 0 长春; 1 青岛
 * @return SMLK_UINT32 SL_SUCCESS/SL_ESUPPORT
 */
SMLK_UINT32 Reporter::SetProductLoc(IN SMLK_UINT8 loc)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_service_flags & IService::Initialized) ) {
        return SL_ESUPPORT;
    }

    m_loc = loc;
    return SL_SUCCESS;
}


/**
 * @brief 设置车辆类型
 * 
 * @param gas 是否为 天然气
 * @param hev 是否为 混动
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::SetReportInfo(IN SMLK_BOOL gas, IN SMLK_BOOL hev)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_service_flags & IService::Initialized) ) {
        return SL_ESUPPORT;
    }

    is_ng = gas;
    is_hev = hev;

    if ((m_protocol_version == 0x06) && is_hev) 
        m_report_period = SL_REPORT_PERIOD_DEF * 2;

    return SL_SUCCESS;
}


/**
 * @brief 设置车辆 vin
 * 
 * @param vin 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::SetVIN(IN std::string &vin)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_service_flags & IService::Initialized) ) {
        return SL_ESUPPORT;
    }

    m_vin = vin;

    return SL_SUCCESS;
}


/**
 * @brief 设置加密芯片 id
 * 
 * @param chipid 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::SetChipID(IN std::string &chipid)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (0 != (m_service_flags & IService::Initialized))
    {
        return SL_ESUPPORT;
    }

    m_chipid = chipid;
    SMLK_LOGE("[%s] Reporter::SetChipID %s", m_taskName.c_str(), m_chipid.c_str());

    return SL_SUCCESS;
}


/**
 * @brief 同步激活状态
 * 
 * @param state 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::SetActStatus(IN SMLK_UINT32 state)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (0 != (m_service_flags & IService::Initialized))
    {
        return SL_ESUPPORT;
    }

    m_actstate = state;
    SMLK_LOGE("[%s] Reporter1239_07::SetActStatus %d", m_taskName.c_str(), state);

    return SL_SUCCESS;
}


/**
 * @brief 设置 iccid
 * 
 * @param iccid 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::SetICCID(IN std::string &iccid)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_service_flags & IService::Initialized) ) {
        return SL_ESUPPORT;
    }

    m_iccid = iccid;

    return SL_SUCCESS;
}


/**
 * @brief 设置授时补偿时间
 * 
 * @param timesyncgap 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::SetTimeSyncGap(IN SMLK_UINT64 timesyncgap) {
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_service_flags & IService::Running) ) {
        return SL_ESUPPORT;
    }

    m_internal_flags |= InternalFlag::SL_MASK_TIME_SYNC;

    m_time_sync_gap = timesyncgap;

    return SL_SUCCESS;
}


/**
 * @brief set recorder data dir
 * 
 * @param dir 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::SetRecorderDir(IN std::string &dir)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_service_flags & IService::Running) ) {
        return SL_ESUPPORT;
    }

    if ( 0 == dir.length() ) {
        return SL_EFAILED;
    }

    m_db_dir = dir;

    return SL_SUCCESS;
}


/**
 * @brief set save raw data to file
 * 
 * @param save 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::SetIsSave(IN SMLK_BOOL save)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_service_flags & IService::Running) ) {
        return SL_ESUPPORT;
    }
    m_todisk = save;
    SMLK_LOGE("[%s] Reporter::SetIsSave %d", m_taskName.c_str(), m_todisk);

    return SL_SUCCESS;
}


/**
 * @brief set telephony status, here the telephony actually means the modem module, 
 *        and we always just foucsed on data service only
 * 
 * @param info 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter::SetTelephonyState(IN ProtocolReporter::telephony::Info &info)
{
    SMLK_LOGD("[%s] Reporter::SetTelephonyState() channel %d, type %d, state %d", m_taskName.c_str(), info.channel, info.type, info.status);

    if ((info.channel == 1)) {
        if (info.status == 1) {
            SMLK_LOGI("[%s] TelephonyNotify: we lost the connection", m_taskName.c_str());

            if (m_TcpClient17691->m_connectState == normal) {
                m_TcpClient17691->close();
            }
            if (m_internal_flags & InternalFlag::SL_MASK_LOGIN) {
                m_internal_flags &= ~InternalFlag::SL_MASK_LOGIN;
            }

            SMLK_LOGI("[%s] [G6TimerDebug] reset sup timer", m_taskName.c_str());
            m_timer_sup->Reset();

        } else if (info.status == 2) {
            SMLK_LOGI("[%s] TelephonyNotify: we get new connection", m_taskName.c_str());
        }
    }
    return SL_SUCCESS;
}

/**
 * @brief 备电切主电通知
 * 
 */
void Reporter::SwitchToMainPower () {
}

/**
 * @brief 主电切备电通知
 * 
 */
void Reporter::SwitchToStandyPower () {
    SMLK_LOGI("[%s] Reporter::Reporter", m_taskName.c_str());
    m_comm17691->saveSequence(false);
}


/**
 * @brief IGN ON notification
 * 
 */
void Reporter::IgnON()
{
    m_comm17691->setIgnState(M_GB_17691_IGN_ON);


    if (0 == (m_internal_flags & InternalFlag::SL_MASK_IGNON))
    {
        m_internal_flags |= InternalFlag::SL_MASK_IGNON;

        m_report_index = 0;

        if (m_TcpClient17691->m_connectState != normal) {
            m_TcpClient17691->connect();
        }

        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
        auto target = SL_PERIOD_TRIGGER_POINT_MS;

        // make sure we almost always trigger the 1s timer at the 500ms of each seconds
        if (ms <= target)
        {
            std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(target - ms));
        } else {
            // here since we may sleep up to 1s, so if we just got a term signal during this time, we may stop until 1 second later
            std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 + target - ms));
        }

        SMLK_LOGI("[%s] [G6TimerDebug] restart rt timer", m_taskName.c_str());
        m_timer_rt->ReStart();
    }
}


/**
 * @brief IGN OFF notification
 * 
 */
void Reporter::IgnOFF()
{
    m_comm17691->setIgnState(M_GB_17691_IGN_OFF);

    if (0 != (m_internal_flags & InternalFlag::SL_MASK_IGNON))
    {
        SMLK_LOGI("[%s] IGN state changed from ON to OFF", m_taskName.c_str());
        m_internal_flags &= ~InternalFlag::SL_MASK_IGNON;

        DoVehicleLogout();

        SMLK_LOGI("[%s] [G6TimerDebug] reset rt timer", m_taskName.c_str());
        m_timer_rt->Reset();

        m_TcpClient17691->close();
        if (m_internal_flags & InternalFlag::SL_MASK_LOGIN) {
            m_internal_flags &= ~InternalFlag::SL_MASK_LOGIN;
        }
    }
}


/**
 * @brief update diagnostic protocol
 * 
 * @param result 
 * @param protocol 
 */

void Reporter::UpdateDiagProtocol(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &protocol)
{
    Post(MsgHead(SL_EVENT_DID_UPDATE, UDS::DID::SL_DID_DP, 0, 0, 0, result), protocol);
}


/**
 * @brief update diagnostic vin
 * 
 * @param result 
 * @param vin 
 */
void Reporter::UpdateVIN(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &vin)
{
    std::stringstream   ss;
    for ( auto ch : vin ) {
        ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
    }

    SMLK_LOGD("[%s] UpdateVIN %s", m_taskName.c_str(), ss.str().c_str());
    Post(MsgHead(SL_EVENT_DID_UPDATE, UDS::DID::SL_DID_VIN, 0, 0, 0, result), vin);
}


/**
 * @brief update diagnostic scin
 * 
 * @param result 
 * @param scin 
 */
void Reporter::UpdateSCIN(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &scin)
{
    Post(MsgHead(SL_EVENT_DID_UPDATE, UDS::DID::SL_DID_SCIN, 0, 0, 0, result), scin);
}


/**
 * @brief update obd
 * 
 * @param result 
 * @param obd 
 */
void Reporter::UpdateOBD(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &obd)
{
    Post(MsgHead(SL_EVENT_DID_UPDATE, UDS::DID::SL_DID_OBD, 0, 0, 0, result), obd);
}


/**
 * @brief update cvn
 * 
 * @param result 
 * @param cvn 
 */
void Reporter::UpdateCVN(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &cvn)
{
    std::stringstream   ss;
    for ( auto ch : cvn ) {
        ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
    }

    SMLK_LOGD("[%s] UpdateCVN %s", m_taskName.c_str(), ss.str().c_str());
    Post(MsgHead(SL_EVENT_DID_UPDATE, UDS::DID::SL_DID_CVN, 0, 0, 0, result), cvn);
}


/**
 * @brief update iupr
 * 
 * @param result 
 * @param iupr 
 */
void Reporter::UpdateIUPR(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &iupr)
{
    Post(MsgHead(SL_EVENT_DID_UPDATE, UDS::DID::SL_DID_IUPR, 0, 0, 0, result), iupr);
}


/**
 * @brief update DTC
 * 
 * @param result 
 * @param dtc 
 */
void Reporter::UpdateDTC(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &dtc)
{
    Post(MsgHead(SL_EVENT_DTC_UPDATE, 0, 0, 0, 0, result), dtc);
}


/**
 * @brief update signal value
 * 
 * @param index 
 * @param val 
 * @param valid 
 */
void Reporter::UpdateSignalVal(IN SMLK_UINT32 index, IN SMLK_DOUBLE val, SMLK_BOOL valid)
{
    Post(
        MsgHead(SL_EVENT_DATAC_NOTIFY, index, valid ? 0x01 : 0x00, 0, sizeof(val), 0),
        reinterpret_cast<const SMLK_UINT8 *>(&val),
        sizeof(val)
    );
}


/**
 * @brief update GNSS information
 * 
 * @param info GNSS information
 */
void Reporter::UpdateGNSS(IN ProtocolReporter::GNSS::Info &info)
{
    Post(
        MsgHead(SL_EVENT_LOCAT_NOTIFY, 0, 0, 0, sizeof(info), 0),
        reinterpret_cast<const SMLK_UINT8 *>(&info),
        sizeof(info)
    );
}


/**
 * @brief 
 * 
 */
void Reporter::UpdateDayChange()
{
    Post(MsgHead(SL_EVENT_DAY_CHANGE_NOTIFY, 0, 0, 0, 0, 0), nullptr, 0);
}

/**
 * @brief post message the service main queue, the function can be called by the service its self or other module
 * 
 * @param head message head which has the basic information about this message
 * @param ptr message data
 * @param sz data length
 */
void Reporter::Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz)
{
    m_events.emplace(
        head,
        ptr,
        sz,
        std::bind(&Reporter::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}


/**
 * @brief post message the service main queue, the function can be called by the service its self or other module
 * 
 * @param head 
 * @param data 
 */
void Reporter::Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_events.emplace(
        head,
        data,
        std::bind(&Reporter::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}


/**
 * @brief post message the service main queue, the function can be called by the service its self or other module
 * 
 * @param head 
 * @param data 
 */
void Reporter::Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data)
{
    m_events.emplace(
        head,
        std::move(data),
        std::bind(&Reporter::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}


/**
 * @brief post message the service main queue, the function can be called by the service its self or other module
 * 
 * @param head message head which has the basic information about this message
 * @param data message data
 */
void Reporter::HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_id ) {
        case SL_EVENT_TIMER_NOTIFY:
            OnTimer(head.m_extra, head.m_seq);
            break;
        case SL_EVENT_DATAC_NOTIFY:
            OnDataChanged(head, data);
            break;
        case SL_EVENT_LOCAT_NOTIFY:
            OnLocationNotify(head, data);
            break;
        case SL_SYSTEM_TIME_NOTIFY:
            OnSystemTimeNotify(head, data);
            break;
        case SL_EVENT_DID_UPDATE:
            OnDIDNotify(head, data);
            break;
        case SL_EVENT_DTC_UPDATE:
            OnDTCNotify(head, data);
            break;
        case SL_EVENT_DAY_CHANGE_NOTIFY:
            OnDayChangeNotify();
            break;
        default:
            break;
    }
}


/**
 * @brief post message the service main queue, the function can be called by the service its self or other module
 *        this is a move operation, so after this call, the function caller should never use the data again
 * @param head message head which has the basic information about this message
 * @param data message data
 */
void Reporter::OnDataChanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_UINT32     index   = head.m_seq;
    bool            valid   = 0 != head.m_protocol;

    auto it = m_cached_index_signals.find(index);
    if ( m_cached_index_signals.end() == it ) {
        SMLK_LOGE("[%s] unregisted data change notification for index:   %u", m_taskName.c_str(), index);
        return;
    } else {
        if ( it->second.Valid() && !valid ) {
            SMLK_LOGW("[%s] data changed:  VALID ---> INVALID, signal index: %u", m_taskName.c_str(), index);
        } else if ( !it->second.Valid() && valid ) {
            // SMLK_LOGD("[%s] data changed:  INVALID ---> VALID, signal index: %u", m_taskName.c_str(), index);
        }
    }

    it->second = *(reinterpret_cast<const double *>(data.data()));
    if ( !valid ) {
        it->second.Invalid();
    }
}


/**
 * @brief location information notification handler
 * 
 */
void Reporter::OnLocationNotify(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    if ( sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8) > data.size() ) {
        SMLK_LOGE("[%s] invalid data size", m_taskName.c_str());
        return;
    }

    ProtocolReporter::GNSS::Info *inf = (ProtocolReporter::GNSS::Info *) data.data();

    auto &flags  = m_cached_signals.find(vehicle::SignalID::LocatedStatus)->second;
    flags = inf->flags;

    auto &latitude  = m_cached_signals.find(vehicle::SignalID::Latitude)->second;
    latitude = inf->latitude;

    auto &longitude  = m_cached_signals.find(vehicle::SignalID::Longitude)->second;
    longitude = inf->longitude;
}


/**
 * @brief system time notify event handler
 * 
 * @param head message head which has the basic information about this message
 * @param data message data
 */
void Reporter::OnSystemTimeNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    if ( 0 != (head.m_seq & SystemTime::SL_MASK_SYNC) ) {
        m_internal_flags |= InternalFlag::SL_MASK_TIME_SYNC;
    } else {
        m_internal_flags &= ~InternalFlag::SL_MASK_TIME_SYNC;
    }
}


/**
 * @brief did event handler
 * 
 * @param head message head which has the basic information about this message
 * @param data message data
 */
void Reporter::OnDIDNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("[%s] OnDIDNotify head.m_seq %x: head.result %x", m_taskName.c_str(), head.m_seq, head.m_extra);
    auto spOBD = std::static_pointer_cast<MessageOBDInfo>(m_obd_info);
    switch ( head.m_seq ) {
        case UDS::DID::SL_DID_DP:
            {
                SMLK_LOGD("[%s] OnDIDNotify case UDS::DID::SL_DID_DP: enter ", m_taskName.c_str());

                static const std::set<SMLK_UINT8>   valid = {
                    M_GB_17691_OBD_PROTOCOL_ISO15765,
                    M_GB_17691_OBD_PROTOCOL_ISO27154,
                    M_GB_17691_OBD_PROTOCOL_SAEJ1939,
                };
                if(1 == head.m_extra)
                {
                    SMLK_LOGE("[%s] OnDIDNotify case UDS::DID::SL_DID_DP: did result err", m_taskName.c_str());
                    spOBD->m_protocol = M_GB_17691_OBD_PROTOCOL_INVALID;
                }
                else
                {
                    if ( data.size() && (valid.end() != valid.find(data[2])) )
                    {
                        spOBD->m_protocol = data[2];
                    } else {
                        spOBD->m_protocol = M_GB_17691_OBD_PROTOCOL_INVALID;
                    }
                }
            }
            break;
        case UDS::DID::SL_DID_VIN:
            {
                decltype(data.size())   sz = data.size()-2;
                if ( sz > sizeof(spOBD->m_vin) ) {
                    sz = sizeof(spOBD->m_vin);
                }
                if(1 == head.m_extra) {
                    SMLK_LOGE("[%s] OnDIDNotify case UDS::DID::SL_DID_VIN: did result err", m_taskName.c_str());
                    m_vin_flags = 0;
                    std::memset(spOBD->m_vin, 0, M_GB_17691_SZ_VIN);
                } else {
                    m_vin_flags = 1;
                    std::memcpy(spOBD->m_vin, data.data()+2, sz);

                    std::stringstream stream;
                    stream << spOBD->m_vin;
                    m_vin = stream.str().substr(0,17);
                }
            }
            break;
         case UDS::DID::SL_DID_SCIN:
            {
                decltype(data.size())   sz = data.size()-2;
                if ( sz > sizeof(spOBD->m_scin) ) {
                    sz = sizeof(spOBD->m_scin);
                }
                for (int i = 0; i < M_GB_17691_SZ_SCIN; i++) {
                    spOBD->m_scin[i] = 0x30;
                }

                if(1 == head.m_extra) {
                    SMLK_LOGE("[%s] OnDIDNotify case UDS::DID::SL_DID_SCIN: did result err", m_taskName.c_str());
                } else {
                    std::memcpy(spOBD->m_scin, data.data()+2, sz);
                }
            }
            break;
        case UDS::DID::SL_DID_CVN:
            {
                decltype(data.size()) sz = data.size() - 2;
                if (sz > sizeof(spOBD->m_cvn) / 2) {
                    sz = sizeof(spOBD->m_cvn) / 2;
                }
                for (int i = 0; i < M_GB_17691_SZ_CVN; i++) {
                    spOBD->m_cvn[i] = 0x30;
                }

                if (1 == head.m_extra) {
                    SMLK_LOGE("[%s] OnDIDNotify case UDS::DID::SL_DID_CVN: did result err", m_taskName.c_str());
                } else {     
                    for (int i = 0; i < sz; i++) {
                        spOBD->m_cvn[2 * i] = ((((*(data.data() + 2 + i)) >> 4) & 0xf) <= 9) ? ((((*(data.data() + 2 + i)) >> 4) & 0xf) + '0') : ((((*(data.data() + 2 + i)) >> 4) & 0xf) + 55);
                        spOBD->m_cvn[2 * i + 1] = (((*(data.data() + 2 + i)) & 0xf) <= 9) ? (((*(data.data() + 2 + i)) & 0xf) + '0') : (((*(data.data() + 2 + i)) & 0xf) + 55);
                    }
                }
            }
            break;
        case UDS::DID::SL_DID_IUPR:
            {
                decltype(data.size())   sz = data.size()-2;
                if ( sz > sizeof(spOBD->m_iupr) ) {
                    sz = sizeof(spOBD->m_iupr);
                }
                std::memset(spOBD->m_iupr, 0, M_GB_17691_SZ_IUPR);
                if(1 == head.m_extra) {
                    SMLK_LOGE("[%s] OnDIDNotify case UDS::DID::SL_DID_IUPR: did result err", m_taskName.c_str());
                } else {
                    std::memcpy(spOBD->m_iupr, data.data()+2, sz);
                }
            }
            break;
        case UDS::DID::SL_DID_OBD:
            {
                SMLK_LOGD("[%s] OnDIDNotify case UDS::DID::SL_DID_OBD: enter ", m_taskName.c_str());
                if(1 == head.m_extra) {
                    SMLK_LOGE("[%s] OnDIDNotify case UDS::DID::SL_DID_OBD: did result err", m_taskName.c_str());
                    spOBD->m_mil_status = 0xFE;
                    spOBD->m_diag_support_status = 0;
                    spOBD->m_diag_ready_status = 0;
                } else {
                    if ( data.size() != (sizeof(SMLK_UINT8) + sizeof(SMLK_UINT16) + sizeof(SMLK_UINT16)) )  {
                        SMLK_LOGE("[%s] invalid data size:   %u", data.size());
                        return;
                    }
                    std::size_t index = 0;
                    spOBD->m_mil_status = data[index];
                    SMLK_LOGI("[%s] OnDIDNotify case UDS::DID::SL_DID_OBD: spOBD->m_mil_status = %u", m_taskName.c_str(), spOBD->m_mil_status);

                    index += sizeof(SMLK_UINT8);
                    spOBD->m_diag_support_status = (SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1];
                    index += sizeof(SMLK_UINT16);
                    spOBD->m_diag_ready_status  = (SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1];
                }
            }
            break;


        default:
            SMLK_LOGE("[%s] unsupported DID: 0x%08X", m_taskName.c_str(), head.m_seq);
            break;
    }
}


/**
 * @brief dtc event handler
 * 
 * @param head message head which has the basic information about this message
 * @param data message data
 */
void Reporter::OnDTCNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    auto spOBD = std::static_pointer_cast<MessageOBDInfo>(m_obd_info);
    spOBD->m_dtcs.clear();
    spOBD->m_dtc = 0;
    if (1 == head.m_extra) {
        SMLK_LOGE("[%s] OnDTCNotify case UDS::DID::SL_DID_DTC: did result err", m_taskName.c_str());
    } else {
        std::size_t sz = data.size() - 5;
        const SMLK_UINT8 *pCur = data.data() + 5;
        while (sz >= sizeof(SMLK_UINT32) + 1) {
            spOBD->m_dtcs.push_back((SMLK_UINT32)pCur[1] << 24 | (SMLK_UINT32)pCur[2] << 16 | (SMLK_UINT32)pCur[3] << 8 | (SMLK_UINT32)pCur[4]);
            sz -= sizeof(SMLK_UINT32) + 1;
            pCur += sizeof(SMLK_UINT32) + 1;
        }
        spOBD->m_dtc = spOBD->m_dtcs.size();
    }
    SMLK_LOGI("[%s] OnDTCNotify spOBD->m_dtc = 0x%02X ", m_taskName.c_str(),spOBD->m_dtc);
}


/**
 * @brief 
 * 
 */
void Reporter::OnDayChangeNotify()
{
    m_comm17691->initSequence();
}


/**
 * @brief timer event handler
 * 
 * @param id timer id
 * @param seq loop timer trigger seq, valid only while loop timer
 */
void Reporter::OnTimer(IN SMLK_UINT32 id, IN SMLK_UINT32 seq)
{
    switch (id)
    {
    case SL_TIMER_GENRIC_1S:
        OnGenericTimer(seq);
        break;
    case SL_TIMER_SUPPLEMENT:
        OnSupplementTimer(seq);
        break;
    default:
        SMLK_LOGE("[%s] unsupported timer id: 0x%08X", m_taskName.c_str(), id);
        break;
    }
}

/**
 * @brief generic timer event handler
 * 
 * @param seq timer trigger seq, valid only while loop timer
 */
void Reporter::OnGenericTimer(IN SMLK_UINT32 seq)
{
    // SMLK_LOGI("[%s] OnGenericTimer enter ", m_taskName.c_str());

    if (0 == (m_internal_flags & InternalFlag::SL_MASK_LOGIN))
    {
        if (0 == (seq % SL_REPORT_FIRST_LOGIN_DEF))
        {
            if ((m_TcpClient17691->m_connectState == normal) && m_vin_flags)
            {
                SMLK_LOGW("[%s] not in login state, do first vehicle login !!!", m_taskName.c_str());
                DoVehicleLogin();
            }
        }
    }

    if (0 != (m_internal_flags & InternalFlag::SL_MASK_IGNON))
    {
        if ((m_report_index >= m_reprot_index_engine) && m_vin_flags)
        {
            SamplingEngineInfo();
        }
        else
        {
            ++m_report_index;
        }

        if (m_TcpClient17691->m_connectState != normal)
        {
            if (0 == (seq % m_reconnect_period))
            {
                SMLK_LOGI("[%s] IGN ON but not connected, retry to connect to remote server", m_taskName.c_str());
                future = std::async(std::launch::async, [this]()
                                    { m_TcpClient17691->connect(); });
            }
        }
    }

    if (0 == (seq % m_expired_record_clear_period))
    {
        ClearExpiredRecord();
    }
}

/**
 * @brief supplement timer event handler
 * 
 */
void Reporter::OnSupplementTimer(IN SMLK_UINT32 /*seq*/)
{
    // 网络未连接 || 未登录
    if ((m_TcpClient17691->m_connectState != normal) || (0 == (m_internal_flags & InternalFlag::SL_MASK_LOGIN)))
    {
        return;
    }

    if (0x06 == m_protocol_version && M_GB_LOCATION_QINGDAO == m_loc)
    {
        if (M_HJ_1239_ACTIVATED_CHIP_ERROR == m_actstate || M_HJ_1239_ACTIVATED_VIN_ERROR == m_actstate || M_HJ_1239_ACTIVATED_OTHER_ERROR == m_actstate)
        {
            return;
        }
    }

    SMLK_LOGE("[%s] OnSupplementTimer enter", m_taskName.c_str());

    std::vector<std::tuple<SMLK_UINT32, std::vector<SMLK_UINT8>>>   records;
    auto rc = m_recorder_sup->QueryOneRecord(records, 0, SMLK_UINT32(-1), 1);      // pickup one item only

    if ( SL_SUCCESS != rc ) {
        SMLK_LOGE("[%s] fail to query suppliment record", m_taskName.c_str());
    } else {
        if ( 0 == records.size() ) {
            SMLK_LOGI("[%s] [G6TimerDebug] reset sup timer", m_taskName.c_str());
            m_timer_sup->Reset();
        } else {

            std::vector<SMLK_UINT8> body;
            body.clear();
            body.assign(std::get<1>(records[0]).begin(), std::get<1>(records[0]).end());

            SMLK_BCD p_timestamp[6] = {0};
            get_bcd_timestamp(p_timestamp);

            if (0x06 == m_protocol_version) {
                body.erase(body.begin(), body.begin() + 6);
                body.insert(body.begin(), p_timestamp, p_timestamp + 6);

                int ret = 0;
                int n_sign_len = 128;
                unsigned char sign_data[n_sign_len] = {0};
                ret = hsm_seal_sign(TYPE_SM2, sign_data, (size_t *)&n_sign_len, body.data(), body.size(), ALG_MD3, 0, (unsigned char *)m_chipid.data(), m_chipid.size());

                int n_R_sign_len = 32;
                unsigned char R_sign_data[n_R_sign_len] = {0};
                int n_S_sign_len = 32;
                unsigned char S_sign_data[n_S_sign_len] = {0};
                ret = hsm_SM3Sign_Split_RCode_And_SCode(sign_data, n_sign_len, R_sign_data, (size_t *)&n_R_sign_len, S_sign_data, (size_t *)&n_S_sign_len);

                body.push_back(0x20); // R 长度
                for (auto i = 0; i < 32; i++) {
                    body.push_back(R_sign_data[i]);
                }

                body.push_back(0x20); // S 长度
                for (auto i = 0; i < 32; i++) {
                    body.push_back(S_sign_data[i]);
                }
            }

            std::vector<SMLK_UINT8> encoded;
            encoded.clear();
            encoded.reserve(body.size() + sizeof(GB_17691_MsgHead) + 1);

            // 0x23 0x23
            // encoded.push_back(M_GB_17691_MSG_FLAG);
            // encoded.push_back(M_GB_17691_MSG_FLAG);

            // 0x7E 0x7E
            // encoded.push_back(M_HJ_1239_MSG_FLAG);
            // encoded.push_back(M_HJ_1239_MSG_FLAG);

            if (0x06 == m_protocol_version)
            {
                encoded.push_back(M_HJ_1239_MSG_FLAG);
                encoded.push_back(M_HJ_1239_MSG_FLAG);
            }
            else
            {
                encoded.push_back(M_GB_17691_MSG_FLAG);
                encoded.push_back(M_GB_17691_MSG_FLAG);
            }

            encoded.push_back(M_GB_17691_CMD_REISSUE_REPORT);
            encoded.insert(encoded.end(), m_vin.cbegin(), m_vin.cend());
            encoded.push_back(m_protocol_version);
            encoded.push_back(M_GB_17691_ENCRYPTION_NONE);
            SMLK_UINT16 length = (SMLK_UINT16)body.size();
            encoded.push_back((SMLK_UINT8)(length >> 8) & 0xFF);
            encoded.push_back((SMLK_UINT8)(length & 0xFF));
            encoded.insert(encoded.end(), body.cbegin(), body.cend());

            SMLK_UINT8  XOR = 0x00;
            for ( decltype(encoded.size()) index = M_GB_17691_SZ_FLAGS; index < encoded.size(); index++ ) {
                XOR ^= encoded[index];
            }
            encoded.push_back(XOR);

            if (m_todisk)
            {
                std::stringstream strStream;
                time_t timep;
                struct tm *p;
                time(&timep);
                p = localtime(&timep);
                struct timeval ms;
                gettimeofday(&ms, NULL);
                strStream << "[" << m_taskName << "]" << " " << "03 " << "send time:" << p->tm_hour << ":" << p->tm_min << ":"
                          << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
                for (int i = 0; i < encoded.size(); i++)
                {
                    strStream << std::hex << std::setw(2) << std::setfill('0') << (int)encoded[i] << " ";
                }
                system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/gb6/record_gb17691.log").c_str());
            }

            SMLK_UINT32 id = bcd_timestamp_2_id(p_timestamp);

            int ret = HandleSendDataToTSP(encoded.data(), encoded.size());
            if (ret > 0) {
                SMLK_LOGI("[%s] SupplementInfo, send time %d-%d-%d %d:%d:%d, send ret = %d, Send success", m_taskName.c_str(),
                             p_timestamp[0], p_timestamp[1], p_timestamp[2], p_timestamp[3], p_timestamp[4], p_timestamp[5], ret);
                m_recorder->Insert(id, Recorder::Flags::reissue, encoded);
                m_recorder_sup->Delete(std::get<0>(records[0]));
            } else {
                SMLK_LOGI("[%s] SupplementInfo, send time %d-%d-%d %d:%d:%d, send ret = %d, Send failed", m_taskName.c_str(),
                             p_timestamp[0], p_timestamp[1], p_timestamp[2], p_timestamp[3], p_timestamp[4], p_timestamp[5], ret);
            }
        }
    }
}


/**
 * @brief login success event handler
 * 
 */
void Reporter::OnLogin()
{
    m_internal_flags |= InternalFlag::SL_MASK_LOGIN;
    m_comm17691->saveSequence();
    SMLK_LOGI("[%s] VEHICLE LOGIN send SUCCESS", m_taskName.c_str());

    SMLK_LOGI("[%s] [G6TimerDebug] restart sup timer", m_taskName.c_str());
    m_timer_sup->ReStart();
}


/**
 * @brief logout success event handler
 * 
 */
void Reporter::OnLogout()
{
    m_internal_flags &= ~InternalFlag::SL_MASK_LOGIN;
    
    OBD::UdsDtcTriggerInfo dtcInfo;

    dtcInfo.dtc_type = OBD::UdsDtcTriggerType::E_UDS_DTC_TRIGGER_GB_CONNECT_STATUS;
    dtcInfo.val = m_platform_type == 0x01 ? 2 : 4;

    if (m_platform_type == 0x01 && m_entnet_state != dtcInfo.val ){
        m_entnet_state = dtcInfo.val;
        if (!(OBD::DidIpcApi::getInstance()->SendDtcErrCode(dtcInfo))) {
            SMLK_LOGW("SyncDTC ipc failed!!!");
        }
    }

    if (m_platform_type == 0x02 && m_locnet_state != dtcInfo.val ){
        m_locnet_state = dtcInfo.val;
        if (!(OBD::DidIpcApi::getInstance()->SendDtcErrCode(dtcInfo))) {
            SMLK_LOGW("SyncDTC ipc failed!!!");
        }
    }  

    m_vin_flags = 0;
    m_comm17691->saveSequence(false);
    SMLK_LOGI("[%s] VEHICLE LOGOUT send SUCCESS", m_taskName.c_str());
}


/**
 * @brief do vehicle logoin
 * 
 */
void Reporter::DoVehicleLogin()
{
    SMLK_LOGI("[%s] DoVehicleLogin enter", m_taskName.c_str());

    auto spVehicleLogin     = std::static_pointer_cast<MessageVehicleLogin>(m_vehicle_login);

    spVehicleLogin->m_timestamp     = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_time_sync_gap / 1000);
    spVehicleLogin->m_seq           = m_comm17691->getSequence(M_GB_17691_CMD_VEHICLE_LOGIN);
    spVehicleLogin->m_iccid         = m_iccid;

    SMLK_BCD t_timestamp[6] = {0};
    timestamp_2_bcd((std::time_t)spVehicleLogin->m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_UINT32 id = bcd_timestamp_2_id(t_timestamp);

    std::vector<SMLK_UINT8> body;
    body.clear();
    spVehicleLogin->Encode(body);

    std::vector<SMLK_UINT8> encoded;
    encoded.clear();
    encoded.reserve(body.size() + sizeof(GB_17691_MsgHead) + 1);

    // 0x23 0x23
    // encoded.push_back(M_GB_17691_MSG_FLAG);
    // encoded.push_back(M_GB_17691_MSG_FLAG);

    // 0x7E 0x7E
    // encoded.push_back(M_HJ_1239_MSG_FLAG);
    // encoded.push_back(M_HJ_1239_MSG_FLAG);

    if (0x06 == m_protocol_version)
    {
        encoded.push_back(M_HJ_1239_MSG_FLAG);
        encoded.push_back(M_HJ_1239_MSG_FLAG);
    }
    else
    {
        encoded.push_back(M_GB_17691_MSG_FLAG);
        encoded.push_back(M_GB_17691_MSG_FLAG);
    }

    encoded.push_back(M_GB_17691_CMD_VEHICLE_LOGIN);
    encoded.insert(encoded.end(), m_vin.cbegin(), m_vin.cend());
    encoded.push_back(m_protocol_version);
    encoded.push_back(M_GB_17691_ENCRYPTION_NONE);
    SMLK_UINT16 length = (SMLK_UINT16)body.size();
    encoded.push_back((SMLK_UINT8)(length >> 8) & 0xFF);
    encoded.push_back((SMLK_UINT8)(length & 0xFF));
    encoded.insert(encoded.end(), body.cbegin(), body.cend());
    SMLK_UINT8 XOR = 0x00;
    for (decltype(encoded.size()) index = M_GB_17691_SZ_FLAGS; index < encoded.size(); index++) {
        XOR ^= encoded[index];
    }
    encoded.push_back(XOR);

    if (m_todisk)
    {
        std::stringstream strStream;
        time_t timep;
        struct tm *p;
        time(&timep);
        p = localtime(&timep);
        struct timeval ms;
        gettimeofday(&ms, NULL);
        strStream << "[" << m_taskName << "]" << " " << "01 " << "send time:" << p->tm_hour << ":" << p->tm_min << ":"
                  << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
        for (int i = 0; i < encoded.size(); i++)
        {
            strStream << std::hex << std::setw(2) << std::setfill('0') << (int)encoded[i] << " ";
        }
        system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/gb6/record_gb17691.log").c_str());
    }

    OBD::UdsDtcTriggerInfo dtcInfo;

    int ret = HandleSendDataToTSP(encoded.data(), encoded.size());
    if (ret > 0) {
        SMLK_LOGI("[%s] DoVehicleLogin, encode time %d-%d-%d %d:%d:%d, send ret = %d, Send success", m_taskName.c_str(), 
                     t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5], ret);
        m_recorder->Insert(id, Recorder::Flags::login, encoded);

        dtcInfo.dtc_type = OBD::UdsDtcTriggerType::E_UDS_DTC_TRIGGER_GB_CONNECT_STATUS;
        dtcInfo.val = m_platform_type == 0x01 ? 2 : 4;

        if (m_platform_type == 0x01 && m_entnet_state != dtcInfo.val ){
            m_entnet_state = dtcInfo.val;
            if (!(OBD::DidIpcApi::getInstance()->SendDtcErrCode(dtcInfo))) {
                SMLK_LOGW("SyncDTC ipc failed!!!");
            }
        }

        if (m_platform_type == 0x02 && m_locnet_state != dtcInfo.val ){
            m_locnet_state = dtcInfo.val;
            if (!(OBD::DidIpcApi::getInstance()->SendDtcErrCode(dtcInfo))) {
                SMLK_LOGW("SyncDTC ipc failed!!!");
            }
        }        

        OnLogin();
    } else {
        SMLK_LOGI("[%s] DoVehicleLogin, encode time %d-%d-%d %d:%d:%d, send ret = %d, Send failed", m_taskName.c_str(), 
                     t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5], ret);

        dtcInfo.dtc_type = OBD::UdsDtcTriggerType::E_UDS_DTC_TRIGGER_GB_CONNECT_STATUS;
        dtcInfo.val = m_platform_type == 0x01 ? 1 : 3;

        if (m_platform_type == 0x01 && m_entnet_state != dtcInfo.val ){
            m_entnet_state = dtcInfo.val;
            if (!(OBD::DidIpcApi::getInstance()->SendDtcErrCode(dtcInfo))) {
                SMLK_LOGW("SyncDTC ipc failed!!!");
            }
        }

        if (m_platform_type == 0x02 && m_locnet_state != dtcInfo.val ){
            m_locnet_state = dtcInfo.val;
            if (!(OBD::DidIpcApi::getInstance()->SendDtcErrCode(dtcInfo))) {
                SMLK_LOGW("SyncDTC ipc failed!!!");
            }
        }    
    }
}


/**
 * @brief do vehicle logout
 * 
 */
void Reporter::DoVehicleLogout()
{
    SMLK_LOGI("[%s] DoVehicleLogout enter", m_taskName.c_str());

    if ( 0 == (m_internal_flags & InternalFlag::SL_MASK_LOGIN) ) {
        SMLK_LOGW("[%s] not in login state, do nothing!!!", m_taskName.c_str());
        return;
    }

    auto spVehicleLogout    = std::static_pointer_cast<MessageVehicleLogout>(m_vehicle_logout);

    spVehicleLogout->m_timestamp    = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_time_sync_gap / 1000);
    spVehicleLogout->m_seq          = m_comm17691->getSequence(M_GB_17691_CMD_VEHICLE_LOGOUT);      /*logout seq == login seq*/
    
    SMLK_BCD t_timestamp[6] = {0};
    timestamp_2_bcd((std::time_t)spVehicleLogout->m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_UINT32 id = bcd_timestamp_2_id(t_timestamp);

    std::vector<SMLK_UINT8> body;
    body.clear();
    spVehicleLogout->Encode(body);

    std::vector<SMLK_UINT8> encoded;
    encoded.clear();
    encoded.reserve(body.size() + sizeof(GB_17691_MsgHead) + 1);

    // 0x23 0x23
    // encoded.push_back(M_GB_17691_MSG_FLAG);
    // encoded.push_back(M_GB_17691_MSG_FLAG);

    // 0x7E 0x7E
    // encoded.push_back(M_HJ_1239_MSG_FLAG);
    // encoded.push_back(M_HJ_1239_MSG_FLAG);

    if (0x06 == m_protocol_version)
    {
        encoded.push_back(M_HJ_1239_MSG_FLAG);
        encoded.push_back(M_HJ_1239_MSG_FLAG);
    }
    else
    {
        encoded.push_back(M_GB_17691_MSG_FLAG);
        encoded.push_back(M_GB_17691_MSG_FLAG);
    }

    encoded.push_back(M_GB_17691_CMD_VEHICLE_LOGOUT);
    encoded.insert(encoded.end(), m_vin.cbegin(), m_vin.cend());
    encoded.push_back(m_protocol_version);
    encoded.push_back(M_GB_17691_ENCRYPTION_NONE);
    SMLK_UINT16 length = (SMLK_UINT16)body.size();
    encoded.push_back((SMLK_UINT8)(length >> 8) & 0xFF);
    encoded.push_back((SMLK_UINT8)(length & 0xFF));
    encoded.insert(encoded.end(), body.cbegin(), body.cend());
    SMLK_UINT8 XOR = 0x00;
    for (decltype(encoded.size()) index = M_GB_17691_SZ_FLAGS; index < encoded.size(); index++) {
        XOR ^= encoded[index];
    }
    encoded.push_back(XOR);

    if (m_todisk)
    {
        std::stringstream strStream;
        time_t timep;
        struct tm *p;
        time(&timep);
        p = localtime(&timep);
        struct timeval ms;
        gettimeofday(&ms, NULL);
        strStream << "[" << m_taskName << "]" << " " << "04 " << "send time:" << p->tm_hour << ":" << p->tm_min << ":"
                  << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
        for (int i = 0; i < encoded.size(); i++)
        {
            strStream << std::hex << std::setw(2) << std::setfill('0') << (int)encoded[i] << " ";
        }
        system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/gb6/record_gb17691.log").c_str());
    }

    int ret = HandleSendDataToTSP(encoded.data(), encoded.size());
    if (ret > 0) {
        SMLK_LOGI("[%s] DoVehicleLogout, encode time %d-%d-%d %d:%d:%d, send ret = %d, Send success", m_taskName.c_str(), 
                     t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5], ret);
        m_recorder->Insert(id, Recorder::Flags::logout, encoded);
        OnLogout();
    } else {
        SMLK_LOGI("[%s] DoVehicleLogout, encode time %d-%d-%d %d:%d:%d, send ret = %d, Send failed", m_taskName.c_str(), 
                     t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5], ret);
    }
}


/**
 * @brief periodic sampline engine information
 * 
 */
void Reporter::SamplingEngineInfo()
{
    if (0x06 == m_protocol_version && M_GB_LOCATION_QINGDAO == m_loc)
    {
        if (M_HJ_1239_ACTIVATED_CHIP_ERROR == m_actstate || M_HJ_1239_ACTIVATED_VIN_ERROR == m_actstate || M_HJ_1239_ACTIVATED_OTHER_ERROR == m_actstate)
        {
            return;
        }
    }

    auto spReport = std::static_pointer_cast<MessageReport>(m_report);

    if (0 == spReport->m_infos.size())
    {
        auto spOBD = std::static_pointer_cast<MessageOBDInfo>(m_obd_info);
        spOBD->m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_time_sync_gap / 1000);

        spReport->m_infos.emplace_back(MessageReport::InfoType::OBD, spOBD);
    }

    auto spEngine = std::static_pointer_cast<MessageEngineInfo>(m_engine_info);
    spEngine->m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_time_sync_gap / 1000);
    spEngine->m_tachograph_vehicle_speed                        = m_cached_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second;
    spEngine->m_barometric_pressure                             = m_cached_signals.find(vehicle::SignalID::BarometricPressure)->second;
    spEngine->m_actual_engine_percent_torque                    = m_cached_signals.find(vehicle::SignalID::ActualEnginePercentTorque)->second;
    spEngine->m_nominal_friction_percent_torque                 = m_cached_signals.find(vehicle::SignalID::NominalFrictionPercentTorque)->second;
    spEngine->m_engine_speed                                    = m_cached_signals.find(vehicle::SignalID::EngineSpeed)->second;
    spEngine->m_engine_fuel_rate                                = m_cached_signals.find(vehicle::SignalID::EngineFuelRate)->second;
    spEngine->m_aftertreatment_1_intake_nox                     = m_cached_signals.find(vehicle::SignalID::Aftertreatment1IntakeNOx)->second;
    spEngine->m_aftertreatment_1_outlet_nox                     = m_cached_signals.find(vehicle::SignalID::Aftertreatment1OutletNOx)->second;
    spEngine->m_catalyst_tank_level                             = m_cached_signals.find(vehicle::SignalID::CatalystTankLevel)->second;
    spEngine->m_engine_inlet_air_mass_flow_rate                 = m_cached_signals.find(vehicle::SignalID::EngineInletAirMassFlowRate)->second;
    spEngine->m_aftertreatment_1_scr_intake_temperature         = m_cached_signals.find(vehicle::SignalID::Aftertreatment1SCRIntakeTemperature)->second;
    spEngine->m_aftertreatment_1_scr_outlet_temperature         = m_cached_signals.find(vehicle::SignalID::Aftertreatment1SCROutletTemperature)->second;
    spEngine->m_aftertreatment_1_dpf_differential_pressure      = m_cached_signals.find(vehicle::SignalID::Aftertreatment1DPFDifferentialPressure)->second;
    spEngine->m_engine_coolant_temperature                      = m_cached_signals.find(vehicle::SignalID::EngineCoolantTemperature)->second;
    spEngine->m_fuel_level                                      = m_cached_signals.find(vehicle::SignalID::FuelLevel)->second;
    spEngine->m_located_status                                  = m_cached_signals.find(vehicle::SignalID::LocatedStatus)->second;
    spEngine->m_longitude                                       = m_cached_signals.find(vehicle::SignalID::Longitude)->second;
    spEngine->m_latitude                                        = m_cached_signals.find(vehicle::SignalID::Latitude)->second;
    spEngine->m_twc_upstream_o2_sensor                          = m_cached_signals.find(vehicle::SignalID::TWCUpstreamO2Sensor)->second;
    spEngine->m_twc_downstream_o2_sensor                        = m_cached_signals.find(vehicle::SignalID::TWCDownstreamO2Sensor)->second;
    spEngine->m_twc_downstream_nox_sensor                       = m_cached_signals.find(vehicle::SignalID::TWCDownstreamNOxSensor)->second;
    spEngine->m_twc_temperature_sensor                          = m_cached_signals.find(vehicle::SignalID::TWCTempSensor)->second;
    if (m_cached_signals.find(vehicle::SignalID::TotalVehicleDistance)->second.Valid())
    {
        spEngine->m_total_vehicle_distance                      = m_cached_signals.find(vehicle::SignalID::TotalVehicleDistance)->second;
    }

    spReport->m_infos.emplace_back((0x06 == m_protocol_version) && is_ng ? MessageReport::InfoType::Stream_NG : MessageReport::InfoType::Stream, spEngine->Duplicate());

    if ((0x06 == m_protocol_version) && is_hev)
    {
        auto spEngineHev = std::static_pointer_cast<MessageEngineHevInfo>(m_engine_hev_info);
        spEngineHev->m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_time_sync_gap / 1000);
        spEngineHev->m_motor_speed                              = m_cached_signals.find(vehicle::SignalID::MotorSpeed)->second;
        spEngineHev->m_drive_ins_power_per                      = m_cached_signals.find(vehicle::SignalID::DriveInsPowerPer)->second;
        spEngineHev->m_power_bat_volt                           = m_cached_signals.find(vehicle::SignalID::PowerBatVolt)->second;
        spEngineHev->m_power_bat_curr                           = m_cached_signals.find(vehicle::SignalID::PowerBatCurr)->second;
        spEngineHev->m_power_bat_soc                            = m_cached_signals.find(vehicle::SignalID::PowerBatSOC)->second;

        spReport->m_infos.emplace_back(MessageReport::InfoType::Stream_HEV, spEngineHev->Duplicate());
    }

    if (spReport->m_infos.size() > m_report_period)
    {
        spReport->m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_time_sync_gap / 1000);
        spReport->m_seq = m_comm17691->getSequence(M_GB_17691_CMD_REALTIME_REPORT);

        SMLK_BCD t_timestamp[6] = {0};
        timestamp_2_bcd((std::time_t)spReport->m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
        SMLK_UINT32 id = bcd_timestamp_2_id(t_timestamp);

        std::vector<SMLK_UINT8> body;
        body.clear();
        spReport->Encode(body);

        std::vector<SMLK_UINT8> r_body;
        r_body.clear();
        r_body.assign(body.begin(), body.end());

        if (0x06 == m_protocol_version) {
            int ret = 0;
            int n_sign_len = 128;
            unsigned char sign_data[n_sign_len] = {0};
            ret = hsm_seal_sign(TYPE_SM2, sign_data, (size_t *)&n_sign_len, body.data(), body.size(), ALG_MD3, 0, (unsigned char *)m_chipid.data(), m_chipid.size());

            int n_R_sign_len = 32;
            unsigned char R_sign_data[n_R_sign_len] = {0};
            int n_S_sign_len = 32;
            unsigned char S_sign_data[n_S_sign_len] = {0};
            ret = hsm_SM3Sign_Split_RCode_And_SCode(sign_data, n_sign_len, R_sign_data, (size_t *)&n_R_sign_len, S_sign_data, (size_t *)&n_S_sign_len);

            body.push_back(0x20); // R 长度
            for (auto i = 0; i < 32; i++) {
                body.push_back(R_sign_data[i]);
            }

            body.push_back(0x20); // S 长度
            for (auto i = 0; i < 32; i++) {
                body.push_back(S_sign_data[i]);
            }
        }

        std::vector<SMLK_UINT8> encoded;
        encoded.clear();
        encoded.reserve(body.size() + sizeof(GB_17691_MsgHead) + 1);

        // 0x23 0x23
        // encoded.push_back(M_GB_17691_MSG_FLAG);
        // encoded.push_back(M_GB_17691_MSG_FLAG);

        // 0x7E 0x7E
        // encoded.push_back(M_HJ_1239_MSG_FLAG);
        // encoded.push_back(M_HJ_1239_MSG_FLAG);

        if (0x06 == m_protocol_version)
        {
            encoded.push_back(M_HJ_1239_MSG_FLAG);
            encoded.push_back(M_HJ_1239_MSG_FLAG);
        }
        else
        {
            encoded.push_back(M_GB_17691_MSG_FLAG);
            encoded.push_back(M_GB_17691_MSG_FLAG);
        }

        encoded.push_back(M_GB_17691_CMD_REALTIME_REPORT);
        encoded.insert(encoded.end(), m_vin.cbegin(), m_vin.cend());
        encoded.push_back(m_protocol_version);
        encoded.push_back(M_GB_17691_ENCRYPTION_NONE);
        SMLK_UINT16 length = (SMLK_UINT16)body.size();
        encoded.push_back((SMLK_UINT8)(length >> 8) & 0xFF);
        encoded.push_back((SMLK_UINT8)(length & 0xFF));
        encoded.insert(encoded.end(), body.cbegin(), body.cend());
        SMLK_UINT8 XOR = 0x00;
        for (decltype(encoded.size()) index = M_GB_17691_SZ_FLAGS; index < encoded.size(); index++)
        {
            XOR ^= encoded[index];
        }
        encoded.push_back(XOR);

        if (m_todisk)
        {
            std::stringstream strStream;
            time_t timep;
            struct tm *p;
            time(&timep);
            p = localtime(&timep);
            struct timeval ms;
            gettimeofday(&ms, NULL);
            strStream << "[" << m_taskName << "]"
                      << " "
                      << "02 "
                      << "send time:" << p->tm_hour << ":" << p->tm_min << ":"
                      << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
            for (int i = 0; i < encoded.size(); i++)
            {
                strStream << std::hex << std::setw(2) << std::setfill('0') << (int)encoded[i] << " ";
            }
            system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/gb6/record_gb17691.log").c_str());
        }


        int ret = HandleSendDataToTSP(encoded.data(), encoded.size());
        if (ret > 0) {
            SMLK_LOGI("[%s] SamplingEngineInfo, encode time %d-%d-%d %d:%d:%d, send ret = %d, Send success", m_taskName.c_str(),
                      t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5], ret);
            m_recorder->Insert(id, Recorder::Flags::realtime, encoded);
        } else {
            SMLK_LOGI("[%s] SamplingEngineInfo, encode time %d-%d-%d %d:%d:%d, send ret = %d, Send failed", m_taskName.c_str(),
                      t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5], ret);
            m_recorder_sup->Insert(id, Recorder::Flags::realtime, r_body);
        }

        spReport->m_infos.clear();
    }
}


/**
 * @brief clear expired record
 * 
 */
void Reporter::ClearExpiredRecord()
{
    SMLK_LOGI("[%s] clear expired record", m_taskName.c_str());

    auto tt     = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    if ( (0 == (m_internal_flags & InternalFlag::SL_MASK_TIME_SYNC))
      || (tt < SL_RECORD_VALID_DURATION) ) {
        // time not synchronization
        SMLK_UINT32     total;
        auto rc = m_recorder->QueryTotal(total);
        if ( SL_SUCCESS != rc ) {
            SMLK_LOGE("[%s] fail to query total record items", m_taskName.c_str());
            return;
        }

        if ( total > SL_RECORD_VALID_DURATION ) {
            m_recorder->DeleteAhead(total - SL_RECORD_VALID_DURATION);
        }

        auto rc_sup = m_recorder_sup->QueryTotal(total);
        if ( SL_SUCCESS != rc_sup ) {
            SMLK_LOGE("[%s] fail to query total record items", m_taskName.c_str());
            return;
        }

        if ( total > SL_RECORD_VALID_DURATION ) {
            m_recorder_sup->DeleteAhead(total - SL_RECORD_VALID_DURATION);
        }

    } else {
        tt -= SL_RECORD_VALID_DURATION;
        struct tm target;
        std::memcpy(&target, std::localtime(&tt), sizeof(target));

        target.tm_year  -= 100;
        target.tm_mon   += 1;

        SMLK_BCD    buffer[M_GB_17691_SZ_TIMESTAMP];

        buffer[0]   = (((SMLK_UINT8)(target.tm_year / 10) & 0x0F) << 4) | (((SMLK_UINT8)(target.tm_year % 10)) & 0x0F);
        buffer[1]   = (((SMLK_UINT8)(target.tm_mon  / 10) & 0x0F) << 4) | (((SMLK_UINT8)(target.tm_mon  % 10)) & 0x0F);
        buffer[2]   = (((SMLK_UINT8)(target.tm_mday / 10) & 0x0F) << 4) | (((SMLK_UINT8)(target.tm_mday % 10)) & 0x0F);
        buffer[3]   = (((SMLK_UINT8)(target.tm_hour / 10) & 0x0F) << 4) | (((SMLK_UINT8)(target.tm_hour % 10)) & 0x0F);
        buffer[4]   = (((SMLK_UINT8)(target.tm_min  / 10) & 0x0F) << 4) | (((SMLK_UINT8)(target.tm_min  % 10)) & 0x0F);
        buffer[5]   = (((SMLK_UINT8)(target.tm_sec  / 10) & 0x0F) << 4) | (((SMLK_UINT8)(target.tm_sec  % 10)) & 0x0F);

        auto id = bcd_timestamp_2_id(buffer);
        SMLK_LOGD("[%s] try to delete recorder id less than: 0x%08u", m_taskName.c_str(), id);
        m_recorder->DeleteLessThan(id);
        m_recorder_sup->DeleteLessThan(id);
    }
}


/**
 * @brief TSP 连接状态回调
 * 
 * @param task 
 * @param state 
 * @return SMLK_INT32 
 */
SMLK_INT32 Reporter::HandleSocketConnState(std::string &task, SMLK_INT32 state)
{
    if (state == normal)
    {
        SMLK_LOGD("[%s] Async: we got a connection notificaiton, m_vin_flags %d ", m_taskName.c_str(), m_vin_flags);
        m_comm17691->initSequence();

        if (m_vin_flags == 1)
        {
            if (0x06 == m_protocol_version)
            {
                if (M_GB_LOCATION_CHANGCHUN == m_loc)
                {
                    DoVehicleLogin();
                }
                else if (M_GB_LOCATION_QINGDAO == m_loc)
                {
                    if (M_HJ_1239_ACTIVATED_RESERVE == m_actstate || M_HJ_1239_ACTIVATED_SUCCESS == m_actstate || M_HJ_1239_ACTIVATED_ING == m_actstate)
                    {
                        DoVehicleLogin();
                    }
                }
            }
            else
            {
                DoVehicleLogin();
            }
        }
    }

    if (state == disconnect)
    {
        SMLK_LOGE("[%s] Async err: we lost the connection", m_taskName.c_str());
        if (m_TcpClient17691->m_connectState == normal)
        {
            m_TcpClient17691->close();
        }
        if (m_internal_flags & InternalFlag::SL_MASK_LOGIN)
        {
            m_internal_flags &= ~InternalFlag::SL_MASK_LOGIN;
        }
        if (m_timer_flags & TimerFlag::SL_MASK_TIMER_RT)
        {
            SMLK_LOGI("[%s] [G6TimerDebug] reset sup timer", m_taskName.c_str());
            m_timer_sup->Reset();
        }
    }

    if (state == notInit)
    {
    }

    return 0;
}

/**
 * @brief 数据发送接口
 * 
 * @param data 
 * @param length 
 * @return SMLK_INT32 
 */
SMLK_INT32 Reporter::HandleSendDataToTSP(SMLK_UINT8 *data, SMLK_UINT32 length)
{
    std::lock_guard<std::mutex> locker(m_sendMtx);
    SMLK_INT32 ret = -1;
    if (m_TcpClient17691->m_connectState == normal) {
        // SMLK_LOGD("[%s] Reporter::HandleSendData2TSP32960() sendMsg", m_taskName.c_str());
        ret = m_TcpClient17691->sendMsg(data, length);
    } else {
        // SMLK_LOGD("[%s] Reporter::HandleSendData2TSP32960() Connect Error!!! ", m_taskName.c_str() );
    }
    return ret;
}
  

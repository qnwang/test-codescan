/*
 * @Author: T
 * @Date: 2022-04-07 13:59:48
 * @LastEditTime: 2022-05-06 18:07:29
 * @LastEditors: taoguanjie taoguanjie@smartlink.com.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /Workspace/tbox2.0/source/apps/gb6/src/protocol_reporter_1239 copy.cpp
 */
#include <set>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <sys/time.h>
#include <unistd.h>
#include <report_signals.h>
#include <smlk_log.h>
#include <smlk_timer_new.h>
#include <protocol_reporter_1239_07.h>
#include <message_body_1239.h>
#include <vehicle_data_index_def.h>

#include "smlk_hsm_api.h"
#include "result_handler.h"

using namespace smartlink;

std::mutex Reporter1239_07::m_sendMtx;
std::mutex Reporter1239_07::m_actMtx;
std::condition_variable Reporter1239_07::cv;

namespace InternalFlag
{
    static const SMLK_UINT32 SL_MASK_IGNON = 0x00000001;     // IGN ON/OFF mask
    static const SMLK_UINT32 SL_MASK_CONNECTED = 0x00000002; // async connection ready mask
    static const SMLK_UINT32 SL_MASK_LOGIN = 0x00000004;     // login mask
    static const SMLK_UINT32 SL_MASK_TIME_SYNC = 0x00000008; // system time sync mask
};

namespace TimerFlag
{
    static const SMLK_UINT32 SL_MASK_TIMER = 0x00000001;
    static const SMLK_UINT32 SL_MASK_TIMER_RUN = 0x00000002;
};

namespace SystemTime
{
    static const SMLK_UINT32 SL_MASK_SYNC = 0x00000001;
};

namespace UDS
{
    namespace DID
    {
        static const SMLK_UINT32 SL_DID_VIN = 0x00000001;
        static const SMLK_UINT32 SL_DID_CVN = 0x00000002;
        static const SMLK_UINT32 SL_DID_SCIN = 0x00000003;
        static const SMLK_UINT32 SL_DID_IUPR = 0x00000004;
        static const SMLK_UINT32 SL_DID_OBD = 0x00000008;
        static const SMLK_UINT32 SL_DID_DP = 0x00000010;
    }
}

// message types define
namespace MessageType
{
    static const SMLK_UINT8 SL_VEHICLE_LOGIN = 0x01;           // vehicle login
    static const SMLK_UINT8 SL_VEHICLE_LOGOUT = 0x02;          // vehicle logout
    static const SMLK_UINT8 SL_REPORT = 0x03;                  // report
    static const SMLK_UINT8 SL_ENGINE_INFO = 0xF1;             // engine info
    static const SMLK_UINT8 SL_OBD_INFO = 0xF2;                // OBD info
    static const SMLK_UINT8 SL_PROTOCOL_MSG_LOGIN = 0xF3;      // protocol vehicle login
    static const SMLK_UINT8 SL_PROTOCOL_MSG_LOGOUT = 0xF4;     // protocol vehicle logout
    static const SMLK_UINT8 SL_PROTOCOL_MSG_REPORT = 0xF5;     // protocol message for report
    static const SMLK_UINT8 SL_PROTOCOL_MSG_SUPPLEMENT = 0xF6; // protocol message for suppment
};

namespace
{
    static const int SL_QUEUE_GET_TIMEOUT_MS = 200; // timeout millisecond to get item from queue
    // static const int            SL_SESSION_EXPIRED_MS       = 3000;             // session expired

    static const SMLK_UINT32 SL_EVENT_TIMER_NOTIFY = 0xFF000001; // event timer notify
    static const SMLK_UINT32 SL_SYSTEM_TIME_NOTIFY = 0xFF000007; // event system time sync notify
    static const SMLK_UINT32 SL_EVENT_DID_UPDATE = 0xFF000008;   // event DID value update
    static const SMLK_UINT32 SL_EVENT_DTC_UPDATE = 0xFF000009;   // event DTC value update

    static const SMLK_UINT32 SL_TIMER_GENRIC_1S = 0x00000001;

    static const SMLK_UINT32 SL_PERIOD_TRIGGER_POINT_MS = 100; // period trigger ms point (0 ~ 999)

    static const SMLK_UINT32 SL_REPORT_PERIOD_DEF = 10; // default report report period 10 s
    // static const SMLK_UINT32    SL_REPORT_PERIOD_DEF        = 1;               // default report report period 1 s
    static const SMLK_UINT32 SL_REPORT_START_INDEX_DEF = 12; // default report engine information index

    static const SMLK_UINT32 SL_REPORT_FIRST_LOGIN_DEF = 120;  // default do first login after 12000 ms (100ms as unit)
    static const SMLK_UINT32 SL_SAMPLING_PERIOD_DEF = 10;      // default sampling report period 1000 ms (100ms as unit)
    static const SMLK_UINT32 SL_SUPPLEMENT_PERIOD_DEF = 10;    // default supplement report period 1000 ms (100ms as unit)
    static const SMLK_UINT32 SL_RECONNECT_PERIOD_DEF = 100;    // default reconnect period 10 s (100ms as unit)
    static const SMLK_UINT32 SL_EXPIRED_RECORD_PERIOD = 36000; // default expired record clear period 3600s (100ms as unit)

    static const SMLK_UINT32 SL_RECORD_VALID_DURATION = 691200; // deafult record valid duration 8 days 691200s
    static const SMLK_INT32 SL_TIMEZONE_OFFSET = 28800;         // default timezone offset seconds (+8:00   8 * 3600)
};


/**
 * @brief Get the bcd timestamp object
 * 
 * @param buffer 
 */
static void get_bcd_timestamp(SMLK_BCD *buffer)
{
    struct tm tt
    {
    };
    time_t now = time(0);
    localtime_r(&now, &tt);

    tt.tm_year -= 100;
    tt.tm_mon += 1;

    buffer[0] = (SMLK_UINT8)tt.tm_year;
    buffer[1] = (SMLK_UINT8)tt.tm_mon;
    buffer[2] = (SMLK_UINT8)tt.tm_mday;
    buffer[3] = (SMLK_UINT8)tt.tm_hour;
    buffer[4] = (SMLK_UINT8)tt.tm_min;
    buffer[5] = (SMLK_UINT8)tt.tm_sec;
}


/**
 * @brief 
 * 
 * @param timestamp 
 * @return SMLK_UINT32 
 */
static SMLK_UINT32 bcd_timestamp_2_id(const SMLK_BCD *timestamp)
{
    /*
     * bit  0 ~  5  second
     * bit  6 ~ 11  minute
     * bit 12 ~ 16  hour
     * bit 17 ~ 21  day
     * bit 22 ~ 25  month
     * bit 26 ~ 31  year
     */
    return (SMLK_UINT32)(((timestamp[0] >> 4) & 0xF) * 10 + (timestamp[0] & 0xF)) << 26   /* year since 2000 */
           | (SMLK_UINT32)(((timestamp[1] >> 4) & 0xF) * 10 + (timestamp[1] & 0xF)) << 22 /* month */
           | (SMLK_UINT32)(((timestamp[2] >> 4) & 0xF) * 10 + (timestamp[2] & 0xF)) << 17 /* day */
           | (SMLK_UINT32)(((timestamp[3] >> 4) & 0xF) * 10 + (timestamp[3] & 0xF)) << 12 /* hour */
           | (SMLK_UINT32)(((timestamp[4] >> 4) & 0xF) * 10 + (timestamp[4] & 0xF)) << 6  /* minute */
           | (SMLK_UINT32)(((timestamp[5] >> 4) & 0xF) * 10 + (timestamp[5] & 0xF));      /* second */
}


/**
 * @brief 
 * 
 * @param tt 
 * @param BCD 
 * @param sz 
 */
static void timestamp_2_bcd(IN std::time_t tt, INOUT SMLK_BCD *BCD, IN std::size_t sz)
{
    if ((nullptr == BCD) || (sz != M_GB_17691_SZ_TIMESTAMP))
    {
        SMLK_LOGE("TimeStamp2BCD FAIL!!!");
        return;
    }
    std::tm tm;
    localtime_r(&tt, &tm);
    // check whether src time is invalid
    std::time_t local_time;
    if (tt == 0)
    {
        SMLK_LOGD("timestamp src = 0");
        std::memset(BCD, 0x0, sz);
        return;
    }
    else
    {
        localtime_r(&tt, &tm);
    }

    tm.tm_year -= 100; // years since 2000
    tm.tm_mon += 1;

    BCD[0] = (SMLK_UINT8)tm.tm_year;
    BCD[1] = (SMLK_UINT8)tm.tm_mon;
    BCD[2] = (SMLK_UINT8)tm.tm_mday;
    BCD[3] = (SMLK_UINT8)tm.tm_hour;
    BCD[4] = (SMLK_UINT8)tm.tm_min;
    BCD[5] = (SMLK_UINT8)tm.tm_sec;
}


/**
 * @brief 
 * 
 * @param pData 
 * @param startIndex 
 * @param length 
 */
void EndianSwap(uint8_t *pData, int startIndex, int length) {
    int i, cnt, end, start;
    if (length == 1) return;
    cnt = length / 2;
    start = startIndex;
    end = startIndex + length - 1;
    uint8_t tmp;
    for (i = 0; i < cnt; i++) {
        tmp = pData[start + i];
        pData[start + i] = pData[end - i];
        pData[end - i] = tmp;
    }
}


/**
 * @brief 
 * 
 * @param data 
 * @param len 
 * @return uint8_t 
 */
uint8_t FCS_calc(const uint8_t *data, size_t len) {
    uint8_t bcc = *(uint8_t *) data;
    uint8_t *p = (uint8_t *) data + 1;
    if (NULL == data || len <= 0) {
        return 0;
    }
    while (--len) {
        bcc ^= *p;
        p++;
    }
    return bcc;
}


/**
 * @brief Construct a new Reporter1239_07::Reporter1239_07 object
 * 
 * @param host 
 * @param port 
 * @param protocol 
 * @param platform 
 */
Reporter1239_07::Reporter1239_07(IN std::string &host, IN SMLK_UINT16 port, IN SMLK_UINT8 protocol, IN SMLK_UINT8 platform)
: m_service_flags(0)
, m_internal_flags(0)
, m_timer_flags(0)
, m_vin_flags(0)
, m_seq(0)
, m_time_sync_gap(0)
, m_protocol_version(protocol)
, m_reconnect_period(SL_RECONNECT_PERIOD_DEF)
, m_report_period(SL_REPORT_PERIOD_DEF)
, m_report_index(0)
, m_reprot_index_engine(SL_REPORT_START_INDEX_DEF)
, m_port(port)
, m_host(host)
, m_vin("0")
, m_chipid("")
, m_pubkey("")
, m_todisk(false)
, m_taskName("HJ07")
{
    SMLK_LOGI("[%s] Reporter1239::Reporter1239, Protocol : 0x%02X ", m_taskName.c_str(), protocol);
}


/**
 * @brief Destroy the Reporter1239_07::Reporter1239_07 object
 * 
 */
Reporter1239_07::~Reporter1239_07()
{
    Stop();
}

SMLK_UINT32 Reporter1239_07::Init()
{
    SMLK_LOGI("[%s] Init enter", m_taskName.c_str());
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_service_flags & IService::Initialized)
    {
        SMLK_LOGD("[%s] Already initialized, do nothing", m_taskName.c_str());
        return SL_SUCCESS;
    }

    auto spVehicleAvtivate = std::make_shared<MessageVehicleActivate1239>();
    m_vehicle_activate = spVehicleAvtivate;

    m_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 100>>(
        [this](SMLK_UINT32 index)
        {
            if (0 == index % SL_SAMPLING_PERIOD_DEF)
            {
                // SMLK_LOGD("[%s] [G6TimerDebug] send SL_TIMER_GENRIC_1S !", m_taskName.c_str());
                MsgHead head(SL_EVENT_TIMER_NOTIFY, index, 0, 0, 0, SL_TIMER_GENRIC_1S);
                Post(head, nullptr, 0);
            }
        });

    m_comm1239 = new common17691(m_taskName);

    // Socket 初始化
    m_TcpClient = new TCPClient(m_taskName);
    // 注册SOCKET状态的回调
    m_TcpClient->RegisterSocketStateCB(std::bind(&Reporter1239_07::HandleSocketConnState, this, std::placeholders::_1, std::placeholders::_2));
    
    m_rec_thread = std::thread( &Reporter1239_07::HandleReceiveDataFromTSP, this);

    // 首次连接一定要连接成功程序才可以继续执行
    while (m_TcpClient->m_connectState != normal)
    {
        SMLK_LOGE("[%s] m_connectState=%d", m_taskName.c_str(), m_TcpClient->m_connectState);

        m_TcpClient->initParam(m_host.c_str(), m_port);
        int ret = m_TcpClient->connect();
        SMLK_LOGI("[%s] Reporter::InitLogic() ret=%d", m_taskName.c_str(), ret);

        if (ret != 0)
        {
            sleep(3);
        }
    }


    m_service_flags |= IService::Initialized;

    SMLK_LOGI("[%s] Init finish", m_taskName.c_str());
    return SL_SUCCESS;
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
SMLK_UINT32 Reporter1239_07::Start()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_service_flags & IService::Running)
    {
        SMLK_LOGI("[%s] service already started, do nothing", m_taskName.c_str());
        return SL_SUCCESS;
    }

    if (!(m_service_flags & IService::Initialized))
    {
        SMLK_LOGE("[%s] service not initialized", m_taskName.c_str());
        return SL_EFAILED;
    }

    m_timer->Start();
    m_timer->Reset();
    m_timer_flags |= TimerFlag::SL_MASK_TIMER;
    SMLK_LOGI("[%s] [G6TimerDebug] start and reset timer", m_taskName.c_str());

    m_service_flags |= IService::Running;

    m_thread = std::thread(
        [this]()
        {
            SMLK_LOGI("[%s] main work thread started", m_taskName.c_str());
            while (0 == (m_service_flags & IService::Stopping))
            {
                m_events.get(SL_QUEUE_GET_TIMEOUT_MS)();
            }
            SMLK_LOGI("[%s] main work thread exit!!!", m_taskName.c_str());
        });

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
void Reporter1239_07::Stop()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_service_flags & IService::Running)
    {
        SMLK_LOGI("[%s] told to stop, mark as stopping", m_taskName.c_str());
        m_service_flags |= IService::Stopping;

        if (m_thread.joinable())
        {
            m_thread.join();
        }

        SMLK_LOGI("[%s] [G6TimerDebug] stop rt timer", m_taskName.c_str());
        m_timer_flags &= ~TimerFlag::SL_MASK_TIMER;
        m_timer->Stop();

        SMLK_LOGI("[%s] stop tcp client", m_taskName.c_str());
        if (m_TcpClient != nullptr)
        {
            m_TcpClient->close();
        }

        if (m_internal_flags & InternalFlag::SL_MASK_LOGIN)
        {
            m_internal_flags &= ~InternalFlag::SL_MASK_LOGIN;
        }

        m_service_flags |= ~(IService::Running | IService::Stopping);

        SMLK_LOGI("[%s] stopped", m_taskName.c_str());
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
bool Reporter1239_07::IsRunning() const
{
    return m_service_flags & IService::Running;
}

SMLK_UINT32 Reporter1239_07::SetVIN(IN std::string &vin)
{    
    SMLK_LOGE("[%s] Reporter1239_07::SetVIN %s", m_taskName.c_str(), vin.data());

    std::lock_guard<std::mutex> lk(m_mutex);
    if (0 != (m_service_flags & IService::Initialized))
    {
        return SL_ESUPPORT;
    }

    m_vin = vin;

    return SL_SUCCESS;
}

/**
 * @brief 同步激活状态
 * 
 * @param state 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter1239_07::SetActStatus(IN SMLK_UINT32 state)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (0 != (m_service_flags & IService::Initialized))
    {
        return SL_ESUPPORT;
    }

    // m_actstate = state;
    SMLK_LOGE("[%s] Reporter1239_07::SetActStatus %d", m_taskName.c_str(), state);

    return SL_SUCCESS;
}


/**
 * @brief 设置加密芯片 id
 * 
 * @param chipid 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 Reporter1239_07::SetChipID(IN std::string &chipid)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (0 != (m_service_flags & IService::Initialized))
    {
        return SL_ESUPPORT;
    }

    m_chipid = chipid;
    SMLK_LOGE("[%s] Reporter1239_07::SetChipID %s", m_taskName.c_str(), m_chipid.c_str());

    return SL_SUCCESS;
}

SMLK_UINT32 Reporter1239_07::SetPubKey(IN std::string &pubkey)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (0 != (m_service_flags & IService::Initialized))
    {
        return SL_ESUPPORT;
    }

    m_pubkey = pubkey;
    SMLK_LOGE("[%s] Reporter1239_07::SetPubKey %s", m_taskName.c_str(), m_pubkey.c_str());

    return SL_SUCCESS;
}

SMLK_UINT32 Reporter1239_07::SetTimeSyncGap(IN SMLK_UINT64 timesyncgap)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (0 != (m_service_flags & IService::Running))
    {
        return SL_ESUPPORT;
    }

    m_internal_flags |= InternalFlag::SL_MASK_TIME_SYNC;

    m_time_sync_gap = timesyncgap;

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: SetIsSave
 *
 * DESCRIPTION: set save raw data to file
 *
 * PARAMETERS:
 *
 * RETURN:
 *  SL_SUCCESS  on no error occurs, otherwise return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Reporter1239_07::SetIsSave(IN SMLK_BOOL save)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (0 != (m_service_flags & IService::Running))
    {
        return SL_ESUPPORT;
    }
    m_todisk = save;
    SMLK_LOGE("[%s] Reporter1239_07::SetIsSave %d", m_taskName.c_str(), m_todisk);

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: SetTelephonyState
 *
 * DESCRIPTION: set telephony status
 *
 * PARAMETERS:
 *      info:   telephony information
 * RETURN:
 *
 * NOTE:
 *      here the telephony actually means the modem module, and we always just
 *      foucsed on data service only
 *****************************************************************************/
SMLK_UINT32 Reporter1239_07::SetTelephonyState(IN ProtocolReporter::telephony::Info &info)
{
    SMLK_LOGD("[%s] Reporter1239_07::SetTelephonyState() channel %d, type %d, state %d", m_taskName.c_str(), info.channel, info.type, info.status);

    if ((info.channel == 1))
    {
        if (info.status == 1)
        {
            SMLK_LOGI("[%s] TelephonyNotify: we lost the connection", m_taskName.c_str());
            if (m_TcpClient->m_connectState == normal)
            {
                m_TcpClient->close();
            }
        }
        else if (info.status == 2)
        {
            SMLK_LOGI("[%s] TelephonyNotify: we get new connection", m_taskName.c_str());
        }
    }
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: IgnON
 *
 * DESCRIPTION: IGN ON notification
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter1239_07::IgnON()
{
    m_comm1239->setIgnState(M_GB_17691_IGN_ON);

    if (0 == (m_internal_flags & InternalFlag::SL_MASK_IGNON))
    {
        m_internal_flags |= InternalFlag::SL_MASK_IGNON;
    }
}

/******************************************************************************
 * NAME: IgnOFF
 *
 * DESCRIPTION: IGN OFF notification
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter1239_07::IgnOFF()
{
    m_comm1239->setIgnState(M_GB_17691_IGN_OFF);

    if (0 != (m_internal_flags & InternalFlag::SL_MASK_IGNON))
    {
        SMLK_LOGI("[%s] IGN state changed from ON to OFF", m_taskName.c_str());
        m_internal_flags &= ~InternalFlag::SL_MASK_IGNON;

        SMLK_LOGI("[%s] [G6TimerDebug] set rt timer", m_taskName.c_str());
        m_timer->Reset();
    }
}

/******************************************************************************
 * NAME: UpdateVIN
 *
 * DESCRIPTION: update vin
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter1239_07::UpdateVIN(SMLK_UINT16 result, const std::vector<SMLK_UINT8> &vin)
{
    std::stringstream   ss;
    for ( auto ch : vin ) {
        ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
    }

    SMLK_LOGD("[%s] UpdateVIN %s", m_taskName.c_str(), ss.str().c_str());
    Post(MsgHead(SL_EVENT_DID_UPDATE, UDS::DID::SL_DID_VIN, 0, 0, 0, result), vin);
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
void Reporter1239_07::Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz)
{
    m_events.emplace(
        head,
        ptr,
        sz,
        std::bind(&Reporter1239_07::HandleEvent, this, std::placeholders::_1, std::placeholders::_2));
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
void Reporter1239_07::Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_events.emplace(
        head,
        data,
        std::bind(&Reporter1239_07::HandleEvent, this, std::placeholders::_1, std::placeholders::_2));
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
void Reporter1239_07::Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data)
{
    m_events.emplace(
        head,
        std::move(data),
        std::bind(&Reporter1239_07::HandleEvent, this, std::placeholders::_1, std::placeholders::_2));
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
void Reporter1239_07::HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch (head.m_id)
    {
    case SL_EVENT_TIMER_NOTIFY:
        OnTimer(head.m_extra, head.m_seq);
        break;
    case SL_SYSTEM_TIME_NOTIFY:
        OnSystemTimeNotify(head, data);
        break;
    case SL_EVENT_DID_UPDATE:
        OnDIDNotify(head, data);
        break;
    default:
        break;
    }
}


/**
 * @brief system time notify event handler
 * 
 * @param head message head which has the basic information about this message
 * @param data message data
 */
void Reporter1239_07::OnSystemTimeNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    if (0 != (head.m_seq & SystemTime::SL_MASK_SYNC))
    {
        m_internal_flags |= InternalFlag::SL_MASK_TIME_SYNC;
    }
    else
    {
        m_internal_flags &= ~InternalFlag::SL_MASK_TIME_SYNC;
    }
}


/**
 * @brief did event handler
 * 
 * @param head 
 * @param data 
 */
void Reporter1239_07::OnDIDNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("[%s] OnDIDNotify head.m_seq %x: head.result %x", m_taskName.c_str(), head.m_seq, head.m_extra);

    switch (head.m_seq)
    {
    case UDS::DID::SL_DID_VIN:
    {
        decltype(data.size())   sz = data.size()-2;
        if ( sz > sizeof(M_GB_17691_SZ_VIN) ) {
            sz = sizeof(M_GB_17691_SZ_VIN);
        }
        m_vin = "00000000000000000";
        if(1 == head.m_extra) {
            SMLK_LOGE("[%s] OnDIDNotify case UDS::DID::SL_DID_VIN: did result err", m_taskName.c_str());
            m_vin_flags = 0;
        } else {
            m_vin_flags = 1;

            std::stringstream stream;
            stream << data.data();
            m_vin = stream.str().substr(2, 17);
            SMLK_LOGE("[%s] OnDIDNotify case UDS::DID::SL_DID_VIN: %s", m_taskName.c_str(), m_vin.c_str());
        }
    }
    break;
    default:
        SMLK_LOGE("[%s] unsupported DID: 0x%08X", m_taskName.c_str(), head.m_seq);
        break;
    }
}


/**
 * @brief timer event handler
 * 
 * @param id 
 * @param seq 
 */
void Reporter1239_07::OnTimer(IN SMLK_UINT32 id, IN SMLK_UINT32 seq)
{
    switch (id)
    {
    case SL_TIMER_GENRIC_1S:
        OnGenericTimer(seq);
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
void Reporter1239_07::OnGenericTimer(IN SMLK_UINT32 seq)
{

    if (0 == seq % 300 || 10 == seq)
    {
        std::unique_lock<std::mutex> lck(m_actMtx); //加锁互斥量
        if (m_TcpClient->m_connectState = normal)
        {   
            if (m_vin_flags == 1) {
                DoVehicleActivate();
            }

            if (cv.wait_for(lck, std::chrono::seconds(10)) == std::cv_status::timeout)
            {
                // resulthandler::getInstance()->handleActiveResult(M_HJ_1239_ACTIVATED_OVERTIME_ERROR);
                SMLK_LOGI("[%s] OnGenericTimer record timeout ==============", m_taskName.c_str());
            }
        }
        else
        {
            m_TcpClient->connect();
        }
    }
}


/**
 * @brief activate success event handler
 * 
 */
void Reporter1239_07::OnActivate()
{
    cv.notify_one();
    SMLK_LOGI("[%s] VEHICLE Activate send SUCCESS", m_taskName.c_str());

    SMLK_LOGI("[%s] [G6TimerDebug] set rt timer", m_taskName.c_str());
    m_timer->Reset();
}

/**
 * @brief
 *
 * @param src
 * @param length
 */
void Reporter1239_07::OnReceiveData(char *src, uint length)
{
    if (m_todisk)
    {
        std::stringstream strStream;
        time_t timep;
        struct tm *p;
        time(&timep);
        p = localtime(&timep);
        struct timeval ms;
        gettimeofday(&ms, NULL);
        strStream << "[" << m_taskName << "]" << " " << "08 " << "receive time:" << p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
        for (int i = 0; i < length; i++)
        {
            strStream << std::hex << std::setw(2) << std::setfill('0') << (int)src[i] << " ";
        }
        system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/gb6/record_gb17691.log").c_str());
    }

    SMLK_LOGD("Reporter1239_07::OnReceiveData() receive length = %0d", length);

    //解出MessageID，根据ID 把方法体的内容同步给具体的业务类
    HJ_1239_MsgEntity entity;
    uint bodyLen = 0;
    bodyLen = length - sizeof(HJ_1239_MsgEntity) + 4;

    memset(&entity, 0, sizeof(entity));

    //head
    // if ((*src == '#') && (*(src + 1) == '#'))
    if ((*src == '~') && (*(src + 1) == '~'))
    {
        uint8_t checkByte = FCS_calc((unsigned char *)src + 2, length - 3);
        if ((uint8_t)src[length - 1] == checkByte)
        {
            entity.msghead.cmd = *(src + 2);
            memcpy(&entity.msghead.vin, src + 3, 17);
            entity.msghead.ver = *(src + 20);
            entity.msghead.encryption = *(src + 21);
            SMLK_LOGD("msgId=%0d, encryption=%0d, vin=%s, ver=%0d", entity.msghead.cmd, entity.msghead.encryption, entity.msghead.vin, entity.msghead.ver);
            memcpy((void *) &entity.msghead.length, src + 22, 2);
            EndianSwap((u_char * ) & entity.msghead.length, 0, 2);
            SMLK_LOGE("decode length=%d, receive bodyLen=%d", entity.msghead.length, bodyLen);
            
            if (entity.msghead.length == bodyLen) {
                SMLK_LOGD("decode msg body, cmd %02x", entity.msghead.cmd);
                if (bodyLen > 0) {
                        entity.dataBody = nullptr;
                        entity.dataBody = new char[bodyLen];
                        if (entity.dataBody != nullptr)
                        {                            
                            memcpy(entity.dataBody, src + 24, bodyLen);

                            if (entity.msghead.cmd == 8 && bodyLen == sizeof(HJ_1239_VehicleActiveResp)) {
                                HJ_1239_VehicleActiveResp resp;
                                resp.statuscode = *entity.dataBody;
                                resp.infor = *(entity.dataBody + 1);
                                SMLK_LOGD("=== statuscode=%0d, infor=%0d ================ ", resp.statuscode, resp.infor);

                                // 状态码   :   0x01:激活成功，0x02:激活失败
                                // 信息     :   激活成功 : 0x00:
                                //              激活失败 : 0x01:芯片已激活，0x02:VIN错误
                                if (resp.statuscode == 0x01 && resp.infor == 0x00)
                                {
                                    cv.notify_one();
                                    m_timer->Reset();
                                }
                                else if (resp.statuscode == 0x02 && resp.infor == 0x01)
                                {
                                    cv.notify_one();
                                }
                                else if (resp.statuscode == 0x02 && resp.infor == 0x02)
                                {
                                    cv.notify_one();
                                }
                                else if (resp.statuscode == 0x02 && resp.infor == 0x03)
                                {
                                    cv.notify_one();
                                }
                                else
                                {
                                    cv.notify_one();
                                }
                            }
                        }
                    }

                    // 释放已经用完的dataBody
                    if (entity.dataBody != nullptr) {
                        delete[]entity.dataBody;
                        entity.dataBody = nullptr;
                    }

                } else {
                    SMLK_LOGI("[%s] Decode ERROR Body Length num", m_taskName.c_str());
                }
            }
        else
        {
            SMLK_LOGI("[%s] Decode ERROR check num", m_taskName.c_str());
        }
    }
    else
    {
        SMLK_LOGI("[%s] Decode ERROR HEAD FLAG", m_taskName.c_str());
    }

    return;
}


/**
 * @brief do vehicle activate
 * 
 */
void Reporter1239_07::DoVehicleActivate()
{
    SMLK_LOGI("[%s] DoVehicleActivate enter", m_taskName.c_str());

    auto spVehicleAvtivate = std::static_pointer_cast<MessageVehicleActivate>(m_vehicle_activate);

    spVehicleAvtivate->m_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - (m_time_sync_gap / 1000);
    spVehicleAvtivate->m_security_chip_id = m_chipid;
    spVehicleAvtivate->m_public_key = m_pubkey;
    spVehicleAvtivate->m_vin = m_vin;

    SMLK_BCD t_timestamp[6] = {0};
    timestamp_2_bcd((std::time_t)spVehicleAvtivate->m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_UINT32 id = bcd_timestamp_2_id(t_timestamp);

    std::vector<SMLK_UINT8> body;
    body.clear();
    spVehicleAvtivate->Encode(body);

    int n_sign_len = 128;
    unsigned char sign_data[n_sign_len] = {0};
    int ret_t = hsm_seal_sign(TYPE_SM2, sign_data, (size_t *)&n_sign_len, body.data(), body.size(), ALG_MD3, 0, (unsigned char *)m_chipid.data(), m_chipid.size());

    int n_R_sign_len = 32;
    unsigned char R_sign_data[n_R_sign_len] = {0};
    int n_S_sign_len = 32;
    unsigned char S_sign_data[n_S_sign_len] = {0};

    ret_t = hsm_SM3Sign_Split_RCode_And_SCode(sign_data, n_sign_len, R_sign_data, (size_t *)&n_R_sign_len, S_sign_data, (size_t *)&n_S_sign_len);

    body.push_back(0x20); // R 长度
    for (auto i = 0; i < 32; i++)
    {
        body.push_back(R_sign_data[i]);
    }

    body.push_back(0x20); // S 长度
    for (auto i = 0; i < 32; i++)
    {
        body.push_back(S_sign_data[i]);
    }

    std::vector<SMLK_UINT8> encoded;
    encoded.clear();
    encoded.reserve(body.size() + sizeof(GB_17691_MsgHead) + 1);

    encoded.push_back(M_HJ_1239_MSG_FLAG);
    encoded.push_back(M_HJ_1239_MSG_FLAG);
    encoded.push_back(M_HJ_1239_CMD_VEHICLE_ACT_REQ);
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
                  << "07 "
                  << "send time:" << p->tm_hour << ":" << p->tm_min << ":"
                  << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
        for (int i = 0; i < encoded.size(); i++)
        {
            strStream << std::hex << std::setw(2) << std::setfill('0') << (int)encoded[i] << " ";
        }
        system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/gb6/record_gb17691.log").c_str());
    }

    int ret = HandleSendDataToTSP(M_HJ_1239_CMD_VEHICLE_ACT_REQ, encoded.data(), encoded.size());
    SMLK_LOGI("[%s] DoVehicleActivate, encode time %d-%d-%d %d:%d:%d, send ret = %d, Send %s ", m_taskName.c_str(),
              t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5],
              ret, ret > 0 ? "success" : "failed");
}


/**
 * @brief 
 * 
 * @param act 
 */
void Reporter1239_07::VehicleActivate(IN SMLK_UINT32 act)
{
    SMLK_LOGI("[%s] VehicleActivate enter", m_taskName.c_str());
    if (act == M_HJ_1239_ACTIVATED_ING)
    {
        m_timer->ReStart();
    }
    else if (act == M_HJ_1239_ACTIVATED_RESERVE)
    {
        m_timer->Reset();
    }
    else
    {
        SMLK_LOGI("[%s] VehicleActivate unsupport act %d", m_taskName.c_str(), act);
    }
}

/**
 * @brief 处理 TCP 连接状态
 *
 * @param task
 * @param state
 * @return SMLK_INT32
 */
SMLK_INT32 Reporter1239_07::HandleSocketConnState(std::string &task, SMLK_INT32 state)
{
    if (state == normal)
    {
        SMLK_LOGD("[%s] [%s] Async: we got a connection notificaiton, m_vin_flags %d ", m_taskName.c_str(), task.c_str(), m_vin_flags);
    }

    if (state == disconnect)
    {
        SMLK_LOGE("[%s] [%s] Async err: we lost the connection", m_taskName.c_str(), task.c_str());
        if (m_TcpClient->m_connectState == normal)
        {
            m_TcpClient->close();
        }
    }

    if (state == notInit)
    {
    }

    return 0;
}

/**
 * @brief
 *
 * @param cmd
 * @param data
 * @param length
 * @return SMLK_INT32
 */
SMLK_INT32 Reporter1239_07::HandleSendDataToTSP(SMLK_UINT8 cmd, SMLK_UINT8 *data, SMLK_UINT32 length)
{
    std::lock_guard<std::mutex> locker(m_sendMtx);
    SMLK_INT32 ret = -1;

    if (m_TcpClient->m_connectState == normal)
    {
        ret = m_TcpClient->sendMsg(data, length);
    }
    else
    {
        // SMLK_LOGD("[%s] Reporter1239_07::HandleSendDataToTSP() Connect Error!!! ", m_taskName.c_str() );
    }

    return ret;
}

/**
 * @brief 监听TSP过来的消息
 * @param arg   this point
 */
void Reporter1239_07::HandleReceiveDataFromTSP(void)
{
    SMLK_LOGD("[T] Reporter::ReceiveTSPData() run");
    int retLen = 0;
    while (1)
    {
        if (m_TcpClient->m_connectState == normal)
        {
            char buf[M_HJ_1239_SZ_DATA_MAX] = {0};
            retLen = 0;
            retLen = m_TcpClient->recMsg(buf, M_HJ_1239_SZ_DATA_MAX);
            if (retLen > 0)
            {
                OnReceiveData(buf, retLen);
            }
        }
        else
        {
            // SMLK_LOGD("[T] Reporter::ReceiveTSPData() Connect Error!!! " );
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

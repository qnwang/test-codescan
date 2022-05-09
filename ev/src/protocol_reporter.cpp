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
#include <unistd.h>

#include <smlk_log.h>
#include <smlk_tools.h>

#include <vehicle_data_index_def.h>
#include "ev_signals.h"
#include "protocol_reporter.h"


using namespace smartlink;

TCPClient *Reporter::m_TcpClient32960 = NULL;
SqlLiteBase *Reporter::m_SqlLiteBase32960 = NULL;

std::mutex Reporter::m_sendMtx;

namespace InternalFlag {
    static const SMLK_UINT32    SL_MASK_IGNON               = 0x00000001;           // ING ON/OFF mask
    static const SMLK_UINT32    SL_MASK_CONNECTED           = 0x00000002;           // async connection ready mask
    static const SMLK_UINT32    SL_MASK_LOGIN               = 0x00000004;           // login mask
    static const SMLK_UINT32    SL_MASK_TIME_SYNC           = 0x00000008;           // system time sync mask
};

namespace SystemTime {
    static const SMLK_UINT32    SL_MASK_SYNC                = 0x00000001;
};

namespace {
    static const int            SL_QUEUE_GET_TIMEOUT_MS     = 200;              // timeout millisecond to get item from queue
    static const int            SL_SESSION_EXPIRED_MS       = 3000;             // session expired

    static const SMLK_UINT32    SL_EVENT_LOCAT_NOTIFY       = 0xFF000001;           // event location notify
    static const SMLK_UINT32    SL_EVENT_DATAC_NOTIFY       = 0xFF000002;           // event data changed notify
    static const SMLK_UINT32    SL_EVENT_DM1_UPDATE         = 0xFF000003;           // event DM1 value update
    static const SMLK_UINT32    SL_SYSTEM_TIME_NOTIFY       = 0xFF000009;           // event system time sync notify

    static const SMLK_INT32     SL_TIMEZONE_OFFSET          = 28800;            // default timezone offset seconds (+8:00   8 * 3600)
};

Reporter::Reporter(IN std::string &host, IN SMLK_UINT16 port)
    : m_flags(0)
    , m_press_1_warn_flags(0)
    , m_press_2_warn_flags(0)
    , m_first_init(false)
    , m_IsDoLogic(false)
    , is_braksysErr(false)
    , m_host(host)
    , m_port(port)
{}

Reporter::~Reporter()
{
    Stop();
    // m_MessageOb->DetachObserver();
    delete  m_protocol_vehicle_login;
    delete m_protocol_vehicle_logout;
    delete m_protocol_realtime_report;
    delete m_protocol_blindinfo_report;
    delete m_protocol_heartbeat;
    m_protocol_vehicle_login = nullptr;
    m_protocol_vehicle_logout = nullptr;
    m_protocol_realtime_report = nullptr;
    m_protocol_blindinfo_report = nullptr;
    m_protocol_heartbeat = nullptr;
}

SMLK_UINT32 Reporter::Init()
{
    SMLK_LOGD("Init enter");

    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Initialized ) {
        SMLK_LOGD("already initialized, do nothing");
        return SL_SUCCESS;
    }

    for ( auto const &pair : vehicle::g_vehicle_signals ) {
        common32960::getInstance()->m_cached_signals.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(pair.first),
                std::forward_as_tuple(pair.second)
                );

        auto const it = vehicle::g_vehicle_signal_index.find(pair.first);
        if ( vehicle::g_vehicle_signal_index.cend() == it ) {
            continue;
        }

        m_cached_index_signals.emplace(
                    std::piecewise_construct,
                    std::forward_as_tuple(it->second),
                    std::forward_as_tuple(std::ref(common32960::getInstance()->m_cached_signals.find(pair.first)->second))
                    );
        // SMLK_LOGD("ID[%u] --> INDEX[%u]", common32960::getInstance()->m_cached_signals.find(pair.first)->second.ID(), it->second);
    }

    m_protocol_vehicle_login            = new MessageVehicleLogin32960();
    m_protocol_vehicle_logout           = new MessageVehLogout32960();
    m_protocol_realtime_report          = new MessageRealTimeInfo32960();
    m_protocol_blindinfo_report         = new MessageBlindInfoUp32960();
    m_protocol_heartbeat                = new MessageHeartBeat32960();

    m_flags |= IService::Initialized;

    return SL_SUCCESS;
}


void Reporter::InitLogic() {
    SMLK_LOGD("[T] Reporter::InitLogic() ");
    if (!m_first_init) {
        if (m_IsDoLogic == true) {
            SMLK_LOGD("[T] Reporter::InitLogic() has already done");
            return;
        }

        // sqlite 初始化
        m_SqlLiteBase32960 = new SqlLiteBase;
        common32960::getInstance()->m_SqlLiteBase32960 = m_SqlLiteBase32960;

        m_SqlLiteBase32960->Init();

        // 注册发送给tsp数据的回调
        m_MessageOb = MessageObserver32960::GetInstance();
        m_MessageOb->RegisterDataSendCB(HandleSendData2TSP32960);
        // Socket 初始化
        m_TcpClient32960 = new TCPClient();
        common32960::getInstance()->m_TcpClient32960 = m_TcpClient32960;

        m_TcpClient32960->RegisterSocketStateCB(HandleSocketConnState);        // 注册SOCKET状态的回调

        // 首次连接一定要连接成功程序才可以继续执行
        int try_cnt = 0;
        while (m_TcpClient32960->m_connectState != normal) {
            SMLK_LOGE("m_connectState=%d", m_TcpClient32960->m_connectState);

            m_TcpClient32960->initParam(m_host.c_str(), m_port);
            int ret = m_TcpClient32960->connect();
            SMLK_LOGD("[T] Reporter::InitLogic() ret=%d", ret);

            if (ret != 0) {
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }
        }

        // 初始化发送周期
        common32960::getInstance()->initConfigParam();
        // 初始化流水号
        common32960::getInstance()->initSequence();
        // Socket 接收数据线程
        common32960::getInstance()->GetThreadPool()->commit(ReceiveTSPData, this);
        // Socket 状态监控线程
        common32960::getInstance()->GetThreadPool()->commit(SocketMonitor, this);

        m_MessageOb->AttachObserver(msgID_32960::M_GB_32960_CMD_VEHICLE_LOGIN, m_protocol_vehicle_login);       //车辆登入
        m_MessageOb->AttachObserver(msgID_32960::M_GB_32960_CMD_VEHICLE_LOGOUT, m_protocol_vehicle_logout);     //车辆登出
        m_MessageOb->AttachObserver(msgID_32960::M_GB_32960_CMD_REALTIME_REPORT, m_protocol_realtime_report);   //实时数据
        m_MessageOb->AttachObserver(msgID_32960::M_GB_32960_CMD_REISSUE_REPORT, m_protocol_blindinfo_report);   //补发数据
        m_MessageOb->AttachObserver(msgID_32960::M_GB_32960_CMD_HEARTBEAT, m_protocol_heartbeat);               //心跳

        if (common32960::getInstance()->m_run_mode == 0x01 || common32960::getInstance()->m_run_mode == 0x02 || common32960::getInstance()->m_run_mode == 0x03) {
            m_protocol_heartbeat->run();
            m_protocol_realtime_report->run(m_protocol_realtime_report);
            m_protocol_blindinfo_report->run();                
        }

        m_IsDoLogic = true;
        m_first_init = true;
    } else {
        if (!m_IsDoLogic) {
            if (common32960::getInstance()->m_run_mode == 0x01 || common32960::getInstance()->m_run_mode == 0x02 || common32960::getInstance()->m_run_mode == 0x03) {
                m_protocol_realtime_report->run(m_protocol_realtime_report);
            }
            m_IsDoLogic = true;
        }
        // 初始化流水号
        common32960::getInstance()->initSequence();
    }
};
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
SMLK_UINT32 Reporter::Start()
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

    m_flags |= IService::Running;

    m_thread = std::thread(
        [this]() {
            SMLK_LOGI("main work thread started");
            while ( 0 == (m_flags & IService::Stopping) ) {
                m_events.get(SL_QUEUE_GET_TIMEOUT_MS)();
            }
            SMLK_LOGI("main work thread exit!!!");
        }
    );

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
void Reporter::Stop()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Running ) {
        SMLK_LOGI("told to stop, mark as stopping");
        m_flags |= IService::Stopping;

        if ( m_thread.joinable() ) {
            m_thread.join();
        }

        SMLK_LOGI("stop tcp client");
        if (m_TcpClient32960 != nullptr) {
            m_TcpClient32960 ->close();
        }

        if(m_SqlLiteBase32960){
            delete m_SqlLiteBase32960;
            m_SqlLiteBase32960 = nullptr;
        }  

        m_flags |= ~(IService::Running | IService::Stopping);

        SMLK_LOGI("stopped");
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
bool Reporter::IsRunning() const
{
    return  m_flags & IService::Running;
}

/******************************************************************************
 * NAME: SetCONFIG
 *
 * DESCRIPTION: set CONFIG information
 *
 * PARAMETERS:
 *
 * RETURN:
 *  SL_SUCCESS  on no error occurs, otherwise return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Reporter::SetCONFIG(IN SMLK_UINT8 config)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_flags & IService::Initialized) ) {
        return SL_ESUPPORT;
    }
    
    common32960::getInstance()->m_run_mode = config;
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: SetTORQUE
 *
 * DESCRIPTION: set TORQUE information
 *
 * PARAMETERS:
 *
 * RETURN:
 *  SL_SUCCESS  on no error occurs, otherwise return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Reporter::SetTORQUE(IN SMLK_UINT16 torque)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_flags & IService::Initialized) ) {
        return SL_ESUPPORT;
    }
    
    common32960::getInstance()->m_refer_torque = torque;
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: SetVIN
 *
 * DESCRIPTION: set VIN information
 *
 * PARAMETERS:
 *
 * RETURN:
 *  SL_SUCCESS  on no error occurs, otherwise return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Reporter::SetVIN(IN std::string &vin)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_flags & IService::Initialized) ) {
        return SL_ESUPPORT;
    }

    std::memcpy(common32960::getInstance()->m_vin, vin.c_str(), M_GB_32960_SZ_VIN);
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: SetICCID
 *
 * DESCRIPTION: set ICCID information
 *
 * PARAMETERS:
 *
 * RETURN:
 *  SL_SUCCESS  on no error occurs, otherwise return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Reporter::SetICCID(IN std::string &iccid)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_flags & IService::Initialized) ) {
        return SL_ESUPPORT;
    }

    std::memcpy(common32960::getInstance()->m_iccid, iccid.c_str(), M_GB_32960_SZ_ICCID);

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
SMLK_UINT32 Reporter::SetIsSave(IN SMLK_BOOL save)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( 0 != (m_flags & IService::Running) ) {
        return SL_ESUPPORT;
    }

    common32960::getInstance()->m_todisk = save;

    return SL_SUCCESS;
}

SMLK_UINT32 Reporter::SetStandbyState()
{
    SMLK_LOGD("Reporter::SetStandbyState()");

    common32960::getInstance()->setIgnState(2);

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: SetTelephonyState
 *
 * DESCRIPTION: update telephony information
 *
 * PARAMETERS:
 *      info:   telephony information
 * RETURN:
 *      SL_SUCCESS
 * NOTE:
 *      here the telephony actually means the modem module, and we always just
 *      foucsed on data service only
 *****************************************************************************/
SMLK_UINT32 Reporter::SetTelephonyState(IN ProtocolReporter::telephony::Info &info)
{
    SMLK_LOGD("Reporter::SetTelephonyState() channel %d, type %d, state %d", info.channel, info.type, info.status);
    // SMLK_LOGD("Reporter::SetTelephonyState() interface %s", info.interface.c_str());

    if ((info.channel == 1)) {
        if  (info.status == 1) {
            m_TcpClient32960 ->close();
            SMLK_LOGD("SetTelephonyState: we lost the connection");
        } else if (info.status == 2) {
            SMLK_LOGD("SetTelephonyState: we get new connection");
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
void Reporter::IgnON()
{
    // SMLK_LOGE("Reporter::IgnON() enter");
    std::lock_guard<std::mutex> lock(m_initMutex);
    if (common32960::getInstance()->getIgnState() == 0)
    {
        InitLogic();
    }
    common32960::getInstance()->setIgnState(1);
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
void Reporter::IgnOFF()
{
    SMLK_LOGE("Reporter::IgnOFF() enter");

    common32960::getInstance()->setIgnState(0);

    if (m_IsDoLogic)
    {
        // ign off 时保存登录和数据上传流水号，并进行设备登出
        // common32960::getInstance()->setInfoUpTime32960(1); //发送一帧实时数据
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // common32960::getInstance()->setInfoUpTime32960(common32960::getInstance()->getConfigParam().upload_period_bak);
        // SMLK_LOGE("common32960::getInstance()->getConfigP1111aram().upload_period_bak: %d", common32960::getInstance()->getInfoUpTime32960());

        // DTC故障代码
        common32960::getInstance()->setDtcErrCode(6);

        m_protocol_vehicle_login->delLoginTimer();
        m_protocol_realtime_report->deleInfoUpTimer();
        if (common32960::getInstance()->m_AuthSignState)
        {
            m_protocol_vehicle_logout->DoVehLogout();
        }
        m_IsDoLogic = false;
    }
}

/******************************************************************************
 * NAME: UpdateSignalVal
 *
 * DESCRIPTION: update signal value
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter::UpdateSignalVal(IN SMLK_UINT32 index, IN SMLK_DOUBLE val, SMLK_BOOL valid)
{
    Post(
        MsgHead(SL_EVENT_DATAC_NOTIFY, index, valid ? 0x01 : 0x00, 0, sizeof(val), 0),
        reinterpret_cast<const SMLK_UINT8 *>(&val),
        sizeof(val)
    );
}

/******************************************************************************
 * NAME: UpdateDM1
 *
 * DESCRIPTION: update DM1 information
 *
 * PARAMETERS:
 *      info:   DM1 information
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter::UpdateDM1Info(IN ProtocolReporter::DM1::Info &info)
{
    Post(
        MsgHead(SL_EVENT_DM1_UPDATE, 0, 0, 0, sizeof(info), 0),
        reinterpret_cast<const SMLK_UINT8 *>(&info),
        sizeof(info)
    );
}

/******************************************************************************
 * NAME: UpdateGNSS
 *
 * DESCRIPTION: update GNSS information
 *
 * PARAMETERS:
 *      info:   GNSS information
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter::UpdateGNSS(IN ProtocolReporter::GNSS::Info &info)
{
    Post(
        MsgHead(SL_EVENT_LOCAT_NOTIFY, 0, 0, 0, sizeof(info), 0),
        reinterpret_cast<const SMLK_UINT8 *>(&info),
        sizeof(info)
    );
}

/******************************************************************************
 * NAME: SystemTimeSync
 *
 * DESCRIPTION: system time sync status notify
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter::SystemTimeSync(bool sync) {
    Post(
        MsgHead(SL_SYSTEM_TIME_NOTIFY, sync ? SystemTime::SL_MASK_SYNC : 0, 0, 0, 0, 0),
        nullptr,
        0
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
 *        sz:   data length
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter::Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz)
{
    m_events.emplace(
        head,
        ptr,
        sz,
        std::bind(&Reporter::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
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
void Reporter::Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_events.emplace(
        head,
        data,
        std::bind(&Reporter::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
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
void Reporter::Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data)
{
    m_events.emplace(
        head,
        std::move(data),
        std::bind(&Reporter::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
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
void Reporter::HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_id ) {
        case SL_EVENT_DATAC_NOTIFY:
            OnDataChanged(head, data);
            break;
        case SL_EVENT_DM1_UPDATE:
            OnDM1Notify(head, data);
            break;
        case SL_EVENT_LOCAT_NOTIFY:
            OnLocationNotify(head, data);
            break;
        case SL_SYSTEM_TIME_NOTIFY:
            OnSystemTimeNotify(head, data);
            break;
        default:
            break;
    }
}

/******************************************************************************
 * NAME: OnDataChanged
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
void Reporter::OnDataChanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_UINT32     index   = head.m_seq;
    bool            valid   = 0 != head.m_protocol;

    auto it = m_cached_index_signals.find(index);
    if ( m_cached_index_signals.end() == it ) {
        SMLK_LOGE("unregisted data change notification for index:   %u", index);
        return;
    } else {
        if ( it->second.Valid() && !valid ) {
            SMLK_LOGW("data changed:  VALID ---> INVALID, signal index: %u", index);
        } else if ( !it->second.Valid() && valid ) {
            // SMLK_LOGD("data changed:  INVALID ---> VALID, signal index: %u", index);
        }
    }

    it->second = *(reinterpret_cast<const double *>(data.data()));
    if ( !valid ) {
        it->second.Invalid();
    }
}

/******************************************************************************
 * NAME: OnLocationNotify
 *
 * DESCRIPTION: location information notification handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter::OnDM1Notify(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{
    if ( sizeof(SMLK_UINT32) + 19 * sizeof(DM1Evnet) > data.size() ) {
        SMLK_LOGE("invalid data size");
        return;
    }

    ProtocolReporter::DM1::Info *inf = (ProtocolReporter::DM1::Info *) data.data();

    vehicle::SignalVariant m_vehicle_speed                          = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVVehicleSpeed)->second;
    vehicle::SignalVariant m_service_brake_circuit_1_air_pressure   = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::ServiceBrakeCircuit1AirPressure)->second;
    vehicle::SignalVariant m_service_brake_circuit_2_air_pressure   = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::ServiceBrakeCircuit2AirPressure)->second;

    // SMLK_LOGD("m_service_brake_circuit_1_air_pressure = %d", m_service_brake_circuit_1_air_pressure.Encoded());
    // SMLK_LOGD("m_service_brake_circuit_2_air_pressure = %d", m_service_brake_circuit_2_air_pressure.Encoded());
    // SMLK_LOGD("m_vehicle_speed = %d", m_vehicle_speed.Encoded());

    inf->dm1event[13].errcode = 0;
    inf->dm1event[13].errvalue = 0;

    SMLK_UINT32 m_flags_1 = 0; 
    SMLK_UINT32 m_flags_2 = 0; 


    if ((m_vehicle_speed.Valid() && (m_vehicle_speed.Encoded() / 10 > 5)) || (is_braksysErr)) {
        if (m_service_brake_circuit_1_air_pressure.Valid()) {
            if (m_service_brake_circuit_1_air_pressure.Encoded() <= 0.25 * 1000) {
                m_flags_1 = 2;
            } else if ((m_service_brake_circuit_1_air_pressure.Encoded() > 0.25 * 1000) && (m_service_brake_circuit_1_air_pressure.Encoded() <= 0.55 * 1000)) {
                m_flags_1 = 1;
            } else if ((m_service_brake_circuit_1_air_pressure.Encoded() > 0.55 * 1000) && (m_service_brake_circuit_1_air_pressure.Encoded() <= 0.60 * 1000)) {
                if (m_press_1_warn_flags = 0) {
                    m_flags_1 = 0;
                } else if (m_press_1_warn_flags = 1) {
                    m_flags_1 = 1;
                } else if (m_press_1_warn_flags = 2) {
                    m_flags_1 = 1;
                }
            } else if (m_service_brake_circuit_1_air_pressure.Encoded() > 0.60 * 1000) {
                m_flags_1 = 0;
            }
        }
    } 
    m_press_1_warn_flags = m_flags_1;

    if ((m_vehicle_speed.Valid() && (m_vehicle_speed.Encoded() / 10 > 5)) || (is_braksysErr)) {
        if (m_service_brake_circuit_2_air_pressure.Valid()) {
            if (m_service_brake_circuit_2_air_pressure.Encoded() <= 0.25 * 1000) {
                m_flags_2 = 2;
            } else if ((m_service_brake_circuit_2_air_pressure.Encoded() > 0.25 * 1000) && (m_service_brake_circuit_2_air_pressure.Encoded() <= 0.55 * 1000)) {
                m_flags_2 = 1;
            } else if ((m_service_brake_circuit_2_air_pressure.Encoded() > 0.55 * 1000) && (m_service_brake_circuit_2_air_pressure.Encoded() <= 0.60 * 1000)) {
                if (m_press_2_warn_flags = 0) {
                    m_flags_2 = 0;
                } else if (m_press_2_warn_flags = 1) {
                    m_flags_2 = 1;
                } else if (m_press_2_warn_flags = 2) {
                    m_flags_2 = 1;
                }
            } else if (m_service_brake_circuit_2_air_pressure.Encoded() > 0.60 * 1000) {
                m_flags_2 = 0;
            }
        }
    } 
    m_press_2_warn_flags = m_flags_2;

    if (m_press_1_warn_flags > m_press_2_warn_flags) {
        inf->dm1event[13].errcode = m_press_1_warn_flags;
        inf->dm1event[13].errvalue = 0;
    } else {
        inf->dm1event[13].errcode = m_press_2_warn_flags;
        inf->dm1event[13].errvalue = 0;
    }

    if (inf->dm1event[13].errcode ) {
        is_braksysErr = true;
    } else {
        is_braksysErr = false;
    }

    for (int m = 0; m < 19; m++) {
        if (inf->dm1event[m].errcode != 0) {
            inf->warn_num++;
            inf->com_warn += (1 << m);
        }
        if (inf->dm1event[m].errcode > inf->max_warn)
            inf->max_warn = inf->dm1event[m].errcode;
    }

    // SMLK_LOGD("[dm1test] dm1info.max_warn:    %d", inf->max_warn);
    // SMLK_LOGD("[dm1test] dm1info.warn_num:    %d", inf->warn_num);
    // SMLK_LOGD("[dm1test] dm1info.com_warn:    %ld", inf->com_warn);

    common32960::getInstance()->m_dm1info.max_warn = inf->max_warn;
    common32960::getInstance()->m_dm1info.warn_num = inf->warn_num;
    common32960::getInstance()->m_dm1info.common_warn = inf->com_warn;
    std::memcpy(common32960::getInstance()->m_dm1info.dm1_event, inf->dm1event, M_GB_32960_SZ_RECHARGE_DEVICE_ERR_CNT_MAX * sizeof(DM1Evnet));

}

/******************************************************************************
 * NAME: OnLocationNotify
 *
 * DESCRIPTION: location information notification handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter::OnLocationNotify(IN MsgHead &/*head*/, IN std::vector<SMLK_UINT8> &data)
{

    if ( sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8) > data.size() ) {
        SMLK_LOGE("invalid data size");
        return;
    }

    ProtocolReporter::GNSS::Info *inf = (ProtocolReporter::GNSS::Info *) data.data();

    auto &flags  = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::LocatedStatus)->second;
    flags = inf->flags;

    auto &latitude  = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::Latitude)->second;
    latitude = inf->latitude;

    auto &longitude  = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::Longitude)->second;
    longitude = inf->longitude;
}

/******************************************************************************
 * NAME: OnSystemTimeNotify
 *
 * DESCRIPTION: system time notify event handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Reporter::OnSystemTimeNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
#ifdef SL_DEBUG
    SMLK_LOGD("OnSystemTimeNotify head.m_seq %x: head.result %x",head.m_seq,head.m_extra);
#endif
}

/******************************************************************************
 * NAME: HandleSendData2TSP32960
 *
 * DESCRIPTION: send message data to TSP
 *
 * PARAMETERS:  data:  message data to send
 *              length: message data length
 *
 * RETURN:      ret: send result
 *
 * NOTE:
 *****************************************************************************/
int Reporter::HandleSendData2TSP32960(char *data, uint length)
{
    std::lock_guard<std::mutex> locker(m_sendMtx);
    int ret = -1;
    if (m_TcpClient32960->m_connectState == normal) {
        // SMLK_LOGD("[T] Reporter::HandleSendData2TSP32960() sendMsg");
        ret = m_TcpClient32960->sendMsg(data, length);
    } else {
        SMLK_LOGD("[T] Reporter::HandleSendData2TSP32960() Connect Error!!! " );
    }
    return ret;
}

/**
 * @brief 收到当前socket的状态
 * @param state   state
 */
int Reporter::HandleSocketConnState(int state)
{
    common32960::getInstance()->setSocketState(state);
    return 0;
}

/**
 * @brief 监听TSP过来的消息
 * @param arg   this point
 */
void Reporter::ReceiveTSPData(void *arg)
{
    SMLK_LOGD("[T] Reporter::ReceiveTSPData() run" );
    Reporter *this_ptr = (Reporter *)arg;
    int retLen = 0;
    while(1) {
        if (m_TcpClient32960->m_connectState == normal) {
            char buf[M_GB_32960_SZ_DATA_MAX] = {0};
            retLen = 0;
            retLen = m_TcpClient32960->recMsg(buf, M_GB_32960_SZ_DATA_MAX);
            if(retLen > 0) {
                this_ptr->m_MessageOb->decodeMsg(buf, retLen);
            }
        } else {
        // SMLK_LOGD("[T] Reporter::ReceiveTSPData() Connect Error!!! " );
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/**
 * @brief Socket 状态监听
 */
void Reporter::SocketMonitor(void *arg)
{
    int count = 0;
    bool b_logout = false;

    Reporter *this_ptr = (Reporter *)arg;
    // SMLK_LOGD("[T] Reporter::SocketMonitor() run" );

    while(1)
    {
        if(m_TcpClient32960->m_connectState == disconnect && common32960::getInstance()->getIgnState()) {
            SMLK_LOGD("we lost the connection ");
            common32960::getInstance()->m_AuthSignState = false;
            int ret = m_TcpClient32960->connect();
        }

        if (m_TcpClient32960->m_connectState == normal) {
            if (!common32960::getInstance()->m_AuthSignState && common32960::getInstance()->getIgnState()) {
                if (common32960::getInstance()->m_run_mode == 0x01 || common32960::getInstance()->m_run_mode == 0x02 || common32960::getInstance()->m_run_mode == 0x03) {
                    SMLK_LOGD("we got a connection notificaiton, do vehicle login ");
                    this_ptr->m_protocol_vehicle_login->DoVehicleLogin();
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
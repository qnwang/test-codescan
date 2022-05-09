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
#pragma once
#include <mutex>
#include <thread>
#include <memory>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <ipc_head.h>
#include <message_inf.h>
#include <smlk_timer_inf.h>
#include <smlk_signal.h>
#include <smlk_service_inf.h>
#include <smlk_queue.h>

#include <smartlink_sdk_tel.h>

#include "callable_event.h"
#include "ev_signals.h"

#include "message_def_32960.h"

#include "message_vehiclelogin_32960.h"
#include "message_vehiclelogout_32960.h"
#include "message_blindinfo_32960.h"
#include "message_realtimeinfo_32960.h"
#include "message_heartbeat_32960.h"

#include "common_32960.h"

namespace smartlink {

namespace ProtocolReporter {

namespace GNSS {
namespace Flags {
    // static const SMLK_UINT32    SL_FIXED                = 0x00000001;
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

namespace DM1 {
struct Info {
    SMLK_UINT8      max_warn;
    SMLK_UINT32     com_warn;
    SMLK_UINT8      warn_num;
    DM1Evnet        dm1event[19];      
};
};

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

class Reporter : public IService {
public:
    Reporter() = default;
    Reporter(IN std::string &host, IN SMLK_UINT16 port);
    ~Reporter();

    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    void            InitLogic();
    void            Stop();
    bool            IsRunning() const;

    SMLK_UINT32     SetCONFIG(IN SMLK_UINT8 config);
    SMLK_UINT32     SetTORQUE(IN SMLK_UINT16 torque);
    SMLK_UINT32     SetVIN(IN std::string &vin);
    SMLK_UINT32     SetICCID(IN std::string &iccid);
    SMLK_UINT32     SetIsSave(IN SMLK_BOOL save);
    SMLK_UINT32     SetStandbyState();
    SMLK_UINT32     SetTelephonyState(IN ProtocolReporter::telephony::Info &info);
    void            IgnON();
    void            IgnOFF();

    void UpdateSignalVal(IN SMLK_UINT32 index, IN SMLK_DOUBLE val, IN SMLK_BOOL valid = true);
    void UpdateDM1Info(IN ProtocolReporter::DM1::Info &info);
    void UpdateGNSS(IN ProtocolReporter::GNSS::Info &info);
    void SystemTimeSync(bool = true);

    void Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz);
    void Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data);
private:
    void HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDataChanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDM1Notify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnLocationNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnSystemTimeNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDIDNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    static int HandleSendData2TSP32960(char *data, uint length);
    static int HandleSocketConnState(int state);
    static void ReceiveTSPData(void *arg);
    static void SocketMonitor(void *arg);

    static TCPClient *m_TcpClient32960;
    static SqlLiteBase *m_SqlLiteBase32960;

private:
    std::string     m_host;
    SMLK_UINT16     m_port;

    SMLK_UINT32     m_flags;
    SMLK_UINT32     m_press_1_warn_flags;
    SMLK_UINT32     m_press_2_warn_flags;
    SMLK_UINT32     m_reconnect_period;
    SMLK_BOOL       m_first_init;
    SMLK_BOOL       m_IsDoLogic;

    SMLK_BOOL       is_braksysErr;

    MessageObserver32960            *m_MessageOb = nullptr;
    MessageVehicleLogin32960        *m_protocol_vehicle_login = nullptr;
    MessageVehLogout32960           *m_protocol_vehicle_logout = nullptr;
    MessageRealTimeInfo32960        *m_protocol_realtime_report = nullptr;
    MessageBlindInfoUp32960         *m_protocol_blindinfo_report = nullptr;
    MessageHeartBeat32960           *m_protocol_heartbeat = nullptr;

    std::mutex                      m_mutex;
    static std::mutex               m_sendMtx;
    std::thread                     m_thread;
    Queue<CallableEvent>            m_events;
    std::unordered_map<SMLK_UINT32, vehicle::SignalVariant &>         m_cached_index_signals;
    std::mutex                      m_initMutex;
};

}
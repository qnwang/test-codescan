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

#include <string>
#include <thread>
#include <future>

#include "recorder.h"
#include "callable_event.h"

#include "did_ipc_api.h"

#include "common_17691.h"

#include "TCPClient.h"

namespace smartlink {

class Reporter : public IService {
public:
    Reporter() = delete;
    Reporter(IN std::string &host, IN SMLK_UINT16 port, IN SMLK_UINT8 protocol, IN SMLK_UINT8 platform);
    ~Reporter();

    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    void            Stop();
    bool            IsRunning() const;

    SMLK_UINT32     SetProductLoc(IN SMLK_UINT8 loc);
    SMLK_UINT32     SetReportInfo(IN SMLK_BOOL gas, IN SMLK_BOOL hev);
    SMLK_UINT32     SetVIN(IN std::string &vin);
    SMLK_UINT32     SetChipID(IN std::string &chipid);
    SMLK_UINT32     SetActStatus(IN SMLK_UINT32 state);
    SMLK_UINT32     SetICCID(IN std::string &iccid);
    SMLK_UINT32     SetRecorderDir(IN std::string &dir);
    SMLK_UINT32     SetIsSave(IN SMLK_BOOL save);
    SMLK_UINT32     SetTelephonyState(IN ProtocolReporter::telephony::Info &info);
    SMLK_UINT32     SetTimeSyncGap(IN SMLK_UINT64 timesyncgap);

    void UpdateDiagProtocol(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &protocol);
    void UpdateVIN(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &vin);
    void UpdateSCIN(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &scin);
    void UpdateCVN(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &cvn);
    void UpdateIUPR(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &iupr);
    void UpdateOBD(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &obd);
    void UpdateDTC(SMLK_UINT16 result,const std::vector<SMLK_UINT8> &dtc);
    void UpdateSignalVal(IN SMLK_UINT32 index, IN SMLK_DOUBLE val, IN SMLK_BOOL valid = true);
    void UpdateGNSS(IN ProtocolReporter::GNSS::Info &info);
    void UpdateDayChange();

    void SwitchToMainPower();
    void SwitchToStandyPower();
    void IgnON();
    void IgnOFF();

    void Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz);
    void Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data);

private:
    void HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDataChanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnLocationNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnSystemTimeNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDIDNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDTCNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDayChangeNotify();
    void OnTimer(IN SMLK_UINT32 id, IN SMLK_UINT32 seq);
    void OnGenericTimer(IN SMLK_UINT32 seq);
    void OnSupplementTimer(IN SMLK_UINT32 seq);
    void OnLogin();
    void OnLogout();

    void DoVehicleLogin();
    void DoVehicleLogout();
    void SamplingEngineInfo();
    void ClearExpiredRecord();

    SMLK_INT32 HandleSocketConnState(std::string &task, SMLK_INT32 state);
    SMLK_INT32 HandleSendDataToTSP(SMLK_UINT8 *data, SMLK_UINT32 length);

private:
    SMLK_UINT32     m_service_flags;
    SMLK_UINT32     m_internal_flags;
    SMLK_UINT32     m_timer_flags;
    SMLK_UINT32     m_vin_flags;
    SMLK_UINT32     m_seq;
    SMLK_UINT32     m_report_period;
    SMLK_UINT64     m_time_sync_gap;
    SMLK_UINT8      m_protocol_version;
    SMLK_UINT8      m_platform_type;
    SMLK_UINT32     m_reprot_first_login;
    SMLK_UINT32     m_sampling_period;
    SMLK_UINT32     m_supplement_period;
    SMLK_UINT32     m_reconnect_period;
    SMLK_UINT32     m_expired_record_clear_period;
    SMLK_UINT32     m_report_index;
    SMLK_UINT32     m_reprot_index_engine;
    SMLK_UINT16     m_port;
    std::string     m_host;
    std::string     m_vin;
    std::string     m_iccid;
    std::string     m_db_dir;
    std::string     m_taskName;
    std::string     m_chipid;

    SMLK_UINT32     m_actstate;

    SMLK_UINT8      m_entnet_state;
    SMLK_UINT8      m_locnet_state;

    SMLK_BOOL       m_todisk;
    SMLK_UINT8      m_loc;
    SMLK_BOOL       is_ng;
    SMLK_BOOL       is_hev;

    std::mutex      m_mutex;
    std::thread     m_thread;
    static std::mutex               m_sendMtx;

    TCPClient       *m_TcpClient17691;
    common17691     *m_comm17691;

    std::shared_ptr<IMessage>       m_vehicle_login;
    std::shared_ptr<IMessage>       m_vehicle_logout;
    std::shared_ptr<IMessage>       m_report;
    std::shared_ptr<IMessage>       m_obd_info;
    std::shared_ptr<IMessage>       m_engine_info;
    std::shared_ptr<IMessage>       m_engine_hev_info;

    std::shared_ptr<IMessage>       m_protocol_vehicle_login;
    std::shared_ptr<IMessage>       m_protocol_vehicle_logout;
    std::shared_ptr<IMessage>       m_protocol_report;
    std::shared_ptr<IMessage>       m_protocol_supplement;

    std::shared_ptr<ITimer>         m_timer_rt;
    std::shared_ptr<ITimer>         m_timer_sup;

    std::shared_ptr<Recorder>       m_recorder;
    std::shared_ptr<Recorder>       m_recorder_sup;

    std::future<void> future;

    Queue<CallableEvent>            m_events;

    std::unordered_map<SMLK_UINT32, vehicle::SignalVariant>         m_cached_signals;
    std::unordered_map<SMLK_UINT32, vehicle::SignalVariant &>       m_cached_index_signals;

};

};

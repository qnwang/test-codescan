/**
 * @file protocol_reporter_1239_07.h
 * @author T (taoguanjie@smartlink.com.cn)
 * @brief 
 * @version 0.1
 * @date 2022-04-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include <mutex>
#include <thread>
#include <memory>
#include <chrono>
#include <condition_variable>
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

#include "common_17691.h"

#include "TCPClient.h"

namespace smartlink
{

    class Reporter1239_07 : public IService
    {
    public:
        Reporter1239_07() = delete;
        Reporter1239_07(IN std::string &host, IN SMLK_UINT16 port, IN SMLK_UINT8 protocol, IN SMLK_UINT8 platform);
        ~Reporter1239_07();

        SMLK_UINT32 Init();
        SMLK_UINT32 Start();
        void Stop();
        bool IsRunning() const;

        SMLK_UINT32 SetProductLoc(IN SMLK_UINT8 loc);
        SMLK_UINT32 SetReportInfo(IN SMLK_BOOL gas, IN SMLK_BOOL hev);
        SMLK_UINT32 SetActiveAddr(IN std::string &host, IN SMLK_UINT16 port);
        SMLK_UINT32 SetVIN(IN std::string &vin);
        SMLK_UINT32 SetActStatus(IN SMLK_UINT32 state);
        SMLK_UINT32 SetChipID(IN std::string &chipid);
        SMLK_UINT32 SetPubKey(IN std::string &pubkey);
        SMLK_UINT32 SetICCID(IN std::string &iccid);
        SMLK_UINT32 SetIsSave(IN SMLK_BOOL save);
        SMLK_UINT32 SetTelephonyState(IN ProtocolReporter::telephony::Info &info);
        SMLK_UINT32 SetTimeSyncGap(IN SMLK_UINT64 timesyncgap);

        void UpdateVIN(SMLK_UINT16 result, const std::vector<SMLK_UINT8> &vin);
        void UpdateSignalVal(IN SMLK_UINT32 index, IN SMLK_DOUBLE val, IN SMLK_BOOL valid = true);
        void UpdateGNSS(IN ProtocolReporter::GNSS::Info &info);
        void SwitchToMainPower();
        void SwitchToStandyPower();
        void IgnON();
        void IgnOFF();
        void VehicleActivate(IN SMLK_UINT32 act);

        void Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz);
        void Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
        void Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data);

    private:
        void HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
        void OnDataChanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
        void OnLocationNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
        void OnSystemTimeNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
        void OnDIDNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
        void OnTimer(IN SMLK_UINT32 id, IN SMLK_UINT32 seq);
        void OnGenericTimer(IN SMLK_UINT32 seq);

        void OnLogin();
        void OnLogout();
        void OnActivate();

        void OnReceiveData(char *src, uint length);

        void DoVehicleActivate();

        SMLK_INT32 HandleSocketConnState(std::string &task, SMLK_INT32 state);
        SMLK_INT32 HandleSendDataToTSP(SMLK_UINT8 cmd, SMLK_UINT8 *data, SMLK_UINT32 length);
        void HandleReceiveDataFromTSP(void );

    private:
        SMLK_UINT32 m_service_flags;
        SMLK_UINT32 m_internal_flags;
        SMLK_UINT32 m_timer_flags;
        SMLK_UINT32 m_vin_flags;
        SMLK_UINT32 m_seq;
        SMLK_UINT32 m_report_period;
        SMLK_UINT64 m_time_sync_gap;
        SMLK_UINT8 m_protocol_version;
        SMLK_UINT32 m_reconnect_period;
        SMLK_UINT32 m_report_index;
        SMLK_UINT32 m_reprot_index_engine;
        SMLK_UINT16 m_port;
        std::string m_host;
        std::string m_taskName;
        SMLK_BOOL m_todisk;
        SMLK_UINT8 m_loc;
        std::string m_vin;
        std::string m_chipid;
        std::string m_pubkey;
        std::mutex m_mutex;
        std::thread m_thread;
        std::thread m_rec_thread;
        static std::mutex m_sendMtx;
        static std::mutex m_actMtx;
        static std::condition_variable cv;
        common17691 *m_comm1239;
        TCPClient *m_TcpClient;
        std::shared_ptr<ITimer> m_timer;
        std::shared_ptr<IMessage> m_vehicle_activate;
        std::future<void> future;
        Queue<CallableEvent> m_events;
    };
};
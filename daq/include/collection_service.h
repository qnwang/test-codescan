#pragma once
#include <mutex>
#include <thread>
#include <chrono>
#include <set>
#include <vector>
#include <utility>
#include <unordered_map>
#include <smlk_types.h>
#include <smlk_queue.h>
#include <smlk_fixed_queue.h>
// #include <smlk_timer_inf.h>
#include <smlk_timer_new.h>
#include <smlk_service_inf.h>
#include <sl_signals.h>
#include <callable_event.h>
#include <message_inf.h>
#include "daq_sqldb.h"
#include "Poco/Timer.h"
namespace smartlink {

#define M_CS_CACHED_ITEMS_MAX       600
#define M_SL_CACHED_AP_POSITION_MAX 20                  // Accelerator pedal (AP) position  cyclic 50ms, so we need 20 frames
#define SETBIT(x,y)  x|=(1<<y)
#define CLRBIT(x,y)  x&=~(1<<y)

#define SETBIT_PARAM(x,y)  x | (1<<y)
#define CLRBIT_PARAM(x,y)  x & ~(1<<y)
class CollectionService : public IService {
public:
    CollectionService();
    virtual ~CollectionService();

    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    void            Stop();
    bool            IsRunning() const;

    void Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz);
    void Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data);
    void PostDM1(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void PostDM1(IN MsgHead &head, IN std::vector<SMLK_UINT8> &&data);
    void PostDM1(IN MsgHead &head, IN SMLK_UINT8 *data, IN std::size_t sz);


private:
    void HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void DM1HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    void OnMessage(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnRequest(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnCompress(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDataChanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnTireNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnTimer(IN SMLK_UINT32 id, IN SMLK_UINT32 param);
    void InitTimerOutCheck();

    void OnMcuNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnLocation(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnSysPower(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnTelNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    void OnSystimeSync(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnSystimeReady(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    void OnIgnOn(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnIgnOff(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    void OnCanBusWakeup(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnCanBusSleep(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    void OnDriveEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDM1Notify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnCheckDataValidTimeout();

    bool CheckWhetherTimeSync();
    bool CheckWhetherInPeriodWakeup();
    bool CheckSamplingCondition();
    bool CheckLocationReportContidion();
    bool Check1sReportContidion();
    bool Check30sReportContidion();
    void ReadTimeSource();

    void OnDriveEventSharpSlowdown(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDriveEventSpeeding(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDriveEventAccident(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDriveEventTireWarn(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDriveEventTirednessDriving(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDriveEventLowSOC(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDriveEventDMSWarn(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDriveEventFuelTankWarn(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDriveEventAgitatorFault(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDriveEventDisMantleTbox(IN MsgHead &head);
    void OnDriveEventGbCheckVinError(IN MsgHead &head);
    void OnDriveEventRefrigeratorFailure(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnFaultEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnFaultEventDM1(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void CompressPost(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data);
    void HandleGbCheckVinResult();

    void ReadNetWorkTypeConfig();
    void ReadCarTypeConfig();
    void ReadEvEmissionConfig();
    void CheckWireConnectStatus(SMLK_UINT8 nCheckItem);
    void HandleSampling1SCollectConfig();
    void HandleSampling1SUploadConfig();
    void HandleStatisticsConfig();
    void HandleWakeupInterval();
    void HandleFinaceLockStatus();
    void OnUpdateJsonConfig();
    void OnReadJsonConfig();

    void SamplingPeriod1S();
    void SamplingPeriod30S();
    void SamplingLocation();
    void CacheAllSignals();
    void GetGNSS();

    void ReportLocation();
    void ReportStatistics();
    void StoreStatistics();
    void OnDayChanged();

    SMLK_UINT8 ClearCanDataType();
    bool CheckWhichCanDataStop();
    void SetAllCanSignalInvalid(SMLK_UINT8 netWorkType);

#ifdef SL_SIMULATOR
    void GenerateRandomValues();
    void SimulatorWorker();
    void OnSimulatorRecv(const SMLK_UINT8 *data, std::size_t sz);
#endif

private:
    DaqSqlDB        m_sqldb;/*数据库*/
    std::mutex      m_sql_mutex;/*操作数据库的锁*/
    SMLK_UINT32     m_flags;
    SMLK_UINT32     m_internal_flags;
#ifdef VEHICLE_VKA
    SMLK_UINT32     m_vka_flags;
#endif
    SMLK_UINT32     m_timer_flags;
    SMLK_UINT8      m_dismantle_tbox_status;
    SMLK_UINT32     m_report_period_default;
    SMLK_UINT32     m_report_period_emergency;
    SMLK_UINT32     m_report_period;
    SMLK_UINT32     m_seq;
    SMLK_UINT32     m_bucket_num_1s;
    SMLK_UINT32     m_bucket_num_30s;
    SMLK_UINT32     m_wakeup_report_cnt;
    SMLK_UINT32     m_wakeup_report_period;
    SMLK_UINT32     m_trigger_period_30s;
    SMLK_UINT32     m_trigger_period_1s_ms;

    SMLK_UINT32     m_time_source;
    SMLK_INT64      m_timeSyncGap_ms;
    SMLK_UINT32     m_time_sync_flags;


    SMLK_UINT32     m_upload_period_1s_ms;
    SMLK_UINT32     m_statistics_report_period;
    SMLK_UINT32     m_statistics_store_period;
    SMLK_UINT8      i_lckcar_sts;
    SMLK_UINT8      m_door_signal_status;
    SMLK_UINT32     m_collect_last_frame_flag;
    std::string     m_clock_id;
    std::mutex      m_mutex;
    std::thread     m_thread;
    std::thread     m_dm1_thread;

    SMLK_BOOL       m_ig_on_to_off;
    SMLK_UINT32     m_ig_sample_count;
    std::mutex      m_cache_map_mutex;

    std::thread     m_compress_thread;
    std::thread     m_simulator_thread;
    std::shared_ptr<ITimer> m_timer;
    std::shared_ptr<ITimer> m_0f3c_timer; // statistics data
    std::shared_ptr<ITimer> m_0f3b_timer; // dense type of data
    std::shared_ptr<ITimer> m_0f3b_30s_timer; // dense type of data 30s
    std::shared_ptr<ITimer> m_0200_timer; // location data
    std::shared_ptr<ITimer> m_dm1_timeout_timer;

    std::unordered_map<SMLK_UINT8, SMLK_UINT32&>   m_frequency_value;    //key: msg type, value: real value


    std::shared_ptr<ITimer> m_timer_shared;
    Queue<CallableEvent>    m_events;
    Queue<CallableEvent>    m_dm1_events;

    Queue<CallableEvent>    m_compress_events;

    std::vector<SMLK_UINT8>     m_protocols;
    using ProtocolMessage = std::unordered_map<SMLK_UINT8, std::shared_ptr<IMessage>>;
    std::unordered_map<SMLK_UINT8, ProtocolMessage>                 m_protocol_messages;
    std::unordered_map<SMLK_UINT8, std::shared_ptr<IMessage>>       m_cached_tires;
    std::unordered_map<vehicle::SignalID, vehicle::SignalVariant>   m_cached_signals;
    std::unordered_map<SMLK_UINT32, vehicle::SignalVariant &>       m_cached_index_signals;
    std::unordered_map<SMLK_UINT32, vehicle::SignalVariant &>       m_cached_tires_signals;
    std::unordered_map<SMLK_UINT32, vehicle::SignalVariant &>       m_cached_tires_ext_signals;
    typedef union {
        SMLK_UINT32 id;
        struct {
            SMLK_UINT32 SPN:19;
            SMLK_UINT32 FMI:5;
            SMLK_UINT32 SRC:8;
        }           dtc;
    } InternalDTC;


#if 1
    using DM1_SPN        = SMLK_UINT32;
    using DM1_SRC        = SMLK_UINT8;
    struct DM1DataInfo
    {
        SMLK_BOOL     data_valid;
        SMLK_UINT32   cycle;                          // can raw data cycle ms
        std::chrono::steady_clock::time_point tp;     // timepoint for data received
        SMLK_UINT8   can_src;
        SMLK_UINT8   dm1_fmi;
    };
    std::unordered_map<DM1_SPN, DM1DataInfo> g_dm1_data_pool;
    std::mutex      g_dm1_data_pool_mutex;
    std::mutex      g_dm1_trip_uploaded_mutex;
    Poco::Timer     m_check_timeout_timer;
    // std::unordered_map<DM1_SPN, std::set<SMLK_UINT32>> g_dm1_uploaded_map;
    std::unordered_map<DM1_SRC, std::vector<DM1_SPN>>               m_dm1_src_all_spn_map;        // spn按源地址分类

    std::set<SMLK_UINT32>       m_actived_dm1;
    std::set<DM1_SPN>           m_trip_uploaded_spn;
#endif
    using   steady_time_point   = std::chrono::time_point<std::chrono::steady_clock>;
    using   cached_item_hash    = std::unordered_map<vehicle::SignalID, vehicle::SignalVariant>;
    FixedQueue<std::pair<steady_time_point, cached_item_hash>, M_CS_CACHED_ITEMS_MAX>   m_cached_signals_pool;
    FixedQueue<std::pair<steady_time_point, SMLK_UINT16>, M_SL_CACHED_AP_POSITION_MAX>  m_cached_ap_positions;
    std::unordered_map<SMLK_UINT8, std::unordered_map<SMLK_UINT32, std::shared_ptr<IMessage>>>  m_pending_events;
};  // class CollectionService

}   // namespace smartlink

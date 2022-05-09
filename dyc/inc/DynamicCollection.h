#pragma once
#include "dyc_util.h"
#include "ipc_head.h"
#include "callable_event.h"
// #include "smlk_timer_inf.h"
// #include "smlk_timer.h"
#include <smlk_signal.h>
#include "SignalFormat.h"
#include "Poco/Net/HTTPClientSession.h"
#include "Poco/Net/HTTPRequest.h"
#include "Poco/Net/HTTPResponse.h"
#include "Poco/Net/HTTPCredentials.h"
#include "Poco/StreamCopier.h"
#include "Poco/NullStream.h"
#include "Poco/Path.h"
#include "Poco/URI.h"
#include "Poco/Exception.h"
#include <iostream>
#include "DbcConfigFormat.h"
#include "SignalFormat.h"
#include "dyc_config.h"
#include "message_inf.h"
#include "filetool.h"
#include "vec_signals.h"
using Poco::Net::HTTPClientSession;
using Poco::Net::HTTPRequest;
using Poco::Net::HTTPResponse;
using Poco::Net::HTTPMessage;
using Poco::StreamCopier;
using Poco::Path;
using Poco::URI;
using Poco::Exception;

using MessageDecodeMap = std::unordered_map<CAN_ID, std::pair<CanFormatData::CanMessageInfo, std::vector<SignalFormat>>>;

namespace smartlink {

#define M_CS_CACHED_ITEMS_MAX       600
#define M_SL_CACHED_AP_POSITION_MAX 20
#define SETBIT(x,y)  x|=(1<<y)
#define CLRBIT(x,y)  x&=~(1<<y)


class DynamicCollectionService : public IService {
public:
    DynamicCollectionService();
    virtual ~DynamicCollectionService();

    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    void            Stop();
    bool            IsRunning() const;
    void Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz);
    void Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data);

private:
    void HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    void OnMcuStateNotify(IN MsgHead &head);
    void OnCanBusWakeup(IN MsgHead &head);
    void OnCanBusSleep(IN MsgHead &head);

    void OnIgnOn(IN MsgHead &head);
    void OnIgnOff(IN MsgHead &head);

    void OnDycConfig(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    void OnTspSetDycInterval(IN MsgHead &head, IN std::vector<SMLK_UINT8> &tsp_data);
    void ResponseTspSetDycInterval(IN MsgHead &head);
    void OnTspReadDycInterval(IN std::vector<SMLK_UINT8> &data);
    void ResponseTspReadDycInterval(IN MsgHead &head);

    void OnTspSendDycDbc(IN MsgHead &head, IN std::vector<SMLK_UINT8> &tsp_data);
    void ResponseTspSendDycDbc(IN MsgHead &head);
    int  HttpGet(std::string &address, std::string &true_name);

    void OnTspReadDycConfig(IN MsgHead &head);
    void ResponseTspReadDycConfig(IN MsgHead &head);

    bool OnTspClearDycConfig(IN std::vector<SMLK_UINT8> &data);
    void ResponseTspClearDycConfig(IN MsgHead &head, IN std::vector<SMLK_UINT8> &tsp_data);
    void SendTspCommonResponse(IN MsgHead &head,IN SMLK_UINT8 &result);
    bool DoExecuteClearDbcConfig(IN std::vector<string> &str_vec);

    void OnTimer(IN MsgHead &head);
    void CheckSampleCondition(IN MsgHead &head);
    void SampleSignalsByDbcIndex(IN MsgHead &head, IN DBC_INDEX &index);
    void InitVehicleDataPool();
    void CanFrameAceept(const CanFormatData::CanFrame* can_frame);
    void ProcessNormalCanFrame(const CanFormatData::CanFrame* can_frame, const std::vector<SignalFormat> &vec_signals);
    void DM1CanFrameAceept(const std::vector<SMLK_UINT8> &data);
    void ProcessPhysicalData(Index index, CanFormatData::CanPhysicalDataInfo& info);
    void OnCanDatachanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnLocation(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void GetGNSS();
    void onNewDbcSet(IN MsgHead &head);
    void LoadDbcConfigByIndex(IN DBC_INDEX &index);
    void UpdateDbcConfigByIndex(IN DBC_INDEX &index, IN dbc_config_struct &info);

    std::size_t CompressOffset() const;
    static size_t write_data(void *ptr, size_t size, size_t nmemb, void *stream);
    void OnDataChanged(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnZipNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void InitSearchDbcConfig();
    void BuildAndUpdateMaps();
    void OnCompress(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    std::uint32_t Encode(std::vector<SMLK_UINT8> &data , IN DBC_INDEX &index);
    std::uint32_t EncodeZip(std::vector<SMLK_UINT8> &data);

    Queue<CallableEvent>    m_events;
    std::thread             m_can_thread;

    Queue<CallableEvent>    m_compress_events;
    // 进程状态标志
    SMLK_UINT32     m_flags;
    std::mutex      m_mutex;
    std::thread     m_thread;
    std::thread     m_compress_thread;
    SMLK_UINT32     m_check_period;
    SMLK_UINT32     m_upload_id;


    std::shared_ptr<smartlink::ITimer>     m_timer_long_miss;
    std::shared_ptr<ITimer> m_timer;
    std::shared_ptr<ITimer> m_timer_shared;

    std::vector<SMLK_UINT8>     m_protocols;
    using ProtocolMessage = std::unordered_map<SMLK_UINT8, std::shared_ptr<IMessage>>;
    std::unordered_map<SMLK_UINT8, ProtocolMessage>                 m_protocol_messages;
    // 应用状态标志
    SMLK_UINT32                 m_internal_flags;
    // std::unordered_map<CAN_ID, std::pair<CanFormatData::CanMessageInfo, std::vector<SignalFormat>>> g_message_decode_map;
    std::mutex                                                          m_message_decode_map_mutex;       // 缓存中的dbc采集频率的锁
    MessageDecodeMap                                                    g_message_decode_map;             // 需要采集的所有数据的所有必要信息 低频更新 都在主线程处理

    std::mutex                                                          g_vehicle_data_pool_mutex;       // 信号解析的锁，在dbc更新时更新
    std::unordered_map<SPN_INDEX, CanFormatData::CanPhysicalDataInfo>   g_vehicle_data_pool;             // can信号解析使用

    std::unordered_map<SPN_INDEX, SignalFormat>                         m_cached_signals;                // 上报数据的数据源，can  data  变更后 g_vehicle_data_pool的更新目标
    std::unordered_map<SMLK_UINT32, vehicle::SignalVariant>             m_vec_signals;                   // 从vehicleData订阅的统计数据
    std::unordered_map<SMLK_UINT32, vehicle::SignalVariant>             m_gnss_signals;                  // 包头填充的gnss数据

    std::mutex                                                          m_dbc_config_mutex;              // 缓存中的dbc采集频率的锁 低频更新
    std::unordered_map<DBC_INDEX, DbcConfigFormat>                      m_dbc_config;                    // 下发后解析到缓存配置

    std::mutex                                                          m_dbc_sample_siganls_mutex;      // 缓存中的dbc对应所有信号map的锁 低频更新
    std::unordered_map<DBC_INDEX, std::vector<SPN_INDEX>>               m_dbc_sample_signals_map;        // timer触发时的采集依据

    std::unordered_map<SPN_INDEX, SignalFormat>                         m_sampled_signals;               // 采集的数据
    std::mutex                                                          m_sampled_frame_mutex;              // 缓存中的dbc采集频率的锁 低频更新

    std::unordered_map<DBC_INDEX, std::vector<std::unordered_map<SPN_INDEX, SignalFormat>>>    m_sampled_frame_signals; // 每个DBC的帧

    std::vector<SMLK_UINT8>                                             m_compressed;
    std::vector<std::string>                                            m_dbc_name_vec;

    SMLK_UINT16             m_sampling_frequency;
    SMLK_UINT8              m_crompression;
    std::mutex              m_frameLimit_mutex;              // 缓存中的dbc采集频率的锁 低频更新
    SMLK_UINT32             m_current_frameLimit;
    SMLK_UINT8              m_upload_data_type;
    SMLK_UINT8              m_tsp_connect_state;

    vehicle::SignalVariant  m_signal_day_vehicle_distance;
    vehicle::SignalVariant  m_signal_total_vehicle_distance;
    vehicle::SignalVariant  m_signal_ecu_total_distacne;
    vehicle::SignalVariant  m_signal_day_fuel_used;
    vehicle::SignalVariant  m_signal_total_fuel_used;
    vehicle::SignalVariant  m_signal_ecu_total_fuel_used;

};  // class DynamicCollectionService

}   // namespace smartlink

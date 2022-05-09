/*****************************************************************************/
/**
 * \file       remote_control.h
 * \author     huangxin
 * \date       2020/10/27
 * \version    Tbox2.0 V1
 * \brief      remote control service class
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks
 ******************************************************************************/
#ifndef _VEHICLE_CONTROL_H_
#define _VEHICLE_CONTROL_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <iostream>
#include <thread>
#include <map>
#include <mutex>
#include <vector>
#include <queue>
#include <atomic>
#include "future"
#include <condition_variable>

#include "smlk_tools.h"
#include "smlk_error.h"
#include "smlk_types.h"
#include "smlk_queue.h"
#include "vehctrl_service_inf.h"
#include "vehctrl_configfile.h"
#include "vehctrl_inf.h"
#include "vehctrl_jtt808.h"
#include "ipc_service_api.h"
#include "vehicle_data_api.h"
#include "smartlink_sdk_tel.h"
#include "did_ipc_api.h"

#include "smartlink_sdk_sys_power.h"
#include "smartlink_sdk_rtn_code.h"

#include <smartlink_sdk_mcu.h>
#include <smlk_timer_new.h>

#include "Poco/Event.h"
#include "Poco/Dynamic/Struct.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/Dynamic/Pair.h"
#include "Poco/Dynamic/VarIterator.h"
#include "Poco/JSON/Array.h"
#include "Poco/JSON/Parser.h"

namespace smartlink
{
    /* 车辆控制管理类(远程控车,近程控车) */
    class VehicleCtrlService : public IService
    {
    public:
        /*构造析构*/
        VehicleCtrlService();
        virtual ~VehicleCtrlService();
        /*进程状态维护*/
        SMLK_RC Init();
        SMLK_RC Start();
        void Stop();
        bool IsRunning() const;

    private:
        /***  基础方法 ***/
        /*初始化*/
        SMLK_RC InitStatusFile();
        /*消息监听处理函数*/
        void OnVehicleChangedIndication(VehicleData data);
        void OnMcuSerIndication(smartlink_sdk::McuEventId id, void *data, int len);
        void OnTelIndication(smartlink_sdk::TelEventId id, void *data, int len);
        void OnTspIndication(IN IpcTspHead &ipc_head, IN void *data, std::size_t len);
        void OnSysPowerSerIndication(smartlink_sdk::SysPowerEventID id, void *data, int len);
        /*消息处理线程*/
        void ThreadMsgFromProcess(void);
        void ThreadControlQueryProcess(void);
        void ThreadAwakenProcess(void);
        void ThreadOfflineProcess(void);
        void ThreadCanBusWakeUpProcess(void);
        /*消息处理方法*/
        void ProcessMcuSerResp(IN void *data, IN int length);                                                                     /*MCU控车结果处理*/
        SMLK_RC ProcessMcuCtrlAutoReport(IN void *data, IN int length);                                                           /*MCU控车主动上报处理--定时熄火&紧急熄火上报tsp*/
        SMLK_RC ProcessTspMessage(IN RctrlMsgQueue &msg_queue, IN std::shared_ptr<IDissector> &dissector);                        /*TSP消息通过消息队列分发到解析器*/
        SMLK_RC PreProcAutoReportDataChanged(IN RctrlMsgQueue &msg_queue);                                                        /*主动上报数据预处理*/
        SMLK_RC ProcessAutoReportDataChanged(IN RctrlMsgQueue &msg_queue, IN std::shared_ptr<IDissector> &dissector);             /*主动上报数据处理*/
        SMLK_RC ProcessAutoReportFromOther(IN SmlkRctrlInnerMsgID id, IN SMLK_UINT8 data);                                        /*主动上报其他消息源处理*/
        SMLK_RC Distribute_8F41(IN std::vector<SmlkMcuCmd> &mcu_cmd_vec, OUT RctrlResultQueue &result);                           /*分配控车指令*/
        SMLK_RC Process_8F41_Cmd(IN SmlkMcuCmd &mcu_cmd, IN RemoteCtrlConfigFileInfo &config_info, OUT RctrlResultQueue &result); /*控车单指令处理*/
        SMLK_RC Process_8F41_Cmd(IN SmlkMcuCmd &mcu_cmd, OUT RctrlResultQueue &result);                                           /*控车单指令处理*/
        SMLK_RC Process_8F41_Cmds(IN std::vector<SmlkMcuCmd> &mcu_cmds, OUT RctrlResultQueue &result);                            /*控车组合指令处理*/
        SMLK_RC Process_8F51(IN std::vector<SMLK_UINT16> &cmd_vec, OUT std::vector<VehctrlGetCmd> &result_vec);                   /*查询消息处理*/
        SMLK_RC ProcessRCtrlResult(IN RctrlResultQueue &result);                                                                  /*处理控车查询结果->分配给解析器组包回复消息*/
        /*通用方法*/
        void GetRctrlHeadFromIpcHead(IN IpcTspHead &ipc_head, OUT RctrlHead &rctrl_head);                                                             /*远控消息头数据提取*/
        SMLK_RC VdDataValidCheck(IN SMLK_UINT32 vehicle_index, IN SMLK_DOUBLE vehicle_data, IN bool vechile_valid, OUT VehctrlGetCmd &rctrl_get_cmd); /*车态消息有效性检测*/
        SMLK_RC SendCtrlCmdToMcu(IN SmlkMcuCmd &mcu_cmd);                                                                                             /*发送单指令数据给mcu--可以合并*/
        SMLK_RC SendCtrlCmdsToMcu(IN std::vector<SmlkMcuCmd> &mcu_cmd);                                                                               /*发送多指令数据给mcu*/

        /***  业务需求  ***/
        /*消贷参数设置*/
        void Process_8F41_FinancialLockFunc(IN void *data, IN int len, OUT RctrlMsgQueue msg_queue); /*8F41金融锁车开关*/
        void Process_8103_4GAnt(IN void *data, IN int, OUT RctrlMsgQueue msg_queue);                 /*8103 4G天线开路检测*/

        /*唤醒上报*/
        SMLK_RC WakeupSendVehicleStatus(); /*唤醒上报车身状态*/
        SMLK_RC WakeupSendIdlingWarmMsg(); /*唤醒上报怠速暖机消息--可以合并*/

        /*初始化主动上报*/
        void InitUpdatePsCtrlMode(); /*上电后主动上报ps控制状态*/

        /*备案开关*/
        void OnRecvGBRecRes(std::string &json); /*国6备案开关处理结果上报*/

        /*仪表盘保养*/
        SMLK_RC OnTspConnectActiveReport();                           /*Tsp成功连接上主动上报功能实现*/
        void RegisterTspConnectActiveReport();                        /*注册tsp成功连接事件主动上报*/
        void SendDBMtnInfo(SMLK_UINT32 index);                        /*根据本地发送策略下发仪表盘保养参数配置*/
        void Process_8F42_0x19(IN SMLK_UINT8 *indata, IN size_t len); /*解析Tsp消息*/

    private:
        /***  进程逻辑参数  ***/
        /*消息处理线程*/
        std::thread m_msg_thread;            /*总消息队列线程*/
        std::thread m_cmd_thread;            /*控车查询消息队列线程*/
        std::thread m_awaken_thread;         /*短信唤醒线程*/
        std::thread m_offline_ctrl_thread;   /*离线控车线程*/
        std::thread m_can_bus_wakeup_thread; /*CAN网络唤醒上报线程*/
        /*消息队列*/
        Queue<RctrlMsgQueue> m_msg_type_from_queue; /*处理消息来源队列:消息来源tsp,短信,vehicledata,其他(前两者用于分配tsp消息到控车or查询队列,短信来源在消贷移除后没有意义 || 后两者用于状态变化主动上报)*/
        Queue<RctrlCmdQueue> m_control_query_queue; /*处理控车or查询消息的队列*/
        /*条件变量&锁*/
        std::mutex m_proc_mtx; /*进程管理锁*/
        std::condition_variable m_cmd_cv;
        std::mutex m_cmd_mtx; /*远控命令锁*/
        std::condition_variable m_awaken_cv;
        std::mutex m_awaken_mtx; /*短信唤醒命令锁*/
        /*进程状态*/
        SMLK_UINT32 m_flags; /*进程状态:Init,Start,Stop*/
        /*表*/
        std::map<SMLK_UINT32, std::shared_ptr<IDissector>> m_map_prot_dissect;
        std::map<SMLK_UINT16, ChangedStatusInfo> m_map_vd_change_status;
        std::map<SMLK_UINT16, ChangedStatusInfo> m_map_mcu_change_status;
        /*车身状态*/
        PowerMode i_power_state;      /*电源状态*/
        EngineMode i_engine_state;    /*发动机状态*/
        CanBusMode i_can_bus_state;   /*Can网络状态*/
        WakeupSource i_wakeup_source; /*唤醒源状态*/
        PsCtrlMode i_ps_state;        /*PS控制状态*/
        /***  业务逻辑参数  ***/
        /*Poco事件*/
        std::shared_ptr<Poco::Event> m_sp_awaken_evt;         /*短信唤醒事件*/
        std::shared_ptr<Poco::Event> m_sp_offlinectrl_evt;    /*离线控车事件*/
        std::shared_ptr<Poco::Event> m_sp_can_bus_wakeup_evt; /*CAN网络唤醒事件*/
        /*异步函数*/
        std::future<void> fur_wakeup_send_idle;    /*怠速暖机异步执行函数*/
        std::future<void> fur_on_tspconn_register; /*tsp成功连接异步注册函数--为了避免产地配置信息获取延迟阻塞初始化函数*/
        /*控车*/
        SMLK_UINT16 m_vehctrl_mcu_seq;                  /*车控模块发送给MCU的流水号*/
        SmlkRctrl_8F41_Result m_mcu_resp;               /*单条控车指令MCU响应*/
        std::vector<SmlkRctrl_8F41_Result> m_mcu_resps; /*组合指令MCU响应*/
        SMLK_UINT8 m_prop_ems = 0;
        SMLK_UINT8 m_prop_vist = 0;
        /*原子常量*/
        std::atomic<bool> m_active_report_flag;  /*是否已can唤醒主动上报*/
        std::atomic<bool> m_is_awaken;           /*是否短信唤醒状态 1:短信唤醒 0:非短信唤醒*/
        std::atomic<bool> m_is_recv_tsp_cmd;     /*短信唤醒是否收到远控指令 1:收到控车指令 0:未收到控车指令*/
        std::atomic<bool> m_is_recv_wakeup_text; /*电源管理模块是否通知短信唤醒源 1:通知 0:未通知*/
        /*仪表盘保养信息*/
        std::mutex m_db_mtn_mutex;                                                                      /*仪表盘保养信息数据锁*/
        std::shared_ptr<ITimer> m_timer_db;                                                             /*仪表盘逻辑定时器*/
        std::vector<SMLK_UINT32> m_db_mileage;                                                          /*保养里程*/
        std::vector<SMLK_UINT8> m_db_mtn_require;                                                       /*保养要求(保养里程是否有效)*/
        DashBoardMaintainSendStrategy m_db_send_type = DashBoardMaintainSendStrategy::DB_MTN_SNED_NONE; /*保养里程下发类型  0:下发默认值全F 1:按照平台下发的数值下发*/
        std::atomic<bool> m_timer_db_stop;                                                              /*仪表盘定时器关闭状态  true:关闭*/
    };

};
#endif

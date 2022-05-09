/*****************************************************************************/
/**
* \file       logger_client.h
* \author     wukai
* \date       2021/06/29
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
#ifndef SMLK_LOGGER_CLIENT_H_
#define SMLK_LOGGER_CLIENT_H_
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/

#include <iostream>
#include <memory>
#include <mutex>
#include <vector>
#include <map>

#include <Poco/Runnable.h>
#include "ftp_pusher.h"
#include "smlk_error.h"
#include "tsp_service_api.h"
#include "smlk_queue.h"
#include "logger_event_collect.h"
#include "logger_monitor.h"

using namespace Poco;


namespace smartlink {
class TspCmdQueue {
public:
    TspCmdQueue() {
        bzero(&m_f52_data, sizeof(F52Data));
        bzero(&m_f53_general, sizeof(F53General));
        bzero(&m_f53_cmd_0, sizeof(F53CMD_0));
    };
    virtual ~TspCmdQueue() {
    };

public:
    LoggerHead m_head;/*将tsp发过来的icp头中的信息取出来*/
    F52Data m_f52_data;/*tsp发过来的消息体*/
    F53General m_f53_general;
    FtpInfo m_ftp_info;
    F53CMD_0 m_f53_cmd_0;
    std::vector<std::string> m_f53_1_files;
};
class LoggerClient : public Runnable {
public:
    explicit LoggerClient();
    ~LoggerClient();

    void run() override;

private:
    SMLK_RC Init();
    void OnTspIndication(IN IpcTspHead &ipc_head, IN void* data, std::size_t len);
    void DealF52Action(IN TspCmdQueue& msg);
    void DealF53Action(IN TspCmdQueue& msg);
    void SendCommonResp(IN TspCmdQueue& msg, CommonRC result);
    void SendActionResp(IN TspCmdQueue& msg, IN SMLK_UINT8* buf, std::size_t len);
    bool GetLogFiles(SMLK_UINT32 time_start, SMLK_UINT32 time_end,
                        std::vector<string>& files,SMLK_UINT8 log_type = 0);
    std::string GetActiveCompressLogName(SMLK_UINT8 log_type, SMLK_BOOL need_update = false);

    std::string GetActiveLogName(SMLK_UINT8 log_type);
    //日志文件压缩
    bool SystemCompressLogFiles(SMLK_UINT8 log_type);

    string GetRightHirainLog(IN std::string &reqLog);

private:
    std::shared_ptr<FtpPushWithNonBlock> m_sp_ftp;
    Queue<TspCmdQueue> m_msg_queue;
    std::mutex m_queue_mutex;
    std::shared_ptr<logger::EventCollector> m_sp_event_collector;
    std::shared_ptr<logger::LoggerMonitor> m_sp_log_monitor;
    std::map<LogType, std::string> m_active_log_name_map;
};
}

#endif // SMLK_LOGGER_CLIENT_H_
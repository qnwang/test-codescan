/*****************************************************************************/
/**
* \file       logger_client.cpp
* \author     wukai
* \date       2021/06/29
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#include "logger_client.h"
#include "ipc_api_def.h"
#include <smlk_tools.h>
#include "decoder_8f53_2.h"
#include <smartlink_sdk_sys_power.h>

#include "Poco/Thread.h"
#include "Poco/BinaryReader.h"
#include "Poco/File.h"
#include "Poco/Path.h"
#include "Poco/RecursiveDirectoryIterator.h"

#include <vector>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <fstream>
#include <iomanip>

using namespace smartlink;
using namespace smartlink::tools;
using namespace smartlink::logger;
using namespace Poco;

const static map<LogType, string> g_map_log_path = {
    {LogType::ELogTspMessage,       "/userdata/record/tsplog/zip/"              },
    {LogType::ELogHirainMessage,    "/userdata/record/hirain/hirain_extdata/zip/"   },
    {LogType::ELogOtaMessage,       "/userdata/record/otalog/rotation/"             }
};

//pair: first: cd path   second: Relative path
const static map<LogType, pair<string,string>> g_map_compare_info = {
    {LogType::ELogTspMessage,       {"/userdata/record/tsplog/",                     "./zip/"}},
    {LogType::ELogHirainMessage,    {"/userdata/record/hirain/hirain_extdata/",      "./zip/"}},
    {LogType::ELogOtaMessage,       {"/userdata/record/otalog/",                "./rotation/"}}
};

inline void SMLK_UINT8toStr(char* str, const unsigned char* UnChar, int ucLen)
{
	int i = 0;
	for(i = 0; i < ucLen; i++)
	{
		sprintf(str + i * 2, "%02x", UnChar[i]);
	}
}

#include <csignal>
#include <execinfo.h>

namespace {
    volatile sig_atomic_t signal_status = 0;
}

LoggerClient::LoggerClient()
{

}

LoggerClient::~LoggerClient() {

}

void LoggerClient::run() {
    SMLK_LOGD("LoggerClient thread run ...");
    SMLK_RC rc = Init();
    if (SMLK_RC::RC_OK != rc)
    {
        SMLK_LOGE("init err, direct return..");
        return;
    }

    while ((signal_status != SIGTERM) && (signal_status != SIGINT))
    {
        TspCmdQueue queue_msg = m_msg_queue.get();
        switch (queue_msg.m_head.msg_id)
        {
        case JTT808_LOG_STATUS:
            SMLK_LOGI("[logger]******** JTT808_LOG_STATUS  0X8F52 message get queue ***********************");
            DealF52Action(queue_msg);
            break;
        case JTT808_LOG_UP_LOAD:
            SMLK_LOGI("[logger]******** JTT808_LOG_UP_LOAD  0X8F53 message get queue ***********************");
            DealF53Action(queue_msg);
            break;
        default:
            break;
        }
    }

    if (m_sp_ftp.get())
    {
        m_sp_ftp->close();
        m_sp_ftp.reset();
    }

    if (m_sp_event_collector.get())
    {
        m_sp_event_collector->Stop();
        m_sp_event_collector.reset();
    }
}

SMLK_RC LoggerClient::Init() {
    mkdir_p("/userdata/vendor/hirain/hirain_extdata/smlkzip/");
    mkdir_p("/userdata/vendor/hirain/hirain_extdata/zip/");
    /*注册tsp消息接收接口*/

    SMLK_RC tsp_rc =TspServiceApi::getInstance()->Init(ModuleID::E_MOUDLE_LOGGER_APP);
    if( SMLK_RC::RC_OK != tsp_rc ){
        SMLK_LOGW("TspServiceApi  Init failed. retry ...\n");
        return SMLK_RC::RC_ERROR;
    }

    if ( !smartlink_sdk::SysPower::GetInstance()->Init(ModuleID::E_MOUDLE_LOGGER_APP) ) {
        SMLK_LOGE("fail to init power service API interface!!!");
        return SMLK_RC::RC_ERROR;
    }

    vector<SMLK_UINT16> msg_id;
    msg_id.push_back(JTT808_LOG_STATUS);/*日志开关设置/日志等级查询*/
    msg_id.push_back(JTT808_LOG_UP_LOAD);/*历史日志上传请求*/

    TspServiceApi::getInstance()->RegisterMsgCB(ProtoclID::E_PROT_JTT808, msg_id,
                                                        std::bind(&LoggerClient::OnTspIndication, this,
                                                        std::placeholders::_1,
                                                        std::placeholders::_2,
                                                        std::placeholders::_3));
    m_sp_event_collector = std::make_shared<logger::EventCollector>();
    m_sp_event_collector->Init();

    //日志逻辑调整， 暂时注释日志监控相关功能。
    m_sp_log_monitor = std::make_shared<logger::LoggerMonitor>();
    m_sp_log_monitor->ThreadRun();

    return SMLK_RC::RC_OK;
}

void LoggerClient::OnTspIndication(IN IpcTspHead &ipc_head, IN void* data, std::size_t len)
{
    SMLK_LOGD("*******enter in func(%s), message from tsp :len=%ld.*******\n",__func__,len);
    TspCmdQueue tsp_msg;
    if ((JTT808_LOG_STATUS == ipc_head.msg_id)
        || (JTT808_LOG_UP_LOAD == ipc_head.msg_id)) {
#if 1
        SMLK_LOGD("LoggerClient message body is :\n");
        for(std::size_t i = 0; i < len; ++i){
            printf("%02x ", *((SMLK_UINT8 *)data + i));
        }
        printf("\n");
#endif
        tsp_msg.m_head.msg_id = ipc_head.msg_id;
        tsp_msg.m_head.seq_id = ipc_head.seq_id;
        tsp_msg.m_head.protocol = ipc_head.protocol;
        tsp_msg.m_head.qos = ipc_head.qos;
        tsp_msg.m_head.priority = ipc_head.priority;
        tsp_msg.m_head.reserved_dw = ipc_head.reserved_dw;
    }

    if(JTT808_LOG_STATUS == ipc_head.msg_id) {
        // TODO send msg 2 loop
        tsp_msg.m_f52_data = *(F52Data*) data;

        SMLK_LOGI("[logger]*******enter in func(%s), message cmd_type=%02x cmd_content=%02x.*******\n",__func__,
                                                                            tsp_msg.m_f52_data.cmd_type,
                                                                            tsp_msg.m_f52_data.cmd_content);
        SendCommonResp(tsp_msg, CommonRC::CommonRC_OK);
        m_msg_queue.put(tsp_msg);
        return;
    }

    if (JTT808_LOG_UP_LOAD == ipc_head.msg_id) {
        memcpy(&tsp_msg.m_f53_general, data, sizeof(F53General));
        SMLK_LOGI("[logger]*******enter in func(%s), message f53_general.general_type=%02x.*******\n",__func__,
                                                                                tsp_msg.m_f53_general.general_type);
        // 0:请求历史日志列表 1:请求发送历史日志  // 2:请求所有历史日志
        switch (tsp_msg.m_f53_general.general_type)
        {
        case 0x00:
        {
            SMLK_LOGD("********** enter request history log list************");
            if (len - sizeof(F53General) != sizeof(F53CMD_0))
            {
                SendCommonResp(tsp_msg, CommonRC::CommonRC_UNSUPPORT);
                SMLK_LOGD("********** enter request history log list server data len err************");
                return;
            }
            memcpy(&tsp_msg.m_f53_cmd_0, (SMLK_UINT8*)data + sizeof(F53General), len - sizeof(F53General));
            if (tsp_msg.m_f53_general.log_type != 0 && tsp_msg.m_f53_general.log_type != 1
                 && tsp_msg.m_f53_general.log_type != 2)
            {
                SMLK_LOGW("no define log type: [%d]",tsp_msg.m_f53_general.log_type);
                SendCommonResp(tsp_msg, CommonRC::CommonRC_UNSUPPORT);
                return;
            }
        }
            break;
        case 0x01:
        {
            Decoder8F53_2 x8f53_2_decoder;
            x8f53_2_decoder.Decode((SMLK_UINT8*)data + sizeof(F53General), len - sizeof(F53General));
            if (tsp_msg.m_f53_general.log_type != 0 && tsp_msg.m_f53_general.log_type != 1
                && tsp_msg.m_f53_general.log_type != 2)
            {
                SendCommonResp(tsp_msg, CommonRC::CommonRC_UNSUPPORT);
                return;
            }
            for (auto it_ : x8f53_2_decoder.file_lists)
            {
                if (it_.empty())
                {
                    continue;
                }
                string local_path_str;
                local_path_str = g_map_log_path.find((LogType)tsp_msg.m_f53_general.log_type)->second + it_ ;
                // if (it_.find("Active") == std::string::npos && access(local_path_str.c_str(), F_OK))
                // {
                //     SMLK_LOGD("not exisit --> %s", local_path_str.c_str());
                //     continue;
                // }
                SMLK_LOGD("server ark local_path_str = %s", local_path_str.c_str());
                tsp_msg.m_f53_1_files.push_back(local_path_str);
            }
            if (tsp_msg.m_f53_1_files.empty())
            {
                SMLK_LOGD("upload history list is empty ...");
                SendCommonResp(tsp_msg, CommonRC::CommonRC_FAILED);
                return;
            }
            tsp_msg.m_ftp_info.host = x8f53_2_decoder.host_;
            tsp_msg.m_ftp_info.passwd = x8f53_2_decoder.passwd_;
            tsp_msg.m_ftp_info.user = x8f53_2_decoder.usrname_;
            tsp_msg.m_ftp_info.port = x8f53_2_decoder.port_;
            tsp_msg.m_ftp_info.server_path = x8f53_2_decoder.server_path_;
            SendCommonResp(tsp_msg, CommonRC::CommonRC_OK);
        }
            break;
        case 0x02:
        {
            Decoder8F53_2 x8f53_2_decoder;
            x8f53_2_decoder.Decode((SMLK_UINT8*)data + sizeof(F53General), len - sizeof(F53General));
            tsp_msg.m_ftp_info.host = x8f53_2_decoder.host_;
            tsp_msg.m_ftp_info.passwd = x8f53_2_decoder.passwd_;
            tsp_msg.m_ftp_info.user = x8f53_2_decoder.usrname_;
            tsp_msg.m_ftp_info.port = x8f53_2_decoder.port_;
            tsp_msg.m_ftp_info.server_path = x8f53_2_decoder.server_path_;
        }
            break;
        default:
            SendCommonResp(tsp_msg, CommonRC::CommonRC_UNSUPPORT);
            return;
        }
        m_msg_queue.put(tsp_msg);
    }

}

void LoggerClient::DealF52Action(IN TspCmdQueue& msg)
{
    struct XF52Resp {
        SMLK_INT16 seq_id;
        SMLK_UINT8 cmd_type;
        SMLK_UINT8 cmd_res;
    } xf52_resp;
    /*  若设置类型字段为0-2，回复：
        0：失败；1：成功
        若设置类型字段为3，回复：
        0: err；1: warning ；2: info ；3: debug
    */
    xf52_resp.seq_id = htobe16(msg.m_head.seq_id);
    xf52_resp.cmd_type = msg.m_f52_data.cmd_type;
    xf52_resp.cmd_res = 1;
    SMLK_LOGI("[logger]*********DealF52Action ",xf52_resp.cmd_type);

    if (m_sp_event_collector.get())
    {
        if ((F52CmdType)xf52_resp.cmd_type == F52CmdType::CMD_OFFLINE_LOG_LEVEL)
        {
            m_sp_event_collector->SetOfflineLogLevel((LogLevel)msg.m_f52_data.cmd_content);
        }
        else if ((F52CmdType)xf52_resp.cmd_type == F52CmdType::CMD_EVENT_LOG_SWITCH)
        {
            m_sp_event_collector->SwitchEvent((SMLK_BOOL)msg.m_f52_data.cmd_content);
        }
        //TBD 实时日志待定。
        else if ((F52CmdType)xf52_resp.cmd_type == F52CmdType::CMD_ACTIVE_LOG_SWITCH)
        {
            // m_sp_event_collector->SwitchActiveLog((SMLK_BOOL)msg.m_f52_data.cmd_content);
        }
        else if ((F52CmdType)xf52_resp.cmd_type == F52CmdType::CMD_QUERY_OFFLINE_LOG_LEVEL)
        {
            xf52_resp.cmd_res = 3;
            LogLevel log_lvl_ = m_sp_event_collector->GetOfflineLogLevel();
            xf52_resp.cmd_res = (SMLK_UINT8)log_lvl_;
        }
        else if((F52CmdType)xf52_resp.cmd_type == F52CmdType::CMD_WARN_VALUE)
        {
            //todo 告警阈值设置
            m_sp_event_collector->SetWarnValue(msg.m_f52_data.cmd_content);
        }
    }
    else
    {
        xf52_resp.cmd_res = 0;
        if((F52CmdType)xf52_resp.cmd_type == F52CmdType::CMD_QUERY_OFFLINE_LOG_LEVEL)
        {
            xf52_resp.cmd_res = 3;
        }
    }

    SendActionResp(msg, (SMLK_UINT8*)&xf52_resp, sizeof(xf52_resp));
}

void LoggerClient::DealF53Action(IN TspCmdQueue& msg)
{
    SMLK_LOGI("[logger]DealF53Action  get f53_general.general_type = %d", msg.m_f53_general.general_type);
    switch (msg.m_f53_general.general_type)
    {
    case 0x00:
    {
        SMLK_UINT8 bcd_start_time[6];
        memcpy(bcd_start_time, msg.m_f53_cmd_0.start_time, 6);
        SMLK_UINT8 bcd_end_time[6];
        memcpy(bcd_end_time, msg.m_f53_cmd_0.end_time, 6);
        string tsp_log;
        tsp_log.clear();
        for(SMLK_UINT16 i = 0; i < 6; ++i){
            char hex[4]={0};
            sprintf(hex, "%02x", bcd_start_time[i]);
            tsp_log.insert(tsp_log.end(), hex, hex+2);
        }
        SMLK_LOGD("DealF53Action start_time = [%s]....", tsp_log.c_str());
        tsp_log.clear();
        for(SMLK_UINT16 i = 0; i < 6; ++i){
            char hex[4]={0};
            sprintf(hex, "%02x", bcd_end_time[i]);
            tsp_log.insert(tsp_log.end(), hex, hex+2);
        }
        SMLK_LOGD("DealF53Action end_time = [%s]....", tsp_log.c_str());
        SMLK_UINT32 time_start_ = bcd_timestamp_2_id(bcd_start_time); // ut 2 cst8
        SMLK_UINT32 time_end_ = bcd_timestamp_2_id(bcd_end_time); // ut 2 cst8
        std::vector<string> files_ ;
        SMLK_LOGD("DealF53Action  get log path = [%s]", msg.m_f53_cmd_0.log_path);
        string path_str(msg.m_f53_cmd_0.log_path, sizeof(msg.m_f53_cmd_0.log_path));
        path_str = g_map_log_path.find((LogType)msg.m_f53_general.log_type)->second;
        SMLK_LOGD("DealF53Action  get log path_str = [%s]", path_str.c_str());

        bool ret = GetLogFiles(time_start_, time_end_, files_, msg.m_f53_general.log_type);
        if (!ret || (files_.size() == 0))
        {
            SMLK_LOGD("scan files is empty for start time 2 end time ...");
            // SendCommonResp(msg, CommonRC::CommonRC_OK);
            // return;
        }
        SendCommonResp(msg, CommonRC::CommonRC_OK);
        XF53CMD_0_Resp xf53_cmd_0_resp;
        get_bcd_timestamp(xf53_cmd_0_resp.general_head.time);
        xf53_cmd_0_resp.general_head.seq_id = htobe16(msg.m_head.seq_id);
        xf53_cmd_0_resp.general_head.action_id = 0x00; // 上传历史日志list resp
        xf53_cmd_0_resp.log_type = msg.m_f53_general.log_type;
        xf53_cmd_0_resp.log_count = (SMLK_UINT8)files_.size();
        bzero(xf53_cmd_0_resp.log_local_path, LOG_SERVER_SVAE_PATH_LENGTH);
        strcat(xf53_cmd_0_resp.log_local_path, path_str.c_str());
        std::vector<SMLK_UINT8> upload_buf;
        upload_buf.insert(upload_buf.end(), (SMLK_UINT8*)&xf53_cmd_0_resp, (SMLK_UINT8*)&xf53_cmd_0_resp + sizeof(XF53CMD_0_Resp));
        int count = 0;
        for (auto path_ : files_)
        {
            SMLK_LOGD("DealF53Action file path = %s", path_.c_str());
            LogFileAttribute  file_attr;
            file_attr.log_id = count++;
            string file_name = path_.substr(path_.rfind("/") + 1);
            bzero(file_attr.log_name, LOG_SERVER_FILE_NAME_LENGTH);
            // file_name = ASCII2BCD(file_name);
            memcpy(file_attr.log_name, file_name.c_str(), file_name.size());
            fstream ifs(path_, std::ios::in);
            SMLK_UINT64 len = 0;
            if (ifs.is_open())
            {
                ifs.seekg(0, std::ios::end);
                len = ifs.tellg();
                ifs.close();
            }
            SMLK_LOGD("DealF53Action get file is open size = %lld", len);
            file_attr.log_size = htobe16((SMLK_UINT16)(len / 1024));
            upload_buf.insert(upload_buf.end(), (SMLK_UINT8*)&file_attr, (SMLK_UINT8*)&file_attr + sizeof(LogFileAttribute));
            SMLK_LOGD("DealF53Action file id = [%d] size = [%d] name = [%s]", file_attr.log_id, file_attr.log_size, file_attr.log_name);
        }
        // SMLK_LOGD("smlk_logger ready 2 send history list 2 server ...");
        SendActionResp(msg, upload_buf.data(), upload_buf.size());
        SMLK_LOGD("smlk_logger ready 2 send history list 2 server success ...");
    }
        break;
    case 0x01:
    {
        smartlink_sdk::SysPower::GetInstance()->LockWake(std::string("SMLK_LOGGER"));
        std::vector<std::string> files_vec;
        std::vector<std::string> files_vec_ftp;
        for (auto file_path_ : msg.m_f53_1_files)
        {
            if ((file_path_.rfind("Active") != std::string::npos))
            {
                std::string log_local_path = GetActiveCompressLogName(msg.m_f53_general.log_type);
                bool ret = SystemCompressLogFiles(msg.m_f53_general.log_type);
                if (!ret) {
                    SMLK_LOGD("*******enter in func(%s), tar log failed *******\n", __func__);
                }
                files_vec.push_back(log_local_path);
                files_vec_ftp.push_back(log_local_path);
                continue;
            }

            //hirain日志触发压缩时，会生成最新的日志文件， 原有日志压缩文件名称编号会变化。
            //导致可能出现文件名称不存在情况， 增加时间戳模糊匹配来处理此类情况。,同时上传至FTP时，将名称改回。
            if (msg.m_f53_general.log_type == 1)
            {
                string strT = GetRightHirainLog(file_path_);
                files_vec.push_back(strT);
            }
            else
            {
                files_vec.push_back(file_path_);
            }
            files_vec_ftp.push_back(file_path_);
        }
        try
        {
            m_sp_ftp.reset(new FtpPushWithNonBlock(msg.m_ftp_info));
            m_sp_ftp->upLoadFile(files_vec, files_vec_ftp, 300000, [this, &msg](FtpErrorCode code){
                // TODO response server ftp action status
                SMLK_LOGD("*******enter in func(FtpPusherListener), get FTPResult code = %d *******\n", code);
                XF53Head m_f53resp_head;
                get_bcd_timestamp(m_f53resp_head.time);
                m_f53resp_head.seq_id = htobe16(msg.m_head.seq_id);
                m_f53resp_head.action_id = (SMLK_UINT8) F53ReplyActionCode::F53ReplyActionCode_UpLoad_Result;

                SMLK_UINT8 event_buf[2];
                event_buf[0] = msg.m_f53_general.log_type;
                event_buf[1] = (SMLK_UINT8) code;

                size_t all_len = sizeof(m_f53resp_head) + 2;
                SMLK_UINT8 rsp_buf[all_len];
                memcpy(rsp_buf, (SMLK_UINT8*)&m_f53resp_head, all_len - 2);
                memcpy(rsp_buf + sizeof(m_f53resp_head), event_buf, 2);
                SendActionResp(msg, rsp_buf, all_len);
                smartlink_sdk::SysPower::GetInstance()->UnLockWake(std::string("SMLK_LOGGER"));
                SMLK_LOGD("upLoadFile end ...");
            })->start();
        }
        catch(const std::exception& e)
        {
            SMLK_LOGE("0x8f53 deal 0x01 upload file err -> %s", e.what());
            smartlink_sdk::SysPower::GetInstance()->UnLockWake(std::string("SMLK_LOGGER"));
        }
    }
        break;
    case 0x02: // upload all history logs
    {
        SMLK_UINT8 log_type = msg.m_f53_general.log_type;
        if (log_type != 0 && log_type != 1) {
            SMLK_LOGW("LoggerClient::DealF53Action upload unknown log type = %d", log_type);
            SendCommonResp(msg, CommonRC::CommonRC_UNSUPPORT);
            return;
        }

        std::vector<string> files_Ftp ;
        bool ret = GetLogFiles(0, 0, files_Ftp);
        if (!ret || (files_Ftp.size() == 0))
        {
            SMLK_LOGD("scan files is empty for start time 2 end time ...");
            SendCommonResp(msg, CommonRC::CommonRC_FAILED);
            return;
        }
        smartlink_sdk::SysPower::GetInstance()->LockWake(std::string("SMLK_LOGGER"));
        SendCommonResp(msg, CommonRC::CommonRC_OK);
        m_sp_ftp.reset(new FtpPushWithNonBlock(msg.m_ftp_info));
        try
        {
            m_sp_ftp->upLoadFile(files_Ftp, files_Ftp, 3000, [this, &msg](FtpErrorCode code){
                // TODO response server ftp action status
                SMLK_LOGD("*******enter in func(FtpPusherListener), get FTPResult code = %d *******\n", code);
                XF53Head m_f53resp_head;
                get_bcd_timestamp(m_f53resp_head.time);
                m_f53resp_head.seq_id = htobe16(msg.m_head.seq_id);
                m_f53resp_head.action_id = (SMLK_UINT8) F53ReplyActionCode::F53ReplyActionCode_UpLoad_Result;

                SMLK_UINT8 event_buf[2];
                event_buf[0] = msg.m_f53_general.log_type;
                event_buf[1] = (SMLK_UINT8) code;

                size_t all_len = sizeof(m_f53resp_head) + 2;
                SMLK_UINT8 rsp_buf[all_len];
                memcpy(rsp_buf, (SMLK_UINT8*)&m_f53resp_head, all_len - 2);
                memcpy(rsp_buf + sizeof(m_f53resp_head), event_buf, 2);
                SendActionResp(msg, rsp_buf, all_len);
                smartlink_sdk::SysPower::GetInstance()->UnLockWake(std::string("SMLK_LOGGER"));
            })->start();
        }
        catch(const std::exception& e)
        {
            SMLK_LOGE("0x8f53 deal 0x02 upload file err -> %s", e.what());
            smartlink_sdk::SysPower::GetInstance()->UnLockWake(std::string("SMLK_LOGGER"));
        }
    }
        break;
    default:
    {
        SendCommonResp(msg, CommonRC::CommonRC_UNSUPPORT);
    }
        break;
    }
}

std::string LoggerClient::GetActiveLogName(SMLK_UINT8 log_type)
{
    std::string active_log_name_str;
    switch ((LogType)log_type)
    {
    case LogType::ELogTspMessage:
        active_log_name_str = "TSPLog_tail_me.log";
        break;
    case LogType::ELogHirainMessage:
        active_log_name_str = "messages";
        break;
    case LogType::ELogOtaMessage:
        active_log_name_str = "*.log";
        break;
    default:
        break;
    }

    return active_log_name_str;
}

/******************************************************************************
* NAME: SystemCompressLogFiles
*
* DESCRIPTION: 执行压缩命令
*
* PARAMETERS:
*   log_type：日志类型
*
* RETURN:
*   bool: 命令执行结果
* NOTE:
*****************************************************************************/
bool LoggerClient::SystemCompressLogFiles(SMLK_UINT8 log_type)
{
    SMLK_LOGI("[logger] run system compress...");
    std::string ctr = "chmod -R 777 " + g_map_log_path.find((LogType)log_type)->second;
    system(ctr.c_str());
    std::string targetPath = GetActiveCompressLogName(log_type);
    std::string sourcePath = GetActiveLogName(log_type);
    std::string cdPath = g_map_compare_info.find((LogType)log_type)->second.first;
    std::string relativePath = g_map_compare_info.find((LogType)log_type)->second.second;

    std::string targetFileName = targetPath.substr(targetPath.rfind("/") + 1);
    relativePath.append(targetFileName);
    std::string cmd = "cd " + cdPath + " && tar -zcvf "+ relativePath + "  " + sourcePath;
    SMLK_LOGI("[logger] run cmd start : %s",cmd.c_str());
    int nRet = system(cmd.c_str());
    SMLK_LOGI("[logger] run cmd end");

    return nRet == 0 ? true : false;
}

/******************************************************************************
* NAME: GetRightHirainLog
*
* DESCRIPTION:
*   获取正确的请求上传的hirain日志。
*
* PARAMETERS:
*   vecReqLog: 平台请求日志列表
*   vecRightLog: 正确的日志列表
*
* RETURN:
*
* NOTE:
*****************************************************************************/
string LoggerClient::GetRightHirainLog(IN std::string &reqLog)
{
    if (reqLog.length() < 17)
    {
        return reqLog;
    }

    string zipPath = g_map_log_path.find(LogType::ELogHirainMessage)->second;
    DIR *dir;
    struct dirent *ptr;

    if ((dir = opendir(zipPath.c_str())) == NULL)
    {
        SMLK_LOGD("****************** GetLogFiles no such file or directory **********");
        return reqLog;
    }

    string logTime = reqLog.substr(reqLog.length()- 14-3, 14);
    while ((ptr = readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
                continue;
        else if(ptr->d_type == 8)
        {
            string strFile = string(ptr->d_name);
            if (strFile.rfind(logTime) != string::npos)
            {
                strFile = zipPath + strFile;
                return strFile;
            }
        }
    }
    closedir(dir);

    return reqLog;
}

std::string LoggerClient::GetActiveCompressLogName(SMLK_UINT8 log_type, SMLK_BOOL need_update)
{
    if (!need_update)
    {
        auto pair = m_active_log_name_map.find((LogType)log_type);
        if (pair != m_active_log_name_map.end())
        {
            return pair->second;
        }
    }

    string log_tag;
    string log_compare_type;
    string log_local_path;

    char time_buf[32] = {0};
    time_t now ;
    struct tm *tm_now ;
    time(&now) ;
    tm_now = localtime(&now) ;//get date
    strftime(time_buf, 32, "%Y%m%d%H%M%S", tm_now);

    log_local_path.append(g_map_log_path.find((LogType)log_type)->second);
    switch ((LogType)log_type)
    {
    case LogType::ELogTspMessage:
        log_tag = "ActiveTSP-";
        log_compare_type = ".tar.gz";
        break;
    case LogType::ELogHirainMessage:
        log_tag = "ActiveHirainMessage-";
        log_compare_type = ".gz";
        break;
    case LogType::ELogOtaMessage:
        log_tag = "ActiveOtaMessage-";
        log_compare_type = ".tar.gz";       //tbox中没有zip压缩命令， 暂用tar.gz格式压缩
        break;
    default:
        break;
    }
    log_local_path.append(log_tag);
    log_local_path.append(time_buf);
    log_local_path.append(log_compare_type);

    auto pair_ = m_active_log_name_map.find((LogType)log_type);
    if (pair_ != m_active_log_name_map.end())
    {
        pair_->second = log_local_path;
    }
    else
    {
        m_active_log_name_map.insert(std::pair<LogType,std::string>((LogType)log_type, log_local_path));
    }
    return log_local_path;
}

/******************************************************************************
* NAME: GetLogFiles
*
* DESCRIPTION: 获取当前时间段内对应的日志文件列表
*
* PARAMETERS:
*    time_start: 开始时间
*    time_end: 结束时间
*    files: 查询出的日志文件列表
*    log_type: 日志类型
*
* RETURN:
*    bool： 查询是否成功
*
* NOTE:
*****************************************************************************/
bool LoggerClient::GetLogFiles(SMLK_UINT32 time_start, SMLK_UINT32 time_end,
                                            std::vector<string>& files, SMLK_UINT8 log_type)
{
    // 日志压缩文件命名格式必须是(包括运行日志)  XXXX__20210708112533.tar.gz
    SMLK_LOGD("******************start time ****** %ld**********", time_start);
    SMLK_LOGD("******************end time ****** %ld**********", time_end);

    string path = g_map_log_path.find((LogType)log_type)->second;
    std::string split_str;
    switch ((LogType)log_type)
    {
    case LogType::ELogTspMessage:
        split_str = ".tar.gz";
        break;
    case LogType::ELogHirainMessage:
        split_str = ".gz";
        break;
    case LogType::ELogOtaMessage:
        split_str = ".zip";
        break;
    default:
        break;
    }

    DIR *dir;
    struct dirent *ptr;

    if ((dir = opendir(path.c_str())) == NULL)
    {
        SMLK_LOGD("****************** GetLogFiles no such file or directory **********");
        return false;
    }

    string absolute_path;
    if (path.rfind("/") != path.size() - 1)
    {
        absolute_path = absolute_path.assign(path).append("/");
    }
    else
    {
        absolute_path = path;
    }

    SMLK_BOOL need_push_active_file_bool = false;
    time_t now ;
    time(&now);
    SMLK_LOGD("need_push_active_file_bool now = %ld", now);
    //ota日志默认上传当前日志， 其他类型日志最后时间小于0，或与当前时间差小于4min时，上传当前日志
    if ((LogType::ELogOtaMessage == (LogType)log_type)
        || ((int)time_end <= 0) || (labs(now - time_end) < 240/*4min*/))
    {
        need_push_active_file_bool = true;
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
                continue;
        else if(ptr->d_type == 8)
        {
            //ota日志默认所有文件全部上传
            if (LogType::ELogOtaMessage == (LogType)log_type)
            {
                 files.push_back(ptr->d_name);
                 continue;
            }

            //避免非法文件名称导致崩溃，substr()方法
            try
            {
                char name_char[64];
                bzero(name_char, 64);
                strcpy(name_char, ptr->d_name);
                string file_name_str = name_char;
                string time_temp = file_name_str.substr(file_name_str.rfind(split_str.c_str()) - 14, 14);
                SMLK_LOGD("substr filename = %s    time_temp = %s", file_name_str.c_str(), time_temp.c_str());
                tm tm_;
                strptime(time_temp.c_str(), "%Y%m%d%H%M%S", &tm_);
                SMLK_UINT32 time_id = mktime(&tm_); // ut 2 cst8
                SMLK_LOGD("GetLogFiles calc time seconds = %ld", time_id);
                std::string p = absolute_path + file_name_str;
                char buf[absolute_path.size() + file_name_str.size()];
                bzero(buf, absolute_path.size() + file_name_str.size());
                strcat(buf, absolute_path.c_str());
                strcat(buf, file_name_str.c_str());
                SMLK_LOGD("get file absolute path = %s", buf);
                int start_ = (int)time_start;
                int end_ = (int)time_end;

                //开始时间和结束时间小于等于0时其他日志文件全部上传。
                if (start_ <= 0 && end_ <= 0)
                {
                    files.push_back(buf);
                }
                else if (time_id >= time_start && time_id <= time_end)
                {
                    files.push_back(buf);
                }
            }
            catch(const std::exception& e)
            {
                SMLK_LOGW("invaild file name: %s",ptr->d_name);
            }
        }
    }
    closedir(dir);
    if (need_push_active_file_bool)
    {
        std::string active_log_name_str = GetActiveCompressLogName(log_type, true);
        files.push_back(active_log_name_str);
    }
    SMLK_LOGD("******************enter****** GetLogFiles success **********");
    return true;
}

void LoggerClient::SendActionResp(IN TspCmdQueue& msg, IN SMLK_UINT8* buf, std::size_t len)
{
    IpcTspHead ipc_head;
    ipc_head.msg_id = msg.m_head.msg_id == JTT808_LOG_UP_LOAD ? JTT808_LOG_UP_LOAD_ARK : JTT808_LOG_STATUS_ARK;
    ipc_head.seq_id = htobe16(msg.m_head.seq_id);
    ipc_head.protocol = (SMLK_UINT8)(msg.m_head.protocol);
    ipc_head.qos = msg.m_head.qos;
    ipc_head.priority = msg.m_head.priority;
    TspServiceApi::getInstance()->SendMsg(ipc_head, buf, len);
}

void LoggerClient::SendCommonResp(IN TspCmdQueue& msg, CommonRC result)
{
    IpcTspHead ipc_head;
    ipc_head.msg_id = JTT808_LOG_COMMON_ARK;
    ipc_head.seq_id = msg.m_head.seq_id;
    ipc_head.protocol = (SMLK_UINT8)(msg.m_head.protocol);
    ipc_head.qos = msg.m_head.qos;
    ipc_head.priority = msg.m_head.priority;
    CommResp response;
    response.seq_id = htobe16(msg.m_head.seq_id);
    response.msg_id = htobe16(msg.m_head.msg_id);
    response.result = (SMLK_UINT8)result;
    TspServiceApi::getInstance()->SendMsg(ipc_head, (SMLK_UINT8 *)&response, (std::size_t)sizeof(response));
}
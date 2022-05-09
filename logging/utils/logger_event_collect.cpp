/*****************************************************************************/
/**
* \file       logger_event_collect.cpp
* \author     wukai
* \date       2021/08/09
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   */
#include <fstream>
#include <unistd.h>
#include <map>
#include <chrono>
#include <string>

#include "logger_event_collect.h"
#include "smlk_log.h"
#include "fota_ipc_api.h"
#include "spdlog/spdlog.h"
#include "smartlink_sdk_tel.h"
#include "smartlink_sdk_sys_os.h"
#include "smartlink_sdk_sys_property.h"
#include "smartlink_sdk_sys_property_def.h"
#include "logger_jt808_def.h"
#include "tsp_service_api.h"
#include "smlk_tools.h"
#include <cstring>

#if !defined(WIN32) && !defined(__WATCOMC__) && !defined(__VXWORKS__)
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sysinfo.h>
#endif

#define     LOGGER_LOGGER_LEVEL_SET                     ("/userdata/smartlink/config/.log_level_new")
#define     LOGGER_EVENT_REPORT_SET                     ("/userdata/smartlink/config/.log_report")
#define     LOGGER_WARN_VALUE_SET                       ("/userdata/smartlink/config/.warn_value")


#define     LOGGER_LOGGER_LEVEL_PROPERTY                ("logger_log_level")    /*log_level*/
#define     LOGGER_EVENT_REPORT_PROPERTY                ("logger_log_report")   /*log report switch*/
#define     LOGGER_WARN_VALUE_PROPERTY                  ("logger_warn_value")   /*warn-value*/


#define SYS_PROPERTY_OPEN   1

using namespace smartlink::tools;

namespace smartlink {
namespace logger {
using namespace smartlink::Fota;

const static std::string LogFileName[] = {"/userdata/record/tsplog/unzip/",
                                "/userdata/record/hirain/hirain_extdata/messages"};

static const std::map<LogLevel, smartlink_sdk::SysLogLevel> g_smlklog_to_syslog = {
    { LogLevel::SMLK_LOG_LEVEL_Error,         smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_ERR },
    { LogLevel::SMLK_LOG_LEVEL_Warning,       smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_WARNING },
    { LogLevel::SMLK_LOG_LEVEL_Info,          smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_INFO },
    { LogLevel::SMLK_LOG_LEVEL_Debug,         smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_DEBUG }
};

static const std::map<smartlink_sdk::SysLogLevel, LogLevel> g_syslog_to_smlklog = {
    {smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_FATAL ,    LogLevel::SMLK_LOG_LEVEL_Error },
    {smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_ERR ,      LogLevel::SMLK_LOG_LEVEL_Error },
    {smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_WARNING ,  LogLevel::SMLK_LOG_LEVEL_Warning },
    {smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_INFO ,     LogLevel::SMLK_LOG_LEVEL_Info},
    {smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_DEBUG ,    LogLevel::SMLK_LOG_LEVEL_Debug},
};

EventCollector::EventCollector()
    : m_imsi_str("")
    , m_pre_line_short(20)
    , m_log_level(NULL)
{
    m_event_switch.store(false);
    m_bDiskOverIsReported.store(false);
    m_bIgOn.store(false);
    m_bBootEventIsReported.store(false);

    m_warnValue = 80;
}

EventCollector::~EventCollector()
{
}

void EventCollector::Init()
{
    //注册Tel回调
    if(!smartlink_sdk::Telephony::GetInstance()->Init(ModuleID::E_MOUDLE_LOGGER_APP)){
        SMLK_LOGF("EventCollector fail to init Telephony service API interface.\n");
        return;
    }
    else
    {
        smartlink_sdk::RtnCode rc = smartlink_sdk::Telephony::GetInstance()->GetIMSI(m_imsi_str);
    }

    //注冊 ig状态变化回调
    if(smartlink_sdk::RtnCode::E_SUCCESS != smartlink_sdk::MCU::GetInstance()->Init(ModuleID::E_MOUDLE_LOGGER_APP))
    {
        SMLK_LOGF("EventCollector fail to init MCU service API interface.\n");
        return;
    }
    std::vector<smartlink_sdk::McuEventId>  mcu_events = {
        smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE,
    };
    smartlink_sdk::MCU::GetInstance()->RegEventCB(mcu_events, std::bind(&EventCollector::OnMcuSerIgonIndication, this,
                                                std::placeholders::_1,
                                                std::placeholders::_2,
                                                std::placeholders::_3)
    );

    //注册属性变化回调。
    if(!smartlink_sdk::SysProperty::GetInstance()->Init(ModuleID::E_MOUDLE_LOGGER_APP))
    {
        SMLK_LOGF("EventCollector fail to init SysProperty.\n");
        return;
    }
    const std::vector<std::string> propertys_key = {
        std::string(SYS_PRO_NAME_FOTA_FAIL_REPORT)
    };
    smartlink_sdk::SysProperty::GetInstance()->RegEventCB(propertys_key, std::bind(&EventCollector::OnSysprotertyIndication, this,
                                                std::placeholders::_1,
                                                std::placeholders::_2)
    );

    //初始化读取日志上报设置
    if(GetLogReportSwitch())
    {
        SwitchEvent(true);
        SMLK_LOGD("[logger readProperty] read log_report  is ture");
    }

    //初始化读取告警阈值
    SMLK_UINT8 warnValue;
    if(GetWarnValue(warnValue))
    {
        m_warnValue = warnValue;
        SMLK_LOGD("[logger readProperty] read warn-value is %d",warnValue);
    }


    //初始化读取日志等级
    if ( (m_shmd_id = shmget((key_t)0x5005, sizeof(int) * 2, 0640|IPC_CREAT)) == -1)
    {
        SMLK_LOGW("shmat(0x5005) failed");
    }
    m_log_level = (int *)shmat(m_shmd_id, 0, 0);
    int level = 4;
    if (!GetOffLogLevel(level))
    {
        level = 4; //默认debug等级。
    }
    SMLK_LOGD("[logger readProperty]read log_level is %d \n",level);

    // int nSpdlogT = SyslogToSpdlog(level);
    // memcpy(m_log_level, &level, sizeof(int));

    m_cur_log_level = SyslogToSmlklog(level);
}

void EventCollector::Stop()
{
    SwitchEvent(false);
    SwitchActiveLog(false);
    shmdt(m_log_level);
}

void EventCollector::SwitchEvent(SMLK_BOOL swt)
{
    SMLK_LOGD("m_event_switch");
    if (m_event_switch.load() == swt) return;
    m_event_switch.store(swt);
    SMLK_LOGD("m_event_switch status = %s", m_event_switch.load() ? "true" : "false");
    SaveLogReportSwitch(swt);

    std::thread thread_ = std::thread([this]() {
        if (m_event_thread.joinable())
        {
            m_event_thread.join();
            return;
        }
        SMLK_LOGD("ready 2 start ProcessEvent ...");
        m_event_thread = std::thread(&EventCollector::ProcessEvent, this);
    });
    thread_.detach();
}

void EventCollector::SwitchActiveLog(SMLK_BOOL swt, SMLK_UINT8 log_type)
{
    if (m_act_log_switch.load() == swt) return;
    m_act_log_switch.store(swt);
    SMLK_LOGD("m_act_log_switch status = %s", m_act_log_switch.load() ? "true" : "false");
    std::thread thread_ = std::thread([this, log_type]() {
        if (!m_act_log_switch.load())
        {
            if (m_act_tsp_log_thread.joinable())
            {
                m_act_tsp_log_thread.join();
            }
            if (m_act_msg_log_thread.joinable())
            {
                m_act_msg_log_thread.join();
            }
            return;
        }
        if (log_type == 0)
        {
            SMLK_LOGD("ready 2 start ProcessActiveLog ...");
            std::string filename_ = LogFileName[0] + "TSPLog_" + m_imsi_str + "_tail_me.log";
            m_act_tsp_log_thread = std::thread(&EventCollector::ProcessActiveLog, this, filename_);
        }
        if (log_type == 1)
        {
            SMLK_LOGD("ready 2 start ProcessActiveLog ...");
            m_act_msg_log_thread = std::thread(&EventCollector::ProcessActiveLog, this, LogFileName[1]);
        }
    });
    thread_.detach();
}

void EventCollector::SetOfflineLogLevel(LogLevel level)
{
    SMLK_LOGD("SetOfflineLogLevel = %d", level);
    if (m_cur_log_level == (int)level)
        return;

    m_cur_log_level = (int)level; //save smlk log level

    int nSyslogT = SmlklogToSyslog((int)level);
    memcpy(m_log_level, &nSyslogT, sizeof(int));

    // 保存syslog等级, 下次重启通过脚本使得sdk日志等级设置生效。
    SaveOffLogLevel(nSyslogT);
}

LogLevel EventCollector::GetOfflineLogLevel()
{
    SMLK_LOGD("cur log level: %d", m_cur_log_level);
    return (LogLevel)m_cur_log_level;
}

void EventCollector::SetWarnValue(SMLK_UINT8 value)
{
    SMLK_LOGD("SetWarnValue %d",value);
    m_warnValue = value;
    SaveWarnValue(value);
}

void EventCollector::ProcessEvent()
{
    SMLK_LOGD("ProcessEvent");
    while (m_event_switch.load())
    {
        CheckAllAbnormalEventOneTime();

        //定时触发检测
        sleep(5);
    }
}

#define  FILE_LINE_LEN 1024
void EventCollector::ProcessActiveLog(IN std::string& path)
{
    if (path.empty()) return;
    std::vector<SMLK_UINT8> send_buf_;
    std::fstream ifs(path.c_str(), std::ios::in);
    std::size_t position_ = GetLastPreviousLogFileCursorWithLine(ifs, path);
    while (m_act_log_switch.load())
    {
        while ((!ifs.is_open()) && m_act_log_switch.load()) {
            // SMLK_LOGE("tail log fail re-try [%s]", path.c_str());
            ifs.open(path.c_str(), std::ios::in);
            sleep(1);
        }

        ifs.seekg(0, std::ios::end);
        position_ = ifs.tellg() < position_ ? 0 : position_;

        ifs.seekg(position_, std::ios::beg);
        char text[FILE_LINE_LEN];
        std::size_t len;
        while ((!ifs.eof()) && m_act_log_switch.load()) {
            memset(text, 0x0, FILE_LINE_LEN);
            ifs.getline(text, FILE_LINE_LEN);
            len = strlen(text);
            if (len == 0) continue;
            text[len] = '\n';
            position_ = ifs.tellg();
            // SMLK_LOGD("%s", text);
            if (send_buf_.size() > 2048)
            {
                //TODO send 2 server
                send_buf_.clear();
            }
            send_buf_.insert(send_buf_.end(), (SMLK_UINT8*)text, (SMLK_UINT8*)text + len);
            usleep(500);
        }
        ifs.close();
        sleep(1);
    }
    if (ifs.is_open())
    {
        ifs.close();
    }
}

std::size_t EventCollector::GetLastPreviousLogFileCursorWithLine(std::fstream& stream_, IN std::string& path)
{
    if (!stream_.is_open()) {
        SMLK_LOGE("log file not exist [%s]", path.c_str());
        return 0;
    }
    stream_.seekg(-1, std::ios::end);
    SMLK_UINT16 line_count = 0;
    char judge_;
    while (m_act_log_switch.load())
	{
        stream_.get(judge_);
		if (judge_ == '\n')
		{
			line_count ++;
			if (line_count == m_pre_line_short)
			{
				return stream_.tellg();
			}
		}
        stream_.seekg(-2, std::ios::cur);
        if (stream_.tellg() < 0)
        {
            SMLK_LOGD("out of file max lines ...");
            return 0;
        }
    }
    return 0;
}

void  EventCollector::OnMcuSerIgonIndication(smartlink_sdk::McuEventId id, void *data, int len)
{
    if (m_event_switch.load())
    {
        switch ( id ) {
            case smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE:
                {
                    smartlink_sdk::IGNState* ign = reinterpret_cast<smartlink_sdk::IGNState*>(data);
                    if (ign->on && !m_bIgOn.load())
                    {
                        SMLK_LOGD("================== ig on");
                        CheckAllAbnormalEventOneTime(true);
                    }

                    m_bIgOn.store(ign->on);
                }
                break;

            default:
                return;
        }
    }
}

/******************************************************************************
* NAME: OnSysprotertyIndication
*
* DESCRIPTION:
*   系统属性值变化回调，用于捕获FOTA失败事件。
*   判断属性处理定义的错误类型中， 上报foto失败事件，并将其属性重置为-1.
*
* PARAMETERS:
*   id:回调类型
*   info：属性信息。
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void EventCollector::OnSysprotertyIndication(smartlink_sdk::SysProperty::PropertyEventId id, smartlink_sdk::PropertyInfo* info)
{
    if (info == nullptr) {
        SMLK_LOGE("[property] PropertyInfo ptr is nullptr!!!");
        return;
    }

    if (!m_event_switch.load())
    {
        return;
    }

    switch (id)
    {
        case smartlink_sdk::SysProperty::PropertyEventId::E_SYS_PROPERTY_EVENT_VALUE_CHANGED:
            if (info->name == std::string(SYS_PRO_NAME_FOTA_FAIL_REPORT) && std::string(info->value) != " ")
            {
                std::string fotaUpdataErrInfo = std::string(info->value);
                SMLK_LOGD("[logger warnEvent] %s:%lld", fotaUpdataErrInfo.c_str(),std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());

                std::string property(SYS_PRO_NAME_FOTA_FAIL_REPORT);
                auto retCode = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, std::string(" "));
                if (smartlink_sdk::RtnCode::E_SUCCESS != retCode ) {
                    SMLK_LOGE("fail to set system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(retCode));
                }
            }
            break;
        default:
            break;
    }
}


void EventCollector::CheckAllAbnormalEventOneTime(SMLK_BOOL bIsIgTurnOn)
{
    if (m_event_switch.load())
    {
        XF54Head x54_cmd_resp;
        get_bcd_timestamp(x54_cmd_resp.time);
        x54_cmd_resp.seq_id = 0xffff;
        std::vector<SMLK_UINT8> uploadBuf;


        //开机事件检查
        struct sysinfo info;
        if (sysinfo(&info)) {
            SMLK_LOGW("Failed to get sysinfo");
        }
        // SMLK_LOGW("boot time: %d",info.uptime);
        if(!m_bBootEventIsReported.load() && info.uptime < 5*60)  /* 5分钟内触发*/
        {
            SMLK_LOGI("[logger warnEvent]  reboot lower than 5 min, cur run time:%d",info.uptime);
            m_bBootEventIsReported.store(true);
            //todo上报
            std::string strDesc="reboot lower than 5 min";
            EventType type = EventType::RebootType;
            x54_cmd_resp.msgLength = strDesc.length() + sizeof(EventType);

            uploadBuf.clear();
            uploadBuf.insert(uploadBuf.end(),(SMLK_UINT8*)&x54_cmd_resp,(SMLK_UINT8*)&x54_cmd_resp + sizeof(XF54Head));
            uploadBuf.insert(uploadBuf.end(),(SMLK_UINT8*)&type,(SMLK_UINT8*)&type+sizeof(SMLK_UINT16));
            uploadBuf.insert(uploadBuf.end(),strDesc.c_str(),strDesc.c_str()+strDesc.length());

            SendEventReportMsg(uploadBuf.data(),uploadBuf.size());
        }

        // CPU超限检查
        // 内存超限检查
        // 磁盘超限检查
        std::vector<smartlink_sdk::SysFileDiskInfo> diskInfo;
        smartlink_sdk::SysOS::GetInstance()->GetFileDiskInfo(diskInfo);
#if 0
        if(diskInfo.used_percent > m_warnValue)
        {
            SMLK_LOGD("disk userd over %d",m_warnValue);
            if(!m_bDiskOverIsReported.load() || bIsIgTurnOn)
            {
                m_bDiskOverIsReported.store(true);
                // 消息上报
                SMLK_LOGW("[logger warnEvent]  disk-userd-over msg is need report");

                WarnMsgId warnId = WarnMsgId::DiskOverWarn;
                WarnLevel warnLevel = WarnLevel::MidLevel;
                std::string strDesc="disk userd over ";
                strDesc.append(std::to_string(m_warnValue));

                x54_cmd_resp.msgLength = strDesc.length() + sizeof(WarnMsgId) + sizeof(warnLevel);
                uploadBuf.clear();
                uploadBuf.insert(uploadBuf.end(),(SMLK_UINT8*)&x54_cmd_resp, (SMLK_UINT8*)&x54_cmd_resp + sizeof(XF54Head));
                uploadBuf.insert(uploadBuf.end(),(SMLK_UINT8*)&warnId, (SMLK_UINT8*)&warnId + sizeof(WarnMsgId));
                uploadBuf.insert(uploadBuf.end(),(SMLK_UINT8*)&warnLevel, (SMLK_UINT8*)&warnId + sizeof(WarnLevel));
                uploadBuf.insert(uploadBuf.end(),strDesc.c_str(),strDesc.c_str()+strDesc.length());


                // SMLK_LOGD("====msg length: %d, %02x,%02x,%02x,%02x",uploadBuf.size(),uploadBuf.at(57),uploadBuf.at(58),uploadBuf.at(59),uploadBuf.at(60));
                SendEventReportMsg(uploadBuf.data(),uploadBuf.size());
            }
        }
        else
        {
            m_bDiskOverIsReported.store(false);
        } 
        #endif
    }
}

/******************************************************************************
* NAME: SyslogToSmlklog
*
* DESCRIPTION: syslog to smlklog
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
int EventCollector::SyslogToSmlklog(int nSysLevel)
{
    LogLevel retLevel = LogLevel::SMLK_LOG_LEVEL_Debug;
    auto const it = g_syslog_to_smlklog.find((smartlink_sdk::SysLogLevel)nSysLevel);
    if (it == g_syslog_to_smlklog.end())
    {
        SMLK_LOGE("cannot find syslog level %d", nSysLevel);
    }
    else
    {
        retLevel = it->second;
    }
    return (int)retLevel;
}

/******************************************************************************
* NAME: SmlklogToSyslog
*
* DESCRIPTION: smlklog to syslog
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
int EventCollector::SmlklogToSyslog(int nSmlkLevel)
{
    smartlink_sdk::SysLogLevel retLevel = smartlink_sdk::SysLogLevel::E_SYS_LOG_LEVEL_DEBUG;
    auto const it = g_smlklog_to_syslog.find((LogLevel)nSmlkLevel);
    if (it == g_smlklog_to_syslog.end())
    {
        SMLK_LOGE("cannot find smlklog level %d", nSmlkLevel);
    }
    else
    {
        retLevel = it->second;
    }
    return (int)retLevel;
}

void EventCollector::SaveLogReportSwitch(SMLK_BOOL bIsReport){
#if SYS_PROPERTY_OPEN
    auto retCode = smartlink_sdk::SysProperty::GetInstance()->SetValue(LOGGER_EVENT_REPORT_PROPERTY, bIsReport?"true":"false");
    if (smartlink_sdk::RtnCode::E_SUCCESS != retCode ) {
        SMLK_LOGE("fail to set system property \"%s\", return code: %d", LOGGER_EVENT_REPORT_PROPERTY, static_cast<std::int32_t>(retCode));
    }
#else
   std::ofstream ofs(std::string(LOGGER_EVENT_REPORT_SET));
    if (ofs.is_open() ) {
        ofs << bIsReport;
        ofs.close();
        SMLK_LOGE("write log_report success");
    } else {
        SMLK_LOGE("fail to save log_report to file=%s.", LOGGER_EVENT_REPORT_SET);
    }
#endif
}


SMLK_BOOL EventCollector::GetLogReportSwitch(){
#if SYS_PROPERTY_OPEN
    std::string property(LOGGER_EVENT_REPORT_PROPERTY);
    std::string strReportSwitch = "false";
    auto retCode = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, strReportSwitch);
    if (smartlink_sdk::RtnCode::E_SUCCESS != retCode ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", LOGGER_EVENT_REPORT_PROPERTY, static_cast<std::int32_t>(retCode));
    }

    return strReportSwitch == "true" ? true : false;
#else
    std::string str_log_switch = "0";
    if (!access(LOGGER_EVENT_REPORT_SET, F_OK))
    {
        std::ifstream   ifs(LOGGER_EVENT_REPORT_SET);
        if ( !ifs.is_open() ) {
            SMLK_LOGE("fail to open log_report file: %s", LOGGER_EVENT_REPORT_SET);
            return false;
        }

        std::getline(ifs, str_log_switch);
        ifs.close();

        return str_log_switch == "1";
    }

    return false;
#endif
}

void EventCollector::SaveOffLogLevel(int level)
{
    char strI[10]={0};
    FILE *fp =  fopen(LOGGER_LOGGER_LEVEL_SET, "w+");
    sprintf(strI, "%d",level);
    fputs(strI, fp);
    fprintf(fp,"\n");
    fflush(fp);
    fsync(fileno(fp));
    fclose(fp);
}

SMLK_BOOL EventCollector::GetOffLogLevel(int &level){
    std::string log_level_str;

    if (!access(LOGGER_LOGGER_LEVEL_SET, F_OK))
    {
        std::ifstream   ifs(LOGGER_LOGGER_LEVEL_SET);
        if ( ifs.is_open() ) {
            std::getline(ifs, log_level_str);
            ifs.close();
            try
            {
                level = std::atoi(log_level_str.c_str());
                return true;
            }
            catch(const std::exception& e)
            {
                SMLK_LOGW("fail stoid log_level  %s", e.what());
            }
        }
        else
        {
            SMLK_LOGW("fail to open log_level file: %s", LOGGER_LOGGER_LEVEL_SET);
        }
    }
    return false;
}

void EventCollector::SaveWarnValue(SMLK_UINT8 value){
#if SYS_PROPERTY_OPEN
    std::string property(LOGGER_WARN_VALUE_PROPERTY);
    auto retCode = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, std::to_string(value));
    if (smartlink_sdk::RtnCode::E_SUCCESS != retCode ) {
        SMLK_LOGE("fail to set system property \"%s\", return code: %d", property, static_cast<std::int32_t>(retCode));
    }
#else
    std::string log_level_str_ = std::to_string(value);
    std::ofstream   ofs(std::string(LOGGER_WARN_VALUE_SET));
    if ( ofs.is_open() ) {
        ofs << log_level_str_;
        ofs.close();
    } else {
        SMLK_LOGE("fail to save warn-value to file=%s.", LOGGER_WARN_VALUE_SET);
    }
#endif
}

SMLK_BOOL EventCollector::GetWarnValue(SMLK_UINT8 &value){
#if SYS_PROPERTY_OPEN
    SMLK_BOOL retBool = true;
    std::string strWarnValue = "80";
    std::string property(LOGGER_WARN_VALUE_PROPERTY);
    auto retCode = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, strWarnValue);
    if (smartlink_sdk::RtnCode::E_SUCCESS != retCode ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", LOGGER_WARN_VALUE_PROPERTY, static_cast<std::int32_t>(retCode));
        retBool = false;
    }

    try
    {
        value = atoi(strWarnValue.c_str());
    }
    catch(const std::exception& e)
    {
        SMLK_LOGW("get logger_log_level err %d",e.what());
        retBool = false;
    }
    return retBool;
#else
    std::string str_warn_value;
    if (!access(LOGGER_WARN_VALUE_SET, F_OK))
    {
        std::ifstream   ifs(LOGGER_WARN_VALUE_SET);
        if ( !ifs.is_open() ) {
            SMLK_LOGE("fail to open warn-value file: %s", LOGGER_WARN_VALUE_SET);
            return false;
        }

        std::getline(ifs, str_warn_value);
        ifs.close();

        try
        {
            value = std::atoi(str_warn_value.c_str());
            return true;
        }
        catch(const std::exception& e)
        {
            SMLK_LOGE("fail stoid warn-value  %s", e.what());
        }
    }
    return false;
#endif
}

void EventCollector::SendEventReportMsg(IN SMLK_UINT8* buf, std::size_t len){
    IpcTspHead ipc_head;
    std::memset(&ipc_head, 0x00, sizeof(IpcTspHead));
    ipc_head.msg_id = JTT808_LOG_REPORT_ARK;
    ipc_head.seq_id = htobe16(0xffff);
    ipc_head.protocol = (SMLK_UINT8)ProtoclID::E_PROT_JTT808;
    ipc_head.qos = QOS_SEND_ALWAYS;

    TspServiceApi::getInstance()->SendMsg(ipc_head, buf, len);
}

} // namespace logger
} // namespace smartlink

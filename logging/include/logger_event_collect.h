/*****************************************************************************/
/**
* \file       logger_event_collect.h
* \author     wukai
* \date       2021/08/09
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
#ifndef _LOGGER_EVENT_COLLECT_H_
#define _LOGGER_EVENT_COLLECT_H_
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/

#include "smlk_types.h"
#include "smartlink_sdk_mcu.h"
#include "smartlink_sdk_sys_property.h"
#include <atomic>
#include <thread>

namespace smartlink {
namespace logger {
enum class LogLevel : SMLK_UINT8 {
    SMLK_LOG_LEVEL_Error = 0,
    SMLK_LOG_LEVEL_Warning,
    SMLK_LOG_LEVEL_Info,
    SMLK_LOG_LEVEL_Debug,
};
class EventCollector
{
public:
    EventCollector();
    ~EventCollector();

    void Init();
    void Stop();

    void SwitchEvent(SMLK_BOOL);
    void SwitchActiveLog(SMLK_BOOL, SMLK_UINT8 log_type = 0/*default tsp log*/);
    void SetOfflineLogLevel(LogLevel);
    LogLevel GetOfflineLogLevel();
    std::string& GetImsiStr(){ return m_imsi_str;};

    void SetWarnValue(SMLK_UINT8 value);


protected:
    void ProcessEvent();
    void ProcessActiveLog(IN std::string& path);

private:
    std::size_t GetLastPreviousLogFileCursorWithLine(std::fstream& stream, IN std::string& path);

    //igon callback
    void OnMcuSerIgonIndication(smartlink_sdk::McuEventId id, void *data, int len);
    void OnSysprotertyIndication(smartlink_sdk::SysProperty::PropertyEventId id, smartlink_sdk::PropertyInfo* info);
    void CheckAllAbnormalEventOneTime(SMLK_BOOL bIsIgTurnOn = false);

    //smlklog & syslog  translate
    int SyslogToSmlklog(int);
    int SmlklogToSyslog(int);


    //file content sava and read
    void SaveLogReportSwitch(SMLK_BOOL bIsReport);
    SMLK_BOOL GetLogReportSwitch();

    void SaveOffLogLevel(int);
    SMLK_BOOL GetOffLogLevel(int &);

    void SaveWarnValue(SMLK_UINT8);
    SMLK_BOOL GetWarnValue(SMLK_UINT8 &);


    //active send  warn-event-msg
    void SendEventReportMsg(IN SMLK_UINT8* buf, std::size_t len); //default msg ipc-head


private:
    std::string m_imsi_str;
    SMLK_UINT16 m_pre_line_short;
    std::atomic<SMLK_BOOL> m_event_switch;
    std::atomic<SMLK_BOOL> m_act_log_switch;
    std::thread m_event_thread;
    std::thread m_act_tsp_log_thread;
    std::thread m_act_msg_log_thread;
    int* m_log_level;
    int m_cur_log_level;
    int m_shmd_id;


    std::atomic<SMLK_BOOL> m_bDiskOverIsReported;
    std::atomic<SMLK_BOOL> m_bIgOn;
    std::atomic<SMLK_BOOL> m_bBootEventIsReported;

    SMLK_UINT8 m_warnValue;

};

} // namespace logger
} // namespace smartlink




#endif // _LOGGER_EVENT_COLLECT_H_
/*****************************************************************************/
/**
* \file       logger_monitor.h
* \date       2021/10/26
* \author     wujian
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#ifndef _LOGGER_MONITOR_H_
#define _LOGGER_MONITOR_H_

#include "smlk_types.h"

#include <thread>
#include <map>
#include <string>

namespace smartlink {
namespace logger {

class LoggerMonitor
{

public:
    LoggerMonitor(/* args */);
    ~LoggerMonitor();

    void ThreadRun();

private:
    void MonitorAllLog();

    SMLK_UINT32 GetAllLogTotalSize();
    SMLK_UINT32 GetDirectorySize(const char *dir);
    SMLK_BOOL CheckIsHaveCoreDump();
    void DeleteOver7DayLog();
    void DeleteCoreDumpFile();
    void DeleteAllFile(std::string dirPath);

private:
    bool m_bIsIgOn = false;

};

} // namespace logger
} // namespace smartlink

#endif
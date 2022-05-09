/*****************************************************************************/
/**
* \file       logger.cpp
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
#include "Poco/Thread.h"

int main() {
    Poco::Thread thread_;
    smartlink::LoggerClient runnable;
    thread_.start(runnable);
    thread_.join();
    return 0;
}
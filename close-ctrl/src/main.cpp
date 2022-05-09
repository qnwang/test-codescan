/*****************************************************************************/
/**
* \file       main.cpp
* \author     weixuefeng
* \date       2021/06/08
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/

#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <getopt.h>
#include "cctrl_service.h"
#include "smlk_signal_handle.h"
#include "smlk_log.h"

using namespace smartlink;
using namespace smartlink::CloseCtrl;

namespace {
    volatile sig_atomic_t signal_status = 0;
}

int main( void )
{
	CloseCtrlService ser;
    SmartlinkSignal::Init();
    if ( SMLK_RC::RC_OK != ser.Init() ) {
        SMLK_LOGE("Cctrl service init failed!");
        return -1;
    }
    if ( SMLK_RC::RC_OK != ser.Start() ) {
        SMLK_LOGE("Cctrl service start failed!");
    }
    while ( (signal_status != SIGTERM) && (signal_status != SIGINT) ) {
        if ( !ser.IsRunning() ) {
            SMLK_LOGE("Cctrl service stopped abnormallly!");
            break;
        }

        std::this_thread::sleep_for(std::chrono::seconds(250));
    }
    ser.Stop();
}

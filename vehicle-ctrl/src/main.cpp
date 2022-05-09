/*****************************************************************************/
/**
 * \file       main.cpp
 * \author     huangxin
 * \date       2020/12/16
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
#include "vehctrl_service.h"
#include "smlk_signal_handle.h"

using namespace smartlink;

namespace
{
    volatile sig_atomic_t signal_status = 0;
}

int main(void)
{
    VehicleCtrlService ser;
    SmartlinkSignal::Init();
    if (SMLK_RC::RC_OK != ser.Init())
    {
        std::cerr << "service init failed" << std::endl;
        return -1;
    }
    if (SMLK_RC::RC_OK != ser.Start())
    {
        std::cerr << "service start failed" << std::endl;
    }
    while ((signal_status != SIGTERM) && (signal_status != SIGINT))
    {
        if (!ser.IsRunning())
        {
            std::cerr << "service stopped abnormallly" << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    ser.Stop();
}

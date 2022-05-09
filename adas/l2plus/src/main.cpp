/*****************************************************************************/
/**
* \file       main.cpp
* \date       2022/02/28
* \author     wujian
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>

#include "l2_service.h"
#include "adas_define_common.h"

smartlink::L2Service ser;

void AdasMessageNotify(const char* msg_data, const unsigned int* msg_id, int msg_num)
{
    ser.AdasMessageNotify(msg_data, msg_id, msg_num);
}

int main(int argc, char *argv[]) {

    if (M_L_RETURN_SUCCESS != ser.Init())
    {
        SMLK_LOGW("service init err...");
        return -1;
    }

    if(M_L_RETURN_SUCCESS != ser.Start( ))
    {
        SMLK_LOGW("service start err...");
        return -1;
    }

    //特殊： 只能注册普通方法， 所以通过声明全局L2Service对象来让普通方法将数据传入类对象中
    adas_set_message_callback(AdasMessageNotify);

    while (true) {
        if ( !ser.IsRunning() ) {
            SMLK_LOGE("service not run...");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    ser.Stop();

    return 0;
}


#endif

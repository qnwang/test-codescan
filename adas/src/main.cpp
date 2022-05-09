/*****************************************************************************/
/**
* \file       main.cpp
* \date       2022/04/12
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

#include "adas_service.h"
using namespace smartlink;


AdasService ser;

int Av2hpMessageNotify(const char *message, int *message_num)
{
    ser.Av2hpMessageNotify(message, message_num);
    return 0;
}

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

    if (ser.IsPccOrL2Plus())
    {
        OnAv2HP_setMessageCB pMsgcb = NULL;
        ser.GetHPMsgCb(pMsgcb);
        if (pMsgcb != NULL)
        {
            pMsgcb(Av2hpMessageNotify);
        }
    }
    else
    {
        adas_set_message_callback(AdasMessageNotify);
    }

    while (true) {
        if ( !ser.IsRunning() ) {
            SMLK_LOGE("service not run...");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    ser.Stop();

}

#endif
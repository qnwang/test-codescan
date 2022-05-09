//
// Created by work on 2021/7/17.
//

#ifndef SMARTLINKT_TBOX_MESSAGE_VEHICLELOGIN_32960_H
#define SMARTLINKT_TBOX_MESSAGE_VEHICLELOGIN_32960_H

#pragma once

#include <smlk_signal.h>
#include "ev_signals.h"

#include "common_32960.h"

#include "message_inf.h"
#include "message_def_32960.h"
#include "message_observer_32960.h"

#include "EV_Timer.h"
#include "TimerClient.h"
#include "TimersPoll.h"

namespace smartlink {

    class MessageVehicleLogin32960 : public IMessage {
    public:
        MessageVehicleLogin32960();

        ~MessageVehicleLogin32960();

        virtual void decodeMsg(char *rawData, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength) override;

        virtual void broadcastMsgAck(uint msgId, uint rspFlag) override;

        void DoVehicleLogin();

        static void delLoginTimer();

    private:

        static void loginInit();

        static void loginLoop();

        static int addLoginTimer();

        static int checkLoginTimer();

        GB_32960_VehicleLogin   m_vehicleLoginData;
        GB_32960_MsgEntity      m_MessageEntity32960;

        static EV_Timer *loginTimer;

        bool logining;                  //是否正在登录
        int curTryNum;                  //当前登录次数
        int maxRetryNum;                //最大登录重试次数
        int nextLoginIntervalTime;      //登录重试间隔（秒）
        int curWaitTime;                //当前已等待时间
        int maxWaitTime;                //等待服务器响应时间(固定值)
        bool nextLoginFlag;             //是否开始等待下一次登录
        int nextLoginWaitTime;          //下一次登录已等待的时间

    };  // class MessageVehicleLogin32960
}   // namespace smartlink


#endif //SMARTLINKT_TBOX_MESSAGE_VEHICLELOGIN_32960_H

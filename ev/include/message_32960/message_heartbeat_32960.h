//
// Created by work on 2021/10/11.
//

#ifndef SMARTLINKT_TBOX_MESSAGE_HEARTBEAT_32960_H
#define SMARTLINKT_TBOX_MESSAGE_HEARTBEAT_32960_H

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

    class MessageHeartBeat32960 : public IMessage {
    public:
        MessageHeartBeat32960();

        ~MessageHeartBeat32960() = default;

        virtual void decodeMsg(char *rawData, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength) override;

        virtual void broadcastMsgAck(uint msgId, uint rspFlag) override;

        void run();

    private:

        void DoHeartBeatUp();

        static void heartBeatUpLoop(void *arg);

        // static void reSendTask();

        // static int stopHeartBeatTimer();

        // static int addHeartBeatTimer();

        GB_32960_MsgEntity m_MessageEntity32960;

        // static EV_Timer *heartbeat_timer; //重发任务

        // static int reTransTimes; //重传次数

    }; // class MessageHeartBeat32960
} // namespace smartlink

#endif //SMARTLINKT_TBOX_MESSAGE_HEARTBEAT_32960_H

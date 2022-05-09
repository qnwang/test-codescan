//
// Created by work on 2021/7/16.
//

#ifndef SMARTLINKT_TBOX_MESSAGE_BLINDINFO_32960_H
#define SMARTLINKT_TBOX_MESSAGE_BLINDINFO_32960_H

#pragma once

#include <smlk_signal.h>
#include "ev_signals.h"

#include "common_32960.h"

#include "message_inf.h"
#include "message_def_32960.h"
#include "message_observer_32960.h"

namespace smartlink {

    class MessageBlindInfoUp32960 : public IMessage {
    public:
        MessageBlindInfoUp32960();

        ~MessageBlindInfoUp32960() = default;

        virtual void decodeMsg(char *rawData, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength) override;

        virtual void broadcastMsgAck(uint msgId, uint rspFlag) override;

        void run();

    private:

        static void blindDataUpLoop();

        static int bufLength;

        static char buf[M_GB_32960_SZ_DATA_MAX];

    }; // class MessageBlindInfoUp32960
}   // namespace smartlink

#endif //SMARTLINKT_TBOX_MESSAGE_BLINDINFO_32960_H

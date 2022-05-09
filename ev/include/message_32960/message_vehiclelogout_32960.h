//
// Created by work on 2021/7/17.
//

#ifndef SMARTLINKT_TBOX_MESSAGE_VEHICLELOGOUT_32960_H
#define SMARTLINKT_TBOX_MESSAGE_VEHICLELOGOUT_32960_H

#pragma once

#include <smlk_signal.h>
#include "ev_signals.h"

#include "common_32960.h"

#include "message_inf.h"
#include "message_def_32960.h"
#include "message_observer_32960.h"

namespace smartlink {

    class MessageVehLogout32960 : public IMessage {
    public:
        MessageVehLogout32960();

        ~MessageVehLogout32960();

        virtual void decodeMsg(char *rawData, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength) override;

        virtual void broadcastMsgAck(uint msgId, uint rspFlag) override;

        void DoVehLogout();

    private:
        GB_32960_VehicleLogout      m_vehicleLogoutData;
        GB_32960_MsgEntity          m_MessageEntity32960;

    };  // class MessageVehicleLogin32960
}   // namespace smartlink

#endif //SMARTLINKT_TBOX_MESSAGE_VEHICLELOGOUT_32960_H

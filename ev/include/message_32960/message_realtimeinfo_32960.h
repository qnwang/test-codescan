//
// Created by work on 2021/7/18.
//

#ifndef SMARTLINKT_TBOX_MESSAGE_REALTIMEINFO_32960_H
#define SMARTLINKT_TBOX_MESSAGE_REALTIMEINFO_32960_H

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

#define CACHE_PKG_NUM 30

namespace smartlink {

/**
 * @brief 缓存实时数据，供3级警告使用
*/
    struct CacheRTInfo {
        SMLK_UINT16 body_len;
        GB_32960_MsgEntity info;
    };

    struct CacheRTInfoReissue {
        CacheRTInfo cachertinfo;
        std::deque<CacheRTInfo> rtinfo_vlot_cache;
    };

    enum VALUE_STATUS {
        VALUE_EXCEPT_BYTE   = 0xFE,
        VALUE_INVALID_BYTE  = 0xFF,
        VALUE_EXCEPT_WORD   = 0xFFFE,
        VALUE_INVALID_WORD  = 0xFFFF,
        VALUE_EXCEPT_DWORD  = 0xFFFFFFFE,
        VALUE_INVALID_DWORD = 0xFFFFFFFF,
    };

    class MessageRealTimeInfo32960 : public IMessage {
    public:
        MessageRealTimeInfo32960();

        ~MessageRealTimeInfo32960();

        virtual void decodeMsg(char *rawData, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length) override;

        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength) override;

        virtual void broadcastMsgAck(uint msgId, uint rspFlag) override;

        void run(MessageRealTimeInfo32960 *);

        void deleInfoUpTimer();

    private:

        void DoRealTimeInfoUp();

        static void dataUpLoop();

        int countSendTime;  //发送时间间隔
        int m_rt_send_num;  //实时数据发送个数
        int m_is_send_num;  //补发数据发送个数

        SMLK_UINT32 m_last_common_warn;
        
        static EV_Timer *RTInfoUpTimer;
        static uint bufLength;
        static char buf[M_GB_32960_SZ_DATA_MAX];
        static MessageRealTimeInfo32960 *RTInfoUpPtr; 

        GB_32960_RealTimeReport     m_dataUpload;
        GB_32960_RealTimeReport     m_dataLastReport;
        GB_32960_RechageDeviceVoltInfo  m_last_volt_info;
        GB_32960_RechageDeviceTempInfo  m_last_temp_info;

        GB_32960_MsgEntity          m_MessageEntity32960;

        std::deque<CacheRTInfo> g_rtinfo_cache;                 // 存放已发送的实时数据
        std::deque<CacheRTInfo> g_rtinfo_vlot_cache;            // 存放多包电池数据
        std::deque<CacheRTInfoReissue> g_rtinfo_cache_reissue;  // 存放实时数据包含电池多包数据


    }; // class MessageRealTimeInfo32960
}   // namespace smartlink


#endif //SMARTLINKT_TBOX_MESSAGE_REALTIMEINFO_32960_H

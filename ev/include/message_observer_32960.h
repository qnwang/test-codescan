//
// Created by work on 2021/7/16.
//

#ifndef SMARTLINKT_TBOX_MESSAGE_OBSERVER_32960_H
#define SMARTLINKT_TBOX_MESSAGE_OBSERVER_32960_H


#include <vector>
#include <list>
#include <map>
#include <ostream>
#include <mutex>
#include <functional>

#include "message_inf.h"

#include "message_def_32960.h"

namespace smartlink {

#define HEADLENTH 24    //除dataBody 和 校验码
    //socket send callback
    typedef std::function<int(char *, uint)> msgSendCallback32960;

    class MessageObserver32960 {
    public:
        static MessageObserver32960 *GetInstance();

        void EndianSwap(uint8_t *pData, int startIndex, int length);

        /**
        * 解TSP数据包
        */
        void decodeMsg(char *src, uint length);

        /**
        * 组包 编码
        */
        int encodeMsg(GB_32960_MsgEntity entity, uint bodyLength);

        /**
        * 组包 编码 存储盲区数据 各业务类自行处理存库存数据库
        */
        int encodeMsg(GB_32960_MsgEntity entity, uint bodyLength, char *dest, uint &msgLength);

        /**
        * 重传数据包
        */
        int resendMsg(char *buf, uint len);

        /**
        * socket send callback
        */
        void RegisterDataSendCB(msgSendCallback32960 cb);

        /**
        * 增加需要监听消息的对象类
        */
        void AttachObserver(msgID_32960 id, IMessage *MessageOb);

        /**
        * 删除监听消息的对象类
        */
        void DetachObserver();

        /**
        * 根据命令id 发送给对应的业务类
        */
        void NotifyObserver(msgID_32960 id, char *messageContent, uint len);

        /**
        * ack的广播 用于业务类自行删除定时器
        */
        void NotifyMsgAck(uint msgId, uint rspFlag);

    private:
        MessageObserver32960() = default;

        ~MessageObserver32960();

        static MessageObserver32960 *m_instance;
        static std::mutex m_mtx;
        std::vector <std::pair<msgID_32960, IMessage *>> ServerVec;
        msgSendCallback32960 m_callBack;

    };

}   // namespace smartlink

#endif //SMARTLINKT_TBOX_MESSAGE_OBSERVER_32960_H

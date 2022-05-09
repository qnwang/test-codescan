/*****************************************************************************/
/**
 * \file       avm_service_inf.h
 * \author     wukai
 * \date       2021/07/20
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef SMLK_AVM_SERVICE_INF_H_
#define SMLK_AVM_SERVICE_INF_H_
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include "avm_msg_builder.h"
#include "smlk_log.h"
#include "smlk_queue.h"
#include "Poco/Activity.h"
#include "Poco/Net/StreamSocket.h"
#include <map>
#define AVM_SYNC_SERVICE_OPEN 1
#define DMSPORT 7002
#define AVMPORT 7003

namespace smartlink
{
    namespace AVM
    {
        using NotifyDelegate = std::function<void()>;
        using AvmEventListener = std::function<void(AvmMessage &)>;
        using StreamSocketSet = std::function<void(IN Poco::Net::StreamSocket &)>;
        enum class NotifyType : std::uint8_t
        {
            AVM_NOTIFY_ONRECV = 0,
            AVM_NOTIFY_ONCONNECT
        };
        class AvmServiceBase
        {
        public:
            virtual ~AvmServiceBase(){};
            virtual void Start() = 0;
            virtual void Stop() = 0;
            virtual void Init(){};
            virtual bool SendMsg(AvmMessage &msg, bool no_resp = false) = 0;
            virtual bool IsRunning() { return m_isRunning.load(); };
            virtual void RegisterEventListener(IN AvmEventListener &listener)
            {
                if (listener)
                    m_listener = listener;
            };
            virtual void RegisterNotifyListener(IN NotifyDelegate &delegate, NotifyType type)
            {
                m_notify.insert(std::make_pair(type, delegate));
            };
            DeviceType getDeviceType() { return m_device_type; }

        protected:
            virtual void SetSocket(IN Poco::Net::StreamSocket &s)
            {
                m_socket = s;
                // TODO send time_sync termintal_id_sync version_sync
                auto it = m_notify.find(NotifyType::AVM_NOTIFY_ONCONNECT);
                if (it != m_notify.end())
                {
                    if (it->second)
                        (it->second)();
                }
            };
            virtual void ReplyHeartRsp() = 0;
            virtual void SendMsg2Tbox(AvmMessage &msg) = 0;

        protected:
            std::atomic<bool> m_isRunning;
            std::map<NotifyType, NotifyDelegate> m_notify;
            Poco::Net::StreamSocket m_socket;
            AvmEventListener m_listener;
            DeviceType m_device_type; /*判断当前连接的设备为Dms或Avm*/
        };
    }
}

#endif // SMLK_AVM_SERVICE_INF_H_
/*****************************************************************************/
/**
 * \file       avm_sync_service.h
 * \author     wukai
 * \date       2021/07/20
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef SMLK_AVM_SYNC_SERVICE_H_
#define SMLK_AVM_SYNC_SERVICE_H_
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include <vector>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <functional>

#include "Poco/Activity.h"
#include "Poco/Net/StreamSocket.h"
#include "Poco/Event.h"

#include "smlk_log.h"
#include "smlk_tools.h"
#include "smlk_queue.h"
#include "smlk_module_inf.h"
#include "smlk_timer_new.h"
#include "avm_service_inf.h"

namespace smartlink
{
    namespace AVM
    {
        class AvmMessage;
        class AVMSyncService : public AvmServiceBase /*: public server_events*/
        {
        public:
            AVMSyncService(std::string host, SMLK_UINT16 nPort);
            virtual ~AVMSyncService();

            virtual void Init();
            virtual void Start();
            virtual void Stop();
            virtual bool SendMsg(AvmMessage &msg, bool no_resp = false);

        protected:
            virtual void SendMsg2Tbox(AvmMessage &msg);
            void NotifyOne();
            void NotifyHeartBeatResp();
            virtual void ReplyHeartRsp();

        private:
            void runActivity();

        private:
            SMLK_UINT16 m_port;
            std::string m_host;
            Poco::Activity<AVMSyncService> m_activity;
            std::shared_ptr<Poco::Event> m_sp_evt;
            std::shared_ptr<Poco::Event> m_sp_evt_heart;
            std::mutex m_send_mtx;
            std::shared_ptr<ITimer> m_timer;
            AvmMessage m_recv_msg;
        };
    }
}

#endif // SMLK_AVM_SYNC_SERVICE_H_
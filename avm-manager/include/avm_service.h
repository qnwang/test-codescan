/*****************************************************************************/
/**
 * \file       avm_service.h
 * \author     wukai
 * \date       2021/07/20
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef SMLK_AVM_SERVICE_H_
#define SMLK_AVM_SERVICE_H_
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include <vector>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <functional>

#include "smlk_timer_new.h"
#include "avm_service_inf.h"

namespace smartlink
{
    namespace AVM
    {
        class AVMService : public AvmServiceBase /*: public server_events*/
        {
        public:
            AVMService(std::string host, SMLK_UINT16 nPort);
            virtual ~AVMService();

            virtual void Start();
            virtual void Stop();
            virtual bool SendMsg(AvmMessage &msg, bool no_resp = false);

        protected:
            virtual void ReplyHeartRsp();
            virtual void SendMsg2Tbox(AvmMessage &msg);

        private:
            void runActivity();
            std::size_t PostMsg(IN AvmMessage &msg);

        private:
            SMLK_UINT16 m_port;
            std::string m_host;
            Poco::Activity<AVMService> m_activity;
            Queue<AvmMessage> m_send_buf;
            std::atomic<bool> m_isRunning;
            std::mutex m_send_mtx;
            std::shared_ptr<ITimer> m_timer;
        };

    }
}

#endif // SMLK_AVM_SERVICE_H_
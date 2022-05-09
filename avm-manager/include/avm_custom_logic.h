/*****************************************************************************/
/**
 * \file       avm_custom_logic.h
 * \author     wukai
 * \date       2021/08/03
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef SMLK_AVM_CUSTOM_LOGIC_H_
#define SMLK_AVM_CUSTOM_LOGIC_H_
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include "avm_service_inf.h"
#include "Poco/Event.h"
#include "smlk_queue.h"

#include <thread>

namespace smartlink
{
    namespace AVM
    {
        static const SMLK_UINT8 SYNC_AVM_ALL = 0;
        static const SMLK_UINT8 SYNC_AVM_VERSION = 1;
        static const SMLK_UINT8 SYNC_AVM_TIME = 2;
        static const SMLK_UINT8 SYNC_AVM_TERMINAL_ID = 3;
        class AvmCustomUtil final
        {
        public:
            AvmCustomUtil(IN std::shared_ptr<AvmServiceBase> &sp)
                : m_sp_avm_service(sp), m_avm_version(""), m_sp_evt(new Poco::Event), m_working(true)
            {
                m_thread = std::thread(&AvmCustomUtil::DealProcess, this);
            };
            ~AvmCustomUtil()
            {
                m_working = false;
                m_sp_evt.reset();
                if (m_thread.joinable())
                    m_thread.join();
                m_sp_evt.reset();
            };

        public:
            SMLK_BOOL SyncAll(IN std::string &imsi);
            SMLK_BOOL GetAvmVersion(std::string &version, SMLK_BOOL block = false);
            SMLK_BOOL SyncTime(SMLK_BOOL block = false);
            SMLK_BOOL SyncTerminalId(SMLK_BOOL block = false);

        private:
            void DealProcess();
            SMLK_BOOL GetAvmVersionBlock(std::string &version);
            std::pair<SMLK_UINT32, SMLK_UINT16> GetCustomTime();

        private:
            std::shared_ptr<AvmServiceBase> m_sp_avm_service;
            std::string m_avm_version;
            std::string m_imsi;
            std::thread m_thread;
            std::shared_ptr<Poco::Event> m_sp_evt;
            SMLK_BOOL m_working;
            Queue<SMLK_UINT8> m_task;
        };
    }
}

#endif // SMLK_AVM_CUSTOM_LOGIC_H_
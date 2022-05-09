/*****************************************************************************/
/**
 * \file       avm_manager.h
 * \author     wukai
 * \date       2021/07/20
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef SMLK_TSP_CLIENT_H_
#define SMLK_TSP_CLIENT_H_
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include "tsp_service_api.h"
#include "smlk_error.h"

#include "avm_service_inf.h"
#include "jit808_avm_def.h"
#include "smlk_module_inf.h"
#include "avm_custom_logic.h"
#include "avm_vehicle_data_collect.h"

#include <memory>
#include <thread>

namespace smartlink
{
    namespace AVM
    {
        class TspQueueMsg
        {
        public:
            TspQueueMsg()
            {
                bzero(&m_general_head, sizeof(GenrnalHead));
                bzero(&m_data_9102, sizeof(Data9102));
                bzero(&m_data_9205, sizeof(Data9205));
                bzero(&m_data_8f62, sizeof(Data8F62));
                bzero(&m_data_9202, sizeof(Data9202));
                bzero(&m_data_9207, sizeof(Data9207));
            };
            virtual ~TspQueueMsg(){};

        public:
            GenrnalHead m_general_head;
            Data8F61 m_data_8f61;
            Data9102 m_data_9102;
            Data9205 m_data_9205;
            Data8F62 m_data_8f62;
            Data9202 m_data_9202;
            Data9206 m_data_9206;
            Data9207 m_data_9207;
            Data8F49_F368 m_data_8f49_f368;
            std::vector<SMLK_UINT8> data;
        };
        class VedioProcessorManager : public BaseModule
        {
        public:
            VedioProcessorManager();
            virtual ~VedioProcessorManager();
            virtual SMLK_RC Init();
            virtual SMLK_RC Start();
            virtual void Stop();
            void Loop();

        private:
            void OnTspIndication(IN IpcTspHead &ipc_head, IN void *data, std::size_t len);
            void AvmServiceInitOrRestart(std::shared_ptr<AvmServiceBase> &sptrService, std::shared_ptr<AvmCustomUtil> &sptrUtil, SMLK_UINT16 port);
            void TspMsgProcess();
            void SendCommonResp(IN TspQueueMsg &msg, AVMCommonRC result);
            void SendActionResp(IN TspQueueMsg &msg, SMLK_UINT8 *data, std::size_t len);
            std::size_t Decode8F61(IN SMLK_UINT8 *data, TspQueueMsg &msg);
            void Decode9206(IN SMLK_UINT8 *data, TspQueueMsg &msg, std::size_t len);
            void OnDeviceMessage(AvmMessage &msg);

        private:
            std::thread m_tsp_msg_thread;
            Queue<TspQueueMsg> m_tsp_queue;
            std::shared_ptr<AvmServiceBase> m_sp_avmservice;
            std::shared_ptr<AvmServiceBase> m_sp_dmsservice;
            std::shared_ptr<AvmCustomUtil> m_sp_avm_util;
            std::shared_ptr<AvmCustomUtil> m_sp_dms_util;
            std::shared_ptr<DataProcessor> m_sp_dataproc;
            bool m_dms_exist;
            bool m_avm_exist;
        };
    }
}

#endif // SMLK_TSP_CLIENT_H_
/*****************************************************************************/
/**
 * \file       avm_service.cpp
 * \author     wukai
 * \date       2021/07/20
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include "avm_service.h"
#include "smlk_log.h"

#include "Poco/Net/TCPServer.h"
#include "Poco/Net/TCPServerConnection.h"
#include "Poco/Net/TCPServerConnectionFactory.h"
#include "Poco/Net/TCPServerParams.h"
#include "Poco/Net/ServerSocket.h"
#include "Poco/Thread.h"

namespace smartlink
{
    namespace AVM
    {
        using Poco::Thread;
        using Poco::Net::ServerSocket;
        using Poco::Net::SocketAddress;
        using Poco::Net::StreamSocket;
        using Poco::Net::TCPServer;
        using Poco::Net::TCPServerConnection;
        using Poco::Net::TCPServerConnectionFactory;
        using Poco::Net::TCPServerConnectionFactoryImpl;
        using Poco::Net::TCPServerConnectionFilter;
        using Poco::Net::TCPServerParams;

        class EchoConnection : public TCPServerConnection
        {
        public:
            EchoConnection(const StreamSocket &s, AVMService *p,
                           NotifyDelegate heartbeat,
                           AvmEventListener listener,
                           StreamSocketSet socketset)
                : TCPServerConnection(s), _parent(p), _heartbeat(heartbeat), _onmsg(listener), _setsocket(socketset)
            {
            }

            void run()
            {
                StreamSocket &ss = socket();
                _setsocket(ss);
                // _parent->OnErrorCode(ERROR_CODE_TCPSERVER_CONNECTED);
                SMLK_UINT8 head_buf[8];
                int flags = 0;
                int len;
                while (_parent->IsRunning())
                {
                    try
                    {
                        // TODO  while make the receiver
                        if (!ss.poll(60000, 1))
                        {
                            usleep(50000); // 50ms
                            continue;
                        }
                        bzero(head_buf, sizeof(SMLK_UINT8) * 8);
                        len = ss.receiveBytes(head_buf, 8, flags);

                        if (len == 8)
                        {
                            AvmHead avm_head;
                            bzero(&avm_head, 8);
                            memcpy(&avm_head, head_buf, 8);
                            avm_head.total_cmd_len = be16toh(avm_head.total_cmd_len);
                            avm_head.data_len = be16toh(avm_head.data_len);
                            SMLK_LOGI("recv avm head device_type = %02x total_cmd_len = 0x%04x cmd_group_id = %02x  \
                                                                            cmd_id = %02x  data_len = 0x%04x",
                                      avm_head.device_type, avm_head.total_cmd_len,
                                      avm_head.cmd_group_id, avm_head.cmd_id,
                                      avm_head.data_len);
                            // TODO go ahead get buffer
                            if (avm_head.cmd_group_id == 0x00 && avm_head.cmd_id == 0x01) // heartbeat
                            {
                                if (avm_head.data_len == 0) // correct heartbeat
                                {
                                    _heartbeat();
                                    continue;
                                }
                            }
                            SMLK_UINT8 data_recv_[256];
                            bzero(data_recv_, sizeof(SMLK_UINT8) * 256);
                            len = ss.receiveBytes(data_recv_, avm_head.data_len, flags);
                            if (len == avm_head.data_len) // receive avm response success
                            {
                                AvmMessageBuilder builder_recv_;
                                builder_recv_.DevicetoTbox().total_len(avm_head.total_cmd_len, 1).cmd_group(avm_head.cmd_group_id).cmd_id(avm_head.cmd_id).cmd_len(avm_head.data_len, 1).cmd_data(data_recv_, avm_head.data_len, 1); // set flag 1 2 tbox parse child data
                                AvmMessage msg_ = builder_recv_.end();
                                DeviceType d_type;
                                _onmsg(msg_);
                            }
                        }
                        else
                            continue;
                    }
                    catch (const Poco::TimeoutException &e)
                    {
                        SMLK_LOGW("TcpServer receiveBytes error --> %s   continue ...", e.what());
                        usleep(1000 * 300); // 300ms
                        continue;
                    }
                    catch (const std::exception &e)
                    {
                        SMLK_LOGE("TcpServer recevied error ---> %s", e.what());
                    }
                }
            }

        private:
            AVMService *_parent;
            NotifyDelegate _heartbeat;
            AvmEventListener _onmsg;
            StreamSocketSet _setsocket;
        };

        template <class S>
        class TCPConnectionFactoryImpl : public TCPServerConnectionFactory
        {
        public:
            TCPConnectionFactoryImpl(AVMService *p,
                                     NotifyDelegate heartbeat,
                                     AvmEventListener listener,
                                     StreamSocketSet socketset)
                : _parent(p), _heartbeat(heartbeat), _onmsg(listener), _setsocket(socketset)
            {
            }

            ~TCPConnectionFactoryImpl()
            {
            }

            TCPServerConnection *createConnection(const StreamSocket &socket)
            {
                return new S(socket, _parent, _heartbeat, _onmsg, _setsocket);
            }

        private:
            AVMService *_parent;
            NotifyDelegate _heartbeat;
            AvmEventListener _onmsg;
            StreamSocketSet _setsocket;
        };

        AVMService::AVMService(std::string host, SMLK_UINT16 nPort)
            : m_port(nPort), m_host(host), m_activity(this, &AVMService::runActivity)
        {
            SMLK_LOGD("TcpServer ...m_port --> %d", m_port);
        }

        AVMService::~AVMService()
        {
        }

        std::size_t AVMService::PostMsg(IN AvmMessage &msg)
        {
            std::size_t send_len = 0;
            try
            {
                while (true)
                {
                    int sent = 0;
                    sent = m_socket.sendBytes(msg.data(), msg.size());
                    if (sent < 0)
                    {
                        SMLK_LOGW("TcpServer sendBytes sent failed ...");
                        break;
                    }

                    send_len += sent;

                    if (send_len >= msg.size())
                    {
                        SMLK_LOGD("TcpServer sendBytes success ... size %d", msg.size());
                        break;
                    }
                }
            }
            catch (const std::exception &e)
            {
                SMLK_LOGE("TcpClient sendBytes error ---> %s", e.what());
            }
            return send_len;
        }

        void AVMService::Start()
        {
            m_activity.start();
            m_isRunning = true;
            m_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 3000 /*3 s*/>>(
                [this](SMLK_UINT32 index)
                {
                    SMLK_LOGD("AVMService Heartbeat ontimer restart tcp server....");
                    (void)index;
                    Stop();
                    m_activity.start();
                    m_isRunning = true;
                });
            m_timer->Start();
            m_timer->Reset();
            SMLK_LOGD("AVMService activity started!");
        }

        void AVMService::Stop()
        {
            m_isRunning = false;
            m_activity.stop();
            m_activity.wait();
            SMLK_LOGD("AVMService activity ended!");
        }

        bool AVMService::SendMsg(AvmMessage &msg, bool no_resp)
        {
            m_send_buf.put(msg);
            return true;
        }

        void AVMService::runActivity()
        {
            try
            {
                Poco::Net::SocketAddress sa(m_host.c_str(), m_port);
                ServerSocket svs(sa);

                auto heart_ = std::bind(&AVMService::ReplyHeartRsp, this);
                auto onmsg_ = std::bind(&AVMService::SendMsg2Tbox, this, std::placeholders::_1);
                auto socketset_ = std::bind(&AVMService::SetSocket, this, std::placeholders::_1);

                TCPServer srv(new TCPConnectionFactoryImpl<EchoConnection>(this,
                                                                           heart_, onmsg_, socketset_),
                              svs);
                srv.start();
                SMLK_LOGD("TCPServer initialize success ...");
                // ReplyHeartRsp();
                while (!m_activity.isStopped())
                {
                    try
                    {
                        bool timeout_flag = false;
                        AvmMessage queue_msg = m_send_buf.get(200, &timeout_flag);
                        if (timeout_flag)
                        {
                            usleep(100000 * 2); // 200ms
                            continue;
                        }
                        {
                            std::lock_guard<std::mutex> __lk(m_send_mtx);
                            PostMsg(queue_msg);
                        }
                    }
                    catch (std::exception &e)
                    {
                        SMLK_LOGE("some unexcept err --> %s", e.what());
                    }
                    usleep(100000); // 100ms
                }
                srv.stop();
            }
            catch (std::exception &e)
            {
                SMLK_LOGE("runActivity error --> %s", e.what());
            }
        }

        void AVMService::ReplyHeartRsp()
        {
            m_timer->Reset();
            AvmMessageBuilder builder_;
            builder_.TboxtoDevice().total_len(0x05).cmd_group(0x00).cmd_id(0x81).cmd_len(0x01).cmd_data(0x00, 1);
            AvmMessage msg_ = builder_.end();
            {
                std::lock_guard<std::mutex> __lk(m_send_mtx);
                PostMsg(msg_);
            }
            m_timer->ReStart();
        }

        void AVMService::SendMsg2Tbox(AvmMessage &msg)
        {
            if (m_listener)
            {
                m_listener(msg);
            }
        }
    }
}
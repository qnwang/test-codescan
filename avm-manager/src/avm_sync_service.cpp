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
#include "avm_sync_service.h"
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
        using Poco::Net::TCPServerParams;

        class SMLK_EchoConnection : public TCPServerConnection
        {
        public:
            SMLK_EchoConnection(const StreamSocket &s, AVMSyncService *p,
                                NotifyDelegate notify,
                                NotifyDelegate heartbeat,
                                AvmEventListener listener,
                                StreamSocketSet socketset)
                : TCPServerConnection(s), _parent(p), _notifyone(notify), _heartbeat(heartbeat), _onmsg(listener), _setsocket(socketset)
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
                SMLK_LOGD("Tcp connection start running.");
                while (_parent->IsRunning())
                {
                    try
                    {
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
                            if (avm_head.head != 0xFE)
                            {
                                SMLK_LOGD("Recv err head flag!");
                                continue;
                            }
                            avm_head.total_cmd_len = be16toh(avm_head.total_cmd_len);
                            avm_head.data_len = be16toh(avm_head.data_len);
                            // TODO go ahead get buffer
                            if (avm_head.cmd_group_id == 0x00 && avm_head.cmd_id == 0x01) // heartbeat
                            {
                                if (avm_head.data_len == 0) // correct heartbeat
                                {
                                    len = ss.receiveBytes(head_buf, 1, flags); // get end 'fe'
                                    _heartbeat();                              // call NotifyHeartBeatResp function
                                    continue;
                                }
                            }
                            SMLK_UINT8 data_recv_[256];
                            bzero(data_recv_, sizeof(SMLK_UINT8) * 256);
                            len = ss.receiveBytes(data_recv_, avm_head.data_len + 1, flags);
                            if (len == avm_head.data_len + 1) // receive avm response success
                            {
                                AvmMessageBuilder builder_recv_;
                                builder_recv_.DevicetoTbox().total_len(avm_head.total_cmd_len, 1).cmd_group(avm_head.cmd_group_id).cmd_id(avm_head.cmd_id).cmd_len(avm_head.data_len, 1).cmd_data(data_recv_, avm_head.data_len, 1); // set flag 1 2 tbox parse child data
                                AvmMessage msg_ = builder_recv_.end();
                                SMLK_UINT8 *temp_data = msg_.data();
                                std::string str_device_type = "Dms";
                                std::string device_msg = tools::uint8_2_string((SMLK_UINT8 *)temp_data, msg_.size());
                                std::string str_group_id = tools::uint8_2_string((SMLK_UINT8 *)&msg_.getAvmHead().cmd_group_id, sizeof(SMLK_UINT8));
                                std::string str_cmd_id = tools::uint8_2_string((SMLK_UINT8 *)&msg_.getAvmHead().cmd_id, sizeof(SMLK_UINT8));
                                std::string str_show_msg = "[" + str_device_type + "2Tbox][Group:0x" + str_group_id + "][Cmd:0x" + str_cmd_id + "]";
                                tools::print_long_string(device_msg, str_show_msg);
                                _onmsg(msg_);
                                if (!((0x01 == msg_.getAvmHead().cmd_group_id) && (0x01 == msg_.getAvmHead().cmd_id))) /*非主动上报通知响应*/
                                {
                                    _notifyone();
                                }
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
                        SMLK_LOGE("TcpServer received error ---> %s", e.what());
                    }
                }
                SMLK_LOGD("avm SMLK_EchoConnection connected leave ...");
            }

        private:
            AVMSyncService *_parent;
            NotifyDelegate _notifyone;
            NotifyDelegate _heartbeat;
            AvmEventListener _onmsg;
            StreamSocketSet _setsocket;
        };

        template <class S>
        class SMLK_TCPConnectionFactoryImpl : public TCPServerConnectionFactory
        {
        public:
            SMLK_TCPConnectionFactoryImpl(AVMSyncService *p,
                                          NotifyDelegate notify,
                                          NotifyDelegate heartbeat,
                                          AvmEventListener listener,
                                          StreamSocketSet socketset)
                : _parent(p), _notifyone(notify), _heartbeat(heartbeat), _onmsg(listener), _setsocket(socketset)
            {
            }

            ~SMLK_TCPConnectionFactoryImpl()
            {
            }

            TCPServerConnection *createConnection(const StreamSocket &socket)
            {
                SMLK_LOGD("Tcpserver connection created-->port:%d", socket.address().port());
                return new S(socket, _parent, _notifyone, _heartbeat, _onmsg, _setsocket);
            }

        private:
            AVMSyncService *_parent;
            NotifyDelegate _notifyone;
            NotifyDelegate _heartbeat;
            AvmEventListener _onmsg;
            StreamSocketSet _setsocket;
        };

        AVMSyncService::AVMSyncService(std::string host, SMLK_UINT16 nPort)
            : m_port(nPort), m_host(host), m_activity(this, &AVMSyncService::runActivity), m_sp_evt(new Poco::Event), m_sp_evt_heart(new Poco::Event)
        {
            switch (nPort)
            {
            case AVMPORT:
                SMLK_LOGD("Avm server created-->port:%d", AVMPORT);
                m_device_type = DeviceType::DEV_AVM;
            case DMSPORT:
                SMLK_LOGD("Dms server created-->port:%d", DMSPORT);
                m_device_type = DeviceType::DEV_DMS;
            }
        }

        AVMSyncService::~AVMSyncService()
        {
            if (m_timer.get())
                m_timer->Stop();
        }

        void AVMSyncService::NotifyOne()
        {
            // SMLK_LOGD("Set evt");
            m_sp_evt->set();
        }

        void AVMSyncService::NotifyHeartBeatResp()
        {
            m_sp_evt_heart->set();
        }

        void AVMSyncService::SendMsg2Tbox(AvmMessage &msg)
        {
            m_recv_msg = msg;
            if (m_recv_msg.getAvmHead().cmd_group_id == 0x01 &&
                m_recv_msg.getAvmHead().cmd_id == 0x01) // event active report
            {
                if (m_listener)
                    m_listener(m_recv_msg);
            }
        }

        bool AVMSyncService::SendMsg(AvmMessage &msg, bool no_resp)
        {
            if (!IsRunning())
            {
                Start();
                return false;
            }
            std::lock_guard<std::mutex> __lk(m_send_mtx);
            std::size_t send_len = 0;
            DeviceType d_type = this->getDeviceType();
            std::string str_device_type = (DeviceType::DEV_DMS == d_type) ? "Dms" : (DeviceType::DEV_AVM == d_type) ? "Avm"
                                                                                                                    : "Unknown";
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
                        break;
                    }
                }
            }
            catch (const std::exception &e)
            {
                SMLK_LOGE("TcpClient sendBytes error ---> %s", e.what());
                return false;
            }

            if (no_resp)
            {
                return true;
            }
            std::string avm_log = tools::uint8_2_string(msg.data(), msg.size());
            std::string str_group_id = tools::uint8_2_string((SMLK_UINT8 *)&msg.getAvmHead().cmd_group_id, sizeof(SMLK_UINT8));
            std::string str_cmd_id = tools::uint8_2_string((SMLK_UINT8 *)&msg.getAvmHead().cmd_id, sizeof(SMLK_UINT8));
            std::string str_show_msg = "[Tbox2" + str_device_type + "][Group:0x" + str_group_id + "][Cmd:0x" + str_cmd_id + "]";
            tools::print_long_string(avm_log, str_show_msg);

            if (m_sp_evt->tryWait(3000))
            {
                // SMLK_LOGD("[SendGroupId]=0x%02x [RecvGroupId]=0x%02x ", msg.getAvmHead().cmd_group_id, m_recv_msg.getAvmHead().cmd_group_id);
                // SMLK_LOGD("[SendCmdId]=0x%02x [RecvCmdId]=0x%02x ", msg.getAvmHead().cmd_id, m_recv_msg.getAvmHead().cmd_id);
                if (!((m_recv_msg.getAvmHead().cmd_group_id == msg.getAvmHead().cmd_group_id) &&
                      (m_recv_msg.getAvmHead().cmd_id == (msg.getAvmHead().cmd_id + 0x80))))
                {
                    SMLK_LOGD("Unmatched resp notify ...");
                    return false;
                }
                memcpy(&m_recv_msg.getTspHead(), &msg.getTspHead(), sizeof(TspHead));
                msg = m_recv_msg;
                return true;
            }
            return false;
        }

        void AVMSyncService::Init()
        {
            m_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 1000 * 5 /*5 s*/>>(
                [this](SMLK_UINT32 index)
                {
                    SMLK_LOGD("AVMSyncService Heartbeat ontimer restart tcp server....");
                    (void)index;
                    Stop();
                    Start();
                });
            m_timer->StartWithPause();
        }

        void AVMSyncService::Start()
        {
            m_activity.start();
        }

        void AVMSyncService::Stop()
        {
            m_isRunning = false;
            try
            {
                m_socket.shutdown();
            }
            catch (const std::exception &e)
            {
                SMLK_LOGE("TcpClient m_socket shutdown error ---> %s", e.what());
            }
            m_activity.stop();
            NotifyHeartBeatResp(); // heart beat evt release
            m_activity.wait();
            m_timer->Reset();
            SMLK_LOGD("AVMSyncService activity ended!");
        }

        void AVMSyncService::runActivity()
        {
            try
            {
                Poco::Net::SocketAddress sa(m_host.c_str(), m_port);
                ServerSocket svs(sa);

                auto notify_ = std::bind(&AVMSyncService::NotifyOne, this);
                auto heart_ = std::bind(&AVMSyncService::NotifyHeartBeatResp, this);
                auto onmsg_ = std::bind(&AVMSyncService::SendMsg2Tbox, this, std::placeholders::_1);
                auto socketset_ = std::bind(&AVMSyncService::SetSocket, this, std::placeholders::_1);

                TCPServer srv(new SMLK_TCPConnectionFactoryImpl<SMLK_EchoConnection>(this, notify_, heart_, onmsg_, socketset_), svs);
                srv.start();
                SMLK_LOGD("TCPServer initialize success ...");
                m_isRunning = true;
                SMLK_LOGD("AVMSyncService activity started!");
                while (!m_activity.isStopped())
                {
                    m_sp_evt_heart->wait();
                    if (!m_activity.isStopped())
                    {
                        ReplyHeartRsp();
                    }
                }
                srv.stop();
            }
            catch (std::exception &e)
            {
                SMLK_LOGE("runActivity error --> %s", e.what());
                m_isRunning = false;
            }
        }

        void AVMSyncService::ReplyHeartRsp()
        {
            m_timer->Reset();
            AvmMessageBuilder builder_;
            SMLK_UINT8 res = 0;
            builder_.TboxtoDevice().total_len(0x0005, 0).cmd_group(0x00).cmd_id(0x81).cmd_len(0x0001, 0).cmd_data(&res, 1);
            AvmMessage msg_ = builder_.end();
            {
                SendMsg(msg_, true);
            }
            m_timer->ReStart();
        }
    }
}
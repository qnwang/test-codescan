/**
 * @file TCPClient.h
 * @author T (taoguanjie@smartlink.com.cn)
 * @brief 
 * @version 0.1
 * @date 2022-04-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef SOCKET_TCPINTERFACE_H_
#define SOCKET_TCPINTERFACE_H_

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <functional>
#include <mutex>
#include <string>
#include "ISocket.h"

namespace smartlink
{
    class TCPClient : public ISocket
    {
        typedef std::function<void(std::string &s, int i)> cbCastSocketState;

    public:
        TCPClient(std::string &taskName);
        ~TCPClient();

        virtual void initParam(char *ip_r, int port) override;
        virtual int connect() override;
        virtual int close() override;
        virtual int recMsg(char *buf, int len) override;
        virtual int sendMsg(char *buf, int len) override;

        void RegisterSocketStateCB(cbCastSocketState cb);
        void bordCastConnState();

        connectState m_connectState;

    private:
        int listen(char *ip_r, int port);
        int accept();
        bool getTcpInfoState();

        int fd;
        fd_set fds;
        std::string m_taskName;
        std::mutex m_stateMtx;
        cbCastSocketState m_callBack;
        struct sockaddr_in remoteAddr;
        struct timeval timeout = {0, 0};
    };
}
#endif /* SOCKET_TCPINTERFACE_H_ */

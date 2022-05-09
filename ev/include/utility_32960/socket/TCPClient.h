//
// Created by work on 2021/7/28.
//

#ifndef SOCKET_TCPINTERFACE_H_
#define SOCKET_TCPINTERFACE_H_

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <string>
#include <mutex>

#include "ISocket.h"

namespace smartlink {

    /**
     * @brief TCP/IP客户端通用类
     */

    class TCPClient : public ISocket {
        typedef std::function<void(int i)> cbCastSocketState;

    public:
        TCPClient();

        ~TCPClient();

        virtual void initParam(const char *ip_r, const int port) override;

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
        std::mutex m_stateMtx;
        cbCastSocketState m_callBack;
        struct sockaddr_in remoteAddr;
        struct timeval timeout = {0, 0};
    };
}
#endif /* SOCKET_TCPINTERFACE_H_ */

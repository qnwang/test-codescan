//
// Created by work on 2021/7/28.
//

#ifndef SOCKET_UDPINTERFACE_H_
#define SOCKET_UDPINTERFACE_H_

#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ISocket.h"

namespace smartlink {


    /**
     * @brief UDP客户端通用类
     */
    class UDPClient : public ISocket {
    public:
        UDPClient();

        virtual ~UDPClient();

        virtual void initParam(const char *ip_r, const int port) override;

        virtual int connect() override;

        virtual int close() override;

        virtual int recMsg(char *buf, int len_r) override;

        virtual int sendMsg(char *buf, int len_r) override;

        struct sockaddr_in remoteAddr;
        // connectState m_connectState;
        int fd;
    };
}
#endif /* SOCKET_UDPINTERFACE_H_ */

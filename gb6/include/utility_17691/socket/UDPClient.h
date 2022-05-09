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
    class UDPClient : public ISocket {
    public:
        UDPClient();
        ~UDPClient();
        virtual void initParam(char *ip_r, int port) override;
        virtual int connect() override;
        virtual int close() override;
        virtual int recMsg(char *buf, int len_r) override;
        virtual int sendMsg(char *buf, int len_r) override;

    private:
        int                 fd;
        struct sockaddr_in  remoteAddr;
    };
}
#endif /* SOCKET_UDPINTERFACE_H_ */

//
// Created by work on 2021/7/28.
//

#ifndef SMARTLINKT_TBOX_ISOCKET_H
#define SMARTLINKT_TBOX_ISOCKET_H

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <functional>

namespace smartlink {

    typedef enum {
        notInit = 0,
        normal = 1,
        disconnect = -1
    } connectState;

    class ISocket {
    public:
        ISocket() = default;

        virtual ~ISocket() = default;

        virtual void initParam(const char *ip_r, const int port) = 0;

        virtual int connect() = 0;

        virtual int close() = 0;

        virtual int recMsg(char *buf, int len) = 0;

        virtual int sendMsg(char *buf, int len) = 0;
    };
}
#endif //SMARTLINKT_TBOX_ISOCKET_H

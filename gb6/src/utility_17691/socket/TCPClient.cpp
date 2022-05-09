/**
 * @file TCPClient.cpp
 * @author T (taoguanjie@smartlink.com.cn)
 * @brief 
 * @version 0.1
 * @date 2022-04-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <smlk_log.h>

#include "TCPClient.h"

using namespace smartlink;

TCPClient::TCPClient(std::string &taskName)
{
    fd = 0;
    m_taskName = taskName;
    m_connectState = notInit;
    bzero(&remoteAddr, sizeof(remoteAddr));
    remoteAddr.sin_family = AF_INET;
    bordCastConnState();
    SMLK_LOGD("[%s] TCPClient::TCPClient() socketClient init ", m_taskName.c_str());
}

TCPClient::~TCPClient()
{
    close();
}

void TCPClient::initParam(char *ip_r, int port)
{
    remoteAddr.sin_port = htons(port);
    inet_aton(ip_r, &remoteAddr.sin_addr);
    SMLK_LOGD("[%s] TCPClient::initParam() ip:%s,port:%d ", m_taskName.c_str(), ip_r, port);
}

int TCPClient::connect() {
    SMLK_LOGD("[%s] TCPClient::connect()", m_taskName.c_str());

    int wait_seconds = 2;

    int ret = -1;
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd != -1) {
        int flags = fcntl(fd, F_GETFL, 0);
        // fcntl(fd, F_SETFL, flags & ~O_NONBLOCK); //设置成阻塞模式；
        fcntl(fd, F_SETFL, flags | O_NONBLOCK); //设置成非阻塞模式；
        int on = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (void *)&on, (socklen_t)sizeof(on));

        //keepalive
        int keepAlive = 1;    // 开启keepalive属性
        int keepIdle = 30;    // 如该连接在60秒内没有任何数据往来,则进行探测
        int keepInterval = 5; // 探测时发包的时间间隔为5 秒
        int keepCount = 3;    // 探测尝试的次数.如果第1次探测包就收到响应了,则后2次的不再发.
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepAlive, sizeof(keepAlive));
        setsockopt(fd, SOL_TCP, TCP_KEEPIDLE, (void *)&keepIdle, sizeof(keepIdle));
        setsockopt(fd, SOL_TCP, TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval));
        setsockopt(fd, SOL_TCP, TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount));

        char *str_n = inet_ntoa(remoteAddr.sin_addr);
        SMLK_LOGI("[%s] TCPClient::connect() socketClient ip:port %s : %d", m_taskName.c_str(), str_n, ntohs(remoteAddr.sin_port));

        ret = ::connect(fd, (struct sockaddr *)&remoteAddr, sizeof(remoteAddr));

        if (ret < 0 && errno == EINPROGRESS) {
            fd_set connect_fdset;
            struct timeval timeout;
            FD_ZERO(&connect_fdset);
            FD_SET(fd, &connect_fdset);
            timeout.tv_sec = wait_seconds;
            timeout.tv_usec = 0;
            do {
                // 一但连接建立，则套接字就可写  所以connect_fdset放在了写集合中
                ret = select(fd + 1, NULL, &connect_fdset, NULL, &timeout);
                // 超时返回0
                // 失败返回-1
                // 成功返回大于0的整数，这个整数表示就绪描述符的数目
            } while (ret < 0 && errno == EINTR);

            if (ret == 0) {
                close();
                ret = -1;
                errno = ETIMEDOUT;
                SMLK_LOGD("[%s] TCPClient::connect()socketClient connect fail, strerror: %s, errorno %d", m_taskName.c_str(), strerror(errno), errno);
                return ret;

                // ret = -1;
                // errno = ETIMEDOUT;
            } else if (ret < 0) {
                close();
                ret = -1;
                SMLK_LOGD("[%s] TCPClient::connect()socketClient connect fail", m_taskName.c_str());
                return ret;

                // return -1;
            } else if (ret == 1) {
                /* ret返回为1（表示套接字可写），可能有两种情况，一种是连接建立成功，一种是套接字产生错误，*/
                /* 此时错误信息不会保存至errno变量中，因此，需要调用getsockopt来获取。 */
                int err;
                socklen_t socklen = sizeof(err);
                int sockoptret = getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &socklen);
                if (sockoptret == -1) {
                    close();
                    ret = -1;
                    SMLK_LOGD("[%s] TCPClient::connect()socketClient connect fail, strerror: %s, errorno %d", m_taskName.c_str(), strerror(errno), errno);
                    return ret;
                }

                if (err == 0) {
                    ret = 0;
                    m_connectState = normal;
                    SMLK_LOGD("[%s] TCPClient::() socketClient connect success", m_taskName.c_str());
                } else {
                    close();
                    ret = -1;
                    errno = err;
                    SMLK_LOGD("[%s] TCPClient::connect()socketClient connect fail, strerror: %s, errorno %d", m_taskName.c_str(), strerror(errno), errno);
                    return ret;
                }
            }
        } else if (ret == 0) {
            m_connectState = normal;
            SMLK_LOGD("[%s] TCPClient::() socketClient connect success", m_taskName.c_str());
        }
        this->bordCastConnState();
    }
    return ret;
}

int TCPClient::recMsg(char *buf, int len_r) {
    int res = 0;
    if (fd != -1) {
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        timeout.tv_sec = 60;
        timeout.tv_usec = 0;
        switch (select(fd + 1, &fds, NULL, NULL, &timeout))//overtime
        {
            case -1:
                SMLK_LOGD("[%s] TCPClient::() SELECT FD ERROR", m_taskName.c_str());
                res = -1;
                close();
                break;
            case 0:
                res = 0;
                break;
            default:
                if (FD_ISSET(fd, &fds)) {
                    res = read(fd, buf, len_r);
                    if (res > 0) {
                        SMLK_LOGD("[%s] TCPClient::() socketClient receive success,receive length %d", m_taskName.c_str(),res);
                    } else if (res == 0) {
                        SMLK_LOGD("[%s] TCPClient::() socketClient receive error, strerror: %s\n, errorno %d , res %d ", m_taskName.c_str(), strerror(errno), errno, res);
                        close();
                    } else if (res < 0) {
                        if ((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == EINTR)) {
                            //SMLK_LOGD("[%s] TCPClient::() res < 00", m_taskName.c_str());
                            //SMLK_LOGD("[%s] TCPClient::() socketClient receive error, strerror: %s\n, errorno %d , res %d ", m_taskName.c_str(), strerror(errno), errno, res);
                        } else {
                            //SMLK_LOGD("[%s] TCPClient::() res < 01", m_taskName.c_str());
                            //SMLK_LOGD("[%s] TCPClient::() socketClient receive error, strerror: %s\n, errorno %d , res %d ", m_taskName.c_str(), strerror(errno), errno, res);
                            close();
                        }
                    }
                } else {
                    //SMLK_LOGD("[%s] TCPClient::() FD_ISSET(fd, &fds) not exist", m_taskName.c_str());
                }
                break;
        }
    }
    return res;
}

int TCPClient::accept() {
    int len = sizeof(struct sockaddr);
    int cfd = ::accept(fd, (struct sockaddr *) (&remoteAddr),
                       (unsigned int *) &len);
    if (0 > cfd) {
        SMLK_LOGE("[%s] accept fail !%d\r\n", m_taskName.c_str(), cfd);
        return -1;
    } else
        return cfd;

}

void TCPClient::RegisterSocketStateCB(cbCastSocketState cb) {
    m_callBack = cb;
}

void TCPClient::bordCastConnState() {
    if (m_callBack) {
        m_callBack(m_taskName, (int) m_connectState);
    }
}

bool TCPClient::getTcpInfoState() {
    std::lock_guard <std::mutex> lk(m_stateMtx);
    int ret = 0;
    if (fd != -1) {
        struct tcp_info info;
        int len = sizeof(info);
        info.tcpi_state = -1;
        ret = getsockopt(fd, IPPROTO_TCP, TCP_INFO, &info, (socklen_t * ) & len);
        if (ret == -1)
            SMLK_LOGD("[%s] TCPClient::()  getTcpInfoState fault, strerror: %s\n, errorno %d ", m_taskName.c_str(), strerror(errno), errno);
        if (info.tcpi_state == TCP_ESTABLISHED)
            return true;
        else {
            SMLK_LOGD("[%s] TCPClient::() info.tcpi_state == %d\n", m_taskName.c_str(), info.tcpi_state);
            SMLK_LOGD("[%s] TCPClient::() getTcpInfoState, strerror: %s\n, errorno %d ", m_taskName.c_str(), strerror(errno), errno);
            return false;
        }
    } else {
        return false;
    }
}

int TCPClient::sendMsg(char *buf, int len) {
    int res = 0;
    if ((fd != -1)) {
        if (getTcpInfoState() == true) {
            res = send(fd, buf, len, 0);
            if (res == 0) {
                SMLK_LOGD("[%s] TCPClient::() socketClient sendMsg,strerrno %s, error%d , res %d , fd %d", m_taskName.c_str(), strerror(errno), errno, res, fd);
            } else if (res < 0) {
                SMLK_LOGD("[%s] TCPClient::() socketClient sendMsg,strerrno %s, error%d , res %d , fd %d", m_taskName.c_str(), strerror(errno), errno, res, fd);
            }
        } else {
            SMLK_LOGD("[%s] TCPClient::() socketClient send error, strerror: %s\n, errorno %d , res %d ", m_taskName.c_str(), strerror(errno), errno, res);
            res = -1;
            close();
        }
    }
    return res;
}

int TCPClient::close() {
    if (fd != -1) {
        SMLK_LOGD("[%s] TCPClient::() socketClient close", m_taskName.c_str());
        m_connectState = disconnect;
        bordCastConnState();
        ::close(fd);
        fd = -1;
    }
    return 0;
}

int TCPClient::listen(char *ip_r, int port) {
    this->fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in localAddr;
    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip_r, &localAddr.sin_addr);

    int nOptval = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const void *)&nOptval, sizeof(int));

    int res = bind(fd, (struct sockaddr *)(&localAddr), sizeof(struct sockaddr));
    if (0 > res) {
        SMLK_LOGD("[%s] bind fail !:%d ", m_taskName.c_str(), res);
        return -1;
    }
    // SMLK_LOGD("[%s] bind ok !", m_taskName.c_str());

    res = ::listen(fd, 5);
    if (0 > res) {
        SMLK_LOGD("[%s] listen fail !%d ", m_taskName.c_str(), res);
        return -1;
    }
    // SMLK_LOGD("[%s] listen ok ", m_taskName.c_str());
    return 0;
}

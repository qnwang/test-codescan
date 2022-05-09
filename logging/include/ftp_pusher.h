/*****************************************************************************/
/**
* \file       ftp_pusher.h
* \author     wukai
* \date       2021/06/29
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
#ifndef SMLK_FTP_PUSHER_H_
#define SMLK_FTP_PUSHER_H_
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#include <iostream>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "smlk_types.h"
#include "smlk_log.h"
#include "logger_jt808_def.h"

#include <Poco/Net/FTPClientSession.h>

using namespace Poco::Net;

using namespace std;

namespace smartlink {
class FtpPusher : public FTPClientSession
{
public:
    explicit FtpPusher(const string host, SMLK_UINT16 FTP_PORT)
        : FTPClientSession(host, FTP_PORT) {};
    virtual ~FtpPusher(){};
};

enum FtpErrorCode :  SMLK_UINT32 {
    FTP_SUCCESS = 0,
    FTP_LOGIN_FAILED,
    FTP_TIMEOUT,
    FTP_ABORT
};

class FtpInfo {
public:
    string host;
    SMLK_UINT16 port;
    string user;
    string passwd;
    string server_path;

private:
    FtpInfo& operator = (IN FtpInfo& other) {
        this->host = other.host;
        this->port = other.port;
        this->user = other.user;
        this->passwd = other.passwd;
        this->server_path = other.server_path;
        return *this;
    }
};

class FtpPushWithNonBlock {
public:
    FtpPushWithNonBlock(IN FtpInfo& info);
    virtual ~FtpPushWithNonBlock();

    using FtpPusherListener = std::function<void(FtpErrorCode)>;

    FtpPushWithNonBlock* upLoadFile(IN string& filePath, IN string &ftpFileName, SMLK_UINT16 timeout, FtpPusherListener listener);
    FtpPushWithNonBlock* upLoadFile(IN std::vector<string>& files, IN std::vector<string>& ftpFiles, SMLK_UINT16 timeout, FtpPusherListener listener);

    void start();
    void close();
    void abort();
private:
    void pushProcess();
private:
    FtpInfo m_ftp_info;
    SMLK_UINT16 m_timeout;
    std::vector<string> m_files_list;
    std::vector<string> m_ftp_files_list;
    FtpPusherListener m_listener;
    shared_ptr<FtpPusher> m_sp_ftp_pusher;
    std::thread m_task_thread;
};
}


#endif // SMLK_FTP_PUSHER_H_
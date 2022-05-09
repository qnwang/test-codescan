/*****************************************************************************/
/**
* \file       ftp_pusher.cpp
* \author     wukai
* \date       2021/06/29
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#include "ftp_pusher.h"
#include <Poco/File.h>
#include <Poco/Exception.h>
#include "Poco/Timespan.h"
#include <Poco/Net/NetException.h>

#include <fstream>


using namespace smartlink;
using namespace Poco;
using namespace Poco::Net;

FtpPushWithNonBlock::FtpPushWithNonBlock(IN FtpInfo& info)
    : m_ftp_info(info)
{
}

FtpPushWithNonBlock::~FtpPushWithNonBlock() {
    m_listener = NULL;
    if (m_task_thread.joinable())
        m_task_thread.join();
}

FtpPushWithNonBlock* FtpPushWithNonBlock::upLoadFile(IN string& filePath, IN string &ftpFileName,
                                                        SMLK_UINT16 timeout, FtpPusherListener listener)
{
    std::vector<string> vec_;
    vec_.push_back(filePath);

    std::vector<string> vecFtp;
    vecFtp.push_back(ftpFileName);


    return upLoadFile(vec_, vecFtp, timeout, listener);
}

FtpPushWithNonBlock* FtpPushWithNonBlock::upLoadFile(IN std::vector<string>& files, IN std::vector<string>& ftpFiles,
                                                        SMLK_UINT16 timeout, FtpPusherListener listener)
{
    m_files_list = files;
    m_ftp_files_list = ftpFiles;
    m_timeout = timeout;
    m_listener = listener;
    return this;
}

void FtpPushWithNonBlock::start()
{
    SMLK_LOGD("FtpPushWithNonBlock start ...");
    if (m_files_list.empty() && m_ftp_files_list.empty())
        throw Exception("[FtpPushWithNonBlock] File not exist!");
    if (m_listener == NULL)
        throw Exception("[FtpPushWithNonBlock] Can not call with none listener!");
    // TODO start push
    m_task_thread = std::thread(&FtpPushWithNonBlock::pushProcess, this);
}

void FtpPushWithNonBlock::pushProcess()
{
    int report_status = 0;
    try
    {
        m_sp_ftp_pusher = std::make_shared<FtpPusher>(m_ftp_info.host, m_ftp_info.port);
        // m_sp_ftp_pusher->setTimeout(Timespan(m_timeout));

        try
        {
            SMLK_LOGD("FtpPushWithNonBlock login user = %s   pass = %s...", m_ftp_info.user.c_str()
                                                                          , m_ftp_info.passwd.c_str());
            m_sp_ftp_pusher->login(m_ftp_info.user, m_ftp_info.passwd);
            while (!m_sp_ftp_pusher->isLoggedIn()) {
                usleep(100000);
            }
        }
        catch(const NetException& e)
        {
            SMLK_LOGW("[%s]  ftp login err [%s]...", __func__, e.what());
            report_status = 2;
            goto post_status;
        }

        // TODO upload
        try
        {
            m_sp_ftp_pusher->createDirectory(m_ftp_info.server_path);
        }
        catch(const NetException& e)
        {
            SMLK_LOGW("[%s]  ftp createDirectory [%s] err [%s]...", __func__, m_ftp_info.server_path.c_str(), e.what());
            // report_status = 4;
            // goto post_status;
            SMLK_LOGW("ftp dir may be exist ...");
        }

        m_sp_ftp_pusher->setWorkingDirectory(m_ftp_info.server_path);

        for (int i = 0; i < m_files_list.size(); i++)
        {
            auto logPath = m_files_list.at(i);
            auto ftpFile = m_ftp_files_list.at(i);
            std::ostream& ostr = m_sp_ftp_pusher->beginUpload(ftpFile.substr(ftpFile.rfind('/') + 1));
            fstream local_file(logPath.c_str());
            if (! local_file.is_open())
            {
                report_status = 4;
                goto post_status;
            }
            ostr << local_file.rdbuf();;
            m_sp_ftp_pusher->endUpload();
            local_file.close();
        }
    }
    catch(const std::exception& e)
    {
        SMLK_LOGW("FTP pusher catch err --> %s", e.what());
        report_status = 1;
    }

post_status:

    //ftp-File update finish, delete all active*.tar.gz.
    for (auto file_path_ : m_files_list)
    {
        if ((file_path_.rfind("Active") != std::string::npos))
        {
            std::remove(file_path_.c_str());
        }
    }

    //TODO post
    if (m_listener) {
        m_listener((FtpErrorCode)report_status);
    }
    close();
    SMLK_LOGD("FTP pusher upload success ...");
}

void FtpPushWithNonBlock::close()
{
    SMLK_LOGD("FtpPushWithNonBlock close start ...");
    try
    {
        if (m_sp_ftp_pusher.get())
        {
            m_sp_ftp_pusher->abort();
            m_sp_ftp_pusher->close();
            m_sp_ftp_pusher.reset();
        }
    }
    catch(const std::exception& e)
    {
        SMLK_LOGW("FtpPushWithNonBlock close err -> %s", e.what());
    }

    SMLK_LOGD("FtpPushWithNonBlock close end ...");
}

void FtpPushWithNonBlock::abort()
{
    try
    {
        if (m_sp_ftp_pusher.get())
        {
            m_sp_ftp_pusher->abort();
        }
    }
    catch(const std::exception& e)
    {
        SMLK_LOGW("FtpPushWithNonBlock abort err -> %s", e.what());
    }
}
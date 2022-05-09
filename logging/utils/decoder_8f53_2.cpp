/*****************************************************************************/
/**
* \file       decoder_8f53_2.cpp
* \author     wukai
* \date       2021/07/12
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/

#include "decoder_8f53_2.h"
#include "smlk_log.h"
#include "smlk_tools.h"

using namespace smartlink;


void Decoder8F53_2::Decode(IN SMLK_UINT8* data, IN std::size_t len)
{
    file_lists.clear();
    int position_ = 0;

    SMLK_UINT8 len_temp = data[position_];
    position_ += 1;

    SMLK_UINT8 host_temp[len_temp + 1];
    memcpy(host_temp, data + position_, len_temp);
    host_temp[len_temp] = '\0';
    host_ = reinterpret_cast<const char*>(host_temp);
    position_ += len_temp;
    SMLK_LOGD("LoggerClient len_temp = %d", len_temp);
    SMLK_LOGD("LoggerClient host_ = %s", host_.c_str());

    memcpy(&port_, data + position_, 2);
    port_ = be16toh(port_);
    position_ += 2;
    SMLK_LOGD("LoggerClient port = %d", (SMLK_UINT32)port_);

    len_temp = data[position_];
    SMLK_UINT8 usr_temp[len_temp + 1];
    position_ += 1;
    memcpy(usr_temp, data + position_, len_temp);
    usr_temp[len_temp] = '\0';
    usrname_ = reinterpret_cast<const char*>(usr_temp);
    position_ += len_temp;
    SMLK_LOGD("LoggerClient len_temp = %d", len_temp);
    SMLK_LOGD("LoggerClient usr_ = %s", usrname_.c_str());

    len_temp = data[position_];
    SMLK_UINT8 passwd_temp[len_temp + 1];
    position_ += 1;
    memcpy(passwd_temp, data + position_, len_temp);
    passwd_temp[len_temp] = '\0';
    passwd_ = reinterpret_cast<const char*>(passwd_temp);
    position_ += len_temp;
    SMLK_LOGD("LoggerClient len_temp = %d", len_temp);
    SMLK_LOGD("LoggerClient passwd_ = %s", passwd_.c_str());


    len_temp = data[position_];
    SMLK_UINT8 server_path_temp[len_temp + 1];
    position_ += 1;
    memcpy(server_path_temp, data + position_, len_temp);
    server_path_temp[len_temp] = '\0';
    server_path_ = reinterpret_cast<const char*>(server_path_temp);
    position_ += len_temp;
    SMLK_LOGD("LoggerClient len_temp = %d", len_temp);
    SMLK_LOGD("LoggerClient server_path_ = %s", server_path_.c_str());

    SMLK_LOGD("LoggerClient len_end = %d", len - position_);

    if (len - position_ < 1)
    {
        return;
    }

    // string file_name_;
    do
    {
        SMLK_UINT8 file_name_temp[LOG_SERVER_FILE_NAME_LENGTH];
        bzero(&file_name_temp, sizeof(SMLK_UINT8) * LOG_SERVER_FILE_NAME_LENGTH);
        memcpy(file_name_temp, data + position_, LOG_SERVER_FILE_NAME_LENGTH);
        position_ += LOG_SERVER_FILE_NAME_LENGTH;
        std::string temp_str = reinterpret_cast<const char*>(file_name_temp);
        file_lists.insert(file_lists.end(),temp_str);
    } while (len - position_ > 0);

    for (auto nm_ : file_lists)
    {
        SMLK_LOGD("LoggerClient file_name_sub = %s", nm_.c_str());
    }
}
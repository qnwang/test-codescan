/*****************************************************************************/
/**
 * \file       avm_custom_logic.cpp
 * \author     wukai
 * \date       2021/08/03
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include "avm_custom_logic.h"
#include <sys/time.h>

#define CheckPtrVaild            \
    if (!m_sp_avm_service.get()) \
        return false;

namespace smartlink
{
    namespace AVM
    {
        SMLK_BOOL AvmCustomUtil::SyncAll(IN std::string &imsi)
        {
            m_imsi = imsi;
            m_task.put(SYNC_AVM_VERSION);
            m_task.put(SYNC_AVM_TIME);
            m_task.put(SYNC_AVM_TERMINAL_ID);
            return true;
        }

        SMLK_BOOL AvmCustomUtil::GetAvmVersion(std::string &version, SMLK_BOOL block)
        {
            if (!m_avm_version.empty())
            {
                version = m_avm_version;
                return true;
            }
            if (block)
                return GetAvmVersionBlock(version);
            m_task.put(SYNC_AVM_VERSION);
            return true;
        }

        SMLK_BOOL AvmCustomUtil::GetAvmVersionBlock(std::string &version)
        {
            SMLK_LOGD("[1] GetDeviceVersion");
            CheckPtrVaild;
            AvmMessageBuilder builder_;
            builder_.TboxtoDevice().total_len(0x0004, 0).cmd_group(0x00).cmd_id(0x02).cmd_len(0x0000, 0);
            AvmMessage msg_ = builder_.end();
            if (m_sp_avm_service->SendMsg(msg_))
            {
                if (msg_.getAvmHead().cmd_group_id == 0x00 && msg_.getAvmHead().cmd_id == 0x82)
                {
                    version.clear();
                    SMLK_UINT8 *data_ = msg_.child_data();
                    version.insert(version.end(), data_, data_ + msg_.child_size());
                    SMLK_LOGD("[GetDeviceVer]=%s", version.c_str());
                    return true;
                }
            }
            SMLK_LOGD("Get device version failed!");
            return false;
        }

        SMLK_BOOL AvmCustomUtil::SyncTime(SMLK_BOOL block)
        {
            if (!block)
                m_task.put(SYNC_AVM_TIME);
            SMLK_LOGD("[2] SyncDeviceSysTime");
            CheckPtrVaild;
            auto pair = GetCustomTime();
            AvmMessageBuilder builder_;
            builder_.TboxtoDevice().total_len(0x000A, 0).cmd_group(0x00).cmd_id(0x03).cmd_len(0x0006, 0).cmd_data((SMLK_UINT8 *)&pair.first, sizeof(SMLK_UINT32)) // seconds
                .cmd_data((SMLK_UINT8 *)&pair.second, sizeof(SMLK_UINT16));                                                                                       // ms
            AvmMessage msg_ = builder_.end();
            if (m_sp_avm_service->SendMsg(msg_))
            {
                if (msg_.getAvmHead().cmd_group_id == 0x00 && msg_.getAvmHead().cmd_id == 0x83)
                {
                    SMLK_UINT8 *data_ = msg_.child_data();
                    SMLK_LOGD("[SyncTimeRes]=%d", *data_);
                    return true;
                }
            }
            SMLK_LOGD("SyncTime avm time failed !");
            return false;
        }

        std::pair<SMLK_UINT32, SMLK_UINT16> AvmCustomUtil::GetCustomTime()
        {
            timeval tTimeVal;
            gettimeofday(&tTimeVal, NULL);
            SMLK_UINT32 time_sec = tTimeVal.tv_sec;
            SMLK_UINT16 time_ms = tTimeVal.tv_usec / 1000;
            SMLK_LOGD("[GetSysTime]={[sec]=%ld [ms]=%d}", time_sec, time_ms);
            return std::make_pair(htobe32(time_sec), htobe16(time_ms));
        }

        SMLK_BOOL AvmCustomUtil::SyncTerminalId(SMLK_BOOL block)
        {
            if (!block)
                m_task.put(SYNC_AVM_TERMINAL_ID);
            SMLK_LOGD("[3] SyncTeriminalId=%s", m_imsi.c_str());
            CheckPtrVaild;
            AvmMessageBuilder builder_;
            builder_.TboxtoDevice().total_len(0x0011, 0).cmd_group(0x00).cmd_id(0x04).cmd_len(0x000D, 0).cmd_data((SMLK_UINT8 *)m_imsi.data(), m_imsi.size());
            AvmMessage msg_ = builder_.end();
            if (m_sp_avm_service->SendMsg(msg_))
            {
                if (msg_.getAvmHead().cmd_group_id == 0x00 && msg_.getAvmHead().cmd_id == 0x84)
                {
                    SMLK_UINT8 *data_ = msg_.child_data();
                    SMLK_LOGD("[SyncTerminalIdRes]=%d", *data_);
                    return true;
                }
            }
            SMLK_LOGD("SyncTerminalId avm IMSI failed !");
            return false;
        }

        void AvmCustomUtil::DealProcess()
        {
            while (m_working)
            {
                SMLK_UINT8 task = m_task.get();
                switch (task)
                {
                case SYNC_AVM_VERSION:
                {
                    GetAvmVersionBlock(m_avm_version);
                }
                break;
                case SYNC_AVM_TIME:
                {
                    SyncTime(true);
                }
                break;
                case SYNC_AVM_TERMINAL_ID:
                {
                    SyncTerminalId(true);
                }
                break;

                default:
                    break;
                }
            }
        }

    }
}
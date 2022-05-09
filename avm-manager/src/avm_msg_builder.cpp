/*****************************************************************************/
/**
 * \file       avm_msg_builder.cpp
 * \author     wukai
 * \date       2021/07/21
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include "avm_msg_builder.h"
#include <cstring>

namespace smartlink
{
    namespace AVM
    {
        AvmMessageBuilder::AvmMessageBuilder()
            : is_avm_2_tbox(false)
        {
        }

        AvmMessageBuilder::~AvmMessageBuilder()
        {
        }

        AvmMessageBuilder &AvmMessageBuilder::TboxtoDevice()
        {
            SMLK_UINT8 device_id = 0x00;
            _recv_msg.getAvmHead().device_type = device_id;
            _recv_msg.append(&device_id, sizeof(SMLK_UINT8));
            return *this;
        }

        AvmMessageBuilder &AvmMessageBuilder::DevicetoTbox()
        {
            is_avm_2_tbox = true;
            SMLK_UINT8 device_id = 0x01;
            _recv_msg.append(&device_id, sizeof(SMLK_UINT8));
            _recv_msg.getAvmHead().device_type = device_id;
            return *this;
        }

        AvmMessageBuilder &AvmMessageBuilder::total_len(SMLK_UINT16 len, SMLK_UINT8 flag)
        {
            SMLK_UINT16 temp_len = 0;
            if (flag) // Avm2Tbox
            {
                temp_len = be16toh(len);
            }
            else // Tbox2Avm
            {
                temp_len = htobe16(len);
            }
            _recv_msg.append((SMLK_UINT8 *)&temp_len, sizeof(SMLK_UINT16));
            _recv_msg.getAvmHead().total_cmd_len = temp_len;
            return *this;
        }

        AvmMessageBuilder &AvmMessageBuilder::cmd_group(SMLK_UINT8 group_id)
        {
            _recv_msg.append(&group_id, sizeof(SMLK_UINT8));
            _recv_msg.getAvmHead().cmd_group_id = group_id;
            return *this;
        }

        AvmMessageBuilder &AvmMessageBuilder::cmd_id(SMLK_UINT8 _id)
        {
            _recv_msg.append(&_id, sizeof(SMLK_UINT8));
            _recv_msg.getAvmHead().cmd_id = _id;
            return *this;
        }

        AvmMessageBuilder &AvmMessageBuilder::cmd_len(SMLK_UINT16 len, SMLK_UINT8 flag)
        {
            SMLK_UINT16 temp_len = 0;
            if (flag) // Avm2Tbox
            {
                temp_len = be16toh(len);
            }
            else // Tbox2Avm
            {
                temp_len = htobe16(len);
            }
            _recv_msg.append((SMLK_UINT8 *)&temp_len, sizeof(SMLK_UINT16));
            _recv_msg.getAvmHead().data_len = temp_len;
            return *this;
        }

        AvmMessageBuilder &AvmMessageBuilder::cmd_data(SMLK_UINT8 *data, std::size_t len, SMLK_UINT8 flag)
        {
            _recv_msg.append(data, len, flag);
            return *this;
        }

        AvmMessage &AvmMessageBuilder::end()
        {
            SMLK_UINT8 end_ = 0xFE;
            _recv_msg.append(&end_, sizeof(end_));
            return _recv_msg;
        }

        AvmMessage::AvmMessage()
        {
            SMLK_UINT8 head_ = 0xFE;
            m_msg.insert(m_msg.end(), &head_, &head_ + sizeof(head_));
            bzero(&m_head, sizeof(TspHead));
            bzero(&m_avm_head, sizeof(AvmHead));
            bzero(&m_avm_result, sizeof(AvmResult));
            m_avm_head.head = head_;
        }

        AvmMessage::~AvmMessage()
        {
            m_msg.clear();
        }

        TspHead &AvmMessage::getTspHead()
        {
            return m_head;
        }

        AvmHead &AvmMessage::getAvmHead()
        {
            return m_avm_head;
        }

        void AvmMessage::append(SMLK_UINT8 *data, std::size_t len, SMLK_UINT8 flag)
        {
            if (!data || len < 1)
                return;
            m_msg.insert(m_msg.end(), data, data + len);
            if (flag == 1)
            {
                m_child_data.insert(m_child_data.end(), data, data + len);
                if (len > 3)
                    memcpy(&m_avm_result, data, sizeof(AvmMessage::AvmResult));
            }
        }

        SMLK_UINT8 *AvmMessage::data() const
        {
            return (SMLK_UINT8 *)m_msg.data();
        }

        SMLK_UINT8 *AvmMessage::child_data() const // data from avm upload, not for tbox2avm use
        {
            return (SMLK_UINT8 *)m_child_data.data();
        }

        std::size_t AvmMessage::size() const
        {
            return m_msg.size();
        }

        std::size_t AvmMessage::child_size() const
        {
            return m_child_data.size();
        }

        const AvmMessage::AvmResult &AvmMessage::getAvmResult()
        {
            return m_avm_result;
        }

        void AvmMessage::setDeviceType(DeviceType dev_type)
        {
            m_device_type = dev_type;
            return;
        }

        DeviceType AvmMessage::getDeviceType()
        {
            return m_device_type;
        }

        AvmMessage &AvmMessage::operator=(IN AvmMessage &other)
        {
            m_msg = other.m_msg;
            m_child_data = other.m_child_data;
            memcpy(&m_head, &other.m_head, sizeof(TspHead));
            memcpy(&m_avm_head, &other.m_avm_head, sizeof(AvmHead));
            memcpy(&m_avm_result, &other.m_avm_result, sizeof(AvmMessage::AvmResult));
            return *this;
        }

    }
}
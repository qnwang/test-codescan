/*****************************************************************************/
/**
 * \file       avm_msg_builder.h
 * \author     wukai
 * \date       2021/07/21
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef SMLK_AVM_MSG_BUILDER_H_
#define SMLK_AVM_MSG_BUILDER_H_
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include "smlk_types.h"
#include <vector>

namespace smartlink
{
    namespace AVM
    {
        typedef struct
        {
            SMLK_UINT8 head;           // 1byte:帧头FE
            SMLK_UINT8 device_type;    // 1byte:发送设备编号
            SMLK_UINT16 total_cmd_len; // 1byte:指令报文总长
            SMLK_UINT8 cmd_group_id;   // 1byte:指令组id
            SMLK_UINT8 cmd_id;         // 1byte:指令id
            SMLK_UINT16 data_len;      // 1byte:指令长度
        } __attribute__((__packed__)) AvmHead;

        typedef struct
        {
            SMLK_UINT32 msg_id;  // 消息ID
            SMLK_UINT32 seq_id;  // 消息序号
            SMLK_UINT8 protocol; /*消息体采用协议类型         0x01: 808, 0x02: MQTT, 0x03: http*/
            SMLK_UINT8 qos;
            SMLK_UINT8 priority;
            SMLK_UINT32 reserved_dw; /* 保留字段，扩展用*/
        } __attribute__((__packed__)) TspHead;

        /*区分消息往来设备*/
        enum class DeviceType : SMLK_UINT8
        {
            DEV_AVM = 0X00,
            DEV_DMS,
            DEV_UNKNOWN,
        };

        enum class AvmErrCode : SMLK_UINT8
        {
            AVM_SUCCESS = 0x00,
            AVM_FAILED
        };

        class AvmMessage
        {
        public:
            typedef struct
            {
                SMLK_UINT16 id;
                SMLK_UINT8 res;
            } __attribute__((__packed__)) AvmResult;

        public:
            AvmMessage();
            ~AvmMessage();
            SMLK_UINT8 *data() const;       /*视频处理设备上传的全部数据*/
            std::size_t size() const;       /*视频处理设备上传的全部数据长度*/
            SMLK_UINT8 *child_data() const; /*视频处理设备上传的内容数据*/
            std::size_t child_size() const; /*视频处理设备上传的内容数据长度*/
            TspHead &getTspHead();
            AvmHead &getAvmHead();
            const AvmResult &getAvmResult();
            void setDeviceType(DeviceType dev_type);
            DeviceType getDeviceType();
            void append(SMLK_UINT8 *data, std::size_t len, SMLK_UINT8 flag = 0);

        private:
            std::vector<SMLK_UINT8> m_msg;        //总包
            std::vector<SMLK_UINT8> m_child_data; //指令内容
            TspHead m_head;
            AvmHead m_avm_head;
            SMLK_UINT8 m_group_id;
            SMLK_UINT8 m_cmd_id;
            AvmResult m_avm_result;
            DeviceType m_device_type;

        public:
            AvmMessage &operator=(IN AvmMessage &other);
        };

        class AvmMessageBuilder
        {
        public:
            AvmMessageBuilder();
            ~AvmMessageBuilder();
            AvmMessageBuilder &TboxtoDevice();
            AvmMessageBuilder &DevicetoTbox();
            AvmMessageBuilder &total_len(SMLK_UINT16 len, SMLK_UINT8 flag = 0); // 0:Tbox2Avm 1:Avm2Tbox
            AvmMessageBuilder &cmd_group(SMLK_UINT8 group_id);
            AvmMessageBuilder &cmd_id(SMLK_UINT8 _id);
            AvmMessageBuilder &cmd_len(SMLK_UINT16 len, SMLK_UINT8 flag = 0); // 0:Tbox2Avm 1:Avm2Tbox
            AvmMessageBuilder &cmd_data(SMLK_UINT8 *data, std::size_t len, SMLK_UINT8 flag = 0);
            AvmMessage &end();

        private:
            AvmMessage _recv_msg;
            SMLK_BOOL is_avm_2_tbox;
        };
    }
}

#endif // SMLK_AVM_MSG_BUILDER_H_
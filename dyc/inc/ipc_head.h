#pragma once
#include "smlk_types.h"
/*
单独下发配置参数
根据下发的upload_id匹配需要更新的参数
如本地无upload_id，此id返回设置失败
word：消息id
word:采集间隔ms
word:上报间隔 s
byte：上报数据类型 物理值 1 原始值
*/

#define DYC_CMD2_SUBBODY_SIZE 7
#define TSP_VERSION          0x00
namespace smartlink {

class MsgHead {
public:
    MsgHead()
        : m_id(0)
        , m_seq(0)
        , m_protocol(0)
        , m_compress(0)
        , m_length(0)
        , m_extra(0)
    {};
    MsgHead(SMLK_UINT32 id, SMLK_UINT32 seq, SMLK_UINT8 protocol, SMLK_UINT8 compress, SMLK_UINT16 length, SMLK_UINT32 extra)
        : m_id(id)
        , m_seq(seq)
        , m_protocol(protocol)
        , m_compress(compress)
        , m_length(length)
        , m_extra(extra)
    {}
    MsgHead(const MsgHead &other)
        : m_id(other.m_id)
        , m_seq(other.m_seq)
        , m_protocol(other.m_protocol)
        , m_compress(other.m_compress)
        , m_length(other.m_length)
        , m_extra(other.m_extra)
    {}
    MsgHead & operator=(const MsgHead &other)
    {
        if ( this != &other ) {
            this->m_id          = other.m_id;
            this->m_seq         = other.m_seq;
            this->m_protocol    = other.m_protocol;
            this->m_compress    = other.m_compress;
            this->m_length      = other.m_length;
            this->m_extra       = other.m_extra;
        }
        return *this;
    }
public:
    SMLK_UINT32     m_id;
    SMLK_UINT32     m_seq;
    SMLK_UINT32     m_protocol:4;
    SMLK_UINT32     m_compress:4;
    SMLK_UINT32     :8;
    SMLK_UINT32     m_length:16;
    SMLK_UINT32     m_extra;
};  // class MsgHead

// ipc struct
typedef struct
{
  SMLK_UINT16 upload_id;
  SMLK_UINT16 collection_interval;
  SMLK_UINT16 upload_interval;
  SMLK_UINT8  md5[16];
  SMLK_UINT8  data_type;
}DbcConfigInfo;

typedef struct
{
  SMLK_UINT8 name_len;
  SMLK_UINT8 name[0];
  SMLK_UINT16 upload_id;
  SMLK_UINT16 collection_interval;
  SMLK_UINT16 upload_interval;
  SMLK_UINT8  url_len;
  SMLK_UINT8  url[0];
  SMLK_UINT8  md5[16];
  SMLK_UINT8  data_type;
} SendDbcSignalPayload;

typedef struct
{
  SMLK_UINT8 version;
  SMLK_UINT8 ID;
  SMLK_UINT8 reserve;
  SMLK_UINT8 dbc_num;
  SendDbcSignalPayload config[0];
}SendDbcRawData;

typedef struct
{
  SMLK_UINT8 version;
  SMLK_UINT8 ID;
  SMLK_UINT8 reserve;
  SMLK_UINT8 dbc_num;
}SendDbcRawDataHead;

typedef struct
{
    SMLK_UINT16  upload_cmd_id;
    SMLK_UINT16  collect_interval;
    SMLK_UINT16  upload_interval;
    SMLK_UINT8   upload_data_type;
}TspSetInterval;

typedef struct
{
  SMLK_UINT8     version;
  SMLK_UINT8     cmd_id;   //
  SMLK_UINT8     id_count; // 设置个数
  TspSetInterval set_info[0];
}TspSetIntervalPayload;   // SIZE = 4

typedef struct
{
  SMLK_UINT8     version;
  SMLK_UINT8     cmd_id;  //
  SMLK_UINT8     id_count;//设置个数
}TspSetIntervalHead;

typedef struct
{
  SMLK_UINT8     name_len;
  SMLK_UINT8     name[0];
}TspClearInterval;

typedef struct
{
  SMLK_UINT8     ver_id;
  SMLK_UINT8     cmd_id;
  SMLK_UINT8     num;
  TspClearInterval clear_info[0];
}TspClearIntervalPayload;

typedef struct
{
    SMLK_UINT8 version;
    SMLK_UINT8 cmd_id;
    SMLK_UINT8 doc_num;
}PayloadHead;

typedef struct
{
    SMLK_UINT8 version;
    SMLK_UINT8 cmd_id;
    SMLK_UINT8 load[0]; // reserved
}RawDycPayload;

typedef struct
{
    SMLK_UINT8  ver_id;
    SMLK_UINT16 upload_id;/*上报消息ID_E260_E266*/
    SMLK_UINT16 collect_interval;/*ms*/
    SMLK_UINT16 upload_interval;/*ms*/
    SMLK_UINT8  type;/*1:原始值 2：物理值*/
}DycReadRespSubBody;

typedef struct
{
    SMLK_UINT8  name_len;  /*文件名长度*/
    char* name;/*文件名*/
    SMLK_UINT8  md5[16];/*1:原始值 2：物理值*/
    SMLK_UINT8  value_type;
}DycReadFileRespSubBody;

typedef struct
{
    SMLK_UINT8  num;   /*配置个数*/
    DycReadRespSubBody data[0];
}DycReadRespSubBodyS;

typedef struct
{
    SMLK_UINT16 seq_id;/*对应平台消息的流水号*/
    SMLK_UINT8  ver_id;/*版本号*/
    SMLK_UINT8  sub_id;/*0x02*/
    SMLK_UINT8  num;   /*配置个数*/
    DycReadRespSubBody sub_data[0];
}DycReadIntervalRespBody;

typedef struct
{
    SMLK_UINT16 seq_id;/*对应平台消息的流水号*/
    SMLK_UINT8  ver_id;/*版本号*/
    SMLK_UINT8  sub_id;/*0x04*/
    SMLK_UINT8  num;   /*配置个数*/
    DycReadFileRespSubBody sub_data[0];
}DycReadFileRespBody;

typedef struct
{
    SMLK_UINT16 upload_id;/*上报消息ID_E260_E266*/
    SMLK_UINT8 result;  /*平台设置结果*/
}DycSetRespSubBody;

typedef struct
{
    SMLK_UINT16 seq_id;/*对应平台消息的流水号*/
    SMLK_UINT8  ver_id;
    SMLK_UINT8  sub_id;/*0x01*/
    SMLK_UINT8  num;   /*配置个数*/
    DycSetRespSubBody sub_data[0];
}DycSetRespBody;

typedef struct
{
    SMLK_UINT16 seq_id;/*对应平台消息的流水号*/
    SMLK_UINT16  msg_id;/*对应平台消息de ID*/
    SMLK_UINT8 result;
}__attribute__((__packed__))J808CommResp;

}   // namespace smartlink

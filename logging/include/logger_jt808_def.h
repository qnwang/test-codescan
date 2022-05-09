/*****************************************************************************/
/**
* \file       logger_jt808_def.h
* \author     wukai
* \date       2021/06/29
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
#ifndef _LOGGER_JT808_DEF_H_
#define _LOGGER_JT808_DEF_H_
/*****************************************************************************
*                                头文件引用                                  *
*****************************************************************************/
#include "smlk_types.h"

namespace smartlink {
/*tsp发过来的消息ID，IPC头部MsgHead中的id*/
#define JTT808_LOG_STATUS              0X8F52  /*日志开关设置/日志等级查询*/
#define JTT808_LOG_STATUS_ARK          0X0F52  /*日志开关设置应答/日志等级查询结果*/

#define JTT808_LOG_UP_LOAD             0X8F53  /*历史日志上传请求*/
#define JTT808_LOG_UP_LOAD_ARK         0X0F53  /*历史日志上传应答(事件日志/实时日志主动上报)*/

#define JTT808_LOG_REPORT_ARK          0X0F54  /*事件告警主动上报*/

#define JTT808_LOG_COMMON_ARK          0X0001  /*general response*/

typedef struct
{
  SMLK_UINT32   msg_id;           // 消息ID
  SMLK_UINT32   seq_id;           // 消息序号
  SMLK_UINT8    protocol;        /*消息体采用协议类型         0x01: 808, 0x02: MQTT, 0x03: http*/
  SMLK_UINT8    qos;
  SMLK_UINT8    priority;
  SMLK_UINT32   reserved_dw;     /* 保留字段，扩展用*/
}__attribute__((__packed__))LoggerHead;

enum class F52CmdType : SMLK_UINT8 {
    CMD_OFFLINE_LOG_LEVEL = 0,      //设置离线日志等级
    CMD_EVENT_LOG_SWITCH,           //事件日志开关
    CMD_ACTIVE_LOG_SWITCH,          //实时日志开关
    CMD_QUERY_OFFLINE_LOG_LEVEL,    //查询当前日志等级
    CMD_WARN_VALUE                  //设置告警阈值---作用于内存、磁盘、CPU检查， 百分比
};

typedef struct
{
    SMLK_UINT8 cmd_type;          // 命令类型 0:离线日志等级 1:事件日志开关 2:实时日志开关 3:查询当前日志等级 4:设置告警阈值
    SMLK_UINT8 cmd_content;
}__attribute__((__packed__))F52Data;

#define     GMT_TIME_LEN    6
#define     LOG_SERVER_SVAE_PATH_LENGTH 128
#define     LOG_SERVER_FILE_NAME_LENGTH 64
typedef struct
{
    SMLK_UINT8 time[GMT_TIME_LEN];
    SMLK_UINT8 general_type;          // 0:请求历史日志列表 1:请求发送历史日志 2:请求所有历史日志
    SMLK_UINT8 log_type;              // 0:TSP消息日志 1:程序运行日志
}__attribute__((__packed__))F53General;

/*jtt808 common response*/
typedef struct
{
    SMLK_UINT16 seq_id;/*对应平台消息的流水号*/
    SMLK_UINT16  msg_id;/*对应平台消息de ID*/
    SMLK_UINT8 result;
}__attribute__((__packed__))CommResp;

typedef struct
{
    SMLK_UINT8 time[GMT_TIME_LEN];
    SMLK_UINT16 seq_id; /*如果是主动上报则填0xffff*/
    SMLK_UINT8 action_id; // 0:上传历史日志列表； 1:上传历史日志结果通知 2:事件日志 3:实时日志
}__attribute__((__packed__))XF53Head;

typedef struct
{
    char log_path[LOG_SERVER_SVAE_PATH_LENGTH];
    SMLK_UINT8 start_time[GMT_TIME_LEN]; // BCD start time
    SMLK_UINT8 end_time[GMT_TIME_LEN]; // BCD end time
}__attribute__((__packed__))F53CMD_0;

typedef struct
{
    SMLK_UINT8 log_id;
    SMLK_UINT8 log_name[LOG_SERVER_FILE_NAME_LENGTH];
    SMLK_UINT16 log_size;
}__attribute__((__packed__))LogFileAttribute;


typedef struct
{
    XF53Head general_head;
    SMLK_UINT8 log_type;
    char      log_local_path[LOG_SERVER_SVAE_PATH_LENGTH];
    SMLK_UINT8 log_count;
}__attribute__((__packed__))XF53CMD_0_Resp;

enum class CommonRC : SMLK_UINT8 {
    CommonRC_OK = 0,
    CommonRC_FAILED,
    CommonRC_OUT_RANGE,
    CommonRC_UNSUPPORT
};

enum class F53ReplyActionCode : SMLK_UINT8 {
    F53ReplyActionCode_UpLoad_LogList = 0,
    F53ReplyActionCode_UpLoad_Result,
    F53ReplyActionCode_Event_Log,
    F53ReplyActionCode_RunTime_Log
};


typedef struct
{
    SMLK_UINT8 time[GMT_TIME_LEN];
    SMLK_UINT16 seq_id; /*如果是主动上报则填0xffff*/
    SMLK_UINT8 action_id; // 0:事件 1：告警
    SMLK_UINT8 commitId[32];
    SMLK_UINT8 reserved[16];
    SMLK_UINT16 msgLength;
}__attribute__((__packed__))XF54Head;

enum class EventType : SMLK_UINT16
{
    RebootType = 0,     //开机事件
    ResetType           //复位事件
};

enum class WarnMsgId : SMLK_UINT16
{
    CPUOverWarn = 0,
    MemOverWarn,
    DiskOverWarn,
    FotaWarn,
    ProcessWarn,
    MaxId = 0xffff
};

enum class WarnLevel : SMLK_UINT8
{
    LowLevel = 0,
    MidLevel,
    HighLevel
};

enum class LogType : SMLK_UINT8
{
    ELogTspMessage = 0,
    ELogHirainMessage,
    ELogOtaMessage
};

};


#endif // _LOGGER_JT808_DEF_H_
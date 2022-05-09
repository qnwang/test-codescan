/*****************************************************************************/
/**
 * \file       jit808_avm_def.h
 * \author     wukai
 * \date       2021/07/20
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef SMLK_JIT808_AVM_DEF_H_
#define SMLK_JIT808_AVM_DEF_H_
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include "smlk_tools.h"
using namespace smartlink::tools;

#define JTT808_AVM_COMMON_ARK 0X0001                      /*general response*/
#define JTT808_8F61_AVM_REAL_TIME_VIDEO 0x8F61            /*实时视频请求: TSP向TBox请求实时视频传输,包括实时视频传输,主动发起双向语音对讲,单向监听,向所有终端广播语音和特定透传等,TBox在收到此消息后回复通用应答0x0001*/
#define JTT808_9102_AVM_REAL_TIME_VIDEO_CTRL 0x9102       /*实时视频控制: TSP向TBox发送音视频实时传输控制指令,用于切换码流,暂停码流传输,关闭音视频传输通道等,消息体数据格式见下表,TBox在收到此消息后回复通用应答0x0001*/
#define JTT808_9205_AVM_HISTRORY_VIDEO_LIST_QUERY 0x9205  /*历史视频资源查询: TSP平台按照视频类型,通道号,报警类型和起止时间等组合条件,向TBox请求查询历史视频录像文件列表,TBox回复0x1205*/
#define JTT808_1205_AVM_HISTRORY_VIDEO_LIST_ARK 0x1205    /*历史视频列表上传: TBox响应平台的查询视频资源列表指令,上传视频资源列表消息应答,如列表过大需要分包传输时,采用JT/T808中定义的分包机制处理,TSP应对每个单独分包回复视频平台通用应答0x8001*/
#define JTT808_8F62_AVM_HISTRORY_VIDEO_PLAY 0x8F62        /*历史视频回放: 平台向终端设备请求视频录像回放,终端首先返回通用应答0x0001,然后把录像用RTMP方式推流到流媒体服务器*/
#define JTT808_9202_AVM_HISTRORY_VIDEO_PLAY_CTRL 0x9202   /*历史视频控制: AVM进行音视频录像回放过程中,TSP可下发回放控制指令对回放过程进行控制*/
#define JTT808_9206_AVM_HISTRORY_VIDEO_UPLOAD 0x9206      /*文件上传请求: TSP向TBox下发文件上传命令,TBox回复通用应答,AVM通过FTP方式将文件上传到指定路径*/
#define JTT808_1206_AVM_HISTRORY_VIDEO_UPLOAD_ARK 0x1206  /*文件传输完成通知: 当全部文件通过FTP上传完成后,TBox上报此指令通知TSP*/
#define JTT808_9207_AVM_HISTRORY_VIDEO_UPLOAD_CTRL 0x9207 /*文件传输控制(暂不支持): TSP通知TBox暂停,继续或取消正在传输中的所有文件*/
#define JTT808_8F49_AVM_EVENT_REPORT 0x8F49               /*平台下行控制指令: 功能id:0x21 为设置报警事件上传附件地址*/
#define JTT808_0F49_AVM_EVENT_REPORT_ARK 0x0F49           /*终端上行上报指令: 功能id:0x23 事件上传报警事件; 功能id:0x21 设置报警事件上传附件地址响应*/
namespace smartlink
{
    namespace AVM
    {
        enum class AVMCommonRC : SMLK_UINT8
        {
            CommonRC_OK = 0,
            CommonRC_FAILED,
            CommonRC_OUT_RANGE,
            CommonRC_UNSUPPORT
        };

        enum class AVMType : SMLK_UINT8
        {
            AVM_NONE = 0,
            AVM_DONGRUAN,
            AVM_SHANGFU
        };

        enum class DMSType : SMLK_UINT8
        {
            DMS_NONE = 0,
            DMS_HENGRUN
        };

        typedef struct
        {
            SMLK_UINT32 msg_id;  // 消息ID
            SMLK_UINT32 seq_id;  // 消息序号
            SMLK_UINT8 protocol; /*消息体采用协议类型         0x01: 808, 0x02: MQTT, 0x03: http*/
            SMLK_UINT8 qos;
            SMLK_UINT8 priority;
            SMLK_UINT32 reserved_dw; /* 保留字段,扩展用*/
        } __attribute__((__packed__)) GenrnalHead;

        class Data8F61
        {
        public:
            std::string host;
            SMLK_UINT8 channel_id;
            SMLK_UINT8 data_type;
            SMLK_UINT8 code_type;
        };

        class Data9206
        {
        public:
            Data9206()
            {
                bzero(&bcd_start_time, sizeof(6));
                bzero(&bcd_end_time, sizeof(6));
            };
            std::string host;
            SMLK_UINT16 port;
            std::string user;
            std::string passwd;
            std::string server_path;
            SMLK_UINT8 channel_id;
            SMLK_UINT8 bcd_start_time[6];
            SMLK_UINT8 bcd_end_time[6];

            std::string toString()
            {
                std::string ret("Data9206 host = [");
                ret += host;
                ret += "]    port = [";
                ret += std::to_string(port);
                ret += "]    user = [";
                ret += user;
                ret += "]    passwd = [";
                ret += passwd;
                ret += "]    server_path = [";
                ret += server_path;
                ret += "]    channel_id = [";
                ret += std::to_string(channel_id);
                ret += "]    start_time = [";
                ret += bcd_timestamp_2_str(bcd_start_time);
                ret += "]    end_time = [";
                ret += bcd_timestamp_2_str(bcd_end_time) + "]";
                return ret;
            };

        private:
            Data9206 &operator=(IN Data9206 &other)
            {
                this->host = other.host;
                this->port = other.port;
                this->user = other.user;
                this->passwd = other.passwd;
                this->server_path = other.server_path;
                return *this;
            }
        };

        typedef struct
        {
            SMLK_UINT8 storage_type;
            SMLK_UINT8 play_type;     /*回放方式*/
            SMLK_UINT8 play_step_num; /*快进或快退倍数*/
            SMLK_UINT8 bcd_start_time[6];
            SMLK_UINT8 bcd_end_time[6];
        } __attribute__((__packed__)) Data8F62;

        typedef struct
        {
            SMLK_UINT8 channel_id;       /*逻辑通道号*/
            SMLK_UINT8 cmd_id;           /*控制指令*/
            SMLK_UINT8 close_video_type; /*关闭音视频类型*/
            SMLK_UINT8 code_type;        /*切换码流类型*/
        } __attribute__((__packed__)) Data9102;

        typedef struct
        {
            SMLK_UINT8 channel_id;
            SMLK_UINT8 play_type;     /*回放方式*/
            SMLK_UINT8 play_step_num; /*快进或快退倍数*/
            SMLK_UINT8 bcd_position_time[6];
        } __attribute__((__packed__)) Data9202;

        typedef struct
        {
            SMLK_UINT8 channel_id;
            SMLK_UINT8 bcd_start_time[6];
            SMLK_UINT8 bcd_end_time[6];
            SMLK_UINT8 alarm_flag[8];
            SMLK_UINT8 stream_type;
            SMLK_UINT8 code_type;
            SMLK_UINT8 storage_type;
        } __attribute__((__packed__)) Data9205;

        typedef struct
        {
            SMLK_UINT16 seq_id;    /*对应平台文件上传指令的流水号*/
            SMLK_UINT8 upload_cmd; /*0:暂停;1:继续;2:取消*/
        } __attribute__((__packed__)) Data9207;

        class Data8F49_F368
        {
        public:
            SMLK_UINT16 fun_id;
            SMLK_UINT16 param_id;
            SMLK_UINT16 param_len;
            SMLK_UINT8 result; // fun_id 0x23 event report result
            std::string host;
        };

    }
}

#endif // SMLK_JIT808_AVM_DEF_H_
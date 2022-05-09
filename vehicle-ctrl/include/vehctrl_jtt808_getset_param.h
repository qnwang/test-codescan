
/*****************************************************************************/
/**
 * \file       vehctrl_jtt808_getset_param.h
 * \author     huangxin
 * \date       2020/11/25
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_JTT808_MSG8103_H_
#define _VEHICLE_JTT808_MSG8103_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <iostream>
#include <string>
#include <vector>
#include "smlk_error.h"
#include "smlk_types.h"
#include "vehctrl_general_definition.h"
#include "vehctrl_queue.h"
#include "vehctrl_jtt808.h"

namespace smartlink
{
  namespace jt808msg8103
  {
/*START,设置终端参数消息8103中的参数ID***************************************************************************************/
#define PARAM_HEARTBEAT_INTERVAL_ID 0X0001    /*终端心跳发送间隔,单位为秒（s）;默认30S，范围(30S~180S)*/
#define PARAM_TCP_RESP_TIMEOUT_ID 0X0002      /*TCP 消息应答超时时间，单位为秒（s）;默认300S,范围(60S~600S)*/
#define PARAM_SLEEP_REPORT_INTERVAL_ID 0X0027 /*休眠时汇报时间间隔，单位为秒（s），默认7200S，范围（1800S~86400）*/
#define PARAM_FATIGUE_DRIVER_TIME_ID 0XF021   /*疲劳驾驶时长阈值（小时），范围:(1~6）,默认为：4小时*/
// #define PARAM_REALTIME_DATA_UPLOAD_CYCLE_ID             0XF022/*实时数据上传周期（单位：s）,范围:(0~60）如果该值为0表示不上传；没有收到该配置则默认10S;*/
// #define PARAM_REALTIME_DATA_GATHER_CYCLE_ID             0XF023/*实时数据采集周期（单位：ms） ；没有收到该配置则默认：1000ms（采集周期为100ms的整数倍）,范围:(100~60000）*/
#define PARAM_STATIS_INFO_UPLOAD_CYCLE_ID 0XF027       /*统计信息上传周期 单位：分钟 ,没有收到该配置则默认10分钟，范围:(5~60）；*/
#define PARAM_LONG_TIME_PARKING_TIME_ID 0XF028         /*超长时间停车阈值 单位：分钟 ,没有收到该配置则默认720分钟 范围（240~7200）*/
#define PARAM_LOW_TO_ULTRALOW_POWER_INTERVAL_ID 0XF210 /*低功耗模式到超低功耗模式的时间间隔，单位为分钟（min），参数长度N=2，默认值2880（48h）。范围（5~65530），0xFFFF代表不进入超低功耗*/
#define PARAM_NORMAL_TO_LOW_POWER_INTERVAL_ID 0XF211   /*从正常工作模式转入低功耗模式的时间间隔，单位为分钟（min），参数长度N=1，默认值240（min）。范围（1~65530），0xFFFF代表不进入低功耗*/
#define PARAM_PREDIC_CRUIS_SWITCH_ID 0XF212            /*预见性巡航功能开关*/
#define PARAM_OVERSPEED_EVENT_ID 0XF213                /*判定车辆超速事件的设置*/
#define PARAM_RAPIDLY_ACCELERATE_EVENT_ID 0XF214       /*判定车辆急加速事件的设置*/
#define PARAM_SHARP_SLOWDOWN_EVENT_ID 0XF215           /*判定车辆急减速事件的设置*/
#define PARAM_4G_ANTENNA_OPEN_ID 0XF217                /*消贷锁车4G天线开路后是否锁车*/

#define SYS_PRO_NAME_TSP_HBT_INTERVAL_DEFALUT_VALUE 30                   /*单位为s*/
#define SYS_PRO_NAME_TSP_TCP_RESP_TIMEOUT_DEFALUT_VALUE 300              /*单位为s*/
#define SYS_PRO_NAME_SLEEP_REPORT_INTERVAL_DEFALUT_VALUE 7200            /*单位为s*/
#define SYS_PRO_NAME_FATIGUE_DRIVER_TIME_DEFALUT_VALUE 4                 /*单位为小时*/
#define SYS_PRO_NAME_REALTIME_DATA_UPLOAD_CYCLE_DEFALUT_VALUE 10         /*单位为s*/
#define SYS_PRO_NAME_REALTIME_DATA_GATHER_CYCLE_DEFALUT_VALUE 1000       /*单位为ms*/
#define SYS_PRO_NAME_STATIS_INFO_UPLOAD_CYCLE_DEFALUT_VALUE 10           /*单位为分钟*/
#define SYS_PRO_NAME_LONG_TIME_PARKING_TIME_DEFALUT_VALUE 720            /*单位为分钟*/
#define SYS_PRO_NAME_LOW_TO_ULTRALOW_POWER_INTERVAL_DEFALUT_VALUE 0XFFFF /*单位为分钟*/
#define SYS_PRO_NAME_NORMAL_TO_LOW_POWER_INTERVAL_DEFALUT_VALUE 60       /*单位为分钟*/
#define SYS_PRO_NAME_PREDIC_CRUIS_SWITCH_DEFALUT_VALUE 1                 /*开启*/
#define SYS_PRO_NAME_4G_ANTENNA_LOCK_DEFALUT_VALUE 2                     /*不锁车*/

#define SYS_PRO_NAME_OVERSPEED_ATTRIBUTE_DEFALUT_VALUE 0
#define SYS_PRO_NAME_OVERSPEED_THRESHOLD_DEFALUT_VALUE 100 /*km/h*/
#define SYS_PRO_NAME_OVERSPEED_DURATION_DEFALUT_VALUE 60   /*单位为5s*/

#define SYS_PRO_NAME_RAPIDLY_SPEED_ATTRIBUTE_DEFALUT_VALUE 0
#define SYS_PRO_NAME_RAPIDLY_SPEED_UP_THRESHOLD_DEFALUT_VALUE 20             /*m/s*/
#define SYS_PRO_NAME_RAPIDLY_SPEED_ACCELERATE_UP_THRESHOLD_DEFALUT_VALUE 4   /*m/s2*/
#define SYS_PRO_NAME_RAPIDLY_SPEED_DOWN_THRESHOLD_DEFALUT_VALUE 5            /*m/s*/
#define SYS_PRO_NAME_RAPIDLY_SPEED_ACCELERATE_DOWN_THRESHOLD_DEFALUT_VALUE 1 /*m/s*/

#define SYS_PRO_NAME_SLOWDOWN_SPEED_ATTRIBUTE_DEFALUT_VALUE 0
#define SYS_PRO_NAME_SLOWDOWN_SPEED_UP_THRESHOLD_DEFALUT_VALUE 20             /*m/s*/
#define SYS_PRO_NAME_SLOWDOWN_SPEED_ACCELERATE_UP_THRESHOLD_DEFALUT_VALUE 8   /*m/s2*/
#define SYS_PRO_NAME_SLOWDOWN_SPEED_DOWN_THRESHOLD_DEFALUT_VALUE 5            /*m/s*/
#define SYS_PRO_NAME_SLOWDOWN_SPEED_ACCELERATE_DOWN_THRESHOLD_DEFALUT_VALUE 4 /*m/s*/

#define BOUNDARY_VALUE_ZERO 0
#define BOUNDARY_VALUE_ONE 1
#define BOUNDARY_VALUE_TWO 2
#define BOUNDARY_VALUE_FATIGUE_DRIVE_TIME 6
#define BOUNDARY_VALUE_UPLOAD_CYCLE_TIME 60
#define BOUNDARY_VALUE_LOW_POWER_INTERVAL_TIME 65530
#define BOUNDARY_VALUE_GATHER_CYCLE_TIME_MIN 100
#define BOUNDARY_VALUE_GATHER_CYCLE_TIME_MAX 60000
#define BOUNDARY_VALUE_INFO_UPLOAD_CYCLE_TIME 5
#define BOUNDARY_VALUE_HEARTBEAT_INTERVAL_MIN 30
#define BOUNDARY_VALUE_HEARTBEAT_INTERVAL_MAX 180
#define BOUNDARY_VALUE_TCP_RESP_TIMEOUT_TIME 600
#define BOUNDARY_VALUE_SLEEP_REPORT_INTERVAL_TIME_MIN 1800
#define BOUNDARY_VALUE_SLEEP_REPORT_INTERVAL_TIME_MAX 86400

    /*设置终端参数消息0x8103d的消息格式*/
    typedef struct
    {
      SMLK_UINT16 seq_id; /*应答流水号*/
      SMLK_UINT8 param_num;
    } __attribute__((__packed__)) Msg0104Head;
#define MSG8104_HEAD_LEN sizeof(Msg0104Head)

    /*设置终端参数消息0x8103的消息格式*/
    typedef struct
    {
      SMLK_UINT8 param_num; /*参数总数*/
    } __attribute__((__packed__)) Msg8013Head;
#define MSG8103_HEAD_LEN sizeof(Msg8013Head)

    typedef struct
    {
      SMLK_UINT32 param_id;
      SMLK_UINT8 param_len;
    } __attribute__((__packed__)) ParamCommon;
#define PARAM_COMMON_LEN sizeof(ParamCommon)

    /*设置终端参数消息0x8106的消息格式*/
    typedef struct
    {
      SMLK_UINT8 param_num;
      SMLK_UINT32 param_id[0];
    } __attribute__((__packed__)) Msg8106;

#define MAX_VOICE_HINT_DATA_LEN 256
    /*车辆超速事件设置的参数格式*/
    typedef struct
    {
      SMLK_UINT8 attri;                                       /*属性*/
      SMLK_UINT8 speed_threshold;                             /*车速阈值*/
      SMLK_UINT8 time;                                        /*持续时间*/
      SMLK_UINT8 voice_hint_len;                              /*语音提示的文字长度*/
      SMLK_UINT8 voice_hint_content[MAX_VOICE_HINT_DATA_LEN]; /*语音提示的文字数据*/
    } __attribute__((__packed__)) OverSpeedEvent;
    enum
    {
      VOICE_HINT_OFF,
      VOICE_HINT_ON
    };

    /*急加减速事件设置的参数格式*/
    typedef struct
    {
      SMLK_UINT8 attri;                                       /*属性*/
      SMLK_UINT8 speed_upper_threshold;                       /*车速上限阈值*/
      SMLK_UINT8 accspeed_upper_threshold;                    /*加速速上限阈值*/
      SMLK_UINT8 speed_lower_threshold;                       /*车速下限阈值*/
      SMLK_UINT8 accspeed_lower_threshold;                    /*加速速下限阈值*/
      SMLK_UINT8 voice_hint_len;                              /*语音提示的文字长度*/
      SMLK_UINT8 voice_hint_content[MAX_VOICE_HINT_DATA_LEN]; /*语音提示的文字数据*/
    } __attribute__((__packed__)) SharpSpeedEvent;

    typedef struct
    {
      /*tsp下发的param的类型*/
      SMLK_UINT8 param_type;
      /*系统参数ID*/
      std::string sys_param_name;
    } ParamValue;
    enum
    {
      TYPE_BYTE = 1,
      TYPE_WORD,
      TYPE_DWORD,
      TYPE_STRING,
      TYPE_COMPOUND
    };
  }

  class DissectorMsgTerminalParam : public IDissector
  {
  public:
    DissectorMsgTerminalParam();
    virtual ~DissectorMsgTerminalParam();

  public:
    virtual SMLK_RC OnChanged(IN SMLK_UINT8 &, IN SMLK_UINT16 &, IN SMLK_DOUBLE &) { return SMLK_RC::RC_OK; };
    virtual SMLK_RC ResultEncode(IN RctrlResultQueue &) { return SMLK_RC::RC_OK; };
    virtual SMLK_RC Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &, OUT SMLK_UINT8 &is_encode);

  private:
    SMLK_RC Msg8103Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode);
    SMLK_RC Msg8104Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode);
    SMLK_RC Msg8106Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode);
    SMLK_UINT32 GetParamDefaultVaule(IN std::string &param_name);
    SMLK_RC GetParamVaule(IN SMLK_UINT32 &param_id, OUT jt808msg8103::ParamValue &valueinfo, OUT std::vector<SMLK_UINT8> &output_vec);
    SMLK_UINT8 SetParamVaule(IN SMLK_UINT32 param_id, IN SMLK_UINT8 param_len, IN SMLK_UINT8 *param_value, IN jt808msg8103::ParamValue &valueinfo);
    SMLK_RC SendMsg0104(IN RctrlHead &head);
    SMLK_RC SendMsg0104(IN RctrlHead &head, std::vector<SMLK_UINT32> &param_id_vec);
    std::map<SMLK_UINT32, jt808msg8103::ParamValue> m_id_map; /*key为tsp8103消息中的参数ID*/
    // std::map<SMLK_UINT32, std::make_pair<SMLK_UINT8, SMLK_UINT32>> m_id_map;/*key为tsp8103消息中的参数ID*/
  };
};
#endif

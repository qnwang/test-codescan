/*****************************************************************************/
/**
 * \file       close_ctrl_common.h
 * \author     weixuefeng
 * \date       2021/7/06
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _CCTRL_COMMON_H_
#define _CCTRL_COMMON_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <vector>
#include <chrono>
#include <iostream>
#include "smlk_error.h"
#include "smlk_types.h"
#include "smlk_log.h"
#include "vehicle_data_index_def.h"
#include <string>
#define MCU_DATA_LEN_1 1                        /*一个字节*/
#define MCU_DATA_LEN_2 2                        /*两个字节*/
#define CCTRL_ACTION_INVALID 0xFF               /*动作无效*/
#define QUERY_INVALID 0xFF                      /*查询结果无效*/
#define CCTRL_DATA_INVALID 0xFFFFFFFF           /*数值无效*/
#define CCTRL_STATUS_INVALID 0xFFFFFFFFFFFFFFFF /*无效状态*/
namespace smartlink
{
    namespace CloseCtrl
    {
        enum VehicleDataUsage : SMLK_UINT8
        {
            VD_USE_FOR_CTRL = 0,
            VD_USE_FOR_QUERY,
        };
        /*****************************************************************************
         *                                控车功能                                    *
         *****************************************************************************/
        /*wifi协议消息*/
        typedef struct
        {
            SMLK_UINT8 inter_id;
            SMLK_UINT8 inter_type;
            SMLK_UINT8 inter_state;
            SMLK_UINT32 inter_uid;
            std::vector<SMLK_UINT8> exts_msg;
        } __attribute__((__packed__)) CCtrlInterInfo;

        /*发给MCU service控制命令格式*/
        typedef struct
        {
            SMLK_UINT16 cmd_id;  /*实际下发的命令Id*/
            SMLK_UINT8 action;   /*动作：0xff表示动作无效*/
            SMLK_UINT8 data_len; /*数据长度*/
        } __attribute__((__packed__)) CCtrlSetMcuCmd;

        /*近控设置命令格式*/
        typedef struct
        {
            SMLK_UINT16 cmd_id; /*实际下发的命令Id*/
            SMLK_UINT8 action;  /*动作：0xff表示动作无效*/
            SMLK_DOUBLE data;   /*命令内容*/
        } __attribute__((__packed__)) CctrlSetCmd;

        /*车辆控制命令头部字段*/
        typedef struct
        {
            SMLK_UINT16 seq;  /*sequence number*/
            SMLK_UINT8 count; /*命令个数*/
        } __attribute__((__packed__)) CctrlMcuCmdHead;

        /*近控设置命令的结果*/
        typedef struct
        {
            SMLK_UINT16 cmd_id; /*实际下发的命令Id*/
            SMLK_UINT8 action;  /*动作：0xff表示动作无效*/
            SMLK_UINT8 result;  /*当前命令的响应结果*/
        } __attribute__((__packed__)) CctrlSetCmdResult;

        /*Ivi电源控制响应*/
        typedef struct
        {
            SMLK_UINT8 cmd_type; /*命令类型:0x03*/
            SMLK_UINT8 result;   /*响应结果*/
        } __attribute__((__packed__)) IviPowerCtrlResult;

        /*Ivi多媒体控制响应*/
        typedef struct
        {
            SMLK_UINT8 cmd_type; /*命令类型:0x50*/
            SMLK_UINT8 act_res;  /*执行结果: 0x01-IVI已执行 0x02-IVI未执行*/
            SMLK_UINT8 ctrl_id;  /*控制分类ID: 0x01-音量控制 0x02-收音机操作 0x03-音频播放器操作 0x04-液晶屏设置*/
            SMLK_UINT8 ctrl_msg; /*功能分类ID: 见控制表各子类所示*/
        } __attribute__((__packed__)) IviMediaCtrlResult;

        enum class IviCmdCommonResult : SMLK_UINT8
        {
            SMLK_IVI_CMD_EXE = 0X01,    /*IVI已执行*/
            SMLK_IVI_CMD_NO_EXE = 0X02, /*IVI未执行*/
        };

        typedef struct
        {
            SMLK_UINT8 cmd;
            SMLK_UINT8 id;
            SMLK_UINT8 msg;
        } __attribute__((__packed__)) ControlMsgToIVI;

        typedef struct
        {
            SMLK_UINT8 cmd;
            SMLK_UINT8 id;
        } __attribute__((__packed__)) ControlMsgToIVIpower;

        typedef struct
        {
            SMLK_UINT8 type;
            SMLK_UINT8 state;
        } __attribute__((__packed__)) ControlMsg;

        typedef struct
        {
            ControlMsg volume;
            SMLK_UINT8 cmd;
        } __attribute__((__packed__)) IviControlMsg;

        /*近控MVU设置命令的结果码*/
        enum CCtrlCmdResult : SMLK_UINT16
        {
            CCTRL_CMD_RESULT_OK = 0,               /*设置成功*/
            CCTRL_CMD_RESULT_ERROR = 1,            /*设置失败*/
            CCTRL_CMD_RESULT_ENGINE_NOT_START = 2, /*发动机未启动*/
            CCTRL_CMD_RESULT_TIMEOUT = 3,          /*设置超时*/
        };

        /*近控获取命令格式*/
        typedef struct
        {
            SMLK_UINT16 cmd_id;    /*近控内部get命令Id*/
            SMLK_DOUBLE value;     /*动作：0xff表示动作无效*/
            SMLK_DOUBLE query_val; /*查询结果,将VD转换成wifi协议规定数据*/
        } __attribute__((__packed__)) CctrlGetCmd;

        /*wifi控制功能列表*/
        enum class CCMsgFrom : SMLK_UINT16
        {
            C_EVENT_VOLUME_CTRL_MSG,
            C_EVENT_SOUND_CTRL_MSG,
            C_EVENT_LCD_CTRL_MSG,
            C_EVENT_RADIO_CTRL_MSG,
            C_EVENT_AUDIO_CTRL_MSG,
            C_EVENT_BCM_CTRL_MSG,
            C_EVENT_DCM_CTRL_MSG,
            C_EVENT_AIRCONDITIONER_CTRL_MSG,
        };

        enum
        {
            CCTRL_ACTION_DOOR_LOCK = 1, /*车门锁上锁*/
            CCTRL_ACTION_DOOR_UNLOCK,   /*车门锁解锁*/
        };

        /**/
        enum
        {
            CCTRL_ACTION_ON = 1, /*打开*/
            CCTRL_ACTION_OFF,    /*关闭*/
        };

        enum
        {
            CCTRL_ACTION_DOWN = 1, /*降低*/
            CCTRL_ACTION_UP,       /*升高*/
        };

        enum
        {
            CCTRL_MCU_ACTION_OFF = 1, /*关闭*/
            CCTRL_MCU_ACTION_ON,      /*打开*/
        };

        /*原车空调模式的动作*/
        enum
        {
            CCTRL_ACTION_BLOW_FACE = 1,     /*吹面*/
            CCTRL_ACTION_BLOW_FACE_FEET,    /*吹面+吹脚*/
            CCTRL_ACTION_BLOW_FEET,         /*吹脚*/
            CCTRL_ACTION_BLOW_FEET_DEFROST, /*吹脚+除霜*/
        };

        enum
        {
            CCTRL_ACTION_AC_OUTER_LOOP = 1, /*空调外循环*/
            CCTRL_ACTION_AC_INNER_LOOP = 2, /*空调内循环*/
        };

        enum
        {
            CCTRL_MCU_ACTION_AC_OUTER_LOOP = 1, /*空调外循环*/
            CCTRL_MCU_ACTION_AC_INNER_LOOP = 2, /*空调内循环*/
        };

        /*近控MCU设置命令*/
        enum
        {
            CCTRL_CMD_AC_SWITCH = 400,              /*原车空调开关:ON OFF*/
            CCTRL_CMD_AC_AIROUTLET_MODE = 401,      /*原车空调出风模式：吹面，吹面+吹脚，吹脚，吹脚+除霜,有action，无data*/
            CCTRL_CMD_AC_TEMPER = 402,              /*原车空调温度,无action，有data*/
            CCTRL_CMD_AC_WIND = 403,                /*原车空调风量,无action，有data*/
            CCTRL_CMD_AC_LOOP_MOOD = 404,           /*原车空调循环模式：内循环，外循环,有action，无data*/
            CCTRL_CMD_AC_COMPRESSOR = 405,          /*原车空调压缩机:ON OFF,有action，无data*/
            CCTRL_CMD_AC_AUTO_SWITCH = 406,         /*原车空调auto开关:ON OF,有action，无data*/
            CCTRL_CMD_AC_DEFROST = 407,             /*原车空调除霜开关:ON OF,有action，无data*/
            CCTRL_CMD_AC_VENTILATION = 408,         /*原车空调一键通风开关:ON OF,有action，无data*/
            CCTRL_CMD_INDEPT_WARM_AIR = 409,        /*独立暖风开关:ON OF*/
            CCTRL_CMD_INDEPT_WARM_AIR_TEMPER = 410, /*独立暖风温度：1-7*/
            CCTRL_CMD_PARKING_AC_SWITCH = 411,      /*驻车空调开关:ON OF*/
            CCTRL_CMD_PARKING_AC_AUTO = 412,        /*驻车空调AUTO开关:ON OF*/
            CCTRL_CMD_PARKING_AC_TEMPER = 413,      /*驻车空调温度,无action，有data*/
            CCTRL_CMD_PARKING_AC_WIND = 414,        /*驻车空调风量,无action，有data*/
            CCTRL_CMD_AC_QUERY = 415,               /*车内温度查询*/
            CCTRL_CMD_DRIVER_WINDOW = 501,          /*主驾侧车窗*/
            CCTRL_CMD_COPILOT_WINDOW = 502,         /*副驾侧车窗*/
            CCTRL_CMD_LIGHT_CTRL = 700,             /*室内灯控制*/
            CCTRL_CMD_DRIVER_LIGHT = 701,           /*主驾侧阅读灯*/
            CCTRL_CMD_COPILOT_LIGHT = 702,          /*副驾侧阅读灯*/
            CCTRL_CMD_INVALID = 0XFFFF,             /*LAST ONE*/
        };

        /*****************************************************************************
         *                                IVI命令                                     *
         *****************************************************************************/
        /*IVI控制功能*/
        enum SmlkCctrlIviCmdType : SMLK_UINT8
        {
            SMLK_CCTRL_IVI_CMD_POWER = 0X03, /*多媒体电源控制*/
            SMLK_CCTRL_IVI_CMD_MEDIA = 0X50, /*多媒体控制*/
        };
        /*IVI控制功能*/
        enum
        {
            CCTRL_CMD_VOLUME_CTRL = 0X01, /*音量控制*/
            CCTRL_CMD_RADIO_CTRL = 0X02,  /*收音机操作*/
            CCTRL_CMD_AUDIO_CTRL = 0X03,  /*音频播放器操作*/
            CCTRL_CMD_LCD_CTRL = 0X04,    /*液晶屏设置*/
        };
        /*音量控制*/
        enum
        {
            CCTRL_CMD_SYS_VOL_UP = 0X11,        /*系统音量加*/
            CCTRL_CMD_SYS_VOL_DOWN = 0X12,      /*系统音量减*/
            CCTRL_CMD_CALL_VOL_UP = 0X13,       /*通话音量加*/
            CCTRL_CMD_CALL_VOL_DOWN = 0X14,     /*通话音量减*/
            CCTRL_CMD_MUTE_SWITCH = 0X20,       /*静音/非静音切换*/
            CCTRL_CMD_BALANCE_UP = 0X31,        /*左右平衡加*/
            CCTRL_CMD_BALANCE_DOWN = 0X32,      /*左右平衡减*/
            CCTRL_CMD_EFFECT_NONE = 0X40,       /*无音效*/
            CCTRL_CMD_EFFECT_ROCK = 0X41,       /*摇滚*/
            CCTRL_CMD_EFFECT_JAZZ = 0X42,       /*爵士/蓝调*/
            CCTRL_CMD_EFFECT_CLASSIC = 0X43,    /*古典音乐*/
            CCTRL_CMD_EFFECT_VOCAL = 0X44,      /*声乐*/
            CCTRL_CMD_EFFECT_ELECTRONIC = 0X45, /*电子音乐*/
            CCTRL_CMD_EFFECT_POP = 0X46,        /*流行*/
            CCTRL_CMD_TONE_LOW = 0X51,          /*音调低*/
            CCTRL_CMD_TONE_MID = 0X52,          /*音调中*/
            CCTRL_CMD_TONE_HIGH = 0X53,         /*音调高*/
            CCTRL_CMD_SOUND_NONE = 0XFF,        /*无*/
        };
        /*收音机操作*/
        enum
        {
            CCTRL_CMD_FM = 0X01,              /*FM*/
            CCTRL_CMD_AM = 0X02,              /*AM*/
            CCTRL_CMD_SEARCH = 0X03,          /*开始搜索/停止搜索*/
            CCTRL_CMD_COLLECTION = 0X04,      /*收藏/取消收藏 当前电台*/
            CCTRL_CMD_COLLECTION_PAGE = 0X05, /*进入/退出 列表界面*/
            CCTRL_CMD_PAGE_UP = 0X06,         /*搜台列表界面，切换上电台*/
            CCTRL_CMD_PAGE_DOWN = 0X07,       /*搜台列表界面，切换下电台*/
            CCTRL_CMD_FREQUENCY_DOWN = 0X08,  /*当前播放电台频率微调减*/
            CCTRL_CMD_FREQUENCY_UP = 0X09,    /*当前播放电台频率微调加*/
            CCTRL_CMD_SUBSCRIBE_RADIO = 0X10, /*收藏指定电台--扩展项*/
            CCTRL_CMD_BROCAST_RADIO = 0X11,   /*播放指定电台--扩展项*/
            CCTRL_CMD_SEARCH_LEFT = 0X0A,     /*搜索并播放左有效电台*/
            CCTRL_CMD_SEARCH_RIGHT = 0X0B,    /*搜索并播放右有效电台*/
            CCTRL_CMD_RADIO_NONE = 0XFF,      /*无*/
        };
        /*音频播放器操作*/
        enum
        {
            CCTRL_CMD_PLAYER_MODE_SINGLE = 0X01,   /*单曲循环*/
            CCTRL_CMD_PLAYER_MODE_FULL = 0X02,     /*全部循环*/
            CCTRL_CMD_PLAYER_MODE_RANDOM = 0X03,   /*随机播放*/
            CCTRL_CMD_PLAYER_PLAY_PAUSE = 0X04,    /*播放/暂停*/
            CCTRL_CMD_PLAYER_PLAY_PREVIOUS = 0X05, /*播放上一首*/
            CCTRL_CMD_PLAYER_PLAY_NEXT = 0X06,     /*播放下一首*/
            CCTRL_CMD_PLAYER_NONE = 0XFF,          /*无*/
        };
        /*液晶屏亮度设置*/
        enum
        {
            CCTRL_CMD_LCD_MODE_DAY = 0X01,           /*白天模式*/
            CCTRL_CMD_LCD_MODE_NIGHT = 0X02,         /*黑夜模式*/
            CCTRL_CMD_LCD_MODE_AUTO = 0X03,          /*自动模式*/
            CCTRL_CMD_LCD_BRIGHTNESS_DIMMING = 0X04, /*亮度调暗*/
            CCTRL_CMD_LCD_BRIGHTNESS_ADJUST = 0X05,  /*亮度调亮*/
            CCTRL_CMD_LCD_NONE = 0XFF,               /*无*/
        };
        /*液晶屏电源设置*/
        enum
        {
            CCTRL_CMD_POWER_MODE_NORMAL = 0X00, /*正常工作*/
            CCTRL_CMD_POWER_MODE_OFF = 0X01,    /*关屏*/
            CCTRL_CMD_POWER_MODE_TIME = 0X02,   /*显示时间*/
        };

        /*****************************************************************************
         *                                查询功能                                    *
         *****************************************************************************/
        enum VehicleDataSwitch : SMLK_UINT8
        {
            VD_SWITCH_OFF = 0,
            VD_SWITCH_ON = 1,
        };

        enum QuerySwitch : SMLK_UINT8
        {
            QUERY_SWITCH_ON = 0,
            QUERY_SWITCH_OFF = 1,
        };

        enum VehicleDataOutletMode : SMLK_UINT8
        {
            VD_OUTLET_FACE = 0,
            VD_OUTLET_FACE_FEET = 1,
            VD_OUTLET_FEET = 2,
            VD_OUTLET_FEET_DEFROST = 3,
            VD_OUTLET_DEFROST = 4,
        };

        enum QueryOutletMode : SMLK_UINT8
        {
            QUERY_OUTLET_FACE = 0,
            QUERY_OUTLET_FACE_FEET = 1,
            QUERY_OUTLET_FEET = 2,
            QUERY_OUTLET_FEET_DEFROST = 3,
        };

        enum VehicleDataCirculationMode : SMLK_UINT8
        {
            VD_CIRCULATION_INNER = 0,
            VD_CIRCULATION_OUTER = 1,
        };

        enum QueryCirculationMode : SMLK_UINT8
        {
            QUERY_CIRCULATION_INNER = 0,
            QUERY_CIRCULATION_OUTER = 2,
        };

    }
}

#endif

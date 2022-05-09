/*****************************************************************************/
/**
 * \file       remote_ctrl_prot_interf.h
 * \author     huangxin
 * \date       2020/10/30
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_CTRL_COMMON_H_
#define _VEHICLE_CTRL_COMMON_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <vector>
#include <chrono>
#include <iostream>
#include "smlk_error.h"
#include "smlk_types.h"
#include "smlk_log.h"
#include "tsp_service_api.h"
#include "ipc_service_api.h"

/*****************************************************************************
 *                                 宏定义                                   *
 *****************************************************************************/
/*车控模块配置文件*/
#define VEHICLE_CTRL_JSON_CONFIG_FILE_PATH_NAME "/oemapp/smartlink/default/property/config_file.json"
/*车控模块维护的一些需掉电保存的状态*/
#define VEHICLE_CTRL_STATUS_FILE_PATH "/userdata/smartlink/record/vehicle_ctrl/"
#define VEHICLE_CTRL_STATUS_FILE_NAME "status.json"
#define VEHICLE_CTRL_STATUS_FILE_PATH_NAME "/userdata/smartlink/record/vehicle_ctrl/status.json"
/*短信消贷*/
#define TEXT_MSG_FINANCIAL_LCK_FUNCTION_OPEN "99982101CLCKON"   /*打开金融锁车功能*/
#define TEXT_MSG_FINANCIAL_LCK_FUNCTION_CLOSE "99982101CLCKOFF" /*关闭金融锁车功能*/
#define TEXT_MSG_TBOX_4G_ANTENNA_LOCK "99982101AOCLCKON"        /*4G天线开路后锁车*/
#define TEXT_MSG_TBOX_4G_ANTENNA_UNLOCK "99982101AOCLCKOFF"     /*4G天线开路后不锁车*/
#define TEXT_MSG_TBOX_WEAK_UP "982101DXHUANXING"                /*短信唤醒*/
#define TEXT_MSG_TBOX_RESTART "982101DXCHONGQI"                 /*短信重启*/
/*自定义数据*/
#define VIN_LENGTH 17                          /*VIN码长度*/
#define EIN_LENGTH 17                          /*EIN码长度*/
#define BOUNDARY_ENGINE_SPEED 500              /*发动机临界转速*/
#define DATA_INVALID_UINT8 0XFF                /*无效值 UINT8*/
#define DATA_INVALID_UINT16 0XFFFF             /*无效值 UINT16*/
#define DATA_INVALID_UINT64 0xFFFFFFFFFFFFFFFF /*无效值 UINT64*/
#define AC_LOWEST_TEMP 17
#define AC_HIGHEST_TEMP 33
#define SL_PROCESS_NAME "RCTRL"
#define GOTO_ENCODE 1
#define NOT_GOTO_ENCODE 2
#define PRO_LOC_CHANGCHUN "0" /*产地配置参数--长春*/
#define PRO_LOC_QINGDAO "1"   /*产地配置参数--青岛*/
/*仪表盘参数*/
#define CHANNEL_COMM 0               /*通信CAN通道-->通信CAN1*/
#define DASHBOARD_PERIOD 5 * 60 * 10 /*仪表盘保养信息下发时长5min,之所以x10是因为计数器内参数设置为100ms周期,10次为1s*/

/*****************************************************************************
 *                          车身状态及车控内部参数                             *
 *****************************************************************************/
/*唤醒源类型*/
enum class WakeupSource : SMLK_UINT8
{
    WAKEUP_SOURCE_UNKNOWN = 0,
    WAKEUP_SOURCE_RTC,
    WAKEUP_SOURCE_GYRO,
    WAKEUP_SOURCE_CAN,
    WAKEUP_SOURCE_SMS,
};
/*发动机状态--0F51 0X06*/
enum class EngineMode : SMLK_UINT8
{
    ENGINE_MODE_UNKNOWN = 0, /*发动机状态无效*/
    ENGINE_MODE_ON,          /*发动机状态已经启动:转速>=500*/
    ENGINE_MODE_OFF,         /*发动机状态停止:转速<500*/
};
/*电源状态--0F51 0X07*/
enum class PowerMode : SMLK_UINT8
{
    POWER_MODE_UNKNOWN = 0, /*车辆电源状态无效*/
    POWER_MODE_ON,          /*车辆电源状态已上电--IGN ON*/
    POWER_MODE_OFF,         /*车辆电源状态已断电--IGN OFF*/
};
/*CAN网络状态--内部自定义*/
enum class CanBusMode : SMLK_UINT8
{
    /*MCU上报状态*/
    CAN_BUS_STATE_NORMAL = 0,            /*正常*/
    CAN_BUS_STATE_SLEEP = 0x01,          /*休眠--无网络管理默认5s无总线报文*/
    CAN_BUS_STATE_AUTHENTICATING = 0x02, /*认证中--mcu不上报*/
    CAN_BUS_STATE_FAILED = 0x03,         /*认证失败或者没有连接--mcu不上报*/
    CAN_BUS_STATE_LOST = 0x04,           /*信号丢失--mcu长短*/
    /*内部定义*/
    CAN_BUS_STATE_UNKONWN = 0X05, /*状态未知*/
};
enum class PsCtrlMode : SMLK_UINT8
{
    PS_MODE_AVAILABLE = 0, /*ps可远控*/
    PS_MODE_UNAVAILABLE,   /*ps不可远控*/
    PS_MODE_UNKNOWN,       /*ps状态未知*/
};
/*远控消息来源*/
enum class RctrlMsgType : SMLK_UINT8
{
    MSG_TYPE_TSP = 1,
    MSG_TYPE_VD_STATUS = 2,
    MSG_TYPE_TEXT_MSG = 3,
    MSG_TYPE_OTHER = 4, /*目前包括MCU提供的电源状态,SystemPower提供的唤醒源*/
};
/*内部事件触发编号*/
enum class VecCtrlInnerEventID : SMLK_UINT8
{
    VEC_CTRL_EVENT_DASH_BOARD_MTN = 0, /*仪表盘保养事件*/
};
enum
{
    CUR_PROT_JTT808 = 1, /*当前是jtt808协议*/
};
/*****************************************************************************
 *                                MCU协议                                     *
 *****************************************************************************/
#define MCU_DATA_LEN_1 0x01                /*一个字节*/
#define MCU_DATA_LEN_2 0x02                /*两个字节*/
#define MCU_SINGLE_MSG_NO_CONTENT_LEN 0X07 /*MCU单条无指令内容消息长度*/
#define MCU_ENGINE_START_OVER_TIME 41      /*MCU发动机启动超时时间40s MPU超时等待41s*/
/*车辆控制命令头部字段*/
typedef struct
{
    SMLK_UINT16 seq;    /*流水号*/
    SMLK_UINT8 cmd_num; /*命令个数*/
} __attribute__((__packed__)) SmlkMcuCmdHead;
#define SMLK_MCU_CMD_DEFAULT_LEN 4
/*发送至MCU的请求格式*/
typedef struct
{
    SMLK_UINT16 cmd_id;                    /*命令名称*/
    SMLK_UINT8 cmd_act;                    /*命令动作*/
    SMLK_UINT8 cmd_data_len;               /*执行数据-数据长度*/
    std::vector<SMLK_UINT8> cmd_data_cont; /*执行数据-内容*/
} __attribute__((__packed__)) SmlkMcuCmd;
/*接受的MCU响应格式*/
typedef struct
{
    SMLK_UINT16 cmd_id; /*命令名称*/
    SMLK_UINT8 cmd_act; /*命令动作*/
    SMLK_UINT8 cmd_res; /*命令执行结果*/
} __attribute__((__packed__)) SmlkMcuCmdResp;
/*接受的MCU响应枚举*/
enum SmlkMcuCmdResult : SMLK_UINT8
{
    SMLK_MCU_CMD_RESULT_SUCCESS = 0,          /*控车成功*/
    SMLK_MCU_CMD_RESULT_ERROR,                /*控车失败*/
    SMLK_MCU_CMD_RESULT_BATTERY_LOW_NO_SP,    /*蓄电池电量低无法执行*/
    SMLK_MCU_CMD_RESULT_NO_SP,                /*条件不满足*/
    SMLK_MCU_CMD_RESULT_PS_AUTH_FAIL,         /*防盗认证失败*/
    SMLK_MCU_CMD_RESULT_VEHICLE_NO_AVALIABLE, /*车辆不可控*/
};

/*车门上锁解锁*/
enum SmlkMcuMsgDoorLockStatus : SMLK_UINT8
{
    SMLK_MCU_ACTION_DOOR_UNLOCK = 1, /*门锁解锁*/
    SMLK_MCU_ACTION_DOOR_LOCK = 2,   /*门锁上锁*/
};
/*通用开关状态*/
enum SmlkMcuMsgCommonSwitch : SMLK_UINT8
{
    SMLK_MCU_ACTION_OFF = 1, /*关闭*/
    SMLK_MCU_ACTION_ON = 2,  /*打开*/
};
/*原车空调出风模式*/
enum SmlkMcuMsgOriACBlowMode : SMLK_UINT8
{
    SMLK_MCU_ACTION_BLOW_FACE = 1,         /*吹面*/
    SMLK_MCU_ACTION_BLOW_FACE_FEET = 2,    /*吹面+吹脚*/
    SMLK_MCU_ACTION_BLOW_FEET = 3,         /*吹脚*/
    SMLK_MCU_ACTION_BLOW_FEET_DEFROST = 4, /*吹脚+除霜*/
};
/*原车空调循环模式*/
enum SmlkMcuMsgOriACWrapMode : SMLK_UINT8
{
    SMLK_MCU_ACTION_AC_OUTER_LOOP = 1, /*空调外循环*/
    SMLK_MCU_ACTION_AC_INNER_LOOP = 2, /*空调内循环*/
};
/*怠速暖机工作模式*/
enum SmlkMcuMsgIdleWarmUpMode : SMLK_UINT8
{
    SMLK_MCU_ACTION_IDLING_WARM_UP_CLOSE = 1,      /*怠速暖机关*/
    SMLK_MCU_ACTION_AC_IDLING_WARM_UP_LEVEL_1 = 2, /*怠速暖机1挡*/
    SMLK_MCU_ACTION_AC_IDLING_WARM_UP_LEVEL_2 = 3, /*怠速暖机2挡*/
};
/*后视镜加热*/
enum SmlkMcuMsgRearMirrorHeatMode : SMLK_UINT8
{
    SMLK_MCU_ACTION_REARVIEW_MIRROR_HEAT_CLOSE = 1, /*后视镜加热关闭*/
    SMLK_MCU_ACTION_REARVIEW_MIRROR_HEAT = 2,       /*后视镜加热*/
};

/*远控内部定义数据,为了避免和MCU以及VD数据冲突,定义一个较大的数据*/
enum SmlkRctrlInnerMsgID : SMLK_UINT32
{
    SMLK_VEHCTRL_CMD_ENGINE_MODE = 0x77FF, /*发动机状态: 无效:0 启动:1 停止:2  判断依据:VehicleData 发动机转速 >500为启动, <500为停止*/
    SMLK_VEHCTRL_CMD_POWER_MODE,           /*车辆电源状态: 无效:0 上电:1 断电:2    判断依据:MCU IGN ON or OFF*/
    SMLK_VEHCTRL_CMD_WAKEUP_SOURCE,        /*唤醒源: 异常类型:0 定时唤醒事件:1 g_sensor唤醒事件:2 can信号唤醒事件:3 短信唤醒事件:4 判断依据:Syspower 唤醒事件*/
    SMLK_VEHCTRL_CMD_PS_CTRL_MODE,         /*PS控制状态: 可远控:0 不可远控:1*/
};

/*MCU & MPU通信命令ID表*/
enum SmlkMcuMsgID : SMLK_UINT32
{
    SMLK_MCU_CMD_DOOR = 100,                       /*车门锁: 解锁:1 上锁:2*/
    SMLK_MCU_CMD_DOUBLE_FLASHING = 200,            /*双闪: 无动作*/
    SMLK_MCU_CMD_ENGINE = 300,                     /*发动机 关闭:1 打开:2*/
    SMLK_MCU_CMD_IDLING_WARM_UP = 301,             /*怠速暖机: 关闭:1 打开:2*/
    SMLK_MCU_CMD_AC_SWITCH = 400,                  /*原车空调开关: 关闭:1 打开:2*/
    SMLK_MCU_CMD_AC_AIROUTLET_MODE = 401,          /*原车空调出风模式: 吹面:1 吹面+吹脚:2 吹脚:3 吹脚+除霜:4*/
    SMLK_MCU_CMD_AC_TEMPER = 402,                  /*原车空调温度: 无动作 数据:170~330*/
    SMLK_MCU_CMD_AC_WIND = 403,                    /*原车空调风量: 无动作 数据:0~13*/
    SMLK_MCU_CMD_AC_LOOP_MOOD = 404,               /*原车空调循环模式: 外循环:1 内循环:2 有action，无data*/
    SMLK_MCU_CMD_AC_COMPRESSOR = 405,              /*原车空调压缩机: 关闭:1 打开:2 有action，无data*/
    SMLK_MCU_CMD_AC_AUTO_SWITCH = 406,             /*原车空调auto开关: 关闭:1 打开:2 有action，无data*/
    SMLK_MCU_CMD_AC_DEFROST = 407,                 /*原车空调除霜开关: 关闭:1 打开:2 有action，无data*/
    SMLK_MCU_CMD_AC_VENTILATION = 408,             /*原车空调一键通风开关: 关闭:1 打开:2 有action，无data*/
    SMLK_MCU_CMD_INDEPT_WARM_AIR = 409,            /*独立暖风开关: 关闭:1 打开:2*/
    SMLK_MCU_CMD_INDEPT_WARM_AIR_TEMPER = 410,     /*独立暖风温度: 无动作 数据:1~7*/
    SMLK_MCU_CMD_PARKING_AC_SWITCH = 411,          /*驻车空调开关: 关闭:1 打开:2*/
    SMLK_MCU_CMD_PARKING_AC_AUTO = 412,            /*驻车空调AUTO开关: 关闭:1 打开:2*/
    SMLK_MCU_CMD_PARKING_AC_TEMPER = 413,          /*驻车空调温度: 无动作 数据:170~330*/
    SMLK_MCU_CMD_PARKING_AC_WIND = 414,            /*驻车空调风量: 无动作 数据:0~13*/
    SMLK_MCU_CMD_AUTO_TEMPER_COOL = 415,           /*一键清凉: 关闭:1 打开:2*/
    SMLK_MCU_CMD_AUTO_TEMPER_HEAT = 416,           /*一键制热: 关闭:1 打开:2*/
    SMLK_MCU_CMD_REARVIEW_MIRROR = 500,            /*后视镜加热: 关闭加热:1 加热:2*/
    SMLK_MCU_CMD_DRIVER_WINDOW = 501,              /*主驾侧车窗 上升:1 下降:2*/
    SMLK_MCU_CMD_COPILOT_WINDOW = 502,             /*副驾侧车窗 上升:1 下降:2*/
    SMLK_MCU_CMD_FINANCIAL_LOCK_FUNC_ENABLE = 600, /*金融锁车功能使能，禁止,SMLK_MCU_ACTION_FLOCK_FUNC_ENABLE*/
    SMLK_MCU_CMD_ANTENNA_LOCK = 605,               /*天线开路锁车: 关闭:1 打开:2*/
    SMLK_MCU_CMD_INDOOR_LIGHT_CTRL = 700,          /*室内灯控制 按下:2*/
    SMLK_MCU_CMD_DRIVER_LIGHT = 701,               /*主驾侧阅读灯 按下:2*/
    SMLK_MCU_CMD_COPILOT_LIGHT = 702,              /*副驾侧阅读灯 按下:2*/
    SMLK_MCU_CMD_MAILBOX_SECURITY = 800,           /*油箱防盗开关: 关闭:1 打开:2*/
    SMLK_MCU_CMD_RUC_SWITCH = 900,                 /*智能冷机开关: 关闭:1 打开:2*/
    SMLK_MCU_CMD_RUC_TEMPER = 901,                 /*智能冷机温度控制: 无动作 数据0~250*/
    SMLK_MCU_CMD_RUC_DEFROST = 902,                /*智能冷机除霜: 关闭:1 打开:2*/
};
/*****************************************************************************
 *                            TSP协议(数据同CAN矩阵)                           *
 *****************************************************************************/
/*Tsp消息头*/
typedef struct
{
    SMLK_UINT32 msg_id;            /*消息ID*/
    SMLK_UINT32 seq_id;            /*消息序号*/
    smartlink::ProtoclID protocol; /*消息体采用协议类型    0x01:808, 0x02:MQTT, 0x03:http*/
    SMLK_UINT8 qos;                /*重传策略*/
    SMLK_UINT8 priority;           /*优先级*/
    SMLK_UINT32 reserved_dw;       /*保留字段*/
} __attribute__((__packed__)) RctrlHead;
/*车门上锁解锁*/
enum SmlkTspMsgDoorLockStatus : SMLK_UINT8
{
    SMLK_TSP_ACTION_DOOR_LOCK = 0,   /*门锁上锁*/
    SMLK_TSP_ACTION_DOOR_UNLOCK = 1, /*门锁解锁*/
};
/*通用开关状态*/
enum SmlkTspMsgCommonSwitch : SMLK_UINT8
{
    SMLK_TSP_ACTION_OFF = 0, /*关闭*/
    SMLK_TSP_ACTION_ON = 1,  /*打开*/
};
/*原车空调出风模式*/
enum SmlkTspMsgOriACBlowMode : SMLK_UINT8
{
    SMLK_TSP_ACTION_BLOW_FACE = 0,         /*吹面*/
    SMLK_TSP_ACTION_BLOW_FACE_FEET = 1,    /*吹面+吹脚*/
    SMLK_TSP_ACTION_BLOW_FEET = 2,         /*吹脚*/
    SMLK_TSP_ACTION_BLOW_FEET_DEFROST = 3, /*吹脚+除霜*/
};
enum SmlkVdMsgOriACModeStatus : SMLK_UINT8
{
    SMLK_VD_ORIAC_BLOW_FACE = 0,         /*吹面*/
    SMLK_VD_ORIAC_BLOW_FACE_FEET = 1,    /*吹面+吹脚*/
    SMLK_VD_ORIAC_BLOW_FEET = 2,         /*吹脚*/
    SMLK_VD_ORIAC_BLOW_FEET_DEFROST = 3, /*吹脚+除霜*/
    SMLK_VD_ORIAC_BLOW_DEFROST = 4,      /*除霜*/
};
/*原车空调循环模式*/
enum SmlkTspMsgOriACWrapMode : SMLK_UINT8
{
    SMLK_TSP_ACTION_AC_INNER_LOOP = 0, /*空调内循环*/
    SMLK_TSP_ACTION_AC_OUTER_LOOP = 1, /*空调外循环*/
};
/*ps是否可远控*/
enum SmlkTspPsRctrlAvailableSatatus : SMLK_UINT8
{
    SMLK_TSP_PS_AVAILABLE = 0,   /*ps可远控*/
    SMLK_TSP_PS_UNAVAILABLE = 1, /*ps不可远控*/
};
/*智能冷机开关控制*/
enum SmlkTspMsgRucSwitch : SMLK_UINT8
{
    SMLK_TSP_RUC_SWITCH_OFF = 0x00,
    SMLK_TSP_RUC_SWITCH_ON = 0x01,
};
/*智能冷机远程除霜控制*/
enum SmlkTspMsgRucDefrost : SMLK_UINT8
{
    SMLK_TSP_RUC_NO_DEFROST = 0x00, /*不除霜(预留)*/
    SMLK_TSP_RUC_DEFROST = 0x01,    /*远程除霜*/
};

/*****************************************************************************
 *                               0001消息部分                                 *
 *****************************************************************************/
/*远控通用应答结果码*/
enum SmlkRctrl_0001_Result : SMLK_UINT8
{
    SMLK_TSP_0001_RESULT_SUCCESS = 0,  /*执行成功or确认信息接受*/
    SMLK_TSP_0001_RESULT_FAILED,       /*失败*/
    SMLK_TSP_0001_RESULT_OUT_OF_RANGE, /*配置内容超出范围*/
    SMLK_TSP_0001_RESULT_NOT_SUPPORT,  /*不支持*/
};

/*****************************************************************************
 *                               8F42消息部分                                 *
 *****************************************************************************/
enum SmlkRctrl_8F42_Result : SMLK_UINT8
{
    SMLK_TSP_8F42_RESULT_ERROR = 0, /*执行失败*/
    SMLK_TSP_8F42_RESULT_OK,        /*执行成功*/
};

/*****************************************************************************
 *                               8F41消息部分                                 *
 *****************************************************************************/
#define GMT_TIME_LEN 6
/*远控子指令定义*/
enum Jtt808_8F41_SubCmd : SMLK_UINT8
{
    JTT808_8F41_DOOR_CTRL = 0x01,       /*0x01 车门控制,实现*/
    JTT808_8F41_DOUBLE_FLASHING_CTRL,   /*0x02 双闪控制,实现*/
    JTT808_8F41_WINDOW_UPDOWN_CTRL,     /*0x03 车窗升降控制*/
    JTT808_8F41_SUNROOF_CTRL,           /*0x04 天窗控制*/
    JTT808_8F41_LIGHT_CTRL,             /*0x05 灯光控制*/
    JTT808_8F41_ENGINE_CTRL,            /*0x06 发动机启动or停止,实现*/
    JTT808_8F41_POWER_CTRL,             /*0x07 车辆电源上电or下电*/
    JTT808_8F41_WIFI_CTRL,              /*0x08 wifi打开or关闭*/
    JTT808_8F41_AIRCONDITION_CTRL,      /*0x09 空调控制,实现*/
    JTT808_8F41_WIPER_CTRL,             /*0x0A 雨刮控制*/
    JTT808_8F41_REARVIEW_MIRROR_CTRL,   /*0x0B 后视镜控制,实现*/
    JTT808_8F41_WATER_HEATING_CTRL,     /*0x0C 车辆水暖热车控制*/
    JTT808_8F41_FINANCIAL_LOCK_CTRL,    /*0x0D 金融锁车功能打开or关闭,实现*/
    JTT808_8F41_GUARD_SPEEDLIMIT_CTRL,  /*0x0E 车辆防盗限速打开or关闭*/
    JTT808_8F41_OILTANK_SECURITY_CTRL,  /*0x0F 油箱防盗开关打开or关闭*/
    JTT808_8F41_IDLING_WARM_UP,         /*0x10 怠速暖机开关打开or关闭*/
    JTT808_8F41_INTELLIGNT_RUC = 0X15,  /*0x15 智能冷机控制*/
    JTT808_8F41_AUTO_TEMPER_SET = 0X16, /*0x16 一键温暖&一键清凉设置*/
    JTT808_8F41_INVAID = 0XFF
};
/*远控命令结果码*/
enum SmlkRctrl_8F41_Result : SMLK_UINT8
{
    SMLK_TSP_8F41_RESULT_OK = 0,                  /*执行成功*/
    SMLK_TSP_8F41_RESULT_ERROR,                   /*执行失败*/
    SMLK_TSP_8F41_RESULT_OVERDUE,                 /*执行超时*/
    SMLK_TSP_8F41_RESULT_EU_AUTH_FAIL,            /*电气单元认证失败*/
    SMLK_TSP_8F41_RESULT_LOCK_NOT_SP,             /*锁车功能终端不支持*/
    SMLK_TSP_8F41_RESULT_LOCK_ECU_NOT_SP,         /*锁车功能车辆ECU不支持*/
    SMLK_TSP_8F41_RESULT_CTRL_NOT_SP,             /*控车功能不支持*/
    SMLK_TSP_8F41_RESULT_ENGINE_NO_START,         /*发动机为启动无法执行*/
    SMLK_TSP_8F41_RESULT_PS_AUTH_FAIL,            /*PS防盗认证失败*/
    SMLK_TSP_8F41_RESULT_CMD_EXECUTING,           /*指令正在政令*/
    SMLK_TSP_8F41_RESULT_CMD_OVERDUE,             /*指令超时*/
    SMLK_TSP_8F41_RESULT_SIGNAL_NOT_SP,           /*远控条件不满足*/
    SMLK_TSP_8F41_RESULT_CTRL_FAIL,               /*控制失败*/
    SMLK_TSP_8F41_RESULT_RCTRL_NOT_SP,            /*车辆不可远程控制*/
    SMLK_TSP_8F41_RESULT_TIMING_ENGINE_STOP_SUCC, /*定时熄火成功*/
    SMLK_TSP_8F41_RESULT_EMERG_ENGINE_STOP_SUCC,  /*紧急熄火成功*/
    SMLK_TSP_8F41_RESULT_TIMING_ENGINE_STOP_FAIL, /*定时熄火失败*/
    SMLK_TSP_8F41_RESULT_EMERG_ENGINE_STOP_FAIL,  /*紧急熄火失败*/
    SMLK_TSP_8F41_RESULT_BATTERY_LOW,             /*蓄电池电量低*/
};
typedef struct
{
    SMLK_UINT8 version;
    SMLK_UINT8 time[GMT_TIME_LEN]; /*yy-mm-dd-hh-mm-ss*/
    SMLK_UINT8 cmd_num;
} __attribute__((__packed__)) Msg8F41_Head;

/*车辆控制消息0X8F41的单个指令的格式*/
typedef struct
{
    SMLK_UINT8 id;    /*功能编码*/
    SMLK_UINT8 cmd;   /*功能指令*/
    SMLK_UINT8 param; /*功能参数*/
} __attribute__((__packed__)) Msg8F41_Body;

/*MCU的命令和远控消息8f41,0f41中的numbering和subcmd_id的映射关系*/
/*m_mcu_to_jtt808_map AND m_jtt808_to_mcu_map*/

/*远控应答消息0x0F41的消息体头部格式*/
typedef struct
{
    SMLK_UINT8 version;
    SMLK_UINT8 time[GMT_TIME_LEN];
    SMLK_UINT16 seq_id;
    SMLK_UINT8 cmd_num;
} __attribute__((__packed__)) Msg0F41_Head;

/*远控应答消息0x0F41的指令格式*/
typedef struct
{
    SMLK_UINT8 id;     /*功能编码*/
    SMLK_UINT8 cmd;    /*功能指令*/
    SMLK_UINT8 result; /*执行状态*/
} __attribute__((__packed__)) Msg0F41_Body;

/*****************************************************************************
 *                               8F51消息部分                                 *
 *****************************************************************************/
/*远控从VehicleData转换成Mcu数据*/
typedef struct
{
    SMLK_UINT16 query_id; /*查询信号ID 包括VehicleData信号宏和内部定义宏SmlkRctrlInnerMsgID*/
    SMLK_DOUBLE value;    /*转换后的数值*/
} __attribute__((__packed__)) VehctrlGetCmd;
/*****************************************************************************
 *                               VehicleData                                 *
 *****************************************************************************/
#define RUC_REGULATION_TEMPERATURE_OFFSET 40
/*****************************************************************************
 *                               重传策略部分                                 *
 *****************************************************************************/

typedef struct
{
    SMLK_UINT16 errcode;         /*错误码*/
    SMLK_UINT16 err_retry_times; /*重试次数*/
} __attribute__((__packed__)) RemoteCtrlErrRetryInfo;

/*information in config file */
typedef struct
{
    SMLK_UINT16 cmd_id;                  /*命令ID*/
    std::vector<SMLK_UINT16> subcmd_vec; /*复合命令下的原子命令*/
    SMLK_UINT32 timieout;                /*超时时间*/
    SMLK_UINT8 max_send_times;           /*该命令最大重传次数*/
    SMLK_UINT8 retry_delay;              /*命令发送间隔*/
    std::vector<RemoteCtrlErrRetryInfo> retry_vec;
} RemoteCtrlConfigFileInfo;
typedef struct
{
    SMLK_DOUBLE data;                                  /*命令状态*/
    std::chrono::steady_clock::time_point report_time; /*状态变化稳定上报的时间*/
} ChangedStatusInfo;

typedef struct
{
    SMLK_UINT16 cmd_id;            /*命令ID*/
    ChangedStatusInfo change_info; /*状态变化信息*/
} CmdChangedStatus;

/*****************************************************************************
 *                              仪表盘保养信息                                 *
 *****************************************************************************/
/*仪表盘保养总成类型*/
enum class DashBoardMaintainType : SMLK_UINT8
{
    DB_MTN_RESERVED = 0,
    DB_MTN_ENGINE,          /*待保养发动机*/
    DB_MTN_GEAR_BOX,        /*待保养变速箱*/
    DB_MTN_REAR_AXLE,       /*待保养后桥*/
    DB_MTN_STEERING_ENGINE, /*待保养转向机*/
};
/*仪表盘保养要求*/
enum class DashBoardMaintainRequirement : SMLK_UINT8
{
    DB_MTN_REQMENT_NOTREQ = 0,
    DB_MTN_REQMENT_REQ,
};
/*仪表盘下发策略*/
enum class DashBoardMaintainSendStrategy : SMLK_UINT8
{
    DB_MTN_SNED_NONE = 0, /*无默认下发策略*/
    DB_MTN_SEND_DEFAULT,  /*默认下发策略:按照全F下发*/
    DB_MTN_SEND_SPECIFIC, /*针对下发策略:按照平台下发参数下发*/
};
/*****************************************************************************
 *                               PS上报信息                                   *
 *****************************************************************************/
/*ps IGN状态*/
enum class VehCtrlPsIgnPos : SMLK_UINT8
{
    VEH_CTRL_PS_IGN_OFF = 0,
    VEH_CTRL_PS_IGN_ACC = 1,
    VEH_CTRL_PS_IGN_ON = 2,
    VEH_CTRL_PS_IGN_START = 3,
};
/*ps 远程控制模式*/
enum class VehCtrlPsRemCtrlMode : SMLK_UINT8
{
    VEH_CTRL_PS_REM_CTRL_MODE_DE_ACTIVE = 0,
    VEH_CTRL_PS_REM_CTRL_MODE_ACTIVE = 1,
};

#endif
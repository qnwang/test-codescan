/*****************************************************************************/
/**
* \file       smlk_loan_common_def.h
* \author     wukai
* \date       2021/12/28
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    tsp public def
******************************************************************************/
#ifndef _LOAN_COMMON_PUBLIC_DEF_H
#define _LOAN_COMMON_PUBLIC_DEF_H
/*****************************************************************************
*                                头文件引用                                  *
*****************************************************************************/
#include <memory>
#include <vector>
#include <cstring>
#include <thread>
#include "smlk_error.h"
#include "smlk_types.h"
#include "smlk_spi_manager.h"
#include "tsp_service_api.h"
namespace smartlink
{
namespace LOAN
{

/*金融锁车功能使能禁止*/
enum
{
    LOAN_ACTION_FLOCK_FUNC_ENABLE =1, /*使能*/
    LOAN_ACTION_FLOCK_FUNC_DISABLE,/*禁止*/

};

/*金融锁车激活失活*/
enum
{
    LOAN_ACTION_FLOCK_ACTIVE =1, /*激活*/
    LOAN_ACTION_FLOCK_INACTIVE,/*失活*/

};

enum
{
    LOAN_ACTION_LOCK = 1,/*打开*/
    LOAN_ACTION_UNLOCK,/*关闭*/
};

enum
{
    LOAN_MCU_ACTION_OFF=1,/*关闭*/
    LOAN_MCU_ACTION_ON,/*打开*/
};

enum
{
    LOAN_ENTER_UDS_ERROR,/*进诊断失败*/
    LOAN_ENTER_UDS_SUCCESS,/*进诊断成功*/
    LOAN_NO_UDS,           /*不需要诊断：潍柴等*/
};

enum
{
    LOAN_CMD_FINANCIAL_LOCK_FUNC_ENABLE = 600,/*金融锁车功能使能，禁止,REMCTRL_ACTION_FLOCK_FUNC_ENABLE*/
    LOAN_CMD_FINANCIAL_LOCK_ACTIVE      = 601,/*金融锁车失活激活，REMCTRL_ACTION_FLOCK_ACTIVE*/
    LOAN_CMD_ROTATE_SPEED               = 602,/*转速限制: ON OFF, 且有值*/
    LOAN_CMD_TORQUE_LIMIT               = 603,/*扭矩限制 ON OFF, 且有值*/
    LOAN_CMD_FUEL_INJECT                = 604,/*喷油量限制 ON OFF, 且有值*/
    LOAN_CMD_ANTENNA_LOCK               = 605,/*天线开路锁车 ON OFF*/
    LOAN_RSP_FROM_MCU                   = 606,/*MCU 主动上报*/
    LOAN_RSP_CHECK_CODE                 = 607,/*握手校验消息*/
};

enum{

    LCKCAR_PROT_TYPE_QINGQI_MD5     = 1,    /*青汽 MD5 锁车*/ // 潍柴
    LCKCAR_PROT_TYPE_QINGQI_TSC1    = 2,    /*青汽 TSC1 锁车*/
    LCKCAR_PROT_TYPE_QINGQI_YUCHAI  = 3,    /*青汽玉柴锁车*/  // 玉柴
    LCKCAR_PROT_TYPE_YIQI_TSC1      = 6,    /*一汽 TSC1 锁车*/
    LCKCAR_PROT_TYPE_YUNNEI         = 7,    /*云内发动机锁车*/
    LCKCAR_PROT_TYPE_QUANCHAI       = 8,    /*全柴发动机锁车*/
    LCKCAR_PROT_TYPE_CUMMINS        = 9,    /*康明斯发动机锁车*/
};

enum class LoanStatusIgoffLckFlag : SMLK_UINT8{
    NEED_LCKCAR_WHEN_IGNON = 1,/*需要在igon时发送最后一次的0x8f40指令*/
    NOT_NEED_LCKCAR_WHEN_IGNON = 2,
};

/*对应0X8F40的控制指令的专用执行结果*/
enum{
    LOAN_CMD_RESULT_OK                          = 0x00,/*设置成功*/
    LOAN_CMD_RESULT_ERROR                       = 0x01,/*设置失败*/
    LOAN_CMD_RESULT_MSG_ERROR                   = 0x02,/*消息有误*/
    LOAN_CMD_RESULT_TIMEOUT                     = 0x03,/*设置超时*/
    LOAN_CMD_RESULT_VEHICLE_OFF                 = 0X09,/*车辆关机，激活失活失败码*/
    LOAN_CMD_RESULT_BUS_EXCEP                   = 0X0A,/*总线通讯异常*/
    LOAN_CMD_RESULT_CONTROLLER_ERROR            = 0X0B,/*控制器通信失败*/
    LOAN_CMD_RESULT_CHANGE_CONVERSATION_ERROR   = 0X0C,/*改变会话失败*/
    LOAN_CMD_RESULT_SECURE_ACCESS_ERROR         = 0X0D,/*安全访问失败*/
    LOAN_CMD_RESULT_WRITE_ERROR                 = 0X0E,/*写操作失败*/
    LOAN_CMD_RESULT_READ_ERROR                  = 0X0F,/*读操作失败*/
    LOAN_CMD_RESULT_PARTS_CTRL_FAIL             = 0X10,/*部件控制失败*/
    LOAN_CMD_RESULT_AGREEMENT_ERROR             = 0X11,/*31服务失败*/
    LOAN_CMD_RESULT_GPSID_ALL_F                 = 0X57,/*gpsID全0或全F*/
    LOAN_CMD_ACTIVE_LOCK_FAILED                 = 0X5C,/*激活失败*/
    LOAN_CMD_ACTIVE_LOCK_SUCCESS                = 0X5D,/*激活成功*/
    LOAN_CMD_CHECKCODE_FAIL                     = 0X5F,/*握手失败*/
    LOAN_CMD_LOCKCAR_SET_FAILED                 = 0X63,/*锁车设置失败*/
    LOAN_CMD_LOCKCAR_SET_SUCCESS                = 0X64,/*锁车设置成功*/
    LOAN_CMD_LOCKCAR_SUCCESS                    = 0X67,/*锁车成功*/
    LOAN_CMD_LOCKCAR_FAILED                     = 0X66,/*锁车失败*/
    LOAN_CMD_UNLOCKCAR_SET_FAILED               = 0X6A,/*解锁设置失败*/
    LOAN_CMD_UNLOCKCAR_SET_SUCCESS              = 0X6B,/*解锁设置成功*/
    LOAN_CMD_UNLOCKCAR_SUCCESS                  = 0X6E,/*解锁成功*/
    LOAN_CMD_UNLOCKCAR_FAILED                   = 0X6D,/*解锁失败*/
    LOAN_CMD_UNACTIVE_LOCK_FAILED               = 0X71,/*失活失败*/
    LOAN_CMD_UNACTIVE_LOCK_SUCCESS              = 0X72,/*失活成功*/
    LOAN_CMD_RESULT_GPSID_ERROR                 = 0XAA,/*gpsID不匹配*/
    LOAN_CMD_CHECKCODE_ERROR                    = 0XC2,/*握手故障*/
    LOAN_CMD_MCU_REPORT_KEY_ERROR               = 0XCA,/*ecu上报KEY不正确*/
    LOAN_CMD_ECU_GPSID_ERROR                    = 0XCB,/*ecu通知gpsID不匹配*/
    LOAN_CMD_REQ_OVERTIME                       = 0xCD,/*请求报文超时*/
    LOAN_CMD_HAND_SHAKE_OVERTIME                = 0xCE,/*校验超时*/
    LOAN_CMD_OTA                                = 0xF1,/*OTA升级中*/
    LOAN_CMD_RESULT_CHECKCODE_ERROR             = 0XFD,/*握手检验读取失败*/
};

enum{
    LOAN_MCU_REPORT_NONE = 0,/*无意义*/
    LOAN_MCU_REPORT_NORMAL = 1,/*ECU通信正常*/
    LOAN_MCU_REPORT_ABNORMAL = 2,/*ECU通信异常*/
    LOAN_MCU_REPORT_HAND_SHAKE_FAIL = 3,/*握手校验失败*/
    LOAN_MCU_REPORT_LOCK_SUCCESS = 4,/*锁车成功*/
    LOAN_MCU_REPORT_LOCK_FAIL = 5,/*锁车失败*/
    LOAN_MCU_REPORT_DISABLE = 6,/*ECU消贷未激活*/
    LOAN_MCU_REPORT_ENABLE = 7,/*ECU消贷激活未锁车*/
    LOAN_MCU_REPORT_LOCK = 8,/*ECU消贷激活且锁车*/
    LOAN_MCU_REPORT_GPSID_ERROR = 9,/*GPSID不匹配*/
    LOAN_MCU_REPORT_KEY_ERROR = 10,/*KEY不正确*/
    LOAN_MCU_REPORT_ECU_OVERTIME = 11,/*ECU上报校验超时*/
    LOAN_MCU_REPORT_REQ_OVERTIME = 12,/*请求报文超时*/
    LOAN_MCU_REPORT_UNLOCK_SUCC = 13,/*解锁成功*/
    LOAN_MCU_REPORT_UNLOCK_FAIL = 14,/*解锁失败*/
    LOAN_MCU_REPORT_INACTIVE_SUCC = 15,/*失活成功*/
    LOAN_MCU_REPORT_INACTIVE_FAIL = 16,/*失活失败*/
};

enum class Query0f40Result : SMLK_UINT8
{
    STATUS_NOT_ACTIVE_LOCK = 0X5A,        /*激活成功前反馈*/
    STATUS_ACTIVE_LOCK_SUCCESS = 0X5D,    /*激活成功*/
    STATUS_LOCK_VAHICLE_SUCCESS = 0X67,   /*锁车成功*/
    STATUS_UNLOCK_VAHICLE_SUCCESS = 0X6E, /*解锁成功*/
    STATUS_INACTIVE_LOCK_SUCCESS = 0X72,  /*失活成功*/

    /*add for 消贷状态事件上报中获取vin,ein等失败时，主动上报的错误码*/
    STATUS_VIN_READ_FAILED = 0XFA,           /*vin码读取失败*/
    STATUS_EIN_READ_FAILED = 0XFB,           /*ein码读取失败*/
    STATUS_FINACIAL_FLAG_READ_FAILED = 0XFC, /*消贷标志位读取失败*/
    STATUS_HANDSHAKE_FAILED = 0XFD,          /*握手校验失败*/
    STATUS_INVALID = 0XFF,                   /*无效*/
};

enum class RctrlStatusFinaLck : SMLK_UINT8{
    STATUS_NOT_ENABLE = 1,/*金融锁车功能未使能*/
    STATUS_NOT_ACTIVE = 2,/*未激活,金融锁车功能已使能*/
    STATUS_IS_ACTIVE = 3,/*已激活未锁车*/
    STATUS_IS_LOCKED = 4,/*已激活已锁车*/
};

enum class GetVinEinFlagResult : SMLK_UINT8{
    GET_ECU_INFO_OK = 0,
    GET_ECU_VIN_FAILED,
    GET_ECU_EIN_FAILED,
    GET_ECU_LAON_FLAG_FAILED,
    GET_ECU_VIN_EIN_FAILED,
    GET_ECU_VIN_LAONFLAG_FAILED,
    GET_ECU_EIN_LAONFLAG_FAILED,
    GET_ECU_FAILED,
    ECU_HANDSHAKE_FAILED,
};

static const int            SL_QUEUE_GET_TIMEOUT_MS     = 200;
#define     IGN_OFF     0
#define     IGN_ON      1
#define DEFALUT_TIMEOUT 6
#define GPSID_LEN 3
#define FIXED_KEY_LEN 3
#define VIN_LENGTH  17
#define EIN_LENGTH  17

#define JTT808_FINANCIAL_LOCK_CTRL  0x0D/*金融锁车功能打开/关闭，实现*/

#define JTT808_COMMON_RESPONSE          0X0001  /*终端通用应答*/
#define JTT808_REMOTE_LOCK              0X8F40  /*金融锁车消息ID*/
#define JTT808_REMOTE_LOCK_RESP         0X0F40  /*金融锁车应答*/
#define JTT808_DRIVE_EVENT              0X0f3d  /*驾驶行为事件*/

#define LOAN_ACTION_INVALID      0XFF/*动作无效*/
#define LOAN_DATA_INVALID        0XFFFF/*数值无效*/

#define KEY1   0x0457
#define KEY2   0x0456
#define KEY3   0x044C
#define KEY4   0x03EB
#define KEY5   0x0000

#define BYTE0 0x1000000
#define BYTE1 0x10000
#define BYTE2 0x100

#define AND0  0xff000000
#define AND1  0xff0000
#define AND2  0xff00
#define AND3  0xff


/*短信消贷*/
#define     TEXT_MSG_FINANCIAL_LCK_FUNCTION_OPEN        "99982101CLCKON"/*打开金融锁车功能*/
#define     TEXT_MSG_FINANCIAL_LCK_FUNCTION_CLOSE       "99982101CLCKOFF"/*关闭金融锁车功能*/
#define     TEXT_MSG_FINANCIAL_LCK_ACTIVE               "99982101FINLOCK2,80"/*激活*/
#define     TEXT_MSG_FINANCIAL_LCK_INACTIVE             "99982101FINLOCK2,81"/*失活*/
#define     TEXT_MSG_FINANCIAL_LCK_LOCK                 "99982101FINLOCK2,82"/*锁车*/
#define     TEXT_MSG_FINANCIAL_LCK_UNLOCK               "99982101FINLOCK2,83"/*解锁*/
#define     TEXT_MSG_FINANCIAL_LCK_STATUS_QUERY         "99982101FINLOCK1,4"/*查询金融锁车状态*/


/*远控消贷需要从依赖模块获取的信息*/
typedef struct {
    SMLK_UINT32 latitude;  /*纬度*/
    SMLK_UINT32 longitude; /*经度*/
    SMLK_UINT16 elevation; /*高程*/
    SMLK_UINT16 direction; /*方向*/
    SMLK_UINT8 time[6];    /*事件*/
    SMLK_UINT8 speed;      /*速度*/

    SMLK_UINT8 vin[VIN_LENGTH];   /**/
    SMLK_UINT8 ein[EIN_LENGTH];   /**/
    SMLK_UINT8 lck_active_status; /*消贷标志位: 1: */
    SMLK_UINT8 tbox4G_status;
} __attribute__((__packed__)) LoanLckSatatusInfo;

/*发给MCU service控制命令格式*/
typedef struct
{
   SMLK_UINT16   cmd_id;
   SMLK_UINT8    action;
   SMLK_UINT8    data_len;/*数据长度*/
}__attribute__((__packed__))LoanCtrlMcuCmd;

/*远控设置命令格式*/
typedef struct
{
    SMLK_UINT16   cmd_id;/*实际下发的命令Id*/
    SMLK_UINT8    action;/*动作：0xff表示动作无效*/
    SMLK_UINT8    data_len;/*数据长度*/
    SMLK_UINT16   data;/*命令内容*/
}__attribute__((__packed__))LoanSetMcuCmd;

/*发给MCU service命令头部字段*/
typedef struct
{
    SMLK_UINT16 seq;  /*sequence number*/
    SMLK_UINT8 count;/*命令个数*/
}__attribute__((__packed__))LoanMcuCmdHead;

typedef struct
{
  SMLK_UINT32               msg_id;       // 消息ID
  SMLK_UINT32               seq_id;        // 消息序号
  smartlink::ProtoclID      protocol;        /*消息体采用协议类型         0x01: 808, 0x02: MQTT, 0x03: http*/
  SMLK_UINT8                qos;
  SMLK_UINT8                priority;
  SMLK_UINT32               reserved_dw;     /* 保留字段，扩展用*/
}__attribute__((__packed__))LoanGeneralHead;

/*jtt808 common response*/
typedef struct
{
    SMLK_UINT16 seq_id;/*对应平台消息的流水号*/
    SMLK_UINT16  msg_id;/*对应平台消息de ID*/
    SMLK_UINT8 result;
}__attribute__((__packed__))Jtt808CommResp;

/*事件位置信息*/
typedef struct 
{
    SMLK_UINT8  version;/*版本*/
    SMLK_UINT32 start_latitude;/*起始点纬度*/
    SMLK_UINT32 start_longitude;/*起始点经度*/
    SMLK_UINT16 start_elevation;/*起始点高程*/
    SMLK_UINT16 start_direction;/*起始点方向*/
    SMLK_UINT8  start_time[6];/*起始点事件*/
    SMLK_UINT8 start_speed;/*起始点速度*/

    SMLK_UINT32 end_latitude;/*结束点纬度*/
    SMLK_UINT32 end_longitude;/*结束点经度*/
    SMLK_UINT16 end_elevation;/*结束点高程*/
    SMLK_UINT16 end_direction;/*结束点方向*/
    SMLK_UINT8  end_time[6];/*结束点事件*/
    SMLK_UINT8 end_speed;/*结束点速度*/
}__attribute__((__packed__))EventLocationInfo;

/*驾驶行为事件公共部分*/
typedef struct 
{
    EventLocationInfo event_location;
    SMLK_UINT16       event_lenth;

}__attribute__((__packed__))Msg0f3dEventCommon;
#define      MSGOF3D_EVENT_COMMON_LEN   sizeof(Msg0f3dEventCommon)

/*消贷锁车状态*/
typedef struct 
{
    SMLK_UINT8  event_id;/*事件Id*/
    SMLK_UINT8  vin[17];/*起始点纬度*/
    SMLK_UINT8  ein[17];/*起始点经度*/
    SMLK_UINT8  lck_flag;/*消贷标志位*/
    SMLK_UINT8  tbox4G;/*tbox4G天线上报*/
}__attribute__((__packed__))LckStatusEvent;
#define LCK_STATUS_EVENT_ID     43
#define LCK_STATUS_EVENT_LEN  sizeof(LckStatusEvent)

#define LCK_FLAG_ACTIVE_LOCK_SUCC   0Xa2    /*激活锁车成功*/
#define LCK_FLAG_ACTIVE_SUCC        0Xa1    /*激活成功*/
#define LCK_FLAG_ACTIVE_FAILED      0Xa0    /*激活失败*/

typedef struct
{
    SMLK_UINT16   cmd_id;/*实际下发的命令Id*/
    SMLK_UINT8    action;/*动作：0xff表示动作无效*/
    SMLK_UINT16   data;/*命令内容*/
}__attribute__((__packed__))LoanSetCmd;

typedef struct
{
    SMLK_UINT16   cmd_id;/*实际下发的命令Id*/
    SMLK_UINT8    action;/*动作：0xff表示动作无效*/
    SMLK_UINT8    result;/*当前命令的响应结果*/
}__attribute__((__packed__))LoanSetCmdResult;

typedef struct
{
    SMLK_UINT8 lock_prot; /*锁车协议类型*/
    SMLK_UINT8 subcmd;    /*子命令*/
} __attribute__((__packed__)) Msg8f40Common;

/*失活激活消息格式*/
typedef struct
{
//  Jtt808Msg8f40Common common;
    SMLK_UINT16 reserved;
    SMLK_UINT8 gpsid[GPSID_LEN];   /*MD5时有效，平台唯一*/
    SMLK_UINT8 key[FIXED_KEY_LEN]; /*MD5时有效，平台唯一*/
} __attribute__((__packed__)) Msg8f40Active;

/*锁车解锁消息格式*/
typedef struct
{
    //  Jtt808Msg8f40Common common;
    SMLK_UINT16 type; /*命令类型：转数/扭矩/喷油量限制*/
    SMLK_UINT8 reserved[3];
    SMLK_UINT8 gpsid[3];
    SMLK_UINT16 param;
} __attribute__((__packed__)) Msg8f40Lock;

/*手工钥匙下发消息格式*/
typedef struct
{
    //  Jtt808Msg8f40Common common;
    SMLK_UINT16 key_01;
    SMLK_UINT16 key_02;
    SMLK_UINT16 key_03;
    SMLK_UINT16 key_04;
    SMLK_UINT8 reserved[8];
} __attribute__((__packed__)) Msg8f40Keys;

/*锁车状态查询应答*/
typedef struct
{
    SMLK_UINT16 seq_id;     /*应答流水号*/
    Query0f40Result result; /*执行结果*/
    SMLK_UINT8 lock_prot;   /*锁车协议类型*/
    SMLK_UINT8 resp_cmd;    /*响应命令*/
    SMLK_UINT8 gpsid[GPSID_LEN];
    SMLK_UINT8 key[FIXED_KEY_LEN];
} __attribute__((__packed__)) Msg0f40Query;

/*激活失活，锁车解锁应答消息格式*/
typedef struct
{
    SMLK_UINT16 seq_id;   /*应答流水号*/
    SMLK_UINT8 result;    /*执行结果*/
    SMLK_UINT8 lock_prot; /*锁车协议类型*/
    SMLK_UINT8 resp_cmd;  /*响应命令*/
} __attribute__((__packed__)) Msg0f40Resp;

enum class LoanCmdType : SMLK_UINT8
{
    LOAN_CMD_SET = 1,
    LOAN_CMD_GET = 2
};

enum
{
    LOAN_KEY_GET = 0,
    LOAN_MSG_GET = 1
};


/*0：成功/确认；1：失败；2：超出范围；3：不支持*/
enum{JTT808_MSG_SUCCESS,JTT808_MSG_FAILED,JTT808_MSG_OUT_OF_RANGE,JTT808_MSG_NOT_SUPPORT};/*result的值*/

enum class LoanCmdFrom : SMLK_UINT8
{
    LOAN_FROM_TSP_MODULE = 1,
    LOAN_FROM_TEXT_MESSAGE = 2,
    LOAN_FROM_OFFLINE_MESSAGE = 3
};

enum
{
    LCK_STATUS_ACTIVE_LOCK  = 0xa2,     /*消贷标志位 已激活且锁车*/
    LCK_STATUS_ACTIVE       = 0xa1,     /*消贷标志位 已激活未锁车*/
    LCK_STATUS_INACTIVE     = 0xa0,     /*消贷标志位 未激活*/
};

class LoanCmdMsg{

    public:
        LoanCmdMsg(){};
        virtual ~LoanCmdMsg(){};
    public:
        LoanGeneralHead m_head;/*如果是tsp下发的，那么就是从ipc头中取出来的数据*/
        LoanCmdFrom m_cmd_from;/*命令来自哪里：TSP 还是 短信*/
        LoanCmdType m_cmd_type;/*REMCTRL_CMD_SET or REMCTRL_CMD_GET*/
        // SMLK_UINT8 m_version;/*用于0x8f41*/
        SMLK_UINT8 m_lock_prot;/*用于0x8f40:锁车协议类型， LCKCAR_PROT_TYPE_QINGQI_MD5*/
        LoanSetCmd m_cmd_vec;
};

class LoanCtrlResult{

    public:
        LoanCtrlResult(){};
        virtual ~LoanCtrlResult(){};
    public:
        LoanGeneralHead m_head;/*其中的msg_id为真正要发送的msgid，其他信息与tsp下发的ipc头部信息一样*/
        LoanCmdFrom m_cmd_from;/*命令来自哪里：TSP 还是 短信*/
        LoanCmdType m_result_type;
        // SMLK_UINT8 m_version;
        SMLK_UINT8 m_lock_prot;
        LoanSetCmdResult m_result_vec;
};

class LoanCmdData{
public:
    LoanCmdData(){};
    ~LoanCmdData(){};
public:
    LoanGeneralHead m_head;/*如果是tsp下发的，那么就是从ipc头中取出来的数据*/
    LoanCmdFrom m_cmd_from;/*命令来自哪里：TSP 还是 短信*/
    LoanCmdType m_cmd_type;/*LOAN_CMD_SET or LOAN_CMD_GET*/
    SMLK_UINT8 m_lock_prot;/*用于0x8f40:锁车协议类型， LCKCAR_PROT_TYPE_QINGQI_MD5*/
    // LoanAction m_cmd_act;
    std::vector<SMLK_UINT8> m_message;/*tsp发过来的消息体*/

    LoanCmdData& operator=(const LoanCmdData& other)
    {
        if (this == &other)
            return *this;
        memcpy(&m_head, &other.m_head, sizeof(LoanGeneralHead));
        m_cmd_from = other.m_cmd_from;
        m_cmd_type = other.m_cmd_type;
        m_lock_prot = other.m_lock_prot;
        // memcpy(&m_cmd_act, &other.m_cmd_act, sizeof(LoanAction));
        memcpy(&m_message, &other.m_message[0], m_message.size());
        return *this;
    }
};

/*发动机类型*/
enum class LoanEngineId : SMLK_UINT8
{
    LOAN_ENGINE_EMS_ECCONTROL = 1,
    LOAN_ENGINE_EMS_G6_CGN ,/*国六天然气*/
    LOAN_ENGINE_EMS_G6,/*国六自主*/
    LOAN_ENGINE_FAW_HCU,/*HCU*/
    LOAN_ENGINE_FAW_VCU,/*VCU*/
    LOAN_ENGINE_EMS_CUMMINS_G6,      // 102A 06: 康明斯国六柴油机
    LOAN_ENGINE_EMS_CUMMINS_G6_CGN,  // 102A 07: 康明斯国六天然气
    LOAN_ENGINE_EMS_WEICAI_G6,       // 102A 08: 潍柴国六柴油机
    LOAN_ENGINE_EMS_WEICHAI_G6_CGN,  // 102A 09: 潍柴国六天然气
    LOAN_ENGINE_EMS_YUCHAI_G6,       // 102A 0A: 玉柴国六柴油机
    LOAN_ENGINE_EMS_YUCHAI_G6_CGN,   // 102A 0B: 玉柴国六天然气
    LOAN_ENGINE_EMS_QUANCHAI_G6,     // 102A 0C: 全柴国六柴油机
    LOAN_ENGINE_EMS_YUNNEI_G6,       // 102A 0D: 云内国六柴油机
};
}
}

#endif // _LOAN_COMMON_PUBLIC_DEF_H
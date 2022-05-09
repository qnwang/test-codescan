/*****************************************************************************/
/**
 * \file       vehctrl_jtt808_msg8f51.h
 * \author     huangxin
 * \date       2020/11/25
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_JTT808_MSG8F51_H_
#define _VEHICLE_JTT808_MSG8F51_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
/*远控状态查询子指令对应的信号数量*/
#define GMT_TIME_LEN1 6
#define MSG0F51_DOOR_STA_LEN 2
#define MSG0F51_ENGINE_MODE_LEN 1
#define MSG0F51_POWER_MODE_LEN 1
#define MSG0F51_ORI_AC_LEN 9
#define MSG0F51_INDENPT_WARMAIR_LEN 2
#define MSG0F51_PARKING_AC_LEN 4
#define MSG0F51_OILTANK_LEN 1
#define MSG0F51_WAKEUPSOURCE_LEN 1
#define MSG0F51_INTELLIGENT_RUC_LEN 5
#define MSG0F51_PS_LEN 3
namespace smartlink
{
  /*****************************************************************************
   *                                  通用部分                                  *
   *****************************************************************************/
  enum class Msg0f51Flag : SMLK_UINT8
  {
    QUERY_RESP = 0,   /*获取车辆状态应答*/
    EVENT_REPORT = 1, /*主动事件上报*/
  };

  typedef struct
  {
    SMLK_UINT8 version; /*应答填充8F51下发的版本,主动上报填0*/
    SMLK_UINT8 time[GMT_TIME_LEN1];
    Msg0f51Flag flag;
    SMLK_UINT16 seq_id;
    SMLK_UINT8 query_num;
  } __attribute__((__packed__)) Msg0F51Head;

  typedef struct
  {
    SMLK_UINT8 version;
    SMLK_UINT8 time[6];
    SMLK_UINT8 cmd_num;
  } __attribute__((__packed__)) Msg8f51Head;

  enum Jtt808RemoteCtrlQueryCmd
  {
    JTT808_QUERY_DOOR = 0x01,             /*获取车门状态 支持*/
    JTT808_QUERY_WINDOW = 0x02,           /*获取车窗状态*/
    JTT808_QUERY_SUNROOF = 0x03,          /*获取天窗状态*/
    JTT808_QUERY_LIGHT = 0x04,            /*获取灯状态*/
    JTT808_QUERY_FINANCIAL_LOCK = 0x05,   /*获取金融锁车状态*/
    JTT808_QUERY_ENGINE = 0x06,           /*获取发动机状态 支持*/
    JTT808_QUERY_POWER = 0x07,            /*获取车辆电源状态 支持*/
    JTT808_QUERY_WIFI = 0x08,             /*获取wifi状态*/
    JTT808_QUERY_ORIGINAL_AC = 0x09,      /*获取原车空调状态 支持*/
    JTT808_QUERY_WIPER = 0x0A,            /*获取雨刮状态*/
    JTT808_QUERY_REARVIEW_MIRROR = 0x0B,  /*获取后视镜状态*/
    JTT808_QUERY_WATER_HEATING = 0x0C,    /*获取车辆水暖ECU状态*/
    JTT808_QUERY_LICENCE = 0x0D,          /*获取预见性巡航licence校验状态*/
    JTT808_QUERY_LOCKCAN = 0x0E,          /*获取LOCKCAN状态*/
    JTT808_QUERY_MULTIMEDIA = 0x0F,       /*获取多媒体信息状态*/
    JTT808_QUERY_INDENTP_WARM_AIR = 0x10, /*获取独立暖风状态 支持*/
    JTT808_QUERY_PARKING_AC = 0x11,       /*获取驻车空调状态 支持*/
    JTT808_QUERY_OILTANK_SECURITY = 0x12, /*获取油箱报警状态 支持*/
    JTT808_QUERY_WAKEUP_SOURCE = 0x25,    /*获取唤醒源 0F51主动上报*/
    JTT808_QUERY_INTELLIGENT_RUC = 0x26,  /*获取智能冷机状态*/
    JTT808_QUERY_CURRENT_PS = 0x27,       /*获取当前ps状态*/
  };

  enum Query_Door_Status : SMLK_UINT8
  {
    /*车门状态内容长度2*/
    /*第0字节: (主驾驶门)bit0: 0-关闭 1-打开  bit1: 0-无效 1-有效  (副驾驶门)bit2: 0-关闭 1-打开  bit3: 0-无效 1-有效 */
    /*第1字节: bit0~bit5: 保留 (中控锁)bit6: 0-关闭 1-打开  bit7: 0-无效 1-有效 */
    DOOR_INVALID = 0,
    DOOR_UNLOCK = 2,
    DOOR_LOCK = 3
  };
  /*****************************************************************************
   *                              0F51响应消息结构                               *
   *****************************************************************************/
  /*发动机模式,电源模式,唤醒源,油箱防盗开关*/
  typedef struct
  {
    SMLK_UINT8 query_id;
    SMLK_UINT8 length;
    SMLK_UINT8 status;
  } __attribute__((__packed__)) Msg0f51CommonStruct;
  /*车门状态*/
  typedef struct
  {
    SMLK_UINT8 query_id;
    SMLK_UINT8 length;
    SMLK_UINT8 driver_door : 2;
    SMLK_UINT8 passenger_door : 2;
    SMLK_UINT8 reserved1 : 4;
    SMLK_UINT8 reserved2 : 6;
    SMLK_UINT8 centor_lock : 2;
  } __attribute__((__packed__)) Msg0f51Door;
  /*原车空调状态*/
  typedef struct
  {
    SMLK_UINT8 query_id;
    SMLK_UINT8 length;
    SMLK_UINT8 ac_switch_status;  /*空调开关状态*/
    SMLK_UINT8 ac_airoutlet_mode; /*出风模式*/
    SMLK_UINT8 ac_temper;         /*当前设置的温度*/
    SMLK_UINT8 ac_wind;           /*风量*/
    SMLK_UINT8 ac_loop_mode;      /*循环模式*/
    SMLK_UINT8 ac_compressor;     /*压缩机状态*/
    SMLK_UINT8 ac_auto_mode;      /*空调AUTO开关状态*/
    SMLK_UINT8 ac_ventilation;    /*一键通风状态*/
    SMLK_UINT8 ac_defrost;        /*强制除霜状态*/
  } __attribute__((__packed__)) Msg0f51OriginalAC;
  /*驻车空调状态*/
  typedef struct
  {
    SMLK_UINT8 query_id;
    SMLK_UINT8 length;
    SMLK_UINT8 switch_status;
    SMLK_UINT8 winder;
    SMLK_UINT8 temper;
    SMLK_UINT8 auto_switch_status;
  } __attribute__((__packed__)) Msg0f51ParkingAC;
  /*独立暖风状态*/
  typedef struct
  {
    SMLK_UINT8 query_id;
    SMLK_UINT8 length;
    SMLK_UINT8 switch_status;
    SMLK_UINT8 temper;
  } __attribute__((__packed__)) Msg0f51IndenptWarmAir;
  /*智能冷机状态*/
  typedef struct
  {
    SMLK_UINT8 query_id;
    SMLK_UINT8 length;
    SMLK_UINT8 rctrl_allowed_state;
    SMLK_UINT8 ruc_switch_state;
    SMLK_UINT8 defrost_allowed_state;
    SMLK_UINT8 defrost_work_mode;
    SMLK_UINT8 ruc_regulation_temper;
  } __attribute__((__packed__)) Msg0f51IntelligentRUC;

  class DissectorMsg8F51 : public IDissector
  {
  public:
    DissectorMsg8F51();
    virtual ~DissectorMsg8F51();

  public:
    virtual SMLK_RC ResultEncode(IN RctrlResultQueue &);
    virtual SMLK_RC Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &, OUT SMLK_UINT8 &is_encode);
    SMLK_RC OnChanged(IN SMLK_UINT8 &, IN SMLK_UINT16 &, IN SMLK_DOUBLE &);

  private:
    /*消息来源:Vehicle Data*/
    SMLK_RC EncodeDoorStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);
    SMLK_RC EncodeOriginalACStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);
    SMLK_RC EncodeIndentpWarmAirStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);
    SMLK_RC EncodeParkingAcStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);
    SMLK_RC EncodeOilTankStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);
    SMLK_RC EncodeEngineModeStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);
    SMLK_RC EncodeIntelligentRucStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);
    SMLK_RC EncodeCurrentPsStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);
    /*消息来源:MCU*/
    SMLK_RC EncodePowerModeStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);
    SMLK_RC EncodeWakeupSourceStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT std::vector<SMLK_UINT8> &output_vec);

  private:
    SMLK_UINT16 m_seq_id;
  };
};
#endif

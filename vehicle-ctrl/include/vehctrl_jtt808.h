/*****************************************************************************/
/**
 * \file       remote_control_jt808.h
 * \author     huangxin
 * \date       2020/10/27
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_CONTROL_JT808_H_
#define _VEHICLE_CONTROL_JT808_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <iostream>
#include <vector>
#include <memory>
#include <map>
#include "smlk_error.h"
#include "smlk_types.h"
#include "vehctrl_general_definition.h"
#include "vehctrl_queue.h"
#include "vehctrl_inf.h"
#include "smlk_tools.h"

namespace smartlink
{
/*****************************************************************************
 *                                宏定义                                   *
 *****************************************************************************/
#define MAX_OUTPUT_LEN 1024

/*tsp发过来的消息ID,IPC头部MsgHead中的id*/
#define JTT808_COMMON_RESPONSE 0X0001  /*终端通用应答*/
#define JTT808_MSG_REMOTE_CTRL 0X8F41  /*远控消息ID*/
#define JTT808_REMOTE_CTRL_RESP 0X0F41 /*远控消息ID的响应消息*/

#define JTT808_MSG_VEHICAL_QUERY 0X8F51  /*车辆状态查询*/
#define JTT808_VEHICAL_QUERY_RESP 0X0F51 /*查询车辆状态应答*/

#define JTT808_GET_SET_TBOX 0X8F42      /*查询,设置TBOOX信息*/
#define JTT808_GET_SET_TBOX_RESP 0X0F42 /*查询,设置TBOOX信息的应答*/

#define JTT808_TERMINAL_PARAM_SET 0X8103           /*设置终端参数*/
#define JTT808_TERMINAL_PARAM_GET 0X8104           /*查询终端参数*/
#define JTT808_TERMINAL_SPECIFIED_PARAM_GET 0X8106 /*查询指定终端参数*/
#define JTT808_TERMINAL_PARAM_GET_RESP 0X0104      /*查询终端参数应答*/

#define MCU_REMOTE_CTRL_RESP 0X2 /*mcu返回给远控的result信息*/
#define MCU_REMOTE_CTRL_MSG 0X3  /*远控给他MCU的命令消息*/

  /*****************************************************************************
   *                             结构体定义                               *
   *****************************************************************************/
  typedef struct
  {
    SMLK_UINT16 seq_id;
    SMLK_UINT8 result;
    SMLK_UINT8 prot;
    SMLK_UINT8 cmd;
  } Jtt808Msg0f40Lock;
  enum
  {
    ENABLE_LOCK_RESP = 1,
    DISABLE_LOCK_RESP = 2,
    LOCK_VEHICLE_RESP = 3,
    UNLOCK_VEHICLE_RESP = 4,
    QUERY_STATUS = 10
  };

  typedef struct
  {
    SMLK_UINT16 seq_id;
    SMLK_UINT8 result;
    SMLK_UINT8 prot;
    SMLK_UINT8 cmd;
    SMLK_UINT8 gpsid[3];
    SMLK_UINT8 key[3];
  } Jtt808Msg0f40Query;

  enum
  {
    LOCK_VAHICLE_SUCCESS = 1,
    LOCK_VAHICLE_FAILED = 2,
    MSG_ERROR = 3,
    ECU_NOT_SUPPORT = 4,
    ECU_SUCCESS = 5,
    ECU_FAILED = 6,
  };

  /*usefull infomation in jtt808 message, corresponding to remote control inner head */
  typedef struct
  {
    SMLK_UINT8 version; /*parm in msg */
    SMLK_UINT8 reserved[3];
  } __attribute__((__packed__)) RemoteCtrlJtt808Head;

  /*jtt808 common response*/
  typedef struct
  {
    SMLK_UINT16 seq_id; /*对应平台消息的流水号*/
    SMLK_UINT16 msg_id; /*对应平台消息de ID*/
    SMLK_UINT8 result;
  } __attribute__((__packed__)) Jtt808CommResp;

  class DissectorJtt808 : public IDissector
  {
  public:
    DissectorJtt808();
    virtual ~DissectorJtt808();

  public:
    virtual SMLK_RC OnChanged(IN SMLK_UINT8 &, IN SMLK_UINT16 &, IN SMLK_DOUBLE &);
    virtual SMLK_RC ResultEncode(IN RctrlResultQueue &);
    virtual SMLK_RC Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &, OUT SMLK_UINT8 &is_encode);
    void RegisterCB(IN VecCtrlInnerCallBack &cb);

  public:
    const static std::map<SMLK_UINT16, std::shared_ptr<IDissector>> m_map_msgdis;
    const static std::map<SMLK_UINT16, std::shared_ptr<IDissector>> m_map_msgchange;
  };

  class DissectorJtt808Common
  {
  public:
    static DissectorJtt808Common *getInstance()
    {
      static DissectorJtt808Common ins;
      return &ins;
    };

  public:
    DissectorJtt808Common(){};
    ~DissectorJtt808Common(){};
    /*参数rctrl_head中*/
    SMLK_RC DoSendMsgToTsp(IN RctrlHead &rctrl_head, IN SMLK_UINT8 *msg_body, IN std::size_t &length);
    SMLK_RC SendMsgCommonResp(IN RctrlHead &head, IN SMLK_UINT8 &result);

  private:
    void GetIpcHeadFromRctrlHead(IN RctrlHead &rctrl_head, OUT IpcTspHead &ipc_head);
  };
};

#endif

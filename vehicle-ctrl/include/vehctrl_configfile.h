/*****************************************************************************/
/**
 * \file       vehctrl_configfile.h
 * \author     huangxin
 * \date       2020/11/03
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_CTRL_CONFIG_H_
#define _VEHICLE_CTRL_CONFIG_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <iostream>
#include <vector>
#include <map>
#include "smlk_types.h"
#include "vehctrl_general_definition.h"

namespace smartlink
{
#define CONFIGFILE_NAME "configfile"

#define DEFAULT_TIMEOUT_NAME "default_timeout"
#define DEFAULT_RETRYTIMES_NAME "default_retrytimes"
#define DEFAULT_RETRYDELAY_NAME "default_retrydelay"
#define DEFAULT_REPORTDELAY_NAME "default_reportdelay"

#define CMD_NAME "cmd"
#define SUBCMD_NAME "subcmd"

#define TIMEOUT_NAME "timeout"
#define MAX_SENDTIMES_NAME "max_sendtimes"
#define RETRYDELAY_NAME "retrydelay"

#define PARAMS_NAME "params"

#define ERRCODE_NAME "errcode"
#define RETRYTIMES_NAME "retrytimes"

#define LOAN_READ_SERVICE_ID 0x22 /*读serviceID*/
#define LOAN_DID_INVALID 0xffff   /*无效did*/
#define LOAN_DID_LENGTH 2         /*did的长度*/

#define LOAN_DM1_VCU_ID 0x18FECA27             /*识别源地址 0x18FECA27（DM1_VCU）报文方式进行识别*/
#define PROP_DASHBOARD_MAINT_CAN_ID 0x18FC174A /*仪表盘保养信息*/

  /** 远控配置加载类 */
  class RemoteCtrlCmdLoad
  {
  public:
    RemoteCtrlCmdLoad();
    virtual ~RemoteCtrlCmdLoad();

  public:
    SMLK_RC ReadCmdConfigFile();
    SMLK_RC GetVehicleTypeFileScript(std::string &file_loction);

  public:
    std::map<SMLK_UINT16, RemoteCtrlConfigFileInfo> m_config_file_map;
    SMLK_UINT16 m_defalut_timeout;
    SMLK_UINT8 m_defalut_retrytimes;
    SMLK_UINT8 m_defalut_retrydelay;
    SMLK_UINT8 m_report_delay;   /*主动上报状态更新保持时间*/
    SMLK_UINT8 m_LockWake_delay; /*短信唤醒电源锁 保持时间*/
    void *m_module_handle;
  };

};

#endif

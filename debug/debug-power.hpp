/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-location.hpp
*  Description:  debug locaiton 头文件
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/

#ifndef DEBUG_POWER_HPP_
#define DEBUG_POWER_HPP_

#include "debug-cmd.hpp"

#include "smartlink_sdk_sys_power.h"

using namespace smartlink_sdk;
/*
 * @Description: 该类的功能描述
 */
class DebugPower
{
  public:
    DebugPower();
    virtual ~DebugPower();

    void CmdHelp();
    void CmdQuit();
    void CmdExit();

    void CmdGet(int argc, char *argv[]);
    void CmdSet(int argc, char *argv[]);
    void CmdstartEvent();
    void CmdstopEvent();

    DebugCmdLine *GetCmdTable();

    void Init();

    void CmdIsReady();
    void  power_recv_cb(SysPowerEventID id, void * data, int len);


  private:
    static DebugCmdLine power_table[];
    static bool m_event;

};

#endif /* DEBUG_LOCATION_HPP_ */

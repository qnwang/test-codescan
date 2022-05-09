/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-location.hpp
*  Description:  debug locaiton 头文件
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/

#ifndef DEBUG_TEL_HPP_
#define DEBUG_TEL_HPP_

#include "debug-cmd.hpp"

#include "smartlink_sdk_tel.h"

using namespace smartlink_sdk;
/*
 * @Description: 该类的功能描述
 */
class DebugTel
{
  public:
    DebugTel();
    virtual ~DebugTel();

	void CmdHelp();
	void CmdQuit();
	void CmdExit();

	void CmdGet(int argc, char *argv[]);
	void CmdSet(int argc, char *argv[]);
	void CmdstartEvent();
	void CmdstopEvent();

    DebugCmdLine *GetCmdTable();

	void Init();

    RtnCode GetIMEI();

    //获取SIM卡IMSI值
    RtnCode GetIMSI();
    
    //获取SIM卡ICCID值
    RtnCode GetICCID();

    void  tel_recv_cb(TelEventId id, void * data, int len);

  private:
    static DebugCmdLine tel_table[];
	static bool m_event;

};

#endif /* DEBUG_LOCATION_HPP_ */

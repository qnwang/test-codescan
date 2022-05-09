/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-location.hpp
*  Description:  debug locaiton 头文件
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/

#ifndef DEBUG_LOCATION_HPP_
#define DEBUG_LOCATION_HPP_

#include "debug-cmd.hpp"

#include "smartlink_sdk_location.h"

using namespace smartlink_sdk;
/*
 * @Description: 该类的功能描述
 */
class DebugLocation
{
  public:
    DebugLocation();
    virtual ~DebugLocation();

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
	
	/*获取GNSS模组的软件版本号*/
	void CmdGetGnssSwVersion();
	
        /*设置定位频率*/
	void CmdGetfrequency();
	
	/*获取定位频率*/
	void CmdSetfrequency();
	
	/*获取天线状态*/
	void CmdGetGnssAntennaState();
	
	
	/*主动获取定位信息，如果定位无效，返回最后一次有效定位，同时定位标志置为无效*/
	void CmdGetLocation();
	
	/*获取卫星信息*/
	void CmdGetSatelliteInfo();
	
	void  gnss_recv_cb(LocationEventId id, void * data, int len);


  private:
    static DebugCmdLine loc_table[];
	static bool m_event;

};

#endif /* DEBUG_LOCATION_HPP_ */

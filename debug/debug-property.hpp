/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-Property.hpp
*  Description:  debug locaiton 头文件
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/

#ifndef DEBUG_PEOPERTY_HPP_
#define DEBUG_PEOPERTY_HPP_

#include "debug-cmd.hpp"

#include "smartlink_sdk_sys_property.h"

using namespace smartlink_sdk;
/*
 * @Description: 该类的功能描述
 */
class DebugProperty
{
  public:
    DebugProperty();
    virtual ~DebugProperty();
             

	void CmdHelp();
	void CmdQuit();
	void CmdExit();

	void CmdGet(int argc, char *argv[]);
	void CmdSet(int argc, char *argv[]);
	void CmdstartEvent();
	void CmdstopEvent();

    DebugCmdLine *GetCmdTable();

	void Init();

    //获取属性名
    RtnCode GetValue(std::string &name, std::string &value);
    
    //设置属性名
    RtnCode SetValue(std::string name, std::string value);

    void  pro_recv_cb(SysProperty::PropertyEventId id, void*data, int len);


  private:
    static DebugCmdLine pro_table[];
	static bool m_event;

};

#endif /* DEBUG_LOCATION_HPP_ */

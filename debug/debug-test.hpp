/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-test.hpp
*  Description:  debug test 头文件
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/

#ifndef DEBUG_TEST_HPP_
#define DEBUG_TEST_HPP_

#include "debug-cmd.hpp"

/*
 * @Description: 该类的功能描述
 */
class DebugTest
{
  public:
    DebugTest();
    virtual ~DebugTest();

    DebugCmdLine *GetCmdTable();

    int CmdHelp();
    int CmdQuit();
    int CmdExit();
    int CmdGet(int argc, char *argv[]);
    int CmdSet(int argc, char *argv[]);


  private:
    static DebugCmdLine test_table[];

};

#endif /* DEBUG_TEST_HPP_ */

/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-session.hpp
*  Description:  debug程序入口
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/


#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
using namespace std;
#include <sys/time.h>

#include "debug-cmd.hpp"
#include "debug-test.hpp"
#include "debug-location.hpp"
#include "debug-power.hpp"
#include "debug-property.hpp"
#include "debug-tel.hpp"


#include <stdint.h>

int main(int argc, char **argv)
{
    DebugCmd *cmd  = DebugCmd::GetInstance();

    DebugTest debug_test;
    DebugLocation debug_loc;
    DebugPower debug_power;
    DebugProperty debug_pro;
    DebugTel debug_tel;

    debug_loc.Init();
    debug_power.Init();
    debug_tel.Init();

    cmd->AddTable(DebugCmd::TEST, debug_test.GetCmdTable());
    cmd->AddTable(DebugCmd::LOC, debug_loc.GetCmdTable());
    cmd->AddTable(DebugCmd::POWER, debug_power.GetCmdTable());
    cmd->AddTable(DebugCmd::PRO, debug_pro.GetCmdTable());
    cmd->AddTable(DebugCmd::TEL, debug_tel.GetCmdTable());

    cmd->Entry();

    return 0;
}

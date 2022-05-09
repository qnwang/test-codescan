/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-test.cpp
*  Description:  debug test 实现
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/

#include <string.h>
#include "debug-test.hpp"
#include "debug-cmd-color.hpp"

char g_version[32] = "v0.0.1";

DebugCmdLine DebugTest::test_table[] =
{
    {
        "?",
        "  --- help.", (DebugCmdCB)& DebugTest::CmdHelp
    },
    {
        "get",
        "\r\n     version     --- Get version."
        , (DebugCmdCB)& DebugTest::CmdGet
    },
    {
        "set",
        "\r\n     version     --- Set version."
        , (DebugCmdCB)& DebugTest::CmdSet
    },
    {"q", "quit", (DebugCmdCB)& DebugTest::CmdQuit},
    {"x", "exit", (DebugCmdCB)& DebugTest::CmdExit},
    {0, 0, 0}
};


DebugTest::DebugTest()
{
    // TODO Auto-generated constructor stub

}

DebugTest::~DebugTest()
{
    // TODO Auto-generated destructor stub
}

int DebugTest::CmdHelp()
{
    DebugCmdLine *entry = test_table;
    while (entry != NULL)
    {
        if (entry->cmd != NULL)
        {
            DEBUG_COLOR_PRINTF(GREEN, " '%s'-%s\n", entry->cmd, entry->help);
        }
        else
        {
            break;
        }
        entry++;
    }
    return 0;
}

int DebugTest::CmdGet(int argc, char *argv[])
{
    if (argc >= 1)
    {
        if (strcmp(argv[1], "version") == 0)
        {
            DEBUG_COLOR_PRINTF(GREEN, "Get Version: %s\r\n", g_version);
        }
        else
        {
            DEBUG_COLOR_PRINTF(RED, "\r\n  get %s not supported\r\n", argv[1]);
        }
    }
  
    return 0;
}

int DebugTest::CmdSet(int argc, char *argv[])
{
	if (strcmp(argv[1], "version") == 0)
	{
	    if (argc >= 2)
	    {
		if (strlen(argv[1]) < 32)
		{
		    DEBUG_COLOR_PRINTF(GREEN, "Set Version: %s\r\n", argv[2]);
		    strcpy(g_version, argv[2]);
		}
	    }
	    else
	    {
			DEBUG_COLOR_PRINTF(RED, "\r\n  Set %s not supported\r\n", argv[1]);
	    }
	}
	else
	{
	    DEBUG_COLOR_PRINTF(RED, "\r\n  Set %s not supported\r\n", argv[1]);
	}

    return 0;
}

int DebugTest::CmdQuit()
{
    DebugCmd::GetInstance()->Quit(0, 0);
    return 0;
}

int DebugTest::CmdExit()
{
    DebugCmd::GetInstance()->Exit(0, 0);
    return 0;
}

DebugCmdLine *DebugTest::GetCmdTable()
{
    return test_table;
}


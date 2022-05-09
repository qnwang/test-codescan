/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-location.cpp
*  Description:  debug location的实现
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/

#include <string.h>
#include <unistd.h>

#include "debug-property.hpp"
#include "debug-cmd-color.hpp"

using namespace smartlink_sdk;

static SysProperty * pro_api = NULL;
bool DebugProperty::m_event = false;

DebugProperty::DebugProperty()
{

}

DebugProperty::~DebugProperty()
{
    // TODO Auto-generated destructor stub
}


DebugCmdLine DebugProperty::pro_table[] =
{
    {
        "?",
        "  --- help.", (DebugCmdCB)& DebugProperty::CmdHelp
    },
    {
        "get",
        "\r\n   name  value     --- GetValue."
        , (DebugCmdCB)& DebugProperty::CmdGet
    },
    {
        "set",
        "\r\n   name  value     --- SetValue."
        "\r\n     event  on/off  ."
        , (DebugCmdCB)& DebugProperty::CmdSet
    },
    {"q", "quit", (DebugCmdCB)& DebugProperty::CmdQuit},
    {"x", "exit", (DebugCmdCB)& DebugProperty::CmdExit},
    {0, 0, 0}
};


void  DebugProperty::pro_recv_cb(SysProperty::PropertyEventId id, void*data, int len)
{
    if(!m_event)
    {
      return;
    }
}


void DebugProperty::Init()
{
    pro_api = SysProperty::GetInstance();
    if(pro_api != NULL )
    {

#if 0
        //    std::vector<std::string> events;
        //   events.push_back(SysProperty::PropertyEventId::E_SYS_PROPERTY_EVENT_VALUE_CHANGED);


        //  if(pro_api->Init(ModuleID::E_MODULE_DEBUG))
        {
            pro_api->RegEventCB(events,std::bind(&DebugProperty::pro_recv_cb,this, std::placeholders::_1, 
                std::placeholders::_2, 
                std::placeholders::_3));
        }

#endif
    }
}

void DebugProperty::CmdHelp()
{
    DebugCmdLine *entry = pro_table;
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
}

void DebugProperty::CmdGet(int argc, char *argv[])
{

    RtnCode rtn = RtnCode::E_ERR;
    std::string  name;

    std::string  result = "no finded";

    printf("argc = %d\r\n",argc);

    printf("argc0 = %s\r\n",argv[0]);
    printf("argc1 = %s\r\n",argv[1]);
    printf("argc2 = %s\r\n",argv[2]);


    if (argc >= 2)
    {
         name =  std::string (argv[1]);
         rtn = pro_api->GetValue(name, result);
         if(rtn ==  RtnCode::E_SUCCESS)
         {
             DEBUG_COLOR_PRINTF(GREEN, "\r\n  Get name:%s value:%s\r\n", argv[1],result.c_str());
         }
         else
         {
             DEBUG_COLOR_PRINTF(RED, "\r\n  Get %s failedd\r\n", argv[1]);
         }
    }
    else
    {
        DEBUG_COLOR_PRINTF(RED, "\r\n  Get %s cmd not supported\r\n", argv[1]);
    }
}

void DebugProperty::CmdSet(int argc, char *argv[])
{

        if (argc >= 3)
        {
            if(pro_api->SetValue(std :: string(argv[1]), std :: string(argv[2])) == RtnCode::E_SUCCESS)
            {
                DEBUG_COLOR_PRINTF(GREEN, "\r\n  Set %s %s ok\r\n", argv[1],argv[2]);
            }
            else
            {
                DEBUG_COLOR_PRINTF(RED, "\r\n Set pro api rtn error \r\n");
            }
        }
        else
        {
            DEBUG_COLOR_PRINTF(RED, "\r\n  Set %s not supported\r\n", argv[1]);
        }
  
}

void DebugProperty::CmdQuit()
{
    m_event = false;
    DebugCmd::GetInstance()->Quit(0, 0);
}

void DebugProperty::CmdExit()
{
    m_event = false;
    DebugCmd::GetInstance()->Exit(0, 0);
}

DebugCmdLine* DebugProperty::GetCmdTable()
{
    return pro_table;
}





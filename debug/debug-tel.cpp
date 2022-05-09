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

#include "debug-tel.hpp"
#include "debug-cmd-color.hpp"

using namespace smartlink_sdk;

static Telephony * tel_api = NULL;
bool DebugTel::m_event = false;

DebugCmdLine DebugTel::tel_table[] =
{
    {
        "?",
        "  --- help.", (DebugCmdCB)& DebugTel::CmdHelp
    },
    {
        "get",
        "\r\n     imei        --- GetIMEI."
        "\r\n     iccid       --- GetICCID."
        "\r\n     imsi        --- GetIMSI."
        , (DebugCmdCB)& DebugTel::CmdGet
    },
    {
        "set",
        "\r\n     event  on/off  ."
        , (DebugCmdCB)& DebugTel::CmdSet
    },
    {"q", "quit", (DebugCmdCB)& DebugTel::CmdQuit},
    {"x", "exit", (DebugCmdCB)& DebugTel::CmdExit},
    {0, 0, 0}
};

void  DebugTel::tel_recv_cb(TelEventId id, void*data, int len)
{
    if(!m_event)
    {
      return;
    }
    
    
    DEBUG_COLOR_PRINTF(GREEN, "tel_recv_cb   id: %d len %d\n", id,len);
    
    switch(id)
    {
            default:
            break;
    }
}

void DebugTel::Init()
{
    tel_api = Telephony::GetInstance();
    if(tel_api != NULL )
    {
        std::vector<TelEventId> events;
        if(tel_api->Init(ModuleID::E_MODULE_DEBUG))
        {
            tel_api->RegEventCB(events,std::bind(&DebugTel::tel_recv_cb,this, std::placeholders::_1, 
                                                                   std::placeholders::_2, 
                                                                std::placeholders::_3));
        }
    }
}

DebugTel::DebugTel()
{

}

DebugTel::~DebugTel()
{
    // TODO Auto-generated destructor stub
}

void DebugTel::CmdHelp()
{
    DebugCmdLine *entry = tel_table;
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



void DebugTel::CmdGet(int argc, char *argv[])
{
    if (argc >= 1)
    {
        if (strcmp(argv[1], "imei") == 0)
        {
            GetIMEI();
        }
        else if (strcmp(argv[1], "imsi") == 0)
        {
            GetIMSI();
        }
        else if (strcmp(argv[1], "iccid") == 0)
        {
            GetICCID();
        }
        else
        {
            DEBUG_COLOR_PRINTF(RED, "\r\n  get %s not supported\r\n", argv[1]);
        }
    }
}

void DebugTel::CmdSet(int argc, char *argv[])
{
    if (strcmp(argv[1], "freq") == 0)
    {
        if (argc >= 2)
        {
            //if (strlen(argv[1]) < 32)
            //{
             //   DEBUG_COLOR_PRINTF(GREEN, "Set Version: %s\r\n", argv[2]);
             //   strcpy(g_version, argv[2]);
            //}
        }
        else
        {
            DEBUG_COLOR_PRINTF(RED, "\r\n  Set %s not supported\r\n", argv[1]);
        }
    }
    else if (strcmp(argv[1], "event") == 0)
    {
        if (argc >= 2)
        {
             if (strcmp(argv[2],"on")==0)
             {
                   DEBUG_COLOR_PRINTF(GREEN, "\r\n set event %s\r\n", argv[2]);
                   m_event = true;
                   DEBUG_COLOR_PRINTF(GREEN, "\r\n m_event = %d\r\n", m_event);
             }
             else if (strcmp(argv[2],"off")==0)
             {
                   DEBUG_COLOR_PRINTF(GREEN, "\r\n set event %s\r\n", argv[2]);
                   m_event = false;
             }
             else
             {
                DEBUG_COLOR_PRINTF(RED, "\r\n Set event argv[2] not surpported\r\n");
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
}

void DebugTel::CmdQuit()
{
    m_event = false;
    DebugCmd::GetInstance()->Quit(0, 0);
}

void DebugTel::CmdExit()
{
    m_event = false;
    DebugCmd::GetInstance()->Exit(0, 0);
}

DebugCmdLine *DebugTel::GetCmdTable()
{
    return tel_table;
}


//获取SIM卡IMEI值
RtnCode DebugTel::GetIMEI()
{
    std::string imei;

    if(tel_api != NULL )
    {
        tel_api->GetIMEI(imei);
        DEBUG_COLOR_PRINTF(GREEN, "\r\n get imei %s\r\n", imei.c_str());
    }
}

//获取SIM卡IMSI值
RtnCode DebugTel::GetIMSI()
{
    std::string  imsi;

    if(tel_api != NULL )
    {
        tel_api->GetIMSI(imsi);
        DEBUG_COLOR_PRINTF(GREEN, "\r\n get imsi %s\r\n", imsi.c_str());
    }
}

//获取SIM卡ICCID值
RtnCode DebugTel::GetICCID()
{
    std::string  iccid;

    if(tel_api != NULL )
    {
        tel_api->GetICCID(iccid);
        DEBUG_COLOR_PRINTF(GREEN, "\r\n get iccid %s\r\n", iccid.c_str());
    }
}




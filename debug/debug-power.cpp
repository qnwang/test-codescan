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

#include "debug-power.hpp"
#include "debug-cmd-color.hpp"

using namespace smartlink_sdk;

static SysPower * power_api = NULL;
bool DebugPower::m_event = false;

static void show_power_wakeup_state(SysPowerWakeupInfo *info)
{
    if (info == NULL)
    {
        return;
    }
    DEBUG_COLOR_PRINTF(GREEN, "<<<<<<<<<<<<<<<<<Power state >>>>>>>>>>>>>>>>>>>\n");
    DEBUG_COLOR_PRINTF(GREEN, "Power   state: %d\n", info->source);

    DEBUG_COLOR_PRINTF(GREEN, "wakeup by: %d\n", info->source);
    if (info->source == SysWakeupSource::E_SYS_POWER_WAKEUP_SOURCE_MPU_RTC)
    {
       DEBUG_COLOR_PRINTF(GREEN, "wakeup clock id: %s\n", info->clock_id);
    }
}


static void show_power_sleep_state()
{
    DEBUG_COLOR_PRINTF(GREEN, "<<<<<<<<<<<<<<<<<Power state >>>>>>>>>>>>>>>>>>>\n");
    DEBUG_COLOR_PRINTF(GREEN, "System to be sleep\n");
}

static void show_power_reboot_state(SysPowerRebootInfo*info)
{
    DEBUG_COLOR_PRINTF(GREEN, "<<<<<<<<<<<<<<<<<Power state >>>>>>>>>>>>>>>>>>>\n");
    DEBUG_COLOR_PRINTF(GREEN, "System to be reboot:type %d \r\n", info->type);
}

void  DebugPower::power_recv_cb(SysPowerEventID id, void*data, int len)
{
    if(!m_event)
    {
        return;
    }
    
    DEBUG_COLOR_PRINTF(GREEN, "power_recv_cb   id: %d\n", id);
    
    switch(id)
    {
        case SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP:
            show_power_wakeup_state((SysPowerWakeupInfo *)data);
            break;

        case SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP:
            show_power_wakeup_state((SysPowerWakeupInfo *)data);
            break;

        case SysPowerEventID::E_SYS_POWER_EVENT_TO_REBOOT:
            show_power_wakeup_state((SysPowerWakeupInfo *)data);
            break;

            default:
            break;
    }
}

DebugCmdLine DebugPower::power_table[] =
{
    {
        "?",
        "  --- help.", (DebugCmdCB)& DebugPower::CmdHelp
    },
    {
        "get",
        "\r\n     ready       --- IsReady."
        , (DebugCmdCB)& DebugPower::CmdGet
    },
    {
        "set",
        "\r\n     freq        --- Setfrequency."
        "\r\n     event  on/off  ."
        , (DebugCmdCB)& DebugPower::CmdSet
    },
    {"q", "quit", (DebugCmdCB)& DebugPower::CmdQuit},
    {"x", "exit", (DebugCmdCB)& DebugPower::CmdExit},
    {0, 0, 0}
};

void DebugPower::Init()
{
    std::vector<SysPowerEventID> events;
    events.push_back(smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP);
    events.push_back(smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP);
    events.push_back(smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_REBOOT);
    events.push_back(smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SHUTDOWN);
    events.push_back(smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_FLASH);
    if(SysPower::GetInstance()->Init(ModuleID::E_MODULE_DEBUG))
    {
        SysPower::GetInstance()->RegEventCB(events,std::bind(&DebugPower::power_recv_cb,this, std::placeholders::_1, 
                                                               std::placeholders::_2, 
                                                            std::placeholders::_3));
    }
}

DebugPower::DebugPower()
{

}

DebugPower::~DebugPower()
{
    // TODO Auto-generated destructor stub
}

void DebugPower::CmdHelp()
{
    DebugCmdLine *entry = power_table;
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

void DebugPower::CmdGet(int argc, char *argv[])
{
    if (argc >= 1)
    {
        if (strcmp(argv[1], "ready") == 0)
        {
            CmdIsReady();
        }
        else
        {
            DEBUG_COLOR_PRINTF(RED, "\r\n  get %s not supported\r\n", argv[1]);
        }
    }
}

void DebugPower::CmdSet(int argc, char *argv[])
{
    if (strcmp(argv[1], "event") == 0)
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

void DebugPower::CmdQuit()
{
    m_event = false;
    DebugCmd::GetInstance()->Quit(0, 0);
}

void DebugPower::CmdExit()
{
    m_event = false;
    DebugCmd::GetInstance()->Exit(0, 0);
}

DebugCmdLine *DebugPower::GetCmdTable()
{
    return power_table;
}

void DebugPower::CmdIsReady()
{
    power_api->IsReady();
}
  


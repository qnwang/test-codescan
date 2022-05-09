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

#include "debug-location.hpp"
#include "debug-cmd-color.hpp"

using namespace smartlink_sdk;

static Location * loc_api = NULL;
bool DebugLocation::m_event = false;

static void show_location(LocationInfo *info)
{
    tm* local;
    int second;
    char date_buf[64] = {};

    if (info == NULL)
    {
        return;
    }
    DEBUG_COLOR_PRINTF(GREEN, "<<<<<<<<<<<<<<<GNSS>>>>>>>>>>>>>>>>\n");

    DEBUG_COLOR_PRINTF(GREEN,    "          status: %lx\n", info->status);
    
    if (info->status& LOCATION_FIX_STATUS_TIME)
    {
       DEBUG_COLOR_PRINTF(GREEN, "             utc: %lld\n", info->utc)
       second = info->utc/1000;
       
       local = gmtime((time_t *) &second);
       (void)strftime(date_buf, 64, "%Y-%m-%d %H:%M:%S", local);
       DEBUG_COLOR_PRINTF(GREEN, "       utc  time: %s.%d\n", date_buf,info->utc%1000)

       local = localtime((time_t *) &second);
       (void)strftime(date_buf, 64, "%Y-%m-%d %H:%M:%S", local);
       DEBUG_COLOR_PRINTF(GREEN, "      local time: %s.%d\n", date_buf,info->utc%1000)
    }
    if (info->fix_valid)
    {
       DEBUG_COLOR_PRINTF(GREEN, "       longitude: %f\n", info->longitude);
       DEBUG_COLOR_PRINTF(GREEN, "        latitude: %f\n", info->latitude);
       DEBUG_COLOR_PRINTF(GREEN, "        altitude: %f\n", info->altitude);
       DEBUG_COLOR_PRINTF(GREEN, "         heading: %f\n", info->heading);
       DEBUG_COLOR_PRINTF(GREEN, "           speed: %f\n", info->speed);
       DEBUG_COLOR_PRINTF(GREEN, "            pdop: %f\n", info->pdop);
       DEBUG_COLOR_PRINTF(GREEN, "            hdop: %f\n", info->hdop);
       DEBUG_COLOR_PRINTF(GREEN, "            vdop: %f\n", info->vdop);
    }
}

void  DebugLocation::gnss_recv_cb(LocationEventId id, void*data, int len)
{
    if(!m_event)
    {
      return;
    }
    
    
    DEBUG_COLOR_PRINTF(GREEN, "gnss_recv_cb   id: %d len %d\n", id,len);
    
    switch(id)
    {
        case LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED:
            show_location((LocationInfo *)data);
            break;
        case  LocationEventId::E_LOCATION_EVENT_NEMA_INFO_CHANGED:
        {
            NEMAInfo*nmeaInfo = (NEMAInfo*)data;
            std::cout << "NMEA:" << nmeaInfo->data.c_str() <<  std::endl;
           break;
          }
            default:
            break;

    }
}

DebugCmdLine DebugLocation::loc_table[] =
{
    {
        "?",
        "  --- help.", (DebugCmdCB)& DebugLocation::CmdHelp
    },
    {
        "get",
        "\r\n     version     --- GetGnssSwVersion."
        "\r\n     ready       --- IsReady."
        "\r\n     freq        --- Getfrequency."
        "\r\n     antenna     --- GetGnssAntennaState."
        "\r\n     loc         --- GetLocation."
        "\r\n     sate        --- GetSatelliteInfo."
        , (DebugCmdCB)& DebugLocation::CmdGet
    },
    {
        "set",
        "\r\n     freq        --- Setfrequency."
        "\r\n     event  on/off  ."
        , (DebugCmdCB)& DebugLocation::CmdSet
    },
    {"q", "quit", (DebugCmdCB)& DebugLocation::CmdQuit},
    {"x", "exit", (DebugCmdCB)& DebugLocation::CmdExit},
    {0, 0, 0}
};

void DebugLocation::Init()
{
    loc_api = Location::GetInstance();
    if(loc_api != NULL )
    {
        std::vector<LocationEventId> events;
        events.push_back(smartlink_sdk::LocationEventId::E_LOCATION_EVENT_NEMA_INFO_CHANGED);
        events.push_back(smartlink_sdk::LocationEventId::E_LOCATION_EVENT_GNSS_ANTENNA_CHANGED);
        events.push_back(smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED);
        events.push_back(smartlink_sdk::LocationEventId::E_LOCATION_EVENT_SATELLITE_INFO_CHANGED);
        
        if(loc_api->Init(ModuleID::E_MODULE_DEBUG))
        {
            loc_api->RegEventCallback(events,std::bind(&DebugLocation::gnss_recv_cb,this, std::placeholders::_1, 
                                                                   std::placeholders::_2, 
                                                                std::placeholders::_3));
        }
    }
}

DebugLocation::DebugLocation()
{

}

DebugLocation::~DebugLocation()
{
    // TODO Auto-generated destructor stub
}

void DebugLocation::CmdHelp()
{
    DebugCmdLine *entry = loc_table;
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

void DebugLocation::CmdGet(int argc, char *argv[])
{
    if (argc >= 1)
    {
        if (strcmp(argv[1], "version") == 0)
        {
            CmdGetGnssSwVersion();
        }
        else if (strcmp(argv[1], "ready") == 0)
        {
            CmdIsReady();
        }
        else if (strcmp(argv[1], "freq") == 0)
        {
            CmdGetfrequency();
        }
        else if (strcmp(argv[1], "antenna") == 0)
        {
            CmdGetGnssAntennaState();
        }
        else if (strcmp(argv[1], "loc") == 0)
        {
            CmdGetLocation();
        }
        else if (strcmp(argv[1], "sate") == 0)
        {
            CmdGetSatelliteInfo();
        }
        else
        {
            DEBUG_COLOR_PRINTF(RED, "\r\n  get %s not supported\r\n", argv[1]);
        }
    }
}

void DebugLocation::CmdSet(int argc, char *argv[])
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

void DebugLocation::CmdQuit()
{
    m_event = false;
    DebugCmd::GetInstance()->Quit(0, 0);
}

void DebugLocation::CmdExit()
{
    m_event = false;
    DebugCmd::GetInstance()->Exit(0, 0);
}

DebugCmdLine *DebugLocation::GetCmdTable()
{
    return loc_table;
}

void DebugLocation::CmdIsReady()
{
    if(loc_api != NULL )
    {
        loc_api->IsReady();
    }
}
    
/*获取GNSS模组的软件版本号*/
void DebugLocation::CmdGetGnssSwVersion()
{
    std::string ver;
    //loc_api->GetGnssSwVersion(ver);
    DEBUG_COLOR_PRINTF(GREEN, "Get Version: %s\r\n", ver.c_str());
}

/*设置定位频率*/
void DebugLocation::CmdGetfrequency()
{
    uint8_t freq;
    if(loc_api != NULL )
    {
        loc_api->Getfrequency(freq);
        DEBUG_COLOR_PRINTF(GREEN, "Get frequency: %d\r\n",freq);
    }
}

/*获取定位频率*/
void DebugLocation::CmdSetfrequency()
{

}

/*获取天线状态*/
void DebugLocation::CmdGetGnssAntennaState()
{
    GnssAntennaState state;
    if(loc_api != NULL )
    {
        loc_api->GetGnssAntennaState(state);
        DEBUG_COLOR_PRINTF(GREEN, "Get antenna status: %d\r\n",state);
    }
}

/*主动获取定位信息，如果定位无效，返回最后一次有效定位，同时定位标志置为无效*/
void DebugLocation::CmdGetLocation()
{
    LocationInfo info;
    if(loc_api != NULL )
    {
        loc_api->GetLocation(info);
        show_location(&info);
    }
}

/*获取卫星信息*/
void DebugLocation::CmdGetSatelliteInfo()
{
    if(loc_api != NULL )
    {
    }
}


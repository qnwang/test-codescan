/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-cmd.cpp
*  Description:  debug cmd接口实现
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/

#include "debug-cmd.hpp"
#include "debug-cmd-color.hpp"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

bool DebugCmd::cmd_exit = false;

int DebugCmd::table_level = 0;

int DebugCmd::table_index = -1;

DebugCmd *DebugCmd::m_instance = new DebugCmd();

DebugCmdLine DebugCmd::cmd_table_0[] =
{
    {"?", "help.", &DebugCmd::Help},
    {"0", "test.", &DebugCmd::CmdTabSelect},
    {"1", "mcu service.", &DebugCmd::CmdTabSelect},
    {"2", "telephony.", &DebugCmd::CmdTabSelect},
    {"3", "location.", &DebugCmd::CmdTabSelect},
    {"4", "power manager.", &DebugCmd::CmdTabSelect},
    {"5", "tsp service.", &DebugCmd::CmdTabSelect},
    {"6", "ivi service.", &DebugCmd::CmdTabSelect},
    {"7", "vehicle service.", &DebugCmd::CmdTabSelect},
    {"8", "property.", &DebugCmd::CmdTabSelect},
    {"q", "quit.", &DebugCmd::Exit},
    {"x", "exit.", &DebugCmd::Exit},
    {0, 0, 0}
};


DebugCmd::DebugCmdTableNode  DebugCmd::table_list[MAX] =
{
    {false, "TEST", 0},
    {false, "MCU", 0},
    {false, "TEL", 0},
    {false, "LOC", 0},
    {false, "POWER", 0},
    {false, "TSP", 0},
    {false, "IVI", 0},
    {false, "VEC", 0},
    {false, "PRO", 0},
};

DebugCmd::DebugCmd()
{
    // TODO Auto-generated constructor stub
}

DebugCmd::~DebugCmd()
{
    // TODO Auto-generated destructor stub
}

DebugCmd *DebugCmd::GetInstance()
{
    return m_instance;
}

void DebugCmd::AddTable(DebugCmd::TABLE_INDEX index, DebugCmdLine *table)
{
    table_list[index].table = table;
    table_list[index].init = true;
}

void DebugCmd::Entry()
{
 
    Welcome();
    char line[DEBUG_CMD_LENGTH + 1] = {0};
    do
    {
        Input();
        memset(line, 0, sizeof(line));
        DEBUG_PRINTF_COLOR_SET(GREEN);
        if (fgets(line, sizeof(line), stdin) != NULL)
        {
            ReadLine(line);
            fflush(stdout);
        }
        else
        {
            if (feof(stdin))
            {
                DEBUG_COLOR_PRINTF(GREEN, "Input end\r\n");
            }
            else if (ferror(stdin))
            {
                DEBUG_COLOR_PRINTF(RED, "Input error\r\n");
                clearerr(stdin);
            }
            cmd_exit = true;
        }
    }
    while (!cmd_exit);
    DEBUG_PRINTF_COLOR_ESC;
    return;
}

bool DebugCmd::ReadLine(char *line)
{
    int argc = 0;
    char *argv[DEBUG_CMD_ARGS + 1] = {0};
    char *p_space = line;
    char *p_head = line;
    char *p_next = line;

    while(p_next != NULL)
    {           
        /*结束，退出*/
        if (*p_next == '\r' || *p_next == '\n' || *p_next == '\0')
        {
            *p_next = '\0';
            break;
        }
        else if(*p_next == ' ')
        {
            *p_next = '\0';
            p_next++;
            p_head = p_next;
            continue;
        }
        else
        {
            argv[argc] = p_head;
            argc++;
            /*查找下一个空格,可能存在下一个参数*/
            if ((p_space = strstr(p_next, " ")) != NULL)
            {   
                p_next = p_space;
            }
            else /*继续直到结束*/
            {
                p_next++;
            }
        }
    }
    CmdProcess(argc , argv);
    return true;
}

int DebugCmd::Quit(int argc, char *argv[])
{
    if (table_level > 0)
    {
        table_level = 0;
    }
    else
    {
        cmd_exit = true;
    }
    return 0;
}

int DebugCmd::Exit(int argc, char *argv[])
{
    cmd_exit = true;
    return 0;
}

int DebugCmd::Help(int argc, char *argv[])
{
    DebugCmdLine *entry = cmd_table_0;
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

int DebugCmd::CmdProcess(int argc, char *argv[])
{
    DebugCmdLine *entry = NULL;

    switch (table_level)
    {
        case 0:
            entry = cmd_table_0;
            break;
        case 1:
            if (table_list[table_index].init)
            {
                entry = table_list[table_index].table;
            }
            else
            {
                DEBUG_COLOR_PRINTF(RED, "%s has not been init!\n", table_list[table_index].name);
            }
            break;
        default:
            break;
    }
    while (entry != NULL)
    {
        if (entry->cmd != NULL)
        {
            if(argv[0] == NULL)
            {
            return Help( argc, argv);
        }
        else if (strcmp(argv[0], entry->cmd) == 0)
        {
            return (this->*(entry->func))(argc , argv);
        }
        }
    else
    {
        break;
    }
        entry++;
    }
    return 0;
}

int DebugCmd::CmdTabSelect(int argc, char *argv[])
{
    int index = atoi(argv[0]);
    switch (index)
    {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 8:
            if (table_list[index].init)
            {
                table_level = 1;
                table_index = index;
            }
            else
            {
                DEBUG_COLOR_PRINTF(RED, "%s has not been init!\n", table_list[index].name);
            }
            break;
        default:
            DEBUG_COLOR_PRINTF(RED, "Command %s not defined\r\n", argv[0]);
            return -1;
    }

    DebugCmdLine *entry = table_list[table_index].table;
    while (entry != NULL)
    {
        if (entry->cmd != NULL)
        {
            DEBUG_COLOR_PRINTF(GREEN, " '%s'%s\n", entry->cmd, entry->help);
        }
        else
        {
            break;
        }
        entry++;
    };
    return 0;
}

void DebugCmd::Welcome()
{
    DEBUG_PRINTF_COLOR_SET(GREEN);
    DEBUG_PRINTF_BUDDHA;
    DEBUG_PRINTF_COLOR_SET(YELLOW);
    printf("           Welcome debug session, version %s.\r\n", DEBUG_CMD_VERSION);
    \
    printf("               Build at %s %s.\r\n", __DATE__, __TIME__);
    \
    DEBUG_PRINTF_COLOR_SET(GREEN);
    printf("Please input your command(? for help):");
    DEBUG_PRINTF_COLOR_ESC
}

void DebugCmd::Input()
{
    switch (table_level)
    {
        case 0:
            DEBUG_COLOR_PRINTF(GREEN, "\n[MENU]>");
            break;
        case 1:
            DEBUG_COLOR_PRINTF(GREEN, "\n[%s]>", table_list[table_index].name);
            break;
        default:
            break;
    }
}



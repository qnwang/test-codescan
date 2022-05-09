/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-cmd.hpp
*  Description:  debug cmd接口描述
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/


#ifndef DEBUG_CMD_HPP_
#define DEBUG_CMD_HPP_

//#include "module_id.hpp"

#define DEBUG_CMD_VERSION          ("0.0.1")

#define DEBUG_CMD_LENGTH           (128)
#define DEBUG_CMD_ARGS             (6)
#define DEBUG_CMD_SUB_TABLIE_NUM   (16)



class DebugCmd;

/*
 *  @function: DebugCmdCB(int argc, char *argv[]);
 *  @argc:命令行解析参数个数n.
 *  @argv:argv[n]字符指针指向每个参数对应字段.
 */
typedef int (DebugCmd::*DebugCmdCB)(int argc, char *argv[]);

typedef struct
{
    const char *cmd; //命令行输入
    const char *help;//命令帮助
    DebugCmdCB func; //命令解析回调函数
} DebugCmdLine;

/*
 * @Description: 该类的功能描述
 */
class DebugCmd
{
  public:
    DebugCmd();
    virtual ~DebugCmd();

    typedef enum
    {
        TEST = 0,/*network service*/
        MCU = 1,
        TEL = 2,
        LOC = 3,
        POWER = 4,
        TSP = 5,
        IVI = 6,
        VEC = 7,
        PRO = 8,
        MAX,
    } TABLE_INDEX;

    static DebugCmd *GetInstance();

    int Help(int argc, char *argv[]);

    /*增加命令菜单，各个模块根据二级功能ID增加菜单列表*/
    void AddTable(TABLE_INDEX index, DebugCmdLine *table);

    /*加载命令菜单*/
    void Entry();

    /*返回上一级*/
    int Quit(int argc, char *argv[]);

    /*退出*/
    int Exit(int argc, char *argv[]);
  private:

    typedef struct
    {
        bool init;
        char name[8];
        DebugCmdLine *table;
    } DebugCmdTableNode;

    static DebugCmd *m_instance;

    static bool cmd_exit;

    static int table_level;

    static int table_index;

    static DebugCmdLine cmd_table_0[];

    static DebugCmdTableNode  table_list[MAX];

    bool ReadLine(char *line);

    int CmdTabSelect(int argc, char *argv[]);

    int CmdProcess(int argc, char *argv[]);

    void Welcome();

    void Input();
};

#endif /* DEBUG_CMD_HPP_ */

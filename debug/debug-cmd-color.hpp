/******************************************************************************  
*  Copyright (C) 2020-2030 SmartLink-Tech. http://www.smartlink-tech.com.cn
*  File Name  :  debug-cmd-color.cpp
*  Description:  debug cmd命令颜色
*  Date          Version     Author       Description
*  ----------    --------    ----------   --------------------------------------
*  2020-12-24    1.0.0       zouyu        Published
*
******************************************************************************/

#ifndef DEBUG_CMD_COLOR_HPP_
#define DEBUG_CMD_COLOR_HPP_

#include <stdio.h>

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#define DEBUG_PRINTF_COLOR_SET(color)  do{printf(color);}while(0);
#define DEBUG_PRINTF_COLOR_ESC         do{printf(RESET);}while(0);

#define DEBUG_COLOR_PRINTF(color,fmt,args...)\
    do\
    {\
        printf(color);\
        printf(fmt, ##args);\
        printf(RESET);\
    }while(0);


#define DEBUG_PRINTF_BUDDHA  do{\
        printf("                            _ooOoo_                         \r\n");\
        printf("                           o8888888o                        \r\n");\
        printf("                           88\" . \"88                        \r\n");\
        printf("                           (| -_- |)                        \r\n");\
        printf("                            O\\ = /O                         \r\n");\
        printf("                        ____/`---'\\___                      \r\n");\
        printf("                      .   ' \\\\| |// `.                      \r\n");\
        printf("                       / \\\\||| s |||// \\                    \r\n");\
        printf("                     / _||||| -m- |||||- \\                  \r\n");\
        printf("                       | | \\\\\\ a /// | |                    \r\n");\
        printf("                     | \\_| ''\\-r-/'' | |                    \r\n");\
        printf("                      \\ .-\\__ `t` ___/-. /                  \r\n");\
        printf("                   ___`. .' /--l--\\ `. . __                 \r\n");\
        printf("                .\"\" '< `.___\\_<i>_/___.' >'\"\".              \r\n");\
        printf("               | | : `- \\`.;`\\ n /`;.`/ - ` : | |           \r\n");\
        printf("                 \\ \\ `-. \\_ __\\k/__ _/ .-` / /              \r\n");\
        printf("         ======`-.____`-.___\\_____\\/___.-`____.-'======      \r\n");\
        printf("                            `=---='                         \r\n");\
    }while(0);
#endif /* DEBUG_CMD_COLOR_HPP_ */

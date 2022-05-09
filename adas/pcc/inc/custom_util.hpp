/*****************************************************************************/
/**
* \file       custom_util.h
* \date       2022/02/17
* \author     wujian
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    公共静态方法类
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#ifndef _CUSTOM_UTIL_H_
#define _CUSTOM_UTIL_H_

#include "smlk_types.h"

class Util
{
public:
    static void get_bcd_timestamp(SMLK_UINT8 *buffer)
    {
        struct tm   tt{};
        time_t now = time(0);
        localtime_r(&now, &tt);

        tt.tm_year  -= 100;
        tt.tm_mon   += 1;

        buffer[0]   = (((SMLK_UINT8)(tt.tm_year / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_year % 10)) & 0x0F);
        buffer[1]   = (((SMLK_UINT8)(tt.tm_mon / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_mon % 10)) & 0x0F);
        buffer[2]   = (((SMLK_UINT8)(tt.tm_mday / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_mday % 10)) & 0x0F);
        buffer[3]   = (((SMLK_UINT8)(tt.tm_hour / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_hour % 10)) & 0x0F);
        buffer[4]   = (((SMLK_UINT8)(tt.tm_min / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_min % 10)) & 0x0F);
        buffer[5]   = (((SMLK_UINT8)(tt.tm_sec / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_sec % 10)) & 0x0F);
    };

    static SMLK_UINT32 bcd_timestamp_2_id(const SMLK_UINT8 *timestamp)
    {
        /*低字节到高字节的顺序：年月日时分秒*/

        struct tm time;
        time.tm_year = 2000+(((timestamp[0] >> 4) & 0xF) * 10 + (timestamp[0] & 0xF)) - 1900;//tm中的年份比实际年份小1900，需要减掉
        time.tm_mon = (((timestamp[1] >> 4) & 0xF) * 10 + (timestamp[1] & 0xF)) - 1;//tm中的月份从0开始，需要减1
        time.tm_mday = (((timestamp[2] >> 4) & 0xF) * 10 + (timestamp[2] & 0xF));
        time.tm_hour = (((timestamp[3] >> 4) & 0xF) * 10 + (timestamp[3] & 0xF));
        time.tm_min = (((timestamp[4] >> 4) & 0xF) * 10 + (timestamp[4] & 0xF));
        time.tm_sec = (((timestamp[5] >> 4) & 0xF) * 10 + (timestamp[5] & 0xF));
        time_t ltime_new = mktime(&time);
        return (SMLK_UINT32)(ltime_new);
    };

};

#endif
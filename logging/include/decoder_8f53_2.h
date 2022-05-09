/*****************************************************************************/
/**
* \file       decoder_8f53_2.h
* \author     wukai
* \date       2021/07/12
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
#ifndef DECODER_8F53_2_H_
#define DECODER_8F53_2_H_
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/

#include "smlk_types.h"
#include <iostream>
#include <vector>
#include "logger_jt808_def.h"

using namespace std;
namespace smartlink {
class Decoder8F53_2
{
public:
    Decoder8F53_2() :port_(0){};
    ~Decoder8F53_2(){};

    void Decode(IN SMLK_UINT8* data, IN std::size_t len);

public:
    string host_;
    string usrname_;
    string passwd_;
    string server_path_;
    SMLK_UINT16 port_;
    vector<string> file_lists;
};
inline string BCD2ASCII(const string &str)
{
    string res;
    size_t nBegin = 0;
    int nTemp;
    char szBuf[16] = { 0 };
    if (str.size() % 2 != 0)
    {
        szBuf[0] = '0';
        szBuf[1] = str[0];
        sscanf(szBuf, "%x", &nTemp);
        nBegin = 1;
        res.push_back((char)nTemp);
    }
    for (; nBegin < str.size(); nBegin += 2)
    {
        szBuf[0] = str[nBegin];
        szBuf[1] = str[nBegin + 1];
        sscanf(szBuf, "%x", &nTemp);
        res.push_back((char)nTemp);
    }
    return res;

}

inline string ASCII2BCD(const string &str)
{
    string res;

    for (size_t i = 0; i < str.size()/2; i++)
    {
        char Htemp = (str[2*i] >> 4) & 0x0F;
        if (Htemp >= 10)
            Htemp = Htemp - 10;
        char Ltemp = str[2*i + 1] & 0x0F;
        if (Ltemp >= 10)
            Ltemp = Ltemp - 10;
        char temp = 0;
        temp = (Htemp << 4) + Ltemp;
        res.push_back(temp);
    }
    return res;
}
}
#endif // DECODER_8F53_2_H_
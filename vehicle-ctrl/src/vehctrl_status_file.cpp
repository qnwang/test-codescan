/*****************************************************************************/
/**
 * \file       rctrl_test.cpp
 * \author     huangxin
 * \date       2021/03/02
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>

#include "vehctrl_configfile.h"
#include "vehctrl_status_file.h"
#include "smlk_log.h"
#include "smlk_property.h"
#include <iconv/iconv.h>

using namespace std;
using namespace smartlink;

using namespace Poco::JSON;
using namespace Poco;

#if 0
std::string to_string(SMLK_UINT8 value)
{
    char value_str1[16] ={0};
    sprintf(value_str1, "%d", value);
    string value_str = value_str1;
    return value_str;
}

std::string to_string(SMLK_UINT16 value)
{
    char value_str1[16] ={0};
    sprintf(value_str1, "%d", value);
    string value_str = value_str1;
    return value_str;


}

std::string to_string(SMLK_UINT32 value)
{
    char value_str1[16] ={0};
    sprintf(value_str1, "%d", value);
    string value_str = value_str1;
    return value_str;


}


std::string to_string(SMLK_DOUBLE value)
{
    char value_str1[16] ={0};
    sprintf(value_str1, "%lf", value);
    string value_str = value_str1;
    return value_str;
}

#endif

SMLK_RC code_convert(char *from_charset, char *to_charset, char *inbuf, size_t inlen, char *outbuf, size_t outlen)
{
    iconv_t cd;
    char **pin = &inbuf;
    char **pout = &outbuf;

    cd = iconv_open(to_charset, from_charset);
    if (cd == 0)
    {
        return SMLK_RC::RC_ERROR;
    }
    memset(outbuf, 0, outlen);
    if (iconv(cd, pin, &inlen, pout, &outlen) == -1)
    {
        return SMLK_RC::RC_ERROR;
    }
    iconv_close(cd);
    *pout = '\0';
    return SMLK_RC::RC_OK;
}

SMLK_RC gbk_to_utf8(char *inbuf, size_t inlen, char *outbuf, size_t outlen)
{
    return code_convert("gbk", "utf-8", inbuf, inlen, outbuf, outlen);
}
SMLK_RC utf8_to_gbk(char *inbuf, size_t inlen, char *outbuf, size_t outlen)
{
    return code_convert("utf-8", "gbk", inbuf, inlen, outbuf, outlen);
}

std::string hex_to_string(SMLK_UINT8 *pvalue, SMLK_UINT8 len)
{
    char value_str1[16] = {0};
    sprintf(&value_str1[0], "0x");
    int j = 2;
    for (SMLK_UINT8 i = 0; i < len; ++i)
    {
        sprintf(&value_str1[j], "%02x", pvalue[i]);
        j += 2;
    }
    string value_str = value_str1;
    return value_str;
}

SMLK_RC string_to_hex(std::string string, SMLK_UINT8 *cbuf, SMLK_UINT8 buflen)
{
    SMLK_UINT8 high, low;
    std::size_t idx;
    int ii = 0;
    SMLK_UINT8 *pstr = (SMLK_UINT8 *)(string.c_str());
    if ((pstr[0] == '0') && (pstr[1] == 'x'))
    {
        ;
    }
    else
    {
        return SMLK_RC::RC_OK;
    }
    for (idx = 2; idx < string.size(); idx += 2)
    {
        high = pstr[idx];
        low = pstr[idx + 1];

        if (high >= '0' && high <= '9')
            high = high - '0';
        else if (high >= 'A' && high <= 'F')
            high = high - 'A' + 10;
        else if (high >= 'a' && high <= 'f')
            high = high - 'a' + 10;
        else
            return SMLK_RC::RC_ERROR;

        if (low >= '0' && low <= '9')
            low = low - '0';
        else if (low >= 'A' && low <= 'F')
            low = low - 'A' + 10;
        else if (low >= 'a' && low <= 'f')
            low = low - 'a' + 10;
        else
            return SMLK_RC::RC_ERROR;
        if (ii + 1 > buflen)
        {
            break;
        }
        cbuf[ii++] = high << 4 | low;
    }
    return SMLK_RC::RC_OK;
}

void MakeNewFolder(string mountpoint)
{
    SMLK_LOGD("Access to path=%s", mountpoint.c_str());
    if (0 != access(mountpoint.c_str(), 0))
    {
        for (SMLK_UINT16 i = 0; i < mountpoint.size(); ++i)
        {
            if (*(char *)(mountpoint.data() + i) == '/')
            {
                string path;
                path.insert(path.end(), (mountpoint.begin()), (mountpoint.begin() + i + 1));
                // SMLK_LOGD("path: (%s)\n",path.c_str());
                if (0 != access(path.c_str(), 0))
                {
                    if (mkdir(path.c_str(), 0555) != 0)
                    {
                        SMLK_LOGW("Can't mkdir=%s", path.c_str());
                        return;
                    }
                    else
                    {
                        SMLK_LOGI("Mkdir=%s", path.c_str());
                    }
                }
            }
        }
    }
    else
    {
        SMLK_LOGD("Filepath=%s already exist!", mountpoint.c_str());
    }
    return;
}

void ClearFile(std::string file_path)
{
    std::ofstream out_file_stream(file_path);
    std::string str_temp = "";
    out_file_stream.write(str_temp.c_str(), 0);
    out_file_stream.close();
}

VehCtrlConfigJsonFile *VehCtrlConfigJsonFile::GetInstance()
{
    static VehCtrlConfigJsonFile s_rctrl_config_file;
    return &s_rctrl_config_file;
}

SMLK_RC VehCtrlConfigJsonFile::GetJsonValAndCheckValid(OUT std::string &str_json, OUT Dynamic::Var &result, IN std::string &file_location)
{
    ostringstream out_buf;
    char ch;
    ifstream in_file_stream(file_location.c_str());
    if (!in_file_stream.is_open())
    {
        SMLK_LOGE("Filepath=%s open failed!", file_location.c_str());
        return SMLK_RC::RC_ERROR;
    }
    while (out_buf && in_file_stream.get(ch))
    {
        out_buf.put(ch);
    }
    str_json = out_buf.str();
    in_file_stream.close();

    JSON::Parser parser;
    parser.reset();
    try
    {
        result = parser.parse(str_json);
    }
    catch (const std::exception &e)
    {
        SMLK_LOGE("Filepath=%s parse failed!", file_location.c_str());
        return SMLK_RC::RC_ERROR;
    }
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/29
 * \brief       初始化状态配置文件
 * \return
 * \remarks
 ******************************************************************************/
SMLK_RC VehCtrlConfigJsonFile::InitStatusJsonFile(IN std::string &path_name)
{
    std::ifstream in_file_stream(path_name);
    if (!in_file_stream.is_open()) /*配置文件不存在,创建文件并写入参数*/
    {
        SetInitialKeyValue(path_name);
    }
    else /*配置文件存在,监测文件内容是否含有车控配置参数,如果没有,创建默认值*/
    {
        std::string str_json;
        Dynamic::Var var_temp;
        SMLK_RC valid_res = GetJsonValAndCheckValid(str_json, var_temp);
        if (SMLK_RC::RC_OK != valid_res) /*文件Json无法解析,尝试清空文件,重新写入初始数据*/
        {
            ClearFile(path_name);
            SetInitialKeyValue(path_name);
        }
        in_file_stream.close();
    }
    return SMLK_RC::RC_OK;
}
SMLK_RC VehCtrlConfigJsonFile::SetInitialKeyValue(IN std::string &file_location)
{
    std::ofstream ofs(file_location);
    if (ofs.is_open())
    {
        /*创建初始化的状态*/
        JSON::Object obj_new;
        obj_new.set(VEHICLE_CTRL_IDLING_WARM_UP_STATUS, to_string((SMLK_UINT8)(RctrlStatusIdlingWarmUp::STATUS_NONE)));
        std::stringstream ostr;
        obj_new.stringify(ostr, 1);
        std::string output = ostr.str();
        SMLK_LOGD("Initial msg=%s", output.data());
        ofs << output;
        ofs.close();
    }
    else
    {
        SMLK_LOGE("Filepath=%s open failed.", file_location.c_str());
        return SMLK_RC::RC_ERROR;
    }
}

SMLK_RC VehCtrlConfigJsonFile::GetValueFromJsonFile(IN std::string &name, OUT std::string &value, IN std::string &file_location)
{
    /*判断文件内容json合法性并获取数据*/
    string config_file_str;
    Dynamic::Var result;
    SMLK_RC rc;
    rc = GetJsonValAndCheckValid(config_file_str, result);
    if (SMLK_RC::RC_OK != rc)
        return SMLK_RC::RC_ERROR;

    Object::Ptr obj;
    try
    {
        obj = result.extract<JSON::Object::Ptr>();
    }
    catch (const std::exception &e)
    {
        SMLK_LOGE("extract json object failed\n");
        return SMLK_RC::RC_ERROR;
    }

    Poco::Dynamic::Var one_param = obj->get(name.c_str());
    if (one_param.isEmpty())
    {
        SMLK_LOGE("Cann't find [Key]=%s", name.c_str());
        return SMLK_RC::RC_ERROR;
    }
    else
    {
        value = one_param.convert<std::string>();
        SMLK_LOGD("Find [Key]=%s [Val]=%s", name.c_str(), value.c_str());
    }

    return SMLK_RC::RC_OK;
}

SMLK_RC VehCtrlConfigJsonFile::SetValueToJsonFile(IN std::string &name, IN std::string &value, IN std::string &file_location)
{
    /*判断文件内容json合法性并获取数据*/
    string config_file_str;
    Dynamic::Var result;
    SMLK_RC rc;
    rc = GetJsonValAndCheckValid(config_file_str, result);
    if (SMLK_RC::RC_OK != rc)
        return SMLK_RC::RC_ERROR;

    Object::Ptr obj;
    try
    {
        obj = result.extract<JSON::Object::Ptr>();
    }
    catch (const std::exception &e)
    {
        SMLK_LOGE("extract json object failed\n");
        return SMLK_RC::RC_ERROR;
    }

    Poco::Dynamic::Var one_param = obj->get(name.c_str());
    if (one_param.isEmpty())
    {
        SMLK_LOGD("Add [Key]=%s", name.c_str());
        obj->set(name.c_str(), value.c_str());
    }
    else
    {
        std::string one_param_value = one_param.convert<std::string>();
        obj->set(name.c_str(), value.c_str());
        SMLK_LOGD("Reset [Key]=%s [Val]=%s->%s", name.c_str(), one_param_value.c_str(), value.c_str());
    }

    // SMLK_LOGD("obj->size()=%ld\n",obj->size());
    std::stringstream ostr;
    obj->stringify(ostr, 1);
    std::string output = ostr.str();
    std::cout << "After set, file is:" << output << endl;

    ofstream fout(file_location.c_str());
    fout.write(output.c_str(), output.size());
    fout.close();

    return SMLK_RC::RC_OK;
}

VehCtrlConfigJsonFile::VehCtrlConfigJsonFile()
{
}

VehCtrlConfigJsonFile::~VehCtrlConfigJsonFile()
{
}
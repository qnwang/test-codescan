/*****************************************************************************/
/**
 * \file       vehctrl_status_file.h
 * \author     huangxin
 * \date       2021/03/02
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_STATUS_FILE_H_
#define _VEHICLE_STATUS_FILE_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <iostream>
#include <cstring>
#include "Poco/JSON/Parser.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/JSON/Object.h"
#include "Poco/JSON/ParseHandler.h"
#include "Poco/JSON/JSONException.h"
#include "Poco/StreamCopier.h"
#include "vehctrl_general_definition.h"
#include "smlk_error.h"

using namespace std;

extern smartlink::SMLK_RC gbk_to_utf8(char *inbuf, size_t inlen, char *outbuf, size_t outlen);
extern smartlink::SMLK_RC utf8_to_gbk(char *inbuf, size_t inlen, char *outbuf, size_t outlen);
extern std::string hex_to_string(SMLK_UINT8 *pvalue, SMLK_UINT8 len);
extern smartlink::SMLK_RC string_to_hex(std::string string, SMLK_UINT8 *cbuf, SMLK_UINT8 buflen);
extern void MakeNewFolder(string mountpoint);

namespace smartlink
{

#define SYS_PRO_NAME_CURRENT_PROTOCOL ("rctrl.current-protocol") /*当前使用的协议：jtt808为0*/

#define VEHICLE_CTRL_IDLING_WARM_UP_STATUS "Idling_Warm_Up"

    enum class RctrlStatusIdlingWarmUp : SMLK_UINT8
    {
        STATUS_NONE = 0,       /*无需发送*/
        STATUS_CLOSE = 1,      /*暖机关闭*/
        STATUS_OPEN_LEV_1 = 2, /*暖机开启1挡*/
        STATUS_OPEN_LEV_2 = 3, /*暖机开启2档*/
    };

#define VEHICLE_CTRL_LAST_4G_ANTENNA_STATUS "Last_4G_Status"

    enum class RctrlLast4gAntennaStatus : SMLK_UINT8
    {
        STATUS_NONE = 0,     //未检测到天线
        STATUS_IS_EFFECTIVE, //检测到天线

    };

    class VehCtrlConfigJsonFile
    {
    public:
        static VehCtrlConfigJsonFile *GetInstance();

    public:
#if 0
    std::string int_to_string(SMLK_UINT8 value);
    std::string int_to_string(SMLK_UINT16 value);
    std::string int_to_string(SMLK_UINT32 value);
    std::string int_to_string(SMLK_DOUBLE value);
    std::string hex_to_string(SMLK_UINT8 *pvalue, SMLK_UINT8 len);
#endif
        SMLK_RC gbk_to_utf8(char *inbuf, size_t inlen, char *outbuf, size_t outlen);
        SMLK_RC utf8_to_gbk(char *inbuf, size_t inlen, char *outbuf, size_t outlen);
        SMLK_RC string_to_hex(std::string string, SMLK_UINT8 *cbuf, SMLK_UINT8 buflen);
        SMLK_RC GetJsonValAndCheckValid(OUT std::string &str_json, OUT Poco::Dynamic::Var &result, IN std::string &file_location = VEHICLE_CTRL_STATUS_FILE_PATH_NAME);
        SMLK_RC SetInitialKeyValue(IN std::string &file_location = VEHICLE_CTRL_STATUS_FILE_PATH_NAME);
        SMLK_RC InitStatusJsonFile(IN std::string &path_name);
        SMLK_RC GetValueFromJsonFile(IN std::string &name, OUT std::string &value, IN std::string &file_location = VEHICLE_CTRL_STATUS_FILE_PATH_NAME);
        SMLK_RC SetValueToJsonFile(IN std::string &name, IN std::string &value, IN std::string &file_location = VEHICLE_CTRL_STATUS_FILE_PATH_NAME);

    private:
        VehCtrlConfigJsonFile();
        ~VehCtrlConfigJsonFile();
    };

};

#endif

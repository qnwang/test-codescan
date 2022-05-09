/*****************************************************************************/
/**
 * \file       remote_ctrl_config.cpp
 * \author     huangxin
 * \date       2020/11/03
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>

#include "Poco/JSON/Parser.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/JSON/Object.h"
#include "Poco/JSON/ParseHandler.h"
#include "Poco/JSON/JSONException.h"
#include "Poco/StreamCopier.h"

#include "vehctrl_configfile.h"
#include <dlfcn.h>

using namespace std;
using namespace smartlink;
using namespace Poco::JSON;
using namespace Poco;

extern smartlink::SMLK_RC string_to_hex(std::string string, SMLK_UINT8 *cbuf, SMLK_UINT8 buflen);

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/11/13
 * \brief       read config file with json format, and parse it then save in hash table
 * \return      error:parse failed; ok:parse successful
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC RemoteCtrlCmdLoad::ReadCmdConfigFile()
{
    SMLK_LOGD("enter in func(%s).", __func__);
    char ch;
    string config_file_str;
    ostringstream obuf;
    string file_loction = VEHICLE_CTRL_JSON_CONFIG_FILE_PATH_NAME;

    ifstream fin(file_loction.c_str());
    if (!fin.is_open())
    {
        SMLK_LOGE("Filepath=%s doesn't exist!", file_loction.c_str());
        return SMLK_RC::RC_ERROR;
    }
    while (obuf && fin.get(ch))
    {
        obuf.put(ch);
    }

    config_file_str = obuf.str();
    fin.close();

    SMLK_LOGD("f:  %s\n", config_file_str.c_str());
    JSON::Parser parser;
    Dynamic::Var result;

    Poco::Dynamic::Var cmd;
    Poco::Dynamic::Var subcmd;
    Poco::Dynamic::Var timeout;
    Poco::Dynamic::Var max_send_times;
    Poco::Dynamic::Var retrydelay;

    Poco::Dynamic::Var params;

    parser.reset();
    try
    {
        result = parser.parse(config_file_str);
    }
    catch (const std::exception &e)
    {
        SMLK_LOGE("JSON::Parser parse file(%s) failed\n", file_loction.c_str());
        return SMLK_RC::RC_ERROR;
    }
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

    Poco::Dynamic::Var configfile = obj->get(CONFIGFILE_NAME);
    if (configfile.isEmpty())
    {
        SMLK_LOGE("cann't find object name(%s) in file\n", CONFIGFILE_NAME);
        return SMLK_RC::RC_ERROR;
    }

    Array::Ptr arr;
    try
    {
        arr = configfile.extract<JSON::Array::Ptr>();
    }
    catch (const std::exception &e)
    {
        SMLK_LOGE("extract json Array failed\n");
        return SMLK_RC::RC_ERROR;
    }

    Object::Ptr default_obj = arr->getObject(0);
    if (nullptr == default_obj)
    {
        SMLK_LOGE("first object in array read failed\n");
        return SMLK_RC::RC_ERROR;
    }
    Poco::Dynamic::Var default_timeout = default_obj->get(DEFAULT_TIMEOUT_NAME);
    if (default_timeout.isEmpty())
    {
        SMLK_LOGE("cann't find object name(%s) in file\n", DEFAULT_TIMEOUT_NAME);
        return SMLK_RC::RC_ERROR;
    }
    m_defalut_timeout = default_timeout.convert<int>();

    Poco::Dynamic::Var default_retrytimes = default_obj->get(DEFAULT_RETRYTIMES_NAME);
    if (default_retrytimes.isEmpty())
    {
        SMLK_LOGE("cann't find object name(%s) in file\n", DEFAULT_RETRYTIMES_NAME);
        return SMLK_RC::RC_ERROR;
    }
    m_defalut_retrytimes = default_retrytimes.convert<int>();

    Poco::Dynamic::Var default_retrydelay = default_obj->get(DEFAULT_RETRYDELAY_NAME);
    if (default_retrydelay.isEmpty())
    {
        SMLK_LOGE("cann't find object name(%s) in file\n", DEFAULT_RETRYDELAY_NAME);
        return SMLK_RC::RC_ERROR;
    }
    m_defalut_retrydelay = default_retrydelay.convert<int>();
    SMLK_LOGD("m_defalut_timeout=%d, m_defalut_retrytimes=%d, m_defalut_retrydelay=%d\n", m_defalut_timeout, m_defalut_retrytimes, m_defalut_retrydelay);

    Poco::Dynamic::Var default_reportdelay = default_obj->get(DEFAULT_REPORTDELAY_NAME);
    if (default_reportdelay.isEmpty())
    {
        SMLK_LOGE("cann't find object name(%s) in file\n", DEFAULT_REPORTDELAY_NAME);
        return SMLK_RC::RC_ERROR;
    }
    m_report_delay = default_reportdelay.convert<int>();
    SMLK_LOGD("m_report_delay=%d\n", m_report_delay);

    for (SMLK_UINT16 i = 1; i < arr->size(); i++)
    {
        Object::Ptr object = nullptr;
        RemoteCtrlConfigFileInfo cmdinfo;
        object = arr->getObject(i);
        if (nullptr == object)
        {
            SMLK_LOGE(" get object(%d) failed.\n", i);
            return SMLK_RC::RC_ERROR;
        }
        cmd = object->get(CMD_NAME);
        if (cmd.isEmpty())
        {
            SMLK_LOGE("cann't find object name(%s) in file\n", CMD_NAME);
            return SMLK_RC::RC_ERROR;
        }
        subcmd = object->get(SUBCMD_NAME);
        if (cmd.isEmpty())
        {
            SMLK_LOGE("cann't find object name(%s) in file\n", SUBCMD_NAME);
            return SMLK_RC::RC_ERROR;
        }
        Array::Ptr subcmd_arr;
        try
        {
            subcmd_arr = subcmd.extract<JSON::Array::Ptr>();
        }
        catch (const std::exception &e)
        {
            SMLK_LOGE("extract subcmd_arr json Array failed\n");
            return SMLK_RC::RC_ERROR;
        }
        for (SMLK_UINT16 k = 0; k < subcmd_arr->size(); k++)
        {
            SMLK_UINT16 temp_subcmd = subcmd_arr->get(k);
            cmdinfo.subcmd_vec.push_back(temp_subcmd);
        }

        timeout = object->get(TIMEOUT_NAME);
        if (timeout.isEmpty())
        {
            SMLK_LOGE("cann't find object name(%s) in file\n", TIMEOUT_NAME);
            return SMLK_RC::RC_ERROR;
        }
        max_send_times = object->get(MAX_SENDTIMES_NAME);
        if (max_send_times.isEmpty())
        {
            SMLK_LOGE("cann't find object name(%s) in file\n", MAX_SENDTIMES_NAME);
            return SMLK_RC::RC_ERROR;
        }
        retrydelay = object->get(RETRYDELAY_NAME);
        if (retrydelay.isEmpty())
        {
            SMLK_LOGE("cann't find object name(%s) in file\n", RETRYDELAY_NAME);
            return SMLK_RC::RC_ERROR;
        }

        try
        {
            cmdinfo.cmd_id = cmd.convert<int>();
            cmdinfo.timieout = timeout.convert<int>();
            cmdinfo.max_send_times = max_send_times.convert<int>();
            cmdinfo.retry_delay = retrydelay.convert<int>();
        }
        catch (const std::exception &e)
        {
            SMLK_LOGE("convert int failed\n");
            return SMLK_RC::RC_ERROR;
        }
        // cmdinfo.timieout = timeout.convert<int>();
        //  cmdinfo.max_send_times = max_send_times.convert<int>();
        params = object->get(PARAMS_NAME);
        if (params.isEmpty())
        {
            SMLK_LOGE("cann't find object name(%s) in file\n", PARAMS_NAME);
            return SMLK_RC::RC_ERROR;
        }
        Array::Ptr params_arr;
        try
        {
            params_arr = params.extract<JSON::Array::Ptr>();
        }
        catch (const std::exception &e)
        {
            SMLK_LOGE("extract params json Array failed\n");
            return SMLK_RC::RC_ERROR;
        }

        SMLK_LOGD("cmd_id=%d, timieout=%d, max_send_times=%d\n",
                  cmdinfo.cmd_id, cmdinfo.timieout, cmdinfo.max_send_times);
        SMLK_LOGD("params_arr->size()=%ld\n", params_arr->size());

        for (SMLK_UINT16 k = 0; k < params_arr->size(); k++)
        {
            RemoteCtrlErrRetryInfo err_retry;
            Poco::Dynamic::Var errcode;
            Poco::Dynamic::Var err_retrytimes;
            Object::Ptr errcode_obj = nullptr;

            errcode_obj = params_arr->getObject(k);
            if (nullptr == errcode_obj)
            {
                SMLK_LOGE(" get object(%d) failed.\n", k);
                return SMLK_RC::RC_ERROR;
            }
            errcode = errcode_obj->get(ERRCODE_NAME);
            if (errcode.isEmpty())
            {
                SMLK_LOGE("cann't find object name(%s) in file\n", ERRCODE_NAME);
                return SMLK_RC::RC_ERROR;
            }
            err_retrytimes = errcode_obj->get(RETRYTIMES_NAME);
            if (err_retrytimes.isEmpty())
            {
                SMLK_LOGE("cann't find object name(%s) in file\n", RETRYTIMES_NAME);
                return SMLK_RC::RC_ERROR;
            }

            try
            {
                err_retry.errcode = errcode.convert<int>();
                err_retry.err_retry_times = err_retrytimes.convert<int>();
            }
            catch (const std::exception &e)
            {
                SMLK_LOGE("convert int failed\n");
                return SMLK_RC::RC_ERROR;
            }

            cmdinfo.retry_vec.push_back(err_retry);
            SMLK_LOGD("errcode=%d, err_retry_times=%d\n",
                      cmdinfo.retry_vec[k].errcode, cmdinfo.retry_vec[k].err_retry_times);
        }

        SMLK_LOGD("insert cmdid=%d to hash\n", cmdinfo.cmd_id);
        m_config_file_map.insert(make_pair(cmdinfo.cmd_id, cmdinfo));
    }

    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/04/25
 * \brief       根据车型读取不同的脚本
 * \param[in]   file_loction
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC RemoteCtrlCmdLoad::GetVehicleTypeFileScript(std::string &)
{

    return SMLK_RC::RC_OK;
}

RemoteCtrlCmdLoad::RemoteCtrlCmdLoad()
{
    SMLK_RC rc;
    m_defalut_retrydelay = 3;
    m_defalut_timeout = 6;
    m_defalut_retrytimes = 0;
    m_report_delay = 5;
    m_LockWake_delay = 30;
    m_module_handle = nullptr;
    rc = ReadCmdConfigFile();
    if (SMLK_RC::RC_ERROR == rc)
    {
        if (m_config_file_map.size() > 0)
        {
            m_config_file_map.erase(m_config_file_map.begin(), m_config_file_map.end());
            SMLK_LOGD("parse config file failed, erase m_config_file_map\n");
        }
    }
}

RemoteCtrlCmdLoad::~RemoteCtrlCmdLoad()
{
}

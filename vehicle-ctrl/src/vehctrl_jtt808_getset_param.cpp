/*****************************************************************************/
/**
 * \file       jtt808_msg8103.cpp
 * \author     huangxin
 * \date       2020/11/25
 * \version    Tbox2.0 V1
 * \brief      decode and encode terminal param related message ,msg id :0x8103 0x8104 0x8106
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include <iostream>
#include <cstring>
#include <smlk_error.h>
#include "vehctrl_general_definition.h"

#include "vehctrl_jtt808_getset_param.h"
#include "vehctrl_queue.h"

#include "smlk_property.h"
#include "smartlink_sdk_sys_property.h"
#include "vehctrl_status_file.h"

using namespace std;
using namespace smartlink;
using namespace smartlink::jt808msg8103;
using namespace smartlink_sdk;

SMLK_UINT32 DissectorMsgTerminalParam::GetParamDefaultVaule(IN std::string &param_name)
{
    SMLK_LOGD("enter in func(%s)\n", __func__);

    SMLK_UINT32 default_value = 0;
    if (strcmp(param_name.c_str(), SYS_PRO_NAME_TSP_HBT_INTERVAL) == 0)
    {
        default_value = SYS_PRO_NAME_TSP_HBT_INTERVAL_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_TSP_TCP_RESP_TIMEOUT) == 0)
    {
        default_value = SYS_PRO_NAME_TSP_TCP_RESP_TIMEOUT_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_SLEEP_REPORT_INTERVAL) == 0)
    {
        default_value = SYS_PRO_NAME_SLEEP_REPORT_INTERVAL_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_FATIGUE_DRIVER_TIME) == 0)
    {
        default_value = SYS_PRO_NAME_FATIGUE_DRIVER_TIME_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_REALTIME_DATA_UPLOAD_CYCLE) == 0)
    {
        default_value = SYS_PRO_NAME_REALTIME_DATA_UPLOAD_CYCLE_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_REALTIME_DATA_GATHER_CYCLE) == 0)
    {
        default_value = SYS_PRO_NAME_REALTIME_DATA_GATHER_CYCLE_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_STATIS_INFO_UPLOAD_CYCLE) == 0)
    {
        default_value = SYS_PRO_NAME_STATIS_INFO_UPLOAD_CYCLE_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_LONG_TIME_PARKING_TIME) == 0)
    {
        default_value = SYS_PRO_NAME_LONG_TIME_PARKING_TIME_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_LOW_TO_ULTRALOW_POWER_INTERVAL) == 0)
    {
        default_value = SYS_PRO_NAME_LOW_TO_ULTRALOW_POWER_INTERVAL_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_NORMAL_TO_LOW_POWER_INTERVAL) == 0)
    {
        default_value = SYS_PRO_NAME_NORMAL_TO_LOW_POWER_INTERVAL_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_PREDIC_CRUIS_SWITCH) == 0)
    {
        default_value = SYS_PRO_NAME_PREDIC_CRUIS_SWITCH_DEFALUT_VALUE;
    }
    else if (strcmp(param_name.c_str(), SYS_PRO_NAME_4G_ANTENNA_LOCK) == 0)
    {
        default_value = SYS_PRO_NAME_4G_ANTENNA_LOCK_DEFALUT_VALUE;
    }

    SMLK_LOGD("param_name(%s), default_value(%ld)\n", param_name.c_str(), default_value);
    return default_value;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/04/13
 * \brief       设置参数值，当前先采用自己写的配置文件，后续替换成恒润接口
 * \param[in]   param_id
 * \param[in]   param_len
 * \param[out]  param_value
 * \param[in]   valueinfo
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_UINT8 DissectorMsgTerminalParam::SetParamVaule(IN SMLK_UINT32 param_id, IN SMLK_UINT8 param_len, IN SMLK_UINT8 *param_value, IN jt808msg8103::ParamValue &valueinfo)
{
    SMLK_LOGD("param_id=0x%04x, param_len=%d,param_type=%d\n", param_id, param_len, valueinfo.param_type);
    RtnCode property_rc = RtnCode::E_SUCCESS;

    SMLK_UINT8 result = SMLK_TSP_0001_RESULT_SUCCESS;
    SMLK_UINT16 value_min;
    SMLK_UINT32 value_max;
    if (TYPE_BYTE == valueinfo.param_type)
    {
        if (1 != param_len)
        {
            result = SMLK_TSP_0001_RESULT_FAILED;
            SMLK_LOGE("param_type=%d, but pparam->param_len != 1.\n", param_id);
        }
        else
        {
            SMLK_UINT8 value = *(param_value);
            switch (param_id)
            {
            case PARAM_PREDIC_CRUIS_SWITCH_ID:
                value_min = BOUNDARY_VALUE_ZERO;
                value_max = BOUNDARY_VALUE_ONE;
                break;
            case PARAM_4G_ANTENNA_OPEN_ID:
                value_min = BOUNDARY_VALUE_ONE;
                value_max = BOUNDARY_VALUE_TWO;
                break;
            default:
                break;
            }
            if ((value < value_min) || (value > value_max))
            {
                SMLK_LOGD("paramid=0x%x,value=0x%x out of range .\n", param_id, value);
                return SMLK_TSP_0001_RESULT_OUT_OF_RANGE;
            }
            property_rc = SysProperty::GetInstance()->SetValue(valueinfo.sys_param_name, to_string(value));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, value, valueinfo.sys_param_name.c_str());
                return 0;
            }
            SMLK_LOGD("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s).\n", param_id, valueinfo.param_type, value, valueinfo.sys_param_name.c_str());
        }
    }
    else if (TYPE_WORD == valueinfo.param_type)
    {
        if (2 != param_len)
        {
            result = SMLK_TSP_0001_RESULT_FAILED;
            SMLK_LOGE("param_type=%d, but pparam->param_len != 2.\n", param_id);
        }
        else
        {
            SMLK_UINT16 value = be16toh(*(SMLK_UINT16 *)param_value);
            switch (param_id)
            {
            case PARAM_FATIGUE_DRIVER_TIME_ID:
                value_min = BOUNDARY_VALUE_ONE;
                value_max = BOUNDARY_VALUE_FATIGUE_DRIVE_TIME;
                break;
            // case PARAM_REALTIME_DATA_UPLOAD_CYCLE_ID:
            //     value_min = BOUNDARY_VALUE_ZERO;
            //     value_max = BOUNDARY_VALUE_UPLOAD_CYCLE_TIME;
            //     break;
            // case PARAM_REALTIME_DATA_GATHER_CYCLE_ID:
            //     value_min = BOUNDARY_VALUE_GATHER_CYCLE_TIME_MIN;
            //     value_max = BOUNDARY_VALUE_GATHER_CYCLE_TIME_MAX;
            //     break;
            case PARAM_STATIS_INFO_UPLOAD_CYCLE_ID:
                value_min = BOUNDARY_VALUE_INFO_UPLOAD_CYCLE_TIME;
                value_max = BOUNDARY_VALUE_UPLOAD_CYCLE_TIME;
                break;
            case PARAM_NORMAL_TO_LOW_POWER_INTERVAL_ID:
                value_min = BOUNDARY_VALUE_ONE;
                value_max = BOUNDARY_VALUE_LOW_POWER_INTERVAL_TIME;
                break;
            default:
                break;
            }
            if ((value < value_min) || (value > value_max))
            {
                SMLK_LOGD("value_min = %d,value_max = %d .\n", value_min, value_max);
                SMLK_LOGD("paramid=0x%x,value=0x%x out of range .\n", param_id, value);
                return SMLK_TSP_0001_RESULT_OUT_OF_RANGE;
            }
            property_rc = SysProperty::GetInstance()->SetValue(valueinfo.sys_param_name, to_string(value));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, value, valueinfo.sys_param_name.c_str());
                return 0;
            }
            // SysProperty::GetInstance()->SetValue(valueinfo.sys_param_name ,to_string(value));
            SMLK_LOGD("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s).\n", param_id, valueinfo.param_type, value, valueinfo.sys_param_name.c_str());
        }
    }
    else if (TYPE_DWORD == valueinfo.param_type)
    {
        if (4 != param_len)
        {
            result = SMLK_TSP_0001_RESULT_FAILED;
            SMLK_LOGE("param_id(0x%04x):param_len(%d) is not equal to 4.\n", param_id, param_len);
        }
        else
        {
            SMLK_UINT32 value = be32toh(*(SMLK_UINT32 *)param_value);
            switch (param_id)
            {
            case PARAM_HEARTBEAT_INTERVAL_ID:
                value_min = BOUNDARY_VALUE_HEARTBEAT_INTERVAL_MIN;
                value_max = BOUNDARY_VALUE_HEARTBEAT_INTERVAL_MAX;
                break;
            case PARAM_TCP_RESP_TIMEOUT_ID:
                value_min = BOUNDARY_VALUE_UPLOAD_CYCLE_TIME;
                value_max = BOUNDARY_VALUE_TCP_RESP_TIMEOUT_TIME;
                break;
            case PARAM_SLEEP_REPORT_INTERVAL_ID:
                value_max = BOUNDARY_VALUE_SLEEP_REPORT_INTERVAL_TIME_MAX;
                value_min = BOUNDARY_VALUE_SLEEP_REPORT_INTERVAL_TIME_MIN;
                break;
            default:
                break;
            }
            if ((value < value_min) || (value > value_max))
            {
                SMLK_LOGD("paramid=0x%x,value=0x%x out of range .\n", param_id, value);
                return SMLK_TSP_0001_RESULT_OUT_OF_RANGE;
            }
            property_rc = SysProperty::GetInstance()->SetValue(valueinfo.sys_param_name, to_string(value));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, value, valueinfo.sys_param_name.c_str());
                return 0;
            }
            // SysProperty::GetInstance()->SetValue(valueinfo.sys_param_name ,to_string(value));
            SMLK_LOGD("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s).\n", param_id, valueinfo.param_type, value, valueinfo.sys_param_name.c_str());
        }
    }
    else if (TYPE_STRING == valueinfo.param_type)
    {
        string value_string;
        value_string.clear();
        value_string.insert(value_string.end(), param_value, param_value + param_len);
        property_rc = SysProperty::GetInstance()->SetValue(valueinfo.sys_param_name, value_string);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, value_string.c_str(), valueinfo.sys_param_name.c_str());
            return 0;
        }
        SMLK_LOGD("paramid=0x%x, type=%d, value=%s, sys_param_name(%s).\n", param_id, valueinfo.param_type, value_string.c_str(), valueinfo.sys_param_name.c_str());
    }
    else if (TYPE_COMPOUND == valueinfo.param_type)
    {
        string name;
        if (PARAM_OVERSPEED_EVENT_ID == param_id)
        {
            OverSpeedEvent *pevent = (OverSpeedEvent *)param_value;
            /*属性*/
            name = SYS_PRO_NAME_OVERSPEED_ATTRIBUTE;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->attri));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->attri, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*车速阈值*/
            name = SYS_PRO_NAME_OVERSPEED_THRESHOLD;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->speed_threshold));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->speed_threshold, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*持续时间*/
            name = SYS_PRO_NAME_OVERSPEED_DURATION;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->time));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->time, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*语音提示的文字数据*/
            string voice_word;
            char buf[pevent->voice_hint_len] = {'0'};
            for (SMLK_UINT8 i = 0; i < (pevent->voice_hint_len); i++)
            {
                buf[i] = pevent->voice_hint_content[i];
            }
            char buf2[2 * (pevent->voice_hint_len)] = {'\0'};
            SMLK_RC rc = gbk_to_utf8(buf, pevent->voice_hint_len, buf2, 2 * (pevent->voice_hint_len));
            if (rc != SMLK_RC::RC_OK)
            {
                SMLK_LOGE("code_convert error");
                return 0;
            }
            voice_word = buf2;
            name = SYS_PRO_NAME_OVERSPEED_VOICE_WORD;
            property_rc = SysProperty::GetInstance()->SetValue(name, voice_word);
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, voice_word.c_str(), valueinfo.sys_param_name.c_str());
                return 0;
            }
        }
        else if (PARAM_RAPIDLY_ACCELERATE_EVENT_ID == param_id)
        {
            SharpSpeedEvent *pevent = (SharpSpeedEvent *)param_value;
            /*属性*/
            name = SYS_PRO_NAME_RAPIDLY_SPEED_ATTRIBUTE;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->attri));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->attri, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*车速上限阈值*/
            name = SYS_PRO_NAME_RAPIDLY_SPEED_UP_THRESHOLD;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->speed_upper_threshold));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->speed_upper_threshold, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*加速速上限阈值*/
            name = SYS_PRO_NAME_RAPIDLY_SPEED_ACCELERATE_UP_THRESHOLD;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->accspeed_upper_threshold));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->accspeed_upper_threshold, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*车速下限阈值*/
            name = SYS_PRO_NAME_RAPIDLY_SPEED_DOWN_THRESHOLD;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->speed_lower_threshold));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->speed_lower_threshold, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*加速速下限阈值*/
            name = SYS_PRO_NAME_RAPIDLY_SPEED_ACCELERATE_DOWN_THRESHOLD;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->accspeed_lower_threshold));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->accspeed_lower_threshold, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*语音提示的文字数据*/
            string voice_word;
            voice_word.insert(voice_word.end(), pevent->voice_hint_content, pevent->voice_hint_content + pevent->voice_hint_len);

            name = SYS_PRO_NAME_RAPIDLY_SPEED_VOICE_WORD_DATA;
            property_rc = SysProperty::GetInstance()->SetValue(name, voice_word);
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, voice_word.c_str(), valueinfo.sys_param_name.c_str());
                return 0;
            }
        }
        else if (PARAM_SHARP_SLOWDOWN_EVENT_ID == param_id)
        {
            SharpSpeedEvent *pevent = (SharpSpeedEvent *)param_value;
            /*属性*/
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_ATTRIBUTE;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->attri));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->attri, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*车速上限阈值*/
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_UP_THRESHOLD;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->speed_upper_threshold));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->speed_upper_threshold, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*加速速上限阈值*/
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_ACCELERATE_UP_THRESHOLD;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->accspeed_upper_threshold));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->accspeed_upper_threshold, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*车速下限阈值*/
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_DOWN_THRESHOLD;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->speed_lower_threshold));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->speed_lower_threshold, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*加速速下限阈值*/
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_ACCELERATE_DOWN_THRESHOLD;
            property_rc = SysProperty::GetInstance()->SetValue(name, to_string(pevent->accspeed_lower_threshold));
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, pevent->accspeed_lower_threshold, valueinfo.sys_param_name.c_str());
                return 0;
            }

            /*语音提示的文字数据*/
            string voice_word;
            voice_word.insert(voice_word.end(), pevent->voice_hint_content, pevent->voice_hint_content + pevent->voice_hint_len);

            name = SYS_PRO_NAME_SLOWDOWN_SPEED_VOICE_WORD;
            property_rc = SysProperty::GetInstance()->SetValue(name, voice_word);
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s), SysProperty SetValue failed.\n", param_id, valueinfo.param_type, voice_word.c_str(), valueinfo.sys_param_name.c_str());
                return 0;
            }
        }
    }
    else
    {

        result = SMLK_TSP_0001_RESULT_NOT_SUPPORT;
        SMLK_LOGE("param_type=%d, no such value.\n", param_id);
    }

    return result;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/04/06
 * \brief       函数描述
 * \param[in]   param_id
 * \param[in]   valueinfo   由于恒润接口没有定成const，故将该参数从IN 改成OUT ,20210406
 * \param[in]   output_vec
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsgTerminalParam::GetParamVaule(IN SMLK_UINT32 &param_id, OUT jt808msg8103::ParamValue &valueinfo, OUT vector<SMLK_UINT8> &output_vec)
{
    SMLK_LOGD("enter in func(%s), param_type=%d, sys_param_name =%s.\n", __func__, valueinfo.param_type, valueinfo.sys_param_name.c_str());

    RtnCode property_rc = RtnCode::E_SUCCESS;
    std::string sys_param_name;
    string value_str;
    ParamCommon param_common;
    param_common.param_id = htobe32(param_id);

    if (TYPE_BYTE == valueinfo.param_type)
    {
        param_common.param_len = sizeof(SMLK_UINT8);
        SMLK_UINT8 value = 0;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&param_common, (SMLK_UINT8 *)&param_common + PARAM_COMMON_LEN);
        property_rc = SysProperty::GetInstance()->GetValue(valueinfo.sys_param_name, value_str);
        if (RtnCode::E_SUCCESS == property_rc)
        {
            value = atoi(value_str.c_str());
        }
        else
        {
            value = (SMLK_UINT8)GetParamDefaultVaule(valueinfo.sys_param_name);
        }
        SMLK_LOGD("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s).\n", param_id, valueinfo.param_type, value, sys_param_name.c_str());
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&value, (SMLK_UINT8 *)&value + param_common.param_len);
    }
    else if (TYPE_WORD == valueinfo.param_type)
    {
        param_common.param_len = sizeof(SMLK_UINT16);
        SMLK_UINT16 value = 0;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&param_common, (SMLK_UINT8 *)&param_common + PARAM_COMMON_LEN);
        property_rc = SysProperty::GetInstance()->GetValue(valueinfo.sys_param_name, value_str);
        if (RtnCode::E_SUCCESS == property_rc)
        {
            value = atoi(value_str.c_str());
        }
        else
        {
            value = (SMLK_UINT16)GetParamDefaultVaule(valueinfo.sys_param_name);
        }
        value = htobe16(value);
        SMLK_LOGD("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s).\n", param_id, valueinfo.param_type, value, sys_param_name.c_str());
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&value, (SMLK_UINT8 *)&value + param_common.param_len);
    }
    else if (TYPE_DWORD == valueinfo.param_type)
    {
        param_common.param_len = sizeof(SMLK_UINT32);
        SMLK_UINT32 value = 0;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&param_common, (SMLK_UINT8 *)&param_common + PARAM_COMMON_LEN);

        property_rc = SysProperty::GetInstance()->GetValue(valueinfo.sys_param_name, value_str);
        if (RtnCode::E_SUCCESS == property_rc)
        {
            value = atoi(value_str.c_str());
        }
        else
        {
            value = (SMLK_UINT32)GetParamDefaultVaule(valueinfo.sys_param_name);
        }
        value = htobe32(value);
        SMLK_LOGD("paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s).\n", param_id, valueinfo.param_type, value, sys_param_name.c_str());
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&value, (SMLK_UINT8 *)&value + param_common.param_len);
    }
    else if (TYPE_STRING == valueinfo.param_type)
    {
#if 0
        property_rc = SysProperty::GetInstance()->GetValue(valueinfo.sys_param_name ,value_str);
        if( RtnCode::E_SUCCESS == property_rc ){
            value = atoi(value_str.c_str());
        }else{
            value = (SMLK_UINT32)GetParamDefaultVaule(valueinfo.sys_param_name);
        }

        param_common.param_len = value_str.size();
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&param_common, (SMLK_UINT8 *)&param_common + PARAM_COMMON_LEN);

        output_vec.insert(output_vec.end(), value_str.begin(), value_str.end());
        SMLK_LOGD( "paramid=0x%x, type=%d, value=0x%x, sys_param_name(%s).\n", param_id, valueinfo.param_type, value, sys_param_name.c_str());
#endif
    }
    else if (TYPE_COMPOUND == valueinfo.param_type)
    {
        if (PARAM_OVERSPEED_EVENT_ID == param_id)
        {

            OverSpeedEvent event;
            bzero(&event, sizeof(OverSpeedEvent));
            string name;
            string value;
            value.clear();
            name = SYS_PRO_NAME_OVERSPEED_ATTRIBUTE;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.attri = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.attri = SYS_PRO_NAME_OVERSPEED_ATTRIBUTE_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_OVERSPEED_THRESHOLD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.speed_threshold = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.speed_threshold = SYS_PRO_NAME_OVERSPEED_THRESHOLD_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_OVERSPEED_DURATION;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.time = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.time = SYS_PRO_NAME_OVERSPEED_DURATION_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_OVERSPEED_VOICE_WORD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                int m = value.size();
                char buf1[m] = {'0'};
                for (SMLK_UINT8 i = 0; i < m; i++)
                {
                    buf1[i] = value[i];
                }
                char buf22[m + 1] = {'\0'};
                SMLK_RC rc = utf8_to_gbk(buf1, m, buf22, m + 1);
                if (rc == SMLK_RC::RC_OK)
                {
                    string out = buf22;
                    event.voice_hint_len = out.length();
                    memcpy(event.voice_hint_content, out.c_str(), out.length());
                }
                else
                {
                    event.voice_hint_len = 0;
                }
            }
            else
            {
                event.voice_hint_len = 0;
            }

            param_common.param_len = sizeof(OverSpeedEvent) - MAX_VOICE_HINT_DATA_LEN + event.voice_hint_len;
            output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&param_common, (SMLK_UINT8 *)&param_common + PARAM_COMMON_LEN);
            output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&event, (SMLK_UINT8 *)&event + sizeof(OverSpeedEvent) - MAX_VOICE_HINT_DATA_LEN + event.voice_hint_len);
        }
        else if (PARAM_RAPIDLY_ACCELERATE_EVENT_ID == param_id)
        {

            SharpSpeedEvent event;
            bzero(&event, sizeof(SharpSpeedEvent));
            string name;

            string value;
            value.clear();
            name = SYS_PRO_NAME_RAPIDLY_SPEED_ATTRIBUTE;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.attri = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.attri = SYS_PRO_NAME_RAPIDLY_SPEED_ATTRIBUTE_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_RAPIDLY_SPEED_UP_THRESHOLD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.speed_upper_threshold = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.speed_upper_threshold = SYS_PRO_NAME_RAPIDLY_SPEED_UP_THRESHOLD_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_RAPIDLY_SPEED_ACCELERATE_UP_THRESHOLD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.accspeed_upper_threshold = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.accspeed_upper_threshold = SYS_PRO_NAME_RAPIDLY_SPEED_ACCELERATE_UP_THRESHOLD_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_RAPIDLY_SPEED_DOWN_THRESHOLD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.speed_lower_threshold = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.speed_lower_threshold = SYS_PRO_NAME_RAPIDLY_SPEED_DOWN_THRESHOLD_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_RAPIDLY_SPEED_ACCELERATE_DOWN_THRESHOLD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.accspeed_lower_threshold = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.accspeed_lower_threshold = SYS_PRO_NAME_RAPIDLY_SPEED_ACCELERATE_DOWN_THRESHOLD_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_OVERSPEED_VOICE_WORD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.voice_hint_len = value.size();
                memcpy(event.voice_hint_content, value.c_str(), event.voice_hint_len);
            }
            else
            {
                event.voice_hint_len = 0;
            }

            memcpy(event.voice_hint_content, value.c_str(), event.voice_hint_len);

            param_common.param_len = sizeof(SharpSpeedEvent) - MAX_VOICE_HINT_DATA_LEN + event.voice_hint_len;
            output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&param_common, (SMLK_UINT8 *)&param_common + PARAM_COMMON_LEN);
            output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&event, (SMLK_UINT8 *)&event + sizeof(SharpSpeedEvent) - MAX_VOICE_HINT_DATA_LEN + event.voice_hint_len);
        }
        else if (PARAM_SHARP_SLOWDOWN_EVENT_ID == param_id)
        {

            SharpSpeedEvent event;
            bzero(&event, sizeof(SharpSpeedEvent));
            string name;
            string value;
            value.clear();
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_ATTRIBUTE;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.attri = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.attri = SYS_PRO_NAME_SLOWDOWN_SPEED_ATTRIBUTE_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_UP_THRESHOLD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.speed_upper_threshold = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.speed_upper_threshold = SYS_PRO_NAME_SLOWDOWN_SPEED_UP_THRESHOLD_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_ACCELERATE_UP_THRESHOLD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.accspeed_upper_threshold = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.accspeed_upper_threshold = SYS_PRO_NAME_SLOWDOWN_SPEED_ACCELERATE_UP_THRESHOLD_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_DOWN_THRESHOLD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.speed_lower_threshold = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.speed_lower_threshold = SYS_PRO_NAME_SLOWDOWN_SPEED_DOWN_THRESHOLD_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_ACCELERATE_DOWN_THRESHOLD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.accspeed_lower_threshold = (SMLK_UINT8)atoi(value.c_str());
            }
            else
            {
                event.accspeed_lower_threshold = SYS_PRO_NAME_SLOWDOWN_SPEED_ACCELERATE_DOWN_THRESHOLD_DEFALUT_VALUE;
            }

            value.clear();
            name = SYS_PRO_NAME_SLOWDOWN_SPEED_VOICE_WORD;
            property_rc = SysProperty::GetInstance()->GetValue(name, value);
            if (RtnCode::E_SUCCESS == property_rc)
            {
                event.voice_hint_len = value.size();
                memcpy(event.voice_hint_content, value.c_str(), event.voice_hint_len);
            }
            else
            {
                event.voice_hint_len = 0;
            }

            param_common.param_len = sizeof(SharpSpeedEvent) - MAX_VOICE_HINT_DATA_LEN + event.voice_hint_len;
            output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&param_common, (SMLK_UINT8 *)&param_common + PARAM_COMMON_LEN);

            output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&event, (SMLK_UINT8 *)&event + sizeof(SharpSpeedEvent) - MAX_VOICE_HINT_DATA_LEN + event.voice_hint_len);
        }
        SMLK_LOGD("paramid=0x%x, type=%d, sys_param_name(%s).\n", param_id, valueinfo.param_type, sys_param_name.c_str());
    }
    return SMLK_RC::RC_OK;
}

SMLK_RC DissectorMsgTerminalParam::SendMsg0104(IN RctrlHead &head)
{
    SMLK_LOGD("enter in func(%s)\n", __func__);
    vector<SMLK_UINT8> output_vec;
    SMLK_UINT32 param_id;
    Msg0104Head msg0104_head;

    msg0104_head.seq_id = htobe16(head.seq_id);
    msg0104_head.param_num = m_id_map.size();
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&msg0104_head, (SMLK_UINT8 *)&msg0104_head + MSG8104_HEAD_LEN);
    for (auto iter = m_id_map.begin(); iter != m_id_map.end(); ++iter)
    {
        param_id = iter->first;
        ParamValue valueinfo = iter->second;
        GetParamVaule(param_id, valueinfo, output_vec);
    }
    SMLK_LOGD("\033[1;40;35m send to msg id=0x%x to tsp. \033[0m\n", head.msg_id);
    RctrlHead head_temp;
    memcpy(&head_temp, &head, sizeof(RctrlHead));
    head_temp.msg_id = JTT808_TERMINAL_PARAM_GET_RESP;
    head_temp.qos = QOS_SEND_TCP_TIMES;

    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    return SMLK_RC::RC_OK;
}

SMLK_RC DissectorMsgTerminalParam::SendMsg0104(IN RctrlHead &head, vector<SMLK_UINT32> &param_id_vec)
{
    vector<SMLK_UINT8> output_vec;

    SMLK_LOGD("enter in func(%s)\n", __func__);
    Msg0104Head msg0104_head;

    msg0104_head.seq_id = htobe16(head.seq_id);
    msg0104_head.param_num = 0;
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&msg0104_head, (SMLK_UINT8 *)&msg0104_head + MSG8104_HEAD_LEN);

    for (SMLK_UINT8 i = 0; i < param_id_vec.size(); i++)
    {
        auto iter = m_id_map.find(param_id_vec[i]);
        if (iter == m_id_map.end())
        {
            SMLK_LOGW("can not find param_id = 0x%04x in msgdissector\n", param_id_vec[i]);
            continue;
        }
        ParamValue valueinfo = iter->second;
        GetParamVaule(param_id_vec[i], valueinfo, output_vec);
        ++msg0104_head.param_num;
    }
    Msg0104Head *phead = (Msg0104Head *)(output_vec.data());
    phead->param_num = msg0104_head.param_num;
    SMLK_LOGD("\033[1;40;35m send to msg id=0x%04x to tsp. \033[0m\n", head.msg_id);
    RctrlHead head_temp;
    memcpy(&head_temp, &head, sizeof(RctrlHead));
    head_temp.msg_id = JTT808_TERMINAL_PARAM_GET_RESP;
    head_temp.qos = QOS_SEND_TCP_TIMES;
    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/11/27
 * \brief       decode msg id 0x8103
 * \param[out]  indata    message content
 * \param[in]   length     message length
 * \param[in]   queue   remote control queue info
 * \param[in]   is_encode    go to encode or not
 ******************************************************************************/
SMLK_RC DissectorMsgTerminalParam::Msg8103Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode)
{
    SMLK_LOGD("enter in func(%s),queue.m_cmd_from=%d.\n", __func__, (SMLK_UINT8)(queue.m_cmd_from));

    SMLK_UINT8 result = SMLK_TSP_0001_RESULT_SUCCESS;
    Msg8013Head *phead = (Msg8013Head *)indata;

    if (length <= 0)
    {
        SMLK_LOGE("length=%d.\n", length);
        return SMLK_RC::RC_ERROR;
    }
    is_encode = NOT_GOTO_ENCODE;

    SMLK_UINT16 offset = MSG8103_HEAD_LEN;
    for (SMLK_UINT8 num = 0; num < phead->param_num; num++)
    {
        ParamCommon param_com;
        ParamCommon *pparam_com = (ParamCommon *)(indata + offset);
        param_com.param_id = be32toh(pparam_com->param_id);
        param_com.param_len = pparam_com->param_len;
        offset += PARAM_COMMON_LEN;

        auto iter = m_id_map.find(param_com.param_id);
        if (iter == m_id_map.end())
        {
            result = SMLK_TSP_0001_RESULT_NOT_SUPPORT;
            SMLK_LOGW("can not find param_id = 0x%x in msgdissector\n", param_com.param_id);
            break;
        }

        SMLK_LOGD("find param_id = 0x%x in msgdissector\n", param_com.param_id);
        ParamValue valueinfo = iter->second;
        result = SetParamVaule(param_com.param_id, param_com.param_len, (SMLK_UINT8 *)indata + offset, valueinfo);
        if (SMLK_TSP_0001_RESULT_SUCCESS != result)
        {
            SMLK_LOGE("result is error, errcode=%d.\n", result);
            break;
        }
        offset += param_com.param_len;
    }
    // queue.m_head.msg_id = JTT808_COMMON_RESPONSE;
    if (VehCtrlCmdFrom::FROM_TEXT_MESSAGE != queue.m_cmd_from)
    {
        DissectorJtt808Common::getInstance()->SendMsgCommonResp(queue.m_head, result);
    }

    // SMLK_LOGD("indata[0]=%d, length=%d, is_encode=%d.\n",indata[0],length,is_encode);
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/11/27
 * \brief       decode msg id 0x8104
 * \param[out]  indata    message content
 * \param[in]   length     message length
 * \param[in]   queue   remote control queue info
 * \param[in]   is_encode    go to encode or not
 ******************************************************************************/
SMLK_RC DissectorMsgTerminalParam::Msg8104Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode)
{

    SMLK_LOGD("enter in func(%s)\n", __func__);
    is_encode = NOT_GOTO_ENCODE;
    if (0 != length)
    {
        SMLK_LOGE("length=%d,indata[0]=%d, but it should be zero.\n", length, indata[0]);
        return SMLK_RC::RC_ERROR;
    }
    SendMsg0104(queue.m_head);

    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/11/27
 * \brief       decode msg id 0x8106
 * \param[out]  indata    message content
 * \param[in]   length     message length
 * \param[in]   queue   remote control queue info
 * \param[in]   is_encode    go to encode or not
 ******************************************************************************/
SMLK_RC DissectorMsgTerminalParam::Msg8106Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode)
{
    SMLK_UINT8 output[MAX_OUTPUT_LEN];
    SMLK_UINT16 offset = 0;
    SMLK_UINT8 result = 0;
    vector<SMLK_UINT32> param_id_vec;

    is_encode = NOT_GOTO_ENCODE;
    SMLK_LOGD("enter in func(%s), length=%d.\n", __func__, length);

    bzero(output, MAX_OUTPUT_LEN);

    SMLK_UINT8 param_num = *indata;
    offset += sizeof(SMLK_UINT8);
    if (offset + param_num * sizeof(SMLK_UINT32) != length)
    {
        result = SMLK_TSP_0001_RESULT_FAILED;
    }
    else
    {
        for (SMLK_UINT8 num = 0; num < param_num; ++num)
        {
            SMLK_UINT32 param_id = be32toh(*(SMLK_UINT32 *)((SMLK_UINT8 *)indata + offset));
            SMLK_LOGD("search param_id = 0x%04x.\n", param_id);

            auto iter = m_id_map.find(param_id);
            if (iter == m_id_map.end())
            {
                result = SMLK_TSP_0001_RESULT_FAILED;
                SMLK_LOGW("can not find param_id = 0x%04x in msgdissector\n", param_id);
                offset += sizeof(SMLK_UINT32);
                continue;
            }
            param_id_vec.push_back(param_id);
            offset += sizeof(SMLK_UINT32);
        }
    }
    /* if(SMLK_TSP_0001_RESULT_FAILED == result){
        queue.m_head.msg_id = JTT808_COMMON_RESPONSE;
        DissectorJtt808Common::getInstance()->SendMsgCommonResp(queue.m_head, result);

    }else */
    {
        queue.m_head.msg_id = JTT808_TERMINAL_PARAM_GET_RESP;
        SendMsg0104(queue.m_head, param_id_vec);
    }

    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/11/27
 * \brief       decode msg id 0x8103 0x8104 0x8106
 * \param[out]  indata    message content
 * \param[in]   length     message length
 * \param[in]   queue   remote control queue info
 * \param[in]   is_encode    go to encode or not
 ******************************************************************************/
SMLK_RC DissectorMsgTerminalParam::Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode)
{
    SMLK_LOGD("enter in func(%s)\n", __func__);

    is_encode = NOT_GOTO_ENCODE;
    if (JTT808_TERMINAL_PARAM_SET == queue.m_head.msg_id)
    {
        Msg8103Decode(indata, length, queue, is_encode);
    }
    else if (JTT808_TERMINAL_PARAM_GET == queue.m_head.msg_id)
    {
        Msg8104Decode(indata, length, queue, is_encode);
    }
    else if (JTT808_TERMINAL_SPECIFIED_PARAM_GET == queue.m_head.msg_id)
    {
        Msg8106Decode(indata, length, queue, is_encode);
    }
    else
    {
        SMLK_LOGE("queue.m_head.msg_id=0x%x, no such msg id.\n", queue.m_head.msg_id);
    }

    return SMLK_RC::RC_OK;
}

DissectorMsgTerminalParam::DissectorMsgTerminalParam()
{
    ParamValue param_value;

    param_value.param_type = TYPE_DWORD;
    param_value.sys_param_name = SYS_PRO_NAME_TSP_HBT_INTERVAL;
    m_id_map.insert(make_pair(PARAM_HEARTBEAT_INTERVAL_ID, param_value));

    param_value.param_type = TYPE_DWORD;
    param_value.sys_param_name = SYS_PRO_NAME_TSP_TCP_RESP_TIMEOUT;
    m_id_map.insert(make_pair(PARAM_TCP_RESP_TIMEOUT_ID, param_value));

    param_value.param_type = TYPE_DWORD;
    param_value.sys_param_name = SYS_PRO_NAME_SLEEP_REPORT_INTERVAL;
    m_id_map.insert(make_pair(PARAM_SLEEP_REPORT_INTERVAL_ID, param_value));

    param_value.param_type = TYPE_WORD;
    param_value.sys_param_name = SYS_PRO_NAME_FATIGUE_DRIVER_TIME;
    m_id_map.insert(make_pair(PARAM_FATIGUE_DRIVER_TIME_ID, param_value));

    // param_value.param_type = TYPE_WORD;
    // param_value.sys_param_name = SYS_PRO_NAME_REALTIME_DATA_UPLOAD_CYCLE;
    // m_id_map.insert(make_pair(PARAM_REALTIME_DATA_UPLOAD_CYCLE_ID, param_value));

    // param_value.param_type = TYPE_WORD;
    // param_value.sys_param_name = SYS_PRO_NAME_REALTIME_DATA_GATHER_CYCLE;
    // m_id_map.insert(make_pair(PARAM_REALTIME_DATA_GATHER_CYCLE_ID, param_value));

    param_value.param_type = TYPE_WORD;
    param_value.sys_param_name = SYS_PRO_NAME_STATIS_INFO_UPLOAD_CYCLE;
    m_id_map.insert(make_pair(PARAM_STATIS_INFO_UPLOAD_CYCLE_ID, param_value));

    // param_value.param_type = TYPE_WORD;
    // param_value.sys_param_name = SYS_PRO_NAME_LONG_TIME_PARKING_TIME;
    // m_id_map.insert(make_pair(PARAM_LONG_TIME_PARKING_TIME_ID, param_value));

    // param_value.param_type = TYPE_WORD;
    // param_value.sys_param_name = SYS_PRO_NAME_LOW_TO_ULTRALOW_POWER_INTERVAL;
    // m_id_map.insert(make_pair(PARAM_LOW_TO_ULTRALOW_POWER_INTERVAL_ID, param_value));

    param_value.param_type = TYPE_WORD;
    param_value.sys_param_name = SYS_PRO_NAME_NORMAL_TO_LOW_POWER_INTERVAL;
    m_id_map.insert(make_pair(PARAM_NORMAL_TO_LOW_POWER_INTERVAL_ID, param_value));

    param_value.param_type = TYPE_BYTE;
    param_value.sys_param_name = SYS_PRO_NAME_PREDIC_CRUIS_SWITCH;
    m_id_map.insert(make_pair(PARAM_PREDIC_CRUIS_SWITCH_ID, param_value));

    param_value.param_type = TYPE_COMPOUND;
    param_value.sys_param_name = "";
    m_id_map.insert(make_pair(PARAM_OVERSPEED_EVENT_ID, param_value));
    // m_id_map.insert(make_pair(PARAM_RAPIDLY_ACCELERATE_EVENT_ID, param_value));
    // m_id_map.insert(make_pair(PARAM_SHARP_SLOWDOWN_EVENT_ID, param_value));

    param_value.param_type = TYPE_BYTE;
    param_value.sys_param_name = SYS_PRO_NAME_4G_ANTENNA_LOCK;
    m_id_map.insert(make_pair(PARAM_4G_ANTENNA_OPEN_ID, param_value));
}

DissectorMsgTerminalParam::~DissectorMsgTerminalParam()
{
}

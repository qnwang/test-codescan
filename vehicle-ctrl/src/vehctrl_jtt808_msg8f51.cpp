/*****************************************************************************/
/**
 * \file       rctrl_jtt808_msg8f51.cpp
 * \author     huangxin
 * \date       2020/11/25
 * \version    Tbox2.0 V1
 * \brief      jtt808的消息id 0x8f51和0x0f51解析
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
#include "vehctrl_jtt808.h"
#include "vehctrl_queue.h"
#include "tsp_service_api.h"
#include "smlk_property.h"
#include "vehctrl_status_file.h"
#include "vehctrl_jtt808_msg8f51.h"
#include "vehicle_data_index_def.h"
#include "smartlink_sdk_sys_property.h"

using namespace std;
using namespace smartlink;

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/11/27
 * \brief       decode msg id 0x8f51
 * \param[out]  indata       message content
 * \param[in]   length       message length
 * \param[in]   queue        remote control queue info
 * \param[in]   is_encode    go to encode or not
 ******************************************************************************/
SMLK_RC DissectorMsg8F51::Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &query_queue, OUT SMLK_UINT8 &is_encode)
{
    SMLK_UINT8 result = SMLK_TSP_0001_RESULT_SUCCESS;
    Msg8f51Head *pctrl = (Msg8f51Head *)indata;
    // SMLK_LOGD("8F51 msginfo:{ver=%d date:%d%d-%d%d-%d%d %d%d:%d%d:%d%d cmd_num=%d}",
    //           pctrl->version, pctrl->time[0] >> 4, pctrl->time[0] & (0x0f), pctrl->time[1] >> 4, pctrl->time[1] & (0x0f),
    //           pctrl->time[2] >> 4, pctrl->time[2] & (0x0f), pctrl->time[3] >> 4, pctrl->time[3] & (0x0f),
    //           pctrl->time[4] >> 4, pctrl->time[4] & (0x0f), pctrl->time[5] >> 4, pctrl->time[5] & (0x0f), pctrl->cmd_num);
    query_queue.m_version = pctrl->version;
    if ((length - sizeof(Msg8f51Head)) != pctrl->cmd_num)
    {
        SMLK_LOGE("8F51 query num=%d, neq to query len=%d", pctrl->cmd_num, (SMLK_UINT16)(length - sizeof(Msg8f51Head)));
        result = SMLK_TSP_0001_RESULT_OUT_OF_RANGE;
    }
    else
    {
        for (auto i = 0; i < pctrl->cmd_num; ++i)
        {
            SMLK_UINT8 query_id = *(indata + sizeof(Msg8f51Head) + i);
            query_queue.query_queue.m_query_vec.emplace_back((SMLK_UINT16)query_id);
        }
    }
    DissectorJtt808Common::getInstance()->SendMsgCommonResp(query_queue.m_head, result);
    if (SMLK_TSP_0001_RESULT_SUCCESS == result)
    {
        /*设置消息状态*/
        query_queue.m_head.msg_id = JTT808_VEHICAL_QUERY_RESP;    /*设置消息id    8F51*/
        query_queue.m_cmd_type = RctrlCmdType::REMCTRL_CMD_GET;   /*设置消息类型  查询*/
        query_queue.m_cmd_from = VehCtrlCmdFrom::FROM_TSP_MODULE; /*设置消息来源  TSP*/
        is_encode = GOTO_ENCODE;
    }
    else
    {
        is_encode = NOT_GOTO_ENCODE;
    }
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/22
 * \brief       将远控的车态获取命令转化成808的指令
 * \param[in]   result   远控获取的车态结果
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F51::ResultEncode(IN RctrlResultQueue &result)
{
    std::vector<SMLK_UINT8> output_vec;
    Msg0F51Head head;
    bzero(&head, sizeof(Msg0F51Head));
    head.version = result.m_version;
    tools::get_bcd_timestamp(head.time);
    /*结果中的消息来源字段在ThreadCmdQueueProcess线程中赋值,跟队列消息一致*/
    if (VehCtrlCmdFrom::FROM_AUTO_REPORTED == result.m_cmd_from)
    {
        SMLK_LOGD("[0F51][MsgType]:Auto report");
        head.flag = (Msg0f51Flag::EVENT_REPORT);
        head.seq_id = 0xffff;
    }
    else
    {
        SMLK_LOGD("[0F51][MsgType]:Tsp/text report");
        head.flag = (Msg0f51Flag::QUERY_RESP);
        head.seq_id = htobe16(result.m_head.seq_id);
    }
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&head, (SMLK_UINT8 *)&head + sizeof(Msg0F51Head));
    VehctrlGetCmd vehicle_result;
    bzero(&vehicle_result, sizeof(VehctrlGetCmd));
    for (auto i = 0; i < result.query_result.m_query_id.size(); ++i)
    {
        SMLK_LOGD("[%dth][QueryID]=0x%02x", i, result.query_result.m_query_id[i]);
        switch (result.query_result.m_query_id[i])
        {
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_DOOR: /*车门锁状态0x01*/
            EncodeDoorStatus(result.query_result.m_query_vec, output_vec);
            break;
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_ENGINE: /*发动机状态0x06*/
            EncodeEngineModeStatus(result.query_result.m_query_vec, output_vec);
            break;
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_POWER: /*车辆电源状态0x07*/
            EncodePowerModeStatus(result.query_result.m_query_vec, output_vec);
            break;
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_ORIGINAL_AC: /*原车空调状态0x09*/
            EncodeOriginalACStatus(result.query_result.m_query_vec, output_vec);
            break;
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_INDENTP_WARM_AIR: /*独立暖风状态0x10*/
            EncodeIndentpWarmAirStatus(result.query_result.m_query_vec, output_vec);
            break;
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_PARKING_AC: /*驻车空调状态0x11*/
            EncodeParkingAcStatus(result.query_result.m_query_vec, output_vec);
            break;
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_OILTANK_SECURITY: /*油箱防盗开关0x12*/
            EncodeOilTankStatus(result.query_result.m_query_vec, output_vec);
            break;
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_WAKEUP_SOURCE: /*唤醒源0x25 不支持主动查询 支持主动上报*/
            EncodeWakeupSourceStatus(result.query_result.m_query_vec, output_vec);
            break;
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_INTELLIGENT_RUC: /*智能冷机状态0x26*/
            EncodeIntelligentRucStatus(result.query_result.m_query_vec, output_vec);
            break;
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_CURRENT_PS: /*当前ps状态0x27*/
            EncodeCurrentPsStatus(result.query_result.m_query_vec, output_vec);
            break;
        default:
            break;
        }
    }
    Msg0F51Head *phead = (Msg0F51Head *)(output_vec.data());
    phead->query_num = result.query_result.m_query_id.size();
    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(result.m_head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      Xiongyijun
 * \date        2022/03/16
 * \brief       车门状态查询组包
 * \param[out]  query_vec        远控获取的车态结果
 * \param[in]   output_vec       上报0F51内容
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F51::EncodeDoorStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51Door door;
    bzero(&door, sizeof(Msg0f51Door));
    door.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_DOOR;
    door.length = MSG0F51_DOOR_STA_LEN;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (VEHICLE_DATA_LOCK_STATUS_OF_DOOR_DRIVER == query_vec[i].query_id /*驾驶侧车门开解锁状态*/
            || VEHICLE_DATA_LOCK_STATUS_OF_DOOR_PASSENGER == query_vec[i].query_id /*乘客侧车门开解锁状态*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth door res{cmd=%d data=%g}",i, m_query_vec[i].query_id, m_query_vec[i].value);
        if (VEHICLE_DATA_LOCK_STATUS_OF_DOOR_DRIVER == m_query_vec[i].query_id) /*查询驾驶侧车门上锁解锁状态*/
        {
            if (SMLK_TSP_ACTION_DOOR_UNLOCK == m_query_vec[i].value)
            {
                door.driver_door = Query_Door_Status::DOOR_LOCK;
            }
            else if (SMLK_TSP_ACTION_DOOR_LOCK == m_query_vec[i].value)
            {
                door.driver_door = Query_Door_Status::DOOR_UNLOCK;
            }
            else
            {
                door.driver_door = Query_Door_Status::DOOR_INVALID;
            }
        }
        else if (VEHICLE_DATA_LOCK_STATUS_OF_DOOR_PASSENGER == m_query_vec[i].query_id) /*查询乘客侧车门上锁解锁状态*/
        {
            if (SMLK_TSP_ACTION_DOOR_UNLOCK == m_query_vec[i].value)
            {
                door.passenger_door = Query_Door_Status::DOOR_LOCK;
            }
            else if (SMLK_TSP_ACTION_DOOR_LOCK == m_query_vec[i].value)
            {
                door.passenger_door = Query_Door_Status::DOOR_UNLOCK;
            }
            else
            {
                door.passenger_door = Query_Door_Status::DOOR_INVALID;
            }
        }
    }
    // std::string log = tools::uint8_2_string((SMLK_UINT8*)&door,sizeof(Msg0f51Door));
    // SMLK_LOGD("Door info=%s len=%d",log.c_str(),sizeof(Msg0f51Door));
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&door, (SMLK_UINT8 *)&door + sizeof(Msg0f51Door));
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      Xiongyijun
 * \date        2022/03/16
 * \brief       发动机状态查询组包
 * \param[out]  query_vec        远控获取的车态结果
 * \param[in]   output_vec       上报0F51内容
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F51::EncodeEngineModeStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51CommonStruct common;
    bzero(&common, sizeof(Msg0f51CommonStruct));
    common.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_ENGINE;
    common.length = MSG0F51_ENGINE_MODE_LEN;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (VEHICLE_DATA_ENGINE_SPEED == query_vec[i].query_id /*发动机转速*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth engine mode res{cmd=%d data=%g}",i, m_query_vec[i].query_id, m_query_vec[i].value);
        common.status = m_query_vec[i].value;
    }
    // std::string log = tools::uint8_2_string((SMLK_UINT8*)&common, sizeof(Msg0f51CommonStruct));
    // SMLK_LOGD("Engine mode info=%s",log.c_str());
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&common, (SMLK_UINT8 *)&common + sizeof(Msg0f51CommonStruct));
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      Xiongyijun
 * \date        2022/03/16
 * \brief       电源状态查询组包
 * \param[out]  query_vec        远控获取的车态结果
 * \param[in]   output_vec       上报0F51内容
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F51::EncodePowerModeStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51CommonStruct common;
    bzero(&common, sizeof(Msg0f51CommonStruct));
    common.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_POWER;
    common.length = MSG0F51_POWER_MODE_LEN;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_POWER_MODE == query_vec[i].query_id /*发动机转速*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth power mode res{cmd=%d data=%g}",i, m_query_vec[i].query_id, m_query_vec[i].value);
        common.status = m_query_vec[i].value;
    }
    // std::string log = tools::uint8_2_string((SMLK_UINT8*)&common, sizeof(Msg0f51CommonStruct));
    // SMLK_LOGD("Power mode info=%s",log.c_str());
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&common, (SMLK_UINT8 *)&common + sizeof(Msg0f51CommonStruct));
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      Xiongyijun
 * \date        2022/03/16
 * \brief       原车空调状态查询组包
 * \param[out]  query_vec        远控获取的车态结果
 * \param[in]   output_vec       上报0F51内容
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F51::EncodeOriginalACStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51OriginalAC ac;
    bzero(&ac, sizeof(Msg0f51OriginalAC));
    ac.ac_switch_status = DATA_INVALID_UINT8;
    ac.ac_airoutlet_mode = DATA_INVALID_UINT8;
    ac.ac_temper = DATA_INVALID_UINT8;
    ac.ac_wind = DATA_INVALID_UINT8;
    ac.ac_loop_mode = DATA_INVALID_UINT8;
    ac.ac_compressor = DATA_INVALID_UINT8;
    ac.ac_auto_mode = DATA_INVALID_UINT8;
    ac.ac_ventilation = DATA_INVALID_UINT8;
    ac.ac_defrost = DATA_INVALID_UINT8;
    ac.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_ORIGINAL_AC;
    ac.length = MSG0F51_ORI_AC_LEN;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS == query_vec[i].query_id              /*原车空调开关状态*/
            || VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS == query_vec[i].query_id             /*原车空调出风模式*/
            || VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE == query_vec[i].query_id            /*原车空调温度*/
            || VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE == query_vec[i].query_id            /*原车空调风量*/
            || VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS == query_vec[i].query_id /*原车空调循环模式*/
            || VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS == query_vec[i].query_id          /*原车空调压缩机状态*/
            || VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS == query_vec[i].query_id        /*原车空调auto状态*/
            || VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS == query_vec[i].query_id /*原车空调一键通风状态*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth ori ac res{cmd=%d data=%g}",i, m_query_vec[i].query_id, m_query_vec[i].value);
        if (VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS == m_query_vec[i].query_id)
        {
            ac.ac_switch_status = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS == m_query_vec[i].query_id) /*原车空调出风模式 + 强制除霜状态*/
        {
            if (SMLK_TSP_ACTION_BLOW_FACE == m_query_vec[i].value)
            {
                ac.ac_airoutlet_mode = SMLK_TSP_ACTION_BLOW_FACE;
                ac.ac_defrost = SMLK_TSP_ACTION_OFF;
            }
            else if (SMLK_TSP_ACTION_BLOW_FACE_FEET == m_query_vec[i].value)
            {
                ac.ac_airoutlet_mode = SMLK_TSP_ACTION_BLOW_FACE_FEET;
                ac.ac_defrost = SMLK_TSP_ACTION_OFF;
            }
            else if (SMLK_TSP_ACTION_BLOW_FEET == m_query_vec[i].value)
            {
                ac.ac_airoutlet_mode = SMLK_TSP_ACTION_BLOW_FEET;
                ac.ac_defrost = SMLK_TSP_ACTION_OFF;
            }
            else if (SMLK_TSP_ACTION_BLOW_FEET_DEFROST == m_query_vec[i].value)
            {
                ac.ac_airoutlet_mode = SMLK_TSP_ACTION_BLOW_FEET_DEFROST;
                ac.ac_defrost = SMLK_TSP_ACTION_OFF;
            }
            else if (SMLK_VD_ORIAC_BLOW_DEFROST == m_query_vec[i].value)
            {
                ac.ac_airoutlet_mode = DATA_INVALID_UINT8;
                ac.ac_defrost = SMLK_TSP_ACTION_ON;
            }
            else
            {
                ac.ac_airoutlet_mode = DATA_INVALID_UINT8;
                ac.ac_defrost = DATA_INVALID_UINT8;
            }
        }
        else if (VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE == m_query_vec[i].query_id)
        {
            if (DATA_INVALID_UINT8 == m_query_vec[i].value)
            {
                ac.ac_temper = DATA_INVALID_UINT8;
            }
            else
            {
                ac.ac_temper = SMLK_UINT8((m_query_vec[i].value - 17) * 2);
            }
        }
        else if (VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE == m_query_vec[i].query_id)
        {
            ac.ac_wind = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS == m_query_vec[i].query_id)
        {
            ac.ac_loop_mode = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS == m_query_vec[i].query_id)
        {
            ac.ac_compressor = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS == m_query_vec[i].query_id)
        {
            ac.ac_auto_mode = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS == m_query_vec[i].query_id)
        {
            ac.ac_ventilation = m_query_vec[i].value;
        }
    }
    // std::string log = tools::uint8_2_string((SMLK_UINT8*)&ac, sizeof(Msg0f51OriginalAC));
    // SMLK_LOGD("Ori AC info=%s",log.c_str());
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&ac, (SMLK_UINT8 *)&ac + sizeof(Msg0f51OriginalAC));
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      Xiongyijun
 * \date        2022/03/16
 * \brief       独立暖风状态查询组包
 * \param[out]  query_vec        远控获取的车态结果
 * \param[in]   output_vec       上报0F51内容
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F51::EncodeIndentpWarmAirStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51IndenptWarmAir warm_air;
    bzero(&warm_air, sizeof(Msg0f51IndenptWarmAir));
    warm_air.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_INDENTP_WARM_AIR;
    warm_air.length = MSG0F51_INDENPT_WARMAIR_LEN;
    warm_air.switch_status = DATA_INVALID_UINT8;
    warm_air.temper = 0xFF;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS == query_vec[i].query_id /*独立暖风开关*/
            || VEHICLE_DATA_WARM_AIR_TEMPERATURE == query_vec[i].query_id /*独立暖风温度*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth idp warm res{cmd=%d data=%g}",i, m_query_vec[i].query_id, m_query_vec[i].value);
        if (VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS == m_query_vec[i].query_id)
        {
            warm_air.switch_status = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_WARM_AIR_TEMPERATURE == m_query_vec[i].query_id)
        {
            warm_air.temper = m_query_vec[i].value;
        }
    }
    // std::string log = tools::uint8_2_string((SMLK_UINT8*)&warm_air, sizeof(Msg0f51IndenptWarmAir));
    // SMLK_LOGD("Idp Warm info=%s",log.c_str());
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&warm_air, (SMLK_UINT8 *)&warm_air + sizeof(Msg0f51IndenptWarmAir));
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      Xiongyijun
 * \date        2022/03/16
 * \brief       驻车空调状态查询组包
 * \param[out]  query_vec        远控获取的车态结果
 * \param[in]   output_vec       上报0F51内容
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F51::EncodeParkingAcStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51ParkingAC parking_ac;
    bzero(&parking_ac, sizeof(Msg0f51ParkingAC));
    parking_ac.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_PARKING_AC;
    parking_ac.length = MSG0F51_PARKING_AC_LEN;
    parking_ac.switch_status = DATA_INVALID_UINT8;
    parking_ac.winder = DATA_INVALID_UINT8;
    parking_ac.temper = DATA_INVALID_UINT8;
    parking_ac.auto_switch_status = DATA_INVALID_UINT8;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS == query_vec[i].query_id   /*驻车空调开关*/
            || VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS == query_vec[i].query_id          /*驻车空调auto开关*/
            || VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE == query_vec[i].query_id /*驻车空调温度*/
            || VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE == query_vec[i].query_id /*驻车空调风量*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth park ac res{cmd=%d data=%g}",i, m_query_vec[i].query_id, m_query_vec[i].value);
        if (VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS == m_query_vec[i].query_id)
        {
            parking_ac.switch_status = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE == m_query_vec[i].query_id)
        {
            parking_ac.winder = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE == m_query_vec[i].query_id)
        {
            if (DATA_INVALID_UINT8 == m_query_vec[i].value)
            {
                parking_ac.temper = m_query_vec[i].value;
            }
            else
            {
                parking_ac.temper = SMLK_UINT8((m_query_vec[i].value - 17) * 2);
            }
        }
        else if (VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS == m_query_vec[i].query_id)
        {
            parking_ac.auto_switch_status = m_query_vec[i].value;
        }
    }
    // std::string log = tools::uint8_2_string((SMLK_UINT8*)&parking_ac, sizeof(Msg0f51ParkingAC));
    // SMLK_LOGD("Park AC info=%s",log.c_str());
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&parking_ac, (SMLK_UINT8 *)&parking_ac + sizeof(Msg0f51ParkingAC));
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      Xiongyijun
 * \date        2022/03/16
 * \brief       油箱防盗开关状态查询组包
 * \param[out]  query_vec        远控获取的车态结果
 * \param[in]   output_vec       上报0F51内容
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F51::EncodeOilTankStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51CommonStruct common;
    bzero(&common, sizeof(Msg0f51CommonStruct));
    common.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_OILTANK_SECURITY;
    common.length = MSG0F51_OILTANK_LEN;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (VEHICLE_DATA_DRIVER_TANK_ANTI_THEFT_STATUS == query_vec[i].query_id /*油箱防盗开关*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth oiltank res{cmd=%d data=%g}",i, m_query_vec[i].query_id, m_query_vec[i].value);
        common.status = m_query_vec[i].value;
    }
    // std::string log = tools::uint8_2_string((SMLK_UINT8*)&common, sizeof(Msg0f51CommonStruct));
    // SMLK_LOGD("Oiltank info=%s",log.c_str());
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&common, (SMLK_UINT8 *)&common + sizeof(Msg0f51CommonStruct));
    return SMLK_RC::RC_OK;
}

SMLK_RC DissectorMsg8F51::EncodeWakeupSourceStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51CommonStruct common;
    bzero(&common, sizeof(Msg0f51CommonStruct));
    common.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_WAKEUP_SOURCE;
    common.length = MSG0F51_WAKEUPSOURCE_LEN;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_WAKEUP_SOURCE == query_vec[i].query_id /*唤醒源*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth wakeup source res{cmd=%d data=%g}",i, m_query_vec[i].query_id, m_query_vec[i].value);
        common.status = (SMLK_UINT8)m_query_vec[i].value;
    }
    // std::string log = tools::uint8_2_string((SMLK_UINT8*)&common, sizeof(Msg0f51CommonStruct));
    // SMLK_LOGD("Wakeup source info=%s",log.c_str());
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&common, (SMLK_UINT8 *)&common + sizeof(Msg0f51CommonStruct));
    return SMLK_RC::RC_OK;
}

SMLK_RC DissectorMsg8F51::EncodeIntelligentRucStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51IntelligentRUC ruc;
    bzero(&ruc, sizeof(Msg0f51IntelligentRUC));
    ruc.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_INTELLIGENT_RUC;
    ruc.length = MSG0F51_INTELLIGENT_RUC_LEN;
    ruc.rctrl_allowed_state = DATA_INVALID_UINT8;
    ruc.ruc_switch_state = DATA_INVALID_UINT8;
    ruc.defrost_allowed_state = DATA_INVALID_UINT8;
    ruc.defrost_work_mode = DATA_INVALID_UINT8;
    ruc.ruc_regulation_temper = DATA_INVALID_UINT8;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (VEHICLE_DATA_TEMPERATURE_REMOTE_OPERATION_ALLOWED_STATE == query_vec[i].query_id /*冷机远程可控制状态*/
            || VEHICLE_DATA_REFRIGERATION_SWITCH_STATUS == query_vec[i].query_id             /*制冷机组开关状态*/
            || VEHICLE_DATA_DEFROST_ALLOWED_STATE == query_vec[i].query_id                   /*除霜允许状态*/
            || VEHICLE_DATA_REFRIGERATION_WORK_MODE == query_vec[i].query_id                 /*除霜工作模式*/
            || VEHICLE_DATA_RUC_REGULATION_TEMPERATURE == query_vec[i].query_id /*冷机当前设置的温度*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth ruc res{cmd=%d data=%g}", i, m_query_vec[i].query_id, m_query_vec[i].value);
        if (VEHICLE_DATA_TEMPERATURE_REMOTE_OPERATION_ALLOWED_STATE == m_query_vec[i].query_id) /*冷机远程可控制状态*/
        {
            ruc.rctrl_allowed_state = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_REFRIGERATION_SWITCH_STATUS == m_query_vec[i].query_id) /*制冷机组开关状态*/
        {
            ruc.ruc_switch_state = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_DEFROST_ALLOWED_STATE == m_query_vec[i].query_id) /*除霜允许状态*/
        {
            ruc.defrost_allowed_state = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_REFRIGERATION_WORK_MODE == m_query_vec[i].query_id) /*除霜工作模式*/
        {
            ruc.defrost_work_mode = m_query_vec[i].value;
        }
        else if (VEHICLE_DATA_RUC_REGULATION_TEMPERATURE == m_query_vec[i].query_id) /*冷机当前设置的温度*/
        {
            ruc.ruc_regulation_temper = m_query_vec[i].value;
        }
    }
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&ruc, (SMLK_UINT8 *)&ruc + sizeof(Msg0f51IntelligentRUC));
    return SMLK_RC::RC_OK;
}

SMLK_RC DissectorMsg8F51::EncodeCurrentPsStatus(IN std::vector<VehctrlGetCmd> &query_vec, OUT vector<SMLK_UINT8> &output_vec)
{
    // SMLK_LOGD("Enter(%s)", __func__);
    Msg0f51CommonStruct common;
    bzero(&common, sizeof(Msg0f51CommonStruct));
    common.query_id = Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_CURRENT_PS;
    common.length = MSG0F51_PS_LEN;
    std::vector<VehctrlGetCmd> m_query_vec;
    for (auto i = 0; i < query_vec.size(); ++i)
    {
        if (SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_PS_CTRL_MODE == query_vec[i].query_id /*PS远控状态*/)
        {
            m_query_vec.emplace_back(query_vec[i]);
        }
    }
    for (auto i = 0; i < m_query_vec.size(); ++i)
    {
        // SMLK_LOGD("%dth ps res{cmd=%d data=%g}", i, m_query_vec[i].query_id, m_query_vec[i].value);
        common.status = (SMLK_UINT8)m_query_vec[i].value;
    }
    SMLK_UINT16 reserved = 0x0000;
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&common, (SMLK_UINT8 *)&common + sizeof(Msg0f51CommonStruct));
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&reserved, (SMLK_UINT8 *)&reserved + sizeof(SMLK_UINT16));
    return SMLK_RC::RC_OK;
}

SMLK_RC DissectorMsg8F51::OnChanged(IN SMLK_UINT8 &data_type, IN SMLK_UINT16 &query_id, IN SMLK_DOUBLE &data)
{
    vector<SMLK_UINT8> output_vec;
    /*组包头*/
    RctrlHead rctrl_head;
    rctrl_head.protocol = ProtoclID::E_PROT_JTT808;
    rctrl_head.msg_id = JTT808_VEHICAL_QUERY_RESP;
    rctrl_head.seq_id = m_seq_id;
    rctrl_head.qos = QOS_SEND_TCP_TIMES;
    rctrl_head.priority = PRIORITY7;
    Msg0F51Head head;
    bzero(&head, sizeof(Msg0F51Head));
    head.version = 0;
    head.flag = (Msg0f51Flag::EVENT_REPORT);
    head.seq_id = htobe16(0xffff);
    head.query_num = 1;
    tools::get_bcd_timestamp(head.time);
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&head, (SMLK_UINT8 *)&head + sizeof(Msg0F51Head));
    /*包头组织完成,继续组织内容*/
    switch ((RctrlMsgType)data_type) /*判断消息来源*/
    {
    case RctrlMsgType::MSG_TYPE_VD_STATUS:
    {
        VehctrlGetCmd result;
        result.query_id = query_id;
        result.value = data;
        std::vector<VehctrlGetCmd> result_vec;
        result_vec.push_back(result);
        switch (query_id)
        {
        /*车门状态主动上报*/
        case VEHICLE_DATA_LOCK_STATUS_OF_DOOR_DRIVER:    /*驾驶侧车门开解锁状态*/
        case VEHICLE_DATA_LOCK_STATUS_OF_DOOR_PASSENGER: /*乘客侧车门开解锁状态*/
            EncodeDoorStatus(result_vec, output_vec);
            break;
        /*发动机状态主动上报*/
        case VEHICLE_DATA_ENGINE_SPEED: /*发动机转速*/
            EncodeEngineModeStatus(result_vec, output_vec);
            break;
        /*原车空调主动上报*/
        case VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS:           /*原车空调开关状态*/
        case VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS:             /*原车空调出风模式*/
        case VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE:            /*原车空调温度*/
        case VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE:            /*原车空调风量*/
        case VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS: /*原车空调循环模式*/
        case VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS:          /*原车空调压缩机状态*/
        case VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS:        /*原车空调auto状态*/
        case VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS:      /*原车空调一键通风状态*/
            EncodeOriginalACStatus(result_vec, output_vec);
            break;
        /*独立暖风状态主动上报*/
        case VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS: /*独立暖风开关*/
        case VEHICLE_DATA_WARM_AIR_TEMPERATURE:          /*独立暖风温度*/
            EncodeIndentpWarmAirStatus(result_vec, output_vec);
            break;
        /*驻车空调状态主动上报*/
        case VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS: /*驻车空调开关*/
        case VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS:           /*驻车空调auto开关*/
        case VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE:  /*驻车空调温度*/
        case VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE:  /*驻车空调风量*/
            EncodeParkingAcStatus(result_vec, output_vec);
            break;
        /*油箱防盗开关主动上报*/
        case VEHICLE_DATA_DRIVER_TANK_ANTI_THEFT_STATUS: /*油箱防盗开关*/
            EncodeOilTankStatus(result_vec, output_vec);
            break;
            /*智能冷机状态主动上报*/
        case VEHICLE_DATA_TEMPERATURE_REMOTE_OPERATION_ALLOWED_STATE:
        case VEHICLE_DATA_REFRIGERATION_SWITCH_STATUS:
        case VEHICLE_DATA_DEFROST_ALLOWED_STATE:
        case VEHICLE_DATA_REFRIGERATION_WORK_MODE:
        case VEHICLE_DATA_RUC_REGULATION_TEMPERATURE:
            EncodeIntelligentRucStatus(result_vec, output_vec);
            break;
        default:
            break;
        }
    }
    case RctrlMsgType::MSG_TYPE_OTHER:
    {
        VehctrlGetCmd result;
        result.query_id = query_id;
        result.value = data;
        std::vector<VehctrlGetCmd> result_vec;
        result_vec.push_back(result);
        switch (query_id)
        {
        /*车辆电源主动上报 直接读取本地IGN状态*/
        case SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_POWER_MODE: /*电源状态*/
            EncodePowerModeStatus(result_vec, output_vec);
            break;
        /*唤醒源变化主动上报*/
        case SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_WAKEUP_SOURCE: /*唤醒源*/
            EncodeWakeupSourceStatus(result_vec, output_vec);
            break;
        /*当前ps状态主动上报*/
        case SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_PS_CTRL_MODE: /*ps控制状态*/
            EncodeCurrentPsStatus(result_vec, output_vec);
            break;
        default:
            break;
        }
    }
    }
    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(rctrl_head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    ++m_seq_id;
    return SMLK_RC::RC_OK;
}

DissectorMsg8F51::DissectorMsg8F51()
{
}

DissectorMsg8F51::~DissectorMsg8F51()
{
}

/*****************************************************************************/
/**
 * \file       rctrl_jtt808_msg8f41.cpp
 * \author     huangxin
 * \date       2020/11/25
 * \version    Tbox2.0 V1
 * \brief      jtt808的消息id 0x8f41和0x0f41解析
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
#include "vehctrl_jtt808_msg8f41.h"
#include "vehctrl_queue.h"
#include "vehctrl_queue.h"
#include "smlk_property.h"
#include "smartlink_sdk_sys_property.h"
#include "vehctrl_status_file.h"
#include "smlk_tools.h"
using namespace std;
using namespace smartlink;
using namespace smartlink_sdk;

/*****************************************************************************/
/**
 * \author      xiongyijun
 * \date        2022/04/08
 * \brief       8F41消息解析
 * \param[out]  indata      消息内容
 * \param[in]   length      消息长度
 * \param[in]   queue       消息队列
 * \param[in]   is_encode   编码标记位
 ******************************************************************************/
SMLK_RC DissectorMsg8F41::Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode)
{
    Msg8F41_Head *phead = (Msg8F41_Head *)indata;
    SMLK_UINT8 result = SMLK_TSP_0001_RESULT_SUCCESS;
    SmlkMcuCmd mcu_cmd;
    SMLK_UINT8 loan_enable_rc = SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_ERROR; /*消贷功能使能禁止结果*/
    SMLK_UINT16 cmd_len = length - sizeof(Msg8F41_Head);
    SMLK_UINT8 cmd_num = phead->cmd_num;
    bool param_err_flag = false; /*若指令的参数不对,返回通用应答*/
    is_encode = NOT_GOTO_ENCODE;
    // SMLK_LOGD("Msg={[ver]=%d [date]=%02x-%02x-%02x %02x:%02x:%02x [cmd_num]=%d}", phead->version, phead->time[0], phead->time[1],
    //           phead->time[2], phead->time[3], phead->time[4], phead->time[5], phead->cmd_num);
    queue.m_version = phead->version;
    /*来自TSP的消息在解析完成后回复通用应答*/
    if (VehCtrlCmdFrom::FROM_TSP_MODULE == queue.m_cmd_from)
    {
        DissectorJtt808Common::getInstance()->SendMsgCommonResp(queue.m_head, result);
    }
    /*消息长度校验*/
    if (cmd_len != cmd_num * sizeof(Msg8F41_Body))
    {
        SMLK_LOGE("Recv 8F41 msg_body len=%d mismatch!", length);
        result = SMLK_TSP_0001_RESULT_NOT_SUPPORT;
    }
    else /*控车指令解析*/
    {
        const Msg8F41_Body *tsp_body = (Msg8F41_Body *)(indata + sizeof(Msg8F41_Head));
        queue.config_queue.m_cmd_vec.clear();
        for (SMLK_UINT8 i = 0; i < cmd_num; ++i)
        {
            mcu_cmd.cmd_act = DATA_INVALID_UINT8;
            mcu_cmd.cmd_data_len = 0;
            mcu_cmd.cmd_data_cont.clear();
            Msg8F41_Body tsp_msg_temp;
            tsp_msg_temp.id = tsp_body->id;
            tsp_msg_temp.cmd = tsp_body->cmd;
            tsp_msg_temp.param = tsp_body->param;
            queue.config_queue.m_tsp_cmd_vec.emplace_back(tsp_msg_temp);
            switch (tsp_body->id)
            {
            case Jtt808_8F41_SubCmd::JTT808_8F41_DOOR_CTRL: /*车门控制*/
                Msg8F41Door(tsp_body->cmd, mcu_cmd, param_err_flag);
                break;
            case Jtt808_8F41_SubCmd::JTT808_8F41_DOUBLE_FLASHING_CTRL: /*双闪控制*/
            {
                mcu_cmd.cmd_id = SMLK_MCU_CMD_DOUBLE_FLASHING;
                mcu_cmd.cmd_act = DATA_INVALID_UINT8;
                mcu_cmd.cmd_data_len = 0;
                param_err_flag = false;
            }
            break;
            case Jtt808_8F41_SubCmd::JTT808_8F41_ENGINE_CTRL: /*发动机启停*/
                Msg8F41Engine(tsp_body->cmd, mcu_cmd, param_err_flag);
                break;
            case Jtt808_8F41_SubCmd::JTT808_8F41_AIRCONDITION_CTRL: /*空调控制*/
                Msg8F41AC(tsp_body->cmd, tsp_body->param, mcu_cmd, param_err_flag);
                break;
            case Jtt808_8F41_SubCmd::JTT808_8F41_REARVIEW_MIRROR_CTRL: /*后视镜控制*/
                Msg8F41Mirror(tsp_body->cmd, tsp_body->param, mcu_cmd, param_err_flag);
                break;
            case Jtt808_8F41_SubCmd::JTT808_8F41_FINANCIAL_LOCK_CTRL: /*金融锁车功能打开or关闭*/
                loan_enable_rc = Msg8F41FinancialLckCar(tsp_body->cmd, mcu_cmd, param_err_flag);
                break;
            case Jtt808_8F41_SubCmd::JTT808_8F41_OILTANK_SECURITY_CTRL: /*油箱防盗开关打开or关闭*/
                Msg8F41OilTankAntiSheftSwitch(tsp_body->cmd, mcu_cmd, param_err_flag);
                break;
            case Jtt808_8F41_SubCmd::JTT808_8F41_IDLING_WARM_UP: /*怠速暖机开关打开or关闭*/
                Msg8F41IdlingWarmUp(tsp_body->cmd, mcu_cmd, param_err_flag);
                break;
            case Jtt808_8F41_SubCmd::JTT808_8F41_INTELLIGNT_RUC: /*智能冷机控制*/
                Msg8F41Ruc(tsp_body->cmd, tsp_body->param, mcu_cmd, param_err_flag);
                break;
            case Jtt808_8F41_SubCmd::JTT808_8F41_AUTO_TEMPER_SET: /*一键温暖&一键清凉设置*/
                Msg8F41AutoTemper(tsp_body->cmd, tsp_body->param, mcu_cmd, param_err_flag);
                break;
            default:
                param_err_flag = true;
                break;
            }
            SMLK_LOGD("Tsp{id=%d sub=%d param=%d}->Mcu{id=0x%02x act=%d len=%d data=%s}", tsp_body->id, tsp_body->cmd, tsp_body->param, mcu_cmd.cmd_id, mcu_cmd.cmd_act, mcu_cmd.cmd_data_len,
                      (mcu_cmd.cmd_data_len == 0) ? "null" : tools::uint8_2_string(mcu_cmd.cmd_data_cont.data(), mcu_cmd.cmd_data_len).c_str());
            /*平台下发参数错误*/
            if (true == param_err_flag)
            {
                SMLK_LOGD("Tsp Msg Param Error!");
                SendMsg0F41(queue.m_head, indata, length);
                is_encode = NOT_GOTO_ENCODE;
                return SMLK_RC::RC_ERROR;
            }

            /*非金融锁车功能开关使能禁止指令->下发给mcu; 开关使能禁止指令->不需要下发给mcu设置参数即可*/
            if (SMLK_MCU_CMD_FINANCIAL_LOCK_FUNC_ENABLE != mcu_cmd.cmd_id)
            {
                queue.config_queue.m_cmd_vec.push_back(mcu_cmd);
            }
            ++tsp_body;
        }
    }

    if (queue.config_queue.m_cmd_vec.size() > 0)
    {
        /*参数无误,继续下发指令至mcu*/
        queue.m_head.msg_id = JTT808_REMOTE_CTRL_RESP;
        queue.m_cmd_type = RctrlCmdType::REMCTRL_CMD_SET;
        is_encode = GOTO_ENCODE;
    }

    if ((SMLK_MCU_CMD_FINANCIAL_LOCK_FUNC_ENABLE == mcu_cmd.cmd_id) && (!param_err_flag))
    {
        SendMsg0F41(queue.m_head, indata, length, loan_enable_rc);
    }
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/11/05
 * \brief       encode mcu result info ,and send msg id 0x0f41
 * \param[in]   result_info:   include: remte_ctrl_head + cmd_result_list
 * \remarks
 ******************************************************************************/
SMLK_RC DissectorMsg8F41::ResultEncode(IN RctrlResultQueue &result)
{
    vector<SMLK_UINT8> output_vec;
    /*0F41响应组包头部信息*/
    Msg0F41_Head head_0f41;
    bzero(&head_0f41, sizeof(Msg0F41_Head));
    head_0f41.version = result.m_version;
    tools::get_bcd_timestamp(head_0f41.time);
    head_0f41.seq_id = be16toh(result.m_head.seq_id);
    head_0f41.cmd_num = result.config_result.m_mcu_res_vec.size();
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&head_0f41, (SMLK_UINT8 *)&head_0f41 + sizeof(Msg0F41_Head));
    /*0F41响应组包控车结果信息*/
    for (SMLK_UINT16 i = 0; i < result.config_result.m_mcu_res_vec.size(); i++)
    {
        SmlkMcuCmdResp mcu_resp_temp;
        Msg0F41_Body resp_body;
        bzero(&resp_body, sizeof(Msg0F41_Body));
        Msg8F41_Body *p_cmd_body = (Msg8F41_Body *)&result.config_result.m_tsp_cmd_vec[i];
        SmlkMcuCmdResp *p_mcu_resp = (SmlkMcuCmdResp *)&result.config_result.m_mcu_res_vec[i];
        SmlkRctrl_8F41_Result tsp_resp = result.config_result.m_tsp_result[i];
        resp_body.id = p_cmd_body->id;
        resp_body.cmd = p_cmd_body->cmd;
        resp_body.result = tsp_resp;
        SMLK_LOGD("Mcu{cmd=%d act=%d}->Tsp{cmd=%d sub=%d res=%d}", p_mcu_resp->cmd_id, p_mcu_resp->cmd_act, resp_body.id, resp_body.cmd, resp_body.result);
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&resp_body, (SMLK_UINT8 *)&resp_body + sizeof(Msg0F41_Body));
    }
    RctrlHead head_temp;
    memcpy(&head_temp, &result.m_head, sizeof(RctrlHead));
    head_temp.qos = QOS_SEND_TCP_TIMES;
    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    return SMLK_RC::RC_OK;
}
/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/22
 * \brief       门锁设置
 * \param[in]   subcmd          功能指令
 * \param[in]   mcu_cmd       对应的远控内部指令
 * \param[in]   param_err_flag  参数错误时,为true;正确为false
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
void DissectorMsg8F41::Msg8F41Door(IN SMLK_UINT8 subcmd, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    mcu_cmd.cmd_id = SmlkMcuMsgID::SMLK_MCU_CMD_DOOR;
    mcu_cmd.cmd_data_len = 0;
    param_err_flag = false;
    if (JTT808_SUBCMD_DOOR_UNLOCK == subcmd)
    {
        mcu_cmd.cmd_act = SmlkMcuMsgDoorLockStatus::SMLK_MCU_ACTION_DOOR_UNLOCK;
    }
    else if (JTT808_SUBCMD_DOOR_LOCK == subcmd)
    {
        mcu_cmd.cmd_act = SmlkMcuMsgDoorLockStatus::SMLK_MCU_ACTION_DOOR_LOCK;
    }
    else
    {
        param_err_flag = true;
    }
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/22
 * \brief       发动机设置
 * \param[in]   sub_id          功能指令
 * \param[in]   mcu_cmd         对应的远控内部指令
 * \param[in]   param_err_flag  参数错误时,为true;正确为false
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
void DissectorMsg8F41::Msg8F41Engine(IN SMLK_UINT8 sub_id, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    mcu_cmd.cmd_id = SMLK_MCU_CMD_ENGINE;
    mcu_cmd.cmd_data_len = 0;
    param_err_flag = false;
    if (JTT808_SUBCMD_ENGINE_START == sub_id)
    {
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
    }
    else if (JTT808_SUBCMD_ENGINE_STOP == sub_id)
    {
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
    }
    else
    {
        param_err_flag = true;
    }
}

void DissectorMsg8F41::Msg8F41IdlingWarmUp(IN SMLK_UINT8 sub_id, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    mcu_cmd.cmd_id = SMLK_MCU_CMD_IDLING_WARM_UP;
    mcu_cmd.cmd_data_len = 0;
    param_err_flag = false;

    if (JTT808_SUBCMD_IDLING_WARM_UP_LEVEL_1 == sub_id)
    {
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_AC_IDLING_WARM_UP_LEVEL_1;
    }
    else if (JTT808_SUBCMD_IDLING_WARM_UP_LEVEL_2 == sub_id)
    {
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_AC_IDLING_WARM_UP_LEVEL_2;
    }
    else if (JTT808_SUBCMD_IDLING_WARM_UP_CLOSE == sub_id)
    {
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_IDLING_WARM_UP_CLOSE;
    }
    else
    {
        param_err_flag = true;
    }
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/22
 * \brief       空调控制
 * \param[in]   sub_id          功能指令
 * \param[in]   param           参数值
 * \param[in]   mcu_cmd         对应的远控内部指令
 * \param[in]   param_err_flag  参数错误时,为true;正确为false
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
void DissectorMsg8F41::Msg8F41AC(IN SMLK_UINT8 sub_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    switch (sub_id)
    {
    case JTT808_SUBCMD_AC_OPEN:           /*请求打开原车空调*/
    case JTT808_SUBCMD_AC_CLOSE:          /*请求关闭原车空调*/
    case JTT808_SUBCMD_AC_AIROUTLET_MODE: /*原车空调出风模式*/
    case JTT808_SUBCMD_AC_TEMPER:         /*原车空调温度控制*/
    case JTT808_SUBCMD_AC_WIND:           /*原车空调出风量设置*/
    case JTT808_SUBCMD_AC_LOOP_MOOD:      /*原车空调循环模式*/
    case JTT808_SUBCMD_AC_COMPRESSOR:     /*原车空调压缩机*/
    case JTT808_SUBCMD_AC_AUTO_SWITCH:    /*原车空调AUTO开关*/
    case JTT808_SUBCMD_AC_VENTILATION:    /*原车空调一键通风开关*/
    case JTT808_SUBCMD_AC_DEFROST:        /*原车空调强制除霜开关*/
        Msg8f41OriginalAC(sub_id, param, mcu_cmd, param_err_flag);
        break;
    case JTT808_SUBCMD_INDEPT_WARM_AIR_SWITCH:
    case JTT808_SUBCMD_INDEPT_WARM_AIR_TEMPER:
        Msg8f41IndptWarmAC(sub_id, param, mcu_cmd, param_err_flag);
        break;
    case JTT808_SUBCMD_PARKING_AC_SWITCH:
    case JTT808_SUBCMD_PARKING_AC_AUTO:
    case JTT808_SUBCMD_PARKING_AC_TEMPER:
    case JTT808_SUBCMD_PARKING_AC_WIND:
        Msg8f41ParkingAC(sub_id, param, mcu_cmd, param_err_flag);
        break;
    default:
        param_err_flag = true;
        break;
    }
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/22
 * \brief       原车空调设置
 * \param[in]   sub_id          功能指令
 * \param[in]   param           参数值
 * \param[in]   mcu_cmd         对应的远控内部指令
 * \param[in]   param_err_flag  参数错误时,为true;正确为false
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
void DissectorMsg8F41::Msg8f41OriginalAC(IN SMLK_UINT8 sub_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    SMLK_UINT8 param_temp = param;
    param_err_flag = false;
    switch (sub_id)
    {
    case JTT808_SUBCMD_AC_OPEN: /*请求打开原车空调*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_SWITCH;
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        break;
    }
    case JTT808_SUBCMD_AC_CLOSE: /*请求关闭原车空调*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_SWITCH;
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        break;
    }
    case JTT808_SUBCMD_AC_AIROUTLET_MODE: /*原车空调出风模式*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_AIROUTLET_MODE;
        if (JTT808_PARAM_BLOW_FACE == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_BLOW_FACE;
        }
        else if (JTT808_PARAM_BLOW_FACE_FEET == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_BLOW_FACE_FEET;
        }
        else if (JTT808_PARAM_BLOW_FEET == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_BLOW_FEET;
        }
        else if (JTT808_PARAM_BLOW_FEET_DEFROST == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_BLOW_FEET_DEFROST;
        }
        else
        {
            param_err_flag = true;
        }
        break;
    }
    case JTT808_SUBCMD_AC_TEMPER: /*原车空调温度控制*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_TEMPER;
        if ((param > JTT808_PARAM_AIRCONDITON_TEMPER_MAX) || (param < JTT808_PARAM_AIRCONDITON_TEMPER_MIN))
        {
            param_err_flag = true;
        }
        else
        {
            SMLK_UINT16 temper_temp = (param * 0.5 + 17) * 10;
            mcu_cmd.cmd_data_len = MCU_DATA_LEN_2;
            mcu_cmd.cmd_data_cont.clear();
            SMLK_UINT8 c0, c1;
            c0 = temper_temp >> 8;
            c1 = temper_temp;
            mcu_cmd.cmd_data_cont.insert(mcu_cmd.cmd_data_cont.end(), &c0, &c0 + MCU_DATA_LEN_1);
            mcu_cmd.cmd_data_cont.insert(mcu_cmd.cmd_data_cont.end(), &c1, &c1 + MCU_DATA_LEN_1);
        }
        break;
    }
    case JTT808_SUBCMD_AC_WIND: /*原车空调出风量设置*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_WIND;
        if ((param > JTT808_PARAM_AIRCONDITON_WIND_MAX) || (param < JTT808_PARAM_AIRCONDITON_WIND_MIN))
        {
            param_err_flag = true;
        }
        else
        {
            mcu_cmd.cmd_data_len = MCU_DATA_LEN_1;
            mcu_cmd.cmd_data_cont.clear();
            mcu_cmd.cmd_data_cont.insert(mcu_cmd.cmd_data_cont.end(), &param, &param + MCU_DATA_LEN_1);
        }
        break;
    }
    case JTT808_SUBCMD_AC_LOOP_MOOD: /*原车空调循环模式*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_LOOP_MOOD;
        if (JTT808_PARAM_AC_OUTER_LOOP == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_AC_OUTER_LOOP;
        }
        else if (JTT808_PARAM_AC_INNER_LOOP == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_AC_INNER_LOOP;
        }
        else
        {
            param_err_flag = true;
        }
        break;
    }
    case JTT808_SUBCMD_AC_COMPRESSOR: /*原车空调压缩机*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_COMPRESSOR;
        if (JTT808_PARAM_AC_COMPRESSOR_OPEN == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (JTT808_PARAM_AC_COMPRESSOR_CLOSE == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
        {
            param_err_flag = true;
        }
        break;
    }
    case JTT808_SUBCMD_AC_AUTO_SWITCH: /*原车空调AUTO开关*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_AUTO_SWITCH;
        if (JTT808_PARAM_AC_AUTO_ON == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (JTT808_PARAM_AC_AUTO_OFF == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
        {
            param_err_flag = true;
        }
        break;
    }
    case JTT808_SUBCMD_AC_VENTILATION: /*原车空调一键通风开关*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_VENTILATION;
        if (JTT808_PARAM_AC_VENTILATION_ON == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (JTT808_PARAM_AC_VENTILATION_OFF == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
        {
            param_err_flag = true;
        }
        break;
    }
    case JTT808_SUBCMD_AC_DEFROST: /*原车空调强制除霜开关*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AC_DEFROST;
        if (JTT808_PARAM_AC_DEFROST_ON == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (JTT808_PARAM_AC_DEFROST_OFF == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
        {
            param_err_flag = true;
        }
        break;
    }
    default:
        param_err_flag = true;
        break;
    }
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/22
 * \brief       独立暖风空调设置
 * \param[in]   sub_id          功能指令
 * \param[in]   param           参数值
 * \param[in]   mcu_cmd         对应的远控内部指令
 * \param[in]   param_err_flag  参数错误时,为true;正确为false
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
void DissectorMsg8F41::Msg8f41IndptWarmAC(IN SMLK_UINT8 sub_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    param_err_flag = false;
    switch (sub_id)
    {
    case JTT808_SUBCMD_INDEPT_WARM_AIR_SWITCH:
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_INDEPT_WARM_AIR;
        if (JTT808_PARAM_WARM_AIR_ON == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (JTT808_PARAM_WARM_AIR_OFF == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
        {
            param_err_flag = true;
        }
        break;
    }
    case JTT808_SUBCMD_INDEPT_WARM_AIR_TEMPER:
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_INDEPT_WARM_AIR_TEMPER;
        if ((param > JTT808_PARAM_WARM_AIR_MAX) || (param < JTT808_PARAM_WARM_AIR_MIN))
        {
            param_err_flag = true;
        }
        else
        {
            mcu_cmd.cmd_data_len = MCU_DATA_LEN_1;
            mcu_cmd.cmd_data_cont.clear();
            mcu_cmd.cmd_data_cont.insert(mcu_cmd.cmd_data_cont.end(), &param, &param + MCU_DATA_LEN_1);
        }
        break;
    }
    default:
        param_err_flag = true;
        break;
    }
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/22
 * \brief       驻车空调设置
 * \param[in]   sub_id          功能指令
 * \param[in]   param           参数值
 * \param[in]   mcu_cmd         对应的远控内部指令
 * \param[in]   param_err_flag  参数错误时,为true;正确为false
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
void DissectorMsg8F41::Msg8f41ParkingAC(IN SMLK_UINT8 sub_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    param_err_flag = false;
    switch (sub_id)
    {
    case JTT808_SUBCMD_PARKING_AC_SWITCH: /*驻车空调开关*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_PARKING_AC_SWITCH;
        if (JTT808_PARAM_PARKING_AIR_ON == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (JTT808_PARAM_PARKING_AIR_OFF == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
        {
            param_err_flag = true;
        }
        break;
    }
    case JTT808_SUBCMD_PARKING_AC_AUTO: /*驻车空调AUTO开关*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_PARKING_AC_AUTO;
        if (JTT808_PARAM_PARKING_AIR_AUTO_ON == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (JTT808_PARAM_PARKING_AIR_AUTO_OFF == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
        {
            param_err_flag = true;
        }
        break;
    }
    case JTT808_SUBCMD_PARKING_AC_TEMPER:
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_PARKING_AC_TEMPER;
        if (param > JTT808_PARAM_PARKING_AIRCONDITON_TEMPER_MAX)
        {
            param_err_flag = true;
        }
        else
        {
            SMLK_UINT16 temper_temp = (param * 0.5 + 17) * 10;
            mcu_cmd.cmd_data_len = MCU_DATA_LEN_2;
            mcu_cmd.cmd_data_cont.clear();
            SMLK_UINT8 c0, c1;
            c0 = temper_temp >> 8;
            c1 = temper_temp;
            mcu_cmd.cmd_data_cont.insert(mcu_cmd.cmd_data_cont.end(), &c0, &c0 + MCU_DATA_LEN_1);
            mcu_cmd.cmd_data_cont.insert(mcu_cmd.cmd_data_cont.end(), &c1, &c1 + MCU_DATA_LEN_1);
        }
        break;
    }
    case JTT808_SUBCMD_PARKING_AC_WIND:
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_PARKING_AC_WIND;
        if (param > JTT808_PARAM_PARKING_AC_WIND_MAX)
        {
            param_err_flag = true;
        }
        else
        {
            mcu_cmd.cmd_data_len = MCU_DATA_LEN_1;
            mcu_cmd.cmd_data_cont.clear();
            mcu_cmd.cmd_data_cont.insert(mcu_cmd.cmd_data_cont.end(), &param, &param + MCU_DATA_LEN_1);
        }
        break;
    }
    default:
        param_err_flag = true;
        break;
    }
}

void DissectorMsg8F41::Msg8F41Mirror(IN SMLK_UINT8 sub_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    if (JTT808_SUBCMD_DRIVER_PASSAGER_MIRROR != sub_id)
    {
        param_err_flag = true;
        return;
    }
    mcu_cmd.cmd_id = SMLK_MCU_CMD_REARVIEW_MIRROR;
    mcu_cmd.cmd_data_len = 0;

    param_err_flag = false;
    if (JTT808_PARAM_REARVIEW_MIRROR_HEAT == param)
    {
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_REARVIEW_MIRROR_HEAT;
    }
    else if (JTT808_PARAM_REARVIEW_MIRROR_HEAT_CLOSE == param)
    {
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_REARVIEW_MIRROR_HEAT_CLOSE;
    }
    else
    {
        mcu_cmd.cmd_act = DATA_INVALID_UINT8;
        param_err_flag = true;
    }
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/22
 * \brief       金融锁车功能
 * \param[in]   sub_id    功能编号
 * \param[in]   mcu_cmd    功能指令
 * \param[in]   param_err_flag    参数错误时,为true;正确为false
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_UINT8 DissectorMsg8F41::Msg8F41FinancialLckCar(IN SMLK_UINT8 sub_id, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    SMLK_LOGD("sub_id=%d, .\n", sub_id);
    RtnCode property_rc = RtnCode::E_SUCCESS;

    mcu_cmd.cmd_id = SMLK_MCU_CMD_FINANCIAL_LOCK_FUNC_ENABLE;
    mcu_cmd.cmd_data_len = 0;
    param_err_flag = false;
    if (!param_err_flag)
    {

        SMLK_LOGE("[T] sys_param_name(%s), %s. 1: Enable, 2: Disable \n", SYS_PRO_NAME_FINACIAL_LCK_FUNCTION_ENEABLE, to_string(sub_id).c_str());
        property_rc = SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_FINACIAL_LCK_FUNCTION_ENEABLE, to_string(sub_id));
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("[T] sys_param_name(%s), SysProperty SetValue failed, errcode=%ld.\n", SYS_PRO_NAME_FINACIAL_LCK_FUNCTION_ENEABLE, (SMLK_INT32)property_rc);
            return SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_ERROR;
        }

        // 1130: 将上一次远控命令清零
        SMLK_LOGE("[T] sys_param_name(%s) %02x. ", SYS_PRO_NAME_LCKCAR_ACTION_JTT808, to_string(JTT808_QUERY_NULL).c_str());
        property_rc = SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_ACTION_JTT808, to_string(JTT808_QUERY_NULL));
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("[T] sys_param_name(%s), SysProperty SetValue failed.\n", SYS_PRO_NAME_LCKCAR_ACTION_JTT808);
            return SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_ERROR;
        }

        return SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_OK;
    }
    SMLK_LOGD("[T] sub_id=%d<--->rctrl(cmd=%d, action=%d, len=%d)\n", sub_id, mcu_cmd.cmd_id, mcu_cmd.cmd_act, mcu_cmd.cmd_data_len);
    return SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_ERROR;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/06/09
 * \brief       油箱防盗开关打开关闭
 * \param[in]   sub_id
 * \param[in]   mcu_cmd
 * \param[in]   param_err_flag
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
void DissectorMsg8F41::Msg8F41OilTankAntiSheftSwitch(IN SMLK_UINT8 sub_id, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    mcu_cmd.cmd_id = SMLK_MCU_CMD_MAILBOX_SECURITY;
    mcu_cmd.cmd_data_len = 0;
    param_err_flag = false;

    if (JTT808_SUBCMD_MAILBOX_SECURITY_OPEN == sub_id)
    {
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
    }
    else if (JTT808_SUBCMD_MAILBOX_SECURITY_CLOSE == sub_id)
    {
        mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
    }
    else
    {
        param_err_flag = true;
    }
}

/**
 * @brief                   解析8F41智能冷机处理消息
 * @param sub_id            8F41功能指令
 * @param param             8F41功能参数
 * @param mcu_cmd           下发给MCU的数据
 * @param param_err_flag    8F41功能参数错误标志位
 *
 * @return
 */
void DissectorMsg8F41::Msg8F41Ruc(IN SMLK_UINT8 sub_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    param_err_flag = false;
    switch (sub_id)
    {
    case JTT808_SUBCMD_RUC_SWITCH: /*智能冷机开关控制*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_RUC_SWITCH;
        if (SmlkTspMsgCommonSwitch::SMLK_TSP_ACTION_ON == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (SmlkTspMsgCommonSwitch::SMLK_TSP_ACTION_OFF == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
            param_err_flag = true;
        break;
    }
    case JTT808_SUBCMD_RUC_TEMPER: /*智能冷机温度设置*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_RUC_TEMPER;
        mcu_cmd.cmd_data_len = 1;
        mcu_cmd.cmd_data_cont.emplace_back(param);
        break;
    }
    case JTT808_SUBCMD_RUC_REMOTE_DEFROST: /*智能冷机远程除霜控制*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_RUC_DEFROST;
        if (SmlkTspMsgRucDefrost::SMLK_TSP_RUC_DEFROST == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (SmlkTspMsgRucDefrost::SMLK_TSP_RUC_NO_DEFROST == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
            param_err_flag = true;
        break;
    }
    default:
        param_err_flag = true;
        break;
    }
}
/**
 * @brief                   解析8F41一键温暖&一键清凉设置
 * @param sub_id            8F41功能指令
 * @param param             8F41功能参数
 * @param mcu_cmd           下发给MCU的数据
 * @param param_err_flag    8F41功能参数错误标志位
 *
 * @return
 */
void DissectorMsg8F41::Msg8F41AutoTemper(IN SMLK_UINT8 sub_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag)
{
    param_err_flag = false;
    switch (sub_id)
    {
    case JTT808_SUBCMD_AUTO_TEMPER_HEAT: /*一键温暖*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AUTO_TEMPER_HEAT;
        if (SmlkTspMsgCommonSwitch::SMLK_TSP_ACTION_ON == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (SmlkTspMsgCommonSwitch::SMLK_TSP_ACTION_OFF == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
            param_err_flag = true;
        break;
    }
    case JTT808_SUBCMD_AUTO_TEMPER_COOL: /*一键清凉*/
    {
        mcu_cmd.cmd_id = SMLK_MCU_CMD_AUTO_TEMPER_COOL;
        if (SmlkTspMsgCommonSwitch::SMLK_TSP_ACTION_ON == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_ON;
        }
        else if (SmlkTspMsgCommonSwitch::SMLK_TSP_ACTION_OFF == param)
        {
            mcu_cmd.cmd_act = SMLK_MCU_ACTION_OFF;
        }
        else
            param_err_flag = true;
        break;
    }
    default:
        param_err_flag = true;
        break;
    }
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/04/26
 * \brief       发送消贷功能使能禁止设置后的返回结果
 * \param[in]   head
 * \param[in]   loan_enable
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F41::SendMsg0F41(IN RctrlHead &head, IN SMLK_UINT8 *indata, IN SMLK_UINT16 &, IN SMLK_UINT8 loan_enable)
{
    SMLK_LOGD("enter in func(%s).", __func__);
    vector<SMLK_UINT8> output_vec;
    const Msg8F41_Head *phead8f41 = (Msg8F41_Head *)indata;
    const Msg8F41_Body *ppayload8f41 = (Msg8F41_Body *)((SMLK_UINT8 *)indata + sizeof(Msg8F41_Head));
    Msg0F41_Head head0f41;

    head0f41.version = phead8f41->version;
    head0f41.seq_id = htobe16(head.seq_id);
    head0f41.cmd_num = phead8f41->cmd_num;
    tools::get_bcd_timestamp(head0f41.time);
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&head0f41, (SMLK_UINT8 *)&head0f41 + sizeof(Msg0F41_Head));

    Msg0F41_Body payload0f41;
    payload0f41.id = ppayload8f41->id;
    payload0f41.cmd = ppayload8f41->cmd;
    payload0f41.result = loan_enable;
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&payload0f41, (SMLK_UINT8 *)&payload0f41 + sizeof(Msg0F41_Body));

    SMLK_LOGD("ppayload8f41->numbering=0x%02x,  ppayload8f41->sub_id=0x%02x.\n", ppayload8f41->id, ppayload8f41->cmd);
    RctrlHead temp_head;
    memcpy(&temp_head, &head, sizeof(RctrlHead));
    temp_head.msg_id = JTT808_REMOTE_CTRL_RESP;
    temp_head.qos = QOS_SEND_TCP_TIMES;

    SMLK_RC rc = SMLK_RC::RC_OK;
    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(temp_head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    return SMLK_RC::RC_OK;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/03/18
 * \brief       8f41的参数不正确,回复错误码为06的0f41
 * \param[out]  indata   8F41消息体
 * \param[in]   length    消息体长度
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorMsg8F41::SendMsg0F41(IN RctrlHead &head, IN SMLK_UINT8 *indata, IN SMLK_UINT16 &)
{
    vector<SMLK_UINT8> output_vec;
    const Msg8F41_Head *phead8f41 = (Msg8F41_Head *)indata;
    const Msg8F41_Body *ppayload8f41 = (Msg8F41_Body *)(indata + sizeof(Msg8F41_Head));
    Msg0F41_Head head0f41;

    head0f41.version = phead8f41->version;
    head0f41.seq_id = htobe16(head.seq_id);
    head0f41.cmd_num = phead8f41->cmd_num;
    tools::get_bcd_timestamp(head0f41.time);
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&head0f41, (SMLK_UINT8 *)&head0f41 + sizeof(Msg0F41_Head));

    for (SMLK_UINT8 i = 0; i < phead8f41->cmd_num; ++i)
    {
        Msg0F41_Body payload0f41;
        payload0f41.id = ppayload8f41->id;
        payload0f41.cmd = ppayload8f41->cmd;
        payload0f41.result = SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_CTRL_NOT_SP;
        ++ppayload8f41;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&payload0f41, (SMLK_UINT8 *)&payload0f41 + sizeof(Msg0F41_Body));
    }

    RctrlHead temp_head;
    memcpy(&temp_head, &head, sizeof(RctrlHead));
    temp_head.msg_id = JTT808_REMOTE_CTRL_RESP;
    temp_head.qos = QOS_SEND_TCP_TIMES;

    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(temp_head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    return SMLK_RC::RC_OK;
}

DissectorMsg8F41::DissectorMsg8F41()
{
}

DissectorMsg8F41::~DissectorMsg8F41()
{
}

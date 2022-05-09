/*****************************************************************************/
/**
 * \file       rctrl_jtt808.cpp
 * \author     huangxin
 * \date       2020/12/16
 * \version    Tbox2.0 V1
 * \brief      jtt808协议解析入口
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/

#include "vehctrl_inf.h"
#include "vehctrl_jtt808.h"
#include "vehctrl_queue.h"
#include "tsp_service_api.h"
#include "vehctrl_jtt808_msg8f41.h"
#include "vehctrl_jtt808_msg8f51.h"
#include "vehctrl_jtt808_msg8f42.h"
#include "vehicle_data_index_def.h"
#include "vehctrl_jtt808_getset_param.h"
using namespace std;
using namespace smartlink;

/*消息ID--解析器表*/
const std::map<SMLK_UINT16, std::shared_ptr<IDissector>> DissectorJtt808::m_map_msgdis =
    {
        {JTT808_MSG_REMOTE_CTRL, make_shared<DissectorMsg8F41>()},
        {JTT808_REMOTE_CTRL_RESP, make_shared<DissectorMsg8F41>()},
        {JTT808_MSG_VEHICAL_QUERY, make_shared<DissectorMsg8F51>()},
        {JTT808_VEHICAL_QUERY_RESP, make_shared<DissectorMsg8F51>()},
        {JTT808_TERMINAL_PARAM_SET, make_shared<DissectorMsgTerminalParam>()},
        {JTT808_TERMINAL_PARAM_GET, make_shared<DissectorMsgTerminalParam>()},
        {JTT808_TERMINAL_SPECIFIED_PARAM_GET, make_shared<DissectorMsgTerminalParam>()},
        {JTT808_GET_SET_TBOX, make_shared<DissectorMsg8F42>()},
};

const std::map<SMLK_UINT16, std::shared_ptr<IDissector>> DissectorJtt808::m_map_msgchange =
    {
        /*VehicleData信号值*/
        /*车门锁状态*/
        {VEHICLE_DATA_LOCK_STATUS_OF_DOOR_DRIVER, make_shared<DissectorMsg8F51>()},    /*驾驶侧车门开解锁状态*/
        {VEHICLE_DATA_LOCK_STATUS_OF_DOOR_PASSENGER, make_shared<DissectorMsg8F51>()}, /*乘客侧车门开解锁状态*/
        /*发动机状态*/
        {VEHICLE_DATA_ENGINE_SPEED, make_shared<DissectorMsg8F51>()}, /*发动机转速*/
        /*原车空调状态*/
        {VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS, make_shared<DissectorMsg8F51>()},           /*原车空调开关状态*/
        {VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS, make_shared<DissectorMsg8F51>()},             /*原车空调出风模式*/
        {VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE, make_shared<DissectorMsg8F51>()},            /*原车空调温度*/
        {VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE, make_shared<DissectorMsg8F51>()},            /*原车空调风量*/
        {VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS, make_shared<DissectorMsg8F51>()}, /*原车空调循环模式*/
        {VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS, make_shared<DissectorMsg8F51>()},          /*原车空调压缩机状态*/
        {VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS, make_shared<DissectorMsg8F51>()},        /*原车空调auto状态*/
        {VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS, make_shared<DissectorMsg8F51>()},      /*原车空调一键通风状态*/
        /*独立暖风状态*/
        {VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS, make_shared<DissectorMsg8F51>()}, /*独立暖风开关*/
        {VEHICLE_DATA_WARM_AIR_TEMPERATURE, make_shared<DissectorMsg8F51>()},          /*独立暖风温度*/
        /*驻车空调状态*/
        {VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS, make_shared<DissectorMsg8F51>()}, /*驻车空调开关*/
        {VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS, make_shared<DissectorMsg8F51>()},           /*驻车空调auto开关*/
        {VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE, make_shared<DissectorMsg8F51>()},  /*驻车空调温度*/
        {VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE, make_shared<DissectorMsg8F51>()},  /*驻车空调风量*/
        /*油箱防盗开关*/
        {VEHICLE_DATA_DRIVER_TANK_ANTI_THEFT_STATUS, make_shared<DissectorMsg8F51>()}, /*油箱防盗开关*/
        /*智能冷机状态*/
        {VEHICLE_DATA_TEMPERATURE_REMOTE_OPERATION_ALLOWED_STATE, make_shared<DissectorMsg8F51>()}, /*冷机远程可控制状态*/
        {VEHICLE_DATA_REFRIGERATION_SWITCH_STATUS, make_shared<DissectorMsg8F51>()},                /*制冷机组开关状态*/
        {VEHICLE_DATA_DEFROST_ALLOWED_STATE, make_shared<DissectorMsg8F51>()},                      /*除霜允许状态*/
        {VEHICLE_DATA_REFRIGERATION_WORK_MODE, make_shared<DissectorMsg8F51>()},                    /*除霜工作模式*/
        {VEHICLE_DATA_RUC_REGULATION_TEMPERATURE, make_shared<DissectorMsg8F51>()},                 /*冷机当前设置的温度*/
        /*内部信号值*/
        /*车辆电源状态*/
        {SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_POWER_MODE, make_shared<DissectorMsg8F51>()},
        /*终端唤醒状态报告*/
        {SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_WAKEUP_SOURCE, make_shared<DissectorMsg8F51>()},
        /*ps控制状态*/
        {SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_PS_CTRL_MODE, make_shared<DissectorMsg8F51>()},
}; /*支持0F51主动上报的变化量*/

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/01/07
 * \brief       将远控的内部头信息提取至ipc head
 * \param[in]   rctrl_head
 * \param[in]   ipc_head
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
void DissectorJtt808Common::GetIpcHeadFromRctrlHead(IN RctrlHead &rctrl_head, OUT IpcTspHead &ipc_head)
{
    ipc_head.msg_id = rctrl_head.msg_id;
    ipc_head.seq_id = rctrl_head.seq_id;
    ipc_head.protocol = (SMLK_UINT8)(rctrl_head.protocol);
    ipc_head.qos = rctrl_head.qos;
    ipc_head.priority = rctrl_head.priority;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/11/30
 * \brief       status changed , report status to tsp
 * \param[in]   cmd_id
 * \param[in]   status
 * \return
 ******************************************************************************/
SMLK_RC DissectorJtt808::OnChanged(IN SMLK_UINT8 &datatype, IN SMLK_UINT16 &cmd_id, IN SMLK_DOUBLE &status)
{
    /*目前仅8F51支持主动上报*/
    SMLK_RC rc;
    if (m_map_msgchange.empty())
    {
        SMLK_LOGW("m_map_msgchange is empty!");
        return SMLK_RC::RC_ERROR;
    }
    auto iter = m_map_msgchange.find(cmd_id);
    if (iter == m_map_msgchange.end())
    {
        SMLK_LOGW("can not find cmd_id = %d in m_map_msgchange", cmd_id);
        return SMLK_RC::RC_ERROR;
    }
    auto func_ptr = iter->second;
    rc = func_ptr->OnChanged(datatype, cmd_id, status);
    return rc;
}

void DissectorJtt808::RegisterCB(IN VecCtrlInnerCallBack &cb)
{
    if (cb)
    {
        auto iter = m_map_msgdis.find(JTT808_GET_SET_TBOX);
        if (iter != m_map_msgdis.end())
        {
            iter->second->RegisterCB(cb);
        }
    }
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/12/10
 * \brief       jtt808协议处理器负责解析tsp消息,检查参数的合法性,生成远控内部指令集
 * \param[out]  indata    tsp过来的消息
 * \param[in]   length    消息的长度
 * \param[in]   queue    远控命令处理消息队列
 * \param[in]   is_encode    是否生成了远控内部命令,如果是,那么要进encode逻辑中处理
 * \return      0：成功表示解析tsp消息的参数合法；-1：参数解析失败
 * \remarks
 ******************************************************************************/
SMLK_RC DissectorJtt808::Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode)
{
    SMLK_LOGD("********Decode Start********");
    if ((nullptr == indata) && (queue.m_head.msg_id != JTT808_TERMINAL_PARAM_GET))
    {
        SMLK_LOGE("Input data invalid!");
        return SMLK_RC::RC_ERROR;
    }
    if (m_map_msgdis.empty())
    {
        SMLK_LOGW("Msgdissector is empty.");
        return SMLK_RC::RC_ERROR;
    }
    /*根据消息ID进入不同的消息解析器*/
    auto iter = m_map_msgdis.find(queue.m_head.msg_id);
    if (iter == m_map_msgdis.end())
    {
        SMLK_LOGW("Can't find msg_id=0x%04x!", queue.m_head.msg_id);
        return SMLK_RC::RC_ERROR;
    }
    auto intf_func = iter->second;
    SMLK_RC rc = intf_func->Decode(indata, length, queue, is_encode);
    SMLK_LOGD("********Decode Over********");
    return rc;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2020/11/05
 * \brief       函数描述         通过消息id将808消息的响应分配到各个模块进行处理
 * \param[in]   result_info:     include: remte_ctrl_head + cmd_result_list
 * \param[in]   outdata_vec:     data send to tsp
 * \param[in]   msg_head:        ipc head sending to tsp service
 * \return
 * \remarks
 ******************************************************************************/
SMLK_RC DissectorJtt808::ResultEncode(IN RctrlResultQueue &result)
{
    SMLK_LOGD("********Result Encode Start********");
    SMLK_RC rc = SMLK_RC::RC_ERROR;
    if (m_map_msgdis.empty())
    {
        SMLK_LOGW("Dissertor is empty.");
        return SMLK_RC::RC_ERROR;
    }
    auto iter = m_map_msgdis.find(result.m_head.msg_id);
    if (iter == m_map_msgdis.end())
    {
        SMLK_LOGW("Can't find msg_id=0x%4x!", result.m_head.msg_id);
        return SMLK_RC::RC_ERROR;
    }
    auto intf_func = iter->second;
    rc = intf_func->ResultEncode(result);
    SMLK_LOGD("********Result Encode Over********");
    return rc;
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/01/07
 * \brief       发送消息体给tsp,并在ipc头中告诉tsp这是哪个消息ID
 * \param[in]   rctrl_head    msg_id为真正要发送的msgID,其他信息依然是tsp穿过的值
 * \param[out]  indata     消息体
 * \param[in]   length     消息体长度
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorJtt808Common::DoSendMsgToTsp(IN RctrlHead &rctrl_head, IN SMLK_UINT8 *msg_body, IN std::size_t &length)
{
    IpcTspHead ipc_head;
    GetIpcHeadFromRctrlHead(rctrl_head, ipc_head);
    std::string log = tools::uint8_2_string((SMLK_UINT8 *)msg_body, length);
    SMLK_LOGD("[Tbox2Tsp][MsgID:0x%04x][SN:0x%04x]=%s", rctrl_head.msg_id, rctrl_head.seq_id, log.c_str());
    return TspServiceApi::getInstance()->SendMsg(ipc_head, msg_body, (std::size_t)length);
}

/*****************************************************************************/
/**
 * \author      huangxin
 * \date        2021/01/07
 * \brief       发送jtt808通用应答
 * \param[in]   head     所有字段为tsp传过来的
 * \param[in]   result    通用应答中的应答码
 * \return      返回值描述
 * \remarks     其它信息
 ******************************************************************************/
SMLK_RC DissectorJtt808Common::SendMsgCommonResp(IN RctrlHead &head, IN SMLK_UINT8 &result)
{
    RctrlHead rctrl_head;
    Jtt808CommResp response;
    response.seq_id = htobe16(head.seq_id);
    response.msg_id = htobe16(head.msg_id);
    response.result = result;
    memcpy(&rctrl_head, &head, sizeof(RctrlHead));
    rctrl_head.msg_id = JTT808_COMMON_RESPONSE;
    DoSendMsgToTsp(rctrl_head, (SMLK_UINT8 *)&response, sizeof(response));
    return SMLK_RC::RC_OK;
}

DissectorJtt808::DissectorJtt808()
{
}

DissectorJtt808::~DissectorJtt808()
{
}

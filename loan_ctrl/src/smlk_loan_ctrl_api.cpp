/*****************************************************************************/
/**
* \file       smlk_loan_ctrl_api.cpp
* \author     wukai
* \date       2021/08/23
* \version    Tbox2.0 V1
* \brief      提供esync 调用接口
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan ctrl
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#include "smlk_loan_ctrl_api.h"
#include "smlk_loan_ecu_impl.h"
#include "smartlink_sdk_sys_property.h"
#include "smartlink_sdk_sys_property_def.h"
#include "smlk_log.h"
#include <stdlib.h>
#include "smartlink_sdk_location.h"

#include "smlk_loan_EMS_Bosch_NG.h"
#include "smlk_loan_EMS_Econtrol.h"
#include "smlk_loan_EMS_G6.h"
#include "smlk_loan_EMS_WEICHAI.h"
#include "smlk_loan_EMS_YUCHAI.h"
#include "smlk_loan_HCU_FAW.h"
#include "smlk_loan_HCU_Bosch_FAW.h"
#include "smlk_loan_VCU_ECONTROL.h"
#include "smlk_loan_VCU_G6.h"
#include "smlk_loan_VCU_G6_CGN.h"
#include "smlk_loan_EMS_CUMMINS.h"
#include "smlk_loan_EMS_YUNNEI.h"
#include "smlk_loan_QQVCU_BASE.h"


using namespace smartlink::CAN;
using namespace smartlink_sdk;
// using namespace smartlink::LOAN;
using namespace smartlink;
using namespace std;

namespace smartlink
{
namespace LOAN
{
inline bool CheckPtr(std::shared_ptr<SmlkEmsLoanCtrl>& _sp_ptr)
{
    if (!_sp_ptr)
        return false;
    if (!_sp_ptr.get())
        return false;
    return true;
}

std::string hex_to_string(SMLK_UINT8 *pvalue, SMLK_UINT8 len)
{
    char value_str1[16] ={0};
    sprintf(&value_str1[0], "0x");
    int j=2;
    for(SMLK_UINT8 i = 0 ; i < len; ++i){
        sprintf(&value_str1[j], "%02x", pvalue[i]);
        j+=2;
    }
    string value_str = value_str1;
    return value_str;
}

static SMLK_RC SeperateStringWithComma(IN void *data, IN SMLK_UINT16 len, OUT vector<pair<SMLK_UINT16, SMLK_UINT16>> &start_end_index_vec)
{

    SMLK_UINT16 comma_index = 0;
    SMLK_UINT16 before_index = 0;
    SMLK_UINT16 end_index = 0;
    for(SMLK_UINT16 i = 0; i < len; ++i){
       if(',' == *((SMLK_UINT8 *)data+i)){
           if(0 == comma_index){
                end_index = i-1;
                start_end_index_vec.push_back(make_pair(before_index, end_index));
                SMLK_LOGD("coma_index = %ld, start_index = %d, end_index=%d.\n",start_end_index_vec.size()-1,before_index, end_index);
                before_index = i+1;
           }else{
               end_index = i-1;
               start_end_index_vec.push_back(make_pair(before_index, end_index));
               SMLK_LOGD("coma_index = %ld, start_index = %d, end_index=%d.\n",start_end_index_vec.size()-1,before_index, end_index);
           }

          ++comma_index;
           before_index = end_index+2;

       }
    }

    end_index = len-1;
    start_end_index_vec.push_back(make_pair(before_index, end_index));
    SMLK_LOGD("coma_index = %ld, start_index = %d, end_index=%d.\n",start_end_index_vec.size()-1,before_index, end_index);

    return SMLK_RC::RC_OK;

}

SMLK_BOOL SmlkLoanManager::InitServeice()
{
        /*注册tsp消息接收接口*/
    if (SMLK_RC::RC_OK != TspServiceApi::getInstance()->Init(ModuleID::E_Module_loan_service))
    {
        SMLK_LOGF("TspServiceApi Init failed.");
        return SMLK_FALSE;
    }
    vector<SMLK_UINT16> msg_id;
    msg_id.push_back(JTT808_REMOTE_LOCK);                                          /*金融锁车*/

    TspServiceApi::getInstance()->RegisterMsgCB( ProtoclID::E_PROT_JTT808, msg_id, [&](IN IpcTspHead &ipc, IN void *data, std::size_t sz) {
            SMLK_LOGD("#########IPC size :%d", (SMLK_UINT32)sz);
            SyncTspLoanMsg(ipc, data, sz);
        }
    );

    if(smartlink_sdk::RtnCode::E_SUCCESS != MCU::GetInstance()->Init(ModuleID::E_Module_loan_service))
    {
        SMLK_LOGF("MCU Init failed.");
        return SMLK_FALSE;
    }

    std::vector<McuEventId>  mcu_events = {
        McuEventId::E_MCU_EVENT_UDS_ASYNC_RESPONSE,
        McuEventId::E_MCU_EVENT_IG_STATE,
        McuEventId::E_MCU_EVENT_VEHICLE_CRTL_ACK
    };
    g_rctrl_ign_status = IGN_OFF;
    g_rctrl_ign_status_before = IGN_OFF;
    MCU::GetInstance()->RegEventCB(mcu_events, [&](McuEventId id, void *data, int len){
        if (id == McuEventId::E_MCU_EVENT_UDS_ASYNC_RESPONSE)
        {
           OnMcuEventSet(id, data, len);
        }
        if (id == McuEventId::E_MCU_EVENT_IG_STATE)
        {
            IGNState *ign = reinterpret_cast<IGNState *>(data);
            if (ign->on)
            {
                g_rctrl_ign_status_before = g_rctrl_ign_status;
                g_rctrl_ign_status = IGN_ON;
            }
            else
            {
                g_rctrl_ign_status_before = g_rctrl_ign_status;
                g_rctrl_ign_status = IGN_OFF;
            }
            if (g_rctrl_ign_status_before != g_rctrl_ign_status)
            {
                SyncIGNStatus((SMLK_BOOL)(ign->on));
            }
        }
        if (id == McuEventId::E_MCU_EVENT_VEHICLE_CRTL_ACK)
        {
            ProcessMcuSerResp(data, len);
        }
        if (id == McuEventId::E_MCU_EVENT_UDS_ASYNC_RESPONSE)
        {
            SMLK_LOGD("get mcu async response event len %d!", len);
        }
    });

    // SysProperty::GetInstance()->Init(ModuleID::E_Module_loan_service);
    const std::vector<std::string> property_key = {
        std::string(SYS_PRO_NAME_4G_ANTENNA_LOCK),
    };

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->RegEventCB(
        property_key,
        [this](smartlink_sdk::SysProperty::PropertyEventId id, smartlink_sdk::PropertyInfo *info)
        {
            switch (id)
            {
                case smartlink_sdk::SysProperty::PropertyEventId::E_SYS_PROPERTY_EVENT_VALUE_CHANGED:
                {
                    if (info->name == std::string(SYS_PRO_NAME_4G_ANTENNA_LOCK))
                    {
                        SetTsc1State();
                    }
                }
            }
        });


    /*location接口初始化*/
    if (!Location::GetInstance()->Init(ModuleID::E_Module_loan_service))
    {
        SMLK_LOGF("Location Init failed.");
        return SMLK_FALSE;
    }
    std::vector<LocationEventId> location_events = {
        LocationEventId::E_LOCATION_EVENT_GNSS_ANTENNA_CHANGED
    };

    Location::GetInstance()->RegEventCallback(location_events, [&](LocationEventId id, void *data, int len) {
        switch ( id ) {
            case LocationEventId::E_LOCATION_EVENT_GNSS_ANTENNA_CHANGED:
                {
                    if (!is_need_4GAntenna){
                        is_need_4GAntenna = true;
                    } else {
                        std::string lck_status_str;
                        std::string property;
                        RtnCode property_rc = RtnCode::E_SUCCESS;
                        property = SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS;
                        property_rc = SysProperty::GetInstance()->GetValue(property, lck_status_str);
                        if ((RtnCode::E_SUCCESS == property_rc) && (std::stoi(lck_status_str) >= ((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_IS_ACTIVE))))
                            m_gpsAntenna_flag.store(true);
                    }
                }
                break;
            default:
                SMLK_LOGE("unregistered event id: 0x%08X", static_cast<SMLK_UINT32>(id));
                break;
            }
        }
    );

    /*tel模块初始化*/
    if (!Telephony::GetInstance()->Init(ModuleID::E_Module_loan_service))
    {
        SMLK_LOGF("Telephony Init failed.");
        return SMLK_FALSE;
    }

    std::vector<TelEventId> tel_event_vec = {
        TelEventId::E_TEL_EVENT_SMS_RECV,
        TelEventId::E_TEL_EVENT_ANTENNA_CHANGED};
    Telephony::GetInstance()->RegEventCB(tel_event_vec, [&](TelEventId id, void *data, int len) {
            switch ( id ) {
                case TelEventId::E_TEL_EVENT_SMS_RECV:
                    {
                        DecodeSMS(data);
                    }
                    break;
                case TelEventId::E_TEL_EVENT_ANTENNA_CHANGED:
                    {
                        if (!is_need_gpsAntenna){
                            is_need_gpsAntenna = true;
                        } else {
                            std::string lck_status_str;
                            std::string property;
                            RtnCode property_rc = RtnCode::E_SUCCESS;
                            property = SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS;
                            property_rc = SysProperty::GetInstance()->GetValue(property, lck_status_str);
                            if ((RtnCode::E_SUCCESS == property_rc) && (std::stoi(lck_status_str) >= ((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_IS_ACTIVE))))
                                m_4gAntenna_flag.store(true);
                        }
                    }
                    break;
                default:
                    SMLK_LOGE("unregistered event id: 0x%08X", static_cast<SMLK_UINT32>(id));
                    break;
            }
        }
    );
}
SMLK_BOOL SmlkLoanManager::Init()
{
    if (!SysProperty::GetInstance()->Init(ModuleID::E_Module_loan_service))
    {
        SMLK_LOGF("SysProperty Init failed.");
        return SMLK_FALSE;
    }

    std::string vehicletype;
    SMLK_UINT8 m_vehicle_type = 0x00;
    std::string property;
    property = SYS_PRO_NAME_DID_VEHICLE_TYPE;
    auto return_code = SysProperty::GetInstance()->GetValue(property, vehicletype);
    if (RtnCode::E_SUCCESS != return_code)
    {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    }
    else
    {
        m_vehicle_type = std::stoi(vehicletype);
        SMLK_LOGI("vehicletype:        %u", m_vehicle_type);
    }

    std::string ems;
    SMLK_UINT8 m_ems_model = 0x02;
    property = SYS_PRO_NAME_EMS;
    return_code = SysProperty::GetInstance()->GetValue(property, ems);
    if (RtnCode::E_SUCCESS != return_code)
    {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    }
    else
    {
        m_ems_model = std::stoi(ems);
        SMLK_LOGI("ems:        %u", m_ems_model);
    }
    std::string vcu;
    SMLK_UINT8 m_vcu_model = 0x00;
    property = SYS_PRO_NAME_VCU;
    return_code = SysProperty::GetInstance()->GetValue(property, vcu);
    if (RtnCode::E_SUCCESS != return_code)
    {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    }
    else
    {
        m_vcu_model = std::stoi(vcu);
        SMLK_LOGI("vcu:        %u", m_vcu_model);
    }

    if(m_vcu_model == 0x01 && m_ems_model == 0x02)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_VCU_G6>();
    else if(m_vcu_model == 0x01 && m_ems_model == 0x04)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_VCU_G6_CGN>();
    else if(m_vcu_model == 0x01 && m_ems_model == 0x05)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_VCU_ECONTROL>();
    else if (m_vcu_model == 0x02)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_HCU_FAW>();
    else if (m_vcu_model == 0x03)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_HCU_BOSCH_FAW>();
    else if (m_vcu_model == 0x04)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_QQVCU_BASE>();
    else if (m_ems_model == 0x02)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_EMS_G6>();
    else if (m_ems_model == 0x04)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_EMS_Bosch_NG>();
    else if (m_ems_model == 0x05)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_EMS_Econtrol>();
    else if (m_ems_model == 0x06 || m_ems_model == 0x07)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_EMS_CUMMINS>();
    else if (m_ems_model == 0x08 || m_ems_model == 0x09)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_EMS_WEICHAI>();
    else if (m_ems_model == 0x0A || m_ems_model == 0x0B)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_EMS_YUCHAI>();
    else if (m_ems_model == 0x0D)
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_EMS_YUNNEI>();
    else
        m_sp_ems_ctrl = std::make_shared<SMLK_LOAN_EMS_G6>();

    if (!m_sp_ems_ctrl->Init())
    {
        return SMLK_FALSE;
    }
    m_sp_ems_ctrl->m_mcu_seq = 0;

    if (!InitServeice())
    {
        return SMLK_FALSE;
    }

    return SMLK_TRUE;
}

SMLK_BOOL SmlkLoanManager::Start()
{
    m_cmd_thread = std::thread(&SmlkLoanManager::ThreadLoanProcess, this);
    return SMLK_TRUE;
}

SMLK_BOOL SmlkLoanManager::DeInit()
{
    if (!CheckPtr(m_sp_ems_ctrl)) return SMLK_TRUE;
    m_sp_ems_ctrl->DeInit();
    m_sp_ems_ctrl.reset();
    m_sp_ems_ctrl = nullptr;
    return SMLK_TRUE;
}

void SmlkLoanManager::OnMcuEventSet(smartlink_sdk::McuEventId id, void *data, int len)
{
    SMLK_LOGD("OnMcuEventSet");
    SmlkUds::getInstance()->OnMcuEvent(id, data, len);
}

void SmlkLoanManager::ProcessMcuSerResp(void *data, int len)
{
    SMLK_LOGD("enter in func(%s).",__func__);

    LoanMcuCmdHead *phead = (LoanMcuCmdHead *)data;
    SMLK_LOGI("rcv seq(0x%04x), send seq(0x%04x).",be16toh(phead->seq), m_sp_ems_ctrl->m_mcu_seq);

    if(phead->seq == htobe16(0xFFFF))
    {
        SMLK_LOGE("rcv seq(%d) == 0xFFFF LOAN_RSP_FROM_MCU .\n", phead->seq);
        LoanSetCmdResult *presult = (LoanSetCmdResult *)((SMLK_UINT8 *)data + sizeof(LoanMcuCmdHead));
        SMLK_LOGI("rcv mcu ack: cmd_id=0x%04x, action=%d, result=%d.\n", presult->cmd_id, presult->action, (SMLK_UINT8)(presult->result));
        if(presult->cmd_id == htobe16(LOAN_RSP_FROM_MCU)){
            SMLK_LOGD("rcv mcu ack: result=%d.\n", (SMLK_UINT8)(presult->result));
            // m_sp_ems_ctrl->Pack0F40MsgQuery((GetVinEinFlagResult)presult->result);
            m_sp_ems_ctrl->EncodeMcuReport(presult->result);
        }
    }
    else if(phead->seq != htobe16(m_sp_ems_ctrl->m_mcu_seq))
    {
        SMLK_LOGE("rcv seq(%d) != send seq(%d).", be16toh(phead->seq), m_sp_ems_ctrl->m_mcu_seq);
        return ;
    }
    else
    {
        m_sp_ems_ctrl->SetResult(LOAN_CMD_RESULT_OK);
        for(SMLK_UINT8 i = 0; i < phead->count; ++i){
            LoanSetCmdResult *presult = (LoanSetCmdResult *)((SMLK_UINT8 *)data + sizeof(LoanMcuCmdHead));
            presult->cmd_id = be16toh(presult->cmd_id);
            SMLK_LOGD("rcv mcu ack: cmd_id=0x%04x, action=0x%02x, result=%d.",(presult->cmd_id),presult->action, (SMLK_UINT8)(presult->result));
            if(presult->result == LOAN_CMD_RESULT_ERROR){
                m_sp_ems_ctrl->SetResult(LOAN_CMD_RESULT_ERROR);
            }
        }
        m_sp_ems_ctrl->Notify_m_cv();
    }
}

SMLK_RC SmlkLoanManager::ThreadLoanProcess(void)
{
    while(1)
    {
        if (m_sp_ems_ctrl->CheckIgnonTime())
        {
            if (m_sp_ems_ctrl->Check0F3D())
            {
                m_sp_ems_ctrl->Send0F3D(SMLK_TRUE);
            }
            AntennaChange();
            bool timeout_flag = false;
            LoanCmdData data = m_loan_cmd_queue.get(SL_QUEUE_GET_TIMEOUT_MS, &timeout_flag);
            LoanCtrlResult result;
            LoanCmdMsg cmd_queue;
            SMLK_UINT8 com_result = JTT808_MSG_SUCCESS;
            if (!timeout_flag)
            {
                SMLK_LOGD("get one loam cmd from rctrl loan cmd queue.\n");
                /*copy necessary info to result*/
                memcpy(&result.m_head, &data.m_head, sizeof(LoanGeneralHead));
                result.m_cmd_from = data.m_cmd_from;
                result.m_result_type = data.m_cmd_type;
                result.m_lock_prot = data.m_lock_prot;
                result.m_head.msg_id = JTT808_REMOTE_LOCK_RESP;

                cmd_queue.m_head.msg_id = JTT808_REMOTE_LOCK;
                cmd_queue.m_head.protocol = data.m_head.protocol;
                cmd_queue.m_head.seq_id = data.m_head.seq_id;
                cmd_queue.m_head.qos = data.m_head.qos;
                cmd_queue.m_head.priority = data.m_head.priority;
                cmd_queue.m_cmd_from = data.m_cmd_from;
                cmd_queue.m_lock_prot = data.m_lock_prot;

                CheckLoanCmd((SMLK_UINT8 *)data.m_message.data(), data.m_message.size(), com_result);
                if (LoanCmdFrom::LOAN_FROM_TSP_MODULE == cmd_queue.m_cmd_from)
                {
                    SendMsgCommonResp(cmd_queue.m_head, com_result);
                }
                if (com_result == JTT808_MSG_SUCCESS)
                {
                    DecodeLoanCmd((SMLK_UINT8 *)data.m_message.data(), data.m_message.size(), cmd_queue, result);

                    Msg8f40Common *pcom = (Msg8f40Common *)data.m_message.data();
                    if ((JTT808_QUERY_LOCK_STATUS != pcom->subcmd) && (JTT808_SEND_HANDMADE_KEYS != pcom->subcmd))
                        ProcessLoanResult(result);
                    if ((JTT808_ACTIVE_LOCK == pcom->subcmd) || (JTT808_INACTIVE_LOCK == pcom->subcmd))
                    {
                        m_sp_ems_ctrl->Send0F3D();
                    }
                }
            }
        }
        usleep(1 * 500 * 1000);
    }
    SMLK_LOGI("Leave loan queue thread process!!!");
    return SMLK_RC::RC_OK;
}

SMLK_RC SmlkLoanManager::CheckLoanCmd(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT SMLK_UINT8 &result)
{
    SMLK_LOGD("*******enter in func(%s)\n",__func__);
    SMLK_UINT16 offset = 0;
    Msg8f40Common *pcom = (Msg8f40Common *)indata;
    offset += sizeof(Msg8f40Common);
    RtnCode property_rc = RtnCode::E_SUCCESS;
    std::string lck_func_en_name = SYS_PRO_NAME_FINACIAL_LCK_FUNCTION_ENEABLE;
    std::string lck_func_en_str;
    SMLK_UINT8 lck_func_en;
    property_rc = SysProperty::GetInstance()->GetValue(lck_func_en_name, lck_func_en_str);
    if (RtnCode::E_SUCCESS != property_rc)
    {
        SMLK_LOGE("SysProperty::GetInstance()->GetValue(name is %s) failed,errcode=%ld, set to off.\n", lck_func_en_name.c_str(), (SMLK_INT32)property_rc);
        lck_func_en = JTT808_SUBCMD_FINANCIAL_LOCK_OFF;
    }
    else
    {
        lck_func_en = atoi(lck_func_en_str.c_str());
    }
    /*如果未使能金融锁车功能，仅回复通用应答*/
    if (JTT808_SUBCMD_FINANCIAL_LOCK_OFF == lck_func_en)
    {
        result = JTT808_MSG_NOT_SUPPORT;
    }
    else if (JTT808_QUERY_LOCK_STATUS == pcom->subcmd) //查询
    {
        if (sizeof(Msg8f40Common) != length)
        {
            result = JTT808_MSG_NOT_SUPPORT;
            SMLK_LOGD("length=%d, but 8f40 msg len =%ld.\n", length, sizeof(Msg8f40Common) + sizeof(Msg8f40Lock));
        }
    } else if ((JTT808_ACTIVE_LOCK == pcom->subcmd) || (JTT808_INACTIVE_LOCK == pcom->subcmd)) //激活、失活
    {
        SMLK_LOGD("--------active/inactive---------");
        if (sizeof(Msg8f40Common) + sizeof(Msg8f40Active) != length) /*长度不对*/
        {   
            result = JTT808_MSG_NOT_SUPPORT;
            SMLK_LOGD("length=%d, but 8f40 msg len =%ld.\n", length, sizeof(Msg8f40Common) + sizeof(Msg8f40Lock));
        }
    } else if ((JTT808_LOCK_VAHICLE == pcom->subcmd) || (JTT808_UNLOCK_VEHICLE == pcom->subcmd)) //锁车、解锁
    {
        SMLK_LOGD("--------lock/unlock---------");
        std::string lck_status_str;
        std::string property;
        property = SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS;
        property_rc = SysProperty::GetInstance()->GetValue(property, lck_status_str);

        if (sizeof(Msg8f40Common) + sizeof(Msg8f40Lock) != length) /*长度不对*/
        {
            result = JTT808_MSG_NOT_SUPPORT;
            SMLK_LOGD("length=%d, but 8f40 msg len =%ld", length, sizeof(Msg8f40Common) + sizeof(Msg8f40Lock));
        }
        else if ((RtnCode::E_SUCCESS == property_rc) && (std::stoi(lck_status_str) == ((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_NOT_ACTIVE))))
        {
            result = JTT808_MSG_NOT_SUPPORT;
            SMLK_LOGD("finacial_lck_status not active.\n");
        }
        else
        {
            Msg8f40Lock *p = (Msg8f40Lock *)(indata + offset);
            if (m_sp_ems_ctrl->m_isYuchai || m_sp_ems_ctrl->m_isWeichai || m_sp_ems_ctrl->m_isYunnei || m_sp_ems_ctrl->m_isQQvcu)
            {
                if (be16toh(p->type) != JTT808_LCKCAR_TYPE_ROTATE)
                {
                    result = JTT808_MSG_OUT_OF_RANGE;
                }
            }
            else if (be16toh(p->type) != JTT808_LCKCAR_TYPE_TORQUE)
            {
                result = JTT808_MSG_OUT_OF_RANGE;
            }
        }
    } else if (JTT808_SEND_HANDMADE_KEYS == pcom->subcmd) //手工钥匙下发
    {
        if (sizeof(Msg8f40Common) + sizeof(Msg8f40Keys) != length) /*长度不对*/
        {
            result = JTT808_MSG_NOT_SUPPORT;
            SMLK_LOGD("length=%d, but 8f40 msg len =%ld.\n", length, sizeof(Msg8f40Common) + sizeof(Msg8f40Keys));
        }
        m_sp_ems_ctrl->CheckLoanKeys(indata, length, result);
    }
    return SMLK_RC::RC_OK;
}

SMLK_RC SmlkLoanManager::DecodeLoanCmd(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT LoanCmdMsg &queue, OUT LoanCtrlResult &result)
{
    SMLK_LOGD("*******enter in func(%s)\n",__func__);
    SMLK_UINT16 offset = 0;
    Msg8f40Common *pcom = (Msg8f40Common *)indata;
    offset += sizeof(Msg8f40Common);
    RtnCode property_rc = RtnCode::E_SUCCESS;

    if (JTT808_QUERY_LOCK_STATUS == pcom->subcmd) //查询
    {
        m_sp_ems_ctrl->DoLoanQury(queue.m_head);

    } else if ((JTT808_ACTIVE_LOCK == pcom->subcmd) || (JTT808_INACTIVE_LOCK == pcom->subcmd)) //激活、失活
    {
        SMLK_LOGD("--------active/inactive---------");
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_ACTION_JTT808, to_string(pcom->subcmd));

        Msg8f40Active *p = (Msg8f40Active *)(indata + offset);
        vector<SMLK_UINT8> gpsid = {0x00, p->gpsid[0], p->gpsid[1], p->gpsid[2]};
        vector<SMLK_UINT8> key = {0x00, p->key[0], p->key[1], p->key[2]};

        property_rc = SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_GPSID_JTT808, hex_to_string(p->gpsid, GPSID_LEN));
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("sys_param_name(%s), SysProperty SetValue failed", SYS_PRO_NAME_LCKCAR_GPSID_JTT808);
        }

        property_rc = SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_FIXED_KEY_JTT808, hex_to_string(p->key, FIXED_KEY_LEN));
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("sys_param_name(%s), SysProperty SetValue failed", SYS_PRO_NAME_LCKCAR_FIXED_KEY_JTT808);
        }

        if (JTT808_ACTIVE_LOCK == pcom->subcmd)
        {
            queue.m_cmd_vec.action = LOAN_ACTION_FLOCK_FUNC_ENABLE;
            result.m_result_vec.action = LOAN_ACTION_FLOCK_ACTIVE;
        }
        else if (JTT808_INACTIVE_LOCK == pcom->subcmd)
        {
            queue.m_cmd_vec.action = LOAN_ACTION_FLOCK_FUNC_DISABLE;
            result.m_result_vec.action = LOAN_ACTION_FLOCK_INACTIVE;
        }
        queue.m_cmd_vec.cmd_id = LOAN_CMD_FINANCIAL_LOCK_FUNC_ENABLE;
        queue.m_cmd_vec.data = LOAN_ACTION_INVALID;
        result.m_result_vec.cmd_id = LOAN_CMD_FINANCIAL_LOCK_FUNC_ENABLE;
        m_sp_ems_ctrl->DoLoanActive(queue, result.m_result_vec.result, gpsid, key);

    } else if ((JTT808_LOCK_VAHICLE == pcom->subcmd) || (JTT808_UNLOCK_VEHICLE == pcom->subcmd)) //锁车、解锁
    {
        SMLK_LOGD("--------lock/unlock---------");

        Msg8f40Lock *p = (Msg8f40Lock *)(indata + offset);

        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_ACTION_JTT808, to_string(pcom->subcmd));

        SMLK_LOGD("--------p->type--(%d)-------",be16toh(p->type));
        if (JTT808_LCKCAR_TYPE_ROTATE == be16toh(p->type))
        {
            SMLK_LOGD("--------speed---------");
            queue.m_cmd_vec.cmd_id = LOAN_CMD_ROTATE_SPEED;
            result.m_result_vec.cmd_id = LOAN_CMD_ROTATE_SPEED;
        }
        else if (JTT808_LCKCAR_TYPE_TORQUE == be16toh(p->type))
        {
            SMLK_LOGD("--------torque---------");
            queue.m_cmd_vec.cmd_id = LOAN_CMD_TORQUE_LIMIT;
            result.m_result_vec.cmd_id = LOAN_CMD_TORQUE_LIMIT;
        }
        else if (JTT808_LCKCAR_TYPE_FUEL_INJECT == be16toh(p->type))
        {
            SMLK_LOGD("--------fuel---------");
            queue.m_cmd_vec.cmd_id = LOAN_CMD_FUEL_INJECT;
            result.m_result_vec.cmd_id = LOAN_CMD_FUEL_INJECT;
        }

        if (JTT808_LOCK_VAHICLE == pcom->subcmd)
        {
            queue.m_cmd_vec.action = LOAN_MCU_ACTION_ON;
            result.m_result_vec.action = LOAN_ACTION_LOCK;
        }
        else if (JTT808_UNLOCK_VEHICLE == pcom->subcmd)
        {
            queue.m_cmd_vec.action = LOAN_MCU_ACTION_OFF;
            result.m_result_vec.action = LOAN_ACTION_UNLOCK;
        }
        queue.m_cmd_vec.data = (SMLK_DOUBLE)(be16toh(p->param)); /*大端转主机序列*/
        SMLK_LOGD("m_cmd_vec.cmd_id == (0x%04x)", queue.m_cmd_vec.cmd_id);
        SMLK_LOGD("m_cmd_vec.data == (0x%04x)", queue.m_cmd_vec.data);

        std::string s_gpsid;
        std::string property = SYS_PRO_NAME_LCKCAR_GPSID_JTT808;
        property_rc = SysProperty::GetInstance()->GetValue(property, s_gpsid);
        SMLK_UINT8 gpsid[GPSID_LEN];
        string_to_hex(s_gpsid, gpsid, GPSID_LEN);

        for(int i=0;i<GPSID_LEN;i++)
        {
            SMLK_LOGD("p->gpsid[%d]=0x%02x, gpsid[%d]=0x%02x",i, p->gpsid[i],i, gpsid[i]);
            if(p->gpsid[i] != gpsid[i])
            {
                result.m_result_vec.result = LOAN_CMD_RESULT_GPSID_ERROR;
                return SMLK_RC::RC_OK;
            }
        }

        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_VALUE_JTT808, to_string((SMLK_UINT16)(queue.m_cmd_vec.data)));

        m_sp_ems_ctrl->DoLoanLock(queue, result.m_result_vec.result);

    } else if (JTT808_SEND_HANDMADE_KEYS == pcom->subcmd) //手工钥匙下发
    {
        Msg8f40Keys *p = (Msg8f40Keys *)(indata + offset);
        vector<SMLK_UINT16> key = {be16toh(p->key_01),be16toh(p->key_02),be16toh(p->key_03),be16toh(p->key_04)};
        m_sp_ems_ctrl->DoLoanKeys(queue, key, result.m_result_vec.result);
    }
    return SMLK_RC::RC_OK;
}
SMLK_RC SmlkLoanManager::SendMsgCommonResp(LoanGeneralHead &head, SMLK_UINT8 &result)
{
    SMLK_LOGD("enter in func(%s),Result=%d.",__func__,result);
    LoanGeneralHead rctrl_head;
    Jtt808CommResp response;
    response.seq_id = htobe16(head.seq_id);
    response.msg_id = htobe16(head.msg_id);
    if (result == 2 || result == 3)
    {
        response.result = result;
    }
    else
        response.result = 0;

    memcpy(&rctrl_head, &head, sizeof(LoanGeneralHead));
    rctrl_head.msg_id = JTT808_COMMON_RESPONSE;

    m_sp_ems_ctrl->DoSendMsgToTsp(rctrl_head, (SMLK_UINT8 *)&response, sizeof(response));

    return SMLK_RC::RC_OK;
}

SMLK_RC SmlkLoanManager::ProcessLoanResult(IN LoanCtrlResult &result)
{
    m_sp_ems_ctrl->ResultEncode(result);
    return SMLK_RC::RC_OK;
}

SMLK_BOOL SmlkLoanManager::SetLoanCmd(IN LoanCmdData& data)
{
    m_loan_cmd_queue.put(data);
    return SMLK_TRUE;
}

SMLK_BOOL SmlkLoanManager::SaveLoanCmd(LoanCmdData& data)
{
    SMLK_UINT8 com_result = JTT808_MSG_SUCCESS;
    data.m_head.msg_id = JTT808_REMOTE_LOCK_RESP;
    CheckLoanCmd((SMLK_UINT8 *)data.m_message.data(), data.m_message.size(), com_result);
    SendMsgCommonResp(data.m_head, com_result);
    if (com_result != JTT808_MSG_SUCCESS)
        return SMLK_TRUE;

    Msg8f40Common *pcom = (Msg8f40Common *)data.m_message.data();
    
    if (JTT808_SEND_HANDMADE_KEYS == pcom->subcmd)
    {
        SMLK_LOGD("*******SetValue***key****");
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_FLAG_KEY_JTT808, to_string((SMLK_UINT8)(LoanStatusIgoffLckFlag::NEED_LCKCAR_WHEN_IGNON)));
    }
    else
    {
        SMLK_LOGD("*******SetValue***active****");
        //需要ign on 后执行离线消贷
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_FLAG_IGNOFF_JTT808, to_string((SMLK_UINT8)(LoanStatusIgoffLckFlag::NEED_LCKCAR_WHEN_IGNON)));
        // 离线锁车指令流水号
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_SEQ_ID_OFFLINE, to_string((SMLK_UINT16)(data.m_head.seq_id)));
        SMLK_LOGD("*******SaveValue***seqid**0x%02x**", data.m_head.seq_id);
    }

    // 锁车协议类型
    // SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_PROT_TYPE_JTT808, to_string((SMLK_UINT16)(data.m_lock_prot)));

    SMLK_UINT8 m_message[data.m_message.size()];
    for (int i=0;i<data.m_message.size();i++)
    {
        SMLK_LOGD("data.m_message:0x%02x",data.m_message[i]);
        m_message[i] = data.m_message[i];
    }
    string lockcmd(hex_to_string(m_message, data.m_message.size()));

    SMLK_LOGD("SaveLoanCmd: %s ,%d",lockcmd.c_str(),lockcmd.size());
    RtnCode property_rc = RtnCode::E_SUCCESS;
    // 锁车消息
    if (JTT808_SEND_HANDMADE_KEYS == pcom->subcmd)
    {
        property_rc = SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_MESSAGE_KEY, lockcmd);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("sys_param_name(%s), SysProperty SetValue failed", SYS_PRO_NAME_LCKCAR_MESSAGE_KEY);
        }
        SMLK_LOGD("Set property key");
    }
    else
    {
        property_rc = SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_MESSAGE_OFFLINE, lockcmd);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("sys_param_name(%s), SysProperty SetValue failed", SYS_PRO_NAME_LCKCAR_MESSAGE_OFFLINE);
        }
        SMLK_LOGD("Set property message");
    }

    return SMLK_TRUE;
}

void SmlkLoanManager::SyncIGNStatus(SMLK_BOOL ign_on)
{
    if (!CheckPtr(m_sp_ems_ctrl)) return;
    SMLK_LOGD("IGN Statue changed:%s",(ign_on == SMLK_TRUE)?"IGN ON":"IGN OFF");
    m_isIgnOn = ign_on;
    m_sp_ems_ctrl->SyncIgnStatus(ign_on);
    if (ign_on)
    {
        // m_checkofflinemsg_flag.store(true);
        if(!m_sp_ems_ctrl->m_isCummins)
            sleep(2);
        CheckOfflineLoan();
    }
}

void SmlkLoanManager::CheckOfflineLoan()
{
    SMLK_LOGD(" CheckOfflineLoan ");
    RtnCode property_rc = RtnCode::E_SUCCESS;
    std::string lck_status_keystr;
    std::string lck_status_str;
    std::string lck_status_4Gstr;
    std::string property_key = SYS_PRO_NAME_LCKCAR_FLAG_KEY_JTT808;
    std::string property = SYS_PRO_NAME_LCKCAR_FLAG_IGNOFF_JTT808;
    std::string property_4G = SYS_PRO_NAME_LCKCAR_FLAG_ANTENNA;
    property_rc = SysProperty::GetInstance()->GetValue(property_key, lck_status_keystr);

    if ((RtnCode::E_SUCCESS == property_rc)&&(atoi(lck_status_keystr.c_str()) == (SMLK_UINT8)(LoanStatusIgoffLckFlag::NEED_LCKCAR_WHEN_IGNON)))
    {
        SMLK_LOGD("*******LoadValue***key****");
        LoadOfflineCmd(LOAN_KEY_GET);
    }
    property_rc = SysProperty::GetInstance()->GetValue(property, lck_status_str);

    if ((RtnCode::E_SUCCESS == property_rc)&&(atoi(lck_status_str.c_str()) == (SMLK_UINT8)(LoanStatusIgoffLckFlag::NEED_LCKCAR_WHEN_IGNON)))
    {
        SMLK_LOGD("*******LoadValue***active****");
        LoadOfflineCmd(LOAN_MSG_GET);
    }

    property_rc = SysProperty::GetInstance()->GetValue(property_4G, lck_status_4Gstr);
    if ((RtnCode::E_SUCCESS == property_rc)&&(atoi(lck_status_4Gstr.c_str()) == (SMLK_UINT8)(LoanStatusIgoffLckFlag::NEED_LCKCAR_WHEN_IGNON)))
    {
        SMLK_LOGD("*******Load***4G*ANTENNA****");
        m_sp_ems_ctrl->Send4GantennaState();
    }

    return;
}

void SmlkLoanManager::LoadOfflineCmd(SMLK_UINT8 i_msg)
{
    SMLK_LOGD(" LoadOfflineCmd i_msg (%d)",i_msg);
    RtnCode property_rc = RtnCode::E_SUCCESS;
    LoanCmdData data;

    std::string property;
    std::string lck_msg_seqid;
    std::string property_seqid = SYS_PRO_NAME_LCKCAR_SEQ_ID_OFFLINE;
    property_rc = SysProperty::GetInstance()->GetValue(property_seqid, lck_msg_seqid);
    if (RtnCode::E_SUCCESS != property_rc)
    {
        SMLK_LOGE("sys_param_name(%s), SysProperty GetValue failed", property);
        data.m_head.seq_id = 0xffff;
    }
    else
    {
        data.m_head.seq_id = std::stoi(lck_msg_seqid);
         SMLK_LOGD("*******GetValue***seqid**0x%02x**", data.m_head.seq_id);
    }

    data.m_head.msg_id = JTT808_REMOTE_LOCK;
    // data.m_head.seq_id = 0xffff;
    data.m_head.protocol = ProtoclID::E_PROT_JTT808;
    data.m_head.qos = QOS_SEND_TCP_TIMES;
    data.m_head.priority = PRIORITY0;

    data.m_cmd_from = LoanCmdFrom::LOAN_FROM_OFFLINE_MESSAGE;
    data.m_cmd_type = LoanCmdType::LOAN_CMD_SET;

    std::string lck_msg_str;
    if (i_msg)
        property = SYS_PRO_NAME_LCKCAR_MESSAGE_OFFLINE;
    else
        property = SYS_PRO_NAME_LCKCAR_MESSAGE_KEY;

    property_rc = SysProperty::GetInstance()->GetValue(property, lck_msg_str);
    if (RtnCode::E_SUCCESS != property_rc)
    {
        SMLK_LOGE("sys_param_name(%s), SysProperty GetValue failed", property);
        return;
    }

    SMLK_LOGD("GetLoanCmd: %s ,%d",lck_msg_str.c_str(),lck_msg_str.size());
    int a = (lck_msg_str.size() - 2)/2;
    SMLK_LOGD("vector size %d",a);
    std::vector<SMLK_UINT8> message;
    message.resize(a);
    SMLK_UINT8 message1[a];
    string_to_hex(lck_msg_str, message1, a);

    for (int i=0;i<a;i++)
    {
        SMLK_LOGD("message:0x%02x",message1[i]);
        message[i] = message1[i];
    }
    data.m_message.insert(data.m_message.end(), message.begin(), message.end());
    SetLoanCmd(data);
    if (i_msg)
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_FLAG_IGNOFF_JTT808, to_string((SMLK_UINT8)(LoanStatusIgoffLckFlag::NOT_NEED_LCKCAR_WHEN_IGNON)));
    else
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_FLAG_KEY_JTT808, to_string((SMLK_UINT8)(LoanStatusIgoffLckFlag::NOT_NEED_LCKCAR_WHEN_IGNON)));

    return;
}

void SmlkLoanManager::AntennaChange()
{
    if (!CheckPtr(m_sp_ems_ctrl)) return;
    if(m_4gAntenna_flag.load())
    {
        SMLK_LOGD(" m_4gAntenna_flag change ");
        m_4gAntenna_flag.store(false);
        m_sp_ems_ctrl->Send0F3D();
    }
    if (m_gpsAntenna_flag.load())
    {
        SMLK_LOGD(" m_gpsAntenna_flag change ");
        m_gpsAntenna_flag.store(false);
        m_sp_ems_ctrl->Send0F3D();
    }
    // if (m_checkofflinemsg_flag.load())
    // {
    //     SMLK_LOGD(" m_checkofflinemsg_flag change ");
    //     m_checkofflinemsg_flag.store(false);
    //     CheckOfflineLoan();
    // }
}

void SmlkLoanManager::DecodeSMS(void *data)
{
    SMLK_LOGD(" DecodeSMS ");
    TelSMSInfo *psms = (TelSMSInfo *)data;
    const char *psms_content = psms->data.c_str();
    int sms_len = psms->data.size();
    SMLK_LOGI("msisdn:(%s), sms:(%s).sms_len:(%d)",psms->msisdn.c_str(),psms->data.c_str(),sms_len);

    LoanCmdData cmddata;
    bzero(&cmddata.m_head, sizeof(LoanGeneralHead));
    cmddata.m_cmd_from = LoanCmdFrom::LOAN_FROM_TEXT_MESSAGE;
    cmddata.m_cmd_type = LoanCmdType::LOAN_CMD_SET;
    cmddata.m_head.msg_id = JTT808_REMOTE_LOCK;
    cmddata.m_head.seq_id = 0xffff;
    cmddata.m_head.protocol = ProtoclID::E_PROT_JTT808;
    cmddata.m_head.qos = QOS_SEND_TCP_TIMES;
    cmddata.m_head.priority = PRIORITY0;

    if((strncmp((char *)psms_content, TEXT_MSG_FINANCIAL_LCK_ACTIVE, strlen(TEXT_MSG_FINANCIAL_LCK_ACTIVE)) == 0)/*激活   or 失活*/
        ||(strncmp((char *)psms_content, TEXT_MSG_FINANCIAL_LCK_INACTIVE, strlen(TEXT_MSG_FINANCIAL_LCK_INACTIVE)) == 0)){

        Pack8f40MsgLckActive(psms_content, sms_len, cmddata);

    }else if((strncmp((char *)psms_content, TEXT_MSG_FINANCIAL_LCK_LOCK, strlen(TEXT_MSG_FINANCIAL_LCK_LOCK)) == 0)/*锁车 or 解锁*/
        ||(strncmp((char *)psms_content, TEXT_MSG_FINANCIAL_LCK_UNLOCK, strlen(TEXT_MSG_FINANCIAL_LCK_UNLOCK)) == 0)){

        Pack8f40MsgLckUnlock(psms_content, sms_len, cmddata);

    }else if(strncmp((char *)psms_content, TEXT_MSG_FINANCIAL_LCK_STATUS_QUERY, strlen(TEXT_MSG_FINANCIAL_LCK_STATUS_QUERY)) == 0){/*查询锁车状态,待确认*/

        Pack8f40MsgLckQuery(psms_content, sms_len, cmddata);
    }
}

void SmlkLoanManager::Pack8f40MsgLckActive(IN void*data, IN int len, OUT LoanCmdData msg_queue)
{
    SMLK_LOGD("*******enter in func(%s)\n",__func__);

    Msg8f40Common comm;
    Msg8f40Active active;
    bzero(&comm, sizeof(comm));
    bzero(&active, sizeof(active));

    if(strncmp((char *)data, TEXT_MSG_FINANCIAL_LCK_ACTIVE, strlen(TEXT_MSG_FINANCIAL_LCK_ACTIVE)) == 0){
        comm.subcmd = JTT808_ACTIVE_LOCK;
    }else if(strncmp((char *)data, TEXT_MSG_FINANCIAL_LCK_INACTIVE, strlen(TEXT_MSG_FINANCIAL_LCK_INACTIVE)) == 0){
        comm.subcmd = JTT808_INACTIVE_LOCK;
    }

    SMLK_LOGD("sms:(%s)",(char*)data);

    string s_data = (char*)data;
    string s_gpsid;
    string s_key;

    s_gpsid = s_data.substr(20,6);
    s_key = s_data.substr(27,6);

    s_gpsid = s_gpsid.insert(0,"0x");
    s_key = s_key.insert(0,"0x");

    SMLK_LOGD("s_data:(%s)",s_data.c_str());
    SMLK_LOGD("s_gpsid:(%s) s_key:(%s)",s_gpsid.c_str(),s_key.c_str());

    string_to_hex(s_gpsid, active.gpsid, GPSID_LEN);
    string_to_hex(s_key, active.key, FIXED_KEY_LEN);

    for (int i=0;i<GPSID_LEN;i++)
    {
        SMLK_LOGD("active.gpsid:0x%02x",active.gpsid[i]);
        SMLK_LOGD("active.key:0x%02x",active.key[i]);
    }

    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&comm, (SMLK_UINT8 *)&comm + sizeof(Msg8f40Common));
    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&active, (SMLK_UINT8 *)&active + sizeof(Msg8f40Active));
    if (m_isIgnOn)
        m_loan_cmd_queue.put(msg_queue);
    else
        SaveLoanCmd(msg_queue);
}

void SmlkLoanManager::Pack8f40MsgLckUnlock(IN void*data, IN int len, OUT LoanCmdData msg_queue)
{
    SMLK_LOGD("*******enter in func(%s)\n",__func__);

    Msg8f40Common comm;
    Msg8f40Lock lock8f40;

    bzero(&comm, sizeof(comm));
    bzero(&lock8f40, sizeof(lock8f40));

    if(strncmp((char *)data, TEXT_MSG_FINANCIAL_LCK_LOCK, strlen(TEXT_MSG_FINANCIAL_LCK_LOCK)) == 0){
        comm.subcmd = JTT808_LOCK_VAHICLE;
    }else if(strncmp((char *)data, TEXT_MSG_FINANCIAL_LCK_UNLOCK, strlen(TEXT_MSG_FINANCIAL_LCK_UNLOCK)) == 0){
        comm.subcmd = JTT808_UNLOCK_VEHICLE;
    } 

    vector<pair<SMLK_UINT16, SMLK_UINT16>> start_end_index_vec;
    SeperateStringWithComma(data, len, start_end_index_vec);

    if(5 != start_end_index_vec.size()){
        SMLK_LOGE("start_end_index_vec.size() should be 5, but it's %ld. \n",start_end_index_vec.size());
        return;
    }
    string lck_type_str;/*转速，扭矩，喷油量*/
    string value_str;
    lck_type_str.insert(lck_type_str.end(), (SMLK_UINT8 *)data+start_end_index_vec[2].first, (SMLK_UINT8 *)data+start_end_index_vec[2].second+1);
    value_str.insert(value_str.end(), (SMLK_UINT8 *)data+start_end_index_vec[3].first, (SMLK_UINT8 *)data+start_end_index_vec[3].second+1);

    SMLK_LOGI("lck_type_str:(%s)",lck_type_str.c_str());
    SMLK_LOGI("value_str:(%s)",value_str.c_str());

    switch (atoi(lck_type_str.c_str())) {
        case LCKCAR_PROT_TYPE_QINGQI_MD5:
        case LCKCAR_PROT_TYPE_QINGQI_YUCHAI:
        case LCKCAR_PROT_TYPE_CUMMINS:
        case LCKCAR_PROT_TYPE_YUNNEI:
            lock8f40.type = be16toh(JTT808_LCKCAR_TYPE_ROTATE);
            break;
        case LCKCAR_PROT_TYPE_YIQI_TSC1:
        case LCKCAR_PROT_TYPE_QINGQI_TSC1:
            lock8f40.type = be16toh(JTT808_LCKCAR_TYPE_TORQUE);
            break;
        case LCKCAR_PROT_TYPE_QUANCHAI:
            lock8f40.type = be16toh(JTT808_LCKCAR_TYPE_FUEL_INJECT);
            break;
        default:
            lock8f40.type = be16toh(JTT808_LCKCAR_TYPE_TORQUE);
            SMLK_LOGE("unregistered event id: %d", atoi(lck_type_str.c_str()));
            break;
    }
    lock8f40.param = be16toh(atoi(value_str.c_str()));
    SMLK_LOGD("!!!!!!!!!!lock8f40.type=0x%02x,lock8f40.param=0x%02x.\n",lock8f40.type, lock8f40.param);

    string s_data = (char*)data;
    string s_gpsid;

    auto pos = s_data.rfind(",");
    if (pos == string::npos)
    {
        return;
    }
    SMLK_LOGD("pos = %d",pos);
    s_gpsid = s_data.substr(pos+1,6);
    s_gpsid = s_gpsid.insert(0,"0x");

    string_to_hex(s_gpsid, lock8f40.gpsid, GPSID_LEN);
    for (int i=0;i<GPSID_LEN;i++)
    {
        SMLK_LOGD("active.gpsid:0x%02x",lock8f40.gpsid[i]);
    }

    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&comm, (SMLK_UINT8 *)&comm + sizeof(Msg8f40Common));
    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&lock8f40, (SMLK_UINT8 *)&lock8f40 + sizeof(Msg8f40Lock));
    if (m_isIgnOn)
        m_loan_cmd_queue.put(msg_queue);
    else
        SaveLoanCmd(msg_queue);
}

void SmlkLoanManager::Pack8f40MsgLckQuery(IN void *, IN int , OUT LoanCmdData msg_queue)
{
    SMLK_LOGD("*******enter in func(%s)\n",__func__);

    Msg8f40Common query;
    bzero(&query, sizeof(Msg8f40Common));

    query.subcmd = JTT808_QUERY_LOCK_STATUS;

    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&query, (SMLK_UINT8 *)&query + sizeof(query));
    if (m_isIgnOn)
        m_loan_cmd_queue.put(msg_queue);
    else
        SaveLoanCmd(msg_queue);
}

void SmlkLoanManager::SetTsc1State()
{
    if (!CheckPtr(m_sp_ems_ctrl)) return;
    m_sp_ems_ctrl->SetTsc1State();
    if (m_isIgnOn)
    {
        m_sp_ems_ctrl->Send4GantennaState();
    }
    else
    {
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_FLAG_ANTENNA, to_string((SMLK_UINT8)(LoanStatusIgoffLckFlag::NEED_LCKCAR_WHEN_IGNON)));
    }
}

void SmlkLoanManager::GetRctrlHeadFromIpcHead(IN IpcTspHead &ipc_head, OUT LoanGeneralHead &rctrl_head)
{
    rctrl_head.msg_id = ipc_head.msg_id;
    rctrl_head.seq_id = ipc_head.seq_id;
    rctrl_head.protocol = (smartlink::ProtoclID)ipc_head.protocol;
    rctrl_head.qos = ipc_head.qos;
    rctrl_head.priority = ipc_head.priority;
    SMLK_LOGD("protocol(%d) msg_id(0x%04x) seq_id(0x%04x) qos(%d) priority(%d) reserved(%ld) reserved_dw(%ld)",
    rctrl_head.protocol, rctrl_head.msg_id,rctrl_head.seq_id,rctrl_head.qos,rctrl_head.priority,ipc_head.reserved,ipc_head.reserved_dw);
}

void SmlkLoanManager::SyncTspLoanMsg(IN IpcTspHead &ipc_head, IN void* data, std::size_t len)
{
    SMLK_LOGD("*******enter in func(%s)*******",__func__);
    if (!CheckPtr(m_sp_ems_ctrl)) return;

    if(JTT808_REMOTE_LOCK == ipc_head.msg_id)
    {
        LoanCmdData cmd;
        GetRctrlHeadFromIpcHead(ipc_head, cmd.m_head);
        cmd.m_cmd_from = LoanCmdFrom::LOAN_FROM_TSP_MODULE;
        cmd.m_cmd_type = LoanCmdType::LOAN_CMD_SET;
        cmd.m_message.insert(cmd.m_message.end(), (SMLK_UINT8 *)data, (SMLK_UINT8 *)data+len);
        Msg8f40Common *pcom = (Msg8f40Common *)cmd.m_message.data();

        if (JTT808_QUERY_LOCK_STATUS == pcom->subcmd)
        {
            SMLK_LOGD("*******QUERY*******");
            SetLoanCmd(cmd);
        }
        else if (m_isIgnOn)
        {
            SMLK_LOGD("*******m_isIgnOn*******");
            if (m_sp_ems_ctrl->m_isCummins)
                SaveLoanCmd(cmd);
            else
                SetLoanCmd(cmd);
        }
        else
        {
            SMLK_LOGD("*******m_isIgnOff*******");
            SaveLoanCmd(cmd);
        }
    }
}

}
}
/*****************************************************************************/
/**
* \file       smlk_loan_EMS_WEICHAI.cpp
* \date       2022/1/5
* \author     wukai
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan ctrl
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/


#include "smlk_loan_EMS_WEICHAI.h"
using namespace std;
namespace smartlink {
namespace LOAN {

SMLK_BOOL SMLK_LOAN_EMS_WEICHAI::Send0F3D(SMLK_BOOL need_handshake)
{
    SMLK_LOGD("enter in func(%s).\n", __func__);

    GetLckCarStatusAndSendStatus(LOAN_NO_UDS);
}

SMLK_BOOL SMLK_LOAN_EMS_WEICHAI::DoLoanActive(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result, std::vector<SMLK_UINT8> &i_gpsid, std::vector<SMLK_UINT8> &i_key)
{
    SMLK_LOGD("*******enter in func::(%s)*******",__func__);
    if((i_gpsid[1] == 0x00 && i_gpsid[2] == 0x00 && i_gpsid[3] == 0x00) ||
       (i_gpsid[1] == 0xFF && i_gpsid[2] == 0xFF && i_gpsid[3] == 0xFF))
    {
        SMLK_LOGD("*******gpsID all 00 or FF*******");
        result = LOAN_CMD_RESULT_GPSID_ALL_F;
        return SMLK_TRUE;
    }

    SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_TYPE_JTT808, to_string((SMLK_UINT8)(JTT808_LCKCAR_TYPE_ROTATE)));
    LoanSetMcuCmd mcu_cmd;
    mcu_cmd.cmd_id = LOAN_CMD_FINANCIAL_LOCK_FUNC_ENABLE;
    mcu_cmd.action = queue.m_cmd_vec.action;
    mcu_cmd.data_len = 8;
    mcu_cmd.data = queue.m_cmd_vec.data;
    i_gpsid.insert(i_gpsid.end(), i_key.begin(), i_key.end());
    if (SendMcuCmd(mcu_cmd, i_gpsid))
    {
        result = LOAN_CMD_RESULT_OK;
    }
    else
    {
        result = LOAN_CMD_RESULT_ERROR;
    }

    return SMLK_TRUE;
}

SMLK_BOOL SMLK_LOAN_EMS_WEICHAI::CheckIgnonTime()
{
    return SMLK_TRUE;
}

SMLK_BOOL SMLK_LOAN_EMS_WEICHAI::ResultEncode(IN LoanCtrlResult &result)
{
    SMLK_LOGD("enter in func(SMLK_LOAN_EMS_WEICHAI::%s)\n", __func__);
    SMLK_UINT8 lck_result = 0;

    std::vector<SMLK_UINT8> output_vec;
    Msg0f40Resp resp;
    bzero(&resp, sizeof(Msg0f40Resp));
    resp.seq_id = htobe16(result.m_head.seq_id);
    resp.lock_prot = LCKCAR_PROT_TYPE_QINGQI_MD5;

    if(LOAN_CMD_RESULT_AGREEMENT_ERROR == result.m_result_vec.result)
    {
        resp.result = LOAN_CMD_RESULT_MSG_ERROR;
    }
    else if (LOAN_CMD_RESULT_GPSID_ALL_F == result.m_result_vec.result)
    {
        resp.result = LOAN_CMD_RESULT_GPSID_ALL_F;
    }
    else if (LoanCmdType::LOAN_CMD_SET == result.m_result_type)
    {
        if (LOAN_CMD_FINANCIAL_LOCK_FUNC_ENABLE == result.m_result_vec.cmd_id) {
            SMLK_LOGD("--------0---------");
            if (LOAN_ACTION_UNLOCK == result.m_result_vec.action) {
                resp.resp_cmd = JTT808_INACTIVE_LOCK;
            } else if (LOAN_ACTION_LOCK == result.m_result_vec.action) {
                resp.resp_cmd = JTT808_ACTIVE_LOCK;
            }
        }
        else if (LOAN_CMD_ROTATE_SPEED == result.m_result_vec.cmd_id) {
            SMLK_LOGD("--------1---------");
            if (LOAN_ACTION_UNLOCK == result.m_result_vec.action) {
                resp.resp_cmd = JTT808_UNLOCK_VEHICLE;
            } else if (LOAN_ACTION_LOCK == result.m_result_vec.action) {
                resp.resp_cmd = JTT808_LOCK_VAHICLE;
            }
        }

        RtnCode property_rc = RtnCode::E_SUCCESS;
        lck_result = result.m_result_vec.result;
        SMLK_LOGE("lck_result=%d .\n", lck_result);

        string name_lckcar = SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS;
        if (JTT808_ACTIVE_LOCK == resp.resp_cmd)
        {
            SMLK_LOGD("--------3---------");
            if (LOAN_CMD_RESULT_OK == lck_result)
            {
                resp.result = LOAN_CMD_ACTIVE_LOCK_SUCCESS;
                property_rc = SysProperty::GetInstance()->SetValue(name_lckcar, to_string((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_IS_ACTIVE)));
                if (RtnCode::E_SUCCESS != property_rc)
                {
                    SMLK_LOGE("name_lckcar(%s), error_code(%d), SysProperty SetValue failed.\n", name_lckcar.c_str(), (SMLK_UINT32)property_rc);
                }
            }
            else
            {
                resp.result = LOAN_CMD_ACTIVE_LOCK_FAILED;
            }
        }
        else if (JTT808_INACTIVE_LOCK == resp.resp_cmd)
        {
            SMLK_LOGD("--------4---------");
            if (LOAN_CMD_RESULT_OK == lck_result)
            {
                resp.result = LOAN_CMD_UNACTIVE_LOCK_SUCCESS;
                property_rc = SysProperty::GetInstance()->SetValue(name_lckcar, to_string((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_NOT_ACTIVE)));
                if (RtnCode::E_SUCCESS != property_rc)
                {
                    SMLK_LOGE("name_lckcar(%s), error_code(%d), SysProperty SetValue failed.\n", name_lckcar.c_str(), (SMLK_UINT32)property_rc);
                }
            }
            else
            {
                resp.result = LOAN_CMD_UNACTIVE_LOCK_FAILED;
            }
        }
        else if (JTT808_LOCK_VAHICLE == resp.resp_cmd)
        {
            SMLK_LOGD("--------5---------");
            std::string lck_status_str;
            property_rc = SysProperty::GetInstance()->GetValue(name_lckcar, lck_status_str);

            if (LOAN_CMD_RESULT_OK == lck_result)
            {
                resp.result = LOAN_CMD_LOCKCAR_SET_SUCCESS;
                property_rc = SysProperty::GetInstance()->SetValue(name_lckcar, to_string((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_IS_LOCKED)));
                if (RtnCode::E_SUCCESS != property_rc)
                {
                    SMLK_LOGE("name_lckcar(%s), error_code(%d), SysProperty SetValue failed.\n", name_lckcar.c_str(), (SMLK_UINT32)property_rc);
                }
            }
            else if (LOAN_CMD_RESULT_GPSID_ERROR  == lck_result)
            {
                resp.result = LOAN_CMD_RESULT_GPSID_ERROR;
            }
            else
            {
                resp.result = LOAN_CMD_LOCKCAR_SET_FAILED;
            }
        }
        else if (JTT808_UNLOCK_VEHICLE == resp.resp_cmd)
        {
            SMLK_LOGD("--------6---------");
            if (LOAN_CMD_RESULT_OK == lck_result)
            {
                resp.result = LOAN_CMD_UNLOCKCAR_SET_SUCCESS;

                property_rc = SysProperty::GetInstance()->SetValue(name_lckcar, to_string((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_IS_ACTIVE)));
                if (RtnCode::E_SUCCESS != property_rc)
                {
                    SMLK_LOGE("name_lckcar(%s), error_code(%d), SysProperty SetValue failed.\n", name_lckcar.c_str(), (SMLK_UINT32)property_rc);
                }
            }
            else if (LOAN_CMD_RESULT_GPSID_ERROR  == lck_result)
            {
                resp.result = LOAN_CMD_RESULT_GPSID_ERROR;
            }
            else
            {
                resp.result = LOAN_CMD_UNLOCKCAR_SET_FAILED;
            }
        }

        /*用于后续消贷状态查询*/
        string name = SYS_PRO_NAME_LCKCAR_RESULT_JTT808;
        property_rc = SysProperty::GetInstance()->SetValue(name, to_string(resp.result));
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("name(%s), resp.result(%d), SysProperty SetValue failed.\n", name.c_str(), resp.result);
        }
    }
    SMLK_LOGD("--------resp.result(0x%02x)---------",resp.result);
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&resp, (SMLK_UINT8 *)&resp + sizeof(Msg0f40Resp));

    LoanGeneralHead head_temp;
    memcpy(&head_temp, &result.m_head, sizeof(LoanGeneralHead));
    head_temp.qos = QOS_SEND_TCP_TIMES;
    DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());

    return SMLK_TRUE;
}

void SMLK_LOAN_EMS_WEICHAI::EncodeMcuReport(SMLK_UINT8 result)
{
    SMLK_LOGD("enter in func(SMLK_LOAN_EMS_WEICHAI::%s)\n", __func__);
    std::vector<SMLK_UINT8> output_vec;
    Msg0f40Resp resp;
    bzero(&resp, sizeof(Msg0f40Resp));
    resp.seq_id = 0xffff;
    resp.lock_prot = LCKCAR_PROT_TYPE_QINGQI_MD5;

    string seqid_name = SYS_PRO_NAME_LCKCAR_SEQ_ID;
    string seqid;
    SysProperty::GetInstance()->GetValue(seqid_name, seqid);

    string lock_action_name = SYS_PRO_NAME_LCKCAR_ACTION_JTT808;
    string lock_action;
    SysProperty::GetInstance()->GetValue(lock_action_name, lock_action);
    resp.resp_cmd = stoi(lock_action);

    string lock_name = SYS_PRO_NAME_LCKCAR_RESULT_JTT808;

    switch (result) {
    case LOAN_MCU_REPORT_NORMAL:
        return;
    case LOAN_MCU_REPORT_ABNORMAL:
        resp.result = LOAN_CMD_RESULT_BUS_EXCEP;
        break;
    case LOAN_MCU_REPORT_HAND_SHAKE_FAIL:
        resp.result = LOAN_CMD_CHECKCODE_FAIL;
        break;
    case LOAN_MCU_REPORT_LOCK_SUCCESS:
        resp.seq_id = stoi(seqid);
        resp.result = LOAN_CMD_LOCKCAR_SUCCESS;
        SysProperty::GetInstance()->SetValue(lock_name, to_string(resp.result));
        break;
    case LOAN_MCU_REPORT_LOCK_FAIL:
        resp.seq_id = stoi(seqid);
        resp.result = LOAN_CMD_LOCKCAR_FAILED;
        SysProperty::GetInstance()->SetValue(lock_name, to_string(resp.result));
        break;
    case LOAN_MCU_REPORT_DISABLE:
        return;
    case LOAN_MCU_REPORT_ENABLE:
        SysProperty::GetInstance()->SetValue(lock_name, to_string(LOAN_CMD_ACTIVE_LOCK_SUCCESS));
        return; 
    case LOAN_MCU_REPORT_LOCK:
        SysProperty::GetInstance()->SetValue(lock_name, to_string(LOAN_CMD_LOCKCAR_SUCCESS));
        return;
    case LOAN_MCU_REPORT_GPSID_ERROR:
        resp.result = LOAN_CMD_ECU_GPSID_ERROR;
        break;
    case LOAN_MCU_REPORT_KEY_ERROR:
        resp.result = LOAN_CMD_MCU_REPORT_KEY_ERROR;
        break;
    case LOAN_MCU_REPORT_ECU_OVERTIME:
        resp.result = LOAN_CMD_HAND_SHAKE_OVERTIME;
        break;
    case LOAN_MCU_REPORT_REQ_OVERTIME:
        resp.result = LOAN_CMD_REQ_OVERTIME;
        break;
    case LOAN_MCU_REPORT_UNLOCK_SUCC:
        resp.seq_id = stoi(seqid);
        resp.result = LOAN_CMD_UNLOCKCAR_SUCCESS;
        SysProperty::GetInstance()->SetValue(lock_name, to_string(resp.result));
        break;
    case LOAN_MCU_REPORT_UNLOCK_FAIL:
        resp.seq_id = stoi(seqid);
        resp.result = LOAN_CMD_UNLOCKCAR_FAILED;
        SysProperty::GetInstance()->SetValue(lock_name, to_string(resp.result));
        break;
    case LOAN_MCU_REPORT_INACTIVE_SUCC:
        resp.result = LOAN_CMD_ACTIVE_LOCK_SUCCESS;
        break;
    case LOAN_MCU_REPORT_INACTIVE_FAIL:
        resp.result = LOAN_CMD_ACTIVE_LOCK_FAILED;
        break;
    default:
        resp.result = LOAN_CMD_RESULT_ERROR;
        break;
    }

    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&resp, (SMLK_UINT8 *)&resp + sizeof(Msg0f40Resp));
    LoanGeneralHead head;
    bzero(&head, sizeof(LoanGeneralHead));
    head.msg_id = JTT808_REMOTE_LOCK_RESP;
    head.seq_id = resp.seq_id;
    head.protocol = ProtoclID::E_PROT_JTT808;
    head.qos = QOS_SEND_TCP_TIMES;

    DoSendMsgToTsp(head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());

}

}
}

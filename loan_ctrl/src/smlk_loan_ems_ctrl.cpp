/*****************************************************************************/
/**
* \file       smlk_loan_ems_ctrl.cpp
* \author     wukai
* \date       2021/08/23
* \version    Tbox2.0 V1
* \brief      提供esync 调用接口
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan vehicle impl
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#include "smlk_loan_ecu_impl.h"
#include "smlk_spi_manager.h"

using namespace smartlink::CAN;
using namespace std;

namespace smartlink
{
namespace LOAN
{


SMLK_BOOL SmlkEmsLoanCtrl::Send0F3D(SMLK_BOOL need_handshake)
{
    SMLK_LOGD("enter in func: %s",__func__);
    if(EnterUdsMode())
        GetLckCarStatusAndSendStatus(LOAN_ENTER_UDS_SUCCESS);
    else
        GetLckCarStatusAndSendStatus(LOAN_ENTER_UDS_ERROR);
    ExitUdsMode();
}

SMLK_BOOL SmlkEmsLoanCtrl::DoLoanQury(IN LoanGeneralHead &head)
{
    SMLK_LOGD("enter in func: %s",__func__);
    SendMsg0F40(head);
}

void SmlkEmsLoanCtrl::CheckLoanKeys(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT SMLK_UINT8 &result)
{
    SMLK_LOGD("enter in func: %s",__func__);
    return;
}

void SmlkEmsLoanCtrl::DoLoanKeys(IN LoanCmdMsg &queue, std::vector<SMLK_UINT16> &key, OUT SMLK_UINT8 &result)
{
    SMLK_LOGD("enter in func: %s",__func__);
    return;
}

SMLK_BOOL SmlkEmsLoanCtrl::DoLoanActive(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result, std::vector<SMLK_UINT8> &i_gpsid, std::vector<SMLK_UINT8> &i_key)
{
    SMLK_LOGD("*******enter in func SmlkEmsLoanCtrl::(%s)*******",__func__);

    SMLK_UINT8 loop_count = LOAN_FAILED_MAX_RETRY_TIMES;
    do
    {
        result = LOAN_CMD_RESULT_ERROR;
        if (!EnterUdsMode()) {
            SmlkUdsMode mode = SmlkUds::getInstance()->GetCurrentUdsMode();
            SMLK_LOGD("*******CurrentUdsMode  (%d) *******",mode);
            if (mode == SmlkUdsMode::UDS_DIAG_MODE_LOAN)
                result = LOAN_CMD_OTA;
            else
                result = LOAN_CMD_RESULT_SECURE_ACCESS_ERROR;
            SMLK_LOGD("*******EnterUdsMode  Fail*******");
            continue;
        }
        if (!SendExterndMode()) {
            result = LOAN_CMD_RESULT_CHANGE_CONVERSATION_ERROR;
            SMLK_LOGD("*******SendExterndMode  Fail*******");
            continue;
        }
        std::vector<SMLK_UINT8> vec_ = {};
        if (!GetSeedRequest(vec_)) {
            result = LOAN_CMD_RESULT_SECURE_ACCESS_ERROR;
            SMLK_LOGD("*******GetSeedRequest  Fail*******");
            continue;
        }
        if (!EncryptSeedKey(vec_)) {
            result = LOAN_CMD_RESULT_SECURE_ACCESS_ERROR;
            SMLK_LOGD("*******EncryptSeedKey  Fail*******");
            continue;
        }
        if (!UploadKey(vec_)) {
            result = LOAN_CMD_RESULT_SECURE_ACCESS_ERROR;
            SMLK_LOGD("*******UploadKey  Fail*******");
            continue;
        }
        if (!WriteLoanFlag(queue.m_cmd_vec.action)) {
            result = LOAN_CMD_RESULT_WRITE_ERROR;
            SMLK_LOGD("*******WriteLoanFlag  Fail*******");
            continue;
        }
        SMLK_UINT8 loan_flag_uint = 0;
        if (!ReadLoanFlag(&loan_flag_uint)) {
            result = LOAN_CMD_RESULT_READ_ERROR;
            SMLK_LOGD("*******ReadLoanFlag  Fail*******");
            continue;
        } else {
            if((loan_flag_uint == 0 && queue.m_cmd_vec.action == LOAN_ACTION_FLOCK_INACTIVE)
             || (loan_flag_uint == 1 && queue.m_cmd_vec.action == LOAN_ACTION_FLOCK_ACTIVE))
            {
                result = LOAN_CMD_RESULT_OK;
                SMLK_LOGD("*******DoLoanActive  Succ*******");
            }
            else
            {
                result = LOAN_CMD_RESULT_READ_ERROR;
                SMLK_LOGD("*******ReadLoanFlag value Fail*******");
            }
        }

    } while(((--loop_count) > 0) && (result != LOAN_CMD_RESULT_OK));

    SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_TYPE_JTT808, to_string((SMLK_UINT8)(JTT808_LCKCAR_TYPE_TORQUE)));
    if (LOAN_CMD_RESULT_OK == result)
    {
        LoanSetMcuCmd mcu_cmd;
        mcu_cmd.cmd_id = LOAN_CMD_FINANCIAL_LOCK_FUNC_ENABLE;
        mcu_cmd.action = queue.m_cmd_vec.action;
        mcu_cmd.data_len = 0;
        SendMcuCmd(mcu_cmd);
    }
    ExitUdsMode();

    return SMLK_TRUE;
}

SMLK_BOOL SmlkEmsLoanCtrl::DoLoanLock(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result)
{
    SMLK_LOGD("enter in func: %s",__func__);
    std::string lck_status_str;

    //保存锁车指令的流水号
    SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_SEQ_ID, to_string(htobe16(queue.m_head.seq_id)));

    LoanSetMcuCmd mcu_cmd;
    mcu_cmd.cmd_id = queue.m_cmd_vec.cmd_id;
    mcu_cmd.action = queue.m_cmd_vec.action;
    if (mcu_cmd.cmd_id == LOAN_CMD_TORQUE_LIMIT || mcu_cmd.cmd_id == LOAN_CMD_FUEL_INJECT)
    {
        mcu_cmd.data_len = 1;
        mcu_cmd.data = queue.m_cmd_vec.data;
    }
    else {
        mcu_cmd.data_len = 2;
        mcu_cmd.data = htobe16(queue.m_cmd_vec.data);
    }
    SMLK_LOGD("m_cmd_vec.data == %ld", queue.m_cmd_vec.data);
    SMLK_LOGD("mcu_cmd.data == (0x%04x)", mcu_cmd.data);

    if (SendMcuCmd(mcu_cmd))
        result = LOAN_CMD_RESULT_OK;
    else
        result = LOAN_CMD_RESULT_ERROR;

    return SMLK_TRUE;
}

SMLK_BOOL SmlkEmsLoanCtrl::ResultEncode(IN LoanCtrlResult &result)
{
    SMLK_LOGD("enter in func(SmlkEmsLoanCtrl::%s)\n", __func__);
    SMLK_UINT8 lck_result = 0;

    std::vector<SMLK_UINT8> output_vec;
    Msg0f40Resp resp;
    bzero(&resp, sizeof(Msg0f40Resp));
    resp.seq_id = htobe16(result.m_head.seq_id);
    

    if(LOAN_CMD_RESULT_AGREEMENT_ERROR== result.m_result_vec.result)
    {
        // resp.result = LOAN_CMD_RESULT_MSG_ERROR;
        return SMLK_FALSE;
    }
    else if (LoanCmdType::LOAN_CMD_SET == result.m_result_type)
    {

        resp.lock_prot = LCKCAR_PROT_TYPE_YIQI_TSC1;

        if (LOAN_CMD_FINANCIAL_LOCK_FUNC_ENABLE == result.m_result_vec.cmd_id) {
            if (LOAN_ACTION_UNLOCK == result.m_result_vec.action) {
                resp.resp_cmd = JTT808_INACTIVE_LOCK;
            } else if (LOAN_ACTION_LOCK == result.m_result_vec.action) {
                resp.resp_cmd = JTT808_ACTIVE_LOCK;
            }
        }
        else if (LOAN_CMD_ROTATE_SPEED == result.m_result_vec.cmd_id
                || LOAN_CMD_TORQUE_LIMIT == result.m_result_vec.cmd_id
                || LOAN_CMD_FUEL_INJECT == result.m_result_vec.cmd_id) {
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
            if (LOAN_CMD_RESULT_VEHICLE_OFF == lck_result || LOAN_CMD_RESULT_BUS_EXCEP == lck_result || LOAN_CMD_RESULT_CONTROLLER_ERROR == lck_result ||
                LOAN_CMD_RESULT_CHANGE_CONVERSATION_ERROR == lck_result || LOAN_CMD_RESULT_SECURE_ACCESS_ERROR == lck_result || LOAN_CMD_RESULT_WRITE_ERROR == lck_result ||
                LOAN_CMD_RESULT_READ_ERROR == lck_result || LOAN_CMD_RESULT_PARTS_CTRL_FAIL == lck_result || LOAN_CMD_RESULT_CHECKCODE_ERROR  == lck_result)
            {
                resp.result = lck_result;
            }
            else if (LOAN_CMD_RESULT_OK == lck_result)
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
            if (LOAN_CMD_RESULT_VEHICLE_OFF == lck_result || LOAN_CMD_RESULT_BUS_EXCEP == lck_result || LOAN_CMD_RESULT_CONTROLLER_ERROR == lck_result ||
                LOAN_CMD_RESULT_CHANGE_CONVERSATION_ERROR == lck_result || LOAN_CMD_RESULT_SECURE_ACCESS_ERROR == lck_result || LOAN_CMD_RESULT_WRITE_ERROR == lck_result ||
                LOAN_CMD_RESULT_READ_ERROR == lck_result || LOAN_CMD_RESULT_PARTS_CTRL_FAIL == lck_result || LOAN_CMD_RESULT_CHECKCODE_ERROR  == lck_result)
            {
                resp.result = lck_result;
            }
            else if (LOAN_CMD_RESULT_OK == lck_result)
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
            std::string lck_status_str;
            property_rc = SysProperty::GetInstance()->GetValue(name_lckcar, lck_status_str);

            if ((RtnCode::E_SUCCESS == property_rc) && (atoi(lck_status_str.c_str()) == ((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_NOT_ACTIVE))))
            {
                SMLK_LOGD("finacial_lck_status not active.\n");
                if (LOAN_CMD_RESULT_ERROR == lck_result)
                {
                    resp.result = LOAN_CMD_RESULT_ERROR;
                }
            }
            else
            {
                if (LOAN_CMD_RESULT_OK == lck_result)
                {
                    resp.result = LOAN_CMD_LOCKCAR_SUCCESS;
                    property_rc = SysProperty::GetInstance()->SetValue(name_lckcar, to_string((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_IS_LOCKED)));
                    if (RtnCode::E_SUCCESS != property_rc)
                    {
                        SMLK_LOGE("name_lckcar(%s), error_code(%d), SysProperty SetValue failed.\n", name_lckcar.c_str(), (SMLK_UINT32)property_rc);
                    }
                }
                else
                {
                    resp.result = LOAN_CMD_LOCKCAR_FAILED;
                }
            }
        }
        else if (JTT808_UNLOCK_VEHICLE == resp.resp_cmd)
        {

            if (LOAN_CMD_RESULT_OK == lck_result)
            {
                resp.result = LOAN_CMD_UNLOCKCAR_SUCCESS;

                property_rc = SysProperty::GetInstance()->SetValue(name_lckcar, to_string((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_IS_ACTIVE)));
                if (RtnCode::E_SUCCESS != property_rc)
                {
                    SMLK_LOGE("name_lckcar(%s), error_code(%d), SysProperty SetValue failed.\n", name_lckcar.c_str(), (SMLK_UINT32)property_rc);
                }
            }
            else
            {
                resp.result = LOAN_CMD_UNLOCKCAR_FAILED;
            }
        }

        if (LOAN_CMD_RESULT_AGREEMENT_ERROR == lck_result)
        {
            resp.result = LOAN_CMD_RESULT_MSG_ERROR;
        }

        /*用于后续消贷状态查询*/
        string name = SYS_PRO_NAME_LCKCAR_RESULT_JTT808;
        property_rc = SysProperty::GetInstance()->SetValue(name, to_string(resp.result));
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("name(%s), resp.result(%d), SysProperty SetValue failed.\n", name.c_str(), resp.result);
        }
    }

    SMLK_LOGE("resp.result(0x%02x)", resp.result);
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&resp, (SMLK_UINT8 *)&resp + sizeof(Msg0f40Resp));

    LoanGeneralHead head_temp;
    memcpy(&head_temp, &result.m_head, sizeof(LoanGeneralHead));
    head_temp.qos = QOS_SEND_TCP_TIMES;
    DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());

    return SMLK_TRUE;
}

void SmlkEmsLoanCtrl::DeInit()
{
    // m_thread_work_flag.store(SMLK_FALSE);
    SMLK_LOGI("leave emc loan ctrl!!!");
}

}
}
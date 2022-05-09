/*****************************************************************************/
/**
* \file       smlk_loan_EMS_6.cpp
* \date       2022/1/19
* \author     wukai
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan ctrl
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/


#include "smlk_loan_EMS_G6.h"
#include "smartlink_sdk_sys_property.h"
#include "smlk_spi_manager.h"
using namespace std;
using namespace smartlink::CAN;
namespace smartlink {
namespace LOAN {

SMLK_BOOL SMLK_LOAN_EMS_G6::Send0F3D(SMLK_BOOL need_handshake)
{
    SMLK_LOGD("*******Send0F3D*******");
    if (EnterUdsMode()) {
        SMLK_LOGD("*******0*******");
        if (need_handshake)
        {
            SMLK_LOGD("*******2*******");
            if (SMLK_RC::RC_OK != FinacailCheckCodeProcess(true))
            {
                SMLK_LOGD("*******3*******");
                Pack0F40MsgQuery(GetVinEinFlagResult::ECU_HANDSHAKE_FAILED);
            }
        }
        GetLckCarStatusAndSendStatus(LOAN_ENTER_UDS_SUCCESS);
        ExitUdsMode();
    } else {
        SMLK_LOGD("*******1*******");
        if (need_handshake)
        {
            FinacailCheckCodeProcess(false);
            Pack0F40MsgQuery(GetVinEinFlagResult::ECU_HANDSHAKE_FAILED);
        }
        GetLckCarStatusAndSendStatus(LOAN_ENTER_UDS_ERROR);
        ExitUdsMode();
    }
}

SMLK_BOOL SMLK_LOAN_EMS_G6::LoanCheckCode(SMLK_BOOL active)
{
    return SMLK_FALSE;
}


SMLK_BOOL SMLK_LOAN_EMS_G6::DoLoanActive(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result, std::vector<SMLK_UINT8> &i_gpsid, std::vector<SMLK_UINT8> &i_key)
{
    SMLK_LOGD("*******enter in func SMLK_LOAN_EMS_G6::(%s)*******",__func__);

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
    if ((LOAN_CMD_RESULT_OK == result) && (LOAN_ACTION_FLOCK_ACTIVE == queue.m_cmd_vec.action)) {
        if(SMLK_RC::RC_OK != FinacailCheckCodeProcess(true)){
            result = LOAN_CMD_RESULT_CHECKCODE_ERROR;
            SMLK_LOGD("*******CheckCode  Fail*******");
        }
    }
    if (LOAN_CMD_RESULT_OK == result)
    {
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_TYPE_JTT808, to_string((SMLK_UINT8)(JTT808_LCKCAR_TYPE_TORQUE)));
        LoanSetMcuCmd mcu_cmd;
        mcu_cmd.cmd_id = LOAN_CMD_FINANCIAL_LOCK_FUNC_ENABLE;
        mcu_cmd.action = queue.m_cmd_vec.action;
        mcu_cmd.data_len = 0;
        SendMcuCmd(mcu_cmd);
    }
    ExitUdsMode();
    return SMLK_TRUE;
}

}
}

/*****************************************************************************/
/**
* \file       smlk_loan_VCU_G6.cpp
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


#include "smlk_loan_EMS_CUMMINS.h"
#include "smartlink_sdk_sys_property.h"
using namespace std;
namespace smartlink {
namespace LOAN {

SMLK_BOOL SMLK_LOAN_EMS_CUMMINS::Send0F3D(SMLK_BOOL need_handshake)
{
    SMLK_LOGD("enter in func(%s).\n", __func__);

    GetLckCarStatusAndSendStatus(LOAN_NO_UDS);
}

void SMLK_LOAN_EMS_CUMMINS::CheckLoanKeys(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT SMLK_UINT8 &result)
{
    SMLK_LOGD("enter in func(%s).\n", __func__);
    SMLK_UINT16 offset = 0;
    Msg8f40Common *pcom = (Msg8f40Common *)indata;
    offset += sizeof(Msg8f40Common);

    Msg8f40Keys *p = (Msg8f40Keys *)(indata + offset);
    SMLK_LOGD("subcmd=%d, key =0x%02x 0x%02x 0x%02x 0x%02x", pcom->subcmd, be16toh(p->key_01), be16toh(p->key_02), be16toh(p->key_03), be16toh(p->key_04));
    vector<SMLK_UINT16> key = {be16toh(p->key_01),be16toh(p->key_02),be16toh(p->key_03),be16toh(p->key_04)};
    for(int i=0;i<4;i++)
    {
        if((key[i] == KEY1) || (key[i] == KEY2) || (key[i] == KEY3) ||(key[i] == KEY4))
        {
            SMLK_LOGD("----i == %d----",i);
            result = JTT808_MSG_OUT_OF_RANGE;
            return;
        }
    }
    result = JTT808_MSG_SUCCESS;
    return;
}

void SMLK_LOAN_EMS_CUMMINS::DoLoanKeys(IN LoanCmdMsg &queue, std::vector<SMLK_UINT16> &key, OUT SMLK_UINT8 &result)
{
    SMLK_LOGD("enter in func(%s).\n", __func__);

    LoanSetMcuCmd mcu_cmd;
    mcu_cmd.cmd_id = LOAN_RSP_CHECK_CODE;
    mcu_cmd.action = LOAN_MCU_ACTION_ON;
    mcu_cmd.data_len = 8;
    SMLK_LOGD(" key = 0x%02x 0x%02x 0x%02x 0x%02x", be16toh(key[0]), be16toh(key[1]), be16toh(key[2]), be16toh(key[3]));
    vector<SMLK_UINT8> i_key;
    for(int i=0;i<4;i++)
    {
        i_key.push_back(be16toh(key[i]) & 0xff);
        i_key.push_back(be16toh(key[i]) >> 8);
    }
    SMLK_LOGD("  size(%d) i_key = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",i_key.size(),i_key[0],i_key[1],i_key[2],i_key[3],i_key[4],i_key[5],i_key[6],i_key[7]);

    if (SendMcuCmd(mcu_cmd, i_key))
    {
        SMLK_LOGD("----------succ------------");
        result = LOAN_CMD_RESULT_OK;
    }
    else
    {
        SMLK_LOGD("----------error------------");
        result = LOAN_CMD_RESULT_ERROR;
    }

    return;
}

SMLK_BOOL SMLK_LOAN_EMS_CUMMINS::DoLoanActive(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result, std::vector<SMLK_UINT8> &i_gpsid, std::vector<SMLK_UINT8> &i_key)
{
    SMLK_LOGD("*******enter in func SMLK_LOAN_EMS_CUMMINS::(%s)*******",__func__);

    LoanSetMcuCmd mcu_cmd;
    mcu_cmd.cmd_id = LOAN_CMD_FINANCIAL_LOCK_FUNC_ENABLE;
    mcu_cmd.action = queue.m_cmd_vec.action;
    mcu_cmd.data_len = 8;
    mcu_cmd.data = queue.m_cmd_vec.data;
    i_gpsid.insert(i_gpsid.end(), i_key.begin(), i_key.end());

    SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_TYPE_JTT808, to_string((SMLK_UINT8)(JTT808_LCKCAR_TYPE_ROTATE)));
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

SMLK_BOOL SMLK_LOAN_EMS_CUMMINS::Check0F3D()
{
    if(m_need_0f3d)
    {
        m_need_0f3d = false;
        return SMLK_TRUE;
    }
    return SMLK_FALSE;
}

SMLK_BOOL SMLK_LOAN_EMS_CUMMINS::CheckIgnonTime()
{
    std::chrono::steady_clock::time_point cur_time;
    cur_time = std::chrono::steady_clock::now();
    if(cur_time < g_ign_on_time)
    {
        SMLK_UINT32 t = static_cast<SMLK_UINT32>(std::chrono::duration<SMLK_DOUBLE, std::milli>(g_ign_on_time - cur_time).count());
        SMLK_LOGD("[Loan] has (%ld)ms left", t);
        return SMLK_TRUE;
    }
    else
    {
        return SMLK_FALSE;
    }
}

void SMLK_LOAN_EMS_CUMMINS::SyncIgnStatus(SMLK_BOOL ign_on)
{
    m_need_0f3d = false;
    if(ign_on)
    {
        g_ign_on_time = std::chrono::steady_clock::now();
        g_ign_on_time += std::chrono::seconds(m_ign_delay_time);
        std::string lck_status_str;
        RtnCode property_rc = RtnCode::E_SUCCESS;
        std::string property;
        property = SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS;
        property_rc = SysProperty::GetInstance()->GetValue(property, lck_status_str);
        /*当前是激活状态,发送0F3D中的消贷状态事件*/
        if ((RtnCode::E_SUCCESS == property_rc) && (std::stoi(lck_status_str)) >= ((SMLK_UINT8)(RctrlStatusFinaLck::STATUS_IS_ACTIVE))) {
            m_need_0f3d = true;
        }
    }
}

}
}

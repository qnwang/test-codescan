/*****************************************************************************/
/**
* \file       smlk_loan_autogen_EMS_Econtrol.h
* \date       2022/1/26
* \author     wukai
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan ctrl
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#ifndef _SMLK_LOAN_EMS_Econtrol_H_
#define _SMLK_LOAN_EMS_Econtrol_H_


#include "smlk_loan_ecu_impl.h"


namespace smartlink {
namespace LOAN {
class SMLK_LOAN_EMS_Econtrol : public SmlkEmsLoanCtrl
{
public:
    SMLK_LOAN_EMS_Econtrol()
    {
        m_read_svc_id  = 0x22;
        m_write_svc_id = 0x2E;
        m_phy_addr     = 0x18DA00F1;
        m_resp_id      = 0x18DAF100;
        m_phy_eng_addr     = 0x18DA00F1;
        m_resp_eng_id      = 0x18DAF100;
        m_ign_delay_time = 20;
        m_keylen       = 4;
        m_encrypt_lib_path     = "/oemapp/smartlink/lib/libEControl_Seedkey_Algorithm.so";
        m_encrypt_func_name    = "seedToKey_EControl";
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("VinDiD", EcuDidVarable(0xF190, "VinDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("EinDiD", EcuDidVarable(0xF80D, "EinDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanReadDiD", EcuDidVarable(0x0209, "LoanReadDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanWriteDiD", EcuDidVarable(0x0209, "LoanWriteDiD")));
        SMLK_UINT8 LoanActive_val_uint = 0x01;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanActive", EcuDidVarable((SMLK_UINT8*)&LoanActive_val_uint, 1, "LoanActive")));
        SMLK_UINT8 LoanInActive_val_uint = 0x00;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanInActive", EcuDidVarable((SMLK_UINT8*)&LoanInActive_val_uint, 1, "LoanInActive")));
        SMLK_UINT8 LoanReadActive_val_uint = 0x01;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanActive", EcuDidVarable((SMLK_UINT8*)&LoanReadActive_val_uint, 1, "LoanReadActive")));
        SMLK_UINT8 LoanReadInActive_val_uint = 0x00;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanInActive", EcuDidVarable((SMLK_UINT8*)&LoanReadInActive_val_uint, 1, "LoanReadInActive")));
    };
    virtual ~SMLK_LOAN_EMS_Econtrol(){};
protected:
    SMLK_BOOL GetSeedRequest(std::vector<SMLK_UINT8>& vec/*IN and OUT*/);
    SMLK_BOOL UploadKey(IN std::vector<SMLK_UINT8>& key);
    SMLK_BOOL EncryptSeedKey(std::vector<SMLK_UINT8>& vec /*IN OUT*/);
    // SMLK_BOOL DoLoanCmd(IN LoanCmdData& data);
};
}
}
#endif // _SMLK_LOAN_EMS_Econtrol_H_

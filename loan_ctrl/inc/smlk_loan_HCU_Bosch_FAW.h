/*****************************************************************************/
/**
* \file       smlk_loan_HCU_Bosch_FAW.h
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan ctrl
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#ifndef _SMLK_LOAN_HCU_BOSCH_FAW_H_
#define _SMLK_LOAN_HCU_BOSCH_FAW_H_


#include "smlk_loan_ecu_impl.h"


namespace smartlink {
namespace LOAN {
class SMLK_LOAN_HCU_BOSCH_FAW : public SmlkEmsLoanCtrl
{
public:
    SMLK_LOAN_HCU_BOSCH_FAW()
    {
        m_read_svc_id  = 0x22;
        m_write_svc_id = 0x2E;
        m_phy_addr     = 0x18DA27F1;
        m_resp_id      = 0x18DAF127;
        m_phy_eng_addr     = 0x18DA27F1;
        m_resp_eng_id      = 0x18DAF127;
        m_ign_delay_time = 20;
        m_keylen       = 8;
        m_encrypt_lib_path     = "/oemapp/smartlink/lib/libBosch_HCU_SecurityAccessAlgorithm.so";
        m_encrypt_func_name    = "seedToKey";
        m_isHCU = true;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("VinDiD", EcuDidVarable(0xF190, "VinDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanReadDiD", EcuDidVarable(0xF1A7, "LoanReadDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanWriteDiD", EcuDidVarable(0xF1A7, "LoanWriteDiD")));
        SMLK_UINT16 LoanActive_val_uint = 0x0100;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanActive", EcuDidVarable((SMLK_UINT8*)&LoanActive_val_uint, 2, "LoanActive")));
        SMLK_UINT16 LoanInActive_val_uint = 0x0000;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanInActive", EcuDidVarable((SMLK_UINT8*)&LoanInActive_val_uint, 2, "LoanInActive")));
        SMLK_UINT8 LoanReadActive_val_uint = 0x0001;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanActive", EcuDidVarable((SMLK_UINT8*)&LoanReadActive_val_uint, 2, "LoanReadActive")));
        SMLK_UINT8 LoanReadInActive_val_uint = 0x0000;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanInActive", EcuDidVarable((SMLK_UINT8*)&LoanReadInActive_val_uint, 2, "LoanReadInActive")));
    };
    virtual ~SMLK_LOAN_HCU_BOSCH_FAW(){};
public:
    // SMLK_BOOL DoLoanCmd(IN LoanCmdData& data);
};
}
}
#endif // _SMLK_LOAN_HCU_BOSCH_FAW_H_

/*****************************************************************************/
/**
* \file       smlk_loan_EMS_6.h
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
#ifndef _SMLK_LOAN_EMS_G6_H_
#define _SMLK_LOAN_EMS_G6_H_


#include "smlk_loan_ecu_impl.h"


namespace smartlink {
namespace LOAN {
class SMLK_LOAN_EMS_G6 : public SmlkEmsLoanCtrl
{
public:
    SMLK_LOAN_EMS_G6()
    {
        m_read_svc_id  = 0x22;
        m_write_svc_id = 0x2E;
        m_phy_addr     = 0x18DA00F1;
        m_resp_id      = 0x18DAF100;
        m_phy_eng_addr     = 0x18DA00F1;
        m_resp_eng_id      = 0x18DAF100;
        m_ign_delay_time = 20;
        m_keylen       = 4;
        m_encrypt_lib_path     = "/oemapp/smartlink/lib/libEMS_Seedkey_Algorithm.so";
        m_encrypt_func_name    = "seedToKey";
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("VinDiD", EcuDidVarable(0xF190, "VinDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("EinDiD", EcuDidVarable(0xF1A6, "EinDiD")));
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
    virtual ~SMLK_LOAN_EMS_G6(){};
protected:
    SMLK_BOOL LoanCheckCode(SMLK_BOOL active);
    // SMLK_BOOL DoLoanCmd(IN LoanCmdData& data);
    SMLK_BOOL DoLoanActive(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result, std::vector<SMLK_UINT8> &i_gpsid, std::vector<SMLK_UINT8> &i_key);
    SMLK_BOOL Send0F3D(SMLK_BOOL need_handshake = SMLK_FALSE);
};
}
}
#endif // _SMLK_LOAN_AUTOGEN_EMS_6_H_

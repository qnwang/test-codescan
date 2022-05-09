/*****************************************************************************/
/**
* \file       smlk_loan_EMS_YUNNEI.h
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
#ifndef _SMLK_LOAN_EMS_YUNNEI_H_
#define _SMLK_LOAN_EMS_YUNNEI_H_


#include "smlk_loan_ecu_impl.h"

#define mask 0x586B838A

namespace smartlink {
namespace LOAN {
    extern SMLK_RC string_to_hex(std::string string, SMLK_UINT8 *cbuf, SMLK_UINT8 buflen);
class SMLK_LOAN_EMS_YUNNEI : public SmlkEmsLoanCtrl
{
public:
    SMLK_LOAN_EMS_YUNNEI(){
        m_isYunnei = true;
        m_read_svc_id  = 0x22;
        m_write_svc_id = 0x2E;
        m_phy_addr     = 0x18DA00FA;
        m_resp_id      = 0x18DAFA00;
        m_phy_eng_addr     = 0x18DA00FA;
        m_resp_eng_id      = 0x18DAFA00;
        m_ign_delay_time = 60;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("TboxIdDiD", EcuDidVarable(0x0203, "TboxIdDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("TboxIdDelDiD", EcuDidVarable(0x0202, "TboxIdDelDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("ReadTboxIdDiD", EcuDidVarable(0xF1A4, "ReadTboxIdDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LimitDiD", EcuDidVarable(0xF1A5, "LimitDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanReadDiD", EcuDidVarable(0x0300, "LoanReadDiD")));
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanWriteDiD", EcuDidVarable(0x0300, "LoanWriteDiD")));
        SMLK_UINT8 LoanActive_val_uint = 0x01;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanActive", EcuDidVarable((SMLK_UINT8*)&LoanActive_val_uint, 1, "LoanActive")));
        SMLK_UINT8 LoanInActive_val_uint = 0x00;
        m_did_map.insert(std::pair<std::string, EcuDidVarable>("LoanInActive", EcuDidVarable((SMLK_UINT8*)&LoanInActive_val_uint, 1, "LoanInActive")));
    };
    virtual ~SMLK_LOAN_EMS_YUNNEI(){};
protected:
    SMLK_BOOL DoLoanActive(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result, std::vector<SMLK_UINT8> &i_gpsid, std::vector<SMLK_UINT8> &i_key);
    SMLK_BOOL ResultEncode(IN LoanCtrlResult &result);
    void EncodeMcuReport(SMLK_UINT8 result);
    SMLK_BOOL Send0F3D(SMLK_BOOL need_handshake = SMLK_FALSE);
};
}
}
#endif // SMLK_LOAN_EMS_YUNNEI

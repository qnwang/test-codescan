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
#ifndef _SMLK_LOAN_EMS_WEICHAI_H_
#define _SMLK_LOAN_EMS_WEICHAI_H_


#include "smlk_loan_ecu_impl.h"


namespace smartlink {
namespace LOAN {
    extern SMLK_RC string_to_hex(std::string string, SMLK_UINT8 *cbuf, SMLK_UINT8 buflen);
class SMLK_LOAN_EMS_WEICHAI : public SmlkEmsLoanCtrl
{
public:
    SMLK_LOAN_EMS_WEICHAI(){
        m_isWeichai = true;
    };
    virtual ~SMLK_LOAN_EMS_WEICHAI(){};
protected:
    SMLK_BOOL DoLoanActive(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result, std::vector<SMLK_UINT8> &i_gpsid, std::vector<SMLK_UINT8> &i_key);
    // SMLK_BOOL DoLoanLock(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result);
    SMLK_BOOL ResultEncode(IN LoanCtrlResult &result);
    SMLK_BOOL CheckIgnonTime();
    void EncodeMcuReport(SMLK_UINT8 result);
    SMLK_BOOL Send0F3D(SMLK_BOOL need_handshake = SMLK_FALSE);
};
}
}
#endif // _SMLK_LOAN_EMS_WEICHAI_H_

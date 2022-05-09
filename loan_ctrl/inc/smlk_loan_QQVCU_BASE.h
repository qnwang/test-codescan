/*****************************************************************************/
/**
* \file       smlk_loan_QQVCU_BASE.h
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
#ifndef _SMLK_LOAN_QQVCU_BASE_H_
#define _SMLK_LOAN_QQVCU_BASE_H_


#include "smlk_loan_ecu_impl.h"


namespace smartlink {
namespace LOAN {
class SMLK_LOAN_QQVCU_BASE : public SmlkEmsLoanCtrl
{
public:
    public:
    SMLK_LOAN_QQVCU_BASE(){
        m_isQQvcu = true;
    };
    virtual ~SMLK_LOAN_QQVCU_BASE(){};
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
#endif // SMLK_LOAN_QQVCU_BASE

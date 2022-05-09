/*****************************************************************************/
/**
* \file       smlk_loan_EMS_CUMMINS.h
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
#ifndef _SMLK_LOAN_EMS_CUMMINS_H_
#define _SMLK_LOAN_EMS_CUMMINS_H_


#include "smlk_loan_ecu_impl.h"


namespace smartlink {
namespace LOAN {
    extern SMLK_RC string_to_hex(std::string string, SMLK_UINT8 *cbuf, SMLK_UINT8 buflen);
class SMLK_LOAN_EMS_CUMMINS : public SmlkEmsLoanCtrl
{
public:
    SMLK_LOAN_EMS_CUMMINS(){
        m_isCummins = true;
        m_ign_delay_time = 2;
    };
    virtual ~SMLK_LOAN_EMS_CUMMINS(){};
protected:
    void DoLoanKeys(IN LoanCmdMsg &queue, std::vector<SMLK_UINT16> &key, OUT SMLK_UINT8 &result);
    void CheckLoanKeys(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT SMLK_UINT8 &result);
    SMLK_BOOL DoLoanActive(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result, std::vector<SMLK_UINT8> &i_gpsid, std::vector<SMLK_UINT8> &i_key);
    SMLK_BOOL Check0F3D();
    SMLK_BOOL Send0F3D(SMLK_BOOL need_handshake = SMLK_FALSE);
    SMLK_BOOL CheckIgnonTime();
    void SyncIgnStatus(SMLK_BOOL ign_on);
protected:
    std::condition_variable m_lockcv;
    std::mutex m_lockcv_mtx;
    std::chrono::steady_clock::time_point g_ign_on_time;
};
}
}
#endif // _SMLK_LOAN_EMS_CUMMINS_H_

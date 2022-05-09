/*****************************************************************************/
/**
* \file       smlk_loan_ctrl_api.h
* \author     wukai
* \date       2021/08/24
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan manager
******************************************************************************/
#ifndef _SMLK_LOAN_CTRL_API_H_
#define _SMLK_LOAN_CTRL_API_H_
/*****************************************************************************
*                                头文件引用                                  *
*****************************************************************************/
#include "smlk_loan_common_def.h"
#include <atomic>
namespace smartlink
{
namespace LOAN
{
extern SMLK_RC string_to_hex(std::string string, SMLK_UINT8 *cbuf, SMLK_UINT8 buflen);
class SmlkEmsLoanCtrl;
class SmlkLoanManager
{
public:
    SmlkLoanManager()
    {
        m_isIgnOn = false;
        is_need_gpsAntenna = false;
        is_need_4GAntenna = false;
        m_gpsAntenna_flag.store(false);
        m_4gAntenna_flag.store(false);
        m_checkofflinemsg_flag.store(false);
    };
    ~SmlkLoanManager(){};
    SMLK_BOOL InitServeice();
    SMLK_BOOL Init(); // get ems type and vcu type
    SMLK_BOOL DeInit();
    SMLK_BOOL Start();
    SMLK_RC ThreadLoanProcess(void);
    void ProcessMcuSerResp(void *data, int len);
    SMLK_BOOL SetLoanCmd(IN LoanCmdData&);
    SMLK_BOOL SaveLoanCmd(LoanCmdData&);
    SMLK_RC CheckLoanCmd(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT SMLK_UINT8 &result);
    SMLK_RC DecodeLoanCmd(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT LoanCmdMsg &queue, OUT LoanCtrlResult &result);
    void OnMcuEventSet(smartlink_sdk::McuEventId id, void *data, int len);
    void SyncIGNStatus(SMLK_BOOL ign_on); // true: on   false: off
    void AntennaChange();
    void DecodeSMS(void *data);
    SMLK_RC ProcessLoanResult(IN LoanCtrlResult &result);
    void GetRctrlHeadFromIpcHead(IN IpcTspHead &ipc_head, OUT LoanGeneralHead &rctrl_head);
    void SyncTspLoanMsg(IN IpcTspHead &ipc_head, IN void* data, std::size_t len);
    void Pack8f40MsgLckActive(IN void*data, IN int len, OUT LoanCmdData msg_queue);
    void Pack8f40MsgLckUnlock(IN void*data, IN int len, OUT LoanCmdData msg_queue);
    void Pack8f40MsgLckQuery(IN void *, IN int , OUT LoanCmdData msg_queue);
    SMLK_RC SendMsgCommonResp(LoanGeneralHead &head, SMLK_UINT8 &result);
    void CheckOfflineLoan();
    void LoadOfflineCmd(SMLK_UINT8 i_msg);
    void SetTsc1State();

private:
    std::shared_ptr<SmlkEmsLoanCtrl> m_sp_ems_ctrl;
    Queue<LoanCmdData> m_loan_cmd_queue;
    std::thread m_cmd_thread;
    SMLK_BOOL m_isIgnOn;
    SMLK_BOOL is_need_gpsAntenna;
    SMLK_BOOL is_need_4GAntenna;
    SMLK_UINT8 g_rctrl_ign_status;
    SMLK_UINT8 g_rctrl_ign_status_before;
    std::atomic<bool>   m_4gAntenna_flag;/*4G天线断开标志*/
    std::atomic<bool>   m_gpsAntenna_flag;/*gps天线断开标志*/
    std::atomic<bool>   m_checkofflinemsg_flag;
};
}
}


#endif // _SMLK_LOAN_CTRL_API_H_
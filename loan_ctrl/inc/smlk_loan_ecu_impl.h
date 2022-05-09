/*****************************************************************************/
/**
* \file       smlk_loan_ecu_impl.h
* \author     wukai
* \date       2021/08/24
* \version    Tbox2.0 V1
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan impl
******************************************************************************/
#ifndef _SMLK_LOAN_ECU_IMPL_H_
#define _SMLK_LOAN_ECU_IMPL_H_
/*****************************************************************************
*                                头文件引用                                  *
*****************************************************************************/
#include <atomic>
#include <map>
#include <algorithm>
#include <functional>
#include <mutex>
#include "smlk_types.h"
#include "smlk_loan_common_def.h"
#include "smlk_log.h"
#include "smartlink_sdk_location.h"
#include "smartlink_sdk_sys_power.h"
#include "smartlink_sdk_tel.h"
#include "tsp_service_api.h"
#include "smlk_property.h"
#include "smartlink_sdk_sys_property.h"

#include "smlk_queue.h"
#include <thread>
namespace smartlink
{
namespace LOAN
{
#define LOAN_WAIT_UDS_INTERVAL  1000/*ms*/
#define VinDidStr "VinDiD"
#define EinDidStr "EinDiD"
#define LoanReadDiDStr "LoanReadDiD"
#define LoanWriteDiDStr "LoanWriteDiD"
#define LoanActive "LoanActive"
#define LoanInActive "LoanInActive"
class EcuDidVarable
{
public:
    EcuDidVarable(SMLK_UINT16 did, IN std::string& tag)
        : m_tag(tag)
        , m_did(did)
        , m_synced(false)
    {

    };
    EcuDidVarable(SMLK_UINT8* data, SMLK_UINT8 len, IN std::string& tag)
        : m_tag(tag)
        , m_did(0x00)
        , m_synced(false)
    {
        m_data.insert(m_data.end(), data, data + len);
    };
    virtual ~EcuDidVarable(){};
    EcuDidVarable(){
        m_data.clear();
    };

    std::string str()
    {
        std::string temp_val;
        temp_val.insert(temp_val.end(), m_data.begin(), m_data.end());
        temp_val.erase(std::remove_if(temp_val.begin(), temp_val.end(), [](char c){
            return std::isspace(c);
        }), temp_val.end());
        return temp_val;
    };
public:
    EcuDidVarable& operator=(const EcuDidVarable& other)
    {
        m_tag = other.m_tag;
        m_did = other.m_did;
        m_data = other.m_data;
        m_synced = other.m_synced;
        return *this;
    };

    std::string m_tag;
    SMLK_UINT16 m_did;
    std::vector<SMLK_UINT8> m_data;
    SMLK_BOOL m_synced;
};
typedef SMLK_UINT32 (*GetEncryptKey) (SMLK_UINT8 *seed, SMLK_UINT32 length, SMLK_UINT8 *key, SMLK_UINT32 *retLen);
#define     LOAN_FAILED_MAX_RETRY_TIMES     3/*消贷激活失活最多重复发送三次*/

enum class VinId: SMLK_UINT8
{
    LOAN_VIN = 0,
    LOAN_EIN
};

class SmlkLoanEcuImpl
{
public:
    SmlkLoanEcuImpl()
    {
        m_ign_status = SMLK_FALSE;
        m_need_0f3d = false;
        m_isYuchai = false;
        m_isWeichai = false;
        m_isYunnei = false;
        m_isCummins = false;
        m_isHCU = false;
        m_isQQvcu = false;
    };
    virtual ~SmlkLoanEcuImpl(){};
    virtual SMLK_BOOL Init();
    virtual void DeInit(){};
    virtual void SyncIgnStatus(SMLK_BOOL ign_on);
    // virtual SMLK_BOOL DoLoanCmd(IN LoanCmdData&) = 0;
    virtual void Pack0F40MsgQuery(IN GetVinEinFlagResult reslut);
    virtual void EncodeMcuReport(SMLK_UINT8 result);
    virtual SMLK_BOOL CheckIgnonTime();
    virtual void Notify_m_cv();
    virtual void SetResult(SMLK_UINT8 result);
    virtual SMLK_BOOL Check0F3D();
    virtual void GetLckCarStatusAndSendStatus(SMLK_UINT8 flag);
    virtual SMLK_RC DoSendMsgToTsp(IN LoanGeneralHead &rctrl_head, IN SMLK_UINT8 *msg_body, IN std::size_t &length);
    virtual void SetTsc1State();
    virtual void Send4GantennaState();
    SMLK_UINT16 m_mcu_seq;/*发给mcu的seqid值*/
    SMLK_BOOL   m_isWeichai;
    SMLK_BOOL   m_isYuchai;
    SMLK_BOOL   m_isYunnei;
    SMLK_BOOL   m_isCummins;
    SMLK_BOOL   m_isHCU;
    SMLK_BOOL   m_isQQvcu;
    SMLK_BOOL   m_need_0f3d;
protected:
    virtual SMLK_BOOL EnterUdsMode();
    virtual SMLK_BOOL ExitUdsMode();
    virtual SMLK_BOOL SendExterndMode();
    virtual SMLK_BOOL GetSeedRequest(std::vector<SMLK_UINT8>& vec);
    virtual SMLK_BOOL GetYunNeiSeedRequest(std::vector<SMLK_UINT8>& vec);
    virtual SMLK_BOOL EncryptSeedKey(std::vector<SMLK_UINT8>& vec);
    virtual SMLK_BOOL UploadKey(IN std::vector<SMLK_UINT8>& vec);
    virtual SMLK_BOOL UploadYunNeiKey(IN std::vector<SMLK_UINT8>& vec);
    virtual SMLK_BOOL WriteTboxId(SMLK_UINT8 action);
    virtual SMLK_BOOL ReadTboxId(IN std::vector<SMLK_UINT8>& vec);
    virtual SMLK_BOOL WriteSpeedLimit(SMLK_UINT8 action);
    virtual SMLK_BOOL ReadSpeedLimit(SMLK_UINT8 action);
    virtual SMLK_BOOL WriteLoanFlag(SMLK_UINT8 action);
    virtual SMLK_BOOL ReadLoanFlag(SMLK_UINT8* flag);
    virtual SMLK_BOOL LoanCheckCode(SMLK_BOOL active){ return SMLK_TRUE;};
    SMLK_BOOL SendCmd(std::vector<SMLK_UINT8>& vec, SMLK_UINT16 svc_id, SMLK_UINT32 phy_addr, SMLK_UINT32 resp_addr, SMLK_UINT16 reqdid);
    virtual SMLK_BOOL SendMcuCmd(LoanSetMcuCmd& mcu_cmd);
    virtual SMLK_BOOL SendMcuCmd(LoanSetMcuCmd &mcu_cmd, std::vector<SMLK_UINT8> &i_key);
    virtual void GetLckStatusInfo(OUT LoanLckSatatusInfo &status_info, SMLK_UINT8 flag);
    virtual void SendLckCarStatus(LoanLckSatatusInfo& xd_status);
    virtual SMLK_RC FinacailCheckCode(std::vector<SMLK_UINT8>& vec);
    virtual SMLK_RC FinacailCheckCodeProcess(bool flag);
    // virtual void GetIpcHeadFromRctrlHead(IN LoanGeneralHead &rctrl_head, OUT IpcTspHead &ipc_head);
    virtual SMLK_RC SendMsg0F40(IN LoanGeneralHead &head);
    GetVinEinFlagResult GetVinEinFlagFromEcu(OUT SMLK_UINT8 *vin, OUT SMLK_UINT8 *ein, OUT SMLK_UINT8 lck_active_status);
    virtual SMLK_BOOL GetVinorEin(VinId, SMLK_UINT8*);
protected:
    SMLK_BOOL m_ign_status;
    SMLK_UINT16 m_read_svc_id;
    SMLK_UINT16 m_write_svc_id;
    SMLK_UINT32 m_phy_addr;
    SMLK_UINT32 m_resp_id;
    SMLK_UINT32 m_phy_eng_addr;
    SMLK_UINT32 m_resp_eng_id;
    SMLK_UINT8  m_keylen;
    SMLK_UINT8  m_ign_delay_time;
    SMLK_UINT8 m_loan_cmd_result;
    std::string m_encrypt_lib_path;
    std::string m_encrypt_func_name;
    std::map<std::string, EcuDidVarable> m_did_map;
    std::condition_variable m_cv;
    std::mutex m_cv_mtx;/*消贷命令锁*/
    // std::vector<SMLK_UINT8> m_checkcode;
private:
    std::chrono::steady_clock::time_point g_ign_on_time;
    SMLK_UINT8 g_rctrl_ign_status;
    SMLK_UINT8 g_rctrl_ign_status_before;
};

// using GetVinOrEinCallBack = std::function<SMLK_BOOL(VinId, std::string&)>;

class SmlkEmsLoanCtrl : public SmlkLoanEcuImpl
{
public:
    SmlkEmsLoanCtrl()
    {

    };
    virtual ~SmlkEmsLoanCtrl(){};
    virtual void DeInit();
    virtual SMLK_BOOL Send0F3D(SMLK_BOOL need_handshake = SMLK_FALSE);
    virtual SMLK_BOOL DoLoanQury(IN LoanGeneralHead & head);
    virtual SMLK_BOOL DoLoanActive(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result, std::vector<SMLK_UINT8> &i_gpsid, std::vector<SMLK_UINT8> &i_key);
    virtual SMLK_BOOL DoLoanLock(IN LoanCmdMsg &queue, OUT SMLK_UINT8 &result);
    virtual SMLK_BOOL ResultEncode(IN LoanCtrlResult &result);
    virtual void CheckLoanKeys(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT SMLK_UINT8 &result);
    virtual void DoLoanKeys(IN LoanCmdMsg &queue, std::vector<SMLK_UINT16> &key, OUT SMLK_UINT8 &result);

private:
    std::atomic<SMLK_BOOL> m_thread_work_flag;
};
}
}


#endif // _SMLK_LOAN_ECU_IMPL_H_
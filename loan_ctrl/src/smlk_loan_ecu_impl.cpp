/*****************************************************************************/
/**
* \file       smlk_loan_ecu_impl.cpp
* \author     wukai
* \date       2021/08/23
* \version    Tbox2.0 V1
* \brief      提供esync 调用接口
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan vehicle impl
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#include "smlk_loan_ecu_impl.h"
#include "smlk_spi_manager.h"
#include <dlfcn.h>
#include <mutex>
using namespace smartlink::CAN;
using namespace std;
typedef unsigned char byte;
namespace smartlink
{
namespace LOAN
{

static void get_bcd_timestamp(SMLK_BCD *buffer) {
    struct tm   tt{};
    time_t now = time(0);
    localtime_r(&now, &tt);

    tt.tm_year  -= 100;
    tt.tm_mon   += 1;

    buffer[0]   = (((SMLK_UINT8)(tt.tm_year / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_year % 10)) & 0x0F);
    buffer[1]   = (((SMLK_UINT8)(tt.tm_mon  / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_mon  % 10)) & 0x0F);
    buffer[2]   = (((SMLK_UINT8)(tt.tm_mday / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_mday % 10)) & 0x0F);
    buffer[3]   = (((SMLK_UINT8)(tt.tm_hour / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_hour % 10)) & 0x0F);
    buffer[4]   = (((SMLK_UINT8)(tt.tm_min  / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_min  % 10)) & 0x0F);
    buffer[5]   = (((SMLK_UINT8)(tt.tm_sec  / 10) & 0x0F) << 4) | (((SMLK_UINT8)(tt.tm_sec  % 10)) & 0x0F);
}

SMLK_RC string_to_hex(std::string string, SMLK_UINT8 *cbuf, SMLK_UINT8 buflen)
{
    SMLK_UINT8 high, low;
    std::size_t idx;
    int ii=0;
    SMLK_UINT8 *pstr = (SMLK_UINT8 *)(string.c_str());
    if((pstr[0]=='0')&&(pstr[1]=='x')){
        ;
    }else{
        return SMLK_RC::RC_OK;
    }
    for (idx=2; idx<string.size(); idx+=2)
    {
        high = pstr[idx];
        low = pstr[idx+1];

        if(high>='0' && high<='9')
            high = high-'0';
        else if(high>='A' && high<='F')
            high = high - 'A' + 10;
        else if(high>='a' && high<='f')
            high = high - 'a' + 10;
        else
            return SMLK_RC::RC_ERROR;

        if(low>='0' && low<='9')
            low = low-'0';
        else if(low>='A' && low<='F')
            low = low - 'A' + 10;
        else if(low>='a' && low<='f')
            low = low - 'a' + 10;
        else
            return SMLK_RC::RC_ERROR;
        if(ii + 1 > buflen)
        {
            break;
        }
        cbuf[ii++] = high<<4 | low;
    }
    return SMLK_RC::RC_OK;
}

SMLK_BOOL SmlkLoanEcuImpl::Init()
{
    return SmlkUds::getInstance()->Init(ModuleID::E_Module_loan_service, SmlkUdsMode::UDS_DIAG_MODE_LOAN);
}

SMLK_BOOL SmlkLoanEcuImpl::SendExterndMode()
{
    std::vector<SMLK_UINT8> vec_ = {0x03};
    SMLK_UINT16 reqdid = 0x0000;
    if (SendCmd(vec_, 0x10, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_[0] == 0x50)
        {
            return SMLK_TRUE;
        }
    }
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::GetSeedRequest(std::vector<SMLK_UINT8>& vec)
{
    std::vector<SMLK_UINT8> vec_ = {0x01};
    SMLK_UINT16 reqdid = 0x0000;
    if (SendCmd(vec_, 0x27, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_.size() < 2)
        {
            return SMLK_FALSE;
        }
        if ((vec_[0] == 0x67) && (vec_[1] == 0x01))
        {
            // get seed key /*请求种子*/
            vec.clear();
            vec.insert(vec.end(), vec_.begin() + 2, vec_.end());
            return SMLK_TRUE;
        }
    }
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::GetYunNeiSeedRequest(std::vector<SMLK_UINT8>& vec)
{
    std::vector<SMLK_UINT8> vec_ = {0x17};
    SMLK_UINT16 reqdid = 0x0000;
    if (SendCmd(vec_, 0x27, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_.size() < 2)
        {
            return SMLK_FALSE;
        }
        if ((vec_[0] == 0x67) && (vec_[1] == 0x17))
        {
            // get seed key /*请求种子*/
            vec.clear();
            vec.insert(vec.end(), vec_.begin() + 2, vec_.end());
            return SMLK_TRUE;
        }
    }
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::EncryptSeedKey(std::vector<SMLK_UINT8>& vec /*IN OUT*/)
{
    SMLK_LOGI("Encrypt libpath -> [%s]", m_encrypt_lib_path.c_str());
    SMLK_LOGI("Encrypt funName -> [%s]", m_encrypt_func_name.c_str());

    SMLK_LOGI(" vec.size() = %d ", vec.size());

    if(m_keylen == 8)
        SMLK_LOGI(" seed : %02X %02X %02X %02X %02X %02X %02X %02X", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5], vec[6], vec[7]);
    else
        SMLK_LOGI(" seed : %02X %02X %02X %02X", vec[0], vec[1], vec[2], vec[3]);

    if (m_encrypt_lib_path.empty()) return SMLK_FALSE;
    void* dl_handle = dlopen(m_encrypt_lib_path.c_str(), RTLD_NOW);
    SMLK_LOGD("Load Gaslib success!\r\n");
    if(dl_handle == NULL)
    {
        char *error = NULL;
        error = dlerror();
        char tp[1024] = {0};
        snprintf(tp, 256, "Load module \"%s\" error: %s\n", m_encrypt_lib_path.c_str(), error);
        SMLK_LOGW("%s",tp);
        return SMLK_FALSE;
    }
    SMLK_LOGD("Load module[%s] success!\r\n", m_encrypt_lib_path.c_str());
    static SMLK_UINT8 key[16] = {0};
    SMLK_UINT16 key_len = m_keylen;

    unsigned int (*myseedToKey) (byte *seed, unsigned int length, byte *key,unsigned int *retLen);
    myseedToKey = (unsigned int (*)(byte *seed, unsigned int length, byte *key,unsigned int *retLen))dlsym(dl_handle, m_encrypt_func_name.c_str());
    myseedToKey(vec.data(), vec.size(), key, (unsigned int *)&key_len);
    if(m_keylen == 8)
        SMLK_LOGI(" key :%02X %02X %02X %02X %02X %02X %02X %02X, key_len:%d ", key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],key_len);
    else
        SMLK_LOGI(" key :%02X %02X %02X %02X, key_len:%d ", key[0], key[1], key[2], key[3],key_len);

    if(dl_handle) {
        dlclose(dl_handle);
        dl_handle = NULL;
    }
    vec.clear();
    vec.insert(vec.end(), key, key + key_len);
    SMLK_LOGI(" vec.size() = %d ", vec.size());
    if(key_len == 8)
        SMLK_LOGI(" vec :%02X %02X %02X %02X %02X %02X %02X %02X", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5], vec[6], vec[7]);
    else
        SMLK_LOGI(" vec :%02X %02X %02X %02X", vec[0], vec[1], vec[2], vec[3]);
    return SMLK_TRUE;
}

SMLK_BOOL SmlkLoanEcuImpl::UploadKey(IN std::vector<SMLK_UINT8>& key)
{
    std::vector<SMLK_UINT8> vec_ = {0x02};

    vec_.insert(vec_.end(), key.begin(), key.end());
    SMLK_UINT16 reqdid = 0x0000;
    if (SendCmd(vec_, 0x27, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_[0] == 0x67)
        {
            SMLK_LOGI("Loan UploadKey success!!! 0x%02x",vec_[1]);
            return SMLK_TRUE;
        }
    }
    SMLK_LOGE("Loan UploadKey failed!!!");
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::UploadYunNeiKey(IN std::vector<SMLK_UINT8>& key)
{
    std::vector<SMLK_UINT8> vec_ = {0x18};

    vec_.insert(vec_.end(), key.begin(), key.end());
    SMLK_UINT16 reqdid = 0x0000;
    if (SendCmd(vec_, 0x27, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_[0] == 0x67)
        {
            SMLK_LOGI("Loan UploadYunNeiKey success!!! 0x%02x",vec_[1]);
            return SMLK_TRUE;
        }
    }
    SMLK_LOGE("Loan UploadYunNeiKey failed!!!");
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::WriteTboxId(SMLK_UINT8 action)
{
    std::vector<SMLK_UINT8> vec_ = {0x01};
    auto it_did = m_did_map.find("TboxIdDiD");
    if(LOAN_ACTION_FLOCK_INACTIVE == action)
    {
        it_did = m_did_map.find("TboxIdDelDiD");
    }
    if (it_did == m_did_map.end())
    {
        SMLK_LOGE("Can not find TboxIdDiD!!!");
        return SMLK_FALSE;
    }
    SMLK_UINT16 w_did = htobe16(it_did->second.m_did);
    SMLK_LOGI("Get Loan TboxIdDiD --> [0x%04x]", w_did);

    vec_.insert(vec_.end(), (SMLK_UINT8*)&w_did, (SMLK_UINT8*)&w_did + sizeof(SMLK_UINT16));

    SMLK_UINT16 reqdid = 0x0102;      //to be commit

    if (SendCmd(vec_, 0x31, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_[0] == 0x71)
        {
            SMLK_LOGI("WriteTboxId success!!!");
            return SMLK_TRUE;
        }
    }
    SMLK_LOGE("WriteTboxId failed!!!");
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::ReadTboxId(IN std::vector<SMLK_UINT8>& vec)
{
    std::vector<SMLK_UINT8> vec_ = {};
    auto it_did = m_did_map.find("ReadTboxIdDiD");
    if (it_did == m_did_map.end())
    {
        SMLK_LOGE("Can not find ReadTboxIdDiD!!!");
        return SMLK_FALSE;
    }
    SMLK_UINT16 r_did = htobe16(it_did->second.m_did);
    vec_.insert(vec_.end(), (SMLK_UINT8*)&r_did, (SMLK_UINT8*)&r_did + sizeof(SMLK_UINT16));
    SMLK_UINT16 reqdid = it_did->second.m_did;

    if (SendCmd(vec_, 0x22, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_[0] == 0x62)
        {
            int v_size = vec.size();
            for(int i=0;i<v_size;i++)
            {
                SMLK_LOGD("i=(%d), gpsid[%d]=0x%02x, gpsid[%d]=0x%02x",i, i, vec[i], i+3, vec_[i+3]);
                if(vec[i] != vec_[i+3])
                {
                    return SMLK_FALSE;
                }
            }
            return SMLK_TRUE;
        }
    }
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::WriteSpeedLimit(SMLK_UINT8 action)
{
    /*先刷写消贷标志位，再写一次0x00*/
    std::vector<SMLK_UINT8> vec_ = {};
    auto it_did = m_did_map.find("LoanWriteDiD");
    if (it_did == m_did_map.end())
    {
        SMLK_LOGE("Can not find LoanWriteDiD!!!");
        return SMLK_FALSE;
    }
    SMLK_UINT16 w_did = htobe16(it_did->second.m_did);
    SMLK_LOGI("Get Loan write did --> [0x%04x]", w_did);
    if(LOAN_ACTION_FLOCK_ACTIVE == action)
    {
        it_did = m_did_map.find("LoanActive");
    }
    else if(LOAN_ACTION_FLOCK_INACTIVE == action)
    {
        it_did = m_did_map.find("LoanInActive");
    }

    if (it_did == m_did_map.end())
    {
        SMLK_LOGW("action [%d] Can not find active or inactive data!!!", action);
        return SMLK_FALSE;
    }

    vec_.insert(vec_.end(), (SMLK_UINT8*)&w_did, (SMLK_UINT8*)&w_did + sizeof(SMLK_UINT16));
    vec_.insert(vec_.end(), it_did->second.m_data.begin(), it_did->second.m_data.end());
    SMLK_UINT16 reqdid = it_did->second.m_did;
    if (SendCmd(vec_, m_write_svc_id, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_[0] == (m_write_svc_id + 0x40))
        {
            SMLK_LOGI("Write SpeedLimit flag success!!!");
            return SMLK_TRUE;
        }
    }
    SMLK_LOGI("Write SpeedLimit flag failed!!!");
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::ReadSpeedLimit(SMLK_UINT8 action)
{
    std::vector<SMLK_UINT8> vec_ = {};
    auto it_did = m_did_map.find("LoanReadDiD");
    if (it_did == m_did_map.end())
    {
        SMLK_LOGE("Can not find LoanReadDiD!!!");
        return SMLK_FALSE;
    }
    SMLK_UINT16 r_did = htobe16(it_did->second.m_did);
    vec_.insert(vec_.end(), (SMLK_UINT8*)&r_did, (SMLK_UINT8*)&r_did + sizeof(SMLK_UINT16));
    SMLK_UINT16 reqdid = it_did->second.m_did;

    if(LOAN_ACTION_FLOCK_ACTIVE == action)
    {
        it_did = m_did_map.find("LoanActive");
    }
    else if(LOAN_ACTION_FLOCK_INACTIVE == action)
    {
        it_did = m_did_map.find("LoanInActive");
    }

    if (SendCmd(vec_, m_read_svc_id, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_.size() < 2) return SMLK_FALSE;
        if (vec_[0] == (m_read_svc_id + 0x40))
        {
            SMLK_UINT8 flag = *(--vec_.end());
            SMLK_LOGI("Get SpeedLimit flag ---> [%d]", flag);
            if((LOAN_ACTION_FLOCK_ACTIVE == action && flag == 1) 
             || (LOAN_ACTION_FLOCK_INACTIVE == action && flag == 0))
            {
                return SMLK_TRUE;
            }
        }
    }
    SMLK_LOGI("Get SpeedLimit flag failed!!!");
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::WriteLoanFlag(SMLK_UINT8 action)
{
    /*先刷写消贷标志位，再写一次0x00*/
    std::vector<SMLK_UINT8> vec_ = {};
    auto it_did = m_did_map.find("LoanWriteDiD");
    if (it_did == m_did_map.end())
    {
        SMLK_LOGE("Can not find LoanWriteDiD!!!");
        return SMLK_FALSE;
    }
    SMLK_UINT16 w_did = htobe16(it_did->second.m_did);
    SMLK_LOGI("Get Loan write did --> [0x%04x]", w_did);
    if(LOAN_ACTION_FLOCK_ACTIVE == action)
    {
        it_did = m_did_map.find("LoanActive");
    }
    else if(LOAN_ACTION_FLOCK_INACTIVE == action)
    {
        it_did = m_did_map.find("LoanInActive");
    }

    if (it_did == m_did_map.end())
    {
        SMLK_LOGW("action [%d] Can not find active or inactive data!!!", action);
        return SMLK_FALSE;
    }

    vec_.insert(vec_.end(), (SMLK_UINT8*)&w_did, (SMLK_UINT8*)&w_did + sizeof(SMLK_UINT16));
    vec_.insert(vec_.end(), it_did->second.m_data.begin(), it_did->second.m_data.end());
    SMLK_UINT16 reqdid = it_did->second.m_did;
    if (SendCmd(vec_, m_write_svc_id, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_[0] == (m_write_svc_id + 0x40))
        {
            SMLK_LOGI("Write Loan flag success!!!");
            return SMLK_TRUE;
        }
    }
    SMLK_LOGI("Write Loan flag failed!!!");
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::ReadLoanFlag(SMLK_UINT8* flag)
{
    std::vector<SMLK_UINT8> vec_ = {};
    auto it_did = m_did_map.find("LoanReadDiD");
    if (it_did == m_did_map.end())
    {
        SMLK_LOGE("Can not find LoanReadDiD!!!");
        return SMLK_FALSE;
    }
    SMLK_UINT16 r_did = htobe16(it_did->second.m_did);
    vec_.insert(vec_.end(), (SMLK_UINT8*)&r_did, (SMLK_UINT8*)&r_did + sizeof(SMLK_UINT16));
    SMLK_UINT16 reqdid = it_did->second.m_did;
    if (SendCmd(vec_, m_read_svc_id, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_.size() < 2) return SMLK_FALSE;
        if (vec_[0] == (m_read_svc_id + 0x40))
        {
            *flag = *(--vec_.end());
            SMLK_LOGI("Get Loan flag ---> [%d]", *flag);
            return SMLK_TRUE;
        }
    }
    SMLK_LOGI("Get Loan flag failed!!!");
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::SendMcuCmd(LoanSetMcuCmd &mcu_cmd)
{
    std::vector<SMLK_UINT8> data_to_mcu;
    LoanMcuCmdHead mcu_head;

    m_mcu_seq++;
    mcu_head.seq = htobe16(m_mcu_seq);
    mcu_head.count = 1;
    data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_head, (SMLK_UINT8 *)&mcu_head + sizeof(LoanMcuCmdHead));
    SMLK_UINT8 data;
    mcu_cmd.cmd_id = htobe16(mcu_cmd.cmd_id);
    
    if (mcu_cmd.data_len == 0)
        data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_cmd, (SMLK_UINT8 *)&mcu_cmd + sizeof(LoanCtrlMcuCmd));
    else if (mcu_cmd.data_len == 1)
    {
        data = (SMLK_UINT8)mcu_cmd.data;
         SMLK_LOGD(" SendMcuCmd data(%d)",data);
        data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_cmd, (SMLK_UINT8 *)&mcu_cmd + sizeof(LoanCtrlMcuCmd));
        data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8*)&data, (SMLK_UINT8*)&data + sizeof(SMLK_UINT8));
    }
    else
        data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_cmd, (SMLK_UINT8 *)&mcu_cmd + sizeof(LoanSetMcuCmd));
    smartlink_sdk::MCU::GetInstance()->SendVehicleCtrlCmd(data_to_mcu);
    unique_lock<std::mutex> lck(m_cv_mtx);
    SMLK_LOGD("wait for %ds",DEFALUT_TIMEOUT);
    if(m_cv.wait_for(lck, std::chrono::seconds(DEFALUT_TIMEOUT)) == std::cv_status::timeout){
        SMLK_LOGW("timeout!");
        return SMLK_FALSE;
    } else {
        SMLK_LOGD("No timeout.");
        if (m_loan_cmd_result == LOAN_CMD_RESULT_OK)
            return SMLK_TRUE;
        else
            return SMLK_FALSE;
    }
}

SMLK_BOOL SmlkLoanEcuImpl::SendMcuCmd(LoanSetMcuCmd &mcu_cmd, std::vector<SMLK_UINT8> &i_key)
{
    std::vector<SMLK_UINT8> data_to_mcu;
    LoanMcuCmdHead mcu_head;
    m_mcu_seq++;
    mcu_head.seq = htobe16(m_mcu_seq);
    mcu_head.count = 1;
    data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_head, (SMLK_UINT8 *)&mcu_head + sizeof(LoanMcuCmdHead));
    mcu_cmd.cmd_id = htobe16(mcu_cmd.cmd_id);
    data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_cmd, (SMLK_UINT8 *)&mcu_cmd + sizeof(LoanCtrlMcuCmd));
    data_to_mcu.insert(data_to_mcu.end(), i_key.begin(), i_key.end());
    smartlink_sdk::MCU::GetInstance()->SendVehicleCtrlCmd(data_to_mcu);
    unique_lock<std::mutex> lck(m_cv_mtx);
    SMLK_LOGD("wait for %ds",DEFALUT_TIMEOUT);
    if(m_cv.wait_for(lck, std::chrono::seconds(DEFALUT_TIMEOUT)) == std::cv_status::timeout){
        SMLK_LOGW("timeout!");
        return SMLK_FALSE;
    } else {
        SMLK_LOGD("No timeout");
        if (m_loan_cmd_result == LOAN_CMD_RESULT_OK)
            return SMLK_TRUE;
        else
            return SMLK_FALSE;
    }
}

SMLK_BOOL SmlkLoanEcuImpl::SendCmd(std::vector<SMLK_UINT8>& vec, SMLK_UINT16 svc_id, SMLK_UINT32 phy_addr, SMLK_UINT32 resp_addr, SMLK_UINT16 reqdid)
{
    EnterUdsMode();
    UdsResp u_req;
    u_req.resp_addr = resp_addr;
    u_req.len = vec.size();
    u_req.data = vec.data();
    SMLK_UINT8 canid = 0;
    if(m_isYunnei)
        canid = 0;
    else
        canid = 1;
    if (!SmlkUds::getInstance()->WriteUdsMsg(phy_addr, svc_id, &u_req, reqdid, canid))
    {
        SMLK_LOGW("Send uds msg failed!!!\n");
        return SMLK_FALSE;
    }
    UdsResp u_resp;
    SMLK_UINT8 buf[1024] = {0};
    u_resp.data = buf;

    while (SmlkUds::getInstance()->ReadUdsMsg(&u_resp, LOAN_WAIT_UDS_INTERVAL))
    {
        SMLK_LOGW("ReadUdsMsg uds msg  0x%02x",u_resp.data[0]);
        if (u_resp.data[0] != svc_id + 0x40)
        {
            SMLK_LOGI("rcv service != send service_id+0x40");
            continue;
        }
        SMLK_LOGI("rcv service == send service_id+0x40");
        vec.clear();
        vec.insert(vec.end(), u_resp.data, u_resp.data + u_resp.len);
        return SMLK_TRUE;
    }

    SMLK_LOGW("ReadUdsMsg uds msg fail");
    return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::EnterUdsMode()
{
    if (!SmlkUds::getInstance()->EnterUdsMode())
    {
        SMLK_LOGE("Enter UDS_DIAG_MODE_LOAN mode failed!!!\n");
        return SMLK_FALSE;
    }
    return SMLK_TRUE;
}

SMLK_BOOL SmlkLoanEcuImpl::ExitUdsMode()
{
    return SmlkUds::getInstance()->ExitUdsMode();
}

void SmlkLoanEcuImpl::GetLckCarStatusAndSendStatus(SMLK_UINT8 flag)
{
    SMLK_LOGD("enter in func: %s",__func__);

    LoanLckSatatusInfo xd_status;
    bzero(&xd_status, sizeof(LoanLckSatatusInfo));

    for(SMLK_UINT8 i = 0; i < VIN_LENGTH; ++i){
        xd_status.vin[i] = '0';
    }
    for(SMLK_UINT8 i = 0; i < EIN_LENGTH; ++i){
        xd_status.ein[i] = '0';
    }
    /*获取消贷锁车状态相关信息:VIN,EIN,消贷标志位，tbox4G天线状态等*/
    GetLckStatusInfo(xd_status,flag);
    /*发送消贷锁车状态*/
    SendLckCarStatus(xd_status);
}

void SmlkLoanEcuImpl::GetLckStatusInfo(OUT LoanLckSatatusInfo &status_info, SMLK_UINT8 flag)
{
    SMLK_LOGD("enter in func (SmlkLoanEcuImpl::%s) \n",__func__);
    smartlink_sdk::LocationInfo location;
    GetVinEinFlagResult result = GetVinEinFlagResult::GET_ECU_INFO_OK;

    auto rc = smartlink_sdk::Location::GetInstance()->GetLocation(location);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != rc ) {
        SMLK_LOGE("[GNSS] fail to get location information, rc: %d\n", (int32_t)rc);
    } else {
        status_info.latitude            = location.latitude * 1000000;
        status_info.longitude           = location.longitude * 1000000;
        status_info.elevation           = location.altitude;
        status_info.direction           = location.heading;
        status_info.speed               = location.speed;

        SMLK_LOGD("[GNSS]   status_info.latitude:   %f\n", location.latitude);
        SMLK_LOGD("[GNSS]   status_info.longitude:  %f\n", location.longitude);
        SMLK_LOGD("[GNSS]   status_info.elevation:  %f\n", location.altitude);
        SMLK_LOGD("[GNSS]   status_info.direction:  %f\n", location.heading);
        SMLK_LOGD("[GNSS]   status_info.speed:      %f\n", location.speed);
    }

    get_bcd_timestamp(status_info.time);

    status_info.lck_active_status = LCK_STATUS_INACTIVE;
    status_info.tbox4G_status = 0;

    /*获取GPS天线状态*/
    smartlink_sdk::GnssAntennaState gps_ant_status;
    smartlink_sdk::RtnCode code = smartlink_sdk::Location::GetInstance()->GetGnssAntennaState(gps_ant_status);
    if(smartlink_sdk::RtnCode::E_SUCCESS == code){

        SMLK_LOGD("[GnssAntennaState]  :  %d.\n", (SMLK_UINT8)gps_ant_status);
        if(smartlink_sdk::GnssAntennaState::E_GNSS_ANTENNA_STATE_PRESENT != gps_ant_status){
            status_info.tbox4G_status |= 0X10;/*bit4为1*/
            SMLK_LOGD("gps -------break---- ");
        }
    }
    /*4G天线状态*/
    smartlink_sdk::RtnCode code_tel = smartlink_sdk::RtnCode::E_ERR;
    smartlink_sdk::ModemAntennaInfo master_antenna_info;
    master_antenna_info.type = smartlink_sdk::ModemAntennaType::E_MODEM_ANTENNA_TYPE_MASTER;
    code_tel = smartlink_sdk::Telephony::GetInstance()->GetModemAntennaInfo(master_antenna_info);
    if(smartlink_sdk::RtnCode::E_SUCCESS == code_tel){
        SMLK_LOGD("[ModemAntennaState_MASTER]  :  %d\n", (SMLK_UINT8)(master_antenna_info.state));
        if(smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_PRESENT != master_antenna_info.state){
            status_info.tbox4G_status |= 0X01;/*bit0为1*/
            SMLK_LOGD("MASTER -------break---- ");
        }
    }

    smartlink_sdk::ModemAntennaInfo slave_antenna_info;
    slave_antenna_info.type = smartlink_sdk::ModemAntennaType::E_MODEM_ANTENNA_TYPE_SLAVE;
    code_tel = smartlink_sdk::Telephony::GetInstance()->GetModemAntennaInfo(slave_antenna_info);
    if(smartlink_sdk::RtnCode::E_SUCCESS == code_tel){
        SMLK_LOGD("[ModemAntennaState_SLAVE]  :  %d\n", (SMLK_UINT8)(slave_antenna_info.state));

        if(smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_PRESENT != slave_antenna_info.state){
            status_info.tbox4G_status |= 0X04;/*bit2为1*/
            SMLK_LOGD("SLAVE -------break---- ");
        }
    }
    SMLK_LOGD( "*****************tbox4G_status = %d.\n", status_info.tbox4G_status);

    RtnCode property_rc = RtnCode::E_SUCCESS;
    if (flag == LOAN_ENTER_UDS_SUCCESS)
    {
        result = GetVinEinFlagFromEcu(status_info.vin,status_info.ein,status_info.lck_active_status);

    } else if (flag == LOAN_ENTER_UDS_ERROR)
    {
        result = GetVinEinFlagResult::GET_ECU_FAILED;
    }
    else
    {
        result = GetVinEinFlagResult::GET_ECU_INFO_OK;
    }

    string lock_result_name = SYS_PRO_NAME_LCKCAR_FINACAIL_LCK_STATUS;
    string lock_result;
    property_rc = SysProperty::GetInstance()->GetValue(lock_result_name, lock_result);
    if (RtnCode::E_SUCCESS == property_rc && stoi(lock_result) == (SMLK_UINT8)RctrlStatusFinaLck::STATUS_IS_ACTIVE)
    {
        SMLK_LOGD( "*******lck_active_status = a1**********");
        status_info.lck_active_status = LCK_STATUS_ACTIVE;
    }
    else if (RtnCode::E_SUCCESS == property_rc && stoi(lock_result) == (SMLK_UINT8)RctrlStatusFinaLck::STATUS_IS_LOCKED)
    {
        SMLK_LOGD( "*******lck_active_status = a2**********");
        status_info.lck_active_status = LCK_STATUS_ACTIVE_LOCK;
    }
    else
    {
        SMLK_LOGD( "*******lck_active_status = a0**********");
        status_info.lck_active_status = LCK_STATUS_INACTIVE;
    }

    /*获取ein，vin，消贷标志位出错，上报0x0F40，同时也要上报0x0f3d（里面的值填0）*/
    if(GetVinEinFlagResult::GET_ECU_INFO_OK != result){
        Pack0F40MsgQuery(result);
    }

    SetTsc1State();
}

void SmlkLoanEcuImpl::SetTsc1State()
{
    string value_str;
    string value_name = SYS_PRO_NAME_4G_ANTENNA_LOCK;
    SysProperty::GetInstance()->GetValue(value_name ,value_str);
    SMLK_UINT8 value = atoi(value_str.c_str());

    /*4G天线状态*/
    SMLK_BOOL antenna_state = SMLK_FALSE;
    smartlink_sdk::RtnCode code_tel = smartlink_sdk::RtnCode::E_ERR;
    smartlink_sdk::ModemAntennaInfo master_antenna_info;

    master_antenna_info.type = smartlink_sdk::ModemAntennaType::E_MODEM_ANTENNA_TYPE_MASTER;
    code_tel = smartlink_sdk::Telephony::GetInstance()->GetModemAntennaInfo(master_antenna_info);
    if(smartlink_sdk::RtnCode::E_SUCCESS == code_tel){
        SMLK_LOGD("[ModemAntennaState_MASTER]  :  %d\n",(SMLK_UINT8)(master_antenna_info.state));
        if(smartlink_sdk::ModemAntennaState::E_MODEM_ANTENNA_STATE_PRESENT == master_antenna_info.state){
            antenna_state = SMLK_TRUE;
        }
    }

    if (value == (SMLK_UINT8)RCTRL_ANTENNA::ANTENNA_4G_UNLOCK)
    {
        SMLK_LOGI("ANTENNA_4G_UNLOCK to set sys_param_name(%s), value 1.\n", SYS_PRO_NAME_TSC1_VALID);
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_TSC1_VALID, "1");
    }
    else if ((value == (SMLK_UINT8)RCTRL_ANTENNA::ANTENNA_4G_LOCK) && antenna_state)
    {
        SMLK_LOGI("ANTENNA_4G_LOCK to set sys_param_name(%s), value 1.\n", SYS_PRO_NAME_TSC1_VALID);
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_TSC1_VALID, "1");
    }
    else
    {
        SMLK_LOGI("to set sys_param_name(%s), value 0.\n", SYS_PRO_NAME_TSC1_VALID);
        SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_TSC1_VALID, "0");
    }
}

void SmlkLoanEcuImpl::Send4GantennaState()
{
    SMLK_LOGD("enter in func: %s",__func__);
    LoanSetMcuCmd mcu_cmd;
    SMLK_UINT16 rctrl_cmd_result;
    mcu_cmd.cmd_id = LOAN_CMD_ANTENNA_LOCK;

    string value_str;
    string value_name = SYS_PRO_NAME_4G_ANTENNA_LOCK;
    SysProperty::GetInstance()->GetValue(value_name ,value_str);
    SMLK_UINT8 value = atoi(value_str.c_str());

    if (value == 1)
    {
        mcu_cmd.action = LOAN_MCU_ACTION_ON;
    }
    else
    {
        mcu_cmd.action = LOAN_MCU_ACTION_OFF;
    }
    mcu_cmd.data_len = 0;
    SendMcuCmd(mcu_cmd);
    SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_LCKCAR_FLAG_ANTENNA, to_string((SMLK_UINT8)(LoanStatusIgoffLckFlag::NOT_NEED_LCKCAR_WHEN_IGNON)));
}

void SmlkLoanEcuImpl::SendLckCarStatus(LoanLckSatatusInfo& xd_status)
{
    SMLK_LOGD( "tbox4G_status = %d.\n", xd_status.tbox4G_status);
    SMLK_LOGD( "lck_active_status = %d.\n", xd_status.lck_active_status);
    LoanGeneralHead head;
    vector<SMLK_UINT8> output_vec;
    Msg0f3dEventCommon event_com;

    bzero(&head, sizeof(LoanGeneralHead));
    bzero(&event_com, MSGOF3D_EVENT_COMMON_LEN);

    event_com.event_location.start_latitude = htobe32(xd_status.latitude);
    event_com.event_location.start_longitude = htobe32(xd_status.longitude);
    event_com.event_location.start_elevation = htobe16(xd_status.elevation);
    event_com.event_location.start_direction = htobe16(xd_status.direction);
    memcpy(event_com.event_location.start_time , xd_status.time, sizeof(event_com.event_location.start_time));
    event_com.event_location.start_speed = xd_status.speed;

    // 单点事件只有起始点，结束点填充0
    event_com.event_location.end_latitude = 0;
    event_com.event_location.end_longitude = 0;
    event_com.event_location.end_elevation = 0;
    event_com.event_location.end_direction = 0;
    memset(event_com.event_location.end_time , 0, sizeof(event_com.event_location.end_time));
    event_com.event_location.end_speed = 0;

    event_com.event_lenth = htobe16(LCK_STATUS_EVENT_LEN);

    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&event_com, (SMLK_UINT8 *)&event_com + MSGOF3D_EVENT_COMMON_LEN);

    LckStatusEvent event;
    bzero(&event, LCK_STATUS_EVENT_LEN);
    event.event_id = LCK_STATUS_EVENT_ID;
    memcpy(event.vin , xd_status.vin, sizeof(event.vin));
    memcpy(event.ein , xd_status.ein, sizeof(event.ein));
    if(LCK_STATUS_ACTIVE == xd_status.lck_active_status){
        event.lck_flag = LCK_FLAG_ACTIVE_SUCC;
    }else if(LCK_STATUS_INACTIVE == xd_status.lck_active_status){
        event.lck_flag = LCK_FLAG_ACTIVE_FAILED;
    }else if(LCK_STATUS_ACTIVE_LOCK == xd_status.lck_active_status){
        if(m_isYuchai || m_isWeichai || m_isYunnei || m_isCummins || m_isQQvcu)
            event.lck_flag = LCK_FLAG_ACTIVE_LOCK_SUCC;
        else
            event.lck_flag = LCK_FLAG_ACTIVE_SUCC;
    }

    event.tbox4G = xd_status.tbox4G_status;
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&event, (SMLK_UINT8 *)&event + LCK_STATUS_EVENT_LEN);

    head.msg_id = JTT808_DRIVE_EVENT;
    head.protocol = ProtoclID::E_PROT_JTT808;
    head.qos = QOS_SEND_TCP_TIMES;
    DoSendMsgToTsp(head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());

    return;
}

GetVinEinFlagResult SmlkLoanEcuImpl::GetVinEinFlagFromEcu(OUT SMLK_UINT8 *vin, OUT SMLK_UINT8 *ein, OUT SMLK_UINT8 lck_active_status)
{
    SMLK_LOGD("enter in func: %s",__func__);
    GetVinEinFlagResult result = GetVinEinFlagResult::GET_ECU_INFO_OK;
    SMLK_BOOL f_vin = SMLK_TRUE;
    SMLK_BOOL f_ein = SMLK_TRUE;
    SMLK_BOOL f_flag = SMLK_TRUE;
    if (!GetVinorEin(VinId::LOAN_VIN, vin))
        f_vin = SMLK_FALSE;
    if (!m_isHCU && !GetVinorEin(VinId::LOAN_EIN, ein))
        f_ein = SMLK_FALSE;
    if (!ReadLoanFlag(&lck_active_status))
        f_flag = SMLK_FALSE;

    if(!f_vin && f_ein && f_flag)
        result = GetVinEinFlagResult::GET_ECU_VIN_FAILED;
    else if(f_vin && !f_ein && f_flag)
        result = GetVinEinFlagResult::GET_ECU_EIN_FAILED;
    else if(f_vin && f_ein && !f_flag)
        result = GetVinEinFlagResult::GET_ECU_LAON_FLAG_FAILED;
    else if(!f_vin && !f_ein && f_flag)
        result = GetVinEinFlagResult::GET_ECU_VIN_EIN_FAILED;
    else if(!f_vin && f_ein && !f_flag)
        result = GetVinEinFlagResult::GET_ECU_VIN_LAONFLAG_FAILED;
    else if(f_vin && !f_ein && !f_flag)
        result = GetVinEinFlagResult::GET_ECU_EIN_LAONFLAG_FAILED;
    else if(!f_vin && !f_ein && !f_flag)
        result = GetVinEinFlagResult::GET_ECU_FAILED;

    SMLK_LOGI("GetVinEinFlagFromEcu --> [%d]", result);
    return result;
}

SMLK_BOOL SmlkLoanEcuImpl::GetVinorEin(VinId id, SMLK_UINT8 *val)
{
    if (id == VinId::LOAN_VIN)
    {
        std::vector<SMLK_UINT8> vec;
        auto it = m_did_map.find(VinDidStr);
        SMLK_UINT16 reqdid = 0x0000;
        if (it != m_did_map.end())
        {
            SMLK_UINT16 vin_did = htobe16(it->second.m_did);
            reqdid = it->second.m_did;
            vec.insert(vec.end(), (SMLK_UINT8*)&vin_did, (SMLK_UINT8*)&vin_did + sizeof(SMLK_UINT16));
            SMLK_LOGI("Get Loan write did --> [0x%04x]", vin_did);
        }
        if (SendCmd(vec, m_read_svc_id, m_phy_eng_addr, m_resp_eng_id, reqdid))
        {
            if (vec[0] == (m_read_svc_id + 0x40))
            {
                for(int i = 0; i < VIN_LENGTH; i++)
                {
                    val[i] = vec[i+3];
                }
                // val.memcpy(val.end(), vec.begin() + 3, vec.end());
                SMLK_LOGD("GetVinorEin --- vin");
                return SMLK_TRUE;
            }
        }
        return SMLK_FALSE;
    }
    if (id == VinId::LOAN_EIN)
    {
        std::vector<SMLK_UINT8> vec;
        auto it = m_did_map.find(EinDidStr);
        SMLK_UINT16 reqdid = 0x0000;
        if (it != m_did_map.end())
        {
            SMLK_UINT16 ein_did = htobe16(it->second.m_did);
            reqdid = it->second.m_did;
            vec.insert(vec.end(), (SMLK_UINT8*)&ein_did, (SMLK_UINT8*)&ein_did + sizeof(SMLK_UINT16));
            SMLK_LOGI("Get Loan write did --> [0x%04x]", ein_did);
        }
        if (SendCmd(vec, m_read_svc_id, m_phy_eng_addr, m_resp_eng_id, reqdid))
        {
            if (vec[0] == (m_read_svc_id + 0x40))
            {
                 for(int i = 0; i < EIN_LENGTH; i++)
                {
                    val[i] = vec[i+3];
                }
                SMLK_LOGD("GetVinorEin --- ein");
                return SMLK_TRUE;
            }
        }
    }
    return SMLK_FALSE;
}

//握手校验 -- 国六自主发动机
SMLK_RC SmlkLoanEcuImpl::FinacailCheckCode(std::vector<SMLK_UINT8>& vec)
{
    SMLK_LOGD("enter in func(%s).",__func__);

    std::vector<SMLK_UINT8> vec_ = {0x02,0x9c};
    SMLK_UINT16 reqdid = 0x029c;
    if (SendCmd(vec_, 0x22, m_phy_addr, m_resp_id, reqdid))
    {
        if (vec_[0] == 0x62)
        {
            vec.clear();
            SMLK_LOGI(" vec_.size() = %d ",vec_.size());
            vec.insert(vec.end(), vec_.begin() + 3, vec_.end());
            if(vec_.size() == 5)
            {
                std::vector<SMLK_UINT8> vec1_ = {0x00,0x00};
                vec.insert(vec.end(), vec1_.begin(), vec1_.end());
            }
            SMLK_LOGI(" vec :%02X %02X %02X %02X", vec[0], vec[1], vec[2], vec[3]);
            SMLK_LOGI("Loan FinacailCheckCode success!!!");
            return SMLK_RC::RC_OK;
        }
    }
    SMLK_LOGE("Loan FinacailCheckCode failed!!!");
    return SMLK_RC::RC_ERROR;
}

SMLK_RC SmlkLoanEcuImpl::FinacailCheckCodeProcess(bool flag)
{
    SMLK_LOGD("enter in func(%s).",__func__);
    SMLK_RC rc = SMLK_RC::RC_OK;
    LoanCtrlMcuCmd mcu_cmd;
    mcu_cmd.cmd_id = LOAN_RSP_CHECK_CODE;
    mcu_cmd.action = LOAN_MCU_ACTION_ON;
    mcu_cmd.data_len = 4;
    std::vector<SMLK_UINT8> data_to_mcu;
    std::vector<SMLK_UINT8> m_checkcode = {};
    if (flag) {
        rc = FinacailCheckCode(m_checkcode);
        if(SMLK_RC::RC_OK != rc ){
            m_checkcode = {0,0,0,0};
        }
    } else {
        m_checkcode = {0,0,0,0};
    }
    SMLK_LOGI(" m_checkcode :%02X %02X %02X %02X", m_checkcode[0], m_checkcode[1], m_checkcode[2], m_checkcode[3]);
    LoanMcuCmdHead mcu_head;
    m_mcu_seq++;
    mcu_head.seq = htobe16(m_mcu_seq);
    mcu_head.count = 1;
    data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_head, (SMLK_UINT8 *)&mcu_head + sizeof(LoanMcuCmdHead));
    mcu_cmd.cmd_id = htobe16(mcu_cmd.cmd_id);
    data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_cmd, (SMLK_UINT8 *)&mcu_cmd + sizeof(LoanCtrlMcuCmd));

    data_to_mcu.insert(data_to_mcu.end(), m_checkcode.begin(), m_checkcode.end());

    smartlink_sdk::MCU::GetInstance()->SendVehicleCtrlCmd(data_to_mcu);
    unique_lock<std::mutex> lck(m_cv_mtx);
    SMLK_LOGD("ready to wait for %ds\n",DEFALUT_TIMEOUT);
    if((m_cv.wait_for(lck, std::chrono::seconds(DEFALUT_TIMEOUT)) == std::cv_status::timeout) || (SMLK_RC::RC_OK != rc))
    {
        SMLK_LOGD("timeout");
        return SMLK_RC::RC_ERROR;
    }
    else
    {
        SMLK_LOGD("no timeout");
        return SMLK_RC::RC_OK;
    }
}

SMLK_RC SmlkLoanEcuImpl::SendMsg0F40(IN LoanGeneralHead &head)
{
    SMLK_LOGD("enter in func(%s).\n", __func__);
    RtnCode property_rc = RtnCode::E_SUCCESS;
    vector<SMLK_UINT8> output_vec;

    Msg0f40Query query;
    bzero(&query, sizeof(Msg0f40Query));

    query.seq_id = htobe16(head.seq_id);
    query.resp_cmd = JTT808_QUERY_LOCK_STATUS;

    string gpid_name = SYS_PRO_NAME_LCKCAR_GPSID_JTT808;
    string gpsid;
    property_rc = SysProperty::GetInstance()->GetValue(gpid_name, gpsid);
    if (RtnCode::E_SUCCESS != property_rc) {
        for (SMLK_UINT8 i = 0; i < GPSID_LEN; ++i) {
            query.gpsid[i] = 0xff;
        }
    } else {
        string_to_hex(gpsid, query.gpsid, GPSID_LEN);
    }

    string fixed_key_name = SYS_PRO_NAME_LCKCAR_FIXED_KEY_JTT808;
    string fixed_key;
    property_rc = SysProperty::GetInstance()->GetValue(fixed_key_name, fixed_key);
    if (RtnCode::E_SUCCESS != property_rc) {
        for (SMLK_UINT8 i = 0; i < FIXED_KEY_LEN; ++i) {
            query.key[i] = 0xff;
        }
    } else {
        string_to_hex(fixed_key, query.key, FIXED_KEY_LEN);
    }

    string lock_result_name = SYS_PRO_NAME_LCKCAR_RESULT_JTT808;
    string lock_result;
    property_rc = SysProperty::GetInstance()->GetValue(lock_result_name, lock_result);

    SMLK_LOGD("---------- lock_result (%s)", lock_result.c_str());

    if (m_isWeichai || m_isQQvcu)
        query.lock_prot = LCKCAR_PROT_TYPE_QINGQI_MD5;
    else if (m_isYuchai)
        query.lock_prot = LCKCAR_PROT_TYPE_QINGQI_YUCHAI;
    else if (m_isYunnei)
        query.lock_prot = LCKCAR_PROT_TYPE_YUNNEI;
    else
        query.lock_prot = LCKCAR_PROT_TYPE_YIQI_TSC1;

    string lock_action_name = SYS_PRO_NAME_LCKCAR_ACTION_JTT808;
    string lock_action;
    property_rc = SysProperty::GetInstance()->GetValue(lock_action_name, lock_action);
    SMLK_LOGD("---------- lock_action (%s)", lock_action.c_str());

    if (RtnCode::E_SUCCESS != property_rc)
    {
        query.result = Query0f40Result::STATUS_NOT_ACTIVE_LOCK;
    }
    else if (JTT808_QUERY_NULL == atoi(lock_action.c_str()))
    {
        query.result = Query0f40Result::STATUS_NOT_ACTIVE_LOCK;
    }
    else if (JTT808_ACTIVE_LOCK == atoi(lock_action.c_str()))
    {
        // 20211115 未激活状态下进行激活 : 激活成功前反馈 5A(未激活),激活成功后反馈 5D(激活成功)
        if (LOAN_CMD_ACTIVE_LOCK_SUCCESS != atoi(lock_result.c_str())) {
            query.result = Query0f40Result::STATUS_NOT_ACTIVE_LOCK;
        } else {
            query.result = Query0f40Result::STATUS_ACTIVE_LOCK_SUCCESS;
        }
    }
    else if (JTT808_INACTIVE_LOCK == atoi(lock_action.c_str()))
    {
        // 20211115 激活未锁车状态下进行失活 : 关闭成功前反馈 5D(激活成功),关闭成功后反馈 72(关闭锁车成功)
        if (LOAN_CMD_UNACTIVE_LOCK_SUCCESS == atoi(lock_result.c_str())) {
            query.result = Query0f40Result::STATUS_INACTIVE_LOCK_SUCCESS;
        } else {
            query.result = Query0f40Result::STATUS_ACTIVE_LOCK_SUCCESS;
        }
    }
    else if (JTT808_LOCK_VAHICLE == atoi(lock_action.c_str()))
    {
        // 20211115 激活状态下进行锁车 : 锁车成功前反馈 5D(激活成功),锁车成功后反馈 67(锁车成功)
        if (LOAN_CMD_LOCKCAR_SUCCESS != atoi(lock_result.c_str())) {
            query.result = Query0f40Result::STATUS_ACTIVE_LOCK_SUCCESS;
        } else {
            query.result = Query0f40Result::STATUS_LOCK_VAHICLE_SUCCESS;
        }
    }
    else if (JTT808_UNLOCK_VEHICLE == atoi(lock_action.c_str()))
    {
        // 20211115 激活锁车状态下进行解锁 : 解锁成功前反馈 67(锁车成功),解锁成功后反馈 5D(激活成功)
        if (LOAN_CMD_UNLOCKCAR_SUCCESS != atoi(lock_result.c_str()) && LOAN_CMD_ACTIVE_LOCK_SUCCESS != atoi(lock_result.c_str())) {
            query.result = Query0f40Result::STATUS_LOCK_VAHICLE_SUCCESS;
        } else {
            query.result = Query0f40Result::STATUS_ACTIVE_LOCK_SUCCESS;
        }
    }

    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&query, (SMLK_UINT8 *)&query + sizeof(Msg0f40Query));

    LoanGeneralHead temp_head;
    memcpy(&temp_head, &head, sizeof(LoanGeneralHead));
    temp_head.msg_id = JTT808_REMOTE_LOCK_RESP;
    temp_head.qos = QOS_SEND_TCP_TIMES;
    DoSendMsgToTsp(temp_head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());

    return SMLK_RC::RC_OK;
}

SMLK_RC SmlkLoanEcuImpl::DoSendMsgToTsp(IN LoanGeneralHead &rctrl_head, IN SMLK_UINT8 *msg_body, IN std::size_t &length)
{
    SMLK_LOGD("enter in func(%s)",__func__);
    IpcTspHead ipc_head;
    // GetIpcHeadFromRctrlHead(rctrl_head, ipc_head);
    ipc_head.msg_id = rctrl_head.msg_id;
    ipc_head.seq_id = rctrl_head.seq_id;
    ipc_head.protocol = (SMLK_UINT8)(rctrl_head.protocol);
    ipc_head.qos = rctrl_head.qos;
    ipc_head.priority = rctrl_head.priority;

    return TspServiceApi::getInstance()->SendMsg(ipc_head, msg_body, (std::size_t)length);
}

void SmlkLoanEcuImpl::SetResult(SMLK_UINT8 result)
{
    m_loan_cmd_result = result;
}

void SmlkLoanEcuImpl::Notify_m_cv()
{
    m_cv.notify_one();
}

void SmlkLoanEcuImpl::EncodeMcuReport(SMLK_UINT8 result)
{
    Pack0F40MsgQuery((GetVinEinFlagResult) result);
}

void SmlkLoanEcuImpl::Pack0F40MsgQuery(IN GetVinEinFlagResult result)
{
    SMLK_LOGI("enter in func(%s), %d. ",__func__,result);

    vector<SMLK_UINT8> output_vec;
    Msg0f40Query query;
    query.lock_prot = JTT808_PROT_YIQI_TSC1_LOCK;
    query.resp_cmd = JTT808_QUERY_LOCK_STATUS;
    query.seq_id = 0xffff;

    string gpid_name = SYS_PRO_NAME_LCKCAR_GPSID_JTT808;
    string gpsid;
    RtnCode property_rc = RtnCode::E_SUCCESS;
    property_rc = SysProperty::GetInstance()->GetValue(gpid_name, gpsid);
    if (RtnCode::E_SUCCESS != property_rc) {
        for (SMLK_UINT8 i = 0; i < GPSID_LEN; ++i) {
            query.gpsid[i] = 0xff;
        }
    } else {
        string_to_hex(gpsid, query.gpsid, GPSID_LEN);
    }

    string fixed_key_name = SYS_PRO_NAME_LCKCAR_FIXED_KEY_JTT808;
    string fixed_key;
    property_rc = SysProperty::GetInstance()->GetValue(fixed_key_name, fixed_key);
    if (RtnCode::E_SUCCESS != property_rc) {
        for (SMLK_UINT8 i = 0; i < FIXED_KEY_LEN; ++i) {
            query.key[i] = 0xff;
        }
    } else {
        string_to_hex(fixed_key, query.key, FIXED_KEY_LEN);
    }

    LoanGeneralHead head;
    bzero(&head, sizeof(LoanGeneralHead));
    head.msg_id = JTT808_REMOTE_LOCK_RESP;
    head.qos = QOS_SEND_TCP_TIMES;
    head.protocol = ProtoclID::E_PROT_JTT808;

    if(GetVinEinFlagResult::ECU_HANDSHAKE_FAILED == result){
        query.result = Query0f40Result::STATUS_HANDSHAKE_FAILED;
    }else if(GetVinEinFlagResult::GET_ECU_VIN_FAILED == result){
        query.result = Query0f40Result::STATUS_VIN_READ_FAILED;
    }else if(GetVinEinFlagResult::GET_ECU_EIN_FAILED == result){
        query.result = Query0f40Result::STATUS_EIN_READ_FAILED;
    }else if(GetVinEinFlagResult::GET_ECU_LAON_FLAG_FAILED == result){
        query.result = Query0f40Result::STATUS_FINACIAL_FLAG_READ_FAILED;
    }else if(GetVinEinFlagResult::GET_ECU_VIN_EIN_FAILED == result){
        query.result = Query0f40Result::STATUS_VIN_READ_FAILED;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&query, (SMLK_UINT8 *)&query + sizeof(Msg0f40Query));
        DoSendMsgToTsp(head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
        output_vec.clear();
        query.result = Query0f40Result::STATUS_EIN_READ_FAILED;
    }else if(GetVinEinFlagResult::GET_ECU_VIN_LAONFLAG_FAILED == result){
        query.result = Query0f40Result::STATUS_VIN_READ_FAILED;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&query, (SMLK_UINT8 *)&query + sizeof(Msg0f40Query));
        DoSendMsgToTsp(head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
        output_vec.clear();
        query.result = Query0f40Result::STATUS_FINACIAL_FLAG_READ_FAILED;
    }else if(GetVinEinFlagResult::GET_ECU_EIN_LAONFLAG_FAILED == result){
        query.result = Query0f40Result::STATUS_EIN_READ_FAILED;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&query, (SMLK_UINT8 *)&query + sizeof(Msg0f40Query));
        DoSendMsgToTsp(head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
        output_vec.clear();
        query.result = Query0f40Result::STATUS_FINACIAL_FLAG_READ_FAILED;
    }else if(GetVinEinFlagResult::GET_ECU_FAILED == result){
        query.result = Query0f40Result::STATUS_VIN_READ_FAILED;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&query, (SMLK_UINT8 *)&query + sizeof(Msg0f40Query));
        DoSendMsgToTsp(head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
        output_vec.clear();
        query.result = Query0f40Result::STATUS_EIN_READ_FAILED;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&query, (SMLK_UINT8 *)&query + sizeof(Msg0f40Query));
        DoSendMsgToTsp(head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
        output_vec.clear();
        query.result = Query0f40Result::STATUS_FINACIAL_FLAG_READ_FAILED;
    }else if(GetVinEinFlagResult::GET_ECU_INFO_OK == result){
        string lock_result_name = SYS_PRO_NAME_LCKCAR_RESULT_JTT808;
        string lock_result;
        SysProperty::GetInstance()->GetValue(lock_result_name ,lock_result);
        query.result = (Query0f40Result)atoi(lock_result.c_str());
    }
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&query, (SMLK_UINT8 *)&query + sizeof(Msg0f40Query));
    DoSendMsgToTsp(head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
}

SMLK_BOOL SmlkLoanEcuImpl::CheckIgnonTime()
{
    std::chrono::steady_clock::time_point cur_time;
    cur_time = std::chrono::steady_clock::now();
    if(cur_time < g_ign_on_time)
    {
        SMLK_UINT32 t = static_cast<SMLK_UINT32>(std::chrono::duration<SMLK_DOUBLE, std::milli>(g_ign_on_time - cur_time).count());
        SMLK_LOGD("[Loan] after sleep(%ld)ms, do loan cmd", t);
        usleep(t * 1000);
    }
    if(m_ign_status)
        return SMLK_TRUE;
    else
        return SMLK_FALSE;
}

SMLK_BOOL SmlkLoanEcuImpl::Check0F3D()
{
    if(m_need_0f3d)
    {
        m_need_0f3d = false;
        return SMLK_TRUE;
    }
    return SMLK_FALSE;
}


void SmlkLoanEcuImpl::SyncIgnStatus(SMLK_BOOL ign_on)
{
    m_need_0f3d = false;
    if(ign_on)
    {
        m_ign_status = SMLK_TRUE;
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
    else
        m_ign_status = SMLK_FALSE;
}
}
}
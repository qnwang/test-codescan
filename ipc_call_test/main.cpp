#include <iostream>
#include <unistd.h>
#include <cstring>

#include "did_ipc_api.h"
#ifdef WITH_HSM_ENCRYPT
#include "smlk_hsm_api.h"
#endif
#include "smlk_timer_new.h"
#include <iostream>
#include <memory>
#include <vector>
using namespace smartlink;
using namespace smartlink::OBD;
#include "smlk_log.h"
// #include "smlk_hsm_api.h"

void TestObdApi()
{
    auto rc_code = DidIpcApi::getInstance()->Init(ModuleID::E_Module_Max);
    printf("DidIpcApi::getInstance()->Init code = [%d]\n", (int)rc_code);

    UdsDtcTriggerInfo info_;
    memset(&info_, 0x0, sizeof(UdsDtcTriggerInfo));
    info_.dtc_type = UdsDtcTriggerType::E_UDS_DTC_TRIGGER_CHECK_VIN_STATUS;
    info_.val = 4;
    std::vector<SMLK_UINT8> vec;
    vec.insert(vec.end(), (SMLK_UINT8*)&info_, (SMLK_UINT8*)&info_ + sizeof(UdsDtcTriggerInfo));
    DidIpcApi::getInstance()->Send2OBD(UdsEventId::E_UDS_DTC_TRIGGER, vec.data(), vec.size());
    printf("DidIpcApi::getInstance()->Send2OBD code = [%d]\n", (int)rc_code);

    UdsEventId evt_id = UdsEventId::E_UDS_CLEAR_ECU_INFO;
    SMLK_RC ret = DidIpcApi::getInstance()->Send2OBD(evt_id, NULL, 0);
    if (ret != SMLK_RC::RC_OK)
    {
        printf("ResetEcuInfoSyncStatus  ipc failed!!!\n");
        return;
    }
    printf("ResetEcuInfoSyncStatus  ipc success!!!\n");
}

void TestTimer()
{
    auto m_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 100>>(
        [&](SMLK_UINT32 index){
            SMLK_LOGI("get smlk_timer_pth index = %d....\n", index);
        }
    );
    // m_timer->SetPeriod(1000);
    m_timer->Start();
    // for (int i = 0 ; i < 10; i ++)
    // {
    //     m_timer->Reset();
    //     m_timer->ReStart();
    //     printf("reset count %d\n", i);
    //     m_timer->Reset();
    //     // m_timer->ReStart();
    // }
    // m_timer->StartWithPause();
    // m_timer->Reset();
    // printf("ready 2 sleep 3s\n");
    // sleep(3);
    // printf("sleep 3s ended!!!  restart !!!\n");
    // for (int i = 0 ; i < 10; i ++)
    // {
    //     m_timer->ReStart();
    //     printf("reset ReStart %d\n", i);
    //     // m_timer->ReStart();
    // }
    // getchar();
    // m_timer->Stop();
    // m_timer->Start();
    getchar();
    m_timer->Stop();
    m_timer.reset();


    auto m_end_timer = std::make_shared<Timer<std::chrono::seconds>>(
        0,
        (int)1,
        [&](std::uint32_t index)
        {
            printf("post fatigue drving end !!!\n");
            printf("get smlk_timer_pth index = %d....\n", index);
        }//end for lambda
    ); //end for make shared;
    m_end_timer->Start();
    getchar();
    m_end_timer->Start();
    getchar();
    m_end_timer->Start();
    getchar();
    m_end_timer->Start();
    getchar();
    m_end_timer->Start();
    getchar();
    m_end_timer->Start();
    getchar();
    m_end_timer->Stop();
    m_end_timer->Stop();
    m_end_timer->Stop();
    m_end_timer->Stop();
    m_end_timer->Stop();
    m_end_timer.reset();
}

void GetAndSetDiagFunc(int nType)
{
    auto rc_code = DidIpcApi::getInstance()->Init(ModuleID::E_Module_Max);
    printf("[obd_ipc_callback] DidIpcApi::getInstance()->Init code = [%d]", rc_code);

    // 测试RegEventCB
    std::vector<OBDEventId> ids;
    ids.push_back(OBDEventId::E_OBD_EVENT_ECU_INFO_SYNC);
    ids.push_back(OBDEventId::E_OBD_EVENT_UDS_MODE_CHANGED);

    rc_code = DidIpcApi::getInstance()->RegEventCB(ids, [](OBDEventId id, void *data, int len){

        printf("[obd_ipc_callback] get callback, eventId: %d \n",id);
        if ( id == OBDEventId::E_OBD_EVENT_UDS_MODE_CHANGED)
        {
            if (len != sizeof(UdsDiagConfig))
            {
                printf("length err \n");
                return;
            }

            UdsDiagConfig configData = *(UdsDiagConfig *)data;
            printf("[obd_ipc_callback] get diag mode: diag_cfg.UdsDiagMode: %d \n", configData.mode);
            printf("[obd_ipc_callback] get diag mode: diag_cfg.UdsDiagPriority: %d \n", configData.pri);
            printf("[obd_ipc_callback] get diag mode: diag_cfg.UdsDiagLock: %d \n", configData.lock);
        }
    });


    // 测试 SetUdsDiagMode
    UdsDiagMode mode;
    bool b_is_start = true;

    if (nType == 1)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_LOAN;
    }
    else if (nType == 2)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_FOTA_READ;
    }
    else if (nType == 3)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_FOTA_WRITE;
    }
    else if (nType == 4)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_DIAG_WRITE;
    }
    else if (nType == 5)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_DIAG_OTHER;
    }
    else if (nType == 6)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_LOAN;
        b_is_start = false;
    }
    else if (nType == 7)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_FOTA_READ;
        b_is_start = false;

    }else if (nType == 8)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_FOTA_WRITE;
        b_is_start = false;

    }else if (nType == 9)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_DIAG_WRITE;
        b_is_start = false;

    }else if (nType == 10)
    {
        mode = UdsDiagMode::E_UDS_DIAG_MODE_DIAG_OTHER;
        b_is_start = false;

    }

    rc_code = DidIpcApi::getInstance()->SetUdsDiagMode(mode, b_is_start);
    printf("[obd_ipc_callback] DidIpcApi::getInstance()->SetUdsDiagMode code = [%d] \n", rc_code);

    // 测试 GetUdsDiagMode
    UdsDiagConfig getConfig;
    std::memset(&getConfig, 0 , sizeof(UdsDiagConfig));
    rc_code = DidIpcApi::getInstance()->GetUdsDiagMode(getConfig);
    printf("[obd_ipc_callback] DidIpcApi::getInstance()->GetUdsDiagMode code = [%d] \n", rc_code);


    printf("[obd_ipc_callback] get diag mode: diag_cfg.UdsDiagMode: %d \n", getConfig.mode);
    printf("[obd_ipc_callback] get diag mode: diag_cfg.UdsDiagPriority: %d \n", getConfig.pri);
    printf("[obd_ipc_callback] get diag mode: diag_cfg.UdsDiagLock: %d \n", getConfig.lock);

}
#include "smlk_spi_manager.h"
using namespace smartlink::CAN;
void TestSmlkSPI(int type)
{
    SmlkUds::getInstance()->Init((ModuleID)((int)ModuleID::E_Module_Min+type), (SmlkUdsMode)((int)SmlkUdsMode::UDS_DIAG_MODE_LOAN + type));
    if (!SmlkUds::getInstance()->CheckUdsWritePermission())
    {
        printf("Current mode is not UDS_DIAG_MODE_LOAN mode!!!\n");
        if (!SmlkUds::getInstance()->EnterUdsMode())
        {
            printf("Enter UDS_DIAG_MODE_LOAN mode failed!!!\n");
        }
        printf("Enter UDS_DIAG_MODE_LOAN mode success!!!\n");
        sleep(10);

        SmlkUds::getInstance()->ExitUdsMode();
        printf("Exit UDS_DIAG_MODE_LOAN mode success!!!\n");
    }
}

//单纯uds读写操作不需要调用 SmlkUds::getInstance()->Init 初始化1
void TestSmlkSPIWrite()
{
    MCU::GetInstance()->Init(ModuleID::E_Module_Min);
    std::vector<McuEventId>  mcu_events = {
        McuEventId::E_MCU_EVENT_UDS_ASYNC_RESPONSE
    };
    MCU::GetInstance()->RegEventCB(mcu_events, [&](McuEventId id, void *data, int len){
        if (id == McuEventId::E_MCU_EVENT_UDS_ASYNC_RESPONSE)
        {
           SmlkUds::getInstance()->OnMcuEvent(id, data, len);
        }
    });
    UdsInfo u_info;
    memset(&u_info, 0x0, sizeof(UdsInfo));
    u_info.resp_addr = 0x18DAF100;
    SmlkUds::getInstance()->SetUdsInfo(u_info);

    std::vector<unsigned char> vec_;
    vec_.push_back(0xF1);
    vec_.push_back(0x87);
    UdsResp u_req;
    u_req.resp_addr = 0; // if addr is 0 use SetUdsInfo cached
    u_req.len = vec_.size();
    u_req.data = vec_.data();
    // send msg need init MCU::GetInstance()->Init();
    if (!SmlkUds::getInstance()->WriteUdsMsg(0x18DA00F1, 0x22/*svc_id*/, &u_req))
    {
        printf("Send uds msg failed!!!\n");
        return;
    }
    printf("Send uds msg success!!!\n");
    UdsResp u_resp;
    unsigned char  buf[1024] = {0};
    u_resp.data = buf;
    if (SmlkUds::getInstance()->ReadUdsMsg(&u_resp, 5000/*ms*/))
    {
        printf("msg_read data is :\n");
        // svc_id = 0x62
        // 0x62 0xf1 0x87 0x33 0x36 0x30 0x31 0x31 0x31 0x35 0x2d 0x33 0x36 0x46 0x20
        for (int i = 0; i < (int)u_resp.len; i++)
        {
            printf("0x%02x ", u_resp.data[i]);
        }
        printf("\n");
        return;
    }
    printf("read uds msg failed!!!\n");
}



#ifdef WITH_HSM_ENCRYPT
void testHsmApi(int nType)
{
    int ret = 0;
    ret = hsm_seal_SE_conn();
    printf("======================hsm_seal_SE_conn ret:%d\n", ret);

    ASYMM_KEY_TYPE type = TYPE_SM2;

    if (nType != 0)
    {
        type = TYPE_RSA;
    }
    ret = hsm_seal_gen_asymm_keypair(type, 2048, 0, 0);

    printf("======================hsm_seal_gen_asymm_keypair ret:%d\n", ret);
    printf("====================== hsm_seal_gen_asymm_keypair end...\n");




    printf("====================== hsm_seal_get_pubkey enter...\n");
    // unsigned char *pubkey, size_t *p_len, int kid
    size_t publength = 1024;
    unsigned char pubkey[publength];
    ret = hsm_seal_get_pubkey(type, pubkey, &publength, 0);
    printf("======================hsm_seal_get_pubkey ret:%d\n", ret);
    printf("pubkey length:%d, data: %s \n", publength, pubkey);
    for(int i = 0; i < publength; i++)
    {
        printf(" %02x", pubkey[i]);
    }
    printf("\n\n");
    printf("====================== hsm_seal_get_pubkey end...\n");

    while (true)
    {
         printf("====================== hsm_seal_sign enter \n");
        size_t n_data_length = 1024;
        // unsigned char dataT[n_data_length] = "hello world, my name is jekin.hello world, my name is jekin.hello world, my name is jekin.hello world, my name is jekin.hello world, my name is jekin.hello world, my name is jekin.hello world, my name is jekin.hello world, my name is jekin.hello world, my name is jekin.hello world, my name is jekin.hello world, my name is jekin.";
        unsigned char dataT[n_data_length] = "hello world, my name is jekin";


        size_t n_sign_length = 256;
        unsigned char signData[n_sign_length];
        memset(signData, 0, n_sign_length);
        printf("=====sign init: %s \n", signData);

        ret = hsm_seal_sign(type, signData, &n_sign_length, dataT, n_data_length, ALG_MD3, 0);
        printf("======================hsm_seal_sign ret:%d\n", ret);
        printf("=====sign length:%d, data: %s \n", n_sign_length, signData);
        for(int i = 0; i < n_sign_length; i++)
        {
            printf(" %02x",signData[i]);
        }
        printf("\n\n");

        // printf("====================== SM3SignSplitRCodeAndSCode run \n");
        // size_t R_length = 32;
        // size_t S_length = 32;
        // unsigned char R_Sign[R_length];
        // unsigned char S_Sign[S_length];
        // ret = SM3SignSplitRCodeAndSCode(signData, n_sign_length, R_Sign, &R_length, S_Sign, &S_length);
        // printf("======================SM3SignSplitRCodeAndSCode ret:%d\n", ret);

        // printf("RCode=====\n");
        // for(int i = 0; i < R_length; i++)
        // {
        //     printf(" %02x",R_Sign[i]);
        // }
        // printf("\n\n");

        // printf("SCode=====\n");
        // for(int i = 0; i < S_length; i++)
        // {
        //     printf(" %02x",S_Sign[i]);
        // }
        // printf("\n\n");



        printf("====================== hsm_seal_sign end.............. \n");


        printf("======================hsm_seal_verify enter----------\n");
        ret = hsm_seal_verify(type, dataT, n_data_length, signData, n_sign_length, ALG_MD3, 0);
        printf("======================hsm_seal_verify ret:%d\n", ret);
        printf("======================hsm_seal_verify end----------\n");


        printf("======================hsm_seal_encrypt enter----------\n");
        size_t out_data_length = 256;
        unsigned char out_data[out_data_length];
        memset(out_data, 0, out_data_length);

        size_t encrypt_data_length = 100;
        unsigned char encryptData[encrypt_data_length] = "hello world, this is test run";
        printf("=====init data: %s \n", encryptData);

        ret = hsm_seal_encrypt(type, out_data, &out_data_length, encryptData, encrypt_data_length, ALG_RSA_NOPAD, 0);
        printf("======================hsm_seal_encrypt ret:%d\n", ret);
        printf("=====encrypt data: %s \n", out_data);
        printf("======================hsm_seal_encrypt end----------\n");


        printf("======================hsm_seal_verify enter----------\n");
        size_t out_decrypt_data_length = 200;
        unsigned char out_crypt_data[out_decrypt_data_length];
        memset(out_crypt_data, 0, out_decrypt_data_length);
        ret = hsm_seal_decrypt(type, out_crypt_data, &out_decrypt_data_length, out_data, out_data_length, ALG_RSA_NOPAD, 0);
        printf("======================hsm_seal_decrypt ret:%d\n", ret);
        printf("=====decrypt data: %s \n", out_crypt_data);
        printf("======================hsm_seal_decrypt end----------\n");


        sleep(3);
    }
}
#endif

int main(int argc,char *argv[])
{
// #ifdef WITH_HSM_ENCRYPT
// #endif
    // TestSmlkSPIWrite();
    // getchar();
    // TestTimer();
    int nType = 0;
    if (argc > 1)
    {
        try
        {
            nType = atoi(argv[1]);
        }
        catch(const std::exception& e)
        {
            printf("logmain atoi err：%s \n",e.what());
        }
    }
    else
    {
        printf("./ipc_call_test 0  ::: for run func \n");
        printf("./ipc_call_test 1  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_LOAN \n");
        printf("./ipc_call_test 2  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_FOTA_READ \n");
        printf("./ipc_call_test 3  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_FOTA_WRITE \n");
        printf("./ipc_call_test 4  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_DIAG_WRITE \n");
        printf("./ipc_call_test 5  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_DIAG_OTHER \n");

        printf("./ipc_call_test 6  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_LOAN end \n");
        printf("./ipc_call_test 7  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_FOTA_READ end \n");
        printf("./ipc_call_test 8  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_FOTA_WRITE end \n");
        printf("./ipc_call_test 9  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_DIAG_WRITE end \n");
        printf("./ipc_call_test 10  ::: for run GetAndSetDiagFunc func, mode is E_UDS_DIAG_MODE_DIAG_OTHER end \n");

        return 0;
    }
    // GetAndSetDiagFunc(nType);
    // TestSmlkSPI(nType);
#ifdef WITH_HSM_ENCRYPT
    testHsmApi(nType);
#endif



    printf("\n\n\n");

    // while (true)
    // {
    //     /* code */
    //     sleep(1);
    // }
    getchar();

    return 0;
}
/*****************************************************************************
*smlk_diag.h
******************************************************************************/
#ifndef _SMLK_DIAG_H_
#define _SMLK_DIAG_H_

#include <iostream>
#include <thread>
#include <set>
#include <mutex>

#include <rxd/tboxInterface.h>
#include "smartlink_sdk_mcu.h"
#include "smartlink_sdk_location.h"
#include "did_ipc_api.h"
#include "tsp_service_api.h"

using namespace smartlink_sdk;


class SmlkDiag
{
    public:
        ~SmlkDiag();

        int Init();

        int Start();

        int Stop();

        static SmlkDiag* GetInstance();

    public:

        static SmlkDiag* m_instance;

        static std::string m_iccid;
        static std::string m_imsi;
        static std::string m_sn;
        static std::string m_engine;
        static std::string m_vin;
        static std::string m_ein;

        static bool m_ign_state;
        static bool m_can_state;

        static LocationInfo m_location;

        /*处理wifi对接业务数据*/
        static std::string m_wifi_ssid;
        static std::string m_wifi_key;
        static std::thread m_wifi_connect_thread;
        static bool m_wifi_connected;
        static uint8_t m_wifi_timeout;

        /*处理tsp对接业务数据*/
        static smartlink::TspConnectState m_tsp_connect_state;
        static smartlink::TspLoginState m_tsp_login_state;

        static tboxInterface_t * m_sdk_instance;

        static std::set<uint32_t>  m_can_filter_list;

        static bool m_can_send_enable;

        /*UDS服务锁*/
        static smartlink::OBD::UdsDiagConfig m_uds_status;
        static std::mutex g_uds_mutex;     // 用到的全局锁

    private:

        SmlkDiag();

        int InitPro();

        int InitODB();

        int InitMcu();

        int InitPower();

        int InitLoc();

        int InitTel();

        int InitTsp();

        int InitWlan();

        int GetDiagMode();

        int SetDiagMode(bool lock);

    private://瑞修德SDK接口

        tboxInterface_t * SdkInit(void);

        static int SdkGetTboxInfo(tboxInfo_t *info);
        static int SdkGetTboxGpsInfo(tboxGPS_t *info);
        static void SdkGetTboxHardwareStatus(tboxHardwareStatusFlag_u *flag);
        static void SdkGetTboxHardwareChecks(tboxHardwareChecksFlag_u *flag);

        static int SdkTboxCanFilter(const tboxCanFilter_t *filter);
        static int SdkTboxCanConfig(const tboxCanConfig_t *config);
        static int SdkSendCanMessageFlag();
        static int SdkSendCanMessage(const tboxCanTxMsg_t *msg, const uint32_t cnt);

        static void SdkSendDatasToServer(const uint8_t *data, const uint32_t sz, const uint32_t channel);
        static void SdkAppModeConfig(int mode);

        static int SdkWifiConfig(tboxWifiConfig_t *config);
        static int SdkConnectAp(const char *ssid, const char *key, int timeout);
        static void SdkSendDataToWifi(const uint8_t *data, const uint32_t sz);

        static int SdkNetworkInit(tboxGPRS_t *gprs);
        static int SdkNetworkStatus(tboxGPRS_t *gprs);

};


#endif
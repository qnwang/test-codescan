#include <smlk_log.h>
#include "smlk_diag.h"
#include "did_ipc_api.h"

#include <chrono>
#include <thread>
#include <unordered_map>
#include <csignal>

namespace {
    volatile sig_atomic_t signal_status = 0;
};

static void signal_handler(int signal) {
    if ( (SIGTERM == signal) || (SIGINT == signal) ) {
        signal_status = signal;
    }
}

int main(int argc, char *argv[])
{
    // if (smartlink::SMLK_RC::RC_OK != smartlink::OBD::DidIpcApi::getInstance()->Init(ModuleID::E_Module_diag_service) )
    // {
    //     SMLK_LOGE("fail to init system property module");
    //     return -1;
    // }

    // smartlink::OBD::UdsDiagConfig config;
    // config.lock = smartlink::OBD::UdsDiagLock::E_UDS_DIAG_LOCK;
    // config.mode = smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_GB6;
    // config.pri  = smartlink::OBD::UdsDiagPriority::E_UDS_DIAG_PRI_FOTA;
    // smartlink::OBD::DidIpcApi::getInstance()->SetUdsDiagMode(config, 600);

    /*初始化诊修模块成功*/
    if(SmlkDiag::GetInstance()->Init() == 0)
    {
       /*启动诊修业务*/
       SmlkDiag::GetInstance()->Start();
    }

   // signal(SIGTERM, signal_handler);
   // signal(SIGINT,  signal_handler);

    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    }
}
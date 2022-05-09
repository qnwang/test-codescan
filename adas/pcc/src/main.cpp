#include <stdio.h>

#include "pcc_service.h"
using namespace smartlink;


PccService ser;
int Av2hpMessageNotify(const char *message, int *message_num)
{
    ser.Av2hpMessageNotify(message, message_num);
    return 0;
}

int main(int argc, char *argv[]) {
    bool b_use_test_data = false;
    if(argc > 2)
    {
        return -1;
    }
    else if(argc == 2)
    {
        b_use_test_data = true;
    }

    if (M_L_RETURN_SUCCESS != ser.Init())
    {
        SMLK_LOGW("service init err...");
        return -1;
    }

    if(M_L_RETURN_SUCCESS != ser.Start( ))
    {
        SMLK_LOGW("service start err...");
        return -1;
    }

    //特殊： 只能注册普通方法， 所以通过声明全局PccService对象来让普通方法将数据传入类对象中
    OnAv2HP_setMessageCB pMsgcb = NULL;
    ser.GetHPMsgCb(pMsgcb);
    if (pMsgcb != NULL)
    {
        pMsgcb(Av2hpMessageNotify);
    }

    if (b_use_test_data)
    {
        ser.RunTestData();
    }

    while (true) {
        if ( !ser.IsRunning() ) {
            SMLK_LOGE("service not run...");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    ser.Stop();

    return 0;
}
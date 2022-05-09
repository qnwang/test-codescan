#include <iostream>
#include "avm_manager.h"

using namespace smartlink::AVM;

int main()
{
    VedioProcessorManager mng;
    smartlink::SMLK_RC ret = mng.Init();
    if (ret != smartlink::SMLK_RC::RC_OK)
    {
        return -1;
    }
    mng.Start();
    mng.Loop();
    return 0;
}
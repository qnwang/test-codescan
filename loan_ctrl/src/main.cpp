#include <iostream>
#include <vector>

#include "smlk_loan_ctrl_api.h"
#include "smlk_log.h"

using namespace smartlink::LOAN;


int main()
{
    SmlkLoanManager ser;
    if (!ser.Init())
    {
        std::cerr << "service init failed" << std::endl;
        return -1;
    }
    if (!ser.Start())
    {
        std::cerr << "service start failed" << std::endl;
    }
    // ser.Init();
    // ser.Start();
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(250));
    }
    ser.DeInit();
    return 0;
}
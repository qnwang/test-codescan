#pragma once
#include <unordered_set>
#include <string>
#include <set>
#include <map>
#include <unordered_map>
#include <type_traits>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <ctime>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "stdio.h"
#include <stdlib.h>
#include <array>
#include <list>
#include <mutex>
#include <thread>
#include <utility>
#include <smlk_types.h>
//#include "smlk_timer_new.h"
#include <unistd.h>
#include <smlk_timer_inf.h>
#include <smlk_service_inf.h>
#include "Poco/Timer.h"
#include "smlk_log.h"
#include "smlk_error.h"
#include <smartlink_sdk_tel.h>
#include <smartlink_sdk_mcu.h>
#include <smartlink_sdk_location.h>
#include <smartlink_sdk_sys_power.h>
#include <location_manager.h>
#include "smartlink_sdk_sys_property_def.h"
#include "smartlink_sdk_sys_property.h"
#include "smartlink_sdk_sys_time.h"
#include <smlk_queue.h>
#include <smlk_fixed_queue.h>
#include "ipc_api_def.h"


#define SL_SUCCESS      0
#define SL_EFAILED      1
#define SL_ESUPPORT     2
#ifdef PLATFORM_ARM
    static const std::size_t    SL_PLATFORM_MAX_BYTES           = 32;
    static const std::size_t    SL_PLATFORM_MAX_CHARS           = 160;
#endif
#define M_SL_TIMESTAMP_SZ       6


static int TimeStamp2BCD(IN std::time_t tt, INOUT SMLK_BCD *BCD, IN std::size_t sz) {
    if ( (nullptr == BCD) || (sz != M_SL_TIMESTAMP_SZ) ) {
        SMLK_LOGE("TimeStamp2BCD FAIL!!!");
        return -1;
    }
    std::tm tm;
    // check whether src time is invalid
    std::time_t local_time;
    if (tt == 0) {
        SMLK_LOGD("timestamp src = 0");
        auto now    = std::chrono::system_clock::now();
        local_time = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        localtime_r(&local_time, &tm);
    } else {
        localtime_r(&tt, &tm);
    }

    //std::memcpy(&tm, std::localtime(&tt), sizeof(tm));

    tm.tm_year -= 100;      // years since 2000
    tm.tm_mon += 1;

    BCD[0] = ((((SMLK_UINT8)(tm.tm_year / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_year % 10));

    BCD[1] = ((((SMLK_UINT8)(tm.tm_mon  / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_mon  % 10));

    BCD[2] = ((((SMLK_UINT8)(tm.tm_mday / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_mday % 10));
    BCD[3] = ((((SMLK_UINT8)(tm.tm_hour / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_hour % 10));
    BCD[4] = ((((SMLK_UINT8)(tm.tm_min  / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_min  % 10));
    BCD[5] = ((((SMLK_UINT8)(tm.tm_sec  / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_sec  % 10));
#ifdef SL_DEBUG
    SMLK_LOGD("[timedebug] BCD[0] is == %2x ", BCD[0]);
    SMLK_LOGD("[timedebug] BCD[1] is == %2x ", BCD[1]);
#endif
    return 0;
}
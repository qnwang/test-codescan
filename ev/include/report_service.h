  /******************************************************************************
*
*  Copyright (C) 2020 SmartLink
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/
#pragma once
#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <set>
#include <vector>
#include <utility>
#include <unordered_map>
#include <smlk_queue.h>
#include <smlk_timer_inf.h>
#include <smlk_service_inf.h>
#include "callable_event.h"
#include "protocol_reporter.h"
#include "message_def_32960.h"

#include "Poco/Timer.h"

#define CYCLE_TIMEOUT_COUNT         10

namespace smartlink {

class ReportService : public IService {
public:
    ReportService();
    ~ReportService();
    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    void            Stop();
    bool            IsRunning() const;

private:
    void Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz);
    void Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data);

    void HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void InitTimerOutCheck();
    void OnCheckDataValidTimeout(Poco::Timer&);
    void OnMcuNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnDM1Notify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnMutiDataNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnLocationNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    void GetGNSS();

private:
    SMLK_UINT32             m_flags;
    SMLK_UINT32             m_internal_flags;
    SMLK_UINT8              m_vehicle_type;
    SMLK_UINT8              is_main_power_flag;

    std::mutex                      m_mutex;
    std::thread                     m_thread;
    Queue<CallableEvent>            m_events;

    ProtocolReporter::DM1::Info         dm1info;

    typedef union {
        SMLK_UINT32 id;
        struct {
            SMLK_UINT32 SPN:19;
            SMLK_UINT32 FMI:5;
            SMLK_UINT32 SRC:8;
        }           dtc;
    } InternalDTC;

    using CAN_ID        = SMLK_UINT32;
    struct DM1DataInfo
    {
        SMLK_BOOL     data_valid;
        SMLK_UINT32   cycle;                          // can raw data cycle ms
        std::chrono::steady_clock::time_point tp;     // timepoint for data received
    };
    std::unordered_map<CAN_ID, DM1DataInfo> g_dm1_data_pool;
    std::mutex      g_dm1_data_pool_mutex;
    Poco::Timer     m_check_timeout_timer;

    std::set<SMLK_UINT32>       m_actived_dm1;
    std::map<SMLK_UINT8, std::shared_ptr<Reporter>> m_protocol_reporter;
};

};  // namespace smartlink

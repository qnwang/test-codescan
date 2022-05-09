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
#include <smlk_queue.h>
#include <smlk_timer_inf.h>
#include <smlk_service_inf.h>
#include "callable_event.h"
#include "protocol_reporter_1239_07.h"
#include "protocol_reporter.h"

#include "did_ipc_api.h"

#include "result_handler.h"

#include <netdb.h>

namespace smartlink {

struct TspInfo {
    SMLK_UINT8      platform;
    SMLK_BOOL       valid;
    SMLK_UINT8      protocol;
    SMLK_UINT16     port;
    std::string     host;
};  // struct TspInfo

class ReportService : public IService {
public:
    ReportService();
    ~ReportService();
    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    SMLK_UINT32     InitTSPConfig();

    void            Stop();
    bool            IsRunning() const;

private:
    void Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz);
    void Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data);

    void HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnLocationNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnMcuNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnUDS(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnTimer1S();
    void OnTimer(IN SMLK_UINT32 id, IN SMLK_UINT32 seq);
    void OnTspNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);

    void ReadTimeSource();
    void GetGNSS();
    void QueryOBD();


private:
    // SMLK_UINT32             m_internal_flags;
    SMLK_UINT32             m_ign_flags;
    SMLK_UINT32             m_systime_flags;
    SMLK_UINT32             m_obd_query_delay;
    SMLK_UINT32             m_service_flags;

    std::string             m_vin;
    std::string             m_imsi;
    std::string             m_iccid;
    std::string             m_chipid;
    std::string             m_pub_key;

    SMLK_UINT8              m_ems_model;
    SMLK_UINT8              m_gb_platform;
    SMLK_UINT32             m_time_source;
    SMLK_INT64              m_timeSyncGap_ms;

    SMLK_UINT32             m_todisk;
    SMLK_BOOL               is_gas;
    SMLK_BOOL               is_hev;
    SMLK_UINT8              m_product_loc;
    SMLK_UINT8              m_activated_status;

    TspInfo                 m_ent_tspinfo; // 企业平台配置
    TspInfo                 m_loc_tspinfo; // 地方平台配置

    std::vector<SMLK_UINT32>    indexs;

    std::mutex              m_mutex; 
    std::thread             m_thread;
    Queue<CallableEvent>    m_events;

    std::shared_ptr<ITimer>     m_timer_1s;
    std::shared_ptr<ITimer>     m_timer_obd;
    std::shared_ptr<ITimer>     m_timer_daychange;

    std::map<SMLK_UINT8, std::shared_ptr<Reporter>> m_protocol_reporter;
    std::shared_ptr<Reporter1239_07> m_protocol_reporter_1239_07;

// 配置文件控制 调试/过检使用
    SMLK_UINT32             m_env;    // 生产 : 0; 调试 : 1

    std::string             m_host_01;      // 企业平台 debug
    SMLK_UINT16             m_port_01;

    std::string             m_host_02;      // 地方平台 debug
    SMLK_UINT16             m_port_02;

    std::string             m_host_03;      // 激活平台 debug
    SMLK_UINT16             m_port_03;

    std::string             m_chipid_head;

};

};  // namespace smartlink

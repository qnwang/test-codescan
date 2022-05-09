#pragma once
#include <mutex>
#include <thread>
#include <chrono>
#include <set>
#include <vector>
#include <utility>
#include <unordered_map>
#include <smlk_types.h>
#include <smlk_queue.h>
#include <smlk_fixed_queue.h>
#include <smlk_timer_new.h>
#include <smlk_service_inf.h>
#include "callable_event.h"
#include "Poco/Timer.h"
namespace smartlink {

class BluetoothService : public IService {

public:

    BluetoothService();
    virtual ~BluetoothService();

    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    void            Stop();
    bool            IsRunning() const;

    void Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz);
    void Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data);

private:
    bool SendDataToMcu(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnMcuNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnIgnOn(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnIgnOff(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    void HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data);
    SMLK_UINT32     m_seq;
    Queue<CallableEvent>    m_events;
    SMLK_UINT32     m_internal_flags;

    SMLK_UINT32     m_flags;
    std::mutex      m_mutex;
    std::thread     m_thread;
};



}
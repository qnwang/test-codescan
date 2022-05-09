#pragma once
#include <vector>
#include <functional>
#include "ipc_head.h"

namespace smartlink {

class CallableEvent {
    using   Func = std::function<void (IN MsgHead &, IN std::vector<SMLK_UINT8> &)>;
public:
    CallableEvent();
    CallableEvent(IN MsgHead &head, IN SMLK_UINT8 *data, std::size_t sz, Func func);
    CallableEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data, Func func);
    CallableEvent(IN MsgHead &head, std::vector<SMLK_UINT8> &&data, Func func);
    CallableEvent(IN CallableEvent &other);
    CallableEvent(CallableEvent &&other);
    CallableEvent & operator=(IN CallableEvent &other);
    void operator()();
private:
    MsgHead                 m_head;
    std::vector<SMLK_UINT8> m_data;
    Func                    m_func;
};

}   // namespace smartlink
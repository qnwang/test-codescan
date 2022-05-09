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

};  // namespace smartlink

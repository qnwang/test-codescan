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
#include <callable_event.h>

using namespace smartlink;

CallableEvent::CallableEvent()
{}

CallableEvent::CallableEvent(IN MsgHead &head, IN SMLK_UINT8 *data, std::size_t sz, Func func)
    : m_head(head)
    , m_data(data, data+sz)
    , m_func(func)
{}

CallableEvent::CallableEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data, Func func)
    : m_head(head)
    , m_data(data)
    , m_func(func)
{}

CallableEvent::CallableEvent(IN MsgHead &head, std::vector<SMLK_UINT8> &&data, Func func)
    : m_head(head)
    , m_data(std::move(data))
    , m_func(func)
{}

CallableEvent::CallableEvent(IN CallableEvent &other)
    : m_head(other.m_head)
    , m_data(other.m_data)
    , m_func(other.m_func)
{}

CallableEvent::CallableEvent(CallableEvent &&other)
    : m_head(other.m_head)
    , m_func(other.m_func)
{
    m_data.swap(other.m_data);
}

CallableEvent & CallableEvent::operator=(IN CallableEvent &other)
{
    m_head = other.m_head;
    m_data = other.m_data;
    m_func = other.m_func;

    return *this;
}

void CallableEvent::operator()(void)
{
    if ( m_func ) {
        m_func(m_head, m_data);
    }
}

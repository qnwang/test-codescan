#pragma once
#include <vector>

#include "event_msg_head.hpp"


namespace smartlink {

class EventVarible
{
    using   EventCB = std::function<void (IN MessageHead &, std::vector<SMLK_UINT8> &)>;
public:
    EventVarible() {}
    EventVarible(IN MessageHead& head, IN SMLK_UINT8* data, IN SMLK_UINT32 lenth, EventCB cb)
                                                            : m_head(head)
                                                            , m_data(data, data + lenth)
                                                            , m_cb(cb)
    {}

    EventVarible(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data, EventCB cb)
                                                            : m_head(head)
                                                            , m_data(data)
                                                            , m_cb(cb)
    {}

    EventVarible(IN EventVarible& other)
                 : m_head(other.m_head)
                 , m_data(other.m_data)
                 , m_cb(other.m_cb)
    {}

    EventVarible(EventVarible&& other)
                : m_head(other.m_head)
                , m_cb(other.m_cb)
    {
        m_data.swap(other.m_data);
    }

    ~EventVarible()
    {}

    void operator()()
    {
        if (m_cb)
        {
            m_cb(m_head, m_data);
        }
    }
private:
    MessageHead                 m_head;
    std::vector<SMLK_UINT8>     m_data;
    EventCB                     m_cb;
};

};
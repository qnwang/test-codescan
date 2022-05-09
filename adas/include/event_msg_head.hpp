#pragma once


namespace smartlink {

class MessageHead
{
public:
    MessageHead() : m_event(0)
                    , m_seq(0)
                    , m_extra(0)
    {}

    ~MessageHead()
    {}

    MessageHead(SMLK_UINT8 event, SMLK_UINT32 seq, SMLK_UINT32 extra)
                                                        : m_event(event)
                                                        , m_seq(seq)
                                                        , m_extra(extra)
    {}

    MessageHead(const MessageHead& other)
                        : m_event(other.m_event)
                        , m_seq(other.m_seq)
                        , m_extra(other.m_extra)
    {}

public:
    SMLK_UINT8              m_event;
    SMLK_UINT32             m_seq;
    SMLK_UINT32             m_extra;
};

};  //namespace smartlink
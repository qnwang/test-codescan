#pragma once
#include "smlk_types.h"

namespace smartlink {

class MsgHead {
public:
    MsgHead()
        : m_id(0)
        , m_seq(0)
        , m_protocol(0)
        , m_compress(0)
        , m_length(0)
        , m_extra(0)
    {};
    MsgHead(SMLK_UINT32 id, SMLK_UINT32 seq, SMLK_UINT8 protocol, SMLK_UINT8 compress, SMLK_UINT16 length, SMLK_UINT32 extra)
        : m_id(id)
        , m_seq(seq)
        , m_protocol(protocol)
        , m_compress(compress)
        , m_length(length)
        , m_extra(extra)
    {}
    MsgHead(const MsgHead &other)
        : m_id(other.m_id)
        , m_seq(other.m_seq)
        , m_protocol(other.m_protocol)
        , m_compress(other.m_compress)
        , m_length(other.m_length)
        , m_extra(other.m_extra)
    {}
    MsgHead & operator=(const MsgHead &other)
    {
        if ( this != &other ) {
            this->m_id          = other.m_id;
            this->m_seq         = other.m_seq;
            this->m_protocol    = other.m_protocol;
            this->m_compress    = other.m_compress;
            this->m_length      = other.m_length;
            this->m_extra       = other.m_extra;
        }
        return *this;
    }
public:
    SMLK_UINT32     m_id;
    SMLK_UINT32     m_seq;
    SMLK_UINT32     m_protocol:4;
    SMLK_UINT32     m_compress:4;
    SMLK_UINT32     :8;
    SMLK_UINT32     m_length:16;
    SMLK_UINT32     m_extra;
};  // class MsgHead

}   // namespace smartlink

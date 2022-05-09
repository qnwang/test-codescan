#pragma once
#include "message.h"

namespace smartlink {

enum
{
    STATUS_NOT_ENABLE = 1,/*金融锁车功能未使能*/
    STATUS_NOT_ACTIVE,/*未激活,金融锁车功能已使能*/
    STATUS_IS_ACTIVE,/*已激活未锁车*/
    STATUS_IS_LOCKED,/*已激活已锁车*/
};

class MessageLocation808 : public MessageLocation {
public:
    MessageLocation808();
    ~MessageLocation808();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data);
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
};

class MessageBucket808 : public MessageBucket {
public:
    MessageBucket808();
    ~MessageBucket808();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data);
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
    std::size_t     CompressOffset() const;
};

class MessageBucket30S808 : public MessageBucket30S {
public:
    MessageBucket30S808();
    ~MessageBucket30S808();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data);
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
    std::size_t     CompressOffset() const;
};

class MessageCompressedBucket808 : public MessageCompressedBucket {
public:
    MessageCompressedBucket808();
    ~MessageCompressedBucket808();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &);
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &);
};

class MessageStatistics808 : public MessageStatistics {
public:
    MessageStatistics808();
    ~MessageStatistics808();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data);
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
};

class MessageEvent808 : public MessageEvent {
public:
    MessageEvent808();
    ~MessageEvent808();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data);
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
};

class MessageLocationRsp808 : public MessageLocationRsp {
public:
    MessageLocationRsp808();
    ~MessageLocationRsp808();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data);
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
};

class MessageDM1808 : public MessageDM1 {
public:
    MessageDM1808();
    ~MessageDM1808();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data);
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
};

class MessageFault808 : public MessageFault {
public:
    MessageFault808();
    ~MessageFault808();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data);
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
};

}   // namespace smartlink

#pragma once
#include <vector>
#include "smlk_types.h"

namespace smartlink {

class IMessage {
public:
    virtual ~IMessage(){};

    virtual SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &) = 0;
    virtual SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &) = 0;
};

}   // namespace smartlink

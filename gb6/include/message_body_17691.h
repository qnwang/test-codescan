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
#include "message_body.h"

namespace smartlink {

class MessageEngineInfo17691 : public MessageEngineInfo {
public:
    MessageEngineInfo17691();
    MessageEngineInfo17691(const MessageEngineInfo17691 &other);
    ~MessageEngineInfo17691();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data) const;
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
    std::shared_ptr<IMessage>   Duplicate() const;    
    std::size_t                 EncodedSize() const;
};  // class MessageEngineInfo17691

class MessageOBDInfo17691 : public MessageOBDInfo {
public:
    MessageOBDInfo17691();
    MessageOBDInfo17691(const MessageOBDInfo17691 &other);
    ~MessageOBDInfo17691();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data) const;
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
    std::shared_ptr<IMessage>   Duplicate() const;
    std::size_t                 EncodedSize() const;
};  // class MessageOBDInfo17691

class MessageReport17691 : public MessageReport {
public:
    MessageReport17691();
    MessageReport17691(const MessageReport17691 &other);
    ~MessageReport17691();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data) const;
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
    std::shared_ptr<IMessage>   Duplicate() const;    
    std::size_t                 EncodedSize() const;
};  // MessageReport17691

class MessageVehicleLogin17691 : public MessageVehicleLogin {
public:
    MessageVehicleLogin17691();
    MessageVehicleLogin17691(const MessageVehicleLogin17691 &other);
    ~MessageVehicleLogin17691();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data) const;
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
    std::shared_ptr<IMessage>   Duplicate() const;    
    std::size_t                 EncodedSize() const;
};  // MessageVehicleLogin17691

class MessageVehicleLogout17691 : public MessageVehicleLogout {
public:
    MessageVehicleLogout17691();
    MessageVehicleLogout17691(const MessageVehicleLogout17691 &other);
    ~MessageVehicleLogout17691();

    SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &data) const;
    SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &data);
    std::shared_ptr<IMessage>   Duplicate() const;
    std::size_t                 EncodedSize() const;    
};  // MessageVehicleLogout17691

};
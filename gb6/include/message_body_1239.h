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

namespace smartlink
{

    class MessageEngineDpfScrInfo1239 : public MessageEngineInfo
    {
    public:
        MessageEngineDpfScrInfo1239();
        MessageEngineDpfScrInfo1239(const MessageEngineDpfScrInfo1239 &other);
        ~MessageEngineDpfScrInfo1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // class MessageEngineDpfScrInfo1239

    class MessageEngineTwcInfo1239 : public MessageEngineInfo
    {
    public:
        MessageEngineTwcInfo1239();
        MessageEngineTwcInfo1239(const MessageEngineTwcInfo1239 &other);
        ~MessageEngineTwcInfo1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // class MessageEngineTwcInfo1239

    class MessageEngineTwcNOxInfo1239 : public MessageEngineInfo
    {
    public:
        MessageEngineTwcNOxInfo1239();
        MessageEngineTwcNOxInfo1239(const MessageEngineTwcNOxInfo1239 &other);
        ~MessageEngineTwcNOxInfo1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // class MessageEngineTwcNOxInfo1239

    class MessageEngineHevInfo1239 : public MessageEngineHevInfo
    {
    public:
        MessageEngineHevInfo1239();
        MessageEngineHevInfo1239(const MessageEngineHevInfo1239 &other);
        ~MessageEngineHevInfo1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // class MessageEngineHevInfo1239

    class MessageOBDInfo1239 : public MessageOBDInfo
    {
    public:
        MessageOBDInfo1239();
        MessageOBDInfo1239(const MessageOBDInfo1239 &other);
        ~MessageOBDInfo1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // class MessageOBDInfo1239

    class MessageReport1239 : public MessageReport
    {
    public:
        MessageReport1239();
        MessageReport1239(const MessageReport1239 &other);
        ~MessageReport1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // MessageReport1239

    class MessageVehicleLogin1239 : public MessageVehicleLogin
    {
    public:
        MessageVehicleLogin1239();
        MessageVehicleLogin1239(const MessageVehicleLogin1239 &other);
        ~MessageVehicleLogin1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // MessageVehicleLogin1239

    class MessageVehicleLogout1239 : public MessageVehicleLogout
    {
    public:
        MessageVehicleLogout1239();
        MessageVehicleLogout1239(const MessageVehicleLogout1239 &other);
        ~MessageVehicleLogout1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // MessageVehicleLogout1239

    class MessageVehicleActivate1239 : public MessageVehicleActivate
    {
    public:
        MessageVehicleActivate1239();
        MessageVehicleActivate1239(const MessageVehicleActivate1239 &other);
        ~MessageVehicleActivate1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // MessageVehicleActivate1239

    class MessageTamperAlarm1239 : public MessageTamperAlarm
    {
    public:
        MessageTamperAlarm1239();
        MessageTamperAlarm1239(const MessageTamperAlarm1239 &other);
        ~MessageTamperAlarm1239();

        SMLK_UINT32 Encode(std::vector<SMLK_UINT8> &data) const;
        SMLK_UINT32 Decode(IN std::vector<SMLK_UINT8> &data);
        std::shared_ptr<IMessage> Duplicate() const;
        std::size_t EncodedSize() const;
    }; // MessageTamperAlarm1239

};

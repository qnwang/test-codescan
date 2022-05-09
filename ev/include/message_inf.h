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
#include <memory>
#include "smlk_types.h"

#include "message_def_32960.h"

namespace smartlink {

#define PUSH_U4(v)  \
    data.push_back((SMLK_UINT8)(((v) >> 24) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v) >> 16) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v) >>  8) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v)      ) & 0xFF))

#define PUSH_U2(v)  \
    data.push_back((SMLK_UINT8)(((v) >>  8) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v)      ) & 0xFF))

#define PUSH_U1(v)   \
    data.push_back(v)

#define PUSH_AR(v)  \
    data.insert(data.end(), (v), (v) + sizeof(v))

    class IMessage {
    public:
        // IMessage() = default;

        virtual ~IMessage() {};

        /**
         * @brief 对TSP过来的数据进行解码
         * @param rawData    socket char*
         * @param inlength   消息总长度
         */
        virtual void decodeMsg(char *rawData, uint length) = 0;

        /**
         * @brief 对各业务类上报的消息对象进行统一编码处理
         * @param entity    socket char*
         * @param bodylength   消息内容的长度(由于消息内容每个业务是不一样的)
         */
        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length) = 0;

        /**
         * @brief 对各业务类上报的消息对象进行统一编码处理
         * @param entity    socket char*
         * @param bodylength   消息内容的长度(由于消息内容每个业务是不一样的)
         * @param result    返回的消息内容
         * @param msgLength  返回的消息体总长度
         */
        virtual void encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength) = 0;

        /**
         * @brief 将平台返回的消息(ack)广播
         * @param type  消息类型
         */
        virtual void broadcastMsgAck(uint msgId, uint rspFlag) = 0;


    };

};  // namespace smartlink

/******************************************************************************
*
*  Copyright (C) 2021 SmartLink
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

#include <cstring>
#include <smlk_log.h>

#include "message_heartbeat_32960.h"

#include "common_32960.h"

using namespace smartlink;

// int MessageHeartBeat32960::reTransTimes = 0;

// EV_Timer *MessageHeartBeat32960::heartbeat_timer = nullptr;

MessageHeartBeat32960::MessageHeartBeat32960() {}

void MessageHeartBeat32960::decodeMsg(char *rawData, uint length)
{
    if(rawData == NULL)
    {
        return;
    }
    SMLK_LOGD("[T] MessageHeartBeat32960::decodeMsg()");

    return;
}

void MessageHeartBeat32960::encodeMsg(GB_32960_MsgEntity entity, uint bodyLength) {
    MessageObserver32960::GetInstance()->encodeMsg(entity, bodyLength);
}

void MessageHeartBeat32960::encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength)
{
}

void MessageHeartBeat32960::broadcastMsgAck(uint msgId, uint rspFlag)
{
    if(msgId == static_cast<uint8_t>(msgID_32960::M_GB_32960_CMD_HEARTBEAT) && rspFlag == static_cast<uint8_t>(rspFlag_32960::M_GB_32960_RSP_FLAG_SUCCESS))
    {
        if(msgId == static_cast<uint8_t>(msgID_32960::M_GB_32960_CMD_HEARTBEAT))
        {
            SMLK_LOGD("[T] MessageHeartBeat32960 heart beat OK,stopTimer \n");
        }
        // this->stopHeartBeatTimer();
    }

    return ;
}

// int MessageHeartBeat32960::addHeartBeatTimer()
// {
//     if(heartbeat_timer == nullptr)
//     {
//         heartbeat_timer = new EV_Timer(1, 5, MessageHeartBeat32960::reSendTask); //3S一个
//     }
//     if(heartbeat_timer != nullptr)
//     {
//         TimerClient::getInstance()->add_timer(*heartbeat_timer);
//     }

//     return 0;
// }

// int MessageHeartBeat32960::stopHeartBeatTimer()
// {
//     if(heartbeat_timer != nullptr)
//     {
//         TimerClient::getInstance()->del_timer(*heartbeat_timer);
//         delete heartbeat_timer;
//         heartbeat_timer = nullptr;
//     }

//     return 0;
// }

// void MessageHeartBeat32960::reSendTask()
// {
//     SMLK_LOGD("[T] MessageHeartBeat32960 ==========MessageHeartBeat32960::reSendTask=========");

//     if(reTransTimes++ < 3)
//     {
//         int body_len = 0;
//         int reSend_len = 0;

//         GB_32960_MsgEntity m_MessageEntity32960;

//         memset(&m_MessageEntity32960, 0, sizeof(m_MessageEntity32960));

//         m_MessageEntity32960.msgId = static_cast<int>(msgID_32960::M_GB_32960_CMD_HEARTBEAT);
//         m_MessageEntity32960.rspFlag = static_cast<int>(rspFlag_32960::M_GB_32960_RSP_FLAG_CMD);
//         m_MessageEntity32960.encryptFlag = static_cast<int>(encryType_32960::M_GB_32960_ENCRYPTION_NONE);

//         if(common32960::getInstance()->m_AuthSignState)
//         {
//             reSend_len = MessageObserver32960::GetInstance()->encodeMsg(m_MessageEntity32960, body_len);
//         }
//         SMLK_LOGD("[T] MessageHeartBeat32960 reSend_len=%d", reSend_len);
//         SMLK_LOGD("[T] MessageHeartBeat32960 resend reTransTimes=%d\n", reTransTimes);
//     }
//     else
//     {
//         reTransTimes = 0;
//         SMLK_LOGD("[T] MessageHeartBeat32960 Message HeartBeat Over 3 times, TSP not Ack..\n");
//         //关
//         //stopTimer(0xFF);
//         //关闭socket
//         common32960::getInstance()->m_TcpClient32960->close();
//     }
// }


void MessageHeartBeat32960::DoHeartBeatUp()
{
    int body_len = 0;

    memset(&m_MessageEntity32960, 0, sizeof(m_MessageEntity32960));

    m_MessageEntity32960.msgId = static_cast<int>(msgID_32960::M_GB_32960_CMD_HEARTBEAT);
    m_MessageEntity32960.rspFlag = static_cast<int>(rspFlag_32960::M_GB_32960_RSP_FLAG_CMD);
    m_MessageEntity32960.encryptFlag = static_cast<int>(encryType_32960::M_GB_32960_ENCRYPTION_NONE);

    SMLK_LOGD("[T] MessageHeartBeat32960 send heartbeat to tsp");
    encodeMsg(m_MessageEntity32960, body_len);
    // addHeartBeatTimer();
}


void MessageHeartBeat32960::run() {
    common32960::getInstance()->GetThreadPool()->commit(heartBeatUpLoop, this);
}

void MessageHeartBeat32960::heartBeatUpLoop(void *arg) {
    int heart_beat_time;
    int count = 0;
    MessageHeartBeat32960 *HeartBeatUpPtr = (MessageHeartBeat32960 *)arg;

    heart_beat_time = 60;
    SMLK_LOGD("[T] Reporter::HeartBeatLoop() heart_beat_time=%d", heart_beat_time);

    while(1) {
        if(++count % heart_beat_time == 0) {
            if (common32960::getInstance()->getSocketState() == normal && common32960::getInstance()->getIgnState() && common32960::getInstance()->m_AuthSignState) {
                count = 0;
                HeartBeatUpPtr->DoHeartBeatUp();
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
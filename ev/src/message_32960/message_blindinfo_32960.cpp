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

#include "message_blindinfo_32960.h"

#include "common_32960.h"

using namespace smartlink;

char MessageBlindInfoUp32960::buf[M_GB_32960_SZ_DATA_MAX] = {0};
int MessageBlindInfoUp32960::bufLength = 0;
extern connectState m_connectState;

MessageBlindInfoUp32960::MessageBlindInfoUp32960() {}

void MessageBlindInfoUp32960::decodeMsg(char *rawData, uint inlength) {
    if (rawData == NULL) {
        return;
    }
    SMLK_LOGD("[T] MessageBlindInfoUp32960::decodeMsg");

    return;
}

void MessageBlindInfoUp32960::encodeMsg(GB_32960_MsgEntity entity, uint length) {
}

void MessageBlindInfoUp32960::encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength) {

}

void MessageBlindInfoUp32960::broadcastMsgAck(uint msgId, uint rspFlag) {
    //SMLK_LOGD("[T] =======MessageBlindInfoUp32960::broadcastMsgAck in msgId=%d rspFlag=%d=========\n",msgId,rspFlag);
    if (msgId == static_cast<uint8_t>(msgID_32960::M_GB_32960_CMD_REISSUE_REPORT) &&
        rspFlag == static_cast<uint8_t>(rspFlag_32960::M_GB_32960_RSP_FLAG_SUCCESS)) {
        SMLK_LOGD("[T] =======MessageBlindInfoUp32960:: send reissue info ok.\n");
    }  
}

void MessageBlindInfoUp32960::run() {
    common32960::getInstance()->GetThreadPool()->commit(blindDataUpLoop);
}

/**
 * @brief  若网络状态恢复正常，则将盲区数据从数据库中取出并上传
 */
void MessageBlindInfoUp32960::blindDataUpLoop() {
    SMLK_LOGD("[T] MessageBlindInfoUp32960:: blindDataUpLoop run");

    while (1) {
        sleep(1);

        //3级告警期间不处理
        if (common32960::getInstance()->getThreeWarnFlag()) {
            continue;
        }

        if (common32960::getInstance()->getSocketState() == normal && common32960::getInstance()->getIgnState() && common32960::getInstance()->m_AuthSignState) {
            common32960::getInstance()->m_SqlLiteBase32960->ReadBlindData(buf, bufLength);
            SMLK_LOGD("[T] blindDataUpLoop bufLength=%d",bufLength);

            if (bufLength > 0) {
                int ret = MessageObserver32960::GetInstance()->resendMsg(buf, bufLength);
                if (ret > 0) {
                    SMLK_LOGD("[T] UpdateMsgFlag");
                    common32960::getInstance()->m_SqlLiteBase32960->UpdateMsgFlag();
                }
            }
        }
    }
}

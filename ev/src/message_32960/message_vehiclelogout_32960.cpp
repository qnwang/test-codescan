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

#include "message_vehiclelogout_32960.h"
#include "common_32960.h"

using namespace smartlink;

MessageVehLogout32960::MessageVehLogout32960() {}

MessageVehLogout32960::~MessageVehLogout32960() {}

void MessageVehLogout32960::decodeMsg(char *rawData, uint inlength) {
    SMLK_LOGD("[T] MessageVehLogout32960::decodeMsg() enter");
    if (rawData == NULL) {
        return;
    }
    return;
}

void MessageVehLogout32960::encodeMsg(GB_32960_MsgEntity entity, uint length) {
    MessageObserver32960::GetInstance()->encodeMsg(entity, length);
}

void MessageVehLogout32960::encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength) {
}

void MessageVehLogout32960::broadcastMsgAck(uint msgId, uint rspFlag) {
    if (msgId == static_cast<uint8_t>(msgID_32960::M_GB_32960_CMD_VEHICLE_LOGOUT) &&
        rspFlag == static_cast<uint8_t>(rspFlag_32960::M_GB_32960_RSP_FLAG_SUCCESS)) {
        SMLK_LOGW(" ============ Logout success ============ ");
        common32960::getInstance()->m_TcpClient32960->close();

        //DTC故障代码
        common32960::getInstance()->setDtcErrCode(6);
    }
}

void MessageVehLogout32960::DoVehLogout() {
    SMLK_LOGD("[T] enter");

    std::vector<SMLK_UINT8> data;
    data.clear();

    memset(&m_MessageEntity32960, 0, sizeof(m_MessageEntity32960));
    memset(&m_vehicleLogoutData, 0, sizeof(m_vehicleLogoutData));

    m_MessageEntity32960.msgId = static_cast<SMLK_UINT8>(msgID_32960::M_GB_32960_CMD_VEHICLE_LOGOUT);
    m_MessageEntity32960.rspFlag = static_cast<SMLK_UINT8>(rspFlag_32960::M_GB_32960_RSP_FLAG_CMD);
    m_MessageEntity32960.encryptFlag = static_cast<SMLK_UINT8>(encryType_32960::M_GB_32960_ENCRYPTION_NONE);

    common32960::getInstance()->getTimestamp(m_vehicleLogoutData.timestamp);
    SMLK_LOGD("[T] m_vehicleLogoutData.timestamp = %d-%d-%d %d:%d:%d", m_vehicleLogoutData.timestamp[0],
              m_vehicleLogoutData.timestamp[1], m_vehicleLogoutData.timestamp[2],
              m_vehicleLogoutData.timestamp[3], m_vehicleLogoutData.timestamp[4],
              m_vehicleLogoutData.timestamp[5]);

    m_vehicleLogoutData.sequence = common32960::getInstance()->getSequence(false);
    SMLK_LOGD("[T] m_vehicleLogoutData.sequce = %d", m_vehicleLogoutData.sequence);

    PUSH_AR(m_vehicleLogoutData.timestamp);
    PUSH_U2(m_vehicleLogoutData.sequence);

    SMLK_LOGD("[T] data.size = %d", data.size());

    SMLK_UINT8 arr[data.size()];
    std::copy(data.begin(), data.end(), arr);

    m_MessageEntity32960.dataBody = arr;
    encodeMsg(m_MessageEntity32960, data.size());
}
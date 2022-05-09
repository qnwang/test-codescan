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

#include "common_32960.h"

#include "message_vehiclelogin_32960.h"

using namespace smartlink;

static MessageVehicleLogin32960 m_ploginObj;
EV_Timer *MessageVehicleLogin32960::loginTimer = nullptr;

MessageVehicleLogin32960::MessageVehicleLogin32960() {}

MessageVehicleLogin32960::~MessageVehicleLogin32960() {}

void MessageVehicleLogin32960::decodeMsg(char *rawData, uint inlength) {
    SMLK_LOGD("[T] MessageVehicleLogin32960::decodeMsg() enter");
    if (rawData == NULL) {
        return;
    }
    return;
}

void MessageVehicleLogin32960::encodeMsg(GB_32960_MsgEntity entity, uint length) {
    SMLK_LOGD("[T] MessageVehicleLogin32960::encodeMsg() enter");
    MessageObserver32960::GetInstance()->encodeMsg(entity, length);
}

void MessageVehicleLogin32960::encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength)
{
}

void MessageVehicleLogin32960::broadcastMsgAck(uint msgId, uint rspFlag)
{
    // SMLK_LOGD("[T] MessageVehicleLogin32960::broadcastMsgAck");
    if (msgId == static_cast<uint8_t>(msgID_32960::M_GB_32960_CMD_VEHICLE_LOGIN))
    {

        if (rspFlag == static_cast<uint8_t>(rspFlag_32960::M_GB_32960_RSP_FLAG_SUCCESS))
        {
            SMLK_LOGW(" ============ Login success ============ ");

            common32960::getInstance()->m_AuthSignState = true;
            m_ploginObj.curTryNum = 0;
            m_ploginObj.logining = false;
            m_ploginObj.nextLoginFlag = false;
            common32960::getInstance()->saveSequence();

            delLoginTimer();

            // DTC故障代码
            common32960::getInstance()->setDtcErrCode(6);
        }
        else
        {
            // DTC故障代码
            common32960::getInstance()->setDtcErrCode(5);
        }
    }
}

void MessageVehicleLogin32960::DoVehicleLogin() {

    if (checkLoginTimer() != 0) {
        SMLK_LOGD("in logining...");
        return;
    }
    SMLK_LOGD("do login");

    std::vector<SMLK_UINT8> data;
    data.clear();

    memset(&m_MessageEntity32960, 0, sizeof(m_MessageEntity32960));
    memset(&m_vehicleLoginData, 0, sizeof(m_vehicleLoginData));

    m_MessageEntity32960.msgId = static_cast<SMLK_UINT8>(msgID_32960::M_GB_32960_CMD_VEHICLE_LOGIN);
    m_MessageEntity32960.rspFlag = static_cast<SMLK_UINT8>(rspFlag_32960::M_GB_32960_RSP_FLAG_CMD);
    m_MessageEntity32960.encryptFlag = static_cast<SMLK_UINT8>(encryType_32960::M_GB_32960_ENCRYPTION_NONE);

    common32960::getInstance()->getTimestamp(m_vehicleLoginData.timestamp);
    SMLK_LOGD("[T] m_vehicleLoginData.timestamp = %d-%d-%d %d:%d:%d", m_vehicleLoginData.timestamp[0],
              m_vehicleLoginData.timestamp[1], m_vehicleLoginData.timestamp[2],
              m_vehicleLoginData.timestamp[3], m_vehicleLoginData.timestamp[4],
              m_vehicleLoginData.timestamp[5]);

    // sequce 登入流水号需计算
    m_vehicleLoginData.sequence                     = common32960::getInstance()->getSequence(true);
    SMLK_LOGD("[T] m_vehicleLoginData.sequce = %d", m_vehicleLoginData.sequence);

    //获取 iccid并赋值给结构体中对应的iccid项, iccid 从第10个字节开始
    memcpy(m_vehicleLoginData.iccid, common32960::getInstance()->m_iccid, M_GB_32960_SZ_ICCID);
    SMLK_LOGD("[T] m_vehicleLoginData.iccid = %s", m_vehicleLoginData.iccid);

    // vehicle::SignalVariant m_baty_sys_count         = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::RechargeableEnergyStorageSubsystemsQuantity)->second;
    // vehicle::SignalVariant m_baty_code_len          = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::RechargeableEnergyStorageSystemCodeLength)->second;

    // m_vehicleLoginData.baty_sys_cnt                 = m_baty_sys_count.Valid() ? m_baty_sys_count.Encoded() : 0xFF;
    m_vehicleLoginData.baty_sys_cnt                 = M_GB_32960_SZ_RECHARGE_DEVICE_SUBSYS_CNT;
    m_vehicleLoginData.baty_code_len                = 0;
    SMLK_LOGD("[T] m_vehicleLoginData.baty_sys_cnt = %d", m_vehicleLoginData.baty_sys_cnt);
    SMLK_LOGD("[T] m_vehicleLoginData.baty_code_len = %d", m_vehicleLoginData.baty_code_len);

    PUSH_AR(m_vehicleLoginData.timestamp);
    PUSH_U2(m_vehicleLoginData.sequence);
    PUSH_AR(m_vehicleLoginData.iccid);
    PUSH_U1(m_vehicleLoginData.baty_sys_cnt);
    PUSH_U1(m_vehicleLoginData.baty_code_len);

    SMLK_LOGD("[T] data.size = %d", data.size());

    SMLK_UINT8 arr[data.size()];
    std::copy(data.begin(), data.end(), arr);

    m_MessageEntity32960.dataBody = arr;
    encodeMsg(m_MessageEntity32960, data.size());
}

void MessageVehicleLogin32960::loginInit() {
    m_ploginObj.logining = false;

    m_ploginObj.curTryNum = 0;
    m_ploginObj.nextLoginIntervalTime = 0;
    m_ploginObj.curWaitTime = 0;
    m_ploginObj.nextLoginFlag = false;
    m_ploginObj.nextLoginWaitTime = 0;
    m_ploginObj.maxWaitTime = 10;
    SMLK_LOGD("[T] m_ploginObj.maxWaitTime=%d", m_ploginObj.maxWaitTime);
    if (m_ploginObj.maxWaitTime <= 0) {
        m_ploginObj.maxWaitTime = 10;
    }
}

int MessageVehicleLogin32960::checkLoginTimer() {
    if (!m_ploginObj.logining) {
        loginInit();
        addLoginTimer();
        return 0;
    } else {
        return -1;
    }
}

int MessageVehicleLogin32960::addLoginTimer() {
    if (loginTimer == nullptr) {
        loginTimer = new EV_Timer(1, 1, loginLoop);
    }
    if (loginTimer != nullptr) {
        if (TimerClient::getInstance()->chekExist_timer(*loginTimer) == 0) {
            SMLK_LOGD("[T] RTInfoUp32960::addLoginTimer....");
            TimerClient::getInstance()->add_timer(*loginTimer);
        }
    }
    return 0;
}

void MessageVehicleLogin32960::delLoginTimer() {
    m_ploginObj.logining = false;

    if (loginTimer != nullptr) {
        if (TimerClient::getInstance()->chekExist_timer(*loginTimer) != 0) {
            SMLK_LOGD("[T] RTInfoUp32960::delLoginTimer....");
            TimerClient::getInstance()->del_timer(*loginTimer);
            delete loginTimer;
            loginTimer = nullptr;
        }
    }
}

/**
 * @brief loginLoop, 计算登录所需的时间及重试机制
 */
void MessageVehicleLogin32960::loginLoop() {
    // SMLK_LOGD("[T] RTInfoUp32960::loginLoop");
    // SMLK_LOGD("[T] m_ploginObj.curlogin=%d", m_ploginObj.curTryNum);
    // SMLK_LOGD("[T] m_ploginObj.curWaitTime=%d", m_ploginObj.curWaitTime + 1);
    // SMLK_LOGD("[T] m_ploginObj.maxWaitTime=%d", m_ploginObj.maxWaitTime);
    // SMLK_LOGD("[T] m_ploginObj.nextLoginFlag=%d", m_ploginObj.nextLoginFlag);

    SMLK_LOGD("[T] curlogin=%d curWaitTime=%d maxWaitTime=%d nextLoginFlag=%d", m_ploginObj.curTryNum, m_ploginObj.curWaitTime + 1, m_ploginObj.maxWaitTime, m_ploginObj.nextLoginFlag);
    
    if (m_ploginObj.logining == false) {
        //SMLK_LOGD("[T] RTInfoUp32960::logining true");
        m_ploginObj.logining = true;
        m_ploginObj.curTryNum++;
        m_ploginObj.curWaitTime = 0;
        m_ploginObj.nextLoginWaitTime = 0;
    }

    if (m_ploginObj.curTryNum <= 3) {
        if (++m_ploginObj.curWaitTime > m_ploginObj.maxWaitTime)//超时
        {
            m_ploginObj.nextLoginIntervalTime = 60;  //国标1min后尝试登录
            m_ploginObj.nextLoginFlag = true;

            //DTC故障代码
            common32960::getInstance()->setDtcErrCode(5);
        }
    } else {
        if (++m_ploginObj.curWaitTime > m_ploginObj.maxWaitTime) {
            m_ploginObj.nextLoginIntervalTime = 30 * 60; //国标30min后尝试登录
            m_ploginObj.nextLoginFlag = true;

            //DTC故障代码
            common32960::getInstance()->setDtcErrCode(5);
        }
    }

    if (m_ploginObj.nextLoginFlag) {
        SMLK_LOGD("[T] m_ploginObj.nextLoginWaitTime=%d", m_ploginObj.nextLoginWaitTime + 1);
        if (++m_ploginObj.nextLoginWaitTime > m_ploginObj.nextLoginIntervalTime) //开始下一次登录
        {
            if (m_ploginObj.nextLoginIntervalTime >= (30 * 60))   //重新连接
            {
                SMLK_LOGD("[T] login timeout,reconnect");
                common32960::getInstance()->m_TcpClient32960->close();

                //DTC故障代码
                common32960::getInstance()->setDtcErrCode(5);
            }
            m_ploginObj.logining = false;
            m_ploginObj.nextLoginFlag = false;
            delLoginTimer();
        }
    }
}


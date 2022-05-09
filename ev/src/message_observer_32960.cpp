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
#include "message_observer_32960.h"

using namespace smartlink;

MessageObserver32960 *MessageObserver32960::m_instance = NULL;
std::mutex MessageObserver32960::m_mtx;

MessageObserver32960 *MessageObserver32960::GetInstance() {
    if (m_instance == NULL) {
        m_mtx.lock();
        if (m_instance == NULL)
            m_instance = new MessageObserver32960();
        m_mtx.unlock();
    }
    return m_instance;
}

/**
* @brief   析构
*/
MessageObserver32960::~MessageObserver32960() {
    DetachObserver();
}

/**
 * @brief 大小端转换
 * @param pData 待转换数据
 * @param startIndex 起始位置
 * @param length 转换字节数
 */
void MessageObserver32960::EndianSwap(uint8_t *pData, int startIndex, int length) {
    int i, cnt, end, start;
    if (length == 1) return;
    cnt = length / 2;
    start = startIndex;
    end = startIndex + length - 1;
    uint8_t tmp;
    for (i = 0; i < cnt; i++) {
        tmp = pData[start + i];
        pData[start + i] = pData[end - i];
        pData[end - i] = tmp;
    }
}

/**
 * @brief 异或校验 x与0异或==x, x与1异或==取反
 * @param data 待校验数
 * @param len 待校验数据长度
 */
uint8_t FCS_calc(const uint8_t *data, size_t len) {
    uint8_t bcc = *(uint8_t *) data;
    uint8_t *p = (uint8_t *) data + 1;
    if (NULL == data || len <= 0) {
        return 0;
    }
    while (--len) {
        bcc ^= *p;
        p++;
    }
    return bcc;
}

/**
* @brief              解TSP数据包
* @param src          总buf
* @param length       总长度
*/
void MessageObserver32960::decodeMsg(char *src, uint length)
{
    if (common32960::getInstance()->m_todisk) {
        std::stringstream strStream;
        time_t timep;
        struct tm *p;
        time(&timep);
        p = localtime(&timep);
        struct timeval ms;
        gettimeofday(&ms, NULL);
        strStream << "receive time:" << p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
        for (int i = 0; i < length; i++) {
            strStream << std::hex << std::setw(2) << std::setfill('0') << (int)src[i] << " ";
        }
        system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/ev/record_gb32960.log").c_str());
    }
    int i = 0;
    bool is_mutidata = false;
    for (i = 0 + 2; i < length - 1; i++) {
        if ((int)src[i] == 0x23 && (int)src[i + 1] == 0x23) {
            is_mutidata = true;
            break;
        }
    }
    if (is_mutidata) {
        length = i;
        SMLK_LOGD("[T] MessageObserver32960::decodeMsg() receive length = %0d", length);
    }

    //解出MessageID，根据ID 把方法体的内容同步给具体的业务类
    GB_32960_MsgEntity entity;
    uint bodyLen = 0;
    bodyLen = length - sizeof(GB_32960_MsgEntity) + 4;

    memset(&entity, 0, sizeof(entity));

    //head
    if ((*src == '#') && (*(src + 1) == '#')) {
        uint8_t checkByte = FCS_calc((unsigned char *) src + 2, length - 3);
        if ((uint8_t) src[length - 1] == checkByte) {
            entity.msgId = *(src + 2);
            SMLK_LOGD("[T] MessageObserver32960::decodeMsg() msgId=%0d", entity.msgId);
            entity.rspFlag = *(src + 3);
            SMLK_LOGD("[T] MessageObserver32960::decodeMsg() rspFlag=%0d", entity.rspFlag);

            memcpy(&entity.vin, src + 4, 17);
            SMLK_LOGD("[T] MessageObserver32960::decodeMsg() vin=%s", entity.vin);
            entity.encryptFlag = *(src + 21);
            SMLK_LOGD("[T] MessageObserver32960::decodeMsg() encryptFlag=%0d", entity.encryptFlag);
            memcpy((void *) &entity.dataLength, src + 22, 2);
            EndianSwap((u_char * ) & entity.dataLength, 0, 2);
            SMLK_LOGE("[T] MessageObserver32960::decodeMsg() dataLength=%d,bodyLen=%d", entity.dataLength, bodyLen);
            if (entity.dataLength == bodyLen) {
                // 通知各业务去处理;
                SMLK_LOGD("[T] MessageObserver32960::decodeMsg() Notify Observer begin , msgId %02x", entity.msgId);

                if (bodyLen > 0) {
                    entity.dataBody = nullptr;
                    entity.dataBody = new char[bodyLen];
                    if (entity.dataBody != nullptr) {
                        memcpy(entity.dataBody, src + 24, bodyLen);
                    }
                }
                this->NotifyObserver((msgID_32960) entity.msgId, (char *) entity.dataBody, bodyLen);

                // 广播收到的命令id
                this->NotifyMsgAck(entity.msgId, entity.rspFlag);

                // 释放已经用完的dataBody
                if (entity.dataBody != nullptr) {
                    delete[]entity.dataBody;
                    entity.dataBody = nullptr;
                }

                SMLK_LOGE("[T] MessageObserver32960::decodeMsg() Decode SUCCESS END \n");
            } else {
                SMLK_LOGE("[T] MessageObserver32960::decodeMsg() Decode ERROR Body Length num \n");
            }
        } else {
            SMLK_LOGE("[T] MessageObserver32960::decodeMsg() Decode ERROR check num \n");
        }
    } else {
        SMLK_LOGE("[T] MessageObserver32960::decodeMsg() Decode ERROR HEAD FLAG \n");
    }

    return;
}

/**
* @brief              编码(不需要重发的业务 推荐使用此接口)
* @param entity       业务类上报需要发送的数据模型
* @param bodyLength   方法内容的长度
*/
int MessageObserver32960::encodeMsg(GB_32960_MsgEntity entity, uint bodyLength) {
    SMLK_LOGD("[T] MessageObserver32960::encodeMsg() MessageObserver32960::encodeMsg , bodylength %d", bodyLength);
    int ret = -1;

    // 起始符
    memcpy(&entity.startFlag, "##", 2);
    // 车辆识别号
    memcpy(entity.vin, common32960::getInstance()->m_vin, M_GB_32960_SZ_VIN);
    // SMLK_LOGD("[T] MessageObserver32960::encodeMsg() entity.vin=%s", entity.vin);
    // 数据加密方式: 0x01, 不加密
    entity.encryptFlag = 0x01;
    // SMLK_LOGD("[T] MessageObserver32960::encodeMsg() entity.encryptFlag=%d", entity.encryptFlag);

    entity.dataLength = bodyLength;
    EndianSwap((uint8_t * ) & entity.dataLength, 0, 2);
    char result[HEADLENTH + bodyLength + 1] = {0};
    memcpy(result, (char *) &entity, HEADLENTH);
    if (bodyLength > 0) {
        memcpy(result + HEADLENTH, entity.dataBody, bodyLength);
    }
    entity.checkCode = FCS_calc(result + 2, sizeof(result) - 3);
    result[HEADLENTH + bodyLength] = entity.checkCode;
    SMLK_LOGD("[T] MessageObserver32960::encodeMsg() CRC=%02x", entity.checkCode);

    ret = this->m_callBack(result, sizeof(result));
    SMLK_LOGD("[T] MessageObserver32960::encodeMsg() sizeof result=  %d, send ret = %d", sizeof(result), ret);

    if ((ret > 0) && (common32960::getInstance()->m_todisk)) {
        std::stringstream strStream;
        time_t timep;
        struct tm *p;
        time(&timep);
        p = localtime(&timep);
        struct timeval ms;
        gettimeofday(&ms, NULL);
        strStream << "send time:" << p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
        for (int i = 0; i < sizeof(result); i++) {
            strStream << std::hex << std::setw(2) << std::setfill('0') << (int)result[i] << " ";
        }
        system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/ev/record_gb32960.log").c_str());
    }

    return ret;
}

/**
 * @brief               编码(需要重发的业务 推荐使用此接口)
 * @param entity        业务类上报需要发送的数据模型
 * @param bodyLength    方法内容的长度
 * @param dest          总buf
 * @param msgLength     总长度
 * @return
 */
int MessageObserver32960::encodeMsg(GB_32960_MsgEntity entity, uint bodyLength, char *dest, uint &msgLength) {
    SMLK_LOGD("[T] MessageObserver32960::encodeMsg() ");
    int ret = -1;

    // 起始符
    memcpy(&entity.startFlag, "##", 2);
    // 车辆识别号
    memcpy(entity.vin, common32960::getInstance()->m_vin, M_GB_32960_SZ_VIN);
    // SMLK_LOGD("[T] MessageObserver32960::encodeMsg() entity.vin=%s", entity.vin);
    // 数据加密方式: 0x01, 不加密
    entity.encryptFlag = 0x01;
    // SMLK_LOGD("[T] MessageObserver32960::encodeMsg() entity.encryptFlag=%d", entity.encryptFlag);
    entity.dataLength = bodyLength;
    EndianSwap((uint8_t * ) & entity.dataLength, 0, 2);
    char result[HEADLENTH + bodyLength + 1] = {0};
    memcpy(result, (char *) &entity, HEADLENTH);
    if (bodyLength > 0) {
        memcpy(result + HEADLENTH, entity.dataBody, bodyLength);
    }
    entity.checkCode = FCS_calc(result + 2, sizeof(result) - 3);
    result[HEADLENTH + bodyLength] = entity.checkCode;
    SMLK_LOGD("[T] MessageObserver32960::encodeMsg() CRC=%02x", entity.checkCode);
    //调用端 出参
    int totalLen = HEADLENTH + bodyLength + 1;
    memcpy(dest, result, totalLen);
    msgLength = totalLen;
    
    if (common32960::getInstance()->m_AuthSignState) {
        // SMLK_LOGD("[T] MessageObserver32960::encodeMsg() common32960::getInstance()->m_AuthSignState =  %d", common32960::getInstance()->m_AuthSignState);
        ret = this->m_callBack(result, sizeof(result));
        SMLK_LOGD("[T] MessageObserver32960::encodeMsg() sizeof result= %d, send ret = %d", sizeof(result), ret);
        
        if ((ret > 0) && (common32960::getInstance()->m_todisk)) {
            std::stringstream strStream;
            time_t timep;
            struct tm *p;
            time(&timep);
            p = localtime(&timep);
            struct timeval ms;
            gettimeofday(&ms, NULL);
            strStream << "send time:" << p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
            for (int i = 0; i < sizeof(result); i++) {
                strStream << std::hex << std::setw(2) << std::setfill('0') << (int)result[i] << " ";
            }
            system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/ev/record_gb32960.log").c_str());
        }
    }
    return ret;
}


/**
* @brief      注册回调的指针，同步这个回调给task发socket
* @param cb   callback function
*/
void MessageObserver32960::RegisterDataSendCB(msgSendCallback32960 cb) {
    this->m_callBack = cb;
}


/**
* @brief                 注册消息的观察业务类
* @param id              业务类id
* @param MessageOb       业务类object
*/
void MessageObserver32960::AttachObserver(msgID_32960 id, IMessage *MessageOb) {
    // SMLK_LOGD("[T] MessageObserver32960::decodeMsg() MessageObserver32960::AttachObserver");
    ServerVec.emplace_back(make_pair(id, MessageOb));
}

void MessageObserver32960::DetachObserver() {
    for (auto iter = ServerVec.begin(); iter != ServerVec.end(); iter++) {
        ServerVec.erase(iter);
    }
}


/**
* @brief                 解析出消息id，将内容发送过去
* @param id              业务类id
* @param messageContent  消息内容
* @param len             消息长度
*/
void MessageObserver32960::NotifyObserver(msgID_32960 id, char *messageContent, uint len) {
    for (auto iter = ServerVec.begin(); iter != ServerVec.end(); iter++) {
        //  SMLK_LOGD("[T] MessageObserver32960::decodeMsg() iter->first=%02x",iter->first);
        if (iter->first == id) {
            SMLK_LOGD("[T] MessageObserver32960::decodeMsg() NotifyObserver len=%d", len);
            iter->second->decodeMsg(messageContent, len);
        }
    }
}

/**
* @brief        广播ack
* @param type   命令id（因为现在没有标识来区分 哪个消息的ack，所以只有根据包里的命令id来判断）
*/
void MessageObserver32960::NotifyMsgAck(uint msgId, uint rspFlag) {
    SMLK_LOGD("[T] MessageObserver32960::decodeMsg() MessageObserver32960::NotifyMsgAck msgId = %d", msgId);
    for (auto iter = ServerVec.begin(); iter != ServerVec.end(); iter++) {
        iter->second->broadcastMsgAck(msgId, rspFlag);
    }
}

int MessageObserver32960::resendMsg(char *buf, uint len) {
    buf[0] = M_GB_32960_MSG_FLAG;
    buf[1] = M_GB_32960_MSG_FLAG;
    buf[2] = static_cast<SMLK_UINT8>(msgID_32960::M_GB_32960_CMD_REISSUE_REPORT);
    buf[3] = static_cast<SMLK_UINT8>(rspFlag_32960::M_GB_32960_RSP_FLAG_CMD);

    buf[len - 1] = FCS_calc(buf + 2, len - 3);

    int ret = -1;
    ret = this->m_callBack(buf, len);
    SMLK_LOGD("[T] MessageObserver32960::resendMsg() len = %d, send ret = %d", len, ret);

    if ((ret > 0) && (common32960::getInstance()->m_todisk)) {
        std::stringstream strStream;
        time_t timep;
        struct tm *p;
        time(&timep);
        p = localtime(&timep);
        struct timeval ms;
        gettimeofday(&ms, NULL);
        strStream << "send time:" << p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec << "." << ms.tv_usec << " " << std::endl;
        for (int i = 0; i < len; i++) {
            strStream << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i] << " ";
        }
        system(("echo \"" + strStream.str() + "\" >> /userdata/smartlink/record/ev/record_gb32960.log").c_str());
    }

    return ret;
}

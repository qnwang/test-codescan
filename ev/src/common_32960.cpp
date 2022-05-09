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

#include <smlk_log.h>
#include <smlk_tools.h>

#include "common_32960.h"

using namespace smartlink;

common32960 *common32960::m_instance = NULL;
std::mutex common32960::m_mtx;

/**
* @brief    构造
*/
common32960::common32960() {
    m_AuthSignState = false;
    m_SocketState = -1;
    sequence_login = 0;
    sequence_infoUp = 0;
    m_ignState = 0;
    m_three_warn_level = false;
    m_run_mode = 0x01;
    m_todisk = false;
    m_last_val = 0;

    m_execThread = std::make_shared<ThreadPool>(50);

    std::memset(&m_vin, 0, M_GB_32960_SZ_VIN);
    std::memset(&m_iccid, 0, M_GB_32960_SZ_ICCID);

    memset(&m_dm1info, 0, sizeof(m_dm1info));

    m_tempinfo = {};
    m_voltinfo = {};
}


common32960::~common32960(){
}

/**
* @brief    单例
*/
common32960 *common32960::getInstance() {
    if (m_instance == NULL) {
        m_mtx.lock();
        if (m_instance == NULL)
            m_instance = new common32960();
        m_mtx.unlock();
    }

    return m_instance;
}

std::shared_ptr <ThreadPool> common32960::GetThreadPool() {
    return m_execThread;
}

/**
* @brief        设置socket连接状态
* @param state  socket连接状态
*/
void common32960::setSocketState(int state) {
    SMLK_LOGD("[T] common32960::setSocketState() %d ", state);
    if (m_SocketState != state) {
        m_SocketState = state;
    }
}

/**
* @brief   获取socket连接状态
*/
int common32960::getSocketState() {
    SMLK_LOGD("[T] common32960::getSocketState() %d ", m_SocketState);
    return m_SocketState;
}


/**
* @brief  获取 32960数据上传的时间间隔
*/
int common32960::getInfoUpTime32960() {
    volatile SMLK_UINT16 upload_period = common32960::getInstance()->getConfigParam().upload_period;
    if (upload_period <= 0) {
        upload_period = 10;
    }
    //SMLK_LOGD("[T] upload_period=%d", upload_period);

    return upload_period;
}

/**
* @brief  设置 32960数据上传的时间间隔
*/
void common32960::setInfoUpTime32960(SMLK_UINT16 value) {
    SMLK_LOGD("[T] setInfoUpTime32960 value=%d", value);
    if (value > 0) {
        common32960::getInstance()->getConfigParam().upload_period_bak = common32960::getInstance()->getConfigParam().upload_period;
        common32960::getInstance()->getConfigParam().upload_period = value;
    }
}


/**
 * @brief 获取流水号
*/
int common32960::getSequence(bool b_in) {
    if (b_in) {
        return sequence_login;      //登入
    } else {
        return sequence_login - 1;  //登出
    }
}

/**
 * @brief 保存流水号到文件
*/
void common32960::saveSequence() {
    time_t timep;
    struct tm *p;
    time( &timep );
    p = localtime( &timep );
    std::stringstream strStream;
    strStream<<(p->tm_year + 1900)<< std::setw(2)<< std::setfill('0')<<(p->tm_mon + 1)<< std::setw(2)<< std::setfill('0')<<p->tm_mday;

    std::fstream seqFile;
    seqFile.open(M_GB_32960_CONFIG_DIR + (M_GB_32960_SEQUEMCE_LOGINHEAD + strStream.str()), ios::out);
    seqFile << ++sequence_login;
    seqFile.close();
}

/**
 * @brief 初始化流水号
*/
void common32960::initSequence() {
    SMLK_LOGD("[T] common32960::initSequence() common32960::initSequence");
    // 遍历文件夹
    std::fstream seqFile;
    string sLogSeqFileName = "";

    DIR *dir = opendir(M_GB_32960_CONFIG_DIR);
    dirent *d = NULL;
    while ((d = readdir(dir)) != NULL) {
        if (d->d_name[0] != '.') {
            if ((d->d_name)[0] == 'l') {
                sLogSeqFileName = string(d->d_name);
            }
        }
    }
    closedir(dir);

    SMLK_LOGD("[T] sLogSeqFileName=%s", sLogSeqFileName.c_str());

    time_t timep;
    struct tm *p;
    time( &timep );
    p = localtime( &timep );
    std::stringstream strStream;
    strStream<<(p->tm_year + 1900)<< std::setw(2)<< std::setfill('0')<<(p->tm_mon + 1)<< std::setw(2)<< std::setfill('0')<<p->tm_mday;

    if (sLogSeqFileName == "") {
        SMLK_LOGD("[T] sLogSeqFileName is null");
        // 创建文件  设置为1， 并且将sequence_login 赋值为1
        seqFile.open(M_GB_32960_CONFIG_DIR + (M_GB_32960_SEQUEMCE_LOGINHEAD + strStream.str()), ios::out);
        sequence_login = 1;
        seqFile << sequence_login;
    } else {
        SMLK_LOGD("[T] sLogSeqFileName no null");
        // 判断名字是否与当天一样
        if (M_GB_32960_SEQUEMCE_LOGINHEAD + strStream.str() == sLogSeqFileName) {
            //读出里面的值 赋值给并且将sequence_login
            seqFile.open(M_GB_32960_CONFIG_DIR + (M_GB_32960_SEQUEMCE_LOGINHEAD + strStream.str()), ios::in | ios::out);
            seqFile >> sequence_login;
            SMLK_LOGD("[T] sequence_login=%d", sequence_login);
            if (sequence_login > 65531) {
                sequence_login = 1;
                seqFile << sequence_login;
            }
        } else {
            //删除当前的文件，并创建一个文件， 并且将sequence_login 赋值为1
            string file_dir = M_GB_32960_CONFIG_DIR + sLogSeqFileName;
            remove(file_dir.c_str());
            seqFile.open(M_GB_32960_CONFIG_DIR + (M_GB_32960_SEQUEMCE_LOGINHEAD + strStream.str()), ios::out);
            sequence_login = 1;
            seqFile << sequence_login;
            SMLK_LOGD("[T] new day sequence_login=%d", sequence_login);
        }
    }
    seqFile.close();
}

/**
 * @brief  设置 IGN 状态
 * @param state  IGN 的值
*/
void common32960::setIgnState(int state) {
    if (m_ignState != state) {
        m_ignState = state;
    }
}

/**
 * @brief  获取IGN状态
*/
int common32960::getIgnState() {
    return m_ignState;
}

/**
 * @brief  获取3级告警级别
*/
bool common32960::getThreeWarnFlag() {
    return m_three_warn_level;
}

/**
 * @brief  设置3级告警级别
*/
void common32960::setThreeWarnFlag(bool value) {
    m_three_warn_level = value;
}


/**
 * @brief  初始化配置参数
 * No used
*/
void common32960::initConfigParam() {
    SMLK_LOGD("[T] common32960::initConfigParam()");
    struct PARAM_T param;

    param.warn_period = 1;
    param.upload_period = 10;
    param.upload_period_bak = 10;
    param.relogin_period = 60;
    
    setConfigParam(param);
}

/**
 * @brief  设置配置参数
 * @ param 参数结构体变量
*/
void common32960::setConfigParam(struct PARAM_T &param) {
    common32960::getInstance()->m_paramObj = param;
}

/**
 * @brief  获取配置参数
 * @ return 返回结构体变量
*/
struct PARAM_T &common32960::getConfigParam() {
    return m_paramObj;
}

/**
 * @brief  获取时间戳
 * @param
*/
void common32960::getTimestamp(SMLK_UINT8 *buffer) {
    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);

    buffer[0] = p->tm_year % 100;
    buffer[1] = p->tm_mon + 1;
    buffer[2] = p->tm_mday;
    buffer[3] = p->tm_hour;
    buffer[4] = p->tm_min;
    buffer[5] = p->tm_sec;
}

/**
 * @brief  设置Utc故障代码
*/
int common32960::setDtcErrCode(SMLK_UINT8 val)
{
    OBD::UdsDtcTriggerInfo dtc_info;

    if(val != m_last_val)
    {
        m_last_val = val;

        // DTC故障代码
        dtc_info.val = val;
        dtc_info.dtc_type = OBD::UdsDtcTriggerType::E_UDS_DTC_TRIGGER_GB_CONNECT_STATUS;
        OBD::DidIpcApi::getInstance()->SendDtcErrCode(dtc_info);
        SMLK_LOGD("[T] common32960::setDtcErrCode dtc_info.val=%d,dtc_info.dtc_type=%d", dtc_info.val,dtc_info.dtc_type);
    }

    return 0;
}

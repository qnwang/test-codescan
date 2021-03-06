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
* @brief    ζι 
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
* @brief    εδΎ
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
* @brief        θ?Ύη½?socketθΏζ₯ηΆζ
* @param state  socketθΏζ₯ηΆζ
*/
void common32960::setSocketState(int state) {
    SMLK_LOGD("[T] common32960::setSocketState() %d ", state);
    if (m_SocketState != state) {
        m_SocketState = state;
    }
}

/**
* @brief   θ·εsocketθΏζ₯ηΆζ
*/
int common32960::getSocketState() {
    SMLK_LOGD("[T] common32960::getSocketState() %d ", m_SocketState);
    return m_SocketState;
}


/**
* @brief  θ·ε 32960ζ°ζ?δΈδΌ ηζΆι΄ι΄ι
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
* @brief  θ?Ύη½? 32960ζ°ζ?δΈδΌ ηζΆι΄ι΄ι
*/
void common32960::setInfoUpTime32960(SMLK_UINT16 value) {
    SMLK_LOGD("[T] setInfoUpTime32960 value=%d", value);
    if (value > 0) {
        common32960::getInstance()->getConfigParam().upload_period_bak = common32960::getInstance()->getConfigParam().upload_period;
        common32960::getInstance()->getConfigParam().upload_period = value;
    }
}


/**
 * @brief θ·εζ΅ζ°΄ε·
*/
int common32960::getSequence(bool b_in) {
    if (b_in) {
        return sequence_login;      //η»ε₯
    } else {
        return sequence_login - 1;  //η»εΊ
    }
}

/**
 * @brief δΏε­ζ΅ζ°΄ε·ε°ζδ»Ά
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
 * @brief εε§εζ΅ζ°΄ε·
*/
void common32960::initSequence() {
    SMLK_LOGD("[T] common32960::initSequence() common32960::initSequence");
    // ιεζδ»Άε€Ή
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
        // εε»Ίζδ»Ά  θ?Ύη½?δΈΊ1οΌ εΉΆδΈε°sequence_login θ΅εΌδΈΊ1
        seqFile.open(M_GB_32960_CONFIG_DIR + (M_GB_32960_SEQUEMCE_LOGINHEAD + strStream.str()), ios::out);
        sequence_login = 1;
        seqFile << sequence_login;
    } else {
        SMLK_LOGD("[T] sLogSeqFileName no null");
        // ε€ζ­εε­ζ―ε¦δΈε½ε€©δΈζ ·
        if (M_GB_32960_SEQUEMCE_LOGINHEAD + strStream.str() == sLogSeqFileName) {
            //θ―»εΊιι’ηεΌ θ΅εΌη»εΉΆδΈε°sequence_login
            seqFile.open(M_GB_32960_CONFIG_DIR + (M_GB_32960_SEQUEMCE_LOGINHEAD + strStream.str()), ios::in | ios::out);
            seqFile >> sequence_login;
            SMLK_LOGD("[T] sequence_login=%d", sequence_login);
            if (sequence_login > 65531) {
                sequence_login = 1;
                seqFile << sequence_login;
            }
        } else {
            //ε ι€ε½εηζδ»ΆοΌεΉΆεε»ΊδΈδΈͺζδ»ΆοΌ εΉΆδΈε°sequence_login θ΅εΌδΈΊ1
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
 * @brief  θ?Ύη½? IGN ηΆζ
 * @param state  IGN ηεΌ
*/
void common32960::setIgnState(int state) {
    if (m_ignState != state) {
        m_ignState = state;
    }
}

/**
 * @brief  θ·εIGNηΆζ
*/
int common32960::getIgnState() {
    return m_ignState;
}

/**
 * @brief  θ·ε3ηΊ§εθ­¦ηΊ§ε«
*/
bool common32960::getThreeWarnFlag() {
    return m_three_warn_level;
}

/**
 * @brief  θ?Ύη½?3ηΊ§εθ­¦ηΊ§ε«
*/
void common32960::setThreeWarnFlag(bool value) {
    m_three_warn_level = value;
}


/**
 * @brief  εε§ειη½?εζ°
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
 * @brief  θ?Ύη½?ιη½?εζ°
 * @ param εζ°η»ζδ½ει
*/
void common32960::setConfigParam(struct PARAM_T &param) {
    common32960::getInstance()->m_paramObj = param;
}

/**
 * @brief  θ·ειη½?εζ°
 * @ return θΏεη»ζδ½ει
*/
struct PARAM_T &common32960::getConfigParam() {
    return m_paramObj;
}

/**
 * @brief  θ·εζΆι΄ζ³
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
 * @brief  θ?Ύη½?Utcζιδ»£η 
*/
int common32960::setDtcErrCode(SMLK_UINT8 val)
{
    OBD::UdsDtcTriggerInfo dtc_info;

    if(val != m_last_val)
    {
        m_last_val = val;

        // DTCζιδ»£η 
        dtc_info.val = val;
        dtc_info.dtc_type = OBD::UdsDtcTriggerType::E_UDS_DTC_TRIGGER_GB_CONNECT_STATUS;
        OBD::DidIpcApi::getInstance()->SendDtcErrCode(dtc_info);
        SMLK_LOGD("[T] common32960::setDtcErrCode dtc_info.val=%d,dtc_info.dtc_type=%d", dtc_info.val,dtc_info.dtc_type);
    }

    return 0;
}

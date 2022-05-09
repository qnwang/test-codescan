/**
 * @file common_17691.cpp
 * @author T (taoguanjie@smartlink.com.cn)
 * @brief 
 * @version 0.1
 * @date 2022-04-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <smlk_log.h>
#include <smlk_tools.h>

#include "common_17691.h"

using namespace smartlink;

/**
 * @brief Construct a new common17691::common17691 object
 * 
 * @param taskName 
 */
common17691::common17691(std::string &taskName)
    : m_AuthSignState(false), m_ignState(0), m_todisk(false), m_socketState(-1), sequence_login(0)
{
    m_taskName = taskName;
    m_loginSeq = "loginSeq" + m_taskName;
    m_infoupSeq = "infoUpSeq" + m_taskName;

    m_sequence_dir = M_GB6_CONFIG_DIR + m_taskName + "/";
}


/**
 * @brief 设置socket连接状态
 * 
 * @param state 
 */
void common17691::setSocketState(int state) {
    SMLK_LOGD("[%s] common17691::setSocketState() %d ", m_taskName.c_str(), state);
    if (m_socketState != state) {
        m_socketState = state;
    }
}

/**
* @brief   获取socket连接状态
*/
int common17691::getSocketState() {
    SMLK_LOGD("[%s] common17691::getSocketState() %d ", m_taskName.c_str(), m_socketState);
    return m_socketState;
}


/**
 * @brief 获取流水号
 * 
 * @param type 
 * @return SMLK_UINT16 
 */
SMLK_UINT16 common17691::getSequence(int type) {
    if (type == M_GB_17691_CMD_VEHICLE_LOGIN) {
        return sequence_login;      //登入
    }
    if (type == M_GB_17691_CMD_VEHICLE_LOGOUT) {
        return sequence_login - 1;  //登出
    }

    if (type == M_GB_17691_CMD_REALTIME_REPORT || type == M_GB_17691_CMD_REISSUE_REPORT) {
        if (m_ignState == 0) {
            return sequence_infoup;
        } else {
            return ++sequence_infoup;
        }
    }
}

/**
 * @brief 保存流水号到文件
*/
void common17691::saveSequence(bool save_login) {
    std::fstream seqFile;
    std::string strDate = getDateFromat(false);

    if (save_login) {
        seqFile.open(m_sequence_dir + (m_loginSeq + strDate), std::ios::out);
        seqFile << ++sequence_login;
        seqFile.close();
    }

    seqFile.open(m_sequence_dir + (m_infoupSeq + strDate), std::ios::out);
    seqFile << sequence_infoup;
    seqFile.close();
}

/**
 * @brief 初始化流水号
*/
void common17691::initSequence() {
    SMLK_LOGD("[%s] common17691::initSequence()", m_taskName.c_str());
    // 遍历文件夹
    std::fstream seqFile;
    std::string sLogSeqFileName = "";
    std::string sInfoSeqFileName = "";

    if ( !tools::dir_exists(m_sequence_dir) ) {
        if ( 0 != tools::mkdir_p(m_sequence_dir) ) {
            SMLK_LOGE("[%s] common17691::initSequence() fail to create sequce dir", m_taskName.c_str());
            return;
        }
    }

    DIR *dir = opendir(m_sequence_dir.c_str());
    dirent *p = NULL;
    while ((p = readdir(dir)) != NULL) {
        if (p->d_name[0] != '.') {
            if ((p->d_name)[0] == 'l') {
                sLogSeqFileName = std::string(p->d_name);
            } else if ((p->d_name)[0] == 'i') {
                sInfoSeqFileName = std::string(p->d_name);
            }
        }
    }
    closedir(dir);
    std::string strDate = getDateFromat(false);
    SMLK_LOGD("[%s] sLogSeqFileName=%s", m_taskName.c_str(), sLogSeqFileName.c_str());
    SMLK_LOGD("[%s] sInfoSeqFileName=%s", m_taskName.c_str(), sInfoSeqFileName.c_str());

    if (sLogSeqFileName == "") {
        SMLK_LOGD("[%s] sLogSeqFileName is null", m_taskName.c_str());
        // 创建文件  设置为1， 并且将sequence_login 赋值为1
        seqFile.open(m_sequence_dir + (m_loginSeq + strDate), std::ios::out);
        sequence_login = 1;
        seqFile << sequence_login;
    } else {
        SMLK_LOGD("[%s] sLogSeqFileName no null", m_taskName.c_str());
        // 判断名字是否与当天一样
        if (m_loginSeq + strDate == sLogSeqFileName) {
            //读出里面的值 赋值给并且将sequence_login
            seqFile.open(m_sequence_dir + (m_loginSeq + strDate), std::ios::in | std::ios::out);
            seqFile >> sequence_login;
            SMLK_LOGD("[%s] sequence_login=%d", m_taskName.c_str(), sequence_login);
            if (sequence_login > 65531) {
                sequence_login = 1;
                seqFile << sequence_login;
            }
        } else {
            //删除当前的文件，并创建一个文件， 并且将sequence_login 赋值为1
            std::string file_dir = m_sequence_dir + sLogSeqFileName;
            remove(file_dir.c_str());
            seqFile.open(m_sequence_dir + (m_loginSeq + strDate), std::ios::out);
            sequence_login = 1;
            seqFile << sequence_login;
            SMLK_LOGD("[%s] new day sequence_login=%d", m_taskName.c_str(), sequence_login);
        }
    }

    seqFile.close();

    if (sInfoSeqFileName == "") {
        SMLK_LOGD("[%s] sInfoSeqFileName is null", m_taskName.c_str());
        // 创建文件  设置为1， 并且将 sequence_infoup 赋值为1
        seqFile.open(m_sequence_dir + (m_infoupSeq + strDate), std::ios::out);
        sequence_infoup = 1;
        seqFile << sequence_infoup;
    } else {
        SMLK_LOGD("[%s] sInfoSeqFileName no null", m_taskName.c_str());
        // 判断名字是否与当天一样
        if (m_infoupSeq + strDate == sInfoSeqFileName) {
            //读出里面的值 赋值给并且将 sequence_infoup
            seqFile.open(m_sequence_dir + (m_infoupSeq + strDate), std::ios::in | std::ios::out);
            seqFile >> sequence_infoup;
            SMLK_LOGD("[%s] sequence_infoup=%d", m_taskName.c_str(), sequence_infoup);
            if (sequence_infoup > 65531) {
                sequence_infoup = 1;
                seqFile << sequence_infoup;
            }
        } else {
            //删除当前的文件，并创建一个文件， 并且将 sequence_infoup 赋值为1
            std::string file_dir = m_sequence_dir + sInfoSeqFileName;
            remove(file_dir.c_str());
            seqFile.open(m_sequence_dir + (m_infoupSeq + strDate), std::ios::out);
            sequence_infoup = 1;
            seqFile << sequence_infoup;
            SMLK_LOGD("[%s] new day sequence_infoup=%d", m_taskName.c_str(), sequence_infoup);
        }
    }

    seqFile.close();
}

/**
 * @brief  设置 IGN 状态
 * @param state  IGN 的值
*/
void common17691::setIgnState(int state) {
    if (m_ignState != state) {
        m_ignState = state;
    }
}

/**
 * @brief  获取IGN状态
*/
int common17691::getIgnState() {
    return m_ignState;
}

/**
 * @brief 获取时间
 * @param isDateTime true:20200407102520(yyyyMMddhhmmss)  false:20200407(yyyyMMdd)
 */
std::string common17691::getDateFromat(bool isDateTime)
{
    time_t timep;
    struct tm *p;
    time( &timep );
    p = localtime( &timep );
    std::stringstream strStream;
    if(isDateTime)
    {
        strStream<<(p->tm_year + 1900)<<"-"<<std::setw(2)<<std::setfill('0')<<(p->tm_mon + 1)<<"-"<<std::setw(2)<<std::setfill('0')<<p->tm_mday \
        <<"_"<<std::setw(2)<<std::setfill('0')<<p->tm_hour<<":"<<std::setw(2)<<std::setfill('0')<<p->tm_min<<":"<<std::setw(2)<<std::setfill('0')<<p->tm_sec;
    }
    else{
        strStream<<(p->tm_year + 1900)<<std::setw(2)<<std::setfill('0')<<(p->tm_mon + 1)<<std::setw(2)<<std::setfill('0')<<p->tm_mday;
    }
    return strStream.str();
}

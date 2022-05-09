#ifndef COMMON17691_H
#define COMMON17691_H

#include <mutex>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <sys/io.h>
#include <dirent.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <unordered_map>

#include <smlk_signal.h>

#include "message_def_17691.h"

namespace smartlink
{
    /**
     * @brief 17691协议公共类
     */
    class common17691
    {
    public:
        common17691(std::string &taskName);

        ~common17691();

        void setSocketState(int state); //设置socket连接状态

        int getSocketState(); // 获取socket连接状态

        void initSequence(); //初始化流水号

        SMLK_UINT16 getSequence(int type); //获取流水号

        void saveSequence(bool save_login = true); //保存流水号

        void setIgnState(int state); // 设置 ign 状态

        int getIgnState(); // 查询ign 状态

        bool    m_AuthSignState; //登入成功标志
        int     m_ignState;       //test
        bool    m_todisk;

    private:

        std::string getDateFromat(bool isDateTime);

        int     m_socketState;

        std::string m_taskName;

        std::string m_loginSeq;
        std::string m_infoupSeq;
        std::string m_sequence_dir;

        SMLK_UINT16 sequence_login;
        SMLK_UINT16 sequence_infoup;
    };

}

#endif // COMMON17691_H

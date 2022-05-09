#ifndef COMMON32960_H
#define COMMON32960_H

#include <mutex>
#include <fstream>
#include <sys/io.h>
#include <dirent.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <unordered_map>

#include <smlk_signal.h>

#include "sqllite_32960.h"

#include "TCPClient.h"

#include "ev_signals.h"
#include "message_def_32960.h"

#include "ThreadPool.h"
#include "ThreadSafeDeque.h"

#include "did_ipc_api.h"

namespace smartlink
{
    struct PARAM_T
    {
        SMLK_UINT16 warn_period;
        SMLK_UINT16 upload_period;
        SMLK_UINT16 upload_period_bak;
        SMLK_UINT8 relogin_period; //连续3次登入失败后，到下一次登入的时间
    };

    /**
     * @brief 32960协议公共类
     */
    class common32960
    {
    public:
        static common32960 *getInstance();

        std::shared_ptr <ThreadPool> GetThreadPool();

        void setSocketState(int state); //设置socket连接状态

        int getSocketState(); // 获取socket连接状态

        int getInfoUpTime32960(); // 获取 32960数据上传的时间间隔

        void setInfoUpTime32960(SMLK_UINT16 value);

        void initSequence(); //初始化流水号

        int getSequence(bool b_in); //获取流水号

        void saveSequence(); //保存流水号

        void setIgnState(int state); // 设置 ign 状态

        int getIgnState(); // 查询ign 状态

        bool getThreeWarnFlag();

        void setThreeWarnFlag(bool value);

        void initConfigParam();

        void setConfigParam(struct PARAM_T &param);

        struct PARAM_T &getConfigParam();
        
        void getTimestamp(SMLK_UINT8 *buffer);

        int setDtcErrCode(SMLK_UINT8 val);

        SqlLiteBase     *m_SqlLiteBase32960  = nullptr;
        TCPClient       *m_TcpClient32960 = nullptr; //业务类使用

        bool            m_AuthSignState; //登入成功标志
        int             m_ignState;       //test
        struct PARAM_T  m_paramObj;
        bool            m_three_warn_level;        //3级告警标志
        SMLK_UINT8      m_run_mode;
        bool            m_todisk;
        
        SMLK_UINT8      m_vin[M_GB_32960_SZ_VIN];
        SMLK_UINT8      m_iccid[M_GB_32960_SZ_ICCID];

        SMLK_UINT16     m_refer_torque;

        DM1EventInfo    m_dm1info;

        MDATAInfo       m_tempinfo;
        MDATAInfo       m_voltinfo;
        std::unordered_map<int, SMLK_UINT8> m_lastvoltinfo;

        std::unordered_map<vehicle::SignalID, vehicle::SignalVariant>     m_cached_signals;

    private:
        common32960();
        ~common32960();

        std::shared_ptr <ThreadPool> m_execThread;

        static common32960 *m_instance;
        static std::mutex m_mtx;
        SMLK_UINT32 sequence_login;
        SMLK_UINT32 sequence_infoUp;
        int m_SocketState;
        SMLK_INT8 m_last_val;

    };

}

#endif // COMMON32960_H

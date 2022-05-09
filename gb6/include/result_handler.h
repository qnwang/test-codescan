/*
 * @Author: T
 * @Date: 2022-04-19 20:39:35
 * @LastEditTime: 2022-05-06 17:58:46
 * @LastEditors: taoguanjie taoguanjie@smartlink.com.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /Workspace/tbox2.0/source/apps/gb6/include/common_handler.h
 */

#ifndef RESULTHANDLER_H
#define RESULTHANDLER_H

#include <mutex>
#include <fstream>
#include <sys/io.h>
#include <dirent.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <unordered_map>

#include "did_ipc_api.h"

namespace smartlink
{
    /**
     * @brief 状态处理公共类
     */
    class resulthandler
    {
    public:
        static resulthandler *getInstance();

        void setGB6VinStatus(SMLK_UINT8 state);
        void setPlatformConnStatus(SMLK_UINT8 state, SMLK_UINT8 platform);

        void setHJActiveCMDSeq(std::string &seq);
        std::string getHJActiveCMDSeq();

        void setHJActiveResult(SMLK_UINT8 state, SMLK_UINT8 save);
        SMLK_UINT8 getHJActiveResult();

        void handleHJActiveResult(SMLK_UINT8 state);

        std::string m_808_sn;

    private:
        resulthandler();
        ~resulthandler();

        void handleDTCErrCode(SMLK_UINT8 errcode);

        static resulthandler *m_instance;
        static std::mutex m_mtx;

        SMLK_UINT8 m_ent_conn_state;
        SMLK_UINT8 m_loc_conn_state;

        std::string m_808_act_seq;
        SMLK_UINT8 m_808_act_state;

    };

}

#endif // RESULTHANDLER_H


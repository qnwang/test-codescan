/*****************************************************************************/
/**
 * \file       close_control.h
 * \author     weixuefeng
 * \date       2021/7/06
 * \version    Tbox2.0 V1
 * \brief      remote control service class
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks
 ******************************************************************************/
#ifndef _CCONTROL_SERVICE_H_
#define _CCONTROL_SERVICE_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>
#include <map>
#include <json/json.h>
#include <condition_variable>
#include <smartlink_sdk_mcu.h>

#include "smlk_error.h"
#include "smlk_types.h"
#include "smlk_queue.h"
#include "cctrl_common.h"
#include "cctrl_service_inf.h"
#include "vehicle_data_api.h"
#include "smartlink_sdk_sys_time.h"
#include "smartlink_sdk_wlan.h"
#include "ipc_api_def.h"
#include "smlk_tools.h"
#include "smlk_spi_manager.h"
#include "did_ipc_api.h"
#include "smlk_wifi_service_api.h"

/*****************************************************************************
 *                                 类定义                                   *
 *****************************************************************************/

namespace smartlink
{
    namespace CloseCtrl
    {
        struct CmpCctrlCmdKey
        {
            bool operator()(const CctrlSetCmd &key1, const CctrlSetCmd &key2) const
            {
                return ((((SMLK_UINT32)key1.cmd_id) << 16 | (SMLK_UINT32)(key1.action)) < (((SMLK_UINT32)key2.cmd_id) << 16 | (SMLK_UINT32)(key2.action)));
            }
        };

        /** 近控管理类 */
        class CloseCtrlService : public IService
        {
        public:
            CloseCtrlService();
            virtual ~CloseCtrlService();

        public:
            SMLK_RC Init();
            SMLK_RC Start();
            void Stop();
            bool IsRunning() const;

        private:
            /*初始化*/
            SMLK_RC InitCctrlToMcuMap();
            /*控车指令分配*/
            void OnWifiSerIndication(smartlink::Wifi::SmlkWifiEventId id, void *data, int length);
            void ProcessWifiSvcCCtrl(IN void *data, IN int length);
            void ProcessWifiSvcStateQuery(IN void *data, IN int length); /*处理wifi svc近控状态查询(BCM+DCM+AC)并回复结果*/
            /*IVI控制*/
            SMLK_RC ProcessIVIReq(CCtrlInterInfo &cctrl_msg);
            size_t FindIviCmd(ControlMsg volume, IviControlMsg *iviCtrlArry);
            void handleIviCtrResult(void *data, int len);
            /*MCU控制*/
            SMLK_RC ProcessMCUReq(CCtrlInterInfo &cctrl_msg);
            SMLK_RC VdToCCtrlValueMap(IN VehicleDataUsage vd_usage, IN SMLK_UINT32 vehicle_index, IN SMLK_DOUBLE vehicle_data, IN bool vechile_valid, OUT CctrlGetCmd &cctrl_get_cmd);
            SMLK_RC ProcessCctrlCmdToMCU(IN CctrlSetCmd &cctrl_cmd_vec, OUT CCtrlCmdResult &result);
            SMLK_RC SendCctrlCmdToMcu(IN CctrlSetCmd &cctrl_cmd);
            void OnMcuSerIndication(smartlink_sdk::McuEventId id, void *data, int len);
            void ProcessMcuSerResp(IN void *data, IN int length);
            /*控车结果处理*/
            void HandleRequestResult(IN CCtrlInterInfo cmd_msg, IN CCtrlCmdResult cmd_result, OUT smartlink::Wifi::SmlkVehicleCtrlResp &cmd_resp);

        private:
            /*进程状态变更*/
            SMLK_UINT32 m_flags;
            std::mutex m_mutex;
            /*mcu*/
            std::condition_variable m_cv;
            std::mutex m_cv_mtx;
            /*车机IVI*/
            std::condition_variable m_ivi;
            std::mutex m_ivi_mtx;

            std::function<void(void *data, int len)> m_ctrl_ivi_cb;                            /*车机控制结果回调*/
            std::map<CctrlSetCmd, std::vector<CCtrlSetMcuCmd>, CmpCctrlCmdKey> m_cctl_mcu_map; /*近控到mcu的命令的map表*/
            SMLK_UINT16 m_cctrl_mcu_seq;                                                       /*发给mcu的序列值,范围0x8000~0xFFFF*/
            CCtrlCmdResult m_cctrl_cmd_result;
        };
    }
}

#endif

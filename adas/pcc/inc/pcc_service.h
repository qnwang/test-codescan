
/*****************************************************************************/
/**
* \file       pcc_service.h
* \date       2022/02/16
* \author     wujian
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#ifndef _PCC_SERVICE_H_
#define _PCC_SERVICE_H_

#include <thread>
#include <atomic>
#include <mutex>
#include <map>

#include <smlk_queue.h>
#include <smlk_timer_new.h>

#include "smlk_types.h"
#include "smlk_log.h"
#include "adas/adasisv2hp.h"
#include "event_variable.hpp"
#include "adas_define_common.h"

namespace smartlink {
    using OnAv2HP_setMessageCB     = int (*) (Av2HP_pushMessageCB av2hp_cb);


class PccService
{
    using OnAv2HP_getMapVersion     = void (*) (char* path, char* version, int lenth);
    using OnAv2HP_getSoftVersion    = void (*) (char* version, int lenth);
    using OnAv2HP_init              = av2hp_e (*) (const char *conf_path);
    using OnAv2HP_destory           = void (*) (void);
    using OnAv2HP_setDeviceId       = void (*) (const char* strDeviceId);
    using OnAv2HP_getMeta           = av2hp_e (*) (av2hp_meta *meta_data);
    using OnAv2HP_run               = av2hp_e (*) (void);
    using OnAv2HP_setGpsInfo        = av2hp_e (*) (av2hp_gpsInfo *gps);
    using OnAv2HP_getSoftValid      = av2hp_license_status (*) (void);
    using OnAv2HP_getMsgType        = unsigned char (*) (unsigned char* value, unsigned char length);

public:
    PccService();
    ~PccService();

    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    void            Stop();
    bool            IsRunning() const;
    void            RunTestData();

    void Post(IN MessageHead &head, IN SMLK_UINT8 *data, IN std::size_t sz);
    void Post(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);

    int Av2hpMessageNotify(const char *message, int *message_num);
    bool GetHPMsgCb(OnAv2HP_setMessageCB &ptr);

private:
    void HandleEvent(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleLocationInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleMCUInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleTspInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleTimerInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);

    void SendCommonResp(SMLK_UINT16 msg_id, SMLK_UINT16 seq_id, bool result);
    void Send0F51MsgResp(int n_lic_status);

    void SuspendMainThread(pthread_cond_t &cond);
    void ResumeMainThread(pthread_cond_t &cond);
    bool CheckCurIsSupportPcc(bool &b_is_qingdao);
    SMLK_UINT32 LoopWaitCondition();
    void InitVehicleType();


private:
    std::thread         m_main_thread;
    Queue<EventVarible> m_main_events;
    av2hp_gpsInfo       m_cur_gps_info;
    std::atomic<bool>   m_b_ign_on;

    std::shared_ptr<ITimer> m_timer;

    std::mutex m_map_info_mutex;        //map缓存数据锁
    // char m_map_info[3][8];              //map数据缓存， 目前存在POSITION、STUB、SHORT数据，都8字节。 0: POSITION  1: STUB  2: SHORT

    bool m_b_vehicle_is_qingdao;        //当前是否为青岛车系

    void *m_p_adas_lib = NULL;          //指向adas.so文件的指针
    OnAv2HP_getMapVersion   m_p_Av2HP_getMapVersion = NULL;    //对应的hp库函数指针
    OnAv2HP_getSoftVersion  m_p_Av2HP_getSoftVersion = NULL;
    OnAv2HP_init            m_p_Av2HP_init           = NULL;
    OnAv2HP_destory         m_p_Av2HP_destory        = NULL;
    OnAv2HP_setDeviceId     m_p_Av2HP_setDeviceId    = NULL;
    OnAv2HP_getMeta         m_p_Av2HP_getMeta        = NULL;
    OnAv2HP_run             m_p_Av2HP_run            = NULL;
    OnAv2HP_setGpsInfo      m_p_Av2HP_setGpsInfo     = NULL;
    OnAv2HP_getSoftValid    m_p_Av2HP_getSoftValid   = NULL;
    OnAv2HP_setMessageCB    m_p_Av2HP_setMessageCB   = NULL;
    OnAv2HP_getMsgType      m_p_Av2HP_getMsgType     = NULL;

    SMLK_UINT8 m_n_power_can_channle;          //动力CAN通道
};

}// namespace smartlink


#endif
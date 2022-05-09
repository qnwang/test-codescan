/*****************************************************************************/
/**
* \file       adas_service.h
* \date       2022/04/12
* \author     wujian
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#ifndef _ADAS_SERVICE_H_
#define _ADAS_SERVICE_H_

#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <set>

#include <smlk_queue.h>
#include <smlk_timer_new.h>

// #include "adas/adasisv2hp.h"
// #include "adas/adas.h"

#include "smlk_types.h"
#include "smlk_log.h"
#include "event_variable.hpp"
#include "adas_define_common.h"
#include "adas_so_l2.h"
#include "adas_so_pcc.h"

namespace smartlink {

class AdasService
{


public:
    AdasService();
    ~AdasService();

    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    void            Stop();
    bool            IsRunning();
    bool            IsPccOrL2Plus();


    int Av2hpMessageNotify(const char *message, int *message_num);
    bool GetHPMsgCb(OnAv2HP_setMessageCB &ptr);

    void AdasMessageNotify(const char* msg_data, const unsigned int* msg_id, int msg_num);



private:
    void Post(IN MessageHead &head, IN SMLK_UINT8 *data, IN std::size_t sz);
    void Post(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);

    void HandleEvent(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleLocationInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleMCUInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleTspInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleTimerInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);

    void SuspendMainThread(pthread_cond_t &cond);
    void ResumeMainThread(pthread_cond_t &cond);
    bool CheckCurIsSupportMap(bool &b_is_pcc);
    bool CheckCurIsSupportPcc(bool &b_is_qingdao);
    bool LoopWaitCondition();
    void InitVehicleType();

private:
    std::thread                     m_main_thread;
    Queue<EventVarible>             m_main_events;

    std::shared_ptr<AdasSoBase>     m_sp_adas_obj;
    std::shared_ptr<ITimer>         m_timer;


    bool   m_b_is_pcc;          //true:当前运行pcc地图  false:当前运行l2+地图
    bool   m_b_is_qingdao;      //true:当前pcc为青岛版  false:当前pcc为一汽版
    SMLK_UINT8 m_n_power_can_channle;          //动力CAN通道
    SMLK_UINT16 m_n_gps_collection_cycle;      //gps采集周期, 默认200ms(一汽pcc & l2+), 青岛pcc为1000ms

    std::mutex          m_location_mutex;
    smartlink_sdk::LocationInfo        m_location_info;
    smartlink_sdk::GnssSatelliteInfo   m_gnss_statelite_info;
};

}// namespace smartlink


#endif
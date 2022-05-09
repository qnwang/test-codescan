/*****************************************************************************/
/**
* \file       l2_service.h
* \date       2022/02/28
* \author     wujian
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/
#ifndef _L2_SERVICE_H_
#define _L2_SERVICE_H_

#include <iostream>
#include <map>
#include <set>


#include <adas/adas.h>
#include <smlk_queue.h>
#include <smlk_timer_new.h>

#include "smlk_types.h"
#include "smlk_log.h"
#include "event_variable.hpp"

namespace smartlink {

class L2Service
{
public:
    L2Service(/* args */);
    ~L2Service();

    SMLK_UINT32     Init();
    SMLK_UINT32     Start();
    void            Stop();
    bool            IsRunning();

    void Post(IN MessageHead &head, IN SMLK_UINT8 *data, IN std::size_t sz);
    void Post(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);

    void AdasMessageNotify(const char* msg_data, const unsigned int* msg_id, int msg_num);

private:
    void HandleEvent(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleLocationInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);
    void OnHandleTimerInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data);


    void SuspendMainThread(pthread_cond_t &cond);
    void ResumeMainThread(pthread_cond_t &cond);
    void InitVehicleType();


private:
    std::thread         m_main_thread;
    Queue<EventVarible> m_main_events;
    std::shared_ptr<ITimer> m_timer;

    adas_gps_info  m_struct_gps_info;

    std::mutex m_map_pcc_info_mutex;            //map缓存数据锁
    char m_map_pcc_info[3][8];              //map -pcc数据缓存， 目前存在POSITION、STUB、SHORT数据，都8字节。 0: POSITION  1: STUB  2: SHORT

    SMLK_UINT8 m_n_power_can_channle;          //动力CAN通道

};
}// namespace smartlink


#endif
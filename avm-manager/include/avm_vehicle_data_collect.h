/*****************************************************************************/
/**
 * \file       avm_vehicle_data_collect.h
 * \author     wukai
 * \date       2021/08/04
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef SMLK_AVM_VEHICLE_DATA_COLLECT_H_
#define SMLK_AVM_VEHICLE_DATA_COLLECT_H_
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include <iostream>
#include <atomic>
#include <mutex>
#include <cstring>
#include <thread>
#include <unordered_map>

#include "avm_msg_builder.h"
#include "smlk_module_inf.h"
#include "smlk_queue.h"
#include "vehicle_data_def.h"
#include "vehicle_data_index_def.h"
#include "smlk_signal.h"

namespace smartlink
{
    namespace AVM
    {
        using namespace vehicle;

        static const SMLK_UINT16 EVENT_REPORT_ID = 0x0023;  // avm function id
        static const SMLK_UINT8 EVENT_REPORT_DEVICE = 0xF1; // tbox self

        typedef struct DataAll_
        {
            DataAll_()
            {
                memset(this, 0x0, sizeof(DataAll_));
                function_id = EVENT_REPORT_ID;
                device_id = EVENT_REPORT_DEVICE;
                accelerator_pedal = 0xFF;
                broken_pedal = 0xFF;
                broken_status = 0xFF;
                engine_round_speed = 0xFFFF;
                steering_wheel_angle = 0xFFFF;
                corner_lamp_status = 0x00;
            };
            SMLK_UINT16 function_id;          // 功能ID 定义为0x23；代表事件上传
            SMLK_UINT8 device_id;             // 设备ID 定义为0xf1；代表Tbox；
            SMLK_UINT32 vehicle_status;       // 车辆状态 0~3 bit  0 0：ACC关；1：ACC开（用IGN表示）| 1    0：未定位；1：已定位| 2    0：北纬；1：南纬| 3    0：东经；1：西经
            SMLK_UINT32 latitude;             // GPS纬度 正：北纬　负：南纬
            SMLK_UINT32 longitude;            // GPS经度 正：东经　负：西经
            SMLK_UINT16 altitude;             // GPS高程/m
            SMLK_UINT16 gps_speed;            // GPS速度 100m/h
            SMLK_UINT16 direction;            // GPS方向 0-359，正北为 0，顺时针
            SMLK_UINT8 upload_time[6];        // 上传时间
            SMLK_UINT16 can_speed;            // CAN车速
            SMLK_UINT8 gear_status;           // 档位状态 0：空挡 1-20：档位 30：倒挡 31：驻车档 0xFF：档位无效
            SMLK_UINT8 accelerator_pedal;     // 加速踏板开合度 范围0-100，单位% 0xFF：无效
            SMLK_UINT8 broken_pedal;          // 制动踏板开合度 范围0-100，单位% 0xFF：无效
            SMLK_UINT8 broken_status;         // 制动状态 0：未制动 1：已制动 0xFF：无效
            SMLK_UINT16 engine_round_speed;   // 发动机转速 单位RPM 0xFFFF：无效
            SMLK_UINT16 steering_wheel_angle; // 方向盘角度 方向盘转过的角度，顺时针为正，逆时针为负，偏移量-2000。0xFFFF：无效
            SMLK_UINT8 corner_lamp_status;    // 转向灯状态 0：未打方向灯 1：左转方向灯 2：右转方向灯 0xFF：无效
        } __attribute__((__packed__)) DataAll;

        typedef struct EventGeneral_
        {
            EventGeneral_()
            {
                memset(this, 0x0, sizeof(EventGeneral_));
                seq_id = 0x0000;
            };
            SMLK_UINT8 event_start_time[6]; // 事件开始时间
            SMLK_UINT8 event_end_time[6];   // 事件结束时间
            SMLK_UINT16 duration_time;      // 事件持续时长/s
            SMLK_UINT16 duration_length;    // 持续行驶距离/m
            SMLK_UINT16 seq_id;             // always 0x0000;
            SMLK_UINT8 temp_buf[6];
            SMLK_UINT8 alarm_father_type; // 报警事件big类型
        } __attribute__((__packed__)) EventGeneral;

#define BCD_TIME_LENGTH 6
#define ALARM_TAG_LENGTH 30

        class DataProcessor final : public BaseModule
        {
        public:
            DataProcessor();
            virtual ~DataProcessor();
            std::size_t DecodeAvmMessage(IN AvmMessage &msg, OUT std::vector<SMLK_UINT8> &data);

            virtual SMLK_RC Init();
            virtual SMLK_RC Start();
            virtual void Stop();

        private:
            std::size_t DealDmsEvent(IN SMLK_UINT8 *event_ptr, std::size_t len, OUT std::vector<SMLK_UINT8> &out_vec);
            std::size_t DealFcwEvent(IN SMLK_UINT8 *event_ptr, std::size_t len, OUT std::vector<SMLK_UINT8> &out_vec);
            std::size_t DealOilBoxEvent(IN SMLK_UINT8 *event_ptr, std::size_t len, OUT std::vector<SMLK_UINT8> &out_vec);
            std::size_t DealFaceRecEvent(IN SMLK_UINT8 *event_ptr, std::size_t len, OUT std::vector<SMLK_UINT8> &out_vec);
            void UpdateCollectData();
            void DoParseVehicle();
            void CalcSignalVal(SMLK_UINT32 id);
            void SimuAvmInputMsg(std::string simu_msg, OUT std::vector<SMLK_UINT8> &event_data);
            void hex2int(char c, OUT std::size_t &int_o);

        private:
            std::atomic<SMLK_UINT8> m_atm_ig_status;
            std::mutex m_data_mutex;
            DataAll m_collect_data;
            std::atomic<SMLK_UINT32> m_atm_vehicle_status;
            std::thread m_vehicle_thread;
            Queue<VehicleData> m_vehicle_data_que;
            static std::unordered_map<SMLK_UINT32, SignalVariant> m_signals;
        };
    }
}

#endif // SMLK_AVM_VEHICLE_DATA_COLLECT_H_
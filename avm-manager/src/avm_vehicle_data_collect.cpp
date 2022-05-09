/*****************************************************************************/
/**
 * \file       avm_vehicle_data_collect.cpp
 * \author     wukai
 * \date       2021/08/04
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/

#include "avm_vehicle_data_collect.h"
#include "vehicle_data_api.h"
#include "smartlink_sdk_sys_power.h"
#include "smartlink_sdk_location.h"

#include "smlk_tools.h"

namespace smartlink
{
    namespace AVM
    {
        using namespace smartlink_sdk;
        using namespace smartlink::tools;

        std::unordered_map<SMLK_UINT32, SignalVariant> DataProcessor::m_signals = {
            {VEHICLE_DATA_TACHOGRAPH_VEHICLE_SPEED, SignalVariant(1624, 6, 1.0 / 256.0, 0.0, 0, 250.99609375)},    // can-speed
            {VEHICLE_DATA_CURRENT_GEAR, SignalVariant(523, 8, 1, -125, -125, 125)},                                // gear status
            {VEHICLE_DATA_ACCELERATOR_PEDAL_POSITION, SignalVariant(91, 8, 0.4, 0, 0, 100)},                       // 加速踏板开合度
            {VEHICLE_DATA_BRAKE_PEDAL_POSITION, SignalVariant(521, 8, 0.4, 0, 0, 100)},                            // 制动踏板开合度
            {VEHICLE_DATA_BRAKE_SWITCH, SignalVariant(0, 2)},                                                      // 制动状态
            {VEHICLE_DATA_ENGINE_SPEED, SignalVariant(190, 16, 0.125, 0, 0, 8031.875)},                            // 发动机转速
            {VEHICLE_DATA_STEERING_WHEEL_ANGLE, SignalVariant(1807, 8, 1.0 / 1024.0, -31.375, -31.375, 31.374024)} // 方向盘角度
        };

        std::size_t DataProcessor::DecodeAvmMessage(IN AvmMessage &msg, OUT std::vector<SMLK_UINT8> &data)
        {
            struct
            {
                SMLK_UINT16 event_id;
                SMLK_UINT16 event_len;
            } event_child_info;
            bzero(&event_child_info, sizeof(event_child_info));

#if 1 // recv Avm True Msg
            memcpy(&event_child_info, msg.child_data(), sizeof(event_child_info));
            event_child_info.event_id = be16toh(event_child_info.event_id);
            event_child_info.event_len = be16toh(event_child_info.event_len);
            std::vector<SMLK_UINT8> event_data;
            event_data.reserve((std::size_t)event_child_info.event_len);
            event_data.insert(event_data.end(), msg.child_data() + 4, msg.child_data() + 4 + event_child_info.event_len);
#else // recv Simu Data Input Msg
            event_child_info.event_id = 0x0001;
            event_child_info.event_len = 0x0035;
            SMLK_LOGD("DecodeAvmMessage recv Avm report event_id: [0x%04x]  event_len: [0x%04x]", event_child_info.event_id, event_child_info.event_len);
            std::vector<SMLK_UINT8> event_data;
            event_data.reserve((std::size_t)event_child_info.event_len);
            std::string simu_msg = "";
            // std::string simu_msg = "2c0601211014104057211014104107000affff000000b4303934303237373938383635fe3231313031343130343035373031313830";
            SimuAvmInputMsg(simu_msg, event_data);
#endif
            std::size_t out_len = -1;
            UpdateCollectData();

            switch (event_child_info.event_id)
            {
            case 0x01: // DMS事件
            {
                out_len = DealDmsEvent(event_data.data(), event_data.size(), data);
            }
            break;
            case 0x02: // 碰撞预警事件
            {
                out_len = DealFcwEvent(event_data.data(), event_data.size(), data);
            }
            break;
            case 0x03: // 油箱防盗事件
            {
                out_len = DealOilBoxEvent(event_data.data(), event_data.size(), data);
            }
            break;
            default:
                break;
            }
            if ((int)out_len < 0)
            {
                return -1;
            }
            SMLK_UINT16 len_ = htobe16((SMLK_UINT16)out_len);
            data.insert(data.begin(), (SMLK_UINT8 *)&len_, (SMLK_UINT8 *)&len_ + sizeof(SMLK_UINT16));
            return out_len + 2;
        }

        std::size_t DataProcessor::DealDmsEvent(IN SMLK_UINT8 *event_ptr, std::size_t len, OUT std::vector<SMLK_UINT8> &out_vec)
        {
            struct
            {
                SMLK_UINT8 id;
                SMLK_UINT8 alarm_type;       // 报警类型
                SMLK_UINT8 file_count;       // 附件数量
                SMLK_UINT8 start_time[6];    // 事件开始时间
                SMLK_UINT8 end_time[6];      // 事件结束时间
                SMLK_UINT16 duration_time;   // 事件持续时长
                SMLK_UINT16 duration_length; // 持续行驶距离
                SMLK_UINT32 alarm_id;        // 报警ID 按照报警先后，从0开始循环累加 （断电存储），不区分报警类型。（断电存储
                SMLK_UINT8 alarm_tag[30];    // 报警标识号 IMSI后13位
            } __attribute__((__packed__)) dms_event_data;

            if (len != sizeof(dms_event_data))
            {
                SMLK_LOGW("DealDmsEvent parse data err --> IN len [%d] not compared!", len);
                return -1;
            }
            bzero(&dms_event_data, sizeof(dms_event_data));
            memcpy(&dms_event_data, event_ptr, len);
            DataAll temp_data_all;
            {
                std::lock_guard<std::mutex> __lk(m_data_mutex);
                memcpy(&temp_data_all, &m_collect_data, sizeof(DataAll));
            }
            temp_data_all.function_id = htobe16(temp_data_all.function_id);
            temp_data_all.vehicle_status = htobe32(temp_data_all.vehicle_status);
            temp_data_all.latitude = htobe32(temp_data_all.latitude);
            temp_data_all.longitude = htobe32(temp_data_all.longitude);
            temp_data_all.altitude = htobe16(temp_data_all.altitude);
            temp_data_all.gps_speed = htobe16(temp_data_all.gps_speed);
            temp_data_all.direction = htobe16(temp_data_all.direction);
            temp_data_all.can_speed = htobe16(temp_data_all.can_speed);
            temp_data_all.engine_round_speed = htobe16(temp_data_all.engine_round_speed);
            temp_data_all.steering_wheel_angle = htobe16(temp_data_all.steering_wheel_angle);
            out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&temp_data_all, (SMLK_UINT8 *)&temp_data_all + sizeof(DataAll));

            EventGeneral temp_general;
            bzero(&temp_general, sizeof(temp_general));
            memcpy(temp_general.event_start_time, dms_event_data.start_time, BCD_TIME_LENGTH);
            memcpy(temp_general.event_end_time, dms_event_data.end_time, BCD_TIME_LENGTH);
            temp_general.duration_time = dms_event_data.duration_time;
            temp_general.duration_length = dms_event_data.duration_length;

            if (dms_event_data.alarm_type == 0x06)
            {
                temp_general.alarm_father_type = 0x04; // 4: 人脸识别事件
                out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&temp_general, (SMLK_UINT8 *)&temp_general + sizeof(EventGeneral));
                struct
                {
                    SMLK_UINT32 alarm_id;
                    /**
                     * 0x00：不可用
                     * 0x01：开始标志
                     * 0x02：结束标志
                     * 该字段仅适用于有开始和结束标志类型的报警或事件，报警类型或事件类型无开始和结束标志，则该位不可用，填入0x00即可；（Tbox默认写00
                     */
                    SMLK_UINT8 tag_state;
                    SMLK_UINT16 alarm_type;
                    SMLK_UINT8 file_count;
                    SMLK_UINT8 alarm_tag[30];
                } __attribute__((__packed__)) fd_808_data; //人脸识别事件内容
                bzero(&fd_808_data, sizeof(fd_808_data));
                fd_808_data.alarm_id = dms_event_data.alarm_id;
                fd_808_data.tag_state = 0x00;
                fd_808_data.alarm_type = htobe16(0x3003);
                fd_808_data.file_count = dms_event_data.file_count;
                memcpy(fd_808_data.alarm_tag, dms_event_data.alarm_tag, ALARM_TAG_LENGTH);
                out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&fd_808_data, (SMLK_UINT8 *)&fd_808_data + sizeof(fd_808_data));
            }
            else
            {
                temp_general.alarm_father_type = 0x03; // 3：防疲劳报警事件
                out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&temp_general, (SMLK_UINT8 *)&temp_general + sizeof(EventGeneral));
                struct
                {
                    SMLK_UINT32 alarm_id;
                    /**
                     * 0x00：不可用
                     * 0x01：开始标志
                     * 0x02：结束标志
                     * 该字段仅适用于有开始和结束标志类型的报警或事件，报警类型或事件类型无开始和结束标志，则该位不可用，填入0x00即可；（Tbox默认写00
                     */
                    SMLK_UINT8 tag_state;
                    SMLK_UINT16 alarm_type;
                    SMLK_UINT8 file_count;
                    SMLK_UINT8 alarm_tag[30];
                } __attribute__((__packed__)) dms_808_data; //防疲劳告警事件内容
                bzero(&dms_808_data, sizeof(dms_808_data));
                dms_808_data.alarm_id = dms_event_data.alarm_id;
                switch (dms_event_data.alarm_type)
                {
                case 0x00:
                    dms_808_data.alarm_type = htobe16(0x2006);
                    break;
                case 0x01:
                    dms_808_data.alarm_type = htobe16(0x2003);
                    break;
                case 0x02:
                    dms_808_data.alarm_type = htobe16(0x2004);
                    break;
                case 0x03:
                    dms_808_data.alarm_type = htobe16(0x2005);
                    break;
                case 0x04:
                    dms_808_data.alarm_type = htobe16(0x2010);
                    break;
                case 0x05:
                    dms_808_data.alarm_type = htobe16(0x2008);
                    break;
                case 0x07:
                    dms_808_data.alarm_type = htobe16(0x2011);
                    break;
                case 0x08:
                    dms_808_data.alarm_type = htobe16(0x2012);
                    break;
                case 0x09:
                    dms_808_data.alarm_type = htobe16(0x2013);
                    break;
                case 0x0a:
                    dms_808_data.alarm_type = htobe16(0x2014);
                    break;
                default:
                    break;
                }
                dms_808_data.file_count = dms_event_data.file_count;
                memcpy(dms_808_data.alarm_tag, dms_event_data.alarm_tag, ALARM_TAG_LENGTH);
                out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&dms_808_data, (SMLK_UINT8 *)&dms_808_data + sizeof(dms_808_data));
            }
            return out_vec.size();
        }

        std::size_t DataProcessor::DealFcwEvent(IN SMLK_UINT8 *event_ptr, std::size_t len, OUT std::vector<SMLK_UINT8> &out_vec)
        {
            struct
            {
                SMLK_UINT8 id;
                SMLK_UINT8 alarm_type;
                SMLK_UINT8 front_car_speed;        // 单位Km/h。范围0~250，仅报警类型为0x1001和0x1002时有效。
                SMLK_UINT16 front_length;          // 单位1cm，仅报警类型为0x1001和0x1002时有效
                SMLK_UINT16 min_len_duration_time; // 单位100ms，仅报警类型为0x1002时有效
                SMLK_UINT8 file_count;             // 附件数量
                SMLK_UINT8 start_time[6];
                SMLK_UINT8 end_time[6];
                SMLK_UINT16 duration_time;
                SMLK_UINT16 duration_length;
                SMLK_UINT32 alarm_id;
                SMLK_UINT8 alarm_tag[30];
            } __attribute__((__packed__)) fcw_event_data;
            if (len != sizeof(fcw_event_data))
            {
                SMLK_LOGW("DealFcwEvent parse data err --> IN len [%d] not compared!", len);
                return -1;
            }
            bzero(&fcw_event_data, sizeof(fcw_event_data));
            memcpy(&fcw_event_data, event_ptr, len);
            DataAll temp_data_all;
            {
                std::lock_guard<std::mutex> __lk(m_data_mutex);
                memcpy(&temp_data_all, &m_collect_data, sizeof(DataAll));
            }
            EventGeneral temp_general;
            memcpy(temp_general.event_start_time, fcw_event_data.start_time, BCD_TIME_LENGTH);
            memcpy(temp_general.event_end_time, fcw_event_data.end_time, BCD_TIME_LENGTH);
            temp_general.duration_time = htobe16(fcw_event_data.duration_time);
            temp_general.duration_length = htobe16(fcw_event_data.duration_length);
            temp_general.alarm_father_type = 0x02; // 2：防碰撞报警事件

            out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&temp_data_all, (SMLK_UINT8 *)&temp_data_all + sizeof(DataAll));
            out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&temp_general, (SMLK_UINT8 *)&temp_general + sizeof(EventGeneral));

            struct
            {
                SMLK_UINT32 alarm_id;
                /**
                 * 0x00：不可用
                 * 0x01：开始标志
                 * 0x02：结束标志
                 * 该字段仅适用于有开始和结束标志类型的报警或事件，报警类型或事件类型无开始和结束标志，则该位不可用，填入0x00即可；（Tbox默认写00
                 */
                SMLK_UINT8 tag_state;
                SMLK_UINT16 alarm_type;
                SMLK_UINT8 front_car_speed;        // 单位Km/h。范围0~250，仅报警类型为0x1001和0x1002时有效。
                SMLK_UINT16 front_length;          // 单位1cm，仅报警类型为0x1001和0x1002时有效
                SMLK_UINT16 min_len_duration_time; // 单位100ms，仅报警类型为0x1002时有效
                SMLK_UINT8 file_count;
                SMLK_UINT8 alarm_tag[30];
            } __attribute__((__packed__)) fcw_808_data;
            bzero(&fcw_808_data, sizeof(fcw_808_data));
            fcw_808_data.alarm_id = htobe16(fcw_event_data.alarm_id);
            fcw_808_data.alarm_type = htobe16((SMLK_UINT16)fcw_event_data.alarm_type);
            fcw_808_data.front_car_speed = fcw_event_data.front_car_speed;
            fcw_808_data.front_length = htobe16(fcw_event_data.front_length);
            fcw_808_data.min_len_duration_time = htobe16(fcw_event_data.min_len_duration_time);
            fcw_808_data.file_count = fcw_event_data.file_count;
            memcpy(fcw_808_data.alarm_tag, fcw_event_data.alarm_tag, ALARM_TAG_LENGTH);
            out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&fcw_808_data, (SMLK_UINT8 *)&fcw_808_data + sizeof(fcw_808_data));
            return out_vec.size();
        }

        std::size_t DataProcessor::DealOilBoxEvent(IN SMLK_UINT8 *event_ptr, std::size_t len, OUT std::vector<SMLK_UINT8> &out_vec)
        {
            struct
            {
                SMLK_UINT8 id;
                SMLK_UINT8 oil_loss_before_percent; // 0-100 %
                SMLK_UINT8 oil_loss_after_percent;  // 0-100 %
                SMLK_UINT8 file_count;              // 附件数量
                SMLK_UINT8 start_time[6];
                SMLK_UINT8 end_time[6];
                SMLK_UINT16 duration_time;
                SMLK_UINT16 duration_length;
                SMLK_UINT32 alarm_id;
                SMLK_UINT8 alarm_tag[30];
            } __attribute__((__packed__)) oil_event_data;
            if (len != sizeof(oil_event_data))
            {
                SMLK_LOGW("DealOilBoxEvent parse data err --> IN len [%d] not compared!", len);
                return -1;
            }
            bzero(&oil_event_data, sizeof(oil_event_data));
            memcpy(&oil_event_data, event_ptr, len);
            DataAll temp_data_all;
            {
                std::lock_guard<std::mutex> __lk(m_data_mutex);
                memcpy(&temp_data_all, &m_collect_data, sizeof(DataAll));
            }
            EventGeneral temp_general;
            memcpy(temp_general.event_start_time, oil_event_data.start_time, BCD_TIME_LENGTH);
            memcpy(temp_general.event_end_time, oil_event_data.end_time, BCD_TIME_LENGTH);
            temp_general.duration_time = htobe16(oil_event_data.duration_time);
            temp_general.duration_length = htobe16(oil_event_data.duration_length);
            temp_general.alarm_father_type = 0x06; // 6.    油箱防盗事件

            out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&temp_data_all, (SMLK_UINT8 *)&temp_data_all + sizeof(DataAll));
            out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&temp_general, (SMLK_UINT8 *)&temp_general + sizeof(EventGeneral));

            struct
            {
                SMLK_UINT32 alarm_id;
                /**
                 * 0x00：不可用
                 * 0x01：开始标志
                 * 0x02：结束标志
                 * 该字段仅适用于有开始和结束标志类型的报警或事件，报警类型或事件类型无开始和结束标志，则该位不可用，填入0x00即可；（Tbox默认写00
                 */
                SMLK_UINT8 tag_state;
                SMLK_UINT16 alarm_type;
                SMLK_UINT8 file_count;
                SMLK_UINT8 alarm_tag[30];
            } __attribute__((__packed__)) oil_808_data;
            bzero(&oil_808_data, sizeof(oil_808_data));
            oil_808_data.alarm_id = htobe16(oil_event_data.alarm_id);
            oil_808_data.alarm_type = htobe16(0x5001); // 油箱防盗报警事件
            oil_808_data.file_count = oil_event_data.file_count;
            memcpy(oil_808_data.alarm_tag, oil_event_data.alarm_tag, ALARM_TAG_LENGTH);
            out_vec.insert(out_vec.end(), (SMLK_UINT8 *)&oil_808_data, (SMLK_UINT8 *)&oil_808_data + sizeof(oil_808_data));
            return out_vec.size();
        }

        std::size_t DataProcessor::DealFaceRecEvent(IN SMLK_UINT8 *event_ptr, std::size_t len, OUT std::vector<SMLK_UINT8> &out_vec)
        {
            return out_vec.size();
        }

        void DataProcessor::UpdateCollectData()
        {
            SMLK_LOGD("enter in func:%s()", __func__);
            smartlink_sdk::LocationInfo location_info;
            auto return_code = smartlink_sdk::Location::GetInstance()->GetLocation(location_info);
            if (smartlink_sdk::RtnCode::E_SUCCESS != return_code)
            {
                SMLK_LOGE("UpdateCollectData active getting location info!!! return code: %d", static_cast<std::int32_t>(return_code));
                return;
            }
            {
                std::lock_guard<std::mutex> __lk(m_data_mutex);
                m_collect_data.longitude = htobe16(location_info.longitude);
                m_collect_data.latitude = htobe16(location_info.latitude);
                m_collect_data.altitude = htobe16((SMLK_UINT16)location_info.altitude);
                m_collect_data.gps_speed = htobe16((SMLK_UINT16)location_info.speed);
                m_collect_data.direction = htobe16((SMLK_UINT16)location_info.heading);
                memcpy(&m_collect_data.vehicle_status, &m_atm_vehicle_status, sizeof(SMLK_UINT32));
                m_collect_data.vehicle_status = htobe32(m_collect_data.vehicle_status);
                get_bcd_timestamp(m_collect_data.upload_time);
                for (auto &pair : m_signals)
                {
                    CalcSignalVal(pair.first);
                }
            }
        }

        void DataProcessor::CalcSignalVal(SMLK_UINT32 id)
        {
            auto it = m_signals.find(id);
            if (it == m_signals.end())
            {
                return;
            }
            switch (id)
            {
            case VEHICLE_DATA_TACHOGRAPH_VEHICLE_SPEED:
            {
                if (it->second.Valid())
                {
                    m_collect_data.can_speed = (SMLK_UINT16)(it->second.Encoded() * 10); // 0.1km/h，上传值为速度值乘以10
                }
                else
                {
                    m_collect_data.can_speed = 0x0000;
                }
            }
            break;
            case VEHICLE_DATA_CURRENT_GEAR:
            {
                if (it->second.Valid())
                {
                    m_collect_data.gear_status = (SMLK_UINT8)it->second.Encoded();
                }
                else
                {
                    m_collect_data.gear_status = 0xFF;
                }
            }
            break;
            case VEHICLE_DATA_ACCELERATOR_PEDAL_POSITION:
            {
                if (it->second.Valid())
                {
                    m_collect_data.accelerator_pedal = (SMLK_UINT8)it->second.Encoded();
                }
                else
                {
                    m_collect_data.accelerator_pedal = 0xFF;
                }
            }
            break;
            case VEHICLE_DATA_BRAKE_PEDAL_POSITION:
            {
                if (it->second.Valid())
                {
                    m_collect_data.broken_pedal = (SMLK_UINT8)it->second.Encoded();
                }
                else
                {
                    m_collect_data.broken_pedal = 0xFF;
                }
            }
            break;
            case VEHICLE_DATA_BRAKE_SWITCH:
            {
                if (it->second.Valid())
                {
                    m_collect_data.broken_status = (SMLK_UINT8)it->second.Encoded();
                }
                else
                {
                    m_collect_data.broken_status = 0xFF;
                }
            }
            break;
            case VEHICLE_DATA_ENGINE_SPEED:
            {
                if (it->second.Valid())
                {
                    m_collect_data.engine_round_speed = (SMLK_UINT16)it->second.Encoded();
                }
                else
                {
                    m_collect_data.engine_round_speed = 0xFFFF;
                }
            }
            break;
            case VEHICLE_DATA_STEERING_WHEEL_ANGLE:
            {
                if (it->second.Valid())
                {
                    m_collect_data.steering_wheel_angle = (SMLK_UINT16)it->second.Encoded();
                }
                else
                {
                    m_collect_data.steering_wheel_angle = 0xFFFF;
                }
            }
            break;
            default:
                break;
            }
        }

        SMLK_RC DataProcessor::Init()
        {
            if (CheckStatus(BaseModule::Initialized))
            {
                SMLK_LOGV("DataProcessor already initialized, do nothing");
                return SMLK_RC::RC_OK;
            }
            auto rc = VehicleDataApi::GetInstance()->Init(ModuleID::E_MOUDLE_AVM_CTRL_APP);
            if (SMLK_RC::RC_OK != rc)
            {
                SMLK_LOGE("DataProcessor fail to init vehicle data service API interface!!!");
                return SMLK_RC::RC_ERROR;
            }

            std::vector<SMLK_UINT32> indexs = {
                VEHICLE_DATA_TACHOGRAPH_VEHICLE_SPEED,   // can-speed
                VEHICLE_DATA_CURRENT_GEAR,               // gear status
                VEHICLE_DATA_ACCELERATOR_PEDAL_POSITION, // 加速踏板开合度
                VEHICLE_DATA_BRAKE_PEDAL_POSITION,       // 制动踏板开合度
                VEHICLE_DATA_BRAKE_SWITCH,               // 制动状态
                VEHICLE_DATA_ENGINE_SPEED,               // 发动机转速
                VEHICLE_DATA_STEERING_WHEEL_ANGLE        // 方向盘角度
                /*corner_lamp_status*/
            };
            rc = VehicleDataApi::GetInstance()->RegistVehicleDataChangedCB(
                indexs,
                [this](VehicleData data)
                {
                    m_vehicle_data_que.put(data);
                });
            if (SMLK_RC::RC_OK != rc)
            {
                SMLK_LOGE("DataProcessor fail to register vehicle data changed callback");
                return SMLK_RC::RC_ERROR;
            }

            SMLK_BOOL ret = Location::GetInstance()->Init(ModuleID::E_MOUDLE_AVM_CTRL_APP);
            if (!ret)
            {
                SMLK_LOGE("DataProcessor fail to init Location service API interface!!!");
                return SMLK_RC::RC_ERROR;
            }

            //     std::vector<LocationEventId> loc_events = {
            //         LocationEventId::E_LOCATION_EVENT_ISREADY,
            //         LocationEventId::E_LOCATION_EVENT_EXIT,
            //         LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED,
            //     };

            //     auto return_code = smartlink_sdk::Location::GetInstance()->RegEventCallback(
            //         loc_events,
            //         [this](smartlink_sdk::LocationEventId id, void *data, int len) {
            //             if (!data) return;
            //             std::vector<SMLK_UINT8> bytes;

            //             switch ( id ) {
            //                 case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_ISREADY: // location service ready
            //                     break;
            //                 case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_EXIT: // location service exit
            //                     break;
            //                 case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED:
            //                 {
            //                     if ( sizeof(smartlink_sdk::LocationInfo) != (std::size_t)len ) {
            //                         SMLK_LOGE("[GNSS] unexpected body length for FIX INFO CHNAGED notification: %d, expected: %d",
            //                                                                             len, (int)sizeof(smartlink_sdk::LocationInfo));
            //                         return;
            //                     }
            //                     {
            //                         smartlink_sdk::LocationInfo* info_ = (smartlink_sdk::LocationInfo*)data;
            //                         if (info_->longitude > 0) // bit3-- 0：东经；1：西经
            //                         {
            //                             m_atm_vehicle_status = m_atm_vehicle_status & 0x00000007/*0111*/;
            //                         }
            //                         else
            //                         {
            //                             m_atm_vehicle_status = m_atm_vehicle_status & 0x0000000F/*1111*/;
            //                         }

            //                         if (info_->latitude > 0) // bit2-- 0：北纬；1：南纬
            //                         {
            //                             m_atm_vehicle_status = m_atm_vehicle_status & 0x0000000B/*1011*/;
            //                         }
            //                         else
            //                         {
            //                             m_atm_vehicle_status = m_atm_vehicle_status & 0x0000000F/*1111*/;
            //                         }

            //                         m_atm_vehicle_status = m_atm_vehicle_status & (info_->fix_valid ? 0x0000000F/*1111*/: 0x0000000D/*1101*/); // bit1-- 0：未定位；1：已定位

            //                         std::lock_guard<std::mutex> __lk(m_data_mutex);
            //                         m_collect_data.longitude = info_->longitude;
            //                         m_collect_data.latitude  = info_->latitude;
            //                         m_collect_data.altitude  = info_->altitude;
            //                         m_collect_data.gps_speed = info_->speed;
            //                         m_collect_data.direction = info_->heading;
            //                     }
            // #if 0
            //                     SMLK_LOGD("[GNSS] fix mode changed notification received");
            //                     SMLK_LOGD("\tinf->speed:        %f",  m_collect_data.gps_speed);
            //                     SMLK_LOGD("\tinf->heading:      %f",  m_collect_data.direction);
            //                     SMLK_LOGD("\tinf->altitude:     %f",  m_collect_data.altitude);
            //                     SMLK_LOGD("\tinf->latitude:     %f",  m_collect_data.latitude);
            //                     SMLK_LOGD("\tinf->longitude:    %f",  m_collect_data.longitude);
            //                     // SMLK_LOGD("\tinf->utc:          %lu", m_collect_data.utc);
            // #endif
            //                 }
            //                     break;
            //                 default:
            //                     SMLK_LOGE("unsupported event id: %u", static_cast<SMLK_UINT32>(id));
            //                     return;
            //             }
            //         }
            //     );

            //     if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
            //         SMLK_LOGE("fail to init register event callback to location service!!! return code: %d", static_cast<std::int32_t>(return_code));
            //         return SMLK_RC::RC_ERROR;
            //     }

            if (!smartlink_sdk::SysPower::GetInstance()->Init(ModuleID::E_MOUDLE_AVM_CTRL_APP))
            {
                SMLK_LOGF("fail to init SysPower service API interface.");
                return SMLK_RC::RC_ERROR;
            }
            else
            {

                std::vector<smartlink_sdk::SysPowerEventID> power_event_vec = {
                    smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP,
                    smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP,
                };
                smartlink_sdk::SysPower::GetInstance()->RegEventCB(power_event_vec, [this](smartlink_sdk::SysPowerEventID event_id, void *, int)
                                                                   {
                    SMLK_LOGD("*****avm msg from power Event_id=%d.", event_id);
                    switch (event_id)
                    {
                    case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP:
                    {
                        m_atm_ig_status.store(1);
                        m_atm_vehicle_status = m_atm_vehicle_status & 0x0000000F /*1111*/; // bit0 -- 0：ACC关；1：ACC开（用IGN表示
                    }
                    break;
                    case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP:
                    {
                        m_atm_ig_status.store(0);
                        m_atm_vehicle_status = m_atm_vehicle_status & 0x0000000E /*1110*/; // bit0 -- 0：ACC关；1：ACC开（用IGN表示
                    }
                    break;
                    default:
                        SMLK_LOGE("[POWER] unsupported event id: %u", (SMLK_UINT32)event_id);
                        return;
                    } });
                SMLK_LOGD("SysPower->Init() OK.");
            }
            SetStatus(BaseModule::Initialized);
            return SMLK_RC::RC_OK;
        }

        void DataProcessor::DoParseVehicle()
        {
            while (!CheckStatus(BaseModule::Stopping))
            {
                VehicleData v_data = m_vehicle_data_que.get();
                SMLK_LOGD("DataProcessor get signal index: [%u%]", v_data.index);
                const double *ptr = reinterpret_cast<const double *>(&v_data.value);
                SMLK_UINT32 index = v_data.index;
                bool valid = v_data.valid;
                auto it = m_signals.find(index);
                if (m_signals.cend() != it)
                {
                    if (it->second.Valid() && !valid)
                    {
                        SMLK_LOGW("signal changed:  VALID ---> INVALID, signal index: %u", index);
                    }
                    else if (!it->second.Valid() && valid)
                    {
                        SMLK_LOGW("signal changed:  INVALID ---> VALID, signal index: %u val = %lf", index, *ptr);
                    }
                    it->second = *ptr;
                    if (!valid)
                    {
                        it->second.Invalid();
                    }
                }
            }
        }

        void DataProcessor::SimuAvmInputMsg(std::string simu_msg, OUT std::vector<SMLK_UINT8> &event_data)
        {
            int char_len = simu_msg.length() / 2;
            for (auto i = 0; i < char_len; ++i)
            {
                std::string str_temp = simu_msg.substr(i * 2, 2);
                SMLK_UINT8 c;
                std::size_t fc, sc;
                hex2int(str_temp[0], fc);
                hex2int(str_temp[1], sc);
                c = (((SMLK_UINT8)fc & 0x0F) << 4) | ((SMLK_UINT8)sc & 0x0F);
                event_data.insert(event_data.end(), &c, &c + sizeof(SMLK_UINT8));
            }
        }

        void DataProcessor::hex2int(char c, OUT std::size_t &int_o)
        {
            int_o = ((c >= '0') && (c <= '9')) ? std::size_t(c - '0') : ((c >= 'A') && (c <= 'F')) ? std::size_t(c - 'A' + 10)
                                                                    : ((c >= 'a') && (c <= 'f'))   ? std::size_t(c - 'a' + 10)
                                                                                                   : (std::size_t)-1;
        }

        SMLK_RC DataProcessor::Start()
        {
            if (IsRunning())
            {
                SMLK_LOGV("DataProcessor already started, do nothing.\n");
                return SMLK_RC::RC_OK;
            }
            SetStatus(BaseModule::Running);
            m_vehicle_thread = std::thread(&DataProcessor::DoParseVehicle, this);
            return SMLK_RC::RC_OK;
        }

        void DataProcessor::Stop()
        {
            SetStatus(BaseModule::Stopping);
            if (m_vehicle_thread.joinable())
            {
                m_vehicle_thread.join();
            }
        }

        DataProcessor::DataProcessor()
            : m_atm_ig_status(0)
        {
            SetStatus(BaseModule::Uninitialized);
        }

        DataProcessor::~DataProcessor()
        {
        }

    }
}
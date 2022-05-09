#pragma once
#include <memory>
#include <string>
#include <set>
#include <vector>
#include <unordered_map>
#include <smlk_fixed_queue.h>
#include "errors_def.h"
#include "sl_signals.h"
#include "message_inf.h"

#define M_SL_ACCIDENT_FRAMES    60
#define M_SL_TIMESTAMP_SZ       6

namespace smartlink {



class LocationInfo {
public:
    LocationInfo();
    ~LocationInfo();
    LocationInfo(IN LocationInfo &other);
    LocationInfo & operator=(IN LocationInfo &other);
public:
    vehicle::SignalVariant  m_status;
    vehicle::SignalVariant  m_heading;
    vehicle::SignalVariant  m_latitude;
    vehicle::SignalVariant  m_longitude;
    vehicle::SignalVariant  m_altitude;
    vehicle::SignalVariant  m_speed;
    vehicle::SignalVariant  m_timestamp;
};

class RegionInfo {
public:
    RegionInfo();
    ~RegionInfo();
    RegionInfo(IN RegionInfo &other);
    RegionInfo & operator=(IN RegionInfo &other);
public:
    SMLK_UINT32 m_id;
    SMLK_UINT8  m_type;
    SMLK_UINT8  m_direction;
};

class TireInfo {
public:
    TireInfo();
    ~TireInfo();
    TireInfo(IN TireInfo &other);
    TireInfo & operator=(IN TireInfo &ohter);
public:
    SMLK_UINT8              m_id;
    vehicle::SignalVariant  m_temperature;
    vehicle::SignalVariant  m_pressure;
    vehicle::SignalVariant  m_extend_pressure;
};

class AccidentFrame {
public:
    AccidentFrame();
    ~AccidentFrame();
    AccidentFrame(  IN SMLK_DOUBLE  braking_system_state,
                    IN SMLK_DOUBLE  braking_system_target_id,
                    IN SMLK_DOUBLE  target_relative_speed,
                    IN SMLK_DOUBLE  target_speed,
                    IN SMLK_DOUBLE  target_distance );
    AccidentFrame(IN AccidentFrame &other);
    AccidentFrame & operator=(IN AccidentFrame &other);
public:
    vehicle::SignalVariant  m_emergency_braking_state;
    vehicle::SignalVariant  m_emergency_braking_target_id;
    vehicle::SignalVariant  m_target_relative_speed;
    vehicle::SignalVariant  m_target_speed;
    vehicle::SignalVariant  m_target_distance;
};

class FaultCode {
public:
    FaultCode();
    FaultCode(SMLK_UINT8 source, SMLK_UINT8 FMI, SMLK_UINT32 SPN);
    ~FaultCode();

    FaultCode(IN FaultCode &other);
    FaultCode & operator=(IN FaultCode &other);

public:
    SMLK_UINT8      m_source;
    SMLK_UINT8      m_FMI;          // failure mode identifier
    SMLK_UINT32     m_SPN;
};

class MessageDM1 : public IMessage {
public:
    static const SMLK_UINT8 FAULT_OCCURS    = 0x01;
    static const SMLK_UINT8 FAULT_CLEAR     = 0x02;
public:
    MessageDM1();
    MessageDM1(SMLK_UINT8, std::vector<FaultCode> &&);
    ~MessageDM1();

    MessageDM1(IN MessageDM1 &);
    MessageDM1(MessageDM1 &&);
    MessageDM1 & operator=(IN MessageDM1 &);
    MessageDM1 & operator=(MessageDM1 &&);
public:
    SMLK_UINT8              m_type;                     // 1: fault occurs; 2: fault clear
    vehicle::SignalVariant  m_tachograph_speed;         // vehicle speed
    vehicle::SignalVariant  m_acc_pedal_travel;         // acc pedal travel
    vehicle::SignalVariant  m_brake_pedal_state;        // brake pedal state
    vehicle::SignalVariant  m_engine_speed;             // engine speed
    vehicle::SignalVariant  m_tubocharing_pressue;      // tubocharing pressure
    vehicle::SignalVariant  m_intake_manifold_pressure; // intake manifole pressure
    vehicle::SignalVariant  m_exhaust_gas_temperature;  // exhaust gas tempereature
    vehicle::SignalVariant  m_coolant_temperature;      // coolant temperture
    vehicle::SignalVariant  m_acc_change_rate;          // acc change rate
    vehicle::SignalVariant  m_gear;                     // gear
    vehicle::SignalVariant  m_torque_percent;           // torque percent
    vehicle::SignalVariant  m_vehicle_load;             // vehicle load
    vehicle::SignalVariant  m_engine_load;              // engine load
    vehicle::SignalVariant  m_acceleration;             // acceleration
    vehicle::SignalVariant  m_deceleration;             // deceleration
    std::vector<FaultCode>  m_faults;
    std::set<SMLK_UINT32>   m_reported_faults;
    SMLK_UINT8              m_add_extra_info_flag;
};

class MessageLocation : public IMessage {
public:
    MessageLocation();
    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}

    MessageLocation & operator=(IN MessageLocation &other);

protected:
    static const SMLK_UINT8 EXT_MILAGE;
    static const SMLK_UINT8 EXT_ENTER_LEAVE_AREA;
    static const SMLK_UINT8 EXT_SIGNAL_STATUS;
    static const SMLK_UINT8 EXT_RSSI;
    static const SMLK_UINT8 EXT_GNSS_SATELLITES;
    static const SMLK_UINT8 EXT_E6;
    static const SMLK_UINT8 EXT_E7;
    static const SMLK_UINT8 EXT_EB;
    // Add 0xEC 2022-03-08
    static const SMLK_UINT8 EXT_EC;

public:
    bool                    m_report_location_only;
    vehicle::SignalVariant  m_alarm_flags;
    vehicle::SignalVariant  m_tachograph_vehicle_speed;
    vehicle::SignalVariant  m_engine_torque_mode;
    vehicle::SignalVariant  m_torque_percent_cmd;
    vehicle::SignalVariant  m_torque_percent;
    vehicle::SignalVariant  m_engine_speed;
    vehicle::SignalVariant  m_coolant_temperature;
    vehicle::SignalVariant  m_oil_pressure;
    vehicle::SignalVariant  m_engine_total_fuel_used;
    vehicle::SignalVariant  m_acc_pedal_travel;
    vehicle::SignalVariant  m_parking;
    vehicle::SignalVariant  m_wheel_speed;
    vehicle::SignalVariant  m_clutch;
    vehicle::SignalVariant  m_brake_pedal_state;
    vehicle::SignalVariant  m_urea_tank_level;
    vehicle::SignalVariant  m_engine_total_time;
    vehicle::SignalVariant  m_engine_total_cycles;
    vehicle::SignalVariant  m_engine_fuel_consumption_rate;
    vehicle::SignalVariant  m_engine_instantaneous_fuel_consumption;
    vehicle::SignalVariant  m_engine_average_fuel_consumption;
    vehicle::SignalVariant  m_dpf_pressure_differential;
    vehicle::SignalVariant  m_intake_manifold_pressure;
    vehicle::SignalVariant  m_intake_manifole_temprerature;
    vehicle::SignalVariant  m_engine_intake_pressure;
    vehicle::SignalVariant  m_barometric_pressure;
    vehicle::SignalVariant  m_cab_interior_temperature;
    vehicle::SignalVariant  m_ambient_air_temperature;
    vehicle::SignalVariant  m_engine_intake_temperature;
    vehicle::SignalVariant  m_cold_start_light;
    vehicle::SignalVariant  m_water_in_fuel_flag;
    vehicle::SignalVariant  m_target_gear;
    vehicle::SignalVariant  m_gear_ratio;
    vehicle::SignalVariant  m_gear;
    vehicle::SignalVariant  m_instrument_fuel_level;
    vehicle::SignalVariant  m_instrument_subtotal_mileage;
    vehicle::SignalVariant  m_instrument_total_mileage;
    vehicle::SignalVariant  m_instrument_total_mileage_head_ext1;
    vehicle::SignalVariant  m_calculus_distance;
    vehicle::SignalVariant  m_calculus_fuel_used;
    vehicle::SignalVariant  m_actural_catalyst_spewed;
    vehicle::SignalVariant  m_total_catalyst_spewed;
    vehicle::SignalVariant  m_acc_status;
    vehicle::SignalVariant  m_extended_signal_status;
    vehicle::SignalVariant  m_rssi;
    vehicle::SignalVariant  m_satellites_used;

    vehicle::SignalVariant  m_battery_1_temperature;
    vehicle::SignalVariant  m_battery_1_vol;
    vehicle::SignalVariant  m_battery_1_soc;
    vehicle::SignalVariant  m_battery_1_soh;
    vehicle::SignalVariant  m_battery_2_temperature;
    vehicle::SignalVariant  m_battery_2_vol;
    vehicle::SignalVariant  m_battery_2_soc;
    vehicle::SignalVariant  m_battery_2_soh;
    vehicle::SignalVariant  m_actual_minimum_soc;
    vehicle::SignalVariant  m_actual_minimum_soh;
    vehicle::SignalVariant  m_agtator_state;
    vehicle::SignalVariant  m_agtator_speed;
    vehicle::SignalVariant  m_agtator_total_rotations;

    vehicle::SignalVariant  m_catalyst_tank_temperature;            //尿素罐温度
    vehicle::SignalVariant  m_engine_power_input1;                  //发动机输入电压
    vehicle::SignalVariant  m_keyswitch_battery_voltage;            //点火开关电压
    vehicle::SignalVariant  m_exhaust_gas_temperature;              //发动机排气温度
    vehicle::SignalVariant  m_sensor1_humidity;                     //传感器1湿度
    vehicle::SignalVariant  m_sensor2_humidity;                     //传感器2
    vehicle::SignalVariant  m_sensor3_humidity;                     //传感器3
    vehicle::SignalVariant  m_sensor4_humidity;                     //传感器4
    vehicle::SignalVariant  m_sensor5_humidity;                     //传感器5
    vehicle::SignalVariant  m_sensor6_humidity;                     //传感器6
    vehicle::SignalVariant  m_sensor7_humidity;                     //传感器7
    vehicle::SignalVariant  m_sensor8_humidity;                     //传感器8
    vehicle::SignalVariant  m_sensor1_temperature;                  //传感器1温度
    vehicle::SignalVariant  m_sensor2_temperature;                  //传感器2
    vehicle::SignalVariant  m_sensor3_temperature;                  //传感器3
    vehicle::SignalVariant  m_sensor4_temperature;                  //传感器4
    vehicle::SignalVariant  m_sensor5_temperature;                  //传感器5
    vehicle::SignalVariant  m_sensor6_temperature;                  //传感器6
    vehicle::SignalVariant  m_sensor7_temperature;                  //传感器7
    vehicle::SignalVariant  m_sensor8_temperature;                  //传感器8
    vehicle::SignalVariant  m_pefrigeration_switch_status;          //制冷机组开关状态
    vehicle::SignalVariant  m_pefrigeration_work_mode;              //制冷机组工作模式
    vehicle::SignalVariant  m_pefrigeration_cpmpartment_temperature;//冷藏室温度
    vehicle::SignalVariant  m_defrost_temperature;                  //除霜温度
    vehicle::SignalVariant  m_supply_voltage;                       //供电电压
    // update 0200-e7
    vehicle::SignalVariant  m_dc_ac_power_consumption;              //逆变器功率
    vehicle::SignalVariant  m_steerwheel_heat_power_consumption;    //方向盘加热功率
    vehicle::SignalVariant  m_cigar_light_power_consumption;        //点烟器功率
    vehicle::SignalVariant  m_24v_socket1_power_consumption;        //24V 供电插座1功率
    vehicle::SignalVariant  m_24v_socket2_power_consumption;        //24V 供电插座2功率
    vehicle::SignalVariant  m_left_seat_power_consumption;          //驾驶员座椅加热功率
    vehicle::SignalVariant  m_right_seat_power_consumption;         //副驾驶员座椅加热功率
    vehicle::SignalVariant  m_lower_bretht_heat_power_consumption;  //下卧铺加热功率
    vehicle::SignalVariant  m_sleep_read_lamp_power_consumption;    //卧铺灯功率

    vehicle::SignalVariant  m_elevated_box_power_consumption;       //高架箱灯功率
    vehicle::SignalVariant  m_humidity_in_box1;                     //箱内湿度 1
    vehicle::SignalVariant  m_humidity_in_box2;                     //箱内湿度 2
    vehicle::SignalVariant  m_temperature_in_box1;                  //箱内温度 1
    vehicle::SignalVariant  m_temperature_in_box2;                  //箱内温度 2
    // add 0200-EC 2022-03-08
    vehicle::SignalVariant  m_tbox_voltage; // Tbox 供电电压
    vehicle::SignalVariant  m_fuel_save_switch; // 节油开关
    vehicle::SignalVariant  m_hydraulic_motor_oil_return_temperature; // 液压马达回油温
    vehicle::SignalVariant  m_ev_vehicle_status; // 车辆状态 新能源
    vehicle::SignalVariant  m_ev_charging_status; // 充电状态
    vehicle::SignalVariant  m_ev_working_status;  // 运行状态
    vehicle::SignalVariant  m_ev_vehicle_speed;  // 车速 新能源
    vehicle::SignalVariant  m_ev_total_voltage; // 总电压
    vehicle::SignalVariant  m_ev_total_current; // 总电流
    vehicle::SignalVariant  m_ev_battery_soc; // SOC
    vehicle::SignalVariant  m_ev_dc_working_status; // DCDC状态
    vehicle::SignalVariant  m_ev_gear_status; // 档位 daq组合计算
    vehicle::SignalVariant  m_ev_insulation_resistance; // 绝缘电阻

    RegionInfo              m_region;
    LocationInfo            m_location;

    std::vector<TireInfo>   m_tires;
};

class MessageBucket : public IMessage {
public:
    MessageBucket();
    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::size_t     CompressOffset() const { return 0; };

    MessageBucket & operator=(const MessageBucket &other);

    static const std::vector<vehicle::SignalID> & SignalIDs();
    static const std::unordered_map<vehicle::SignalID, SMLK_UINT8> &SignalsPrecision();
public:
    SMLK_UINT16             m_sampling_frequency;
    SMLK_UINT8              m_crompression;
    SMLK_UINT8              i_lckcar_sts;
    vehicle::SignalVariant  m_signal_day_vehicle_distance;
    vehicle::SignalVariant  m_signal_total_vehicle_distance;
    vehicle::SignalVariant  m_signal_ecu_total_distacne;
    vehicle::SignalVariant  m_signal_day_fuel_used;
    vehicle::SignalVariant  m_signal_total_fuel_used;
    vehicle::SignalVariant  m_signal_ecu_total_fuel_used;
    std::vector<std::unordered_map<vehicle::SignalID, vehicle::SignalVariant>>  m_signals;
};

class MessageBucket30S : public IMessage {
public:
    MessageBucket30S();
    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::size_t     CompressOffset() const { return 0; };

    MessageBucket30S & operator=(const MessageBucket30S &other);

    static const std::vector<vehicle::SignalID> & SignalIDs();
    static const std::unordered_map<vehicle::SignalID, SMLK_UINT8> &SignalsPrecision();
public:
    SMLK_UINT16             m_sampling_frequency;
    SMLK_UINT8              m_crompression;
    vehicle::SignalVariant  m_signal_day_vehicle_distance;
    vehicle::SignalVariant  m_signal_total_vehicle_distance;
    vehicle::SignalVariant  m_signal_ecu_total_distacne;
    vehicle::SignalVariant  m_signal_day_fuel_used;
    vehicle::SignalVariant  m_signal_total_fuel_used;
    vehicle::SignalVariant  m_signal_ecu_total_fuel_used;
    std::vector<std::unordered_map<vehicle::SignalID, vehicle::SignalVariant>>  m_signals;
};

class MessageCompressedBucket : public IMessage {
public:
    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}

    MessageCompressedBucket & operator=(IN MessageCompressedBucket &other);

public:
    SMLK_UINT16             m_sampling_frequency;
    SMLK_UINT8              m_crompression;
    std::vector<SMLK_UINT8> m_compressed;
};

class MessageStatistics : public IMessage {
public:
    MessageStatistics();

    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}

    MessageStatistics & operator=(IN MessageStatistics &other);

public:
    SMLK_UINT8              m_flags;
    vehicle::SignalVariant  m_trip_start;
    vehicle::SignalVariant  m_trip_stop;
    vehicle::SignalVariant  m_gnss_statistics_distance;
    vehicle::SignalVariant  m_tachograph_vehicle_speed_statistics_distance;
    vehicle::SignalVariant  m_vehicle_speed_statistics_life_total_distance;
    vehicle::SignalVariant  m_vehicle_total_distance;
    vehicle::SignalVariant  m_vehicle_estimated_load;
    vehicle::SignalVariant  m_trip_statistics_fuel_used;
    vehicle::SignalVariant  m_total_statistics_fuel_used;
    vehicle::SignalVariant  m_engine_total_fuel_used;
    vehicle::SignalVariant  m_engine_average_fuel_economy;
    vehicle::SignalVariant  m_speeding_times;
    vehicle::SignalVariant  m_speeding_distance;
    vehicle::SignalVariant  m_speeding_fuel_used;
    vehicle::SignalVariant  m_brake_pedal_times;
    vehicle::SignalVariant  m_brake_pedal_brake_distance;
    vehicle::SignalVariant  m_brake_pedal_brake_duration;
    vehicle::SignalVariant  m_sharp_slowdown_times;
    vehicle::SignalVariant  m_sharp_slowdown_duration;
    vehicle::SignalVariant  m_sharp_slowdown_distance;
    vehicle::SignalVariant  m_sharp_speedup_times;
    vehicle::SignalVariant  m_sharp_speedup_duration;
    vehicle::SignalVariant  m_sharp_speedup_distance;
    vehicle::SignalVariant  m_sharp_speedup_fuel_used;
    vehicle::SignalVariant  m_sharp_turnning_times;
    vehicle::SignalVariant  m_sharp_turnning_distance;
    vehicle::SignalVariant  m_sleepy_drive_times;
    LocationInfo            m_location;
};

class MessageEvent : public IMessage {
public:
    static const SMLK_UINT8 SOC_STATUS_INVALID                  = 0x00;
    static const SMLK_UINT8 SOC_STATUS_LOW_WARNING              = 0x01;
    static const SMLK_UINT8 SOC_STATUS_LOW_ALARM                = 0x02;
    static const SMLK_UINT8 SOC_STATUS_OVERCHARGE               = 0x03;

    static const SMLK_UINT8 DMS_WAR_EYES_CLOSED                 = 0x00;
    static const SMLK_UINT8 DMS_WAR_DOZING                      = 0x01;
    static const SMLK_UINT8 DMS_WAR_SMOKING                     = 0x02;
    static const SMLK_UINT8 DMS_WAR_CALL                        = 0x03;
    static const SMLK_UINT8 DMS_WAR_INFRARED_INTERDICT          = 0x04;
    static const SMLK_UINT8 DMS_WAR_MONITOR_INTERDICT           = 0x05;
    static const SMLK_UINT8 DMS_WAR_DRIVER_UNDETECTED           = 0x06;
    static const SMLK_UINT8 DMS_WAR_MILD_FATIGUE                = 0x07;
    static const SMLK_UINT8 DMS_WAR_SEVERE_FATIGUE              = 0x08;
    static const SMLK_UINT8 DMS_WAR_MILD_INATTENTIVE            = 0x09;
    static const SMLK_UINT8 DMS_WAR_SEVERE_INATTENTIVE          = 0x0A;

    static const SMLK_UINT8 AGITATOR_WAR_SIG1_OPEN              = 0x00;
    static const SMLK_UINT8 AGITATOR_WAR_SIG1_SHORT_POWER       = 0x01;
    static const SMLK_UINT8 AGITATOR_WAR_SIG1_SHORT_GROUND      = 0x02;
    static const SMLK_UINT8 AGITATOR_WAR_SIG2_OPEN              = 0x03;
    static const SMLK_UINT8 AGITATOR_WAR_SIG2_SHORT_POWER       = 0x04;
    static const SMLK_UINT8 AGITATOR_WAR_SIG2_SHORT_GROUND      = 0x05;
    static const SMLK_UINT8 AGITATOR_WAR_SIG1_SHORT_SIG2        = 0x06;
    static const SMLK_UINT8 AGITATOR_WAR_SIG1_SIG2_SEQ_REVERSED = 0x07;
    static const SMLK_UINT8 AGITATOR_WAR_SENSOR_GAP_BIG         = 0x08;
    static const SMLK_UINT8 AGITATOR_WAR_SENSOR_GAP_SMALL       = 0x09;
    static const SMLK_UINT8 AGITATOR_WAR_SENSOR_INACTIVE        = 0x0A;

    static const SMLK_UINT8 TIRE_PRESSURE_NORMAL                = 0x00;
    static const SMLK_UINT8 TIRE_PRESSURE_HIGH                  = 0x02;
    static const SMLK_UINT8 TIRE_PRESSURE_LOW                   = 0x05;
    static const SMLK_UINT8 TIRE_PRESSURE_INVALID               = 0x0D;

    static const SMLK_UINT8 TIRE_AIR_LEAKAGE_UNDETACT           = 0x00;
    static const SMLK_UINT8 TIRE_AIR_LEAKAGE_DETACT             = 0x01;
    static const SMLK_UINT8 TIRE_AIR_LEAKAGE_INVALID            = 0x02;

public:
    MessageEvent();
    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}

    MessageEvent & operator=(IN MessageEvent &other);

public:
    SMLK_UINT8                  m_id;
    SMLK_UINT8                  m_reversion;
    SMLK_UINT32                 m_status;
    SMLK_UINT32                 m_type; // for accident report
    vehicle::SignalVariant      m_speed_start;
    vehicle::SignalVariant      m_speed_stop;
    vehicle::SignalVariant      m_duration;
    vehicle::SignalVariant      m_distance;
    vehicle::SignalVariant      m_fuel_used;
    vehicle::SignalVariant      m_total_distance;

    vehicle::SignalVariant      m_actual_minimum_soc;
    TireInfo                    m_tire_info;

    std::vector<LocationInfo>   m_locations;
    FixedQueue<AccidentFrame, M_SL_ACCIDENT_FRAMES> m_frames;
};

class MessageLocationRsp : public IMessage {
public:
    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}

    MessageLocationRsp & operator=(IN MessageLocationRsp &other);

public:
    SMLK_UINT16     m_seq;
    MessageLocation m_message_location;
};

class MessageFault : public IMessage {
public:
    static const SMLK_UINT8 MSG_RES         = 0x00;
    static const SMLK_UINT8 MSG_DM1         = 0x01;
    static const SMLK_UINT8 MSG_ENG         = 0x02;
    static const SMLK_UINT8 MSG_UDS         = 0x03;
    static const SMLK_UINT8 MSG_ABN         = 0x04;
public:
    MessageFault();
    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}

    void ResetMessage() noexcept;
    SMLK_UINT32 ResetMessage(SMLK_UINT8, std::shared_ptr<IMessage>);

    MessageFault & operator=(IN MessageFault &other);

public:
    SMLK_UINT8      m_reversion;
    LocationInfo    m_location;
    SMLK_UINT8      m_type;

protected:
    std::shared_ptr<IMessage> m_fault;
};

}   // namespace smartlink

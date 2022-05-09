/******************************************************************************
*
*  Copyright (C) 2021 SmartLink
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include <set>
#include <map>
#include <array>
#include <vector>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <time.h>
#include <json/json.h>

#include <smlk_spn.h>
#include <smlk_log.h>
#include <smlk_tools.h>

#include <vehicle_data_api.h>
#include <vehicle_data_index_def.h>
#include <smartlink_sdk_mcu.h>
#include <smartlink_sdk_tel.h>
#include <smartlink_sdk_sys_power.h>
#include <smartlink_sdk_location.h>
#include <smartlink_sdk_sys_property.h>
#include <smartlink_sdk_sys_property_def.h>

#include "report_service.h"

#include "INIReader.h"

#ifdef SL_SIMULATOR
#include <random>
#endif

#pragma pack(1)
/*dm1*/
typedef struct {
    SMLK_UINT8  src;
    SMLK_UINT8  fmi;
    SMLK_UINT32 spn;
} FaultCodeT;

typedef struct {
    SMLK_UINT8  count;
    FaultCodeT  codes[0];
} FaultCodesT;

typedef struct {
    SMLK_UINT32 mask:8;
    SMLK_UINT32 flag:8;
    SMLK_UINT32 :8;
    SMLK_UINT32 :8;
} ExtraFault;

struct RawDTC {
    SMLK_UINT8  Byte0;
    SMLK_UINT8  Byte1;
    SMLK_UINT8  Byte2;
    SMLK_UINT8  Byte3;
};

struct RawDM1 {
    SMLK_UINT8  can_channel;
    SMLK_UINT8  can_src;
    SMLK_UINT8  lamp_status_protect:2;
    SMLK_UINT8  lamp_status_amber:2;
    SMLK_UINT8  lamp_status_red_stop:2;
    SMLK_UINT8  lamp_status_malfunction_indicator:2;
    SMLK_UINT8  lamp_status_reserved;
    RawDTC      dtcs[0];
};

struct EvData {
    SMLK_UINT32 index;
    SMLK_BOOL   valid;
    SMLK_UINT8  data[0];
};

#pragma pack()

using namespace smartlink;

namespace Config {
    static const SMLK_UINT8     SL_CONFIG_BATTERY_ELECTRIC  = 0x00;
    static const SMLK_UINT8     SL_CONFIG_HYBRID_ELECTRIC   = 0x01;
    static const SMLK_UINT8     SL_CONFIG_FOSSIL_FUEL       = 0x02;
    static const SMLK_UINT8     SL_CONFIG_EXCEPTION         = 0xFE;
    static const SMLK_UINT8     SL_CONFIG_INVALID           = 0xFF;
};

namespace Location {
    static const SMLK_UINT32    SL_SERVICE_READY            = 0x00000001;
    static const SMLK_UINT32    SL_SERVICE_EXIT             = 0x00000002;
};

namespace MCU {
    static const SMLK_UINT32    SL_SERVICE_READY            = 0x000000001;
    static const SMLK_UINT32    SL_IGN_NOTIFY               = 0x000000002;
};

namespace IGState {
    static const SMLK_UINT32    SL_IGN_OFF                  = 0x000000000;
    static const SMLK_UINT32    SL_IGN_ON                   = 0x000000001;
};
namespace {
    static const int            SL_QUEUE_GET_TIMEOUT_MS     = 200;              // timeout millisecond to get item from queue

    static const SMLK_UINT32    SL_EVENT_LOCAT_NOTIFY       = 0xFF000001;       // event LOCAT notify
    static const SMLK_UINT32    SL_EVENT_MCU_NOTIFY         = 0xFF000002;       // event MCU notify
    static const SMLK_UINT32    SL_EVENT_SHARED_TIMER       = 0xFF000003;       // event shared timer notify
    static const SMLK_UINT32    SL_EVENT_UDS_RESPONSE       = 0xFF000100;       // event UDS response
    static const SMLK_UINT32    SL_EVENT_DM1_NOTIFY         = 0xFF00000A;       // event DM1 value update
    static const SMLK_UINT32    SL_EVENT_MUTIDATA_NOTIFY    = 0xFF00000B;       // event mutidata update

    static const SMLK_UINT32    SL_FLAG_MASK_IGNON      = 0x00000001;           // ING ON/OFF mask
    static const SMLK_UINT32    SL_FLAG_MASK_POWERON    = 0x00000002;           // POWER ON/OFF mask
};

ReportService::ReportService()
    : m_flags(IService::Uninitialized)
    , m_internal_flags(0)
    , m_vehicle_type(0x0A)
    , is_main_power_flag(0)
    , dm1info{}
{}

ReportService::~ReportService()
{}

/******************************************************************************
 * NAME: Init
 *
 * DESCRIPTION: Init function for the report service, you may finish all
 *              the initialization work here
 *
 * PARAMETERS:
 *
 * RETURN:
 *      SL_SUCCESS on success, and for now no error will return
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 ReportService::Init()
{
    SMLK_LOGD("enter");

    std::lock_guard <std::mutex> lk(m_mutex);
    if (m_flags & IService::Initialized) {
        SMLK_LOGD("already initialized, do nothing");
        return SL_SUCCESS;
    }

    if ( !tools::dir_exists(M_GB_32960_CONFIG_DIR) ) {
        if ( 0 != tools::mkdir_p(M_GB_32960_CONFIG_DIR) ) {
            SMLK_LOGE("fail to create config dir");
            return SL_EFAILED;
        }
    }
    if ( !tools::dir_exists(M_GB_32960_DATABASE_DIR) ) {
        if ( 0 != tools::mkdir_p(M_GB_32960_DATABASE_DIR) ) {
            SMLK_LOGE("fail to create database dir");
            return SL_EFAILED;
        }
    }

    if (!smartlink_sdk::SysProperty::GetInstance()->Init(ModuleID::E_Module_ev_service)) {
        SMLK_LOGE("fail to init system property module");
        return SL_EFAILED;
    }
   
    INIReader reader(M_GB_32960_CONFIG_FILE_DIR);

    SMLK_UINT32     m_env       = 0;    // 生产 : 0; 调试 : 1
    SMLK_UINT32     m_todisk    = 0;    // 不保存 : 0; 保存 : 1
    std::string     m_host      = "";
    SMLK_UINT16     m_port      = 0;

    if (reader.ParseError() < 0) {
        SMLK_LOGD("can't load Configure file: %s ", M_GB_32960_CONFIG_FILE_DIR);
    } else {
        SMLK_LOGD("load Configure file: %s ", M_GB_32960_CONFIG_FILE_DIR);
        m_env       = reader.GetInteger("environment", "env", 0);
        m_todisk    = reader.GetInteger("rawdata", "todisk", 0);
        m_host      = reader.GetString("remote", "host", "");
        m_port      = reader.GetInteger("remote", "port", 0);
        SMLK_LOGE("read ini file, [%s] %s:%d, save %d", m_env ? "DEV" : "PRD", m_host.c_str(), m_port, m_todisk);
    }

    const std::vector<std::string> property_key = {
        std::string(SYS_PRO_NAME_VEHICLE_VIN),
        std::string(SYS_PRO_NAME_DID_VEHICLE_TYPE),
        std::string(SYS_PRO_NAME_DID_NEW_ENERGY_EMISSION_REMOTE_MONITOR_CONFIG),
        std::string(SYS_PRO_NAME_DID_NEW_ENERGY_VEHICLE_CONFIG),
    };

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->RegEventCB(
        property_key,
        [this](smartlink_sdk::SysProperty::PropertyEventId id, smartlink_sdk::PropertyInfo* info) {
            switch ( id ) {
                case smartlink_sdk::SysProperty::PropertyEventId::E_SYS_PROPERTY_EVENT_VALUE_CHANGED:
                {
                    if (info->name == std::string(SYS_PRO_NAME_VEHICLE_VIN)) {
                        if (!info->value.empty()) {
                            SMLK_LOGI("[property] %s, value is %s!!!", info->name.c_str(), info->value.c_str());
                            for (auto &pair : m_protocol_reporter) {
                                pair.second->SetVIN(info->value);
                            }
                            return;
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_DID_VEHICLE_TYPE)) {
                        if (!info->value.empty()) {
                            SMLK_LOGI("[property] %s, value is %s, 0x%02X!!!", info->name.c_str(), info->value.c_str(), std::stoi(info->value));
                            return;
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_DID_NEW_ENERGY_EMISSION_REMOTE_MONITOR_CONFIG)) {
                        if (!info->value.empty()) {
                            SMLK_LOGI("[property] %s, value is %s, %u!!!", info->name.c_str(), info->value.c_str(), std::stoi(info->value));
                            for (auto &pair : m_protocol_reporter) {
                                pair.second->SetCONFIG(std::stoi(info->value));
                            }
                            return;
                        }
                    } else if (info->name == std::string(SYS_PRO_NAME_DID_NEW_ENERGY_VEHICLE_CONFIG)) {
                        if (!info->value.empty()) {
                            SMLK_LOGI("[property] %s, value is %s, %u!!!", info->name.c_str(), info->value.c_str(), std::stoi(info->value));
                            for (auto &pair : m_protocol_reporter) {
                                pair.second->SetTORQUE(std::stoi(info->value));
                            }
                            return;
                        }
                    }
                }
            }
        }
    );

    std::string     property;

    /*
     * 00：长春
     * 01：青岛
     */
    SMLK_UINT8      m_product_loc = 0;

    std::string     productloc;
    property = SYS_PRO_NAME_PRODUCT_LOC;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, productloc);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        m_product_loc = std::stoi(productloc);
        // SMLK_LOGD("loc:        %s",m_product_loc == 0?"Changchun":"Qingdao");
    }
    SMLK_LOGD("loc:        %s",m_product_loc == 0?"Changchun":"Qingdao");

    /*
     * 00：J7 中低配（柴油机）
     * 01：J7 高配（柴油机）
     * 02：J6 低配（柴油机）
     * 03：J6 中配（柴油机）
     * 04：J6 高配（柴油机）
     * 05：J7 中低配（LNG天然气）
     * 06：J7 高配（天然气）
     * 07：J6 低配（天然气）
     * 08：J6 中配（天然气）
     * 09：J6 高配（天然气）
     * 0A：新能源纯电车型
     * 0B：新能源混动车型
     */
    std::string     vehicletype;
    property = SYS_PRO_NAME_DID_VEHICLE_TYPE;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, vehicletype);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    }
    else
    {
        m_vehicle_type = std::stoi(vehicletype);
        SMLK_LOGI("vehicletype:        %u", m_vehicle_type);
    }


    std::string     config;
    SMLK_UINT8      monitor_config = Config::SL_CONFIG_INVALID;
    property        = SYS_PRO_NAME_DID_NEW_ENERGY_EMISSION_REMOTE_MONITOR_CONFIG;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, config);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    }
    else
    {
        monitor_config = std::stoi(config);
        SMLK_LOGI("config:        %u", monitor_config);
    }

    std::string     torque;
    SMLK_UINT16     refer_torque = 0;
    property        = SYS_PRO_NAME_DID_NEW_ENERGY_VEHICLE_CONFIG;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, torque);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        refer_torque = std::stoi(torque);
        SMLK_LOGI("torque:        %u", refer_torque);
    }

    //  std::string     vin = "SMLKA1GB6TEST0001";
    //  property = SYS_PRO_NAME_VEHICLE_VIN;

    //  SMLK_LOGD("get vin is [%s]", vin.c_str());

    std::string     vin;
    property = SYS_PRO_NAME_VEHICLE_VIN;  
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, vin);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        SMLK_LOGD("vin:        %s", vin.c_str());
    }
    
    std::string host;
    std::string ev_tcp_ip;
    //property = m_product_loc ? SYS_PRO_NAME_32960_IP_QD : SYS_PRO_NAME_32960_IP_CC;
    property = SYS_PRO_NAME_32960_IP;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, ev_tcp_ip);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
        SMLK_LOGE("fail to get ev ip \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        host = ev_tcp_ip;
        SMLK_LOGI("host:        %s", host.c_str());
    }

    SMLK_UINT16 port;
    std::string ev_tcp_port;
    //property = m_product_loc ? SYS_PRO_NAME_32960_PORT_QD : SYS_PRO_NAME_32960_PORT_CC;
    property = SYS_PRO_NAME_32960_PORT;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, ev_tcp_port);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code)
    {
        SMLK_LOGE("fail to get ev port \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        port = std::stoi(ev_tcp_port);
        SMLK_LOGI("port:        %u", port);
    }

    SMLK_UINT8      rev  = 1;
    std::set<SMLK_UINT8>   exists;
    if ( exists.cend() != exists.find(rev) ) {
        SMLK_LOGE("duplicated Rev: %u configured, skip this platform!!!", rev);
    }

    exists.insert(rev);
        
    std::vector<std::tuple<std::string, SMLK_UINT16, SMLK_UINT8>>  servers;  

    if (m_env) {
        SMLK_LOGE("configure dev env");
        servers.emplace_back(m_host, m_port, rev);
    } else {
        SMLK_LOGE("configure prod env");
        servers.emplace_back(host, port, rev);
    }

    if ( 0 == servers.size() ) {
        SMLK_LOGW("no valid platform specified, we will do nothing!!!");
    }


    SMLK_LOGD("register vehilce data callback");
    auto r = VehicleDataApi::GetInstance()->Init(ModuleID::E_Module_ev_service);
    if ( SMLK_RC::RC_OK != r ) {
        SMLK_LOGE("fail to init vehicle data service API interface!!!");
        return SL_EFAILED;
    }

    std::vector<SMLK_UINT32>    indexs = {
            //
            VEHICLE_DATA_EV_SERVICE_BRAKE_CIRCUIT_1_AIR_PRESSURE,
            VEHICLE_DATA_EV_SERVICE_BRAKE_CIRCUIT_2_AIR_PRESSURE,
            //
            VEHICLE_DATA_EV_CODE_LENGTH_RECHARGEABLE_ENERGY_STORAGE_SYSTEM,			//	301 可充电储能系统编码长度
            VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_SUBSYSTEMS_QUANTITY,        //	410 可充电储能子系统个数
            // 整车数据
            VEHICLE_DATA_EV_VEHICLE_STATUS,											//	310 车辆状态
            VEHICLE_DATA_EV_POWER_BATTERY_CHARGING_STATUS,							//	311 充电状态
            VEHICLE_DATA_EV_VEHICLE_SPEED,                                          //  317 车速(新能源)
            VEHICLE_DATA_TOTAL_VEHICLE_DISTANCE,
            VEHICLE_DATA_EV_POWER_BATTERY_VOLTAGE,                                  //  312 总电压/可充电储能装置电压
            VEHICLE_DATA_EV_POWER_BATTERY_CURRENT,                                  //	313 总电流/可充电储能装置电流
            VEHICLE_DATA_EV_POWER_BATTERY_SOC,                                      //	314 SOC
            VEHICLE_DATA_EV_DCDC_WORKING_STATUS,                                    //	315 DC-DC状态
            VEHICLE_DATA_EV_INSULATION_RESISTANCE,                                  //	316 绝缘电阻
            VEHICLE_DATA_CURRENT_GEAR,
            VEHICLE_DATA_ACCELERATOR_PEDAL_POSITION,
            VEHICLE_DATA_BRAKE_SWITCH,            
            // 驱动电机数据
            VEHICLE_DATA_EV_ELEC_ENGINE_SYSTEM_STATUS,
            VEHICLE_DATA_EV_ELEC_MACHINE_CTRL_TEMP,                                 //	330 驱动电机控制器温度
            VEHICLE_DATA_EV_ELEC_ENGINE_SPEED,                                      //	331 驱动电机转速
            VEHICLE_DATA_EV_ELEC_ENGINE_HIGH_ACCURACY_TORQUE,       
            VEHICLE_DATA_EV_ELEC_ENGINE_TORQUE,                                     //	332 驱动电机转矩
            VEHICLE_DATA_EV_ELEC_MACHINE_TEMP,                                      //	333 驱动电机温度
            VEHICLE_DATA_EV_ELEC_MACHINE_INPUT_VOL,                                 //	334 电机控制器输入电压
            VEHICLE_DATA_EV_ELEC_MACHINE_CTRL_CURRENT,                              //	335 电机控制器直流母线电流
            VEHICLE_DATA_EV_ELEC_ENGINE_ROTATION_DIRECTION,
            // 燃料电池数据
            VEHICLE_DATA_EV_FUEL_BATTERY_VOL,                                       //	350 燃料电池电压
            VEHICLE_DATA_EV_FUEL_BATTERY_CURRENT,                                   //	351 燃料电池电流
            VEHICLE_DATA_EV_FUEL_RATE,                                              //	352 燃料消耗率
            VEHICLE_DATA_EV_FUEL_BATTERY_TEMP_NEEDLE_NUM,                           //	353 燃料电池温度探针总数
            VEHICLE_DATA_EV_H_HIGHEST_TEMP,                                         //	355 氢系统最高温度
            VEHICLE_DATA_EV_H_HIGHEST_TEMP_NEEDLE_CODE,                             //	356 氢系统最高温度探针代号
            VEHICLE_DATA_EV_H_HIGHEST_CONCENTRATION,                                //	357 氢气最高浓度
            VEHICLE_DATA_EV_H_HIGHEST_CONCENTRATION_CODE,                           //	358 氢气最高浓度传感器代号
            VEHICLE_DATA_EV_H_HIGHEST_PRESSURE,                                     //	359 氢气最高压力
            VEHICLE_DATA_EV_H_HIGHEST_PRESSURE_SENSOR_CODE,                         //	360 氢气最高压力传感器代号
            VEHICLE_DATA_EV_HIGH_PRESSURE_DC_STATUS,                                       //	361 高压DC/DC状态
            VEHICLE_DATA_EV_WATER_IN_TEMP,                                          //  354 水入温度
            VEHICLE_DATA_EV_WATER_OUT_TEMP,                                         //  362 水出温度
            // 发动机数据
            VEHICLE_DATA_ENGINE_SPEED,
            VEHICLE_DATA_ENGINE_INSTANTANEOUS_FUEL_ECONOMY,
            // 极值数据
            VEHICLE_DATA_EV_HIGHEST_VOLTAGE_BATTERY_SUBSYSTEM_NUMBER,               //	380 最高电压底池子系统
            VEHICLE_DATA_EV_NUMBER_OF_SINGLE_BATTERY_HIGHEST_VOLTAGE,               //	381 最高电压电池单体代号
            VEHICLE_DATA_EV_SINGLE_BATTERY_HIGHEST_VOLTAGE,                         //	382 电池单体电压最高值
            VEHICLE_DATA_EV_LOWEST_VOLTAGE_BATTERY_SUBSYSTEM_NUMBER,                //	383 最低电压电池子系统号
            VEHICLE_DATA_EV_NUMBER_OF_SINGLE_BATTERY_MINIMUM_VOLTAGE,               //	384 最低电压电池单体代号
            VEHICLE_DATA_EV_SINGLE_BATTERY_MINIMUM_VOLTAGE,                         //	385 电池单体电压最低值
            VEHICLE_DATA_EV_MAXIMUM_TEMPERATURE_SUBSYSTEM_NUMBER,                   //	386 最高温度子系统号
            VEHICLE_DATA_EV_NUMBER_OF_MAXIMUM_TEMPERATURE_PROBE_MONOMER,            //	387 最高温度探针序号
            VEHICLE_DATA_EV_MAXIMUM_TEMPERATURE_OF_POWER_BATTERY,                   //	388 最高温度值
            VEHICLE_DATA_EV_MINIMUM_TEMPERATURE_SUBSYSTEM_NUMBER,                   //	389 最低温度子系统号
            VEHICLE_DATA_EV_NUMBER_OF_MINIMUM_TEMPERATURE_PROBE_MONOMER,            //	390 最低温度探针序号
            VEHICLE_DATA_EV_MINIMUM_TEMPERATURE_OF_POWER_BATTERY,                   //	391 最低温度值
            
            VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_SUBSYSTEMS_NUMBER,          //	411 可充电储能子系统号
            // 可充电储能装置电压数据
            // VEHICLE_DATA_EV_POWER_BATTERY_VOLTAGE,
            // VEHICLE_DATA_EV_POWER_BATTERY_CURRENT,
            VEHICLE_DATA_EV_TOTAL_NUMBER_SINGLE_CELLS,                              //	414 单体电池总数
            // VEHICLE_DATA_EV_SINGLE_BATTERY_VOL,                                     //	415 单体电池电压
            // 可充电储能装置温度数据
            VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_TEMPERATURE_PROBES_QUANTITY,//	416 可充电储能温度探针个数
            // VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_TEMPERATURE,                //	416 可充电储能子系统各温度探针检测到的温度值
    };

    r = VehicleDataApi::GetInstance()->RegistVehicleDataChangedCB(
        indexs,
        [this](VehicleData data){
            for ( auto &pair : m_protocol_reporter ) {
                if ( pair.second->IsRunning() ) {
                    // SMLK_LOGD ("========== data.index = %ld, data.value = %lf, data.valid = %d \n", data.index, data.value, data.valid);
                    pair.second->UpdateSignalVal(data.index, data.value, data.valid);
                }
            }
        }
    );
    if ( SMLK_RC::RC_OK != r ) {
        SMLK_LOGE("fail to register vehicle data changed callback");
        return SL_EFAILED;
    }

    SMLK_LOGD("register vehilce data EV Muti CAN data callback");
    std::vector<SMLK_UINT32>    muti_indexs = {
        VEHICLE_DATA_EV_SINGLE_BATTERY_VOL,
        VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_TEMPERATURE,
    };

    r = VehicleDataApi::GetInstance()->RegistEvMutiData(
            muti_indexs,
            [this](Index index, bool valid, std::vector<SMLK_UINT8> &data) {
                SMLK_LOGD("receive vehicle Index  %d Valid %d ", index, valid);
                Post(MsgHead(SL_EVENT_MUTIDATA_NOTIFY, index, 0, 0, 0, valid), data);
            }
            );
    if ( SMLK_RC::RC_OK != r ) {
        SMLK_LOGE("fail to register vehilce data EV Muti CAN data callback");
        return SL_EFAILED;
    }

    SMLK_LOGD("register vehilce data DM1 raw CAN data callback");
    r = VehicleDataApi::GetInstance()->RegistDM1RawData(
            [this](CAN_ID id, std::vector<SMLK_UINT8> &data) {
                // SMLK_LOGD("receive vehicle CAN_ID id  %d ", id);
                Post(MsgHead(SL_EVENT_DM1_NOTIFY, id, 0, 0, 0, 0), data);
            }
            );
    if ( SMLK_RC::RC_OK != r ) {
        SMLK_LOGE("fail to register vehilce data DM1 raw CAN data callback");
        return SL_EFAILED;
    }

    SMLK_LOGD("register location service related data callback");
    if ( !smartlink_sdk::Location::GetInstance()->Init(ModuleID::E_Module_ev_service) ) {
        SMLK_LOGE("fail to init location service API interface!!!");
        return SL_EFAILED;
    }

    std::vector<smartlink_sdk::LocationEventId> loc_events = {
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_ISREADY,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_EXIT,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED,
    };

    return_code = smartlink_sdk::Location::GetInstance()->RegEventCallback(
        loc_events,
        [this](smartlink_sdk::LocationEventId id, void *data, int len) {
            switch ( id ) {
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_ISREADY:
                    Post(MsgHead (SL_EVENT_LOCAT_NOTIFY, Location::SL_SERVICE_READY, 0, 0, 0, 0), nullptr, 0);
                    break;
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_EXIT:
                    Post(MsgHead (SL_EVENT_LOCAT_NOTIFY, Location::SL_SERVICE_EXIT, 0, 0, 0, 0), nullptr, 0);
                    break;
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED:
                    {
                        if ( sizeof(smartlink_sdk::LocationInfo) != (std::size_t)len ) {
                            SMLK_LOGE("[GNSS] unexpected body length for FIX INFO CHNAGED notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::LocationInfo));
                            return;
                        }

                        auto    p_inf = reinterpret_cast<smartlink_sdk::LocationInfo *>(data);

                        ProtocolReporter::GNSS::Info    inf;

                        inf.flags       = p_inf->fix_valid ? ProtocolReporter::GNSS::Flags::SL_VALID : ProtocolReporter::GNSS::Flags::SL_INVALID;
                        inf.latitude    = p_inf->latitude;
                        inf.longitude   = p_inf->longitude;
                        inf.altitude    = p_inf->altitude;
                        inf.speed       = p_inf->speed;
                        inf.heading     = p_inf->heading;
                        inf.timestamp   = p_inf->utc;

                        for ( auto &pair : m_protocol_reporter ) {
                            if ( pair.second->IsRunning() ) {
                                pair.second->UpdateGNSS(inf);
                            }
                        }
                    }
                    break;
                default:
                    SMLK_LOGE("unsupported event id: %u", static_cast<SMLK_UINT32>(id));
                    return;
            }
        }
    );
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to register event callback to location service, return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }


    SMLK_LOGD("init sys-power service");
    if (!smartlink_sdk::SysPower::GetInstance()->Init(ModuleID::E_Module_ev_service)) {
        SMLK_LOGE("fail to init power service API interface!!!");
        return SL_EFAILED;
    }
    std::vector<smartlink_sdk::SysPowerEventID> power_events = {
        smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_SOURCE_CHANGED,
        smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP,
        smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP,
    };
    SMLK_LOGD("register sys-power event callback");
    auto pow_ret = smartlink_sdk::SysPower::GetInstance()->RegEventCB(
        power_events,
        [this](smartlink_sdk::SysPowerEventID id, void *data, int len) {
            switch (id) {
                case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_SOURCE_CHANGED: {
                    if (sizeof(smartlink_sdk::SysPowerSource) != (std::size_t)len) {
                        SMLK_LOGE("[POWER] unexpected body length for SYS POWER source CHANGED: %d, expected: %d", len, (SMLK_UINT8)sizeof(smartlink_sdk::SysPowerSource));
                        return;
                    }
                    auto inf = reinterpret_cast<const smartlink_sdk::SysPowerSource *>(data);

                    if ((SMLK_UINT8)smartlink_sdk::SysPowerSource::E_SYS_POWER_SOURCE_MAIN == (SMLK_UINT8)*inf) {
                        if (0 == is_main_power_flag) {
                            // SMLK_LOGD("[POWER] alerady change to main power !!!");
                            return;
                        } else {
                            is_main_power_flag = 0;
                            SMLK_LOGD("[POWER] system power source change to main power !!!");
                        }
                    } else if ((SMLK_UINT8)smartlink_sdk::SysPowerSource::E_SYS_POWER_SOURCE_STANDBY == (SMLK_UINT8)*inf) {
                        if (1 == is_main_power_flag) {
                            // SMLK_LOGD("[POWER] alerady change to standby power !!!");
                            return;
                        } else {
                            is_main_power_flag = 1;
                            for ( auto &pair : m_protocol_reporter ) {
                                if ( pair.second->IsRunning() ) {
                                    pair.second->SetStandbyState();
                                }
                            }
                            SMLK_LOGD("[POWER] system power source change to standby power !!!");
                        }
                    } else {
                        SMLK_LOGD("[POWER] system power source change to invalid data%d !!!", (SMLK_UINT8)*inf);
                    }
                }
                break;
                case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP: {
                    SMLK_LOGD("[POWER] system power event wakeup received!!!");

                    // SMLK_LOGD("[timesyncDebug] m_timeSyncGap_ms is %lld", m_timeSyncGap_ms);
                    // SMLK_LOGD("[timesyncDebug] RTC wake up reset m_timeSyncGap_ms to 0");
                    // m_timeSyncGap_ms = 0;
                    // m_systime_flags = SysTime::SL_SYSTIME_SYN_DEF;
                }
                break;
                case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP: {
                    SMLK_LOGD("[POWER] system power event to sleep received!!!");
                }
                break;
                default:
                    SMLK_LOGE("[POWER] unsupported event id: %u", static_cast<SMLK_UINT8>(id));
                    return;
                }
            }
        );
    if (smartlink_sdk::RtnCode::E_SUCCESS != pow_ret)
    {
        SMLK_LOGE("fail to init register event callback to power service!!! return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    SMLK_LOGD("register MCU service related data callback");
    return_code = smartlink_sdk::MCU::GetInstance()->Init(ModuleID::E_Module_ev_service);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to init MCU service, return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    std::vector<smartlink_sdk::McuEventId> mcu_events = {
        smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE,
    };
    return_code = smartlink_sdk::MCU::GetInstance()->RegEventCB(
        mcu_events,
        [this](smartlink_sdk::McuEventId id, void *data, int len) {
            // SMLK_LOGD("receive McuEventId  id %d",id);
            switch ( id ) {
                case smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE:
                    {
                        if ( sizeof(smartlink_sdk::IGNState) != (std::size_t)len ) {
                            SMLK_LOGE("[MCU] unexpected body length for MCU IGN state changed notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::IGNState));
                            return;
                        }
        
                        smartlink_sdk::IGNState *ign = reinterpret_cast<smartlink_sdk::IGNState *>(data);

                        Post(MsgHead(SL_EVENT_MCU_NOTIFY, MCU::SL_IGN_NOTIFY, 0, 0, 0, ign->on ? IGState::SL_IGN_ON : IGState::SL_IGN_OFF), nullptr, 0);
                    }
                    break;
                default:
                    SMLK_LOGE("[MCU] unsupported event id: %u", static_cast<SMLK_UINT32>(id));
                    return;
            }
        }
    );
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to register event callback to MCU service, return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    SMLK_LOGD("register Telephony service related data callback");
    if ( !smartlink_sdk::Telephony::GetInstance()->Init(ModuleID::E_Module_ev_service) ) {
        SMLK_LOGE("fail to init Telephony service");
        return SL_EFAILED;
    }

    std::vector<smartlink_sdk::TelEventId> tel_events = {
        smartlink_sdk::TelEventId::E_TEL_EVENT_DATA_CHANNEL_CHANGED,
    };
    return_code = smartlink_sdk::Telephony::GetInstance()->RegEventCB(
        tel_events,
        [this](smartlink_sdk::TelEventId id, void *data, int len) {
            switch ( id ) {
                case smartlink_sdk::TelEventId::E_TEL_EVENT_DATA_CHANNEL_CHANGED:
                    {
                        if ( sizeof(smartlink_sdk::TelDataChnlInfo) != std::size_t(len) ) {
                            SMLK_LOGE("[Telephony] unexpected body lenght for telephony data channel changed notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::TelDataChnlInfo));
                            return;
                        }

                        auto p_inf = reinterpret_cast<smartlink_sdk::TelDataChnlInfo *>(data);

                        ProtocolReporter::telephony::Info   inf;

                        inf.channel     = static_cast<SMLK_UINT8>(p_inf->channel);
                        inf.status      = static_cast<SMLK_UINT8>(p_inf->state);
                        inf.type        = static_cast<SMLK_UINT8>(p_inf->type);
                        inf.interface   = p_inf->if_name;

                        for ( auto &pair : m_protocol_reporter ) {
                            if ( pair.second->IsRunning() ) {
                                pair.second->SetTelephonyState(inf);
                            }
                        }
                    }
                    break;
                default:
                    SMLK_LOGE("unregistered event id:   0x%08X", static_cast<SMLK_UINT32>(id));
                    break;
            }
        }
    );

    // get configuration
    std::string     iccid;
    return_code = smartlink_sdk::Telephony::GetInstance()->GetICCID(iccid);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get ICCID from telephony module! return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    SMLK_LOGD("ICCID:   %s", iccid.c_str());

    for ( auto const &server : servers ) {
        m_protocol_reporter[std::get<2>(server)]    = std::make_shared<Reporter>(std::get<0>(server), std::get<1>(server));
    }

    for ( auto &pair : m_protocol_reporter ) {

        pair.second->SetICCID(iccid);
        pair.second->SetVIN(vin);
        pair.second->SetCONFIG(monitor_config);
        pair.second->SetTORQUE(refer_torque);
        pair.second->SetIsSave(m_todisk);

        auto rc = pair.second->Init();
        if ( SL_SUCCESS != rc ) {
            SMLK_LOGE("fail to init protocol reporter, protocol = 0x%02X", pair.first);
            return SL_EFAILED;
        }
    }

    InitTimerOutCheck();

    m_flags |= IService::Initialized;

    SMLK_LOGD("init obd-diag service");
    auto obd_ret = OBD::DidIpcApi::getInstance()->Init(ModuleID::E_Module_ev_service);
    if ( SMLK_RC::RC_OK != obd_ret ) {
        SMLK_LOGF("fail to init diag service API interface, err=%d.\n",obd_ret);
        return SL_EFAILED;
    }

#ifdef SL_DEBUG
    SMLK_LOGD("reportService finish ");
#endif
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Start
 *
 * DESCRIPTION: start the service
 *
 * PARAMETERS:
 *
 * RETURN:
 *      SL_SUCCESS on success
 *      SL_EFAILED on failed
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 ReportService::Start()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Running ) {
        SMLK_LOGI("service already started, do nothing");
        return SL_SUCCESS;
    }

    if ( !(m_flags & IService::Initialized) ) {
        SMLK_LOGE("service not initialized");
        return SL_EFAILED;
    }

    for ( auto &pair : m_protocol_reporter ) {
        auto rc = pair.second->Start();
        if ( SL_SUCCESS != rc ) { 
            SMLK_LOGE("fail to start protocol repoter, pair.first = 0x%02X, rc = 0x%08X", pair.first, rc);
            return SL_EFAILED;
        }
    }
    
    std::vector<SMLK_UINT32>    indexs = {
            //
            VEHICLE_DATA_EV_SERVICE_BRAKE_CIRCUIT_1_AIR_PRESSURE,
            VEHICLE_DATA_EV_SERVICE_BRAKE_CIRCUIT_2_AIR_PRESSURE,
            //
            VEHICLE_DATA_EV_CODE_LENGTH_RECHARGEABLE_ENERGY_STORAGE_SYSTEM,			//	301 可充电储能系统编码长度
            VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_SUBSYSTEMS_QUANTITY,        //	410 可充电储能子系统个数
            // 整车数据
            VEHICLE_DATA_EV_VEHICLE_STATUS,											//	310 车辆状态
            VEHICLE_DATA_EV_POWER_BATTERY_CHARGING_STATUS,							//	311 充电状态
            VEHICLE_DATA_EV_VEHICLE_SPEED,                                          //  317 车速(新能源)
            VEHICLE_DATA_TOTAL_VEHICLE_DISTANCE,
            VEHICLE_DATA_EV_POWER_BATTERY_VOLTAGE,                                  //  312 总电压/可充电储能装置电压
            VEHICLE_DATA_EV_POWER_BATTERY_CURRENT,                                  //	313 总电流/可充电储能装置电流
            VEHICLE_DATA_EV_POWER_BATTERY_SOC,                                      //	314 SOC
            VEHICLE_DATA_EV_DCDC_WORKING_STATUS,                                    //	315 DC-DC状态
            VEHICLE_DATA_EV_INSULATION_RESISTANCE,                                  //	316 绝缘电阻
            VEHICLE_DATA_CURRENT_GEAR,
            VEHICLE_DATA_ACCELERATOR_PEDAL_POSITION,
            VEHICLE_DATA_BRAKE_SWITCH,     
            // 驱动电机数据
            VEHICLE_DATA_EV_ELEC_ENGINE_SYSTEM_STATUS,
            VEHICLE_DATA_EV_ELEC_MACHINE_CTRL_TEMP,                                 //	330 驱动电机控制器温度
            VEHICLE_DATA_EV_ELEC_ENGINE_SPEED,                                      //	331 驱动电机转速
            VEHICLE_DATA_EV_ELEC_ENGINE_HIGH_ACCURACY_TORQUE,            
            VEHICLE_DATA_EV_ELEC_ENGINE_TORQUE,                                     //	332 驱动电机转矩
            VEHICLE_DATA_EV_ELEC_MACHINE_TEMP,                                      //	333 驱动电机温度
            VEHICLE_DATA_EV_ELEC_MACHINE_INPUT_VOL,                                 //	334 电机控制器输入电压
            VEHICLE_DATA_EV_ELEC_MACHINE_CTRL_CURRENT,                              //	335 电机控制器直流母线电流
            VEHICLE_DATA_EV_ELEC_ENGINE_ROTATION_DIRECTION,
            // 燃料电池数据
            VEHICLE_DATA_EV_FUEL_BATTERY_VOL,                                       //	350 燃料电池电压
            VEHICLE_DATA_EV_FUEL_BATTERY_CURRENT,                                   //	351 燃料电池电流
            VEHICLE_DATA_EV_FUEL_RATE,                                              //	352 燃料消耗率
            VEHICLE_DATA_EV_FUEL_BATTERY_TEMP_NEEDLE_NUM,                           //	353 燃料电池温度探针总数
            VEHICLE_DATA_EV_H_HIGHEST_TEMP,                                         //	355 氢系统最高温度
            VEHICLE_DATA_EV_H_HIGHEST_TEMP_NEEDLE_CODE,                             //	356 氢系统最高温度探针代号
            VEHICLE_DATA_EV_H_HIGHEST_CONCENTRATION,                                //	357 氢气最高浓度
            VEHICLE_DATA_EV_H_HIGHEST_CONCENTRATION_CODE,                           //	358 氢气最高浓度传感器代号
            VEHICLE_DATA_EV_H_HIGHEST_PRESSURE,                                     //	359 氢气最高压力
            VEHICLE_DATA_EV_H_HIGHEST_PRESSURE_SENSOR_CODE,                         //	360 氢气最高压力传感器代号
            VEHICLE_DATA_EV_HIGH_PRESSURE_DC_STATUS,                                       //	361 高压DC/DC状态
            VEHICLE_DATA_EV_WATER_IN_TEMP,                                          //  354 水入温度
            VEHICLE_DATA_EV_WATER_OUT_TEMP,                                         //  362 水出温度
            // 发动机数据
            VEHICLE_DATA_ENGINE_SPEED,
            VEHICLE_DATA_ENGINE_INSTANTANEOUS_FUEL_ECONOMY,
            //极值数据
            VEHICLE_DATA_EV_HIGHEST_VOLTAGE_BATTERY_SUBSYSTEM_NUMBER,               //	380 最高电压底池子系统
            VEHICLE_DATA_EV_NUMBER_OF_SINGLE_BATTERY_HIGHEST_VOLTAGE,               //	381 最高电压电池单体代号
            VEHICLE_DATA_EV_SINGLE_BATTERY_HIGHEST_VOLTAGE,                         //	382 电池单体电压最高值
            VEHICLE_DATA_EV_LOWEST_VOLTAGE_BATTERY_SUBSYSTEM_NUMBER,                //	383 最低电压电池子系统号
            VEHICLE_DATA_EV_NUMBER_OF_SINGLE_BATTERY_MINIMUM_VOLTAGE,               //	384 最低电压电池单体代号
            VEHICLE_DATA_EV_SINGLE_BATTERY_MINIMUM_VOLTAGE,                         //	385 电池单体电压最低值
            VEHICLE_DATA_EV_MAXIMUM_TEMPERATURE_SUBSYSTEM_NUMBER,                   //	386 最高温度子系统号
            VEHICLE_DATA_EV_NUMBER_OF_MAXIMUM_TEMPERATURE_PROBE_MONOMER,            //	387 最高温度探针序号
            VEHICLE_DATA_EV_MAXIMUM_TEMPERATURE_OF_POWER_BATTERY,                   //	388 最高温度值
            VEHICLE_DATA_EV_MINIMUM_TEMPERATURE_SUBSYSTEM_NUMBER,                   //	389 最低温度子系统号
            VEHICLE_DATA_EV_NUMBER_OF_MINIMUM_TEMPERATURE_PROBE_MONOMER,            //	390 最低温度探针序号
            VEHICLE_DATA_EV_MINIMUM_TEMPERATURE_OF_POWER_BATTERY,                   //	391 最低温度值

            VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_SUBSYSTEMS_NUMBER,          //	411 可充电储能子系统号
            // 可充电储能装置电压数据
            // VEHICLE_DATA_EV_POWER_BATTERY_VOLTAGE,
            // VEHICLE_DATA_EV_POWER_BATTERY_CURRENT,
            VEHICLE_DATA_EV_TOTAL_NUMBER_SINGLE_CELLS,                              //	414 单体电池总数
            // VEHICLE_DATA_EV_SINGLE_BATTERY_VOL,                                     //	415 单体电池电压
            // 可充电储能装置温度数据
            VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_TEMPERATURE_PROBES_QUANTITY,//	416 可充电储能温度探针个数
            // VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_TEMPERATURE,                //	416 可充电储能子系统各温度探针检测到的温度值
    };

    for ( auto index : indexs ) {
        VehicleData data(index, 0.0);
        auto rc = VehicleDataApi::GetInstance()->GetVehicleData(data);
        if ( SMLK_RC::RC_OK != rc ) {
            SMLK_LOGD("fail to get vehicle data for index:  %u", index);
            continue;
        }
        
        // SMLK_LOGD ("data.index = %ld, data.value = %lf, data.valid = %d \n", data.index, data.value, data.valid);
        for ( auto &pair : m_protocol_reporter ) {
            pair.second->UpdateSignalVal(data.index, data.value, data.valid);
        }
    }

    if ( smartlink_sdk::Location::GetInstance()->IsReady() ) {
        GetGNSS();
    }

    if ( 0 != (m_internal_flags & SL_FLAG_MASK_IGNON) ) {
        for ( auto &pair : m_protocol_reporter ) {
            pair.second->IgnON();
        }
    } else {
        for ( auto &pair : m_protocol_reporter ) {
            pair.second->IgnOFF();
        }
    }

    m_flags |= IService::Running;

    m_thread = std::thread(
        [this]() {
            SMLK_LOGI("main work thread started");
            while ( !(m_flags & IService::Stopping) ) {
                m_events.get(SL_QUEUE_GET_TIMEOUT_MS)();
            }
            SMLK_LOGI("main work thread stoppped");
        }
    );

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Stop
 *
 * DESCRIPTION: stop the service
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void ReportService::Stop()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Running ) {
        SMLK_LOGD("told to stop, mark as stopping");
        m_flags |= IService::Stopping;

        if ( m_thread.joinable() ) {
            m_thread.join();
        }

        SMLK_LOGD("stop reporter one by one");
        for ( auto &pair : m_protocol_reporter ) {
            pair.second->Stop();
            SMLK_LOGD("one reporter stopped!");
        }
        SMLK_LOGD("all reporter stopped");

        m_flags |= ~(IService::Running | IService::Stopping);
        SMLK_LOGD("stopped");
    }
}

/******************************************************************************
 * NAME: IsRunning
 *
 * DESCRIPTION: check the service running status
 *
 * PARAMETERS:
 *
 * RETURN:      true if the service already running, otherwise false
 *
 * NOTE:
 *****************************************************************************/
bool ReportService::IsRunning() const
{
    return  m_flags & IService::Running;
}

/******************************************************************************
 * NAME: Post
 *
 * DESCRIPTION: post message the service main queue, the function can be called 
 *              by the service its self or other module
 *
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *        sz:   data length
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void ReportService::Post(IN MsgHead &head, IN SMLK_UINT8 *ptr, IN std::size_t sz)
{
    m_events.emplace(
        head,
        ptr,
        sz,
        std::bind(&ReportService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: Post
 *
 * DESCRIPTION: post message the service main queue, the function can be called 
 *              by the service its self or other module
 *
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void ReportService::Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_events.emplace(
        head,
        data,
        std::bind(&ReportService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: Post
 *
 * DESCRIPTION: post message the service main queue, the function can be called 
 *              by the service its self or other module
 *
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *
 * RETURN:
 *
 * NOTE:        this is a move operation, so after this call, the function 
 *              caller should never use the data again
 *****************************************************************************/
void ReportService::Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data)
{
    m_events.emplace(
        head,
        std::move(data),
        std::bind(&ReportService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: HandleEvent
 *
 * DESCRIPTION: the main entry for all the events includeing the IPC message, 
 *              timer notify message, notify message between threads etc.
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:        this is the main event entry for the main thread, any cost time
 *              operation may block other event, so do not do any const time
 *              operation in the main thread
 *****************************************************************************/
void ReportService::HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_id ) {
        case SL_EVENT_LOCAT_NOTIFY:
            OnLocationNotify(head, data);
            break;
        case SL_EVENT_MCU_NOTIFY:
            OnMcuNotify(head, data);
            break;
        case SL_EVENT_DM1_NOTIFY:
            OnDM1Notify(head, data);
            break;
        case SL_EVENT_MUTIDATA_NOTIFY:
            OnMutiDataNotify(head, data);
            break;
        default:
            SMLK_LOGE("unsupported event id:    0x%08X", head.m_id);
            break;
    }
}

void ReportService::OnMcuNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch (head.m_seq) {
        case MCU::SL_IGN_NOTIFY:
        {
            if (IGState::SL_IGN_ON == head.m_extra)
            {
                if (0 == (m_internal_flags & SL_FLAG_MASK_IGNON)) {
                    m_internal_flags |= SL_FLAG_MASK_IGNON;
                    SMLK_LOGD("[IGN] IGN state changed from OFF to ON");
                }
                for (auto &pair : m_protocol_reporter) {
                    if (pair.second->IsRunning()) {
                        pair.second->IgnON();
                    }
                }
            } else {
                if (0 != (m_internal_flags & SL_FLAG_MASK_IGNON)) {
                    m_internal_flags &= ~SL_FLAG_MASK_IGNON;
                    SMLK_LOGD("[IGN] IGN state changed from ON to OFF");

                    int is_logout = 1;
                    for (int i = 0; i < 4; i++) {
                        if (is_main_power_flag == 1) {
                            is_logout = 0;
                            break;
                        } 
                        usleep(500 * 1000);
                    }

                    if (is_logout) {
                        for (auto &pair : m_protocol_reporter) {
                            if (pair.second->IsRunning()) {
                                pair.second->IgnOFF();
                            }
                        }
                    }
                }
            }
        }
        break;
        default:
            SMLK_LOGE("[MCU] unsupported MCU event notify: 0x%08x", head.m_seq);
            break;
        }
}

void ReportService::InitTimerOutCheck()
{

    DM1DataInfo dm1_info;
    dm1_info.tp =  std::chrono::steady_clock::now();
    dm1_info.cycle = 1000;
    dm1_info.data_valid = true;

    g_dm1_data_pool.emplace(0xF3, dm1_info);
    g_dm1_data_pool.emplace(0x1A, dm1_info);
    g_dm1_data_pool.emplace(0xEF, dm1_info);

    //立刻执行
    m_check_timeout_timer.setStartInterval(0);
    //timeout周期为m_check_period * m_timeout_cycle_cnt(长断周期数)以此来降低负载
    m_check_timeout_timer.setPeriodicInterval(1000 * 10);// all dm1 1000ms cycle
    m_check_timeout_timer.start(Poco::TimerCallback<ReportService>(*this, &ReportService::OnCheckDataValidTimeout));
}


void ReportService::OnCheckDataValidTimeout(Poco::Timer& /*t*/)
{
    std::lock_guard<std::mutex> lk(g_dm1_data_pool_mutex);
    SMLK_LOGD("[dm1test] g_dm1_data_pool size is %d", g_dm1_data_pool.size());

    for (auto &pair : g_dm1_data_pool)
    {
        // SMLK_LOGE("[dm1debug] index[0x%02X] time diff is %ld ", pair.first, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - pair.second.tp).count());

        // since last revice time is longer than 3 period cyle, mark valid to false;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - pair.second.tp).count() >= 3 * pair.second.cycle)
        {
            // SMLK_LOGE("[dm1debug] index[0x%02X] to set pair.second.data_valid false", pair.first);
            if (true == pair.second.data_valid)
            {
                SMLK_LOGW("[dm1test] index[0x%02X] is time out!!!!", pair.first);
                pair.second.data_valid = false;
            }
        }
    }

    if (!(g_dm1_data_pool.find(0xF3)->second.data_valid) && !(g_dm1_data_pool.find(0x1A)->second.data_valid)
        && !(g_dm1_data_pool.find(0xEF)->second.data_valid)) {
        
        dm1info = {};

        for (auto &pair : m_protocol_reporter) {
            if (pair.second->IsRunning()) {
                pair.second->UpdateDM1Info(dm1info);
            }
        }
    }


}

/******************************************************************************
 * NAME: OnDM1Notify
 *
 * DESCRIPTION: DM1 raw CAN data notify
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void ReportService::OnDM1Notify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    if ( data.size() < sizeof(RawDM1) ) {
        SMLK_LOGE("body size is too small, at least %u", sizeof(RawDM1));
        return;
    }

    auto dm1_r = reinterpret_cast<const RawDM1 *>(data.data());

    SMLK_UINT8 src = dm1_r->can_src;

    if (g_dm1_data_pool.find(src) == g_dm1_data_pool.end()) {
        // SMLK_LOGE("[dm1debug] g_dm1_data_pool not support index[0x%02X]", src);
        return;
    }

    g_dm1_data_pool.find(src)->second.tp = std::chrono::steady_clock::now();
    g_dm1_data_pool.find(src)->second.data_valid = true;

    std::lock_guard<std::mutex> lk(g_dm1_data_pool_mutex);

    std::size_t left = data.size() - sizeof(RawDM1);

    // if (0 != left % sizeof(dm1_r->dtcs[0])) {
    //     SMLK_LOGE("invalid body size:   %u", data.size());
    //     return;
    // }

    // for can fd, it will have at most 64 bytes data
    // static const std::size_t max_bytes = 64;

    // if (data.size() > max_bytes) {
    //     SMLK_LOGE("invalid data received, data.size() = %u", data.size());
    //     return;
    // }

    /**********************************************************************
     * For now, we only have the SPN conversion method 0 which will be looks 
     * like as following (SAE J1939 version 4):
     * _____________________________________________________________________
     * | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 |
     * |87654321|87654321|87654321|87654321|87654321|87654321|
     * |        |        | SPN-L  | SPN-M  | SPN-H  |        |
     * 
     * Byte 0: lamp status
     * Byte 1: reserved lamp status
     * Byte 2 ~ Byte 5: the first DTC
     * 
     * SPN = Byte 4 >> 5;
     * SPN <<= 8;
     * SPN |= Byte 3
     * SPN << = 8;
     * SPN |= Byte 2
     * 
     * CM = Byte 5 & 0x80 >> 7;
     * OC = Byte 5 & 0x7F;
    ***********************************************************************/
    auto dtc = &dm1_r->dtcs[0];
    while ( left >= sizeof(dm1_r->dtcs[0]) ) {
        InternalDTC internal;

        SMLK_LOGD("[dm1test] %02X %02X %02X %02X", dtc->Byte0, dtc->Byte1, dtc->Byte2, dtc->Byte3 );

        internal.dtc.SRC    = src;
        internal.dtc.FMI    = dtc->Byte2 & 0x1F;
        internal.dtc.SPN    = dtc->Byte2 >> 5;
        internal.dtc.SPN    <<= 8;
        internal.dtc.SPN    |= dtc->Byte1;
        internal.dtc.SPN    <<= 8;
        internal.dtc.SPN    |= dtc->Byte0;

        SMLK_LOGD("[dm1test] internal.dtc.SPN:    %u", internal.dtc.SPN);
        SMLK_LOGD("[dm1test] internal.dtc.FMI:    %u", internal.dtc.FMI);
        SMLK_LOGD("[dm1test] internal.dtc.SRC:    %u", internal.dtc.SRC);
        SMLK_LOGD("[dm1test] internal.id:         0x%08X", internal.id);

        if (g_dm1_data_pool.find(0xF3)->second.data_valid) {
            SMLK_LOGD("[dm1test] g_dm1_data_pool.find(0xF3) status true");
            if (internal.dtc.SRC == 0xF3) {
                if (internal.dtc.SPN != 0) {
                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523304 && internal.dtc.FMI == 16) {
                        dm1info.dm1event[0].errcode = 1;
                        dm1info.dm1event[0].errvalue = internal.dtc.FMI;
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523313) {
                        if (internal.dtc.FMI == 16 || internal.dtc.FMI == 0) {
                            dm1info.dm1event[1].errcode = 2;
                            dm1info.dm1event[1].errvalue = internal.dtc.FMI;
                        }
                        if (internal.dtc.FMI == 3) {
                            dm1info.dm1event[1].errcode = 3;
                            dm1info.dm1event[1].errvalue = internal.dtc.FMI;
                        }
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523300) {
                        if (internal.dtc.FMI == 16 || internal.dtc.FMI == 0 || internal.dtc.FMI == 3) {
                            dm1info.dm1event[2].errcode = 2;
                            dm1info.dm1event[2].errvalue = internal.dtc.FMI;
                        }
                        if (internal.dtc.FMI == 18 || internal.dtc.FMI == 1) {
                            dm1info.dm1event[3].errcode = 2;
                            dm1info.dm1event[3].errvalue = internal.dtc.FMI;
                        }
                        if (internal.dtc.FMI == 4) {
                            dm1info.dm1event[3].errcode = 3;
                            dm1info.dm1event[3].errvalue = internal.dtc.FMI;
                        }
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523308) {
                        if (internal.dtc.FMI == 18) {
                            dm1info.dm1event[4].errcode = 1;
                            dm1info.dm1event[4].errvalue = internal.dtc.FMI;
                        }
                        if (internal.dtc.FMI == 4 || internal.dtc.FMI == 5) {
                            dm1info.dm1event[4].errcode = 2;
                            dm1info.dm1event[4].errvalue = internal.dtc.FMI;
                        }
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523305) {
                        if (internal.dtc.FMI == 16 || internal.dtc.FMI == 0 || internal.dtc.FMI == 3) {
                            dm1info.dm1event[5].errcode = 2;
                            dm1info.dm1event[5].errvalue = internal.dtc.FMI;
                        }
                        if (internal.dtc.FMI == 18 || internal.dtc.FMI == 1) {
                            dm1info.dm1event[6].errcode = 2;
                            dm1info.dm1event[6].errvalue = internal.dtc.FMI;
                        }
                        if (internal.dtc.FMI == 4) {
                            dm1info.dm1event[6].errcode = 3;
                            dm1info.dm1event[6].errvalue = internal.dtc.FMI;
                        }
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523311 && internal.dtc.FMI == 0) {
                        dm1info.dm1event[7].errcode = 1;
                        dm1info.dm1event[7].errvalue = internal.dtc.FMI;
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523311 && internal.dtc.FMI == 12) {
                        dm1info.dm1event[8].errcode = 1;
                        dm1info.dm1event[8].errvalue = internal.dtc.FMI;
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523321 && internal.dtc.FMI == 12) {
                        dm1info.dm1event[9].errcode = 1;
                        dm1info.dm1event[9].errvalue = internal.dtc.FMI;
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523301 && internal.dtc.FMI == 3) {
                        dm1info.dm1event[10].errcode = 1;
                        dm1info.dm1event[10].errvalue = internal.dtc.FMI;
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523309) {
                        if (internal.dtc.FMI == 18 || internal.dtc.FMI == 1) {
                            dm1info.dm1event[11].errcode = 2;
                            dm1info.dm1event[11].errvalue = internal.dtc.FMI;
                        }
                        if (internal.dtc.FMI == 4) {
                            dm1info.dm1event[11].errcode = 3;
                            dm1info.dm1event[11].errvalue = internal.dtc.FMI;
                        }
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523320 && internal.dtc.FMI == 7) {
                        dm1info.dm1event[16].errcode = 2;
                        dm1info.dm1event[16].errvalue = internal.dtc.FMI;
                    }

                    if (internal.dtc.SRC == 0xF3 && internal.dtc.SPN == 523300 && internal.dtc.FMI == 12) {
                        dm1info.dm1event[18].errcode = 3;
                        dm1info.dm1event[18].errvalue = internal.dtc.FMI;
                    }
                } else {
                    SMLK_LOGD("[dm1test] g_dm1_data_pool.find(0xF3) SPN = 0");
                    dm1info.dm1event[0].errcode = 0;
                    dm1info.dm1event[0].errvalue = 0;
                    dm1info.dm1event[1].errcode = 0;
                    dm1info.dm1event[1].errvalue = 0;
                    dm1info.dm1event[2].errcode = 0;
                    dm1info.dm1event[3].errvalue = 0;
                    dm1info.dm1event[3].errcode = 0;
                    dm1info.dm1event[3].errvalue = 0;
                    dm1info.dm1event[4].errcode = 0;
                    dm1info.dm1event[4].errvalue = 0;
                    dm1info.dm1event[5].errcode = 0;
                    dm1info.dm1event[5].errvalue = 0;
                    dm1info.dm1event[6].errcode = 0;
                    dm1info.dm1event[6].errvalue = 0;
                    dm1info.dm1event[7].errcode = 0;
                    dm1info.dm1event[7].errvalue = 0;
                    dm1info.dm1event[8].errcode = 0;
                    dm1info.dm1event[8].errvalue = 0;
                    dm1info.dm1event[9].errcode = 0;
                    dm1info.dm1event[9].errvalue = 0;
                    dm1info.dm1event[10].errcode = 0;
                    dm1info.dm1event[10].errvalue = 0;
                    dm1info.dm1event[11].errcode = 0;
                    dm1info.dm1event[11].errvalue = 0;
                    dm1info.dm1event[16].errcode = 0;
                    dm1info.dm1event[16].errvalue = 0;
                    dm1info.dm1event[18].errcode = 0;
                    dm1info.dm1event[18].errvalue = 0;
                }
            }
        } else {
            dm1info.dm1event[0].errcode = 0;
            dm1info.dm1event[0].errvalue = 0;
            dm1info.dm1event[1].errcode = 0;
            dm1info.dm1event[1].errvalue = 0;
            dm1info.dm1event[2].errcode = 0;
            dm1info.dm1event[3].errvalue = 0;
            dm1info.dm1event[3].errcode = 0;
            dm1info.dm1event[3].errvalue = 0;
            dm1info.dm1event[4].errcode = 0;
            dm1info.dm1event[4].errvalue = 0;
            dm1info.dm1event[5].errcode = 0;
            dm1info.dm1event[5].errvalue = 0;
            dm1info.dm1event[6].errcode = 0;
            dm1info.dm1event[6].errvalue = 0;
            dm1info.dm1event[7].errcode = 0;
            dm1info.dm1event[7].errvalue = 0;
            dm1info.dm1event[8].errcode = 0;
            dm1info.dm1event[8].errvalue = 0;
            dm1info.dm1event[9].errcode = 0;
            dm1info.dm1event[9].errvalue = 0;
            dm1info.dm1event[10].errcode = 0;
            dm1info.dm1event[10].errvalue = 0;
            dm1info.dm1event[11].errcode = 0;
            dm1info.dm1event[11].errvalue = 0;
            dm1info.dm1event[16].errcode = 0;
            dm1info.dm1event[16].errvalue = 0;
            dm1info.dm1event[18].errcode = 0;
            dm1info.dm1event[18].errvalue = 0;
        }

        if (g_dm1_data_pool.find(0x1A)->second.data_valid) {
            SMLK_LOGD("[dm1test] g_dm1_data_pool.find(0x1A) status true");
            if (internal.dtc.SRC == 0x1A) {
                if (internal.dtc.SPN != 0) {
                    if (internal.dtc.SPN == 523504 && internal.dtc.FMI == 15) {
                        dm1info.dm1event[12].errcode = 1;
                        dm1info.dm1event[12].errvalue = internal.dtc.FMI;
                    }
                    if (internal.dtc.SPN == 523504 && internal.dtc.FMI == 16) {
                        dm1info.dm1event[12].errcode = 3;
                        dm1info.dm1event[12].errvalue = internal.dtc.FMI;
                    }
                    if (internal.dtc.SPN == 523505 && internal.dtc.FMI == 12) {
                        dm1info.dm1event[14].errcode = 3;
                        dm1info.dm1event[14].errvalue = internal.dtc.FMI;
                    }
                } else {
                    SMLK_LOGD("[dm1test] g_dm1_data_pool.find(0x1A) SPN = 0");
                    dm1info.dm1event[12].errcode = 0;
                    dm1info.dm1event[12].errvalue = 0;
                    dm1info.dm1event[14].errcode = 0;
                    dm1info.dm1event[14].errvalue = 0;
                }
            }
        } else {
            dm1info.dm1event[12].errcode = 0;
            dm1info.dm1event[12].errvalue = 0;
            dm1info.dm1event[14].errcode = 0;
            dm1info.dm1event[14].errvalue = 0;
        }

        dm1info.dm1event[13].errcode = 0;
        dm1info.dm1event[13].errvalue = 0;

        if (g_dm1_data_pool.find(0xEF)->second.data_valid) {
            SMLK_LOGD("[dm1test] g_dm1_data_pool.find(0xEF) status true");
            if (internal.dtc.SRC == 0xEF) {
                if (internal.dtc.SPN != 0) {
                    if (internal.dtc.SPN == 523211 && internal.dtc.FMI == 15) {
                        dm1info.dm1event[15].errcode = 2;
                        dm1info.dm1event[15].errvalue = internal.dtc.FMI;
                    }
                    if (internal.dtc.SPN == 523211 && internal.dtc.FMI == 0) {
                        dm1info.dm1event[15].errcode = 3;
                        dm1info.dm1event[15].errvalue = internal.dtc.FMI;
                    }
                    if (internal.dtc.SPN == 523209 && internal.dtc.FMI == 15) {
                        dm1info.dm1event[17].errcode = 2;
                        dm1info.dm1event[17].errvalue = internal.dtc.FMI;
                    }
                    if (internal.dtc.SPN == 523209 && internal.dtc.FMI == 0) {
                        dm1info.dm1event[17].errcode = 3;
                        dm1info.dm1event[17].errvalue = internal.dtc.FMI;
                    }
                } else {
                    SMLK_LOGD("[dm1test] g_dm1_data_pool.find(0xEF) SPN = 0");
                    dm1info.dm1event[15].errcode = 0;
                    dm1info.dm1event[15].errvalue = 0;
                    dm1info.dm1event[17].errcode = 0;
                    dm1info.dm1event[17].errvalue = 0;
                }
            }
        } else {
            dm1info.dm1event[15].errcode = 0;
            dm1info.dm1event[15].errvalue = 0;
            dm1info.dm1event[17].errcode = 0;
            dm1info.dm1event[17].errvalue = 0;
        }

        left -= sizeof(dm1_r->dtcs[0]);
        ++dtc;
    }

    dm1info.max_warn = 0;
    dm1info.warn_num = 0;
    dm1info.com_warn = 0;

    // printf("dm1 report \n");
    // for (int m = 0; m < 19; m++) {
    //     printf("%02d:%02d/%02d, ", m, dm1info.dm1event[m].errcode, dm1info.dm1event[m].errvalue);
    //     if (dm1info.dm1event[m].errcode != 0) {
    //         dm1info.warn_num++;
    //         dm1info.com_warn += (1 << m);
    //     }
    //     if (dm1info.dm1event[m].errcode > dm1info.max_warn)
    //         dm1info.max_warn = dm1info.dm1event[m].errcode;
    // }


    // printf(" \n");

    // SMLK_LOGD("[dm1test] dm1info.max_warn:    %d", dm1info.max_warn);
    // SMLK_LOGD("[dm1test] dm1info.warn_num:    %d", dm1info.warn_num);
    // SMLK_LOGD("[dm1test] dm1info.com_warn:    %ld", dm1info.com_warn);

    for (auto &pair : m_protocol_reporter) {
        if (pair.second->IsRunning()) {
            pair.second->UpdateDM1Info(dm1info);
        }
    }
}

/******************************************************************************
 * NAME: OnMutiDataNotify
 *
 * DESCRIPTION: mutidata information notification handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void ReportService::OnMutiDataNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("[T] ReportService::OnMutiDataNotify() %d %u %u", head.m_seq, head.m_extra, data.size());
    if (data.size() < 0) {
        SMLK_LOGE("[T] fail to get MutiData information");
        return;
    }

    switch ( head.m_seq ) {
        case VEHICLE_DATA_EV_SINGLE_BATTERY_VOL:
            if (head.m_extra) {
                std::vector<SMLK_UINT8> m_evdata = data;
                SMLK_UINT16 val = 0;

                // 这里判断是否电压在0——60V范围内
                for (size_t i=0; i<(data.size()/2); i++) {
                    val = m_evdata[i*2] + ((SMLK_UINT16)m_evdata[i*2+1]<<8);
                    if(0<=val && val<=60000) {
                        if(common32960::getInstance()->m_lastvoltinfo.end() != common32960::getInstance()->m_lastvoltinfo.find(i*2)) {
                            common32960::getInstance()->m_lastvoltinfo[i*2] = m_evdata[i*2];
                            common32960::getInstance()->m_lastvoltinfo[i*2+1] = m_evdata[i*2+1];
                        } else {
                            common32960::getInstance()->m_lastvoltinfo.insert(make_pair(i*2,m_evdata[i*2]));
                            common32960::getInstance()->m_lastvoltinfo.insert(make_pair(i*2+1,m_evdata[i*2+1]));
                        }
                    } else {
                        auto iter = common32960::getInstance()->m_lastvoltinfo.find(i*2);
                        if(iter != common32960::getInstance()->m_lastvoltinfo.end()) {
                            m_evdata[i*2] = iter->second;
                        } else {
                            m_evdata[i*2] = 0xFF;
                        }

                        auto iter1 = common32960::getInstance()->m_lastvoltinfo.find(i*2+1);
                        if(iter1 != common32960::getInstance()->m_lastvoltinfo.end()) {
                            m_evdata[i*2+1] = iter1->second;
                        } else {
                            m_evdata[i*2+1] = 0xFF;
                        }
                    }
                }

                common32960::getInstance()->m_voltinfo.valid = true;
                // common32960::getInstance()->m_voltinfo.mdata.clear();
                common32960::getInstance()->m_voltinfo.mdata = m_evdata;
            } else {
                common32960::getInstance()->m_voltinfo.valid = false;
                // common32960::getInstance()->m_voltinfo.mdata.clear();
            }

            break;
        case VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_TEMPERATURE:
            if (head.m_extra) {
                int m_len = data.size();
                SMLK_UINT8 m_evdata[m_len] = {0};
                // bzero(m_evdata, sizeof(SMLK_UINT8) * m_len);
                memcpy(&m_evdata, data.data(), data.size());

                common32960::getInstance()->m_tempinfo.valid = true;
                common32960::getInstance()->m_tempinfo.mdata.clear();
                // printf ("11 ==== \n");
                for (size_t i=0; i<data.size(); i++) {
                    common32960::getInstance()->m_tempinfo.mdata.push_back(m_evdata[i]);; // 逐个复制
                    // printf ("%02X ",common32960::getInstance()->m_tempinfo.mdata[i]);
                }
                // printf ("\n");
            } else {
                common32960::getInstance()->m_tempinfo.valid = false;
                // common32960::getInstance()->m_tempinfo.mdata.clear();
            }

            break;
        default:
            SMLK_LOGE("unsupported Mutidata event: 0x%02X", head.m_seq);
            break;
    }
}

/******************************************************************************
 * NAME: OnLocationNotify
 *
 * DESCRIPTION: location information notification handler
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void ReportService::OnLocationNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_seq ) {
        case Location::SL_SERVICE_READY:
            GetGNSS();
            break;
        default:
            SMLK_LOGE("[GNSS] unsupported location event: 0x%08X", head.m_seq);
            break;
    }
}

/******************************************************************************
 * NAME: GetGNSS
 *
 * DESCRIPTION: get GNSS information
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void ReportService::GetGNSS()
{
    smartlink_sdk::LocationInfo location;
    auto rc = smartlink_sdk::Location::GetInstance()->GetLocation(location);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != rc ) {
        SMLK_LOGE("[GNSS] fail to get location information, rc: %d", static_cast<std::int32_t>(rc));
    } else {
        ProtocolReporter::GNSS::Info    inf = {};

        inf.flags       = location.fix_valid ? ProtocolReporter::GNSS::Flags::SL_VALID : ProtocolReporter::GNSS::Flags::SL_INVALID;
        inf.latitude    = location.latitude;
        inf.longitude   = location.longitude;
        inf.altitude    = location.altitude;
        inf.heading     = location.heading;
        inf.speed       = location.speed;
        inf.timestamp   = location.utc;

        for ( auto &pair : m_protocol_reporter ) {
            if ( pair.second->IsRunning() ) {
                pair.second->UpdateGNSS(inf);
            }
        }
    }
}

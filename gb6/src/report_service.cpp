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
#include <obd.h>
#include <smlk_spn.h>
#include <smlk_log.h>
#include <smlk_tools.h>
#include <smlk_timer_new.h>
#include <report_service.h>
#include <message_body_17691.h>
#include <message_def_17691.h>
#include <vehicle_data_api.h>
#include <vehicle_data_index_def.h>

#include <smartlink_sdk_mcu.h>
#include <smartlink_sdk_tel.h>
#include <smartlink_sdk_location.h>
#include <smartlink_sdk_sys_time.h>
#include <smartlink_sdk_sys_power.h>
#include <smartlink_sdk_sys_property.h>
#include <smartlink_sdk_sys_property_def.h>

#include <inttypes.h>
#include "INIReader.h"

#include "Poco/Event.h"
#include "Poco/Dynamic/Struct.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/Dynamic/Pair.h"
#include "Poco/Dynamic/VarIterator.h"
#include "Poco/JSON/Array.h"
#include "Poco/JSON/Parser.h"

#include <result_handler.h>
#include "smlk_hsm_api.h"


#ifdef SL_SIMULATOR
#include <random>
#endif

using namespace smartlink;

using namespace Poco;
using namespace Poco::Dynamic;

namespace ReportSer {
    static const SMLK_UINT8     BUSINESS_PLATFORM           = 0x01; // 01: 企业平台
    static const SMLK_UINT8     NATIONAL_PLATFORM           = 0x02; // 02: 国家平台
}

namespace ProtocolVer {
    static const SMLK_UINT8     BJ                          = 0x01;             // 京六
    static const SMLK_UINT8     TS                          = 0x04;             // HJ
    static const SMLK_UINT8     HJ                          = 0x06;             // 唐六
}   // namespace ProtocolVer

namespace UDS {
namespace Config {
namespace Type {
    static const SMLK_UINT8     GB6                         = 0;
    static const SMLK_UINT8     Others                      = 1;
}   // namespace Type

namespace Priority {
    static const SMLK_UINT8     Highest                     = 10;
    static const SMLK_UINT8     High                        = 20;
    static const SMLK_UINT8     MediumAbove                 = 30;
    static const SMLK_UINT8     Medium                      = 40;
    static const SMLK_UINT8     MediumBelow                 = 50;
    static const SMLK_UINT8     Low                         = 60;
    static const SMLK_UINT8     Lowest                      = 70;
}   // namespace Priority

namespace Trigger {
    static const SMLK_UINT8     Cyclic                      = 0x00;
    static const SMLK_UINT8     ACC                         = 0x01;
    static const SMLK_UINT8     IGN                         = 0x02;
    static const SMLK_UINT8     Network                     = 0x03;
    static const SMLK_UINT8     Others                      = 0x04;
}   // namespace Trigger


/*uds*/
struct UdsInnerRequest
{
    SMLK_UINT8 type; /*0:国六 1:其他*/
    SMLK_UINT8 property; /*优先级值越小越高 (10，20，30.。。。70)*/
    SMLK_UINT8 trigger; /*采集触发时机:
                            0- 周期
                            1- ACC
                            2- IG ON
                            3- CAN唤醒
                            4- 其他
                            */
    SMLK_UINT16 time;/*单位: ms
                            单次表示采集触发后采集的延迟时间
                            周期则表示采集间隔
                    */
    SMLK_UINT32 req_id;/*Request ID*/
    SMLK_UINT32 rsp_id;/*Response ID*/
    SMLK_UINT32 svc_id;/*Service  ID*/
    SMLK_UINT16 extend_id;/*DID\RID*/
    SMLK_UINT16 len;
    SMLK_UINT8 *data;
};

static const std::array<UdsInnerRequest, 6>  did_diesel {
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::diesel::Request, obd::can::identifier::diesel::Response, obd::sid::ReadDataByIdentifier, obd::did::MIL,         0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::diesel::Request, obd::can::identifier::diesel::Response, obd::sid::ReadDataByIdentifier, obd::did::VIN,         0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::diesel::Request, obd::can::identifier::diesel::Response, obd::sid::ReadDataByIdentifier, obd::did::SCIN,        0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::diesel::Request, obd::can::identifier::diesel::Response, obd::sid::ReadDataByIdentifier, obd::did::CVN,         0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::diesel::Request, obd::can::identifier::diesel::Response, obd::sid::ReadDataByIdentifier, obd::did::IUPR_D,      0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::diesel::Request, obd::can::identifier::diesel::Response, obd::sid::ReadDataByIdentifier, obd::did::Protocol,    0, nullptr },
};

static const std::array<UdsInnerRequest, 6>  did_gas {
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::gas::Request, obd::can::identifier::gas::Response, obd::sid::ReadDataByIdentifier, obd::did::MIL,         0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::gas::Request, obd::can::identifier::gas::Response, obd::sid::ReadDataByIdentifier, obd::did::VIN,         0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::gas::Request, obd::can::identifier::gas::Response, obd::sid::ReadDataByIdentifier, obd::did::SCIN,        0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::gas::Request, obd::can::identifier::gas::Response, obd::sid::ReadDataByIdentifier, obd::did::CVN,         0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::gas::Request, obd::can::identifier::gas::Response, obd::sid::ReadDataByIdentifier, obd::did::IUPR_G,      0, nullptr },
    UdsInnerRequest{ Type::GB6, Priority::Highest, Trigger::IGN, 1000, obd::can::identifier::gas::Request, obd::can::identifier::gas::Response, obd::sid::ReadDataByIdentifier, obd::did::Protocol,    0, nullptr },
};

}   // namespace Config
}   // namespace UDS

namespace EMSModel {
    static const SMLK_UINT8     SL_INVALID                  = 0x00; // 00: 无
    static const SMLK_UINT8     SL_FAW_GB5_DIESEL           = 0x01; // 01: 预留
    static const SMLK_UINT8     SL_FAW_GB6_DIESEL           = 0x02; // 02: FAW 国六柴油机
    static const SMLK_UINT8     SL_BOSCH_GB5_DIESEL         = 0x03; // 03: 预留
    static const SMLK_UINT8     SL_BOSCH_GB6_GAS            = 0x04; // 04: Bosch 国六天然气
    static const SMLK_UINT8     SL_ECONTROLS_GB6_GAS        = 0x05; // 05: Econtrols 国六天然气
    static const SMLK_UINT8     SL_FAW_CUMMINS_GB6_DIESEL   = 0x06; // 06: 康明斯国六柴油机
    static const SMLK_UINT8     SL_FAW_CUMMINS_GB6_GASL     = 0x07; // 07: 康明斯国六天然气
    static const SMLK_UINT8     SL_WEICHAI_GB6_DIESEL       = 0x08; // 08: 潍柴国六柴油机
    static const SMLK_UINT8     SL_WEICHAI_GB6_GAS_OH6      = 0x09; // 09: 潍柴国六天然气_OH6
    static const SMLK_UINT8     SL_YUCHAI_GB6_DIESEL        = 0x0A; // 0A: 玉柴博世国六柴油机
    static const SMLK_UINT8     SL_YUCHAI_GB6_GAS           = 0x0B; // 0B: 玉柴 Econtrols 国六天然气
    static const SMLK_UINT8     SL_QUANCHAI_GB6_DIESEL      = 0x0C; // 0C: 全柴 C878 国六柴油机
    static const SMLK_UINT8     SL_YUNNEI_GB6_DIESEL        = 0x0D; // 0D: 云内国六柴油机
    static const SMLK_UINT8     SL_WEICHAI_GB6_GAS_WISE11   = 0x0E; // 0E: 潍柴国六天然气_WISE11
    static const SMLK_UINT8     SL_CUMMINS_GB6_DIESEL_9L    = 0x0F; // 0F: 非解放康明斯国六柴油机 9L
    static const SMLK_UINT8     SL_CUMMINS_GB6_NG_9L        = 0x10; // 10: 非解放康明斯国六 NG_9L
    static const SMLK_UINT8     SL_YUCHAI_DELPHI_GB6_DIESEL = 0x11; // 11: 玉柴德尔福国六柴油机
    static const SMLK_UINT8     SL_QUANCHAI_C81_GB6_DIESEL  = 0x12; // 12: 全柴 C81 国六柴油机


};

namespace Location {
    static const SMLK_UINT32    SL_SERVICE_READY            = 0x00000001;
    static const SMLK_UINT32    SL_SERVICE_EXIT             = 0x00000002;
};

namespace PowerState {
    static const SMLK_UINT32    SL_WAKEUP                   = 0x00000001;
    static const SMLK_UINT32    SL_SLEEPING                 = 0x00000002;
    static const SMLK_UINT32    SL_SLEEPING_CANCEL          = 0x00000003;
    static const SMLK_UINT32    SL_STATE_CHANGED            = 0x00010000;
    static const SMLK_UINT32    SL_SOURCE_CHANGED           = 0x00100000;

    static const SMLK_UINT32    SL_INVALID                  = 0xFFFFFFFF;
}

namespace MCU {
    static const SMLK_UINT32    SL_SERVICE_READY            = 0x000000001;
    static const SMLK_UINT32    SL_IGN_NOTIFY               = 0x000000002;
};

namespace SysTime {
    static const SMLK_UINT32    SL_SYSTIME_SYN_DEF          = 0x00000001;
    static const SMLK_UINT32    SL_SYSTIME_SYN_OK           = 0x00000002;
}

namespace IGState {
    static const SMLK_UINT32    SL_IGN_OFF                  = 0x000000000;
    static const SMLK_UINT32    SL_IGN_ON                   = 0x000000001;
};

namespace HJActState {
    static const SMLK_UINT32    SL_ACT_SUCCESS              = 0x000000000;
    static const SMLK_UINT32    SL_IGN_ON                   = 0x000000001;
};

namespace TSP {
    static const SMLK_UINT32    SL_ACTIVATE_NOTIFY          = 0x000000001;
};


namespace InternalFlags {
    static const SMLK_UINT32    SL_FLAG_IGN_ON              = 0x000000001;
    static const SMLK_UINT32    SL_FLAG_SYSTIME_SYN         = 0x000000002;
}

namespace {
    static const int            SL_QUEUE_GET_TIMEOUT_MS     = 200;              // timeout millisecond to get item from queue
    static const SMLK_UINT32    SL_OBD_QUERY_DELAY_DEF      = 10000;            // default OBD query delay after IGN ON
    static const SMLK_UINT32    SL_RTC_TIME_SOURCE_DEF      = 5;                // TimeSource::E_TIME_SOURCE_RTC

    static const SMLK_UINT32    SL_TIMER_ID_OBD_QUERY       = 0x00000001;           // timer id for OBD query
    static const SMLK_UINT32    SL_TIMER_ID_DAYCHANGE       = 0x00000002;           // timer id for day change

    static const SMLK_UINT32    SL_EVENT_LOCAT_NOTIFY       = 0xFF000001;           // event LOCAT notify
    static const SMLK_UINT32    SL_EVENT_MCU_NOTIFY         = 0xFF000002;           // event MCU notify
    static const SMLK_UINT32    SL_EVENT_SYSPOWER_NOTIFY    = 0xFF000003;           // event sys power notify
    static const SMLK_UINT32    SL_EVENT_808_MSG_NOTIFY     = 0xFF000004;           // event 808 tsp notify
    static const SMLK_UINT32    SL_EVENT_TIMER_1S_NOTIFY    = 0xFF000006;           // event 1s timer notify
    static const SMLK_UINT32    SL_EVENT_SHARED_TIMER       = 0xFF000010;           // event shared timer notify

    static const SMLK_UINT32    SL_EVENT_UDS_RESPONSE       = 0xFF000100;           // event UDS response

    static const SMLK_UINT32    SL_FLAG_SYSTIME_SER         = 0x00000001;
    static const SMLK_UINT32    SL_FLAG_SYSTIME_SYN         = 0x00000002;
};

SMLK_BOOL isIP(const char *str)
{
    int a, b, c, d;
    char temp[100];
    if ((sscanf(str, "%d.%d.%d.%d", &a, &b, &c, &d)) != 4)
        return false;
    sprintf(temp, "%d.%d.%d.%d", a, b, c, d);
    if (strcmp(temp, str) != 0)
        return false;
    if (!((a <= 255 && a >= 0) && (b <= 255 && b >= 0) && (c <= 255 && c >= 0)))
        return false;
    else
        return true;
}

ReportService::ReportService()
    : m_service_flags(IService::Uninitialized)
    , m_ign_flags(IGState::SL_IGN_OFF)
    , m_systime_flags(SysTime::SL_SYSTIME_SYN_DEF)
    , m_obd_query_delay(SL_OBD_QUERY_DELAY_DEF)
    , m_product_loc(M_GB_LOCATION_CHANGCHUN)
    , m_ems_model(EMSModel::SL_FAW_GB6_DIESEL)
    , m_gb_platform(0x02)
    , m_time_source(SL_RTC_TIME_SOURCE_DEF)
    , m_timeSyncGap_ms(0)
    , m_todisk(0)
    , is_gas(false)
    , is_hev(false)
    , m_pub_key("")
    , m_env(0)
{
    m_ent_tspinfo.platform = ReportSer::BUSINESS_PLATFORM;
    m_loc_tspinfo.platform = ReportSer::NATIONAL_PLATFORM;
}

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
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_service_flags & IService::Initialized ) {
        SMLK_LOGD("already initialized, do nothing");
        return SL_SUCCESS;
    }

    if ( !tools::dir_exists(M_GB6_RECORD_DATABASE_DIR) ) {
        if ( 0 != tools::mkdir_p(M_GB6_RECORD_DATABASE_DIR) ) {
            SMLK_LOGE("fail to create database dir");
            return SL_EFAILED;
        }
    }

    if ( !tools::dir_exists(M_GB6_CONFIG_DIR) ) {
        if ( 0 != tools::mkdir_p(M_GB6_CONFIG_DIR) ) {
            SMLK_LOGE("fail to create database dir");
            return SL_EFAILED;
        }
    }

    SMLK_UINT8 data[] = {
        obd::sfid::ReportWWHOBDDTCByMaskRecord,                 // sub-function
        obd::dtc::FGID::emissions,                              // functional group id
        obd::dtc::status::PDTC | obd::dtc::status::CDTC         // dtc status pending and confirmed
    };

    INIReader reader(M_GB6_CONFIG_FILE_DIR);

    if (reader.ParseError() < 0) {
        SMLK_LOGD("[T] can't load Configure file: %s ", M_GB6_CONFIG_FILE_DIR);
    } else {
        SMLK_LOGD("[T] load Configure file: %s ", M_GB6_CONFIG_FILE_DIR);
        m_env       = reader.GetInteger("environment", "env", 0);
        m_todisk    = reader.GetInteger("rawdata", "todisk", 0);

        m_host_01   = reader.GetString("remote", "server01", "");
        m_port_01   = reader.GetInteger("remote", "port01", 0);
        m_host_02   = reader.GetString("remote", "server02", "");
        m_port_02   = reader.GetInteger("remote", "port02", 0);
        m_host_03   = reader.GetString("remote", "server03", "");
        m_port_03   = reader.GetInteger("remote", "port03", 0);

        m_chipid_head   = reader.GetString("secureelement", "head", "");
        SMLK_LOGE("read ini file, [%s] %s:%d, %s:%d, %s:%d, save %d", m_env ? "DEV" : "PRD", m_host_01.c_str(), m_port_01, m_host_02.c_str(), m_port_02, m_host_03.c_str(), m_port_03, m_todisk);
    }

    if ( !smartlink_sdk::SysProperty::GetInstance()->Init(ModuleID::E_Module_gb6_service) )
    {
        SMLK_LOGE("fail to init system property module");
        return SL_EFAILED;
    }

    std::string  property;

    const std::vector<std::string> property_key = {
        std::string(SYS_PRO_NAME_DID_VEHICLE_TYPE),
        std::string(SYS_PRO_NAME_STD_DISCHAR_GW_SET),
        std::string(SYS_PRO_NAME_GB_RECORDED_STATUS),
        std::string(SYS_PRO_NAME_ENC_CHIP_ID_SET),
        std::string(SYS_PRO_NAME_HJ_ACTIVATED_STATUS),
    };

    auto return_code = smartlink_sdk::SysProperty::GetInstance()->RegEventCB(
        property_key,
        [this](smartlink_sdk::SysProperty::PropertyEventId id, smartlink_sdk::PropertyInfo* info) {
            switch ( id ) {
                case smartlink_sdk::SysProperty::PropertyEventId::E_SYS_PROPERTY_EVENT_VALUE_CHANGED:
                {
                    if(info->name == std::string(SYS_PRO_NAME_DID_VEHICLE_TYPE))
                    {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[property] %s, value is %s!!!", info->name.c_str(), info->value.c_str());
                            return;
                        }
                    }
                    else if(info->name == std::string(SYS_PRO_NAME_STD_DISCHAR_GW_SET))
                    {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[property] %s, value is %s!!!", info->name.c_str(), info->value.c_str());
                        }
                        Post(MsgHead(SL_EVENT_MCU_NOTIFY, MCU::SL_IGN_NOTIFY, 0, 0, 0, IGState::SL_IGN_OFF), nullptr, 0);
                        return;
                    }
                    else if(info->name == std::string(SYS_PRO_NAME_GB_RECORDED_STATUS))
                    {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[property] %s, value is %s!!!", info->name.c_str(), info->value.c_str());
                            return;
                        }
                        
                        SMLK_UINT8 m_state = 0;

                        Poco::JSON::Parser parser;
                        Poco::Dynamic::Var var_json;
                        /*判断参数json合法性*/
                        try
                        {
                            var_json = parser.parse(info->value.c_str());
                        }
                        catch (Poco::Exception &exc)
                        {
                            SMLK_LOGE("Parse Error:%s", exc.displayText().c_str());
                            return;
                        }
                        /*判断json中RecRes字段是否存在且数据类型为int*/
                        SMLK_UINT8 m_rec_sta = 0;
                        SMLK_UINT8 m_rec_res = 0;
                        Poco::JSON::Object obj = *var_json.extract<Poco::JSON::Object::Ptr>();
                        Poco::Dynamic::Var sn = obj.get("SN");
                        if (!sn.isString())
                        {
                            SMLK_LOGD("JSON \"SN\" type error!");
                            resulthandler::getInstance()->m_808_sn = "";
                        }
                        else
                        {
                            resulthandler::getInstance()->m_808_sn = sn.convert<std::string>();
                        }

                        Poco::Dynamic::Var rec_sta = obj.get("RecSta");
                        if (!rec_sta.isInteger())
                        {
                            SMLK_LOGD("JSON \"RecSta\" type error!");
                            m_rec_sta = 0;
                        }
                        else
                        {
                            m_rec_sta = rec_sta.convert<SMLK_UINT8>();
                        }

                        Poco::Dynamic::Var rec_res = obj.get("RecRes");
                        if (!rec_res.isInteger())
                        {
                            SMLK_LOGD("JSON \"RecSta\" type error!");
                            m_rec_res = 0;
                        }
                        else
                        {
                            m_rec_res = rec_res.convert<SMLK_UINT8>();
                        }

                        if (m_rec_sta == 1 || (m_rec_sta == 3 && m_rec_res != 2))
                        {
                            m_state = M_HJ_1239_ACTIVATED_ING;
                        } else if (m_rec_sta == 0 || m_rec_sta == 3) {
                            m_state = M_HJ_1239_ACTIVATED_RESERVE;
                        }

                        for (auto &pair : m_protocol_reporter)
                        {
                            pair.second->SetActStatus(m_state);
                        }

                        m_protocol_reporter_1239_07->SetActStatus(m_state);

                        Post(MsgHead(SL_EVENT_808_MSG_NOTIFY, TSP::SL_ACTIVATE_NOTIFY, 0, 0, 0, m_state), nullptr, 0);
                        
                        return;
                    }
                    else if(info->name == std::string(SYS_PRO_NAME_ENC_CHIP_ID_SET))
                    {
                        if (!info->value.empty())
                        {
                            SMLK_LOGI("[property] %s, value is %s!!!", info->name.c_str(), info->value.c_str());
                        }
                        m_protocol_reporter_1239_07->SetChipID(info->value.c_str());
                        return;
                    }
                }
            }
        }
    );


    std::string stdDiscIpSet = "";
    property = SYS_PRO_NAME_STD_DISCHAR_GW_SET;
    return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, stdDiscIpSet);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
        SMLK_LOGE("Set name=%s as %s fail!", SYS_PRO_NAME_STD_DISCHAR_GW_SET, stdDiscIpSet.c_str());
    }

    std::string config;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, config);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code)
    {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        SMLK_LOGI("8F42 config:        %s", config.c_str());
    }

    /*
     * 00: J7 中低配 (柴油机)
     * 01: J7 高配 (柴油机)
     * 02: J6 低配 (柴油机)
     * 03: J6 中配 (柴油机)
     * 04: J6 高配 (柴油机)
     * 05: J7 中低配 (LNG天然气)
     * 06: J7 高配 (天然气)
     * 07: J6 低配 (天然气)
     * 08: J6 中配 (天然气)
     * 09: J6 高配 (天然气)
     * 0A: 新能源纯电车型
     * 0B: 新能源混动车型
     * 0C: 青汽架构 (柴油)
     * 0D: 青汽架构 (天然气)
     */
    std::string     vehicletype;
    SMLK_UINT8      m_vehicle_type;
    property = SYS_PRO_NAME_DID_VEHICLE_TYPE;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, vehicletype);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        m_vehicle_type = std::stoi(vehicletype);
        SMLK_LOGI("vehicletype:        %u", m_vehicle_type);
    }

    std::string     evmode;
    SMLK_UINT8      ev_run_mode = 0;
    property = SYS_PRO_NAME_DID_NEW_ENERGY_EMISSION_REMOTE_MONITOR_CONFIG;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, evmode);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        ev_run_mode = std::stoi(evmode);
        SMLK_LOGI("config:        %u", ev_run_mode);
    }

    if (vehicletype == 0x0A || vehicletype == 0x0B || vehicletype == 0x0C || vehicletype == 0x0D)
    {
        if (ev_run_mode == 0x03 || ev_run_mode == 0x04)
        {
            is_hev = true;
        }
        else if (ev_run_mode == 0x01 || ev_run_mode == 0x02)
        {
            SMLK_LOGI("Do not send g6 protocol");
        }
    }

    /*
     *00: 无
     *01: 预留
     *02: FAW 国六柴油机 
     *03: 预留
     *04: Bosch 国六天然气
     *05: Econtrols 国六天然气
     *06: 康明斯国六柴油机
     *07: 康明斯国六天然气
     *08: 潍柴国六柴油机
     *09: 潍柴国六天然气_OH6
     *0A: 玉柴博世国六柴油机
     *0B: 玉柴 Econtrols 国六天然气
     *0C: 全柴 C878 国六柴油机
     *0D: 云内国六柴油机
     *0E: 潍柴国六天然气_WISE11
     *0F: 非解放康明斯国六柴油机 9L
     *10: 非解放康明斯国六 NG_9L
     *11: 玉柴德尔福国六柴油机
     *12: 全柴 C81 国六柴油机
    */
    std::string     ems;
    property = SYS_PRO_NAME_EMS;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, ems);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        m_ems_model = std::stoi(ems);
        SMLK_LOGI("ems:        %u", m_ems_model);
    }

    if (EMSModel::SL_BOSCH_GB6_GAS == m_ems_model || EMSModel::SL_ECONTROLS_GB6_GAS == m_ems_model ||
        EMSModel::SL_FAW_CUMMINS_GB6_GASL == m_ems_model || EMSModel::SL_WEICHAI_GB6_GAS_OH6 == m_ems_model ||
        EMSModel::SL_YUCHAI_GB6_GAS == m_ems_model || EMSModel::SL_WEICHAI_GB6_GAS_WISE11 == m_ems_model ||
        EMSModel::SL_CUMMINS_GB6_NG_9L == m_ems_model)
    {
        is_gas = true;
    }
    else if (EMSModel::SL_FAW_GB6_DIESEL == m_ems_model || EMSModel::SL_FAW_CUMMINS_GB6_DIESEL == m_ems_model ||
             EMSModel::SL_WEICHAI_GB6_DIESEL == m_ems_model || EMSModel::SL_YUCHAI_GB6_DIESEL == m_ems_model ||
             EMSModel::SL_QUANCHAI_GB6_DIESEL == m_ems_model || EMSModel::SL_YUNNEI_GB6_DIESEL == m_ems_model ||
             EMSModel::SL_QUANCHAI_GB6_DIESEL == m_ems_model || EMSModel::SL_YUNNEI_GB6_DIESEL == m_ems_model ||
             EMSModel::SL_CUMMINS_GB6_DIESEL_9L == m_ems_model || EMSModel::SL_YUCHAI_DELPHI_GB6_DIESEL == m_ems_model ||
             EMSModel::SL_QUANCHAI_C81_GB6_DIESEL == m_ems_model)
    {
        is_gas = false;
    }
    else
    {
        SMLK_LOGW("------EMS Model not surpport %d------", m_ems_model);
    }

    // m_vin = "SMLKA1GB6TEST8720";
    // property = SYS_PRO_NAME_VEHICLE_VIN;
    // SMLK_LOGD("get vin is [%s]", m_vin.c_str());

    property = SYS_PRO_NAME_VEHICLE_VIN;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, m_vin);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        SMLK_LOGI("vin:             %s", m_vin.c_str());
    }

    property = SYS_PRO_NAME_GB_CHECK_VIN;
    return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, "0");
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
        SMLK_LOGE("fail to set system property \"%s\", return code: %d",  property.c_str(), static_cast<std::int32_t>(return_code));
    }

    property = SYS_PRO_NAME_OBD_VIN;
    return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, "00000000000000000");
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
        SMLK_LOGE("fail to set system property \"%s\", return code: %d", property, static_cast<std::int32_t>(return_code));
    }

    // 1. 0110 判断产地
    /*
     * 00: 长春
     * 01: 青岛
     */
    SMLK_BOOL       is_setloc;
    std::string     productloc;
    property = SYS_PRO_NAME_PRODUCT_LOC;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, productloc);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
        is_setloc = false;
    } else {
        m_product_loc = std::stoi(productloc);
        is_setloc = true;
    }
    SMLK_LOGD("loc:        %s",m_product_loc == 0?"Changchun":"Qingdao");

    // 2. 102B 读取排放法规
    /*
    00: 无              // 未使用
    01: 北京京六        // 唐六 (企业) + 京六 (地方)
    02: 唐山, 上海国六  // 唐六
    03: 唐山, 上海国五  // 未使用
    04: 全国国六        // 未使用
    05: 北京在用车国五  // 未使用
    06: HJ 国六柴油     // HJ
    07: HJ 国六天然气   // HJ
    08: HJ 国五         // 未使用
    09: HJ, 京六        // HJ (企业) + 京六 (地方)
    */
    std::string platform;
    property = SYS_PRO_NAME_GB_PLATFORM;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, platform);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        m_gb_platform = std::stoi(platform);
        SMLK_LOGI("gb platform:     %u", m_gb_platform);
    }

    if (m_gb_platform == 0x01) {
        m_ent_tspinfo.valid = true;
        m_ent_tspinfo.protocol = ProtocolVer::TS;

        m_loc_tspinfo.valid = true;
        m_loc_tspinfo.protocol = ProtocolVer::BJ;
    } else if (m_gb_platform == 0x02) {
        m_ent_tspinfo.valid = true;
        m_ent_tspinfo.protocol = ProtocolVer::TS;

        m_loc_tspinfo.valid = false;
    } else if (m_gb_platform == 0x06 || m_gb_platform == 0x07) {
        m_ent_tspinfo.valid = true;
        m_ent_tspinfo.protocol = ProtocolVer::HJ;

        m_loc_tspinfo.valid = false;
    } else if (m_gb_platform == 0x09) {
        m_ent_tspinfo.valid = true;
        m_ent_tspinfo.protocol = ProtocolVer::HJ;

        m_loc_tspinfo.valid = true;
        m_loc_tspinfo.protocol = ProtocolVer::BJ;
    } else {
        SMLK_LOGI("gb platform %d not supported, use default gb platform 2", m_gb_platform);

        m_gb_platform = 0x02;
        m_ent_tspinfo.valid = true;
        m_ent_tspinfo.protocol = ProtocolVer::TS;

        m_loc_tspinfo.valid = false;
    }

    std::string gb_tcp_ip;
    std::string gb_tcp_port;

    property = m_product_loc == M_GB_LOCATION_CHANGCHUN ? SYS_PRO_NAME_G6_ENT_TCP_IP_CC : SYS_PRO_NAME_G6_ENT_TCP_IP_QD;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, gb_tcp_ip);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        m_ent_tspinfo.host = gb_tcp_ip;
        SMLK_LOGI("%s ent host:            %s", m_product_loc == 0?"Changchun":"Qingdao", m_ent_tspinfo.host.c_str());
    }

    property = m_product_loc == M_GB_LOCATION_CHANGCHUN ? SYS_PRO_NAME_G6_ENT_TCP_PORT_CC : SYS_PRO_NAME_G6_ENT_TCP_PORT_QD;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, gb_tcp_port);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        m_ent_tspinfo.port = std::stoi(gb_tcp_port);
        SMLK_LOGI("%s ent port:            %u", m_product_loc == 0?"Changchun":"Qingdao", m_ent_tspinfo.port);
    }

    property = m_product_loc == M_GB_LOCATION_CHANGCHUN ? SYS_PRO_NAME_G6_LOC_TCP_IP_CC : SYS_PRO_NAME_G6_LOC_TCP_IP_QD;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, gb_tcp_ip);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        m_loc_tspinfo.host = gb_tcp_ip;
        SMLK_LOGI("%s loc host:            %s", m_product_loc == 0?"Changchun":"Qingdao", m_loc_tspinfo.host.c_str());
    }

    property = m_product_loc == M_GB_LOCATION_CHANGCHUN ? SYS_PRO_NAME_G6_LOC_TCP_PORT_CC : SYS_PRO_NAME_G6_LOC_TCP_PORT_QD;
    return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, gb_tcp_port);
    if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
        SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    } else {
        m_loc_tspinfo.port = std::stoi(gb_tcp_port);
        SMLK_LOGI("%s loc port:            %u", m_product_loc == 0?"Changchun":"Qingdao", m_loc_tspinfo.port);
    }

    std::string act_host;
    SMLK_UINT16 act_port;

    if (m_env)
    {
        act_host = m_host_03;
        act_port = m_port_03;
    }
    else
    {
        std::string hj_act_ip;
        property = SYS_PRO_NAME_HJ_ACTIVE_TCP_IP;
        return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, hj_act_ip);
        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code)
        {
            SMLK_LOGE("fail to get ev ip \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
        }
        else
        {
            act_host = hj_act_ip;
            SMLK_LOGI("act host:        %s", act_host.c_str());
        }

        std::string hj_act_port;
        property = SYS_PRO_NAME_HJ_ACTIVE_TCP_PORT;
        return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, hj_act_port);
        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code)
        {
            SMLK_LOGE("fail to get ev port \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
        }
        else
        {
            act_port = std::stoi(hj_act_port);
            SMLK_LOGI("act port:        %u", act_port);
        }
    }

    if (isIP(act_host.data()) == false) {
        char **pptr;
        char str[32] = {0};
        struct hostent *hptr = ::gethostbyname(act_host.c_str());
        if (hptr == nullptr) {
            SMLK_LOGE("[client] gethostbyname failed. \n");
        } else {
            pptr = hptr->h_addr_list;
            const char *ip = inet_ntop(hptr->h_addrtype, hptr->h_addr, str, sizeof(str));
            act_host = ip;
        }
        SMLK_LOGE("[client] gethostbyname success. %s \n", act_host.c_str());
    }

    SMLK_LOGD("[timesyncDebug] init systime service");
    if (!smartlink_sdk::SysTime::GetInstance()->Init(ModuleID::E_Module_gb6_service)) {
        SMLK_LOGE("fail to init systime service API interface!!!");
        return SL_EFAILED;
    }

    std::vector<smartlink_sdk::SysTimeEventId> timer_event = {
        smartlink_sdk::SysTimeEventId::E_TIME_EVENT_TIME_CHANGED,
    };
    smartlink_sdk::SysTime::GetInstance()->RegEventCB(
        timer_event,
        [this](smartlink_sdk::SysTimeEventId id, void *data, int len) {
            SMLK_LOGD("***** [timesyncDebug] SysTime id = %d, .\n", id);
            switch (id) {
                case smartlink_sdk::SysTimeEventId::E_TIME_EVENT_TIME_CHANGED: {
                    smartlink_sdk::SysTimeEventInfo *info = (smartlink_sdk::SysTimeEventInfo *)data;
                    SMLK_LOGI("[timesyncDebug]***** timesync = [%d]", info->timesync);
                    SMLK_LOGI("[timesyncDebug] TimeSource = [%d], 1:GNSS 2:NTP 3:NITZ 4:TSP 5:RTC", info->source);
                    SMLK_LOGI("[timesyncDebug] gap us is = [%ld] ", info->us);
                    SMLK_UINT32 new_time_source = (SMLK_UINT32)info->source;
                    SMLK_INT32 change_ms = 0;
                    if (0 != info->us) {
                        change_ms = SMLK_INT32(info->us / 1000);
                        SMLK_LOGI("[timesyncDebug] change_ms = [%ld] ", change_ms);
                    }
                    // 恒润: 同优先级的时间源 判断<5s 是可信的; 高优先级校时没有此判断
                    if (info->timesync == 1) {
                        // 业务数据中的时间, 以TBOX拿到的第一个可信任的授时源为准, 最低标准为NTP
                        if (smartlink_sdk::TimeSource::E_TIME_SOURCE_NTP == new_time_source ||
                            smartlink_sdk::TimeSource::E_TIME_SOURCE_GNSS == new_time_source) {
                            SMLK_LOGI("[timesyncDebug] post change_ms = %ld", change_ms);
                            m_time_source = new_time_source;

                            // 从未授时时,先缓存数据,然后在授时后修改缓存的GNSSTimeStamp字段,从缓存中取出所有数据发送
                            // 已有ntp授时后 取系统时间 再出现GNSS授时, systime + (授时差)
                            if (m_systime_flags == SysTime::SL_SYSTIME_SYN_OK) {
                                SMLK_LOGI("[timesyncDebug] last m_timeSyncGap_ms %lld !!!", m_timeSyncGap_ms);
                                SMLK_LOGI("[timesyncDebug] change_ms = [%ld] ", change_ms);
                                // 负值: 授时后系统时间 < 授时前时间 正值: 授时后系统时间 > 授时前系统时间
                                // 为保持和旧系统获取的时戳 连续 newtp = newSysTime - (m_timeSyncGap_ms)
                                m_timeSyncGap_ms += change_ms;
                                SMLK_LOGI("[timesyncDebug] new m_timeSyncGap_ms  %lld !!!", m_timeSyncGap_ms);

                                for ( auto &pair : m_protocol_reporter ) {
                                    if ( pair.second->IsRunning() ) {
                                        pair.second->SetTimeSyncGap(m_timeSyncGap_ms);
                                    }
                                }
                            }
                            m_systime_flags = SysTime::SL_SYSTIME_SYN_OK;
                        }
                    } else {
                        // 同优先级的时间源 判断>5s
                        // 此种授时适用debug手动调同级时间
                        SMLK_LOGI("[timesyncDebug]***** time sync false");
                    }
                }
                break;
            default:
                SMLK_LOGE("[timesyncDebug] unsupported event id: %d", (SMLK_UINT8)id);
                return;
            }
            return;
        }
    );

    SMLK_LOGD("init vehicle-data service");
    auto vec_ret = VehicleDataApi::GetInstance()->Init(ModuleID::E_Module_gb6_service);
    if ( SMLK_RC::RC_OK != vec_ret ) {
        SMLK_LOGE("fail to init vehicle data service API interface!!!");
        return SL_EFAILED;
    }

    indexs.reserve(vehicle::g_vehicle_signal_index.size());
    for (auto const &pair : vehicle::g_vehicle_signal_index) {
        indexs.push_back(pair.second);
    }

    SMLK_LOGD("register vehicle-data event callback");
    vec_ret = VehicleDataApi::GetInstance()->RegistVehicleDataChangedCB(
        indexs,
        [this](VehicleData data){
            for ( auto &pair : m_protocol_reporter ) {
                if ( pair.second->IsRunning() ) {
                    // SMLK_LOGD ("data.index = %ld, data.value = %lf, data.valid = %d \n", data.index, data.value, data.valid);
                    pair.second->UpdateSignalVal(data.index, data.value, data.valid);
                }
            }
        }
    );
    if ( SMLK_RC::RC_OK != vec_ret ) {
        SMLK_LOGE("fail to register vehicle data changed callback");
        return SL_EFAILED;
    }

    SMLK_LOGD("init obd-diag service");
    auto obd_ret = OBD::DidIpcApi::getInstance()->Init(ModuleID::E_Module_gb6_service);
    if ( SMLK_RC::RC_OK != obd_ret ) {
        SMLK_LOGF("fail to init diag service API interface, err=%d.\n",obd_ret);
        return SL_EFAILED;
    }

    SMLK_LOGD("init location service");
    if ( !smartlink_sdk::Location::GetInstance()->Init(ModuleID::E_Module_gb6_service) ) {
        SMLK_LOGE("fail to init location service API interface!!!");
        return SL_EFAILED;
    }
    std::vector<smartlink_sdk::LocationEventId> loc_events = {
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_ISREADY,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_EXIT,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED,
    };
    SMLK_LOGD("register location event callback");
    auto loc_ret = smartlink_sdk::Location::GetInstance()->RegEventCallback(
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
    if ( smartlink_sdk::RtnCode::E_SUCCESS != loc_ret ) {
        SMLK_LOGE("fail to register event callback to location service, return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    SMLK_LOGD("init sys-power service");
    if (!smartlink_sdk::SysPower::GetInstance()->Init(ModuleID::E_Module_gb6_service))
    {
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
                    static SMLK_UINT8 dismantle_main_power_flag = 0;

                    if (sizeof(smartlink_sdk::SysPowerSource) != (std::size_t)len) {
                        SMLK_LOGE("[POWER] unexpected body length for SYS POWER source CHANGED: %d, expected: %d", len, (SMLK_UINT8)sizeof(smartlink_sdk::SysPowerSource));
                        return;
                    }
                    auto inf = reinterpret_cast<const smartlink_sdk::SysPowerSource *>(data);

                    if ((SMLK_UINT8)smartlink_sdk::SysPowerSource::E_SYS_POWER_SOURCE_MAIN == (SMLK_UINT8)*inf) {
                        if (0 == dismantle_main_power_flag) {
                            // SMLK_LOGD("[POWER] alerady change to main power!!!");
                            return;
                        } else {
                            dismantle_main_power_flag = 0;
                            SMLK_LOGD("[POWER] system power source change to main power!!!");
                        }
                    } else if ((SMLK_UINT8)smartlink_sdk::SysPowerSource::E_SYS_POWER_SOURCE_STANDBY == (SMLK_UINT8)*inf) {
                        if (1 == dismantle_main_power_flag) {
                            // SMLK_LOGD("[POWER] alerady change to backup power!!!");
                            return;
                        } else {
                            dismantle_main_power_flag = 1;
                            SMLK_LOGD("[POWER] system power source change to standy power!!!");
                        }
                    } else {
                        SMLK_LOGD("[POWER] system power source change to invalid data%d!!!", (SMLK_UINT8)*inf);
                    }
                }
                break;
                case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP: {
                    SMLK_LOGD("[POWER] system power event wakeup received!!!");

                    SMLK_LOGD("[timesyncDebug] m_timeSyncGap_ms is %lld", m_timeSyncGap_ms);
                    SMLK_LOGD("[timesyncDebug] RTC wake up reset m_timeSyncGap_ms to 0");
                    m_timeSyncGap_ms = 0;
                    m_systime_flags = SysTime::SL_SYSTIME_SYN_DEF;
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

    SMLK_LOGD("init mcu service");
    auto mcu_ret = smartlink_sdk::MCU::GetInstance()->Init(ModuleID::E_Module_gb6_service);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != mcu_ret ) {
        SMLK_LOGE("fail to init MCU service, return code: %d", static_cast<std::int32_t>(mcu_ret));
        return SL_EFAILED;
    }
    std::vector<smartlink_sdk::McuEventId> mcu_events = {
        smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE,
        smartlink_sdk::McuEventId::E_MCU_EVENT_UDS_RESPONSE,
    };
    SMLK_LOGD("register mcu event callback");
    mcu_ret = smartlink_sdk::MCU::GetInstance()->RegEventCB(
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
                case smartlink_sdk::McuEventId::E_MCU_EVENT_UDS_RESPONSE:
                    {
                        SMLK_LOGD("case smartlink_sdk::McuEventId::E_MCU_EVENT_UDS_RESPONSE ");

                        SMLK_UINT16     result = 0;
                        auto rsp = reinterpret_cast<const smartlink_sdk::UdsRspInfo *>(data);
                        for ( auto const &item : rsp->list ) {
                            for ( auto const &node : item.list ) {
                                /* here we reuse the following fields
                                 * head.seq         --> UDS service id
                                 * head.m_protocol  --> the high byte of UDS reponse error code
                                 * head.m_compress  --> the low byte of UDS response error code
                                 * head.m_length    --> the result code of UDS response
                                 * head.m_extra     --> the extended id of response
                                */
                                if ( 0 != node.result ) {
                                    result = 1;
                                }
                                SMLK_LOGD("receive uds  svc_id 0x%x: result 0x%x",item.svc_id,node.result);
                                Post(MsgHead(SL_EVENT_UDS_RESPONSE, item.svc_id, (SMLK_UINT8)(node.err >> 8), (SMLK_UINT8)(node.err & 0xFF), node.len, node.result), node.data, node.len);
                            }
                        }
                    }
                    break;
                default:
                    SMLK_LOGE("[MCU] unsupported event id: %u", static_cast<SMLK_UINT32>(id));
                    return;
            }
        }
    );
    if ( smartlink_sdk::RtnCode::E_SUCCESS != mcu_ret ) {
        SMLK_LOGE("fail to register event callback to MCU service, return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }


    SMLK_LOGD("init telephony service");
    if ( !smartlink_sdk::Telephony::GetInstance()->Init(ModuleID::E_Module_gb6_service) ) {
        SMLK_LOGE("fail to init telephony service");
        return SL_EFAILED;
    }
    std::vector<smartlink_sdk::TelEventId> tel_events = {
        smartlink_sdk::TelEventId::E_TEL_EVENT_DATA_CHANNEL_CHANGED,
    };
    SMLK_LOGD("register telephony event callback");
    auto tel_ret = smartlink_sdk::Telephony::GetInstance()->RegEventCB(
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

    tel_ret = smartlink_sdk::Telephony::GetInstance()->GetICCID(m_iccid);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != tel_ret ) {
        SMLK_LOGE("fail to get ICCID from telephony module! return code: %d", static_cast<std::int32_t>(tel_ret));
        return SL_EFAILED;
    } else {
        SMLK_LOGD("ICCID:   %s", m_iccid.c_str());
    }

    tel_ret = smartlink_sdk::Telephony::GetInstance()->GetIMSI(m_imsi);
    if (smartlink_sdk::RtnCode::E_SUCCESS != tel_ret) {
        SMLK_LOGE("fail to get IMSI from telephony module! return code: %d", static_cast<std::int32_t>(tel_ret));
        m_imsi = "0000000000000";
        return SL_EFAILED;
    } else {
        SMLK_LOGD("IMSI:   %s", m_imsi.c_str());
        if (m_imsi.length() >= 13) {
            m_imsi = m_imsi.substr(m_imsi.length() - 13, 13);
        }
        SMLK_LOGD("IMSI_tmp:   %s", m_imsi.c_str());
    }

    std::string m_tmp_chipid = m_product_loc == M_GB_LOCATION_CHANGCHUN ? "590" + m_imsi : "YRW" + m_imsi;
    SMLK_LOGD("tmp_chipid:   %s", m_tmp_chipid.c_str());

    if (is_setloc) {
        property = SYS_PRO_NAME_ENC_CHIP_ID_SET;
        return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, m_chipid);
        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
            SMLK_LOGE("fail to get chip id \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
            return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, m_tmp_chipid);
            if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
                SMLK_LOGE("fail to set system property \"%s\", return code: %d", property, static_cast<std::int32_t>(return_code));
            }
            m_chipid = m_tmp_chipid;
        } else {
            SMLK_LOGI("m_chipid:            %s", m_chipid.c_str());
        }
    }

    // std::string m_tmp_chipid = "HNW";
    // std::string m_tmp_chipid = m_product_loc == M_GB_LOCATION_CHANGCHUN ? "590" + m_imsi : "YRW" + m_imsi;
    // SMLK_LOGD("tmp_chipid:   %s", m_tmp_chipid.c_str());

    // property = SYS_PRO_NAME_ENC_CHIP_ID_SET;
    // return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, m_tmp_chipid);
    // if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
    //     SMLK_LOGE("fail to set system property \"%s\", return code: %d", property, static_cast<std::int32_t>(return_code));
    // }
    // m_chipid = m_tmp_chipid;
    // SMLK_LOGI("m_chipid:            %s", m_chipid.c_str());

    int ret = 0;
    int n_sm2_pub_len = 64;
    unsigned char           m_sm2_pub_key[64];
    ret = hsm_seal_gen_asymm_keypair(TYPE_SM2, 2048, 0, 0);
    ret = hsm_seal_get_pubkey(TYPE_SM2, m_sm2_pub_key, (size_t *)&n_sm2_pub_len, 0);
    m_pub_key.insert(m_pub_key.cend(), m_sm2_pub_key, m_sm2_pub_key + 64);

    m_timer_1s = std::make_shared<LoopTimer<std::chrono::milliseconds, 1000>>(
        [this](SMLK_UINT32 index)
        {
            Post(MsgHead(SL_EVENT_TIMER_1S_NOTIFY, 0, 0, 0, 0, 0), nullptr, 0);
        });

    m_protocol_reporter.clear();

    m_protocol_reporter_1239_07 = std::make_shared<Reporter1239_07>(act_host, act_port, ProtocolVer::HJ, 0);
    m_protocol_reporter_1239_07->SetIsSave(m_todisk);    
    m_protocol_reporter_1239_07->SetPubKey(m_pub_key);
    m_protocol_reporter_1239_07->SetChipID(m_chipid);
    // m_protocol_reporter_1239_07->SetVIN(m_vin);

    m_protocol_reporter_1239_07->Init();

    m_service_flags |= IService::Initialized;

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
    if ( m_service_flags & IService::Running ) {
        SMLK_LOGI("service already started, do nothing");
        return SL_SUCCESS;
    }

    if ( !(m_service_flags & IService::Initialized) ) {
        SMLK_LOGE("service not initialized");
        return SL_EFAILED;
    }

    m_timer_1s->Start();
    m_timer_1s->Reset();

    m_protocol_reporter_1239_07->Start();

    m_service_flags |= IService::Running;

    m_thread = std::thread(
        [this]() {
            SMLK_LOGI("main work thread started");
            while ( !(m_service_flags & IService::Stopping) ) {
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
    if ( m_service_flags & IService::Running ) {
        SMLK_LOGD("told to stop, mark as stopping");
        m_service_flags |= IService::Stopping;

        if ( m_thread.joinable() ) {
            m_thread.join();
        }

        if ( m_timer_1s ) {
            m_timer_1s->Reset();
        }

        if ( m_timer_obd ) {
            m_timer_obd->Reset();
        }

        if ( m_timer_daychange ) {
            m_timer_daychange->Reset();
        }

        SMLK_LOGD("stop reporter one by one");
        for ( auto &pair : m_protocol_reporter ) {
            pair.second->Stop();
            SMLK_LOGD("one reporter stopped!");
        }

        SMLK_LOGD("all reporter stopped");

        m_service_flags |= ~(IService::Running | IService::Stopping);
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
    return  m_service_flags & IService::Running;
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
        case SL_EVENT_808_MSG_NOTIFY:
            OnTspNotify(head, data);
            break;
        case SL_EVENT_SHARED_TIMER:
            OnTimer(head.m_seq, head.m_extra);
            break;
        case SL_EVENT_TIMER_1S_NOTIFY:
            OnTimer1S();
            break;
        case SL_EVENT_UDS_RESPONSE:
            OnUDS(head, data);
            break;
        default:
            SMLK_LOGE("unsupported event id:    0x%08X", head.m_id);
            break;
    }
}


/**
 * @brief location information notification handler
 * 
 * @param head message head which has the basic information about this message
 * @param data message data
 */
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


/**
 * @brief location information notification handler
 * 
 * @param head message head which has the basic information about this message
 * @param data message data
 */
void ReportService::OnMcuNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch (head.m_seq) {
        case MCU::SL_IGN_NOTIFY:
        {
            if (IGState::SL_IGN_ON == head.m_extra) {
                if (m_ign_flags == IGState::SL_IGN_OFF) {
                    m_ign_flags = IGState::SL_IGN_ON;
                    SMLK_LOGD("IGN state changed from OFF to ON");

                    m_timer_obd = std::make_shared<Timer<std::chrono::milliseconds>>(
                        SL_TIMER_ID_OBD_QUERY, m_obd_query_delay,
                        [this](std::uint32_t id){
                            Post(MsgHead(SL_EVENT_SHARED_TIMER, id, 0, 0, 0, 0), nullptr, 0);
                        }
                    );
                    m_timer_obd->Start();

                    InitTSPConfig();

                    for (auto &pair : m_protocol_reporter) {
                        auto rc = pair.second->Start();
                        if (SL_SUCCESS != rc) {
                            SMLK_LOGE("fail to start protocol repoter, pair.first = 0x%02X, rc = 0x%08X", pair.first, rc);
                            return;
                        }
                    }
          
                    if (smartlink_sdk::SysTime::GetInstance()->IsReady()) {
                        ReadTimeSource();
                    } else {
                        SMLK_LOGE("[timesyncDebug] SysTime not ready");
                    }

                    for (auto index : indexs) {
                        VehicleData data(index, 0.0);
                        auto rc = VehicleDataApi::GetInstance()->GetVehicleData(data);
                        if (SMLK_RC::RC_OK != rc) {
                            SMLK_LOGD("fail to get vehicle data for index:  %u", index);
                            continue;
                        }
                        for (auto &pair : m_protocol_reporter) {
                            pair.second->UpdateSignalVal(data.index, data.value, data.valid);
                        }
                    }

                    if (smartlink_sdk::Location::GetInstance()->IsReady()) {
                        GetGNSS();
                    }

                    for (auto &pair : m_protocol_reporter) {
                        if (pair.second->IsRunning()) {
                            pair.second->IgnON();
                        }
                    }
                }
            } else {
                if (m_ign_flags == IGState::SL_IGN_ON) {
                    m_ign_flags = IGState::SL_IGN_OFF;
                    SMLK_LOGD("IGN state changed from ON to OFF");
                    for (auto &pair : m_protocol_reporter) {
                        if (pair.second->IsRunning()) {
                            pair.second->IgnOFF();
                        }
                    }
                    m_protocol_reporter.clear();
                }
            }
        }
        break;
        default:
            SMLK_LOGE("[MCU] unsupported MCU event notify: 0x%08x", head.m_seq);
            break;
        }
}


/**
 * @brief Tsp information notification handler
 * 
 * @param head message head which has the basic information about this message
 * @param data message data
 */
void ReportService::OnTspNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
#ifdef _HJ_DEV

    switch (head.m_seq)
    {
    case TSP::SL_ACTIVATE_NOTIFY:
    {
        if (m_protocol_reporter_1239_07->IsRunning())
        {
            m_protocol_reporter_1239_07->VehicleActivate(M_HJ_1239_ACTIVATED_ING);
        }
    }
    break;
    default:
        SMLK_LOGE("[TSP] unsupported TSP event notify: 0x%08x", head.m_seq);
        break;
    }
#endif
}

/**
 * @brief UDS response event handler
 * 
 * @param head message head which has the basic information about this message
 * @param data message data
 */
void ReportService::OnUDS(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnUDS enter");

    /* here we reuse the following fields
        * head.seq         --> UDS service id
        * head.m_protocol  --> the high byte of UDS reponse error code
        * head.m_compress  --> the low byte of UDS response error code
        * head.m_length    --> the result code of UDS response
        * head.m_extra     --> the extended id of response
    */
    std::stringstream   ss;
    for ( auto ch : data ) {
        ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
    }

    SMLK_LOGI("head.m_seq(0x%x) head.m_extra(0x%x) data(%d): %s",head.m_seq, head.m_extra, head.m_length , ss.str().c_str());

    smartlink_sdk::RtnCode return_code;

    switch ( head.m_seq ) {

        case obd::sid::ReadDataByIdentifierRsp:
        {
            if (0 == head.m_length) {
                SMLK_LOGE("fail data len %d, error code: 0x%04X", head.m_length, ((SMLK_UINT16)head.m_protocol << 8 | (SMLK_UINT16)head.m_compress));
                return;
            }

            uint16_t did;
            did = data[0] << 8 | data[1];
            SMLK_LOGD("did: %02x", did);

            switch (htobe16(did))
            {
            case obd::did::CVN:
                for (auto &pair : m_protocol_reporter) {
                    pair.second->UpdateCVN(head.m_extra, data);
                }

                break;
            case obd::did::VIN:
                if (m_ign_flags == IGState::SL_IGN_ON) {

                    OBD::UdsEventId evt_id = OBD::UdsEventId::E_UDS_DTC_TRIGGER;
                    std::vector<SMLK_UINT8> data_to_obd;
                    data_to_obd.clear();

                    OBD::UdsDtcTriggerInfo m_data_to_obd;
                    m_data_to_obd.dtc_type = OBD::UdsDtcTriggerType::E_UDS_DTC_TRIGGER_CHECK_VIN_STATUS;

                    SMLK_UINT8 m_status_daq = GB_READ_VIN_DEFAULT;
                    SMLK_UINT8 m_status_obd = GB_READ_VIN_DEFAULT;

                    if (17 > data.size() - 2) {
                        SMLK_LOGE("invalid vin body size: %u", data.size() - 2);

                        m_status_daq = GB_READ_VIN_FAIL;
                        m_status_obd = GB_READ_VIN_FAIL;

                        return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_GB_CHECK_VIN_FOR_DAQ, std::to_string(m_status_daq));
                        SMLK_LOGI("Set SYS_PRO_NAME_GB_CHECK_VIN_FOR_DAQ %d ", m_status_daq);
                        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
                            SMLK_LOGE("fail to set system property \"%s\", return code: %d", SYS_PRO_NAME_GB_CHECK_VIN_FOR_DAQ, static_cast<std::int32_t>(return_code));
                        }

                        return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_GB_CHECK_VIN, std::to_string(m_status_obd));
                        SMLK_LOGI("Set SYS_PRO_NAME_GB_CHECK_VIN %d ", m_status_obd);
                        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
                            SMLK_LOGE("fail to set system property \"%s\", return code: %d", SYS_PRO_NAME_GB_CHECK_VIN, static_cast<std::int32_t>(return_code));
                        }

                        m_data_to_obd.val = m_status_obd;
                        if (!(OBD::DidIpcApi::getInstance()->SendDtcErrCode(m_data_to_obd))) {
                            SMLK_LOGW("SyncDTC ipc failed!!!");
                        }

                        return;
                    } else {
                        SMLK_LOGD("vin data length is %zu data[2] 0x%x data[3] 0x%x", data.size(), data[2], data[3]);
                        if ((data[2] == 0x4c || data[2] == 0x6c) && (data[3] == 0x46 || data[3] == 0x66)) {
                            m_status_daq = GB_CHECK_VIN_VALID;
                        } else {
                            m_status_daq = GB_CHECK_VIN_INVALID;
                        }

                        std::stringstream stream;
                        stream << data.data();
                        std::string m_obd_vin = stream.str().substr(2, 17);
                        SMLK_LOGD("get m_obd_vin is [%s]", m_obd_vin.c_str());

                        return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_OBD_VIN, m_obd_vin.c_str());
                        SMLK_LOGI("Set SYS_PRO_NAME_OBD_VIN %s ", m_obd_vin.c_str());
                        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
                            SMLK_LOGE("fail to set system property \"%s\", return code: %d", SYS_PRO_NAME_OBD_VIN, static_cast<std::int32_t>(return_code));
                        }


                        if (m_obd_vin.empty()) {
                            m_status_daq = GB_READ_VIN_FAIL;
                            m_status_obd = GB_READ_VIN_FAIL;
                        } else {
                            if (m_vin == m_obd_vin) {
                                m_status_obd = GB_COMP_VIN_SAME;
                            } else {
                                m_status_obd = GB_COMP_VIN_DIFF;
                            }

                            for (auto &pair : m_protocol_reporter) {
                                pair.second->UpdateVIN(head.m_extra, data);
                            }

                            // m_protocol_reporter_1239_07->UpdateVIN(head.m_extra, data);

                        }
                        return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_GB_CHECK_VIN_FOR_DAQ, std::to_string(m_status_daq));
                        SMLK_LOGI("Set SYS_PRO_NAME_GB_CHECK_VIN_FOR_DAQ %d ", m_status_daq);
                        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
                            SMLK_LOGE("fail to set system property \"%s\", return code: %d", SYS_PRO_NAME_GB_CHECK_VIN_FOR_DAQ, static_cast<std::int32_t>(return_code));
                        }

                        return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_GB_CHECK_VIN, std::to_string(m_status_obd));
                        SMLK_LOGI("Set SYS_PRO_NAME_GB_CHECK_VIN %d ", m_status_obd);
                        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
                            SMLK_LOGE("fail to set system property \"%s\", return code: %d", SYS_PRO_NAME_GB_CHECK_VIN, static_cast<std::int32_t>(return_code));
                        }

                        m_data_to_obd.val = m_status_obd;
                        if (!(OBD::DidIpcApi::getInstance()->SendDtcErrCode(m_data_to_obd))) {
                            SMLK_LOGW("SyncDTC ipc failed!!!");
                        }
                    }
                }
                break;
            case obd::did::MIL:
            {
                SMLK_LOGE("obd::did::gas::MIL enter");
                if (data.size() - 2 < sizeof(obd::message::diagnostic)) {
                    SMLK_LOGE("invalid body size: %u", data.size() - 2);
                    for (auto &pair : m_protocol_reporter) {
                        pair.second->UpdateOBD(head.m_extra, data);
                    }
                    return;
                }

                auto diag = reinterpret_cast<const obd::message::diagnostic *>(data.data() + 2);

                SMLK_UINT8 lamp_status = diag->A.malfunction_indicator_lamp_status;
                SMLK_LOGI(" lamp_status = diag->A.malfunction_indicator_lamp_status = %u ", lamp_status);

                SMLK_UINT16 support_status = 0;

                // bit 15
                support_status <<= 1;
                support_status |= diag->B.comprehensive_component_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->B.comprehensive_component_monitoring_supported = %u ", diag->B.comprehensive_component_monitoring_supported);
#endif
                // bit 14
                support_status <<= 1;
                support_status |= diag->B.fuel_system_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->B.fuel_system_monitoring_supported = %u ", diag->B.fuel_system_monitoring_supported);
#endif
                // bit 13
                support_status <<= 1;
                support_status |= diag->B.misfire_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->B.misfire_monitoring_supported = %u ", diag->B.misfire_monitoring_supported);
#endif
                // bit 12
                support_status <<= 1;
                support_status |= diag->C.catalyst_monitoring_nmhc_catalyst_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.catalyst_monitoring_nmhc_catalyst_monitoring_supported = %u ", diag->C.catalyst_monitoring_nmhc_catalyst_monitoring_supported);
#endif
                // bit 11
                support_status <<= 1;
                support_status |= diag->C.heated_catalyst_monitoring_nox_aftertreatment_monitorin_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.heated_catalyst_monitoring_nox_aftertreatment_monitorin_supported = %u ", diag->C.heated_catalyst_monitoring_nox_aftertreatment_monitorin_supported);
#endif
                // bit 10
                support_status <<= 1;
                support_status |= diag->C.oxygen_sensor_heater_monitoring_pm_filter_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.oxygen_sensor_heater_monitoring_pm_filter_monitoring_supported = %u ", diag->C.oxygen_sensor_heater_monitoring_pm_filter_monitoring_supported);
#endif
                // bit 9
                support_status <<= 1;
                support_status |= diag->C.secondary_air_system_monitoring_boost_pressure_system_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.secondary_air_system_monitoring_boost_pressure_system_monitoring_supported = %u ", diag->C.secondary_air_system_monitoring_boost_pressure_system_monitoring_supported);
#endif
                // bit 8
                support_status <<= 1;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] support_status bit 8 = 0x%08X ", support_status);
#endif
                // bit 7
                support_status <<= 1;
                support_status |= diag->C.egr_vvt_system_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.egr_vvt_system_monitoring_supported = %u ", diag->C.egr_vvt_system_monitoring_supported);
#endif
                // bit 6
                support_status <<= 1;
                support_status |= diag->C.oxygen_sensor_heater_monitoring_pm_filter_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.oxygen_sensor_heater_monitoring_pm_filter_monitoring_supported = %u ", diag->C.oxygen_sensor_heater_monitoring_pm_filter_monitoring_supported);
#endif
                // bit 5
                support_status <<= 1;
                support_status |= diag->C.exhaust_gas_sensor_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.exhaust_gas_sensor_monitoring_supported = %u ", diag->C.exhaust_gas_sensor_monitoring_supported);
#endif
                // bit 4
                support_status <<= 1;
#ifdef _T_DEBUG
#endif
                // bit 3
                support_status <<= 1;
                support_status |= diag->C.secondary_air_system_monitoring_boost_pressure_system_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.secondary_air_system_monitoring_boost_pressure_system_monitoring_supported = %u ", diag->C.secondary_air_system_monitoring_boost_pressure_system_monitoring_supported);
#endif
                // bit 2
                support_status <<= 1;
                support_status |= diag->C.evaporative_system_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.evaporative_system_monitoring_supported = %u ", diag->C.evaporative_system_monitoring_supported);
#endif
                // bit 1
                support_status <<= 1;
                support_status |= diag->C.heated_catalyst_monitoring_nox_aftertreatment_monitorin_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.heated_catalyst_monitoring_nox_aftertreatment_monitorin_supported = %u ", diag->C.heated_catalyst_monitoring_nox_aftertreatment_monitorin_supported);
#endif
                // bit 0
                support_status <<= 1;
                support_status |= diag->C.catalyst_monitoring_nmhc_catalyst_monitoring_supported & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->C.catalyst_monitoring_nmhc_catalyst_monitoring_supported = %u ", diag->C.catalyst_monitoring_nmhc_catalyst_monitoring_supported);
#endif
                SMLK_LOGD("support_status end = 0x%08X ", support_status);

                SMLK_UINT16 ready_status = 0;

                // bit 15
                ready_status <<= 1;
                ready_status |= diag->B.comprehensive_component_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->B.comprehensive_component_monitoring_ready = %u ", diag->B.comprehensive_component_monitoring_ready);
#endif
                // bit 14
                ready_status <<= 1;
                ready_status |= diag->B.fuel_system_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->B.fuel_system_monitoring_ready = %u ", diag->B.fuel_system_monitoring_ready);
#endif
                // bit 13
                ready_status <<= 1;
                ready_status |= diag->B.misfire_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->B.misfire_monitoring_ready = %u ", diag->B.misfire_monitoring_ready);
#endif
                // bit 12
                ready_status <<= 1;
                ready_status |= diag->D.catalyst_monitoring_nmhc_catalyst_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.catalyst_monitoring_nmhc_catalyst_monitoring_ready = %u ", diag->D.catalyst_monitoring_nmhc_catalyst_monitoring_ready);
#endif
                // bit 11
                ready_status <<= 1;
                ready_status |= diag->D.heated_catalyst_monitoring_nox_aftertreatment_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.heated_catalyst_monitoring_nox_aftertreatment_monitoring_ready = %u ", diag->D.heated_catalyst_monitoring_nox_aftertreatment_monitoring_ready);
#endif
                // bit 10
                ready_status <<= 1;
                ready_status |= diag->D.oxygen_sensor_heater_monitoring_pm_filter_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.oxygen_sensor_heater_monitoring_pm_filter_monitoring_ready = %u ", diag->D.oxygen_sensor_heater_monitoring_pm_filter_monitoring_ready);
#endif
                // bit 9
                ready_status <<= 1;
                ready_status |= diag->D.secondary_air_system_monitoring_boost_pressure_system_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.secondary_air_system_monitoring_boost_pressure_system_monitoring_ready = %u ", diag->D.secondary_air_system_monitoring_boost_pressure_system_monitoring_ready);
#endif
                // bit 8
                ready_status <<= 1;
#ifdef _T_DEBUG
#endif
                // bit 7
                ready_status <<= 1;
                ready_status |= diag->D.egr_vvt_system_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.egr_vvt_system_monitoring_ready = %u ", diag->D.egr_vvt_system_monitoring_ready);
#endif
                // bit 6
                ready_status <<= 1;
                ready_status |= diag->D.oxygen_sensor_heater_monitoring_pm_filter_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.oxygen_sensor_heater_monitoring_pm_filter_monitoring_ready = %u ", diag->D.oxygen_sensor_heater_monitoring_pm_filter_monitoring_ready);
#endif
                // bit 5
                ready_status <<= 1;
                ready_status |= diag->D.exhaust_gas_sensor_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.exhaust_gas_sensor_monitoring_ready = %u ", diag->D.exhaust_gas_sensor_monitoring_ready);
#endif
                // bit 4
                ready_status <<= 1;
#ifdef _T_DEBUG
#endif
                // bit 3
                ready_status <<= 1;
                ready_status |= diag->D.secondary_air_system_monitoring_boost_pressure_system_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.secondary_air_system_monitoring_boost_pressure_system_monitoring_ready = %u ", diag->D.secondary_air_system_monitoring_boost_pressure_system_monitoring_ready);
#endif
                // bit 2
                ready_status <<= 1;
                ready_status |= diag->D.evaporative_system_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.evaporative_system_monitoring_ready = %u ", diag->D.evaporative_system_monitoring_ready);
#endif
                // bit 1
                ready_status <<= 1;
                ready_status |= diag->D.heated_catalyst_monitoring_nox_aftertreatment_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.heated_catalyst_monitoring_nox_aftertreatment_monitoring_ready = %u ", diag->D.heated_catalyst_monitoring_nox_aftertreatment_monitoring_ready);
#endif
                // bit 0
                ready_status <<= 1;
                ready_status |= diag->D.catalyst_monitoring_nmhc_catalyst_monitoring_ready & 0x01;
#ifdef _T_DEBUG
                SMLK_LOGD("[T] diag->D.catalyst_monitoring_nmhc_catalyst_monitoring_ready = %u ", diag->D.catalyst_monitoring_nmhc_catalyst_monitoring_ready);
#endif

                SMLK_LOGD("ready_status end = 0x%08X ", ready_status);

                std::vector<SMLK_UINT8> buffer;
                buffer.push_back(lamp_status);
                buffer.push_back(support_status >> 8);
                buffer.push_back(support_status & 0xFF);
                buffer.push_back(ready_status >> 8);
                buffer.push_back(ready_status & 0xFF);

                for (auto &pair : m_protocol_reporter) {
                    pair.second->UpdateOBD(head.m_extra, buffer);
                }
            }
            break;
            case obd::did::IUPR_G:
                SMLK_LOGE("iupr data length is %d ", data.size());
                for (auto &pair : m_protocol_reporter) {
                    pair.second->UpdateIUPR(head.m_extra, data);
                }
                break;
            case obd::did::IUPR_D:
                SMLK_LOGE("iupr data length is %d ", data.size());
                for (auto &pair : m_protocol_reporter) {
                    pair.second->UpdateIUPR(head.m_extra, data);
                }
                break;
            case obd::did::SCIN:
                for (auto &pair : m_protocol_reporter) {
                    pair.second->UpdateSCIN(head.m_extra, data);
                }
                break;
            case obd::did::Protocol:
                if (0 == data.size()) {
                    SMLK_LOGE("diagnostic protocol data length is 0");
                    return;
                }
                for (auto &pair : m_protocol_reporter) {
                    pair.second->UpdateDiagProtocol(head.m_extra, data);
                }
                break;
            default:
                SMLK_LOGE("unsupported did: 0x%08X", did);
                break;
            }
        }
        break;
        case obd::sid::ReadDTCInformationRsp: {
            if (0 == head.m_length) {
                SMLK_LOGE("fail to read DTC, error code: 0x%04X", ((SMLK_UINT16)head.m_protocol << 8 | (SMLK_UINT16)head.m_compress));
                for (auto &pair : m_protocol_reporter) {
                    pair.second->UpdateDTC(head.m_extra, data);
                }
                return;
            }

            SMLK_UINT8 DTCHeader[] = {
                obd::sfid::ReportWWHOBDDTCByMaskRecord,         // sub-function
                obd::dtc::FGID::emissions,                      // functional group id
                obd::dtc::status::PDTC | obd::dtc::status::CDTC // dtc status pending and confirmed
            };

            if ((DTCHeader[0] == data[0]) && (DTCHeader[1] == data[1])) {
                SMLK_LOGI("DTC sub servcie.");
                for (auto &pair : m_protocol_reporter) {
                    if (pair.second->IsRunning()) {
                        pair.second->UpdateDTC(head.m_extra, data);
                    }
                }
            } else {
                SMLK_LOGE("unsupported sub service");
            }
        }
        break;
        default:
            SMLK_LOGE("unsupported UDS service id:  0x%08X", head.m_seq);
            break;
    }
}

void ReportService::ReadTimeSource()
{
    SMLK_LOGI("[timesyncDebug] on ReadTimeSource");
    smartlink_sdk::TimeInfo current_info;
    memset(&current_info, 0, sizeof(current_info));
    auto result = smartlink_sdk::SysTime::GetInstance()->GetClockInfo(current_info);
    if (smartlink_sdk::RtnCode::E_SUCCESS != result) {
        SMLK_LOGE("[timesyncDebug] GetClockInfo fail");
    } else {
        bool timeSourceSync = current_info.timesync;
        SMLK_UINT32 currentTimeSource = (SMLK_UINT32)current_info.source;
        SMLK_LOGI("[timesyncDebug]***** timesync = [%d]", current_info.timesync);
        SMLK_LOGI("[timesyncDebug] TimeSource = [%d], 1:GNSS 2:NTP 3:NITZ 4:TSP 5:RTC", current_info.source);
        SMLK_LOGI("[timesyncDebug] current utc s is = [%llu] ", current_info.timestamp / 1000000);
        if (timeSourceSync) {
            SMLK_LOGI("[timesyncDebug] timesync when read");
            m_systime_flags = SysTime::SL_SYSTIME_SYN_OK;
        } else {
            if (smartlink_sdk::TimeSource::E_TIME_SOURCE_NTP == currentTimeSource || smartlink_sdk::TimeSource::E_TIME_SOURCE_GNSS == currentTimeSource) {
                SMLK_LOGI("[timesyncDebug] read useful time source");
                m_systime_flags = SysTime::SL_SYSTIME_SYN_OK;
            }
        }
    }
    SMLK_LOGI("[timesyncDebug] m_timeSyncGap_ms is %lld", m_timeSyncGap_ms);
}


/**
 * @brief get GNSS information
 * 
 */
void ReportService::GetGNSS()
{
    smartlink_sdk::LocationInfo location;
    auto rc = smartlink_sdk::Location::GetInstance()->GetLocation(location);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != rc ) {
        SMLK_LOGE("[gnss] fail to get location information, rc: %d", static_cast<std::int32_t>(rc));
    } else {

#ifdef SL_DEBUG
        SMLK_LOGD("[gnss]   location.speed:     %f", location.speed);
        SMLK_LOGD("[gnss]   location.heading:   %f", location.heading);
        SMLK_LOGD("[gnss]   location.altitude:  %f", location.altitude);
        SMLK_LOGD("[gnss]   location.latitude:  %f", location.latitude);
        SMLK_LOGD("[gnss]   location.longitude: %f", location.longitude);
        SMLK_LOGD("[gnss]   location.fix_valid: %s", location.fix_valid ? "YES" : "NO");
        SMLK_LOGD("[gnss]   location.utc:       %u", location.utc);
#endif

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


/**
 * @brief 
 * 
 */
void ReportService::OnTimer1S()
{
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    struct tm tm;
    std::time_t tt = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    localtime_r(&tt, &tm);

    if (tm.tm_hour == 23 && tm.tm_min >= 58 && !m_timer_daychange)
    {
        decltype(ms) ms_offset = 60 - tm.tm_min;
        ms_offset *= 60;
        ms_offset -= tm.tm_sec;
        ms_offset *= 1000;
        ms_offset -= ms - 1;

        m_timer_daychange = std::make_shared<Timer<std::chrono::milliseconds>>(
            SL_TIMER_ID_DAYCHANGE,
            (int)ms_offset,
            [this](IN std::uint32_t id)
            {
                Post(MsgHead(SL_EVENT_SHARED_TIMER, id, 0, 0, 0, 0), nullptr, 0);

            });

        m_timer_daychange->Start();
    }
}


/**
 * @brief shared timer event handler
 * 
 * @param id timer id
 * @param seq loop timer trigger sequence
 */
void ReportService::OnTimer(IN SMLK_UINT32 id, IN SMLK_UINT32 seq)
{
    switch ( id ) {
        case SL_TIMER_ID_OBD_QUERY: {
                SMLK_LOGD("try to query OBD information!!!");
                QueryOBD();
            }
            break;
        case SL_TIMER_ID_DAYCHANGE: {
                SMLK_LOGD("========== day change!!!");
                m_timer_daychange->Reset();
                for (auto &pair : m_protocol_reporter) {
                    if (pair.second->IsRunning()) {
                        pair.second->UpdateDayChange();
                    }
                }
            }
            break;
        default:
            SMLK_LOGE("unknown timer id: 0x%08X", id);
            break;
    }
}

/******************************************************************************
 * NAME: QueryOBD
 *
 * DESCRIPTION: query OBD information
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void ReportService::QueryOBD()
{
    SMLK_LOGI("QueryOBD enter");

    SMLK_UINT8 data[] = {
        obd::sfid::ReportWWHOBDDTCByMaskRecord,                 // sub-function
        obd::dtc::FGID::emissions,                              // functional group id
        obd::dtc::status::PDTC | obd::dtc::status::CDTC         // dtc status pending and confirmed
    };

    std::vector<smartlink_sdk::UdsReqItem>  requests = {};
    requests.push_back({});
    smartlink_sdk::UdsReqItem   req;
    req.property    = UDS::Config::Priority::Highest;
    req.svc_id      = obd::sid::ReadDTCInformation;

    //req.list.push_back(smartlink_sdk::UdsReqNode{0, sizeof(data), data});
    req.list.push_back(smartlink_sdk::UdsReqNode{ sizeof(data), data});
    if ( EMSModel::SL_BOSCH_GB6_GAS == m_ems_model ||
         EMSModel::SL_ECONTROLS_GB6_GAS == m_ems_model  ||
         EMSModel::SL_FAW_CUMMINS_GB6_GASL == m_ems_model ||
         EMSModel::SL_WEICHAI_GB6_GAS_OH6 == m_ems_model ||
         EMSModel::SL_YUCHAI_GB6_GAS == m_ems_model  ) {
        requests.begin()->req_id    = UDS::Config::did_gas.begin()->req_id;
        requests.begin()->rsp_id    = UDS::Config::did_gas.begin()->rsp_id;
        requests.begin()->svc_id    = UDS::Config::did_gas.begin()->svc_id;
        requests.begin()->property  = UDS::Config::did_gas.begin()->property;
        for ( auto const &cfg : UDS::Config::did_gas ) {
            requests.begin()->list.push_back(smartlink_sdk::UdsReqNode{ 2,(SMLK_UINT8 *) &cfg.extend_id});
        }
        req.req_id      = obd::can::identifier::gas::Request;
        req.rsp_id      = obd::can::identifier::gas::Response;
    } else if( EMSModel::SL_FAW_GB6_DIESEL == m_ems_model ||
        EMSModel::SL_FAW_CUMMINS_GB6_DIESEL == m_ems_model ||
        EMSModel::SL_WEICHAI_GB6_DIESEL == m_ems_model ||
        EMSModel::SL_YUCHAI_GB6_DIESEL == m_ems_model ||
        EMSModel::SL_QUANCHAI_GB6_DIESEL == m_ems_model ||
        EMSModel::SL_YUNNEI_GB6_DIESEL == m_ems_model ) {
        requests.begin()->req_id    = UDS::Config::did_diesel.begin()->req_id;
        requests.begin()->rsp_id    = UDS::Config::did_diesel.begin()->rsp_id;
        requests.begin()->svc_id    = UDS::Config::did_diesel.begin()->svc_id;
        requests.begin()->property  = UDS::Config::did_diesel.begin()->property;
        for ( auto const &cfg : UDS::Config::did_diesel ) {
            requests.begin()->list.push_back(smartlink_sdk::UdsReqNode{ 2, (SMLK_UINT8 *) &cfg.extend_id});
        }
        req.req_id      = obd::can::identifier::diesel::Request;
        req.rsp_id      = obd::can::identifier::diesel::Response;
    }
    else
    {
         SMLK_LOGW("------EMS Model not surpport %d------",m_ems_model);
         return;
    }
    requests.push_back(req);

    SMLK_LOGI("try to send UDS request");
    for ( auto const &request : requests ) {
        SMLK_LOGI("================================");
        SMLK_LOGI("request.req_id:      0x%08X",    request.req_id);
        SMLK_LOGI("request.rsp_id:      0x%08X",    request.rsp_id);
        SMLK_LOGI("request.svc_id:      0x%08X",    request.svc_id);
        SMLK_LOGI("request.property:    %d",        request.property);
        for ( auto const &node : request.list ) {
            std::stringstream   ss;
            for ( decltype(node.len) index = 0; index < node.len; index++ ) {
                ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)node.data[index];
            }
            SMLK_LOGI("--------------------------------");
            SMLK_LOGI("\tnode len: %d, data:%s", node.len, ss.str().c_str());
        }
        SMLK_LOGI("================================");
    }

    auto return_code = smartlink_sdk::MCU::GetInstance()->SendUDSReq(requests);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to SendUDSReq  informaiton, return_code = %d", static_cast<std::int32_t>(return_code));
    }
    SMLK_LOGI("QueryOBD exit");
}


/**
 * @brief 初始化平台配置
 * 
 * @return SMLK_UINT32 
 */
SMLK_UINT32 ReportService::InitTSPConfig()
{
    std::string property;

    if (m_env)
    {
        m_ent_tspinfo.host = m_host_01;
        m_ent_tspinfo.port = m_port_01;
        m_loc_tspinfo.host = m_host_02;
        m_loc_tspinfo.port = m_port_02;
    }
    else
    {
        // 2. 8F42 读取参数设置
        std::string config;
        property = SYS_PRO_NAME_STD_DISCHAR_GW_SET;
        auto return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, config);
        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code)
        {
            SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
        }
        else
        {
            SMLK_LOGI("8F42 config:        %s", config.c_str());
            Var a = Var::parse(config);

            std::string send_switch = a["SendSwitch"];

            std::string ep_ptc_type = a["EpProtType"];
            std::string ep_ptc_ip = a["EpProtIp"];
            std::string ep_ptc_port = a["EpProtPort"];

            std::string lm_ptc_type = a["LmProtType"];
            std::string lm_ptc_ip = a["LmProtIp"];
            std::string lm_ptc_port = a["LmProtPort"];

            SMLK_UINT8 protocol = std::stoi(ep_ptc_type);
            SMLK_UINT16 port = std::stoi(ep_ptc_port);

            if (protocol == 0x04)
            {
                m_ent_tspinfo.valid = true;
                m_ent_tspinfo.protocol = ProtocolVer::BJ;
                m_ent_tspinfo.host = ep_ptc_ip;
                m_ent_tspinfo.port = port;
            }
            else if (protocol == 0x05)
            {
                m_ent_tspinfo.valid = true;
                m_ent_tspinfo.protocol = ProtocolVer::TS;
                m_ent_tspinfo.host = ep_ptc_ip;
                m_ent_tspinfo.port = port;
            }
            else if (protocol == 0x06)
            {
                m_ent_tspinfo.valid = true;
                m_ent_tspinfo.protocol = ProtocolVer::HJ;
                m_ent_tspinfo.host = ep_ptc_ip;
                m_ent_tspinfo.port = port;
            }
            else
            {
                m_loc_tspinfo.valid = false;
            }

            protocol = std::stoi(lm_ptc_type);
            port = std::stoi(lm_ptc_port);
            if (protocol == 0x04)
            {
                m_loc_tspinfo.valid = true;
                m_loc_tspinfo.protocol = ProtocolVer::BJ;
                m_loc_tspinfo.host = lm_ptc_ip;
                m_loc_tspinfo.port = port;
            }
            else if (protocol == 0x05)
            {
                m_loc_tspinfo.valid = true;
                m_loc_tspinfo.protocol = ProtocolVer::TS;
                m_loc_tspinfo.host = lm_ptc_ip;
                m_loc_tspinfo.port = port;
            }
            else if (protocol == 0x06)
            {
                m_loc_tspinfo.valid = true;
                m_loc_tspinfo.protocol = ProtocolVer::HJ;
                m_loc_tspinfo.host = lm_ptc_ip;
                m_loc_tspinfo.port = port;
            }
            else
            {
                m_loc_tspinfo.valid = false;
            }
            SMLK_LOGI("8F42 config: send_switch %s", send_switch.c_str());
            SMLK_LOGI("8F42 config: type %s, %s:%d", ep_ptc_type.c_str(), m_ent_tspinfo.host.c_str(), m_ent_tspinfo.port);
            SMLK_LOGI("8F42 config: type %s, %s:%d", lm_ptc_type.c_str(), m_loc_tspinfo.host.c_str(), m_loc_tspinfo.port);
        }
    }

    // DynamicStruct dySturct;

    // dySturct["SN"] = "1";

    // Var v1(dySturct);
    // std::string stdDiscIpSet = v1.convert<std::string>();
    // auto return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_GB_RECORDED_STATUS, stdDiscIpSet);
    // if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
    //     SMLK_LOGE("Set name=%s as %s fail!", SYS_PRO_NAME_STD_DISCHAR_GW_SET, stdDiscIpSet.c_str());
    // }

    // SMLK_UINT32 m_state = M_HJ_1239_ACTIVATED_RESERVE;

    // std::string json;
    // property = SYS_PRO_NAME_GB_RECORDED_STATUS;
    //  return_code = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, json);
    // if (smartlink_sdk::RtnCode::E_SUCCESS != return_code)
    // {
    //     SMLK_LOGE("fail to get system property \"%s\", return code: %d", property.c_str(), static_cast<std::int32_t>(return_code));
    // }
    // else
    // {
    //     SMLK_LOGE("system property %s %s", property.c_str(), json.c_str());

    //     Poco::JSON::Parser parser;
    //     Poco::Dynamic::Var var_json;
    //     /*判断参数json合法性*/
    //     try
    //     {
    //         var_json = parser.parse(json.c_str());
    //     }
    //     catch (Poco::Exception &exc)
    //     {
    //         SMLK_LOGE("Parse Error:%s", exc.displayText().c_str());
    //         return SL_EFAILED;
    //     }
    //     /*判断json中RecRes字段是否存在且数据类型为int*/
    //     SMLK_UINT8 m_rec_sta = 0;
    //     SMLK_UINT8 m_rec_res = 0;
    //     Poco::JSON::Object obj = *var_json.extract<Poco::JSON::Object::Ptr>();
    //     Poco::Dynamic::Var sn = obj.get("SN");
    //     if (!sn.isString())
    //     {
    //         SMLK_LOGD("JSON \"SN\" type error!");
    //         resulthandler::getInstance()->m_808_sn = "";
    //     } else {
    //         resulthandler::getInstance()->m_808_sn = sn.convert<std::string>();
    //     }

    //     Poco::Dynamic::Var rec_sta = obj.get("RecSta");
    //     if (!rec_sta.isInteger())
    //     {
    //         SMLK_LOGD("JSON \"RecSta\" type error!");
    //     }
    //     else
    //     {
    //         m_rec_sta = rec_sta.convert<SMLK_UINT8>();
    //     }

    //     Poco::Dynamic::Var rec_res = obj.get("RecRes");
    //     if (!rec_res.isInteger())
    //     {
    //         SMLK_LOGD("JSON \"RecSta\" type error!");
    //     }
    //     else
    //     {
    //         m_rec_res = rec_res.convert<SMLK_UINT8>();
    //     }

    //     if (m_rec_sta == 1 || (m_rec_sta == 3 && m_rec_res != 2))
    //     {
    //         m_state = M_HJ_1239_ACTIVATED_ING;
    //     }
    // }

    // m_protocol_reporter_1239_07->SetActStatus(m_state);    


    if (m_ent_tspinfo.valid && isIP(m_ent_tspinfo.host.data()) == false)
    {
        char **pptr;
        char str[32] = {0};
        struct hostent *hptr = ::gethostbyname(m_ent_tspinfo.host.c_str());
        if (hptr == nullptr)
        {
            SMLK_LOGE("[client] gethostbyname failed. \n");
            m_ent_tspinfo.valid = false;
        }
        else
        {
            pptr = hptr->h_addr_list;
            const char *ip = inet_ntop(hptr->h_addrtype, hptr->h_addr, str, sizeof(str));
            m_ent_tspinfo.host = ip;
            SMLK_LOGE("[client] gethostbyname success. %s \n", m_ent_tspinfo.host.c_str());
        }
    }

    if (m_loc_tspinfo.valid && isIP(m_loc_tspinfo.host.data()) == false)
    {
        char **pptr;
        char str[32] = {0};
        struct hostent *hptr = ::gethostbyname(m_loc_tspinfo.host.c_str());
        if (hptr == nullptr)
        {
            SMLK_LOGE("[client] gethostbyname failed. \n");
            m_loc_tspinfo.valid = false;
        }
        else
        {
            pptr = hptr->h_addr_list;
            const char *ip = inet_ntop(hptr->h_addrtype, hptr->h_addr, str, sizeof(str));
            m_loc_tspinfo.host = ip;
            SMLK_LOGE("[client] gethostbyname success. %s \n", m_loc_tspinfo.host.c_str());
        }
    }

    if (m_ent_tspinfo.valid && (m_ent_tspinfo.protocol == ProtocolVer::TS || m_ent_tspinfo.protocol == ProtocolVer::BJ || m_ent_tspinfo.protocol == ProtocolVer::HJ))
    {
        m_protocol_reporter[m_ent_tspinfo.platform] = std::make_shared<Reporter>(m_ent_tspinfo.host, m_ent_tspinfo.port, m_ent_tspinfo.protocol, m_ent_tspinfo.platform);
    }

    if (m_loc_tspinfo.valid && (m_loc_tspinfo.protocol == ProtocolVer::TS || m_loc_tspinfo.protocol == ProtocolVer::BJ || m_loc_tspinfo.protocol == ProtocolVer::HJ))
    {
        m_protocol_reporter[m_loc_tspinfo.platform] = std::make_shared<Reporter>(m_loc_tspinfo.host, m_loc_tspinfo.port, m_loc_tspinfo.protocol, m_loc_tspinfo.platform);
    }

    for (auto &pair : m_protocol_reporter)
    {
        pair.second->SetICCID(m_iccid);
        pair.second->SetRecorderDir(M_GB6_RECORD_DATABASE_DIR);
        pair.second->SetIsSave(m_todisk);
        pair.second->SetProductLoc(m_product_loc);
        pair.second->SetReportInfo(is_gas, is_hev);
        // pair.second->SetActStatus(m_state);    
        // pair.second->SetVIN(m_vin);
        pair.second->SetChipID(m_chipid);


        auto rc = pair.second->Init();
        if (SL_SUCCESS != rc)
        {
            SMLK_LOGE("fail to init protocol reporter, protocol = 0x%02X", pair.first);
            return SL_EFAILED;
        }
    }
}
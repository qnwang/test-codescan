#include <dlfcn.h>

#include "adas_so_pcc.h"
#include "custom_util.hpp"

using namespace smartlink;


AdasSoPcc::AdasSoPcc()
    : AdasSoBase()
{
}

AdasSoPcc::~AdasSoPcc()
{

}

bool AdasSoPcc::Init()
{
    bool b_is_qingdao = false;
    std::string str_vcu_type;
    std::string str_pro = SYS_PRO_NAME_VCU;
    smartlink_sdk::SysProperty::GetInstance()->GetValue(str_pro, str_vcu_type);

    if (str_vcu_type != "1"  && str_vcu_type != "4")
    {
        return false;
    }
    else if(str_vcu_type == "4")
    {
        b_is_qingdao = true;
    }

     //动态加载adas.so库
    if (b_is_qingdao)
    {
        m_p_adas_lib = dlopen("/oemapp/smartlink/lib/libadasisv2hp_qingdao.so", RTLD_NOW);
        SMLK_LOGI("[pcc] this is qingdao");
    }
    else
    {
        m_p_adas_lib = dlopen("/oemapp/smartlink/lib/libadasisv2hp.so", RTLD_NOW);
        SMLK_LOGI("[pcc] this is yiqi");

    }

    if( NULL == m_p_adas_lib )
    {
        SMLK_LOGE("dlopen adas.so failure.....");
        return false;
    }
    // 加载HP相关函数
    m_p_Av2HP_getMapVersion = (OnAv2HP_getMapVersion)dlsym(m_p_adas_lib, "Av2HP_getMapVersion");
    if( NULL == m_p_Av2HP_getMapVersion )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getMapVersion err.....");
        return false;
    }

    m_p_Av2HP_getSoftVersion = (OnAv2HP_getSoftVersion)dlsym(m_p_adas_lib, "Av2HP_getSoftVersion");
    if( NULL == m_p_Av2HP_getSoftVersion )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getSoftVersion err.....");
        return false;
    }

    m_p_Av2HP_init = (OnAv2HP_init)dlsym(m_p_adas_lib, "Av2HP_init");
    if( NULL == m_p_Av2HP_init )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_init err.....");
        return false;
    }

    m_p_Av2HP_destory = (OnAv2HP_destory)dlsym(m_p_adas_lib, "Av2HP_destory");
    if( NULL == m_p_Av2HP_destory )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_destory err.....");
        return false;
    }

    m_p_Av2HP_setDeviceId = (OnAv2HP_setDeviceId)dlsym(m_p_adas_lib, "Av2HP_setDeviceId");
    if( NULL == m_p_Av2HP_setDeviceId )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_setDeviceId err.....");
        return false;
    }

    m_p_Av2HP_getMeta = (OnAv2HP_getMeta)dlsym(m_p_adas_lib, "Av2HP_getMeta");
    if( NULL == m_p_Av2HP_getMeta )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getMeta err.....");
        return false;
    }

    m_p_Av2HP_run = (OnAv2HP_run)dlsym(m_p_adas_lib, "Av2HP_run");
    if( NULL == m_p_Av2HP_run )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_run err.....");
        return false;
    }

    m_p_Av2HP_setGpsInfo = (OnAv2HP_setGpsInfo)dlsym(m_p_adas_lib, "Av2HP_setGpsInfo");
    if( NULL == m_p_Av2HP_setGpsInfo )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_setGpsInfo err.....");
        return false;
    }

    m_p_Av2HP_getSoftValid = (OnAv2HP_getSoftValid)dlsym(m_p_adas_lib, "Av2HP_getSoftValid");
    if( NULL == m_p_Av2HP_getSoftValid )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getSoftValid err.....");
        return false;
    }

    m_p_Av2HP_setMessageCB = (OnAv2HP_setMessageCB)dlsym(m_p_adas_lib, "Av2HP_setMessageCB");
    if( NULL == m_p_Av2HP_setMessageCB )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_setMessageCB err.....");
        return false;
    }

    m_p_Av2HP_getMsgType = (OnAv2HP_getMsgType)dlsym(m_p_adas_lib, "Av2HP_getMsgType");
    if( NULL == m_p_Av2HP_getMsgType )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getMsgType err.....");
        return false;
    }

    return true;
}

void AdasSoPcc::AdasGetMapVersion(const char *map_path, char *version, int length)
{
    m_p_Av2HP_getMapVersion(const_cast<char *>(map_path), version, length);
}

void AdasSoPcc::AdasGetSoftVersion(char* version, int length)
{
    m_p_Av2HP_getSoftVersion(version, length);
}
bool AdasSoPcc::AdasInit(const char* config_path, const char* device_id)
{
    return (m_p_Av2HP_init(config_path) == IAV2HP_SUCCESS);
}

void AdasSoPcc::AdasSetDeviceId(const char* device_id)
{
    m_p_Av2HP_setDeviceId(device_id);
}

void AdasSoPcc::AdasDestory()
{
    m_p_Av2HP_destory();
}

bool AdasSoPcc::AdasRun()
{
    return (m_p_Av2HP_run() == IAV2HP_SUCCESS);
}

int AdasSoPcc::AdasGetSoftValid()
{
    return m_p_Av2HP_getSoftValid();
}

void AdasSoPcc::AdasSetGpsInfo(const smartlink_sdk::LocationInfo &location_info, const smartlink_sdk::GnssSatelliteInfo &satellite_info)
{
    m_hp_gps_info.m_valid = location_info.fix_valid;
    m_hp_gps_info.m_speed.m_unit = av2hp_speedUnit_ms;
    if(m_hp_gps_info.m_valid)
    {
        m_hp_gps_info.m_pos.m_lon = location_info.longitude;
        m_hp_gps_info.m_pos.m_lat = location_info.latitude;
        m_hp_gps_info.m_pos.m_alt = location_info.altitude;

        m_hp_gps_info.m_hdop = location_info.hdop;
        m_hp_gps_info.m_pdop = location_info.pdop;
        m_hp_gps_info.m_vdop = location_info.vdop;

        m_hp_gps_info.m_speed.m_value = location_info.speed;

        m_hp_gps_info.m_satInViewNum = satellite_info.visible_num;
        m_hp_gps_info.m_satNum = satellite_info.tracked_num;

        m_hp_gps_info.m_orient = location_info.heading;
        m_hp_gps_info.m_timestamp = location_info.utc;
        SMLK_UINT16 timestamp[7];
        Util::millisecondTime_2_bcd(location_info.utc, timestamp);
        m_hp_gps_info.m_dateTime.m_year    = timestamp[0];
        m_hp_gps_info.m_dateTime.m_month   = timestamp[1];
        m_hp_gps_info.m_dateTime.m_day     = timestamp[2];
        m_hp_gps_info.m_dateTime.m_hours   = timestamp[3];
        m_hp_gps_info.m_dateTime.m_minutes = timestamp[4];
        m_hp_gps_info.m_dateTime.m_seconds = timestamp[5];
    }
    else
    {
        m_hp_gps_info.m_pos.m_lon = 0;
        m_hp_gps_info.m_pos.m_lat = 0;
        m_hp_gps_info.m_pos.m_alt = 0;

        m_hp_gps_info.m_hdop = 0;
        m_hp_gps_info.m_pdop = 0;
        m_hp_gps_info.m_vdop = 0;

        m_hp_gps_info.m_speed.m_value = 0;

        m_hp_gps_info.m_satInViewNum = 0;
        m_hp_gps_info.m_satNum = 0;

        m_hp_gps_info.m_orient = 0;
        m_hp_gps_info.m_timestamp = 0;

        m_hp_gps_info.m_dateTime.m_year    = 0;
        m_hp_gps_info.m_dateTime.m_month   = 0;
        m_hp_gps_info.m_dateTime.m_day     = 0;
        m_hp_gps_info.m_dateTime.m_hours   = 0;
        m_hp_gps_info.m_dateTime.m_minutes = 0;
        m_hp_gps_info.m_dateTime.m_seconds = 0;
    }
    SMLK_LOGD("set adas gpd info, fix_vaild:%d, m_lon:%f, m_lat:%f, m_alt:%f, m_speed:%f,m_timestamp: %lld", m_hp_gps_info.m_valid, m_hp_gps_info.m_pos.m_lon, m_hp_gps_info.m_pos.m_lat, m_hp_gps_info.m_pos.m_alt, m_hp_gps_info.m_speed.m_value, m_hp_gps_info.m_timestamp);

    m_p_Av2HP_setGpsInfo(&m_hp_gps_info);
}

bool AdasSoPcc::AdasGetHPMsgCb(OnAv2HP_setMessageCB &ptr)
{
    if (m_p_Av2HP_setMessageCB == NULL)
    {
        return false;
    }
    ptr = m_p_Av2HP_setMessageCB;
    return true;
}

int AdasSoPcc::AdasGetMsgType(const unsigned char* value, unsigned char length)
{
    return m_p_Av2HP_getMsgType(const_cast<unsigned char*>(value), (uint8_t)length);
}

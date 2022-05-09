#include "adas_so_l2.h"
#include "custom_util.hpp"



using namespace smartlink;

AdasSoL2::AdasSoL2(): AdasSoBase()
{

}

AdasSoL2::~AdasSoL2()
{

}

bool AdasSoL2::Init()
{
    return true;
}


void AdasSoL2::AdasGetMapVersion(const char *map_path, char *version, int length)
{
    adas_get_map_version(const_cast<char *>(map_path), version, length, NULL);
}

void AdasSoL2::AdasGetSoftVersion(char* version, int length)
{
    adas_get_soft_version(version, length);
}

bool AdasSoL2::AdasInit(const char* config_path, const char* device_id)
{
    bool b_ret = true;
    int ret = adas_init(config_path, device_id);
    if (ret != adas_status_success)
    {
        b_ret = false;
        SMLK_LOGW("adas_init err, ret: %d ", ret);
    }
    return b_ret;
}

void AdasSoL2::AdasSetDeviceId(const char* device_id)
{
}

void AdasSoL2::AdasDestory()
{
    adas_destory();
}

bool AdasSoL2::AdasRun()
{
    return true;
}

int AdasSoL2::AdasGetSoftValid()
{
    return adas_get_soft_valid();
}

void AdasSoL2::AdasSetGpsInfo(const smartlink_sdk::LocationInfo &location_info, const smartlink_sdk::GnssSatelliteInfo &satellite_info)
{
    m_adas_gps_info.pos.kind = adas_coordinate_wgs84;
    if (location_info.fix_valid)
    {
        m_adas_gps_info.gprmc_status = 'A';
        m_adas_gps_info.gprmc_mode = 'A';
        m_adas_gps_info.mode = 1;

        m_adas_gps_info.pos.lon = location_info.longitude;
        m_adas_gps_info.pos.lat = location_info.latitude;
        m_adas_gps_info.pos.alt = location_info.altitude;

        m_adas_gps_info.orient = location_info.heading;
        m_adas_gps_info.speed  = location_info.speed;

        m_adas_gps_info.pdop  = location_info.pdop;
        m_adas_gps_info.hdop  = location_info.hdop;
        m_adas_gps_info.vdop  = location_info.vdop;

        m_adas_gps_info.sat_num = satellite_info.tracked_num;

        SMLK_UINT16 timestamp[7];
        Util::millisecondTime_2_bcd(location_info.utc, timestamp);
        m_adas_gps_info.date_time.year        = timestamp[0];
        m_adas_gps_info.date_time.month       = timestamp[1];
        m_adas_gps_info.date_time.day         = timestamp[2];
        m_adas_gps_info.date_time.hour        = timestamp[3];
        m_adas_gps_info.date_time.minute      = timestamp[4];
        m_adas_gps_info.date_time.second      = timestamp[5];
        m_adas_gps_info.date_time.millisecond = timestamp[6];

    }
    else
    {
        m_adas_gps_info.gprmc_status = 'V';
        m_adas_gps_info.gprmc_mode = 'N';
        m_adas_gps_info.mode = 0;

        m_adas_gps_info.pos.lon = 0;
        m_adas_gps_info.pos.lat = 0;
        m_adas_gps_info.pos.alt = 0;

        m_adas_gps_info.orient = 0;
        m_adas_gps_info.speed  = 0;

        m_adas_gps_info.pdop  = 0;
        m_adas_gps_info.hdop  = 0;
        m_adas_gps_info.vdop  = 0;

        m_adas_gps_info.sat_num = 0;
        m_adas_gps_info.date_time.year        = 0;
        m_adas_gps_info.date_time.month       = 0;
        m_adas_gps_info.date_time.day         = 0;
        m_adas_gps_info.date_time.hour        = 0;
        m_adas_gps_info.date_time.minute      = 0;
        m_adas_gps_info.date_time.second      = 0;
        m_adas_gps_info.date_time.millisecond = 0;

    }
    m_adas_gps_info.speed_up = 0;


    adas_set_gps_info(&m_adas_gps_info);
}

bool AdasSoL2::AdasGetHPMsgCb(OnAv2HP_setMessageCB &ptr)
{
    ptr = NULL;
    return false;
}

int AdasSoL2::AdasGetMsgType(const unsigned char* value, unsigned char length)
{
    return -1;
}

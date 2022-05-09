/*****************************************************************************/
/**
* \file       adas_so_l2.h
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
#ifndef _ADAS_SO_L2_H_
#define _ADAS_SO_L2_H_


#include "adas_so_base.h"

namespace smartlink
{

class AdasSoL2 : public AdasSoBase
{
public:
    AdasSoL2();
    ~AdasSoL2() override;

    bool Init() override;
    void AdasGetMapVersion(const char *map_path, char *version, int length) override;
    void AdasGetSoftVersion(char* version, int length) override;
    bool AdasInit(const char* config_path, const char* device_id) override;
    void AdasSetDeviceId(const char* device_id) override;
    void AdasDestory() override;
    bool AdasRun() override;
    int  AdasGetSoftValid() override;
    void AdasSetGpsInfo(const smartlink_sdk::LocationInfo &location_info, const smartlink_sdk::GnssSatelliteInfo &satellite_info) override;
    bool AdasGetHPMsgCb(OnAv2HP_setMessageCB &ptr) override;
    int  AdasGetMsgType(const unsigned char* value, unsigned char length) override;


private:
    adas_gps_info  m_adas_gps_info;
};

}



#endif
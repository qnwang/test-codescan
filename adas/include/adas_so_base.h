/*****************************************************************************/
/**
* \file       adas_so_base.h
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
#ifndef _ADAS_SO_BASE_H_
#define _ADAS_SO_BASE_H_

#include <smartlink_sdk_sys_property.h>
#include <smartlink_sdk_location.h>


#include <adas/adasisv2hp.h>
#include <adas/adas.h>


#include "smartlink_sdk_sys_property_def.h"
#include "smlk_types.h"
#include "smlk_log.h"

namespace smartlink
{
    using OnAv2HP_setMessageCB     = int (*) (Av2HP_pushMessageCB av2hp_cb);

class AdasSoBase
{
public:
    AdasSoBase(){};
    virtual ~AdasSoBase(){};

    virtual bool Init() = 0;
    virtual void AdasGetMapVersion(const char *map_path, char *version, int length) = 0;
    virtual void AdasGetSoftVersion(char* version, int length) = 0;
    virtual bool AdasInit(const char* config_path, const char* device_id) = 0;
    virtual void AdasSetDeviceId(const char* device_id) = 0;
    virtual void AdasDestory() = 0;
    virtual bool AdasRun() = 0;
    virtual int  AdasGetSoftValid() = 0;
    virtual void AdasSetGpsInfo(const smartlink_sdk::LocationInfo &location_info, const smartlink_sdk::GnssSatelliteInfo &satellite_info) = 0;
    virtual bool AdasGetHPMsgCb(OnAv2HP_setMessageCB &ptr) = 0;
    virtual int  AdasGetMsgType(const unsigned char* value, unsigned char length) = 0;
};

}



#endif
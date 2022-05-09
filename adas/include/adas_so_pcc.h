/*****************************************************************************/
/**
* \file       adas_so_pcc.h
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
#ifndef _ADAS_SO_PCC_H_
#define _ADAS_SO_PCC_H_


#include "adas_so_base.h"

namespace smartlink
{


class AdasSoPcc : public AdasSoBase
{
    using OnAv2HP_getMapVersion     = void (*) (char* path, char* version, int lenth);
    using OnAv2HP_getSoftVersion    = void (*) (char* version, int lenth);
    using OnAv2HP_init              = av2hp_e (*) (const char *conf_path);
    using OnAv2HP_destory           = void (*) (void);
    using OnAv2HP_setDeviceId       = void (*) (const char* strDeviceId);
    using OnAv2HP_getMeta           = av2hp_e (*) (av2hp_meta *meta_data);
    using OnAv2HP_run               = av2hp_e (*) (void);
    using OnAv2HP_setGpsInfo        = av2hp_e (*) (av2hp_gpsInfo *gps);
    using OnAv2HP_getSoftValid      = av2hp_license_status (*) (void);
    using OnAv2HP_getMsgType        = unsigned char (*) (unsigned char* value, unsigned char length);

public:
    AdasSoPcc();
    ~AdasSoPcc() override;

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
    void *m_p_adas_lib = NULL;      //指向adas.so文件的指针
    OnAv2HP_getMapVersion   m_p_Av2HP_getMapVersion = NULL;    //对应的hp库函数指针
    OnAv2HP_getSoftVersion  m_p_Av2HP_getSoftVersion = NULL;
    OnAv2HP_init            m_p_Av2HP_init           = NULL;
    OnAv2HP_destory         m_p_Av2HP_destory        = NULL;
    OnAv2HP_setDeviceId     m_p_Av2HP_setDeviceId    = NULL;
    OnAv2HP_getMeta         m_p_Av2HP_getMeta        = NULL;
    OnAv2HP_run             m_p_Av2HP_run            = NULL;
    OnAv2HP_setGpsInfo      m_p_Av2HP_setGpsInfo     = NULL;
    OnAv2HP_getSoftValid    m_p_Av2HP_getSoftValid   = NULL;
    OnAv2HP_setMessageCB    m_p_Av2HP_setMessageCB   = NULL;
    OnAv2HP_getMsgType      m_p_Av2HP_getMsgType     = NULL;

    av2hp_gpsInfo       m_hp_gps_info;
};
}


#endif
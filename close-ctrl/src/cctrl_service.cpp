/*******************************************************************************
|  File Name:   cctrl_service.cpp
|  Function:    Receive ctrl msg from wifi service and send ctrl msg to mcu&IVI
|-------------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
| Date           Version      Author        Description
| ------------   --------     -------       ------------------------------------
| 2021-01-10     01.00.00     Xiongyijun    Reconstruct: remove http server
|
|******************************************************************************/
#include <sys/msg.h>
#include <sys/ipc.h>
#include <iostream>
#include <cstring>
#include <mutex>
#include <queue>
#include <chrono>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <json/json.h>
#include "cctrl_service.h"
#include "smartlink_sdk_mcu.h"
#include "smartlink_sdk_ivi.h"
#include "smartlink_sdk_sys_property.h"
#include "smlk_log.h"

using namespace smartlink::CloseCtrl;
using namespace smartlink::Wifi;
using namespace std;
using namespace smartlink;
using namespace smartlink_sdk;

/*IVI音量控制信号表*/
IviControlMsg iviCtrlVolumeArry[9] =
    {
        {0, 0, CCTRL_CMD_SYS_VOL_DOWN},
        {0, 1, CCTRL_CMD_SYS_VOL_UP},
        {1, 0, CCTRL_CMD_CALL_VOL_DOWN},
        {1, 1, CCTRL_CMD_CALL_VOL_UP},
        {2, 0, CCTRL_CMD_MUTE_SWITCH},
        {2, 1, CCTRL_CMD_MUTE_SWITCH},
        {4, 0, CCTRL_CMD_BALANCE_DOWN},
        {4, 1, CCTRL_CMD_BALANCE_UP},
        {0xff, 0xff, CCTRL_CMD_SOUND_NONE},
};
/*IVI音效控制信号表*/
IviControlMsg iviCtrlSoundArry[11] =
    {
        {0, 0, CCTRL_CMD_EFFECT_NONE},
        {0, 1, CCTRL_CMD_EFFECT_ROCK},
        {0, 2, CCTRL_CMD_EFFECT_JAZZ},
        {0, 3, CCTRL_CMD_EFFECT_CLASSIC},
        {0, 4, CCTRL_CMD_EFFECT_VOCAL},
        {0, 5, CCTRL_CMD_EFFECT_ELECTRONIC},
        {0, 6, CCTRL_CMD_EFFECT_POP},
        {1, 0, CCTRL_CMD_TONE_LOW},
        {1, 1, CCTRL_CMD_TONE_MID},
        {1, 2, CCTRL_CMD_TONE_HIGH},
        {0xff, 0xff, CCTRL_CMD_SOUND_NONE},
};
/*IVI液晶屏控制信号表*/
IviControlMsg iviCtrlLCDArry[9] =
    {
        {0, 0, CCTRL_CMD_LCD_MODE_DAY},
        {0, 1, CCTRL_CMD_LCD_MODE_NIGHT},
        {0, 2, CCTRL_CMD_LCD_MODE_AUTO},
        {1, 0, CCTRL_CMD_POWER_MODE_NORMAL},
        {1, 1, CCTRL_CMD_POWER_MODE_OFF},
        {1, 2, CCTRL_CMD_POWER_MODE_TIME},
        {2, 0, CCTRL_CMD_LCD_BRIGHTNESS_DIMMING},
        {2, 1, CCTRL_CMD_LCD_BRIGHTNESS_ADJUST},
        {0xff, 0xff, CCTRL_CMD_LCD_NONE},
};
/*IVI收音机控制信号表*/
IviControlMsg iviCtrlRadioArry[15] =
    {
        {0, 0, CCTRL_CMD_FM},
        {0, 1, CCTRL_CMD_AM},
        {1, 0, CCTRL_CMD_SEARCH},
        {2, 0, CCTRL_CMD_COLLECTION},
        {3, 0, CCTRL_CMD_COLLECTION_PAGE},
        {4, 0, CCTRL_CMD_PAGE_UP},
        {4, 1, CCTRL_CMD_PAGE_DOWN},
        {4, 2, CCTRL_CMD_SEARCH_LEFT},
        {4, 3, CCTRL_CMD_SEARCH_RIGHT},
        {5, 0, CCTRL_CMD_BROCAST_RADIO}, /*播放电台*/
        {6, 0, CCTRL_CMD_FREQUENCY_UP},
        {6, 1, CCTRL_CMD_FREQUENCY_DOWN},
        {6, 2, CCTRL_CMD_SEARCH_RIGHT},
        {6, 3, CCTRL_CMD_SEARCH_LEFT},
        {0xff, 0xff, CCTRL_CMD_RADIO_NONE},
};
/*IVI音频控制信号表*/
IviControlMsg iviCtrlAudioArry[11] =
    {
        {0, 0, CCTRL_CMD_PLAYER_MODE_SINGLE},
        {0, 1, CCTRL_CMD_PLAYER_MODE_FULL},
        {0, 2, CCTRL_CMD_PLAYER_MODE_RANDOM},
        {1, 0, CCTRL_CMD_PLAYER_PLAY_PAUSE},
        {2, 0, CCTRL_CMD_PLAYER_PLAY_PREVIOUS},
        {2, 1, CCTRL_CMD_PLAYER_PLAY_NEXT},
        {0xff, 0xff, CCTRL_CMD_PLAYER_NONE},
};

int i_McuCmdMsg[] =
    {
        CCTRL_CMD_AC_SWITCH,
        CCTRL_CMD_AC_AIROUTLET_MODE,
        CCTRL_CMD_AC_WIND,
        CCTRL_CMD_AC_TEMPER,
        CCTRL_CMD_AC_LOOP_MOOD,
        CCTRL_CMD_AC_COMPRESSOR,
        CCTRL_CMD_AC_AUTO_SWITCH,
        CCTRL_CMD_AC_DEFROST,
        CCTRL_CMD_INDEPT_WARM_AIR,
        CCTRL_CMD_INDEPT_WARM_AIR_TEMPER,
        CCTRL_CMD_PARKING_AC_SWITCH,
        CCTRL_CMD_PARKING_AC_TEMPER,
        CCTRL_CMD_PARKING_AC_WIND,
        CCTRL_CMD_AC_QUERY,
        CCTRL_CMD_PARKING_AC_AUTO,
        CCTRL_CMD_AC_VENTILATION,
};
/*空调控制MCU<-->VD消息绑定*/
std::map<size_t, size_t> m_cctl_vd_map = {
    {CCTRL_CMD_AC_WIND, VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE},
    {CCTRL_CMD_AC_TEMPER, VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE},
    {CCTRL_CMD_INDEPT_WARM_AIR_TEMPER, VEHICLE_DATA_WARM_AIR_TEMPERATURE},
    {CCTRL_CMD_PARKING_AC_TEMPER, VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE},
    {CCTRL_CMD_PARKING_AC_WIND, VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE},
};

std::map<size_t, size_t> m_vd_cctl_map = {
    {VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE, CCTRL_CMD_AC_WIND},
    {VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE, CCTRL_CMD_AC_TEMPER},
    {VEHICLE_DATA_WARM_AIR_TEMPERATURE, CCTRL_CMD_INDEPT_WARM_AIR_TEMPER},
    {VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE, CCTRL_CMD_PARKING_AC_TEMPER},
    {VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE, CCTRL_CMD_PARKING_AC_WIND},
};

std::map<SmlkStateQueryCCtrl, size_t> m_wifimcu_vd_map = {
    /*保留暂时不实现*/
    // {SmlkStateQueryCCtrl::QUERY_BCM_DRIVER_LIGHT,                  VEHICLE_DATA_BCM_DRIVER_LIGHT},
    // {SmlkStateQueryCCtrl::QUERY_BCM_PASSENGER_LIGHT,               VEHICLE_DATA_BCM_PASSENGER_LIGHT},
    // {SmlkStateQueryCCtrl::QUERY_BCM_REST_ROOM_LIGHT,               VEHICLE_DATA_BCM_REST_ROOM_LIGHT},
    // {SmlkStateQueryCCtrl::QUERY_DCM_DRIVER_WINDOW,                 VEHICLE_DATA_DCM_DRIVER_WINDOW},
    // {SmlkStateQueryCCtrl::QUERY_DCM_PASSENGER_WINDOW,              VEHICLE_DATA_DCM_PASSENGER_WINDOW},
    {SmlkStateQueryCCtrl::QUERY_AIRCON_SWITCH, VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS},
    /*由于原车空调出风状态VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS包括了出风模式和强制除霜状态,在查询的时候做特殊处理*/
    // {SmlkStateQueryCCtrl::QUERY_AIRCON_OUTLET_MODE,                VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS},
    {SmlkStateQueryCCtrl::QUERY_AIRCON_OUTLET_VOL, VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE},
    {SmlkStateQueryCCtrl::QUERY_AIRCON_OUTLET_TEMP, VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE},
    {SmlkStateQueryCCtrl::QUERY_AIRCON_CYCLE_MODE, VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS},
    {SmlkStateQueryCCtrl::QUERY_AIRCON_A_C_SWITCH, VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS},
    {SmlkStateQueryCCtrl::QUERY_AIRCON_AUTO_SWITCH, VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS},
    /*由于原车空调出风状态VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS包括了出风模式和强制除霜状态,在查询的时候做特殊处理*/
    // {SmlkStateQueryCCtrl::QUERY_AIRCON_DEFROST_SWITCH,             VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS},
    {SmlkStateQueryCCtrl::QUERY_AIRCON_HEATING_SWITCH, VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS},
    {SmlkStateQueryCCtrl::QUERY_AIRCON_HEATING_TEMP, VEHICLE_DATA_WARM_AIR_TEMPERATURE},
    {SmlkStateQueryCCtrl::QUERY_P_AIRCON_SWITCH, VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS},
    {SmlkStateQueryCCtrl::QUERY_P_AIRCON_OUTLET_TEMP, VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE},
    {SmlkStateQueryCCtrl::QUERY_P_AIRCON_OUTLET_VOL, VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE},
    {SmlkStateQueryCCtrl::QUERY_CURRENT_ROOM_TEMP, VEHICLE_DATA_CAB_TEMPERATURE},
    {SmlkStateQueryCCtrl::QUERY_P_AIRCON_AUTO_SWITCH, VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS},
    {SmlkStateQueryCCtrl::QUERY_ORIGIN_AIRCON_VENTILATION_SWITCH, VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS},
};

/*****************************************************************************
 *                                进程管理                                    *
 *****************************************************************************/
CloseCtrlService::CloseCtrlService()
{
    m_flags = IService::Uninitialized;
}

CloseCtrlService::~CloseCtrlService()
{
}

SMLK_RC CloseCtrlService::Init()
{
    SMLK_LOGI("*******enter in func(CloseCtrlService::Init).*******");
    setenv("TZ", "CST-8", 1);
    tzset();
    SMLK_LOGD("set timezone: CST-8 finished.");

    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_flags & IService::Initialized)
    {
        SMLK_LOGI("service already initialized, do nothing.");
        return SMLK_RC::RC_OK;
    }

    /*从MCU获取近控命令指令返回值,IGN状态 下发控车指令到MCU*/
    smartlink_sdk::RtnCode mcu_rc = smartlink_sdk::MCU::GetInstance()->Init(ModuleID::E_Module_cctrl_service);
    if (smartlink_sdk::RtnCode::E_SUCCESS != mcu_rc)
    {
        SMLK_LOGF("fail to init mcu service API interface, err=%d.", (SMLK_UINT8)mcu_rc);
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("MCU->Init() OK.");

    /*注册mcu service的事件通知:车控命令响应,IGN状态*/
    std::vector<smartlink_sdk::McuEventId> mcu_events = {
        smartlink_sdk::McuEventId::E_MCU_EVENT_VEHICLE_CRTL_ACK, /*近控命令执行返回值*/
    };
    smartlink_sdk::MCU::GetInstance()->RegEventCB(mcu_events, std::bind(&CloseCtrlService::OnMcuSerIndication, this,
                                                                        std::placeholders::_1,
                                                                        std::placeholders::_2,
                                                                        std::placeholders::_3));
    m_cctrl_mcu_seq = 0x8000; /*为了区分远近控的序列号,近控范围从0x8000~0xFFFF*/
    /*初始化MCU控制表*/
    InitCctrlToMcuMap();

    /*从VehicleDataApi获取原车,驻车空调温度*/
    SMLK_RC vd_rc = VehicleDataApi::GetInstance()->Init(ModuleID::E_Module_cctrl_service);
    if (SMLK_RC::RC_OK != vd_rc)
    {
        SMLK_LOGF("Fail to init VehicleData service API, err = %d.", vd_rc);
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("VehicleDataApi->Init() OK.");

    /*从IVI获取车机控车执行结果*/
    if (!smartlink_sdk::IVI::GetInstance()->Init(ModuleID::E_Module_cctrl_service))
    {
        SMLK_LOGF("Fail to init IVI service API!");
        return SMLK_RC::RC_ERROR;
    }
    m_ctrl_ivi_cb = bind(&CloseCtrlService::handleIviCtrResult, this, placeholders::_1, placeholders::_2);

    /*从wifi服务模块获取app当前的控车指令*/
    if (SMLK_RC::RC_OK != smartlink::Wifi::WifiServiceApi::getInstance()->Init(E_Module_cctrl_service))
    {
        SMLK_LOGF("Fail to init WIFI service API.");
        return SMLK_RC::RC_ERROR;
    }

    std::vector<smartlink::Wifi::SmlkWifiEventId> wifi_events = {
        smartlink::Wifi::SmlkWifiEventId::WIFI_EVENT_VEHICLE_CTRL, /*近控命令执行返回值*/
        smartlink::Wifi::SmlkWifiEventId::WIFI_EVENT_STATUS_QUERY, /*近控状态回传查询*/
    };
    smartlink::Wifi::WifiServiceApi::getInstance()->RegEventCB(wifi_events, std::bind(&CloseCtrlService::OnWifiSerIndication, this,
                                                                                      std::placeholders::_1,
                                                                                      std::placeholders::_2,
                                                                                      std::placeholders::_3));
    /*Initiation finish*/
    m_flags |= IService::Initialized;
    return SMLK_RC::RC_OK;
}

SMLK_RC CloseCtrlService::Start()
{
    SMLK_LOGD("*******enter in func(%s).*******", __func__);
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_flags & IService::Running)
    {
        SMLK_LOGD("service already started, do nothing.");
        return SMLK_RC::RC_OK;
    }

    if (!(m_flags & IService::Initialized))
    {
        SMLK_LOGE("service not initialized.");
        return SMLK_RC::RC_ERROR;
    }
    m_flags |= IService::Running;
    return SMLK_RC::RC_OK;
}

void CloseCtrlService::Stop()
{
    SMLK_LOGD("*******enter in func(%s).*******", __func__);
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_flags & IService::Running)
    {
        m_flags |= IService::Stopping;
    }

    m_flags &= ~(IService::Running | IService::Stopping);
}

bool CloseCtrlService::IsRunning() const
{
    return m_flags & IService::Running;
}

/*****************************************************************************
 *                               控车指令分配                                  *
 *****************************************************************************/
void CloseCtrlService::OnWifiSerIndication(smartlink::Wifi::SmlkWifiEventId id, void *data, int length)
{
    /*收到Wifi服务模块的控车指令*/
    switch (id)
    {
    case smartlink::Wifi::SmlkWifiEventId::WIFI_EVENT_VEHICLE_CTRL:
        ProcessWifiSvcCCtrl(data, length);
        break;
    case smartlink::Wifi::SmlkWifiEventId::WIFI_EVENT_STATUS_QUERY:
        ProcessWifiSvcStateQuery(data, length);
        break;
    default:
        return;
    }
}

void CloseCtrlService::ProcessWifiSvcCCtrl(IN void *data, IN int len)
{
    SMLK_LOGD("********Ivi Ctrl Start********");
    SmlkWifiIpcData *pcmd = (SmlkWifiIpcData *)data;
    if (pcmd->cmd_type != SmlkWifiIpcCmdType::WIFI_CMD_VEHILCE_CMD)
        return;
    CCtrlInterInfo m_msg;
    m_msg.inter_id = pcmd->ctrlInfo.inter_id;
    m_msg.inter_type = pcmd->ctrlInfo.inter_type;
    m_msg.inter_state = pcmd->ctrlInfo.inter_state;
    m_msg.inter_uid = pcmd->i_uid;
    SMLK_LOGD("[WifiMsg]={id=%d type=%d state=%d uid=%d}", m_msg.inter_id, m_msg.inter_type, m_msg.inter_state, m_msg.inter_uid);
    switch (CCMsgFrom(m_msg.inter_id))
    {
    case CCMsgFrom::C_EVENT_VOLUME_CTRL_MSG:
    case CCMsgFrom::C_EVENT_SOUND_CTRL_MSG:
    case CCMsgFrom::C_EVENT_LCD_CTRL_MSG:
    case CCMsgFrom::C_EVENT_AUDIO_CTRL_MSG:
        ProcessIVIReq(m_msg);
        break;
    case CCMsgFrom::C_EVENT_BCM_CTRL_MSG:
    case CCMsgFrom::C_EVENT_DCM_CTRL_MSG:
    case CCMsgFrom::C_EVENT_AIRCONDITIONER_CTRL_MSG:
        ProcessMCUReq(m_msg);
        break;
    case CCMsgFrom::C_EVENT_RADIO_CTRL_MSG:
    {
        auto msg_len = pcmd->exts_len;
        SMLK_UINT8 *p_uint8 = (SMLK_UINT8 *)data;
        p_uint8 += sizeof(SmlkWifiIpcData);
        m_msg.exts_msg.clear();
        m_msg.exts_msg.insert(m_msg.exts_msg.end(), p_uint8, p_uint8 + msg_len);
        std::string str_msg_cont;
        str_msg_cont.insert(str_msg_cont.end(), p_uint8, p_uint8 + msg_len);
        ProcessIVIReq(m_msg);
        break;
    }
    default:
        break;
    }
}

void CloseCtrlService::ProcessWifiSvcStateQuery(IN void *data, IN int len)
{
    SMLK_LOGD("********QUERY START********");
    SmlkWifiIpcData *pcmd = (SmlkWifiIpcData *)data;
    if (pcmd->cmd_type != SmlkWifiIpcCmdType::WIFI_CMD_STATE_QUERY)
        return;
    bool isAddModeSta = SMLK_FALSE; /*用于判断是否往vd列表中添加原车模式状态查询*/
    std::vector<VehicleData> vd_query;
    /*把WIFI SVC的查询信息转换成VD进行查询*/
    for (auto i = 0; i < pcmd->queryInfo.query_num; ++i)
    {
        VehicleData vd;
        SMLK_LOGD("%dth Rcv query from WIFI SVC=%d", i, pcmd->queryInfo.query_cont[i]);
        /*如果查询序号是原车出风模式或原车强制除霜状态 并且 当前未添加过VD原车模式状态*/
        if ((((SmlkStateQueryCCtrl)pcmd->queryInfo.query_cont[i] == SmlkStateQueryCCtrl::QUERY_AIRCON_OUTLET_MODE) || ((SmlkStateQueryCCtrl)pcmd->queryInfo.query_cont[i] == SmlkStateQueryCCtrl::QUERY_AIRCON_DEFROST_SWITCH)) && (!isAddModeSta))
        {
            isAddModeSta = SMLK_TRUE;
            vd.index = VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS;
            vd_query.push_back(vd);
            // SMLK_LOGD("%dth Query Vd=%d",i,vd_query[i].index);
            continue;
        }
        else if (m_wifimcu_vd_map.find((SmlkStateQueryCCtrl)pcmd->queryInfo.query_cont[i]) != m_wifimcu_vd_map.end()) /*除了原车出风模式或原车强制除霜状态*/
        {
            auto vd_info = m_wifimcu_vd_map.find((SmlkStateQueryCCtrl)pcmd->queryInfo.query_cont[i]);
            vd.index = vd_info->second;
            vd_query.push_back(vd);
            // SMLK_LOGD("%dth Query Vd=%d",i,vd_query[i].index);
        }
    }
    /*调用VD接口进行状态查询*/
    VehicleDataApi::GetInstance()->GetVehicleData(vd_query);
    VehicleData vd_mode_status; //用于记录vd中原车空调出风模式
    CctrlGetCmd getcmd;
    for (auto i = 0; i < vd_query.size(); ++i)
    {
        if (vd_query[i].index == VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS)
        {
            vd_mode_status = vd_query[i];
        }
        // SMLK_LOGD("%dth VD id=%d | val=%g | valid=%d",i,vd_query[i].index,vd_query[i].value,vd_query[i].valid);
    }
    /*对VD查询结果生成json回传WIFI*/
    Json::Value resp_json;
    Json::FastWriter js_fw;
    for (auto i = 0; i < pcmd->queryInfo.query_num; ++i)
    {
        Json::Value resp_temp;
        resp_temp["type"] = pcmd->queryInfo.query_cont[i];
        if ((SmlkStateQueryCCtrl)pcmd->queryInfo.query_cont[i] == SmlkStateQueryCCtrl::QUERY_AIRCON_OUTLET_MODE)
        {
            VdToCCtrlValueMap(VehicleDataUsage::VD_USE_FOR_QUERY, vd_mode_status.index, vd_mode_status.value, vd_mode_status.valid, getcmd);
            SMLK_UINT8 outlet_mode = (SMLK_UINT8)((VehicleDataOutletMode)getcmd.query_val == VehicleDataOutletMode::VD_OUTLET_FACE) ? QueryOutletMode::QUERY_OUTLET_FACE : ((VehicleDataOutletMode)getcmd.query_val == VehicleDataOutletMode::VD_OUTLET_FACE_FEET)  ? QueryOutletMode::QUERY_OUTLET_FACE_FEET
                                                                                                                                                                       : ((VehicleDataOutletMode)getcmd.query_val == VehicleDataOutletMode::VD_OUTLET_FEET)         ? QueryOutletMode::QUERY_OUTLET_FEET
                                                                                                                                                                       : ((VehicleDataOutletMode)getcmd.query_val == VehicleDataOutletMode::VD_OUTLET_FEET_DEFROST) ? QueryOutletMode::QUERY_OUTLET_FEET_DEFROST
                                                                                                                                                                                                                                                                    : QUERY_INVALID;
            resp_temp["state_val"] = outlet_mode;
        }
        else if ((SmlkStateQueryCCtrl)pcmd->queryInfo.query_cont[i] == SmlkStateQueryCCtrl::QUERY_AIRCON_DEFROST_SWITCH)
        {
            VdToCCtrlValueMap(VehicleDataUsage::VD_USE_FOR_QUERY, vd_mode_status.index, vd_mode_status.value, vd_mode_status.valid, getcmd);
            SMLK_UINT8 def_switch = (SMLK_UINT8)((VehicleDataOutletMode)getcmd.query_val == VehicleDataOutletMode::VD_OUTLET_DEFROST) ? QuerySwitch::QUERY_SWITCH_ON : QuerySwitch::QUERY_SWITCH_OFF;
            resp_temp["state_val"] = def_switch;
        }
        else
        {
            auto map_iter = m_wifimcu_vd_map.find((SmlkStateQueryCCtrl)pcmd->queryInfo.query_cont[i]);
            VehicleData vd;
            for (auto iter = vd_query.begin(); iter != vd_query.end(); ++iter)
            {
                if (iter->index == map_iter->second)
                {
                    vd.index = iter->index;
                    vd.value = iter->value;
                    vd.valid = iter->valid;
                    break;
                }
                continue;
            }
            VdToCCtrlValueMap(VehicleDataUsage::VD_USE_FOR_QUERY, vd.index, vd.value, vd.valid, getcmd);
            if ((SmlkStateQueryCCtrl)pcmd->queryInfo.query_cont[i] == SmlkStateQueryCCtrl::QUERY_AIRCON_OUTLET_TEMP ||
                (SmlkStateQueryCCtrl)pcmd->queryInfo.query_cont[i] == SmlkStateQueryCCtrl::QUERY_P_AIRCON_OUTLET_TEMP)
            {
                resp_temp["state_val"] = getcmd.query_val;
            }
            else
            {
                resp_temp["state_val"] = (SMLK_UINT8)getcmd.query_val;
            }
        }
        resp_json.append(resp_temp);
    }
    std::string strJson = js_fw.write(resp_json);
    SMLK_LOGD("Resp str=%s", strJson.c_str());
    SmlkVehicleQueryResp query_resp;
    memset(&query_resp, 0x0, sizeof(SmlkVehicleQueryResp));
    query_resp.i_uid = pcmd->i_uid;
    query_resp.datalen = strJson.length();
    strcpy(query_resp.data, strJson.c_str());
    smartlink::Wifi::WifiServiceApi::getInstance()->Send2WifiService(SmlkWifiEventId::WIFI_EVENT_STATUS_QUERY_RESP, (SMLK_UINT8 *)&query_resp, sizeof(SmlkVehicleQueryResp::i_uid) + sizeof(SmlkVehicleQueryResp::datalen) + strJson.length());
    SMLK_LOGD("********QUERY QUIT********");
}

/*****************************************************************************
 *                                IVI控制                                     *
 *****************************************************************************/
SMLK_RC CloseCtrlService::ProcessIVIReq(CCtrlInterInfo &wifi_whole_msg)
{
    ControlMsg wifi_ctrl_msg;
    ControlMsgToIVI ivi_ctrl_msg;
    ControlMsgToIVIpower ivi_power_msg;

    wifi_ctrl_msg.type = wifi_whole_msg.inter_type;
    wifi_ctrl_msg.state = wifi_whole_msg.inter_state;
    ivi_ctrl_msg.cmd = 0x50;
    ivi_power_msg.cmd = 0x03;

    switch (CCMsgFrom(wifi_whole_msg.inter_id))
    {
    case CCMsgFrom::C_EVENT_VOLUME_CTRL_MSG:
        ivi_ctrl_msg.id = CCTRL_CMD_VOLUME_CTRL;
        ivi_ctrl_msg.msg = FindIviCmd(wifi_ctrl_msg, iviCtrlVolumeArry);
        break;
    case CCMsgFrom::C_EVENT_SOUND_CTRL_MSG:
        ivi_ctrl_msg.id = CCTRL_CMD_VOLUME_CTRL;
        ivi_ctrl_msg.msg = FindIviCmd(wifi_ctrl_msg, iviCtrlSoundArry);
        break;
    case CCMsgFrom::C_EVENT_LCD_CTRL_MSG:
        if (wifi_ctrl_msg.type == 1)
        {
            ivi_power_msg.id = FindIviCmd(wifi_ctrl_msg, iviCtrlLCDArry);
        }
        else
        {
            ivi_ctrl_msg.id = CCTRL_CMD_LCD_CTRL;
            ivi_ctrl_msg.msg = FindIviCmd(wifi_ctrl_msg, iviCtrlLCDArry);
        }
        break;
    case CCMsgFrom::C_EVENT_RADIO_CTRL_MSG:
        ivi_ctrl_msg.id = CCTRL_CMD_RADIO_CTRL;
        ivi_ctrl_msg.msg = FindIviCmd(wifi_ctrl_msg, iviCtrlRadioArry);
        break;
    case CCMsgFrom::C_EVENT_AUDIO_CTRL_MSG:
        ivi_ctrl_msg.id = CCTRL_CMD_AUDIO_CTRL;
        ivi_ctrl_msg.msg = FindIviCmd(wifi_ctrl_msg, iviCtrlAudioArry);
        break;
    default:
        return SMLK_RC::RC_ERROR;
    }

    std::vector<SMLK_UINT8> tbox2ivi_msg;
    tbox2ivi_msg.clear();
    if ((CCMsgFrom(wifi_whole_msg.inter_id) == CCMsgFrom::C_EVENT_LCD_CTRL_MSG) && (wifi_ctrl_msg.type == 1))
    {
        tbox2ivi_msg.insert(tbox2ivi_msg.end(), (SMLK_UINT8 *)&ivi_power_msg, (SMLK_UINT8 *)&ivi_power_msg + sizeof(ivi_power_msg));
    }
    else
    {
        tbox2ivi_msg.insert(tbox2ivi_msg.end(), (SMLK_UINT8 *)&ivi_ctrl_msg, (SMLK_UINT8 *)&ivi_ctrl_msg + sizeof(ControlMsgToIVI));
        if (wifi_whole_msg.exts_msg.size() > 0)
        {
            SMLK_UINT16 msg_len = wifi_whole_msg.exts_msg.size();
            msg_len = htobe16(msg_len);
            tbox2ivi_msg.insert(tbox2ivi_msg.end(), (SMLK_UINT8 *)&msg_len, (SMLK_UINT8 *)&msg_len + sizeof(SMLK_UINT16));
            tbox2ivi_msg.insert(tbox2ivi_msg.end(), wifi_whole_msg.exts_msg.begin(), wifi_whole_msg.exts_msg.end());
        }
    }
    std::string ivi_log = tools::uint8_2_string(tbox2ivi_msg.data(), tbox2ivi_msg.size());
    SMLK_LOGD("[Tbox2Ivi][Id:0x%02x][Info:0x%02x]=%s", ivi_ctrl_msg.id, ivi_ctrl_msg.msg, ivi_log.c_str());
    smartlink_sdk::IVI::GetInstance()->SendRawDataAsync(reinterpret_cast<void *>(tbox2ivi_msg.data()), tbox2ivi_msg.size(), m_ctrl_ivi_cb);
    unique_lock<std::mutex> lck(m_ivi_mtx);
    SMLK_LOGD("Waiting for 2s");
    CCtrlCmdResult m_ivi_res;
    if (m_ivi.wait_for(lck, std::chrono::seconds(2)) == std::cv_status::timeout)
    {
        SMLK_LOGD("[IviResp] timeout!");
        m_ivi_res = CCTRL_CMD_RESULT_TIMEOUT;
    }
    else
    {
        m_ivi_res = m_cctrl_cmd_result;
        SMLK_LOGD("[IviRespResult]=%d.", m_ivi_res);
    }
    SmlkVehicleCtrlResp cmd_resp;
    HandleRequestResult(wifi_whole_msg, m_ivi_res, cmd_resp);
    SMLK_UINT8 *s_cmd_resp = (SMLK_UINT8 *)&cmd_resp;
    smartlink::Wifi::WifiServiceApi::getInstance()->Send2WifiService(SmlkWifiEventId::WIFI_EVENT_VEHICLE_CTRL_RESP, s_cmd_resp, sizeof(SmlkVehicleCtrlResp));
    SMLK_LOGD("********Ivi Ctrl End********");
    return SMLK_RC::RC_OK;
}

size_t CloseCtrlService::FindIviCmd(ControlMsg volume, IviControlMsg *iviCtrlArry)
{
    for (int idx = 0; iviCtrlArry[idx].volume.type != 0xff; idx++)
    {
        if ((volume.type == iviCtrlArry[idx].volume.type) && (volume.state == iviCtrlArry[idx].volume.state))
        {
            return iviCtrlArry[idx].cmd;
        }
    }
    return 0xff;
}

void CloseCtrlService::handleIviCtrResult(void *data, int len)
{
    SMLK_UINT8 *p_byte = (SMLK_UINT8 *)data;
    SMLK_UINT8 ivi_resp_type = (SMLK_UINT8)*p_byte;
    switch (ivi_resp_type)
    {
    case SmlkCctrlIviCmdType::SMLK_CCTRL_IVI_CMD_POWER:
    {
        IviPowerCtrlResult *power_res = (IviPowerCtrlResult *)data;
        std::string str_ivi_resp = tools::uint8_2_string((SMLK_UINT8 *)data, len);
        SMLK_LOGD("[Ivi2Tbox][Power]=%s", str_ivi_resp.c_str());
        if (power_res->result == (SMLK_UINT8)IviCmdCommonResult::SMLK_IVI_CMD_NO_EXE)
        {
            m_cctrl_cmd_result = CCTRL_CMD_RESULT_ERROR;
        }
        else if (power_res->result == (SMLK_UINT8)IviCmdCommonResult::SMLK_IVI_CMD_EXE)
        {
            m_cctrl_cmd_result = CCTRL_CMD_RESULT_OK;
        }
        m_ivi.notify_one();
        break;
    }
    case SmlkCctrlIviCmdType::SMLK_CCTRL_IVI_CMD_MEDIA:
    {
        IviMediaCtrlResult *media_res = (IviMediaCtrlResult *)data;
        std::string str_ivi_resp = tools::uint8_2_string((SMLK_UINT8 *)data, len);
        SMLK_LOGD("[Ivi2Tbox][Media]=%s", str_ivi_resp.c_str());
        if (media_res->act_res == (SMLK_UINT8)IviCmdCommonResult::SMLK_IVI_CMD_NO_EXE)
        {
            m_cctrl_cmd_result = CCTRL_CMD_RESULT_ERROR;
        }
        else if (media_res->act_res == (SMLK_UINT8)IviCmdCommonResult::SMLK_IVI_CMD_EXE)
        {
            m_cctrl_cmd_result = CCTRL_CMD_RESULT_OK;
        }
        m_ivi.notify_one();
        break;
    }
    default:
        return;
    }
}

/*****************************************************************************
 *                                MCU控制                                     *
 *****************************************************************************/
SMLK_RC CloseCtrlService::ProcessMCUReq(CCtrlInterInfo &cctrl_msg)
{
    CctrlSetCmd m_setCmd_vec;
    CCtrlCmdResult m_setCmd_result;

    switch (CCMsgFrom(cctrl_msg.inter_id))
    {
    case CCMsgFrom::C_EVENT_BCM_CTRL_MSG:
        if (cctrl_msg.inter_type == 0)
        {
            m_setCmd_vec.cmd_id = CCTRL_CMD_DRIVER_LIGHT;
        }
        else if (cctrl_msg.inter_type == 1)
        {
            m_setCmd_vec.cmd_id = CCTRL_CMD_COPILOT_LIGHT;
        }
        else
        {
            m_setCmd_vec.cmd_id = CCTRL_CMD_LIGHT_CTRL;
        }
        m_setCmd_vec.action = CCTRL_ACTION_INVALID;
        break;
    case CCMsgFrom::C_EVENT_DCM_CTRL_MSG:
        if (cctrl_msg.inter_type == 0)
        {
            m_setCmd_vec.cmd_id = CCTRL_CMD_DRIVER_WINDOW;
        }
        else if (cctrl_msg.inter_type == 1)
        {
            m_setCmd_vec.cmd_id = CCTRL_CMD_COPILOT_WINDOW;
        }
        if (cctrl_msg.inter_state == 0)
        {
            m_setCmd_vec.action = CCTRL_ACTION_DOWN;
        }
        else if (cctrl_msg.inter_state == 1)
        {
            m_setCmd_vec.action = CCTRL_ACTION_UP;
        }
        break;
    case CCMsgFrom::C_EVENT_AIRCONDITIONER_CTRL_MSG:
        m_setCmd_vec.cmd_id = i_McuCmdMsg[cctrl_msg.inter_type];
        switch (m_setCmd_vec.cmd_id)
        {
        case CCTRL_CMD_AC_SWITCH:
        case CCTRL_CMD_AC_AIROUTLET_MODE:
        case CCTRL_CMD_AC_COMPRESSOR:
        case CCTRL_CMD_AC_AUTO_SWITCH:
        case CCTRL_CMD_INDEPT_WARM_AIR:
        case CCTRL_CMD_PARKING_AC_SWITCH:
        case CCTRL_CMD_PARKING_AC_AUTO:
        case CCTRL_CMD_AC_VENTILATION:
        case CCTRL_CMD_AC_DEFROST:
            m_setCmd_vec.action = cctrl_msg.inter_state + 1;
            break;
        case CCTRL_CMD_AC_LOOP_MOOD:
            if (cctrl_msg.inter_state == 0)
                m_setCmd_vec.action = CCTRL_MCU_ACTION_AC_INNER_LOOP;
            else if (cctrl_msg.inter_state == 2)
                m_setCmd_vec.action = CCTRL_MCU_ACTION_AC_OUTER_LOOP;
            break;
        /*原车,驻车空调温度设置需要采集数据*/
        case CCTRL_CMD_AC_TEMPER:
        case CCTRL_CMD_PARKING_AC_TEMPER:
        {
            VehicleData vd;
            auto iter = m_cctl_vd_map.find(m_setCmd_vec.cmd_id);
            if (iter == m_cctl_vd_map.end())
            {
                SMLK_LOGE("Can not find cctrl cmd=%d in m_cctl_vd_map.", m_setCmd_vec.cmd_id);
                return SMLK_RC::RC_ERROR;
            }
            vd.index = iter->second;
            VehicleDataApi::GetInstance()->GetVehicleData(vd);
            CctrlGetCmd getcmd;
            VdToCCtrlValueMap(VehicleDataUsage::VD_USE_FOR_CTRL, vd.index, vd.value, vd.valid, getcmd);
            if (cctrl_msg.inter_state == 0)
            {
                m_setCmd_vec.data = (SMLK_UINT16)((getcmd.value - 0.5) * 10);
            }
            else
            {
                m_setCmd_vec.data = (SMLK_UINT16)((getcmd.value + 0.5) * 10);
            }
            m_setCmd_vec.action = CCTRL_ACTION_INVALID;
        }
        break;
        /*原车,驻车风量,独暖温度设置需要采集数据*/
        case CCTRL_CMD_AC_WIND:
        case CCTRL_CMD_INDEPT_WARM_AIR_TEMPER:
        case CCTRL_CMD_PARKING_AC_WIND:
        {
            VehicleData vd;
            auto iter = m_cctl_vd_map.find(m_setCmd_vec.cmd_id);
            if (iter == m_cctl_vd_map.end())
            {
                SMLK_LOGE("Can not find cctrl cmd=%d in m_cctl_vd_map.", m_setCmd_vec.cmd_id);
                return SMLK_RC::RC_ERROR;
            }
            vd.index = iter->second;
            VehicleDataApi::GetInstance()->GetVehicleData(vd);
            CctrlGetCmd getcmd;
            VdToCCtrlValueMap(VehicleDataUsage::VD_USE_FOR_CTRL, vd.index, vd.value, vd.valid, getcmd);
            if (((cctrl_msg.inter_type == 3) || (cctrl_msg.inter_type == 12)) && (getcmd.query_val == (SMLK_DOUBLE)AirConditionerBlowingRate::E_AIR_CON_LEVEL0)) /*设置原车or驻车空调风量且当前风量最低档*/
            {
                m_setCmd_vec.data = (int)AirConditionerBlowingRate::E_AIR_CON_LEVEL0;
            }
            else if (((cctrl_msg.inter_type == 3) || (cctrl_msg.inter_type == 12)) && (getcmd.query_val == (SMLK_DOUBLE)AirConditionerBlowingRate::E_AIR_CON_LEVEL13)) /*设置原车or驻车空调风量且当前风量最高档*/
            {
                m_setCmd_vec.data = (int)AirConditionerBlowingRate::E_AIR_CON_LEVEL13;
            }
            else if ((cctrl_msg.inter_type == 9) && (getcmd.query_val == (SMLK_DOUBLE)WarmAirTempLevel::E_WARM_AIR_INACTIVE)) /*设置独暖温度且当前风量最低档*/
            {
                m_setCmd_vec.data = (int)WarmAirTempLevel::E_WARM_AIR_INACTIVE;
            }
            else if ((cctrl_msg.inter_type == 9) && (getcmd.query_val == (SMLK_DOUBLE)WarmAirTempLevel::E_WARM_AIR_LEVEL7)) /*设置独暖温度且当前风量最低档*/
            {
                m_setCmd_vec.data = (int)WarmAirTempLevel::E_WARM_AIR_LEVEL7;
            }
            else
            {
                if (cctrl_msg.inter_state == 0)
                {
                    m_setCmd_vec.data = (SMLK_UINT8)getcmd.value - 1;
                }
                else
                {
                    m_setCmd_vec.data = (SMLK_UINT8)getcmd.value + 1;
                }
            }
            m_setCmd_vec.action = CCTRL_ACTION_INVALID;
        }
        break;
        default:
            return SMLK_RC::RC_ERROR;
        }
        break;
    default:
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGD("WIFI{id=%d;type=%d;state=%d} -> CCtrl{id=%d;action=%02x,data=%g}",
              cctrl_msg.inter_id, cctrl_msg.inter_type, cctrl_msg.inter_state,
              m_setCmd_vec.cmd_id, m_setCmd_vec.action, m_setCmd_vec.data);
    ProcessCctrlCmdToMCU(m_setCmd_vec, m_setCmd_result);
    SmlkVehicleCtrlResp cmd_resp;
    HandleRequestResult(cctrl_msg, m_setCmd_result, cmd_resp);
    SMLK_UINT8 *s_cmd_resp = (SMLK_UINT8 *)&cmd_resp;
    smartlink::Wifi::WifiServiceApi::getInstance()->Send2WifiService(SmlkWifiEventId::WIFI_EVENT_VEHICLE_CTRL_RESP, s_cmd_resp, sizeof(SmlkVehicleCtrlResp));
    SMLK_LOGD("********CCTRL END********");
    return SMLK_RC::RC_OK;
}

/**
 * @brief
 * @param vehicle_index     vd数据id
 * @param vehicle_data      vd数据
 * @param vechile_valid     vd数据有效性
 * @param cctrl_get_cmd     近控数据采集,用作MCU控制指令下发&查询结果转换
 *
 * @return 吉大错误码
 */
SMLK_RC CloseCtrlService::VdToCCtrlValueMap(IN VehicleDataUsage vd_usage, IN SMLK_UINT32 vehicle_index, IN SMLK_DOUBLE vehicle_data, IN bool vechile_valid, OUT CctrlGetCmd &cctrl_get_cmd)
{
    /**/
    if (vd_usage == VehicleDataUsage::VD_USE_FOR_CTRL)
    {
        auto iter = m_vd_cctl_map.find(vehicle_index);
        cctrl_get_cmd.cmd_id = iter->second;
    }
    /*校验数据的合法性*/
    if (!vechile_valid)
    {
        cctrl_get_cmd.value = CCTRL_STATUS_INVALID;
        cctrl_get_cmd.query_val = QUERY_INVALID;
        SMLK_LOGD("VD idx=%d val=%g -> MCU val=%g", vehicle_index, vehicle_data, cctrl_get_cmd.value);
        return SMLK_RC::RC_ERROR;
    }
    switch (vehicle_index)
    {
    /******************************************************
     *                      查询部分                       *
     ******************************************************/
    case VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS:         /*原车空调开关        0:OFF 1:ON*/
    case VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS:        /*原车空调压缩机状态  0:OFF 1:ON*/
    case VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS:      /*原车空调自动状态    0:OFF 1:ON*/
    case VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS:    /*原车空调一键通风状态    0:OFF 1:ON*/
    case VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS:         /*独立暖风开关状态    0:OFF 1:ON*/
    case VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS: /*驻车空调开关状态    0:OFF 1:ON*/
    case VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS:           /*驻车空调自动模式    0:OFF 1:ON*/
        // case VEHICLE_DATA_CAB_TEMPERATURE:               /*保留 驾驶室温度*/
        {
            cctrl_get_cmd.query_val = ((SMLK_UINT8)vehicle_data == VehicleDataSwitch::VD_SWITCH_OFF) ? QuerySwitch::QUERY_SWITCH_OFF : QuerySwitch::QUERY_SWITCH_ON;
            break;
        }
    case VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS: /*原车空调出风模式      0:吹面 1:吹面吹脚 2:吹脚 3:吹脚除霜 4:强制除霜*/
    {
        cctrl_get_cmd.query_val = (SMLK_UINT8)vehicle_data;
        break;
    }
    case VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS: /*原车空调循环状态    0:内循环 1:外循环*/
    {
        cctrl_get_cmd.query_val = ((SMLK_UINT8)vehicle_data == VehicleDataCirculationMode::VD_CIRCULATION_INNER) ? QueryCirculationMode::QUERY_CIRCULATION_INNER : QueryCirculationMode::QUERY_CIRCULATION_OUTER;
        break;
    }
    /******************************************************
     *                    查询+控车部分                     *
     ******************************************************/
    case VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE:         /*原车空调温度°C  17~33 步长0.5*/
    case VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE: /*驻车空调温度°C  17~33 步长0.5*/
    {
        cctrl_get_cmd.value = vehicle_data;
        cctrl_get_cmd.query_val = vehicle_data;
        break;
    }
    case VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE: /*原车空调风量状态  0~13:Level 0~13*/
    {
        if (((AirConditionerBlowingRate)vehicle_data >= AirConditionerBlowingRate::E_AIR_CON_LEVEL0) && ((AirConditionerBlowingRate)vehicle_data <= AirConditionerBlowingRate::E_AIR_CON_LEVEL13))
        {
            cctrl_get_cmd.value = (SMLK_UINT8)vehicle_data;
            cctrl_get_cmd.query_val = (SMLK_UINT8)vehicle_data;
        }
        else
        {
            cctrl_get_cmd.value = CCTRL_STATUS_INVALID;
            cctrl_get_cmd.query_val = QUERY_INVALID;
        }
        break;
    }
    case VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE: /*驻车空调风量  0~13:Level 0~13*/
    {
        if (((ParkAirConditionerBlowingRate)vehicle_data >= ParkAirConditionerBlowingRate::E_PARK_AIR_CON_LEVEL0) && ((ParkAirConditionerBlowingRate)vehicle_data <= ParkAirConditionerBlowingRate::E_PARK_AIR_CON_LEVEL13))
        {
            cctrl_get_cmd.value = SMLK_UINT8(vehicle_data);
            cctrl_get_cmd.query_val = (SMLK_UINT8)vehicle_data;
        }
        else
        {
            cctrl_get_cmd.value = CCTRL_STATUS_INVALID;
            cctrl_get_cmd.query_val = QUERY_INVALID;
        }
        break;
    }
    case VEHICLE_DATA_WARM_AIR_TEMPERATURE: /*独立暖风温度  0:未开启 1~7:Level1~7*/
    {
        if (((WarmAirTempLevel)vehicle_data >= WarmAirTempLevel::E_WARM_AIR_INACTIVE) && ((WarmAirTempLevel)vehicle_data <= WarmAirTempLevel::E_WARM_AIR_LEVEL7))
        {
            cctrl_get_cmd.value = (SMLK_UINT8)vehicle_data;
            cctrl_get_cmd.query_val = (SMLK_UINT8)vehicle_data;
        }
        else
        {
            cctrl_get_cmd.value = CCTRL_STATUS_INVALID;
            cctrl_get_cmd.query_val = QUERY_INVALID;
        }
        break;
    }
    default:
        break;
    }
    if (vd_usage == VehicleDataUsage::VD_USE_FOR_QUERY)
    {
        SMLK_LOGD("QUERY: VD{idx=%d val=%g}->WIFI{val=%g}", vehicle_index, vehicle_data, cctrl_get_cmd.query_val);
    }
    else
    {
        SMLK_LOGD("CTRL: VD{idx=%d val=%g}->MCU{val=%g}", vehicle_index, vehicle_data, cctrl_get_cmd.value);
    }
    return SMLK_RC::RC_OK;
}

SMLK_RC CloseCtrlService::ProcessCctrlCmdToMCU(IN CctrlSetCmd &cctrl_cmd_vec, OUT CCtrlCmdResult &result)
{
    SendCctrlCmdToMcu(cctrl_cmd_vec);
    unique_lock<std::mutex> lck(m_cv_mtx);
    SMLK_LOGD("ready to wait for %ds", 5);
    if (m_cv.wait_for(lck, std::chrono::seconds(5)) == std::cv_status::timeout)
    {
        SMLK_LOGD("MCU resp TIMEOUT!");
        result = CCTRL_CMD_RESULT_TIMEOUT;
    }
    else
    {
        result = m_cctrl_cmd_result;
        SMLK_LOGD("MCU resp no timeout. Result=%d.", result);
    }
    return SMLK_RC::RC_OK;
}

SMLK_RC CloseCtrlService::SendCctrlCmdToMcu(IN CctrlSetCmd &cctrl_cmd)
{
    CctrlSetCmd key;
    key.data = CCTRL_DATA_INVALID;
    key.cmd_id = cctrl_cmd.cmd_id;
    key.action = cctrl_cmd.action;

    // SMLK_LOGD("search key(cmd_id=%d, action=%d) in m_mcu_to_jtt808_map.",cctrl_cmd.cmd_id, cctrl_cmd.action);
    auto iter = m_cctl_mcu_map.find(key);
    if (iter == m_cctl_mcu_map.end())
    {
        SMLK_LOGW("NO such pair{cmd_id =%d, action=%d}", key.cmd_id, key.action);
        return SMLK_RC::RC_ERROR;
    }
    std::vector<CCtrlSetMcuCmd> mcu_cmd_vec = iter->second;
    // SMLK_LOGD("ToMcu: cmd=%d action=0x%02x data_len=0x%02x data=%g",
    //     mcu_cmd_vec[0].cmd_id, mcu_cmd_vec[0].cmd_id, mcu_cmd_vec[0].action, mcu_cmd_vec[0].data_len, cctrl_cmd.data);

    vector<SMLK_UINT8> data_to_mcu;
    CctrlMcuCmdHead mcu_head;

    if (m_cctrl_mcu_seq == 0xffff)
    {
        m_cctrl_mcu_seq = 0x8000; /*m_cctrl_mcu_seq范围0x8000~0xffff*/
    }
    else
        m_cctrl_mcu_seq++;
    mcu_head.seq = htobe16(m_cctrl_mcu_seq);
    mcu_head.count = mcu_cmd_vec.size();
    data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_head, (SMLK_UINT8 *)&mcu_head + sizeof(CctrlMcuCmdHead));

    for (SMLK_UINT16 i = 0; i < mcu_head.count; ++i)
    {
        CCtrlSetMcuCmd mcu_cmd;
        memcpy(&mcu_cmd, &mcu_cmd_vec[i], sizeof(CCtrlSetMcuCmd));
        mcu_cmd.cmd_id = htobe16(mcu_cmd.cmd_id);
        data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_cmd, (SMLK_UINT8 *)&mcu_cmd + sizeof(CCtrlSetMcuCmd));
        if (MCU_DATA_LEN_1 == mcu_cmd.data_len)
        {
            SMLK_UINT8 data = (SMLK_UINT8)(cctrl_cmd.data);
            data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&data, (SMLK_UINT8 *)&data + sizeof(data));
        }
        else if (MCU_DATA_LEN_2 == mcu_cmd.data_len)
        {
            SMLK_UINT16 data = (SMLK_UINT16)(cctrl_cmd.data);
            data = htobe16(data);
            data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&data, (SMLK_UINT8 *)&data + sizeof(data));
        }
    }
    smartlink_sdk::MCU::GetInstance()->SendVehicleCtrlCmd(data_to_mcu);

    std::string log = tools::uint8_2_string(&data_to_mcu[0], data_to_mcu.size());
    SMLK_LOGD("Send msg=%s to mcu", log.c_str());
    return SMLK_RC::RC_OK;
}

void CloseCtrlService::OnMcuSerIndication(smartlink_sdk::McuEventId id, void *data, int length)
{
    switch (id)
    {
    case smartlink_sdk::McuEventId::E_MCU_EVENT_VEHICLE_CRTL_ACK:
        ProcessMcuSerResp(data, length);
        break;
    default:
        return;
    }
}

void CloseCtrlService::ProcessMcuSerResp(IN void *data, IN int len)
{
    std::string log = tools::uint8_2_string((SMLK_UINT8 *)data, len);
    SMLK_LOGD("Rcv resp=%s from mcu", log.c_str());

    unique_lock<std::mutex> lck(m_cv_mtx);
    if (len < (int)(sizeof(CctrlMcuCmdHead)))
    {
        SMLK_LOGE("Data len(%d) < head(%ld).", len, sizeof(CctrlMcuCmdHead));
        return;
    }
    CctrlMcuCmdHead *phead = (CctrlMcuCmdHead *)data;
    phead->seq = be16toh(phead->seq);
    if (phead->seq != m_cctrl_mcu_seq)
    {
        SMLK_LOGW("rev seq=0x%04x != send seq=0x%04x.", phead->seq, m_cctrl_mcu_seq);
        return;
    }
    CctrlSetCmdResult *presult = (CctrlSetCmdResult *)((SMLK_UINT8 *)data + sizeof(CctrlMcuCmdHead));
    if (presult->result == CCTRL_CMD_RESULT_ERROR)
    {
        m_cctrl_cmd_result = CCTRL_CMD_RESULT_ERROR;
    }
    else
    {
        m_cctrl_cmd_result = CCTRL_CMD_RESULT_OK;
    }
    m_cv.notify_one();
}

/*****************************************************************************
 *                               控车结果处理                                  *
 *****************************************************************************/
void CloseCtrlService::HandleRequestResult(IN CCtrlInterInfo cmd_msg, IN CCtrlCmdResult cmd_result, OUT smartlink::Wifi::SmlkVehicleCtrlResp &cmd_resp)
{
    cmd_resp.i_uid = (SMLK_INT32)cmd_msg.inter_uid;
    if (cmd_result == CCtrlCmdResult::CCTRL_CMD_RESULT_OK)
    {
        cmd_resp.result = (SMLK_UINT16)smartlink::Wifi::SmlkHttpResult::HTTP_RESULT_OK;
    }
    else
        cmd_resp.result = (SMLK_UINT16)smartlink::Wifi::SmlkHttpResult::HTTP_RESULT_ERROR;
}

SMLK_RC CloseCtrlService::InitCctrlToMcuMap()
{
    SMLK_LOGD("enter in constructor (%s).", __func__);
    CctrlSetCmd cctrl_cmd;
    CCtrlSetMcuCmd mcu_cmd;
    vector<CCtrlSetMcuCmd> mcu_cmd_vec;

    cctrl_cmd.data = CCTRL_DATA_INVALID;

    /*原车空调开关*/
    cctrl_cmd.cmd_id = CCTRL_CMD_AC_SWITCH;
    cctrl_cmd.action = CCTRL_ACTION_ON;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_SWITCH;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_AC_SWITCH;
    cctrl_cmd.action = CCTRL_ACTION_OFF;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_SWITCH;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*原车空调出风模式*/
    cctrl_cmd.cmd_id = CCTRL_CMD_AC_AIROUTLET_MODE;
    cctrl_cmd.action = CCTRL_ACTION_BLOW_FACE;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_AIROUTLET_MODE;
    mcu_cmd.action = CCTRL_ACTION_BLOW_FACE;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_AC_AIROUTLET_MODE;
    cctrl_cmd.action = CCTRL_ACTION_BLOW_FACE_FEET;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_AIROUTLET_MODE;
    mcu_cmd.action = CCTRL_ACTION_BLOW_FACE_FEET;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_AC_AIROUTLET_MODE;
    cctrl_cmd.action = CCTRL_ACTION_BLOW_FEET;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_AIROUTLET_MODE;
    mcu_cmd.action = CCTRL_ACTION_BLOW_FEET;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_AC_AIROUTLET_MODE;
    cctrl_cmd.action = CCTRL_ACTION_BLOW_FEET_DEFROST;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_AIROUTLET_MODE;
    mcu_cmd.action = CCTRL_ACTION_BLOW_FEET_DEFROST;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*原车空调温度*/
    cctrl_cmd.cmd_id = CCTRL_CMD_AC_TEMPER;
    cctrl_cmd.action = CCTRL_ACTION_INVALID;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_TEMPER;
    mcu_cmd.action = CCTRL_ACTION_INVALID;
    mcu_cmd.data_len = 2; /*2个字节*/
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*原车空调风量*/
    cctrl_cmd.cmd_id = CCTRL_CMD_AC_WIND;
    cctrl_cmd.action = CCTRL_ACTION_INVALID;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_WIND;
    mcu_cmd.action = CCTRL_ACTION_INVALID;
    mcu_cmd.data_len = 1; /*1个字节*/
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*原车空调循环模式：内循环，外循环*/
    cctrl_cmd.cmd_id = CCTRL_CMD_AC_LOOP_MOOD;
    cctrl_cmd.action = CCTRL_ACTION_AC_OUTER_LOOP;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_LOOP_MOOD;
    mcu_cmd.action = CCTRL_MCU_ACTION_AC_OUTER_LOOP;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_AC_LOOP_MOOD;
    cctrl_cmd.action = CCTRL_ACTION_AC_INNER_LOOP;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_LOOP_MOOD;
    mcu_cmd.action = CCTRL_MCU_ACTION_AC_INNER_LOOP;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*原车空调压缩机*/
    cctrl_cmd.cmd_id = CCTRL_CMD_AC_COMPRESSOR;
    cctrl_cmd.action = CCTRL_ACTION_ON;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_COMPRESSOR;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_AC_COMPRESSOR;
    cctrl_cmd.action = CCTRL_ACTION_OFF;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_COMPRESSOR;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*原车空调auto开关*/
    cctrl_cmd.cmd_id = CCTRL_CMD_AC_AUTO_SWITCH;
    cctrl_cmd.action = CCTRL_ACTION_ON;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_AUTO_SWITCH;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_AC_AUTO_SWITCH;
    cctrl_cmd.action = CCTRL_ACTION_OFF;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_AUTO_SWITCH;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*原车空调除霜开关*/
    cctrl_cmd.cmd_id = CCTRL_CMD_AC_DEFROST;
    cctrl_cmd.action = CCTRL_ACTION_ON;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_DEFROST;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_AC_DEFROST;
    cctrl_cmd.action = CCTRL_ACTION_OFF;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_DEFROST;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*原车空调一键通风开关*/
    cctrl_cmd.cmd_id = CCTRL_CMD_AC_VENTILATION;
    cctrl_cmd.action = CCTRL_ACTION_ON;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_VENTILATION;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_AC_VENTILATION;
    cctrl_cmd.action = CCTRL_ACTION_OFF;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_AC_VENTILATION;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*独立暖风开关*/
    cctrl_cmd.cmd_id = CCTRL_CMD_INDEPT_WARM_AIR;
    cctrl_cmd.action = CCTRL_ACTION_ON;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_INDEPT_WARM_AIR;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_INDEPT_WARM_AIR;
    cctrl_cmd.action = CCTRL_ACTION_OFF;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_INDEPT_WARM_AIR;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*独立暖风温度*/
    cctrl_cmd.cmd_id = CCTRL_CMD_INDEPT_WARM_AIR_TEMPER;
    cctrl_cmd.action = CCTRL_ACTION_INVALID;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_INDEPT_WARM_AIR_TEMPER;
    mcu_cmd.action = CCTRL_ACTION_INVALID;
    mcu_cmd.data_len = 1;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*驻车空调开关*/
    cctrl_cmd.cmd_id = CCTRL_CMD_PARKING_AC_SWITCH;
    cctrl_cmd.action = CCTRL_ACTION_ON;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_PARKING_AC_SWITCH;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_PARKING_AC_SWITCH;
    cctrl_cmd.action = CCTRL_ACTION_OFF;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_PARKING_AC_SWITCH;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*驻车空调AUTO开关*/
    cctrl_cmd.cmd_id = CCTRL_CMD_PARKING_AC_AUTO;
    cctrl_cmd.action = CCTRL_ACTION_ON;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_PARKING_AC_AUTO;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_PARKING_AC_AUTO;
    cctrl_cmd.action = CCTRL_ACTION_OFF;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_PARKING_AC_AUTO;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*驻车空调温度*/
    cctrl_cmd.cmd_id = CCTRL_CMD_PARKING_AC_TEMPER;
    cctrl_cmd.action = CCTRL_ACTION_INVALID;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_PARKING_AC_TEMPER;
    mcu_cmd.action = CCTRL_ACTION_INVALID;
    mcu_cmd.data_len = 2;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*驻车空调风量*/
    cctrl_cmd.cmd_id = CCTRL_CMD_PARKING_AC_WIND;
    cctrl_cmd.action = CCTRL_ACTION_INVALID;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_PARKING_AC_WIND;
    mcu_cmd.action = CCTRL_ACTION_INVALID;
    mcu_cmd.data_len = 1;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*室内灯控制*/
    cctrl_cmd.cmd_id = CCTRL_CMD_LIGHT_CTRL;
    cctrl_cmd.action = CCTRL_ACTION_INVALID;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_LIGHT_CTRL;
    mcu_cmd.action = CCTRL_ACTION_INVALID;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*主驾侧阅读灯*/
    cctrl_cmd.cmd_id = CCTRL_CMD_DRIVER_LIGHT;
    cctrl_cmd.action = CCTRL_ACTION_INVALID;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_DRIVER_LIGHT;
    mcu_cmd.action = CCTRL_ACTION_INVALID;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*副驾侧阅读灯*/
    cctrl_cmd.cmd_id = CCTRL_CMD_COPILOT_LIGHT;
    cctrl_cmd.action = CCTRL_ACTION_INVALID;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_COPILOT_LIGHT;
    mcu_cmd.action = CCTRL_ACTION_INVALID;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*主驾车窗*/
    cctrl_cmd.cmd_id = CCTRL_CMD_DRIVER_WINDOW;
    cctrl_cmd.action = CCTRL_ACTION_DOWN;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_DRIVER_WINDOW;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_DRIVER_WINDOW;
    cctrl_cmd.action = CCTRL_ACTION_UP;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_DRIVER_WINDOW;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    /*副驾车窗*/
    cctrl_cmd.cmd_id = CCTRL_CMD_COPILOT_WINDOW;
    cctrl_cmd.action = CCTRL_ACTION_DOWN;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_COPILOT_WINDOW;
    mcu_cmd.action = CCTRL_MCU_ACTION_ON;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    cctrl_cmd.cmd_id = CCTRL_CMD_COPILOT_WINDOW;
    cctrl_cmd.action = CCTRL_ACTION_UP;

    mcu_cmd_vec.clear();
    mcu_cmd.cmd_id = CCTRL_CMD_COPILOT_WINDOW;
    mcu_cmd.action = CCTRL_MCU_ACTION_OFF;
    mcu_cmd.data_len = 0;
    mcu_cmd_vec.push_back(mcu_cmd);

    m_cctl_mcu_map.insert(make_pair(cctrl_cmd, mcu_cmd_vec));

    return SMLK_RC::RC_OK;
}
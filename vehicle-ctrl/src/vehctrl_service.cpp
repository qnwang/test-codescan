/*****************************************************************************/
/**
 * \file       vehicle_control.cpp
 * \version    Tbox2.0 V1
 * \brief      vehicle control service
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 ******************************************************************************/

#include "vehctrl_service.h"
#include <dlfcn.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <unistd.h>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include "vehctrl_configfile.h"
#include "vehctrl_jtt808.h"
#include "vehctrl_jtt808_getset_param.h"
#include "vehctrl_jtt808_msg8f41.h"
#include "vehctrl_jtt808_msg8f42.h"
#include "vehctrl_jtt808_msg8f51.h"
#include "vehctrl_queue.h"
#include "vehctrl_status_file.h"
#include "smartlink_sdk_mcu.h"
#include "smartlink_sdk_sys_property.h"
#include "smartlink_sdk_wlan.h"
#include "smlk_log.h"
#include "smlk_property.h"
#include "smlk_spi_manager.h"
#include "tsp_service_api.h"
#include "vehicle_data_def.h"
#include "vehicle_data_index_def.h"

using namespace std;
using namespace smartlink;
using namespace smartlink_sdk;
using namespace smartlink::CAN;
typedef unsigned char byte;
typedef long DWORD;

static RemoteCtrlCmdLoad cmd_load;
SMLK_UINT8 g_rctrl_protocol = CUR_PROT_JTT808;

/*Mcu响应与Tsp响应映射表*/
const static std::map<SmlkMcuCmdResult, SmlkRctrl_8F41_Result> m_mcu_tsp_resp_map = {
    {SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_SUCCESS, SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_OK},
    {SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_ERROR, SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_ERROR},
    {SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_BATTERY_LOW_NO_SP, SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_BATTERY_LOW},
    {SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_NO_SP, SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_SIGNAL_NOT_SP},
    {SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_PS_AUTH_FAIL, SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_PS_AUTH_FAIL},
    {SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_VEHICLE_NO_AVALIABLE, SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_RCTRL_NOT_SP},
};

static void FindErrcodeInConfigfile(IN SmlkRctrl_8F41_Result m_errcode, IN vector<RemoteCtrlErrRetryInfo> &vec, OUT SMLK_UINT8 &location)
{
    SMLK_LOGD("enter in func(%s).", __func__);
    SMLK_BOOL find_flag = false;
    for (SMLK_UINT8 i = 0; i < vec.size(); i++)
    {
        if (m_errcode == vec[i].errcode)
        {
            location = i;
            find_flag = true;
            SMLK_LOGD("find errcode=%d, location=%d.", m_errcode, location);
        }
    }
    if (!find_flag)
    {
        location = vec.size();
        SMLK_LOGD("can not find errcode=%d.", m_errcode);
    }
    SMLK_LOGD("find_flag=%d, location=%d.", find_flag, location);
}

/*****************************************************************************
 *                                进程管理                                    *
 *****************************************************************************/
/*构造析构*/
VehicleCtrlService::VehicleCtrlService()
    : m_sp_awaken_evt(new Poco::Event), m_sp_offlinectrl_evt(new Poco::Event), m_sp_can_bus_wakeup_evt(new Poco::Event)
{
    m_flags = IService::Uninitialized;
}
VehicleCtrlService::~VehicleCtrlService()
{
}

/*进程状态维护*/
SMLK_RC VehicleCtrlService::Init()
{
    /**********************************************************************
     *                       内部服务 参数初始化                            *
     **********************************************************************/
    /*************************************************
     *                  系统参数初始化                *
     *************************************************/
    setenv("TZ", "CST-8", 1);
    tzset();
    SMLK_LOGD("VehicleCtrlService->%s() Start, set [Timezone]=CST-8", __func__);
    std::lock_guard<std::mutex> lk(m_proc_mtx);
    if (m_flags & IService::Initialized)
    {
        SMLK_LOGI("Service already initialized, do nothing.");
        return SMLK_RC::RC_OK;
    }
    i_wakeup_source = WakeupSource::WAKEUP_SOURCE_UNKNOWN;
    i_engine_state = EngineMode::ENGINE_MODE_OFF;
    i_power_state = PowerMode::POWER_MODE_UNKNOWN;
    i_can_bus_state = CanBusMode::CAN_BUS_STATE_UNKONWN;
    i_ps_state = PsCtrlMode::PS_MODE_UNAVAILABLE;
    /*************************************************
     *                 系统服务初始化                 *
     *************************************************/
    /*初始化远控的状态参数配置文件*/
    if (SMLK_RC::RC_OK != InitStatusFile())
    {
        SMLK_LOGF("Init VehCtrl StatusFile Failed!");
        return SMLK_RC::RC_ERROR;
    }
    /*添加jtt808协议解析器*/
    m_map_prot_dissect.insert({(SMLK_UINT8)ProtoclID::E_PROT_JTT808, make_shared<DissectorJtt808>()});
    /*VecCtrl内部回调函数注册*/
    auto _sp_808 = make_shared<DissectorJtt808>();
    _sp_808->RegisterCB([this](VecCtrlInnerEventID event_id, void *data, size_t length) -> SMLK_BOOL
                        {
    std::string log = tools::uint8_2_string((SMLK_UINT8 *)data, length);
    SMLK_LOGD("[VecCtrlCB][EventID]=%d [len]=%d [data]=%s", event_id, length, log.c_str());
    switch (event_id)
    {
      case VecCtrlInnerEventID::VEC_CTRL_EVENT_DASH_BOARD_MTN:
      {
        if(m_timer_db_stop.load())
        {
            SMLK_LOGD("Dashboard timer already stop!");
            return SMLK_FALSE;
        }
        Process_8F42_0x19((SMLK_UINT8 *)data, length);
        break;
      }
      default:
        break;
    }
    return SMLK_TRUE; });
    /*仪表盘信息下发定时器初始化:异步发送0x18FC174A 周期1s发送5m*/
    m_timer_db = std::make_shared<LoopTimer<std::chrono::milliseconds, 100>>(
        [this](SMLK_UINT32 index)
        {
            if (0 == (index % 10))
            {
                if (index > DASHBOARD_PERIOD)
                {
                    SMLK_LOGD("Stop sending db mtn info");
                    m_timer_db->Reset();
                    m_timer_db_stop.store(SMLK_TRUE);
                    m_db_send_type = DashBoardMaintainSendStrategy::DB_MTN_SNED_NONE;
                }
                else
                {
                    SendDBMtnInfo(index);
                }
            }
        });
    m_timer_db->StartWithPause();
    SMLK_LOGD("[8F42][0x19][DashBoardTimer] ready.");
    m_timer_db_stop.store(SMLK_FALSE);

    /**********************************************************************
     *                          服务模块初始化                             *
     **********************************************************************/
    /*************************************************
     *                   TSP模块                     *
     *************************************************/
    /*TSP模块初始化*/
    SMLK_RC tsp_rc = TspServiceApi::getInstance()->Init(ModuleID::E_Module_rctrl_service);
    if (SMLK_RC::RC_OK != tsp_rc)
    {
        SMLK_LOGF("TspServiceApi Init failed.");
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("TspServiceApi->Init() OK.");
    /*注册TSP消息回调*/
    vector<SMLK_UINT16> msg_id;
    msg_id.push_back(JTT808_MSG_REMOTE_CTRL);              /*远程控车*/
    msg_id.push_back(JTT808_GET_SET_TBOX);                 /*远程配置tbox信息*/
    msg_id.push_back(JTT808_MSG_VEHICAL_QUERY);            /*远程查询车辆状态*/
    msg_id.push_back(JTT808_TERMINAL_PARAM_SET);           /*设置终端参数*/
    msg_id.push_back(JTT808_TERMINAL_PARAM_GET);           /*查询终端参数*/
    msg_id.push_back(JTT808_TERMINAL_SPECIFIED_PARAM_GET); /*查询指定终端参数*/
    /*绑定TSP消息回调处理函数*/
    TspServiceApi::getInstance()->RegisterMsgCB(ProtoclID::E_PROT_JTT808, msg_id, std::bind(&VehicleCtrlService::OnTspIndication, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    /*************************************************
     *                  车辆数据模块                   *
     *************************************************/
    /*VehicleData模块初始化*/
    SMLK_RC vd_rc = VehicleDataApi::GetInstance()->Init(ModuleID::E_Module_rctrl_service);
    if (SMLK_RC::RC_OK != vd_rc)
    {
        SMLK_LOGF("VehicleDataApi Init failed, err = %d.", vd_rc);
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("VehicleDataApi->Init() OK.");
    /*注册VD消息回调*/
    vector<SMLK_UINT32> index;
    index.push_back(VEHICLE_DATA_LOCK_STATUS_OF_DOOR_DRIVER);                 /*驾驶侧车门开解锁状态*/
    index.push_back(VEHICLE_DATA_LOCK_STATUS_OF_DOOR_PASSENGER);              /*乘客侧车门开解锁状态*/
    index.push_back(VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS);              /*原车空调开关状态*/
    index.push_back(VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS);                /*原车空调出风模式*/
    index.push_back(VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE);               /*原车空调温度*/
    index.push_back(VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE);               /*原车空调风量*/
    index.push_back(VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS);    /*原车空调循环模式*/
    index.push_back(VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS);             /*原车空调压缩机状态*/
    index.push_back(VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS);           /*原车空调auto状态*/
    index.push_back(VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS);         /*原车空调一键通风状态*/
    index.push_back(VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS);              /*独立暖风开关*/
    index.push_back(VEHICLE_DATA_WARM_AIR_TEMPERATURE);                       /*独立暖风温度*/
    index.push_back(VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS);      /*驻车空调开关*/
    index.push_back(VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE);       /*驻车空调auto开关*/
    index.push_back(VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE);       /*驻车空调温度*/
    index.push_back(VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS);                /*驻车空调风量*/
    index.push_back(VEHICLE_DATA_ENGINE_SPEED);                               /*发动机转速*/
    index.push_back(VEHICLE_DATA_DRIVER_TANK_ANTI_THEFT_STATUS);              /*油箱防盗开关*/
    index.push_back(VEHICLE_DATA_IGNITION_POSITION);                          /*ps IGN位置*/
    index.push_back(VEHICLE_DATA_VEHICLE_REMOTE_CONTROL_MODE);                /*ps 远控模式激活状态*/
    index.push_back(VEHICLE_DATA_TEMPERATURE_REMOTE_OPERATION_ALLOWED_STATE); /*冷机远程可控制状态*/
    index.push_back(VEHICLE_DATA_REFRIGERATION_SWITCH_STATUS);                /*制冷机组开关状态*/
    index.push_back(VEHICLE_DATA_DEFROST_ALLOWED_STATE);                      /*除霜允许状态*/
    index.push_back(VEHICLE_DATA_REFRIGERATION_WORK_MODE);                    /*除霜工作模式*/
    index.push_back(VEHICLE_DATA_RUC_REGULATION_TEMPERATURE);                 /*冷机当前设置的温度*/
    /*绑定VD消息回调处理函数*/
    VehicleDataApi::GetInstance()->RegistVehicleDataChangedCB(index, std::bind(&VehicleCtrlService::OnVehicleChangedIndication, this, std::placeholders::_1));
    /*************************************************
     *                  移动通信模块                   *
     *************************************************/
    /*移动通信TEL模块初始化*/
    if (!smartlink_sdk::Telephony::GetInstance()->Init(ModuleID::E_Module_rctrl_service))
    {
        SMLK_LOGF("fail to init Telephony service API interface.");
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("TelEventId->Init() OK.");
    /*注册TEL消息回调*/
    std::vector<smartlink_sdk::TelEventId> tel_event_vec = {
        smartlink_sdk::TelEventId::E_TEL_EVENT_SMS_RECV,
    };
    /*绑定TEL消息回调处理函数*/
    smartlink_sdk::Telephony::GetInstance()->RegEventCB(tel_event_vec, std::bind(&VehicleCtrlService::OnTelIndication, this,
                                                                                 std::placeholders::_1, std::placeholders::_2,
                                                                                 std::placeholders::_3));
    /*************************************************
     *                 电源管理模块                   *
     *************************************************/
    /*电源管理SysPower模块初始化*/
    if (!smartlink_sdk::SysPower::GetInstance()->Init(ModuleID::E_Module_rctrl_service))
    {
        SMLK_LOGE("fail to init power service API interface!!!");
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("SysPower->Init() OK.");
    /*注册SysPower消息回调*/
    std::vector<smartlink_sdk::SysPowerEventID> power_events = {
        smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP,
        smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP,
    };
    smartlink_sdk::SysPower::GetInstance()->RegEventCB(power_events, std::bind(&VehicleCtrlService::OnSysPowerSerIndication, this,
                                                                               std::placeholders::_1, std::placeholders::_2,
                                                                               std::placeholders::_3));
    /*************************************************
     *                  系统参数模块                   *
     *************************************************/
    /*SysProperty模块初始化*/
    if (!smartlink_sdk::SysProperty::GetInstance()->Init(ModuleID::E_Module_rctrl_service))
    {
        SMLK_LOGE("fail to init system property module");
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("SysProperty->Init() OK.");
    /*注册参数消息回调*/
    const std::vector<std::string> property_key = {
        std::string(SYS_PRO_NAME_GB_RECORDED_STATUS),
    };
    /*处理参数变化消息回调*/
    auto return_code = smartlink_sdk::SysProperty::GetInstance()->RegEventCB(property_key, [this](smartlink_sdk::SysProperty::PropertyEventId id, smartlink_sdk::PropertyInfo *info)
                                                                             {
        switch (id)
        {
          case smartlink_sdk::SysProperty::PropertyEventId::E_SYS_PROPERTY_EVENT_VALUE_CHANGED: {
            if (info->name ==std::string(SYS_PRO_NAME_GB_RECORDED_STATUS)) /*收到国6备案结果*/
            {
                if (!info->value.empty())
                {
#if 0
                    /*本地打桩模拟TSP消息,需配合TSP模拟数据打开*/
                    IpcTspHead ipc_temp;
                    ipc_temp.msg_id=JTT808_GET_SET_TBOX; /*模拟8F42数据*/
                    void *data;
                    size_t len;
                    OnTspIndication(ipc_temp,data,len);
#endif
                    OnRecvGBRecRes(info->value);
                }
            }
          }
        } });
    /*************************************************
     *                    MCU模块                     *
     *************************************************/
    if (smartlink_sdk::RtnCode::E_SUCCESS != smartlink_sdk::MCU::GetInstance()->Init(ModuleID::E_Module_rctrl_service))
    {
        SMLK_LOGE("fail to init mcu service API interface.");
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("MCU->Init() OK.");
    /*注册MCU消息回调*/
    std::vector<smartlink_sdk::McuEventId> mcu_events = {
        smartlink_sdk::McuEventId::E_MCU_EVENT_VEHICLE_CRTL_ACK, /*远控命令执行返回值*/
        smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE,
        smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_STATE};
    /*绑定MCU消息回调处理函数*/
    smartlink_sdk::MCU::GetInstance()->RegEventCB(mcu_events, std::bind(&VehicleCtrlService::OnMcuSerIndication, this,
                                                                        std::placeholders::_1, std::placeholders::_2,
                                                                        std::placeholders::_3));
    /*************************************************
     *                   WLAN模块                     *
     *************************************************/
    /*wlan模块初始化*/
    if (!smartlink_sdk::Wlan::GetInstance()->Init(ModuleID::E_Module_rctrl_service))
    {
        SMLK_LOGE("fail to init Wlan service API interface.");
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("Wlan->Init() OK.");
    /*************************************************
     *               OBD车载诊断系统模块               *
     *************************************************/
    /*OBD模块初始化*/
    if (SMLK_RC::RC_OK != smartlink::OBD::DidIpcApi::getInstance()->Init(ModuleID::E_Module_rctrl_service))
    {
        SMLK_LOGE("fail to init OBD service API interface.");
        return SMLK_RC::RC_ERROR;
    }
    SMLK_LOGI("OBD->Init() OK.");
    /**********************************************************************
     *                             业务处理                                *
     **********************************************************************/
    /*************************************************
     *             系统参数获取并保存到本地            *
     *************************************************/
    std::string property;
    std::string str_recv_ems, str_recv_vist;
    SMLK_LOGI("===[SysProperty][Sync] START===");
    /*获取EMS*/
    property = SYS_PRO_NAME_EMS;
    auto prop_get_rc = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, str_recv_ems);
    if (smartlink_sdk::RtnCode::E_SUCCESS != prop_get_rc)
    {
        SMLK_LOGE("Fail to get system property=%s rc=%d", property.c_str(), static_cast<std::int32_t>(prop_get_rc));
    }
    else
    {
        m_prop_ems = std::stoi(str_recv_ems);
        SMLK_LOGI("[EMS_TYPE]=%d", m_prop_ems);
    }
    /*获取VIST*/
    property = SYS_PRO_NAME_VIST;
    prop_get_rc = SysProperty::GetInstance()->GetValue(property, str_recv_vist);
    if (smartlink_sdk::RtnCode::E_SUCCESS != prop_get_rc)
    {
        SMLK_LOGE("Fail to get system property=%s rc=%d", property.c_str(), static_cast<std::int32_t>(prop_get_rc));
    }
    else
    {
        m_prop_vist = std::stoi(str_recv_vist);
        SMLK_LOGI("[VIST]=%d", m_prop_vist);
    }
    SMLK_LOGI("===[SysProperty][Sync] END===");
    /*************************************************
     *                  本地状态更新                  *
     *************************************************/
    /*0F51 0x27 ps状态变化查询并且更新本地状态*/
    InitUpdatePsCtrlMode();
    /*************************************************
     *                    主动上报                    *
     *************************************************/
    /*8F42 0x18仪表盘保养信息请求*/
    /*判断当前版本:V卡or基础 若为基础,读取产地配置参数:长春or青岛,若为长春,则不走业务*/
#ifdef FAW_MICRO_TRUCK
    {
        SMLK_LOGD("VKA");
        RegisterTspConnectActiveReport();
        /*判断当前tsp连接状态,防止在类似重新上电后Tsp成功连接状态已经通知过了但是rctrl仍未启导致漏报的情况*/
        TspConnectState m_tsp_connect_state;
        TspServiceApi::getInstance()->GetConnectState(m_tsp_connect_state);
        if (TspConnectState::E_CONNECTED == (TspConnectState)m_tsp_connect_state)
        {
            OnTspConnectActiveReport();
        }
    }
#endif
#ifdef FAW_TRUCK
    {
        SMLK_LOGD("JICHU");
        fur_on_tspconn_register = std::async(std::launch::async, [this]()
                                             {
        RtnCode rc;
        std::string prod_loc,name;
        name=SYS_PRO_NAME_PRODUCT_LOC;
        do {
            rc = SysProperty::GetInstance()->GetValue(name, prod_loc);
        } while (RtnCode::E_SUCCESS != rc);
        SMLK_LOGD("Cur production location is:%s", prod_loc == PRO_LOC_QINGDAO ? "Qingdao" : "Changchun");
        /*如果当前的产地为青岛*/
        if (PRO_LOC_QINGDAO == prod_loc)
        {
            RegisterTspConnectActiveReport();
            /*判断当前tsp连接状态,防止在类似重新上电后Tsp成功连接状态已经通知过了但是rctrl仍未启导致漏报的情况*/
            TspConnectState m_tsp_connect_state;
            TspServiceApi::getInstance()->GetConnectState(m_tsp_connect_state);
            if (TspConnectState::E_CONNECTED == (TspConnectState)m_tsp_connect_state)    OnTspConnectActiveReport();
        } });
    }
#endif

    /**********************************************************************
     *                       内部服务 参数初始化                            *
     **********************************************************************/
    /*************************************************
     *                  参数初始化                     *
     *************************************************/
    m_vehctrl_mcu_seq = 0; /*远控MCU消息流水号初始化0*/
    m_is_recv_tsp_cmd.store(SMLK_FALSE);
    m_is_recv_wakeup_text.store(SMLK_FALSE);
    m_flags |= IService::Initialized;

    SMLK_LOGD("VehicleCtrlService->%s() Done", __func__);
    return SMLK_RC::RC_OK;
}
SMLK_RC VehicleCtrlService::Start()
{
    SMLK_LOGD("VehicleCtrlService->%s()", __func__);
    std::lock_guard<std::mutex> lk(m_proc_mtx);
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
    /*************************************************
     *                   线程启动                     *
     *************************************************/
    m_msg_thread = std::thread(&VehicleCtrlService::ThreadMsgFromProcess, this);
    m_cmd_thread = std::thread(&VehicleCtrlService::ThreadControlQueryProcess, this);
    m_awaken_thread = std::thread(&VehicleCtrlService::ThreadAwakenProcess, this);
    m_offline_ctrl_thread = std::thread(&VehicleCtrlService::ThreadOfflineProcess, this);
    m_can_bus_wakeup_thread = std::thread(&VehicleCtrlService::ThreadCanBusWakeUpProcess, this);
    return SMLK_RC::RC_OK;
}

void VehicleCtrlService::Stop()
{
    SMLK_LOGD("*******enter in func(%s).*******", __func__);
    std::lock_guard<std::mutex> lk(m_proc_mtx);
    if (m_flags & IService::Running)
    {
        m_flags |= IService::Stopping;
        /*内部poco事件析构*/
        m_sp_awaken_evt->reset();
        m_sp_offlinectrl_evt->reset();
        m_sp_can_bus_wakeup_evt->reset();

        if (m_msg_thread.joinable())
        {
            m_msg_thread.join();
        }
        if (m_cmd_thread.joinable())
        {
            m_cmd_thread.join();
        }
        if (m_awaken_thread.joinable())
        {
            m_awaken_thread.join();
        }
        if (m_offline_ctrl_thread.joinable())
        {
            m_offline_ctrl_thread.join();
        }
        if (m_can_bus_wakeup_thread.joinable())
        {
            m_can_bus_wakeup_thread.join();
        }

        m_flags &= ~(IService::Running | IService::Stopping);
    }
}
bool VehicleCtrlService::IsRunning() const
{
    return m_flags & IService::Running;
}

/*****************************************************************************
 *                                基础方法                                    *
 *****************************************************************************/
/*初始化*/
/**
 * @brief   初始化车控参数配置类
 * @return
 */
SMLK_RC VehicleCtrlService::InitStatusFile()
{
    std::string status_file_path = VEHICLE_CTRL_STATUS_FILE_PATH;
    std::string status_file_name = VEHICLE_CTRL_STATUS_FILE_NAME;
    std::string path_name = status_file_path + status_file_name;
    MakeNewFolder(status_file_path);
    if (SMLK_RC::RC_OK != VehCtrlConfigJsonFile::GetInstance()->InitStatusJsonFile(path_name))
    {
        SMLK_LOGE("InitStatusJsonFile error.");
        return SMLK_RC::RC_ERROR;
    }
    return SMLK_RC::RC_OK;
}
/*消息监听处理函数*/
/**
 * @brief   车态模块回调处理
 * @return
 */
void VehicleCtrlService::OnVehicleChangedIndication(VehicleData data)
{
    if (VEHICLE_DATA_ENGINE_SPEED != data.index)
    {
        SMLK_LOGD("[Vd2Vec]={idx=%d value=%g valid=%d}", data.index, data.value, data.valid);
    }
    RctrlMsgQueue msg_queue;
    if (CUR_PROT_JTT808 == g_rctrl_protocol)
    {
        msg_queue.m_head.protocol = ProtoclID::E_PROT_JTT808;
    }
    else
    {
        msg_queue.m_head.protocol = ProtoclID::E_PROT_MAX;
    }
    /*明确队列消息的来源:VehicleData*/
    msg_queue.m_msg_type = RctrlMsgType::MSG_TYPE_VD_STATUS;
    CmdChangedStatus changed_status;
    VehctrlGetCmd get_cmd;

    if (VEHICLE_DATA_ENGINE_SPEED == data.index)
    {
        EngineMode input_engine_mode = (!data.valid) ? EngineMode::ENGINE_MODE_UNKNOWN : (SMLK_UINT16(data.value) < BOUNDARY_ENGINE_SPEED) ? EngineMode::ENGINE_MODE_OFF
                                                                                                                                           : EngineMode::ENGINE_MODE_ON;
        if (i_engine_state == input_engine_mode) /*防止转速频繁变动但是发动机状态未发生变化*/
        {
            return;
        }
        else
        {
            SMLK_LOGD("[Vd2Vec]={idx=%d value=%g valid=%d}. Cur Engine Sta:%d->%d", data.index, data.value, data.valid, i_engine_state, input_engine_mode);
            i_engine_state = input_engine_mode;
            get_cmd.query_id = VEHICLE_DATA_ENGINE_SPEED;
            get_cmd.value = SMLK_UINT8(i_engine_state);
        }
    }
    /*ps状态变化上报*/
    else if (VEHICLE_DATA_VEHICLE_REMOTE_CONTROL_MODE == data.index)
    {
        VehicleData vd_temp;
        get_cmd.query_id = SMLK_VEHCTRL_CMD_PS_CTRL_MODE;
        VehctrlGetCmd get_ign_pos, get_rctrl_mode;
        std::vector<VehicleData> vd_vec;
        vd_temp.index = VEHICLE_DATA_IGNITION_POSITION;
        vd_vec.push_back(vd_temp);
        VehicleDataApi::GetInstance()->GetVehicleData(vd_vec);
        VdDataValidCheck(vd_vec[0].index, vd_vec[0].value, vd_vec[0].valid, get_ign_pos);
        VdDataValidCheck(data.index, data.value, data.valid, get_rctrl_mode);
        PsCtrlMode pc_mode_temp;
        if ((VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_ACTIVE == (VehCtrlPsRemCtrlMode)get_rctrl_mode.value) /*ps远控模式*/
            || ((VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_DE_ACTIVE == (VehCtrlPsRemCtrlMode)get_rctrl_mode.value) && (PsIgnitionPosition::E_PS_IGN_POS_OFF == (PsIgnitionPosition)get_ign_pos.value)))
            pc_mode_temp = PsCtrlMode::PS_MODE_AVAILABLE;
        else
            pc_mode_temp = PsCtrlMode::PS_MODE_UNAVAILABLE;
        if (i_ps_state != pc_mode_temp)
        {
            SMLK_LOGD("[PsStatueChanged]:[%s]-->[%s]", (i_ps_state == PsCtrlMode::PS_MODE_UNKNOWN) ? "PS UNKNOWN" : (i_ps_state == PsCtrlMode::PS_MODE_AVAILABLE) ? "PS AVAILABLE"
                                                                                                                                                                  : "PS UNAVAILABLE",
                      (pc_mode_temp == PsCtrlMode::PS_MODE_UNKNOWN) ? "PS UNKNOWN" : (pc_mode_temp == PsCtrlMode::PS_MODE_AVAILABLE) ? "PS AVAILABLE"
                                                                                                                                     : "PS UNAVAILABLE");
            i_ps_state = pc_mode_temp;
            msg_queue.m_msg_type = RctrlMsgType::MSG_TYPE_OTHER;
            if (pc_mode_temp == PsCtrlMode::PS_MODE_AVAILABLE)
                get_cmd.value = SmlkTspPsRctrlAvailableSatatus::SMLK_TSP_PS_AVAILABLE;
            else if (pc_mode_temp == PsCtrlMode::PS_MODE_UNAVAILABLE)
                get_cmd.value = SmlkTspPsRctrlAvailableSatatus::SMLK_TSP_PS_UNAVAILABLE;
        }
        else
            return;
    }
    /*ps状态变化上报*/
    else if (VEHICLE_DATA_IGNITION_POSITION == data.index)
    {
        VehicleData vd_temp;
        get_cmd.query_id = SMLK_VEHCTRL_CMD_PS_CTRL_MODE;
        VehctrlGetCmd get_ign_pos, get_rctrl_mode;
        std::vector<VehicleData> vd_vec;
        vd_temp.index = VEHICLE_DATA_VEHICLE_REMOTE_CONTROL_MODE;
        vd_vec.push_back(vd_temp);
        VehicleDataApi::GetInstance()->GetVehicleData(vd_vec);
        VdDataValidCheck(vd_vec[0].index, vd_vec[0].value, vd_vec[0].valid, get_rctrl_mode);
        VdDataValidCheck(data.index, data.value, data.valid, get_ign_pos);
        PsCtrlMode pc_mode_temp;
        if ((VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_ACTIVE == (VehCtrlPsRemCtrlMode)get_rctrl_mode.value) /*ps远控模式*/
            || ((VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_DE_ACTIVE == (VehCtrlPsRemCtrlMode)get_rctrl_mode.value) && (PsIgnitionPosition::E_PS_IGN_POS_OFF == (PsIgnitionPosition)get_ign_pos.value)))
            pc_mode_temp = PsCtrlMode::PS_MODE_AVAILABLE;
        else
            pc_mode_temp = PsCtrlMode::PS_MODE_UNAVAILABLE;
        if (i_ps_state != pc_mode_temp)
        {
            SMLK_LOGD("[PsStatueChanged]:[%s]-->[%s]", (i_ps_state == PsCtrlMode::PS_MODE_UNKNOWN) ? "PS UNKNOWN" : (i_ps_state == PsCtrlMode::PS_MODE_AVAILABLE) ? "PS AVAILABLE"
                                                                                                                                                                  : "PS UNAVAILABLE",
                      (pc_mode_temp == PsCtrlMode::PS_MODE_UNKNOWN) ? "PS UNKNOWN" : (pc_mode_temp == PsCtrlMode::PS_MODE_AVAILABLE) ? "PS AVAILABLE"
                                                                                                                                     : "PS UNAVAILABLE");
            i_ps_state = pc_mode_temp;
            msg_queue.m_msg_type = RctrlMsgType::MSG_TYPE_OTHER;
            if (pc_mode_temp == PsCtrlMode::PS_MODE_AVAILABLE)
                get_cmd.value = SmlkTspPsRctrlAvailableSatatus::SMLK_TSP_PS_AVAILABLE;
            else if (pc_mode_temp == PsCtrlMode::PS_MODE_UNAVAILABLE)
                get_cmd.value = SmlkTspPsRctrlAvailableSatatus::SMLK_TSP_PS_UNAVAILABLE;
        }
        else
            return;
    }
    else /*VD数据有效性过滤*/
    {
        VdDataValidCheck(data.index, data.value, data.valid, get_cmd);
    }
    /*发动机状态变换主动上报*/
    changed_status.cmd_id = get_cmd.query_id;
    changed_status.change_info.data = get_cmd.value;
    changed_status.change_info.report_time = chrono::steady_clock::now() + std::chrono::seconds(cmd_load.m_report_delay);
    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&changed_status, (SMLK_UINT8 *)&changed_status + sizeof(CmdChangedStatus));
    m_msg_type_from_queue.put(msg_queue);
}
/**
 * @brief   MCU模块回调处理
 * @return
 */
void VehicleCtrlService::OnMcuSerIndication(smartlink_sdk::McuEventId id, void *data, int length)
{
    switch (id)
    {
    case smartlink_sdk::McuEventId::E_MCU_EVENT_VEHICLE_CRTL_ACK: /*控车回复--包括主动上报和控车响应*/
    {
        ProcessMcuSerResp(data, length);
        break;
    }
    case smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE: /*IGN状态事件上报*/
    {
        if (length != sizeof(smartlink_sdk::IGNState))
        {
            SMLK_LOGE("Recv ign state len=%d, expeceted len=%d!", length, sizeof(smartlink_sdk::IGNState));
            break;
        }
        smartlink_sdk::IGNState *ign = reinterpret_cast<smartlink_sdk::IGNState *>(data);
        PowerMode input_powermode = (ign->on) ? PowerMode::POWER_MODE_ON : PowerMode::POWER_MODE_OFF;
        if (i_power_state != input_powermode)
        {
            SMLK_LOGD("[IgnStatueChanged]:[%s]-->[%s]",
                      (i_power_state == PowerMode::POWER_MODE_UNKNOWN) ? "IGN UNKNOWN" : (i_power_state == PowerMode::POWER_MODE_OFF) ? "IGN OFF"
                                                                                                                                      : "IGN ON",
                      (input_powermode == PowerMode::POWER_MODE_UNKNOWN) ? "IGN UNKNOWN" : (input_powermode == PowerMode::POWER_MODE_OFF) ? "IGN OFF"
                                                                                                                                          : "IGN ON");
            /*IGN ON处理业务*/
            if ((PowerMode::POWER_MODE_ON != i_power_state) && (PowerMode::POWER_MODE_ON == input_powermode))
            {
                /*IGN ON后发送5s仪表盘保养信息, 填充全F*/
                {
                    std::lock_guard<std::mutex> lck(m_db_mtn_mutex);
                    if (DashBoardMaintainSendStrategy::DB_MTN_SEND_SPECIFIC != m_db_send_type) /*如果在判断IGON之前已经收到了TSP下发的配置消息,就直接下发配置消息*/
                    {
                        SMLK_LOGD("[DashBoardTimer][SendStrategy]=Default");
                        m_db_send_type = DashBoardMaintainSendStrategy::DB_MTN_SEND_DEFAULT;
                        m_db_mtn_require.clear();
                        m_db_mileage.clear();
                        for (auto i = 0; i < 4; ++i)
                        {
                            m_db_mtn_require.emplace_back((SMLK_UINT8)DashBoardMaintainRequirement::DB_MTN_REQMENT_NOTREQ);
                            m_db_mileage.emplace_back(0xFFFFFFFF);
                        }
                    }
                    SMLK_LOGD("[DashBoardTimer] start");
                    m_timer_db_stop.store(SMLK_FALSE);
                    m_timer_db->Reset();
                    m_timer_db->ReStart();
                }
                /*IGN ON读取配置信息 下发怠速暖机指令*/
                fur_wakeup_send_idle = std::async(std::launch::async, [this]()
                                                  { WakeupSendIdlingWarmMsg(); });
            }
            i_power_state = input_powermode;
            /*车辆电源状态变化主动上报*/
            ProcessAutoReportFromOther(SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_POWER_MODE, SMLK_UINT8(i_power_state));
        }
        break;
    }
    case smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_STATE: /*CAN网络状态事件上报*/
    {
        if (length != sizeof(smartlink_sdk::CANState))
        {
            SMLK_LOGE("Recv can state len=%d, expeceted len=%d!", length, sizeof(smartlink_sdk::IGNState));
            break;
        }
        CanBusMode input_can_state = (CanBusMode) * (reinterpret_cast<smartlink_sdk::CANState *>(data));
        if (i_can_bus_state != input_can_state)
        {
            SMLK_LOGD("[CanStatusChanged]:[%s]-->[%s]",
                      (i_can_bus_state == CanBusMode::CAN_BUS_STATE_NORMAL) ? "CAN NORMAL" : (i_can_bus_state == CanBusMode::CAN_BUS_STATE_SLEEP) ? "CAN SLEEP"
                                                                                         : (i_can_bus_state == CanBusMode::CAN_BUS_STATE_LOST)    ? "CAN LOST"
                                                                                                                                                  : "CAN UNKNOWN",
                      (input_can_state == CanBusMode::CAN_BUS_STATE_NORMAL) ? "CAN NORMAL" : (input_can_state == CanBusMode::CAN_BUS_STATE_SLEEP) ? "CAN SLEEP"
                                                                                         : (input_can_state == CanBusMode::CAN_BUS_STATE_LOST)    ? "CAN LOST"
                                                                                                                                                  : "CAN UNKNOWN");
            /*CAN网络唤醒业务--无论是从睡眠到Can normal或者长断都判定为CAN网络唤醒*/
            if ((CanBusMode::CAN_BUS_STATE_SLEEP == i_can_bus_state) && (CanBusMode::CAN_BUS_STATE_LOST == input_can_state) ||
                (CanBusMode::CAN_BUS_STATE_SLEEP == i_can_bus_state) && (CanBusMode::CAN_BUS_STATE_NORMAL == input_can_state) ||
                (CanBusMode::CAN_BUS_STATE_UNKONWN == i_can_bus_state) && (CanBusMode::CAN_BUS_STATE_LOST == input_can_state) ||
                (CanBusMode::CAN_BUS_STATE_UNKONWN == i_can_bus_state) && (CanBusMode::CAN_BUS_STATE_NORMAL == input_can_state))
            {
                m_sp_can_bus_wakeup_evt->set();
            }
            i_can_bus_state = input_can_state;
        }
        break;
    }
    default:
        return;
    }
}
/**
 * @brief   移动通信模块回调处理
 * @return
 */
void VehicleCtrlService::OnTelIndication(smartlink_sdk::TelEventId id, void *data, int len)
{
    SMLK_LOGD("enter in func(%s)", __func__);
    SMLK_LOGD("[event_id]=%ld [len]=%d ", (SMLK_UINT32)id, len);
    if (smartlink_sdk::TelEventId::E_TEL_EVENT_SMS_RECV != id)
    {
        SMLK_LOGE("[event_id]=%ld, not sms.", (SMLK_UINT32)id);
        return;
    }
    TelSMSInfo *psms = (TelSMSInfo *)data;
    const char *psms_content = psms->data.c_str();
    int sms_len = psms->data.size();
    SMLK_LOGI("[msisdn]=%s [sms]=%s.", psms->msisdn.c_str(), psms->data.c_str());

    RctrlMsgQueue msg_queue;
    bzero(&msg_queue.m_head, sizeof(RctrlHead));
    msg_queue.m_msg_type = RctrlMsgType::MSG_TYPE_TEXT_MSG;
    msg_queue.m_head.seq_id = 0xffff;
    if (CUR_PROT_JTT808 == g_rctrl_protocol)
    {
        msg_queue.m_head.protocol = ProtoclID::E_PROT_JTT808;
    }

    /*金融锁车功能开关打开关闭*/
    if ((strncmp((char *)psms_content, TEXT_MSG_FINANCIAL_LCK_FUNCTION_OPEN, strlen(TEXT_MSG_FINANCIAL_LCK_FUNCTION_OPEN)) == 0) ||
        (strncmp((char *)psms_content, TEXT_MSG_FINANCIAL_LCK_FUNCTION_CLOSE, strlen(TEXT_MSG_FINANCIAL_LCK_FUNCTION_CLOSE)) == 0))
    {
        if (CUR_PROT_JTT808 == g_rctrl_protocol)
        {
            Process_8F41_FinancialLockFunc(psms_content, sms_len, msg_queue);
        }
    }
    else if (strncmp((char *)psms_content, TEXT_MSG_TBOX_WEAK_UP, strlen(TEXT_MSG_TBOX_WEAK_UP)) == 0) /*短信唤醒*/
    {
        if (m_is_awaken)
        {
            SMLK_LOGD("Awaken. No need to wake up");
            return;
        }
        else
        {
            SMLK_LOGD("Sleeping. Do text wake up");
            m_sp_awaken_evt->set();
        }
    }
    else if (strncmp((char *)psms_content, TEXT_MSG_TBOX_RESTART, strlen(TEXT_MSG_TBOX_RESTART)) == 0) /*短信重启*/
    {
        smartlink_sdk::SysPower::GetInstance()->LockWake(std::string(SL_PROCESS_NAME));
        /*唤醒MCU*/
        smartlink_sdk::SysPower::GetInstance()->WakeUpMCU();
        usleep(1000 * 200);
        MCU::GetInstance()->SendRebootReq(ProcessorType::E_PROCESSOR_TYPE_MCU);
        usleep(1000 * 200);
        RtnCode rc = smartlink_sdk::SysPower::GetInstance()->Reboot((SMLK_UINT8)1, "DXCHONGQI");
        SMLK_LOGD("On Text Reboot, rc=%d.", (SMLK_INT32)(rc));
        smartlink_sdk::SysPower::GetInstance()->UnLockWake(std::string(SL_PROCESS_NAME));
    }
    /*打开或关闭tbox天线开路的锁车功能*/
    if ((strncmp((char *)psms_content, TEXT_MSG_TBOX_4G_ANTENNA_LOCK, strlen(TEXT_MSG_TBOX_4G_ANTENNA_LOCK)) == 0) ||
        (strncmp((char *)psms_content, TEXT_MSG_TBOX_4G_ANTENNA_UNLOCK, strlen(TEXT_MSG_TBOX_4G_ANTENNA_UNLOCK)) == 0))
    {
        Process_8103_4GAnt(psms_content, sms_len, msg_queue);
    }
}
/**
 * @brief   TSP模块回调处理
 * @return
 */
void VehicleCtrlService::OnTspIndication(IN IpcTspHead &ipc_head, IN void *data, std::size_t len)
{
    if ((JTT808_MSG_REMOTE_CTRL == ipc_head.msg_id) ||
        (JTT808_GET_SET_TBOX == ipc_head.msg_id) ||
        (JTT808_MSG_VEHICAL_QUERY == ipc_head.msg_id) ||
        (JTT808_TERMINAL_PARAM_SET == ipc_head.msg_id) ||
        (JTT808_TERMINAL_PARAM_GET == ipc_head.msg_id) ||
        (JTT808_TERMINAL_SPECIFIED_PARAM_GET == ipc_head.msg_id))
    {
        std::string log = tools::uint8_2_string((SMLK_UINT8 *)data, len);
        SMLK_LOGD("[Tsp2Tbox][MsgID:0x%04x][SN:0x%04x]=%s", ipc_head.msg_id, ipc_head.seq_id, log.c_str());

#if 0 /* Do simulate 8f41 msg*/
        std::function<std::size_t(char, OUT std::size_t)> hex2int = [this](char c, std::size_t int_o)
        {
            int_o = ((c >= '0') && (c <= '9')) ? std::size_t(c - '0') : ((c >= 'A') && (c <= 'F')) ? std::size_t(c - 'A' + 10)
                                                                    : ((c >= 'a') && (c <= 'f'))   ? std::size_t(c - 'a' + 10)
                                                                                                   : (std::size_t)-1;
            return int_o;
        };
        /* 02|211124210245|01|02|00|00 02为版本号|发送时间|功能个数|功能编号|功能指令|功能参数 */
        // std::string simu_msg = "02211124210245010B0104";                                                               /*8F41后视镜加热*/
        // std::string simu_msg = "0221112421024501100200";                                                               /*8F41怠速暖机1档*/
        // std::string simu_msg = "0221112421024501020000";                                                               /*8F41远程寻车*/
        // std::string simu_msg = "0221112421024503090100090508090406";                                                   /*8F41组合指令--空调控制*/
        // std::string simu_msg = "0221112421024501060100";                                                               /*8F41发动机启动*/
        // std::string simu_msg = "0500185a686f6e674875616e6e5f3132330a70417373776f726400";                               /*8F42WIFI SSID&密码设置*/
        // std::string simu_msg = "0d002718033231392e3134362e3234392e31393000520c043131332e3234302e3233392e313634003179"; /*8F42标准排放网关设置*/
        // std::string simu_msg = "0f00020001";                                                                           /*8F42国标备案开关设置开启*/
        // std::string simu_msg = "130000";                                                                               /*8F42发动机vin查询*/
        // std::string simu_msg = "140000";                                                                               /*8F42仓储模式查询*/
        // std::string simu_msg = "15000101";                                                                             /*8F42仓储模式设置01:仓储模式*/
        // std::string simu_msg = "19001400014b177fffffff7fffffff7fffffffffffffff";                                       /*8F42仪表盘保养里程设置*/
        // std::string simu_msg = "012203150955210101";                                                                   /*8F51获取车门状态*/
        // std::string simu_msg = "012203150955210106";                                                                   /*8F51获取发动机状态*/
        // std::string simu_msg = "012203150955210107";                                                                   /*8F51获取车辆电源状态*/
        // std::string simu_msg = "012203150955210109";                                                                   /*8F51获取原车空调状态*/
        // std::string simu_msg = "012203150955210110";                                                                   /*8F51获取独立暖风状态*/
        // std::string simu_msg = "01220315095521020910";                                                                 /*8F51组合查询原车+独立暖风状态*/
        // std::string simu_msg = "01220315095521020607";                                                                 /*8F51获取车辆电源状态*/
        // std::string simu_msg = "052204191542080127";                                                                   /*8F51获取ps状态*/
        std::vector<SMLK_UINT8> simu_data;
        size_t str_len = simu_msg.length() / 2;
        len = str_len;
        for (auto i = 0; i < str_len; ++i)
        {
            std::string str_temp = simu_msg.substr(i * 2, 2);
            SMLK_UINT8 c;
            std::size_t fc = 0, sc = 0;
            fc = hex2int(str_temp[0], fc);
            sc = hex2int(str_temp[1], sc);
            c = (((SMLK_UINT8)fc & 0x0F) << 4) | ((SMLK_UINT8)sc & 0x0F);
            simu_data.insert(simu_data.end(), &c, &c + sizeof(SMLK_UINT8));
        }
        data = (void *)&simu_data[0];

        log.clear();
        log = tools::uint8_2_string((SMLK_UINT8 *)data, str_len);
        SMLK_LOGD("[Simu][TspMsg]=%s", log.c_str());
#endif
        /*当IGOFF且未休眠,can网络已休眠,仍与tsp网关保持连接时:收到下行的控车指令(8F41&8F51)时,需要唤醒can网络。*/
        if ((CanBusMode::CAN_BUS_STATE_SLEEP == i_can_bus_state) && (PowerMode::POWER_MODE_OFF == i_power_state))
        {
            if (JTT808_MSG_REMOTE_CTRL == ipc_head.msg_id || JTT808_MSG_VEHICAL_QUERY == ipc_head.msg_id)
            {
                m_sp_offlinectrl_evt->set();
            }
        }
        RctrlMsgQueue msg_queue;
        GetRctrlHeadFromIpcHead(ipc_head, msg_queue.m_head);
        msg_queue.m_msg_type = RctrlMsgType::MSG_TYPE_TSP;
        msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)data, (SMLK_UINT8 *)data + len);
        /*置入消息队列中处理TSP消息*/
        m_msg_type_from_queue.put(msg_queue);
        if (((JTT808_MSG_REMOTE_CTRL == ipc_head.msg_id) && m_is_awaken) || ((JTT808_MSG_VEHICAL_QUERY == ipc_head.msg_id) && m_is_awaken))
        {
            /*如果在短信唤醒期间or离线控车期间收到了控车消息:8F41&8F51,则刷新30s计时器*/
            m_is_recv_tsp_cmd.store(SMLK_TRUE);
        }
    }
}
/**
 * @brief   电源管理模块回调处理
 * @return
 */
void VehicleCtrlService::OnSysPowerSerIndication(smartlink_sdk::SysPowerEventID id, void *data, int len)
{
    switch (id)
    {
    case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_WAKEUP:
    {
        auto inf = reinterpret_cast<smartlink_sdk::SysPowerWakeupInfo *>(data);
        SMLK_LOGD("[WakeUp] source=0x%02X", static_cast<SMLK_UINT8>(inf->source));
        WakeupSource input_wakeup_mode;
        switch (inf->source)
        {
        case SysWakeupSource::E_SYS_POWER_WAKEUP_SOURCE_MPU_RTC:
        case SysWakeupSource::E_SYS_POWER_WAKEUP_SOURCE_MCU_RTC:
            input_wakeup_mode = WakeupSource::WAKEUP_SOURCE_RTC;
            break;
        case SysWakeupSource::E_SYS_POWER_WAKEUP_SOURCE_GYRO:
            input_wakeup_mode = WakeupSource::WAKEUP_SOURCE_GYRO;
            break;
        case SysWakeupSource::E_SYS_POWER_WAKEUP_SOURCE_CAN:
            input_wakeup_mode = WakeupSource::WAKEUP_SOURCE_CAN;
            break;
        case SysWakeupSource::E_SYS_POWER_WAKEUP_SOURCE_MODEM_SMS:
            input_wakeup_mode = WakeupSource::WAKEUP_SOURCE_SMS;
            m_is_recv_wakeup_text.store(SMLK_TRUE);
            break;
        default:
            SMLK_LOGE("Unsupported WAKEUP_SOURCE: %d", (SMLK_UINT32)inf->source);
            input_wakeup_mode = WakeupSource::WAKEUP_SOURCE_UNKNOWN;
            break;
        }
        if (i_wakeup_source != input_wakeup_mode)
        {
            SMLK_LOGD("[WakeUpSource Changed]:[%s]-->[%s]",
                      (i_wakeup_source == WakeupSource::WAKEUP_SOURCE_UNKNOWN) ? "WakeSource UNKNOWN" : (i_wakeup_source == WakeupSource::WAKEUP_SOURCE_RTC) ? "WakeSource RTC"
                                                                                                    : (i_wakeup_source == WakeupSource::WAKEUP_SOURCE_GYRO)  ? "WakeSource GYRO"
                                                                                                    : (i_wakeup_source == WakeupSource::WAKEUP_SOURCE_CAN)   ? "WakeSource CAN"
                                                                                                                                                             : "WakeSource SMS",
                      (input_wakeup_mode == WakeupSource::WAKEUP_SOURCE_UNKNOWN) ? "WakeSource UNKNOWN" : (input_wakeup_mode == WakeupSource::WAKEUP_SOURCE_RTC) ? "WakeSource RTC"
                                                                                                      : (input_wakeup_mode == WakeupSource::WAKEUP_SOURCE_GYRO)  ? "WakeSource GYRO"
                                                                                                      : (input_wakeup_mode == WakeupSource::WAKEUP_SOURCE_CAN)   ? "WakeSource CAN"
                                                                                                                                                                 : "WakeSource SMS");
            i_wakeup_source = input_wakeup_mode;
            /*唤醒源变化主动上报*/
            ProcessAutoReportFromOther(SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_WAKEUP_SOURCE, SMLK_UINT8(i_wakeup_source));
        }
    }
    break;
    case smartlink_sdk::SysPowerEventID::E_SYS_POWER_EVENT_TO_SLEEP:
        m_is_awaken.store(SMLK_FALSE);
        break;
    default:
        SMLK_LOGE("[POWER] unsupported event id: %u", (SMLK_UINT32)id);
        return;
    }
}

/*消息处理线程*/
/**
 * @brief   从源头上根据消息队列消息来源进行任务分配:处理TSP,短信消息或者处理主动上报 线程
 * @return
 */
void VehicleCtrlService::ThreadMsgFromProcess(void)
{
    SMLK_LOGD("Thread %s Start!", __func__);
    while (!(m_flags & IService::Stopping))
    {
        bool timeout_flag = false;
        RctrlMsgQueue msg_queue = m_msg_type_from_queue.get(SL_QUEUE_GET_TIMEOUT_MS, &timeout_flag);
        if (m_map_prot_dissect.empty())
        {
            SMLK_LOGW("Prot dissector map is empty!");
            continue;
        }
        /*目前默认808协议,后续如果添加其他tsp协议,可根据RctrlHead::protocol判断*/
        auto iter = m_map_prot_dissect.find((SMLK_UINT8)(msg_queue.m_head.protocol));
        if (iter == m_map_prot_dissect.end())
            continue;
        shared_ptr<IDissector> dissector = iter->second;
        if (!timeout_flag)
        {
            if ((RctrlMsgType::MSG_TYPE_TSP == msg_queue.m_msg_type) || (RctrlMsgType::MSG_TYPE_TEXT_MSG == msg_queue.m_msg_type))
            {
                ProcessTspMessage(msg_queue, dissector);
            }
            else if (RctrlMsgType::MSG_TYPE_VD_STATUS == msg_queue.m_msg_type || (RctrlMsgType::MSG_TYPE_OTHER == msg_queue.m_msg_type))
            {
                PreProcAutoReportDataChanged(msg_queue);
            }
        }
        else
        {
            /*when queue is empty, go here to check status changed cmds, if over 5s , send status to tsp*/
            ProcessAutoReportDataChanged(msg_queue, dissector);
        }
    }
}
/**
 * @brief   控车,查询指令下发以及响应组包下发处理线程
 * @return
 */
void VehicleCtrlService::ThreadControlQueryProcess(void)
{
    SMLK_LOGD("Thread %s Start!", __func__);
    RctrlCmdQueue queue;
    while (!(m_flags & IService::Stopping))
    {
        bool timeout_flag = false;
        queue = m_control_query_queue.get(SL_QUEUE_GET_TIMEOUT_MS, &timeout_flag);
        RctrlResultQueue result;
        if (!timeout_flag)
        {
            /*结果中 消息来源 消息类型 版本号 协议类型都跟队列消息一致*/
            memcpy(&result.m_head, &queue.m_head, sizeof(RctrlHead));
            result.m_cmd_from = queue.m_cmd_from;
            result.m_result_type = queue.m_cmd_type;
            result.m_version = queue.m_version;
            /*判断消息类型 分配到控车和查询业务*/
            if (RctrlCmdType::REMCTRL_CMD_SET == queue.m_cmd_type) /*远程控车 8F41*/
            {
                SMLK_LOGD("********Ctrl Start********");
                Distribute_8F41(queue.config_queue.m_cmd_vec, result);
                /*同步tsp控车消息到响应组包中*/
                result.config_result.m_tsp_cmd_vec.clear();
                for (auto i = 0; i < queue.config_queue.m_cmd_vec.size(); i++)
                {
                    result.config_result.m_tsp_cmd_vec.emplace_back(queue.config_queue.m_tsp_cmd_vec[i]);
                }
                SMLK_LOGD("********Ctrl End********");
            }
            else if (RctrlCmdType::REMCTRL_CMD_GET == queue.m_cmd_type) /*远程状态查询 8F51 消息类型设置见8F51 cpp Decode*/
            {
                for (auto i = 0; i < queue.query_queue.m_query_vec.size(); ++i) /*将8F51查询的内容拷贝置入result*/
                {
                    result.query_result.m_query_id.emplace_back(queue.query_queue.m_query_vec[i]);
                }
                Process_8F51(queue.query_queue.m_query_vec, result.query_result.m_query_vec);
            }
            /*处理控车or查询结果 回复专有应答*/
            ProcessRCtrlResult(result);
        }
    }
}
/**
 * @brief   短信唤醒处理线程
 * @return
 */
void VehicleCtrlService::ThreadAwakenProcess(void)
{
    SMLK_LOGD("Thread %s Start!", __func__);
    while (!(m_flags & IService::Stopping))
    {
        m_sp_awaken_evt->wait();
        SMLK_LOGD("enter in thread: func(%s).", __func__);
        if (m_flags & IService::Stopping)
        {
            return;
        }
        if (m_is_awaken)
        {
            continue;
        }
        else
        {
            while (!m_is_recv_wakeup_text)
                ; /*收到电源管理模块的短信唤醒源后执行唤醒*/
            SMLK_LOGD("m_is_recv_wakeup_text=true");
            smartlink_sdk::SysPower::GetInstance()->LockWake(std::string(SL_PROCESS_NAME));
            m_is_awaken.store(SMLK_TRUE);
            // 唤醒mcu与can网络
            smartlink_sdk::SysPower::GetInstance()->WakeUpMCU();
            usleep(1000 * 100);
            smartlink_sdk::MCU::GetInstance()->WakeUpCanBus();

            std::atomic<SMLK_UINT8> timer;
            timer.store(0);
            do
            {
                if (!m_is_recv_tsp_cmd)
                {
                    // SMLK_LOGD("Cur timer=%d",timer.load());
                    timer.operator++();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                else
                {
                    SMLK_LOGD("Rcv RC cmd! Reset timer.");
                    timer.store(0);
                    m_is_recv_tsp_cmd.store(SMLK_FALSE);
                }
            } while (timer < 30);

            SMLK_LOGD("30s not rcv RM cmd!");
            smartlink_sdk::SysPower::GetInstance()->UnLockWake(std::string(SL_PROCESS_NAME));
            m_is_awaken.store(SMLK_FALSE);
            m_is_recv_wakeup_text.store(SMLK_FALSE);
        }
    }
}
/**
 * @brief   离线控车处理线程--离线控车时唤醒mcu和can网络
 * @return
 */
void VehicleCtrlService::ThreadOfflineProcess(void)
{
    SMLK_LOGD("Thread %s Start!", __func__);
    while (!(m_flags & IService::Stopping))
    {
        m_sp_offlinectrl_evt->wait();
        SMLK_LOGD("enter in thread: func(%s).", __func__);
        if (m_flags & IService::Stopping)
        {
            return;
        }
        // 唤醒mcu与can网络
        smartlink_sdk::SysPower::GetInstance()->WakeUpMCU();
        usleep(1000 * 100);
        smartlink_sdk::MCU::GetInstance()->WakeUpCanBus();
    }
}
/**
 * @brief   CAN网络唤醒上报线程--CAN唤醒后2s上报车身数据
 * @return
 */
void VehicleCtrlService::ThreadCanBusWakeUpProcess(void)
{
    SMLK_LOGD("Thread %s Start!", __func__);
    while (!(m_flags & IService::Stopping))
    {
        m_sp_can_bus_wakeup_evt->wait();
        SMLK_LOGD("enter in thread: func(%s).", __func__);
        if (m_flags & IService::Stopping)
        {
            return;
        }
        sleep(2);
        WakeupSendVehicleStatus();
    }
}

/*消息处理方法*/
/**
 * @brief           MCU控车响应处理
 * @param   data    MCU控车响应消息内容:流水号+控制消息数量+(消息id+消息动作+消息长度+消息内容)*N
 * @param   length  MCU控车响应消息内容长度
 * @return  SMLK_RC
 */
void VehicleCtrlService::ProcessMcuSerResp(IN void *data, IN int len)
{
    unique_lock<std::mutex> lck(m_cmd_mtx);
    if (len < (int)(sizeof(SmlkMcuCmdHead)))
    {
        SMLK_LOGE("Data len(%d) < head(%ld).", len, sizeof(SmlkMcuCmdHead));
        return;
    }
    SmlkMcuCmdHead *phead = (SmlkMcuCmdHead *)data;
    // SMLK_LOGI("[Mcu2Mpu][SN]=0x%04x [Mpu2Mcu][SN]=0x%04x", be16toh(phead->seq), m_vehctrl_mcu_seq);
    std::string pszMcuMsg = tools::uint8_2_string((SMLK_UINT8 *)data, len);
    SMLK_LOGD("[Mcu2Mpu]=%s", pszMcuMsg.c_str());

    if ((phead->seq != htobe16(m_vehctrl_mcu_seq)) && (phead->seq != 0xffff))
    {
        SMLK_LOGE("rcv seq(%d) != send seq(%d).", be16toh(phead->seq), m_vehctrl_mcu_seq);
        return;
    }
    else
    {
        if (phead->seq == 0xffff) /*处理MCU主动上报消息*/
        {
            ProcessMcuCtrlAutoReport(data, len);
        }
        else
        {
            if (phead->cmd_num == 1) /*单指令响应*/
            {
                SmlkMcuCmdResp *presult = (SmlkMcuCmdResp *)((SMLK_UINT8 *)data + sizeof(SmlkMcuCmdHead));
                presult->cmd_id = be16toh(presult->cmd_id);
                SMLK_LOGD("[McuResp]={id=%d act=0x%02x res=%d}", presult->cmd_id, presult->cmd_act, presult->cmd_res);
                auto iter = m_mcu_tsp_resp_map.find((SmlkMcuCmdResult)presult->cmd_res);
                if (iter == m_mcu_tsp_resp_map.end())
                {
                    SMLK_LOGW("Mcu Resp mismatch!");
                    m_mcu_resp = SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_ERROR;
                }
                else
                {
                    m_mcu_resp = iter->second;
                }
            }
            else if (phead->cmd_num > 1) /*组合指令响应*/
            {
                SmlkMcuCmdResp *presult = (SmlkMcuCmdResp *)((SMLK_UINT8 *)data + sizeof(SmlkMcuCmdHead));
                for (SMLK_UINT8 i = 0; i < phead->cmd_num; ++i)
                {
                    presult->cmd_id = be16toh(presult->cmd_id);
                    SMLK_LOGD("[%dth][McuResp]={id=%d act=0x%02x res=%d}", i, presult->cmd_id, presult->cmd_act, presult->cmd_res);
                    SmlkRctrl_8F41_Result resp_temp;
                    auto iter = m_mcu_tsp_resp_map.find((SmlkMcuCmdResult)presult->cmd_res);
                    if (iter == m_mcu_tsp_resp_map.end())
                    {
                        SMLK_LOGW("Mcu Resp mismatch!");
                        resp_temp = SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_ERROR;
                    }
                    else
                    {
                        resp_temp = iter->second;
                    }
                    m_mcu_resps.emplace_back(resp_temp);
                    presult++;
                }
            }
        }

        m_cmd_cv.notify_one();
    }
}
/**
 * @brief           MCU控车主动上报处理--定时熄火&紧急熄火上报tsp
 * @param   data    MCU主动上报消息内容:版本号+控制消息数量+(消息id+消息动作+消息长度+消息内容)*N
 * @param   length  MCU主动上报消息内容长度
 * @return  SMLK_RC
 */
SMLK_RC VehicleCtrlService::ProcessMcuCtrlAutoReport(IN void *data, IN int len)
{
    SmlkMcuCmdHead *phead = (SmlkMcuCmdHead *)data;
    SmlkMcuCmdResp *presult = (SmlkMcuCmdResp *)((SMLK_UINT8 *)data + sizeof(SmlkMcuCmdHead));
    if (MCU_SINGLE_MSG_NO_CONTENT_LEN != len)
    {
        SMLK_LOGW("[McuAutoReport] len error!");
        return SMLK_RC::RC_ERROR;
    }
    // SMLK_LOGD("Seq=0x%04x cmd_num=%d", phead->seq, phead->cmd_num);
    presult->cmd_id = be16toh(presult->cmd_id);
    // SMLK_LOGD("[McuResp]={id=%d act=0x%02x res=%d}", presult->cmd_id, presult->cmd_act, presult->cmd_res);
    switch ((SmlkMcuMsgID)presult->cmd_id)
    {
    case SmlkMcuMsgID::SMLK_MCU_CMD_ENGINE:
    {
        std::vector<SMLK_UINT8> output_vec;
        /*0F41响应组包头部信息*/
        Msg0F41_Head head_0f41;
        bzero(&head_0f41, sizeof(Msg0F41_Head));
        head_0f41.version = 0x00;
        tools::get_bcd_timestamp(head_0f41.time);
        head_0f41.seq_id = 0xffff;
        head_0f41.cmd_num = 0x01;
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&head_0f41, (SMLK_UINT8 *)&head_0f41 + sizeof(Msg0F41_Head));
        /*tsp svc头部信息*/
        RctrlHead resp_head;
        resp_head.msg_id = JTT808_REMOTE_CTRL_RESP;
        resp_head.priority = PRIORITY0;
        resp_head.protocol = (ProtoclID)CUR_PROT_JTT808;
        resp_head.qos = QOS_SEND_ALWAYS;
        resp_head.seq_id = 0xffff;
        /*0F41响应组包内容信息*/
        Msg0F41_Body resp_body;
        bzero(&resp_body, sizeof(Msg0F41_Body));
        resp_body.id = JTT808_8F41_ENGINE_CTRL;
        resp_body.cmd = JTT808_SUBCMD_ENGINE_START;
        switch (presult->cmd_act)
        {
        case JTT808_PARAM_ENGINE_STOP_ON_TIME:
        {
            /*0F41响应结果判断*/
            if (presult->cmd_res == (SMLK_UINT8)SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_SUCCESS)
                resp_body.result = (SMLK_UINT8)SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_TIMING_ENGINE_STOP_SUCC;
            else if (presult->cmd_res == (SMLK_UINT8)SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_ERROR)
                resp_body.result = (SMLK_UINT8)SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_TIMING_ENGINE_STOP_FAIL;

            break;
        }
        case JTT808_PARAM_ENGINE_STOP_EMERGENCY:
        {
            /*0F41响应结果判断*/
            if (presult->cmd_res == (SMLK_UINT8)SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_SUCCESS)
                resp_body.result = (SMLK_UINT8)SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_EMERG_ENGINE_STOP_SUCC;
            else if (presult->cmd_res == (SMLK_UINT8)SmlkMcuCmdResult::SMLK_MCU_CMD_RESULT_ERROR)
                resp_body.result = (SMLK_UINT8)SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_EMERG_ENGINE_STOP_FAIL;
            break;
        }
        default:
            return SMLK_RC::RC_ERROR;
        }
        SMLK_LOGD("[EngineAutoReport]={id=%d cmd=%d res=%d}", resp_body.id, resp_body.cmd, resp_body.result);
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&resp_body, (SMLK_UINT8 *)&resp_body + sizeof(Msg0F41_Body));
        DissectorJtt808Common::getInstance()->DoSendMsgToTsp(resp_head, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
        break;
    }
    default:
        return SMLK_RC::RC_ERROR;
    }
}
/**
 * @brief               处理TSP消息,根据协议id进入不同的协议解析器解析;如果生成了远控内部命令,那么扔进远控命令处理消息队列中
 * @param   msg_queue   消息处理消息队列中的一条数据
 * @param   dissector   协议处理器智能指针
 * @return  SMLK_RC
 */
SMLK_RC VehicleCtrlService::ProcessTspMessage(IN RctrlMsgQueue &msg_queue, IN shared_ptr<IDissector> &dissector)
{
    RctrlCmdQueue cmd_queue;
    SMLK_UINT8 is_encode = NOT_GOTO_ENCODE;
    /*拷贝tsp消息头信息*/
    cmd_queue.m_head.msg_id = msg_queue.m_head.msg_id;
    cmd_queue.m_head.protocol = msg_queue.m_head.protocol;
    cmd_queue.m_head.seq_id = msg_queue.m_head.seq_id;
    cmd_queue.m_head.qos = msg_queue.m_head.qos;
    cmd_queue.m_head.priority = msg_queue.m_head.priority;
    if (RctrlMsgType::MSG_TYPE_TEXT_MSG == msg_queue.m_msg_type)
    {
        cmd_queue.m_cmd_from = VehCtrlCmdFrom::FROM_TEXT_MESSAGE;
    }
    else if (RctrlMsgType::MSG_TYPE_TSP == msg_queue.m_msg_type)
    {
        cmd_queue.m_cmd_from = VehCtrlCmdFrom::FROM_TSP_MODULE;
    }
    SMLK_RC rc = dissector->Decode((SMLK_UINT8 *)msg_queue.m_message.data(), msg_queue.m_message.size(), cmd_queue, is_encode);
    if ((SMLK_RC::RC_OK == rc) && (GOTO_ENCODE == is_encode))
    {
        m_control_query_queue.put(cmd_queue);
    }
    return SMLK_RC::RC_OK;
}
/**
 * @brief           主动上报消息预处理
 * @param msg_queue 消息处理消息队列
 *
 * @return
 */
SMLK_RC VehicleCtrlService::PreProcAutoReportDataChanged(IN RctrlMsgQueue &msg_queue)
{
    CmdChangedStatus cur_status;
    memcpy(&cur_status, msg_queue.m_message.data(), sizeof(CmdChangedStatus));
    switch (msg_queue.m_msg_type)
    {
    case RctrlMsgType::MSG_TYPE_VD_STATUS:
    {
        // SMLK_LOGD("[ChangeVD]={id=%d val=%g}.", cur_status.cmd_id, cur_status.change_info.data);
        auto iter = m_map_vd_change_status.find(cur_status.cmd_id);
        if (iter == m_map_vd_change_status.end())
        {
            // SMLK_LOGD("Gene new VD={id=%d}", cur_status.cmd_id);
            m_map_vd_change_status.insert(make_pair(cur_status.cmd_id, cur_status.change_info));
        }
        else
        {
            // SMLK_LOGD("Change VD={id=%d data=%g}.", cur_status.cmd_id, cur_status.change_info.data);
            iter->second.data = cur_status.change_info.data;
            iter->second.report_time = cur_status.change_info.report_time; /*+ 5s*/
        }
        break;
    }
    case RctrlMsgType::MSG_TYPE_OTHER:
    {
        SMLK_LOGD("[%sChanged]:{data=%g}.", (cur_status.cmd_id == (SMLK_UINT16)SMLK_VEHCTRL_CMD_ENGINE_MODE) ? "EngineMode" : (cur_status.cmd_id == (SMLK_UINT16)SMLK_VEHCTRL_CMD_POWER_MODE)  ? "PowerMode"
                                                                                                                          : (cur_status.cmd_id == (SMLK_UINT16)SMLK_VEHCTRL_CMD_WAKEUP_SOURCE) ? "WakeupSource"
                                                                                                                          : (cur_status.cmd_id == (SMLK_UINT16)SMLK_VEHCTRL_CMD_PS_CTRL_MODE)  ? "PsCtrlMode"
                                                                                                                                                                                               : "UNKNOWN",
                  cur_status.change_info.data);
        auto iter = m_map_mcu_change_status.find(cur_status.cmd_id);
        if (iter == m_map_mcu_change_status.end())
        {
            // SMLK_LOGD("Gene new msg={id=%d}",cur_status.cmd_id);
            m_map_mcu_change_status.insert(make_pair(cur_status.cmd_id, cur_status.change_info));
        }
        else
        {
            // SMLK_LOGD("Change msg={id=%d data=%g}.", cur_status.cmd_id,cur_status.change.status);
            iter->second.data = cur_status.change_info.data;
            iter->second.report_time = cur_status.change_info.report_time; /*+ 5s*/
        }
        break;
    }
    default:
        break;
    }
    return SMLK_RC::RC_OK;
}
/**
 * @brief           主动上报消息处理
 * @param msg_queue 消息处理消息队列
 * @param dissector 消息解析器,用于消息组包上报
 *
 * @return
 */
SMLK_RC VehicleCtrlService::ProcessAutoReportDataChanged(IN RctrlMsgQueue &msg_queue, IN shared_ptr<IDissector> &dissector)
{
    SMLK_RC rc = SMLK_RC::RC_OK;
    switch (msg_queue.m_msg_type)
    {
    case RctrlMsgType::MSG_TYPE_VD_STATUS:
    {
        vector<SMLK_UINT16> vd_vec;
        for (auto iter = m_map_vd_change_status.begin(); iter != m_map_vd_change_status.end(); ++iter)
        {
            std::chrono::steady_clock::time_point expired_at = std::chrono::steady_clock::now();
            SMLK_UINT16 cmd_id = iter->first;
            ChangedStatusInfo changed = iter->second;
            if (expired_at >= changed.report_time)
            {
                SMLK_LOGD("[VdStable]:{id=%d data=%g}", cmd_id, changed.data); /*VD数据5s未变化*/
                rc = dissector->OnChanged((SMLK_UINT8)RctrlMsgType::MSG_TYPE_VD_STATUS, cmd_id, changed.data);
                vd_vec.push_back(cmd_id);
            }
        }
        for (auto id : vd_vec)
        {
            m_map_vd_change_status.erase(id);
        }
        break;
    }
    case RctrlMsgType::MSG_TYPE_OTHER:
    {
        vector<SMLK_UINT16> mcu_msg_vec;
        for (auto iter = m_map_mcu_change_status.begin(); iter != m_map_mcu_change_status.end(); ++iter)
        {
            std::chrono::steady_clock::time_point expired_at = std::chrono::steady_clock::now();
            SMLK_UINT16 cmd_id = iter->first;
            ChangedStatusInfo changed = iter->second;
            if (expired_at >= changed.report_time)
            {
                SMLK_LOGD("[%sStable]:{data=%g}", (cmd_id == (SMLK_UINT16)SMLK_VEHCTRL_CMD_ENGINE_MODE) ? "EngineMode" : (cmd_id == (SMLK_UINT16)SMLK_VEHCTRL_CMD_POWER_MODE)  ? "PowerMode"
                                                                                                                     : (cmd_id == (SMLK_UINT16)SMLK_VEHCTRL_CMD_WAKEUP_SOURCE) ? "WakeupSource"
                                                                                                                     : (cmd_id == (SMLK_UINT16)SMLK_VEHCTRL_CMD_PS_CTRL_MODE)  ? "PsCtrlMode"
                                                                                                                                                                               : "UNKNOWN",
                          changed.data);
                rc = dissector->OnChanged((SMLK_UINT8)RctrlMsgType::MSG_TYPE_OTHER, cmd_id, changed.data);
                mcu_msg_vec.push_back(cmd_id);
            }
        }
        for (auto id : mcu_msg_vec)
        {
            m_map_mcu_change_status.erase(id);
        }
        break;
    }
    default:
        break;
    }
    return rc;
}
/**
 * @brief                       主动上报其他消息源处理: 发动机,车辆电源,唤醒源状态主动上报
 * @param SmlkRctrlInnerMsgID   消息ID
 * @param data                  消息内容
 *
 * @return
 */
SMLK_RC VehicleCtrlService::ProcessAutoReportFromOther(IN SmlkRctrlInnerMsgID id, IN SMLK_UINT8 data)
{
    RctrlMsgQueue msg_queue;
    CmdChangedStatus changed_status;
    msg_queue.m_head.protocol = smartlink::ProtoclID::E_PROT_JTT808;
    msg_queue.m_msg_type = RctrlMsgType::MSG_TYPE_OTHER;
    changed_status.cmd_id = (SMLK_UINT16)id;
    changed_status.change_info.data = SMLK_UINT8(data);
    changed_status.change_info.report_time = chrono::steady_clock::now() + std::chrono::seconds(cmd_load.m_report_delay);
    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&changed_status, (SMLK_UINT8 *)&changed_status + sizeof(CmdChangedStatus));
    m_msg_type_from_queue.put(msg_queue);
}
/**
 * @brief           分配8F41控车指令
 * @param cmd_vec   输入控车数据
 * @param result    输出控车结果
 *
 * @return
 */
SMLK_RC VehicleCtrlService::Distribute_8F41(IN vector<SmlkMcuCmd> &cmd_vec, OUT RctrlResultQueue &result)
{
    SMLK_RC rc = SMLK_RC::RC_ERROR;
    if (cmd_vec.size() > 1) /*组合指令下发*/
    {
        Process_8F41_Cmds(cmd_vec, result);
    }
    else if (cmd_vec.size() == 1) /*单指令下发*/
    {
        auto mcu_cmd = cmd_vec[0];
        SmlkMcuCmdResp result_temp;
        result_temp.cmd_id = mcu_cmd.cmd_id;
        result_temp.cmd_act = mcu_cmd.cmd_act;
        // result_temp.cmd_res = mcu_resp_temp;
        auto config_info = cmd_load.m_config_file_map.find(mcu_cmd.cmd_id);
        /*怠速暖机--先判断vist类型,再根据IGN状态选择直接下发MCU或者保存配置下次IGON再下发*/
        if (SMLK_MCU_CMD_IDLING_WARM_UP == mcu_cmd.cmd_id)
        {
            SMLK_LOGD("vist=%d", m_prop_vist);
            SMLK_RC rc = SMLK_RC::RC_OK;
            if (m_prop_vist == 0)
            {
                if (i_power_state == PowerMode::POWER_MODE_OFF)
                {
                    if (SMLK_MCU_ACTION_IDLING_WARM_UP_CLOSE == mcu_cmd.cmd_act)
                    {
                        rc = VehCtrlConfigJsonFile::GetInstance()->SetValueToJsonFile(VEHICLE_CTRL_IDLING_WARM_UP_STATUS, to_string(((SMLK_UINT8)(RctrlStatusIdlingWarmUp::STATUS_CLOSE))));
                    }
                    else if (SMLK_MCU_ACTION_AC_IDLING_WARM_UP_LEVEL_1 == mcu_cmd.cmd_act)
                    {
                        rc = VehCtrlConfigJsonFile::GetInstance()->SetValueToJsonFile(VEHICLE_CTRL_IDLING_WARM_UP_STATUS, to_string(((SMLK_UINT8)(RctrlStatusIdlingWarmUp::STATUS_OPEN_LEV_1))));
                    }
                    else if (SMLK_MCU_ACTION_AC_IDLING_WARM_UP_LEVEL_2 == mcu_cmd.cmd_act)
                    {
                        rc = VehCtrlConfigJsonFile::GetInstance()->SetValueToJsonFile(VEHICLE_CTRL_IDLING_WARM_UP_STATUS, to_string(((SMLK_UINT8)(RctrlStatusIdlingWarmUp::STATUS_OPEN_LEV_2))));
                    }
                    if (rc == SMLK_RC::RC_OK)
                    {
                        result.config_result.m_tsp_result.emplace_back(SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_OK);
                    }
                    else
                    {
                        result.config_result.m_tsp_result.emplace_back(SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_ERROR);
                    }
                }
                else
                {
                    Process_8F41_Cmd(mcu_cmd, result);
                }
            }
            else
            {
                result.config_result.m_tsp_result.emplace_back(SmlkRctrl_8F41_Result::SMLK_TSP_8F41_RESULT_CTRL_NOT_SP);
            }
        }
        else
        {
            if (config_info == cmd_load.m_config_file_map.end())
            {
                Process_8F41_Cmd(mcu_cmd, result);
            }
            else
            {
                Process_8F41_Cmd(mcu_cmd, config_info->second, result);
            }
        }
        result.config_result.m_mcu_res_vec.push_back(result_temp);
    }
    return SMLK_RC::RC_OK;
}
/**
 * @brief               根据命令在配置文件中的信息,发送命令到mcu service
 * @param mcu_cmd       mcu控制指令
 * @param config_info   配置文件信息
 * @param result        响应处理消息队列
 *
 * @return
 */
SMLK_RC VehicleCtrlService::Process_8F41_Cmd(IN SmlkMcuCmd &mcu_cmd, IN RemoteCtrlConfigFileInfo &config_info, OUT RctrlResultQueue &result)
{
    SMLK_LOGD("enter in func(%s).. find in config file", __func__);

    SMLK_UINT8 send_times = 0;                 /*cmd send times*/
    SMLK_UINT8 location = 0;                   /*loction of the errcode */
    vector<SMLK_UINT16> errcode_sendtimes_vec; /*corresponding to errcode in config file*/
    SmlkRctrl_8F41_Result tsp_resp;
    /*最后一个为其他错误码,采用默认超时,重传次数*/
    for (SMLK_UINT8 i = 0; i < config_info.retry_vec.size() + 1; i++)
    {
        errcode_sendtimes_vec.push_back(0);
    }
    do
    {
        if (send_times > 0)
        {
            SMLK_LOGD("delay time:%d s", ((location == config_info.retry_vec.size()) ? (cmd_load.m_defalut_retrydelay) : (config_info.retry_delay)));
            sleep(((location == config_info.retry_vec.size()) ? (cmd_load.m_defalut_retrydelay) : (config_info.retry_delay)));
        }
        send_times++;

        SendCtrlCmdToMcu(mcu_cmd);
        unique_lock<std::mutex> lck(m_cmd_mtx);
        SMLK_LOGD("wait for %ds", config_info.timieout);
        if (m_cmd_cv.wait_for(lck, std::chrono::seconds(config_info.timieout)) == std::cv_status::timeout)
        {
            tsp_resp = SMLK_TSP_8F41_RESULT_ERROR;
            result.config_result.m_tsp_result.emplace_back(SMLK_TSP_8F41_RESULT_ERROR);
        }
        else
        {
            result.config_result.m_tsp_result.emplace_back(m_mcu_resp);
            SMLK_LOGD("no timeout,rctrl_cmd.cmd_id=%d m_result=%d.", mcu_cmd.cmd_id, m_mcu_resp);
        }

        if (SMLK_TSP_8F41_RESULT_OK != result.config_result.m_tsp_result[0])
        {
            FindErrcodeInConfigfile(result.config_result.m_tsp_result[0], config_info.retry_vec, location);
            errcode_sendtimes_vec[location]++;
        }
        SMLK_LOGD("location=%d,config_info.retry_vec.size()=%ld,m_defalut_retrytimes=%d\r", location, config_info.retry_vec.size(), cmd_load.m_defalut_retrytimes);
    } while ((errcode_sendtimes_vec[location] < ((location == config_info.retry_vec.size()) ? (cmd_load.m_defalut_retrytimes) : (config_info.retry_vec[location].err_retry_times + 1))) && (send_times < config_info.max_send_times));
    return SMLK_RC::RC_OK;
}
/**
 * @brief           根据重传策略发送指令给mcu,并获取mcu响应
 * @param mcu_cmd   mcu控制指令
 * @param result    响应处理消息队列
 *
 * @return
 */
SMLK_RC VehicleCtrlService::Process_8F41_Cmd(IN SmlkMcuCmd &mcu_cmd, OUT RctrlResultQueue &result)
{
    SMLK_UINT8 send_times = 0;
    while (send_times <= cmd_load.m_defalut_retrytimes)
    {
        if (send_times > 0)
        {
            SMLK_LOGD("delay time=%ds", cmd_load.m_defalut_retrydelay);
            sleep(cmd_load.m_defalut_retrydelay); /*第一次发送无需延时*/
        }
        send_times++;
        // SMLK_LOGD("Try time=%d.",send_times - 1);
        SendCtrlCmdToMcu(mcu_cmd);
        int overtime = cmd_load.m_defalut_timeout;
        if ((SMLK_MCU_CMD_ENGINE == mcu_cmd.cmd_id) && (SMLK_MCU_ACTION_ON == mcu_cmd.cmd_act)) /*发动机启动MCU超时时间40s*/
        {
            overtime = MCU_ENGINE_START_OVER_TIME;
        }
        unique_lock<std::mutex> lck(m_cmd_mtx);
        SMLK_LOGD("Waiting for %ds", overtime);
        if (m_cmd_cv.wait_for(lck, std::chrono::seconds(overtime)) == std::cv_status::timeout)
        {
            SMLK_LOGD("Recv mcu resp timeout!");
            result.config_result.m_tsp_result.emplace_back(SMLK_TSP_8F41_RESULT_ERROR);
        }
        else
        {
            result.config_result.m_tsp_result.emplace_back(m_mcu_resp);
        }
    }
    return SMLK_RC::RC_OK;
}
/**
 * @brief           根据重传策略发送组合指令给mcu,并获取mcu响应
 * @param mcu_cmds  mcu组合控制指令
 * @param result    响应处理消息队列
 *
 * @return
 */
SMLK_RC VehicleCtrlService::Process_8F41_Cmds(IN std::vector<SmlkMcuCmd> &mcu_cmds, OUT RctrlResultQueue &result)
{
    SendCtrlCmdsToMcu(mcu_cmds);
    unique_lock<std::mutex> lck(m_cmd_mtx);
    int overtime = cmd_load.m_defalut_timeout;
    auto iter = mcu_cmds.begin();
    for (iter = mcu_cmds.begin(); iter != mcu_cmds.end();)
    {
        if ((SMLK_MCU_CMD_ENGINE == iter->cmd_id) && (SMLK_MCU_ACTION_ON == iter->cmd_act))
        {
            overtime = MCU_ENGINE_START_OVER_TIME;
            break;
        }
        else
            iter++;
    }
    SMLK_LOGD("Waiting for %ds", overtime);
    if (m_cmd_cv.wait_for(lck, std::chrono::seconds(overtime)) == std::cv_status::timeout)
    {
        SMLK_LOGW("Recv comb cmds from mcu timeout!");
        for (auto i = 0; i < m_mcu_resps.size(); i++)
        {
            SmlkMcuCmdResp mcu_resp_temp;
            mcu_resp_temp.cmd_id = mcu_cmds[i].cmd_id;
            mcu_resp_temp.cmd_act = mcu_cmds[i].cmd_act;
            result.config_result.m_mcu_res_vec.emplace_back(mcu_resp_temp);
            result.config_result.m_tsp_result.emplace_back(SMLK_TSP_8F41_RESULT_ERROR);
        }
    }
    else
    {
        for (auto i = 0; i < m_mcu_resps.size(); i++)
        {
            SmlkMcuCmdResp mcu_resp_temp;
            mcu_resp_temp.cmd_id = mcu_cmds[i].cmd_id;
            mcu_resp_temp.cmd_act = mcu_cmds[i].cmd_act;
            result.config_result.m_mcu_res_vec.emplace_back(mcu_resp_temp);
            result.config_result.m_tsp_result.emplace_back(m_mcu_resps[i]);
        }
        m_mcu_resps.clear();
    }
    return SMLK_RC::RC_OK;
}
/**
 * @brief               根据8F51查询指令向车态查询数据
 * @param query_vec     8F51查询子指令,详见Jtt808RemoteCtrlQueryCmd
 * @param result_vec    查询结果vector,数据经过有效性校验,在8F51响应处理中直接获取结果
 *
 * @return
 */
SMLK_RC VehicleCtrlService::Process_8F51(IN vector<SMLK_UINT16> &query_vec, OUT vector<VehctrlGetCmd> &result_vec)
{
    std::vector<VehicleData> vd_vec;
    std::function<std::vector<VehicleData>(SMLK_UINT32, OUT std::vector<VehicleData>)>
        vd_pb = [this](SMLK_UINT32 vd_id, std::vector<VehicleData> vd_vec)
    {
        VehicleData vd;
        vd.index = vd_id;
        vd_vec.push_back(vd);
        return vd_vec;
    };
    bool isRecvPsQuery = SMLK_FALSE;
    for (SMLK_UINT8 i = 0; i < query_vec.size(); ++i)
    {
        switch (query_vec[i])
        {
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_DOOR: /*车门状态查询*/
        {
            vd_vec = vd_pb(VEHICLE_DATA_LOCK_STATUS_OF_DOOR_DRIVER, vd_vec);    /*驾驶侧车门开解锁状态*/
            vd_vec = vd_pb(VEHICLE_DATA_LOCK_STATUS_OF_DOOR_PASSENGER, vd_vec); /*乘客侧车门开解锁状态*/
            break;
        }
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_ENGINE: /*发动机状态查询*/
        {
            vd_vec = vd_pb(VEHICLE_DATA_ENGINE_SPEED, vd_vec); /*发动机转速*/
            break;
        }
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_POWER: /*车辆电源查询--特例:不从VD获取数据,直接读取本地IGN状态*/
        {
            VehctrlGetCmd getcmd;
            getcmd.query_id = SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_POWER_MODE;
            getcmd.value = (int)i_power_state;
            result_vec.push_back(getcmd);
            continue;
        }
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_ORIGINAL_AC: /*原车空调查询*/
        {
            vd_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS, vd_vec);           /*原车空调开关状态*/
            vd_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS, vd_vec);             /*原车空调出风模式*/
            vd_vec = vd_pb(VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE, vd_vec);            /*原车空调温度*/
            vd_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE, vd_vec);            /*原车空调风量*/
            vd_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS, vd_vec); /*原车空调循环模式*/
            vd_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS, vd_vec);          /*原车空调压缩机状态*/
            vd_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS, vd_vec);        /*原车空调auto状态*/
            vd_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS, vd_vec);      /*原车空调一键通风状态*/
            break;
        }
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_INDENTP_WARM_AIR: /*独立暖风状态查询*/
        {
            vd_vec = vd_pb(VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS, vd_vec); /*独立暖风开关*/
            vd_vec = vd_pb(VEHICLE_DATA_WARM_AIR_TEMPERATURE, vd_vec);          /*独立暖风温度*/
            break;
        }
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_PARKING_AC: /*驻车空调状态查询*/
        {
            vd_vec = vd_pb(VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS, vd_vec); /*驻车空调开关*/
            vd_vec = vd_pb(VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS, vd_vec);           /*驻车空调auto开关*/
            vd_vec = vd_pb(VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE, vd_vec);  /*驻车空调温度*/
            vd_vec = vd_pb(VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE, vd_vec);  /*驻车空调风量*/
            break;
        }
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_OILTANK_SECURITY: /*油箱防盗开关查询*/
        {
            vd_vec = vd_pb(VEHICLE_DATA_DRIVER_TANK_ANTI_THEFT_STATUS, vd_vec); /*油箱防盗开关*/
            break;
        }
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_INTELLIGENT_RUC: /*智能冷机状态查询*/
        {
            vd_vec = vd_pb(VEHICLE_DATA_TEMPERATURE_REMOTE_OPERATION_ALLOWED_STATE, vd_vec); /*冷机远程可控制状态*/
            vd_vec = vd_pb(VEHICLE_DATA_REFRIGERATION_SWITCH_STATUS, vd_vec);                /*制冷机组开关状态*/
            vd_vec = vd_pb(VEHICLE_DATA_DEFROST_ALLOWED_STATE, vd_vec);                      /*除霜允许状态*/
            vd_vec = vd_pb(VEHICLE_DATA_REFRIGERATION_WORK_MODE, vd_vec);                    /*除霜工作模式*/
            vd_vec = vd_pb(VEHICLE_DATA_RUC_REGULATION_TEMPERATURE, vd_vec);                 /*冷机当前设置的温度*/
            break;
        }
        case Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_CURRENT_PS: /*当前ps状态查询*/
        {
            isRecvPsQuery = SMLK_TRUE;
            vd_vec = vd_pb(VEHICLE_DATA_IGNITION_POSITION, vd_vec);           /*IGN位置信号*/
            vd_vec = vd_pb(VEHICLE_DATA_VEHICLE_REMOTE_CONTROL_MODE, vd_vec); /*远控模式激活状态*/
            break;
        }
        default:
            break;
        }
    }
    VehicleDataApi::GetInstance()->GetVehicleData(vd_vec);
    /*针对 PS 0x27 查询获取的两个状态量转换成单个状态量以便上报*/
    if (isRecvPsQuery)
    {
        VehCtrlPsIgnPos m_ign_pos;
        VehCtrlPsRemCtrlMode m_rctrl_mode;
        auto iter = vd_vec.begin();
        for (iter = vd_vec.begin(); iter != vd_vec.end();)
        {
            if (VEHICLE_DATA_IGNITION_POSITION == iter->index) /*ps IGN状态*/
            {
                if (iter->valid)
                {
                    m_ign_pos = (VehCtrlPsIgnPos)iter->value;
                }
                else
                {
                    m_ign_pos = (VehCtrlPsIgnPos)DATA_INVALID_UINT8;
                }
                iter = vd_vec.erase(iter);
            }
            else if (VEHICLE_DATA_VEHICLE_REMOTE_CONTROL_MODE == iter->index) /*ps 远控模式激活状态*/
            {
                if (iter->valid)
                {
                    m_rctrl_mode = (VehCtrlPsRemCtrlMode)iter->value;
                }
                else
                {
                    m_rctrl_mode = (VehCtrlPsRemCtrlMode)DATA_INVALID_UINT8;
                }
                iter = vd_vec.erase(iter);
            }
        }
        VehctrlGetCmd getcmd;
        getcmd.query_id = SMLK_VEHCTRL_CMD_PS_CTRL_MODE;
        if (VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_ACTIVE == m_rctrl_mode)
        {
            getcmd.value = (int)SmlkTspPsRctrlAvailableSatatus::SMLK_TSP_PS_AVAILABLE;
        }
        else if ((VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_DE_ACTIVE == m_rctrl_mode) && (VehCtrlPsIgnPos::VEH_CTRL_PS_IGN_OFF == m_ign_pos))
        {
            getcmd.value = (int)SmlkTspPsRctrlAvailableSatatus::SMLK_TSP_PS_AVAILABLE;
        }
        else
        {
            getcmd.value = (int)SmlkTspPsRctrlAvailableSatatus::SMLK_TSP_PS_UNAVAILABLE;
        }
        SMLK_LOGD("{[IGN]=%d [Mode]=%d [PsStatus]=%g}", m_ign_pos, m_rctrl_mode, getcmd.value);
        result_vec.push_back(getcmd);
    }
    /*将从VehicleData获取的数据转换成Mcu数据*/
    for (SMLK_UINT8 j = 0; j < vd_vec.size(); ++j)
    {
        VehctrlGetCmd getcmd;
        VdDataValidCheck(vd_vec[j].index, vd_vec[j].value, vd_vec[j].valid, getcmd);
        result_vec.push_back(getcmd);
    }
WITHOUT_VD_QUERY:
    /*如果在离线控车状态下查询需要保持CAN网络30s工作状态*/
    smartlink_sdk::MCU::GetInstance()->WakeUpCanBus();
    return SMLK_RC::RC_OK;
}
SMLK_RC VehicleCtrlService::ProcessRCtrlResult(IN RctrlResultQueue &result)
{
    auto iter = m_map_prot_dissect.find((SMLK_UINT8)(result.m_head.protocol));
    if (iter == m_map_prot_dissect.end())
    {
        SMLK_LOGD("Can't find prot=%d in dissector", (SMLK_UINT8)result.m_head.protocol);
        return SMLK_RC::RC_OK;
    }
    auto intf_func = iter->second;
    intf_func->ResultEncode(result);
    return SMLK_RC::RC_OK;
}

/*通用方法*/
/**
 * @brief               根据ipc head提取remcontrol head信息
 * @param ipc_head
 * @param rctrl_head
 *
 * @return
 */
void VehicleCtrlService::GetRctrlHeadFromIpcHead(IN IpcTspHead &ipc_head, OUT RctrlHead &rctrl_head)
{
    rctrl_head.msg_id = ipc_head.msg_id;
    rctrl_head.seq_id = ipc_head.seq_id;
    rctrl_head.protocol = (smartlink::ProtoclID)ipc_head.protocol;
    rctrl_head.qos = ipc_head.qos;
    rctrl_head.priority = ipc_head.priority;
}
/**
 * @brief                   车态数据合理性检测
 * @param vehicle_index
 * @param vehicle_data
 * @param vehicle_valid     车态数据有效性
 * @param get_cmd           车控查询指令
 *
 * @return
 */
SMLK_RC VehicleCtrlService::VdDataValidCheck(IN SMLK_UINT32 vehicle_index, IN SMLK_DOUBLE vehicle_data, IN SMLK_BOOL vechile_valid, OUT VehctrlGetCmd &get_cmd)
{
    get_cmd.query_id = vehicle_index; /*不针对VD的信息宏进行转换*/
    /*对数据进行有效性过滤*/
    switch (vehicle_index)
    {
    /*车门锁状态0x01*/
    case VEHICLE_DATA_LOCK_STATUS_OF_DOOR_DRIVER:    /*驾驶侧门*/
    case VEHICLE_DATA_LOCK_STATUS_OF_DOOR_PASSENGER: /*乘客侧门*/
    {
        if (DriveDoorStatus::E_DRIVE_DOOR_UNLOCKED == (DriveDoorStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_DOOR_UNLOCK;
        }
        else if (DriveDoorStatus::E_DRIVE_DOOR_LOCKED == (DriveDoorStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_DOOR_LOCK;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    /*发动机状态0x06*/
    case VEHICLE_DATA_ENGINE_SPEED: /*发动机转速*/
    {
        if (!vechile_valid)
        {
            get_cmd.value = (SMLK_DOUBLE)EngineMode::ENGINE_MODE_UNKNOWN;
        }
        else if ((0 < SMLK_UINT16(vehicle_data)) && (SMLK_UINT16(vehicle_data) < BOUNDARY_ENGINE_SPEED))
        {
            get_cmd.value = (SMLK_DOUBLE)EngineMode::ENGINE_MODE_OFF;
        }
        else
        {
            get_cmd.value = (SMLK_DOUBLE)EngineMode::ENGINE_MODE_ON;
        }
        break;
    }
    /*车辆电源状态0x07*/
    case SMLK_VEHCTRL_CMD_POWER_MODE:
    {
        get_cmd.value = SMLK_UINT8(i_power_state);
        break;
    }
    /*原车空调状态0x09*/
    case VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS: /*原车空调开关*/
    {
        if (AirConditionerStatus::E_AIR_CON_ON == (AirConditionerStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_ON;
        }
        else if (AirConditionerStatus::E_AIR_CON_OFF == (AirConditionerStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_OFF;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS: /*原车空调出风模式*/
    {
        if (AirConditionerMode::E_AIR_CON_FACE == (AirConditionerMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_BLOW_FACE;
        }
        else if (AirConditionerMode::E_AIR_CON_FOOT_AND_FACE == (AirConditionerMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_BLOW_FACE_FEET;
        }
        else if (AirConditionerMode::E_AIR_CON_FOOT == (AirConditionerMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_BLOW_FEET;
        }
        else if (AirConditionerMode::E_AIR_CON_FOOT_AND_DEFROST == (AirConditionerMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_BLOW_FEET_DEFROST;
        }
        else if (AirConditionerMode::E_AIR_CON_DEFROST == (AirConditionerMode)vehicle_data)
        {
            get_cmd.value = SMLK_VD_ORIAC_BLOW_DEFROST;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE: /*原车空调温度*/
    {
        if ((AC_LOWEST_TEMP > vehicle_data) || (AC_HIGHEST_TEMP < vehicle_data))
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        else
        {
            get_cmd.value = vehicle_data;
        }
        break;
    }
    case VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE: /*原车空调风量*/
    {
        if (((AirConditionerBlowingRate)vehicle_data >= AirConditionerBlowingRate::E_AIR_CON_LEVEL0) && ((AirConditionerBlowingRate)vehicle_data < AirConditionerBlowingRate::E_VEHICLE_DATA_MAX))
        {
            get_cmd.value = vehicle_data;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS: /*原车空调循环模式：内循环,外循环*/
    {
        get_cmd.value = SMLK_UINT8(vehicle_data);
        if (AirConditionerCirculationMode::E_AIR_CON_INTERNAL == (AirConditionerCirculationMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_AC_INNER_LOOP;
        }
        else if (AirConditionerCirculationMode::E_AIR_CON_EXTERNAL == (AirConditionerCirculationMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_AC_OUTER_LOOP;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS: /*原车空调压缩机*/
    {
        if (AirConditionerAcMode::E_AIR_CON_DEACTIVATE == (AirConditionerAcMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_OFF;
        }
        else if (AirConditionerAcMode::E_AIR_CON_ACTIVATE == (AirConditionerAcMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_ON;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS: /*原车空调auto开关*/
    {
        if (AirConditionerAutoMode::E_AIR_CON_DEACTIVATE == (AirConditionerAutoMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_OFF;
        }
        else if (AirConditionerAutoMode::E_AIR_CON_ACTIVATE == (AirConditionerAutoMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_ON;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS: /*原车空调一键通风*/
    {
        if (AirConditionerVentilationStatus::E_AIR_CON_DEACTIVATE == (AirConditionerVentilationStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_OFF;
        }
        else if (AirConditionerVentilationStatus::E_AIR_CON_ACTIVATE == (AirConditionerVentilationStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_ON;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    /*独立暖风状态0x10*/
    case VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS: /*独立暖风开关*/
    {
        if (WarmAirBlowerStatus::E_WARM_AIR_OFF == (WarmAirBlowerStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_OFF;
        }
        else if (WarmAirBlowerStatus::E_WARM_AIR_ON == (WarmAirBlowerStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_ON;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_WARM_AIR_TEMPERATURE: /*独立暖风温度*/
    {
        if (((WarmAirTempLevel)vehicle_data > WarmAirTempLevel::E_WARM_AIR_INACTIVE) && ((WarmAirTempLevel)vehicle_data < WarmAirTempLevel::E_VEHICLE_DATA_MAX))
        {
            get_cmd.value = (SMLK_DOUBLE)vehicle_data;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    /*驻车空调状态0x11*/
    case VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS: /*驻车空调开关*/
    {
        if (ParkAirConditionerStatus::E_PARK_AIR_CON_OFF == (ParkAirConditionerStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_OFF;
        }
        else if (ParkAirConditionerStatus::E_PARK_AIR_CON_ON == (ParkAirConditionerStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_ON;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS: /*驻车空调AUTO开关*/
    {
        get_cmd.value = SMLK_UINT8(vehicle_data);
        if (ParkAcAutoMode::E_PARK_AC_AUTO_DEACTIVATE == (ParkAcAutoMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_OFF;
        }
        else if (ParkAcAutoMode::E_PARK_AC_AUTO_ACTIVATE == (ParkAcAutoMode)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_ON;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE: /*驻车空调温度*/
        if ((AC_LOWEST_TEMP > vehicle_data) || (AC_HIGHEST_TEMP < vehicle_data))
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        else
        {
            get_cmd.value = vehicle_data;
        }
        break;
    case VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE: /*驻车空调风量*/
    {
        if (((ParkAirConditionerBlowingRate)vehicle_data >= ParkAirConditionerBlowingRate::E_PARK_AIR_CON_LEVEL0) && ((ParkAirConditionerBlowingRate)vehicle_data < ParkAirConditionerBlowingRate::E_VEHICLE_DATA_MAX))
        {
            get_cmd.value = SMLK_UINT8(vehicle_data);
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    /*油箱防盗开关0x12*/
    case VEHICLE_DATA_DRIVER_TANK_ANTI_THEFT_STATUS: /*油箱防盗报警开关*/
    {
        if (MailBoxStatus::E_MAIL_BOX_SECURITY_OFF == (MailBoxStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_OFF;
        }
        else if (MailBoxStatus::E_MAIL_BOX_SECURITY_ON == (MailBoxStatus)vehicle_data)
        {
            get_cmd.value = SMLK_TSP_ACTION_ON;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    /*唤醒源0x25 不支持主动查询 支持主动上报*/
    case SMLK_VEHCTRL_CMD_WAKEUP_SOURCE: /*内部定义唤醒源*/
    {
        get_cmd.value = SMLK_UINT8(i_wakeup_source);
        break;
    }
    /*智能冷机状态0x26 tsp上报数据跟Can网络数据完全一致*/
    case VEHICLE_DATA_TEMPERATURE_REMOTE_OPERATION_ALLOWED_STATE: /*冷机远程可控制状态*/
    case VEHICLE_DATA_REFRIGERATION_SWITCH_STATUS:                /*制冷机组开关状态*/
    case VEHICLE_DATA_DEFROST_ALLOWED_STATE:                      /*除霜允许状态*/
    case VEHICLE_DATA_REFRIGERATION_WORK_MODE:                    /*除霜工作模式*/
    {
        get_cmd.value = vehicle_data;
        break;
    }
    case VEHICLE_DATA_RUC_REGULATION_TEMPERATURE: /*冷机当前设置的温度*/
    {
        get_cmd.value = vehicle_data + RUC_REGULATION_TEMPERATURE_OFFSET;
        break;
    }
    /*当前ps状态0x27*/
    case VEHICLE_DATA_IGNITION_POSITION: /*ps IGN位置*/
    {
        if (PsIgnitionPosition::E_PS_IGN_POS_OFF == (PsIgnitionPosition)vehicle_data)
        {
            get_cmd.value = (int)VehCtrlPsIgnPos::VEH_CTRL_PS_IGN_OFF;
        }
        else if (PsIgnitionPosition::E_PS_IGN_POS_ACC == (PsIgnitionPosition)vehicle_data)
        {
            get_cmd.value = (int)VehCtrlPsIgnPos::VEH_CTRL_PS_IGN_ACC;
        }
        else if (PsIgnitionPosition::E_PS_IGN_POS_ON == (PsIgnitionPosition)vehicle_data)
        {
            get_cmd.value = (int)VehCtrlPsIgnPos::VEH_CTRL_PS_IGN_ON;
        }
        else if (PsIgnitionPosition::E_PS_IGN_POS_START == (PsIgnitionPosition)vehicle_data)
        {
            get_cmd.value = (int)VehCtrlPsIgnPos::VEH_CTRL_PS_IGN_START;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    case VEHICLE_DATA_VEHICLE_REMOTE_CONTROL_MODE: /*ps 车辆远控模式*/
    {
        if (PsRemCtrlMode::E_PS_REM_CTRL_MODE_DE_ACTIVE == (PsRemCtrlMode)vehicle_data)
        {
            get_cmd.value = (int)VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_DE_ACTIVE;
        }
        else if (PsRemCtrlMode::E_PS_REM_CTRL_MODE_ACTIVE == (PsRemCtrlMode)vehicle_data)
        {
            get_cmd.value = (int)VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_ACTIVE;
        }
        else
        {
            get_cmd.value = DATA_INVALID_UINT8;
        }
        break;
    }
    default:
        break;
    }
    if ((!vechile_valid) && (vehicle_index != SMLK_VEHCTRL_CMD_POWER_MODE) && (vehicle_index != SMLK_VEHCTRL_CMD_WAKEUP_SOURCE))
    {
        get_cmd.value = DATA_INVALID_UINT8;
    }
    // SMLK_LOGD("Vd{id=%d data=%g valid=%d}->Query{id=%d data=%g}", vehicle_index, vehicle_data, vechile_valid, get_cmd.query_id, get_cmd.value);
    return SMLK_RC::RC_OK;
}
/**
 * @brief           发送指令到mcu
 * @param mcu_cmd   mcu指令
 *
 * @return
 */
SMLK_RC VehicleCtrlService::SendCtrlCmdToMcu(IN SmlkMcuCmd &mcu_cmd)
{
    /*Mcu消息头组包*/
    vector<SMLK_UINT8> data_to_mcu;
    SmlkMcuCmdHead mcu_head;
    if (m_vehctrl_mcu_seq == 0xfffe) /*为了避免控车流水号和主动上报流水号0xffff冲突*/
        m_vehctrl_mcu_seq = 0x0000;
    else
        m_vehctrl_mcu_seq++;
    mcu_head.seq = htobe16(m_vehctrl_mcu_seq);
    mcu_head.cmd_num = 0x01;
    data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_head, (SMLK_UINT8 *)&mcu_head + sizeof(SmlkMcuCmdHead));
    /*Mcu消息体组包*/
    SmlkMcuCmd mcu_cmd_temp = mcu_cmd;
    mcu_cmd_temp.cmd_id = htobe16(mcu_cmd_temp.cmd_id);
    data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_cmd_temp, (SMLK_UINT8 *)&mcu_cmd_temp + SMLK_MCU_CMD_DEFAULT_LEN);
    data_to_mcu.insert(data_to_mcu.end(), mcu_cmd_temp.cmd_data_cont.data(), mcu_cmd_temp.cmd_data_cont.data() + mcu_cmd_temp.cmd_data_cont.size());
    std::string pszMcuMsg = tools::uint8_2_string(data_to_mcu.data(), data_to_mcu.size());
    SMLK_LOGD("[Mpu2Mcu]=%s", pszMcuMsg.c_str());
    smartlink_sdk::MCU::GetInstance()->SendVehicleCtrlCmd(data_to_mcu);
    return SMLK_RC::RC_OK;
}
/**
 * @brief           发送组合指令到mcu
 * @param mcu_cmd   mcu指令
 *
 * @return
 */
SMLK_RC VehicleCtrlService::SendCtrlCmdsToMcu(IN std::vector<SmlkMcuCmd> &mcu_cmds)
{
    /*Mcu消息头组包*/
    vector<SMLK_UINT8> data_to_mcu;
    SmlkMcuCmdHead mcu_head;
    if (m_vehctrl_mcu_seq == 0xfffe) /*为了避免控车流水号和主动上报流水号0xffff冲突*/
        m_vehctrl_mcu_seq = 0x0000;
    else
        m_vehctrl_mcu_seq++;
    mcu_head.seq = htobe16(m_vehctrl_mcu_seq);
    mcu_head.cmd_num = mcu_cmds.size();
    data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_head, (SMLK_UINT8 *)&mcu_head + sizeof(SmlkMcuCmdHead));
    /*Mcu消息体组包*/
    for (SMLK_UINT8 i = 0; i < mcu_head.cmd_num; ++i)
    {
        SmlkMcuCmd mcu_cmd_temp = mcu_cmds[i];
        mcu_cmd_temp.cmd_id = htobe16(mcu_cmd_temp.cmd_id);
        data_to_mcu.insert(data_to_mcu.end(), (SMLK_UINT8 *)&mcu_cmd_temp, (SMLK_UINT8 *)&mcu_cmd_temp + SMLK_MCU_CMD_DEFAULT_LEN);
        data_to_mcu.insert(data_to_mcu.end(), mcu_cmd_temp.cmd_data_cont.data(), mcu_cmd_temp.cmd_data_cont.data() + mcu_cmd_temp.cmd_data_cont.size());
    }
    std::string pszMcuMsg = tools::uint8_2_string(data_to_mcu.data(), data_to_mcu.size());
    SMLK_LOGD("[Mpu2Mcu]=%s", pszMcuMsg.c_str());
    smartlink_sdk::MCU::GetInstance()->SendVehicleCtrlCmd(data_to_mcu);
    return SMLK_RC::RC_OK;
}

/*****************************************************************************
 *                                业务需求                                    *
 *****************************************************************************/
/*消贷参数设置*/
void VehicleCtrlService::Process_8F41_FinancialLockFunc(IN void *data, IN int, OUT RctrlMsgQueue msg_queue)
{
    SMLK_LOGD("*******enter in func(%s)", __func__);

    msg_queue.m_head.msg_id = JTT808_MSG_REMOTE_CTRL;

    Msg8F41_Head head;
    Msg8F41_Body payload;
    bzero(&head, sizeof(Msg8F41_Head));
    bzero(&payload, sizeof(Msg8F41_Body));

    head.version = 0;
    head.cmd_num = 1;

    payload.id = JTT808_8F41_FINANCIAL_LOCK_CTRL;
    payload.param = 0;
    if (strncmp((char *)data, TEXT_MSG_FINANCIAL_LCK_FUNCTION_OPEN, strlen(TEXT_MSG_FINANCIAL_LCK_FUNCTION_OPEN)) == 0)
    {
        payload.cmd = JTT808_SUBCMD_FINANCIAL_LOCK_ON;
    }
    else if (strncmp((char *)data, TEXT_MSG_FINANCIAL_LCK_FUNCTION_CLOSE, strlen(TEXT_MSG_FINANCIAL_LCK_FUNCTION_CLOSE)) == 0)
    {
        payload.cmd = JTT808_SUBCMD_FINANCIAL_LOCK_OFF;
    }
    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&head, (SMLK_UINT8 *)&head + sizeof(Msg8F41_Head));
    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&payload, (SMLK_UINT8 *)&payload + sizeof(Msg8F41_Body));
    m_msg_type_from_queue.put(msg_queue);
}
void VehicleCtrlService::Process_8103_4GAnt(IN void *data, IN int, OUT RctrlMsgQueue msg_queue)
{
    SMLK_LOGD("*******enter in func(%s)", __func__);
    msg_queue.m_head.msg_id = JTT808_TERMINAL_PARAM_SET;

    jt808msg8103::Msg8013Head head;
    head.param_num = 1;

    jt808msg8103::ParamCommon param_info;
    param_info.param_id = htobe32(PARAM_4G_ANTENNA_OPEN_ID);
    param_info.param_len = 1;
    RCTRL_ANTENNA lock4G;
    if (strncmp((char *)data, TEXT_MSG_TBOX_4G_ANTENNA_LOCK, strlen(TEXT_MSG_TBOX_4G_ANTENNA_LOCK)) == 0)
    {
        lock4G = RCTRL_ANTENNA::ANTENNA_4G_LOCK;
    }
    else
    {
        lock4G = RCTRL_ANTENNA::ANTENNA_4G_UNLOCK;
    }
    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&head, (SMLK_UINT8 *)&head + sizeof(head));
    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&param_info, (SMLK_UINT8 *)&param_info + sizeof(param_info));
    msg_queue.m_message.insert(msg_queue.m_message.end(), (SMLK_UINT8 *)&lock4G, (SMLK_UINT8 *)&lock4G + sizeof(lock4G));
    m_msg_type_from_queue.put(msg_queue);
}

/*唤醒上报*/
/**
 * @brief   从休眠到wakeup状态,主动上报车态信息
 *
 * @return
 */
SMLK_RC VehicleCtrlService::WakeupSendVehicleStatus()
{
    SMLK_LOGD("enter in func(%s).", __func__);
    /*组织VehicleData数据查询并且转换*/
    std::vector<VehicleData> vehicle_data_vec;
    std::function<std::vector<VehicleData>(SMLK_UINT32, OUT std::vector<VehicleData>)>
        vd_pb = [this](SMLK_UINT32 vd_id, std::vector<VehicleData> vd_vec)
    {
        VehicleData vd;
        vd.index = vd_id;
        vd_vec.push_back(vd);
        return vd_vec;
    };
    /*车门锁状态0x01*/
    vehicle_data_vec = vd_pb(VEHICLE_DATA_LOCK_STATUS_OF_DOOR_DRIVER, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_LOCK_STATUS_OF_DOOR_PASSENGER, vehicle_data_vec);
    /*发动机状态0x06*/
    vehicle_data_vec = vd_pb(VEHICLE_DATA_ENGINE_SPEED, vehicle_data_vec);
    /*原车空调状态0x09*/
    vehicle_data_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS, vehicle_data_vec);
    /*独立暖风状态0x10*/
    vehicle_data_vec = vd_pb(VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_WARM_AIR_TEMPERATURE, vehicle_data_vec);
    /*驻车空调状态0x11*/
    vehicle_data_vec = vd_pb(VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS, vehicle_data_vec);
    /*油箱防盗开关0x12*/
    vehicle_data_vec = vd_pb(VEHICLE_DATA_DRIVER_TANK_ANTI_THEFT_STATUS, vehicle_data_vec);
    /*智能冷机状态0x26*/
    vehicle_data_vec = vd_pb(VEHICLE_DATA_TEMPERATURE_REMOTE_OPERATION_ALLOWED_STATE, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_REFRIGERATION_SWITCH_STATUS, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_DEFROST_ALLOWED_STATE, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_REFRIGERATION_WORK_MODE, vehicle_data_vec);
    vehicle_data_vec = vd_pb(VEHICLE_DATA_RUC_REGULATION_TEMPERATURE, vehicle_data_vec);
    VehicleDataApi::GetInstance()->GetVehicleData(vehicle_data_vec);
    SMLK_UINT8 cur_protocol = (SMLK_UINT8)ProtoclID::E_PROT_JTT808;

    auto iter = m_map_prot_dissect.find(cur_protocol);
    if (iter == m_map_prot_dissect.end())
    {
        SMLK_LOGE("can not find prot=%d in dissector", cur_protocol);
        return SMLK_RC::RC_ERROR;
    }
    RctrlResultQueue result;
    if (CUR_PROT_JTT808 == g_rctrl_protocol)
    {
        result.m_head.msg_id = JTT808_VEHICAL_QUERY_RESP;
        result.m_head.protocol = ProtoclID::E_PROT_JTT808;
    }
    else
    {
        result.m_head.protocol = ProtoclID::E_PROT_MAX;
    }
    result.m_head.qos = QOS_SEND_ALWAYS;
    result.m_head.priority = PRIORITY0;
    result.m_version = 0;
    result.m_cmd_from = VehCtrlCmdFrom::FROM_AUTO_REPORTED;
    result.m_result_type = RctrlCmdType::REMCTRL_CMD_GET;
    result.query_result.m_query_vec.clear();
    for (std::size_t i = 0; i < vehicle_data_vec.size(); ++i)
    {
        VehctrlGetCmd get_cmd;
        VdDataValidCheck(vehicle_data_vec[i].index, vehicle_data_vec[i].value, vehicle_data_vec[i].valid, get_cmd);
        result.query_result.m_query_vec.push_back(get_cmd);
    }
    /*组织除VD以外信息的主动上报*/
    /*终端唤醒源状态报告0x25*/
    VehctrlGetCmd wakeup_source;
    wakeup_source.query_id = SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_WAKEUP_SOURCE;
    wakeup_source.value = (SMLK_DOUBLE)i_wakeup_source;
    result.query_result.m_query_vec.push_back(wakeup_source);
    SMLK_LOGD("[WakeupSource]={id=%d data=%g}", result.query_result.m_query_vec[result.query_result.m_query_vec.size() - 1].query_id, result.query_result.m_query_vec[result.query_result.m_query_vec.size() - 1].value);
    /*车辆电源状态0x07*/
    VehctrlGetCmd power_state;
    power_state.query_id = SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_POWER_MODE;
    power_state.value = (SMLK_DOUBLE)i_power_state;
    result.query_result.m_query_vec.push_back(power_state);
    SMLK_LOGD("[PowerMode]={id=%d data=%g}", result.query_result.m_query_vec[result.query_result.m_query_vec.size() - 1].query_id, result.query_result.m_query_vec[result.query_result.m_query_vec.size() - 1].value);
    /*当前ps状态0x27*/
    VehctrlGetCmd ps_ctrl_mode;
    ps_ctrl_mode.query_id = SmlkRctrlInnerMsgID::SMLK_VEHCTRL_CMD_PS_CTRL_MODE;
    ps_ctrl_mode.value = (SMLK_DOUBLE)i_ps_state;
    result.query_result.m_query_vec.push_back(ps_ctrl_mode);
    SMLK_LOGD("[PsCtrlMode]={id=%d data=%g}", result.query_result.m_query_vec[result.query_result.m_query_vec.size() - 1].query_id, result.query_result.m_query_vec[result.query_result.m_query_vec.size() - 1].value);
    /*主动上报消息id收集*/
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_DOOR);
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_ENGINE);
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_POWER);
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_ORIGINAL_AC);
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_INDENTP_WARM_AIR);
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_PARKING_AC);
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_OILTANK_SECURITY);
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_WAKEUP_SOURCE);
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_INTELLIGENT_RUC);
    result.query_result.m_query_id.push_back(Jtt808RemoteCtrlQueryCmd::JTT808_QUERY_CURRENT_PS);
    shared_ptr<IDissector> dissector = iter->second;
    SMLK_RC rc = dissector->ResultEncode(result);
    return rc;
}
/**
 * @brief   从休眠到wakeup状态,根据上次配置信息下发怠速暖机控制信息
 *
 * @return
 */
SMLK_RC VehicleCtrlService::WakeupSendIdlingWarmMsg()
{
    SMLK_LOGD("enter in func(%s).", __func__);
    SMLK_RC rc = SMLK_RC::RC_OK;
    std::string warmup_status_str;
    rc = VehCtrlConfigJsonFile::GetInstance()->GetValueFromJsonFile(VEHICLE_CTRL_IDLING_WARM_UP_STATUS, warmup_status_str);
    SmlkMcuCmd mcu_cmd;
    mcu_cmd.cmd_data_len = 0;
    if (SMLK_RC::RC_OK == rc)
    {
        if ((SMLK_UINT8)(RctrlStatusIdlingWarmUp::STATUS_NONE) == atoi(warmup_status_str.c_str()))
        {
            SMLK_LOGD("No need to send.");
            return SMLK_RC::RC_OK;
        }
        else if ((SMLK_UINT8)(RctrlStatusIdlingWarmUp::STATUS_CLOSE) == atoi(warmup_status_str.c_str()))
        {
            mcu_cmd.cmd_id = SMLK_MCU_CMD_IDLING_WARM_UP;
            mcu_cmd.cmd_act = SmlkMcuMsgIdleWarmUpMode::SMLK_MCU_ACTION_IDLING_WARM_UP_CLOSE;
        }
        else if ((SMLK_UINT8)(RctrlStatusIdlingWarmUp::STATUS_OPEN_LEV_1) == atoi(warmup_status_str.c_str()))
        {
            mcu_cmd.cmd_id = SMLK_MCU_CMD_IDLING_WARM_UP;
            mcu_cmd.cmd_act = SmlkMcuMsgIdleWarmUpMode::SMLK_MCU_ACTION_AC_IDLING_WARM_UP_LEVEL_1;
        }
        else if ((SMLK_UINT8)(RctrlStatusIdlingWarmUp::STATUS_OPEN_LEV_2) == atoi(warmup_status_str.c_str()))
        {
            mcu_cmd.cmd_id = SMLK_MCU_CMD_IDLING_WARM_UP;
            mcu_cmd.cmd_act = SmlkMcuMsgIdleWarmUpMode::SMLK_MCU_ACTION_AC_IDLING_WARM_UP_LEVEL_2;
        }
    }
    else
    {
        return SMLK_RC::RC_ERROR;
    }
    SendCtrlCmdToMcu(mcu_cmd);
    unique_lock<std::mutex> lck(m_cmd_mtx);
    SMLK_LOGD("Idle Warm_up rdy to wait for %ds", cmd_load.m_defalut_timeout);
    if (m_cmd_cv.wait_for(lck, std::chrono::seconds(cmd_load.m_defalut_timeout)) == std::cv_status::timeout)
    {
        SMLK_LOGW("Idle Warm_up timeout! Send msg next time!");
        return SMLK_RC::RC_ERROR;
    }
    else
    {
        VehCtrlConfigJsonFile::GetInstance()->SetValueToJsonFile(VEHICLE_CTRL_IDLING_WARM_UP_STATUS, to_string(((SMLK_UINT8)(RctrlStatusIdlingWarmUp::STATUS_NONE))));
        return SMLK_RC::RC_OK;
    }
}

/*初始化主动上报*/
/**
 * @brief   初始化更新本地ps控制模式状态
 *
 * @return
 */
void VehicleCtrlService::InitUpdatePsCtrlMode()
{
    std::vector<VehicleData> vd_vec;
    vd_vec.push_back(VEHICLE_DATA_IGNITION_POSITION);
    vd_vec.push_back(VEHICLE_DATA_VEHICLE_REMOTE_CONTROL_MODE);
    VehicleDataApi::GetInstance()->GetVehicleData(vd_vec);
    if (!vd_vec[0].valid)
    {
        SMLK_LOGD("Ps signal invalid, ps ctrl mode remain unavailable.");
        return;
    }
    PsCtrlMode pc_mode_temp;
    if ((VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_ACTIVE == (VehCtrlPsRemCtrlMode)vd_vec[1].value) /*ps远控模式*/
        || ((VehCtrlPsRemCtrlMode::VEH_CTRL_PS_REM_CTRL_MODE_DE_ACTIVE == (VehCtrlPsRemCtrlMode)vd_vec[1].value) && (PsIgnitionPosition::E_PS_IGN_POS_OFF == (PsIgnitionPosition)vd_vec[0].value)))
        pc_mode_temp = PsCtrlMode::PS_MODE_AVAILABLE;
    else
        pc_mode_temp = PsCtrlMode::PS_MODE_UNAVAILABLE;
    if (i_ps_state != pc_mode_temp)
    {
        SMLK_LOGD("[PsStatueChanged]:[%s]-->[%s]", (i_ps_state == PsCtrlMode::PS_MODE_UNKNOWN) ? "PS UNKNOWN" : (i_ps_state == PsCtrlMode::PS_MODE_AVAILABLE) ? "PS AVAILABLE"
                                                                                                                                                              : "PS UNAVAILABLE",
                  (pc_mode_temp == PsCtrlMode::PS_MODE_UNKNOWN) ? "PS UNKNOWN" : (pc_mode_temp == PsCtrlMode::PS_MODE_AVAILABLE) ? "PS AVAILABLE"
                                                                                                                                 : "PS UNAVAILABLE");
        i_ps_state = pc_mode_temp;
    }
    else
        return;
}

/*备案开关*/
/**
 * @brief   根据国6备案json处理上报逻辑
 *
 * @return
 */
void VehicleCtrlService::OnRecvGBRecRes(std::string &json)
{
    SMLK_LOGD("[GbRecRes]=%s", json.c_str());
    Poco::JSON::Parser parser;
    Poco::Dynamic::Var var_json;
    /*判断参数json合法性*/
    try
    {
        var_json = parser.parse(json.c_str());
    }
    catch (Poco::Exception &exc)
    {
        SMLK_LOGE("Parse Error:%s", exc.displayText().c_str());
        return;
    }
    /*判断json中RecRes字段是否存在且数据类型为int*/
    Poco::JSON::Object obj = *var_json.extract<Poco::JSON::Object::Ptr>();
    Poco::Dynamic::Var rec_res = obj.get("RecSta");
    if (!rec_res.isInteger())
    {
        SMLK_LOGW("JSON \"RecSta\" type error!");
        return;
    }
    SMLK_UINT8 m_rec_sta = rec_res.convert<SMLK_UINT8>();
    if ((GB_REC_STATUS::GB_REC_CLOSE_FINISH == m_rec_sta) || (GB_REC_STATUS::GB_REC_OPEN_FINISH == m_rec_sta))
    {
        Poco::Dynamic::Var sn = obj.get("SN");
        if (!sn.isString())
        {
            SMLK_LOGW("JSON \"SN\" type error!");
            return;
        }
        std::string m_sn = sn.convert<std::string>();
        Poco::Dynamic::Var res = obj.get("RecRes");
        if (!res.isInteger())
        {
            SMLK_LOGW("JSON \"RecRes\" type error!");
            return;
        }
        SMLK_UINT8 m_res = res.convert<SMLK_UINT8>();
        std::vector<SMLK_UINT8> output_vec;
        Msg0F42GetCommon getcommon;
        getcommon.seq_id = htobe16(std::atoi(m_sn.c_str()));
        getcommon.cmd_id = 0x0f;
        getcommon.length = htobe16(0x0001);
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&getcommon, (SMLK_UINT8 *)&getcommon + sizeof(Msg0F42GetCommon)); //组入包头
        output_vec.insert(output_vec.end(), m_res);
        RctrlHead head_temp;
        head_temp.protocol = smartlink::ProtoclID::E_PROT_JTT808;
        head_temp.msg_id = JTT808_GET_SET_TBOX_RESP;
        head_temp.qos = QOS_SEND_TCP_TIMES;
        head_temp.seq_id = htobe16(std::atoi(m_sn.c_str()));
        head_temp.priority = 0x00;
        DissectorJtt808Common::getInstance()->DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    }
    else
        SMLK_LOGD("Recv status hasn't completely finished!");
    return;
}

/*仪表盘保养*/
/**
 * @brief   当TSP成功连接上做主动上报
 *
 * @return  SMLK_RC
 */
SMLK_RC VehicleCtrlService::OnTspConnectActiveReport()
{
    SMLK_LOGD("Conn 2 tsp, do send 8F42:0X18");
    SMLK_RC rc = SMLK_RC::RC_OK;
    /*主动上报 0F42 -- 0X18:仪表盘保养提醒信息*/
    vector<SMLK_UINT8> output_vec;
    Msg0F42GetCommon t1;
    t1.seq_id = 0xffff;
    t1.cmd_id = 0x18;
    t1.length = htobe16(0x0011);
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&t1, (SMLK_UINT8 *)&t1 + sizeof(Msg0F42GetCommon));
    SMLK_UINT8 emp_buf[VIN_LENGTH] = {0 * VIN_LENGTH};
    SMLK_UINT8 reserve_buf[4] = {0 * 4};
    std::string vin;
    std::string property = SYS_PRO_NAME_VEHICLE_VIN;
    smartlink_sdk::RtnCode prop_rc = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, vin);
    if (smartlink_sdk::RtnCode::E_SUCCESS != prop_rc)
    {
        SMLK_LOGI("In %s fail to get prop=%s, set vin empty.", __func__, property.c_str());
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&emp_buf[0], (SMLK_UINT8 *)&emp_buf[0] + VIN_LENGTH);
    }
    else
    {
        // SMLK_LOGD("Get vin=%s", vin.c_str());
        output_vec.insert(output_vec.end(), vin.begin(), vin.end());
    }
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&reserve_buf[0], (SMLK_UINT8 *)&reserve_buf[0] + 4);
    std::string log = tools::uint8_2_string(output_vec.data(), output_vec.size());
    RctrlHead head_temp;
    head_temp.seq_id = 0xffff;
    head_temp.qos = QOS_SEND_ALWAYS;
    head_temp.msg_id = JTT808_GET_SET_TBOX_RESP;
    head_temp.protocol = smartlink::ProtoclID::E_PROT_JTT808;
    rc = DissectorJtt808Common::getInstance()->DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)output_vec.data(), output_vec.size());
    return rc;
}
/**
 * @brief   注册tsp成功连接事件主动上报(单写方法是为了避免代码冗余)
 *
 * @return  SMLK_RC
 */
void VehicleCtrlService::RegisterTspConnectActiveReport()
{
    /*Tsp成功连接后主动上报*/
    std::vector<TspEventId> tsp_event_id = {
        TspEventId::E_TSP_EVENT_CONNECT_STATE,
    };
    auto tsp_conn_rc = TspServiceApi::getInstance()->RegEventCB(tsp_event_id, [this](TspEventId id, void *data, int len)
                                                                {
        SMLK_LOGD("Recv tsp event %d", id);
        switch (id) {
          case TspEventId::E_TSP_EVENT_CONNECT_STATE: {
            TspConnectState *state = (TspConnectState *)data;
            SMLK_LOGI("Recv tsp connected state=%d", *state);
            if (TspLoginState::E_LOGIN == (TspLoginState)*state) {
              SMLK_RC conn_rc;
              do {
                conn_rc = OnTspConnectActiveReport();
              } while (SMLK_RC::RC_OK != conn_rc);
            }
            break;
          }
          default:
            break;
        } });
    if (SMLK_RC::RC_OK != tsp_conn_rc)
    {
        SMLK_LOGE("Fail to register TSP service event callback");
    }
}
/**
 * @brief   发送仪表盘保养信息
 *
 * @return
 */
void VehicleCtrlService::SendDBMtnInfo(SMLK_UINT32 index)
{
    std::lock_guard<std::mutex> lck(m_db_mtn_mutex);
    MsgCan18FC174A t;
    int type_idx = ((index - 10) / 10) % 4;
    switch (m_db_send_type)
    {
    case DashBoardMaintainSendStrategy::DB_MTN_SEND_DEFAULT:
    {
        t.p1 = 0xff;
        t.p2 = 0xffff;
        t.p3 = 0xff;
        t.p4 = 0xffffffff;
        break;
    }
    case DashBoardMaintainSendStrategy::DB_MTN_SEND_SPECIFIC:
    {
        t.p1 = (m_db_mtn_require[type_idx] << 4) | (type_idx + 1);
        t.p2 = 0xffff;
        t.p3 = 0xff;
        t.p4 = htobe32(m_db_mileage[type_idx]);
        break;
    }
    default:
        return;
    }
    CanFrame can_data;
    can_data.can_dlc = 8;
    can_data.can_id = PROP_DASHBOARD_MAINT_CAN_ID | 0x80000000;
    memcpy(can_data.data, &t, 8);
    MCU::GetInstance()->SendCanFrame(CHANNEL_COMM, &can_data, true);
}
/**
 * @brief   仪表盘保养信息解析
 *
 * @return
 */
void VehicleCtrlService::Process_8F42_0x19(IN SMLK_UINT8 *indata, IN size_t len)
{
    std::lock_guard<std::mutex> lck(m_db_mtn_mutex);
    m_db_send_type = DashBoardMaintainSendStrategy::DB_MTN_SEND_SPECIFIC;
    m_db_mileage.clear();
    m_db_mtn_require.clear();
    indata += sizeof(Msg8F42Common);
    //存储待保养发动机,变速箱,后桥,转向机里程.收到消息后每种类型轮流发送,持续5m
    //有效保养里程-50000后下发,无保养里程下发7FFFFFFF
    for (auto i = 0; i < 4; ++i)
    {
        SMLK_UINT32 t = 0;
        for (auto j = 0; j < 4; ++j)
        {
            t = (t << 8U);
            t |= (SMLK_UINT8)*indata;
            indata++;
        }
        if (0x7FFFFFFF == t)
        {
            m_db_mileage.emplace_back(t);
        }
        else
        {
            m_db_mileage.emplace_back(t - 50000U);
        }
        SMLK_LOGD("[%dth][mileage]:0x%08x", i, m_db_mileage[i]);
    }
    for (auto i = 0; i < 4; ++i)
    {
        SMLK_UINT8 t = 0;
        t = (0x7FFFFFFF == m_db_mileage[i]) ? 0 : 1;
        m_db_mtn_require.emplace_back(t);
        SMLK_LOGD("[%dth][mileage][valid]:%d", i, m_db_mtn_require[i]);
    }
    SMLK_LOGD("[DashBoardTimer][SendStrategy]=Specific");
}
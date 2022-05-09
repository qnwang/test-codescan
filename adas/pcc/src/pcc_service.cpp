#include <string>
#include <fstream>
#include <dlfcn.h>

#include <smartlink_sdk_location.h>
#include <smartlink_sdk_mcu.h>
#include <smartlink_sdk_tel.h>
#include <smartlink_sdk_sys_property.h>

#include "pcc_service.h"
#include "tsp_service_api.h"
#include "ipc_api_def.h"
#include "custom_util.hpp"
#include "smartlink_sdk_sys_property_def.h"

using namespace smartlink_sdk;
using namespace smartlink;

const std::string g_str_conf_path = "/userdata/map/AV2HP.conf";
const std::string g_str_license_path = "/userdata/map/adas.license";
const std::string g_str_map_data_path = "/userdata/map/adasdata/ADASRoute.dat";

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond_vcu = PTHREAD_COND_INITIALIZER;
static pthread_cond_t cond_map = PTHREAD_COND_INITIALIZER;

static const std::map<SMLK_UINT32, VehicleType> g_map_vehicle_type = {
    {0x00, VehicleType::EM_J6_HIGH},
    {0x01, VehicleType::EM_J7_HIGH},
    {0x02, VehicleType::EM_J6_LOW },
    {0x03, VehicleType::EM_J6_LOW },
    {0x04, VehicleType::EM_J6_HIGH},
    {0x05, VehicleType::EM_J6_HIGH},
    {0x06, VehicleType::EM_J7_HIGH},
    {0x07, VehicleType::EM_J6_LOW },
    {0x08, VehicleType::EM_J6_LOW },
    {0x09, VehicleType::EM_J6_HIGH},
    {0x0A, VehicleType::EM_EV     },
    {0x0B, VehicleType::EM_EV     },
    {0x0C, VehicleType::EM_QINGQI },
    {0x0D, VehicleType::EM_QINGQI }
};

static const std::map<VehicleType, SMLK_UINT8> g_map_vehicle_power_can_channle = {
    {VehicleType::EM_J6_LOW,   0},
    {VehicleType::EM_J6_HIGH,  2},
    {VehicleType::EM_J7_HIGH,  0},
    {VehicleType::EM_EV,       2},
    {VehicleType::EM_QINGQI,   0}
};

/*AV2Msg *********************************/
typedef struct HRBuffer
{
    uint8_t     *m_buffer;
    int32_t      m_length;
}HRBuffer;

typedef struct HRNetDataHirain
{
    uint8_t     m_buffer[8];
}HRNetDataHirain;

#define AV2_MSG_TYPE_SYSTEM_SPECIFIC	( 0 )
#define AV2_MSG_TYPE_POSITION			( 1 )
#define AV2_MSG_TYPE_SEGMENT			( 2 )
#define AV2_MSG_TYPE_STUB				( 3 )
#define AV2_MSG_TYPE_PROFILE_SHORT		( 4 )
#define AV2_MSG_TYPE_PROFILE_lONG		( 5 )
#define AV2_MSG_TYPE_META_DATA			( 6 )
#define AV2_MSG_TYPE_RESERVED			( 7 )
/**********************************/


const av2hp_gpsInfo  av2hp_dummy_gpsInfos[10]  =
{    //m_valid  // m_dateTime        // m_timestamp // m_pos（度格式）                   // m_orient  // m_speed      // m_gpsQuality // m_hdop,m_pdop, m_vdop, m_satInViewNum, m_satNum  // m_satellites(no data)
	{ 1, { 9, 1, 58, 2018, 8, 8, }, 0, { av2hp_coordinate_WGS84, 111.1430032, 32.505295, 0 }, 103.45, { av2hp_speedUnit_ms, 14.65395 }, 0, 0, 1, 0, 12, 9 },
	{ 1, { 9, 1, 59, 2017, 8, 8, }, 0, { av2hp_coordinate_WGS84, 111.143156,  32.50526383, 0 }, 103.43, { av2hp_speedUnit_ms, 14.81908667 }, 0, 0, 1, 0, 12, 9 },
	{ 1, { 9, 2, 0, 2017, 8, 8, }, 0,  { av2hp_coordinate_WGS84, 111.1433102, 32.5052315, 0 }, 103.94, { av2hp_speedUnit_ms, 14.97496333 }, 0, 0, 1, 0, 12, 9 },
	{ 1, { 9, 2, 1, 2017, 8, 8, }, 0,  { av2hp_coordinate_WGS84, 111.1434657, 32.50519733, 0 }, 104.48, { av2hp_speedUnit_ms, 15.14164333 }, 0, 0, 1, 0, 12, 9 },
	{ 1, { 9, 2, 2, 2017, 8, 8, }, 0,  { av2hp_coordinate_WGS84, 111.1436228, 32.50516217, 0 }, 104.97, { av2hp_speedUnit_ms, 15.37880222 }, 0, 0, 1, 0, 12, 9 },
	{ 1, { 9, 2, 3, 2017, 8, 8, }, 0,  { av2hp_coordinate_WGS84, 111.1437825, 32.50512483, 0 }, 105.35, { av2hp_speedUnit_ms, 15.62727889 }, 0, 0, 1, 0, 12, 9 },
	{ 1, { 9, 2, 4, 2017, 8, 8, }, 0,  { av2hp_coordinate_WGS84, 111.1439442, 32.50508667, 0 }, 105.71, { av2hp_speedUnit_ms, 15.88038556 }, 0, 0, 1, 0, 12, 9 },
	{ 1, { 9, 2, 5, 2017, 8, 8, }, 0,  { av2hp_coordinate_WGS84, 111.1441087, 32.50504683, 0 }, 105.92, { av2hp_speedUnit_ms, 16.15921444 }, 0, 0, 1, 0, 12, 9 },
	{ 1, { 9, 2, 6, 2017, 8, 8, }, 0,  { av2hp_coordinate_WGS84, 111.1442745, 32.50500633, 0 }, 105.97, { av2hp_speedUnit_ms, 16.27650778 }, 0, 0, 1, 0, 12, 9 },
	{ 1, { 9, 2, 7, 2017, 8, 8, }, 0,  { av2hp_coordinate_WGS84, 111.1444413, 32.50496583, 0 }, 105.89, { av2hp_speedUnit_ms, 16.33309667 }, 0, 0, 1, 0, 12, 9 },
};

void av2hp_getGpsInfo(size_t index, av2hp_gpsInfo* gpsinfo)
{
    if (index > 6)
        index = 6;

    if (NULL != gpsinfo)
    {
        *gpsinfo = av2hp_dummy_gpsInfos[index];
    }
}


PccService::PccService()
{
    m_b_ign_on = false;
    m_b_vehicle_is_qingdao = false;
    m_n_power_can_channle = 0;
}

PccService::~PccService()
{

}

/******************************************************************************
* NAME: Init
*
* DESCRIPTION: 初始化sdk相关代码
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
SMLK_UINT32 PccService::Init()
{
    //初始化property-sdk
    if (!smartlink_sdk::SysProperty::GetInstance()->Init(ModuleID::E_MOUDLE_adas_service)) {
        SMLK_LOGE("Init Property failed....");
        return M_L_RETURN_FAILURE;
    }
    const std::vector<std::string> propertys = {
        std::string(SYS_PRO_NAME_VCU)
    };

    smartlink_sdk::SysProperty::GetInstance()->RegEventCB(propertys,
        [this](smartlink_sdk::SysProperty::PropertyEventId id, smartlink_sdk::PropertyInfo* info){

            if (info == nullptr)
            {
                SMLK_LOGE("[property] PropertyInfo ptr is nullptr!!!");
                return;
            }
            switch(id)
            {
                case smartlink_sdk::SysProperty::PropertyEventId::E_SYS_PROPERTY_EVENT_VALUE_CHANGED:
                    {
                        if (info->name == std::string(SYS_PRO_NAME_VCU))
                        {
                            bool b_is_qingdao = false;
                            if(CheckCurIsSupportPcc(b_is_qingdao))
                            {

                                m_b_vehicle_is_qingdao = b_is_qingdao;
                                ResumeMainThread(cond_vcu); //恢复主线程
                            }
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    );
    //挂起时， 仅注册监控property-sdk, 其他的sdk初始化都在条件判断之后。
    if (LoopWaitCondition()  == M_L_RETURN_FAILURE)
    {
        return M_L_RETURN_FAILURE;
    }

    //初始化GPS相关
    if ( !Location::GetInstance()->Init(ModuleID::E_MOUDLE_adas_service) ) {
        SMLK_LOGE("fail to init location service API interface!!!");
        return M_L_RETURN_FAILURE;
    }

    std::vector<LocationEventId> loc_events = {
        LocationEventId::E_LOCATION_EVENT_ISREADY,
        LocationEventId::E_LOCATION_EVENT_EXIT,
        LocationEventId::E_LOCATION_EVENT_GNSS_ANTENNA_CHANGED,
        LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED,
        LocationEventId::E_LOCATION_EVENT_SATELLITE_INFO_CHANGED
    };

    auto return_code = Location::GetInstance()->RegEventCallback(
        loc_events,
        [this](LocationEventId id, void *data, int len){
            MessageHead head(M_L_MSG_LOCATION_NET, 0, 0);
            std::vector<SMLK_UINT8> bytes;
            bytes.clear();

            switch(id)
            {
                case LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED:
                    {
                        SMLK_LOGD("[adad_gps] ===========E_LOCATION_EVENT_FIX_INFO_CHANGED \n");
                        if ( sizeof(smartlink_sdk::LocationInfo) != (std::size_t)len ) {
                                SMLK_LOGE("[adad_gps] unexpected body length for FIX INFO CHNAGED notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::LocationInfo));
                                return;
                        }
                        smartlink_sdk::LocationInfo *info = (smartlink_sdk::LocationInfo *)data;
                        bytes.insert(bytes.end(), (SMLK_UINT8 *)info, ((SMLK_UINT8 *)info + len));
                        head.m_seq = M_L_MSG_LOCATION_FIX_INFO_CHANGE;
                        head.m_extra   = sizeof(smartlink_sdk::LocationInfo);
                        SMLK_LOGD("get init gps: time: %llu,  m_valid : %d m_lon: %f, m_lat : %f, m_alt: %f, m_speed.m_value: %f, m_hdop: %f, m_pdop:%f, m_vdop:%f", info->utc, info->fix_valid, info->longitude, info->latitude,info->speed, info->altitude, info->pdop, info->hdop, info->vdop);
                    }
                    break;
                case LocationEventId::E_LOCATION_EVENT_SATELLITE_INFO_CHANGED:
                    {
                        SMLK_LOGD("[adad_gps]=============== E_LOCATION_EVENT_SATELLITE_INFO_CHANGED \n");

                        smartlink_sdk::GnssSatelliteInfo *info = reinterpret_cast<smartlink_sdk::GnssSatelliteInfo *>(data);
                        bytes.insert(bytes.end(), (SMLK_UINT8 *)info, ((SMLK_UINT8 *)info + 2));

                        head.m_seq = M_L_MSG_LOCATION_SATELLITE_INFO_CHANGE;
                        head.m_extra = 2;
                    }
                    break;
                default:
                    break;
            }
            Post(head, bytes);
    });
    if (RtnCode::E_SUCCESS != return_code)
    {
        SMLK_LOGW("Location::GetInstance()->RegEventCallback err");
        return M_L_RETURN_FAILURE;
    }

    //初始化MCU-SDK
    if ( RtnCode::E_SUCCESS != MCU::GetInstance()->Init(ModuleID::E_MOUDLE_adas_service) ) {
        SMLK_LOGE("fail to init location service API interface!!!");
        return M_L_RETURN_FAILURE;
    }

    std::vector<smartlink_sdk::McuEventId>  mcu_events = {
        smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE,
    };

    return_code = smartlink_sdk::MCU::GetInstance()->RegEventCB(
        mcu_events,
        [this](smartlink_sdk::McuEventId id, void *data, int len) {
            MessageHead head(M_L_MSG_MCU_NET, 0, 0);
            std::vector<SMLK_UINT8> bytes;
            bytes.clear();
            bool m_b_is_post_msg = false;

            switch ( id ) {
                case smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE:
                    {
                        if ( sizeof(smartlink_sdk::IGNState) != (std::size_t)len ) {
                            SMLK_LOGE("[MCU] unexpected body length for MCU IGN state changed notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::IGNState));
                            return;
                        }

                        smartlink_sdk::IGNState *ign = reinterpret_cast<smartlink_sdk::IGNState *>(data);
                        if (ign->on)
                        {
                            if (!m_b_ign_on)
                            {
                                m_b_ign_on = true;
                                head.m_seq = M_L_MSG_MCU_IGN_ON;
                                m_b_is_post_msg = true;
                            }
                        }
                        else
                        {
                            m_b_ign_on = false;
                        }
                    }
                    break;
                default:
                    break;
            }
            if (m_b_is_post_msg)  Post(head, std::move(bytes));
    });

    //初始化Tel-sdk
    if (!Telephony::GetInstance()->Init(ModuleID::E_MOUDLE_adas_service)) {
        SMLK_LOGE("fail to init Telephony service API interface!!!");
        return M_L_RETURN_FAILURE;
    }

    //初始化TSP相关
    if ( SMLK_RC::RC_OK != TspServiceApi::getInstance()->Init(ModuleID::E_MOUDLE_adas_service) ) {

        SMLK_LOGE("fail to init TSP service API interface!!!");
        return M_L_RETURN_FAILURE;
    }

    auto ret = TspServiceApi::getInstance()->RegisterMsgCB(
        ProtoclID::E_PROT_JTT808,
        {M_L_TSP_8F42_REQ},
        [this](IN IpcTspHead &ipc, IN void *data, std::size_t sz){

            if (data == nullptr)
            {
                SMLK_LOGE("[tsp] data is nullptr!!!");
                return;
            }

            SMLK_LOGI("[tsp] rcv tsp msg[0x%04x]", ipc.msg_id);

            switch (ipc.msg_id)
            {
                case M_L_TSP_8F42_REQ:
                    Post(MessageHead(M_L_MSG_TSP_NET, ipc.seq_id, ipc.msg_id), (SMLK_UINT8*)data, sz);
                    break;
                default:
                    SMLK_LOGW("[tsp] received unregist message!!!");
                    break;
            }
        }
    );
    if (SMLK_RC::RC_OK != ret)
    {
        SMLK_LOGW("TspServiceApi::getInstance()->RegisterMsgCB err");
        return M_L_RETURN_FAILURE;
    }

    //初始化定时器
    m_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 200>>(
            [this](SMLK_UINT32 index){
                {
                    MessageHead head(M_L_MSG_TIMER_NET, M_L_MSG_TIMER_200ms, 0);
                    Post(head, nullptr, 0);
                }
            }
    );

   return M_L_RETURN_SUCCESS;
}

/******************************************************************************
* NAME: Start
*
* DESCRIPTION: 程序运行
*
* PARAMETERS:
*
* RETURN:
*
* NOTE: 动态加载adas库, 青岛库为libadasisv2hp_qingdao.so， 非青岛为libadasisv2hp.so
*****************************************************************************/
SMLK_UINT32 PccService::Start()
{
    //初始化车型，获取动力CAN-channle
    InitVehicleType();

    // //重新设置GPS采集频率    //暂去除
    // uint8_t frequency = 0;
    // Location::GetInstance()->Setfrequency(200);
    // Location::GetInstance()->Getfrequency(frequency);
    // SMLK_LOGI("===========Getfrequency:: %d \n", frequency);


    //初始化map相关
    char mapVersion[10] = { 0 };
    m_p_Av2HP_getMapVersion(const_cast<char*>(g_str_map_data_path.c_str()), mapVersion, 10);

    SMLK_LOGD( "Map version: %s \n", mapVersion);

    char softVersion[100] = { 0 };
    m_p_Av2HP_getSoftVersion(softVersion, 100);

    SMLK_LOGD("SOFT version: %s \n", softVersion);

    int ret = m_p_Av2HP_init(g_str_conf_path.c_str());

    if(IAV2HP_SUCCESS != ret)
    {
        m_p_Av2HP_destory();

        SMLK_LOGD("av2hp:init failed, the most likely reason is wrong configuration(working directory is not right or no map database)\n");
        return M_L_RETURN_FAILURE;
    }

    std::string str_imsi;
    if (RtnCode::E_SUCCESS != Telephony::GetInstance()->GetIMSI(str_imsi))
    {
        SMLK_LOGW("[adas_init]  GetIMSI err");
    }
    str_imsi = "460099023776882";               //pcc测试设备ID
    m_p_Av2HP_setDeviceId(str_imsi.c_str());    //设置设备id，"test001"是示例，实际应该以真实设备id为准


    // 启动av2hp，内部启动工作线程。
    ret =  m_p_Av2HP_run();
    if(IAV2HP_SUCCESS != ret)
    {
        SMLK_LOGD("av2hp: run failed\n");
        return M_L_RETURN_FAILURE;
    }

    //设置MCU白名单 -- 暂时处理， 应调用VEC-api接口
    std::vector<CanFilterItem> can_list;
    CanFilterItem can_item;
    can_item.delay = 2000;
    can_item.canid = M_L_MSG_ADAS_POSITION_CAN_ID | 0x80000000;
    can_list.insert(can_list.end(), can_item);

    can_item.delay = 1000;
    can_item.canid = M_L_MSG_ADAS_STUB_CAN_ID | 0x80000000;
    can_list.insert(can_list.end(), can_item);

    can_item.delay = 1000;
    can_item.canid = M_L_MSG_ADAS_SHORT_CAN_ID | 0x80000000;
    can_list.insert(can_list.end(), can_item);
    //TODO 该CANID走 power-CAN, 需要根据车型来判断power-can在哪路CAN线上
    auto ret_code = MCU::GetInstance()->SetCanFilter(CanChannelId::E_CAN_CAHNNEL_0, false, can_list);
    if (RtnCode::E_SUCCESS != ret_code)
    {
        SMLK_LOGW("MCU::GetInstance()->SetCanFilter err \n");
    }

    //启动主消息队列处理线程
    m_main_thread = std::thread(
        [this](){
            SMLK_LOGI("main work thread started");
            // while ( !(m_flags & IService::Stopping) ) {
            while (true) {
                m_main_events.get(200)();
            }

            SMLK_LOGI("main work thread greacefully exit!!!");
        }
    );


    m_timer->Start();


    return M_L_RETURN_SUCCESS;
}

void PccService::Stop()
{
    m_p_Av2HP_destory();

}

bool PccService::IsRunning() const
{
    return true;
}


void PccService::RunTestData()
{
    av2hp_gpsInfo gpsinfo = {0};
    size_t index = 0;
    while(true)
    {
        SMLK_LOGD("get gps data....\n");
        av2hp_getGpsInfo(index, &gpsinfo);
        m_p_Av2HP_setGpsInfo(&gpsinfo);


        sleep(1);
        index++;
    }
}

void PccService::Post(IN MessageHead &head, IN SMLK_UINT8 *data, IN std::size_t sz)
{
    m_main_events.emplace(
            head,
            data,
            sz,
            std::bind(&PccService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
        );
}

void PccService::Post(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_main_events.emplace(
            head,
            data,
            std::bind(&PccService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
        );
}

/******************************************************************************
* NAME: Av2hpMessageNotify
*
* DESCRIPTION: 地图数据回调消息 解析处理(仅把八字节长度消息塞入Can接口中)， 动力CAN;     TODO 根据车型判断对应CAN线
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
int PccService::Av2hpMessageNotify(const char *message, int *message_num)
{
    SMLK_LOGD("Av2hpMessageNotify.....................\n");

    if (NULL == message || NULL == message_num)
    {
        SMLK_LOGW("av2hp: Unknown message received\n");
        return -1;
    }

    CanFrame can_data;
    can_data.can_dlc = 8;

    size_t count = *message_num;
    HRBuffer buffer;
    buffer.m_length = 8;
    buffer.m_buffer = (uint8_t*)malloc(sizeof(uint8_t) * buffer.m_length);

    HRNetDataHirain* p = (HRNetDataHirain*)message;
    size_t i = 0;
    for (i = 0; i < count; i++)
    {
        memcpy(buffer.m_buffer, (p + i)->m_buffer, buffer.m_length);

		uint8_t flag = 0;
		uint8_t messageType = m_p_Av2HP_getMsgType(buffer.m_buffer, (uint8_t)buffer.m_length);

        if (buffer.m_length !=8)
        {
            SMLK_LOGW("[map_data] err data lentsh: %d \n", buffer.m_length);
            continue;
        }

        switch (messageType)
        {
        case AV2_MSG_TYPE_POSITION:         //POSITION
        {
            SMLK_LOGD("av2hp: POSITION is received\n");

            can_data.can_id  = M_L_MSG_ADAS_POSITION_CAN_ID | 0x80000000;
            memcpy(can_data.data , buffer.m_buffer, 8);

            MCU::GetInstance()->SendCanFrame(m_n_power_can_channle, &can_data, true);
            usleep(10000);
        }
            break;
        case AV2_MSG_TYPE_SEGMENT:          //SEGMENT
            printf("av2hp: SEGMENT received\n");
            break;
        case AV2_MSG_TYPE_STUB:             //STUB
        {
            SMLK_LOGD("av2hp: STUB received\n");
            can_data.can_id  = M_L_MSG_ADAS_STUB_CAN_ID | 0x80000000;
            memcpy(can_data.data , buffer.m_buffer, 8);

            MCU::GetInstance()->SendCanFrame(m_n_power_can_channle, &can_data, true);
            usleep(10000);
        }
            // memcpy(m_map_info + 8, buffer.m_buffer, 8);
            break;
        case AV2_MSG_TYPE_PROFILE_SHORT:    //SHORT
        {
            SMLK_LOGD("av2hp: SHORT received\n");
            can_data.can_id  = M_L_MSG_ADAS_SHORT_CAN_ID | 0x80000000;
            memcpy(can_data.data , buffer.m_buffer, 8);

            MCU::GetInstance()->SendCanFrame(m_n_power_can_channle, &can_data, true);
            usleep(10000);
        }
            break;
        case AV2_MSG_TYPE_PROFILE_lONG:     //LONG
            printf("av2hp: LONG received\n");
            break;
        case AV2_MSG_TYPE_META_DATA:     //META
            printf("av2hp: meta received\n");
            break;
        default:
            SMLK_LOGD("av2hp: Unknown message received\n");
            break;
        }
    }

    free(buffer.m_buffer);
    buffer.m_buffer = NULL;
    buffer.m_length = 0;

    return 0;
}


bool PccService::GetHPMsgCb(OnAv2HP_setMessageCB &ptr)
{
    if (m_p_Av2HP_setMessageCB == NULL)
    {
        return false;
    }
    ptr = m_p_Av2HP_setMessageCB;
    return true;
}

/******************************************************************************
* NAME: HandleEvent
*
* DESCRIPTION: 主消息处理函数
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void PccService::HandleEvent(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_event ) {
        case M_L_MSG_LOCATION_NET:
            OnHandleLocationInfo(head, data);
            break;
        case M_L_MSG_TSP_NET:
            OnHandleTspInfo(head, data);
            break;
        case M_L_MSG_MCU_NET:
            OnHandleMCUInfo(head, data);
            break;
        case M_L_MSG_TIMER_NET:
            OnHandleTimerInfo(head, data);
            break;

        default:
            break;
    }
}

/******************************************************************************
* NAME: OnHandleLocationInfo
*
* DESCRIPTION: 解析处理gps位置变化消息
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void PccService::OnHandleLocationInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnHandleLocationInfo ... type:%d \n",head.m_seq);
    std::lock_guard<std::mutex> lck(m_map_info_mutex);
    switch (head.m_seq)
    {
        case M_L_MSG_LOCATION_FIX_INFO_CHANGE:
            {
                SMLK_LOGI("M_L_MSG_LOCATION_FIX_INFO_CHANGE ...data len: %d, locationinfo len: %d \n", data.size(), sizeof(smartlink_sdk::LocationInfo));

                if (data.size() != sizeof(smartlink_sdk::LocationInfo))
                {
                    SMLK_LOGW("[adas_gps] err fix-gps data length..");
                    return;
                }


                smartlink_sdk::LocationInfo info;
                std::memcpy(&info, data.data(), sizeof(smartlink_sdk::LocationInfo));
                // SMLK_LOGD("get init gps:   m_valid : %d m_lon: %f, m_lat : %f, m_alt: %f, m_speed.m_value: %f, m_hdop: %f, m_pdop:%f, m_vdop:%f", info.fix_valid, info.longitude, info.latitude,info.speed, info.altitude, info.pdop, info.hdop, info.vdop);


                m_cur_gps_info.m_valid = info.fix_valid;

                m_cur_gps_info.m_pos.m_lon = info.longitude;
                m_cur_gps_info.m_pos.m_lat = info.latitude;
                m_cur_gps_info.m_pos.m_alt = info.altitude;

                m_cur_gps_info.m_speed.m_unit = av2hp_speedUnit_ms;
                m_cur_gps_info.m_speed.m_value = info.speed;

                m_cur_gps_info.m_hdop = info.hdop;
                m_cur_gps_info.m_pdop = info.pdop;
                m_cur_gps_info.m_vdop = info.vdop;

                m_cur_gps_info.m_orient = info.heading;
            }
            break;
        case M_L_MSG_LOCATION_SATELLITE_INFO_CHANGE:
            {
                SMLK_LOGD("M_L_MSG_LOCATION_SATELLITE_INFO_CHANGE ...legnth: %d \n",data.size() );

                if (data.size() != 2)
                {
                    SMLK_LOGW("[adas_gps] err satellite-info data length..");
                    return;
                }

                m_cur_gps_info.m_satInViewNum = data.at(0);
                m_cur_gps_info.m_satNum = data.at(1);
            }
            break;
        default:
            break;
    }

    //update time value
    SMLK_UINT8 timestamp[6];
    Util::get_bcd_timestamp(timestamp);
    m_cur_gps_info.m_dateTime.m_year    = timestamp[0];
    m_cur_gps_info.m_dateTime.m_month   = timestamp[1];
    m_cur_gps_info.m_dateTime.m_day     = timestamp[2];
    m_cur_gps_info.m_dateTime.m_hours   = timestamp[3];
    m_cur_gps_info.m_dateTime.m_minutes = timestamp[4];
    m_cur_gps_info.m_dateTime.m_seconds = timestamp[5];

    m_cur_gps_info.m_timestamp = Util::bcd_timestamp_2_id(timestamp);


    // m_p_Av2HP_setGpsInfo(&m_cur_gps_info);
}

/******************************************************************************
* NAME: OnHandleMCUInfo
*
* DESCRIPTION: 解析处理MCU_sdk相关
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void PccService::OnHandleMCUInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGI("[MCU_SDK] OnHandleMCUInfo");

    switch (head.m_seq)
    {
    case M_L_MSG_MCU_IGN_ON:
        {
            //todo check license...
            SMLK_LOGI("[MCU_SDK] ignoff -> ignof, now check license");

            if(( access( g_str_license_path.c_str(), F_OK ) != -1 ))
            {
                int lic_status = m_p_Av2HP_getSoftValid();

                SMLK_LOGI("[MCU_SDK] check license, return status: %d", lic_status);
                Send0F51MsgResp(lic_status);
            }
        }
        break;

    default:
        break;
    }
}


/******************************************************************************
* NAME: OnHandleTspInfo
*
* DESCRIPTION: 解析处理tsp消息
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void PccService::OnHandleTspInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    bool m_b_msg_right = true;


    if (data.size() != 515) //512 + 2 + 1
    {
        SMLK_LOGE("[tsp] err length data length");
        m_b_msg_right = false;
    }
    else
    {
        if (data[0] == M_L_MSG_TSP_8F42_0B_INFO)
        {
            //解析license
            std::string str_license;
            int n_lencese_len = (data[1] << 8) | data[2];
            if (n_lencese_len != 512)
            {
                m_b_msg_right = false;
            }
            else
            {
                str_license.insert(str_license.end(), data.begin()+3, data.end());

                std::ofstream out(g_str_license_path.c_str());
                if (out.is_open())
                {
                    out << str_license;
                    out.close();
                }
            }
        }
        else
        {
            return;
        }
    }

    //先回通用应答, 消息格式正确时， 才校验License
    SendCommonResp(head.m_extra, head.m_seq, m_b_msg_right);
    if (m_b_msg_right)
    {
        int lic_status = m_p_Av2HP_getSoftValid();

        SMLK_LOGI("[MCU_SDK] check license, return status: %d", lic_status);
        Send0F51MsgResp(lic_status);
    }
}

/******************************************************************************
* NAME: OnHandleTimerInfo
*
* DESCRIPTION:
*
* PARAMETERS:
*
* RETURN:
*
* NOTE://TODO 该CANID走 power-CAN, 需要根据车型来判断power-can在哪路CAN线上
*****************************************************************************/
void PccService::OnHandleTimerInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    std::lock_guard<std::mutex> lck(m_map_info_mutex);
    switch (head.m_seq)
    {
    case M_L_MSG_TIMER_200ms:
        {
            // SMLK_LOGD("M_L_MSG_TIMER_200ms... \n ");
            // SMLK_LOGD("");
            SMLK_LOGD("get init gps:   m_valid : %d m_lon: %f, m_lat : %f, m_alt: %f", m_cur_gps_info.m_valid, m_cur_gps_info.m_pos.m_lon, m_cur_gps_info.m_pos.m_lat, m_cur_gps_info.m_pos.m_alt);

            //每200ms喂一次GPS数据
            m_p_Av2HP_setGpsInfo(&m_cur_gps_info);
        }
        break;
    default:
        break;
    }
}

/******************************************************************************
* NAME: SendCommonResp
*
* DESCRIPTION: 发送通用应答消息
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void PccService::SendCommonResp(SMLK_UINT16 msg_id, SMLK_UINT16 seq_id, bool result)
{
    IpcTspHead ipc_head;
    ipc_head.msg_id = M_L_TSP_COMMON_RSP;
    ipc_head.seq_id = static_cast<SMLK_UINT16>(seq_id);
    ipc_head.protocol = 0x01;
    ipc_head.qos = QOS_SEND_ONE;
    ipc_head.priority = 0;

    CommResp response;
    response.seq_id = htobe16(msg_id);
    response.msg_id = htobe16(seq_id);
    response.result = (SMLK_UINT8)result;

    TspServiceApi::getInstance()->SendMsg(ipc_head, (SMLK_UINT8 *)&response, (std::size_t)sizeof(response));
}

/******************************************************************************
* NAME:
*
* DESCRIPTION:
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void PccService::Send0F51MsgResp(int n_lic_status)
{
    IpcTspHead ipc_head;
    ipc_head.msg_id = M_L_TSP_0F51_RSP;
    ipc_head.seq_id = 0xFFFF;
    ipc_head.protocol = 0x01;
    ipc_head.qos = QOS_SEND_ALWAYS;
    ipc_head.priority = 0;

    Msg0f510DHead msg;
    msg.flag = 1;
    msg.seq_id = 0xFFFF;
    msg.numbering_num = 1;
    msg.data_type =M_L_TSP_0F51_0D_KEY;
    msg.data_length = 1;
    msg.data = n_lic_status;

    TspServiceApi::getInstance()->SendMsg(ipc_head, (SMLK_UINT8 *)&msg, (std::size_t)sizeof(msg));
}

/******************************************************************************
* NAME: SuspendMainThread
*
* DESCRIPTION: 挂起主线程
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void PccService::SuspendMainThread(pthread_cond_t &cond)
{
    pthread_mutex_lock(&mtx);           //这个mutex主要是用来保证pthread_cond_wait的并发性
    pthread_cond_wait(&cond, &mtx);         // pthread_cond_wait会先解除之前的pthread_mutex_lock锁定的mtx，然后阻塞在等待对列里休眠，直到再次被唤醒（大多数情况下是等待的条件成立而被唤醒，唤醒后，该进程会先锁定先pthread_mutex_lock(&mtx);，再读取资源
    pthread_mutex_unlock(&mtx);             //临界区数据操作完毕，释放互斥锁
}

/******************************************************************************
* NAME: ResumeMainThread
*
* DESCRIPTION: 恢复主线程
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void PccService::ResumeMainThread(pthread_cond_t &cond)
{
    pthread_mutex_lock(&mtx);             //需要操作head这个临界资源，先加锁，
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mtx);           //解锁
}

/******************************************************************************
* NAME: CheckCurIsSupportPcc
*
* DESCRIPTION: 判断当前是否支持PCC功能， vcu为1时加载长春库， vcu为4时加载青岛库，其他不支持PCC
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
bool PccService::CheckCurIsSupportPcc(bool &b_is_qingdao)
{
    std::string str_vcu_type;
    std::string str_pro = SYS_PRO_NAME_VCU;
    smartlink_sdk::SysProperty::GetInstance()->GetValue(str_pro, str_vcu_type);

    if (str_vcu_type != "1"  && str_vcu_type != "4")
    {
        return false;
    }
    else if(str_vcu_type == "1")
    {
        b_is_qingdao = false;
    }
    else
    {
        b_is_qingdao = true;
    }
    return true;
}

/******************************************************************************
* NAME: LoopWaitCondition
*
* DESCRIPTION: 循环等待地图进程运行条件。
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
SMLK_UINT32 PccService::LoopWaitCondition()
{
     if(!CheckCurIsSupportPcc(m_b_vehicle_is_qingdao))
    {
        SuspendMainThread(cond_vcu);
    }

    //动态加载adas.so库
    if (m_b_vehicle_is_qingdao)
    {
        m_p_adas_lib = dlopen("/oemapp/smartlink/lib/libadasisv2hp_qingdao.so", RTLD_NOW);
    }
    else
    {
        m_p_adas_lib = dlopen("/oemapp/smartlink/lib/libadasisv2hp.so", RTLD_NOW);
    }

    if( NULL == m_p_adas_lib )
    {
        SMLK_LOGE("dlopen adas.so failure.....");
        return M_L_RETURN_FAILURE;
    }
    // 加载HP相关函数
    m_p_Av2HP_getMapVersion = (OnAv2HP_getMapVersion)dlsym(m_p_adas_lib, "Av2HP_getMapVersion");
    if( NULL == m_p_Av2HP_getMapVersion )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getMapVersion err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_getSoftVersion = (OnAv2HP_getSoftVersion)dlsym(m_p_adas_lib, "Av2HP_getSoftVersion");
    if( NULL == m_p_Av2HP_getSoftVersion )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getSoftVersion err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_init = (OnAv2HP_init)dlsym(m_p_adas_lib, "Av2HP_init");
    if( NULL == m_p_Av2HP_init )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_init err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_destory = (OnAv2HP_destory)dlsym(m_p_adas_lib, "Av2HP_destory");
    if( NULL == m_p_Av2HP_destory )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_destory err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_setDeviceId = (OnAv2HP_setDeviceId)dlsym(m_p_adas_lib, "Av2HP_setDeviceId");
    if( NULL == m_p_Av2HP_setDeviceId )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_setDeviceId err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_getMeta = (OnAv2HP_getMeta)dlsym(m_p_adas_lib, "Av2HP_getMeta");
    if( NULL == m_p_Av2HP_getMeta )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getMeta err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_run = (OnAv2HP_run)dlsym(m_p_adas_lib, "Av2HP_run");
    if( NULL == m_p_Av2HP_run )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_run err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_setGpsInfo = (OnAv2HP_setGpsInfo)dlsym(m_p_adas_lib, "Av2HP_setGpsInfo");
    if( NULL == m_p_Av2HP_setGpsInfo )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_setGpsInfo err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_getSoftValid = (OnAv2HP_getSoftValid)dlsym(m_p_adas_lib, "Av2HP_getSoftValid");
    if( NULL == m_p_Av2HP_getSoftValid )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getSoftValid err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_setMessageCB = (OnAv2HP_setMessageCB)dlsym(m_p_adas_lib, "Av2HP_setMessageCB");
    if( NULL == m_p_Av2HP_setMessageCB )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_setMessageCB err.....");
        return M_L_RETURN_FAILURE;
    }

    m_p_Av2HP_getMsgType = (OnAv2HP_getMsgType)dlsym(m_p_adas_lib, "Av2HP_getMsgType");
    if( NULL == m_p_Av2HP_getMsgType )
    {
        SMLK_LOGE("dlsym adas.so Av2HP_getMsgType err.....");
        return M_L_RETURN_FAILURE;
    }


    if(( access( g_str_map_data_path.c_str(), F_OK ) == -1 ))
    {
        SMLK_LOGE(" lose map file, path:  %s, direct exit", g_str_map_data_path.c_str());
        //map 缺失导致进程挂起时，暂时不恢复
        SuspendMainThread(cond_map);
        return M_L_RETURN_FAILURE;
    }

   return M_L_RETURN_SUCCESS;
}

/******************************************************************************
* NAME: InitVehicleType
*
* DESCRIPTION: 初始化获取车型， 以获取动力CAN通道
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void PccService::InitVehicleType()
{
    std::string str_vehicle_type;
    std::string str_pro = SYS_PRO_NAME_DID_VEHICLE_TYPE;
    if (smartlink_sdk::SysProperty::GetInstance()->GetValue(str_pro, str_vehicle_type) == RtnCode::E_SUCCESS)
    {
        int n_vehicle_type = atoi(str_vehicle_type.c_str());
        auto iter = g_map_vehicle_type.find(n_vehicle_type);
        if (iter != g_map_vehicle_type.end())
        {
            auto iter1 = g_map_vehicle_power_can_channle.find(iter->second);
            m_n_power_can_channle = iter1->second;
            SMLK_LOGI("cur power channle:  %d", m_n_power_can_channle);
        }
    }
}

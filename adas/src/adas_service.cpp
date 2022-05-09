#include "adas_service.h"

// #include <fstream>

#include <smartlink_sdk_mcu.h>
#include <smartlink_sdk_tel.h>
#include <smartlink_sdk_sys_property.h>

#include "smartlink_sdk_sys_property_def.h"
#include "tsp_service_api.h"
#include "custom_util.hpp"

using namespace smartlink_sdk;
using namespace smartlink;

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond_oem = { { {1}, {0}, {0, 0}, {0, 0}, 0, 0, {0, 0} } };
static pthread_cond_t cond_vcu = { { {2}, {0}, {0, 0}, {0, 0}, 0, 0, {0, 0} } };
static pthread_cond_t cond_map = { { {3}, {0}, {0, 0}, {0, 0}, 0, 0, {0, 0} } };


const std::string g_str_adas_ini_path = "/userdata/map/adas.ini";
const std::string g_str_conf_path = "/userdata/map/AV2HP.conf";
const std::string g_str_license_path = "/userdata/map/adas.license";
const std::string g_str_map_data_path = "/userdata/map/adasdata/ADASRoute.dat";

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

static const std::set<SMLK_UINT32> g_vec_efence_can_msg = {
    0x600, 0x601, 0x602, 0x603, 0x604, 0x605, 0x606, 0x607, 0x608, 0x609, 0x60A, 0x60B, 0x60C, 0x60D, 0x60E, 0x60F,
    0x610, 0x611, 0x612, 0x613, 0x614, 0x615, 0x616, 0x617
};

static const std::set<SMLK_UINT32> g_vec_pcc_can_msg = {
    0x18FFCC4A, 0x18FFCD4A, 0x18FFCE4A
};

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
/*************************************************/


AdasService::AdasService()
    :m_n_power_can_channle(0)
    ,m_n_gps_collection_cycle(200)
{
}

AdasService::~AdasService()
{
}

SMLK_UINT32 AdasService::Init()
{
    //初始化property-sdk
    if (!smartlink_sdk::SysProperty::GetInstance()->Init(ModuleID::E_MOUDLE_adas_service)) {
        SMLK_LOGE("Init Property failed....");
        return M_L_RETURN_FAILURE;
    }
    const std::vector<std::string> propertys = {
        std::string(SYS_PRO_NAME_VCU),
        std::string(SYS_PRO_NAME_DID_OEM_PARTNUMBER)
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
                        if(info->name == std::string(SYS_PRO_NAME_DID_OEM_PARTNUMBER))
                        {
                            //判断当前零件号是否支持地图(pcc or l2+)
                            if(CheckCurIsSupportMap(m_b_is_pcc))
                            {
                                ResumeMainThread(cond_oem);
                            }
                        }
                        else if (info->name == std::string(SYS_PRO_NAME_VCU))
                        {
                            if(m_b_is_pcc)
                            {
                                //判断当前vcu支持哪种PCC
                                if(CheckCurIsSupportPcc(m_b_is_qingdao))
                                {
                                    ResumeMainThread(cond_vcu);
                                }
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
    if (!LoopWaitCondition())
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

                        SMLK_LOGD("get init gps: time: %llu,  m_valid : %d m_lon: %f, m_lat : %f, m_alt: %f, m_speed.m_value: %f, m_hdop: %f, m_pdop:%f, m_vdop:%f", info->utc, info->fix_valid, info->longitude, info->latitude,info->speed, info->altitude, info->pdop, info->hdop, info->vdop);
                        std::lock_guard<std::mutex> lck(m_location_mutex);
                        memcpy(&m_location_info, info, len);
                    }
                    break;
                case LocationEventId::E_LOCATION_EVENT_SATELLITE_INFO_CHANGED:
                    {
                        SMLK_LOGD("[adad_gps]=============== E_LOCATION_EVENT_SATELLITE_INFO_CHANGED \n");

                        smartlink_sdk::GnssSatelliteInfo *info = reinterpret_cast<smartlink_sdk::GnssSatelliteInfo *>(data);
                        std::lock_guard<std::mutex> lck(m_location_mutex);
                        memcpy(&m_gnss_statelite_info, info, 2);
                    }
                    break;
                default:
                    break;
            }
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
                        // if (ign->on)
                        // {
                        //     if (!m_b_ign_on)
                        //     {
                        //         m_b_ign_on = true;
                        //         head.m_seq = M_L_MSG_MCU_IGN_ON;
                        //         m_b_is_post_msg = true;
                        //     }
                        // }
                        // else
                        // {
                        //     m_b_ign_on = false;
                        // }
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

                if (0 == (index % (m_n_gps_collection_cycle / 200)))
                {
                    MessageHead head(M_L_MSG_TIMER_NET, M_L_MSG_TIMER_200ms, 0);
                    Post(head, nullptr, 0);
                }
            }
    );

   return M_L_RETURN_SUCCESS;
}

SMLK_UINT32 AdasService::Start()
{
    //初始化车型，获取动力CAN-channle
    InitVehicleType();

    //重新设置GPS采集频率    //暂去除
    uint8_t frequency = 0;
    Location::GetInstance()->Setfrequency(m_n_gps_collection_cycle);
    Location::GetInstance()->Getfrequency(frequency);
    SMLK_LOGI("===========Getfrequency:: %d \n", frequency);

    {
        //初始化adas库相关
        char mapVersion[12] = { 0 };
        m_sp_adas_obj->AdasGetMapVersion(g_str_map_data_path.c_str(), mapVersion, 12);
        printf( "Map version: %s \n", mapVersion);
        std::string str_pro = SYS_PRO_NAME_DID_MAP_VERSION;
        std::string str_value = mapVersion;
        if (RtnCode::E_SUCCESS != SysProperty::GetInstance()->SetValue(str_pro, str_value))
        {
            SMLK_LOGW("[adas_init]  SysProperty::GetInstance()->SetValue err");
        }

        char softVersion[100] = { 0 };
        m_sp_adas_obj->AdasGetSoftVersion(softVersion, 100);

        printf("SOFT version: %s \n", softVersion);

        std::string str_imsi;
        if (RtnCode::E_SUCCESS != Telephony::GetInstance()->GetIMSI(str_imsi))
        {
            SMLK_LOGW("[adas_init]  GetIMSI err");
        }

        std::string ini_path;
        if (m_b_is_pcc)
        {
            if(m_b_is_qingdao)
            {
                str_imsi = "460099023776377";    //青岛 pcc测试设备ID
            }
            else
            {
                str_imsi = "460099023776882";    //一汽 pcc测试设备ID
            }
            ini_path = g_str_conf_path;
        }
        else
        {
            str_imsi = "460090044731066";   //L2+测试设备ID.
            ini_path = g_str_adas_ini_path;
        }
        bool b_ret = m_sp_adas_obj->AdasInit(ini_path.c_str(), str_imsi.c_str());

        if(!b_ret)
        {
            m_sp_adas_obj->AdasDestory();

            SMLK_LOGD("adas:init failed, the most likely reason is wrong configuration(working directory is not right or no map database)\n");
            return M_L_RETURN_FAILURE;
        }
        m_sp_adas_obj->AdasSetDeviceId(str_imsi.c_str());


        // 启动av2hp，内部启动工作线程。
        b_ret =  m_sp_adas_obj->AdasRun();

        if(!b_ret)
        {
            m_sp_adas_obj->AdasDestory();

            SMLK_LOGD("av2hp: run failed\n");
            return M_L_RETURN_FAILURE;
        }
    }


    //启动主消息队列处理线程
    m_main_thread = std::thread(
        [this](){
            SMLK_LOGI("main work thread started");
            while (true) {
                m_main_events.get(200)();
            }

            SMLK_LOGI("main work thread greacefully exit!!!");
        }
    );

    m_timer->Start();

    return M_L_RETURN_SUCCESS;
}

void AdasService::Stop()
{

}

bool AdasService::IsRunning()
{
    return true;
}

bool AdasService::IsPccOrL2Plus()
{
    return m_b_is_pcc;
}

int AdasService::Av2hpMessageNotify(const char *message, int *message_num)
{
    printf("pcc==========Av2hpMessageNotify.....................\n");

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
         uint8_t messageType = m_sp_adas_obj->AdasGetMsgType(buffer.m_buffer, buffer.m_length);

        if (buffer.m_length !=8)
        {
            SMLK_LOGW("[map_data] err data lentsh: %d \n", buffer.m_length);
            continue;
        }

        switch (messageType)
        {
        case AV2_MSG_TYPE_POSITION:         //POSITION
        {
            printf("av2hp: POSITION is received\n");

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
            printf("av2hp: STUB received\n");
            can_data.can_id  = M_L_MSG_ADAS_STUB_CAN_ID | 0x80000000;
            memcpy(can_data.data , buffer.m_buffer, 8);

            MCU::GetInstance()->SendCanFrame(m_n_power_can_channle, &can_data, true);
            usleep(10000);
        }
            break;
        case AV2_MSG_TYPE_PROFILE_SHORT:    //SHORT
        {
            printf("av2hp: SHORT received\n");
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

bool AdasService::GetHPMsgCb(OnAv2HP_setMessageCB &ptr)
{
    if(m_b_is_pcc)
    {
        return m_sp_adas_obj->AdasGetHPMsgCb(ptr);
    }
    return false;
}

void AdasService::AdasMessageNotify(const char* msg_data, const unsigned int* msg_id, int msg_num)
{
    static CanFrame can_data;
    can_data.can_dlc = 8;
    unsigned long long* data = (unsigned long long*)msg_data;
    for (int i = 0; i < msg_num; ++i)
    {
        printf("================msg_id:%X, msg_data:%016llX\n", msg_id[i], data[i]);

        // g_vec_pcc_can_msg.
        unsigned int n_msg_id = msg_id[i];
        unsigned long long n_msg_data = data[i];
        if(g_vec_pcc_can_msg.find(n_msg_id) != g_vec_pcc_can_msg.end())
        {
            printf("[l2+] this is pcc msg, msg_id:%X \n", msg_id[i]);
            can_data.can_id  = n_msg_id | 0x80000000;
            memcpy(can_data.data , &n_msg_data, 8);

            //power-channle
            MCU::GetInstance()->SendCanFrame(m_n_power_can_channle, &can_data, true);
            usleep(100000);
        }
        else if(g_vec_efence_can_msg.find(n_msg_id) != g_vec_efence_can_msg.end())
        {
            printf("[l2+] this is efence msg, msg_id:%x , msg_data:%016llx\n", n_msg_id);
            can_data.can_id  = n_msg_id;
            memcpy(can_data.data , &n_msg_data, 8);

            if (can_data.can_id > 0x80000000)
            {
                SMLK_LOGE("========================err canid: %x", can_data.can_id );
            }

            //channle-6
            MCU::GetInstance()->SendCanFrame(5, &can_data, true);
            usleep(100000);
        }
    }
}

void AdasService::Post(IN MessageHead &head, IN SMLK_UINT8 *data, IN std::size_t sz)
{
    m_main_events.emplace(
        head,
        data,
        sz,
        std::bind(&AdasService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void AdasService::Post(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_main_events.emplace(
        head,
        data,
        std::bind(&AdasService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void AdasService::HandleEvent(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_event ) {
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

void AdasService::OnHandleMCUInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{

}

void AdasService::OnHandleTspInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{

}

void AdasService::OnHandleTimerInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch (head.m_seq)
    {
    case M_L_MSG_TIMER_200ms:
        {
            SMLK_LOGD("M_L_MSG_TIMER_200ms... \n");
            //每200ms喂一次GPS数据
            std::lock_guard<std::mutex> lck(m_location_mutex);
            m_sp_adas_obj->AdasSetGpsInfo(m_location_info, m_gnss_statelite_info);
        }
        break;
    default:
        break;
    }
}

void AdasService::SuspendMainThread(pthread_cond_t &cond)
{
    pthread_mutex_lock(&mtx);           //这个mutex主要是用来保证pthread_cond_wait的并发性
    pthread_cond_wait(&cond, &mtx);         // pthread_cond_wait会先解除之前的pthread_mutex_lock锁定的mtx，然后阻塞在等待对列里休眠，直到再次被唤醒（大多数情况下是等待的条件成立而被唤醒，唤醒后，该进程会先锁定先pthread_mutex_lock(&mtx);，再读取资源
    pthread_mutex_unlock(&mtx);             //临界区数据操作完毕，释放互斥锁
}

void AdasService::ResumeMainThread(pthread_cond_t &cond)
{
    pthread_mutex_lock(&mtx);             //需要操作head这个临界资源，先加锁，
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mtx);           //解锁
}

bool AdasService::CheckCurIsSupportMap(bool &b_is_pcc)
{
    std::string str_hw_num;
    std::string str_pro = SYS_PRO_NAME_DID_OEM_PARTNUMBER;
    smartlink_sdk::SysProperty::GetInstance()->GetValue(str_pro, str_hw_num);

    if(str_hw_num == "7925100C88P-C00")
    {
        b_is_pcc = true;
        return true;
    }
    else if (str_hw_num == "7925100D88P-C00")
    {
        b_is_pcc = false;
        return true;
    }
    return false;
}

bool AdasService::CheckCurIsSupportPcc(bool &b_is_qingdao)
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
        m_n_gps_collection_cycle = 1000;    //青岛pcc gps采集周期为1000ms
    }
    return true;
}

bool AdasService::LoopWaitCondition()
{
    //判断当前零件号是否支持地图(pcc or l2+)
    if(!CheckCurIsSupportMap(m_b_is_pcc))
    {
        SuspendMainThread(cond_oem);
    }

    //当前支持PCC地图后， 判断地图为青岛地图还是非青岛地图
    if (m_b_is_pcc)
    {
        if(!CheckCurIsSupportPcc(m_b_is_qingdao))
        {
            SuspendMainThread(cond_vcu);
        }
    }

    //判断当前是否有地图文件
    if(( access( g_str_map_data_path.c_str(), F_OK ) == -1 ))
    {
        SMLK_LOGE(" lose map file, path:  %s, direct exit", g_str_map_data_path.c_str());
        //map 缺失导致进程挂起时，暂时不恢复
        SuspendMainThread(cond_map);
        return false;
    }

    if (m_b_is_pcc)
    {
        m_sp_adas_obj.reset(new AdasSoPcc);
    }
    else
    {
        m_sp_adas_obj.reset(new AdasSoL2);
    }

    if (!m_sp_adas_obj->Init())
    {
        return false;
    }

    return true;
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
void AdasService::InitVehicleType()
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

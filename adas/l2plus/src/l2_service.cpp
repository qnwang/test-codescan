#include <smartlink_sdk_location.h>
#include <smartlink_sdk_tel.h>
#include <smartlink_sdk_mcu.h>
#include <smartlink_sdk_sys_property.h>
#include <smartlink_sdk_sys_property_def.h>

#include "l2_service.h"
#include "adas_define_common.h"
#include "custom_util.hpp"

using namespace smartlink;
using namespace smartlink_sdk;

const std::string g_str_adas_ini_path = "/userdata/map/adas.ini";
const std::string g_str_map_data_path = "/userdata/map/adasdata/ADASRoute.dat";

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
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

static const std::set<SMLK_UINT32> g_vec_efence_can_msg = {
    0x600, 0x601, 0x602, 0x603, 0x604, 0x605, 0x606, 0x607, 0x608, 0x609, 0x60A, 0x60B, 0x60C, 0x60D, 0x60E, 0x60F,
    0x610, 0x611, 0x612, 0x613, 0x614, 0x615, 0x616, 0x617
};

static const std::set<SMLK_UINT32> g_vec_pcc_can_msg = {
    0x18FFCC4A, 0x18FFCD4A, 0x18FFCE4A
};

L2Service::L2Service()
    :m_n_power_can_channle(0)
{
}

L2Service::~L2Service()
{
}

/******************************************************************************
* NAME: Init
*
* DESCRIPTION: 服务初始化， 初始化GPS、 Tel、 timer
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
SMLK_UINT32 L2Service::Init()
{
    //阻塞等待地图文件， 缺失挂起主进程
    if(( access( g_str_map_data_path.c_str(), F_OK ) == -1 ))
    {
        SMLK_LOGE(" lose map file, path:  %s, direct exit", g_str_map_data_path.c_str());
        //map 缺失导致进程挂起时，暂时不恢复
        SuspendMainThread(cond_map);
        return M_L_RETURN_FAILURE;
    }

    //初始化GPS相关
    if ( !Location::GetInstance()->Init(ModuleID::E_MOUDLE_adas_service) ) {
        SMLK_LOGE("fail to init location service API interface!!!");
        return M_L_RETURN_FAILURE;
    }

    std::vector<LocationEventId> loc_events = {
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
                        SMLK_LOGD("[adad_gps] E_LOCATION_EVENT_FIX_INFO_CHANGED \n");
                        if ( sizeof(smartlink_sdk::LocationInfo) != (std::size_t)len ) {
                                SMLK_LOGE("[adad_gps] unexpected body length for FIX INFO CHNAGED notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::LocationInfo));
                                return;
                        }
                        smartlink_sdk::LocationInfo *info = (smartlink_sdk::LocationInfo *)data;
                        bytes.insert(bytes.end(), (SMLK_UINT8 *)info, ((SMLK_UINT8 *)info + len));
                        head.m_seq = M_L_MSG_LOCATION_FIX_INFO_CHANGE;
                        head.m_extra   = sizeof(smartlink_sdk::LocationInfo);
                        SMLK_LOGD("get init gps:   m_valid : %d m_lon: %f, m_lat : %f, m_alt: %f, m_speed.m_value: %f, m_hdop: %f, m_pdop:%f, m_vdop:%f", info->fix_valid, info->longitude, info->latitude,info->speed, info->altitude, info->pdop, info->hdop, info->vdop);
                    }
                    break;
                case LocationEventId::E_LOCATION_EVENT_SATELLITE_INFO_CHANGED:
                    {
                        SMLK_LOGD("[adad_gps] E_LOCATION_EVENT_SATELLITE_INFO_CHANGED \n");

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

    //初始化Tel-sdk
    if (Telephony::GetInstance()->Init(ModuleID::E_MOUDLE_adas_service) == false) {
        SMLK_LOGE("fail to init Telephony service API interface!!!");
        return M_L_RETURN_FAILURE;
    }

    //初始化MCU-SDK
    if ( RtnCode::E_SUCCESS != MCU::GetInstance()->Init(ModuleID::E_MOUDLE_adas_service) ) {
        SMLK_LOGE("fail to init location service API interface!!!");
        return M_L_RETURN_FAILURE;
    }

    //初始化定时器
    m_timer = std::make_shared<LoopTimer<std::chrono::milliseconds, 1000>>(
            [this](SMLK_UINT32 index){
                {
                    MessageHead head(M_L_MSG_TIMER_NET, M_L_MSG_TIMER_100ms, 0);
                    Post(head, nullptr, 0);
                }

                if( 0 == index % 2)
                {
                    MessageHead head(M_L_MSG_TIMER_NET, M_L_MSG_TIMER_200ms, 0);
                    Post(head, nullptr, 0);
                }
            }
    );

    return M_L_RETURN_SUCCESS;
}

SMLK_UINT32 L2Service:: Start()
{
    //获取动力CAN通道
    InitVehicleType();

    //重新设置GPS采集频率    //暂去除
    // uint8_t frequency = 0;
    // Location::GetInstance()->Setfrequency(200);
    // Location::GetInstance()->Getfrequency(frequency);
    // SMLK_LOGI("===========Getfrequency:: %d \n", frequency);

    // 查询软件版本号
    char soft_version[100] = { 0 };
    adas_get_soft_version(soft_version, 100);
    printf("soft version: %s", soft_version);

    // 初始化
    std::string str_imsi;
    if (RtnCode::E_SUCCESS != Telephony::GetInstance()->GetIMSI(str_imsi))
    {
        SMLK_LOGW("[adas_init]  GetIMSI err");
    }
    str_imsi = "460090044731066";   //L2+测试设备ID.
    if (adas_init(g_str_adas_ini_path.c_str(), str_imsi.c_str()) != adas_status_success)
    {
        printf("init failed\n");
        adas_destory();
        return M_L_RETURN_FAILURE;
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

void L2Service::Stop()
{
    adas_destory();
}

bool L2Service::IsRunning()
{
    return true;
}

void L2Service::Post(IN MessageHead &head, IN SMLK_UINT8 *data, IN std::size_t sz)
{
    m_main_events.emplace(
            head,
            data,
            sz,
            std::bind(&L2Service::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
        );
}

void L2Service::Post(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_main_events.emplace(
            head,
            data,
            std::bind(&L2Service::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
        );
}

/******************************************************************************
* NAME: AdasMessageNotify
*
* DESCRIPTION: adas地图回调函数
*
* PARAMETERS:
*
* RETURN:
*
* NOTE: TODO 数据处理？？？？
*****************************************************************************/
void L2Service::AdasMessageNotify(const char* msg_data, const unsigned int* msg_id, int msg_num)
{

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
            std::lock_guard<std::mutex> lck(m_map_pcc_info_mutex);
            switch (msg_id[i])
            {
                case 0x18FFCC4A:
                    memcpy(m_map_pcc_info, &n_msg_data, 8);
                    break;
                case 0x18FFCD4A:
                    memcpy(m_map_pcc_info + 8, &n_msg_data, 8);
                    break;
                case 0x18FFCE4A:
                    memcpy(m_map_pcc_info + 16, &n_msg_data, 8);
                    break;
                default:
                    break;
            }
        }
        else if(g_vec_efence_can_msg.find(n_msg_id) != g_vec_efence_can_msg.end())
        {
            printf("[l2+] this is efence msg, msg_id:%x , msg_data:%016llx\n", n_msg_id);

            //TODO send efence can msg to channle-6
            static CanFrame can_data;
            can_data.can_dlc = 8;
            can_data.can_id  = n_msg_id;
            memcpy(can_data.data , &n_msg_data, 8);

            //channle-6
            MCU::GetInstance()->SendCanFrame(5, &can_data, true);
            usleep(10000);
        }
    }
}

/******************************************************************************
* NAME: HandleEvent
*
* DESCRIPTION: 消息队列总处理函数
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void L2Service::HandleEvent(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    switch ( head.m_event ) {
        case M_L_MSG_LOCATION_NET:
            OnHandleLocationInfo(head, data);
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
* DESCRIPTION: gps事件处理函数， 获取GPS上报信息， 缓存gps位置信息
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void L2Service::OnHandleLocationInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("OnHandleLocationInfo ... type:%d \n",head.m_seq);
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
                SMLK_LOGD("get init gps:   m_valid : %d m_lon: %f, m_lat : %f, m_alt: %f, m_speed.m_value: %f, m_hdop: %f, m_pdop:%f, m_vdop:%f", info.fix_valid, info.longitude, info.latitude,info.speed, info.altitude, info.pdop, info.hdop, info.vdop);


                if (info.fix_valid)
                {
                    m_struct_gps_info.gprmc_status = 'A';
                    m_struct_gps_info.gprmc_mode = 'A';
                    m_struct_gps_info.mode = 1;
                }
                else
                {
                    m_struct_gps_info.gprmc_status = 'V';
                    m_struct_gps_info.gprmc_mode = 'N';
                    m_struct_gps_info.mode = 0;
                }

                m_struct_gps_info.pos.kind = adas_coordinate_wgs84;
                m_struct_gps_info.pos.lon = info.longitude;
                m_struct_gps_info.pos.lat = info.latitude;
                m_struct_gps_info.pos.alt = info.altitude;

                m_struct_gps_info.orient = info.heading;
                m_struct_gps_info.speed  = info.speed;

                m_struct_gps_info.pdop  = info.pdop;
                m_struct_gps_info.hdop  = info.hdop;
                m_struct_gps_info.vdop  = info.vdop;
                m_struct_gps_info.speed_up = 0;
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

                m_struct_gps_info.sat_num = data.at(1);
            }
            break;
        default:
            break;
    }

    //update time value
    SMLK_UINT8 timestamp[6];
    Util::get_bcd_timestamp(timestamp);
    m_struct_gps_info.date_time.year   = timestamp[0];
    m_struct_gps_info.date_time.month  = timestamp[1];
    m_struct_gps_info.date_time.day    = timestamp[2];
    m_struct_gps_info.date_time.hour   = timestamp[3];
    m_struct_gps_info.date_time.minute = timestamp[4];
    m_struct_gps_info.date_time.second = timestamp[5];
    m_struct_gps_info.date_time.millisecond = 0;

    printf("---adas_set_gps_info--------\n");
    // adas_set_gps_info(&m_struct_gps_info);
}

/******************************************************************************
* NAME: OnHandleTimerInfo
*
* DESCRIPTION: 定时器事件处理函数
*
* PARAMETERS:
*
* RETURN:
*
* NOTE: 处理两类需求。 需求一: 每200ms喂一次GPS数据给adas   需求二: 发送pcc缓存数据至Power-can通道
*****************************************************************************/
void L2Service::OnHandleTimerInfo(IN MessageHead &head, IN std::vector<SMLK_UINT8> &data)
{
    printf("----------ontimer---------\n");
    CanFrame can_data_info;
    can_data_info.can_dlc = 8;

    std::lock_guard<std::mutex> lck(m_map_pcc_info_mutex);
    switch (head.m_seq)
    {
        case M_L_MSG_TIMER_100ms:
        {
            printf("-----------send 100ms pcc msg-----\n");

            can_data_info.can_id  = M_L_MSG_ADAS_STUB_CAN_ID | 0x80000000;
            memcpy(can_data_info.data , m_map_pcc_info + 8, 8);
            MCU::GetInstance()->SendCanFrame(m_n_power_can_channle, &can_data_info, true);
            usleep(10000);

            can_data_info.can_id  = M_L_MSG_ADAS_SHORT_CAN_ID | 0x80000000;
            memcpy(can_data_info.data , m_map_pcc_info + 16, 8);
            MCU::GetInstance()->SendCanFrame(m_n_power_can_channle, &can_data_info, true);
            usleep(10000);
        }
            break;
        case M_L_MSG_TIMER_200ms:
        {
            printf("-----------send 200ms pcc msg-----\n");
            can_data_info.can_id  = M_L_MSG_ADAS_POSITION_CAN_ID | 0x80000000;
            memcpy(can_data_info.data , m_map_pcc_info, 8);
            MCU::GetInstance()->SendCanFrame(m_n_power_can_channle, &can_data_info, true);
            usleep(10000);


            printf("----------adas_set_gps_info enter---------\n");
            adas_set_gps_info(&m_struct_gps_info);
        }
            break;
        default:
            break;
    }
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
void L2Service::SuspendMainThread(pthread_cond_t &cond)
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
void L2Service::ResumeMainThread(pthread_cond_t &cond)
{
    pthread_mutex_lock(&mtx);             //需要操作head这个临界资源，先加锁，
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mtx);           //解锁
}

void L2Service::InitVehicleType()
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

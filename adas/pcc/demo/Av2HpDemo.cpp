#include <memory.h>
#include <stdio.h>

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
// #include "adasisv2hp.h"
#include "adas/adasisv2hp.h"


// message type
// 0 System Specific
// 1 POSITION
// 2 SEGMENT
// 3 STUB
// 4 PROFILE SHORT
// 5 PROFILE LONG
// 6 META-DATA
// 7 Reserved
#define AV2_MSG_TYPE_SYSTEM_SPECIFIC	( 0 )
#define AV2_MSG_TYPE_POSITION			( 1 )
#define AV2_MSG_TYPE_SEGMENT			( 2 )
#define AV2_MSG_TYPE_STUB				( 3 )
#define AV2_MSG_TYPE_PROFILE_SHORT		( 4 )
#define AV2_MSG_TYPE_PROFILE_lONG		( 5 )
#define AV2_MSG_TYPE_META_DATA			( 6 )
#define AV2_MSG_TYPE_RESERVED			( 7 )

typedef unsigned char uint8_t;
typedef int           int32_t;
typedef unsigned int  uint32_t;

typedef struct HRNetDataHirain
{
    uint8_t     m_buffer[8];
}HRNetDataHirain;

typedef struct HRBuffer
{
    uint8_t     *m_buffer;
    int32_t      m_length;
}HRBuffer;

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

void av2hp_sleep(uint32_t milliseconds)
{
#ifdef WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}

static av2hp_e av2hp_messageNotify(const char *message, int *message_num)
{
    if (NULL == message || NULL == message_num)
    {
        printf("av2hp: Unknown message received\n");
        return 0;
    }

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
		uint8_t messageType = Av2HP_getMsgType(buffer.m_buffer, (uint8_t)buffer.m_length);
        switch (messageType)
        {
        case AV2_MSG_TYPE_POSITION:         //POSITION
            printf("av2hp: POSITION is received\n");
            break;
        case AV2_MSG_TYPE_SEGMENT:          //SEGMENT
            printf("av2hp: SEGMENT received\n");
            break;
        case AV2_MSG_TYPE_STUB:             //STUB
            printf("av2hp: STUB received, [%d]\n", flag);
            break;
        case AV2_MSG_TYPE_PROFILE_SHORT:    //SHORT
            printf("av2hp: SHORT received\n");
            break;
        case AV2_MSG_TYPE_PROFILE_lONG:     //LONG
            printf("av2hp: LONG received\n");
            break;
        case AV2_MSG_TYPE_META_DATA:     //META
            printf("av2hp: meta received\n");
            break;
        default:
            printf("av2hp: Unknown message received\n");
            break;
        }
    }

    free(buffer.m_buffer);
    buffer.m_buffer = NULL;
    buffer.m_length = 0;

    return 0;
}

// demo main function, call it in your project
int av2hp_demo(short isWithSimulator)
{
    // 地图数据需要放到和AV2HP.conf文件同级的adasdata目录下
    // 指定路径为""，默认在执行程序同级目录下寻找

#ifdef WIN32
    const char* configFilePath = "./AV2HP.conf";
#else
    // const char* configFilePath = "AV2HP.conf";
    const char* configFilePath = "/userdata/map/AV2HP.conf";

#endif

    char mapVersion[10] = { 0 };
    Av2HP_getMapVersion("/userdata/map/adasdata/ADASRoute.dat", mapVersion, 10);
    printf( "Map version: %s \n", mapVersion);

    char softVersion[100] = { 0 };
    Av2HP_getSoftVersion(softVersion, 100);
    printf("SOFT version: %s \n", softVersion);

    av2hp_e ret = Av2HP_init(configFilePath);

    if(IAV2HP_SUCCESS != ret)
    {
        Av2HP_destory();
        printf("av2hp:init failed, the most likely reason is wrong configuration(working directory is not right or no map database)\n");
        return (-1);
    }

    Av2HP_setDeviceId("test001");    //设置设备id，"test001"是示例，实际应该以真实设备id为准
    int lic_status = Av2HP_getSoftValid();

    // get meta
    av2hp_meta meta_data;
    Av2HP_getMeta(&meta_data);

    // set callback
    Av2HP_setMessageCB(av2hp_messageNotify);

    // 启动av2hp，内部启动工作线程。
    ret =  Av2HP_run();
    if(IAV2HP_SUCCESS != ret)
    {
        printf("av2hp: run failed\n");
        return (-1);
    }

    // keep av2hp runnnig:
    // Av2HP_run为非同期处理，增加等待处理， 等待时间（demo时间）可以任意设定，不设定等待，进程将直接退出。
    // 如果再平台中跑demo，需要删除这个等待（isStopped = 1），否则av2hp_demo会把平台exe lock住。
    short isStopped = 0;
	av2hp_sleep(1000);
    av2hp_gpsInfo gpsinfo = {0};
    size_t index = 0;
    while(!isStopped)
    {
        av2hp_getGpsInfo(index, &gpsinfo);

        Av2HP_setGpsInfo(&gpsinfo);
        av2hp_sleep(1000);
        index++;
    }

    Av2HP_destory();

    return 0;
}

int main()
{
    av2hp_demo(0);
    return 0;
}
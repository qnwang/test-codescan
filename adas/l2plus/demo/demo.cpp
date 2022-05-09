#include "adas.h"
#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <thread>

void adas_sleep(int millisecond)
{
#ifdef WIN32
    Sleep(millisecond);
#else
    usleep(millisecond * 1000);
#endif
}

/**
* @brief 回调函数
*/
void callback(const char* msg_data, const unsigned int* msg_id, int msg_num)
{
    unsigned long long* data = (unsigned long long*)msg_data;
    for (int i = 0; i < msg_num; ++i)
    {
        printf("msg_id:%X, msg_data:%016llX\n", msg_id[i], data[i]);
    }
}

int main(int argc, char **argv)
{
    const char* config_path = "./adas.ini";
    // 查询软件版本号
    char soft_version[100] = { 0 };
    adas_get_soft_version(soft_version, 100);
    printf("soft version: %s", soft_version);

    // 初始化
    if (adas_init(config_path, "test_device_id") != adas_status_success)
    {
        printf("init failed\n");
        adas_destory();
        return 0;
    }

    // 设置回调函数
    adas_set_message_callback(callback);

    // 获取license状态
    if (adas_get_soft_valid() != adas_status_success)
    {
        printf("license check faild\n");
    }

    // 设置GPS位置
    const adas_gps_info gps_info = { 0 };
    const adas_six_axis_imu_info imu_info = { 0 };
    const adas_odometry_info odometry_info = { 0 };

    bool stop = false;
    while (!stop)
    {
        adas_set_gps_info(&gps_info);
        adas_sleep(200);
    }

    // 清理
    adas_destory();
    return 0;
}
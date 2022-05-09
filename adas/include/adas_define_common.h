#include "smlk_types.h"

/*函数运行结果宏定义-------------------------------------------------*/
#define M_L_RETURN_SUCCESS     0    //成功
#define M_L_RETURN_FAILURE     1    //失败

/*TSP消息类型宏定义-------------------------------------------------*/
#define  M_L_TSP_8F42_REQ       0x8F42
#define  M_L_TSP_COMMON_RSP     0x0001
#define  M_L_TSP_0F51_RSP       0x0F51

#define  M_L_TSP_0F51_0D_KEY    0x0D


/*消息队列：消息类型宏定义-------------------------------------------------*/

/*主消息类型宏定义-------------------------------------------------*/
#define M_L_MSG_LOCATION_NET                    0x01        //GPS_SDK消息
#define M_L_MSG_TSP_NET                         0x02        //TSP消息
#define M_L_MSG_MCU_NET                         0x03        //MCU_SDK消息
#define M_L_MSG_TIMER_NET                       0x04        //timer触发消息


/*次消息类型宏定义-------------------------------------------------*/
#define M_L_MSG_LOCATION_FIX_INFO_CHANGE          0x01      //GPS位置改变
#define M_L_MSG_LOCATION_SATELLITE_INFO_CHANGE    0x02      //GPS卫星信息改变

#define M_L_MSG_MCU_IGN_ON                        0x11     //Ignoff -> ignon

#define M_L_MSG_TSP_8F42_0B_INFO                  0x0B      //tsp-8f42-0b子消息

#define M_L_MSG_TIMER_100ms                       0x31      //100ms定时器触发子消息
#define M_L_MSG_TIMER_200ms                       0x32      //200ms定时器触发子消息


/*预见性巡航CAN报文-------------------------------------------------*/
#define M_L_MSG_ADAS_POSITION_CAN_ID               0x18FFCC4A
#define M_L_MSG_ADAS_STUB_CAN_ID                   0x18FFCD4A
#define M_L_MSG_ADAS_SHORT_CAN_ID                  0x18FFCE4A




typedef struct
{
    SMLK_UINT16 seq_id;/*对应平台消息的流水号*/
    SMLK_UINT16  msg_id;/*对应平台消息de ID*/
    SMLK_UINT8 result;
}__attribute__((__packed__))CommResp;

typedef struct
{
    SMLK_UINT8 version;/*主动上报时填0*/
    SMLK_UINT8 time[6];
    SMLK_UINT8 flag;
    SMLK_UINT16 seq_id;
    SMLK_UINT8 numbering_num;
    SMLK_UINT8 data_type;
    SMLK_UINT8 data_length;
    SMLK_UINT8 data;
}__attribute__((__packed__))Msg0f510DHead;

enum class VehicleType : SMLK_UINT8
{
    EM_J6_LOW = 0,   //J6 中低配
    EM_J6_HIGH,      //J6 高配 ， J7中低配。 统称J7一代
    EM_J7_HIGH,      //J7高配。 统称J7-1.5代
    EM_EV,           //新能源
    EM_QINGQI,       //青汽
};

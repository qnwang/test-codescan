#ifndef _SMLK_TSP_JTT808_H_
#define _SMLK_TSP_JTT808_H_

#include <stdint.h>

/*808传递消息的通用消息头*/
#define JT_T808_CELL_NUM_LEN                    6

#define JT808_MSG_ID_TP_RSP_GENERIC             (0x0001)
#define JT808_MSG_ID_PT_RSP_GENERIC             (0x8001)

#define JT_T808_MSG_ID_TP_REQ_DIAG              (0xE501)
#define JT_T808_MSG_ID_TP_RSP_DIAG              (0xF501)

typedef struct 
{
  uint16_t msg_id;/*消息ID*/
  uint16_t attribute;/*属性*/
  uint8_t  cell_num[JT_T808_CELL_NUM_LEN];/*设备号*/
  uint16_t seq_id;/*流水号*/
  uint8_t  data[0];
}__attribute__((__packed__))JTT808Msg;
#define JTT808_MSGHEAD_LEN  sizeof(JTT808Msg)


enum class GenericRspResult:uint8_t
{
    E_SUCCESS = 0,/*success*/
    E_FAILED, /*failed*/
    E_INVALID_MSG,/* invalid message*/
    E_NOT_SURRPORT,/*not supported*/
    E_ALARM_CONFIRM,/*alarm confirm*/
};

/*通用应答*/
typedef struct {
    uint16_t seq_id;
    uint16_t msg_id;
    GenericRspResult result;     
}__attribute__((__packed__))GenericRsp;

#endif
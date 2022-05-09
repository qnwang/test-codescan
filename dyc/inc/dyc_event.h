#pragma once

#define     M_SL_MSGID_PT_LOCATION_REQ       0x8201
#define     M_SL_MSGID_TP_LOCATION_RSP       0x0201

#define     M_SL_MSGID_TSP_DYC_REQ           0xF220   // 下行
#define     M_SL_MSGID_TSP_DYC_REP           0xE220   // TBOX上行

#define     M_SL_MSGID_TBOX_COMMOM_REP       0x0001

//内部事件通知ID
#define     M_SL_MSGID_CAN_EVENT                         0xF1    //  mcu can数据更新
#define     M_SL_MSGID_TIMER_NOTIFY                      0xF2    // timer触发
#define     M_SL_MSGID_SIGNAL_DATA_CHANGE                0xF3    // can signal数据变化
#define     M_SL_MSGID_DATA_CHANGED                      0xF4    // vehicle 统计数据变化
#define     M_SL_MSGID_LOCATOIN_NTF                      0xF5    // gps信号
#define     M_SL_MSGID_DBC_FILE_UPDATE_NOTIFY            0xF6    // 下载新的DBC
#define     M_SL_MSGID_MCU_STATE_NOTIFY                  0xF7    // mcu提供系统状态


#define     M_SL_MSGID_COMPRESS_REQ           0xF527  // 压缩请求
#define     M_SL_MSGID_COMPRESS_RSP           0x7527  // 压缩回复

// TSP
#define     M_SL_MSGID_TP_DYC                 0xE260  // 终端上报id

#define     TSP_SET_DYC_INTERVAL              0x01    // 设置动态采集和上报间隔
#define     TSP_READ_DYC_INTERVAL             0x02    // 查询动态采集和上报间隔
#define     TSP_SEND_DYC_DBC                  0x03    // DBC文件配置下发
#define     TSP_READ_DYC_CONFIG               0x04    // 读取终端DBC配置
#define     TSP_CLEAR_DYC_CONFIG              0x05    // 清除终端DBC配置
#define     TBOX_REQ_DYC_CONFIG               0x06    // 终端请求DBC配置

#define     TSP_CLEAR_ALL_CONFIG              250




/*****************************************************************************/
/**
 * \file       vehctrl_queue.h
 * \author     huangxin
 * \date       2020/11/18
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_CTRL_QUEUE_H_
#define _VEHICLE_CTRL_QUEUE_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <vector>
#include "smlk_error.h"
#include "smlk_types.h"
#include "vehctrl_general_definition.h"

namespace smartlink
{
#define SL_QUEUE_GET_TIMEOUT_MS 200 // timeout millisecond to get item from queue
    /*来自tsp or 短信 or 主动上报*/
    enum class VehCtrlCmdFrom : SMLK_UINT8
    {
        FROM_TSP_MODULE = 1,
        FROM_TEXT_MESSAGE = 2,
        FROM_AUTO_REPORTED = 3
    };
    /*m_queue_type的值： 设置/获取*/
    enum class RctrlCmdType : SMLK_UINT8
    {
        REMCTRL_CMD_SET = 1,
        REMCTRL_CMD_GET = 2
    };

    /*处理来自tsp的消息和状态变化主动上报的消息*/
    class RctrlMsgQueue
    {
    public:
        RctrlMsgQueue();
        virtual ~RctrlMsgQueue();

    public:
        RctrlMsgType m_msg_type;           /*tsp or auto-report or text message*/
        RctrlHead m_head;                  /*将tsp发过来的icp头中的信息取出来*/
        std::vector<SMLK_UINT8> m_message; /*tsp发过来的消息体*/
    };

    class RctrlCmdQueue
    {
    public:
        RctrlCmdQueue(){};
        virtual ~RctrlCmdQueue(){};

    public:
        RctrlHead m_head;          /*Tsp消息头*/
        VehCtrlCmdFrom m_cmd_from; /*消息来源: Tsp or Text*/
        RctrlCmdType m_cmd_type;   /*消息类型: 控车 or 查询*/
        SMLK_UINT8 m_version;      /*0x8F41 专用*/
        class ConfigQueue
        {
        public:
            std::vector<Msg8F41_Body> m_tsp_cmd_vec;
            std::vector<SmlkMcuCmd> m_cmd_vec;
        } config_queue;
        class QueryQueue
        {
        public:
            std::vector<SMLK_UINT16> m_query_vec;
        } query_queue;
    };

    /*远控结果返回队列*/
    class RctrlResultQueue
    {
    public:
        RctrlResultQueue(){};
        virtual ~RctrlResultQueue(){};

    public:
        RctrlHead m_head;          /*Tsp消息头*/
        VehCtrlCmdFrom m_cmd_from; /*消息来源: Tsp or Text*/
        RctrlCmdType m_result_type;
        SMLK_UINT8 m_version;
        class ConfigReuslt
        {
        public:
            std::vector<Msg8F41_Body> m_tsp_cmd_vec;
            std::vector<SmlkMcuCmdResp> m_mcu_res_vec;
            std::vector<SmlkRctrl_8F41_Result> m_tsp_result;
        } config_result;
        class QueryReuslt
        {
        public:
            std::vector<SMLK_UINT16> m_query_id;    /*记录8F51查询的内容*/
            std::vector<VehctrlGetCmd> m_query_vec; /*记录从VD获取的数据,根据8F51查询的内容判定,有的查询内容不需要从VD获取数据*/
        } query_result;
    };
};
#endif

/*****************************************************************************/
/**
 * \file       tsp_client.cpp
 * \author     wukai
 * \date       2021/07/20
 * \version    Tbox2.0 V2
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/
#include "avm_manager.h"
#include "avm_msg_builder.h"
#include "avm_service.h"
#include "avm_sync_service.h"
#include "smartlink_sdk_tel.h"
#include "smartlink_sdk_sys_time.h"
#include "smartlink_sdk_sys_property.h"
#include "smartlink_sdk_sys_property_def.h"

namespace smartlink
{
    namespace AVM
    {
        void VedioProcessorManager::TspMsgProcess()
        {
            SMLK_LOGD("*********TspMsgProcess Thread Start***********");
            while (!CheckStatus(BaseModule::Stopping))
            {
                bool timeout_flag = false;
                TspQueueMsg queue_msg = m_tsp_queue.get(200, &timeout_flag);
                if (timeout_flag)
                {
                    usleep(200000); // 200ms
                    continue;
                }
                /*****************************************************************************
                 *                      Tsp消息解析,AVM&DMS组包 开始                           *
                 *****************************************************************************/
                AvmMessage avm_msg_;
                struct
                {
                    SMLK_UINT16 child_cmd_id;
                    SMLK_UINT16 event_len;
                } __attribute__((__packed__)) cmd_data;
                cmd_data.event_len = (SMLK_UINT16)queue_msg.data.size();

                // AVM消息由:帧头(1byte)+发送设备编号(1byte)+指令报文长度(2byte)+指令表报文内容[指令组id(1byte)+指令id(1byte)+指令长度(2byte)+指令内容(nbyte)]+帧尾(1byte)组成
                SMLK_UINT16 total_len_ /*指令报文长度*/ = (SMLK_UINT16)(2 /*指令组id+指令id*/ + 2 /*指令长度*/ + cmd_data.event_len /*指令内容长度*/ + 4 /*子指令id和子指令长度*/);
                SMLK_UINT16 cmd_len_ = cmd_data.event_len + 4;
                // SMLK_LOGD("cmd_len_ = 0x%04x", cmd_len_);
                SMLK_UINT8 cmd_group_id = 0x01; //所有涉及tsp的avm消息的指令组id都为01
                SMLK_UINT8 cmd_id = 0x00;
                AvmMessageBuilder builder;
                switch (queue_msg.m_general_head.msg_id)
                {
                case JTT808_8F61_AVM_REAL_TIME_VIDEO: /*0x8F61:real time video stream*/
                {
                    cmd_id = 0x02;
                    cmd_data.child_cmd_id = 0x0001;
                }
                break;
                case JTT808_9102_AVM_REAL_TIME_VIDEO_CTRL: /*0x9102:real time video ctrl*/
                {
                    cmd_id = 0x02;
                    cmd_data.child_cmd_id = 0x0002;
                }
                break;
                case JTT808_9205_AVM_HISTRORY_VIDEO_LIST_QUERY: /*0x9205*/
                {
                    cmd_id = 0x03;
                    cmd_data.child_cmd_id = 0x0001;
                }
                break;
                case JTT808_8F62_AVM_HISTRORY_VIDEO_PLAY: /*0x8F62*/
                {
                    cmd_id = 0x03;
                    cmd_data.child_cmd_id = 0x0003;
                }
                break;
                case JTT808_9202_AVM_HISTRORY_VIDEO_PLAY_CTRL: /*0x9202:play ctrl  pause, stop ...*/
                {
                    cmd_id = 0x03;
                    cmd_data.child_cmd_id = 0x0004;
                }
                break;
                case JTT808_9206_AVM_HISTRORY_VIDEO_UPLOAD: /*0x9206:upload req*/
                {
                    cmd_id = 0x04;
                    cmd_data.child_cmd_id = 0x0001;
                }
                break;
                case JTT808_9207_AVM_HISTRORY_VIDEO_UPLOAD_CTRL: /*0x9207:TSP通知TBox暂停、继续或取消正在传输中的所有文件*/
                {
                    cmd_id = 0x04;
                    cmd_data.child_cmd_id = 0x0003;
                }
                break;
                case JTT808_8F49_AVM_EVENT_REPORT: /*0x8F49:event report(其实这应该是时间上报的响应)*/
                {
                    cmd_group_id = 0x01;
                    cmd_id = 0x05;
                }
                break;
                default:
                    return;
                }
                SMLK_BOOL not_block = false;
                cmd_data.event_len = htobe16(cmd_data.event_len);
                if (queue_msg.m_general_head.msg_id != JTT808_8F49_AVM_EVENT_REPORT)
                {
                    cmd_data.child_cmd_id = htobe16(cmd_data.child_cmd_id);
                    std::vector<SMLK_UINT8> send_;
                    send_.insert(send_.end(), (SMLK_UINT8 *)&cmd_data, (SMLK_UINT8 *)&cmd_data + sizeof(cmd_data));
                    send_.insert(send_.end(), queue_msg.data.begin(), queue_msg.data.end());
                    if ((queue_msg.m_general_head.msg_id == JTT808_9205_AVM_HISTRORY_VIDEO_LIST_QUERY) ||
                        (queue_msg.m_general_head.msg_id == JTT808_9206_AVM_HISTRORY_VIDEO_UPLOAD))
                    {
                        queue_msg.m_general_head.seq_id = be16toh(queue_msg.m_general_head.seq_id);
                        send_.insert(send_.end(), (SMLK_UINT8 *)&queue_msg.m_general_head.seq_id, (SMLK_UINT8 *)&queue_msg.m_general_head.seq_id + sizeof(SMLK_UINT16));
                        total_len_ = total_len_ + 2;
                        cmd_len_ = cmd_len_ + 2;
                        cmd_data.event_len = cmd_data.event_len + 2;
                    }
                    builder.TboxtoDevice().total_len(total_len_, 0).cmd_group(cmd_group_id).cmd_id(cmd_id).cmd_len(cmd_len_, 0).cmd_data(send_.data(), send_.size());
                    avm_msg_ = builder.end();
                }
                else
                {
                    /*8f49设置附件上传url*/
                    if (queue_msg.m_data_8f49_f368.fun_id == 0x0021)
                    {
                        builder.TboxtoDevice().total_len((4 + queue_msg.m_data_8f49_f368.param_len), 0).cmd_group(cmd_group_id).cmd_id(cmd_id).cmd_len((queue_msg.m_data_8f49_f368.param_len), 0).cmd_data((SMLK_UINT8 *)queue_msg.m_data_8f49_f368.host.data(), queue_msg.m_data_8f49_f368.host.size());
                        avm_msg_ = builder.end();
                    }
                    /*当收到0f49上报的回复时无需下发给dms*/
                    else if (queue_msg.m_data_8f49_f368.fun_id == 0x0023)
                        continue;
                }
                /*保存tsp消息头信息*/
                memcpy(&avm_msg_.getTspHead(), &queue_msg.m_general_head, sizeof(GenrnalHead));
/*****************************************************************************
 *                      Tsp消息解析,AVM&DMS组包 结束                           *
 *****************************************************************************/

/*开始分配发送任务到Avm or Dms服务*/
#if 0
            if (m_sp_avmservice.get())
            {
#if !AVM_SYNC_SERVICE_OPEN
                m_sp_avmservice->SendMsg(avm_msg_);
#else
                SMLK_LOGD("m_sp_avmservice ready to send msg to avm side ...");
                if (!m_sp_avmservice->SendMsg(avm_msg_, not_block))
                {
                    SMLK_LOGD("m_sp_avmservice get avm result failed ...");
                    SendCommonResp(queue_msg, AVMCommonRC::CommonRC_FAILED);
                    continue;
                }
                SMLK_LOGD("m_sp_avmservice get avm result success ...");
                // TODO send 2 server
                if (avm_msg_.getAvmHead().device_type == 0x01) // response success if msg from avm
                {
                    OnDeviceMessage(avm_msg_);
                }
#endif
            }
#endif
                /*将Tsp消息转换为Dms消息并且发送*/
                if (m_dms_exist)
                {
                    if (!m_sp_dmsservice->SendMsg(avm_msg_, not_block))
                    {
                        SMLK_LOGD("m_sp_dmsservice get avm result failed ...");
                        continue;
                    }
                    if (avm_msg_.getAvmHead().device_type == 0x01) // response success if msg from avm
                    {
                        avm_msg_.setDeviceType(DeviceType::DEV_DMS);
                        OnDeviceMessage(avm_msg_);
                    }
                }
            }
            SMLK_LOGD("*********TspMsgProcess Thread End***********");
        }

        void VedioProcessorManager::OnDeviceMessage(AvmMessage &msg)
        {
            auto ret = AVMCommonRC::CommonRC_OK;
            SMLK_BOOL need_send_child_data = false;
            TspQueueMsg tspmsg_;
            memcpy(&tspmsg_.m_general_head, &msg.getTspHead(), sizeof(TspHead));
            tspmsg_.m_general_head.protocol = (SMLK_UINT8)ProtoclID::E_PROT_JTT808;
            tspmsg_.m_general_head.qos = QOS_SEND_TCP_TIMES;
            /*实时视频查看请求响应*/
            if (msg.getAvmHead().cmd_group_id == 0x01 && msg.getAvmHead().cmd_id == 0x82)
            {
                if ((AvmErrCode)msg.getAvmResult().res != AvmErrCode::AVM_SUCCESS)
                {
                    ret = AVMCommonRC::CommonRC_FAILED;
                }
                goto sendResult;
            }
            /*视频资源列表查询,视频回放请求,视频回放控制响应*/
            if (msg.getAvmHead().cmd_group_id == 0x01 && msg.getAvmHead().cmd_id == 0x83)
            {
                if ((AvmErrCode)msg.getAvmResult().res != AvmErrCode::AVM_SUCCESS)
                {
                    ret = AVMCommonRC::CommonRC_FAILED;
                }
                if (msg.getAvmResult().id == 0x02) // history list resp
                {
                    need_send_child_data = true;
                    tspmsg_.m_general_head.msg_id = JTT808_1205_AVM_HISTRORY_VIDEO_LIST_ARK;
                }
                goto sendResult;
            }
            /*历史视频上传*/
            if (msg.getAvmHead().cmd_group_id == 0x01 && msg.getAvmHead().cmd_id == 0x84)
            {
                if ((AvmErrCode)msg.getAvmResult().res != AvmErrCode::AVM_SUCCESS)
                {
                    ret = AVMCommonRC::CommonRC_FAILED;
                }
                if (msg.getAvmResult().id == 0x02) // history list upload resp
                {
                    need_send_child_data = true;
                    tspmsg_.m_general_head.msg_id = JTT808_1206_AVM_HISTRORY_VIDEO_UPLOAD_ARK;
                }
                goto sendResult;
            }
            /*附件上传url设置响应*/
            if (msg.getAvmHead().cmd_group_id == 0x01 && msg.getAvmHead().cmd_id == 0x85)
            {
                SMLK_UINT8 resp = *msg.child_data();
                struct
                {
                    SMLK_UINT16 cont_len;
                    SMLK_UINT16 fun_id;
                    SMLK_UINT16 seq_id;
                    SMLK_UINT8 res;
                } __attribute__((__packed__)) resp_data;
                resp_data.cont_len = htobe16(0x0005);
                resp_data.fun_id = htobe16(0x0021);
                resp_data.seq_id = htobe16(msg.getTspHead().seq_id);
                resp_data.res = resp;
                tspmsg_.m_general_head.msg_id = JTT808_0F49_AVM_EVENT_REPORT_ARK;
                SendActionResp(tspmsg_, (SMLK_UINT8 *)&resp_data, sizeof(resp_data));
                return;
            }
            /*报警事件上报*/
            if (msg.getAvmHead().cmd_group_id == 0x01 && msg.getAvmHead().cmd_id == 0x01)
            {
                AvmMessageBuilder builder_;
                std::vector<SMLK_UINT8> _f49_data;
                std::size_t _f49_len = 0;
                builder_.TboxtoDevice().total_len(0x0007, 0).cmd_group(0x01).cmd_id(0x81).cmd_len(0x0003, 0).cmd_data((SMLK_UINT8 *)&msg.getAvmResult().id, sizeof(SMLK_UINT16));
                AvmErrCode succ_code_ = AvmErrCode::AVM_SUCCESS;
                AvmErrCode err_code_ = AvmErrCode::AVM_FAILED;
                if (!m_sp_dataproc.get())
                {
                    builder_.cmd_data((SMLK_UINT8 *)&err_code_, sizeof(SMLK_UINT8));
                }
                else
                {
                    _f49_len = m_sp_dataproc->DecodeAvmMessage(msg, _f49_data);
                    // TODO if send 2 server resp 2 avm
                    if (_f49_len != _f49_data.size())
                    {
                        builder_.cmd_data((SMLK_UINT8 *)&err_code_, sizeof(SMLK_UINT8));
                    }
                    else
                    {
                        tspmsg_.m_general_head.msg_id = JTT808_0F49_AVM_EVENT_REPORT_ARK;
                        tspmsg_.m_general_head.qos = QOS_SEND_ALWAYS;
                        SendActionResp(tspmsg_, _f49_data.data(), _f49_len);
                        builder_.cmd_data((SMLK_UINT8 *)&succ_code_, sizeof(SMLK_UINT8));
                        // return; // wait server parse success resp, here need save result id for 8f49 fun 0x23 response set
                    }
                }
                AvmMessage tbox_msg = builder_.end();
                m_sp_dmsservice->SendMsg(tbox_msg, true);
                std::string log = tools::uint8_2_string((SMLK_UINT8 *)tbox_msg.data(), tbox_msg.size());
                SMLK_LOGD("[Tbox2Dms][Gourp:0x%02x][Cmd:0x%02x]:%s", tbox_msg.getAvmHead().cmd_group_id, tbox_msg.getAvmHead().cmd_id, log.c_str());
                return;
            }

        sendResult:
            // SMLK_LOGD("SendCommonResp seq_id = 0x%04x", tspmsg_.m_general_head.seq_id);
            // SendCommonResp(tspmsg_, ret);
            if (need_send_child_data)
            {
                SendActionResp(tspmsg_, msg.child_data(), msg.child_size());
            }
        }

        void VedioProcessorManager::OnTspIndication(IN IpcTspHead &ipc_head, IN void *data, std::size_t len)
        {
            TspQueueMsg tsp_msg;
            if ((JTT808_8F61_AVM_REAL_TIME_VIDEO == ipc_head.msg_id) ||
                (JTT808_9102_AVM_REAL_TIME_VIDEO_CTRL == ipc_head.msg_id) ||
                (JTT808_9205_AVM_HISTRORY_VIDEO_LIST_QUERY == ipc_head.msg_id) ||
                (JTT808_8F62_AVM_HISTRORY_VIDEO_PLAY == ipc_head.msg_id) ||
                (JTT808_9202_AVM_HISTRORY_VIDEO_PLAY_CTRL == ipc_head.msg_id) ||
                (JTT808_9206_AVM_HISTRORY_VIDEO_UPLOAD == ipc_head.msg_id) ||
                (JTT808_9207_AVM_HISTRORY_VIDEO_UPLOAD_CTRL == ipc_head.msg_id) ||
                (JTT808_8F49_AVM_EVENT_REPORT == ipc_head.msg_id))
            {
                tsp_msg.m_general_head.msg_id = ipc_head.msg_id;
                tsp_msg.m_general_head.seq_id = ipc_head.seq_id;
                tsp_msg.m_general_head.protocol = ipc_head.protocol;
                tsp_msg.m_general_head.qos = ipc_head.qos;
                tsp_msg.m_general_head.priority = ipc_head.priority;
                tsp_msg.m_general_head.reserved_dw = ipc_head.reserved_dw;
                tsp_msg.data.insert(tsp_msg.data.end(), (SMLK_UINT8 *)data, (SMLK_UINT8 *)data + len);
                std::string log = tools::uint8_2_string((SMLK_UINT8 *)data, len);
                SMLK_UINT16 temp_msg_id = be16toh(ipc_head.msg_id);
                SMLK_UINT16 temp_seq_id = be16toh(ipc_head.seq_id);
                std::string str_msg_id = tools::uint8_2_string((SMLK_UINT8 *)&temp_msg_id, sizeof(SMLK_UINT16));
                std::string str_seq_id = tools::uint8_2_string((SMLK_UINT8 *)&temp_seq_id, sizeof(SMLK_UINT16));
                std::string show_msg = "[Tsp2Tbox][MsgID:0x" + str_msg_id + "][SN:0x" + str_seq_id + "]";
                tools::print_long_string(log, show_msg);
            }
            else
                return;
            AVMCommonRC common_rc = AVMCommonRC::CommonRC_OK;
            switch (tsp_msg.m_general_head.msg_id)
            {
            case JTT808_8F61_AVM_REAL_TIME_VIDEO:
            {
                Decode8F61((SMLK_UINT8 *)data, tsp_msg);
            }
            break;
            case JTT808_9102_AVM_REAL_TIME_VIDEO_CTRL:
            {
                memcpy(&tsp_msg.m_data_9102, data, sizeof(Data9102));
                SMLK_LOGD("[MsgId:0x9102]:{[ChannelId]=%d [CmdId]=%d [CloseVideoType]=%d  [CodeType]=%d}", tsp_msg.m_data_9102.channel_id, tsp_msg.m_data_9102.cmd_id, tsp_msg.m_data_9102.close_video_type, tsp_msg.m_data_9102.code_type);
            }
            break;
            case JTT808_9205_AVM_HISTRORY_VIDEO_LIST_QUERY:
            {
                memcpy(&tsp_msg.m_data_9205, data, sizeof(Data9205));
                SMLK_LOGD("[MsgId:0x9205]:{[ChannelId]=%d [StreamType]=%d [CodeType]=%d [StorageType]=%d}", tsp_msg.m_data_9205.channel_id, tsp_msg.m_data_9205.stream_type, tsp_msg.m_data_9205.code_type, tsp_msg.m_data_9205.storage_type);
            }
            break;
            case JTT808_8F62_AVM_HISTRORY_VIDEO_PLAY:
            {
                std::size_t position = Decode8F61((SMLK_UINT8 *)data, tsp_msg);
                if (len - position < 1)
                {
                    SendCommonResp(tsp_msg, AVMCommonRC::CommonRC_UNSUPPORT);
                    return;
                }
                memcpy(&tsp_msg.m_data_8f62, (SMLK_UINT8 *)data + position, len - position);
                SMLK_LOGD("[MsgId:0x8F61]:{[StorageType]=%d [PlayType]=%d [PlayStepNum]=%d}", tsp_msg.m_data_8f62.storage_type, tsp_msg.m_data_8f62.play_type, tsp_msg.m_data_8f62.play_step_num);
            }
            break;
            case JTT808_9202_AVM_HISTRORY_VIDEO_PLAY_CTRL:
            {
                memcpy(&tsp_msg.m_data_9202, data, sizeof(Data9202));
                SMLK_LOGD("[MsgId:0x9202]:{[ChannelId]=%s [PlayType]=%d [PlayStepNum]=%d]", tsp_msg.m_data_9202.channel_id, tsp_msg.m_data_9202.play_type, tsp_msg.m_data_9202.play_step_num);
            }
            break;
            case JTT808_9206_AVM_HISTRORY_VIDEO_UPLOAD:
            {
                Decode9206((SMLK_UINT8 *)data, tsp_msg, len);
                SMLK_LOGD("[MsgId:0x9206]=%s", tsp_msg.m_data_9206.toString().c_str());
            }
            break;
            case JTT808_9207_AVM_HISTRORY_VIDEO_UPLOAD_CTRL:
            {
                memcpy(&tsp_msg.m_data_9207, data, sizeof(Data9207));
                SMLK_LOGD("[MsgId:0x9207]:{[SeqId]=%d [UploadCmd]=%d]", tsp_msg.m_data_9207.seq_id, tsp_msg.m_data_9207.upload_cmd);
            }
            break;
            case JTT808_8F49_AVM_EVENT_REPORT:
            {
                SMLK_UINT8 *temp_data = (SMLK_UINT8 *)data;
                SMLK_UINT16 data_len_ = 0x0000;
                memcpy(&data_len_, temp_data, sizeof(SMLK_UINT16));
                temp_data += 2;
                memcpy(&tsp_msg.m_data_8f49_f368.fun_id, temp_data, sizeof(SMLK_UINT16));
                tsp_msg.m_data_8f49_f368.fun_id = be16toh(tsp_msg.m_data_8f49_f368.fun_id);
                temp_data += 2;
                memcpy(&tsp_msg.m_data_8f49_f368.param_id, temp_data, sizeof(SMLK_UINT16));
                tsp_msg.m_data_8f49_f368.param_id = be16toh(tsp_msg.m_data_8f49_f368.param_id);
                temp_data += 2;

                if (tsp_msg.m_data_8f49_f368.fun_id == 0x0021)
                {
                    memcpy(&tsp_msg.m_data_8f49_f368.param_len, temp_data, sizeof(SMLK_UINT16));
                    tsp_msg.m_data_8f49_f368.param_len = be16toh(tsp_msg.m_data_8f49_f368.param_len);
                    tsp_msg.m_data_8f49_f368.param_len -= 1;
                    // here we + 3 because feature content also has one byte host length!!!!
                    temp_data += 3;
                    tsp_msg.m_data_8f49_f368.host.insert(tsp_msg.m_data_8f49_f368.host.end(), temp_data, temp_data + tsp_msg.m_data_8f49_f368.param_len);
                    SMLK_LOGD("[FunId:0x%04x][ParamId:0xF368]:{[Host]=%s}", tsp_msg.m_data_8f49_f368.fun_id, tsp_msg.m_data_8f49_f368.host.c_str());
                    common_rc = AVMCommonRC::CommonRC_OK;
                }
                else if (tsp_msg.m_data_8f49_f368.fun_id == 0x0023)
                {
                    memcpy(&tsp_msg.m_data_8f49_f368.result, temp_data, sizeof(SMLK_UINT8));
                }
                else
                {
                    SMLK_LOGD("[MsgID:0x8F49][UnsupportFunId]=0x%02x", tsp_msg.m_data_8f49_f368.fun_id);
                    common_rc = AVMCommonRC::CommonRC_UNSUPPORT;
                    return;
                }
            }
            break;
            default:
                return;
            }
            SendCommonResp(tsp_msg, common_rc);
            m_tsp_queue.put(tsp_msg);
        }

        void VedioProcessorManager::SendCommonResp(IN TspQueueMsg &msg, AVMCommonRC result)
        {
            IpcTspHead ipc_head;
            ipc_head.msg_id = JTT808_AVM_COMMON_ARK;
            ipc_head.seq_id = htobe16(msg.m_general_head.seq_id);
            ipc_head.protocol = (SMLK_UINT8)(msg.m_general_head.protocol);
            ipc_head.qos = msg.m_general_head.qos;
            ipc_head.priority = msg.m_general_head.priority;
            struct
            {
                SMLK_UINT16 seq_id; /*对应平台消息的流水号*/
                SMLK_UINT16 msg_id; /*对应平台消息de ID*/
                SMLK_UINT8 result;
            } __attribute__((__packed__)) response;
            response.seq_id = htobe16(msg.m_general_head.seq_id);
            response.msg_id = htobe16(msg.m_general_head.msg_id);
            response.result = (SMLK_UINT8)result;
            std::string log = tools::uint8_2_string((SMLK_UINT8 *)&response, sizeof(response));
            SMLK_LOGD("[Tbox2Tsp][MsgId:0x%04x][SN:0x%04x]=%s", JTT808_AVM_COMMON_ARK, msg.m_general_head.seq_id, log.c_str());
            TspServiceApi::getInstance()->SendMsg(ipc_head, (SMLK_UINT8 *)&response, (std::size_t)sizeof(response));
        }

        void VedioProcessorManager::SendActionResp(IN TspQueueMsg &msg, SMLK_UINT8 *data, std::size_t len)
        {
            IpcTspHead ipc_head;
            ipc_head.msg_id = msg.m_general_head.msg_id;
            ipc_head.seq_id = msg.m_general_head.seq_id;
            ipc_head.protocol = (SMLK_UINT8)(msg.m_general_head.protocol);
            ipc_head.qos = msg.m_general_head.qos;
            ipc_head.priority = msg.m_general_head.priority;
            std::string log = tools::uint8_2_string((SMLK_UINT8 *)data, len);
            SMLK_LOGD("[Tbox2Tsp][MsgId:0x%04x][SN:0x%04x]=%s", ipc_head.msg_id, ipc_head.seq_id, log.c_str());
            TspServiceApi::getInstance()->SendMsg(ipc_head, data, len);
        }

        std::size_t VedioProcessorManager::Decode8F61(IN SMLK_UINT8 *data, TspQueueMsg &msg)
        {
            int position_ = 0;
            SMLK_UINT8 len_temp = data[position_];
            position_ += 1;

            std::string host_;
            SMLK_UINT8 host_temp[len_temp + 1];
            memcpy(host_temp, data + position_, len_temp);
            host_temp[len_temp] = '\0';
            msg.m_data_8f61.host = reinterpret_cast<const char *>(host_temp);
            position_ += len_temp;
            struct
            {
                SMLK_UINT8 channel_id;
                SMLK_UINT8 data_type;
                SMLK_UINT8 code_type;
            } __attribute__((__packed__)) avm_data;
            memcpy(&avm_data, data + position_, sizeof(avm_data));
            msg.m_data_8f61.channel_id = avm_data.channel_id;
            msg.m_data_8f61.data_type = avm_data.data_type;
            msg.m_data_8f61.code_type = avm_data.code_type;
            position_ += 3;
            SMLK_LOGD("[MsgId:0x8F61]:{[Host]=%s [ChannelId]=%d [DataType]=%d [CodeType]=%d}", msg.m_data_8f61.host.c_str(), msg.m_data_8f61.channel_id, msg.m_data_8f61.data_type, msg.m_data_8f61.code_type);
            return position_;
        }

        void VedioProcessorManager::Decode9206(IN SMLK_UINT8 *data, TspQueueMsg &msg, std::size_t len)
        {
            int position_ = 0;

            SMLK_UINT8 len_temp = data[position_];
            position_ += 1;

            SMLK_UINT8 host_temp[len_temp + 1];
            memcpy(host_temp, data + position_, len_temp);
            host_temp[len_temp] = '\0';
            msg.m_data_9206.host = reinterpret_cast<const char *>(host_temp);
            position_ += len_temp;
            SMLK_LOGD("Decode9206 len_temp = %d", len_temp);
            SMLK_LOGD("Decode9206 host_ = %s", msg.m_data_9206.host.c_str());

            memcpy(&msg.m_data_9206.port, data + position_, 2);
            position_ += 2;
            msg.m_data_9206.port = htobe16(msg.m_data_9206.port);
            SMLK_LOGD("Decode9206 port = %d", (SMLK_UINT32)msg.m_data_9206.port);

            len_temp = data[position_];
            SMLK_UINT8 usr_temp[len_temp + 1];
            position_ += 1;
            memcpy(usr_temp, data + position_, len_temp);
            usr_temp[len_temp] = '\0';
            msg.m_data_9206.user = reinterpret_cast<const char *>(usr_temp);
            position_ += len_temp;
            SMLK_LOGD("Decode9206 len_temp = %d", len_temp);
            SMLK_LOGD("Decode9206 usr_ = %s", msg.m_data_9206.user.c_str());

            len_temp = data[position_];
            SMLK_UINT8 passwd_temp[len_temp + 1];
            position_ += 1;
            memcpy(passwd_temp, data + position_, len_temp);
            passwd_temp[len_temp] = '\0';
            msg.m_data_9206.passwd = reinterpret_cast<const char *>(passwd_temp);
            position_ += len_temp;
            SMLK_LOGD("Decode9206 len_temp = %d", len_temp);
            SMLK_LOGD("Decode9206 passwd_ = %s", msg.m_data_9206.passwd.c_str());

            len_temp = data[position_];
            SMLK_UINT8 server_path_temp[len_temp + 1];
            position_ += 1;
            memcpy(server_path_temp, data + position_, len_temp);
            server_path_temp[len_temp] = '\0';
            msg.m_data_9206.server_path = reinterpret_cast<const char *>(server_path_temp);
            position_ += len_temp;
            SMLK_LOGD("Decode9206 len_temp = %d", len_temp);
            SMLK_LOGD("Decode9206 server_path_ = %s", msg.m_data_9206.server_path.c_str());

            struct
            {
                SMLK_UINT8 channel_id;
                SMLK_UINT8 bcd_start_time[6];
                SMLK_UINT8 bcd_end_time[6];
            } __attribute__((__packed__)) avm_data;
            memcpy(&avm_data, data + position_, len - position_);
        }

        SMLK_RC VedioProcessorManager::Init()
        {
            if (CheckStatus(BaseModule::Initialized))
            {
                SMLK_LOGV("Avm Manager already initialized, do nothing");
                return SMLK_RC::RC_OK;
            }
            /*************************************************
             *                    TSP模块                     *
             *************************************************/
            /*注册tsp消息接收接口*/
            while (true)
            {
                SMLK_RC tsp_rc = TspServiceApi::getInstance()->Init(ModuleID::E_MOUDLE_AVM_CTRL_APP);
                if (SMLK_RC::RC_OK != tsp_rc)
                {
                    SMLK_LOGW("TspServiceApi Init failed. Retry");
                    sleep(1);
                    continue;
                }
                SMLK_LOGI("TspServiceApi->Init() OK.");
                break;
            }
            std::vector<SMLK_UINT16> msg_id;
            msg_id.push_back(JTT808_8F61_AVM_REAL_TIME_VIDEO);
            msg_id.push_back(JTT808_9102_AVM_REAL_TIME_VIDEO_CTRL);
            msg_id.push_back(JTT808_9205_AVM_HISTRORY_VIDEO_LIST_QUERY);
            msg_id.push_back(JTT808_8F62_AVM_HISTRORY_VIDEO_PLAY);
            msg_id.push_back(JTT808_9202_AVM_HISTRORY_VIDEO_PLAY_CTRL);
            msg_id.push_back(JTT808_9206_AVM_HISTRORY_VIDEO_UPLOAD);
            msg_id.push_back(JTT808_9207_AVM_HISTRORY_VIDEO_UPLOAD_CTRL);
            msg_id.push_back(JTT808_8F49_AVM_EVENT_REPORT);
            TspServiceApi::getInstance()->RegisterMsgCB(ProtoclID::E_PROT_JTT808, msg_id, std::bind(&VedioProcessorManager::OnTspIndication, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            /*************************************************
             *                  移动通信模块                  *
             *************************************************/
            /*初始化移动网络服务模块*/
            if (!smartlink_sdk::Telephony::GetInstance()->Init(ModuleID::E_MOUDLE_AVM_CTRL_APP))
            {
                SMLK_LOGF("AVM_CTRL fail to init Telephony service API interface.");
                return SMLK_RC::RC_ERROR;
            }
            SMLK_LOGI("Telephony->Init() OK.");
            /*************************************************
             *                 系统时间管理模块               *
             *************************************************/
            /*初始化系统时间管理模块*/
            if (!smartlink_sdk::SysTime::GetInstance()->Init(ModuleID::E_MOUDLE_AVM_CTRL_APP))
            {
                SMLK_LOGF("AVM_CTRL fail to init SysTime service API interface.");
                return SMLK_RC::RC_ERROR;
            }
            else
            {
                /*如果当前系统时间发现变化,同步更新DMS/AVM系统时间*/
                std::vector<smartlink_sdk::SysTimeEventId> timer_event_vec;
                timer_event_vec.push_back(smartlink_sdk::SysTimeEventId::E_TIME_EVENT_TIME_CHANGED);
                smartlink_sdk::SysTime::GetInstance()->RegEventCB(timer_event_vec, [this](smartlink_sdk::SysTimeEventId event_id, void *data, int len) -> SMLK_UINT8
                                                                  {
                SMLK_LOGD("OnTimerIndication(AVM_CTRL)  event_id=%d.", event_id);
                switch (event_id)
                {
                case smartlink_sdk::SysTimeEventId::E_TIME_EVENT_TIME_CHANGED:
                {
                    smartlink_sdk::SysTimeEventInfo *info = (smartlink_sdk::SysTimeEventInfo *)data;
                    SMLK_LOGD("Timesync=%d,TimeSource=%d", info->timesync, info->source);
                    if (info->timesync == 1)
                    {
                        if (m_sp_avm_util.get())
                        {
                            m_sp_avm_util->SyncTime();
                        }
                    }
                }
                break;
                default:
                    SMLK_LOGE("[Timer(AVM_CTRL)] unsupported event id: %d", (SMLK_UINT8)event_id);
                    return 0;
                }
                return 0; });
                SMLK_LOGI("SysTime->Init() OK.");
            }
            /*************************************************
             *                  系统参数模块                  *
             *************************************************/
            /*初始化系统参数模块*/
            if (!smartlink_sdk::SysProperty::GetInstance()->Init(ModuleID::E_MOUDLE_AVM_CTRL_APP))
            {
                SMLK_LOGF("AVM_CTRL fail to init SysProperty service API interface.");
                return SMLK_RC::RC_ERROR;
            }
            SMLK_LOGI("SysProperty->Init() OK.");
            /*判断当前AVM&DMS的存在状态*/
            std::string property = SYS_PRO_NAME_AVM;
            std::string str_avm_type, str_dms_type;
            smartlink_sdk::RtnCode rc;
            do
            {
                rc = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, str_avm_type);
            } while (rc != smartlink_sdk::RtnCode::E_SUCCESS);
            if ((stoi(str_avm_type) != (int)AVMType::AVM_NONE))
            {
                m_avm_exist = true;
                SMLK_LOGI("[Avm][Type]=%d", stoi(str_avm_type));
            }
            else
            {
                m_avm_exist = false;
                SMLK_LOGI("[Avm] doesn't exist!");
            }
            property = SYS_PRO_NAME_DMS;
            do
            {
                rc = smartlink_sdk::SysProperty::GetInstance()->GetValue(property, str_dms_type);
            } while (rc != smartlink_sdk::RtnCode::E_SUCCESS);
            if ((stoi(str_dms_type) != (int)DMSType::DMS_NONE))
            {
                m_dms_exist = true;
                SMLK_LOGI("[Dms][Type]=%d", stoi(str_dms_type));
            }
            else
            {
                m_dms_exist = false;
                SMLK_LOGI("[Dms] doesn't exist!");
            }

            /*************************************************
             *                   内部模块                     *
             *************************************************/
            /*初始化数据处理模块*/
            m_sp_dataproc = std::make_shared<DataProcessor>();
            m_sp_dataproc->Init();

            SetStatus(BaseModule::Initialized);
            SMLK_LOGI("Avm Manager Init successfullly.");
            return SMLK_RC::RC_OK;
        }

        SMLK_RC VedioProcessorManager::Start()
        {
            if (IsRunning())
            {
                SMLK_LOGV("Avm Manager already started, do nothing.");
                return SMLK_RC::RC_OK;
            }
            SetStatus(BaseModule::Running);
            if (m_dms_exist)
                AvmServiceInitOrRestart(m_sp_dmsservice, m_sp_dms_util, DMSPORT);
            if (m_avm_exist)
                AvmServiceInitOrRestart(m_sp_avmservice, m_sp_avm_util, AVMPORT);
            m_tsp_msg_thread = std::thread(&VedioProcessorManager::TspMsgProcess, this);
            m_sp_dataproc->Start();
            return SMLK_RC::RC_OK;
        }

        void VedioProcessorManager::AvmServiceInitOrRestart(std::shared_ptr<AvmServiceBase> &sptrService, std::shared_ptr<AvmCustomUtil> &sptrUtil, SMLK_UINT16 port)
        {
            if (sptrService.get())
            {
                sptrService->Stop();
                sptrService.reset();
            }
            sptrService = std::make_shared<AVMSyncService>("2.2.2.2", port);
            if (sptrService.get())
            {
                sptrService->Init();
                sptrUtil = std::make_shared<AvmCustomUtil>(sptrService);
                auto func_ = std::bind(&VedioProcessorManager::OnDeviceMessage, this, std::placeholders::_1);
                sptrService->RegisterEventListener(func_);
                if (m_sp_avm_util == sptrUtil)
                {
                    sptrService->RegisterNotifyListener([this]()
                                                        {
                    if (m_sp_avm_util.get())
                    {
                        std::string sim_cell("");
                        smartlink_sdk::RtnCode rc = smartlink_sdk::Telephony::GetInstance()->GetIMSI(sim_cell);
                        if((smartlink_sdk::RtnCode::E_SUCCESS  == rc)&&(0 != sim_cell.size()))
                        {
                            SMLK_LOGI("Telephony::GetInstance()->GetIMSI() ok, GetIMSI:(%s).",sim_cell.c_str());
                            std::string imsi_temp;
                            std::string ter_zero = "00";
                            if(sim_cell.size() < 13)
                            {
                                imsi_temp.insert(imsi_temp.end(), sim_cell.begin(), sim_cell.end());
                            }
                            else
                            {
                                imsi_temp.insert(imsi_temp.end(), sim_cell.end()-11, sim_cell.end());
                                imsi_temp.insert(imsi_temp.end(), ter_zero.begin(),ter_zero.end());
                            }
                            SMLK_LOGI("imsi_temp(%s)",imsi_temp.c_str());
                            m_sp_avm_util->SyncAll(imsi_temp);
                        }
                    } },
                                                        NotifyType::AVM_NOTIFY_ONCONNECT);
                }
                else if (m_sp_dms_util == sptrUtil)
                {
                    sptrService->RegisterNotifyListener([this]()
                                                        {
                    if (m_sp_dms_util.get())
                    {
                        std::string sim_cell("");
                        smartlink_sdk::RtnCode rc = smartlink_sdk::Telephony::GetInstance()->GetIMSI(sim_cell);
                        if((smartlink_sdk::RtnCode::E_SUCCESS  == rc)&&(0 != sim_cell.size())){
                            SMLK_LOGI("GetIMSI=%s",sim_cell.c_str());
                            std::string imsi_temp;
                            std::string ter_zero = "00";
                            if(sim_cell.size() < 13)
                            {
                                imsi_temp.insert(imsi_temp.end(), sim_cell.begin(), sim_cell.end());
                            }
                            else
                            {
                                imsi_temp.insert(imsi_temp.end(), sim_cell.end()-11, sim_cell.end());
                                imsi_temp.insert(imsi_temp.end(), ter_zero.begin(),ter_zero.end());
                            }
                            SMLK_LOGI("SetIMSI=%s",imsi_temp.c_str());
                            m_sp_dms_util->SyncAll(imsi_temp);
                        }
                    } },
                                                        NotifyType::AVM_NOTIFY_ONCONNECT);
                }
                sptrService->Start();
            }
        }

        void VedioProcessorManager::Stop()
        {
            SetStatus(BaseModule::Stopping);
            if (m_sp_avmservice.get())
                m_sp_avmservice->Stop();
            m_sp_avmservice.reset();
            m_sp_avm_util.reset();

            if (m_sp_dmsservice.get())
                m_sp_dmsservice->Stop();
            m_sp_dmsservice.reset();
            m_sp_avm_util.reset();

            if (m_sp_dataproc.get())
                m_sp_dataproc->Stop();
            m_sp_dataproc.reset();
        }

        void VedioProcessorManager::Loop()
        {
            if (m_tsp_msg_thread.joinable())
            {
                m_tsp_msg_thread.join();
            }
        }

        VedioProcessorManager::VedioProcessorManager()
        {
            SetStatus(BaseModule::Uninitialized);
        }
        VedioProcessorManager::~VedioProcessorManager()
        {
        }
    }
}
/*****************************************************************************/
/**
 * \file       rctrl_jtt808_msg8f42.cpp
 * \author     huangxin
 * \date       2020/11/30
 * \version    Tbox2.0 V1
 * \brief      jtt808的消息id 0x8f42和0x0f42解析
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/

/*****************************************************************************
 *                               头文件引用                                   *
 *****************************************************************************/

#include <iostream>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <smlk_error.h>
#include "vehctrl_general_definition.h"
#include <unistd.h>
#include "vehctrl_jtt808.h"
#include "vehctrl_service.h"
#include "vehctrl_jtt808_msg8f42.h"
#include "vehctrl_queue.h"
#include "smlk_property.h"
#include "vehctrl_status_file.h"
#include "smartlink_sdk_sys_property.h"
#include "smartlink_sdk_wlan.h"
#include "smlk_tools.h"

using namespace std;
using namespace smartlink;
using namespace smartlink_sdk;
using namespace Poco::Dynamic;

#define DASHBOARD_PERIOD 5 * 60 * 10
#define CHANNEL_COMM 0 //通信CAN通道-->通信CAN1

/**
 * @brief               8F42业务功能实现
 * @param indata        8F42下发指令内容
 * @param length        8F42下发指令长度
 * @param queue         0F42指令回复队列
 * @param is_encode     是否还有消贷业务需要处理
 *
 * @return 吉大错误码
 */
SMLK_RC DissectorMsg8F42::Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &queue, OUT SMLK_UINT8 &is_encode)
{
    Msg8F42Common *pcommon = (Msg8F42Common *)(indata);
    // std::string s_data = tools::uint8_2_string((SMLK_UINT8 *)&indata[0], length);
    // SMLK_LOGD("[Tsp2Tbox][MsgID:0x8F42]=%s,len=%d", s_data.c_str(), length);
    /*初始化参数*/
    RtnCode property_rc = RtnCode::E_SUCCESS;
    std::string prop_name; /*存储SysProp参数名*/
    SMLK_UINT8 result = COM_SET_SUCC;

    /*发送通用应答*/
    SMLK_UINT8 result_comm = SMLK_TSP_0001_RESULT_SUCCESS;
    DissectorJtt808Common::getInstance()->SendMsgCommonResp(queue.m_head, result_comm);
    /*0x03 获取VIN码*/
    if (CMD_VIN_GET == pcommon->cmd_id)
    {
        string vin;
        prop_name = SYS_PRO_NAME_VEHICLE_VIN;
        property_rc = SysProperty::GetInstance()->GetValue(prop_name, vin);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("get [name]=%s fail!", prop_name.c_str());
        }
        SMLK_LOGD("get vin=%s.", vin.c_str());
        SendMsg0F42(queue.m_head, pcommon->cmd_id, vin);
    }
    /*0x04 设置VIN码*/
    else if (CMD_VIN_SET == pcommon->cmd_id)
    {
        string vin;
        prop_name = SYS_PRO_NAME_VEHICLE_VIN;
        vin.insert(vin.end(), (char *)(pcommon + 1), (char *)(pcommon + 1) + length - sizeof(Msg8F42Common));
        SMLK_LOGD("[Tsp][Set][vin]=%s", vin.c_str());
        string vin_head = vin.substr(0, 2);
        transform(vin_head.begin(), vin_head.end(), vin_head.begin(), ::toupper);
        if (vin_head == "LF")
        {
            property_rc = SysProperty::GetInstance()->SetValue(prop_name, vin);
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("Set %s with %s failed.", prop_name.c_str(), vin.c_str());
                result = COM_SET_FAIL;
            }
            else
            {
                SMLK_LOGD("Set [name]=%s with %s successfully", prop_name.c_str(), vin.c_str());
            }
        }
        else
        {
            result = COM_SET_FAIL;
            SMLK_LOGW("Recv invalid vin!");
        }
        SendMsg0F42(queue.m_head, pcommon->cmd_id, result);
    }
    /*0x05 设置SSID和密码*/
    else if (CMD_SSID_PASWD_SET == pcommon->cmd_id)
    {
        std::string ssid, ssid_passwd, passwd;
        smartlink_sdk::RtnCode ret;
        ssid_passwd.insert(ssid_passwd.end(), (char *)(pcommon + 1), (char *)(pcommon + 1) + length - sizeof(Msg8F42Common));
        auto pos = ssid_passwd.find("\n");
        if (pos == string::npos)
        {
            SMLK_LOGW("Need line break!");
            result = COM_SET_FAIL;
        }
        else
        {
            ssid.assign(ssid_passwd.c_str(), pos);
            passwd.assign(ssid_passwd.c_str() + pos + 1, ssid_passwd.length() - pos - 2);
            SMLK_LOGD("[ssid]=%s [passwd]=%s [pos]=%d,", ssid.c_str(), passwd.c_str(), pos);

            std::string ssid_word;
            char buf[pos] = {'0'};

            strncpy(buf, ssid.c_str(), ssid.length());

            char buf2[2 * pos] = {'\0'};
            SMLK_RC rc = gbk_to_utf8(buf, pos, buf2, 2 * pos);
            if (rc != SMLK_RC::RC_OK)
            {
                SMLK_LOGE("code_convert error");
                result = COM_SET_FAIL;
            }
            ssid_word = buf2;
            // SMLK_LOGD("transform ssid as %s", ssid_word.c_str());

            smartlink_sdk::WifiApInfo info;
            ret = smartlink_sdk::Wlan::GetInstance()->GetApConfig(info);
            if (ret != smartlink_sdk::RtnCode::E_SUCCESS)
            {
                SMLK_LOGE("GetApConfig failed!, error code[%d]", (SMLK_UINT32)ret);
                result = COM_SET_FAIL;
                goto GeneResp;
            }
            info.password = passwd;

            ret = smartlink_sdk::Wlan::GetInstance()->SetApConfig(info);
            if (ret != smartlink_sdk::RtnCode::E_SUCCESS)
            {
                SMLK_LOGE("SetApConfig failed!, error code[%d]", (SMLK_UINT32)ret);
                result = COM_SET_FAIL;
                goto GeneResp;
            }
            smartlink_sdk::WifiApInfo g_info;
            SMLK_BOOL is_needCheck = SMLK_TRUE;
            SMLK_UINT8 i_num = 0;
            /*wifi密码写入成功校验*/
            while (is_needCheck)
            {
                ret = smartlink_sdk::Wlan::GetInstance()->GetApConfig(g_info);
                if (ret != smartlink_sdk::RtnCode::E_SUCCESS)
                {
                    result = COM_SET_FAIL;
                    SMLK_LOGE("GetApConfig failed! Error code[%d]", (SMLK_UINT32)ret);
                }
                else
                {
                    SMLK_LOGI("[%dth]: Get pw[%s] Set pw[%s]", i_num, g_info.password.c_str(), info.password.c_str());
                    if ((i_num > 10) && (g_info.ssid != info.ssid))
                    {
                        SMLK_LOGW("Over 10 times checking pw fail!");
                        result = COM_SET_FAIL;
                        is_needCheck = SMLK_FALSE;
                    }
                    else if (g_info.ssid == info.ssid)
                    {
                        is_needCheck = SMLK_FALSE;
                    }
                    else
                    {
                        usleep(1000 * 100); // 100ms
                        i_num++;
                    }
                }
            }
            prop_name = SYS_PRO_NAME_TBOX_WIFI_AP_SSID;
            property_rc = SysProperty::GetInstance()->SetValue(prop_name, g_info.ssid);
            if (RtnCode::E_SUCCESS != property_rc)
            {
                result = COM_SET_FAIL;
                SMLK_LOGE("Set [name]=%s with %s failed.", prop_name.c_str(), g_info.ssid.c_str());
            }
            else
            {
                SMLK_LOGD("Set [name]=%s with %s.", prop_name.c_str(), g_info.ssid.c_str());
            }
            prop_name = SYS_PRO_NAME_TBOX_WIFI_AP_PASSWORD;
            property_rc = SysProperty::GetInstance()->SetValue(prop_name, g_info.password);
            if (RtnCode::E_SUCCESS != property_rc)
            {
                result = COM_SET_FAIL;
                SMLK_LOGE("Set [name]=%s with %s failed.", prop_name.c_str(), g_info.password.c_str());
            }
            else
            {
                SMLK_LOGD("Set [name]=%s with %s.", prop_name.c_str(), g_info.password.c_str());
            }
        }
    GeneResp:
        SendMsg0F42(queue.m_head, pcommon->cmd_id, result);
    }
    /*0x06 获取终端平台通讯协议版本--待优化*/
    else if (CMD_PROT_VERSION_GET == pcommon->cmd_id)
    {
        string version("V1.11");
        prop_name = SYS_PRO_NAME_COMMUNICATION_PROT_VERSION;
        property_rc = SysProperty::GetInstance()->GetValue(prop_name, version);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("get [name]=%s fail!", prop_name.c_str());
        }
        SMLK_LOGD("cmd_id=%d, version=%s.", pcommon->cmd_id, version.c_str());
        SendMsg0F42(queue.m_head, pcommon->cmd_id, version);
    }
    /*0X07 获取wifi和ssid密码*/
    else if (CMD_WIFI_SSID_PASSWD_GET == pcommon->cmd_id)
    {
        string ssid_passwd("123456\nhx-test");
        smartlink_sdk::WifiApInfo info;
        smartlink_sdk::RtnCode ret = smartlink_sdk::Wlan::GetInstance()->GetApConfig(info);
        if (smartlink_sdk::RtnCode::E_SUCCESS != ret)
        {
            SMLK_LOGE("GetApConfig failed!, error code[%d]", (SMLK_UINT32)ret);
        }

        SMLK_LOGI("Get wifi ssid[%s]", info.ssid.c_str());
        SMLK_LOGI("Get wifi password[%s]", info.password.c_str());

        int m = info.ssid.length();
        std::string out;
        char buf1[m] = {'0'};

        strncpy(buf1, info.ssid.c_str(), info.ssid.length());
        char buf22[m + 1] = {'\0'};
        SMLK_RC rc = utf8_to_gbk(buf1, m, buf22, m + 1);
        if (rc == SMLK_RC::RC_OK)
        {
            out = buf22;
        }
        ssid_passwd = out + "\n" + info.password;
        SMLK_LOGD("cmd_id=%d, ssid_passwd=%s.", pcommon->cmd_id, ssid_passwd.c_str());
        SendMsg0F42(queue.m_head, pcommon->cmd_id, ssid_passwd);
    }
    /*0x0A 车型配置下发*/
    else if (CMD_VEHICLE_CONFIG_SET == pcommon->cmd_id)
    {
        string engine_config;
        engine_config.copy((char *)(pcommon + 1), 0, length - sizeof(Msg8F42Common));
        SMLK_LOGD("cmd_id=%d, engine_config=%s.", pcommon->cmd_id, engine_config.c_str());
        // setparamtosystem();
    }
    /*0X0B 预见性驾驶license配置*/
    else if (CMD_LICENSE_SET == pcommon->cmd_id)
    {
// todo:等预见性巡航做好了再做
#if 0
        string license;
        SMLK_LOGD( "cmd_id=%d, license=%s.",pcommon->cmd_id,license.c_str());
        if(LICENSE_LENGTH != length-sizeof(Msg8F42Common)){
            result = SMLK_TSP_0001_RESULT_FAILED;
        }else{
            license.insert(license.end(), (char *)(pcommon+1), (char *)(pcommon+1)+LICENSE_LENGTH);
        }
        queue.query_queue.m_query_vec.push_back(REMCTRL_GET_CMD_LICENSE_CHECKSTATUS);
        queue.m_head.msg_id = JTT808_VEHICAL_QUERY_RESP;
        queue.m_cmd_type = RctrlCmdType::REMCTRL_CMD_GET;
        queue.m_cmd_from = VehCtrlCmdFrom::FROM_TSP_MODULE;
        is_encode = GOTO_ENCODE;
#endif
    }
    /*0x0D 排放标准网关设置*/
    else if (CMD_DISCHAR_STD_GW_SET == pcommon->cmd_id)
    {
        vector<SMLK_UINT8> cont;
        string send_switch, ep_ptc_type, ep_ptc_ip, ep_ptc_port, lm_ptc_type, lm_ptc_ip, lm_ptc_port;
        /*解析tsp消息*/
        cont.insert(cont.end(), (char *)(pcommon + 1), (char *)(pcommon + 1) + length - sizeof(Msg8F42Common));
        SMLK_UINT8 *pCont = cont.data();
        send_switch = std::to_string(pCont[0]);
        ep_ptc_type = std::to_string(pCont[1]);
        ep_ptc_ip.insert(ep_ptc_ip.end(), (char *)(pCont + 2), (char *)(pCont + 2) + 16);
        SMLK_UINT16 p_ep_ptc_port = (pCont[18] << 8) | (pCont[19]);
        ep_ptc_port = std::to_string(p_ep_ptc_port);
        lm_ptc_type = std::to_string(pCont[20]);
        lm_ptc_ip.insert(lm_ptc_ip.end(), (char *)(pCont + 21), (char *)(pCont + 21) + 16);
        SMLK_UINT16 p_lm_ptc_port = (pCont[37] << 8) | (pCont[38]);
        lm_ptc_port = std::to_string(p_lm_ptc_port);
        /*组织json,设置参数*/
        Poco::DynamicStruct dySturct;
        dySturct["SendSwitch"] = send_switch.c_str();
        dySturct["EpProtType"] = ep_ptc_type.c_str();
        dySturct["EpProtIp"] = ep_ptc_ip.c_str();
        dySturct["EpProtPort"] = ep_ptc_port.c_str();
        dySturct["LmProtType"] = lm_ptc_type.c_str();
        dySturct["LmProtIp"] = lm_ptc_ip.c_str();
        dySturct["LmProtPort"] = lm_ptc_port.c_str();
        std::string szStdDiscGwSet = dySturct.toString();
        prop_name = SYS_PRO_NAME_STD_DISCHAR_GW_SET;
        property_rc = SysProperty::GetInstance()->SetValue(prop_name, szStdDiscGwSet);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            result = COM_SET_FAIL;
            SMLK_LOGE("Set [name]=%s as %s fail!", prop_name.c_str(), szStdDiscGwSet.c_str());
        }
        else
        {
            SMLK_LOGD("Set [name]=%s with %s successfully", prop_name.c_str(), szStdDiscGwSet.c_str());
        }
        SendMsg0F42(queue.m_head, CMD_DISCHAR_STD_GW_SET, result);
    }
    /*0x0F 国6TBOX是否备案*/
    else if (CMD_GB_REC_STATUS_SET == pcommon->cmd_id)
    {
        SMLK_UINT8 *rec = (SMLK_UINT8 *)pcommon + sizeof(Msg8F42Common) + 1;
        if ((GB_REC_STATUS::GB_REC_CLOSE == (GB_REC_STATUS)*rec) || (GB_REC_STATUS::GB_REC_OPEN == (GB_REC_STATUS)*rec))
        {
            Poco::DynamicStruct dySturct;
            std::string seq_id;
            queue.m_head.seq_id = be16toh(queue.m_head.seq_id);
            seq_id = tools::uint8_2_string((SMLK_UINT8 *)&(queue.m_head.seq_id), sizeof(SMLK_UINT16));
            dySturct["SN"] = seq_id;
            dySturct["RecSta"] = *rec;
            prop_name = SYS_PRO_NAME_GB_RECORDED_STATUS;
            property_rc = SysProperty::GetInstance()->SetValue(prop_name, dySturct.toString());
            if (RtnCode::E_SUCCESS != property_rc)
            {
                SMLK_LOGE("set [name]=%s with %s fail!", prop_name.c_str(), dySturct.toString().c_str());
            }
            else
            {
                SMLK_LOGD("set [name]=%s as %s", prop_name.c_str(), dySturct.toString().c_str());
            }
            /*专有应答在收到对应模块处理结果后上报*/
        }
        else
            return SMLK_RC::RC_ERROR;
    }
    /*0x10 设置加密芯片ID*/
    else if (CMD_ENC_CHIP_ID_SET == pcommon->cmd_id)
    {
        SMLK_UINT8 *chip_id = (SMLK_UINT8 *)pcommon + sizeof(Msg8F42Common) + 1;
        string str_chip_id = std::to_string(*chip_id);
        SMLK_LOGD("str_chip_id=%s", str_chip_id.c_str());
        /*设置参数,回复专有应答*/
        prop_name = SYS_PRO_NAME_ENC_CHIP_ID_SET;
        property_rc = SysProperty::GetInstance()->SetValue(prop_name, str_chip_id);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            result = COM_SET_FAIL;
            SMLK_LOGE("set [name]=%s with %s fail!", prop_name.c_str(), str_chip_id.c_str());
        }
        else
        {
            SMLK_LOGD("set [name]=%s with %s.", prop_name.c_str(), pcommon->cmd_id, str_chip_id.c_str());
        }
        // todo:组织专有应答
        SendMsg0F42(queue.m_head, pcommon->cmd_id, result);
    }
    /*0x11  整车配置修改*/
    else if (CMD_CAR_CONFIG_MODIFY == pcommon->cmd_id)
    {
        SMLK_BOOL rc;
        SMLK_UINT16 did;
        std::vector<SMLK_UINT8> output_vec;
        SMLK_UINT8 did_byte_no, did_set_val;
        if (CONFIG_MODIFY_LEN != length)
        {
            SMLK_LOGW("[0x11] len mismatch!");
            return SMLK_RC::RC_ERROR;
        }
        queue.m_head.seq_id = be16toh(queue.m_head.seq_id);
        indata++;
        did = (((SMLK_UINT16)*indata) << 8) | (SMLK_UINT16) * (++indata);
        SMLK_UINT16 be_did = htobe16(did);
        did_byte_no = *(++indata);
        did_set_val = *(++indata);
        SMLK_LOGD("[0x11 SetDid]={[Did]=0x%04x [ByteNo]=%d [Val]=%d}", did, did_byte_no, did_set_val);
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&queue.m_head.seq_id, (SMLK_UINT8 *)&queue.m_head.seq_id + sizeof(SMLK_UINT16));
        output_vec.emplace_back(CMD_CAR_CONFIG_MODIFY);
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&be_did, (SMLK_UINT8 *)&be_did + sizeof(SMLK_UINT16));
        output_vec.emplace_back(did_byte_no);
        if (((int)VEHCLE_SUPPORT_DID_0100 == did) ||
            ((int)VEHCLE_SUPPORT_DID_0110 == did) ||
            ((int)VEHCLE_SUPPORT_DID_102a == did) ||
            ((int)VEHCLE_SUPPORT_DID_102b == did) ||
            ((int)VEHCLE_SUPPORT_DID_102c == did))
        {
            rc = smartlink::OBD::DidIpcApi::getInstance()->SetDidByteValue(did, did_byte_no, did_set_val);
            if (SMLK_TRUE != rc)
            {
                SMLK_LOGW("[0x11 Set] fail!");
                result = COM_SET_FAIL;
            }
            else
                SMLK_LOGD("[0x11 Set] succ!");
        }
        else
            result = COM_SET_FAIL;
        output_vec.emplace_back(result);
        SMLK_LOGD("OutVec=%s", tools::uint8_2_string(output_vec.data(), output_vec.size()).c_str());
        SendMsg0F42(queue.m_head, output_vec);
    }
    /*0x12  整车配置查询*/
    else if (CMD_CAR_CONFIG_QUERY == pcommon->cmd_id)
    {
        SMLK_BOOL rc;
        SMLK_UINT16 did;
        std::vector<SMLK_UINT8> output_vec, did_vec;
        if (CONFIG_QUERY_LEN != length)
        {
            SMLK_LOGW("[0x12] len mismatch!");
            return SMLK_RC::RC_ERROR;
        }
        queue.m_head.seq_id = be16toh(queue.m_head.seq_id);
        indata++;
        did = (((SMLK_UINT16)*indata) << 8) | (SMLK_UINT16) * (++indata);
        SMLK_UINT16 be_did = htobe16(did);
        SMLK_LOGD("[0x12 QueryDid]={[Did]=0x%04x}", did);
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&queue.m_head.seq_id, (SMLK_UINT8 *)&queue.m_head.seq_id + sizeof(SMLK_UINT16));
        output_vec.emplace_back(CMD_CAR_CONFIG_QUERY);
        output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&be_did, (SMLK_UINT8 *)&be_did + sizeof(SMLK_UINT16));
        if (((int)VEHCLE_SUPPORT_DID_0100 == did) ||
            ((int)VEHCLE_SUPPORT_DID_0110 == did) ||
            ((int)VEHCLE_SUPPORT_DID_102a == did) ||
            ((int)VEHCLE_SUPPORT_DID_102b == did) ||
            ((int)VEHCLE_SUPPORT_DID_102c == did))
        {
            rc = smartlink::OBD::DidIpcApi::getInstance()->GetDidWholeData(did, did_vec);
            if (SMLK_TRUE != rc)
            {
                SMLK_LOGW("[0x12 Query] fail!");
                result = COM_SET_FAIL;
            }
            else
            {
                output_vec.emplace_back(did_vec.size());
                output_vec.insert(output_vec.end(), did_vec.data(), did_vec.data() + did_vec.size());
                SMLK_LOGD("[0x12 QueryMsg]=%s", tools::uint8_2_string(did_vec.data(), did_vec.size()).c_str());
            }
        }
        else
            result = COM_SET_FAIL;
        if (COM_SET_FAIL == result)
            output_vec.emplace_back(0x00);
        SMLK_LOGD("OutVec=%s", tools::uint8_2_string(output_vec.data(), output_vec.size()).c_str());
        SendMsg0F42(queue.m_head, output_vec);
    }
    /*0x13 获取发动机VIN码*/
    else if (CMD_ENGINE_VIN_GET == pcommon->cmd_id)
    {
        /*目前获取发动机VIN码直接从Property中读取并透传,国6模块会对参数进行配置,上电后默认写17位0,10s后读取VIN码并写入*/
        std::string engin_vin;
        prop_name = SYS_PRO_NAME_OBD_VIN;
        property_rc = SysProperty::GetInstance()->GetValue(prop_name, engin_vin);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("get [name]=%s fail!", prop_name.c_str());
            engin_vin = "00000000000000000"; /*若发动机VIN码读取失败,上报全0*/
        }
        else
        {
            SMLK_LOGD("get [name]=%s with %s", prop_name.c_str(), engin_vin.c_str());
        }
        SendMsg0F42(queue.m_head, pcommon->cmd_id, engin_vin);
    }
    /*0x14 获取仓储模式*/
    else if (CMD_STORAGE_MODE_GET == pcommon->cmd_id)
    {
        string storage;
        SMLK_UINT8 nStorage;
        prop_name = SYS_PRO_NAME_DID_STORAGE_MODE;
        property_rc = SysProperty::GetInstance()->GetValue(prop_name, storage);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("get [name]=%s fail!", prop_name.c_str());
            nStorage = 1; //当仓储模式参数未配置导致查询失败时，默认回复1:仓储模式
        }
        else
        {
            SMLK_LOGD("cmd_id=0x%02x, get storage=%s.", pcommon->cmd_id, storage.c_str());
            if (storage == "0")
            {
                nStorage = 0;
            }
            else
                nStorage = 1;
        }
        SMLK_LOGD("Get storage mode=%s", (nStorage == 0) ? "Normal" : "Repository");
        SendMsg0F42(queue.m_head, pcommon->cmd_id, nStorage);
    }
    /*0x15 设置仓储模式*/
    else if (CMD_STORAGE_MODE_SET == pcommon->cmd_id)
    {
        vector<SMLK_UINT8> cont;
        cont.insert(cont.end(), (char *)(pcommon + 1), (char *)(pcommon + 1) + length - sizeof(Msg8F42Common));
        SMLK_UINT8 *pCont = cont.data();
        string storage;
        /*判断当前存储模式，如果是正常模式，则不允许设置为仓储模式*/
        prop_name = SYS_PRO_NAME_DID_STORAGE_MODE;
        property_rc = SysProperty::GetInstance()->GetValue(prop_name, storage);
        if (RtnCode::E_SUCCESS != property_rc)
        {
            SMLK_LOGE("get [name]=%s fail!", prop_name.c_str());
            goto SET_STORAGE;
        }
        else
        {
            SMLK_LOGD("cmd_id=0x%02x, get storage=%s.", pcommon->cmd_id, storage.c_str());
            if (storage == "1")
            {
                goto SET_STORAGE;
            }
            else
            {
                if (pCont[0] == 1)
                {
                    SMLK_LOGW("Can't switch storage mode from normal to repository!");
                    result = COM_SET_FAIL;
                    goto SEND_RESP;
                }
                else
                    goto SET_STORAGE;
            }
        }
    SET_STORAGE:
        property_rc = SysProperty::GetInstance()->SetValue(prop_name, std::to_string(pCont[0]));
        if (RtnCode::E_SUCCESS != property_rc)
        {
            result = COM_SET_FAIL;
            SMLK_LOGE("set [name]=%s with %s fail!", prop_name.c_str(), std::to_string(pCont[0]).c_str());
        }
        else
        {
            SMLK_LOGD("Set [name]=%s [storage]=%s successfully", prop_name.c_str(), std::to_string(pCont[0]).c_str());
        }
    SEND_RESP:
        SendMsg0F42(queue.m_head, pcommon->cmd_id, result);
    }
    /*0x19 仪表盘保养提醒信息设置*/
    else if (CMD_DB_MTN_RMD_SET == pcommon->cmd_id)
    {
/*V卡版*/
#ifdef FAW_MICRO_TRUCK
        Process_8F42_0x19(indata, length, result);
        SendMsg0F42(queue.m_head, pcommon->cmd_id, result);
#endif
/*基础版*/
#ifdef FAW_TRUCK
        std::string prod_loc;
        prop_name = SYS_PRO_NAME_PRODUCT_LOC;
        RtnCode rc = SysProperty::GetInstance()->GetValue(prop_name, prod_loc);
        SMLK_LOGD("Cur production location is:%s", (prod_loc == PRO_LOC_QINGDAO) ? "QingDao" : "ChangChun");
        /*如果当前的产地为青岛*/
        if ((RtnCode::E_SUCCESS == rc) && (PRO_LOC_QINGDAO == prod_loc))
        {
            Process_8F42_0x19(indata, length, result);
            SendMsg0F42(queue.m_head, pcommon->cmd_id, result);
        }
#endif
    }
    is_encode = NOT_GOTO_ENCODE;
    return SMLK_RC::RC_OK;
}

void DissectorMsg8F42::Process_8F42_0x19(IN SMLK_UINT8 *indata, IN size_t length, INOUT SMLK_UINT8 &result)
{
    SMLK_BOOL rc;
    if (m_vec_cb)
    {
        rc = m_vec_cb(VecCtrlInnerEventID::VEC_CTRL_EVENT_DASH_BOARD_MTN, (void *)indata, length);
    }
    else
    {
        SMLK_LOGE("Process_8F42_0x19 CallBack doesn't exist!");
        return;
    }
    SMLK_LOGD("get cb=%d", rc);
    if (rc != SMLK_TRUE)
        result = COM_SET_FAIL;
    return;
}

/**
 * @brief               0F42消息组包回复
 * @param head
 * @param length        8F42下发指令长度
 * @param queue         0F42指令回复队列
 * @param is_encode     是否还有消贷业务需要处理
 *
 * @return
 */
SMLK_RC DissectorMsg8F42::SendMsg0F42(IN RctrlHead &head, IN SMLK_UINT8 &cmd_id, IN SMLK_UINT8 &result)
{
    vector<SMLK_UINT8> output_vec;
    Msg0F42SetCommon setcommon;
    setcommon.seq_id = htobe16(head.seq_id);
    setcommon.cmd_id = cmd_id;
    setcommon.length = htobe16(sizeof(SMLK_UINT8));
    setcommon.result = result;
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&setcommon, (SMLK_UINT8 *)&setcommon + sizeof(Msg0F42SetCommon));

    RctrlHead head_temp;
    memcpy(&head_temp, &head, sizeof(RctrlHead));
    head_temp.msg_id = JTT808_GET_SET_TBOX_RESP;
    head_temp.qos = QOS_SEND_TCP_TIMES;
    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    return SMLK_RC::RC_OK;
}

SMLK_RC DissectorMsg8F42::SendMsg0F42(IN RctrlHead &head, IN SMLK_UINT8 &cmd_id, IN string &str)
{
    vector<SMLK_UINT8> output_vec;
    Msg0F42GetCommon getcommon;
    getcommon.seq_id = htobe16(head.seq_id);
    getcommon.length = htobe16(str.size() + 1); // 0f42回复字符串以\0结尾
    getcommon.cmd_id = cmd_id;
    std::string tem_str = str;
    tem_str.push_back('\0');
    output_vec.insert(output_vec.end(), (SMLK_UINT8 *)&getcommon, (SMLK_UINT8 *)&getcommon + sizeof(Msg0F42GetCommon)); //组入包头
    output_vec.insert(output_vec.end(), tem_str.begin(), tem_str.end());                                                // 0f42回复字符串以\0结尾
    RctrlHead head_temp;
    memcpy(&head_temp, &head, sizeof(RctrlHead));
    head_temp.msg_id = JTT808_GET_SET_TBOX_RESP;
    head_temp.qos = QOS_SEND_TCP_TIMES;
    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    return SMLK_RC::RC_OK;
}

SMLK_RC DissectorMsg8F42::SendMsg0F42(IN RctrlHead &head, IN std::vector<SMLK_UINT8> &msg)
{
    std::vector<SMLK_UINT8> output_vec;
    output_vec.insert(output_vec.end(), msg.data(), msg.data() + msg.size());
    RctrlHead head_temp;
    memcpy(&head_temp, &head, sizeof(RctrlHead));
    head_temp.msg_id = JTT808_GET_SET_TBOX_RESP;
    head_temp.qos = QOS_SEND_TCP_TIMES;
    DissectorJtt808Common::getInstance()->DoSendMsgToTsp(head_temp, (SMLK_UINT8 *)(output_vec.data()), output_vec.size());
    return SMLK_RC::RC_OK;
}

DissectorMsg8F42::DissectorMsg8F42()
{
}

DissectorMsg8F42::~DissectorMsg8F42()
{
}

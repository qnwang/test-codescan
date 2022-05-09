/*
 * @Author: T
 * @Date: 2022-04-19 20:39:27
 * @LastEditTime: 2022-05-06 17:58:01
 * @LastEditors: taoguanjie taoguanjie@smartlink.com.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /Workspace/tbox2.0/source/apps/gb6/src/common_handler.cpp
 */

#include <smlk_log.h>
#include <smlk_tools.h>
#include <smlk_types.h>

#include "Poco/Event.h"
#include "Poco/Dynamic/Struct.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/Dynamic/Pair.h"
#include "Poco/Dynamic/VarIterator.h"
#include "Poco/JSON/Array.h"
#include "Poco/JSON/Parser.h"

#include <message_def_17691.h>
#include <smartlink_sdk_sys_property.h>
#include <smartlink_sdk_sys_property_def.h>

#include "result_handler.h"

using namespace smartlink;

using namespace Poco;
using namespace Poco::Dynamic;

resulthandler *resulthandler::m_instance = NULL;
std::mutex resulthandler::m_mtx;

/**
* @brief    构造
*/
resulthandler::resulthandler()
: m_808_sn("")
, m_ent_conn_state(M_ENT_PLATFORM_CONNECT_NORMAL)
, m_loc_conn_state(M_LOC_PLATFORM_CONNECT_NORMAL)
, m_808_act_seq("")
, m_808_act_state(M_HJ_1239_ACTIVATED_RESERVE)
{
}

resulthandler::~resulthandler(){
}

/**
* @brief    单例
*/
resulthandler *resulthandler::getInstance() {
    if (m_instance == NULL) {
        m_mtx.lock();
        if (m_instance == NULL)
            m_instance = new resulthandler();
        m_mtx.unlock();
    }

    return m_instance;
}

void resulthandler::setGB6VinStatus(SMLK_UINT8 state)
{
    SMLK_LOGI("gb6 vin status %d", state);
    handleDTCErrCode(state);

}

/**
 * @brief 设置平台连接状态
 * 
 * @param state 
 * @param platform 
 */
void resulthandler::setPlatformConnStatus(SMLK_UINT8 state, SMLK_UINT8 platform)
{
    SMLK_LOGI("platform %d new state %d, last ent state %d, last loc state %d", platform, state, m_ent_conn_state, m_loc_conn_state);
    if (platform == M_PLATFORM_BUSINESS)
    {
        if (m_ent_conn_state != state)
        {
            m_ent_conn_state = state;
            handleDTCErrCode(m_ent_conn_state);
        }
    }
    else if (platform == M_PLATFORM_NATIONAL)
    {
        if (m_loc_conn_state != state)
        {
            m_loc_conn_state = state;
            handleDTCErrCode(m_loc_conn_state);
        }
    }
    else
    {
        SMLK_LOGI("unsupport platform %d", platform);
    }
}

/**
 * @brief 处理 dtc 故障码
 * 
 * @param state 
 * @return SMLK_UINT32 
 */
void resulthandler::handleDTCErrCode(SMLK_UINT8 errcode)
{
    OBD::UdsDtcTriggerInfo dtc_info;
    // DTC故障代码
    dtc_info.val = errcode;
    // dtc_info.dtc_type = OBD::UdsDtcTriggerType::E_UDS_DTC_TRIGGER_CHECK_VIN_STATUS;

    dtc_info.dtc_type = OBD::UdsDtcTriggerType::E_UDS_DTC_TRIGGER_GB_CONNECT_STATUS;
    if (!(OBD::DidIpcApi::getInstance()->SendDtcErrCode(dtc_info)))
    {
        SMLK_LOGW("SyncDTC ipc failed!!!");
    }
    else
    {
        SMLK_LOGD("handleDTCErrCode dtc_info.val=%d,dtc_info.dtc_type=%d", dtc_info.val, dtc_info.dtc_type);
    }
}


/**
 * @brief 
 * 
 * @param seq 
 */
void resulthandler::setHJActiveCMDSeq(std::string &seq)
{
    m_808_act_seq = seq;
}

/**
 * @brief 
 * 
 * @return std::string 
 */
std::string resulthandler::getHJActiveCMDSeq()
{
    return m_808_act_seq;
}

/**
 * @brief 
 * 
 * @param state 
 * @param save 
 */
void resulthandler::setHJActiveResult(SMLK_UINT8 state, SMLK_UINT8 save)
{
    SMLK_LOGI("new state %d, last state %d, save %d", state, m_808_act_state, save);
    if (m_808_act_state != state)
    {
        m_808_act_state = state;
        if (save)
        {
            std::string property = SYS_PRO_NAME_HJ_ACTIVATED_STATUS;
            smartlink_sdk::RtnCode return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(property, std::to_string(m_808_act_state));
            SMLK_LOGI("Set %s %d ", SYS_PRO_NAME_HJ_ACTIVATED_STATUS, m_808_act_state);
            if (smartlink_sdk::RtnCode::E_SUCCESS != return_code)
            {
                SMLK_LOGE("fail to set system property \"%s\", return code: %d", SYS_PRO_NAME_HJ_ACTIVATED_STATUS, static_cast<std::int32_t>(return_code));
            }
        }
    }
}

/**
 * @brief 
 * 
 * @return SMLK_UINT8 
 */
SMLK_UINT8 resulthandler::getHJActiveResult()
{
    SMLK_LOGI(" get state %d ", m_808_act_state);
    return m_808_act_state;
}

/**
 * @brief 处理激活结果
 * 
 * @param state 
 * @return SMLK_UINT32 
 */
void resulthandler::handleHJActiveResult(SMLK_UINT8 state) {
    if (state == m_808_act_state) {
        SMLK_LOGI(" same activated result %d !!!", state);
    } else {
        DynamicStruct dySturct;

        dySturct["SN"] = m_808_sn.c_str();

        dySturct["RecSta"] = "2";

        dySturct["RecRes"] = "4";

        Var v(dySturct);
        std::string stdDiscIpSet = v.convert<std::string>();
        auto return_code = smartlink_sdk::SysProperty::GetInstance()->SetValue(SYS_PRO_NAME_GB_RECORDED_STATUS, stdDiscIpSet);
        if (smartlink_sdk::RtnCode::E_SUCCESS != return_code) {
            SMLK_LOGE("Set name=%s as %s fail!", SYS_PRO_NAME_STD_DISCHAR_GW_SET, stdDiscIpSet.c_str());
        }
    }

}
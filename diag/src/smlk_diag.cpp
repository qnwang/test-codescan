#include <iostream>
#include <sstream>
#include <iomanip>
#include <smlk_log.h>

#include "smlk_types.h"
#include "tsp_service_api.h"

#include "smlk_common.h"

#include "smartlink_sdk_tel.h"

#include "smartlink_sdk_wlan.h"
#include "smartlink_sdk_sys_property.h"
#include "smartlink_sdk_sys_property_def.h"
#include "smartlink_sdk_rtn_code.h"
#include "smlk_diag.h"
#include "smlk_spi_manager.h"

#include "smlk_tsp_jtt808.h"

using namespace smartlink::CAN;
using namespace smartlink;

SmlkDiag* SmlkDiag::m_instance = NULL;

/*诊修相关参数*/
std::string SmlkDiag::m_sn;
std::string SmlkDiag::m_imsi;
std::string SmlkDiag::m_iccid;
std::string SmlkDiag::m_vin;
std::string SmlkDiag::m_ein;

bool SmlkDiag::m_ign_state = false;
bool SmlkDiag::m_can_state = false;

smartlink_sdk::LocationInfo SmlkDiag::m_location;

/*处理wifi对接业务*/
std::string SmlkDiag::m_wifi_ssid;
std::string SmlkDiag::m_wifi_key;
std::thread SmlkDiag::m_wifi_connect_thread;
bool SmlkDiag::m_wifi_connected = false;
uint8_t SmlkDiag::m_wifi_timeout = 10;

/*UDS服务锁*/
std::mutex SmlkDiag::g_uds_mutex;     // 用到的全局锁
smartlink::OBD::UdsDiagConfig  SmlkDiag::m_uds_status;

/*TSP状态管理*/
smartlink::TspConnectState SmlkDiag::m_tsp_connect_state = TspConnectState::E_INVALID;
smartlink::TspLoginState SmlkDiag::m_tsp_login_state = TspLoginState::E_INVALID;

/*诊修SDK索引*/
tboxInterface_t * SmlkDiag::m_sdk_instance = NULL;
std::set<uint32_t>  SmlkDiag::m_can_filter_list;

bool SmlkDiag::m_can_send_enable = true;

bool m_uds_lock = true;/*默认锁定，防止误发*/

SmlkDiag::SmlkDiag()
{
    m_sn.clear();
    m_imsi.clear();
    m_iccid.clear();
    m_vin.clear();
    m_ein.clear();

    /*处理wifi对接业务*/
    m_wifi_ssid.clear();
    m_wifi_key.clear();

    memset(&m_uds_status,0,sizeof(m_uds_status));
}

SmlkDiag::~SmlkDiag()
{
    if(m_instance != NULL)
    {
        delete m_instance;
    };
};

SmlkDiag* SmlkDiag::GetInstance()
{
    if(m_instance == NULL)
    {
        m_instance = new SmlkDiag();
    }
    else
    {
        return m_instance;
    }
};

int SmlkDiag::Init()
{
    /*初始化参数配置服务*/
    InitPro();

    /*初始化MCU配置服务*/
    InitMcu();

    /*初始化定位信息服务*/
    InitLoc();

    /*初始化4G服务*/
    InitTel();

    /*初始化后台服务*/
    InitTsp();

    /*初始化4G服务*/
    InitWlan();

    /*初始化4G服务*/
    InitODB();

    return 0;
};

int SmlkDiag::InitODB()
{
   // SmlkUds::getInstance()->Init(ModuleID::E_Module_diag_service, SmlkUdsMode::UDS_DIAG_MODE_DIAG_OTHER);

    /*注册仲裁模块*/
    if (smartlink::SMLK_RC::RC_OK != smartlink::OBD::DidIpcApi::getInstance()->Init(ModuleID::E_Module_diag_service) )
    {
        SMLK_LOGE("fail to init system property module");
        return -1;
    }

    /*获取当前仲裁配置*/
    smartlink::OBD::DidIpcApi::getInstance()->GetUdsDiagMode(m_uds_status);

    /*仲裁优先级变化事件*/
    std::vector<smartlink::OBD::OBDEventId> events = {
        smartlink::OBD::OBDEventId::E_OBD_EVENT_UDS_MODE_CHANGED
    };
    
    auto rtn = smartlink::OBD::DidIpcApi::getInstance()->RegEventCB(
        events,
        [&](smartlink::OBD::OBDEventId id, void *data, int len)
        {
            if (id == smartlink::OBD::OBDEventId::E_OBD_EVENT_UDS_MODE_CHANGED)
            {
                if (len != sizeof(smartlink::OBD::UdsDiagConfig)) 
                {
                    return;
                }
                smartlink::OBD::UdsDiagConfig* uds_diag = (smartlink::OBD::UdsDiagConfig*)data;
                /*仲裁模式改变*/
                if(uds_diag->mode != m_uds_status.mode)
                {
                    //std::lock_guard<std::mutex> lock(g_uds_mutex);     
                    memcpy((void*)&m_uds_status,uds_diag,sizeof(smartlink::OBD::UdsDiagConfig));
                    SMLK_LOGI("uds mode changed to [0x%02x], pri:%x", m_uds_status.mode,m_uds_status.pri);
                    /*仲裁优先级大于诊修，则需要通知诊修SDK*/
                    if((m_uds_status.mode != smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_INVALID)/*有仲裁占用*/
                    && (m_uds_status.mode != smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_DIAG_WRITE)
                    && (m_uds_status.mode != smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_DIAG_OTHER)
                    && (m_uds_status.pri < smartlink::OBD::UdsDiagPriority::E_UDS_DIAG_PRI_DIAG_OTHER))
                    {
                        SMLK_LOGI("uds mode changed to sdk, pri:%x", m_uds_status.mode,m_uds_status.pri);
                        tboxCan1Can2RcvMsg_t pOutCanRxMsg = {0};
                           /*扩展位置处理*/
                        //pOutCanRxMsg.canRcvMsg.IDE = uds_rsp->req_id>>31;
                        pOutCanRxMsg.canRcvMsg.CanId =  0x18DB33F1;
                        //   pOutCanRxMsg.canRcvMsg.timeStamp = frame->utc;
                        pOutCanRxMsg.Rc1Rc2flag = 0x62;
                        pOutCanRxMsg.canRcvMsg.RDA.bytes.byte4 = 0x02;
                        pOutCanRxMsg.canRcvMsg.RDA.bytes.byte3 = 0x3E;
                        pOutCanRxMsg.canRcvMsg.RDA.bytes.byte2 = (uint8_t)m_uds_status.pri;/*响应仲裁优先级*/
                        pOutCanRxMsg.canRcvMsg.RDA.bytes.byte1 = 0;
                        pOutCanRxMsg.canRcvMsg.RDB.bytes.byte4 = 0;
                        pOutCanRxMsg.canRcvMsg.RDB.bytes.byte3 = 0;
                        pOutCanRxMsg.canRcvMsg.RDB.bytes.byte2 = 0;
                        pOutCanRxMsg.canRcvMsg.RDB.bytes.byte1 = 0;
                        if( NULL != m_sdk_instance )
                        {
                            m_sdk_instance->rcv_irq_can_message(&pOutCanRxMsg);
                        }
                    }
                } 
            }
    });
    return 0;
}

int SmlkDiag::Start()
{
    /*获取瑞修德的sdk索引*/
    m_sdk_instance = SdkInit();
    /*启动智能诊修模式*/
    std::thread work(tboxInterface_main);
    work.detach();
}

int SmlkDiag::Stop()
{
    
}

int SmlkDiag::InitPro()
{
    if (!smartlink_sdk::SysProperty::GetInstance()->Init(ModuleID::E_Module_diag_service) )
    {
        SMLK_LOGE("fail to init system property module");
        return -1;
    }
    std::string name_str = SYS_PRO_NAME_DID_VIN;
    if( smartlink_sdk::RtnCode::E_SUCCESS != SysProperty::GetInstance()->GetValue(name_str ,m_vin))
    {
        SMLK_LOGE("fail to get system property \"%s\"", name_str.c_str());
    }
    else
    {
        SMLK_LOGI("get property vin [%s]",m_vin.c_str());
    }
    return 0;
};

int SmlkDiag::InitMcu()
{
    auto return_code = smartlink_sdk::MCU::GetInstance()->Init(ModuleID::E_Module_diag_service);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to init MCU service, return code: %d", static_cast<std::int32_t>(return_code));
        return -1;
    }
    std::vector<smartlink_sdk::McuEventId> mcu_events = {
        smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE,
        smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_STATE,
        smartlink_sdk::McuEventId::E_MCU_EVENT_UDS_ASYNC_RESPONSE
    };
    return_code = smartlink_sdk::MCU::GetInstance()->RegEventCB(
        mcu_events,
        [](smartlink_sdk::McuEventId id, void *data, int len)
        {
            switch(id)
            {
                case smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE:
                {
                    if (len != sizeof(smartlink_sdk::IGNState))
                    {
                        SMLK_LOGE("ign state lenth[%d] error, expeceted lenth[%d]", len, sizeof(smartlink_sdk::IGNState));
                    }
                    smartlink_sdk::IGNState* ign = reinterpret_cast<smartlink_sdk::IGNState*>(data);

                    /*IGN 由ON切换到OFF*/
                    if(m_ign_state && !ign->on)
                    {
                        /*恢复tbox热点模式*/
                        smartlink_sdk::WifiMode mode;
                        smartlink_sdk::RtnCode ret = smartlink_sdk::Wlan::GetInstance()->GetMode(mode);
                        if(smartlink_sdk::WifiMode::E_WIFI_MODE_STA == mode)
                        {
                            smartlink_sdk::Wlan::GetInstance()->SetMode(WifiMode::E_WIFI_MODE_AP);
                            SMLK_LOGI("set wifi mode AP");
                        }
                    }
                    /*更新IGN状态*/
                    m_ign_state = ign->on?true:false;
                    break;
                }

                case smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_STATE:
                {
                    if (len != sizeof(smartlink_sdk::CANState))
                    {
                        SMLK_LOGE("can state lenth[%d] error, expeceted lenth[%d]", len, sizeof(smartlink_sdk::CANState));
                    }
                    m_can_state = (smartlink_sdk::CANState::E_CAN_BUS_STATE_NORMAL == *(reinterpret_cast<smartlink_sdk::CANState *>(data)))?true:false;
                    break;
                }
                case smartlink_sdk::McuEventId::E_MCU_EVENT_UDS_ASYNC_RESPONSE:
                {
                    UdsRspAsync *uds_rsp = (UdsRspAsync *)data;   /*len+service id+data*/
                    // printf("uds rev:  request_id=0x%08x, response_id=0x%08x,service_id=0x%08x,result=0x%02x",uds_rsp->req_id,
                    //                                                                         uds_rsp->rsp_id,
                    //                                                                         uds_rsp->svc_id,
                    //                                              uds_rsp->result);
                 
                    m_can_send_enable = true;
                    /*响应超时*/
                    if(uds_rsp->result == 0x1)
                    {
                        break;
                    }

                    int uds_left_len = uds_rsp->len;
                    int uds_deal_len = 0;

                    std::vector<SMLK_UINT8> can_data;
                    tboxCan1Can2RcvMsg_t pOutCanRxMsg = {0};

                    if((uds_rsp->data[0] == 0x7F) && (uds_rsp->len!=1) )/*否定应答*/
                    {
                        can_data.push_back(uds_left_len);
                        for( ;uds_deal_len< uds_left_len;uds_deal_len++)
                        {
                            can_data.push_back(uds_rsp->data[uds_deal_len]);
                        }
                        //补齐8字节
                        for(int i = uds_deal_len;i< 8;i++)
                        {
                            can_data.push_back(0);
                        }
                        /*扩展位置处理*/
                        pOutCanRxMsg.canRcvMsg.IDE = uds_rsp->req_id>>31;
                        pOutCanRxMsg.canRcvMsg.CanId =  uds_rsp->rsp_id&0x7FFFFFFF;
                        //   pOutCanRxMsg.canRcvMsg.timeStamp = frame->utc;
                        pOutCanRxMsg.Rc1Rc2flag = 0x62;
                        pOutCanRxMsg.canRcvMsg.RDA.bytes.byte4 = can_data[0];
                        pOutCanRxMsg.canRcvMsg.RDA.bytes.byte3 = can_data[1];
                        pOutCanRxMsg.canRcvMsg.RDA.bytes.byte2 = can_data[2];
                        pOutCanRxMsg.canRcvMsg.RDA.bytes.byte1 = can_data[3];
                        pOutCanRxMsg.canRcvMsg.RDB.bytes.byte4 = can_data[4];
                        pOutCanRxMsg.canRcvMsg.RDB.bytes.byte3 = can_data[5];
                        pOutCanRxMsg.canRcvMsg.RDB.bytes.byte2 = can_data[6];
                        pOutCanRxMsg.canRcvMsg.RDB.bytes.byte1 = can_data[7];
                        if( NULL != m_sdk_instance )
                        {
                            m_sdk_instance->rcv_irq_can_message(&pOutCanRxMsg);
                        }
                        {//加一对大括号是可以让tmp退出{}的时候自动析构
                            std::vector<SMLK_UINT8> tmp;
                            can_data.swap(tmp);/*清除空间*/
                        }
                    }
                    else//肯定应答
                    {
                        bool first_frame;
                        bool single_frame;
                        uint8_t  multi_index;
                        /*单帧处理*/
                        if(uds_left_len<=6 )
                        {
                            single_frame = true;
                        }
                        else
                        {
                            /*多帧处理*/
                            single_frame = false;
                            first_frame = true;
                            multi_index = 0x21;
                        }
                        do
                        {
                            if(single_frame)/*处理单帧*/
                            {
                                can_data.push_back(uds_left_len+1);
                                can_data.push_back(uds_rsp->svc_id);
                                for(;uds_deal_len< uds_left_len;uds_deal_len++)
                                {
                                    can_data.push_back(uds_rsp->data[uds_deal_len]);
                                }
                                //补齐8字节
                                for(int i = uds_deal_len;i< 8;i++)
                                {
                                    can_data.push_back(0);
                                }
                                uds_left_len = 0;
                            }
                            else
                            {
                                if(first_frame) //首帧
                                {
                                    can_data.push_back(0x10);
                                    can_data.push_back(uds_left_len+1);
                                    can_data.push_back(uds_rsp->svc_id);
                                    for(;uds_deal_len< 5;uds_deal_len++,uds_left_len--)
                                    {
                                        can_data.push_back(uds_rsp->data[uds_deal_len]);
                                    }
                                    first_frame = false;
                                    multi_index = 0x21;
                                }
                                else//后续帧s
                                {
                                    if(uds_left_len>7)
                                    {
                                        can_data.push_back(multi_index);
                                        for(int i=0;i< 7;i++,uds_deal_len++,uds_left_len--)
                                        {
                                            can_data.push_back(uds_rsp->data[uds_deal_len]);
                                        }
                                        /*已经达到0x2f,从0x20开始*/
                                        multi_index = (multi_index == 0x2f)?0x20:(multi_index+1);
                                    }
                                    else/*无后续帧*/
                                    {
                                        /*继续前面的index累计*/
                                        can_data.push_back(multi_index);
                                        for(int m=0;m< uds_left_len;m++,uds_deal_len++)
                                        {
                                            can_data.push_back(uds_rsp->data[uds_deal_len]);
                                        }
                                        uds_left_len = 0;
                                    }
                                }
                            }
                            /*扩展位置处理*/
                            pOutCanRxMsg.canRcvMsg.IDE = uds_rsp->req_id>>31;
                            pOutCanRxMsg.canRcvMsg.CanId =  uds_rsp->rsp_id&0x7FFFFFFF;;
                            //   pOutCanRxMsg.canRcvMsg.timeStamp = frame->utc;
                            pOutCanRxMsg.Rc1Rc2flag = 0x62;
                            pOutCanRxMsg.canRcvMsg.RDA.bytes.byte4 = can_data[0];
                            pOutCanRxMsg.canRcvMsg.RDA.bytes.byte3 = can_data[1];
                            pOutCanRxMsg.canRcvMsg.RDA.bytes.byte2 = can_data[2];
                            pOutCanRxMsg.canRcvMsg.RDA.bytes.byte1 = can_data[3];
                            pOutCanRxMsg.canRcvMsg.RDB.bytes.byte4 = can_data[4];
                            pOutCanRxMsg.canRcvMsg.RDB.bytes.byte3 = can_data[5];
                            pOutCanRxMsg.canRcvMsg.RDB.bytes.byte2 = can_data[6];
                            pOutCanRxMsg.canRcvMsg.RDB.bytes.byte1 = can_data[7];

                            if( NULL != m_sdk_instance )
                            {
                                  
                                // printf( "recv EMS CAN ID %x:",pOutCanRxMsg.canRcvMsg.CanId);
                                // for(int i = 0;i<8 ;i++ )
                                // {
                                //     printf( " %x",can_data[i]);
                                // }
                                // printf( "\r\n");
                         
                                m_sdk_instance->rcv_irq_can_message(&pOutCanRxMsg);
                            }
                            {//加一对大括号是可以让tmp退出{}的时候自动析构
                                std::vector<SMLK_UINT8> tmp;
                                can_data.swap(tmp);/*清除空间*/
                            }
                        }while(uds_left_len>0);
                    }
                    break;
                }
                default:
                    break;
            } // end for switch
    }
    );
};

int SmlkDiag::InitLoc()
{
    if ( !smartlink_sdk::Location::GetInstance()->Init(ModuleID::E_Module_diag_service) ) {
        SMLK_LOGE("fail to init location service API interface!!!");
        return -1;
    }
    std::vector<smartlink_sdk::LocationEventId> loc_events = {
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_ISREADY,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_EXIT,
        smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED,
    };
    smartlink_sdk::Location::GetInstance()->RegEventCallback(
        loc_events,
        [](smartlink_sdk::LocationEventId id, void *data, int len) {
            switch ( id ) {
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_ISREADY:

                    break;
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_EXIT:

                    break;
                case smartlink_sdk::LocationEventId::E_LOCATION_EVENT_FIX_INFO_CHANGED:
                    {
                        if ( sizeof(smartlink_sdk::LocationInfo) != (std::size_t)len ) {
                            SMLK_LOGE("[GNSS] unexpected body length for FIX INFO CHNAGED notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::LocationInfo));
                            return;
                        }
                        /*更新本地位置信息*/
                        memcpy(&m_location,data,len);
                    }
                    break;
                default:
                    SMLK_LOGE("unsupported event id: %u", static_cast<SMLK_UINT32>(id));
                    return;
            }
        }
    );
    smartlink_sdk::Location::GetInstance()-> GetLocation(m_location);
};

int SmlkDiag::InitTel()
{
    if(!smartlink_sdk::Telephony::GetInstance()->Init(ModuleID::E_Module_diag_service)){
        SMLK_LOGF("fail to init Telephony service API interface.");
        return -1;
    }
    std::string iccid;
    smartlink_sdk::Telephony::GetInstance()->GetICCID(iccid);
    m_iccid = iccid;
    SMLK_LOGI("Init iccid : %s",m_iccid.c_str());

    std::string imsi;
    smartlink_sdk::Telephony::GetInstance()->GetIMSI(imsi);
    //imsi = "460097047777186";
    m_imsi = imsi;
    SMLK_LOGI("Init imsi : %s",m_imsi.c_str());

    m_sn = m_imsi.substr(m_imsi.length()-11,m_imsi.length());
    SMLK_LOGI("Init SN : %s",m_sn.c_str());
};

int SmlkDiag::InitTsp()
{
    SMLK_RC tsp_rc =TspServiceApi::getInstance()->Init(ModuleID::E_Module_diag_service);
    if( SMLK_RC::RC_OK != tsp_rc ){
        SMLK_LOGW("TspServiceApi  Init failed ...\n");
        return -1;
    }
    SMLK_LOGI("TspServiceApi  Init success ...\n");
    printf("TspServiceApi  Init success ...\n");

        /*获取tsp连接及登录状态*/
    TspServiceApi::getInstance()->GetConnectState(m_tsp_connect_state);
    printf("m_tsp_connect_state = %d...\n");

    TspServiceApi::getInstance()->GetLoginState(m_tsp_login_state);
    printf("m_tsp_login_state = %d ...\n");

    /*注册tsp登录状态*/
    std::vector<TspEventId> tsp_events = {
        TspEventId::E_TSP_EVENT_CONNECT_STATE,
        TspEventId::E_TSP_EVENT_LOGIN_STATE,
    };

    SMLK_LOGD("register TSP event callback");
    auto rc = TspServiceApi::getInstance()->RegEventCB(
        tsp_events,
        [this](TspEventId id, void *data, int len){
             SMLK_LOGI("revceive tsp event %d",id);
            switch(id)
            {
                case TspEventId::E_TSP_EVENT_CONNECT_STATE:
                {
                    TspConnectState* state =  (TspConnectState*)data;
                    m_tsp_connect_state = *state;
                    SMLK_LOGI("revceive tsp connected state %d",m_tsp_connect_state);
                    break;
                }
                case TspEventId::E_TSP_EVENT_LOGIN_STATE:
                {
                    TspLoginState* state =  (TspLoginState*)data;
                    m_tsp_login_state = *state;
                    SMLK_LOGI("revceive tsp login state %d",m_tsp_login_state);
                    break;
                }
                default:
                    break;
            }
        }
    );

    rc = TspServiceApi::getInstance()->RegisterMsgCB(
        ProtoclID::E_PROT_JTT808,
        {JT_T808_MSG_ID_TP_RSP_DIAG},
        [this](IN IpcTspHead &ipc_head, IN void *data, std::size_t sz) {
            static int seq = 0;
            std::stringstream   ss;
            for ( decltype(ipc_head.length) index = 0; index < sz; index++ ) {
                ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << ((char *)data)[index];
            }
            printf(" revice tsp data %s",ss.str().c_str());
            printf("<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
            printf("*******message from tsp :id= 0x%04x .*******\r\n", ipc_head.msg_id);
            printf("*******message from tsp :seq_id= 0x%04x .*******\r\n", ipc_head.seq_id);
            if (JT_T808_MSG_ID_TP_RSP_DIAG != ipc_head.msg_id)
            {
                return;
            }
            IpcTspHead ipc_head_resp;
            ipc_head_resp.msg_id = JT808_MSG_ID_TP_RSP_GENERIC;
            ipc_head_resp.seq_id = seq++;
            ipc_head_resp.protocol = (SMLK_UINT8)(ipc_head.protocol);
            ipc_head_resp.qos =ipc_head.qos;
            ipc_head_resp.priority = ipc_head.priority;
            struct
            {
                SMLK_UINT16 seq_id;/*对应平台消息的流水号*/
                SMLK_UINT16  msg_id;/*对应平台消息de ID*/
                SMLK_UINT8 result;
            } response;
            response.seq_id = htobe16(ipc_head.seq_id);
            response.msg_id = htobe16(ipc_head.msg_id);
            response.result = (SMLK_UINT8)0;
            TspServiceApi::getInstance()->SendMsg(ipc_head_resp, (SMLK_UINT8 *)&response, (std::size_t)sizeof(response));
            /*收到云端的数据传递给诊修SDK*/
            if( NULL != m_sdk_instance)
            {
                m_sdk_instance->rcv_irq_datas_from_server((uint8_t*)data,sz,1);
            }
        }
    );
};

int SmlkDiag::InitWlan()
{
    SMLK_BOOL wlan_ret = smartlink_sdk::Wlan::GetInstance()->Init(ModuleID::E_Module_diag_service);
    if (!wlan_ret)
    {
        SMLK_LOGE("Init wlan failed");
        return -1;
    }
    std::vector<smartlink_sdk::WifiEventId> wlan_events =
    {
        smartlink_sdk::WifiEventId::E_WIFI_EVENT_MODE_CHANGED,
        smartlink_sdk::WifiEventId::E_WIFI_EVENT_STA_SCAN_AVAILABLE,
        smartlink_sdk::WifiEventId::E_WIFI_EVENT_STA_STATE_CHANGED,
    };

    auto return_code = smartlink_sdk::Wlan::GetInstance()->RegEventCB(
        wlan_events,
        [](smartlink_sdk::WifiEventId id, void *data, int len) {
            switch ( id ) {
                case smartlink_sdk::WifiEventId::E_WIFI_EVENT_MODE_CHANGED:
                    {
                        smartlink_sdk::WifiMode *mode = (smartlink_sdk::WifiMode *)data;
                        SMLK_LOGI("wifi mode changed  %d",*mode);
                    }
                    break;
                case smartlink_sdk::WifiEventId::E_WIFI_EVENT_STA_SCAN_AVAILABLE:
                    {
                        std::vector<WifiApInfo> list;
                        smartlink_sdk::Wlan::GetInstance()->GetApList(list);
                        /*遍历热点，寻找app的热点*/
                        auto iter = list.cbegin();
                        for ( ; iter != list.cend(); iter++)
                        {
                            //扫描热点是否与诊修APP一
                            if(iter->ssid == m_wifi_ssid)
                            {
                                break;
                            }
                        }
                        /*存在诊修热点*/
                         if (iter!=list.cend())
                        {
                            /*若未连接APP，则连接APP*/
                            if(!m_wifi_connected)
                            {
                                smartlink_sdk::WifiConfig config;
                                config.ssid = m_wifi_ssid;
                                config.password = m_wifi_key;
                                config.auth = WifiAuthType::E_WIFI_AUTH_TYPE_WPA2;
                                config.priority = 1;
                                config.networkId = 0;
                                config.status = smartlink_sdk::WifiConfigStatus::E_WIFI_CONFIG_CURRENT;
                                SMLK_INT32 netId = smartlink_sdk::Wlan::GetInstance()->addNetwork(config);
                                RtnCode rtn= smartlink_sdk::Wlan::GetInstance()->enableNetwork(netId,true);
                                if(rtn == RtnCode::E_SUCCESS)
                                {
                                    SMLK_LOGI("enable wifi %s successed",m_wifi_ssid.c_str());
                                }
                            }
                        }
                        // else /*APP 热点已经关闭*/
                        // {
                        //     if(m_wifi_connected)/*如果已经连上手机热点*/
                        //     {
                        //         m_wifi_connected = false;
                        //         SMLK_LOGI("wifi %s disconnected!",m_wifi_ssid.c_str());
                        //         /*诊修APP热点关闭，tbox切换到AP模式*/
                        //         smartlink_sdk::Wlan::GetInstance()->SetMode(WifiMode::E_WIFI_MODE_AP);
                        //     }
                        // }
                    }
                    break;

                case smartlink_sdk::WifiEventId::E_WIFI_EVENT_STA_STATE_CHANGED:
                    {
                        smartlink_sdk::WifiConnectInfo *info = (smartlink_sdk::WifiConnectInfo *)data;
                        SMLK_LOGI("wlan connect ssid      %s", info->ssid.c_str());
                        SMLK_LOGI("wlan connect networkId %d",info->networkId);
                        SMLK_LOGI("wlan connect state     %d",info->state);
                        SMLK_LOGI("app wifi ap name       %s",m_wifi_ssid.c_str());
                        if(info->state == smartlink_sdk::WifiStaState::E_WIFI_STA_STATE_CONNECTED)
                        {
                            /*连接到设置热点*/
                            if( info->ssid == m_wifi_ssid)
                            {
                                m_wifi_connected = true;
                                SMLK_LOGI("wifi %s connected !" ,m_wifi_ssid.c_str());
                            }
                            else /*连接不该连的热点，删掉该热点，避免自动重连*/
                            {
                                std::vector<WifiConfig> list;
                                smartlink_sdk::Wlan::GetInstance()->getConfiguredNetworks(list);
                                auto iter = list.cbegin();
                                for ( ; iter != list.cend(); iter++)
                                {
                                    //删掉该热点的自动重连配置
                                    if(iter->ssid == info->ssid)
                                    {
                                        smartlink_sdk::Wlan::GetInstance()->removeNetwork(iter->networkId);
                                        SMLK_LOGI("delete unexpected wifi %s connected !",info->ssid);
                                        break;
                                    }
                                }
                            }
                        }
                        else if (info->state == smartlink_sdk::WifiStaState::E_WIFI_STA_STATE_DISCONNECTED)//断开连接成功
                        {
                            m_wifi_connected = false;
                            SMLK_LOGI("wifi %s disconnected !",m_wifi_ssid.c_str());
                            /*诊修APP热点关闭，tbox切换到AP模式*/
                            smartlink_sdk::Wlan::GetInstance()->SetMode(WifiMode::E_WIFI_MODE_AP);
                            SMLK_LOGI("set wifi mode AP");
                        }
                    }
                    break;

                default:
                    SMLK_LOGE("unsupported event id: %u", static_cast<SMLK_UINT32>(id));
                    return;
            }
        }
    );
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to register event callback to wlan service, return code: %d", static_cast<std::int32_t>(return_code));
        return -1;
    }
    smartlink_sdk::Wlan::GetInstance()->SetMode(WifiMode::E_WIFI_MODE_AP);
    SMLK_LOGI("------------set wifi mode AP---------------");
    return 1;
};

int SmlkDiag::InitPower()
{

}

/*获取当前诊断模式*/
int SmlkDiag::GetDiagMode()
{

}

int SmlkDiag::SetDiagMode(bool lock)
{
}


/*  瑞修德sdk接口返回意义
    1: 成功
    0: 失败
    -1: 不支持
*/
tboxInterface_t * SmlkDiag::SdkInit()
{
    tboxInterface_t ins;
    std::memset(&ins, 0x00, sizeof(ins));
    ins.get_hardware_status     = SmlkDiag::GetInstance()->SdkGetTboxHardwareStatus;
    ins.get_hardware_checks     = SmlkDiag::GetInstance()->SdkGetTboxHardwareChecks;
    ins.get_tbox_info           = SmlkDiag::GetInstance()->SdkGetTboxInfo;
    ins.app_mode_config         = SmlkDiag::GetInstance()->SdkAppModeConfig;
    ins.send_can_messages       = SmlkDiag::GetInstance()->SdkSendCanMessage;
    ins.send_can_messages_flag  = SmlkDiag::GetInstance()->SdkSendCanMessageFlag;
    ins.can_filter              = SmlkDiag::GetInstance()->SdkTboxCanFilter;
    ins.can_config              = SmlkDiag::GetInstance()->SdkTboxCanConfig;
    ins.send_datas_to_server    = SmlkDiag::GetInstance()->SdkSendDatasToServer;
    ins.connect_ap              = SmlkDiag::GetInstance()->SdkConnectAp;
    ins.send_datas_to_wifi      = SmlkDiag::GetInstance()->SdkSendDataToWifi;
    ins.gps_info                = SmlkDiag::GetInstance()->SdkGetTboxGpsInfo;
    ins.ntwkInit                = SmlkDiag::GetInstance()->SdkNetworkInit;
    ins.get_ntwk_status         = SmlkDiag::GetInstance()->SdkNetworkStatus;
    ins.wifi_config             = SmlkDiag::GetInstance()->SdkWifiConfig;
    /*
     * It seems that nobody call following interfaces in the SDK
     * printf
     *
     * And the following interfaces should be called from TBox
     * rcv_irq_datas_from_server
     * rcv_irq_can_message
     * rcv_irq_datas_from_wifi
     *
     */
    return tboxInterface_init(&ins);
};

int SmlkDiag::SdkGetTboxInfo(tboxInfo_t *info)
{
    if ( nullptr == info ) {
        SMLK_LOGE("info is nullptr");
        return 0;
    }
    std::memset(info, 0x00, sizeof(*info));

    // info->isSysTimeValid = xxx;

    /*ign 状态,  0：正常上电，
                1：IG 断开有 CAN 数据，
                2: IG 断开且10s 无 can 数据
                3：休眠后 30min 唤醒状态 */
    if(m_ign_state){
        info->flag.Bits.ignStatus =  0;
    }
    else if(m_can_state){
         info->flag.Bits.ignStatus = 1;
    }
    else{
        info->flag.Bits.ignStatus =  2;
    };
    /*SN号*/
    std::strncpy((char *)info->snBuf, m_sn.c_str(), sizeof(info->snBuf)-1);
    info->snBufCnt = std::strlen((const char *)info->snBuf);
    info->flag.Bits.isSnBufValid = true;

    /*通信设备号和SN当前一致*/
    std::strncpy((char *)info->uniqueNumBuf,m_sn.c_str(), sizeof(info->uniqueNumBuf)-1);
    info->uniqueNumBufCnt = std::strlen((const char *)info->uniqueNumBuf);
    info->flag.Bits.isUniqueNumBufValid = true;

    SMLK_LOGD("info->flag.Bits.ign         : 0x%02X", info->flag.Bits.ignStatus);
    SMLK_LOGD("info->flag.Bits.isSnBufValid: 0x%02X", info->flag.Bits.isSnBufValid);
    SMLK_LOGD("info->snBufCnt              : 0x%02X", info->snBufCnt);
    SMLK_LOGD("info->snBuf                 : %s",info->snBuf);
    SMLK_LOGD("info->flag.Bits.isUniqueNumBufValid: 0x%02X", info->flag.Bits.isUniqueNumBufValid);
    SMLK_LOGD("info->uniqueNumBufCnt              : 0x%02X", info->uniqueNumBufCnt);
    SMLK_LOGD("info->uniqueNumBuf                 : %s",info->uniqueNumBuf);
    return 1;
};

int SmlkDiag::SdkGetTboxGpsInfo(tboxGPS_t *info)
{
    if ( nullptr == info )
    {
        SMLK_LOGE("info is nullptr");
        return 0;
    }
    /*主动获取定位信息，如果定位无效，返回最后一次有效定位，同时定位标志置为无效*/
    info->nsDimensionValue    = m_location.latitude;
    info->ewLongitudeValue   =  m_location.longitude;
    info->height    =  m_location.altitude;
    info->speed       =  m_location.speed;
    info->direction    =  m_location.heading;
    info->flag.Bits.V1A0  =  m_location.fix_valid ?0:1;
    info->flag.Bits.E0W1  =   m_location.longitude>0 ?0:1;
    info->flag.Bits.N0S1  =  m_location.latitude>0 ?0:1;

    SMLK_LOGD("info->nsDimensionValue : 0x%X",info->nsDimensionValue);
    SMLK_LOGD("info->ewLongitudeValue : 0x%X",info->ewLongitudeValue);
    SMLK_LOGD("info->height           : 0x%X",info->height);
    SMLK_LOGD("info->speed            : 0x%x",info->speed );
    SMLK_LOGD("info->direction        : 0x%x",info->direction );
    SMLK_LOGD("flag.Bits.V1A0         : 0x%X",info->flag.Bits.V1A0 );
    SMLK_LOGD("info->flag.Bits.E0W1   : 0x%X",info->flag.Bits.E0W1 );
    SMLK_LOGD("info->flag.Bits.N0S1   : 0x%X",info->flag.Bits.N0S1);
    return 1;
};

void SmlkDiag::SdkGetTboxHardwareStatus(tboxHardwareStatusFlag_u *flag)
{
    if ( nullptr == flag )
    {
        SMLK_LOGE("flag is nullptr");
        return;
    }
    /*是否登录云端并鉴权成功*/
    flag->Bits.isGprsOk = (m_tsp_login_state == TspLoginState::E_LOGIN)?1:0;
    /*是否可以发送数据，需要等收到云端应答才能发送，否者应答报文会乱序*/
    flag->Bits.isGprsUploadBufWriteEnable   = 1;//SmlkDiagTsp::GetInstance()->SendEnable();
    flag->Bits.isRtcUsed                    = 1;
    flag->Bits.isWifiConnected              = m_wifi_connected;
    flag->Bits.isWifiDataTransferEnable     = 1;
    flag->Bits.tbox_pid                     = 5;
    flag->Bits.tbox_vid                     = 5;
};

void SmlkDiag::SdkGetTboxHardwareChecks(tboxHardwareChecksFlag_u *flag)
{
    if ( nullptr == flag )
    {
        SMLK_LOGE("flag is nullptr");
        return;
    }
    flag->Bits.Can      = 0;
    flag->Bits.Gprs     = 0;
    flag->Bits.Gps      = 0;
    flag->Bits.Power    = 0;
    flag->Bits.SD       = 0;
};

int SmlkDiag::SdkTboxCanFilter(const tboxCanFilter_t *filter)
{
    if ( nullptr == filter )
    {
        SMLK_LOGE("filter is nullptr");
        return 0;
    }
    for(int i = 0;i<  filter->exdFilterIdCnt.dWord;i++)
    {
        uint32_t can_id = filter->pexdFilterIdBuf[i];
        /*使用set防止重复*/
        m_can_filter_list.insert(can_id);
        SMLK_LOGD("set can filter 0x%x",can_id);
    }
    /*设置设白名单到MCU*/
    std::vector<smartlink_sdk::CanFilterItem>  can_filter;
    for(std::set<uint32_t>::iterator it=m_can_filter_list.begin();it!=m_can_filter_list.end();it++)
    {
        /*0x0和0x80000000 表示通配，去掉通配符*/
        if( *it != 0x00 && *it != 0x80000000)
        {
            CanFilterItem item = {0};
            /*CAN 白名单*/
            item.reserve = 1;/*诊断报文，白名单不做超时处理*/
            item.delay = 0;/*can超期时间,默认150MS*/
            item.canid = *it;/*can id*/
            can_filter.push_back(item);
        }
    }
    if(can_filter.size() > 0)
    {
        RtnCode ret = smartlink_sdk::MCU::GetInstance()->SetCanFilter(smartlink_sdk::CanChannelId::E_CAN_CAHNNEL_1, false, can_filter);
        SMLK_LOGD("set  %d can id to filter \r\n",can_filter.size());
    }
    return 1;
};

int SmlkDiag::SdkTboxCanConfig(const tboxCanConfig_t *config)
{
    if ( nullptr == config ) {
        SMLK_LOGE("config is nullptr");
        return 0;
    }
#ifdef SL_DEBUG
    SMLK_LOGD("config->canChannel:          0x%02X", config->canChannel);
    SMLK_LOGD("config->canHPin:             0x%02X", config->canHPin);
    SMLK_LOGD("config->canLPin:             0x%02X", config->canLPin);
    SMLK_LOGD("config->canBaud.dWord:       0x%08X", config->canBaud.dWord);
    SMLK_LOGD("config->can1ResistorOnOff:   0x%02X", config->can1ResistorOnOff);
    SMLK_LOGD("config->can2ResistorOnOff:   0x%02X", config->can2ResistorOnOff);
#endif
    /*tbox2.0不支持pin设置*/
    return -1;
};

int SmlkDiag::SdkSendCanMessageFlag()
{
    static bool time_check_flag = false;
    static std::chrono::steady_clock ::time_point tp1;
    /*如果做了发送限制，需等待恢复，并做超时检测*/
    
    if((m_can_send_enable == false) && (time_check_flag == false))
    {
        time_check_flag = true;
        tp1 = std::chrono::steady_clock ::now();
    }
    /*进入超时检测*/
    if(time_check_flag)
    {
        std::chrono::steady_clock::time_point tp2 = std::chrono::steady_clock ::now();
        uint64_t sec= std::chrono::duration_cast<std::chrono::duration<int>> (tp2-tp1).count();
        if(sec >= 5)/*超过5s仍未接收响应,则退出发送限制*/
        {
            /*设置发送使能*/
            m_can_send_enable= true;
            time_check_flag = false;
            SMLK_LOGW("check send time  out %d!!!",sec);
        }
    }
    /*发送使能，或上一条UDS请求得到响应*/
    if( m_can_send_enable == true )
    {
         time_check_flag = false;
        //std::lock_guard<std::mutex> lock(g_uds_mutex);     
        /*判断当前是否有仲裁业务*/
        if(m_uds_status.mode == smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_INVALID)/*仲裁空闲,申请仲裁*/
        {
            /*申请仲裁*/
            SMLK_RC ret = smartlink::OBD::DidIpcApi::getInstance()->SetUdsDiagMode(smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_DIAG_OTHER, true,6);
            SMLK_LOGI("set uds diag mode rtn: %d!!!",ret);
            return ret == SMLK_RC::RC_OK ? true : false;
        }
        else
        {
            /*非诊修业务占据*/
            if(m_uds_status.mode != smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_DIAG_WRITE 
            && m_uds_status.mode != smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_DIAG_OTHER)
            {
                /*优先级小于诊修,申请仲裁*/
                if((m_uds_status.pri > smartlink::OBD::UdsDiagPriority::E_UDS_DIAG_PRI_DIAG_OTHER) 
                && (m_uds_status.pri != smartlink::OBD::UdsDiagPriority::E_UDS_DIAG_PRI_IDLE))
                {
                    SMLK_RC ret = smartlink::OBD::DidIpcApi::getInstance()->SetUdsDiagMode(smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_DIAG_OTHER, true,6);
                    SMLK_LOGI("set uds diag pri %0x rtn: %d!!!",m_uds_status.pri ,ret);
                    return ret == SMLK_RC::RC_OK ? true : false;
                }
                else /*当前模式诊修不可以发送*/
                {
                    //SMLK_LOGW("SdkSendCanMessageFlag m_uds_status.mode %d ,m_uds_status.pri %x!!!",m_uds_status.mode,m_uds_status.pri);
                    tboxCan1Can2RcvMsg_t pOutCanRxMsg = {0};
                        /*扩展位置处理*/
                    // pOutCanRxMsg.canRcvMsg.IDE = uds_rsp->req_id>>31;
                    pOutCanRxMsg.canRcvMsg.CanId =  0x18DB33F1;
                    //   pOutCanRxMsg.canRcvMsg.timeStamp = frame->utc;
                    pOutCanRxMsg.Rc1Rc2flag = 0x62;
                    pOutCanRxMsg.canRcvMsg.RDA.bytes.byte4 = 0x02;
                    pOutCanRxMsg.canRcvMsg.RDA.bytes.byte3 = 0x3E;
                    pOutCanRxMsg.canRcvMsg.RDA.bytes.byte2 = (uint8_t)m_uds_status.pri;/*响应仲裁优先级*/
                    pOutCanRxMsg.canRcvMsg.RDA.bytes.byte1 = 0;
                    pOutCanRxMsg.canRcvMsg.RDB.bytes.byte4 = 0;
                    pOutCanRxMsg.canRcvMsg.RDB.bytes.byte3 = 0;
                    pOutCanRxMsg.canRcvMsg.RDB.bytes.byte2 = 0;
                    pOutCanRxMsg.canRcvMsg.RDB.bytes.byte1 = 0;
                    if( NULL != m_sdk_instance )
                    {
                        SMLK_LOGW("uds busy for mode: %d ,pri :%x",m_uds_status.mode,m_uds_status.pri);
                        m_sdk_instance->rcv_irq_can_message(&pOutCanRxMsg);
                    }
                    sleep(5);/*冲突模式下轮询过快意义也不大，为降低CPU资源消耗暂停50ms*/
                    return 0;  
                }
            }
        }
    }
    else /*未得到UDS响应前不能发送下一帧,等10ms,否则SDK占用CPU过高*/
    {
        usleep(10000);
        return 0;
    }
    return 1;
};

int SmlkDiag::SdkSendCanMessage(const tboxCanTxMsg_t *msg, const uint32_t cnt)
{
   // printf("SdkSendCanMessage  \r\n");
    if ( nullptr == msg )
    {
        SMLK_LOGE("msg is nullptr");
        return 0;
    }

    static bool  uds_finish = false;
    uint32_t requestID;
    uint32_t responseID;
    static uint8_t serviceID;
    static uint16_t uds_len = 0;
    static std::vector<SMLK_UINT8> data;

    for( int i = 0; i<cnt; i++)
    {
        CanFrame frame = {0};
        frame.can_id = msg[i].CANID.dWord|(msg[i].txFrameInfo.Bits.IDE<<31);
        frame.can_dlc = msg[i].txFrameInfo.Bits.DLC;
        memcpy(&frame.data[0], &msg[i].TDA.dWord, 4);
        memcpy(&frame.data[4], &msg[i].TDB.dWord, 4);

        requestID = frame.can_id;
        responseID = (frame.can_id&0xFFFF0000) | ((frame.can_id<<8)&0x0000FF00) | ((frame.can_id>>8)&0x000000FF);
 
        smartlink_sdk::CanChannelId  channel =(smartlink_sdk::CanChannelId) (msg[i].txFrameInfo.Bits.tboxCanChannel-1);
        
        if(cnt == 1)
        {
            printf( "recv CAN ID %x, len %d, data: %02x %02x %02x %02x %02x %02x %02x %02x \r\n"
            ,frame.can_id,frame.can_dlc,frame.data[0],frame.data[1],frame.data[2],frame.data[3],
                                        frame.data[4],frame.data[5],frame.data[6],frame.data[7]);


            /*进入刷写模式*/  
            if((frame.data[0] == 0x02)&& (frame.data[1] == 0x3E) && (frame.data[2] == 0x80))
            {
                /*申请诊修刷写仲裁*/
                SMLK_RC ret = smartlink::OBD::DidIpcApi::getInstance()->SetUdsDiagMode(smartlink::OBD::UdsDiagMode::E_UDS_DIAG_MODE_DIAG_WRITE, true,10);
                if(ret != SMLK_RC::RC_OK)
                {
                    SMLK_LOGI("set uds diag mode write err: %d!!!",ret); 
                } 
            }
            /*进入编程会话*/
            else if((frame.data[0] == 0x02)&& (frame.data[1] == 0x10) && (frame.data[2] == 0x02))
            {
                /*发送响应报文*/
                tboxCan1Can2RcvMsg_t pOutCanRxMsg = {0};
                /*扩展位置处理*/
                pOutCanRxMsg.canRcvMsg.IDE = responseID >>31;
                data.reserve(8);
                //pOutCanRxMsg.canRcvMsg.timeStamp = frame->utc;
                pOutCanRxMsg.canRcvMsg.CanId = responseID&0x7FFFFFFF;
                if(channel == smartlink_sdk::CanChannelId::E_CAN_CAHNNEL_0)
                {
                    pOutCanRxMsg.Rc1Rc2flag = 0x52;
                }
                else if(channel == smartlink_sdk::CanChannelId::E_CAN_CAHNNEL_1)
                {
                    pOutCanRxMsg.Rc1Rc2flag = 0x62;
                }
                pOutCanRxMsg.canRcvMsg.RDA.bytes.byte4 = 0x03;
                pOutCanRxMsg.canRcvMsg.RDA.bytes.byte3 = 0x7F;
                pOutCanRxMsg.canRcvMsg.RDA.bytes.byte2 = 0x10;
                pOutCanRxMsg.canRcvMsg.RDA.bytes.byte1 = 0x78;
                if( NULL != m_sdk_instance )
                {
                    printf( "send to sdk delay CAN ID %x, data: %02x %02x %02x %02x %02x %02x %02x %02x \r\n"
                    ,pOutCanRxMsg.canRcvMsg.CanId
                    ,pOutCanRxMsg.canRcvMsg.RDA.bytes.byte4 
                    ,pOutCanRxMsg.canRcvMsg.RDA.bytes.byte3 
                    ,pOutCanRxMsg.canRcvMsg.RDA.bytes.byte2 
                    ,pOutCanRxMsg.canRcvMsg.RDA.bytes.byte1 
                    ,pOutCanRxMsg.canRcvMsg.RDB.bytes.byte4 
                    ,pOutCanRxMsg.canRcvMsg.RDB.bytes.byte3 
                    ,pOutCanRxMsg.canRcvMsg.RDB.bytes.byte2 
                    ,pOutCanRxMsg.canRcvMsg.RDB.bytes.byte1 
                    );

                    m_sdk_instance->rcv_irq_can_message(&pOutCanRxMsg);
                }

                std::vector<SMLK_UINT8> tmp;
                data.swap(tmp);/*清除空间*/
            }
        }

        if((frame.data[0]&0xF0) == 0x10 )/*首帧*/
        {
            uds_len = frame.data[0];
            uds_len = ( uds_len<<8 |frame.data[1])&0x0FFF;/*首帧长度*/
           // printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 111 uds len = %x\r\n",uds_len);
            serviceID = frame.data[2];/*首诊服务*/
            /*发送响应报文*/
            tboxCan1Can2RcvMsg_t pOutCanRxMsg = {0};
            /*扩展位置处理*/
            pOutCanRxMsg.canRcvMsg.IDE = responseID >>31;
            data.reserve(uds_len+2);
            //pOutCanRxMsg.canRcvMsg.timeStamp = frame->utc;
            pOutCanRxMsg.canRcvMsg.CanId = responseID&0x7FFFFFFF;
            if(channel == smartlink_sdk::CanChannelId::E_CAN_CAHNNEL_0)
            {
                pOutCanRxMsg.Rc1Rc2flag = 0x52;
            }
            else if(channel == smartlink_sdk::CanChannelId::E_CAN_CAHNNEL_1)
            {
                pOutCanRxMsg.Rc1Rc2flag = 0x62;
            }
            pOutCanRxMsg.canRcvMsg.RDA.bytes.byte4 = 0x30;
            pOutCanRxMsg.canRcvMsg.RDA.bytes.byte3 = 0x00;
            pOutCanRxMsg.canRcvMsg.RDA.bytes.byte2 = 0x00;

            if( NULL != m_sdk_instance )
            {
                m_sdk_instance->rcv_irq_can_message(&pOutCanRxMsg);
            }
            /*去掉serviceid*/
            for(int i = 3;i< 8; i++)
            {
                data.push_back( frame.data[i] );
            }
            uds_len = uds_len - 6;
            uds_finish = false;
        }
        else if((frame.data[0]&0xF0) == 0x20 )
        {
            if( uds_len > 7)
            {
                for(int i = 1;i< 8; i++)
                {
                    data.push_back( frame.data[i] );
                }
                uds_len = uds_len - 7;
                uds_finish = false;
            }
            else
            {
                for(int i = 1;i< uds_len+1; i++)
                {
                    data.push_back( frame.data[i] );
                }
                uds_finish = true;
                uds_len = 0;
            }
        }
        else if((frame.data[0]&0xF0) == 0x30) /*流控帧*/
        {
           // printf( "NULL CAN ID %x, len %d\r\n",requestID,data.size());
            uds_len = 0;
        }
        else /*单帧*/
        {

            uds_len = frame.data[0];/*第一个字节为长度*/
            serviceID = frame.data[1];/*第一个字节为serviceID*/
            data.reserve(uds_len);
            for(int i = 0; i<uds_len-1; i++ )
            {
                data.push_back( frame.data[2+i] );
            }


            // printf( "recv frame ID %x, len %d, data: %02x %02x %02x %02x %02x %02x %02x %02x \r\n"
            // ,frame.can_id,frame.can_dlc,frame.data[0],frame.data[1],frame.data[2],frame.data[3],
            //                             frame.data[4],frame.data[5],frame.data[6],frame.data[7]);

            // printf("frame.data[2]&0x80 == 0x80 = %d\r\n",frame.data[2]&0x80 == 0x80);

            if( ((frame.data[1] == 0x3E) || (frame.data[1] == 0x10 )|| (frame.data[1] == 0x28 ) || (frame.data[1] == 0x85 ))
              && ((frame.data[2]&0x80) == 0x80) )
            {
                m_can_send_enable  = true;
                // printf( "m_can_send_enable  set %d\r\n",m_can_send_enable);
            }
            else
            {
                // printf("frame.data[2]&0x80 == 0x80 = %d\r\n",frame.data[2]&0x80 == 0x80);
                m_can_send_enable  = false;
            }

            uds_finish = true;
        }
        if(uds_finish)
        {
  
            // if(cnt == 1)
            // {
            //     printf( "send CAN ID %x, len %d",requestID,data.size());
            //     for(int i = 0;i<data.size();i++ )
            //     {
            //         printf( " %x",data[i]);
            //     }
            //     printf( "\r\n");
            // }
            MCU::GetInstance()->sendUdsRequestAsync( requestID, responseID, serviceID, data.size(),(const char*) data.data());
            {//加一对大括号是可以让tmp退出{}的时候自动析构
                std::vector<SMLK_UINT8> tmp;
                data.swap(tmp);/*清除空间*/
                uds_finish = false;
            }
        }
    }
    return 1;
};

void SmlkDiag::SdkSendDatasToServer(const uint8_t *data, const uint32_t len, const uint32_t channel)
{
    static int m_send_seq = 0;
    IpcTspHead ipc_head = {0};
    ipc_head.msg_id = JT_T808_MSG_ID_TP_REQ_DIAG;
    ipc_head.seq_id = 0xFFFF;//主动上报，流水号为0xffff
    ipc_head.protocol = (SMLK_UINT16)ProtoclID::E_PROT_JTT808;
    ipc_head.qos = QOS_SEND_ALWAYS;
    ipc_head.priority = PRIORITY0;
    printf("send %x len %d to server>>>",ipc_head.msg_id,len);
    auto rc = TspServiceApi::getInstance()->SendMsg(ipc_head, data, (std::size_t)len);
    if (rc == SMLK_RC::RC_OK)
    {
        return;
    }
};

void SmlkDiag::SdkAppModeConfig(int mode)
{
     SMLK_LOGI(" SdkAppModeConfig set mode :    %d\r\n", mode);
    // smartlink_sdk::Wlan::GetInstance()->SetMode(WifiMode::E_WIFI_MODE_STA);
    // if (mode == 0x03) // diag uds write ecu version
    // {
    //     SmlkUds::getInstance()->SwitchOwnUdsMode(SmlkUdsMode::UDS_DIAG_MODE_DIAG_WRITE);
    // }
    // else
    // {
    //     SmlkUds::getInstance()->SwitchOwnUdsMode(SmlkUdsMode::UDS_DIAG_MODE_DIAG_OTHER);
    // }
};

int SmlkDiag::SdkWifiConfig(tboxWifiConfig_t *config)
{
    if ( nullptr == config ) {
        SMLK_LOGE("config is nullptr");
        return 0;
    }
    smartlink_sdk::WifiMode mode;
    smartlink_sdk::RtnCode ret = smartlink_sdk::Wlan::GetInstance()->GetMode(mode);
    if(smartlink_sdk::WifiMode::E_WIFI_MODE_AP == mode)
    {
        smartlink_sdk::WifiApInfo info;
        smartlink_sdk::RtnCode ret = smartlink_sdk::Wlan::GetInstance()->GetApConfig(info);
        if(ret != smartlink_sdk::RtnCode::E_SUCCESS)
        {
            SMLK_LOGE("GetApConfig failed!, error code[%d]",(SMLK_UINT32)ret);
            return -1;
        }
        config->wifiMode = 0;

        if((std::size_t)info.ssid.length() < sizeof(config->wifiSSID))
        {
            strncpy((char*)config->wifiSSID,info.ssid.c_str(),info.ssid.length());
            config->wifiSSIDLength = info.ssid.length();
        }
        if((std::size_t)info.password.length() < sizeof(config->wifiPassword))
        {
            strncpy((char*)config->wifiPassword,info.password.c_str(),info.password.length());
            config->wifiPasswordLength = info.password.length();
        }
    }
    else if(smartlink_sdk::WifiMode::E_WIFI_MODE_STA == mode)
    {
        config->wifiMode = 1;
    }
    else
    {
        SMLK_LOGE("Get wifi mode failed!, error wifi mode[%d]", mode);
    }
    return 1;
};

int SmlkDiag::SdkConnectAp(const char *ssid, const char *key, int timeout)
{
    if ( nullptr == ssid || nullptr == key )
    {
        SMLK_LOGE("ssid is nullptr or key is nullptr");
        return 0;
    }
    SMLK_LOGI("ssid:%s,key:%s,timeout: %d", ssid,key,timeout);
    m_wifi_connected = false;
    m_wifi_ssid = std::string(ssid);
    m_wifi_key =  std::string(key);
    m_wifi_timeout = timeout;
    m_wifi_connect_thread = std::thread(
    [timeout]() {
        auto tp1 = std::chrono::steady_clock::now();
        while(!m_wifi_connected )
        {
            auto tp2 = std::chrono::steady_clock::now();
            auto dtime = std::chrono::duration_cast<std::chrono::seconds>(tp2 - tp1).count();
            /*wifi连接超时,防止异常情况无法退出，由于wifi连接可能比较慢，比实际预留5s时间*/
            if( dtime > timeout*3)
            {
                //删掉该热点的自动重连配置
                std::vector<WifiConfig> list;
                smartlink_sdk::Wlan::GetInstance()->getConfiguredNetworks(list);
                auto iter = list.cbegin();
                for ( ; iter != list.cend(); iter++)
                {

                    if(iter->ssid == m_wifi_ssid)
                    {
                        smartlink_sdk::Wlan::GetInstance()->removeNetwork(iter->networkId);
                        SMLK_LOGI("delete unexpected wifi %s connected !",m_wifi_ssid.c_str());
                    }
                }
                /*tbox切回到AP模式*/
                smartlink_sdk::Wlan::GetInstance()->SetMode(WifiMode::E_WIFI_MODE_AP);
                SMLK_LOGI("timeout %d to connect wifi app, set wifi to AP mode",dtime);
                break;
            }
            smartlink_sdk::WifiMode mode;
            smartlink_sdk::RtnCode ret = smartlink_sdk::Wlan::GetInstance()->GetMode(mode);
            if(smartlink_sdk::WifiMode::E_WIFI_MODE_STA == mode)
            {
                smartlink_sdk::Wlan::GetInstance()->startScan();
                SMLK_LOGD("start scan %s",m_wifi_ssid.c_str());
            }
            else
            {
                smartlink_sdk::Wlan::GetInstance()->SetMode(WifiMode::E_WIFI_MODE_STA);
            }
            /*每3s扫描一次热点状态*/
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    });
    m_wifi_connect_thread.detach();
    return 1;
};

void SmlkDiag::SdkSendDataToWifi(const uint8_t *data, const uint32_t sz)
{
    if ( nullptr == data ) {
        SMLK_LOGE("tbox_send_datas_to_wifi");
        return;
    }
#ifdef SL_DEBUG
    std::stringstream   ss;
    for ( uint32_t index = 0; index < sz; index++ ) {
        ss << " 0x" << std::setw(2) << std::setfill('0') << std::hex << (int)data[index];
    }
    SMLK_LOGD("data:       %s", ss.str().c_str());
#endif
};

/*
问题：tboxGPRS_t 需要明确每个字段的具体意思以及怎么填写？
反馈：云端通讯完全由TBOX 处理，因此 2 个网络接口已不需要。
*/
int SmlkDiag::SdkNetworkInit(tboxGPRS_t *gprs)
{
    if ( nullptr == gprs ) {
        SMLK_LOGE("gprs is nullptr");
        return 0;
    }
    if(m_iccid.length() <= sizeof(gprs->simInfo.ICCID))
    {
        strncpy((char*)gprs->simInfo.ICCID,m_iccid.c_str(),m_iccid.length());
        gprs->simInfo.ICCID[m_iccid.length()] = '\0';
    }
    gprs->tbox_info_Buf.tbox_vin_ecu = (uint8_t*)m_vin.c_str();
    gprs->tbox_info_buf_len.tbox_vin_ecu_len = m_vin.length();
    SMLK_LOGI("m_vin:               %s\r\n",     m_vin.c_str());
    return 1;
};

/*
问题：tboxGPRS_t 需要明确每个字段的具体意思以及怎么填写？
反馈：云端通讯完全由TBOX 处理，因此 2 个网络接口已不需要。
*/
int SmlkDiag::SdkNetworkStatus(tboxGPRS_t *gprs)
{
    if ( nullptr == gprs ) {
        SMLK_LOGE("gprs is nullptr");
        return 0;
    }
    if(m_iccid.length() <= sizeof(gprs->simInfo.ICCID))
    {
        strncpy((char*)gprs->simInfo.ICCID,m_iccid.c_str(),m_iccid.length());
        gprs->simInfo.ICCID[m_iccid.length()] = '0';
    }
    gprs->tbox_info_Buf.tbox_vin_ecu = (uint8_t*)m_vin.c_str();
    gprs->tbox_info_buf_len.tbox_vin_ecu_len = m_vin.length();
    SMLK_LOGI("m_vin:               %s\r\n",     m_vin.c_str());
    return 1;
};

/******************************************************************************
*
*  Copyright (C) 2020 SmartLink
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/
#include <sstream>
#include <fstream>
#include <cstring>
#include <smlk_log.h>
#include <smlk_timer_new.h>
#include "event.h"
#include "errors_def.h"
#include <tsp_service_api.h>
#include <vehicle_data_api.h>
#include <smartlink_sdk_tel.h>
#include <smartlink_sdk_mcu.h>
#include <smartlink_sdk_location.h>
#include <smartlink_sdk_tel.h>
#include <smartlink_sdk_sys_power.h>
#include <location_manager.h>
#include "smartlink_sdk_sys_property_def.h"
#include "smartlink_sdk_sys_property.h"
#include "smartlink_sdk_sys_time.h"
#include "bluetooth_service.h"
#include "msg_def.h"
#include "ipc_api_def.h"
#include <vehicle_data_api.h>
#include <iomanip>

#pragma pack(1)


#pragma pack()

using namespace smartlink;

#define PUSH_U4(v)  \
    data.push_back((SMLK_UINT8)(((v) >> 24) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v) >> 16) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v) >>  8) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v)      ) & 0xFF))

#define PUSH_U2(v)  \
    data.push_back((SMLK_UINT8)(((v) >>  8) & 0xFF));   \
    data.push_back((SMLK_UINT8)(((v)      ) & 0xFF))

#define PUSH_U1(v)   \
    data.push_back(v)

#define PUSH_AR(v)  \
    data.insert(data.end(), (v), (v) + sizeof(v))

namespace CanBusState {
    static const SMLK_UINT32    CAN_NORMAL                  = 0x000000000;
    static const SMLK_UINT32    CAN_SLEEP                   = 0x000000001;
    static const SMLK_UINT32    CAN_AUTHENTICATING          = 0x000000002;
    static const SMLK_UINT32    CAN_FAILED                  = 0x000000003;
}

namespace InternalFlags {
    static const SMLK_UINT32    SL_FLAG_SYSTIME_SER        = 0x00000001;
    static const SMLK_UINT32    SL_FLAG_SYSTIME_SYN        = 0x00000002;
    static const SMLK_UINT32    SL_FLAG_COMPRESS           = 0x00000004;
    static const SMLK_UINT32    SL_FLAG_IGNON              = 0x00000008;
    static const SMLK_UINT32    SL_FLAG_ETC2_RECEIVED      = 0x00000010;
    static const SMLK_UINT32    SL_FLAG_SET_SIGNAL_INVALID = 0x00000020;

    static const SMLK_UINT32    SL_FLAG_PERIOD_WAKEUP      = 0x00000100;
    static const SMLK_UINT32    SL_FLAG_TEL_READY          = 0x00001000;
    static const SMLK_UINT32    SL_FLAG_CAN_NORMAL         = 0x00010000;
    static const SMLK_UINT32    SL_FLAG_DIRECT_CONTROL     = 0x00100000;
    static const SMLK_UINT32    SL_FLAG_DAY_CHANGE         = 0x01000000;
    static const SMLK_UINT32    SL_FLAG_TRIP_START         = 0x10000000;
}

namespace Protocol {
    static const SMLK_UINT8     SL_808  = static_cast<SMLK_UINT8>(ProtoclID::E_PROT_JTT808);
}

namespace IgnState {
    static const SMLK_UINT32    SL_OFF                  = 0x000000000;
    static const SMLK_UINT32    SL_ON                   = 0x000000001;
}

namespace MCU {
    static const SMLK_UINT32    SL_SERVICE_READY        = 0x00000001;
    static const SMLK_UINT32    SL_IGN_NOTIFY           = 0x00000002;
    static const SMLK_UINT32    SL_CAN_STATUS_NOTIFY    = 0x00000003;
}


BluetoothService::BluetoothService()
    : m_flags(IService::Uninitialized)
    , m_internal_flags(0)
{

}

BluetoothService::~BluetoothService()
{
    Stop();
}

/******************************************************************************
 * NAME: Stop
 *
 * DESCRIPTION: stop the service
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void BluetoothService::Stop()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Running ) {
        m_flags |= IService::Stopping;

        m_flags &= ~(IService::Running | IService::Stopping);
    }
}

/******************************************************************************
 * NAME: IsRunning
 *
 * DESCRIPTION: check the service running status
 *
 * PARAMETERS:
 *
 * RETURN:      true if the service already running, otherwise false
 *
 * NOTE:
 *****************************************************************************/
bool BluetoothService::IsRunning() const
{
    return  m_flags & IService::Running;
}

/******************************************************************************
 * NAME: Init
 *
 * DESCRIPTION: Init function for the collection service, you may finish all
 *              the initialization work here
 *
 * PARAMETERS:
 *
 * RETURN:
 *      SL_SUCCESS on success, and for now no error will return
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 BluetoothService::Init()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Initialized ) {
        SMLK_LOGI("service already initialized, do nothing");
        return SL_SUCCESS;
    }
    SMLK_LOGD("init TSP service");
    auto rc = TspServiceApi::getInstance()->Init(ModuleID::E_MODULE_bt_service);
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to init TSP service API interface!!!");
        return SL_EFAILED;
    }

    std::vector<TspEventId> tsp_events = {
        TspEventId::E_TSP_EVENT_CONNECT_STATE,
        TspEventId::E_TSP_EVENT_LOGIN_STATE,
    };

    SMLK_LOGD("register TSP event callback");
    rc = TspServiceApi::getInstance()->RegEventCB(
        tsp_events,
        [this](TspEventId id, void *data, int len) {
             SMLK_LOGI("revceive tsp event %d",id);
            switch(id)
            {
                case TspEventId::E_TSP_EVENT_CONNECT_STATE:
                {
                    TspConnectState* state =  (TspConnectState*)data;
                    SMLK_LOGI("revceive tsp connected state %d",*state);
                    break;
                }
                case TspEventId::E_TSP_EVENT_LOGIN_STATE:
                {
                    TspLoginState* state =  (TspLoginState*)data;
                    SMLK_LOGI("revceive tsp login state %d",*state);
                    break;
                }
                default:
                    break;
            }
        }
    );

    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register TSP service event callback");
        return SL_EFAILED;
    }

    SMLK_LOGD("register TSP message callback");
    std::vector<SMLK_UINT16> tsp_event = {
        M_SL_MSGID_PT_BLUETOOTH_DATA_RSP,
        M_SL_MSGID_PT_BLUETOOTH_CMD_REQ
    };
    rc = TspServiceApi::getInstance()->RegisterMsgCB(
        ProtoclID::E_PROT_JTT808,
        tsp_event,
        [this](IN IpcTspHead &ipc, IN void *data, std::size_t sz) {
#ifdef SL_DEBUG
            std::stringstream   ss;
            for ( decltype(ipc.length) index = 0; index < sz; index++ ) {
                ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << ((char *)data)[index];
            }
            SMLK_LOGD("IPC:%s", ss.str().c_str());
#endif
            Post(
                MsgHead(ipc.msg_id, ipc.seq_id, (SMLK_UINT8)ipc.protocol, (SMLK_UINT8)ipc.compress, ipc.length, 0),
                (SMLK_UINT8 *)data,
                sz
            );
        }
    );
    if ( SMLK_RC::RC_OK != rc ) {
        SMLK_LOGE("fail to register TSP service event callback");
        return SL_EFAILED;
    }

    auto return_code = smartlink_sdk::MCU::GetInstance()->Init(ModuleID::E_MODULE_bt_service);
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to init MCU service API instance!!! return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    std::vector<smartlink_sdk::McuEventId>  mcu_events = {
        smartlink_sdk::McuEventId::E_MCU_EVENT_ISREADY,
        smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE,
        smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_STATE,
        smartlink_sdk::McuEventId::E_MCU_EVENT_EXT_DATA,
    };
    SMLK_LOGD("register MCU event callback");
    return_code = smartlink_sdk::MCU::GetInstance()->RegEventCB(
        mcu_events,
        [this](smartlink_sdk::McuEventId id, void *data, int len) {
            MsgHead head(M_SL_MSGID_MCU_NTF, 0, 0, 0, 0, 0);
            std::vector<SMLK_UINT8> bytes;

            SMLK_LOGD("[MCU] MCU event received: 0x%08X", static_cast<SMLK_UINT32>(id));

            switch ( id ) {
                case smartlink_sdk::McuEventId::E_MCU_EVENT_ISREADY:
                    head.m_seq      = MCU::SL_SERVICE_READY;
                    break;
                case smartlink_sdk::McuEventId::E_MCU_EVENT_IG_STATE:
                    {
                        head.m_seq  = MCU::SL_IGN_NOTIFY;
                        if ( sizeof(smartlink_sdk::IGNState) != (std::size_t)len ) {
                            SMLK_LOGE("[MCU] unexpected body length for MCU IGN state changed notification: %d, expected: %d", len, (int)sizeof(smartlink_sdk::IGNState));
                            return;
                        }

                        // modfiy by lsc 2021/6/16  adaptive new IGN struct
                        smartlink_sdk::IGNState *ign = reinterpret_cast<smartlink_sdk::IGNState *>(data);

                        if (ign->on)
                        {
                            head.m_extra = IgnState::SL_ON;
                        }
                        else
                        {
                            head.m_extra = IgnState::SL_OFF;
                        }
                        if (ign->acc)
                        {

                        }
                        else
                        {

                        }
                        //modify end
                    }
                    break;
                case smartlink_sdk::McuEventId::E_MCU_EVENT_CAN_STATE:
                    {
                        if ( smartlink_sdk::CANState::E_CAN_BUS_STATE_NORMAL == *(reinterpret_cast<smartlink_sdk::CANState *>(data)) ) {
                            head.m_extra = CanBusState::CAN_NORMAL;
                        } else if (smartlink_sdk::CANState::E_CAN_BUS_STATE_SLEEP == *(reinterpret_cast<smartlink_sdk::CANState *>(data))) {
                            head.m_extra = CanBusState::CAN_SLEEP;
                        }
                        head.m_seq  = MCU::SL_CAN_STATUS_NOTIFY;
                    }
                    break;
                case smartlink_sdk::McuEventId::E_MCU_EVENT_EXT_DATA:
                    {
                        // void* data, int len;
                        SMLK_LOGI("[bledebug] E_MCU_EVENT_EXT_DATA callback ");
                        smartlink_sdk::McuExtData *inf = (smartlink_sdk::McuExtData *)data;

                        if (inf->type != MCU_EXT_MESSAGE_BLE)
                        {
                            SMLK_LOGD("[bledebug] E_MCU_EVENT_EXT_DATA not expect data type, len: %d", inf->type);
                            return;
                        }

                        SMLK_LOGI("[bledebug] receive mcu extra ble useful data len %d", (inf->len-2));
                        std::vector<SMLK_UINT8> data;
                        data.clear();

                        SMLK_UINT16 data_len_808 = (inf->len-2) + 1;
                        PUSH_U2(data_len_808);
                        SMLK_LOGI("[bledebug] push data_len_808 %d", data_len_808 );

                        PUSH_U1(0x01);

                        for (int i = 2; i < inf->len; i++)
                        {
                            PUSH_U1(inf->data[i]);
                            /* code */
                        }
                        IpcTspHead  resp_head;
                        std::memset(&resp_head, 0x00, sizeof(resp_head));

                        resp_head.msg_id    = M_SL_MSGID_TP_BLUETOOTH_SEND_DATA;
                        resp_head.protocol  = Protocol::SL_808;
                        resp_head.qos       = QOS_SEND_ALWAYS;
                        SMLK_LOGI("[bledebug] send [%x] data to TSP", M_SL_MSGID_TP_BLUETOOTH_SEND_DATA);
                        SMLK_LOGI("[bledebug] send data size %d", data.size() );

                        TspServiceApi::getInstance()->SendMsg(resp_head, data.data(), data.size());
                        break;
                    }
                default:
                    SMLK_LOGE("[MCU] unsupported event id: %u", static_cast<SMLK_UINT32>(id));
                    return;
            }
            Post(head, std::move(bytes));
        }
    );
    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("fail to init register event callback to MCU service!!! return code: %d", static_cast<std::int32_t>(return_code));
        return SL_EFAILED;
    }

    SMLK_LOGD("mark service as initialized");
    m_flags |= IService::Initialized;

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Start
 *
 * DESCRIPTION: Start function for the collection service, you may finish all
 *              the initialization work here
 *
 * PARAMETERS:
 *
 * RETURN:
 *      SL_SUCCESS on success, and for now no error will return
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 BluetoothService::Start()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if ( m_flags & IService::Running ) {
        SMLK_LOGI("service already started, do nothing");
        return SL_SUCCESS;
    }

    if ( !(m_flags & IService::Initialized) ) {
        SMLK_LOGE("service not initialized");
        return SL_EFAILED;
    }

    // TODO: read sql and check whether need use it

    TspConnectState m_tsp_connect_state;
    TspServiceApi::getInstance()->GetConnectState(m_tsp_connect_state);
    SMLK_LOGI("m_tsp_connect_state = %d.\n",m_tsp_connect_state);

    TspLoginState m_tsp_login_state;
    TspServiceApi::getInstance()->GetLoginState(m_tsp_login_state);
    SMLK_LOGI("m_tsp_login_state = %d.\n",m_tsp_login_state);

    SMLK_LOGD("mark service as running");
    m_flags |= IService::Running;

    SMLK_LOGD("try to start main work thread");
    SMLK_LOGD("try to start main work thread");
    m_thread = std::thread(
        [this](){
            SMLK_LOGI("main work thread started");
            while ( !(m_flags & IService::Stopping) ) {
                m_events.get(200)();
            }

            SMLK_LOGI("main work thread greacefully exit!!!");
        }
    );
    return SL_SUCCESS;
}


/******************************************************************************
 * NAME: Post
 *
 * DESCRIPTION: post message the service main queue, the function can be called
 *              by the service its self or other module
 *
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *        sz:   data length
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void BluetoothService::Post(IN MsgHead &head, IN SMLK_UINT8 *data, IN std::size_t sz)
{
    m_events.emplace(
        head,
        data,
        sz,
        std::bind(&BluetoothService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: Post
 *
 * DESCRIPTION: post message the service main queue, the function can be called
 *              by the service its self or other module
 *
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void BluetoothService::Post(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    m_events.emplace(
        head,
        data,
        std::bind(&BluetoothService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/******************************************************************************
 * NAME: Post
 *
 * DESCRIPTION: post message the service main queue, the function can be called
 *              by the service its self or other module
 *
 * PARAMETERS:
 *      head:   message head which has the basic information about this message
 *      data:   message data
 *
 * RETURN:
 *
 * NOTE:        this is a move operation, so after this call, the function
 *              caller should never use the data again
 *****************************************************************************/
void BluetoothService::Post(IN MsgHead &head, INOUT std::vector<SMLK_UINT8> &&data)
{
    m_events.emplace(
        head,
        std::move(data),
        std::bind(&BluetoothService::HandleEvent, this, std::placeholders::_1, std::placeholders::_2)
    );
}

bool BluetoothService::SendDataToMcu(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{

    SMLK_LOGE("BleDebug SendDataToMcu data size %d", data.size());
    std::vector<SMLK_UINT8> data_to_mcu;
    const TspCmdData* cmd = reinterpret_cast<const TspCmdData*>(data.data());

    SMLK_UINT16 datd_len = cmd->data_len - 1;
    SMLK_LOGE("BleDebug ### receive data len %d ", cmd->data_len-1);

    for ( int index = 0; index < datd_len; index++ ) {
        SMLK_LOGE("BleDebug ### receive  %d = %02x", index , cmd->data[index]);
        data_to_mcu.push_back(cmd->data[index]);
    }

    std::stringstream ss;
    for ( decltype(datd_len) index = 0; index < datd_len; index++ ) {
        ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << ((char *)cmd->data)[index];
    }

    SMLK_LOGD("[BleDebug] send data to mcu str is :%s", ss.str().c_str());
    auto return_code = smartlink_sdk::MCU::GetInstance()->SendExtraCmd(smartlink_sdk::MpuExtCmdType::E_MPU_BT_DATA, data_to_mcu);

    if ( smartlink_sdk::RtnCode::E_SUCCESS != return_code ) {
        SMLK_LOGE("[BleDebug] send data to mcu fail return code: %d", static_cast<std::int32_t>(return_code));
        return false;
    } else {
        return true;
        SMLK_LOGE("BleDebug ### send data to mcu success");
    }

}


/******************************************************************************
 * NAME: HandleEvent
 *
 * DESCRIPTION: the main entry for all the events includeing the IPC message,
 *              timer notify message, notify message between threads etc.
 *
 * PARAMETERS:
 *       head:  message head which has the basic information about this message
 *        data: message data
 *
 * RETURN:
 *
 * NOTE:        this is the main event entry for the main thread, any cost time
 *              operation may block other event, so do not do any const time
 *              operation in the main thread
 *****************************************************************************/
void BluetoothService::HandleEvent(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("HandleEvent m_id %d", head.m_id);

    switch ( head.m_id ) {
        case M_SL_MSGID_PT_BLUETOOTH_DATA_RSP:
        {
            SMLK_LOGI("[BleDebug] IPC receive %02X", M_SL_MSGID_PT_BLUETOOTH_DATA_RSP);
            SMLK_LOGI("[BleDebug] TSP reponse 0f71");
            // xuke 返回状态下发给mcu
            if (SendDataToMcu(head, data)) {
                SMLK_LOGI("[BleDebug] 0f71 send data to mcu");
            }
            break;
        }
        case M_SL_MSGID_PT_BLUETOOTH_CMD_REQ:
        {
            SMLK_LOGI("[BleDebug] IPC receive %02X", M_SL_MSGID_PT_BLUETOOTH_CMD_REQ);
            SMLK_LOGI("[BleDebug] should send tsp data to Mcu");
            /// 返回应答
            SMLK_BOOL result = false;
            if (SendDataToMcu(head, data)) {
                SMLK_LOGE("BleDebug Send data to mcu success");
            } else {
                SMLK_LOGE("BleDebug Send data to mcu fail");
            }

            std::vector<SMLK_UINT8> data;
            data.clear();
            PUSH_U2(head.m_seq); // htobe16
            PUSH_U1(0x01); // type
            if (!result) {
                PUSH_U1(0x01); // success
            } else {
                PUSH_U1(0x00); // success
            }
            IpcTspHead      resp_head;
            std::memset(&resp_head, 0x00, sizeof(resp_head));
            resp_head.protocol  = Protocol::SL_808;
            resp_head.msg_id    = M_SL_MSGID_TP_BLUETOOTH_CMD_RSP;
            resp_head.seq_id    = htobe16(head.m_seq);
            resp_head.qos       = QOS_SEND_ALWAYS;
            SMLK_LOGE("[BleDebug] send 0f72 response %d", data.size());
            TspServiceApi::getInstance()->SendMsg(resp_head, data.data(), data.size());

            break;
        }
        case M_SL_MSGID_MCU_NTF:
            OnMcuNotify(head, data);
            break;
        default:
            // TODO
            break;
    }
}

/******************************************************************************
 * NAME: OnMcuNotify
 *
 * DESCRIPTION: MCU service event handler
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void BluetoothService::OnMcuNotify(IN MsgHead &head, IN std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("[MCU] xukedebug OnMcuNotify");

    switch ( head.m_seq ) {
        case MCU::SL_CAN_STATUS_NOTIFY:
            {
            }
        break;
        case MCU::SL_SERVICE_READY:
            // MCU service will always broadcast current IGN status, so do not need to get the state
            break;
        case MCU::SL_IGN_NOTIFY:
            {
                if ( IgnState::SL_ON == head.m_extra ) {
                    if ( 0 == (m_internal_flags & InternalFlags::SL_FLAG_IGNON) ) {
                        m_internal_flags |= InternalFlags::SL_FLAG_IGNON;
                        m_internal_flags &= ~InternalFlags::SL_FLAG_SET_SIGNAL_INVALID;
                        OnIgnOn(head, data);
                    }
                } else {
                    if ( 0 != (m_internal_flags & InternalFlags::SL_FLAG_IGNON) ) {
                        m_internal_flags &= ~InternalFlags::SL_FLAG_IGNON;
                        // OnIgnOff(head, data);
                    }
                }
            }
            break;

        default:
            SMLK_LOGE("[MCU] unsupported MCU event notify: 0x%08x", head.m_seq);
            break;
    }
}

/******************************************************************************
 * NAME: OnIgnOn
 *
 * DESCRIPTION: IGN ON event handler
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void BluetoothService::OnIgnOn(IN MsgHead &head/*head*/, IN std::vector<SMLK_UINT8> &/*data*/)
{

    SMLK_LOGI("[bleDebug] xukedebug======BluetoothService::OnIgnOn=====");
    std::vector<SMLK_UINT8> test = {0x06, 0x00,0x01,0x00,0x01,0x02,0x03,0x04};
    SMLK_LOGI("[bleDebug]===send test data 0x00->0x04 =====");

   // SendDataToMcu(head, test);

}
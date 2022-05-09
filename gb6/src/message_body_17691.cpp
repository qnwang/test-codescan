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
#include <cstring>
#include <ctime>
#include <smlk_log.h>
#include <message_body_17691.h>

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


static void TimeStamp2BCD(IN std::time_t tt, INOUT SMLK_BCD *BCD, IN std::size_t sz) {
    if ( (nullptr == BCD) || (sz != M_GB_17691_SZ_TIMESTAMP) ) {
        SMLK_LOGE("TimeStamp2BCD FAIL!!!");
        return;
    }
    std::tm tm;
    localtime_r(&tt, &tm);
    // check whether src time is invalid
    std::time_t local_time;
    if (tt == 0) {
        SMLK_LOGD("timestamp src = 0");
        std::memset(BCD, 0x0, sz);
        return;
    } else {
        localtime_r(&tt, &tm);
    }

    tm.tm_year -= 100;      // years since 2000
    tm.tm_mon += 1;

    BCD[0]   = (SMLK_UINT8)tm.tm_year;
    BCD[1]   = (SMLK_UINT8)tm.tm_mon;
    BCD[2]   = (SMLK_UINT8)tm.tm_mday;
    BCD[3]   = (SMLK_UINT8)tm.tm_hour;
    BCD[4]   = (SMLK_UINT8)tm.tm_min;
    BCD[5]   = (SMLK_UINT8)tm.tm_sec;
}

MessageEngineInfo17691::MessageEngineInfo17691()
{}

MessageEngineInfo17691::MessageEngineInfo17691(const MessageEngineInfo17691 &other)
    : MessageEngineInfo(other)
{}

MessageEngineInfo17691::~MessageEngineInfo17691()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION:     encode the message to bytes
 *
 * PARAMETERS:
 *  encoded         vector of bytes to store the encoded data
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 MessageEngineInfo17691::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    if ( m_tachograph_vehicle_speed.Valid() ) {
        PUSH_U2(m_tachograph_vehicle_speed.Encoded());
    } else {
        PUSH_U2(0xFFFF);
    }

    if ( m_barometric_pressure.Valid() ) {
        PUSH_U1(m_barometric_pressure.Encoded());
    } else {
        PUSH_U1(0xFF);
    }

    if ( m_actual_engine_percent_torque.Valid() ) {
        PUSH_U1(m_actual_engine_percent_torque.Encoded());
    } else {
        PUSH_U1(0xFF);
    }

    if ( m_nominal_friction_percent_torque.Valid() ) {
        PUSH_U1(m_nominal_friction_percent_torque.Encoded());
    } else {
        PUSH_U1(0xFF);
    }

    if ( m_engine_speed.Valid() ) {
        PUSH_U2(m_engine_speed.Encoded());
    } else {
        PUSH_U2(0xFFFF);
    }

    if ( m_engine_fuel_rate.Valid() ) {
        PUSH_U2(m_engine_fuel_rate.Encoded());
    } else {
        PUSH_U2(0xFFFF);
    }

    if ( m_aftertreatment_1_intake_nox.Valid() ) {
        PUSH_U2(m_aftertreatment_1_intake_nox.Encoded());
    } else {
        PUSH_U2(0xFFFF);
    }

    if ( m_aftertreatment_1_outlet_nox.Valid() ) {
        PUSH_U2(m_aftertreatment_1_outlet_nox.Encoded());
    } else {
        PUSH_U2(0xFFFF);
    }

    if ( m_catalyst_tank_level.Valid() ) {
        PUSH_U1(m_catalyst_tank_level.Encoded());
    } else {
        PUSH_U1(0xFF);
    }

    if ( m_engine_inlet_air_mass_flow_rate.Valid() ) {
        PUSH_U2(m_engine_inlet_air_mass_flow_rate.Encoded());
    } else {
        PUSH_U2(0xFFFF);
    }

    if ( m_aftertreatment_1_scr_intake_temperature.Valid() ) {
        PUSH_U2(m_aftertreatment_1_scr_intake_temperature.Encoded());
    } else {
        PUSH_U2(0xFFFF);
    }

    if ( m_aftertreatment_1_scr_outlet_temperature.Valid() ) {
        PUSH_U2(m_aftertreatment_1_scr_outlet_temperature.Encoded());
    } else {
        PUSH_U2(0xFFFF);
    }

    if ( m_aftertreatment_1_dpf_differential_pressure.Valid() ) {
        PUSH_U2(m_aftertreatment_1_dpf_differential_pressure.Encoded());
    } else {
        PUSH_U2(0xFFFF);
    }

    if ( m_engine_coolant_temperature.Valid() ) {
        PUSH_U1(m_engine_coolant_temperature.Encoded());
    } else {
        PUSH_U1(0xFF);
    }

    if ( m_fuel_level.Valid() ) {
        PUSH_U1(m_fuel_level.Encoded());
    } else {
        PUSH_U1(0xFF);
    }

    if ( m_located_status.Valid() ) {
        PUSH_U1(m_located_status.Encoded());
    } else {
        PUSH_U1(0xFF);
    }

    if ( m_longitude.Valid() ) {
        PUSH_U4(m_longitude.Encoded());
    } else {
        PUSH_U4(0xFFFFFFFF);
    }

    if ( m_latitude.Valid() ) {
        PUSH_U4(m_latitude.Encoded());
    } else {
        PUSH_U4(0xFFFFFFFF);
    }

    if ( m_total_vehicle_distance.Valid() ) {
        PUSH_U4(m_total_vehicle_distance.Encoded());
    } else {
        PUSH_U4(0xFFFFFFFF);
    }

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Decode
 *
 * DESCRIPTION:     decode message from bytes
 *
 * PARAMETERS:
 *  data            vector of bytes to be decoded
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 * NOTE:
 *                  this function will only parse it's required bytes from 
 *                  data, that means the data must contain at least the required
 *                  bytes at the head of data.
 *****************************************************************************/
SMLK_UINT32 MessageEngineInfo17691::Decode(IN std::vector<SMLK_UINT8> &data)
{
    if ( data.size() < sizeof(GB_17691_EngineDataStream) ) {
        return SL_EFAILED;
    }

    std::size_t index = 0;

    m_tachograph_vehicle_speed.AssignEncoded((SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1]);
    index += sizeof(SMLK_UINT16);

    m_barometric_pressure.AssignEncoded(data[index]);
    ++index;

    m_actual_engine_percent_torque.AssignEncoded(data[index]);
    ++index;

    m_nominal_friction_percent_torque.AssignEncoded(data[index]);
    ++index;

    m_engine_speed.AssignEncoded((SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1]);
    index += sizeof(SMLK_UINT16);

    m_engine_fuel_rate.AssignEncoded((SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1]);
    index += sizeof(SMLK_UINT16);

    m_aftertreatment_1_intake_nox.AssignEncoded((SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1]);
    index += sizeof(SMLK_UINT16);

    m_aftertreatment_1_outlet_nox.AssignEncoded((SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1]);
    index += sizeof(SMLK_UINT16);

    m_catalyst_tank_level.AssignEncoded(data[index]);
    ++index;

    m_engine_inlet_air_mass_flow_rate.AssignEncoded((SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1]);
    index += sizeof(SMLK_UINT16);

    m_aftertreatment_1_scr_intake_temperature.AssignEncoded((SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1]);
    index += sizeof(SMLK_UINT16);

    m_aftertreatment_1_scr_outlet_temperature.AssignEncoded((SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1]);
    index += sizeof(SMLK_UINT16);

    m_aftertreatment_1_dpf_differential_pressure.AssignEncoded((SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1]);
    index += sizeof(SMLK_UINT16);

    m_engine_coolant_temperature.AssignEncoded(data[index]);
    ++index;

    m_fuel_level.AssignEncoded(data[index]);
    ++index;

    m_located_status.AssignEncoded(data[index]);
    ++index;

    m_longitude.AssignEncoded((SMLK_UINT32)data[index] << 24 | (SMLK_UINT32)data[index+1] << 16 | (SMLK_UINT32)data[index+2] << 8 | (SMLK_UINT32)data[index+3]);
    index += sizeof(SMLK_UINT32);

    m_latitude.AssignEncoded((SMLK_UINT32)data[index] << 24 | (SMLK_UINT32)data[index+1] << 16 | (SMLK_UINT32)data[index+2] << 8 | (SMLK_UINT32)data[index+3]);
    index += sizeof(SMLK_UINT32);

    m_total_vehicle_distance.AssignEncoded((SMLK_UINT32)data[index] << 24 | (SMLK_UINT32)data[index+1] << 16 | (SMLK_UINT32)data[index+2] << 8 | (SMLK_UINT32)data[index+3]);
    index += sizeof(SMLK_UINT32);

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: EncodedSize
 *
 * DESCRIPTION:     get the message encoded size
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the encoded size
 *
 * NOTE:
 *****************************************************************************/
std::size_t MessageEngineInfo17691::EncodedSize() const
{
    return sizeof(GB_17691_EngineDataStream);
}

/******************************************************************************
 * NAME: Duplicate
 *
 * DESCRIPTION:     create a duplicate instance of this instance, deep copy
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the duplicated instance
 *
 * NOTE:
 *****************************************************************************/
std::shared_ptr<IMessage> MessageEngineInfo17691::Duplicate() const
{
    return std::make_shared<MessageEngineInfo17691>(*this);
}

MessageOBDInfo17691::MessageOBDInfo17691()
{}

MessageOBDInfo17691::MessageOBDInfo17691(const MessageOBDInfo17691 &other)
    : MessageOBDInfo(other)
{}

MessageOBDInfo17691::~MessageOBDInfo17691()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION:     encode the message to bytes
 *
 * PARAMETERS:
 *  encoded         vector of bytes to store the encoded data
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 MessageOBDInfo17691::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    PUSH_U1(m_protocol);
    PUSH_U1(m_mil_status);
    PUSH_U2(m_diag_support_status);
    PUSH_U2(m_diag_ready_status);
    PUSH_AR(m_vin);
    PUSH_AR(m_scin);
    PUSH_AR(m_cvn);
    PUSH_AR(m_iupr);

    if (m_dtc != 0xFE){
        PUSH_U1((SMLK_UINT8)m_dtcs.size());
        for ( auto const &dtc : m_dtcs ) {
            PUSH_U4(dtc);
        }
    } else {
        PUSH_U1(m_dtc);
    }

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Decode
 *
 * DESCRIPTION:     decode message from bytes
 *
 * PARAMETERS:
 *  data            vector of bytes to be decoded
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 * NOTE:
 *                  this function will only parse it's required bytes from 
 *                  data, that means the data must contain at least the required
 *                  bytes at the head of data.
 *****************************************************************************/
SMLK_UINT32 MessageOBDInfo17691::Decode(IN std::vector<SMLK_UINT8> &data)
{
    /* check message body basic size */
    if ( data.size() <= sizeof(GB_17691_OBD_InformationHead) ) {
        SMLK_LOGE("invalid data size: %lu, expected at least: %lu", data.size(), sizeof(GB_17691_OBD_InformationHead));
        return SL_EFAILED;
    }

    /* check message body size*/
    SMLK_UINT8 dtc_count = data[sizeof(GB_17691_OBD_InformationHead)];
    std::size_t expected_sz = sizeof(GB_17691_OBD_InformationHead);

    if (dtc_count != 0xFE) {
        expected_sz = expected_sz + ((std::size_t)dtc_count) * sizeof(SMLK_UINT32);
    }

    if ( data.size() < expected_sz ) {
        SMLK_LOGE("dtc_cout:   0x%02X", dtc_count);
        SMLK_LOGE("data.size():    %lu, expected_sz:   %lu", data.size(), expected_sz);
        return SL_EFORMAT;
    }    

    std::size_t index = 0;
    SMLK_UINT8  protocol    = data[index++];
    SMLK_UINT8  mil_status  = data[index++];

    /* valid OBD protocol */
    if ( M_GB_17691_OBD_PROTOCOL_ISO15765   != protocol 
      && M_GB_17691_OBD_PROTOCOL_ISO27154   != protocol 
      && M_GB_17691_OBD_PROTOCOL_SAEJ1939   != protocol 
      && M_GB_17691_OBD_PROTOCOL_INVALID    != protocol ) {
          SMLK_LOGE("invalid OBD protocol: 0x%02X", protocol);
          return SL_EFORMAT;
    }

    if ( M_GB_17691_MIL_OFF     != mil_status 
      && M_GB_17691_MIL_ON      != mil_status 
      && M_GB_17691_MIL_INVALID != mil_status ) {
          SMLK_LOGE("invalid mil_status");
          return SL_EFORMAT;
    }

    m_protocol              = protocol;
    m_mil_status            = mil_status;

    m_diag_support_status   = (((SMLK_UINT16)data[index]) << 8) | ((SMLK_UINT16)data[index+1]);
    index += 2;

    m_diag_ready_status     = (((SMLK_UINT16)data[index]) << 8) | ((SMLK_UINT16)data[index+1]);
    index += 2;

    std::memcpy(m_vin, &data[index], sizeof(m_vin));
    index += sizeof(m_vin);

    std::memcpy(m_scin, &data[index], sizeof(m_scin));
    index += sizeof(m_scin);

    std::memcpy(m_cvn, &data[index], sizeof(m_cvn));
    index += sizeof(m_cvn);

    std::memcpy(m_iupr, &data[index], sizeof(m_iupr));
    index += sizeof(m_iupr);

    ++index;        // one byte dtc code length

    m_dtc = dtc_count;

    if (dtc_count != 0xFE) {
    m_dtcs.clear();
        for ( std::size_t i = 0; i < (std::size_t)dtc_count; i++ ) {
            SMLK_UINT32 dtc = ((SMLK_UINT32)data[index]) << 24 | ((SMLK_UINT32)data[index+1]) << 16 | ((SMLK_UINT32)data[index+2]) << 8 | (SMLK_UINT32)data[index+3];
            m_dtcs.push_back(dtc);

            index += 4;
        }
    }

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: EncodedSize
 *
 * DESCRIPTION:     get the message encoded size
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the encoded size
 *
 * NOTE:
 *****************************************************************************/
std::size_t MessageOBDInfo17691::EncodedSize() const
{
    return sizeof(GB_17691_OBD_InformationHead) + 1 + sizeof(SMLK_UINT32) * m_dtcs.size();
}

/******************************************************************************
 * NAME: Duplicate
 *
 * DESCRIPTION:     create a duplicate instance of this instance, deep copy
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the duplicated instance
 *
 * NOTE:
 *****************************************************************************/
std::shared_ptr<IMessage> MessageOBDInfo17691::Duplicate() const
{
    return std::make_shared<MessageOBDInfo17691>(*this);
}

MessageReport17691::MessageReport17691()
{}

MessageReport17691::MessageReport17691(const MessageReport17691 &other)
    : MessageReport(other)
{}

MessageReport17691::~MessageReport17691()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION:     encode the message to bytes
 *
 * PARAMETERS:
 *  encoded         vector of bytes to store the encoded data
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 MessageReport17691::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    // SMLK_LOGI("MessageReport17691::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);

    PUSH_AR(t_timestamp);
    
    for ( auto const &inf : m_infos ) {
        std::vector<SMLK_UINT8> body;
        std::get<1>(inf)->Encode(body);
        // append message type
        PUSH_U1(std::get<0>(inf));
        // append sequence
         PUSH_U2(m_seq);
        data.insert(data.end(), body.cbegin(), body.cend());
    }

    return SL_SUCCESS;    
}

/******************************************************************************
 * NAME: Decode
 *
 * DESCRIPTION:     decode message from bytes
 *
 * PARAMETERS:
 *  data            vector of bytes to be decoded
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 * NOTE:
 *                  this function will only parse it's required bytes from 
 *                  data, that means the data must contain at least the required
 *                  bytes at the head of data.
 *****************************************************************************/
SMLK_UINT32 MessageReport17691::Decode(IN std::vector<SMLK_UINT8> &data)
{
    // // printf("MessageReport17691::Decode \n");

    // // for (auto iter = data.cbegin(); iter != data.cend(); iter++) {
    // //     printf("%02X ", *iter);
    // // }
    // // printf(" \n");

    // if ( data.size() < sizeof(m_timestamp) + sizeof(m_seq) ) {
    //     SMLK_LOGE("invalid data size, data.size()");
    //     return SL_EFAILED;
    // }

    // std::size_t index = 0;
    // std::memcpy(m_timestamp, &data[index], sizeof(m_timestamp));
    // index += sizeof(m_timestamp);

    // // m_seq = (SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1];
    // // index += sizeof(m_seq);

    // m_infos.clear();

    // while ( index < data.size() ) {
    //     switch ( data[index] ) {
    //         case M_GB_17691_MESSAGE_OBD:
    //         {
    //             m_seq = (SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1];
    //             index += sizeof(m_seq);

    //             std::shared_ptr<MessageOBDInfo17691> info = std::make_shared<MessageOBDInfo17691>();
    //             std::vector<SMLK_UINT8> item(&data[index+1], &data[0] + data.size());

    //             auto rc = info->Decode(item);
    //             if ( SL_SUCCESS != rc ) {
    //                 SMLK_LOGE("fail to parse OBD information!");
    //                 return rc;
    //             }

    //             m_infos.emplace_back(InfoType::OBD, info);

    //             index += 1 + info->EncodedSize();
    //             break;
    //         }
    //         case M_GB_17691_MESSAGE_STREAM:
    //         {
    //             m_seq = (SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1];
    //             index += sizeof(m_seq);

    //             std::shared_ptr<MessageEngineInfo17691> info = std::make_shared<MessageEngineInfo17691>();
    //             std::vector<SMLK_UINT8> item(&data[index+1], &data[0] + data.size());
    //             auto rc = info->Decode(item);
    //             if ( SL_SUCCESS != rc ) {
    //                 SMLK_LOGE("fail to parse ENGINE stream information!");
    //                 return rc;
    //             }

    //             m_infos.emplace_back(InfoType::Stream, info);

    //             index += 1 + info->EncodedSize();                
    //             break;
    //         }
    //         default:
    //         {
    //             return SL_EFORMAT;
    //         }
    //     }
    // }
    // return SL_SUCCESS;
}

/******************************************************************************
 * NAME: EncodedSize
 *
 * DESCRIPTION:     get the message encoded size
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the encoded size
 *
 * NOTE:
 *****************************************************************************/
std::size_t MessageReport17691::EncodedSize() const
{
    std::size_t sz = sizeof(m_timestamp) + sizeof(m_seq);

    for ( auto const &tuple : m_infos ) {
        sz += sizeof(std::get<0>(tuple)) + std::get<1>(tuple)->EncodedSize();
    }

    return sz;
}

/******************************************************************************
 * NAME: Duplicate
 *
 * DESCRIPTION:     create a duplicate instance of this instance, deep copy
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the duplicated instance
 *
 * NOTE:
 *****************************************************************************/
std::shared_ptr<IMessage> MessageReport17691::Duplicate() const
{
    return std::make_shared<MessageReport17691>(*this);
}

MessageVehicleLogin17691::MessageVehicleLogin17691()
{}

MessageVehicleLogin17691::MessageVehicleLogin17691(const MessageVehicleLogin17691 &other)
    : MessageVehicleLogin(other)
{}

MessageVehicleLogin17691::~MessageVehicleLogin17691()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION:     encode the message to bytes
 *
 * PARAMETERS:
 *  encoded         vector of bytes to store the encoded data
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 MessageVehicleLogin17691::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    // SMLK_LOGI("MessageVehicleLogin17691::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);

    PUSH_AR(t_timestamp);
    PUSH_U2(m_seq);

    data.insert(data.end(), m_iccid.cbegin(), m_iccid.cend());

    return SL_SUCCESS;    
}

/******************************************************************************
 * NAME: Decode
 *
 * DESCRIPTION:     decode message from bytes
 *
 * PARAMETERS:
 *  data            vector of bytes to be decoded
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 * NOTE:
 *                  this function will only parse it's required bytes from 
 *                  data, that means the data must contain at least the required
 *                  bytes at the head of data.
 *****************************************************************************/
SMLK_UINT32 MessageVehicleLogin17691::Decode(IN std::vector<SMLK_UINT8> &data)
{
    // if ( data.size() < sizeof(m_timestamp) + sizeof(m_seq) ) {
    //     return SL_EFAILED;
    // }

    // std::size_t index = 0;
    // std::memcpy(m_timestamp, &data[index], sizeof(m_timestamp));
    // index += sizeof(m_timestamp);

    // m_seq = (SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1];
    // index += sizeof(m_seq);

    // m_iccid = std::string(&data[index], &data[data.size()]);

    // return SL_SUCCESS;
}

/******************************************************************************
 * NAME: EncodedSize
 *
 * DESCRIPTION:     get the message encoded size
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the encoded size
 *
 * NOTE:
 *****************************************************************************/
std::size_t MessageVehicleLogin17691::EncodedSize() const
{
    return sizeof(m_timestamp) + sizeof(m_seq) + m_iccid.size();
}

/******************************************************************************
 * NAME: Duplicate
 *
 * DESCRIPTION:     create a duplicate instance of this instance, deep copy
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the duplicated instance
 *
 * NOTE:
 *****************************************************************************/
std::shared_ptr<IMessage> MessageVehicleLogin17691::Duplicate() const
{
    return std::make_shared<MessageVehicleLogin17691>(*this);
}

MessageVehicleLogout17691::MessageVehicleLogout17691()
{}

MessageVehicleLogout17691::MessageVehicleLogout17691(const MessageVehicleLogout17691 &other)
    : MessageVehicleLogout(other)
{}

MessageVehicleLogout17691::~MessageVehicleLogout17691()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION:     encode the message to bytes
 *
 * PARAMETERS:
 *  encoded         vector of bytes to store the encoded data
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 MessageVehicleLogout17691::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    // SMLK_LOGI("MessageVehicleLogout17691::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);

    PUSH_AR(t_timestamp);
    PUSH_U2(m_seq);

    return SL_SUCCESS;    
}

/******************************************************************************
 * NAME: Decode
 *
 * DESCRIPTION:     decode message from bytes
 *
 * PARAMETERS:
 *  data            vector of bytes to be decoded
 * 
 * RETURN:
 *  SL_SUCCESS      on success
 *                  else return the error code
 * NOTE:
 *                  this function will only parse it's required bytes from 
 *                  data, that means the data must contain at least the required
 *                  bytes at the head of data.
 *****************************************************************************/
SMLK_UINT32 MessageVehicleLogout17691::Decode(IN std::vector<SMLK_UINT8> &data)
{
    // if ( data.size() < sizeof(GB_17691_VehicleLogout) ) {
    //     return SL_EFAILED;
    // }

    // std::size_t index = 0;
    // std::memcpy(m_timestamp, &data[index], sizeof(m_timestamp));
    // index += sizeof(m_timestamp);

    // m_seq = (SMLK_UINT16)data[index] << 8 | (SMLK_UINT16)data[index+1];
    // index += sizeof(m_seq);

    // return SL_SUCCESS;
}

/******************************************************************************
 * NAME: EncodedSize
 *
 * DESCRIPTION:     get the message encoded size
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the encoded size
 *
 * NOTE:
 *****************************************************************************/
std::size_t MessageVehicleLogout17691::EncodedSize() const
{
    return sizeof(m_timestamp) + sizeof(m_seq);
}

/******************************************************************************
 * NAME: Duplicate
 *
 * DESCRIPTION:     create a duplicate instance of this instance, deep copy
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  the duplicated instance
 *
 * NOTE:
 *****************************************************************************/
std::shared_ptr<IMessage> MessageVehicleLogout17691::Duplicate() const
{
    return std::make_shared<MessageVehicleLogout17691>(*this);
}

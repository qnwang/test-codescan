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
#include <message_body_1239.h>

#include <stdint.h>
#include <inttypes.h>


using namespace smartlink;

#define PUSH_U4(v)                                    \
    data.push_back((SMLK_UINT8)(((v) >> 24) & 0xFF)); \
    data.push_back((SMLK_UINT8)(((v) >> 16) & 0xFF)); \
    data.push_back((SMLK_UINT8)(((v) >> 8) & 0xFF));  \
    data.push_back((SMLK_UINT8)(((v)) & 0xFF))

#define PUSH_U2(v)                                   \
    data.push_back((SMLK_UINT8)(((v) >> 8) & 0xFF)); \
    data.push_back((SMLK_UINT8)(((v)) & 0xFF))

#define PUSH_U1(v) \
    data.push_back(v)

#define PUSH_AR(v) \
    data.insert(data.end(), (v), (v) + sizeof(v))

static void TimeStamp2BCD(IN std::time_t tt, INOUT SMLK_BCD *BCD, IN std::size_t sz)
{
    if ((nullptr == BCD) || (sz != M_GB_17691_SZ_TIMESTAMP))
    {
        SMLK_LOGE("TimeStamp2BCD FAIL!!!");
        return;
    }
    std::tm tm;
    localtime_r(&tt, &tm);
    // check whether src time is invalid
    std::time_t local_time;
    if (tt == 0)
    {
        SMLK_LOGD("timestamp src = 0");
        std::memset(BCD, 0x0, sz);
        return;
    }
    else
    {
        localtime_r(&tt, &tm);
    }

    tm.tm_year -= 100; // years since 2000
    tm.tm_mon += 1;

    BCD[0] = (SMLK_UINT8)tm.tm_year;
    BCD[1] = (SMLK_UINT8)tm.tm_mon;
    BCD[2] = (SMLK_UINT8)tm.tm_mday;
    BCD[3] = (SMLK_UINT8)tm.tm_hour;
    BCD[4] = (SMLK_UINT8)tm.tm_min;
    BCD[5] = (SMLK_UINT8)tm.tm_sec;
}

MessageEngineDpfScrInfo1239::MessageEngineDpfScrInfo1239()
{
}

MessageEngineDpfScrInfo1239::MessageEngineDpfScrInfo1239(const MessageEngineDpfScrInfo1239 &other)
    : MessageEngineInfo(other)
{
}

MessageEngineDpfScrInfo1239::~MessageEngineDpfScrInfo1239()
{
}

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
SMLK_UINT32 MessageEngineDpfScrInfo1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageEngineDpfScrInfo1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);

    PUSH_AR(t_timestamp);
    PUSH_U2(m_tachograph_vehicle_speed.Valid() ? m_tachograph_vehicle_speed.Encoded() : 0xFFFF);
    PUSH_U1(m_barometric_pressure.Valid() ? m_barometric_pressure.Encoded() : 0xFF);
    PUSH_U1(m_actual_engine_percent_torque.Valid() ? m_actual_engine_percent_torque.Encoded() : 0xFF);
    PUSH_U1(m_nominal_friction_percent_torque.Valid() ? m_nominal_friction_percent_torque.Encoded() : 0xFF);
    PUSH_U2(m_engine_speed.Valid() ? m_engine_speed.Encoded() : 0xFFFF);
    PUSH_U2(m_engine_fuel_rate.Valid() ? m_engine_fuel_rate.Encoded() : 0xFFFF);
    PUSH_U2(m_aftertreatment_1_intake_nox.Valid() ? m_aftertreatment_1_intake_nox.Encoded() : 0xFFFF);
    PUSH_U2(m_aftertreatment_1_outlet_nox.Valid() ? m_aftertreatment_1_outlet_nox.Encoded() : 0xFFFF);
    PUSH_U1(m_catalyst_tank_level.Valid() ? m_catalyst_tank_level.Encoded() : 0xFF);
    PUSH_U2(m_engine_inlet_air_mass_flow_rate.Valid() ? m_engine_inlet_air_mass_flow_rate.Encoded() : 0xFFFF);
    PUSH_U2(m_aftertreatment_1_scr_intake_temperature.Valid() ? m_aftertreatment_1_scr_intake_temperature.Encoded() : 0xFFFF);
    PUSH_U2(m_aftertreatment_1_scr_outlet_temperature.Valid() ? m_aftertreatment_1_scr_outlet_temperature.Encoded() : 0xFFFF);
    PUSH_U2(m_aftertreatment_1_dpf_differential_pressure.Valid() ? m_aftertreatment_1_dpf_differential_pressure.Encoded() : 0xFFFF);
    PUSH_U1(m_engine_coolant_temperature.Valid() ? m_engine_coolant_temperature.Encoded() : 0xFF);
    PUSH_U1(m_fuel_level.Valid() ? m_fuel_level.Encoded() : 0xFF);
    PUSH_U1(m_located_status.Valid() ? m_located_status.Encoded() : 0xFF);
    PUSH_U4(m_longitude.Valid() ? m_longitude.Encoded() : 0xFFFFFFFF);
    PUSH_U4(m_latitude.Valid() ? m_latitude.Encoded() : 0xFFFFFFFF);
    PUSH_U4(m_total_vehicle_distance.Valid() ? m_total_vehicle_distance.Encoded() : 0xFFFFFFFF);

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
SMLK_UINT32 MessageEngineDpfScrInfo1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
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
std::size_t MessageEngineDpfScrInfo1239::EncodedSize() const
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
std::shared_ptr<IMessage> MessageEngineDpfScrInfo1239::Duplicate() const
{
    return std::make_shared<MessageEngineDpfScrInfo1239>(*this);
}


MessageEngineTwcInfo1239::MessageEngineTwcInfo1239()
{
}

MessageEngineTwcInfo1239::MessageEngineTwcInfo1239(const MessageEngineTwcInfo1239 &other)
    : MessageEngineInfo(other)
{
}

MessageEngineTwcInfo1239::~MessageEngineTwcInfo1239()
{
}

SMLK_UINT32 MessageEngineTwcInfo1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageEngineTwcInfo1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);
    PUSH_AR(t_timestamp);

#ifdef _CERT_HJ_1239
    PUSH_U2(m_tachograph_vehicle_speed.Valid() ? m_tachograph_vehicle_speed.Encoded() : 0x6400);
    PUSH_U1(m_barometric_pressure.Valid() ? m_barometric_pressure.Encoded() : 0x04);
    PUSH_U1(m_actual_engine_percent_torque.Valid() ? m_actual_engine_percent_torque.Encoded() : 0x7d);
    PUSH_U1(m_nominal_friction_percent_torque.Valid() ? m_nominal_friction_percent_torque.Encoded() : 0x7d);
    PUSH_U2(m_engine_speed.Valid() ? m_engine_speed.Encoded() : 0x1f40);
    PUSH_U2(m_engine_fuel_rate.Valid() ? m_engine_fuel_rate.Encoded() : 0x0190);
    PUSH_U2(m_twc_upstream_o2_sensor.Valid() ? m_twc_upstream_o2_sensor.Encoded() : 0xc01c);
    PUSH_U1(m_twc_downstream_o2_sensor.Valid() ? m_twc_downstream_o2_sensor.Encoded() : 0xc8);
    PUSH_U2(m_engine_inlet_air_mass_flow_rate.Valid() ? m_engine_inlet_air_mass_flow_rate.Encoded() : 0x00c8);
    PUSH_U1(m_engine_coolant_temperature.Valid() ? m_engine_coolant_temperature.Encoded() : 0x37);
    PUSH_U2(m_twc_temperature_sensor.Valid() ? m_twc_temperature_sensor.Encoded() : 0x2360);
    PUSH_U1(0x00);
    PUSH_U4(0x0714d938);
    PUSH_U4(0x01e6fda4);
    PUSH_U4(m_total_vehicle_distance.Valid() ? m_total_vehicle_distance.Encoded() : 0x000186a0);
#else 
    PUSH_U2(m_tachograph_vehicle_speed.Valid() ? m_tachograph_vehicle_speed.Encoded() : 0xFFFF);
    PUSH_U1(m_barometric_pressure.Valid() ? m_barometric_pressure.Encoded() : 0xFF);
    PUSH_U1(m_actual_engine_percent_torque.Valid() ? m_actual_engine_percent_torque.Encoded() : 0xFF);
    PUSH_U1(m_nominal_friction_percent_torque.Valid() ? m_nominal_friction_percent_torque.Encoded() : 0xFF);
    PUSH_U2(m_engine_speed.Valid() ? m_engine_speed.Encoded() : 0xFFFF);
    PUSH_U2(m_engine_fuel_rate.Valid() ? m_engine_fuel_rate.Encoded() : 0xFFFF);
    PUSH_U2(m_twc_upstream_o2_sensor.Valid() ? m_twc_upstream_o2_sensor.Encoded() : 0xFFFF);
    PUSH_U1(m_twc_downstream_o2_sensor.Valid() ? m_twc_downstream_o2_sensor.Encoded() : 0xFF);
    PUSH_U2(m_engine_inlet_air_mass_flow_rate.Valid() ? m_engine_inlet_air_mass_flow_rate.Encoded() : 0xFFFF);
    PUSH_U1(m_engine_coolant_temperature.Valid() ? m_engine_coolant_temperature.Encoded() : 0xFF);
    PUSH_U2(m_twc_temperature_sensor.Valid() ? m_twc_temperature_sensor.Encoded() : 0xFFFF);
    PUSH_U1(m_located_status.Valid() ? m_located_status.Encoded() : 0xFF);
    PUSH_U4(m_longitude.Valid() ? m_longitude.Encoded() : 0xFFFFFFFF);
    PUSH_U4(m_latitude.Valid() ? m_latitude.Encoded() : 0xFFFFFFFF);
    PUSH_U4(m_total_vehicle_distance.Valid() ? m_total_vehicle_distance.Encoded() : 0xFFFFFFFF);
#endif

    return SL_SUCCESS;
}

SMLK_UINT32 MessageEngineTwcInfo1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
    return SL_SUCCESS;
}

std::size_t MessageEngineTwcInfo1239::EncodedSize() const
{
    return sizeof(GB_17691_EngineDataStream);
}

std::shared_ptr<IMessage> MessageEngineTwcInfo1239::Duplicate() const
{
    return std::make_shared<MessageEngineTwcInfo1239>(*this);
}


/**
 * @brief Construct a new MessageEngineTwcNOxInfo1239:: MessageEngineTwcNOxInfo1239 object
 * 
 */
MessageEngineTwcNOxInfo1239::MessageEngineTwcNOxInfo1239()
{
}

MessageEngineTwcNOxInfo1239::MessageEngineTwcNOxInfo1239(const MessageEngineTwcNOxInfo1239 &other)
    : MessageEngineInfo(other)
{
}

MessageEngineTwcNOxInfo1239::~MessageEngineTwcNOxInfo1239()
{
}

SMLK_UINT32 MessageEngineTwcNOxInfo1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageEngineTwcNOxInfo1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);
    PUSH_AR(t_timestamp);

#ifdef _CERT_HJ_1239
    PUSH_U2(m_tachograph_vehicle_speed.Valid() ? m_tachograph_vehicle_speed.Encoded() : 0x6400);
    PUSH_U1(m_barometric_pressure.Valid() ? m_barometric_pressure.Encoded() : 0x04);
    PUSH_U1(m_actual_engine_percent_torque.Valid() ? m_actual_engine_percent_torque.Encoded() : 0x7d);
    PUSH_U1(m_nominal_friction_percent_torque.Valid() ? m_nominal_friction_percent_torque.Encoded() : 0x7d);
    PUSH_U2(m_engine_speed.Valid() ? m_engine_speed.Encoded() : 0x1f40);
    PUSH_U2(m_engine_fuel_rate.Valid() ? m_engine_fuel_rate.Encoded() : 0x0190);
    PUSH_U2(m_twc_upstream_o2_sensor.Valid() ? m_twc_upstream_o2_sensor.Encoded() : 0xc01c);
    PUSH_U1(m_twc_downstream_o2_sensor.Valid() ? m_twc_downstream_o2_sensor.Encoded() : 0xc8);
    PUSH_U2(m_twc_downstream_nox_sensor.Valid() ? m_twc_downstream_nox_sensor.Encoded() : 0x2ee0);
    PUSH_U2(m_engine_inlet_air_mass_flow_rate.Valid() ? m_engine_inlet_air_mass_flow_rate.Encoded() : 0x00c8);
    PUSH_U1(m_engine_coolant_temperature.Valid() ? m_engine_coolant_temperature.Encoded() : 0x37);
    PUSH_U2(m_twc_temperature_sensor.Valid() ? m_twc_temperature_sensor.Encoded() : 0x2360);
    PUSH_U1(0x00);
    PUSH_U4(0x0714d938);
    PUSH_U4(0x01e6fda4);
    PUSH_U4(m_total_vehicle_distance.Valid() ? m_total_vehicle_distance.Encoded() : 0x000186a0);
#else 
    PUSH_U2(m_tachograph_vehicle_speed.Valid() ? m_tachograph_vehicle_speed.Encoded() : 0xFFFF);
    PUSH_U1(m_barometric_pressure.Valid() ? m_barometric_pressure.Encoded() : 0xFF);
    PUSH_U1(m_actual_engine_percent_torque.Valid() ? m_actual_engine_percent_torque.Encoded() : 0xFF);
    PUSH_U1(m_nominal_friction_percent_torque.Valid() ? m_nominal_friction_percent_torque.Encoded() : 0xFF);
    PUSH_U2(m_engine_speed.Valid() ? m_engine_speed.Encoded() : 0xFFFF);
    PUSH_U2(m_engine_fuel_rate.Valid() ? m_engine_fuel_rate.Encoded() : 0xFFFF);
    PUSH_U2(m_twc_upstream_o2_sensor.Valid() ? m_twc_upstream_o2_sensor.Encoded() : 0xFFFF);
    PUSH_U1(m_twc_downstream_o2_sensor.Valid() ? m_twc_downstream_o2_sensor.Encoded() : 0xFF);
    PUSH_U2(m_twc_downstream_nox_sensor.Valid() ? m_twc_downstream_nox_sensor.Encoded() : 0xFFFF);
    PUSH_U2(m_engine_inlet_air_mass_flow_rate.Valid() ? m_engine_inlet_air_mass_flow_rate.Encoded() : 0xFFFF);
    PUSH_U1(m_engine_coolant_temperature.Valid() ? m_engine_coolant_temperature.Encoded() : 0xFF);
    PUSH_U2(m_twc_temperature_sensor.Valid() ? m_twc_temperature_sensor.Encoded() : 0xFFFF);
    PUSH_U1(m_located_status.Valid() ? m_located_status.Encoded() : 0xFF);
    PUSH_U4(m_longitude.Valid() ? m_longitude.Encoded() : 0xFFFFFFFF);
    PUSH_U4(m_latitude.Valid() ? m_latitude.Encoded() : 0xFFFFFFFF);
    PUSH_U4(m_total_vehicle_distance.Valid() ? m_total_vehicle_distance.Encoded() : 0xFFFFFFFF);
#endif

    return SL_SUCCESS;
}

SMLK_UINT32 MessageEngineTwcNOxInfo1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
    return SL_SUCCESS;
}

std::size_t MessageEngineTwcNOxInfo1239::EncodedSize() const
{
    return sizeof(GB_17691_EngineDataStream);
}

std::shared_ptr<IMessage> MessageEngineTwcNOxInfo1239::Duplicate() const
{
    return std::make_shared<MessageEngineTwcNOxInfo1239>(*this);
}


MessageEngineHevInfo1239::MessageEngineHevInfo1239()
{
}

MessageEngineHevInfo1239::MessageEngineHevInfo1239(const MessageEngineHevInfo1239 &other)
    : MessageEngineHevInfo(other)
{
}

MessageEngineHevInfo1239::~MessageEngineHevInfo1239()
{
}

SMLK_UINT32 MessageEngineHevInfo1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageEngineHevInfo1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);
    PUSH_AR(t_timestamp);

#ifdef _CERT_HJ_1239
    PUSH_U2(m_motor_speed.Valid() ? m_motor_speed.Encoded() : 0xFFFF);
    PUSH_U1(m_drive_ins_power_per.Valid() ? m_drive_ins_power_per.Encoded() : 0xFFFF);
    PUSH_U2(m_power_bat_volt.Valid() ? m_power_bat_volt.Encoded() : 0xFFFF);
    PUSH_U2(m_power_bat_curr.Valid() ? m_power_bat_curr.Encoded() : 0xFFFF);
    PUSH_U1(m_power_bat_soc.Valid() ? m_power_bat_soc.Encoded() : 0xFFFF);
#else 
    PUSH_U2(m_motor_speed.Valid() ? m_motor_speed.Encoded() : 0xFFFF);
    PUSH_U1(m_drive_ins_power_per.Valid() ? m_drive_ins_power_per.Encoded() : 0xFFFF);
    PUSH_U2(m_power_bat_volt.Valid() ? m_power_bat_volt.Encoded() : 0xFFFF);
    PUSH_U2(m_power_bat_curr.Valid() ? m_power_bat_curr.Encoded() : 0xFFFF);
    PUSH_U1(m_power_bat_soc.Valid() ? m_power_bat_soc.Encoded() : 0xFFFF);
#endif

    return SL_SUCCESS;
}

SMLK_UINT32 MessageEngineHevInfo1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
    return SL_SUCCESS;
}

std::size_t MessageEngineHevInfo1239::EncodedSize() const
{
    return sizeof(GB_17691_EngineDataStream);
}

std::shared_ptr<IMessage> MessageEngineHevInfo1239::Duplicate() const
{
    return std::make_shared<MessageEngineHevInfo1239>(*this);
}

MessageOBDInfo1239::MessageOBDInfo1239()
{
}

MessageOBDInfo1239::MessageOBDInfo1239(const MessageOBDInfo1239 &other)
    : MessageOBDInfo(other)
{
}

MessageOBDInfo1239::~MessageOBDInfo1239()
{
}

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
SMLK_UINT32 MessageOBDInfo1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageOBDInfo1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);
    PUSH_AR(t_timestamp);

#ifdef _CERT_HJ_1239
    PUSH_U1(1);
    PUSH_U1(1);
    PUSH_U2(m_diag_support_status);
    PUSH_U2(m_diag_ready_status);

    SMLK_LOGI("MessageOBDInfo1239::Encode() t_vin %s", t_vin.c_str());
    data.insert(data.end(), t_vin.cbegin(), t_vin.cend());
    // std::string en_vin = "LFWSRXSJ6MAY00108";
    // data.insert(data.end(), en_vin.cbegin(), en_vin.cend());
    std::string m_scin = "123456789000000000";
    data.insert(data.end(), m_scin.cbegin(), m_scin.cend());
    std::string en_cvn = "123456789123456789";
    data.insert(data.end(), en_cvn.cbegin(), en_cvn.cend());
    PUSH_AR(m_iupr);

    PUSH_U1(1);
    std::string en_dtc = "2323";
    data.insert(data.end(), en_dtc.cbegin(), en_dtc.cend());
#else
    PUSH_U1(m_protocol);
    PUSH_U1(m_mil_status);
    PUSH_U2(m_diag_support_status);
    PUSH_U2(m_diag_ready_status);
    PUSH_AR(m_vin);
    PUSH_AR(m_scin);
    PUSH_AR(m_cvn);
    PUSH_AR(m_iupr);

    if (m_dtc != 0xFE)
    {
        PUSH_U1((SMLK_UINT8)m_dtcs.size());
        for (auto const &dtc : m_dtcs)
        {
            PUSH_U4(dtc);
        }
    }
    else
    {
        PUSH_U1(m_dtc);
    }
#endif

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
SMLK_UINT32 MessageOBDInfo1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
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
std::size_t MessageOBDInfo1239::EncodedSize() const
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
std::shared_ptr<IMessage> MessageOBDInfo1239::Duplicate() const
{
    return std::make_shared<MessageOBDInfo1239>(*this);
}

MessageReport1239::MessageReport1239()
{
}

MessageReport1239::MessageReport1239(const MessageReport1239 &other)
    : MessageReport(other)
{
}

MessageReport1239::~MessageReport1239()
{
}

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
SMLK_UINT32 MessageReport1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageReport1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);

    PUSH_AR(t_timestamp);
    PUSH_U2(m_seq);

    for (auto const &inf : m_infos)
    {
        std::vector<SMLK_UINT8> body;
        std::get<1>(inf)->Encode(body);
        // append message type
        PUSH_U1(std::get<0>(inf));
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
SMLK_UINT32 MessageReport1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
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
std::size_t MessageReport1239::EncodedSize() const
{
    std::size_t sz = sizeof(m_timestamp) + sizeof(m_seq);

    for (auto const &tuple : m_infos)
    {
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
std::shared_ptr<IMessage> MessageReport1239::Duplicate() const
{
    return std::make_shared<MessageReport1239>(*this);
}

MessageVehicleLogin1239::MessageVehicleLogin1239()
{
}

MessageVehicleLogin1239::MessageVehicleLogin1239(const MessageVehicleLogin1239 &other)
    : MessageVehicleLogin(other)
{
}

MessageVehicleLogin1239::~MessageVehicleLogin1239()
{
}

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
SMLK_UINT32 MessageVehicleLogin1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageVehicleLogin1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);

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
SMLK_UINT32 MessageVehicleLogin1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
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
std::size_t MessageVehicleLogin1239::EncodedSize() const
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
std::shared_ptr<IMessage> MessageVehicleLogin1239::Duplicate() const
{
    return std::make_shared<MessageVehicleLogin1239>(*this);
}

MessageVehicleLogout1239::MessageVehicleLogout1239()
{
}

MessageVehicleLogout1239::MessageVehicleLogout1239(const MessageVehicleLogout1239 &other)
    : MessageVehicleLogout(other)
{
}

MessageVehicleLogout1239::~MessageVehicleLogout1239()
{
}

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
SMLK_UINT32 MessageVehicleLogout1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageVehicleLogout1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);

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
SMLK_UINT32 MessageVehicleLogout1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
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
std::size_t MessageVehicleLogout1239::EncodedSize() const
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
std::shared_ptr<IMessage> MessageVehicleLogout1239::Duplicate() const
{
    return std::make_shared<MessageVehicleLogout1239>(*this);
}

MessageVehicleActivate1239::MessageVehicleActivate1239()
{
}

MessageVehicleActivate1239::MessageVehicleActivate1239(const MessageVehicleActivate1239 &other)
    : MessageVehicleActivate(other)
{
}

MessageVehicleActivate1239::~MessageVehicleActivate1239()
{
}

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
SMLK_UINT32 MessageVehicleActivate1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageVehicleActivate1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);

    PUSH_AR(t_timestamp);
    data.insert(data.end(), m_security_chip_id.cbegin(), m_security_chip_id.cend());
    data.insert(data.end(), m_public_key.cbegin(), m_public_key.cend());
    // std::string en_vin = "LFNFVUNX0K1E30368";
    // data.insert(data.end(), en_vin.cbegin(), en_vin.cend());
    data.insert(data.end(), m_vin.cbegin(), m_vin.cend());

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
SMLK_UINT32 MessageVehicleActivate1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
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
std::size_t MessageVehicleActivate1239::EncodedSize() const
{
    // return sizeof(m_timestamp) + m_security_chip_id.size() + m_public_key.size() + m_vin.size();
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
std::shared_ptr<IMessage> MessageVehicleActivate1239::Duplicate() const
{
    return std::make_shared<MessageVehicleActivate1239>(*this);
}

MessageTamperAlarm1239::MessageTamperAlarm1239()
{
}

MessageTamperAlarm1239::MessageTamperAlarm1239(const MessageTamperAlarm1239 &other)
    : MessageTamperAlarm(other)
{
}

MessageTamperAlarm1239::~MessageTamperAlarm1239()
{
}

SMLK_UINT32 MessageTamperAlarm1239::Encode(std::vector<SMLK_UINT8> &data) const
{
    data.clear();

    SMLK_BCD t_timestamp[6] = {0};
    TimeStamp2BCD((std::time_t)m_timestamp.Val(), t_timestamp, sizeof(t_timestamp));
    SMLK_LOGI("MessageTamperAlarm1239::Encode() time %d-%d-%d %d:%d:%d  ", t_timestamp[0], t_timestamp[1], t_timestamp[2], t_timestamp[3], t_timestamp[4], t_timestamp[5]);

    PUSH_AR(t_timestamp);
    PUSH_U2(m_seq);
    PUSH_U1(m_located_status.Valid() ? m_located_status.Encoded() : 0xFF);
    PUSH_U4(m_longitude.Valid() ? m_longitude.Encoded() : 0xFFFFFFFF);
    PUSH_U4(m_latitude.Valid() ? m_latitude.Encoded() : 0xFFFFFFFF);

    return SL_SUCCESS;
}

SMLK_UINT32 MessageTamperAlarm1239::Decode(IN std::vector<SMLK_UINT8> &data)
{
    return SL_SUCCESS;
}

std::size_t MessageTamperAlarm1239::EncodedSize() const
{
    return sizeof(GB_17691_EngineDataStream);
}

std::shared_ptr<IMessage> MessageTamperAlarm1239::Duplicate() const
{
    return std::make_shared<MessageTamperAlarm1239>(*this);
}

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
#pragma once
#include <string>
#include <tuple>
#include <vector>
#include <memory>
#include <message_def_17691.h>
#include <message_inf.h>
#include <report_signals.h>
#include <sl_errors.h>

namespace smartlink {

class MessageEngineInfo : public IMessage {
public:
    MessageEngineInfo();
    MessageEngineInfo(const MessageEngineInfo &other);
    ~MessageEngineInfo();

    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) const {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::shared_ptr<IMessage>   Duplicate() const;
    virtual std::size_t                 EncodedSize() const {return 0;}

    MessageEngineInfo & operator=(const MessageEngineInfo &other);

public:
    vehicle::SignalVariant  m_timestamp;
    vehicle::SignalVariant  m_tachograph_vehicle_speed;
    vehicle::SignalVariant  m_barometric_pressure;
    vehicle::SignalVariant  m_actual_engine_percent_torque;
    vehicle::SignalVariant  m_nominal_friction_percent_torque;
    vehicle::SignalVariant  m_engine_speed;
    vehicle::SignalVariant  m_engine_fuel_rate;
    vehicle::SignalVariant  m_aftertreatment_1_intake_nox;
    vehicle::SignalVariant  m_aftertreatment_1_outlet_nox;
    vehicle::SignalVariant  m_catalyst_tank_level;
    vehicle::SignalVariant  m_engine_inlet_air_mass_flow_rate;
    vehicle::SignalVariant  m_aftertreatment_1_scr_intake_temperature;
    vehicle::SignalVariant  m_aftertreatment_1_scr_outlet_temperature;
    vehicle::SignalVariant  m_aftertreatment_1_dpf_differential_pressure;
    vehicle::SignalVariant  m_engine_coolant_temperature;
    vehicle::SignalVariant  m_fuel_level;
    vehicle::SignalVariant  m_twc_upstream_o2_sensor;
    vehicle::SignalVariant  m_twc_downstream_o2_sensor;
    vehicle::SignalVariant  m_twc_downstream_nox_sensor;
    vehicle::SignalVariant  m_twc_temperature_sensor;
    vehicle::SignalVariant  m_total_vehicle_distance;
    vehicle::SignalVariant  m_located_status;
    vehicle::SignalVariant  m_longitude;
    vehicle::SignalVariant  m_latitude;
};  // class MessageEngineInfo


class MessageEngineHevInfo : public IMessage {
public:
    MessageEngineHevInfo();
    MessageEngineHevInfo(const MessageEngineHevInfo &other);
    ~MessageEngineHevInfo();

    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) const {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::shared_ptr<IMessage>   Duplicate() const;
    virtual std::size_t                 EncodedSize() const {return 0;}

    MessageEngineHevInfo & operator=(const MessageEngineHevInfo &other);

public:
    vehicle::SignalVariant  m_timestamp;
    vehicle::SignalVariant  m_motor_speed;
    vehicle::SignalVariant  m_drive_ins_power_per;
    vehicle::SignalVariant  m_power_bat_volt;
    vehicle::SignalVariant  m_power_bat_curr;
    vehicle::SignalVariant  m_power_bat_soc;
};  // class MessageEngineHevInfo


class MessageOBDInfo : public IMessage {
public:
    MessageOBDInfo();
    MessageOBDInfo(const MessageOBDInfo &other);
    ~MessageOBDInfo();

    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) const {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::shared_ptr<IMessage>   Duplicate() const;
    virtual std::size_t                 EncodedSize() const {return 0;};

    MessageOBDInfo & operator=(const MessageOBDInfo &other);
public:
    vehicle::SignalVariant  m_timestamp;
    SMLK_UINT8              m_protocol;
    SMLK_UINT8              m_mil_status;
    SMLK_UINT16             m_diag_support_status;
    SMLK_UINT16             m_diag_ready_status;
    SMLK_UINT8              m_vin[M_GB_17691_SZ_VIN];
    SMLK_UINT8              m_scin[M_GB_17691_SZ_SCIN];
    SMLK_UINT8              m_cvn[M_GB_17691_SZ_CVN];
    SMLK_UINT8              m_iupr[M_GB_17691_SZ_IUPR];
    SMLK_UINT8              m_dtc;
    std::vector<SMLK_UINT32>    m_dtcs;
};  // class MessageOBDInfo


class MessageReport : public IMessage {
public:
    enum InfoType : SMLK_UINT8 {
        OBD             = M_GB_17691_MESSAGE_OBD,
        Stream          = M_GB_17691_MESSAGE_STREAM,

        Stream_NG       = M_HJ_1239_MESSAGE_STREAM_TWC,
        Stream_HEV      = M_HJ_1239_MESSAGE_STREAM_HEV,
        Stream_NG_NOX   = M_HJ_1239_MESSAGE_STREAM_TWC_NOX,

    };
public:
    MessageReport();
    MessageReport(const MessageReport &other);
    ~MessageReport();

    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) const {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::shared_ptr<IMessage>   Duplicate() const;
    virtual std::size_t                 EncodedSize() const {return 0;};

    MessageReport & operator=(const MessageReport &other);

public:
    vehicle::SignalVariant  m_timestamp;
    SMLK_UINT16             m_seq;
    std::vector<std::tuple<InfoType, std::shared_ptr<IMessage>>>    m_infos;
};  // class MessageReport


class MessageVehicleLogin : public IMessage {
public:
    MessageVehicleLogin();
    MessageVehicleLogin(const MessageVehicleLogin &other);
    ~MessageVehicleLogin();

    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) const {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::shared_ptr<IMessage>   Duplicate() const;
    virtual std::size_t                 EncodedSize() const {return 0;};

    MessageVehicleLogin & operator=(const MessageVehicleLogin &other);
public:
    vehicle::SignalVariant  m_timestamp;
    SMLK_UINT16             m_seq;
    std::string             m_iccid;
};  // MessageVehicleLogin


class MessageVehicleLogout : public IMessage {
public:
    MessageVehicleLogout();
    MessageVehicleLogout(const MessageVehicleLogout &other);
    ~MessageVehicleLogout();

    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) const {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::shared_ptr<IMessage>   Duplicate() const;
    virtual std::size_t                 EncodedSize() const {return 0;};

    MessageVehicleLogout & operator=(const MessageVehicleLogout &other);
public:
    vehicle::SignalVariant  m_timestamp;
    SMLK_UINT16             m_seq;
};  // MessageVehicleLogout


class MessageVehicleActivate : public IMessage {
public:
    MessageVehicleActivate();
    MessageVehicleActivate(const MessageVehicleActivate &other);
    ~MessageVehicleActivate();

    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) const {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::shared_ptr<IMessage>   Duplicate() const;
    virtual std::size_t                 EncodedSize() const {return 0;};

    MessageVehicleActivate & operator=(const MessageVehicleActivate &other);
public:
    vehicle::SignalVariant  m_timestamp;
    std::string             m_security_chip_id;
    std::string             m_public_key;
    std::string             m_vin;
};  // MessageVehicleActivate


class MessageTamperAlarm : public IMessage {
public:
    MessageTamperAlarm();
    MessageTamperAlarm(const MessageTamperAlarm &other);
    ~MessageTamperAlarm();

    virtual SMLK_UINT32     Encode(std::vector<SMLK_UINT8> &) const {return SL_SUCCESS;}
    virtual SMLK_UINT32     Decode(IN std::vector<SMLK_UINT8> &) {return SL_SUCCESS;}
    virtual std::shared_ptr<IMessage>   Duplicate() const;
    virtual std::size_t                 EncodedSize() const {return 0;};

    MessageTamperAlarm & operator=(const MessageTamperAlarm &other);
public:
    vehicle::SignalVariant  m_timestamp;
    SMLK_UINT16             m_seq;
    vehicle::SignalVariant  m_located_status;
    vehicle::SignalVariant  m_longitude;
    vehicle::SignalVariant  m_latitude;
};  // MessageTamperAlarm

};  // namespace smartlink

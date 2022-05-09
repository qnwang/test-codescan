#include <cstring>
#include <message_body.h>

using namespace smartlink;

MessageEngineInfo::MessageEngineInfo()
    : m_timestamp(signal::TimeStamp)
    , m_tachograph_vehicle_speed(signal::TachographVehicleSpeed)
    , m_barometric_pressure(signal::BarometricPressure)
    , m_actual_engine_percent_torque(signal::ActualEnginePercentTorque)
    , m_nominal_friction_percent_torque(signal::NominalFrictionPercentTorque)
    , m_engine_speed(signal::EngineSpeed)
    , m_engine_fuel_rate(signal::EngineFuelRate)
    , m_aftertreatment_1_intake_nox(signal::Aftertreatment1IntakeNOx)
    , m_aftertreatment_1_outlet_nox(signal::Aftertreatment1OutletNOx)
    , m_catalyst_tank_level(signal::CatalystTankLevel)
    , m_engine_inlet_air_mass_flow_rate(signal::EngineInletAirMassFlowRate)
    , m_aftertreatment_1_scr_intake_temperature(signal::Aftertreatment1SCRIntakeTemperature)
    , m_aftertreatment_1_scr_outlet_temperature(signal::Aftertreatment1SCROutletTemperature)
    , m_aftertreatment_1_dpf_differential_pressure(signal::Aftertreatment1DPFDifferentialPressure)
    , m_engine_coolant_temperature(signal::EngineCoolantTemperature)
    , m_fuel_level(signal::FuelLevel)
    , m_twc_upstream_o2_sensor(signal::TWCUpstreamO2Sensor)
    , m_twc_downstream_o2_sensor(signal::TWCDownstreamO2Sensor)
    , m_twc_downstream_nox_sensor(signal::TWCDownstreamNOxSensor)
    , m_twc_temperature_sensor(signal::TWCTempSensor)
    , m_total_vehicle_distance(signal::TotalVehicleDistance)
    , m_located_status(signal::LocatedStatus)
    , m_longitude(signal::Longitude)
    , m_latitude(signal::Latitude)
{}

MessageEngineInfo::MessageEngineInfo(const MessageEngineInfo &other)
    : m_timestamp(other.m_timestamp)
    , m_tachograph_vehicle_speed(other.m_tachograph_vehicle_speed)
    , m_barometric_pressure(other.m_barometric_pressure)
    , m_actual_engine_percent_torque(other.m_actual_engine_percent_torque)
    , m_nominal_friction_percent_torque(other.m_nominal_friction_percent_torque)
    , m_engine_speed(other.m_engine_speed)
    , m_engine_fuel_rate(other.m_engine_fuel_rate)
    , m_aftertreatment_1_intake_nox(other.m_aftertreatment_1_intake_nox)
    , m_aftertreatment_1_outlet_nox(other.m_aftertreatment_1_outlet_nox)
    , m_catalyst_tank_level(other.m_catalyst_tank_level)
    , m_engine_inlet_air_mass_flow_rate(other.m_engine_inlet_air_mass_flow_rate)
    , m_aftertreatment_1_scr_intake_temperature(other.m_aftertreatment_1_scr_intake_temperature)
    , m_aftertreatment_1_scr_outlet_temperature(other.m_aftertreatment_1_scr_outlet_temperature)
    , m_aftertreatment_1_dpf_differential_pressure(other.m_aftertreatment_1_dpf_differential_pressure)
    , m_engine_coolant_temperature(other.m_engine_coolant_temperature)
    , m_fuel_level(other.m_fuel_level)
    , m_twc_upstream_o2_sensor(other.m_twc_upstream_o2_sensor)
    , m_twc_downstream_o2_sensor(other.m_twc_downstream_o2_sensor)
    , m_twc_downstream_nox_sensor(other.m_twc_downstream_nox_sensor)
    , m_twc_temperature_sensor(other.m_twc_temperature_sensor)
    , m_total_vehicle_distance(other.m_total_vehicle_distance)
    , m_located_status(other.m_located_status)
    , m_longitude(other.m_longitude)
    , m_latitude(other.m_latitude)
{}

MessageEngineInfo::~MessageEngineInfo()
{}

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
std::shared_ptr<IMessage> MessageEngineInfo::Duplicate() const
{
    return std::make_shared<MessageEngineInfo>(*this);
}

/******************************************************************************
 * NAME: operator=
 *
 * DESCRIPTION:     assign operator
 *
 * PARAMETERS:
 *  other           the other instance which this will be assigned to
 * 
 * RETURN:
 *                  the reference of this instance
 *
 * NOTE:
 *****************************************************************************/
MessageEngineInfo & MessageEngineInfo::operator=(const MessageEngineInfo &other)
{
    if ( this != &other ) {
        m_timestamp                                                             = other.m_timestamp;
        m_tachograph_vehicle_speed                                              = other.m_tachograph_vehicle_speed;
        m_barometric_pressure                                                   = other.m_barometric_pressure;
        m_actual_engine_percent_torque                                          = other.m_actual_engine_percent_torque;
        m_nominal_friction_percent_torque                                       = other.m_nominal_friction_percent_torque;
        m_engine_speed                                                          = other.m_engine_speed;
        m_engine_fuel_rate                                                      = other.m_engine_fuel_rate;
        m_aftertreatment_1_intake_nox                                           = other.m_aftertreatment_1_intake_nox;
        m_aftertreatment_1_outlet_nox                                           = other.m_aftertreatment_1_outlet_nox;
        m_catalyst_tank_level                                                   = other.m_catalyst_tank_level;
        m_engine_inlet_air_mass_flow_rate                                       = other.m_engine_inlet_air_mass_flow_rate;
        m_aftertreatment_1_scr_intake_temperature                               = other.m_aftertreatment_1_scr_intake_temperature;
        m_aftertreatment_1_scr_outlet_temperature                               = other.m_aftertreatment_1_scr_outlet_temperature;
        m_aftertreatment_1_dpf_differential_pressure                            = other.m_aftertreatment_1_dpf_differential_pressure;
        m_engine_coolant_temperature                                            = other.m_engine_coolant_temperature;
        m_fuel_level                                                            = other.m_fuel_level;
        m_twc_upstream_o2_sensor                                                = other.m_twc_upstream_o2_sensor;
        m_twc_downstream_o2_sensor                                              = other.m_twc_downstream_o2_sensor;
        m_twc_downstream_nox_sensor                                             = other.m_twc_downstream_nox_sensor;        
        m_twc_temperature_sensor                                                = other.m_twc_temperature_sensor;        
        m_total_vehicle_distance                                                = other.m_total_vehicle_distance;
        m_located_status                                                        = other.m_located_status;
        m_longitude                                                             = other.m_longitude;
        m_latitude                                                              = other.m_latitude;
    }
    return *this;
}


MessageEngineHevInfo::MessageEngineHevInfo()
    : m_timestamp(signal::TimeStamp)
    , m_motor_speed(signal::MotorSpeed)
    , m_drive_ins_power_per(signal::DriveInsPowerPer)
    , m_power_bat_volt(signal::PowerBatVolt)
    , m_power_bat_curr(signal::PowerBatCurr)
    , m_power_bat_soc(signal::PowerBatSOC)
{}

MessageEngineHevInfo::MessageEngineHevInfo(const MessageEngineHevInfo &other)
    : m_timestamp(other.m_timestamp)
    , m_motor_speed(other.m_motor_speed)
    , m_drive_ins_power_per(other.m_drive_ins_power_per)
    , m_power_bat_volt(other.m_power_bat_volt)
    , m_power_bat_curr(other.m_power_bat_curr)
    , m_power_bat_soc(other.m_power_bat_soc)
{}

MessageEngineHevInfo::~MessageEngineHevInfo()
{}

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
std::shared_ptr<IMessage> MessageEngineHevInfo::Duplicate() const
{
    return std::make_shared<MessageEngineHevInfo>(*this);
}

/******************************************************************************
 * NAME: operator=
 *
 * DESCRIPTION:     assign operator
 *
 * PARAMETERS:
 *  other           the other instance which this will be assigned to
 * 
 * RETURN:
 *                  the reference of this instance
 *
 * NOTE:
 *****************************************************************************/
MessageEngineHevInfo & MessageEngineHevInfo::operator=(const MessageEngineHevInfo &other)
{
    if ( this != &other ) {
        m_timestamp                                                             = other.m_timestamp;
        m_motor_speed                                                           = other.m_motor_speed;
        m_drive_ins_power_per                                                   = other.m_drive_ins_power_per;
        m_power_bat_volt                                                        = other.m_power_bat_volt;
        m_power_bat_curr                                                        = other.m_power_bat_curr;
        m_power_bat_soc                                                         = other.m_power_bat_soc;

    }
    return *this;
}

MessageOBDInfo::MessageOBDInfo()
    : m_timestamp(signal::TimeStamp)
    , m_protocol(M_GB_17691_OBD_PROTOCOL_ISO15765)
    , m_mil_status(M_GB_17691_MIL_INVALID)
    , m_diag_support_status(0)
    , m_diag_ready_status(0)
    , m_vin{}
    , m_scin{}
    , m_cvn{}
    , m_iupr{}
    , m_dtc{}
{}

MessageOBDInfo::MessageOBDInfo(const MessageOBDInfo &other)
    : m_timestamp(other.m_timestamp)
    , m_protocol(other.m_protocol)
    , m_mil_status(other.m_mil_status)
    , m_diag_support_status(other.m_diag_support_status)
    , m_diag_ready_status(other.m_diag_ready_status)
    , m_vin{}
    , m_cvn{}
    , m_iupr{}
    , m_dtc{}
    , m_dtcs(other.m_dtcs)
{
    std::memcpy(m_vin, other.m_vin, sizeof(m_vin));
    std::memcpy(m_cvn, other.m_cvn, sizeof(m_cvn));
    std::memcpy(m_iupr, other.m_iupr, sizeof(m_iupr));
}

MessageOBDInfo::~MessageOBDInfo()
{}

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
std::shared_ptr<IMessage> MessageOBDInfo::Duplicate() const
{
    return std::make_shared<MessageOBDInfo>(*this);
}

/******************************************************************************
 * NAME: operator=
 *
 * DESCRIPTION:     assign operator
 *
 * PARAMETERS:
 *  other           the other instance which this will be assigned to
 * 
 * RETURN:
 *                  the reference of this instance
 *
 * NOTE:
 *****************************************************************************/
MessageOBDInfo & MessageOBDInfo::operator=(const MessageOBDInfo &other)
{
    if ( this != &other ) {
        m_timestamp             = other.m_timestamp;
        m_protocol              = other.m_protocol;
        m_mil_status            = other.m_mil_status;
        m_diag_support_status   = other.m_diag_support_status;
        m_diag_ready_status     = other.m_diag_ready_status;
        m_dtc                   = other.m_dtc;
        m_dtcs                  = other.m_dtcs;

        std::memcpy(m_vin,  other.m_vin,    sizeof(m_vin));
        std::memcpy(m_scin, other.m_scin,   sizeof(m_scin));
        std::memcpy(m_cvn,  other.m_cvn,    sizeof(m_cvn));
        std::memcpy(m_iupr, other.m_iupr,   sizeof(m_iupr));
    }

    return *this;
}

MessageReport::MessageReport()
    : m_timestamp(signal::TimeStamp)
    , m_seq(0)
{}

MessageReport::MessageReport(const MessageReport &other) 
    : m_timestamp(other.m_timestamp)
    , m_seq(other.m_seq)
    , m_infos(other.m_infos)
{
    for ( auto &tuple : m_infos ) {
        auto &msg = std::get<1>(tuple);
        msg = msg->Duplicate();
    }    
}

MessageReport::~MessageReport()
{}

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
std::shared_ptr<IMessage> MessageReport::Duplicate() const
{
    return std::make_shared<MessageReport>(*this);
}

/******************************************************************************
 * NAME: operator=
 *
 * DESCRIPTION:     assign operator
 *
 * PARAMETERS:
 *  other           the other instance which this will be assigned to
 * 
 * RETURN:
 *                  the reference of this instance
 *
 * NOTE:
 *****************************************************************************/
MessageReport & MessageReport::operator=(const MessageReport &other)
{
    if ( this != &other ) {
        m_timestamp = other.m_timestamp;
        m_infos = other.m_infos;
    }

    return *this;
}

MessageVehicleLogin::MessageVehicleLogin() 
    : m_timestamp(signal::TimeStamp)
    , m_seq(0)
    , m_iccid{}
{}

MessageVehicleLogin::MessageVehicleLogin(const MessageVehicleLogin &other) 
    : m_timestamp(other.m_timestamp)
    , m_seq(other.m_seq)
    , m_iccid(other.m_iccid)
{}

MessageVehicleLogin::~MessageVehicleLogin()
{}

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
std::shared_ptr<IMessage> MessageVehicleLogin::Duplicate() const
{
    return std::make_shared<MessageVehicleLogin>(*this);
}

/******************************************************************************
 * NAME: operator=
 *
 * DESCRIPTION:     assign operator
 *
 * PARAMETERS:
 *  other           the other instance which this will be assigned to
 * 
 * RETURN:
 *                  the reference of this instance
 *
 * NOTE:
 *****************************************************************************/
MessageVehicleLogin & MessageVehicleLogin::operator=(const MessageVehicleLogin &other)
{
    if ( this != &other ) {
        m_seq   = other.m_seq;
        m_iccid = other.m_iccid;
        m_timestamp = other.m_timestamp;
    }

    return *this;
}

MessageVehicleLogout::MessageVehicleLogout()
    : m_timestamp(signal::TimeStamp)
    , m_seq(0)
{}

MessageVehicleLogout::MessageVehicleLogout(const MessageVehicleLogout &other)
    : m_timestamp(other.m_timestamp)
    , m_seq(other.m_seq)
{}

MessageVehicleLogout::~MessageVehicleLogout()
{}

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
std::shared_ptr<IMessage> MessageVehicleLogout::Duplicate() const
{
    return std::make_shared<MessageVehicleLogout>(*this);
}

/******************************************************************************
 * NAME: operator=
 *
 * DESCRIPTION:     assign operator
 *
 * PARAMETERS:
 *  other           the other instance which this will be assigned to
 * 
 * RETURN:
 *                  the reference of this instance
 *
 * NOTE:
 *****************************************************************************/
MessageVehicleLogout & MessageVehicleLogout::operator=(const MessageVehicleLogout &other)
{
    if ( this != &other ) {
        m_seq   = other.m_seq;
        m_timestamp = other.m_timestamp;
    }

    return *this;
}


MessageVehicleActivate::MessageVehicleActivate()
    : m_timestamp(signal::TimeStamp)
    , m_security_chip_id{}
    , m_public_key{}
    , m_vin{}
{}

MessageVehicleActivate::MessageVehicleActivate(const MessageVehicleActivate &other)
    : m_timestamp(other.m_timestamp)
    , m_security_chip_id(other.m_security_chip_id)
    , m_public_key(other.m_public_key)
    ,  m_vin(other.m_vin)
{}

MessageVehicleActivate::~MessageVehicleActivate()
{}

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
std::shared_ptr<IMessage> MessageVehicleActivate::Duplicate() const
{
    return std::make_shared<MessageVehicleActivate>(*this);
}

/******************************************************************************
 * NAME: operator=
 *
 * DESCRIPTION:     assign operator
 *
 * PARAMETERS:
 *  other           the other instance which this will be assigned to
 * 
 * RETURN:
 *                  the reference of this instance
 *
 * NOTE:
 *****************************************************************************/
MessageVehicleActivate & MessageVehicleActivate::operator=(const MessageVehicleActivate &other)
{
    if ( this != &other ) {
        m_timestamp         = other.m_timestamp;
        m_security_chip_id  = other.m_security_chip_id;
        m_public_key        = other.m_public_key;
        m_vin               = other.m_vin;
    }

    return *this;
}

MessageTamperAlarm::MessageTamperAlarm()
    : m_timestamp(signal::TimeStamp)
    , m_seq(0)
    , m_located_status(signal::LocatedStatus)
    , m_longitude(signal::Longitude)
    , m_latitude(signal::Latitude)
{}

MessageTamperAlarm::MessageTamperAlarm(const MessageTamperAlarm &other)
    : m_timestamp(other.m_timestamp)
    , m_seq(other.m_seq)
    , m_located_status(other.m_located_status)
    , m_longitude(other.m_longitude)
    , m_latitude(other.m_latitude)
{}

MessageTamperAlarm::~MessageTamperAlarm()
{}

std::shared_ptr<IMessage> MessageTamperAlarm::Duplicate() const
{
    return std::make_shared<MessageTamperAlarm>(*this);
}

MessageTamperAlarm & MessageTamperAlarm::operator=(const MessageTamperAlarm &other)
{
    if ( this != &other ) {
        m_timestamp                                                             = other.m_timestamp;
        m_seq                                                                   = other.m_seq;
        m_located_status                                                        = other.m_located_status;
        m_longitude                                                             = other.m_longitude;
        m_latitude                                                              = other.m_latitude;
    }
    return *this;
}

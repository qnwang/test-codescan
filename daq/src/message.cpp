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
#include <memory>
#include <message.h>

using namespace smartlink;

const SMLK_UINT8 MessageLocation::EXT_MILAGE                = 0x01;
const SMLK_UINT8 MessageLocation::EXT_ENTER_LEAVE_AREA      = 0x12;
const SMLK_UINT8 MessageLocation::EXT_SIGNAL_STATUS         = 0x25;
const SMLK_UINT8 MessageLocation::EXT_RSSI                  = 0x30;
const SMLK_UINT8 MessageLocation::EXT_GNSS_SATELLITES       = 0x31;
const SMLK_UINT8 MessageLocation::EXT_E6                    = 0xE6;
const SMLK_UINT8 MessageLocation::EXT_E7                    = 0xE7;
const SMLK_UINT8 MessageLocation::EXT_EB                    = 0xEB;
const SMLK_UINT8 MessageLocation::EXT_EC                    = 0xEC;


LocationInfo::LocationInfo()
    : m_status(vehicle::g_vehicle_signals.find(vehicle::GNSSLocatedStatus)->second)
    , m_heading(vehicle::g_vehicle_signals.find(vehicle::GNSSHeading)->second)
    , m_latitude(vehicle::g_vehicle_signals.find(vehicle::GNSSLatitude)->second)
    , m_longitude(vehicle::g_vehicle_signals.find(vehicle::GNSSLongitude)->second)
    , m_altitude(vehicle::g_vehicle_signals.find(vehicle::GNSSAltitude)->second)
    , m_speed(vehicle::g_vehicle_signals.find(vehicle::GNSSSpeed)->second)
    , m_timestamp{vehicle::g_vehicle_signals.find(vehicle::GNSSTimeStamp)->second}
{}

LocationInfo::~LocationInfo()
{}

LocationInfo::LocationInfo(IN LocationInfo &other)
    : m_status(other.m_status)
    , m_heading(other.m_heading)
    , m_latitude(other.m_latitude)
    , m_longitude(other.m_longitude)
    , m_altitude(other.m_altitude)
    , m_speed(other.m_speed)
    , m_timestamp(other.m_timestamp)
{
}

LocationInfo & LocationInfo::operator=(IN LocationInfo &other)
{
    if ( this != &other ) {
        m_status    = other.m_status;
        m_heading   = other.m_heading;
        m_latitude  = other.m_latitude;
        m_longitude = other.m_longitude;
        m_altitude  = other.m_altitude;
        m_speed     = other.m_speed;
        m_timestamp = other.m_timestamp;
    }

    return *this;
}

RegionInfo::RegionInfo()
    : m_id(0)
    , m_type(0)
    , m_direction(0)
{}

RegionInfo::~RegionInfo()
{}

RegionInfo::RegionInfo(IN RegionInfo &other)
    : m_id(other.m_id)
    , m_type(other.m_type)
    , m_direction(other.m_direction)
{}

RegionInfo & RegionInfo::operator=(IN RegionInfo &other)
{
    if ( this != &other ) {
        m_id        = other.m_id;
        m_type      = other.m_type;
        m_direction = other.m_direction;
    }

    return *this;
}

TireInfo::TireInfo()
    : m_id(0)
    , m_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::TireTemperature)->second)
    , m_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::TirePressure)->second)
    , m_extend_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::TirePressureExtendedRange)->second)
{}

TireInfo::~TireInfo()
{}

TireInfo::TireInfo(IN TireInfo &other)
    : m_id(other.m_id)
    , m_temperature(other.m_temperature)
    , m_pressure(other.m_pressure)
    , m_extend_pressure(other.m_extend_pressure)
{}

TireInfo & TireInfo::operator=(IN TireInfo &other)
{
    if ( this != &other ) {
        m_id                = other.m_id;
        m_temperature       = other.m_temperature;
        m_pressure          = other.m_pressure;
        m_extend_pressure   = other.m_extend_pressure;
    }

    return *this;
}

AccidentFrame::AccidentFrame()
    : m_emergency_braking_state(vehicle::g_vehicle_signals.find(vehicle::EmergencyBrakingSystemState)->second)
    , m_emergency_braking_target_id(vehicle::g_vehicle_signals.find(vehicle::EmergencyBrakingTargetID)->second)
    , m_target_relative_speed(vehicle::g_vehicle_signals.find(vehicle::EmergencyTargetRelativeSpeed)->second)
    , m_target_speed(vehicle::g_vehicle_signals.find(vehicle::EmergencyTargetSpeed)->second)
    , m_target_distance(vehicle::g_vehicle_signals.find(vehicle::EmergencyTargetDistance)->second)
{}

AccidentFrame::~AccidentFrame()
{}

AccidentFrame::AccidentFrame( IN SMLK_DOUBLE braking_system_state,
    IN SMLK_DOUBLE braking_system_target_id, IN SMLK_DOUBLE target_relative_speed,
    IN SMLK_DOUBLE target_speed, IN SMLK_DOUBLE target_distance )
    : AccidentFrame()
{
    m_emergency_braking_state       = braking_system_state;
    m_emergency_braking_target_id   = braking_system_target_id;
    m_target_relative_speed         = target_relative_speed;
    m_target_speed                  = target_speed;
    m_target_distance               = target_distance;
}

AccidentFrame::AccidentFrame(IN AccidentFrame &other)
    : m_emergency_braking_state(other.m_emergency_braking_state)
    , m_emergency_braking_target_id(other.m_emergency_braking_target_id)
    , m_target_relative_speed(other.m_target_relative_speed)
    , m_target_speed(other.m_target_speed)
    , m_target_distance(other.m_target_distance)
{}

AccidentFrame & AccidentFrame::operator=(IN AccidentFrame &other)
{
    if ( this != &other ) {
        m_emergency_braking_state       = other.m_emergency_braking_state;
        m_emergency_braking_target_id   = other.m_emergency_braking_target_id;
        m_target_relative_speed         = other.m_target_relative_speed;
        m_target_speed                  = other.m_target_speed;
        m_target_distance               = other.m_target_distance;
    }

    return *this;
}

FaultCode::FaultCode()
    : m_source(0)
    , m_FMI(0)
    , m_SPN(0)
{}

FaultCode::FaultCode(SMLK_UINT8 source, SMLK_UINT8 FMI, SMLK_UINT32 SPN)
    : m_source(source)
    , m_FMI(FMI)
    , m_SPN(SPN)
{}

FaultCode::~FaultCode()
{}

FaultCode::FaultCode(IN FaultCode &other)
    : m_source(other.m_source)
    , m_FMI(other.m_FMI)
    , m_SPN(other.m_SPN)
{}

FaultCode & FaultCode::operator=(IN FaultCode &other)
{
    if ( this != &other ) {
        m_source    = other.m_source;
        m_FMI       = other.m_FMI;
        m_SPN       = other.m_SPN;
    }
    return *this;
}

MessageDM1::MessageDM1()
    : m_type(FAULT_OCCURS)
    , m_tachograph_speed(vehicle::g_vehicle_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second)
    , m_acc_pedal_travel(vehicle::g_vehicle_signals.find(vehicle::SignalID::AccPedalTravel)->second)
    , m_brake_pedal_state(vehicle::g_vehicle_signals.find(vehicle::SignalID::BrakeSwitch)->second)
    , m_engine_speed(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineSpeed)->second)
    , m_tubocharing_pressue(0, 8, 2)
    , m_intake_manifold_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineIntakeManifoldPressure)->second)
    , m_exhaust_gas_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineExhaustGasTemperature)->second)
    , m_coolant_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineCoolantTemperature)->second)
    , m_acc_change_rate(0, 16)
    , m_gear(vehicle::g_vehicle_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second)
    , m_torque_percent(vehicle::g_vehicle_signals.find(vehicle::SignalID::ActualEnginePercentTorque)->second)
    , m_vehicle_load(0, 32)
    , m_acceleration(0, 16, 0.01)
    , m_deceleration(0, 16, 0.01)
    , m_engine_load(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineLoadOnCurrentSpeed)->second)
{}

MessageDM1::MessageDM1(SMLK_UINT8 type, std::vector<FaultCode> &&faults)
    : m_type(type)
    , m_tachograph_speed(vehicle::g_vehicle_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second)
    , m_acc_pedal_travel(vehicle::g_vehicle_signals.find(vehicle::SignalID::AccPedalTravel)->second)
    , m_brake_pedal_state(vehicle::g_vehicle_signals.find(vehicle::SignalID::BrakeSwitch)->second)
    , m_engine_speed(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineSpeed)->second)
    , m_tubocharing_pressue(0, 8, 2)
    , m_intake_manifold_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineIntakeManifoldPressure)->second)
    , m_exhaust_gas_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineExhaustGasTemperature)->second)
    , m_coolant_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineCoolantTemperature)->second)
    , m_acc_change_rate(0, 16)
    , m_gear(vehicle::g_vehicle_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second)
    , m_torque_percent(vehicle::g_vehicle_signals.find(vehicle::SignalID::ActualEnginePercentTorque)->second)
    , m_vehicle_load(0, 32)
    , m_acceleration(0, 16, 0.01)
    , m_deceleration(0, 16, 0.01)
    , m_engine_load(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineLoadOnCurrentSpeed)->second)
    , m_faults{std::forward<std::vector<FaultCode>>(faults)}
{}

MessageDM1::~MessageDM1()
{}

MessageDM1::MessageDM1(IN MessageDM1 &other)
    : m_type(other.m_type)
    , m_tachograph_speed(other.m_tachograph_speed)
    , m_acc_pedal_travel(other.m_acc_pedal_travel)
    , m_brake_pedal_state(other.m_brake_pedal_state)
    , m_engine_speed(other.m_engine_speed)
    , m_tubocharing_pressue(other.m_tubocharing_pressue)
    , m_intake_manifold_pressure(other.m_intake_manifold_pressure)
    , m_exhaust_gas_temperature(other.m_exhaust_gas_temperature)
    , m_coolant_temperature(other.m_coolant_temperature)
    , m_acc_change_rate(other.m_acc_change_rate)
    , m_gear(other.m_gear)
    , m_torque_percent(other.m_torque_percent)
    , m_vehicle_load(other.m_vehicle_load)
    , m_acceleration(other.m_acceleration)
    , m_deceleration(other.m_deceleration)
    , m_engine_load(other.m_engine_load)
    , m_faults(other.m_faults)
{}

MessageDM1::MessageDM1(MessageDM1 &&other)
    : m_type(other.m_type)
    , m_tachograph_speed(other.m_tachograph_speed)
    , m_acc_pedal_travel(other.m_acc_pedal_travel)
    , m_brake_pedal_state(other.m_brake_pedal_state)
    , m_engine_speed(other.m_engine_speed)
    , m_tubocharing_pressue(other.m_tubocharing_pressue)
    , m_intake_manifold_pressure(other.m_intake_manifold_pressure)
    , m_exhaust_gas_temperature(other.m_exhaust_gas_temperature)
    , m_coolant_temperature(other.m_coolant_temperature)
    , m_acc_change_rate(other.m_acc_change_rate)
    , m_gear(other.m_gear)
    , m_torque_percent(other.m_torque_percent)
    , m_vehicle_load(other.m_vehicle_load)
    , m_acceleration(other.m_acceleration)
    , m_deceleration(other.m_deceleration)
    , m_engine_load(other.m_engine_load)
    , m_faults(std::move(other.m_faults))
{}

MessageDM1 & MessageDM1::operator=(IN MessageDM1 &other)
{
    if ( this != &other ) {
        this->m_type                        = other.m_type;
        this->m_tachograph_speed            = other.m_tachograph_speed;
        this->m_acc_pedal_travel            = other.m_acc_pedal_travel;
        this->m_brake_pedal_state           = other.m_brake_pedal_state;
        this->m_engine_speed                = other.m_engine_speed;
        this->m_tubocharing_pressue         = other.m_tubocharing_pressue;
        this->m_intake_manifold_pressure    = other.m_intake_manifold_pressure;
        this->m_exhaust_gas_temperature     = other.m_exhaust_gas_temperature;
        this->m_coolant_temperature         = other.m_coolant_temperature;
        this->m_acc_change_rate             = other.m_acc_change_rate;
        this->m_gear                        = other.m_gear;
        this->m_torque_percent              = other.m_torque_percent;
        this->m_vehicle_load                = other.m_vehicle_load;
        this->m_acceleration                = other.m_acceleration;
        this->m_deceleration                = other.m_deceleration;
        this->m_engine_load                 = other.m_engine_load;
        this->m_faults                      = other.m_faults;
    }

    return *this;
}

MessageDM1 & MessageDM1::operator=(MessageDM1 &&other)
{
    if ( this != &other ) {
        this->m_type    = other.m_type;
        this->m_tachograph_speed            = other.m_tachograph_speed;
        this->m_acc_pedal_travel            = other.m_acc_pedal_travel;
        this->m_brake_pedal_state           = other.m_brake_pedal_state;
        this->m_engine_speed                = other.m_engine_speed;
        this->m_tubocharing_pressue         = other.m_tubocharing_pressue;
        this->m_intake_manifold_pressure    = other.m_intake_manifold_pressure;
        this->m_exhaust_gas_temperature     = other.m_exhaust_gas_temperature;
        this->m_coolant_temperature         = other.m_coolant_temperature;
        this->m_acc_change_rate             = other.m_acc_change_rate;
        this->m_gear                        = other.m_gear;
        this->m_torque_percent              = other.m_torque_percent;
        this->m_vehicle_load                = other.m_vehicle_load;
        this->m_acceleration                = other.m_acceleration;
        this->m_deceleration                = other.m_deceleration;
        this->m_engine_load                 = other.m_engine_load;
        this->m_faults                      = std::move(other.m_faults);
    }

    return *this;
}

MessageLocation::MessageLocation()
    : m_report_location_only(false)
    , m_alarm_flags(vehicle::g_vehicle_signals.find(vehicle::SignalID::AlarmFlags)->second)
    , m_tachograph_vehicle_speed(vehicle::g_vehicle_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second)
    , m_engine_torque_mode(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineTorqueMode)->second)
    , m_torque_percent_cmd(vehicle::g_vehicle_signals.find(vehicle::SignalID::DriverDemandEnginePercentTorque)->second)
    , m_torque_percent(vehicle::g_vehicle_signals.find(vehicle::SignalID::ActualEnginePercentTorque)->second)
    , m_engine_speed(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineSpeed)->second)
    , m_coolant_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineCoolantTemperature)->second)
    , m_oil_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineOilPressure)->second)
    , m_engine_total_fuel_used(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineTotalFuelUsed)->second)
    , m_acc_pedal_travel(vehicle::g_vehicle_signals.find(vehicle::SignalID::AccPedalTravel)->second)
    , m_parking(vehicle::g_vehicle_signals.find(vehicle::SignalID::ParkingBrakeSwitch)->second)
    , m_wheel_speed(vehicle::g_vehicle_signals.find(vehicle::SignalID::WheelSpeed)->second)
    , m_clutch(vehicle::g_vehicle_signals.find(vehicle::SignalID::ClutchSwitch)->second)
    , m_brake_pedal_state(vehicle::g_vehicle_signals.find(vehicle::SignalID::BrakeSwitch)->second)
    , m_urea_tank_level(vehicle::g_vehicle_signals.find(vehicle::SignalID::CatalystTankLevel)->second)
    , m_engine_total_time(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineTotalHoursOfOperation)->second)
    , m_engine_total_cycles(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineTotalRevolutions)->second)
    , m_engine_fuel_consumption_rate(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineFuelRate)->second)
    , m_engine_instantaneous_fuel_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineInstantaneousFuelEconomy)->second)
    , m_engine_average_fuel_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineAverageFuelEconomy)->second)
    , m_dpf_pressure_differential(vehicle::g_vehicle_signals.find(vehicle::SignalID::AftertreatmentDieselParticulateFilterDifferentialPressure)->second)
    , m_intake_manifold_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineIntakeManifoldPressure)->second)
    , m_intake_manifole_temprerature(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineIntakeManifoldTemperature)->second)
    , m_engine_intake_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineAirIntakePressure)->second)
    , m_barometric_pressure(vehicle::g_vehicle_signals.find(vehicle::SignalID::BarometricPressure)->second)
    , m_cab_interior_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::CabInteriorTemperature)->second)
    , m_ambient_air_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::AmbientAirTemperature)->second)
    , m_engine_intake_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineAirIntakeTemperature)->second)
    , m_cold_start_light(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineWaitToStartLamp)->second)
    , m_water_in_fuel_flag(vehicle::g_vehicle_signals.find(vehicle::SignalID::WaterInFuelIndicator)->second)
    , m_target_gear(vehicle::g_vehicle_signals.find(vehicle::SignalID::TransmissionSelectedGear)->second)
    , m_gear_ratio(vehicle::g_vehicle_signals.find(vehicle::SignalID::TransmissionActualGearRatio)->second)
    , m_gear(vehicle::g_vehicle_signals.find(vehicle::SignalID::TransmissionCurrentGear)->second)
    , m_instrument_fuel_level(vehicle::g_vehicle_signals.find(vehicle::SignalID::FuelLevel)->second)
    , m_instrument_subtotal_mileage(vehicle::g_vehicle_signals.find(vehicle::SignalID::TripDistance)->second)
    , m_instrument_total_mileage(245, 32, 0.125, 0, 0, 4211081215)  // report can original data
    , m_instrument_total_mileage_head_ext1(245, 32, 0.1, 0, 0, 4211081215) // report phycial value(0.1km)
    , m_calculus_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::CalculusDistance)->second)
    , m_calculus_fuel_used(vehicle::g_vehicle_signals.find(vehicle::SignalID::CalculusFuelConsumption)->second)
    , m_actural_catalyst_spewed(vehicle::g_vehicle_signals.find(vehicle::SignalID::ActuralCatalystSpewed)->second)
    , m_total_catalyst_spewed(vehicle::g_vehicle_signals.find(vehicle::SignalID::TotalCatalystSpewed)->second)
    , m_acc_status(vehicle::g_vehicle_signals.find(vehicle::SignalID::ACCStatus)->second)
    , m_extended_signal_status(vehicle::g_vehicle_signals.find(vehicle::SignalID::ExtendedSignalStatus)->second)
    , m_rssi(vehicle::g_vehicle_signals.find(vehicle::SignalID::RSSI)->second)
    , m_satellites_used(vehicle::g_vehicle_signals.find(vehicle::SignalID::GNSSSatelliteUsed)->second)
    , m_battery_1_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Battery1Temperature)->second)
    , m_battery_1_vol(vehicle::g_vehicle_signals.find(vehicle::SignalID::Battery1Voltage)->second)
    , m_battery_1_soc(vehicle::g_vehicle_signals.find(vehicle::SignalID::Battery1SOC)->second)
    , m_battery_1_soh(vehicle::g_vehicle_signals.find(vehicle::SignalID::Battery1SOH)->second)
    , m_battery_2_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Battery2Temperature)->second)
    , m_battery_2_vol(vehicle::g_vehicle_signals.find(vehicle::SignalID::Battery2Voltage)->second)
    , m_battery_2_soc(vehicle::g_vehicle_signals.find(vehicle::SignalID::Battery2SOC)->second)
    , m_battery_2_soh(vehicle::g_vehicle_signals.find(vehicle::SignalID::Battery2SOH)->second)
    , m_actual_minimum_soc(vehicle::g_vehicle_signals.find(vehicle::SignalID::ActualMinimumSOC)->second)
    , m_actual_minimum_soh(vehicle::g_vehicle_signals.find(vehicle::SignalID::ActualMinimumSOH)->second)
    , m_agtator_state(vehicle::g_vehicle_signals.find(vehicle::SignalID::AgtatorState)->second)
    , m_agtator_speed(vehicle::g_vehicle_signals.find(vehicle::SignalID::AgtatorSpeed)->second)
    , m_agtator_total_rotations(vehicle::g_vehicle_signals.find(vehicle::SignalID::AgtatorTotalRotations)->second)
    , m_catalyst_tank_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::CatalystTankTemperature)->second)
    , m_engine_power_input1(vehicle::g_vehicle_signals.find(vehicle::SignalID::EnginePowerInput1)->second)
    , m_keyswitch_battery_voltage(vehicle::g_vehicle_signals.find(vehicle::SignalID::keyswitchBatteryVoltage)->second)
    , m_exhaust_gas_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::EngineExhaustGasTemperature)->second)
    , m_sensor1_humidity(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor1Humidity)->second)
    , m_sensor2_humidity(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor2Humidity)->second)
    , m_sensor3_humidity(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor3Humidity)->second)
    , m_sensor4_humidity(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor4Humidity)->second)
    , m_sensor5_humidity(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor5Humidity)->second)
    , m_sensor6_humidity(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor6Humidity)->second)
    , m_sensor7_humidity(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor7Humidity)->second)
    , m_sensor8_humidity(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor8Humidity)->second)
    , m_sensor1_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor1Temperature)->second)
    , m_sensor2_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor2Temperature)->second)
    , m_sensor3_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor3Temperature)->second)
    , m_sensor4_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor4Temperature)->second)
    , m_sensor5_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor5Temperature)->second)
    , m_sensor6_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor6Temperature)->second)
    , m_sensor7_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor7Temperature)->second)
    , m_sensor8_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::Sensor8Temperature)->second)
    , m_pefrigeration_switch_status(vehicle::g_vehicle_signals.find(vehicle::SignalID::PefrigerationSwitchStatus)->second)
    , m_pefrigeration_work_mode(vehicle::g_vehicle_signals.find(vehicle::SignalID::PefrigerationWorkMode)->second)
    , m_pefrigeration_cpmpartment_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::PefrigerationCpmpartmentTemperature)->second)
    , m_defrost_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::DefrostTemperature)->second)
    , m_supply_voltage(vehicle::g_vehicle_signals.find(vehicle::SignalID::SupplyVoltage)->second)
    , m_dc_ac_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::DcAcPowerConsumption)->second)
    , m_steerwheel_heat_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::SteerWheelHeatPowerConsumption)->second)
    , m_cigar_light_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::CigarLighterPowerConsumption)->second)
    , m_24v_socket1_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::Socket1PowerConsumption)->second)
    , m_24v_socket2_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::Socket2PowerConsumption)->second)
    , m_left_seat_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::LeftSeatHeatPowerConsumption)->second)
    , m_right_seat_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::RightSeatHeatPowerConsumption)->second)
    , m_lower_bretht_heat_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::LowerBerthtHeatPowerConsumption)->second)
    , m_sleep_read_lamp_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::SleepReadLampPowerConsumption)->second)
    , m_elevated_box_power_consumption(vehicle::g_vehicle_signals.find(vehicle::SignalID::ElevatedBoxPowerConsumption)->second)
    , m_humidity_in_box1(vehicle::g_vehicle_signals.find(vehicle::SignalID::HumidityInBox1)->second)
    , m_humidity_in_box2(vehicle::g_vehicle_signals.find(vehicle::SignalID::HumidityInBox2)->second)
    , m_temperature_in_box1(vehicle::g_vehicle_signals.find(vehicle::SignalID::TemperatureInBox1)->second)
    , m_temperature_in_box2(vehicle::g_vehicle_signals.find(vehicle::SignalID::TemperatureInBox2)->second)
    , m_tbox_voltage(vehicle::g_vehicle_signals.find(vehicle::SignalID::TboxVlotage)->second)
    , m_ev_vehicle_speed(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvVehicleSpeed)->second)
    , m_fuel_save_switch(vehicle::g_vehicle_signals.find(vehicle::SignalID::FuelSaveSwitch)->second)
    , m_hydraulic_motor_oil_return_temperature(vehicle::g_vehicle_signals.find(vehicle::SignalID::HydraulicMotorOilReturnTemperature)->second)
    , m_ev_vehicle_status(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvVehicleStatus)->second)
    , m_ev_charging_status(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvChargingStatus)->second)
    , m_ev_working_status(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvWorkingStatus)->second)
    , m_ev_total_voltage(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvTotalVoltage)->second)
    , m_ev_total_current(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvTotalCurrent)->second)
    , m_ev_battery_soc(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvBatterySoc)->second)
    , m_ev_dc_working_status(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvDcDcWoringStatus)->second)
    , m_ev_gear_status(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvGearStatus)->second)
    , m_ev_insulation_resistance(vehicle::g_vehicle_signals.find(vehicle::SignalID::EvInsulationResistance)->second)
{}

MessageLocation & MessageLocation::operator=(IN MessageLocation &other)
{
    if ( this != &other ) {
        m_alarm_flags                   = other.m_alarm_flags;
        m_tachograph_vehicle_speed      = other.m_tachograph_vehicle_speed;
        m_engine_torque_mode            = other.m_engine_torque_mode;
        m_torque_percent_cmd            = other.m_torque_percent_cmd;
        m_torque_percent                = other.m_torque_percent;
        m_engine_speed                  = other.m_engine_speed;
        m_coolant_temperature           = other.m_coolant_temperature;
        m_oil_pressure                  = other.m_oil_pressure;
        m_engine_total_fuel_used        = other.m_engine_total_fuel_used;
        m_acc_pedal_travel              = other.m_acc_pedal_travel;
        m_parking                       = other.m_parking;
        m_wheel_speed                   = other.m_wheel_speed;
        m_clutch                        = other.m_clutch;
        m_brake_pedal_state             = other.m_brake_pedal_state;
        m_urea_tank_level               = other.m_urea_tank_level;
        m_engine_total_time             = other.m_engine_total_time;
        m_engine_total_cycles           = other.m_engine_total_cycles;
        m_engine_fuel_consumption_rate  = other.m_engine_fuel_consumption_rate;
        m_engine_instantaneous_fuel_consumption = other.m_engine_instantaneous_fuel_consumption;
        m_engine_average_fuel_consumption       = other.m_engine_average_fuel_consumption;
        m_dpf_pressure_differential     = other.m_dpf_pressure_differential;
        m_intake_manifold_pressure      = other.m_intake_manifold_pressure;
        m_intake_manifole_temprerature  = other.m_intake_manifole_temprerature;
        m_engine_intake_pressure        = other.m_engine_intake_pressure;
        m_barometric_pressure           = other.m_barometric_pressure;
        m_cab_interior_temperature      = other.m_cab_interior_temperature;
        m_ambient_air_temperature       = other.m_ambient_air_temperature;
        m_engine_intake_temperature     = other.m_engine_intake_temperature;
        m_cold_start_light              = other.m_cold_start_light;
        m_water_in_fuel_flag            = other.m_water_in_fuel_flag;
        m_target_gear                   = other.m_target_gear;
        m_gear_ratio                    = other.m_gear_ratio;
        m_gear                          = other.m_gear;
        m_instrument_fuel_level         = other.m_instrument_fuel_level;
        m_instrument_subtotal_mileage   = other.m_instrument_subtotal_mileage;
        m_instrument_total_mileage      = other.m_instrument_total_mileage;
        m_instrument_total_mileage_head_ext1 = other.m_instrument_total_mileage_head_ext1;
        m_calculus_distance             = other.m_calculus_distance;
        m_calculus_fuel_used            = other.m_calculus_fuel_used;
        m_actural_catalyst_spewed       = other.m_actural_catalyst_spewed;
        m_total_catalyst_spewed         = other.m_total_catalyst_spewed;
        m_acc_status                    = other.m_acc_status;
        m_extended_signal_status        = other.m_extended_signal_status;
        m_rssi                          = other.m_rssi;
        m_satellites_used               = other.m_satellites_used;
        m_battery_1_temperature         = other.m_battery_1_temperature;
        m_battery_1_vol                 = other.m_battery_1_vol;
        m_battery_1_soc                 = other.m_battery_1_soc;
        m_battery_1_soh                 = other.m_battery_1_soh;
        m_battery_2_temperature         = other.m_battery_2_temperature;
        m_battery_2_vol                 = other.m_battery_2_vol;
        m_battery_2_soc                 = other.m_battery_2_soc;
        m_battery_2_soh                 = other.m_battery_2_soh;
        m_actual_minimum_soc            = other.m_actual_minimum_soc;
        m_actual_minimum_soh            = other.m_actual_minimum_soh;
        m_agtator_state                 = other.m_agtator_state;
        m_agtator_speed                 = other.m_agtator_speed;
        m_agtator_total_rotations       = other.m_agtator_total_rotations;
        m_region                        = other.m_region;
        m_location                      = other.m_location;
        m_tires                         = other.m_tires;
    }

    return *this;
}

MessageBucket::MessageBucket()
    : m_signal_day_vehicle_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::CalculusDayDistance)->second)
    , m_signal_total_vehicle_distance(245,   32, 0.1)
    , m_signal_ecu_total_distacne(0,     32, 0.1)
    , m_signal_day_fuel_used(0,     16, 0.1)
    , m_signal_total_fuel_used(0,     32)
    , m_signal_ecu_total_fuel_used(0,     32)
{}

MessageBucket & MessageBucket::operator=(IN MessageBucket &other)
{
    if ( this != &other ) {
        m_sampling_frequency                    = other.m_sampling_frequency;
        m_crompression                          = other.m_crompression;
        m_signal_day_vehicle_distance           = other.m_signal_day_vehicle_distance;
        m_signal_total_vehicle_distance         = other.m_signal_total_vehicle_distance;
        m_signal_ecu_total_distacne             = other.m_signal_ecu_total_distacne;
        m_signal_day_fuel_used                  = other.m_signal_day_fuel_used;
        m_signal_total_fuel_used                = other.m_signal_total_fuel_used;
        m_signal_ecu_total_fuel_used            = other.m_signal_ecu_total_fuel_used;

        m_signals.clear();

        for ( auto const record : other.m_signals ) {
            m_signals.emplace_back();

            for ( auto const &pair : record ) {
                m_signals.back().emplace(
                    std::piecewise_construct,
                    std::forward_as_tuple(pair.first),
                    std::forward_as_tuple(pair.second)
                );
            }
        }
    }

    return *this;
}

const std::vector<vehicle::SignalID> & MessageBucket::SignalIDs()
{
    static const std::vector<vehicle::SignalID>  signals = {
        vehicle::SignalID::GNSSTimeStamp,
        vehicle::SignalID::GNSSLatitude,
        vehicle::SignalID::GNSSLongitude,
        vehicle::SignalID::GNSSSpeed,
        vehicle::SignalID::GNSSLocatedStatus,
        vehicle::SignalID::GNSSAltitude,
        vehicle::SignalID::TachographVehicleSpeed,
        vehicle::SignalID::EngineSpeed,
        vehicle::SignalID::ActualEnginePercentTorque,
        vehicle::SignalID::DriverDemandEnginePercentTorque,
        vehicle::SignalID::AccPedalTravel,
        vehicle::SignalID::EngineFuelRate,
        vehicle::SignalID::EngineInstantaneousFuelEconomy,
        vehicle::SignalID::VehicleSwitchStatus,
        vehicle::SignalID::TBoxAcceleration,
        vehicle::SignalID::LateralAcceleration,
        vehicle::SignalID::LongitudinalAcceleration,
        vehicle::SignalID::YawRate,
        vehicle::SignalID::EngineIntakeManifoldPressure,
        vehicle::SignalID::EngineOilPressure,
        vehicle::SignalID::EngineCoolantTemperature,
        vehicle::SignalID::EngineBrakingPercentTorque,
        vehicle::SignalID::EngineExhaustPercentTorque,
        vehicle::SignalID::RetarderSelection,
        vehicle::SignalID::ActualRetarderPercentTorque,
        vehicle::SignalID::RetarderTorqueMode,
        vehicle::SignalID::DriverDemandRetarderPercentTorque,
        vehicle::SignalID::ActualMaximumAvailableRetarderPercentTorque,
        vehicle::SignalID::TransmissionCurrentGear,
        vehicle::SignalID::TransmissionSelectedGear,
        vehicle::SignalID::SteeringWheelAngle,
        vehicle::SignalID::FrontAxleSpeed,
        vehicle::SignalID::FrontAxleLeftWheelSpeed,
        vehicle::SignalID::FrontAxleRightWheelSpeed,
        vehicle::SignalID::RearAxle1LeftWheelSpeed,
        vehicle::SignalID::RearAxle1RightWheelSpeed,
        vehicle::SignalID::BrakePedalTravel,
        vehicle::SignalID::TransmissionInputShaftSpeed,
        vehicle::SignalID::TransmissionOutputShaftSpeed,
        vehicle::SignalID::EngineLoadOnCurrentSpeed,
        vehicle::SignalID::NominalFrictionPercentTorque,
        vehicle::SignalID::PercentClutchSlip,
        vehicle::SignalID::EngineIntakeAirMassFlowRat,
        vehicle::SignalID::OverrideControlMode,
        vehicle::SignalID::RequestSpeedLimit,
        vehicle::SignalID::RequestTorqueLimit,
        vehicle::SignalID::RequestTorqueHighResolution,
        vehicle::SignalID::PitchAngle,
        vehicle::SignalID::CurrentSlope,
        vehicle::SignalID::AirSuspensionControl1,
        vehicle::SignalID::AirSuspensionControl2,
        vehicle::SignalID::AirSuspensionControl3,
        vehicle::SignalID::AirSuspensionControl4,
        vehicle::SignalID::ActuralCatalystSpewed,
        vehicle::SignalID::TotalCatalystSpewed,
        vehicle::SignalID::CatalystTankTemperature,
        vehicle::SignalID::TurnSwitchStatus,
        vehicle::SignalID::TireLocation1,
        vehicle::SignalID::TirePressure,
        vehicle::SignalID::TireTemperature,
        vehicle::SignalID::TireStatus,
        vehicle::SignalID::TireExtrentPressureSupport,
        vehicle::SignalID::TireAirLeakageRate,
        vehicle::SignalID::TirePressureThresholdDetection,
        vehicle::SignalID::TireLocation2,
        vehicle::SignalID::TirePressureExtendedRange,
        vehicle::SignalID::TirePressureRequired,
        vehicle::SignalID::AirConditionerOnOffStatus,
        vehicle::SignalID::AirConditionerModeStatus,
        vehicle::SignalID::AirConditionerAutoModeStatus,
        vehicle::SignalID::AirConditionerBlowingRate,
        vehicle::SignalID::AirConditionerACModeStatus,
        vehicle::SignalID::AirConditionerCirculationModeStatus,
        vehicle::SignalID::WarmAirBlowerOnOffStatus,
        vehicle::SignalID::WarmAirTemperatureStatus,
        vehicle::SignalID::ParkingAirConditionerOnOffStatus,
        vehicle::SignalID::ParkingAirConditionerBlowingRate,
        vehicle::SignalID::ACRegulationTemperature,
        vehicle::SignalID::ParkingACRegulationTemperature,
        vehicle::SignalID::CabTemperature,
        vehicle::SignalID::ACRegulationHRTemperature,
        vehicle::SignalID::ParkingACRegulationHRTemperature,
        vehicle::SignalID::AirConditionerVentilationStatus,
        vehicle::SignalID::ParkingACAutoModeStatus,
        vehicle::SignalID::DieselParticulateFilterStatus,
        vehicle::SignalID::DieselParticulateFilterLamp,
        vehicle::SignalID::DieselParticulateFilterActiveRegenerationStatus,
        vehicle::SignalID::DieselParticulateFilterActiveRegenerationInhibitedStatus,
        vehicle::SignalID::DriverInducementIndicatorStatus,
        vehicle::SignalID::MalfunctionIndicatorLampStatus,
        vehicle::SignalID::AftertreatmentRegenerationInhibitSwitch,
        vehicle::SignalID::AgtatorSpeed,
        vehicle::SignalID::AgtatorState,
    };

    return signals;
}

const std::unordered_map<vehicle::SignalID, SMLK_UINT8> &   MessageBucket::SignalsPrecision()
{
    static const std::unordered_map<vehicle::SignalID, SMLK_UINT8> precisions = {
        {vehicle::SignalID::GNSSLatitude,                               6},
        {vehicle::SignalID::GNSSLongitude,                              6},
        {vehicle::SignalID::ActuralCatalystSpewed,                      2},
        {vehicle::SignalID::TotalCatalystSpewed,                        2},
#if 0
        {vehicle::SignalID::GNSSAltitude,                               2},
        {vehicle::SignalID::GNSSSpeed,                                  2},
        {vehicle::SignalID::EngineInstantaneousFuelEconomy,             2},
        {vehicle::SignalID::TBoxAcceleration,                           2},
        {vehicle::SignalID::LateralAcceleration,                        2},
        {vehicle::SignalID::YawRate,                                    2},
        {vehicle::SignalID::SteeringWheelAngle,                         2},
        {vehicle::SignalID::FrontAxleSpeed,                             2},
        {vehicle::SignalID::FrontAxleLeftWheelSpeed,                    2},
        {vehicle::SignalID::FrontAxleRightWheelSpeed,                   2},
        {vehicle::SignalID::RearAxle1LeftWheelSpeed,                    2},
        {vehicle::SignalID::RearAxle1RightWheelSpeed,                   2},
        {vehicle::SignalID::BrakePedalTravel,                           2},
        {vehicle::SignalID::EngineIntakeAirMassFlowRat,                 2},
        {vehicle::SignalID::PitchAngle,                                 2},
        {vehicle::SignalID::CurrentSlope,                               2},
        {vehicle::SignalID::ActuralCatalystSpewed,                      2},
        {vehicle::SignalID::TireTemperature,                            2},
        {vehicle::SignalID::TireAirLeakageRate,                         2},
        {vehicle::SignalID::ACRegulationHRTemperature,                  2},
        {vehicle::SignalID::ParkingACRegulationHRTemperature,           2},
#endif
    };

    return precisions;
}

MessageBucket30S::MessageBucket30S()
    : m_signal_day_vehicle_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::CalculusDayDistance)->second)
    , m_signal_total_vehicle_distance(245,   32, 0.1)
    , m_signal_ecu_total_distacne(0,     32, 0.1)
    , m_signal_day_fuel_used(0,     16, 0.1)
    , m_signal_total_fuel_used(0,     32)
    , m_signal_ecu_total_fuel_used(0,     32)
{}

MessageBucket30S & MessageBucket30S::operator=(IN MessageBucket30S &other)
{
    if ( this != &other ) {
        m_sampling_frequency                    = other.m_sampling_frequency;
        m_crompression                          = other.m_crompression;
        m_signal_day_vehicle_distance           = other.m_signal_day_vehicle_distance;
        m_signal_total_vehicle_distance         = other.m_signal_total_vehicle_distance;
        m_signal_ecu_total_distacne             = other.m_signal_ecu_total_distacne;
        m_signal_day_fuel_used                  = other.m_signal_day_fuel_used;
        m_signal_total_fuel_used                = other.m_signal_total_fuel_used;
        m_signal_ecu_total_fuel_used            = other.m_signal_ecu_total_fuel_used;

        m_signals.clear();

        for ( auto const record : other.m_signals ) {
            m_signals.emplace_back();

            for ( auto const &pair : record ) {
                m_signals.back().emplace(
                    std::piecewise_construct,
                    std::forward_as_tuple(pair.first),
                    std::forward_as_tuple(pair.second)
                );
            }
        }
    }

    return *this;
}

const std::vector<vehicle::SignalID> & MessageBucket30S::SignalIDs()
{
    static const std::vector<vehicle::SignalID>  signals = {
        vehicle::SignalID::GNSSTimeStamp,
        vehicle::SignalID::GNSSLatitude,
        vehicle::SignalID::GNSSLongitude,
        vehicle::SignalID::GNSSSpeed,
        vehicle::SignalID::GNSSLocatedStatus,
        vehicle::SignalID::GNSSAltitude,
        vehicle::SignalID::TrailerWeight,
        vehicle::SignalID::BrightnensPercent,
        vehicle::SignalID::EngineAverageFuelEconomy,
        vehicle::SignalID::AxleWeight,
        vehicle::SignalID::AftertreatmentIntakeNOx,
        vehicle::SignalID::AftertreatmentOutletNOx,
        vehicle::SignalID::AftertreatmentIntakeGasSensorTemperature,
        vehicle::SignalID::AftertreatmentOutletGasSensorTemperature,
        vehicle::SignalID::ATMTransmissionMode,
        vehicle::SignalID::ReferenceEngineTorque,
        vehicle::SignalID::FuelTemperature,
        vehicle::SignalID::CruiseControlSetSpeed,
    };

    return signals;
}

const std::unordered_map<vehicle::SignalID, SMLK_UINT8> &   MessageBucket30S::SignalsPrecision()
{
    static const std::unordered_map<vehicle::SignalID, SMLK_UINT8> precisions = {
        {vehicle::SignalID::GNSSLatitude,                               6},
        {vehicle::SignalID::GNSSLongitude,                              6},
#if 0
        {vehicle::SignalID::GNSSAltitude,                               2},
        {vehicle::SignalID::GNSSSpeed,                                  2},
        {vehicle::SignalID::EngineAverageFuelEconomy,                   2},
        {vehicle::SignalID::AftertreatmentIntakeNOx,                    2},
        {vehicle::SignalID::AftertreatmentOutletNOx,                    2},
        {vehicle::SignalID::AftertreatmentIntakeGasSensorTemperature,   2},
        {vehicle::SignalID::AftertreatmentOutletGasSensorTemperature,   2},
#endif
    };

    return precisions;
}

MessageCompressedBucket & MessageCompressedBucket::operator=(IN MessageCompressedBucket &other)
{
    if ( this != &other ) {
        m_sampling_frequency    = other.m_sampling_frequency;
        m_crompression          = other.m_crompression;
        m_compressed            = other.m_compressed;
    }

    return *this;
}

MessageStatistics::MessageStatistics()
    : m_flags(0)
    , m_trip_start(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsTripStartTime)->second)
    , m_trip_stop(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsTripEndTime)->second)
    , m_gnss_statistics_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsGnssSpeedDistance)->second)
    , m_tachograph_vehicle_speed_statistics_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsTachographVehicleSpeedDistance)->second)
    , m_vehicle_speed_statistics_life_total_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsVehicleSpeedLifeTotalDistance)->second)
    , m_vehicle_total_distance(245, 32, 0.001, 0, 0, 4294967.295) // resolution to tsp: M, max should is FFFFFFFF * 0.001
    , m_vehicle_estimated_load(vehicle::g_vehicle_signals.find(vehicle::SignalID::TrailerWeight)->second)
    , m_trip_statistics_fuel_used(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsTripFuelUsed)->second)
    , m_total_statistics_fuel_used(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsTotalFuelUsed)->second)
    , m_engine_total_fuel_used(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsVehicleTotalFuelUsed)->second)
    , m_engine_average_fuel_economy(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsAverageFuelEconomy)->second)
    , m_speeding_times(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSpeedingTimes)->second)
    , m_speeding_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSpeedingDistance)->second)
    , m_speeding_fuel_used(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSpeedingFuelUsed)->second)
    , m_brake_pedal_times(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsBrakePedalBrakeTimes)->second)
    , m_brake_pedal_brake_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsBrakePedalBrakeDistance)->second)
    , m_brake_pedal_brake_duration(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsBrakePedalBrakeDuration)->second)
    , m_sharp_slowdown_times(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSharpShowDownTimes)->second)
    , m_sharp_slowdown_duration(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSharpShowDownDuration)->second)
    , m_sharp_slowdown_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSharpShowDownDistance)->second)
    , m_sharp_speedup_times(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpTimes)->second)
    , m_sharp_speedup_duration(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpDuration)->second)
    , m_sharp_speedup_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpDistance)->second)
    , m_sharp_speedup_fuel_used(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSharpSpeedUpFuelUsed)->second)
    , m_sharp_turnning_times(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSharpTrunningTimes)->second)
    , m_sharp_turnning_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSharpTrunningDistance)->second)
    , m_sleepy_drive_times(vehicle::g_vehicle_signals.find(vehicle::SignalID::StatisticsSleepyDriveTimes)->second)
{}

MessageStatistics & MessageStatistics::operator=(IN MessageStatistics &other)
{
    if ( this != &other ) {
        m_flags                                         = other.m_flags;
        m_trip_start                                    = other.m_trip_start;
        m_trip_stop                                     = other.m_trip_stop;
        m_gnss_statistics_distance                      = other.m_gnss_statistics_distance;
        m_tachograph_vehicle_speed_statistics_distance  = other.m_tachograph_vehicle_speed_statistics_distance;
        m_vehicle_speed_statistics_life_total_distance  = other.m_vehicle_speed_statistics_life_total_distance;
        m_vehicle_total_distance                        = other.m_vehicle_total_distance;
        m_vehicle_estimated_load                        = other.m_vehicle_estimated_load;
        m_trip_statistics_fuel_used                     = other.m_trip_statistics_fuel_used;
        m_total_statistics_fuel_used                    = other.m_total_statistics_fuel_used;
        m_engine_total_fuel_used                        = other.m_engine_total_fuel_used;
        m_engine_average_fuel_economy                   = other.m_engine_average_fuel_economy;
        m_speeding_times                                = other.m_speeding_times;
        m_speeding_distance                             = other.m_speeding_distance;
        m_speeding_fuel_used                            = other.m_speeding_fuel_used;
        m_brake_pedal_times                             = other.m_brake_pedal_times;
        m_brake_pedal_brake_distance                    = other.m_brake_pedal_brake_distance;
        m_brake_pedal_brake_duration                    = other.m_brake_pedal_brake_duration;
        m_sharp_slowdown_times                          = other.m_sharp_slowdown_times;
        m_sharp_slowdown_duration                       = other.m_sharp_slowdown_duration;
        m_sharp_slowdown_distance                       = other.m_sharp_slowdown_distance;
        m_sharp_speedup_times                           = other.m_sharp_speedup_times;
        m_sharp_speedup_duration                        = other.m_sharp_speedup_duration;
        m_sharp_speedup_distance                        = other.m_sharp_speedup_distance;
        m_sharp_speedup_fuel_used                       = other.m_sharp_speedup_fuel_used;
        m_sharp_turnning_times                          = other.m_sharp_turnning_times;
        m_sharp_turnning_distance                       = other.m_sharp_turnning_distance;
        m_sleepy_drive_times                            = other.m_sleepy_drive_times;
        m_location                                      = other.m_location;
    }

    return *this;
}

MessageEvent::MessageEvent()
    : m_id(0xFF)
    , m_reversion(0x00)
    , m_status(0x00000000)
    , m_speed_start(vehicle::g_vehicle_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second)
    , m_speed_stop(vehicle::g_vehicle_signals.find(vehicle::SignalID::TachographVehicleSpeed)->second)
    , m_duration(0,     16)
    , m_distance(0,     16,     1,      0,      0,      65530)
    , m_fuel_used(0,    16,     1,      0,      0,      65530)
    , m_total_distance(vehicle::g_vehicle_signals.find(vehicle::SignalID::TotalVehicleDistance)->second)
    , m_actual_minimum_soc(0,  16, 1,     0,          0,          64255)
{}

MessageEvent & MessageEvent::operator=(IN MessageEvent &other)
{
    if ( this != &other ) {
        m_id                        = other.m_id;
        m_reversion                 = other.m_reversion;
        m_status                    = other.m_status;
        m_speed_start               = other.m_speed_start;
        m_speed_stop                = other.m_speed_stop;
        m_duration                  = other.m_duration;
        m_distance                  = other.m_distance;
        m_fuel_used                 = other.m_fuel_used;
        m_total_distance            = other.m_total_distance;
        m_actual_minimum_soc        = other.m_actual_minimum_soc;
        m_tire_info                 = other.m_tire_info;

        m_locations                 = other.m_locations;
        m_frames                    = other.m_frames;
    }

    return *this;
}

MessageLocationRsp & MessageLocationRsp::operator=(IN MessageLocationRsp &other)
{
    if ( this != &other ) {
        m_seq                       = other.m_seq;
        m_message_location          = other.m_message_location;
    }

    return *this;
}

MessageFault::MessageFault()
    : m_reversion(0)
    , m_location()
    , m_type(MSG_RES)
{}

void MessageFault::ResetMessage() noexcept
{
    m_type = MSG_RES;
    m_fault.reset();
}

SMLK_UINT32 MessageFault::ResetMessage(SMLK_UINT8 type, const std::shared_ptr<IMessage> fault)
{
    m_type = type;
    m_fault = fault;
}

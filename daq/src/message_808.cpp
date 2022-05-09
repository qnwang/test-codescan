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
#include <chrono>
#include <array>
#include <ctime>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <smlk_log.h>
#include <errors_def.h>
#include <event.h>
#include <message_808.h>
#include "collection_service.h"
#include <math.h>
using namespace smartlink;

namespace PID {
    static const SMLK_UINT8 TachographVehicleSpeed              = 0x01;
    static const SMLK_UINT8 EngineTorqueMode                    = 0x02;
    static const SMLK_UINT8 DriverDemandEnginePercentTorque     = 0x03;
    static const SMLK_UINT8 ActualEnginePercentTorque           = 0x04;
    static const SMLK_UINT8 EngineSpeed                         = 0x05;
    static const SMLK_UINT8 EngineCoolantTemperature            = 0x06;
    static const SMLK_UINT8 EngineOilPressure                   = 0x07;
    static const SMLK_UINT8 EngineTotalFuelUsed                 = 0x08;
    static const SMLK_UINT8 AcceleratorPedalPosition1           = 0x09;
    static const SMLK_UINT8 ParkingBrakeSwitch                  = 0x0A;
    static const SMLK_UINT8 WheelBasedVehicleSpeed              = 0x0B;
    static const SMLK_UINT8 ClutchSwitch                        = 0x0C;
    static const SMLK_UINT8 BrakeSwitch                         = 0x0D;
    static const SMLK_UINT8 CatalystTankLevel                   = 0x0E;
    static const SMLK_UINT8 EngineTotalHoursOfOperation         = 0x12;
    static const SMLK_UINT8 EngineTotalRevolutions              = 0x13;
    static const SMLK_UINT8 EngineFuelRate                      = 0x14;
    static const SMLK_UINT8 EngineInstantaneousFuelEconomy      = 0x15;
    static const SMLK_UINT8 EngineAverageFuelEconomy            = 0x16;
    static const SMLK_UINT8 Aftertreatment1DieselParticulateFilterDifferentialPressure  = 0x17;
    static const SMLK_UINT8 EngineIntakeManifold1Pressure       = 0x18;
    static const SMLK_UINT8 EngineIntakeManifold1Temperature    = 0x19;
    static const SMLK_UINT8 EngineAirInletPressure              = 0x1A;
    static const SMLK_UINT8 BarometricPressure                  = 0x1C;
    static const SMLK_UINT8 CabInteriorTemperature              = 0x1D;
    static const SMLK_UINT8 AmbientAirTemperature               = 0x1E;
    static const SMLK_UINT8 EngineAirInletTemperature           = 0x1F;
    static const SMLK_UINT8 EngineWaitToStartLamp               = 0x20;
    static const SMLK_UINT8 WaterInFuelIndicator                = 0x23;
    static const SMLK_UINT8 TransmissionSelectedGear            = 0x24;
    static const SMLK_UINT8 TransmissionActualGearRatio         = 0x25;
    static const SMLK_UINT8 TransmissionCurrentGear             = 0x26;
    static const SMLK_UINT8 FuelLevel                           = 0x27;
    static const SMLK_UINT8 TripDistance                        = 0x28;
    static const SMLK_UINT8 TotalVehicleDistance                = 0x29;
    static const SMLK_UINT8 CalculusDistance                    = 0x2A;
    static const SMLK_UINT8 CalculusFuelUsed                    = 0x2B;
    static const SMLK_UINT8 ActualCatalystRate                  = 0x2C;
    static const SMLK_UINT8 TotalCatalystRate                   = 0x2D;
    static const SMLK_UINT8 Battery1Temperature                 = 0x2E;
    static const SMLK_UINT8 Battery1Voltage                     = 0x2F;
    static const SMLK_UINT8 Battery1SOC                         = 0x30;
    static const SMLK_UINT8 Battery1SOH                         = 0x31;
    static const SMLK_UINT8 Battery2Temperature                 = 0x32;
    static const SMLK_UINT8 Battery2Voltage                     = 0x33;
    static const SMLK_UINT8 Battery2SOC                         = 0x34;
    static const SMLK_UINT8 Battery2SOH                         = 0x35;
    static const SMLK_UINT8 ActualMinimumSOC                    = 0x36;
    static const SMLK_UINT8 ActualMinimumSOH                    = 0x37;
    static const SMLK_UINT8 AgtatorSpeed                        = 0x38;
    static const SMLK_UINT8 AgtatorState                        = 0x39;
    static const SMLK_UINT8 AgtatorTotalRotations               = 0x3A;

    // 0200 E7 pid
    static const SMLK_UINT8 CatalystTankTemperaturePID          = 0x01;
    static const SMLK_UINT8 EnginePowerInput1PID                = 0x02;
    static const SMLK_UINT8 KeyswitchBatteryVoltagePID          = 0x03;
    static const SMLK_UINT8 EngineExhaustGasTempPID             = 0x04;

    static const SMLK_UINT8 Sensor1HumidityPID                  = 0x07;
    static const SMLK_UINT8 Sensor2HumidityPID                  = 0x08;
    static const SMLK_UINT8 Sensor3HumidityPID                  = 0x09;
    static const SMLK_UINT8 Sensor4HumidityPID                  = 0x0A;
    static const SMLK_UINT8 Sensor5HumidityPID                  = 0x0B;
    static const SMLK_UINT8 Sensor6HumidityPID                  = 0x0C;
    static const SMLK_UINT8 Sensor7HumidityPID                  = 0x0D;
    static const SMLK_UINT8 Sensor8HumidityPID                  = 0x0E;
    static const SMLK_UINT8 Sensor1temperaturePID               = 0x0F;
    static const SMLK_UINT8 Sensor2temperaturePID               = 0x10;
    static const SMLK_UINT8 Sensor3temperaturePID               = 0x11;
    static const SMLK_UINT8 Sensor4temperaturePID               = 0x12;
    static const SMLK_UINT8 Sensor5temperaturePID               = 0x13;
    static const SMLK_UINT8 Sensor6temperaturePID               = 0x14;
    static const SMLK_UINT8 Sensor7temperaturePID               = 0x15;
    static const SMLK_UINT8 Sensor8temperaturePID               = 0x16;
    static const SMLK_UINT8 PefrigerationSwitchStatusPID        = 0x17;
    static const SMLK_UINT8 PefrigerationWorkModePID            = 0x18;
    static const SMLK_UINT8 PefrigerationCpmpartmentPemperaturePID               = 0x19;
    static const SMLK_UINT8 DefrostTemperaturePID               = 0x1A;
    static const SMLK_UINT8 SupplyVoltagePID                    = 0x1B;
    // update add 2022-02-15 0200-e7
    static const SMLK_UINT8 DcAcPowerConsumptionPID             = 0x1C;
    static const SMLK_UINT8 SteerWheelHeatPowerConsumption      = 0x1D;
    static const SMLK_UINT8 CigarLighterPowerConsumption        = 0x1E;
    static const SMLK_UINT8 Socket1PowerConsumption             = 0x1F;
    static const SMLK_UINT8 Socket2PowerConsumption             = 0x20;
    static const SMLK_UINT8 LeftSeatHeatPowerConsumption        = 0x21;
    static const SMLK_UINT8 RightSeatHeatPowerConsumption       = 0x22;
    static const SMLK_UINT8 LowerBerthtHeatPowerConsumption     = 0x23;
    static const SMLK_UINT8 SleepReadLampPowerConsumption       = 0x24;
    static const SMLK_UINT8 ElevatedBoxPowerConsumption         = 0x25;
    static const SMLK_UINT8 HumidityInBox1                      = 0x26;
    static const SMLK_UINT8 HumidityInBox2                      = 0x27;
    static const SMLK_UINT8 TemperatureInBox1                   = 0x28;
    static const SMLK_UINT8 TemperatureInBox2                   = 0x29;
    // add 0200-ec 2020-0308
    static const SMLK_UINT8 TboxVlotage                         = 0x01;
    static const SMLK_UINT8 FuelSaveSwitch                      = 0x02;
    static const SMLK_UINT8 HydraulicMotorOilReturnTemperature  = 0x03;
    static const SMLK_UINT8 EvVehicleStatus                     = 0x04;
    static const SMLK_UINT8 EvChargingStatus                    = 0x05;
    static const SMLK_UINT8 EvWorkingStatus                     = 0x06;
    static const SMLK_UINT8 EvVehicleSpeed                      = 0x07;
    static const SMLK_UINT8 EvTotalVoltage                      = 0x08;
    static const SMLK_UINT8 EvTotalCurrent                      = 0x09;
    static const SMLK_UINT8 EvBatterySoc                        = 0x0A;
    static const SMLK_UINT8 EvDcDcWoringStatus                  = 0x0B;
    static const SMLK_UINT8 EvGearStatus                        = 0x0C;
    static const SMLK_UINT8 EvInsulationResistance              = 0x0D;

};

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


static int TimeStamp2BCD(IN std::time_t tt, INOUT SMLK_BCD *BCD, IN std::size_t sz) {
    if ( (nullptr == BCD) || (sz != M_SL_TIMESTAMP_SZ) ) {
        SMLK_LOGE("TimeStamp2BCD FAIL!!!");
        return -1;
    }
    std::tm tm;
    localtime_r(&tt, &tm);
    // check whether src time is invalid
    std::time_t local_time;
    if (tt == 0) {
        SMLK_LOGD("timestamp src = 0");
        std::memset(BCD, 0x0, sz);
        return 0;
    } else {
        localtime_r(&tt, &tm);
    }

    //std::memcpy(&tm, std::localtime(&tt), sizeof(tm));

    tm.tm_year -= 100;      // years since 2000
    tm.tm_mon += 1;

    BCD[0] = ((((SMLK_UINT8)(tm.tm_year / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_year % 10));

    BCD[1] = ((((SMLK_UINT8)(tm.tm_mon  / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_mon  % 10));

    BCD[2] = ((((SMLK_UINT8)(tm.tm_mday / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_mday % 10));
    BCD[3] = ((((SMLK_UINT8)(tm.tm_hour / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_hour % 10));
    BCD[4] = ((((SMLK_UINT8)(tm.tm_min  / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_min  % 10));
    BCD[5] = ((((SMLK_UINT8)(tm.tm_sec  / 10)) & 0xF) << 4) | ((SMLK_UINT8)(tm.tm_sec  % 10));
#ifdef SL_DEBUG
    SMLK_LOGD("[timedebug] BCD[0] is == %2x ", BCD[0]);
    SMLK_LOGD("[timedebug] BCD[1] is == %2x ", BCD[1]);
#endif
    return 0;
}

MessageLocation808::MessageLocation808()
{}

MessageLocation808::~MessageLocation808()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION: encode the message body to bytes
 *
 * PARAMETERS:
 *     data: buffer to handle the result
 * RETURN:
 *      0: success
 *  other:
 * NOTE:
 *
 *****************************************************************************/
std::uint32_t   MessageLocation808::Encode(std::vector<SMLK_UINT8> &data)
{
    data.clear();

    SMLK_BCD    timestamp[M_SL_TIMESTAMP_SZ];
    decltype(m_location.m_latitude)     latitude(m_location.m_latitude);
    decltype(m_location.m_longitude)    longitude(m_location.m_longitude);
    decltype(m_location.m_speed)        speed(m_location.m_speed);
    decltype(m_location.m_altitude)     altitude(m_location.m_altitude);
    decltype(m_location.m_heading)      heading(m_location.m_heading);

    /*
    bit 0:  ACC status
    bit 1:  located status
    bit 2:  sourth latitude
    bit 3:  west longitude
    others: reserved for now
    */
    SMLK_UINT32 status = 0x00000000;

    if ( m_acc_status ) {
        status |= ((SMLK_UINT32)1) << 0;
        SMLK_LOGD("============== acc on");
    }
    else
    {
        SMLK_LOGD("============== acc off");
    }

    if ( m_location.m_status.Encoded() & 0x01 ) {
        // location fix not vaild
        altitude    = 0;
        speed       = 0;
        heading     = 0;
    } else {
        status |= ((SMLK_UINT32)1) << 1;
    }

    if ( latitude.Val() < 0 ) {
        status |= ((SMLK_UINT32)1) << 2;
        latitude = -latitude.Val();
    } else if ( !latitude.Valid() ) {
        latitude = latitude.Val();
    }

    if ( longitude.Val() < 0 ) {
        status |= ((SMLK_UINT32)1) << 3;
        longitude   = -longitude.Val();
    } else if ( longitude.Valid() ) {
        longitude = longitude.Val();
    }

    TimeStamp2BCD((std::time_t)m_location.m_timestamp.Val(), timestamp, sizeof(timestamp));

    PUSH_U4(m_alarm_flags.Encoded()); // 报警标志位
    PUSH_U4(status);
    PUSH_U4(latitude.Encoded());
    PUSH_U4(longitude.Encoded());
    PUSH_U2((SMLK_UINT16)altitude.Val());
    PUSH_U2(speed.Encoded());
    PUSH_U2(heading.Encoded());
    PUSH_AR(timestamp);

#ifdef SL_DEBUG
    SMLK_LOGD("m_report_location_only is == %d", (SMLK_UINT8)m_report_location_only);
#endif

    if ( m_report_location_only ) {
        SMLK_LOGI("[wakeupdebug]  0200 report emmc data");

        // 0x01
        PUSH_U1(EXT_MILAGE);
        PUSH_U1(0x04);
        // PUSH_U4(m_instrument_total_mileage.Encoded());
        if ( m_instrument_total_mileage_head_ext1.Valid() ) {
            PUSH_U4(m_instrument_total_mileage_head_ext1.Encoded());
        } else {
            decltype(m_instrument_total_mileage_head_ext1)    instrument_total_mileage(m_instrument_total_mileage_head_ext1);
            instrument_total_mileage = m_instrument_total_mileage_head_ext1.Val();
            PUSH_U4(instrument_total_mileage.Encoded());
        }

    #if 0       // for now, do not report extended signal status
        // 0x25
        PUSH_U1(MessageLocation::EXT_SIGNAL_STATUS);
        PUSH_U1(0x04);
        PUSH_U4(m_extended_signal_status.Encoded());
    #endif

        // 0x30
        PUSH_U1(MessageLocation::EXT_RSSI);
        PUSH_U1(0x01);
        PUSH_U1(m_rssi.Encoded());

        // 0x31
        PUSH_U1(MessageLocation::EXT_GNSS_SATELLITES);
        PUSH_U1(0x01);
        PUSH_U1(m_satellites_used.Encoded());

        // 0xE6
        PUSH_U1(MessageLocation::EXT_E6);

        std::size_t szf = data.size();

        PUSH_U1(0x00);                                      // extent E6 size placeholder

        // 总油耗
        PUSH_U2(PID::EngineTotalFuelUsed);
        PUSH_U1(sizeof(SMLK_UINT32));
        if ( m_engine_total_fuel_used.Valid() ) {
            PUSH_U4(m_engine_total_fuel_used.Encoded());
        } else {
            decltype(m_engine_total_fuel_used)  engine_total_fuel_used(m_engine_total_fuel_used);
            engine_total_fuel_used = m_engine_total_fuel_used.Val();
            PUSH_U4(engine_total_fuel_used.Encoded());
        }

        // 尿素液位
        PUSH_U2(PID::CatalystTankLevel);
        PUSH_U1(sizeof(SMLK_UINT8));
        if ( m_urea_tank_level.Valid() ) {
            PUSH_U1(m_urea_tank_level.Encoded());
        } else {
            decltype(m_urea_tank_level) catalyst_tank_level(m_urea_tank_level);
            catalyst_tank_level = m_urea_tank_level.Val();
            PUSH_U1(catalyst_tank_level.Encoded());
        }

        // 发动机运行总时长
        PUSH_U2(PID::EngineTotalHoursOfOperation);
        PUSH_U1(sizeof(SMLK_UINT32));
        if ( m_engine_total_time.Valid() ) {
            PUSH_U4(m_engine_total_time.Encoded());
        } else {
            decltype(m_engine_total_time)   engine_total_time(m_engine_total_time);
            engine_total_time = m_engine_total_time.Val();
            PUSH_U4(engine_total_time.Encoded());
        }

        // 发动机累积运行转数
        PUSH_U2(PID::EngineTotalRevolutions);
        PUSH_U1(sizeof(SMLK_UINT32));
        if ( m_engine_total_cycles.Valid() ) {
            PUSH_U4(m_engine_total_cycles.Encoded());
        } else {
            decltype(m_engine_total_cycles) engine_total_cycles(m_engine_total_cycles);
            engine_total_cycles = m_engine_total_cycles.Val();
            PUSH_U4(engine_total_cycles.Encoded());
        }

        // 油箱液位
        PUSH_U2(PID::FuelLevel);
        PUSH_U1(sizeof(SMLK_UINT8));
        if ( m_instrument_fuel_level.Valid() ) {
            PUSH_U1(m_instrument_fuel_level.Encoded());
        } else {
            decltype(m_instrument_fuel_level)   instrument_fuel_level(m_instrument_fuel_level);
            instrument_fuel_level = m_instrument_fuel_level.Val();
            PUSH_U1(instrument_fuel_level.Encoded());
        }
        // 仪表总里程
            PUSH_U2(PID::TotalVehicleDistance);
            PUSH_U1(sizeof(SMLK_UINT32));
            if ( m_instrument_total_mileage.Valid() ) {
                PUSH_U4(m_instrument_total_mileage.Encoded());
            } else {
                decltype(m_instrument_total_mileage)    instrument_total_mileage(m_instrument_total_mileage);
                instrument_total_mileage = m_instrument_total_mileage.Val();
                PUSH_U4(instrument_total_mileage.Encoded());
            }

            // 积分里程
            PUSH_U2(PID::CalculusDistance);
            PUSH_U1(sizeof(SMLK_UINT32));
            PUSH_U4(m_calculus_distance.Encoded());

            // 积分油耗
            PUSH_U2(PID::CalculusFuelUsed);
            PUSH_U1(sizeof(SMLK_UINT32));
            PUSH_U4(m_calculus_fuel_used.Encoded());

            // 累计尿素消耗量
            PUSH_U2(PID::TotalCatalystRate);
            PUSH_U1(sizeof(SMLK_UINT32));
            if ( m_total_catalyst_spewed.Valid() ) {
                PUSH_U4(m_total_catalyst_spewed.Encoded());
            } else {
                decltype(m_total_catalyst_spewed)   total_catalyst_spewed(m_total_catalyst_spewed);
                total_catalyst_spewed   = m_total_catalyst_spewed.Val();
                PUSH_U4(total_catalyst_spewed.Encoded());
            }
            // fix the E6 data size
            data[szf] = (SMLK_UINT8)(data.size() - szf - 1);
            return SL_SUCCESS;
    }

    // 0x01
    PUSH_U1(EXT_MILAGE);
    PUSH_U1(0x04);
    // PUSH_U4(m_instrument_total_mileage.Encoded());
    if ( m_instrument_total_mileage_head_ext1.Valid() ) {
        PUSH_U4(m_instrument_total_mileage_head_ext1.Encoded());
    } else {
        decltype(m_instrument_total_mileage_head_ext1)    instrument_total_mileage(m_instrument_total_mileage_head_ext1);
        instrument_total_mileage = m_instrument_total_mileage_head_ext1.Val();
        PUSH_U4(instrument_total_mileage.Encoded());
    }
    // 0x12
    PUSH_U1(MessageLocation::EXT_ENTER_LEAVE_AREA);
    PUSH_U1(0x06);
    PUSH_U1(m_region.m_type);
    PUSH_U4(m_region.m_id);
    PUSH_U1(m_region.m_direction);

#if 0       // for now, do not report extended signal status
    // 0x25
    PUSH_U1(MessageLocation::EXT_SIGNAL_STATUS);
    PUSH_U1(0x04);
    PUSH_U4(m_extended_signal_status.Encoded());
#endif

    // 0x30
    PUSH_U1(MessageLocation::EXT_RSSI);
    PUSH_U1(0x01);
    PUSH_U1(m_rssi.Encoded());

    // 0x31
    PUSH_U1(MessageLocation::EXT_GNSS_SATELLITES);
    PUSH_U1(0x01);
    PUSH_U1(m_satellites_used.Encoded());

    // 0xE6
    PUSH_U1(MessageLocation::EXT_E6);

    std::size_t szf = data.size();

    PUSH_U1(0x00);                                      // extent E6 size placeholder

    if ( m_tachograph_vehicle_speed.Valid() ) {
        PUSH_U2(PID::TachographVehicleSpeed);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_tachograph_vehicle_speed.Encoded());
    }

    if ( m_engine_torque_mode.Valid() ) {
        PUSH_U2(PID::EngineTorqueMode);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_engine_torque_mode.Encoded());
    }

    if ( m_torque_percent_cmd.Valid() ) {
        PUSH_U2(PID::DriverDemandEnginePercentTorque);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_torque_percent_cmd.Encoded());
    }

    if ( m_torque_percent.Valid() ) {
        PUSH_U2(PID::ActualEnginePercentTorque);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_torque_percent.Encoded());
    }

    if ( m_engine_speed.Valid() ) {
        PUSH_U2(PID::EngineSpeed);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_engine_speed.Encoded());
    }

    if ( m_coolant_temperature.Valid() ) {
        PUSH_U2(PID::EngineCoolantTemperature);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_coolant_temperature.Encoded());
    }

    if ( m_oil_pressure.Valid() ) {
        PUSH_U2(PID::EngineOilPressure);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_oil_pressure.Encoded());
    }

    // 总油耗
    PUSH_U2(PID::EngineTotalFuelUsed);
    PUSH_U1(sizeof(SMLK_UINT32));
    if ( m_engine_total_fuel_used.Valid() ) {
        PUSH_U4(m_engine_total_fuel_used.Encoded());
    } else {
        decltype(m_engine_total_fuel_used)  engine_total_fuel_used(m_engine_total_fuel_used);
        engine_total_fuel_used = m_engine_total_fuel_used.Val();
        PUSH_U4(engine_total_fuel_used.Encoded());
    }

    if ( m_acc_pedal_travel.Valid() ) {
        PUSH_U2(PID::AcceleratorPedalPosition1);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_acc_pedal_travel.Encoded());
    }

    if ( m_parking.Valid() ) {
        PUSH_U2(PID::ParkingBrakeSwitch);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_parking.Encoded());
    }

    if ( m_wheel_speed.Valid() ) {
        PUSH_U2(PID::WheelBasedVehicleSpeed);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_wheel_speed.Encoded());
    }

    if ( m_clutch.Valid() ) {
        PUSH_U2(PID::ClutchSwitch);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_clutch.Encoded());
    }

    if ( m_brake_pedal_state.Valid() ) {
        PUSH_U2(PID::BrakeSwitch);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_brake_pedal_state.Encoded());
    }

    // 尿素液位
    PUSH_U2(PID::CatalystTankLevel);
    PUSH_U1(sizeof(SMLK_UINT8));
    if ( m_urea_tank_level.Valid() ) {
        PUSH_U1(m_urea_tank_level.Encoded());
    } else {
        decltype(m_urea_tank_level) catalyst_tank_level(m_urea_tank_level);
        catalyst_tank_level = m_urea_tank_level.Val();
        PUSH_U1(catalyst_tank_level.Encoded());
    }

    // 发动机运行总时长
    PUSH_U2(PID::EngineTotalHoursOfOperation);
    PUSH_U1(sizeof(SMLK_UINT32));
    if ( m_engine_total_time.Valid() ) {
        PUSH_U4(m_engine_total_time.Encoded());
    } else {
        decltype(m_engine_total_time)   engine_total_time(m_engine_total_time);
        engine_total_time = m_engine_total_time.Val();
        PUSH_U4(engine_total_time.Encoded());
    }

    // 发动机累积运行转数
    PUSH_U2(PID::EngineTotalRevolutions);
    PUSH_U1(sizeof(SMLK_UINT32));
    if ( m_engine_total_cycles.Valid() ) {
        PUSH_U4(m_engine_total_cycles.Encoded());
    } else {
        decltype(m_engine_total_cycles) engine_total_cycles(m_engine_total_cycles);
        engine_total_cycles = m_engine_total_cycles.Val();
        PUSH_U4(engine_total_cycles.Encoded());
    }

    if ( m_engine_fuel_consumption_rate.Valid() ) {
        PUSH_U2(PID::EngineFuelRate);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_engine_fuel_consumption_rate.Encoded());
    }

    if ( m_engine_instantaneous_fuel_consumption.Valid() ) {
        PUSH_U2(PID::EngineInstantaneousFuelEconomy);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_engine_instantaneous_fuel_consumption.Encoded());
    }

    if ( m_engine_average_fuel_consumption.Valid() ) {
        PUSH_U2(PID::EngineAverageFuelEconomy);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_engine_average_fuel_consumption.Encoded());
    }

    if ( m_dpf_pressure_differential.Valid() ) {
        PUSH_U2(PID::Aftertreatment1DieselParticulateFilterDifferentialPressure);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_dpf_pressure_differential.Encoded());
    }

    if ( m_intake_manifold_pressure.Valid() ) {
        PUSH_U2(PID::EngineIntakeManifold1Pressure);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_intake_manifold_pressure.Encoded());
    }

    if ( m_intake_manifole_temprerature.Valid() ) {
        PUSH_U2(PID::EngineIntakeManifold1Temperature);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_intake_manifole_temprerature.Encoded());
    }

    if ( m_engine_intake_pressure.Valid() ) {
        PUSH_U2(PID::EngineAirInletPressure);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_engine_intake_pressure.Encoded());
    }

    if ( m_barometric_pressure.Valid() ) {
        PUSH_U2(PID::BarometricPressure);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_barometric_pressure.Encoded());
    }

    if ( m_cab_interior_temperature.Valid() ) {
        PUSH_U2(PID::CabInteriorTemperature);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_cab_interior_temperature.Encoded());
    }

    if ( m_ambient_air_temperature.Valid() ) {
        PUSH_U2(PID::AmbientAirTemperature);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_ambient_air_temperature.Encoded());
    }

    if ( m_engine_intake_temperature.Valid() ) {
        PUSH_U2(PID::EngineAirInletTemperature);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_engine_intake_temperature.Encoded());
    }

    if ( m_cold_start_light.Valid() ) {
        PUSH_U2(PID::EngineWaitToStartLamp);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_cold_start_light.Encoded());
    }

    if ( m_water_in_fuel_flag.Valid() ) {
        PUSH_U2(PID::WaterInFuelIndicator);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_water_in_fuel_flag.Encoded());
    }

    if ( m_target_gear.Valid() ) {
        PUSH_U2(PID::TransmissionSelectedGear);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_target_gear.Encoded());
    }

    if ( m_gear_ratio.Valid() ) {
        PUSH_U2(PID::TransmissionActualGearRatio);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_gear_ratio.Encoded());
    }

    if ( m_gear.Valid() ) {
        PUSH_U2(PID::TransmissionCurrentGear);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_gear.Encoded());
    } else {
        SMLK_LOGD("[GearDebug] m_gear invalid and value %d ", m_gear.Val());
    }

    // 油箱液位
    PUSH_U2(PID::FuelLevel);
    PUSH_U1(sizeof(SMLK_UINT8));
    if ( m_instrument_fuel_level.Valid() ) {
        PUSH_U1(m_instrument_fuel_level.Encoded());
    } else {
        decltype(m_instrument_fuel_level)   instrument_fuel_level(m_instrument_fuel_level);
        instrument_fuel_level = m_instrument_fuel_level.Val();
        PUSH_U1(instrument_fuel_level.Encoded());
    }

    if ( m_instrument_subtotal_mileage.Valid() ) {
        PUSH_U2(PID::TripDistance);
        PUSH_U1(sizeof(SMLK_UINT32));
        PUSH_U4(m_instrument_subtotal_mileage.Encoded());
    }

    // 仪表总里程
    PUSH_U2(PID::TotalVehicleDistance);
    PUSH_U1(sizeof(SMLK_UINT32));
    if ( m_instrument_total_mileage.Valid() ) {
        PUSH_U4(m_instrument_total_mileage.Encoded());
    } else {
        decltype(m_instrument_total_mileage)    instrument_total_mileage(m_instrument_total_mileage);
        instrument_total_mileage = m_instrument_total_mileage.Val();
        PUSH_U4(instrument_total_mileage.Encoded());
    }

    // 积分里程
    PUSH_U2(PID::CalculusDistance);
    PUSH_U1(sizeof(SMLK_UINT32));
    PUSH_U4(m_calculus_distance.Encoded());

    // 积分油耗
    PUSH_U2(PID::CalculusFuelUsed);
    PUSH_U1(sizeof(SMLK_UINT32));
    PUSH_U4(m_calculus_fuel_used.Encoded());

    if ( m_actural_catalyst_spewed.Valid() ) {
        PUSH_U2(PID::ActualCatalystRate);
        PUSH_U1(sizeof(SMLK_UINT32));
        PUSH_U4(m_actural_catalyst_spewed.Encoded());
    }

    // 累计尿素消耗量
    PUSH_U2(PID::TotalCatalystRate);
    PUSH_U1(sizeof(SMLK_UINT32));
    if ( m_total_catalyst_spewed.Valid() ) {
        PUSH_U4(m_total_catalyst_spewed.Encoded());
    } else {
        decltype(m_total_catalyst_spewed)   total_catalyst_spewed(m_total_catalyst_spewed);
        total_catalyst_spewed   = m_total_catalyst_spewed.Val();
        PUSH_U4(total_catalyst_spewed.Encoded());
    }

    if ( m_battery_1_temperature.Valid() ) {
        PUSH_U2(PID::Battery1Temperature);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_battery_1_temperature.Encoded());
    }

    if ( m_battery_1_vol.Valid() ) {
        PUSH_U2(PID::Battery1Voltage);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_battery_1_vol.Encoded());
    }

    if ( m_battery_1_soc.Valid() ) {
        PUSH_U2(PID::Battery1SOC);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_battery_1_soc.Encoded());
    }

    if ( m_battery_1_soh.Valid() ) {
        PUSH_U2(PID::Battery1SOH);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_battery_1_soh.Encoded());
    }

    if ( m_battery_2_temperature.Valid() ) {
        PUSH_U2(PID::Battery2Temperature);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_battery_2_temperature.Encoded());
    }

    if ( m_battery_2_vol.Valid() ) {
        PUSH_U2(PID::Battery2Voltage);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_battery_2_vol.Encoded());
    }

    if ( m_battery_2_soc.Valid() ) {
        PUSH_U2(PID::Battery2SOC);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_battery_2_soc.Encoded());
    }

    if ( m_battery_2_soh.Valid() ) {
        PUSH_U2(PID::Battery2SOH);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_battery_2_soh.Encoded());
    }

    if ( m_actual_minimum_soc.Valid() ) {
        PUSH_U2(PID::ActualMinimumSOC);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_actual_minimum_soc.Encoded());
    }

    if ( m_actual_minimum_soh.Valid() ) {
        PUSH_U2(PID::ActualMinimumSOH);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_actual_minimum_soh.Encoded());
    }

    if ( m_agtator_speed.Valid() ) {
        PUSH_U2(PID::AgtatorSpeed);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_agtator_speed.Encoded());
    }

    if ( m_agtator_state.Valid() ) {
        PUSH_U2(PID::AgtatorState);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_agtator_state.Encoded());
    }

     if ( m_agtator_total_rotations.Valid() ) {
        PUSH_U2(PID::AgtatorTotalRotations);
        PUSH_U1(sizeof(SMLK_UINT32));
        PUSH_U4(m_agtator_total_rotations.Encoded());
    }

    // fix the E6 data size
    data[szf] = (SMLK_UINT8)(data.size() - szf - 1);

    //0xE7

    PUSH_U1(MessageLocation::EXT_E7);
    szf = data.size();
    PUSH_U1(0x00);                                      // extent E7 size placeholder

    //尿素罐温度
    if ( m_catalyst_tank_temperature.Valid() ) {
        PUSH_U2(PID::CatalystTankTemperaturePID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_catalyst_tank_temperature.Encoded());
    }
    //发动机输入电压
    if ( m_engine_power_input1.Valid() ) {
        PUSH_U2(PID::EnginePowerInput1PID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_engine_power_input1.Encoded());
    }
    //点火开关电压
    if ( m_keyswitch_battery_voltage.Valid() ) {
        PUSH_U2(PID::KeyswitchBatteryVoltagePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_keyswitch_battery_voltage.Encoded());
    }
    //发动机排气温度
    if ( m_exhaust_gas_temperature.Valid() ) {
        PUSH_U2(PID::EngineExhaustGasTempPID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_exhaust_gas_temperature.Encoded());
    }
    //传感器1湿度
    if ( m_sensor1_humidity.Valid() ) {
        PUSH_U2(PID::Sensor1HumidityPID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_sensor1_humidity.Encoded());
    }
    //传感器2湿度
    if ( m_sensor2_humidity.Valid() ) {
        PUSH_U2(PID::Sensor2HumidityPID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_sensor2_humidity.Encoded());
    }
    //传感器3湿度
    if ( m_sensor3_humidity.Valid() ) {
        PUSH_U2(PID::Sensor3HumidityPID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_sensor3_humidity.Encoded());
    }
    //传感器4湿度
    if ( m_sensor4_humidity.Valid() ) {
        PUSH_U2(PID::Sensor4HumidityPID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_sensor4_humidity.Encoded());
    }
    //传感器5湿度
    if ( m_sensor5_humidity.Valid() ) {
        PUSH_U2(PID::Sensor5HumidityPID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_sensor5_humidity.Encoded());
    }
    //传感器6湿度
    if ( m_sensor6_humidity.Valid() ) {
        PUSH_U2(PID::Sensor6HumidityPID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_sensor6_humidity.Encoded());
    }
    //传感器7湿度
    if ( m_sensor7_humidity.Valid() ) {
        PUSH_U2(PID::Sensor7HumidityPID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_sensor7_humidity.Encoded());
    }
    //传感器8湿度
    if ( m_sensor8_humidity.Valid() ) {
        PUSH_U2(PID::Sensor8HumidityPID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_sensor8_humidity.Encoded());
    }
    //传感器1温度
    if ( m_sensor1_temperature.Valid() ) {
        PUSH_U2(PID::Sensor1temperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_sensor1_temperature.Encoded());
    }
    //传感器2温度
    if ( m_sensor2_temperature.Valid() ) {
        PUSH_U2(PID::Sensor2temperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_sensor2_temperature.Encoded());
    }
    //传感器3温度
    if ( m_sensor3_temperature.Valid() ) {
        PUSH_U2(PID::Sensor3temperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_sensor3_temperature.Encoded());
    }
    //传感器4温度
    if ( m_sensor4_temperature.Valid() ) {
        PUSH_U2(PID::Sensor4temperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_sensor4_temperature.Encoded());
    }
    //传感器5温度
    if ( m_sensor5_temperature.Valid() ) {
        PUSH_U2(PID::Sensor5temperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_sensor5_temperature.Encoded());
    }
    //传感器6温度
    if ( m_sensor6_temperature.Valid() ) {
        PUSH_U2(PID::Sensor6temperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_sensor6_temperature.Encoded());
    }
    //传感器7温度
    if ( m_sensor7_temperature.Valid() ) {
        PUSH_U2(PID::Sensor7temperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_sensor7_temperature.Encoded());
    }
    //传感器8温度
    if ( m_sensor8_temperature.Valid() ) {
        PUSH_U2(PID::Sensor8temperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_sensor8_temperature.Encoded());
    }

    //制冷机组开关状态
    if ( m_pefrigeration_switch_status.Valid() ) {
        PUSH_U2(PID::PefrigerationSwitchStatusPID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_pefrigeration_switch_status.Encoded());
    }
    //制冷机组工作模式
    if ( m_pefrigeration_work_mode.Valid() ) {
        PUSH_U2(PID::PefrigerationWorkModePID);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_pefrigeration_work_mode.Encoded());
    }
    //冷藏室温度
    if ( m_pefrigeration_cpmpartment_temperature.Valid() ) {
        PUSH_U2(PID::PefrigerationCpmpartmentPemperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_pefrigeration_cpmpartment_temperature.Encoded());
    }
    //除霜温度
    if ( m_defrost_temperature.Valid() ) {
        PUSH_U2(PID::DefrostTemperaturePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_defrost_temperature.Encoded());
    }
    /*
    //供电电压
    if ( m_supply_voltage.Valid() ) {
        PUSH_U2(PID::SupplyVoltagePID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_supply_voltage.Encoded());
    }
    */
    // update add 2022-02-15 0200-e7
    // 逆变器功率

    if ( m_dc_ac_power_consumption.Valid() ) {
        PUSH_U2(PID::DcAcPowerConsumptionPID);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_dc_ac_power_consumption.Encoded());
    }

    // 方向盘加热功率
    if ( m_steerwheel_heat_power_consumption.Valid() ) {
        PUSH_U2(PID::SteerWheelHeatPowerConsumption);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_steerwheel_heat_power_consumption.Encoded());
    }

    // 点烟器功率
    if ( m_cigar_light_power_consumption.Valid() ) {
        PUSH_U2(PID::CigarLighterPowerConsumption);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_cigar_light_power_consumption.Encoded());
    }

    //  供电插座 1 功率
    if ( m_24v_socket1_power_consumption.Valid() ) {
        PUSH_U2(PID::Socket1PowerConsumption);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_24v_socket1_power_consumption.Encoded());
    }

    //  供电插座 2 功率
    if ( m_24v_socket2_power_consumption.Valid() ) {
        PUSH_U2(PID::Socket2PowerConsumption);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_24v_socket2_power_consumption.Encoded());
    }

    // 驾驶员座椅加热功率
    if ( m_left_seat_power_consumption.Valid() ) {
        PUSH_U2(PID::LeftSeatHeatPowerConsumption);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_left_seat_power_consumption.Encoded());
    }

    if ( m_right_seat_power_consumption.Valid() ) {
        PUSH_U2(PID::RightSeatHeatPowerConsumption);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_right_seat_power_consumption.Encoded());
    }

    if ( m_lower_bretht_heat_power_consumption.Valid() ) {
        PUSH_U2(PID::LowerBerthtHeatPowerConsumption);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_lower_bretht_heat_power_consumption.Encoded());
    }

    if ( m_sleep_read_lamp_power_consumption.Valid() ) {
        PUSH_U2(PID::SleepReadLampPowerConsumption);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_sleep_read_lamp_power_consumption.Encoded());
    }

    if ( m_elevated_box_power_consumption.Valid() ) {
        PUSH_U2(PID::ElevatedBoxPowerConsumption);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_elevated_box_power_consumption.Encoded());
    }

    if ( m_humidity_in_box1.Valid() ) {
        PUSH_U2(PID::HumidityInBox1);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_humidity_in_box1.Encoded());
    }

    if ( m_humidity_in_box2.Valid() ) {
        PUSH_U2(PID::HumidityInBox2);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_humidity_in_box2.Encoded());
    }

    if ( m_temperature_in_box1.Valid() ) {
        PUSH_U2(PID::TemperatureInBox1);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_temperature_in_box1.Encoded());
    }

    if ( m_temperature_in_box2.Valid() ) {
        PUSH_U2(PID::TemperatureInBox2);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_temperature_in_box2.Encoded());
    }

    data[szf] = (SMLK_UINT8)(data.size() - szf - 1);

    // 0xEB
    if ( m_tires.size() > 0xFF ) {
        m_tires.resize(0xFF);
    }
    PUSH_U1(MessageLocation::EXT_EB);
    szf = data.size();
    PUSH_U1(0x00);                                      // extent EB size placeholder
    PUSH_U1(m_tires.size());

    for ( auto const &tire : m_tires ) {
#ifdef SL_DEBUG
        SMLK_LOGE("[tiredebug] m_tire.id is %2x", tire.m_id);
        SMLK_LOGE("[tiredebug] tire.m_temperature is %2x", tire.m_temperature.Encoded());
        SMLK_LOGE("[tiredebug] tire.m_pressure is %2x", tire.m_pressure.Encoded());
        SMLK_LOGE("[tiredebug] tire.m_extend_pressure %2x", tire.m_extend_pressure.Encoded());
#endif
        PUSH_U1(tire.m_id);
        PUSH_U2(tire.m_temperature.Encoded());
        PUSH_U1(tire.m_pressure.Encoded());
        PUSH_U2(tire.m_extend_pressure.Encoded());
    }

    // fix the EB data size
    data[szf] = (SMLK_UINT8)(data.size() - szf - 1);

    // 0xEC
    PUSH_U1(MessageLocation::EXT_EC);
    szf = data.size();
    PUSH_U1(0x00);                                      // extent EC size placeholder

    if ( m_tbox_voltage.Valid() ) {
        PUSH_U2(PID::TboxVlotage);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_tbox_voltage.Encoded());
    }

    if ( m_fuel_save_switch.Valid() ) {
        PUSH_U2(PID::FuelSaveSwitch);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_fuel_save_switch.Encoded());
    }

    if ( m_hydraulic_motor_oil_return_temperature.Valid() ) {
        PUSH_U2(PID::HydraulicMotorOilReturnTemperature);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_hydraulic_motor_oil_return_temperature.Encoded());
    }

    if ( m_ev_vehicle_status.Valid() ) {
        PUSH_U2(PID::EvVehicleStatus);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_ev_vehicle_status.Encoded());
    }

    if ( m_ev_charging_status.Valid() ) {
        PUSH_U2(PID::EvChargingStatus);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_ev_charging_status.Encoded());
    }

    if ( m_ev_working_status.Valid() ) {
        PUSH_U2(PID::EvWorkingStatus);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_ev_working_status.Encoded());
    }

    if ( m_ev_vehicle_speed.Valid() ) {
        PUSH_U2(PID::EvVehicleSpeed);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_ev_vehicle_speed.Encoded());
    }

    if ( m_ev_total_voltage.Valid() ) {
        PUSH_U2(PID::EvTotalVoltage);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_ev_total_voltage.Encoded());
    }

    if ( m_ev_total_current.Valid() ) {
        PUSH_U2(PID::EvTotalCurrent);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_ev_total_current.Encoded());
    }

    if ( m_ev_battery_soc.Valid() ) {
        PUSH_U2(PID::EvBatterySoc);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_ev_battery_soc.Encoded());
    }

    if ( m_ev_dc_working_status.Valid() ) {
        PUSH_U2(PID::EvDcDcWoringStatus);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_ev_dc_working_status.Encoded());
    }

    if ( m_ev_gear_status.Valid() ) {
        PUSH_U2(PID::EvGearStatus);
        PUSH_U1(sizeof(SMLK_UINT8));
        PUSH_U1(m_ev_gear_status.Encoded());
    }

    if ( m_ev_insulation_resistance.Valid() ) {
        PUSH_U2(PID::EvInsulationResistance);
        PUSH_U1(sizeof(SMLK_UINT16));
        PUSH_U2(m_ev_insulation_resistance.Encoded());
    }

    // fix the EC data size
    data[szf] = (SMLK_UINT8)(data.size() - szf - 1);

    return SL_SUCCESS;
}

std::uint32_t   MessageLocation808::Decode(IN std::vector<SMLK_UINT8> &)
{
    return SL_SUCCESS;
}

MessageBucket808::MessageBucket808()
{}

MessageBucket808::~MessageBucket808()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION: encode the message body to bytes
 *
 * PARAMETERS:
 *     data: buffer to handle the result
 * RETURN:
 *      0: success
 *  other:
 * NOTE:
 *
 *****************************************************************************/
std::uint32_t MessageBucket808::Encode(std::vector<SMLK_UINT8> &data)
{
    static const std::array<SMLK_UINT32, 33> bitsmasks = {
        0x00000000,
        0x00000001, 0x00000003, 0x00000007, 0x0000000F, 0x0000001F, 0x0000003F, 0x0000007F, 0x000000FF,
        0x000001FF, 0x000003FF, 0x000007FF, 0x00000FFF, 0x00001FFF, 0x00003FFF, 0x00007FFF, 0x0000FFFF,
        0x0001FFFF, 0x0003FFFF, 0x0007FFFF, 0x000FFFFF, 0x001FFFFF, 0x003FFFFF, 0x007FFFFF, 0x00FFFFFF,
        0x01FFFFFF, 0x03FFFFFF, 0x07FFFFFF, 0x0FFFFFFF, 0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF
    };

    data.clear();

    PUSH_U2(m_sampling_frequency);
    PUSH_U1(m_crompression);
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U1(0x00);

    std::size_t length_index = data.size();

    // content length
    PUSH_U1(0x00);
    PUSH_U1(0x00);

    PUSH_U2(m_signal_day_vehicle_distance.Encoded());

    // TAPD:1018384
    m_signal_total_vehicle_distance.SetValid();
    m_signal_ecu_total_fuel_used.SetValid();

    PUSH_U4(m_signal_total_vehicle_distance.Encoded());
    PUSH_U4(m_signal_ecu_total_distacne.Encoded());
    PUSH_U2(m_signal_day_fuel_used.Encoded());
    PUSH_U4(m_signal_total_fuel_used.Encoded());
    PUSH_U4(m_signal_ecu_total_fuel_used.Encoded());

    std::stringstream   ss;

    if ( 0 == m_signals.size() ) {
        return SL_EFAILED;
    }

    PUSH_U2(m_signals.size() + 1); //内容行数

    for ( auto const id : SignalIDs() ) {
        auto const it = m_signals[0].find(id);
        ss << std::dec << (it->second.ID() & 0x0007FFFF);
        if ( it->second.ID() & 0xFF000000 ) {
            ss << "_";
            ss << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (it->second.ID() >> 24);
        }
        ss << ",";
    }

    std::string head(ss.str());
    // remove last ','
    head.pop_back();

    data.insert(data.end(), head.begin(), head.end());

    for ( auto const &record : m_signals ) {
        // clear stream buffer for reuse
        ss.clear();
        ss.str(std::string());

        for ( auto const id : SignalIDs() ) {
            if ( vehicle::SignalID::GNSSTimeStamp == id ) {
                std::time_t timestamp = record.find(id)->second.Val();
                //struct tm tt = *std::localtime(&timestamp);
                struct tm tt;
                localtime_r(&timestamp, &tt);
                ss << std::setw(2) << std::setfill('0') << std::dec << (tt.tm_year - 100)%100;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_mon + 1;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_mday;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_hour;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_min;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_sec;
            } else if ( vehicle::SignalID::GNSSLocatedStatus == id ) {
                SMLK_UINT16                     status = 0;

                auto it = record.find(vehicle::SignalID::GNSSLocatedStatus);
                if ( 0 != it->second.Encoded() ) {
                    ss << "FFFF";
                } else {
                    auto it = record.find(vehicle::SignalID::GNSSHeading);
                    if (!it->second.Valid()) {
                        // hirain range: 0 < data < 360
                        // tsp range: 0-359
                        if (359 <= it->second.Val() ) {
                            status |= 359 & 0x1FF;
                        } else {
                            status |= 0 & 0x1FF;
                        }
                    } else {
                        status |= it->second.Encoded() & 0x1FF;
                    }

                    it = record.find(vehicle::SignalID::GNSSLatitude);
                    if ( it->second.Val() < 0  ) {
                        status |= ((SMLK_UINT32)1) << 14;
                    }

                    it = record.find(vehicle::SignalID::GNSSLongitude);
                    if ( it->second.Val() < 0 ) {
                        status |= ((SMLK_UINT32)1) << 15;
                    }

                    ss << std::dec << (int)status;
                }
            } else if ( vehicle::SignalID::GNSSLatitude == id ) {
                auto it = record.find(id);
                ss << std::fixed << std::setprecision(6) << ((it->second.Val() < 0) ? - it->second.Val() : it->second.Val());
            } else if ( vehicle::SignalID::GNSSLongitude == id ) {
                auto it = record.find(id);
                ss << std::fixed << std::setprecision(6) << ((it->second.Val() < 0) ? - it->second.Val() : it->second.Val());
            } else if ( vehicle::SignalID::TBoxAcceleration == id ) {
                auto it = record.find(id);
                if ( it->second.Valid() ) {
                    ss << std::fixed << std::setprecision(2) << it->second.Val();
                } else {
                    ss << "FF";
                }
            } else if ( vehicle::SignalID::GNSSAltitude == id ) {
                auto it = record.find(vehicle::SignalID::GNSSLocatedStatus);
                if ( 0 != it->second.Encoded() ) {
                    ss << "FFFF";
                } else {
                    auto it = record.find(id);
                    ss << std::fixed << std::setprecision(2) << it->second.Val();
                }
            } else if ( vehicle::SignalID::GNSSSpeed == id ) {
                auto it = record.find(vehicle::SignalID::GNSSLocatedStatus);
                if ( 0 != it->second.Encoded() ) {
                    ss << "FF";
                } else {
                    it = record.find(vehicle::SignalID::GNSSSpeed);
                    if ( it->second.Val() > it->second.Max() || it->second.Val() < it->second.Min() ) {
                        ss << "FF";
                    } else {
                        ss << std::fixed << std::setprecision(2) << it->second.Val();
                    }
                }
            } else if (vehicle::SignalID::AirSuspensionControl1 == id ) {
                static const std::array<vehicle::SignalID, 13>  air_suspension_control_1_sub_signals = {
                    vehicle::SignalID::FrontAxleInBumperRange,
                    vehicle::SignalID::LiftAxle1Position,
                    vehicle::SignalID::LevelControlMode,
                    vehicle::SignalID::LiftingControlModeRearAxle,
                    vehicle::SignalID::LiftingControlModeFrontAxle,
                    vehicle::SignalID::LoweringControlModeRearAxle,
                    vehicle::SignalID::LoweringControlModeFrontAxle,
                    vehicle::SignalID::AboveNominalLevelRearAxle,
                    vehicle::SignalID::AboveNominalLevelFrontAxle,
                    vehicle::SignalID::BelowNominalLevelRearAxle,
                    vehicle::SignalID::BelowNominalLevelFrontAxle,
                    vehicle::SignalID::NominalLevelRearAxle,
                    vehicle::SignalID::NominalLevelFrontAxle,
                };

                SMLK_UINT32 val = 0;
                SMLK_BOOL invalid_flag = false;
                for ( auto const id : air_suspension_control_1_sub_signals ) {
                    auto const &signal = record.find(id)->second;
                    if (!signal.Valid()) {
                        invalid_flag = true;
                        break;
                    } else {
                        val <<= signal.Bits();
                        val |= signal.Encoded() & bitsmasks[signal.Bits()];
                    }
                }
                if (invalid_flag) {
                    ss << "FF";
                } else {
                    ss << std::dec << val;
                }
            } else if (vehicle::SignalID::AirSuspensionControl2 == id ) {
                static const std::array<vehicle::SignalID, 4>   air_suspension_control_2_sub_signals = {
                    vehicle::SignalID::SuspensionControlRefusalInformation,
                    vehicle::SignalID::SuspensionRemoteControl1,
                    vehicle::SignalID::LiftAxle2Position,
                    vehicle::SignalID::RearAxleInBumperRange,
                };

                SMLK_UINT32 val = 0;
                SMLK_BOOL invalid_flag = false;

                for ( auto const id : air_suspension_control_2_sub_signals ) {
                    auto const &signal = record.find(id)->second;
                    if (!signal.Valid()) {
                        invalid_flag = true;
                        break;
                    } else {
                        val <<= signal.Bits();
                        val |= signal.Encoded() & bitsmasks[signal.Bits()];
                    }
                }
                if (invalid_flag) {
                    ss << "FF";
                } else {
                    ss << std::dec << val;
                }
            } else if ( vehicle::SignalID::AirSuspensionControl3 == id ) {
                static const std::array<vehicle::SignalID, 4>   air_suspension_control_3_sub_signals = {
                    vehicle::SignalID::RelativeLevelFrontAxleLeft,
                    vehicle::SignalID::RelativeLevelFrontAxleRight,
                    vehicle::SignalID::RelativeLevelRearAxleLeft,
                    vehicle::SignalID::RelativeLevelRearAxleRight
                };

                for ( auto const id : air_suspension_control_3_sub_signals ) {
                    auto const &signal = record.find(id)->second;
                    if ( signal.Valid() ) {
                        SMLK_UINT16 encoded = (SMLK_UINT16)signal.Encoded();
                        ss << std::setw(2) << std::setfill('0') << std::hex << (int)((encoded >> 8) & 0xFF);
                        ss << std::setw(2) << std::setfill('0') << std::hex << (int)(encoded & 0xFF);
                    } else {
                        ss << "FFFF";
                    }
                }
            } else if ( vehicle::SignalID::AirSuspensionControl4 == id ) {
                static const std::array<vehicle::SignalID, 4>   air_suspension_control_4_sub_signals = {
                    vehicle::SignalID::BellowPressureFrontAxleLeft,
                    vehicle::SignalID::BellowPressureFrontAxleRight,
                    vehicle::SignalID::BellowPressureRearAxleLeft,
                    vehicle::SignalID::BellowPressureRearAxleRight
                };

                for ( auto const id : air_suspension_control_4_sub_signals ) {
                    auto const &signal = record.find(id)->second;
                    if ( signal.Valid() ) {
                        SMLK_UINT16 encoded = (SMLK_UINT16)signal.Encoded();
                        ss << std::setw(2) << std::setfill('0') << std::hex << (int)((encoded >> 8) & 0xFF);
                        ss << std::setw(2) << std::setfill('0') << std::hex << (int)(encoded & 0xFF);
                    } else {
                        ss << "FFFF";
                    }
                }
            } else if (vehicle::SignalID::OverrideControlMode == id ||
                       vehicle::SignalID::RequestSpeedLimit == id ||
                       vehicle::SignalID::RequestTorqueLimit == id ||
                       vehicle::SignalID::RequestTorqueHighResolution == id) {
                auto const &signal = record.find(id)->second;
                if (!signal.Valid()) {
                    ss << "FF";
                } else {
                    auto const &signal = record.find(id)->second;
                    auto const it = SignalsPrecision().find(id);
                    if ( SignalsPrecision().end() == it ) {
                        if ( signal.IsInteger() ) {
                            // make sure the formated string is a integer like string
                            ss << std::dec << (std::int64_t)signal.Val();
                        } else {
                            // ss << std::fixed << std::setprecision(2) << signal.Val();
                            auto val = ((double)((std::int64_t)(signal.Val() * 100.0))) * 0.01 + 0.001;
                            ss << std::fixed << std::setprecision(2) << val;
                        }
                    } else {
                        ss << std::fixed << std::setprecision(it->second) << signal.Val();
                    }
                }
            } else if (vehicle::SignalID::PitchAngle == id) {
                auto const &signal = record.find(id)->second;
                if (!signal.Valid()) {
                    ss << "FF";
                } else {
                    auto const &signal = record.find(id)->second;
                    auto const it = SignalsPrecision().find(id);
                    if ( SignalsPrecision().end() == it ) {
                        if ( signal.IsInteger() ) {
                            // make sure the formated string is a integer like string
                            ss << std::dec << (std::int64_t)signal.Val();
                        } else {
                            const double EPS = 1e-6;
                            if (fabs(signal.Val()- signal.Max()) < EPS) {
                                SMLK_LOGD("PitchAngle is max value");
                                ss << std::fixed << std::setprecision(2) << signal.Max();
                            } else {
                                auto val = ((double)((std::int64_t)(signal.Val() * 100.0))) * 0.01 + 0.001;
                                ss << std::fixed << std::setprecision(2) << val;
                            }
                        }
                    } else {
                        ss << std::fixed << std::setprecision(it->second) << signal.Val();
                    }
                }
            } else {
                auto const &signal = record.find(id)->second;
                if ( signal.Valid() ) {
                    auto const it = SignalsPrecision().find(id);
                    if ( SignalsPrecision().end() == it ) {
                        if ( signal.IsInteger() ) {
                            // make sure the formated string is a integer like string
                            ss << std::dec << (std::int64_t)signal.Val();
                        } else {
                            //ss << std::fixed << std::setprecision(2) << signal.Val();
                            auto val = ((double)((std::int64_t)(signal.Val() * 100.0))) * 0.01 + 0.001;
                            ss << std::fixed << std::setprecision(2) << val;
                        }
                    } else {
                        ss << std::fixed << std::setprecision(it->second) << signal.Val();
                    }
                } else {
                    //ss << (signal.Bits() > 16 ? "FFFFFFFF" : signal.Bits() > 8 ? "FFFF" : "FF");
                    ss << "FF";
                }
            }

            ss << ",";
        }

        std::string row(ss.str());
        // remove the last ','
        row.pop_back();

        PUSH_U1('\r');
        PUSH_U1('\n');

        data.insert(data.end(), row.begin(), row.end());
    }

    // try to fix the length parameter
    SMLK_UINT16 length = data.size() - length_index - sizeof(SMLK_UINT16);

    data[length_index]      = (SMLK_UINT8)((length >> 8) & 0xFF);
    data[length_index+1]    = (SMLK_UINT8)(length & 0xFF);

    return SL_SUCCESS;
}

std::uint32_t MessageBucket808::Decode(IN std::vector<SMLK_UINT8> &)
{
    return SL_SUCCESS;
}

std::size_t MessageBucket808::CompressOffset() const {
    //     frequency             compress flag        reserved              length
    return sizeof(SMLK_UINT16) + sizeof(SMLK_UINT8) + sizeof(SMLK_UINT32) + sizeof(SMLK_UINT16);
}

MessageBucket30S808::MessageBucket30S808()
{}

MessageBucket30S808::~MessageBucket30S808()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION: encode the message body to bytes
 *
 * PARAMETERS:
 *     data: buffer to handle the result
 * RETURN:
 *      0: success
 *  other:
 * NOTE:
 *
 *****************************************************************************/
std::uint32_t MessageBucket30S808::Encode(std::vector<SMLK_UINT8> &data)
{
    data.clear();

    PUSH_U2(m_sampling_frequency);
    PUSH_U1(m_crompression);
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U1(0x00);

    std::size_t length_index = data.size();

    // content length
    PUSH_U1(0x00);
    PUSH_U1(0x00);

    // TAPD:1018384
    m_signal_total_vehicle_distance.SetValid();
    m_signal_ecu_total_fuel_used.SetValid();

    PUSH_U2(m_signal_day_vehicle_distance.Encoded());
    PUSH_U4(m_signal_total_vehicle_distance.Encoded());
    PUSH_U4(m_signal_ecu_total_distacne.Encoded());
    PUSH_U2(m_signal_day_fuel_used.Encoded());
    PUSH_U4(m_signal_total_fuel_used.Encoded());
    PUSH_U4(m_signal_ecu_total_fuel_used.Encoded());

    std::stringstream   ss;

    if ( 0 == m_signals.size() ) {
        return SL_EFAILED;
    }

    PUSH_U2(m_signals.size() + 1);

    for ( auto const id : SignalIDs() ) {
        ss << m_signals[0].find(id)->second.ID() << ",";
    }

    std::string     head(ss.str());
    // remove the last ','
    head.pop_back();

    data.insert(data.end(), head.begin(), head.end());

    for ( auto const &record : m_signals ) {
        // clear stream buffer for reuse
        ss.clear();
        ss.str(std::string());

        for ( auto const id : SignalIDs() ) {
            if ( vehicle::SignalID::GNSSTimeStamp == id ) {
                std::time_t timestamp = record.find(id)->second.Val();
                //struct tm tt = *std::localtime(&timestamp);
                struct tm tt;
                localtime_r(&timestamp, &tt);
                ss << std::setw(2) << std::setfill('0') << std::dec << (tt.tm_year - 100)%100;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_mon + 1;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_mday;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_hour;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_min;
                ss << std::setw(2) << std::setfill('0') << std::dec << tt.tm_sec;
            } else if ( vehicle::SignalID::GNSSLocatedStatus == id ) {
                SMLK_UINT16                     status = 0;
                auto it = record.find(vehicle::SignalID::GNSSLocatedStatus);
                if ( 0 != it->second.Encoded() ) {
                    ss << "FFFF";
                } else {
                    auto it = record.find(vehicle::SignalID::GNSSHeading);
                    if (!it->second.Valid()) {
                        // hirain range: 0 < data < 360
                        // tsp range: 0-359
                        if (359 <= it->second.Val() ) {
                            status |= 359 & 0x1FF;
                        } else {
                            status |= 0 & 0x1FF;
                        }
                    } else {
                        status |= it->second.Encoded() & 0x1FF;
                    }

                    it = record.find(vehicle::SignalID::GNSSLatitude);
                    if ( it->second.Val() < 0  ) {
                        status |= ((SMLK_UINT32)1) << 14;
                    }

                    it = record.find(vehicle::SignalID::GNSSLongitude);
                    if ( it->second.Val() < 0 ) {
                        status |= ((SMLK_UINT32)1) << 15;
                    }
                    ss << std::dec << (int)status;
                }

            } else if ( vehicle::SignalID::GNSSLatitude == id ) {
                auto it = record.find(id);
                ss << std::fixed << std::setprecision(6) << ((it->second.Val() < 0) ? - it->second.Val() : it->second.Val());
            } else if ( vehicle::SignalID::GNSSLongitude == id ) {
                auto it = record.find(id);
                ss << std::fixed << std::setprecision(6) << ((it->second.Val() < 0) ? - it->second.Val() : it->second.Val());
            } else if ( vehicle::SignalID::GNSSAltitude == id ) {
                auto it = record.find(vehicle::SignalID::GNSSLocatedStatus);
                if ( 0 != it->second.Encoded() ) {
                    ss << "FFFF";
                } else {
                    auto it = record.find(id);
                    ss << std::fixed << std::setprecision(2) << it->second.Val();
                }
            } else if ( vehicle::SignalID::GNSSSpeed == id ) {
                auto it = record.find(vehicle::SignalID::GNSSLocatedStatus);
                if ( 0 != it->second.Encoded() ) {
                    ss << "FF";
                } else {
                    it = record.find(vehicle::SignalID::GNSSSpeed);
                    if ( it->second.Val() > it->second.Max() || it->second.Val() < it->second.Min() ) {
                        ss << "FF";
                    } else {
                        ss << std::fixed << std::setprecision(2) << it->second.Val();
                    }
                }
            } else if ( vehicle::SignalID::ATMTransmissionMode == id ) {
                static const std::array<vehicle::SignalID, 4>   transmission_mode = {
                    vehicle::SignalID::TransmissionMode1,
                    vehicle::SignalID::TransmissionMode2,
                    vehicle::SignalID::TransmissionMode3,
                    vehicle::SignalID::TransmissionMode4
                };

                SMLK_BOOL isValid =false;
                for ( auto const id : transmission_mode ) {
                    auto const &signal = record.find(id)->second;
                    if ( signal.Valid() )
                    {
                        isValid = true;
                        break;
                    }
                }
                if ( isValid )
                {
                    int             bits = 0;
                    SMLK_UINT8      val = 0;

                    for ( auto const id : transmission_mode ) {
                          auto const &signal = record.find(id)->second;
                          val |= signal.Encoded() << bits;
                          bits += signal.Bits();
                    }

                    auto var(record.find(id)->second);
                    var.AssignEncoded(val);
                    ss << std::dec << (std::int64_t)var.Val();
                }
                else
                {
                    ss << "FF";
                }
            } else {
                auto const &signal = record.find(id)->second;
                if ( signal.Valid() ) {
                    auto const it = SignalsPrecision().find(id);
                    if ( SignalsPrecision().end() == it ) {
                        if ( signal.IsInteger() ) {
                            // make sure the formated string is a integer like string
                            ss << std::dec << (std::int64_t)signal.Val();
                        } else {
                            //ss << std::fixed << std::setprecision(2) << signal.Val();
                            auto val = ((double)((std::int64_t)(signal.Val() * 100.0))) * 0.01 + 0.001;
                            ss << std::fixed << std::setprecision(2) << val;
                        }
                    } else {
                        ss << std::fixed << std::setprecision(it->second) << signal.Val();
                    }
                } else {
                    ss << "FF";         // out of range
                }
            }

            ss << ",";
        }

        std::string row(ss.str());
        // remove the last ','
        row.pop_back();

        PUSH_U1('\r');
        PUSH_U1('\n');

        data.insert(data.end(), row.begin(), row.end());
    }

    // try to fix the length parameter
    SMLK_UINT16 length = data.size() - length_index - sizeof(SMLK_UINT16);

    data[length_index]      = (SMLK_UINT8)((length >> 8) & 0xFF);
    data[length_index+1]    = (SMLK_UINT8)(length & 0xFF);

    return SL_SUCCESS;
}

std::uint32_t MessageBucket30S808::Decode(IN std::vector<SMLK_UINT8> &)
{
    return SL_SUCCESS;
}

std::size_t MessageBucket30S808::CompressOffset() const {
    //     frequency             compress flag        reserved              length
    return sizeof(SMLK_UINT16) + sizeof(SMLK_UINT8) + sizeof(SMLK_UINT32) + sizeof(SMLK_UINT16);
}

MessageCompressedBucket808::MessageCompressedBucket808()
{}

MessageCompressedBucket808::~MessageCompressedBucket808()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION: encode the message body to bytes
 *
 * PARAMETERS:
 *     data: buffer to handle the result
 * RETURN:
 *      0: success
 *  other:
 * NOTE:
 *
 *****************************************************************************/
std::uint32_t MessageCompressedBucket808::Encode(std::vector<SMLK_UINT8> &data)
{
    data.clear();

    SMLK_UINT16    length = (SMLK_UINT16)m_compressed.size();

    PUSH_U2(m_sampling_frequency);
    PUSH_U1(m_crompression);
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U1(0x00);
    PUSH_U2(length);

    data.insert(data.end(), m_compressed.begin(), m_compressed.end());

    return SL_SUCCESS;
}

std::uint32_t MessageCompressedBucket808::Decode(IN std::vector<SMLK_UINT8> &)
{
    return SL_SUCCESS;
}

MessageStatistics808::MessageStatistics808()
{}

MessageStatistics808::~MessageStatistics808()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION: encode the message body to bytes
 *
 * PARAMETERS:
 *     data: buffer to handle the result
 * RETURN:
 *      0: success
 *  other:
 * NOTE:
 *
 *****************************************************************************/
std::uint32_t MessageStatistics808::Encode(std::vector<SMLK_UINT8> &data)
{
    data.clear();

    SMLK_UINT16 heading_ex = 0x0000;
    SMLK_BCD    timestamp[M_SL_TIMESTAMP_SZ];
    SMLK_BCD    trip_start[M_SL_TIMESTAMP_SZ];
    SMLK_BCD    trip_stop[M_SL_TIMESTAMP_SZ];
    decltype(m_location.m_latitude)     latitude(m_location.m_latitude);
    decltype(m_location.m_longitude)    longitude(m_location.m_longitude);
    decltype(m_location.m_altitude)     altitude(m_location.m_altitude);
    decltype(m_location.m_heading)      heading(m_location.m_heading);
    decltype(m_location.m_speed)        speed(m_location.m_speed);

    if ( m_location.m_status.Encoded() & 0x01 ) {
        altitude = 0;
        heading = 0;
        speed = 0;
    }

    if ( latitude.Val() < 0 ) {
        latitude = -latitude.Val();
        heading_ex |= 0x4000;
    } else if ( !latitude.Valid() ) {
        latitude = latitude.Val();
    }

    if ( longitude.Val() < 0 ) {
        longitude   = -longitude.Val();
        heading_ex |= 0x8000;
    } else if ( !longitude.Valid() ) {
        longitude = longitude.Val();
    }

    heading_ex |= heading.Encoded() & 0x1FF;

    TimeStamp2BCD((std::time_t)m_location.m_timestamp.Val(), timestamp, sizeof(timestamp));
    TimeStamp2BCD((std::time_t)m_trip_start.Val(), trip_start, sizeof(trip_start));
    TimeStamp2BCD((std::time_t)m_trip_stop.Val(), trip_stop, sizeof(trip_stop));

    PUSH_U1(m_flags);
    PUSH_U4(latitude.Encoded());
    PUSH_U4(longitude.Encoded());
    PUSH_U2((SMLK_UINT16)altitude.Val());
    PUSH_U2(heading_ex);
    PUSH_AR(timestamp);
    PUSH_AR(trip_start);                                                // 1
    PUSH_AR(trip_stop);                                                 // 2
    PUSH_U4(m_gnss_statistics_distance.Encoded());                      // 3
    PUSH_U4(m_tachograph_vehicle_speed_statistics_distance.Encoded());  // 4
    PUSH_U4(m_vehicle_speed_statistics_life_total_distance.Encoded());  // 5
    PUSH_U4(m_vehicle_total_distance.Encoded());                        // 6
    PUSH_U2(m_vehicle_estimated_load.Encoded());                        // 7
    PUSH_U2(m_trip_statistics_fuel_used.Encoded());                     // 8
    PUSH_U4(m_total_statistics_fuel_used.Encoded());                    // 9
    if ( m_engine_total_fuel_used.Valid() ) {                           // 10
        PUSH_U4(m_engine_total_fuel_used.Encoded());
    } else {
        decltype(m_engine_total_fuel_used)  engine_total_fuel_used(m_engine_total_fuel_used);
        engine_total_fuel_used = m_engine_total_fuel_used.Val();
        PUSH_U4(engine_total_fuel_used.Encoded());
    }
    PUSH_U2(m_engine_average_fuel_economy.Encoded());                   // 11
    PUSH_U2(m_speeding_times.Encoded());                                // 12
    PUSH_U4(m_speeding_distance.Encoded());                             // 13
    PUSH_U2(m_speeding_fuel_used.Encoded());                            // 14
    PUSH_U2(m_brake_pedal_times.Encoded());                             // 15
    PUSH_U4(m_brake_pedal_brake_distance.Encoded());                    // 16
    PUSH_U4(m_brake_pedal_brake_duration.Encoded());                    // 17
    PUSH_U4(0xFFFFFFFF);                                                // 18
    PUSH_U4(0xFFFFFFFF);                                                // 19
    PUSH_U4(0xFFFFFFFF);                                                // 20
    PUSH_U1(0xFF);                                                      // 21
    PUSH_U1(0xFF);                                                      // 22
    PUSH_U1(0xFF);                                                      // 23
    PUSH_U2(0xFFFF);                                                    // 24
    PUSH_U4(0xFFFFFFFF);                                                // 25
    PUSH_U2(0xFFFF);                                                    // 26
    PUSH_U2(0xFFFF);                                                    // 27
    PUSH_U4(0xFFFFFFFF);                                                // 28
    PUSH_U2(0xFFFF);                                                    // 29
    PUSH_U2(0xFFFF);                                                    // 30
    PUSH_U4(0xFFFFFFFF);                                                // 31
    PUSH_U2(0xFFFF);                                                    // 32
    PUSH_U2(0xFFFF);                                                    // 33
    PUSH_U4(0xFFFFFFFF);                                                // 34
    PUSH_U2(0xFFFF);                                                    // 35
    PUSH_U2(0xFFFF);                                                    // 36
    PUSH_U4(0xFFFFFFFF);                                                // 37
    PUSH_U2(0xFFFF);                                                    // 38
    PUSH_U2(0xFFFF);                                                    // 39
    PUSH_U4(0xFFFFFFFF);                                                // 40
    PUSH_U2(0xFFFF);                                                    // 41
    PUSH_U2(0xFFFF);                                                    // 42
    PUSH_U4(0xFFFFFFFF);                                                // 43
    PUSH_U2(0xFFFF);                                                    // 44
    PUSH_U2(0xFFFF);                                                    // 45
    PUSH_U4(0xFFFFFFFF);                                                // 46
    PUSH_U2(0xFFFF);                                                    // 47
    PUSH_U2(0xFFFF);                                                    // 48
    PUSH_U4(0xFFFFFFFF);                                                // 49
    PUSH_U2(0xFFFF);                                                    // 50
    PUSH_U2(0xFFFF);                                                    // 51
    PUSH_U2(0xFFFF);                                                    // 52
    PUSH_U2(m_sharp_slowdown_duration.Encoded());                       // 53
    PUSH_U4(m_sharp_slowdown_distance.Encoded());                       // 54
    PUSH_U2(m_sharp_slowdown_times.Encoded());                          // 55
    PUSH_U2(m_sharp_speedup_duration.Encoded());                        // 56
    PUSH_U4(m_sharp_speedup_distance.Encoded());                        // 57
    PUSH_U2(m_sharp_speedup_times.Encoded());                           // 58
    PUSH_U2(m_sharp_speedup_fuel_used.Encoded());                       // 59
    PUSH_U4(0xFFFFFFFF);                                                // 60
    PUSH_U2(0xFFFF);                                                    // 61
    PUSH_U2(0xFFFF);                                                    // 62
    PUSH_U4(m_sharp_turnning_distance.Encoded());                       // 63
    PUSH_U2(m_sharp_turnning_times.Encoded());                          // 64
    PUSH_U1(0xFF);                                                      // 65
    PUSH_U2(0xFFFF);                                                    // 66
    PUSH_U1(0xFF);                                                      // 67
    PUSH_U2(0xFFFF);                                                    // 68
    PUSH_U2(0xFFFF);                                                    // 69
    PUSH_U2(0xFFFF);                                                    // 70
    PUSH_U1(0xFF);                                                      // 71
    PUSH_U2(0xFFFF);                                                    // 72
    PUSH_U2(0xFFFF);                                                    // 73
    PUSH_U2(0xFFFF);                                                    // 74
    PUSH_U2(0xFFFF);                                                    // 75
    PUSH_U2(0xFFFF);                                                    // 76
    PUSH_U2(0xFFFF);                                                    // 77
    PUSH_U2(0xFFFF);                                                    // 78
    PUSH_U1(0xFF);                                                      // 79
    PUSH_U1(0x00);                                                      // 80
                                                                        // 81
    PUSH_U2(0xFFFF);                                                    // 82
    PUSH_U2(0xFFFF);                                                    // 83
    PUSH_U1(m_sleepy_drive_times.Encoded());                            // 84
    PUSH_U1(0x00);                                                      // 85
                                                                        // 86
    PUSH_U1(0x00);                                                      // 87
                                                                        // 88
    PUSH_U2(0xFFFF);                                                    // 89
    PUSH_U2(0xFFFF);                                                    // 90
    PUSH_U2(0xFFFF);                                                    // 91
    PUSH_U2(0xFFFF);                                                    // 92
    PUSH_U2(0xFFFF);                                                    // 93
    PUSH_U2(0xFFFF);                                                    // 94
    PUSH_U2(0xFFFF);                                                    // 95
    PUSH_U2(0xFFFF);                                                    // 96
    PUSH_U1(0xFF);                                                      // 97
    PUSH_U2(0xFFFF);                                                    // 98
    PUSH_U2(0xFFFF);                                                    // 99
    PUSH_U1(0x00);                                                      // 100
                                                                        // 101
    PUSH_U1(0x00);                                                      // 102
                                                                        // 103
    PUSH_U1(0x00);                                                      // 104
                                                                        // 105

    return SL_SUCCESS;
}

std::uint32_t MessageStatistics808::Decode(IN std::vector<SMLK_UINT8> &)
{
    return SL_SUCCESS;
}

MessageEvent808::MessageEvent808()
{}

MessageEvent808::~MessageEvent808()
{}

/******************************************************************************
 * NAME: Encode
 *
 * DESCRIPTION: encode the message body to bytes
 *
 * PARAMETERS:
 *     data: buffer to handle the result
 * RETURN:
 *      0: success
 *  other:
 * NOTE:
 *
 *****************************************************************************/
std::uint32_t MessageEvent808::Encode(std::vector<SMLK_UINT8> &data)
{
    data.clear();

    if ( 0 == m_locations.size() ) {
        return SL_EFAILED;
    }

    SMLK_BCD    timestamp[M_SL_TIMESTAMP_SZ];

    PUSH_U1(m_reversion);

    auto const &lo_start = m_locations[0];

    decltype(lo_start.m_latitude)           latitude(lo_start.m_latitude);
    decltype(lo_start.m_longitude)          longitude(lo_start.m_longitude);
    decltype(lo_start.m_altitude)           altitude(lo_start.m_altitude);
    decltype(lo_start.m_heading)            heading(lo_start.m_heading);
    decltype(lo_start.m_speed)              speed(lo_start.m_speed);
    // 起始点或结束点的位置信息中，当 GPS 未定位成功或 GPS 异常时，
    // 高程、速度、方向填充为 0
    if ( lo_start.m_status.Encoded() & 0x01 ) {
        SMLK_LOGD("lo_start.m_status.Encoded is invalid");
        altitude    = 0;
        heading     = 0;
        speed       = 0;
    }
    // 拆除事件详细信息
    // Bit0: 0 有效定位，1 无效定位
    // Bit1:0 北纬,1 南纬
    // Bit2:0 东经,1 西经
    // Bit3~Bit7:保留
    // 纬度 正:北纬 负:南纬
    // 经度 正:东经 负:西经
    SMLK_UINT8 dismantle_tbox_gnss_status_ext = 0;
    if (M_SL_EVID_DISMANTLE_TBOX == m_id) {
        if ( lo_start.m_status.Encoded() & 0x01 ) {
            SETBIT(dismantle_tbox_gnss_status_ext, 0);
        } else {
            CLRBIT(dismantle_tbox_gnss_status_ext, 0);
        }
        if ( latitude.Val() < 0 ) {
            SETBIT(dismantle_tbox_gnss_status_ext, 1);
        }
        if ( longitude.Val() < 0 ) {
            SETBIT(dismantle_tbox_gnss_status_ext, 2);
        }
#ifdef SL_DEBUG
        SMLK_LOGE("[tboxtest] encode dismantle_tbox_gnss_status_ext =  %d", dismantle_tbox_gnss_status_ext);
#endif
    }

    SMLK_UINT16 heading_ex = heading.Encoded() & 0x1FF;

    if ( latitude.Val() < 0 ) {
        latitude = -latitude.Val();
        heading_ex |= 0x4000;
    } else {
        if ( !latitude.Valid() ) {
            latitude = latitude.Val();
        }
    }

    if ( longitude.Val() < 0 ) {
        longitude   = -longitude.Val();
        heading_ex |= 0x8000;
    } else if ( !latitude.Valid() ) {
        latitude = longitude.Val();
    }

    TimeStamp2BCD((std::time_t)lo_start.m_timestamp.Val(), timestamp, sizeof(timestamp));

    PUSH_U4(latitude.Encoded());
    PUSH_U4(longitude.Encoded());
    PUSH_U2((SMLK_UINT16)altitude.Val());
    PUSH_U2(heading_ex);
    PUSH_AR(timestamp);
    PUSH_U1((SMLK_UINT8)speed.Val());

    if ( m_locations.size() > 1 ) {

        auto const &lo_stop = m_locations[m_locations.size()-1];
        latitude    = lo_stop.m_latitude;
        longitude   = lo_stop.m_longitude;
        altitude    = lo_stop.m_altitude;
        heading     = lo_stop.m_heading;
        speed       = lo_stop.m_speed;
        // 如果定位无效，返回最后一次有效定位，同时定位标志置为无效
        if ( lo_stop.m_status.Encoded() & 0x01 ) {
            altitude    = 0;
            speed       = 0;
            heading     = 0;
        }

        heading_ex  = heading.Encoded() & 0x1FF;

        if ( latitude.Val() < 0 ) {
            latitude = -latitude.Val();
            heading_ex |= 0x4000;
        } else if ( !latitude.Valid() ) {
            latitude = latitude.Val();
        }

        if ( longitude.Val() < 0 ) {
            longitude   = -longitude.Val();
            heading_ex |= 0x8000;
        } else if ( !longitude.Valid() ) {
            longitude = longitude.Val();
        }

        // modify by lsc
        // if end time is 0, that means this event is start event, just fill 0;
        if ((std::time_t)lo_stop.m_timestamp.Val()){
            TimeStamp2BCD((std::time_t)lo_stop.m_timestamp.Val(), timestamp, sizeof(timestamp));
        } else {
            std::memset(timestamp, 0x0, sizeof(timestamp));
        }

        PUSH_U4(latitude.Encoded());
        PUSH_U4(longitude.Encoded());
        PUSH_U2((SMLK_UINT16)altitude.Val());
        PUSH_U2(heading_ex);
        PUSH_AR(timestamp);
        PUSH_U1((SMLK_UINT8)speed.Val());
    } else {
        SMLK_LOGD("m_locations is <= 1");
        std::memset(timestamp, 0x00, sizeof(timestamp));

        PUSH_U4(0);
        PUSH_U4(0);
        PUSH_U2(0);
        PUSH_U2(0);
        PUSH_AR(timestamp);
        PUSH_U1(0);
    }

    auto index_length = data.size();

    // place holder for the content length
    PUSH_U1(0x00);
    PUSH_U1(0x00);

    PUSH_U1(m_id);

    switch ( m_id ) {
        case M_SL_EVID_SHARP_SLOW_DOWN:
            {
                PUSH_U2(m_distance.Encoded());
                PUSH_U2(m_duration.Encoded());

                // to be safe enough, check the cached location size
                if ( m_locations.size() > 0xFF + 2 ) {
                    m_locations.resize(0xFF + 2);
                }

                PUSH_U1(m_locations.size() - 1);
                for ( auto it = m_locations.cbegin() + 1 ; it < m_locations.cend(); ++it ) {
                    latitude    = it->m_latitude.Val() < 0 ? -it->m_latitude.Val() : it->m_latitude.Val();
                    longitude   = it->m_longitude.Val() < 0 ? -it->m_longitude.Val() : it->m_longitude.Val();
                    PUSH_U4(latitude.Encoded());
                    PUSH_U4(longitude.Encoded());
                }
            }
            break;
        case M_SL_EVID_SHARP_SPEED_UP:
            {
                PUSH_U2(m_distance.Encoded());
                PUSH_U2(m_fuel_used.Encoded());
                PUSH_U2(m_duration.Encoded());

                if ( m_locations.size() > 0xFF + 2) {
                    m_locations.resize(0xFF + 2);
                }

                PUSH_U1(m_locations.size() - 1);
                for ( auto it = m_locations.cbegin() + 1; it < m_locations.cend(); ++it ) {
                    latitude    = it->m_latitude.Val() < 0 ? -it->m_latitude.Val() : it->m_latitude.Val();
                    longitude   = it->m_longitude.Val() < 0 ? -it->m_longitude.Val() : it->m_longitude.Val();

                    PUSH_U4(latitude.Encoded());
                    PUSH_U4(longitude.Encoded());
                }
            }
            break;
        case M_SL_EVID_SHARP_TURN:
            {
                PUSH_U2(m_distance.Encoded());
                PUSH_U2(m_fuel_used.Encoded());

                if ( m_locations.size() > 0xFF + 2) {
                    m_locations.resize(0xFF + 2);
                }

                PUSH_U1(m_locations.size() - 2);
                for ( auto it = m_locations.cbegin() + 1; it < m_locations.cend() - 1; ++it ) {
                    latitude    = it->m_latitude.Val() < 0 ? -it->m_latitude.Val() : it->m_latitude.Val();
                    longitude   = it->m_longitude.Val() < 0 ? -it->m_longitude.Val() : it->m_longitude.Val();

                    PUSH_U4(latitude.Encoded());
                    PUSH_U4(longitude.Encoded());
                }
            }
            break;
        case M_SL_EVID_SPEEDING:
            {
                PUSH_U2(m_distance.Encoded());
                PUSH_U2(m_fuel_used.Encoded());
            }
            break;
        case M_SL_EVID_LONG_LONG_PARKING:
            {
                // reuse the event duration, but its unit is h instead of s
                PUSH_U1(m_duration.Encoded());
            }
            break;
        case M_SL_EVID_ACCIDENT:
            {
                auto frames = m_frames;
                // reuse the status
                PUSH_U1(m_type);
                PUSH_U1((SMLK_UINT8)frames.size());

                while ( frames.size() ) {
                    PUSH_U2(frames.front().m_emergency_braking_state.Encoded());
                    PUSH_U2(frames.front().m_emergency_braking_target_id.Encoded());
                    PUSH_U2(frames.front().m_target_relative_speed.Encoded());
                    PUSH_U2(frames.front().m_target_speed.Encoded());
                    PUSH_U2(frames.front().m_target_distance.Encoded());
                    frames.pop();
                }
            }
            break;
        case M_SL_EVID_TPMS_TIRE_WARN:
            {
                PUSH_U1(m_tire_info.m_id);
                PUSH_U1(m_status & 0xFF);
                PUSH_U1((m_status >> 8) & 0xFF);
                PUSH_U2(m_tire_info.m_temperature.Encoded());
                PUSH_U1(m_tire_info.m_pressure.Encoded());
                PUSH_U2(m_tire_info.m_extend_pressure.Encoded());
            }
            break;
        case M_SL_EVID_TIREDNESS_DRIVING:
            {
                vehicle::SignalVariant  duration(0, 16, 1,  0,  0,  65530);
                duration = m_duration;
                PUSH_U2(duration.Encoded());
            }
            break;
        case M_SL_EVID_SOC_LOW:
            {
                PUSH_U1(m_status & 0xFF);
                // m_actual_minimum_soc from vehicle event report api is can origin value
                if (!m_actual_minimum_soc.Valid()) {
                    PUSH_U2(0xFFFF);
                } else {
                    PUSH_U2(m_actual_minimum_soc.Encoded());
                }
            }
            break;
        case M_SL_EVID_DMS_WARN:
            {
                PUSH_U1(m_status & 0xFF);
            }
            break;
        case M_SL_EVID_FUEL_TANK_WARN:
            break;
        case M_SL_EVID_AGITATOR_FAULT:
            {
                PUSH_U1(m_status & 0xFF);
            }
            break;
        case M_SL_EVID_DISMANTLE_TBOX:
            {
#ifdef SL_DEBUG
                SMLK_LOGD("[tboxtest] encode  m_dismantle_tbox_status =  %d", m_status);
#endif
                PUSH_U1(m_status & 0xFF);
                PUSH_U1(dismantle_tbox_gnss_status_ext);
                PUSH_U4(lo_start.m_longitude.Encoded());
                PUSH_U4(lo_start.m_latitude.Encoded());
            }
        case M_SL_EVID_GB_CHECK_VIN_ERROR:
            break;
        case M_SL_EVID_REFRIGERATOR_FAILURE:
            {
                PUSH_U1(m_status & 0xFF);
            }
            break;
        default:
            data.clear();
            return SL_ESUPPORT;
    }

    auto sz = data.size() - index_length - 2;

    data[index_length]      = (SMLK_UINT8)((sz >> 8) & 0xFF);
    data[index_length+1]    = (SMLK_UINT8)(sz & 0xFF);

    return SL_SUCCESS;
}

std::uint32_t MessageEvent808::Decode(IN std::vector<SMLK_UINT8> &)
{
    return SL_SUCCESS;
}

MessageLocationRsp808::MessageLocationRsp808()
{}

MessageLocationRsp808::~MessageLocationRsp808()
{}

std::uint32_t MessageLocationRsp808::Encode(std::vector<SMLK_UINT8> &data)
{
    data.clear();
    /*
     * This is not a good idea to insert at a the head of a vector.
     * I have ever thought that if it is OK to operator data without
     * clear() operation, but finally I give up that. Any idea about
     * this will be a welcomed.
     * */
    m_message_location.Encode(data);
    data.insert(data.begin(), m_seq);

    return SL_SUCCESS;
}

std::uint32_t MessageLocationRsp808::Decode(IN std::vector<SMLK_UINT8> &)
{
    return SL_SUCCESS;
}


MessageDM1808::MessageDM1808()
{}

MessageDM1808::~MessageDM1808()
{}

SMLK_UINT32 MessageDM1808::Encode(std::vector<SMLK_UINT8> &data)
{
    SMLK_LOGD("[dm1debug] MessageDM1808::Encode");

    data.clear();

    PUSH_U1(m_type);

    decltype(data.size())   index = data.size();

    PUSH_U1(0x00);      // length placeholder

    SMLK_LOGD("[dm1debug] m_add_extra_info_flag is %d", m_add_extra_info_flag);
    if ( 0 == m_add_extra_info_flag) {
        return SL_SUCCESS;
    }

    PUSH_U2(m_tachograph_speed.Encoded());
    PUSH_U1(m_acc_pedal_travel.Encoded());
    PUSH_U1(m_brake_pedal_state.Encoded());
    PUSH_U2(m_engine_speed.Encoded());
    PUSH_U1(m_tubocharing_pressue.Encoded());
    PUSH_U1(m_intake_manifold_pressure.Encoded());
    PUSH_U2(m_exhaust_gas_temperature.Encoded());
    PUSH_U1(m_coolant_temperature.Encoded());
    PUSH_U2((SMLK_UINT16)std::round(std::abs(m_acc_change_rate.Val())));
    PUSH_U1(m_gear.Encoded());
    PUSH_U1(m_torque_percent.Encoded());
    PUSH_U4(m_vehicle_load.Encoded());
    PUSH_U1(m_engine_load.Encoded());
    PUSH_U2(m_acceleration.Encoded());
    // TAPD 1029943 减速度WORD 上传的值最高位为 1
    PUSH_U2(m_deceleration.Encoded() | 0x8000);

    decltype(data.size())   sz = 0xFF + 1 - data.size();

    if ( m_faults.size() * (sizeof(SMLK_UINT32) + sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8)) > sz ) {
        decltype(m_faults.size())   max = sz / (sizeof(SMLK_UINT32) + sizeof(SMLK_UINT8) + sizeof(SMLK_UINT8));
        SMLK_LOGD("[dm1debug] MessageDM1808::Encode m_faults max size is %d", max);
        m_faults.resize(max);
    }

    PUSH_U1(m_faults.size());

    for ( auto const &fault : m_faults ) {
        SMLK_LOGD("[dm1debug] add one faults ");
        PUSH_U1(fault.m_source);
        PUSH_U4(fault.m_SPN);
        PUSH_U1(fault.m_FMI);
    }

    data[index] = (SMLK_UINT8)(data.size() - 2);

    return SL_SUCCESS;
}

SMLK_UINT32 MessageDM1808::Decode(IN std::vector<SMLK_UINT8> &)
{
    return SL_SUCCESS;
}

MessageFault808::MessageFault808()
{}

MessageFault808::~MessageFault808()
{}

SMLK_UINT32 MessageFault808::Encode(std::vector<SMLK_UINT8> &data)
{
    data.clear();
    SMLK_LOGD("[dm1debug] MessageFault808::Encode ");

    decltype(m_location.m_latitude)     latitude(m_location.m_latitude);
    decltype(m_location.m_longitude)    longitude(m_location.m_longitude);
    decltype(m_location.m_altitude)     altitude(m_location.m_altitude);
    decltype(m_location.m_heading)      heading(m_location.m_heading);

    if ( m_location.m_status.Encoded() & 0x01 ) {
        altitude    = 0;
        heading     = 0;
    }

    SMLK_UINT16 heading_ex = heading.Encoded() & 0x1FF;

    if ( latitude.Val() < 0 ) {
        latitude = -latitude.Val();
        heading_ex |= 0x4000;
    } else {
        if ( !latitude.Valid() ) {
            latitude = latitude.Val();
        }
    }

    if ( longitude.Val() < 0 ) {
        longitude   = -longitude.Val();
        heading_ex |= 0x8000;
    } else if ( !latitude.Valid() ) {
        latitude = longitude.Val();
    }

    PUSH_U1(m_reversion);
    PUSH_U4(latitude.Encoded());
    PUSH_U4(longitude.Encoded());
    PUSH_U2((SMLK_UINT16)altitude.Val());
    PUSH_U2(heading_ex);

    SMLK_BCD    timestamp[M_SL_TIMESTAMP_SZ];

    TimeStamp2BCD((std::time_t)m_location.m_timestamp.Val(), timestamp, sizeof(timestamp));

    PUSH_AR(timestamp);
    PUSH_U1(m_type);

    if ( !m_fault ) {
        SMLK_LOGD("[dm1debug] return fail ");
        return SL_EFAILED;
    }

    std::vector<SMLK_UINT8> encoded;
    if ( SL_SUCCESS != m_fault->Encode(encoded) ) {
        return SL_EFAILED;
    }

    PUSH_U2(encoded.size());

    data.insert(data.end(), encoded.cbegin(), encoded.cend());

    return SL_SUCCESS;
}

SMLK_UINT32 MessageFault808::Decode(IN std::vector<SMLK_UINT8> &data)
{
    return SL_SUCCESS;
}

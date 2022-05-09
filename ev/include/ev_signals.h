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
#include <tuple>
#include <unordered_map>
#include <smlk_types.h>
#include <smlk_signal.h>
#include <vehicle_data_index_def.h>

namespace smartlink {
    namespace vehicle {

        enum SignalID : SMLK_UINT32 {
            StartHere,
            // 
            ServiceBrakeCircuit1AirPressure,
            ServiceBrakeCircuit2AirPressure,
            //
            RechargeableEnergyStorageSubsystemsQuantity,
            RechargeableEnergyStorageSystemCodeLength,
            // M_GB_32960_MESSAGE_VEHICLE_INFO
            EVVehicleStatus,
            EVPowerBatteryChargeStatus,
            EVVehicleSpeed,
            TotalVehicleDistance,
            EVPowerBatteryVoltage,
            EVPowerBatteryCurrent,
            EVPowerBatterySOC,
            EVDCDCWorkStatus,
            EVCurrentGear,
            EVInsulationResistance,
            EVAcceleratorPedalPosition,
            EVBrakeSwitch,
            // M_GB_32960_MESSAGE_MOTOR_INFO
            EVMotorSysStatus,
            EVMotorCtrlTemp,
            EVMotorSpeed,
            EVMotorHighAccuracyTorque,
            EVMotorTorque,
            EVMotorDirection,
            EVMotorTemp,
            EVMotorInputVolt,
            EVMotorCurr,
            // M_GB_32960_MESSAGE_FUELCELL_INFO
            EVFuelCellVolt,
            EVFuelCellCurr,
            EVFuelCellRate,
            EVFuelCellProbeCnt,
            // SMLK_UINT8 probe_temp[M_GB_32960_SZ_PROBE_CNT_MAX];  // 探针温度值 1×Ⅳ 有效值范围：O～240(数值偏移薏40℃，表示-40℃～ +200℃)，最小计量单元：1'C
            EVHSysHighTemp,
            EVHSysHighTempNum,
            EVHSysHighConcentration,
            EVHSysHighConcentrationNum,
            EVHSysHighPress,
            EVHSysHighPressNum,
            EVHighDcdcState,
            EVWaterInTemp,
            EVWaterOutTemp,
            // M_GB_32960_MESSAGE_ENGINE_INFO
            EngineSpeed,
            EngineInstantaneousFuelEconomy,
            // M_GB_32960_MESSAGE_LOCATION_INFO
            LocatedStatus,
            Longitude,
            Latitude,
            // M_GB_32960_MESSAGE_EXTREME_VALUES
            EVMaxVoltCellSubsysSeq,
            EVMaxVoltSigleCellSeq,
            EVMaxVoltSigleCellValue,
            EVMinVoltCellSubsysSeq,
            EVMinVoltSigleCellSeq,
            EVMinVoltSigleCellValue,
            EVMaxTempSubsysSeq,
            EVMaxTempProbeSeq,
            EVMaxTempValue,
            EVMinTempSubsysSeq,
            EVMinTempProbeSeq,
            EVMinTempValue,
            // M_GB_32960_MESSAGE_ALERT_INFO

            EVEnergyStorageSubsysSeq,
            // M_GB_32960_MESSAGE_RECHARGE_DEVICE_VOLT_INFO
            EVEnergyStorageCellNum,
            // M_GB_32960_MESSAGE_RECHARGE_DEVICE_TEMP_INFO
            EVEnergyStorageTempProbeNum,
        };

        static const std::unordered_map<SignalID, SignalVariant>    g_vehicle_signals = {
                //
                {ServiceBrakeCircuit1AirPressure,                           SignalVariant(420,           16, 1,          0,          0,          2000)},             // kPa
                {ServiceBrakeCircuit2AirPressure,                           SignalVariant(421,           16, 1,          0,          0,          2000)},
                //
                {RechargeableEnergyStorageSubsystemsQuantity,               SignalVariant(410,           8,  1,          0,          0,          250)},
                {RechargeableEnergyStorageSystemCodeLength,                 SignalVariant(301,           8,  1,          0,          0,          50)},
                // M_GB_32960_MESSAGE_VEHICLE_INFO
                {EVVehicleStatus,                                           SignalVariant(310,           8,  1,          0,          0,          3)},
                {EVPowerBatteryChargeStatus,                                SignalVariant(311,           8,  1,          0,          0,          3)},
                {EVVehicleSpeed,                                            SignalVariant(317,           16, 0.1,        0.0,        0,          220)},           // 车速 km/h
                {TotalVehicleDistance,                                      SignalVariant(245,           32, 0.1,        0,          0,          999999.9)},                                                   // 累计里程 km
                {EVPowerBatteryVoltage,                                     SignalVariant(312,           16, 0.1,        0,          0,          1000)},
                {EVPowerBatteryCurrent,                                     SignalVariant(313,           16, 0.1,        -1000,      -1000,      1000)},
                {EVPowerBatterySOC,                                         SignalVariant(314,           8,  1,          0,          0,          100)},
                {EVDCDCWorkStatus,                                          SignalVariant(315,           8)},
                {EVCurrentGear,                                             SignalVariant(40,            8,  1,          -125,       -125,       125)},
                {EVInsulationResistance,                                    SignalVariant(316,           16, 1,          0,          0,          60000)},
                {EVAcceleratorPedalPosition,                                SignalVariant(4,             8)},
                {EVBrakeSwitch,                                             SignalVariant(45,            8)},
                // M_GB_32960_MESSAGE_MOTOR_INFO
                {EVMotorSysStatus,                                          SignalVariant(329,           8,  1,          0,          0,          16)},
                {EVMotorCtrlTemp,                                           SignalVariant(330,           8,  1,          -40,        -40,        210)},
                {EVMotorSpeed,                                              SignalVariant(331,           16)},
                {EVMotorHighAccuracyTorque,                                 SignalVariant(336,           8)},
                {EVMotorTorque,                                             SignalVariant(332,           8,  1,          -125,       -125,       125)},         
                {EVMotorDirection,                                          SignalVariant(337,           8)},
                {EVMotorTemp,                                               SignalVariant(333,           8,  1,          -40,        -40,        210)},
                {EVMotorInputVolt,                                          SignalVariant(334,           16, 0.1,        0,          0,          6000)},
                {EVMotorCurr,                                               SignalVariant(335,           16, 0.1,        -1000,      -1000,      1000)},
                // M_GB_32960_MESSAGE_FUELCELL_INFO
                {EVFuelCellVolt,                                            SignalVariant(350,           16, 0.1,        0,          0,          2000)},
                {EVFuelCellCurr,                                            SignalVariant(351,           16, 0.1,        0,          0,          2000)},
                {EVFuelCellRate,                                            SignalVariant(352,           16, 0.01,       0,          0,          600.00)},        
                {EVFuelCellProbeCnt,                                        SignalVariant(353,           16, 1,          0,          0,          65531)},
                {EVHSysHighTemp,                                            SignalVariant(355,           16, 0.1,        -40,        -40,        200)},
                {EVHSysHighTempNum,                                         SignalVariant(356,           8,  1,          0,          1,          252)},
                {EVHSysHighConcentration,                                   SignalVariant(357,           16, 1,          0,          0,          50000)},
                {EVHSysHighConcentrationNum,                                SignalVariant(358,           8,  1,          0,          1,          252)},
                {EVHSysHighPress,                                           SignalVariant(359,           16, 0.1,        0,          0,          100.0 * 10)}, //bar = 0.1 mpa
                {EVHSysHighPressNum,                                        SignalVariant(360,           8,  1,          0,          1,          252)},
                {EVHighDcdcState,                                           SignalVariant(361,           8)},
                // SMLK_UINT8 probe_temp[M_GB_32960_SZ_PROBE_CNT_MAX];  // 探针温度值 1×N 有效值范围：O～240(数值偏移量 40℃，表示 -40℃ ～ +200℃)，最小计量单元：1℃
                {EVWaterInTemp,                                             SignalVariant(354,           8,  1,          -40,        -40,        200)},
                {EVWaterOutTemp,                                            SignalVariant(362,           8,  1,          -40,        -40,        200)},
                // M_GB_32960_MESSAGE_ENGINE_INFO
                {EngineSpeed,                                               SignalVariant(25,            16, 1,          0,          0,          60000)},          // rpm // vec index
                {EngineInstantaneousFuelEconomy,                            SignalVariant(184,           16)},
                // M_GB_32960_MESSAGE_LOCATION_INFO
                {LocatedStatus,                                             SignalVariant(516133,        3) },
                {Longitude,                                                 SignalVariant(516131,        32, 0.000001,   0,          0,          180)},
                {Latitude,                                                  SignalVariant(516130,        32, 0.000001,   0,          0,          90)},
                // M_GB_32960_MESSAGE_EXTREME_VALUES
                {EVMaxVoltCellSubsysSeq,                                    SignalVariant(380,           8,  1,          0,          1,          250)},
                {EVMaxVoltSigleCellSeq,                                     SignalVariant(381,           8,  1,          0,          1,          250)},
                {EVMaxVoltSigleCellValue,                                   SignalVariant(382,           16, 0.001,      0,          0,          15)},
                {EVMinVoltCellSubsysSeq,                                    SignalVariant(383,           8,  1,          0,          1,          250)},
                {EVMinVoltSigleCellSeq,                                     SignalVariant(384,           8,  1,          0,          1,          250)},
                {EVMinVoltSigleCellValue,                                   SignalVariant(385,           16, 0.001,      0,          0,          15)},
                {EVMaxTempSubsysSeq,                                        SignalVariant(386,           8,  1,          0,          1,          250)},
                {EVMaxTempProbeSeq,                                         SignalVariant(387,           8,  1,          0,          1,          250)},
                {EVMaxTempValue,                                            SignalVariant(388,           8,  1,          -40,        -40,        210)},
                {EVMinTempSubsysSeq,                                        SignalVariant(389,           8,  1,          0,          1,          250)},
                {EVMinTempProbeSeq,                                         SignalVariant(390,           8,  1,          0,          1,          250)},
                {EVMinTempValue,                                            SignalVariant(391,           8,  1,          -40,        -40,        210)},
                // M_GB_32960_MESSAGE_ALERT_INFO

                {EVEnergyStorageSubsysSeq,                                  SignalVariant(411,           8,  1,          0,          1,          250)},
                // M_GB_32960_MESSAGE_RECHARGE_DEVICE_VOLT_INFO
                {EVEnergyStorageCellNum,                                    SignalVariant(414,           16, 1,          0,          1,          65531)},
                // M_GB_32960_MESSAGE_RECHARGE_DEVICE_TEMP_INFO
                {EVEnergyStorageTempProbeNum,                               SignalVariant(416,           16, 1,          0,          1,          65531)}
        };


        static const std::unordered_map <SignalID, SMLK_UINT32> g_vehicle_signal_index = {

                {ServiceBrakeCircuit1AirPressure,             VEHICLE_DATA_EV_SERVICE_BRAKE_CIRCUIT_1_AIR_PRESSURE},
                {ServiceBrakeCircuit2AirPressure,             VEHICLE_DATA_EV_SERVICE_BRAKE_CIRCUIT_2_AIR_PRESSURE},

                {RechargeableEnergyStorageSubsystemsQuantity, VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_SUBSYSTEMS_QUANTITY},
                {RechargeableEnergyStorageSystemCodeLength,   VEHICLE_DATA_EV_CODE_LENGTH_RECHARGEABLE_ENERGY_STORAGE_SYSTEM},

                {EVVehicleStatus,                             VEHICLE_DATA_EV_VEHICLE_STATUS},
                {EVPowerBatteryChargeStatus,                  VEHICLE_DATA_EV_POWER_BATTERY_CHARGING_STATUS},
                {EVVehicleSpeed,                              VEHICLE_DATA_EV_VEHICLE_SPEED},
                {TotalVehicleDistance,                        VEHICLE_DATA_TOTAL_VEHICLE_DISTANCE},
                {EVPowerBatteryVoltage,                       VEHICLE_DATA_EV_POWER_BATTERY_VOLTAGE},
                {EVPowerBatteryCurrent,                       VEHICLE_DATA_EV_POWER_BATTERY_CURRENT},
                {EVPowerBatterySOC,                           VEHICLE_DATA_EV_POWER_BATTERY_SOC},
                {EVDCDCWorkStatus,                            VEHICLE_DATA_EV_DCDC_WORKING_STATUS},
                {EVCurrentGear,                               VEHICLE_DATA_CURRENT_GEAR},
                {EVInsulationResistance,                      VEHICLE_DATA_EV_INSULATION_RESISTANCE},
                {EVAcceleratorPedalPosition,                  VEHICLE_DATA_ACCELERATOR_PEDAL_POSITION},
                {EVBrakeSwitch,                               VEHICLE_DATA_BRAKE_SWITCH},

                {EVMotorSysStatus,                            VEHICLE_DATA_EV_ELEC_ENGINE_SYSTEM_STATUS},
                {EVMotorCtrlTemp,                             VEHICLE_DATA_EV_ELEC_MACHINE_CTRL_TEMP},
                {EVMotorSpeed,                                VEHICLE_DATA_EV_ELEC_ENGINE_SPEED},
                {EVMotorHighAccuracyTorque,                   VEHICLE_DATA_EV_ELEC_ENGINE_HIGH_ACCURACY_TORQUE},
                {EVMotorTorque,                               VEHICLE_DATA_EV_ELEC_ENGINE_TORQUE},
                {EVMotorDirection,                            VEHICLE_DATA_EV_ELEC_ENGINE_ROTATION_DIRECTION},
                {EVMotorTemp,                                 VEHICLE_DATA_EV_ELEC_MACHINE_TEMP},
                {EVMotorInputVolt,                            VEHICLE_DATA_EV_ELEC_MACHINE_INPUT_VOL},
                {EVMotorCurr,                                 VEHICLE_DATA_EV_ELEC_MACHINE_CTRL_CURRENT},

                {EVFuelCellVolt,                              VEHICLE_DATA_EV_FUEL_BATTERY_VOL},
                {EVFuelCellCurr,                              VEHICLE_DATA_EV_FUEL_BATTERY_CURRENT},
                {EVFuelCellRate,                              VEHICLE_DATA_EV_FUEL_RATE},
                {EVFuelCellProbeCnt,                          VEHICLE_DATA_EV_FUEL_BATTERY_TEMP_NEEDLE_NUM},
                {EVHSysHighTemp,                              VEHICLE_DATA_EV_H_HIGHEST_TEMP},
                {EVHSysHighTempNum,                           VEHICLE_DATA_EV_H_HIGHEST_TEMP_NEEDLE_CODE},
                {EVHSysHighConcentration,                     VEHICLE_DATA_EV_H_HIGHEST_CONCENTRATION},
                {EVHSysHighConcentrationNum,                  VEHICLE_DATA_EV_H_HIGHEST_CONCENTRATION_CODE},
                {EVHSysHighPress,                             VEHICLE_DATA_EV_H_HIGHEST_PRESSURE},
                {EVHSysHighPressNum,                          VEHICLE_DATA_EV_H_HIGHEST_PRESSURE_SENSOR_CODE},
                {EVHighDcdcState,                             VEHICLE_DATA_EV_HIGH_PRESSURE_DC_STATUS},
                {EVWaterInTemp,                               VEHICLE_DATA_EV_WATER_IN_TEMP},
                {EVWaterOutTemp,                              VEHICLE_DATA_EV_WATER_OUT_TEMP},

                {EngineSpeed,                                 VEHICLE_DATA_ENGINE_SPEED},
                {EngineInstantaneousFuelEconomy,              VEHICLE_DATA_ENGINE_INSTANTANEOUS_FUEL_ECONOMY},

                {EVMaxVoltCellSubsysSeq,                      VEHICLE_DATA_EV_HIGHEST_VOLTAGE_BATTERY_SUBSYSTEM_NUMBER},
                {EVMaxVoltSigleCellSeq,                       VEHICLE_DATA_EV_NUMBER_OF_SINGLE_BATTERY_HIGHEST_VOLTAGE},
                {EVMaxVoltSigleCellValue,                     VEHICLE_DATA_EV_SINGLE_BATTERY_HIGHEST_VOLTAGE},
                {EVMinVoltCellSubsysSeq,                      VEHICLE_DATA_EV_LOWEST_VOLTAGE_BATTERY_SUBSYSTEM_NUMBER},
                {EVMinVoltSigleCellSeq,                       VEHICLE_DATA_EV_NUMBER_OF_SINGLE_BATTERY_MINIMUM_VOLTAGE},
                {EVMinVoltSigleCellValue,                     VEHICLE_DATA_EV_SINGLE_BATTERY_MINIMUM_VOLTAGE},
                {EVMaxTempSubsysSeq,                          VEHICLE_DATA_EV_MAXIMUM_TEMPERATURE_SUBSYSTEM_NUMBER},
                {EVMaxTempProbeSeq,                           VEHICLE_DATA_EV_NUMBER_OF_MAXIMUM_TEMPERATURE_PROBE_MONOMER},
                {EVMaxTempValue,                              VEHICLE_DATA_EV_MAXIMUM_TEMPERATURE_OF_POWER_BATTERY},
                {EVMinTempSubsysSeq,                          VEHICLE_DATA_EV_MINIMUM_TEMPERATURE_SUBSYSTEM_NUMBER},
                {EVMinTempProbeSeq,                           VEHICLE_DATA_EV_NUMBER_OF_MINIMUM_TEMPERATURE_PROBE_MONOMER},
                {EVMinTempValue,                              VEHICLE_DATA_EV_MINIMUM_TEMPERATURE_OF_POWER_BATTERY},

                {EVEnergyStorageSubsysSeq,                    VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_SUBSYSTEMS_NUMBER},

                {EVEnergyStorageCellNum,                      VEHICLE_DATA_EV_TOTAL_NUMBER_SINGLE_CELLS},

                {EVEnergyStorageTempProbeNum,                 VEHICLE_DATA_EV_RECHARGEABLE_ENERGY_STORAGE_TEMPERATURE_PROBES_QUANTITY},
        };


    }  // namespace vehicle
}  // namespace smartlink
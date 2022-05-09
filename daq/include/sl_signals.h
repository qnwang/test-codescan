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
    TachographVehicleSpeed,
#ifdef VEHICLE_VKA
    TachographVehicleSpeed_backup,
#endif
    EngineSpeed,
    AccPedalTravel,
    EngineFuelRate,
    EngineInstantaneousFuelEconomy,
    EngineIntakeManifoldPressure,
    EngineIntakeManifoldTemperature,
    EngineOilPressure,
    EngineCoolantTemperature,
    AmbientAirTemperature,
    BarometricPressure,
    FuelTemperature,
    TransmissionCurrentGear,
    TrailerWeight,
    AxleWeight,
    BrakePedalTravel,
    CatalystTankLevel,
    CatalystTankTemperature,
    ActualEnginePercentTorque,
    DriverDemandEnginePercentTorque,
    EngineLoadOnCurrentSpeed,
    NominalFrictionPercentTorque,
    PercentClutchSlip,
    FrontAxleSpeed,
    FrontAxleLeftWheelSpeed,
    FrontAxleRightWheelSpeed,
    RearAxle1LeftWheelSpeed,
    RearAxle1RightWheelSpeed,
    RearAxle2LeftWheelSpeed,
    RearAxle2RightWheelSpeed,
    YawRate,
    LateralAcceleration,
    LongitudinalAcceleration,
    EngineAverageFuelEconomy,
    TransmissionInputShaftSpeed,
    TransmissionOutputShaftSpeed,
    RetarderTorqueMode,
    AftertreatmentExhaustGasMassFlow,
    EngineIntakeAirMassFlowRat,
    AftertreatmentIntakeNOx,
    AftertreatmentOutletNOx,
    AftertreatmentIntakeGasSensorTemperature,
    AftertreatmentOutletGasSensorTemperature,
    AftertreatmentDieselParticulateFilterDifferentialPressure,
    ChargingSystemPotential,
    EngineTotalHoursOfOperation,
    EngineTotalRevolutions,
    EngineAirIntakePressure,
    EngineExhaustGasTemperature,
    EngineAirIntakeTemperature,
    EngineWaitToStartLamp,
    WaterInFuelIndicator,
    TransmissionActualGearRatio,
    FuelLevel,
    ReferenceEngineTorque,
    RelativeLevelFrontAxleLeft,
    RelativeLevelFrontAxleRight,
    RelativeLevelRearAxleLeft,
    RelativeLevelRearAxleRight,
    BellowPressureFrontAxleLeft,
    BellowPressureFrontAxleRight,
    BellowPressureRearAxleLeft,
    BellowPressureRearAxleRight,
    VehicleSwitchStatus,
    TBoxAcceleration,

    NetworkSignalStrength,
    NumberOfPositioningSatellites,

    Battery1Temperature,
    Battery1Voltage,
    Battery1SOC,
    Battery1SOH,
    Battery2Temperature,
    Battery2Voltage,
    Battery2SOC,
    Battery2SOH,
    KeyswitchBatteryPotential,
    AirSuspensionControl1,
    AirSuspensionControl2,
    RetarderSelection,

    OverrideControlMode,
    RequestSpeedLimit,
    RequestTorqueLimit,
    RequestTorqueHighResolution,

    IlluminationBrightnessPercent,
    ATMTransmissionMode,
    GNSSTimeStamp,
    GNSSLatitude,
    GNSSLongitude,
    GNSSSpeed,
    GNSSLocatedStatus,
    GNSSAltitude,
    AirSuspensionControl3,
    AirSuspensionControl4,
    ActuralCatalystSpewed,
    TotalCatalystSpewed,
    TurnSwitchStatus,
    AirConditionerOnOffStatus,
    AirConditionerModeStatus,
    AirConditionerAutoModeStatus,
    AirConditionerBlowingRate,
    AirConditionerACModeStatus,
    AirConditionerCirculationModeStatus,
    WarmAirBlowerOnOffStatus,
    WarmAirTemperatureStatus,
    ParkingAirConditionerOnOffStatus,
    ParkingAirConditionerBlowingRate,
    ACRegulationTemperature,
    ParkingACRegulationTemperature,
    CabTemperature,
    ACRegulationHRTemperature,
    ParkingACRegulationHRTemperature,
    AirConditionerVentilationStatus,
    ParkingACAutoModeStatus,
    DieselParticulateFilterStatus,
    DieselParticulateFilterLamp,
    DieselParticulateFilterActiveRegenerationStatus,
    DieselParticulateFilterActiveRegenerationInhibitedStatus,
    DriverInducementIndicatorStatus,
    MalfunctionIndicatorLampStatus,
    AftertreatmentRegenerationInhibitSwitch,

    EngineBrakingPercentTorque,
    EngineExhaustPercentTorque,
    ActualRetarderPercentTorque,
    DriverDemandRetarderPercentTorque,
    ActualMaximumAvailableRetarderPercentTorque,
    TransmissionSelectedGear,
    SteeringWheelAngle,
    PitchAngle,
    CurrentSlope,

    EngineTorqueMode,
    EngineTotalFuelUsed,

    EngineRetarderTorqueModes,

    TotalECUDistance,
    ParkingBrakeSwitch,
    ClutchSwitch,
    BrakeSwitch,
    WheelSpeed,

    TripDistance,
    TotalVehicleDistance,

    CalculusDistance,
    CalculusFuelConsumption,
    CalculusDayDistance,
    CalculusDayFuelConsumption,

    AlarmFlags,

    GNSSHeading,
    GNSSSatelliteUsed,
    GNSSSatelliteVisible,

    ExtendedSignalStatus,
    RSSI,
    ACCStatus,
    GNSSLocatedStatusEx,

    BrightnensPercent,
    CruiseControlSetSpeed,

    EmergencyBrakingSystemState,
    EmergencyBrakingTargetID,
    EmergencyTargetRelativeSpeed,
    EmergencyTargetSpeed,
    EmergencyTargetDistance,

    CabInteriorTemperature,
    AcceleratorChangeRate,

    TireLocation,
    TirePressure,
    TireTemperature,
    TireStatus,
    TireExtrentPressureSupport,
    TireAirLeakageRate,
    TirePressureThresholdDetection,
    TirePressureThresholdDetectionOriginal,
    TirePressureExtendedRange,
    TirePressureRequired,
    TireLocation1,
    TireLocation2,

    TransmissionMode1,
    TransmissionMode2,
    TransmissionMode3,
    TransmissionMode4,

    NominalLevelRearAxle,
    NominalLevelFrontAxle,
    AboveNominalLevelRearAxle,
    AboveNominalLevelFrontAxle,
    BelowNominalLevelFrontAxle,
    LiftingControlModeFrontAxle,
    LoweringControlModeFrontAxle,
    LevelControlMode,
    LiftAxle1Position,
    BelowNominalLevelRearAxle,
    LoweringControlModeRearAxle,
    LiftingControlModeRearAxle,
    LiftAxle2Position,
    RearAxleInBumperRange,
    FrontAxleInBumperRange,
    SuspensionRemoteControl1,
    SuspensionControlRefusalInformation,

    ActualMinimumSOC,
    ActualMinimumSOH,
    AgtatorSpeed,
    AgtatorState,
    AgtatorTotalRotations,

    StatisticsTripStartTime,
    StatisticsTripEndTime,
    StatisticsGnssSpeedDistance,
    StatisticsTachographVehicleSpeedDistance,
    StatisticsVehicleSpeedLifeTotalDistance,
    StatisticsVehicleEstimatedLoad,
    StatisticsTripFuelUsed,
    StatisticsTotalFuelUsed,
    StatisticsVehicleTotalFuelUsed,
    StatisticsAverageFuelEconomy,
    StatisticsSpeedingTimes,
    StatisticsSpeedingDistance,
    StatisticsSpeedingFuelUsed,
    StatisticsBrakePedalBrakeTimes,
    StatisticsBrakePedalBrakeDistance,
    StatisticsBrakePedalBrakeDuration,
    StatisticsSharpShowDownTimes,
    StatisticsSharpShowDownDuration,
    StatisticsSharpShowDownDistance,
    StatisticsSharpSpeedUpTimes,
    StatisticsSharpSpeedUpDuration,
    StatisticsSharpSpeedUpDistance,
    StatisticsSharpSpeedUpFuelUsed,
    StatisticsSharpTrunningTimes,
    StatisticsSharpTrunningDistance,
    StatisticsSleepyDriveTimes,

    //add by wukai 20210807
    SMLK_AsrBrakeControl,        // ABS 介入信号
    SMLK_AsrEngineControl,       // TCS 介入信号
    SMLK_TrailerConnected,       // 挂车连接上信号
    SMLK_OpenStatusDoorDrv,      // 车门关信号/驾驶员车门1开关信号
    SMLK_OpenStatusDoorPas,      // 车门关信号/乘客车门2开关信号
    SMLK_CruiseControl,          // 巡航激活状态
    SMLK_TrailerParkBrakeSwitch, // 挂车驻车状态
    SMLK_ReverseGearSwitch,      // 倒挡开关状态
    SMLK_FuelLeakageAlert,       // 燃油泄漏报警

    //add 0200-E7 signal by wujian 20211227
    EnginePowerInput1,               //发动机输入电压
    keyswitchBatteryVoltage,         //点火开关电压
    Sensor1Humidity,                 //传感器1 湿度
    Sensor2Humidity,                 //传感器2 湿度
    Sensor3Humidity,                 //传感器3 湿度
    Sensor4Humidity,                 //传感器4 湿度
    Sensor5Humidity,                 //传感器5 湿度
    Sensor6Humidity,                 //传感器6 湿度
    Sensor7Humidity,                 //传感器7 湿度
    Sensor8Humidity,                 //传感器8 湿度
    Sensor1Temperature,              //传感器1 温度
    Sensor2Temperature,              //传感器2 温度
    Sensor3Temperature,              //传感器3 温度
    Sensor4Temperature,              //传感器4 温度
    Sensor5Temperature,              //传感器5 温度
    Sensor6Temperature,              //传感器6 温度
    Sensor7Temperature,              //传感器7 温度
    Sensor8Temperature,              //传感器8 温度
    PefrigerationSwitchStatus,       //制冷机组开关状态
    PefrigerationWorkMode,           //制冷机组工作模式
    PefrigerationCpmpartmentTemperature,//冷藏室温度
    DefrostTemperature,                 //除霜温度
    SupplyVoltage,                      //供电电压
    // update 0200-E7 2022-02-11
    DcAcPowerConsumption,               // 逆变器功率
    SteerWheelHeatPowerConsumption,     // 方向盘加热功率
    CigarLighterPowerConsumption,       // 点烟器功率
    Socket1PowerConsumption,            // 24V供电插座1功率
    Socket2PowerConsumption,            // 24V供电插座2功率
    LeftSeatHeatPowerConsumption,       // 驾驶员座椅加热功率
    RightSeatHeatPowerConsumption,      // 副驾驶员座椅加热功率
    LowerBerthtHeatPowerConsumption,    // 下卧铺加热功率
    SleepReadLampPowerConsumption,      // 卧铺灯功率
    ElevatedBoxPowerConsumption,        // 高架箱灯功率
    HumidityInBox1,                     // 箱内湿度 1
    HumidityInBox2,                     // 箱内湿度 2
    TemperatureInBox1,                  // 箱内温度 1
    TemperatureInBox2,                  // 箱内温度 2
    // add 0200-EC 2022-03-08
    TboxVlotage,                        // Tbox 供电电压
    FuelSaveSwitch,                     // 节油开关
    HydraulicMotorOilReturnTemperature, // 液压马达回油温
    EvVehicleStatus,                    // 车辆状态 新能源
    EvChargingStatus,                   // 充电状态
    EvWorkingStatus,                    // 运行状态
    EvVehicleSpeed,                     // 车速 新能源 VEHICLE_DATA_EV_VEHICLE_SPEED
    EvTotalVoltage,                     // 总电压
    EvTotalCurrent,                     // 总电流
    EvBatterySoc,                       // SOC
    EvDcDcWoringStatus,                 // DCDC状态
    EvGearStatus,                       // 档位 daq组合计算
    EvInsulationResistance              // 绝缘电阻
};  // enum class Signal



// all the following signal defines should be auto generated from DBC files.
static const std::unordered_map<SignalID, SignalVariant>    g_vehicle_signals = {
    { GNSSTimeStamp,                                                SignalVariant(516129,       32)                                                     },
    { GNSSLatitude,                                                 SignalVariant(516130,       32, 0.000001,   0,          0,          90)             },
    { GNSSLongitude,                                                SignalVariant(516131,       32, 0.000001,   0,          0,          180)            },
    { GNSSSpeed,                                                    SignalVariant(516132,       16, 0.1,        0,          0,          150)                                                },
    { GNSSLocatedStatus,                                            SignalVariant(516133,       3)                                                      },
    { GNSSAltitude,                                                 SignalVariant(516134,       32, 1.0/256.0)                                          },
    { GNSSHeading,                                                  SignalVariant(0,            16, 1,          0,          0,          359)            },
    { GNSSSatelliteUsed,                                            SignalVariant(0,            8)                                                      },
    { GNSSSatelliteVisible,                                         SignalVariant(0,            8)                                                      },
    { GNSSLocatedStatusEx,                                          SignalVariant(516133,       16)                                                     },
    { TachographVehicleSpeed,                                       SignalVariant(1624,         16, 1.0/256.0,  0.0,        0,          250.99609375)   },
#ifdef VEHICLE_VKA
    { TachographVehicleSpeed_backup,                                SignalVariant(162402,         16, 1.0/256.0,  0.0,        0,          250.99609375)   },
#endif
    { EngineTorqueMode,                                             SignalVariant(900,          4)                                                      },
    { DriverDemandEnginePercentTorque,                              SignalVariant(512,          8,  1,          -125,       -125,       125)            },
    { ActualEnginePercentTorque,                                    SignalVariant(513,          8,  1,          -125,       -125,       125)            },
    { EngineSpeed,                                                  SignalVariant(190,          16, 0.125,      0,          0,         8031.875)        },
    { EngineCoolantTemperature,                                     SignalVariant(110,          8,  1,          -40,        -40,        210)            },
    { EngineOilPressure,                                            SignalVariant(100,          8,  4,          0,          0,          1000)           },
    { EngineTotalFuelUsed,                                          SignalVariant(250,          32, 0.5,        0,          0,          2105540607.5)   },
    { AccPedalTravel,                                               SignalVariant(91,           8,  0.4,        0,          0,          100)            },
    { ParkingBrakeSwitch,                                           SignalVariant(0,            2, 1, 0, 0, 1)                                                      },
    { WheelSpeed,                                                   SignalVariant(0,            16, 1.0/256.0,  0,          0,          250.996095)        },
    { ClutchSwitch,                                                 SignalVariant(0,            2)                                                      },
    { BrakeSwitch,                                                  SignalVariant(0,            2)                                                      },
    { CatalystTankLevel,                                            SignalVariant(0,            8,  0.4,        0,          0,          100)            },
    { EngineTotalHoursOfOperation,                                  SignalVariant(247,          32, 0.05,       0,          0,          210554060.75)   },
    { EngineTotalRevolutions,                                       SignalVariant(249,          32, 1000,          0,          0,       4211081215000)  }, // it should be can origin value
    { EngineFuelRate,                                               SignalVariant(183,          16, 0.05,       0,          0,          3212.75)        },
    { EngineInstantaneousFuelEconomy,                               SignalVariant(184,          16, 1.0/512.0,  0,          0,          125.5)          },
    { EngineAverageFuelEconomy,                                     SignalVariant(185,          16, 1.0/512.0,  0,          0,          125.499)         },
    { AftertreatmentDieselParticulateFilterDifferentialPressure,    SignalVariant(3251,         16, 0.1,        0,          0,          64255)          },
    { EngineIntakeManifoldPressure,                                 SignalVariant(102,          8,  2,          0,          0,          500)            },
    { EngineIntakeManifoldTemperature,                              SignalVariant(105,          8,  1,          -40,        -40,        210)            },
    { EngineAirIntakePressure,                                      SignalVariant(0,            8,  2,          0,          0,          500)            },
    { BarometricPressure,                                           SignalVariant(108,          8,  0.5,        0,          0,          125)            },
    { CabInteriorTemperature,                                       SignalVariant(170,          16, 0.03125,    -273,       -273,       1735)           },
    { AmbientAirTemperature,                                        SignalVariant(171,          16, 0.03125,    -273,       -273,       1735)           },
    { EngineAirIntakeTemperature,                                   SignalVariant(0,            8,  1,          -40,        -40,        210)            },
    { EngineWaitToStartLamp,                                        SignalVariant(0,            2, 1, 0, 0, 1)                                                      },
    { WaterInFuelIndicator,                                         SignalVariant(0,            2, 1, 0, 0, 1)                                                      },
    { TransmissionSelectedGear,                                     SignalVariant(524,          8,  1,          -125,       -125,       125)            },
    { TransmissionActualGearRatio,                                  SignalVariant(0,            16, 0.001)                                              },
    { TransmissionCurrentGear,                                      SignalVariant(523,          8,  1,          -125,       -125,       125)            },
    { FuelLevel,                                                    SignalVariant(0,            8,  0.4,        0,          0,          100)            },
    { TripDistance,                                                 SignalVariant(244,          32, 0.125,      0,          0,          526385151.9)    },
    { TotalVehicleDistance,                                         SignalVariant(245,          32, 0.125,      0,          0,          526385151.9)    },
    { CalculusDistance,                                             SignalVariant(0,            32, 1,       0,          0,          4294967295)     },
    { CalculusFuelConsumption,                                      SignalVariant(0,            32, 0.0005,     0,          0,          2147483.6475)   },
    { ActuralCatalystSpewed,                                        SignalVariant(516137,       32, 0.01,          0,          0,       42110812.15)     },
    { TotalCatalystSpewed,                                          SignalVariant(516138,       32, 1,          0,          0,          4211081215)     },
    { EngineExhaustGasTemperature,                                  SignalVariant(173,          16, 0.03125,    -273,       -273,       1735)           },

#if 0
    { TachographVehicleSpeed,                                       SignalVariant(1624,         16, 1.0/256.0,  0.0,        -125,       125)            },      // 1
    { EngineSpeed,                                                  SignalVariant(190,          16, 0.125)                                              },      // 2
    { ActualEnginePercentTorque,                                    SignalVariant(513,          8,  1,          -125,       -125,       125)            },      // 3
    { DriverDemandEnginePercentTorque,                              SignalVariant(512,          8,  1,          -125,       -125,       125)            },      // 4
    { AccPedalTravel,                                               SignalVariant(91,           8,  0.4,        0,          0,          100)            },      // 5
    { EngineFuelRate,                                               SignalVariant(183,          16, 0.05,       0,          0,          3212.75)        },      // 6
    { EngineInstantaneousFuelEconomy,                               SignalVariant(184,          16, 1.0/512.0,  0,          0,          125.5)          },      // 7
#endif
    { VehicleSwitchStatus,                                          SignalVariant(516097,       16)                                                     },      // 8
    { TBoxAcceleration,                                             SignalVariant(516098,       8,  1,                0, -(250/3.6),    250/3.6 )       },      // 9
    { LateralAcceleration,                                          SignalVariant(1809,         16, 1.0/2048.0, -15.687,    -15.687,    (64255.0/2048.0-15.687))},      // 10
    { LongitudinalAcceleration,                                     SignalVariant(1810,         8,  0.1,        -12.5,      -12.5,      12.5)           },      // 11
    { YawRate,                                                      SignalVariant(1808,         16, 1.0/8192.0, -3.92,      -3.92,      3.92363)           },      // 12
    { EngineIntakeManifoldPressure,                                 SignalVariant(102,          8,  2,          0,          0,          500)            },      // 13
#if 0
    { EngineOilPressure,                                            SignalVariant(100,          8,  4,          0,          0,          1000)           },      // 14
#endif
    { EngineCoolantTemperature,                                     SignalVariant(110,          8,  1,          -40,        0,          210)            },      // 15
    //{ EngineBrakingPercentTorque,                                   SignalVariant(0x18F0000F,   8,  1,          -125,       -125,       125)            },      // 16
    { EngineBrakingPercentTorque,                                   SignalVariant(0x0F << 24 | 520,   8,  1,          -125,       -125,       125)            },      // 16
    //{ EngineExhaustPercentTorque,                                   SignalVariant(0x18F00029,   8,  1,          -125,       -125,       125)            },      // 17
    { EngineExhaustPercentTorque,                                   SignalVariant(0x29 << 24 | 520,   8,  1,          -125,       -125,       125)            },      // 17
    { RetarderSelection,                                            SignalVariant(516119,       3,  1,          0,          0,          7)              },      // 18
    //{ ActualRetarderPercentTorque,                                  SignalVariant(0x18F00010,   8,  1,          -125,       -125,       125)            },      // 19

    { ActualRetarderPercentTorque,                                  SignalVariant(0x10 << 24 | 520,   8,  1,          -125,       -125,       125)            },      // 19
    { RetarderTorqueMode,                                           SignalVariant(516120,          4)                                                      },      // 20
    // TODO to be verified the correct SPN for EngineRetarderTorqueModes?
    //{ EngineRetarderTorqueModes,                                    SignalVariant(516120,       4)                                                      },      // 20 ?
    { DriverDemandRetarderPercentTorque,                            SignalVariant(1715,         8,  1,          -125,       -125,       125)            },      // 21
    { ActualMaximumAvailableRetarderPercentTorque,                  SignalVariant(1717,         8,  1,          -125,       -125,       125)            },      // 22
#if 0
    { TransmissionCurrentGear,                                      SignalVariant(523,          8,  1,          -125,       -125,       125)            },      // 23
    { TransmissionSelectedGear,                                     SignalVariant(524,          8,  1,          -125,       -125,       125)            },      // 24
#endif
    { SteeringWheelAngle,                                           SignalVariant(1807,         8,  1.0/1024.0, -31.375,    -31.375,    31.374024)         },      // 25
    { FrontAxleSpeed,                                               SignalVariant(904,          8,  1.0/256.0,  0,          0,          250.996095)        },      // 26
    { FrontAxleLeftWheelSpeed,                                      SignalVariant(905,          8,  1.0/16.0,   -7.8125,    -7.8125,    7.8125)         },      // 27
    { FrontAxleRightWheelSpeed,                                     SignalVariant(906,          8,  1.0/16.0,   -7.8125,    -7.8125,    7.8125)         },      // 28
    { RearAxle1LeftWheelSpeed,                                      SignalVariant(907,          8,  1.0/16.0,   -7.8125,    -7.8125,    7.8125)         },      // 29
    { RearAxle1RightWheelSpeed,                                     SignalVariant(908,          8,  1.0/16.0,   -7.8125,    -7.8125,    7.8125)         },      // 30
    { BrakePedalTravel,                                             SignalVariant(521,          8,  0.4,        0,          0,          100)            },      // 31
    { TransmissionInputShaftSpeed,                                  SignalVariant(161,          16, 0.125,      0,          0,          8032)           },      // 32
    { TransmissionOutputShaftSpeed,                                 SignalVariant(191,          16, 0.125,      0,          0,          8032)           },      // 33
    { EngineLoadOnCurrentSpeed,                                     SignalVariant(92,           8,  1,          0,          0,          250)            },      // 34
    { NominalFrictionPercentTorque,                                 SignalVariant(514,          8,  1,          -125,       -125,       125)            },      // 35
    { PercentClutchSlip,                                            SignalVariant(522,          8,  0.4,        0,          0,          100)            },      // 36
    { EngineIntakeAirMassFlowRat,                                   SignalVariant(132,          16, 0.05,       0,          0,          3212.75)        },      // 37
    { OverrideControlMode,                                          SignalVariant(516121,       8)                                                      },      // 38
    { RequestSpeedLimit,                                            SignalVariant(516122,       16, 0.125,      0,          0,          8031.875)       },      // 39
    { RequestTorqueLimit,                                           SignalVariant(516123,       8,  1,          -125,       -125,       125)            },      // 40
    { RequestTorqueHighResolution,                                  SignalVariant(516124,       4)                                                      },      // 41
    { PitchAngle,                                                   SignalVariant(516125,       16, 0.002,      -64,        -64,        64.51)          },      // 42
    { CurrentSlope,                                                 SignalVariant(516126,       10, 0.1,        -51.1,      -51.1,      51.1)           },      // 43
    { AirSuspensionControl1,                                        SignalVariant(516117,       32)                                                     },      // 44
    { AirSuspensionControl2,                                        SignalVariant(516118,       10)                                                     },      // 45
    { AirSuspensionControl3,                                        SignalVariant(516135,       32)                                                     },      // 46
    { AirSuspensionControl4,                                        SignalVariant(516136,       32)                                                     },      // 47
    { RelativeLevelFrontAxleLeft,                                   SignalVariant(1721,         16, 0.1,        -3200,      -3200,          3225.5)         },      // 46 - 1
    { RelativeLevelFrontAxleRight,                                  SignalVariant(1722,         16, 0.1,        -3200,      -3200,          3225.5)         },      // 46 - 2
    { RelativeLevelRearAxleLeft,                                    SignalVariant(1723,         16, 0.1,        -3200,      -3200,          3225.5)         },      // 46 - 3
    { RelativeLevelRearAxleRight,                                   SignalVariant(1724,         16, 0.1,        -3200,      -3200,          3225.5)         },      // 46 - 4
    { BellowPressureFrontAxleLeft,                                  SignalVariant(1725,         16, 0.1,        0,          0,          6425.5)         },      // 47 - 1
    { BellowPressureFrontAxleRight,                                 SignalVariant(1726,         16, 0.1,        0,          0,          6425.5)         },      // 47 - 2
    { BellowPressureRearAxleLeft,                                   SignalVariant(1727,         16, 0.1,        0,          0,          6425.5)         },      // 47 - 3
    { BellowPressureRearAxleRight,                                  SignalVariant(1728,         16, 0.1,        0,          0,          6425.5)         },      // 47 - 4
#if 0
    { ActuralCatalystSpewed,                                        SignalVariant(516137,       32, 0.01,       0,          0,          42110812.15)    },      // 48
    { TotalCatalystSpewed,                                          SignalVariant(516138,       32, 1,          0,          0,          4211081215)     },      // 49
#endif
    { CatalystTankTemperature,                                      SignalVariant(3031,         8,  1,          -40,        -40,        210)            },      // 50
    { TurnSwitchStatus,                                             SignalVariant(516139,       4)                                                      },      // 51
#if 0
    { TireLocation1,                                                SignalVariant(516140,       8,  1,          0,          0,          250)            },      // 52
    { TirePressure,                                                 SignalVariant(516141,       8,  4,          0,          0,          1000)           },      // 53
    { TireTemperature,                                              SignalVariant(516142,       16, 0.03125,    -273,       -273,       1734.96875)     },      // 54
    { TireStatus,                                                   SignalVariant(516143,       2)                                                      },      // 55
    { TireExtrentPressureSupport,                                   SignalVariant(516144,       2)                                                      },      // 56
    { TireAirLeakageRate,                                           SignalVariant(516145,       16, 0.1,        0,          0,          6425.5)         },      // 57
    { TirePressureThresholdDetection,                               SignalVariant(516146,       2)                                                      },      // 58
    { TireLocation2,                                                SignalVariant(516147,       8,  1,          0,          0,          250)            },      // 59
    { TireExtendedPressureRange,                                    SignalVariant(516148,       16, 1,          0,          0,          64255)          },      // 60
    { TirePressureRequired,                                         SignalVariant(516149,       16, 1,          0,          0,          64255)          },      // 61
#endif
    { AirConditionerOnOffStatus,                                    SignalVariant(516150,       2)                                                      },      // 61
    { AirConditionerModeStatus,                                     SignalVariant(516151,       4,  1,          0,          0,          15)              },      // 63
    { AirConditionerAutoModeStatus,                                 SignalVariant(516152,       2)                                                      },      // 64
    { AirConditionerBlowingRate,                                    SignalVariant(516153,       4)                                                      },      // 65
    { AirConditionerACModeStatus,                                   SignalVariant(516154,       2)                                                      },      // 66
    { AirConditionerCirculationModeStatus,                          SignalVariant(516155,       2)                                                      },      // 67
    { WarmAirBlowerOnOffStatus,                                     SignalVariant(516156,       2)                                                      },      // 68
    { WarmAirTemperatureStatus,                                     SignalVariant(516157,       4)                                                      },      // 69
    { ParkingAirConditionerOnOffStatus,                             SignalVariant(516158,       2)                                                      },      // 70
    { ParkingAirConditionerBlowingRate,                             SignalVariant(516159,       4)                                                      },      // 71
    { ACRegulationTemperature,                                      SignalVariant(516160,       8,  1,          -40,        -40,        210)            },      // 72
    { ParkingACRegulationTemperature,                               SignalVariant(516161,       8,  1,          -40,        -40,        210)            },      // 73
    { CabTemperature,                                               SignalVariant(516162,       8,  1,          -40,        -40,        210)            },      // 74
    { ACRegulationHRTemperature,                                    SignalVariant(516163,       16, 0.03125,    -273,       -273,       1734.96875)     },      // 75
    { ParkingACRegulationHRTemperature,                             SignalVariant(516164,       16, 0.03125,    -273,       -273,       1734.96875)     },      // 76
    { AirConditionerVentilationStatus,                              SignalVariant(516165,       2)                                                      },      // 77
    { ParkingACAutoModeStatus,                                      SignalVariant(516166,       2)                                                      },      // 78
    { DieselParticulateFilterStatus,                                SignalVariant(516167,       4)                                                      },      // 79
    { DieselParticulateFilterLamp,                                  SignalVariant(516168,       3)                                                      },      // 80
    { DieselParticulateFilterActiveRegenerationStatus,              SignalVariant(516169,       2)                                                      },      // 81
    { DieselParticulateFilterActiveRegenerationInhibitedStatus,     SignalVariant(516170,       2)                                                      },      // 82
    { DriverInducementIndicatorStatus,                              SignalVariant(516171,       4)                                                      },      // 83
    { MalfunctionIndicatorLampStatus,                               SignalVariant(516172,       2)                                                      },      // 84
    { AftertreatmentRegenerationInhibitSwitch,                      SignalVariant(516173,       2)                                                      },      // 85

    { TrailerWeight,                                                SignalVariant(180,          16, 10,          0,          0,          642550)         },      // 1
    { BrightnensPercent,                                            SignalVariant(516127,       8,  0.4,          0,          0,          100)            },      // 2
#if 0
    { EngineAverageFuelEconomy,                                     SignalVariant(185,          16, 1.0/512.0,  0,          0,          125.19)         },      // 3
#endif
    { AxleWeight,                                                   SignalVariant(582,          16, 0.5,          0,          0,          32127.51)         },      // 4
    { AftertreatmentIntakeNOx,                                      SignalVariant(3216,         16, 0.05,       -200,       -200,       3012.75)        },      // 5
    { AftertreatmentOutletNOx,                                      SignalVariant(3226,         16, 0.05,       -200,       -200,       3012.75)        },      // 6
    { AftertreatmentIntakeGasSensorTemperature,                     SignalVariant(4360,         16, 0.03125,    -273,       -273,       1734.97)        },      // 7
    { AftertreatmentOutletGasSensorTemperature,                     SignalVariant(4363,         16, 0.03125,    -273,       -273,       1734.97)        },      // 8
    { ATMTransmissionMode,                                          SignalVariant(516128,       6)                                                      },      // 9
    { ReferenceEngineTorque,                                        SignalVariant(544,          16,  1,          0,          0,          64255)          },      // 10
    { FuelTemperature,                                              SignalVariant(174,          8,  1,          -40,        -40,        210)            },      // 11
    { CruiseControlSetSpeed,                                        SignalVariant(86,           8,  1,          0,          0,          250)            },      // 12

    { NominalLevelRearAxle,                                         SignalVariant(1733,         4)                                                      },
    { NominalLevelFrontAxle,                                        SignalVariant(1734,         4)                                                      },
    { AboveNominalLevelRearAxle,                                    SignalVariant(1736,         2)                                                      },
    { AboveNominalLevelFrontAxle,                                   SignalVariant(1737,         2)                                                      },
    { BelowNominalLevelFrontAxle,                                   SignalVariant(1738,         2)                                                      },
    { LiftingControlModeFrontAxle,                                  SignalVariant(1739,         2)                                                      },
    { LoweringControlModeFrontAxle,                                 SignalVariant(1740,         2)                                                      },
    { LevelControlMode,                                             SignalVariant(1741,         4)                                                      },
    { LiftAxle1Position,                                            SignalVariant(1743,         2)                                                      },
    { BelowNominalLevelRearAxle,                                    SignalVariant(1754,         2)                                                      },
    { LoweringControlModeRearAxle,                                  SignalVariant(1755,         2)                                                      },
    { LiftingControlModeRearAxle,                                   SignalVariant(1756,         2)                                                      },
    { LiftAxle2Position,                                            SignalVariant(1822,         2)                                                      },
    { RearAxleInBumperRange,                                        SignalVariant(1823,         2)                                                      },
    { FrontAxleInBumperRange,                                       SignalVariant(1824,         2)                                                      },
    { SuspensionRemoteControl1,                                     SignalVariant(1826,         2)                                                      },
    { SuspensionControlRefusalInformation,                          SignalVariant(1827,         4)                                                      },

    { TransmissionMode1,                                            SignalVariant(1852,         2)                                                      },
    { TransmissionMode2,                                            SignalVariant(1853,         2)                                                      },
    { TransmissionMode3,                                            SignalVariant(1854,         2)                                                      },
    { TransmissionMode4,                                            SignalVariant(1855,         2)                                                      },

    { EmergencyBrakingSystemState,                                  SignalVariant(0,            4)                                                      },
    { EmergencyBrakingTargetID,                                     SignalVariant(0,            6)                                                      },
    { EmergencyTargetRelativeSpeed,                                 SignalVariant(0,            16)                                                     },
    { EmergencyTargetSpeed,                                         SignalVariant(0,            16)                                                     },
    { EmergencyTargetDistance,                                      SignalVariant(0,            16)                                                     },
    { AlarmFlags,                                                   SignalVariant(0,            32)                                                     },
    { ExtendedSignalStatus,                                         SignalVariant(0,            32)                                                     },
    { RSSI,                                                         SignalVariant(0,            8,  1,          0,          0,          100)            },
    { ACCStatus,                                                    SignalVariant(0,            1)                                                      },

    { TireLocation1,                                                SignalVariant(516140,       8,  1,          0,          0,          254)            },      // 52
    { TirePressure,                                                 SignalVariant(516141,       8,  4,          0,      0,      1000)                   },
    { TireTemperature,                                              SignalVariant(516142,       16, 0.03125,    -273,   -273,   1734.96875)             },
    { TireStatus,                                                   SignalVariant(516143,       2)                                                      },
    { TireExtrentPressureSupport,                                   SignalVariant(516144,       2)                                                      },
    { TireAirLeakageRate,                                           SignalVariant(516145,       16, 0.1,        0,      0,      6425.5)                 },
    { TirePressureThresholdDetection,                               SignalVariant(516146,       3)                                                      },
    { TireLocation2,                                                SignalVariant(516147,       8,  1,          0,          0,          254)            },      // 59
    { TirePressureExtendedRange,                                    SignalVariant(516148,       16, 1,          0,          0,          64255)          },
    { TirePressureRequired,                                         SignalVariant(516149,       16, 1,          0,          0,          64255)          },

    { Battery1Temperature,                                          SignalVariant(516108,       8,  1,          -40,        -40,        210)            },
    { Battery1Voltage,                                              SignalVariant(516109,       16, 0.05,       0,          0,          3212.75)        },
    { Battery1SOC,                                                  SignalVariant(516110,       16, 0.0025,     0,          0,          160.63751)      },
    { Battery1SOH,                                                  SignalVariant(516111,       8,  0.5,        0,          0,          125)            },
    { Battery2Temperature,                                          SignalVariant(516112,       8,  1,          -40,        -40,        210)            },
    { Battery2Voltage,                                              SignalVariant(516113,       16, 0.05,       0,          0,          3212.75)        },
    { Battery2SOC,                                                  SignalVariant(516114,       16, 0.0025,     0,          0,          160.63751)      },
    { Battery2SOH,                                                  SignalVariant(516115,       8,  0.5,        0,          0,          125)            },
    { ActualMinimumSOC,                                             SignalVariant(0,            16, 0.0025,     0,          0,          160.63751)      },
    { ActualMinimumSOH,                                             SignalVariant(0,            8,  0.5,        0,          0,          125)            },
    { AgtatorState,                                                 SignalVariant(516174,       2)                                                      },
    { AgtatorSpeed,                                                 SignalVariant(516175,       8,  0.25,       0,          0,          20)             },
    { AgtatorTotalRotations,                                        SignalVariant(0,       32,  1,       0,          0,         6500000)                },
    { TotalECUDistance,                                             SignalVariant(1032,         32, 0.125,      0,          0,          526385151.9)    },
    { CalculusDayDistance,                                          SignalVariant(0,            16,  0.1,       0,          0,          6553.5)         },  // max shoule is 0xffff * 0.1
    { CalculusDayFuelConsumption,                                   SignalVariant(0,            32)                                                     },
    { AcceleratorChangeRate,                                        SignalVariant(0,            16, 1.0/256.0,  -100,       -100,       100)            },

    { StatisticsTripStartTime,                                      SignalVariant(0,            32)                                                     },
    { StatisticsTripEndTime,                                        SignalVariant(0,            32)                                                     },
    { StatisticsGnssSpeedDistance,                                  SignalVariant(0,            32)                                                     },
    { StatisticsTachographVehicleSpeedDistance,                     SignalVariant(0,            32)                                                     },
    { StatisticsVehicleSpeedLifeTotalDistance,                      SignalVariant(0,            32)                                                     },
    { StatisticsVehicleEstimatedLoad,                               SignalVariant(0,            16, 10,     0,           0,          642550)           },

    { StatisticsTripFuelUsed,                                       SignalVariant(0,            16, 0.1)                                                },
    { StatisticsTotalFuelUsed,                                      SignalVariant(0,            32, 0.001)                                              },
    { StatisticsVehicleTotalFuelUsed,                               SignalVariant(0,            32, 0.1)                                                },
    { StatisticsAverageFuelEconomy,                                 SignalVariant(0,            16, 0.1)                                                },
    { StatisticsSpeedingTimes,                                      SignalVariant(0,            16)                                                     },
    { StatisticsSpeedingDistance,                                   SignalVariant(0,            32)                                                     },
    { StatisticsSpeedingFuelUsed,                                   SignalVariant(0,            16, 100)                                                },
    { StatisticsBrakePedalBrakeTimes,                               SignalVariant(0,            16)                                                     },
    { StatisticsBrakePedalBrakeDistance,                            SignalVariant(0,            32)                                                     },
    { StatisticsBrakePedalBrakeDuration,                            SignalVariant(0,            32)                                                     },
    { StatisticsSharpShowDownTimes,                                 SignalVariant(0,            16)                                                     },
    { StatisticsSharpShowDownDuration,                              SignalVariant(0,            16, 100)                                                },
    { StatisticsSharpShowDownDistance,                              SignalVariant(0,            32)                                                     },
    { StatisticsSharpSpeedUpTimes,                                  SignalVariant(0,            16)                                                     },
    { StatisticsSharpSpeedUpDuration,                               SignalVariant(0,            16, 0.1)                                                },
    { StatisticsSharpSpeedUpDistance,                               SignalVariant(0,            32)                                                     },
    { StatisticsSharpSpeedUpFuelUsed,                               SignalVariant(0,            16, 0.1)                                                },
    { StatisticsSharpTrunningTimes,                                 SignalVariant(0,            16)                                                     },
    { StatisticsSharpTrunningDistance,                              SignalVariant(0,            32)                                                     },
    { StatisticsSleepyDriveTimes,                                   SignalVariant(0,            8,      1,      0,          0,          250)            },

    { TirePressureThresholdDetectionOriginal,                       SignalVariant(516146,        3)                                                 },
    { SMLK_AsrBrakeControl,                                         SignalVariant(0,             2)},
    { SMLK_AsrEngineControl,                                        SignalVariant(0,             2)},
    { SMLK_TrailerConnected,                                        SignalVariant(0,             2)},
    { SMLK_OpenStatusDoorDrv,                                       SignalVariant(228,             2,     1,     0,    0,     3)},
    { SMLK_OpenStatusDoorPas,                                       SignalVariant(229,             2,     1,     0,    0,     3)},
    { SMLK_CruiseControl,                                           SignalVariant(0,             2)},
    { SMLK_TrailerParkBrakeSwitch,                                  SignalVariant(0,             2)},
    { SMLK_ReverseGearSwitch,                                       SignalVariant(0,             2)},
    { SMLK_FuelLeakageAlert,                                        SignalVariant(0,             2,      1,     0,    0,    3)},

    //add 0200-E7 signal by wujian 20211227
    { EnginePowerInput1,                                        SignalVariant(0, 16, 0.05, 0, 0, 3212.75)},
    { keyswitchBatteryVoltage,                                  SignalVariant(0, 16, 0.05, 0, 0, 3212.75)},
    { Sensor1Humidity,                                          SignalVariant(0, 8,  0.4,  0, 0, 100)},
    { Sensor2Humidity,                                          SignalVariant(0, 8,  0.4,  0, 0, 100)},
    { Sensor3Humidity,                                          SignalVariant(0, 8,  0.4,  0, 0, 100)},
    { Sensor4Humidity,                                          SignalVariant(0, 8,  0.4,  0, 0, 100)},
    { Sensor5Humidity,                                          SignalVariant(0, 8,  0.4,  0, 0, 100)},
    { Sensor6Humidity,                                          SignalVariant(0, 8,  0.4,  0, 0, 100)},
    { Sensor7Humidity,                                          SignalVariant(0, 8,  0.4,  0, 0, 100)},
    { Sensor8Humidity,                                          SignalVariant(0, 8,  0.4,  0, 0, 100)},
    { Sensor1Temperature,                                       SignalVariant(0, 16, 0.1,  -100, -100, 200)},
    { Sensor2Temperature,                                       SignalVariant(0, 16, 0.1,  -100, -100, 200)},
    { Sensor3Temperature,                                       SignalVariant(0, 16, 0.1,  -100, -100, 200)},
    { Sensor4Temperature,                                       SignalVariant(0, 16, 0.1,  -100, -100, 200)},
    { Sensor5Temperature,                                       SignalVariant(0, 16, 0.1,  -100, -100, 200)},
    { Sensor6Temperature,                                       SignalVariant(0, 16, 0.1,  -100, -100, 200)},
    { Sensor7Temperature,                                       SignalVariant(0, 16, 0.1,  -100, -100, 200)},
    { Sensor8Temperature,                                       SignalVariant(0, 16, 0.1,  -100, -100, 200)},
    { PefrigerationSwitchStatus,                                SignalVariant(0, 8,  1,    0,   0, 3)},
    { PefrigerationWorkMode,                                    SignalVariant(0, 8,  1,    0,   0, 15)},
    { PefrigerationCpmpartmentTemperature,                      SignalVariant(0, 16, 0.1,  -50, -50, 6375.5)},
    { DefrostTemperature,                                       SignalVariant(0, 16, 0.1,  -50, -50, 6375.5)},
    { SupplyVoltage,                                            SignalVariant(0, 16, 0.1,  0,   0, 6425.5)},
    // update 0200-E7 2022-02-11
    { DcAcPowerConsumption,                                     SignalVariant(0, 16, 0.1, 0, 0, 6425.5)},
    { SteerWheelHeatPowerConsumption,                           SignalVariant(0, 8, 0.5,  0, 0, 125)},
    { CigarLighterPowerConsumption,                             SignalVariant(0, 8, 0.5,  0, 0, 125)},
    { Socket1PowerConsumption,                                  SignalVariant(0, 8, 0.5,  0, 0, 125)},
    { Socket2PowerConsumption,                                  SignalVariant(0, 8, 0.5,  0, 0, 125)},
    { LeftSeatHeatPowerConsumption,                             SignalVariant(0, 8, 0.5,  0, 0, 125)},
    { RightSeatHeatPowerConsumption,                            SignalVariant(0, 8, 0.5,  0, 0, 125)},
    { LowerBerthtHeatPowerConsumption,                          SignalVariant(0, 8, 0.5,  0, 0, 125)},
    { SleepReadLampPowerConsumption,                            SignalVariant(0, 8, 0.1,  0, 0, 25)},
    { ElevatedBoxPowerConsumption,                              SignalVariant(0, 8, 0.1,  0, 0, 25)},
    { HumidityInBox1,                                           SignalVariant(0, 8, 0.4,  0, 0, 100)},
    { HumidityInBox2,                                           SignalVariant(0, 8, 0.4,  0, 0, 100)},
    { TemperatureInBox1,                                        SignalVariant(0, 16, 0.1,  -50, -50, 6375.5)},
    { TemperatureInBox2,                                        SignalVariant(0, 16, 0.1,  -50, -100, 6375.5)},
    // add 0200-EC 2022-03-08
    { TboxVlotage,                                              SignalVariant(0, 16, 0.1, 0, 0, 65535)},
    { FuelSaveSwitch,                                           SignalVariant(0,  8, 1,  0, 0, 3)},
    { HydraulicMotorOilReturnTemperature,                       SignalVariant(0,  8, 1,  -40, -40, 210)},
    { EvVehicleStatus,                                          SignalVariant(0,  8, 1, 0, 0, 3)},
    { EvChargingStatus,                                         SignalVariant(0,  8, 1, 0, 0, 3)},
    { EvWorkingStatus,                                          SignalVariant(0,  8, 1, 0, 0, 5)},
    { EvVehicleSpeed,                                           SignalVariant(0, 16, 1.0/256.0, 0, 0, 250.996095)},
    { EvTotalVoltage,                                           SignalVariant(0, 16, 0.05,  0, 0, 3212.75)},
    { EvTotalCurrent,                                           SignalVariant(0, 16, 0.05,  -1600, -1600, 1612.75)},
    { EvBatterySoc,                                             SignalVariant(0,  8, 0.4,  0, 0, 100)},
    { EvDcDcWoringStatus,                                       SignalVariant(0,  8, 1,  0, 0, 3)},
    { EvGearStatus,                                             SignalVariant(0,  8, 1,  0, 0, 255)},
    { EvInsulationResistance,                                   SignalVariant(0,  16, 1,  0, 0, 64255)}
};

static const std::unordered_map<SignalID, SMLK_UINT32>  g_vehicle_signal_index = {
    { TachographVehicleSpeed,                                       VEHICLE_DATA_TACHOGRAPH_VEHICLE_SPEED                                               },
#ifdef VEHICLE_VKA
    { TachographVehicleSpeed_backup,                                VEHICLE_DATA_TACHOGRAPH_VEHICLE_SPEED_02                                             },
#endif
    { EngineTorqueMode,                                             VEHICLE_DATA_ENGINE_TORQUE_MODE                                                     },
    { DriverDemandEnginePercentTorque,                              VEHICLE_DATA_DRIVER_DEMAND_ENGINE_PERCENT_TORQUE                                    },
    { ActualEnginePercentTorque,                                    VEHICLE_DATA_ACTUAL_ENGINE_PERCENT_TORQUE                                           },
    { EngineSpeed,                                                  VEHICLE_DATA_ENGINE_SPEED                                                           },
    { EngineCoolantTemperature,                                     VEHICLE_DATA_ENGINE_COOLANT_TEMPERATURE                                             },
    { EngineOilPressure,                                            VEHICLE_DATA_ENGINE_OIL_PRESSURE                                                    },
    { EngineTotalFuelUsed,                                          VEHICLE_DATA_TOTAL_FUEL_USED                                                        },
    { AccPedalTravel,                                               VEHICLE_DATA_ACCELERATOR_PEDAL_POSITION                                             },
    { ParkingBrakeSwitch,                                           VEHICLE_DATA_PARKING_BRAKE_SWITCH                                                   },
    { WheelSpeed,                                                   VEHICLE_DATA_WHEELBASED_VEHICLE_SPEED                                               },
    { ClutchSwitch,                                                 VEHICLE_DATA_CLUTCH_SWITCH                                                          },
    { BrakeSwitch,                                                  VEHICLE_DATA_BRAKE_SWITCH                                                           },
    { CatalystTankLevel,                                            VEHICLE_DATA_CATALYST_TANK_LEVEL                                                    },
    { EngineTotalHoursOfOperation,                                  VEHICLE_DATA_ENGINE_TOTAL_HOURS_OF_OPERATION                                        },
    { EngineTotalRevolutions,                                       VEHICLE_DATA_TOTAL_ENGINE_REVOLUTIONS                                               },
    { EngineFuelRate,                                               VEHICLE_DATA_ENGINE_FUEL_RATE                                                       },
    { EngineInstantaneousFuelEconomy,                               VEHICLE_DATA_ENGINE_INSTANTANEOUS_FUEL_ECONOMY                                      },
    { EngineAverageFuelEconomy,                                     VEHICLE_DATA_ENGINE_AVERAGE_FUELECONOMY                                             },
    { AftertreatmentDieselParticulateFilterDifferentialPressure,    VEHICLE_DATA_AFTERTREATMENT1_DIESEL_PARTICULATE_FILTER_DIFFERENTIAL_PRESSURE        },
    { EngineIntakeManifoldPressure,                                 VEHICLE_DATA_ENGINE_INTAKE_MANIFOLD1_PRESSURE                                       },
    { EngineIntakeManifoldTemperature,                              VEHICLE_DATA_ENGINE_INTAKE_MANIFOLD1_TEMPERATURE                                    },
    { EngineAirIntakePressure,                                      VEHICLE_DATA_ENGINE_AIR_INLET_PRESSURE                                              },
    { BarometricPressure,                                           VEHICLE_DATA_BAROMETRIC_PRESSURE                                                    },
    { CabInteriorTemperature,                                       VEHICLE_DATA_CAB_TEMPERATURE                                                        },
    { AmbientAirTemperature,                                        VEHICLE_DATA_AMBIENT_AIR_TEMPERATURE                                                },
    { EngineAirIntakeTemperature,                                   VEHICLE_DATA_ENGINE_AIR_INLET_TEMPERATURE                                           },
    { EngineWaitToStartLamp,                                        VEHICLE_DATA_ENGINE_WAIT_TO_START_LAMP                                              },
    { WaterInFuelIndicator,                                         VEHICLE_DATA_WATER_IN_FUEL_INDICATOR                                                },
    { TransmissionSelectedGear,                                     VEHICLE_DATA_SELECTED_GEAR                                                          },
    { TransmissionActualGearRatio,                                  VEHICLE_DATA_TRANSMISSION_ACTUAL_GEAR_RATIO                                         },
    { TransmissionCurrentGear,                                      VEHICLE_DATA_CURRENT_GEAR                                                           },
    { FuelLevel,                                                    VEHICLE_DATA_FUEL_LEVEL                                                             },
    { TripDistance,                                                 VEHICLE_DATA_TRIP_DISTANCE                                                          },
    { TotalVehicleDistance,                                         VEHICLE_DATA_TOTAL_VEHICLE_DISTANCE                                                 },
    { CalculusDistance,                                             VEHICLE_DATA_STA_CALC_DISTANCE                                                      },
    { CalculusFuelConsumption,                                      VEHICLE_DATA_STA_CALC_TOTAL_FUEL                                                    },
    { CalculusDayDistance,                                          VEHILCE_DATA_STA_DAY_TRIP_DISTANCE                                                  },
    { CalculusDayFuelConsumption,                                   VEHILCE_DATA_STA_DAY_FUEL                                                           },
    { ActuralCatalystSpewed,                                        VEHICLE_DATA_ACTUAL_CATALYST_RATE                                                   },
    { TotalCatalystSpewed,                                          VEHICLE_DATA_TOTAL_CATALYST_RATE                                                    },
    { EngineExhaustGasTemperature,                                  VEHICLE_DATA_ENGINE_EXHAUST_GAS_TEMPERATURE                                         },
    { TBoxAcceleration,                                             VEHICLE_DATA_TBOX_ACC                                                               },
    { LateralAcceleration,                                          VEHICLE_DATA_LATERAL_ACCELERATION                                                   },
    { LongitudinalAcceleration,                                     VEHICLE_DATA_LONGITUDINAL_ACCELERATION                                              },
    { YawRate,                                                      VEHICLE_DATA_YAW_RATE                                                               },
    { EngineBrakingPercentTorque,                                   VEHICLE_DATA_ACTUAL_RETARDER_PERCENT_TORQUE1                                        },
    { EngineExhaustPercentTorque,                                   VEHICLE_DATA_ACTUAL_RETARDER_PERCENT_TORQUE2                                        },
    { RetarderSelection,                                            VEHICLE_DATA_RETAR_SELECTION_NONENGINE                                              },
    { ActualRetarderPercentTorque,                                  VEHICLE_DATA_ACTUAL_RETARDER_PERCENT_TORQUE3                                        },
    { RetarderTorqueMode,                                           VEHICLE_DATA_RETARDER_TORQUE_MODE                                                   },
    { DriverDemandRetarderPercentTorque,                            VEHICLE_DATA_DRIVERS_DEMAND_RETARDER_PERCENT_TORQUE                                 },
    { ActualMaximumAvailableRetarderPercentTorque,                  VEHICLE_DATA_ACTUAL_MAXIMUM_AVAILABLE_RETARDER_PERCENT_TORQUE                       },
    { SteeringWheelAngle,                                           VEHICLE_DATA_STEERING_WHEEL_ANGLE                                                   },
    { FrontAxleSpeed,                                               VEHICLE_DATA_FRONT_AXLE_SPEED                                                       },
    { FrontAxleLeftWheelSpeed,                                      VEHICLE_DATA_RELATIVE_SPEED_FRONT_AXLE_LEFT_WHEEL                                   },
    { FrontAxleRightWheelSpeed,                                     VEHICLE_DATA_RELATIVE_SPEED_FRONT_AXLE_RIGHT_WHEEL                                  },
    { RearAxle1LeftWheelSpeed,                                      VEHICLE_DATA_RELATIVE_SPEED_REAR_AXLE1_LEFT_WHEEL                                   },
    { RearAxle1RightWheelSpeed,                                     VEHICLE_DATA_RELATIVE_SPEED_REAR_AXLE1_RIGHT_WHEEL                                  },
    { BrakePedalTravel,                                             VEHICLE_DATA_BRAKE_PEDAL_POSITION                                                   },
    { TransmissionInputShaftSpeed,                                  VEHICLE_DATA_INPUT_SHAFT_SPEED                                                      },
    { TransmissionOutputShaftSpeed,                                 VEHICLE_DATA_OUTPUT_SHAFT_SPEED                                                     },
    { EngineLoadOnCurrentSpeed,                                     VEHICLE_DATA_ENGINE_PERCENT_LOAD_AT_CURRENT_SPEED                                   },
    { NominalFrictionPercentTorque,                                 VEHICLE_DATA_NOMINAL_FRICTION_PERCENT_TORQUE                                        },
    { PercentClutchSlip,                                            VEHICLE_DATA_PERCENT_CLUTCH_SLIP                                                    },
    { EngineIntakeAirMassFlowRat,                                   VEHICLE_DATA_ENGINE_INTAKE_AIR_MASS_FLOW_RATE                                       },
#if 0
    { OverrideControlMode,                                          VEHICLE_DATA_OVERRIDE_CONTROL_MODE                                                  },
    { RequestSpeedLimit,                                            VEHICLE_DATA_REQUESTED_SPEED_LIMIT                                                  },
    { RequestTorqueLimit,                                           VEHICLE_DATA_REQUESTED_TORQUE_LIMIT                                                 },
    { RequestTorqueHighResolution,                                  VEHICLE_DATA_REQUESTED_TORQUE_HIGH_RESOLUTION                                       },
#endif
    { PitchAngle,                                                   VEHICLE_DATA_PITCH_ANGLE                                                            },
    { CurrentSlope,                                                 VEHICLE_DATA_CURRENT_SLOPE                                                          },
    { NominalLevelRearAxle,                                         VEHICLE_DATA_NOMINAL_LEVEL_REAR_AXLE                                                },
    { NominalLevelFrontAxle,                                        VEHICLE_DATA_NOMINAL_LEVEL_FRONT_AXLE                                               },
    { AboveNominalLevelRearAxle,                                    VEHICLE_DATA_ABOVE_NOMINAL_LEVEL_REAR_AXLE                                          },
    { AboveNominalLevelFrontAxle,                                   VEHICLE_DATA_ABOVE_NOMINAL_LEVEL_FRONT_AXLE                                         },
    { BelowNominalLevelFrontAxle,                                   VEHICLE_DATA_BELOW_NOMINAL_LEVEL_FRONT_AXLE                                         },
    { LiftingControlModeFrontAxle,                                  VEHICLE_DATA_LIFTING_CONTROL_MODE_FRONT_AXLE                                        },
    { LoweringControlModeFrontAxle,                                 VEHICLE_DATA_LOWERING_CONTROL_MODE_FRONT_AXLE                                       },
    { LevelControlMode,                                             VEHICLE_DATA_LEVEL_CONTROL_MODE                                                     },
    { LiftAxle1Position,                                            VEHICLE_DATA_LIFT_AXLE1_POSITION                                                    },
    { BelowNominalLevelRearAxle,                                    VEHICLE_DATA_BELOW_NOMINAL_LEVEL_REAR_AXLE                                          },
    { LoweringControlModeRearAxle,                                  VEHICLE_DATA_LOWERING_CONTROL_MODE_REAR_AXLE                                        },
    { LiftingControlModeRearAxle,                                   VEHICLE_DATA_LIFTING_CONTROL_MODE_REAR_AXLE                                         },
    { LiftAxle2Position,                                            VEHICLE_DATA_LIFT_AXLE2_POSITION                                                    },
    { RearAxleInBumperRange,                                        VEHICLE_DATA_REAR_AXLE_INBUMPER_RANGE                                               },
    { FrontAxleInBumperRange,                                       VEHICLE_DATA_FRONT_AXLE_IN_BUMPER_RANGE                                             },
    { SuspensionRemoteControl1,                                     VEHICLE_DATA_SUSPENSION_REMOTE_CONTROL1                                             },
    { SuspensionControlRefusalInformation,                          VEHICLE_DATA_SUSPENSION_CONTROL_REFUSAL_INFORMATION                                 },
    { RelativeLevelFrontAxleLeft,                                   VEHICLE_DATA_RELATIVE_LEVEL_FRONT_AXLE_LEFT                                         },
    { RelativeLevelFrontAxleRight,                                  VEHICLE_DATA_RELATIVE_LEVEL_FRONT_AXLE_RIGHT                                        },
    { RelativeLevelRearAxleLeft,                                    VEHICLE_DATA_RELATIVE_LEVEL_REAR_AXLE_RIGHT                                         },
    { RelativeLevelRearAxleRight,                                   VEHICLE_DATA_RELATIVE_LEVEL_REAR_AXLE_LEFT                                          },
    { BellowPressureFrontAxleLeft,                                  VEHICLE_DATA_BELLOW_PRESSURE_FRONT_AXLE_LEFT                                        },
    { BellowPressureFrontAxleRight,                                 VEHICLE_DATA_BELLOW_PRESSURE_FRONT_AXLE_RIGHT                                       },
    { BellowPressureRearAxleLeft,                                   VEHICLE_DATA_BELLOW_PRESSURE_REAR_AXLE_LEFT                                         },
    { BellowPressureRearAxleRight,                                  VEHICLE_DATA_BELLOW_PRESSURE_REAR_AXLE_RIGHT                                        },
    { CatalystTankTemperature,                                      VEHICLE_DATA_CATALYST_TANK_TEMPERATURE                                              },
    { TurnSwitchStatus,                                             VEHICLE_DATA_TURN_SIGNAL_SWITCH                                                     },
    { TirePressure,                                                 VEHICLE_DATA_TIRE_PRESSURE                                                          },
    { TireTemperature,                                              VEHICLE_DATA_TIRE_TEMPERATURE                                                       },
    { TireStatus,                                                   VEHICLE_DATA_TIRE_STATUS                                                            },
    { TireExtrentPressureSupport,                                   VEHICLE_DATA_EXTENDED_TIRE_PRESSURE                                                 },
    { TireAirLeakageRate,                                           VEHICLE_DATA_TIRE_AIR_LEAKAGE_RATE                                                  },
    //{ TirePressureThresholdDetection,                               VEHICLE_DATA_TIRE_PRESSURE_THRESHOLD_DETECTION                                      },
    { TirePressureThresholdDetectionOriginal,                       VEHICLE_DATA_TIRE_PRESSURE_THRESHOLD_DETECTION                                      },
    { TirePressureExtendedRange,                                    VEHICLE_DATA_TIRE_PRESSURE_EXTENDED_RANGE                                           },
    { TirePressureRequired,                                         VEHICLE_DATA_REQUIRED_TIRE_PRESSURE                                                 },
    { AirConditionerOnOffStatus,                                    VEHICLE_DATA_AIR_CONDITIONER_ON_OFF_STATUS                                          },
    { AirConditionerModeStatus,                                     VEHICLE_DATA_AIR_CONDITIONER_MODE_STATUS                                            },
    { AirConditionerAutoModeStatus,                                 VEHICLE_DATA_AIR_CONDITIONER_AUTO_MODE_STATUS                                       },
    { AirConditionerBlowingRate,                                    VEHICLE_DATA_AIR_CONDITIONER_BLOWING_RATE                                           },
    { AirConditionerACModeStatus,                                   VEHICLE_DATA_AIR_CONDITIONER_AC_MODE_STATUS                                         },
    { AirConditionerCirculationModeStatus,                          VEHICLE_DATA_AIR_CONDITIONER_CIRCULATION_MODE_STATUS                                },
    { WarmAirBlowerOnOffStatus,                                     VEHICLE_DATA_WARM_AIR_BLOWER_ON_OFF_STATUS                                          },
    { WarmAirTemperatureStatus,                                     VEHICLE_DATA_WARM_AIR_TEMPERATURE                                                   },
    { ParkingAirConditionerOnOffStatus,                             VEHICLE_DATA_PARKING_AIR_CONDITIONER_ON_OFF_STATUS                                  },
    { ParkingAirConditionerBlowingRate,                             VEHICLE_DATA_PARKING_AIR_CONDITIONER_BLOWING_RATE                                   },
    { ACRegulationTemperature,                                      VEHICLE_DATA_AC_REGULATION_TEMPERATURE                                              },
    { ParkingACRegulationTemperature,                               VEHICLE_DATA_PARKING_AC_REGULATION_TEMPERATURE                                      },
    { CabTemperature,                                               VEHICLE_DATA_CAB_TEMPERATURE                                                        },
    { ACRegulationHRTemperature,                                    VEHICLE_DATA_AC_REGULATION_HR_TEMPERATURE                                           },
    { ParkingACRegulationHRTemperature,                             VEHICLE_DATA_PARKING_AC_REGULATION_HR_TEMPERATURE                                   },
    { AirConditionerVentilationStatus,                              VEHICLE_DATA_AIR_CONDITIONER_VENTILATION_STATUS                                     },
    { ParkingACAutoModeStatus,                                      VEHICLE_DATA_PARKING_AC_AUTO_MODE_STATUS                                            },
    { DieselParticulateFilterStatus,                                VEHICLE_DATA_DIESEL_PARTICULATE_FILTER_STATUS                                       },
    { DieselParticulateFilterLamp,                                  VEHICLE_DATA_DIESEL_PARTICULATE_FILTER_LAMP                                         },
    { DieselParticulateFilterActiveRegenerationStatus,              VEHICLE_DATA_DIESEL_PARTICULATE_FILTER_ACTIVE_REGENERATION_STATUS                   },
    { DieselParticulateFilterActiveRegenerationInhibitedStatus,     VEHICLE_DATA_DIESEL_PARTICULATE_FILTER_ACTIVE_REGENERATION_INHIBITED_STATUS         },
    { DriverInducementIndicatorStatus,                              VEHICLE_DATA_DRIVER_INDUCEMENT_INDICATOR_STATUS                                     },
    { MalfunctionIndicatorLampStatus,                               VEHICLE_DATA_MALFUNCTION_INDICATOR_LAMP_STATUS                                      },
    { AftertreatmentRegenerationInhibitSwitch,                      VEHICLE_DATA_AFTERTREATMENT_REGENERATION_INHIBIT_SWITCH                             },
    { TrailerWeight,                                                VEHICLE_DATA_GROSS_COMBINATION_VEHICLE_WEIGHT                                       },
    { BrightnensPercent,                                            VEHICLE_DATA_ILLUMINATION_BRIGHTNESS_PERCENT                                        },
    { AxleWeight,                                                   VEHICLE_DATA_AXLE_WEIGHT                                                            },
    { AftertreatmentIntakeNOx,                                      VEHICLE_DATA_AFTERTREATMENT1_INTAKE_NOX                                             },
    { AftertreatmentOutletNOx,                                      VEHICLE_DATA_AFTERTREATMENT1_OUTLET_NOX                                             },
    { AftertreatmentIntakeGasSensorTemperature,                     VEHICLE_DATA_AFTERTREATMENT1_SCR_INTAKE_TEMPERATURE                                 },
    { AftertreatmentOutletGasSensorTemperature,                     VEHICLE_DATA_AFTERTREATMENT1_SCR_OUTLET_TEMPERATURE                                 },
    //{ ATMTransmissionMode,                                          VEHICLE_DATA_TRANSMISSION_MODE                                                      },
    { ReferenceEngineTorque,                                        VEHICLE_DATA_REFERENCE_ENGINE_TORQUE                                                },
    { FuelTemperature,                                              VEHICLE_DATA_ENGINE_FUEL_TEMPERATURE                                                },
    { CruiseControlSetSpeed,                                        VEHICLE_DATA_CRUISE_CONTROL_SET_SPEED                                               },
#ifdef VEHICLE_VKA
    { TransmissionMode1,                                            VEHICLE_DATA_ETC7_EP_MODE                                                     },
#else
    { TransmissionMode1,                                            VEHICLE_DATA_TRANSMISSION_MODE1                                                     },
#endif
    { TransmissionMode2,                                            VEHICLE_DATA_TRANSMISSION_MODE2                                                     },
    { TransmissionMode3,                                            VEHICLE_DATA_TRANSMISSION_MODE3                                                     },
    { TransmissionMode4,                                            VEHICLE_DATA_TRANSMISSION_MODE4                                                     },

    { Battery1Temperature,                                          VEHICLE_DATA_BATTERY1_TEMPERATURE                                                   },
    { Battery1Voltage,                                              VEHICLE_DATA_BATTERY1_VOLTAGE                                                       },
    { Battery1SOC,                                                  VEHICLE_DATA_BATTERY1_SOC                                                           },
    { Battery1SOH,                                                  VEHICLE_DATA_BATTERY1_SOH                                                           },
    { Battery2Temperature,                                          VEHICLE_DATA_BATTERY2_TEMPERATURE                                                   },
    { Battery2Voltage,                                              VEHICLE_DATA_BATTERY2_VOLTAGE                                                       },
    { Battery2SOC,                                                  VEHICLE_DATA_BATTERY2_SOC                                                           },
    { Battery2SOH,                                                  VEHICLE_DATA_BATTERY2_SOH                                                           },
    { ActualMinimumSOC,                                             VEHICLE_DATA_ACTUAL_MINIMUM_SOC                                                     },
    { ActualMinimumSOH,                                             VEHICLE_DATA_ACTUAL_MINIMUM_SOH                                                     },
    { AgtatorState,                                                 VEHICLE_DATA_MIX_TANK_WORK_STATUS                                                   },
    { AgtatorSpeed,                                                 VEHICLE_DATA_MIX_TANK_ROTATION_SPEED                                                },
    { AgtatorTotalRotations,                                        VEHICLE_DATA_MIX_TANK_TOTAL_CYCLE_NUMBER                                            },

    { TotalECUDistance,                                             VEHICLE_DATA_TOTAL_ECU_DISTANCE                                                     },

    { AcceleratorChangeRate,                                        VEHICLE_DATA_STA_FUEL_CHANGE_RANGE                                                  },

    { StatisticsTripStartTime,                                      VEHICLE_DATA_STA_TRIP_START_TIME                                                    },
    { StatisticsTripEndTime,                                        VEHICLE_DATA_STA_TRIP_END_TIME                                                      },
    { StatisticsGnssSpeedDistance,                                  VEHICLE_DATA_STA_GPS_SPEED_TRIP_DISTANCE                                            },
    { StatisticsTachographVehicleSpeedDistance,                     VEHICLE_DATA_STA_CLUSTER_CALC_TRIP_DISTANCE                                         },
    { StatisticsVehicleSpeedLifeTotalDistance,                      VEHICLE_DATA_STA_SPEED_VALID_PERIOD_TRIP_DISTANCE                                   },
    { StatisticsVehicleEstimatedLoad,                               VEHICLE_DATA_STA_EST_WEIGHT                                       },
    { StatisticsTripFuelUsed,                                       VEHICLE_DATA_STA_CALC_TRIP_FUEL                                                     },
    //{ StatisticsTotalFuelUsed,                                      VEHICLE_DATA_STA_CALC_TOTAL_FUEL                                                    },
    { StatisticsAverageFuelEconomy,                                 VEHICLE_DATA_STA_AVE_FUEL                                                           },
    { StatisticsSpeedingTimes,                                      VEHICLE_DATA_STA_OVERSPEED_CNT                                                      },
    { StatisticsSpeedingDistance,                                   VEHICLE_DATA_STA_OVERSPEED_DISTANCE                                                 },
    { StatisticsSpeedingFuelUsed,                                   VEHICLE_DATA_STA_OVERSPEED_FUEL                                                     },
    { StatisticsBrakePedalBrakeTimes,                               VEHICLE_DATA_STA_BRAKE_CNT                                                          },
    { StatisticsBrakePedalBrakeDistance,                            VEHICLE_DATA_STA_BREAK_DISTANCE                                                     },
    { StatisticsBrakePedalBrakeDuration,                            VEHICLE_DATA_STA_BREAK_TIME                                                         },
    { StatisticsSharpShowDownTimes,                                 VEHICLE_DATA_STA_SLOWDOWN_CNT                                                       },
    { StatisticsSharpShowDownDuration,                              VEHICLE_DATA_STA_SLOWDOWN_TIME                                                      },
    { StatisticsSharpShowDownDistance,                              VEHICLE_DATA_STA_SLOWDOWN_DISTANCE                                                  },
    { StatisticsSharpSpeedUpTimes,                                  VEHICLE_DATA_STA_SPEED_UP_CNT                                                       },
    { StatisticsSharpSpeedUpDuration,                               VEHICLE_DATA_STA_SPEED_UP_TIME                                                      },
    { StatisticsSharpSpeedUpDistance,                               VEHICLE_DATA_STA_SPEED_UP_DISTANCE                                                  },
    { StatisticsSharpSpeedUpFuelUsed,                               VEHICLE_DATA_STA_SPEED_UP_FUEL                                                      },
    { StatisticsSharpTrunningTimes,                                 VEHICLE_DATA_STA_TURNING_CNT                                                        },
    { StatisticsSharpTrunningDistance,                              VEHICLE_DATA_STA_TURNING_DISTANCE                                                   },
    { StatisticsSleepyDriveTimes,                                   VEHICLE_DATA_STA_FATIGUE_DRIVE_CNT                                                  },

#if 0
    { EmergencyBrakingSystemState,                                  },
    { EmergencyBrakingTargetID,                                     },
    { EmergencyTargetRelativeSpeed,                                 },
    { EmergencyTargetSpeed,                                         },
    { EmergencyTargetDistance,                                      },
    { AlarmFlags,                                                   },
    { ExtendedSignalStatus,                                         },
    { RSSI,                                                         },
    { ACCStatus,                                                    },
#endif
    { SMLK_AsrBrakeControl,                                         VEHICLE_DATA_ASR_BRAKE_CONTROL_ACTIVE     },
    { SMLK_AsrEngineControl,                                        VEHICLE_DATA_ASR_ENGINE_CONTROL_ACTIVE    },

#ifdef FAW_MICRO_TRUCK
    { SMLK_TrailerConnected,                                        VEHICLE_DATA_AUXILIARY_IO_5_TRAILER_CONNECTED              },
#else
    { SMLK_TrailerConnected,                                        VEHICLE_DATA_TRAILER_CONNECTED            },
#endif
    { SMLK_OpenStatusDoorDrv,                                       VEHICLE_DATA_OPEN_STATUS_OF_DOOR_DRIVER   },
    { SMLK_OpenStatusDoorPas,                                       VEHICLE_DATA_OPEN_STATUS_OF_DOOR_PASSENGER},
    { SMLK_CruiseControl,                                           VEHICLE_DATA_CRUISE_CONTROL_ACTIVE        },
    { SMLK_TrailerParkBrakeSwitch,                                  VEHICLE_DATA_TRAILER_PARKING_BRAKE_SWITCH },
    { SMLK_FuelLeakageAlert,                                        VEHICLE_DATA_FUEL_LEAKAGE_ALERT           },

#ifdef FAW_MICRO_TRUCK
    { SMLK_ReverseGearSwitch,                                        VEHICLE_DATA_TRANSMISSION_REVERSE_DIRECTION_SWICH           },
#else
    { SMLK_ReverseGearSwitch,                                       VEHICLE_DATA_REVERSE_GEAR_SWITCH          },
#endif

    //add 0200-E7 signal by wujian 20211227
    { EnginePowerInput1,                                        VEHICLE_DATA_BATTERY_POTENTIAL_POWER_INPUT1   },
    { keyswitchBatteryVoltage,                                  VEHICLE_DATA_KEYSWITCH_BATTERY_POTENTIAL      },
    { Sensor1Humidity,                                          VEHICLE_DATA_SENSOR1_HUMIDITY           },
    { Sensor2Humidity,                                          VEHICLE_DATA_SENSOR2_HUMIDITY           },
    { Sensor3Humidity,                                          VEHICLE_DATA_SENSOR3_HUMIDITY           },
    { Sensor4Humidity,                                          VEHICLE_DATA_SENSOR4_HUMIDITY           },
    { Sensor5Humidity,                                          VEHICLE_DATA_SENSOR5_HUMIDITY           },
    { Sensor6Humidity,                                          VEHICLE_DATA_SENSOR6_HUMIDITY           },
    { Sensor7Humidity,                                          VEHICLE_DATA_SENSOR7_HUMIDITY           },
    { Sensor8Humidity,                                          VEHICLE_DATA_SENSOR8_HUMIDITY           },
    { Sensor1Temperature,                                       VEHICLE_DATA_SENSOR1_TEMPERATURE           },
    { Sensor2Temperature,                                       VEHICLE_DATA_SENSOR2_TEMPERATURE           },
    { Sensor3Temperature,                                       VEHICLE_DATA_SENSOR3_TEMPERATURE           },
    { Sensor4Temperature,                                       VEHICLE_DATA_SENSOR4_TEMPERATURE           },
    { Sensor5Temperature,                                       VEHICLE_DATA_SENSOR5_TEMPERATURE           },
    { Sensor6Temperature,                                       VEHICLE_DATA_SENSOR6_TEMPERATURE           },
    { Sensor7Temperature,                                       VEHICLE_DATA_SENSOR7_TEMPERATURE           },
    { Sensor8Temperature,                                       VEHICLE_DATA_SENSOR8_TEMPERATURE           },
    { PefrigerationSwitchStatus,                                VEHICLE_DATA_REFRIGERATION_SWITCH_STATUS   },
    { PefrigerationWorkMode,                                    VEHICLE_DATA_REFRIGERATION_WORK_MODE       },
    { PefrigerationCpmpartmentTemperature,                      VEHICLE_DATA_REFRIGERATION_COMPARTMENT_TEMPERATURE           },
    { DefrostTemperature,                                       VEHICLE_DATA_DEFROST_TEMPERATURE           },
    { SupplyVoltage,                                            VEHICLE_DATA_SUPPLY_VOLTAGE                },
    // update 0200-E7 2022-02-11
    { DcAcPowerConsumption,                                     VEHICLE_DCAC_POWERCONSUMPTION              },
    { SteerWheelHeatPowerConsumption,                           VEHICLE_STEERINGWHEEL_HEATING_POWERCONSUMPTION     },
    { CigarLighterPowerConsumption,                             VEHICLE_CIGAR_LIGHTER_POWERCONSUMPTION             },
    { Socket1PowerConsumption,                                  VEHICLE_24V_SOCKET_1_POWERCONSUMPTION              },
    { Socket2PowerConsumption,                                  VEHICLE_24V_SOCKET_2_POWERCONSUMPTION              },
    { LeftSeatHeatPowerConsumption,                             VEHICLE_L_SEAT_HEAT_POWERCONSUMPTION               },
    { RightSeatHeatPowerConsumption,                            VEHICLE_R_SEAT_HEAT_POWERCONSUMPTION               },
    { LowerBerthtHeatPowerConsumption,                          VEHICLE_LOWER_BERTHT_HEAT_POWERCONSUMPTION         },
    { SleepReadLampPowerConsumption,                            VEHICLE_SLEEPREAD_LAMP_POWERCONSUMPTION            },
    { ElevatedBoxPowerConsumption,                              VEHICLE_ELEVATED_BOX_POWERCONSUMPTION              },
    { HumidityInBox1,                                           VEHICLE_HUMIDITY_INSIDE_BOX_1                      },
    { HumidityInBox2,                                           VEHICLE_HUMIDITY_INSIDE_BOX_2                      },
    { TemperatureInBox1,                                        VEHICLE_TEMPERATURE_INSIDE_BOX_1                   },
    { TemperatureInBox2,                                        VEHICLE_TEMPERATURE_INSIDE_BOX_2                   },
    // add 0200-EC 2022-03-08
    { FuelSaveSwitch,                                           VEHICLE_DATA_FUEL_SAVING_SWITCH_GEAR               },
    { HydraulicMotorOilReturnTemperature,                       VEHICLE_HYDRAULIC_MOTOR_OIL_RETURN_TEMPERATURE     },
    { EvVehicleStatus,                                          VEHICLE_DATA_EV_VEHICLE_STATUS                     },
    { EvChargingStatus,                                         VEHICLE_DATA_EV_POWER_BATTERY_CHARGING_STATUS      },
    { EvVehicleSpeed,                                           VEHICLE_DATA_EV_VEHICLE_SPEED                      },
    { EvTotalVoltage,                                           VEHICLE_DATA_EV_POWER_BATTERY_VOLTAGE              },
    { EvTotalCurrent,                                           VEHICLE_DATA_EV_POWER_BATTERY_CURRENT              },
    { EvBatterySoc,                                             VEHICLE_DATA_EV_POWER_BATTERY_SOC                  },
    { EvDcDcWoringStatus,                                       VEHICLE_DATA_EV_DCDC_WORKING_STATUS                },
    { EvInsulationResistance,                                   VEHICLE_DATA_EV_INSULATION_RESISTANCE              }
};

}   // namespace vehicle
}   // namespace smartlink

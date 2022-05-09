#pragma once
#include <tuple>
#include <unordered_map>
#include <smlk_types.h>
#include <smlk_signal.h>
#include <vehicle_data_index_def.h>

namespace smartlink
{
    namespace vehicle
    {
        enum SignalID : SMLK_UINT32
        {
            StartHere,

            TachographVehicleSpeed,
            BarometricPressure,
            ActualEnginePercentTorque,
            NominalFrictionPercentTorque,
            EngineSpeed,
            EngineFuelRate,
            Aftertreatment1IntakeNOx,
            Aftertreatment1OutletNOx,
            CatalystTankLevel,
            EngineInletAirMassFlowRate,
            Aftertreatment1SCRIntakeTemperature,
            Aftertreatment1SCROutletTemperature,
            Aftertreatment1DPFDifferentialPressure,
            EngineCoolantTemperature,
            FuelLevel,
            TotalVehicleDistance,

            LocatedStatus,
            Longitude,
            Latitude,
            TimeStamp,
        
            TWCUpstreamO2Sensor,
            TWCDownstreamO2Sensor,
            TWCDownstreamNOxSensor,
            TWCTempSensor,

            MotorSpeed,
            DriveInsPowerPer,
            PowerBatVolt,
            PowerBatCurr,
            PowerBatSOC,
        };

        static const std::unordered_map <SignalID, SMLK_UINT32> g_vehicle_signal_index = {
            {TachographVehicleSpeed,                                     VEHICLE_DATA_TACHOGRAPH_VEHICLE_SPEED                  },
            {BarometricPressure,                                         VEHICLE_DATA_BAROMETRIC_PRESSURE                       },
            {ActualEnginePercentTorque,                                  VEHICLE_DATA_ACTUAL_ENGINE_PERCENT_TORQUE              },
            {NominalFrictionPercentTorque,                               VEHICLE_DATA_NOMINAL_FRICTION_PERCENT_TORQUE           },
            {EngineSpeed,                                                VEHICLE_DATA_ENGINE_SPEED                              },
            {EngineFuelRate,                                             VEHICLE_DATA_ENGINE_FUEL_RATE                          },
            {Aftertreatment1IntakeNOx,                                   VEHICLE_DATA_AFTERTREATMENT1_INTAKE_NOX                },
            {Aftertreatment1OutletNOx,                                   VEHICLE_DATA_AFTERTREATMENT1_OUTLET_NOX                },
            {CatalystTankLevel,                                          VEHICLE_DATA_CATALYST_TANK_LEVEL                       },
            {EngineInletAirMassFlowRate,                                 VEHICLE_DATA_ENGINE_INTAKE_AIR_MASS_FLOW_RATE          },
            {Aftertreatment1SCRIntakeTemperature,                        VEHICLE_DATA_AFTERTREATMENT1_SCR_INTAKE_TEMPERATURE    },
            {Aftertreatment1SCROutletTemperature,                        VEHICLE_DATA_AFTERTREATMENT1_SCR_OUTLET_TEMPERATURE    },
            {Aftertreatment1DPFDifferentialPressure,                     VEHICLE_DATA_AFTERTREATMENT1_DIESEL_PARTICULATE_FILTER_DIFFERENTIAL_PRESSURE    },
            {EngineCoolantTemperature,                                   VEHICLE_DATA_ENGINE_COOLANT_TEMPERATURE                },
            {FuelLevel,                                                  VEHICLE_DATA_FUEL_LEVEL                                },
            {TotalVehicleDistance,                                       VEHICLE_DATA_TOTAL_VEHICLE_DISTANCE                    },     
        
            {TWCUpstreamO2Sensor,                                        VEHICLE_DATA_THREE_WAY_CATALYST_UPSTREAM_OXYGEN_SENSOR },
            {TWCDownstreamO2Sensor,                                      VEHICLE_DATA_THREE_WAY_CATALYST_DOWNSTREAM_OXYGEN_SENSOR   },
            {TWCTempSensor,                                              VEHICLE_DATA_THREE_WAY_CATALYST_TEMPERATURE_SENSOR     },
        
            {MotorSpeed,                                                 VEHICLE_DATA_EV_ELEC_ENGINE_SPEED                      },
            {DriveInsPowerPer,                                           VEHICLE_DATA_EV_DRIVE_INSTANTANEOUS_POWER_PERCENTAGE   },
            {PowerBatVolt,                                               VEHICLE_DATA_EV_POWER_BATTERY_VOLTAGE                  },
            {PowerBatCurr,                                               VEHICLE_DATA_EV_POWER_BATTERY_CURRENT                  },
            {PowerBatSOC,                                                VEHICLE_DATA_EV_POWER_BATTERY_SOC                      },   
        };
    };


    namespace signal
    {
        static const vehicle::SignalVariant TachographVehicleSpeed(1624, 16, 1.0 / 256.0, 0.0, 0, 250.996094);                       // km/h
        static const vehicle::SignalVariant BarometricPressure(108, 8, 0.5, 0, 0, 125);                                              // kPa
        static const vehicle::SignalVariant ActualEnginePercentTorque(513, 8, 1, -125, -125, 125);                                   // %
        static const vehicle::SignalVariant NominalFrictionPercentTorque(514, 8, 1, -125, -125, 125);                                // %
        static const vehicle::SignalVariant EngineSpeed(190, 16, 0.125, 0, 0, 8031.875);                                             // rpm
        static const vehicle::SignalVariant EngineFuelRate(183, 16, 0.05, 0, 0, 3212.75);                                            // L/h
        static const vehicle::SignalVariant Aftertreatment1IntakeNOx(3216, 16, 0.05, -200, -200, 3012.75);                           // ppm
        static const vehicle::SignalVariant Aftertreatment1OutletNOx(3226, 16, 0.05, -200, -200, 3012.75);                           // ppm
        static const vehicle::SignalVariant CatalystTankLevel(1761, 8, 0.4, 0, 0, 100);                                              // %
        static const vehicle::SignalVariant EngineInletAirMassFlowRate(132, 16, 0.05, 0, 0, 3212.75);                                // kg/h
        static const vehicle::SignalVariant Aftertreatment1SCRIntakeTemperature(4360, 16, 0.03125, -273, -273, 1734.96875);          // deg C
        static const vehicle::SignalVariant Aftertreatment1SCROutletTemperature(4363, 16, 0.03125, -273, -273, 1734.96875);          // deg C
        static const vehicle::SignalVariant Aftertreatment1DPFDifferentialPressure(3251, 16, 0.1, 0, 0, 6425.5); // kPa
        static const vehicle::SignalVariant EngineCoolantTemperature(110, 8, 1, -40, -40, 210);                                      // deg C
        static const vehicle::SignalVariant FuelLevel(96, 8, 0.4, 0, 0, 100);                                                        // %
        // please note that in J1939, this signal unit is 0.125 km/bit, but in our document, the unit is 0.1, please make sure be careful to handle this signal
        static const vehicle::SignalVariant TotalVehicleDistance(245, 32, 0.1);                                                      // km

        static const vehicle::SignalVariant LocatedStatus(516133, 3);
        static const vehicle::SignalVariant Longitude(516131, 32, 0.000001, 0, 0, 180);
        static const vehicle::SignalVariant Latitude(516130, 32, 0.000001, 0, 0, 90);
        static const vehicle::SignalVariant TimeStamp(516129, 32);

        static const vehicle::SignalVariant TWCUpstreamO2Sensor(505, 16, 0.0000305, 0, 0, 1.999);
        static const vehicle::SignalVariant TWCDownstreamO2Sensor(506, 8, 0.01, 0, 0, 2.550);
        static const vehicle::SignalVariant TWCDownstreamNOxSensor(507, 16, 0.05, -200, -200, 3012.75);
        static const vehicle::SignalVariant TWCTempSensor(507, 16, 0.03125, -273, -273, 1734.96875);

        static const vehicle::SignalVariant MotorSpeed(331, 16, 1.0, 0, 0, 65535);
        static const vehicle::SignalVariant DriveInsPowerPer(418, 8, 1.0, 0, 0, 100);
        static const vehicle::SignalVariant PowerBatVolt(312, 16, 0.1, 0, 0, 6553.4);
        static const vehicle::SignalVariant PowerBatCurr(313, 16, 0.1, -1000, -1000, 5553.4);
        static const vehicle::SignalVariant PowerBatSOC(314, 8, 1.0, 0, 0, 100);

    }; // namespace signal

}; // namespace smartlink

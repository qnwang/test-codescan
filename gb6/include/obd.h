#pragma once
#include <cstdint>

namespace smartlink {
namespace obd {

namespace sid {
    static const std::uint8_t   ReadProtocolIdentification  = 0x12;
    static const std::uint8_t   ClearDiagnosticInformation  = 0x14;
    static const std::uint8_t   ReadDTCInformation          = 0x19;
    static const std::uint8_t   ReadDataByIdentifier        = 0x22;
    static const std::uint8_t   RoutineControl              = 0x31;

    static const std::uint8_t   ReadProtocolIdentificationRsp  = 0x12;
    static const std::uint8_t   ClearDiagnosticInformationRsp  = 0x54;
    static const std::uint8_t   ReadDTCInformationRsp          = 0x59;
    static const std::uint8_t   ReadDataByIdentifierRsp        = 0x62;
    static const std::uint8_t   RoutineControlRsp              = 0x71;
};  // namespace sid


namespace negative {
    static const std::uint8_t   IMLOIF                      = 0x13; // incorrect message length or invalid format
    static const std::uint8_t   CNC                         = 0x22; // conditions not correct
    static const std::uint8_t   ROOR                        = 0x31; // request out of range
    static const std::uint8_t   SAD                         = 0x33; // security access denied
};  // namespace negative

namespace sfid {
    static const std::uint8_t   ReportDTCSnapshotRecordByDTCNumber      = 0x04;
    static const std::uint8_t   ReportDTCExtendedDataRecordByDTCNumber  = 0x06;
    static const std::uint8_t   ReportWWHOBDDTCByMaskRecord             = 0x42;
};  // sub-function id

namespace dtc {

namespace FGID {
    static const std::uint8_t   emissions                   = 0x33;
    static const std::uint8_t   safety                      = 0xD0;
    static const std::uint8_t   vobd                        = 0xFE;
    static const std::uint8_t   all                         = 0xFF;
};  // namespace FGID

namespace status {
    static const std::uint8_t   TF                          = 0x01;     // test failed
    static const std::uint8_t   TFTOC                       = 0x02;     // test failed this operation cycle
    static const std::uint8_t   PDTC                        = 0x04;     // pending DTC
    static const std::uint8_t   CDTC                        = 0x08;     // confirmed DTC
    static const std::uint8_t   TNCSLC                      = 0x10;     // test not completed since last clear
    static const std::uint8_t   TFSLC                       = 0x20;     // test failed since last clear
    static const std::uint8_t   TNCTOC                      = 0x40;     // test not completed this operation cycle
    static const std::uint8_t   WIR                         = 0x80;     // warning indicator requested
};  // namespace status

};  // namespace dtc

namespace protocol {
    static const std::uint8_t   ISO15765                    = 0x00;     // ISO 15765
    static const std::uint8_t   ISO27145                    = 0x01;     // ISO 27145
    static const std::uint8_t   SAEJ1939                    = 0x02;     // SAE J1939
    static const std::uint8_t   INVALID                     = 0xFE;     // invalid
};  // namespace protocol

namespace did {
    static const std::uint16_t  MIL                         = 0x01F4;
    static const std::uint16_t  VIN                         = 0x02F8;
    static const std::uint16_t  SCIN                        = 0x04F8;
    static const std::uint16_t  CVN                         = 0x06F8;
    static const std::uint16_t  IUPR_D                      = 0x0BF8;
    static const std::uint16_t  IUPR_G                      = 0x08F8;
    static const std::uint16_t  Protocol                    = 0x10F8;
};  // namespace did

namespace can {
namespace identifier {

namespace diesel {
    static const std::uint32_t  Functional                  = 0x18DB33F1;
    static const std::uint32_t  Request                     = 0x18DA00F1;
    static const std::uint32_t  Response                    = 0x18DAF100;
};  // namespace fuel

namespace gas {
    static const std::uint32_t  Functional                  = 0x18DB33F1;
    static const std::uint32_t  Request                     = 0x18DA00F1;
    static const std::uint32_t  Response                    = 0x18DAF100;
};  // namespace gas

};  // namespace identifier
};  // namespace can

namespace message {

    struct diagnostic {
        struct {
            std::uint8_t    reserved                                    :7;
            std::uint8_t    malfunction_indicator_lamp_status           :1;
        }               A;
        struct {
            std::uint8_t    misfire_monitoring_supported                :1;
            std::uint8_t    fuel_system_monitoring_supported            :1;
            std::uint8_t    comprehensive_component_monitoring_supported:1;
            std::uint8_t                                                :1;
            std::uint8_t    misfire_monitoring_ready                    :1;
            std::uint8_t    fuel_system_monitoring_ready                :1;
            std::uint8_t    comprehensive_component_monitoring_ready    :1;
            std::uint8_t                                                :1;
        }               B;
        struct {
            std::uint8_t    catalyst_monitoring_nmhc_catalyst_monitoring_supported                      :1;  // catalyst_monitoring_supported & nmhc_catalyst_monitoring_supported
            std::uint8_t    heated_catalyst_monitoring_nox_aftertreatment_monitorin_supported           :1;  // heated_catalyst_monitoring_supported & nox_aftertreatment_monitoring_supported
            std::uint8_t    evaporative_system_monitoring_supported                                     :1;  // 
            std::uint8_t    secondary_air_system_monitoring_boost_pressure_system_monitoring_supported  :1;  // secondary_air_system_monitoring_supported & boost_pressure_system_monitoring_supported
            std::uint8_t                                                                                :1;
            std::uint8_t    exhaust_gas_sensor_monitoring_supported                                     :1;  // 
            std::uint8_t    oxygen_sensor_heater_monitoring_pm_filter_monitoring_supported              :1;  // oxygen_sensor_heater_monitoring_supported & pm_filter_monitoring_supported
            std::uint8_t    egr_vvt_system_monitoring_supported                                         :1;  // 
        }               C;
        struct {
            std::uint8_t    catalyst_monitoring_nmhc_catalyst_monitoring_ready                          :1;  // catalyst_monitoring_ready & nmhc_catalyst_monitoring_ready
            std::uint8_t    heated_catalyst_monitoring_nox_aftertreatment_monitoring_ready              :1;  // heated_catalyst_monitoring_ready & nox_aftertreatment_monitoring_ready
            std::uint8_t    evaporative_system_monitoring_ready                                         :1;  // 
            std::uint8_t    secondary_air_system_monitoring_boost_pressure_system_monitoring_ready      :1;  // secondary_air_system_monitoring_ready & boost_pressure_system_monitoring_ready
            std::uint8_t                                                                                :1;
            std::uint8_t    exhaust_gas_sensor_monitoring_ready                                         :1;  // 
            std::uint8_t    oxygen_sensor_heater_monitoring_pm_filter_monitoring_ready                  :1;  // oxygen_sensor_heater_monitoring_ready & pm_filter_monitoring_ready
            std::uint8_t    egr_vvt_system_monitoring_ready                                             :1;  //
        }               D;
    };
};  // namespace message

};  // namespace obd
};  // namespace smartlink
//
// Created by t on 4/6/22.
//

#include <iostream>
#include <sstream>
#include <ctime>
#include <cstring>
#include <iomanip>
#include <memory>
#include <vector>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
//#include <message_17691.h>

#include "session.h"

static std::string BCD2TimeStamp(const BCD data[], std::size_t sz) {
    if ((nullptr == data) || (M_GB_17691_SZ_TIMESTAMP != sz)) {
        return "";
    }

    std::tm tm;

    std::memset(&tm, 0x00, sizeof(tm));

    tm.tm_year = (int) ((data[0] & 0xF0) >> 4) * 10 + (int) (data[0] & 0x0F) + 2000;
    tm.tm_mon = (int) ((data[1] & 0xF0) >> 4) * 10 + (int) (data[1] & 0x0F) - 1;
    tm.tm_mday = (int) ((data[2] & 0xF0) >> 4) * 10 + (int) (data[2] & 0x0F);
    tm.tm_hour = (int) ((data[3] & 0xF0) >> 4) * 10 + (int) (data[3] & 0x0F);
    tm.tm_min = (int) ((data[4] & 0xF0) >> 4) * 10 + (int) (data[4] & 0x0F);
    tm.tm_sec = (int) ((data[5] & 0xF0) >> 4) * 10 + (int) (data[5] & 0x0F);

    std::stringstream ss;

    ss << std::setw(4) << std::setfill('0') << std::dec << tm.tm_year;
    ss << "-";
    ss << std::setw(2) << std::setfill('0') << std::dec << tm.tm_mon;
    ss << "-";
    ss << std::setw(2) << std::setfill('0') << std::dec << tm.tm_mday;
    ss << " ";
    ss << std::setw(2) << std::setfill('0') << std::dec << tm.tm_hour;
    ss << ":";
    ss << std::setw(2) << std::setfill('0') << std::dec << tm.tm_min;
    ss << ":";
    ss << std::setw(2) << std::setfill('0') << std::dec << tm.tm_sec;

    return ss.str();
}

Session::Session(int fd) : m_sfd(fd), m_tail(0) {}

Session::~Session() {
    close(m_sfd);
}

void Session::OnRecv(const BYTE data[], std::size_t sz) {
    std::cout << "OnRecv" << std::endl;
    if (nullptr == data) {
        std::cerr << "data is nullptr" << std::endl;
        return;
    }

    if (sz <= 0) {
        return;
    }

    const BYTE *ptr = data;

    while (sz > 0) {
        if (m_tail < sizeof(GB_17691_MsgHead)) {
            if (m_tail + sz < sizeof(GB_17691_MsgHead)) {
                std::memcpy(&m_recv_buffer[m_tail], ptr, sz);
                m_tail += sz;
                return;
            }

            auto cnt = sizeof(GB_17691_MsgHead) - m_tail;

            std::memcpy(&m_recv_buffer[m_tail], ptr, cnt);
            ptr += cnt;
            sz -= cnt;
            m_tail += cnt;

            if ((M_GB_17691_MSG_FLAG != m_recv_buffer[0]) || (M_GB_17691_MSG_FLAG != m_recv_buffer[1])) {
                std::cerr << "unexpected message flag" << std::endl;
                return;
            }
        }

        auto const pHead = reinterpret_cast<const GB_17691_MsgHead *>(m_recv_buffer);
        auto const length = be16toh(pHead->length);
        auto const total = sizeof(GB_17691_MsgHead) + length + 1;

        if (m_tail + sz < total) {
            std::memcpy(&m_recv_buffer[m_tail], ptr, sz);
            m_tail += sz;
            return;
        }

        auto n = total - m_tail;
        std::memcpy(&m_recv_buffer[m_tail], ptr, n);
        ptr += n;
        sz -= n;

        std::vector<BYTE> data(m_recv_buffer, m_recv_buffer + total);
        m_tail = 0;

        OnMessage(data);
    }
}

void Session::OnMessage(const std::vector<BYTE> &data) {
    if (data.size() < sizeof(GB_17691_MsgHead)) {
        std::cerr << "unexpected message body size, at least sizeof(GB_17691_MsgHead)" << std::endl;
        return;
    }

    auto const *pHead = reinterpret_cast<const GB_17691_MsgHead *>(data.data());

    GB_17691_MsgHead head = *pHead;

    head.length = be16toh(pHead->length);

    if (data.size() != sizeof(GB_17691_MsgHead) + head.length + 1) {
        std::cerr << "unexpected message body size, expected: " << sizeof(GB_17691_MsgHead) + head.length + 1
                  << ", but data.size(): " << data.size() << std::endl;
        return;
    }

    BYTE XOR = 0x00;

    for (decltype(data.size()) index = sizeof(head.flags); index < data.size(); index++) {
        XOR ^= data[index];
    }

    if (XOR != 0) {
        std::cerr << "XOR verification failed." << std::endl;
        return;
    }

    switch (head.cmd) {
        case M_GB_17691_CMD_VEHICLE_LOGIN:
            OnVehicleLogin(head, &data.data()[sizeof(GB_17691_MsgHead)], data.size() - sizeof(GB_17691_MsgHead) - 1);
            break;
        case M_GB_17691_CMD_REALTIME_REPORT:
            OnReport(head, &data.data()[sizeof(GB_17691_MsgHead)], data.size() - sizeof(GB_17691_MsgHead) - 1);
            break;
        case M_GB_17691_CMD_REISSUE_REPORT:
            OnReport(head, &data.data()[sizeof(GB_17691_MsgHead)], data.size() - sizeof(GB_17691_MsgHead) - 1);
            break;
        case M_GB_17691_CMD_VEHICLE_LOGOUT:
            OnVehicleLogout(head, &data.data()[sizeof(GB_17691_MsgHead)], data.size() - sizeof(GB_17691_MsgHead) - 1);
            break;
        case M_GB_17691_CMD_TERMIAL_TIMING:
            OnSyncTime(head, &data.data()[sizeof(GB_17691_MsgHead)], data.size() - sizeof(GB_17691_MsgHead) - 1);
            break;
        default:
            std::cerr << "unsupported message id:   0x" << std::uppercase << std::setw(2) << std::setfill('0')
                      << std::hex << (int) head.cmd << std::endl;
            break;
    }
}

void Session::OnVehicleLogin(const GB_17691_MsgHead &head, const BYTE data[], std::size_t sz) {
    if (sz != sizeof(GB_17691_VehicleLogin)) {
        std::cerr << "unexpected message body size: " << sz << std::endl;
        return;
    }

//    auto pLogin = reinterpret_cast<const GB_17691_VehicleLogin *>(data);
//
//    std::cout << "OnVehicleLogin:" << std::endl;
//    std::cout << "\thead.flags:         0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)head.flags[0] << " 0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)head.flags[1] << std::endl;
//    std::cout << "\thead.cmd:           0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)head.cmd << std::endl;
//    std::cout << "\thead.vin:           "   << std::string(head.vin, head.vin + sizeof(head.vin)) << std::endl;
//    std::cout << "\thead.ver:           0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)head.ver << std::endl;
//    std::cout << "\thead.length:        "   << std::dec << (int)head.length << std::endl;
//    std::cout << "\ttimestamp:          "   << BCD2TimeStamp(pLogin->timestamp, sizeof(pLogin->timestamp)) << std::endl;
//    std::cout << "\tsequence:           "   << be16toh(pLogin->sequence) << std::endl;
//    std::cout << "\ticcid:              "   << std::string(pLogin->iccid, pLogin->iccid + sizeof(pLogin->iccid)) << std::endl;
}

void Session::OnVehicleLogout(const GB_17691_MsgHead &head, const BYTE data[], std::size_t sz) {
    if (sz != sizeof(GB_17691_VehicleLogout)) {
        std::cerr << "unexpected message body size: " << sz << std::endl;
        return;
    }

//    auto pLogout = reinterpret_cast<const GB_17691_VehicleLogout *>(data);
//
//    std::cout << "OnVehicleLogout:" << std::endl;
//    std::cout << "\thead.flags:         0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)head.flags[0] << " 0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)head.flags[1] << std::endl;
//    std::cout << "\thead.cmd:           0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)head.cmd << std::endl;
//    std::cout << "\thead.vin:           "   << std::string(head.vin, head.vin + sizeof(head.vin)) << std::endl;
//    std::cout << "\thead.ver:           0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)head.ver << std::endl;
//    std::cout << "\thead.length:        "   << std::dec << (int)head.length << std::endl;
//    std::cout << "\ttimestamp:          "   << BCD2TimeStamp(pLogout->timestamp, sizeof(pLogout->timestamp)) << std::endl;
//    std::cout << "\tsequence:           "   << be16toh(pLogout->sequence) << std::endl;
}

void Session::OnSyncTime(const GB_17691_MsgHead &head, const BYTE [], std::size_t sz) {
    if (sz != 0) {
        std::cerr << "unexpected message body size: " << sz << std::endl;
        return;
    }

    std::cout << "OnSyncTime:" << std::endl;
    std::cout << "\thead.flags:         0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex
              << (int) head.flags[0] << " 0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex
              << (int) head.flags[1] << std::endl;
    std::cout << "\thead.cmd:           0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex
              << (int) head.cmd << std::endl;
    std::cout << "\thead.vin:           " << std::string(head.vin, head.vin + sizeof(head.vin)) << std::endl;
    std::cout << "\thead.ver:           0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex
              << (int) head.ver << std::endl;
    std::cout << "\thead.length:        " << std::dec << (int) head.length << std::endl;
}

void Session::OnReport(const GB_17691_MsgHead &head, const BYTE data[], std::size_t sz) {
    std::cout << "OnReport:" << std::endl;
    std::cout << "\thead.flags:         0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex
              << (int) head.flags[0] << " 0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex
              << (int) head.flags[1] << std::endl;
    std::cout << "\thead.cmd:           0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex
              << (int) head.cmd << std::endl;
    std::cout << "\thead.vin:           " << std::string(head.vin, head.vin + sizeof(head.vin)) << std::endl;
    std::cout << "\thead.ver:           0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex
              << (int) head.ver << std::endl;
    std::cout << "\thead.length:        " << std::dec << (int) head.length << std::endl;

//    std::shared_ptr<smartlink::MessageReport17691> spReport = std::make_shared<smartlink::MessageReport17691>();
//
//    auto rc = spReport->Decode(std::vector<BYTE>(data, data + sz));
//    if ( SL_SUCCESS != rc ) {
//        std::cerr << "fail to decode report message" << std::endl;
//        return;
//    }
//
//    std::cout << "\ttimestamp:          "   << BCD2TimeStamp(spReport->m_timestamp, sizeof(spReport->m_timestamp)) << std::endl;
//    std::cout << "\tseq:                "   << std::uppercase << std::setw(4) << std::setfill('0') << std::hex << (int)spReport->m_seq << std::endl;
//
//    for ( auto const &tuple : spReport->m_infos ) {
//        std::cout << "-------------------------------------------------------" << std::endl;
//        std::cout << "\t\tinfo type:    0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)std::get<0>(tuple) << std::endl;
//        if ( M_GB_17691_MESSAGE_OBD == std::get<0>(tuple) ) {
//            std::shared_ptr<smartlink::MessageOBDInfo>  spOBD = std::dynamic_pointer_cast<smartlink::MessageOBDInfo>(std::get<1>(tuple));
//            if ( !spOBD ) {
//                std::cerr << "mismatched information type and message body" << std::endl;
//                continue;
//            }
//
//            std::cout << "\t\tprotocol:         0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)spOBD->m_protocol << std::endl;
//            std::cout << "\t\tmil status:       0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)spOBD->m_mil_status << std::endl;
//            std::cout << "\t\tsupported status: 0x" << std::uppercase << std::setw(4) << std::setfill('0') << std::hex << (int)spOBD->m_diag_support_status << std::endl;
//            std::cout << "\t\tready status:     0x" << std::uppercase << std::setw(4) << std::setfill('0') << std::hex << (int)spOBD->m_diag_ready_status << std::endl;
//            std::cout << "\t\tVIN:              "   << std::string(spOBD->m_vin, spOBD->m_vin + sizeof(spOBD->m_vin)) << std::endl;
//            std::cout << "\t\tSCID:             "   << std::string(spOBD->m_scin, spOBD->m_scin + sizeof(spOBD->m_scin)) << std::endl;
//            std::cout << "\t\tCVN:              "   << std::string(spOBD->m_cvn, spOBD->m_cvn + sizeof(spOBD->m_cvn)) << std::endl;
//            std::cout << "\t\tIUPR:             "   << std::string(spOBD->m_iupr, spOBD->m_iupr + sizeof(spOBD->m_iupr)) << std::endl;
//            for ( auto const &dtc : spOBD->m_dtcs ) {
//                std::cout << "\t\tDTC               0x" << std::uppercase << std::setw(8) << std::setfill('0') << std::hex << dtc << std::endl;
//            }
//
//        } else if ( M_GB_17691_MESSAGE_STREAM == std::get<0>(tuple) ) {
//            std::shared_ptr<smartlink::MessageEngineInfo>   spEngine = std::dynamic_pointer_cast<smartlink::MessageEngineInfo>(std::get<1>(tuple));
//            if ( !spEngine ) {
//                std::cerr << "mismatched information type and message body" << std::endl;
//                continue;
//            }
//
//            std::cout << "\t\tm_tachograph_vehicle_speed:                                           " << spEngine->m_tachograph_vehicle_speed.Val() << std::endl;
//            std::cout << "\t\tm_barometric_pressure:                                                " << spEngine->m_barometric_pressure.Val() << std::endl;
//            std::cout << "\t\tm_actual_engine_percent_torque:                                       " << spEngine->m_actual_engine_percent_torque.Val() << std::endl;
//            std::cout << "\t\tm_nominal_friction_percent_torque:                                    " << spEngine->m_nominal_friction_percent_torque.Val() << std::endl;
//            std::cout << "\t\tm_engine_speed:                                                       " << spEngine->m_engine_speed.Val() << std::endl;
//            std::cout << "\t\tm_engine_fuel_rate:                                                   " << spEngine->m_engine_fuel_rate.Val() << std::endl;
//            std::cout << "\t\tm_aftertreatment_1_intake_nox:                                        " << spEngine->m_aftertreatment_1_intake_nox.Val() << std::endl;
//            std::cout << "\t\tm_aftertreatment_1_outlet_nox:                                        " << spEngine->m_aftertreatment_1_outlet_nox.Val() << std::endl;
//            std::cout << "\t\tm_catalyst_tank_level:                                                " << spEngine->m_catalyst_tank_level.Val() << std::endl;
//            std::cout << "\t\tm_engine_inlet_air_mass_flow_rate:                                    " << spEngine->m_engine_inlet_air_mass_flow_rate.Val() << std::endl;
//            std::cout << "\t\tm_aftertreatment_1_scr_intake_temperature:                            " << spEngine->m_aftertreatment_1_scr_intake_temperature.Val() << std::endl;
//            std::cout << "\t\tm_aftertreatment_1_scr_outlet_temperature:                            " << spEngine->m_aftertreatment_1_scr_outlet_temperature.Val() << std::endl;
//            std::cout << "\t\tm_aftertreatment_1_diesel_particulate_filter_differential_pressure:   " << spEngine->m_aftertreatment_1_diesel_particulate_filter_differential_pressure.Val() << std::endl;
//            std::cout << "\t\tm_engine_coolant_temperature:                                         " << spEngine->m_engine_coolant_temperature.Val() << std::endl;
//            std::cout << "\t\tm_fuel_level:                                                         " << spEngine->m_fuel_level.Val() << std::endl;
//            std::cout << "\t\tm_located_status:                                                     0x" << std::uppercase << std::hex << spEngine->m_located_status.Encoded() << std::endl;
//            std::cout << "\t\tm_longitude:                                                          " << spEngine->m_longitude.Val() << std::endl;
//            std::cout << "\t\tm_latitude:                                                           " << spEngine->m_latitude.Val() << std::endl;
//            std::cout << "\t\tm_total_vehicle_distance:                                             " << spEngine->m_total_vehicle_distance.Val() << std::endl;
//        }
//    }
}
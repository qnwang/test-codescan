//
// Created by t on 4/6/22.
//

#ifndef GB17691_PLATFORM_SESSION_H
#define GB17691_PLATFORM_SESSION_H


#pragma once
#include <cstdint>
#include <vector>

#include "protocol_def.h"

class Session {
public:
    Session() = delete;
    Session(int fd);
    ~Session();

    void OnRecv(const BYTE [], std::size_t);

private:
    void OnMessage(const std::vector<BYTE> &data);
    void OnVehicleLogin(const GB_17691_MsgHead &head, const BYTE [], std::size_t);
    void OnVehicleLogout(const GB_17691_MsgHead &head, const BYTE [], std::size_t);
    void OnSyncTime(const GB_17691_MsgHead &head, const BYTE [], std::size_t);
    void OnReport(const GB_17691_MsgHead &head, const BYTE [], std::size_t);

private:
    const int   m_sfd;
    std::size_t m_tail;
    BYTE        m_recv_buffer[16384];
};


#endif //GB17691_PLATFORM_SESSION_H

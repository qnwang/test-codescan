//
// Created by t on 4/6/22.
//

#ifndef GB17691_PLATFORM_SERVER_H
#define GB17691_PLATFORM_SERVER_H


#pragma once
#include <cstdint>
#include <mutex>
#include <thread>

class Server {
public:
    Server();
    ~Server();
    void    Stop();
    int     Start();
    bool    Running() const;
private:
    std::uint32_t   m_flags;
    std::mutex      m_mutex;
    std::thread     m_thread;
};


#endif //GB17691_PLATFORM_SERVER_H

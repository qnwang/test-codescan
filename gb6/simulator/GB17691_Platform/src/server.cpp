//
// Created by t on 4/6/22.
//

#include <set>
#include <vector>
#include <unordered_map>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>

#include "server.h"
#include "session.h"

#include "smlk_typedefs.h"

namespace Flags {
    const static std::uint32_t uninitialited = 0x00000000;
    const static std::uint32_t initialited = 0x00000001;
    const static std::uint32_t running = 0x00000002;
    const static std::uint32_t stopping = 0x00000004;
};

Server::Server()
        : m_flags(Flags::uninitialited) {}

Server::~Server() {}

void Server::Stop() {
    std::lock_guard<std::mutex> lk(m_mutex);
    if ((m_flags & Flags::running) && !(m_flags & Flags::stopping)) {
        m_flags |= Flags::stopping;
        if (m_thread.joinable()) {
            m_thread.join();
        }

        m_flags &= (Flags::running | Flags::stopping);
    }
}

int Server::Start() {
    std::cout << "server start" << std::endl;
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_flags & (Flags::running | Flags::stopping)) {
        return 0;
    }

    m_flags |= Flags::running;

    m_thread = std::thread(
            [this]() {
                struct sockaddr_in server_addr;

                std::memset(&server_addr, 0x00, sizeof(server_addr));

                server_addr.sin_family = AF_INET;
                // server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
                server_addr.sin_addr.s_addr = inet_addr("59.46.48.107");
                server_addr.sin_port = htons(6666);

                auto sfd = socket(AF_INET, SOCK_STREAM, 0);
                if (sfd < 0) {
                    m_flags &= ~(Flags::running | Flags::stopping);
                    std::cout << "fail to create socket" << std::endl;
                    std::cerr << "fail to create socket" << std::endl;
                    return;
                }

                if (bind(sfd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
                    m_flags &= ~(Flags::running | Flags::stopping);
                    std::cout << "fail to bind socket" << std::endl;
                    std::cerr << "fail to bind socket" << std::endl;
                    return;
                }

                if (listen(sfd, 5) < 0) {
                    m_flags &= ~(Flags::running | Flags::stopping);
                    std::cout << "fail to bind listen" << std::endl;
                    std::cerr << "fail to bind listen" << std::endl;
                    return;
                }

    std::cout << "server 11111" << std::endl;

                fd_set rsets;
                std::vector<int> fds;
                BYTE buffer[16384];

                std::unordered_map<int, std::shared_ptr<Session>> sessions;

                while (!(m_flags & Flags::stopping)) {
    std::cout << "server 22222" << std::endl;

                    int max_fd = sfd;
                    struct timeval tv = {0, 250000};

                    FD_ZERO(&rsets);
                    FD_SET(sfd, &rsets);

                    for (auto const &pair: sessions) {
                        FD_SET(pair.first, &rsets);
                        if (pair.first > max_fd) {
                            max_fd = pair.first;
                        }
                    }

                    auto nready = select(max_fd + 1, &rsets, nullptr, nullptr, &tv);
                    if (nready < 0) {
                        std::cerr << "fail to select" << std::endl;
                        break;
                    } else if (0 == nready) {
                        continue;
                    }

                    if (FD_ISSET(sfd, &rsets)) {
                        auto cfd = accept(sfd, nullptr, nullptr);
                        if (cfd < 0) {
                            std::cerr << "fail to accept" << std::endl;
                            break;
                        }
                        sessions[cfd] = std::make_shared<Session>(cfd);
                        continue;
                    }

                    fds.clear();
                    for (auto const &pair: sessions) {
                        if (FD_ISSET(pair.first, &rsets)) {
                            auto nbytes = recv(pair.first, buffer, sizeof(buffer), 0);
                            if (nbytes <= 0) {
                                fds.push_back(pair.first);
                                continue;
                            }
                            pair.second->OnRecv(buffer, nbytes);
                        }
                    }

                    for (auto fd: fds) {
                        close(fd);
                        std::cout << "one connection disconnected" << std::endl;
                        sessions.erase(fd);
                    }
                }

                close(sfd);

                for (auto const &pair: sessions) {
                    close(pair.first);
                }

                sessions.clear();

                m_flags &= ~(Flags::running | Flags::stopping);
            }
    );

    std::cout << "server exit" << std::endl;

    return 0;
}

bool Server::Running() const {
    return m_flags & (Flags::running | Flags::stopping);
}

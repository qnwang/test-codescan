#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>

#include "server.h"

namespace {
    volatile sig_atomic_t signal_status = 0;
}

static void signal_handler(int signal) {
    if ((SIGTERM == signal) || (SIGINT == signal)) {
        signal_status = signal;
    }
}


int main(int argc, char *argv[]) {
    for (auto index = 1; index < argc; index++) {
        std::cout << argv[index];
    }

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    Server server;

    server.Start();

    while ((SIGTERM != signal_status) && (SIGINT != signal_status) && server.Running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    server.Stop();

    return 0;
}
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
#include <thread>
#include <csignal>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <getopt.h>

#include "message_def_32960.h"

#include "report_service.h"


namespace {
    volatile sig_atomic_t signal_status = 0;
}

static void signal_handler(int signal) {
    if ( (SIGTERM == signal) || (SIGINT == signal) ) {
        signal_status = signal;
    }
}

static const struct option long_options[] = {
    {"help",                    no_argument,        nullptr,    'h'},
    {0,                         0,                  0,          0  }
};

static void print_help(void) {
    std::cout << "Options: \r\n\r\n"
        << "\t--help Print the help message.\r\n"
        << std::endl;
}

static int parse_arguments(int argc, char *argv[], std::unordered_map<std::string, std::string> &options) {
    int     character = 0;
    std::stringstream optss;

    for (size_t index = 0; index < sizeof(long_options)/sizeof(long_options[0]); index++) {
        if ( nullptr == long_options[index].name ) {
            break;
        }

        optss << static_cast<char>(long_options[index].val);
        if ( required_argument == long_options[index].has_arg ) {
            optss << ":";
        }
    }

    character = getopt_long(argc, argv, optss.str().c_str(), long_options, nullptr);
    while ( character > 0 ) {
        switch (character) {
            case 'h':
                options["help"] = "";
                break;
            default:
                return 1;
        }
        character = getopt_long(argc, argv, optss.str().c_str(), long_options, nullptr);
    }

    return 0;
}

static void get_envs(std::unordered_map<std::string, std::string> &envs) {
    for ( auto it = envs.begin(); it != envs.end(); ++it ) {
        const char *ptr = std::getenv(it->first.c_str());
        if ( nullptr != ptr ) {
            envs[it->first] = ptr;
        }
    }
}

int main(int argc, char *argv[]) {
    std::unordered_map<std::string, std::string>    options;
    std::unordered_map<std::string, std::string>    envs = {};

    setenv("TZ", "CST-8", 1);
    tzset();

    get_envs(envs);

    if ( 0 != parse_arguments(argc, argv, options) ) {
        std::cerr << "fail to parse arguments!!!" << std::endl;
        print_help();
        return -1;
    }

    if ( options.end() != options.find("help") ) {
        print_help();
        return 0;
    }

    smartlink::ReportService    ser;

    if ( SL_SUCCESS != ser.Init() ) {
        std::cerr << "fail to init service" << std::endl;
        return -1;
    }

    if ( SL_SUCCESS != ser.Start() ) {
        std::cerr << "fail to start service" << std::endl;
        return -1;
    }

    signal(SIGTERM, signal_handler);
    signal(SIGINT,  signal_handler);

    while ( (signal_status != SIGTERM) && (signal_status != SIGINT) ) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        if ( !ser.IsRunning() ) {
            std::cerr << "serivce stopped unexpected" << std::endl;
            break;
        }
    }

    ser.Stop();

    return 0;
}

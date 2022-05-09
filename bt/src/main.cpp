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
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <time.h>
#include <stdlib.h>
#include <getopt.h>
#include <execinfo.h>
#include "errors_def.h"
#include <bluetooth_service.h>
#include <smlk_log.h>

using namespace smartlink;

namespace {
    volatile sig_atomic_t signal_status = 0;
}

static void signal_handler(int signal) {
    if ( (SIGTERM == signal) || (SIGINT == signal) ) {
        signal_status = signal;
    } else if ( SIGABRT == signal ) {
        void    *buffer[32];

        int     nptrs       = backtrace(buffer, sizeof(buffer)/sizeof(buffer[0]));
        char    **strings   = backtrace_symbols(buffer, nptrs);

        if ( nullptr != strings ) {
            for ( decltype(nptrs) i = 0; i < nptrs; i++ ) {
                SMLK_LOGE("%s", strings[i]);
            }
            free(strings);
        }
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
    static const std::unordered_map<std::string, std::string>   params = {
    };

    std::unordered_map<std::string, std::string>    options;
    if ( 0 != parse_arguments(argc, argv, options) ) {
        std::cerr << "fail to parse arguments!!!" << std::endl;
        print_help();
        return -1;
    }

    if ( options.end() != options.find("help") ) {
        print_help();
        return 0;
    }

    {
        setenv("TZ", "CST-8", 1);
        tzset();
    }

    std::unordered_map<std::string, std::string>    envs;
    for ( auto const pair : params ) {
        envs[pair.second] = "";
    }
    get_envs(envs);

    for ( auto const pair : params ) {
        auto const it = options.find(pair.first);
        if ( (options.end() == it) || (0 == it->second.length()) ) {
            options[pair.first] = envs[pair.second];
        }
    }

    signal(SIGTERM, signal_handler);
    signal(SIGINT,  signal_handler);
    signal(SIGABRT, signal_handler);

    BluetoothService   ser;

    if ( SL_SUCCESS != ser.Init() ) {
        std::cerr << "service init failed" << std::endl;
        return -1;
    }

    if ( SL_SUCCESS != ser.Start() ) {
        std::cerr << "service start failed" << std::endl;
    }

    while ( (signal_status != SIGTERM) && (signal_status != SIGINT) ) {
        if ( !ser.IsRunning() ) {
            std::cerr << "service stopped abnormallly" << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    ser.Stop();

    return 0;
}

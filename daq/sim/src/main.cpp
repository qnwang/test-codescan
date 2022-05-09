#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <vector>
#include <unordered_map>
#include <csignal>
#include <cstring>
#include <endian.h>
#include <unistd.h>
#include <getopt.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <json/json.h>

using BYTE = std::uint8_t;
using WORD = std::uint16_t;
using DWORD = std::uint32_t;

typedef struct
{
    WORD    msg_id;         // 消息ID
    WORD    seq_id;         // 消息序号
    WORD    protocol:4;     // 消息体采用协议类型   0x01: 808, 0x02: MQTT, 0x03: http*/
    WORD    compress:4;     // 消息体压缩标识       0x00: 不压缩; 0x01：zlib压缩*/
    WORD    reserved:8;     // 保留字段，扩展用
    WORD    length;         // 有效载荷长度(不包括本消息体长度)
    BYTE    qos;
    BYTE    priority;
    DWORD   reserved_dw;    // 保留字段，扩展用
}__attribute__((__packed__))IpcTspHead;

namespace {
    volatile sig_atomic_t signal_status = 0;
}

namespace UnixDomainSocket {
    const std::string   sun_path    = "/run/collection_service.sock";
};

namespace Event {
    static const WORD   SL_SM_EVENT_START   = 0xFE01;       // drive event start
    static const WORD   SL_SM_EVENT_STOP    = 0xFE02;       // drive event stop
    static const WORD   SL_SM_FAULT_OCCUR   = 0xFE11;       // fault occur
    static const WORD   SL_SM_FAULT_CLEAR   = 0xFE12;       // fault clear
};

namespace FaultEventMask {
    static const BYTE   SL_EVENT_DM1        = 0x01;
    static const BYTE   SL_EVENT_ENG        = 0x02;
    static const BYTE   SL_EVENT_UDS        = 0x04;
    static const BYTE   SL_EVENT_ABN        = 0x08;
};

static void signal_handler(int signal) {
    if ( (SIGTERM == signal) || (SIGINT == signal) ) {
        signal_status = signal;
    }
}

static const struct option long_options[] = {
    {"help",                    no_argument,        nullptr,        'h'},
    {"event",                   no_argument,        nullptr,        'e'},
    {"fault",                   no_argument,        nullptr,        'f'},
    {"id",                      required_argument,  nullptr,        'i'},
    {"period",                  required_argument,  nullptr,        'p'},
    {"dtc",                     required_argument,  nullptr,        'd'},
    {"start",                   no_argument,        nullptr,        's'},
    {"occur",                   no_argument,        nullptr,        'o'},
    {0,                         0,                  0,              0  },
};

static void print_help(void) {
    std::cout << "Options: \r\n\r\n"
        << "\t--help                                    Print the help message.\r\n"
        << "\t--event                                   Simulator event.\r\n"
        << "\t--fault                                   Simulator fault.\r\n"
        << "\t[--id=[0x0A|0x0C|0x0E|0x12|0x13|0x14]]    Event to simulator, default will be 0x0C.\r\n"
        << "\t[--dtc=src,fmi,spn[;src,fmi,spn...]]      Fault dtc lists.\r\n"
        << "\t[--period=[5]]                            Event continus seconds, default is 5 seconds.\r\n"
        << "\t[--start]                                 Event start flag, otherwise, it will be consider as event stop event.\r\n"
        << "\t[--occur]                                 Fault occurs.\r\n"
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
            case 'e':
                options["event"] = "";
                break;
            case 'f':
                options["fault"] = "";
                break;
            case 'i':
                options["id"] = optarg;
                break;
            case 'p':
                options["period"] = optarg;
                break;
            case 'd':
                options["dtc"] = optarg;
                break;
            case 's':
                options["start"] = "";
                break;
            case 'o':
                options["occur"] = "";
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
    std::unordered_map<std::string, std::string>    options = {
        {"id",          "0x0C"                      },
        {"period",      "5"                         },
        {"dtc",         "123,456,789"               },
    };
    if ( 0 != parse_arguments(argc, argv, options) ) {
        std::cerr << "fail to parse arguments!!!" << std::endl;
        print_help();
        return -1;
    }

    if ( options.end() != options.find("help") ) {
        print_help();
        return 0;
    }

    static constexpr auto lambda = [](IpcTspHead &head){
        head.msg_id         = htobe16(head.msg_id);
        head.seq_id         = htobe16(head.seq_id);
        head.length         = htobe16(head.length);
        head.reserved_dw    = htobe32(head.reserved_dw);
    };

    static constexpr auto split = [](const std::string &str, const std::string &sub)->std::vector<std::string> {
    	std::vector<std::string>    vs;
        std::string                 s(str);
        auto pos    = s.find(sub);
        while ( std::string::npos != pos ) {
        	vs.push_back(s.substr(0, pos));
            s = s.substr(pos + sub.length());
            pos = s.find(sub);
        }
        vs.push_back(s);

        return vs;
    };

    IpcTspHead  head = {};
    std::vector<BYTE>   body;

    if ( options.end() != options.find("event") ) {
        if ( options.end() != options.find("start") ) {
            head.msg_id = Event::SL_SM_EVENT_START;
        } else {
            head.msg_id = Event::SL_SM_EVENT_STOP;
        }

        BYTE    id;

        try {
            id      = (std::uint8_t)std::stoul(options["id"], nullptr, 16);
        } catch (const std::invalid_argument &) {
            std::cerr << "invalid argument" << std::endl;
            print_help();
            return -1;
        } catch (const std::out_of_range &) {
            std::cerr << "out of range" << std::endl;
            print_help();
            return -1;
        }

        body.push_back(id);
        body.push_back(0x00);
    } else if ( options.end() != options.find("fault") ) {
        if ( options.end() != options.find("occur") ) {
            head.msg_id = Event::SL_SM_FAULT_OCCUR;
        } else {
            head.msg_id = Event::SL_SM_FAULT_CLEAR;
        }

        head.reserved_dw    = (DWORD)(FaultEventMask::SL_EVENT_DM1) << 24;

        auto const it = options.find("dtc");
        if ( options.end() != it && it->second.length() ) {
            auto fields = split(it->second, ";");
            body.push_back((std::uint8_t)fields.size());
            for ( auto field : fields ) {
                auto vars = split(field, ",");
                if ( 3 != vars.size() ) {
                    std::cerr << "invalid dtc format" << std::endl;
                    print_help();
                    return -1;
                }

                std::uint8_t    src;
                std::uint8_t    fmi;
                std::uint32_t   spn;

                try {
                    src = (std::uint8_t)std::stoul(vars[0], nullptr);
                    fmi = (std::uint8_t)std::stoul(vars[1], nullptr);
                    spn = (std::uint32_t)std::stoul(vars[2], nullptr);
                } catch (const std::invalid_argument &) {
                    std::cerr << "invalid argument" << std::endl;
                    print_help();
                    return -1;
                } catch (const std::out_of_range &) {
                    std::cerr << "out of range" << std::endl;
                    print_help();
                    return -1;
                }

                spn = htobe32(spn);

                body.push_back(src);
                body.push_back(fmi);
                body.insert(body.end(), (std::uint8_t *)&spn, (std::uint8_t *)&spn + sizeof(spn));
            }
        } else {
            if ( Event::SL_SM_FAULT_OCCUR == head.msg_id ) {
                std::cerr << "no valid dtc specified" << std::endl;
                return -1;
            }

            body.push_back(0x00);
        }
    } else {
        std::cerr << "invalid argument" << std::endl;
        print_help();
        return -1;
    }

    std::vector<BYTE>   buffer;

    buffer.push_back('|');
    buffer.push_back('#');

    head.length = body.size();

    lambda(head);

    buffer.insert(buffer.end(), (std::uint8_t *)&head, (std::uint8_t *)&head + sizeof(head));
    buffer.insert(buffer.end(), body.cbegin(), body.cend());

    int                 rc;
    int                 fd;
    struct sockaddr_un  addr;

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if ( fd < 0 ) {
        std::cerr << "fail to crete unix domain socket fd" << std::endl;
        return -3;
    }

    std::memset(&addr, 0, sizeof(addr));

    /* Connect socket to socket address */
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, UnixDomainSocket::sun_path.c_str(), sizeof(addr.sun_path) - 1);

    rc = connect(fd, (const struct sockaddr *) &addr, sizeof(addr));
    if ( rc < 0 ) {
        std::cerr << "fail to connect to server: " << std::strerror(errno) << std::endl;
        return -4;
    }

    std::stringstream   ss;
    for ( auto ch : buffer ) {
        ss << " " << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)ch;
    }
    std::cout << "---->" << ss.str() << std::endl;

    std::cout << "try to send out request" << std::endl;
    send(fd, buffer.data(), buffer.size(), 0);
    std::cout << "send out reqeust finished" << std::endl;

    if ( (options.cend() != options.find("event"))
      && (options.cend() != options.find("start")) ) {
        std::uint32_t   period = 0;

        auto const it = options.find("period");
        try {
            period = (std::uint32_t)std::stoul(it->second, nullptr);
        } catch ( ... ) {
            std::cerr << "fail to parse period value, use default value" << std::endl;
            period = 10;        // no matther what's the reason exception happended, default value 10s
        }

        if ( period ) {
            std::cout << "wait timeout" << std::endl;
            for ( decltype(period) index = 0; index < period; index++ ) {
                std::cout << "sleep 1 seconds ..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            head.msg_id = Event::SL_SM_EVENT_STOP;
            head.length = sizeof(BYTE) + sizeof(BYTE);

            lambda(head);

            buffer.clear();

            buffer.push_back('|');
            buffer.push_back('#');
            buffer.insert(buffer.end(), (std::uint8_t *)&head, (std::uint8_t *)&head + sizeof(head));
            buffer.push_back((std::uint8_t)std::stoul(options["id"], nullptr, 16));
            buffer.push_back(0x00);

            std::cout << "try to send automatic stop request" << std::endl;
            send(fd, buffer.data(), buffer.size(), 0);
        }
    }

    close(fd);

    return 0;
}

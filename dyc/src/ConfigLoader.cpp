#include <dyc_util.h>
#include "ConfigLoader.h"
#include "dyc_config.h"
#include "vec_signals.h"
using namespace std;
#define BLANKTAG ' '          //
#define COLONTAG ':'          //
#define ATTAG '@'            //描述大小端
#define VERTICALBARTAG '|'   // 前：起始bit 后：信号长度
#define LITTLEENDDIANTAG '0' //
#define BIGENDDIANTAG '1'    //
#define SIGNEDTAG '-'        // SG_
#define UNSIGNEDTAG '+'      //
#define SOLUTION_AND_OFFSET ','
#define SOLUTION_LEFT '('
#define OFFSET_RIGHT ')'
#define DEFAULT_CYCLE 1000


#define BA_PARTS_NUM 3
#define BA_PART_MSGID 0
#define BA_PART_NAME 1
#define BA_PART_VALUE 2


#define BO_PARTS_NUM 4
#define BO_PART_ID 0
#define BO_PART_NAME 1
#define CAN_MAX_LENGTH 64
#define CAN_VALUE_LENGTH 32
#define ROUND_UP_INTERGER(a, b)  (((a) + (b) - 1) / (b))

using namespace smartlink;
/******************************************************************************
 * NAME: trim
 *
 * DESCRIPTION: 以关键字分割获取目标字符
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *  give string : "RFDTCSeriousLevel : 4|4@1+ (1,0) [0|1] ""  TBOX"
 *  with delim " ", container "@"
 *  return "string 4|4@1+"
 *****************************************************************************/
static std::string& getDestString(std::string& str, char delim, char container)
{
    std::size_t found = str.find_first_of(container);
    if (found != std::string::npos)
    {
        for (std::size_t i = (std::size_t)found + 1; i < (std::size_t)str.size(); i++)
        {
            if (str[i] == delim)
            {
                str = str.erase(i);
                break;
            }
        }
        for (SMLK_UINT32  i = (SMLK_UINT32)found - 1; i >= 0; i--)
        {
            if (str[i] == delim)
            {
                str = str.erase(0, i + 1);
                break;
            }
        }
    }
    else
    {
        str = str.erase();
    }
    return str;
}

/******************************************************************************
 * NAME: trim
 *
 * DESCRIPTION: 从字符串中以关键字去除
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
static std::string& trim(std::string& s)
{
    if (s.empty())
    {
        return s;
    }
    s = s.erase(0, s.find_first_not_of(BLANKTAG));
    s = s.erase(s.find_last_not_of(BLANKTAG) + 1);
    return s;
}

/******************************************************************************
 * NAME: trim
 *
 * DESCRIPTION: 从字符串中以关键字去除
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
static std::string& trim(std::string& s, const char ignorestr)
{
    if (s.empty())
    {
        return s;
    }
    s = s.erase(0, s.find_first_of(ignorestr) + 1);
    s = s.erase(s.find_last_of(ignorestr));
    return s;
}

template<typename Out>
void split(const std::string& s, char delim, Out result)
{
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim))
    {
        *(result++) = item;
    }
}

/******************************************************************************
 * NAME: split
 *
 * DESCRIPTION: 从字符串中以关键字分割
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
std::vector<std::string> split(const std::string& s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}


std::string ConfigLoader::GetDbcPath(DBC_INDEX &dbc_index)
{
    SMLK_LOGD("on GetDbcPath index[%d]", dbc_index);
    std::string store_path(DBC_FILE_PATH);
    switch(dbc_index)
    {
        case DBC_INDEX_ID::DBC_INDEX_A:
            store_path.append("a.dbc");
            break;
        case DBC_INDEX_ID::DBC_INDEX_B:
            store_path.append("b.dbc");
            break;
        case DBC_INDEX_ID::DBC_INDEX_C:
            store_path.append("c.dbc");
            break;
        case DBC_INDEX_ID::DBC_INDEX_D:
            store_path.append("d.dbc");
            break;
        case DBC_INDEX_ID::DBC_INDEX_E:
            store_path.append("e.dbc");
            break;
        default:
           break;
    }
    return store_path;
}

/******************************************************************************
 * NAME: CheckDbcValid
 *
 * DESCRIPTION: 检查文件内容有效性
 * PARAMETERS:
 * 初步只检查是否有BO_ 2565866755 CAN_1_ETC2: 8 Vector__XXX此类数据
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
bool ConfigLoader::CheckDbcValid(DBC_INDEX &dbc_index)
{
    bool ret = true;
    SMLK_LOGD("on LoadConfigFromDBCFile ");
    std::string dbc_path = GetDbcPath(dbc_index);
    if (dbc_path.length() < 30) // modify hard code
    {
        SMLK_LOGE("error on dbc_path is %s ", dbc_path.c_str());
        ret = false;
        return ret;
    }
    SMLK_LOGD("on dbc_path is %s ", dbc_path.c_str());
    int64_t fileSize = 0;
    fileSize = FileTool::fileSize(dbc_path);
    SMLK_LOGD("%s file size is %d", dbc_path.c_str(), fileSize);
    if (fileSize <= 0 ) {
        SMLK_LOGE("Now file size <= 0 ,return true and delete it!");
        ret = false;
        return ret;
    }

    ifstream fin(dbc_path);
    string str;
    string signalhead("SG_ ");
    string baHead("BA_ \"CollectInterval\" SG_ ");
    string versionHead("VERSION");
    string msghead("BO_ ");

    list<string> signalList;
    list<string> msgList;

    if (!fin.is_open())
    {
        SMLK_LOGE("open dbc fail");
        ret = false;
        return ret;
    }

    // get version
    (void)getline(fin, str);
    trim(str);

    if (str.compare(0, versionHead.size(), versionHead) == 0)
    {
        trim(str, '\"');
        std::cout << "Version is " << str << std::endl;
    }

    SMLK_UINT16 valid_can_data_count = 0;

    while (!fin.eof())
    {
        (void)getline(fin, str);
        trim(str);
        SMLK_UINT32 signal_bit_len = 0;
        SMLK_UINT8 signal_start_bit = 0;
        SMLK_DOUBLE signal_solution = 0;
        SMLK_DOUBLE signal_offset = 0;
        SMLK_DOUBLE signal_phy_max_value = 0;
        SMLK_DOUBLE signal_phy_min_value = 0;
        SMLK_UINT32 signal_spn = 0;
        std::string signal_str;
        ByteOrderType order_type = ByteOrderType::BYTE_ORDER_BIG_ENDIAN;
        ValueType val_type = ValueType::VALUE_TYPE_UNSIGNED;

        if (str.compare(0, msghead.size(), msghead) == 0)
        {
            // 以通道分类填充canid
            std::string msg_all = str.substr(msghead.size());
            vector<std::string> splitStrs = split(msg_all, BLANKTAG);
            if (splitStrs.size() < BO_PARTS_NUM) {
                ret = false;
                return ret;
                // error size fomart
            }
            // add
            ++valid_can_data_count;

            (void)splitStrs[BO_PART_NAME].erase(splitStrs[BO_PART_NAME].find_last_of(':'));
            (void)getline(fin, str);
            trim(str);

            while (str.compare(0, signalhead.size(), signalhead) == 0) {
                // find one signal
                std::cout << "signal is " << str.substr(signalhead.size()) << std::endl;
                signal_str = str.substr(signalhead.size(), (str.find_first_of(":")-signalhead.size()-1));
                std::string signal_str_bit_info = str.substr(signalhead.size());
                std::string offset_info = str.substr(signalhead.size());
                int range_len = (int)str.find_last_of("]")-str.find_first_of("[");
                std::string value_range = str.substr(str.find_first_of("["),range_len+1);
                // std::cout << "value_range is " << value_range << std::endl;
                getDestString(signal_str_bit_info, BLANKTAG, ATTAG);
                getDestString(offset_info, BLANKTAG, SOLUTION_AND_OFFSET);
                // add spn to dbc map for sample by interval
                SMLK_LOGD("push back spn %d to vec", signal_spn);

                SMLK_UINT8 byte_lenth = ROUND_UP_INTERGER(signal_bit_len, 8);
                // rounded up the start byte
                SMLK_UINT8 start_byte = ROUND_UP_INTERGER(signal_start_bit, 8) + 1;
                SMLK_UINT32 signal_start_bit_cal = signal_start_bit + 1;

                (void)getline(fin, str);
                trim(str);
            }
            continue;
        }
    }
    fin.close();
    if ( valid_can_data_count >= 1) {
        ret = true;
    } else {
        ret = false;
    }
    return ret;
}

/******************************************************************************
 * NAME: NewLoadConfigFromDBCFile
 *
 * DESCRIPTION: 给定dbc_index 将此DBC的  msg-signal/ dbc-signal/ signal解析规则
 *              解析到全量数据和dbc采集规则
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void ConfigLoader::NewLoadConfigFromDBCFile(DBC_INDEX &dbc_index , \
                                                std::unordered_map<DBC_INDEX, std::vector<SPN_INDEX>> *dbc_sample_signals_map,\
                                                MessageDecodeMap *message_decode_map)
{
    SMLK_LOGD("on LoadConfigFromDBCFile ");
    std::string dbc_path = GetDbcPath(dbc_index);
    if (dbc_path.length() < 30) // modify hard code
    {
        SMLK_LOGE("error on dbc_path is %s ", dbc_path.c_str());
        return;
    }
    SMLK_LOGD("on dbc_path is %s ", dbc_path.c_str());
    int64_t fileSize = 0;
    fileSize = FileTool::fileSize(dbc_path);
    SMLK_LOGD("%s file size is %d", dbc_path.c_str(), fileSize);
    if (fileSize <= 0 ) {
        SMLK_LOGE("Now file size <= 0 ,return true and delete it!");
        return;
    }

    auto iter_dbc = dbc_sample_signals_map->find(dbc_index);
    if ( dbc_sample_signals_map->end() != iter_dbc) {
        SMLK_LOGI("clear old dbc sample index!");
        iter_dbc->second.clear();
    }

    ifstream fin(dbc_path);
    string str;
    string signalhead("SG_ ");
    string baHead("BA_ \"CollectInterval\" SG_ ");
    string versionHead("VERSION");
    string msghead("BO_ ");

    list<string> signalList;
    list<string> msgList;

    if (!fin.is_open())
    {
        SMLK_LOGE("open dbc fail");
        return;
    }

    // get version
    (void)getline(fin, str);
    trim(str);

    if (str.compare(0, versionHead.size(), versionHead) == 0)
    {
        trim(str, '\"');
        std::cout << "Version is " << str << std::endl;
    }
    while (!fin.eof())
    {
        (void)getline(fin, str);
        trim(str);
        MessageDecodeMap msg_signals_map;
        CanFormatData::CanMessageInfo CanMsgInfo;
        SMLK_UINT32 signal_bit_len = 0;
        SMLK_UINT8 signal_start_bit = 0;
        SMLK_DOUBLE signal_solution = 0;
        SMLK_DOUBLE signal_offset = 0;
        SMLK_DOUBLE signal_phy_max_value = 0;
        SMLK_DOUBLE signal_phy_min_value = 0;
        SMLK_UINT32 signal_spn = 0;
        std::string signal_str;
        ByteOrderType order_type = ByteOrderType::BYTE_ORDER_BIG_ENDIAN;
        ValueType val_type = ValueType::VALUE_TYPE_UNSIGNED;
        std::vector<SignalFormat> var_vec;
        std::vector<SPN_INDEX> spn_vec;

        if (str.compare(0, msghead.size(), msghead) == 0)
        {
            // 以通道分类填充canid
            std::string msg_all = str.substr(msghead.size());
            vector<std::string> splitStrs = split(msg_all, BLANKTAG);
            if (splitStrs.size() < BO_PARTS_NUM) {
                return;
                // error size fomart
            }
            // std::cout << "----------another msg  id is----------- "<< splitStrs[0] << std::endl;
            SMLK_LOGD("#######another msgId######");
            CAN_ID raw_msgId = (SMLK_UINT32)stod(splitStrs[0]);
            std::cout << "raw_msgId id is " << raw_msgId << std::endl;
            CAN_ID msgId = ((raw_msgId << 3) >> 3);
            SMLK_LOGD("msgId [%d]", msgId);

            (void)splitStrs[BO_PART_NAME].erase(splitStrs[BO_PART_NAME].find_last_of(':'));

            std::string msg_name = splitStrs[BO_PART_NAME].c_str();
            SMLK_LOGD("msg name [%s]", msg_name.c_str());

            (void)getline(fin, str);
            trim(str);

            while (str.compare(0, signalhead.size(), signalhead) == 0) {
                // find one signal
                std::cout << "signal is " << str.substr(signalhead.size()) << std::endl;
                signal_str = str.substr(signalhead.size(), (str.find_first_of(":")-signalhead.size()-1));
                // std::cout << "signal name is " << signal_str << std::endl;

                std::string spn_str = signal_str.substr(signal_str.find_last_of("_")+1);

                // signal_spn = stod(spn_str);
                std::stringstream str_ss;
                str_ss << spn_str;
                str_ss >> signal_spn;
                // std::cout << "signal_spn is " << signal_spn << std::endl;

                signalList.push_back(str.substr(signalhead.size()));
                std::string signal_str_bit_info = str.substr(signalhead.size());
                std::string offset_info = str.substr(signalhead.size());
                int range_len = (int)str.find_last_of("]")-str.find_first_of("[");
                std::string value_range = str.substr(str.find_first_of("["),range_len+1);
                // std::cout << "value_range is " << value_range << std::endl;

                getDestString(signal_str_bit_info, BLANKTAG, ATTAG);
                getDestString(offset_info, BLANKTAG, SOLUTION_AND_OFFSET);

                std::vector<std::string> splits1 = split(signal_str_bit_info, VERTICALBARTAG);
                if (2 == splits1.size())
                {
                    // pSignal->setStartBit((SMLK_UINT32)atoi(splits1[0].c_str()));
                    // std::cout << " # signal setStartBit "  << splits1[0].c_str() << std::endl;
                    signal_start_bit = stod(splits1[0]);
                    // std::cout << " # signal StartBit_value "  << signal_start_bit << std::endl;

                    vector<std::string> splits2 = split(splits1[1], ATTAG);

                    if (splits2.size() == 2)
                    {
                        signal_bit_len = (SMLK_UINT32)atoi(splits2[0].c_str());

                        if (signal_bit_len > CAN_VALUE_LENGTH)
                        {
                            signal_bit_len = CAN_MAX_LENGTH;
                        }
                        // std::cout << "# signal bitLength" << signal_bit_len << std::endl;

                        if (BIGENDDIANTAG == splits2[1][0])
                        {
                            // std::cout << "setByteOrder BYTE_ORDER_BIG_ENDIAN"  << std::endl;
                            order_type = ByteOrderType::BYTE_ORDER_BIG_ENDIAN;
                        }
                        else if (LITTLEENDDIANTAG == splits2[1][0])
                        {
                            // std::cout << "setByteOrder LITTLEENDDIANTAG "<< std::endl;
                            order_type = ByteOrderType::BYTE_ORDER_LITTLE_ENDIAN;
                        }
                        if (UNSIGNEDTAG == splits2[1][1])
                        {
                            val_type = ValueType::VALUE_TYPE_UNSIGNED;
                        }
                        else if (SIGNEDTAG == splits2[1][1])
                        {
                            // std::cout << "setValueType SIGNEDTAG"  <<std::endl;
                            val_type = ValueType::VALUE_TYPE_SIGNED;
                        }
                    }
                }

                // std::cout << "offset_info  is "  << offset_info<< std::endl ;
                auto solution_start = offset_info.find_first_of("(");
                // offset_info.erase(solution_start,1);
                // stol stod
                int soultion_len = offset_info.find_first_of(",") - offset_info.find_first_of("(") -1;
                std::string soultion = offset_info.substr(offset_info.find_first_of("(")+1, soultion_len);
                // std::cout << "#signal solution is "  << soultion << std::endl;

                auto start = offset_info.find_first_of(",");
                auto end = offset_info.find_first_of(")");
                int offset_len = end - start -1;
                std::string offset = offset_info.substr(offset_info.find_first_of(",")+1 , offset_len);
                // std::cout << "#signal offset  is "  << offset << std::endl;
                // signal_offset = stod(offset);
                // std::cout << "#signal offset_value  is "  << signal_offset << std::endl;

                std::stringstream temp;
                temp << soultion;
                temp >> signal_solution;
                temp.clear();
                temp.str(std::string());
                std::cout << "#signal soultion_value  is "  << signal_solution << std::endl;

                temp << offset;
                temp >> signal_offset;
                temp.clear();
                temp.str(std::string());
                std::cout << "#signal soultion_value  is "  << signal_solution << std::endl;

                int phy_min_len = (int)value_range.find_last_of("|")- value_range.find_first_of("[")-1;
                int phy_max_len = (int)value_range.find_last_of("]")- value_range.find_first_of("|")-1;

                std::string phy_min = value_range.substr(value_range.find_first_of("[")+1, phy_min_len);
                std::cout << "#signal value_phy_min is " << phy_min << std::endl;

                std::string phy_max = value_range.substr(value_range.find_first_of("|")+1, phy_max_len);
                std::cout << "#signal phy_max is " << phy_max << std::endl;

                temp << phy_max;
                temp >> signal_phy_max_value;
                temp.clear();
                temp.str(std::string());

                temp << phy_min;
                temp >> signal_phy_min_value;
                temp.clear();
                temp.str(std::string());
                std::cout << "#signal phy_max value is " << signal_phy_max_value << std::endl;

                std::cout << "#signal signal_phy_min_value  is "  << signal_phy_min_value << std::endl;

                CanMsgInfo.byte_lenth = signal_bit_len;
                CanMsgInfo.cycle = DEFAULT_CYCLE;
                // add spn to dbc map for sample by interval
                SMLK_LOGD("push back spn %d to vec", signal_spn);
                spn_vec.push_back(signal_spn);

                SMLK_UINT8 byte_lenth = ROUND_UP_INTERGER(signal_bit_len, 8);
                // rounded up the start byte
                SMLK_UINT8 start_byte = ROUND_UP_INTERGER(signal_start_bit, 8) + 1;
                SMLK_UINT32 signal_start_bit_cal = signal_start_bit + 1;

                SMLK_LOGD("SPN_INDEX[%u] and Min[signal_phy_min_value[%f]  Max[signal_phy_max_value%f]",\
                   signal_spn , signal_phy_min_value, signal_phy_max_value);
                SMLK_LOGD("SPN_INDEX[%u]  start_byte[%d]  signal_start_bit[%d]",\
                   signal_spn , start_byte, signal_start_bit_cal);
                SMLK_LOGD("SPN_INDEX[%u]  resolution[%lf]  offset[%lf]",\
                   signal_spn , signal_solution, signal_offset);

                var_vec.emplace_back(signal_spn, signal_start_bit_cal, signal_bit_len,
                    signal_phy_min_value, signal_phy_max_value, signal_offset,
                    signal_solution, ByteOrderType::BYTE_ORDER_BIG_ENDIAN,
                    byte_lenth, start_byte);

                SMLK_LOGD("#var_vec size [%d] ", var_vec.size());
                (void)getline(fin, str);
                trim(str);
            }
            auto it = dbc_sample_signals_map->find(dbc_index);
            if (dbc_sample_signals_map->end() == it) { // 缓存中没有此dbc索引，需要增加
                SMLK_LOGD("not find this dbc_index %d", dbc_index);
                dbc_sample_signals_map->emplace(
                    std::piecewise_construct,
                    std::forward_as_tuple(dbc_index),
                    std::forward_as_tuple(spn_vec)
                );
            } else {
                // 更新新出现的spn到已有dbc的spn列表
                for (auto &sig : spn_vec) {
                    SMLK_LOGD("push spn %d ", sig);
                    it->second.push_back(sig);
                }
            }
            SMLK_LOGD("insert msgId [%d] ", msgId);
            // check repeated msg id or repeat signalID ind different dbc
            message_decode_map->emplace(std::make_pair(msgId, std::make_pair(CanMsgInfo, var_vec)));
            continue;
        }
    }
    fin.close();
    // spn去重
    auto iter = dbc_sample_signals_map->find(dbc_index);
    sort(iter->second.begin(), iter->second.end());
    auto iter_sort = unique(iter->second.begin(), iter->second.end());
    iter->second.erase(iter_sort,iter->second.end());

    for (const auto &index : vehicle::g_header_signal_spn_vec) {
        // SMLK_LOGD(" add index [%d] ", index);
        iter->second.insert(iter->second.begin(), index);
    }

    std::cout << "signalList size is " << signalList.size() << std::endl;
    signalList.clear();

}

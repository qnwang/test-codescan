#pragma once
#include <tuple>
#include <unordered_map>
#include <smlk_types.h>
#include <smlk_signal.h>
#include "dyc_util.h"
#include "DbcConfigFormat.h"
#include "DynamicCollection.h"
#define     DBC_FILE_PATH          "/userdata/smartlink/record/dbc/"
#define     DBC_FILE_CONFIG_PATH  "/userdata/smartlink/record/dbc/config/"
#define     TSP_CLEAR_CONFIG_MAX_NUM          5
#define SPN_GNSS_TimeStamp_S              (516129)             // 时间
#define SPN_GNSS_LATITUDE                 (516130)             // 纬度
#define SPN_GNSS_LONGITUDE                (516131)             // 经度
#define SPN_GNSS_SPEED                    (516132)             // 速度
#define SPN_GNSS_LOCATEDSTATUS            (516133)             // 定位状态
#define SPN_GNSS_ALTITUDE                 (516134)             // 高度
#define SPN_GNSSTimeStamp_ABS_MS          (516200)             // 毫秒数

enum DBC_INDEX_ID : SMLK_UINT32 {
    DBC_INDEX_START,
    DBC_INDEX_A,
    DBC_INDEX_B,
    DBC_INDEX_C,
    DBC_INDEX_D,
    DBC_INDEX_E
};

static const std::vector<DbcConfigFormat> g_dbc_config = {
      DbcConfigFormat("a.dbc", "12345", "https", 0xE260, 0, 0, 1),
      DbcConfigFormat("b.dbc", "12345", "https", 0xE261, 0, 0, 1),
      DbcConfigFormat("c.dbc", "12345", "https", 0xE262, 0, 0, 1),
      DbcConfigFormat("d.dbc", "12345", "https", 0xE263, 0, 0, 1),
      DbcConfigFormat("e.dbc", "12345", "https", 0xE264, 0, 0, 1)
};

// DBC文件名规则
static const std::unordered_map< DBC_INDEX, std::string > g_txt_name_map = {
    { DBC_INDEX_A , "a_config.txt"},
    { DBC_INDEX_B , "b_config.txt" },
    { DBC_INDEX_C , "c_config.txt"},
    { DBC_INDEX_D , "d_config.txt" },
    { DBC_INDEX_E , "e_config.txt"}
};

static const std::unordered_map<std::string , DBC_INDEX > g_name_index_map = {
    {"a.dbc",  DBC_INDEX_A },
    {"b.dbc",  DBC_INDEX_B },
    {"c.dbc",  DBC_INDEX_C },
    {"d.dbc",  DBC_INDEX_D },
    {"e.dbc",  DBC_INDEX_E }
};

static const std::unordered_map<DBC_INDEX, std::string > g_index_name_map = {
    {DBC_INDEX_A , "a.dbc" },
    {DBC_INDEX_B , "b.dbc" },
    {DBC_INDEX_C , "c.dbc" },
    {DBC_INDEX_D , "d.dbc" },
    {DBC_INDEX_E , "e.dbc" }
};

static const std::unordered_map<DBC_INDEX, DbcConfigFormat> g_dbc_default = {
    { DBC_INDEX_A , DbcConfigFormat("a.dbc" , "12345" , "https", 0xE260 , 0, 0, 1) },
    { DBC_INDEX_B , DbcConfigFormat("b.dbc" , "12345" , "https", 0xE261 , 0, 0, 1) },
    { DBC_INDEX_C , DbcConfigFormat("c.dbc" , "12345" , "https", 0xE262 , 0, 0, 1) },
    { DBC_INDEX_D,  DbcConfigFormat("d.dbc" , "12345" , "https", 0xE263 , 0, 0, 1) },
    { DBC_INDEX_E , DbcConfigFormat("e.dbc" , "12345" , "https", 0xE264 , 0, 0, 1) }
};

typedef struct
{
    SMLK_UINT16 upload_id;
    SMLK_UINT16 collect_interval;
    SMLK_UINT16 upload_interval;
    SMLK_UINT8  data_type;
} dbc_config_struct;

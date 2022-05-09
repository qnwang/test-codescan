#ifndef _CONFIG_LOADER_H_
#define _CONFIG_LOADER_H_
#include <smlk_types.h>
#include "SignalFormat.h"
#include "DynamicCollection.h"
#include "DbcConfigFormat.h"
//#include <message_inf.h>
//#include <smlk_log.h>

class ConfigLoader
{
  public:
    static void NewLoadConfigFromDBCFile(DBC_INDEX &dbc_index, std::unordered_map<DBC_INDEX, std::vector<SPN_INDEX>> *dbc_sample_signals_map, MessageDecodeMap *message_decode_map);
    static bool CheckDbcValid(DBC_INDEX &dbc_index);
    static void LoadConfigFromDBCFile(MessageDecodeMap *message_decode_map);
    static std::string GetDbcPath(DBC_INDEX &dbc_index);
  private:
    static SMLK_UINT32 GetDBCVersion();

};

#endif
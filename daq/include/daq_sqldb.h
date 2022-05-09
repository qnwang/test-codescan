


#pragma once
#include <tuple>
#include <vector>
#include <unordered_map>
#include "smlk_sqlite.h"
#include "smlk_types.h"
#include "smlk_error.h"
#define DAQ_SQL_FILE_PATH "/userdata/smartlink/record/database/"     /*sql文件存储位置*/
namespace smartlink {

class DaqSqlDB : public BaseSqlite{
public:
    enum Flags : SMLK_UINT8 {
        success     = 0x00,
        failed      = 0x01
    };
public:
    DaqSqlDB();
    DaqSqlDB(IN std::string &dir, IN std::string &db="record_daq.sqlite3");
    DaqSqlDB(IN DaqSqlDB &) = delete;
    DaqSqlDB(DaqSqlDB &&) = delete;
    ~DaqSqlDB();

    DaqSqlDB & operator=(IN DaqSqlDB &) = delete;

    SMLK_RC Init() override;
    void upgrade() override{};

    using Record = std::tuple<SMLK_UINT32, SMLK_UINT8, std::vector<SMLK_UINT8>>;

    SMLK_RC Insert(IN SMLK_UINT32 time_id, IN SMLK_UINT8 data[], IN std::size_t sz);
    SMLK_RC Insert(IN SMLK_UINT32 time_id, IN std::vector<SMLK_UINT8> &data);
    SMLK_RC Update(IN SMLK_UINT32 time_id, IN SMLK_UINT8 flag);
    SMLK_RC Update(IN std::vector<SMLK_UINT32> &ids, IN SMLK_UINT8 flag);
    SMLK_RC Query(IN SMLK_UINT32 time_id, SMLK_UINT8 &flag,  SMLK_UINT8 data[], std::size_t &sz);
    SMLK_RC Query(IN SMLK_UINT32 time_id, SMLK_UINT8 &flag, std::vector<SMLK_UINT8> &data);
    SMLK_RC Delete(IN SMLK_UINT32 time_id);
    SMLK_RC QueryWhereLessTimeId(IN SMLK_UINT32 time_id, std::vector<std::tuple<SMLK_UINT16, std::vector<SMLK_UINT8>>> &records, std::size_t limit=32);
    SMLK_RC QueryAllInfo(std::vector<std::tuple<SMLK_UINT32,  std::vector<SMLK_UINT8>>> &records);

    const std::string & LastError() const;

private:
    void LastError(IN int rc, IN char *err);

private:

    struct {
        int         rc;
        std::string desc;
    }               m_last_error;
};

};
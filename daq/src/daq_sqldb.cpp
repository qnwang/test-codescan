#include "daq_sqldb.h"
#include <sqlite3.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <vector>
#include "Poco/Format.h"
#include "Poco/Path.h"
#include "smlk_log.h"

using namespace std;
using namespace smartlink;
using namespace Poco::Data::Keywords;

static const std::unordered_map<std::string, std::string>   tables = {
    {
        "records",
        "CREATE TABLE IF NOT EXISTS records("
            "time_id     INT     PRIMARY KEY     NOT NULL,"
            "data        BLOB"
        ")"
    }
};

DaqSqlDB::DaqSqlDB()
    :DaqSqlDB(DAQ_SQL_FILE_PATH, "records_daq.sqlite3")
{}

DaqSqlDB::DaqSqlDB(IN std::string &dir, IN std::string &db)
    : BaseSqlite(dir + "/" + db)
{}

DaqSqlDB::~DaqSqlDB()
{
}

SMLK_RC DaqSqlDB::Init()
{
    std::string mountpoint(DAQ_SQL_FILE_PATH);
    if (0 != access(mountpoint.c_str(), 0))
    {
        for(SMLK_UINT16 i = 0; i < mountpoint.size(); ++i){
            if (*(char *)(mountpoint.data()+i) == '/'){
                string path;
                path.insert(path.end(), (mountpoint.begin()), (mountpoint.begin()+i+1));
                if (0 != access(path.c_str(), 0)) {
                    if(mkdir(path.c_str(), 0555) != 0) {
                        SMLK_LOGD("[sqldebug] Can't mkdir [ %s ]", path.c_str());
                        SMLK_RC::RC_OK ;
                    } else {
                        SMLK_LOGD("[sqldebug] mkdir : [ %s ]", path.c_str());
                    }
                }
            }
        }
    } else {
        SMLK_LOGD("[sqldebug][ %s ] already exist !", mountpoint.c_str());
    }
    try {
        Poco::Data::Session session(m_sessionPool->get());
        if (!session.isConnected()) {
            SMLK_LOGE("[T] session is not  connected");
            return SMLK_RC::RC_ERROR;
        }
        for (auto const &pair : tables) {
            session << pair.second, now;
        }

    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] exception error:%s", e.displayText().c_str());
        LastError(e.code(), e.displayText().c_str());
        return SMLK_RC::RC_ERROR;
    }

    return SMLK_RC::RC_OK;
}

SMLK_RC DaqSqlDB::Insert(IN SMLK_UINT32 time_id, IN SMLK_UINT8 data[], IN std::size_t sz)
{
    try {
        Poco::Data::Session session(m_sessionPool->get());
        if (!session.isConnected()) {
            SMLK_LOGE("[T] session is not  connected");
            return SMLK_RC::RC_ERROR;
        }
        Poco::Data::BLOB dataBLob((unsigned char *)data, sz);
        session << "INSERT INTO records(time_id, data) VALUES (?, ?)", useRef(time_id), use(dataBLob), now;

    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] exception error:%s", e.displayText().c_str());
        LastError(e.code(), e.displayText().c_str());
        return SMLK_RC::RC_ERROR;
    }

    return SMLK_RC::RC_OK;
}

SMLK_RC DaqSqlDB::Insert(IN SMLK_UINT32 time_id, IN std::vector<SMLK_UINT8> &data)
{
    return Insert(time_id,  data.data(), data.size());
}

SMLK_RC DaqSqlDB::Update(IN SMLK_UINT32 time_id, IN SMLK_UINT8 flag)
{
    try {
        Poco::Data::Session session(m_sessionPool->get());
        if (!session.isConnected()) {
            SMLK_LOGE("[T] session is not  connected");
            return SMLK_RC::RC_ERROR;
        }
        session << "UPDATE records SET flag = ? WHERE time_id = ?", useRef(flag), useRef(time_id), now;

    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] exception error:%s", e.displayText().c_str());
        LastError(e.code(), e.displayText().c_str());
        return SMLK_RC::RC_ERROR;
    }

    return SMLK_RC::RC_OK;
}

SMLK_RC DaqSqlDB::Delete(IN SMLK_UINT32 time_id)
{
    try {
        Poco::Data::Session session(m_sessionPool->get());
        if (!session.isConnected()) {
            SMLK_LOGE("[T] session is not  connected");
            return SMLK_RC::RC_ERROR;
        }
        session << "DELETE FROM records WHERE time_id = ?", useRef(time_id), now;
        

    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] exception error:%s", e.displayText().c_str());
        LastError(e.code(), e.displayText().c_str());
        return SMLK_RC::RC_ERROR;
    }

    return SMLK_RC::RC_OK;
}

SMLK_RC DaqSqlDB::QueryAllInfo(std::vector<std::tuple<SMLK_UINT32, std::vector<SMLK_UINT8>>> &records)
{
    try {
        Poco::Data::Session session(m_sessionPool->get());
        if (!session.isConnected()) {
            SMLK_LOGE("[T] session is not  connected");
            return SMLK_RC::RC_ERROR;
        }
        records.clear();
        Poco::Data::Statement stmt = (session << "SELECT time_id, data FROM records", now);
        Poco::Data::RecordSet rset(stmt);
        for (auto it = rset.begin(); it != rset.end(); ++it) {
            auto time_id = it->get(0).convert<SMLK_UINT32>();
            auto data = it->get(1).convert<std::string>();
            records.emplace_back(time_id, std::vector<SMLK_UINT8>(data.begin(), data.end()));
        }

    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] exception error:%s", e.displayText().c_str());
        LastError(e.code(), e.displayText().c_str());
        return SMLK_RC::RC_ERROR;
    }

    return SMLK_RC::RC_OK;
}

const std::string & DaqSqlDB::LastError() const
{
    return m_last_error.desc;
}

void DaqSqlDB::LastError(IN int rc, IN char *err)
{
    m_last_error.rc     = rc;
    m_last_error.desc   = err;
}
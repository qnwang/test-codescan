#include <string>
#include <unordered_map>
#include <sstream>
#include <iterator>
#include <cstring>
#include <recorder.h>
#include <sqlite3.h>

using namespace smartlink;

Recorder::Recorder()
    : m_dir(".")
    , m_db("records_17691.sqlite3")
    , m_dbh(nullptr)
{}

Recorder::Recorder(IN std::string &dir, IN std::string &db)
    : m_dir(dir)
    , m_db(db)
    , m_dbh(nullptr)
{}

Recorder::~Recorder()
{
    Close();
}

/******************************************************************************
 * NAME: Init
 *
 * DESCRIPTION: Init function for the recorder module, you may finish all
 *              the initialization work here
 *
 * PARAMETERS:
 *
 * RETURN:
 *      SL_SUCCESS on success, and for now no error will return
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Init()
{
    auto rc = sqlite3_initialize();
    if ( SQLITE_OK != rc ) {
        LastError(rc, "Fail to initialize the sqlite3");
        return SL_ELIBINIT;
    }

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Open
 *
 * DESCRIPTION: Open function for the recorder module, it will try to create the
 *              table if it is not exists
 *
 * PARAMETERS:
 *
 * RETURN:
 *      SL_SUCCESS on success, and for now no error will return
 *
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Open()
{
    if ( nullptr != m_dbh ) {
        return SL_SUCCESS;
    }

    if ( m_dir.back() == '/' ) {
        m_dir.pop_back();
    }

    auto path = m_dir + "/" + m_db;
    sqlite3 *dbh = nullptr;

    auto rc = sqlite3_open_v2(path.c_str(), &dbh, SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE, nullptr);
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    static const std::unordered_map<std::string, std::string>   tables = {
        {
            "records",
            "CREATE TABLE IF NOT EXISTS records("
                "_id    INTEGER     PRIMARY KEY    ,"
                "id     INTEGER                     NOT NULL,"
                "flag   INTEGER                     DEFAULT 0,"
                "data   BLOB"
            ")"
        }
    };

    char *  err_msg = nullptr;
    for ( auto const &pair : tables ) {
        rc = sqlite3_exec(dbh, pair.second.c_str(), nullptr, nullptr, &err_msg);
        if ( SQLITE_OK != rc ) {
            printf("SQL error: %s\n", err_msg);
            LastError(rc, err_msg);
            sqlite3_free(err_msg);
            return SL_EFAILED;
        }
    }

    m_dbh = static_cast<void *>(dbh);

    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Close
 *
 * DESCRIPTION: close the db file
 *
 * PARAMETERS:
 *
 * RETURN:
 *
 * NOTE:
 *****************************************************************************/
void Recorder::Close()
{
    if ( nullptr != m_dbh ) {
        sqlite3_close(static_cast<sqlite3 *>(m_dbh));
        m_dbh = nullptr;
    }
}

/******************************************************************************
 * NAME: Insert
 *
 * DESCRIPTION: insert a new record into the db file
 *
 * PARAMETERS:
 *      id:     recorder id
 *      flag:   recorder flag
 *      data:   bytes pointer
 *      sz:     data length
 *
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     the new record inserted successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Insert(IN SMLK_UINT32 id, IN SMLK_UINT8 flag, IN SMLK_UINT8 data[], IN std::size_t sz)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "INSERT INTO records(id, flag, data) VALUES (?, ?, ?)",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    try {
        rc = sqlite3_bind_int64(stmt, 1, id);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_bind_int(stmt, 2, flag);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_bind_blob(stmt, 3, data, sz, SQLITE_STATIC);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        if ( SQLITE_DONE != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, err.what());
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Insert
 *
 * DESCRIPTION: insert a new record into the db file
 *
 * PARAMETERS:
 *      id:     recorder id
 *      flag:   recorder flag
 *      data:   bytes vector
 *
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     the new record inserted successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Insert(IN SMLK_UINT32 id, IN SMLK_UINT8 flag, IN std::vector<SMLK_UINT8> &data)
{
    return Insert(id, flag, data.data(), data.size());    
}

/******************************************************************************
 * NAME: Update
 *
 * DESCRIPTION: update the flag of a record with id
 *
 * PARAMETERS:
 *      id:     recorder id
 *      flag:   recorder flag
 *
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     the record updated successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Update(IN SMLK_UINT32 id, IN SMLK_UINT8 flag)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "UPDATE records SET flag = ? WHERE id = ?",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    try {
        rc = sqlite3_bind_int(stmt, 1, flag);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_bind_int64(stmt, 2, id);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        if ( SQLITE_DONE != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: Update
 *
 * DESCRIPTION: update the flag of record vector
 *
 * PARAMETERS:
 *      ids:    recorder id vector
 *      flag:   recorder flag
 *
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     records updated successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Update(IN std::vector<SMLK_UINT32> &ids, IN SMLK_UINT8 flag)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    if ( 0 == ids.size() ) {
        return SL_SUCCESS;
    }

    auto rc = sqlite3_prepare_v2(dbh,
            "UPDATE records SET flag = ? WHERE id IN (?)",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    std::ostringstream  oss;
    std::copy(ids.begin(), ids.end()-1, std::ostream_iterator<std::uint32_t>(oss, ","));
    oss << ids.back();

    try {
        rc = sqlite3_bind_int(stmt, 1, flag);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_bind_text(stmt, 2, oss.str().c_str(), oss.str().length(), SQLITE_STATIC);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        if ( SQLITE_DONE != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;

}

/******************************************************************************
 * NAME: Query
 *
 * DESCRIPTION: query a record via id
 *
 * PARAMETERS:
 *      id:     reocord id
 *      flag:   recorder flag
 *      data:   data buffer
 *      sz:     buffer size, and it will be set to the valid data size stored
 *              in the buffer
 *
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EEXISTS:     record with the specified id not exists
 *  SL_OVERFLOW:    buffer too smal to sore the record data
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     record queried successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Query(IN SMLK_UINT32 id, SMLK_UINT8 &flag,  SMLK_UINT8 data[], std::size_t &sz)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "SELECT flag, data FROM records WHERE id = ?",
            -1,
            &stmt,
            nullptr
         );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    auto rt = SL_SUCCESS;

    try {
        rc = sqlite3_bind_int64(stmt, 1, id);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        if ( SQLITE_DONE == rc ) {
            rt = SL_EEXISTS;
        } else if ( SQLITE_ROW != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        } else {
            flag                = (SMLK_UINT8)sqlite3_column_int(stmt, 0);
            auto len            = sqlite3_column_bytes(stmt, 1);

            if ( std::size_t(len) > sz ) {
                rt = SL_OVERFLOW;
            } else {
                std::uint8_t *  raw = (std::uint8_t *)sqlite3_column_blob(stmt, 1);
                std::memcpy(data, raw, len);
                sz = std::size_t(len);
            }
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return rt;    
}

/******************************************************************************
 * NAME: Query
 *
 * DESCRIPTION: query a record via id
 *
 * PARAMETERS:
 *      id:     record id
 *      flag:   recorder flag
 *      data:   vector to store the data
 *
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EEXISTS:     record with the specified id not exists
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     record queried successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Query(IN SMLK_UINT32 id, SMLK_UINT8 &flag, std::vector<SMLK_UINT8> &data)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "SELECT flag, data FROM records WHERE id = ?",
            -1,
            &stmt,
            nullptr
         );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    data.clear();

    auto rt = SL_SUCCESS;
    try {
        rc = sqlite3_bind_int64(stmt, 1, id);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        if ( SQLITE_DONE == rc ) {
            rt = SL_EEXISTS;
        } else if ( SQLITE_ROW != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        } else {
            flag                = (SMLK_UINT8)sqlite3_column_int(stmt, 0);
            auto len            = sqlite3_column_bytes(stmt, 1);
            std::uint8_t *  raw = (std::uint8_t *)sqlite3_column_blob(stmt, 1);

            data.insert(data.begin(), raw, raw+len);
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return rt;

}

/******************************************************************************
 * NAME: Delete
 *
 * DESCRIPTION: delete a record via id
 *
 * PARAMETERS:
 *      id:     record id
 *
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     record queried successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Delete(IN SMLK_UINT32 id)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "DELETE FROM records WHERE id = ?",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    try {
        rc = sqlite3_bind_int64(stmt, 1, id);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        if ( SQLITE_DONE != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;

}

/******************************************************************************
 * NAME: Delete
 *
 * DESCRIPTION: delete records via ids
 *
 * PARAMETERS:
 *      ids:    record id list
 *
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     record queried successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::Delete(IN std::vector<SMLK_UINT32> &ids)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    if ( 0 == ids.size() ) {
        return SL_SUCCESS;
    }

    auto rc = sqlite3_prepare_v2(dbh,
            "DELETE FROM records WHERE id IN (?)",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    std::ostringstream  oss;
    std::copy(ids.begin(), ids.end()-1, std::ostream_iterator<std::uint32_t>(oss, ","));
    oss << ids.back();

    try {
        rc = sqlite3_bind_text(stmt, 1, oss.str().c_str(), oss.str().length(), SQLITE_STATIC);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        if ( SQLITE_DONE != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: QueryTotal
 *
 * DESCRIPTION:     query total records count
 *
 * PARAMETERS:
 *      total:      total count
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     record queried successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::QueryTotal(SMLK_UINT32 &total)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "SELECT COUNT(1) FROM records",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    try {
        rc = sqlite3_step(stmt);
        if ( SQLITE_ROW != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        } else {
            total = (std::uint32_t)sqlite3_column_int64(stmt, 0);
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: DeleteLessThan
 *
 * DESCRIPTION:     delete records where id less than specified id (not include id)
 *
 * PARAMETERS:
 *        id:       specified id
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     record queried successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::DeleteLessThan(IN SMLK_UINT32 id)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "DELETE FROM records WHERE id < ?",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    try {
        rc = sqlite3_bind_int64(stmt, 1, id);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        if ( SQLITE_DONE != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: DeleteAhead
 *
 * DESCRIPTION:     delete ahead n records in the table
 *
 * PARAMETERS:
 *         n:       records count
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     record queried successfully
 * NOTE:
 *  this function will try to delete the ahead n records in the tables.
 *****************************************************************************/
SMLK_UINT32 Recorder::DeleteAhead(IN SMLK_UINT32 n)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "DELETE FROM records WHERE rowid IN (SELECT rowid FROM records limit ?)",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    try {
        rc = sqlite3_bind_int64(stmt, 1, n);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        if ( SQLITE_DONE != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: QueryOneRecord
 *
 * DESCRIPTION:     query one record
 *
 * PARAMETERS:
 *      records:    vector to store the queried records
 *      id_from:    min id
 *      id_to:      max_id
 *      limit:      limit max number of queried records
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     record queried successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::QueryOneRecord(std::vector<std::tuple<SMLK_UINT32, std::vector<SMLK_UINT8>>> &records, SMLK_UINT32 id_from, SMLK_UINT32 id_to, std::size_t limit)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "SELECT id, data FROM records WHERE id BETWEEN ? AND ? LIMIT ?",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    records.clear();

    try {
        rc = sqlite3_bind_int64(stmt, 1, id_from);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_bind_int64(stmt, 2, id_to);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_bind_int(stmt, 3, limit);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        while ( SQLITE_ROW == rc ) {
            SMLK_UINT32     id      = (std::uint32_t)sqlite3_column_int64(stmt, 0);
            auto            len     = sqlite3_column_bytes(stmt, 1);
            std::uint8_t *  raw     = (std::uint8_t *)sqlite3_column_blob(stmt, 1);            

            records.emplace_back(id, std::vector<SMLK_UINT8>(raw, raw+len));
            rc = sqlite3_step(stmt);
        }

        if ( SQLITE_DONE != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: QueryWhereFlag
 *
 * DESCRIPTION:     query records with specified flag
 *
 * PARAMETERS:
 *      flag:       specified flag
 *      records:    vector to store the queried records
 *      id_from:    min id
 *      id_to:      max_id
 *      limit:      limit max number of queried records
 * RETURN:
 *  SL_ESUPPORT:    not initlized, operation not support
 *  SL_EFAILED:     failed, you may get the detailed information via LastError
 *  SL_SUCCESS:     record queried successfully
 * NOTE:
 *****************************************************************************/
SMLK_UINT32 Recorder::QueryWhereFlag(IN SMLK_UINT8 flag, std::vector<std::tuple<SMLK_UINT32, std::vector<SMLK_UINT8>>> &records, SMLK_UINT32 id_from, SMLK_UINT32 id_to, std::size_t limit)
{
    if ( nullptr == m_dbh ) {
        return SL_ESUPPORT;
    }

    sqlite3 *       dbh = static_cast<sqlite3 *>(m_dbh);
    sqlite3_stmt *  stmt = nullptr;

    auto rc = sqlite3_prepare_v2(dbh,
            "SELECT id, data FROM records WHERE flag = ? AND id BETWEEN ? AND ? LIMIT ?",
            -1,
            &stmt,
            nullptr
        );
    if ( SQLITE_OK != rc ) {
        LastError(rc, sqlite3_errmsg(dbh));
        return SL_EFAILED;
    }

    records.clear();

    try {
        rc = sqlite3_bind_int(stmt, 1, flag);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_bind_int64(stmt, 2, id_from);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_bind_int64(stmt, 3, id_to);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_bind_int(stmt, 4, limit);
        if ( SQLITE_OK != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }

        rc = sqlite3_step(stmt);
        while ( SQLITE_ROW == rc ) {
            SMLK_UINT32     id      = (std::uint32_t)sqlite3_column_int64(stmt, 0);
            auto            len     = sqlite3_column_bytes(stmt, 1);
            std::uint8_t *  raw     = (std::uint8_t *)sqlite3_column_blob(stmt, 1);            

            records.emplace_back(id, std::vector<SMLK_UINT8>(raw, raw+len));
            rc = sqlite3_step(stmt);
        }

        if ( SQLITE_DONE != rc ) {
            throw std::runtime_error(sqlite3_errmsg(dbh));
        }
    } catch (const std::runtime_error &err) {
        LastError(rc, sqlite3_errmsg(dbh));
        sqlite3_finalize(stmt);
        return SL_EFAILED;
    }

    sqlite3_finalize(stmt);
    return SL_SUCCESS;
}

/******************************************************************************
 * NAME: LastError
 *
 * DESCRIPTION:     get the last error descrption
 *
 * PARAMETERS:
 * 
 * RETURN:
 *                  last error descrption
 * NOTE:
 *****************************************************************************/
const std::string & Recorder::LastError() const
{
    return m_last_error.desc;
}

/******************************************************************************
 * NAME: LastError
 *
 * DESCRIPTION:     set the last error code and descrption
 *
 * PARAMETERS:
 *      rc:         error code
 *      err:        error string
 * 
 * RETURN:
 * 
 * NOTE:
 *****************************************************************************/
void Recorder::LastError(IN int rc, IN char *err)
{
    m_last_error.rc     = rc;
    m_last_error.desc   = err;
}
#pragma once
#include "dyc_util.h"
#define default_name "dbc"
#define default_md5  "12345"
#define default_url  "https"
class DbcConfigFormat {

public:
    DbcConfigFormat(std::string m_dbc_name,
                    std::string m_dbc_md5sum,
                    std::string m_dbc_http_url,
                    SMLK_UINT16 m_upload_id,
                    SMLK_UINT16 m_collection_interval,
                    SMLK_UINT16 m_upload_interval,
                    SMLK_UINT8  m_data_type
                    )
    : m_dbc_name(default_name)
    , m_dbc_md5sum(default_md5)
    , m_dbc_http_url(default_url)
    , m_upload_id(0)
    , m_collection_interval(0)
    , m_upload_interval(0)
    , m_data_type(1)
    {
    }
    DbcConfigFormat(const DbcConfigFormat& other)
    : m_dbc_name(other.m_dbc_name)
    , m_dbc_md5sum(other.m_dbc_md5sum)
    , m_dbc_http_url(other.m_dbc_http_url)
    , m_upload_id(other.m_upload_id)
    , m_collection_interval(other.m_collection_interval)
    , m_upload_interval(other.m_upload_interval)
    , m_data_type(other.m_data_type)
    {
    }
    ~DbcConfigFormat();
    void setDbcName(IN std::string &name);
    std::string  getDbcName();
    void setHttpUrl(IN std::string &url);
    std::string  getHttpUrl();
    void setUploadId(IN SMLK_UINT16 &uploadId);
    void setFileMd5(IN std::string &md5);
    std::string  getFileMd5();
    SMLK_UINT16  getUploadId();
    void setCollectionTnterval(IN SMLK_UINT16 &collectInterval);
    SMLK_UINT16  getCollectionTnterval();
    void setUploadInterval(IN SMLK_UINT16 &uploadInterval);
    SMLK_UINT16  getUploadInterval();
    void setUploadDataType(IN SMLK_UINT8 &type);
    SMLK_UINT8  getUploadDataType();
private:
    std::string m_dbc_name;
    std::string m_dbc_md5sum;
    std::string m_dbc_http_url;
    SMLK_UINT16 m_upload_id;
    SMLK_UINT16 m_collection_interval;
    SMLK_UINT16 m_upload_interval;
    SMLK_UINT8   m_data_type; // phy:1 raw:2
};
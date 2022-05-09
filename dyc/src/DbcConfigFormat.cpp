#include "DbcConfigFormat.h"

DbcConfigFormat::~DbcConfigFormat()
{

}
void DbcConfigFormat::setDbcName(IN std::string &name)
{
    m_dbc_name = name;
}

std::string DbcConfigFormat::getDbcName()
{
    return m_dbc_name;
}

void DbcConfigFormat::setFileMd5(IN std::string &md5)
{
    m_dbc_md5sum = md5;
}

std::string  DbcConfigFormat::getFileMd5()
{
    return m_dbc_md5sum;
}

void DbcConfigFormat::setHttpUrl(IN std::string &url)
{
    m_dbc_http_url = url;
}

std::string  DbcConfigFormat::getHttpUrl()
{
    return m_dbc_http_url;
}

void DbcConfigFormat::setUploadId(IN SMLK_UINT16 &uploadId)
{
    m_upload_id = uploadId;
}

SMLK_UINT16  DbcConfigFormat::getUploadId()
{
    return m_upload_id;
}

void DbcConfigFormat::setCollectionTnterval(IN SMLK_UINT16 &collectInterval)
{
  m_collection_interval = collectInterval;
}

SMLK_UINT16  DbcConfigFormat::getCollectionTnterval()
{
  return m_collection_interval;
}

void DbcConfigFormat::setUploadInterval(IN SMLK_UINT16 &uploadInterval)
{
  m_upload_interval = uploadInterval;
}

SMLK_UINT16  DbcConfigFormat::getUploadInterval()
{
  return m_upload_interval;
}

void DbcConfigFormat::setUploadDataType(IN SMLK_UINT8 &type)
{
  m_data_type =  type;
}

SMLK_UINT8  DbcConfigFormat::getUploadDataType()
{
  return m_data_type;
}

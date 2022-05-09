/******************************************************************************
*
*  Copyright (C) 2021 SmartLink
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

#include <smlk_log.h>
#include <smlk_tools.h>

#include "sqllite_32960.h"

#include <sys/time.h>

using namespace Poco::Data::Keywords;

SqlLiteBase::SqlLiteBase()
    : BaseSqlite(M_GB_32960_DATABASE_FILE_DIR) {}

SqlLiteBase::~SqlLiteBase() {
}

/**
 * @brief 打开/新建数据库（如数据库文件已经存在 则不create表）
 */
SMLK_RC SqlLiteBase::Init() {
    InitTable();
    return SMLK_RC::RC_OK;
}

/**
 * @brief 进程一加载 或则 ign on？初始化相关的表,删除多余的表，每天新增一张表
 */
void SqlLiteBase::InitTable() {
    SMLK_LOGD("[T] SqlLiteBase InitTable");
    int ret = -1;
    bool checkIsExist = false;

    //新建一张当前日志的表 如tb20200407 存储在变量m_todayTable
    this->SetCurTabName();
    SMLK_LOGD("[T] SqlLiteBase m_todayTable: %s", m_todayTable.c_str());
    this->GetTables(true);
    //判断当前的表是否已经创建
    std::vector<std::string>::iterator iter;
    iter = find(m_listTableDesc.begin(), m_listTableDesc.end(), m_todayTable);
    if (iter != m_listTableDesc.end()) {
        checkIsExist = true;
    }

    try {
        Poco::Data::Session session(m_sessionPool->get());
        if (!session.isConnected()) {
            SMLK_LOGE("[T] session is not  connected");
            return;
        }
        //如果当天表不存在则新增表
        if (!checkIsExist) {
            session << "create table if not exists %s (currentDate text PRIMARY KEY,sendMsg blob,sendState int)", m_todayTable, now;
            SMLK_LOGD("[T] SqlLiteBase create tableName: %s", m_todayTable.c_str());
        }
        //获取数据库中的表名，降序的形式,方便删除 存储在变量 m_listTableDesc(如 20200407 20200406 20200405)
        this->GetTables(true);

        //删除多余的表
        if (m_listTableDesc.size() > M_GB_32960_DATABASE_SAVEDAY) {
            int maxLength = m_listTableDesc.size();
            for (int i = M_GB_32960_DATABASE_SAVEDAY; i < maxLength; i++) {
                session << "drop table IF EXISTS %s;", m_listTableDesc.at(i), now;
                SMLK_LOGD("[T] SqlLiteBase del more table protocol:%s,ret:%d,tableName: %s", m_protocolName.c_str(), ret,
                          m_listTableDesc.at(i).c_str());
            }
        }

        //删除多余的表重新获取当前最新的表
        this->GetTables(true);

        //是对m_listTableDesc的 反转 作用是方便从索引0开始找最老时间的数据
        m_listTableAsc.clear();
        m_listTableAsc = m_listTableDesc;
        reverse(m_listTableAsc.begin(), m_listTableAsc.end());

        for (int i = 0; i < m_listTableAsc.size(); i++) {
            SMLK_LOGD("[T]  SqlLiteBase::m_listTableAsc  %s ", m_listTableAsc[i].c_str());
        }
        //找到最旧的需要 发送的数据表
        int nrow, maxlength = 0;
        maxlength = m_listTableAsc.size();

        for (int j = 0; j < maxlength; j++) {
            Poco::Data::Statement stmt = (session << "select currentDate from %s  where sendState=0 order by currentDate asc limit 1;", m_listTableAsc.at(j), now);
            SMLK_LOGD("select currentDate sql : %s", stmt.toString().c_str());
            Poco::Data::RecordSet rset(stmt);
            nrow = rset.rowCount();
            SMLK_LOGD("[T] SqlLiteBase foreach table from old to new, tablename is %s, ret %d, nrow %d", m_listTableAsc.at(j).c_str(),
                      ret, nrow);
            if (nrow > 0) {
                m_readingTable = m_listTableAsc.at(j);
                SMLK_LOGD("[T]  SqlLiteBase::InitTable() %s ", m_readingTable.c_str());
                break;
            }
        }

        SMLK_LOGD("[T] SqlLiteBase InitTable nrow=%d", nrow);

        if (nrow <= 0)  //未找到
        {
            m_readingTable = m_todayTable;
        }
    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] SqlLiteBase::InitTable() error:%s", e.displayText().c_str());
    }
}

/**
 * @brief 记录当天所需要处理的数据表
 */
void SqlLiteBase::SetCurTabName() {
    time_t timep;
    struct tm *p;
    time( &timep );
    p = localtime( &timep );
    std::stringstream strStream;
    strStream<<(p->tm_year + 1900)<<std::setw(2)<<std::setfill('0')<<(p->tm_mon + 1)<<std::setw(2)<<std::setfill('0')<<p->tm_mday;

    m_todayTable = "tb";
    m_todayTable += strStream.str();
}

/**
 * @brief 查询出所有的表
 */
void SqlLiteBase::GetTables(bool isDesc) {
    SMLK_LOGD("[T] SqlLiteBase GetTables");
    //查询表
    m_listTableDesc.clear();
    try {
        std::string order = isDesc ? "desc" : "asc";
        Poco::Data::Session session(m_sessionPool->get());
        session << "select name from sqlite_master where type ='table' order by name %s", order, into(m_listTableDesc), now;
        for (auto &&table : m_listTableDesc) {
            SMLK_LOGD("[T] SqlLiteBase::GetTables() table name:%s", table.c_str());
        }
    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] SqlLiteBase::GetTables() error:%s", e.displayText().c_str());
    }
}

/**
 * @brief 盲区数据的写入
 * @param buf   整包消息内容
 * @param len   消息长度
 */
void SqlLiteBase::WriteBlindData(char *buf, int len) {
    time_t timep;
    struct tm *p;
    time( &timep );
    p = localtime( &timep );
    std::stringstream strStream;
    struct timeval tpend;
    int milliseconds = 0;

    //获取毫秒,电池包大于200时会发送多包数据需要精确到毫秒
    gettimeofday(&tpend, NULL);
    milliseconds = tpend.tv_usec/1000;
    
    strStream<<(p->tm_year + 1900)<<"-"<<std::setw(2)<<std::setfill('0')<<(p->tm_mon + 1)<<"-"<<std::setw(2)<<std::setfill('0')<<p->tm_mday \
        <<"_"<<std::setw(2)<<std::setfill('0')<<p->tm_hour<<":"<<std::setw(2)<<std::setfill('0')<<p->tm_min<<":"<<std::setw(2)<<std::setfill('0')<<p->tm_sec \
        <<"."<<std::setw(2)<<std::setfill('0')<<milliseconds;

    std::string date = strStream.str();

    try {
        // SMLK_LOGD("[T] SqlLiteBase::WriteBlindData() date:%s data:%s sendState:%d len:%d", date.c_str(), buf, 0, len);
        SMLK_LOGD("[T] SqlLiteBase::WriteBlindData() sendState:%d, len:%d", 0, len);
        Poco::Data::BLOB dataBLob((unsigned char *)buf, len);
        Poco::Data::Session session(m_sessionPool->get());
        session << "insert into %s values (?, ?, ?);", m_todayTable, use(date), use(dataBLob), bind(0), now;
    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] SqlLiteBase::WriteBlindData() error:%s", e.displayText().c_str());
    }
}

/**
 * @brief 读取盲区最旧一条消息数据
 * @param buf   整包消息内容
 * @param len   消息长度
 */
void SqlLiteBase::ReadBlindData(char *buf, int &len) {
    // SMLK_LOGD("[T] SqlLiteBase::ReadBlindData()");
    int ncounts = 0;
    if (m_readingTable == "") {
        //SMLK_LOGD("[T] no any data need to resend");
        return;
    }

    try {
        Poco::Data::Session session(m_sessionPool->get());
        Poco::Data::Statement stmt = (session << "select currentDate,sendMsg from %s  where sendState=0 ORDER BY currentDate ASC limit 1;", m_readingTable, now);
        Poco::Data::RecordSet rset(stmt);
        ncounts = rset.rowCount();
        if (ncounts == 0)  //找下一张表
        {
            if (m_readingTable != m_todayTable) {
                this->FindNextTable();
                //SMLK_LOGD("[T] ReadBlindData protocol:%s,  no any ncols",m_protocolName.c_str());
            } else {
                //SMLK_LOGD("[T] no any date need to send ReadBlindData protocol:%s,  no any ncols",m_protocolName.c_str());
            }
            len = 0;
            return;
        } else {
            m_sendMsgDate = rset[0].convert<std::string>();             //获取第一个字段
            const char *data = rset[1].convert<std::string>().c_str();  //得到纪录中的BLOB字段
            len = rset[1].convert<std::string>().length();  //得到纪录中的BLOB字段
            memcpy(buf, data, len);
            // SMLK_LOGD("[T] SqlLiteBase::ReadBlindData() data:%s len:%d", data, len);
        }
    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] SqlLiteBase::ReadBlindData() error:%s", e.displayText().c_str());
    }
}

/**
 * @brief 找下一张需要重发的表明
 */
void SqlLiteBase::FindNextTable() {
    int index, maxLength = -1;
    maxLength = m_listTableAsc.size();
    std::vector<std::string>::iterator ite1 = find(m_listTableAsc.begin(), m_listTableAsc.end(), m_readingTable);
    SMLK_LOGD("[T]  SqlLiteBase::FindNextTable %s ", m_readingTable.c_str());
    index = std::distance(std::begin(m_listTableAsc), ite1);
    if (index != -1 && index < maxLength) {
        m_readingTable = m_listTableAsc.at(index + 1);
        //SMLK_LOGD("[T] findNextTable protocol:%s,find next table",m_protocolName.c_str(),m_readingTable.c_str());
    }
}

/**
 * @brief 删除没有任何需要上传的盲区数据表  (暂时用不到 库中最多只有8张表，即使没有任何数据 也放着，方便逻辑处理!)
 * @param path   表名
 */
void SqlLiteBase::DelNoUseDataTable(const std::string &table) {
    int ret = -1;
    if (table != m_todayTable) {
        try {
            Poco::Data::Session session(m_sessionPool->get());
            session << "drop table %s ;", table, now;
        } catch (Poco::Exception &e) {
            SMLK_LOGE("[T] SqlLiteBase::DelNoUseDataTable() error:%s", e.displayText().c_str());
        }
        //SMLK_LOGD("[T] DelNoUseDataTable  protocol:%s,ret:%d,tableName: %s",m_protocolName.c_str(),ret,table.c_str());
    } else {
        //SMLK_LOGD("[T] today table, did not need to del  protocol:%s,ret:%d,tableName: %s, ",m_protocolName.c_str(),ret,table.c_str());
    }
}

/**
 * @brief 检查db文件是否存在
 * @param path   文件路径
 */
bool SqlLiteBase::CheckDBExist(const std::string &path) {
    return (access(path.c_str(), F_OK) != -1);
}

void SqlLiteBase::UpdateMsgFlag() {
    try {
        Poco::Data::Session session(m_sessionPool->get());
        session << "update %s  set sendState=1 where currentDate=?;", m_readingTable, use(m_sendMsgDate), now;
    } catch (Poco::Exception &e) {
        SMLK_LOGE("[T] SqlLiteBase::UpdateMsgFlag() error:%s", e.displayText().c_str());
    }
    SMLK_LOGD("[T] SqlLiteBase UpdateMsgFlag  protocol:%s", m_protocolName.c_str());
}

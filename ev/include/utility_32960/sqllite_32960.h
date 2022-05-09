#ifndef SQLLITEBASE_32960
#define SQLLITEBASE_32960

#include <iostream>
#include <functional>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>

#include "message_def_32960.h"
#include "smlk_sqlite.h"

using namespace smartlink;

/**
 * @brief 封装sqlite的Base
 */
class SqlLiteBase : public BaseSqlite {
public:
    SqlLiteBase();

    ~SqlLiteBase();

    SMLK_RC Init() override;
    void upgrade()override{}

    void InitTable();

    void WriteBlindData(char *buf, int len);

    void ReadBlindData(char *buf, int &len);

    void UpdateMsgFlag();

private:
    //协议名称
    std::string m_protocolName;
    //发送成功后 盲区数据的flag（20200407152930）需要在表中更新下标记位
    std::string m_sendMsgDate;
    //今天创建的数据表
    std::string m_todayTable;
    //当前正常读的数据表
    std::string m_readingTable;
    //获取数据库中的表名，降序的形式， 存储在变量 m_listTableDesc
    std::vector <std::string> m_listTableDesc;
    //是对m_listTableDesc的 反转 作用是方便从索引0开始找最老时间的数据
    std::vector <std::string> m_listTableAsc;

    void SetCurTabName();

    void GetTables(bool isDesc);

    void FindNextTable();

    void DelNoUseDataTable(const std::string& table);

    bool CheckDBExist(const std::string& path);
};

#endif // SQLCLIENT


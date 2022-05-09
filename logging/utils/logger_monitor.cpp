
#include "logger_monitor.h"
#include "smlk_log.h"
#include "smartlink_sdk_mcu.h"
#include "smartlink_sdk_sys_power.h"

#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <vector>

#if !defined(WIN32) && !defined(__WATCOMC__) && !defined(__VXWORKS__)
#include <sys/stat.h>
#endif

namespace smartlink {
namespace logger {

const  SMLK_UINT32 LogDiskMaxSize = 400*1024*1024;             //ota日志文件总大小阈值 400M //临时固定阈值

const  std::string CoredumpPath = "/userdata/record/hirain/hirain_extdata/";
const  std::string g_str_ota_path =      "/userdata/record/otalog/";


const  std::vector<std::string> g_vec_log_path = {
    "/userdata/record/otalog/"
};

const  std::vector<std::string> g_vec_zip_log_path =  {
    "/userdata/record/otalog/rotation/"
};

LoggerMonitor::LoggerMonitor(/* args */)
{
}

LoggerMonitor::~LoggerMonitor()
{
}

/******************************************************************************
* NAME: ThreadRun
*
* DESCRIPTION:
*   触发检查回调函数
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void LoggerMonitor::ThreadRun()
{
    std::thread threadObj(&LoggerMonitor::MonitorAllLog, this);
    threadObj.detach();
}

/******************************************************************************
* NAME: MonitorAllLog
*
* DESCRIPTION: 日志文件大小监控入口
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void LoggerMonitor::MonitorAllLog()
{
    SMLK_LOGD("MonitorAllLog");

    while (true)
    {
        if (GetAllLogTotalSize() > LogDiskMaxSize)
        {
            DeleteAllFile(g_str_ota_path);
        }

        if (CheckIsHaveCoreDump())
        {
            DeleteCoreDumpFile();
        }

        sleep(60 * 60 * 2);
    }
}

/******************************************************************************
* NAME: GetAllLogTotalSize
*
* DESCRIPTION:
*   获取所管理的日志总大小（包含当前日志和压缩日志，以及coredump文件）
*
* PARAMETERS:
*
* RETURN:
*   总大小(单位Byte)
*
* NOTE:
*****************************************************************************/
SMLK_UINT32 LoggerMonitor::GetAllLogTotalSize()
{
    SMLK_UINT32 nTotalSize = 0;
    for(auto path : g_vec_log_path)
    {
        SMLK_UINT32 size = GetDirectorySize(path.c_str());
        nTotalSize += size;
        SMLK_LOGD("=====[%s]file:total size:%d",path.c_str() ,size);

    }
    SMLK_LOGD("=====total size:%d", nTotalSize);

    return nTotalSize;
}

/******************************************************************************
* NAME: GetDirectorySize
*
* DESCRIPTION:
*   查询目录文件大小
*
* PARAMETERS:
*   dir: 查询目录路径
*
* RETURN:
*   目录大小 单位Byte
*
* NOTE:
*****************************************************************************/
SMLK_UINT32 LoggerMonitor::GetDirectorySize(const char *dir)
{
    DIR *dp;
    struct dirent *entry;
    struct stat statbuf;
    SMLK_UINT32 totalSize=0;

    if ((dp = opendir(dir)) == NULL)
    {
        fprintf(stderr, "Cannot open dir: %s\n", dir);
        return -1; //可能是个文件，或者目录不存在
    }

    //先加上自身目录的大小
    lstat(dir, &statbuf);
    totalSize+=statbuf.st_size;

    while ((entry = readdir(dp)) != NULL)
    {
        char subdir[256];
        sprintf(subdir, "%s/%s", dir, entry->d_name);
        lstat(subdir, &statbuf);

        if (S_ISDIR(statbuf.st_mode))
        {
            if (strcmp(".", entry->d_name) == 0 ||
                strcmp("..", entry->d_name) == 0)
            {
                continue;
            }

            SMLK_UINT32 subDirSize = GetDirectorySize(subdir);
            totalSize += subDirSize;
        }
        else
        {
            totalSize += statbuf.st_size;
        }
    }

    closedir(dp);
    return totalSize;
}

/******************************************************************************
* NAME:CheckIsHaveCoreDump
*
* DESCRIPTION:
*   检查对应目录下是否存在coredump文件
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
SMLK_BOOL LoggerMonitor::CheckIsHaveCoreDump()
{
    DIR *dir;
    struct dirent *ptr;

    if ((dir = opendir(CoredumpPath.c_str())) == NULL)
    {
        SMLK_LOGD("[%s]GetLogFiles no such file or directory ",CoredumpPath.c_str());
        return false;
    }

     while ((ptr = readdir(dir)) != NULL)
    {
        if(ptr->d_type == 8)        //8:FILE
        {
            std::string name = ptr->d_name;
            if (name.find("core") != name.npos)
            {
                return true;
            }
        }
    }
    closedir(dir);

    return false;
}

/******************************************************************************
* NAME: DeleteOver7DayLog
*
* DESCRIPTION:
*   删除所管理的日志中超过7天的压缩日志
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void LoggerMonitor::DeleteOver7DayLog()
{
    for (auto logPath : g_vec_zip_log_path)
    {
        std::string cmd = "find " + logPath + " -type f -mtime +7 | xargs rm -f 2>/dev/null";
        system(cmd.c_str());
    }

    SMLK_LOGD("log size over, delete over 7 day zip-log files");
}

/******************************************************************************
* NAME: DeleteCoreDumpFile
*
* DESCRIPTION:
*
* PARAMETERS:
*
* RETURN:
*
* NOTE:
*****************************************************************************/
void LoggerMonitor::DeleteCoreDumpFile()
{
    //删除dump文件
    std::string cmd = "cd " + CoredumpPath + " && rm -f *core*";
    system(cmd.c_str());
    SMLK_LOGD("log size over ,delete *core* file");
}

void LoggerMonitor::DeleteAllFile( std::string dirPath)
{
    //删除dump文件
    std::string cmd = "cd " + dirPath + " && rm -rf *";
    system(cmd.c_str());
}




} // namespace logger
} // namespace smartlink

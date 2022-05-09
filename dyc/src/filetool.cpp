
#include <stdio.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <dirent.h>
#include "filetool.h"
#include "Poco/StreamCopier.h"
#include "Poco/MD5Engine.h"
#include "Poco/DigestStream.h"
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using Poco::DigestEngine;
using Poco::MD5Engine;
using Poco::DigestOutputStream;
using Poco::StreamCopier;

bool FileTool::createDirectory(const std::string& path)
{
    if (fileExist(path))
    {
        return true;
    }
    int isCreate = mkdir(path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    bool success = (0==isCreate);
    if (success)
    {
        SMLK_LOGD("*** create path:%s", path.c_str());
    }
    else
    {
        SMLK_LOGD("create path failed! error code:%d, path : %s", (isCreate?1 : 0), path.c_str());
        return false;
    }
    return success;
}

bool FileTool::createMutiDirectory(std::string& path, mode_t mode)
{
    size_t pre=0,pos;
    std::string dir;
    int mdret = 0;
    if (path[path.size()-1] != '/'){
        // force trailing / so we can handle everything in loop
        path += '/';
    }
    if (fileExist(path))
    {
        return true;
    }

    while ((pos=path.find_first_of('/', pre)) != std::string::npos){
        dir = path.substr(0, pos++);
        pre = pos;
        if (dir.size() == 0) continue; // if leading / first time is 0 length
        if ((mdret=::mkdir(dir.c_str(), mode)) && errno!=EEXIST)
        {
            return mdret;
        }
    }
    return (0 == mdret);
}

int FileTool::fileSize(const std::string& fileName)
{
    struct stat statbuf;
    if (0 == stat(fileName.c_str(), &statbuf))
    {
        return statbuf.st_size;
    }
    return -1;
}

bool FileTool::fileExist(const char *fileName)
{
    if (NULL == fileName)
    {
        return false;
    }
    return (access(fileName, 0) != -1);
}

bool FileTool::fileExist(const std::string& fileName)
{
    return fileExist(fileName.c_str());
}

bool FileTool::isFile(const char* fileName)
{
    if (NULL == fileName)
    {
        return false;
    }
    struct stat st;
    int ret = stat(fileName, &st);
    if (ret < 0) {
        SMLK_LOGD("lstat %s failed, %s", fileName, strerror(errno));
        return false;
    }
    if (S_ISREG(st.st_mode))
    {
        //printf("*** is a file\n");
        return true;
    }
    //printf("*** is not a file\n");
    return false;
}

bool FileTool::isFile(const std::string& fileName)
{
    return isFile(fileName.c_str());
}

bool FileTool::isDirectory(const char* fileName)
{
    if (NULL == fileName)
    {
        return false;
    }
    struct stat st;
    int ret = stat(fileName, &st);
    if (ret < 0) {
        SMLK_LOGD("lstat %s failed, %s", fileName, strerror(errno));
        return false;
    }
    if (S_ISDIR(st.st_mode))
    {
        //printf("*** is a dir\n");
        return true;
    }
    //printf("*** is not a dir\n");
    return false;
}

bool FileTool::isDirectory(const std::string& fileName)
{
    return isDirectory(fileName.c_str());
}

int FileTool::openFile(const char *fileName)
{
    if (NULL == fileName)
    {
        SMLK_LOGD("*** open file failed, the parameter is error!");
        return -1;
    }
    int fd = open(fileName, O_RDWR|O_CREAT, S_IWUSR|S_IRUSR|S_IROTH);
    if (0 > fd)
    {
        SMLK_LOGD("*** open file failed, errno: %d", errno);
        return -1;
    }

    return fd;
}

int FileTool::openFile(const char *fileName, int oflag, mode_t mode)
{
    if (NULL == fileName)
    {
        SMLK_LOGD("*** open file failed, the parameter is error!");
        return -1;
    }
    int fd = open(fileName, oflag, mode);
    if (0 > fd)
    {
        SMLK_LOGD("*** open file failed, errno: %d", errno);
        return -1;
    }

    return fd;
}

FILE* FileTool::fopenFile(const std::string& fileName, const std::string& mode)
{
    return fopen(fileName.c_str(), mode.c_str());
}

size_t FileTool::freadFromFile(void* ptr, size_t size, size_t nmenb, FILE* stream)
{
    return fread(ptr, size, nmenb, stream);
}

void FileTool::rewindFile(FILE* stream)
{
    return rewind(stream);
}

size_t FileTool::fwriteToFile(const void* buffer, size_t size, size_t count, FILE* stream)
{
    return fwrite(buffer, size, count, stream);
}

size_t FileTool::fwriteToFile(const std::string& fileName, const std::string& content)
{
    FILE* pFile = fopenFile(fileName, "wt");
    if (NULL == pFile)
    {
        return -1;
    }
    return fwriteToFile(content.c_str(), content.length(), 1, pFile);
}

bool FileTool::fcloseFile(FILE* file)
{
    if (NULL != file)
    {
        fclose(file);
        file = NULL;
        return true;
    }
    return false;
}


bool FileTool::getAllfiles(const std::string& directory, std::vector<std::string>& fileVec)
{
       fileVec.clear();
       DIR *dir;
       struct dirent *ptr;
       if ((dir=opendir(directory.c_str())) == NULL) {
           SMLK_LOGD("open COLLECTION_DATA_SAVE_DIRECTORY failed");
           return false;
       }

       while ((ptr=readdir(dir)) != NULL) {
           if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0) {    ///current dir OR parrent dir
               continue;
           } else if (ptr->d_type == 8) {    ///file
               std::string fileName(ptr->d_name);
               if (fileName.length() > 0) {
                   SMLK_LOGD("filename is %s", fileName.c_str());
                   fileVec.push_back(fileName);
               } else {
                   SMLK_LOGD("filename.length = 0");
               }
           } else {
               std::string otherfileName(ptr->d_name);
               if (otherfileName.length() > 0) {
                   SMLK_LOGD("otherfileName is %s", otherfileName.c_str());
               } else {
                   SMLK_LOGD("otherfileName.length = 0");
               }
               continue;
           }
       }
       closedir(dir);
    return true;
}

bool FileTool::writeToFile(const std::string& fileName, const std::string& content)
{
    std::ofstream fout(fileName, std::ios::out);
    fout << content << std::endl;
    fout.clear();
    fout.close();
    return true;
}

int FileTool::writeToFile(const std::string& fileName, const char* buff, const int size)
{
    int result = -1;
    int ret = -1;
    int fd = -1;
    if (NULL == buff)
    {
        SMLK_LOGD("*** write file failed, the parameter is error!");
        return -1;
    }
    fd = open(fileName.c_str(), O_RDWR|O_CREAT, S_IWUSR|S_IRUSR|S_IROTH);
    if (0 > fd)
    {
        SMLK_LOGD("*** open file failed for write!!!");
        goto close;
    }
    if((flock(fd, LOCK_EX | LOCK_NB))< 0)
    {
        SMLK_LOGD("*** The log have been locked for write");
        goto close;
    }
    result = FileTool::writeToFile(fd, buff, size);
    if((flock(fd, LOCK_UN)) < 0)
    {
        SMLK_LOGD("*** Unlock the file failed for write, errno: %d", errno);
        goto close;
    }

    ret = fdatasync(fd);
    if (ret < 0) {
        SMLK_LOGD("fdatasync failed: %s", strerror(errno));
        goto close;
    }

close:
    if (0 < fd)
    {
        close(fd);
        fd = -1;
    }
    return result;
}

int FileTool::writeToFile(const int fd, const char* onceBuff, const int count)
{
    if (0>fd || NULL==onceBuff || 0>=count)
    {
        SMLK_LOGD("*** write file failed, the parameter is error!");
        return -1;
    }
    /*function: ssize_t write(int fd, const void buf, size_t count);
        parameter:
            fd: file descriptor to be written;
            buf: write the contents of buf to FD
            count: number of bytes written at a time
    */
    int write_num = write(fd, onceBuff, count);
    //SMLK_LOGD("*** fd = %d, count: %d, write_num:%d, after write.", fd, count, write_num);
    if (write_num <= 0 )
    {
        SMLK_LOGD("*** write failed, errno: %d", errno);
        return -1;
    }
    return write_num;
}

int FileTool::writeToFile(const int fd, const char* pContent, const int tryTimes, const int onceCacheSize, const bool& endFlag)
{
    int result = -1;
    char* onceBuff = NULL;
    int contentLength = -1;
    int totalSize = 0;
    int onceSize = 0;
    int times = 0;
    if (0>fd || NULL==pContent || 0>tryTimes || 0>=onceCacheSize)
    {
        SMLK_LOGD("*** write file failed, the parameter is error!");
        goto end;
    }

    contentLength = strlen(pContent);
    while(totalSize < contentLength)
    {
        if (endFlag)
        {
            break;
        }
        onceSize = writeToFile(fd, pContent, ((totalSize+onceCacheSize)>contentLength)? (contentLength-totalSize) : onceCacheSize);
        if (0 >= onceSize)
        {
            SMLK_LOGD("*** write file failed, retry 4 times!");
            if (tryTimes > times)
            {
                times++;
                continue;
            }
            else
            {
                SMLK_LOGD("*** write file failed!");
                times = 0;
                break;
            }
        }
        totalSize += onceSize;
        pContent += onceSize;
        times = 0;
    }
    result = totalSize;
    //SMLK_LOGD("*** write complete, onceSize= %d, totalSize = %d", onceSize, totalSize);

end:
    return result;
}

bool FileTool::readFromFile(const std::string& fileName)
{
    std::string str;
    ifstream fin;
    fin.open(fileName);
    while (!fin.eof())
    {
        getline(fin, str);
        SMLK_LOGD("*** str=%s", str.c_str());
    }
    fin.clear();
    fin.close();
    return true;
}

int FileTool::readFromFile(const int fd, char * onceBuff, const int count)
{
    char *buffer = NULL;
    if (fd<0 || NULL==onceBuff || 0>=count)
    {
        SMLK_LOGD("*** read file failed, the parameter is error!");
        return -1;
    }
    buffer = onceBuff;
    /* Function: ssize_t read (int FD, void * buf, size_t count);
    Function: read count bytes from the file descriptor FD and put them into buf;
    Return value: count = 0 return 0
    If count is not zero and the file is not empty, the current location of the file will be returned; if fail, return - 1;
    */
    int read_num = read(fd, buffer, count);
    //SMLK_LOGD("*** fd = %d, buffer=  %s read_num = %d, after read.", fd, buffer, read_num);
    if (read_num < 0)
    {
        SMLK_LOGD("*** read failed, errno: %d", errno);
        return -1;
    }
    if (0 == read_num)
    {
        SMLK_LOGD("*** read file tail and complete.");
    }
    return read_num;
}

std::string FileTool::getFileName(const int fd)
{
  if (0 >= fd) {
    return std::string ();
  }

  char buf[1024] = {'\0'};
  char file_path[1024] = {'0'}; // PATH_MAX in limits.h
  snprintf(buf, sizeof (buf), "/proc/self/fd/%d", fd);
  int ret = readlink(buf, file_path, sizeof(file_path) - 1);
  if ( ret != -1) {
    file_path[ret] = '\0';
    return std::string (file_path);
  }

  return std::string ();
}

bool FileTool::deleteFile(const std::string& filePath)
{
    remove(((std::string*)&filePath)->c_str());
    return true;
}

bool FileTool::deleteFile(const int fd)
{
    if (0 > fd)
    {
        SMLK_LOGD("*** delete file failed, the parameter is error!");
        return false;
    }
    remove(getFileName(fd).c_str());
    return true;
}

bool FileTool::hasWritePermission(const char* fileName)
{

    return (access(fileName, 2) != -1);
}


// List all files in one directory
void FileTool::scanDirFiles(const std::string& filePath)
{
    scanDirFiles(filePath.c_str());
}

void FileTool::scanDirFiles(const char* filePath)
{
    if (NULL == filePath)
    {
        SMLK_LOGD("*** file path is null!");
        return;
    }
    struct stat s;
    int ret = lstat(filePath, &s);
    if (ret < 0) {
        SMLK_LOGD("lstat %s failed, %s", filePath, strerror(errno));
        return;
    }
    if (!S_ISDIR(s.st_mode))
    {
        SMLK_LOGD("*** file path is not exist!");
        return;
    }

    struct dirent* filename = NULL;
    DIR* dir = NULL;
    dir = opendir(filePath);
    if (NULL == dir)
    {
        return;
    }

    int iName=0;
    while((filename=readdir(dir)) != NULL)
    {
        if(strcmp(filename->d_name , ".") == 0 || strcmp(filename->d_name , "..") == 0)
        {
            continue;
        }
        char wholePath[128] = {0};
        sprintf(wholePath, "%s/%s", filePath, filename->d_name);
        //cout << "wholePath= " << wholePath << endl;
    }
    if (NULL != dir)
    {
        chdir(".."); // Go back to the parent directory
        closedir(dir);
    }
}

// List all directories and files recursively
void FileTool::recursionScanDirFiles(const char *dir, int depth)
{
        DIR *p_dir = NULL;
        struct dirent *p_entry = NULL;
        struct stat statbuf;
        if (NULL==dir || NULL==(p_dir=opendir(dir)))
        {
            SMLK_LOGD("*** can't open dir.");
            return;
        }

        chdir (dir);
        while (NULL != (p_entry=readdir(p_dir)))    // Get next level directory information
        {
            lstat(p_entry->d_name, &statbuf);       // Get next level member properties
            if(S_IFDIR & statbuf.st_mode)           // Determine whether the next level member is a directory
            {
                if (strcmp(".", p_entry->d_name)==0 || strcmp("..", p_entry->d_name)==0)
                {
                    continue;
                }
                //SMLK_LOGD("*** %*s%s", depth, "", p_entry->d_name);
                recursionScanDirFiles(p_entry->d_name, depth+4); // Scan the contents of the next level directory
            }
            else
            {
                //SMLK_LOGD("*** %*s%s", depth, "", p_entry->d_name);  // Output property is not a member of directory
            }
        }
        chdir(".."); // Go back to the parent directory
        closedir(p_dir);
}

int FileTool::renameFile(char* oldFile, char* newFile)
{
        if (NULL==oldFile || NULL==newFile)
        {
            SMLK_LOGD("*** Prameter error when rename file.");
            return -1;
        }
        return rename(oldFile, newFile);
}

int FileTool::renameFile(std::string& oldFile, std::string& newFile)
{
        return renameFile((char*)(oldFile.c_str()), (char*)(newFile.c_str()));
}

bool FileTool::replaceString(std::string& src, const std::string& before, const std::string& after)
{
    for (std::string::size_type pos(0); pos != std::string::npos; pos += after.length())
    {
        pos = src.find(before, pos);
        if (pos != std::string::npos)
        {
            src.replace(pos, before.length(), after);
        }
        else
        {
            break;
        }
    }
    return true;
}

std::string FileTool::getFileName(std::string filePath)
{
    if (filePath.empty())
    {
        return "";
    }

    replaceString(filePath, "/", "\\");

    std::string::size_type iPos = filePath.find_last_of('\\') + 1;

    return filePath.substr(iPos, filePath.length() - iPos);
}

bool FileTool::CalculateFileMd5Sum(std::string & FilePath, std::vector<unsigned char> &Md5StrVec)
{
    ifstream inputStream(FilePath, ios::binary);
    MD5Engine md5;
    DigestOutputStream dos(md5);
    StreamCopier::copyStream(inputStream, dos);
    dos.close();
    Md5StrVec = md5.digest();
    // Md5Str = DigestEngine::digestToHex(md5.digest());
    return true;
}

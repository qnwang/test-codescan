
#ifndef __FILETOOL_H__
#define __FILETOOL_H__
#include <sys/types.h>
#include <sys/stat.h>
extern "C++" {
#include <vector>
#include <string>
#include <list>
}
#include "smlk_log.h"

using namespace std;

class FileTool
{
    public:
        FileTool()
        {

        };
        ~FileTool()
        {

        };
        static bool createDirectory(const std::string& path);
        static bool createMutiDirectory(std::string& path, mode_t mode=(S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IROTH));
        static int fileSize(const std::string& fileName);
        static bool fileExist(const char* fileName);
        static bool fileExist(const std::string& fileName);
        static bool isDirectory(const char* fileName);
        static bool isDirectory(const std::string& fileName);
        static bool isFile(const char* fileName);
        static bool isFile(const std::string& fileName);
        static int openFile(const char* fileName);
        static int openFile(const char* fileName, int oflag, mode_t mode);

        /*
        r (read): read
        w (write): write
        a (append): append
        t (text): text file
        B (binary): binary
        +: read and write
        */
        static FILE* fopenFile(const std::string& fileName, const std::string& mode="+");
        /*
        Read the data from the file stream,
        ptr: ptr is the space to save the read file data,
        size is the size of one character read from the file,
        and nmenb is the number of characters to be read.
        stream: stream is the open file pointer,
        After reading successfully, freed will return a value equal to that of nmenb, and - 1 will be returned conversely
        */
        static size_t freadFromFile(void* ptr, size_t size, size_t nmenb, FILE* stream);
        // Reorient the file pointer to the beginning of the file
        static void rewindFile(FILE* stream);
        /*
        Buffer: it is a pointer. For fwrite, it is the address to get data, buffer can be a struct,
        Size: number of single bytes to write content,
        Count: number of data items to write size bytes,
        Stream: target file pointer,
        Return the number of data items actually written count.
        */
        static size_t fwriteToFile(const void* buffer, size_t size, size_t count, FILE* stream);
        static size_t fwriteToFile(const std::string& fileName, const std::string& content);
        static bool fcloseFile(FILE* file);
        static bool getAllfiles(const std::string& directory, std::vector<std::string>& fileVec);
        static bool writeToFile(const std::string& fileName, const std::string& content);
        static int writeToFile(const std::string& fileName, const char* buff, const int size);
        static int writeToFile(const int fd, const char* onceBuff, const int count);
        static int writeToFile(const int fd, const char* content, const int tryTimes, const int onceCacheSize, const bool& endFlag);
        static bool readFromFile(const std::string& fileName);
        static int readFromFile(const int fd, char* onceBuff, const int count);
        static std::string getFileName(const int fd);
        static bool deleteFile(const std::string& filePath);
        static bool deleteFile(const int fd);
        static bool hasWritePermission(const char* fileName);
        static void scanDirFiles(const std::string& filePath);
        static void scanDirFiles(const char* filePath);
        static void recursionScanDirFiles(const char* dir, int depth);
        static int renameFile(char* oldFile, char* newFile);
        static int renameFile(std::string& oldFile, std::string& newFile);
        static std::string getFileName(std::string filePath);
        static bool replaceString(std::string& src, const std::string& before, const std::string& after);
        static bool CalculateFileMd5Sum(std::string& FilePath, std::vector<unsigned char> &Md5StrVec);
};
#endif //__FILETOOL_H__

#ifndef CVSUITE_SOURCE_DRIVER_FILEIO_H
#define CVSUITE_SOURCE_DRIVER_FILEIO_H
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>

#include <opencv2/core/persistence.hpp>

namespace fs = boost::filesystem;

namespace image {
    // Write and read the protobuf binary file.
    template<typename ProtoType>
    void ReadFromFile(const char* file_name, ProtoType& proto) {
        std::fstream r(file_name, std::ios::in | std::ios::binary);
        proto.ParseFromIstream(&r);
    }

    template<typename ProtoType>
    void SerializeToFile(const char* file_name, const ProtoType& proto) {
        std::fstream w(file_name, std::ios::out | std::ios::trunc | std::ios::binary);
        proto.SerializeToOstream(&w);
    }
    template<typename ProtoType>
    void SerializeToFile(const std::string file_name, const ProtoType& proto) {
        SerializeToFile(file_name.c_str(), proto);
    }

    // Return current working directory.
    std::string	GetCurrentDir();
    // Return true if p.extension().string() == std::string(suffix).
    bool CheckExtension(const fs::path& p, const char* suffix = nullptr);

    // Replace stem(filename without extension) with new_stem.
    // Example:
    // auto new_file = ReplaceStem(file,"new")// new_file.stem().string() = "new".
    fs::path ReplaceStem(const fs::path& origin, const char* new_stem);


    std::string textRead(const std::string& path);
    bool textWrite(const std::string& text, const std::string& path);

    size_t GetFileSize(FILE* file);
    size_t GetFileSize(std::string file_path);
    std::vector<unsigned char> ReadFromFileBinary(const std::string& file_path);

    // Hash Function
    // Lower-level versions of Get... that read directly from a character buffer
    // without any bounds checking.
    inline uint32_t DecodeFixed32(const char* ptr);
    // Hash
    inline uint32_t Hash(const char* data, size_t n, uint32_t seed);

}

#endif
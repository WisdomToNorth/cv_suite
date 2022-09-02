#include "pch.h"
#include "fileio.h"

#pragma warning(disable : 4996)
namespace image {
    std::string	GetCurrentDir() {
        auto current_path = boost::filesystem::current_path();
        return current_path.generic_string();
    }
    bool CheckExtension(const fs::path& p, const char* suffix) {
        auto ex = p.extension();
        if (ex.empty() && suffix != nullptr) return false;
        return (ex.generic_string() == std::string(suffix));
    }

    // Replace stem(filename without extension) with new_stem.
    // Example:
    // auto new_file = ReplaceStem(file,"new")// new_file.stem().string() = "new".
    fs::path ReplaceStem(const fs::path& origin, const char* new_stem) {
        return origin.parent_path() / fs::path(new_stem + origin.extension().string());
    }

    std::string textRead(const std::string& path) {
        std::ifstream in(path.c_str(), std::ios::in | std::ios::binary);
        if (in) {
            std::string contents;
            in.seekg(0, std::ios::end);
            contents.resize(static_cast<int>(in.tellg()));
            in.seekg(0, std::ios::beg);
            in.read(&contents[0], contents.size());
            in.close();
            return (contents);
        }
        else {
            return std::string();
        }
    }

    bool textWrite(const std::string& text, const std::string& path) {
        std::ofstream out(path, std::ios::out | std::ios::binary);
        if (out) {
            out << text << std::endl;
            out.close();
            return true;
        }
        return false;
    }

    size_t GetFileSize(FILE* file)
    {
        fseek(file, 0, SEEK_END);
        size_t read_len = ftell(file);
        fseek(file, 0, SEEK_SET);
        return read_len;
    }

    size_t GetFileSize(std::string file_path)
    {
        FILE* file = fopen(file_path.c_str(), "rb");
        if (file == nullptr) return 0;
        size_t size = GetFileSize(file);
        fclose(file);
        return size;
    }

    std::vector<unsigned char> ReadFromFileBinary(const std::string& file_path)
    {
        FILE* file = fopen(file_path.c_str(), "rb");
        std::vector<unsigned char> result;
        if (file == nullptr) return result;

        size_t file_size = GetFileSize(file);
        if (file_size != 0)
        {
            result.resize(file_size);
            size_t n = fread(&result[0], 1, file_size, file);
            assert(n <= file_size);
            if (n != file_size)
            {
                result.resize(n);
            }
        }

        // 在读取过程中可能文件变化，再尝试读取。
        const size_t read_len = 1024;
        char buf[read_len];
        for (;;)
        {
            size_t n = fread(buf, 1, read_len, file);
            result.insert(result.end(), buf, buf + n);
            if (n < read_len) break;
        }
        fclose(file);
        return result;
    }

    uint32_t DecodeFixed32(const char* ptr) {
        // Load the raw bytes
        uint32_t result;
        memcpy(&result, ptr, sizeof(result)); // gcc optimizes this to a plain load
        return result;
    }

    uint32_t Hash(const char* data, size_t n, uint32_t seed) {
        // Similar to murmur hash
        const uint32_t m = 0xc6a4a793;
        const uint32_t r = 24;
        const char* limit = data + n;
        uint32_t h = seed ^ (n * m);

        // Pick up four bytes at a time
        while (data + 4 <= limit) {
            uint32_t w = DecodeFixed32(data);
            data += 4;
            h += w;
            h *= m;
            h ^= (h >> 16);
        }

        // Pick up remaining bytes
        switch (limit - data) {
        case 3:
            h += static_cast<unsigned char>(data[2]) << 16;
        case 2:
            h += static_cast<unsigned char>(data[1]) << 8;
        case 1:
            h += static_cast<unsigned char>(data[0]);
            h *= m;
            h ^= (h >> r);
            break;
        }
        return h;
    }
}

#include "pch.h"
#include "core.h"

#include "fileio.h"

#include <boost/filesystem.hpp>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>

// TODO: get rid of stdio or iostream. DONT use mixture of these two lib.
#include <iostream>
#include <chrono>
#include <cstdio>
#include <mutex>

#pragma warning(disable : 4996)
namespace common {
    using namespace std::chrono;
    ImageID::ImageID(uint64_t frame, uint64_t stamp, high_resolution_clock::time_point time, system_clock::time_point sys_time) :
        frameID(frame), timestamp(stamp), timepoint(time), timepoint_sys(sys_time)
    {}

    ImageID::ImageID(const ImageID& r) :
        frameID(r.frameID), timestamp(r.timestamp), timepoint(r.timepoint), timepoint_sys(r.timepoint_sys)
    {}

    //Copy and swap
    ImageID& ImageID::operator=(ImageID rhs) {
        swap(rhs);
        return *this;
    }

    bool ImageID::operator<(const ImageID& r) const {
        return frameID < r.frameID;
    }

    std::string ImageID::FileName() const {
        return std::string();
    }

    void ImageID::swap(ImageID& rhs) {
        using std::swap;
        swap(frameID, rhs.frameID);
        swap(timestamp, rhs.timestamp);
        swap(timepoint, rhs.timepoint);
        swap(timepoint_sys, rhs.timepoint_sys);
    }

    std::string NameOfWeldType(WeldType type) {
        switch (type)
        {
        case common::WeldType::UNKNOWN:
            return std::string("Unknown");
            break;
        case common::WeldType::V:
            return std::string("V");
            break;
        case common::WeldType::BUTT:
            return std::string("Butt");
            break;
        case common::WeldType::CORNER:
            return std::string("T");
            break;
        default:
            return std::string("Error!");
            break;
        }
    }

    std::tuple<bool, WeldType> Int2WeldType(int val) {
        switch (val)
        {
        case static_cast<int>(common::WeldType::UNKNOWN):
            return std::make_tuple(true, common::WeldType::UNKNOWN);
            break;
        case static_cast<int>(common::WeldType::V):
            return std::make_tuple(true, common::WeldType::V);
            break;
        case static_cast<int>(common::WeldType::BUTT):
            return std::make_tuple(true, common::WeldType::BUTT);
            break;
        case static_cast<int>(common::WeldType::CORNER):
            return std::make_tuple(true, common::WeldType::CORNER);
            break;
        default:
            return std::make_tuple(false, common::WeldType::UNKNOWN);
            break;
        }
    }

    RawTime RawTime::Now() {
        auto t = system_clock::now();
        auto ms = duration_cast<milliseconds>(t.time_since_epoch()).count();
        return RawTime{ ms };
    }

    system_clock::time_point RawTime::BaseSystemTime() {
        static system_clock::time_point basetime{ system_clock::now() };
        return basetime;
    }
    high_resolution_clock::time_point RawTime::BaseHighResolutionTime() {
        static high_resolution_clock::time_point basetime{ high_resolution_clock::now() };
        return basetime;
    }

    AppClock::AppClock()
        : high_resolution_epoch(high_resolution_clock::now())
        , system_epoch(system_clock::now())
        , save_time_point(0)
        , camera_base_point()
        , isSync(false)
    {

    }

    AppClock::~AppClock() {

    }

    bool AppClock::IsSync() const
    {
        return isSync;
    }
    void AppClock::SyncImageTime(const ImageID& image_info, const clock_time_point& time_point)
    {
        APPDuration stamp_epoch_duration(image_info.timestamp);
        camera_base_point = time_point - stamp_epoch_duration;
        isSync = true;
    }
    //Global
    AppClock clock;
    WeldData::WeldData() {

    }
    WeldData::~WeldData() {

    }

    //Format: 2016_1011_13_34_52
    //second level precision��combine FileNameFromTime for more precise time string.
    std::string StringFromDate(system_clock::time_point now_sys) {
        time_t t = system_clock::to_time_t(now_sys);   // get time now
        struct tm* now = localtime(&t);
        const char* fmt = "%04d_%02d%02d_%02d_%02d_%02d";
        char data_cstr[100];
        std::snprintf(data_cstr, sizeof(data_cstr), fmt,
            now->tm_year + 1900,
            now->tm_mon + 1,
            now->tm_mday,
            now->tm_hour,
            now->tm_min,
            now->tm_sec);
        return  std::string(data_cstr);
    }

    // http://stackoverflow.com/questions/27136854/c11-actual-system-time-with-milliseconds
    std::string MsTimeString() {
        auto this_time = system_clock::now();
        auto duration = this_time.time_since_epoch();
        auto duration_s_part = duration - duration_cast<seconds>(duration);

        auto ms_count = duration_cast<milliseconds>(duration_s_part).count();
        //system_clock has no microsecond-level precision.
        //auto duration_ms_part = duration - duration_cast<milliseconds>(duration);
        //auto us_count = duration_cast<milliseconds>(duration_ms_part).count();
        if (ms_count >= 1000) {
            ms_count = 999;
        }
        char ms_cstr[100];
        const char* fmt_ms = "_%03d";
        std::snprintf(ms_cstr, sizeof(ms_cstr), fmt_ms,
            ms_count);
        return StringFromDate(this_time) + ms_cstr;
    }

    void swap(ImageResultData& l, ImageResultData& r)
    {
        using std::swap;
        swap(l.type, r.type);
        swap(l.featurepoints, r.featurepoints);
        swap(l.bad, r.bad);
        swap(l.height, r.height);
    }
}

namespace global {
    common::AppClock clock_point;
    common::WeldData weld_data;
    std::unordered_map<common::ImageID, common::ImageResultData> image_result_table;

    std::string data_dir;
    std::string setting_dir;
    std::string image_dir;

    void init() {
        // Initialize all required dirs under working dir.
        auto working_dir = fs::path(image::GetCurrentDir());
        std::vector<fs::path> sub_dirs;

        sub_dirs.push_back(fs::path(working_dir) / "image");
        global::image_dir = sub_dirs.back().generic_string();
        sub_dirs.push_back(fs::path(working_dir) / "data");
        global::data_dir = sub_dirs.back().generic_string();
        sub_dirs.push_back(fs::path(working_dir) / "setting");
        global::setting_dir = sub_dirs.back().generic_string();

        sub_dirs.push_back(fs::path(working_dir) / "data/log");

        for (auto& p : sub_dirs) {
            if (!fs::exists(p)) {
                fs::create_directory(p);
            }
        }
        google::InitGoogleLogging("cv-suite");
        FLAGS_logtostderr = 0;
        FLAGS_log_dir = "data/log";
    }

    //Global Robot State
    ///Lock guarding the state
    static std::mutex global_inner_lock;
    ///Local robotinfo data.
    static common::RobotInfo g;

    ///Update
    void UpdateRobotInfo(common::RobotInfo now) {
        std::lock_guard<std::mutex> l(global_inner_lock);
        g = now;
    }
    ///Get
    common::RobotInfo GetRobotInfo() {
        std::lock_guard<std::mutex> l(global_inner_lock);
        return g;
    }

    struct GlobalImpl {
        using time_point_t = std::chrono::system_clock::time_point;
        FILE* fileTemp_;
        size_t log_limit_;//size_t 类型为 unsigned int 
        GlobalImpl(size_t log_limit = 10000) :
            fileTemp_(NULL),
            log_limit_(log_limit) {
            init();
            auto filename = "data/log_" + common::StringFromDate(std::chrono::system_clock::now());
            fileTemp_ = std::fopen(filename.c_str(), "w");
            //This is an unbuffered stream
            if (fileTemp_) {
                std::setbuf(fileTemp_, NULL);
            }
        }
        ~GlobalImpl() {
            if (fileTemp_) {
                std::fclose(fileTemp_);
            }
        }
        std::deque<std::pair<time_point_t, common::RobotPosition>> pos_by_time;
    };

    Global::Global() :global_(new GlobalImpl) {
    }
    Global::~Global() {

    }

    void Global::Global::Print2TempLog(const std::string& log) {
        Print2TempLog(log.c_str());
    }
    void Global::Global::Print2TempLog(const char* log) {
        if (global_->fileTemp_) {
            std::fprintf(global_->fileTemp_, log);
        }
    }
    void Global::Print2Glog(const std::string& qdebug_info) {
        LOG(INFO) << qdebug_info;
    }
    void Global::Print2Glog(const char* qdebug_info) {
        LOG(INFO) << qdebug_info;
    }
    void Global::UpdatePosition(const std::chrono::system_clock::time_point, const common::RobotPosition) {
    }
    common::RobotPosition Global::findClosestPosition(const std::chrono::system_clock::time_point) const
    {
        return common::RobotPosition();
    }
    // Singleton
    Global* GetGlobal() {
        static Global global_;
        return &global_;
    }
}

#ifndef CVSUITE_SOURCE_DRIVER_CORE_H
#define CVSUITE_SOURCE_DRIVER_CORE_H

#include <chrono>
#include <ratio>
#include <string>
#include <deque>
#include <array>
#include <unordered_map>
#include <map>
#include <memory>
#include <stdint.h>

namespace common {
    using namespace std::chrono;
    using s = seconds;
    using ms = milliseconds;
    using us = microseconds;
    using ns = nanoseconds;
    using clock_time_point = std::chrono::high_resolution_clock::time_point;

    struct ImageID {
        uint64_t										frameID;
        uint64_t										timestamp;
        clock_time_point								timepoint;
        system_clock::time_point						timepoint_sys;
        ImageID() = default;
        explicit ImageID(uint64_t frame, uint64_t stamp, high_resolution_clock::time_point time, system_clock::time_point sys_time);
        ImageID(const ImageID& r);
        //Copy and swap
        ImageID& operator=(ImageID rhs);
        virtual ~ImageID() = default;
        bool operator<(const ImageID& r) const;
        std::string FileName() const;
        void swap(ImageID& rhs);
    };

    inline int64_t ms_since_epoch() { return duration_cast<ms>(system_clock::now().time_since_epoch()).count(); }

    //	Frames from Daheng Camera owns a timestamp with 20ns resolution. 
    constexpr int DH_NANO = 20;
    using DHDuration = duration<uint64_t, std::ratio_multiply<std::ratio<DH_NANO, 1>, std::nano>>;
    constexpr int APP_NANO_RESOLUTION = DH_NANO;
    using APPDuration = DHDuration;

    struct RawTime {
        int64_t ms;
        static RawTime Now();
        static system_clock::time_point BaseSystemTime();
        static high_resolution_clock::time_point BaseHighResolutionTime();
    };

    struct AppClock {
        AppClock();
        ~AppClock();
        //Noncopyable
        AppClock(const AppClock&) = delete;
        AppClock& operator=(const AppClock&) = delete;


        bool IsSync() const;
        void SyncImageTime(const ImageID&, const clock_time_point&);


        clock_time_point high_resolution_epoch;
        system_clock::time_point system_epoch;
        std::vector<clock_time_point> save_time_point;
        clock_time_point camera_base_point;
        bool isSync;
    };


    ///----------------
    constexpr int V_NUM = 1;
    constexpr int BUTT_NUM = 1;
    constexpr int CORNER_NUM = 1;

    enum class WeldType {
        UNKNOWN = 0,
        V = 1,
        BUTT = 2,
        CORNER = 3
    };
    std::string NameOfWeldType(WeldType);
    std::tuple<bool, WeldType> Int2WeldType(int);
    struct RobotPosition {
        float x, y, z, w, p, r, e1, e2, e3;
        short c1, c2, c3, c4, c5, c6, c7, ut, uf, validc;
    };



    //Contain All robot info: position, num reg state.
    struct RobotInfo {
        RobotPosition pos;
        RawTime time;
        int loops; // NumReg 6
        int timer; // NumReg 7
        int state; // NumReg 8
        int label; // NumReg 10
        int arg1; //NumReg 11
        int arg2; //NumReg 12
    };
    struct RobotPositionSlim {
        float x, y, z, w, p, r;
        RobotPositionSlim() = default;
        RobotPositionSlim(const RobotPositionSlim&) = default;
        RobotPositionSlim& operator=(const RobotPositionSlim&) = default;
    };
    // XYZWPR(WPR in angle) is the form of position data from Fanuc Controller.
    using FanucRobotPosition = RobotPositionSlim;
    //Necessary info to convert to proto 
    struct RobotInfoSlim {
        RobotPositionSlim pos;
        RawTime time;
    };
    template<typename RobotInfo, typename Proto>
    void RobotToProto(Proto* p, const RobotInfo& r) {
        p->set_p1(r.pos.x); p->set_p2(r.pos.y);
        p->set_p3(r.pos.z); p->set_p4(r.pos.w);
        p->set_p5(r.pos.p); p->set_p6(r.pos.r);
        p->set_timestamp(r.time.ms);
    }
    template<typename RobotInfo, typename Proto>
    RobotInfo ProtoToRobot(const Proto& p) {
        RobotInfo r{};
        r.pos.x = p.p1(); r.pos.y = p.p2(); r.pos.z = p.p3();
        r.pos.w = p.p4(); r.pos.p = p.p5(); r.pos.r = p.p6();
        r.time.epoch = p.time().epoch(); r.time.us = p.time().us();
        return r;
    }

    //
    template<typename Float>
    struct Point2D {
        Float x;
        Float y;
    };
    template<typename Float>
    struct LinePoint2D {
        Float x;
        Float y;
        Float angle;
        LinePoint2D(Float x_, Float y_, Float angle_) :x(x_), y(y_), angle(angle_) {}
        LinePoint2D(const LinePoint2D&) = default;
        LinePoint2D& operator=(const LinePoint2D&) = default;
    };

    using VecPts = std::vector<Point2D<float>>;
    using VecLinePts = std::vector<LinePoint2D<float>>;

    struct ImageResultData {
        WeldType type;
        bool bad;
        std::vector<std::pair<int, int>> featurepoints;
        std::vector<std::pair<int, int>> points;
        double area, gap, mismatch;
        int height;
        int state;
    };
    struct WeldData {
        friend struct Clock;
        std::vector<std::shared_ptr<ImageID>>																frame_vector;
        std::vector<std::pair<ImageID, ImageResultData>>											image_result_buffer;
        std::vector<std::pair<clock_time_point, RobotPosition>>										robot_data_buffer;
        std::map<uint64_t, std::shared_ptr<ImageID>>														time2image;
        std::map<clock_time_point, std::vector<clock_time_point>>											weld_pass_table;
        WeldData();
        ~WeldData();

    };

    struct InverseExp
    {
        InverseExp() :a(), b(), c() {}
        InverseExp(double a_, double b_, double c_)
            : a(a_)
            , b(b_)
            , c(c_) {}

        double operator()(double x) const {
            if (b + c - 0 < DBL_EPSILON) return 0;
            return a / (b + x) + c;
        }
        double Inverse(double z) const {
            if (z - c < DBL_EPSILON) return 0;
            return a * (z - c) - b;
        }
        double a, b, c;
    };

    //
    std::string StringFromDate(system_clock::time_point now_sys);
    // Unsafe C-style api, it's user to offer buffer. 
    //void StringFromDateC(system_clock::time_point now_sys, char*);
    std::string MsTimeString();
}

namespace std {
    template<>
    struct hash<::common::ImageID> {
        size_t operator()(const common::ImageID& k) const {
            return hash<decltype(k.frameID)>()(k.frameID);
        }
    };
}

namespace global {
    extern common::AppClock clock_point;
    extern common::WeldData weld_data;
    extern std::unordered_map<common::ImageID, common::ImageResultData> image_result_table;

    extern std::string data_dir;
    extern std::string setting_dir;
    extern std::string image_dir;
    struct GlobalImpl;

    struct Global {
        Global();
        ~Global();

        //
        // A temp log file created under data directory
        // Safety is not tested.
        //
        void Print2TempLog(const std::string&);
        void Print2TempLog(const char*);
        //
        // Glog file
        //
        void Print2Glog(const std::string&);
        void Print2Glog(const char*);

        GlobalImpl* global_;
        void UpdatePosition(const std::chrono::system_clock::time_point, const common::RobotPosition);
        common::RobotPosition findClosestPosition(const std::chrono::system_clock::time_point) const;
    };

    //Singleton
    Global* GetGlobal();

    void UpdateRobotInfo(common::RobotInfo);
    common::RobotInfo GetRobotInfo();
}



#endif

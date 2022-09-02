#ifndef CVSUITE_SOURCE_DRIVER_IMAGE_H
#define CVSUITE_SOURCE_DRIVER_IMAGE_H
#include "core.h"

#include <opencv2/core.hpp>
#include <Eigen/Core>

#include <memory>
#include <map>
#include <mutex>
#include <functional>
#include <vector>
#include <bitset>
#include <atomic>

namespace image {
    extern const char* IMAGE_SUFFIX;
    extern const char* IMAGE_FOLDER;
    enum {
        Original = 1,
        Process = 2,
        Point = 3,
        Hist = 4
    };
    enum
    {
        Profile = 1
    };
    // Draw cross in given point.
    void DrawCross(cv::Mat img, cv::Point pt, cv::Scalar color = cv::Scalar(0, 0, 255), int lineTk = 4, size_t ratio = 50);
    int PlotHist(const cv::Mat& in, cv::Mat& out);

    /* ImageProcessBase:
    3 key function in ImageProcessBase, which you should implement in your derived class:
    virtual store();
    virtual process();
    virtual output();

    callback you may be interested to register:
    DisplayCallback
    DataCallback
    StoreCallback

    if you donot register StoreCallback, it will call cv::write
    */
    class ImageProcessBase {
    public:
        using Ptr = std::shared_ptr<ImageProcessBase>;

        using DisplayCallback = std::function<void(const cv::Mat, const int id)>;
        using DataCallback = std::function<void(const common::ImageResultData)>;
        using StoreCallback = std::function<bool(const std::string& absFilepath, const cv::Mat&)>;
        ImageProcessBase(std::string name);
        virtual ~ImageProcessBase();
        // ----ImageProcessBase Interface-------------------------------
        // Store the raw frame.
        //[Note]:
        virtual void  store() const;
        // Common image preprocess. Store the result for output().
        virtual void  process();
        // Output the result data structure.
        virtual void  output();
        // Bind the gui.
        ///Bind the image displaying
        void registerDisplayEvent(DisplayCallback);
        ///Bind the plotting.
        //void registPlotEvent(PlotCallback);
        void registerDataEvent(DataCallback);
        void registerStoreEvent(StoreCallback);
        // ---------------------------------------------------

        // --set/get------------------------------------------
        std::string getName() const;
        void setOrigin(cv::Mat, const int64_t timestamp, bool clone = false);
        cv::Mat getOrigin();
        void setROI(int x, int y, int width, int height);
        cv::Rect getROI() const;
        void setFreq(int new_frequency);
        void setStore(bool);
        void setTimestamp(int64_t);
        int64_t getTimestamp() const;

        bool isDisabled() const;
        void setDisable();
        void setEnable();
        //----------------------------------------------------

    protected:
        const std::string name_;
        cv::Mat origin_;
        int64_t timestamp_;
        cv::Mat imageInProcess_;
        cv::Rect roi_;
        // For image saving frequency.
        int mutable counter_;
        std::atomic<int> save_frequency_;
        // States
        std::atomic<bool> isStore_;
        std::atomic<bool> disabled_;
        bool lastIsBad_; // if process() fail, set this = true. set to false at the end of process if success.
        // Callback for messaging and display.
        //@Input: RGB mat.
        DisplayCallback eventDisplay_;
        //PlotCallback eventPlot_;
        DataCallback eventData_;
        StoreCallback eventStore_;
        // Locks
        mutable std::mutex callback_lock_;
        //mutable std::mutex roi_lock_;
        mutable std::mutex value_lock_;
        //std::chrono::system_clock::time_point tp_;
        // Messaging
        bool _display(const cv::Mat, const int) const;
        bool _sendData(const common::ImageResultData&) const;
        bool _store(const std::string& name, const cv::Mat&) const;
    };
}

#endif //CVSUITE_SOURCE_DRIVER_IMAGE_H

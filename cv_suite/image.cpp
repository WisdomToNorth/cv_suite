#include "pch.h"
#include "image.h"
#include "core.h"
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iostream>
#include <mutex>


namespace image {
    const char* IMAGE_SUFFIX = ".bmp";
    const char* IMAGE_FOLDER = "image\\";
    //auto image_logger = spdlog::basic_logger_mt("image", "image.txt");
    ImageProcessBase::ImageProcessBase(std::string name)
        : name_(name)
        , imageInProcess_()
        , roi_()
        , counter_(0)
        , save_frequency_(1)
        , isStore_(true)
        , disabled_(false)
        , lastIsBad_(false)
    {
        eventStore_ = [](const std::string& absFilepath, const cv::Mat& to_store)->bool {
            try {
                return cv::imwrite(absFilepath, to_store);
            }
            catch (cv::Exception& ex) {
                LOG(ERROR) << "Exception from storing image: " << ex.what();
                return false;
            }
        };
    }

    ImageProcessBase::~ImageProcessBase() {}

    // Store the raw frame.
    void ImageProcessBase::store() const {
        if (isStore_ && !lastIsBad_) {
            ++counter_;
            if (counter_ % save_frequency_ == 0) {
                _store(IMAGE_FOLDER + common::MsTimeString() + IMAGE_SUFFIX, origin_);
            }
        }
    }
    void ImageProcessBase::process() {
        lastIsBad_ = false;
    }
    void ImageProcessBase::output() {
        //std::thread plotThread;
        //plotThread.detach();
        _display(origin_, 1);
    }

    void ImageProcessBase::registerDisplayEvent(std::function<void(const cv::Mat, int id)> callbackF) {
        std::lock_guard<std::mutex> guard(callback_lock_);
        eventDisplay_ = callbackF;
    }

    void ImageProcessBase::registerDataEvent(std::function<void(common::ImageResultData)> f) {
        std::lock_guard<std::mutex> guard(callback_lock_);
        eventData_ = f;
    }
    void ImageProcessBase::registerStoreEvent(StoreCallback f) {
        std::lock_guard<std::mutex> guard(callback_lock_);
        eventStore_ = f;
    }

    std::string ImageProcessBase::getName() const {
        return name_;
    }
    void ImageProcessBase::setOrigin(cv::Mat m, const int64_t timestamp, bool clone) {
        std::lock_guard<std::mutex> guard(value_lock_);
        timestamp_ = timestamp;
        origin_ = clone ? m.clone() : m;
    }
    cv::Mat ImageProcessBase::getOrigin() {
        std::lock_guard<std::mutex> guard(value_lock_);
        return origin_;
    }

    void ImageProcessBase::setROI(int x, int y, int width, int height) {
        std::lock_guard<std::mutex> guard(value_lock_);
        roi_ = cv::Rect(x, y, width, height);
    }
    cv::Rect ImageProcessBase::getROI() const {
        return roi_;
    }

    void ImageProcessBase::setFreq(int new_frequency) {
        //std::lock_guard<std::mutex> guard(value_lock);
        if (new_frequency <= 0) {
            LOG(WARNING) << "frequency should be larger than 0";
        }
        save_frequency_ = new_frequency;
    }

    void ImageProcessBase::setStore(bool is_store) {
        //std::lock_guard<std::mutex> guard(value_lock);
        isStore_ = is_store;
    }

    void ImageProcessBase::setTimestamp(int64_t t) {
        std::lock_guard<std::mutex> guard(value_lock_);
        timestamp_ = t;
    }

    int64_t ImageProcessBase::getTimestamp() const {
        std::lock_guard<std::mutex> guard(value_lock_);
        return timestamp_;
    }

    bool ImageProcessBase::isDisabled() const {
        //std::lock_guard<std::mutex> guard(value_lock);
        return disabled_;
    }
    void ImageProcessBase::setDisable() {
        //std::lock_guard<std::mutex> guard(value_lock);
        disabled_ = true;
    }
    void ImageProcessBase::setEnable() {
        //std::lock_guard<std::mutex> guard(value_lock);
        disabled_ = false;
    }

    // Necessary for returning bool?
    bool ImageProcessBase::_display(const cv::Mat i, const int view) const {
        std::lock_guard<std::mutex> guard(callback_lock_);
        if (eventDisplay_) {
            cv::Mat rgbMat;
            switch (i.type()) {
            case CV_8UC3:
                //cv::cvtColor(i, rgbMat, CV_BGR2RGB);
                rgbMat = i;
                break;
            case CV_8UC1:
                cv::cvtColor(i, rgbMat, 1);//TODO:code is random to build here
                break;
            default:
                break;
            }
            CV_Assert(rgbMat.isContinuous());
            //cv::Mat behaves like a shared_ptr, dont worry the lifetime.
            eventDisplay_(rgbMat, view);
            return true;
        }
        else
            return false;
    }

    bool ImageProcessBase::_sendData(const common::ImageResultData& data) const {
        std::lock_guard<std::mutex> guard(callback_lock_);
        if (eventData_) eventData_(data);
        return true;
    }
    bool ImageProcessBase::_store(const std::string& name, const cv::Mat& to_store) const {
        std::lock_guard<std::mutex> guard(callback_lock_);
        if (eventStore_) return eventStore_(name, to_store);
        return false;
    }

    void DrawCross(cv::Mat img, cv::Point pt, cv::Scalar color, int lineTk, size_t ratio) {
        int length = img.cols < img.rows ? img.cols : img.rows;
        length = length / ratio;

        cv::Point left, right, up, down;
        left = right = up = down = pt;

        left.x = left.x - length;
        left.y = left.y + length;
        right.x = right.x + length;
        right.y = right.y - length;

        up.x = up.x + length;
        up.y = up.y + length;
        down.x = down.x - length;
        down.y = down.y - length;

        cv::line(img, up, down, color, lineTk);
        cv::line(img, left, right, color, lineTk);
    }
    int PlotHist(const cv::Mat& in, cv::Mat& out) {
        CV_Assert(in.depth() == CV_8U && !in.empty());

        cv::Mat reducedRows;
        // Reduce to one-column mat
        cv::reduce(in, reducedRows, 1, cv::REDUCE_SUM, CV_32SC1);
        cv::Mat normalized;
        // Normalize :
        // normType = INF (max(src) = 1)
        // range(0 - in.cols)
        cv::normalize(reducedRows, normalized, in.cols, 0, cv::NORM_INF);
        double max, min;
        cv::Point min_p, max_p;
        cv::minMaxLoc(reducedRows, &min, &max, &min_p, &max_p);
        auto color_black = cv::Scalar(0);
        cv::Mat plotMat(in.size(), CV_8UC3, color_black);
        for (int i = 0; i < normalized.rows; ++i) {
            auto barSize = normalized.at<int32_t>(i, 0);
            //Point(col, row);
            //Draw red line.
            if (abs(max_p.y - i) < 3) {
                cv::line(plotMat, cv::Point(0, i), cv::Point(barSize, i), cv::Scalar(0, 255, 0));
            }
            else {
                cv::line(plotMat, cv::Point(0, i), cv::Point(barSize, i), cv::Scalar(0, 0, 255));
            }
        }
        out = plotMat;
        return max_p.y;
    }

}

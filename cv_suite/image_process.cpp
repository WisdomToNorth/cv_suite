#include "pch.h"

#include "image_process.h"
#include "image_algorithm.h"
#include "shape.h"
#include "fit.h"
#include "camera.h"
#include <mutex>

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/imgproc.hpp>

float STEGER_SIGMA = 5.0;
constexpr size_t BUF_SIZE = 100;
namespace image {
    ImageProcessSnap::ImageProcessSnap(std::string name, std::string folder) :
        ImageProcessBase(name),
        r_(),
        state_(NO),
        num_(),
        folder_((boost::filesystem::path(folder)).generic_string()),
        state_lock_() {
        if (!boost::filesystem::exists(folder_)) {
            boost::filesystem::create_directory(folder_);
        }
    }
    void ImageProcessSnap::setRobotInfo(common::RobotInfo data) {
        r_ = data;
    }
    void ImageProcessSnap::process() {
    }
    void ImageProcessSnap::store() const {
        std::lock_guard<std::mutex> l(state_lock_);
        if (lastIsBad_) return;
        char buf[BUF_SIZE];
        const char* filename = "Error%04d";
        switch (state_) {
        case State::NO:
            return;
        case State::SINGLE:
            LOG(INFO) << "[ImageProcessSnap::store] Store as Image";
            filename = "Image%04d";
            break;
        case State::SINGLE_LASER:
            LOG(INFO) << "[ImageProcessSnap::store] Store as Laser and set laser image to display";
            filename = "Laser%04d";
            laser_mat_ = origin_.clone();
            break;
        default:
            break;
        }
        sprintf_s(buf, BUF_SIZE, filename, num_);
        _store((boost::filesystem::path(folder_) / buf).generic_string()
            + IMAGE_SUFFIX,
            origin_);
        common::ImageResultData d;

        d.state = state_;
        //LOG(INFO) << "Send Data.";
        _sendData(d);
        state_ = State::NO;
    }

    void ImageProcessSnap::setState(State state, int num) {
        std::lock_guard<std::mutex> l(state_lock_);
        state_ = state;
        num_ = num;
    }
    ImageProcessSnap::State ImageProcessSnap::getState() {
        std::lock_guard<std::mutex> l(state_lock_);
        return state_;
    }
    void ImageProcessSnap::output() {
        std::lock_guard<std::mutex> l(state_lock_);
        if (lastIsBad_) return;
        cv::Mat hist_mat;
        auto max = PlotHist(origin_, hist_mat);
        common::ImageResultData height_data;
        height_data.height = max;
        //global::weld_data.image_result_buffer.push_back(std::make_pair(static_cast<common::APPImageID>(originalFrame_), height_data));

        _display(origin_, Original);
        _display(laser_mat_, Hist);
    }
    ImageProcessTest::ImageProcessTest(std::string name)
        : ImageProcessBase(name)
        , ptCenter_()
    {}

    void ImageProcessTest::process() {
        cv::Mat roi_image = origin_(roi_);
        if (roi_image.cols == 0 || roi_image.rows == 0) {
            lastIsBad_ = true;
            return;
        }
        //std::cout << roi_image.size() << std::endl;
        cv::Mat roi_to_display = roi_image.clone();
        image::StergerLineDetector steger;
        steger.Compute(roi_image, STEGER_SIGMA);
        /*
        if (funcFitting_) {
            auto coefs = funcFitting_(steger.GetSubPixelVec());
            cv::Mat point_flag_mat_bgr;
            cv::cvtColor(steger.GetFlagMat(), point_flag_mat_bgr, cv::COLOR_GRAY2BGR);
            for(const auto &coef:coefs) {
                //coef: point, direction.
                //TODO DEBUG: coef may be none
                if (coef.size() > 2) {
                    DrawCross(point_flag_mat_bgr, cv::Point(int(coef(1)), int(coef(0))));
                }
            }
            if (coefs.size() == 2) {
                // Corner center. may be null
                auto intersect = shape::IntersectOfTwoLines(coefs[0], coefs[1]);
                if (intersect.size() >= 2) {
                    auto intersect_cv = cv::Point(int(intersect(1)), int(intersect(0)));
                    DrawCross(point_flag_mat_bgr, cv::Point(int(intersect(1)), int(intersect(0))), cv::Scalar(0, 255, 0), 10);
                    for (const auto &coef : coefs) {
                        if (coef.size() > 2) {
                            cv::line(point_flag_mat_bgr, intersect_cv, cv::Point(int(coef(1)), int(coef(0))), cv::Scalar(0,0 , 255));
                        }
                    }
                    if (funcCorner_) {
                        funcCorner_(intersect(1), intersect(0));
                    }
                }
            }
            _display(point_flag_mat_bgr, Point);
        }
        _display(roi_to_display, Process);
        std::vector<std::pair<double, double>> profile_data;
        _plot(profile_data, 1, 2);
        //from output function
        cv::Mat hist_mat;
        auto max = PlotHist(origin_, hist_mat);
        //common::ImageResultData height_data = shape::Line2Feature(steger.GetSubPixelMat());
        //height_data.height = max;
        //_sendData(height_data);
        _display(origin_, Original);
        _display(hist_mat, Hist);
        */
        lastIsBad_ = false;
    }

    void ImageProcessTest::output() {
        //
        //cv::line(imageInProcess_, pt_[1], pt_[3], cv::Scalar(255), 1);
        //cv::line(imageInProcess_, pt_[2], pt_[3], cv::Scalar(255), 1);
        //_display(imageInProcess_, 1);


    }

    void ImageProcessTest::_FindBottomLine() {
        int threshold = 122;
        int dilateXNum = 1;
        int erodeXNum = 1;
        //int erodeYNum = 1;
        cv::threshold(origin_,
            imageInProcess_,
            threshold,
            255,
            cv::THRESH_BINARY);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 1));
        for (int i = 0; i < dilateXNum; ++i) {
            cv::dilate(imageInProcess_, imageInProcess_, element, cv::Point(-1, -1));
        }
        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 1));
        for (int i = 0; i < erodeXNum; ++i) {
            cv::erode(imageInProcess_, imageInProcess_, element, cv::Point(-1, -1));
        }
        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 3));
        for (int i = 0; i < erodeXNum; ++i) {
            cv::erode(imageInProcess_, imageInProcess_, element, cv::Point(-1, -1));
        }
        cv::Mat matTmp = imageInProcess_.clone();
        int ImgHeight = imageInProcess_.rows;
        int ImgWidth = imageInProcess_.cols;
        cv::Point2i	ptLowest(0, 0);
        for (int i = 0; i < ImgHeight; ++i) {
            for (int j = 0; j < ImgWidth; ++j) {
                if (matTmp.at<uchar>(i, j) == 255) {
                    if (ptLowest.y < i) {
                        ptLowest.x = j;
                        ptLowest.y = i;
                    }
                }
            }
        }
        pt_[3].x = ptLowest.x + roi_.x;
        pt_[3].y = ptLowest.y + roi_.y;
        _findLine();
    }

    void ImageProcessTest::_findLine() {
        if (imageInProcess_.empty()) return;
        cv::Mat matTmp = imageInProcess_;

        cv::Point2i pts1(0, 0), pts2(0, 0);
        int index = 0, count = 0, Max = 0;
        int height = matTmp.rows, width = matTmp.cols;

        for (int i = 0; i < height; i++) {
            count = 0;
            for (int j = 0; j < width; j++) {
                if (matTmp.at<uchar>(i, j) == 255) {
                    ++count;
                }
            }
            if (count > Max) {
                Max = count;
                index = i;
            }
        }
        for (int i = 0; i < height; i++) {
            if (i != index)
                for (int j = 0; j < width; j++)
                    matTmp.at<uchar>(i, j) = 0;
        }
        int gap1 = 0, gap2 = 0;
        for (int i = 1; i < width - 1; ++i) {
            if (matTmp.at<uchar>(index, i - 1) == 255
                && matTmp.at<uchar>(index, i) == 255
                && matTmp.at<uchar>(index, i + 1) == 0) {
                gap1 = abs(pts1.x - width / 2);
                if (gap1 > (abs(i - width / 2))) pts1.x = i;
            }
            if (matTmp.at<uchar>(index, i - 1) == 0
                && matTmp.at<uchar>(index, i) == 255
                && matTmp.at<uchar>(index, i + 1) == 255) {
                gap2 = abs(pts2.x - width / 2);
                if (gap2 > (abs(i - width / 2))) pts2.x = i;
            }
        }
        pts1.y = pts2.y = index;
        pt_[1].x = pts1.x + roi_.x;
        pt_[1].y = pts1.y + roi_.y;
        pt_[2].x = pts2.x + roi_.x;
        pt_[2].y = pts2.y + roi_.y;
        pt_[0].x = (pt_[1].x + pt_[2].x) / 2;
        pt_[0].y = (pt_[1].y + pt_[2].y) / 2;
    }

    double ImageProcessTest::_ImgLaserHeight(int line_height) const {
        double A = 1.1;
        double B = 1.2;
        double C = 1.1;
        return A / (B * line_height) + C;
    }

    // [in] : gray scale threshold.
    int ImageProcessTest::_ImgLineHeight(const cv::Mat& laser_mat, const uchar gray_threshold = 180) const {
        if (laser_mat.empty() || laser_mat.type() != CV_8UC1) return 0;
        int ImgHeight = laser_mat.rows;
        int ImgWidth = laser_mat.cols;
        int ptSUM = 0, ptCount = 0, xIndex = 0;
        for (int i = 0; i < ImgHeight; ++i) {
            ptCount = 0;
            for (int j = 0; j < ImgWidth; ++j) {
                if (laser_mat.at<uchar>(i, j) >= gray_threshold) {
                    ++ptCount;
                }
            }
            if (ptCount > ptSUM) {
                ptSUM = ptCount;
                xIndex = i;
            }
        }
        return ImgHeight - xIndex;
        //return ImgHeight -xIndex - gROI_Y;
    }
    //Input: 1 rows * n cols mat CV_XXC1(<32S).
    double ImageProcessTest::_Correlation1D(const cv::Mat& input1, const cv::Mat& input2) {
        CV_Assert(!input1.empty() &&
            !input2.empty() &&
            input1.rows == 1 &&
            input2.rows == 1 &&
            input1.cols == input2.cols &&
            input1.channels() == input2.channels() &&
            input1.type() == input2.type() && input2.depth() <= CV_32S);

        int cn = input1.channels();
        cv::Mat row_mean1, row_mean2;
        cv::reduce(input1, row_mean1, 1, cv::REDUCE_AVG, CV_MAKE_TYPE(CV_64F, cn));
        cv::reduce(input2, row_mean2, 1, cv::REDUCE_AVG, CV_MAKE_TYPE(CV_64F, cn));
        int mean1 = static_cast<int>(row_mean1.at<double>(0, 0));
        int mean2 = static_cast<int>(row_mean2.at<double>(0, 0));

        cv::Mat input1_mean = input1 - mean1;
        cv::Mat input2_mean = input2 - mean2;

        double top_sum = input1_mean.dot(input2_mean);
        double bottom_sum = input1_mean.dot(input1_mean) * input2_mean.dot(input2_mean);
        double bottom_root = std::sqrt(bottom_sum);

        return top_sum / bottom_root;
    }

    std::vector<cv::Point2i> ImageProcessTest::VJointFeature(const cv::Mat& in, cv::Mat& out) {
        origin_ = in.clone();

        _FindBottomLine();

        std::vector<cv::Point2i> feature_points(pt_, pt_ + 4);
        cv::cvtColor(imageInProcess_, out, cv::COLOR_GRAY2BGR);
        cv::line(out, pt_[1], pt_[3], cv::Scalar(255), 1);
        cv::line(out, pt_[2], pt_[3], cv::Scalar(255), 1);

        for (auto& pt : pt_) {
            DrawCross(out, pt);
        }
        return feature_points;
    }
}
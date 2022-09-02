#ifndef CVSUITE_SOURCE_DRIVER_IMAGE_PROCESS_H
#define CVSUITE_SOURCE_DRIVER_IMAGE_PROCESS_H
#include "image.h"

namespace image {
    //
    class ImageProcessSnap : public ImageProcessBase {
    public:
        // callback will do different task at different state.
        enum State {
            SINGLE,
            SINGLE_LASER,
            NO
        };
        ImageProcessSnap(std::string name, std::string folder);
        void setRobotInfo(common::RobotInfo);
        virtual void process() override;
        virtual void output() override;
        virtual void store() const override;
        //ThreadSafe
        void setState(State, int);
        State getState();
    private:
        common::RobotInfo r_;
        mutable State state_;
        int num_;
        std::string folder_;
        std::mutex mutable state_lock_;
        mutable cv::Mat laser_mat_;
    };

    class ImageProcessTest : public ImageProcessBase {
    public:
        ImageProcessTest(std::string name);
        // Override methods.
        virtual void  process() override;
        virtual void  output() override;

    private:
        cv::Point2i ptCenter_;
        cv::Point2i pt_[6];
        // counter may incre/decre in const methods. so set to mutable.
        void _findLine();
        void _FindBottomLine();
        // From WeldSystem by Huang Seji
        int _ImgLineHeight(const cv::Mat&, const uchar gray_threshold) const;
        double _ImgLaserHeight(int line_height) const;
        // Correlation between two 1d mat.
        double _Correlation1D(const cv::Mat& input1, const cv::Mat& input2);
        std::vector<cv::Point2i> VJointFeature(const cv::Mat& in, cv::Mat& out);
        // Steger should not be member variable, since it should reset before
        //  every computing, local variable is prefered;
        //image::StergerLineDetector steger;

    };



}
#endif
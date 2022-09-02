#pragma once
#include "image.h"
#include "image_algorithm.h"
#include "shape.h"
namespace image {
    /*Image process based on Steger Algorithm.

    ImageProcessBase ----> StegerProcess ----> StegerProcessT, StegerProcessV....

    Use MakeImageProcessSteger factory function to construct
    processor object by weld profile type.
    */
    class StegerProcess : public ImageProcessBase {
    public:
        using PointsCallback = std::function<void(const std::vector<shape::OutputVec>&, const int64_t timestamp)>;
        using PointCloudCallback = std::function<void(const common::VecLinePts&, const int64_t timestamp)>;
        StegerProcess(std::string, double sigma);

        void store() const override;
        void  process() override;
        void output() override;

        void registerPointsCallback(const PointsCallback&);
        void registerPointCloudCallback(const PointCloudCallback&);
        void setProfileModel(shape::ProfileModelPtr);
        void setStegerSigma(double sigma);

        shape::ProfileModelPtr getProfileModel() const;
        enum DisplayID {
            kNone,
            kOrigin,
            kROI,
            kROI2,
            kProfile,
        };
    protected:

        mutable std::mutex callback_lock_;
        PointsCallback callback_on_points_;
        PointCloudCallback callback_on_pointcloud_;
        std::unique_ptr<image::StergerLineDetector> steger_;
        double steger_sigma_;
        shape::ProfileModelPtr profile_model_;
        common::WeldType type_;
        //mutable std::mutex steger_lock_;
        common::VecLinePts lastPoints_;
        cv::Rect lastROI_;
    };
    using StegerProcessPtr = std::shared_ptr<StegerProcess>;
    using StegerProcessWeakPtr = std::weak_ptr<StegerProcess>;
    StegerProcessPtr MakeImageProcessSteger(common::WeldType, std::string, std::string, double sigma = 4.0);
    class StegerProcessV : public StegerProcess {
    public:
        StegerProcessV(std::string, double);
        void  process() override;
        void  output() override;
    };
    class StegerProcessT : public StegerProcess {
    public:
        StegerProcessT(std::string, double);
        void  process() override;
        void  output() override;
    };
    class StegerProcessButt : public StegerProcess {
    public:
        StegerProcessButt(std::string, double);
        void  process() override;
        void  output() override;
    };
}
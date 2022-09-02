#pragma once
#include "image_algorithm.h"
#include "core.h"
//#include "binary.pb.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#include <boost/shared_ptr.hpp>
#include <exception>
#include <array>
#include <vector>
#include <limits>
#include <type_traits>
#include <memory>

namespace shape {
    using std::vector;
    using Vec2f = std::array<float, 2>;
    using Vec3f = std::array<float, 3>;
    using Vec4f = std::array<float, 4>;
    using VecXf = std::vector<float>;
    using Vec2d = std::array<double, 2>;
    using Vec3d = std::array<double, 3>;
    using Vec4d = std::array<double, 4>;
    using VecXd = std::vector<double>;
    //bool for invalid or valid, invalid VecXf may be empty.
    using OutputVec = std::tuple<bool, shape::VecXf>;
    using PointCloud3D = pcl::PointCloud<pcl::PointXYZ>;
    using Point3D = PointCloud3D::PointType;


    struct ProfileModel {
    protected:
        common::WeldType type_;
    public:
        ProfileModel() = default;
        virtual ~ProfileModel() = default;
        virtual std::vector<PointCloud3D::Ptr> SelectPoints(const common::VecLinePts& pts) = 0;
        virtual std::vector<std::vector<int>> SelectPointsIndex(const common::VecLinePts& pts) = 0;
        common::WeldType GetWeldType() const;

        virtual size_t GeModelSize() const = 0;
        virtual std::tuple<size_t, size_t, float, float> GetModelNo(size_t) const = 0;
        virtual std::string ToString() const = 0;
        virtual bool IsValid() const = 0;
    };

    using ProfileModelPtr = std::shared_ptr<ProfileModel>;
    ProfileModelPtr MakeProfileModel(common::WeldType type, const std::string& config_str);

    struct TransformerImpl;
    // Currently we only transform to camera frame.
    class Transformer {
        std::unique_ptr<TransformerImpl> d;
    public:
        //KKK//Transformer(binary::DoubleMatrix);
        Transformer();
        ~Transformer();
        Transformer(const Transformer&);
        Transformer& operator=(const Transformer& r);

        //KKK//bool SetConversion(binary::DoubleMatrix);
        // How to get the input(a 4*4 matrix): add tool frame translation to handeye
        //KKK//bool SetTransform(binary::DoubleMatrix);

        Vec3d ToCameraFrame(const Vec2d& point) const;
        Vec3f ToCameraFrame(const Vec2f& point) const;
        // Inpput should be homogeneous form, use matrix.colwise().homogeneous() or vec.homogeneous() before passing.
        Eigen::Matrix3Xd ToCameraFrameEigen(const Eigen::Matrix3Xd&) const;
        Eigen::Matrix3Xf ToCameraFrameEigen(const Eigen::Matrix3Xf&) const;

        Vec3d ToWorldFrame(const Vec2d& point) const;
        Vec3f ToWorldFrame(const Vec2f& point) const;
        // Inpput should be homogeneous form, use matrix.colwise().homogeneous() or vec.homogeneous() before passing.
        Eigen::Matrix3Xd ToWorldFrameEigen(const Eigen::Matrix3Xd&) const;
        Eigen::Matrix3Xf ToWorldFrameEigen(const Eigen::Matrix3Xf&) const;

        std::string GetConversionStringRepr() const;

    };
    using TransformerPtr = std::unique_ptr<Transformer>;

    //TransformerPtr MakeTransformer(binary::DoubleMatrix);
    Eigen::Matrix2Xf LinePts2Matrix(const common::VecLinePts& pts, const size_t sample_n = 1);
    Eigen::Matrix3Xf LinePts2MatrixInHomogeneous(const common::VecLinePts& pts, const size_t sample_n = 1);
    Eigen::Matrix3Xf LinePts2MatrixInHomogeneousFilteredByAngle(const common::VecLinePts& pts, std::vector<std::pair<float, float>>acceptableRanges);
    Eigen::Quaternionf WPR2AngleAxis(float w, float p, float r);
}
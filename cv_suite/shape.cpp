#include "pch.h"

#include "shape.h"
#include <string>
#include <sstream>
#include <vector>
#include <iterator>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <boost/math/constants/constants.hpp>
#pragma warning(disable : 4996)
namespace shape {
    using T3d = Eigen::Transform<double, 3, Eigen::Affine>;
    using T3f = Eigen::Transform<float, 3, Eigen::Affine>;
    using ConversionMatrixd = Eigen::Matrix<double, 4, 3>;
    using ConversionMatrixf = Eigen::Matrix<float, 4, 3>;
    using HomogeneousMatrixd = Eigen::Matrix<double, 4, 4>;
    using HomogeneousMatrixf = Eigen::Matrix<float, 4, 4>;
    using ImagePoints = Eigen::Matrix3Xd;


    template<typename Scalar>
    struct Range {
        Scalar lower;
        Scalar upper;
    };

    // This may be used in steger class internal? line detection and filter in one pass.
    template<typename Float, size_t LineNum, bool UseYForPoint = true>
    struct AngleROI {
        // Index is type used for range array idex and pont vector index.
        // use invalid_idx for 'false' result.
        using Index = decltype(LineNum);
        using Ranges = std::array<Range<Index>, LineNum>;
        using Angles = std::array<Range<Float>, LineNum>;

        static constexpr Float angle_max = (std::numeric_limits<Float>::max)();
        static constexpr Float angle_min = (std::numeric_limits<Float>::min)();
        static constexpr Index invalid_idx = (std::numeric_limits<Index>::max)();
        static constexpr size_t line_num = LineNum;

        Ranges ranges;
        Angles angles;

        AngleROI() {}
        AngleROI(const AngleROI&) = default;
        AngleROI& operator=(const AngleROI&) = default;
        std::string to_string() const {
            std::string ret;
            for (auto i = 0; i < LineNum; ++i) {
                std::string r1 = std::to_string(ranges[i].lower), r2 = std::to_string(ranges[i].upper);
                auto a1_val = angles[i].lower, a2_val = angles[i].upper;
                std::string a1, a2;
                if (a1_val == angle_min) {
                    a1 = std::string("min");
                }
                else {
                    a1 = std::to_string(a1_val);
                }
                if (a2_val == angle_max) {
                    a2 = std::string("max");
                }
                else {
                    a2 = std::to_string(a2_val);
                }

                ret += r1 + "," + r2 + "," + a1 + "," + a2 + ";";
            }
            return ret;
        }

        template<typename VecPoints>
        std::vector<PointCloud3D::Ptr>
            select_pts(const VecPoints& pts) const {
            std::vector<PointCloud3D::Ptr> groups;
            for (auto i = 0; i < LineNum; ++i) {
                groups.emplace_back(new PointCloud3D());
            }
            // Not efficient enough
            // if input pts are ordered, it wil be not necessary to compare range.
            // This assumption of order should be guarenteed in steger algo class.
            for (const auto& pt : pts) {
                auto idx = which_range(pt);
                if (idx == invalid_idx) continue;

                auto angle_range = angles[idx];
                if (pt.angle <= angle_range.upper && pt.angle >= angle_range.lower) {
                    groups[idx]->push_back(Point3D(pt.x, pt.y, 0.f));
                }
            }
            return groups;
        }

        template<typename VecPoints>
        std::vector<std::vector<int>>
            select_pts_index(const VecPoints& pts) const {
            std::vector<std::vector<int>> groups(LineNum);
            // Not efficient enough
            // if input pts are ordered, it wil be not necessary to compare range.
            // This assumption on order should be guarenteed in steger algo class.
            for (auto i = 0; i < pts.size(); ++i) {
                const auto& pt = pts[i];
                auto idx = which_range(pt);
                if (idx == invalid_idx) continue;

                auto angle_range = angles[idx];
                if (pt.angle <= angle_range.upper && pt.angle >= angle_range.lower) {
                    groups[idx].push_back(i);
                }
            }
            return groups;
        }

        template<typename Point, typename std::enable_if<true, int>::type = 0>
        Index which_range(Point p) const {
            return _range_compare_loop(p.x);
        }


        template<Index loop = 0>
        Index _range_compare_loop(Index val) const {
            return (val <= ranges[loop].upper && val >= ranges[loop].lower) ? loop : _range_compare_loop<loop + 1>(val);
        }

        template<>
        Index _range_compare_loop<LineNum>(Index val) const { return invalid_idx; }

        std::vector<Eigen::VectorXf> fitted_points(const common::VecLinePts& line_pts) {
            std::vector<Eigen::VectorXf> coefs_of_models;
            //TODO:delete all code here
            return coefs_of_models;
        }
    };

    using VModel = AngleROI<float, 4, true>;
    using CornerModel = AngleROI<float, 2, true>;
    template<typename Out>
    void Split(const std::string& s, char delim, Out result) {
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            *(result++) = item;
        }
    }

    std::vector<std::string> Split(const std::string& s, char delim) {
        std::vector<std::string> elems;
        Split(s, delim, std::back_inserter(elems));
        return elems;
    }
    template<typename Model>
    bool _AddModel(Model& model, const std::string& input_str) {
        auto models_as_string = Split(input_str, ';');
        if (models_as_string.size() != model.line_num) {
            LOG(INFO) << "[_ADDModel] Wrong splited size:" << models_as_string.size();
            return false;
        }
        for (size_t i = 0; i < models_as_string.size(); ++i) {
            auto list_of_nums = Split(models_as_string[i], ',');
            if (list_of_nums.size() != 4) {
                LOG(ERROR) << "[_AddModel][Error]: model should be like: 200, 300, 20.5, 30.5; ...";
                return false;
            }
            auto r1 = static_cast<size_t>(std::stoi(list_of_nums[0]));
            auto r2 = static_cast<size_t>(std::stoi(list_of_nums[1]));
            if (r2 < r1) {
                LOG(ERROR) << "[_AddModel][Error]: r2 < r1";
                return false;
            }
            auto a1 = std::stof(list_of_nums[2]);
            auto a2 = std::stof(list_of_nums[3]);

            model.angles[i] = { a1 < 0.f ? VModel::angle_min : a1, a2 < 0.f ? VModel::angle_max : a2 };
            model.ranges[i] = { r1, r2 };
        }
        return true;
    }

    common::WeldType ProfileModel::GetWeldType() const {
        return type_;
    }

    template<typename Model>
    struct ProfileModelByType : public ProfileModel {
        explicit ProfileModelByType(const std::string& config) :model_(), isValid_(false) {
            isValid_ = _AddModel(model_, config);
        }
        ProfileModelByType() : model_() {}
        Model model_;
        bool isValid_;
        std::vector<PointCloud3D::Ptr>
            SelectPoints(const common::VecLinePts& pts) override {
            //TODO
            return model_.select_pts(pts);
        }
        std::vector<std::vector<int>>
            SelectPointsIndex(const common::VecLinePts& pts) override {
            return model_.select_pts_index(pts);
        }
        size_t GeModelSize() const override {
            return Model::line_num;
        }
        std::tuple<size_t, size_t, float, float> GetModelNo(size_t n) const override {
            if (n >= Model::line_num) {
                return std::tuple<size_t, size_t, float, float>{};
            }
            return std::tuple<size_t, size_t, float, float>{
                model_.ranges[n].lower, model_.ranges[n].upper,
                    model_.angles[n].lower, model_.angles[n].upper
            };
        }

        std::string ToString() const override {
            return model_.to_string();
        }
        bool IsValid() const override {
            return isValid_;
        }
    };

    using ProfileModelV = ProfileModelByType<VModel>;
    using ProfileModelT = ProfileModelByType<CornerModel>;

    ProfileModelPtr MakeProfileModel(common::WeldType type, const std::string& config_str) {
        auto res = ProfileModelPtr();
        switch (type) {
        case common::WeldType::V:
            res = std::make_shared<ProfileModelV>(config_str);
            break;
        case common::WeldType::BUTT:
            res = std::make_shared<ProfileModelT>(config_str);
            break;
        case common::WeldType::CORNER:
            res = std::make_shared<ProfileModelT>(config_str);
            break;
        case common::WeldType::UNKNOWN:
        default:
            LOG(ERROR) << "[MakeProfileModel]: Unknown weld type";
        }
        return res;
    }
   
    Eigen::Matrix2Xf LinePts2Matrix(const common::VecLinePts& pts, const size_t sample_n) {
        if (sample_n == 0) {
            LOG(WARNING) << "LinePts2Matrix: sample at every 0 point? Impossible.";
            return Eigen::Matrix2Xf{};
        }
        const auto sz = pts.size() / sample_n;
        if (sz == 0) return Eigen::Matrix2Xf();
        Eigen::Matrix2Xf res(2, sz);
        for (size_t i = 0; i < sz; ++i) {
            const auto idx = i * sample_n;
            res(0, i) = pts[idx].x; res(1, i) = pts[idx].y;
        }
        return res;
    }
    Eigen::Matrix3Xf LinePts2MatrixInHomogeneous(const common::VecLinePts& pts, const size_t sample_n) {
        if (sample_n == 0) {
            LOG(WARNING) << "LinePts2Matrix: sample at every 0 point? Impossible.";
            return Eigen::Matrix3Xf{};
        }
        const auto sz = pts.size() / sample_n;
        if (sz == 0) return Eigen::Matrix3Xf();
        Eigen::Matrix3Xf res(3, sz);
        for (size_t i = 0; i < sz; ++i) {
            const auto idx = i * sample_n;
            res(0, i) = pts[idx].x; res(1, i) = pts[idx].y; res(2, i) = 1.f;
        }
        return res;
    }
    Eigen::Matrix3Xf LinePts2MatrixInHomogeneousFilteredByAngle(const common::VecLinePts& pts, std::vector<std::pair<float, float>>acceptableRanges) {
        common::VecLinePts filteredPts;
        for (const auto& point : pts) {
            for (const auto& range : acceptableRanges) {
                if (point.angle > range.first && point.angle < range.second) {
                    filteredPts.emplace_back(point);
                    break;
                }
            }
        }
        const auto sz = filteredPts.size();
        if (sz == 0) return Eigen::Matrix3Xf();
        Eigen::Matrix3Xf res(3, sz);
        for (size_t i = 0; i < filteredPts.size(); ++i) {
            res(0, i) = filteredPts[i].x; res(1, i) = filteredPts[i].y; res(2, i) = 1.f;
        }
        return res;
    }
    Eigen::Quaternionf WPR2AngleAxis(float w, float p, float r) {
        constexpr auto PI = boost::math::constants::pi<float>();
        return Eigen::AngleAxisf(r / 180.f * PI, Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(p / 180.f * PI, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(w / 180.f * PI, Eigen::Vector3f::UnitX());
    }

    struct TransformerImpl {
        ConversionMatrixd cm_d_;
        ConversionMatrixf cm_f_;
        HomogeneousMatrixd hm_d_;
        HomogeneousMatrixf hm_f_;
        ConversionMatrixd hcm_d_;
        ConversionMatrixf hcm_f_;
        TransformerImpl() :cm_d_(ConversionMatrixd::Ones()), cm_f_(ConversionMatrixf::Ones()),
            hm_d_(HomogeneousMatrixd::Ones()), hm_f_(HomogeneousMatrixf::Ones()),
            hcm_d_(ConversionMatrixd::Ones()), hcm_f_(ConversionMatrixf::Ones())
        {}
        ~TransformerImpl() = default;
        TransformerImpl(const TransformerImpl& r) = default;
        TransformerImpl& operator=(const TransformerImpl& r) = default;

        // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    std::string Transformer::GetConversionStringRepr() const {
        std::stringstream ss;
        ss << d->cm_d_;
        return ss.str();
    }
    
    Vec3f Transformer::ToCameraFrame(const Vec2f& point) const {
        Eigen::Vector3f pt_2f_homo{ point[0], point[1], 1.f };
        Eigen::Vector3f pt_3d = (d->cm_f_ * pt_2f_homo).hnormalized();
        return Vec3f{ pt_3d(0), pt_3d(1), pt_3d(2) };
    }
    Vec3d Transformer::ToCameraFrame(const Vec2d& point) const {
        Eigen::Vector3d pt_2d_homo{ point[0], point[1], 1. };
        Eigen::Vector3d pt_3d = (d->cm_d_ * pt_2d_homo).hnormalized();
        return Vec3d{ pt_3d(0), pt_3d(1), pt_3d(2) };
    }
    Eigen::Matrix3Xd Transformer::ToCameraFrameEigen(const Eigen::Matrix3Xd& homo_points) const {
        return (d->cm_d_ * homo_points).colwise().hnormalized();
    }
    Eigen::Matrix3Xf Transformer::ToCameraFrameEigen(const Eigen::Matrix3Xf& homo_points) const
    {
        return (d->cm_f_ * homo_points).colwise().hnormalized();
    }

    Vec3d Transformer::ToWorldFrame(const Vec2d& point) const
    {
        Eigen::Vector3d pt_2d_homo{ point[0], point[1], 1. };
        Eigen::Vector3d pt_3d = (d->hcm_d_ * pt_2d_homo).hnormalized();
        return Vec3d{ pt_3d(0), pt_3d(1), pt_3d(2) };
    }
    Vec3f Transformer::ToWorldFrame(const Vec2f& point) const
    {
        Eigen::Vector3f pt_2f_homo{ point[0], point[1], 1.f };
        Eigen::Vector3f pt_3d = (d->hcm_f_ * pt_2f_homo).hnormalized();
        return Vec3f{ pt_3d(0), pt_3d(1), pt_3d(2) };
    }
    Eigen::Matrix3Xd Transformer::ToWorldFrameEigen(const Eigen::Matrix3Xd& homo_points) const {
        return (d->hcm_d_ * homo_points).colwise().hnormalized();
    }
    Eigen::Matrix3Xf Transformer::ToWorldFrameEigen(const Eigen::Matrix3Xf& homo_points) const {
        return (d->hcm_f_ * homo_points).colwise().hnormalized();
    }


}